// ================================================================================================
//  Residency - M6e: ONE residency manager for every tiled tenant, descended from the classic
//  D3D11 TiledResources sample's ResidencyManager (studied from the user's copy of the extinct
//  original) and extended with what this project already proved out.
//
//  Two CLASSES of tenant live here, and the distinction is the design:
//
//  * FIELDS (churn, SWE eta/flux, wind Mv2, cloud volume): a NULL tile MEANS "this quantity is
//    identically zero here" -- the Tier-2 read-zero guarantee is the SEMANTICS. Shaders sample
//    unconditionally; residency is DECIDED by physics policies (jet envelopes, wet masks, cloud
//    fraction), not discovered from sampling. These tenants keep their validated policies and
//    register here for the shared pool, stats, and the grade-signature registry.
//
//  * TEXTURES (Mars's BC1/BC5 pyramids, Google's Earth tiles): a NULL tile means ABSENCE, not
//    zero -- so each texture tenant carries a shader-visible RESIDENCY MAP (R8 cube, byte =
//    finest-resident mip * 16, the classic sample's exact scheme) and samplers CLAMP their LOD
//    to what is resident: misses degrade to blur, never to garbage. The classic's ordering
//    invariant is enforced: a finer tile is never resident where its parent is NULL (loads run
//    coarse-to-fine per uv, evictions fine-to-coarse).
//
//  The GA layer ("keeping the models in harmony"):
//  * Grade signatures: per-tile signature bits (one per grade) for registered fields, with the
//    PROVEN Cayley closure (Cl2ProductSignature, selftest-tight) propagating residency through
//    products: a derived field's tiles need to exist exactly where its inputs' product
//    signature is non-zero -- decided by algebra, without reading data. Air (wind curl) and
//    water (SWE flux) tenants coordinate through the same registry.
//  * Motor prefetch: the camera flies on Pga.h motors; the screw log of the last frame's pose
//    delta EXTRAPOLATES the view a beat ahead, and predicted-view tiles enter the seen list
//    early. The classic sample reacts to last frame's misses; we anticipate along the screw.
//
//  Frame flow: tenants Enqueue() wanted tiles (CDLOD node walks for textures -- deterministic,
//  no sampling-feedback pass needed; policies for fields) -> ProcessQueues() sorts seen /
//  loading / mapped lists, starts provider loads (worker threads; HTTP tenants throttle and
//  cache), maps+fills a budgeted batch (queue-side UpdateTileMappings + CopyTiles
//  LINEAR_BUFFER_TO_SWIZZLED from a fenced upload ring), evicts LRU (only tiles unseen longer
//  than the frame-overlap window, so in-flight GPU reads stay valid), and refreshes residency
//  maps. PIX sees one marked span per phase.
// ================================================================================================
#pragma once

#include "core/Gpu.h"
#include "core/Pga.h"

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <functional>
#include <map>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace ga {

// A provider fills one 64KB tile's worth of LINEAR data for a texture tenant. Runs on a worker
// thread; must be self-contained and cache-first (HTTP providers throttle themselves and honor
// a hard per-run fetch budget). Returns false if the tile cannot be produced (kept NULL).
struct TileRequest {
    uint32_t face = 0, mip = 0, x = 0, y = 0;
};
using TileProviderFn = std::function<bool(const TileRequest&, std::vector<uint8_t>& out64k)>;

class ResidencyManager {
public:
    static constexpr uint32_t kPoolChunkTiles = 128;     // 8 MB heap chunks
    // M7w: 12 in-flight loads on 2 workers drained ~0.5 tiles/frame -- a rail descent wants
    // thousands, so most of the view rode coarse fallbacks for the whole flight (the vintage
    // patchwork). Loads are disk/CPU paints; feed as many workers as the machine has.
    static constexpr uint32_t kMaxLoadsInFlight = 48;
    static constexpr uint32_t kMaxMapsPerFrame = 96;     // tiles mapped+filled per frame
    static constexpr uint32_t kPoolCapTiles = 8192;      // 512 MB ceiling before eviction
    static constexpr uint32_t kEvictAgeFrames = 4;       // > frame overlap: no in-flight reads

    void Init(Gpu& gpu);
    void Shutdown();

    // ---- texture tenants (Mars, Earth) ------------------------------------------------------
    // Creates the reserved cube (ArraySize 6, full mip chain), its SRV, the R8 residency-map
    // cube + SRV, and registers the provider. Packed mips are loaded and mapped up front (the
    // planet is never bald). Returns the tenant id.
    int AddTextureCube(Gpu& gpu, const wchar_t* name, uint32_t faceDim, DXGI_FORMAT fmt,
                       TileProviderFn provider) {
        return AddTextureInternal(gpu, name, faceDim, fmt, std::move(provider), 6);
    }
    // M6f: a single-face detail WINDOW (the Merrimack imagery pyramid) -- same machinery,
    // arraySize 1, callers pass face 0.
    int AddTexture2D(Gpu& gpu, const wchar_t* name, uint32_t dim, DXGI_FORMAT fmt,
                     TileProviderFn provider) {
        return AddTextureInternal(gpu, name, dim, fmt, std::move(provider), 1);
    }

    // Tiles known but not yet mapped (seen + loading + in flight). The warm-cache loop drains
    // this to zero before the camera ever moves.
    uint32_t PendingCount() const {
        return static_cast<uint32_t>(m_seen.size() + m_loading.size());
    }

    uint32_t TextureSrv(int tenant) const { return m_tenants[tenant].srv; }
    // M7l: the hypervisor asks what is ACTUALLY resident at a uv -- the same CPU-side map
    // the GPU residency clamp samples (byte = finest resident mip * 16).
    uint32_t ResidentMipAt(int tenant, uint32_t face, float u, float v) const {
        const Tenant& t = m_tenants[tenant];
        if (face >= t.faces || t.resCpu[face].empty() || t.resMap.width == 0) return 255u;
        const uint32_t rdim = t.resMap.width;
        uint32_t x = static_cast<uint32_t>(u * rdim);
        uint32_t y = static_cast<uint32_t>(v * rdim);
        if (x >= rdim) x = rdim - 1;
        if (y >= rdim) y = rdim - 1;
        return t.resCpu[face][static_cast<size_t>(y) * rdim + x] / 16u;
    }
    ID3D12Resource* TextureRes(int tenant) const { return m_tenants[tenant].res.Get(); }
    D3D12_RESOURCE_STATES TextureState(int tenant) const { return m_tenants[tenant].state; }
    uint32_t ResidencySrv(int tenant) const { return m_tenants[tenant].resMapSrv; }

    // The renderer's per-frame demand: face-uv rect (of THIS tenant's cube) wanted at `mip`.
    // Internally expands to tiles, bumps lastSeen, enqueues unseen ones coarse-to-fine.
    // M9w: is Want expensive because of the map, or because it TOUCHES THE SAME TILES over and
    // over? The mip-tail rule re-walks every ancestor for every leaf, so a coarse tile is
    // visited once per descendant. Counting touches against unique inserts separates
    // "the hash map is slow" from "we are asking it the same question hundreds of times".
    mutable uint64_t wantTouches = 0, wantHits = 0;
    void WantStatsReset() { wantTouches = wantHits = 0; }

    void Want(int tenant, uint32_t face, uint32_t mip, float u0, float v0, float u1, float v1,
              bool predicted = false);

    // Screw-prefetch input: this frame's camera pose motor. The manager keeps the previous one
    // and hands back the extrapolated pose for the caller to run its node walk a second time
    // with `predicted = true`.
    Motor PredictNextPose(const Motor& current, double aheadSeconds);

    // One call per frame, on the frame command list (mapping updates are queue-side and land
    // before this list executes). Records CopyTiles + barriers + residency-map uploads.
    void ProcessQueues(Gpu& gpu, ID3D12GraphicsCommandList* cl);

    // ---- field tenants (adapters; physics policies stay where they are) ---------------------
    // Registered for the unified stats line and the grade-signature registry only.
    void RegisterField(const char* name, std::function<uint64_t()> residentBytes,
                       std::function<uint32_t()> residentTiles);

    // ---- grade signatures (Cl(2): bit0 scalar, bit1 vector, bit2 bivector) ------------------
    // A field publishes its per-tile signatures on a grid it chooses; DeriveDemand applies the
    // proven Cayley closure so a derived field learns WHERE it can be non-zero without reading
    // any data. Grids must match dimensions.
    void PublishSignatures(const std::string& field, uint32_t tilesX, uint32_t tilesY,
                           std::vector<uint8_t> sig);
    // out[i] = union over products: Cl2ProductSignature(a[i], b[i]). Returns false if unknown.
    bool DeriveDemand(const std::string& srcA, const std::string& srcB,
                      std::vector<uint8_t>& out, uint32_t& tilesX, uint32_t& tilesY) const;

    std::string stats;       // "streams: mars 212/512 t 13 MB | earth 96 t | fetches 34" etc.
    uint32_t fetchesThisRun = 0;    // HTTP providers report through their closure

private:
    struct Tenant {
        std::wstring name;
        Com<ID3D12Resource> res;
        DXGI_FORMAT fmt = DXGI_FORMAT_UNKNOWN;
        uint32_t faceDim = 0, mips = 0, packedMips = 0, faces = 6;
        uint32_t srv = UINT32_MAX;
        std::vector<D3D12_SUBRESOURCE_TILING> tilings;   // per subresource (face*mips + mip)

        // M9x: THE FLAT STAMP ARRAY. Want() asks "have I already seen this tile this frame"
        // about twelve thousand times a frame, and 94% of the answers are yes. Asking an
        // unordered_map<uint64, shared_ptr> costs a hash, a bucket probe and a pointer chase
        // into scattered heap -- about 81 ns measured -- to retrieve a fact that fits in four
        // bytes and whose address is pure arithmetic on (face, mip, x, y).
        //
        // So the answer moves into a dense array indexed by exactly that arithmetic. The map
        // stays: it still owns the Tracked objects and everything the loader, mapper and
        // evictor do with them. What leaves the map is the HOT QUESTION, which never needed it.
        //
        // Encoding: frame * 2 + predicted, so one 32-bit read carries both halves of the skip
        // test. Zero means never seen, which is why frames are counted from 1.
        std::vector<uint32_t> stamp;
        std::vector<uint32_t> stampBase;   // offset of each (face, mip) plane into stamp
        std::vector<uint32_t> stampW;      // that plane's width in tiles, for the row stride
        TileProviderFn provider;
        // Residency map (base-tile granularity, per face): byte = finest resident mip * 16.
        GpuTexture resMap;
        uint32_t resMapSrv = UINT32_MAX;
        std::vector<uint8_t> resCpu[6];
        bool resDirty = false;
        D3D12_RESOURCE_STATES state = D3D12_RESOURCE_STATE_COPY_DEST;
    };

    enum class TileState : uint8_t { Seen, Loading, Loaded, Mapped, Failed };
    struct Tracked {
        int tenant;
        TileRequest req;
        uint32_t lastSeen = 0;
        uint32_t pool = UINT32_MAX;      // (chunk << 16) | tileInChunk when mapped
        TileState state = TileState::Seen;
        bool predicted = false;
        uint8_t retries = 0;             // M7w: failed loads retry, then go honestly NULL
        std::vector<uint8_t> data;
    };
    // Index of one tile in a tenant's flat stamp array. Pure arithmetic -- no hashing, no
    // indirection, and neighbours in a rect land next to each other in memory, which is the
    // half of the win the instruction count does not show.
    static inline size_t StampIndex(const Tenant& t, uint32_t face, uint32_t mip, uint32_t x,
                                    uint32_t y) {
        const uint32_t plane = face * t.mips + mip;
        return size_t(t.stampBase[plane]) + size_t(y) * t.stampW[plane] + x;
    }

    using Key = uint64_t;                // tenant:8 | face:3 | mip:5 | y:24 | x:24
    static Key MakeKey(int tenant, const TileRequest& r) {
        return (static_cast<Key>(tenant) << 56) | (static_cast<Key>(r.face) << 53) |
               (static_cast<Key>(r.mip) << 48) | (static_cast<Key>(r.y) << 24) |
               static_cast<Key>(r.x);
    }

    struct FieldAdapter {
        std::string name;
        std::function<uint64_t()> bytes;
        std::function<uint32_t()> tiles;
    };
    struct SigGrid {
        uint32_t tilesX = 0, tilesY = 0;
        std::vector<uint8_t> sig;
    };

    int AddTextureInternal(Gpu& gpu, const wchar_t* name, uint32_t faceDim, DXGI_FORMAT fmt,
                           TileProviderFn provider, uint32_t faces);
    void LoaderThread();
    uint32_t AcquirePoolTile(Gpu& gpu);
    void UpdateResidencyByte(Tenant& t, const TileRequest& r, bool mapped);
    void MapAndFill(Gpu& gpu, ID3D12GraphicsCommandList* cl,
                    const std::vector<std::shared_ptr<Tracked>>& batch);

    Gpu* m_gpu = nullptr;
    std::vector<Tenant> m_tenants;
    std::vector<Com<ID3D12Heap>> m_heaps;
    std::vector<uint32_t> m_freePool;

    std::map<Key, std::shared_ptr<Tracked>> m_tracked;
    std::deque<std::shared_ptr<Tracked>> m_seen;
    std::vector<std::shared_ptr<Tracked>> m_loading;
    std::vector<std::shared_ptr<Tracked>> m_mapped;
    uint32_t m_frame = 0;

    // Upload ring: kFrameCount slabs of kMaxMapsPerFrame tiles, frame-indexed like the CB arena.
    GpuBuffer m_uploadRing[Gpu::kFrameCount];

    // Loader workers.
    std::vector<std::thread> m_workers;
    std::deque<std::shared_ptr<Tracked>> m_loadQueue;
    std::mutex m_mx;
    std::condition_variable m_cv;
    std::atomic<bool> m_quit{false};
    std::atomic<int> m_inFlight{0};
    uint32_t m_failedLoads = 0;   // M7w: terminal load failures (tiles left honestly NULL)

    std::vector<FieldAdapter> m_fields;
    std::map<std::string, SigGrid> m_signatures;
    Motor m_prevPose;
    bool m_havePrevPose = false;
};

}  // namespace ga
