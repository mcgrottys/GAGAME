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

#include "compose/TileIndex.h"
#include "core/TileStream.h"
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
// M9ai: WHERE A TILE IS, instead of what it contains.
//
// A provider that finds its tile in an archive fills this and returns true WITHOUT touching
// out64k. The bytes then never enter CPU address space at all: DirectStorage takes the path,
// the offset and the mapped tile, and the read goes NVMe -> GPU. A provider that has to paint,
// or whose tile is only a loose file, fills out64k as before and leaves this empty -- so the
// two paths coexist per TILE, not per build.
struct TileLoc {
    const wchar_t* path = nullptr;   // archive path; owned by the archive, outlives the request
    uint64_t offset = 0;
    uint32_t size = 0;
    bool Valid() const { return path != nullptr && size != 0; }
};

using TileProviderFn =
    std::function<bool(const TileRequest&, std::vector<uint8_t>& out64k, TileLoc* loc)>;

class ResidencyManager {
public:
    static constexpr uint32_t kPoolChunkTiles = 128;     // 8 MB heap chunks
    // M7w: 12 in-flight loads on 2 workers drained ~0.5 tiles/frame -- a rail descent wants
    // thousands, so most of the view rode coarse fallbacks for the whole flight (the vintage
    // patchwork). Loads are disk/CPU paints; feed as many workers as the machine has.
    static constexpr uint32_t kMaxLoadsInFlight = 48;
    // M9af: slots held open for tiles that are NOT on disk, so preferring cached reads cannot
    // starve the painting that fills the cache. 12 of 48 = a quarter.
    static constexpr uint32_t kPaintReserve = 12;
    static constexpr uint32_t kMaxMapsPerFrame = 96;     // tiles mapped+filled per frame
    static constexpr uint64_t kMapStageBytes = 8ull << 20;   // M9bb: residency-map staging reserve
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
    // M9ap: ONE TENANT, N PAGES. The planet's colour as a single reserved Texture2DArray whose
    // slices are pages of one ladder: 0..5 the cube faces, 6.. the Mercator pages. One SRV, one
    // residency map, one budget, one provider that dispatches on the slice. The three inset
    // tenants this replaces were three pages of a ladder nobody had written down as a ladder,
    // with hand-off fades in the shader that each rung needed to know about its neighbour to
    // compute. A page's slice IS its `face` in every TileRequest. Slices 0..5 are also viewed
    // as a TextureCube so the globe keeps hardware-seamless cube filtering.
    int AddTexturePages(Gpu& gpu, const wchar_t* name, uint32_t dim, DXGI_FORMAT fmt,
                        TileProviderFn provider, uint32_t pages) {
        return AddTextureInternal(gpu, name, dim, fmt, std::move(provider), pages);
    }
    // The cube views over slices 0..5 of a page tenant (UINT32_MAX for other tenants).
    uint32_t TextureSrvCube(int tenant) const { return m_tenants[tenant].srvCube; }
    uint32_t ResidencySrvCube(int tenant) const { return m_tenants[tenant].resMapSrvCube; }

    // Tiles known but not yet mapped (seen + loading + in flight). The warm-cache loop drains
    // this to zero before the camera ever moves.
    uint32_t PendingCount() const {
        return static_cast<uint32_t>(m_seen.size() + m_loading.size());
    }

    uint32_t TextureSrv(int tenant) const { return m_tenants[tenant].srv; }
    uint32_t Mips(int tenant) const { return m_tenants[tenant].mips; }
    // M7l: the hypervisor asks what is ACTUALLY resident at a uv -- the same CPU-side map
    // the GPU residency clamp samples (byte = finest resident mip * 16).
    uint32_t ResidentMipAt(int tenant, uint32_t face, float u, float v) const {
        const Tenant& t = m_tenants[tenant];
        if (face >= t.resCpu.size() || t.resCpu[face].empty() || t.resMap.width == 0) return 255u;
        const uint32_t rdim = t.resMap.width;
        uint32_t x = static_cast<uint32_t>(u * rdim);
        uint32_t y = static_cast<uint32_t>(v * rdim);
        if (x >= rdim) x = rdim - 1;
        if (y >= rdim) y = rdim - 1;
        return t.resCpu[face][static_cast<size_t>(y) * rdim + x] / 16u;
    }
    ID3D12Resource* TextureRes(int tenant) const { return m_tenants[tenant].res.Get(); }
    ID3D12Resource* ResidencyRes(int tenant) const { return m_tenants[tenant].resMap.res.Get(); }
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
    // Step 5 (docs/PERF_EXPERIMENT.md): THE PREDICTED REQUEST STREAM, HASHED IN CALL ORDER.
    // FNV-1a over (tenant, face, mip, the rect's four float bit patterns) of every
    // Want(predicted = true), for the whole run. It is the A/B instrument for any change to
    // the prefetch walk -- the walk that moved to a worker thread in step 5 had to hand the
    // manager the identical stream in the identical order, and "identical" is this number:
    // the walk on the main thread (--predict-inline) and the walk on the worker print the
    // same hash over the storm rail, or the change is not exact. Printed by [rail],
    // --res-trace and the end-of-run [predict] line.
    uint64_t predictedHash = 14695981039346656037ull;
    uint64_t predictedCalls = 0;

    // M9af: hand a tenant the index of its own realization.
    //
    // MEASURED, AND NOT USED FOR SCHEDULING. The obvious use -- prefer loads that are a 64 KB
    // read over loads that are a 16384-sample paint -- was implemented and made the picture
    // WORSE: the descent seam came straight back, 33% of pixels changed for the worse against
    // the reference frame. The reason is that only ~1% of the finest level has ever been
    // painted, so the tiles a readiness preference demotes are exactly the ones that would fill
    // the far field. Preferring what is cached starves the work that fills the cache. Reserving
    // a quarter of the slots for paints recovered about a third of the loss and no more.
    //
    // It would pay on a WARM cache -- a repeat flight, where reads really are the whole job --
    // so the hook stays and the policy does not. What the index is actually for here is the
    // DirectStorage read path: knowing a tile's address, size and hash without touching the
    // filesystem is what lets a read be issued straight to the GPU.
    // M9ai: the NVMe -> GPU reader. Null keeps every tile on the upload-ring path.
    void SetTileStream(TileStream* ts) { m_stream = ts; }

    void SetTileIndex(int tenant, const TileIndex* idx) {
        if (tenant >= 0 && tenant < static_cast<int>(m_tenants.size())) {
            m_tenants[tenant].index = idx;
        }
    }

    void Want(int tenant, uint32_t face, uint32_t mip, float u0, float v0, float u1, float v1,
              bool predicted = false);
    // M9ba: DROP a tenant's every tile -- its provider's identity moved (the exposure node's
    // bucket rolled), so what is mapped is a different field now. Residency bytes go to
    // "nothing" this frame (consumers read absence, never the stale tile); the pool slots are
    // NULL-mapped after the frame-overlap window (priors 19: something recorded may still read
    // them); loads in flight for the old identity are discarded when they land.
    void Drop(int tenant);
    // M9bb: ONE tile changed on disk (a pyramid fold rewrote it, or a composite of it was
    // dropped): forget what is mapped at that address so the next Want refetches. Safe from
    // any thread -- it queues; ProcessQueues applies it on the main thread with Drop's rules.
    void Invalidate(int tenant, const TileRequest& r);
    // Debug: what the manager believes about one tile (state, pool slot, bytes it carried).
    std::string DebugTile(int tenant, const TileRequest& r) const {
        const Tracked* t = Find(tenant, r);
        if (!t) return "untracked";
        char b[160];
        snprintf(b, sizeof(b), "state %d pool %u data %zu loc %d dropped %d lastSeen %u retries %u",
                 int(t->state), t->pool, t->data.size(), int(t->loc.Valid()), int(t->dropped),
                 t->lastSeen, unsigned(t->retries));
        return b;
    }

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

    // M9al: THE INSTRUMENT, before the change. --res-trace prints, every `traceEvery` frames
    // and per texture tenant: the residency DEFICIT -- tiles wanted THIS frame that are not
    // mapped, by mip -- the queue depths, and where the load slots went: reads (a 64 KB file)
    // against paints (a 16384-sample composition), told apart by the provider's wall time.
    // The picture is made of the deficit; the slots are why it is what it is.
    bool traceRes = false;
    uint32_t traceEvery = 30;
    // M9al: RING LOADS. The user's proposal: do not ask for the finest mip a node needs in one
    // go -- admit a new request only if its PARENT is already mapped, so every pass advances
    // the whole view by one mip and neighbouring ground never differs by more than a ring.
    // What it changes is the QUEUE, not the invariant: coarse-before-fine MAPPING was always
    // enforced; REQUESTING was not, so on a fast descent the 48 load slots fill with finest-
    // mip tiles that are not on disk, each a paint, while the next ring of the ground the
    // camera is actually over queues behind them. Measured (section 32: frame 600 of the rail,
    // baseline patchy, ring uniformly sharp; timing a wash) and made the DEFAULT by the user.
    // --no-ring-loads is the A/B.
    bool ringLoads = true;
    bool dsSerial = false;   // M9ap diagnostic: one DirectStorage batch in flight at a time
    uint32_t ringHeld = 0;       // requests deferred by the gate, cumulative
    uint32_t ringHeldFrame = 0;  // ...and this frame alone

    // THE TURN'S CPU COST, BY PHASE. ProcessQueues runs inside GlobeLayer::Render, i.e. inside
    // the [rail] RENDER bracket, and was the only unnamed CPU work in it: the descent's record
    // tail (render minus GPU: p95 8.5-8.7 ms, max 15.2 ms at frame 561 against a 2.8 ms GPU,
    // out/instr_bench) has no owner until these say which phase carries it -- the per-tile
    // IDStorageFactory::OpenFile, the two shared_ptr sorts under m_mx, the 96 x 64 KB ring
    // memcpy, or the whole-tenant residency-map memcpy. Every phase is a steady_clock bracket;
    // queue-side UpdateTileMappings is inside its phase (it is CPU-side driver work), GPU
    // execution is not. Zeroed at the top of ProcessQueues; main reads them after RenderFrame.
    static constexpr int kPhases = 10;
    static const char* PhaseName(int k) {
        static const char* kNames[kPhases] = {
            "invalidate + retire (NULL maps)", "DS landed: fence poll + CopyTiles",
            "sort m_seen",                     "start loads",
            "sort m_loading",                  "gather mappable",
            "map: evict + UpdateTileMappings", "fill: DS OpenFile + Enqueue",
            "fill: ring memcpy + CopyTiles",   "residency map memcpy + copies"};
        return (k >= 0 && k < kPhases) ? kNames[k] : "?";
    }
    double phaseMs[kPhases] = {};
    double turnMs = 0.0;   // the whole ProcessQueues call (phases + the untimed stats string)
    // DirectStorage batches whose fence has not signalled: mapped, not yet claimed. Together
    // with PendingCount() this is "nothing is still landing" -- what --settle-sync waits for.
    uint32_t InFlightReads() const { return static_cast<uint32_t>(m_inFlightReads.size()); }

    // Step 25 (docs/PERF_EXPERIMENT.md): --settle-exact. THE RESIDENT SET IS THE WANT SET.
    //
    // --settle-sync's quiet test (pending 0, no in-flight read, nothing ring-held) fires over
    // two different resident sets: a request the ring gate held and the walk then stopped
    // asking for is neither pending nor resident, and what stays MAPPED is whatever the 240
    // real frames happened to land -- tiles the predicted walk asked for, tiles wanted at a
    // finer mip on the approach than at the pose. MEASURED (step 21's first attempt, bird,
    // --settle-sync --settle-hold 400, two runs, both quiet at pending 0): --lens bed differed
    // by 2.1 % of the pixels with max |d| 217, whole tiles resident at different mips; the
    // ring-held totals were 3.31 M against 3.11 M. The picture the shader samples is the
    // finest mapped mip under each pixel (the residency byte clamps the LOD there), so a
    // mapped tile the walk does not want is not inert: it is what a still is made of.
    //
    // While main raises settleExact (the held frames of a --settle-exact still), every
    // ProcessQueues turn reads the walk's own record -- the per-tile frame stamp Want()
    // writes down the whole ancestor column -- and (a) counts the DEFICIT, the stamped tiles
    // not mapped at their mip; (b) DROPS every tracked tile whose stamp is stale (not wanted
    // this frame at that mip) through Untrack + DropOne, the invalidation's own path: the
    // residency byte rises now, the NULL map and the pool slot follow kEvictAgeFrames later,
    // a tile still crossing the bus on a DirectStorage batch waits for its fence, and a tile
    // is only dropped once it has been unwanted for kEvictAgeFrames turns (the evictor's own
    // age rule); the coarsest mip is the floor and is never dropped; (c) leaves the issuing to
    // the walks, which run every held frame at the fixed pose; (d) reports the turn as EXACT
    // when the deficit is zero, no stale tile is tracked, and nothing is pending, in flight
    // or retiring -- main dumps after kEvictAgeFrames + 4 consecutive exact turns. A wanted
    // tile that can never land -- its own load failed, or an ancestor's did, so the ring gate
    // holds its children forever -- is excluded from the deficit BY ITS STATE (the manager
    // knows), never by a timeout.
    //
    // THE POOL DURING THE HOLD IS THE WANT SET. MEASURED (bird, the first exact run): the
    // walk wants 9176 tiles (573 MB) and kPoolCapTiles is 8192 (512 MB); at the cap the
    // evictor finds no victim (every mapped tile is wanted every frame), MapAndFill breaks
    // out of its batch, and the batch's remaining tiles -- already erased from m_loading --
    // are lost in state Loaded, tracked and unqueued, until something invalidates them.
    // Which 984 tiles lose is decided by landing order: that is the race a quiet
    // --settle-sync fired over. So while settleExact is raised MapAndFill's cap test is
    // skipped (the pool grows to the want set, bounded by the walk) and the exact turn
    // returns a wanted Loaded tile in no queue to m_loading. The ledger prints the mapped
    // total against the shipped cap so an over-cap pose is named. Outside a hold settleExact
    // is false and this is the shipped turn, call for call.
    bool settleExact = false;
    // Of ringHeldFrame: requests held under a parent whose load FAILED. They can never be
    // admitted, so the exact settle counts them as unreachable, not as deficit.
    uint32_t ringHeldDeadFrame = 0;
    struct SettleTurn {
        uint32_t wanted = 0;        // tracked tiles stamped by this frame's walks
        uint32_t mapped = 0;        // ... of which mapped at their mip
        uint32_t deficit = 0;       // ... wanted, not mapped, and able to land (+ live ring-held)
        uint32_t unreachable = 0;   // ... wanted, and never will land (Failed, or under one)
        uint32_t stale = 0;         // tracked, not wanted this frame, still tracked after the turn
        uint32_t dropped = 0;       // stale tiles dropped this turn
        uint32_t requeued = 0;      // wanted, Loaded, in no queue (the cap's casualties) -> m_loading
        uint32_t pending = 0, reads = 0, retiring = 0;
        bool exact = false;
    };
    SettleTurn settleTurn;   // the last turn's finding; main reads it after RenderFrame
    // The per-tenant report at the exit of an exact hold: wanted, mapped, deficit, dropped,
    // and an FNV-1a over the mapped set (face, mip, x, y in key order) -- two runs, or two
    // binaries that issue the same want stream, print the same hash or the difference is real.
    void LogSettleExact(uint32_t heldFrames) const;

private:
    struct Tracked;
    struct Tenant {
        std::wstring name;
        Com<ID3D12Resource> res;
        DXGI_FORMAT fmt = DXGI_FORMAT_UNKNOWN;
        uint32_t faceDim = 0, mips = 0, packedMips = 0, faces = 6;
        uint32_t srv = UINT32_MAX;
        uint32_t srvCube = UINT32_MAX;   // M9ap: page tenants: slices 0..5 as a cube
        std::vector<D3D12_SUBRESOURCE_TILING> tilings;   // per subresource (face*mips + mip)

        // M9x: THE FLAT STAMP ARRAY. Want() asks "have I already seen this tile this frame"
        // about twelve thousand times a frame, and 94% of the answers are yes. Asking an
        // unordered_map<uint64, shared_ptr> costs a hash, a bucket probe and a pointer chase
        // into scattered heap -- about 81 ns measured -- to retrieve a fact that fits in four
        // bytes and whose address is pure arithmetic on (face, mip, x, y).
        //
        // So the answer moves into a dense array indexed by exactly that arithmetic. What
        // leaves the map is the HOT QUESTION, which never needed it.
        //
        // Encoding: frame * 2 + predicted, so one 32-bit read carries both halves of the skip
        // test. Zero means never seen, which is why frames are counted from 1.
        std::vector<uint32_t> stamp;
        std::vector<uint32_t> stampBase;   // offset of each (face, mip) plane into stamp
        std::vector<uint32_t> stampW;      // that plane's width in tiles, for the row stride
        // Step 4 (docs/PERF_EXPERIMENT.md): THE SLOT ARRAY BESIDE IT. The map the M9x note
        // left in place was a std::map, not the unordered_map it described, and every touch
        // the stamp did NOT answer still went through it: the first touch of a tile in a frame
        // (~26.7k touches/frame of which 19.3k were stamp hits, step 3's storm bench), the
        // ring gate's parent probe on every new tile, the mapper's parent check and the
        // evictor's child check -- a red-black tree of every tracked tile of every tenant,
        // walked at 100-200 ns a find. The same arithmetic that addresses the stamp addresses
        // the Tracked itself: `slot` is laid out exactly like `stamp`, holds the tile's
        // Tracked (null = untracked), and every probe becomes one array read. The index is a
        // pure function of (face, mip, x, y) with no packing at all, which is what makes the
        // M9bf key-aliasing class of bug impossible here. `tracked` OWNS this tenant's tiles
        // (unordered; Tracked::pos is the entry's index, so removal is a swap with the back),
        // so Drop() walks its own tenant instead of every tenant's map. Every erase path --
        // Drop, an invalidation, an eviction -- goes through Untrack(), which clears the slot;
        // a stale slot would alias a reused Tracked, so --res-trace audits slot against list
        // for the first kSlotAuditFrames frames. 8 B per virtual tile (about 7 MB in all).
        std::vector<Tracked*> slot;
        std::vector<std::shared_ptr<Tracked>> tracked;
        TileProviderFn provider;
        // Residency map (base-tile granularity, per face): byte = finest resident mip * 16.
        GpuTexture resMap;
        uint32_t resMapSrv = UINT32_MAX;
        uint32_t resMapSrvCube = UINT32_MAX;   // M9ap
        std::vector<std::vector<uint8_t>> resCpu;   // per face/page
        bool resDirty = false;
        D3D12_RESOURCE_STATES state = D3D12_RESOURCE_STATE_COPY_DEST;
        // M9af: what this tenant's realization already holds on the NVMe. Borrowed; main owns
        // the indices. Null means "no information", which schedules exactly as before.
        const TileIndex* index = nullptr;
        // M9al: where this tenant's load slots went. A provider that returns in under 4 ms
        // read a file; one that took longer painted. Updated under m_mx by the workers.
        uint32_t loadsRead = 0, loadsPaint = 0;
        uint64_t readUs = 0, paintUs = 0;
        // Step 25: the exact settle's per-tenant ledger -- the last turn's counts and the
        // drops summed over the hold, for LogSettleExact.
        uint32_t exWanted = 0, exMapped = 0, exDeficit = 0, exUnreachable = 0, exStale = 0;
        uint32_t exDropped = 0, exDroppedMapped = 0, exRequeued = 0;
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
        std::vector<uint8_t> data;       // empty when loc is valid: the bytes stayed on disk
        TileLoc loc;
        uint64_t stageOffset = 0;   // where its bytes landed in the device buffer
        bool dropped = false;       // M9ba: identity moved under it; discard when it lands
        uint32_t pos = UINT32_MAX;  // step 4: its index in the tenant's `tracked` list
    };
    struct Retiring {
        std::shared_ptr<Tracked> tile;
        uint32_t frame;
    };
    std::vector<Retiring> m_retiring;   // M9ba: dropped-while-mapped, NULL-mapped after overlap
    std::mutex m_invMx;
    std::vector<std::pair<int, TileRequest>> m_invQ;   // M9bb: invalidations from paint threads
    // Marks the tile dropped and, when it was mapped, retires it. Returns that -- the caller
    // compacts m_mapped ONCE afterwards (erase_if on `dropped`, order kept) instead of a
    // std::find per tile: Drop() of a 23-slice tenant was O(dropped x mapped).
    bool DropOne(const std::shared_ptr<Tracked>& tr);
    // Index of one tile in a tenant's flat stamp array. Pure arithmetic -- no hashing, no
    // indirection, and neighbours in a rect land next to each other in memory, which is the
    // half of the win the instruction count does not show.
    static inline size_t StampIndex(const Tenant& t, uint32_t face, uint32_t mip, uint32_t x,
                                    uint32_t y) {
        const uint32_t plane = face * t.mips + mip;
        return size_t(t.stampBase[plane]) + size_t(y) * t.stampW[plane] + x;
    }
    // Step 4: the slot array's three operations. Find() is bounds-checked because its callers
    // hand in coordinates the walk did not compute (an invalidation from a paint thread, the
    // trace's DebugTile, the evictor's child probe on a grid whose width need not halve
    // evenly); the walk itself indexes `slot` directly with coordinates it derived in range.
    Tracked* Find(int tenant, const TileRequest& r) const {
        if (tenant < 0 || size_t(tenant) >= m_tenants.size()) return nullptr;
        const Tenant& t = m_tenants[tenant];
        if (r.face >= t.faces || r.mip >= t.mips) return nullptr;
        const auto& ti = t.tilings[r.face * t.mips + r.mip];
        if (r.x >= ti.WidthInTiles || r.y >= ti.HeightInTiles) return nullptr;
        return t.slot[StampIndex(t, r.face, r.mip, r.x, r.y)];
    }
    void Track(Tenant& t, const std::shared_ptr<Tracked>& tr);
    void Untrack(Tenant& t, Tracked* tr);
    // The bring-up gate for the slot array, under --res-trace: every non-null slot is a list
    // entry and every list entry's slot points back at it. A mismatch is the M9bf class of
    // bug (a stale index aliasing a reused Tracked) and aborts the run.
    static constexpr uint32_t kSlotAuditFrames = 1000;
    void AuditSlots() const;

    // M9ai: tiles whose bytes are in flight on the DirectStorage queue. They are MAPPED but not
    // yet claimed in the residency map, so the shader keeps sampling their coarser ancestor
    // until the fence says the read landed. That is the whole cross-queue synchronisation: no
    // barrier, no graphics-queue wait, just not lying about what has arrived yet.
    struct InFlightRead {
        uint64_t fence = 0;
        std::vector<std::shared_ptr<Tracked>> tiles;
    };

    // M9bf: THE KEY ALIASED. Three bits of face was the cube's six; a page tenant has up to
    // 23 slices, and face 14 of the wave tenant packed to the same key as face 6 of the tenant
    // after it -- the bank's coarsest plane tiles were "already tracked" (as wave tiles) and
    // never loaded, the mapping order held every finer level, the water went flat. The wave
    // planes 8..16 had been aliasing each other and their neighbours since the 23-slice tenant.
    // tenant:8 | face:8 | mip:6 | y:21 | x:21 -- 2M tiles per axis is 2^27 texels at 64/tile.
    // Step 4: no longer the map's key (the slot array is indexed without packing); it is the
    // ORDER the map walked a tenant's tiles in, which Drop() keeps so that m_retiring, the
    // NULL-map calls and the freed pool slots come out in the sequence they always did.
    using Key = uint64_t;
    static Key MakeKey(int tenant, const TileRequest& r) {
        return (static_cast<Key>(tenant) << 56) | (static_cast<Key>(r.face & 0xFFu) << 48) |
               (static_cast<Key>(r.mip & 0x3Fu) << 42) |
               (static_cast<Key>(r.y & 0x1FFFFFu) << 21) | static_cast<Key>(r.x & 0x1FFFFFu);
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

    // (The tracked set lives per tenant: Tenant::slot + Tenant::tracked, step 4.)
    std::deque<std::shared_ptr<Tracked>> m_seen;
    std::vector<std::shared_ptr<Tracked>> m_loading;
    std::vector<std::shared_ptr<Tracked>> m_mapped;
    uint32_t m_frame = 0;
    TileStream* m_stream = nullptr;
    std::vector<InFlightRead> m_inFlightReads;
    uint64_t m_directTiles = 0, m_ringTiles = 0, m_directLanded = 0;
    uint32_t m_stageUsed = 0;   // (unused since M9ap; the free list below owns the slots)
    // M9ap: landing slots are a FREE LIST, returned only after the CopyTiles that drains them
    // has been recorded. The previous "reset every frame" reused slot 0 while last frame's
    // read into slot 0 was still waiting for its fence -- a busy queue wrote one tile's bytes
    // into another's slot, and the globe came up green and magenta.
    std::vector<uint32_t> m_stageFree;
    bool m_stageInit = false;
    // M9ap: a drained slot is RETIRED, not freed. The CopyTiles that drains it is only
    // RECORDED when the batch completes; it executes when the frame's command list runs, and
    // DirectStorage would happily overwrite the slot before then. So a slot returns to the
    // free list kStageRetireFrames later -- the same overlap discipline the upload ring and
    // eviction already keep. Freeing on record put a NEW tile's bytes into an OLD coordinate.
    static constexpr uint32_t kStageRetireFrames = 4;
    std::deque<std::pair<uint32_t, std::vector<uint32_t>>> m_stageRetire;   // (frame, slots)
    bool m_directLogged = false;
public:
    uint64_t DirectTiles() const { return m_directTiles; }
    uint64_t RingTiles() const { return m_ringTiles; }
private:

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
