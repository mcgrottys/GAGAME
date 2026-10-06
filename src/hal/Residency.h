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
#include "core/Lattice.h"
#include "core/TileAddress.h"
#include "hal/TileStream.h"
#include "hal/Gpu.h"
#include "hal/ResidencyAudit.h"
#include "core/Pga.h"

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <functional>
#include <map>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

namespace ga {

class ResidencyManager {
public:
    static constexpr uint32_t kPoolChunkTiles = 128;     // 8 MB heap chunks
    // M7w: 12 in-flight loads on 2 workers drained ~0.5 tiles/frame -- a rail descent wants
    // thousands, so most of the view rode coarse fallbacks for the whole flight (the vintage
    // patchwork). Loads are disk/CPU paints; feed as many workers as the machine has.
    static constexpr uint32_t kMaxLoadsInFlight = 48;
    static constexpr uint32_t kMaxMapsPerFrame = 96;     // tiles mapped+filled per frame
    static constexpr uint64_t kMapStageBytes = 8ull << 20;   // M9bb: residency-map staging reserve
    static constexpr uint32_t kPoolCapTiles = 32768;     // 2 GB ceiling before eviction (the owner's, 2026-10-03)
    static constexpr uint32_t kEvictAgeFrames = 4;       // > frame overlap: no in-flight reads
    // ---- THE SAMPLERS (M13) -------------------------------------------------------------
    // ONE CACHE FOR THE EARTH, MANY READERS. A sampler is anything that will read the planet
    // and can be named: a view's walk, a subject whose surroundings stay resident, a gate's
    // window, a solver's domain pin, a warm-up tool. They share this pool -- a tile two
    // samplers want is ONE slot -- and each is answerable for what it asked for, which is what
    // a reserve is written against. So Want() carries WHO, and the per-tile record beside the
    // stamp is a MASK, not a flag: the second sampler's touch of an already-stamped tile used
    // to be skipped by the freshness test and left no trace at all, and a want nobody records
    // cannot be charged, protected, or reported.
    static constexpr int kMaxSamplers = 16;
    // Register (or find) a sampler by name; ids are handed out in registration order and are
    // stable for the run. Beyond kMaxSamplers the last id is shared and the overflow is logged
    // once -- a crowded scene loses accounting, never tiles.
    // `pin` (step 5): the reader stands, as a solver's domain does, and what it asks for is the
    // order's first class (HIERARCHY 4.19). Asking once with it marks the sampler for the run.
    int Sampler(const char* name, bool pin = false);
    int Samplers() const { return static_cast<int>(m_samplers.size()); }
    const char* SamplerName(int id) const {
        return (id >= 0 && id < Samplers()) ? m_samplers[id].c_str() : "?";
    }
    // Unique tiles each sampler asked for THIS frame (the currency a reserve is written in),
    // and the tiles it was alone in asking for. Reset when the frame's stamp changes, so they
    // read the same whether main asks before or after the turn.
    const uint32_t* SamplerTiles() const { return m_sampTiles; }
    const uint32_t* SamplerTilesAlone() const { return m_sampAlone; }
    // One line per sampler -- printed only when there is more than one, so a single-sampler
    // run's log is what it was.
    void LogSamplers() const;

    void Init(Gpu& gpu);
    // THE MANAGER is HIERARCHY 4.19's one order over tiles (ResidencyOrder.cpp): the held set is
    // the order's first P, the map a function of what is held. H2, streaming.holdMargin: a held
    // tile, and a tile above a held one, count for this times their measure (1: no margin).
    float holdMargin = 1.41421356f;
    // Law 8's table (step 5 D): the frame loop opens each frame and names each read; the wants and
    // the turn name themselves. One frame's order is logged once ([frame-table]).
    void FrameBegin();
    void Mark(const char* what);

    // Idempotent, and the destructor calls it: the pool outlives this object and its jobs
    // capture `this`, so leaving without draining them is a use-after-free waiting for a
    // slow load. main still calls it explicitly at every early exit.
    void Shutdown();
    ~ResidencyManager() { Shutdown(); }

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
    // `sliceTop`, per slice, the coarsest mip it holds (F1: a window's floor); empty, or a value
    // past the array's, is the array's coarsest.
    int AddTexturePages(Gpu& gpu, const wchar_t* name, uint32_t dim, DXGI_FORMAT fmt,
                        TileProviderFn provider, uint32_t pages, std::vector<uint8_t> sliceTop = {}) {
        return AddTextureInternal(gpu, name, dim, fmt, std::move(provider), pages, std::move(sliceTop));
    }
    // The cube views over slices 0..5 of a page tenant (UINT32_MAX for other tenants).
    uint32_t TextureSrvCube(int tenant) const { return m_tenants[tenant].srvCube; }
    uint32_t ResidencySrvCube(int tenant) const { return m_tenants[tenant].resMapSrvCube; }

    // Tiles known but not yet mapped. The warm-cache loop drains this to zero before the camera
    // ever moves. The tiles of the order's first P not held yet, whatever they wait on.
    uint32_t PendingCount() const { return m_orderPending; }
    // Tiles mapped in the pool, every tenant (a DirectStorage tile counts from its map, before its
    // bytes land). The solver's bed wait reports the tiles it waited for as the growth of this.
    uint32_t MappedCount() const { return static_cast<uint32_t>(m_mapped.size()); }

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

    // ---- THE FLOOR LAW (docs/HIERARCHY.md 4.6): PROPOSED, MEASURED, AND NOT STAGED ------------
    // The proposal: the map the GPU reads is the true one with each byte the largest (the
    // coarsest) of its 3 x 3, read BILINEAR. A bilinear read mixes the four bytes around its
    // sample; each of those, being the largest of its own 3 x 3, is at least the largest of the
    // four TRUE bytes there, so the mix is never finer than the gather + max in effect -- and it
    // is continuous, where the gather's sharpness steps along tile lines (porch_floor.py: the
    // largest step between samples a sixteenth of a cell apart falls from 7 mips to 0.44, for
    // 0.15 of a mip on average). The same widening would let a kernel's ONE byte (PageHaveLoad)
    // cover the four texels PageLoad4 reads around it (review finding 9). --selftest's [restest]
    // holds this construction to all of that, and it holds.
    //
    // WHY THE GPU STILL READS THE TRUE MAP. A clamp that is never finer is not a read that is
    // never outside what is resident. Measured on this GPU ([restest] gpu): once an anisotropic
    // footprint minifies, the sampler keeps its tap count and spaces the taps by the CLAMPED
    // mip's texels -- an 8:1 footprint of 8 texels under a clamp of 6.5 reaches 2.7 cells, where
    // one bilinear tap reaches half a cell. The floor ramps the clamp through intermediate coarse
    // mips in the cell before a frontier, so the colour's anisotropic taps (sAniso) cross it: under
    // the anisotropic sampler the floor touched NULL tiles the gather + max kept out at 2615 of the
    // probe's points, and a second ring (5 x 5) at 5394. The trilinear sampler and the kernels'
    // taps stay sound under it. The construction is kept, gated, for the law that replaces it.
    //
    // A cube face's border takes its neighbours from the face across the edge: the true cell
    // under the direction of the point one cell further along in this face's plane
    // (ComposeCubeDir past the square, CubeFaceOfDir), found once per map size (CubeFloorRing).
    // At a corner, where three faces meet, the geometry gives what it gives. Slices 0..5 of a
    // tenant with six or more are the cube (its cube views say so); every other slice is a
    // window and clamps at its edge, as the sampler does. The map is square (rdim) while a
    // tenant's tile grid need not be (UpdateResidencyByte's sx, sy): the dilation is in MAP
    // cells, which are what the shader's bilinear mixes.
    //
    // The cube's border, per map size: for faces 0..5, the flat index (face * rdim^2 + y * rdim
    // + x) of the true cell under each cell of a one-cell ring around the face, in the padded
    // map's raster order -- rdim + 2 above, then the left and the right cell of each of the
    // rdim rows, then rdim + 2 below: 4 rdim + 4 a face.
    static std::vector<uint32_t> CubeFloorRing(uint32_t rdim);
    // Slice s of `in` inside that ring: (rdim + 2)^2 bytes. With `cubeRing`, the border reads
    // the neighbouring faces; without one it repeats the slice's own edge (a window's clamp).
    static void FloorPad(const std::vector<std::vector<uint8_t>>& in, uint32_t rdim, uint32_t s,
                         const std::vector<uint32_t>* cubeRing, std::vector<uint8_t>& padded);
    // The floor map: every byte the largest of its 3 x 3 in `in`. `cubeRing` (null: every slice
    // clamps) is CubeFloorRing(rdim) and serves slices 0..5.
    static void FloorMap(const std::vector<std::vector<uint8_t>>& in, uint32_t rdim,
                         const std::vector<uint32_t>* cubeRing,
                         std::vector<std::vector<uint8_t>>& out);

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
    // F19 (the CPU walk's instrument): WHAT A WANT SPENDS, by phase. Calls by the slice asked
    // (a cube face, a window slice, a field with a focus); the column scan's stamp reads
    // (wantTouches, above); the weight loop's visits (a slot read and a record chased per tile per
    // level, fresh or not); the marks and the tracks; and the cycles of each phase (__rdtsc,
    // wantProfile on: a headless run is a measurement).
    bool wantProfile = false;
    mutable uint64_t wantCalls = 0, wantCube = 0, wantWindow = 0, wantField = 0;
    mutable uint64_t wantWeightVisits = 0, wantWeightRecs = 0, wantMarks = 0, wantTracks = 0;
    mutable uint64_t wantCycScan = 0, wantCycWeight = 0, wantCycMark = 0;
    // F19's audit (on with the residency audit, capture.residencyAudit): after a want, the reader's
    // statement on every tile of the column is no farther than the want's distance -- the law the
    // chase on every visit kept; a statement is only ever a visit's distance, so this bound makes
    // it the least of them. Counted, never acted on.
    mutable uint64_t wantAuditChecks = 0, wantAuditFails = 0;
    void WantStatsReset() {
        wantTouches = wantHits = 0;
        wantCalls = wantCube = wantWindow = wantField = 0;
        wantWeightVisits = wantWeightRecs = wantMarks = wantTracks = 0;
        wantCycScan = wantCycWeight = wantCycMark = 0;
        wantAuditChecks = wantAuditFails = 0;
    }
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

    // `sampler` is an id from Sampler(); every call site names the reader it belongs to.
    // Step 5 D: the want's WEIGHT, the order's fourth key: `nearM`, metres
    // from the reader's eye to the rect (a walk's leaf gives its own distance), and, for a reader
    // of one wide rect, its focus in the slice's uv -- a tile's weight is then its distance from
    // the focus on the ground and up to the eye's height (nearM). Negative focus: none.
    void Want(int sampler, int tenant, uint32_t face, uint32_t mip, float u0, float v0, float u1,
              float v1, bool predicted = false, float nearM = 0.0f, float focusU = -1.0f,
              float focusV = -1.0f);
    // M9ba: DROP a tenant's every tile -- its provider's identity moved (the exposure node's
    // bucket rolled), so what is mapped is a different field now. Residency bytes go to
    // "nothing" this frame (consumers read absence, never the stale tile); the pool slots are
    // NULL-mapped after the frame-overlap window (priors 19: something recorded may still read
    // them); loads in flight for the old identity are discarded when they land.
    void Drop(int tenant);
    // F13: THE IDENTITY MOVED UNDER EVERY TILE, AND THE HELD ONES HOLD. The version law
    // (Invalidate, the refill in MapAndFill) said of a whole tenant: a held tile stays mapped and
    // drawn, stale, and is asked anew in the order at its own measure; its replacement lands whole
    // and swaps in place. Loads in flight for the old identity are discarded as Drop discards
    // them. Nothing reads absence: the picture keeps the field it had until the new one lands.
    void Reload(int tenant);
    // M9bb: ONE tile changed on disk (a pyramid fold rewrote it, or a composite of it was
    // dropped): forget what is mapped at that address so the next Want refetches. Safe from
    // any thread -- it queues; ProcessQueues applies it on the main thread with Drop's rules.
    // B11: `moved` -- the slot's GROUND changed (a window's step, Tenant::Move), not the tile's
    // version: the old bytes are another place's and are let go at once (readers fall to the
    // parent); the version law (old bytes held until the replacement lands) is for a repaint.
    void Invalidate(int tenant, const TileRequest& r, bool moved = false);
    // PHASE A1 (the window's step, an instrument): the pool slot of a held tile -- mapped and
    // landed -- at a slot address, or UINT32_MAX. A tile a step kept is held at the same pool slot
    // the turns after it: its bytes were never moved, re-read or copied.
    uint32_t HeldPool(int tenant, const TileRequest& r) const {
        const Tracked* t = Find(tenant, r);
        return (t && t->state == TileState::Mapped && t->landed) ? t->pool : UINT32_MAX;
    }
    // F4 (2026-10-04, the corridor's struggle): THE STEP TELLS THE TILES IT TRACKS, NOT THE BOX.
    // f(req, heldPool) for every tile the manager tracks in slice `slice` of `tenant` (the slot
    // array's; a replacement's load is not a tile), heldPool its pool slot when held (mapped and
    // landed) else UINT32_MAX. A window's step (Tenant::Move) and the step's ledger ask this; a
    // slice that tracks nothing -- a set just claimed by a world of the corridor -- is a step of
    // no work. Before it, each step asked every one of the box's 21,760 slots twice (a six-slot
    // claim: 5 million asks in one frame), whether or not the slot held anything.
    template <class F> void ForTrackedIn(int tenant, uint32_t slice, F&& f) const {
        if (tenant < 0 || size_t(tenant) >= m_tenants.size()) return;
        for (const std::shared_ptr<Tracked>& tr : m_tenants[size_t(tenant)].tracked) {
            if (tr->req.face != slice || tr->refresh) continue;
            f(tr->req, (tr->state == TileState::Mapped && tr->landed) ? tr->pool : UINT32_MAX);
        }
    }
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
    // Step 5 E (1a): the prediction's lead, `aheadSeconds` frames, is how long its wants stand.
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
    // M12 step 3e: and the grid's GROUND, when the publisher has one to declare (a page
    // tenant's lattice; a bank on its own dense frame has none and passes null). Two grids
    // combine only on the same ground: DeriveDemand refuses otherwise, once aloud.
    void PublishSignatures(const std::string& field, uint32_t tilesX, uint32_t tilesY,
                           std::vector<uint8_t> sig, const Lattice* lattice = nullptr);
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
    bool dsSerial = false;   // M9ap diagnostic: one DirectStorage batch in flight at a time

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
            "the order: buckets + first P",    "the loader: first P not held",
            "release past P + let go",         "gather: first P loaded, to slots",
            "map: UpdateTileMappings",         "fill: DS OpenFile + Enqueue",
            "fill: ring memcpy + CopyTiles",   "map from held + copies"};
        return (k >= 0 && k < kPhases) ? kNames[k] : "?";
    }
    double phaseMs[kPhases] = {};
    double turnMs = 0.0;   // the whole ProcessQueues call (phases + the untimed stats string)
    // DirectStorage batches whose fence has not signalled: mapped, not yet claimed. Together
    // with PendingCount() this is "nothing is still landing" -- what --settle-sync waits for.
    uint32_t InFlightReads() const { return static_cast<uint32_t>(m_inFlightReads.size()); }

    // --settle-exact (an instrument): while main raises settleExact (the held frames of a still),
    // the order's cut is the want set whole, the pool may grow past its budget, and a turn is
    // EXACT when every wanted tile is held and nothing is stale, pending, in flight or retiring.
    // main dumps after kEvictAgeFrames + 4 exact turns (OrderTurn keeps the ledger).
    bool settleExact = false;
    struct SettleTurn {
        uint32_t wanted = 0;        // tracked tiles stamped by this frame's walks
        uint32_t mapped = 0;        // ... of which mapped at their mip
        uint32_t deficit = 0;       // ... wanted, not mapped, and able to land (+ live ring-held)
        uint32_t unreachable = 0;   // ... wanted, and never will land (Failed, or under one)
        uint32_t magnified = 0;     // ... wanted, and the level above magnified (PHASE A4): no bytes
        uint32_t stale = 0;         // tracked, not wanted this frame, still tracked after the turn
        uint32_t dropped = 0;       // stale tiles dropped this turn
        uint32_t pending = 0, reads = 0, retiring = 0;
        bool exact = false;
    };
    SettleTurn settleTurn;   // the last turn's finding; main reads it after RenderFrame
    // The per-tenant report at the exit of an exact hold: wanted, mapped, deficit, dropped,
    // and an FNV-1a over the mapped set (face, mip, x, y in key order) -- two runs, or two
    // binaries that issue the same want stream, print the same hash or the difference is real.
    void LogSettleExact(uint32_t heldFrames) const;
    // The pages ledger (pagesEvery), printed from inside the turn under m_mx.
    void LogPages() const;

    // Step 28 (docs/PERF_EXPERIMENT.md): THE LANDING LEDGER. What one turn did to the tiles
    // between the queue and the sampler, by the class of thing that can go wrong there: a
    // DirectStorage batch landing for a coordinate its tile no longer owns, a ring fill of a
    // tile that has NO bytes, a turn whose landed copies had no mapped batch behind them.
    // Zeroed at the top of ProcessQueues; printed every turn of the --res-trace-frames window
    // (main raises traceTurn per frame) and summed over the run for the [rail] tail.
    //
    // MEASURED (2026-09-05, the storm rail with --probe-cull-far, recorded frames 700-820,
    // the pool at its 8192-tile cap the whole time): the landing buffer never ran short (410-511
    // of 512 slots free at up to 96 direct tiles a turn), every fence signalled one turn after
    // its Submit, no fill lacked bytes -- and one turn in three landed a batch with NO mapped
    // batch behind it (rec746, 755, 758, 761, 764, 767, 770, 776, 779, 791, 800).
    //
    // RE-MEASURED (2026-09-06, this ledger, same rail and window): the same eleven recorded
    // frames came back LANDED-ONLY, plus rec774, every one of them with `barriered` = 0. Over
    // the whole flood rail 89 such turns and 85 unowned claims; over the 19:30Z rail 68 and 126.
    // Zero ring fills without bytes on both.
    //
    // THE BARRIER IS STILL AT THE TAIL OF MapAndFill, which a turn with an empty batch never
    // reaches, so those turns still draw the tenant mid-copy. Nothing here fixes that; this
    // ledger exists so the fix has a number to be gated against, and so the count can be
    // required to reach zero rather than assumed to (see ProcessQueues).
    struct TurnLedger {
        uint32_t landedBatches = 0, landedTiles = 0;   // fences signalled this turn; tiles claimed
        uint32_t landedRetired = 0;   // ... landed for a tile the retire loop already NULL-mapped
        uint32_t landedUnowned = 0;   // ... landed for a coordinate the tile no longer owns
        uint32_t lagMin = 0, lagMax = 0;   // turns from a batch's Submit to its fence, this turn
        uint32_t batch = 0, direct = 0, ring = 0;   // the mapped batch, by fill path
        uint32_t ringNoBytes = 0;   // ring fills of a tile with no bytes: a garbage fill
        uint32_t evicted = 0;
        uint32_t reclaimed = 0;     // of `evicted`, the ones the headroom pass took unasked
        uint32_t landedOnly = 0;    // 1: tiles landed and no batch was mapped this turn
        uint32_t barriered = 0;     // tenants transitioned back to shader reads after the copies
        uint32_t stageFree = 0, stageRetiring = 0, inFlightTiles = 0;   // after the turn
    };
    TurnLedger turn;
    uint64_t ringNoBytesTotal = 0, landedUnownedTotal = 0, landedOnlyTurns = 0;
    // Step 5: what the order's turn decided (OrderTurn), printed under the [res-turn] line. A
    // tile LET GO had its load finish after it left the first P: its bytes (letGoRead) or its
    // archive place were dropped in that turn, never mapped (law 4).
    struct OrderTurnLedger {
        uint32_t candidates = 0, cut = 0, firstPHeld = 0, pending = 0, waiting = 0;
        uint32_t letGo = 0, letGoRead = 0, forgotten = 0, lost = 0;
        // The livelock's instrument: a load started for a tile let go within the glance. And
        // the rescue's number: a tile released past P, back in the first P while its old slot
        // still held its bytes (the retire delay), so it is read again into a new slot.
        uint32_t reloaded = 0, rewanted = 0;
        uint32_t pass = 0;   // 1: the order was recomputed this turn (an event), 0: not
        uint32_t rescued = 0;      // 1b: taken back from the retire list, no read, no new slot
        uint32_t incomplete = 0;   // finding 83: loads that answered a tile not whole
        // H1: tiles that crossed the pass's cut since the pass before (in: into the first P), and
        // the readers whose statement changed this turn (a bit a reader).
        uint32_t crossIn = 0, crossOut = 0, spoke = 0;
        // F14: the loader's turn -- reads issued (and refills), slots mapped to bytes held (no
        // read), tiles known magnified without a load, and whether it stopped at the cap.
        uint32_t issued = 0, refills = 0, aliases = 0, magnifiedKnown = 0;
        bool atCap = false;
    };
    // H1: the measure's motion over the run, one line (the frame loop calls it at the end).
    void LogOrderMotion() const;
    uint64_t rescuedTotal = 0, incompleteTotal = 0;
    // Decision 5: the order's pass runs on events only. Its turns, the turns it skipped, the
    // entries it ordered and its time by part: statements, candidates, sort + cut, release + forget.
    uint64_t passTurns = 0, passSkipped = 0, passEntries = 0;
    double passMs[4] = {};
    OrderTurnLedger orderTurn;
    uint64_t letGoTotal = 0, letGoReadTotal = 0, reloadedTotal = 0, rewantedTotal = 0;
    uint64_t releasedTotal = 0, mappedTotal = 0;   // an untraced rail's releases and maps
    // PHASE B2w: THE RELEASES OF A TURN, and those of a tile a reader's statement of the frame before
    // (the one this turn acts on) still named -- by the order's cut (pool pressure) and by an
    // invalidation (a window's step, Tenant::Move, or a repaint). Reset every turn.
    struct ReleaseLedger {
        uint32_t cut = 0, cutNamed = 0, invalidated = 0, invalidatedNamed = 0;
    } releaseLedger;
    std::vector<uint32_t> railMaps;   // H5: the tiles each recorded frame's turn mapped (the frame loop)
    bool traceTurn = false;        // print this turn's ledger (main: --res-trace-frames)
    uint32_t traceRecFrame = 0;    // the recorded frame main labels it with

    // ---- THE PAGES LEDGER (slice pool, stage 0, 2026-09-17) --------------------------------
    // Every other residency line sums a tenant over its slices, so a page is invisible in them:
    // the z14 window's want and the cube's arrive as one number. This one splits each tenant by
    // the lattices its slices sit on -- the cube's six faces as one line, each page as its own --
    // and counts, per mip, what this frame wanted, how much of that is mapped, and what the pool
    // holds that nobody wanted. It reads the manager's own state and NO GEOMETRY: to the manager
    // a cube face and a page are the same thing, a slice of one array, and an instrument that
    // re-derived the shader's page choice on the CPU would be a second copy of a law the shader
    // owns. Counted at the settle's own point with the settle's own test (the stamp says this
    // frame), so under --settle-exact a tenant's lines sum to its [settle-exact] line, and the
    // tenants' mapped tiles sum to the pool's -- the print checks the second itself.
    uint32_t pagesEvery = 0;       // main: --pages-trace N prints every Nth turn (0 = never)
    // What the manager did not already know about a slice: the cache tag of the lattice it sits
    // on and that lattice's mip-0 ground in metres. Handed over by the declaration
    // (hal::Tenant::Sparse), once; an unlabelled slice prints by index.
    void LabelSlices(int tenant, std::vector<std::string> tags, std::vector<double> ground0M);
    // THE STARVED TILE'S GLOBAL NAME (the watchdog below): a tenant of block slices tells the slot
    // tile's pyramid tile (hal::Tenant::Sparse hands it over); empty = the slot's own name alone.
    void SetTileNamer(int tenant, std::function<std::string(const TileRequest&)> namer) {
        if (tenant >= 0 && tenant < static_cast<int>(m_tenants.size())) m_tenants[tenant].namer = std::move(namer);
    }
    // ONE TILE, MANY WINDOWS (HIERARCHY 4.7; 2026-10-04): a slot's ADDRESS in the pyramid, from
    // the tenant of block slices (hal::Tenant::Sparse: BlockBinding::Global). The pool holds
    // bytes by address, once; a slot is a mapping of its address's bytes. A tenant that hands
    // over no function shares nothing (the cube's faces, a plane: today's path).
    void SetGlobalOf(int tenant, std::function<bool(const TileRequest&, TileRequest&)> fn) {
        if (tenant >= 0 && tenant < static_cast<int>(m_tenants.size())) m_tenants[tenant].globalOf = std::move(fn);
    }
    // F14 (HIERARCHY 4.20, the third clause, asked before the load): a tile finer than every
    // source's own level is the level above, magnified -- no bytes, nothing mapped, its residency
    // byte names the parent. The tree knows this by arithmetic; a tenant that hands the question
    // over has it answered in the order (Failed + magnified, the same state a load's answer gave)
    // and never spends one of the queue's slots on it. One that hands over nothing loads as before.
    void SetMagnifiedOf(int tenant, TileQueryFn fn) {
        if (tenant >= 0 && tenant < static_cast<int>(m_tenants.size())) m_tenants[tenant].magnifiedOf = std::move(fn);
    }
    // F18: A FIELD'S PLANES ARE ONE TILE. A field tenant's slices are the planes of one value at one
    // address (the shader adds them: a plane not landed is zero), so the tile the order cuts is all
    // of them: the planes are the fiber, the cut is over the base. Declared by the tenant
    // (Semantics::Field); the order ties by the address with its face last, and the cut inside the
    // straddling bucket lets a tile go whole rather than keep some of its planes.
    void SetPlanesOneTile(int tenant, bool on) {
        if (tenant >= 0 && tenant < static_cast<int>(m_tenants.size())) m_tenants[tenant].planesOneTile = on;
    }
    // F14: THE LOADER'S LEDGER over the run (LogLoader at the exit): what the order issued a turn,
    // the turns it stopped at the cap, the queue's depth as a turn began, and a load's way from
    // issue to the batch that maps it, in turns. The per-tenant halves (read/paint/failed/
    // magnified, queue wait, work) live on Tenant.
    struct LoaderLedger {
        uint64_t turns = 0, turnsAtCap = 0, inFlightAtTurn = 0, pendingAtTurn = 0;
        uint64_t reads = 0, refills = 0, aliases = 0, magnifiedKnown = 0;
        uint64_t gathered = 0, gatherTurns = 0;
        uint64_t unitCuts = 0, unitPlanes = 0;   // F18: the cut fell inside a tile's planes; the planes let go with it
    };
    LoaderLedger loader;
    void LogLoader() const;
    uint64_t sharedMapsTotal = 0;   // mappings made without a read, this run
    // ---- THE WATCHDOG, a law of the ledger (nothing here acts). Every turn, a tile of the first P
    // that is not held, has no load in flight and no retiring slot is STARVED; one starved past the
    // glance (kGlanceTurns) is printed once with its whole state ("[starved]") and again every
    // kStarvePrintTurns while it lasts. A tile whose load stays in flight past kStarvePrintTurns is
    // printed the same way ("[stuck]"): the same stall seen from the loader's side.
    static constexpr uint32_t kStarvePrintTurns = 600;
    static constexpr uint32_t kStarveLinesPerTurn = 16;   // whole lines a turn; the rest summed
    uint32_t starvePlant = 0;   // the plant: the loader never starts a load of this slice (0 = off)
    uint32_t starvedNow = 0, starvedPast = 0, stuckPast = 0;   // this turn's counts
    uint64_t starvedPrinted = 0;                                // over the run
    // THE VERSION LAW's ledger: replacements swapped in whole / refused (old bytes kept), and the
    // held tiles standing at an old version this turn.
    uint64_t refreshedTotal = 0, refreshRefusedTotal = 0;
    // THE GUARD: held tiles released because their parent was refused while not held. By the
    // loader's order (ParentGate) it cannot happen; it must read 0.
    uint64_t releasedUnderRefusedTotal = 0;
    uint32_t staleHeld = 0;

    // ---- THE RESIDENCY AUDIT (hal/ResidencyAudit.h; the bodies live in ResidencyAudit.cpp, so
    // this class carries the declarations and Residency.cpp two call lines). Every auditEvery-th
    // turn, at the turn's end -- the bytes this frame's globe samples -- every residency byte
    // against what the tiles mapped AND landed say it should be: one [res-audit] line when clean,
    // the new offenders named when not; and the invalidations applied since, with those that
    // dropped a tile over a mapped descendant, the only kind that can leave finding 3's signature.
    uint32_t auditEvery = 0;       // main: the scene's capture.residencyAudit (0 = never)
    void LogAudit();
    // The audit on CONSTRUCTED tenants, through this class's own byte writer (--selftest).
    static bool AuditSelfTest();
    // One pass of it through the one byte writer, WriteHeldFootprint, fed the constructed tiles as
    // its held set (the map a function of what is held).
    static bool AuditSuite();

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
        // M13: WHICH SAMPLERS asked for this tile this frame -- one bit each, addressed by the
        // same arithmetic as the stamp and only meaningful while the stamp is this frame's.
        // 2 B per virtual tile beside the stamp's 4 and the slot's 8.
        std::vector<uint16_t> want;
        // F19: THE LEAST DISTANCE SAID OF THE TILE THIS FRAME (WeightBits), dense beside the stamp
        // and meaningful while the stamp is this frame's. A leaf's want gives every tile of its
        // column its distance and a tile's weight is the least of its leaves'; this is that least,
        // kept where the stamp is, so a tile's record is chased only when a reader says a lesser
        // distance than any said so far -- not on every visit of every leaf under it (140,000
        // chases a frame at the helm, 2.1 ms, for 20,000 distinct tiles). PER READER: the least
        // is the reader's in `leastSid`; another reader's visit chases its own statement and takes
        // the pair over, so each reader's statement is exactly the least of its own visits (a
        // prediction's stands for its lead across frames, so the least cannot be the frame's).
        std::vector<uint32_t> least;
        std::vector<uint8_t> leastSid;
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
        TileQueryFn magnifiedOf;   // F14: known without a load (SetMagnifiedOf); empty = ask the provider
        bool planesOneTile = false;   // F18: the slices are planes of one value; the cut's unit is all of them
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
        // F14: the rest of where the slots went -- loads that answered nothing (magnified by the
        // provider, failed or not whole), and the queue wait from the order's Submit to the job's
        // first instruction, summed (the Io lane's saturation, told apart from the cap's).
        uint32_t loadsMagnified = 0, loadsFailed = 0, magnifiedKnown = 0;
        uint64_t queueUs = 0;
        // Step 25: the exact settle's per-tenant ledger -- the last turn's counts and the
        // drops summed over the hold, for LogSettleExact.
        uint32_t exWanted = 0, exMapped = 0, exDeficit = 0, exUnreachable = 0, exStale = 0;
        uint32_t exMagnified = 0;   // PHASE A4: wanted tiles that are their parent, magnified
        uint32_t exDropped = 0, exDroppedMapped = 0;
        // The pages ledger's names for the slices (LabelSlices): empty until declared.
        std::vector<std::string> sliceTag;
        std::function<std::string(const TileRequest&)> namer;   // SetTileNamer
        bool saidNoSlice = false;   // Want: a slice past its faces, said once
        std::vector<double> sliceGround0M;
        // Step 5: floor(log2 of the slice's mip-0 texel in metres), the order's rung at mip 0
        // (kNoRung for a slice that declared no ground); and the exact hold's lost tail.
        std::vector<int> sliceRung0;
        uint32_t exLost = 0;
        // F1 (HIERARCHY 4.1): the coarsest mip each slice HOLDS, its chain's end -- the array's
        // coarsest (mips - 1) for every slice but a window, whose FLOOR is its mip 3: the same
        // ground in the rank above answers beyond it, so its mips above are neither wanted nor
        // mapped, and its residency byte is computed from its mips up to the floor alone (255,
        // nothing held, where the floor's tile is not held: the rank above answers).
        std::vector<uint8_t> sliceTop;
        // 4.7: the slot's address in the pyramid (SetGlobalOf), and what is HELD by address:
        // its pool slot (none while its read is on the way), the slots mapped to it, whether its
        // bytes have landed, and the slot whose read fills it.
        std::function<bool(const TileRequest&, TileRequest&)> globalOf;
        struct Held {
            uint32_t pool = UINT32_MAX;
            bool landed = false;
            TileRequest reader;             // the slot whose read fills the address
            std::vector<Tracked*> slots;    // the slots mapped to the pool slot (refs)
        };
        std::unordered_map<uint64_t, Held> held;
    };
    // The address packed: face 3 bits, the pyramid's mip 6, x and y 26 each (rung 17's 2^24 tiles).
    static uint64_t AddressKey(const TileRequest& g) {
        return (uint64_t(g.face & 7u) << 58) | (uint64_t(g.mip & 63u) << 52) |
               (uint64_t(g.y & 0x3FFFFFFu) << 26) | uint64_t(g.x & 0x3FFFFFFu);
    }
    // A slot's address key, 0 for a slot of no address.
    static uint64_t KeyOf(const Tenant& t, const TileRequest& r) {
        TileRequest g;
        return (t.globalOf && t.globalOf(r, g)) ? AddressKey(g) : 0ull;
    }
    void Landed(Tracked* tile);                    // held from this turn: it and every slot of its address
    void ReleaseSlot(Tenant& t, Tracked& tile);    // a retiring slot lets go of its address
    static uint32_t TopOf(const Tenant& t, uint32_t face) {
        return face < t.sliceTop.size() ? t.sliceTop[face] : t.mips - 1u;
    }


    enum class TileState : uint8_t { Seen, Loading, Loaded, Mapped, Failed };
    struct Tracked {
        int tenant;
        TileRequest req;
        uint32_t lastSeen = 0;
        uint32_t pool = UINT32_MAX;      // (chunk << 16) | tileInChunk when mapped
        TileState state = TileState::Seen;
        bool predicted = false;
        uint8_t retries = 0;             // M7w: failed loads retry, then go honestly NULL
        // PHASE A4: the tree answered it as the level above, magnified (g_tileMagnified): it holds
        // no bytes and is never mapped -- the Failed state's law, "honestly NULL so consumers fall
        // back to the coarser REAL mip", which is what it is -- and the ledger counts it apart.
        bool magnified = false;
        bool magKnown = false;           // F14: the magnified question asked of the tree once
        uint32_t issuedTurn = 0;         // F14: the turn its load was issued (the ledger's latency)
        std::chrono::steady_clock::time_point issuedAt{};
        std::vector<uint8_t> data;       // empty when loc is valid: the bytes stayed on disk
        TileLoc loc;
        uint64_t stageOffset = 0;   // where its bytes landed in the device buffer
        bool dropped = false;       // M9ba: identity moved under it; discard when it lands
        uint32_t pos = UINT32_MAX;  // step 4: its index in the tenant's `tracked` list
        // Step 5: its bytes are on the GPU -- the boot's fill, a ring fill with a
        // whole tile, a DirectStorage batch whose fence signalled. HELD = Mapped and landed.
        bool landed = false;
        uint32_t firstP = 0;          // the last pass that placed it among the first P
        uint32_t rec = UINT32_MAX;    // step 5 E: its record in m_rec
        // The watchdog: since when it has been starved (1) or in flight (2), the turn last seen
        // so, and the turn last printed (0 = never).
        uint32_t starveSince = 0, starveSeen = 0, starvePrinted = 0;
        uint8_t starveClass = 0;
        // THE VERSION LAW (after B3): an invalidation changes a held tile's VERSION, not its
        // presence. `stale` -- held at an old version, its replacement asked; `refresh` -- this
        // Tracked is that replacement's load (never in the slot array); `incomplete` -- answered
        // not whole: a refusal of this attempt, retried when the tree next changes for the tile;
        // while it is not held, it blocks its children (they are unreachable as it is).
        bool stale = false, refresh = false, incomplete = false;
        std::shared_ptr<Tracked> refill;   // F13: a stale held tile's replacement on its way (m_refresh)
        // 4.7: the slot's address (0: none) and whether it is a mapping of bytes another slot read.
        uint64_t gkey = 0;
        bool alias = false;
    };
    // A held tile's replacement in flight: the held tile (its bytes stay mapped), the load, and
    // whether its version changed again while the load was out (then the load is let go and a new
    // one asked).
    struct Refresh {
        std::shared_ptr<Tracked> held, job;
        bool again = false;
    };
    std::vector<Refresh> m_refresh;
    struct Retiring {
        std::shared_ptr<Tracked> tile;
        uint32_t frame;
        bool rewanted = false;   // step 5: counted once as wanted back before its slot was free
        bool rescuable = false;  // step 5 E: released past P (bytes still good), not a drop
    };
    std::vector<Retiring> m_retiring;   // M9ba: dropped-while-mapped, NULL-mapped after overlap
    std::mutex m_invMx;
    struct Inv { int tenant; TileRequest req; bool moved; };
    std::vector<Inv> m_invQ;   // M9bb: invalidations from paint threads; B11: moved = ground changed
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
        uint32_t frame = 0;   // the turn that submitted it (step 28: the ledger's lag)
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
        const Lattice* lattice = nullptr;   // M12 step 3e: the ground; null = undeclared
    };

    int AddTextureInternal(Gpu& gpu, const wchar_t* name, uint32_t faceDim, DXGI_FORMAT fmt,
                           TileProviderFn provider, uint32_t faces, std::vector<uint8_t> sliceTop = {});
    // One load, on a Lane::Io job. Was the body of LoaderThread's loop.
    void RunLoad(const std::shared_ptr<Tracked>& job);
    uint32_t AcquirePoolTile(Gpu& gpu);
    void MapAndFill(Gpu& gpu, ID3D12GraphicsCommandList* cl,
                    const std::vector<std::shared_ptr<Tracked>>& batch,
                    const std::vector<std::shared_ptr<Tracked>>& refills);

    Gpu* m_gpu = nullptr;
    std::vector<Tenant> m_tenants;
    std::vector<Com<ID3D12Heap>> m_heaps;
    std::vector<uint32_t> m_freePool;

    // (The tracked set lives per tenant: Tenant::slot + Tenant::tracked, step 4.)
    std::vector<std::shared_ptr<Tracked>> m_loading;
    std::vector<std::shared_ptr<Tracked>> m_mapped;
    // M13: the sampler registry and this frame's per-sampler tile counts.
    std::vector<std::string> m_samplers;
    uint32_t m_sampTiles[kMaxSamplers] = {};
    uint32_t m_sampAlone[kMaxSamplers] = {};
    uint32_t m_sampFrame = 0;          // the stamp frame those counts belong to
    bool m_sampOverflowed = false;

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

    // The loader threads live in the process pool now (core/ThreadManager.h, Lane::Io, capped
    // at this pool's old worker count and served FIFO). What stays here is the STATE they
    // touched: m_mx still guards Tracked and the per-tenant load counters, m_inFlight is still
    // the kMaxLoadsInFlight admission, and m_drainCv is how Shutdown waits for jobs that are
    // already running to leave before the manager they capture goes away.
    std::mutex m_mx;
    std::condition_variable m_drainCv;
    std::atomic<bool> m_quit{false};
    std::atomic<int> m_inFlight{0};
    uint32_t m_failedLoads = 0;   // M7w: terminal load failures (tiles left honestly NULL)

    std::vector<FieldAdapter> m_fields;
    std::map<std::string, SigGrid> m_signatures;
    mutable bool m_groundRefusalSaid = false;   // M12 step 3e: DeriveDemand's refusal, once
    Motor m_prevPose;
    bool m_havePrevPose = false;

    // ---- STEP 5, THE ORDER'S MANAGER (ResidencyOrder.cpp; docs/HIERARCHY.md 4.19) ----------------
    // LAW 1: THE MAP IS A FUNCTION OF WHAT IS HELD. The byte over every mip-0 cell under tile r is
    // the finest mip at which the tile over the cell is held with every coarser one held, times
    // 16; 255 where the coarsest is not held. Written from the tilings and `held` alone: no
    // increment, so no order of maps and unmaps can leave it wrong (findings 3, 63, 64).
    using HeldFn = bool (*)(const void* ctx, uint32_t face, uint32_t mip, uint32_t x, uint32_t y);
    static void WriteHeldFootprint(Tenant& t, const TileRequest& r, HeldFn held, const void* ctx);
    // The engine's predicate: the slot array's tile is Mapped and landed. ctx is the Tenant.
    static bool HeldInSlots(const void* ctx, uint32_t face, uint32_t mip, uint32_t x, uint32_t y);
    // A tile whose held state changed this turn; ApplyChanged rewrites every such footprint once
    // the turn's maps, claims and releases are all made, before the map goes up.
    std::vector<std::pair<int, TileRequest>> m_changed;
    void NoteChanged(int tenant, const TileRequest& r) {
        m_changed.push_back({tenant, r});
        m_edgeNotes.push_back({tenant, r});   // H2's closure, by events
    }
    void ApplyChanged();
    // LAW 9: the map born saying nothing, then written from the boot's held floor and sent to the
    // GPU on an upload that waits, so no reader ever meets the zeroed copy (finding 66).
    void BirthMap(Gpu& gpu, int tenant);
    // LAWS 2, 4, 5: THE ORDER and its cut (OrderTurn). P is the pool less a reserve for the slots
    // a release keeps until no frame in flight can read them: a turn's maps times the overlap.
    static constexpr uint32_t kSlotReserve = kMaxMapsPerFrame * kEvictAgeFrames;   // 384
    static constexpr uint32_t kGlanceTurns = 60;   // "within the glance": ~2 s at 30 fps
    // Room for rung 17 (4.7 mm, floor(log2) = -8) up to a planet's width (2^23 m) and their mips:
    // a parent's bucket strictly before its child's everywhere (the selftest holds it).
    static constexpr uint32_t kRungs = 56, kRungTop = 32;         // bucket = kRungTop - rung
    static constexpr uint32_t kBuckets = 2u * 3u * kRungs;        // (pin, lateness, rung)
    static constexpr int kNoRung = -1000;
    struct OrderEntry {
        uint32_t bucket = 0, weight = 0;   // weight: the order's fourth key, the nearer the lower
        uint64_t key = 0;                  // the address, the tie's breaker (MakeKey)
        Tracked* tile = nullptr;
    };
    std::vector<OrderEntry> m_ord, m_ordSorted, m_need;
    std::unordered_map<uint64_t, uint32_t> m_letGoAt;   // address -> the turn it was let go
    uint32_t m_orderPending = 0;   // first-P tiles not held (PendingCount)
    uint16_t m_pinMask = 0;        // the samplers that stand (Sampler(name, true))
    // Decides the turn's loads (toLoad), its releases and let-gos, and the batch to map.
    void OrderTurn(std::vector<std::shared_ptr<Tracked>>& toLoad,
                   std::vector<std::shared_ptr<Tracked>>& batch,
                   std::vector<std::shared_ptr<Tracked>>& refills);
    static uint32_t RungIndex(const Tenant& t, uint32_t face, uint32_t mip);
    // THE MAP LAW AS THE LOADER'S ORDER (4.7: mapped coarse to fine): a tile is asked for only when
    // its nearest ancestor that is not magnified is HELD (Open). Not held yet -> Wait (wanted,
    // pending, asked next turn). Refused and not held -> Refused (unreachable, as its parent is,
    // until the tree changes for it). `at` gets that ancestor (null when untracked or none).
    enum class Gate : uint8_t { Open, Wait, Refused };
    Gate ParentGate(const Tenant& t, const Tracked* tr, const Tracked** at = nullptr) const;
    static uint32_t RungIndexOf(int r0, uint32_t mip, uint32_t mips, int rungTop, int rungs);
    // STEP 5 E: WHAT THE ORDER READS OF A TILE, IN ONE ARRAY (decision 3). Written when a reader
    // marks the tile (OrderNote), when the tile is tracked or untracked (Track, Untrack: one
    // record a tracked tile, swap-removed), when it is held; read front to back by the pass.
    static constexpr uint64_t kStatementBasis = 14695981039346656037ull;
    static constexpr uint16_t kNoBucket = 0xFFFFu;
    enum : uint8_t { kHeldBit = 1, kFloorBit = 2, kDeadBit = 4, kInPBit = 8, kForgetBit = 16,
                     kWasInBit = 32,     // H1: in the first P at the pass before (the instrument's)
                     kUpBit = 64 };      // H2: not held, above a held tile: counts as held (the pass's)
    struct OrdRec {
        uint64_t key = 0;          // MakeKey: the address, the tie's breaker
        Tracked* tile = nullptr;
        uint32_t weight = 0;       // the least distance, m, from the eye of a reader of `stamp`'s frame
        uint32_t stamp = 0;        // the frame of its last statement, any reader
        uint32_t predUntil = 0;    // 1a: a prediction's want stands until this frame
        uint32_t pinFrame = 0;     // the last frame a standing reader (a pin) asked for it
        uint16_t mask = 0;         // the readers of `stamp`'s frame
        uint16_t bucket = kNoBucket;
        uint8_t rung = 0, flags = 0;
        float texel = 0.0f;        // F: the tile's texel on the ground, m (0: a slice without ground)
        uint32_t meas = 0;         // F: the measure's bits, texel / distance from the eye (the pass)
        // H1, the instrument (nothing decides by them): the record at the last pass that ordered it.
        uint32_t ppass = 0;        // that pass's number (passTurns; 0: never ordered)
        uint32_t pmeas = 0;        // its measure's bits then
        uint16_t pmask = 0, pbucket = kNoBucket;   // its readers and its bucket then
        // H5, THE WEIGHT STANDS WITH ITS STATEMENT: each reader's own last distance for the tile
        // and the frame it said it; a reader that speaks again replaces its own slot only. The
        // pass takes the nearest over the readers whose statement stands (`weight` above is the
        // pass's result: the distance its measure was made from).
        static constexpr int kSlots = 4;
        int8_t ssid[kSlots] = {-1, -1, -1, -1};
        uint32_t sstamp[kSlots] = {}, sweight[kSlots] = {};
    };
    static int SlotFor(const OrdRec& r, int sid);   // H5: the reader's slot, a free one, or the oldest
    static void SlotApply(OrdRec& r, int sid, uint32_t stampFrame, uint32_t wbits);
    // H5: the distance the pass measures from: the nearest over the standing statements.
    static uint32_t StandingWeight(const OrdRec& r, uint32_t now, int predSid, uint32_t predLead,
                                   const uint32_t* sampLast);
    // H2's closure by events (the law of MarkAboveHeld, walked only from where it can break): the
    // held-state changes since the last pass, and the held tiles whose parent was not held then.
    std::vector<std::pair<int, TileRequest>> m_edgeNotes, m_orphans;
    void FindOrphans();
    static void MarkUpFrom(std::vector<OrdRec>& rec, uint32_t i,
                           uint32_t (*parentOf)(const void*, uint32_t), const void* ctx);
    uint64_t m_closureChecks = 0, m_closureMissTurns = 0, m_closureMissMax = 0, m_orphansMax = 0;
    // H1: THE MEASURE'S MOTION between two passes, tile by tile, and the cut's crossings, over the run.
    struct OrderMotion {
        uint64_t pairs = 0;          // records ordered by two passes running
        uint64_t left[4] = {};       // ... whose measure moved by more than 1.01, 1.1, 1.41, 2
        float qmax = 1.0f, qmaxPrev = 0.0f, qmaxNow = 0.0f;   // the largest move and its measures
        uint64_t qmaxKey = 0;
        uint32_t qmaxFrame = 0;
        uint64_t crossIn = 0, crossOut = 0, passesCrossed = 0;
        uint32_t lastCrossFrame = 0;
        uint64_t crossOwn[6] = {};   // crossings by their own move: none, 1.01, 1.1, 1.41, 2, more
        uint64_t heldOutGap[5] = {}; // held tiles crossing out, the cut's smallest kept over theirs
        uint32_t printed[2] = {};    // crossing lines printed from frame 400 and from frame 1060
    } m_motion;
    // F (HIERARCHY 4.19, the order's third and fourth keys made one): the size of a tile's texel on
    // its reader's screen, texel / max(distance from the eye to its nearest point, texel), the
    // larger the sooner; its bucket is -floor(log2) of it. A parent's texel is twice its child's at
    // no greater distance, so its bucket is never after its child's (the selftest holds it).
    static float Measure(float texel, float dist);
    static uint32_t MeasureBucket(float measure);
    std::vector<OrdRec> m_rec;
    struct PendNote {   // a mark made before its tile is tracked (Want tracks right after)
        int tenant = -1;
        size_t idx = 0;
        OrdRec r;
    } m_pend;
    int m_predSid = -1;
    uint32_t m_predLead = 0;                  // 1a: frames ahead the prediction places the eye
    uint32_t m_sampLast[kMaxSamplers] = {};   // the frame each reader last spoke
    uint32_t m_pinSpoke = 0;                  // ... the latest a standing reader spoke
    // F12: THE LATEST TURN ANY READER SPOKE (the prediction apart: its statement is an interval).
    // A reader silent in a turn when another spoke has said nothing: its statement stands no more.
    uint32_t m_lastSpoke = 0;
    uint32_t m_stFrame[kMaxSamplers] = {};    // a reader's statement: its frame and its hash
    uint64_t m_stHash[kMaxSamplers] = {}, m_stHashLatest[kMaxSamplers] = {};
    std::vector<std::pair<int, TileRequest>> m_failedKeys;   // failed or not whole, for the pass
    void OrderNote(Tenant& t, Tracked* tr, int tenant, int sid, uint32_t stampFrame, uint32_t face,
                   uint32_t m, uint32_t x, uint32_t y, size_t idx, float nearM, float fu, float fv);
    void RecApply(OrdRec& r, int sid, uint32_t stampFrame, uint32_t wbits) const;
    void RecAdd(Tenant& t, int tenant, Tracked* tr);
    void RecDrop(Tracked* tr);
    void Rescue(OrderTurnLedger& L);
    // The straddling rung's boundary in metres from the eye (decision 2's line).
    uint32_t m_cutRung = kNoBucket, m_keptFar = 0, m_lostNear = 0, m_keptFarKey = 0, m_lostNearKey = 0;
    float m_cutKeptM[kRungs] = {}, m_cutLostM[kRungs] = {};   // F: by rung, metres from the eye
    uint64_t m_keptFarTile = 0, m_lostNearTile = 0;
    uint32_t m_cutUnitPlanes = 0, m_cutUnitMeas = 0;   // F18: the pivot's tile let go whole: its kept planes, their measure
    // Decision 5: the events that recompute the order, and the pass's own snapshot of them.
    void OrderPass(OrderTurnLedger& L);
    std::string TileName(uint64_t key) const;
    void StarveWatch();   // the watchdog's turn (ResidencyOrder.cpp, after the loader)
    size_t m_loaderStop = 0;   // the m_need index the loader stopped at (the queue at capacity)   // "tenant [slice] mip (x,y)" from MakeKey's bits
    // H2 (HIERARCHY 4.19, the law completed): A TILE THAT IS HELD COUNTS FOR THE MARGIN TIMES ITS
    // MEASURE, and so does every tile above a held one (the held set stays closed upward: a parent
    // is never after its held child). The pass's pieces, static so the selftest drives them:
    static float PassMeasure(float texel, uint32_t weightBits, bool countsHeld, float margin);
    // kUpBit on every record above a held one, up to the first held ancestor; parentOf gives the
    // record of the nearest tracked ancestor, or UINT32_MAX.
    static void MarkAboveHeld(std::vector<OrdRec>& rec, uint32_t (*parentOf)(const void*, uint32_t),
                              const void* ctx);
    static uint32_t ParentRecInSlots(const void* ctx, uint32_t i);   // ctx: the manager
    // The cut: kInPBit on the first `cut` records of the order (count: records a bucket; mid:
    // scratch, the straddling bucket's records, its first `keep` kept). The straddling bucket.
    // `units`: the tenants (a bit each) whose faces are planes of one tile (F18); `unitPlanes`
    // counts the kept planes let go so the straddling tile goes whole.
    static uint32_t CutOrder(std::vector<OrdRec>& rec, const std::vector<uint32_t>& count,
                             uint32_t cut, std::vector<uint32_t>& mid, uint32_t& keep,
                             uint16_t units = 0, uint32_t* unitPlanes = nullptr);
    // F18: the tie's key. For a tenant whose faces are one tile the face is the LAST word of the
    // address, so a tile's planes stand together in the order and one tile at most straddles a cut.
    static uint64_t TieKey(uint64_t key, uint16_t units) {
        if (!((units >> (key >> 56)) & 1u)) return key;
        const uint64_t face = (key >> 48) & 0xFFu;
        return (key & 0xFF00000000000000ull) | ((key & 0x0000FFFFFFFFFFFFull) << 8) | face;
    }
    uint16_t UnitTenants() const {
        uint16_t m = 0;
        for (size_t i = 0; i < m_tenants.size() && i < 16; ++i) if (m_tenants[i].planesOneTile) m |= uint16_t(1u << i);
        return m;
    }
    static bool OrderSelfTest();
    uint64_t m_trackEpoch = 0, m_failEvents = 0, m_claimEvents = 0;
    uint64_t m_passEpoch = ~0ull, m_passFails = ~0ull, m_passClaims = ~0ull;
    bool m_passHold = false;
    uint32_t m_passFrame = 0, m_passCut = 0, m_nextExpiry = 0;
    // The want's tail past the cut, by tenant and rung, from the last pass; CountTail at any cut.
    uint32_t m_tailTenant[16] = {}, m_tailRung[kRungs] = {}, m_tailTotal = 0, m_wantTotal = 0;
    void CountTail(uint32_t cut, uint32_t* byTenant, uint32_t* byRung, uint32_t& tail,
                   uint32_t& wanted) const;
    void FlushNullMaps(Gpu& gpu, std::vector<std::pair<int, D3D12_TILED_RESOURCE_COORDINATE>>& nulls);
    static constexpr uint32_t kMarkFrame = 200;   // the frame law 8's table is taken from
    uint32_t m_markFrame = 0;
    std::vector<std::string> m_marks;

    // The audit's memory between turns, and its count of one applied invalidation (called with
    // the tile the address holds, or null, before the drop; ResidencyAudit.cpp).
    AuditLedger m_audit;
    void AuditInvalidation(int tenant, const TileRequest& r, const Tracked* tr);
};

class ShaderCompiler;
// The residency manager's own gate (ResidencyTest.cpp): the floor law held to its arithmetic on
// the CPU -- sound on the cube's faces and a window, the kernels' taps covered, porch_floor.py's
// numbers reproduced, each check seen to catch a planted defect -- and, against the law in
// effect, on this GPU through the engine's own samplers (shaders/ResidencyFloor.hlsl). Logs
// [restest]; true when every instrument works and sees what it exists to see. Whether a floor
// could be staged is a finding it prints, measured afresh every run.
bool RunResidencySelfTest(Gpu& gpu, ShaderCompiler& sc, const std::wstring& shaderDir);

}  // namespace ga
