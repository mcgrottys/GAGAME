// ================================================================================================
//  BuildingLayer - the composed building SOLIDS (compose/BuildingSolids.h), drawn as prisms: walls
//  from the ring's edges, a flat roof ear-clipped from its rings, each solid stood on the composed
//  ground at the lowest point of its footprint (OSM's datum for `height`).
//
//  A SEPARATE LAYER, by the owner's decision (2026-10-09): the solids never enter earth.height, so
//  the water solver's bed (which IS earth.height) does not see them. The water will meet the same
//  solids later through its own sparse voxels; nothing here is the picture's private copy of them.
//
//  STREAMED BY CELL ABOUT THE EYE. A cell is kCellDeg square (the harvests' own index, ~4 x 5.5 km
//  at 43 N). A cell is WANTED while its nearest point lies within `radius` of the eye -- the ground
//  distance and the eye's altitude together, so an eye in orbit wants none -- and DROPPED past
//  kKeep x radius, so an eye on a cell's edge does not make it flicker. A dropped cell is not
//  freed: it goes WARM, its buffer kept (oldest first out past kWarmBytes), and wanted again it is
//  drawn the same frame -- zooming out and back in rebuilt every cell (Mark saw the pop). Wanted cells
//  not warm are built nearest first, at most kInFlight at once, on the thread pool's Io lane: the stack composed for
//  the cell's box (BuildingStack::Compose, its own file handles), the ground asked under every
//  footprint, the prisms triangulated, and the cell's two buffers MADE THERE too (a staging buffer
//  filled, the cell's own empty): creating a 12 MB committed resource on the frame cost it up to
//  25 ms (measured, 2026-10-09). The frame takes at most kUploadsPerFrame finished cells and only
//  records the copy ON ITS OWN COMMAND LIST (no wait on the GPU); a dropped cell's buffers are
//  freed kRetireFrames later, when no frame in flight can still read them.
//
//  PRECISION, the vessel layer's law: every vertex is built in DOUBLES in the flat frame, then
//  stored as a float offset from its cell's origin (a few km at most: a float step under 0.5 mm).
//  At Render each cell's origin is taken relative to the eye in doubles and only that difference is
//  cast. One draw a cell, its offset in the constants.
//
//  Drawn with no culling: a footprint's winding is the file's, and a solid's inside is occluded by
//  its own walls anyway. Procedural fetch by SV_VertexID, as every layer here (no input assembler).
//
//  THE RANKED TREE (docs/BUILDING_LOD.md, compose/BuildingLod.h): everything past what the detail
//  cells draw. The tree files a building by its size (level L holds radii in (Q/8, Q/4] of its quad
//  Q), so its levels ARE the rank: the coarse pages carry the largest buildings and stream first,
//  the smaller ones fill in between. Its pages stream on the pool (each building's box stood on the
//  composed ground there, once), and a WALK of the loaded tree -- on the pool, again whenever the
//  eye has moved, the view turned, a page landed or a detail cell came or went -- writes the boxes:
//  ONE LAW PER BUILDING, at its own distance: its diameter in pixels s against lodPixels t, drawn
//  with alpha (s - t) / t up to 1 (it fades in, it does not pop). A node whose largest possible
//  building is under t is not walked (nor anything under it: all smaller). No fold, no ring, no
//  tile edge. A box whose detail cell is resident is not drawn (the cell's prisms are the
//  building). The walk's boxes are one buffer and one draw, about the eye the walk stood at.
// ================================================================================================
#pragma once

#include "compose/BuildingLod.h"
#include "compose/BuildingSolids.h"
#include "scene/Layer.h"

#include <atomic>
#include <chrono>
#include <unordered_map>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <tuple>
#include <vector>

namespace ga {

class BuildingLayer : public Layer {
public:
    // A point of the planet (degrees, metres on the height datum) in the flat frame, doubles.
    using Place = std::function<void(double latDeg, double lonDeg, double h, double out[3])>;
    // ...and back: a flat-frame point's latitude, longitude and height above the sphere.
    using Locate = std::function<void(const double flat[3], double& latDeg, double& lonDeg, double& h)>;
    // The composed ground (metres on the height datum) under a point, degrees. Called from pool
    // threads.
    using Ground = std::function<double(double latDeg, double lonDeg)>;

    ~BuildingLayer() override;
    void Configure(const std::wstring& shaderDir, std::shared_ptr<const BuildingStack> stack, double radiusM,
                   Place place, Locate locate, Ground ground);
    // The folded tree (null = none) and the pixels a building needs to be drawn on its own.
    void ConfigureFar(std::shared_ptr<const BuildingLodFile> lod, double pixels);

    const char* Name() const override { return "buildings"; }
    void Init(Gpu& gpu, ShaderCompiler& sc, FieldSet& fields, hal::RootSignature rootSig) override;
    void ReloadShaders(Gpu& gpu, ShaderCompiler& sc) override;
    void Simulate(const FrameContext& ctx) override;   // the streaming: want, build, upload, drop
    void Render(const FrameContext& ctx) override;

    size_t ResidentCells() const { return m_cells.size(); }

    // Mirrors `struct BuildingVertex` in shaders/Buildings.hlsl (priors 22).
    struct Vertex {
        float pos[3];   // metres from the cell's origin, flat frame
        float kind;     // 0 wall, 1 roof (+2 for a building:part)
        float n[3];     // outward unit normal, flat frame
        float pad;
    };
    static constexpr double kCellDeg = 0.05;
    static constexpr double kKeep = 1.25;
    static constexpr int kInFlight = 6, kUploadsPerFrame = 4;
    static constexpr uint64_t kWarmBytes = 512ull << 20;   // dropped cells' buffers kept for a return
    static constexpr uint64_t kRetireFrames = 4;
    // TWO STREAMS, each with its own slots (the owner, 2026-10-10): the COARSE one sends only the
    // coarsest level still wanted, so the whole view's coarse pass lands first and nothing finer can
    // queue ahead of it; the FINE one sends every other wanted page, the largest on screen first,
    // beside it (the detail cells are the fine stream's too, on their own kInFlight).
    static constexpr int kCoarseInFlight = 4, kFineInFlight = 4;
    static constexpr uint64_t kPageBytes = 1024ull << 20;   // the loaded pages' budget (CPU)
    static constexpr uint32_t kMaxBoxes = 2000000;
    // THE SHAPES (GALOD04, compose/BuildingShape.h): a loaded page's footprints in one GPU pool,
    // extruded by Buildings.hlsl MsShape one TASK at a time -- kTaskEdges walls or kTaskTris roof
    // triangles of one building (MIRRORED there).
    static constexpr uint64_t kPoolBytes = 768ull << 20;
    // THE POOL IS SLOTS OF ONE SIZE: a page takes whole slots, so "does it fit" is a count of free
    // slots and freeing can never fragment. A slot holds the largest record many times over (the
    // Tokyo tree's largest is 5.3 KB); a record over a slot keeps its box, and is counted.
    static constexpr uint32_t kSlotBytes = 64u << 10;
    static constexpr uint32_t kSlots = static_cast<uint32_t>(kPoolBytes / kSlotBytes);
    static constexpr uint32_t kTaskEdges = 32, kTaskTris = 64;

    // Mirrors `struct BuildingBox` in shaders/Buildings.hlsl: a moment box about the walk's origin.
    struct Box {
        float c[3], alpha;  // centre, metres from the walk's origin, flat frame; its fade-in
        float u[3], pad1;   // the long half-axis (direction x half-extent); pad1 its page's landing
        float v[3], pad2;   // the short half-axis
        float w[3], pad3;   // the up half-axis
        // A SHAPED instance is the same 64 bytes: c the ground under the centroid, u v w the unit
        // east, north and up there, pad2 its record's byte address in the shape pool.
    };

private:
    bool BuildPso(Gpu& gpu, ShaderCompiler& sc);

    using Key = std::pair<int, int>;   // (ix, iy): the cell [ix, ix+1) x [iy, iy+1) of kCellDeg
    struct Built {                      // a cell finished on a pool thread, waiting for a frame
        Key key;
        double origin[3];
        uint32_t count = 0;             // vertices
        GpuBuffer staging, vb;          // made and filled on the pool (the device is free-threaded)
        std::vector<size_t> drawn;      // per source of the stack
        double ms = 0.0;
        bool stale = false;             // the eye left its reach before it was built: nothing made
    };
    // THE TREE IN MEMORY: a page's nodes and buildings, their boxes already stood on the ground.
    struct RtBox {
        double p[3];                    // centre, flat frame
        float u[3], v[3], w[3];         // half-axes
        bool valid = false;
    };
    struct RtNode {
        int x, y;
        uint32_t own0, ownN;            // its own buildings in the page's list
        double c[3];                    // the quad's centre on the ground, flat frame
        float rad;                      // a sphere about it holding the quad and its solids
        bool kids;                      // anything stands under it
        // THE DESCENDANTS' FOLD: the spread of everything under it, drawn at its coverage while the
        // children's page is on its way -- what stands there is never a hole, only coarser.
        RtBox desc;
        float descCover = 0.0f;
        Key descCell{INT32_MIN, INT32_MIN};   // the detail cell its centroid stands in
    };
    struct RtBld {
        RtBox box;
        Key cell;
        float rho;
        // Its shape: the record's PACKED address -- its slot's ordinal in the page times kSlotBytes,
        // plus its offset inside that slot (UINT32_MAX: none, the box is drawn) -- its counts, and the
        // ground frame it stands in (the ground under the centroid).
        uint32_t shapeOff = UINT32_MAX;
        uint16_t nV = 0, nT = 0;
        double g[3] = {};
        float e[3] = {}, n[3] = {}, up[3] = {};
    };
    struct Page {
        int L = 0;
        std::vector<RtNode> nodes;
        std::unordered_map<uint64_t, uint32_t> at;   // QuadKey(x, y) -> node
        static uint64_t QuadKey(int x, int y) { return (uint64_t(uint32_t(x)) << 32) | uint32_t(y); }
        // The finest level's pages: per PARENT quad, the largest radius among its four children's own
        // buildings -- so a parent skips them all with one lookup when even that is under a pixel.
        std::unordered_map<uint64_t, float> parentRho;
        std::vector<RtBld> blds;
        uint64_t bytes = 0;
        GpuBuffer shapeStaging;         // its shape records packed into whole slots, staged on the pool thread
        std::vector<uint32_t> slotUsed; // bytes used in each of its slots (no record crosses a slot)
        std::vector<uint32_t> slots;    // the pool slots holding them once it landed (empty: no shapes drawn)
        std::vector<uint8_t> packed;    // the same bytes on the CPU: what a lost device's report reads back
        uint32_t tooBig = 0;            // records larger than a slot: those buildings keep their boxes
        float landed = 0.0f;            // the layer's clock (s) when it joined the tree: its boxes fade in from here
    };
    using PageKey = std::tuple<int, int, int>;   // (level, px, py)
    struct DrawCpu {                    // a walk's shaped instances and tasks, as the GPU got them
        std::vector<Box> inst;
        std::vector<uint32_t> tasks;    // two uints a task: instance, code
    };
    struct Walked {                     // a walk's answer
        double origin[3];
        uint32_t count = 0;             // boxes, then insts shaped instances, then tasks (one buffer)
        uint32_t insts = 0, tasks = 0;
        GpuBuffer staging, vb;
        std::vector<std::pair<double, PageKey>> wants;   // pages asked for, by pixels (largest first)
        std::vector<PageKey> used;
        std::vector<std::shared_ptr<const Page>> hold;   // its pages: their pool ranges stay while it is drawn
        std::shared_ptr<const DrawCpu> cpu;
        uint64_t nodes = 0, singles = 0, fading = 0;
        uint64_t cellsVersion = 0;      // the detail cells it skipped were these
        double msVisit = 0.0, msSort = 0.0, msBuf = 0.0;   // the walk's three phases, for its log line

        double ms = 0.0;
    };
    struct Shared {                     // what a pool job and the layer both hold
        std::mutex mx;
        std::vector<Built> done;
        std::vector<std::pair<PageKey, std::shared_ptr<Page>>> pagesDone;
        std::vector<Walked> walked;
        std::atomic<int> inflight{0}, pageInflight{0}, walking{0};
        // THE WALK'S BUFFERS, REUSED: a walk takes a retired one at least its size before it makes one.
        std::vector<GpuBuffer> freeStaging, freeBoxes;
        std::atomic<bool> cancel{false};
        // THE EYE, as the frame last saw it (under mx): a cell build asks it whether its cell is still
        // within the keep reach, before and while it builds, and stops when it is not.
        double eyeLat = 0.0, eyeLon = 0.0, eyeH = 0.0, keepM = 0.0;
    };
    struct Cell {
        double origin[3];
        GpuBuffer vb;
        uint32_t count = 0;
    };
    struct Retired {
        GpuBuffer buf;
        uint64_t frame;
        int pool = 0;   // 0 freed; 1, 2: back to the walk's staging / box free list (Shared)
    };
    // Distance (m) from a point to the cell's box over the sphere's surface, the eye's height
    // folded in: the one measure that both wants and drops.
    double Reach(const Key& k, double latDeg, double lonDeg, double h) const;
    // ...of any box of `deg` with its south-west corner at (lon0, lat0).
    static double ReachBox(double lon0, double lat0, double deg, double latDeg, double lonDeg, double h);
    void TreeFrame(const FrameContext& ctx, double latDeg, double lonDeg, double h);
    void LoadPage(const PageKey& k);
    void Want(Gpu* gpu, double latDeg, double lonDeg, double h);
    void Upload(const FrameContext& ctx);

    std::wstring m_shaderDir;
    std::shared_ptr<const BuildingStack> m_stack;
    double m_radius = 3000.0;
    Place m_place;
    Locate m_locate;
    Ground m_ground;
    hal::RootSignature m_rootSig = nullptr;
    hal::Pso m_pso, m_psoBox;
    std::shared_ptr<Shared> m_shared = std::make_shared<Shared>();
    std::map<Key, Cell> m_cells;
    struct Warm {
        Cell cell;
        uint64_t frame;   // when it went warm: the oldest leaves first
        // THE HAND-OFF: the cells version its leaving made. Until a walk at least that new is on
        // screen (m_drawnCells), the boxes drawn still skip this cell, so its prisms stay drawn.
        uint64_t leftAt = 0;
    };
    std::map<Key, Warm> m_warm;
    uint64_t m_warmBytes = 0;
    std::set<Key> m_pending;
    std::vector<Retired> m_retired;
    uint64_t m_frame = 0;
    uint64_t m_rewarmed = 0, m_built = 0, m_staleBuilds = 0;
    Key m_eyeCell{INT32_MIN, INT32_MIN};   // the instrument: cells taken back warm vs built

    std::shared_ptr<const BuildingLodFile> m_lod;
    double m_lodPixels = 1.0;
    std::map<PageKey, std::shared_ptr<const Page>> m_pages;
    std::map<PageKey, uint64_t> m_pageUsed;   // the frame a walk last used it
    std::set<PageKey> m_pagePending;
    uint64_t m_pageBytes = 0, m_pagesLoaded = 0;
    uint64_t m_cellsVersion = 0, m_pagesVersion = 0;   // what a walk depends on, besides the eye
    struct Drawn {
        double origin[3];
        GpuBuffer vb;
        uint32_t count = 0, insts = 0, tasks = 0;
        std::shared_ptr<const DrawCpu> cpu;   // its instances and tasks, kept for a lost device's report
    } m_drawn;
    // THE SHAPES' DRAW IN CHUNKS of kChunkGroups mesh groups, one DispatchMesh each: DRED then names
    // the chunk the GPU stopped in, and DumpChunk names its buildings from the CPU's copy.
    static constexpr uint32_t kChunkGroups = 16384;
    struct DrawLog { uint64_t frame = 0; std::shared_ptr<const DrawCpu> cpu; };
    DrawLog m_drawLog[3];
    void DumpChunk(uint32_t k) const;
    std::set<PageKey> m_drawnUsed;      // the pages the drawn walk reads: not evicted under it
    // THE SHAPE POOL: one buffer, a heap SRV (StructuredBuffer<float4>), its free ranges.
    Gpu* m_gpu = nullptr;
    GpuBuffer m_pool;
    uint32_t m_poolSrv = UINT32_MAX;
    std::vector<uint32_t> m_freeSlots;   // the pool's free slots (a stack)
    struct SlotsRetired {   // a dropped page's slots: free once no walk holds the page, then kRetireFrames on
        std::vector<uint32_t> slots;
        uint64_t frame;
        std::weak_ptr<const Page> page;
    };
    std::vector<SlotsRetired> m_slotsRetired;
    // Pages read and staged but waiting for slots: they land in order as room comes, and while any
    // waits, no new page is asked for -- nothing is loaded that cannot land.
    std::vector<std::pair<PageKey, std::shared_ptr<Page>>> m_roomWait;
    uint64_t m_roomWaitFrames = 0, m_tooBig = 0;
    uint32_t m_walkTasksMax = 0;   // the largest walk's mesh tasks so far (each new largest is logged)
    std::vector<std::shared_ptr<const Page>> m_drawnHold;   // the drawn walk's pages (their slots wait on them)
    hal::Pso m_psoShape;
    uint64_t m_drawnCells = 0;          // the cells version the drawn boxes were walked against
    // THE LAYER'S CLOCK (s): a page's landing and the frame's now, for the boxes' fade-in.
    std::chrono::steady_clock::time_point m_t0 = std::chrono::steady_clock::now();
    float Now() const { return std::chrono::duration<float>(std::chrono::steady_clock::now() - m_t0).count(); }
    static constexpr float kPageFadeS = 0.6f;   // a page's buildings come in over this, not in one frame
    // The last walk's inputs: a new one starts when they have moved.
    double m_walkEye[3] = {1e30, 1e30, 1e30}, m_walkFwd[3] = {0, 0, 0};
    uint64_t m_walkCells = ~0ull, m_walkPages = ~0ull;
    uint64_t m_walks = 0;
    // The pages the last walk asked for, coarsest level first (the two streams read it each frame).
    std::vector<std::pair<double, PageKey>> m_wants;
};

}  // namespace ga
