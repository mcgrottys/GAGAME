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
//  FAR BOXES (docs/BUILDING_LOD.md, step 3): past the detail cells, the size-stratified pyramid
//  (compose/BuildingLod.h) streams the same way. Level k's tiles are wanted within its REACH,
//  rho0 2^k / (lodPixels * pixAng) -- the span-over-distance measure of the globe's LeafWants, so a
//  thing is drawn while it covers lodPixels -- with pixAng the camera's vertical field over the
//  viewport's height, and dropped past kKeep x reach. Each tile's boxes are built on the pool (the
//  ground asked under each centroid, its east/north/up taken from the same Place as the prisms)
//  and drawn as 36 vertices a box. A box is SKIPPED where its detail cell is resident: the records
//  are sorted by detail cell, so a tile near the eye draws the runs whose cell is not, and a tile
//  beyond every resident cell draws whole. No building is drawn twice; one still building shows as
//  its box until its cell lands.
// ================================================================================================
#pragma once

#include "compose/BuildingLod.h"
#include "compose/BuildingSolids.h"
#include "scene/Layer.h"

#include <atomic>
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
    // The far boxes' pyramid (null = none) and the law's pixel count.
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
    static constexpr int kFarInFlight = 4;

    // Mirrors `struct BuildingBox` in shaders/Buildings.hlsl: a moment box about its tile's origin.
    struct Box {
        float c[3], pad0;   // centre, metres from the tile's origin, flat frame
        float u[3], pad1;   // the long half-axis (direction x half-extent)
        float v[3], pad2;   // the short half-axis
        float w[3], pad3;   // the up half-axis
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
    };
    using FarKey = std::tuple<int, int, int>;   // (level, tx, ty)
    struct Run {                        // boxes [first, first + count) of one detail cell
        Key cell;
        uint32_t first = 0, count = 0;
    };
    struct FarBuilt {
        FarKey key;
        double origin[3];
        uint32_t count = 0;             // boxes
        std::vector<Run> runs;
        GpuBuffer staging, vb;
        double ms = 0.0;
    };
    struct Shared {                     // what a pool job and the layer both hold
        std::mutex mx;
        std::vector<Built> done;
        std::vector<FarBuilt> farDone;
        std::atomic<int> inflight{0}, farInflight{0};
        std::atomic<bool> cancel{false};
    };
    struct Cell {
        double origin[3];
        GpuBuffer vb;
        uint32_t count = 0;
    };
    struct Retired {
        GpuBuffer buf;
        uint64_t frame;
    };
    // Distance (m) from a point to the cell's box over the sphere's surface, the eye's height
    // folded in: the one measure that both wants and drops.
    double Reach(const Key& k, double latDeg, double lonDeg, double h) const;
    // ...of any box of `deg` with its south-west corner at (lon0, lat0).
    static double ReachBox(double lon0, double lat0, double deg, double latDeg, double lonDeg, double h);
    void WantFar(Gpu* gpu, double latDeg, double lonDeg, double h, double pixAng);
    void UploadFar(const FrameContext& ctx);
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
    };
    std::map<Key, Warm> m_warm;
    uint64_t m_warmBytes = 0;
    std::set<Key> m_pending;
    std::vector<Retired> m_retired;
    uint64_t m_frame = 0;
    uint64_t m_rewarmed = 0, m_built = 0;
    Key m_eyeCell{INT32_MIN, INT32_MIN};   // the instrument: cells taken back warm vs built

    struct Far {
        double origin[3];
        GpuBuffer vb;
        uint32_t count = 0;
        std::vector<Run> runs;
        bool nearDetail = true;   // within reach of a resident detail cell: drawn run by run
    };
    std::shared_ptr<const BuildingLodFile> m_lod;
    double m_lodPixels = 1.0;
    std::map<FarKey, Far> m_far;
    std::set<FarKey> m_farPending;
    uint64_t m_farBuilt = 0;
    uint64_t m_farBoxesDrawn = 0, m_farTilesDrawn = 0;   // the last frame's, for the instrument
    int m_farLevelsLogged = -1;
};

}  // namespace ga
