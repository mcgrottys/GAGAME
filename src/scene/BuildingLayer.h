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
//  THE FOLDED TREE (docs/BUILDING_LOD.md, compose/BuildingLod.h): everything past what the detail
//  cells draw. Its pages stream on the pool (each node's and building's box stood on the composed
//  ground there, once), and a WALK of the loaded tree -- on the pool, again whenever the eye has
//  moved, the view turned, a page landed or a detail cell came or went -- writes the boxes to draw:
//  a node under kQuadPixels across draws its fold; else its own buildings one by one where each
//  covers lodPixels (their fold where they do not) and its children are walked, or, where a child's
//  page is not loaded yet, the fold of its descendants while the page is asked for, the largest on
//  screen first. Every choice is per node at its own distance: no ring and no tile edge. A box
//  whose detail cell is resident is not drawn (the cell's prisms are the building). The walk's
//  boxes are one buffer and one draw, about the eye the walk stood at.
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
    static constexpr int kPageInFlight = 6;
    static constexpr double kQuadPixels = 4.0;          // a node's fold is drawn below this width
    // A fold is drawn over its mass's spread, its footprints' coverage c shrinking it by c^kFoldShrink:
    // 1/2 keeps the footprint's area (a lattice of gaps in a dense city, which aliases), 0 fills the
    // spread as one mass at its roof height (what a dense block is, seen from far and low).
    static constexpr double kFoldShrink = 0.0;
    // ...and it is drawn with its coverage as its alpha, far to near: a fold shows roof over the share
    // of its spread its footprints cover and the ground through the rest, so its colour is the patch's
    // own average, not a slab of roof.
    static constexpr uint64_t kPageBytes = 1024ull << 20;   // the loaded pages' budget (CPU)
    static constexpr uint32_t kMaxBoxes = 2000000;

    // Mirrors `struct BuildingBox` in shaders/Buildings.hlsl: a moment box about the walk's origin.
    struct Box {
        float c[3], alpha;  // centre, metres from the walk's origin, flat frame; the share it covers
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
    // THE TREE IN MEMORY: a page's nodes and buildings, their boxes already stood on the ground.
    struct RtBox {
        double p[3];                    // centre, flat frame
        float u[3], v[3], w[3];         // half-axes
        float alpha = 1.0f;             // a fold's coverage: 1 for a building
        bool valid = false;
    };
    struct RtNode {
        int x, y;
        uint32_t own0, ownN;            // its own buildings in the page's list
        float rhoMin;
        double c[3];                    // the quad's centre on the ground, flat frame
        float rad;                      // a sphere about it holding the quad and its solids
        RtBox own, desc, all;
        Key ownCell, descCell, allCell; // the detail cell each fold's centroid stands in
    };
    struct RtBld {
        RtBox box;
        Key cell;
        float rho;
    };
    struct Page {
        int L = 0;
        std::vector<RtNode> nodes;
        std::map<std::pair<int, int>, uint32_t> at;   // (x, y) -> node
        std::vector<RtBld> blds;
        uint64_t bytes = 0;
    };
    using PageKey = std::tuple<int, int, int>;   // (level, px, py)
    struct Walked {                     // a walk's answer
        double origin[3];
        uint32_t count = 0;
        GpuBuffer staging, vb;
        std::vector<std::pair<double, PageKey>> wants;   // pages asked for, by pixels (largest first)
        std::vector<PageKey> used;
        uint64_t nodes = 0, folds = 0, singles = 0;
        double ms = 0.0;
    };
    struct Shared {                     // what a pool job and the layer both hold
        std::mutex mx;
        std::vector<Built> done;
        std::vector<std::pair<PageKey, std::shared_ptr<Page>>> pagesDone;
        std::vector<Walked> walked;
        std::atomic<int> inflight{0}, pageInflight{0}, walking{0};
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
    };
    std::map<Key, Warm> m_warm;
    uint64_t m_warmBytes = 0;
    std::set<Key> m_pending;
    std::vector<Retired> m_retired;
    uint64_t m_frame = 0;
    uint64_t m_rewarmed = 0, m_built = 0;
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
        uint32_t count = 0;
    } m_drawn;
    // The last walk's inputs: a new one starts when they have moved.
    double m_walkEye[3] = {1e30, 1e30, 1e30}, m_walkFwd[3] = {0, 0, 0};
    uint64_t m_walkCells = ~0ull, m_walkPages = ~0ull;
    uint64_t m_walks = 0;
};

}  // namespace ga
