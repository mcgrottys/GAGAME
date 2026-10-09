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
//  kKeep x radius, so an eye on a cell's edge does not make it flicker. Wanted cells are built
//  nearest first, at most kInFlight at once, on the thread pool's Io lane: the stack composed for
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
// ================================================================================================
#pragma once

#include "compose/BuildingSolids.h"
#include "scene/Layer.h"

#include <atomic>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <set>
#include <string>
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
    static constexpr int kInFlight = 2, kUploadsPerFrame = 2;
    static constexpr uint64_t kRetireFrames = 4;

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
    struct Shared {                     // what a pool job and the layer both hold
        std::mutex mx;
        std::vector<Built> done;
        std::atomic<int> inflight{0};
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
    void Want(Gpu* gpu, double latDeg, double lonDeg, double h);
    void Upload(const FrameContext& ctx);

    std::wstring m_shaderDir;
    std::shared_ptr<const BuildingStack> m_stack;
    double m_radius = 3000.0;
    Place m_place;
    Locate m_locate;
    Ground m_ground;
    hal::RootSignature m_rootSig = nullptr;
    hal::Pso m_pso;
    std::shared_ptr<Shared> m_shared = std::make_shared<Shared>();
    std::map<Key, Cell> m_cells;
    std::set<Key> m_pending;
    std::vector<Retired> m_retired;
    uint64_t m_frame = 0;
};

}  // namespace ga
