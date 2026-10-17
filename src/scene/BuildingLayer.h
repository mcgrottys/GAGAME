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
#include "core/OriginLattice.h"
#include "sim/PlaceField.h"
#include "compose/BuildingSolids.h"
#include "scene/Layer.h"
#include "scene/Air.h"   // the walk's air: what the atmosphere leaves of a building's contrast
#include "compose/Compositor.h"   // ComposedSurfaceCb: the sun map's rows on b2

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
    // THE GROUND, AT A GRAIN (2026-10-11): a field of the composed height at the grain asked for
    // (compose/HeightPage: the pyramid's texels, memoised), made per job -- a page's buildings stand
    // on the ground at the page's own level, a ribbon's profile and a near cell's prisms at a metre.
    // The stack sampled raw at a metre for every footprint vertex cost 140 us a sample where the
    // rasters were cold (Haulover after a carry: 1.4 s a page, the stream one page at a time).
    using Ground = std::function<std::unique_ptr<PlaceField>(double resM)>;

    ~BuildingLayer() override;
    void Configure(const std::wstring& shaderDir, std::shared_ptr<const BuildingStack> stack, double radiusM,
                   Place place, Locate locate, Ground ground);
    // The folded tree (null = none) and the pixels a building needs to be drawn on its own.
    // THREE FORMS BY SIZE (the owner, 2026-10-15): under boxPixels across, a building is a POINT --
    // one vertex carrying the building's rotor, so the eye's elevation against its up mixes roof
    // and wall and the point is lit as the building would be; from boxPixels its moment box; from
    // shapePixels its own form. Every form is opaque: a fade between forms, or in as a page
    // lands, left transparent blocks over the imagery, and the owner saw them.
    void ConfigureFar(std::shared_ptr<const BuildingLodFile> lod, double pixels, double shapePixels = 8.0,
                      double pageMB = 1024.0, double marginDeg = 5.0, double boxPixels = 2.0, double standMB = 256.0);

    const char* Name() const override { return "buildings"; }
    void Init(Gpu& gpu, ShaderCompiler& sc, FieldSet& fields, hal::RootSignature rootSig) override;
    void ReloadShaders(Gpu& gpu, ShaderCompiler& sc) override;
    void Simulate(const FrameContext& ctx) override;   // the streaming: want, build, upload, drop
    void Render(const FrameContext& ctx) override;
    // A SECOND EYE'S WALK (the minimap's, FrameLoop::MinimapFrame): the tree walked from that eye's
    // own camera and rectangle, keyed by ctx.viewIndex, its boxes drawn when Render comes with that
    // index. The pages, the pool and the budget are the layer's, shared by every eye; what one eye
    // asks for the streams send, what any eye's drawn walk reads is kept. Its result lands in the
    // next Simulate, with the first eye's.
    void SimulateEye(const FrameContext& ctx);
    static constexpr uint32_t kEyes = 2;   // the session's own and the minimap's (ViewContext index)
    // THE WORLDS A VIEW REACHES THROUGH ITS WINDOWS (2026-10-11, the owner: the portal owns the
    // residency of what it shows). An eye's walk is its own world's; each window world the view
    // reaches is walked from the eye carried through that window (its camera in that world's own
    // frame) and drawn pulled back through the window's motor, kept only where the window shows that
    // depth (Buildings.hlsl PsMain, the hulls' rule). The frame loop says each eye's worlds every
    // frame (FrameLoop::PublishBuildingWorlds); kWorlds - 1 windows deep at most -- the corridor
    // beyond shows a window too small to hold a building.
    struct WorldEye {
        double cam[3] = {0.0, 0.0, 0.0};   // the eye in that world's frame (root coordinates)
        double fwd[3] = {0.0, 0.0, 1.0};
        Motor pull;                        // that world drawn in the true frame
    };
    static constexpr uint32_t kWorlds = 3;
    void SetWorlds(uint32_t eye, std::vector<WorldEye> worlds) { if (eye < kEyes) m_worlds[eye] = std::move(worlds); }
    // THE OCCLUDER (Renderer::Occluder; the owner, 2026-10-11): the eye's farthest drawn depth by
    // cell, two frames back, with the camera it was drawn by. A node whose nearest point lies beyond
    // every cell its sphere covers is hidden -- by the ground, the sea, a hull or a nearer building,
    // whatever the frame drew -- and asks for nothing. A window world's node is pulled into the true
    // frame first. Null: no test.
    struct Occluder {
        uint32_t w = 0, h = 0;
        std::vector<float> dist;
        double eye[3] = {0.0, 0.0, 0.0}, fwd[3] = {0.0, 0.0, 1.0}, right[3] = {1.0, 0.0, 0.0}, up[3] = {0.0, 1.0, 0.0};
        double tanX = 1.0, tanY = 1.0, nearZ = 0.25;
    };
    void SetOccluder(uint32_t eye, std::shared_ptr<const Occluder> o) { if (eye < kEyes) m_occ[eye] = std::move(o); }
    // THE CONE'S MARGIN (the owner: widen what is loaded a couple of degrees around the view, so a
    // turn finds its buildings asked for and stood): the walk's cone is the view's half-diagonal
    // plus this. layers.buildings.marginDeg.
    double m_marginDeg = 5.0;

    size_t ResidentCells() const { return m_cells.size(); }

    // THE SUN MAPS: A CASCADE ON THE ORIGIN LATTICE (scene render.buildingShadowM, buildingShadowLevels;
    // shaders/TerrainRay.hlsli SunMapClear; docs/VISIBILITY_FIELD.md 9). The solids this layer draws
    // for the eye's own world are drawn once more each frame per LEVEL, depth only, as the SUN sees
    // them: an orthographic cast along the sun over a cube the span of one CELL of the origin lattice
    // (core/OriginLattice.h) -- the eye's ground address at the finest level whose cells are no wider
    // than `sideM`, then that cell's parents, the span doubling each level, `levels` of them. Every
    // level has kSunTexels a side, so the grain grows with the cell as a pixel's footprint grows with
    // its distance from the eye. A cell's map stands on the cell's own point, lifted to the ground
    // under the eye (`eyeClearM`: the frame's own clamp, no read) in steps of a sixteenth of the
    // span, a world quantity: it is the cell's, not the eye's, so it has no hysteresis and no
    // snapping -- an eye moving within its cell moves no shadow edge, and the set of cells changes
    // only when the eye's address does. A map is KEPT across frames and recast only when its cell,
    // its altitude step, the sun (by a texel's worth), the drawn walk or the near cells change; the
    // maps are dropped while the walk draws nothing that casts (orbit). PlanSunMap (the frame loop,
    // after the surface rows are filled) plans the casts and writes b2's rows (ComposedSurfaceCb::
    // shadow.w, sunMapU, sunSrv, sunVP); Simulate records the passes due, one ExecuteIndirect walk
    // per level. A lit point takes the FINEST map that holds it (the floor of its address). sideM
    // <= 0 or levels <= 0: no maps, the rows say none, nothing is drawn.
    // `view`: the instrument (render.buildingShadowView) -- the faces drawn as their maps' clear.
    void PlanSunMap(const Camera& cam, const float sunDir[3], double sideM, int levels, bool view, double eyeClearM, ComposedSurfaceCb& cb);
    static constexpr uint32_t kSunTexels = 2048;
    static constexpr uint32_t kSunLevels = 6;   // the cascade's cap: b2's rows (ComposedSurfaceCb::sunSrv, sunVP)
    // THE AIR IN THE WALK'S LAW (2026-10-15; the owner: cull what the atmosphere obscures). A
    // building is drawn when it covers lodPixels of the picture -- but what the picture holds of it
    // is its contrast against the ground, and the air between takes that contrast: a face at range
    // r keeps T(r) of it (Common.hlsli AerialPerspective: col T + L). So the walk's threshold is
    // lodPixels / T: a building 100 km off in clear air must be six pixels wide to be worth a pixel,
    // and the far field's nodes, the most numerous, stop where the air says they are not seen.
    // T is bounded from above (the least extinction the path meets, at its highest point -- the
    // eye's altitude above a city), so the law can only keep what the air leaves, never cut what
    // it does not. The planet's rows are the scene's (scene/Air.h); set each frame by the loop.
    void SetAir(const AirRows& a) { m_air = a; }
    // The maps as pictures (--dump writes them beside the still, one per level: X.sunmap0.png ...):
    // depth toward the sun as grey, 0 where the sun meets nothing. An instrument; it waits for the GPU.
    void DumpSunMap(const std::wstring& path);

    // THE DECKS THE PHYSICS READS (2026-10-12): every bridge deck and ramp ribbon (compose/
    // BuildingShape.h kDeck) in the pages resident about the eye whose polyline comes within rM of
    // the place -- its vertices in degrees with the profile each stands over the higher of the
    // ground and the sea (metres), and its width. The frame thread's (the pages are its); the
    // entity turns them into solids in its own frame (sim/Solids.h).
    struct DeckStrip {
        std::vector<double> lat, lon, over;   // per vertex
        double widthM = 0.0;
    };
    void DecksNear(double latDeg, double lonDeg, double rM, std::vector<DeckStrip>& out) const;
    // ...and a count that moves whenever the resident page set does, so a reader re-asks then.
    uint64_t PagesVersion() const { return m_pagesVersion; }

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
    uint64_t m_pageBudget = 1024ull << 20;   // the loaded pages' budget (CPU): layer.buildings.pageMB
    // THE RECORDS A WALK MAY NAME (the classify pass's threads, and its buffer's size at 48 bytes each).
    static constexpr uint32_t kMaxRecords = 2000000;
    // THE SHAPES (GALOD04, compose/BuildingShape.h): a loaded page's footprints in one GPU pool,
    // extruded by Buildings.hlsl MsShape one TASK at a time -- kTaskEdges walls or kTaskTris roof
    // triangles of one building (MIRRORED there).
    static constexpr uint64_t kPoolBytes = 768ull << 20;
    // THE POOL IS SLOTS OF ONE SIZE: a page takes whole slots, so "does it fit" is a count of free
    // slots and freeing can never fragment. A slot holds the largest record many times over (the
    // Tokyo tree's largest is 5.3 KB); a record over a slot keeps its box, and is counted.
    static constexpr uint32_t kSlotBytes = 64u << 10;
    static constexpr uint32_t kSlots = static_cast<uint32_t>(kPoolBytes / kSlotBytes);
    // THE STREAMS (Buildings.hlsl AsShape): an amplification group takes kAsInsts instances and
    // launches a mesh group for every kStream edges or roof triangles of them, end to end.
    static constexpr uint32_t kAsInsts = 64, kStream = 64, kBoxesPerGroup = 16;

    // ONE INSTANCE of the walk (Buildings.hlsl ReadInst), 48 bytes, three float4 rows: the centre
    // and its fade-in; THE ATTITUDE AS ONE ROTOR (east, north and up are q acting on the flat
    // frame's axes -- one object, not three unit vectors); and either a box's three half-extents
    // with its page's landing, or a shape's record address (an integer) with its landing.
    struct Box {
        float c[3], alpha;   // the centre (a box) or the ground under the centroid (a shape)
        float q[4];          // the rotor taking flat x, y, z to east, up, north: (x, y, z, w)
        float x[4];          // a box: a1, a2, hz, landed; a shape: asuint(addr), landed, 0, 0
    };
    static_assert(sizeof(Box) == 48, "three float4 rows");
    // A STAND'S RECORD ON THE GPU (shaders/BuildingsClassify.hlsl; 2026-10-15, the owner: the walk
    // O(log n), not O(N)). One a building, written ONCE when the stand lands, into a persistent
    // pool, stated about the stand's own centre (a translator per node, so the record is small
    // floats wherever the node stands and the walk's origin can move without touching it); the
    // walk names a node's range, and the classify pass makes a Box of it for the form its size on
    // screen earns. The same object as a Box plus what the per-building law needs: its size (rho:
    // 0 claims nothing), the centroid's height over the ground (zc: the shape's ground point),
    // its shape's pool address, its heading (the box's rotor is the ground frame's turned by it)
    // and its detail cell. No fold is ever a record: a building is a point, a hexahedron or its
    // mesh, and a page that has not landed shows nothing (the owner, 2026-10-15).
    struct StoodRec {
        float p[3], rho;     // the box's centre about the stand's centre; its radius
        float q[4];          // the box's rotor (flat x, y, z to its long axis, up, its short axis)
        float a1, a2, hz, zc;
        uint32_t addr;       // the shape record's byte address in the shape pool; UINT32_MAX none
        float heading;       // radians, the long axis from east toward north
        uint32_t cell;       // the detail cell (iy << 16 | ix as int16s)
        float landed;        // the page's landing (the layer's clock, s)
    };
    static_assert(sizeof(StoodRec) == 64, "four float4 rows");
    // A NODE OF A WALK (two float4 rows): what the walk emits per visited node that has something
    // to draw -- O(nodes), never O(buildings) -- expanded on the GPU.
    struct NodeEntry {
        float off[3];        // the stand's centre about the walk's origin (the difference taken in doubles)
        uint32_t poolOff;    // its first record in the stands' pool
        uint32_t first;      // the node's first thread in the pass (a running sum over the list)
        uint32_t count;      // its threads: the buildings within the node's size bound
        float air;           // 1 / T at the node's nearest point: every threshold's factor there
        uint32_t pad;
    };
    static_assert(sizeof(NodeEntry) == 32, "two float4 rows");
    static constexpr uint32_t kWalkHeader = 8;   // the walk buffer's header rows (Buildings.hlsl)

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
    using PageKey = std::tuple<int, int, int>;   // (level, px, py)
    struct RtBox {
        double p[3];                    // centre, flat frame
        float u[3], v[3], w[3];         // half-axes
        bool valid = false;
    };
    struct RtNode {
        int x, y;
        uint32_t own0, ownN;            // its own buildings in the page's list
        double c[3];                    // the quad's centre AT SEA LEVEL, flat frame (the stand has the ground's)
        float rad;                      // a sphere about it holding the quad and its solids
        bool kids;                      // anything stands under it
        // THE FOLDS, as the tree filed them: its own buildings' and its descendants' -- the node's
        // size bound and its sphere. Never drawn (the owner, 2026-10-15): a building is its own
        // point, hexahedron or mesh, and a page on its way shows nothing until it lands.
        LodBox own{}, desc{};
    };
    // A node whose stand has not come is tested at the worst of elevations: its sphere grown by
    // this, so a town on a plateau is never culled before its ground is known.
    static constexpr double kUnstoodM = 9000.0;
    struct RtBld {
        Key cell;
        float rho;
        // Its shape: the record's PACKED address -- its slot's ordinal in the page times kSlotBytes,
        // plus its offset inside that slot (UINT32_MAX: none, the box is drawn) -- and its counts.
        uint32_t shapeOff = UINT32_MAX;
        uint16_t nV = 0, nT = 0;
    };
    // A STAND (2026-10-11, the owner's rule: what is loaded and stood is by proximity and view,
    // nowhere by page). A page lands with nothing on the ground: its nodes at sea level, its
    // buildings as records. The walk, on reaching a node, asks for its stand -- the node's own and
    // descendants' folds on the composed ground, and its buildings when the walk has come close
    // enough to draw them -- at the grain the node's distance earns, and a pool job makes it. A
    // stand is published whole under an atomic pointer, so a walk reads a stand or none, never half
    // of one, and a finer stand replaces a coarser without a race. The cost then follows the view:
    // a page 88 km wide over a cold raster stood its 3,466 buildings in 4.7 s before this.
    struct StoodFrame {                   // the ground frame under a building's centroid
        double g[3] = {};                 // its point...
        float e[3] = {}, n[3] = {}, up[3] = {};   // ...east, north, up
    };
    // A stand's buildings live on the GPU (StoodRec, the pool m_stood): the stand holds the quad's
    // centre for the walk's node tests and its RANGE in the pool. Published when its records are
    // copied (Simulate), so a walk that reads a stand reads records the GPU holds.
    struct Stood {
        double c[3] = {};                 // the quad's centre on the ground
        uint32_t gpuOff = 0, gpuN = 0;    // its records: one a building (withBlds; none otherwise)
        float grainM = 0.0f;
        bool withBlds = false;
    };
    struct StandAsk {                     // what a walk asks: a node's stand, this fine, with its buildings or not
        PageKey page;
        uint32_t node = 0;
        bool blds = false;
        double grainM = 1.0;
        double px = 0.0;                  // its worth on screen: the order the stands are made in
        uint32_t gpuOff = 0, gpuN = 0;    // the pool range StandBatch gave it (the frame thread's)
    };
    // THE POOL'S RANGES: free runs of records by offset and by length, taken best-fit, given back
    // coalesced -- a stand of three records beside one of three thousand, no slot size to waste.
    struct Ranges {
        std::map<uint32_t, uint32_t> free;        // offset -> length
        std::multimap<uint32_t, uint32_t> byLen;  // length -> offset
        uint32_t total = 0, used = 0;
        void Reset(uint32_t n);
        bool Take(uint32_t n, uint32_t& off);
        void Give(uint32_t off, uint32_t n);
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
        std::vector<LodBuilding> raw;   // the buildings as the tree filed them: a stand reads these
        mutable std::unique_ptr<std::atomic<std::shared_ptr<const Stood>>[]> stood;   // one a node
        uint64_t bytes = 0;
        GpuBuffer shapeStaging;         // its shape records packed into whole slots, staged on the pool thread
        std::vector<uint32_t> slotUsed; // bytes used in each of its slots (no record crosses a slot)
        std::vector<uint32_t> slots;    // the pool slots holding them once it landed (empty: no shapes drawn)
        double waitMs = 0.0, readMs = 0.0;   // the instrument: its job's wait in the lane, its read
        std::vector<uint8_t> packed;    // the same bytes on the CPU: what a lost device's report reads back
        uint32_t tooBig = 0;
        float landed = 0.0f;            // the layer's clock (s) when it joined the tree: its boxes fade in from here
    };
    // A walk's draw buffer: the header and the three lists the classify pass fills. Born COMMON
    // (fresh); after its pass it is in kListRead, and stays so through the free list.
    struct ListBuf {
        GpuBuffer b;
        bool fresh = true;
    };
    static constexpr D3D12_RESOURCE_STATES kListRead = D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE |
                                                       D3D12_RESOURCE_STATE_INDIRECT_ARGUMENT | D3D12_RESOURCE_STATE_COPY_SOURCE;
    struct Walked {                     // a walk's answer
        double origin[3];
        float eo[3] = {};               // the eye about the origin: the classify pass's distances
        float pixAng = 0.0f;            // a pixel's angle from that eye (rad): the pass's sizes on screen
        uint32_t named = 0, cap = 0;    // nodes named, records they send (the pass's threads)
        uint32_t cellsN = 0;            // the resident detail cells listed after the nodes
        GpuBuffer list;                 // the node list (upload heap: read by the pass as it is)
        ListBuf out;                    // the header and the lists, kWalkHeader + 3 cap rows
        std::vector<std::shared_ptr<const Stood>> stands;   // the stands named: their ranges stay until the pass ran
        std::vector<std::pair<double, PageKey>> wants;   // pages asked for, by pixels (largest first)
        std::vector<StandAsk> asks;     // the stands asked for, the largest on screen first
        std::vector<PageKey> used;
        std::vector<std::shared_ptr<const Page>> hold;   // its pages: their pool ranges stay while it is drawn
        uint64_t nodes = 0, singles = 0, hidden = 0;   // singles: buildings sent to the pass; hidden: nodes the occluder cut
        uint64_t aired = 0;   // nodes the air cut: under the threshold through it, not without it
        uint64_t cellsVersion = 0;      // the detail cells it skipped were these
        double msVisit = 0.0, msList = 0.0, msBuf = 0.0;   // the walk's three phases, for its log line
        double msRoots = 0.0, msDrain = 0.0;               // ...and the visit's: the coarse levels, the subtrees' drain (the rest is the merge)
        uint32_t subtrees = 0, threads = 0;
        uint32_t eye = 0;               // whose walk it is: the walker's id (Eye::id), wherever it stands now

        double ms = 0.0;
    };
    struct Shared {                     // what a pool job and the layer both hold
        std::mutex mx;
        std::vector<Built> done;
        std::vector<std::pair<PageKey, std::shared_ptr<Page>>> pagesDone;
        std::vector<Walked> walked;
        // A stand made on the pool: its page (held), and a ribbon's heights for the pool's slots.
        struct StandDone {
            std::shared_ptr<const Page> page;
            PageKey key{INT32_MIN, 0, 0};
            uint32_t node = 0;
            std::vector<std::pair<uint64_t, std::vector<uint8_t>>> zWrites;   // (the pool's byte offset, the bytes)
            std::shared_ptr<const Stood> stood;   // the stand, published by Simulate once its records are copied
            std::vector<uint8_t> recs;            // its records (StoodRec each), for the pool at stood->gpuOff
            double ms = 0.0;
            uint32_t blds = 0;
        };
        std::vector<StandDone> standDone;
        std::atomic<int> inflight{0}, pageInflight{0}, standInflight{0};
        std::atomic<int> walking[kEyes * kWorlds] = {};   // one walk in flight per walker, by its id
        // THE WALK'S BUFFERS, REUSED: a walk takes a retired one at least its size before it makes one.
        std::vector<GpuBuffer> freeStaging;
        std::vector<ListBuf> freeBoxes;
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
    // The walk launched for one walker (an eye's world), from an eye and a forward, when its inputs
    // have moved and none of its is running.
    void Walk(const FrameContext& ctx, uint32_t walker, const double eye[3], const double fwd[3], double latDeg, double lonDeg, double h);
    // Every world of an eye walked: its own from the context's camera, each window world from the
    // eye carried through the window.
    void WalkEye(const FrameContext& ctx, uint32_t eye);
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
    hal::Pso m_pso, m_psoBox, m_psoPoint;   // the cells' prisms, the boxes by vertex, the points
    // THE SUN MAPS' pass: the same four pipelines depth-only, and per level of the cascade the
    // texture, its views, the cell and the cast. A level's texture is made when the scene first asks
    // for that many levels (kSunTexels square, D32: 16 MB each).
    hal::Pso m_psoSun, m_psoSunBox, m_psoSunBoxMesh, m_psoSunShape;
    // A LEVEL'S MAP IS KEPT until one of its inputs moves (the still frame's law, docs/VISIBILITY_FIELD.md
    // 9): the cell, the cube's altitude step, the sun by a texel's worth, the walk the eye draws, the
    // near cells. The cast's own axes and centre are held with it, so the lookup's matrix is the
    // kept map's and never a newer sun's.
    struct SunLevel {
        bool on = false;                   // the rows name it this frame
        bool cast = false;                 // its map must be drawn (an input moved since it was)
        bool stood = false;                // a cell is known from a frame before
        int lattice = 0;                   // the lattice level whose cell this is
        OriginLattice::Origin cell{};      // the eye's ground address at that level
        double alt = 0.0;                  // the cube's altitude: the ground under the eye, in steps of a sixteenth of the span
        double S[3] = {}, R[3] = {}, U[3] = {};   // the cast's sun axes (down-sun is -S)
        double centre[3] = {};             // the cube's centre, flat frame
        uint64_t walks = 0, cells = 0;     // the walk and the near cells the map was cast from
        float side = 0.0f;                 // the cell's span (m): the cube's every edge
        float vp[16] = {};
        GpuTexture tex;
        uint32_t dsv = UINT32_MAX, srv = UINT32_MAX;
    };
    SunLevel m_sun[kSunLevels];
    uint32_t m_sunLevels = 0;   // how many levels stand this frame
    uint32_t m_sunContent = 0;  // maps due to be drawn anew, counted: the kept frame's key (sunMapU[3])
    bool m_sunSaid = false, m_sunPlanSaid = false;
    bool EnsureSunLevel(uint32_t k);   // the level's texture and views, made once
    AirRows m_air = EarthAir();
    void RenderSunMap(const FrameContext& ctx);
    // THE SOLIDS OF ONE EYE, drawn through the pipelines given: the resident cells (and the ones
    // leaving whose boxes the drawn walk does not hold yet), then each of `worlds` worlds' boxes
    // and shapes -- the frame's draw (Render, three worlds, lit) and the sun's (RenderSunMap, the
    // eye's own world, depth only, once per level of the cascade) are the one walk over the one set
    // of buffers.
    struct Pipes {
        ID3D12PipelineState* cells = nullptr;
        ID3D12PipelineState* box = nullptr;
        ID3D12PipelineState* boxMesh = nullptr;
        ID3D12PipelineState* shape = nullptr;
        ID3D12PipelineState* point = nullptr;   // null: the points are not drawn (the sun's pass)
    };
    void DrawSolids(const FrameContext& ctx, uint32_t eyeIndex, uint32_t worlds, const Pipes& p, const char* tag);
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
    bool m_cellsSaid = false;   // the near cells' state, said once
    Key m_eyeCell{INT32_MIN, INT32_MIN};   // the instrument: cells taken back warm vs built

    std::shared_ptr<const BuildingLodFile> m_lod;
    double m_lodPixels = 1.0, m_shapePixels = 8.0, m_boxPixels = 2.0;
    std::map<PageKey, std::shared_ptr<const Page>> m_pages;
    std::map<PageKey, uint64_t> m_pageUsed;   // the frame a walk last used it
    std::set<PageKey> m_pagePending;
    std::map<PageKey, uint64_t> m_pendingSince;   // the instrument: the frame each pending page was asked
    uint64_t m_pageBytes = 0, m_pagesLoaded = 0, m_evictions = 0;
    uint64_t m_cellsVersion = 0, m_pagesVersion = 0;   // what a walk depends on, besides the eye
    struct Drawn {
        double origin[3];
        GpuBuffer vb;                   // the walk's buffer: its header holds the counts and the draws' arguments
        uint32_t named = 0, cap = 0;    // what the walk sent to the GPU (its log line)
    };
    // A lost device: DRED names the dispatch the GPU stopped in; this says what each walker's drawn
    // walk sent (the instances themselves are the GPU's, made by the classify pass).
    void DumpChunk(uint32_t k) const;
    // THE SHAPE POOL: one buffer, a heap SRV (StructuredBuffer<float4>), its free ranges.
    Gpu* m_gpu = nullptr;
    GpuBuffer m_pool;
    // THE STANDS' RECORD POOL (StoodRec): the buildings stood, written once; its ranges; the ranges
    // of stands replaced or dropped, freed once nothing holds them and no pass in flight reads them.
    GpuBuffer m_stood;
    uint32_t m_stoodRecords = 0;        // layers.buildings.standMB / 64
    Ranges m_stoodFree;
    struct StoodRetired {
        uint32_t off, n;
        uint64_t frame;
        std::weak_ptr<const Stood> stood;
    };
    std::vector<StoodRetired> m_stoodRetired;
    uint64_t m_stoodFull = 0;           // stands that waited for room (the instrument)
    // THE CLASSIFY PASS (shaders/BuildingsClassify.hlsl): its root, its kernel, and the two command
    // signatures the forms are drawn through (a mesh dispatch, a draw) from the buffer's header.
    hal::RootSignatureRef m_clsRs;
    hal::Pso m_clsPso;
    hal::CommandSignatureRef m_sigMesh, m_sigDraw;
    void Classify(const FrameContext& ctx, Walked& w);
    // THE COUNTS, BROUGHT BACK: the header's first row copied to a ring the CPU reads kCountsLag
    // frames on, so the walk's log line can say what the GPU drew of what it sent.
    static constexpr uint32_t kCountsRing = 16, kCountsLag = 3;
    GpuBuffer m_countsRb;
    struct CountsPending {
        uint64_t frame = 0;
        uint32_t slot = 0, eye = 0;     // eye: the walker's id
        uint64_t walk = 0;
        uint32_t named = 0, cap = 0;
        double ms = 0.0;
    };
    std::vector<CountsPending> m_countsPending;
    uint32_t m_countsNext = 0;
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
    // THE LOADS' INSTRUMENT: what landed since the last line, and how long each waited and read.
    uint32_t m_landN = 0;
    double m_landWait = 0.0, m_landWaitMax = 0.0, m_landRead = 0.0, m_landReadMax = 0.0;
    hal::Pso m_psoShape, m_psoBoxMesh;   // AsShape + MsShape; MsBox (m_psoBox is VsBox: no mesh shaders)
    // THE LAYER'S CLOCK (s): a page's landing and the frame's now, for the boxes' fade-in.
    std::chrono::steady_clock::time_point m_t0 = std::chrono::steady_clock::now();
    float Now() const { return std::chrono::duration<float>(std::chrono::steady_clock::now() - m_t0).count(); }
    static constexpr float kPageFadeS = 0.6f;   // a page's buildings come in over this, not in one frame
    // ONE EYE'S WALK (keyed by ctx.viewIndex: the session's own, the minimap's). THE WALK'S ORIGIN
    // (core/OriginLattice.h): the planet's cloud's origin at the eye -- its shell and direction --
    // kept by the lattice's hysteresis. Every instance of a walk is stated about it, not about the
    // eye, so a camera moving inside the cell moves one float3 in the constants and not a buffer of
    // instances; the walk itself is asked again only when the eye has moved 2 % of its height.
    struct Eye {
        uint32_t id = 0;                // the walker's own name: its walks and their counts come home to it
        OriginLattice::Origin walkOrigin{};
        bool walkOriginSet = false;
        // The last walk's inputs: a new one starts when they have moved.
        double walkEye[3] = {1e30, 1e30, 1e30}, walkFwd[3] = {0, 0, 0};
        uint64_t walkCells = ~0ull, walkPages = ~0ull;
        uint64_t walks = 0;
        Drawn drawn;
        uint32_t lastDrawn[3] = {};     // boxes, shapes, points the GPU counted for its latest walk read back
        std::set<PageKey> drawnUsed;    // the pages the drawn walk reads: not evicted under it
        std::vector<std::shared_ptr<const Page>> drawnHold;   // the drawn walk's pages (their slots wait on them)
        uint64_t drawnCells = 0;        // the cells version the drawn boxes were walked against
        // The pages the last walk asked for, coarsest level first (the two streams read it each frame).
        std::vector<std::pair<double, PageKey>> wants;
        std::vector<StandAsk> stands;   // the stands it asked for, the largest on screen first
    } m_eyes[kEyes * kWorlds];          // the walker drawing (eye, world) at eye * kWorlds + world
    static uint32_t WalkerOf(uint32_t eye, uint32_t world) { return eye * kWorlds + world; }
    // Where the walker of id `id` stands now (its slot); a walk lands there, wherever it was asked.
    uint32_t SlotOf(uint32_t id) const {
        for (uint32_t s = 0; s < kEyes * kWorlds; ++s) if (m_eyes[s].id == id) return s;
        return 0;
    }
    std::vector<WorldEye> m_worlds[kEyes];
    std::shared_ptr<const Occluder> m_occ[kEyes];
    // What the occluder hid, the last walk of each walker (the instrument).
    uint64_t m_hidden[kEyes * kWorlds] = {};
    // THE STANDS' STREAM: every eye's asks, each node once at the finest grain and with buildings if
    // any eye wants them, the largest on screen first; a job stands a batch of kStandBatch nodes,
    // kStandInFlight jobs at a time (a warm stand is 0.1 ms, a cold one 2.5: thousands a frame, or
    // hundreds). A walk asks for its kStandAskMax most worth on screen; the rest it asks again when
    // those have landed. A landing re-walks the eye at most every kStandWalkFrames frames: a city's
    // tree is walked a few times a second while its stands flow, not every frame.
    static constexpr int kStandInFlight = 8, kStandBatch = 64, kStandWalkFrames = 15;
    static constexpr size_t kStandAskMax = 4096;
    std::set<std::pair<PageKey, uint32_t>> m_standPending;
    uint64_t m_lastStandWalk = 0;
    // The stream's order: every eye's asks merged and sorted when a walk lands, sent from the cursor.
    std::vector<StandAsk> m_standQueue;
    size_t m_standNext = 0;
    bool m_standQueueStale = false;
    void StandBatch(std::vector<std::pair<std::shared_ptr<const Page>, StandAsk>> batch);
    // A box stood on the composed ground at its centroid, on the prisms' own axes; the shape's own
    // frame (the ground under the centroid, east, north, up) beside it when asked.
    static void StandBox(const Place& place, double lat, double lon, double base, double zc, double hz, double a1,
                         double a2, double hd, RtBox& o, StoodFrame* frame);
    // A stood box as its GPU record, about the stand's centre (the rotor and extents the walk made
    // of it before; rho what it claims on screen, 0 for nothing).
    static void RecordOf(const RtBox& b, const double c[3], float rho, uint32_t addr, float heading, uint32_t cell,
                         float landed, float zc, StoodRec& r);
    uint64_t m_stands = 0;
    double m_standMs = 0.0, m_streamMs = 0.0;   // the jobs' time on the pool; the stream's on the frame thread (the instrument)
    // Every eye's wants as one list, each page once at the most it is worth to any eye.
    std::vector<std::pair<double, PageKey>> Wants() const;
    bool DrawnUses(const PageKey& k) const;
};

}  // namespace ga
