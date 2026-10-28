// ================================================================================================
//  SweSolver - M5c: the sparse shallow-water solver. The estuary stops being a picture.
//
//  THE WATER AS COLUMNS (WATER_V2 section 12, C1, 2026-10-23). A shallow-water solve over the
//  stack's bed on staggered signed face fluxes (shaders/Swe.hlsl: Stelling & Duinmeijer's momentum
//  advection, the hydrostatic face depth, positivity by the donor's water), its state the water each
//  column HOLDS (m_h, R32F: h >= 0; a NULL tile holds none) over its bed (m_bed, cached a frame from
//  the stack) and the fluxes (two banks: the advection reads the substep before). The tide enters only
//  through the boundary: Flather faces on the open ocean sides, riding the manager's Level along each
//  side (SetLevelSource: the tide atlas), with the domain's stateless prism as their transport; and the
//  river faces, riding the river tide with the rivers' transport. Everything in between -- the throat
//  jet, the basin's lag, the ebb/flood asymmetry, a flat that waits for its water -- emerges.
//
//  Published for the readers, each frame: m_eta (R32F) = the column's surface less the frame's tide
//  plane (the readers add the plane back: WaterBank.hlsl, DeviationAt), and the RGBA16F fibre (u east,
//  v north, h, wet) -- the sea shader takes its current INSTEAD of the analytic Gaussian jet wherever
//  the column is wet, which hands the wave-current blocking physics the real channel's shape.
//
//  Time: the solver advances its own clock toward the scene clock, at most kMaxSubsteps per
//  frame. If the scene clock runs away (a scrub), each column refills to the stateless column (the
//  water the tide plane stands over it) and steps on.
// ================================================================================================
#pragma once

#include "core/GradeField.h"
#include "sim/ImpactField.h"
#include <memory>
#include "core/Lattice.h"
#include "hal/Context.h"
#include "hal/Gpu.h"
#include "hal/Readback.h"
#include "hal/Shader.h"
#include "hal/TileAtlas.h"
#include "hal/Views.h"
#include "sim/RiverModel.h"

#include <algorithm>
#include <cmath>
#include <functional>
#include <string>
#include <vector>

namespace ga {

// M6x: the solver is a WINDOW TYPE now, not the Merrimack. Per-window knobs; defaults preserve the
// validated estuary behavior.
struct SweConfig {
    const char* name = "swe";   // PHASE C3: the scene names it (Assembly: scene.name)
    // THE EDGE IS THE FRONTIER'S LAW (2026-10-27, one water on the tree): every edge cell of the window
    // that no river face holds is the stateless sea's, pinned to the tide atlas's Level along its side
    // (Swe.hlsl OceanSideOf); wet or dry is that Level over the cell's bed, each instant. No side is
    // "open" and none is a wall: a channel the edge cuts fills and drains with the sea, a dune at the
    // edge stands over it. (Until this date a side was open when EVERY cell of it lay under +1.2 m,
    // else only its cells below mean sea were the sea's, and an offshore sponge stood on the open
    // sides: three laws, and a harbour mouth open on 1525 of 2048 cells had no open side.)
    // THE RIVER FACES (2026-10-21): where a river enters the domain -- the crossing of its reach's line
    // with the box's edge (sim/RiverModel.h EntersBox) -- a Flather open boundary on that side, over the
    // run of wet-capable edge cells about the crossing (SweDomain::Bound). Until this date one face
    // stood on the west side of every domain that said `river: west`, the whole column.
    struct RiverFace {
        int side = -1;               // 0 N, 1 S, 2 W, 3 E
        double lat = 0.0, lon = 0.0; // where the river's line crosses the edge
        std::string name;
    };
    // STAGE 2b OF ONE WATER ON THE TREE (2026-10-27): THE TAIL -- a coarse lattice across the ocean about the same
    // anchor, tailCells a side of cells 2^tailRank of the window's, on the window's own lattice (the window sits
    // whole in its central cells); its bed the one stack at that grain (bedAt: lat, lon degrees, the grain m);
    // 0 cells = no tail (water.swe.tailCells, water.swe.tailRank).
    uint32_t tailCells = 2048;
    uint32_t tailRank = 9;
    // STAGE 2c: the anchor levels between the tail and the window -- this many, each telescopeCells a side, their
    // ranks telescopeStep apart below the tail's (water.swe.telescopeLevels / telescopeCells / telescopeStep)
    uint32_t telescopeLevels = 3, telescopeCells = 512, telescopeStep = 2;
    double patchCellsPerLip = 8.0;   // an event's patch: its lip's radius over this many cells (16 = a rank finer: the convergence run)
    std::function<float(double, double, double)> bedAt;
    std::vector<RiverFace> faces;    // at most kMaxRiverFaces
    // C2: THE LAND SOAKS -- one stated soil for now (WATER_V2 12.3): the water a dry column's soil can
    // take (m), its saturated conductivity (m/s: a sandy loam's 36 mm/h) and Green and Ampt's suction
    // times moisture deficit (m). The soil's own measurements (SoilGrids2, HiHydroSoil v2) replace them.
    float soilCapM = 0.10f, soilKs = 1.0e-5f, soilSuctionM = 0.05f;
};

// PHASE C1 (out/integration/plan_phase_c.md): THE SOLVER'S CHART. A domain is a lat/lon box -- since stage
// 2 of one water on the tree (2026-10-26) the window about the eye's start on the global lattice of the
// cell's angle (water.swe.windowTiles, app/Assembly.cpp), no scene's box -- its ANCHOR is the box's
// centre and its lattice a window of its own planes there
// (HIERARCHY 4.4: a ratio of planes about an anchor in doubles). A point P of the body (planet frame,
// any scale) has the cell coordinates
//     tx = (U . P) / (A . P),   ty = (V . P) / (A . P)
// -- the central projection onto the anchor's tangent plane, east and north there as the axes, a cell
// a true dx by dy metres at the anchor (the box's width at its own latitude over nx, its height over
// ny), row 0 north, cell i spanning [i, i + 1). Orthogonal at the anchor and conformal to (r / R)^2
// within the box. Nothing here reads BathyModel's anchor or its frozen metres.
struct SweDomain {
    double lat0 = 0.0, lon0 = 0.0, lat1 = 0.0, lon1 = 0.0;   // the box, degrees
    double latC = 0.0, lonC = 0.0, R = 0.0;                  // its anchor, and the planet's radius
    double A[3] = {}, E[3] = {}, N[3] = {};                  // up, east, north at the anchor
    double U[3] = {}, V[3] = {};                             // the cells' planes (through the centre)
    uint32_t nx = 0, ny = 0;
    double dx = 0.0, dy = 0.0;                               // a cell, metres (true, at the anchor)
    double ox = 0.0, oy = 0.0;   // stage 2c: a level's rectangle on the chart: the centred lattice's cell less this one's
    std::vector<float> elev;      // the bed at the cell centres, m NAVD88, row 0 north (the stack's)
    // THE RIVER FACES' CELLS (Bound): a side and the run [c0, c1] along it (columns for N and S,
    // rows for W and E) of wet-capable edge cells (bed under +2 m) about the river's crossing.
    struct FaceCells {
        int side = -1, c0 = 0, c1 = -1;
        std::string name;
    };
    std::vector<FaceCells> faces;
    size_t EdgeCell(int side, int a) const {   // the cell index of edge cell `a` along `side`
        return side == 0 ? size_t(a) : side == 1 ? size_t(ny - 1) * nx + a : side == 2 ? size_t(a) * nx
                                                                                      : size_t(a) * nx + (nx - 1);
    }
    std::vector<uint8_t> FaceMask() const;   // 1 at every face cell whose bed is under +2 m
    // THE SEA'S EDGE CELL: edge cell `a` of `side` is the stateless sea's exterior (Swe.hlsl OceanSideOf)
    // when it has a bed and no river face holds it; the Level over that bed, each instant, says whether
    // it holds water. One law for every edge cell: no open side, no wall.
    bool SeaEdgeCell(int side, int a) const;
    bool Ready() const { return nx > 0 && ny > 0; }
    void Place(double boxLat0, double boxLon0, double boxLat1, double boxLon1, uint32_t cellsX,
               uint32_t cellsY, double planetR);
    // The cell coordinates of a planet direction or a place; false on the far hemisphere.
    bool CellOfDir(const double P[3], double& tx, double& ty) const;
    bool CellOf(double latDeg, double lonDeg, double& tx, double& ty) const;
    void LatLonOf(double tx, double ty, double& latDeg, double& lonDeg) const;
    bool Holds(double latDeg, double lonDeg) const;   // the lattice holds the place
    // The bed at every cell's centre, from the one stack (bedAt(lat, lon, the cell's grain), degrees).
    void FillBed(const std::function<float(double, double, double)>& bedAt);
    // The river faces' cells, from the bed (elev filled first).
    void Bound(const SweConfig& cfg);
    // THE ROWS A READER CARRIES (WindowRows.hlsli HP_SOLVER_ROWS_DECL): the planes U, V, W about a
    // frame whose origin is `origin` and whose axes are east / up / north (planet frame), over R so
    // their w is the cell at the origin; then (nx, ny, 1 = a solver stands, 0). Zeros: none.
    void KernelRows(const double east[3], const double up[3], const double north[3],
                    const double origin[3], float out[16]) const;
};

// A SOLVER'S WINDOW HOLDS ITS WATER (an instrument, said once where a window is made): the grid
// flooded 4-connected (the kernel's faces) through cells whose bed is under `level`, from its
// boundary -- every edge cell under the level (the sea's: the kernel's one edge law) and the river
// faces' cells (`face`, a mask a cell; SweDomain::FaceMask). Water the sea's flood does not reach is
// listed, largest first: a pool the edge does not touch, or water a river face alone holds.
struct WaterPiece {
    struct EdgeRun {
        char edge;      // 'N', 'S', 'W', 'E'
        int from, to;   // cells along the edge: columns for N and S, rows for W and E
        float deepest;
    };
    int id = -1;                          // its label in the flood's label map
    int cells = 0;
    int c0 = 0, r0 = 0, c1 = 0, r1 = 0;   // its box, cells
    bool face = false;                    // a river face reaches it (so it is held by the face alone)
    std::vector<EdgeRun> runs;            // where it meets the window's edge (a river face's cells only)
};
struct WaterHold {
    int joined = 0;      // cells the sea's flood reaches
    int faceCells = 0;   // the river faces' seeds
    std::vector<WaterPiece> pieces;
    std::vector<uint8_t> sea;   // by label: 1 = the sea's flood reaches it
};
WaterHold FloodWindow(const float* bed, int nx, int ny, float level, const uint8_t* face,
                      std::vector<int>* labels = nullptr);
// The two beds' floods side by side: the cells the grid's joins that the kernel's does not (and
// back), the largest piece of the kernel's water the grid's joins, and where it parts from the
// sea -- the cell of the lowest crest on its way there over the kernel's bed, and that cell's bed
// in each.
struct FloodParting {
    int gridJoined = 0, kernelJoined = 0;   // the cells each bed's flood joins to the sea
    int gridOnly = 0, kernelOnly = 0;
    WaterPiece piece;       // the kernel's piece (cells = those of it the grid joins); id -1 = none
    int crestCell = -1;
    float crestKernel = 0.0f, crestGrid = 0.0f;
};
FloodParting CompareFloods(const float* grid, const float* kernel, int nx, int ny, float level,
                           const uint8_t* face);
void LogWindowHoldsWater(const SweDomain& dom, const SweConfig& cfg, const char* when,
                         const std::vector<float>* kernelBed = nullptr);
bool RunWaterHoldSelfTest();

class SweSolver {
public:
    static constexpr uint32_t kMaxSubsteps = 16;
    // THE FRAME'S STEP BUDGET (2026-10-25, the asteroid's arrival at 30x): a fast clock asks more substeps
    // than kMaxSubsteps a frame and the solver STARVES -- its clock re-anchors each frame and its
    // dynamics run time-dilated, the surge at its faces rising at the scene's rate and the water inside
    // answering at a fraction of it (the beach clip stayed dry through a 600 m surge). A scene says the
    // budget (water.swe.stepsPerFrame); a headless record sets it to what its clock needs.
    void SetStepBudget(uint32_t steps) { m_stepBudget = steps ? steps : kMaxSubsteps; }

    // M9ar: bind the bed -- the height tenant's array with its residency map, so the solver reads
    // the same megatexture the water shading and the globe read. Must be called before the first
    // Step; there is no bed otherwise.
    // PHASE B2 (D4): THE SOLVER'S DOMAIN IS A STANDING WINDOW (SurfaceFrame::StandAbout), held whole
    // before the spin-up. The kernel reads its bed through that window's chain (one entry, rank 2:
    // SurfaceFrame::StandingRows about the domain's ANCHOR, KernelRows' packing in `rows`), each cell's
    // point its place in the anchor's tangent plane carried onto the sphere (small numbers: no large
    // cancellation), at its own cell's grain. PHASE C1: the window stands about the domain's anchor.
    struct BedWindow {
        float rows[80];   // SurfaceFrame::KernelWindowRows, as KernelRows packs it
        uint32_t slice = UINT32_MAX;
    };
    void SetBed(Gpu& gpu, hal::Resource heightArr, hal::Resource resMapArr, uint32_t mips,
                const BedWindow& w);
    bool BedBound() const { return m_bedBound; }

    void Init(Gpu& gpu, ShaderCompiler& sc, const std::wstring& shaderDir,
              const SweDomain& dom, const SweConfig& cfg = {});
    const SweDomain& Domain() const { return m_dom; }
    bool Ready() const { return m_ready; }

    // The river faces' targets, per face: the river tide as a DEVIATION from the ocean tide plane
    // (NAVD m; the truncated upriver prism arrives through the station fits), and the TRANSPORT
    // the Flather face must carry (m^3/s, + into the domain; discharge minus the upriver prism's
    // demand) -- radiation alone cannot supply a prescribed prism, it would throttle behind the
    // standing dEta it needs to sustain the flow. Update per frame.
    int FaceCount() const { return int(m_dom.faces.size()); }
    void SetBoundaries(const float* dEtaM, const float* qM3s) {
        for (int f = 0; f < FaceCount(); ++f) {
            m_faceDEta[f] = dEtaM[f];
            m_faceQ[f] = qM3s[f];
        }
    }

    // C2, AN INSTRUMENT: a surge on the open sea's Level, A exp(-((t - peak) / tau)^2) metres, its rate
    // in the sea's transport (OceanBoundary) -- the scripted surge of WATER_V2 12.5's C2 gate.
    void SetSurge(double ampM, double peakUnix, double tauS) {
        m_surgeA = ampM;
        m_surgePeak = peakUnix;
        m_surgeTau = tauS;
    }
    // C2: THE LEDGER (an instrument). On, the water in through the open faces is summed every substep
    // on the GPU (CsSweLedger); LedgerIn reads it (m^3 since LedgerOn). Released: the water the residency
    // gave up with the tiles it released (it releases only tiles dry past the hysteresis). Volumes: the
    // interior columns' water and their soils' (m^3; the exterior cells are the boundary's).
    void LedgerOn(Gpu& gpu);
    double LedgerIn(Gpu& gpu);
    double LedgerReleased() const { return m_released; }
    void Volumes(Gpu& gpu, double& columns, double& soil, std::vector<float>* hOut = nullptr);
    uint32_t StateTiles() const { return m_h.ResidentCount(); }
    // The sea's Level at the window's edge the last batch was given (m NAVD88): the mean, over the edge
    // cells that Level stands over, of the Level there (the surge included); the tide plane where it
    // stands over none (OceanBoundary).
    double OpenSeaLevel() const { return m_openSea; }
    uint32_t DryWaiting() const;   // resident state tiles holding no water, inside their hysteresis
    float WetDepth() const { return m_cb.wetEps; }   // wet is h > this (m): the readers' scale for a column's water

    // C4: THE SEA AT THE DOMAIN'S OPEN WATER: Hs (m), the peak period (s) and the direction it comes
    // FROM (degrees true) at a place (degrees) and an instant -- the global wave grid (GFS-Wave). The
    // waves' push is the solver's own from it (CsSweWaves); unset or false, no waves push.
    void SetSeaSource(std::function<bool(double, double, double, double&, double&, double&)> seaAt) {
        m_seaAt = std::move(seaAt);
    }

    // THE OPEN SEA'S LEVEL (C1): the manager's Level, m NAVD88, at a place (degrees) and an instant --
    // the tide atlas (EOT20 and the stations). The open sides' exterior cells ride it; unset, they ride
    // the tide plane Record is given.
    void SetLevelSource(std::function<double(double, double, double)> levelAt) { m_levelAt = std::move(levelAt); }
    // STAGE 3 OF ONE WATER ON THE TREE (2026-10-27): the impacts' record. Each event, at the first batch the field
    // stands past its zero, is IMPOSED on the columns at every mip the field holds -- the law's level and radial
    // velocity then (its cavity at T = 0), shoaled to the column's water -- and the law carries it from there
    // (Swe.hlsl CsSweImpose; BeginImpose, RecordImpose). One event a batch; the next batch takes the next.
    void SetImpacts(std::shared_ptr<const ImpactField> impacts) { m_impacts = std::move(impacts); }
    // STAGE 2b: THE TAIL'S OWN EXTERIOR -- the departure from mean sea (m) at a place and instant at the tail's
    // far edge: the impacts' law for events outside the tail (FrameLoop); unset, nothing.
    void SetTailLevelSource(std::function<double(double, double, double)> at) { m_tailLevelAt = std::move(at); }
    bool HasTail() const { return !m_lv.empty(); }
    // STAGE 2c: the levels to the readers of the far field (sim/ImpactGpu.h LevelsGpu; shaders/TailRead.hlsli):
    // their textures, cells and rectangles on the one chart; `exag` the display exaggeration on the drawn departure.
    LevelsGpu LevelsGpuDesc(float exag) const;
    // the levels' lat/lon bounds and the cube's mip each reads (WaterManager's want for their beds), coarsest first
    struct LevelBox { double box[4]; uint32_t mip; };
    std::vector<LevelBox> LevelBoxes() const;

    // Advance toward simUnix (records compute through cmd) and leave eta + uv sampleable.
    // tideNavd = the analytic water level, NAVD88 m. Returns substeps executed this frame.
    int Record(hal::CommandContext& cmd, Gpu& gpu, double simUnix, float tideNavd);

    // Solver-only advancement (own submits; no rendering): integrate up to targetUnix in
    // batches of 64 substeps per command list. tideAt(unix) supplies the ocean boundary level,
    // faceAt(face, unix) the faces' deviations, faceQAt(face, unix) their transports (+ inward).
    template <typename F, typename G, typename Q>
    void AdvanceTo(Gpu& gpu, double targetUnix, F tideAt, G faceAt, Q faceQAt) {
        while (m_simTime < targetUnix - m_dt) {
            // a step under 5 ms on these cells is a blow-up (a face at hundreds of metres a second), and the
            // spin-up would crawl for hours: it stops here and says so (the 10 km stone's crest at the bar, 2026-10-27)
            if (m_dt < 0.005f) {
                Log("[swe] the step collapsed to %.4f s at t=%.0f (%.2f h of the spin-up left): the spin-up stops here", m_dt, m_simTime, (targetUnix - m_simTime) / 3600.0);
                break;
            }
            const double target = (std::min)(m_simTime + 64.0 * m_dt, targetUnix);
            for (int f = 0; f < FaceCount(); ++f) {
                m_faceDEta[f] = static_cast<float>(faceAt(f, m_simTime));
                m_faceQ[f] = static_cast<float>(faceQAt(f, m_simTime));
            }
            hal::CommandContext up(gpu, gpu.BeginUpload(), hal::Owner::Upload);
            Record(up, gpu, target, static_cast<float>(tideAt(m_simTime)), 9999);
            gpu.EndUpload();
            gpu.ResetConstantArenaAfterIdle();   // thousands of batches; EndUpload waited
            AfterBatch(gpu);                     // C2: the water's residency and the step, read now
        }
    }

    // Spin-up: reset to the analytic plane `hours` before startUnix and integrate forward, so
    // the first rendered frame carries real basin history instead of a flat start.
    template <typename F, typename G, typename Q>
    void Spinup(Gpu& gpu, double startUnix, double hours, F tideAt, G faceAt, Q faceQAt) {
        m_simTime = startUnix - hours * 3600.0;
        m_pendingReset = true;
        AdvanceTo(gpu, startUnix, tideAt, faceAt, faceQAt);
    }

    uint32_t EtaSrv() const { return m_eta.Srv(); }    // the column's surface less the tide plane, R32F
    uint32_t UvSrv() const { return m_uvSrv; }         // currents, RGBA16F (slice-0 view)
    uint32_t VelGradSrv() const { return m_velGrad.Srv(); }   // M9h: (div, curl), RG16F
    uint32_t VelGradResMapSrv() const { return m_velGrad.ResidencyMapSrv(); }
    uint32_t VelGradMips() const { return m_velGrad.MipCount(); }
    // M9i: the bank itself, so a COMPOSED page can be written into its coarse levels from
    // outside. The solver deliberately does not know where such a page came from -- that is
    // the compositor'''s business, and the last attempt failed by making this class know.
    GradeBank& VelGradBank() { return m_velGrad; }
    // The residency map is one texel per mip-0 TILE, so the lens indexes it in tile space.
    uint32_t VelGradResMapW() const { return m_velGrad.TilesX(); }
    uint32_t VelGradResMapH() const { return m_velGrad.TilesY(); }
    hal::Resource VelGradRes() const { return m_velGrad.Res(); }
    hal::Resource UvRes() const { return m_uvBank.Res(); }

    uint32_t Nx() const { return m_cb.nx; }
    float CellM() const { return m_cb.dx; }   // level-0 ground size, for a page ladder
    uint32_t Ny() const { return m_cb.ny; }
    uint32_t ResidentTiles() const {
        return m_h.ResidentCount() + m_bed.ResidentCount() + m_eta.ResidentCount() + m_flux.ResidentCount() +
               m_velA.ResidentCount() + m_velB.ResidentCount();
    }
    uint64_t ResidentBytes() const {
        return m_h.Atlas().ResidentBytes() + m_bed.ResidentBytes() + m_eta.ResidentBytes() + m_flux.ResidentBytes() +
               m_velA.ResidentBytes() + m_velB.ResidentBytes();
    }
    // The domain's wet area (m^2) at a level L: the cells whose spill level is under it (Init's priority
    // flood from the open sides and the river faces) -- the hypsometry the open sea's prism comes from.
    double WetAreaAt(double levelNavd) const;
    // THE LEVEL AT AN EDGE CELL (m NAVD88), as the kernel reads it: a river face's exterior level over its
    // run, else the side's cubic through the Level's four samples (Swe.hlsl OceanEtaAt), from the last
    // OceanBoundary. And the state tiles the boundary's water stands on now: a tile holding an edge cell
    // that Level stands over -- wanted by the same rule as any water (UpdateResidency).
    double EdgeLevel(int side, int a) const;
    std::vector<uint8_t> BoundaryTiles() const;
    float PadW() const { return static_cast<float>(m_eta.TilesX() * m_eta.TileW()); }
    float PadH() const { return static_cast<float>(m_eta.TilesY() * m_eta.TileH()); }

    // Synchronous full-texture readbacks for validation probes at CELL coordinates (tx, ty pairs:
    // SweDomain::CellOf). Headless use. One eta + one uv readback serve all points.
    struct Probe {
        float dEta, u, v;   // the column's surface less the tide plane; the current where wet
        bool valid;         // the column is wet (h > 1 cm)
    };
    void ReadProbes(Gpu& gpu, const float* cellPairs, int count, Probe* out);
    // M6x: the full-field CPU MIRROR for the weather manager -- eta (PADDED atlas dims) and
    // the derived currents (exact bathy dims, xyzw = u, v, speed, valid), one readback each.
    // F13: the same fields WITHOUT the drain: Begin submits the two copies and returns; Ready
    // says whether their fence passed; Take unpacks them exactly as ReadFields does. A reader on
    // a cadence (the wave solve's current, every 90 s) asks on one frame and reads on a later one.
    bool ReadFieldsBegin(Gpu& gpu);
    bool ReadFieldsPending() const { return m_rbEta.Pending(); }
    bool ReadFieldsReady(const Gpu& gpu) const;
    // ...and the same read handed to ANOTHER THREAD: Detach gives the two finished readbacks
    // away (the solver forgets them); Unpack maps and unpacks them wherever it is called -- the
    // map and the half-to-float loop are 100 ms the frame need not pay. Only the grid's size is
    // read of the solver, by value, at Detach.
    struct FieldsRead {
        Gpu::TextureReadback eta, uv;
        uint32_t nx = 0, ny = 0;
    };
    FieldsRead ReadFieldsDetach();
    static void UnpackRead(Gpu& gpu, FieldsRead& r, std::vector<float>& etaOut, uint32_t& etaW,
                           uint32_t& etaH, std::vector<float>& uv4Out, uint32_t& uvW, uint32_t& uvH);
    void ReadFields(Gpu& gpu, std::vector<float>& etaOut, uint32_t& etaW, uint32_t& etaH,
                    std::vector<float>& uv4Out, uint32_t& uvW, uint32_t& uvH);
    // Debug: raw readback of the flux bank (padded dims, RGBA32F). outW/outH = padded texels.
    std::vector<uint8_t> ReadFluxRaw(Gpu& gpu, uint32_t* outW, uint32_t* outH, uint32_t* outPitch);
    // The transport through river face `f`, m^3/s (+ into the domain): its Flather faces' flux summed
    // along it -- what the river actually brings in, beside what the face was told to carry
    // (SetBoundaries). A full readback of the flux bank: tools only.
    double FaceTransport(Gpu& gpu, int f);
    // THE FIRST RIVER FACE, read back (an instrument): for each cell along its side, the Flather
    // face's terms as the flux kernel makes them for its next substep on this instant's state
    // (CsSweFlux compiled with SWE_FACE_TRACE: its own expressions, and no flux written). One row
    // per cell along the side; `pinned` is the kernel's own test (the cell is the face's exterior
    // and its bed below +2 m).
    struct FaceRow {
        float etaX, etaI, sill, hf;      // the exterior level, the first interior surface, the
                                         // face's sill and its depth (m NAVD, m)
        float uext, rad, ub, qRaw;       // u_ext, the radiation term, ub (m/s), q before the clamp
        float q, qLo, qHi, pinned;       // q after it (m^3/s, + inward), the clamp's bounds, 1 = pinned
        float dEta0, hI, bed0, bed1;     // the exterior cell's eta, the interior depth, both beds
    };
    bool TraceFace(Gpu& gpu, std::vector<FaceRow>& rows);
    float FaceArea(int f) const { return m_faceArea[f]; }   // the live section u_ext was last divided by

    // ---- THE BED THE KERNEL READS (the bed trace: an instrument, never on the frame path). The
    // solver's own BedAt over its whole lattice, through the rule and the residency map the flux
    // and height kernels read at this instant (Swe.hlsl CsSweBedTrace), read back with the mip
    // the rule read at and the slice it read. The CPU bed (BathyModel) is a different reader of
    // the same stack and cannot see what residency did to this one. `floorMip` raises the rule's
    // residency floor for this read alone (the planted failure: the coarsest mip). It records on
    // its own upload list and waits, so it is never called inside a frame's recording.
    struct BedTrace {
        uint32_t nx = 0, ny = 0;
        std::vector<float> bed;       // m NAVD88, row 0 north, nx * ny
        std::vector<uint8_t> mip;     // the mip the rule read at
        std::vector<uint8_t> slice;   // the slice it read: a cube face 0..5, or the page's
    };
    bool TraceBed(Gpu& gpu, BedTrace& out, float floorMip = 0.0f);
    // A SOLVER'S WINDOW HOLDS ITS WATER, said once: `traced` = the whole-bed wait finished, so the
    // trace reads the kernel's bed back and the line floods that (and the CPU grid's flood beside
    // it where the two differ); otherwise the CPU grid, and the line says the kernel's may differ.
    void LogHoldsWater(Gpu& gpu, bool traced);
    // THE SCHEME'S TESTS (WATER_V2 12.5, C1's analytic judges; an instrument, --tool swe-test): the
    // solver's own kernels on analytic beds in one square of resident tiles, walls elsewhere -- a lake at
    // rest (flat to 1 mm after an hour), a hump on it (the volume ledger to 1e-5), Ritter's dam break
    // onto a dry bed and Stoker's onto a wet one against their solutions. Logs each; true when all pass.
    // The state is reset after (the next Record refills the stateless column on the real bed).
    bool RunSchemeTests(Gpu& gpu);
    uint32_t PageSlice() const { return m_bedSlice; }

    std::string stats;         // "swe 476t 31/54MB 16ss" for the title bar
    double SimTime() const { return m_simTime; }
    float Dt() const { return m_dt; }

    // ---- THE SOLVER, READ WITHOUT A STALL (the water match, step 1 -- Mark, 2026-09-15: the
    // solver is truth). A hull reads the surface the solver holds, not the tide plane it was forced
    // by, and it may not stop the renderer to do it (Gpu::WaitIdle is 18-27 ms: WaterManager.h).
    // So a consumer ASKS for a region -- the eta and current texels within radiusM of a point of
    // the flat frame, plus the one-texel apron the bilinear reconstruction reads -- and Record, on
    // the frame ring only, copies it through hal::RegionReadback; the answer arrives
    // Gpu::kFrameCount frames later, always, and never by a flush. A request lives one frame: a
    // consumer that wants its answer kept fresh asks every frame (the answers stay until newer
    // ones replace them). The upload-list paths (Spinup, AdvanceTo) serve nothing.
    void RequestRegion(double latDeg, double lonDeg, double radiusM);
    // Someone asked for a region this frame: the solver is wanted whether or not any view draws it
    // (SeaLayer::Simulate steps it on this as well as on the drawn sea).
    bool Demanded() const { return !m_requests.empty(); }
    struct Deviation {
        float dEta = 0.0f;           // m, from the tide plane the solver was forced by: the wet columns' surface
        float wet = 0.0f;            // the corners' say, by the water each holds (h / (h + h_wet)): 0 = no water here
        float u = 0.0f, v = 0.0f;    // the solved surface current, m/s east / north
        bool currentValid = false;   // the solver has a current there (its valid channel > 0.5)
        double asOf = 0.0;           // the solver's clock when the texels were copied
    };
    // THE KERNEL'S RECONSTRUCTION, at a point of the flat frame (shaders/WaterBank.hlsl: texel =
    // (u Nx, (1 - v) Ny), centres at -0.5, bilinear, clamped to the grid), from the newest
    // delivered region holding all four texels. False where no delivered region holds them.
    bool DeviationAt(double latDeg, double lonDeg, Deviation& out) const;
    // The same reconstruction over whole-field arrays (the tools' mirror: eta at rows of etaRowW,
    // the current's four channels at rows of uvRowW), so the two transports cannot disagree about
    // the law. False when the arrays are empty.
    bool DeviationFromFields(double latDeg, double lonDeg, const std::vector<float>& eta,
                             uint32_t etaRowW, const std::vector<float>& uv4, uint32_t uvRowW,
                             double asOf, Deviation& out) const;
    // THE DOMAIN'S WEIGHT: the solver's surface owns a point in proportion to how far inside the
    // grid it stands, rising from 0 at the grid's edge to 1 one cell in -- the field is defined at
    // cell centres and has no support of its own nearer the edge than that. The kernel's
    // expression, in the kernel's texel units.
    double DomainWeight(double latDeg, double lonDeg) const;
    // The newest delivered region's clock; a large negative number when none has been delivered.
    double DeliveredAsOf() const { return m_regions.empty() ? -1.0e18 : m_regions.front().asOf; }
    size_t RegionCount() const { return m_regions.size(); }

private:
    int Record(hal::CommandContext& cmd, Gpu& gpu, double simUnix, float tideNavd,
               int maxSub);
    void RecordReset(hal::CommandContext& cmd, D3D12_GPU_VIRTUAL_ADDRESS hList, UINT hCount,
                     D3D12_GPU_VIRTUAL_ADDRESS fList, UINT fCount);
    // C2: after a batch on the upload list (AdvanceTo): the tile statistics read now, the residency and
    // the step from them.
    void AfterBatch(Gpu& gpu);
    // C2: THE WATER'S RESIDENCY from the tile statistics (CsSweTileStats): a state tile stays while a
    // column holds water or water touches its edge from a neighbour (the ring); one dry past the
    // hysteresis is released; the step from the deepest and fastest water.
    void UpdateResidency(Gpu& gpu, const std::vector<uint8_t>& raw, uint32_t pitch);
    // The banks that cover the state's tiles, mapped and released to match it; the derived bank's.
    void CoverState(Gpu& gpu);
    void DeriveGradResidency(Gpu& gpu);
    // The open sides' Level samples and their Flather transport, into m_cb (Record, each batch).
    void OceanBoundary(float tideNavd, float tideRate);
    // The flux bank: the newest substep's fluxes (one bank; the velocities take turns).
    TileAtlas2D& FluxNew() { return m_flux; }
    // M12 step 4b instrument: the [kernel] swe cb fingerprint, logged at every upload when it
    // changes (the gate for the lattice-row moves and the fills of 4e/4f).
    void LogCbFingerprint();

    // Mirrors SweCb in shaders/Swe.hlsl exactly.
    struct SweCbData {
        uint32_t nx, ny, etaTilesX, fluxTilesX;
        uint32_t etaTileW, etaTileH, fluxTileW, fluxTileH;
        uint32_t listCount;
        float unused0;     // the slot the open-side bitmask held (the edge is one law since 2026-10-27); keeps faceA on its register
        float dy;   // M6r: north-south texel size -- the CUDEM grid is EQUIANGULAR, so
                    // dy = dlat*mPerLat (13.65 m) != dx = dlon*mPerLon (10.08 m)
        float faceCount;  // the river faces (gFaceA, gFaceB)
        float dx, dt, damp, tideNavd;
        float wetEps, soilRate, cd, gravity;   // wet is h > wetEps (m); soilRate: C2's soil (0 = none);
                                               // cd: the bed's quadratic drag coefficient
        float faceA[16];   // a face (four floats each, kMaxRiverFaces): side, c0, c1 (the run along it), 0
        float faceB[16];   // its u_ext (m/s, + inward), its exterior's level less the tide plane (m), 0, 0
        // C1: the open sides' Flather u_ext, m/s + inward: the domain's stateless prism (its wet area
        // at the tide plane times the plane's rate) less the rivers' transport, over the sides' live
        // section (OceanBoundary). The prism arrives through the boundary; nothing books it as debt.
        float oceanU;
        float mip, mips, etaTilesXUp;   // stage 1c: the dispatch's mip, the chain's mips, the mip above's tiles across
        // PHASE C1: A CELL'S POINT in the domain's chart. cellX = the first cell's centre in the anchor's
        // tangent plane over R (east s0, north t0) and the steps (dx / R, -dy / R): cell (i, j) at
        // (s0 + i ds, t0 + j dt); anc = the anchor's up (planet frame) and R; east = its east and the
        // cell's grain (m). Then the standing window's rows (WindowRows.hlsli).
        float cellX[4];
        float anc[4];
        float east[4];
        float hwU[20];
        float hwV[20];
        float hwW[20];
        float hwO[12];
        uint32_t hwS[8];
        // C1: the open sides' Level, four samples a side (a = 0, 1/3, 2/3, 1 of its length), less the
        // tide plane; and C2's soil (capacity m, conductivity m/s, suction head m, 1 = on). APPENDED AT
        // THE END on both sides -- a same-size insertion in the middle passes the byte-parity gate and
        // silently offsets every later row (the lesson WaterBank.hlsl:44 records).
        float oceanEta[16];
        float soil[4];
        float testA[4];   // the scheme's tests (RunSchemeTests): the case, its square's first cell, its side
        float testB[4];   // hL, hR, the lake's level, 0
        float sea[4];     // C4: Hs, the peak's angular frequency, Hs^2 / 16, 1 = waves push
        float sea2[4];    // the direction the sea travels (texture frame: x east, y south), 0, 0
        // STAGE 3 (2026-10-27): an impact imposed on the columns (Swe.hlsl CsSweImpose, BeginImpose): its place in
        // the anchor's chart (east, north; m), the seconds since its zero, the basin depth; then its table's term
        // count (0 = nothing this batch), 0, 0, 0.
        float impactA[4];
        float impactB[4];
        // STAGE 2b: the tail under this lattice (Swe.hlsl gTail/gTail2): the shift to its cells (< 0 none), the
        // tail cell the window's corner lies in, its cells across; its cells down, its tiles across, 0, 0.
        float tail[4];
        float tail2[4];
        float impactC[4];   // stage 3: the event's cavity radius and depth (m), 0, 0
        float up[4];        // stage 2d: the magnify's parent -- the shift to its cells, the cell above the corner (x, y), 0
    };

    bool m_ready = false;
    SweDomain m_dom;   // PHASE C1: the solver's own chart and its CPU bed

    TileAtlas2D m_eta, m_flux;
    // C1: THE STATE -- the water each column holds (m_h), its bed cached a frame (m_bed), the faces'
    // velocities (m_velA and m_velB take turns: the advection reads the substep before), the soil's
    // water (m_soil, C2). m_flux holds each substep's fluxes. Every bank covers m_h's tiles.
    // STAGE 0 OF ONE WATER ON THE TREE (2026-10-26, docs/WATER_V2.md 14): the column field is a GradeBank
    // -- grade 0, the water each column holds -- with the chain's mips in the one reserved resource
    // (unbuilt at stage 0: mip 0 is today's window, byte for byte). The discharge (grade 1) joins it when
    // the staggered faces are the vector's own home (stage 1); until then the faces' banks below.
    GradeBank m_h;
    TileAtlas2D m_bed, m_soil, m_velA, m_velB;
    // C4: the sea in the columns, two banks taking turns sweep by sweep (Swe.hlsl CsSweWaves); the solve
    // reads m_waveB, where an even number of sweeps ends. m_waveTable[0] writes A from B, [1] B from A.
    TileAtlas2D m_waveA, m_waveB;
    TileAtlas2D m_lim;      // positivity: each column's share of its outgoing fluxes (Swe.hlsl CsSweLimit)
    hal::Pso m_limitK;
    hal::Table m_waveTable[2];
    hal::Pso m_wavesK;
    std::function<bool(double, double, double, double&, double&, double&)> m_seaAt;
    static constexpr int kWaveSweeps = 4;
    void SeaBoundary();
    int m_velNew = 0;   // the bank holding the newest velocities: 0 m_velA, 1 m_velB
    std::vector<uint32_t> m_live;   // a word a state tile: 1 = resident (Swe.hlsl gLive)
    std::function<double(double, double, double)> m_levelAt;   // SetLevelSource
    std::vector<float> m_spill;     // the connected cells' spill levels, sorted (WetAreaAt)
    // RunSchemeTests: the square's own tiles (the state's, the fluxes'), dispatched in place of the
    // resident lists while a test runs; empty otherwise.
    std::vector<uint32_t> m_testH, m_testF;
    // C2: the tile statistics (RGBA32_UINT, a texel a state tile) and their readback; the ledger (1 x 1);
    // the tiles mapped since the last Record, cleared at its start; each state tile's dry time (s).
    GpuTexture m_statsTex, m_ledgerTex;
    Gpu::TextureReadback m_rbStats;
    double m_statsAt = 0.0;
    hal::Pso m_statsK, m_ledgerK, m_clearTileK;
    std::vector<uint32_t> m_ledgerFaces;   // the count, then a face a word (Swe.hlsl CsSweLedger)
    bool m_ledgerOn = false;
    double m_ledgerBase = 0.0, m_released = 0.0;
    std::vector<uint32_t> m_freshH, m_freshF;
    std::vector<double> m_dryFor;
    double m_openSea = 0.0;                // OpenSeaLevel (OceanBoundary)
    double m_releaseS = 1800.0;            // the hysteresis: dry this long, released
    double m_surgeA = 0.0, m_surgePeak = 0.0, m_surgeTau = 3600.0;
    double m_openSaidT[4] = {-1e18, -1e18, -1e18, -1e18};   // each side's level last said (sim s)
    uint32_t m_stepBudget = kMaxSubsteps;   // SetStepBudget
    // M9h: grad(flow) -- residency derived from the Cayley closure, not a physics policy.
    GradeBank m_velGrad;
    ShaderCompiler* m_sc = nullptr;   // M9h: BuildChain compiles the reducer on first use
    std::wstring m_shaderDir;
    // M9ax: the derived currents as a Volatile GradeBank over the wet tiles -- the same
    // residency as eta -- with a TEXTURE2D view over slice 0 for the consumers that read a
    // plain texture (Sea.hlsl, SeaChurn.hlsl, WaterBank.hlsl). The dense committed RGBA16F
    // this replaces was the last flat texture on the per-frame water path (AUDIT_WATER item 4).
    GradeBank m_uvBank;
    uint32_t m_uvSrv = UINT32_MAX;
    hal::RootSignatureRef m_rs;
    hal::Pso m_resetK, m_bedFillK, m_clearFlux, m_uvClear, m_fluxK, m_heightK, m_deriveK;
    hal::Pso m_velGradK;   // M9h: grad(flow) -> div + curl
    // [t1 height page, t2 its residency map, u0 eta, u1 flux, u2 uv, u3 mv, u6 h, u7 the velocities
    // before, u8 bed, u9 soil, u10 the velocities made, u11 the tile statistics, u12 the ledger]:
    // m_table reads m_velA and writes m_velB, m_tableB the other way.
    hal::Table m_table, m_tableB;
    D3D12_RESOURCE_STATES m_etaState = D3D12_RESOURCE_STATE_UNORDERED_ACCESS;
    D3D12_RESOURCE_STATES m_uvState = D3D12_RESOURCE_STATE_UNORDERED_ACCESS;
    Gpu::TextureReadback m_rbEta, m_rbUv;   // F13: the asynchronous read of the two fields
    static void UnpackFields(uint32_t nx, uint32_t ny, const std::vector<uint8_t>& etaData,
                             uint32_t etaPitch, const std::vector<uint8_t>& uvData, uint32_t uvPitch,
                             std::vector<float>& etaOut, uint32_t& etaW, uint32_t& etaH,
                             std::vector<float>& uv4Out, uint32_t& uvW, uint32_t& uvH);

    SweCbData m_cb{};
    // STAGE 1a OF ONE WATER ON THE TREE (2026-10-26, docs/WATER_V2.md 14.4): THE COARSE TWIN. The same
    // lattice at mip 1 -- cells twice the size, the whole box resident -- stepped by the same kernels
    // over its own lists with its own constants and views, after mip 0's steps and over the same
    // interval (half as many steps of twice the size: the subcycling), uncoupled from mip 0 until 1b
    // (the fold) and 1c (the faces). Its level at the anchor against mip 0's is the gate.
    bool m_coarse = true;
    SweCbData m_cb1{};
    hal::Table m_table1, m_tableB1;
    hal::Table m_waveTable1[2];   // stage 1c: the twin's sweeps' tables (the two sea banks in turn at mip 1)
    std::vector<uint32_t> m_list1, m_fluxList1, m_live1;
    bool m_coarseFresh = true;   // its tiles' first batch: cleared and reset to the stateless column
    uint32_t m_velNew1 = 0;
    // STAGE 1b: THE FOLD -- the coarse cells under four resident fine cells take their fold each batch
    // (Swe.hlsl CsSweFold); one table per parity pair (the fine's newest velocity bank, the twin's read bank).
    hal::Pso m_foldK;
    hal::Table m_tableFold[2][2];
    std::vector<uint32_t> m_foldList;
    // STAGE 1c: THE CHAINS' TABLE (Swe.hlsl gHChain/gBedChain/gVelChain, root param 4): h and the bed at every
    // mip, the velocity bank each mip reads, one table per parity pair [the fine's read][the twin's read].
    hal::Table m_tableChain[2][2][2];   // stage 2b: [fine parity][twin parity][tail parity]
    // STAGE 1d: THE EYE'S DEMAND -- mip 0 is wanted within reachM of the eye's ground point (0 = everywhere);
    // the twin carries the rest as its own cells (UpdateResidency).
    bool m_eyeOn = false;
    double m_eyeLat = 0.0, m_eyeLon = 0.0, m_reachM = 0.0;
    std::vector<uint32_t> m_stepList1;   // the twin's tiles outside the fine set: the ones its height and limit step
    double m_probeAt = -1e18, m_probeLat = 0.0, m_probeLon = 0.0;
    bool m_probeOn = false;
    // stage 3: the impacts' record, which events are imposed, the instant of the last imposition (the probe says
    // the law beside the solver for the hour after), this batch's event and its table on the GPU
    std::shared_ptr<const ImpactField> m_impacts;
    std::vector<uint8_t> m_imposed;
    double m_imposedAt = -1e18;
    int m_imposeEvent = -1;
    D3D12_GPU_VIRTUAL_ADDRESS m_imposeVa = 0;
    hal::Pso m_imposeK;
    bool BeginImpose(Gpu& gpu, bool reset);
    void RecordImpose(hal::CommandContext& cmd, D3D12_GPU_VIRTUAL_ADDRESS listVa, UINT count, const hal::Table& table);
    // STAGE 2c (2026-10-27, Mark: "the telescope"): THE LEVELS -- the tail of stage 2b as the coarsest of a list, each a
    // rectangle of the one anchor chart at its own cell size (2^rank window cells) with its corner on its parent's
    // lattice, the same kernels on its own banks, lists and clock; its edge the sea's exterior at its parent's
    // departure in the ghost cell beyond (the top's at the impacts' law); a level of mean sea's gauge folds its wet
    // mean into its parent before each parent step. The anchor levels telescope from the tail to the window; an
    // event whose cavity is unresolved where it falls gets a patch of its own at the rank its radius needs.
    struct Level {
        std::string name;
        uint32_t rank = 9, nx = 0, ny = 0;      // the cell = 2^rank window cells; cells across and down
        int64_t c0x = 0, c0y = 0;               // the corner in rank-0 (window) cells; the window's corner is (0, 0)
        int parent = -1;                        // the next coarser level holding this one and a ghost beyond (-1: the top)
        bool foldUp = false, patch = false;
        // stage 2d: a SPARSE level -- the parent's whole extent, resident only where the water departs from rest and
        // a tile ahead of it (the window's residency law on the departure: a train's front demands the tiles ahead of
        // itself and the sea it has left is released); its bed is the stack's on the GPU alone (no CPU bed, no flood)
        bool sparse = false;
        uint32_t px = 0, py = 0;                // a patch: the event's cell (the probe reads there)
        uint32_t qx = 0, qy = 0;                // ...and the cell 300 km from it toward the anchor (the convergence probe)
        GpuTexture stats;                       // its tile statistics (CsSweTileStats on the departure)
        std::vector<uint32_t> freshH, freshF;   // tiles mapped since its last batch: cleared, bedded, rested
        std::vector<double> dryFor;             // per tile: how long quiet (the release's hysteresis)
        double statsAt = -1.0e18, deepest = 0.0;
        bool statsDue = false;                  // the statistics were dispatched this batch: read them back outside the recording
        Gpu::TextureReadback rb;                // the frame ring's asynchronous read of them
        SweDomain dom;
        SweCbData cb{};
        TileAtlas2D h, bed, soil, eta, flux, velA, velB, lim;
        hal::Table table[2];                    // the step tables, by the read velocity bank
        hal::Table chain[2];                    // the parent's views at the chain's slot 2, by the parent's parity
        hal::Table fold[2][2];                  // [this level's newest bank][the parent's read bank]
        std::vector<uint32_t> list, fluxList, live;
        uint32_t velNew = 0, bedMip = 3;
        double simTime = 0.0;
        float dt = 1.0f, imposeN = 0.0f;
        bool fresh = true;
        D3D12_RESOURCE_STATES state = D3D12_RESOURCE_STATE_UNORDERED_ACCESS;
        D3D12_GPU_VIRTUAL_ADDRESS liveVa = 0, hlVa = 0, flVa = 0, cbVa = 0;   // pushed once a batch (the prelude); the steps bind them by address
    };
    std::vector<std::unique_ptr<Level>> m_lv;   // coarsest first
    int m_winParent = -1;                        // the level the window's edge reads
    std::function<float(double, double, double)> m_bedAt;
    double m_patchCellsPerLip = 8.0;
    std::function<double(double, double, double)> m_tailLevelAt;
    hal::Resource m_heightArr = nullptr, m_resMapArr = nullptr;   // the stack's page views, for a level made later
    uint32_t m_heightMips = 1;
    hal::Pso m_foldTailK, m_magInitK;   // the fold into the parent; a fresh tile from the parent (stage 2d)
    void ParentFields(SweCbData& cb, int parent, uint32_t rank, int64_t c0x, int64_t c0y) const;
    uint32_t WinParentParity() const;
    void LevelCb(Level& L);
    void LevelBoundary(Level& L);
    void LevelTables(Gpu& gpu, Level& L);
    int AddLevel(Gpu& gpu, uint32_t rank, uint32_t cells, double cx, double cy, const char* name, bool patch, bool tables,
                 bool sparse = false, uint32_t sparseHalfCells = 0);
    void LevelLists(Level& L);
    void UpdateLevelResidency(Gpu& gpu, Level& L, const std::vector<uint8_t>& raw, uint32_t pitch);
    void InitTelescope(Gpu& gpu, const SweConfig& cfg);
    void LevelTo(hal::CommandContext& cmd, Level& L, D3D12_RESOURCE_STATES to);
    void RecordLevel(hal::CommandContext& cmd, Gpu& gpu, Level& L, double batchEnd, bool reset, bool steps);   // the prelude (steps = false), then the steps
    double TailRayArrivalS(double latDeg, double lonDeg, double h0, double& shallowest) const;
    double TailFastestArrivalS(double latDeg, double lonDeg) const;   // the fastest sqrt(g h) path over the tail's sea (Fermat)
    void CoarseCb(int n, int n1);
    // STAGE 1c (2026-10-26): THE TWIN INTERLEAVED -- Berger and Oliger's order. Begin: the twin's constants, lists,
    // clear, bed, reset and sea sweeps for the batch; Step: the fold of the fine's state, then ONE coarse step,
    // after which the fine takes its two. Stepping the fine's whole batch against a coarse state frozen for
    // sixteen steps, then the twin against a fold frozen for eight, grew a metre of ringing from the fine
    // set's seaward edge (the gate runs cd..cd4): two Dirichlet couplings alternating a batch apart.
    void RecordCoarseBegin(hal::CommandContext& cmd, Gpu& gpu, int n, int n1, bool reset);
    void RecordCoarseStep(hal::CommandContext& cmd, Gpu& gpu);
    D3D12_GPU_VIRTUAL_ADDRESS m_hl1Va = 0, m_fl1Va = 0, m_sl1Va = 0, m_live1Va = 0;
    UINT m_hn1 = 0, m_fn1 = 0, m_sn1 = 0;
    void ProbeCoarse(Gpu& gpu);
public:
    // The gate's probe: the level at this place, mip 0 against mip 1, said once a sim-minute (AfterBatch).
    void SetProbe(double latDeg, double lonDeg) { m_probeLat = latDeg; m_probeLon = lonDeg; m_probeOn = true; }
    void SetEye(double latDeg, double lonDeg, double reachM);   // stage 1d: the eye's ground point and its reach for mip 0
private:
    double m_simTime = 0;
    float m_faceDEta[kMaxRiverFaces] = {};   // the faces' target deviations (m)
    float m_faceQ[kMaxRiverFaces] = {};      // their transport targets, m^3/s (+ inward)
    std::vector<float> m_faceBed[kMaxRiverFaces];   // their exterior cells' beds: the live section
    float m_faceL[kMaxRiverFaces] = {};      // a face cell's length along the side (m)
    SweConfig m_cfg;                   // as Init was given it (the holds-water line)
    bool m_bedBound = false;           // M9ar: SetBed has run
    uint32_t m_bedSlice = UINT32_MAX;  // PHASE B3: the standing window's slice (was the z14 page's)
    // The bed trace's own binding (TraceBed): the height tenant as SetHeightPage bound it, and the
    // kernel, table and target built on its first call -- nothing of it exists in a run without.
    hal::Resource m_bedArr = nullptr, m_bedRes = nullptr;
    uint32_t m_bedMips = 0;
    hal::RootSignatureRef m_traceRs;
    hal::Pso m_traceK;
    hal::Table m_traceTable;   // [t1 height page, t2 its residency map, u4 the trace]
    GpuTexture m_traceTex;     // RG32F: BedAt, mip + 16 * slice
    // The face trace's (TraceFace): the flux kernel under SWE_FACE_TRACE, its table and target.
    hal::RootSignatureRef m_faceRs;
    hal::Pso m_faceK;
    hal::Table m_faceTable;    // [t1, t2, u0 eta, u1 flux, u2 uv, u3 mv, u5 the trace, u6 h, u7, u8 bed, u9, u10, u11, u12]
    GpuTexture m_faceTex;      // RGBA32F, 4 x max(nx, ny)
    float m_faceArea[kMaxRiverFaces] = {};   // Record's live sections, m^2
    uint64_t m_cbFp = 0;               // M12 step 4b: the [kernel] swe cb fingerprint's last value
    float m_lastTideNavd = 0;          // for the tide plane's rate (the open sea's prism)
    double m_lastTideTime = 0;
    float m_dt = 0.25f;
    bool m_pendingReset = true;

    // ---- the region readback (RequestRegion / DeviationAt) --------------------------------------
    // A place in the kernel's continuous cell coordinates (the domain's chart).
    bool TexelOf(double latDeg, double lonDeg, double& tx, double& ty) const;
    // Read what the ring delivered for this frame's slot, then copy this frame's requests into it.
    void CollectRegions(Gpu& gpu);
    void ServeRegions(hal::CommandContext& cmd, Gpu& gpu,
                      const std::function<void(D3D12_RESOURCE_STATES)>& etaTo,
                      const std::function<void(D3D12_RESOURCE_STATES)>& uvTo);
    struct RegionRect {
        uint32_t x0 = 0, y0 = 0, w = 0, h = 0;
    };
    struct DeliveredRegion {
        RegionRect rect;
        std::vector<float> eta;   // w * h
        std::vector<float> uv;    // w * h * 4: u, v, speed, valid
        double asOf = 0.0;
    };
    // Bounds, and why these: a hull asks for its footprint plus its motion over the ring's latency
    // (a few metres to tens), so a region of kMaxRegionTexels a side holds any hull at any speed a
    // hull reaches in two frames; kMaxRegions requests a frame holds eight subjects. The readback
    // slot is sized from the two, and a request past either is refused and logged, never clipped.
    static constexpr uint32_t kMaxRegions = 8;
    static constexpr uint32_t kMaxRegionTexels = 64;
    std::vector<RegionRect> m_requests;           // this frame's
    std::vector<DeliveredRegion> m_regions;       // newest first
    double m_slotAsOf[Gpu::kFrameCount] = {};      // the clock each slot's copies were taken at
    hal::RegionReadback m_readback;
    bool m_refusedLogged = false;
};

}  // namespace ga
