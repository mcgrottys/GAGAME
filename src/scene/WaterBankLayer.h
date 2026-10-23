// ================================================================================================
//  WaterBankLayer - M7: THE WAVE VERTEX BANK -- the water's geometry as ONE tiled resource.
//
//  Two RGBA16F TileAtlas2D banks (disp = wave vertex + foam; param = level + sigma^2 +
//  current) laid out as a MIP LADDER of camera-anchored rings: ring m covers 2^m times the
//  ground of ring m-1 at half the texel density, every ring 512x512 texels (4x4 logical
//  tiles of 128^2). The window re-anchors by snapping to tile boundaries as the camera
//  moves -- free, because every resident tile is RECOMPUTED each frame from stateless state
//  (the M6t fold makes ring handovers energy-conserving: geometry a fine ring resolves is
//  sigma^2 in the coarse ring, never lost, never doubled). LAND tiles are NULL -- tested
//  against the one height stack at anchor time, unmapped in the tiled resource: no water
//  here costs no memory and no compute, the atlas thesis as hydrodynamic geometry.
//
//  In the state-diagram architecture (the user's): this layer is the EDGE from the weather-
//  manager node into the renderer node -- per-tile corner params (tide level by constituent
//  rotors, local Hs, the one bed) flow in as the CPU-side geometric product; the fill
//  kernel composes them with the cascade rotors and the SWE banks on the GPU; the accepting
//  state (GlobeMesh) samples the bank and never asks who computed what.
// ================================================================================================
#pragma once

#include "compose/Compositor.h"
#include "compose/SurfaceFrame.h"
#include "compose/WaterAtlas.h"
#include "hal/TileAtlas.h"
#include "hal/Readback.h"
#include "core/OriginLattice.h"
#include "scene/Layer.h"
#include "sim/GlobeModel.h"
#include "sim/WaveChart.h"
#include "sim/SeaState.h"
#include "sim/SweSolver.h"

#include <array>
#include <condition_variable>
#include <cstring>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

namespace ga {

class SeaLayer;

class WaterBankLayer : public Layer {
public:
    // M9p: --flat-bed. Replaces the sampled bed with a constant so the same scene can be
    // filled twice and diffed. Public because it is a debug lever, not state.
    bool flatBed = false;
    float flatBedNavd = -30.0f;
    // THE COLUMN PLANE (the rolling-wave work, 2026-10-18; WaterBank.hlsl CsBankScatter): the
    // Lagrangian sheet scattered into Eulerian columns of three crossings (face, lip underside,
    // lip top), so an overturning crest has geometry instead of a faded lateral term. Scene
    // `water.bank.column`; off, no plane is made and the mesh draws as it did.
    bool column = true;
    static constexpr float kColScale = 100.0f;    // the fixed point: a centimetre (14 bits, +-82 m;
                                                  // the label's offset rides in the low 18 bits)
    static constexpr float kColBias = 64.0f;      // metres below the waterline the encoding reaches
    static constexpr uint32_t kColSubLabels = 2;  // sub-labels an axis a texel in the scatter
    uint32_t ColumnSrv() const { return (column && m_ready) ? m_column.Srv() : 0xFFFFFFFFu; }
    // THE COLUMN LADDER (2026-10-20; Mark: nothing in code is "fine" or "coarse"). Ring 0's
    // columns again at half its texel, a quarter, an eighth: the rings UNDER ring 0, each a mip of
    // the one above over ring 0's span (so ring -(k+1) has 512 x 2^(k+1) texels a side), four bands
    // each, in its own reserved atlas of 128^2 tiles, resident only where the ring above witnessed a
    // fold (the scatter's inverted triangles flag the tiles of the ring under; the flags come back
    // through the frame ring; the CPU maps those tiles and their eight neighbours and unmaps the
    // rest). How deep the ladder goes is the scene's (`water.bank.columnTexel`, the finest texel it
    // reaches, 0.3 m by default = two rings under). A NULL tile reads absent and the readers take the
    // next ring up: the hierarchy's law, finest resident. Nothing here is named by its grain.
    static constexpr uint32_t kColumnRingsUnderMax = 3;   // 0.6, 0.3, 0.15 m; 4096^2 x 4 bands is the API's ceiling
    float columnTexel = 0.3f;                              // the finest texel the ladder reaches (m)
    uint32_t ColumnRingsUnder() const { return m_ringsUnder; }
    uint32_t ColumnRingSrv(uint32_t k) const { return (column && m_ready && k < m_ringsUnder) ? m_under[k].atlas.Srv() : 0xFFFFFFFFu; }
    uint32_t ColumnRingTexels(uint32_t k) const { return k < m_ringsUnder ? m_under[k].texels : 0u; }
    uint32_t ColumnRingTiles(uint32_t k) const { return k < m_ringsUnder ? m_under[k].tiles : 0u; }
    uint32_t ColumnRingMapped(uint32_t k) const { return k < m_ringsUnder ? m_under[k].mappedCount : 0u; }
    // THE SPRAY (2026-10-19, Mark's idea): the water the crest law cannot carry -- the throw the
    // quarter-wavelength bound removes -- leaves the sheet as DROPLETS. A fixed ring of particles
    // on the GPU (two float4 each: position in the rings' chart + age, velocity + size), spawned by
    // the fill kernel at crest texels where the bound bites, stepped by CsSprayStep (gravity, the
    // drag of a water drop in air by its terminal velocity, death at the column's face or by age),
    // drawn by GlobeLayer as lit sprites after the lip sheets. Not water a hull feels: no CPU twin.
    static constexpr uint32_t kSprayMax = 65536;
    static constexpr float kSprayTerminalMs = 5.0f;   // Gunn & Kinzer 1949: a 1-2 mm drop falls at 4-6.5 m/s
    static constexpr float kSprayLifeS = 6.0f;
    uint32_t SpraySrv() const { return (column && m_ready) ? m_spraySrv : 0xFFFFFFFFu; }
    // THE SEA'S FOLDS (2026-10-20): ring 0's inverted triangles marked on a 64 x 64 grid of 8-texel
    // blocks (the scatter), their distance field for the globe's morph (CsFoldDist, in blocks),
    // and the grid back on the CPU for the walk (the blocks a fold stood over, bx | bz << 8, as of
    // the readback's frame, about ring 0's origin of this one -- a frame or two of slip).
    static constexpr uint32_t kFoldGrid = 64;
    uint32_t FoldDistSrv() const { return (column && m_ready && m_foldDistSrv) ? m_foldDistSrv : 0xFFFFFFFFu; }
    const std::vector<uint16_t>& FoldCells() const { return m_foldCells; }
    double FoldBlockM() const { return 8.0 * m_baseTexelM; }
    uint32_t SprayAlive() const { return m_sprayAlive; }
    uint32_t SpraySpawned() const { return m_spraySpawned; }

    static constexpr int kMips = 6;         // texel = base * 2^m; at base 1.2: 1.2..38 m
                                            // texels, 0.6..20 km spans (scene bankTexelM)
    static constexpr int kRingTiles = 4;    // 4x4 logical tiles per ring
    static constexpr int kTileTexels = 128;
    static constexpr int kRingTexels = kRingTiles * kTileTexels;

    void Configure(const std::wstring& shaderDir, SeaLayer* sea, SweSolver* swe,
                   const WaterAtlas* atlas, Compositor* comp,
                   int hgtCh, const GlobeModel* globe);

    const char* Name() const override { return "waterbank"; }
    void Init(Gpu& gpu, ShaderCompiler& sc, FieldSet& fields,
              hal::RootSignature rootSig) override;
    void ReloadShaders(Gpu& gpu, ShaderCompiler& sc) override;
    void Render(const FrameContext& ctx) override;   // ring update + fill dispatch

    // Per frame, BEFORE RenderFrame (the SeaLayer::SetTime pattern): ring anchors follow
    // the camera and tile residency commits here, so consumers bind THIS frame's origins.
    // `eye` is the rings' eye in the flat frame (x east, y up, z north of world.flat's plane).
    void SetFrame(Gpu& gpu, double simUnix, const double eye[3]);
    // THE RINGS' CHART FLOATS WITH THE EYE. A ring is addressed by the radial projection onto a
    // tangent plane, (x, z) = R (d.e, d.n) of a surface direction d -- linear in the point, so a
    // reader needs only the plane's two axes and the eye's coordinate. That plane was the root's
    // (place.anchor): exact at home, and past 90 degrees of arc (Tokyo from the Merrimack) no plane
    // at all -- the eye's coordinate ten thousand km out, a metre of float grain, the waves in
    // triangles (Mark, 2026-10-10). Now the plane is the bank's own: the root's while the eye is
    // within kChartReachM of its anchor (so home reads bit for bit what it read), re-anchored on the
    // ground under the eye when it leaves -- where the float grain of the eye's coordinate passes a
    // millimetre and the plane's tilt a tenth of a degree.
    // (2026-10-11: the chart is the LATTICE ORIGIN under the eye, core/OriginLattice.h -- the reach
    // rule above is history; the hysteresis is the lattice's.)
    const Space::Anchor& Chart() const { return m_chart; }
    // A point of the flat frame (the probes', the fleet's) in the chart: R (d.e, d.n) of its
    // direction for a surface point; for any point, its planet position's (P.e, P.n).
    void ChartOfFlat(double x, double y, double z, double& cx, double& cz) const;
    // The rows a reader needs, in the frame's TANGENT axes (east, up, north of world.flat): the
    // chart's east and north (w = the eye's coordinate along each, in doubles before the cast),
    // its up, and the eye itself, sphere-centred (the level's sLvlCamAbs, cast from the same
    // doubles, so a reader standing at this eye cancels it exactly). 16 floats.
    void ChartRows(float out[16]) const;
    // Per frame, BEFORE RenderFrame: the tide plane the solver is forced by this frame (the value
    // SeaLayer::SetTime hands it) -- inside the solver's domain the level IS that plane plus the
    // solver's deviation (the solver is truth), and the kernel needs the plane to say so.
    void SetTidePlane(double navdM) { m_tidePlane = static_cast<float>(navdM); }
    // Per frame, BEFORE RenderFrame: the frame's live waterline (the value the globe shades and
    // classifies against, GlobeLayer::waterNavd). The param plane's level is stored as its
    // DEPARTURE from this (WaterBank.hlsl), so a tile the rings never mapped reads as the
    // waterline and not as 0 m NAVD; every reader adds it back, this layer's own included.
    void SetWaterline(double navdM) { m_waterline = static_cast<float>(navdM); }

    uint32_t DispSrv() const { return m_disp.Srv(); }
    uint32_t ParamSrv() const { return m_param.Srv(); }
    uint32_t DetailSrv() const { return m_detail.Srv(); }
    // Ring georef for consumers: per-mip window origin (world m) + base texel size.
    void RingOrigin(int m, float& x, float& z) const { x = m_orgX[m]; z = m_orgZ[m]; }
    // M7j --trace: read the ACTUAL bank texel at a world point -- the hypervisor's
    // cross-check between the CPU's expectation and what the GPU wrote.
    void TraceProbe(Gpu& gpu, double wx, double wz);
    // M9bq --twin-surface: what the bank ACTUALLY holds at a set of world points, for the
    // CPU/GPU twin gate. TraceProbe answers one point and costs three whole-texture readbacks
    // to do it; a gate wants hundreds of points and can afford exactly one fence, so this reads
    // each plane ONCE and then addresses every point out of that snapshot. Same texel maths,
    // same finest-resident-ring rule, one drain instead of 3N.
    struct BankPoint {
        bool valid = false;      // false = outside every resident ring
        int ring = -1;           // which mip answered
        float texelM = 0.0f;
        float dispX = 0, dispY = 0, dispZ = 0, foam = 0;
        float level = 0, sigma2 = 0, curU = 0, curV = 0;   // level in m NAVD (the texel's departure + the waterline)
        // The detail plane, as the kernel wrote it: the per-band gains of the FULL closure (sea-state
        // scale x shadow x shoaling x wave-current, before any window stand-down) and the dry weight.
        float gain0 = 0, gain1 = 0, gain2 = 0, dry = 0;
    };
    void ReadBankPoints(Gpu& gpu, const double* worldXz, int n, BankPoint* out);

    // M7k --dump-fibers: export the three bank planes as PNGs + validate the value
    // ranges against the AST's declarations -- the hypervisor for whole fields.
    void DumpFibers(Gpu& gpu);
    // PHASE B2: THE WINDOWS THE RINGS STAND IN, set each frame by the frame loop: the rows of the
    // level whose eye the rings stand about (its slot's, SurfaceFrame::RowsOf), that eye (planet
    // frame) and the tangent axes the rows were pulled into. No rows (K 0): the cube alone.
    void SetWindows(const SurfaceFrame::ChainRows& rows, const double eye[3]) {
        SurfaceFrame::KernelRows(rows, m_hw);
        for (int i = 0; i < 3; ++i) m_hwEye[i] = eye[i];
        // PHASE C1: the solver's chart about the same frame (zeros: no solver).
        if (m_surface && m_swe) {
            m_swe->Domain().KernelRows(m_surface->east, m_surface->up, m_surface->north, eye, m_sv);
        }
        m_hwOn = rows.K > 0;
    }
    int injectPattern = 0;   // M7m/M7n: 1 = bank world card, 2 = cascade-edge card
    // PHASE B0 (out/integration/plan_phase_b.md): THE BANK'S BED TRACE (--bank-trace). With traceOn
    // every fill is followed by the same kernel compiled with HP_TRACE and BANK_TRACE, which writes
    // in place of the banks what the fill read and decided per texel -- the bed, the slice and mip
    // the bed's rule chose (or the corner lerp), the depth, the vertical excursion before the
    // breaking clamp -- into a texture of its own. TraceRead reads the last one back and says it per
    // ring: where the rings stand, the bed's sources and mips, the bed, and hmax = 0.55 depth (the
    // clamp) with the share of texels it cut; two lens images (hmax, the source) and the raw planes
    // go to `dir`. traceFloor plants a residency floor under the rule's reads (the instrument must
    // then call the bed coarse). An instrument: off, nothing is compiled, dispatched or read.
    bool traceOn = false;
    float traceFloor = 0.0f;
    bool TraceRead(Gpu& gpu, const std::string& dir, const std::string& label);
    // M7q: the composed height, per texel, in the kernel -- the corner-lerp bed quantized depth
    // to ~600 m patches. PHASE B3: the height tenant's array views (its cube and its windows); the
    // rows are SetWindows'.
    void SetHeightWindow(uint32_t srv, uint32_t resMapSrv) {
        m_hgtWinSrv = srv;
        m_hgtWinResSrv = resMapSrv;
    }
    // the surface, for the world.flat chart the rings' places are read through
    // (SurfaceFrame::FlatRows). Must precede the first Render.
    void SetSurface(const SurfaceFrame* s) { m_surface = s; }
    // The coast mask (GisVectorMask): a ring tile the survey calls water anywhere is wet, whatever
    // the height under it says (MaskWet, HeightWet).
    void SetMask(const class GisVectorMask* m) { m_mask = m; }
    // The mask rasters' state, shared with the loader threads that run them (AskMask). Close: no
    // raster starts after it, and it returns when none runs -- the mask must outlive them, so the
    // Assembly closes every bank's before its gisMask destructs (by this state: a bank's layer may
    // already be gone by then).
    struct MaskJobs {
        std::mutex mx;
        std::condition_variable idle;
        uint64_t want[kMips] = {}, ready[kMips] = {};   // the ring's latest ask; the ask answered
        std::vector<std::pair<int, uint8_t>> answers[kMips];
        int inflight = 0;
        bool closed = false;
        void Close() {
            std::unique_lock<std::mutex> lk(mx);
            closed = true;
            idle.wait(lk, [&] { return inflight == 0; });
        }
    };
    std::shared_ptr<MaskJobs> MaskJobsOf() const { return m_maskJobs; }
    // M8: the solved wave field (may be null / not Ready -- the kernel falls back to
    // the cascade closures outside the window, which is also the fallback everywhere).
    void SetWaveField(const class WaveField* wf) { m_wave = wf; }
    // M9bc: the wave field's PAGES (the tree's tenant) and the z16 frame they sit in. winPxX/Y: the
    // window's NW texel in that frame's own pixels (the water match, step 2: the kernel finds a
    // point's page texel through the solver's grid, from this corner).
    void SetWavePages(uint32_t srv, uint32_t resSrv, double orgPxX, double orgPxY, uint32_t nx,
                      uint32_t ny, double winPxX, double winPxY) {
        m_wavePages = srv;
        m_wavePagesRes = resSrv;
        m_waveOrgPx[0] = orgPxX;
        m_waveOrgPx[1] = orgPxY;
        m_waveWinPx[0] = winPxX;
        m_waveWinPx[1] = winPxY;
        m_waveNx = nx;
        m_waveNy = ny;
    }
    // M8: the water scene config (data/wave_scene.json, hot-reloaded in main) -- the
    // bank reads the LIVE values every frame, so an edit lands on the next recompose.
    void SetScene(const struct WaterSceneConfig* sc) { m_scene = sc; }
    // M8 wakes: the fleet table (8 slots, vqview layout). Disabled slots stay zero.
    // Positions and headings arrive in the flat frame and are said in the chart here.
    void SetBoats(const float* a32, const float* b32);
    float BaseTexelM() const { return m_baseTexelM; }
    // M8h: ring density from the scene (data/wave_scene.json bankTexelM). Call BEFORE
    // Init -- the ring spans, the kernel's fold thresholds, the 9b emulator, and the
    // globe's gBankA.x all derive from this one number, but only at construction.
    void SetBaseTexel(float m) {
        if (m > 0.1f && m < 100.0f) m_baseTexelM = m;
    }
    uint32_t ResidentTiles() const {
        return m_disp.ResidentCount() + m_param.ResidentCount() + m_detail.ResidentCount();
    }
    uint64_t ResidentBytes() const {
        return m_disp.ResidentBytes() + m_param.ResidentBytes() + m_detail.ResidentBytes();
    }

    bool enabled = true;
    std::string stats;
    // The CPU tile-list build inside Render (up to 384 CornerParams, each Level() = 18
    // SampleFieldStack + 18 sincos, plus the height-stack sample), last frame, ms. It runs
    // inside the [rail] RENDER bracket and was unnamed there; main reads and zeroes it.
    double tileListMs = 0.0;

private:
    // Mirrors BankCb in WaterBank.hlsl.

    struct BankCbData {
        float org[4];
        float patch[4];
        float bandK[4];
        float sweDims[4];
        float misc[4];
        uint32_t slotsA[4];
        uint32_t slotsB[4];
        uint32_t slotsC[4];   // x = churn atlas SRV (M7e foam memory)
        float churn[4];       // xy origin, z 1/domain, w atlas texels
        float peakDir[4];     // M7p: peak propagation dir xy, z valid (gPeakDir)
        uint32_t slotsD[4];   // M7q: height window SRV, its residency-map SRV
        uint32_t slotsE[4];   // M8 foamlaw: cascade DERIV SRVs x3 (Jacobian foam union)
        float rmsRef[4];      // M8: unit-sea rms envelope per band (crest gate / excess)
        uint32_t waveU[4];    // M9bc wavefield: page tenant SRV, its residency SRV, nUsed, env plane
        float waveA[4];       // window org xy (world m), 1/cellM, feather m
        float waveB[4];       // envMax, sumMax, chop, solved-at level
        float foamA[4];       // scene closures: churnGain, shedSteepCap, shedMssCeil, crestLo
        float foamB[4];       // crestHi, depthLo, depthHi, spare
        float waveSig[32];     // (cos, sin)(sigma_c t), packed 2 comps per float4 row
        float waveDir[32];     // unit propagation (east, north), same packing (gWaveDir)
        float waveScale[32];   // (aMax, kMax) dequant scales, same packing
        float boatA[32];       // M8 wakes: (x, z, heading rad, speed m/s) x8
        float boatB[32];       // (wake amp m, hull half-length m, enabled, spare) x8
        float bandKFold[4];    // M9c: the FOLD's wavenumber per band (energy-weighted);
                               // bandK above keeps the cut mean for the physics closures.
                               // APPENDED at the end, per the layout law two rows up.
        float debugA[4];       // M9p: x != 0 = flat-bed override, y = the bed (NAVD m). Sits
                               // AFTER bandKFold because gDebugA does in the HLSL -- the two
                               // orders are the contract, and a same-size swap passes the
                               // byte-parity gate while silently offsetting nothing here but
                               // reading the wrong row there.
        float waveP[4];       // M9bc: z16 page frame -- org px x, y, 1/16384, world px
        float waveD[4];       // M9bc: window nx, ny (cells = texels), 0, 0
        // M9bl: components 16..31. The first sixteen keep waveSig/waveDir/waveScale above at
        // their original offsets and these APPEND at the end -- the layout law: widening
        // those arrays in place would silently slide boatA and everything after it, which is
        // the exact failure the law was written for. Same split, same order, in the HLSL.
        float waveSig2[32];
        float waveDir2[32];
        float waveScale2[32];
        // M9bt: the fold's second moment, per band. APPENDED at the end on both sides, per
        // the layout law above -- widening bandKFold in place would slide every row after it.
        float bandKSpread[4];
        // THE SOLVER IS TRUTH (the water match, step 1): x = the tide plane the solver was forced
        // by this frame (NAVD m), which its deviation is measured from. APPENDED at the end on
        // both sides, per the layout law.
        float sweB[4];
        // PHASE B2: THE WINDOWS THE RINGS STAND IN (WindowRows.hlsli's HP_WINDOW_ROWS_DECL), the
        // rows of the level whose eye the rings stand about -- SurfaceFrame::KernelRows' packing,
        // appended at the END on both sides.
        float hwU[20];
        float hwV[20];
        float hwW[20];
        float hwO[12];
        uint32_t hwS[8];
        // PHASE C1: THE SOLVER'S CHART about the rings' frame (SweDomain::KernelRows): a texel reads
        // the solver where its ground lies in the domain's cells. Appended at the END on both sides.
        float svU[4], svV[4], svW[4], svO[4];
        // THE PREVIOUS TABLE (the wave roll; WaterBank.hlsl gWaveSigP..): appended at the END.
        float waveSigP[64], waveDirP[64], waveScaleP[64], waveBP[4];
        // THE COLUMN PLANE (gColU / gColA): its UAV slot, sub-labels an axis, on; the fixed point's
        // scale and bias. Appended at the END on both sides.
        uint32_t colU[4];
        float colA[4];
        // THE COLUMN LADDER (gColR / gColN / gColMap): ring under k's UAV slot, its flags' UAV
        // slot, its texels a side, its tiles a side; the rings under and on; the MAPPED SET of
        // every ring under as bits (ring k's at ColMapBase(k): 0, 64, 320; row-major by tile).
        uint32_t colR[12];   // ring k's four at 4k
        uint32_t colN[4];
        uint32_t colMap[128];
        // THE SPRAY (gSprayU / gSprayA): the particles' UAV slot, the counters' UAV slot, the ring's
        // size, on; dt (s), the drop's terminal speed (m/s), its life (s), the spawn gain.
        uint32_t sprayU[4];
        float sprayA[4];
        // THE PLANET (core/Planet.h): x its surface gravity, GM / R^2 (Jet.hlsli, the spray, the
        // wakes). Appended at the END on both sides.
        float planet[4];
        // WATER V2, STAGE 2: THE TIDE'S ROTORS of this instant (WaterAtlas::Rotors), two constituents
        // a row: cos, sin of constituent 2j, then of 2j + 1. Appended at the END on both sides.
        float tideRot[36];
    };
    struct BankTile {
        float orgXZ[2];
        float texelM;
        uint32_t dstX, dstY;
        float msl[4];   // MSL in NAVD88 at the corners (the tide's own datum, WaterAtlas::MslNavd)
        float bed[4];
        float hs[4];   // the local sea-state scale at the corners (sim/WaveScale.h)
        // M13 step 2: THE TILE'S PLACE, ADDRESSED ONCE. The kernel used to turn every texel's
        // (x, z) into lat/lon through the anchor-linear chart (gGeoA), which stands up to 28 m
        // from the ground the mesh draws it on at 5 km and 215 km away at Haulover. These rows
        // are the exact map (Space::Anchor::PlaceOfProjected: the place a ring texel's radial
        // projection comes from) evaluated in doubles at this tile's ORIGIN, with its tangent
        // map taken at the tile's CENTRE, so the kernel forms lat/lon per texel as
        //   lat = placeA.x + ex * placeA.z + ez * placeB.x
        //   lon = placeA.y + ex * placeA.w + ez * placeB.y
        // with (ex, ez) the texel's own metres inside the tile -- small, exact, and no
        // transcendental per texel. placeB.z != 0 marks the rows valid (a tile past this
        // frame's horizon has no place and keeps the old chart, which is all it ever had).
        float placeA[4];   // lat0, lon0, dLat/dx, dLon/dx
        float placeB[4];   // dLat/dz, dLon/dz, valid, spare
        // M13 step 2: THE CASCADE SEA'S PLANES for this tile (sim/WaveChart.h), addressed once.
        // Four charts, in the law's own order (the cell, its x neighbour, its y neighbour, the
        // diagonal), each as: its coordinate at the tile's ORIGIN reduced mod each cascade's
        // period (so the kernel's floats carry metres, not hundreds of kilometres), the tangent
        // map of that coordinate over the tile, and its axes said in the place's east/north.
        // bandX/bandY carry the edge distances the shares are computed from, so the kernel
        // recomputes the same weights per texel that the hull's twin computes per point.
        float chart[4][16];   // per chart: u0(c0).xy, u0(c1).xy, u0(c2).xy, J(4), rot(4), 2 spare
        float bandX[4];       // edgeX at the origin, d/dex, d/dez, the band in metres
        float bandY[4];       // edgeY at the origin, d/dex, d/dez, charts valid (0 = none)
        // PHASE B2: the tile's point for the windows' address (WaterBank.hlsl BankTile): the
        // ground point at the tile's origin relative to the rows' eye in the tangent axes (w = 1
        // valid), and its derivative along the ring's x and z at the tile's centre.
        float pointA[4];
        float pointX[4];
        float pointZ[4];
        // WATER V2, STAGE 2: THE TIDE AS A FUNCTION OF TIME at the corners -- each corner's 18
        // constituent phasors (re, im), two a row; the kernel turns them by this frame's rotors. They
        // are a place's and never change with t, so the ring's lattice asks the atlas once per point
        // (TideLattice), not every frame.
        float tide[4][36];
    };
    // The tide's phasors at a ring's lattice points, asked when the ring's origin moves.
    struct TideLattice {
        float orgX = 0.0f, orgZ = 0.0f;
        bool ok = false;
        float msl[(kRingTiles + 1) * (kRingTiles + 1)];
        float p[(kRingTiles + 1) * (kRingTiles + 1)][36];
    };
    TideLattice m_tideLat[kMips];

    void ReanchorRing(Gpu& gpu, int m, double camX, double camZ);
    // A tile carries waves where the mask calls any of it water, or the height under it is low.
    void MaskBox(double wx0, double wz0, double spanM, double box[4]) const;   // the tile's lat/lon box
    static bool MaskAnswer(const class GisVectorMask& mask, const double box[4]);
    void AskMask(int m, const std::vector<std::pair<int, std::array<double, 4>>>& tiles);
    bool HeightWet(double wx0, double wz0, double spanM, bool& provisional) const;
    void CornerParams(double wx, double wz, float& bed) const;
    void CornerTide(double wx, double wz, float& msl, float p[36]) const;
    // M13 step 2: the place of a ring point, and the tile rows above. Both go through the
    // surface's chart where it carries the renderer's rows; without them (a bank built before a
    // surface, or Mars) they are the anchor-linear law, as everything here was.
    void PlaceOfRing(double wx, double wz, double& latDeg, double& lonDeg) const;
    WaveChart m_waveChart;   // M13 step 2: the cascade sea's planes, as the hull's twin has them

    std::wstring m_shaderDir;
    SeaLayer* m_sea = nullptr;
    SweSolver* m_swe = nullptr;
    const WaterAtlas* m_atlas = nullptr;
    Compositor* m_comp = nullptr;
    int m_hgtCh = -1;
    uint32_t m_hgtWinSrv = 0xFFFFFFFFu, m_hgtWinResSrv = 0xFFFFFFFFu;
    const SurfaceFrame* m_surface = nullptr;   // the world.flat chart (PHASE C5: exact)
    const class GisVectorMask* m_mask = nullptr;   // SetMask
    uint64_t m_cbFp = 0;   // M12 step 4b: the [kernel] waterbank cb fingerprint's last value
    const GlobeModel* m_globe = nullptr;
    const class WaveField* m_wave = nullptr;   // M8: the solved wave field (optional)
    uint32_t m_wavePages = UINT32_MAX, m_wavePagesRes = UINT32_MAX;   // M9bc
    double m_waveOrgPx[2] = {0.0, 0.0};
    double m_waveWinPx[2] = {0.0, 0.0};
    uint32_t m_waveNx = 0, m_waveNy = 0;
    const struct WaterSceneConfig* m_scene = nullptr;   // M8: live scene closures
    float m_boatA[32] = {}, m_boatB[32] = {};           // M8: the fleet (zeros = off)

    TileAtlas2D m_column;                     // R32_UINT, four bands of rings: the Eulerian columns
    struct ColumnRing {                       // a ring under ring 0 (WaterBankLayer.h, the ladder)
        TileAtlas2D atlas;                    // R32_UINT, four bands tall
        GpuTexture flags;                     // tiles x tiles: the tiles the ring above flagged
        uint32_t flagUav = 0xFFFFFFFFu;
        uint32_t texels = 0, tiles = 0;
        std::vector<uint8_t> mapped;          // tiles x tiles
        uint32_t mappedCount = 0, logged = 0xFFFFFFFFu;
    };
    ColumnRing m_under[kColumnRingsUnderMax];
    uint32_t m_ringsUnder = 0;
    hal::RegionReadback m_flagRb;             // every ring's flags and the spray's counters, back through the frame ring
    Com<ID3D12Resource> m_sprayBuf;           // kSprayMax x 2 float4
    uint32_t m_sprayUav = 0xFFFFFFFFu, m_spraySrv = 0xFFFFFFFFu;
    D3D12_RESOURCE_STATES m_sprayState = D3D12_RESOURCE_STATE_UNORDERED_ACCESS;
    GpuTexture m_sprayCount;                  // 4x1: spawned this frame, alive this frame, the running total
    uint32_t m_sprayCountUav = 0xFFFFFFFFu;
    hal::Pso m_sprayStep;
    hal::Pso m_columnClear;                   // CsColumnClear: the mapped tiles of every ring under to absent, every frame
    uint32_t m_sprayAlive = 0, m_spraySpawned = 0, m_sprayLogFrame = 0;
    double m_lastSimUnix = 0.0;
    hal::Pso m_scatter;                       // CsBankScatter (after the fill, a UAV barrier between)
    TileAtlas2D m_disp, m_param, m_detail;   // detail: per-tile sea-state context the PS
                                             // needs to recover sub-ring sparkle (hsScale;
                                             // churn joins it next)
    hal::RootSignatureRef m_rs;
    hal::Pso m_fill;
    // PHASE B0: the bank's bed trace (traceOn above): its kernel, its target and that target's
    // bindless UAV, and the tile list of the fill it last traced.
    hal::Pso m_traceK;
    GpuTexture m_traceTex;
    uint32_t m_traceUav = 0xFFFFFFFFu;
    std::vector<BankTile> m_traceTiles;
    float m_traceFloorUsed = 0.0f;
    ShaderCompiler* m_sc = nullptr;
    SurfaceFrame::KernelWindowRows m_hw{};   // PHASE B2: SetWindows
    float m_sv[16] = {};                     // ...and the solver's chart
    double m_hwEye[3] = {};
    bool m_hwOn = false;
    float m_baseTexelM = 4.8f;
    float m_orgX[kMips] = {}, m_orgZ[kMips] = {};
    bool m_orgValid[kMips] = {};
    uint8_t m_maskWet[kMips][kRingTiles * kRingTiles] = {};   // the mask's answer where the ring stands
    uint8_t m_maskKnown[kMips][kRingTiles * kRingTiles] = {};   // answered (else asked, and wet meanwhile)
    std::shared_ptr<MaskJobs> m_maskJobs = std::make_shared<MaskJobs>();
    bool m_provisional[kMips] = {};   // a height read answered coarser than asked at the last test
    uint64_t m_landSeen[kMips] = {};   // the height tree's loads when each ring's land test last ran
    uint8_t m_wet[kMips][kRingTiles * kRingTiles] = {};   // per logical tile
    double m_simUnix = 0, m_camX = 0, m_camZ = 0;   // the eye in the chart
    double m_camH = 0;
    GpuTexture m_fold, m_foldDist;   // the fold grid (R32_UINT) and its distance (R32_FLOAT), 64 x 64
    uint32_t m_foldUav = 0, m_foldDistUav = 0, m_foldDistSrv = 0;
    std::vector<uint16_t> m_foldCells;
    hal::Pso m_foldDistPso;   // ...and its height over the chart's tangent plane (P.up - R): ChartRows' row 3 w
    double m_eyeFlat[3] = {};                       // ...and in the flat frame
    Space::Anchor m_chart{};                        // the rings' plane (Chart above)
    OriginLattice::Origin m_origin{};               // ...which is this origin of the planet's cloud
    bool m_originSet = false;
    bool m_chartSet = false;
    float m_tidePlane = 0.0f;   // SetTidePlane: the plane the solver's deviation is measured from
    float m_waterline = 0.0f;   // SetWaterline: the plane the param plane's level is measured from
    D3D12_RESOURCE_STATES m_state = D3D12_RESOURCE_STATE_UNORDERED_ACCESS;
    bool m_ready = false;
};

}  // namespace ga
