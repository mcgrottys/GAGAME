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
#include "scene/Layer.h"
#include "sim/GlobeModel.h"
#include "sim/SeaState.h"
#include "sim/SweSolver.h"

#include <cstring>
#include <string>

namespace ga {

class SeaLayer;

class WaterBankLayer : public Layer {
public:
    // M9p: --flat-bed. Replaces the sampled bed with a constant so the same scene can be
    // filled twice and diffed. Public because it is a debug lever, not state.
    bool flatBed = false;
    float flatBedNavd = -30.0f;

    static constexpr int kMips = 6;         // texel = base * 2^m; at base 1.2: 1.2..38 m
                                            // texels, 0.6..20 km spans (scene bankTexelM)
    static constexpr int kRingTiles = 4;    // 4x4 logical tiles per ring
    static constexpr int kTileTexels = 128;
    static constexpr int kRingTexels = kRingTiles * kTileTexels;

    void Configure(const std::wstring& shaderDir, SeaLayer* sea, SweSolver* swe,
                   const BathyModel* sweBathy, const WaterAtlas* atlas, Compositor* comp,
                   int hgtCh, const GlobeModel* globe, const SeaState* seaState);

    const char* Name() const override { return "waterbank"; }
    void Init(Gpu& gpu, ShaderCompiler& sc, FieldSet& fields,
              hal::RootSignature rootSig) override;
    void ReloadShaders(Gpu& gpu, ShaderCompiler& sc) override;
    void Render(const FrameContext& ctx) override;   // ring update + fill dispatch

    // Per frame, BEFORE RenderFrame (the SeaLayer::SetTime pattern): ring anchors follow
    // the camera and tile residency commits here, so consumers bind THIS frame's origins.
    void SetFrame(Gpu& gpu, double simUnix, double camX, double camZ);
    // Per frame, BEFORE RenderFrame: the tide plane the solver is forced by this frame (the value
    // SeaLayer::SetTime hands it) -- inside the solver's domain the level IS that plane plus the
    // solver's deviation (the solver is truth), and the kernel needs the plane to say so.
    void SetTidePlane(double navdM) { m_tidePlane = static_cast<float>(navdM); }

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
        float level = 0, sigma2 = 0, curU = 0, curV = 0;
        // The detail plane, as the kernel wrote it: the per-band gains of the FULL closure (sea-state
        // scale x shadow x shoaling x wave-current, before any window stand-down) and the dry weight.
        float gain0 = 0, gain1 = 0, gain2 = 0, dry = 0;
    };
    void ReadBankPoints(Gpu& gpu, const double* worldXz, int n, BankPoint* out);

    // M7k --dump-fibers: export the three bank planes as PNGs + validate the value
    // ranges against the AST's declarations -- the hypervisor for whole fields.
    void DumpFibers(Gpu& gpu);
    int injectPattern = 0;   // M7m/M7n: 1 = bank world card, 2 = cascade-edge card
    // M7q: the composed height WINDOW, per texel, in the kernel -- the corner-lerp bed
    // quantized depth to ~600 m patches and the M7p physics inherited the blockiness (the
    // data lens showed breaking bands cutting at tile edges; the user called it).
    // M9aq: `slice` != ~0 means srv/resMapSrv are Texture2DArray views of the height PAGE
    // tenant and the window is that slice; ~0 is the old single-face window.
    // M12 step 4b: `window` is the z14 lattice the page sits on (the surface's winH); its
    // Rows() are the kernel's winA row.
    void SetHeightWindow(uint32_t srv, uint32_t resMapSrv, const Lattice& window,
                         uint32_t slice = 0xFFFFFFFFu) {
        m_hgtWinSrv = srv;
        m_hgtWinResSrv = resMapSrv;
        m_hgtWinSlice = slice;
        m_hgtWin = window;
    }
    // M12 step 4b: the surface, for the world.flat chart the geoA row is cast from
    // (SurfaceFrame::FlatRows). Must precede the first Render.
    void SetSurface(const SurfaceFrame* s) { m_surface = s; }
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
    void SetBoats(const float* a32, const float* b32) {
        memcpy(m_boatA, a32, sizeof(m_boatA));
        memcpy(m_boatB, b32, sizeof(m_boatB));
    }
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
        float swe[4];
        float sweDims[4];
        float misc[4];
        uint32_t slotsA[4];
        uint32_t slotsB[4];
        uint32_t slotsC[4];   // x = churn atlas SRV (M7e foam memory)
        float churn[4];       // xy origin, z 1/domain, w atlas texels
        float peakDir[4];     // M7p: peak propagation dir xy, z valid (gPeakDir)
        uint32_t slotsD[4];   // M7q: height window SRV, its residency-map SRV
        float geoA[4];        // world->latlon: orgLat, orgLon, 1/mPerLat, 1/mPerLon
        float winA[4];        // window: org px x, org px y, 1/sizePx, full-world px (z14)
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
    };
    struct BankTile {
        float orgXZ[2];
        float texelM;
        uint32_t dstX, dstY;
        float lvl[4];
        float bed[4];
        float hs[4];   // the local sea-state scale at the corners (sim/WaveScale.h)
    };

    void ReanchorRing(Gpu& gpu, int m, double camX, double camZ);
    bool TileWet(double wx0, double wz0, double spanM) const;
    void CornerParams(double wx, double wz, float& lvl, float& bed) const;

    std::wstring m_shaderDir;
    SeaLayer* m_sea = nullptr;
    SweSolver* m_swe = nullptr;
    const BathyModel* m_sweBathy = nullptr;
    const WaterAtlas* m_atlas = nullptr;
    Compositor* m_comp = nullptr;
    int m_hgtCh = -1;
    uint32_t m_hgtWinSrv = 0xFFFFFFFFu, m_hgtWinResSrv = 0xFFFFFFFFu;
    uint32_t m_hgtWinSlice = 0xFFFFFFFFu;
    Lattice m_hgtWin;   // M12 step 4b: the z14 height window the page sits on (winA = Rows)
    const SurfaceFrame* m_surface = nullptr;   // M12 step 4b: the world.flat chart (geoA)
    uint64_t m_cbFp = 0;   // M12 step 4b: the [kernel] waterbank cb fingerprint's last value
    const GlobeModel* m_globe = nullptr;
    const SeaState* m_seaState = nullptr;
    const class WaveField* m_wave = nullptr;   // M8: the solved wave field (optional)
    uint32_t m_wavePages = UINT32_MAX, m_wavePagesRes = UINT32_MAX;   // M9bc
    double m_waveOrgPx[2] = {0.0, 0.0};
    double m_waveWinPx[2] = {0.0, 0.0};
    uint32_t m_waveNx = 0, m_waveNy = 0;
    const struct WaterSceneConfig* m_scene = nullptr;   // M8: live scene closures
    float m_boatA[32] = {}, m_boatB[32] = {};           // M8: the fleet (zeros = off)

    TileAtlas2D m_disp, m_param, m_detail;   // detail: per-tile sea-state context the PS
                                             // needs to recover sub-ring sparkle (hsScale;
                                             // churn joins it next)
    hal::RootSignatureRef m_rs;
    hal::Pso m_fill;
    float m_baseTexelM = 4.8f;
    float m_orgX[kMips] = {}, m_orgZ[kMips] = {};
    bool m_orgValid[kMips] = {};
    uint8_t m_wet[kMips][kRingTiles * kRingTiles] = {};   // per logical tile
    double m_simUnix = 0, m_camX = 0, m_camZ = 0;
    float m_tidePlane = 0.0f;   // SetTidePlane: the plane the solver's deviation is measured from
    D3D12_RESOURCE_STATES m_state = D3D12_RESOURCE_STATE_UNORDERED_ACCESS;
    bool m_ready = false;
};

}  // namespace ga
