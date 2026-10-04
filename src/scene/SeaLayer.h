// ================================================================================================
//  SeaLayer - M2: the open sea at 1:1 scale, riding the analytic tide.
//
//  Owns the OceanFft compute chain and two draws: the camera-centred displaced grid, and the
//  spectrum-comparison overlay (solid = model S(f) the cascades were synthesised from, dashed =
//  buoy 44013's measured density). The layer's numbers for the title bar -- model Hs vs buoy Hs
//  -- are computed from the SAME partition parameters handed to the GPU, so the readout cannot
//  drift from the synthesis.
// ================================================================================================
#pragma once

#include "compose/Compositor.h"
#include "compose/ExposureSource.h"
#include "compose/SurfaceFrame.h"
#include "core/OceanFft.h"
#include "hal/TileAtlas.h"
#include "hal/Views.h"
#include "core/GradeField.h"
#include "scene/Layer.h"
#include "sim/BathyModel.h"
#include "sim/CurrentModel.h"
#include "sim/OceanCpu.h"
#include "sim/SeaState.h"
#include "sim/SweSolver.h"

#include <string>
#include <vector>

namespace ga {

class SeaLayer : public Layer {
public:
    void Configure(const std::wstring& shaderDir, const SeaState* sea) {
        m_shaderDir = shaderDir;
        m_sea = sea;
    }

    // M3: the ACT0816 tidal clock drives the entrance jet and its wave steepening.
    void SetCurrents(const CurrentModel* currents) {
        m_currents = currents;
        m_ctSta = currents ? currents->StationIndex("ACT0816") : -1;
    }

    // M5: the bed's survey window, geo in world metres. heightSrv only says a window exists:
    // the bed itself is the height megatexture (Assembly passes 0).
    void SetBathy(uint32_t heightSrv, float x0, float z0, float sizeX, float sizeZ) {
        (void)heightSrv;
        m_bathyGeo[0] = x0;
        m_bathyGeo[1] = z0;
        m_bathyGeo[2] = 1.0f / sizeX;
        m_bathyGeo[3] = 1.0f / sizeZ;
    }

    // M5c: the shallow-water solver (owned by main; recorded into this layer's command list
    // each frame) and the CPU bathy grid the swell-shadow march walks.
    void SetSwe(SweSolver* swe) { m_swe = swe; }
    // PHASE B2: the churn's windows (the camera's world's rows) and the camera's eye, planet frame.
    void SetChurnWindows(const SurfaceFrame::ChainRows& rows, const double eye[3]) {
        SurfaceFrame::KernelRows(rows, m_churnHw);
        if (!m_surface) return;
        const SurfaceFrame& sf = *m_surface;
        float st[16];
        sf.StandingKernel(eye, st);   // the solver's standing window about the churn's frame
        memcpy(m_churnCb.stU, st, sizeof(st));
        const double* ax[3] = {sf.east, sf.up, sf.north};
        for (int c = 0; c < 3; ++c) {
            m_churnEyeT[c] = static_cast<float>(eye[0] * ax[c][0] + eye[1] * ax[c][1] + eye[2] * ax[c][2] -
                                                (c == 1 ? sf.planetR : 0.0));
        }
        m_churnEyeT[3] = static_cast<float>(sf.planetR);
    }
    // The height tenant's array and residency map, the churn's bed (PHASE B3: its windows' slices).
    void SetHeightPage(hal::Resource heightArr, hal::Resource resMapArr, uint32_t mips) {
        m_hgtArr = heightArr;
        m_hgtRes = resMapArr;
        m_hgtMips = mips;
    }
    // M12 step 4b: the surface, for the world.flat chart the churn's geoA row is cast from
    // (SurfaceFrame::FlatRows). Must precede the first churn update.
    void SetSurface(const SurfaceFrame* s) { m_surface = s; }
    // M6i's composed channels + survey masks are the renderer's one surface buffer (b2)
    // since M12 step 4g: the same rows the globe reads, from the same upload.
    uint32_t ChurnTiles() const { return m_churnReady ? m_churn.ResidentCount() : 0; }
    // M7e: the bank reads the foam MEMORY -- advected churn joins the one water's fiber.
    uint32_t ChurnAtlasSrv() const { return m_churnReady ? m_churn.Srv() : 0xFFFFFFFFu; }
    // M9az: the churn window's world origin (a tile multiple, camera-following) and the
    // domain's inverse span -- consumers test the window with these and sample the toroidal
    // atlas at frac(world / domain).
    float ChurnOriginX() const { return m_churnOrgX; }
    float ChurnOriginZ() const { return m_churnOrgZ; }
    static float ChurnDomainM() { return kChurnDomainM; }
    uint64_t ChurnBytes() const { return m_churnReady ? m_churn.ResidentBytes() : 0; }
    // M9ba: the swell EXPOSURE is a tree node read as a page tenant (swell.exposure, the z14
    // slice at mips >= 3); the bank and the sea read the same pages. One-water once lost this
    // edge silently -- the GA AST's orphan rule exists because of it.
    void SetExposurePage(uint32_t arrSrv, uint32_t resSrv, const ExposureSource* src) {
        m_expSrv = arrSrv;
        m_expRes = resSrv;
        m_exposure = src;
    }
    uint32_t ExposureSrv() const { return m_expSrv; }
    uint32_t ExposureResSrv() const { return m_expRes; }
    // M7p: the peak propagation direction, for the bank's wave-current amplification.
    bool PeakDirValid() const { return m_peakDirValid; }
    float PeakDirX() const { return m_peakDirX; }
    float PeakDirZ() const { return m_peakDirZ; }
    // M8 foamlaw: unit-sea rms envelope per cascade band (sqrt(2 m0), exaggerated) --
    // the bank kernel scales by its per-texel gains for the depth-excess trigger and
    // the crest gate; the globe PS scales the same way for the tanh peak shaping.
    float BandRms(int c) const { return m_bandRms[c]; }
    // M9c: the wavenumber the FOLD should judge this band by -- energy-weighted, not the
    // band's geometric midpoint. Only the fold weight reads it; the physics keeps gBandK.
    float BandKFold(int c) const { return m_bandKFold[c]; }
    // M9bt: the band's energy-weighted log-WIDTH -- the second moment the fold needs to
    // answer with a fraction instead of a yes or a no.
    float BandKSpread(int c) const { return m_bandKSpread[c]; }
    // M8 wavefield: the live partition set, for the solved-field bucket key + spectrum.
    const PartParam* Parts() const { return m_parts; }
    // M8: the Monahan wind gate -- whitecap COVERAGE follows wind speed; the Jacobian
    // only says where foam sits. The bank's steepness trigger multiplies by this (a
    // 4 m/s breeze must not micro-break the whole sea -- the Sea.hlsl law, restored
    // when M8b brought the long-dead cascade foam path back to life).
    float WindGate() const { return m_windGate; }
    // M8: a --storm override makes the GFS grid stale by definition -- consumers that
    // ratio local grid Hs against the reference must treat the storm AS the reference.
    bool StormOn() const { return m_stormHs > 0.01f; }
    // The forecast the cascades were synthesised from (the reference WaveScale divides by).
    const SeaState* State() const { return m_sea; }
    // M9ba --trace: the CPU mirror of the page read -- the node itself, at the page's
    // ~76 m grain, so the hypervisor prints what the GPU will see.
    float ShadowAtWorld(float x, float z) const {
        if (!m_exposure) return 1.0f;
        return m_exposure->At(BathyModel::kOrgLat + z / BathyModel::kMPerLat,
                              BathyModel::kOrgLon + x / BathyModel::kMPerLon);
    }

    const char* Name() const override { return "sea"; }
    void Init(Gpu& gpu, ShaderCompiler& sc, FieldSet& fields,
              hal::RootSignature rootSig) override;
    void ReloadShaders(Gpu& gpu, ShaderCompiler& sc) override;
    void Simulate(const FrameContext& ctx) override;   // the solver's step, for whoever asked
    void Render(const FrameContext& ctx) override;

    // Once per frame, before RenderFrame. seaLevelM = the tide; cam XZ centres the grid.
    void SetTime(double simUnix, double seaLevelM, double camX, double camZ);

    // Sandbox override: ignore the forecast and synthesise one sea state (swell + a wind-sea
    // fraction). hs <= 0 returns to the forecast.
    void SetStorm(float hs, float tp, float fromDeg);

    double MeasureRenderedHs(Gpu& gpu) { return m_fft.MeasureHs(gpu); }

    double hsModel = 0;      // 4 sqrt(m0) of the active parameterization
    double hsBuoy = 0;       // latest 44013 observation
    int activeParts = 0;
    std::string statusNote;  // "f012" style label for the title bar
    std::string currentStatus;   // "ebb 0.72 m/s" for the title bar
    std::string atlasStats;      // "churn 34/2048 t 2.1 MB" for the title bar
    // The CUDEM window holds roughly a third of the real tidal prism, so the solved currents run
    // ~3x under the ACT0816 predictions; this gain (calibrated from the --swe-cycle run, peak
    // ACT / peak solved) restores the magnitude while the solver keeps the spatial shape.
    float sweCurrentGain = 3.2f;
    float heightScale = 1.15f;   // vertical exaggeration; vqview shipped 1.15 as its look

    // M9bq: the cascades, evaluated on the CPU for the hull. Owned here because this is where
    // the sea state is decided, and fed in the same breath as the GPU's copy (SeaLayer.cpp).
    const OceanCpu& Ocean() const { return m_oceanCpu; }
    // M9a: fill the missing wind sea from the GFS wind (data/wave_scene.json; 0 = off)
    float windSeaFill = 1.0f;
    // M9c: how far the fold's band wavelength follows the SPECTRUM instead of the band's
    // geometric midpoint. 0 = the shipped constant, byte for byte; 1 = fully energy-weighted.
    float bandFoldWeight = 1.0f;
    // M8 buoy assimilation closures (data/wave_scene.json; main mirrors them here)
    float buoyAssimAgeH = 6.0f;
    float buoyAssimGainMax = 1.8f;
    uint32_t FftDispSrv(int c) const { return m_fft.DispSrv(c); }
    uint32_t FftDerivSrv(int c) const { return m_fft.DerivSrv(c); }
    float FftPatchL(int c) const { return m_fft.PatchL(c); }
    // Harness only: raised by main for the frames a still is HELD at one instant
    // (--settle-sync / --settle-hold). The churn is a stateful atlas -- at a held instant
    // dtSim is 0 and CsChurnUpdate reduces to max(old, src), so the foam can only CLIMB, once
    // per held frame, against a bed that is still landing. MEASURED (bird, 2026-09-05): two
    // holds of different length gave different foam, which is what made two settled stills
    // incomparable across binaries. Frozen, the atlas is whatever the last unheld frame left
    // and hold length stops being a term. The CLEAR kernel still runs, so a tile mapped during
    // the hold is never read as undefined pool memory. Proof of wire: [gpu] sea.churn 0.000 ms
    // on held frames (p50 0.000 over 540 frames of a 240 + 300 render, 0.178 unheld).
    bool freezeChurn = false;
    // Harness only (--settle-clear-churn, step 25 of docs/PERF_EXPERIMENT.md): zero the whole
    // churn atlas on the next Update by putting every resident tile on the clear list the
    // fresh-tile clear kernel already runs. Raised once, at the first held frame, so a held
    // still carries NO foam history from the real frames before the hold: --settle-exact
    // makes the resident set a function of the pose, but the churn's deposits depend on WHEN
    // the bed and the wave pages landed during those frames -- the A/B that tells "the
    // residency" from "the churn's history" is this render against one without the flag.
    // MEASURED (helm_ebb, 19:30, 2026-09-05, out/step25): two --settle-exact runs differed on
    // 17.1 % of the pixels; with the atlas cleared, 10.0 % -- the churn's history is part of
    // the residual, and the SWE's own state (level within 1.5 cm, current within 3 cm/s over
    // 38-65 % of the inlet cells, exported by --dump-water-state) is the rest. Returns the
    // tiles put on the list, for the proof-of-wire line main prints.
    uint32_t ClearChurn() {
        const std::vector<uint32_t>& r = m_churn.ResidentList();
        m_pendingClear.insert(m_pendingClear.end(), r.begin(), r.end());
        return static_cast<uint32_t>(r.size());
    }

private:
    // Mirrored in shaders/SeaChurn.hlsl. (Count float4 rows on BOTH sides after any edit -- a
    // shader field without its mirror here reads garbage past the push; see the gSweG incident.)
    struct ChurnCbData {
        float originX, originZ, texelM, domainM;
        uint32_t tilesX, tileW, tileH, listCount;
        float dt, tau, pad0, pad1;
        float jetA[4];
        float jetB[4];
        float miscC[4];  // x = chop-band wavenumber (M5c; was deep phase speed); gMiscC
        float waveD[4];
        float bathyG[4]; // M5c: CUDEM world x0, z0, 1/sizeX, 1/sizeZ
        float sweM[4];   // M5c: solved-field on, current gain, seaward blend x-range
        // M9ar: THE BED IS THE HEIGHT MEGATEXTURE. world -> lat/lon (orgLat, orgLon, 1/mPerLat,
        // 1/mPerLon) and the page frame (org px x, y, 1/16384, world px at z14). Appended LAST
        // -- and M9ax found them inserted BEFORE sweM on this side only: same bytes, every row
        // from gSweM on rotated (the churn read its current gain from the longitude for a
        // week; priors 22). The order here IS the shader's.
        float geoA[4];
        // M9az: THE WINDOW. The atlas is addressed TOROIDALLY on a world-anchored tile lattice
        // (slot = world tile mod atlas tiles), and the domain is the +-8 km window around the
        // camera: x, y = the world tile index of the window's origin, z = atlas tiles in y.
        float window[4];
        // PHASE B2: THE WINDOWS THE CHURN STANDS IN -- the camera's world's (slot 0), its chain at
        // each texel's own point: the flat world point about the camera's eye (eyeT: the eye in the
        // tangent axes less (0, R, 0); w = R). Appended at the END on both sides.
        float eyeT[4];
        float hwU[20];
        float hwV[20];
        float hwW[20];
        float hwO[12];
        uint32_t hwS[8];
        float stU[4], stV[4], stW[4], stO[4];   // the solver's standing window about the churn's frame; LAST
    };
    std::wstring m_shaderDir;
    const SeaState* m_sea = nullptr;
    OceanFft m_fft;
    OceanCpu m_oceanCpu;   // M9bq: the hull's copy of the same three cascades

    // What the churn and the solver take from SetTime: the level, the entrance jet, the chop band's
    // wavenumber and the churn tiles' span.
    float m_seaLevel = 0.0f;
    float m_jet[4] = {}, m_jetDir[4] = {};
    float m_chopK = 0.0f;
    float m_churnSpan[2] = {1.0f, 1.0f};
    int m_lastHour = -1;
    float m_tSec = 0;
    bool m_haveData = false;
    float m_stormHs = 0, m_stormTp = 10, m_stormDir = 90;
    float m_windGate = 1.0f;   // Monahan-style wind gate on whitecap coverage
    Gpu* m_gpu = nullptr;
    const CurrentModel* m_currents = nullptr;
    int m_ctSta = -1;
    float m_bathyGeo[4] = {0, 0, 1, 1};
    float m_cPeak = 10.0f;     // peak-partition phase speed for the amplification factor
    float m_peakDirX = -1.0f, m_peakDirZ = 0.0f;
    float m_bandRms[3] = {};   // M8: unit-sea rms envelope per band (sqrt(2 m0) * exag)
    // M9bt: defaults are the cut widths under a flat spread, range/sqrt(12) in ln k, so a
    // spectrum that never loads folds exactly as a uniformly-filled band would.
    float m_bandKSpread[3] = {0.732f, 0.465f, 0.933f};
    float m_bandKFold[3] = {0.0209f, 0.2339f, 2.8420f};   // M9c: fold wavenumbers; the cut
                                                          // means until the first SetTime
    PartParam m_parts[4] = {};   // M8: the live partition set (wavefield spectrum input)

    // ---- M4: the churn atlas (16 x 16 km virtual at 2 m texels; resident only where breaking)
    static constexpr float kChurnDomainM = 16384.0f;
    static constexpr float kChurnTexelM = 2.0f;
    static constexpr double kChurnTau = 90.0;      // sim-seconds of streak memory
    void InitChurn(Gpu& gpu, ShaderCompiler& sc);
    void UpdateChurnResidency(Gpu& gpu, double simUnix, double signedMs);
    void RecordChurn(const FrameContext& ctx);


    SweSolver* m_swe = nullptr;
    bool m_peakDirValid = false;   // M7j: never march the default direction
    // M9ba: the exposure page tenant (array SRV, residency SRV) and the node behind it.
    uint32_t m_expSrv = UINT32_MAX, m_expRes = UINT32_MAX;
    const ExposureSource* m_exposure = nullptr;
    double m_simUnix = 0;

    // M9h: the first bank ported to GradeBank. Driven MANUALLY -- the churn policy is
    // hysteretic (a tile stays warm several decay constants past the last breaking, or the
    // memory this field exists to carry is deleted the instant the surf stops) and resets
    // wholesale on a time scrub. GradeBank carries the descriptor, the grade signature and the
    // resident-list discipline; the loop below stays honest about its own state.
    GradeBank m_churn;
    hal::RootSignatureRef m_churnRs;
    hal::Pso m_churnClear, m_churnUpdate;
    hal::Table m_churnTable;               // [t1 chop deriv, t2 swe uv, t3 height page,
                                           //  t4 its residency map (M9ar), u0 churn]
    hal::Resource m_hgtArr = nullptr;    // M9ar: borrowed from the residency manager
    hal::Resource m_hgtRes = nullptr;
    uint32_t m_hgtMips = 7;
    const SurfaceFrame* m_surface = nullptr;   // M12 step 4b: the world.flat chart (geoA)
    SurfaceFrame::KernelWindowRows m_churnHw{};   // PHASE B2: SetChurnWindows
    float m_churnEyeT[4] = {};
    bool m_churnSweWired = false;          // t2/t3 start as null views; wired when the solver is
    GpuTexture m_maskTex;                  // tilesX x tilesY R8: residency for the visualizer
    std::vector<uint8_t> m_maskCpu;
    std::vector<double> m_lastActive;
    std::vector<uint32_t> m_pendingClear;
    // M9az: which WORLD tile each atlas slot holds (the toroidal window). A slot whose world
    // tile changes hands is stale: cleared if it stays resident, forgotten either way.
    std::vector<int32_t> m_slotWorldX, m_slotWorldY;
    float m_churnOrgX = 0.0f, m_churnOrgZ = 0.0f;
    D3D12_RESOURCE_STATES m_churnState = D3D12_RESOURCE_STATE_UNORDERED_ACCESS;
    ChurnCbData m_churnCb{};
    uint64_t m_churnCbFp = 0;   // M12 step 4b: the [kernel] churn cb fingerprint's last value
    double m_prevChurnT = 0;
    float m_churnDt = 0;
    bool m_churnReady = false;
    bool m_maskDirty = false;
};

}  // namespace ga
