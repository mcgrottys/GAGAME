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

#include "core/OceanFft.h"
#include "core/TileAtlas.h"
#include "scene/Layer.h"
#include "sim/BathyModel.h"
#include "sim/CurrentModel.h"
#include "sim/SeaState.h"
#include "sim/SweSolver.h"

#include <string>
#include <vector>

namespace ga {

class SeaLayer : public Layer {
public:
    static constexpr uint32_t kPatches = 64;    // tessellated patches per side (M5b)
    static constexpr uint32_t kSpecSamples = 96;

    void Configure(const std::wstring& shaderDir, const SeaState* sea) {
        m_shaderDir = shaderDir;
        m_sea = sea;
    }

    // M3: the ACT0816 tidal clock drives the entrance jet and its wave steepening.
    void SetCurrents(const CurrentModel* currents) {
        m_currents = currents;
        m_ctSta = currents ? currents->StationIndex("ACT0816") : -1;
    }

    // M5: the bed. heightSrv = the terrain layer's CUDEM texture; geo in world metres.
    void SetBathy(uint32_t heightSrv, float x0, float z0, float sizeX, float sizeZ) {
        m_bathySrv = heightSrv;
        m_bathyGeo[0] = x0;
        m_bathyGeo[1] = z0;
        m_bathyGeo[2] = 1.0f / sizeX;
        m_bathyGeo[3] = 1.0f / sizeZ;
    }

    // M5c: the shallow-water solver (owned by main; recorded into this layer's command list
    // each frame) and the CPU bathy grid the swell-shadow march walks.
    void SetSwe(SweSolver* swe) { m_swe = swe; }
    void SetBathyCpu(const BathyModel* bm) { m_bathyCpu = bm; }
    uint32_t ChurnTiles() const { return m_churnReady ? m_churn.ResidentCount() : 0; }
    uint64_t ChurnBytes() const { return m_churnReady ? m_churn.ResidentBytes() : 0; }

    const char* Name() const override { return "sea"; }
    void Init(Gpu& gpu, ShaderCompiler& sc, FieldSet& fields,
              ID3D12RootSignature* rootSig) override;
    void ReloadShaders(Gpu& gpu, ShaderCompiler& sc) override;
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
    float foamIntensity = 1.0f;
    float targetEdgePx = 12.0f;  // tessellated triangle edge target, screen pixels
    // The CUDEM window holds roughly a third of the real tidal prism, so the solved currents run
    // ~3x under the ACT0816 predictions; this gain (calibrated from the --swe-cycle run, peak
    // ACT / peak solved) restores the magnitude while the solver keeps the spatial shape.
    float sweCurrentGain = 3.2f;
    float heightScale = 1.15f;   // vertical exaggeration; vqview shipped 1.15 as its look
    bool atlasVisualize = false;   // V key: draw the tile grid + residency over the water

private:
    struct SeaCbData {
        float sea[4];       // seaLevel, gridSpan, foamIntensity, skirtStart
        float snap[4];
        uint32_t dispSrv[4];
        uint32_t derivSrv[4];
        float patchL[4];    // xyz sizes, w = quads per side
        float fadeD[4];
        float jet[4];       // signed speed, half width, seaward decay, enabled
        float jetDir[4];    // flood-toward xy, ebb-toward xy
        float waveC[4];     // c0 peak, peak dir xy, advection wrap time
        float bandK[4];     // representative WAVENUMBER per cascade (rad/m); the shader derives
                            // phase speed from the local depth (M5b)
        uint32_t churnU[4]; // churn SRV, residency-mask SRV, visualize, unused
        float churnF[4];    // atlas origin xy, 1/domain, churn foam gain
        float churnF2[4];   // tile world size xy, tile counts xy
        uint32_t bathyU[4]; // CUDEM heightfield SRV (0xFFFFFFFF = open-ocean mode)
        float bathyGeo[4];  // world x0, z0, 1/sizeX, 1/sizeZ
        uint32_t sweU[4];   // M5c: eta SRV, uv SRV, solver on, swell-shadow mask SRV
        float sweF[4];      // bathy grid dims xy, 1 / eta-atlas padded dims zw
        float sweG[4];      // x = prism-truncation current gain
    };
    // Mirrored in shaders/SeaChurn.hlsl. (Count float4 rows on BOTH sides after any edit -- a
    // shader field without its mirror here reads garbage past the push; see the gSweG incident.)
    struct ChurnCbData {
        float originX, originZ, texelM, domainM;
        uint32_t tilesX, tileW, tileH, listCount;
        float dt, tau, pad0, pad1;
        float jetA[4];
        float jetB[4];
        float misc[4];   // x = chop-band wavenumber (M5c; was deep phase speed)
        float waveD[4];
        float bathyG[4]; // M5c: CUDEM world x0, z0, 1/sizeX, 1/sizeZ
        float sweM[4];   // M5c: solved-field on, current gain, seaward blend x-range
    };
    struct SpecCbData {
        float rect[4];
        float axis[4];      // fMax, sMax, nSamples, hasBuoy
        float colM[4];
        float colB[4];
        float model[kSpecSamples / 4][4];
        float buoy[kSpecSamples / 4][4];
    };

    bool BuildPsos(Gpu& gpu, ShaderCompiler& sc);

    std::wstring m_shaderDir;
    const SeaState* m_sea = nullptr;
    ID3D12RootSignature* m_rootSig = nullptr;
    Com<ID3D12PipelineState> m_seaPso, m_specPso;
    OceanFft m_fft;

    SeaCbData m_seaCb{};
    SpecCbData m_specCb{};
    int m_lastHour = -1;
    float m_tSec = 0;
    bool m_haveData = false;
    float m_stormHs = 0, m_stormTp = 10, m_stormDir = 90;
    float m_windGate = 1.0f;   // Monahan-style wind gate on whitecap coverage
    Gpu* m_gpu = nullptr;
    const CurrentModel* m_currents = nullptr;
    int m_ctSta = -1;
    uint32_t m_bathySrv = UINT32_MAX;
    float m_bathyGeo[4] = {0, 0, 1, 1};
    float m_cPeak = 10.0f;     // peak-partition phase speed for the amplification factor
    float m_peakDirX = -1.0f, m_peakDirZ = 0.0f;

    // ---- M4: the churn atlas (16 x 16 km virtual at 2 m texels; resident only where breaking)
    static constexpr float kChurnDomainM = 16384.0f;
    static constexpr float kChurnTexelM = 2.0f;
    static constexpr double kChurnTau = 90.0;      // sim-seconds of streak memory
    void InitChurn(Gpu& gpu, ShaderCompiler& sc);
    void UpdateChurnResidency(Gpu& gpu, double simUnix, double signedMs);
    void RecordChurn(const FrameContext& ctx);

    // ---- M5c: solver coupling + the swell-shadow mask (CPU line-of-sight march)
    static constexpr uint32_t kShadowN = 160;
    void BuildShadowMask(Gpu& gpu, float waterNavd);

    SweSolver* m_swe = nullptr;
    const BathyModel* m_bathyCpu = nullptr;
    GpuTexture m_shadowTex;
    std::vector<uint8_t> m_shadowCpu;
    float m_shadowDirX = 0, m_shadowDirZ = 0, m_shadowLevel = 0;
    bool m_shadowBuilt = false;
    double m_simUnix = 0;

    TileAtlas2D m_churn;
    Com<ID3D12RootSignature> m_churnRs;
    Com<ID3D12PipelineState> m_churnClear, m_churnUpdate;
    uint32_t m_churnTable = UINT32_MAX;    // [t1 chop deriv, t2 swe uv, t3 bathy, u0 churn]
    bool m_churnSweWired = false;          // t2/t3 start as null views; wired when the solver is
    GpuTexture m_maskTex;                  // tilesX x tilesY R8: residency for the visualizer
    std::vector<uint8_t> m_maskCpu;
    std::vector<double> m_lastActive;
    std::vector<uint32_t> m_pendingClear;
    D3D12_RESOURCE_STATES m_churnState = D3D12_RESOURCE_STATE_UNORDERED_ACCESS;
    ChurnCbData m_churnCb{};
    double m_prevChurnT = 0;
    float m_churnDt = 0;
    bool m_churnReady = false;
    bool m_maskDirty = false;
};

}  // namespace ga
