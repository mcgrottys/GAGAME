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
#include "compose/WaterAtlas.h"
#include "core/TileAtlas.h"
#include "scene/Layer.h"
#include "sim/GlobeModel.h"
#include "sim/SeaState.h"
#include "sim/SweSolver.h"

#include <string>

namespace ga {

class SeaLayer;

class WaterBankLayer : public Layer {
public:
    static constexpr int kMips = 6;         // 4.8 m .. 154 m texels, 2.5 .. 79 km spans
    static constexpr int kRingTiles = 4;    // 4x4 logical tiles per ring
    static constexpr int kTileTexels = 128;
    static constexpr int kRingTexels = kRingTiles * kTileTexels;

    void Configure(const std::wstring& shaderDir, SeaLayer* sea, SweSolver* swe,
                   const BathyModel* sweBathy, const WaterAtlas* atlas, Compositor* comp,
                   int hgtCh, const GlobeModel* globe, const SeaState* seaState);

    const char* Name() const override { return "waterbank"; }
    void Init(Gpu& gpu, ShaderCompiler& sc, FieldSet& fields,
              ID3D12RootSignature* rootSig) override;
    void ReloadShaders(Gpu& gpu, ShaderCompiler& sc) override;
    void Render(const FrameContext& ctx) override;   // ring update + fill dispatch

    // Per frame, BEFORE RenderFrame (the SeaLayer::SetTime pattern): ring anchors follow
    // the camera and tile residency commits here, so consumers bind THIS frame's origins.
    void SetFrame(Gpu& gpu, double simUnix, double camX, double camZ);

    uint32_t DispSrv() const { return m_disp.Srv(); }
    uint32_t ParamSrv() const { return m_param.Srv(); }
    uint32_t DetailSrv() const { return m_detail.Srv(); }
    // Ring georef for consumers: per-mip window origin (world m) + base texel size.
    void RingOrigin(int m, float& x, float& z) const { x = m_orgX[m]; z = m_orgZ[m]; }
    // M7j --trace: read the ACTUAL bank texel at a world point -- the hypervisor's
    // cross-check between the CPU's expectation and what the GPU wrote.
    void TraceProbe(Gpu& gpu, double wx, double wz);
    // M7k --dump-fibers: export the three bank planes as PNGs + validate the value
    // ranges against the AST's declarations -- the hypervisor for whole fields.
    void DumpFibers(Gpu& gpu);
    float BaseTexelM() const { return m_baseTexelM; }
    uint32_t ResidentTiles() const {
        return m_disp.ResidentCount() + m_param.ResidentCount() + m_detail.ResidentCount();
    }
    uint64_t ResidentBytes() const {
        return m_disp.ResidentBytes() + m_param.ResidentBytes() + m_detail.ResidentBytes();
    }

    bool enabled = true;
    std::string stats;

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
    };
    struct BankTile {
        float orgXZ[2];
        float texelM;
        uint32_t dstX, dstY;
        float lvl[4];
        float bed[4];
        float hsScale;
        float pad[3];
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
    const GlobeModel* m_globe = nullptr;
    const SeaState* m_seaState = nullptr;

    TileAtlas2D m_disp, m_param, m_detail;   // detail: per-tile sea-state context the PS
                                             // needs to recover sub-ring sparkle (hsScale;
                                             // churn joins it next)
    Com<ID3D12RootSignature> m_rs;
    Com<ID3D12PipelineState> m_fill;
    float m_baseTexelM = 4.8f;
    float m_orgX[kMips] = {}, m_orgZ[kMips] = {};
    bool m_orgValid[kMips] = {};
    uint8_t m_wet[kMips][kRingTiles * kRingTiles] = {};   // per logical tile
    double m_simUnix = 0, m_camX = 0, m_camZ = 0;
    D3D12_RESOURCE_STATES m_state = D3D12_RESOURCE_STATE_UNORDERED_ACCESS;
    bool m_ready = false;
};

}  // namespace ga
