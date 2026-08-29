// ================================================================================================
//  GlobeLayer - M6: the planet, rendered.
//
//  CPU walks a quadtree per cube face each frame (doubles throughout), culls against the view
//  frustum and the planet's own horizon, and emits a flat list of CDLOD nodes; the GPU draws
//  ONE shared 32x32 grid per node, morphing vertices between LOD rings in the vertex shader
//  (Globe.hlsl). Relief is the ETOPO texture, the ocean is shaded from the live GFS-Wave
//  fields. The node list rides root parameter 2 (the FieldSet slot -- rebound by the renderer
//  every frame, and the globe draws last, so borrowing it is safe).
// ================================================================================================
#pragma once

#include "core/Residency.h"
#include "core/TileAtlas.h"
#include "render/Camera.h"
#include "scene/Layer.h"
#include "sim/GlobeModel.h"

#include <string>
#include <vector>

namespace ga {

class GlobeLayer : public Layer {
public:
    static constexpr int kMaxDepth = 6;      // smallest node ~156 km: vertex spacing == texel
    static constexpr double kLodFactor = 3.0;

    void Configure(const std::wstring& shaderDir, const GlobeModel* globe) {
        m_shaderDir = shaderDir;
        m_globe = globe;
    }

    const char* Name() const override { return "globe"; }
    void Init(Gpu& gpu, ShaderCompiler& sc, FieldSet& fields,
              ID3D12RootSignature* rootSig) override;
    void ReloadShaders(Gpu& gpu, ShaderCompiler& sc) override;
    void Render(const FrameContext& ctx) override;

    // Once per frame before RenderFrame: camera in the PLANET frame + projection aspect and
    // viewport height in pixels (the relief-mip selector needs the pixel's angular size).
    void SetView(const Camera& cam, float aspect, float viewportH, double simTime);

    // M6e: streamed planet surface. surf/norm are ResidencyManager tenant ids (-1 = none);
    // isMars switches the whole shading path (no relief, no live-Earth fields, BC5 normals).
    void SetResidency(ResidencyManager* rm, int surf, int norm, bool isMars) {
        m_res = rm;
        m_surfT = surf;
        m_normT = norm;
        m_streamMars = isMars;
    }
    void SetPlanetRadius(double r) { m_radius = r; }
    uint32_t AirTiles() const {
        return (m_windReady ? m_windBank.ResidentCount() : 0) +
               (m_cloudReady ? m_cloud.ResidentCount() : 0);
    }
    uint64_t AirBytes() const {
        return (m_windReady ? m_windBank.ResidentBytes() : 0) +
               (m_cloudReady ? m_cloud.ResidentBytes() : 0);
    }
    // Screw-prefetch: run the node walk for a PREDICTED camera, emitting Want(predicted) only.
    void PredictWants(const Camera& cam, float aspect);

    float reliefExagg = 1.0f;       // set per frame by main (altitude-scaled display choice)
    bool windOverlay = false;       // V key: tint the Mv2 wind bank's curl (violet cyclonic)
    std::string stats;              // "globe 214 nodes  alt 3520 km" for the title bar

private:
    // Mirrors GlobeNode in Globe.hlsl.
    struct NodeData {
        float uv0[2];
        float uvStep[2];
        uint32_t face;
        float morphStart, morphEnd, pad;
    };
    // Mirrors GlobeCb in Globe.hlsl. (Count float4 rows on BOTH sides after any edit.)
    struct GlobeCbData {
        float glo[4];
        float camAbs[4];
        uint32_t texIdx[4];   // relief, hs, wind, cloud VOLUME
        float wavesA[4];
        float wavesB[4];      // nx, ny, pixel angular, max relief mip
        float beacon[4];
        float cloudA[4];      // extinction, shell top m, sun boost, ground-shadow strength
        uint32_t texIdx2[4];  // M6d: NE relief SRV, wind Mv2 bank SRV, overlay on, unused
        float neGeo[4];       // NE window: lon0, lat1, 1/lonSpan, 1/latSpan (degrees)
        float windGeo[4];     // wind grid: lat1, lon1, 1/dlat, 1/dlon
        float windB[4];       // nx, ny, unused, unused
        uint32_t streamU[4];  // M6e: surface cube SRV, normal cube SRV, their residency maps
        float streamF[4];     // surface on, normal on, planet-is-Mars, unused
    };
    // Mirrors WindCb in GlobeWind.hlsl.
    struct WindCbData {
        uint32_t nx, ny, listCount, tilesX;
        uint32_t tileW, tileH, pad0, pad1;
        float lat1, dLatDeg, radius, scale;
    };
    // Mirrors GlobeSkyCb (b2) in Globe.hlsl.
    struct SkyCbData {
        float fwd[4];         // xyz forward, w = tan(fovY/2)
        float right[4];       // xyz right, w = aspect
        float up[4];
    };
    // Mirrors CloudCb in CloudVol.hlsl.
    struct CloudCbData {
        uint32_t volNx, volNy, volNz, listCount;
        uint32_t tilesX, tilesY, tileW, tileH;
        uint32_t tileD, srcNx, srcNy, srcNz;
        float altA[4];
        float altB[4];
        float altC[4];
    };

    void SelectNode(int face, int level, double u0, double v0, double size);
    bool BuildPso(Gpu& gpu, ShaderCompiler& sc);

    void InitClouds(Gpu& gpu, ShaderCompiler& sc);
    void InitNeAndWind(Gpu& gpu, ShaderCompiler& sc);

    std::wstring m_shaderDir;
    const GlobeModel* m_globe = nullptr;
    ID3D12RootSignature* m_rootSig = nullptr;
    Com<ID3D12PipelineState> m_pso, m_skyPso;
    GpuTexture m_relief, m_hs, m_wind, m_cloudSrc, m_ne, m_windSrc;

    // M6d: the sparse Mv2 wind bank (div, u, v, curl) -- resident where storms live.
    TileAtlas2D m_windBank;
    Com<ID3D12RootSignature> m_windRs;
    Com<ID3D12PipelineState> m_windBuild;
    uint32_t m_windTable = UINT32_MAX;   // [t1 wind source SRV, u0 bank UAV]
    bool m_windReady = false;

    // M6c: the sky as a sparse VOLUME bank (R16F, 200 m vertical texels; NULL tile = clear
    // air). 64 deep so the 32-deep hardware tiles split the low sky from the high sky --
    // full-height column tiles made residency degenerate (every 320 km column has SOME cloud).
    static constexpr uint32_t kVolNx = 1024, kVolNy = 512, kVolNz = 64;
    static constexpr float kShellTopM = 12800.0f;
    TileAtlas3D m_cloud;
    Com<ID3D12RootSignature> m_cloudRs;
    Com<ID3D12PipelineState> m_cloudBuild;
    uint32_t m_cloudTable = UINT32_MAX;   // [t1 source SRV, u0 volume UAV]
    bool m_cloudReady = false;

    // M6e: streaming.
    ResidencyManager* m_res = nullptr;
    int m_surfT = -1, m_normT = -1;
    bool m_streamMars = false;
    bool m_predictPass = false;
    double m_radius = GlobeModel::kR;
    float m_pixAng = 1.0e-3f;

    std::vector<NodeData> m_nodes;
    GlobeCbData m_cb{};
    SkyCbData m_skyCb{};
    float m_viewportH = 900.0f;
    double m_camPos[3] = {0, 0, 2.0e7};
    double m_frustum[6][4];          // camera-relative plane equations
    int m_planeCount = 0;
};

}  // namespace ga
