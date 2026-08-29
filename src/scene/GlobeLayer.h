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

#include "compose/Compositor.h"
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
    // M6g: ONE WORLD. The globe renders in the estuary tangent frame; rows are the
    // planet->tangent rotation (east, up-at-origin, north). SetView now receives the FLAT
    // (tangent-frame) camera; the sphere centre sits at flat (0, -R, 0).
    void SetFrame(const double east[3], const double up[3], const double north[3]) {
        for (int i = 0; i < 3; ++i) {
            m_frameE[i] = east[i];
            m_frameU[i] = up[i];
            m_frameN[i] = north[i];
        }
    }
    // The CUDEM window in degrees, for the foundation sink (0 span = absent).
    void SetEstuaryWindow(double lon0, double lat1, double lonSpan, double latSpan) {
        m_estGeo[0] = lon0;
        m_estGeo[1] = lat1;
        m_estGeo[2] = lonSpan;
        m_estGeo[3] = latSpan;
    }
    // M6i: the composed channels -- the tenants realized from the layer compositor's stacks.
    // colorCube/window carry the color channel (cube + Mercator z14 window), heightCube and
    // heightWindow the height channel (same window FRAME as color); -1 = absent. Wants for
    // all of them come from the SAME CDLOD walk.
    void SetComposed(int colorCube, int window, int heightCube, int heightWindow,
                     double orgPxX, double orgPxY, double sizePx) {
        m_colorT = colorCube;
        m_winT = window;
        m_hgtT = heightCube;
        m_hgtWinT = heightWindow;
        m_detOrg[0] = orgPxX;
        m_detOrg[1] = orgPxY;
        m_detSize = sizePx;
    }
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
    float waterNavd = 0.0f;         // M6j: live water level, for the close-up material model
    bool msSurface = true;          // M6j: request the mesh-shader unified surface
    bool MeshPathActive() const { return m_msPath; }
    bool stencilOverlay = false;    // M6i: --stencil, the GIS alignment overlay
    // GIS survey stencil textures (GisStencil), for the --stencil overlay.
    void SetGisStencil(uint32_t winSrv, uint32_t globSrv) {
        m_gisWinSrv = winSrv;
        m_gisGlobSrv = globSrv;
    }
    bool skyPassEnabled = true;     // M6g: off while SkyLayer owns the low-altitude backdrop
    bool windOverlay = false;       // V key: tint the Mv2 wind bank's curl (violet cyclonic)
    bool marsReliefValid = false;   // M6f: the configured model's relief IS Mars (MOLA)
    std::string stats;              // "globe 214 nodes  alt 3520 km" for the title bar

private:
    // Mirrors GlobeNode in Globe.hlsl.
    struct NodeData {
        float uv0[2];
        float uvStep[2];
        uint32_t face;
        float morphStart, morphEnd, pad;
    };
    // Mirrors MeshletRec in GlobeMesh.hlsl (96 B; structured-buffer raw layout).
    struct MeshletRec {
        float uv0[2], uvStepCell[2];
        uint32_t face, cell0;
        float morphStart, morphEnd;
        float anchorRel[3], arc;
        float dPdu[3], pad0;
        float dPdv[3], pad1;
        float upT[3], pad2;
    };
    // Mirrors GlobeCb in Globe.hlsl. (Count float4 rows on BOTH sides after any edit.)
    struct GlobeCbData {
        float glo[4];
        float camAbs[4];
        uint32_t texIdx[4];   // unused (was relief), hs, wind, cloud VOLUME
        float wavesA[4];
        float wavesB[4];      // nx, ny, pixel angular, unused
        float cloudA[4];      // extinction, shell top m, sun boost, ground-shadow strength
        uint32_t texIdx2[4];  // unused (was NE relief), wind Mv2 bank SRV, overlay on
        float windGeo[4];     // wind grid: lat1, lon1, 1/dlat, 1/dlon
        float windB[4];       // nx, ny, unused, unused
        uint32_t streamU[4];  // M6e: Mars native surface/normal cubes + their residency maps
        float streamF[4];     // surface on, normal on, planet-is-Mars, unused
        ComposedSurfaceCb cs; // M6i: the composed channels + the one-world frame (8 rows)
        float estGeo[4];      // CUDEM window deg: lon0, lat1, 1/lonSpan, 1/latSpan (0 = none)
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
    void EmitMeshlets(int face, double u0, double v0, double size, double arc,
                      float morphStart, float morphEnd);
    bool BuildPso(Gpu& gpu, ShaderCompiler& sc);
    bool BuildMeshPso(Gpu& gpu, ShaderCompiler& sc);

    void InitClouds(Gpu& gpu, ShaderCompiler& sc);
    void InitNeAndWind(Gpu& gpu, ShaderCompiler& sc);

    std::wstring m_shaderDir;
    const GlobeModel* m_globe = nullptr;
    ID3D12RootSignature* m_rootSig = nullptr;
    Com<ID3D12PipelineState> m_pso, m_skyPso;
    // M6i: m_relief and m_ne retired -- the composed height cube streams what they carried
    // (and returns ~90 MB of committed equirect memory to the pool).
    GpuTexture m_hs, m_wind, m_cloudSrc, m_windSrc;

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

    // M6e/M6i: streaming -- Mars's native pyramids (surf/norm) + the composed channels.
    ResidencyManager* m_res = nullptr;
    int m_surfT = -1, m_normT = -1;
    int m_colorT = -1, m_winT = -1, m_hgtT = -1, m_hgtWinT = -1;
    uint32_t m_gisWinSrv = UINT32_MAX, m_gisGlobSrv = UINT32_MAX;
    double m_detOrg[2] = {0, 0};
    double m_detSize = 1;
    bool m_streamMars = false;
    bool m_predictPass = false;
    double m_radius = GlobeModel::kR;
    float m_pixAng = 1.0e-3f;
    double m_frameE[3] = {1, 0, 0}, m_frameU[3] = {0, 1, 0}, m_frameN[3] = {0, 0, 1};
    double m_camPlanet[3] = {0, 0, 2.0e7};   // for the horizon cull (doubles, per SetView)
    double m_estGeo[4] = {0, 0, 0, 0};

    std::vector<NodeData> m_nodes;
    // M6j: the mesh-shader path.
    static constexpr uint32_t kMaxMeshlets = 65536;
    bool m_msPath = false;
    Com<ID3D12PipelineState> m_msPso;
    Com<ID3D12GraphicsCommandList6> m_cl6;
    std::vector<MeshletRec> m_meshlets;
    GpuBuffer m_recBuf[Gpu::kFrameCount];
    GlobeCbData m_cb{};
    SkyCbData m_skyCb{};
    float m_viewportH = 900.0f;
    double m_camPos[3] = {0, 0, 2.0e7};
    double m_frustum[6][4];          // camera-relative plane equations
    int m_planeCount = 0;
};

}  // namespace ga
