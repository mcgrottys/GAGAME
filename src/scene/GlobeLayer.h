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
#include "core/GradeField.h"
#include "core/MemGridLoader.h"
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
    // M7: per-frame wave-bank binding (SRVs + ring origins), and the one-water switch.
    void SetWaterBank(uint32_t dispSrv, uint32_t paramSrv, uint32_t detailSrv,
                      const uint32_t derivSrv[3], const float patchL[3],
                      const float bandK[3], const float bandRms[3], const float bandFold[3],
                      float heightScale,
                      float baseTexelM, const float* org12, bool oneWater) {
        m_bankSrv[0] = dispSrv;
        m_bankSrv[1] = paramSrv;
        m_bankSrv[2] = detailSrv;
        for (int i = 0; i < 3; ++i) {
            m_bankDeriv[i] = derivSrv[i];
            m_bankPatch[i] = patchL[i];
            m_bankK[i] = bandK[i];
            m_bankRms[i] = bandRms[i];   // M8: unit-sea rms envelope (peak shaping)
            m_bankFold[i] = bandFold[i];  // M9c: the fold's own wavenumber
        }
        m_bankExag = heightScale;
        m_bankBase = baseTexelM;
        for (int i = 0; i < 12; ++i) m_bankOrg[i] = org12[i];
        m_oneWater = oneWater;
    }
    void SetComposed(int colorCube, int window, int heightCube, int heightWindow,
                     double orgPxX, double orgPxY, double sizePx, int detailWin = -1,
                     double detOrgPxX = 0.0, double detOrgPxY = 0.0) {
        m_colorT = colorCube;
        m_winT = window;
        const bool pages = colorCube >= 0 && window == colorCube;
        m_winFace = pages ? 6u : 0u;
        m_detFace = pages ? 7u : 0u;
        m_hgtWinFace = (heightCube >= 0 && heightWindow == heightCube) ? 6u : 0u;
        m_hgtT = heightCube;
        m_hgtWinT = heightWindow;
        m_detOrg[0] = orgPxX;
        m_detOrg[1] = orgPxY;
        m_detSize = sizePx;
        m_detWinT = detailWin;               // M7f: the z17 detail color window
        m_det17Org[0] = detOrgPxX;
        m_det17Org[1] = detOrgPxY;
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
    int debugLens = 0;              // M7m: --lens (1 worldxz, 2 winuv, 3 mip, 4 ring,
                                    // 7 velgrad -- the derived div/curl bank)
    // M9h: the grad(flow) bank and the grid it lives on, for lens 7.
    void SetVelGradLens(uint32_t srv, float x0, float z0, float sizeX, float sizeZ,
                        uint32_t resMapSrv = 0xFFFFFFFFu, float texelM = 1.0f,
                        float resMapW = 0.0f, float resMapH = 0.0f, uint32_t mipCount = 1) {
        m_lensSrv = srv;
        m_lensGeo[0] = x0;
        m_lensGeo[1] = z0;
        m_lensGeo[2] = (sizeX != 0.0f) ? 1.0f / sizeX : 0.0f;
        m_lensGeo[3] = (sizeZ != 0.0f) ? 1.0f / sizeZ : 0.0f;
        m_lensResMapSrv = resMapSrv;
        m_lensChain[0] = texelM;
        m_lensChain[1] = resMapW;
        m_lensChain[2] = resMapH;
        m_lensChain[3] = static_cast<float>(mipCount);
    }
    // M9j: the REGION page's geography (slice 1 of the same bank). Both pages are read from
    // one array view and composited on coverage -- which is what retires the wind fallback.
    void SetVelGradRegion(double lon0, double lat0, double spanLon, double spanLat) {
        m_lensRegion[0] = static_cast<float>(lon0);
        m_lensRegion[1] = static_cast<float>(lat0);
        m_lensRegion[2] = (spanLon != 0.0) ? static_cast<float>(1.0 / spanLon) : 0.0f;
        m_lensRegion[3] = (spanLat != 0.0) ? static_cast<float>(1.0 / spanLat) : 0.0f;
    }
    float foamOpacity = 0.72f;      // M8: peak foam opacity (data/wave_scene.json)
    float ringBlendTexels = 48.0f;  // M8: bank ring cross-fade width (scene cfg)
    float windGateVal = 1.0f;       // M8: Monahan whitecap gate (per frame, from sea)
    float causticStrength = 0.6f;   // M8: bed dapple strength (scene cfg; 0 = off)
    bool waterOptics = true;        // M9: data-driven K_d + deep colour (scene cfg)
    // M9b: SURFACE DEBUG. Shading alone cannot tell geometry from normals (priors 8), so
    // the raster answers instead. 0 = shipped, 1 = wireframe (every triangle: is the mesh
    // actually moving?), 2 = meshlet tint (one flat colour per amplification record: what
    // drew it, and where the CDLOD rings hand over -- readable at altitudes where
    // every-triangle wireframe collapses into moire). PSO-selected; PsMain untouched.
    int surfaceDebug = 0;
    // M9d: --mesh-stats. The storm wireframe LOOKED like the near field carried coarser
    // triangles than the mid field, which would be the LOD running backwards. Squinting at
    // rasterized lines is exactly the guessing priors 8 warns about, so count the records
    // instead: cell size (arc/32) against distance to the meshlet's own anchor.
    bool meshStats = false;
    float editFloorNavd = 1.8f;     // M8g: edit-land geometry floor, ABSOLUTE NAVD m --
                                    // set from the datum envelope (MLLW + margin) at boot
                                    // so high water drowns the outer jetty (origin planes)
    bool sliceOn = false;           // M7o: the cutaway plane node
    float sliceD = 0.0f;            // plane offset (world z, metres)
    bool albedoLens = false;        // M6j: --albedo, raw composed color -- no lighting, no
                                    // atmosphere, no materials; THE view for texture work
    // GIS survey stencil textures (GisStencil), for the --stencil overlay.
    void SetGisStencil(uint32_t winSrv, uint32_t globSrv, uint32_t editSrv = 0xFFFFFFFFu,
                       const float* editBox = nullptr) {
        m_gisWinSrv = winSrv;
        m_gisGlobSrv = globSrv;
        m_gisEditSrv = editSrv;              // M7f: the ~1 m fine edit mask
        for (int i = 0; i < 4; ++i) m_gisEditBox[i] = editBox ? editBox[i] : 0.0f;
        m_gisEditOn = editSrv != 0xFFFFFFFFu && editBox != nullptr;
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
        // M7: the wave vertex bank (one-water mode): water geometry + params from ONE tiled
        // resource, sampled by ring (camera-anchored mip ladder).
        uint32_t bankU[4];    // disp SRV, param SRV, one-water on, ring texels
        float bankA[4];       // base texel m, mip count, unused, unused
        float bankOrg01[4];   // ring origins (world m): r0.xy, r1.xy
        float bankOrg23[4];
        float bankOrg45[4];
        uint32_t bankU2[4];   // M7a: detail bank SRV, cascade deriv SRVs x3
        float bankB[4];       // cascade patch sizes x3, height exaggeration
        float bankC[4];       // representative wavenumber per cascade
        float bankD[4];       // M8: unit-sea rms envelope per band, w = foam opacity
        float bankE[4];       // M8: ring cross-fade width (texels), rest spare
        // M9 (ALGEBRA "optics"): the water's quality + the sea ice. THREE rows -- keep the
        // count in step with GlobeCb in Globe.hlsl.
        uint32_t optU[4];     // ocean-colour SRV, ice SRV, optics on, spare
        float optA[4];        // ocean grid: lat1, lon1, 1/dlat, 1/dlon
        float optB[4];        // nx, ny, deep-albedo gain g, spare
        float bankFold[4];    // M9c: the FOLD's wavenumber per band (energy-weighted)
        uint32_t lensU[4];    // M9h: grad(flow) bank SRV for --lens velgrad
        float lensA[4];       // bathy grid: org x, org z, 1/sizeX, 1/sizeZ
        float lensB[4];       // M9h: bank texel m, residency-map dims, spare
        float lensR[4];       // M9j: region page lon0, lat0, 1/spanLon, 1/spanLat
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

public:
    // M9h: THE CAYLEY CLOSURE, DRIVING rather than narrating. DeriveDemand answers where a
    // wind-PRODUCT field can be non-zero, from the published grade signatures and no data.
    // It is a CONSERVATIVE bound -- "cannot be non-zero outside here" -- so it is AND-ed with
    // the physics policy rather than replacing it: algebra proves the outside is empty,
    // physics decides which of the inside is worth carrying. Either alone is wrong. The
    // algebra alone cannot know that a 3 m/s breeze is beneath notice; the physics alone has
    // no proof it has not thrown away something a downstream product needs.
    void ApplyWindDemand(Gpu& gpu, const std::vector<uint8_t>& derived, uint32_t dTx,
                         uint32_t dTy);

private:

    std::wstring m_shaderDir;
    const GlobeModel* m_globe = nullptr;
    ID3D12RootSignature* m_rootSig = nullptr;
    Com<ID3D12PipelineState> m_pso, m_skyPso;
    // M6i: m_relief and m_ne retired -- the composed height cube streams what they carried
    // (and returns ~90 MB of committed equirect memory to the pool).
    GpuTexture m_cloudSrc;

    // ---- M9q: THE GLOBAL PLANES, IN THE TREE.
    //
    // gfswave Hs and wind, GFS sea ice, and the packed ocean-colour retrieval were four
    // committed Texture2Ds -- uploaded once, no georeference anything could check, no coverage,
    // no mip chain that understands absence, and no way into the sparse structure. They are
    // global field data, which is precisely what the tree is for.
    //
    // Each is now MemGridLoader -> RasterSource -> DomainCompositor -> a paged GradeBank, with a
    // TEXTURE2D view over slice 0 so Globe.hlsl binds them exactly as before. Same drop-in trick
    // as the bed, and for the same reason: if the picture changes, the bank's CONTENT is the
    // only thing that can have done it.
    struct PlaneBank {
        std::unique_ptr<GradeBank> bank;
        uint32_t srv = UINT32_MAX;
        bool Valid() const { return srv != UINT32_MAX; }
    };
    PlaneBank m_hsB, m_windB, m_oceanB, m_iceB, m_windSrcB;

private:

    // M9w: what the CDLOD walk actually costs, split. The walk is 3.95 ms at helm and the
    // question is whether that is TRAVERSAL (cullable, parallel over six independent face
    // roots) or EMIT (m_res->Want, which funnels every leaf into one shared tracking map).
    // Those two want opposite fixes, so they are counted apart before either is attempted.
public:
    mutable uint64_t walkNodes = 0, walkLeaves = 0, walkWantNs = 0;
    void WalkReset() { walkNodes = walkLeaves = walkWantNs = 0; }
private:

    // Load -> compose -> sparse, and report the worst disagreement with the source array.
    bool BuildPlaneBank(Gpu& gpu, PlaneBank& out, const char* name, const char* structure,
                        const GeoRef& ref, std::vector<MemGridLoader::Plane> planes,
                        DXGI_FORMAT fmt, float nodataFill);

    // M6d: the sparse Mv2 wind bank (div, u, v, curl) -- resident where storms live.
    TileAtlas2D m_windBank;
    std::vector<uint8_t> m_windPhys;   // M9h: the physics verdict, kept for the AND below
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
    // M9ap: pages mode -- window == colorT and these are its slices (6, 7). Otherwise 0.
    uint32_t m_winFace = 0, m_detFace = 0;
    uint32_t m_hgtWinFace = 0;   // M9aq: heightWindow == hgtT -> slice 6
    uint32_t m_gisEditSrv = 0xFFFFFFFFu;
    float m_gisEditBox[4] = {0, 0, 0, 0};
    bool m_gisEditOn = false;
    int m_detWinT = -1;
    double m_det17Org[2] = {0.0, 0.0};
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
    uint32_t m_bankSrv[3] = {0xFFFFFFFFu, 0xFFFFFFFFu, 0xFFFFFFFFu};
    uint32_t m_bankDeriv[3] = {0xFFFFFFFFu, 0xFFFFFFFFu, 0xFFFFFFFFu};
    float m_bankPatch[3] = {756.0f, 186.0f, 47.0f};
    float m_bankK[3] = {0.03f, 0.15f, 1.0f};
    float m_bankRms[3] = {};   // M8: unit-sea rms envelope per band
    float m_bankFold[3] = {0.0209f, 0.2339f, 2.8420f};   // M9c: the fold's wavenumbers
    uint32_t m_lensSrv = 0xFFFFFFFFu;   // M9h: grad(flow) bank
    uint32_t m_lensResMapSrv = 0xFFFFFFFFu;
    float m_lensChain[4] = {1.0f, 0.0f, 0.0f, 0.0f};
    float m_lensRegion[4] = {0.0f, 0.0f, 0.0f, 0.0f};   // region page: lon0, lat0, 1/dLon, 1/dLat
    float m_lensGeo[4] = {0, 0, 0, 0};
    float m_bankExag = 1.15f;
    float m_bankBase = 4.8f;
    float m_bankOrg[12] = {};
    bool m_oneWater = false;

    std::vector<NodeData> m_nodes;
    // M6j: the mesh-shader path.
    static constexpr uint32_t kMaxMeshlets = 65536;
    uint32_t m_meshletDrops = 0;    // M8h: leaves dropped at the record cap this frame
    bool m_dropsReported = false;   // one report per drop episode, not per frame
    bool m_msPath = false;
    uint32_t m_meshStatWalks = 0;   // M9d: --mesh-stats prints on the 8th walk
    Com<ID3D12PipelineState> m_msPso, m_msPsoWire, m_msPsoMeshlet;
    Com<ID3D12PipelineState> m_psoWire, m_psoMeshlet;
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
