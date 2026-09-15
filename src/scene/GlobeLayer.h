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
#include "compose/SurfaceFrame.h"
#include "core/GradeField.h"
#include "core/Common.h"
#include "core/MemGridLoader.h"
#include "core/Space.h"   // M12 step 4d: the level's gauge placement, for the plane transport
#include "hal/Residency.h"
#include "hal/TileAtlas.h"
#include "hal/Views.h"
#include "render/Camera.h"
#include "scene/Layer.h"
#include "sim/GlobeModel.h"

#include <algorithm>
#include <condition_variable>
#include <mutex>
#include <string>
#include <unordered_set>
#include <thread>
#include <vector>

namespace ga {

class GlobeLayer : public Layer {
public:
    static constexpr int kMaxDepth = 6;      // smallest node ~156 km: vertex spacing == texel
    static constexpr double kLodFactor = 3.0;
    // M9bk: the wave-grain rule's screen floor -- a surface triangle never goes below this
    // many pixels. Past it the fold has already shed the band into sigma^2, so more geometry
    // buys variance the shading is carrying anyway.
    //
    // MEASURED: at 3.0, unbounded in range, the rule split all the way to the outer ring at
    // 9.8 km and blew the record budget ("meshlet budget hit: 39 leaves dropped
    // (65520/65536)") -- holes in the surface. The range gate in WalkNode is the real bound;
    // this floor is the safety under it, so a triangle never goes below a few pixels.
    static constexpr double kWavePxFloor = 3.0;
    // How many of the bank's rings the rule refines over. Ring m reaches 256*grain*2^m, so
    // 2 rings is ~614 m at the shipped 1.2 m grain: the helm's neighbourhood, and nothing a
    // globe view can ever be inside.
    static constexpr double kWaveRings = 2.0;
    // M9bk: NYQUIST ON THE WAVE GRAIN. The scene deliberately sets bankTexelM 1.2 to "match
    // the mesh vertex spacing (~1.19 m)" -- and matching a sample grid to the grid it samples
    // is the worst case, not the best: the vertex phase inside a texel drifts at the
    // difference frequency, beating at 1.2*1.19/0.01 ~ 143 m. That beat is the long straight
    // ridges fanning down a storm face, aligned to the MESH and not to the wave (seen only
    // once --wireflat took the water shading off the lines). Two vertices per texel puts the
    // mesh above the field's Nyquist, so it RESOLVES the bank instead of beating with it.
    static constexpr double kWaveOversample = 1.0;   // MEASURED: 2.0 did NOT remove the ridges (they are in the FIELD, not the sampling) and cost the record budget.
    // ... and its depth ceiling. Level 20 is a 9.6 m node = 0.30 m cells, a quarter of the
    // bank's ring-0 texel: enough to carry the cubic's curvature between texels without
    // pretending to data that is not there.
    static constexpr int kWaveMaxDepth = 20;

    void Configure(const std::wstring& shaderDir, const GlobeModel* globe) {
        m_shaderDir = shaderDir;
        m_globe = globe;
    }

    const char* Name() const override { return "globe"; }
    void Init(Gpu& gpu, ShaderCompiler& sc, FieldSet& fields,
              hal::RootSignature rootSig) override;
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
    // M6g: ONE WORLD. The globe renders in the estuary tangent frame; the rows are the
    // planet->tangent rotation (east, up-at-origin, north). SetView receives the FLAT
    // (tangent-frame) camera; the sphere centre sits at flat (0, -R, 0). M12 step 4a: the
    // rows, the radius, the lattices and the tenants are the SurfaceFrame's -- SetSurface.
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
    // M12 step 4a: THE SURFACE, declared once (compose/SurfaceFrame.h). The globe keeps
    // copies of what its walk and its mip-floor wants read (the tenant ids, the page slices,
    // the z14 and z17 origins, the radius: CaptureWalk captures them into every WalkParams)
    // and reads the tangent frame's rows and the fill from the surface itself. Must precede
    // the first SetView, as SetResidency must.
    void SetSurface(const SurfaceFrame* s);
    uint32_t AirTiles() const {
        return (m_windReady ? m_windBank.ResidentCount() : 0) +
               (m_cloudReady ? m_cloud.ResidentCount() : 0);
    }
    uint64_t AirBytes() const {
        return (m_windReady ? m_windBank.ResidentBytes() : 0) +
               (m_cloudReady ? m_cloud.ResidentBytes() : 0);
    }
    // Step 5 (docs/PERF_EXPERIMENT.md): THE PREFETCH WALK LEAVES THE MAIN THREAD.
    //
    // The screw-predicted walk (M6e: the same six-face descent under the pose 24 frames
    // ahead, no frustum cull, Want(predicted) only) was 1.9-2.0 ms of main-thread CPU on
    // every third helm frame -- the measured comb: loop 13.8-14.2 ms on frame%3==1 against
    // 11.9-12.5 on the others. The walk is pure geometry: the eye, the planet frame, the
    // window origins and the tenant ids decide every rect it emits, and none of them changes
    // once the frame's pose is clamped. So StartPredictWalk captures them into a WalkParams
    // and posts it to a worker the moment the pose is final; the worker walks into a vector
    // of rects in DFS order while the main thread runs SetView; ReplayPredictWants, at the
    // point the walk used to run (after SetView, before ProcessQueues advances the stamp
    // frame), waits for the job -- never skips it: a skipped walk would make a rail's want
    // stream depend on the machine -- and issues the same Want(predicted) calls in the same
    // order. The manager sees the identical stream (Residency.h predictedHash: the same
    // FNV-1a on the worker and inline under --predict-inline), so its stamps, queues and
    // sort ties are unchanged. PredictNextPose keeps its call cadence and so its 72-frame
    // effective lookahead (it moves m_prevPose only on the frames it runs); that quirk is
    // its own A/B, not this step's.
    //
    // Everything the node walk reads, captured. The real walk runs over one of these too
    // (SetView fills it, then adds the five planes), so the two walks are ONE geometry.
    struct WalkParams {
        double R = 0.0;
        double frameE[3] = {}, frameU[3] = {}, frameN[3] = {};   // planet -> tangent rows
        double camPos[3] = {};      // the eye in the tangent frame (the predicted one here)
        double camPlanet[3] = {};   // the REAL eye in the planet frame: the horizon test's r
                                    // (PredictWants never moved it; kept so, to the bit)
        int camFace = 0;            // step 24: the eye's own cube face, walked first
        double frustum[6][4] = {};  // camera-relative planes; planeCount 0 = no cull
        int planeCount = 0;
        float reliefExagg = 1.0f;
        int maxDepth = kMaxDepth;
        // M9bk: THE WAVE GRAIN. The wave bank's ring-0 texel (m); 0 = no bank, rule inert.
        // The walk carries the surface at the density the WAVE FIELD is stored at wherever
        // the bank actually reaches -- see WalkNode.
        float waveGrainM = 0.0f;
        int waveMaxDepth = kMaxDepth;
        float pixAng = 1.0e-3f;     // one pixel's angle: the relief-mip selector
        bool wants = false;         // a residency manager and at least one cube tenant
        int surfT = -1, normT = -1, colorT = -1, hgtT = -1, maskT = -1;
        int winT = -1, hgtWinT = -1, detWinT = -1;
        uint32_t winFace = 0, hgtWinFace = 0, detFace = 0;
        double detOrg[2] = {}, detSize = 1.0, det17Org[2] = {};
        bool probeCullFar = false;   // step 23 probe
        // M10: an OCCLUDING SPHERE in this walk's own frame (centre, radius; radius 0 = none).
        // For a level the camera's planet floats in, that planet hides most of it: a node whose
        // bounding sphere lies wholly inside the planet's silhouette cone and beyond its tangent
        // distance cannot be seen, and is not walked.
        double occ[4] = {0.0, 0.0, 0.0, 0.0};
        // M10: a LOCAL relief bound for the culls. The shipped bound pads every node with 9 km
        // of relief (the planet's worst case, x the exaggeration) -- right from orbit, and ruinous
        // for a Droste level whose eye sits 70 m above its sea: nothing within 9 km of that eye
        // can ever be culled, so the whole entrance walks to the wave grain in every direction
        // (MEASURED: 34 k records for the outer level alone, the inner globe then starved to 0).
        // With the planet's own CPU relief (ETOPO) a small node's bound is its local height plus
        // a margin for what a 1.8 km cell can hide. Extra levels only: the camera's own walk keeps
        // its bytes (priors 29 -- a changed want set is a changed picture, through the streamer).
        const GlobeModel* relief = nullptr;
    };
    // One Want() the walk asked for, as it asked (face and mip as the manager takes them).
    struct WantRect {
        int tenant;
        uint32_t face, mip;
        float u0, v0, u1, v1;
    };
    // Post the prefetch walk for this frame: `cam` is the frame's clamped camera (its planet
    // position and pixel angle are the walk's, exactly as SetView's are), `pred` the
    // extrapolated pose the walk runs under. Returns at once; the worker walks while SetView
    // runs. Nothing is posted without a manager and a cube tenant (PredictWants's own gate).
    void StartPredictWalk(const Camera& cam, const Camera& pred, float viewportH);
    // Wait for that walk and issue its rects through Want(predicted = true), in DFS order --
    // where PredictWants used to run. A no-op when nothing was posted.
    void ReplayPredictWants();
    // The worker's figures for the [rail] table: walks replayed, their nodes, leaves and
    // rects, the walk's own time on the worker, the main thread's wait at the join, the
    // replay's time. All frames of the run.
    uint64_t predictWalks = 0, predictNodes = 0, predictLeaves = 0, predictRects = 0;
    uint64_t predictWalkNs = 0, predictWaitNs = 0, predictReplayNs = 0;
    ~GlobeLayer() override;   // joins the worker

    float reliefExagg = 1.0f;       // set per frame by main (altitude-scaled display choice)
    float waterNavd = 0.0f;         // M6j: live water level, for the close-up material model
    bool msSurface = true;          // M6j: request the mesh-shader unified surface
    bool MeshPathActive() const { return m_msPath; }
    bool probeCullFar = false;      // step 23 probe: horizon cull at every altitude (+0.1 rad)
    void DumpMeshlets(const std::wstring& path) const;   // step 23 probe: the records drawn
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
    // ---- M10 THE DROSTE LEVELS (src/core/Droste.h) -----------------------------------------
    // The camera's own level is always walked (slot 0). Each extra level is the ROOT walked
    // again under the eye S^-k(C) -- the same tree, the same addresses, the same wants into the
    // same tenants (the sparse structure serves every level from one resident set) -- and its
    // records are drawn through the gauge: shaded in the level's own frame, rasterized at
    // s^k Q^k of it. A level whose globe is under a pixel emits nothing; that is where the
    // recursion stops, and it is the screen that stops it.
    static constexpr int kMaxLevels = 8;
    struct DrosteLevel {
        int rel = 0;                  // the level, relative to the camera's
        double cam[3] = {0.0, 0.0, 0.0};   // the eye in this level's OWN tangent frame: S^-k(C)
        double sigma = 1.0;           // true size over own size: s^k
        double Q[3][3] = {{1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 1.0}};   // own -> true
        // M12 step 4d: the same map as a Placement -- the linear part of Level(rel), rotor and
        // scale with t = 0 (the eye-to-eye translation the gauge identity cancels) -- which the
        // frustum planes ARE pulled through (PullPlane; step 4d-2: the transport the walk culls
        // by, the hand form from Q and sigma above being the instrument's record).
        Placement gauge;
        float reliefExagg = 1.0f;     // the display exaggeration at this level's own altitude
        float sun[3] = {0.0f, 1.0f, 0.0f};   // the sun in this level's own frame
        int bankSet = -1;             // 0 = the camera's rings, 1 = set B, -1 = none (far)
        float skyUp[3] = {0.0f, 1.0f, 0.0f};   // the zenith of the sky this level SEES, own frame
        float skyDay = -1.0f;         // that sky's daylight; < 0 = the level's own local day
    };
    // Per frame, BEFORE SetView: the scene's sun as the renderer will write it (the level
    // table's slot 0 carries it -- the globe's shading reads the table, not gSunDir).
    void SetSun(const float sun[3]) {
        for (int i = 0; i < 3; ++i) m_camSun[i] = sun[i];
    }
    // M10: the sky the camera's own level SEES (slot 0's row 5): its zenith in this frame and its
    // daylight (< 0 = the level's own local day, the default and every non-Droste run).
    void SetCamSky(const float up[3], float day) {
        for (int i = 0; i < 3; ++i) m_camSkyUp[i] = up[i];
        m_camSkyDay = day;
    }
    // M10: the sun the SPACE backdrop shows (camera frame) -- the sun of the level whose orbit
    // calls for space (Droste.h Grounds, spaceRel). Read only under Droste.
    void SetSpaceSun(const float sun[3]) {
        for (int i = 0; i < 3; ++i) m_spaceSun[i] = sun[i];
    }
    // Per frame, BEFORE SetView, when the link is live. `extra` are the levels other than the
    // camera's; the portal (centre, radius) is the next level down in any level's own frame, for
    // the shadow; camLevelAbs is for the title bar.
    void SetDroste(const DrosteLevel* extra, int n, int lighting, const double portalCentre[3],
                   double portalRadius, int camLevelAbs) {
        m_levels.assign(extra, extra + (std::min)(n, kMaxLevels - 1));
        for (int i = 0; i < 3; ++i) m_portal[i] = portalCentre[i];
        m_portal[3] = portalRadius;
        m_lighting = lighting;
        m_camLevelAbs = camLevelAbs;
        m_drosteOn = true;
    }
    // THE GATE'S WINDOW (scene/Gateway.h), per frame before SetView; null clears it. `level` is
    // the destination drawn as one more level of the walk -- its eye the carried eye, sigma 1, Q
    // the gate motor's rotation back -- and the box is given in the TRUE camera frame: its centre
    // relative to the eye, the rows camera -> box, its half extents. Appended after any Droste
    // levels; the Droste switches (the portal's shadow, the limbs) are not touched.
    void SetGate(const DrosteLevel* level, const float centreRel[3], const float rows[9],
                 const float half[3]) {
        if (!m_drosteOn) m_levels.clear();
        m_gateSlot = -1;
        if (!level || m_levels.size() + 1 >= size_t(kMaxLevels)) return;
        m_levels.push_back(*level);
        m_gateSlot = static_cast<int>(m_levels.size());
        for (int i = 0; i < 3; ++i) {
            m_gateC[i] = centreRel[i];
            m_gateHalf[i] = half[i];
        }
        for (int i = 0; i < 9; ++i) m_gateRows[i] = rows[i];
    }
    // Set B: the rings anchored at the OUTER level's eye (a second WaterBankLayer).
    void SetWaterBankB(uint32_t disp, uint32_t param, uint32_t detail, const float* org12,
                       bool on) {
        m_bankB[0] = disp;
        m_bankB[1] = param;
        m_bankB[2] = detail;
        for (int i = 0; i < 12; ++i) m_bankBOrg[i] = org12 ? org12[i] : 0.0f;
        m_bankBOn = on;
    }
    // THE ADDRESS <-> PLACE maps of the walk's own quadtree (CubeDir's face convention): the
    // unit direction at a leaf's centre, and the leaf at `level` that holds a direction.
    static void LeafDir(int face, int level, uint32_t ix, uint32_t iy, double out[3]);
    static void LeafOf(const double dir[3], int level, int& face, uint32_t& ix, uint32_t& iy);
    uint32_t levelRecords[kMaxLevels] = {};   // records emitted per slot, last frame
    void LogDrosteProbe() const;   // M12 step 4d instrument: the transport comparison's totals

    float foamOpacity = 0.72f;      // M8: peak foam opacity (data/wave_scene.json)
    float ringBlendTexels = 48.0f;  // M8: bank ring cross-fade width (scene cfg)
    float windGateVal = 1.0f;       // M8: Monahan whitecap gate (per frame, from sea)
    float causticStrength = 0.6f;   // M8: bed dapple strength (scene cfg; 0 = off)
    bool waterOptics = true;        // M9: data-driven K_d + deep colour (scene cfg)
    // M9bh --pixel-water: shade the sea per PIXEL (the two rays -- the Cl(3) sky mirror and
    // the refracted bed cast that makes it TRANSLUCENT), instead of the M9bg vertex-shaded
    // default. A look switch, not a quality tier: the geometry is identical either way, and
    // the pixel path carries no foam at all.
    bool pixelWater = false;
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
    bool skyPassEnabled = true;     // M6g: off while SkyLayer owns the low-altitude backdrop
    float skyPassWeight = 1.0f;     // M10: the limb backdrop's blend over the dome (1 = replace)
    // M10: the share of that space backdrop which is the camera level's OWN air (the rest is the
    // space some other level's altitude called for, and holds no air of the camera's). 1 = the
    // shipped backdrop; ignored without Droste.
    float skyOwnAir = 1.0f;
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
        float dPdu[3];
        uint32_t seamX;   // step 23: the seam word across the W (mx 0) / E (mx 3) edge
        float dPdv[3];
        uint32_t seamY;   // step 23: the same across the N (my 0) / S (my 3) edge
        float upT[3];
        uint32_t level;   // M10: the Droste level slot (the record's frame is that level's own)
    };
    static_assert(sizeof(MeshletRec) == 96, "MeshletRec mirrors GlobeMesh.hlsl: 96 B");
    // Step 23: one emitted leaf, keyed for the seam table (SeamTable): face, level and the
    // node's integer grid position, and its first record.
    struct LeafKey {
        uint64_t key;
        uint32_t base;
    };
    void SeamTable();
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
        // (M6i's composed channels + the one-world frame rows: the renderer's one surface
        // buffer, b2, since M12 step 4g -- Common.hlsli's SurfaceCb, not a row of this one.)
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
        uint32_t optU[4];     // ocean-colour SRV, ice SRV, optics on, M9bh pixel-water on
        float optA[4];        // ocean grid: lat1, lon1, 1/dlat, 1/dlon
        float optB[4];        // nx, ny, deep-albedo gain g, spare
        float bankFold[4];    // M9c: the FOLD's wavenumber per band (energy-weighted)
        uint32_t lensU[4];    // M9h: grad(flow) bank SRV for --lens velgrad
        float lensA[4];       // bathy grid: org x, org z, 1/sizeX, 1/sizeZ
        float lensB[4];       // M9h: bank texel m, residency-map dims, spare
        float lensR[4];       // M9j: region page lon0, lat0, 1/spanLon, 1/spanLat
        // M10 THE DROSTE LEVELS -- appended at the END on both sides (priors 22).
        float drosteA[4];     // levels in the table, camera's absolute level, lighting, shadow on
        float drostePortal[4];   // the inner globe in any level's own frame: centre, radius
        uint32_t bankBU[4];   // set B: disp, param, detail SRVs, on
        float bankBOrg01[4];
        float bankBOrg23[4];
        float bankBOrg45[4];
        float droste[192];    // 8 levels x 6 rows (Globe.hlsl LoadLevel)
        // THE GATE'S WINDOW (scene/Gateway.h) -- appended at the END on both sides (priors 22).
        float gateA[4];       // x = the window's slot in the level table (-1 = no window)
        float gateC[4];       // the box's centre relative to the eye, TRUE camera frame (m)
        float gateR0[4];      // rows: TRUE camera frame -> the box's own frame; w = half extent
        float gateR1[4];
        float gateR2[4];
    };
    // Mirrors WindCb in GlobeWind.hlsl.
    struct WindCbData {
        uint32_t nx, ny, listCount, tilesX;
        uint32_t tileW, tileH, pad0, pad1;
        float lat1, dLatDeg, radius, scale;
    };
    // Mirrors GlobeSkyCb (b3 -- root parameter 5 of the shared layout; M12 step 4g moved it
    // off b2, the surface's) in Globe.hlsl.
    struct SkyCbData {
        float fwd[4];         // xyz forward, w = tan(fovY/2)
        float right[4];       // xyz right, w = aspect
        float up[4];
        float lvl[4];         // M10: limb slot, own-air share, Droste sun rule, pixel angle
        float spaceSun[4];    // M10: the space backdrop's sun (GlobeSkyCb gSkySpaceSun)
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

    // Step 5: everything the walk reads, from the members and this camera (planeCount 0).
    WalkParams CaptureWalk(const Camera& cam, float viewportH) const;
    void PredictWalk();   // the prefetch walk, as one pool job
    // camPos is the eye of the level being emitted, in that level's own frame (M10); slot is
    // its row in the level table.
    void EmitMeshlets(int face, double u0, double v0, double size, double arc,
                      float morphStart, float morphEnd, const double camPos[3], uint32_t slot);
    // M10: one walk of the root under a level's eye, emitting records into `slot`.
    void WalkLevel(const WalkParams& wp, uint32_t slot);
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
    hal::RootSignature m_rootSig = nullptr;
    hal::Pso m_pso, m_skyPso;
    hal::Pso m_limbPso;   // M10: PsLimb, dual-source blend, SV_Depth-tested
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
    // M9bk probe: leaves sitting past their own morph band (k == 1: the odd vertices are
    // snapped onto the even ones, so the level renders at HALF the density the walk paid for).
    mutable uint64_t walkMorphFull = 0, walkMorphPart = 0;
    // M13 step 0 (--water-tiles): WHAT WOULD THE WATER COST ON THE PLANET'S OWN LATTICE? The
    // counter answers it before a line of the kernel moves: under the plan's rule each water leaf
    // reads the cube-quadtree tile at its level - 2 (one texel per cell) and the one at level - 3
    // (the morph's target), so the walk records those two addresses per leaf and this reports the
    // distinct set -- the number a sampler's reserve has to hold, and the pool bytes it costs at
    // 128^2 texels x 3 planes x RGBA16F. It counts EVERY leaf: a land test can only take tiles
    // away, and the plan's gate-free test (the min/max height pyramid) does not exist yet.
    // Off by default; a std::set touch per leaf is not free.
    bool waterTileCount = false;
    mutable std::unordered_set<uint64_t> waterTiles;
    mutable uint32_t waterTilesByLevel[32] = {};
    static constexpr uint64_t kWaterTileBytes = 128ull * 128ull * 8ull * 3ull;   // 384 KB a slot
    void WalkReset() {
        walkNodes = walkLeaves = walkWantNs = 0;
        walkMorphFull = walkMorphPart = 0;
        if (waterTileCount) {
            waterTiles.clear();
            for (uint32_t& c : waterTilesByLevel) c = 0;
        }
    }
    // The line the still poses and the Haulover carry are read from.
    std::string WaterTileReport() const {
        char b[512];
        int n = snprintf(b, sizeof(b), "%zu tiles (%.1f MB) from %llu leaves:", waterTiles.size(),
                         double(waterTiles.size() * kWaterTileBytes) / (1024.0 * 1024.0),
                         static_cast<unsigned long long>(walkLeaves));
        for (int L = 0; L < 32; ++L) {
            if (!waterTilesByLevel[L]) continue;
            n += snprintf(b + n, sizeof(b) - size_t(n), " L%d %u;", L, waterTilesByLevel[L]);
        }
        return std::string(b);
    }
    // The meshlet-record memcpy into the frame's upload buffer (Render), last frame, ms: the
    // one CPU cost of the mesh path inside the RENDER bracket. main reads and zeroes it.
    double meshletCopyMs = 0.0;
private:

    // Load -> compose -> sparse, and report the worst disagreement with the source array.
    bool BuildPlaneBank(Gpu& gpu, PlaneBank& out, const char* name, const char* structure,
                        const GeoRef& ref, std::vector<MemGridLoader::Plane> planes,
                        DXGI_FORMAT fmt, float nodataFill);

    // M6d: the sparse Mv2 wind bank (div, u, v, curl) -- resident where storms live.
    TileAtlas2D m_windBank;
    std::vector<uint8_t> m_windPhys;   // M9h: the physics verdict, kept for the AND below
    hal::RootSignatureRef m_windRs;
    hal::Pso m_windBuild;
    hal::Table m_windTable;   // [u1 wind source (a bank, slice 0), u0 bank]
    bool m_windReady = false;

    // M6c: the sky as a sparse VOLUME bank (R16F, 200 m vertical texels; NULL tile = clear
    // air). 64 deep so the 32-deep hardware tiles split the low sky from the high sky --
    // full-height column tiles made residency degenerate (every 320 km column has SOME cloud).
    static constexpr uint32_t kVolNx = 1024, kVolNy = 512, kVolNz = 64;
    static constexpr float kShellTopM = 12800.0f;
    TileAtlas3D m_cloud;
    hal::RootSignatureRef m_cloudRs;
    hal::Pso m_cloudBuild;
    hal::Table m_cloudTable;   // [t1 source SRV, u0 volume UAV]
    bool m_cloudReady = false;

    // M6e/M6i: streaming -- Mars's native pyramids (surf/norm) + the composed channels.
    ResidencyManager* m_res = nullptr;
    int m_surfT = -1, m_normT = -1;
    int m_colorT = -1, m_winT = -1, m_hgtT = -1, m_hgtWinT = -1;
    // M9ap: pages mode -- window == colorT and these are its slices (6, 7). Otherwise 0.
    uint32_t m_winFace = 0, m_detFace = 0;
    uint32_t m_hgtWinFace = 0;   // M9aq: heightWindow == hgtT -> slice 6
    int m_detWinT = -1;
    double m_det17Org[2] = {0.0, 0.0};
    int m_maskT = -1;   // M9ay: the survey mask page tenant
    double m_detOrg[2] = {0, 0};
    double m_detSize = 1;
    bool m_streamMars = false;
    double m_radius = GlobeModel::kR;
    // M12 step 4a: THE SURFACE (SetSurface). The tenant, slice, origin and radius members
    // above are its copies for the walk; the tangent frame's rows (east / up / north) and
    // the fill are read from it.
    const SurfaceFrame* m_surface = nullptr;
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
    // M10: the Droste levels of this frame (slots 1..n; slot 0 is the camera's own).
    std::vector<DrosteLevel> m_levels;
    int m_gateSlot = -1;                       // the window's slot in the level table
    float m_gateC[3] = {0.0f, 0.0f, 0.0f};
    float m_gateRows[9] = {1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f};
    float m_gateHalf[3] = {0.0f, 0.0f, 0.0f};
    float m_camSun[3] = {0.0f, 1.0f, 0.0f};
    float m_camSkyUp[3] = {0.0f, 1.0f, 0.0f};
    float m_camSkyDay = -1.0f;
    float m_spaceSun[3] = {0.0f, 1.0f, 0.0f};
    double m_portal[4] = {0.0, 0.0, 0.0, 0.0};
    int m_lighting = 0;
    int m_camLevelAbs = 0;
    bool m_drosteOn = false;
    // M12 step 4d instrument: the frustum transport compared, per plane per level, hand (Q^T n,
    // d / sigma) against PullPlane through the level's gauge placement (ProbeTransport).
    struct TransportProbeRow {
        uint32_t slot = 0;
        int rel = 0, plane = 0;
        double hand[4] = {}, pulled[4] = {};
    };
    void ProbeTransport(const TransportProbeRow* rows, int n);
    UlpTally m_probeN, m_probeD;
    uint64_t m_probeFp = 0, m_probeWalks = 0, m_probeDumps = 0;
    // M10: the level slots whose limb PsLimb draws this frame, farthest first (each dims what is
    // behind it, so the nearer limb must composite last).
    uint32_t m_limbSlots[kMaxLevels] = {};
    int m_limbCount = 0;
    uint32_t m_bankB[3] = {0xFFFFFFFFu, 0xFFFFFFFFu, 0xFFFFFFFFu};
    float m_bankBOrg[12] = {};
    bool m_bankBOn = false;
    // M6j: the mesh-shader path.
    // M10: 2^17. DispatchMesh caps ONE dimension at 65535 groups, and that was the budget; the
    // Droste levels outgrew it (a turned-around helm at level 2 walked 41 k records of its own
    // beside 24 k for the two worlds outside it, and dropped leaves = holes). The dispatch is now
    // 2-D (GlobeMesh.hlsl folds the group id) and the seam word's record index is 17 bits wide.
    static constexpr uint32_t kMaxMeshlets = 131072;
    uint32_t m_meshletDrops = 0;    // M8h: leaves dropped at the record cap this frame
    std::vector<LeafKey> m_leafKeys;   // step 23: this frame's emitted leaves (SeamTable)
    bool m_dropsReported = false;   // one report per drop episode, not per frame
    bool m_msPath = false;
    uint32_t m_meshStatWalks = 0;   // M9d: --mesh-stats prints on the 8th walk
    hal::Pso m_msPso, m_msPsoWire, m_msPsoMeshlet, m_msPsoWireFlat;
    hal::Pso m_psoWire, m_psoMeshlet;
    std::vector<MeshletRec> m_meshlets;
    GpuBuffer m_recBuf[Gpu::kFrameCount];
    GlobeCbData m_cb{};
    SkyCbData m_skyCb{};
    float m_viewportH = 900.0f;
    double m_camPos[3] = {0, 0, 2.0e7};   // the flat eye: meshlet anchors and gCamAbs
    // Step 5: the real walk's inputs (SetView fills them; the walk reads nothing else) and
    // the prefetch walk's job. The walk is a pool job now (core/ThreadManager.h, Lane::Compute)
    // rather than a dedicated thread parked on the cv: one post, one job. The state handed
    // across under this mutex -- busy (the walk is out; the rects are not back), outstanding
    // (posted and not yet replayed). `posted` and `quit` existed to drive a parked thread and
    // are gone with it.
    WalkParams m_wp{};
    struct PredictJob {
        std::mutex mx;
        std::condition_variable cv;
        bool busy = false;
        bool outstanding = false;
        WalkParams params{};
        std::vector<WantRect> rects;   // the worker's answer; ReplayPredictWants empties it
        uint64_t nodes = 0, leaves = 0, walkNs = 0;
    } m_predict;
};

}  // namespace ga
