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
#include "scene/WindowBox.h"
#include "sim/GlobeModel.h"
#include "sim/WaveChart.h"

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
        // M13: this view is a sampler of the one earth cache. Its walk, its prefetch and its
        // mip floors are all one reader; the extra levels it walks name their own.
        m_sampler = rm ? rm->Sampler("view") : 0;
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
    // M7: per-frame wave-bank binding (SRVs + ring origins).
    void SetWaterBank(uint32_t dispSrv, uint32_t paramSrv, uint32_t detailSrv,
                      const uint32_t derivSrv[3], const float patchL[3],
                      const float bandK[3], const float bandRms[3], const float bandFold[3],
                      float heightScale,
                      float baseTexelM, const float* org12) {
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
    }
    // M12 step 4a: THE SURFACE, declared once (compose/SurfaceFrame.h). The globe keeps
    // copies of what its walk reads (the tenant ids and the radius: CaptureWalk captures them
    // into every WalkParams)
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
        int surfT = -1, normT = -1, colorT = -1, hgtT = -1, maskT = -1, bldT = -1;
        int bldFloorRungs = 0;   // the field is wanted at a window mip m >= rung - this (BuildingField)
        bool hgtWindows = false;   // PHASE B2: the height tenant reads the eye's windows too
        // PHASE A2: every slot's windows (SurfaceFrame::bound[s]), rank i + 1 at slice
        // SurfaceFrame::WindowSlice(s, i + 1) of the colour and the mask: its face, its rung and
        // its box's origin in texels of the rung; wnK[s] is the slot's K. walkSlot is the slot
        // this walk draws for when it walks no shared worlds.
        // PHASE A3: the windows are a SET's (a place's: SurfaceFrame::kWindowSets of them), and each
        // slot of the level table reads the set it claimed (wnSetOf; ~0 = none) -- as many slots as
        // the table has.
        static constexpr uint32_t kBlocks = 5, kSets = 8;
        uint32_t walkSlot = 0;
        std::vector<uint32_t> wnSetOf;
        uint32_t wnK[kSets] = {}, wnFace[kSets][kBlocks] = {};
        int wnRung[kSets][kBlocks] = {};
        long long wnAx[kSets][kBlocks] = {}, wnAy[kSets][kBlocks] = {};
        uint32_t wnSlice[kSets][kBlocks] = {};   // F9: the slice each rank reads (shared or its own)
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
        // M13: THE WORLDS THIS WALK DRAWS -- a place's tiles, walked once. Every world that shows
        // the same place (the eye's own, and each depth of a corridor of windows that comes back
        // to it) is drawn from ONE traversal: a node is walked while any of them sees it and
        // still needs it finer, and each world takes as its leaves the nodes its own eye stops
        // at -- its own cut, the records its own walk would have made -- with the tiles asked of
        // the residency once. worlds[0] is the walk's own (its eye is camPos); each is its level
        // slot, its eye from camPos, and its cull -- its own horizon, planes it must be inside and
        // a hole (the next window's cone, which belongs to a deeper world) it must not be wholly
        // inside, the planes eye-relative in this frame. Worlds share a walk only where their own
        // walks would split and bound a node alike (the wave grain, the relief exaggeration).
        // worldCount 0 is the single walk, frustum and all.
        struct World {
            uint32_t slot = 0;
            double off[3] = {0.0, 0.0, 0.0};   // this world's eye minus camPos
            double planes[6][4] = {};
            int planeCount = 0;
            double hole[5][4] = {};
            int holeCount = 0;
            // Its horizon (GlobeLayer.cpp HorizonOf): the eye in the planet frame and its radius,
            // the horizon's angle from there, the margin past it (< 0: no horizon test), and the
            // cosine of the two together (a node's centre inside it is never past, whatever its
            // size).
            double planet[3] = {0.0, 0.0, 0.0};
            double planetR = 0.0;
            double horizon = 0.0;
            double margin = -1.0;
            double cosLimit = -1.0;
        };
        // A world a bit of the walk's masks (live, seen): a place's worlds past these walk again.
        static constexpr int kMaxWorlds = 32;
        World worlds[kMaxWorlds];
        int worldCount = 0;
        // The local relief bound for the WORLDS' tests when the walk's own bound is the planet's
        // worst case (the camera's walk keeps that bound for its own frustum -- priors 29 -- but
        // a window's cone padded by 9 km of relief culls nothing near the eye).
        const GlobeModel* worldRelief = nullptr;
    };
    // One Want() the walk asked for, as it asked (face and mip as the manager takes them).
    struct WantRect {
        int tenant;
        uint32_t face, mip;
        float u0, v0, u1, v1;
        float nearM = 0.0f;   // step 5: the leaf's distance, the want's weight
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
    // Phase A0: --ground-probe, G's planet direction (GlobeLayer::EyeInstruments).
    bool groundProbeOn = false;
    double groundProbeDir[3] = {0.0, 1.0, 0.0};
    // Phase B0: the probe's height and exposure lanes (ResidencyLens.hlsl ProbeRankTexel): the
    // exposure tenant's views, and each tenant's window floor (~0 = no windows), set by the frame
    // loop.
    uint32_t probeX[4] = {0xFFFFFFFFu, 0xFFFFFFFFu, 0xFFFFFFFFu, 0xFFFFFFFFu};
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
    // M13: worlds whose eyes stand this near each other are one place, and share one walk.
    static constexpr double kShareReachM = 2.0e4;
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
        // M13: WHICH SAMPLER this level's wants are charged to (ResidencyManager::Sampler).
        // A Droste level and a gate's window read the same cache the camera does, and each
        // answers for what it asked for; -1 means the view's own.
        int sampler = -1;
        // M13: THE LEVEL'S OWN CULL. A world seen through windows is seen only along the rays that
        // pass them, so it is walked only there: planes in the TRUE camera frame, eye-relative, in
        // the frame's axes (a x + b y + c z + d >= 0 inside), pulled through the gauge exactly as
        // the camera's are. A cull and nothing else -- what the level shows is decided per pixel.
        // 0 = the camera's own frustum (every Droste level).
        double planes[6][4] = {};
        int planeCount = 0;
        // ...and what it need not walk: the cone of the next window it shows (a deeper world's),
        // same frame and convention, inside = hidden (scene/Gateway.h LinkHole).
        double hole[5][4] = {};
        int holeCount = 0;
        // A gate's world: drawn in the camera's own frame at sigma 1, so it may share the walk of
        // any world whose eye stands near its own -- the same place, its tiles chosen once.
        bool share = false;
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
    // ANOTHER EYE: the minimap, a split screen's second player, a remote player's view. It walks
    // the same planet from its own camera, after the first eye's SetView, with no Droste levels
    // or gate worlds of its own, under its own exaggeration, zenith and residency sampler
    // ("eye<view>"), and keeps what it walked for Render at ctx.viewIndex == view. The layer's
    // working set stays the first eye's, so nothing the first eye reads afterwards has moved.
    // --gate-overdraw (an instrument): on the frame it is raised, the first eye's surface is drawn one
    // world per dispatch, each in a pipeline-statistics query (primitives into the rasterizer, pixel
    // shader invocations) and an occlusion query (samples written: past the window test and depth).
    // ReadOverdraw, after the frame, waits for the GPU and returns a row per world.
    bool overdrawProbe = false;
    struct OverdrawRow {
        uint32_t level = 0, records = 0, culled = 0;
        uint64_t primitives = 0, clipped = 0, fragments = 0, samples = 0;
    };
    std::vector<OverdrawRow> ReadOverdraw(Gpu& gpu);
    static constexpr uint32_t kMaxOtherEyes = 3;
    // `gates`/`boxes`/`n`/`hole` are that eye's own windows (SetGates' arguments, from its own cone);
    // `rings`, when given, is the ring set its own world reads as its set A (its own bank, or the
    // first eye's where it stands at the first eye's place), with the cascade's plane at the eye.
    struct EyeRings {
        uint32_t disp = 0, param = 0, detail = 0;
        float org[12] = {};
        WaveChart::Frame chart{};
        bool chartOn = false;
    };
    void SetOtherView(Gpu& gpu, uint32_t view, const Camera& cam, float aspect, float viewportH,
                      double simTime, float exagg, const float skyUp[3], bool skyPass,
                      const DrosteLevel* gates = nullptr,
                      int n = 0, const double (*hole)[4] = nullptr, int holeN = 0,
                      const EyeRings* rings = nullptr);
    // The eye stops drawing (its walk is not refreshed this frame).
    // THE LEVEL TABLE an eye drew this frame: 6 float4 a level (Globe.hlsl LoadLevel), for the
    // view's world table (scene/WorldTable.h). View 0 is the first eye.
    const std::vector<float>& LevelRows(uint32_t view) const {
        return (view == 0 || view > kMaxOtherEyes) ? m_levelRows : m_other[view - 1].levelRows;
    }
    void DropOtherView(uint32_t view) {
        if (view > 0 && view <= kMaxOtherEyes) m_other[view - 1].valid = false;
    }

    // Per frame, BEFORE SetView, when the link is live. `extra` are the levels other than the
    // camera's; the portal (centre, radius) is the next level down in any level's own frame, for
    // the shadow; camLevelAbs is for the title bar.
    void SetDroste(const DrosteLevel* extra, int n, int lighting, const double portalCentre[3],
                   double portalRadius, int camLevelAbs) {
        m_levels.assign(extra, extra + (n > 0 ? n : 0));
        for (int i = 0; i < 3; ++i) m_portal[i] = portalCentre[i];
        m_portal[3] = portalRadius;
        m_lighting = lighting;
        m_camLevelAbs = camLevelAbs;
        m_drosteOn = true;
    }
    // THE VIEW'S WINDOWS (scene/Gateway.h WindowChain), per frame before SetView; n = 0 clears
    // them. levels[k] is the world seen through k + 1 windows, drawn as one more level of the walk --
    // its eye the eye carried that far, sigma 1, Q the chain's rotation back, its cull the rays
    // through the windows. A pixel of level k + 1 is kept where its ray passes exactly k + 1
    // windows of the view's chain (the boxes are the view's world table's), in order; every other
    // level keeps the pixels that pass none. Appended after any Droste levels; the Droste switches
    // are not touched. `viewHole` is what the eye's own world need not walk: the first window's
    // cone (LinkHole).
    void SetGates(const DrosteLevel* levels, int n, const double viewHole[5][4], int viewHoleCount) {
        if (!m_drosteOn) m_levels.clear();
        m_gateFirst = -1;
        m_gateCount = 0;
        m_viewHoleCount = (viewHole && viewHoleCount > 0) ? (std::min)(viewHoleCount, 5) : 0;
        for (int p = 0; p < m_viewHoleCount; ++p) {
            for (int j = 0; j < 4; ++j) m_viewHole[p][j] = viewHole[p][j];
        }
        if (!levels || n <= 0) {
            m_viewHoleCount = 0;
            return;
        }
        const int take = n;
        m_gateFirst = static_cast<int>(m_levels.size()) + 1;
        for (int k = 0; k < take; ++k) m_levels.push_back(levels[k]);
        m_gateCount = take;
    }
    // Set B: the rings anchored at the OUTER level's eye (a second WaterBankLayer).
    // M13 step 2: the cascade sea's plane at the eye (sim/WaveChart.h), for the pixel stage's
    // sub-ring bands -- the same plane the bank filled its texels from. Off until a frame hands
    // one over, in which case the pixels read the root's tangent plane, as they always did.
    void SetWaveChartFrame(const WaveChart::Frame& f, bool on) {
        m_chartFrame = f;
        m_chartOn = on;
    }
    // Whether any world of this frame's level table reads ring set `set` (DrosteLevel::bankSet):
    // the bank is stateless, so it is filled for its readers and for nobody else (FrameLoop).
    // Final once SetDroste and SetGates have run.
    bool ReadsBankSet(int set) const {
        for (const DrosteLevel& L : m_levels) {
            if (L.bankSet == set) return true;
        }
        return false;
    }
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
    std::vector<uint32_t> levelRecords;   // records emitted per slot, last frame
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
    struct LeafKey {   // a leaf's world slot, then its place on the cube (face, level, ix, iy)
        uint32_t slot;
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
        // THE VIEW'S WINDOWS (scene/Gateway.h) -- both sides changed together (priors 22). The
        // level rows and the boxes are the view's world table's (scene/WorldTable.h).
        float gateA[4];       // x = the first window level's slot (-1 = none), y = how many
        // M13 step 2: THE CASCADE SEA'S PLANE AT THE EYE (sim/WaveChart.h) -- appended at the END
        // on both sides (priors 22). The pixel stage adds the bands a ring texel cannot carry by
        // reading the cascade DERIVATIVE textures directly, and those reads have to happen in the
        // same plane the bank filled its texels from, or the fine ripples are a second sea laid
        // over the first. One plane serves every pixel: a chart cell is hundreds of km across and
        // a frame's pixels sit inside one. The rows say that plane IN THE TANGENT FRAME, about the
        // tangent point (Globe.hlsl ChartUOf; packed in GlobeLayer.cpp, REVIEW finding 7).
        float chartOrg[4];    // the constant along east, wrapped to cascade 0 / 1 / 2; w = 1 when
                              // the rows are live
        float chartE[4];      // the cell's east in the tangent frame, w = the constant along
                              // north, wrapped to cascade 0
        float chartN[4];      // its north in the tangent frame, w = the same, cascade 1
        float chartCn[4];     // the constant along north, wrapped to cascade 2; yzw spare --
                              // appended at the END on both sides (priors 22)
        // Phase A0: THE TWO-WORLDS PROBE (--ground-probe, read by --lens blocks) -- appended at the
        // END on both sides (priors 22). G's planet direction, w = the slots filled; per slot, G
        // less that slot's eye in the tangent axes (doubles, cast), w = 1 where filled.
        float probeG[4];
        float probeP[32];
        // Phase B0: the probe's height and exposure lanes -- appended at the END on both sides
        // (priors 22): the exposure's array SRV and residency SRV, the height windows' floor mip,
        // the exposure windows' floor mip (~0 = that tenant has no windows).
        uint32_t probeX[4];
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
    void WalkLevel(const WalkParams& wp, uint32_t slot, int sampler);
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
    // The meshlet's own cull (GlobeLayer.cpp): its records marked, after the walk emits a leaf.
    // The same box test on the LEAF whole, before its records are made: true when none of it can
    // be seen by world m (-1: the walk's own frustum) from `eye`.
    bool LeafHidden(const WalkParams& wp, int m, int face, double u0, double v0, double size,
                    double arc, const double eye[3]) const;
    double LeafHeadroom(const WalkParams& wp, int face, double u0, double v0, double size,
                        double arc) const;
    // Returns how many of the 16 it marked; all 16 and the leaf is not kept at all.
    uint32_t CullMeshlets(const WalkParams& wp, int m, size_t base, int face, double u0, double v0,
                      double size, double arc);
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
    // F19 (the CPU walk's instrument): what a leaf's emit is made of -- the cube wants, the window
    // wants asked and the ones a world found asked, behind the face, outside the box or past the
    // floor; the corner projections; and the meshlets' time, beside the emit's.
    struct LeafStats {
        uint64_t cube = 0, win = 0, winAsked = 0, winBehind = 0, winOut = 0, winFloor = 0, corners = 0;
        uint64_t worlds = 0;   // (world, slice) pairs considered
    };
    mutable LeafStats leafStats;
    mutable uint64_t walkMeshNs = 0;
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
        leafStats = LeafStats{};
        walkMeshNs = 0;
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
    int m_sampler = 0;   // M13: this view's id on the shared cache
    int m_surfT = -1, m_normT = -1;
    int m_colorT = -1, m_hgtT = -1;
    int m_maskT = -1;   // M9ay: the survey mask page tenant
    int m_bldT = -1;    // the buildings under a pixel (compose/BuildingField.h)
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

    std::vector<NodeData> m_nodes;
    // M10: the Droste levels of this frame (slots 1..n; slot 0 is the camera's own).
    std::vector<DrosteLevel> m_levels;
    // Phase A0: the levels' blocks log (changed frames only) and the probe's rows.
    void EyeInstruments();
public:
    // PHASE A2: the frame's level table as SetDroste / SetGates left it, for the windows' step:
    // how many slots (the camera's and the extra levels), and an extra slot's eye in its own frame.
    size_t LevelSlots() const { return 1u + m_levels.size(); }
    // The first gate world's slot in the table and how many there are (-1, 0: none), as SetGates left it.
    int GateFirst() const { return m_gateFirst; }
    int GateCount() const { return m_gateCount; }
    const double* LevelCam(size_t slot) const { return slot ? m_levels[slot - 1].cam : nullptr; }
private:
    uint64_t m_eyeFrame = 0;
    std::string m_eyeLast;
    int m_gateFirst = -1;                      // the first window level's slot in the table
    int m_gateCount = 0;                       // how many windows deep the view's chain goes
    double m_viewHole[5][4] = {};              // the first window's cone: the eye's world skips it
    int m_viewHoleCount = 0;

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
    std::vector<uint32_t> m_limbSlots;
    int m_limbCount = 0;
    WaveChart::Frame m_chartFrame;   // M13 step 2: the plane at the eye
    bool m_chartOn = false;
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
    // The residency lens (shaders/ResidencyLens.hlsl): built only when debugLens asks for it.
    // (14 was the address lens, --lens addr: deleted with the Mercator pages, B3.)
    hal::Pso m_msPsoLens, m_psoLens;
    bool ResidencyLensOn() const { return (debugLens >= 9 && debugLens <= 11) || debugLens >= 14; }
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
    // ANOTHER EYE's walk, kept for its own Render (SetOtherView), and the flag that keeps the
    // first eye's instruments and title out of it.
    struct OtherEye {
        GlobeCbData cb{};
        std::vector<float> levelRows;   // its level table (WorldTable::levels)
        SkyCbData skyCb{};
        std::vector<MeshletRec> meshlets;
        std::vector<NodeData> nodes;
        std::vector<uint32_t> limbSlots;
        int limbCount = 0;
        bool skyPass = true;
        float skyWeight = 1.0f;
        int sampler = -1;
        bool valid = false;
        GpuBuffer recBuf[Gpu::kFrameCount];
    };
    OtherEye m_other[kMaxOtherEyes];
    std::vector<float> m_levelRows;   // the first eye's level table (LevelRows)
    // the probe's queries (made at its first use) and the worlds it measured
    Com<ID3D12QueryHeap> m_odStats, m_odOccl;
    Com<ID3D12Resource> m_odReadback;
    uint32_t m_odLevels = 0;
    uint32_t m_odCap = 0;   // the query heaps' length
    std::vector<uint32_t> m_odRecords, m_odCulled;
    bool m_borrowed = false;
    void RenderEye(const FrameContext& ctx, const GlobeCbData& cbData, const SkyCbData& skyCbData,
                   const std::vector<MeshletRec>& meshlets, const std::vector<NodeData>& nodes,
                   const uint32_t* limbSlots, int limbCount, GpuBuffer& rec, bool skyPass,
                   float skyWeight);
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
