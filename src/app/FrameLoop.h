// ================================================================================================
//  FrameLoop - the session and the frame loop as one object: everything main() did after the
//  Assembly, from the mode selection to the explicit shutdown.
//
//  M12 step 1d. The span of main() that followed the Assembly seam -- the session setup (mode,
//  cameras, the one frame, the Droste portal, the rails, the clocks, the boat, the weather
//  manager, the wave field, the pickers), the instrumentation block, the frame loop, and the
//  post-loop tools and shutdown -- moved VERBATIM into Run() = Session(), then Frame() per
//  iteration, then Finish(), by the technique Assembly.h documents: the members are that span's
//  block-level locals, SAME NAMES (m_ prefixed), SAME TYPES, IN MAIN'S DECLARATION ORDER, and
//  each method opens with one alias per member it touches (`auto& cam = m_cam;`,
//  `auto& model = m_A.model;`) so the moved bodies compile unchanged.
//
//  THE LIFETIME LAW, continued. main() declares the FrameLoop AFTER the Assembly, so it
//  destructs FIRST -- the session locals used to die before the assembly locals -- and the
//  members here still destruct in the reverse of main()'s order. Heap-allocated once
//  (std::unique_ptr), never copied, never moved: the wave.prefill worker job captures
//  &wavePending, &wavePrefillDone, &wavePendingTiles and &wavePendingSec by reference (members
//  now), the exposure re-roll closure captures &resMgr and &exposureT (Assembly members), and
//  ga::Threads().Shutdown() at the tail of Finish() joins the pool before either object dies,
//  as it did before.
//
//  WHAT IS NOT THE LOCAL'S EXACT SHAPE, and why. A lambda the loop calls, or that another such
//  lambda captures by reference, is a std::function member with the lambda's own signature,
//  ASSIGNED at the line that used to declare it: applyMode, altOf, poseMotor, motorPose,
//  railPose, diveFrom, diveAt, legU, diveU, keyedPose, gravityUp, drosteRailPose, oceanAt,
//  southAt, westAt, westQAt, sceneToWaveCfg, groundAt, pixelRay, pickGround, pickGlobe,
//  pickAny, stepBoat. A `[&]` inside Session() captures the aliases, which are the members
//  (CWG 2011), and every other block-level local it captures is itself a member, so nothing
//  dangles. Four lambdas only Session() calls stay its locals: planetToFlatPose,
//  flatToPlanetPose, orbPose, orbKey (oceanAtBoston and the wave.field page provider were
//  nested-block locals and stay so). A `const` local whose initialiser was code (lenM,
//  diveFromAbove, startUnix, kWestKm, wT, kPredictEvery) and the two `auto` clocks (last,
//  lastTitle) are plain members assigned at that line; a `const` local with a literal
//  initialiser (kDiveT0, hasSound, kUpriverAreaM2) is a const member with that initialiser;
//  the `constexpr` locals (kProfN, kInPhases..kInN, kSettleQuietFrames, kSettleCapFrames,
//  kSettleExactFrames) and the kProfName table are static constexpr members under their own
//  names; `using Clock` is the class's alias. The three function-local statics -- stepBoat's
//  quantaOwed and telTick, the Droste gauge step's loggedLevel -- are members initialised as
//  the statics were: one per FrameLoop, of which there is one.
//
//  CONTROL FLOW. Session() returns std::optional<int>: its five early exits (the frame-basis
//  fatal, --ocean-probe, --swe-cycle, a --rail with no keys, --export) stay `return <code>;`
//  verbatim and Run() returns that code at the same point, the FrameLoop unwinding in main()
//  as the session locals did. Frame() is the loop body: its two `continue`s are `return true`,
//  its two `break`s `return false`, and the body's own end returns true. The PROF_BEGIN /
//  PROF_END brackets stay macros, defined in FrameLoop.cpp before the methods and undefined
//  after them; they name the members through the aliases.
//
//  THE ONE DELIBERATE CHANGE (an instrument, not a picture): the --settle-exact hold now also
//  waits for the wave prefill to be idle -- Frame(), the `prefillIdle` line, MEASURED
//  2026-09-13 (the exact hold and the prefill raced).
//
//  M12 step 5e: the boat, the portal and the rails LEFT the session for scene nodes. The
//  hull's step state and stepBoat are scene::Entity (one node per `entities` element, stepped
//  by StepEntities from both clock branches); the portal's build and its cycle are
//  scene::Portal (Link() and Cycle() where m_portal and m_drosteLeaf were read); the five rail
//  tables and the six rail lambdas (railPose, diveFrom, diveAt, legU, diveU, keyedPose,
//  gravityUp, drosteRailPose) are scenes/rails/<name>.json read by scene::Rail, At(t) pure.
//  Two of the three function-local statics named above are the entity's now (quantaOwed,
//  telTick); the [droste] probes stay here, reading the node.
// ================================================================================================
#pragma once

#include "app/Assembly.h"
#include "app/FramePipe.h"
#include "app/Options.h"
#include "app/Scene.h"
#include "compose/TileTree.h"
#include "core/Droste.h"   // M10: the globe within the globe, as one Cl(4,1) versor
#include "core/Pga.h"
#include "core/Space.h"   // M12 step 4d: the planet, the tangent frame and the Droste cycle, declared
#include "hal/Residency.h"
#include "hal/Tenant.h"
#include "core/SceneConfig.h"
#include "render/Camera.h"
#include "scene/Route.h"
#include "scene/WaterComponent.h"
#include "scene/SceneReload.h"
#include "sim/Ephemeris.h"   // M13: the one light, kept for every viewpoint
#include "sim/SimClock.h"
#include "scene/Entity.h"   // M12 step 5e: the hull as a node (its step state, its water)
#include "scene/Portal.h"   // M12 step 5e: the Droste link and its cycle as a node
#include "scene/Gateway.h"     // the cuboid gates to other places on the planet
#include "scene/Rail.h"     // M12 step 5e: the rails as data
#include "sim/VesselSpec.h"
#include "sim/WaveField.h"
#include "sim/WaveFieldSource.h"
#include "sim/WeatherManager.h"

#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <utility>
#include <set>
#include <vector>

namespace ga::app {

class FrameLoop {
public:
    // M12 step 5d: the scene comes with them. Every value the session reads that is scene state
    // -- the mode and the start view, the eyes, the clocks, the sun, the sea, the capture, the
    // streaming, the portal and the entity -- is read from `S`; `opt` keeps the pure instruments
    // and the one-shot tools' own arguments.
    FrameLoop(const Options& opt, const Scene& S, Assembly& A);   // the references; nothing runs
    FrameLoop(const FrameLoop&) = delete;
    FrameLoop& operator=(const FrameLoop&) = delete;
    FrameLoop(FrameLoop&&) = delete;
    FrameLoop& operator=(FrameLoop&&) = delete;

    // main()'s remaining span, verbatim: Session(), then Frame() until it says stop, then
    // Finish(). Returns the process exit code.
    int Run();

private:
    // main.cpp ff2f732 lines 242..1518: from `int mode = ...` through stepBoat. An early exit
    // (the five `return`s) comes back as the code; nullopt means the loop runs.
    std::optional<int> Session();
    // The `for (;;)` body, 1521..3160. false = the loop's `break`; true = the next iteration
    // (its `continue`, or the end of the body).
    bool Frame();
    // 3162..3230: the post-loop tools, the reports, the dumps, the explicit shutdown; `return 0`.
    int Finish();
    // M12 step 5e: the entities' step (stepBoat's one path from both clock branches), the
    // profiler slot the hull steps kept, and the chase camera after them.
    void StepEntities(int quanta, float dt);

    // M12 step 5c: the solved wave field re-Configured at the water scene's live window --
    // today's hot-reload lines, moved verbatim. It reads what the SESSION owns (the compositor,
    // the water atlas, the tide and current models, the height channel, the aligned page frame),
    // which is why it is here and not in the component: that is construction, not a property
    // fan-out. The component names the need through this adapter and this does the work.
    void ReconfigureWaveField(const WaterSceneConfig& cfg);
    // M12 step 5f: the `water` section's fan-out at reload -- the sea layer's four numbers, and
    // the wavefield/closures/fleet subtree through 5c's own WaterComponent::Apply.
    void ApplyWater();
    class WaveRebuild final : public scene::WaterComponent::Rebuild {
    public:
        explicit WaveRebuild(FrameLoop* fl) : m_fl(fl) {}
        void ReconfigureWaveField(const WaterSceneConfig& cfg) override {
            m_fl->ReconfigureWaveField(cfg);
        }

    private:
        FrameLoop* m_fl;
    };

    const Options& m_opt;
    const Scene& m_S;
    Assembly& m_A;
    WaveRebuild m_waveRebuild{this};

    // M12 step 5f: THE WHOLE SCENE HOT-RELOADS (scene/SceneReload.h). `m_live` is the session's
    // own copy of the document's sections -- the LIVE state a reload writes, where `m_S` is the
    // record the run was BUILT from and stays const -- and each target's fanOut carries the
    // written fields to the objects that are not that struct (the camera's optics, the sea
    // layer's four water numbers, the clock, the predict cadence, and 5c's water Apply).
    scene::SceneDocument m_live;
    scene::SceneReload m_reload;
    // THE FRAME A PLACEMENT RESOLVES IN (scene/Props.h PoseFrame): the surface's own tangent
    // rows and the planet's radius. A view's set carries `at`, and a placement without a frame
    // refuses rather than guessing a planet -- which would take the whole view set down with it.
    scene::PoseFrame m_poseFrame;

    // M12 step 5d: the scene's LIST elements this session reads, looked up ONCE in Session() and
    // held as values -- a named lookup is a string compare, and the frame reads some of these
    // (the portal's lighting, the chase camera's three numbers) every frame. Default-constructed
    // they ARE the declared defaults, which are the flags' defaults, so a scene that declares
    // neither reads exactly what the flag-shaped code read.
    ScenePortal m_portalDecl;   // `portals[droste]`
    bool m_portalOn = false;    // ...and whether it is enabled (--droste)
    // (M12 step 5e: the `entities` elements are the Entity nodes below, one per element.)
    SceneView m_startView;      // the view `scene.view` names (its optics and its chase camera)
    // The interests the view named that were logged once (held, or naming nothing).
    std::set<std::string> m_interestsLogged, m_interestsMissing;

    // ---- main()'s block-level locals over the span, in main()'s order (ff2f732 lines
    // 246..1518), then the three function-local statics. Each is reached in the methods
    // through an alias under its old name (`auto& cam = m_cam;`); the static constexpr
    // members keep their own names and need none.
    int m_mode = 0;
    std::function<void(int)> m_applyMode;
    double m_lenM = 0.0;
    Camera m_cam;
    Camera m_camSea;
    Camera m_camGlobe;
    Camera m_camChart;
    // (M12 step 4a: the tangent frame's rows -- m_oDir, m_east0, m_north0 -- are the
    // Assembly's SurfaceFrame's; Session() writes them there through the same aliases.)
    // M12 step 5e: THE PORTAL NODE (scene/Portal.h): the link BuildPortal resolves and the
    // cycle it declares, read through Link() and Cycle() where m_portal and m_drosteLeaf were.
    scene::Portal m_portalNode;
    int m_camLevel = 0;   // the camera's ABSOLUTE level: 0 = the root, 1 = inside the first link
    // M12 step 4d: THE FRAME CALCULUS' DECLARATIONS (core/Space.h). The planet at unit length
    // R; the tangent frame under it, linked from the surface's rows; and the Droste tower as a
    // CYCLE -- the root's tangent frame hung under a leaf of itself by the portal's similarity,
    // Level(k) = S^k. The portal stays the builder (BuildPortal resolves the address); the Space
    // is the declaration the scene will carry. Space keeps parent POINTERS: they point at these
    // members, and FrameLoop is neither copied nor moved (above), so they never dangle.
    Space m_planetSpace;
    Space m_tangentSpace;
    // (the Droste cycle, Space::Cycle over m_tangentSpace, is the portal node's: Cycle())
    // M12 step 4d instrument: the [droste] closed-form comparison. Every Droste read of the
    // portal (the level table's cam / sigma / Q / sun / sky zenith, the camera level's sun,
    // the dive rail's S^f(helm)) is evaluated from Level(k) beside it and compared bit for bit
    // (core/Common.h UlpTally); the table's dump prints when its geometry changes, Finish()
    // prints the totals. Step 4d-2: the renderer's reads ARE the cycle's -- LevelApply /
    // LevelApplyDir (the power about its fixed point) and PullPlane for the planes -- and the
    // instrument records the residual against the portal's closed forms, computed beside them.
    struct DrosteProbeRow {
        int rel = 0;
        bool hasSky = false;
        double cam[3] = {}, sigma = 1.0, Q[3][3] = {}, sun[3] = {}, skyUp[3] = {};        // the portal's
        double cam2[3] = {}, sigma2 = 1.0, Q2[3][3] = {}, sun2[3] = {}, skyUp2[3] = {};   // Level(rel)'s
    };
    void ProbeDrosteTable(const DrosteProbeRow* rows, int n, const double sun[3],
                          const double sun2[3], uint32_t frame);
    void ProbeDive(double f, const double c0[3], const double f0[3], const double up0[3],
                   double c[3], double fw[3], double up[3], bool live, double u);
    UlpTally m_probeCam, m_probeSigma, m_probeQ, m_probeLevelSun, m_probeSkyUp, m_probeSun;
    UlpTally m_probeDiveC, m_probeDiveFw, m_probeDiveUp;
    UlpTally m_probeSweepC, m_probeSweepFw, m_probeSweepUp;
    uint64_t m_probeTableFp = 0, m_probeTableBuilds = 0, m_probeTableDumps = 0;
    uint64_t m_probeDiveCalls = 0;
    bool m_probeSweepShown = false;
    std::function<double(const Camera&)> m_altOf;
    std::function<Motor(const Camera&)> m_poseMotor;
    std::function<void(const Motor&, Camera&)> m_motorPose;
    // M12 step 5e: THE RAIL AS DATA (scene/Rail.h): the active rail, read from
    // scenes/rails/<active>.json and resolved in this session's frame; the dive rail beside
    // it when another rail (or none) is flown, because the [droste] probe's sweep is the
    // dive rail's own schedule -- its helm pose and its law are that file's.
    scene::RailFrame m_railFrame;
    scene::Rail m_rail;
    scene::Rail m_diveProbeRail;
    const scene::Rail* m_diveRail = nullptr;
    double m_simUnix = 0.0;
    SimClock m_simClock;
    VesselRegistry m_vesselReg;
    // M12 step 5e: THE HULLS ARE ENTITY NODES (scene/Entity.h), one per `entities` element,
    // each with its own step state and its own TreeWater; m_followed is the one the start
    // view's chase camera follows. Heap-held: an Entity is never moved once wired.
    std::vector<std::unique_ptr<scene::Entity>> m_entities;
    std::vector<std::unique_ptr<scene::Gateway>> m_gates;
    // THE PLACES the gates stand at and lead to, other than the root's own frame: one space per
    // place (scene/Gateway.h Place), so a body carried to a place stands in the very frame a gate
    // there was built in. Heap-held: the gates and the carried hulls point into them.
    std::vector<std::unique_ptr<scene::Place>> m_places;
    // WHAT THE VIEW SEES THROUGH THE GATES this frame (scene/Gateway.h WindowChain): link 0 is the
    // first window the view enters, link k the window its view meets k windows in. Rebuilt every
    // frame from whatever gates exist, so a gate that moves, grows or appears is simply seen.
    std::vector<scene::WindowLink> m_windows;
    static constexpr double kWindowReachM = 2.0e4;   // a window further than this is not looked for
    int m_windowDepthLogged = -1;   // the corridor's depth as last logged (on a change only)
    bool m_windowRecordsDue = false;   // the depth changed: log the next walk's records per world
    scene::ViewCone ViewConeOf(const Camera& cam, float aspect, float viewH) const;
    std::vector<const scene::Gateway*> GateList() const;
    // Every hull, once per world the view reaches: the eye's own, and each place the chain of
    // windows shows -- pulled back through the windows, so a boat is seen wherever a window shows
    // its place, its own reflection down the corridor included.
    void PublishHulls();
    scene::Entity* m_followed = nullptr;
    // ---- THE EYE'S OWN CROSSING (M13). A chase eye does not teleport with its subject. When the
    // hull goes through a window, the eye keeps standing on this side and chases the hull's
    // APPARENT pose -- pulled back through that window's motor, which is the map the window's
    // geometry is already drawn by -- so the boat is watched THROUGH the portal, growing smaller
    // in it, until the eye itself reaches the box. Then the eye crosses by the hull's own rule
    // (its centre inside the box), and at that instant every ray already starts inside the
    // window, so nothing on screen moves: it stops marching because it is in.
    // These are the gates the subject has crossed and the eye has not, oldest first.
    static constexpr size_t kMaxEyeOwes = 4;
    std::vector<const scene::Gateway*> m_eyeOwes;
    uint32_t m_followCarries = 0;   // the subject's carry count as of the last frame
    // M9br: THE WAVE PREFILL, OFF THE FRAME THREAD. When a tide or current bucket rolls,
    // the solve was already backgrounded but the PREFILL was not -- and writing 7359 tiles
    // across 33 planes and every mip takes 11-19 s, on the frame thread, which is the
    // once-a-minute freeze. Nothing reads the new tree until the atomic_store below, so it
    // can be built and filled on a worker and swapped in when it is whole.
    std::shared_ptr<TileTree> m_wavePending;
    std::atomic<bool> m_wavePrefillDone{false};
    std::atomic<bool> m_wavePrefillBusy{false};
    uint64_t m_wavePendingKey = 0;
    uint32_t m_wavePendingTiles = 0, m_wavePendingPlanes = 0;
    double m_wavePendingSec = 0.0;
    bool m_skyProbed = false;   // --sky-probe reads the tables once
    // The solar system this frame was lit by -- the one light, so every viewpoint (a gate's
    // window included) asks it for its own direction instead of borrowing the camera's.
    sun::SolarSystem m_solar;
    bool m_solarValid = false;
    bool m_sunLogged = false;   // M9bi: log the placed sun once, with its numbers
    bool m_winSkyLogged = false;   // M13: the gate window's sky, once -- its up and its sun
    double m_startUnix = 0.0;
    int m_entSta = 0, m_westA = 0, m_westB = 0;
    double m_kWestKm = 0.0;
    double m_wT = 0.0;
    std::function<double(double)> m_oceanAt;
    const bool m_hasSound = false;   // retired with the solver's south strip (see SweSolver)
    std::function<double(double)> m_southAt;
    std::function<double(double)> m_westAt;
    const double m_kUpriverAreaM2 = 3.9e6;
    std::function<double(double)> m_westQAt;
    WeatherManager m_weather;
    std::function<WaveFieldConfig(const WaterSceneConfig&)> m_sceneToWaveCfg;
    std::unique_ptr<WaveField> m_waveField;
    int m_wfCtSta = -1;
    std::shared_ptr<WaveFieldSource> m_waveSrc;
    std::shared_ptr<std::shared_ptr<TileTree>> m_waveTree;
    WaveFieldSource::Frame m_waveFrame;
    int m_waveT = -1;
    hal::Tenant m_waveTenant;   // M12 step 3e: the wave planes' declaration (hal/Tenant.h)
    Route m_route;
    double m_timeScale = 1.0;
    double m_windowSec = 0.0;
    bool m_paused = false;
    int m_dragMode = 0;          // 0 none, 1 pan-grab, 2 tilt (SHIFT), 3 rotate (ALT)
    bool m_lmbWas = false;
    double m_dragPivot[3] = {};
    double m_lastWaterNavd = 0;  // last frame's level; the pivot ray tests against it
    std::function<double(double, double)> m_groundAt;
    std::function<void(float, float, double[3])> m_pixelRay;
    std::function<bool(float, float, double[3])> m_pickGround;
    std::function<bool(float, float, double[3])> m_pickGlobe;
    std::function<bool(float, float, double[3])> m_pickAny;
    using Clock = std::chrono::steady_clock;
    Clock::time_point m_last;
    Clock::time_point m_lastTitle;
    uint32_t m_frame = 0;
    // M9b: a programmatic .wpix is serialized on a PIX background thread AFTER the
    // capture frames present. The process used to exit ~4 frames later and the file
    // landed as a 1 KB stub -- armed, never written. Remember that we armed, and hold
    // the process open at the end until the file stops growing.
    bool m_pixArmed = false;
    bool m_dumpedSolid = false;   // --dump-both: the solid image is already on disk
    double m_frameMsSum = 0.0;
    uint32_t m_frameMsN = 0;
    // M9o: a recording carries its OWN cost. A rail is the only run long enough and varied
    // enough -- globe to helm, every residency regime in one take -- for frame time to mean
    // something, and it is exactly the run nobody measures because they are watching the
    // pictures. Per-frame, not averaged: a mean hides the stall that a viewer actually sees.
    std::vector<float> m_railMs, m_railLoopMs;
    // M9u: WHERE THE OTHER HALF OF THE FRAME GOES. --bench showed render is only 7.9 of a
    // 17.2 ms helm frame; this splits the remaining 9.4 ms by section. Accumulated in two
    // buckets -- the whole rail, and the HELM leg alone -- because the cost is altitude
    // dependent and a single total would average the expensive view away, which is the
    // mistake the phase table already caught once.
    // M9v: prefetch cadence, --predict-every. 1 = every frame (the shipped behaviour).
    uint32_t m_kPredictEvery = 1;
    // Step 1 (perf plan): two brackets that were unnamed -- the 17 wave-plane Wants after
    // waveField.Update (the 0.60 ms helm residual between "sum of the ten" and the
    // pre-RenderFrame total in out/instr_bench.log) and the exposure roll + Want that
    // sea.SetTime's bracket used to swallow.
    static constexpr int kProfN = 12;
    double m_profMs[kProfN] = {}, m_profHelmMs[kProfN] = {};
    std::vector<float> m_railPreMs;   // M9u: the whole pre-RenderFrame half
    uint64_t m_walkNodesAcc = 0, m_walkLeavesAcc = 0, m_walkWantNsAcc = 0, m_walkFrames = 0;
    uint64_t m_morphFullAcc = 0, m_morphPartAcc = 0;
    uint64_t m_wantTouchAcc = 0, m_wantHitAcc = 0;
    static constexpr const char* kProfName[kProfN] = {
        "weather.Update", "scene hot-reload stat", "waveField.Update",
        "waterBank.SetFrame", "tide.SetTime", "sea.SetTime",
        "groundAt (cam clamp)", "globe.SetView", "globe.PredictWants", "swe (solver step)",
        "wave-plane Wants (17)", "exposure roll + Want"};
    std::chrono::steady_clock::time_point m_profT0;
    bool m_profHelm = false;
    // The section brackets accumulate only over the frames the [rail] series keeps (a rail's
    // 150 settle frames are excluded, as the walk stats already were); the divisor is the
    // series length, so the two now agree. profOn is set at the top of every iteration.
    bool m_profOn = true;
    // THE CPU INSIDE RenderFrame, named. RENDER = record + fence (+ serialized GPU under
    // --bench); the record half held three untimed CPU costs: the residency turn
    // (ProcessQueues, by phase -- probe P5 of the plan), the bank's tile-list build (P6) and
    // the meshlet memcpy. Each layer times itself; they are read and zeroed here so a frame
    // that skipped a layer contributes nothing. Accumulated like the section table, and
    // written per frame into metrics.csv.
    static constexpr int kInPhases = ResidencyManager::kPhases;
    static constexpr int kInResTurn = kInPhases, kInBankList = kInPhases + 1,
                         kInMeshCopy = kInPhases + 2, kInN = kInPhases + 3;
    double m_profInMs[kInN] = {}, m_profInHelmMs[kInN] = {};
    std::vector<std::array<float, kInN>> m_railInMs;
    // metrics.csv pairs loop_ms with its own frame: the interval measured at the top of an
    // iteration is the PREVIOUS frame's, so it closes the row that frame opened.
    bool m_loopRowOpen = false;
    // --settle-sync state: the instant is held at the dump frame while the residency
    // manager drains; the still is then a function of pose and instant alone.
    bool m_settling = false;
    uint32_t m_settleFrames = 0, m_settleQuiet = 0, m_settlePending0 = 0, m_settleReads0 = 0;
    static constexpr uint32_t kSettleQuietFrames = ResidencyManager::kEvictAgeFrames;
    static constexpr uint32_t kSettleCapFrames = 3000;   // then dump anyway, and say so
    // Step 25: --settle-exact's own count -- consecutive turns the manager reported the
    // resident set equal to the want set. kEvictAgeFrames past the last drop its NULL map
    // has landed; the +4 is one more overlap window with nothing moving.
    uint32_t m_settleExactQuiet = 0;
    static constexpr uint32_t kSettleExactFrames = ResidencyManager::kEvictAgeFrames + 4u;
    FramePipe m_recPipe;
    std::vector<uint8_t> m_recPixels;
    std::vector<uint64_t> m_railPool;
    int m_loggedLevel = 0;   // Frame(): the Droste level last logged (the gauge step)
};

}  // namespace ga::app
