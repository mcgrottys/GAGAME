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
// ================================================================================================
#pragma once

#include "app/Assembly.h"
#include "app/FramePipe.h"
#include "app/Options.h"
#include "compose/TileTree.h"
#include "core/Droste.h"   // M10: the globe within the globe, as one Cl(4,1) versor
#include "core/Pga.h"
#include "core/Residency.h"
#include "core/SceneConfig.h"
#include "render/Camera.h"
#include "scene/Route.h"
#include "sim/SimClock.h"
#include "sim/Vessel.h"
#include "sim/VesselSpec.h"
#include "sim/WaterSurfaceTree.h"
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
#include <vector>

namespace ga::app {

class FrameLoop {
public:
    FrameLoop(const Options& opt, Assembly& A);   // stores the two references; nothing else runs
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

    const Options& m_opt;
    Assembly& m_A;

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
    double m_oDir[3], m_east0[3], m_north0[3];
    droste::Portal m_portal;
    int m_camLevel = 0;   // the camera's ABSOLUTE level: 0 = the root, 1 = inside the first link
    std::function<double(const Camera&)> m_altOf;
    std::function<Motor(const Camera&)> m_poseMotor;
    std::function<void(const Motor&, Camera&)> m_motorPose;
    std::vector<std::pair<double, Motor>> m_railKeys;
    std::function<void(double, Camera&)> m_railPose;
    Camera m_drosteHelm;
    double m_drosteHelmUp[3] = {0.0, 1.0, 0.0};
    bool m_diveFromAbove = false;
    const double m_kDiveT0 = 40.0 + 2.0;   // the storm rail, then two seconds at the helm
    std::function<void(const Camera&, double, Camera&, int&, double[3])> m_diveFrom;
    std::function<void(double, Camera&, int&, double[3])> m_diveAt;
    Camera m_drosteHelmBack;
    std::function<double(double, double, double)> m_legU;
    std::function<double(double)> m_diveU;
    std::vector<std::pair<double, Motor>> m_climbKeys;
    std::function<void(const std::vector<std::pair<double, Motor>>&, double, Camera&)> m_keyedPose;
    std::function<void(const Camera&, double[3])> m_gravityUp;
    std::function<void(double, Camera&, int&, double[3])> m_drosteRailPose;
    double m_simUnix = 0.0;
    SimClock m_simClock;
    VesselRegistry m_vesselReg;
    std::unique_ptr<Vessel> m_boat;
    TreeWater m_boatSea;
    VesselControls m_boatCtl;
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
    bool m_boatPlaced = false;      // set down on the surface at the first step, see stepBoat
    bool m_helming = false;         // T detaches the camera; the physics never stops
    double m_helmYawRef = 0.0;      // the look-steer's heading reference -- see the note in
                                    // stepBoat for why it is NOT read off the live camera
    bool m_sunLogged = false;   // M9bi: log the placed sun once, with its numbers
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
    std::function<void(int)> m_stepBoat;
    int m_quantaOwed = 0;    // stepBoat: the 240 Hz quanta owed to the 60 Hz hull step
    int m_telTick = 0;       // stepBoat: one telemetry line a second
    int m_loggedLevel = 0;   // Frame(): the Droste level last logged (the gauge step)
};

}  // namespace ga::app
