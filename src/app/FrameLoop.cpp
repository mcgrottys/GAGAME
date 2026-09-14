// ================================================================================================
//  FrameLoop - Run(): main()'s session, frame loop and shutdown as one object (M12 step 1d).
//
//  The bodies are main.cpp's span after the Assembly seam at ff2f732, VERBATIM, under the
//  aliases FrameLoop.h describes: Session() is lines 242..1518 (the mode selection through
//  stepBoat), Frame() the `for (;;)` body, Finish() the post-loop tools and the explicit
//  shutdown. The only lines that changed shape are the ones the header lists -- the block-level
//  declarations (members now), the lambdas that became std::function assignments, the loop's
//  `break`/`continue` (return false/true) -- and the one instrument change at `prefillIdle`.
//  FormatTitle and the two water-mip constants moved with the loop because only it calls them.
//  Comments stayed with the code they describe; the ones that described the instrumentation
//  members went to the header with them.
// ================================================================================================
#include "app/FrameLoop.h"

#include <sys/stat.h>

#include "compose/ColorStackSource.h"
#include "compose/ExposureSource.h"
#include "compose/GisMask.h"
#include "compose/HeightStackSource.h"
#include "compose/TileTree.h"
#include "compose/TileArchive.h"
#include "compose/TileIndex.h"
#include "hal/TileStream.h"
#include "compose/ComposeTree.h"
#include "compose/DomainSource.h"
#include "core/CurrentFieldLoader.h"
#include "core/GeoGridLoader.h"
#include "hal/Gpu.h"
#include "core/Image.h"
#include "hal/PixEvents.h"
#include "hal/TileAtlas.h"
#include "core/Window.h"
#include "render/Renderer.h"
#include "scene/FieldSet.h"
#include "scene/GisLayer.h"
#include "scene/GlobeLayer.h"
#include "scene/MarkerLayer.h"
#include "scene/GulfLayer.h"
#include "scene/Pose.h"   // M12 step 5a: the session's pose maps as pure functions
#include "scene/Route.h"
#include "scene/SeaLayer.h"
#include "scene/SkyLayer.h"
#include "scene/TerrainLayer.h"
#include "scene/WaterBankLayer.h"
#include "scene/TideLayer.h"
#include "compose/Compositor.h"
#include "compose/Exchange.h"
#include "compose/GisStencil.h"
#include "compose/Projections.h"
#include "compose/Sources.h"
#include "compose/WaterAtlas.h"
#include "core/Json.h"
#include "core/GaAst.h"
#include "core/BuildInfo.h"
#include "core/CrashTrace.h"
#include "core/ThreadAudit.h"
#include "core/ThreadManager.h"
#include "sim/RigidBody.h"
#include "sim/Vessel.h"
#include "sim/WaterSurfaceTree.h"
#include "sim/VesselSpec.h"
#include "scene/VesselLayer.h"
#include "sim/SimClock.h"
#include "hal/DxTest.h"
#include "core/Droste.h"   // M10: the globe within the globe, as one Cl(4,1) versor
#include "core/Pga.h"
#include "core/TileProviders.h"
#include "core/SceneConfig.h"
#include "sim/BathyModel.h"
#include "sim/Ephemeris.h"   // M9bi: the sun as a place, in Cl(4,1)
#include "sim/GlobeModel.h"
#include "sim/WaveField.h"
#include "sim/WaveFieldSource.h"
#include "sim/CurrentModel.h"
#include "sim/SeaState.h"
#include "sim/SweSolver.h"
#include "sim/TideModel.h"
#include "sim/WeatherManager.h"
#include "app/Assembly.h"
#include "app/FramePipe.h"
#include "app/Options.h"
#include "app/Tools.h"

#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <ctime>
#include <fstream>
#include <filesystem>
#include <string>

using namespace ga;
using namespace ga::app;

namespace {

// M9bl: the wave tenant's coarsest mip (7 levels, like the other page tenants). The window's
// want rides the mip its on-screen size justifies, so the field arrives as a gradient.
constexpr double kWaveMaxMip = 6.0;
// M9bn: how many levels FINER than the screen rule the water asks for. Water is this
// engine's subject; a coarse mip on it reads as structure (the page grid), not softness.
constexpr double kWaterMipBias = 2.0;

void FormatTitle(wchar_t* buf, size_t n, double simUnix, double timeScale, bool paused,
                 const TideModel& model, const TideLayer& tide, double windowDays,
                 const SeaLayer* sea, const SeaState& seaState, const char* gulfInfo,
                 const char* globeInfo, int mode) {
    const time_t tt = static_cast<time_t>(llround(simUnix));
    tm g{};
    gmtime_s(&g, &tt);
    const TideStation& focus = model.S(model.Focus());
    if (mode == 3) {
        swprintf(buf, n,
                 L"GAGAME GLOBE  |  %04d-%02d-%02d %02d:%02d:%02d UTC  |  %s%.0fx  |  "
                 L"ETOPO 2022 + live gfswave  |  %hs  |  drag = grab the planet, SHIFT tilt, "
                 L"ALT rotate, wheel zoom",
                 g.tm_year + 1900, g.tm_mon + 1, g.tm_mday, g.tm_hour, g.tm_min, g.tm_sec,
                 paused ? L"PAUSED " : L"", timeScale, globeInfo ? globeInfo : "");
        return;
    }
    if (mode == 2) {
        swprintf(buf, n,
                 L"GAGAME GULF  |  %04d-%02d-%02d %02d:%02d:%02d UTC  |  %s%.0fx  |  GoMOFS "
                 L"surface currents  |  %hs  |  violet = cyclonic, amber = anticyclonic (OW < 0)",
                 g.tm_year + 1900, g.tm_mon + 1, g.tm_mday, g.tm_hour, g.tm_min, g.tm_sec,
                 paused ? L"PAUSED " : L"", timeScale, gulfInfo ? gulfInfo : "");
        return;
    }
    if (mode == 1 && sea) {
        swprintf(buf, n,
                 L"GAGAME SEA  |  %04d-%02d-%02d %02d:%02d:%02d UTC  |  %s%.0fx  |  tide %.2f m  "
                 L"|  %hs  |  Hs model %.2f m (44013 obs %.2f m)  |  %hs %hs  |  %hs",
                 g.tm_year + 1900, g.tm_mon + 1, g.tm_mday, g.tm_hour, g.tm_min, g.tm_sec,
                 paused ? L"PAUSED " : L"", timeScale, tide.focusHeight,
                 sea->currentStatus.empty() ? "no current data" : sea->currentStatus.c_str(),
                 sea->hsModel, sea->hsBuoy, seaState.CycleLabel().c_str(),
                 sea->statusNote.c_str(), sea->atlasStats.c_str());
        return;
    }
    const double trend = model.Height(model.Focus(), simUnix + 600.0) - tide.focusHeight;
    swprintf(buf, n,
             L"GAGAME  |  %04d-%02d-%02d %02d:%02d:%02d UTC  |  %s%.0fx  |  %hs %.2f m MLLW %lc  "
             L"|  window %.1f d  |  fit rms %.1f mm",
             g.tm_year + 1900, g.tm_mon + 1, g.tm_mday, g.tm_hour, g.tm_min, g.tm_sec,
             paused ? L"PAUSED " : L"", timeScale, focus.name.c_str(), tide.focusHeight,
             trend >= 0 ? L'\x2191' : L'\x2193', windowDays, focus.fitRmsM * 1000.0);
}

}  // namespace

namespace ga::app {

// The two brackets stepBoat (Session) and Frame() time themselves with. They reach the members
// through the aliases (profT0, profOn, profMs, profHelm, profHelmMs) and the class's Clock, so
// they are defined here, before the methods, and undefined after them.
#define PROF_BEGIN() profT0 = Clock::now()
#define PROF_END(slot)                                                                         do {                                                                                           const double _e =                                                                              std::chrono::duration<double>(Clock::now() - profT0).count() * 1000.0;                  if (profOn) {                                                                                  profMs[slot] += _e;                                                                        if (profHelm) profHelmMs[slot] += _e;                                                  }                                                                                          } while (0)

FrameLoop::FrameLoop(const Options& opt, Assembly& A) : m_opt(opt), m_A(A) {}

int FrameLoop::Run() {
    if (const std::optional<int> rc = Session()) return *rc;
    for (;;) {
        if (!Frame()) break;
    }
    return Finish();
}

// M12 step 4d instrument: THE LEVEL TABLE FROM THE CYCLE, beside the portal's closed forms.
// Per level of a build: cam, the eye in the level's own frame -- portal.Apply(-rel, C) against
// LevelApply(-rel, C) (step 4d-2: the power about its fixed point, THE READ; 4d's
// Level(rel).Inverse().Apply(C) was the same map about the origin, 5546 ulps and 1.42e6 m off
// at rel +3); sigma -- portal.Scale(rel) against Level(rel).s; Q -- portal.Rot(rel) against the
// rotor's columns (Rows(): GlobeLayer's own -> true layout, Q[i][j] = axis_j[i]); the level's
// sun and sky zenith -- portal.ApplyDir(-(camLevel + rel)) against LevelApplyDir, the read; and
// the camera level's sun. The cam, sun and sky-zenith reads are the cycle's; sigma and Q stay
// the portal's (the constant buffer's rows). Every build is
// compared bit for bit (core/Common.h UlpTally; the totals print in Finish()); the dump of both
// paths' doubles prints when the table's GEOMETRY changes (its FNV-1a over cam, sigma, Q and
// the level set -- the [kernel] rule: a held still builds one table 450 times, and the sun walks
// with sim time under it), with that build's sun words beside it.
void FrameLoop::ProbeDrosteTable(const DrosteProbeRow* rows, int n, const double sun[3],
                                 const double sun2[3], uint32_t frame) {
    ++m_probeTableBuilds;
    uint64_t fp = Fnv1aBytes(&n, sizeof n);
    for (int i = 0; i < n; ++i) {
        const DrosteProbeRow& r = rows[i];
        fp = Fnv1aBytes(&r.rel, sizeof r.rel, fp);
        fp = Fnv1aBytes(r.cam, sizeof r.cam, fp);
        fp = Fnv1aBytes(&r.sigma, sizeof r.sigma, fp);
        fp = Fnv1aBytes(r.Q, sizeof r.Q, fp);
        fp = Fnv1aBytes(r.cam2, sizeof r.cam2, fp);
        fp = Fnv1aBytes(&r.sigma2, sizeof r.sigma2, fp);
        fp = Fnv1aBytes(r.Q2, sizeof r.Q2, fp);
    }
    const bool dump = fp != m_probeTableFp;
    m_probeTableFp = fp;
    const std::string wSun = UlpWord(sun, sun2, 3, m_probeSun);
    if (dump) {
        ++m_probeTableDumps;
        Log("[droste] table build %llu (frame %u, camera level %d): %d levels -- the portal's "
            "closed forms | Level(rel) from the cycle; camera sun: portal %.17g %.17g %.17g | "
            "Level %.17g %.17g %.17g | %s",
            static_cast<unsigned long long>(m_probeTableBuilds), frame, m_camLevel, n, sun[0],
            sun[1], sun[2], sun2[0], sun2[1], sun2[2], wSun.c_str());
    }
    for (int i = 0; i < n; ++i) {
        const DrosteProbeRow& r = rows[i];
        const std::string wCam = UlpWord(r.cam, r.cam2, 3, m_probeCam);
        const std::string wSig = UlpWord(&r.sigma, &r.sigma2, 1, m_probeSigma);
        const std::string wQ = UlpWord(&r.Q[0][0], &r.Q2[0][0], 9, m_probeQ);
        const std::string wLs = UlpWord(r.sun, r.sun2, 3, m_probeLevelSun);
        const std::string wUp =
            r.hasSky ? UlpWord(r.skyUp, r.skyUp2, 3, m_probeSkyUp) : std::string("(none)");
        if (!dump) continue;
        Log("[droste] level %+d portal: cam %.17g %.17g %.17g sigma %.17g Q %.17g %.17g %.17g "
            "%.17g %.17g %.17g %.17g %.17g %.17g sun %.17g %.17g %.17g skyUp %.17g %.17g %.17g",
            r.rel, r.cam[0], r.cam[1], r.cam[2], r.sigma, r.Q[0][0], r.Q[0][1], r.Q[0][2],
            r.Q[1][0], r.Q[1][1], r.Q[1][2], r.Q[2][0], r.Q[2][1], r.Q[2][2], r.sun[0], r.sun[1],
            r.sun[2], r.skyUp[0], r.skyUp[1], r.skyUp[2]);
        Log("[droste] level %+d Level : cam %.17g %.17g %.17g sigma %.17g Q %.17g %.17g %.17g "
            "%.17g %.17g %.17g %.17g %.17g %.17g sun %.17g %.17g %.17g skyUp %.17g %.17g %.17g",
            r.rel, r.cam2[0], r.cam2[1], r.cam2[2], r.sigma2, r.Q2[0][0], r.Q2[0][1], r.Q2[0][2],
            r.Q2[1][0], r.Q2[1][1], r.Q2[1][2], r.Q2[2][0], r.Q2[2][1], r.Q2[2][2], r.sun2[0],
            r.sun2[1], r.sun2[2], r.skyUp2[0], r.skyUp2[1], r.skyUp2[2]);
        Log("[droste] level %+d compare: cam %s sigma %s Q %s sun %s skyUp %s", r.rel,
            wCam.c_str(), wSig.c_str(), wQ.c_str(), wLs.c_str(), wUp.c_str());
    }
}

// M12 step 4d instrument: THE DIVE THROUGH THE CYCLE. The rail's pose is S^f of the helm, f =
// u - floor(u) in [0, 1): LevelApply(f) / LevelApplyDir(f) -- step 4d-2, THE READ: the power
// about its fixed point (Space.h PowApply: p re-solved by Cramer, the rotor's principal power,
// the point through Motor::QRotate) -- beside portal.Apply(f) and ApplyDir(f), Rodrigues by
// twist f about the stored fixed point, computed for the residual. The live rail (every dive
// frame, one line each) and the Session sweep of the rail's own u schedule tally separately.
void FrameLoop::ProbeDive(double f, const double c0[3], const double f0[3], double c[3],
                          double fw[3], double up[3], bool live, double u) {
    m_drosteLeaf.LevelApply(f, c0, c);
    m_drosteLeaf.LevelApplyDir(f, f0, fw);
    m_drosteLeaf.LevelApplyDir(f, m_drosteHelmUp, up);
    double c2[3], fw2[3], up2[3];   // the portal's closed forms, for the record
    m_portal.Apply(f, c0, c2);
    m_portal.ApplyDir(f, f0, fw2);
    m_portal.ApplyDir(f, m_drosteHelmUp, up2);
    const std::string wc = UlpWord(c2, c, 3, live ? m_probeDiveC : m_probeSweepC);
    const std::string wf = UlpWord(fw2, fw, 3, live ? m_probeDiveFw : m_probeSweepFw);
    const std::string wu = UlpWord(up2, up, 3, live ? m_probeDiveUp : m_probeSweepUp);
    const bool allEq = wc == "EQUAL" && wf == "EQUAL" && wu == "EQUAL";
    if (live) ++m_probeDiveCalls;
    if (live || (!allEq && !m_probeSweepShown)) {
        if (!live) m_probeSweepShown = true;
        Log("[droste] dive %s u %.6f (f %.17g): c %s | fw %s | up %s || portal c %.17g %.17g "
            "%.17g fw %.17g %.17g %.17g up %.17g %.17g %.17g | Level c %.17g %.17g %.17g fw %.17g "
            "%.17g %.17g up %.17g %.17g %.17g",
            live ? "frame" : "sweep (the first not EQUAL)", u, f, wc.c_str(), wf.c_str(),
            wu.c_str(), c2[0], c2[1], c2[2], fw2[0], fw2[1], fw2[2], up2[0], up2[1], up2[2], c[0],
            c[1], c[2], fw[0], fw[1], fw[2], up[0], up[1], up[2]);
    }
}

std::optional<int> FrameLoop::Session() {
    // ---- The aliases: one reference per member this body touches, under main()'s names, so
    // what follows is main()'s code unchanged. A closure that captures one of these by
    // reference captures the member it is bound to (CWG 2011) and outlives this call.
    const Options& opt = m_opt;
    auto& model = m_A.model;
    auto& window = m_A.window;
    auto& gpu = m_A.gpu;
    auto& renderer = m_A.renderer;
    auto& fields = m_A.fields;
    auto& sky = m_A.sky;
    auto& tide = m_A.tide;
    auto& seaState = m_A.seaState;
    auto& sea = m_A.sea;
    auto& currents = m_A.currents;
    auto& haveCurrents = m_A.haveCurrents;
    auto& datumOff = m_A.datumOff;
    auto& marsMode = m_A.marsMode;
    auto& globeModel = m_A.globeModel;
    GlobeModel& activeGlobe = *m_A.activeGlobe;
    auto& bathyBoston = m_A.bathyBoston;
    auto& compositor = m_A.compositor;
    auto& hgtCh = m_A.hgtCh;
    auto& waterAtlas = m_A.waterAtlas;
    auto& bathy = m_A.bathy;
    auto& terrain = m_A.terrain;
    auto& swe = m_A.swe;
    auto& riverQ = m_A.riverQ;
    auto& gulf = m_A.gulf;
    auto& waterScene = m_A.waterScene;
    auto& waterBank = m_A.waterBank;
    auto& waterBankB = m_A.waterBankB;
    auto& globe = m_A.globe;
    auto& vesselLayer = m_A.vesselLayer;
    auto& planetR = m_A.planetR;
    auto& resMgr = m_A.resMgr;
    auto& gisLayer = m_A.gisLayer;
    auto& exchange = m_A.exchange;
    auto& surface = m_A.surface;   // M12 step 4a: the shipped surface, declared once
    auto& winTenant = m_A.winTenant;
    auto& hgtTenant = m_A.hgtTenant;
    auto& hgtWinTenant = m_A.hgtWinTenant;
    auto& colCh = m_A.colCh;
    auto& mode = m_mode;
    auto& applyMode = m_applyMode;
    auto& lenM = m_lenM;
    auto& cam = m_cam;
    auto& camSea = m_camSea;
    auto& camGlobe = m_camGlobe;
    auto& camChart = m_camChart;
    // M12 step 4a: the tangent frame's rows are the surface's (compose/SurfaceFrame.h).
    auto& oDir = m_A.surface.up;
    auto& east0 = m_A.surface.east;
    auto& north0 = m_A.surface.north;
    auto& portal = m_portal;
    auto& drosteLeaf = m_drosteLeaf;   // M12 step 4d: the Droste tower as a Space::Cycle
    auto& altOf = m_altOf;
    auto& poseMotor = m_poseMotor;
    auto& motorPose = m_motorPose;
    auto& railKeys = m_railKeys;
    auto& railPose = m_railPose;
    auto& drosteHelm = m_drosteHelm;
    auto& drosteHelmUp = m_drosteHelmUp;
    auto& diveFromAbove = m_diveFromAbove;
    auto& kDiveT0 = m_kDiveT0;
    auto& diveFrom = m_diveFrom;
    auto& diveAt = m_diveAt;
    auto& drosteHelmBack = m_drosteHelmBack;
    auto& legU = m_legU;
    auto& diveU = m_diveU;
    auto& climbKeys = m_climbKeys;
    auto& keyedPose = m_keyedPose;
    auto& gravityUp = m_gravityUp;
    auto& drosteRailPose = m_drosteRailPose;
    auto& simUnix = m_simUnix;
    auto& simClock = m_simClock;
    auto& vesselReg = m_vesselReg;
    auto& boat = m_boat;
    auto& boatSea = m_boatSea;
    auto& boatCtl = m_boatCtl;
    auto& boatPlaced = m_boatPlaced;
    auto& helming = m_helming;
    auto& helmYawRef = m_helmYawRef;
    auto& startUnix = m_startUnix;
    auto& entSta = m_entSta;
    auto& westA = m_westA;
    auto& westB = m_westB;
    auto& kWestKm = m_kWestKm;
    auto& wT = m_wT;
    auto& oceanAt = m_oceanAt;
    auto& hasSound = m_hasSound;
    auto& southAt = m_southAt;
    auto& westAt = m_westAt;
    auto& kUpriverAreaM2 = m_kUpriverAreaM2;
    auto& westQAt = m_westQAt;
    auto& weather = m_weather;
    auto& sceneToWaveCfg = m_sceneToWaveCfg;
    auto& waveField = m_waveField;
    auto& wfCtSta = m_wfCtSta;
    auto& waveSrc = m_waveSrc;
    auto& waveTree = m_waveTree;
    auto& waveFrame = m_waveFrame;
    auto& waveT = m_waveT;
    auto& waveTenant = m_waveTenant;
    auto& route = m_route;
    auto& timeScale = m_timeScale;
    auto& windowSec = m_windowSec;
    auto& lastWaterNavd = m_lastWaterNavd;
    auto& groundAt = m_groundAt;
    auto& pixelRay = m_pixelRay;
    auto& pickGround = m_pickGround;
    auto& pickGlobe = m_pickGlobe;
    auto& pickAny = m_pickAny;
    auto& last = m_last;
    auto& lastTitle = m_lastTitle;
    auto& railMs = m_railMs;
    auto& railLoopMs = m_railLoopMs;
    auto& kPredictEvery = m_kPredictEvery;
    auto& profMs = m_profMs;
    auto& profHelmMs = m_profHelmMs;
    auto& profT0 = m_profT0;
    auto& profHelm = m_profHelm;
    auto& profOn = m_profOn;
    auto& recPipe = m_recPipe;
    auto& railPool = m_railPool;
    auto& stepBoat = m_stepBoat;
    auto& quantaOwed = m_quantaOwed;
    auto& telTick = m_telTick;

    // M6g: THREE views, not four -- 0 chart, 1 THE WORLD (estuary and planet, one
    // continuous scene), 2 gulf map. --sea and --globe both open the world; they differ
    // only in the starting camera. Per-frame altitude gates refine world-mode layer
    // enables continuously (sky hands to the limb shell, the FFT sea sheds at height).
    mode = ((opt.globeStart || opt.seaStart) && (sea || globe)) ? 1
             : (opt.gulfStart && gulf)                              ? 2 : 0;
    applyMode = [&](int m) {
        sky->enabled = (m != 1);   // world mode gates the sky per frame by altitude
        tide->enabled = (m == 0);
        if (sea) sea->enabled = (m == 1) && !marsMode && !opt.albedo;
        if (terrain) terrain->enabled = (m == 1) && !marsMode;
        if (gulf) gulf->enabled = (m == 2);
        if (globe) globe->enabled = (m == 1);
    };
    applyMode(mode);

    fields.Finalize();
    renderer.SetFieldTable(fields.TableGpuVa());

    // Camera: an oblique of the river profile from the south, low enough that the ribbon
    // fills the band between the sky and the plot.
    lenM = model.TotalRiverKm() * TideLayer::kKmToSceneM;
    cam.px = lenM * 0.08;
    cam.py = 240.0;
    cam.pz = -660.0;
    cam.LookAt(lenM * 0.46, 35.0, 0.0);
    cam.fovY = opt.fovDeg * 3.14159265f / 180.0f;
    cam.speed = 150.0f;

    // Chart and sea each keep their own camera; TAB cycles views and each resumes where you
    // left it. With bathymetry loaded, the sea default is the LITERAL vqview signature shot:
    // standing on the north jetty tip (world 522, 72 -- located in the CUDEM data), az 246
    // back into the inlet.
    if (bathy.Ready()) camSea.SetFromCompass(522.0, 7.0, 72.0, 246.0f, -4.0f);
    else camSea.SetFromCompass(0.0, 12.0, 0.0, 246.0f, -5.0f);
    camSea.fovY = cam.fovY;
    camSea.speed = 30.0f;
    // Globe mode: the planet frame (centre at the origin). Start over the North Atlantic
    // with home in view.
    {
        // M12 step 5a: the four lines that placed the eye are scene::GlobeCamera (the
        // scene's {lat, lon, alt} spelling resolves through the same function); gR is
        // planetR + gAlt inside it, the same sum.
        const double gLat = (opt.gcamLat < 1e8f) ? opt.gcamLat : 34.0;
        const double gLon = (opt.gcamLat < 1e8f) ? opt.gcamLon : -52.0;
        const double gAlt = (opt.gcamLat < 1e8f) ? opt.gcamAltKm * 1000.0 : planetR * 2.1;
        camGlobe = scene::GlobeCamera(gLat, gLon, gAlt, planetR);
        camGlobe.fovY = cam.fovY;
        camGlobe.speed = 800000.0f;
    }
    camChart = cam;
    if (mode == 1) cam = camSea;   // camGlobe start applies below, after frame conversion

    // ---- M6g: ONE WORLD, ONE FRAME. Everything renders in the estuary's tangent frame
    // (the planet's centre sits at flat (0, -R, 0)); the globe rotates into it, the
    // estuary layers were always in it, and the old mode-switch handoff -- the last
    // smoke-and-mirror in the engine -- is DELETED. The pose maps below survive only to
    // express orbit keyframes and cull volumes. (Mars anchors its frame at (0N, 0E).)
    GlobeModel::LatLonDir(marsMode ? 0.0 : BathyModel::kOrgLat,
                          marsMode ? 0.0 : BathyModel::kOrgLon, oDir);
    {
        const double yl = std::sqrt(oDir[0] * oDir[0] + oDir[2] * oDir[2]);
        east0[0] = -oDir[2] / yl; east0[1] = 0.0; east0[2] = oDir[0] / yl;   // d(dir)/dlon
        // M6i: north = east x up -- NOT up x east. This planet frame (x at 0N0E, y at the
        // POLE, z at 90E) is an odd permutation of ECEF, so the familiar identity flips
        // sign; the old cross order produced SOUTH, making the planet->tangent map a
        // REFLECTION (det -1). Every lat/lon-registered layer rendered N/S-mirrored about
        // the anchor relative to the world-metre layers -- the "inverted textures" of the
        // M6h report, invisible until the terrain wore imagery. The det check below makes
        // a reflection impossible to reintroduce silently.
        north0[0] = east0[1] * oDir[2] - east0[2] * oDir[1];
        north0[1] = east0[2] * oDir[0] - east0[0] * oDir[2];
        north0[2] = east0[0] * oDir[1] - east0[1] * oDir[0];
        const double det =
            east0[0] * (oDir[1] * north0[2] - oDir[2] * north0[1]) -
            east0[1] * (oDir[0] * north0[2] - oDir[2] * north0[0]) +
            east0[2] * (oDir[0] * north0[1] - oDir[1] * north0[0]);
        if (det < 0.999) {
            Log("FATAL: frame basis det %.3f -- planet->tangent must be a proper rotation",
                det);
            return 1;
        }
    }
    // M12 step 4d: THE SPACES, DECLARED (core/Space.h). The planet at unit length R --
    // "planet.re" -- and the tangent frame under it, its link the rows just derived: own x, y,
    // z = east, up, north in the planet's frame (Frame() keeps the rows exactly and derives the
    // rotor), its origin the anchor on the sphere (the centre sits at (0, -R, 0) in the tangent
    // frame, as the pose maps below say). Its unit length is R as well: Space.h's banner
    // prescribes the RULE (priors 32 -- the points a space names sit near |x| ~ 1 unit, checked
    // by Declare() against its extent), not a number, and this frame names the whole planet (the
    // walk, and the Droste tower: Droste.h builds the portal's versor at R for that reason);
    // SpaceTest's 1 m tangent is the boat-scale case. Declare() is the rule said once; the cycle
    // below inherits the unit it hangs under.
    {
        m_planetSpace.name = "planet.re";
        m_planetSpace.unitM = planetR;
        m_planetSpace.extentM = 2.0 * planetR;
        m_tangentSpace.name = marsMode ? "tangent.mars" : "tangent.merrimack";
        m_tangentSpace.unitM = planetR;
        m_tangentSpace.extentM = 2.0 * planetR;
        m_tangentSpace.parent = &m_planetSpace;
        const double anchor[3] = {oDir[0] * planetR, oDir[1] * planetR, oDir[2] * planetR};
        m_tangentSpace.link = Placement::Frame(east0, oDir, north0, anchor);
        std::string why;
        if (!m_planetSpace.Declare(&why) || !m_tangentSpace.Declare(&why)) {
            Log("FATAL: [space] %s", why.c_str());
            return 1;
        }
    }
    // M12 step 5a: the pose maps' bodies moved VERBATIM to scene/Pose.h, so the scene's
    // placement sugar is pinned against the functions the session itself calls; the lambdas
    // keep their names and their captures (the surface's rows, planetR).
    auto planetToFlatPose = [&](const Camera& g) -> Camera {
        return scene::PlanetToFlatPose(g, east0, oDir, north0, planetR);
    };
    auto flatToPlanetPose = [&](const Camera& f) -> Camera {
        return scene::FlatToPlanetPose(f, east0, oDir, north0, planetR);
    };
    // The camera bookmarks live in the ONE frame now: convert the orbit start pose, and
    // hand the globe its frame + the CUDEM window (for the foundation sink).
    camGlobe = planetToFlatPose(camGlobe);
    if (mode == 1 && opt.globeStart) cam = camGlobe;
    if (globe) {
        // (M12 step 4a: the rows above went into the surface the globe reads -- SetSurface.)
        if (bathy.Ready()) {
            const double lon0 = BathyModel::kOrgLon + bathy.WorldX0() / BathyModel::kMPerLon;
            const double lat1 = BathyModel::kOrgLat +
                                (bathy.WorldZ0() + bathy.WorldSizeZ()) / BathyModel::kMPerLat;
            globe->SetEstuaryWindow(lon0, lat1, bathy.WorldSizeX() / BathyModel::kMPerLon,
                                    bathy.WorldSizeZ() / BathyModel::kMPerLat);
        }
    }
    // ---- M10: THE DROSTE LINK (src/core/Droste.h). The root's ADDRESS, hung as a leaf:
    // the quadtree leaf at (drosteLat, drosteLon, drosteLevel) gets the root as its child.
    // What that means in space follows from the address alone -- the globe's diameter is
    // the leaf's span (x fill), it rests on the composed ground at the leaf's centre, and
    // the fixed point where the tower converges is then FORCED by the similarity. The link
    // is one Cl(4,1) versor; everything per frame is its closed form.
    if (opt.droste && globe && !marsMode) {
        double pd[3];
        GlobeModel::LatLonDir(opt.drosteLat, opt.drosteLon, pd);
        int lf = 0;
        uint32_t lix = 0, liy = 0;
        GlobeLayer::LeafOf(pd, opt.drosteLevel, lf, lix, liy);
        double ld[3];
        GlobeLayer::LeafDir(lf, opt.drosteLevel, lix, liy, ld);
        const double latC = std::asin(std::clamp(ld[1], -1.0, 1.0));
        const double lonC = std::atan2(ld[2], ld[0]);
        const double spanM =
            (3.14159265358979 / 2.0) * planetR / double(1u << opt.drosteLevel);
        const double ground =
            (hgtCh >= 0) ? double(compositor.SampleHeightStack(hgtCh, latC, lonC, spanM * 0.25))
                         : 0.0;
        const double twist = opt.drosteTwistDeg * 3.14159265358979 / 180.0;
        const double axisN[3] = {0.0, 0.0, 1.0};   // the anchor's north, in the one frame
        portal = droste::BuildPortal(lf, opt.drosteLevel, lix, liy, ld, east0, oDir, north0,
                                     planetR, ground, opt.drosteFill, axisN, twist);
        Log("[droste] the root hangs at leaf (face %d, level %d, %u, %u) = %.5f N %.5f E: "
            "globe radius %.2f m (s %.4e, %.2f decades a level) resting on %.2f m, centre "
            "(%.2f, %.2f, %.2f), twist %.1f deg about north, fixed point (%.4f, %.4f, %.4f)",
            lf, opt.drosteLevel, lix, liy, latC * 57.29577951308232,
            lonC * 57.29577951308232, portal.radius, portal.s, -std::log10(portal.s), ground,
            portal.centre[0], portal.centre[1], portal.centre[2], opt.drosteTwistDeg,
            portal.p[0], portal.p[1], portal.p[2]);
        // M12 step 4d: THE CYCLE, DECLARED. The root's tangent frame hung under its own leaf by
        // the portal's similarity, as a Space: Level(k) = S^k. Similar(p, s, axis, twist) takes
        // the twist in RADIANS, as Portal::twist holds it (radians per level); p, s and axis are
        // the portal's own (BuildPortal resolved the address into them, and stays the builder).
        drosteLeaf = Space::Cycle("droste.leaf", m_tangentSpace,
                                  Placement::Similar(portal.p, portal.s, portal.axis, portal.twist));
        Log("[space] droste.leaf: %s hung under its own leaf, S = Similar(p, s %.4e, axis (%.0f, "
            "%.0f, %.0f), twist %.17g rad); Level(k) = S^k, unit length %.4g m",
            m_tangentSpace.name.c_str(), portal.s, portal.axis[0], portal.axis[1], portal.axis[2],
            portal.twist, drosteLeaf.unitM);
    }
    // M12 step 4g: the composed-surface rows are filled once a FRAME, into the renderer's one
    // surface buffer (b2), in Frame() beside the renderer's other per-frame members -- where
    // this block filled the terrain's, the sea's and the GIS layer's copies once, at boot
    // (M9ap: all of them sample the SAME page tenant the globe does, slices 6 and 7
    // included), and the globe refilled its own every frame.
    // ---- M6j: the first GA-product-buffer plugin, end to end. The CPU ORGANIZES: one
    // PGA motor per tide station, placing and orienting a pylon on the sphere (Pga.h --
    // conventions pinned by selftest). The Exchange CARRIES: a typed, versioned channel.
    // The GPU RENDERS: Markers.hlsl applies the same sandwich (GA.hlsli). This is the
    // socket a physics plugin drives with real mesh/vertex buffers later.
    if (!marsMode && globe && model.Count() > 0) {
        struct MarkerRec {
            float re[4], du[4], scaleColor[4];
        };
        std::vector<MarkerRec> recs;
        for (size_t i = 0; i < model.Count(); ++i) {
            const TideStation& st = model.S(i);
            double d[3];
            GlobeModel::LatLonDir(st.lat, st.lon, d);
            const double upF[3] = {east0[0] * d[0] + east0[1] * d[1] + east0[2] * d[2],
                                   oDir[0] * d[0] + oDir[1] * d[1] + oDir[2] * d[2],
                                   north0[0] * d[0] + north0[1] * d[1] + north0[2] * d[2]};
            const double fx = upF[0] * planetR;
            const double fy = upF[1] * planetR - planetR + 2.0;   // base ~2 m above geoid
            const double fz = upF[2] * planetR;
            // Align the pylon's +y with the LOCAL up: rotate about y x up.
            double axis[3] = {upF[2], 0.0, -upF[0]};   // cross((0,1,0), upF), y term 0
            const double axLen =
                std::sqrt(axis[0] * axis[0] + axis[1] * axis[1] + axis[2] * axis[2]);
            Motor m = Motor::Translation(fx, fy, fz);
            if (axLen > 1e-9) {
                axis[0] /= axLen;
                axis[1] /= axLen;
                axis[2] /= axLen;
                const double org[3] = {0, 0, 0};
                const double ang = std::atan2(axLen, upF[1]);
                m = m * Motor::Rotation(org, axis, ang);
            }
            MarkerRec r{};
            r.re[0] = static_cast<float>(m.s);
            r.re[1] = static_cast<float>(m.r23);
            r.re[2] = static_cast<float>(m.r31);
            r.re[3] = static_cast<float>(m.r12);
            r.du[0] = static_cast<float>(m.q);
            r.du[1] = static_cast<float>(m.t01);
            r.du[2] = static_cast<float>(m.t02);
            r.du[3] = static_cast<float>(m.t03);
            r.scaleColor[0] = 8.0f;    // half-width m
            r.scaleColor[1] = 45.0f;   // height m
            r.scaleColor[2] = static_cast<float>(i);
            recs.push_back(r);
        }
        const int ch = exchange.Register(
            "markers.stations",
            {48, 0b10101, "pga-motor (dq8: re s/r23/r31/r12, du q/t01/t02/t03) + "
                          "halfwidth/height/colorIdx"},
            "plugin.tideStations");
        exchange.Publish(gpu, ch, recs.data(), recs.size() * sizeof(MarkerRec));
        exchange.LogRegistry();
    }
    // Altitude above the geoid, valid at any longitude (flat y is NOT altitude far from
    // the origin): |flat + (0,R,0)| - R, in doubles.
    altOf = [&](const Camera& c) {
        const double y = c.py + planetR;
        return std::sqrt(c.px * c.px + y * y + c.pz * c.pz) - planetR;
    };

    // ---- M6b: the camera ON RAILS -- the debug flight, as GA. Each keyframe pose is a
    // MOTOR (position and aim as one element); segments interpolate with the screw Slerp
    // (M0 Exp(u Log(~M0 M1))), so the descent from orbit is one smooth helical motion per
    // leg, eased at the ends. Extraction back to yaw/pitch drops any interpolated roll --
    // the horizon stays level, Google-Earth style.
    // (M12 step 5a: bodies in scene/Pose.h -- FromCamera / ToCamera, verbatim.)
    poseMotor = [&](const Camera& c) -> Motor { return scene::FromCamera(c); };
    motorPose = [&](const Motor& m, Camera& c) { scene::ToCamera(m, c); };
    // Keys (all in the PLANET frame; flat poses go through flatToPlanetPose):
    //   0-5 s   orbit -> 2.6 km over the estuary (via a 500 km mid key: no screw dives)
    //   5-10 s  hold over the river (the handoff has already switched to the estuary)
    //  10-15 s  descend to the north-jetty helm
    //  15-25 s  hold the helm while the real-time sea runs
    // A pose on any planet: stand at (lat, lon, alt), aim at a surface target.
    auto orbPose = [&](double lat, double lon, double altM, double tLat, double tLon) {
        return scene::OrbitPose(lat, lon, altM, tLat, tLon, planetR);   // 5a: scene/Pose.h
    };
    // M6g: every key is a FLAT-frame pose now -- the rails never change frames, because
    // there is only one. orbPose builds in planet terms for readability and converts.
    auto orbKey = [&](double lat, double lon, double altM, double tLat, double tLon) {
        return poseMotor(planetToFlatPose(orbPose(lat, lon, altM, tLat, tLon)));
    };
    if (globe && marsMode) {
        // The Mars flyover: fall in over Valles Marineris, run the canyon west along its
        // 4000 km, then climb toward Tharsis with Olympus Mons on the horizon.
        railKeys.push_back({0.0, orbKey(10, -25, planetR * 1.1, -12, -58)});
        railKeys.push_back({7.0, orbKey(-7, -40, 800e3, -13, -62)});
        railKeys.push_back({13.0, orbKey(-11, -52, 220e3, -13, -72)});
        railKeys.push_back({19.0, orbKey(-13, -68, 150e3, -11, -90)});
        railKeys.push_back({26.0, orbKey(-6, -98, 600e3, 18.6, -133.8)});
        railKeys.push_back({30.0, orbKey(-4, -104, 900e3, 18.6, -133.8)});
    } else if (globe && opt.railFlood && bathy.Ready()) {
        // M6s: THE FLOOD RIDE -- orbit to the throat, ending at a boat's-helm pose
        // mid-channel WEST of the gap, facing the entrance: the surveyed jetties (world
        // z +60..+155 north, -225..-76 south, tips near x 640) frame the incoming tide.
        // Shoot with --start at a max-flood hour so the jet pours toward the camera.
        Camera cOver;    // 1.5 km over the harbor, aimed down-channel at the gap
        cOver.SetFromCompass(-1400.0, 1500.0, 0.0, 93.0f, -40.0f);
        Camera cHelmIn;  // helm height in the channel, the entrance dead ahead
        cHelmIn.SetFromCompass(-250.0, 9.0, -15.0, 92.0f, -2.0f);
        Camera cHelmGap; // ...then a ~3 kn push to between the jetty roots, gap 500 m out
        cHelmGap.SetFromCompass(120.0, 7.0, -10.0, 92.5f, -1.5f);
        railKeys.push_back({0.0, poseMotor(camGlobe)});
        railKeys.push_back({8.0, orbKey(41.9, -71.6, 800e3, 42.8183, -70.81)});
        railKeys.push_back({15.0, orbKey(42.55, -70.98, 80e3, 42.8183, -70.81)});
        railKeys.push_back({21.0, orbKey(42.74, -70.87, 7e3, 42.8183, -70.81)});
        railKeys.push_back({26.0, poseMotor(cOver)});
        railKeys.push_back({32.0, poseMotor(cHelmIn)});
        railKeys.push_back({40.0, poseMotor(cHelmGap)});
    } else if (globe && opt.railJetty && bathy.Ready()) {
        // M7c: THE JETTY PASS -- the shot the refracted ray was built for. Tip to tip
        // across the entrance at helm height (the surveyed jetties: world z +60..+155
        // north, -225..-76 south, tips near x 640), the bar and the channel reading
        // through the surface the whole way, then a climb to a bird's eye where the
        // same formula turns into the chart: the ebb shoal, the throat, the flats,
        // depth as color. Shoot at a LOW-TIDE hour (--start) so the bars stand proud.
        // M7e: GROUND TO SPACE -- helm water at the gap, tip-to-tip pass, then one
        // continuous climb to orbit: the same water, the same formula, every altitude.
        Camera cHelm;    // on the water mid-channel, the entrance dead ahead
        cHelm.SetFromCompass(250.0, 5.0, 40.0, 94.0f, -1.0f);
        Camera cNTip;    // over the north tip, the gap ahead
        cNTip.SetFromCompass(680.0, 14.0, 200.0, 192.0f, -10.0f);
        Camera cMid;     // mid-gap, swung to look west up the channel
        cMid.SetFromCompass(650.0, 12.0, -30.0, 262.0f, -8.0f);
        Camera cSTip;    // over the south tip, looking back northwest across the gap
        cSTip.SetFromCompass(680.0, 16.0, -290.0, 300.0f, -13.0f);
        Camera cRise;    // climbing, the whole entrance opening below
        cRise.SetFromCompass(520.0, 420.0, -140.0, 284.0f, -56.0f);
        Camera cBird;    // bird's eye over the entrance: depth as color
        cBird.SetFromCompass(380.0, 1500.0, 10.0, 272.0f, -88.0f);
        railKeys.push_back({0.0, poseMotor(cHelm)});
        railKeys.push_back({4.0, poseMotor(cHelm)});
        railKeys.push_back({9.0, poseMotor(cNTip)});
        railKeys.push_back({14.0, poseMotor(cMid)});
        railKeys.push_back({18.0, poseMotor(cSTip)});
        railKeys.push_back({21.5, poseMotor(cRise)});
        railKeys.push_back({25.0, poseMotor(cBird)});
        railKeys.push_back({28.0, poseMotor(cBird)});
        railKeys.push_back({31.0, orbKey(42.79, -70.84, 7e3, 42.8183, -70.81)});
        railKeys.push_back({34.5, orbKey(42.62, -70.95, 80e3, 42.8183, -70.81)});
        railKeys.push_back({38.0, orbKey(41.9, -71.6, 800e3, 42.8183, -70.81)});
        railKeys.push_back({40.0, orbKey(41.9, -71.6, 800e3, 42.8183, -70.81)});
    } else if (globe && opt.railZoom && bathy.Ready()) {
        // The inlet zoom: orbit -> the warmed Google pyramid -> the CUDEM estuary, with NO
        // handoff to hide behind any more: the same scene refines the whole way down.
        railKeys.push_back({0.0, poseMotor(camGlobe)});
        railKeys.push_back({8.0, orbKey(41.9, -71.6, 800e3, 42.8183, -70.81)});
        railKeys.push_back({16.0, orbKey(42.55, -70.98, 80e3, 42.8183, -70.81)});
        railKeys.push_back({23.0, orbKey(42.74, -70.87, 7e3, 42.8183, -70.81)});
        railKeys.push_back({27.0, orbKey(42.79, -70.84, 2400.0, 42.8183, -70.81)});
        railKeys.push_back({30.0, orbKey(42.80, -70.835, 2200.0, 42.8183, -70.81)});
    } else if (globe && sea && bathy.Ready() && !marsMode) {
        Camera cHover;
        cHover.SetFromCompass(-200.0, 1800.0, -2500.0, 22.0f, -46.0f);
        Camera cHelm;
        cHelm.SetFromCompass(522.0, 7.0, 72.0, 246.0f, -4.0f);
        railKeys.push_back({0.0, poseMotor(camGlobe)});
        railKeys.push_back({3.0, orbKey(41.2, -66.5, 500000.0,
                                        BathyModel::kOrgLat, BathyModel::kOrgLon)});
        railKeys.push_back({5.0, poseMotor(cHover)});
        railKeys.push_back({10.0, poseMotor(cHover)});
        railKeys.push_back({15.0, poseMotor(cHelm)});
        railKeys.push_back({25.0, poseMotor(cHelm)});
    }
    railPose = [&](double t, Camera& out) {
        size_t i = 0;
        while (i + 1 < railKeys.size() && railKeys[i + 1].first <= t) ++i;
        if (i + 1 >= railKeys.size()) {
            motorPose(railKeys.back().second, out);
            return;
        }
        const double t0 = railKeys[i].first, t1 = railKeys[i + 1].first;
        double u = (t - t0) / std::max(t1 - t0, 1e-6);
        u = u * u * (3.0 - 2.0 * u);   // ease both ends of every leg
        motorPose(Motor::Slerp(railKeys[i].second, railKeys[i + 1].second, u), out);
    };

    // ---- M10: THE DIVE (--rail-droste). The storm rail flies its keys to the helm -- its
    // last key re-aimed at the fixed point -- and then the camera leaves the keys for the
    // similarity's own one-parameter subgroup:
    //
    //     pose(u) = S^u(helm):   C(u) = p + s^u Q^u (C_helm - p),   view Q^u (fwd, up)
    //
    // the logarithmic spiral through the helm that converges on the fixed point. PGA could
    // not write this path -- a motor has no scale -- and nothing about it is keyed: it is
    // exp(u log S), with the level n = floor(u) handed to the frame and f = u - n the only
    // number ever pushed through the versor, so the doubles never see s^n. Every level of
    // the dive is the SAME flight one level down (S^(u+1) = S S^u); only the world around
    // the tower says which level it is. The spiral runs at a constant rate in log-scale,
    // d(ln |C - p|)/dt = ln(s) / T -- the logarithm the user asked for, with gravity's
    // signature: the approach never arrives, it only gets smaller.
    drosteHelm.SetFromCompass(120.0, 7.0, -10.0, 92.5f, -1.5f);   // the storm rail's helm
    drosteHelm.fovY = cam.fovY;
    // WHICH POSE THE SPIRAL RUNS THROUGH. The helm's spiral clears the inner globe only if
    // the twist lifts it up and over: MEASURED (clearance over dist-to-p along S^u(helm),
    // u in [0, 1.2]) -0.21 at 0 deg, -0.10 at 30, -0.02 at 60, +0.012 at 90. With less twist
    // the line from the fixed point through the helm, extended outward, runs underground in
    // the inner frame. So an untwisted tower dives from ABOVE: 45 deg over the mouth, looking
    // down at the fixed point -- the classic straight Droste zoom, outside the globe for
    // every u because the approach lies above the globe's tangent plane at the top.
    diveFromAbove = portal.Valid() && std::abs(opt.drosteTwistDeg) < 75.0;
    if (diveFromAbove) {
        drosteHelm.px = portal.p[0] - 300.0;
        drosteHelm.py = portal.p[1] + 300.0;
        drosteHelm.pz = portal.p[2];
    }
    if (portal.Valid()) {
        drosteHelm.LookAt(portal.p[0], portal.p[1], portal.p[2]);
        const double gy = drosteHelm.py + planetR;
        const double gl = std::sqrt(drosteHelm.px * drosteHelm.px + gy * gy +
                                    drosteHelm.pz * drosteHelm.pz);
        drosteHelm.upHint[0] = static_cast<float>(drosteHelm.px / gl);
        drosteHelm.upHint[1] = static_cast<float>(gy / gl);
        drosteHelm.upHint[2] = static_cast<float>(drosteHelm.pz / gl);
        DirectX::XMFLOAT3 hf, hr, hu;
        drosteHelm.ViewBasis(hf, hr, hu);   // the helm's TRUE up: the roll the spiral carries
        drosteHelmUp[0] = hu.x;
        drosteHelmUp[1] = hu.y;
        drosteHelmUp[2] = hu.z;
        if ((opt.railDroste || opt.railDrosteOut) && !railKeys.empty()) {
            railKeys.back().second = poseMotor(drosteHelm);
        }
    }
    // S^u(base), written in level floor(u): the pose, its level, and its up. The base is the
    // helm (the dive) or the helm turned around (the out-and-back's return leg).
    diveFrom = [&](const Camera& base, double u, Camera& out, int& level, double up[3]) {
        const double n = std::floor(u);
        const double f = u - n;
        const double c0[3] = {base.px, base.py, base.pz};
        const DirectX::XMFLOAT3 hf = base.Forward();
        const double f0[3] = {hf.x, hf.y, hf.z};
        double c[3], fw[3];
        // M12 step 4d-2: S^f(helm) through the cycle -- LevelApply(f) on the eye, LevelApplyDir(f)
        // on the forward and the up (Space.h: the power about its fixed point) -- with the
        // portal's closed forms evaluated beside them for the record (ProbeDive).
        ProbeDive(f, c0, f0, c, fw, up, true, u);
        out = base;
        out.px = c[0];
        out.py = c[1];
        out.pz = c[2];
        out.yaw = static_cast<float>(std::atan2(fw[2], fw[0]));
        const float lim = 3.14159265f / 2.0f - 0.0017f;
        out.pitch = std::clamp(
            static_cast<float>(std::atan2(fw[1], std::sqrt(fw[0] * fw[0] + fw[2] * fw[2]))),
            -lim, lim);
        level = static_cast<int>(n);
    };
    diveAt = [&](double u, Camera& out, int& level, double up[3]) {
        diveFrom(drosteHelm, u, out, level, up);
    };
    // The out-and-back's other face of the helm: turned about the local vertical to look
    // back up the channel, a little down at the water.
    drosteHelmBack = drosteHelm;
    drosteHelmBack.yaw = drosteHelm.yaw + 3.14159265f;
    drosteHelmBack.pitch = -0.07f;
    // A leg that moves u by dU in D seconds with 3 s velocity ramps at both ends: rest,
    // the subgroup's constant log-rate, rest.
    legU = [](double tau, double D, double dU) {
        const double r = (std::min)(3.0, 0.5 * D);
        const double v = dU / (D - r);
        if (tau <= 0.0) return 0.0;
        if (tau >= D) return dU;
        if (tau < r) return v * tau * tau / (2.0 * r);
        if (tau <= D - r) return v * (tau - 0.5 * r);
        const double e = D - tau;
        return dU - v * e * e / (2.0 * r);
    };
    // The dive's clock: u(t) eased in over the first seconds (C1 -- the camera leaves the
    // helm from rest, then runs at the subgroup's constant log-rate), one level per
    // drosteLevelSec.
    diveU = [&](double tau) {
        const double T = opt.drosteLevelSec, ramp = 3.0;
        if (tau <= 0.0) return 0.0;
        const double u = (tau < ramp) ? tau * tau / (2.0 * ramp * T) : (tau - 0.5 * ramp) / T;
        // The dive rail ends ON a helm (a whole level): it holds there for the last frames.
        return opt.railDroste ? (std::min)(u, double(opt.drosteLevels)) : u;
    };
    // M12 step 4d instrument: the dive's closed forms compared over the rail's own schedule --
    // every recorded frame of the dive at 30 fps, u = diveU(tau), tau to the last helm hold --
    // here, where the helm and its up are final, so every Droste run's log carries the dive's
    // verdict (the rail reaches its spiral 1260 recorded frames in); the live rail compares the
    // same way, one line per dive frame.
    if (portal.Valid()) {
        const int nF = static_cast<int>((opt.drosteLevels * opt.drosteLevelSec + 2.0) * 30.0);
        const double c0[3] = {drosteHelm.px, drosteHelm.py, drosteHelm.pz};
        const DirectX::XMFLOAT3 hf = drosteHelm.Forward();
        const double f0[3] = {hf.x, hf.y, hf.z};
        for (int i = 0; i <= nF; ++i) {
            const double u = diveU(double(i) / 30.0);
            double c[3], fw[3], up[3];
            ProbeDive(u - std::floor(u), c0, f0, c, fw, up, false, u);
        }
        Log("[droste] probe dive rail (sweep of %d frames' u, %d levels x %.0f s): c %s | fw %s "
            "| up %s",
            nF + 1, opt.drosteLevels, opt.drosteLevelSec, m_probeSweepC.Verdict().c_str(),
            m_probeSweepFw.Verdict().c_str(), m_probeSweepUp.Verdict().c_str());
    }
    // THE OUT-AND-BACK (--rail-droste-out, the user's side quest): in two levels, turn
    // around at the bottom, fly back out along the SAME logarithmic spiral facing outward --
    // S^u of the turned helm, u running 2 -> 0, the frame re-rooting outward on its own --
    // then turn back and climb away along the storm rail's keys reversed, the tower
    // shrinking into its entrance until the planet is whole again.
    if (portal.Valid() && opt.railDrosteOut) {
        Camera cRise;   // rising over the harbor, looking back east at the entrance
        cRise.SetFromCompass(-900.0, 450.0, -60.0, 84.0f, -17.0f);
        climbKeys.push_back({0.0, poseMotor(drosteHelmBack)});
        climbKeys.push_back({7.0, poseMotor(cRise)});
        climbKeys.push_back({13.0, orbKey(42.74, -70.87, 7e3, 42.8183, -70.81)});
        climbKeys.push_back({18.5, orbKey(42.55, -70.98, 80e3, 42.8183, -70.81)});
        climbKeys.push_back({23.5, orbKey(41.9, -71.6, 800e3, 42.8183, -70.81)});
        climbKeys.push_back({28.0, poseMotor(camGlobe)});
    }
    keyedPose = [&](const std::vector<std::pair<double, Motor>>& keys, double t,
                         Camera& out) {
        size_t i = 0;
        while (i + 1 < keys.size() && keys[i + 1].first <= t) ++i;
        if (i + 1 >= keys.size()) {
            motorPose(keys.back().second, out);
            return;
        }
        double u = (t - keys[i].first) / std::max(keys[i + 1].first - keys[i].first, 1e-6);
        u = u * u * (3.0 - 2.0 * u);
        motorPose(Motor::Slerp(keys[i].second, keys[i + 1].second, u), out);
    };
    gravityUp = [&](const Camera& c, double up[3]) {
        const double gy = c.py + planetR;
        const double gl = std::sqrt(c.px * c.px + gy * gy + c.pz * c.pz);
        up[0] = c.px / gl;
        up[1] = gy / gl;
        up[2] = c.pz / gl;
    };
    drosteRailPose = [&](double t, Camera& out, int& level, double up[3]) {
        level = 0;
        if (opt.railDroste) {
            if (t < 40.0) {
                railPose(t, out);   // the storm rail (its last key aimed at the fixed point)
                gravityUp(out, up);
                return;
            }
            diveAt(diveU(t - kDiveT0), out, level, up);
            return;
        }
        // --rail-droste-out
        const double T = opt.drosteLevelSec, Din = 2.0 * T, Dout = 2.0 * T, Tturn = 3.0;
        double tau = t;
        if (tau < 14.0) {   // the storm rail's last leg: 1.5 km over the harbor -> the helm
            railPose(26.0 + tau, out);
            gravityUp(out, up);
            return;
        }
        tau -= 14.0;
        if (tau < 2.0) {    // a breath at the helm, the tower dead ahead
            diveAt(0.0, out, level, up);
            return;
        }
        tau -= 2.0;
        if (tau < Din) {    // IN: u 0 -> 2
            diveAt(legU(tau, Din, 2.0), out, level, up);
            return;
        }
        tau -= Din;
        if (tau < Tturn) {  // THE TURN, at the second level's helm
            double w = tau / Tturn;
            w = w * w * (3.0 - 2.0 * w);
            motorPose(Motor::Slerp(poseMotor(drosteHelm), poseMotor(drosteHelmBack), w), out);
            level = 2;
            for (int i = 0; i < 3; ++i) up[i] = drosteHelmUp[i];
            return;
        }
        tau -= Tturn;
        if (tau < Dout) {   // OUT: u 2 -> 0, facing outward
            diveFrom(drosteHelmBack, 2.0 - legU(tau, Dout, 2.0), out, level, up);
            return;
        }
        tau -= Dout;
        keyedPose(climbKeys, tau, out);   // the climb to orbit, looking back at the tower
        gravityUp(out, up);
    };
    if (opt.camAlt > 0) {
        const double cx = (opt.camX < 1e8f) ? opt.camX : 0.0;
        const double cz = (opt.camZ < 1e8f) ? opt.camZ : 0.0;
        cam.SetFromCompass(cx, opt.camAlt, cz, opt.camAz, opt.camPitch);
    }

    simUnix = (opt.startUnix > 0) ? opt.startUnix : NowUnix();
    // The playable clock (sim/SimClock.h). Headless keeps its own frame-indexed formula
    // below -- that path was already fixed-step, which is why rails reproduce and sessions
    // did not.
    simClock.Reset(simUnix);

    // ==================================================================================
    //  M9bq THE BOAT. A vessel from the registry, stepped on the scene clock's whole
    //  quanta against the tree's own water. Nothing here is a demo path: this is the
    //  same Vessel the gates exercise, the same TreeWater the twin measures, and the
    //  same SimClock everything else in the scene rides.
    // ==================================================================================
    RegisterBuiltinVessels(vesselReg);
    if (!opt.boat.empty()) {
        const VesselSpec spec = vesselReg.Build(opt.boat);
        if (spec.kind.empty()) {
            Log("[vessel] --boat '%s' is not a registered kind; known:", opt.boat.c_str());
            for (const std::string& k : vesselReg.Kinds()) Log("[vessel]   %s", k.c_str());
        } else {
            spec.PrintLedger();
            boat = std::make_unique<Vessel>();
            // Spawn in the FLAT world frame, at --campos. NOT at cam.px/cam.pz: on a
            // rail the camera starts in the PLANET frame, and a hull built there lands at
            // (5.3e6, -2.4e6) where the bed lookup is meaningless -- it reported AGROUND at
            // depth -278 m, which is exactly what a frame confusion looks like from inside.
            const Motor start = Motor::Translation(opt.camX, 0.0, opt.camZ);
            if (!boat->Build(spec, start)) {
                Log("[vessel] build FAILED -- not spawning (a partial hull would sink and "
                    "look like a physics bug)");
                boat.reset();
            } else {
                helming = true;
                Log("[vessel] '%s' spawned at (%.1f, %.1f): %s", spec.kind.c_str(), cam.px,
                    cam.pz, spec.display.c_str());
            }
        }
    }
    startUnix = simUnix;

    // M5c boundary clocks. Ocean = the ENTRANCE station (the physically right open-water
    // level; Newburyport stays the chart focus). West = the river tide interpolated to the
    // window's west edge (~km 6, between Newburyport at 4.4 and Salisbury Point at 8.2),
    // expressed as a DEVIATION from the ocean tide so the datum offset cancels.
    entSta = model.Focus(); westA = model.Focus(); westB = model.Focus();
    // The west edge's along-channel kilometre tracks the WINDOW (straight-line distance is
    // a fine proxy on this reach): ~5.3 km for the original mouth window, ~15.5 for the
    // M6d wide window (bracketed by Merrimacport/Riverside instead of Newburyport/Salisbury).
    kWestKm =
        bathy.Ready() ? std::min(20.0, std::abs(bathy.WorldX0()) / 1000.0) : 6.0;
    {
        double bestEnt = 1e9, bestA = -1e9, bestB = 1e9;
        for (size_t i = 0; i < model.Count(); ++i) {
            const double km = model.S(i).riverKm;
            if (km < 0) continue;
            if (km < bestEnt) { bestEnt = km; entSta = static_cast<int>(i); }
            if (km <= kWestKm && km > bestA) { bestA = km; westA = static_cast<int>(i); }
            if (km > kWestKm && km < bestB) { bestB = km; westB = static_cast<int>(i); }
        }
        if (bestB > 1e8) westB = westA;
    }
    wT = (model.S(westB).riverKm > model.S(westA).riverKm)
                          ? (kWestKm - model.S(westA).riverKm) /
                                (model.S(westB).riverKm - model.S(westA).riverKm)
                          : 0.0;
    oceanAt = [&](double t) { return model.Height(entSta, t) + datumOff; };
    // The sound's tide: the entrance clock ~10 min later (its Ipswich mouth is a few km
    // down an open coast). Active only when the window holds the sound.
    southAt = [&, hasSound](double t) {
        return hasSound ? model.Height(entSta, t - 600.0) - model.Height(entSta, t) : 0.0;
    };
    // TIDAL parts only: each station's Height is in its OWN local MLLW, so raw differences
    // smuggle a constant datum offset into the boundary (with the Merrimacport bracket that
    // was a permanent 19 cm seaward slope -- an artificial ever-ebb). The true NAVD river
    // slope at this reach is cm-scale; call it zero and let the tide be the signal.
    westAt = [&](double t) {
        if (opt.sweWestOff) return 0.0;
        const double tw = (model.Height(westA, t) - model.S(westA).meanMllwM) * (1.0 - wT) +
                          (model.Height(westB, t) - model.S(westB).meanMllwM) * wT;
        return tw - (model.Height(entSta, t) - model.S(entSta).meanMllwM);
    };
    // M6r: the transport the Flather west boundary must CARRY (+east): river discharge
    // minus the upriver prism demand. The reach beyond the window (km ~15.5 to the head
    // of tide at Haverhill, ~km 35) fills and drains THROUGH this boundary; its surface
    // area is the prism knob (~19.5 km of ~200 m river; an NHD-integrated area is the
    // named refinement). d(eta_west)/dt by central difference of the station-fit clocks.
    westQAt = [&, riverQ](double t) {
        if (opt.sweWestOff) return 0.0;
        const double dh = (oceanAt(t + 300.0) + westAt(t + 300.0) - oceanAt(t - 300.0) -
                           westAt(t - 300.0)) / 600.0;
        return riverQ - kUpriverAreaM2 * dh;
    };
    if (swe.Ready()) {
        Log("[swe] boundaries: ocean=%s, west=lerp(%s,%s,%.2f) Flather "
            "(Q %.0f m^3/s, prism area %.1f km^2)",
            model.S(entSta).name.c_str(), model.S(westA).name.c_str(),
            model.S(westB).name.c_str(), wT, riverQ, kUpriverAreaM2 / 1.0e6);
    }

    // ---- M6x: THE GLOBAL WEATHER/PHYSICS MANAGER -- one query surface over every water
    // and near-surface product, each component answered by the finest resident rung.
    // The Merrimack solver registers as an EXTERNAL window (the render loop drives it);
    // Boston Harbor registers DORMANT and spins up when the camera arrives -- detail
    // rises on zoom because residency rises on zoom. The manager's RENDER product (the
    // user's contract): ONE 2D tiled resource of wave vertexes + parameter buffers, LOD
    // by tiled residency / amplification / tessellation -- no piecewise water. That bank
    // composes exactly what the manager already owns (tide rotors, solver eta/uv banks,
    // cascade displacement, the one bed) and is the M7 milestone.
    weather.Init(&compositor, hgtCh, &waterAtlas, &model, &globeModel, &seaState,
                 haveCurrents ? &currents : nullptr);
    if (swe.Ready()) weather.AddExternalWindow("merrimack", &swe, &bathy, oceanAt);
    if (bathyBoston.Ready()) {
        int iBos = -1;
        for (size_t i = 0; i < model.Count(); ++i) {
            if (model.S(i).id == "8443970") iBos = static_cast<int>(i);
        }
        if (iBos >= 0 && model.S(iBos).mllwMinusNavdM > -900.0) {
            const double bosOff = model.S(iBos).mllwMinusNavdM;
            auto oceanAtBoston = [&model, iBos, bosOff](double t) {
                return model.Height(static_cast<size_t>(iBos), t) + bosOff;
            };
            SweConfig bcfg;
            bcfg.spongeX0 = -2500.0f;   // Mass Bay, east of the outer harbor islands
            bcfg.westBoundary = false;  // the Charles is dammed; the west edge is a wall
            weather.AddDormantWindow("boston", &bathyBoston, bcfg, oceanAtBoston, 0.5);
            // M9ar: Boston lies inside the z14 height page; its solver reads slice 6 too.
            if (hgtTenant >= 0) {
                weather.SetHeightPage(resMgr.TextureRes(hgtTenant),
                                      resMgr.ResidencyRes(hgtTenant), 6u,
                                      resMgr.Mips(hgtTenant), surface.winH);
            }
        }
    }

    if (!opt.oceanProbe.empty()) {
        // nullopt = "lat,lon" did not parse: the block never returned, so the run goes on.
        if (const auto rc = tools::RunOceanProbe(opt, model, gpu, renderer, swe, resMgr,
                                                 simUnix, oceanAt, southAt, westAt, westQAt,
                                                 weather)) {
            return *rc;
        }
    }

    // M8: THE SOLVED WAVE FIELD (ALGEBRA.md wavefield) -- the stationary wave BVP
    // solved per cell over the inlet window on a worker thread, cached by content
    // identity (solver version + buckets + spectrum + height-stack signature), and
    // blended into the bank kernel inside its feathered window. NOAA carries the
    // state, the solve carries the structure, the GPU carries the phase.
    // The water scene is DATA (data/wave_scene.json, authored if absent, hot-reloaded
    // per frame): move the solved window, retune closures, save -- no recompile.
    sceneToWaveCfg = [&opt](const WaterSceneConfig& s) {
        WaveFieldConfig c;
        c.orgX = s.wfOrgX;
        c.orgZ = s.wfOrgZ;
        c.nx = s.wfNx;
        c.ny = s.wfNy;
        c.cellM = s.wfCellM;
        c.nComp = s.wfComps;
        c.spreadDeg = s.wfSpreadDeg;
        c.barNormalDeg = s.wfBarNormalDeg;
        c.gammaHs = s.wfGammaHs;
        c.minSamplesPerLambda = s.wfMinSamplesPerLambda;
        c.tideBucketM = s.wfTideBucketM;
        c.currentBucketMs = s.wfCurrentBucketMs;
        c.featherM = s.wfFeatherM;
        c.displayExag = s.wfExag;
        c.mapPath = opt.waveMap;   // the instrument, not the answer -- see WaveField.h
        return c;
    };
    // M9bc: THE WAVE FIELD AS A TREE NODE. The solver's grid is aligned to the z16 page
    // (WaveFieldSource::Align), the node paints planes as the frame's faces, the tree caches
    // them, and a page tenant serves them to the bank. A bucket roll re-keys the tree and the
    // window's whole pyramid is prefilled before the tenant is told.
    if (waterBank && hgtCh >= 0) {
        waveField = std::make_unique<WaveField>();
        wfCtSta = haveCurrents ? currents.StationIndex("ACT0816") : -1;
        WaveFieldConfig wcfg = sceneToWaveCfg(waterScene);
        waveFrame = WaveFieldSource::Align(wcfg);
        Log("[wave] grid aligned to the z16 page: cell %.3f m, %d x %d cells, window px "
            "(%lld, %lld), frame org (%lld, %lld)",
            wcfg.cellM, wcfg.nx, wcfg.ny, waveFrame.winPxX, waveFrame.winPxY,
            waveFrame.orgPxX, waveFrame.orgPxY);
        waveField->Configure(wcfg, &compositor, hgtCh,
                             &waterAtlas, &model, entSta,
                             haveCurrents ? &currents : nullptr, wfCtSta);
        // M8 flows into waves: the SWE's SOLVED current drives the dispersion when
        // resident (the bent jet, the tip shear); the ACT proxy is the fallback.
        if (swe.Ready() && sea) {
            waveField->SetSweCurrent(&swe, &bathy, sea->sweCurrentGain);
        }
        waterBank->SetWaveField(waterScene.wfEnabled ? waveField.get() : nullptr);
        waterBank->SetScene(&waterScene);
        if (waterBankB) {
            waterBankB->SetWaveField(waterScene.wfEnabled ? waveField.get() : nullptr);
            waterBankB->SetScene(&waterScene);
        }
        if (waterScene.wfEnabled) {
            waveSrc = std::make_shared<WaveFieldSource>(waveField.get(), waveFrame);
            waveTree = std::make_shared<std::shared_ptr<TileTree>>(
                std::make_shared<TileTree>(waveSrc.get(), TileTree::Fmt::Raw4));
            // M12 step 3e: THE DECLARATION. Slices 6.. are the component planes on the wave
            // frame's z16 lattice (slice 6 + plane), painted by the tree in the holder -- a
            // bucket roll prefills a fresh tree on a worker and swaps it in; the cube faces
            // are not this node's frame and answer zeros. Nothing here streams: a tile is the
            // solve's own bytes, repainted per bucket and dropped whole at the roll.
            hal::TenantDesc wd;
            wd.name = L"wave.field (pages)";
            wd.astNode = "wave.solver";
            wd.fiber = {DXGI_FORMAT_R8G8B8A8_UNORM, 128, 128,
                        "one component's a, k, cos, sin (quantized; per-comp aMax, kMax)"};
            wd.semantics = hal::Semantics::Field;
            wd.residence = hal::Residence::Volatile;
            wd.absence = hal::Absence::Zero;
            wd.absentTile.assign(65536, 0);
            wd.slices = 6u + uint32_t(WaveField::kMaxComp) + 1u;
            wd.bindings.push_back({6u, uint32_t(WaveField::kMaxComp) + 1u, waveFrame.color,
                                   nullptr, "a/k/phase-spinor planes"});
            wd.holder = waveTree;
            waveTenant = hal::Tenant::Sparse(gpu, resMgr, std::move(wd));
            waveT = waveTenant.Id();
            waterBank->SetWavePages(resMgr.TextureSrv(waveT), resMgr.ResidencySrv(waveT),
                                    double(waveFrame.orgPxX), double(waveFrame.orgPxY),
                                    waveFrame.nx, waveFrame.ny);
            if (waterBankB) {   // the same pages: one solve, every level's sea
                waterBankB->SetWavePages(resMgr.TextureSrv(waveT), resMgr.ResidencySrv(waveT),
                                         double(waveFrame.orgPxX), double(waveFrame.orgPxY),
                                         waveFrame.nx, waveFrame.ny);
            }
            Log("[wave] wave.field is page tenant %d: %u planes over the z16 window, tiles "
                "exact bytes of the solve, pyramid prefilled per bucket",
                waveT, uint32_t(WaveField::kMaxComp) + 1u);
        }
    }
    // M9bi: --sun pins the pre-ephemeris art direction; sunPlaced stays false and the
    // renderer keeps using these two constants, exactly as every earlier baseline did.
    if (opt.sunPinned) {
        renderer.sunAzimuthDeg = opt.sunAz;
        renderer.sunElevationDeg = opt.sunEl;
        Log("[sun] PINNED to azimuth %.1f, elevation %.1f -- the ephemeris is off",
            opt.sunAz, opt.sunEl);
    }
    if (globe) {
        globe->pixelWater = opt.pixelWater;   // M9bh: the two-ray water, per pixel
        globe->foamOpacity = waterScene.foamOpacity;
        globe->ringBlendTexels = waterScene.ringBlendTexels;
        globe->causticStrength = waterScene.causticStrength;
        globe->waterOptics = waterScene.waterOptics;
        // M8g THE ORIGIN PLANES: the edit-land geometry floor comes from the datum
        // envelope (MLLW + margin at the structure), not a tide-relative constant --
        // the old floor tracked the live waterline, which made the jetty unsinkable.
        float floorNavd = waterScene.jettyCrestNavd;
        if (floorNavd <= -90.0f) {
            floorNavd = 1.8f;
            if (waterAtlas.Ready()) {
                float elo = 0.0f, ehi = 0.0f;
                waterAtlas.EnvelopeNavd(42.8190, -70.8031, 0.0, &elo, &ehi);
                // Anchored to the TOP plane: a decayed structure is awash at spring
                // high but a continuous ridge below mid-tide. (lo + margin was the
                // first draft -- that floors at MLLW, which rescues the smear only
                // at dead low.) Surveyed crests taller than the floor still win.
                floorNavd = ehi - 0.45f;
                Log("[datum] envelope at north jetty: lo %+.2f hi %+.2f m NAVD "
                    "(synodic-month min/max) -> edit floor %+.2f",
                    elo, ehi, floorNavd);
            }
        }
        globe->editFloorNavd = floorNavd;
    }
    if (sea) {
        sea->windSeaFill = waterScene.windSeaFill;
        sea->bandFoldWeight = waterScene.bandFoldWeight;
        sea->buoyAssimAgeH = waterScene.buoyAssimAgeH;
        sea->buoyAssimGainMax = waterScene.buoyAssimGainMax;
    }

    // M8 THE FLEET: the AIS traffic lane (harvest_route.py) -- boats are pure
    // f(simUnix) on it (ping-pong at the ends), so scrubbing time scrubs the
    // traffic and headless renders are deterministic. No state anywhere.
    if (waterBank) route.Load("data/gis/route_merrimack.json");

    // M5c: give the solver history before the first frame, and run the validation cycle if
    // asked (headless CSV; the ebb/flood-asymmetry and basin-lag gates read from it).
    if (swe.Ready()) {
        if (opt.sweCycleH > 0) {
            return tools::RunSweCycleMode(opt, gpu, currents, haveCurrents, bathy, swe, resMgr,
                                          simUnix, oceanAt, southAt, westAt, westQAt);
        }
        if (opt.sweSpinupH > 0) {
            const auto t0 = std::chrono::steady_clock::now();
            swe.Spinup(gpu, simUnix, opt.sweSpinupH, oceanAt, westAt, southAt, westQAt);
            Log("[swe] spun up %.2f h of history in %.1f s", opt.sweSpinupH,
                std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count());
        }
        if (!opt.sweUvDump.empty()) tools::RunSweUv(opt, gpu, bathy, swe);
        {
            // Spot probes for the log: bar, throat, ocean. dEta pathologies show instantly.
            const float pts[6] = {900.0f, 100.0f, 250.0f, 60.0f, 2600.0f, 0.0f};
            SweSolver::Probe pr[3];
            swe.ReadProbes(gpu, pts, 3, pr);
            Log("[swe] probes  bar(900,100): dEta %+.3f u %+.2f,%+.2f v%d | throat(250,60): "
                "dEta %+.3f u %+.2f,%+.2f v%d | ocean(2600,0): dEta %+.3f",
                pr[0].dEta, pr[0].u, pr[0].v, pr[0].valid ? 1 : 0, pr[1].dEta, pr[1].u,
                pr[1].v, pr[1].valid ? 1 : 0, pr[2].dEta);
        }
    }
    timeScale = opt.timeScale;
    windowSec = opt.windowDays * 86400.0;

    // ---- Google-Earth camera gestures (PGA motors, core/Pga.h). LMB grabs the ground:
    // plain drag pans (the grabbed point stays under the cursor), SHIFT tilts and ALT
    // rotates -- each ONE motor rotation about a line through the ground pivot frozen at
    // the moment of grab. Wheel zooms toward the point under the cursor (CTRL+wheel keeps
    // the old fly-speed dial).

    groundAt = [&](double x, double z) -> double {
        if (mode == 1 && bathy.Ready() && !marsMode) {
            const float b = bathy.SampleWorld(static_cast<float>(x), static_cast<float>(z));
            if (b > -9000.0f) {
                return std::max(static_cast<double>(b), lastWaterNavd);
            }
        }
        if (mode == 1 && globe) {
            // M6g: beyond the CUDEM window the ground is the SPHERE (+ relief), in flat
            // coordinates: y = sqrt(R^2 - x^2 - z^2) - R (limb-clamped past the horizon).
            const double h2 = x * x + z * z;
            const double rr = planetR * planetR;
            const double sy =
                (h2 < rr * 0.9999) ? std::sqrt(rr - h2) - planetR : -planetR;
            double elev = 0.0;
            if (activeGlobe.Ready()) {
                const double py = sy + planetR;
                double p[3];
                for (int i = 0; i < 3; ++i) {
                    p[i] = oDir[i] * py + east0[i] * x + north0[i] * z;
                }
                const double pr = std::sqrt(p[0] * p[0] + p[1] * p[1] + p[2] * p[2]);
                const double r2d = 180.0 / 3.14159265358979;
                elev = (std::max)(0.0, activeGlobe.ElevAt(
                                           std::asin(std::clamp(p[1] / pr, -1.0, 1.0)) * r2d,
                                           std::atan2(p[2], p[0]) * r2d)) *
                       globe->reliefExagg;
            }
            return sy + elev;
        }
        return 0.0;   // chart mode: the ribbon's ground plane
    };
    // Unit ray through a client pixel, from the camera basis (shared by both pickers).
    pixelRay = [&](float sxPx, float syPx, double d[3]) {
        const double W = std::max(1u, window.Width()), H = std::max(1u, window.Height());
        const double th = std::tan(cam.fovY * 0.5);
        const double rx = (2.0 * sxPx / W - 1.0) * th * (W / H);
        const double ry = (1.0 - 2.0 * syPx / H) * th;
        const DirectX::XMFLOAT3 ff = cam.Forward();
        const DirectX::XMFLOAT3 rr = cam.Right();
        const double f[3] = {ff.x, ff.y, ff.z}, r[3] = {rr.x, rr.y, rr.z};
        const double u[3] = {f[1] * r[2] - f[2] * r[1], f[2] * r[0] - f[0] * r[2],
                             f[0] * r[1] - f[1] * r[0]};   // f x r = camera up
        d[0] = f[0] + r[0] * rx + u[0] * ry;
        d[1] = f[1] + r[1] * rx + u[1] * ry;
        d[2] = f[2] + r[2] * rx + u[2] * ry;
        const double dl = std::sqrt(d[0] * d[0] + d[1] * d[1] + d[2] * d[2]);
        d[0] /= dl; d[1] /= dl; d[2] /= dl;
    };
    // Ray through a client pixel, marched against the ground field; hits within 30 km.
    pickGround = [&](float sxPx, float syPx, double out[3]) -> bool {
        double d[3];
        pixelRay(sxPx, syPx, d);

        double t = 0, prevT = 0;
        bool hit = false;
        for (int i = 0; i < 500 && t < 30000.0; ++i) {
            prevT = t;
            t += std::max(2.0, t * 0.02);
            if (cam.py + d[1] * t <= groundAt(cam.px + d[0] * t, cam.pz + d[2] * t)) {
                double lo = prevT, hi = t;   // bisect the crossing
                for (int j = 0; j < 18; ++j) {
                    const double mid = 0.5 * (lo + hi);
                    if (cam.py + d[1] * mid <=
                        groundAt(cam.px + d[0] * mid, cam.pz + d[2] * mid)) hi = mid;
                    else lo = mid;
                }
                t = hi;
                hit = true;
                break;
            }
        }
        if (!hit) {   // above-horizon fallback: the flat plane at the local ground level
            const double g = groundAt(cam.px, cam.pz);
            if (d[1] >= -1e-4) return false;
            t = (g - cam.py) / d[1];
            if (t <= 0.0 || t > 30000.0) return false;
        }
        out[0] = cam.px + d[0] * t;
        out[1] = cam.py + d[1] * t;
        out[2] = cam.pz + d[2] * t;
        return true;
    };
    // Globe mode: analytic ray-sphere at sea level (5 km texels make terrain-precise
    // picking pointless from orbit; the pivot is for orbiting, not surveying).
    pickGlobe = [&](float sxPx, float syPx, double out[3]) -> bool {
        // M6g: the sphere lives in the ONE flat frame, centred at (0, -R, 0).
        double d[3];
        pixelRay(sxPx, syPx, d);
        const double oy = cam.py + planetR;
        const double b = cam.px * d[0] + oy * d[1] + cam.pz * d[2];
        const double c = cam.px * cam.px + oy * oy + cam.pz * cam.pz - planetR * planetR;
        const double disc = b * b - c;
        if (disc < 0.0) return false;
        const double t = -b - std::sqrt(disc);
        if (t <= 0.0) return false;
        out[0] = cam.px + d[0] * t;
        out[1] = cam.py + d[1] * t;
        out[2] = cam.pz + d[2] * t;
        return true;
    };
    pickAny = [&](float sx, float sy, double out[3]) {
        return (mode == 1 && altOf(cam) > 6000.0) ? pickGlobe(sx, sy, out)
                                                  : pickGround(sx, sy, out);
    };

    if (!opt.rail.empty()) {
        CreateDirectoryW(opt.rail.c_str(), nullptr);
        if (railKeys.empty()) {
            Log("FATAL: --rail needs globe + sea + bathy data all present");
            return 1;
        }
    }

    if (opt.warmInlet && (winTenant >= 0 || hgtTenant >= 0)) {
        tools::RunWarmInlet(opt, gpu, compositor, resMgr, winTenant, hgtTenant, hgtWinTenant);
    }

    // ---- M6j: channel export mode -- pull data OUT through the manager and exit.
    if (!opt.exportSpec.empty()) {
        return tools::RunExport(opt, gpu, compositor, hgtCh, resMgr, colCh, surface);
    }

    // ---- The instrumentation block's runtime part; its members and their notes are
    // declared in FrameLoop.h (main.cpp ff2f732 lines 1307..1389).
    last = Clock::now();
    lastTitle = last;
    kPredictEvery = (std::max)(1u, opt.predictEvery);
    if (!opt.mp4.empty()) {
        recPipe.Open(opt.mp4, renderer.Width(), renderer.Height(), 30);
    }
    if (!opt.rail.empty()) {
        railMs.reserve(opt.frames ? opt.frames : 1200);
        railLoopMs.reserve(opt.frames ? opt.frames : 1200);
        railPool.reserve(opt.frames ? opt.frames : 1200);
    }

    // ONE step path, called from BOTH clock branches. The windowed clock advances in whole
    // quanta off SimClock and the headless clock is frame-indexed, but a boat that
    // integrated differently between them could not be gated by any rail -- and the rails
    // are the only reproducible instrument this engine has. So the quanta COUNT differs and
    // nothing else does.
    stepBoat = [&](int quanta) {
        if (!boat) return;
        boatSea.Configure(&weather, waveField.get(), sea ? &sea->Ocean() : nullptr,
                          &seaState, sea ? double(sea->heightScale) : 1.0,
                          waterScene.wfExag, waterScene.wfChop);
        // PLACE THE HULL ON THE WATER, ONCE. A vessel is built before the weather
        // manager exists, so it cannot be spawned at the right height -- and NAVD 0 is half
        // a metre under the surface here at this tide. Dropped in submerged, the hull takes
        // a buoyancy impulse of its own displacement in one quantum and the RHIB simply
        // capsized: heel 179 deg, floating inverted, never recovering. So the first step
        // sets it down gently instead of the scene throwing it in.
        if (!boatPlaced) {
            double bp0[3] = {0, 0, 0};
            boat->Body().pose.TransformPoint(bp0[0], bp0[1], bp0[2]);
            const SurfaceSample ss = boatSea.At(bp0[0], bp0[2], simUnix);
            if (ss.valid) {
                // Put the KEEL at its static draught, not the CG on the waterline. The CG
                // sits well above the keel, so placing it at the surface immersed the hull
                // half a metre deeper than it floats and it came up like a cork. The keel
                // depth comes from the spec's own stations, after Build re-referenced them
                // onto the CG, so it is whatever this hull actually is.
                double keel = 0.0;
                for (const Element& el : boat->Spec().elements) {
                    for (const Section& st : el.stations) {
                        for (double y : st.oy) keel = (std::min)(keel, y);
                    }
                }
                const double draft = (boat->Spec().draftStatic.v > 0.0)
                                         ? boat->Spec().draftStatic.v : -keel;
                const double y0 = ss.heightNavd - keel - draft;
                boat->Body().SetPose(Motor::Translation(bp0[0], y0, bp0[2]));
                boatPlaced = true;
                Log("[vessel] set down: surface %+.3f, keel %.3f below CG, draught %.2f "
                    "-> CG at %+.3f m NAVD", ss.heightNavd, -keel, draft, y0);
            } else {
                return;   // no water yet: do not integrate a hull that has nothing to float on
            }
        }
        if (opt.boatDrive) {
            for (int t = 0; t < VesselControls::kMaxThrusters; ++t) {
                boatCtl.throttle[t] = opt.boatThrottle;
            }
            boatCtl.steer = opt.boatSteer;
        }
        // THE HULL STEPS AT 60 Hz, NOT 240. SimClock's quantum is 240 Hz because that is
        // what the SCENE needed; nothing in a hull's dynamics asks for 4 ms resolution. Its
        // fastest modes are heave at ~1 s, roll at ~1.8 s, and the collar's slam at ~10 Hz
        // -- 60 Hz resolves the quickest of those by six to one.
        //
        // It matters because a step is not cheap: ~73 water queries for this hull, and each
        // one sums the solved field's 32 components and the retained cascade bins. At 240 Hz
        // that measured 26 ms a frame, which is more than the entire renderer costs.
        //
        // Determinism is untouched -- this is still a FIXED step off the same clock, just a
        // coarser multiple of it, so the boat is as frame-rate independent as before. The
        // leftover quanta are carried, never dropped, so no owed time is lost.
        constexpr int kPerStep = 4;                     // 240 / 4 = 60 Hz
        quantaOwed += quanta;
        const int steps = quantaOwed / kPerStep;
        quantaOwed -= steps * kPerStep;
        PROF_BEGIN();
        for (int q = 0; q < steps; ++q) {
            boat->Step(boatSea, boatCtl, simUnix, SimClock::kDt * kPerStep);
        }
        if (!boat->Body().Sane()) boatCtl = VesselControls{};
        PROF_END(11);

        const Vessel* vs[1] = {boat.get()};
        if (vesselLayer) vesselLayer->SetVessels(vs, 1);

        // One telemetry line a second. Cheap, and it is the only way to tell a hull that is
        // floating wrong from one that is not being DRAWN.
        if ((telTick++ % 60) == 0) {
            const VesselTelemetry& t = boat->Telemetry();
            double bp[3] = {0, 0, 0};
            boat->Body().pose.TransformPoint(bp[0], bp[1], bp[2]);
            // The water the hull is standing on, at the hull. Buoyancy acts along this
            // NORMAL, so a wrong slope is not a cosmetic error -- it is a horizontal force.
            const SurfaceSample ws = boatSea.At(bp[0], bp[2], simUnix);
            const double slope = std::sqrt(ws.nx * ws.nx + ws.nz * ws.nz) /
                                 ((std::abs(ws.ny) > 1e-9) ? std::abs(ws.ny) : 1e-9);
            Log("[vessel]   water: eta %+.2f n (%+.3f, %+.3f, %+.3f) |slope| %.3f = %.1f deg"
                "  orbital (%+.2f, %+.2f, %+.2f) m/s",
                ws.heightNavd, ws.nx, ws.ny, ws.nz, slope,
                std::atan(slope) * 57.2957795, ws.vx, ws.vy, ws.vz);
            Log("[vessel] pos (%.1f, %+.2f, %.1f) %.1f kn hdg %.0f heel %+.1f trim %+.1f "
                "draught %.3f vol %.2f (hull %.2f collar %.2f) lam %.2f cop %+.2f "
                "depth %.1f%s%s | parts %u",
                bp[0], bp[1], bp[2], t.speedKn, t.headingRad * 57.2957795,
                t.heelRad * 57.2957795, t.trimRad * 57.2957795, t.draughtM, t.immersedVol,
                t.hullVol, t.collarVol, t.wettedLambda, t.copZ, t.depthM,
                (t.immersedVol < 1e-6) ? " AIRBORNE" : (t.aground ? " AGROUND" : ""),
                t.waterValid ? "" : " NO-WATER",
                vesselLayer ? vesselLayer->PartCount() : 0u);
        }

        // ---- THE CHASE CAMERA. Gravity-up and roll-free by choice: a camera that heels
        // with the hull reads as the WORLD rolling, which is nauseating and is not what a
        // helmsman's inner ear reports. The motor-native view that DOES heel is the
        // first-person one, later.
        if (helming) {
            const RigidBody& b = boat->Body();
            double p[3] = {0, 0, 0};
            b.pose.TransformPoint(p[0], p[1], p[2]);
            double f[3] = {0, 0, 1};
            b.pose.TransformDir(f[0], f[1], f[2]);
            const double fl = std::sqrt(f[0] * f[0] + f[2] * f[2]);
            if (fl > 1e-6) { f[0] /= fl; f[2] /= fl; }
            // The look-steer's reference is THIS camera's heading, kept here rather than
            // read off `cam`, so detaching the view (T, or a spectator flying to orbit in a
            // future multiplayer) cannot feed the assist a heading from the other side of
            // the planet.
            helmYawRef = std::atan2(f[2], f[0]);
            const double back = 15.0, up = 5.0;
            cam.px = p[0] - f[0] * back;
            cam.py = p[1] + up;
            cam.pz = p[2] - f[2] * back;
            cam.LookAt(p[0], p[1] + 0.6, p[2]);
        }
    };
    return std::nullopt;
}

bool FrameLoop::Frame() {
    const Options& opt = m_opt;
    auto& model = m_A.model;
    auto& window = m_A.window;
    auto& gpu = m_A.gpu;
    auto& renderer = m_A.renderer;
    auto& sky = m_A.sky;
    auto& tide = m_A.tide;
    auto& seaState = m_A.seaState;
    auto& sea = m_A.sea;
    auto& currents = m_A.currents;
    auto& haveCurrents = m_A.haveCurrents;
    auto& datumOff = m_A.datumOff;
    auto& marsMode = m_A.marsMode;
    auto& compositor = m_A.compositor;
    auto& hgtCh = m_A.hgtCh;
    auto& waterAtlas = m_A.waterAtlas;
    auto& bathy = m_A.bathy;
    auto& terrain = m_A.terrain;
    auto& swe = m_A.swe;
    auto& gulf = m_A.gulf;
    auto& waterScene = m_A.waterScene;
    auto& waterSceneMtime = m_A.waterSceneMtime;
    auto& kScenePath = m_A.kScenePath;
    auto& sceneWatch = m_A.sceneWatch;
    auto& waterBank = m_A.waterBank;
    auto& waterBankB = m_A.waterBankB;
    auto& globe = m_A.globe;
    auto& planetR = m_A.planetR;
    auto& resMgr = m_A.resMgr;
    auto& gisLayer = m_A.gisLayer;
    // M12 step 4a: the z14 page origin, read off the surface's height window -- the doubles
    // the bank, the trace and the exposure's uv closure below take.
    const double winOrgX = static_cast<double>(m_A.surface.winH.orgPxX);
    const double winOrgY = static_cast<double>(m_A.surface.winH.orgPxY);
    auto& hgtTenant = m_A.hgtTenant;
    auto& hgtWinTenant = m_A.hgtWinTenant;
    auto& exposureSrc = m_A.exposureSrc;
    auto& exposureRoot = m_A.exposureRoot;
    auto& exposureTree = m_A.exposureTree;
    auto& exposureT = m_A.exposureT;
    auto& exposureTenant = m_A.exposureTenant;
    auto& mode = m_mode;
    auto& applyMode = m_applyMode;
    auto& cam = m_cam;
    auto& camSea = m_camSea;
    auto& camChart = m_camChart;
    // M12 step 4a: the tangent frame's rows are the surface's (compose/SurfaceFrame.h).
    auto& oDir = m_A.surface.up;
    auto& east0 = m_A.surface.east;
    auto& north0 = m_A.surface.north;
    auto& portal = m_portal;
    auto& drosteLeaf = m_drosteLeaf;   // M12 step 4d: the Droste tower as a Space::Cycle
    auto& camLevel = m_camLevel;
    auto& altOf = m_altOf;
    auto& poseMotor = m_poseMotor;
    auto& motorPose = m_motorPose;
    auto& railKeys = m_railKeys;
    auto& railPose = m_railPose;
    auto& drosteRailPose = m_drosteRailPose;
    auto& simUnix = m_simUnix;
    auto& simClock = m_simClock;
    auto& boat = m_boat;
    auto& boatCtl = m_boatCtl;
    auto& wavePending = m_wavePending;
    auto& wavePrefillDone = m_wavePrefillDone;
    auto& wavePrefillBusy = m_wavePrefillBusy;
    auto& wavePendingKey = m_wavePendingKey;
    auto& wavePendingTiles = m_wavePendingTiles;
    auto& wavePendingPlanes = m_wavePendingPlanes;
    auto& wavePendingSec = m_wavePendingSec;
    auto& boatPlaced = m_boatPlaced;
    auto& helming = m_helming;
    auto& sunLogged = m_sunLogged;
    auto& startUnix = m_startUnix;
    auto& entSta = m_entSta;
    auto& oceanAt = m_oceanAt;
    auto& southAt = m_southAt;
    auto& westAt = m_westAt;
    auto& westQAt = m_westQAt;
    auto& weather = m_weather;
    auto& sceneToWaveCfg = m_sceneToWaveCfg;
    auto& waveField = m_waveField;
    auto& wfCtSta = m_wfCtSta;
    auto& waveSrc = m_waveSrc;
    auto& waveTree = m_waveTree;
    auto& waveFrame = m_waveFrame;
    auto& waveT = m_waveT;
    auto& route = m_route;
    auto& timeScale = m_timeScale;
    auto& windowSec = m_windowSec;
    auto& paused = m_paused;
    auto& dragMode = m_dragMode;
    auto& lmbWas = m_lmbWas;
    auto& dragPivot = m_dragPivot;
    auto& lastWaterNavd = m_lastWaterNavd;
    auto& groundAt = m_groundAt;
    auto& pickGround = m_pickGround;
    auto& pickGlobe = m_pickGlobe;
    auto& pickAny = m_pickAny;
    auto& last = m_last;
    auto& lastTitle = m_lastTitle;
    auto& frame = m_frame;
    auto& pixArmed = m_pixArmed;
    auto& dumpedSolid = m_dumpedSolid;
    auto& frameMsSum = m_frameMsSum;
    auto& frameMsN = m_frameMsN;
    auto& railMs = m_railMs;
    auto& railLoopMs = m_railLoopMs;
    auto& kPredictEvery = m_kPredictEvery;
    auto& profMs = m_profMs;
    auto& profHelmMs = m_profHelmMs;
    auto& railPreMs = m_railPreMs;
    auto& walkNodesAcc = m_walkNodesAcc;
    auto& walkLeavesAcc = m_walkLeavesAcc;
    auto& walkWantNsAcc = m_walkWantNsAcc;
    auto& walkFrames = m_walkFrames;
    auto& morphFullAcc = m_morphFullAcc;
    auto& morphPartAcc = m_morphPartAcc;
    auto& wantTouchAcc = m_wantTouchAcc;
    auto& wantHitAcc = m_wantHitAcc;
    auto& profT0 = m_profT0;
    auto& profHelm = m_profHelm;
    auto& profOn = m_profOn;
    auto& profInMs = m_profInMs;
    auto& profInHelmMs = m_profInHelmMs;
    auto& railInMs = m_railInMs;
    auto& loopRowOpen = m_loopRowOpen;
    auto& settling = m_settling;
    auto& settleFrames = m_settleFrames;
    auto& settleQuiet = m_settleQuiet;
    auto& settlePending0 = m_settlePending0;
    auto& settleReads0 = m_settleReads0;
    auto& settleExactQuiet = m_settleExactQuiet;
    auto& recPipe = m_recPipe;
    auto& recPixels = m_recPixels;
    auto& railPool = m_railPool;
    auto& stepBoat = m_stepBoat;
    auto& loggedLevel = m_loggedLevel;

    if (!opt.headless) {
        window.NewFrame();
        if (!window.PumpMessages()) return false;
        if (window.TakeResized()) renderer.OnResize(window.Width(), window.Height());
    }

    int simSteps = 0;   // whole sim quanta this frame (sim/SimClock.h)
    // M10: the roll reference the Droste rail writes (its pose is a similarity, roll
    // and all); without a rail the gravity blend below supplies it.
    double drosteUp[3] = {0.0, 1.0, 0.0};
    bool drosteRailUp = false;
    const auto now = Clock::now();
    float dt = std::chrono::duration<float>(now - last).count();
    last = now;
    const auto preT0 = Clock::now();   // M9u: everything before RenderFrame
    // The interval just measured is the previous frame's whole loop (its render, its
    // capture, this iteration's message pump): close that frame's row with it. It used
    // to be pushed beside the CURRENT frame's render time, so every metrics.csv row
    // paired frame N's render with frame N-1's loop and the 61-frame comb sat one row
    // late. Unclamped: a stall is a stall in a series.
    if (loopRowOpen) {
        railLoopMs.push_back(dt * 1000.0f);
        loopRowOpen = false;
    }
    profOn = opt.rail.empty() || frame >= 150u;
    dt = (dt > 0.25f) ? 0.25f : dt;   // a debugger break must not teleport time

    if (!opt.headless) {
        InputState in = window.Input();   // by value: gestures may consume wheel

        if (mode != 2) {   // the gulf map keeps its plain controls
            if (in.lmb && !lmbWas) {
                // Modifier at the moment of grab decides the gesture (GE behaviour).
                // Tilt/rotate pivot on the ground at screen centre; pan under cursor.
                const bool alt = in.keyDown[VK_MENU], shift = in.keyDown[VK_SHIFT];
                const float cx = (alt || shift) ? window.Width() * 0.5f : in.mouseX;
                const float cy = (alt || shift) ? window.Height() * 0.5f : in.mouseY;
                dragMode = pickAny(cx, cy, dragPivot) ? (alt ? 3 : shift ? 2 : 1) : 0;
            }
            if (!in.lmb) dragMode = 0;
            lmbWas = in.lmb;

            // M6g: gesture flavour follows ALTITUDE, continuously in one frame -- no
            // mode split. High = orbital (radial verticals, planet-centre grab);
            // low = ground (terrain pivots). The 6 km threshold only picks WHICH line
            // a motor rotates about; the world never changes under the camera.
            const bool orbital = (mode == 1) && altOf(cam) > 6000.0;
            if (dragMode != 0 && (in.mouseDx != 0.0f || in.mouseDy != 0.0f)) {
                constexpr double kOrbitRate = 0.006;   // rad per pixel
                const double minAlt =
                    orbital ? -1.0e30 : groundAt(cam.px, cam.pz) + 1.2;
                if (dragMode == 3) {          // ALT: rotate about the local vertical
                    double up[3] = {0, 1, 0};
                    if (orbital) {            // ...which on a planet is the radial line
                        up[0] = dragPivot[0];
                        up[1] = dragPivot[1] + planetR;   // sphere centre (0,-R,0)
                        up[2] = dragPivot[2];
                        const double pr = std::sqrt(up[0] * up[0] + up[1] * up[1] +
                                                    up[2] * up[2]);
                        up[0] /= pr; up[1] /= pr; up[2] /= pr;
                    }
                    cam.OrbitAboutLine(dragPivot, up, in.mouseDx * kOrbitRate, minAlt);
                } else if (dragMode == 2) {   // SHIFT: tilt about the horizontal line
                    const DirectX::XMFLOAT3 r = cam.Right();
                    const double ax[3] = {r.x, 0.0, r.z};
                    cam.OrbitAboutLine(dragPivot, ax, -in.mouseDy * kOrbitRate, minAlt);
                } else if (orbital) {         // grab the GLOBE: one motor about the
                                              // planet-centre line takes now -> grabbed
                    double nowPt[3];
                    if (pickGlobe(in.mouseX, in.mouseY, nowPt)) {
                        const double R = planetR;
                        double a[3] = {nowPt[0] / R, (nowPt[1] + R) / R, nowPt[2] / R};
                        double b[3] = {dragPivot[0] / R, (dragPivot[1] + R) / R,
                                       dragPivot[2] / R};
                        double ax[3] = {a[1] * b[2] - a[2] * b[1],
                                        a[2] * b[0] - a[0] * b[2],
                                        a[0] * b[1] - a[1] * b[0]};
                        const double s = std::sqrt(ax[0] * ax[0] + ax[1] * ax[1] +
                                                   ax[2] * ax[2]);
                        const double cdot = a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
                        if (s > 1e-9) {
                            const double ctr[3] = {0, -planetR, 0};
                            cam.OrbitAboutLine(ctr, ax,
                                               std::atan2(s, cdot), -1.0e30);
                        }
                    }
                } else {                      // grab-pan: the point stays under the cursor
                    double nowPt[3];
                    if (pickGround(in.mouseX, in.mouseY, nowPt)) {
                        cam.px += dragPivot[0] - nowPt[0];
                        cam.pz += dragPivot[2] - nowPt[2];
                    }
                }
            }
            if (in.wheel != 0.0f && !in.keyDown[VK_CONTROL]) {
                double tgt[3];
                if (pickAny(in.mouseX, in.mouseY, tgt)) {
                    cam.DollyToward(tgt, in.wheel * 0.18, orbital ? 2500.0 : 3.0);
                    in.wheel = 0;   // consumed; Update() keeps its speed dial off it
                }
            }
        }

        // While helming, the free-fly integration does not run at all: Camera::Update
        // owns W A S D Q E Shift Ctrl, which is a head-on collision with the binnacle.
        // A control MODE is the only honest fix -- rebinding would just move the clash.
        if (!helming) cam.Update(in, dt);
        if (in.keyPressed[VK_F5]) {
            // SAVE THIS CAMERA. The five numbers SetFromCompass takes, inverted from
            // the live pose, appended to data/views.json under a generated name (rename
            // it in the file) and logged as the flags that reproduce it directly.
            float az = 90.0f - cam.yaw * 180.0f / 3.14159265f;
            az = std::fmod(std::fmod(az, 360.0f) + 360.0f, 360.0f);
            const float pitchDeg = cam.pitch * 180.0f / 3.14159265f;
            std::vector<std::string> kept;
            {
                std::ifstream vf("data/views.json", std::ios::binary);
                const std::string text((std::istreambuf_iterator<char>(vf)),
                                       std::istreambuf_iterator<char>());
                std::string err;
                const JsonValue root = JsonParser::Parse(text, &err);
                const JsonValue* views = err.empty() ? root.Get("views") : nullptr;
                if (views) {
                    for (const JsonValue& v : views->arr) {
                        char line[320];
                        snprintf(line, sizeof(line),
                                 "  {\"name\": \"%s\", \"x\": %.2f, \"alt\": %.2f, "
                                 "\"z\": %.2f, \"az\": %.2f, \"pitch\": %.2f, "
                                 "\"time\": %.0f}",
                                 v.Str("name").c_str(), v.Num("x", 0), v.Num("alt", 0),
                                 v.Num("z", 0), v.Num("az", 0), v.Num("pitch", 0),
                                 v.Num("time", 0));
                        kept.push_back(line);
                    }
                }
            }
            char name[32];
            snprintf(name, sizeof(name), "view-%zu", kept.size() + 1);
            char line[320];
            snprintf(line, sizeof(line),
                     "  {\"name\": \"%s\", \"x\": %.2f, \"alt\": %.2f, \"z\": %.2f, "
                     "\"az\": %.2f, \"pitch\": %.2f, \"time\": %.0f}",
                     name, cam.px, cam.py, cam.pz, az, pitchDeg, simUnix);
            kept.push_back(line);
            std::filesystem::create_directories("data");
            std::ofstream vf("data/views.json", std::ios::binary);
            vf << "{\"views\": [\n";
            for (size_t k = 0; k < kept.size(); ++k) {
                vf << kept[k] << (k + 1 < kept.size() ? ",\n" : "\n");
            }
            vf << "]}\n";
            Log("[view] %s %s: --view %s   (= --cam %.2f,%.2f,%.2f --campos %.2f,%.2f)",
                vf ? "saved" : "FAILED TO WRITE data/views.json for", name, name,
                cam.py, az, pitchDeg, cam.px, cam.pz);
        }
        if (in.keyPressed['R']) renderer.ReloadShaders();
        if (in.keyPressed[VK_SPACE]) paused = !paused;
        if (in.keyPressed[VK_UP]) timeScale = std::min(timeScale * 10.0, 864000.0);
        if (in.keyPressed[VK_DOWN]) timeScale = std::max(timeScale / 10.0, 1.0);
        const double nudge = in.keyDown[VK_SHIFT] ? 86400.0 : 3600.0;
        if (in.keyPressed[VK_LEFT]) simUnix -= nudge;
        if (in.keyPressed[VK_RIGHT]) simUnix += nudge;
        if (in.keyPressed[VK_HOME] || in.keyPressed['N']) simUnix = NowUnix();
        // A deliberate jump re-bases the clock; the partial quantum does not smear
        // across the discontinuity.
        if (in.keyPressed[VK_LEFT] || in.keyPressed[VK_RIGHT] ||
            in.keyPressed[VK_HOME] || in.keyPressed['N']) {
            simClock.SetTo(simUnix);
            // THE CLOCK POLICY, which a stateful body needs and nothing else in this
            // engine does. Everything else here is f(simUnix) and simply re-evaluates;
            // a hull carries momentum, so jumping an hour teleports the sea out from
            // under it while it keeps the velocity it had. It came back as the boat
            // being flung. A deliberate jump resets it to rest at its last pose.
            if (boat) {
                boat->Body().Rest();
                boatCtl = VesselControls{};
                boatPlaced = false;   // re-seat it on the new instant's surface
                Log("[vessel] time jumped -- hull reset to rest (a boat cannot be "
                    "integrated across a scrub)");
            }
        }
        // A boat at 100x time is not a simulation of anything: the tide and current
        // buckets roll every few frames, and each roll costs an 8-11 s wave solve plus
        // a 12-19 s page prefill, which is what reads as a freeze. Time scaling stays
        // available with no hull aboard.
        if (boat && timeScale > 10.0) {
            timeScale = 10.0;
            Log("[vessel] time scale held at 10x while a hull is aboard -- faster than "
                "that spends every frame re-solving the wave field, not sailing");
        }
        if (in.keyPressed[VK_OEM_4]) windowSec = std::max(windowSec * 0.5, 0.5 * 86400.0);
        if (in.keyPressed[VK_OEM_6]) windowSec = std::min(windowSec * 2.0, 30.0 * 86400.0);
        if (in.keyPressed['C']) {   // cycle the ribbon's contour gauge
            tide->contourStepM = (tide->contourStepM > 0.4f) ? 0.25f
                               : (tide->contourStepM > 0.2f) ? 0.0f : 0.5f;
        }
        if (in.keyPressed['G']) {
            // M9b: THE GEOMETRY QUESTION, answered by the rasterizer. Shading sells
            // amplitude the geometry may not actually have (priors 8), so the honest
            // check is to stop filling the triangles. G cycles shipped -> wireframe
            // -> meshlet tint. Both surfaces flip together: in one-water mode the
            // globe mesh IS the sea, and off it the SeaLayer grid is what draws.
            const int mode = globe ? (globe->surfaceDebug + 1) % 3
                                   : ((sea && !sea->wireframe) ? 1 : 0);
            if (globe) globe->surfaceDebug = mode;
            if (sea) sea->wireframe = (mode == 1);
        }
        if (in.keyPressed['V']) {
            if (mode == 1 && globe && altOf(cam) > 6000.0) {
                globe->windOverlay = !globe->windOverlay;
            } else if (sea) {
                sea->atlasVisualize = !sea->atlasVisualize;
            }
        }
        if (in.keyPressed[VK_TAB] && (sea || gulf || globe)) {
            int next = mode;
            do {
                next = (next + 1) % 3;
            } while ((next == 1 && !(sea || globe)) || (next == 2 && !gulf));
            if (mode == 0) camChart = cam;
            else if (mode == 1) camSea = cam;
            mode = next;
            applyMode(mode);
            if (mode == 0) cam = camChart;
            else if (mode == 1) cam = camSea;
        }
        // THE SCENE CLOCK ADVANCES IN WHOLE QUANTA. It used to be `simUnix += dt *
        // timeScale` with dt straight off the wall clock, so how much world a frame
        // covered was a function of how fast that frame rendered -- and everything that
        // integrates inherited it. SweSolver already chases this clock with its own
        // fixed 0.25 s substeps, capped at kMaxSubsteps a frame, so a lurching clock is
        // what leaves it behind by a frame-rate-dependent amount (PERF_EXPERIMENT step
        // 26). `simSteps` is the count an integrating consumer steps: the boat's
        // `for (int i = 0; i < simSteps; ++i) Step(SimClock::kDt)` hangs here.
        if (!paused) {
            simSteps = simClock.Advance(dt, timeScale);
            simUnix = simClock.Now();

            // ---- THE HELM. Reading the controls is the only part of the boat that
            // is windowed-only; the STEP itself is shared with the headless path below,
            // because a boat that integrates differently in a rail than in a window is
            // a boat no rail can gate.
            if (boat) {
                if (in.keyPressed['T']) {
                    helming = !helming;
                    Log("[vessel] %s", helming ? "helm" : "camera detached (the boat "
                                                         "keeps sailing)");
                }
                if (helming) {
                    // Keyboard for now; the analog triggers are the twin-lever binnacle
                    // and land with XInput. W/S drive BOTH levers, Q/E split them, which
                    // is what makes a pivot a squeeze rather than a mode.
                    const double rate = dt * 1.5;
                    double demand = 0.0;
                    if (in.keyDown['W']) demand += 1.0;
                    if (in.keyDown['S']) demand -= 1.0;
                    double split = 0.0;
                    if (in.keyDown['E']) split += 1.0;
                    if (in.keyDown['Q']) split -= 1.0;
                    for (int t = 0; t < VesselControls::kMaxThrusters; ++t) {
                        const double want =
                            std::clamp(demand + ((t % 2 == 0) ? -split : split),
                                       -1.0, 1.0);
                        boatCtl.throttle[t] +=
                            std::clamp(want - boatCtl.throttle[t], -rate, rate);
                    }
                    // D IS STARBOARD, and the sign is the outboard's, not the
                    // wheel's. Motor::Rotation about +y takes +z to +x (RunPgaSelfTest
                    // pins it), so a POSITIVE steer swings the thrust to starboard --
                    // and that thrust acts at the transom, ABAFT the CG, so it pushes
                    // the stern to starboard and the bow to PORT. A helm that turns the
                    // boat to starboard therefore commands a NEGATIVE angle here, which
                    // is exactly what the real linkage does: the leg kicks the stern the
                    // opposite way to the turn.
                    double sd = 0.0;
                    if (in.keyDown['D']) sd -= 1.0;
                    if (in.keyDown['A']) sd += 1.0;
                    // A mechanical steering RATE limit, not a snap: the outboards swing
                    // at a finite speed and that lag is a real part of how a boat feels.
                    const double sMax = 0.6;
                    boatCtl.steer += std::clamp(sd * sMax - boatCtl.steer,
                                                -dt * 1.2, dt * 1.2);
                    boatCtl.steer = std::clamp(boatCtl.steer, -sMax, sMax);

                    // TRIM. Shift trims OUT (bow up), Ctrl trims IN (bow down). It is
                    // slow on purpose -- a trim pump takes seconds to sweep its range --
                    // and it is the helmsman's only direct hold on running attitude.
                    double td = 0.0;
                    if (in.keyDown[VK_SHIFT]) td += 1.0;
                    if (in.keyDown[VK_CONTROL]) td -= 1.0;
                    boatCtl.tilt = std::clamp(boatCtl.tilt + td * dt * 0.20,
                                              -0.0873, 0.2618);
                }
                stepBoat(simSteps);
            }
        }
    } else {
        // Deterministic time in headless mode so a dump sequence is reproducible.
        // M7h: rail SETTLE -- the first recorded frame used to be the coldest:
        // the height window still streaming, the classifier reading coarse
        // fallback and calling half the channel LAND (the flat grey panels at
        // t=0). The rail now holds its opening pose for kRailSettle unrecorded
        // frames so residency, the solver mirror, and the composed caches are
        // warm before the camera rolls.
        const uint32_t settle = opt.rail.empty() ? 0u : 150u;
        uint32_t recFrame = (frame > settle) ? frame - settle : 0u;
        // Step 28: the landing ledger prints on every turn of the --res-trace-frames
        // window, labelled with the recorded frame (the mp4's index). It does not
        // ride --res-trace: that flag's per-frame slot audit slows the loop, and a
        // slower loop is a different landing schedule -- the very thing the ledger
        // is there to watch at the shipped timing.
        resMgr.traceTurn = frame > settle && recFrame >= opt.traceFrom &&
                           recFrame <= opt.traceTo;
        resMgr.traceRecFrame = recFrame;
        // --dump-both renders ONE extra frame in wireframe. Hold the sim clock at
        // the solid frame's instant so the two images are the same water, not the
        // same water 33 ms later -- otherwise the lines do not sit on the crests
        // they are supposed to explain.
        // --settle-sync / --settle-hold hold the same instant for as many frames as the
        // residency manager needs to drain, or for a counted N (the exit test below
        // decides which). The held instant is opt.frames - 1, the LAST frame an unheld
        // --frames N run renders and dumps: an off-by-one here would put the held still
        // a whole 1/30 s of sea away from the unheld one, so it was MEASURED rather than
        // read (helm_ebb, 2026-09-05, out/step21/ob_*): --frames 240 --settle-hold 1
        // against an unheld --frames 241 is 54.76 % identical, 33.08 % of pixels over
        // 1 LSB -- indistinguishable from the 1/30 s step itself (240 vs 241: 54.76 %,
        // 33.07 %) -- while against the unheld --frames 240 it is 94.62 % identical,
        // the extra frame of residency landing at a pose that is nowhere near settled
        // (pending 2992 -> 3004). The held frame names the unheld dump's moment; there
        // is no off-by-one. The same clamp is what puts --dump-both's wireframe on the
        // solid frame's crests, which is the second reason not to move it.
        if ((opt.dumpBoth || settling) && opt.frames && recFrame >= opt.frames) {
            recFrame = opt.frames - 1u;
        }
        simUnix = startUnix + static_cast<double>(recFrame) * (timeScale / 30.0);
        // The headless clock is frame-indexed, so a frame is worth exactly
        // timeScale/30 seconds of world; the boat owes that many whole quanta. Rounding
        // rather than truncating keeps the owed time from drifting slow over a long
        // rail, and at the default scale it is an exact 8.
        stepBoat(static_cast<int>(std::lround((timeScale / 30.0) / SimClock::kDt)));
        // The churn atlas is stateful and its kernel only climbs at a frozen dt, so a
        // held frame would advance the foam the hold's length decides. Freeze it for
        // exactly the held frames (SeaLayer.h freezeChurn).
        if (sea) sea->freezeChurn = settling;
        // The helm leg of --rail-flood: keys at 32 s (cHelmIn) and 40 s (cHelmGap).
        profHelm = !opt.rail.empty() && recFrame >= 32u * 30u;
        if (!opt.rail.empty() && !railKeys.empty()) {
            // M6g: the rails just set a pose in the ONE frame. Nothing switches.
            // Helming outranks the rail: the chase camera has already placed the
            // view on the boat this frame and a rail pose would yank it away. This is
            // what makes `--rail-flood --boat` a chase-cam recording rather than a
            // flypast that happens to contain a hull.
            // M10: the Droste rails leave the keys at the helm for the similarity's
            // own spiral, and say which level the pose is written in.
            if (!helming) {
                if (portal.Valid() && (opt.railDroste || opt.railDrosteOut)) {
                    drosteRailPose(static_cast<double>(recFrame) / 30.0, cam, camLevel,
                                   drosteUp);
                    drosteRailUp = true;
                } else {
                    railPose(static_cast<double>(recFrame) / 30.0, cam);
                }
            }
        }
    }

    // ---- M9bi: THE SUN, PLACED. One conformal point at the origin of the
    // heliocentric frame, carried into THIS frame by the versor chain in Ephemeris.h
    // and handed to the renderer as the scene's single light. Everything that shades
    // reads gSunDir, so placing it once here places it for the globe, the sea, the sky,
    // the terrain and every reflection -- which is what "a global constant for the
    // solar system" has to mean.
    //
    // The direction is taken from the CAMERA'S OWN PLACE on the planet rather than from
    // the Earth's centre: that is the finite-source parallax, at most 8.8 arcsec, and
    // it is the difference between a sun that is somewhere and a sun that merely points.
    // (M10 moved this block up from just before RenderFrame: the globe's level table
    // carries the sun, and the walk that fills the table runs before RenderFrame.)
    if (!opt.sunPinned) {
        const sun::SolarSystem ss = sun::Build(simUnix);
        const double here[3] = {oDir[0], oDir[1], oDir[2]};   // unit = 1 Earth radius
        double sdir[3];
        sun::SunDirFromPlanetPoint(ss, here, sdir);
        renderer.sunPlaced = true;
        renderer.sunDirTangent[0] = static_cast<float>(
            sdir[0] * east0[0] + sdir[1] * east0[1] + sdir[2] * east0[2]);
        renderer.sunDirTangent[1] = static_cast<float>(
            sdir[0] * oDir[0] + sdir[1] * oDir[1] + sdir[2] * oDir[2]);
        renderer.sunDirTangent[2] = static_cast<float>(
            sdir[0] * north0[0] + sdir[1] * north0[1] + sdir[2] * north0[2]);
        renderer.sunAngRadiusDeg = static_cast<float>(ss.app.angRadiusDeg);
        if (!sunLogged) {
            sunLogged = true;
            const double el = std::asin(std::max(
                -1.0, std::min(1.0, double(renderer.sunDirTangent[1])))) * 180.0 / 3.14159265358979;
            double az = std::atan2(double(renderer.sunDirTangent[0]),
                                   double(renderer.sunDirTangent[2])) *
                        180.0 / 3.14159265358979;
            if (az < 0.0) az += 360.0;
            Log("[sun] placed from the ephemeris: subsolar %.3f N %.3f E, %.6f AU "
                "(%.1f W/m2), angular radius %.4f deg; from here azimuth %.2f, "
                "elevation %+.2f  [--sun 112,26 pins the pre-M9bi art direction]",
                ss.app.subsolarLatDeg, ss.app.subsolarLonDeg, ss.app.distAu,
                ss.app.irradianceWm2, ss.app.angRadiusDeg, az, el);
        }
    }

    // ---- M10: THE FLOATING SCALE, and the levels this frame draws (Droste.h). ------
    // (1) THE GAUGE STEP. The camera belongs to the ground it would fall onto -- the
    // gravity analog. When the nearest ground is the next level in (or out), the frame
    // re-roots there: C <- S^-1(C), the view rotated with it, the level index moved.
    // Nothing on screen changes (the gauge identity, gated in RunDrosteSelfTest); what
    // changes is that every double is again near the scale of the world it describes,
    // which is the floating origin's own reason to exist, extended to scale.
    std::vector<GlobeLayer::DrosteLevel> drosteLv;
    bool drosteOuter = false;
    double drosteOuterCam[3] = {0.0, 0.0, 0.0};
    float sunRootF[3];
    renderer.SunDir(sunRootF);
    float sunCamF[3] = {sunRootF[0], sunRootF[1], sunRootF[2]};
    if (portal.Valid() && mode == 1) {
        for (int guard = 0; guard < 4; ++guard) {
            const double C[3] = {cam.px, cam.py, cam.pz};
            const int step = droste::NearestLevel(portal, camLevel, C);
            if (step == 0) break;
            double Cn[3], fw[3], fn[3], un[3];
            portal.Apply(-double(step), C, Cn);
            const DirectX::XMFLOAT3 f = cam.Forward();
            fw[0] = f.x;
            fw[1] = f.y;
            fw[2] = f.z;
            portal.ApplyDir(-double(step), fw, fn);
            portal.ApplyDir(-double(step), drosteUp, un);
            cam.px = Cn[0];
            cam.py = Cn[1];
            cam.pz = Cn[2];
            cam.yaw = static_cast<float>(std::atan2(fn[2], fn[0]));
            const float lim = 3.14159265f / 2.0f - 0.0017f;
            cam.pitch = std::clamp(
                static_cast<float>(std::atan2(fn[1], std::sqrt(fn[0] * fn[0] + fn[2] * fn[2]))),
                -lim, lim);
            for (int i = 0; i < 3; ++i) drosteUp[i] = un[i];
            // Speed is a length per second: the new frame's metres are s^-step old ones.
            cam.speed = static_cast<float>(
                std::clamp(double(cam.speed) / portal.Scale(double(step)), 0.05, 2.5e6));
            camLevel += step;
        }
        // Logged on the frame the camera's level CHANGES (the dive rail re-derives the
        // pose from level floor(u) every frame, so the step itself repeats; the change
        // is the event).
        if (camLevel != loggedLevel) {
            Log("[droste] frame %u: the nearest ground is now level %d -- the frame "
                "re-roots there (a gauge change, C -> S^%+d(C)); local scale %.3g m",
                frame, camLevel, loggedLevel - camLevel,
                droste::LocalScale(portal, camLevel, std::array<double, 3>{cam.px, cam.py, cam.pz}.data()));
            loggedLevel = camLevel;
        }
        // (2) THE SUN, PER LEVEL -- the one trick the user allowed (lighting and
        // atmosphere may cheat). REALISTIC: one sun, the real one; a level twisted k
        // times sees it rotated by Q^-k in its own frame, so an inner Merrimack that
        // faces away from the sun is at night. APPEALING: every level is lit exactly as
        // the root is, in its own frame -- the tower self-similar to the last photon.
        const double sr[3] = {sunRootF[0], sunRootF[1], sunRootF[2]};
        double sc[3] = {sr[0], sr[1], sr[2]};
        // M12 step 4d-2: the camera level's sun through the cycle (LevelApplyDir, the power
        // about its fixed point); the portal's form beside it for the residual (ProbeDrosteTable,
        // with the table below).
        if (opt.drosteLight == 0) drosteLeaf.LevelApplyDir(-double(camLevel), sr, sc);
        for (int i = 0; i < 3; ++i) sunCamF[i] = static_cast<float>(sc[i]);
        double sc2[3] = {sr[0], sr[1], sr[2]};   // the portal's, for the record
        if (opt.drosteLight == 0) portal.ApplyDir(-double(camLevel), sr, sc2);
        renderer.sunPlaced = true;
        for (int i = 0; i < 3; ++i) renderer.sunDirTangent[i] = sunCamF[i];
        // (3) THE LEVELS: two out (never above the root), three in. The globe walks
        // each under its own eye S^-k(C) and drops any whose planet is under half a
        // pixel -- the screen, not this range, is what ends the tower.
        const double C[3] = {cam.px, cam.py, cam.pz};
        // INNER FIRST (+1, +2, +3, then -1, -2): the globes the camera is diving into are
        // small and cheap, and they are the subject; the worlds outside are big and
        // mostly hidden behind the planet the camera stands on. Walked in this order a
        // tight record budget costs an outer horizon, never the next globe (MEASURED on
        // the first dive: rel -1 took 34 k records and the next globe got none).
        const int nOut = (std::min)(camLevel, 2);
        // M12 step 4d instrument: each level's row from the cycle beside the portal's, compared
        // after the loop (ProbeDrosteTable).
        DrosteProbeRow probeRows[GlobeLayer::kMaxLevels];
        int probeN = 0;
        for (int k = 0; k < 3 + nOut; ++k) {
            const int rel = (k < 3) ? k + 1 : -(k - 2);
            GlobeLayer::DrosteLevel L;
            L.rel = rel;
            // M12 step 4d-2: THE EYE FROM THE CYCLE -- S^-rel(C) about the fixed point
            // (Space.h PowApply: p + s^-rel Q^-rel (C - p), the form the gauge identity is drawn
            // by); the portal's own closed form is computed beside it for the record only.
            // sigma and Q (the constant buffer's rows) stay the portal's.
            drosteLeaf.LevelApply(-double(rel), C, L.cam);
            L.sigma = portal.Scale(double(rel));
            portal.Rot(double(rel), L.Q);
            DrosteProbeRow& pr = probeRows[probeN++];
            {
                const Placement Lk = drosteLeaf.Level(double(rel));
                pr.rel = rel;
                portal.Apply(-double(rel), C, pr.cam);                // the portal's, for the record
                for (int i = 0; i < 3; ++i) pr.cam2[i] = L.cam[i];   // the read
                pr.sigma = L.sigma;
                for (int qr = 0; qr < 3; ++qr) {
                    for (int qc = 0; qc < 3; ++qc) pr.Q[qr][qc] = L.Q[qr][qc];
                }
                pr.sigma2 = Lk.s;
                double ax[3], ay[3], az[3];
                Lk.Rows(ax, ay, az);   // the columns of Q: Q[i][j] = axis_j[i], own -> true
                for (int i = 0; i < 3; ++i) {
                    pr.Q2[i][0] = ax[i];
                    pr.Q2[i][1] = ay[i];
                    pr.Q2[i][2] = az[i];
                }
                // The gauge placement the globe pulls its frustum planes through: the linear
                // part of Level(rel) -- the eye-to-eye translation cancels exactly, S^k(C_k) = C.
                L.gauge = Lk;
                L.gauge.t[0] = L.gauge.t[1] = L.gauge.t[2] = 0.0;
            }
            const double gy = L.cam[1] + planetR;
            const double altK =
                std::sqrt(L.cam[0] * L.cam[0] + gy * gy + L.cam[2] * L.cam[2]) - planetR;
            L.reliefExagg = static_cast<float>(std::clamp(altK / 250000.0, 1.0, 20.0));
            double sk[3] = {sr[0], sr[1], sr[2]};
            if (opt.drosteLight == 0) drosteLeaf.LevelApplyDir(-double(camLevel + rel), sr, sk);
            for (int i = 0; i < 3; ++i) L.sun[i] = static_cast<float>(sk[i]);
            for (int i = 0; i < 3; ++i) {
                pr.sun[i] = sr[i];    // the portal's, for the record (below)
                pr.sun2[i] = sk[i];   // the read
            }
            if (opt.drosteLight == 0) portal.ApplyDir(-double(camLevel + rel), sr, pr.sun);
            // THE SKY IT SEES. Realistic: every level inside the root sits a few
            // hundred metres up in the root's air, so the sky over it is the ROOT's --
            // its zenith turned into this level's frame, lit by the root's day -- and a
            // night-side inner sea mirrors that bright sky. Appealing: its own sky.
            if (opt.drosteLight == 0 && camLevel + rel > 0) {
                const double upR[3] = {0.0, 1.0, 0.0};
                double su[3];
                drosteLeaf.LevelApplyDir(-double(camLevel + rel), upR, su);
                for (int i = 0; i < 3; ++i) L.skyUp[i] = static_cast<float>(su[i]);
                L.skyDay = static_cast<float>(std::clamp(sr[1] * 3.0 + 0.12, 0.0, 1.0));
                for (int i = 0; i < 3; ++i) pr.skyUp2[i] = su[i];             // the read
                portal.ApplyDir(-double(camLevel + rel), upR, pr.skyUp);   // the portal's
                pr.hasSky = true;
            }
            L.bankSet = (rel == -1 && waterBankB) ? 1 : -1;
            if (rel == -1) {
                drosteOuter = true;
                for (int i = 0; i < 3; ++i) drosteOuterCam[i] = L.cam[i];
            }
            drosteLv.push_back(L);
        }
        ProbeDrosteTable(probeRows, probeN, sc2, sc, frame);   // (the portal's, the read)
    }
    if (globe) {
        globe->SetSun(sunCamF);
        // The camera level's own sky (slot 0): the root's, turned, under realistic
        // lighting inside the tower; its own everywhere else.
        {
            float upC[3] = {0.0f, 1.0f, 0.0f};
            float dayC = -1.0f;
            if (portal.Valid() && mode == 1 && opt.drosteLight == 0 && camLevel > 0) {
                const double upR[3] = {0.0, 1.0, 0.0};
                double su[3];
                portal.ApplyDir(-double(camLevel), upR, su);
                for (int i = 0; i < 3; ++i) upC[i] = static_cast<float>(su[i]);
                dayC = static_cast<float>(std::clamp(double(sunRootF[1]) * 3.0 + 0.12, 0.0, 1.0));
            }
            globe->SetCamSky(upC, dayC);
        }
        if (portal.Valid() && mode == 1) {
            globe->SetDroste(drosteLv.data(), static_cast<int>(drosteLv.size()),
                             opt.drosteLight, portal.centre, portal.radius, camLevel);
        }
    }
    // M10: WHOSE SKY. Identity and the camera's sun reproduce the old dome exactly; under
    // REALISTIC lighting inside the tower the backdrop is the ROOT's sky -- every inner
    // level sits a few hundred metres up in the root's air -- turned into the camera's
    // frame by Q^L, with the root's own sun. Under APPEALING lighting every level has a
    // sky of its own, and the dome is the one whose ground calls for a dome the loudest
    // (Droste.h Grounds, domeRel) -- NOT the camera's level, which is the gauge: drawn in
    // the camera's frame it swung 60 deg at the re-root. The space backdrop's sun is
    // likewise the sun of the level whose orbit calls for space (spaceRel).
    {
        float rows[9] = {1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f};
        const float* skySun = sunCamF;
        float spaceSun[3] = {sunCamF[0], sunCamF[1], sunCamF[2]};
        int domeRel = 0;
        if (portal.Valid() && mode == 1) {
            if (opt.drosteLight == 0) {
                domeRel = -camLevel;   // the root's
            } else {
                const double C[3] = {cam.px, cam.py, cam.pz};
                const droste::GroundField gf = droste::Grounds(portal, camLevel, C);
                domeRel = gf.domeRel;
                double m[3][3], sv[3];
                const double sr[3] = {sunRootF[0], sunRootF[1], sunRootF[2]};
                portal.Rot(double(gf.spaceRel), m);   // that level -> the camera's frame
                droste::MatVec(m, sr, sv);
                for (int i = 0; i < 3; ++i) spaceSun[i] = static_cast<float>(sv[i]);
            }
            if (domeRel != 0) {
                double m[3][3];
                portal.Rot(double(-domeRel), m);   // the camera's frame -> the dome's
                for (int r = 0; r < 3; ++r) {
                    for (int c = 0; c < 3; ++c) rows[r * 3 + c] = static_cast<float>(m[r][c]);
                }
                skySun = sunRootF;   // every level's own sun has the root's numbers
            }
        }
        sky->SetSkyFrame(rows, skySun);
        if (globe) globe->SetSpaceSun(spaceSun);
    }

    // M6g world housekeeping, all continuous in altitude: gravity-up for the view
    // basis, speed and relief exaggeration scale with height, the camera never sinks
    // under the ground (terrain near home, sphere+relief elsewhere), and the layer
    // set sheds by altitude the way residency sheds by distance.
    const double altV = altOf(cam);
    if (mode == 1) {
        const double gy = cam.py + planetR;   // anti-gravity = radial from (0,-R,0)
        const double gl = std::sqrt(cam.px * cam.px + gy * gy + cam.pz * cam.pz);
        cam.upHint[0] = static_cast<float>(cam.px / gl);
        cam.upHint[1] = static_cast<float>(gy / gl);
        cam.upHint[2] = static_cast<float>(cam.pz / gl);
        if (altV > 6000.0) {
            cam.speed = static_cast<float>(std::clamp(altV * 0.45, 60.0, 2.5e6));
        }
        // M10: THE SECOND BODY'S TERM (M6g's upHint comment promised one). Anti-gravity
        // of the Droste world: each level's planet radial, weighted 1/d^2 by the distance
        // to its ground, so the roll follows the ground you would fall onto and turns
        // continuously as another ground takes over -- an inner planet's up is its
        // parent's WEST under the quarter twist. The dive rail writes its own up (the
        // spiral carries roll exactly); free flight takes the field. And the speed rides
        // the local scale (Droste.h LocalScale): every approach is exponential, which is
        // the logarithm the camera needs to cross decades without a gear change.
        double altWater = altV;   // the lowest eye over a sea this frame draws waves on
        if (portal.Valid()) {
            const double C[3] = {cam.px, cam.py, cam.pz};
            // The ground field (Droste.h Grounds): the roll's up and the speed's scale
            // from ONE gauge-invariant weight.
            const droste::GroundField gf = droste::Grounds(portal, camLevel, C);
            const double* gU = drosteRailUp ? drosteUp : gf.up;
            const double gn = std::sqrt(gU[0] * gU[0] + gU[1] * gU[1] + gU[2] * gU[2]);
            if (gn > 1e-30) {
                for (int i = 0; i < 3; ++i) cam.upHint[i] = static_cast<float>(gU[i] / gn);
            }
            const double lam = gf.lambda;
            if (!drosteRailUp) {
                cam.speed = (lam > 6000.0)
                                ? static_cast<float>(std::clamp(lam * 0.45, 60.0, 2.5e6))
                                : static_cast<float>((std::min)(double(cam.speed),
                                                                (std::max)(lam * 1.5, 0.05)));
            }
            if (drosteOuter) {
                const double oy = drosteOuterCam[1] + planetR;
                const double altO = std::sqrt(drosteOuterCam[0] * drosteOuterCam[0] +
                                              oy * oy +
                                              drosteOuterCam[2] * drosteOuterCam[2]) -
                                    planetR;
                altWater = (std::min)(altWater, altO);
            }
        }
        // The FFT cascades serve every level's sea (their tiles are world-XZ periodic),
        // so they run while ANY drawn sea is close -- the outer level's included.
        if (sea) sea->enabled = altWater < 60000.0 && !marsMode && !opt.albedo;
        // M6p: the survey vectors decimate to the view -- tolerance = one ground
        // pixel; the layer republishes only when the x8 bucket changes.
        if (gisLayer) {
            const float vh = opt.headless
                                 ? static_cast<float>(opt.height)
                                 : static_cast<float>(std::max(1u, window.Height()));
            gisLayer->tolMeters = static_cast<float>(altV * cam.fovY / vh);
        }
        // M6x: the weather manager's residency clock -- the camera's ground position
        // is the demand signal; dormant windows spin up as it arrives, owned solvers
        // advance. All in the flat one-world frame. (The CPU mirrors no longer
        // refresh here: nothing in the loop reads them -- step 2 of PERF_EXPERIMENT.)
        if (!marsMode) {
            PROF_BEGIN();
            weather.Update(gpu, renderer.Shaders(), opt.shaderDir, simUnix,
                           BathyModel::kOrgLat + cam.pz / BathyModel::kMPerLat,
                           BathyModel::kOrgLon + cam.px / BathyModel::kMPerLon,
                           altV);
            // M9ay: every ACTIVE window's lattice at mip 0, from the manager that owns them.
            weather.PinDomains(resMgr, hgtTenant);
            PROF_END(0);
        }
        // M7: the bank's rings follow the camera; the globe binds THIS frame's
        // origins (residency committed in SetFrame, content recomposed in Render).
        if (waterBank && globe) {
            // M8: the scene file hot-reloads -- edit, save, watch the water
            // change. A geometry edit re-Configures the solver (its bucket key
            // rolls, the cache answers or a background solve runs).
            // Step 3 (docs/PERF_EXPERIMENT.md): the stat runs only when the
            // directory watcher says the file moved (every 30 frames with no
            // watch); the mtime compare and the reload are unchanged. The stat
            // alone was 0.140 ms whole / 0.185 ms helm per frame (step 2's bench).
            PROF_BEGIN();
            if (sceneWatch.Poll(frame) &&
                WaterSceneChanged(kScenePath, &waterSceneMtime) &&
                LoadWaterScene(kScenePath, waterScene)) {
                if (waveField) {
                    WaveFieldConfig wcfg2 = sceneToWaveCfg(waterScene);
                    waveFrame = WaveFieldSource::Align(wcfg2);   // M9bc: the page grid
                    waveField->Configure(wcfg2, &compositor,
                                         hgtCh, &waterAtlas, &model, entSta,
                                         haveCurrents ? &currents : nullptr,
                                         wfCtSta);
                    waterBank->SetWaveField(
                        waterScene.wfEnabled ? waveField.get() : nullptr);
                    if (waterBankB) {
                        waterBankB->SetWaveField(
                            waterScene.wfEnabled ? waveField.get() : nullptr);
                    }
                }
                globe->foamOpacity = waterScene.foamOpacity;
                globe->ringBlendTexels = waterScene.ringBlendTexels;
                globe->causticStrength = waterScene.causticStrength;
                globe->waterOptics = waterScene.waterOptics;
                if (waterScene.jettyCrestNavd > -90.0f) {
                    globe->editFloorNavd = waterScene.jettyCrestNavd;
                }
                Log("[scene] %s hot-reloaded at frame %u, %s", kScenePath, frame,
                    sceneWatch.Trigger().c_str());
            }
            PROF_END(1);
            // M8: bucket-watch + background solve + upload/swap for the solved
            // wave field, BEFORE the bank recomposes so the kernel binds a whole
            // field or the previous one -- never a half-written atlas.
            // (frame >= 2: the first frames run on the boot clock before --start
            // settles; solving them caches a real answer for the wrong instant.)
            if (waveField && sea && waterScene.wfEnabled && frame >= 2) {
                PROF_BEGIN();
                waveField->Update(gpu, simUnix, sea->Parts(), sea->activeParts,
                                  opt.headless);
                PROF_END(2);
                // M9bc: the solve moved -> a new tree under the same tenant, its
                // pyramid prefilled to disk, the old tiles dropped, then the wants.
                // Publish a finished prefill. Only the swap and the Drop touch the
                // residency manager, and both stay on this thread.
                if (wavePrefillDone.load(std::memory_order_acquire)) {
                    std::atomic_store(waveTree.get(), wavePending);
                    resMgr.Drop(waveT);
                    Log("[wave] bucket %016llx -> tree %s: %u tiles prefilled (%u planes, "
                        "every mip) in %.2f s on a worker -- the frame did not wait",
                        static_cast<unsigned long long>(wavePendingKey),
                        wavePending->Id().c_str(), wavePendingTiles, wavePendingPlanes,
                        wavePendingSec);
                    wavePending.reset();
                    wavePrefillDone.store(false, std::memory_order_release);
                    wavePrefillBusy.store(false, std::memory_order_release);
                }
                // Kick a new one when the bucket has rolled and none is in flight. The
                // busy flag matters: the source's key is mutated here, so a second job
                // over the same source would be filling a tree whose identity moved.
                if (waveSrc && waveT >= 0 && waveField->Ready() &&
                    waveField->LiveKey() != waveSrc->Key() &&
                    !wavePrefillBusy.load(std::memory_order_acquire)) {
                    waveSrc->SetKey(waveField->LiveKey());
                    wavePendingKey = waveSrc->Key();
                    wavePendingPlanes = waveField->Table().nUsed + 1u;
                    float u0, v0, u1, v1;
                    waveSrc->WindowUv(u0, v0, u1, v1);
                    wavePrefillBusy.store(true, std::memory_order_release);
                    const ColorFrame wf = waveFrame.color;
                    WaveFieldSource* wsrc = waveSrc.get();
                    const uint32_t planes = wavePendingPlanes;
                    Threads().Submit(Lane::Compute, "wave.prefill",
                                     [&wavePending, &wavePrefillDone, &wavePendingTiles,
                                      &wavePendingSec, wf, wsrc, planes, u0, v0, u1, v1]() {
                        const auto tp0 = Clock::now();
                        auto fresh = std::make_shared<TileTree>(wsrc, TileTree::Fmt::Raw4);
                        wavePendingTiles = fresh->Prefill(wf, 0u, planes, u0, v0, u1, v1);
                        wavePendingSec =
                            std::chrono::duration<double>(Clock::now() - tp0).count();
                        wavePending = fresh;
                        wavePrefillDone.store(true, std::memory_order_release);
                    });
                }
                if (waveSrc && waveT >= 0 && waveSrc->Key() != 0) {
                    // Bracketed apart from waveField.Update: these Wants are the whole
                    // window at mip 0 on every plane, a map find per un-stamped tile.
                    //
                    // M9bl: ...and THAT is the whole point of the gate below. Mip 0 is
                    // the finest level of a 16384-texel pyramid, asked for across the
                    // entire window, on every plane, every frame, from every altitude.
                    // From orbit the solved window subtends single-digit pixels, so
                    // this demanded the sharpest data in the tenant for a handful of
                    // pixels -- and it competes for the SAME 48-slot in-flight loader
                    // queue the imagery streams through.
                    //
                    // MEASURED (--res-trace, storm rail f30): earth.height held 27 read
                    // slots against 17 wave planes and only 17 slots against 33, so the
                    // land lost a third of its loader bandwidth and stood at a coarse
                    // mip through the whole descent -- the blurry coastline. Raising
                    // --tile-budget from 3000 to 9000 changed nothing, because the
                    // frame budget was never the bottleneck; the queue was.
                    //
                    // The rule: stream the window only once it is worth its finest mip.
                    // WavePageSample reads mip 0 and gates on residency, so under the
                    // threshold the solved field contributes nothing -- correct, since
                    // at that range it cannot be seen, and the ambient cascades own the
                    // water anyway. At 3.8 km of window this crosses at ~40 km, which
                    // the flood rail reaches ~17 s in: fourteen seconds of lead before
                    // the helm, so the field is long resident by the time it matters.
                    const auto& wtab = waveField->Table();
                    const double cellW = 1.0 / (std::max)(double(wtab.invCell), 1e-9);
                    const double spanW = double(wtab.nx) * cellW;
                    const double cxW = double(wtab.orgX) + spanW * 0.5;
                    const double czW = double(wtab.orgZ) + double(wtab.ny) * cellW * 0.5;
                    const double dxW = cam.px - cxW, dzW = cam.pz - czW;
                    double distW = (std::max)(
                        std::sqrt(dxW * dxW + cam.py * cam.py + dzW * dzW), 1.0);
                    // M10: the outer level's sea is the SAME solve seen from S(C); the
                    // nearer of the two eyes decides the mip (one tenant, one want).
                    if (drosteOuter) {
                        const double ox = drosteOuterCam[0] - cxW;
                        const double oz = drosteOuterCam[2] - czW;
                        distW = (std::min)(distW, (std::max)(std::sqrt(
                            ox * ox + drosteOuterCam[1] * drosteOuterCam[1] + oz * oz), 1.0));
                    }
                    const double vhW = opt.headless
                                           ? double(opt.height)
                                           : double((std::max)(1u, window.Height()));
                    const double pixAngW =
                        double(cam.fovY) / (std::max)(vhW, 1.0);
                    const double winPx = spanW / (distW * (std::max)(pixAngW, 1e-9));
                    // The mip the window can actually be SEEN at: nx texels across
                    // mapped onto winPx pixels. Every other tenant chooses this way
                    // (LeafWants: px = arc / (distNear * pixAng)); this one asked for
                    // mip 0 flat, which is why it never formed a gradient and why
                    // --ring-loads had no parent chain to admit. Now the request walks
                    // in coarse-first like everything else, and WavePageSample reads
                    // whatever level has landed.
                    // ...biased FINER than that, because the water is the subject of
                    // this renderer and must never be the coarsest thing on screen.
                    // The screen rule alone is right for imagery, where a coarse mip
                    // just looks soft; on the solved field a coarse level is a
                    // structured artefact -- the shallow water showed the page's own
                    // texel grid as faint rectangles (the user's catch). Two levels
                    // of bias put the window at mip 0 well before the helm arrives,
                    // and it costs nothing at altitude: the clamp to kWaveMaxMip is
                    // already binding there, so the orbit legs request exactly what
                    // they requested before and the imagery keeps its loader slots.
                    const double texAcross = (std::max)(double(wtab.nx), 1.0);
                    const double lvl =
                        std::ceil(std::log2((std::max)(texAcross /
                                                       (std::max)(winPx, 1.0), 1.0))) -
                        kWaterMipBias;
                    const uint32_t wantMip = static_cast<uint32_t>(
                        (std::min)((std::max)(lvl, 0.0), kWaveMaxMip));
                    PROF_BEGIN();
                    float u0, v0, u1, v1;
                    waveSrc->WindowUv(u0, v0, u1, v1);
                    const uint32_t nPlanes = waveField->Table().nUsed + 1u;
                    for (uint32_t p = 0; p < nPlanes; ++p) {
                        resMgr.Want(waveT, 6u + p, wantMip, u0, v0, u1, v1);
                    }
                    PROF_END(10);
                }
            }
            // M8 fleet: advance the traffic (stateless), hand the table to the
            // bank kernel (wakes) -- and to the vessel layer when it exists.
            {
                float bA[32] = {}, bB[32] = {};
                if (waterScene.fleetEnabled && route.Ready()) {
                    const double LR = route.Length();
                    const double kPiF = 3.14159265358979;
                    for (int b = 0; b < waterScene.fleetCount && b < 8; ++b) {
                        const WaterSceneConfig::Boat& fb = waterScene.fleet[b];
                        double q = std::fmod(fb.offsetS + fb.speed * simUnix,
                                             2.0 * LR);
                        if (q < 0.0) q += 2.0 * LR;
                        double s = (q < LR) ? q : 2.0 * LR - q;
                        bool back = q >= LR;
                        if (fb.dir < 0) {
                            s = LR - s;
                            back = !back;
                        }
                        double bx, bz, hd;
                        route.At(s, bx, bz, hd);
                        if (back) hd += kPiF;
                        bA[b * 4 + 0] = static_cast<float>(bx);
                        bA[b * 4 + 1] = static_cast<float>(bz);
                        bA[b * 4 + 2] = static_cast<float>(hd);
                        bA[b * 4 + 3] = static_cast<float>(fb.speed);
                        bB[b * 4 + 0] = static_cast<float>(fb.wakeAmp);
                        bB[b * 4 + 1] = static_cast<float>(fb.halfLen);
                        bB[b * 4 + 2] = 1.0f;
                    }
                }
                waterBank->SetBoats(bA, bB);
                if (waterBankB) waterBankB->SetBoats(bA, bB);
            }
            PROF_BEGIN();
            waterBank->SetFrame(gpu, simUnix, cam.px, cam.pz);
            PROF_END(3);
            float orgs[12];
            for (int mR = 0; mR < WaterBankLayer::kMips; ++mR) {
                waterBank->RingOrigin(mR, orgs[mR * 2], orgs[mR * 2 + 1]);
            }
            uint32_t derivS[3];
            float patchS[3], bandKS[3], bandRmsS[3], bandFoldS[3];
            const double kPiB = 3.14159265358979;
            const double kCutB[4] = {2.0 * kPiB / 756.0, 2.0 * kPiB / 60.0,
                                     2.0 * kPiB / 12.0, 0.9 * kPiB * 256.0 / 47.0};
            for (int c = 0; c < 3; ++c) {
                derivS[c] = sea->FftDerivSrv(c);
                patchS[c] = sea->FftPatchL(c);
                bandKS[c] = static_cast<float>(std::sqrt(kCutB[c] * kCutB[c + 1]));
                bandRmsS[c] = sea->BandRms(c);
                bandFoldS[c] = sea->BandKFold(c);
            }
            waterBank->injectPattern = opt.inject;
            if (hgtWinTenant >= 0) {
                // M9aq: in pages mode the window is slice 6 of the height array.
                waterBank->SetHeightWindow(resMgr.TextureSrv(hgtWinTenant),
                                           resMgr.ResidencySrv(hgtWinTenant),
                                           m_A.surface.winH,
                                           hgtWinTenant == hgtTenant ? 6u : UINT32_MAX);
                if (waterBankB) {
                    waterBankB->SetHeightWindow(resMgr.TextureSrv(hgtWinTenant),
                                                resMgr.ResidencySrv(hgtWinTenant),
                                                m_A.surface.winH,
                                                hgtWinTenant == hgtTenant ? 6u
                                                                          : UINT32_MAX);
                }
            }
            globe->windGateVal = sea->WindGate();
            globe->SetWaterBank(waterBank->DispSrv(), waterBank->ParamSrv(),
                                waterBank->DetailSrv(), derivS, patchS, bandKS,
                                bandRmsS, bandFoldS, sea->heightScale,
                                waterBank->BaseTexelM(), orgs, opt.oneWater);
            // M10: set B follows the OUTER level's eye, S(C) -- the sea the camera's
            // planet floats in, seen from where the camera really is in that level's
            // own frame. Off (and not filled) while the camera is at the root.
            if (waterBankB) {
                waterBankB->enabled = drosteOuter;
                float orgB[12] = {};
                if (drosteOuter) {
                    waterBankB->injectPattern = opt.inject;
                    waterBankB->SetFrame(gpu, simUnix, drosteOuterCam[0],
                                         drosteOuterCam[2]);
                    for (int mR = 0; mR < WaterBankLayer::kMips; ++mR) {
                        waterBankB->RingOrigin(mR, orgB[mR * 2], orgB[mR * 2 + 1]);
                    }
                }
                globe->SetWaterBankB(waterBankB->DispSrv(), waterBankB->ParamSrv(),
                                     waterBankB->DetailSrv(), orgB, drosteOuter);
            }
        }
        // (--albedo: the water stands down too -- textures judged as layered images,
        // nothing else in the frame; the lit look retunes separately.)
        // M10: under REALISTIC Droste lighting the sky belongs to the OUTERMOST world:
        // every inner level sits a few hundred metres up in the root's air, so from
        // anywhere inside the tower the backdrop is the root's low sky, never space.
        // APPEALING gives each level its own -- its orbit reads as an orbit.
        const bool rootSky = portal.Valid() && opt.drosteLight == 0 && camLevel > 0;
        sky->enabled = (altV < 9000.0 || rootSky) && !marsMode && !opt.albedo;   // low haze dome...
        if (globe) globe->skyPassEnabled = !sky->enabled && !opt.albedo;   // ...or the
                                                    // limb shell, never both at once
                                                    // (--albedo: neither -- textures)
        if (globe) {
            globe->skyPassWeight = 1.0f;
            globe->skyOwnAir = 1.0f;
        }
        // M10 APPEALING: every level wears the backdrop its OWN altitude calls for (the
        // dome low, the limb shell and space above ~9 km of its own units), and the
        // grounds within reach blend by the same 1/d^2 weights the roll follows. The
        // weights read true distances and each level's own altitude -- both blind to
        // which frame the camera happens to be rooted in -- so the re-root cannot pop the
        // sky, and a camera leaving an inner planet's orbit for its surface fades from
        // its space into its day. Two backdrops drawn, one cross-fade, no switch.
        if (globe && portal.Valid() && opt.drosteLight == 1 && !marsMode && !opt.albedo) {
            const double C[3] = {cam.px, cam.py, cam.pz};
            const droste::GroundField gf = droste::Grounds(portal, camLevel, C);
            const float W = static_cast<float>(gf.W);
            sky->enabled = W < 0.999f;
            globe->skyPassEnabled = W > 0.001f;
            globe->skyPassWeight = W;
            // Whose space is it? The part of W an INNER planet's orbit called for is
            // space with that planet's limb in it (PsLimb) -- none of the camera level's
            // air. Only the camera level's own share carries its shell (the grey haze of
            // the root's low sky was standing in for the inner globe's black).
            globe->skyOwnAir = static_cast<float>(gf.ownShare);
        }
    } else {
        cam.upHint[0] = 0.0f;
        cam.upHint[1] = 1.0f;
        cam.upHint[2] = 0.0f;
    }
    if (mode == 1 && globe) {
        globe->reliefExagg =
            static_cast<float>(std::clamp(altV / 250000.0, 1.0, 20.0));
        PROF_BEGIN();
        const double g = groundAt(cam.px, cam.pz);
        PROF_END(6);
        if (cam.py < g + 1.2) cam.py = g + 1.2;
        const float viewH = opt.headless ? static_cast<float>(opt.height)
                                         : static_cast<float>(
                                               std::max(1u, window.Height()));
        const float aspect =
            (opt.headless ? static_cast<float>(opt.width) : window.Width()) / viewH;
        globe->WalkReset();
        resMgr.WantStatsReset();
        // M6e screw-prefetch: extrapolate the pose ~0.8 s ahead along its own screw and
        // let the walk under THAT camera queue tiles early (predicted priority).
        // M9v: THE PREFETCH WALK, AMORTIZED. Measured at 3.25 ms per frame at helm --
        // a full six-face quadtree descent with NO frustum cull (the predicted view is
        // approximate by design, so it walks more nodes than the real pass does). Paying
        // that at 30 Hz to serve a prediction 0.8 SECONDS ahead is spending a whole
        // frame budget to refine a guess whose own horizon is 24 frames wide.
        //
        // Every 6th frame is 5 Hz -- 0.2 s of granularity against an 0.8 s lookahead, so
        // the prefetch still lands four times inside its own horizon. The phase is tied
        // to the frame counter rather than a timer so a headless rail and an interactive
        // run walk the same nodes on the same frames.
        // --settle-sync: the hold runs WITHOUT the predicted walk. MEASURED (step 1 of the
        // perf plan, helm still): with it, pending reached 0 within 300 held frames but
        // was never quiet for four consecutive frames in 3000 -- the walk re-wants tiles
        // on every third frame at a held pose; with it off for the WHOLE run
        // (--predict-every 1000000) the hold converged in 217-226 frames and the two
        // held stills were 5 px apart. The 240 real frames keep it (the still's history
        // is the shipped one); the hold's residual is logged at the exit test.
        // Step 5 (docs/PERF_EXPERIMENT.md): THE PREFETCH WALK LEAVES THE MAIN THREAD.
        // Its inputs are final here -- the pose is clamped, the frame vectors and the
        // window origins are per-run -- so the worker starts NOW and walks beside
        // SetView; the replay below, where the walk used to run, waits for it and
        // issues the same Want(predicted) calls in the same order (never skips: a rail's
        // want stream must not depend on the machine). PredictNextPose keeps its call
        // cadence, and so its 72-frame effective lookahead. Slot 8 brackets the pose and
        // the post here plus the join and the replay below. --predict-inline runs the
        // same walk synchronously at the old point, for the A/B: both print the
        // predicted stream's FNV-1a ([predict], [rail], --res-trace).
        const bool predictFrame = !settling && (frame % kPredictEvery) == 0;
        if (predictFrame && !opt.predictInline) {
            PROF_BEGIN();
            Camera pred = cam;
            motorPose(resMgr.PredictNextPose(poseMotor(cam), 24.0), pred);
            globe->StartPredictWalk(cam, pred, viewH);
            PROF_END(8);
        }
        PROF_BEGIN();
        globe->SetView(cam, aspect, viewH, simUnix - startUnix);
        PROF_END(7);
        if (!opt.rail.empty() && frame >= 150u) {
            walkNodesAcc += globe->walkNodes;
            walkLeavesAcc += globe->walkLeaves;
            walkWantNsAcc += globe->walkWantNs;
            morphFullAcc += globe->walkMorphFull;
            morphPartAcc += globe->walkMorphPart;
            wantTouchAcc += resMgr.wantTouches;
            wantHitAcc += resMgr.wantHits;
            ++walkFrames;
        }
        if (predictFrame) {
            PROF_BEGIN();
            if (opt.predictInline) {
                Camera pred = cam;
                motorPose(resMgr.PredictNextPose(poseMotor(cam), 24.0), pred);
                globe->StartPredictWalk(cam, pred, viewH);
            }
            globe->ReplayPredictWants();
            PROF_END(8);
        }
    }

    PROF_BEGIN();
    tide->SetTime(simUnix, windowSec);
    PROF_END(4);
    // M12 step 4g: THE ONE SURFACE CONSTANT BUFFER. The surface fills its rows once a frame
    // into the renderer's b2 buffer (Renderer::surfaceCb), which RenderFrame pushes once and
    // binds for every layer: the globe, the sea, the terrain and the GIS vectors read the
    // same bytes from one buffer where each carried a copy inside its own cbuffer -- the
    // globe's refilled every frame from this same SurfaceFrame, the other three once at boot
    // (Session), and the fingerprints of all of them agreed with the bytes pushed for b2 at
    // every pose (scratchpad/step4g_probe.py). Mars fills too, as the globe's own fill did:
    // its rows say "height cube, no page" (SurfaceFrame.h's banner). The step 0 fingerprint
    // stays, printed when the bytes change, and is the only one.
    if (globe) {
        m_A.surface.Fill(renderer.surfaceCb, resMgr);
        static uint64_t sLastSurface = 0;
        const uint64_t h = Fnv1aBytes(&renderer.surfaceCb, sizeof(renderer.surfaceCb));
        if (h != sLastSurface) {
            sLastSurface = h;
            Log("[surface] main fill FNV-1a %016llx", static_cast<unsigned long long>(h));
        }
    }
    renderer.waterLevel = static_cast<float>(tide->focusHeight);
    // The terrain speaks NAVD88; the tide speaks MLLW. One offset joins them. In
    // estuary mode the open-water level is the ENTRANCE station's (M5c), and the west
    // boundary rides the interpolated river tide.
    const double waterNavd =
        bathy.Ready() ? oceanAt(simUnix) : tide->focusHeight + datumOff;
    lastWaterNavd = waterNavd;   // next frame's camera-pivot rays test against it
    PROF_BEGIN();
    if (swe.Ready()) {
        swe.SetBoundaries(static_cast<float>(westAt(simUnix)),
                          static_cast<float>(southAt(simUnix)),
                          static_cast<float>(westQAt(simUnix)));
    }
    PROF_END(9);
    // M7k: arm the programmatic .wpix capture so it records the run's LAST warm
    // frames -- every pass named by its state-diagram node.
    if (opt.pixFrames > 0 && opt.frames > 0 && opt.frames + (opt.rail.empty() ? 0u : 150u) >= opt.pixFrames + 4 &&
        frame + opt.pixFrames + 4 == opt.frames + (opt.rail.empty() ? 0u : 150u)) {
        PixGpuCaptureFrames(L"gagame.wpix", opt.pixFrames);
        pixArmed = true;
    }
    PROF_BEGIN();
    if (sea) sea->SetTime(simUnix, bathy.Ready() ? waterNavd : tide->focusHeight,
                          cam.px, cam.pz);
    PROF_END(5);
    // M9ba: the exposure node's inputs; a bucket roll re-keys the tree and drops the
    // tenant's tiles. Then demand the page at mip 3 over +-20 km around the camera --
    // the bank's outer ring -- every frame. Its own bracket (slot 11): sea.SetTime
    // used to carry it.
    PROF_BEGIN();
    if (sea && exposureSrc && exposureT >= 0) {
        if (exposureSrc->Set(sea->PeakDirX(), sea->PeakDirZ(), waterNavd,
                             sea->PeakDirValid())) {
            auto fresh = std::make_shared<TileTree>(exposureRoot.get(), TileTree::Fmt::Half);
            exposureTenant.Bind(*fresh);   // M12 step 3e: the one law -- its folds invalidate slice 6
            std::atomic_store(exposureTree.get(), fresh);
            resMgr.Drop(exposureT);
            Log("[exposure] bucket rolled -> tree %s", fresh->Id().c_str());
        }
        if (exposureSrc->Valid()) {
            const double piP = 3.14159265358979, n14 = 16384.0 * 256.0;
            const double latC = BathyModel::kOrgLat + cam.pz / BathyModel::kMPerLat;
            const double lonC = BathyModel::kOrgLon + cam.px / BathyModel::kMPerLon;
            const double dLat = 20000.0 / BathyModel::kMPerLat;
            const double dLon = 20000.0 / BathyModel::kMPerLon;
            auto mU = [&](double lonDeg) {
                return ((lonDeg + 180.0) / 360.0 * n14 - winOrgX) / 16384.0;
            };
            auto mV = [&](double latDeg) {
                const double l = latDeg * piP / 180.0;
                return ((0.5 - std::log(std::tan(piP * 0.25 + l * 0.5)) / (2.0 * piP)) *
                            n14 - winOrgY) / 16384.0;
            };
            const float u0 = float(std::clamp(mU(lonC - dLon), 0.0, 1.0));
            const float u1 = float(std::clamp(mU(lonC + dLon), 0.0, 1.0));
            const float v0 = float(std::clamp(mV(latC + dLat), 0.0, 1.0));
            const float v1 = float(std::clamp(mV(latC - dLat), 0.0, 1.0));
            if (u1 > u0 && v1 > v0) resMgr.Want(exposureT, 6u, 3u, u0, v0, u1, v1);
            // M10: and around the outer level's eye, whose sea set B draws.
            if (drosteOuter) {
                const double latO = BathyModel::kOrgLat + drosteOuterCam[2] / BathyModel::kMPerLat;
                const double lonO = BathyModel::kOrgLon + drosteOuterCam[0] / BathyModel::kMPerLon;
                const float ou0 = float(std::clamp(mU(lonO - dLon), 0.0, 1.0));
                const float ou1 = float(std::clamp(mU(lonO + dLon), 0.0, 1.0));
                const float ov0 = float(std::clamp(mV(latO + dLat), 0.0, 1.0));
                const float ov1 = float(std::clamp(mV(latO - dLat), 0.0, 1.0));
                if (ou1 > ou0 && ov1 > ov0) resMgr.Want(exposureT, 6u, 3u, ou0, ov0, ou1, ov1);
            }
            if (opt.resTrace && (frame % 150u) == 0u) {
                // The instrument (--res-trace): what the page holds at the camera vs what
                // the node says, and what the manager believes about the tiles under it.
                const float uc = float(std::clamp(mU(lonC), 0.0, 1.0));
                const float vc = float(std::clamp(mV(latC), 0.0, 1.0));
                Log("[exposure] frame %u cam (%.4f, %.4f): page z14 resident mip %u, node "
                    "%.2f, tree %s",
                    frame, latC, lonC, resMgr.ResidentMipAt(exposureT, 6u, uc, vc),
                    exposureSrc->At(latC, lonC), std::atomic_load(exposureTree.get())->Id().c_str());
                for (uint32_t mm = 3; mm <= 5; ++mm) {
                    const uint32_t dim = 16384u >> mm;
                    const TileRequest tq{6u, mm, uint32_t(uc * dim) / 256u, uint32_t(vc * dim) / 128u};
                    Log("[exposure]   mip %u tile (%u,%u): %s", mm, tq.x, tq.y,
                        resMgr.DebugTile(exposureT, tq).c_str());
                }
            }
        }
    }
    PROF_END(11);
    if (terrain) terrain->waterNavd = static_cast<float>(waterNavd);
    if (globe) globe->waterNavd = static_cast<float>(waterNavd);   // M6j materials

    // M9o/M9t: the ENGINE's cost. dt (the loop interval) also carries the recorder's
    // bill -- PNG encode, or readback plus the encoder pipe -- so both are kept:
    // renderMs judges the engine, dt judges the capture.
    //
    // AND A CAVEAT THAT COST ME A WRONG NUMBER. RenderFrame only RECORDS and submits;
    // without a fence, the GPU is still working when the clock stops. In a captured run
    // that GPU time was real but hidden -- DumpRaw calls WaitIdle, so execution was
    // being paid for inside the READBACK and renderMs read as pure CPU submit. I
    // reported 1.67 ms as the frame cost on that basis, which flattered it.
    //
    // --bench closes the fence here instead, so the number includes execution and is
    // the honest answer to "what does a frame cost". It is deliberately NOT on by
    // default: a per-frame WaitIdle destroys the CPU/GPU overlap a real run depends on,
    // which would make the captured runs measure something nobody ships.
    const float preMs =
        std::chrono::duration<float>(Clock::now() - preT0).count() * 1000.0f;
    const auto rf0 = Clock::now();
    // --gpu-time rows are labelled like the [rail] series: a rail's 150 settle frames
    // are negative and excluded from the statistics.
    renderer.gpuFrameLabel =
        static_cast<int64_t>(frame) - (opt.rail.empty() ? 0 : 150);
    // --settle-sync reads the ring gate's hold count for THIS frame's wants here;
    // ProcessQueues zeroes it inside RenderFrame.
    // (M9bi's sun placement moved to the top of the frame's housekeeping in M10: the
    // globe's level table carries the sun, and the walk that fills it runs before here.)
    const uint32_t ringHeldBefore = resMgr.ringHeldFrame;
    renderer.RenderFrame(cam, static_cast<float>(simUnix - startUnix), dt);
    // --bench-overlap keeps the overlap: RENDER is then record + the BeginFrame fence
    // wait, and the loop mean is the pipelined max(CPU, GPU) a player's frame costs.
    if (opt.bench && !opt.benchOverlap) gpu.WaitIdle();
    const float renderMs =
        std::chrono::duration<float>(Clock::now() - rf0).count() * 1000.0f;
    // The CPU brackets inside RenderFrame (see profInMs): read, accumulate, zero.
    std::array<float, kInN> inMs{};
    for (int k = 0; k < kInPhases; ++k) {
        inMs[k] = static_cast<float>(resMgr.phaseMs[k]);
        resMgr.phaseMs[k] = 0.0;
    }
    inMs[kInResTurn] = static_cast<float>(resMgr.turnMs);
    resMgr.turnMs = 0.0;
    if (waterBank) {
        inMs[kInBankList] = static_cast<float>(waterBank->tileListMs);
        waterBank->tileListMs = 0.0;
    }
    if (globe) {
        inMs[kInMeshCopy] = static_cast<float>(globe->meshletCopyMs);
        globe->meshletCopyMs = 0.0;
    }
    if (profOn) {
        for (int k = 0; k < kInN; ++k) {
            profInMs[k] += inMs[k];
            if (profHelm) profInHelmMs[k] += inMs[k];
        }
    }

    if (!opt.rail.empty() && frame >= 150u && opt.bench) {
        // Bench records the timing and nothing else -- no readback, no encode, no disk.
        railMs.push_back(renderMs);
        loopRowOpen = true;   // closed by the next iteration's interval
        railPreMs.push_back(preMs);
        railPool.push_back(PoolCommittedBytes());
        railInMs.push_back(inMs);
    } else if (!opt.rail.empty() && frame >= 150u) {
        if (recPipe.Open()) {
            uint32_t rowPitch = 0;
            if (renderer.DumpRaw(recPixels, &rowPitch)) {
                recPipe.Write(recPixels, rowPitch);
            }
        } else {
            wchar_t rp[512];
            swprintf(rp, 512, L"%s\\rail_%04u.png", opt.rail.c_str(), frame - 150u);
            renderer.DumpPng(rp);
        }
        // Two clocks, because they answer different questions. renderMs is the engine
        // alone; the loop interval (closed by the next iteration) includes this frame's
        // PNG encode, which at 1600x900 is tens of milliseconds of the recorder's own
        // cost. A rail reported on it would look three times slower than what it shows.
        railMs.push_back(renderMs);
        loopRowOpen = true;
        railPreMs.push_back(preMs);
        railPool.push_back(PoolCommittedBytes());
        railInMs.push_back(inMs);
    }

    if (!opt.headless &&
        std::chrono::duration<float>(now - lastTitle).count() > 0.25f) {
        lastTitle = now;
        wchar_t title[512];
        FormatTitle(title, 512, simUnix, timeScale, paused, model, *tide,
                    windowSec / 86400.0, sea, seaState,
                    gulf ? gulf->validation.c_str() : nullptr,
                    globe ? globe->stats.c_str() : nullptr,
                    (mode == 1 && altV > 60000.0) ? 3 : mode);   // title by altitude
        window.SetTitle(title);
    }

    if (frame >= 10) {   // skip warm-up: PSO/upload stalls are not frame cost
        frameMsSum += dt * 1000.0;
        ++frameMsN;
    }
    ++frame;
    // --dump-both: the solid frame has just been rendered. Dump it, flip BOTH
    // surfaces to wireframe, and take one more lap -- the clock is held above, so
    // the second image is the same instant seen as lines.
    if (opt.dumpBoth && !opt.dump.empty() && !dumpedSolid && opt.frames &&
        frame == opt.frames + (opt.rail.empty() ? 0u : 150u)) {
        renderer.DumpPng(opt.dump);
        dumpedSolid = true;
        if (globe) globe->surfaceDebug = 1;
        if (sea) sea->wireframe = true;
        return true;
    }
    if (opt.frames &&
        frame >= opt.frames + (opt.rail.empty() ? 0u : 150u) +
                     (opt.dumpBoth ? 1u : 0u)) {
        // --settle-sync: the frame just rendered is the dump frame's instant. Judge it
        // AFTER its RenderFrame (ProcessQueues has mapped, copied and re-uploaded the
        // residency maps for everything that landed this frame): nothing queued, no
        // DirectStorage batch awaiting its fence, and no request the ring gate held
        // this frame (a held child is admitted only once its parent maps, so a quiet
        // queue can refill next frame). Quiet for kEvictAgeFrames consecutive frames
        // -- one full overlap window -- before the image is taken. The still is then
        // the same bytes whichever frame the far tiles landed on.
        //
        // --settle-hold N holds the same instant for exactly N frames instead, and the
        // two compose: drain first, then to at least N. THE RULE THAT KEPT IT is named
        // in the exit line, because the two are not interchangeable -- a drain-judged
        // hold runs as long as the residency makes it (211 and 225 frames on two runs
        // of one binary at the bird), and two stills held for different numbers of
        // frames are not like for like until the churn is frozen through the hold
        // (it is, below).
        if ((opt.settleSync || opt.settleHold || opt.settleExact) && !opt.dump.empty() &&
            opt.headless && !opt.dumpBoth) {
            const uint32_t pend = resMgr.PendingCount();
            const uint32_t reads = resMgr.InFlightReads();
            const bool quiet = pend == 0 && reads == 0 && ringHeldBefore == 0;
            if (!settling) {
                settling = true;
                settlePending0 = pend;
                settleReads0 = reads;
                // Step 25: the manager's exact turn runs on every held frame from the
                // next one on (this frame's turn was the shipped one) and never
                // outside a hold.
                resMgr.settleExact = opt.settleExact;
                // --settle-clear-churn: the whole atlas onto the clear list; the next
                // held frame's clear dispatch zeroes it and the freeze keeps it so.
                if (opt.settleClearChurn && sea) {
                    Log("[settle-clear-churn] %u resident churn tiles onto the clear "
                        "list at the first held frame",
                        sea->ClearChurn());
                }
            }
            settleQuiet = quiet ? settleQuiet + 1 : 0;
            // --settle-exact: the turn just taken found the resident set equal to the
            // want set (Residency.h settleExact) -- for kSettleExactFrames turns
            // running, one overlap window past the last drop's NULL map.
            const ResidencyManager::SettleTurn& ex = resMgr.settleTurn;
            // M12 step 1d, THE ONE INSTRUMENT CHANGE OF THE MOTION (MEASURED 2026-09-13): the
            // exact hold and the wave prefill raced. The hold could end while a background prefill
            // was still in flight, so two runs of one binary dumped with different wave.field
            // mapped sets (the globe pose; the bird pose's two-state churn). The hold now also
            // requires that no prefill is in flight.
            const bool prefillIdle = !wavePrefillBusy.load(std::memory_order_acquire);
            settleExactQuiet =
                (opt.settleExact && ex.exact && prefillIdle) ? settleExactQuiet + 1 : 0;
            // The drain's give-up cap bounds the DRAIN, never the counted hold: a run
            // asked for N frames gets N whatever the residency is doing.
            const bool drainHolds =
                opt.settleSync && settleQuiet < kSettleQuietFrames &&
                settleFrames < kSettleCapFrames;
            const bool exactHolds =
                opt.settleExact && settleExactQuiet < kSettleExactFrames &&
                settleFrames < kSettleCapFrames;
            const bool countHolds = settleFrames < opt.settleHold;
            if (drainHolds || exactHolds || countHolds) {
                ++settleFrames;
                if (settleFrames % 150u == 0u) {
                    Log("[settle-sync] +%u frames at the held instant: pending %u, "
                        "in-flight reads %u, ring-held %u, pool %.0f MB",
                        settleFrames, pend, reads, ringHeldBefore,
                        PoolCommittedBytes() / 1048576.0);
                    if (opt.settleExact) {
                        Log("[settle-exact] +%u: wanted %u, mapped %u, deficit %u, "
                            "unreachable %u, stale %u, dropped this turn %u, retiring "
                            "%u, exact for %u turns",
                            settleFrames, ex.wanted, ex.mapped, ex.deficit,
                            ex.unreachable, ex.stale, ex.dropped, ex.retiring,
                            settleExactQuiet);
                    }
                }
                return true;
            }
            if (opt.settleExact) resMgr.LogSettleExact(settleFrames);
            // MEASURED (helm still, 14:00, 2026-09-05): with the predicted walk running
            // through the hold, pending hit 0 within 300 frames yet was never quiet four
            // frames running in 3000; with it suspended (this code), 149 tiles stay
            // pending from +300 frames on with the pool at 236 MB -- below the 512 MB
            // cap, so they are not eviction victims; which parent they wait on is the
            // next probe (--res-trace through the hold). With the walk off for the WHOLE
            // run the hold converged in 217-226 frames. Every held pair agreed to
            // 2-13 px, the same band as two unheld runs (5-18 px): the horizon-line
            // floor is not the residency's, and 'none' at the helm is still open (P16).
            //
            // Step 25, --settle-exact, MEASURED (2026-09-05, out/step25): the 149 were
            // stale pending tiles (wanted on the approach, not at the pose) and the
            // exact turn drops them; the helm reaches EXACT in 243-252 held frames
            // (11367 wanted = 11367 mapped), the bird in 214-216 (9176 -- a want set
            // 984 tiles OVER the shipped pool cap, so the drain-judged bird was a race:
            // two binaries' --settle-sync birds differed by 21.7 % of the pixels), the
            // globe in 619-737, key7km in 228-253. Two exact runs of one binary: bird
            // 0 px, globe 0 px (at different hold lengths), --lens bed at the bird
            // 0 px, key7km 8 px at |d| 1, helm 65 px all on rows 430-432 (the horizon
            // line, max |d| 15) with every tenant's mapped-set hash equal -- P16 is not
            // the residency's. helm_ebb (19:30): 17 % of the water, 10 % with the churn
            // cleared at the hold (--settle-clear-churn): the churn's pre-hold history
            // is part of it and a second field integrated over the real frames is the
            // rest; the still is exact in residency and not yet in history.
            const char* rule =
                opt.settleExact
                    ? (opt.settleHold ? "exact, then --settle-hold" : "--settle-exact, judged")
                : opt.settleSync
                    ? (opt.settleHold ? "drain, then --settle-hold" : "--settle-sync, judged")
                    : "--settle-hold, counted";
            const bool drained = !opt.settleSync || settleQuiet >= kSettleQuietFrames;
            const bool exact = !opt.settleExact || settleExactQuiet >= kSettleExactFrames;
            Log("[settle-sync] dump frame held %u extra frames at the same instant (%s): "
                "pending %u -> %u, in-flight reads %u -> %u, quiet for %u frames, exact "
                "for %u turns, pool %.0f MB%s%s",
                settleFrames, rule, settlePending0, pend, settleReads0, reads, settleQuiet,
                settleExactQuiet, PoolCommittedBytes() / 1048576.0,
                drained ? ""
                        : " -- CAP REACHED, the residency never drained (a pending set "
                          "that stopped shrinking with the pool below its cap is waiting "
                          "on parents no walk asks for); dumping anyway",
                exact ? ""
                      : " -- CAP REACHED, the resident set never matched the want set "
                        "(see the [settle-exact] ledger above); dumping anyway");
        }
        // The last frame's loop interval: measured here rather than at a next
        // iteration that never comes.
        if (loopRowOpen) {
            railLoopMs.push_back(
                std::chrono::duration<float>(Clock::now() - last).count() * 1000.0f);
            loopRowOpen = false;
        }
        // The encoder gets its end-of-stream BEFORE the metrics print, so the mp4 is
        // complete and closed by the time the numbers describing it appear.
        recPipe.Close();

        // ---- M9o: THE RECORDING'S OWN COST, reported with the recording.
        //
        // Percentiles, not a mean. A rail runs from orbit to helm height, crossing
        // every residency regime the engine has, so its frame times are deliberately
        // NOT a single distribution -- averaging them describes no moment that ever
        // happened. p95 and max are what a viewer perceives as stutter; the worst
        // frame's INDEX says where in the flight it happened, which is the half that
        // makes it actionable (a spike at frame 600 is the 80 km -> 7 km descent
        // paging in, a spike at 1100 is helm-height water detail, and they have different
        // fixes).
        if (!railMs.empty()) {
            std::vector<float> sorted = railMs;
            std::sort(sorted.begin(), sorted.end());
            auto pct = [&](double q) {
                const size_t i = (std::min)(sorted.size() - 1,
                                            size_t(q * double(sorted.size() - 1) + 0.5));
                return sorted[i];
            };
            double sum = 0.0;
            size_t worstI = 0;
            for (size_t i = 0; i < railMs.size(); ++i) {
                sum += railMs[i];
                if (railMs[i] > railMs[worstI]) worstI = i;
            }
            const double mean = sum / double(railMs.size());
            Log("[rail] %zu frames RENDER%s: mean %.2f ms (%.1f fps), p50 %.2f, "
                "p95 %.2f, p99 %.2f, max %.2f ms at frame %zu",
                railMs.size(),
                opt.benchOverlap
                    ? " (CPU record + BeginFrame fence; the GPU overlaps -- OVERLAP bench)"
                    : (opt.bench ? " (record + serialized GPU: fenced bench)" : ""),
                mean, mean > 0.0 ? 1000.0 / mean : 0.0, pct(0.50), pct(0.95), pct(0.99),
                double(railMs[worstI]), worstI);
            if (!railLoopMs.empty()) {
                double lsum = 0.0;
                for (float m : railLoopMs) lsum += m;
                const double lmean = lsum / double(railLoopMs.size());
                // In bench there is no capture at all, so the gap is the REST OF THE
                // LOOP -- sim clock, weather residency, solver advance, scene watch.
                // Calling that "capture overhead" would have been a second wrong label
                // on the same line.
                Log("[rail] loop mean %.2f ms%s, so %.2f ms/frame outside RenderFrame (%s)",
                    lmean,
                    opt.benchOverlap ? " = the shipped pipelined frame, max(CPU, GPU)"
                                     : "",
                    (lmean > mean) ? (lmean - mean) : 0.0,
                    opt.bench ? "sim, residency and solver -- nothing is captured"
                              : (recPipe.Open() ? "readback + encode: the recorder's bill"
                                                : "PNG encode and disk"));
            }
            // THE BUDGET IS THE WHOLE FRAME, not RenderFrame. Counting renderMs
            // against 16.7 ms reported "0/1200 over budget" while the helm phase was
            // actually running a 17.2 ms LOOP -- the sim, residency and solver work
            // outside RenderFrame is about half the cost at low altitude, and a budget
            // that ignores half the frame is a budget that always passes.
            const std::vector<float>& budgetSrc =
                railLoopMs.empty() ? railMs : railLoopMs;
            uint32_t over33 = 0, over16 = 0;
            for (float m : budgetSrc) {
                if (m > 33.3f) ++over33;
                if (m > 16.7f) ++over16;
            }
            Log("[rail] budget (FULL FRAME): %u/%zu over 16.7 ms (%.1f%%), %u over "
                "33.3 ms (%.1f%%)",
                over16, budgetSrc.size(), 100.0 * over16 / double(budgetSrc.size()),
                over33, 100.0 * over33 / double(budgetSrc.size()));
            {
                double tot = 0.0, totH = 0.0;
                for (int k = 0; k < kProfN; ++k) { tot += profMs[k]; totH += profHelmMs[k]; }
                const double n = double(railMs.size());
                const double nH = 240.0;   // the helm leg, 8 s at 30 fps
                Log("[rail] outside RenderFrame, per frame -- %-22s %8s %8s",
                    "section", "whole", "helm");
                for (int k = 0; k < kProfN; ++k) {
                    Log("[rail]   %-22s %6.3f ms %6.3f ms", kProfName[k],
                        profMs[k] / n, profHelmMs[k] / nH);
                }
                if (globe && walkFrames) {
                    // NOT just Want(): the bracket spans the whole leaf EMIT, which
                    // also carries the window-rect Mercator math (nine CubeDir corners
                    // with asin/atan2/log/tan each). Naming it "Want()" would credit
                    // the map for trig it never touched.
                    Log("[rail]   MORPH PROBE: %.0f%% of leaves are FULLY morphed "
                        "(k==1, half density), %.0f%% partially",
                        100.0 * double(morphFullAcc) / (std::max)(1.0, double(walkLeavesAcc)),
                        100.0 * double(morphPartAcc) / (std::max)(1.0, double(walkLeavesAcc)));
                    Log("[rail]   walk: %.0f nodes, %.0f leaves per frame; leaf emit "
                        "(Want + window rects) is %.3f ms of the %.3f ms SetView "
                        "(%.0f%%)",
                        double(walkNodesAcc) / double(walkFrames),
                        double(walkLeavesAcc) / double(walkFrames),
                        double(walkWantNsAcc) / double(walkFrames) / 1e6,
                        profMs[7] / n,
                        100.0 * (double(walkWantNsAcc) / double(walkFrames) / 1e6) /
                            (std::max)(1e-6, profMs[7] / n));
                    const double tch = double(wantTouchAcc) / double(walkFrames);
                    const double hit = double(wantHitAcc) / double(walkFrames);
                    Log("[rail]   Want(): %.0f tile touches/frame, %.0f already tracked "
                        "(%.1f%% repeat) -- %.1f touches per leaf",
                        tch, hit, 100.0 * hit / (std::max)(1.0, tch),
                        tch / (std::max)(1.0, double(walkLeavesAcc) /
                                                  double(walkFrames)));
                }
                if (globe && globe->predictWalks) {
                    // Step 5: the prefetch walk's own bill, per walk (every
                    // kPredictEvery frames, all frames of the run). The join wait is
                    // what the worker did NOT hide; the replay is the Want() calls
                    // the walk used to make inline. The hash is the stream itself.
                    const double w = double(globe->predictWalks);
                    Log("[rail]   prefetch walk (%s): %.0f walks of %.0f nodes / %.0f "
                        "leaves / %.0f rects; the walk %.3f ms, the main thread waited "
                        "%.3f ms at the join and replayed in %.3f ms, per walk (one in "
                        "%u frames); predicted stream %llu Want calls, FNV-1a %016llx",
                        opt.predictInline ? "inline, --predict-inline" : "on the worker",
                        w, double(globe->predictNodes) / w,
                        double(globe->predictLeaves) / w,
                        double(globe->predictRects) / w,
                        double(globe->predictWalkNs) / w / 1e6,
                        double(globe->predictWaitNs) / w / 1e6,
                        double(globe->predictReplayNs) / w / 1e6, kPredictEvery,
                        static_cast<unsigned long long>(resMgr.predictedCalls),
                        static_cast<unsigned long long>(resMgr.predictedHash));
                }
                Log("[rail]   %-22s %6.3f ms %6.3f ms  <- measured here",
                    "sum of the twelve", tot / n, totH / nH);
                if (!railPreMs.empty()) {
                    double pre = 0.0, preH = 0.0;
                    for (size_t k = 0; k < railPreMs.size(); ++k) {
                        pre += railPreMs[k];
                        if (k >= 960) preH += railPreMs[k];
                    }
                    Log("[rail]   %-22s %6.3f ms %6.3f ms  <- ALL of it, so the "
                        "remainder is post-render",
                        "pre-RenderFrame total", pre / n, preH / nH);
                }
                // The CPU inside RenderFrame, by owner. The residency turn's phases
                // sum to its total less the untimed stats string; the descent's p95
                // (frames 450-899, the 80 km -> 7 km paging) is probe P5's number.
                Log("[rail] inside RenderFrame (CPU record), per frame -- %-22s %8s %8s",
                    "section", "whole", "helm");
                Log("[rail]   %-34s %6.3f ms %6.3f ms", "residency turn (ProcessQueues)",
                    profInMs[kInResTurn] / n, profInHelmMs[kInResTurn] / nH);
                for (int k = 0; k < kInPhases; ++k) {
                    Log("[rail]     %-32s %6.3f ms %6.3f ms",
                        ResidencyManager::PhaseName(k), profInMs[k] / n,
                        profInHelmMs[k] / nH);
                }
                Log("[jobs] %llu jobs submitted, stream FNV-1a %016llx%s",
                    static_cast<unsigned long long>(ga::Threads().JobsSubmitted()),
                    static_cast<unsigned long long>(ga::Threads().JobsHash()),
                    ga::Threads().Inline() ? " (--jobs-inline)" : "");
                // Step 28: the landing ledger over the run (Residency.h TurnLedger).
                Log("[rail]     landing: %llu turns landed tiles with no batch behind "
                    "them (drawn without a barrier before step 28), %llu ring fills "
                    "without bytes, %llu claims for a coordinate the tile no longer "
                    "owned",
                    static_cast<unsigned long long>(resMgr.landedOnlyTurns),
                    static_cast<unsigned long long>(resMgr.ringNoBytesTotal),
                    static_cast<unsigned long long>(resMgr.landedUnownedTotal));
                Log("[rail]   %-34s %6.3f ms %6.3f ms", "waterbank tile list (CPU)",
                    profInMs[kInBankList] / n, profInHelmMs[kInBankList] / nH);
                Log("[rail]   %-34s %6.3f ms %6.3f ms", "globe meshlet memcpy",
                    profInMs[kInMeshCopy] / n, profInHelmMs[kInMeshCopy] / nH);
                if (railInMs.size() > 899) {
                    std::vector<float> turn;
                    float turnMax = 0.0f;
                    size_t turnAt = 450;
                    for (size_t k = 450; k < 900; ++k) {
                        turn.push_back(railInMs[k][kInResTurn]);
                        if (railInMs[k][kInResTurn] > turnMax) {
                            turnMax = railInMs[k][kInResTurn];
                            turnAt = k;
                        }
                    }
                    std::sort(turn.begin(), turn.end());
                    Log("[rail]   residency turn on the descent (frames 450-899): p50 "
                        "%.3f, p95 %.3f, max %.3f ms at frame %zu",
                        turn[size_t(0.50 * double(turn.size() - 1) + 0.5)],
                        turn[size_t(0.95 * double(turn.size() - 1) + 0.5)], turnMax,
                        turnAt);
                }
            }
            Log("[rail] tile pool at end: %.2f GB committed across every atlas -- the "
                "sparse structure's real cost for this flight",
                railPool.empty() ? 0.0 : railPool.back() / 1073741824.0);

            // The per-frame series, next to the frames it describes. A summary answers
            // "was it smooth"; only the series answers "where did it stop being smooth",
            // and that question is the reason to record at all.
            std::string csv;
            for (const wchar_t* w = opt.rail.c_str(); *w; ++w) {
                csv.push_back(static_cast<char>(*w));   // rail dirs are ASCII
            }
            csv += "\\metrics.csv";
            if (FILE* f = nullptr; fopen_s(&f, csv.c_str(), "w") == 0 && f) {
                // loop_ms is the frame's OWN loop interval (closed by the next
                // iteration); pre_ms the CPU before RenderFrame; resq_* the residency
                // turn inside it by phase (header order = ResidencyManager::PhaseName).
                fprintf(f, "frame,render_ms,render_fps,loop_ms,pool_bytes,pre_ms,resq_ms,"
                           "resq_retire,resq_dsland,resq_sortseen,resq_loads,"
                           "resq_sortload,resq_gather,resq_map,resq_dsenq,resq_ring,"
                           "resq_resmap,bank_list_ms,meshlet_copy_ms\n");
                for (size_t i = 0; i < railMs.size(); ++i) {
                    fprintf(f, "%zu,%.4f,%.2f,%.4f,%llu,%.4f", i, double(railMs[i]),
                            railMs[i] > 0.0f ? 1000.0 / double(railMs[i]) : 0.0,
                            i < railLoopMs.size() ? double(railLoopMs[i]) : 0.0,
                            static_cast<unsigned long long>(
                                i < railPool.size() ? railPool[i] : 0),
                            i < railPreMs.size() ? double(railPreMs[i]) : 0.0);
                    const std::array<float, kInN> zero{};
                    const std::array<float, kInN>& in =
                        i < railInMs.size() ? railInMs[i] : zero;
                    fprintf(f, ",%.4f", double(in[kInResTurn]));
                    for (int k = 0; k < kInPhases; ++k) fprintf(f, ",%.4f", double(in[k]));
                    fprintf(f, ",%.4f,%.4f\n", double(in[kInBankList]),
                            double(in[kInMeshCopy]));
                }
                fclose(f);
                Log("[rail] per-frame series -> %s", csv.c_str());
            }
        }

        // M7j: THE HYPERVISOR -- one sample, every transformation, each hop tagged
        // with the AST edge it exercises. CPU-derivable steps print values; fields
        // that live only on the GPU print their frame contract and where to look.
        if (pixArmed) {
            // The .wpix is serialized on a PIX background thread after the capture
            // frames PRESENT, so wait for the file to stop growing before exiting.
            //
            // MEASURED (M9b): under --headless it never grows at all. PIX's
            // PIXGpuCaptureNextFrames counts FRAME BOUNDARIES, and a frame boundary
            // is IDXGISwapChain::Present -- which Gpu::EndFrame only calls when a
            // swapchain exists (Gpu.cpp:206). Headless renders to an offscreen
            // target and never presents, so the capturer arms, sees zero frames, and
            // leaves a ~1 KB stub. Bail out loudly instead of waiting on it: GPU
            // captures need a WINDOWED run today. Making headless captures work
            // means the non-frame-based PIXBeginCapture/PIXEndCapture pair, which
            // this build does not load.
            uint64_t last = 0;
            int stable = 0, empty = 0;
            for (int s = 0; s < 60 && stable < 2 && empty < 3; ++s) {
                Sleep(1000);
                struct _stat64 st;
                const uint64_t sz =
                    (_stat64("gagame.wpix", &st) == 0) ? st.st_size : 0;
                stable = (sz > 4096 && sz == last) ? stable + 1 : 0;
                empty = (sz <= 4096) ? empty + 1 : 0;
                last = sz;
            }
            if (last <= 4096) {
                Log("[pix] gagame.wpix is a %llu-byte STUB -- nothing was captured. "
                    "A GPU capture needs frame boundaries (Present); --headless has "
                    "no swapchain. Re-run WINDOWED with --pix.",
                    static_cast<unsigned long long>(last));
            } else {
                Log("[pix] gagame.wpix settled at %.1f MB",
                    last / (1024.0 * 1024.0));
            }
        }
        if (opt.dumpFibers && waterBank) waterBank->DumpFibers(gpu);
        if (opt.dumpWater && !marsMode && sea) {
            tools::RunDumpWaterState(opt, gpu, sea, simUnix, weather);
        }
        if (opt.twinSurface && !marsMode && sea && waterBank) {
            tools::RunTwinSurface(opt, gpu, seaState, sea, waterScene, waterBank, cam,
                                  simUnix, weather, waveField);
        }
        if (opt.trace && !marsMode && sea && waterBank) {
            tools::RunTrace(opt, gpu, sea, compositor, hgtCh, waterAtlas, waterBank, globe,
                            resMgr, winOrgX, winOrgY, hgtTenant, hgtWinTenant, simUnix,
                            weather);
        }
        return false;
    }
    return true;
}

int FrameLoop::Finish() {
    const Options& opt = m_opt;
    auto& window = m_A.window;
    auto& gpu = m_A.gpu;
    auto& renderer = m_A.renderer;
    auto& sea = m_A.sea;
    auto& compositor = m_A.compositor;
    auto& sceneWatch = m_A.sceneWatch;
    auto& globe = m_A.globe;
    auto& resMgr = m_A.resMgr;
    auto& frame = m_frame;
    auto& dumpedSolid = m_dumpedSolid;
    auto& frameMsSum = m_frameMsSum;
    auto& frameMsN = m_frameMsN;

    // M12 step 4d instrument: the [droste] probe's totals over the run -- one line per site
    // (the per-build dumps are above: ProbeDrosteTable, ProbeDive, GlobeLayer::ProbeTransport).
    if (m_portal.Valid()) {
        Log("[droste] probe totals: level table %llu builds (%llu distinct): cam %s | sigma %s | "
            "Q %s | level sun %s | sky up %s | camera sun %s",
            static_cast<unsigned long long>(m_probeTableBuilds),
            static_cast<unsigned long long>(m_probeTableDumps), m_probeCam.Verdict().c_str(),
            m_probeSigma.Verdict().c_str(), m_probeQ.Verdict().c_str(),
            m_probeLevelSun.Verdict().c_str(), m_probeSkyUp.Verdict().c_str(),
            m_probeSun.Verdict().c_str());
        Log("[droste] probe totals: dive rail, live %llu frames: c %s | fw %s | up %s; the sweep: "
            "c %s | fw %s | up %s",
            static_cast<unsigned long long>(m_probeDiveCalls), m_probeDiveC.Verdict().c_str(),
            m_probeDiveFw.Verdict().c_str(), m_probeDiveUp.Verdict().c_str(),
            m_probeSweepC.Verdict().c_str(), m_probeSweepFw.Verdict().c_str(),
            m_probeSweepUp.Verdict().c_str());
        if (globe) globe->LogDrosteProbe();
    }

    // --gpu-time: the per-pass GPU table, next to the [rail] lines it explains. The last
    // frames in flight are still unread; an idle wait drains them before the report.
    if (GpuProfiler* prof = renderer.Profiler()) {
        gpu.WaitIdle();
        prof->Drain();
        prof->Report(opt.rail.empty() ? -1 : 900);
        if (!opt.rail.empty()) {
            std::string csv;
            for (const wchar_t* w = opt.rail.c_str(); *w; ++w) {
                csv.push_back(static_cast<char>(*w));
            }
            csv += "\\gpu_ms.csv";
            if (!prof->WriteCsv(csv)) Log("[gpu] could not write %s", csv.c_str());
        }
    }
    if (frameMsN > 30) {
        Log("[perf] mean frame %.2f ms over %u frames (%.0f fps)%s", frameMsSum / frameMsN,
            frameMsN, 1000.0 / (frameMsSum / frameMsN),
            opt.headless ? "" : (gpu.TearingEnabled() ? " [no-vsync, tearing]" : " [vsync]"));
    }
    // Step 5: the predicted request stream's hash, every run (Residency.h): two runs of
    // the same flight that print different hashes asked the manager for different tiles.
    if (resMgr.predictedCalls) {
        Log("[predict] %llu predicted Want calls this run, FNV-1a %016llx (call order "
            "included)",
            static_cast<unsigned long long>(resMgr.predictedCalls),
            static_cast<unsigned long long>(resMgr.predictedHash));
    }
    // Windowed: what the panel actually showed -- presents per refresh beside [perf]'s
    // loop mean, which cannot see a present that was skipped.
    if (!opt.headless) gpu.ReportPresentStats();
    // M7l: THE DEBUG SESSION REPORT -- the free byproducts, printed every run: what the
    // diagram holds, what the compositor did, what streaming did. The gates print their
    // own PASS lines under --selftest; the trace and fibers print theirs when asked.
    Log("[report] ---- session: %zu AST edges (%s) | compose %u painted %u cache-hit | "
        "fetches %u | pending %u ----",
        ga::ast::Edges().size(), ga::ast::Validate() ? "frames hold" : "FLIP FAILURES",
        compositor.painted.load(), compositor.cacheHits.load(),
        resMgr.fetchesThisRun, resMgr.PendingCount());

    if (opt.seaVerify && sea) tools::RunSeaVerify(opt, gpu, sea);
    if (!opt.dump.empty()) {
        if (dumpedSolid) {
            // out.png already holds the solid frame; this pass is the wireframe twin.
            std::wstring wp = opt.dump;
            const size_t dot = wp.find_last_of(L'.');
            wp = (dot == std::wstring::npos) ? wp + L"_wire"
                                             : wp.substr(0, dot) + L"_wire" + wp.substr(dot);
            renderer.DumpPng(wp);
            Log("[dump] wrote %S (solid) + %S (wireframe, same instant)",
                opt.dump.c_str(), wp.c_str());
        } else {
            renderer.DumpPng(opt.dump);
        }
    }
    // The same frame's radiance before the tonemap (tools/imgdiff.py --hdr).
    if (!opt.dumpHdr.empty()) renderer.DumpHdr(opt.dumpHdr);
    if (!opt.dumpMeshlets.empty() && globe) globe->DumpMeshlets(opt.dumpMeshlets);

    gpu.WaitIdle();
    resMgr.Shutdown();
    sceneWatch.Stop();   // cancels the pending directory read and joins (logged)
    renderer.Shutdown();
    gpu.Shutdown();
    window.Destroy();
    ga::threadaudit::Report();   // --thread-audit: what the threads did to the tile files
    ga::Threads().Shutdown();
    Log("done (%u frames)", frame);
    return 0;
}

#undef PROF_BEGIN
#undef PROF_END

}  // namespace ga::app
