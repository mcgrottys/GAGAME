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
#include "sim/WaterTerms.h"
#include "compose/GisMask.h"
#include "compose/HeightStackSource.h"
#include "compose/TileTree.h"
#include "compose/TileArchive.h"
#include "compose/TileIndex.h"
#include "hal/TileStream.h"
#include "compose/ComposeTree.h"
#include "compose/DomainSource.h"
#include "core/CurrentFieldLoader.h"
#include "core/Dome.h"
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
#include "scene/View.h"   // M12 step 5d: the optics' one conversion (View::FovRadOf)
#include "scene/Entity.h"   // M12 step 5e: the hull as a node
#include "scene/Portal.h"   // M12 step 5e: the link as a node
#include "scene/Rail.h"     // M12 step 5e: the rails as data
#include "scene/ViewContext.h"
#include "scene/SeaLayer.h"
#include "scene/SkyLayer.h"
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

namespace {

// ================================================================================================
//  M12 step 5d: A VIEW'S DECLARED EYE, IN THE SESSION'S OWN CALLS. The sugar's NUMBERS -- read by
//  the one parser, in the declared units (scene::ReadPoseSugar) -- go to Camera::SetFromCompass
//  and scene::GlobeCamera exactly as the flags' numbers did, so a scene camera and a flag camera
//  are the same arithmetic and not a second conversion. A view with no `at`, or one written in a
//  spelling this camera cannot take, leaves the engine's default for the mode standing: that is
//  what "absent = the engine decides" means (SceneSchema.h's views section).
// ================================================================================================
bool ReadEye(const SceneView* v, scene::PoseSugar& out) {
    if (!v || !v->hasAt) return false;
    std::string why;
    if (!scene::ReadPoseSugar(v->at, "views." + v->p.name + ".at", out, &why)) {
        Log("[scene] %s -- the view keeps the engine's default", why.c_str());
        return false;
    }
    return true;
}
bool ViewEyeCompass(const SceneView* v, Camera& cam) {
    scene::PoseSugar s;
    if (!ReadEye(v, s) || s.kind != scene::PoseSugar::Kind::Compass) return false;
    cam.SetFromCompass(s.x, s.alt, s.z, static_cast<float>(s.az), static_cast<float>(s.pitch));
    return true;
}
bool ViewEyeOrbit(const SceneView* v, double planetR, Camera& cam) {
    scene::PoseSugar s;
    if (!ReadEye(v, s) || s.kind != scene::PoseSugar::Kind::Orbit) return false;
    cam = s.lookAt ? scene::OrbitPose(s.lat, s.lon, s.alt, s.tLat, s.tLon, planetR)
                   : scene::GlobeCamera(s.lat, s.lon, s.alt, planetR);
    return true;
}

// A tree's counters summed over it and every node under it: painted, read from disk, served from
// its cache, composed, folded, cached composites dropped, and the archive's share -- tiles answered
// as PLACES (DirectStorage reads them) and through a HANDLE (the bytes cross the CPU) -- the
// residency audit's closing lines, Finish().
void TreeTally(TileTree& t, uint64_t (&n)[8]) {
    n[0] += t.painted.load();
    n[1] += t.read.load();
    n[2] += t.hits.load();
    n[3] += t.composed.load();
    n[4] += t.folded.load();
    n[5] += t.dropped.load();
    n[6] += t.arcPlaces.load();
    n[7] += t.arcReads.load();
    for (size_t i = 0; i < t.KidCount(); ++i) TreeTally(t.Kid(i), n);
}

}  // namespace

FrameLoop::FrameLoop(const Options& opt, const Scene& S, Assembly& A)
    : m_opt(opt), m_S(S), m_A(A) {}

int FrameLoop::Run() {
    if (const std::optional<int> rc = Session()) return *rc;
    // THE BOOT'S LENGTH, from the process's creation to the first frame -- the number a wait
    // before the first frame (the solver's bed wait) is judged by.
    {
        FILETIME created{}, exited{}, kernel{}, user{}, now{};
        GetProcessTimes(GetCurrentProcess(), &created, &exited, &kernel, &user);
        GetSystemTimeAsFileTime(&now);
        auto ticks = [](const FILETIME& f) {
            return (static_cast<uint64_t>(f.dwHighDateTime) << 32) | f.dwLowDateTime;
        };
        Log("[boot] the first frame begins %.1f s after the process started",
            double(ticks(now) - ticks(created)) * 1.0e-7);
    }
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
    // A DISTINCT table is counted and compared every time; its rows are PRINTED for the first
    // kProbePrintCap only. A still builds one table, a flight builds one a frame -- and a camera
    // flown by hand or chasing a hull, 20 lines of %.17g a frame through a flushed log, which a
    // playable scene cannot carry (the Haulover portal demo, 2026-09-14). The totals line at the
    // end of the run is the record, over every table.
    static constexpr uint64_t kProbePrintCap = 16;
    const bool dump = fp != m_probeTableFp;
    m_probeTableFp = fp;
    const std::string wSun = UlpWord(sun, sun2, 3, m_probeSun);
    if (dump) {
        ++m_probeTableDumps;
        if (m_probeTableDumps == kProbePrintCap + 1) {
            Log("[droste] table %llu: the level tables are compared on every build from here; their "
                "rows were printed for the first %llu (the totals at the end are over all of them)",
                static_cast<unsigned long long>(m_probeTableDumps),
                static_cast<unsigned long long>(kProbePrintCap));
        }
    }
    const bool print = dump && m_probeTableDumps <= kProbePrintCap;
    if (print) {
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
        if (!print) continue;
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
void FrameLoop::ProbeDive(double f, const double c0[3], const double f0[3], const double up0[3],
                          double c[3], double fw[3], double up[3], bool live, double u) {
    const Space& drosteLeaf = m_portalNode.Cycle();   // M12 step 5e: the node's
    const droste::Portal& portal = m_portalNode.Link();
    drosteLeaf.LevelApply(f, c0, c);
    drosteLeaf.LevelApplyDir(f, f0, fw);
    drosteLeaf.LevelApplyDir(f, up0, up);
    double c2[3], fw2[3], up2[3];   // the portal's closed forms, for the record
    portal.Apply(f, c0, c2);
    portal.ApplyDir(f, f0, fw2);
    portal.ApplyDir(f, up0, up2);
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

// M12 step 5c: the solved wave field at a new window -- the hot-reload's own three calls, moved
// here so the water component can ASK for them without naming the compositor, the atlas, the
// models or the page frame. A changed window rolls the solver's bucket key, so the next Update
// re-solves or hits the cache (core/SceneConfig.h's banner); what does NOT move is the page
// tenant's lattice, bound at construction -- the component logs that when it happens.
void FrameLoop::ReconfigureWaveField(const WaterSceneConfig& cfg) {
    if (!m_waveField || !m_sceneToWaveCfg) return;
    WaveFieldConfig wcfg2 = m_sceneToWaveCfg(cfg);
    m_waveFrame = WaveFieldSource::Align(wcfg2);   // M9bc: the page grid
    m_waveField->Configure(wcfg2, &m_A.compositor, m_A.hgtCh, &m_A.waterAtlas, &m_A.model,
                           m_entSta, m_A.haveCurrents ? &m_A.currents : nullptr, m_wfCtSta);
}

std::optional<int> FrameLoop::Session() {
    // ---- The aliases: one reference per member this body touches, under main()'s names, so
    // what follows is main()'s code unchanged. A closure that captures one of these by
    // reference captures the member it is bound to (CWG 2011) and outlives this call.
    const Options& opt = m_opt;
    const Scene& S = m_S;
    // M12 step 5d: the scene's list elements, looked up ONCE (see FrameLoop.h). The aliases are
    // the MEMBERS, so the lambdas that outlive this call read them safely.
    if (const ScenePortal* p = S.Portal("droste")) {
        m_portalDecl = *p;
        m_portalOn = p->p.enabled;
    }
    // M12 step 5e: THE NODES. The portal node from `portals[droste]` (scene/Portal.h) and
    // one Entity node per `entities` element (scene/Entity.h: its own step state, its own
    // TreeWater), declared here and wired below where the session's objects exist.
    m_portalNode.Declared() = m_portalDecl.p;
    m_portalNode.enabled = m_portalOn;
    if (m_portalDecl.hasTo) m_portalNode.SetDestination(m_portalDecl.p.toLat, m_portalDecl.p.toLon);
    for (const SceneEntity& decl : S.entities) {
        if (decl.p.vessel.empty()) continue;
        scene::PoseSugar es;
        std::string ewhy = "no `at` declared";
        if (decl.hasAt &&
            scene::ReadPoseSugar(decl.at, "entities." + decl.p.name + ".at", es, &ewhy) &&
            es.kind == scene::PoseSugar::Kind::Compass) {
            auto node = std::make_unique<scene::Entity>();
            node->Declared() = decl.p;
            // The bow's heading when the sugar declares `az`; a spawn that does not say one
            // faces north (+z), the heading every hull has always been built at.
            if (decl.at.Get("az")) {
                node->SetSpawn(es.x, es.alt, es.z, es.az);
            } else {
                node->SetSpawn(es.x, es.alt, es.z);
            }
            m_entities.push_back(std::move(node));
        } else {
            // The shim already refuses `--boat` without `--campos` (the 1e9 sentinel spawn); a
            // scene file that declares a vessel without a place gets the same answer, said here.
            Log("[vessel] '%s' declares no spawn in the compass sugar {x, alt, z} (%s) -- not "
                "spawning: a hull with no place is aground at a meaningless depth",
                decl.p.vessel.c_str(), ewhy.c_str());
        }
    }
    if (const SceneView* v = S.Start()) m_startView = *v;
    auto& portalDecl = m_portalDecl.p;
    const bool portalOn = m_portalOn;
    auto& startView = m_startView;
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
    auto& bathyBostonSwe = m_A.bathyBostonSwe;   // HIERARCHY 4.17: the solvers' own grids
    auto& bathySwe = m_A.bathySwe;
    auto& compositor = m_A.compositor;
    auto& hgtCh = m_A.hgtCh;
    auto& waterAtlas = m_A.waterAtlas;
    auto& bathy = m_A.bathy;
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
    const droste::Portal& portal = m_portalNode.Link();   // M12 step 5e: the node's (scene/Portal.h)
    const Space& drosteLeaf = m_portalNode.Cycle();       // M12 step 4d: the Droste tower as a Space::Cycle
    auto& altOf = m_altOf;
    auto& poseMotor = m_poseMotor;
    auto& motorPose = m_motorPose;
    auto& simUnix = m_simUnix;
    auto& simClock = m_simClock;
    auto& vesselReg = m_vesselReg;
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

    // M6g: THREE views, not four -- 0 chart, 1 THE WORLD (estuary and planet, one
    // continuous scene), 2 gulf map. --sea and --globe both open the world; they differ
    // only in the starting camera. Per-frame altitude gates refine world-mode layer
    // enables continuously (sky hands to the limb shell, the FFT sea sheds at height).
    mode = (S.scene.mode == Scene::kWorld && (sea || globe)) ? 1
             : (S.scene.mode == Scene::kGulf && gulf)             ? 2 : 0;
    applyMode = [&](int m) {
        sky->enabled = (m != 1);   // world mode gates the sky per frame by altitude
        tide->enabled = (m == 0);
        if (sea) sea->enabled = (m == 1) && !marsMode && !opt.albedo;
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
    // M12 step 5d: the optics are the START view's, through 5b's own conversion (View.h: the
    // declaration is degrees because that is what a file and --fov say; the Camera holds radians)
    // -- and then the chart view's declared eye, when it declares one (--cam with no mode flag).
    cam.fovY = scene::View::FovRadOf(startView.p.fovY);
    cam.speed = 150.0f;
    ViewEyeCompass(S.View("chart"), cam);

    // Chart and sea each keep their own camera; TAB cycles views and each resumes where you
    // left it. With bathymetry loaded, the sea default is the LITERAL vqview signature shot:
    // standing on the north jetty tip (world 522, 72 -- located in the CUDEM data), az 246
    // back into the inlet.
    if (bathy.Ready()) camSea.SetFromCompass(522.0, 7.0, 72.0, 246.0f, -4.0f);
    else camSea.SetFromCompass(0.0, 12.0, 0.0, 246.0f, -5.0f);
    // M12 step 5d: THE DECLARED EYE OVER THE ENGINE'S DEFAULT. --cam/--campos write into the
    // START view (`scene.view`), so that is read first; a scene whose start view is the orbit (or
    // is not written in the compass sugar at all) leaves the `sea` view's own declaration to say
    // where the world camera stands, which is what merrimack.json's jetty tip is.
    if (!ViewEyeCompass(&startView, camSea)) ViewEyeCompass(S.View("sea"), camSea);
    camSea.fovY = cam.fovY;
    camSea.speed = 30.0f;
    // Globe mode: the planet frame (centre at the origin). Start over the North Atlantic
    // with home in view.
    {
        // M12 step 5a: the four lines that placed the eye are scene::GlobeCamera (the
        // scene's {lat, lon, alt} spelling resolves through the same function); gR is
        // planetR + gAlt inside it, the same sum. M12 step 5d: the engine's default -- over the
        // North Atlantic at 2.1 planet radii -- then the `orbit` view's declared eye, which is
        // where --globe-cam writes and what merrimack.json writes down (34 N, 52 W, 2.1 R).
        camGlobe = scene::GlobeCamera(34.0, -52.0, planetR * 2.1, planetR);
        ViewEyeOrbit(S.View("orbit"), planetR, camGlobe);
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
        // M13 step 2: THE ROOT CHART BECOMES EXACT. The surface's flat chart carries the rows just
        // derived and the planet's radius, so every water reader that asks it for a place gets the
        // point on the sphere the mesh is drawn on instead of the anchor-linear guess (28 m out at
        // 5 km). The linear numbers stay for the consumers that are still charts (the solvers'
        // own domains); nothing here re-derives a convention -- these are the renderer's own rows.
        for (int i = 0; i < 3; ++i) {
            m_A.surface.flat.east[i] = east0[i];
            m_A.surface.flat.up[i] = oDir[i];
            m_A.surface.flat.north[i] = north0[i];
        }
        m_A.surface.flat.planetR = planetR;
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
    if (mode == 1 && S.scene.view == "orbit") cam = camGlobe;
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
    // M12 step 5e: THE PORTAL NODE (scene/Portal.h) runs BuildPortal from its declaration and
    // declares the cycle -- the session's own block, moved verbatim, its two boot lines with
    // it; the frame reads Link() and Cycle() through the aliases above.
    {
        scene::Portal::Observers po;
        po.east = east0;
        po.up = oDir;
        po.north = north0;
        po.planetR = planetR;
        po.compositor = &compositor;
        po.hgtCh = hgtCh;
        po.tangent = &m_tangentSpace;
        po.globe = globe != nullptr;
        po.marsMode = marsMode;
        m_portalNode.Configure(po);
        m_portalNode.Init(gpu);
    }
    // ---- THE GATES (scene/Gateway.h) AND THE PLACES THEY STAND AT AND LEAD TO. The root's own
    // flat frame is a place; every other place a gate names gets ONE tangent space, shared by every
    // gate that names it -- so a body carried to a place stands in the very frame a gate standing
    // there was built in, and a way back is simply another gate.
    {
        const Space::Anchor& rootChart = m_A.surface.flat;
        struct PlaceRef {
            double lat = 0.0, lon = 0.0;
            const Space* space = nullptr;
            const Space::Anchor* chart = nullptr;
        };
        std::vector<PlaceRef> known;
        known.push_back({rootChart.latDeg, rootChart.lonDeg, &m_tangentSpace, &rootChart});
        auto placeAt = [&](bool named, double lat, double lon, const std::string& gate,
                           PlaceRef& out) {
            if (!named) {
                out = known.front();
                return true;
            }
            for (const PlaceRef& k : known) {
                if (k.lat == lat && k.lon == lon) {
                    out = k;
                    return true;
                }
            }
            auto pl = std::make_unique<scene::Place>();
            char nm[96];
            snprintf(nm, sizeof(nm), "place.%.5f,%.5f", lat, lon);
            std::string why;
            if (!pl->Build(m_planetSpace, planetR, lat, lon, nm, &why)) {
                Log("[gate] '%s': %.5f N %.5f E is not a place (%s) -- not built", gate.c_str(),
                    lat, lon, why.c_str());
                return false;
            }
            out = PlaceRef{lat, lon, &pl->space, &pl->chart};
            known.push_back(out);
            m_places.push_back(std::move(pl));
            return true;
        };
        // A GATE'S BOX, AT EITHER END, IS ONE STRUCTURE POSED TWICE (scene/Gateway.h BoxPose): the
        // placement sugar, read the same way for the entry and for the exit. The compass spelling
        // puts it and turns it -- `az` to the heading, `pitch` to the lean, absent meaning no lean,
        // which is why a box with neither stands y-up -- and the motor spelling carries any
        // orientation at all. `toAz` remains the exit's heading when its own sugar names none.
        auto boxPose = [](const JsonValue& sugar, bool has, double azFallback,
                          const std::string& path, Motor& out, std::string* why) {
            if (!has) {
                out = scene::Gateway::BoxPose(0.0, 0.0, 0.0, azFallback, 0.0);
                return true;
            }
            scene::PoseSugar ps;
            if (!scene::ReadPoseSugar(sugar, path, ps, why)) return false;
            if (ps.kind == scene::PoseSugar::Kind::MotorForm) {
                return scene::ResolveMotorSugar(sugar, scene::PoseFrame{}, path, out, why);
            }
            if (ps.kind != scene::PoseSugar::Kind::Compass) {
                if (why) *why = path + ": a box is {x, alt, z, az, pitch} or {motor: {re, du}}";
                return false;
            }
            out = scene::Gateway::BoxPose(ps.x, ps.alt, ps.z, sugar.Get("az") ? ps.az : azFallback,
                                          sugar.Get("pitch") ? ps.pitch : 0.0);
            return true;
        };
        for (const SceneGate& gd : S.gates) {
            if (!gd.p.enabled || marsMode) continue;
            Motor entry, exit;
            std::string gwhy = "no `at` declared";
            if (!gd.hasAt ||
                !boxPose(gd.at, true, 0.0, "gates." + gd.p.name + ".at", entry, &gwhy)) {
                Log("[gate] '%s' needs its box in the compass sugar {x, alt, z, az[, pitch]} (%s) "
                    "-- not built", gd.p.name.c_str(), gwhy.c_str());
                continue;
            }
            std::string twhy;
            if (!boxPose(gd.toAt, gd.hasToAt, gd.p.toAz, "gates." + gd.p.name + ".toAt", exit,
                         &twhy)) {
                Log("[gate] '%s' needs its exit in the compass sugar {x, alt, z[, az, pitch]} (%s) "
                    "-- not built", gd.p.name.c_str(), twhy.c_str());
                continue;
            }
            PlaceRef from, to;
            if (!placeAt(gd.hasFrom, gd.p.fromLat, gd.p.fromLon, gd.p.name, from) ||
                !placeAt(gd.hasTo, gd.p.toLat, gd.p.toLon, gd.p.name, to)) {
                continue;
            }
            auto gate = std::make_unique<scene::Gateway>();
            gate->Declared() = gd.p;
            if (gate->Build(*from.space, *to.space, *to.chart, m_tangentSpace, entry, exit)) {
                m_gates.push_back(std::move(gate));
            }
        }
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
    // M12 step 5e: THE RAILS ARE DATA (scene/Rail.h; scenes/rails/<name>.json). The five hand
    // tables and the lambdas that flew them -- railKeys, railPose, diveFrom/diveAt, legU,
    // diveU, keyedPose, gravityUp, drosteRailPose -- are the files' segments and Rail::At.
    // WHICH FILE is the tables' own selection law: the Mars flyover on Mars; the flood, jetty
    // and zoom rails by name over the bathymetry; the two Droste rails ride the FLOOD keys to
    // the helm (--rail-droste's flag implied the flood table until 5d compared the enum and
    // the fallthrough chose the classic keys -- the data says flood, as M10 wrote it); else
    // the classic flight. A rail is read only when one is recorded; the dive rail is read
    // beside it whenever the portal stands, for the [droste] probe's sweep.
    {
        const bool towerRail = (S.rails.active == Scene::kRailDroste) ||
                               (S.rails.active == Scene::kRailDrosteOut);
        const char* railName =
            (globe && marsMode)                                                    ? "mars"
            : (globe && (S.rails.active == Scene::kRailFlood) && bathy.Ready())    ? "flood"
            : (globe && (S.rails.active == Scene::kRailJetty) && bathy.Ready())    ? "jetty"
            : (globe && (S.rails.active == Scene::kRailZoom) && bathy.Ready())     ? "zoom"
            : (globe && towerRail && bathy.Ready())
                ? ((S.rails.active == Scene::kRailDroste) ? "droste" : "droste-out")
            : (globe && sea && bathy.Ready() && !marsMode)                         ? "classic"
                                                                                   : nullptr;
        // THE FRAME a rail resolves in: the tangent rows and the planet, the portal and its
        // cycle, the dive's declared pace, the session camera's optics, the views a key names.
        m_railFrame = scene::RailFrame{};
        m_railFrame.frame.planetR = planetR;
        for (int i = 0; i < 3; ++i) {
            m_railFrame.frame.east[i] = east0[i];
            m_railFrame.frame.up[i] = oDir[i];
            m_railFrame.frame.north[i] = north0[i];
        }
        m_railFrame.frame.valid = true;
        m_railFrame.fovY = cam.fovY;
        m_railFrame.portal = &portal;
        m_railFrame.cycle = portal.Valid() ? &drosteLeaf : nullptr;
        m_railFrame.twistDeg = portalDecl.twistDeg;
        m_railFrame.levelSec = S.rails.droste.levelSec;
        m_railFrame.levels = S.rails.droste.levels;
        m_railFrame.clampLevels = (S.rails.active == Scene::kRailDroste);
        const Scene* scenePtr = &S;
        m_railFrame.viewAt = [scenePtr](const std::string& name) -> const JsonValue* {
            const SceneView* v = scenePtr->View(name);
            return (v && v->hasAt) ? &v->at : nullptr;
        };
        std::string rwhy;
        if (railName && !S.railDirW.empty()) {
            const std::string path = std::string("scenes/rails/") + railName + ".json";
            if (!m_rail.Load(path, &rwhy) || !m_rail.Resolve(m_railFrame, &rwhy)) {
                Log("FATAL: [rail] %s", rwhy.c_str());
                return 1;
            }
            Log("[rail] %s: %zu segments from %s%s", m_rail.Name().c_str(),
                m_rail.SegmentCount(), path.c_str(),
                m_rail.Tower() ? " (a tower rail: it writes the camera's level and its up)" : "");
        }
        // M12 step 4d instrument: the dive's closed forms compared over the rail's own schedule
        // -- every recorded frame of the dive at 30 fps, u = the dive's clock, tau to the last
        // helm hold -- here, where the helm and its up are final, so every Droste run's log
        // carries the dive's verdict (the rail reaches its spiral 1260 recorded frames in); the
        // live rail compares the same way, one line per dive frame. The schedule is the DIVE
        // RAIL's: its helm pose (aimed at the fixed point, stood off under a small twist) and
        // its law, read from its file when the run flies another rail or none.
        m_diveRail = nullptr;
        if (portal.Valid()) {
            if (m_rail.Loaded() && m_rail.Tower()) {
                m_diveRail = &m_rail;
            } else if (m_diveProbeRail.Load("scenes/rails/droste.json", &rwhy) &&
                       m_diveProbeRail.Resolve(m_railFrame, &rwhy)) {
                m_diveRail = &m_diveProbeRail;
            } else {
                Log("[droste] probe dive rail: scenes/rails/droste.json not read (%s) -- the "
                    "sweep is skipped", rwhy.c_str());
            }
        }
        if (const scene::RailPose* helm = m_diveRail ? m_diveRail->Pose("helm") : nullptr) {
            const int nF =
                static_cast<int>((S.rails.droste.levels * S.rails.droste.levelSec + 2.0) * 30.0);
            for (int i = 0; i <= nF; ++i) {
                const double u = scene::Rail::DiveU(double(i) / 30.0, S.rails.droste.levelSec,
                                                    S.rails.droste.levels,
                                                    S.rails.active == Scene::kRailDroste);
                double c[3], fw[3], up[3];
                ProbeDive(u - std::floor(u), helm->c0, helm->f0, helm->up0, c, fw, up, false, u);
            }
            Log("[droste] probe dive rail (sweep of %d frames' u, %d levels x %.0f s): c %s | fw %s "
                "| up %s",
                nF + 1, S.rails.droste.levels, S.rails.droste.levelSec, m_probeSweepC.Verdict().c_str(),
                m_probeSweepFw.Verdict().c_str(), m_probeSweepUp.Verdict().c_str());
        }
    }
    // (M12 step 5d: --cam / --campos are the START view's `at` now -- applied to the camera the
    // mode picked, above, where the view that declares the eye is the view that starts.)

    // M12 step 5d: `time.start` -- "now" (the sentinel), a civil UTC stamp, or unix seconds;
    // ParseStartTime is the flag's own reader, so the two spellings land on the same instant.
    const double sceneStart = (S.time.start == "now") ? -1.0 : ParseStartTime(S.time.start);
    simUnix = (sceneStart > 0) ? sceneStart : NowUnix();
    // The playable clock (sim/SimClock.h). Headless keeps its own frame-indexed formula
    // below -- that path was already fixed-step, which is why rails reproduce and sessions
    // did not.
    simClock.Reset(simUnix);

    // ==================================================================================
    //  M9bq THE BOAT -- M12 step 5e: an ENTITY node per hull (scene/Entity.h). A vessel
    //  from the registry, stepped on the scene clock's whole quanta against the tree's
    //  own water. Nothing here is a demo path: this is the same Vessel the gates
    //  exercise, the same TreeWater the twin measures, and the same SimClock everything
    //  else in the scene rides.
    // ==================================================================================
    RegisterBuiltinVessels(vesselReg);
    for (auto& e : m_entities) e->Spawn(vesselReg);
    // The chase camera's subject: the entity the start view's `follow` names, or -- when it
    // names none -- the first hull at the helm, which is what a spawned boat has always meant.
    m_followed = nullptr;
    for (auto& e : m_entities) {
        if (!e->Active()) continue;
        if (startView.p.follow.target == e->Name() ||
            (startView.p.follow.target.empty() && !m_followed)) {
            m_followed = e.get();
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
        bathySwe.Ready() ? std::min(20.0, std::abs(bathySwe.WorldX0()) / 1000.0) : 6.0;
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
    // M12 step 5d: `water.swe.westBoundary` (--swe-west-off). Captured BY VALUE below, because
    // the two closures are std::function members that outlive this call.
    const bool westBoundary = S.water.swe.westBoundary;
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
    westAt = [&, westBoundary](double t) {
        if (!westBoundary) return 0.0;
        const double tw = (model.Height(westA, t) - model.S(westA).meanMllwM) * (1.0 - wT) +
                          (model.Height(westB, t) - model.S(westB).meanMllwM) * wT;
        return tw - (model.Height(entSta, t) - model.S(entSta).meanMllwM);
    };
    // M6r: the transport the Flather west boundary must CARRY (+east): river discharge
    // minus the upriver prism demand. The reach beyond the window (km ~15.5 to the head
    // of tide at Haverhill, ~km 35) fills and drains THROUGH this boundary; its surface
    // area is the prism knob (~19.5 km of ~200 m river; an NHD-integrated area is the
    // named refinement). d(eta_west)/dt by central difference of the station-fit clocks.
    westQAt = [&, riverQ, westBoundary](double t) {
        if (!westBoundary) return 0.0;
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
    if (swe.Ready()) weather.AddExternalWindow("merrimack", &swe, &bathySwe, oceanAt);
    if (bathyBostonSwe.Ready()) {
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
            weather.AddDormantWindow("boston", &bathyBostonSwe, bcfg, oceanAtBoston, 0.5);
            // M9ar: Boston lies inside the z14 height page; its solver reads slice 6 too.
            if (hgtTenant >= 0) {
                weather.SetHeightPage(resMgr.TextureRes(hgtTenant),
                                      resMgr.ResidencyRes(hgtTenant), 6u,
                                      resMgr.Mips(hgtTenant), surface.winH);
            }
        }
    }

    if (S.Tool("ocean-probe")) {
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
        c.mapPath = opt.waveMap;   // the tool's own argument (--wave-map / --tool wave-map)
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
            waveField->SetSweCurrent(&swe, &bathySwe, sea->sweCurrentGain);
        }
        // M12 step 5c: the bank's wave field and its live config pointer (and set B's) are
        // applied BELOW, by the water component's one Apply -- the same call the hot-reload
        // makes. Nothing between here and there reads either.
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
                                    waveFrame.nx, waveFrame.ny, double(waveFrame.winPxX),
                                    double(waveFrame.winPxY));
            if (waterBankB) {   // the same pages: one solve, every level's sea
                waterBankB->SetWavePages(resMgr.TextureSrv(waveT), resMgr.ResidencySrv(waveT),
                                         double(waveFrame.orgPxX), double(waveFrame.orgPxY),
                                         waveFrame.nx, waveFrame.ny, double(waveFrame.winPxX),
                                         double(waveFrame.winPxY));
            }
            Log("[wave] wave.field is page tenant %d: %u planes over the z16 window, tiles "
                "exact bytes of the solve, pyramid prefilled per bucket",
                waveT, uint32_t(WaveField::kMaxComp) + 1u);
        }
    }
    // M9bi: --sun pins the pre-ephemeris art direction; sunPlaced stays false and the
    // renderer keeps using these two constants, exactly as every earlier baseline did.
    if (S.sun.source == Scene::kPinned) {
        renderer.sunAzimuthDeg = S.sun.az;
        renderer.sunElevationDeg = S.sun.el;
        Log("[sun] PINNED to azimuth %.1f, elevation %.1f -- the ephemeris is off",
            S.sun.az, S.sun.el);
    }
    if (globe) globe->pixelWater = S.water.pixelWater;   // M9bh: the two-ray water, per pixel
    // M12 step 5c: THE WATER SCENE HAS ONE APPLY (scene/WaterComponent.h). Everything this
    // block used to fan out by hand -- the bank's wave field and its live config pointer, set
    // B's, the globe's four water fields, the AUTO edit floor from the datum envelope, and the
    // sea's four closures -- is the component's Apply, and the hot-reload in Frame() calls the
    // SAME one over a complete resolved candidate. That is the step's deliberate change: before,
    // a reload applied neither the sea's closures nor the auto floor, and it took the jetty
    // floor only above the -90 sentinel.
    scene::WaterComponent::Observers wo;
    wo.config = &waterScene;
    wo.bank = waterBank;
    wo.bankB = waterBankB;
    wo.sea = sea;
    wo.globe = globe;
    wo.waveField = waveField.get();
    wo.atlas = &waterAtlas;
    wo.rebuild = &m_waveRebuild;
    wo.path = m_A.kScenePath;
    wo.watch = &m_A.sceneWatch;
    wo.mtime = &m_A.waterSceneMtime;
    m_A.water.Configure(wo);
    m_A.water.Apply(m_A.water.BootSet());

    // M8 THE FLEET: the AIS traffic lane (harvest_route.py) -- boats are pure
    // f(simUnix) on it (ping-pong at the ends), so scrubbing time scrubs the
    // traffic and headless renders are deterministic. No state anywhere.
    if (waterBank) route.Load("data/gis/route_merrimack.json");

    // M5c: give the solver history before the first frame, and run the validation cycle if
    // asked (headless CSV; the ebb/flood-asymmetry and basin-lag gates read from it).
    if (swe.Ready()) {
        // --bed-trace N: THE BED THE KERNEL READS, read back before the hour is integrated on it
        // (app/Tools/BedTrace.cpp); a run's readings land in a directory of their own under out/.
        const bool bedTrace = opt.bedTraceEvery > 0;
        const double spinT0 = simUnix - S.water.swe.spinupH * 3600.0;
        if (bedTrace) {
            char dir[96];
            snprintf(dir, sizeof(dir), "out/bedtrace/%lld-%lu",
                     static_cast<long long>(std::time(nullptr)),
                     static_cast<unsigned long>(GetCurrentProcessId()));
            m_bedTracer.Configure(bathySwe, dir, [&](uint64_t hist[16]) {
                return weather.ClaimedMips(resMgr, hgtTenant, hist, "merrimack");
            });
            m_bedTracer.Read(gpu, swe, S.water.swe.bedWait ? "before-wait" : "before-spinup",
                             oceanAt(spinT0));
        }
        // THE BED BEFORE THE HOUR (review finding 48). The spin-up integrates on the bed its
        // kernel reads, and before the first frame nothing has asked for any of the domain and
        // no residency turn has run. water.swe.bedWait says what the hour waits for: "whole",
        // every active solver's domain read at mip 0; "map", one residency turn with nothing
        // asked, so the kernel reads the manager's own map (the coarsest mip) -- a diagnostic.
        bool bedWhole = false;
        if (S.water.swe.bedWait == 2 && S.water.swe.spinupH > 0) {
            const WeatherManager::BedWait bw = weather.WaitForBeds(gpu, resMgr, hgtTenant);
            bedWhole = bw.done;
            Log("[swe] bed wait: %u window(s), %u tiles mapped in %u residency turns, %.2f s -- %s "
                "(%llu of %llu cells read the page at mip 0)",
                bw.windows, bw.tiles, bw.turns, bw.seconds,
                bw.done ? "the bed is whole"
                        : bw.windows ? "NOT whole at the bound; the hour spins up on what there is"
                                     : "no domain inside the page to wait for",
                static_cast<unsigned long long>(bw.whole),
                static_cast<unsigned long long>(bw.cells));
            if (bedTrace) m_bedTracer.Read(gpu, swe, "before-spinup", oceanAt(spinT0));
        } else if (S.water.swe.bedWait == 1 && S.water.swe.spinupH > 0) {
            gpu.WaitIdle();
            {
                hal::CommandContext up(gpu, gpu.BeginUpload(), hal::Owner::Upload);
                resMgr.ProcessQueues(gpu, up.Native());
                gpu.EndUpload();
            }
            Log("[swe] bed wait: the residency map only (one turn, nothing asked) -- the hour "
                "integrates on the coarsest resident mip");
            if (bedTrace) m_bedTracer.Read(gpu, swe, "before-spinup", oceanAt(spinT0));
        }
        // A SOLVER'S WINDOW HOLDS ITS WATER: flooded on the bed its kernel reads when the whole-bed
        // wait finished (the trace reads it back), else on the CPU grid, and the line says which.
        swe.LogHoldsWater(gpu, bedWhole);
        // --swe-cycle N: the validation cycle integrates on the bed a run's spin-up would, so
        // it comes after the wait (before it, the kernel reads a bed no turn has delivered).
        if (S.Tool("swe-cycle")) {
            // The west boundary's inputs, for the cycle's west log: the two stations that bracket
            // the boundary's river kilometre (each tide in its own MLLW, less its own mean), the
            // weight between them, and the entrance's tide and datum link to NAVD88.
            auto describeWest = [&](double t) {
                char b[480];
                snprintf(b, sizeof(b),
                         "boundary at river km %.2f: %s (km %.2f) %+.3f less mean %.3f | %s (km %.2f) "
                         "%+.3f less mean %.3f | weight %.3f | entrance %s %+.3f less mean %.3f, "
                         "MLLW->NAVD %+.3f",
                         kWestKm, model.S(westA).name.c_str(), model.S(westA).riverKm,
                         model.Height(westA, t), model.S(westA).meanMllwM,
                         model.S(westB).name.c_str(), model.S(westB).riverKm,
                         model.Height(westB, t), model.S(westB).meanMllwM, wT,
                         model.S(entSta).name.c_str(), model.Height(entSta, t),
                         model.S(entSta).meanMllwM, datumOff);
                return std::string(b);
            };
            // The stations' own tides over the cycle, a row every 120 s like the cycle's: each in
            // its own MLLW less its own mean, the river's stations only (out/swe_stations_<pid>.csv).
            {
                char sp[96];
                snprintf(sp, sizeof(sp), "out/swe_stations_%lu.csv",
                         static_cast<unsigned long>(GetCurrentProcessId()));
                FILE* sf = nullptr;
                fopen_s(&sf, sp, "w");
                if (sf) {
                    fprintf(sf, "unix");
                    for (size_t i = 0; i < model.Count(); ++i) {
                        if (model.S(i).riverKm < 0) continue;
                        std::string nm = model.S(i).name;
                        std::replace(nm.begin(), nm.end(), ',', ' ');
                        fprintf(sf, ",%s@%.2f", nm.c_str(), model.S(i).riverKm);
                    }
                    fprintf(sf, "\n");
                    for (double t = simUnix; t <= simUnix + opt.sweCycleH * 3600.0 + 1.0; t += 120.0) {
                        fprintf(sf, "%.0f", t);
                        for (size_t i = 0; i < model.Count(); ++i) {
                            if (model.S(i).riverKm < 0) continue;
                            fprintf(sf, ",%.4f", model.Height(i, t) - model.S(i).meanMllwM);
                        }
                        fprintf(sf, "\n");
                    }
                    fclose(sf);
                    Log("[swe-cycle] wrote %s", sp);
                }
            }
            return tools::RunSweCycleMode(opt, gpu, currents, haveCurrents, bathySwe, swe, resMgr,
                                          simUnix, oceanAt, southAt, westAt, westQAt, describeWest);
        }
        if (S.water.swe.spinupH > 0) {
            const auto t0 = std::chrono::steady_clock::now();
            swe.Spinup(gpu, simUnix, S.water.swe.spinupH, oceanAt, westAt, southAt, westQAt);
            Log("[swe] spun up %.2f h of history in %.1f s", S.water.swe.spinupH,
                std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count());
        }
        if (bedTrace) m_bedTracer.Read(gpu, swe, "after-spinup", oceanAt(simUnix));
        if (S.Tool("swe-uv")) tools::RunSweUv(opt, gpu, bathySwe, swe);
        {
            // Spot probes for the log: bar, throat, ocean. dEta pathologies show instantly.
            const float pts[6] = {900.0f, 100.0f, 250.0f, 60.0f, 2400.0f, 0.0f};
            SweSolver::Probe pr[3];
            swe.ReadProbes(gpu, pts, 3, pr);
            Log("[swe] probes  bar(900,100): dEta %+.3f u %+.2f,%+.2f v%d | throat(250,60): "
                "dEta %+.3f u %+.2f,%+.2f v%d | ocean(2400,0): dEta %+.3f",
                pr[0].dEta, pr[0].u, pr[0].v, pr[0].valid ? 1 : 0, pr[1].dEta, pr[1].u,
                pr[1].v, pr[1].valid ? 1 : 0, pr[2].dEta);
        }
    }
    timeScale = S.time.timeScale;
    windowSec = S.time.windowDays * 86400.0;
    m_paused = S.time.paused;

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

    if (!S.railDirW.empty()) {
        CreateDirectoryW(S.railDirW.c_str(), nullptr);
        if (!m_rail.Loaded()) {
            Log("FATAL: --rail needs globe + sea + bathy data all present");
            return 1;
        }
    }

    if (S.Tool("warm-inlet") && (winTenant >= 0 || hgtTenant >= 0)) {
        tools::RunWarmInlet(opt, gpu, compositor, resMgr, winTenant, surface.winSlice, hgtTenant,
                            hgtWinTenant);
        // What the warm did to the trees that serve the pages (the compositor's own counters
        // above stay at zero when the trees are the providers, which is the default).
        if (m_A.megaTree) Log("[warm] the colour tree:\n%s", m_A.megaTree->Stats().c_str());
        if (m_A.heightTree) Log("[warm] the height tree:\n%s", m_A.heightTree->Stats().c_str());
    }

    // ---- M6j: channel export mode -- pull data OUT through the manager and exit.
    if (S.Tool("export")) {
        return tools::RunExport(opt, gpu, compositor, hgtCh, resMgr, colCh, surface);
    }

    // ---- The instrumentation block's runtime part; its members and their notes are
    // declared in FrameLoop.h (main.cpp ff2f732 lines 1307..1389).
    last = Clock::now();
    lastTitle = last;
    kPredictEvery = (std::max)(1u, S.streaming.predictEvery);
    if (!S.capture.mp4.empty()) {
        recPipe.Open(S.capture.mp4, renderer.Width(), renderer.Height(), 30);
    }
    if (!S.railDirW.empty()) {
        railMs.reserve(S.capture.frames ? S.capture.frames : 1200);
        railLoopMs.reserve(S.capture.frames ? S.capture.frames : 1200);
        railPool.reserve(S.capture.frames ? S.capture.frames : 1200);
    }

    // M12 step 5e: THE ENTITIES' WATER AND LAYER (scene/Entity.h Observers), wired here where
    // stepBoat used to be assigned -- after the wave field exists, which the lambda captured by
    // reference. The step itself is Entity::Update, from both clock branches (StepEntities).
    for (auto& e : m_entities) {
        scene::Entity::Observers eo;
        eo.weather = &weather;
        eo.waveField = waveField.get();
        eo.sea = sea;
        eo.seaState = &seaState;
        eo.waterScene = &waterScene;
        eo.vesselLayer = vesselLayer;
        eo.gpu = &gpu;
        eo.gates = &m_gates;
        eo.swellShadow = m_A.exposureShadow.get();
        eo.bed = m_A.heightBed.get();
        eo.rootChart = &m_A.surface.flat;   // M13 step 2: places, not the linear chart
        eo.planet = &m_planetSpace;         // ... and the two spaces a hull re-centres between
        eo.rootSpace = &m_tangentSpace;
        e->Configure(eo);
    }

    // ---- M12 step 5f: ARM THE WHOLE-SCENE HOT RELOAD (scene/SceneReload.h).
    //
    // THE CHAIN, NOT THE FILE. 5c watched data/wave_scene.json, which is ONE overlay of a fold
    // whose chain is the base scene, the scene THAT inherits, every `include` in order and the
    // rail this run flies. So the fold is run again here -- deterministically, from the same
    // flags, by the same function the boot and --print-scene use -- to learn which files it
    // actually read, and every one of them gets a watcher. The water's watcher is ADOPTED
    // (Assembly::sceneWatch, already running on that file with its own mtime cell): one watcher
    // per file, and its poll now belongs to the reload rather than to WaterComponent::Reload.
    //
    // THE TARGETS. One per section, taken from the ROOT TABLE'S OWN declaration -- the schema
    // and the offset both -- so this list cannot drift from the file's sections; then one per
    // live node the scene declares. A section with no fanOut is not inert: its Restart keys are
    // REPORTED at reload, which is the whole difference between "not applied" and "ignored".
    {
        scene::SceneBuilder rb;
        SceneArgs ra;
        std::string rwhy2;
        if (!BuildScene(opt, rb, ra, &rwhy2)) {
            Log("[scene] hot reload NOT armed: the boot's own fold does not resolve again (%s)",
                rwhy2.c_str());
        } else {
            m_live = static_cast<const scene::SceneDocument&>(S);
            m_reload.Configure(
                [this](scene::SceneBuilder& b, std::string* why) {
                    SceneArgs a;
                    return BuildScene(m_opt, b, a, why);
                },
                rb.Resolved());
            for (const std::string& f : rb.Files()) {
                const bool isWater = (f == std::string(m_A.kScenePath));
                m_reload.Watch(f, isWater ? &m_A.sceneWatch : nullptr,
                               isWater ? &m_A.waterSceneMtime : nullptr);
            }
            if (m_rail.Loaded() && !m_rail.Path().empty()) m_reload.Watch(m_rail.Path());
            using RT = scene::SceneReload::Target;
            auto section = [this](const char* key, std::function<void()> fan = nullptr,
                                  std::vector<std::string> lists = {}) {
                const scene::PropDecl* d = scene::SceneFileSchema().Find(key);
                if (!d || !d->sub) return;
                RT t;
                t.path = key;
                t.schema = d->sub;
                t.instance = reinterpret_cast<char*>(&m_live) + d->offset;
                t.fanOut = std::move(fan);
                t.lists = std::move(lists);
                m_reload.Add(std::move(t));
            };
            section("scene");   // all Restart: a label, the layer law, the planet, the start view
            section("data");    // every path is read where a file is opened: all Restart
            section("time", [this] {
                m_timeScale = m_live.time.timeScale;
                m_windowSec = m_live.time.windowDays * 86400.0;
                m_paused = m_live.time.paused;
            });
            section("sun", [this] {
                // `source` is Restart (the AST's solar edges are registered from it at boot, so
                // switching it live would make docs/GA_AST.md describe a path the run does not
                // walk); the pinned pair is the renderer's own two floats, read every frame --
                // and inert under the ephemeris, by the declaration's own meaning.
                if (m_live.sun.source == Scene::kPinned) {
                    m_A.renderer.sunAzimuthDeg = m_live.sun.az;
                    m_A.renderer.sunElevationDeg = m_live.sun.el;
                }
            });
            section("sea");     // the storm and the datum are read where the sea state is built
            section("water", [this] { ApplyWater(); }, {"fleet.boats"});
            section("streaming", [this] {
                m_kPredictEvery = (std::max)(1u, m_live.streaming.predictEvery);
            });
            section("capture");   // the dump, the size and the hold are the run's identity
            section("rails");     // the rail is resolved once, against the tower it was built in
            m_poseFrame.planetR = m_A.planetR;
            for (int i = 0; i < 3; ++i) {
                m_poseFrame.east[i] = m_A.surface.east[i];
                m_poseFrame.up[i] = m_A.surface.up[i];
                m_poseFrame.north[i] = m_A.surface.north[i];
            }
            m_poseFrame.valid = true;
            if (!m_startView.p.name.empty()) {
                RT t;
                t.path = "views." + m_startView.p.name;
                t.schema = &scene::ViewSchema();
                t.instance = &m_startView.p;
                t.frame = &m_poseFrame;
                t.fanOut = [this] {
                    // THE OPTICS, through 5b's own conversion (View.h: the declaration is
                    // degrees, the Camera holds radians). The eye itself is Restart: this run's
                    // camera is the session's state, moved by WASD, the rails and the chase cam.
                    m_cam.fovY = scene::View::FovRadOf(m_startView.p.fovY);
                    m_cam.nearZ = m_startView.p.nearZ;
                };
                m_reload.Add(std::move(t));
            }
            if (!m_portalDecl.p.name.empty()) {
                RT t;
                t.path = "portals." + m_portalDecl.p.name;
                t.schema = &scene::PortalSchema();
                t.instance = &m_portalDecl.p;   // `lighting` is read every frame from here
                m_reload.Add(std::move(t));
            }
            for (auto& e : m_entities) {
                RT t;
                t.path = std::string("entities.") + e->Name();
                t.schema = &scene::EntitySchema();
                scene::Entity* ep = e.get();
                t.apply = [ep](const scene::PropSet& p) { ep->Apply(p); };
                m_reload.Add(std::move(t));
            }
            if (const SceneEffect* fx = S.EffectOfType("slice.plane")) {
                // A typed element answers to TWO schemas (SceneBuilder::ElementChain): its own
                // (name, type, enabled) and its type's (d). Two targets, so every key under the
                // element is seen by one of them -- a key covered by neither would be a hole in
                // the report, which is the one thing this must not have. The element's own half
                // is all Restart (`enabled` decides the AST edge at boot), so it REPORTS and never
                // applies: no instance, no apply.
                const std::string fxPath = std::string("effects.") + fx->p.name;
                RT a;
                a.path = fxPath;
                a.schema = scene::SceneFileSchema().Find("effects")->sub;
                m_reload.Add(std::move(a));
                RT b;
                b.path = fxPath;
                b.schema = &scene::SlicePlaneSchema();
                b.apply = [this](const scene::PropSet& p) { m_A.slice.Apply(p); };
                m_reload.Add(std::move(b));
            }
            if (!m_reload.Ready(&rwhy2)) {
                Log("[scene] hot reload NOT armed: %s", rwhy2.c_str());
            }
        }
    }
    return std::nullopt;
}

// M12 step 5f: THE WATER SECTION'S FAN-OUT, both halves. The scene's own three numbers are the
// sea layer's, read every frame from the layer; the wavefield/closures/fleet subtree is 5c's,
// and it goes through WaterComponent::Apply -- the SAME call the boot made -- rather than being
// applied a second way here. `WaterSceneDoc` is the reduction app/Scene.h already declares, so
// the live section and data/wave_scene.json reach that component through one door.
void FrameLoop::ApplyWater() {
    if (SeaLayer* sea = m_A.sea) {
        sea->foamIntensity = m_live.water.foam;
        sea->targetEdgePx = m_live.water.edgePx;
        sea->heightScale = m_live.water.heightScale;
        sea->sweCurrentGain = m_live.water.swe.gain;
    }
    const scene::PropDecl* d = scene::SceneFileSchema().Find("water");
    if (!d || !d->sub) return;
    JsonValue section = scene::PropSet::Defaults(*d->sub, &m_live.water).ToJson();
    // THE BOATS ARE A LIST, which a PropSet does not carry (5a's law) -- ToJson writes it as []
    // -- so they come from the document being applied. Without this line the first reload of
    // ANY file in the chain handed the component an empty fleet.
    if (const JsonValue* boats = scene::SceneReload::At(m_reload.Document(), "water.fleet.boats")) {
        if (JsonValue* fl = scene::JsonGet(section, "fleet")) scene::JsonSet(*fl, "boats", *boats);
    }
    scene::PropSet set(scene::WaterSchema());
    scene::WaterComponent::Fleet fleet;
    std::string why;
    if (!m_A.water.ReadJson(WaterSceneDoc(section), set, fleet, &why)) {
        Log("[scene] water: %s -- the previous water set stands", why.c_str());
        return;
    }
    const int boatsWere = m_A.waterScene.fleetCount;
    m_A.water.StageFleet(fleet);
    m_A.water.Apply(set);
    // The instrument the [water] apply fingerprint cannot be: that hash is a PropSet's, and a
    // PropSet carries no list, so an emptied fleet would read as an unchanged set.
    Log("[scene] water fleet: %d boats (was %d)%s", m_A.waterScene.fleetCount, boatsWere,
        m_A.waterScene.fleetCount == boatsWere ? "" : " -- the fleet MOVED");
}

// M12 step 5e: THE ENTITIES' STEP, from both clock branches (stepBoat's contract: the quanta
// COUNT differs between the windowed and the headless clock and nothing else does). The
// snapshot the hulls read is declared to them (FrameInfo: the clock, its quanta, and asOf --
// the instant the solver's delivered answers are coherent at); each entity's step is its own;
// the hull steps' wall time lands in the profiler's slot 11 as stepBoat's bracket did; and the
// chase camera is the start view's Follow of the followed hull, placed after the steps as the
// hand code placed it after the telemetry (and, as there, only once the hull is set down).
void FrameLoop::StepEntities(int quanta, float dt) {
    scene::FrameInfo fi;
    fi.simUnix = m_simUnix;
    fi.dt = dt;
    fi.frame = m_frame;
    fi.quanta = quanta;
    fi.asOf = m_weather.SolverAsOf();
    for (auto& e : m_entities) {
        if (!e->Active()) continue;
        e->Update(fi);
        if (m_profOn) {
            m_profMs[11] += e->LastStepMs();
            if (m_profHelm) m_profHelmMs[11] += e->LastStepMs();
        }
    }
    // ---- THE CHASE CAMERA. Gravity-up and roll-free by choice: a camera that heels
    // with the hull reads as the WORLD rolling, which is nauseating and is not what a
    // helmsman's inner ear reports. The motor-native view that DOES heel is the
    // first-person one, later. (scene/View.h Follow: the start view's four numbers.)
    if (m_followed && m_followed->Helming() && m_followed->Placed()) {
        // ---- THE EYE DOES NOT GO THROUGH WITH ITS SUBJECT (M13). A gate carries the hull the
        // instant its centre of gravity enters the box; the eye trails it by `follow.back` and
        // is still outside. Teleporting the eye with it was the wrong body obeying the rule --
        // what a helmsman's camera should do is keep watching the boat THROUGH the window and
        // follow it in. So the eye owes that gate until the box takes the eye too.
        if (m_followed->Carries() != m_followCarries) {
            m_followCarries = m_followed->Carries();
            if (const scene::Gateway* g = m_followed->LastGate()) {
                if (m_eyeOwes.size() < kMaxEyeOwes) {
                    m_eyeOwes.push_back(g);
                    Log("[gate] '%s' went through '%s'; the eye stays on this side and follows it "
                        "through the window", m_followed->Name(),
                        g->Declared().name.c_str());
                } else {
                    // A subject that outruns its camera through four windows has left it behind:
                    // the eye takes the last one whole rather than growing an unbounded chain.
                    Log("[gate] the eye is %zu windows behind '%s' -- it crosses to catch up",
                        m_eyeOwes.size(), m_followed->Name());
                    m_eyeOwes.clear();
                }
            }
        }
        // The pull-back: every window the eye still owes, outermost first. A window's motor
        // inverted (Gateway::Window, said in the root frame wherever the gate stands) is the map its
        // far side's geometry is drawn by, so a chase put through it lands where the window shows it.
        Motor pull = Motor::Identity();
        for (const scene::Gateway* g : m_eyeOwes) pull = pull * g->Window().Inverse();
        double p[3], f[3];
        m_followed->ChaseFrame(p, f);
        if (m_followed->InSpace() == nullptr && m_eyeOwes.empty()) {
            scene::View::Follow(m_startView.p.follow, p, f, m_cam);
        } else {
            // A hull in another space: the same chase, said in ITS frame (y is its up), and the
            // eye and the aim carried into the root's frame by that space's placement -- then
            // pulled back through the windows the eye has not yet walked into.
            const scene::FollowProps& fw = m_startView.p.follow;
            double eye[3] = {p[0] - f[0] * fw.back, p[1] + fw.up, p[2] - f[2] * fw.back};
            double aim[3] = {p[0], p[1] + fw.aimLift, p[2]};
            if (const Space* sp = m_followed->InSpace()) {
                (void)sp;
                const Motor& W = m_followed->SpaceInRoot();
                W.TransformPoint(eye[0], eye[1], eye[2]);
                W.TransformPoint(aim[0], aim[1], aim[2]);
            }
            pull.TransformPoint(eye[0], eye[1], eye[2]);
            pull.TransformPoint(aim[0], aim[1], aim[2]);
            // THE EYE CROSSES BY THE HULL'S OWN RULE: its centre inside the box -- the box where
            // the root frame has it, wherever the gate stands. Every ray from inside the box
            // already starts in the window (scenetest [gate]), so the picture does not move -- the
            // eye simply stops looking through the window and stands in the place it was looking at.
            if (!m_eyeOwes.empty()) {
                const scene::Gateway* g = m_eyeOwes.front();
                if (g->InsideRoot(eye[0], eye[1], eye[2])) {
                    m_eyeOwes.erase(m_eyeOwes.begin());
                    const Motor& Kw = g->Window();
                    Kw.TransformPoint(eye[0], eye[1], eye[2]);
                    Kw.TransformPoint(aim[0], aim[1], aim[2]);
                    Log("[gate] the eye reached '%s' and went through: it is standing in %s now, "
                        "and stops marching", g->Declared().name.c_str(),
                        g->Destination().name.c_str());
                }
            }
            m_cam.px = eye[0];
            m_cam.py = eye[1];
            m_cam.pz = eye[2];
            m_cam.LookAt(aim[0], aim[1], aim[2]);
        }
        // THE CHASE EYE'S LEVEL IS THE HULL'S. A camera belongs to the ground its SUBJECT would
        // fall onto: the free eye is its own subject and re-roots with the Droste gauge; a chase
        // eye's subject is the hull, which lives in the root's flat frame -- as do the vessel
        // layer, the markers and the rings drawn around it. So the eye it places is a root eye
        // (level 0), exactly as a tower rail writes the level of the pose it samples; the gauge
        // step below does not move it. Found building the Haulover portal demo (2026-09-14):
        // with a portal in the scene, a chase eye within a few metres of the inner globe
        // re-rooted, the hull vanished from its own frame, and the next frame placed a root
        // eye under an inner level's index.
        m_camLevel = 0;
    }
    // (What each hull is drawn through is decided with the view's windows: PublishHulls.)
}

scene::ViewCone FrameLoop::ViewConeOf(const Camera& cam, float aspect, float viewH) const {
    scene::ViewCone v;
    v.eye[0] = cam.px;
    v.eye[1] = cam.py;
    v.eye[2] = cam.pz;
    DirectX::XMFLOAT3 f, r, u;
    cam.ViewBasis(f, r, u);   // THE render basis: the frame the rays are rasterized in
    const DirectX::XMFLOAT3* src[3] = {&f, &r, &u};
    double* dst[3] = {v.fwd, v.right, v.up};
    for (int k = 0; k < 3; ++k) {
        dst[k][0] = src[k]->x;
        dst[k][1] = src[k]->y;
        dst[k][2] = src[k]->z;
    }
    v.tanY = std::tan(0.5 * double(cam.fovY));
    v.tanX = v.tanY * double(aspect);
    v.pixTan = 2.0 * v.tanY / (std::max)(double(viewH), 1.0);
    return v;
}

std::vector<const scene::Gateway*> FrameLoop::GateList() const {
    std::vector<const scene::Gateway*> out;
    for (const auto& g : m_gates) {
        if (g && g->Valid()) out.push_back(g.get());
    }
    return out;
}

void FrameLoop::PublishHulls() {
    VesselLayer* layer = m_A.vesselLayer;
    if (!layer) return;
    std::vector<const Vessel*> hulls;
    std::vector<Motor> frames;
    std::vector<uint8_t> depths;
    for (const auto& e : m_entities) {
        const Vessel* hull = e->Hull();
        if (!hull) continue;
        const Motor base = e->DrawFrame();
        hulls.push_back(hull);
        frames.push_back(base);
        depths.push_back(0u);
        for (size_t k = 0; k < m_windows.size(); ++k) {
            hulls.push_back(hull);
            frames.push_back(m_windows[k].pull * base);
            depths.push_back(static_cast<uint8_t>(k + 1));
        }
    }
    layer->SetVessels(hulls.data(), frames.data(), depths.data(), static_cast<int>(hulls.size()));
}

bool FrameLoop::Frame() {
    const Options& opt = m_opt;
    const Scene& S = m_S;
    auto& portalDecl = m_portalDecl.p;
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
    auto& swe = m_A.swe;
    auto& gulf = m_A.gulf;
    auto& waterScene = m_A.waterScene;
    auto& waterBank = m_A.waterBank;
    auto& waterBankB = m_A.waterBankB;
    auto& globe = m_A.globe;
    auto& planetR = m_A.planetR;
    auto& resMgr = m_A.resMgr;
    // M13: THE VIEW'S OWN SAMPLER on the one earth cache. The globe's walk registers the same
    // name, so a view's walk, its prefetch and the loop's page wants for the same eye are ONE
    // reader -- the thing a reserve is written against -- while subjects and a gate's window
    // name their own below.
    const int sampView = resMgr.Sampler("view");
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
    const droste::Portal& portal = m_portalNode.Link();   // M12 step 5e: the node's (scene/Portal.h)
    const Space& drosteLeaf = m_portalNode.Cycle();       // M12 step 4d: the Droste tower as a Space::Cycle
    auto& camLevel = m_camLevel;
    auto& altOf = m_altOf;
    auto& poseMotor = m_poseMotor;
    auto& motorPose = m_motorPose;
    auto& rail = m_rail;   // M12 step 5e: the rail as data (scene/Rail.h)
    auto& simUnix = m_simUnix;
    auto& simClock = m_simClock;
    auto& wavePending = m_wavePending;
    auto& wavePrefillDone = m_wavePrefillDone;
    auto& wavePrefillBusy = m_wavePrefillBusy;
    auto& wavePendingKey = m_wavePendingKey;
    auto& wavePendingTiles = m_wavePendingTiles;
    auto& wavePendingPlanes = m_wavePendingPlanes;
    auto& wavePendingSec = m_wavePendingSec;
    // M12 step 5e: the hulls are entity nodes; the helm -- the chase camera's subject -- is
    // the followed entity's state, and a hull aboard is any entity that spawned.
    const auto helming = [this] { return m_followed && m_followed->Helming(); };
    bool anyHull = false;
    for (auto& e : m_entities) anyHull = anyHull || e->Active();
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
    auto& loggedLevel = m_loggedLevel;

    if (!S.capture.headless) {
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
    profOn = S.railDirW.empty() || frame >= 150u;
    dt = (dt > 0.25f) ? 0.25f : dt;   // a debugger break must not teleport time

    if (!S.capture.headless) {
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
        if (!helming()) cam.Update(in, dt);
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
            for (auto& e : m_entities) e->ResetAtRest();
        }
        // A boat at 100x time is not a simulation of anything: the tide and current
        // buckets roll every few frames, and each roll costs an 8-11 s wave solve plus
        // a 12-19 s page prefill, which is what reads as a freeze. Time scaling stays
        // available with no hull aboard.
        if (anyHull && timeScale > 10.0) {
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
            // M12 step 5e: THE ENTITIES -- the helm controller reads the keys (Entity::Helm,
            // the keyboard block verbatim), the step is Entity::Update (stepBoat), and the
            // chase camera is the start view's Follow of the followed hull (StepEntities).
            for (auto& e : m_entities) {
                if (e->Active()) e->Helm(in, dt);
            }
            StepEntities(simSteps, dt);
        }
    } else {
        // Deterministic time in headless mode so a dump sequence is reproducible.
        // M7h: rail SETTLE -- the first recorded frame used to be the coldest:
        // the height window still streaming, the classifier reading coarse
        // fallback and calling half the channel LAND (the flat grey panels at
        // t=0). The rail now holds its opening pose for kRailSettle unrecorded
        // frames so residency, the solver mirror, and the composed caches are
        // warm before the camera rolls.
        const uint32_t settle = S.railDirW.empty() ? 0u : 150u;
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
        // decides which). The held instant is S.capture.frames - 1, the LAST frame an unheld
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
        if ((opt.dumpBoth || settling) && S.capture.frames && recFrame >= S.capture.frames) {
            recFrame = S.capture.frames - 1u;
        }
        simUnix = startUnix + static_cast<double>(recFrame) * (timeScale / 30.0);
        // The headless clock is frame-indexed, so a frame is worth exactly
        // timeScale/30 seconds of world; the boat owes that many whole quanta. Rounding
        // rather than truncating keeps the owed time from drifting slow over a long
        // rail, and at the default scale it is an exact 8.
        StepEntities(static_cast<int>(std::lround((timeScale / 30.0) / SimClock::kDt)), dt);
        // The churn atlas is stateful and its kernel only climbs at a frozen dt, so a
        // held frame would advance the foam the hold's length decides. Freeze it for
        // exactly the held frames (SeaLayer.h freezeChurn).
        if (sea) sea->freezeChurn = settling;
        // The helm leg of --rail-flood: keys at 32 s (cHelmIn) and 40 s (cHelmGap).
        profHelm = !S.railDirW.empty() && recFrame >= 32u * 30u;
        if (!S.railDirW.empty() && rail.Loaded()) {
            // M6g: the rails just set a pose in the ONE frame. Nothing switches.
            // Helming outranks the rail: the chase camera has already placed the
            // view on the boat this frame and a rail pose would yank it away. This is
            // what makes `--rail-flood --boat` a chase-cam recording rather than a
            // flypast that happens to contain a hull.
            // M10: the Droste rails leave the keys at the helm for the similarity's
            // own spiral, and say which level the pose is written in.
            // M12 step 5e: the rail is data (scene/Rail.h). A TOWER rail flown through a
            // valid portal writes the camera's level and its up (drosteRailPose's contract);
            // any other flies its keys (railPose's). AimCamera is the one rasterizer
            // boundary, and the [droste] probe reads the sample's own spiral beside it.
            if (!helming()) {
                const double t = static_cast<double>(recFrame) / 30.0;
                const bool tower = rail.Tower() && portal.Valid();
                const scene::RailSample rs = tower ? rail.At(t) : rail.KeysAt(t);
                scene::Rail::AimCamera(rs, cam);
                if (tower) {
                    camLevel = rs.level;
                    if (rs.upWritten) {
                        for (int i = 0; i < 3; ++i) drosteUp[i] = rs.up[i];
                        drosteRailUp = true;
                    }
                    if (rs.spiral && rs.base && rs.upOf) {
                        double c[3], fw[3], up[3];
                        ProbeDive(rs.f, rs.base->c0, rs.base->f0, rs.upOf->up0, c, fw, up, true,
                                  rs.u);
                    }
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
    if (S.sun.source != Scene::kPinned) {
        // THE EARTH AROUND A LIGHT AT THE ORIGIN. A scene that places the Earth (sun.earth)
        // gets exactly that placement, every frame, whatever the clock says -- the tide and the
        // sun are not coupled. Only the ephemeris source reads the time.
        const sun::SolarSystem ss =
            (S.sun.source == Scene::kEarth)
                ? sun::BuildEarth(S.sun.earth.at, S.sun.earth.axis, S.sun.earth.spin)
                : sun::Build(simUnix);
        m_solar = ss;
        m_solarValid = true;
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
            Log("[sun] placed %s: subsolar %.3f N %.3f E, %.6f AU "
                "(%.1f W/m2), angular radius %.4f deg; from here azimuth %.2f, "
                "elevation %+.2f  [--sun 112,26 pins the pre-M9bi art direction]",
                (S.sun.source == Scene::kEarth) ? "by the scene (the Earth around a light at 0,0,0)"
                                                : "from the ephemeris",
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
        // (The chase eye is the hull's and stays at its level: StepEntities.)
        for (int guard = 0; guard < 4 && !helming(); ++guard) {
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
        if (portalDecl.lighting == 0) drosteLeaf.LevelApplyDir(-double(camLevel), sr, sc);
        for (int i = 0; i < 3; ++i) sunCamF[i] = static_cast<float>(sc[i]);
        double sc2[3] = {sr[0], sr[1], sr[2]};   // the portal's, for the record
        if (portalDecl.lighting == 0) portal.ApplyDir(-double(camLevel), sr, sc2);
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
            if (portalDecl.lighting == 0) drosteLeaf.LevelApplyDir(-double(camLevel + rel), sr, sk);
            for (int i = 0; i < 3; ++i) L.sun[i] = static_cast<float>(sk[i]);
            for (int i = 0; i < 3; ++i) {
                pr.sun[i] = sr[i];    // the portal's, for the record (below)
                pr.sun2[i] = sk[i];   // the read
            }
            if (portalDecl.lighting == 0) portal.ApplyDir(-double(camLevel + rel), sr, pr.sun);
            // THE SKY IT SEES. Realistic: every level inside the root sits a few
            // hundred metres up in the root's air, so the sky over it is the ROOT's --
            // its zenith turned into this level's frame, lit by the root's day -- and a
            // night-side inner sea mirrors that bright sky. Appealing: its own sky.
            if (portalDecl.lighting == 0 && camLevel + rel > 0) {
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
        // lighting inside the tower; its own everywhere else -- and its own zenith is the
        // planet's radial AT THE EYE (ZenithAt), which +y is only at the tangent origin.
        {
            float upC[3] = {0.0f, 1.0f, 0.0f};
            float dayC = -1.0f;
            if (portal.Valid() && mode == 1 && portalDecl.lighting == 0 && camLevel > 0) {
                const double upR[3] = {0.0, 1.0, 0.0};
                double su[3];
                portal.ApplyDir(-double(camLevel), upR, su);
                for (int i = 0; i < 3; ++i) upC[i] = static_cast<float>(su[i]);
                dayC = static_cast<float>(std::clamp(double(sunRootF[1]) * 3.0 + 0.12, 0.0, 1.0));
            } else if (mode == 1) {
                const double C[3] = {cam.px, cam.py, cam.pz};
                double z[3];
                ZenithAt(C, planetR, z);
                for (int i = 0; i < 3; ++i) upC[i] = static_cast<float>(z[i]);
            }
            globe->SetCamSky(upC, dayC);
        }
        if (portal.Valid() && mode == 1) {
            globe->SetDroste(drosteLv.data(), static_cast<int>(drosteLv.size()),
                             portalDecl.lighting, portal.centre, portal.radius, camLevel);
        }
    }
    // ---- THE VIEW'S WINDOWS, AND THE SEA BEYOND THE FIRST (scene/Gateway.h WindowChain). The
    // chain is found here, where the eye has its pose, so the second ring set can follow the eye
    // the first window carries -- set B, the bank the Droste outer level uses, when no Droste outer
    // level already holds it. With no window in view its rings still stand where the nearest gate
    // within reach would carry the eye -- anchored and mapped, not filled (the fill is its readers',
    // below) -- so a window the eye turns toward finds them in place. The worlds themselves
    // are added beside SetView, below, re-measured after the camera's last clamp.
    m_windows.clear();
    if (!drosteOuter && camLevel == 0 && mode == 1 && globe && !m_gates.empty()) {
        const float viewHw = S.capture.headless ? static_cast<float>(S.capture.height)
                                                : static_cast<float>(std::max(1u, window.Height()));
        const float aspectW =
            (S.capture.headless ? static_cast<float>(S.capture.width) : window.Width()) / viewHw;
        m_windows = scene::WindowChain(GateList(), m_eyeOwes, ViewConeOf(cam, aspectW, viewHw),
                                       kMaxWindowChain, kWindowReachM);
        double E[3] = {cam.px, cam.py, cam.pz};
        if (!m_windows.empty()) {
            m_windows.front().carry.TransformPoint(E[0], E[1], E[2]);
            drosteOuter = true;
        } else {
            double best = kWindowReachM;
            const scene::Gateway* nearest = nullptr;
            for (const auto& gp : m_gates) {
                if (!gp->Valid()) continue;
                double bx = 0.0, by = 0.0, bz = 0.0;
                gp->EntryInRoot().TransformPoint(bx, by, bz);
                const double dx = bx - cam.px, dy = by - cam.py, dz = bz - cam.pz;
                const double dist = std::sqrt(dx * dx + dy * dy + dz * dz);
                if (dist >= best) continue;
                best = dist;
                nearest = gp.get();
            }
            if (nearest) {
                nearest->Window().TransformPoint(E[0], E[1], E[2]);
                drosteOuter = true;
            }
        }
        if (drosteOuter) {
            for (int i = 0; i < 3; ++i) drosteOuterCam[i] = E[i];
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
            if (portalDecl.lighting == 0) {
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
        // THE CAMERA'S OWN DOME stands on the zenith at the eye (DomeFrame, above): the rows carry
        // that zenith onto +y and the sun rides the same rotor. The identity at the tangent origin;
        // 18.5 degrees for a camera the gate carried to Haulover, whose dome stood on the
        // Merrimack's zenith -- a slab of below-horizon grey over the sea.
        float zenSun[3];
        if (domeRel == 0 && mode == 1) {
            const double C[3] = {cam.px, cam.py, cam.pz};
            double u[3];
            ZenithAt(C, planetR, u);
            DomeFrame(u, skySun, rows, zenSun);
            skySun = zenSun;
        }
        sky->SetSkyFrame(rows, skySun);
        // M13: AND THE AIR ITSELF. The air's one table (the same for every ray on the planet) and
        // this eye's distance from the planet's centre; the sky is marched from there. An eye
        // above the air marches nothing -- the limb shell owns that backdrop (the atmosphere
        // ledger) -- and every consumer answers with the gradient it always had; the shader
        // decides that from the radius, per viewpoint, so a re-rooted Droste level (local scale
        // ~3e6 m) keeps the sky it drew before.
        {
            const double C[3] = {cam.px, cam.py, cam.pz};
            const double gy = C[1] + planetR;
            sky->SetPlanetRadius(planetR);
            renderer.skyMsSrv = sky->MultiScatterSrv();
            renderer.planetRadiusM = static_cast<float>(planetR);
            renderer.eyeRadiusM = static_cast<float>(std::sqrt(C[0] * C[0] + gy * gy + C[2] * C[2]));
        }
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
            // ...and the up is the SUBJECT's too (StepEntities' law): a chase eye keeps the root
            // planet's radial set above -- the hull's gravity -- rather than the field of the
            // grounds around the EYE, which beside the inner globe tipped the horizon ~30 degrees
            // while the hull ran level on its own sea (the Haulover portal demo's chase A/B).
            if (gn > 1e-30 && !helming()) {
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
            const float vh = S.capture.headless
                                 ? static_cast<float>(S.capture.height)
                                 : static_cast<float>(std::max(1u, window.Height()));
            gisLayer->tolMeters = static_cast<float>(altV * cam.fovY / vh);
        }
        // M6x: the weather manager's residency clock -- the camera's ground position
        // is the demand signal; dormant windows spin up as it arrives, owned solvers
        // advance. All in the flat one-world frame. (The CPU mirrors no longer
        // refresh here: nothing in the loop reads them -- step 2 of PERF_EXPERIMENT.)
        if (!marsMode) {
            PROF_BEGIN();
            weather.Update(gpu, renderer.Shaders(), S.shadersW, simUnix,
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
            // M12 step 5c: THE RELOAD LAW, whole, in the component (scene/WaterComponent.h):
            // the watcher's poll, the mtime compare, the file resolved into a COMPLETE
            // candidate over the DECLARED DEFAULTS (a key the file no longer carries reverts to
            // its default, not to the value this run was carrying), validated with the refusing
            // key's path, and the accepted diff applied through THE SAME Apply the boot used.
            // The two deliberate changes are here: the sea's four closures and the auto edit
            // floor now reload, because there is only one set left to apply.
            // M12 step 5f: THE WHOLE SCENE, not one file (scene/SceneReload.h). The poll is
            // one atomic exchange per watched file; the stat and the re-resolution run only
            // when a directory said the file moved, exactly as 5c's did. The water reaches
            // WaterComponent::Apply through the `water` target's fan-out, so the reload law
            // 5c wrote is called from here and not duplicated.
            PROF_BEGIN();
            m_reload.Poll(frame);
            PROF_END(1);
            // M8: bucket-watch + background solve + upload/swap for the solved
            // wave field, BEFORE the bank recomposes so the kernel binds a whole
            // field or the previous one -- never a half-written atlas.
            // (frame >= 2: the first frames run on the boot clock before --start
            // settles; solving them caches a real answer for the wrong instant.)
            if (waveField && sea && waterScene.wfEnabled && frame >= 2) {
                PROF_BEGIN();
                waveField->Update(gpu, simUnix, sea->Parts(), sea->activeParts,
                                  S.capture.headless);
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
                    uint32_t tx0, ty0, tx1, ty1;   // the window in whole mip-0 tiles
                    waveSrc->WindowTiles(tx0, ty0, tx1, ty1);
                    wavePrefillBusy.store(true, std::memory_order_release);
                    const ColorFrame wf = waveFrame.color;
                    WaveFieldSource* wsrc = waveSrc.get();
                    const uint32_t planes = wavePendingPlanes;
                    Threads().Submit(Lane::Compute, "wave.prefill",
                                     [&wavePending, &wavePrefillDone, &wavePendingTiles,
                                      &wavePendingSec, wf, wsrc, planes, tx0, ty0, tx1, ty1]() {
                        const auto tp0 = Clock::now();
                        auto fresh = std::make_shared<TileTree>(wsrc, TileTree::Fmt::Raw4);
                        wavePendingTiles = fresh->Prefill(wf, 0u, planes, 0u, tx0, ty0, tx1, ty1);
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
                    const double vhW = S.capture.headless
                                           ? double(S.capture.height)
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
                        resMgr.Want(sampView, waveT, 6u + p, wantMip, u0, v0, u1, v1);
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
                                waterBank->BaseTexelM(), orgs, S.water.oneWater);
            // M13 step 2: the cascade sea's plane AT THE EYE, for the pixel stage's sub-ring
            // bands -- the same plane the bank's texels were filled from. A chart cell is
            // hundreds of kilometres across, so one frame's pixels sit inside one.
            {
                WaveChart wcEye;
                double dEye[3];
                const bool onEye = m_A.surface.flat.Exact() &&
                                   m_A.surface.flat.DirOfProjected(cam.px, cam.pz, dEye);
                globe->SetWaveChartFrame(onEye ? wcEye.CellAt(dEye).f[0] : WaveChart::Frame{},
                                         onEye);
            }
            // M10: set B follows the OUTER level's eye, S(C) -- the sea the camera's
            // planet floats in, seen from where the camera really is in that level's
            // own frame -- or the eye a gate carries (the view's windows, above). Its rings
            // STAND wherever such an eye is: anchored and mapped here, so the frame a reader
            // arrives in fills them in place. Whether they are FILLED is the readers' to say,
            // once the level table is final (below, before the tide).
            if (waterBankB) {
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
        const bool rootSky = portal.Valid() && portalDecl.lighting == 0 && camLevel > 0;
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
        if (globe && portal.Valid() && portalDecl.lighting == 1 && !marsMode && !opt.albedo) {
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
        const float viewH = S.capture.headless ? static_cast<float>(S.capture.height)
                                         : static_cast<float>(
                                               std::max(1u, window.Height()));
        const float aspect =
            (S.capture.headless ? static_cast<float>(S.capture.width) : window.Width()) / viewH;
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
        // ---- THE VIEW'S WINDOWS (scene/Gateway.h WindowChain). Each link of the chain is a world
        // the view reaches through the gates: the SAME planet walked once more from the eye carried
        // that far, its geometry drawn back by the chain's rotation (the Droste level at scale 1:
        // eye, sigma 1, Q), walked only along the rays through its windows, and kept per pixel by
        // the ordered slab test. So each is the world behind a pane of glass -- from any angle, with
        // the eye's own parallax -- and a corridor of windows within windows is the same rule asked
        // again. Nothing moves and nothing is copied; every walk's wants go to the one resident set,
        // charged to its gate. The chain found above is re-measured here, after the camera's last
        // clamp; none while the eye stands in a Droste level.
        {
            const scene::ViewCone view = ViewConeOf(cam, aspect, viewH);
            if (camLevel == 0 && !m_windows.empty()) {
                std::vector<const scene::Gateway*> seq;
                for (const scene::WindowLink& l : m_windows) seq.push_back(l.gate);
                m_windows = scene::WindowChain(GateList(), seq, view, static_cast<int>(seq.size()),
                                               kWindowReachM);
            } else {
                m_windows.clear();
            }
            const int n = static_cast<int>(m_windows.size());
            if (n > 0) {
                GlobeLayer::DrosteLevel levels[kMaxWindowChain];
                WindowBox boxes[kMaxWindowChain];
                float upWin[kMaxWindowChain * 4] = {};
                float sunWin[kMaxWindowChain * 3] = {};
                const double C[3] = {cam.px, cam.py, cam.pz};
                double E1[3] = {C[0], C[1], C[2]};
                m_windows.front().carry.TransformPoint(E1[0], E1[1], E1[2]);
                auto within = [](const double a[3], const double b[3]) {
                    const double d[3] = {a[0] - b[0], a[1] - b[1], a[2] - b[2]};
                    return d[0] * d[0] + d[1] * d[1] + d[2] * d[2] < kWindowReachM * kWindowReachM;
                };
                double zE1[3] = {0.0, 1.0, 0.0}, sunE1[3] = {0.0, 1.0, 0.0};
                for (int k = 0; k < n; ++k) {
                    const scene::WindowLink& W = m_windows[size_t(k)];
                    const Motor& Gm = W.pull;   // this world -> the true frame
                    GlobeLayer::DrosteLevel& L = levels[k];
                    L.rel = k + 1;
                    double E[3] = {C[0], C[1], C[2]};
                    W.carry.TransformPoint(E[0], E[1], E[2]);   // the eye, carried this far
                    for (int i = 0; i < 3; ++i) L.cam[i] = E[i];
                    L.sigma = 1.0;
                    for (int c = 0; c < 3; ++c) {
                        double v[3] = {c == 0 ? 1.0 : 0.0, c == 1 ? 1.0 : 0.0, c == 2 ? 1.0 : 0.0};
                        Gm.TransformDir(v[0], v[1], v[2]);
                        for (int r = 0; r < 3; ++r) L.Q[r][c] = v[r];
                    }
                    L.gauge = Placement::Rigid(Gm);
                    L.gauge.t[0] = L.gauge.t[1] = L.gauge.t[2] = 0.0;
                    L.reliefExagg = globe->reliefExagg;
                    // M13: THE GATE IS ITS OWN SAMPLER. What a window shows is read from the same
                    // earth cache the camera reads, and the gate it is seen through answers for it.
                    L.sampler = resMgr.Sampler(("gate." + W.gate->Declared().name).c_str());
                    // WALKED ONLY ALONG THE RAYS THROUGH ITS WINDOWS -- the cull, not the picture --
                    // and not where the next window shows a deeper world. Its tiles are the tiles
                    // of its place, chosen once for every world standing there (GlobeLayer).
                    double pl[5][4];
                    scene::LinkPlanes(W, view, pl);
                    for (int p = 0; p < 5; ++p) {
                        for (int j = 0; j < 4; ++j) L.planes[p][j] = pl[p][j];
                    }
                    L.planeCount = 5;
                    if (k + 1 < n && scene::LinkHole(m_windows[size_t(k + 1)], view, pl)) {
                        for (int p = 0; p < 5; ++p) {
                            for (int j = 0; j < 4; ++j) L.hole[p][j] = pl[p][j];
                        }
                        L.holeCount = 5;
                    }
                    L.share = true;
                    // THE PLACE'S VIEWPOINT, LIT BY THE ONE LIGHT. The carried eye stands at the
                    // place; the sun as seen from THERE is asked of the solar system at that place's
                    // own planet point (the light at 0,0,0 does the rest) and said in the root frame,
                    // which is the frame this level is drawn in. Not a copy of this frame's sun.
                    const double gyE = E[1] + planetR;
                    const double rE = std::sqrt(E[0] * E[0] + gyE * gyE + E[2] * E[2]);
                    double sunE[3] = {sunRootF[0], sunRootF[1], sunRootF[2]};
                    if (m_solarValid) {
                        double pE[3], sd[3];
                        for (int i = 0; i < 3; ++i) {
                            pE[i] = (east0[i] * E[0] + oDir[i] * gyE + north0[i] * E[2]) / planetR;
                        }
                        sun::SunDirFromPlanetPoint(m_solar, pE, sd);
                        sunE[0] = sd[0] * east0[0] + sd[1] * east0[1] + sd[2] * east0[2];
                        sunE[1] = sd[0] * oDir[0] + sd[1] * oDir[1] + sd[2] * oDir[2];
                        sunE[2] = sd[0] * north0[0] + sd[1] * north0[1] + sd[2] * north0[2];
                    }
                    for (int i = 0; i < 3; ++i) L.sun[i] = static_cast<float>(sunE[i]);
                    // ITS SEA'S RINGS: the camera's where this world's eye stands at the camera's own
                    // place (a corridor comes back home every other window), set B where it stands
                    // at the first window's, none elsewhere (the fold, ringless). A level's rings are
                    // read at its OWN points, so a world back home reads the home rings as they are.
                    L.bankSet = within(E, C) ? 0
                              : ((waterBankB && !portal.Valid() && within(E, E1)) ? 1 : -1);
                    double zE[3];
                    ZenithAt(E, planetR, zE);   // the place's own zenith, at the carried eye
                    for (int i = 0; i < 3; ++i) L.skyUp[i] = static_cast<float>(zE[i]);
                    L.skyDay = -1.0f;
                    if (k == 0) {
                        for (int i = 0; i < 3; ++i) {
                            zE1[i] = zE[i];
                            sunE1[i] = sunE[i];
                        }
                    }
                    // THE WINDOW WHERE THE TRUE EYE SEES IT: the chain's pull on its box.
                    WindowBox& B = boxes[k];
                    double bc[3] = {0.0, 0.0, 0.0};
                    W.boxInRoot.TransformPoint(bc[0], bc[1], bc[2]);
                    for (int i = 0; i < 3; ++i) B.centre[i] = static_cast<float>(bc[i] - C[i]);
                    for (int r = 0; r < 3; ++r) {
                        double a[3] = {r == 0 ? 1.0 : 0.0, r == 1 ? 1.0 : 0.0, r == 2 ? 1.0 : 0.0};
                        W.boxInRoot.TransformDir(a[0], a[1], a[2]);   // the box's axis r
                        for (int c = 0; c < 3; ++c) B.rows[r * 3 + c] = static_cast<float>(a[c]);
                        B.half[r] = static_cast<float>(0.5 * W.gate->Declared().size[r]);
                    }
                    // THE RAY, CARRIED. Everything a window shows is where its rays land: the place's
                    // zenith and the one sun as seen from there, turned into this frame by the
                    // rotation this world's geometry is drawn with (Q = rot(Gm), TrueRel), and the
                    // carried eye's distance from the planet's centre. The sky marches the air from
                    // there; the hulls seen there are lit from there. No table of any view.
                    double uW[3] = {zE[0], zE[1], zE[2]};
                    double sW[3] = {sunE[0], sunE[1], sunE[2]};
                    Gm.TransformDir(uW[0], uW[1], uW[2]);
                    Gm.TransformDir(sW[0], sW[1], sW[2]);
                    for (int i = 0; i < 3; ++i) {
                        upWin[k * 4 + i] = static_cast<float>(uW[i]);
                        sunWin[k * 3 + i] = static_cast<float>(sW[i]);
                    }
                    upWin[k * 4 + 3] = static_cast<float>(rE);
                }
                // The eye's own world need not walk what the first window shows.
                double viewHole[5][4];
                const int viewHoleN = scene::LinkHole(m_windows.front(), view, viewHole) ? 5 : 0;
                globe->SetGates(levels, boxes, n, viewHole, viewHoleN);
                if (m_A.vesselLayer) m_A.vesselLayer->SetGateWindows(boxes, sunWin, n);
                if (sky) sky->SetGateWindows(boxes, upWin, sunWin, n);
                if (!m_winSkyLogged) {
                    m_winSkyLogged = true;
                    const double kDeg = 180.0 / 3.14159265358979;
                    double cz[3];
                    ZenithAt(C, planetR, cz);
                    const double elH = std::asin(std::clamp(
                        double(sunRootF[0]) * cz[0] + double(sunRootF[1]) * cz[1] +
                            double(sunRootF[2]) * cz[2], -1.0, 1.0)) * kDeg;
                    const double elT = std::asin(std::clamp(
                        sunE1[0] * zE1[0] + sunE1[1] * zE1[1] + sunE1[2] * zE1[2], -1.0, 1.0)) * kDeg;
                    const double tilt = std::acos(std::clamp(
                        cz[0] * zE1[0] + cz[1] * zE1[1] + cz[2] * zE1[2], -1.0, 1.0)) * kDeg;
                    const scene::Gateway* g1 = m_windows.front().gate;
                    Log("[gate] '%s': the ground turns %.2f deg between the two places; the one "
                        "light stands %+.2f deg here and %+.2f deg at %.4f N %.4f E, and the "
                        "window's rays march that sky",
                        g1->Declared().name.c_str(), tilt, elH, elT, g1->Chart().latDeg,
                        g1->Chart().lonDeg);
                }
                if (n != m_windowDepthLogged) {
                    std::string names;
                    for (const scene::WindowLink& l : m_windows) {
                        if (!names.empty()) names += " -> ";
                        names += "'" + l.gate->Declared().name + "'";
                    }
                    Log("[gate] the view reaches %d window%s deep: %s", n, n == 1 ? "" : "s",
                        names.c_str());
                    m_windowDepthLogged = n;
                    m_windowRecordsDue = true;
                }
            } else if (!m_gates.empty()) {
                globe->SetGates(nullptr, nullptr, 0, nullptr, 0);
                if (m_A.vesselLayer) m_A.vesselLayer->SetGateWindows(nullptr, nullptr, 0);
                if (sky) sky->SetGateWindows(nullptr, nullptr, nullptr, 0);
                if (m_windowDepthLogged > 0) {
                    Log("[gate] the view reaches no window");
                    m_windowDepthLogged = 0;
                }
            }
        }
        PROF_BEGIN();
        globe->SetView(cam, aspect, viewH, simUnix - startUnix);
        PROF_END(7);
        // WHAT EACH WORLD COSTS: this walk's records per slot, when the corridor's depth changes and
        // every ten seconds of frames -- a window is walked only along its own rays, and a place's
        // tiles are chosen once for every world standing there.
        if (!m_windows.empty() && (m_windowRecordsDue || (frame % 600u) == 0u)) {
            std::string per;
            const size_t n = (std::min)(m_windows.size() + 1u, size_t(GlobeLayer::kMaxLevels));
            for (size_t k = 0; k < n; ++k) {
                per += (k ? " " : "") + std::to_string(globe->levelRecords[k]);
            }
            Log("[gate] walk records by world (own first): %s", per.c_str());
            m_windowRecordsDue = false;
        }
        // M13 step 0 (--water-tiles): what this view's leaves would ask of a water tenant on the
        // cube quadtree -- the count that sizes a sampler's reserve, printed every 30 frames.
        if (opt.waterTiles && (frame % 30u) == 0u) {
            Log("[water-tiles] frame %u: %s", frame, globe->WaterTileReport().c_str());
        }
        if (!S.railDirW.empty() && frame >= 150u) {
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
    // SET B IS FILLED FOR ITS READERS. The bank keeps nothing between frames -- every texel is
    // recomputed from this frame's inputs -- so a fill no world reads is thrown away whole: a
    // gate in reach with no window in view paid 1.1 ms of tile list and 0.6 ms of GPU for one
    // every frame (measured 2026-09-17). Its readers are the level table's worlds whose rings
    // are set B -- the outer Droste level, a window's world -- and the rings already stand where
    // the next one's eye will be (above), so the frame that first shows a window fills them
    // before the globe draws it.
    if (waterBankB) waterBankB->enabled = drosteOuter && globe && globe->ReadsBankSet(1);

    PROF_BEGIN();
    tide->SetTime(simUnix, windowSec);
    PROF_END(4);
    // M12 step 4g: THE ONE SURFACE CONSTANT BUFFER. The surface fills its rows once a frame
    // into the renderer's b2 buffer (Renderer::surfaceCb), which RenderFrame pushes once and
    // binds for every layer: the globe, the sea and the GIS vectors read the
    // same bytes from one buffer where each carried a copy inside its own cbuffer -- the
    // globe's refilled every frame from this same SurfaceFrame, the others once at boot
    // (Session), and the fingerprints of all of them agreed with the bytes pushed for b2 at
    // every pose (scratchpad/step4g_probe.py). Mars fills too, as the globe's own fill did:
    // its rows say "height cube, no page" (SurfaceFrame.h's banner). The step 0 fingerprint
    // stays, printed when the bytes change, and is the only one.
    if (globe) {
        // HIERARCHY 4.17 commit 3: the eye the standing blocks' rows are taken about -- the
        // globe walk's own formula on the same camera (GlobeLayer::CaptureWalk), so the rows and
        // the mesh records' eye-relative points share one origin, to the double.
        {
            SurfaceFrame& sf = m_A.surface;
            const double ry = sf.planetR + cam.py;
            for (int k = 0; k < 3; ++k) {
                sf.eye[k] = sf.up[k] * ry + sf.east[k] * cam.px + sf.north[k] * cam.pz;
            }
        }
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
    // THE PLANE THE SOLVER IS FORCED BY, to the banks: inside its domain the level is that plane
    // plus its deviation (the solver is truth) -- the same value sea->SetTime hands the solver.
    {
        const double tidePlane = bathy.Ready() ? waterNavd : tide->focusHeight;
        if (waterBank) waterBank->SetTidePlane(tidePlane);
        if (waterBankB) waterBankB->SetTidePlane(tidePlane);
    }
    PROF_BEGIN();
    if (swe.Ready()) {
        swe.SetBoundaries(static_cast<float>(westAt(simUnix)),
                          static_cast<float>(southAt(simUnix)),
                          static_cast<float>(westQAt(simUnix)));
    }
    PROF_END(9);
    // M7k: arm the programmatic .wpix capture so it records the run's LAST warm
    // frames -- every pass named by its state-diagram node.
    const uint32_t capEnd = S.capture.frames + (S.railDirW.empty() ? 0u : 150u);
    if (opt.pixFrames > 0 && S.capture.frames > 0 && capEnd >= opt.pixFrames + 4 &&
        frame + opt.pixFrames + 4 == capEnd) {
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
            if (u1 > u0 && v1 > v0) resMgr.Want(sampView, exposureT, 6u, 3u, u0, v0, u1, v1);
            // M10: and around the outer level's eye, whose sea set B draws.
            if (drosteOuter) {
                const double latO = BathyModel::kOrgLat + drosteOuterCam[2] / BathyModel::kMPerLat;
                const double lonO = BathyModel::kOrgLon + drosteOuterCam[0] / BathyModel::kMPerLon;
                const float ou0 = float(std::clamp(mU(lonO - dLon), 0.0, 1.0));
                const float ou1 = float(std::clamp(mU(lonO + dLon), 0.0, 1.0));
                const float ov0 = float(std::clamp(mV(latO + dLat), 0.0, 1.0));
                const float ov1 = float(std::clamp(mV(latO - dLat), 0.0, 1.0));
                if (ou1 > ou0 && ov1 > ov0)
                    resMgr.Want(sampView, exposureT, 6u, 3u, ou0, ov0, ou1, ov1);
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
    // ---- THE INTERESTS (the water match): the subjects and places the recorded view names keep
    // their water resident at the grain its kernels read -- the solved field's pages at the solver's
    // own cells (mip 0), the swell shadow at the bank's floor (kSwellShadowMipFloor), the bed at the
    // fine rings' grain (mip 0) -- so an eye arriving there finds the data landed rather than
    // landing. By name, per view: a view that names no interest holds none (a spectator need not
    // load every boat's water). --water-probe measured the need on a fly-in down the flood rail: for
    // ~2 s after arriving at the helm the drawn solved sea stood at 0.77-0.92 of the hull's.
    if (!marsMode && !m_startView.interests.empty()) {
        const Lattice& page = m_A.surface.winH;
        const uint32_t pageSlice = m_A.surface.hgtWinSlice;   // the z14 page's slice (0..5: the cube)
        for (const std::string& iname : m_startView.interests) {
            const SceneInterest* si = nullptr;
            for (const SceneInterest& x : S.interests) {
                if (x.p.name == iname) si = &x;
            }
            if (!si) {
                if (m_interestsMissing.insert(iname).second) {
                    Log("[interest] view '%s' names '%s', which no interest declares -- nothing held",
                        m_startView.p.name.c_str(), iname.c_str());
                }
                continue;
            }
            // Where it stands: the entity's centre of gravity in the root's flat frame (a hull
            // carried into another space is placed back through its space's motor), or the place.
            double cx = 0.0, cz = 0.0;
            bool placed = false;
            if (!si->p.target.empty()) {
                for (const auto& e : m_entities) {
                    if (e->Name() != si->p.target || !e->Hull()) continue;
                    double c[3] = {0.0, 0.0, 0.0};
                    e->Hull()->Body().pose.TransformPoint(c[0], c[1], c[2]);
                    if (e->InSpace()) e->SpaceInRoot().TransformPoint(c[0], c[1], c[2]);
                    cx = c[0];
                    cz = c[2];
                    placed = true;
                    break;
                }
            } else if (si->hasAt) {
                scene::PoseSugar ps;
                std::string pwhy;
                if (scene::ReadPoseSugar(si->at, "interests." + si->p.name + ".at", ps, &pwhy) &&
                    ps.kind == scene::PoseSugar::Kind::Compass) {
                    cx = ps.x;
                    cz = ps.z;
                    placed = true;
                }
            }
            if (!placed) continue;
            // M13: a subject is a SAMPLER of the one earth cache -- it answers for the tiles it
            // holds resident around itself, separately from the view that named it.
            const int sampI = resMgr.Sampler(("subject." + iname).c_str());
            const double r = (std::max)(si->p.radius, 0.0);
            uint32_t held = 0;
            // The z14 page tenants: the swell shadow and the bed.
            if (pageSlice != UINT32_MAX && page.kind == Lattice::Kind::Window) {
                double pxW = 0.0, pyN = 0.0, pxE = 0.0, pyS = 0.0;
                page.PxOf(BathyModel::kOrgLat + (cz + r) / BathyModel::kMPerLat,
                          BathyModel::kOrgLon + (cx - r) / BathyModel::kMPerLon, pxW, pyN);
                page.PxOf(BathyModel::kOrgLat + (cz - r) / BathyModel::kMPerLat,
                          BathyModel::kOrgLon + (cx + r) / BathyModel::kMPerLon, pxE, pyS);
                const auto uvOf = [&](double px, long long org) {
                    return float(std::clamp((px - double(org)) / double(page.faceDim), 0.0, 1.0));
                };
                const float u0 = uvOf(pxW, page.orgPxX), u1 = uvOf(pxE, page.orgPxX);
                const float v0 = uvOf(pyN, page.orgPxY), v1 = uvOf(pyS, page.orgPxY);
                if (u1 > u0 && v1 > v0) {
                    if (exposureT >= 0 && exposureSrc && exposureSrc->Valid()) {
                        resMgr.Want(sampI, exposureT, pageSlice, kSwellShadowMipFloor, u0, v0, u1, v1);
                        ++held;
                    }
                    if (hgtWinTenant >= 0) {
                        resMgr.Want(sampI, hgtWinTenant, pageSlice, 0u, u0, v0, u1, v1);
                        ++held;
                    }
                }
            }
            // The solved field's pages (z16), through the solver's grid (the page texel IS the cell).
            if (waveSrc && waveT >= 0 && waveSrc->Key() != 0 && waveField && waveField->Ready()) {
                const WaveField::GpuTable& tab = waveField->Table();
                const double nx = double(waveFrame.nx), ny = double(waveFrame.ny);
                const auto cellsOf = [&](double w, double org, double n) {
                    return std::clamp((w - org) * double(tab.invCell), 0.0, n);
                };
                const double cx0 = cellsOf(cx - r, double(tab.orgX), nx);
                const double cx1 = cellsOf(cx + r, double(tab.orgX), nx);
                const double cz0 = cellsOf(cz - r, double(tab.orgZ), ny);
                const double cz1 = cellsOf(cz + r, double(tab.orgZ), ny);
                if (cx1 > cx0 && cz1 > cz0) {
                    const double ox = double(waveFrame.winPxX - waveFrame.orgPxX);
                    const double oy = double(waveFrame.winPxY - waveFrame.orgPxY);
                    const float u0 = float((ox + cx0) / 16384.0), u1 = float((ox + cx1) / 16384.0);
                    const float v0 = float((oy + ny - cz1) / 16384.0);
                    const float v1 = float((oy + ny - cz0) / 16384.0);
                    const uint32_t planes = tab.nUsed + 1u;
                    for (uint32_t p = 0; p < planes; ++p)
                        resMgr.Want(sampI, waveT, 6u + p, 0u, u0, v0, u1, v1);
                    held += planes;
                }
            }
            if (m_interestsLogged.insert(iname).second) {
                Log("[interest] view '%s' holds '%s' (%s%s, %.0f m): %u page wants a frame -- the "
                    "solved field at mip 0, the swell shadow at mip %u, the bed at mip 0",
                    m_startView.p.name.c_str(), iname.c_str(),
                    si->p.target.empty() ? "a place" : "follows ", si->p.target.c_str(), r, held,
                    kSwellShadowMipFloor);
            }
        }
    }
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
        static_cast<int64_t>(frame) - (S.railDirW.empty() ? 0 : 150);
    // --settle-sync reads the ring gate's hold count for THIS frame's wants here;
    // ProcessQueues zeroes it inside RenderFrame.
    // (M9bi's sun placement moved to the top of the frame's housekeeping in M10: the
    // globe's level table carries the sun, and the walk that fills it runs before here.)
    const uint32_t ringHeldBefore = resMgr.ringHeldFrame;
    // M12 step 5b: THE VIEWS SEAM. The renderer records a LIST of views in file order and the
    // session hands it one, built here rather than inside RenderFrame so that the day a scene
    // carries two the loop appends the second and the renderer does not change. The View is
    // the RENDER's input; the session's Camera is still the state, and every other reader in
    // this loop -- the predicted pose and the globe's walk, the pickers, the chase camera, the
    // gravity-up and speed clamps, the Droste gauge, the bank's centre, the exposure and GIS
    // probes -- still reads `cam` directly (5d moves them, with the scene wired).
    PublishHulls();   // after the view has decided what it looks through this frame
    const scene::ViewSet viewSet =
        renderer.OneView(cam, static_cast<float>(simUnix - startUnix));
    renderer.RenderFrame(viewSet);
    // --sky-probe: the atmosphere's tables, read back once the first one is built and held
    // against published optical depths (SkyLayer::Probe). Reads back and waits: an instrument.
    if (opt.skyProbe && sky && frame >= 2u && !m_skyProbed) {
        m_skyProbed = true;
        sky->Probe(gpu);
    }
    // --water-probe N: the drawn sea against each hull's own water, every N recorded frames
    // (app/Tools/WaterProbe.cpp). An instrument: it reads back and waits, so never in play.
    if (opt.waterProbeEvery > 0 && !m_entities.empty()) {
        const uint32_t probeSettle = S.railDirW.empty() ? 0u : 150u;
        if (frame >= probeSettle && ((frame - probeSettle) % opt.waterProbeEvery) == 0u) {
            tools::RunWaterProbe(gpu, renderer, cam, planetR, m_entities, waterBank,
                                 m_A.vesselLayer, &waterAtlas, exposureSrc.get(),
                                 m_waveField.get(), sea, &seaState, m_A.surface,
                                 m_oceanAt ? m_oceanAt(simUnix) : 0.0, simUnix,
                                 frame - probeSettle);
        }
    }
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

    if (!S.railDirW.empty() && frame >= 150u && opt.bench) {
        // Bench records the timing and nothing else -- no readback, no encode, no disk.
        railMs.push_back(renderMs);
        loopRowOpen = true;   // closed by the next iteration's interval
        railPreMs.push_back(preMs);
        railPool.push_back(PoolCommittedBytes());
        railInMs.push_back(inMs);
    } else if (!S.railDirW.empty() && frame >= 150u) {
        if (recPipe.Open()) {
            uint32_t rowPitch = 0;
            if (renderer.DumpRaw(recPixels, &rowPitch)) {
                recPipe.Write(recPixels, rowPitch);
            }
        } else {
            wchar_t rp[512];
            swprintf(rp, 512, L"%s\\rail_%04u.png", S.railDirW.c_str(), frame - 150u);
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

    if (!S.capture.headless &&
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
    // --bed-trace N: the bed the solver reads, every N frames, beside the solver's own probes
    // and, under --twin-surface, the three levels at the camera; the first reading keeps the
    // frame it follows as a still beside --dump's (<name>_f<N>.png). An instrument: it reads
    // back and waits, after the frame it describes was recorded.
    if (opt.bedTraceEvery > 0 && m_bedTracer.Configured() && !marsMode &&
        frame % opt.bedTraceEvery == 0u) {
        const std::string tag = "f" + std::to_string(frame);
        if (frame == opt.bedTraceEvery && !S.dumpW.empty()) {
            std::wstring still = S.dumpW;
            const size_t dot = still.find_last_of(L'.');
            still.insert(dot == std::wstring::npos ? still.size() : dot,
                         L"_f" + std::to_wstring(frame));
            renderer.DumpPng(still);
        }
        m_bedTracer.Read(gpu, swe, tag, oceanAt(simUnix));
        // --bed-trace-plant: the same instant with the rule's floor at the coarsest mip. The
        // instrument has to call that bed coarse, or it cannot be trusted to call one fine.
        if (opt.bedTracePlant && frame == opt.bedTraceEvery && hgtTenant >= 0) {
            m_bedTracer.Read(gpu, swe, tag + "-planted", oceanAt(simUnix),
                             static_cast<float>(resMgr.Mips(hgtTenant) - 1u));
        }
        if (swe.Ready()) {
            const float pts[6] = {900.0f, 100.0f, 250.0f, 60.0f, 2400.0f, 0.0f};
            SweSolver::Probe pr[3];
            swe.ReadProbes(gpu, pts, 3, pr);
            Log("[swe] probes  bar(900,100): dEta %+.3f u %+.2f,%+.2f v%d | throat(250,60): "
                "dEta %+.3f u %+.2f,%+.2f v%d | ocean(2400,0): dEta %+.3f -- frame %u (t+%.1f s)",
                pr[0].dEta, pr[0].u, pr[0].v, pr[0].valid ? 1 : 0, pr[1].dEta, pr[1].u,
                pr[1].v, pr[1].valid ? 1 : 0, pr[2].dEta, frame, simUnix - startUnix);
        }
        if (S.Tool("twin-surface") && sea && waterBank) {
            tools::LogLevelsAtCamera(gpu, waterBank, cam, simUnix, weather, lastWaterNavd);
        }
    }
    // --dump-both: the solid frame has just been rendered. Dump it, flip BOTH
    // surfaces to wireframe, and take one more lap -- the clock is held above, so
    // the second image is the same instant seen as lines.
    if (opt.dumpBoth && !S.dumpW.empty() && !dumpedSolid && S.capture.frames &&
        frame == S.capture.frames + (S.railDirW.empty() ? 0u : 150u)) {
        renderer.DumpPng(S.dumpW);
        dumpedSolid = true;
        if (globe) globe->surfaceDebug = 1;
        if (sea) sea->wireframe = true;
        return true;
    }
    if (S.capture.frames &&
        frame >= S.capture.frames + (S.railDirW.empty() ? 0u : 150u) +
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
        if ((S.capture.settle.sync || S.capture.settle.hold || S.capture.settle.exact) &&
            !S.dumpW.empty() && S.capture.headless && !opt.dumpBoth) {
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
                resMgr.settleExact = S.capture.settle.exact;
                // --settle-clear-churn: the whole atlas onto the clear list; the next
                // held frame's clear dispatch zeroes it and the freeze keeps it so.
                if (S.capture.settle.clearChurn && sea) {
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
                (S.capture.settle.exact && ex.exact && prefillIdle) ? settleExactQuiet + 1 : 0;
            // The drain's give-up cap bounds the DRAIN, never the counted hold: a run
            // asked for N frames gets N whatever the residency is doing.
            const bool drainHolds =
                S.capture.settle.sync && settleQuiet < kSettleQuietFrames &&
                settleFrames < kSettleCapFrames;
            const bool exactHolds =
                S.capture.settle.exact && settleExactQuiet < kSettleExactFrames &&
                settleFrames < kSettleCapFrames;
            const bool countHolds = settleFrames < S.capture.settle.hold;
            if (drainHolds || exactHolds || countHolds) {
                ++settleFrames;
                if (settleFrames % 150u == 0u) {
                    Log("[settle-sync] +%u frames at the held instant: pending %u, "
                        "in-flight reads %u, ring-held %u, pool %.0f MB",
                        settleFrames, pend, reads, ringHeldBefore,
                        PoolCommittedBytes() / 1048576.0);
                    if (S.capture.settle.exact) {
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
            if (S.capture.settle.exact) resMgr.LogSettleExact(settleFrames);
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
                S.capture.settle.exact
                    ? (S.capture.settle.hold ? "exact, then --settle-hold"
                                             : "--settle-exact, judged")
                : S.capture.settle.sync
                    ? (S.capture.settle.hold ? "drain, then --settle-hold"
                                             : "--settle-sync, judged")
                    : "--settle-hold, counted";
            const bool drained = !S.capture.settle.sync || settleQuiet >= kSettleQuietFrames;
            const bool exact = !S.capture.settle.exact || settleExactQuiet >= kSettleExactFrames;
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
            for (const wchar_t* w = S.railDirW.c_str(); *w; ++w) {
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
        if (S.Tool("dump-water-state") && !marsMode && sea) {
            tools::RunDumpWaterState(opt, gpu, sea, simUnix, weather);
        }
        if (S.Tool("twin-surface") && !marsMode && sea && waterBank) {
            tools::RunTwinSurface(opt, gpu, seaState, sea, waterScene, waterBank, cam,
                                  simUnix, weather, waveField, lastWaterNavd);
        }
        if (S.Tool("trace") && !marsMode && sea && waterBank) {
            tools::RunTrace(opt, gpu, sea, compositor, hgtCh, waterAtlas, waterBank, globe,
                            resMgr, winOrgX, winOrgY, hgtTenant, hgtWinTenant, simUnix,
                            weather);
        }
        // --bed-trace: THE WHOLE BED, the readings' reference. After everything above has read
        // the run's last instant, the domain is made whole the way the bed wait makes it -- the
        // same turns, the same trace as its test -- and read once more; every reading of the run
        // is then held against it.
        if (opt.bedTraceEvery > 0 && m_bedTracer.Configured() && !marsMode) {
            const WeatherManager::BedWait bw = weather.WaitForBeds(gpu, resMgr, hgtTenant);
            Log("[bedtrace] the domain made whole at the run's end: %u tiles mapped in %u turns, "
                "%.2f s -- %s",
                bw.tiles, bw.turns, bw.seconds, bw.done ? "whole" : "NOT whole (see the reading)");
            if (m_bedTracer.Read(gpu, swe, "whole", oceanAt(simUnix))) {
                m_bedTracer.CompareWithWhole("whole");
            }
        }
        return false;
    }
    return true;
}

int FrameLoop::Finish() {
    const Options& opt = m_opt;
    const Scene& S = m_S;
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

    // THE SHUTDOWN TRAIL (core/ExitTrail.h): a flushed line before every phase from here to the
    // last destructor, so a death in teardown names its phase.
    ExitStep("finish: the loop is over -- the run's reports and dumps");

    // M12 step 4d instrument: the [droste] probe's totals over the run -- one line per site
    // (the per-build dumps are above: ProbeDrosteTable, ProbeDive, GlobeLayer::ProbeTransport).
    if (m_portalNode.Valid()) {
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
        prof->Report(S.railDirW.empty() ? -1 : 900);
        if (!S.railDirW.empty()) {
            std::string csv;
            for (const wchar_t* w = S.railDirW.c_str(); *w; ++w) {
                csv.push_back(static_cast<char>(*w));
            }
            csv += "\\gpu_ms.csv";
            if (!prof->WriteCsv(csv)) Log("[gpu] could not write %s", csv.c_str());
        }
    }
    if (frameMsN > 30) {
        Log("[perf] mean frame %.2f ms over %u frames (%.0f fps)%s", frameMsSum / frameMsN,
            frameMsN, 1000.0 / (frameMsSum / frameMsN),
            S.capture.headless ? "" : (gpu.TearingEnabled() ? " [no-vsync, tearing]" : " [vsync]"));
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
    if (!S.capture.headless) gpu.ReportPresentStats();
    // M7l: THE DEBUG SESSION REPORT -- the free byproducts, printed every run: what the
    // diagram holds, what the compositor did, what streaming did. The gates print their
    // own PASS lines under --selftest; the trace and fibers print theirs when asked.
    Log("[report] ---- session: %zu AST edges (%s) | compose %u painted %u cache-hit | "
        "fetches %u | pending %u ----",
        ga::ast::Edges().size(), ga::ast::Validate() ? "frames hold" : "FLIP FAILURES",
        compositor.painted.load(), compositor.cacheHits.load(),
        resMgr.fetchesThisRun, resMgr.PendingCount());
    // The fetches a larger streaming.tileBudget would have made: with the budget at zero, the
    // run's whole appetite for source tiles it did not have.
    if (m_A.googleTiles.Ready()) {
        Log("[google] %u source tiles refused this run (budget %u, %u fetched): the distinct "
            "fetches a larger budget or day cap would have made",
            m_A.googleTiles.Refused(), m_A.googleTiles.Budget(), m_A.googleTiles.Fetched());
    }

    if (S.Tool("sea-verify") && sea) tools::RunSeaVerify(opt, gpu, sea);
    if (!S.dumpW.empty()) {
        if (dumpedSolid) {
            // out.png already holds the solid frame; this pass is the wireframe twin.
            std::wstring wp = S.dumpW;
            const size_t dot = wp.find_last_of(L'.');
            wp = (dot == std::wstring::npos) ? wp + L"_wire"
                                             : wp.substr(0, dot) + L"_wire" + wp.substr(dot);
            renderer.DumpPng(wp);
            Log("[dump] wrote %S (solid) + %S (wireframe, same instant)",
                S.dumpW.c_str(), wp.c_str());
        } else {
            renderer.DumpPng(S.dumpW);
        }
    }
    // The same frame's radiance before the tonemap (tools/imgdiff.py --hdr).
    if (!S.hdrW.empty()) renderer.DumpHdr(S.hdrW);
    if (!opt.dumpMeshlets.empty() && globe) globe->DumpMeshlets(opt.dumpMeshlets);

    // THE PAINTS BEHIND THE INVALIDATIONS (capture.residencyAudit): what each tree bound to a
    // tenant painted, read and folded over the run. A fold announces the coarser tile it rewrote
    // (TileTree::FoldUp -> onChanged -> ResidencyManager::Invalidate), so a run that painted
    // nothing invalidated nothing, and a run without invalidations cannot have shown finding 3.
    if (resMgr.auditEvery) {
        auto tally = [](const char* name, TileTree* t) {
            if (!t) return;
            uint64_t n[8] = {};
            TreeTally(*t, n);
            Log("[res-audit] tree %-16s %s: painted %llu, read %llu, cache hits %llu, composed %llu, "
                "folded %llu, cached composites dropped %llu | archive: %llu places, %llu handle "
                "reads (every node under it, over the run)",
                name, t->Id().c_str(), static_cast<unsigned long long>(n[0]),
                static_cast<unsigned long long>(n[1]), static_cast<unsigned long long>(n[2]),
                static_cast<unsigned long long>(n[3]), static_cast<unsigned long long>(n[4]),
                static_cast<unsigned long long>(n[5]), static_cast<unsigned long long>(n[6]),
                static_cast<unsigned long long>(n[7]));
        };
        tally("earth.height", m_A.heightTree.get());
        tally("megatexture", m_A.megaTree.get());
        if (m_A.exposureTree) tally("swell.exposure", std::atomic_load(m_A.exposureTree.get()).get());
        if (m_waveTree) tally("wave.field", std::atomic_load(m_waveTree.get()).get());
    }

    ExitStep("finish: gpu.WaitIdle -- the queue drains");
    gpu.WaitIdle();
    ExitStep("finish: residency Shutdown -- the loads still running on the pool leave");
    resMgr.Shutdown();
    ExitStep("finish: scene watch Stop");
    sceneWatch.Stop();   // cancels the pending directory read and joins (logged)
    ExitStep("finish: renderer Shutdown -- the layers' teardown (the globe joins its worker)");
    renderer.Shutdown();
    ExitStep("finish: Gpu::Shutdown");
    gpu.Shutdown();
    ExitStep("finish: window Destroy, thread audit report");
    window.Destroy();
    ga::threadaudit::Report();   // --thread-audit: what the threads did to the tile files
    ExitStep("finish: the job pool's Shutdown -- the workers join; jobs still queued are dropped");
    ga::Threads().Shutdown();
    Log("done (%u frames)", frame);
    return 0;
}

#undef PROF_BEGIN
#undef PROF_END

}  // namespace ga::app
