// ================================================================================================
//  gagame - M0 + M1.
//
//  Modes:
//    (default)     windowed viewer. WASD/QE fly, right-drag look, wheel speed, R reload shaders,
//                  F5 save the camera to data/views.json (relaunch into it with --view NAME).
//                  Time controls: SPACE pause, UP/DOWN time-scale x10, LEFT/RIGHT nudge -/+ 1 h
//                  (SHIFT: 1 day), HOME or N back to now, [ ] halve/double the plot window.
//    --selftest    M0 gate: run the reserved-resource null-tile test suite headless and exit
//                  with 0 (pass) / 1 (fail). See src/core/TileAtlas.cpp.
//    --headless    no window; render --frames frames and write --dump to a PNG. The verification
//                  path: a renderer change is provable from a shell.
//
//  The window title is the HUD: sim clock (UTC), time scale, focus-station tide, fit RMS.
// ================================================================================================
#include <sys/stat.h>

#include "compose/ColorStackSource.h"
#include "compose/ExposureSource.h"
#include "compose/GisMask.h"
#include "compose/HeightStackSource.h"
#include "compose/TileTree.h"
#include "compose/TileArchive.h"
#include "compose/TileIndex.h"
#include "core/TileStream.h"
#include "compose/ComposeTree.h"
#include "compose/DomainSource.h"
#include "core/CurrentFieldLoader.h"
#include "core/GeoGridLoader.h"
#include "core/Gpu.h"
#include "core/Image.h"
#include "core/PixEvents.h"
#include "core/TileAtlas.h"
#include "core/Window.h"
#include "render/Renderer.h"
#include "scene/FieldSet.h"
#include "scene/GisLayer.h"
#include "scene/GlobeLayer.h"
#include "scene/MarkerLayer.h"
#include "scene/GulfLayer.h"
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
#include "sim/SimClock.h"
#include "core/DxTest.h"
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

#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <ctime>
#include <fstream>
#include <filesystem>
#include <string>

using namespace ga;

namespace {

struct Options {
    uint32_t width = 1600, height = 900;
    bool headless = false;
    bool selftest = false;
    bool trace = false;               // --trace lat,lon: the hypervisor walk (M7j)
    uint32_t pixFrames = 0;           // --pix N: programmatic .wpix capture of N frames
    bool dumpFibers = false;          // --dump-fibers: bank planes as PNGs + range gate
    int lens = 0;                     // --lens worldxz|winuv|mip|ring: value-as-color
    bool probeCullFar = false;        // step 23 probe: cull beyond the horizon at every altitude
    std::wstring dumpMeshlets;        // step 23 probe: the dump frame's meshlet records
    bool dumpWater = false;           // --dump-water-state: inlet fields for proofs/
    bool sliceOn = false;             // --slice d: the cutaway plane (M7o)
    double sliceD = 0.0;              // plane offset, world z metres
    int inject = 0;                   // --inject [bank|cascade]: edge test cards
    double traceLat = 42.816, traceLon = -70.81;
    bool debugLayer = false;
    uint32_t frames = 0;              // 0 = run until the window closes
    std::wstring dump;
    std::wstring shaderDir = L"shaders";
    std::string tidesPath = "data/tides/stations.json";
    std::string seaPath = "data/sea/seastate.json";
    std::string currentsPath = "data/currents/currents.json";
    bool seaStart = false;            // begin in the open-sea view (TAB cycles views)
    bool gulfStart = false;           // begin in the gulf map view
    bool globeStart = false;          // begin on the planet (M6)
    float gcamLat = 1e9f, gcamLon = 0, gcamAltKm = 13000;   // --globe-cam override
    std::wstring rail;                // --rail DIR: record the debug camera rails to PNGs
    std::string planet = "earth";     // M6e: earth | mars (the rescued sample's pyramids)
    uint32_t tileBudget = 1000;       // --tile-budget: hard cap on Google fetches per run
    bool warmInlet = false;           // --warm-inlet: pre-cache the Merrimack detail pyramid
    bool railZoom = false;            // --rail-zoom DIR: orbit -> inlet imagery zoom -> estuary
    bool framesSet = false;           // an explicit --frames beats a rail default
    uint32_t predictEvery = 3;        // --predict-every N: prefetch-walk cadence (1 = old)
    bool predictInline = false;       // --predict-inline: the prefetch walk on the main thread
                                      // where it used to run (step 5's A/B: the same FNV-1a)
    bool packTiles = false;           // --pack-tiles: pack the composed cache, then exit
    bool directStorage = true;        // --no-direct-storage: the upload ring, for the A/B (M9ao)
    bool dsSerial = false;            // --ds-serial: one DS batch in flight (diagnostic)
    bool colorTrees = true;           // --no-color-trees: the incumbent providers, for the A/B.
                                      // The DEFAULT: colour AND height pages fed from the trees.
    bool gisGate = true;              // --no-gis-gate: drop the vector land/sea gate on the bed
    bool seafloor = true;             // --no-seafloor: drop the global seafloor relief source
    bool exposure = true;             // --no-exposure: no swell-exposure page (everything exposed)
    std::string gisDump;              // --gis-dump PATH: the gate over the survey box as PGM, exit
    bool ringLoads = true;            // --no-ring-loads: the old queue, for the A/B (M9al)
    bool resTrace = false;            // --res-trace: residency deficit + slot accounting, per 30 f
    bool threadAudit = false;         // --thread-audit: count tile-file collisions between threads
    bool jobsInline = false;          // --jobs-inline: every job on the calling thread, in order
    uint32_t traceFrom = UINT32_MAX;  // --res-trace-frames A:B: the landing ledger every turn of
    uint32_t traceTo = 0;             // recorded frames A..B (Residency.h TurnLedger, step 28)
    uint32_t treeAudit = 0;           // --tree-audit N: compare N tiles/frame, report, exit
    bool warmTrees = false;           // --warm-trees: build them without comparing, then exit
    bool packTrees = false;           // --pack-trees: one archive per node per frame, then exit
    bool bench = false;               // --bench: fly the rail, capture nothing, time honestly
    bool benchOverlap = false;        // --bench-overlap: --bench WITHOUT the per-frame WaitIdle --
                                      // the loop mean is then the shipped max(CPU, GPU) pipeline
    bool gpuTime = false;             // --gpu-time: timestamp queries per pass, [gpu] lines + gpu_ms.csv
    bool noVsync = false;             // --no-vsync: windowed, ALLOW_TEARING + Present(0, tearing)
    bool settleSync = false;          // --settle-sync: hold the --dump frame's instant until the
                                      // residency queues and DirectStorage reads have drained
    uint32_t settleHold = 0;          // --settle-hold N: hold it for exactly N frames, counted;
                                      // with --settle-sync, drain first and then to at least N
    bool settleExact = false;         // --settle-exact: hold until the resident set IS the
                                      // walk's want set (Residency.h settleExact), churn frozen
    bool settleClearChurn = false;    // --settle-clear-churn: zero the churn atlas at the first
                                      // held frame (SeaLayer.h ClearChurn), the history A/B
    std::wstring dumpHdr;             // --dump-hdr PATH: the RGBA16F radiance before the tonemap
    std::string mp4;                  // --mp4 PATH: pipe rail frames straight to an encoder
    bool flatBed = false;             // --flat-bed N: constant bed, to A/B bathymetry
    float flatBedNavd = -30.0f;
    bool railFlood = false;           // --rail-flood DIR: orbit -> zoom -> the throat at helm
    bool railJetty = false;           // --rail-jetty DIR: jetty tip -> jetty tip -> bird's eye
                                      // height, facing the entrance (shoot at max flood)
    double startUnix = -1;            // < 0 = now
    double timeScale = 1.0;           // sim seconds per wall second. REAL TIME by default --
                                      // waves at 900x looked like a kettle at full boil; the
                                      // arrow keys fast-forward (UP x10 per press)
    float exaggeration = 60.0f;       // ribbon vertical exaggeration
    double windowDays = 7.0;          // plot window
    float fovDeg = 55.0f;
    float foam = 1.0f;                // sea-mode whitecap intensity (0 disables)
    float edgePx = 12.0f;             // tessellated triangle edge target, pixels
    float heightScale = 1.15f;        // wave vertical exaggeration (vqview's shipped look)
    float stormHs = 0, stormTp = 10, stormDir = 90;   // --storm sandbox override
    bool seaVerify = false;           // measure rendered Hs from the displacement textures
    bool viz = false;                 // start with the atlas residency visualizer on
    int surfaceDebug = 0;             // G / --wireframe / --meshlets: 0 shipped, 1 lines,
                                      // 2 one flat colour per amplification record
    std::string loadField;            // --load-field PATH: run a file through the loader
                                      // plugin into a sparse bank and report what it cost
    bool meshStats = false;           // --mesh-stats: one meshlet cell-size-vs-distance table
    bool dumpBoth = false;            // --dump-both: write out.png AND out_wire.png from the
                                      // SAME instant (one extra frame, sim clock frozen), so
                                      // shading and geometry are compared without a re-run
    bool stencil = false;             // M6i --stencil: coast/graticule alignment overlay
    bool msSurface = true;            // M6j: mesh-shader planet surface (--no-ms falls back)
    bool albedo = false;              // M6j: raw-texture lens (no lighting/atmosphere)
    std::string exportSpec;           // M6j --export: composed-channel export spec
    std::wstring exportOut;
    float camAlt = -1, camAz = 246, camPitch = -5;   // --cam alt,az,pitch override
    float camX = 1e9f, camZ = 1e9f;   // --campos x,z world override (sea mode)
    std::string view;                 // --view NAME: a camera saved with F5 (data/views.json)
    std::string bathyPath = "data/bathy/merrimack.json";
    float datumOff = -1.30f;          // tide (m MLLW) + this = water level in NAVD88
    bool datumSet = false;            // --datum given: overrides the CO-OPS datum resolution
    bool sweOff = false;              // --swe-off: analytic tide plane only (pre-M5c behaviour)
    bool sweWestOff = false;          // --swe-west-off: zero the west-boundary deviation
    double sweSpinupH = 1.0;          // solver history integrated before the first frame
                                      // (M6r: the prism-debt field needs ~an hour to settle;
                                      // costs ~1 s at startup)
    double sweCycleH = 0;             // --swe-cycle N: headless validation over N hours -> CSV
    std::wstring waterMap;            // --water-map out.png: the REPROJECTION PROOF -- the
                                      // water atlas + survey printed through a custom Lambert
                                      // conformal sheet (paper-chart foundation)
    std::wstring bathyMap;            // --bathy-map out.png: the same sheet, hypsometric
    std::wstring fidelityMap;         // M8j --fidelity-map out.png: the HETEROGENEITY SHEET --
                                      // the schema registry drawn, hue = finest source here,
                                      // brightness = log resolution, columns = zoom rungs
                                      // channel bathymetry (M6w: the chart IS the stack)
    std::string oceanProbe;           // --ocean-probe lat,lon: the weather manager's
                                      // verification harness (rungs + provenance + gates)
    bool oneWater = false;            // --one-water: M7 -- water geometry from the wave
                                      // vertex bank alone (SeaLayer's grid retires)
    bool pixelWater = false;          // --pixel-water: M9bh -- shade the water per PIXEL (the
                                      // two rays: sky mirror + refracted bed cast, translucent,
                                      // no foam). Off = the M9bg vertex-shaded default.
    // M9bi: --sun az,el PINS the pre-ephemeris art direction (112, 26) that every baseline
    // before M9bi was lit by. Without it the sun comes from the EPHEMERIS at the scene's own
    // timestamp and place -- a deliberate look change, which is why the escape hatch exists.
    bool sunPinned = false;
    float sunAz = 112.0f, sunEl = 26.0f;
    double riverQ = -1;               // --river q overrides data/river/river.json
    std::wstring sweUvDump;           // --swe-uv f.png: dump the solved current field after
                                      // spin-up (debug picture: red east, blue west)
    float sweGain = 1.0f;             // solved-current gain (see SeaLayer). M6r RETIRED the
                                      // x5 compensation: the old deviation formulation let
                                      // the basin fill by construction (the moving tide
                                      // plane), so the gap only carried ~1/5 of the prism.
                                      // With the prism source term + Flather + per-axis
                                      // metric + walled jetties the solver's throat core
                                      // reads 0.93 vs ACT 1.06 -- honest at gain 1
};

std::wstring Widen(const char* s) {
    std::wstring w;
    while (*s) w.push_back(static_cast<wchar_t>(*s++));
    return w;
}

double NowUnix() {
    return std::chrono::duration<double>(
               std::chrono::system_clock::now().time_since_epoch()).count();
}

// Days since the unix epoch for a civil date (Howard Hinnant's algorithm).
int64_t DaysFromCivil(int y, unsigned m, unsigned d) {
    y -= m <= 2;
    const int64_t era = (y >= 0 ? y : y - 399) / 400;
    const unsigned yoe = static_cast<unsigned>(y - era * 400);
    const unsigned doy = (153 * (m + (m > 2 ? -3 : 9)) + 2) / 5 + d - 1;
    const unsigned doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
    return era * 146097 + static_cast<int64_t>(doe) - 719468;
}

// "YYYY-MM-DD[THH:MM[:SS]]" (UTC, trailing Z tolerated) or a raw unix-seconds number.
double ParseStartTime(const std::string& s) {
    int y = 0, mo = 0, d = 0, h = 0, mi = 0, se = 0;
    const int n = sscanf_s(s.c_str(), "%d-%d-%dT%d:%d:%d", &y, &mo, &d, &h, &mi, &se);
    if (n >= 3) {
        return static_cast<double>(DaysFromCivil(y, static_cast<unsigned>(mo),
                                                 static_cast<unsigned>(d)) * 86400ll) +
               h * 3600.0 + mi * 60.0 + se;
    }
    return atof(s.c_str());
}

Options ParseArgs(int argc, char** argv) {
    Options o;
    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        // M9ay: a flag with a DEFAULT must not eat the next flag. `--storm --rail-flood DIR`
        // parsed as storm="--rail-flood" and a stray DIR: no rail, no frame cap, an encoder
        // pipe waiting forever on stdin while the sim free-ran and re-solved the wave field
        // every 90 s -- "the recording is stuck". A token that starts with "--" is a flag.
        auto next = [&](const char* def) -> std::string {
            if (i + 1 < argc && !(argv[i + 1][0] == '-' && argv[i + 1][1] == '-')) {
                return argv[++i];
            }
            return def;
        };
        if (a == "--selftest") o.selftest = true;
        else if (a == "--crash-test") {
            // M7v: prove the crash tracer end to end -- the only honest test of a crash
            // handler is a crash.
            volatile int* p = nullptr;
            *p = 1;
        }
        else if (a == "--headless") o.headless = true;
        else if (a == "--debug") o.debugLayer = true;
        else if (a == "--width") o.width = static_cast<uint32_t>(atoi(next("1600").c_str()));
        else if (a == "--height") o.height = static_cast<uint32_t>(atoi(next("900").c_str()));
        else if (a == "--frames") {
            o.frames = static_cast<uint32_t>(atoi(next("1").c_str()));
            o.framesSet = true;
        }
        else if (a == "--dump") o.dump = Widen(next("out.png").c_str());
        else if (a == "--shaders") o.shaderDir = Widen(next("shaders").c_str());
        else if (a == "--tides") o.tidesPath = next("data/tides/stations.json");
        else if (a == "--seastate") o.seaPath = next("data/sea/seastate.json");
        else if (a == "--currents") o.currentsPath = next("data/currents/currents.json");
        else if (a == "--sea") o.seaStart = true;
        else if (a == "--gulf") o.gulfStart = true;
        else if (a == "--globe") o.globeStart = true;
        else if (a == "--globe-cam") {
            // lat,lon,alt_km -- headless verification shots aim at the planet centre
            const std::string v = next("34,-52,13000");
            sscanf_s(v.c_str(), "%f,%f,%f", &o.gcamLat, &o.gcamLon, &o.gcamAltKm);
        }
        else if (a == "--start") o.startUnix = ParseStartTime(next("0"));
        else if (a == "--scale") o.timeScale = atof(next("1").c_str());
        else if (a == "--rail") o.rail = Widen(next("rail_frames").c_str());
        else if (a == "--rail-zoom") { o.rail = Widen(next("rail_frames").c_str()); o.railZoom = true; }
        else if (a == "--rail-flood") { o.rail = Widen(next("rail_frames").c_str()); o.railFlood = true; }
        else if (a == "--rail-jetty") { o.rail = Widen(next("rail_frames").c_str()); o.railJetty = true; }
        else if (a == "--trace") {
            // M7j: THE HYPERVISOR. One sample walked through the whole one-water chain on
            // the CPU, every transformation printed with its AST edge -- validate against
            // external tools, find the failing step BEFORE the GPU is involved.
            if (swscanf(Widen(next("42.816,-70.81").c_str()).c_str(), L"%lf,%lf",
                        &o.traceLat, &o.traceLon) != 2) {
                o.traceLat = 42.816; o.traceLon = -70.81;
            }
            o.trace = true;
        }
        else if (a == "--pix") {
            o.pixFrames = static_cast<uint32_t>(_wtoi(Widen(next("1").c_str()).c_str()));
            if (o.pixFrames == 0) o.pixFrames = 1;
        }
        else if (a == "--dump-fibers") o.dumpFibers = true;
        else if (a == "--lens") {
            const std::string n = next("worldxz");
            o.lens = n == "worldxz" ? 1 : n == "winuv" ? 2 : n == "mip" ? 3
                     : n == "ring" ? 4 : n == "cascade" ? 5
                     : n == "waterdata" ? 6 : n == "velgrad" ? 7 : n == "shell" ? 8 : 1;
        }
        else if (a == "--probe-cull-far") o.probeCullFar = true;
        else if (a == "--dump-meshlets") o.dumpMeshlets = Widen(next("meshlets.bin").c_str());
        else if (a == "--dump-water-state") o.dumpWater = true;
        else if (a == "--slice") {
            o.sliceOn = true;
            o.sliceD = _wtof(Widen(next("0").c_str()).c_str());
        }
        else if (a == "--inject") {
            const std::string n = next("bank");
            o.inject = n == "cascade" ? 2 : 1;
        }
        else if (a == "--warm-inlet") o.warmInlet = true;
        else if (a == "--planet") o.planet = next("earth");
        else if (a == "--tile-budget") o.tileBudget = static_cast<uint32_t>(atoi(next("1000").c_str()));
        else if (a == "--exagg") o.exaggeration = static_cast<float>(atof(next("60").c_str()));
        else if (a == "--window-days") o.windowDays = atof(next("7").c_str());
        else if (a == "--fov") o.fovDeg = static_cast<float>(atof(next("55").c_str()));
        else if (a == "--foam") o.foam = static_cast<float>(atof(next("1").c_str()));
        else if (a == "--edge-px") o.edgePx = static_cast<float>(atof(next("12").c_str()));
        else if (a == "--height-scale") o.heightScale = static_cast<float>(atof(next("1.15").c_str()));
        else if (a == "--sea-verify") o.seaVerify = true;
        else if (a == "--viz") o.viz = true;
        else if (a == "--wireframe") o.surfaceDebug = 1;
        // M9s: encode the rail straight to mp4, no PNG frames at all.
        else if (a == "--mp4") o.mp4 = next("out.mp4");
        // M9t: fly the rail and measure it, capturing NOTHING. See the note at the timing site
        // for why this is not the same as reading renderMs out of a captured run.
        else if (a == "--bench") o.bench = true;
        // --bench-overlap: the same flight with the CPU/GPU overlap the shipped loop keeps. The
        // fenced --bench prints CPU + GPU in series; this prints what a player's frame costs.
        else if (a == "--bench-overlap") { o.bench = true; o.benchOverlap = true; }
        // Instruments: GPU time per pass (timestamp queries, read frames-in-flight deep, no
        // stall) and a tearing present for the windowed frame-rate question.
        else if (a == "--gpu-time") o.gpuTime = true;
        else if (a == "--no-vsync") o.noVsync = true;
        // --settle-sync: a still's dump frame is held (the sim clock frozen at its instant, the
        // way --dump-both holds it) until nothing is still landing -- the residency queues empty,
        // no DirectStorage batch in flight, no ring-held request -- for kEvictAgeFrames frames,
        // so when the far tiles happened to arrive stops deciding the still. MEASURED (helm,
        // 2026-09-05): held pairs agree to 2-13 px; unheld pairs to 5-18 px or one far-water
        // tile (~870 px) -- the horizon-line residue is not the residency's (probe P16 stays open).
        else if (a == "--settle-sync") o.settleSync = true;
        // --settle-hold N: the same hold, COUNTED. --settle-sync's length is whatever the
        // residency needs, and the two binaries of an A/B rarely need the same (211 vs 225
        // frames on two runs of one binary; 300 vs 900 across a change) -- so two settled
        // stills were taken after different numbers of frames. Held exactly N frames past the
        // dump instant whatever residency is doing, N is a flag both sides share. With
        // --settle-sync it composes: drain first, then hold to at least N. Honoured past the
        // 3000-frame drain cap, which is a give-up rule for the drain, not a budget.
        else if (a == "--settle-hold")
            o.settleHold = static_cast<uint32_t>(atoi(next("300").c_str()));
        // --settle-exact: the same clock hold and churn freeze, exited only when the resident
        // set IS the walk's want set -- every wanted tile mapped at its mip, every mapped tile
        // the walk does not want dropped, nothing pending, in flight or retiring, for
        // kEvictAgeFrames + 4 consecutive turns (Residency.h settleExact). --settle-sync's
        // quiet test fires over two different resident sets (step 21's first attempt: two
        // quiet bird holds differed in whole tiles' mips); this one names the set. With
        // --settle-hold N it composes: exact first, then to at least N.
        else if (a == "--settle-exact") o.settleExact = true;
        // --settle-clear-churn: the churn atlas is zeroed at the first held frame, so the still
        // carries no foam deposited during the real frames (which lands when the bed and the
        // wave pages happen to). The A/B of a held still with and without it tells the
        // residency's share of a residual from the churn history's (SeaLayer.h ClearChurn).
        else if (a == "--settle-clear-churn") o.settleClearChurn = true;
        else if (a == "--dump-hdr") o.dumpHdr = Widen(next("out.rgba16f").c_str());
        // M9ah: pack every realization's loose tiles into one archive and exit.
        else if (a == "--pack-tiles") o.packTiles = true;
        // M9ai: route archived tile reads NVMe -> GPU. Off by default until the streamed
        // path is proven pixel-equal to the upload-ring path it replaces.
        else if (a == "--direct-storage") o.directStorage = true;    // the default; kept
        else if (a == "--no-direct-storage") o.directStorage = false;
        else if (a == "--ds-serial") o.dsSerial = true;
        // M9aj: the three trees. --color-trees composes colour FROM the per-source trees
        // instead of from the sources; --tree-audit measures the two answers against each
        // other on tiles the shipped path already painted, then exits.
        else if (a == "--color-trees") o.colorTrees = true;      // the default; kept for scripts
        else if (a == "--no-color-trees") o.colorTrees = false;
        // M9ak: the vector land/sea gate is ON. The flag exists to A/B what it changed.
        else if (a == "--no-gis-gate") o.gisGate = false;
        else if (a == "--no-seafloor") o.seafloor = false;
        else if (a == "--no-exposure") o.exposure = false;
        else if (a == "--gis-dump") o.gisDump = next("gis_gate.pgm");
        // M9al: the ring gate and its instrument. Instrument first, gate second, both off.
        else if (a == "--ring-loads") o.ringLoads = true;      // the default; kept for scripts
        else if (a == "--no-ring-loads") o.ringLoads = false;
        else if (a == "--res-trace") o.resTrace = true;
        // The tree's thread instrument (core/ThreadAudit.h): every tile write, read and delete
        // is scoped, and a run reports how often two threads met at one path. Off by default --
        // it takes a mutex per tile file, which is a different landing schedule.
        else if (a == "--thread-audit") o.threadAudit = true;
        // The pool's A/B, the same shape as --predict-inline: every Submit and ParallelFor runs
        // on the calling thread in submission order, so a threading difference shows as a
        // difference against a single-threaded reference rather than as a mystery pixel.
        else if (a == "--jobs-inline") o.jobsInline = true;
        // Step 28: --res-trace-frames A:B prints the residency turn's landing ledger on every
        // recorded frame of [A, B]: what landed, from where, and what the landing buffer had
        // left. On its own -- not under --res-trace, whose audit moves the loop's timing.
        else if (a == "--res-trace-frames") {
            const std::string v = next("740:800");
            unsigned f0 = 0, f1 = 0;
            if (sscanf_s(v.c_str(), "%u:%u", &f0, &f1) == 2) {
                o.traceFrom = f0;
                o.traceTo = f1;
            }
        }
        else if (a == "--tree-audit") o.treeAudit = uint32_t(atoi(next("400").c_str()));
        // After a source is added there is nothing to compare against -- which is exactly when
        // the trees most need building. --warm-trees composes every address regardless.
        else if (a == "--warm-trees") { o.warmTrees = true; o.treeAudit = 1000000u; }
        // M9ao: pack every node of the megatexture tree so a tile -- or a reference to one --
        // resolves to a place DirectStorage can read. Needs the graph, so it runs after it.
        else if (a == "--pack-trees") { o.packTrees = true; o.treeAudit = 1u; }
        else if (a == "--predict-every") o.predictEvery = uint32_t(atoi(next("1").c_str()));
        else if (a == "--predict-inline") o.predictInline = true;
        // M9p: replace the bed with a flat floor at this NAVD height. The A/B against a normal
        // run isolates BATHYMETRY's contribution to the geometry from everything else.
        else if (a == "--flat-bed") { o.flatBed = true; o.flatBedNavd = float(atof(next("-30").c_str())); }
        else if (a == "--dump-both") o.dumpBoth = true;
        else if (a == "--load-field") o.loadField = next("");
        else if (a == "--meshlets") o.surfaceDebug = 2;
        else if (a == "--mesh-stats") o.meshStats = true;
        else if (a == "--stencil") o.stencil = true;
        else if (a == "--no-ms") o.msSurface = false;
        else if (a == "--albedo") o.albedo = true;
        else if (a == "--export") {
            // M6j utility: pull a composed channel OUT through the manager -- the same
            // provider path the renderer streams. <channel>[:mip] <out.(png|raw|obj)>
            o.exportSpec = next("earth.color.window:2");
            o.exportOut = Widen(next("export.png").c_str());
        }
        else if (a == "--cam") {
            const std::string v = next("12,246,-5");
            sscanf_s(v.c_str(), "%f,%f,%f", &o.camAlt, &o.camAz, &o.camPitch);
        }
        else if (a == "--campos") {
            const std::string v = next("522,72");
            sscanf_s(v.c_str(), "%f,%f", &o.camX, &o.camZ);
        }
        else if (a == "--view") {
            // A camera the user saved with F5: the same five numbers --cam/--campos take,
            // looked up by name so a pose survives the session it was found in.
            o.view = next("view-1");
            // Views the user asked to KEEP live here, not in the gitignored data/ folder.
            // east, alt, north, azimuth (compass), pitch -- SetFromCompass's own order.
            struct BuiltInView { const char* name; float x, alt, z, az, pitch; };
            static const BuiltInView kBuiltInViews[] = {
                // On the north jetty a little in from its tip, looking back along it toward
                // the range tower (saved with F5 2026-09-01: "keep this!").
                {"jetty-north", 541.20f, 7.00f, 72.52f, 246.0f, -4.0f},
                // Off the jetty tips looking west into the entrance, three heights.
                {"entrance-low", 900.0f, 40.0f, -10.0f, 270.0f, -10.0f},
                {"entrance-mid", 1200.0f, 120.0f, -10.0f, 270.0f, -18.0f},
                {"entrance-high", 1500.0f, 300.0f, -10.0f, 270.0f, -28.0f},
            };
            bool found = false;
            for (const BuiltInView& b : kBuiltInViews) {
                if (o.view != b.name) continue;
                o.camX = b.x; o.camAlt = b.alt; o.camZ = b.z; o.camAz = b.az; o.camPitch = b.pitch;
                found = true;
            }
            std::ifstream vf("data/views.json", std::ios::binary);
            const std::string text((std::istreambuf_iterator<char>(vf)),
                                   std::istreambuf_iterator<char>());
            std::string err;
            const JsonValue root = JsonParser::Parse(text, &err);
            const JsonValue* views = err.empty() ? root.Get("views") : nullptr;
            if (views) {   // the file may override a built-in of the same name
                for (const JsonValue& v : views->arr) {
                    if (v.Str("name") != o.view) continue;
                    o.camX = static_cast<float>(v.Num("x", 0.0));
                    o.camAlt = static_cast<float>(v.Num("alt", 2.0));
                    o.camZ = static_cast<float>(v.Num("z", 0.0));
                    o.camAz = static_cast<float>(v.Num("az", 90.0));
                    o.camPitch = static_cast<float>(v.Num("pitch", 0.0));
                    found = true;
                }
            }
            if (!found) {
                fprintf(stderr, "--view %s: not built in and not in data/views.json (press F5 "
                        "in the viewer to save one)\n", o.view.c_str());
                exit(2);
            }
        }
        else if (a == "--bathy") o.bathyPath = next("data/bathy/merrimack.json");
        else if (a == "--datum") {
            o.datumOff = static_cast<float>(atof(next("-1.30").c_str()));
            o.datumSet = true;
        }
        else if (a == "--swe-off") o.sweOff = true;
        else if (a == "--swe-west-off") o.sweWestOff = true;   // diagnostic: west strip = ocean clock
        else if (a == "--swe-uv") o.sweUvDump = Widen(next("swe_uv.png").c_str());
        else if (a == "--swe-gain") o.sweGain = static_cast<float>(atof(next("3.2").c_str()));
        else if (a == "--swe-spinup") o.sweSpinupH = atof(next("0.25").c_str());
        else if (a == "--swe-cycle") o.sweCycleH = atof(next("13").c_str());
        else if (a == "--water-map") o.waterMap = Widen(next("water_map.png").c_str());
        else if (a == "--bathy-map") o.bathyMap = Widen(next("bathy_map.png").c_str());
        else if (a == "--fidelity-map") {
            o.fidelityMap = Widen(next("fidelity_map.png").c_str());
        }
        else if (a == "--ocean-probe") o.oceanProbe = next("42.35,-70.65");
        else if (a == "--one-water") o.oneWater = true;
        else if (a == "--pixel-water") o.pixelWater = true;
        else if (a == "--sun") {
            const std::string v = next("112,26");
            sscanf_s(v.c_str(), "%f,%f", &o.sunAz, &o.sunEl);
            o.sunPinned = true;
        }
        else if (a == "--river") o.riverQ = atof(next("70").c_str());
        else if (a == "--storm") {
            // hs,tp,fromdeg -- sandbox sea state override
            const std::string v = next("4,11,80");
            sscanf_s(v.c_str(), "%f,%f,%f", &o.stormHs, &o.stormTp, &o.stormDir);
        }
        else Log("[args] ignoring '%s'", a.c_str());
    }
    if (!o.dump.empty() && o.frames == 0) o.frames = 1;
    if (!o.dump.empty()) o.headless = true;
    if (!o.fidelityMap.empty()) {   // M8j: a registry picture never opens a window
        o.headless = true;
        o.frames = 1;
    }
    if (o.sweCycleH > 0) o.headless = true;   // the validation cycle never opens a window
    if (!o.rail.empty()) {
        // The rails demos: headless, deterministic real-time waves, 30 fps. Classic = 25 s;
        // the zoom and Mars flyover run 30 s; the flood ride holds the helm for 40 s total.
        o.headless = true;
        o.globeStart = true;
        // An explicit --frames wins: dumping a rail at a CHOSEN moment is how you inspect
        // something a viewer noticed at 0:12 rather than guessing camera arguments for it.
        if (!o.framesSet) o.frames = o.railJetty                             ? 40 * 30
                                : o.railFlood                        ? 40 * 30
                                : (o.railZoom || o.planet == "mars") ? 30 * 30
                                                                     : 25 * 30;
        o.timeScale = 1.0;
    }
    if (o.planet == "mars") o.globeStart = true;   // there is only orbit on Mars (for now)
    return o;
}

// M5c: latest Merrimack discharge for the SWE's upstream boundary. Lawrence when it reports;
// else Lowell scaled up ~8% for the intervening drainage (Shawsheen, Spicket, Little); else a
// low-summer climatological 70 m3/s. The ~20 h Lawrence-to-mouth travel time is ignored -- at
// summer flows the tidal prism dwarfs it.
double LoadRiverDischarge(const std::string& path) {
    std::ifstream f(path, std::ios::binary);
    if (!f) {
        Log("[river] no %s (run: py -3 harvester\\harvest_river.py); climatological 70 m3/s",
            path.c_str());
        return 70.0;
    }
    std::string text((std::istreambuf_iterator<char>(f)), std::istreambuf_iterator<char>());
    std::string err;
    const JsonValue root = JsonParser::Parse(text, &err);
    double lowell = -1, lawrence = -1;
    if (const JsonValue* sites = root.Get("sites")) {
        for (const auto& s : sites->arr) {
            const std::string id = s.Str("id");
            if (id == "01100000") lowell = s.Num("discharge_cms", -1);
            if (id == "01100500") lawrence = s.Num("discharge_cms", -1);
        }
    }
    double q = 70.0;
    const char* src = "climatology (no gauge value)";
    if (lawrence > 0) {
        q = lawrence;
        src = "USGS 01100500 Lawrence";
    } else if (lowell > 0) {
        q = lowell * 1.08;
        src = "USGS 01100000 Lowell +8% drainage";
    }
    Log("[river] discharge %.1f m3/s (%s)", q, src);
    return q;
}

// M5c: the MLLW -> NAVD88 join from CO-OPS station datums instead of a hand constant. Use the
// focus station's own NAVD link when CO-OPS publishes one; otherwise transfer via the relation
// NAVD88 = local MSL + delta, calibrated from whichever harvested station carries both datums
// (Boston: delta = +0.09 m) -- picking the smallest |delta| candidate, since NAVD tracks MSL on
// the open coast while upriver stations drift.
// ================================================================================================
// M6j --export: the manager as a DATA INTERFACE, not just a renderer's feeder. A composed
// channel is pulled tile by tile through the exact provider path the renderer streams
// (cache-first: a warmed machine exports offline) and written as:
//   .png  color RGBA, or height normalized to grayscale
//   .raw  height as row-major float32 (plus dims in the log)
//   .obj  height triangulated as a mesh in local metres -- the same tiles the mesh-shader
//         surface consumes, hand-inspectable in any DCC tool
// ================================================================================================
static int RunChannelExport(const std::string& spec, const std::wstring& outPath,
                            Compositor& comp, int colCh, int hgtCh) {
    std::string name = spec;
    uint32_t mip = 2;
    if (const size_t c = spec.find(':'); c != std::string::npos) {
        name = spec.substr(0, c);
        mip = static_cast<uint32_t>(atoi(spec.c_str() + c + 1));
    }
    TileProviderFn fn;
    bool height = false;
    uint32_t tileW = 128, tileH = 128, face = 0;
    if (name == "earth.color.window" && colCh >= 0) {
        fn = comp.WindowColor(colCh, 1263360, 1538048, 16384, 14);
    } else if (name == "earth.color.inlet" && colCh >= 0) {
        // M6l: a z19 export-only realization (~22 cm ground at this latitude) centred on the
        // MassGIS ortho coverage -- deep enough to JUDGE the 15 cm aerial layer's painting.
        fn = comp.WindowColor(colCh, 40699567, 49405858, 16384, 19);
    } else if (name == "earth.height.window" && hgtCh >= 0) {
        fn = comp.WindowHeight(hgtCh, 1263360, 1538048, 16384, 14);
        height = true;
        tileW = 256;
    } else if (name.rfind("earth.color.cube.f", 0) == 0 && colCh >= 0) {
        face = static_cast<uint32_t>(name.back() - '0') % 6;
        fn = comp.CubeColor(colCh);
    } else if ((name.rfind("earth.height.cube.f", 0) == 0 ||
                name.rfind("mars.height.cube.f", 0) == 0) &&
               hgtCh >= 0) {
        face = static_cast<uint32_t>(name.back() - '0') % 6;
        fn = comp.CubeHeight(hgtCh);
        height = true;
        tileW = 256;
    } else {
        Log("[export] unknown channel '%s' (or its stack is not configured). Channels: "
            "earth.color.window, earth.color.inlet (z19), earth.height.window, "
            "earth.color.cube.f0..5, earth|mars.height.cube.f0..5",
            name.c_str());
        return 1;
    }
    const uint32_t dim = Compositor::kFaceDim >> mip;
    if (dim > 4096) {
        Log("[export] mip %u is %ux%u -- use mip >= 2 (<= 4096 wide)", mip, dim, dim);
        return 1;
    }
    const uint32_t tilesX = dim / tileW, tilesY = dim / tileH;
    Log("[export] %s mip %u: %ux%u texels, %u tiles (cache-first through the manager)",
        name.c_str(), mip, dim, dim, tilesX * tilesY);

    std::vector<float> hgtData;
    std::vector<uint8_t> rgba;
    if (height) hgtData.resize(static_cast<size_t>(dim) * dim);
    else rgba.resize(static_cast<size_t>(dim) * dim * 4);
    std::vector<uint8_t> tile;
    for (uint32_t ty = 0; ty < tilesY; ++ty) {
        for (uint32_t tx = 0; tx < tilesX; ++tx) {
            if (!fn({face, mip, tx, ty}, tile, nullptr) || tile.size() != 65536) continue;
            for (uint32_t py = 0; py < tileH; ++py) {
                const size_t row = static_cast<size_t>(ty) * tileH + py;
                if (height) {
                    const uint16_t* src = reinterpret_cast<const uint16_t*>(tile.data());
                    for (uint32_t px = 0; px < tileW; ++px) {
                        hgtData[row * dim + tx * tileW + px] =
                            HalfToFloat(src[py * tileW + px]);
                    }
                } else {
                    memcpy(&rgba[(row * dim + tx * tileW) * 4], &tile[py * tileW * 4],
                           static_cast<size_t>(tileW) * 4);
                }
            }
        }
    }

    const std::wstring ext =
        outPath.size() > 4 ? outPath.substr(outPath.size() - 4) : std::wstring();
    if (ext == L".obj") {
        if (!height) {
            Log("[export] .obj export needs a HEIGHT channel");
            return 1;
        }
        if (dim > 1024) {
            Log("[export] .obj at %u^2 would be %u M verts -- use mip >= 4", dim,
                dim * dim / 1000000);
            return 1;
        }
        const double worldPx = static_cast<double>((1ll << 14) * 256ll >> mip);
        const double ground = 40075016.686 / worldPx * std::cos(42.8 * 3.14159265 / 180.0);
        FILE* f = nullptr;
        _wfopen_s(&f, outPath.c_str(), L"w");
        if (!f) return 1;
        fprintf(f, "# GAGAME composed-channel export: %s mip %u (%u^2, %.2f m/px)\n",
                name.c_str(), mip, dim, ground);
        for (uint32_t y = 0; y < dim; ++y) {
            for (uint32_t x = 0; x < dim; ++x) {
                fprintf(f, "v %.2f %.2f %.2f\n", (static_cast<double>(x) - dim / 2.0) * ground,
                        hgtData[static_cast<size_t>(y) * dim + x],
                        (dim / 2.0 - static_cast<double>(y)) * ground);
            }
        }
        for (uint32_t y = 0; y + 1 < dim; ++y) {
            for (uint32_t x = 0; x + 1 < dim; ++x) {
                const uint32_t a = y * dim + x + 1, b = a + 1, c = a + dim, d = c + 1;
                fprintf(f, "f %u %u %u\nf %u %u %u\n", a, b, c, b, d, c);
            }
        }
        fclose(f);
        Log("[export] wrote %S (%u verts, %u tris)", outPath.c_str(), dim * dim,
            (dim - 1) * (dim - 1) * 2);
    } else if (ext == L".raw") {
        if (!height) {
            Log("[export] .raw export is for height channels; color goes to .png");
            return 1;
        }
        FILE* f = nullptr;
        _wfopen_s(&f, outPath.c_str(), L"wb");
        if (!f) return 1;
        fwrite(hgtData.data(), 4, hgtData.size(), f);
        fclose(f);
        Log("[export] wrote %S (float32 row-major, %ux%u, row 0 north)", outPath.c_str(), dim,
            dim);
    } else {
        if (height) {
            float lo = 1e9f, hi = -1e9f;
            for (const float v : hgtData) {
                lo = (std::min)(lo, v);
                hi = (std::max)(hi, v);
            }
            rgba.resize(static_cast<size_t>(dim) * dim * 4);
            const float inv = hi > lo ? 255.0f / (hi - lo) : 0.0f;
            for (size_t i = 0; i < hgtData.size(); ++i) {
                const uint8_t g = static_cast<uint8_t>((hgtData[i] - lo) * inv);
                rgba[i * 4] = rgba[i * 4 + 1] = rgba[i * 4 + 2] = g;
                rgba[i * 4 + 3] = 255;
            }
            Log("[export] height range %.1f .. %.1f m (normalized to grayscale)", lo, hi);
        }
        SavePng(outPath, rgba.data(), dim, dim, dim * 4, rgba.size());
        Log("[export] wrote %S", outPath.c_str());
    }
    return 0;
}

// M9n: NAVD88 zero above LOCAL MSL, in metres, from published CO-OPS station datums -- the same
// staff carries MSL and NAVD88, so their difference is data, not an assumption. Boston: MSL
// 2.660, NAVD88 2.752 -> +0.092. Returns false when no station in the table publishes both.
//
// The bed's ETOPO layers are MSL-referenced and need the NEGATIVE of this to reach NAVD88; the
// tide's MLLW link is a different offset from the same table. One derivation, two consumers.
bool NavdAboveMsl(const TideModel& model, double* outM, std::string* from) {
    double best = 1e9;
    bool got = false;
    for (size_t i = 0; i < model.Count(); ++i) {
        const TideStation& s = model.S(i);
        if (s.mllwMinusNavdM <= -900.0) continue;
        const double d = -s.mllwMinusNavdM - s.meanMllwM;   // NAVD88 zero above local MSL
        if (!got || std::abs(d) < std::abs(best)) {
            best = d;
            got = true;
            if (from) *from = s.name;
        }
    }
    if (got && outM) *outM = best;
    return got;
}

// ================================================================================================
//  M9s: THE RECORDER, ENCODING AS IT GOES.
//
//  A rail used to write 1200 PNGs and then run ffmpeg over them. The rail metrics priced that
//  honestly for the first time: 1.7 ms to RENDER a frame, ~90 ms to PNG it. Fifty-three parts
//  in fifty-four of a recording's wall clock went into compressing intermediates that were
//  deleted immediately afterwards -- and it also meant the mp4 could not exist until every
//  frame had hit the disk twice.
//
//  So the frames go straight into an encoder over a pipe: no PNG, no intermediate directory,
//  no second pass. NVENC where the GPU offers it (it is already rendering; the encoder blocks
//  are idle), libx264 where it does not. rawvideo in, mp4 out, one process for the whole rail.
//
//  Deliberately ffmpeg over a pipe rather than linking an encoder: it keeps the codec choice
//  a runtime decision, costs nothing when unused, and a recorder that shells out is a recorder
//  that cannot corrupt the renderer.
// ================================================================================================
class FramePipe {
public:
    bool Open(const std::string& path, uint32_t w, uint32_t h, uint32_t fps) {
        m_w = w;
        m_h = h;
        const Codec& c = PickCodec();
        char cmd[1024];
        snprintf(cmd, sizeof(cmd),
                 "ffmpeg -hide_banner -loglevel warning -y -f rawvideo -pixel_format rgba "
                 "-video_size %ux%u -framerate %u -i - %s -pix_fmt yuv420p \"%s\"",
                 w, h, fps, c.args, path.c_str());
        m_f = _popen(cmd, "wb");
        if (!m_f) {
            Log("[rec] could not start ffmpeg -- falling back to PNG frames");
            return false;
        }
        Log("[rec] encoding %ux%u @%u fps straight from the framebuffer via %s -> %s", w, h, fps,
            c.label, path.c_str());
        return true;
    }
    bool Open() const { return m_f != nullptr; }

    // The readback row pitch is aligned and usually exceeds width*4, so rows are written one at
    // a time. Writing the whole buffer would shear the picture by the padding, every frame.
    void Write(const std::vector<uint8_t>& px, uint32_t rowPitch) {
        if (!m_f) return;
        const size_t row = size_t(m_w) * 4;
        size_t wrote = 0;
        for (uint32_t y = 0; y < m_h; ++y) {
            const size_t off = size_t(y) * rowPitch;
            if (off + row > px.size()) break;
            wrote += fwrite(px.data() + off, 1, row, m_f);
        }
        if (!m_reported) {
            m_reported = true;
            Log("[rec] first frame: %zu px bytes, pitch %u, wrote %zu of %zu expected%s",
                px.size(), rowPitch, wrote, row * m_h,
                ferror(m_f) ? " -- PIPE ERROR" : "");
        }
    }
    void Close() {
        if (!m_f) return;
        _pclose(m_f);
        m_f = nullptr;
        Log("[rec] encoder closed");
    }

private:
    struct Codec {
        const char* args;
        const char* label;
    };

    // PROBE BY ENCODING, NOT BY LISTING. The first version of this asked `ffmpeg -encoders` for
    // h264_nvenc, found it, and produced a ZERO-BYTE mp4: the encoder is compiled in and listed,
    // but this machine's driver exposes NVENC API 13.0 while that ffmpeg build requires 13.1, so
    // it fails at open. "Is it listed" and "does it work" are different questions and only the
    // second one matters -- so the probe actually encodes one frame to null and checks the exit
    // status. It costs a fraction of a second, once, and it means the day the driver is updated
    // NVENC starts being used with no code change at all.
    static const Codec& PickCodec() {
        static const Codec kCandidates[] = {
            {"-c:v h264_nvenc -preset p5 -rc vbr -cq 21 -b:v 0", "h264_nvenc (NVIDIA GPU)"},
            {"-c:v h264_qsv -global_quality 21", "h264_qsv (Intel GPU)"},
            {"-c:v h264_amf -quality balanced -rc cqp -qp_i 21 -qp_p 21", "h264_amf (AMD GPU)"},
            {"-c:v libx264 -preset veryfast -crf 20", "libx264 (CPU)"},
        };
        static const Codec* chosen = nullptr;
        if (chosen) return *chosen;
        for (const Codec& c : kCandidates) {
            char probe[512];
            snprintf(probe, sizeof(probe),
                     "ffmpeg -hide_banner -loglevel error -f lavfi "
                     "-i color=c=black:s=64x64:d=0.1 -frames:v 1 %s -f null - >nul 2>&1",
                     c.args);
            if (system(probe) == 0) {
                chosen = &c;
                if (&c != &kCandidates[0]) {
                    Log("[rec] %s unavailable here (encoder present but fails to open -- on this "
                        "machine the driver exposes an older NVENC API than this ffmpeg needs); "
                        "using %s",
                        "h264_nvenc", c.label);
                }
                return *chosen;
            }
        }
        chosen = &kCandidates[3];   // libx264 is always the last word
        return *chosen;
    }
    FILE* m_f = nullptr;
    uint32_t m_w = 0, m_h = 0;
    bool m_reported = false;
};

float ResolveDatum(const TideModel& model) {
    const TideStation& fs = model.S(model.Focus());
    if (fs.mllwMinusNavdM > -900.0) {
        Log("[datum] %s CO-OPS datums: MLLW - NAVD88 = %+.3f m", fs.name.c_str(),
            fs.mllwMinusNavdM);
        return static_cast<float>(fs.mllwMinusNavdM);
    }
    double delta = 0.0;
    std::string from = "NAVD=MSL assumption";
    NavdAboveMsl(model, &delta, &from);
    const float off = static_cast<float>(-(fs.meanMllwM + delta));
    Log("[datum] no NAVD link at %s; MLLW - NAVD88 = %+.3f m via NAVD=MSL%+.3f (from %s)",
        fs.name.c_str(), off, delta, from.c_str());
    return off;
}

// M5c validation harness: integrate a full tidal cycle headless and log probes to CSV --
// ocean sponge (must track the analytic tide), the entrance throat AT the ACT0816 station
// (solved current vs the CO-OPS prediction, the real gate), the Joppa Flats basin (lag +
// attenuation must EMERGE), and the river's standing slope upstream.
template <typename F, typename G, typename H, typename Q>
void RunSweCycle(Gpu& gpu, SweSolver& swe, F oceanAt, G westAt, H southAt, Q westQAt,
                 const CurrentModel* currents, int ctSta, const BathyModel& bathy,
                 double startUnix, double hours) {
    auto tideAt = oceanAt;

    // River probe: the deepest channel cell near x = -5000 m.
    const int nx = bathy.Nx(), ny = bathy.Ny();
    const int ix = static_cast<int>((-5000.0f - bathy.WorldX0()) / bathy.WorldSizeX() * nx);
    int iyBest = ny / 2;
    float eBest = 1e9f;
    for (int iy = 0; iy < ny; ++iy) {
        const float e = bathy.Elev()[iy * nx + ix];
        if (e > -9000.0f && e < eBest) {
            eBest = e;
            iyBest = iy;
        }
    }
    const float riverZ = bathy.WorldZ0() + bathy.WorldSizeZ() * (1.0f - (iyBest + 0.5f) / ny);
    Log("[swe-cycle] river probe (-5000, %.0f), bed %.1f m", riverZ, eBest);

    // Throat probe: the deepest channel cell within 150 m of the ACT0816 station -- the jet
    // core, not whatever shoal the exact origin lands on.
    float throatX = 0, throatZ = 0, tBest = 1e9f;
    for (float wz = -150; wz <= 150; wz += 10) {
        for (float wx = -150; wx <= 150; wx += 10) {
            const float e = bathy.SampleWorld(wx, wz);
            if (e > -9000.0f && e < tBest) {
                tBest = e;
                throatX = wx;
                throatZ = wz;
            }
        }
    }
    Log("[swe-cycle] throat probe (%.0f, %.0f), bed %.1f m", throatX, throatZ, tBest);

    const char* path = "data/swe_cycle.csv";
    FILE* f = nullptr;
    fopen_s(&f, path, "w");
    if (!f) {
        Log("[swe-cycle] cannot write %s", path);
        return;
    }
    fprintf(f,
            "unix,tide_navd,act_pred_ms,ocean_deta,throat_deta,throat_u,throat_v,basin_deta,"
            "river_deta,west_target,gap_u\n");
    const double endUnix = startUnix + hours * 3600.0;
    int rows = 0;
    for (double t = startUnix; t <= endUnix; t += 120.0) {
        swe.AdvanceTo(gpu, t, tideAt, westAt, southAt, westQAt);
        // 4 named probes + a 6-point transect across the jetty gap (x = 350) whose valid-point
        // mean is the fair section current to hold against ACT0816.
        const float pts[20] = {2600.0f, 0.0f,   throatX, throatZ, -2500.0f, -900.0f,
                               -5000.0f, riverZ, 350.0f, -350.0f, 350.0f,  -250.0f,
                               350.0f,  -150.0f, 350.0f, -50.0f,  350.0f,  50.0f,
                               350.0f,  150.0f};
        SweSolver::Probe pr[10];
        swe.ReadProbes(gpu, pts, 10, pr);
        const SweSolver::Probe &ocean = pr[0], &throat = pr[1], &basin = pr[2], &river = pr[3];
        float gapU = 0;
        int gapN = 0;
        for (int gi = 4; gi < 10; ++gi) {
            if (pr[gi].valid) {
                gapU += pr[gi].u;
                ++gapN;
            }
        }
        gapU = gapN ? gapU / gapN : 0.0f;
        const double act =
            (currents && ctSta >= 0) ? currents->SignedSpeed(static_cast<size_t>(ctSta), t) : 0.0;
        fprintf(f, "%.0f,%.4f,%.3f,%.4f,%.4f,%.3f,%.3f,%.4f,%.4f,%.4f,%.3f\n", t, tideAt(t),
                act, ocean.dEta, throat.dEta, throat.u, throat.v, basin.dEta, river.dEta,
                westAt(t), gapU);
        if (++rows % 30 == 0) {
            Log("[swe-cycle] +%.1f h  tide %+.2f  gap u %+.2f (ACT %+.2f)  basin dEta %+.3f",
                (t - startUnix) / 3600.0, tideAt(t), gapU, act, basin.dEta);
        }
    }
    fclose(f);
    Log("[swe-cycle] wrote %s (%d rows)", path, rows);
}

// ================================================================================================
//  M8j: --fidelity-map -- THE HETEROGENEITY SHEET. The registry drawn as a picture.
//
//  Every source in this engine declares the same four things (Compositor.h SourceInfo): what it
//  is, what CRS it speaks, its FINEST GROUND RESOLUTION in cm/px, and its COVERAGE box. That
//  declaration is not decoration -- SourceTouches() decides cache identity from it, so a source
//  that lies about its box paints the wrong tiles. This map renders exactly that contract:
//
//    hue        WHICH source is finest at this point (the winner of the stack)
//    intensity  HOW fine it is -- log10(cm/px) from 500 km (the analytic equilibrium tide)
//               to 15 cm (the MassGIS ortho), five and a half orders of magnitude
//
//  So "more colour = better data" reads literally, and the sparse-HQ-inset thesis becomes
//  visible: a dim global wash with bright islands where we have paid for detail.
//
//  THE LADDER IS THE POINT. A single global sheet cannot show this -- the 15 cm ortho covers
//  0.018 degrees and would occupy a third of one pixel. So the sheet is a CONTACT SHEET: rows
//  are channels (what KIND of data), columns are zoom rungs each ~10x tighter than the last.
//  That is the rung rule (ATLAS.md) as a photograph.
//
//  HONESTY NOTE, printed on the sheet: this draws DECLARED coverage (the bbox the compositor
//  itself tests), not realized paint weight. Several sources feather inside their box (the
//  CUDEM insets 4%, the ortho 25 m, synth.bed ~700 m) and two are honest liars by construction
//  -- the equilibrium tide and EOT20 declare global boxes but EOT20 returns nodata over land.
//  Sampling the real weight per pixel would be truer, but ColorSource::Sample on the Google
//  tree ENQUEUES NETWORK FETCHES, and a debug picture must never cost the user their API
//  politeness budget. Declared coverage is what the cache keys on; that is the thing worth
//  auditing.
// ================================================================================================

// A 5x7 bitmap font, columns LSB-first (bit 0 = top row). Labels make this a tool instead of
// an abstract picture -- a legend you have to decode elsewhere is a legend nobody reads.
struct GlyphRow { char c; uint8_t col[5]; };
const GlyphRow kFont5x7[] = {
    {' ', {0x00, 0x00, 0x00, 0x00, 0x00}}, {'A', {0x7E, 0x11, 0x11, 0x11, 0x7E}},
    {'B', {0x7F, 0x49, 0x49, 0x49, 0x36}}, {'C', {0x3E, 0x41, 0x41, 0x41, 0x22}},
    {'D', {0x7F, 0x41, 0x41, 0x41, 0x3E}}, {'E', {0x7F, 0x49, 0x49, 0x49, 0x41}},
    {'F', {0x7F, 0x09, 0x09, 0x09, 0x01}}, {'G', {0x3E, 0x41, 0x49, 0x49, 0x7A}},
    {'H', {0x7F, 0x08, 0x08, 0x08, 0x7F}}, {'I', {0x00, 0x41, 0x7F, 0x41, 0x00}},
    {'J', {0x20, 0x40, 0x41, 0x3F, 0x01}}, {'K', {0x7F, 0x08, 0x14, 0x22, 0x41}},
    {'L', {0x7F, 0x40, 0x40, 0x40, 0x40}}, {'M', {0x7F, 0x02, 0x0C, 0x02, 0x7F}},
    {'N', {0x7F, 0x04, 0x08, 0x10, 0x7F}}, {'O', {0x3E, 0x41, 0x41, 0x41, 0x3E}},
    {'P', {0x7F, 0x09, 0x09, 0x09, 0x06}}, {'Q', {0x3E, 0x41, 0x51, 0x21, 0x5E}},
    {'R', {0x7F, 0x09, 0x19, 0x29, 0x46}}, {'S', {0x46, 0x49, 0x49, 0x49, 0x31}},
    {'T', {0x01, 0x01, 0x7F, 0x01, 0x01}}, {'U', {0x3F, 0x40, 0x40, 0x40, 0x3F}},
    {'V', {0x1F, 0x20, 0x40, 0x20, 0x1F}}, {'W', {0x7F, 0x20, 0x18, 0x20, 0x7F}},
    {'X', {0x63, 0x14, 0x08, 0x14, 0x63}}, {'Y', {0x03, 0x04, 0x78, 0x04, 0x03}},
    {'Z', {0x61, 0x51, 0x49, 0x45, 0x43}}, {'0', {0x3E, 0x51, 0x49, 0x45, 0x3E}},
    {'1', {0x00, 0x42, 0x7F, 0x40, 0x00}}, {'2', {0x42, 0x61, 0x51, 0x49, 0x46}},
    {'3', {0x21, 0x41, 0x45, 0x4B, 0x31}}, {'4', {0x18, 0x14, 0x12, 0x7F, 0x10}},
    {'5', {0x27, 0x45, 0x45, 0x45, 0x39}}, {'6', {0x3C, 0x4A, 0x49, 0x49, 0x30}},
    {'7', {0x01, 0x71, 0x09, 0x05, 0x03}}, {'8', {0x36, 0x49, 0x49, 0x49, 0x36}},
    {'9', {0x06, 0x49, 0x49, 0x29, 0x1E}}, {'.', {0x00, 0x40, 0x60, 0x00, 0x00}},
    {',', {0x00, 0x80, 0x60, 0x00, 0x00}}, {'-', {0x08, 0x08, 0x08, 0x08, 0x08}},
    {'/', {0x20, 0x10, 0x08, 0x04, 0x02}}, {':', {0x00, 0x36, 0x36, 0x00, 0x00}},
    {'(', {0x00, 0x1C, 0x22, 0x41, 0x00}}, {')', {0x00, 0x41, 0x22, 0x1C, 0x00}},
    {'*', {0x14, 0x08, 0x3E, 0x08, 0x14}}, {'+', {0x08, 0x08, 0x3E, 0x08, 0x08}},
    {'=', {0x14, 0x14, 0x14, 0x14, 0x14}}, {'<', {0x08, 0x14, 0x22, 0x41, 0x00}},
    {'>', {0x00, 0x41, 0x22, 0x14, 0x08}}, {'%', {0x23, 0x13, 0x08, 0x64, 0x62}},
};

// One source group as the legend shows it: sources with identical structure/resolution/box
// collapse (the 18 tide constituents share three rungs -- 54 legend lines is not a legend).
struct FidGroup {
    std::string name;
    std::string structure;
    double cm = 0;
    double lon0 = 0, lat0 = 0, lon1 = 0, lat1 = 0;
    uint8_t r = 0, g = 0, b = 0;
};

// Fidelity 0..1 from resolution: log10 between the coarsest thing we carry (500 km, the
// analytic equilibrium tide) and the finest (15 cm, the MassGIS leaf-off ortho).
double FidelityOf(double cmPerPixel) {
    if (cmPerPixel <= 0) return 0.0;
    const double lo = std::log10(15.0), hi = std::log10(5.0e7);
    const double t = (hi - std::log10(cmPerPixel)) / (hi - lo);
    return std::clamp(t, 0.0, 1.0);
}

void HsvToRgb(double h, double s, double v, uint8_t& r, uint8_t& g, uint8_t& b) {
    h = h - std::floor(h);
    const double i = std::floor(h * 6.0), f = h * 6.0 - i;
    const double p = v * (1 - s), q = v * (1 - f * s), t = v * (1 - (1 - f) * s);
    double rr = v, gg = t, bb = p;
    switch (static_cast<int>(i) % 6) {
        case 0: rr = v; gg = t; bb = p; break;
        case 1: rr = q; gg = v; bb = p; break;
        case 2: rr = p; gg = v; bb = t; break;
        case 3: rr = p; gg = q; bb = v; break;
        case 4: rr = t; gg = p; bb = v; break;
        default: rr = v; gg = p; bb = q; break;
    }
    r = static_cast<uint8_t>(std::clamp(rr, 0.0, 1.0) * 255.0);
    g = static_cast<uint8_t>(std::clamp(gg, 0.0, 1.0) * 255.0);
    b = static_cast<uint8_t>(std::clamp(bb, 0.0, 1.0) * 255.0);
}

void RenderFidelityMap(Compositor& comp, VectorPack& vec, const std::wstring& outPath) {
    // ---- 1. the registry, grouped. Sources that declare the same structure, resolution and
    // box ARE the same rung wearing 18 constituent names; the legend says so once.
    struct ChanRow {
        std::string title;
        std::vector<FidGroup> groups;
    };
    std::vector<ChanRow> rows;
    std::vector<FidGroup*> allGroups;

    auto collect = [&](const char* title, const std::vector<std::string>& wantChannels) {
        ChanRow row;
        row.title = title;
        for (int ci = 0; ci < comp.ChannelCount(); ++ci) {
            const Compositor::Channel& ch = comp.ChannelAt(ci);
            bool want = false;
            for (const std::string& w : wantChannels) {
                if (ch.name.rfind(w, 0) == 0) want = true;
            }
            if (!want) continue;
            auto add = [&](const SourceInfo& si) {
                for (FidGroup& gp : row.groups) {
                    if (gp.structure == si.structure && gp.cm == si.cmPerPixel &&
                        gp.lon0 == si.lon0 && gp.lat0 == si.lat0 && gp.lon1 == si.lon1 &&
                        gp.lat1 == si.lat1) {
                        // Same rung, another constituent: fold the name to a common stem.
                        size_t k = 0;
                        while (k < gp.name.size() && k < si.name.size() &&
                               gp.name[k] == si.name[k]) {
                            ++k;
                        }
                        while (k > 0 && gp.name[k - 1] != '.') --k;
                        if (k > 0) gp.name = gp.name.substr(0, k) + "*";
                        return;
                    }
                }
                row.groups.push_back({si.name, si.structure, si.cmPerPixel, si.lon0, si.lat0,
                                      si.lon1, si.lat1, 0, 0, 0});
            };
            for (const ColorSource* s : ch.color) add(s->Info());
            for (const HeightSource* s : ch.height) add(s->Info());
            for (const FieldSource* s : ch.field) add(s->Info());
        }
        // Finest first: the winner test below is a min, and the legend reads top-down.
        std::sort(row.groups.begin(), row.groups.end(),
                  [](const FidGroup& a, const FidGroup& b) { return a.cm < b.cm; });
        if (!row.groups.empty()) rows.push_back(std::move(row));
    };
    collect("EARTH.HEIGHT  (THE BED)", {"earth.height"});
    collect("EARTH.COLOR  (THE SKIN)", {"earth.color"});
    collect("WATER.TIDE  (THE ROTORS)", {"water.tide"});
    if (rows.empty()) {
        Log("[fidelity] no channels registered -- nothing to draw");
        return;
    }

    // Hues walk by the golden ratio so neighbours in the legend never collide.
    {
        double h = 0.04;
        for (ChanRow& r : rows) {
            for (FidGroup& gp : r.groups) {
                HsvToRgb(h, 0.78, 0.95, gp.r, gp.g, gp.b);
                h += 0.381966;
                allGroups.push_back(&gp);
            }
        }
    }

    // ---- 2. the sheet: rows = channels, columns = zoom rungs (each ~10x tighter).
    struct Panel {
        const char* label;
        double latC, lonC, latSpan;   // degrees
    };
    const Panel kPanels[] = {
        {"GLOBAL  180 DEG", 0.0, -30.0, 180.0},
        {"GULF OF MAINE  8 DEG", 42.9, -69.6, 8.0},
        {"THE ESTUARY  0.8 DEG", 42.78, -70.83, 0.8},
        {"THE INLET  0.08 DEG", 42.8165, -70.8125, 0.08},
    };
    const int kNP = static_cast<int>(sizeof(kPanels) / sizeof(kPanels[0]));
    const int PW = 430, PH = 300, PAD = 14, LEFT = 200, TOP = 46;
    const int nRows = static_cast<int>(rows.size());
    int legendLines = 0;
    for (const ChanRow& r : rows) legendLines += static_cast<int>(r.groups.size()) + 1;
    const int LEGY = TOP + nRows * (PH + PAD) + 16;
    const int W = LEFT + kNP * (PW + PAD) + PAD;
    const int H = LEGY + legendLines * 13 + 30;

    std::vector<uint8_t> img(static_cast<size_t>(W) * H * 4, 255);
    auto px = [&](int x, int y) -> uint8_t* {
        return &img[(static_cast<size_t>(y) * W + x) * 4];
    };
    for (int y = 0; y < H; ++y) {
        for (int x = 0; x < W; ++x) {
            uint8_t* p = px(x, y);
            p[0] = 18; p[1] = 20; p[2] = 24;
        }
    }
    auto text = [&](int x, int y, const char* s, uint8_t r, uint8_t g, uint8_t b) {
        for (const char* c = s; *c; ++c) {
            const char up = static_cast<char>(*c >= 'a' && *c <= 'z' ? *c - 32 : *c);
            const GlyphRow* gl = nullptr;
            for (const GlyphRow& e : kFont5x7) {
                if (e.c == up) { gl = &e; break; }
            }
            if (gl) {
                for (int cx = 0; cx < 5; ++cx) {
                    for (int cy = 0; cy < 7; ++cy) {
                        if (!(gl->col[cx] & (1 << cy))) continue;
                        const int ax = x + cx, ay = y + cy;
                        if (ax < 0 || ay < 0 || ax >= W || ay >= H) continue;
                        uint8_t* p = px(ax, ay);
                        p[0] = r; p[1] = g; p[2] = b;
                    }
                }
            }
            x += 6;
        }
    };

    text(PAD, 14, "GAGAME DATA FIDELITY -- HUE = FINEST SOURCE, BRIGHTNESS = LOG RESOLUTION",
         235, 238, 245);
    text(PAD, 26, "DECLARED COVERAGE AND CM/PX FROM THE SCHEMA REGISTRY (SOURCEINFO)",
         120, 128, 140);

    const std::vector<float>* coastSegs = nullptr;
    std::vector<float> coastG, coastN;
    if (const VectorPack::Layer* L = vec.Find("coast_global")) coastG = vec.Segments(*L, 3000.0f);
    if (const VectorPack::Layer* L = vec.Find("coast_ne")) coastN = vec.Segments(*L, 20.0f);
    (void)coastSegs;

    for (int ri = 0; ri < nRows; ++ri) {
        const ChanRow& row = rows[ri];
        const int oy = TOP + ri * (PH + PAD);
        text(PAD, oy + PH / 2 - 4, row.title.c_str(), 225, 228, 236);
        for (int pi = 0; pi < kNP; ++pi) {
            const Panel& pa = kPanels[pi];
            const int ox = LEFT + pi * (PW + PAD);
            // Equal-area-ish framing: lon span widened by 1/cos(lat) so shapes stay honest.
            const double cosL = (pa.latSpan > 90.0)
                                    ? 1.0
                                    : std::max(0.2, std::cos(pa.latC * 3.14159265 / 180.0));
            const double latSpan = pa.latSpan;
            const double lonSpan = latSpan * (double(PW) / double(PH)) / cosL;
            const double la0 = pa.latC - latSpan * 0.5, la1 = pa.latC + latSpan * 0.5;
            const double lo0 = pa.lonC - lonSpan * 0.5, lo1 = pa.lonC + lonSpan * 0.5;
            // The panel's own ground resolution -- what a pixel of THIS panel is worth.
            const double panelM = latSpan * 111320.0 / PH;

            if (ri == 0) {
                text(ox, oy - 12, pa.label, 200, 206, 216);
            }
            for (int y = 0; y < PH; ++y) {
                const double lat = la1 - (y + 0.5) / PH * (la1 - la0);
                for (int x = 0; x < PW; ++x) {
                    const double lon = lo0 + (x + 0.5) / PW * (lo1 - lo0);
                    uint8_t* p = px(ox + x, oy + y);
                    const FidGroup* best = nullptr;
                    int depth = 0;
                    for (const FidGroup& gp : row.groups) {
                        if (lon < gp.lon0 || lon > gp.lon1 || lat < gp.lat0 || lat > gp.lat1) {
                            continue;
                        }
                        ++depth;
                        if (!best || gp.cm < best->cm) best = &gp;
                    }
                    if (!best) {
                        p[0] = 30; p[1] = 32; p[2] = 38;   // genuinely no data here
                        continue;
                    }
                    const double f = FidelityOf(best->cm);
                    // Brightness carries fidelity; a thin depth term says "and others agree".
                    const double dep = std::min(depth - 1, 3) * 0.045;
                    const double s = 0.22 + 0.70 * f, v = 0.26 + 0.66 * f + dep;
                    // Re-derive the group's hue from its swatch so one law makes both.
                    const double mx = std::max({best->r, best->g, best->b}) / 255.0;
                    const double mn = std::min({best->r, best->g, best->b}) / 255.0;
                    double hue = 0.0;
                    if (mx > mn) {
                        const double rr = best->r / 255.0, gg = best->g / 255.0,
                                     bb = best->b / 255.0, d = mx - mn;
                        if (mx == rr) hue = (gg - bb) / d / 6.0;
                        else if (mx == gg) hue = (2.0 + (bb - rr) / d) / 6.0;
                        else hue = (4.0 + (rr - gg) / d) / 6.0;
                    }
                    HsvToRgb(hue, s, std::min(v, 1.0), p[0], p[1], p[2]);
                }
            }
            // Coast ink + graticule, drawn over the fill so geography is readable.
            auto drawSegs = [&](const std::vector<float>& segs, uint8_t cr, uint8_t cg,
                                uint8_t cb) {
                for (size_t i = 0; i + 3 < segs.size(); i += 4) {
                    const double aLon = segs[i], aLat = segs[i + 1];
                    const double bLon = segs[i + 2], bLat = segs[i + 3];
                    if (std::abs(bLon - aLon) > 180.0) continue;   // the wrap chord
                    if ((aLat < la0 && bLat < la0) || (aLat > la1 && bLat > la1)) continue;
                    if ((aLon < lo0 && bLon < lo0) || (aLon > lo1 && bLon > lo1)) continue;
                    const double ax = (aLon - lo0) / (lo1 - lo0) * PW;
                    const double ay = (la1 - aLat) / (la1 - la0) * PH;
                    const double bx = (bLon - lo0) / (lo1 - lo0) * PW;
                    const double by = (la1 - bLat) / (la1 - la0) * PH;
                    const int steps = static_cast<int>(std::hypot(bx - ax, by - ay)) + 1;
                    for (int st = 0; st <= steps; ++st) {
                        const double t = static_cast<double>(st) / steps;
                        const int cx = static_cast<int>(ax + (bx - ax) * t);
                        const int cy = static_cast<int>(ay + (by - ay) * t);
                        if (cx < 0 || cy < 0 || cx >= PW || cy >= PH) continue;
                        uint8_t* q = px(ox + cx, oy + cy);
                        q[0] = cr; q[1] = cg; q[2] = cb;
                    }
                }
            };
            drawSegs(latSpan > 20.0 ? coastG : coastN, 236, 240, 248);
            // Panel frame + the pixel's worth, so a reader can size what they are seeing.
            for (int x = -1; x <= PW; ++x) {
                for (int e = 0; e < 2; ++e) {
                    const int yy = oy + (e ? PH : -1);
                    if (ox + x < 0 || ox + x >= W || yy < 0 || yy >= H) continue;
                    uint8_t* q = px(ox + x, yy);
                    q[0] = 70; q[1] = 76; q[2] = 88;
                }
            }
            for (int y = -1; y <= PH; ++y) {
                for (int e = 0; e < 2; ++e) {
                    const int xx = ox + (e ? PW : -1);
                    if (xx < 0 || xx >= W || oy + y < 0 || oy + y >= H) continue;
                    uint8_t* q = px(xx, oy + y);
                    q[0] = 70; q[1] = 76; q[2] = 88;
                }
            }
            char note[64];
            if (panelM >= 1000.0) snprintf(note, sizeof(note), "%.0f KM/PX", panelM / 1000.0);
            else snprintf(note, sizeof(note), "%.0f M/PX", panelM);
            text(ox + 2, oy + PH + 4, note, 150, 156, 168);   // under the frame, never on it
        }
    }

    // ---- 3. the legend: every rung, its resolution, its declared box.
    int ly = LEGY;
    text(PAD, ly, "THE RUNGS  (FINEST FIRST)", 235, 238, 245);
    ly += 14;
    for (const ChanRow& row : rows) {
        text(PAD, ly, row.title.c_str(), 170, 178, 192);
        ly += 13;
        for (const FidGroup& gp : row.groups) {
            for (int y = 0; y < 8; ++y) {
                for (int x = 0; x < 14; ++x) {
                    uint8_t* q = px(PAD + 8 + x, ly + y);
                    const double f = FidelityOf(gp.cm);
                    q[0] = static_cast<uint8_t>(gp.r * (0.32 + 0.68 * f));
                    q[1] = static_cast<uint8_t>(gp.g * (0.32 + 0.68 * f));
                    q[2] = static_cast<uint8_t>(gp.b * (0.32 + 0.68 * f));
                }
            }
            char line[220];
            const double m = gp.cm / 100.0;
            char res[32];
            if (m >= 1000.0) snprintf(res, sizeof(res), "%.0f KM", m / 1000.0);
            else if (m >= 1.0) snprintf(res, sizeof(res), "%.0f M", m);
            else snprintf(res, sizeof(res), "%.0f CM", gp.cm);
            snprintf(line, sizeof(line), "%-26s %8s   (%.1f..%.1f) X (%.1f..%.1f)",
                     gp.name.c_str(), res, gp.lon0, gp.lon1, gp.lat0, gp.lat1);
            text(PAD + 30, ly, line, 205, 210, 220);
            ly += 13;
        }
    }

    if (SavePng(outPath, img.data(), W, H, W * 4, img.size())) {
        Log("[fidelity] %S : %dx%d, %zu rungs over %d channels", outPath.c_str(), W, H,
            allGroups.size(), nRows);
    }
}

// M6v: --water-map -- THE REPROJECTION PROOF. The same sources that feed the engine's
// realizations (the water.tide phasor stack, the survey vectors, the station registry)
// rendered through an ARBITRARY print projection: a custom Lambert conformal sheet built
// for this page, not any realization the renderer uses. Nothing in the physics or the data
// model changes -- a projection is just another realization, which is the whole foundation
// the user asked for ("print maps onto paper" without breaking anything). M2 amplitude as
// the field (co-amplitude chart), GSHHG parity-filled land, coast + structures + stations.
// bathyChannel >= 0 switches the sheet to HYPSOMETRIC BATHYMETRY: every pixel is
// SampleHeightStack -- the same painted stack the renderer's tiles and the solver's bed
// come from, so the chart IS the channel (all three CUDEM insets, the NE-15s base, the
// hand-edit structures, one surface).
void RenderWaterMap(Compositor& comp, const WaterAtlas& wa, VectorPack& vec,
                    const GlobeModel& gm, const std::wstring& outPath,
                    int bathyChannel = -1) {
    const int H = 1500;
    int W = 1800;   // trimmed to the sheet's true aspect below
    const double kD2R = 3.14159265358979 / 180.0;
    const double lon0 = -71.15, lon1 = -70.30, lat0 = 42.18, lat1 = 43.02;

    // The print sheet: a Lambert conformal cone laid over the focus box (standard parallels
    // inside it) -- chosen for the PAGE, proving realizations are free to pick projections.
    LambertConformalConic lcc{42.35 * kD2R, 42.90 * kD2R, 42.0 * kD2R, -70.725 * kD2R,
                              0.0, 0.0};
    lcc.Derive();
    double xMin = 1e18, xMax = -1e18, yMin = 1e18, yMax = -1e18;
    for (int k = 0; k <= 100; ++k) {
        const double t = k / 100.0;
        double x, y;
        lcc.Forward(lat0 * kD2R, (lon0 + t * (lon1 - lon0)) * kD2R, x, y);
        xMin = (std::min)(xMin, x); xMax = (std::max)(xMax, x);
        yMin = (std::min)(yMin, y); yMax = (std::max)(yMax, y);
        lcc.Forward(lat1 * kD2R, (lon0 + t * (lon1 - lon0)) * kD2R, x, y);
        xMin = (std::min)(xMin, x); xMax = (std::max)(xMax, x);
        yMin = (std::min)(yMin, y); yMax = (std::max)(yMax, y);
        lcc.Forward((lat0 + t * (lat1 - lat0)) * kD2R, lon0 * kD2R, x, y);
        xMin = (std::min)(xMin, x); xMax = (std::max)(xMax, x);
        yMin = (std::min)(yMin, y); yMax = (std::max)(yMax, y);
        lcc.Forward((lat0 + t * (lat1 - lat0)) * kD2R, lon1 * kD2R, x, y);
        xMin = (std::min)(xMin, x); xMax = (std::max)(xMax, x);
        yMin = (std::min)(yMin, y); yMax = (std::max)(yMax, y);
    }
    const double scale = (H - 40) / (yMax - yMin);
    W = static_cast<int>((xMax - xMin) * scale) + 40;
    auto toPx = [&](double latD, double lonD, double& px, double& py) {
        double x, y;
        lcc.Forward(latD * kD2R, lonD * kD2R, x, y);
        px = 20.0 + (x - xMin) * scale;
        py = H - 20.0 - (y - yMin) * scale;
    };
    // Analytic LCC inverse (Snyder): rho/theta -> t -> three fixed-point steps for lat.
    auto fromPx = [&](double px, double py, double& latD, double& lonD) {
        const double x = (px - 20.0) / scale + xMin;
        const double y = (H - 20.0 - py) / scale + yMin;
        const double rx = x, ry = lcc.rho0 - y;
        const double rho = std::sqrt(rx * rx + ry * ry) * (lcc.n < 0 ? -1.0 : 1.0);
        const double th = std::atan2(rx, ry);
        lonD = (th / lcc.n + lcc.lon0Rad) / kD2R;
        const double tt = std::pow(rho / lcc.aF, 1.0 / lcc.n);
        constexpr double f = 1.0 / 298.257222101;
        const double e = std::sqrt(f * (2.0 - f));
        double phi = 3.14159265358979 / 2.0 - 2.0 * std::atan(tt);
        for (int i = 0; i < 3; ++i) {
            const double es = e * std::sin(phi);
            phi = 3.14159265358979 / 2.0 -
                  2.0 * std::atan(tt * std::pow((1.0 - es) / (1.0 + es), e / 2.0));
        }
        latD = phi / kD2R;
    };

    // Land: the NE 15" relief grid (461 m -- honest chart-scale coastline). The vpack coast
    // is CLIPPED polylines, so scanline parity is unsound (tried; false closure chords
    // flipped the whole Atlantic to land). Heights are the region's land authority anyway.
    auto isLand = [&](double latD, double lonD) {
        if (gm.NeNx() <= 0) return false;
        const double fx = (lonD - gm.NeLon0()) / gm.NeDLon();
        const double fy = (latD - gm.NeLat1()) / gm.NeDLat();
        const int c = static_cast<int>(fx), r = static_cast<int>(fy);
        if (r < 0 || c < 0 || r >= gm.NeNy() || c >= gm.NeNx()) return false;
        return gm.NeElev()[static_cast<size_t>(r) * gm.NeNx() + c] > 0;
    };

    // The page: paper white; ocean tinted by M2 AMPLITUDE from the composed stack (the same
    // SampleFieldStack every realization paints from); graticule at 0.25 degrees.
    std::vector<uint8_t> img(static_cast<size_t>(W) * H * 4, 255);
    const int wmChannel = wa.ChannelId(0);
    for (int py = 0; py < H; ++py) {
        for (int px = 0; px < W; ++px) {
            double latD, lonD;
            fromPx(px, py, latD, lonD);
            uint8_t* p = &img[(static_cast<size_t>(py) * W + px) * 4];
            if (latD < lat0 || latD > lat1 || lonD < lon0 || lonD > lon1) continue;
            if (bathyChannel >= 0) {
                const float h =
                    comp.SampleHeightStack(bathyChannel, latD * kD2R, lonD * kD2R, 60.0);
                if (h > 0.0f) {                       // hypsometric land from the SAME stack
                    const double t = std::clamp(h / 60.0, 0.0, 1.0);
                    p[0] = static_cast<uint8_t>(196 + 40 * t);
                    p[1] = static_cast<uint8_t>(206 - 60 * t);
                    p[2] = static_cast<uint8_t>(178 - 80 * t);
                } else {                              // depth ramp: light flats -> dark deep
                    const double t = std::clamp(-h / 60.0, 0.0, 1.0);
                    const double s = std::sqrt(t);
                    p[0] = static_cast<uint8_t>(190 - 165 * s);
                    p[1] = static_cast<uint8_t>(222 - 150 * s);
                    p[2] = static_cast<uint8_t>(236 - 110 * s);
                }
            } else if (isLand(latD, lonD)) {
                p[0] = 238; p[1] = 234; p[2] = 222;                      // chart-paper land
            } else {
                float ph[2];
                comp.SampleFieldStack(wmChannel, latD * kD2R, lonD * kD2R, 200.0, ph);
                const double amp = std::hypot(ph[0], ph[1]);
                const double t = std::clamp((amp - 0.9) / 0.6, 0.0, 1.0);   // 0.9..1.5 m
                p[0] = static_cast<uint8_t>(214 - 120 * t);
                p[1] = static_cast<uint8_t>(228 - 90 * t);
                p[2] = static_cast<uint8_t>(240 - 40 * t);
            }
            const double gl = 0.25;
            const double dLat = std::abs(latD / gl - std::round(latD / gl)) * gl;
            const double dLon = std::abs(lonD / gl - std::round(lonD / gl)) * gl;
            if (dLat < 0.0008 || dLon < 0.0008) {
                p[0] = static_cast<uint8_t>(p[0] * 0.82);
                p[1] = static_cast<uint8_t>(p[1] * 0.82);
                p[2] = static_cast<uint8_t>(p[2] * 0.82);
            }
        }
    }

    auto drawSeg = [&](double aLat, double aLon, double bLat, double bLon, uint8_t r,
                       uint8_t g, uint8_t b, int thick) {
        double ax, ay, bx, by;
        toPx(aLat, aLon, ax, ay);
        toPx(bLat, bLon, bx, by);
        const int steps = static_cast<int>(std::hypot(bx - ax, by - ay)) + 1;
        for (int s = 0; s <= steps; ++s) {
            const double t = static_cast<double>(s) / steps;
            const int cx = static_cast<int>(ax + (bx - ax) * t);
            const int cy = static_cast<int>(ay + (by - ay) * t);
            for (int dy = -thick; dy <= thick; ++dy) {
                for (int dx = -thick; dx <= thick; ++dx) {
                    if (cx + dx < 0 || cy + dy < 0 || cx + dx >= W || cy + dy >= H) continue;
                    uint8_t* p = &img[(static_cast<size_t>(cy + dy) * W + (cx + dx)) * 4];
                    p[0] = r; p[1] = g; p[2] = b;
                }
            }
        }
    };
    // Survey vectors through the SAME projection: coast (ink), structures (amber).
    auto drawLayer = [&](const char* name, float tol, uint8_t r, uint8_t g, uint8_t b,
                         int thick) {
        const VectorPack::Layer* L = vec.Find(name);
        if (!L) return;
        const std::vector<float> segs = vec.Segments(*L, tol);
        for (size_t i = 0; i + 3 < segs.size(); i += 4) {
            const double aLon = segs[i], aLat = segs[i + 1];
            const double bLon = segs[i + 2], bLat = segs[i + 3];
            if (aLat < lat0 || aLat > lat1 || aLon < lon0 || aLon > lon1) continue;
            drawSeg(aLat, aLon, bLat, bLon, r, g, b, thick);
        }
    };
    drawLayer("coast_ne", 60.0f, 60, 52, 44, 0);
    drawLayer("structures", 10.0f, 190, 120, 20, 1);

    // The station survey (registry.json): tide = red, current = teal, buoy = green.
    {
        std::ifstream rf("data/water/registry.json", std::ios::binary);
        if (rf) {
            std::string txt((std::istreambuf_iterator<char>(rf)),
                            std::istreambuf_iterator<char>());
            std::string err;
            const JsonValue root = JsonParser::Parse(txt, &err);
            auto mark = [&](const JsonValue* arr, uint8_t r, uint8_t g, uint8_t b, int sz) {
                if (!arr) return;
                for (const JsonValue& s : arr->arr) {
                    const double lat = s.Num("lat", 0), lon = s.Num("lon", 0);
                    if (lat < lat0 || lat > lat1 || lon < lon0 || lon > lon1) continue;
                    double px, py;
                    toPx(lat, lon, px, py);
                    for (int dy = -sz; dy <= sz; ++dy) {
                        for (int dx = -sz; dx <= sz; ++dx) {
                            if (dx * dx + dy * dy > sz * sz) continue;
                            const int cx = static_cast<int>(px) + dx;
                            const int cy = static_cast<int>(py) + dy;
                            if (cx < 0 || cy < 0 || cx >= W || cy >= H) continue;
                            uint8_t* p = &img[(static_cast<size_t>(cy) * W + cx) * 4];
                            p[0] = r; p[1] = g; p[2] = b;
                        }
                    }
                }
            };
            mark(root.Get("current_stations"), 20, 150, 160, 2);
            mark(root.Get("tide_stations"), 200, 30, 30, 4);
            mark(root.Get("buoys"), 30, 160, 40, 5);
        }
    }
    SavePng(outPath, img.data(), W, H, W * 4, img.size());
    Log("[water] chart printed: %S (Lambert conformal sheet, M2 co-amplitude, survey "
        "vectors + %s)", outPath.c_str(), "stations");
}

// M5c debug: the solved surface-current field as a picture. u east = red, u west = blue,
// v north = green tint, brightness = speed; land/invalid = dark grey. The fastest way to SEE
// whether the throat jet, the eddies, and the boundary plumbing are doing physics or nonsense.
void DumpSweUv(Gpu& gpu, SweSolver& swe, const BathyModel& bathy, const std::wstring& path) {
    const uint32_t nx = swe.Nx(), ny = swe.Ny();
    std::vector<float> pts(static_cast<size_t>(nx) * ny * 2);
    // One giant batch is wasteful; sample the texture directly through ReadProbes' machinery
    // instead: probe every texel via a single readback by asking for row-major world points.
    for (uint32_t y = 0; y < ny; ++y) {
        for (uint32_t x = 0; x < nx; ++x) {
            const size_t i = (static_cast<size_t>(y) * nx + x) * 2;
            pts[i + 0] = bathy.WorldX0() + (x + 0.5f) / nx * bathy.WorldSizeX();
            pts[i + 1] = bathy.WorldZ0() + bathy.WorldSizeZ() * (1.0f - (y + 0.5f) / ny);
        }
    }
    std::vector<SweSolver::Probe> pr(static_cast<size_t>(nx) * ny);
    swe.ReadProbes(gpu, pts.data(), static_cast<int>(nx * ny), pr.data());

    std::vector<uint8_t> rgba(static_cast<size_t>(nx) * ny * 4);
    for (size_t i = 0; i < pr.size(); ++i) {
        uint8_t* px = &rgba[i * 4];
        if (!pr[i].valid) {
            px[0] = px[1] = px[2] = 46;
        } else {
            const float u = pr[i].u, v = pr[i].v;
            const float s = std::sqrt(u * u + v * v);
            const float b = std::min(1.0f, s / 1.5f);   // full brightness at 1.5 m/s
            const float uf = std::clamp(u / std::max(s, 1e-4f), -1.0f, 1.0f);
            const float vf = std::clamp(v / std::max(s, 1e-4f), -1.0f, 1.0f);
            px[0] = static_cast<uint8_t>(40 + 215 * b * std::max(0.0f, uf));
            px[2] = static_cast<uint8_t>(40 + 215 * b * std::max(0.0f, -uf));
            px[1] = static_cast<uint8_t>(40 + 130 * b * std::abs(vf));
        }
        px[3] = 255;
    }
    SavePng(path, rgba.data(), nx, ny, nx * 4, rgba.size());
    Log("[swe] uv field dumped to %S", path.c_str());

    // Companion picture: the deviation field itself. Red = above the tide plane, blue = below
    // (full at 25 cm); land grey. At max ebb the west strip must glow red and slope away east.
    for (size_t i = 0; i < pr.size(); ++i) {
        uint8_t* px = &rgba[i * 4];
        const float e = pr[i].dEta;
        const float b = std::min(1.0f, std::abs(e) / 0.25f);
        px[0] = static_cast<uint8_t>(40 + (e > 0 ? 215 * b : 0));
        px[1] = 40;
        px[2] = static_cast<uint8_t>(40 + (e < 0 ? 215 * b : 0));
        px[3] = 255;
    }
    SavePng(path + L".eta.png", rgba.data(), nx, ny, nx * 4, rgba.size());

    // Third picture: raw flux magnitude (log scale). Distinguishes "flux kernel never ran here"
    // (exact zero, black) from "ran but weak" (dim) at a glance.
    uint32_t fw = 0, fh = 0, fpitch = 0;
    const std::vector<uint8_t> flux = swe.ReadFluxRaw(gpu, &fw, &fh, &fpitch);
    for (uint32_t y = 0; y < ny; ++y) {
        for (uint32_t x = 0; x < nx; ++x) {
            const float* f = reinterpret_cast<const float*>(&flux[y * fpitch + x * 8]);
            const float mag = std::abs(f[0]) + std::abs(f[1]);
            uint8_t* px = &rgba[(static_cast<size_t>(y) * nx + x) * 4];
            const float v = (mag > 0) ? std::min(1.0f, 0.18f * std::log2(1.0f + mag)) : 0.0f;
            px[0] = static_cast<uint8_t>(20 + 235 * v);
            px[1] = static_cast<uint8_t>(20 + 90 * v);
            px[2] = (mag == 0.0f) ? 20 : 60;
            px[3] = 255;
        }
    }
    SavePng(path + L".flux.png", rgba.data(), nx, ny, nx * 4, rgba.size());

    // M6r continuity audit: NET eastward transport through full N-S sections (sum of raw east
    // face fluxes down a column, m^3/s -- land and NULL faces are exact zeros). Continuity says
    // consecutive sections differ only by the storage filling between them; a jump reveals a
    // leak, a flat profile with a weak gap says the demand never concentrated.
    for (const float wx : {-12000.0f, -5000.0f, -2500.0f, 0.0f, 350.0f, 900.0f, 1600.0f}) {
        const int ix = static_cast<int>((wx - bathy.WorldX0()) / bathy.WorldSizeX() * nx);
        if (ix < 0 || ix >= static_cast<int>(nx)) continue;
        double q = 0.0;
        for (uint32_t y = 0; y < ny; ++y) {
            q += reinterpret_cast<const float*>(&flux[y * fpitch + ix * 8])[0];
        }
        Log("[swe] section x=%+6.0f m: net east transport %+8.1f m^3/s", wx, q);
    }
}

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

int main(int argc, char** argv) {
    ga::InstallCrashTrace();   // M7v: symbolized stacks on any crash, headless
    // Before any pool exists: this is the thread that owns the frame, the device, the queue and
    // the command list, and GA_MAIN_THREAD_ONLY() is measured against it.
    ga::threadaudit::SetMainThread();
    try {
        const Options opt = ParseArgs(argc, argv);
        if (opt.threadAudit) ga::threadaudit::Enable();
        // The process pool, before anything can submit to it. Everything that used to spawn its
        // own threads is a client of this (core/ThreadManager.h).
        ga::Threads().Init();
        ga::Threads().SetInline(opt.jobsInline);
        {
            // The first line of every log: which binary, which flags. A baseline still or a
            // bench log is otherwise unmatchable to the recipe that made it (out/baseline/*.log
            // named neither; the memory note carried the command instead).
            std::string args;
            for (int i = 1; i < argc; ++i) {
                if (i > 1) args += ' ';
                args += argv[i];
            }
            Log("[boot] gagame rev %s | argv: %s", BuildGitRev(), args.c_str());
        }

        // M9ah: pack the composed cache into per-realization archives and exit. No device, no
        // scene -- this is a disk-to-disk job. The loose tiles are kept: the archive is derived,
        // and the compositor keeps writing loose files as it paints, so anything painted after a
        // pack must still be findable the old way.
        if (opt.packTiles) {
            const char* jobs[][2] = {
                {"earth.color", "cube16k"},
                {"earth.color", "window_z14_1263360_1538048"},
                {"earth.color", "window_z17_10168820_12344774"},
                {"earth.color", "window_z19_40699567_49405858"},
                {"earth.height", "cube16k"},
                {"earth.height", "window_z14_1263360_1538048"},
                {"mars.height", "cube16k"},
            };
            uint32_t total = 0;
            for (const auto& j : jobs) total += TileArchive::Pack(j[0], j[1]);
            Log("[tilearch] %u tiles packed across %zu realizations", total,
                sizeof(jobs) / sizeof(jobs[0]));
            return 0;
        }

        // ---- M0 + M4: the self-test path needs a device and the shader compiler, nothing else.
        // M9h: --load-field -- the plugin path, end to end and standalone. Register a loader
        // for a file type, open a file, and the sparse bank falls out of the ingest rule:
        // nodata is absence, absence is a NULL tile, and the tiles that were never allocated
        // are the measurement. No renderer, no scene -- just the road.
        if (!opt.loadField.empty()) {
            Gpu gpu;
            gpu.Init(nullptr, 64, 64, opt.debugLayer);
            LoaderRegistry plugin;
            plugin.Register("f32", GeoGridLoader::Open);
            plugin.Register("json", GeoGridLoader::Open);
            std::unique_ptr<FieldLoader> ld = plugin.Open(opt.loadField);
            if (!ld) {
                Log("[loader] cannot open %s (known types: f32, json)", opt.loadField.c_str());
                return 1;
            }
            const GeoRef& gr = ld->Ref();
            GradeBankDesc d;
            d.name = std::string("loaded.") + ld->Name();
            d.width = gr.width;
            d.height = gr.height;
            d.fmt = DXGI_FORMAT_R32_FLOAT;
            d.gradeSig = ld->GradeSig();
            d.metersPerTexel = gr.MetersPerTexelX();
            d.vNorth = gr.VNorth();
            d.units = gr.valueUnit;
            GradeBank bank;
            bank.Init(gpu, d, policy::None());
            const uint32_t tX = bank.TilesX(), tY = bank.TilesY();
            auto* grid = static_cast<GeoGridLoader*>(ld.get());
            const std::vector<uint8_t> mask =
                grid->CoverageMask(bank.TileW(), bank.TileH(), tX, tY);
            bank.SetPolicy(policy::Mask(mask, tX));
            bank.Update(gpu);
            bank.UploadDenseTiles(gpu, reinterpret_cast<const uint8_t*>(grid->Samples().data()),
                                  gr.width, 4u, bank.ResidentList());
            const uint32_t all = tX * tY, res = bank.ResidentCount();
            char sig[16];
            snprintf(sig, sizeof(sig), "%s%s%s", (d.gradeSig & kG0) ? "g0" : "",
                     (d.gradeSig & kG1) ? "g1" : "", (d.gradeSig & kG2) ? "g2" : "");
            Log("[loader] %s -> bank '%s' [%s]: %u/%u tiles resident (%.1f%%), "
                "%.1f of %.1f MB, %u tiles were pure absence",
                opt.loadField.c_str(), d.name.c_str(), sig, res, all,
                all ? 100.0 * res / all : 0.0, bank.ResidentBytes() / 1048576.0,
                bank.VirtualBytes() / 1048576.0, all - res);
            Log("[loader] georeference: %s, %.3g deg/texel (%.2f m), value %s",
                gr.Describe().c_str(), gr.scaleX, gr.MetersPerTexelX(), gr.valueUnit);
            gpu.WaitIdle();
            return 0;
        }

        if (opt.selftest) {
            Gpu gpu;
            gpu.Init(nullptr, 64, 64, opt.debugLayer);
            ShaderCompiler sc;
            sc.Init();
            bool ok = RunPgaSelfTest();   // pure CPU: the motor conventions, pinned first
            ok &= RunDxSelfTest();        // M7v: the DX12 contract gate (CB parity via
                                          // reflection, the sampler law, AST anchors)
            ok &= RunGaSelfTest();        // pure CPU: GA products + the frame/orientation
                                          // ledger as executable contract (M7j)
            ok &= RunComposeSelfTest();   // pure CPU: the layer compositor's contracts
            ok &= RunWaterSelfTest();     // pure CPU: the water atlas' datum/epoch/field gates
            ok &= RunTileSelfTest(gpu, sc, opt.shaderDir);
            ok &= RunAtlasSelfTest(gpu, sc, opt.shaderDir);
            ok &= RunThreadSelfTest();    // the thread instrument's own gate: it must SEE a race
            ok &= RunSimClockSelfTest();  // the scene clock: whole quanta, framing-independent
            gpu.Shutdown();
            return ok ? 0 : 1;
        }

        // ---- M1: the tide viewer.
        TideModel model;
        if (!model.Load(opt.tidesPath)) {
            Log("FATAL: no tide data at '%s'.", opt.tidesPath.c_str());
            Log("Run the harvester once (network, cached forever after):");
            Log("    py -3 harvester\\harvest_tides.py");
            return 1;
        }

        Window window;
        if (!opt.headless) {
            if (!window.Create(opt.width, opt.height, L"GAGAME")) return 1;
        }

        // M7k: the PIX capturer must be resident BEFORE device creation.
        if (opt.pixFrames > 0) PixLoadGpuCapturer();
        Gpu gpu;
        gpu.Init(opt.headless ? nullptr : window.Handle(), opt.width, opt.height, opt.debugLayer,
                 opt.noVsync && !opt.headless);
        if (!opt.headless) {
            Log("[window] client area %ux%u (requested %ux%u; the swapchain matches the client "
                "rect, not the outer window)",
                window.Width(), window.Height(), opt.width, opt.height);
        }

        RendererDesc rd;
        rd.shaderDir = opt.shaderDir;
        Renderer renderer;
        renderer.Init(gpu, rd);
        if (opt.gpuTime) renderer.EnableGpuProfiler();

        FieldSet fields;
        fields.Init(gpu, L".", 1.0f, 1.0f);

        // Registration order IS draw order: sky (backdrop, depth off), then the tide product.
        auto skyOwned = std::make_unique<SkyLayer>();
        SkyLayer* sky = skyOwned.get();
        sky->Configure(opt.shaderDir);
        sky->Init(gpu, renderer.Shaders(), fields, renderer.RootSignature());
        renderer.AddLayer(std::move(skyOwned));

        auto tideOwned = std::make_unique<TideLayer>();
        TideLayer* tide = tideOwned.get();
        tide->Configure(opt.shaderDir, &model, opt.exaggeration);
        tide->Init(gpu, renderer.Shaders(), fields, renderer.RootSignature());
        renderer.AddLayer(std::move(tideOwned));

        // The open sea (M2) is optional until harvest_waves.py has run once.
        SeaState seaState;
        SeaLayer* sea = nullptr;
        if (seaState.Load(opt.seaPath)) {
            auto seaOwned = std::make_unique<SeaLayer>();
            sea = seaOwned.get();
            sea->Configure(opt.shaderDir, &seaState);
            sea->foamIntensity = opt.foam;
            sea->pixelWater = opt.pixelWater;   // M9bh: shade in PsMain, not DsMain
            sea->targetEdgePx = opt.edgePx;
            sea->sweCurrentGain = opt.sweGain;
            sea->heightScale = opt.heightScale;
            sea->atlasVisualize = opt.viz;
            sea->wireframe = opt.surfaceDebug == 1;
            sea->Init(gpu, renderer.Shaders(), fields, renderer.RootSignature());
            if (opt.stormHs > 0.01f) sea->SetStorm(opt.stormHs, opt.stormTp, opt.stormDir);
            renderer.AddLayer(std::move(seaOwned));
        } else {
            Log("[main] no sea state (run: py -3 harvester\\harvest_waves.py)");
        }

        // M3: currents -- the ACT tidal clock for the sea's jet, the GoMOFS field for the gulf.
        CurrentModel currents;
        const bool haveCurrents = currents.Load(opt.currentsPath);
        if (haveCurrents && sea) sea->SetCurrents(&currents);
        if (!haveCurrents) {
            Log("[main] no currents (run: py -3 harvester\\harvest_currents.py)");
        }

        // M5c: the MLLW -> NAVD88 join, now resolved from CO-OPS datums unless --datum forces it.
        const float datumOff = opt.datumSet ? opt.datumOff : ResolveDatum(model);

        // ---- M6w: THE ONE BED. The planets' CPU models, the raw CUDEM planes, and the
        // composed HEIGHT channel all come up BEFORE the solver -- because the solver's bed
        // is no longer a private file: it is REALIZED from the same painted stack the
        // renderer's tiles come from (ETOPO <- NE-15s <- the CUDEM windows <- hand edits).
        const bool marsMode = (opt.planet == "mars");
        GlobeModel globeModel;
        GlobeModel marsModel;
        if (marsMode) marsModel.LoadMars("data/globe/globe.json");
        const bool globeDataOk = globeModel.Load("data/globe/globe.json");
        GlobeModel& activeGlobe =
            (marsMode && marsModel.Ready()) ? marsModel : globeModel;

        // The raw CUDEM planes are SOURCES (immutable after load -- their bytes are part of
        // the tile-cache identity). capeann/boston are the M6w HQ insets: channel-only, no
        // solver of their own yet.
        BathyModel bathyRaw, bathyCapeAnn, bathyBoston;
        const bool haveBathyRaw = bathyRaw.Load(opt.bathyPath);
        bathyCapeAnn.Load("data/bathy/capeann.json");
        bathyBoston.Load("data/bathy/boston.json");

        Compositor compositor;
        // M9n: THE HEIGHT STACK'S VERTICAL DATUM, resolved once and applied at the sources.
        // ETOPO is MSL/geoid referenced and everything above it in the stack is NAVD88; the two
        // were blended as one height for as long as the stack has existed. The link is published
        // -- CO-OPS carries MSL and NAVD88 on one station staff -- so it is derived, logged, and
        // handed to the ETOPO sources, which means the CORRECTION REACHES EVERY CONSUMER of the
        // stack rather than only the products that happen to know about it.
        double navdAboveMsl = 0.0;
        std::string datumFrom = "none";
        const bool haveMslLink = NavdAboveMsl(model, &navdAboveMsl, &datumFrom);
        const double mslToNavd = haveMslLink ? -navdAboveMsl : 0.0;
        if (haveMslLink) {
            Log("[datum] height stack: MSL -> NAVD88 = %+.3f m, from CO-OPS published datums at "
                "%s (regional, not GEOID18) -- applied to the ETOPO layers at the source",
                mslToNavd, datumFrom.c_str());
        } else {
            Log("[datum] height stack: NO station publishes both MSL and NAVD88 -- ETOPO stays "
                "in its own datum and the stack blends two frames (pre-M9n behaviour)");
        }
        EquirectHeightSource srcEtopo("noaa.etopo2022", "equirect-grid int16 8192x4096",
                                      489200.0, &globeModel.Elev(), globeModel.Nx(),
                                      globeModel.Ny(), mslToNavd);
        EquirectHeightSource srcMola("nasa.mola.megdr16", "equirect-grid int16 5760x2880",
                                     369700.0, &marsModel.Elev(), marsModel.Nx(),
                                     marsModel.Ny());
        std::unique_ptr<WindowHeightSource> srcNe15;
        std::unique_ptr<CudemHeightSource> srcCudem, srcCudemCA, srcCudemBos;
        EditsHeightSource srcEdits;   // M6w: the hand edits ARE a stack layer now -- their
                                      // cache identity is the geojson content, so an operator
                                      // edit repaints exactly the touched tiles at every rung
        int hgtCh = -1;
        if (marsMode && marsModel.Ready()) {
            hgtCh = compositor.AddHeightChannel("mars.height", {&srcMola});
        } else if (globeDataOk) {
            std::vector<HeightSource*> hstack{&srcEtopo};
            if (globeModel.NeNx() > 0) {
                srcNe15 = std::make_unique<WindowHeightSource>(
                    "noaa.etopo15s.ne", "window-grid int16 1440x1200", 46100.0,
                    &globeModel.NeElev(), globeModel.NeNx(), globeModel.NeNy(),
                    globeModel.NeLon0(), globeModel.NeLat1(), globeModel.NeDLon(),
                    globeModel.NeDLat(), 0.04, mslToNavd);
                hstack.push_back(srcNe15.get());
            }
            if (bathyCapeAnn.Ready()) {
                srcCudemCA = std::make_unique<CudemHeightSource>(&bathyCapeAnn, 0.04,
                                                                 "noaa.cudem.capeann");
                hstack.push_back(srcCudemCA.get());
            }
            if (bathyBoston.Ready()) {
                srcCudemBos = std::make_unique<CudemHeightSource>(&bathyBoston, 0.04,
                                                                  "noaa.cudem.boston");
                hstack.push_back(srcCudemBos.get());
            }
            if (haveBathyRaw) {
                srcCudem = std::make_unique<CudemHeightSource>(&bathyRaw);
                hstack.push_back(srcCudem.get());
            }
            if (srcEdits.Load("data/gis/edits.geojson", 2.5f)) {
                hstack.push_back(&srcEdits);
            }
            hgtCh = compositor.AddHeightChannel("earth.height", std::move(hstack));
        }

        // M6v/M8i: THE WATER ATLAS -- water parameters through the same registry. 18 phasor
        // channels (water.tide.M2..O1): equilibrium base <- EOT20 global medium <- the NE
        // station field (20 CO-OPS fits, Merrimack to Scituate), epoch-laddered, cached as
        // RG16F window tiles on demand. The sim manager consumes these next.
        WaterAtlas waterAtlas;
        waterAtlas.Init(compositor, model, "data/water");
        if (!opt.waterMap.empty() || !opt.bathyMap.empty()) {
            VectorPack mapVec;
            mapVec.Load("data/vectors/vectors.vpack");
            if (!opt.waterMap.empty()) {
                RenderWaterMap(compositor, waterAtlas, mapVec, globeModel, opt.waterMap);
            }
            if (!opt.bathyMap.empty()) {
                RenderWaterMap(compositor, waterAtlas, mapVec, globeModel, opt.bathyMap,
                               hgtCh);
            }
            gpu.WaitIdle();
            gpu.Shutdown();
            return 0;
        }

        // M5: the CUDEM terrain. Registered AFTER tide (chart) and BEFORE sea so the opaque
        // land draws first and the water covers only what it actually stands above.
        // M6w: the SOLVER'S grid realizes from the channel -- one bed for the solver, the
        // renderer, and every future physics product. Fallback (no channel): raw + walls.
        BathyModel bathy;
        TerrainLayer* terrain = nullptr;
        if (haveBathyRaw && bathy.Load(opt.bathyPath)) {
            if (hgtCh >= 0 && !marsMode) {
                bathy.RealizeFromChannel(compositor, hgtCh);
            } else {
                bathy.ApplyMaskEdits("data/gis/edits.geojson", 2.5f);
            }
            auto terrOwned = std::make_unique<TerrainLayer>();
            terrain = terrOwned.get();
            terrain->Configure(opt.shaderDir, &bathy);
            terrain->Init(gpu, renderer.Shaders(), fields, renderer.RootSignature());
            renderer.AddLayer(std::move(terrOwned));
            // M9k/M9n: THE BED, through GA Load -> normalize -> GA Compose (the six-layer
            // height stack, LayeredOver) -> a reserved, paged, mipped sparse array. Built HERE,
            // before anything binds a bed, because the consumers below now take the bank: the
            // solver, the sea shader, the churn kernel and the water bank all read one bed and
            // it has to exist before the first of them asks.
            // M9ar: the bed bank is NOT built. The height megatexture (the height page tenant,
            // slice 6 = the z14 survey page) is the only bed on the GPU; the solver, the sea
            // shader, the water bank and the globe all read it. The bank was a second
            // realization of the same six layers -- proved equal at 0.0000 m in section 28,
            // which is exactly why it can go.

            if (sea) {
                // M9n: THE BED IS NOW THE GA BANK. Proved equal to the committed texture at
                // 0.0000 m across all 2187162 texels, through GA Load -> normalize -> Compose
                // (six layers, LayeredOver) -> a reserved, paged, mipped sparse array. M9an:
                // the committed texture is GONE; the bank is the bed and there is no fallback.
                // The sea keeps the survey's WORLD frame (the eta atlas is aligned to it);
                // 0 in the SRV slot means "a survey window exists" and is never sampled.
                Log("[bed] the sea and the solver read the height megatexture -- the only bed");
                sea->SetBathy(0u, bathy.WorldX0(), bathy.WorldZ0(), bathy.WorldSizeX(),
                              bathy.WorldSizeZ());
            }
        } else {
            Log("[main] no bathymetry (run: py -3 harvester\\harvest_bathy.py); open-ocean sea");
        }

        // M5c: the sparse shallow-water solver -- the estuary's own hydrodynamics, tide-forced
        // offshore and river-forced upstream, feeding the sea's mean surface and currents.
        SweSolver swe;
        double riverQ = 70.0;
        if (terrain && sea && !opt.sweOff) {
            // ---- M9m: THE COMPOSE TREE, and the tide step that forced it into existence.
            //
            // Depth is not a dataset anyone ships. It is water level minus bed, and those two
            // arrive in DIFFERENT VERTICAL DATUMS from different domains on different clocks --
            // which is why it needed a tree rather than another entry in a source list. The
            // compositor blends sources that disagree about ONE quantity; this combines two
            // quantities into a third, so it is a different node with a different rule.
            //
            // The join used to be a constant in a comment in BathyModel. GaUnits made guessing
            // it illegal, and the number it demanded turned out to already exist per station as
            // the CO-OPS link, so the tide now carries its own frame and the graph shows it.
            {
                LoaderRegistry breg;
                breg.Register("json", GeoGridLoader::Open);
                if (auto bld = breg.Open("data/bathy/merrimack.json")) {
                    auto bed = Normalize(std::make_shared<RasterSource>(std::move(bld), 0),
                                         UnitSpec::Of(Quantity::Length, "NAVD88"));
                    // The tide arrives in the frame its harmonics were fitted in, with every
                    // station. The MLLW -> NAVD88 link is then DECLARED, using the number main
                    // already resolved and logged -- an estimate for Newburyport, which has no
                    // published link, and now an estimate visible in the graph instead of buried.
                    auto tideMllw = std::make_shared<TideSource>(&model);
                    auto tideNavd = DeclareDatumLink(tideMllw, datumOff, "NAVD88",
                                                     "main::ResolveDatum -- see [datum] above");
                    auto depth = std::make_shared<BinaryFieldSource>(
                        "water.depth", BinaryFieldSource::Op::Subtract, tideNavd, bed);
                    PrintTree("water.depth = tide - bed", depth.get());

                    // The guard, demonstrated rather than asserted: the SAME subtraction with
                    // the tide left in MLLW. Both operands are lengths, both look like metres,
                    // and the answer would be wrong by the datum link at every texel forever.
                    BinaryFieldSource bad("water.depth.WRONG-DATUM",
                                          BinaryFieldSource::Op::Subtract, tideMllw, bed);
                    Log("[tree] MLLW tide - NAVD88 bed: %s",
                        bad.Valid() ? "accepted -- THE GUARD IS BROKEN" : "refused, as it must be");

                    // Equivalence against the engine's own water level. main computes
                    // oceanAt(t) = Height(focus,t) + datumOff with datumOff from ResolveDatum;
                    // the tree reaches the same number through a declared per-station link.
                    const int fs = model.Focus();
                    double worstLvl = 0.0, worstDep = 0.0;
                    uint32_t probed = 0;
                    for (int k = 0; k < 24; ++k) {
                        const double t = model.EpochUnix() + k * 3600.0;
                        DomainQuery q;
                        q.lon = model.S(size_t(fs)).lon;
                        q.lat = model.S(size_t(fs)).lat;
                        q.unixT = t;
                        q.groundM = 10.0;
                        DomainValue lv;
                        if (!tideNavd->SampleAt(q, lv) || lv.weight <= 0.0f) continue;
                        const double engine = model.Height(size_t(fs), t) + datumOff;
                        worstLvl = (std::max)(worstLvl, std::abs(double(lv.c[0]) - engine));
                        DomainValue dv;
                        if (depth->SampleAt(q, dv) && dv.weight > 0.0f) {
                            DomainValue bv;
                            if (bed->SampleAt(q, bv) && bv.weight > 0.0f) {
                                worstDep = (std::max)(
                                    worstDep, std::abs(double(dv.c[0]) - (engine - bv.c[0])));
                            }
                        }
                        ++probed;
                    }
                    Log("[tree] %u hourly probes at %s: worst |tree - engine| level %.4f m, "
                        "depth %.4f m (%s)",
                        probed, model.S(size_t(fs)).name.c_str(), worstLvl, worstDep,
                        (worstLvl < 0.01 && worstDep < 0.01) ? "equivalent"
                                                             : "DIVERGENT -- check the link");
                }
            }
            swe.Init(gpu, renderer.Shaders(), opt.shaderDir, bathy);   // bed bound below
            // M6r: the discharge is LIVE again -- it rides the Flather boundary's u_ext (the
            // station stage still carries it into eta; the prism term dwarfs it either way).
            riverQ = (opt.riverQ > 0) ? opt.riverQ : LoadRiverDischarge("data/river/river.json");
            sea->SetSwe(&swe);
            sea->SetBathyCpu(&bathy);
        }
        GulfLayer* gulf = nullptr;
        if (haveCurrents && currents.Field().Valid()) {
            auto gulfOwned = std::make_unique<GulfLayer>();
            gulf = gulfOwned.get();
            gulf->Configure(opt.shaderDir, &currents);
            gulf->Init(gpu, renderer.Shaders(), fields, renderer.RootSignature());
            renderer.AddLayer(std::move(gulfOwned));
        }

        // M7: THE WAVE VERTEX BANK -- the water's geometry as ONE tiled resource, filled
        // per frame from everything the weather manager federates, sampled by the globe in
        // one-water mode. Registered BEFORE the planet so the rings are recomposed (and the
        // cascades already advanced by SeaLayer) when the globe's meshlets sample them.
        // M8h: the water scene loads BEFORE the layers stand up -- the bank's ring
        // density (bankTexelM) is a construction-time choice. Hot-reload still lives
        // in the frame loop; geometry-of-the-bank changes need a restart (logged).
        WaterSceneConfig waterScene;
        long long waterSceneMtime = 0;
        const char* kScenePath = "data/wave_scene.json";
        LoadWaterScene(kScenePath, waterScene);
        WaterSceneChanged(kScenePath, &waterSceneMtime);
        // Step 3 (docs/PERF_EXPERIMENT.md): the directory watches the file; the frame polls
        // one atomic instead of paying the 0.14-0.19 ms stat through the data/ junction.
        WaterSceneWatch sceneWatch;
        sceneWatch.Start(kScenePath);

        WaterBankLayer* waterBank = nullptr;
        if (sea && bathy.Ready() && !marsMode) {
            auto wbOwned = std::make_unique<WaterBankLayer>();
            waterBank = wbOwned.get();
            waterBank->Configure(opt.shaderDir, sea, &swe, &bathy, &waterAtlas, &compositor,
                                 hgtCh, &globeModel, &seaState);
            waterBank->SetBaseTexel(waterScene.bankTexelM);   // M8h ring density (scene)
            waterBank->flatBed = opt.flatBed;
            waterBank->flatBedNavd = opt.flatBedNavd;
            if (opt.flatBed) {
                Log("[bed] --flat-bed %.1f m NAVD: the bank fills against a CONSTANT floor. Diff "
                    "this run's wireframe against a normal one -- whatever differs is what "
                    "bathymetry does to the MESH, with shading held out of it.",
                    opt.flatBedNavd);
            }
            waterBank->Init(gpu, renderer.Shaders(), fields, renderer.RootSignature());
            renderer.AddLayer(std::move(wbOwned));
            sea->drawEnabled = !opt.oneWater;
            if (opt.oneWater) {
                Log("[waterbank] ONE-WATER: the SeaLayer grid retires; the globe's meshlets "
                    "displace from the bank");
            }
        }

        // M6: the planet (registered LAST -- it borrows root param 2 for its node list).
        // M6f: which planet is a MODEL choice -- Mars loads MOLA into the same relief fields
        // and the whole pipeline (texture, mips, camera clamps) serves it unchanged.
        // (M6w: the models + the compositor's height side were HOISTED above the bathy/solver
        // block -- the solver's bed realizes from the channel.)
        GlobeLayer* globe = nullptr;
        if (globeDataOk) {
            auto globeOwned = std::make_unique<GlobeLayer>();
            globe = globeOwned.get();
            globe->Configure(opt.shaderDir, &activeGlobe);
            globe->marsReliefValid = marsMode && marsModel.Ready();
            globe->windOverlay = opt.viz;
            globe->surfaceDebug = opt.surfaceDebug;
            globe->meshStats = opt.meshStats;
            globe->msSurface = opt.msSurface;
            globe->albedoLens = opt.albedo;
            globe->Init(gpu, renderer.Shaders(), fields, renderer.RootSignature());
            renderer.AddLayer(std::move(globeOwned));
            // M6j: with the unified mesh surface active, the terrain layer stops rendering
            // (it keeps the heightfield the SWE physics and the sea's bed read).
            if (terrain && globe->MeshPathActive()) terrain->renderEnabled = false;
        } else {
            Log("[main] no globe data (run: py -3 harvester\\harvest_globe.py)");
        }

        // ---- M6e: the unified residency manager + the streamed planet surfaces. Two worlds,
        // one machine: Mars streams the rescued sample's BC1/BC5 pyramids from disk; Earth
        // streams Google 2D tiles (cache-first, throttled, budget-capped) reprojected onto the
        // same cube faces. Residency is driven by the CDLOD walk, clamped by residency-map
        // cubes, prefetched along the camera's screw.
        const double planetR = marsMode ? 3389500.0 : GlobeModel::kR;
        ResidencyManager resMgr;
        MarsBinProvider marsDiff, marsNorm;
        GoogleTileProvider googleTiles;

        // ---- M6i: THE LAYER COMPOSITOR's color side (the height side moved above the
        // solver, M6w). Sources register their schemas; channels stack them in order;
        // realizations paint composed quadtrees ONCE and cache every 64KB tile.
        GoogleColorSource srcGoogle(&googleTiles);
        BedSynthSource srcBed;          // M7d: the bed classifier -- the first synthesis
        SeafloorReliefSource srcRelief; // M9av: the seafloor's appearance from the ingested bathymetry
                                        // node; its program is data/bed/bed_rules.json
        AerialOrthoSource srcAerial;    // M6l: MassGIS 15 cm orthos (loads if harvested)
        AerialOrthoSource srcOverlay;   // M6o: user GeoTIFF overlays -- ALPHA IS FIBER: a
                                        // mostly-transparent highlights plane bleeds through
                                        // the composed quadtree pixel by pixel
        GisStencil gisStencil;   // survey vectors + mask realizations (GSHHG/WDBII)
        // M9ak: the SAME survey, as rings rather than as a parity fill -- the compositor's
        // land/sea gate. Neither .raw mask is opened by this one.
        GisVectorMask gisMask;
        GisMaskSource srcGisMask(&gisMask);
        VectorPack vectors;      // M6p: lossless vector layers, LOD by wedge importance
        GisLayer* gisLayer = nullptr;

        Exchange exchange;       // M6j: the plugin bus -- named GA buffer channels
        const double winOrgX = 4935.0 * 256.0, winOrgY = 6008.0 * 256.0;   // Merrimack z14 px
        int colorCubeT = -1, winTenant = -1, hgtTenant = -1, hgtWinTenant = -1;
        int maskTenant = -1;   // M9ay: the survey mask pages (gis.landsea's tree)
        int detTenant = -1;   // M7f: z17 detail color window
        double det17OrgX = 0.0, det17OrgY = 0.0;
        int colCh = -1;   // color channel id (hgtCh registered above the solver, M6w)
        // M9am: the megatexture graph and its on-disk tile cache. Declared HERE, beside the
        // tenants that hold providers into them, so they cannot die first.
        std::vector<std::shared_ptr<DomainSource>> megaKeep;
        std::unique_ptr<TileTree> megaTree;
        // M9as: THE HEIGHT GRAPH AND ITS DISK TREE. BuildHeightStack's six layers, each its own
        // sparse tree on the NVMe (FloatW: value + coverage), composed LayeredOver into ONE root
        // whose tiles are the R16F pages the height tenant reads. Inputs are ordered by
        // fidelity -- coarsest first, so the deepest tree paints over everything it intersects
        // (the user's rule) -- and the order is printed, not assumed.
        std::shared_ptr<DomainSource> heightRoot;
        std::unique_ptr<TileTree> heightTree;
        // M9ba: THE SWELL EXPOSURE AS A TREE NODE. A DomainSource (the LOS march over the height
        // stack), a one-input compose root so the tree materializes R16F, the tree behind a
        // holder so a bucket roll swaps it under the tenant's provider, and the tenant itself.
        std::shared_ptr<ExposureSource> exposureSrc;
        std::shared_ptr<DomainSource> exposureRoot;
        // shared_ptr, swapped ATOMICALLY: loader threads are inside the old tree's provider
        // when a bucket rolls; they hold their own reference until their paint returns.
        std::shared_ptr<std::shared_ptr<TileTree>> exposureTree;
        int exposureT = -1;
        if (globe) {
            resMgr.Init(gpu);
            resMgr.ringLoads = opt.ringLoads;
            resMgr.dsSerial = opt.dsSerial;
            resMgr.traceRes = opt.resTrace;
            int surf = -1, norm = -1;
            if (marsMode) {
                // Mars: color/normal stay NATIVE streams (the rescued sample's pyramids are
                // already tile-shaped truth); height composes from MOLA through the same
                // machinery Earth uses -- one relief path in the shader for both planets.
                if (marsDiff.Open(L"data/earth/diffuse.bin", DXGI_FORMAT_BC1_UNORM_SRGB)) {
                    surf = resMgr.AddTextureCube(gpu, L"mars.diffuse (the rescued sample)",
                                                 16384, DXGI_FORMAT_BC1_UNORM_SRGB,
                                                 marsDiff.Fn());
                }
                if (marsNorm.Open(L"data/earth/normal.bin", DXGI_FORMAT_BC5_SNORM)) {
                    norm = resMgr.AddTextureCube(gpu, L"mars.normal", 16384,
                                                 DXGI_FORMAT_BC5_SNORM, marsNorm.Fn());
                }
                if (marsModel.Ready() && hgtCh >= 0) {
                    hgtTenant = resMgr.AddTextureCube(gpu, L"mars.height (composed: MOLA)",
                                                      Compositor::kFaceDim,
                                                      DXGI_FORMAT_R16_FLOAT,
                                                      compositor.CubeHeight(hgtCh));
                }
                globe->SetComposed(-1, -1, hgtTenant, -1, 0.0, 0.0, 1.0);
            } else {
                // earth.height REGISTERED above the solver (M6w) -- here it becomes GPU
                // tenants: the global cube and the Merrimack z14 window.
                // M9aq: ONE height tenant -- pages 0..5 the cube faces, 6 the z14 Mercator
                // page (the frame the colour window shares, so the near-field land/sea gate
                // and the normals ride CUDEM truth). One provider dispatching on the slice.
                // M9as: fed by the height TileTree when --color-trees is on.
                if (opt.colorTrees || opt.treeAudit) {
                    auto layers = BuildHeightStack(compositor, hgtCh);
                    std::vector<std::pair<double, std::shared_ptr<DomainSource>>> byRes;
                    const Compositor::Channel& hch = compositor.ChannelAt(hgtCh);
                    for (size_t i = 0; i < layers.size() && i < hch.height.size(); ++i) {
                        byRes.push_back({hch.height[i]->Info().cmPerPixel, layers[i]});
                    }
                    std::stable_sort(byRes.begin(), byRes.end(),
                                     [](const auto& a, const auto& b) { return a.first > b.first; });
                    auto dc = std::make_shared<DomainCompositor>();
                    dc->SetBlend(DomainCompositor::Blend::LayeredOver);
                    std::string order;
                    for (auto& p : byRes) {
                        if (dc->Add(p.second)) order += " " + std::string(p.second->Name()) +
                                                        "(" + std::to_string(int(p.first)) + "cm)";
                    }
                    heightRoot = std::make_shared<CompositeSource>("earth.height", dc);
                    for (auto& l : layers) megaKeep.push_back(l);
                    Log("[height-tree] compose order, coarsest first (the deepest tree paints "
                        "last):%s",
                        order.c_str());
                    heightTree = std::make_unique<TileTree>(heightRoot.get(), TileTree::Fmt::Half);
                    heightTree->onChanged = [&resMgr, &hgtTenant](const std::string& tag,
                                                                  const TileRequest& r) {
                        if (hgtTenant < 0) return;
                        TileRequest q = r;
                        if (tag.rfind("window_z14", 0) == 0) q.face = 6u;
                        resMgr.Invalidate(hgtTenant, q);
                    };
                    heightTree->Print();
                }
                {
                    const TileProviderFn hCube = (opt.colorTrees && heightTree)
                        ? heightTree->Provider(ColorFrame::Cube(Compositor::kFaceDim, 256, 128))
                        : compositor.CubeHeight(hgtCh);
                    const TileProviderFn hWin = (opt.colorTrees && heightTree)
                        ? heightTree->Provider(ColorFrame::Window(1263360, 1538048, 14, 256, 128))
                        : compositor.WindowHeight(hgtCh, 1263360, 1538048, 16384, 14);
                    TileProviderFn hPages = [hCube, hWin](const TileRequest& r,
                                                          std::vector<uint8_t>& out,
                                                          TileLoc* loc) {
                        if (r.face < 6) return hCube(r, out, loc);
                        TileRequest w = r;
                        w.face = 0;
                        return hWin(w, out, loc);
                    };
                    hgtTenant = resMgr.AddTexturePages(gpu, L"earth.height (megatexture pages)",
                                                       Compositor::kFaceDim,
                                                       DXGI_FORMAT_R16_FLOAT, std::move(hPages),
                                                       7);
                    hgtWinTenant = hgtTenant;   // pages mode: the window is slice 6
                    // M9ar: the solver's bed, and the churn kernel's, is slice 6 of this tenant.
                    if (swe.Ready()) {
                        swe.SetHeightPage(gpu, resMgr.TextureRes(hgtTenant),
                                          resMgr.ResidencyRes(hgtTenant), 6u,
                                          resMgr.Mips(hgtTenant), 1263360.0, 1538048.0);
                    }
                        if (sea && hgtCh >= 0 && opt.exposure) {
                            exposureSrc = std::make_shared<ExposureSource>(&compositor, hgtCh);
                            auto xdc = std::make_shared<DomainCompositor>();
                            xdc->SetBlend(DomainCompositor::Blend::LayeredOver);
                            if (!xdc->Add(exposureSrc)) Log("[exposure] compose REFUSED the node");
                            exposureRoot = std::make_shared<CompositeSource>("swell.exposure", xdc);
                            exposureTree = std::make_shared<std::shared_ptr<TileTree>>(
                                std::make_shared<TileTree>(exposureRoot.get(), TileTree::Fmt::Half));
                            (*exposureTree)->onChanged = [&resMgr, &exposureT](const std::string&,
                                                                               const TileRequest& r) {
                                if (exposureT < 0) return;
                                TileRequest q = r;
                                q.face = 6u;
                                resMgr.Invalidate(exposureT, q);
                            };
                            auto holder = exposureTree;
                            TileProviderFn xp = [holder](const TileRequest& r,
                                                         std::vector<uint8_t>& out, TileLoc* loc) {
                                if (r.face != 6) {
                                    // The cube faces are not this node's frame: answer "fully
                                    // exposed" (R16F 1.0) so the boot's coarsest loads and any
                                    // stray want land once instead of retrying forever.
                                    out.assign(65536, 0);
                                    uint16_t* h = reinterpret_cast<uint16_t*>(out.data());
                                    for (size_t i = 0; i < 32768; ++i) h[i] = 0x3C00u;
                                    if (loc) *loc = TileLoc{};
                                    return true;
                                }
                                std::shared_ptr<TileTree> t = std::atomic_load(holder.get());
                                if (!t) return false;
                                TileRequest w = r;
                                w.face = 0;
                                return t->Provider(
                                    ColorFrame::Window(1263360, 1538048, 14, 256, 128))(w, out, loc);
                            };
                            exposureT = resMgr.AddTexturePages(gpu, L"swell.exposure (pages)",
                                                               Compositor::kFaceDim,
                                                               DXGI_FORMAT_R16_FLOAT, std::move(xp),
                                                               7);
                            sea->SetExposurePage(resMgr.TextureSrv(exposureT),
                                                 resMgr.ResidencySrv(exposureT), exposureSrc.get());
                            Log("[exposure] swell.exposure is page tenant %d: the LOS march over "
                                "the height stack, cached per (direction, level) bucket, read at "
                                "page mips >= 3 (%.0f m)",
                                exposureT, 9.55 * 8.0);
                        }
                    if (sea) {
                        sea->SetHeightPage(resMgr.TextureRes(hgtTenant),
                                           resMgr.ResidencyRes(hgtTenant), 6u,
                                           resMgr.Mips(hgtTenant), 1263360.0, 1538048.0);
                    }
                }
                // earth.color: the Google mercator tree, realized twice -- the global cube
                // and the Merrimack z14 window (same stack, deeper footprint).
                if (googleTiles.Init("satellite", opt.tileBudget)) {
                    googleTiles.SetFetchCounter(&resMgr.fetchesThisRun);
                    // M6l: the MassGIS 15 cm plane orthos paint ABOVE Google wherever they
                    // have coverage -- the compositor's first independent high-res layer,
                    // aligned by its own declared projection (EPSG:6348), not by luck.
                    std::vector<ColorSource*> colorStack{&srcGoogle};
                    size_t bedLayer = SIZE_MAX, maskLayer = SIZE_MAX;
                    // M7x (user catch): the ortho was painting its capture-day WATER over the
                    // drained-bed albedo -- a hard-edged dark rectangle the sea shader then
                    // attenuated AGAIN. Photos are LAND authorities; the bed classifier is
                    // the WATER authority. The stack order encodes that ranking: google under
                    // aerial (both photos, finer wins), bed above both (its height-band alpha
                    // reclaims everything below the intertidal ramp and hands land back to
                    // the photos above +1.2 m NAVD), hand overlays on top of everything.
                    bool srcAerialLoaded = false, srcOverlayLoaded = false;
                    if (srcAerial.Load("data/aerial/aerial.json")) {
                        colorStack.push_back(&srcAerial);
                        srcAerialLoaded = true;
                    }
                    // M9av: the GLOBAL seafloor under the bed classifier -- the ingested
                    // bathymetry's hillshade x sediment ramp, every ocean texel; the classifier
                    // keeps its authority inside its own box by painting over it.
                    size_t reliefLayer = SIZE_MAX;
                    if (opt.seafloor &&
                        srcRelief.Load("data/bed/seafloor_rules.json", &compositor, hgtCh)) {
                        colorStack.push_back(&srcRelief);
                        reliefLayer = colorStack.size() - 1;
                    }
                    if (srcBed.Load("data/bed/bed_rules.json", &compositor, hgtCh)) {
                        colorStack.push_back(&srcBed);
                        bedLayer = colorStack.size() - 1;
                    }
                    if (srcOverlay.Load("data/overlay/overlay.json")) {
                        colorStack.push_back(&srcOverlay);
                        srcOverlayLoaded = true;
                    }
                    // M9ak: THE GATE. The user's rule: "the GIS mask gates, the height band
                    // refines". srcBed is the WATER authority and its alpha is a height band,
                    // which cannot tell an inland hollow below +1.2 m NAVD from the sea; the
                    // survey can, and cannot place a waterline to the metre. So the survey
                    // multiplies the bed's weight and the height band decides where inside it.
                    //
                    // The mask is a LAYER (so it earns a cache identity and its own tree on the
                    // same addresses as the imagery) that PAINTS NOTHING. Its footprint is the
                    // rings' own bounds, so every tile outside New England keeps the identity
                    // it already has -- the global cube is not repainted for this.
                    const size_t bedIdx = bedLayer;
                    if (opt.gisGate && (bedIdx != SIZE_MAX || reliefLayer != SIZE_MAX) &&
                        gisMask.Load("data/gis/")) {
                        srcGisMask.Refresh();   // the rings are loaded: declare the real box
                        if (!opt.gisDump.empty()) {
                            // M9av: LOOK AT THE GATE. 255 = water (or no opinion), 0 = land.
                            double b0, a0, b1, a1;
                            gisMask.Bounds(b0, a0, b1, a1);
                            const uint32_t dim = 1024;
                            std::vector<uint8_t> g;
                            gisMask.RasterizeGate(a0, a1, b0, b1, dim, g);
                            std::vector<uint8_t> v(static_cast<size_t>(dim) * dim);
                            for (size_t i = 0; i < v.size(); ++i) {
                                // value where surveyed; 128 (grey) where the mask has no opinion
                                v[i] = (g[i * 2 + 1] & 1u) ? g[i * 2] : 128u;
                            }
                            std::ofstream pf(opt.gisDump, std::ios::binary);
                            pf << "P5\n" << dim << " " << dim << "\n255\n";
                            pf.write(reinterpret_cast<const char*>(v.data()), v.size());
                            Log("[gismask] --gis-dump: %s (%ux%u over %.3f,%.3f..%.3f,%.3f) %s",
                                opt.gisDump.c_str(), dim, dim, b0, a0, b1, a1,
                                pf ? "written" : "FAILED TO WRITE");
                            std::exit(0);
                        }
                        colorStack.push_back(&srcGisMask);
                        maskLayer = colorStack.size() - 1;
                    }
                    colCh = compositor.AddColorChannel("earth.color", std::move(colorStack));
                    if (maskLayer != SIZE_MAX) {
                        if (bedIdx != SIZE_MAX) compositor.SetColorGate(colCh, bedIdx, maskLayer);
                        if (reliefLayer != SIZE_MAX) {
                            compositor.SetColorGate(colCh, reliefLayer, maskLayer);
                        }
                    }
                    // M9am: THE MEGATEXTURE, AS THE WATER'S GRAPH. The same nodes the bed and
                    // the tide compose through -- ColorLayerSource, NormalizeToSi,
                    // DomainCompositor::LayeredOver, CompositeSource -- with TileTree caching
                    // every node's output on the NVMe on the shared tile addresses:
                    //
                    //     google ---+
                    //               +-> earth.land ----------------+
                    //     aerial ---+                              |
                    //                                              +-> earth.color (mega)
                    //     relief --+                              |
                    //              +-> earth.seafloor --+          |
                    //     bed -----+                    +-> gate --+
                    //     gis.landsea (vector) ---------+
                    //
                    // Land is the base; the seafloor paints over it with weight = its own height
                    // band x the survey's water coverage ("the GIS mask gates, the height band
                    // refines"), so over surveyed water the land goes transparent per pixel and
                    // the seafloor shows; inland the seafloor is gated out and the mega tile is a
                    // stored REFERENCE to the land tree's. The flat incumbent channel above stays
                    // as the definition the audit compares against.
                    std::vector<std::shared_ptr<DomainSource>>& keepAlive = megaKeep;
                    auto leaf = [&](ColorSource* s) -> std::shared_ptr<DomainSource> {
                        auto l = std::make_shared<ColorLayerSource>(s);
                        auto n = NormalizeToSi(l);
                        keepAlive.push_back(l);
                        return n ? n : l;
                    };
                    auto over = [&](const char* name,
                                    std::vector<std::shared_ptr<DomainSource>> in)
                        -> std::shared_ptr<DomainSource> {
                        auto dc = std::make_shared<DomainCompositor>();
                        dc->SetBlend(DomainCompositor::Blend::LayeredOver);
                        for (auto& s : in) {
                            if (!dc->Add(s)) Log("[mega] %s: input %s refused", name, s->Name());
                        }
                        return std::make_shared<CompositeSource>(name, dc);
                    };
                    std::vector<std::shared_ptr<DomainSource>> landIn{leaf(&srcGoogle)};
                    if (srcAerialLoaded) landIn.push_back(leaf(&srcAerial));
                    std::shared_ptr<DomainSource> land = over("earth.land", landIn);
                    std::shared_ptr<DomainSource> mega;
                    std::vector<std::shared_ptr<DomainSource>> seaIn;
                    if (reliefLayer != SIZE_MAX) seaIn.push_back(leaf(&srcRelief));
                    if (bedLayer != SIZE_MAX) seaIn.push_back(leaf(&srcBed));
                    if (!seaIn.empty()) {
                        std::shared_ptr<DomainSource> sea = over("earth.seafloor", seaIn);
                        std::shared_ptr<DomainSource> seaGated = sea;
                        if (maskLayer != SIZE_MAX) {
                            seaGated = std::make_shared<GateSource>("seafloor<gis", sea,
                                                                    leaf(&srcGisMask));
                        }
                        std::vector<std::shared_ptr<DomainSource>> megaIn{land, seaGated};
                        if (srcOverlayLoaded) megaIn.push_back(leaf(&srcOverlay));
                        mega = over("earth.color", megaIn);
                    } else {
                        mega = land;
                    }
                    keepAlive.push_back(land);
                    keepAlive.push_back(mega);
                    PrintTree("earth.color (megatexture)", mega.get());
                    if (opt.colorTrees || opt.treeAudit) {
                        megaTree = std::make_unique<TileTree>(mega.get());
                        // M9bb: a fold or a drop below changed a root tile: the colour tenant
                        // refetches that address (the frame's tag names the page slice).
                        megaTree->onChanged = [&resMgr, &colorCubeT](const std::string& tag,
                                                                     const TileRequest& r) {
                            if (colorCubeT < 0) return;
                            TileRequest q = r;
                            if (tag.rfind("window_z14", 0) == 0) q.face = 6u;
                            else if (tag.rfind("window_z17", 0) == 0) q.face = 7u;
                            resMgr.Invalidate(colorCubeT, q);
                        };
                        megaTree->Print();
                    }
                    auto mkColor = [&](const ColorFrame& f) -> TileProviderFn {
                        return (opt.colorTrees && megaTree) ? megaTree->Provider(f)
                                                            : compositor.ColorRealization(colCh, f);
                    };
                    // M9ap: NO INSET TEXTURES. The planet's colour is ONE tenant -- a reserved
                    // Texture2DArray of pages: slices 0..5 the cube faces, 6 the z14 Mercator
                    // page, 7 the z17 page -- with one SRV, one residency map, one budget, and
                    // one provider that dispatches on the slice. The three tenants this
                    // replaces were three pages of a ladder with hand-off fades between them.
                    const double n17 = 16384.0 * 256.0 * 8.0;
                    {
                        const double piD = 3.14159265358979;
                        const double lonC = -70.8125, latC = 42.8160 * piD / 180.0;
                        const double mx = (lonC + 180.0) / 360.0 * n17;
                        const double my =
                            (0.5 - std::log(std::tan(piD * 0.25 + latC * 0.5)) /
                                       (2.0 * piD)) *
                            n17;
                        det17OrgX = std::floor(mx - 8192.0);
                        det17OrgY = std::floor(my - 8192.0);
                    }
                    const TileProviderFn pCube = mkColor(ColorFrame::Cube(Compositor::kFaceDim));
                    const TileProviderFn pWin = mkColor(ColorFrame::Window(1263360, 1538048, 14));
                    const TileProviderFn pDet = mkColor(ColorFrame::Window(
                        static_cast<long long>(det17OrgX), static_cast<long long>(det17OrgY), 17));
                    TileProviderFn pages = [pCube, pWin, pDet](const TileRequest& r,
                                                               std::vector<uint8_t>& out,
                                                               TileLoc* loc) {
                        if (r.face < 6) return pCube(r, out, loc);
                        TileRequest w = r;
                        w.face = 0;   // a Mercator page is a single-face frame
                        return r.face == 6 ? pWin(w, out, loc) : pDet(w, out, loc);
                    };
                    colorCubeT = resMgr.AddTexturePages(gpu, L"earth.color (megatexture pages)",
                                                        Compositor::kFaceDim,
                                                        DXGI_FORMAT_R8G8B8A8_UNORM_SRGB,
                                                        std::move(pages), 8);
                    winTenant = colorCubeT;   // pages mode: window == cube, slice 6
                    detTenant = colorCubeT;   // slice 7
                    (void)n17;
                    // M9ay: THE SURVEY AS A PAGE TENANT. gis.landsea's own tree -- the vector
                    // rings swept per tile on the SAME addresses as the imagery and the bed --
                    // feeds a third page tenant (r = water coverage, b = edited, a = surveyed).
                    // The classifier reads it; the three committed rasters GisStencil built
                    // from the .raw parity fills are gone (AUDIT_WATER item 2). Addresses the
                    // survey has no opinion about have no tile: the loader marks them NULL after
                    // its retries and the shader falls back to the height sign there.
                    if (opt.colorTrees && megaTree && maskLayer != SIZE_MAX) {
                        if (TileTree* gt = megaTree->Find("gis.landsea")) {
                            gt->onChanged = [&resMgr, &maskTenant](const std::string& tag,
                                                                   const TileRequest& r) {
                                if (maskTenant < 0) return;
                                TileRequest q = r;
                                if (tag.rfind("window_z14", 0) == 0) q.face = 6u;
                                else if (tag.rfind("window_z17", 0) == 0) q.face = 7u;
                                resMgr.Invalidate(maskTenant, q);
                            };
                            const TileProviderFn gCube =
                                gt->Provider(ColorFrame::Cube(Compositor::kFaceDim));
                            const TileProviderFn gWin =
                                gt->Provider(ColorFrame::Window(1263360, 1538048, 14));
                            const TileProviderFn gDet =
                                gt->Provider(ColorFrame::Window(static_cast<long long>(det17OrgX),
                                                                static_cast<long long>(det17OrgY),
                                                                17));
                            TileProviderFn gpages = [gCube, gWin, gDet](const TileRequest& r,
                                                                       std::vector<uint8_t>& out,
                                                                       TileLoc* loc) {
                                if (r.face < 6) return gCube(r, out, loc);
                                TileRequest w = r;
                                w.face = 0;
                                return r.face == 6 ? gWin(w, out, loc) : gDet(w, out, loc);
                            };
                            maskTenant = resMgr.AddTexturePages(
                                gpu, L"gis.landsea (survey mask pages)", Compositor::kFaceDim,
                                DXGI_FORMAT_R8G8B8A8_UNORM, std::move(gpages), 8);
                            Log("[gis] the survey is page tenant %d: r = water coverage, b = "
                                "edited, a = surveyed -- no .raw raster is opened",
                                maskTenant);
                        } else {
                            Log("[gis] no gis.landsea node in the megatexture tree: the "
                                "classifier falls back to the height sign");
                        }
                    }
                    // The gate. Every realization the render path uses, on tiles the shipped
                    // paint loop already wrote, with the worst per-channel disagreement printed.
                    if (opt.treeAudit) {
                        const ColorFrame frames[] = {
                            ColorFrame::Cube(Compositor::kFaceDim),
                            ColorFrame::Window(1263360, 1538048, 14),
                            ColorFrame::Window(static_cast<long long>(det17OrgX),
                                               static_cast<long long>(det17OrgY), 17),
                        };
                        const ColorFrame hframes[] = {
                            ColorFrame::Cube(Compositor::kFaceDim, 256, 128),
                            ColorFrame::Window(1263360, 1538048, 14, 256, 128),
                        };
                        if (opt.packTrees) {
                            std::vector<std::string> tags;
                            for (const ColorFrame& f : frames) tags.push_back(f.Tag());
                            uint32_t n = megaTree->Pack(tags);
                            if (heightTree) {
                                std::vector<std::string> htags;
                                for (const ColorFrame& f : hframes) htags.push_back(f.Tag());
                                n += heightTree->Pack(htags);
                            }
                            Log("[tiletree] packed %u tiles across the megatexture tree; "
                                "references resolve into the archives they name",
                                n);
                            ga::threadaudit::Report();
                            resMgr.Shutdown();
                            return 0;
                        }
                        TreeAudit all;
                        if (heightTree) {
                            for (const ColorFrame& f : hframes) {
                                TreeAudit ha;
                                AuditTileTree(compositor, hgtCh, *heightTree, f, opt.treeAudit,
                                              ha, opt.warmTrees, true, "windowH");
                                Log("[tree-audit] earth.height/%s: %u tiles, %u exact, worst "
                                    "|direct - tree| = %.3f m, %llu of %llu texels differ",
                                    f.Tag().c_str(), ha.tiles, ha.exact, ha.worstDelta / 1000.0,
                                    static_cast<unsigned long long>(ha.difTexels),
                                    static_cast<unsigned long long>(ha.texels));
                            }
                            Log("[trees]\n%s", heightTree->Stats().c_str());
                        }
                        for (const ColorFrame& f : frames) {
                            TreeAudit a;
                            AuditTileTree(compositor, colCh, *megaTree, f, opt.treeAudit, a,
                                          opt.warmTrees);
                            all.tiles += a.tiles;
                            all.exact += a.exact;
                            all.difTexels += a.difTexels;
                            all.texels += a.texels;
                            all.alphaDif += a.alphaDif;
                            all.worstDelta = (std::max)(all.worstDelta, a.worstDelta);
                        }
                        Log("[tree-audit] TOTAL: %u tiles, %u byte-identical (%.1f%%), worst "
                            "|direct - tree| = %u/255, %llu of %llu texels differ (%.4f%%), "
                            "%u cover mismatches",
                            all.tiles, all.exact,
                            all.tiles ? 100.0 * double(all.exact) / double(all.tiles) : 0.0,
                            all.worstDelta,
                            static_cast<unsigned long long>(all.difTexels),
                            static_cast<unsigned long long>(all.texels),
                            all.texels ? 100.0 * double(all.difTexels) / double(all.texels)
                                       : 0.0,
                            all.alphaDif);
                        Log("[trees]\n%s", megaTree->Stats().c_str());
                        ga::threadaudit::Report();   // the tree runs hammer it hardest
                        resMgr.Shutdown();
                        return 0;
                    }
                }
                globe->SetComposed(colorCubeT, winTenant, hgtTenant, hgtWinTenant, winOrgX,
                                   winOrgY, 16384.0, detTenant, det17OrgX, det17OrgY,
                                   maskTenant);

                // ---- M9ae: WHAT THE DISK ALREADY HOLDS, in memory, once.
                //
                // The composed cache is the tree's backing store -- 64 KB tiles laid out for
                // CopyTiles -- but nothing in RAM knew what was in it, so "is a finer level
                // ready here?" could only be answered by building a filename and asking the
                // filesystem. Residency could request a level; it could not prefer the levels
                // that would be a cheap READ over the ones that mean a paint or a fetch.
                //
                // The index is one entry per TILE, not per texel, so the asymmetry that makes
                // NVMe paging worth doing shows up directly: terabytes of tree, megabytes of map.
                {
                    static TileIndex idxColorCube, idxColorWin, idxColorDet, idxHeightCube;
                    idxColorCube.Scan("earth.color", "cube16k");
                    idxColorWin.Scan("earth.color", "window_z14_1263360_1538048");
                    idxColorDet.Scan("earth.color", "window_z17_10168820_12344774");
                    idxHeightCube.Scan("earth.height", "cube16k");
                    idxColorCube.Report();
                    idxColorWin.Report();
                    idxColorDet.Report();
                    idxHeightCube.Report();
                    // Each tenant gets the index of its OWN realization -- the scheduler then
                    // prefers loads that are a read over loads that are a paint.
                    resMgr.SetTileIndex(colorCubeT, &idxColorCube);
                    resMgr.SetTileIndex(winTenant, &idxColorWin);
                    resMgr.SetTileIndex(detTenant, &idxColorDet);
                    resMgr.SetTileIndex(hgtTenant, &idxHeightCube);
                    // M9ag: the NVMe -> GPU reader. Created once; a machine without the
                    // redist or with a driver that declines keeps the ReadFile path.
                    static TileStream tileStream;
                    // M9ao: ON by default. Through the packed trees the streamed path is
                    // pixel-identical to the upload ring (section 33); the user made it the
                    // default. --no-direct-storage is the A/B.
                    if (opt.directStorage) {
                        if (tileStream.Init(gpu)) resMgr.SetTileStream(&tileStream);
                    } else {
                        Log("[dstorage] OFF by request (--no-direct-storage): every tile takes "
                            "the upload ring");
                    }
                }

            }
            globe->stencilOverlay = opt.stencil;
            globe->debugLens = opt.lens;
            globe->probeCullFar = opt.probeCullFar;
            // M9h: the grad(flow) bank plus the grid it lives on, for --lens velgrad. The
            // SWE solver owns the bank; the bathy model owns the world mapping.
            if (swe.Ready() && bathy.Ready()) {
                // The residency map and the bank's texel size ride along: the lens picks the
                // level its footprint wants and the map clamps it to what has arrived.
                // ---- M9i: CLOSE THE LOOP. The regional current becomes a GA object through
                // the plugin seam, composes into a page, and lands in the bank's COARSE levels.
                // Nothing here reaches into another layer: a loader answers where/what/where-not,
                // a compositor answers value-and-weight, and the bank takes texels. The solver
                // never learns that GoMOFS exists, which is exactly what the backed-out version
                // got wrong.
                //
                // grad() is taken on the composed page rather than on the source, because
                // divergence and vorticity are properties of the COMPOSITE -- taking them
                // per-source and blending afterwards would average two different derivatives.
                if (swe.Ready() && bathy.Ready()) {
                    LoaderRegistry freg;
                    freg.Register("json", CurrentFieldLoader::Open);
                    if (auto cld = freg.Open("data/currents/currents.json")) {
                        auto ras = std::make_shared<RasterSource>(std::move(cld), 0);
                        DomainCompositor fc;
                        LevelLadder lad;
                        lad.level0MetersPerTexel = swe.CellM();
                        fc.SetLadder(lad);
                        fc.Add(ras);

                        GradeBank& vb = swe.VelGradBank();
                        uint32_t wrote = 0;
                        // Levels coarse enough that a ~700 m model is honest there.
                        for (uint32_t lvl = 3; lvl < vb.MipCount(); ++lvl) {
                            const uint32_t w = (std::max)(1u, swe.Nx() >> lvl);
                            const uint32_t h = (std::max)(1u, swe.Ny() >> lvl);
                            // The page's own lat/lon extent, EXACTLY -- not the anchor-linear
                            // form, which is declared valid only near its anchor.
                            DomainCompositor::PageGeo geo;
                            const double mpt = swe.CellM() * double(1u << lvl);
                            geo.lon0 = BathyModel::kOrgLon +
                                       (bathy.WorldX0() + 0.5 * mpt) / BathyModel::kMPerLon;
                            geo.lat0 = BathyModel::kOrgLat +
                                       (bathy.WorldZ0() + 0.5 * mpt) / BathyModel::kMPerLat;
                            geo.dLon = mpt / BathyModel::kMPerLon;
                            geo.dLat = mpt / BathyModel::kMPerLat;

                            std::vector<float> uv, cov;
                            const uint32_t covered =
                                fc.ComposePage(PageAddr{lvl, 0, 0}, geo, w, h, 2, uv, cov);
                            if (!covered) continue;

                            // grad(flow) on the composed page: divergence to grade 0, vorticity
                            // to grade 2, exactly what the type says kG1 * kG1 produces. Coverage
                            // rides in .z so the shader composite can weight it, and a texel with
                            // no current stays absent rather than reading as still water.
                            std::vector<float> rgba(size_t(w) * h * 4, 0.0f);
                            const double dxm = mpt, dym = mpt;
                            for (uint32_t y = 0; y < h; ++y) {
                                for (uint32_t x = 0; x < w; ++x) {
                                    const size_t i = size_t(y) * w + x;
                                    if (cov[i] <= 0.0f) continue;
                                    const uint32_t xm = (x > 0) ? x - 1 : x;
                                    const uint32_t xp = (x + 1 < w) ? x + 1 : x;
                                    const uint32_t ym = (y > 0) ? y - 1 : y;
                                    const uint32_t yp = (y + 1 < h) ? y + 1 : y;
                                    const size_t a = (size_t(y) * w + xm) * 2;
                                    const size_t b = (size_t(y) * w + xp) * 2;
                                    const size_t c = (size_t(ym) * w + x) * 2;
                                    const size_t d = (size_t(yp) * w + x) * 2;
                                    const float dudx = (uv[b + 0] - uv[a + 0]) / float(2 * dxm);
                                    const float dvdx = (uv[b + 1] - uv[a + 1]) / float(2 * dxm);
                                    const float dudy = (uv[d + 0] - uv[c + 0]) / float(2 * dym);
                                    const float dvdy = (uv[d + 1] - uv[c + 1]) / float(2 * dym);
                                    rgba[i * 4 + 0] = dudx + dvdy;   // grade 0: divergence
                                    rgba[i * 4 + 1] = dvdx - dudy;   // grade 2: vorticity
                                    rgba[i * 4 + 2] = cov[i];        // coverage
                                }
                            }
                            vb.UploadLevel(gpu, lvl, rgba.data(), w * 4 * sizeof(float), w, h);
                            ++wrote;
                        }
                        Log("[compose] grad(GoMOFS) -> swe.velgrad slice 0 levels 3..%u: %u "
                            "composed (loader -> source -> page -> bank)",
                            vb.MipCount() - 1, wrote);

                        // ---- SLICE 1: THE REGION. Same sources, same compositor, a page over
                        // the REGION's own ground instead of the window's. This is what makes
                        // the wind fallback unnecessary: outside the solve there is now a page
                        // of the SAME quantity in the SAME units, so the consumer composites
                        // two pages of one bank instead of blending in a foreign field.
                        if (vb.Slices() > 1) {
                            // Every level, not just the floor: an upload into an unmapped tile
                            // is discarded and the page would simply not be there.
                            vb.MapAllLevels(gpu, 1);
                            const GeoRef& rg = ras->Ref();
                            uint32_t rwrote = 0;
                            // Start where the bank's texels approach the source's own scale.
                            // Differentiating a field upsampled 8x invents structure that is
                            // not in it -- the derivative of the interpolation, not of the
                            // current -- so the region owns only levels it can honestly fill,
                            // and the residency clamp keeps a sampler from asking for finer.
                            for (uint32_t lvl = 3; lvl < vb.MipCount(); ++lvl) {
                                const uint32_t w = (std::max)(1u, swe.Nx() >> lvl);
                                const uint32_t h = (std::max)(1u, swe.Ny() >> lvl);
                                // The page spans the SOURCE's extent, sampled onto this
                                // level's grid -- the region gets its own geography, not the
                                // window's stretched over it.
                                DomainCompositor::PageGeo rgeo;
                                rgeo.dLon = rg.scaleX * double(rg.width) / double(w);
                                rgeo.dLat = rg.scaleY * double(rg.height) / double(h);
                                rgeo.lon0 = rg.originX + 0.5 * rgeo.dLon;
                                rgeo.lat0 = rg.originY + 0.5 * rgeo.dLat;

                                std::vector<float> ruv, rcov;
                                const uint32_t rc =
                                    fc.ComposePage(PageAddr{lvl, 0, 0}, rgeo, w, h, 2, ruv, rcov);
                                if (!rc) continue;
                                std::vector<float> rr(size_t(w) * h * 4, 0.0f);
                                // Metres per texel of THIS page: the region's degrees converted
                                // at its own latitude, not the window's frozen anchor.
                                const double mx = std::abs(rgeo.dLon) * 111319.49 *
                                                  std::cos(rgeo.lat0 * 3.14159265 / 180.0);
                                const double my = std::abs(rgeo.dLat) * 110574.0;
                                for (uint32_t y = 0; y < h; ++y) {
                                    for (uint32_t x = 0; x < w; ++x) {
                                        const size_t i = size_t(y) * w + x;
                                        if (rcov[i] <= 0.0f) continue;
                                        const uint32_t xm = (x > 0) ? x - 1 : x;
                                        const uint32_t xp = (x + 1 < w) ? x + 1 : x;
                                        const uint32_t ym = (y > 0) ? y - 1 : y;
                                        const uint32_t yp = (y + 1 < h) ? y + 1 : y;
                                        const size_t a = (size_t(y) * w + xm) * 2;
                                        const size_t b = (size_t(y) * w + xp) * 2;
                                        const size_t c2 = (size_t(ym) * w + x) * 2;
                                        const size_t d2 = (size_t(yp) * w + x) * 2;
                                        const float dudx = float((ruv[b + 0] - ruv[a + 0]) / (2 * mx));
                                        const float dvdx = float((ruv[b + 1] - ruv[a + 1]) / (2 * mx));
                                        const float dudy = float((ruv[d2 + 0] - ruv[c2 + 0]) / (2 * my));
                                        const float dvdy = float((ruv[d2 + 1] - ruv[c2 + 1]) / (2 * my));
                                        rr[i * 4 + 0] = dudx + dvdy;
                                        rr[i * 4 + 1] = dvdx - dudy;
                                        rr[i * 4 + 2] = rcov[i];
                                    }
                                }
                                vb.UploadLevel(gpu, lvl, rr.data(), w * 4 * sizeof(float), w, h,
                                               1);
                                ++rwrote;
                            }
                            // The lens needs the region page's geography to sample it.
                            globe->SetVelGradRegion(
                                rg.originX, rg.originY, rg.scaleX * double(rg.width),
                                rg.scaleY * double(rg.height));
                            Log("[compose] grad(GoMOFS) -> swe.velgrad slice 1 (REGION): %u "
                                "levels over %.2f x %.2f deg -- the wind fallback is now "
                                "unnecessary",
                                rwrote, std::abs(rg.scaleX) * rg.width,
                                std::abs(rg.scaleY) * rg.height);
                        }
                    }
                }
                globe->SetVelGradLens(
                    swe.VelGradSrv(), bathy.WorldX0(), bathy.WorldZ0(), bathy.WorldSizeX(),
                    bathy.WorldSizeZ(), swe.VelGradResMapSrv(),
                    static_cast<float>(bathy.WorldSizeX() / (std::max)(1u, swe.Nx())),
                    static_cast<float>(swe.VelGradResMapW()),
                    static_cast<float>(swe.VelGradResMapH()), swe.VelGradMips());
            }
            globe->sliceOn = opt.sliceOn;
            globe->sliceD = static_cast<float>(opt.sliceD);
            compositor.LogRegistry();
            // M8j: --fidelity-map draws that same registry. It runs HERE, not at the
            // --water-map exit, because earth.color is registered 200 lines later than
            // earth.height -- the first cut rendered a sheet with the skin channel simply
            // missing, which is the exact class of error this picture exists to catch.
            if (!opt.fidelityMap.empty()) {
                VectorPack fidVec;
                fidVec.Load("data/vectors/vectors.vpack");
                RenderFidelityMap(compositor, fidVec, opt.fidelityMap);
                // No early return: the flag forces headless + 1 frame, so the run falls
                // through to the NORMAL teardown. Tearing down by hand from this deep in
                // the wiring exits 9 -- the layers and residency tenants are live here,
                // unlike at the --water-map exit which runs before any of them exist.
            }
            // M7j: the GA AST -- the state diagram printed and validated EVERY run, so a
            // frame mismatch or an orphaned field is a boot-time report, not a debugging
            // session. (The workflow as an AST: domains, axes, units, scales, ranges.)
            ga::ast::RegisterKnownWaterEdges();
            ga::ast::SetActive("sea.ps", !opt.oneWater);
            // M9bh: --pixel-water re-opens eight edges into the pixel stage (the two rays
            // and what they read). Declared only when the flag is on -- an edge for a mode
            // the run is not in is graph rot wearing the other sign.
            if (opt.pixelWater) ga::ast::RegisterPixelWaterEdges();
            // M9bi: the sun's own edges, when the ephemeris is the one driving it.
            if (!opt.sunPinned) ga::ast::RegisterSolarEdges();
            if (opt.sliceOn) {
                // M7o: the demo node registers its edge like any other -- the AST is how
                // features arrive now. One blade, one inner product, one discard.
                ga::ast::Register({"user.plane", "globe.ps", "slice",
                                   {"world.m", true, 0, 0, 0}, {"world.m", true, 0, 0, 0},
                                   false, "signed distance m", "keep s<=0", 1.0,
                                   "Globe.hlsl slice discard (gatest: sandwich negates s; "
                                   "proofs/slice_plane.py)"});
            }
            ga::ast::Print();
            ga::ast::Validate();
            ga::ast::WriteMarkdown("docs/GA_AST.md");   // the scriptorium indexes this
            ga::ast::WriteJson("docs/ga_ast.json");     // the Blueprint contract (M7u)
            // The survey pack loads whenever it exists: the land MASKS are the default
            // classifier (always on); the VECTOR overlay draws only under --stencil.
            if (!marsMode && gisStencil.Load("data/gis/gis.json")) {
                // M9ay: no raster is built here any more; the classifier reads the mask pages.
                vectors.Load("data/vectors/vectors.vpack");
                auto gisOwned = std::make_unique<GisLayer>();
                gisLayer = gisOwned.get();
                gisLayer->Configure(opt.shaderDir, &gisStencil, &exchange, &vectors);
                gisLayer->Init(gpu, renderer.Shaders(), fields, renderer.RootSignature());
                gisLayer->enabled = opt.stencil;
                renderer.AddLayer(std::move(gisOwned));
            }
            if (!marsMode) {
                auto mkOwned = std::make_unique<MarkerLayer>();
                mkOwned->Configure(opt.shaderDir, &exchange, "markers.stations");
                mkOwned->Init(gpu, renderer.Shaders(), fields, renderer.RootSignature());
                mkOwned->enabled = !opt.albedo;   // the lens shows textures, nothing else
                renderer.AddLayer(std::move(mkOwned));
            }
            globe->SetResidency(&resMgr, surf, norm, marsMode);
            globe->SetPlanetRadius(planetR);

            // Field adapters: the whole tiled economy reports on one stats line.
            if (swe.Ready()) {
                resMgr.RegisterField("swe", [&swe] { return swe.ResidentBytes(); },
                                     [&swe] { return swe.ResidentTiles(); });
            }
            if (sea) {
                resMgr.RegisterField("churn", [sea] { return sea->ChurnBytes(); },
                                     [sea] { return sea->ChurnTiles(); });
                resMgr.RegisterField("air", [globe] { return globe->AirBytes(); },
                                     [globe] { return globe->AirTiles(); });
            }

            // The grade-signature registry's first demand derivation: wind published as a
            // Cl(2) field (vector where it blows, bivector where it curls), and the Cayley
            // closure says WHERE any wind-product field (enstrophy, OW, advection terms) can
            // be non-zero -- residency for derived tenants decided by algebra, no data read.
            if (!marsMode && globeModel.WindNx() > 0) {
                const int tx = 6, ty = 6;
                std::vector<uint8_t> sig(tx * ty, 0);
                const int nx = globeModel.WindNx(), ny = globeModel.WindNy();
                for (int y = 0; y < ny; ++y) {
                    for (int x = 0; x < nx; ++x) {
                        const float u = globeModel.WindU()[y * nx + x];
                        const float v = globeModel.WindV()[y * nx + x];
                        const int i = (y * ty / ny) * tx + (x * tx / nx);
                        if (u * u + v * v > 36.0f) sig[i] |= 0b010;          // grade 1: wind
                        const int xr = (std::min)(nx - 1, x + 1);
                        const int yd = (std::min)(ny - 1, y + 1);
                        const float curl = (globeModel.WindV()[y * nx + xr] - v) -
                                           (globeModel.WindU()[yd * nx + x] - u);
                        if (std::abs(curl) > 2.5f) sig[i] |= 0b100;          // grade 2: curl
                    }
                }
                resMgr.PublishSignatures("wind10m", tx, ty, sig);
                std::vector<uint8_t> derived;
                uint32_t dx = 0, dy = 0;
                if (resMgr.DeriveDemand("wind10m", "wind10m", derived, dx, dy)) {
                    int need = 0;
                    for (uint8_t s : derived) {
                        if (s) ++need;
                    }
                    Log("[residency] Cayley-derived demand for wind-product fields: %d/%d "
                        "tiles (the rest are ALGEBRAICALLY zero -- never allocated, never "
                        "dispatched)",
                        need, dx * dy);
                    // M9h: and now it DRIVES. Until this call the closure was a report; the
                    // bank's residency came from a CPU curl scan alone.
                    if (globe) globe->ApplyWindDemand(gpu, derived, dx, dy);
                }
            }
        }

        // M6g: THREE views, not four -- 0 chart, 1 THE WORLD (estuary and planet, one
        // continuous scene), 2 gulf map. --sea and --globe both open the world; they differ
        // only in the starting camera. Per-frame altitude gates refine world-mode layer
        // enables continuously (sky hands to the limb shell, the FFT sea sheds at height).
        int mode = ((opt.globeStart || opt.seaStart) && (sea || globe)) ? 1
                 : (opt.gulfStart && gulf)                              ? 2 : 0;
        auto applyMode = [&](int m) {
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
        const double lenM = model.TotalRiverKm() * TideLayer::kKmToSceneM;
        Camera cam;
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
        Camera camSea;
        if (bathy.Ready()) camSea.SetFromCompass(522.0, 7.0, 72.0, 246.0f, -4.0f);
        else camSea.SetFromCompass(0.0, 12.0, 0.0, 246.0f, -5.0f);
        camSea.fovY = cam.fovY;
        camSea.speed = 30.0f;
        // Globe mode: the planet frame (centre at the origin). Start over the North Atlantic
        // with home in view.
        Camera camGlobe;
        {
            const double gLat = (opt.gcamLat < 1e8f) ? opt.gcamLat : 34.0;
            const double gLon = (opt.gcamLat < 1e8f) ? opt.gcamLon : -52.0;
            const double gR = planetR +
                              ((opt.gcamLat < 1e8f) ? opt.gcamAltKm * 1000.0
                                                    : planetR * 2.1);
            double d[3];
            GlobeModel::LatLonDir(gLat, gLon, d);
            camGlobe.px = d[0] * gR;
            camGlobe.py = d[1] * gR;
            camGlobe.pz = d[2] * gR;
            camGlobe.LookAt(0.0, 0.0, 0.0);
            camGlobe.fovY = cam.fovY;
            camGlobe.speed = 800000.0f;
        }
        Camera camChart = cam;
        if (mode == 1) cam = camSea;   // camGlobe start applies below, after frame conversion

        // ---- M6g: ONE WORLD, ONE FRAME. Everything renders in the estuary's tangent frame
        // (the planet's centre sits at flat (0, -R, 0)); the globe rotates into it, the
        // estuary layers were always in it, and the old mode-switch handoff -- the last
        // smoke-and-mirror in the engine -- is DELETED. The pose maps below survive only to
        // express orbit keyframes and cull volumes. (Mars anchors its frame at (0N, 0E).)
        double oDir[3], east0[3], north0[3];
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
        auto planetToFlatPose = [&](const Camera& g) -> Camera {
            Camera f = g;
            const double p[3] = {g.px, g.py, g.pz};
            f.px = p[0] * east0[0] + p[1] * east0[1] + p[2] * east0[2];
            f.py = p[0] * oDir[0] + p[1] * oDir[1] + p[2] * oDir[2] - planetR;
            f.pz = p[0] * north0[0] + p[1] * north0[1] + p[2] * north0[2];
            const DirectX::XMFLOAT3 ff = g.Forward();
            const double d[3] = {ff.x, ff.y, ff.z};
            const double fx = d[0] * east0[0] + d[1] * east0[1] + d[2] * east0[2];
            const double fy = d[0] * oDir[0] + d[1] * oDir[1] + d[2] * oDir[2];
            const double fz = d[0] * north0[0] + d[1] * north0[1] + d[2] * north0[2];
            f.yaw = static_cast<float>(std::atan2(fz, fx));
            const float lim = 3.14159265f / 2.0f - 0.0017f;
            f.pitch = std::clamp(
                static_cast<float>(std::atan2(fy, std::sqrt(fx * fx + fz * fz))), -lim, lim);
            return f;
        };
        auto flatToPlanetPose = [&](const Camera& f) -> Camera {
            Camera g = f;
            const double r = planetR + f.py;
            g.px = oDir[0] * r + east0[0] * f.px + north0[0] * f.pz;
            g.py = oDir[1] * r + east0[1] * f.px + north0[1] * f.pz;
            g.pz = oDir[2] * r + east0[2] * f.px + north0[2] * f.pz;
            const DirectX::XMFLOAT3 ff = f.Forward();
            const double d[3] = {ff.x, ff.y, ff.z};
            const double gx = east0[0] * d[0] + oDir[0] * d[1] + north0[0] * d[2];
            const double gy = east0[1] * d[0] + oDir[1] * d[1] + north0[1] * d[2];
            const double gz = east0[2] * d[0] + oDir[2] * d[1] + north0[2] * d[2];
            g.yaw = static_cast<float>(std::atan2(gz, gx));
            const float lim = 3.14159265f / 2.0f - 0.0017f;
            g.pitch = std::clamp(
                static_cast<float>(std::atan2(gy, std::sqrt(gx * gx + gz * gz))), -lim, lim);
            return g;
        };
        // The camera bookmarks live in the ONE frame now: convert the orbit start pose, and
        // hand the globe its frame + the CUDEM window (for the foundation sink).
        camGlobe = planetToFlatPose(camGlobe);
        if (mode == 1 && opt.globeStart) cam = camGlobe;
        if (globe) {
            globe->SetFrame(east0, oDir, north0);
            if (bathy.Ready()) {
                const double lon0 = BathyModel::kOrgLon + bathy.WorldX0() / BathyModel::kMPerLon;
                const double lat1 = BathyModel::kOrgLat +
                                    (bathy.WorldZ0() + bathy.WorldSizeZ()) / BathyModel::kMPerLat;
                globe->SetEstuaryWindow(lon0, lat1, bathy.WorldSizeX() / BathyModel::kMPerLon,
                                        bathy.WorldSizeZ() / BathyModel::kMPerLat);
            }
        }
        // M6i: the terrain samples the SAME composed color the globe does -- one fill
        // function, one frame, one answer (its geometry stays the CUDEM grid the physics
        // reads, so its height channel is off).
        if (!marsMode && globe) {
            ComposedSurfaceCb cs{};
            // M9ap: pages mode -- the terrain, the sea and the GIS layer sample the SAME page
            // tenant the globe does, slices 6 and 7 included.
            const double det17Org[2] = {det17OrgX, det17OrgY};
            const bool pagesMode = colorCubeT >= 0 && winTenant == colorCubeT;
            FillComposedCb(cs, &resMgr, colorCubeT, winTenant, hgtTenant, hgtWinTenant,
                           winOrgX, winOrgY, 16384.0, 14, planetR, east0, oDir, north0,
                           opt.stencil, maskTenant,
                           pagesMode ? detTenant : -1, pagesMode ? det17Org : nullptr, 17,
                           pagesMode ? 6u : UINT32_MAX,
                           pagesMode ? 7u : UINT32_MAX,
                           (hgtTenant >= 0 && hgtWinTenant == hgtTenant) ? 6u : UINT32_MAX);
            if (terrain) terrain->SetComposed(cs);
            if (sea) sea->SetComposed(cs);
            if (gisLayer) gisLayer->SetComposed(cs);
        }
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
        auto altOf = [&](const Camera& c) {
            const double y = c.py + planetR;
            return std::sqrt(c.px * c.px + y * y + c.pz * c.pz) - planetR;
        };

        // ---- M6b: the camera ON RAILS -- the debug flight, as GA. Each keyframe pose is a
        // MOTOR (position and aim as one element); segments interpolate with the screw Slerp
        // (M0 Exp(u Log(~M0 M1))), so the descent from orbit is one smooth helical motion per
        // leg, eased at the ends. Extraction back to yaw/pitch drops any interpolated roll --
        // the horizon stays level, Google-Earth style.
        auto poseMotor = [&](const Camera& c) -> Motor {
            const double org[3] = {0, 0, 0};
            const double yAxis[3] = {0, 1, 0};
            const double rAxis[3] = {std::sin(c.yaw), 0.0, -std::cos(c.yaw)};
            return Motor::Translation(c.px, c.py, c.pz) *
                   Motor::Rotation(org, rAxis, -c.pitch) * Motor::Rotation(org, yAxis, -c.yaw);
        };
        auto motorPose = [&](const Motor& m, Camera& c) {
            double px = 0, py = 0, pz = 0;
            m.TransformPoint(px, py, pz);
            double fx = 1, fy = 0, fz = 0;
            m.TransformDir(fx, fy, fz);
            c.px = px;
            c.py = py;
            c.pz = pz;
            c.yaw = static_cast<float>(std::atan2(fz, fx));
            const float lim = 3.14159265f / 2.0f - 0.0017f;
            c.pitch = std::clamp(
                static_cast<float>(std::atan2(fy, std::sqrt(fx * fx + fz * fz))), -lim, lim);
        };
        // Keys (all in the PLANET frame; flat poses go through flatToPlanetPose):
        //   0-5 s   orbit -> 2.6 km over the estuary (via a 500 km mid key: no screw dives)
        //   5-10 s  hold over the river (the handoff has already switched to the estuary)
        //  10-15 s  descend to the north-jetty helm
        //  15-25 s  hold the helm while the real-time sea runs
        std::vector<std::pair<double, Motor>> railKeys;
        // A pose on any planet: stand at (lat, lon, alt), aim at a surface target.
        auto orbPose = [&](double lat, double lon, double altM, double tLat, double tLon) {
            Camera c;
            double d[3];
            GlobeModel::LatLonDir(lat, lon, d);
            const double rr = planetR + altM;
            c.px = d[0] * rr;
            c.py = d[1] * rr;
            c.pz = d[2] * rr;
            double t[3];
            GlobeModel::LatLonDir(tLat, tLon, t);
            c.LookAt(t[0] * planetR, t[1] * planetR, t[2] * planetR);
            return c;
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
        auto railPose = [&](double t, Camera& out) {
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
        if (opt.camAlt > 0) {
            const double cx = (opt.camX < 1e8f) ? opt.camX : 0.0;
            const double cz = (opt.camZ < 1e8f) ? opt.camZ : 0.0;
            cam.SetFromCompass(cx, opt.camAlt, cz, opt.camAz, opt.camPitch);
        }

        double simUnix = (opt.startUnix > 0) ? opt.startUnix : NowUnix();
        // The playable clock (sim/SimClock.h). Headless keeps its own frame-indexed formula
        // below -- that path was already fixed-step, which is why rails reproduce and sessions
        // did not.
        SimClock simClock;
        simClock.Reset(simUnix);
        bool sunLogged = false;   // M9bi: log the placed sun once, with its numbers
        const double startUnix = simUnix;

        // M5c boundary clocks. Ocean = the ENTRANCE station (the physically right open-water
        // level; Newburyport stays the chart focus). West = the river tide interpolated to the
        // window's west edge (~km 6, between Newburyport at 4.4 and Salisbury Point at 8.2),
        // expressed as a DEVIATION from the ocean tide so the datum offset cancels.
        int entSta = model.Focus(), westA = model.Focus(), westB = model.Focus();
        // The west edge's along-channel kilometre tracks the WINDOW (straight-line distance is
        // a fine proxy on this reach): ~5.3 km for the original mouth window, ~15.5 for the
        // M6d wide window (bracketed by Merrimacport/Riverside instead of Newburyport/Salisbury).
        const double kWestKm =
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
        const double wT = (model.S(westB).riverKm > model.S(westA).riverKm)
                              ? (kWestKm - model.S(westA).riverKm) /
                                    (model.S(westB).riverKm - model.S(westA).riverKm)
                              : 0.0;
        auto oceanAt = [&](double t) { return model.Height(entSta, t) + datumOff; };
        // The sound's tide: the entrance clock ~10 min later (its Ipswich mouth is a few km
        // down an open coast). Active only when the window holds the sound.
        const bool hasSound = false;   // retired with the solver's south strip (see SweSolver)
        auto southAt = [&, hasSound](double t) {
            return hasSound ? model.Height(entSta, t - 600.0) - model.Height(entSta, t) : 0.0;
        };
        // TIDAL parts only: each station's Height is in its OWN local MLLW, so raw differences
        // smuggle a constant datum offset into the boundary (with the Merrimacport bracket that
        // was a permanent 19 cm seaward slope -- an artificial ever-ebb). The true NAVD river
        // slope at this reach is cm-scale; call it zero and let the tide be the signal.
        auto westAt = [&](double t) {
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
        const double kUpriverAreaM2 = 3.9e6;
        auto westQAt = [&, riverQ](double t) {
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
        WeatherManager weather;
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
                                          resMgr.Mips(hgtTenant), 1263360.0, 1538048.0);
                }
            }
        }

        // --ocean-probe lat,lon: the manager's verification harness. Standing at the point
        // below the activation altitude IS the zoom -- dormant windows containing it spin up
        // through the same rule the camera uses; then the sample prints at three rungs with
        // full provenance, plus consistency gates against the station truth.
        if (!opt.oceanProbe.empty()) {
            double plat = 0, plon = 0;
            if (sscanf_s(opt.oceanProbe.c_str(), "%lf,%lf", &plat, &plon) == 2) {
                // The external merrimack window needs its history before it can be mirrored
                // (interactive runs spin it up right after this block).
                Log("[wx] probe: spinning the merrimack window");
                if (swe.Ready()) swe.Spinup(gpu, simUnix, 0.5, oceanAt, westAt, southAt,
                                            westQAt);
                Log("[wx] probe: manager update at the probe point");
                weather.Update(gpu, renderer.Shaders(), opt.shaderDir, simUnix, plat, plon,
                               1000.0);
                // The probe reads the windows' CPU mirrors: fill them once, at its instant.
                weather.RefreshMirrorsTo(gpu, simUnix);
                auto show = [&](double res) {
                    const WeatherSample ws = weather.Query(plat, plon, simUnix, res);
                    Log("[wx] res %6.0f m: level %+6.2f  cur %+5.2f,%+5.2f  Hs %4.2f Tp %4.1f "
                        "dir %3.0f  wind %+5.1f,%+5.1f  bed %+7.1f  depth %6.1f",
                        res, ws.levelNavd, ws.u, ws.v, ws.hs, ws.tp, ws.dirDeg, ws.windU,
                        ws.windV, ws.bedNavd, ws.depthM);
                    Log("[wx]   level=%s | current=%s | waves=%s | wind=%s | bed=%s",
                        ws.levelSrc, ws.currentSrc, ws.waveSrc, ws.windSrc, ws.bedSrc);
                };
                Log("[wx] probe (%.4f, %.4f) t=now -- three rungs:", plat, plon);
                show(20000.0);
                show(500.0);
                show(15.0);
                // Consistency gates: the manager's level vs the fitted station truth.
                bool ok = true;
                for (const char* sid : {"8443970", "8441841"}) {
                    for (size_t i = 0; i < model.Count(); ++i) {
                        if (model.S(i).id != sid) continue;
                        const TideStation& st = model.S(i);
                        if (st.mllwMinusNavdM < -900.0) continue;
                        const double direct = model.Height(i, simUnix) + st.mllwMinusNavdM;
                        const WeatherSample q = weather.Query(st.lat, st.lon, simUnix, 200.0);
                        const double err = std::abs(q.levelNavd - direct);
                        Log("[wx] gate %s (%s): manager %+.3f vs station %+.3f -- err %.0f mm "
                            "(%s)",
                            sid, st.name.c_str(), q.levelNavd, direct, err * 1000.0,
                            q.levelSrc);
                        if (err > 0.25) ok = false;
                    }
                }
                Log("[wx] ---- %s: %d windows active ----", ok ? "PASS" : "FAIL",
                    weather.ActiveWindows());
                gpu.WaitIdle();
                resMgr.Shutdown();
                gpu.Shutdown();
                return ok ? 0 : 1;
            }
        }

        // M8: THE SOLVED WAVE FIELD (ALGEBRA.md wavefield) -- the stationary wave BVP
        // solved per cell over the inlet window on a worker thread, cached by content
        // identity (solver version + buckets + spectrum + height-stack signature), and
        // blended into the bank kernel inside its feathered window. NOAA carries the
        // state, the solve carries the structure, the GPU carries the phase.
        // The water scene is DATA (data/wave_scene.json, authored if absent, hot-reloaded
        // per frame): move the solved window, retune closures, save -- no recompile.
        auto sceneToWaveCfg = [](const WaterSceneConfig& s) {
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
            return c;
        };
        std::unique_ptr<WaveField> waveField;
        int wfCtSta = -1;
        // M9bc: THE WAVE FIELD AS A TREE NODE. The solver's grid is aligned to the z16 page
        // (WaveFieldSource::Align), the node paints planes as the frame's faces, the tree caches
        // them, and a page tenant serves them to the bank. A bucket roll re-keys the tree and the
        // window's whole pyramid is prefilled before the tenant is told.
        std::shared_ptr<WaveFieldSource> waveSrc;
        std::shared_ptr<std::shared_ptr<TileTree>> waveTree;
        WaveFieldSource::Frame waveFrame;
        int waveT = -1;
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
            if (waterScene.wfEnabled) {
                waveSrc = std::make_shared<WaveFieldSource>(waveField.get(), waveFrame);
                waveTree = std::make_shared<std::shared_ptr<TileTree>>(
                    std::make_shared<TileTree>(waveSrc.get(), TileTree::Fmt::Raw4));
                auto holder = waveTree;
                const ColorFrame wframe = waveFrame.color;
                TileProviderFn wp = [holder, wframe](const TileRequest& r,
                                                     std::vector<uint8_t>& out, TileLoc* loc) {
                    if (r.face < 6u) {
                        out.assign(65536, 0);   // the cube faces are not this node's frame
                        if (loc) *loc = TileLoc{};
                        return true;
                    }
                    std::shared_ptr<TileTree> t = std::atomic_load(holder.get());
                    if (!t) return false;
                    TileRequest w = r;
                    w.face = r.face - 6u;   // the plane
                    return t->Provider(wframe)(w, out, loc);
                };
                waveT = resMgr.AddTexturePages(gpu, L"wave.field (pages)", Compositor::kFaceDim,
                                               DXGI_FORMAT_R8G8B8A8_UNORM, std::move(wp),
                                               6u + uint32_t(WaveField::kMaxComp) + 1u);
                waterBank->SetWavePages(resMgr.TextureSrv(waveT), resMgr.ResidencySrv(waveT),
                                        double(waveFrame.orgPxX), double(waveFrame.orgPxY),
                                        waveFrame.nx, waveFrame.ny);
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
        Route route;
        if (waterBank) route.Load("data/gis/route_merrimack.json");

        // M5c: give the solver history before the first frame, and run the validation cycle if
        // asked (headless CSV; the ebb/flood-asymmetry and basin-lag gates read from it).
        if (swe.Ready()) {
            if (opt.sweCycleH > 0) {
                swe.Spinup(gpu, simUnix, 2.0, oceanAt, westAt, southAt, westQAt);
                const int ctSta = haveCurrents ? currents.StationIndex("ACT0816") : -1;
                RunSweCycle(gpu, swe, oceanAt, westAt, southAt, westQAt,
                            haveCurrents ? &currents : nullptr, ctSta, bathy, simUnix,
                            opt.sweCycleH);
                gpu.WaitIdle();
                resMgr.Shutdown();
                gpu.Shutdown();
                return 0;
            }
            if (opt.sweSpinupH > 0) {
                const auto t0 = std::chrono::steady_clock::now();
                swe.Spinup(gpu, simUnix, opt.sweSpinupH, oceanAt, westAt, southAt, westQAt);
                Log("[swe] spun up %.2f h of history in %.1f s", opt.sweSpinupH,
                    std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count());
            }
            if (!opt.sweUvDump.empty()) DumpSweUv(gpu, swe, bathy, opt.sweUvDump);
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
        double timeScale = opt.timeScale;
        double windowSec = opt.windowDays * 86400.0;
        bool paused = false;

        // ---- Google-Earth camera gestures (PGA motors, core/Pga.h). LMB grabs the ground:
        // plain drag pans (the grabbed point stays under the cursor), SHIFT tilts and ALT
        // rotates -- each ONE motor rotation about a line through the ground pivot frozen at
        // the moment of grab. Wheel zooms toward the point under the cursor (CTRL+wheel keeps
        // the old fly-speed dial).
        int dragMode = 0;          // 0 none, 1 pan-grab, 2 tilt (SHIFT), 3 rotate (ALT)
        bool lmbWas = false;
        double dragPivot[3] = {};
        double lastWaterNavd = 0;  // last frame's level; the pivot ray tests against it

        auto groundAt = [&](double x, double z) -> double {
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
        auto pixelRay = [&](float sxPx, float syPx, double d[3]) {
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
        auto pickGround = [&](float sxPx, float syPx, double out[3]) -> bool {
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
        auto pickGlobe = [&](float sxPx, float syPx, double out[3]) -> bool {
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
        auto pickAny = [&](float sx, float sy, double out[3]) {
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

        // M6f/M6i: pre-warm the composed pyramids -- the Merrimack window coarse-to-z14 in
        // rings, plus the planet-wide height cube at a working mip (height paints are LOCAL:
        // no network, just source reads). Everything lands in the composed forever-cache:
        // warming is a once-per-machine cost (re-runs drain instantly from disk).
        if (opt.warmInlet && (winTenant >= 0 || hgtTenant >= 0)) {
            struct WarmRing {
                float a, b;
                uint32_t mip;
            };
            // Mip 2 covers the WHOLE window (one zoom level = one color grading across the
            // view -- the patchwork of per-zoom gradings was half of the "uneven shading"
            // report); deeper rings tighten on the inlet.
            const WarmRing rings[] = {
                {0.00f, 1.00f, 3}, {0.00f, 1.00f, 2}, {0.38f, 0.62f, 1}, {0.44f, 0.56f, 0}};
            if (winTenant >= 0) {
                for (const auto& w : rings) {
                    resMgr.Want(winTenant, 0, w.mip, w.a, w.a, w.b, w.b);
                }
            }
            if (hgtWinTenant >= 0) {
                // Height paints are pure local math: warm the WHOLE window at mip 2 (~32 MB)
                // so land/sea classification is never a coarse-mip smear anywhere in view.
                const uint32_t hwf = (hgtWinTenant == hgtTenant) ? 6u : 0u;   // M9aq slice
                resMgr.Want(hgtWinTenant, hwf, 2, 0, 0, 1, 1);
                for (const auto& w : rings) {
                    resMgr.Want(hgtWinTenant, hwf, w.mip, w.a, w.a, w.b, w.b);
                }
            }
            if (hgtTenant >= 0) {
                for (uint32_t f = 0; f < 6; ++f) resMgr.Want(hgtTenant, f, 4, 0, 0, 1, 1);
            }
            Log("[warm] pre-caching composed pyramids (%u fetch budget; composed tiles land "
                "in cache/composed forever)",
                opt.tileBudget);
            for (int it = 0; it < 12000 && resMgr.PendingCount() > 0; ++it) {
                ID3D12GraphicsCommandList* cl = gpu.BeginUpload();
                resMgr.ProcessQueues(gpu, cl);
                gpu.EndUpload();
                std::this_thread::sleep_for(std::chrono::milliseconds(20));   // loads need time
                if (it % 150 == 0) {
                    Log("[warm] pending %u, fetched %u, painted %u (+%u cached)",
                        resMgr.PendingCount(), resMgr.fetchesThisRun,
                        compositor.painted.load(), compositor.cacheHits.load());
                }
            }
            Log("[warm] done: %u fetches, %u tiles painted, %u from composed cache, %s",
                resMgr.fetchesThisRun, compositor.painted.load(), compositor.cacheHits.load(),
                resMgr.stats.c_str());
        }

        // ---- M6j: channel export mode -- pull data OUT through the manager and exit.
        if (!opt.exportSpec.empty()) {
            const int rc =
                RunChannelExport(opt.exportSpec, opt.exportOut, compositor, colCh, hgtCh);
            gpu.WaitIdle();
            resMgr.Shutdown();
            return rc;
        }

        using Clock = std::chrono::steady_clock;
        auto last = Clock::now();
        auto lastTitle = last;
        uint32_t frame = 0;
        // M9b: a programmatic .wpix is serialized on a PIX background thread AFTER the
        // capture frames present. The process used to exit ~4 frames later and the file
        // landed as a 1 KB stub -- armed, never written. Remember that we armed, and hold
        // the process open at the end until the file stops growing.
        bool pixArmed = false;
        bool dumpedSolid = false;   // --dump-both: the solid image is already on disk
        double frameMsSum = 0.0;
        uint32_t frameMsN = 0;
        // M9o: a recording carries its OWN cost. A rail is the only run long enough and varied
        // enough -- globe to helm, every residency regime in one take -- for frame time to mean
        // something, and it is exactly the run nobody measures because they are watching the
        // pictures. Per-frame, not averaged: a mean hides the stall that a viewer actually sees.
        std::vector<float> railMs, railLoopMs;
        // M9u: WHERE THE OTHER HALF OF THE FRAME GOES. --bench showed render is only 7.9 of a
        // 17.2 ms helm frame; this splits the remaining 9.4 ms by section. Accumulated in two
        // buckets -- the whole rail, and the HELM leg alone -- because the cost is altitude
        // dependent and a single total would average the expensive view away, which is the
        // mistake the phase table already caught once.
        // M9v: prefetch cadence, --predict-every. 1 = every frame (the shipped behaviour).
        const uint32_t kPredictEvery = (std::max)(1u, opt.predictEvery);
        // Step 1 (perf plan): two brackets that were unnamed -- the 17 wave-plane Wants after
        // waveField.Update (the 0.60 ms helm residual between "sum of the ten" and the
        // pre-RenderFrame total in out/instr_bench.log) and the exposure roll + Want that
        // sea.SetTime's bracket used to swallow.
        constexpr int kProfN = 12;
        double profMs[kProfN] = {}, profHelmMs[kProfN] = {};
        std::vector<float> railPreMs;   // M9u: the whole pre-RenderFrame half
        uint64_t walkNodesAcc = 0, walkLeavesAcc = 0, walkWantNsAcc = 0, walkFrames = 0;
        uint64_t wantTouchAcc = 0, wantHitAcc = 0;
        static const char* kProfName[kProfN] = {
            "weather.Update", "scene hot-reload stat", "waveField.Update",
            "waterBank.SetFrame", "tide.SetTime", "sea.SetTime",
            "groundAt (cam clamp)", "globe.SetView", "globe.PredictWants", "swe (solver step)",
            "wave-plane Wants (17)", "exposure roll + Want"};
        std::chrono::steady_clock::time_point profT0;
        bool profHelm = false;
        // The section brackets accumulate only over the frames the [rail] series keeps (a rail's
        // 150 settle frames are excluded, as the walk stats already were); the divisor is the
        // series length, so the two now agree. profOn is set at the top of every iteration.
        bool profOn = true;
#define PROF_BEGIN() profT0 = Clock::now()
#define PROF_END(slot)                                                                         do {                                                                                           const double _e =                                                                              std::chrono::duration<double>(Clock::now() - profT0).count() * 1000.0;                  if (profOn) {                                                                                  profMs[slot] += _e;                                                                        if (profHelm) profHelmMs[slot] += _e;                                                  }                                                                                          } while (0)
        // THE CPU INSIDE RenderFrame, named. RENDER = record + fence (+ serialized GPU under
        // --bench); the record half held three untimed CPU costs: the residency turn
        // (ProcessQueues, by phase -- probe P5 of the plan), the bank's tile-list build (P6) and
        // the meshlet memcpy. Each layer times itself; they are read and zeroed here so a frame
        // that skipped a layer contributes nothing. Accumulated like the section table, and
        // written per frame into metrics.csv.
        constexpr int kInPhases = ResidencyManager::kPhases;
        constexpr int kInResTurn = kInPhases, kInBankList = kInPhases + 1,
                      kInMeshCopy = kInPhases + 2, kInN = kInPhases + 3;
        double profInMs[kInN] = {}, profInHelmMs[kInN] = {};
        std::vector<std::array<float, kInN>> railInMs;
        // metrics.csv pairs loop_ms with its own frame: the interval measured at the top of an
        // iteration is the PREVIOUS frame's, so it closes the row that frame opened.
        bool loopRowOpen = false;
        // --settle-sync state: the instant is held at the dump frame while the residency
        // manager drains; the still is then a function of pose and instant alone.
        bool settling = false;
        uint32_t settleFrames = 0, settleQuiet = 0, settlePending0 = 0, settleReads0 = 0;
        constexpr uint32_t kSettleQuietFrames = ResidencyManager::kEvictAgeFrames;
        constexpr uint32_t kSettleCapFrames = 3000;   // then dump anyway, and say so
        // Step 25: --settle-exact's own count -- consecutive turns the manager reported the
        // resident set equal to the want set. kEvictAgeFrames past the last drop its NULL map
        // has landed; the +4 is one more overlap window with nothing moving.
        uint32_t settleExactQuiet = 0;
        constexpr uint32_t kSettleExactFrames = ResidencyManager::kEvictAgeFrames + 4u;
        FramePipe recPipe;
        std::vector<uint8_t> recPixels;
        if (!opt.mp4.empty()) {
            recPipe.Open(opt.mp4, renderer.Width(), renderer.Height(), 30);
        }
        std::vector<uint64_t> railPool;
        if (!opt.rail.empty()) {
            railMs.reserve(opt.frames ? opt.frames : 1200);
            railLoopMs.reserve(opt.frames ? opt.frames : 1200);
            railPool.reserve(opt.frames ? opt.frames : 1200);
        }

        for (;;) {
            if (!opt.headless) {
                window.NewFrame();
                if (!window.PumpMessages()) break;
                if (window.TakeResized()) renderer.OnResize(window.Width(), window.Height());
            }

            int simSteps = 0;   // whole sim quanta this frame (sim/SimClock.h)
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

                cam.Update(in, dt);
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
                // The churn atlas is stateful and its kernel only climbs at a frozen dt, so a
                // held frame would advance the foam the hold's length decides. Freeze it for
                // exactly the held frames (SeaLayer.h freezeChurn).
                if (sea) sea->freezeChurn = settling;
                // The helm leg of --rail-flood: keys at 32 s (cHelmIn) and 40 s (cHelmGap).
                profHelm = !opt.rail.empty() && recFrame >= 32u * 30u;
                if (!opt.rail.empty() && !railKeys.empty()) {
                    // M6g: the rails just set a pose in the ONE frame. Nothing switches.
                    railPose(static_cast<double>(recFrame) / 30.0, cam);
                }
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
                if (sea) sea->enabled = altV < 60000.0 && !marsMode && !opt.albedo;
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
                        if (waveSrc && waveT >= 0 && waveField->Ready() &&
                            waveField->LiveKey() != waveSrc->Key()) {
                            waveSrc->SetKey(waveField->LiveKey());
                            auto fresh = std::make_shared<TileTree>(waveSrc.get(), TileTree::Fmt::Raw4);
                            float u0, v0, u1, v1;
                            waveSrc->WindowUv(u0, v0, u1, v1);
                            const auto tp0 = Clock::now();
                            const uint32_t nPlanes = waveField->Table().nUsed + 1u;
                            const uint32_t filled = fresh->Prefill(waveFrame.color, 0u, nPlanes, u0, v0, u1, v1);
                            std::atomic_store(waveTree.get(), fresh);
                            resMgr.Drop(waveT);
                            Log("[wave] bucket %016llx -> tree %s: %u tiles prefilled (%u planes, "
                                "every mip) in %.2f s",
                                static_cast<unsigned long long>(waveSrc->Key()), fresh->Id().c_str(),
                                filled, nPlanes,
                                std::chrono::duration<double>(Clock::now() - tp0).count());
                        }
                        if (waveSrc && waveT >= 0 && waveSrc->Key() != 0) {
                            // Bracketed apart from waveField.Update: these Wants are the whole
                            // window at mip 0 on every plane, a map find per un-stamped tile.
                            PROF_BEGIN();
                            float u0, v0, u1, v1;
                            waveSrc->WindowUv(u0, v0, u1, v1);
                            const uint32_t nPlanes = waveField->Table().nUsed + 1u;
                            for (uint32_t p = 0; p < nPlanes; ++p) {
                                resMgr.Want(waveT, 6u + p, 0u, u0, v0, u1, v1);
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
                                                   winOrgX, winOrgY,
                                                   hgtWinTenant == hgtTenant ? 6u : UINT32_MAX);
                    }
                    globe->windGateVal = sea->WindGate();
                    globe->SetWaterBank(waterBank->DispSrv(), waterBank->ParamSrv(),
                                        waterBank->DetailSrv(), derivS, patchS, bandKS,
                                        bandRmsS, bandFoldS, sea->heightScale,
                                        waterBank->BaseTexelM(), orgs, opt.oneWater);
                }
                // (--albedo: the water stands down too -- textures judged as layered images,
                // nothing else in the frame; the lit look retunes separately.)
                sky->enabled = altV < 9000.0 && !marsMode && !opt.albedo;   // low haze dome...
                if (globe) globe->skyPassEnabled = !sky->enabled && !opt.albedo;   // ...or the
                                                            // limb shell, never both at once
                                                            // (--albedo: neither -- textures)
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
                    fresh->onChanged = [&resMgr, &exposureT](const std::string&, const TileRequest& r) {
                        if (exposureT < 0) return;
                        TileRequest q = r;
                        q.face = 6u;
                        resMgr.Invalidate(exposureT, q);
                    };
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
                        return ((lonDeg + 180.0) / 360.0 * n14 - 1263360.0) / 16384.0;
                    };
                    auto mV = [&](double latDeg) {
                        const double l = latDeg * piP / 180.0;
                        return ((0.5 - std::log(std::tan(piP * 0.25 + l * 0.5)) / (2.0 * piP)) *
                                    n14 - 1538048.0) / 16384.0;
                    };
                    const float u0 = float(std::clamp(mU(lonC - dLon), 0.0, 1.0));
                    const float u1 = float(std::clamp(mU(lonC + dLon), 0.0, 1.0));
                    const float v0 = float(std::clamp(mV(latC + dLat), 0.0, 1.0));
                    const float v1 = float(std::clamp(mV(latC - dLat), 0.0, 1.0));
                    if (u1 > u0 && v1 > v0) resMgr.Want(exposureT, 6u, 3u, u0, v0, u1, v1);
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
                continue;
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
                    settleExactQuiet = (opt.settleExact && ex.exact) ? settleExactQuiet + 1 : 0;
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
                        continue;
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
                // M7p: export the inlet box's REAL fields (bed, level, current, shadow)
                // so proofs/inlet_storm.py -- the user's own vqview wave model -- can run
                // the independent 2D storm figure on the exact data this engine uses.
                if (opt.dumpWater && !marsMode && sea) {
                    // The export reads the solver mirrors through Query: bring them to this
                    // instant first (the one readback of the run; the loop never did one).
                    weather.RefreshMirrorsTo(gpu, simUnix);
                    const double bx0 = -1200.0, bz0 = -1600.0, cellW = 10.0;
                    const int nxW = 420, nyW = 300;
                    std::vector<float> bedW(nxW * nyW), lvlW2(nxW * nyW), uW(nxW * nyW),
                        vW(nxW * nyW), shW(nxW * nyW);
                    for (int j = 0; j < nyW; ++j) {          // row 0 = SOUTH (+v = north)
                        for (int i2 = 0; i2 < nxW; ++i2) {
                            const double wxD = bx0 + (i2 + 0.5) * cellW;
                            const double wzD = bz0 + (j + 0.5) * cellW;
                            const double latD =
                                BathyModel::kOrgLat + wzD / BathyModel::kMPerLat;
                            const double lonD =
                                BathyModel::kOrgLon + wxD / BathyModel::kMPerLon;
                            const WeatherSample q =
                                weather.Query(latD, lonD, simUnix, cellW);
                            const size_t at = static_cast<size_t>(j) * nxW + i2;
                            bedW[at] = q.bedNavd;
                            lvlW2[at] = static_cast<float>(q.levelNavd);
                            uW[at] = q.u;
                            vW[at] = q.v;
                            shW[at] = sea->ShadowAtWorld(static_cast<float>(wxD),
                                                         static_cast<float>(wzD));
                        }
                    }
                    auto wr = [&](const char* pth, std::vector<float>& g) {
                        if (FILE* f2 = fopen(pth, "wb")) {
                            fwrite(g.data(), sizeof(float), g.size(), f2);
                            fclose(f2);
                        }
                    };
                    wr("ws_bed.f32", bedW);
                    wr("ws_level.f32", lvlW2);
                    wr("ws_u.f32", uW);
                    wr("ws_v.f32", vW);
                    wr("ws_shadow.f32", shW);
                    const WeatherSample qc = weather.Query(42.816, -70.79, simUnix, 500.0);
                    if (FILE* fj2 = fopen("ws_meta.json", "wb")) {
                        fprintf(fj2,
                                "{ \"x0\": %.1f, \"z0\": %.1f, \"cell\": %.1f, "
                                "\"nx\": %d, \"ny\": %d, \"rows\": \"south-to-north\", "
                                "\"hs\": %.2f, \"tp\": %.2f, \"dirFrom\": %.1f, "
                                "\"peakDirX\": %.3f, \"peakDirZ\": %.3f }",
                                bx0, bz0, cellW, nxW, nyW,
                                sea->hsModel > 0.01 ? sea->hsModel : qc.hs, qc.tp,
                                qc.dirDeg, sea->PeakDirX(), sea->PeakDirZ());
                        fclose(fj2);
                    }
                    Log("[waterstate] ws_*.f32 + ws_meta.json exported (%dx%d at %.0f m)",
                        nxW, nyW, cellW);
                }
                if (opt.trace && !marsMode && sea && waterBank) {
                    weather.RefreshMirrorsTo(gpu, simUnix);   // a no-op after the export above
                    const double tlat = opt.traceLat, tlon = opt.traceLon;
                    const double wx = (tlon - BathyModel::kOrgLon) * BathyModel::kMPerLon;
                    const double wz = (tlat - BathyModel::kOrgLat) * BathyModel::kMPerLat;
                    Log("[trace] ==== ONE SAMPLE THROUGH THE STATE DIAGRAM ====");
                    Log("[trace] input       lat %.5f lon %.5f  t %.0f unix", tlat, tlon,
                        simUnix);
                    Log("[trace] 1 frame     latlon.deg -> world.m: x %+.1f z %+.1f  "
                        "(org %.5f,%.5f; mPerLon %.0f mPerLat %.0f; +x=east +z=north)",
                        wx, wz, BathyModel::kOrgLat, BathyModel::kOrgLon,
                        BathyModel::kMPerLon, BathyModel::kMPerLat);
                    const WeatherSample wq = weather.Query(tlat, tlon, simUnix, 30.0);
                    Log("[trace] 2 bed       compose.stack SampleHeightStack: %+.2f m NAVD "
                        "[%s]  (edge compose.stack->water.bank corners)",
                        wq.bedNavd, wq.bedSrc);
                    Log("[trace] 3 level     water.atlas rotors + swe mirror: %+.2f m NAVD "
                        "[%s]", wq.levelNavd, wq.levelSrc);
                    if (waterAtlas.Ready()) {
                        float elo = 0.0f, ehi = 0.0f;
                        waterAtlas.EnvelopeNavd(tlat, tlon, simUnix, &elo, &ehi);
                        Log("[trace] 3b envelope water.atlas origin planes: lo %+.2f hi "
                            "%+.2f m NAVD (synodic-month min/max; live level must sit "
                            "inside; edit floor %+.2f)",
                            elo, ehi, globe ? globe->editFloorNavd : 0.0f);
                    }
                    Log("[trace] 4 current   swe.solver (row0N raster, FLIP into +v=N): "
                        "u %+.2f v %+.2f m/s [%s]  (edge swe.solver->water.bank uv)",
                        wq.u, wq.v, wq.currentSrc);
                    Log("[trace] 5 depth     level - bed = %.2f m  dry %.2f  breaking clamp "
                        "0.55*depth = %.2f m", wq.depthM,
                        std::clamp((wq.depthM - 0.05) / 0.6, 0.0, 1.0),
                        0.55 * (std::max)(static_cast<double>(wq.depthM), 0.05));
                    const double hsRefT = 0.8;
                    const double hsScaleT =
                        wq.hs > 0.0f ? std::clamp(wq.hs / hsRefT, 0.15, 3.0) : 1.0;
                    Log("[trace] 6 sea state Hs %.2f m Tp %.1f s dir %.0f [%s] -> hsScale "
                        "%.2f  (Hs/gulfRef %.2f, clamp 0.15..3)",
                        wq.hs, wq.tp, wq.dirDeg, wq.waveSrc, hsScaleT, hsRefT);
                    const float expoT = sea->ShadowAtWorld(static_cast<float>(wx),
                                                           static_cast<float>(wz));
                    Log("[trace] 7 exposure  swell.exposure node (page z14 mips >= 3, no flip, "
                        "floor 0.18): %.2f  (edge exposure.node->water.bank exposure)",
                        (std::max)(expoT, 0.18f));
                    for (int m = 0; m < 3; ++m) {
                        const double texel = 4.8 * (1 << m);
                        const double lam[3] = {213.0, 26.9, 2.2};
                        Log("[trace] 8 fold r%d  texel %.1f m: w(213m) %.2f  w(26.9m) %.2f  "
                            "w(2.2m) %.2f  (geometry vs sigma2 split, M6t)",
                            m, texel,
                            1.0 - std::clamp((texel - lam[0] * 0.12) / (lam[0] * 0.38), 0.0, 1.0),
                            1.0 - std::clamp((texel - lam[1] * 0.12) / (lam[1] * 0.38), 0.0, 1.0),
                            1.0 - std::clamp((texel - lam[2] * 0.12) / (lam[2] * 0.38), 0.0, 1.0));
                    }
                    Log("[trace] 9 gpu fibers cascades (patch.wrap, no flip) + churn "
                        "(atlas.texel flat, x1.05) + bank write ring texels: GPU-resident; "
                        "contracts printed by [gaast] at boot");
                    Log("[trace] 10 render   bank -> globe BankSample (no flip) -> two rays "
                        "(sandwich -n d n, refraction rotor; gatest-pinned)");
                    Log("[trace] ==== cross-check: NOAA tides at 8440452, GoMOFS currents, "
                        "GFS-Wave Hs -- the provenance strings above name the rungs ====");
                    waterBank->TraceProbe(gpu, wx, wz);
                    // Step 11: THE COMPOSED TILES THEMSELVES. Same point, three answers
                    // that must agree: the CPU stack (the law), the resident GPU texel of
                    // the height window (what the renderer actually reads), and the
                    // residency map that says which mip that is. stack -> cache -> GPU,
                    // end to end.
                    if (hgtWinTenant >= 0 && hgtCh >= 0) {
                        const double piT = 3.14159265358979;
                        const double n14 = 16384.0 * 256.0;
                        const double mxT = (tlon + 180.0) / 360.0 * n14;
                        const double myT =
                            (0.5 - std::log(std::tan(piT * 0.25 + tlat * piT / 360.0)) /
                                       (2.0 * piT)) *
                            n14;
                        const double uT = (mxT - winOrgX) / 16384.0;
                        const double vT = (myT - winOrgY) / 16384.0;
                        if (uT > 0.0 && uT < 1.0 && vT > 0.0 && vT < 1.0) {
                            const uint32_t hwfT = (hgtWinTenant == hgtTenant) ? 6u : 0u;
                            const uint32_t mipT = resMgr.ResidentMipAt(
                                hgtWinTenant, hwfT, static_cast<float>(uT),
                                static_cast<float>(vT));
                            if (mipT <= 7) {
                                const uint32_t dimT = 16384u >> mipT;
                                uint32_t txT = static_cast<uint32_t>(uT * dimT);
                                uint32_t tyT = static_cast<uint32_t>(vT * dimT);
                                if (txT >= dimT) txT = dimT - 1;
                                if (tyT >= dimT) tyT = dimT - 1;
                                uint8_t pxT[16] = {};
                                float gpuH = 0.0f;
                                if (gpu.ReadbackTexel(resMgr.TextureRes(hgtWinTenant),
                                                      hwfT * resMgr.Mips(hgtWinTenant) + mipT,
                                                      txT, tyT,
                                                      resMgr.TextureState(hgtWinTenant),
                                                      pxT)) {
                                    const uint16_t h16 =
                                        static_cast<uint16_t>(pxT[0] | (pxT[1] << 8));
                                    const uint32_t sT = (h16 >> 15) & 1u,
                                                   eT = (h16 >> 10) & 31u,
                                                   mT2 = h16 & 1023u;
                                    gpuH = (eT == 0)
                                               ? 0.0f
                                               : std::ldexp(1.0f + mT2 / 1024.0f,
                                                            static_cast<int>(eT) - 15) *
                                                     (sT ? -1.0f : 1.0f);
                                }
                                // CPU stack at the TEXEL CENTRE, at the texel's own res.
                                const double pxC = winOrgX + (txT + 0.5) * (1 << mipT);
                                const double pyC = winOrgY + (tyT + 0.5) * (1 << mipT);
                                const double lonC = pxC / n14 * 360.0 - 180.0;
                                const double latC =
                                    std::atan(std::sinh(piT * (1.0 - 2.0 * pyC / n14)));
                                const float cpuH = compositor.SampleHeightStack(
                                    hgtCh, latC, lonC * piT / 180.0,
                                    9.55 * (1 << mipT));
                                const float dH = std::abs(gpuH - cpuH);
                                const float tol =
                                    0.06f + 0.02f * std::abs(cpuH);
                                Log("[trace] 11 compose  height.pages z14 mip %u texel "
                                    "(%u,%u): GPU %+.2f m vs CPU stack %+.2f m  %s "
                                    "(edge compose.stack->height.pages, |d| %.3f tol %.3f)",
                                    mipT, txT, tyT, gpuH, cpuH,
                                    dH <= tol ? "MATCH" : "MISMATCH", dH, tol);
                            } else {
                                Log("[trace] 11 compose  height.pages z14: nothing resident "
                                    "at this uv yet");
                            }
                        }
                    }
                    // M9ba: the exposure field is the tree folder cache\trees\swell.exposure.*
                }
                break;
            }
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

        if (opt.seaVerify && sea) {
            const double hr = sea->MeasureRenderedHs(gpu);
            Log("[verify] sea: model Hs %.3f m, RENDERED Hs %.3f m "
                "(one realization; agreement within ~10%% passes)", sea->hsModel, hr);
        }
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
    } catch (const std::exception& e) {
        Log("FATAL: %s", e.what());
        return 1;
    }
}
