// ================================================================================================
//  Options - the command line, as a struct.
//
//  M12 step 1a: moved VERBATIM out of main.cpp so the parser is a unit of its own. Nothing here
//  changed meaning; the flag -> field table is exactly main.cpp's at c27ea44. Step 5a adds the
//  scene front door beside it -- a positional scene file, --set, --tool, --print-scene -- and
//  Options::ToSets, the PURE shim from these fields to the scene's property paths (the
//  implication laws reproduced, the pure instruments listed as the raw flags they stay). ParseArgs
//  survives verbatim as the parser; nothing in the engine reads the scene until step 5d.
// ================================================================================================
#pragma once

#include "core/Json.h"

#include <cstdint>
#include <string>
#include <vector>

namespace ga::scene {
class SceneBuilder;
}

namespace ga::app {

struct SceneArgs;

struct Options {
    uint32_t width = 1600, height = 900;
    bool headless = false;
    bool selftest = false;
    bool trace = false;               // --trace lat,lon: the hypervisor walk (M7j)
    uint32_t pixFrames = 0;           // --pix N: programmatic .wpix capture of N frames
    bool dumpFibers = false;          // --dump-fibers: bank planes as PNGs + range gate
    int lens = 0;                     // --lens worldxz|winuv|mip|ring: value-as-color; 9..11 the
                                      // residency lens of earth.color / earth.height / gis.landsea
    bool probeCullFar = false;        // step 23 probe: cull beyond the horizon at every altitude
    std::wstring dumpMeshlets;        // step 23 probe: the dump frame's meshlet records
    bool dumpWater = false;           // --dump-water-state: inlet fields for proofs/
    bool twinSurface = false;         // --twin-surface: CPU WaterSurface vs the GPU bank
    std::string boat;                 // --boat <kind>: spawn a vessel and helm it
    // --boat-drive t,s: hold the throttles and the helm at fixed values. Headless has no
    // keyboard, so without this a powered run cannot be reproduced or gated at all -- and
    // "does it plane / does it launch off a crest" are exactly the questions that need one.
    double boatThrottle = 0.0, boatSteer = 0.0;
    bool boatDrive = false;
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
    bool waterTiles = false;          // --water-tiles: what the water would cost on the lattice
    bool threadAudit = false;         // --thread-audit: count tile-file collisions between threads
    bool jobsInline = false;          // --jobs-inline: every job on the calling thread, in order
    uint32_t traceFrom = UINT32_MAX;  // --res-trace-frames A:B: the landing ledger every turn of
    uint32_t traceTo = 0;             // recorded frames A..B (Residency.h TurnLedger, step 28)
    bool skyProbe = false;            // --sky-probe: read the atmosphere's tables back off
                                      // the device and hold them against published optical
                                      // depths (SkyLayer::Probe). Once, then the run goes on.
    uint32_t waterProbeEvery = 0;     // --water-probe N: every N recorded frames, the DRAWN sea
                                      // (depth read back) against each hull's own water (WaterProbe)
    uint32_t pagesEvery = 0;          // --pages-trace N: every Nth residency turn, each tenant split
                                      // by the lattices its slices sit on (Residency.h pagesEvery)
    uint32_t resAudit = 0;            // --res-audit N: the scene's capture.residencyAudit -- every
                                      // Nth turn, the residency bytes against the mapped set
                                      // (hal/ResidencyAudit.h)
    uint32_t treeAudit = 0;           // --tree-audit N: compare N tiles/frame, report, exit
    bool warmTrees = false;           // --warm-trees: build them without comparing, then exit
    bool packTrees = false;           // --pack-trees: one archive per node per frame, then exit
    bool treePrune = false;           // --tree-prune: list the trees' tag folders by last use,
                                      // exit; retire and purge are prune.mode + prune.confirm
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
    std::wstring waveMap;             // --wave-map out.png: the solved field on the solver's
                                      // OWN cells, no camera, mesh, fold or light in the way
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
    // M9bp: BOTH DEFAULT ON -- this is the water the M9bk..M9bo pass was measured against and
    // the one the user judged. Every fix in that pass lives on this path: the tangent-bivector
    // normal, the wave-grain walk and its morph band, the fold guard on the lateral term, the
    // residency gradient and water mip bias, the cascade prefilter. With the flags off the
    // SeaLayer grid draws instead and none of it applies, so shipping them off meant shipping
    // the water nobody was looking at. --no-one-water / --no-pixel-water are the escape
    // hatches; the positive forms still parse, so existing scripts and the storm-rail recipe
    // are unchanged.
    bool oneWater = true;             // --no-one-water: M7 -- water geometry from the wave
                                      // vertex bank alone (SeaLayer's grid retires)
    bool pixelWater = true;           // --no-pixel-water: M9bh -- shade the water per PIXEL
                                      // (the two rays: sky mirror + refracted bed cast,
                                      // translucent, no foam). Off = M9bg vertex-shaded.
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
    // ---- M10 THE DROSTE LINK (src/core/Droste.h): the root's address, hung as a leaf.
    bool droste = false;              // --droste: link the root under a leaf of itself
    double drosteLat = 42.81826;      // --droste-at lat,lon[,level]: the leaf is the one at this
    double drosteLon = -70.80045;     // place (default: the entrance mouth, east of the jetty
    int drosteLevel = 16;             // tips) and this quadtree level (16 = a 153 m leaf)
    double drosteFill = 1.0;          // --droste-fill f: the inner globe's diameter / leaf span
    double drosteTwistDeg = 90.0;     // --droste-twist deg: the twist per level about north
    int drosteLight = 0;              // --droste-light realistic|appealing (the lighting A/B)
    bool railDroste = false;          // --rail-droste DIR: the storm rail, then the dive
    bool railDrosteOut = false;       // --rail-droste-out DIR: in two levels, turn, fly out
    double drosteLevelSec = 16.0;     // --droste-level-sec s: rail seconds per level
    int drosteLevels = 3;             // --droste-levels n: how deep the dive rail goes
    // ---- M12 step 5a: THE SCENE FRONT DOOR (scene/SceneBuilder.h). A positional scene file,
    // --set a.b.c=value overrides in command-line order, --tool name[:args], and --print-scene
    // (the resolved document to stdout, exit 0, before the pool, the boot line and any device
    // work). Nothing in the engine reads the scene yet (step 5d wires Assemble and FrameLoop).
    std::string scenePath;            // gagame scenes/x.json
    std::vector<std::string> sets;    // --set a.b.c=value, in order
    std::vector<std::string> tools;   // --tool name[:args]
    bool printScene = false;          // --print-scene

    // THE SHIM: the scene's spelling of these fields (Options.cpp, the banner there).
    static SceneArgs ToSets(const Options& o);
};

// One property override: a path into the scene document and the JSON value assigned.
struct SceneSet {
    std::string path;    // "views.sea.at"
    JsonValue value;
};
// What ToSets answers: a base scene, the overrides in order, the tools, the raw instruments.
struct SceneArgs {
    bool ok = true;
    std::string why;                  // the refusal, when !ok (the --boat sentinel)
    std::string scene;                // the base scene file
    std::vector<SceneSet> sets;       // the property overrides, in order
    std::vector<std::string> tools;   // the one-shot modes, "name[:args]"
    std::vector<std::string> raw;     // the pure instruments, as the flags they stay
};
std::string SetText(const SceneSet& s);   // "path=value", for a log line
// The scene the flags mean, folded (SceneBuilder) and validated; false with `why` naming the
// path. M12 step 5d: the BOOT resolves through this too, so --print-scene and the run that
// follows it cannot resolve differently.
bool BuildScene(const Options& o, scene::SceneBuilder& b, SceneArgs& a, std::string* why);
// --print-scene: the flags' scene form, folded, validated and printed; 0, or 2 with the reason.
int PrintScene(const Options& o, int argc, char** argv);

std::wstring Widen(const char* s);
double NowUnix();
// Days since the unix epoch for a civil date (Howard Hinnant's algorithm).
int64_t DaysFromCivil(int y, unsigned m, unsigned d);
// "YYYY-MM-DD[THH:MM[:SS]]" (UTC, trailing Z tolerated) or a raw unix-seconds number.
double ParseStartTime(const std::string& s);
Options ParseArgs(int argc, char** argv);

}  // namespace ga::app
