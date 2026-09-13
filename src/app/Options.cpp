// Options - the parser, moved verbatim from main.cpp (M12 step 1a). See Options.h.
#include "app/Options.h"

#include "core/Common.h"
#include "core/Json.h"

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <string>

namespace ga::app {

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
        // M10: the Droste rails. Both ride the storm rail's keys to the helm (--rail-droste
        // implies the flood keys), then leave them for the similarity's own spiral.
        else if (a == "--rail-droste") {
            o.rail = Widen(next("rail_frames").c_str());
            o.railFlood = true;
            o.railDroste = true;
            o.droste = true;
        }
        else if (a == "--rail-droste-out") {
            o.rail = Widen(next("rail_frames").c_str());
            o.railFlood = true;
            o.railDrosteOut = true;
            o.droste = true;
        }
        else if (a == "--droste") o.droste = true;
        else if (a == "--droste-at") {
            const std::string v = next("42.81826,-70.80045,16");
            double la = o.drosteLat, lo = o.drosteLon;
            int lv = o.drosteLevel;
            const int n = sscanf_s(v.c_str(), "%lf,%lf,%d", &la, &lo, &lv);
            if (n >= 2) { o.drosteLat = la; o.drosteLon = lo; }
            if (n >= 3) o.drosteLevel = std::clamp(lv, 4, 20);
            o.droste = true;
        }
        else if (a == "--droste-fill") o.drosteFill = std::clamp(atof(next("1").c_str()), 0.05, 2.0);
        else if (a == "--droste-twist") o.drosteTwistDeg = atof(next("90").c_str());
        else if (a == "--droste-light") {
            const std::string n = next("realistic");
            o.drosteLight = (n == "appealing" || n == "1") ? 1 : 0;
        }
        else if (a == "--droste-level-sec") o.drosteLevelSec = std::clamp(atof(next("16").c_str()), 2.0, 120.0);
        else if (a == "--droste-levels") o.drosteLevels = std::clamp(atoi(next("3").c_str()), 1, 6);
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
        else if (a == "--twin-surface") o.twinSurface = true;
        else if (a == "--boat") o.boat = next("box.test");
        else if (a == "--boat-drive") {
            const std::string v = next("1,0");
            float t = 1.0f, st = 0.0f;
            sscanf_s(v.c_str(), "%f,%f", &t, &st);
            o.boatThrottle = t;
            o.boatSteer = st;
            o.boatDrive = true;
        }
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
        // M9bk: the wireframe with the water shading OFF -- geometry, read as geometry.
        else if (a == "--wireflat") o.surfaceDebug = 3;
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
        else if (a == "--wave-map") o.waveMap = Widen(next("wave_map.png").c_str());
        else if (a == "--bathy-map") o.bathyMap = Widen(next("bathy_map.png").c_str());
        else if (a == "--fidelity-map") {
            o.fidelityMap = Widen(next("fidelity_map.png").c_str());
        }
        else if (a == "--ocean-probe") o.oceanProbe = next("42.35,-70.65");
        // M9bp: both are the DEFAULT now; the positive forms stay so scripts keep parsing.
        else if (a == "--one-water") o.oneWater = true;
        else if (a == "--no-one-water") o.oneWater = false;
        else if (a == "--pixel-water") o.pixelWater = true;
        else if (a == "--no-pixel-water") o.pixelWater = false;
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
                                : o.railDroste
                                    // storm rail + the 2 s helm hold + the dive (whose ease-in
                                    // costs half its 3 s ramp) + a 2 s hold on the last helm
                                    ? static_cast<uint32_t>((40.0 + 2.0 + 1.5 +
                                                             o.drosteLevels * o.drosteLevelSec + 2.0) * 30.0)
                                : o.railDrosteOut
                                    // 14 s of storm rail + 2 s hold + in (2T) + 3 s turn +
                                    // out (2T) + the 28 s climb to orbit
                                    ? static_cast<uint32_t>((47.0 + 4.0 * o.drosteLevelSec) * 30.0)
                                : o.railFlood                        ? 40 * 30
                                : (o.railZoom || o.planet == "mars") ? 30 * 30
                                                                     : 25 * 30;
        o.timeScale = 1.0;
    }
    if (o.planet == "mars") o.globeStart = true;   // there is only orbit on Mars (for now)
    return o;
}

}  // namespace ga::app
