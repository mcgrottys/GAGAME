// Options - the parser, moved verbatim from main.cpp (M12 step 1a). See Options.h.
#include "app/Options.h"

#include "core/Common.h"
#include "core/Json.h"
#include "scene/Props.h"
#include "scene/SceneBuilder.h"
#include "sim/GlobeModel.h"   // kR: the orbit eye's default altitude is 2.1 radii

#include <algorithm>
#include <chrono>
#include <cmath>
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
        // ---- M12 step 5a: the scene front door (see Options.h) -- its own block ahead of the
        // legacy chain, which sits at the compiler's block-nesting limit (MSVC C1061).
        if (a == "--print-scene") { o.printScene = true; continue; }
        if (a == "--set") { o.sets.push_back(next("")); continue; }
        if (a == "--tool") { o.tools.push_back(next("")); continue; }
        if (a.size() > 5 && a[0] != '-' && a.compare(a.size() - 5, 5, ".json") == 0) {
            o.scenePath = a;
            continue;
        }
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
        else if (a == "--trace") {
            // M7j: THE HYPERVISOR. One sample walked through the whole one-water chain on
            // the CPU, every transformation printed with its AST edge -- validate against
            // external tools, find the failing step BEFORE the GPU is involved.
            // PHASE C4: "" (or no point) = the scene's place.anchor.
            if (swscanf(Widen(next("").c_str()).c_str(), L"%lf,%lf", &o.traceLat, &o.traceLon) != 2) {
                o.traceLat = o.traceLon = std::numeric_limits<double>::quiet_NaN();
            }
            o.trace = true;
        }
        else if (a == "--pix") {
            o.pixFrames = static_cast<uint32_t>(_wtoi(Widen(next("1").c_str()).c_str()));
            if (o.pixFrames == 0) o.pixFrames = 1;
        }
        else if (a == "--dump-fibers" || a == "--sky-probe" || a == "--bed-trace-plant" ||
                 a == "--bank-trace-plant") {
            // Four flags on one link: the else-if chain below is AT the compiler's
            // nesting limit (C1061 on one more `else if`), so a new flag joins a
            // neighbour rather than deepening it.
            if (a == "--sky-probe") o.skyProbe = true;
            else if (a == "--bed-trace-plant") o.bedTracePlant = true;
            else if (a == "--bank-trace-plant") o.bankTracePlant = true;
            else o.dumpFibers = true;
        }
        else if (a == "--lens") {
            // residency[.height|.landsea]: what the sampler is allowed to read -- the page that
            // answers and the floor its sample is clamped by (shaders/ResidencyLens.hlsl).
            const std::string n = next("worldxz");
            o.lens = n == "worldxz" ? 1 : n == "winuv" ? 2 : n == "mip" ? 3
                     : n == "ring" ? 4 : n == "cascade" ? 5
                     : n == "waterdata" ? 6 : n == "velgrad" ? 7 : n == "shell" ? 8
                     : n == "residency" ? 9 : n == "residency.height" ? 10
                     : n == "residency.landsea" ? 11 : n == "mix" ? 12 : n == "mix.near" ? 13
                     : n == "cloudalt" ? 15 : n == "blocks" ? 16 : 1;
        }
        // Phase A0 (plan_eye_windows.md): one ground point (lat,lon, degrees) read back through every
        // world of the frame, in --lens blocks' bottom-left strip (ResidencyLens.hlsl BlockProbe).
        else if (a == "--ground-probe") o.groundProbe = next("");
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
        else if (a == "--res-trace") o.resTrace = true;
        // M13 step 0: count the water tiles the walk would want on the planet's own lattice.
        else if (a == "--water-tiles") o.waterTiles = true;
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
        // The drawn sea against the water each hull reads (app/Tools/WaterProbe.cpp): the scene
        // depth read back every N recorded frames. An instrument -- its readbacks stop the GPU.
        else if (a == "--water-probe" || a == "--pages-trace" || a == "--res-audit" ||
                 a == "--bed-trace" || a == "--bank-trace" || a == "--near-ground") {
            // Four every-N instruments on one link (the chain is at C1061's limit, line ~158).
            // --pages-trace is the slice pool's (stage 0): the pages ledger every Nth turn.
            // --res-audit is the scene's capture.residencyAudit: the residency bytes against the
            // mapped set every Nth turn (hal/ResidencyAudit.h).
            // --bed-trace is the solver's bed read back through its own kernel (Tools/BedTrace).
            const uint32_t every = uint32_t(atoi(next("30").c_str()));
            if (a == "--pages-trace") o.pagesEvery = every;
            else if (a == "--res-audit") o.resAudit = every;
            else if (a == "--bed-trace") o.bedTraceEvery = every;
            // --bank-trace is the water bank's bed (Phase B0): WaterBankLayer::TraceRead.
            else if (a == "--bank-trace") o.bankTraceEvery = every;
            // --near-ground is Phase B2w's: the rank and mip under the frame's bottom third.
            else if (a == "--near-ground") o.nearGroundEvery = every;
            // --starve-plant S: the watchdog's plant -- the loader never starts a load of slice S.
            else if (a == "--starve-plant") o.starvePlant = every;
            else o.waterProbeEvery = every;
        }
        else if (a == "--tree-audit" || a == "--tree-prune") {
            // Two tree tools on one link (the chain is at C1061's limit, line ~158). --tree-prune
            // takes NO argument, so it can never swallow the scene path after it: everything it
            // is told arrives as prune.* scene keys, and its default is a listing.
            if (a == "--tree-prune") o.treePrune = true;
            else o.treeAudit = uint32_t(atoi(next("400").c_str()));
        }
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
            o.exportSpec = next("earth.color.cube.f5:2");
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
            // Views the user asked to KEEP live in scenes/views/builtin.json (M12 step 5a: the
            // table that was code here, as DATA in the scene file's own spelling -- {name, at:
            // {x east, alt, z north, az compass, pitch}}, SetFromCompass's order), not in the
            // gitignored data/ folder; the same five floats come back out of it.
            bool found = false;
            {
                std::ifstream bf("scenes/views/builtin.json", std::ios::binary);
                const std::string btext((std::istreambuf_iterator<char>(bf)),
                                        std::istreambuf_iterator<char>());
                std::string berr;
                const JsonValue broot = JsonParser::Parse(btext, &berr);
                const JsonValue* bviews = berr.empty() ? broot.Get("views") : nullptr;
                if (bviews) {
                    for (const JsonValue& v : bviews->arr) {
                        if (v.Str("name") != o.view) continue;
                        const JsonValue* at = v.Get("at");
                        if (!at) continue;
                        o.camX = static_cast<float>(at->Num("x", 0.0));
                        o.camAlt = static_cast<float>(at->Num("alt", 2.0));
                        o.camZ = static_cast<float>(at->Num("z", 0.0));
                        o.camAz = static_cast<float>(at->Num("az", 90.0));
                        o.camPitch = static_cast<float>(at->Num("pitch", 0.0));
                        found = true;
                    }
                }
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
                fprintf(stderr, "--view %s: not in scenes/views/builtin.json and not in "
                        "data/views.json (press F5 in the viewer to save one)\n", o.view.c_str());
                exit(2);
            }
        }
        else if (a == "--swe-off") o.sweOff = true;
        else if (a == "--swe-west-off") o.sweWestOff = true;   // diagnostic: west strip = ocean clock
        else if (a == "--swe-uv") o.sweUvDump = Widen(next("swe_uv.png").c_str());
        else if (a == "--swe-gain") o.sweGain = static_cast<float>(atof(next("3.2").c_str()));
        else if (a == "--swe-spinup") o.sweSpinupH = atof(next("0.25").c_str());
        else if (a == "--swe-cycle" || a == "--swe-cycle-stage") {   // one link: the chain is at
            if (a == "--swe-cycle") o.sweCycleH = atof(next("13").c_str());   // MSVC's nesting limit
            else o.sweCycleStageM = atof(next("0").c_str());
        }
        else if (a == "--water-map") o.waterMap = Widen(next("water_map.png").c_str());
        else if (a == "--wave-map") o.waveMap = Widen(next("wave_map.png").c_str());
        else if (a == "--bathy-map") o.bathyMap = Widen(next("bathy_map.png").c_str());
        else if (a == "--fidelity-map") {
            o.fidelityMap = Widen(next("fidelity_map.png").c_str());
        }
        else if (a == "--ocean-probe") o.oceanProbe = next("");   // PHASE C4: the point is the caller's
        // M9bp: both are the DEFAULT now; the positive forms stay so scripts keep parsing.
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

// ================================================================================================
//  M12 step 5a: THE SHIM. Options::ToSets is a PURE function from the parsed flags to the
//  scene's spelling of them -- a base scene file, the property overrides in command-line order,
//  the tools to run, and the pure instruments that stay flags -- reproducing the implication
//  laws ParseArgs applied (a rail is headless, in orbit, 1200 frames at real time; a dump is
//  one headless frame; Mars is orbit). A field is emitted when it differs from the struct's
//  default, so what the flag line SAID is what the sets carry and the base file carries the
//  rest; --print-scene folds both (scene/SceneBuilder.h) and prints the resolved document,
//  which is the memento a recipe is judged by (scenes/recipes/*.json).
//
//  THE NUMBER LAW here: a legacy float field consumed as a float (sun.az, water.foam, the
//  compass az/pitch) prints as the shortest decimal that narrows back to it, and the reader
//  narrows; a legacy float consumed as a DOUBLE by the code (--campos / --cam / --globe-cam's
//  positions and lat/lon, which SetFromCompass and LatLonDir take as doubles) prints as the
//  double the code held -- the float widened, shortest round-trip -- so a double scene
//  reproduces the legacy value bit for bit either way. Every recorded recipe value is exactly
//  representable, so the recipes read as they were typed.
//
//  THE ONE DELIBERATE NON-IDENTITY (the plan's): --boat without --campos spawns the hull at
//  the 1e9 sentinel today -- off the world, "AGROUND at depth -278 m"; the scene form refuses
//  it with the reason rather than write a spawn that means nothing.
// ================================================================================================
namespace {

std::string Narrow(const std::wstring& w) {
    std::string s;
    for (wchar_t c : w) s.push_back(static_cast<char>(c));
    return s;
}

// The inverse of DaysFromCivil (Howard Hinnant's civil_from_days).
void CivilFromDays(int64_t z, int& y, unsigned& m, unsigned& d) {
    z += 719468;
    const int64_t era = (z >= 0 ? z : z - 146096) / 146097;
    const unsigned doe = static_cast<unsigned>(z - era * 146097);
    const unsigned yoe = (doe - doe / 1460 + doe / 36524 - doe / 146096) / 365;
    const int64_t yy = static_cast<int64_t>(yoe) + era * 400;
    const unsigned doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
    const unsigned mp = (5 * doy + 2) / 153;
    d = doy - (153 * mp + 2) / 5 + 1;
    m = mp < 10 ? mp + 3 : mp - 9;
    y = static_cast<int>(yy + (m <= 2));
}

// time.start: "now" for the sentinel, civil UTC text for a whole second, else the number.
JsonValue StartValue(double startUnix) {
    if (startUnix < 0) return scene::JsonStr("now");
    const double whole = std::floor(startUnix);
    if (whole != startUnix) return scene::JsonNum(startUnix);
    const int64_t secs = static_cast<int64_t>(whole);
    int64_t days = secs / 86400;
    int64_t rem = secs - days * 86400;
    if (rem < 0) {
        rem += 86400;
        --days;
    }
    int y;
    unsigned m, d;
    CivilFromDays(days, y, m, d);
    char buf[40];
    snprintf(buf, sizeof(buf), "%04d-%02u-%02uT%02lld:%02lld:%02lldZ", y, m, d,
             static_cast<long long>(rem / 3600), static_cast<long long>((rem / 60) % 60),
             static_cast<long long>(rem % 60));
    return scene::JsonStr(buf);
}

const char* kLensNames[] = {"worldxz", "worldxz",   "winuv",    "mip",
                            "ring",    "cascade",   "waterdata", "velgrad",
                            "shell",   "residency", "residency.height", "residency.landsea",
                            "mix",     "mix.near", "worldxz",  "cloudalt", "blocks"};  // 14: --lens addr, deleted (B3)

}  // namespace

SceneArgs Options::ToSets(const Options& o) {
    using scene::JsonBool;
    using scene::JsonNum;
    using scene::JsonObj;
    using scene::JsonSet;
    using scene::JsonStr;
    const Options D;   // the struct's defaults: a field that differs was said, or implied
    SceneArgs out;
    auto set = [&](const std::string& path, JsonValue v) { out.sets.push_back({path, std::move(v)}); };
    auto num = [](double v) { return JsonNum(v); };
    auto f32 = [](float v) { return JsonNum(scene::FloatAsDouble(v)); };   // consumed as a float
    auto wide = [](float v) { return JsonNum(static_cast<double>(v)); };     // consumed as a double
    auto str = [](const std::string& s) { return JsonStr(s); };
    auto wstr = [](const std::wstring& w) { return JsonStr(Narrow(w)); };

    // ---- the mode law (FrameLoop::Session: --sea or --globe open the world, --gulf the gulf
    // map, else the chart; the start camera is the globe's in orbit, the sea's at the jetty).
    const bool world = o.seaStart || o.globeStart;
    const char* mode = world ? "world" : (o.gulfStart ? "gulf" : "chart");
    const char* view = o.globeStart ? "orbit" : (o.seaStart ? "sea" : "chart");
    // M12 step 5f: MARS IS A SCENE FILE (5d's owed finding). --planet mars implies --globe
    // (there is only orbit there), so it used to land on scenes/merrimack.json with two --sets
    // over it -- the planet, and the orbit eye at 2.1 radii OF MARS, a number the shim has to
    // evaluate because a scene file's single value cannot be a function of the planet. Naming
    // scenes/mars.json instead makes the base file carry both; the sets still write the same
    // two values over it, so the resolved document is what it was.
    out.scene = !o.scenePath.empty() ? o.scenePath
                : (o.planet == "mars")     ? "scenes/mars.json"
                : (world || o.gulfStart)   ? "scenes/merrimack.json"
                                           : "scenes/chart.json";
    // M12 step 5d: A NAMED SCENE FILE KEEPS ITS OWN MODE. The law above is what the FLAGS mean --
    // no mode flag is the chart -- so it is written into the document only when the flags are the
    // whole story: no scene file named, or a mode flag given over one. Otherwise `gagame
    // scenes/recipes/helm.json` would be forced to the chart by a law about flags nobody typed.
    // Every recorded recipe's --print-scene line names no scene file, so its text is unchanged.
    const bool modeSaid = o.seaStart || o.globeStart || o.gulfStart;
    if (o.scenePath.empty() || modeSaid) {
        set("scene.mode", str(mode));
        set("scene.view", str(view));
    }
    if (o.planet != D.planet) set("scene.planet", str(o.planet));
    // ---- data
    if (o.shaderDir != D.shaderDir) set("data.shaders", wstr(o.shaderDir));
    // ---- time
    if (o.startUnix != D.startUnix) set("time.start", StartValue(o.startUnix));
    if (o.timeScale != D.timeScale) set("time.timeScale", num(o.timeScale));
    if (o.windowDays != D.windowDays) set("time.windowDays", num(o.windowDays));
    // ---- sun
    if (o.sunPinned) {
        set("sun.source", str("pinned"));
        set("sun.az", f32(o.sunAz));
        set("sun.el", f32(o.sunEl));
    }
    // ---- sea
    if (o.stormHs != D.stormHs || o.stormTp != D.stormTp || o.stormDir != D.stormDir) {
        set("sea.storm.hs", f32(o.stormHs));
        set("sea.storm.tp", f32(o.stormTp));
        set("sea.storm.dir", f32(o.stormDir));
    }
    // ---- water
    if (o.pixelWater != D.pixelWater) set("water.pixelWater", JsonBool(o.pixelWater));
    if (o.foam != D.foam) set("water.foam", f32(o.foam));
    if (o.edgePx != D.edgePx) set("water.edgePx", f32(o.edgePx));
    if (o.heightScale != D.heightScale) set("water.heightScale", f32(o.heightScale));
    if (o.sweOff) set("water.swe.enabled", JsonBool(false));
    if (o.sweWestOff) set("water.swe.westBoundary", JsonBool(false));
    if (o.sweSpinupH != D.sweSpinupH) set("water.swe.spinupH", num(o.sweSpinupH));
    if (o.sweGain != D.sweGain) set("water.swe.gain", f32(o.sweGain));
    if (o.riverQ != D.riverQ) set("water.swe.riverQ", num(o.riverQ));
    if (o.flatBed) {
        set("water.bank.flatBed", JsonBool(true));
        set("water.bank.flatBedNavd", f32(o.flatBedNavd));
    }
    // ---- streaming
    if (o.tileBudget != D.tileBudget) set("streaming.tileBudget", num(o.tileBudget));
    if (o.predictEvery != D.predictEvery) set("streaming.predictEvery", num(o.predictEvery));
    if (o.directStorage != D.directStorage) set("streaming.directStorage", JsonBool(o.directStorage));
    if (o.colorTrees != D.colorTrees) set("streaming.colorTrees", JsonBool(o.colorTrees));
    if (o.gisGate != D.gisGate) set("streaming.gisGate", JsonBool(o.gisGate));
    if (o.seafloor != D.seafloor) set("streaming.seafloor", JsonBool(o.seafloor));
    if (o.exposure != D.exposure) set("streaming.exposure", JsonBool(o.exposure));
    // ---- capture
    if (o.headless) set("capture.headless", JsonBool(true));
    if (o.width != D.width) set("capture.width", num(o.width));
    if (o.height != D.height) set("capture.height", num(o.height));
    if (o.frames != 0) set("capture.frames", num(o.frames));
    if (!o.dump.empty()) set("capture.dump", wstr(o.dump));
    if (!o.dumpHdr.empty()) set("capture.hdr", wstr(o.dumpHdr));
    if (!o.mp4.empty()) set("capture.mp4", str(o.mp4));
    if (!o.rail.empty()) set("capture.railDir", wstr(o.rail));
    if (o.settleSync) set("capture.settle.sync", JsonBool(true));
    if (o.settleHold) set("capture.settle.hold", num(o.settleHold));
    if (o.settleExact) set("capture.settle.exact", JsonBool(true));
    if (o.settleClearChurn) set("capture.settle.clearChurn", JsonBool(true));
    if (o.resAudit != D.resAudit) set("capture.residencyAudit", num(o.resAudit));
    // ---- views: --globe-cam is the orbit view's eye whether or not it starts there (the
    // code's camGlobe); --cam/--campos override the ACTIVE camera after the mode chose it.
    // M12 step 5d: THE ORBIT EYE IS ALWAYS WRITTEN, BECAUSE ITS DEFAULT IS A FUNCTION OF THE
    // PLANET. Without --globe-cam the engine stood at 2.1 radii OF THE PLANET THE FLAGS NAME
    // (main's `planetR * 2.1`, planetR = marsMode ? 3389500 : kR -- app/Assembly.cpp's own
    // expression), and a scene file's single number cannot be that function. So the shim
    // evaluates it: Earth prints 13379100, which is exactly what scenes/merrimack.json already
    // declares, so every recorded recipe's text is byte for byte what it was; Mars prints
    // 7117950, which is what the flag path has always meant and what the pre-scene binary did.
    // A Mars SCENE FILE (scenes/mars.json, with its own orbit view) is the follow-on; this line
    // is what keeps the FLAG path identical now that the boot reads the document.
    // ...and, as with the mode above, the DEFAULT is what the FLAGS mean, so it is written only
    // when the flags are the whole story: no scene file named, or --globe-cam naming the eye
    // outright over one. A named file keeps its own orbit view.
    const bool gcamSaid = o.gcamLat < 1e8f;   // --globe-cam names all three
    if (gcamSaid || o.scenePath.empty()) {
        const double planetR = (o.planet == "mars") ? 3389500.0 : GlobeModel::kR;
        JsonValue at = JsonObj();
        JsonSet(at, "lat", gcamSaid ? wide(o.gcamLat) : num(34.0));
        JsonSet(at, "lon", gcamSaid ? wide(o.gcamLon) : num(-52.0));
        JsonSet(at, "alt",
                num(gcamSaid ? static_cast<double>(o.gcamAltKm) * 1000.0 : planetR * 2.1));
        set("views.orbit.at", at);
    }
    if (o.camAlt > 0) {
        const double cx = (o.camX < 1e8f) ? o.camX : 0.0;
        const double cz = (o.camZ < 1e8f) ? o.camZ : 0.0;
        JsonValue at = JsonObj();
        JsonSet(at, "x", num(cx));
        JsonSet(at, "alt", wide(o.camAlt));
        JsonSet(at, "z", num(cz));
        JsonSet(at, "az", f32(o.camAz));
        JsonSet(at, "pitch", f32(o.camPitch));
        set(std::string("views.") + view + ".at", at);
    }
    if (o.fovDeg != D.fovDeg) set(std::string("views.") + view + ".fovY", f32(o.fovDeg));
    // ---- rails
    const char* rail = o.railJetty ? "jetty" : o.railDroste ? "droste" : o.railDrosteOut ? "droste-out"
                       : o.railFlood ? "flood" : o.railZoom ? "zoom" : !o.rail.empty() ? "classic"
                                                                                       : nullptr;
    if (rail) set("rails.active", str(rail));
    // ---- portals
    if (o.droste) {
        set("portals.droste.enabled", JsonBool(true));   // the --rail-droste* recipes' portal
    }
    // ---- entities
    if (!o.boat.empty()) {
        if (o.camX >= 1e8f || o.camZ >= 1e8f) {
            out.ok = false;
            out.why = "--boat " + o.boat + " without --campos x,z: the legacy run spawns the hull "
                      "at the 1e9 sentinel (off the world, aground at a meaningless depth); the "
                      "scene form refuses to write that spawn -- give --campos";
            return out;
        }
        set("entities.boat.vessel", str(o.boat));
        JsonValue at = JsonObj();
        JsonSet(at, "x", wide(o.camX));
        JsonSet(at, "alt", num(0.0));
        JsonSet(at, "z", wide(o.camZ));
        set("entities.boat.at", at);
        set("entities.boat.controller", str(o.boatDrive ? "fixed" : "helm"));
        // A spawned boat takes the helm (FrameLoop::Session, `helming = true`), and the helm IS
        // the chase camera -- so the active view follows it. The four numbers are the View's
        // declared defaults, which are the literals that block holds.
        set(std::string("views.") + view + ".follow.target", str("boat"));
        if (o.boatDrive) {
            set("entities.boat.throttle", num(o.boatThrottle));
            set("entities.boat.steer", num(o.boatSteer));
        }
    }
    // ---- effects
    if (o.sliceOn) {
        set("effects.slice.type", str("slice.plane"));
        set("effects.slice.d", num(o.sliceD));
    }
    // ---- layers
    if (o.exaggeration != D.exaggeration) set("layers.tide.exaggeration", f32(o.exaggeration));
    // ---- tools, in the boot's order (before the scene, during assembly, after the loop)
    auto tool = [&](const std::string& name, const std::string& args = std::string()) {
        out.tools.push_back(args.empty() ? name : name + ":" + args);
    };
    if (o.packTiles) tool("pack-tiles");
    if (o.treePrune) tool("tree-prune");
    if (!o.loadField.empty()) tool("load-field", o.loadField);
    if (o.selftest) tool("selftest");
    if (!o.waterMap.empty()) tool("water-map", Narrow(o.waterMap));
    if (!o.bathyMap.empty()) tool("bathy-map", Narrow(o.bathyMap));
    if (!o.gisDump.empty()) tool("gis-dump", o.gisDump);
    if (o.warmTrees) tool("warm-trees");
    else if (o.packTrees) tool("pack-trees");
    else if (o.treeAudit) tool("tree-audit", std::to_string(o.treeAudit));
    if (!o.fidelityMap.empty()) tool("fidelity-map", Narrow(o.fidelityMap));
    if (!o.oceanProbe.empty()) tool("ocean-probe", o.oceanProbe);
    if (o.sweCycleH > 0) tool("swe-cycle", scene::NumberText(o.sweCycleH));
    if (!o.sweUvDump.empty()) tool("swe-uv", Narrow(o.sweUvDump));
    if (!o.exportSpec.empty()) tool("export", o.exportSpec + "," + Narrow(o.exportOut));
    if (o.warmInlet) tool("warm-inlet");
    if (o.dumpWater) tool("dump-water-state");
    if (o.twinSurface) tool("twin-surface");
    if (o.trace) {
        tool("trace", std::isnan(o.traceLat) ? std::string()
                                             : scene::NumberText(o.traceLat) + "," + scene::NumberText(o.traceLon));
    }
    if (o.seaVerify) tool("sea-verify");
    if (!o.waveMap.empty()) tool("wave-map", Narrow(o.waveMap));
    for (const std::string& t : o.tools) out.tools.push_back(t);
    // ---- the pure instruments: they stay flags, and are listed as such
    auto rawf = [&](const std::string& s) { out.raw.push_back(s); };
    if (o.pixFrames) rawf("--pix " + std::to_string(o.pixFrames));
    if (o.dumpFibers) rawf("--dump-fibers");
    if (o.lens) rawf(std::string("--lens ") + kLensNames[o.lens > 0 && o.lens < 17 ? o.lens : 0]);
    if (!o.groundProbe.empty()) rawf("--ground-probe " + o.groundProbe);
    if (o.probeCullFar) rawf("--probe-cull-far");
    if (!o.dumpMeshlets.empty()) rawf("--dump-meshlets " + Narrow(o.dumpMeshlets));
    if (o.inject) rawf(o.inject == 2 ? "--inject cascade" : "--inject bank");
    if (o.debugLayer) rawf("--debug");
    if (o.predictInline) rawf("--predict-inline");
    if (o.dsSerial) rawf("--ds-serial");
    if (o.resTrace) rawf("--res-trace");
    if (o.waterTiles) rawf("--water-tiles");
    if (o.threadAudit) rawf("--thread-audit");
    if (o.jobsInline) rawf("--jobs-inline");
    if (o.traceFrom != UINT32_MAX) {
        rawf("--res-trace-frames " + std::to_string(o.traceFrom) + ":" + std::to_string(o.traceTo));
    }
    if (o.skyProbe) rawf("--sky-probe");
    if (o.waterProbeEvery) rawf("--water-probe " + std::to_string(o.waterProbeEvery));
    if (o.pagesEvery) rawf("--pages-trace " + std::to_string(o.pagesEvery));
    if (o.bedTraceEvery) rawf("--bed-trace " + std::to_string(o.bedTraceEvery));
    if (o.bedTracePlant) rawf("--bed-trace-plant");
    if (o.bankTraceEvery) rawf("--bank-trace " + std::to_string(o.bankTraceEvery));
    if (o.bankTracePlant) rawf("--bank-trace-plant");
    if (o.nearGroundEvery) rawf("--near-ground " + std::to_string(o.nearGroundEvery));
    if (o.starvePlant) rawf("--starve-plant " + std::to_string(o.starvePlant));
    if (o.sweCycleStageM != 0) rawf("--swe-cycle-stage " + scene::NumberText(o.sweCycleStageM));
    if (o.benchOverlap) rawf("--bench-overlap");
    else if (o.bench) rawf("--bench");
    if (o.gpuTime) rawf("--gpu-time");
    if (o.noVsync) rawf("--no-vsync");
    if (o.viz) rawf("--viz");
    if (o.surfaceDebug == 1) rawf("--wireframe");
    else if (o.surfaceDebug == 2) rawf("--meshlets");
    else if (o.surfaceDebug == 3) rawf("--wireflat");
    if (o.meshStats) rawf("--mesh-stats");
    if (o.dumpBoth) rawf("--dump-both");
    if (o.stencil) rawf("--stencil");
    if (!o.msSurface) rawf("--no-ms");
    if (o.albedo) rawf("--albedo");
    // ---- the explicit --set lines, last: the strongest spelling, in command-line order
    for (const std::string& s : o.sets) {
        const size_t eq = s.find('=');
        if (eq == std::string::npos) {
            out.ok = false;
            out.why = "--set " + s + ": expected path=value";
            return out;
        }
        out.sets.push_back({s.substr(0, eq), scene::SceneBuilder::SetValue(s.substr(eq + 1))});
    }
    return out;
}

std::string SetText(const SceneSet& s) {
    std::string v = scene::SceneBuilder::WriteJson(s.value);
    while (!v.empty() && (v.back() == '\n' || v.back() == '\r')) v.pop_back();
    return s.path + "=" + v;
}

// The scene the flags mean, folded (SceneBuilder) and validated; the refusal, when there is
// one, names the path. Shared by --print-scene and the boot's [scene] line.
bool BuildScene(const Options& o, scene::SceneBuilder& b, SceneArgs& a, std::string* why) {
    a = Options::ToSets(o);
    if (!a.ok) {
        if (why) *why = a.why;
        return false;
    }
    if (!b.Load(a.scene, why)) return false;
    for (const SceneSet& s : a.sets) {
        if (!b.Set(s.path, s.value, why)) return false;
    }
    for (const std::string& t : a.tools) {
        const size_t colon = t.find(':');
        JsonValue e = scene::JsonObj();
        scene::JsonSet(e, "args", scene::JsonStr(colon == std::string::npos ? "" : t.substr(colon + 1)));
        if (!b.Set("tools." + t.substr(0, colon), e, why)) return false;
    }
    return b.Resolve(why);
}

int PrintScene(const Options& o, int argc, char** argv) {
    scene::SceneBuilder b;
    SceneArgs a;
    std::string why;
    if (!BuildScene(o, b, a, &why)) {
        fprintf(stderr, "[scene] refused: %s\n", why.c_str());
        return 2;
    }
    for (const std::string& r : a.raw) {
        fprintf(stderr, "[scene] raw instrument, not scene data: %s\n", r.c_str());
    }
    // The flag line that made this document, as its first comment: the memento names its recipe.
    std::string line;
    for (int i = 1; i < argc; ++i) {
        if (std::string(argv[i]) == "--print-scene") continue;
        if (!line.empty()) line += ' ';
        line += argv[i];
    }
    JsonValue& doc = b.Document();
    doc.obj.insert(doc.obj.begin(), {"_recipe", scene::JsonStr(line)});
    const std::string text = scene::SceneBuilder::WriteJson(doc);
    fwrite(text.data(), 1, text.size(), stdout);
    fflush(stdout);
    return 0;
}

}  // namespace ga::app
