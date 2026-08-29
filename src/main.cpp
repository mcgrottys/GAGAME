// ================================================================================================
//  gagame - M0 + M1.
//
//  Modes:
//    (default)     windowed viewer. WASD/QE fly, right-drag look, wheel speed, R reload shaders.
//                  Time controls: SPACE pause, UP/DOWN time-scale x10, LEFT/RIGHT nudge -/+ 1 h
//                  (SHIFT: 1 day), HOME or N back to now, [ ] halve/double the plot window.
//    --selftest    M0 gate: run the reserved-resource null-tile test suite headless and exit
//                  with 0 (pass) / 1 (fail). See src/core/TileAtlas.cpp.
//    --headless    no window; render --frames frames and write --dump to a PNG. The verification
//                  path: a renderer change is provable from a shell.
//
//  The window title is the HUD: sim clock (UTC), time scale, focus-station tide, fit RMS.
// ================================================================================================
#include "core/Gpu.h"
#include "core/Image.h"
#include "core/TileAtlas.h"
#include "core/Window.h"
#include "render/Renderer.h"
#include "scene/FieldSet.h"
#include "scene/GlobeLayer.h"
#include "scene/GulfLayer.h"
#include "scene/SeaLayer.h"
#include "scene/SkyLayer.h"
#include "scene/TerrainLayer.h"
#include "scene/TideLayer.h"
#include "core/Json.h"
#include "core/Pga.h"
#include "core/TileProviders.h"
#include "sim/BathyModel.h"
#include "sim/GlobeModel.h"
#include "sim/CurrentModel.h"
#include "sim/SeaState.h"
#include "sim/SweSolver.h"
#include "sim/TideModel.h"

#include <chrono>
#include <cmath>
#include <cstdio>
#include <ctime>
#include <fstream>
#include <string>

using namespace ga;

namespace {

struct Options {
    uint32_t width = 1600, height = 900;
    bool headless = false;
    bool selftest = false;
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
    float camAlt = -1, camAz = 246, camPitch = -5;   // --cam alt,az,pitch override
    float camX = 1e9f, camZ = 1e9f;   // --campos x,z world override (sea mode)
    std::string bathyPath = "data/bathy/merrimack.json";
    float datumOff = -1.30f;          // tide (m MLLW) + this = water level in NAVD88
    bool datumSet = false;            // --datum given: overrides the CO-OPS datum resolution
    bool sweOff = false;              // --swe-off: analytic tide plane only (pre-M5c behaviour)
    bool sweWestOff = false;          // --swe-west-off: zero the west-boundary deviation
    double sweSpinupH = 0.25;         // solver history integrated before the first frame
    double sweCycleH = 0;             // --swe-cycle N: headless validation over N hours -> CSV
    double riverQ = -1;               // --river q overrides data/river/river.json
    std::wstring sweUvDump;           // --swe-uv f.png: dump the solved current field after
                                      // spin-up (debug picture: red east, blue west)
    float sweGain = 5.0f;             // solved-current gain (see SeaLayer). Recalibrated for
                                      // the M6d wide window: the 13.7 m box-averaged channel
                                      // under-conveys (throat point 0.16 vs ACT 1.08); the
                                      // subgrid-conveyance fix is the tracked next step
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
        auto next = [&](const char* def) -> std::string {
            return (i + 1 < argc) ? argv[++i] : def;
        };
        if (a == "--selftest") o.selftest = true;
        else if (a == "--headless") o.headless = true;
        else if (a == "--debug") o.debugLayer = true;
        else if (a == "--width") o.width = static_cast<uint32_t>(atoi(next("1600").c_str()));
        else if (a == "--height") o.height = static_cast<uint32_t>(atoi(next("900").c_str()));
        else if (a == "--frames") o.frames = static_cast<uint32_t>(atoi(next("1").c_str()));
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
        else if (a == "--cam") {
            const std::string v = next("12,246,-5");
            sscanf_s(v.c_str(), "%f,%f,%f", &o.camAlt, &o.camAz, &o.camPitch);
        }
        else if (a == "--campos") {
            const std::string v = next("522,72");
            sscanf_s(v.c_str(), "%f,%f", &o.camX, &o.camZ);
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
    if (o.sweCycleH > 0) o.headless = true;   // the validation cycle never opens a window
    if (!o.rail.empty()) {
        // The rails demo: 25 s at 30 fps, headless, deterministic real-time waves.
        o.headless = true;
        o.globeStart = true;
        o.frames = 25 * 30;
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
float ResolveDatum(const TideModel& model) {
    const TideStation& fs = model.S(model.Focus());
    if (fs.mllwMinusNavdM > -900.0) {
        Log("[datum] %s CO-OPS datums: MLLW - NAVD88 = %+.3f m", fs.name.c_str(),
            fs.mllwMinusNavdM);
        return static_cast<float>(fs.mllwMinusNavdM);
    }
    double delta = 0.0, best = 1e9;
    std::string from = "NAVD=MSL assumption";
    for (size_t i = 0; i < model.Count(); ++i) {
        const TideStation& s = model.S(i);
        if (s.mllwMinusNavdM > -900.0) {
            const double d = -s.mllwMinusNavdM - s.meanMllwM;   // NAVD height above local MSL
            if (std::abs(d) < std::abs(best)) {
                best = d;
                from = s.name;
            }
        }
    }
    if (best < 1e8) delta = best;
    const float off = static_cast<float>(-(fs.meanMllwM + delta));
    Log("[datum] no NAVD link at %s; MLLW - NAVD88 = %+.3f m via NAVD=MSL%+.3f (from %s)",
        fs.name.c_str(), off, delta, from.c_str());
    return off;
}

// M5c validation harness: integrate a full tidal cycle headless and log probes to CSV --
// ocean sponge (must track the analytic tide), the entrance throat AT the ACT0816 station
// (solved current vs the CO-OPS prediction, the real gate), the Joppa Flats basin (lag +
// attenuation must EMERGE), and the river's standing slope upstream.
template <typename F, typename G, typename H>
void RunSweCycle(Gpu& gpu, SweSolver& swe, F oceanAt, G westAt, H southAt,
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
        swe.AdvanceTo(gpu, t, tideAt, westAt, southAt);
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
    try {
        const Options opt = ParseArgs(argc, argv);

        // ---- M0 + M4: the self-test path needs a device and the shader compiler, nothing else.
        if (opt.selftest) {
            Gpu gpu;
            gpu.Init(nullptr, 64, 64, opt.debugLayer);
            ShaderCompiler sc;
            sc.Init();
            bool ok = RunPgaSelfTest();   // pure CPU: the motor conventions, pinned first
            ok &= RunTileSelfTest(gpu, sc, opt.shaderDir);
            ok &= RunAtlasSelfTest(gpu, sc, opt.shaderDir);
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

        Gpu gpu;
        gpu.Init(opt.headless ? nullptr : window.Handle(), opt.width, opt.height, opt.debugLayer);

        RendererDesc rd;
        rd.shaderDir = opt.shaderDir;
        Renderer renderer;
        renderer.Init(gpu, rd);

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
            sea->targetEdgePx = opt.edgePx;
            sea->sweCurrentGain = opt.sweGain;
            sea->heightScale = opt.heightScale;
            sea->atlasVisualize = opt.viz;
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

        // M5: the CUDEM terrain. Registered AFTER tide (chart) and BEFORE sea so the opaque
        // land draws first and the water covers only what it actually stands above.
        BathyModel bathy;
        TerrainLayer* terrain = nullptr;
        if (bathy.Load(opt.bathyPath)) {
            auto terrOwned = std::make_unique<TerrainLayer>();
            terrain = terrOwned.get();
            terrain->Configure(opt.shaderDir, &bathy);
            terrain->Init(gpu, renderer.Shaders(), fields, renderer.RootSignature());
            renderer.AddLayer(std::move(terrOwned));
            if (sea) {
                sea->SetBathy(terrain->HeightSrv(), bathy.WorldX0(), bathy.WorldZ0(),
                              bathy.WorldSizeX(), bathy.WorldSizeZ());
            }
        } else {
            Log("[main] no bathymetry (run: py -3 harvester\\harvest_bathy.py); open-ocean sea");
        }

        // M5c: the sparse shallow-water solver -- the estuary's own hydrodynamics, tide-forced
        // offshore and river-forced upstream, feeding the sea's mean surface and currents.
        SweSolver swe;
        if (terrain && sea && !opt.sweOff) {
            swe.Init(gpu, renderer.Shaders(), opt.shaderDir, bathy, terrain->HeightTex().res.Get());
            // Logged for the record; the west boundary's station tides already carry the river
            // stage (their fitted means include it), so no explicit injection is needed.
            LoadRiverDischarge("data/river/river.json");
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

        // M6: the planet (registered LAST -- it borrows root param 2 for its node list).
        GlobeModel globeModel;
        GlobeLayer* globe = nullptr;
        if (globeModel.Load("data/globe/globe.json")) {
            auto globeOwned = std::make_unique<GlobeLayer>();
            globe = globeOwned.get();
            globe->Configure(opt.shaderDir, &globeModel);
            globe->windOverlay = opt.viz;
            globe->Init(gpu, renderer.Shaders(), fields, renderer.RootSignature());
            renderer.AddLayer(std::move(globeOwned));
        } else {
            Log("[main] no globe data (run: py -3 harvester\\harvest_globe.py)");
        }

        // ---- M6e: the unified residency manager + the streamed planet surfaces. Two worlds,
        // one machine: Mars streams the rescued sample's BC1/BC5 pyramids from disk; Earth
        // streams Google 2D tiles (cache-first, throttled, budget-capped) reprojected onto the
        // same cube faces. Residency is driven by the CDLOD walk, clamped by residency-map
        // cubes, prefetched along the camera's screw.
        const bool marsMode = (opt.planet == "mars");
        const double planetR = marsMode ? 3389500.0 : GlobeModel::kR;
        ResidencyManager resMgr;
        MarsBinProvider marsDiff, marsNorm;
        GoogleTileProvider googleTiles;
        if (globe) {
            resMgr.Init(gpu);
            int surf = -1, norm = -1;
            if (marsMode) {
                if (marsDiff.Open(L"data/earth/diffuse.bin", DXGI_FORMAT_BC1_UNORM)) {
                    surf = resMgr.AddTextureCube(gpu, L"mars.diffuse (the rescued sample)",
                                                 16384, DXGI_FORMAT_BC1_UNORM, marsDiff.Fn());
                }
                if (marsNorm.Open(L"data/earth/normal.bin", DXGI_FORMAT_BC5_SNORM)) {
                    norm = resMgr.AddTextureCube(gpu, L"mars.normal", 16384,
                                                 DXGI_FORMAT_BC5_SNORM, marsNorm.Fn());
                }
            } else if (googleTiles.Init("satellite", opt.tileBudget)) {
                surf = resMgr.AddTextureCube(gpu, L"earth.google2d", 16384,
                                             DXGI_FORMAT_R8G8B8A8_UNORM,
                                             googleTiles.Fn(&resMgr.fetchesThisRun));
            }
            globe->SetResidency((surf >= 0 || norm >= 0) ? &resMgr : nullptr, surf, norm,
                                marsMode);
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
                }
            }
        }

        int mode = (opt.globeStart && globe) ? 3
                 : (opt.gulfStart && gulf)   ? 2
                 : (opt.seaStart && sea)     ? 1 : 0;
        auto applyMode = [&](int m) {
            sky->enabled = (m != 3);   // in space, space is the backdrop
            tide->enabled = (m == 0);
            if (sea) sea->enabled = (m == 1);
            if (terrain) terrain->enabled = (m == 1);
            if (gulf) gulf->enabled = (m == 2);
            if (globe) globe->enabled = (m == 3);
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
        if (mode == 1) cam = camSea;
        if (mode == 3) cam = camGlobe;

        // ---- M6b: the two frames JOINED. The estuary's flat world is the local tangent frame
        // at the ACT0816 origin; below ~3 km near home the estuary takes over, above ~4 km the
        // planet does. Both directions map position AND aim exactly, so crossing the seam is a
        // continuous camera move (hysteresis keeps it from flapping).
        double oDir[3], east0[3], north0[3];
        GlobeModel::LatLonDir(BathyModel::kOrgLat, BathyModel::kOrgLon, oDir);
        {
            const double yl = std::sqrt(oDir[0] * oDir[0] + oDir[2] * oDir[2]);
            east0[0] = -oDir[2] / yl; east0[1] = 0.0; east0[2] = oDir[0] / yl;   // y-hat x up
            north0[0] = oDir[1] * east0[2] - oDir[2] * east0[1];                 // up x east
            north0[1] = oDir[2] * east0[0] - oDir[0] * east0[2];
            north0[2] = oDir[0] * east0[1] - oDir[1] * east0[0];
        }
        auto planetToFlatPose = [&](const Camera& g) -> Camera {
            Camera f = g;
            const double p[3] = {g.px, g.py, g.pz};
            f.px = p[0] * east0[0] + p[1] * east0[1] + p[2] * east0[2];
            f.py = p[0] * oDir[0] + p[1] * oDir[1] + p[2] * oDir[2] - GlobeModel::kR;
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
            const double r = GlobeModel::kR + f.py;
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
        auto autoHandoff = [&]() {
            if (marsMode) return;   // the estuary is an Earth feature
            if (!(globe && sea && bathy.Ready())) return;
            if (mode == 3) {
                const Camera f = planetToFlatPose(cam);
                if (f.py < 3000.0 && std::abs(f.px) < 12000.0 && std::abs(f.pz) < 12000.0) {
                    camGlobe = cam;
                    cam = f;
                    cam.speed = 150.0f;
                    mode = 1;
                    applyMode(mode);
                }
            } else if (mode == 1 && cam.py > 4000.0 && globe) {
                camSea = cam;
                cam = flatToPlanetPose(cam);
                mode = 3;
                applyMode(mode);
            }
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
        if (globe && sea && bathy.Ready() && !marsMode) {
            const Motor k0 = poseMotor(camGlobe);
            Camera cMid;
            {
                double d[3];
                GlobeModel::LatLonDir(41.2, -66.5, d);
                const double r = GlobeModel::kR + 500000.0;
                cMid.px = d[0] * r; cMid.py = d[1] * r; cMid.pz = d[2] * r;
                cMid.LookAt(oDir[0] * GlobeModel::kR, oDir[1] * GlobeModel::kR,
                            oDir[2] * GlobeModel::kR);
            }
            Camera cHover;
            cHover.SetFromCompass(-200.0, 1800.0, -2500.0, 22.0f, -46.0f);
            Camera cHelm;
            cHelm.SetFromCompass(522.0, 7.0, 72.0, 246.0f, -4.0f);
            railKeys.push_back({0.0, k0});
            railKeys.push_back({3.0, poseMotor(cMid)});
            railKeys.push_back({5.0, poseMotor(flatToPlanetPose(cHover))});
            railKeys.push_back({10.0, poseMotor(flatToPlanetPose(cHover))});
            railKeys.push_back({15.0, poseMotor(flatToPlanetPose(cHelm))});
            railKeys.push_back({25.0, poseMotor(flatToPlanetPose(cHelm))});
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
        if (swe.Ready()) {
            Log("[swe] boundaries: ocean=%s, west=lerp(%s,%s,%.2f)",
                model.S(entSta).name.c_str(), model.S(westA).name.c_str(),
                model.S(westB).name.c_str(), wT);
        }

        // M5c: give the solver history before the first frame, and run the validation cycle if
        // asked (headless CSV; the ebb/flood-asymmetry and basin-lag gates read from it).
        if (swe.Ready()) {
            if (opt.sweCycleH > 0) {
                swe.Spinup(gpu, simUnix, 2.0, oceanAt, westAt, southAt);
                const int ctSta = haveCurrents ? currents.StationIndex("ACT0816") : -1;
                RunSweCycle(gpu, swe, oceanAt, westAt, southAt,
                            haveCurrents ? &currents : nullptr, ctSta, bathy, simUnix,
                            opt.sweCycleH);
                gpu.WaitIdle();
                resMgr.Shutdown();
                gpu.Shutdown();
                return 0;
            }
            if (opt.sweSpinupH > 0) {
                const auto t0 = std::chrono::steady_clock::now();
                swe.Spinup(gpu, simUnix, opt.sweSpinupH, oceanAt, westAt, southAt);
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
            if (mode == 1 && bathy.Ready()) {
                const float b = bathy.SampleWorld(static_cast<float>(x), static_cast<float>(z));
                return std::max(static_cast<double>(b > -9000.0f ? b : -30.0f), lastWaterNavd);
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
            double d[3];
            pixelRay(sxPx, syPx, d);
            const double b = cam.px * d[0] + cam.py * d[1] + cam.pz * d[2];
            const double c = cam.px * cam.px + cam.py * cam.py + cam.pz * cam.pz -
                             planetR * planetR;
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
            return (mode == 3) ? pickGlobe(sx, sy, out) : pickGround(sx, sy, out);
        };

        if (!opt.rail.empty()) {
            CreateDirectoryW(opt.rail.c_str(), nullptr);
            if (railKeys.empty()) {
                Log("FATAL: --rail needs globe + sea + bathy data all present");
                return 1;
            }
        }

        using Clock = std::chrono::steady_clock;
        auto last = Clock::now();
        auto lastTitle = last;
        uint32_t frame = 0;
        double frameMsSum = 0.0;
        uint32_t frameMsN = 0;

        for (;;) {
            if (!opt.headless) {
                window.NewFrame();
                if (!window.PumpMessages()) break;
                if (window.TakeResized()) renderer.OnResize(window.Width(), window.Height());
            }

            const auto now = Clock::now();
            float dt = std::chrono::duration<float>(now - last).count();
            last = now;
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

                    if (dragMode != 0 && (in.mouseDx != 0.0f || in.mouseDy != 0.0f)) {
                        constexpr double kOrbitRate = 0.006;   // rad per pixel
                        const double minAlt =
                            (mode == 3) ? -1.0e30 : groundAt(cam.px, cam.pz) + 1.2;
                        if (dragMode == 3) {          // ALT: rotate about the local vertical
                            double up[3] = {0, 1, 0};
                            if (mode == 3) {          // ...which on a planet is the radial line
                                const double pr = std::sqrt(dragPivot[0] * dragPivot[0] +
                                                            dragPivot[1] * dragPivot[1] +
                                                            dragPivot[2] * dragPivot[2]);
                                up[0] = dragPivot[0] / pr;
                                up[1] = dragPivot[1] / pr;
                                up[2] = dragPivot[2] / pr;
                            }
                            cam.OrbitAboutLine(dragPivot, up, in.mouseDx * kOrbitRate, minAlt);
                        } else if (dragMode == 2) {   // SHIFT: tilt about the horizontal line
                            const DirectX::XMFLOAT3 r = cam.Right();
                            const double ax[3] = {r.x, 0.0, r.z};
                            cam.OrbitAboutLine(dragPivot, ax, -in.mouseDy * kOrbitRate, minAlt);
                        } else if (mode == 3) {       // grab the GLOBE: one motor about the
                                                      // planet-centre line takes now -> grabbed
                            double nowPt[3];
                            if (pickGlobe(in.mouseX, in.mouseY, nowPt)) {
                                const double R = planetR;
                                double a[3] = {nowPt[0] / R, nowPt[1] / R, nowPt[2] / R};
                                double b[3] = {dragPivot[0] / R, dragPivot[1] / R,
                                               dragPivot[2] / R};
                                double ax[3] = {a[1] * b[2] - a[2] * b[1],
                                                a[2] * b[0] - a[0] * b[2],
                                                a[0] * b[1] - a[1] * b[0]};
                                const double s = std::sqrt(ax[0] * ax[0] + ax[1] * ax[1] +
                                                           ax[2] * ax[2]);
                                const double cdot = a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
                                if (s > 1e-9) {
                                    const double org[3] = {0, 0, 0};
                                    cam.OrbitAboutLine(org, ax,
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
                            cam.DollyToward(tgt, in.wheel * 0.18, mode == 3 ? 2500.0 : 3.0);
                            in.wheel = 0;   // consumed; Update() keeps its speed dial off it
                        }
                    }
                }

                cam.Update(in, dt);
                autoHandoff();   // M6b: descend into the estuary / climb back to the planet
                if (in.keyPressed['R']) renderer.ReloadShaders();
                if (in.keyPressed[VK_SPACE]) paused = !paused;
                if (in.keyPressed[VK_UP]) timeScale = std::min(timeScale * 10.0, 864000.0);
                if (in.keyPressed[VK_DOWN]) timeScale = std::max(timeScale / 10.0, 1.0);
                const double nudge = in.keyDown[VK_SHIFT] ? 86400.0 : 3600.0;
                if (in.keyPressed[VK_LEFT]) simUnix -= nudge;
                if (in.keyPressed[VK_RIGHT]) simUnix += nudge;
                if (in.keyPressed[VK_HOME] || in.keyPressed['N']) simUnix = NowUnix();
                if (in.keyPressed[VK_OEM_4]) windowSec = std::max(windowSec * 0.5, 0.5 * 86400.0);
                if (in.keyPressed[VK_OEM_6]) windowSec = std::min(windowSec * 2.0, 30.0 * 86400.0);
                if (in.keyPressed['C']) {   // cycle the ribbon's contour gauge
                    tide->contourStepM = (tide->contourStepM > 0.4f) ? 0.25f
                                       : (tide->contourStepM > 0.2f) ? 0.0f : 0.5f;
                }
                if (in.keyPressed['V']) {
                    if (mode == 3 && globe) globe->windOverlay = !globe->windOverlay;
                    else if (sea) sea->atlasVisualize = !sea->atlasVisualize;
                }
                if (in.keyPressed[VK_TAB] && (sea || gulf || globe)) {
                    int next = mode;
                    do {
                        next = (next + 1) % 4;
                    } while ((next == 1 && !sea) || (next == 2 && !gulf) ||
                             (next == 3 && !globe));
                    if (mode == 0) camChart = cam;
                    else if (mode == 1) camSea = cam;
                    else if (mode == 3) camGlobe = cam;
                    mode = next;
                    applyMode(mode);
                    if (mode == 0) cam = camChart;
                    else if (mode == 1) cam = camSea;
                    else if (mode == 3) cam = camGlobe;
                }
                if (!paused) simUnix += dt * timeScale;
            } else {
                // Deterministic time in headless mode so a dump sequence is reproducible.
                simUnix = startUnix + static_cast<double>(frame) * (timeScale / 30.0);
                if (!opt.rail.empty() && !railKeys.empty()) {
                    // The rails drive the PLANET-frame pose; the same handoff as interactive
                    // flight decides which world renders it. One unbroken shot -- the M6 gate.
                    if (mode != 3) {
                        mode = 3;
                        applyMode(mode);
                    }
                    railPose(static_cast<double>(frame) / 30.0, cam);
                    autoHandoff();
                }
            }

            // M6 globe housekeeping: fly speed and relief exaggeration scale with altitude,
            // and the camera never sinks beneath the (exaggerated) terrain.
            if (mode == 3 && globe) {
                const double r = std::sqrt(cam.px * cam.px + cam.py * cam.py + cam.pz * cam.pz);
                const double alt = r - planetR;
                cam.speed = static_cast<float>(std::clamp(alt * 0.45, 60.0, 2.5e6));
                globe->reliefExagg =
                    static_cast<float>(std::clamp(alt / 250000.0, 1.0, 20.0));
                const double r2d = 180.0 / 3.14159265358979;
                const double lat = std::asin(std::clamp(cam.py / r, -1.0, 1.0)) * r2d;
                const double lon = std::atan2(cam.pz, cam.px) * r2d;
                const double minR = planetR +
                                    (marsMode ? 0.0
                                              : (std::max)(0.0, globeModel.ElevAt(lat, lon)) *
                                                    globe->reliefExagg) +
                                    800.0;
                if (r < minR) {
                    const double s = minR / r;
                    cam.px *= s;
                    cam.py *= s;
                    cam.pz *= s;
                }
                const float viewH = opt.headless ? static_cast<float>(opt.height)
                                                 : static_cast<float>(
                                                       std::max(1u, window.Height()));
                const float aspect =
                    (opt.headless ? static_cast<float>(opt.width) : window.Width()) / viewH;
                globe->SetView(cam, aspect, viewH, simUnix - startUnix);
                // M6e screw-prefetch: extrapolate the pose ~0.8 s ahead along its own screw and
                // let the walk under THAT camera queue tiles early (predicted priority).
                Camera pred = cam;
                motorPose(resMgr.PredictNextPose(poseMotor(cam), 24.0), pred);
                globe->PredictWants(pred, aspect);
            }

            tide->SetTime(simUnix, windowSec);
            renderer.waterLevel = static_cast<float>(tide->focusHeight);
            // The terrain speaks NAVD88; the tide speaks MLLW. One offset joins them. In
            // estuary mode the open-water level is the ENTRANCE station's (M5c), and the west
            // boundary rides the interpolated river tide.
            const double waterNavd =
                bathy.Ready() ? oceanAt(simUnix) : tide->focusHeight + datumOff;
            lastWaterNavd = waterNavd;   // next frame's camera-pivot rays test against it
            if (swe.Ready()) {
                swe.SetBoundaries(static_cast<float>(westAt(simUnix)),
                                  static_cast<float>(southAt(simUnix)));
            }
            if (sea) sea->SetTime(simUnix, bathy.Ready() ? waterNavd : tide->focusHeight,
                                  cam.px, cam.pz);
            if (terrain) terrain->waterNavd = static_cast<float>(waterNavd);

            renderer.RenderFrame(cam, static_cast<float>(simUnix - startUnix), dt);

            if (!opt.rail.empty()) {
                wchar_t rp[512];
                swprintf(rp, 512, L"%s\\rail_%04u.png", opt.rail.c_str(), frame);
                renderer.DumpPng(rp);
            }

            if (!opt.headless &&
                std::chrono::duration<float>(now - lastTitle).count() > 0.25f) {
                lastTitle = now;
                wchar_t title[512];
                FormatTitle(title, 512, simUnix, timeScale, paused, model, *tide,
                            windowSec / 86400.0, sea, seaState,
                            gulf ? gulf->validation.c_str() : nullptr,
                            globe ? globe->stats.c_str() : nullptr, mode);
                window.SetTitle(title);
            }

            if (frame >= 10) {   // skip warm-up: PSO/upload stalls are not frame cost
                frameMsSum += dt * 1000.0;
                ++frameMsN;
            }
            ++frame;
            if (opt.frames && frame >= opt.frames) break;
        }
        if (frameMsN > 30) {
            Log("[perf] mean frame %.2f ms over %u frames (%.0f fps)", frameMsSum / frameMsN,
                frameMsN, 1000.0 / (frameMsSum / frameMsN));
        }

        if (opt.seaVerify && sea) {
            const double hr = sea->MeasureRenderedHs(gpu);
            Log("[verify] sea: model Hs %.3f m, RENDERED Hs %.3f m "
                "(one realization; agreement within ~10%% passes)", sea->hsModel, hr);
        }
        if (!opt.dump.empty()) renderer.DumpPng(opt.dump);

        gpu.WaitIdle();
        resMgr.Shutdown();
        renderer.Shutdown();
        gpu.Shutdown();
        window.Destroy();
        Log("done (%u frames)", frame);
        return 0;
    } catch (const std::exception& e) {
        Log("FATAL: %s", e.what());
        return 1;
    }
}
