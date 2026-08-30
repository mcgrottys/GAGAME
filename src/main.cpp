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
#include "scene/GisLayer.h"
#include "scene/GlobeLayer.h"
#include "scene/MarkerLayer.h"
#include "scene/GulfLayer.h"
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
#include "core/Pga.h"
#include "core/TileProviders.h"
#include "sim/BathyModel.h"
#include "sim/GlobeModel.h"
#include "sim/CurrentModel.h"
#include "sim/SeaState.h"
#include "sim/SweSolver.h"
#include "sim/TideModel.h"
#include "sim/WeatherManager.h"

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
    bool warmInlet = false;           // --warm-inlet: pre-cache the Merrimack detail pyramid
    bool railZoom = false;            // --rail-zoom DIR: orbit -> inlet imagery zoom -> estuary
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
    bool stencil = false;             // M6i --stencil: coast/graticule alignment overlay
    bool msSurface = true;            // M6j: mesh-shader planet surface (--no-ms falls back)
    bool albedo = false;              // M6j: raw-texture lens (no lighting/atmosphere)
    std::string exportSpec;           // M6j --export: composed-channel export spec
    std::wstring exportOut;
    float camAlt = -1, camAz = 246, camPitch = -5;   // --cam alt,az,pitch override
    float camX = 1e9f, camZ = 1e9f;   // --campos x,z world override (sea mode)
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
                                      // channel bathymetry (M6w: the chart IS the stack)
    std::string oceanProbe;           // --ocean-probe lat,lon: the weather manager's
                                      // verification harness (rungs + provenance + gates)
    bool oneWater = false;            // --one-water: M7 -- water geometry from the wave
                                      // vertex bank alone (SeaLayer's grid retires)
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
        else if (a == "--rail-zoom") { o.rail = Widen(next("rail_frames").c_str()); o.railZoom = true; }
        else if (a == "--rail-flood") { o.rail = Widen(next("rail_frames").c_str()); o.railFlood = true; }
        else if (a == "--rail-jetty") { o.rail = Widen(next("rail_frames").c_str()); o.railJetty = true; }
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
        else if (a == "--ocean-probe") o.oceanProbe = next("42.35,-70.65");
        else if (a == "--one-water") o.oneWater = true;
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
        // The rails demos: headless, deterministic real-time waves, 30 fps. Classic = 25 s;
        // the zoom and Mars flyover run 30 s; the flood ride holds the helm for 40 s total.
        o.headless = true;
        o.globeStart = true;
        o.frames = o.railJetty                             ? 40 * 30
                   : o.railFlood                           ? 40 * 30
                   : (o.railZoom || o.planet == "mars")    ? 30 * 30
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
            if (!fn({face, mip, tx, ty}, tile) || tile.size() != 65536) continue;
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
    try {
        const Options opt = ParseArgs(argc, argv);

        // ---- M0 + M4: the self-test path needs a device and the shader compiler, nothing else.
        if (opt.selftest) {
            Gpu gpu;
            gpu.Init(nullptr, 64, 64, opt.debugLayer);
            ShaderCompiler sc;
            sc.Init();
            bool ok = RunPgaSelfTest();   // pure CPU: the motor conventions, pinned first
            ok &= RunComposeSelfTest();   // pure CPU: the layer compositor's contracts
            ok &= RunWaterSelfTest();     // pure CPU: the water atlas' datum/epoch/field gates
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
        EquirectHeightSource srcEtopo("noaa.etopo2022", "equirect-grid int16 8192x4096",
                                      489200.0, &globeModel.Elev(), globeModel.Nx(),
                                      globeModel.Ny());
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
                    globeModel.NeDLat());
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

        // M6v: THE WATER ATLAS -- water parameters through the same registry. Five phasor
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
        double riverQ = 70.0;
        if (terrain && sea && !opt.sweOff) {
            swe.Init(gpu, renderer.Shaders(), opt.shaderDir, bathy, terrain->HeightTex().res.Get());
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
        WaterBankLayer* waterBank = nullptr;
        if (sea && bathy.Ready() && !marsMode) {
            auto wbOwned = std::make_unique<WaterBankLayer>();
            waterBank = wbOwned.get();
            waterBank->Configure(opt.shaderDir, sea, &swe, &bathy, &waterAtlas, &compositor,
                                 hgtCh, &globeModel, &seaState);
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
                                        // node; its program is data/bed/bed_rules.json
        AerialOrthoSource srcAerial;    // M6l: MassGIS 15 cm orthos (loads if harvested)
        AerialOrthoSource srcOverlay;   // M6o: user GeoTIFF overlays -- ALPHA IS FIBER: a
                                        // mostly-transparent highlights plane bleeds through
                                        // the composed quadtree pixel by pixel
        GisStencil gisStencil;   // survey vectors + mask realizations (GSHHG/WDBII)
        VectorPack vectors;      // M6p: lossless vector layers, LOD by wedge importance
        GisLayer* gisLayer = nullptr;

        Exchange exchange;       // M6j: the plugin bus -- named GA buffer channels
        const double winOrgX = 4935.0 * 256.0, winOrgY = 6008.0 * 256.0;   // Merrimack z14 px
        int colorCubeT = -1, winTenant = -1, hgtTenant = -1, hgtWinTenant = -1;
        int detTenant = -1;   // M7f: z17 detail color window
        double det17OrgX = 0.0, det17OrgY = 0.0;
        int colCh = -1;   // color channel id (hgtCh registered above the solver, M6w)
        if (globe) {
            resMgr.Init(gpu);
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
                hgtTenant = resMgr.AddTextureCube(gpu, L"earth.height (composed)",
                                                  Compositor::kFaceDim, DXGI_FORMAT_R16_FLOAT,
                                                  compositor.CubeHeight(hgtCh));
                // The height WINDOW: the same Mercator frame as the color window, so the
                // globe's near-field land/sea gate and normals ride CUDEM truth.
                hgtWinTenant = resMgr.AddTexture2D(
                    gpu, L"earth.height.window (composed, Merrimack z14)",
                    Compositor::kFaceDim, DXGI_FORMAT_R16_FLOAT,
                    compositor.WindowHeight(hgtCh, 1263360, 1538048, 16384, 14));
                // earth.color: the Google mercator tree, realized twice -- the global cube
                // and the Merrimack z14 window (same stack, deeper footprint).
                if (googleTiles.Init("satellite", opt.tileBudget)) {
                    googleTiles.SetFetchCounter(&resMgr.fetchesThisRun);
                    // M6l: the MassGIS 15 cm plane orthos paint ABOVE Google wherever they
                    // have coverage -- the compositor's first independent high-res layer,
                    // aligned by its own declared projection (EPSG:6348), not by luck.
                    std::vector<ColorSource*> colorStack{&srcGoogle};
                    // The bed paints ABOVE google (its photo of open water) and BELOW the
                    // surveyed orthos: stack order is the authority ranking.
                    if (srcBed.Load("data/bed/bed_rules.json", &compositor, hgtCh)) {
                        colorStack.push_back(&srcBed);
                    }
                    if (srcAerial.Load("data/aerial/aerial.json")) {
                        colorStack.push_back(&srcAerial);
                    }
                    if (srcOverlay.Load("data/overlay/overlay.json")) {
                        colorStack.push_back(&srcOverlay);
                    }
                    colCh = compositor.AddColorChannel("earth.color", std::move(colorStack));
                    colorCubeT = resMgr.AddTextureCube(gpu, L"earth.color (composed)",
                                                       Compositor::kFaceDim,
                                                       DXGI_FORMAT_R8G8B8A8_UNORM_SRGB,
                                                       compositor.CubeColor(colCh));
                    winTenant = resMgr.AddTexture2D(
                        gpu, L"earth.color.window (composed, Merrimack z14)",
                        Compositor::kFaceDim, DXGI_FORMAT_R8G8B8A8_UNORM_SRGB,
                        compositor.WindowColor(colCh, 1263360, 1538048, 16384, 14));
                    // M7f: the z17 DETAIL window -- 1.2 m px, ~14 km centred on the inlet
                    // mouth: the near field stops being capped at the z14 window's 9.5 m.
                    // Same channel, same stack (massgis where harvested, google's own z17
                    // ladder elsewhere), the compose ladder's third rung.
                    {
                        const double n17 = 16384.0 * 256.0 * 8.0;
                        const double piD = 3.14159265358979;
                        const double lonC = -70.8125, latC = 42.8160 * piD / 180.0;
                        const double mx = (lonC + 180.0) / 360.0 * n17;
                        const double my =
                            (0.5 - std::log(std::tan(piD * 0.25 + latC * 0.5)) /
                                       (2.0 * piD)) *
                            n17;
                        det17OrgX = std::floor(mx - 8192.0);
                        det17OrgY = std::floor(my - 8192.0);
                        detTenant = resMgr.AddTexture2D(
                            gpu, L"earth.color.detail (composed, Merrimack z17)",
                            Compositor::kFaceDim, DXGI_FORMAT_R8G8B8A8_UNORM_SRGB,
                            compositor.WindowColor(
                                colCh, static_cast<long long>(det17OrgX),
                                static_cast<long long>(det17OrgY), 16384, 17));
                    }
                }
                globe->SetComposed(colorCubeT, winTenant, hgtTenant, hgtWinTenant, winOrgX,
                                   winOrgY, 16384.0, detTenant, det17OrgX, det17OrgY);
            }
            globe->stencilOverlay = opt.stencil;
            compositor.LogRegistry();
            // The survey pack loads whenever it exists: the land MASKS are the default
            // classifier (always on); the VECTOR overlay draws only under --stencil.
            if (!marsMode && gisStencil.Load("data/gis/gis.json")) {
                gisStencil.BuildMasks(gpu, winOrgX, winOrgY, 16384.0);
                globe->SetGisStencil(gisStencil.MaskWinSrv(), gisStencil.MaskGlobSrv(),
                                     gisStencil.MaskEditSrv(), gisStencil.EditBox());
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
            FillComposedCb(cs, &resMgr, colorCubeT, winTenant, hgtTenant, hgtWinTenant,
                           winOrgX, winOrgY, 16384.0, 14, planetR, east0, oDir, north0,
                           opt.stencil, gisStencil.MaskWinSrv(), gisStencil.MaskGlobSrv());
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
                resMgr.Want(hgtWinTenant, 0, 2, 0, 0, 1, 1);
                for (const auto& w : rings) {
                    resMgr.Want(hgtWinTenant, 0, w.mip, w.a, w.a, w.b, w.b);
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
                if (!paused) simUnix += dt * timeScale;
            } else {
                // Deterministic time in headless mode so a dump sequence is reproducible.
                simUnix = startUnix + static_cast<double>(frame) * (timeScale / 30.0);
                if (!opt.rail.empty() && !railKeys.empty()) {
                    // M6g: the rails just set a pose in the ONE frame. Nothing switches.
                    railPose(static_cast<double>(frame) / 30.0, cam);
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
                // is the demand signal; dormant windows spin up as it arrives, mirrors
                // refresh, owned solvers advance. All in the flat one-world frame.
                if (!marsMode) {
                    weather.Update(gpu, renderer.Shaders(), opt.shaderDir, simUnix,
                                   BathyModel::kOrgLat + cam.pz / BathyModel::kMPerLat,
                                   BathyModel::kOrgLon + cam.px / BathyModel::kMPerLon,
                                   altV);
                }
                // M7: the bank's rings follow the camera; the globe binds THIS frame's
                // origins (residency committed in SetFrame, content recomposed in Render).
                if (waterBank && globe) {
                    waterBank->SetFrame(gpu, simUnix, cam.px, cam.pz);
                    float orgs[12];
                    for (int mR = 0; mR < WaterBankLayer::kMips; ++mR) {
                        waterBank->RingOrigin(mR, orgs[mR * 2], orgs[mR * 2 + 1]);
                    }
                    uint32_t derivS[3];
                    float patchS[3], bandKS[3];
                    const double kPiB = 3.14159265358979;
                    const double kCutB[4] = {2.0 * kPiB / 756.0, 2.0 * kPiB / 60.0,
                                             2.0 * kPiB / 12.0, 0.9 * kPiB * 256.0 / 47.0};
                    for (int c = 0; c < 3; ++c) {
                        derivS[c] = sea->FftDerivSrv(c);
                        patchS[c] = sea->FftPatchL(c);
                        bandKS[c] = static_cast<float>(std::sqrt(kCutB[c] * kCutB[c + 1]));
                    }
                    globe->SetWaterBank(waterBank->DispSrv(), waterBank->ParamSrv(),
                                        waterBank->DetailSrv(), derivS, patchS, bandKS,
                                        sea->heightScale, waterBank->BaseTexelM(), orgs,
                                        opt.oneWater);
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
                const double g = groundAt(cam.px, cam.pz);
                if (cam.py < g + 1.2) cam.py = g + 1.2;
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
                                  static_cast<float>(southAt(simUnix)),
                                  static_cast<float>(westQAt(simUnix)));
            }
            if (sea) sea->SetTime(simUnix, bathy.Ready() ? waterNavd : tide->focusHeight,
                                  cam.px, cam.pz);
            if (terrain) terrain->waterNavd = static_cast<float>(waterNavd);
            if (globe) globe->waterNavd = static_cast<float>(waterNavd);   // M6j materials

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
                            globe ? globe->stats.c_str() : nullptr,
                            (mode == 1 && altV > 60000.0) ? 3 : mode);   // title by altitude
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
