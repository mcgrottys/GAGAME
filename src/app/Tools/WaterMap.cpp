// WaterMap - --water-map / --bathy-map.
// Moved verbatim from main.cpp (M12 step 1b); declared in app/Tools.h.
#include "app/Tools.h"

#include "compose/Compositor.h"
#include "compose/Projections.h"
#include "compose/VectorPack.h"
#include "compose/WaterAtlas.h"
#include "core/Common.h"
#include "core/Gpu.h"
#include "core/Image.h"
#include "core/Json.h"
#include "sim/GlobeModel.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iterator>
#include <string>
#include <vector>

namespace ga::app::tools {

namespace {

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

}  // namespace

int RunWaterMap(const Options& opt, Gpu& gpu, const GlobeModel& globeModel,
                Compositor& compositor, int hgtCh, const WaterAtlas& waterAtlas) {
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

}  // namespace ga::app::tools
