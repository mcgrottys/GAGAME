// ================================================================================================
//  ComposeTest - M6j: the compositor's --selftest gate, same discipline as tiletest/atlastest/
//  pgatest. The manager wants to be TRUSTED as the single interface between data and everything
//  else (renderer, physics, exports, plugins) -- so its contracts are pinned by assertions, not
//  by "the last render looked right":
//    1. stack ORDER: the top layer wins where it has full weight
//    2. per-PIXEL weights: partial coverage lerps; uncovered texels keep what is beneath
//    3. alpha: texels no source covered carry a=0 (the shader shows the layer below)
//    4. transient failures are NEVER cached (holes this run, repainted next)
//    5. the composed cache round-trips byte-identical
//    6. every realization reproduces an analytic source at its own texel centres (cube AND
//       window addressing verified against closed-form lat/lon)
//    7. reordering a stack changes its cache tag (stale tiles are unreachable)
// ================================================================================================
#include "compose/Compositor.h"
#include "compose/Projections.h"

#include <cmath>
#include <cstdio>
#include <cstring>

namespace ga {

namespace {

constexpr double kPi = 3.14159265358979;

bool g_ok = true;
void Check(bool cond, const char* what) {
    if (!cond) {
        Log("[composetest] FAIL: %s", what);
        g_ok = false;
    }
}

// An analytic color: R encodes longitude, G latitude -- any addressing mistake in a
// realization shows up as a wrong closed-form value at a texel centre.
class GradientColor : public ColorSource {
public:
    GradientColor() {
        m_info = {"test.gradient", "analytic", "EPSG:4326", 1, -180, -90, 180, 90};
    }
    const SourceInfo& Info() const override { return m_info; }
    float Sample(double lat, double lon, double, const PaintCtx&, uint8_t rgba[4]) override {
        rgba[0] = static_cast<uint8_t>((lon / (2.0 * kPi) + 0.5) * 255.0);
        rgba[1] = static_cast<uint8_t>((lat / kPi + 0.5) * 255.0);
        rgba[2] = 7;
        rgba[3] = 255;
        return 1.0f;
    }
    SourceInfo m_info;
};

// A half-weight overlay covering only the eastern hemisphere; transient in a band.
class OverlayColor : public ColorSource {
public:
    OverlayColor() { m_info = {"test.overlay", "analytic", "EPSG:4326", 1, 0, -90, 180, 90}; }
    const SourceInfo& Info() const override { return m_info; }
    float Sample(double lat, double lon, double, const PaintCtx&, uint8_t rgba[4]) override {
        if (transientBand && lat > 0.5 && lat < 0.6) return -1.0f;
        if (lon < 0.0) return 0.0f;
        rgba[0] = 200;
        rgba[1] = 100;
        rgba[2] = 50;
        rgba[3] = 255;
        return 0.5f;
    }
    bool transientBand = false;
    SourceInfo m_info;
};

class RampHeight : public HeightSource {
public:
    RampHeight() { m_info = {"test.ramp", "analytic", "EPSG:4326", 1, -180, -90, 180, 90}; }
    const SourceInfo& Info() const override { return m_info; }
    float Sample(double lat, double, double, float& m) override {
        m = static_cast<float>(lat * 1000.0);
        return 1.0f;
    }
    SourceInfo m_info;
};

}  // namespace

bool RunComposeSelfTest() {
    g_ok = true;
    Log("[composetest] ---- M6j gate: the layer compositor's contracts ----");

    GradientColor base;
    OverlayColor over;
    RampHeight ramp;

    Compositor comp;
    const int col = comp.AddColorChannel("selftest.color", {&base, &over});
    const int hgt = comp.AddHeightChannel("selftest.height", {&ramp});

    // ---- cube realization: analytic reproduction + ordering + per-pixel weights + alpha
    {
        auto fn = comp.CubeColor(col);
        TileRequest r{0, 3, 5, 9};   // face +x, an arbitrary interior tile
        std::vector<uint8_t> tile;
        Check(fn(r, tile) && tile.size() == 65536, "cube paint returns a 64KB tile");
        const uint32_t faceTexels = Compositor::kFaceDim >> r.mip;
        int checked = 0;
        for (uint32_t py = 10; py < 128; py += 37) {
            for (uint32_t px = 5; px < 128; px += 31) {
                const double u = (r.x * 128.0 + px + 0.5) / faceTexels;
                const double v = (r.y * 128.0 + py + 0.5) / faceTexels;
                double d[3];
                ComposeCubeDir(r.face, u, v, d);
                const double lat = std::asin((std::max)(-1.0, (std::min)(1.0, d[1])));
                const double lon = std::atan2(d[2], d[0]);
                const uint8_t* p = &tile[(py * 128 + px) * 4];
                uint8_t g[4];
                PaintCtx ctx;
                base.Sample(lat, lon, 0, ctx, g);
                double expR = g[0], expG = g[1];
                if (lon >= 0.0) {   // overlay lerps in at weight 0.5
                    expR = expR + (200.0 - expR) * 0.5;
                    expG = expG + (100.0 - expG) * 0.5;
                }
                Check(std::abs(p[0] - expR) <= 1.5 && std::abs(p[1] - expG) <= 1.5,
                      "cube texel matches the analytic stack (order + weights)");
                Check(p[3] == 255, "covered texel has alpha 255");
                ++checked;
            }
        }
        Check(checked > 8, "cube spot-checks ran");
        // cache round-trip: byte identity, no repaint
        const uint32_t paintedBefore = comp.painted.load();
        std::vector<uint8_t> tile2;
        Check(fn(r, tile2), "cube cache read");
        Check(comp.painted.load() == paintedBefore, "second call did not repaint");
        Check(comp.cacheHits.load() >= 1 && tile2 == tile, "composed cache is byte-identical");
    }

    // ---- window realization: analytic reproduction in Mercator addressing
    {
        auto fn = comp.WindowColor(col, 1263360, 1538048, 16384, 14);
        TileRequest r{0, 2, 7, 11};
        std::vector<uint8_t> tile;
        Check(fn(r, tile) && tile.size() == 65536, "window paint returns a 64KB tile");
        const double worldPx = static_cast<double>((1ll << 14) * 256ll >> r.mip);
        int checked = 0;
        for (uint32_t py = 3; py < 128; py += 41) {
            for (uint32_t px = 9; px < 128; px += 29) {
                const double Y = ((1538048ll >> r.mip) + r.y * 128 + py + 0.5) / worldPx;
                const double lat = std::atan(std::sinh(kPi * (1.0 - 2.0 * Y)));
                const double X = ((1263360ll >> r.mip) + r.x * 128 + px + 0.5) / worldPx;
                const double lon = (X - 0.5) * 2.0 * kPi;
                const uint8_t* p = &tile[(py * 128 + px) * 4];
                const double expR0 = (lon / (2.0 * kPi) + 0.5) * 255.0;
                const double expG0 = (lat / kPi + 0.5) * 255.0;
                const double expR = lon >= 0 ? expR0 + (200 - expR0) * 0.5 : expR0;
                const double expG = lon >= 0 ? expG0 + (100 - expG0) * 0.5 : expG0;
                Check(std::abs(p[0] - expR) <= 1.5 && std::abs(p[1] - expG) <= 1.5,
                      "window texel matches the analytic stack in Mercator addressing");
                ++checked;
            }
        }
        Check(checked > 8, "window spot-checks ran");
    }

    // ---- transient failures: usable this run, NEVER cached
    {
        over.transientBand = true;
        Compositor comp2;
        const int c2 = comp2.AddColorChannel("selftest.transient", {&base, &over});
        auto fn = comp2.CubeColor(c2);
        // Face +x, mip 5, top-row tile: covers ~24..35 deg latitude, straddling the
        // 0.5..0.6 rad (28.6..34.4 deg) transient band.
        TileRequest r{0, 5, 1, 0};
        std::vector<uint8_t> tile;
        Check(fn(r, tile), "transient tile still paints (holes, not failure)");
        Check(comp2.painted.load() == 0, "transient tile was NOT counted as painted");
        std::vector<uint8_t> tile2;
        fn(r, tile2);
        Check(comp2.cacheHits.load() == 0, "transient tile was NOT served from cache");
        over.transientBand = false;
    }

    // ---- height realization: analytic ramp through FloatToHalf
    {
        auto fn = comp.CubeHeight(hgt);
        TileRequest r{0, 4, 2, 3};
        std::vector<uint8_t> tile;
        Check(fn(r, tile), "height paint");
        const uint16_t* h16 = reinterpret_cast<const uint16_t*>(tile.data());
        const uint32_t faceTexels = Compositor::kFaceDim >> r.mip;
        int checked = 0;
        for (uint32_t py = 7; py < 128; py += 43) {
            for (uint32_t px = 11; px < 256; px += 61) {
                const double u = (r.x * 256.0 + px + 0.5) / faceTexels;
                const double v = (r.y * 128.0 + py + 0.5) / faceTexels;
                double d[3];
                ComposeCubeDir(r.face, u, v, d);
                const double lat = std::asin((std::max)(-1.0, (std::min)(1.0, d[1])));
                const float got = HalfToFloat(h16[py * 256 + px]);
                const float exp = static_cast<float>(lat * 1000.0);
                Check(std::abs(got - exp) <= (std::max)(0.5f, std::abs(exp) * 0.002f),
                      "height texel matches the analytic ramp (half precision)");
                ++checked;
            }
        }
        Check(checked > 6, "height spot-checks ran");
    }

    // ---- stack identity: reordering the SAME sources composes differently AND lands in a
    // different cache tag (if either failed -- e.g. the reordered fn read the other order's
    // cache -- the tiles would come back byte-identical).
    {
        Compositor comp3;
        const int a = comp3.AddColorChannel("selftest.hash", {&base, &over});
        const int b = comp3.AddColorChannel("selftest.hash", {&over, &base});
        auto fa = comp3.CubeColor(a);
        auto fb = comp3.CubeColor(b);
        TileRequest r{0, 6, 0, 0};
        std::vector<uint8_t> ta, tb;
        fa(r, ta);
        fb(r, tb);
        Check(ta != tb, "stack ORDER changes the composition (and cache tags stay isolated)");
    }

    // ---- projections: the alignment contract's math, pinned at the ACT0816 anchor against
    // independently computed references (python, Snyder). A layer that declares EPSG:6348 or
    // EPSG:26986 aligns because THESE numbers are right, not because a render looked right.
    {
        const double lat = 42.81833 * kPi / 180.0, lon = -70.81 * kPi / 180.0;
        double e = 0, n = 0;
        TransverseMercator::Utm(19).Forward(lat, lon, e, n);
        Check(std::abs(e - 352034.1) < 0.5 && std::abs(n - 4742229.8) < 0.5,
              "UTM 19N forward matches the reference at the anchor");
        double x = 0, y = 0;
        LambertConformalConic::MassMainland().Forward(lat, lon, x, y);
        Check(std::abs(x - 256429.4) < 0.5 && std::abs(y - 952196.8) < 0.5,
              "MA State Plane LCC forward matches the reference at the anchor");
    }

    if (g_ok) {
        Log("[composetest] ---- PASS: paint order, weights, alpha, cache, addressing, "
            "projections ----");
    }
    return g_ok;
}

}  // namespace ga
