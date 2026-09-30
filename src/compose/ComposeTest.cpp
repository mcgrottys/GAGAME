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
#include "compose/Sources.h"
#include "compose/VectorPack.h"

#include <fstream>

#include <algorithm>
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
        Check(fn(r, tile, nullptr) && tile.size() == 65536, "cube paint returns a 64KB tile");
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
        Check(fn(r, tile2, nullptr), "cube cache read");
        Check(comp.painted.load() == paintedBefore, "second call did not repaint");
        Check(comp.cacheHits.load() >= 1 && tile2 == tile, "composed cache is byte-identical");
    }

    // ---- window realization: analytic reproduction in Mercator addressing
    {
        auto fn = comp.WindowColor(col, 1263360, 1538048, 16384, 14);
        TileRequest r{0, 2, 7, 11};
        std::vector<uint8_t> tile;
        Check(fn(r, tile, nullptr) && tile.size() == 65536, "window paint returns a 64KB tile");
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
        Check(fn(r, tile, nullptr), "transient tile still paints (holes, not failure)");
        Check(comp2.painted.load() == 0, "transient tile was NOT counted as painted");
        std::vector<uint8_t> tile2;
        fn(r, tile2, nullptr);
        Check(comp2.cacheHits.load() == 0, "transient tile was NOT served from cache");
        over.transientBand = false;
    }

    // ---- height realization: analytic ramp through FloatToHalf
    {
        auto fn = comp.CubeHeight(hgt);
        TileRequest r{0, 4, 2, 3};
        std::vector<uint8_t> tile;
        Check(fn(r, tile, nullptr), "height paint");
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

    // ---- THE SURVEY ORDER: one lossless vector structure serves every LOD -- the wedge
    // filter (importance >= tolerance) reproduces decimation without precomputed levels,
    // and tolerance 0 returns the geometry bit for bit.
    {
        struct V { float lon, lat, imp; };
        const V verts[5] = {{0, 0, 1e9f}, {0.1f, 0.01f, 5.0f}, {0.2f, -0.02f, 50.0f},
                            {0.3f, 0.01f, 500.0f}, {0.4f, 0, 1e9f}};
        std::vector<uint8_t> pack;
        auto put = [&](const void* p, size_t n) {
            pack.insert(pack.end(), static_cast<const uint8_t*>(p),
                        static_cast<const uint8_t*>(p) + n);
        };
        put("VPK1", 4);
        const uint32_t one = 1, five = 5;
        put(&one, 4);
        const uint16_t nameLen = 4;
        put(&nameLen, 2);
        put("test", 4);
        const uint8_t kind = 0, flags = 0;
        put(&kind, 1);
        put(&flags, 1);
        put(&one, 4);
        put(&five, 4);
        put(verts, sizeof(verts));
        const char* path = "cache\\composed\\selftest.vpack";
        std::ofstream(path, std::ios::binary)
            .write(reinterpret_cast<const char*>(pack.data()), pack.size());
        VectorPack vp;
        Check(vp.Load(path) && vp.Find("test"), "vpack round-trip");
        if (const VectorPack::Layer* L = vp.Find("test")) {
            Check(vp.Segments(*L, 0.0f).size() == 4 * 4, "tolerance 0 is LOSSLESS");
            Check(vp.Segments(*L, 10.0f).size() == 3 * 4,
                  "tol 10 m drops exactly the 5 m vertex");
            Check(vp.Segments(*L, 1000.0f).size() == 1 * 4,
                  "tol 1 km keeps only the endpoints");
        }
    }

    // ---- ALPHA IS FIBER: a source whose per-pixel alpha VARIES across the tile must land
    // in the composition as a per-pixel weight -- a translucent highlight bleeds through
    // exactly as much as its alpha says, texel by texel, never tile by tile.
    {
        class AlphaRamp : public ColorSource {
        public:
            AlphaRamp() { m_info = {"test.ramp", "analytic", "EPSG:4326", 1, -180, -90, 180, 90}; }
            const SourceInfo& Info() const override { return m_info; }
            float Sample(double lat, double, double, const PaintCtx&,
                         uint8_t rgba[4]) override {
                rgba[0] = 200; rgba[1] = 0; rgba[2] = 0; rgba[3] = 255;
                // alpha ramps north-south: 0 at lat 0 to 1 at lat 1 rad
                return static_cast<float>(std::clamp(lat, 0.0, 1.0));
            }
            SourceInfo m_info;
        } aramp;
        Compositor comp5;
        const int c5 = comp5.AddColorChannel("selftest.alpha", {&base, &aramp});
        auto fn = comp5.CubeColor(c5);
        TileRequest r{0, 5, 1, 0};   // face +x top row: lats ~0.42..0.62 rad, ramp mid-band
        std::vector<uint8_t> tile;
        Check(fn(r, tile, nullptr), "alpha-ramp paint");
        const uint32_t faceTexels = Compositor::kFaceDim >> r.mip;
        int checked = 0;
        for (uint32_t py = 5; py < 128; py += 39) {
            for (uint32_t px = 7; px < 128; px += 33) {
                const double u = (r.x * 128.0 + px + 0.5) / faceTexels;
                const double v = (r.y * 128.0 + py + 0.5) / faceTexels;
                double d[3];
                ComposeCubeDir(r.face, u, v, d);
                const double lat = std::asin((std::max)(-1.0, (std::min)(1.0, d[1])));
                const double lon = std::atan2(d[2], d[0]);
                const double a = std::clamp(lat, 0.0, 1.0);
                const double baseR = (lon / (2.0 * kPi) + 0.5) * 255.0;
                const double expR = baseR + (200.0 - baseR) * a;
                Check(std::abs(tile[(py * 128 + px) * 4] - expR) <= 1.5,
                      "per-pixel alpha weights the paint texel by texel");
                ++checked;
            }
        }
        Check(checked > 8, "alpha-ramp spot-checks ran");
    }

    // ---- the soak rule's SHORT CIRCUIT: a source whose footprint spans less than ~2 texels
    // at a LOD drops out of that tile's subset -- the tile's cache identity REVERTS to the
    // without-it tag, so the "repaint" is a byte-identical cache hit. Painting that cannot
    // change more than ~a pixel never runs (the user's redundancy rule, as a contract).
    {
        class SpeckSource : public ColorSource {
        public:
            SpeckSource() {
                m_info = {"test.speck", "analytic", "EPSG:4326", 1,
                          -70.001, 42.0, -70.0, 42.001};   // ~100 m footprint
            }
            const SourceInfo& Info() const override { return m_info; }
            float Sample(double, double, double, const PaintCtx&, uint8_t rgba[4]) override {
                rgba[0] = rgba[1] = rgba[2] = rgba[3] = 255;
                return 1.0f;
            }
            SourceInfo m_info;
        } speck;
        Compositor comp4;
        const int without = comp4.AddColorChannel("selftest.soak", {&base});
        const int with = comp4.AddColorChannel("selftest.soak", {&base, &speck});
        auto fw = comp4.CubeColor(without);
        auto fs = comp4.CubeColor(with);
        // A COARSE tile covering the speck (face -z holds lon -70; mip 6: ~40 km texels):
        // the speck spans far under 2 texels -> excluded -> ONE shared cache identity.
        TileRequest coarse{5, 6, 0, 0};
        std::vector<uint8_t> ta, tb;
        fw(coarse, ta, nullptr);
        const uint32_t hitsBefore = comp4.cacheHits.load();
        fs(coarse, tb, nullptr);
        Check(comp4.cacheHits.load() == hitsBefore + 1 && ta == tb,
              "sub-texel source short-circuits into the without-it cache identity");
        // A FINE tile over the speck (mip 0: ~600 m texels... still > 2 texels? the speck is
        // ~0.001 deg ~ 2 texels of mip 0's ~0.0005-deg texels): it must now BE in the subset
        // and actually paint white where it covers.
        // Locate a mip-0 tile containing lon -70.0005, lat 42.0005 on face +x.
        // dir for (lat,lon): the +x face spans lon -45..45 -- -70 is NOT on face +x; use the
        // subset hash difference instead, realization-agnostic:
        Compositor::TileBox fine{42.0 * 3.14159265358979 / 180.0,
                                 42.001 * 3.14159265358979 / 180.0,
                                 -70.001 * 3.14159265358979 / 180.0,
                                 -70.0 * 3.14159265358979 / 180.0,
                                 8e-6, 8e-6};   // ~0.5-texel spans -> speck covers >= 2 texels
        std::vector<size_t> incA, incB;
        const uint64_t hA = comp4.ColorSubset(comp4.ChannelAt(without), fine, incA);
        const uint64_t hB = comp4.ColorSubset(comp4.ChannelAt(with), fine, incB);
        Check(hA != hB && incB.size() == incA.size() + 1,
              "the same source RE-ENTERS the subset at a LOD fine enough to see it");
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
        fa(r, ta, nullptr);
        fb(r, tb, nullptr);
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

    {
        // M6w: THE EDITS-IDENTITY CONTRACT. A hand-edit source's cache identity is its file
        // CONTENT -- editing the geojson must change Info().structure (and with it every
        // touched tile's cache tag), and the polygon must actually paint its crest. The M6r
        // pre-bake broke this silently: content changed under a fixed identity, and tiles
        // painted earlier served the un-walled bed forever.
        const char* p1 = "cache\\composed\\_edits_test1.geojson";
        const char* p2 = "cache\\composed\\_edits_test2.geojson";
        auto writeEdits = [](const char* path, double lon) {
            std::ofstream f(path);
            f << "{\"type\":\"FeatureCollection\",\"features\":[{\"type\":\"Feature\","
                 "\"properties\":{\"mask\":\"land\"},\"geometry\":{\"type\":\"Polygon\","
                 "\"coordinates\":[[["
              << lon << ",42.0],[" << lon + 0.01 << ",42.0],[" << lon + 0.01
              << ",42.01],[" << lon << ",42.01],[" << lon << ",42.0]]]}}]}";
        };
        writeEdits(p1, -70.5);
        writeEdits(p2, -70.6);
        EditsHeightSource e1, e2;
        Check(e1.Load(p1, 2.5f) && e2.Load(p2, 2.5f), "edit sources load");
        Check(e1.Info().structure != e2.Info().structure,
              "edit identity follows CONTENT (different files, different cache tags)");
        float m = 0.0f;
        const double d2r = 3.14159265358979 / 180.0;
        Check(e1.Sample(42.005 * d2r, -70.495 * d2r, 10.0, m) > 0.5f && m == 2.5f,
              "edit polygon paints its crest inside");
        Check(e1.Sample(42.005 * d2r, -70.7 * d2r, 10.0, m) == 0.0f,
              "edit polygon silent outside");
    }

    // ---- the Google source's finest zoom is the scene's (streaming.googleZoom), and part of
    // what the source IS. Printed line by line; the plant is the clamp left at the old literal.
    {
        auto say = [](bool ok, const char* line) {
            if (ok) Log("[composetest] %s", line);
            else Check(false, line);
        };
        const GoogleColorSource g14(nullptr), g17(nullptr, 17);
        const std::string today =   // the identity when 14 was a literal, written out
            "google.satellite|mercator-tile-tree jpeg 256px (sessioned, cache-first)";
        auto id = [](const GoogleColorSource& g) { return g.Info().name + "|" + g.Info().structure; };
        // The one check at 17, asked of the source and of the plant alike.
        auto zoomsAt17 = [](auto zoom) {
            return zoom(1.19) == 17 && zoom(9.55) == 14 && zoom(611.0) == 8;
        };
        char b[512];
        snprintf(b, sizeof b,
                 "googleZoom at its default (14): a 1.19 m ask is z%d and a 611 m ask z%d, as when "
                 "14 was a literal",
                 g14.ZoomFor(1.19), g14.ZoomFor(611.0));
        say(g14.ZoomFor(1.19) == 14 && g14.ZoomFor(611.0) == 8, b);
        snprintf(b, sizeof b, "googleZoom 17: a 1.19 m ask is z%d, a 9.55 m ask z%d, a 611 m ask z%d",
                 g17.ZoomFor(1.19), g17.ZoomFor(9.55), g17.ZoomFor(611.0));
        say(zoomsAt17([&](double r) { return g17.ZoomFor(r); }), b);
        snprintf(b, sizeof b,
                 "the identity at 14 is today's literal byte for byte (\"%s\", %.0f cm/px): no "
                 "tree moves",
                 id(g14).c_str(), g14.Info().cmPerPixel);
        say(id(g14) == today && g14.Info().cmPerPixel == 955.0, b);
        snprintf(b, sizeof b,
                 "the identity at 17 differs (\"%s\", %.1f cm/px): a raised zoom paints its own "
                 "tree",
                 id(g17).c_str(), g17.Info().cmPerPixel);
        say(id(g17) != today && id(g17) == today + ", to z17", b);
        // PLANTED: ZoomFor as it was, the clamp at the literal 14, with the key at 17.
        auto planted = [](double r) {
            constexpr double kCirc = 40075016.686;
            return std::clamp(static_cast<int>(std::lround(std::log2(kCirc / (256.0 * r)))), 0, 14);
        };
        snprintf(b, sizeof b,
                 "PLANTED, the clamp left at 14 with the key at 17: a 1.19 m ask is z%d, not z17",
                 planted(1.19));
        if (!zoomsAt17(planted)) {
            Log("[composetest] CAUGHT: %s", b);
        } else {
            Check(false, "(MISSED) the planted clamp at 14 passed the check at 17");
        }
    }

    if (g_ok) {
        Log("[composetest] ---- PASS: paint order, weights, alpha, cache, addressing, "
            "projections, the Google source's zoom cap and its identity ----");
    }
    return g_ok;
}

}  // namespace ga
