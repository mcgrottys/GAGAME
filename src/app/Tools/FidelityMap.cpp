// FidelityMap - --fidelity-map.
// Moved verbatim from main.cpp (M12 step 1b); declared in app/Tools.h.
// No early return, as before: the call site is a statement and the run falls through.
#include "app/Tools.h"

#include "compose/Compositor.h"
#include "compose/VectorPack.h"
#include "core/Common.h"
#include "core/Font5x7.h"
#include "core/Image.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <string>
#include <utility>
#include <vector>

namespace ga::app::tools {

namespace {

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

// The 5x7 font is core/Font5x7.h (shared with the HUD). Labels make this a tool instead of
// an abstract picture -- a legend you have to decode elsewhere is a legend nobody reads.

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

void RenderFidelityMap(Compositor& comp, VectorPack& vec, const std::wstring& outPath,
                       const Space::Anchor& chart) {
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
        // PHASE C4: the rungs about the scene's place.anchor.
        {"THE REGION  8 DEG", chart.latDeg, chart.lonDeg, 8.0},
        {"THE ESTUARY  0.8 DEG", chart.latDeg, chart.lonDeg, 0.8},
        {"THE INLET  0.08 DEG", chart.latDeg, chart.lonDeg, 0.08},
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
            if (const GlyphRow* gl = Glyph5x7(*c)) {
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

}  // namespace

void RunFidelityMap(const Options& opt, Compositor& compositor, const Space::Anchor& chart) {
    VectorPack fidVec;
    fidVec.Load("data/vectors/vectors.vpack");
    RenderFidelityMap(compositor, fidVec, opt.fidelityMap, chart);
    // No early return: the flag forces headless + 1 frame, so the run falls
    // through to the NORMAL teardown. Tearing down by hand from this deep in
    // the wiring exits 9 -- the layers and residency tenants are live here,
    // unlike at the --water-map exit which runs before any of them exist.
}

}  // namespace ga::app::tools
