// GisDump - --gis-dump PATH.
// Moved verbatim from main.cpp (M12 step 1b); declared in app/Tools.h.
// The body ends in std::exit(0), exactly as it did in main(); the call site is a statement.
#include "app/Tools.h"

#include "compose/GisMask.h"
#include "core/Common.h"

#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <string>
#include <vector>

namespace ga::app::tools {

void RunGisDump(const Options& opt, const GisVectorMask& gisMask) {
    // M9av: LOOK AT THE GATE. 255 = water (or no opinion), 0 = land. PATH@lat0,lon0,lat1,lon1,dim
    // (2026-10-21) dumps that box in one tile of dim texels a side -- the grain a tile of the mask's
    // tree reads at -- and shows water at a level of its own (inland, flag bit 2) as 192.
    double b0, a0, b1, a1;
    gisMask.Bounds(b0, a0, b1, a1);
    uint32_t dim = 1024;
    std::string path = opt.gisDump;
    if (const size_t at = path.find('@'); at != std::string::npos) {
        unsigned d = 0;
        if (sscanf_s(path.c_str() + at + 1, "%lf,%lf,%lf,%lf,%u", &a0, &b0, &a1, &b1, &d) == 5 && d >= 2) dim = d;
        path.resize(at);
    }
    std::vector<uint8_t> g;
    gisMask.RasterizeGate(a0, a1, b0, b1, dim, g);
    std::vector<uint8_t> v(static_cast<size_t>(dim) * dim);
    for (size_t i = 0; i < v.size(); ++i) {
        // value where surveyed; 128 (grey) where the mask has no opinion; 192 inland water
        v[i] = !(g[i * 2 + 1] & 1u) ? 128u : (g[i * 2 + 1] & 4u) ? 192u : g[i * 2];
    }
    std::ofstream pf(path, std::ios::binary);
    pf << "P5\n" << dim << " " << dim << "\n255\n";
    pf.write(reinterpret_cast<const char*>(v.data()), v.size());
    Log("[gismask] --gis-dump: %s (%ux%u over %.3f,%.3f..%.3f,%.3f) %s",
        path.c_str(), dim, dim, b0, a0, b1, a1,
        pf ? "written" : "FAILED TO WRITE");
    std::exit(0);
}

void RunGisSweepTest(const GisVectorMask& gisMask) {
    double b0, a0, b1, a1;
    gisMask.Bounds(b0, a0, b1, a1);
    // The survey's centre: the grains run from a 20 km tile to a 2 m one, 12 x 12 tiles each,
    // so the finest grid lies in one ring.
    const double clon = 0.5 * (b0 + b1), clat = 0.5 * (a0 + a1);
    const double widths[5] = {0.2, 0.02, 0.002, 0.0002, 0.00002};
    uint32_t tiles = 0, mismatched = 0, fullBefore = 0, oneBefore = 0;
    gisMask.SweepCounts(fullBefore, oneBefore);
    std::vector<uint8_t> a, b;
    for (double w : widths) {
        for (int iy = -6; iy < 6; ++iy) {
            for (int ix = -6; ix < 6; ++ix) {
                const double lon0 = clon + ix * w, lat0 = clat + iy * w;
                gisMask.RasterizeGate(lat0, lat0 + w, lon0, lon0 + w, 256, a);
                gisMask.RasterizeGateFull(lat0, lat0 + w, lon0, lon0 + w, 256, b);
                ++tiles;
                if (a != b) {
                    ++mismatched;
                    if (mismatched <= 8) {
                        size_t k = 0;
                        while (k < a.size() && a[k] == b[k]) ++k;
                        Log("[gismask] sweep-test MISMATCH: tile %.6f,%.6f +%.5f deg, first byte %zu "
                            "(cell %zu,%zu): one column %u, full %u",
                            lon0, lat0, w, k, (k / 2) % 256, (k / 2) / 256, a[k], b[k]);
                    }
                }
            }
        }
    }
    uint32_t full = 0, one = 0;
    gisMask.SweepCounts(full, one);
    // RasterizeGateFull counts as full too: the one-column tiles are the shortcut's own count.
    const uint32_t oneTiles = one - oneBefore;
    Log("[gismask] ---- sweep-test %s: %u tiles at 5 grains (20 km .. 2 m), %u decided by one "
        "column, %u swept in full, %u MISMATCHED against the full sweep ----",
        mismatched ? "FAIL" : "PASS", tiles, oneTiles, tiles - oneTiles, mismatched);
    std::exit(mismatched ? 1 : 0);
}

}  // namespace ga::app::tools
