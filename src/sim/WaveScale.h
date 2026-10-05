// ================================================================================================
//  WaveScale - the local sea-state scale: the global wave grid's Hs over the reference Hs the
//  cascades were synthesised for (the water match, step 2: one wave rule).
//
//  ONE LAW, TWO READERS. The bank kernel multiplies every cascade band by this scale (its tiles
//  carry it at their corners, WaterBankLayer::Render) and the hull's twin multiplies its cascades by
//  it at the point (TreeWater::At). Written once here, so the sea a hull rides and the sea the rings
//  draw cannot be scaled by two different readings of the same grid.
//
//  THE GRID IS NODE-CENTRED. gfswave is 1440 x 721 nodes on 0..360, row 0 at the north pole
//  (GlobeLayer states it: `wref.centers = false`). A value belongs to its node, so a point between
//  nodes reads the bilinear blend of its four. The bank used to take the node at or west and north of
//  each TILE's centre: a staircase at every node line, stepping again at every tile edge, and half a
//  cell off -- a piecewise answer to a continuous question.
//
//  ABSENCE READS THE REFERENCE. gfswave writes -1 over land. A node with no Hs has no opinion about
//  the sea, and the sea there is the cascades' own -- scale 1, exactly what it is where the grid is
//  missing altogether -- blended as a value, so the scale stays continuous across every coast.
//
//  PHASE C2: THE SEA STATE IS A FIELD OF SOURCES OVER THE GRID. Each sea-state file is a source by
//  being a file (the scene names it: data.seastate, and `sources` entries of kind "seastate"), placed
//  by its own box; its Hs is the one law the synthesis uses (SeaLayer::PartsOf: its forecast hour at
//  its own clock, its own buoys assimilated; data.seastate's replaced by a declared storm). A source
//  paints over the grid (LayeredOver, the later listed on top) at full weight inside its box, its
//  weight falling to 0 one grid node (0.25 deg) outside it. Where data.seastate's box holds a place
//  the scale is its Hs over itself: 1. The storm is no longer the planet's: it is that source's sea.
//
//  The clamp [0.15, 3] on a node's ratio is the bank's own (carried, not re-derived): a grid node an
//  order of magnitude from the reference is a different sea than the cascades can stand in for.
// ================================================================================================
#pragma once

#include "sim/GlobeModel.h"

#include <algorithm>
#include <cmath>
#include <vector>

namespace ga {

struct WaveScale {
    struct Source {
        double box[4] = {0.0, 0.0, 0.0, 0.0};   // lat0, lon0, lat1, lon1, degrees
        double hs = 0.0;                         // its Hs now, m
    };
    double hsRef = 1.0;              // the Hs the cascades were synthesised for, m (SeaLayer)
    std::vector<Source> sources;     // over the grid, the later on top
    // 2026-10-04 (the owner: Haulover flat under the storm): a declared storm is a SANDBOX sea, the
    // planet's, not one source's box -- the scale is 1 everywhere while it stands.
    bool storm = false;
    static constexpr double kFeatherDeg = 0.25;   // one gfswave node

    // The scale at a place: the grid's, then every source over it by its weight there.
    double At(const GlobeModel* grid, double latDeg, double lonDeg) const {
        if (storm) return 1.0;
        double s = GridAt(grid, latDeg, lonDeg);
        for (const Source& src : sources) {
            const double d = (std::max)((std::max)(src.box[0] - latDeg, latDeg - src.box[2]),
                                        (std::max)(src.box[1] - lonDeg, lonDeg - src.box[3]));
            const double t = std::clamp(d / kFeatherDeg, 0.0, 1.0);
            const double w = 1.0 - t * t * (3.0 - 2.0 * t);
            if (w > 0.0) s += w * (std::clamp(src.hs / hsRef, 0.15, 3.0) - s);
        }
        return s;
    }

    // The grid's node ratio, bilinear; 1 where it has no opinion.
    double GridAt(const GlobeModel* grid, double latDeg, double lonDeg) const {
        if (!grid) return 1.0;
        const int nx = grid->WavesNx(), ny = grid->WavesNy();
        const std::vector<float>& hs = grid->Hs();
        if (nx < 2 || ny < 2 || hs.size() < size_t(nx) * size_t(ny)) return 1.0;
        const double dLon = grid->WavesDLon(), dLat = grid->WavesDLat();
        // Longitude wraps on a grid that closes the circle; one that does not is clamped at its ends.
        const bool wraps = std::abs(double(nx) * dLon - 360.0) < 1e-6;
        double fx = (lonDeg - grid->WavesLon1()) / dLon;
        if (wraps) {
            fx -= std::floor(fx / double(nx)) * double(nx);
        } else {
            fx = std::clamp(fx, 0.0, double(nx - 1));
        }
        const double fy = std::clamp((grid->WavesLat1() - latDeg) / dLat, 0.0, double(ny - 1));
        const int i0 = (std::min)(int(fx), wraps ? nx - 1 : nx - 2);
        const int j0 = (std::min)(int(fy), ny - 2);
        const int i1 = wraps ? (i0 + 1) % nx : i0 + 1;
        const int j1 = j0 + 1;
        const double tx = fx - double(i0), ty = fy - double(j0);
        const auto node = [&](int i, int j) {
            const float h = hs[size_t(j) * size_t(nx) + size_t(i)];
            return (h >= 0.0f) ? std::clamp(double(h) / hsRef, 0.15, 3.0) : 1.0;
        };
        return (node(i0, j0) * (1.0 - tx) + node(i1, j0) * tx) * (1.0 - ty) +
               (node(i0, j1) * (1.0 - tx) + node(i1, j1) * tx) * ty;
    }
};

}  // namespace ga
