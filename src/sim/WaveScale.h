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
//  A DECLARED STORM IS THE REFERENCE. A storm rewrites the partitions, not the product, so the grid
//  is stale by definition (WaterBankLayer's M8 note: ratioing it painted a 17x tile staircase along
//  the grid's land/sea edge). Under a storm the scale is 1 everywhere.
//
//  The clamp [0.15, 3] on a node's ratio is the bank's own (carried, not re-derived): a grid node an
//  order of magnitude from the reference is a different sea than the cascades can stand in for.
// ================================================================================================
#pragma once

#include "sim/GlobeModel.h"
#include "sim/SeaState.h"

#include <algorithm>
#include <cmath>

namespace ga {

struct WaveScale {
    const GlobeModel* grid = nullptr;
    double hsRef = 1.0;    // the reference the cascades were synthesised for, m
    bool storm = false;    // the partitions are a declared storm: the scale is 1

    // The law's inputs at one instant. The reference is the forecast hour's combined Hs, floored at
    // 0.3 m (a calm reference would divide the grid by nearly nothing); 1 m when no sea state loaded.
    static WaveScale For(const GlobeModel* grid, const SeaState* reference, bool storm,
                         double unixT) {
        WaveScale s;
        s.grid = grid;
        s.storm = storm;
        s.hsRef = (reference && reference->Ready())
                      ? (std::max)(reference->Hour(reference->HourIndex(unixT)).combinedHs, 0.3)
                      : 1.0;
        return s;
    }

    // The scale at a place.
    double At(double latDeg, double lonDeg) const {
        if (storm || !grid) return 1.0;
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
