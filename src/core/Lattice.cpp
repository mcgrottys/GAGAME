// Lattice.cpp - the M9aj bodies of ColorFrame, moved verbatim (M12 step 2b). See Lattice.h.
#include "core/Lattice.h"

#include <algorithm>
#include <cmath>
#include <cstdio>

namespace ga {

namespace {
constexpr double kPi = 3.14159265358979;
constexpr double kMercCirc = Lattice::kMercCirc;
}  // namespace

void ComposeCubeDir(uint32_t face, double u, double v, double out[3]) {
    const double s = u * 2.0 - 1.0, t = v * 2.0 - 1.0;
    double p[3];
    switch (face) {
        case 0: p[0] = 1;  p[1] = -t; p[2] = -s; break;
        case 1: p[0] = -1; p[1] = -t; p[2] = s;  break;
        case 2: p[0] = s;  p[1] = 1;  p[2] = t;  break;
        case 3: p[0] = s;  p[1] = -1; p[2] = -t; break;
        case 4: p[0] = s;  p[1] = -t; p[2] = 1;  break;
        default: p[0] = -s; p[1] = -t; p[2] = -1; break;
    }
    const double l = std::sqrt(p[0] * p[0] + p[1] * p[1] + p[2] * p[2]);
    out[0] = p[0] / l;
    out[1] = p[1] / l;
    out[2] = p[2] / l;
}

// ---- the frame, said once (see Lattice.h) -------------------------------------------------

double Lattice::GroundRes(uint32_t mip) const {
    if (kind == Kind::Cube) {
        // A cube face spans a quarter of the equator, so its texel is the circumference over
        // four face-widths -- the SAME quantity a Mercator zoom reports, which is what lets a
        // tile-tree source pick a level from it without knowing which realization is asking.
        return kMercCirc / (4.0 * double(faceDim >> mip));
    }
    return kMercCirc / double((1ll << zBase) * 256ll >> mip);
}

void Lattice::Box(const TileRequest& r, TileBox& box) const {
    if (kind == Kind::Cube) {
        // Corners plus edge midpoints and the centre: a gnomonic tile's angular extremes live
        // on its edge midpoints only near the poles, where a loose box is harmless (a global
        // source is always in the subset; every regional source of ours sits far from there).
        const double invFace = 1.0 / double(faceDim >> r.mip);
        box = {10, -10, 10, -10, 0, 0};
        for (int cy = 0; cy < 3; ++cy) {
            for (int cx = 0; cx < 3; ++cx) {
                double d[3];
                ComposeCubeDir(r.face, (r.x * double(texW) + cx * (texW * 0.5)) * invFace,
                               (r.y * double(texH) + cy * (texH * 0.5)) * invFace, d);
                const double la = std::asin((std::max)(-1.0, (std::min)(1.0, d[1])));
                const double lo = std::atan2(d[2], d[0]);
                box.latMin = (std::min)(box.latMin, la);
                box.latMax = (std::max)(box.latMax, la);
                box.lonMin = (std::min)(box.lonMin, lo);
                box.lonMax = (std::max)(box.lonMax, lo);
            }
        }
    } else {
        const double worldPx = double((1ll << zBase) * 256ll >> r.mip);
        const long long gx0 = (orgPxX >> r.mip) + static_cast<long long>(r.x) * texW;
        const long long gy0 = (orgPxY >> r.mip) + static_cast<long long>(r.y) * texH;
        box = {};
        box.latMin = std::atan(std::sinh(kPi * (1.0 - 2.0 * (gy0 + double(texH)) / worldPx)));
        box.latMax = std::atan(std::sinh(kPi * (1.0 - 2.0 * gy0 / worldPx)));
        box.lonMin = (gx0 / worldPx - 0.5) * 2.0 * kPi;
        box.lonMax = ((gx0 + double(texW)) / worldPx - 0.5) * 2.0 * kPi;
    }
    box.texLat = (box.latMax - box.latMin) / double(texH);
    box.texLon = (box.lonMax - box.lonMin) / double(texW);
}

void Lattice::Texel(const TileRequest& r, uint32_t px, uint32_t py, double& latRad,
                    double& lonRad) const {
    if (kind == Kind::Cube) {
        const double invFace = 1.0 / double(faceDim >> r.mip);
        double d[3];
        ComposeCubeDir(r.face, (r.x * double(texW) + px + 0.5) * invFace,
                       (r.y * double(texH) + py + 0.5) * invFace, d);
        latRad = std::asin((std::max)(-1.0, (std::min)(1.0, d[1])));
        lonRad = std::atan2(d[2], d[0]);
        return;
    }
    // Window texels ARE Mercator pixels of zoom (zBase - mip): the lat/lon roundtrip through a
    // Mercator-tree source lands back on the same pixel, so fills stay straight pixel moves.
    const double worldPx = double((1ll << zBase) * 256ll >> r.mip);
    const double X = ((orgPxX >> r.mip) + double(r.x) * texW + px + 0.5) / worldPx;
    const double Y = ((orgPxY >> r.mip) + double(r.y) * texH + py + 0.5) / worldPx;
    latRad = std::atan(std::sinh(kPi * (1.0 - 2.0 * Y)));
    lonRad = (X - 0.5) * 2.0 * kPi;
}

std::string Lattice::Tag(const char* kindName) const {
    char buf[96];
    if (kind == Kind::Cube) {
        snprintf(buf, sizeof(buf), "cube%uk", faceDim / 1024);
        return buf;
    }
    snprintf(buf, sizeof(buf), "%s_z%d_%lld_%lld", kindName, zBase, orgPxX, orgPxY);
    return buf;
}

}  // namespace ga
