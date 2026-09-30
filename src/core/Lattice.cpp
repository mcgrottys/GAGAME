// Lattice.cpp - the M9aj bodies of ColorFrame, moved verbatim (M12 step 2b). See Lattice.h.
// After them, HIERARCHY step 3's face-plane window (FaceWindow), new.
#include "core/Lattice.h"

#include "core/Space.h"

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

void Lattice::TexelGround(uint32_t mip, double lat, double lon, double& du, double& dv) const {
    du = dv = GroundRes(mip) * std::cos(lat);
    if (kind != Kind::Cube) return;
    const double d[3] = {std::cos(lat) * std::cos(lon), std::sin(lat), std::cos(lat) * std::sin(lon)};
    double uv[2], p[3][3];
    const uint32_t face = CubeFaceOfDir(d, uv);
    const double s = 1.0 / double(faceDim >> mip), R = kMercCirc / (2.0 * kPi);
    ComposeCubeDir(face, uv[0], uv[1], p[0]);
    ComposeCubeDir(face, uv[0] + s, uv[1], p[1]);
    ComposeCubeDir(face, uv[0], uv[1] + s, p[2]);
    auto arc = [&](const double* a, const double* b) {   // the angle between two unit vectors, x R
        const double c[3] = {a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]};
        return R * std::atan2(std::sqrt(c[0] * c[0] + c[1] * c[1] + c[2] * c[2]),
                              a[0] * b[0] + a[1] * b[1] + a[2] * b[2]);
    };
    du = arc(p[0], p[1]);
    dv = arc(p[0], p[2]);
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

ast::Frame Lattice::AstFrame() const {
    // The table's two frames (GaAst.cpp's `cube` and `mercPx`, {space, vNorth, orgX, orgY,
    // metersPerUnit, centers}), produced from the declaration: the space from the kind, +v
    // from VNorth(), origin and pitch 0, centres.
    return ast::Frame{kind == Kind::Cube ? "cube.face" : "mercator.px", VNorth(), 0.0, 0.0,
                      0.0, true};
}

// ---- HIERARCHY step 3: the face-plane window (see Lattice.h) ---------------------------------

void CubeFaceAxes(uint32_t face, double n[3], double a[3], double b[3]) {
    // ComposeCubeDir's table, row by row: its unnormalized point is n + s a + t b.
    static const double kAxes[6][3][3] = {
        {{1, 0, 0}, {0, 0, -1}, {0, -1, 0}},    // 0: (1, -t, -s)
        {{-1, 0, 0}, {0, 0, 1}, {0, -1, 0}},    // 1: (-1, -t, s)
        {{0, 1, 0}, {1, 0, 0}, {0, 0, 1}},      // 2: (s, 1, t)
        {{0, -1, 0}, {1, 0, 0}, {0, 0, -1}},    // 3: (s, -1, -t)
        {{0, 0, 1}, {1, 0, 0}, {0, -1, 0}},     // 4: (s, -t, 1)
        {{0, 0, -1}, {-1, 0, 0}, {0, -1, 0}}};  // 5: (-s, -t, -1)
    const double(&f)[3][3] = kAxes[face < 6 ? face : 5];   // ComposeCubeDir's default is face 5
    for (int i = 0; i < 3; ++i) {
        n[i] = f[0][i];
        a[i] = f[1][i];
        b[i] = f[2][i];
    }
}

double FaceWindow::FaceTexels() const { return std::ldexp(double(Lattice::kFaceDim), rung); }

void FaceWindow::TexelOf(const double P[3], double& x, double& y) const {
    double n[3], a[3], b[3];
    CubeFaceAxes(face, n, a, b);
    const double pn = P[0] * n[0] + P[1] * n[1] + P[2] * n[2];
    const double s = (P[0] * a[0] + P[1] * a[1] + P[2] * a[2]) / pn;
    const double t = (P[0] * b[0] + P[1] * b[1] + P[2] * b[2]) / pn;
    const double N = FaceTexels();
    x = (s * 0.5 + 0.5) * N - double(anchorX);
    y = (t * 0.5 + 0.5) * N - double(anchorY);
}

FaceWindow FaceWindow::Nearest(const double P[3]) const {
    FaceWindow o = *this;
    o.anchorX = o.anchorY = 0;
    double X = 0.0, Y = 0.0;
    o.TexelOf(P, X, Y);   // anchored at the face's corner: the global texel
    const double dim = double(Lattice::kFaceDim);
    o.anchorX = std::llround(X / dim) * static_cast<long long>(Lattice::kFaceDim);
    o.anchorY = std::llround(Y / dim) * static_cast<long long>(Lattice::kFaceDim);
    return o;
}

FaceWindow::Planes FaceWindow::PlanesIn(const Placement& own) const {
    double n[3], a[3], b[3];
    CubeFaceAxes(face, n, a, b);
    const double N = FaceTexels(), k = 0.5 * N;
    const double s0 = 2.0 * double(anchorX) / N - 1.0;
    const double t0 = 2.0 * double(anchorY) / N - 1.0;
    // The three planes through the centre, planet frame. They are not unit, and PullPlane does
    // not need them to be: its arithmetic is linear in the normal.
    const double m[3][3] = {{k * (a[0] - s0 * n[0]), k * (a[1] - s0 * n[1]), k * (a[2] - s0 * n[2])},
                            {k * (b[0] - t0 * n[0]), k * (b[1] - t0 * n[1]), k * (b[2] - t0 * n[2])},
                            {n[0], n[1], n[2]}};
    Planes pl{};
    float* const rows[3] = {pl.u, pl.v, pl.w};
    for (int i = 0; i < 3; ++i) {
        // Through the centre (d = 0): n' = R^T m, d' = -(m . own.t) / own.s, so the plane's
        // value at a point x of own's frame is n' . x - d' = (m . X) / own.s -- the same factor
        // on every row, which is why it cancels in the ratio.
        double mn[3], md = 0.0;
        own.PullPlane(m[i], 0.0, mn, md);
        rows[i][0] = static_cast<float>(mn[0]);
        rows[i][1] = static_cast<float>(mn[1]);
        rows[i][2] = static_cast<float>(mn[2]);
        rows[i][3] = static_cast<float>(-md);
    }
    return pl;
}

void FaceWindow::PageTexel(const float p[3], const Planes& pl, float& x, float& y) {
    // dot(p, row.xyz) + row.w, the dot left to right: one named float per operation.
    auto plane = [p](const float r[4]) {
        const float m0 = p[0] * r[0];
        const float m1 = p[1] * r[1];
        const float m2 = p[2] * r[2];
        const float d01 = m0 + m1;
        const float d = d01 + m2;
        const float e = d + r[3];
        return e;
    };
    const float nu = plane(pl.u);
    const float nv = plane(pl.v);
    const float nw = plane(pl.w);
    x = nu / nw;
    y = nv / nw;
}

}  // namespace ga
