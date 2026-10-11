// OriginLattice - the lattice of floating origins (OriginLattice.h).
#include "core/OriginLattice.h"

#include <algorithm>

namespace ga {

namespace {
constexpr double kPi = 3.14159265358979323846;
constexpr double kPhi = 1.6180339887498948482;   // the golden ratio
double Frac(double x) { return x - std::floor(x); }
double Dot(const double a[3], const double b[3]) { return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]; }
}  // namespace

void OriginLattice::Direction(int64_t i, double d[3]) const {
    // Keinert 2015 eq. 2: phi = 2 pi frac(i / phi), cos theta = 1 - (2 i + 1) / n -- the paper's
    // z is the planet frame's y (the polar axis).
    const double phi = 2.0 * kPi * Frac(double(i) * (kPhi - 1.0));
    const double ct = 1.0 - (2.0 * double(i) + 1.0) / double(n);
    const double st = std::sqrt(std::max(0.0, 1.0 - ct * ct));
    d[0] = std::cos(phi) * st;
    d[1] = ct;
    d[2] = std::sin(phi) * st;
}

int64_t OriginLattice::InverseIndex(const double p[3]) const {
    // Keinert 2015, listing 1, in doubles: the local lattice basis about the point from the two
    // Fibonacci numbers of the zone's scale, the cell's corner, and the nearest of its four points.
    const double N = double(n);
    const double phi = std::min(std::atan2(p[2], p[0]), kPi);
    const double ct = std::clamp(p[1], -1.0, 1.0);
    const double st2 = std::max(1.0 - ct * ct, 1e-300);
    const double k = std::max(2.0, std::floor(std::log(N * kPi * std::sqrt(5.0) * st2) / std::log(kPhi * kPhi)));
    const double Fk = std::pow(kPhi, k) / std::sqrt(5.0);
    const double F0 = std::round(Fk), F1 = std::round(Fk * kPhi);
    const double ka0 = 2.0 * F0 / N, ka1 = 2.0 * F1 / N;
    const double kb0 = 2.0 * kPi * (Frac((F0 + 1.0) * kPhi) - (kPhi - 1.0));
    const double kb1 = 2.0 * kPi * (Frac((F1 + 1.0) * kPhi) - (kPhi - 1.0));
    const double det = ka1 * kb0 - ka0 * kb1;
    const double v0 = phi, v1 = ct - (1.0 - 1.0 / N);
    const double c0 = std::floor((ka1 * v0 + kb1 * v1) / det);
    const double c1 = std::floor((-ka0 * v0 - kb0 * v1) / det);
    double best = 8.0;
    int64_t j = 0;
    for (int s = 0; s < 4; ++s) {
        const double u0 = double(s & 1) + c0, u1 = double(s >> 1) + c1;
        const double iF = F0 * u0 + F1 * u1;
        if (!(iF >= 0.0) || iF >= N) continue;
        const int64_t i = int64_t(iF);
        double q[3];
        Direction(i, q);
        const double dd = (q[0] - p[0]) * (q[0] - p[0]) + (q[1] - p[1]) * (q[1] - p[1]) + (q[2] - p[2]) * (q[2] - p[2]);
        if (dd < best) {
            best = dd;
            j = i;
        }
    }
    return j;
}

OriginLattice::Origin OriginLattice::At(int shell, int64_t index) const {
    Origin o;
    if (index < 0) {
        o.centre = true;
        o.shell = 0;
        o.index = -1;
        o.r = 0.0;
        // The identity frame: east x, up y, north z -- the planet frame's own axes.
        o.anchor.east[0] = 1.0; o.anchor.east[1] = 0.0; o.anchor.east[2] = 0.0;
        o.anchor.up[0] = 0.0; o.anchor.up[1] = 1.0; o.anchor.up[2] = 0.0;
        o.anchor.north[0] = 0.0; o.anchor.north[1] = 0.0; o.anchor.north[2] = 1.0;
        o.anchor.planetR = 0.0;
        return o;
    }
    o.shell = shell;
    o.index = index;
    o.r = RadiusOf(shell);
    double d[3];
    Direction(index, d);
    const double r2d = 180.0 / kPi;
    const double lat = std::asin(std::clamp(d[1], -1.0, 1.0)) * r2d, lon = std::atan2(d[2], d[0]) * r2d;
    o.anchor = Space::Anchor::About(lat, lon, o.r);
    for (int i = 0; i < 3; ++i) o.c[i] = o.anchor.up[i] * o.r;   // the anchor's up IS its direction
    return o;
}

OriginLattice::Origin OriginLattice::Nearest(const double P[3]) const {
    const double r = std::sqrt(Dot(P, P));
    if (r < rMin) return At(0, -1);
    const int k = ShellOf(r);
    if (RadiusOf(k) < rMin) return At(0, -1);
    const double d[3] = {P[0] / r, P[1] / r, P[2] / r};
    return At(k, InverseIndex(d));
}

OriginLattice::Origin OriginLattice::Pick(const double P[3], const Origin* current, double margin) const {
    const Origin best = Nearest(P);
    if (!current || current->Same(best)) return best;
    const double dn[3] = {P[0] - best.c[0], P[1] - best.c[1], P[2] - best.c[2]};
    const double dc[3] = {P[0] - current->c[0], P[1] - current->c[1], P[2] - current->c[2]};
    const double ln = std::sqrt(Dot(dn, dn)), lc = std::sqrt(Dot(dc, dc));
    return lc <= (1.0 + margin) * ln ? *current : best;
}

void OriginLattice::Floats(const Origin& o, const double P[3], double out[3]) {
    const double d[3] = {P[0] - o.c[0], P[1] - o.c[1], P[2] - o.c[2]};
    out[0] = Dot(o.anchor.east, d);
    out[1] = Dot(o.anchor.up, d);
    out[2] = Dot(o.anchor.north, d);
}

void OriginLattice::Point(const Origin& o, const double f[3], double P[3]) {
    for (int i = 0; i < 3; ++i) P[i] = o.c[i] + o.anchor.east[i] * f[0] + o.anchor.up[i] * f[1] + o.anchor.north[i] * f[2];
}

Motor OriginLattice::Between(const Origin& from, const Origin& to) {
    // floats_to = R_to^T (R_from floats_from + c_from - c_to), R the frame's columns (east, up,
    // north): a rotation Q = R_to^T R_from and a translation t = R_to^T (c_from - c_to), as one
    // motor T(t) Rot(Q). Q's axis and angle from its quaternion (Shepperd's choice of the largest
    // diagonal keeps it exact for every attitude).
    const double* A[3] = {from.anchor.east, from.anchor.up, from.anchor.north};
    const double* B[3] = {to.anchor.east, to.anchor.up, to.anchor.north};
    double Q[3][3];   // Q[r][c] = B_r . A_c
    for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 3; ++c) Q[r][c] = Dot(B[r], A[c]);
    const double dc[3] = {from.c[0] - to.c[0], from.c[1] - to.c[1], from.c[2] - to.c[2]};
    const double t[3] = {Dot(B[0], dc), Dot(B[1], dc), Dot(B[2], dc)};
    const double tr = Q[0][0] + Q[1][1] + Q[2][2];
    double x, y, z, w;
    if (tr > 0.0) {
        const double s = std::sqrt(tr + 1.0) * 2.0;
        w = 0.25 * s; x = (Q[2][1] - Q[1][2]) / s; y = (Q[0][2] - Q[2][0]) / s; z = (Q[1][0] - Q[0][1]) / s;
    } else if (Q[0][0] > Q[1][1] && Q[0][0] > Q[2][2]) {
        const double s = std::sqrt(1.0 + Q[0][0] - Q[1][1] - Q[2][2]) * 2.0;
        w = (Q[2][1] - Q[1][2]) / s; x = 0.25 * s; y = (Q[0][1] + Q[1][0]) / s; z = (Q[0][2] + Q[2][0]) / s;
    } else if (Q[1][1] > Q[2][2]) {
        const double s = std::sqrt(1.0 + Q[1][1] - Q[0][0] - Q[2][2]) * 2.0;
        w = (Q[0][2] - Q[2][0]) / s; x = (Q[0][1] + Q[1][0]) / s; y = 0.25 * s; z = (Q[1][2] + Q[2][1]) / s;
    } else {
        const double s = std::sqrt(1.0 + Q[2][2] - Q[0][0] - Q[1][1]) * 2.0;
        w = (Q[1][0] - Q[0][1]) / s; x = (Q[0][2] + Q[2][0]) / s; y = (Q[1][2] + Q[2][1]) / s; z = 0.25 * s;
    }
    const double sh = std::sqrt(x * x + y * y + z * z);
    const double org[3] = {0.0, 0.0, 0.0};
    Motor rot = Motor::Identity();
    if (sh > 1e-15) {
        const double axis[3] = {x / sh, y / sh, z / sh};
        rot = Motor::Rotation(org, axis, 2.0 * std::atan2(sh, w));
    }
    return Motor::Translation(t[0], t[1], t[2]) * rot;
}

}  // namespace ga
