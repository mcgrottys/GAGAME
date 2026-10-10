#include "compose/BuildingMoments.h"

#include "core/Common.h"

#include <algorithm>
#include <cmath>

namespace ga {

namespace {

// One ring's area moments by Green's theorem: A, int x, int y, int x^2, int y^2, int xy, signed by
// the ring's winding (counter-clockwise positive).
void RingMoments(const std::vector<double>& r, double out[6]) {
    for (int k = 0; k < 6; ++k) out[k] = 0.0;
    const size_t n = r.size() / 2;
    if (n < 3) return;
    for (size_t i = 0; i < n; ++i) {
        const size_t j = (i + 1) % n;
        const double x0 = r[2 * i], y0 = r[2 * i + 1], x1 = r[2 * j], y1 = r[2 * j + 1];
        const double cr = x0 * y1 - x1 * y0;
        out[0] += cr;
        out[1] += (x0 + x1) * cr;
        out[2] += (y0 + y1) * cr;
        out[3] += (x0 * x0 + x0 * x1 + x1 * x1) * cr;
        out[4] += (y0 * y0 + y0 * y1 + y1 * y1) * cr;
        out[5] += (x0 * y1 + 2.0 * x0 * y0 + 2.0 * x1 * y1 + x1 * y0) * cr;
    }
    out[0] /= 2.0;
    out[1] /= 6.0;
    out[2] /= 6.0;
    out[3] /= 12.0;
    out[4] /= 12.0;
    out[5] /= 24.0;
}

}  // namespace

Moments& Moments::operator+=(const Moments& o) {
    m += o.m;
    for (int k = 0; k < 3; ++k) s[k] += o.s[k];
    for (int k = 0; k < 6; ++k) S[k] += o.S[k];
    return *this;
}

Moments Moments::About(const double o[3]) const {
    // x' = x - o:  s' = s - m o,  S'_ij = S_ij - o_i s_j - o_j s_i + m o_i o_j.
    static const int I[6] = {0, 1, 2, 0, 0, 1}, J[6] = {0, 1, 2, 1, 2, 2};
    Moments r;
    r.m = m;
    for (int k = 0; k < 3; ++k) r.s[k] = s[k] - m * o[k];
    for (int k = 0; k < 6; ++k) {
        const int i = I[k], j = J[k];
        r.S[k] = S[k] - o[i] * s[j] - o[j] * s[i] + m * o[i] * o[j];
    }
    return r;
}

cga::Mv Moments::Conformal(double L) const {
    const double tr = S[0] + S[1] + S[2];
    return cga::N0() * m + cga::Dir(s[0] / L, s[1] / L, s[2] / L) + cga::Ni() * (0.5 * tr / (L * L));
}

Moments PrismMoments(const std::vector<std::vector<double>>& rings, double bottom, double top) {
    Moments r;
    if (rings.empty() || !(top > bottom)) return r;
    double a[6] = {0, 0, 0, 0, 0, 0};
    for (size_t k = 0; k < rings.size(); ++k) {
        double q[6];
        RingMoments(rings[k], q);
        // The outer adds and every hole subtracts, whatever the file's winding.
        const double sign = ((q[0] >= 0.0) == (k == 0)) ? 1.0 : -1.0;
        for (int i = 0; i < 6; ++i) a[i] += sign * q[i];
    }
    if (!(a[0] > 0.0)) return r;
    const double Lz = top - bottom, Z1 = (top * top - bottom * bottom) / 2.0,
                 Z2 = (top * top * top - bottom * bottom * bottom) / 3.0;
    r.m = a[0] * Lz;
    r.s[0] = a[1] * Lz;
    r.s[1] = a[2] * Lz;
    r.s[2] = a[0] * Z1;
    r.S[0] = a[3] * Lz;
    r.S[1] = a[4] * Lz;
    r.S[2] = a[0] * Z2;
    r.S[3] = a[5] * Lz;
    r.S[4] = a[1] * Z1;
    r.S[5] = a[2] * Z1;
    return r;
}

double MomentBox::Radius() const {
    return std::sqrt(half[0] * half[0] + half[1] * half[1] + half[2] * half[2]);
}

MomentBox BoxOf(const Moments& mo) {
    MomentBox b;
    if (mo.Empty()) return b;
    for (int k = 0; k < 3; ++k) b.c[k] = mo.s[k] / mo.m;
    const double cxx = mo.S[0] / mo.m - b.c[0] * b.c[0], cyy = mo.S[1] / mo.m - b.c[1] * b.c[1],
                 czz = mo.S[2] / mo.m - b.c[2] * b.c[2], cxy = mo.S[3] / mo.m - b.c[0] * b.c[1];
    b.heading = 0.5 * std::atan2(2.0 * cxy, cxx - cyy);
    const double mid = 0.5 * (cxx + cyy), dev = std::hypot(0.5 * (cxx - cyy), cxy);
    const double l1 = std::max(mid + dev, 0.0), l2 = std::max(mid - dev, 0.0);
    b.half[2] = std::sqrt(3.0 * std::max(czz, 0.0));
    double a1 = std::sqrt(3.0 * l1), a2 = std::sqrt(3.0 * l2);
    // The footprint keeps its area: volume = 8 a1 a2 hz.
    if (a1 * a2 > 0.0 && b.half[2] > 0.0) {
        const double k = std::sqrt(mo.m / (8.0 * a1 * a2 * b.half[2]));
        a1 *= k;
        a2 *= k;
    }
    b.half[0] = a1;
    b.half[1] = a2;
    return b;
}

int LodLevel(double rho, double rho0) {
    if (!(rho > rho0)) return 0;
    return static_cast<int>(std::floor(std::log2(rho / rho0)));
}

double LodReach(int level, double rho0, double pixels, double pixAng) {
    return rho0 * std::ldexp(1.0, level) / (pixels * pixAng);
}

// ---- the gate -----------------------------------------------------------------------------------

namespace {

std::vector<double> RotRect(double cx, double cy, double w, double h, double ang) {
    const double c = std::cos(ang), s = std::sin(ang);
    const double p[4][2] = {{-w / 2, -h / 2}, {w / 2, -h / 2}, {w / 2, h / 2}, {-w / 2, h / 2}};
    std::vector<double> r;
    for (auto& q : p) {
        r.push_back(cx + c * q[0] - s * q[1]);
        r.push_back(cy + s * q[0] + c * q[1]);
    }
    return r;
}

std::vector<double> Shift(std::vector<double> r, double dx, double dy) {
    for (size_t i = 0; i < r.size(); i += 2) {
        r[i] += dx;
        r[i + 1] += dy;
    }
    return r;
}

std::vector<double> Reversed(const std::vector<double>& r) {
    std::vector<double> o;
    for (size_t i = r.size(); i >= 2; i -= 2) {
        o.push_back(r[i - 2]);
        o.push_back(r[i - 1]);
    }
    return o;
}

double MomDiff(const Moments& a, const Moments& b) {
    double d = std::abs(a.m - b.m) / std::max(1.0, std::abs(a.m));
    for (int k = 0; k < 3; ++k) d = std::max(d, std::abs(a.s[k] - b.s[k]) / std::max(1.0, std::abs(a.s[k])));
    for (int k = 0; k < 6; ++k) d = std::max(d, std::abs(a.S[k] - b.S[k]) / std::max(1.0, std::abs(a.S[k])));
    return d;
}

double MvDiff(const cga::Mv& a, const cga::Mv& b) { return (a - b).Max() / std::max(1.0, a.Max()); }

}  // namespace

bool RunLodSelfTest() {
    bool ok = true;
    auto check = [&ok](bool c, const char* what, double v) {
        Log("[lod] %s %s (%.3g)", c ? "ok  " : "FAIL", what, v);
        ok &= c;
    };
    const double kPi = 3.14159265358979323846;

    // A rectangle returns itself: 40 x 10 m at 30 degrees, 2..62 m up, about (100, -50).
    {
        const MomentBox b = BoxOf(PrismMoments({RotRect(100, -50, 40, 10, kPi / 6)}, 2, 62));
        const double e = std::max({std::abs(b.half[0] - 20), std::abs(b.half[1] - 5), std::abs(b.half[2] - 30),
                                   std::abs(b.c[0] - 100), std::abs(b.c[1] + 50), std::abs(b.c[2] - 32),
                                   std::abs(std::remainder(b.heading - kPi / 6, kPi))});
        check(e < 1e-9, "a rectangle's moment box is the rectangle", e);
    }
    // An L with a hole, either winding of the hole: same moments; the box keeps volume, centroid
    // and the prism's own height.
    const std::vector<double> L = {0, 0, 30, 0, 30, 10, 10, 10, 10, 40, 0, 40};
    const std::vector<double> hole = RotRect(5, 30, 4, 6, 0);
    const Moments mL = PrismMoments({L, hole}, 0, 24);
    {
        const double d = MomDiff(mL, PrismMoments({Reversed(L), Reversed(hole)}, 0, 24));
        check(d < 1e-12, "winding does not change a solid's moments", d);
        const MomentBox b = BoxOf(mL);
        const double vol = 8.0 * b.half[0] * b.half[1] * b.half[2];
        const double area = 30 * 10 + 10 * 30 - 24;
        const double e = std::max({std::abs(vol - mL.m) / mL.m, std::abs(mL.m - area * 24) / mL.m,
                                   std::abs(b.half[2] - 12), std::abs(b.c[2] - 12)});
        check(e < 1e-9, "the box keeps volume, the footprint area and the prism's height", e);
    }
    // FOLD + FRAME: two solids about the origin, summed, then moved, equal the solids built
    // directly about the new origin and summed.
    const std::vector<double> R2 = RotRect(-60, 25, 18, 7, 1.1);
    const Moments m2 = PrismMoments({R2}, 3, 140);
    const double o[3] = {37.5, -12.25, 4.0};
    Moments sum = mL;
    sum += m2;
    {
        Moments direct = PrismMoments({Shift(L, -o[0], -o[1]), Shift(hole, -o[0], -o[1])}, -o[2], 24 - o[2]);
        direct += PrismMoments({Shift(R2, -o[0], -o[1])}, 3 - o[2], 140 - o[2]);
        const double d = MomDiff(sum.About(o), direct);
        check(d < 1e-9, "FOLD: the sum moved is the moved parts summed", d);
    }
    // FRAME as a versor: the translator's sandwich of M is M of the moved moments.
    {
        const double Lu = 100.0;   // a unit length, as the sun's embedding takes one
        const cga::Mv T = cga::Translator(-o[0] / Lu, -o[1] / Lu, -o[2] / Lu);
        const double d = MvDiff(cga::Sandwich(T, sum.Conformal(Lu)), sum.About(o).Conformal(Lu));
        check(d < 1e-12, "FRAME: T M ~T is the parallel-axis theorem", d);
        const cga::Mv Tw = cga::Translator(o[0] / Lu, o[1] / Lu, o[2] / Lu);   // planted: the wrong way
        const double dw = MvDiff(cga::Sandwich(Tw, sum.Conformal(Lu)), sum.About(o).Conformal(Lu));
        check(dw > 1e-3, "FRAME: the gate sees a translator of the wrong sign", dw);
        // ...and a change of unit is the dilator, up to the homogeneous weight.
        const cga::Mv D = cga::Dilator(1.0 / Lu);
        const cga::Mv DM = cga::Sandwich(D, sum.Conformal(1.0));
        const double w = -cga::Dot(DM, cga::Ni()) / sum.m;
        const double dd = MvDiff(DM * (1.0 / w), sum.Conformal(Lu));
        check(dd < 1e-9, "FRAME: a change of unit is the dilator", dd);
    }
    // SPHERE: the centroid is Down(M) and the spread is -M^2 / (M.ni)^2.
    {
        const cga::Mv M = sum.Conformal(1.0);
        double c[3];
        cga::Down(M, c[0], c[1], c[2]);
        double e = 0.0, tr = 0.0;
        for (int k = 0; k < 3; ++k) {
            e = std::max(e, std::abs(c[k] - sum.s[k] / sum.m));
            tr += sum.S[k] / sum.m - (sum.s[k] / sum.m) * (sum.s[k] / sum.m);
        }
        const double mn = cga::Dot(M, cga::Ni());
        const double sigma2 = -cga::Dot(M, M) / (mn * mn);
        check(e < 1e-9, "SPHERE: the centroid is Down(M)", e);
        check(std::abs(sigma2 - tr) / tr < 1e-9, "SPHERE: sigma^2 = -M^2/(M.ni)^2 is the spread", sigma2);
    }
    // The level law: size picks the level, the level its reach.
    {
        const bool lv = LodLevel(3.9, 4) == 0 && LodLevel(8.0, 4) == 1 && LodLevel(1000.0, 4) == 7;
        check(lv, "the level is floor(log2(rho / rho0))", LodLevel(1000.0, 4));
        const double reach = LodReach(8, 4.0, 1.0, 1e-3);
        check(std::abs(reach - 1024000.0) < 1e-6, "level 8 is wanted to 1024 km at 1 px of 1 mrad", reach);
    }
    Log("[lod] selftest %s", ok ? "PASS" : "FAIL");
    return ok;
}

}  // namespace ga
