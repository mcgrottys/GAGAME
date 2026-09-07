// ================================================================================================
//  Cga.h - M9bi: the CONFORMAL geometric algebra Cl(4,1), the algebra the SOLAR SYSTEM needed.
//
//  Pga.h (Cl(3,0,1)) carries rigid motion: motors, the camera, the rails. It cannot carry a
//  SPHERE, and it cannot tell a finite point from a direction -- in a degenerate projective
//  algebra "the sun" and "toward the sun" are the same object. That was fine while the sun was
//  an art direction (Renderer's sunAzimuthDeg/sunElevationDeg, two constants, no date, no
//  place). It is not fine once the sun is a THING at a PLACE: 1.496e11 m away, 6.957e8 m across,
//  moving, and shared by every frame in the scene.
//
//  Cl(4,1) is Euclidean 3-space plus two extra basis vectors, arranged into a NULL PAIR:
//
//      e1^2 = e2^2 = e3^2 = e4^2 = +1,   e5^2 = -1
//      n0 = (e5 - e4)/2   the ORIGIN         n0^2 = 0
//      ni = (e4 + e5)     point at INFINITY  ni^2 = 0,   n0.ni = -1
//
//  and a Euclidean point x embeds as the NULL vector
//
//      P = n0 + x + (1/2) x^2 ni,        P^2 = 0,   P.ni = -1
//
//  from which the whole model follows: P.Q = -(1/2)|p - q|^2 (distance without a square root),
//  a sphere is P_c - (1/2) r^2 ni, a plane is n + d ni, translations/rotations/SCALINGS are all
//  versors acting by the same sandwich V X ~V, and intersections are the outer product. A point
//  and a direction are finally different objects: the direction is the one whose n0 part is zero.
//
//  ---- THE UNIT LENGTH IS PART OF THE MODEL (and it is why this file exists) -------------------
//  The origin n0 carries +-1/2 on e4 and e5, and the embedding's ni term adds x^2/2 to those
//  SAME two coefficients. Once x^2/2 passes 1/(2 eps) the 1/2 is annihilated and P.ni becomes
//  exactly 0 -- and a conformal vector with P.ni = 0 is not an imprecise point, it IS a point
//  at infinity. In metres that happens at |x| = 9.49e7 m: geostationary orbit still works, the
//  MOON does not, and the sun written in metres is a DIRECTION, which is precisely the thing
//  this algebra was brought in to stop being. (Measured, and gated in RunCgaSelfTest -- the
//  first version of this comment argued from ulp size and was simply wrong.)
//
//  So the embedding is always taken at an explicit UNIT LENGTH L, x = p / L, and each space
//  picks its own:
//
//      solar.hci  L = 1 AU        the sun sits at the origin; Earth is 1 unit away
//      planet.ecef L = R_earth    the sun is 23481 units away; the estuary is 1e-7 units
//
//  Changing L is not a reinterpretation of the numbers -- it is a DILATOR, a versor, and the
//  sandwich carries points, spheres and planes across the scale change together. That is the
//  answer to "how does a solar-system constant reach a metre-scale estuary": eleven orders of
//  magnitude as one versor, with every space's own embedding well conditioned.
//
//  Implementation: the FULL algebra, not a closed-form imitation. Basis blades are 5-bit masks,
//  a multivector is 32 doubles, and the geometric product is the standard mask algorithm
//  (reordering sign x the metric of the shared vectors). Everything else -- outer product, left
//  contraction, reverse, exp, the sandwich -- is written in terms of it, so a versor really is a
//  versor and a meet really is a meet. It runs a handful of times per frame on the CPU.
//
//  Gates: RunCgaSelfTest (called from gatest) pins the null identities, the distance law, each
//  versor's action AND that each is a genuine sandwich, the dilator's scale change, and the
//  sphere/plane meet. Ephemeris.h builds the sun on top of it and is pinned against an external
//  solar-noon time.
// ================================================================================================
#pragma once

#include "core/Common.h"

#include <cmath>
#include <cstdint>

namespace ga::cga {

// ---- the algebra -----------------------------------------------------------------------------
// Blade index = 5-bit mask over (e1, e2, e3, e4, e5). Metric is diagonal: the last one squares
// to -1, which is the whole reason a NULL pair exists to be built out of e4 and e5.
inline constexpr int kDim = 5;
inline constexpr int kBlades = 32;
inline constexpr double kMetric[kDim] = {1.0, 1.0, 1.0, 1.0, -1.0};

inline int PopCount(uint32_t v) {
    int n = 0;
    while (v) { v &= v - 1; ++n; }
    return n;
}
inline int Grade(uint32_t mask) { return PopCount(mask); }

// The sign from sorting the concatenated basis vectors of two blades into canonical order:
// every vector of `a` above bit i must hop over the vectors of `b` at or below it.
inline double ReorderSign(uint32_t a, uint32_t b) {
    int swaps = 0;
    a >>= 1;
    while (a) {
        swaps += PopCount(a & b);
        a >>= 1;
    }
    return (swaps & 1) ? -1.0 : 1.0;
}

// The metric factor contributed by the basis vectors the two blades SHARE (each pair collapses
// to its square). Zero is impossible here: this algebra is non-degenerate, unlike Pga.h's.
inline double MetricFactor(uint32_t shared) {
    double m = 1.0;
    for (int i = 0; i < kDim; ++i) {
        if (shared & (1u << i)) m *= kMetric[i];
    }
    return m;
}

struct Mv {
    double c[kBlades] = {};

    double& operator[](uint32_t b) { return c[b]; }
    double operator[](uint32_t b) const { return c[b]; }

    static Mv Scalar(double s) {
        Mv m;
        m.c[0] = s;
        return m;
    }
    static Mv Basis(uint32_t blade, double s = 1.0) {
        Mv m;
        m.c[blade] = s;
        return m;
    }
    Mv operator+(const Mv& o) const {
        Mv r;
        for (int i = 0; i < kBlades; ++i) r.c[i] = c[i] + o.c[i];
        return r;
    }
    Mv operator-(const Mv& o) const {
        Mv r;
        for (int i = 0; i < kBlades; ++i) r.c[i] = c[i] - o.c[i];
        return r;
    }
    Mv operator*(double s) const {
        Mv r;
        for (int i = 0; i < kBlades; ++i) r.c[i] = c[i] * s;
        return r;
    }
    // Grade projection -- the one operation that makes "take the scalar part" a statement about
    // the algebra rather than about an array index.
    Mv Grade(int g) const {
        Mv r;
        for (uint32_t b = 0; b < kBlades; ++b) {
            if (cga::Grade(b) == g) r.c[b] = c[b];
        }
        return r;
    }
    // Reverse: grade g picks up (-1)^(g(g-1)/2). The sandwich needs it and nothing else does.
    Mv Reverse() const {
        Mv r;
        for (uint32_t b = 0; b < kBlades; ++b) {
            const int g = cga::Grade(b);
            r.c[b] = ((g * (g - 1) / 2) & 1) ? -c[b] : c[b];
        }
        return r;
    }
    double Max() const {
        double m = 0.0;
        for (int i = 0; i < kBlades; ++i) m = (std::fabs(c[i]) > m) ? std::fabs(c[i]) : m;
        return m;
    }
};

inline Mv operator*(double s, const Mv& m) { return m * s; }

// THE GEOMETRIC PRODUCT -- 32x32 blade pairs, each one a reorder sign and a metric factor.
inline Mv Gp(const Mv& a, const Mv& b) {
    Mv r;
    for (uint32_t i = 0; i < kBlades; ++i) {
        if (a.c[i] == 0.0) continue;
        for (uint32_t j = 0; j < kBlades; ++j) {
            if (b.c[j] == 0.0) continue;
            const double f = ReorderSign(i, j) * MetricFactor(i & j);
            r.c[i ^ j] += a.c[i] * b.c[j] * f;
        }
    }
    return r;
}
inline Mv operator*(const Mv& a, const Mv& b) { return Gp(a, b); }

// The OUTER product: the same sum, restricted to blade pairs that share no basis vector. This is
// the join -- and, on IPNS objects, the MEET of what they represent.
inline Mv Op(const Mv& a, const Mv& b) {
    Mv r;
    for (uint32_t i = 0; i < kBlades; ++i) {
        if (a.c[i] == 0.0) continue;
        for (uint32_t j = 0; j < kBlades; ++j) {
            if (b.c[j] == 0.0 || (i & j)) continue;
            r.c[i ^ j] += a.c[i] * b.c[j] * ReorderSign(i, j);
        }
    }
    return r;
}
inline Mv operator^(const Mv& a, const Mv& b) { return Op(a, b); }

// The LEFT CONTRACTION a _| b, grade(b) - grade(a). "P lies on the object O" is P _| O == 0 for
// every IPNS object, of any grade -- one incidence test for points, spheres, planes and circles.
inline Mv Lc(const Mv& a, const Mv& b) {
    Mv r;
    for (uint32_t i = 0; i < kBlades; ++i) {
        if (a.c[i] == 0.0) continue;
        for (uint32_t j = 0; j < kBlades; ++j) {
            if (b.c[j] == 0.0) continue;
            const int want = cga::Grade(j) - cga::Grade(i);
            const uint32_t out = i ^ j;
            if (want < 0 || cga::Grade(out) != want) continue;
            r.c[out] += a.c[i] * b.c[j] * ReorderSign(i, j) * MetricFactor(i & j);
        }
    }
    return r;
}

// The scalar product of two grade-1 vectors -- the workhorse: P.Q = -|p-q|^2/2.
inline double Dot(const Mv& a, const Mv& b) { return Gp(a, b).c[0]; }

// exp of a bivector whose square is a SCALAR -- which covers every versor generator this file
// builds: rotors (B^2 < 0, circular), translators (B^2 = 0, the linear case) and dilators
// (E^2 > 0, hyperbolic). One closed form, selected by the sign of the square.
inline Mv Exp(const Mv& B) {
    const double b2 = Gp(B, B).c[0];
    if (b2 < -1e-300) {
        const double t = std::sqrt(-b2);
        return Mv::Scalar(std::cos(t)) + B * (std::sin(t) / t);
    }
    if (b2 > 1e-300) {
        const double t = std::sqrt(b2);
        return Mv::Scalar(std::cosh(t)) + B * (std::sinh(t) / t);
    }
    return Mv::Scalar(1.0) + B;   // null generator: the series terminates after one term
}

// THE SANDWICH. Every transform in this file is this line.
inline Mv Sandwich(const Mv& V, const Mv& X) { return Gp(Gp(V, X), V.Reverse()); }

// ---- the conformal model ---------------------------------------------------------------------
inline const Mv& E1() { static const Mv m = Mv::Basis(1u << 0); return m; }
inline const Mv& E2() { static const Mv m = Mv::Basis(1u << 1); return m; }
inline const Mv& E3() { static const Mv m = Mv::Basis(1u << 2); return m; }
// n0 = (e5 - e4)/2 and ni = e4 + e5: the null pair, hand-checked and gate-checked.
inline const Mv& N0() {
    static const Mv m = Mv::Basis(1u << 4, 0.5) - Mv::Basis(1u << 3, 0.5);
    return m;
}
inline const Mv& Ni() {
    static const Mv m = Mv::Basis(1u << 3) + Mv::Basis(1u << 4);
    return m;
}

// A Euclidean direction as a grade-1 vector (no n0, no ni): a FREE vector, not a place.
inline Mv Dir(double x, double y, double z) { return E1() * x + E2() * y + E3() * z; }

// THE UP MAP. x is in UNIT LENGTHS (see the header): pass p/L, never p.
inline Mv Up(double x, double y, double z) {
    const double x2 = x * x + y * y + z * z;
    return N0() + Dir(x, y, z) + Ni() * (0.5 * x2);
}
inline Mv Up(const Mv& euclideanDir) {
    return Up(euclideanDir[1u << 0], euclideanDir[1u << 1], euclideanDir[1u << 2]);
}

// THE DOWN MAP: normalise by -(P.ni) -- a conformal point is homogeneous, so any positive
// multiple of P is the same place, and forgetting this is the classic CGA bug.
inline void Down(const Mv& P, double& x, double& y, double& z) {
    const double w = -Dot(P, Ni());
    const double s = (std::fabs(w) > 1e-300) ? 1.0 / w : 1.0;
    x = P[1u << 0] * s;
    y = P[1u << 1] * s;
    z = P[1u << 2] * s;
}

// NORMALISE a conformal point back to P.ni = -1. A conformal point is HOMOGENEOUS -- any
// scalar multiple is the same place -- and the versors do not all preserve the scale: a rotor
// and a translator do, but the DILATOR does not, because scaling the space is exactly what it
// is for. Every constructor below that reads a radius or an offset out of a vector (DualSphere,
// and Ephemeris.h's terminator) assumes the -1, so a dilated point must come through here
// first. Skipping this makes a sphere whose radius is wrong by sqrt(scale), which is how the
// terminator gate first failed. See docs/ALGEBRA.md priors.
inline Mv Normalize(const Mv& P) {
    const double w = -Dot(P, Ni());
    return (std::fabs(w) > 1e-300) ? P * (1.0 / w) : P;
}

// RE-PROJECT a drifted point back onto the null cone, by rebuilding its ni coefficient from
// its Euclidean part. This is the conformal analogue of re-orthonormalising a rotation matrix
// after a long product, and it is needed for the same reason: the ni coefficient carries |x|^2,
// so a DILATOR by s multiplies its error by s^2. Pushing the sun 1 AU through a 23481x dilation
// leaves an ni residue that an inner product reads as a 1.5 km displacement while the Euclidean
// part is still exact -- so the fix is to rebuild the coefficient the products degraded, not to
// widen the tolerance of everything downstream. Gated: the gate reports how far it moved.
inline Mv Reproject(const Mv& P) {
    double x, y, z;
    Down(P, x, y, z);
    return Up(x, y, z);
}

// Distance between two conformal points, in unit lengths. No square root inside the algebra --
// the square root is only there to turn the invariant back into a length.
inline double Distance(const Mv& P, const Mv& Q) {
    const double wp = -Dot(P, Ni()), wq = -Dot(Q, Ni());
    const double d2 = -2.0 * Dot(P, Q) / ((wp != 0.0 ? wp : 1.0) * (wq != 0.0 ? wq : 1.0));
    return std::sqrt(d2 > 0.0 ? d2 : 0.0);
}

// A SPHERE, in its dual (IPNS) form: centre point minus half the squared radius times infinity.
// The sun IS one of these; so is the Earth. A point lies on it iff P _| S == 0.
inline Mv DualSphere(const Mv& centre, double radius) {
    return centre - Ni() * (0.5 * radius * radius);
}
// A PLANE: unit normal plus signed distance from the origin times infinity.
inline Mv DualPlane(double nx, double ny, double nz, double d) {
    return Dir(nx, ny, nz) + Ni() * d;
}

// ---- the versors ------------------------------------------------------------------------------
// TRANSLATOR: T = exp(-t ni / 2) = 1 - t ni / 2 (the generator is null, so the series stops).
inline Mv Translator(double tx, double ty, double tz) {
    return Exp(Op(Dir(tx, ty, tz), Ni()) * -0.5);
}
// ROTOR about a unit Euclidean axis. The generator is the DUAL bivector of the axis, so that a
// positive angle is a right-handed turn about it -- the same convention Pga.h pins.
inline Mv Rotor(double ax, double ay, double az, double angle) {
    // The bivector orthogonal to the axis: e23 for x, e31 for y, e12 for z.
    const Mv B = Op(E2(), E3()) * ax + Op(E3(), E1()) * ay + Op(E1(), E2()) * az;
    return Exp(B * (-0.5 * angle));
}
// DILATOR: the versor that CHANGES THE UNIT LENGTH. Its generator is the null pair's own
// bivector E = n0 ^ ni, with E^2 = +1 -- the hyperbolic case of Exp above.
inline Mv Dilator(double scale) {
    return Exp(Op(N0(), Ni()) * (0.5 * std::log(scale)));
}

// ---- the gate ---------------------------------------------------------------------------------
// Pins the MODEL, so everything built on it (Ephemeris.h, the sun) rests on checked ground.
// Called by RunGaSelfTest; the sun's own gate lives in GaTest.cpp beside the other water and
// frame parities. See docs/ALGEBRA.md "cga".
inline bool RunCgaSelfTest() {
    bool ok = true;
    int checks = 0;
    auto nearly = [&](double a, double b, double tol, const char* what) {
        ++checks;
        if (std::fabs(a - b) <= tol) return;
        Log("[cga] FAIL %s: %.17g vs %.17g (tol %g)", what, a, b, tol);
        ok = false;
    };
    // A deterministic sequence -- this gate must not depend on a library RNG's version.
    uint32_t st = 0x9b1e5f35u;
    auto rnd = [&] {
        st = st * 1664525u + 1013904223u;
        return (double(st >> 8) / double(1u << 24)) * 2.0 - 1.0;
    };

    // ---- 1. THE NULL PAIR. Everything else is a consequence of these three numbers.
    nearly(Dot(N0(), N0()), 0.0, 1e-15, "n0^2 = 0");
    nearly(Dot(Ni(), Ni()), 0.0, 1e-15, "ni^2 = 0");
    nearly(Dot(N0(), Ni()), -1.0, 1e-15, "n0.ni = -1");

    // ---- 2. THE UP MAP: points are NULL, and normalised so that P.ni = -1. The second is what
    // makes a conformal point a PLACE rather than a ray of scalar multiples.
    for (int i = 0; i < 64; ++i) {
        const double x = rnd() * 3.0, y = rnd() * 3.0, z = rnd() * 3.0;
        const Mv P = Up(x, y, z);
        nearly(Dot(P, P), 0.0, 1e-12, "P^2 = 0 (the point is null)");
        nearly(Dot(P, Ni()), -1.0, 1e-15, "P.ni = -1 (normalised)");
        double dx, dy, dz;
        Down(P, dx, dy, dz);
        nearly(dx, x, 1e-14, "down(up(x)) = x");
        nearly(dy, y, 1e-14, "down(up(y)) = y");
        nearly(dz, z, 1e-14, "down(up(z)) = z");
        // ...and a SCALED point is the same place: homogeneity, the classic CGA trap.
        double sx, sy, sz;
        Down(P * 7.5, sx, sy, sz);
        nearly(sx, x, 1e-13, "down(7.5 P) = x (homogeneous)");
    }

    // ---- 3. THE DISTANCE LAW: P.Q = -|p-q|^2 / 2. Distance without a square root, and the
    // reason spheres, planes and circles can all be tested for incidence the same way.
    for (int i = 0; i < 64; ++i) {
        const double a0 = rnd() * 5.0, a1 = rnd() * 5.0, a2 = rnd() * 5.0;
        const double b0 = rnd() * 5.0, b1 = rnd() * 5.0, b2 = rnd() * 5.0;
        const double d2 = (a0 - b0) * (a0 - b0) + (a1 - b1) * (a1 - b1) + (a2 - b2) * (a2 - b2);
        nearly(Dot(Up(a0, a1, a2), Up(b0, b1, b2)), -0.5 * d2, 1e-12, "P.Q = -d^2/2");
        nearly(Distance(Up(a0, a1, a2), Up(b0, b1, b2)), std::sqrt(d2), 1e-12, "Distance()");
    }

    // ---- 4. THE VERSORS. Each is checked twice: that its SANDWICH lands where the closed form
    // says, and that it is a genuine versor (the point stays null; the dilator scales, which is
    // the whole point of it).
    for (int i = 0; i < 32; ++i) {
        const double x = rnd() * 4.0, y = rnd() * 4.0, z = rnd() * 4.0;
        const Mv P = Up(x, y, z);

        // TRANSLATOR
        const double t0 = rnd() * 6.0, t1 = rnd() * 6.0, t2 = rnd() * 6.0;
        const Mv TP = Sandwich(Translator(t0, t1, t2), P);
        nearly(Dot(TP, TP), 0.0, 1e-10, "translator keeps the point null");
        double tx, ty, tz;
        Down(TP, tx, ty, tz);
        nearly(tx, x + t0, 1e-12, "translator moves x");
        nearly(ty, y + t1, 1e-12, "translator moves y");
        nearly(tz, z + t2, 1e-12, "translator moves z");

        // ROTOR, against Rodrigues -- and the HANDEDNESS, which is the part that silently
        // mirrors a sky if it is wrong.
        double ax = rnd(), ay = rnd(), az = rnd() + 1.5;
        const double al = std::sqrt(ax * ax + ay * ay + az * az);
        ax /= al; ay /= al; az /= al;
        const double th = rnd() * 3.0;
        double rx, ry, rz;
        Down(Sandwich(Rotor(ax, ay, az, th), P), rx, ry, rz);
        const double cth = std::cos(th), sth = std::sin(th);
        const double dt = ax * x + ay * y + az * z;
        const double crx = ay * z - az * y, cry = az * x - ax * z, crz = ax * y - ay * x;
        nearly(rx, x * cth + crx * sth + ax * dt * (1 - cth), 1e-11, "rotor = Rodrigues (x)");
        nearly(ry, y * cth + cry * sth + ay * dt * (1 - cth), 1e-11, "rotor = Rodrigues (y)");
        nearly(rz, z * cth + crz * sth + az * dt * (1 - cth), 1e-11, "rotor = Rodrigues (z)");

        // DILATOR: THE CHANGE OF UNIT LENGTH. It must scale the coordinate by exactly s --
        // eleven orders of magnitude ride on this sign.
        const double sc = 3.7;
        double dx2, dy2, dz2;
        Down(Sandwich(Dilator(sc), P), dx2, dy2, dz2);
        nearly(dx2, x * sc, 1e-10, "dilator scales x by s");
        nearly(dy2, y * sc, 1e-10, "dilator scales y by s");
        nearly(dz2, z * sc, 1e-10, "dilator scales z by s");
        // ...and the TRAP, which the terminator gate caught the hard way: the dilator is the
        // one versor here that does NOT preserve P.ni = -1, because rescaling the space is
        // exactly its job. Down() renormalises, so the three checks above never see it -- but
        // DualSphere reads a radius straight out of the vector, and an un-normalised centre
        // yields a sphere of radius r/sqrt(lambda). Assert the scale really is lost, that
        // Normalize restores it, and that a sphere on the normalised centre is the right size.
        {
            const Mv DP = Sandwich(Dilator(sc), P);
            ++checks;
            if (std::fabs(-Dot(DP, Ni()) - 1.0) < 1e-9) {
                Log("[cga] FAIL the dilator was expected NOT to preserve P.ni = -1 -- if it "
                    "now does, the normalisation trap below is no longer the real hazard");
                ok = false;
            }
            nearly(-Dot(Normalize(DP), Ni()), 1.0, 1e-12, "Normalize restores P.ni = -1");
            double ox, oy, oz;
            Down(Normalize(DP), ox, oy, oz);
            nearly(Lc(Up(ox + 2.0, oy, oz), DualSphere(Normalize(DP), 2.0)).Max(), 0.0, 1e-9,
                   "a sphere on a NORMALISED dilated centre has the radius it was asked for");
        }

        // COMPOSITION: the sandwich of a product is the product of the sandwiches. Without this
        // the four-versor frame chain in Ephemeris.h would not be a chain at all.
        const Mv V1 = Translator(t0, t1, t2);
        const Mv V2 = Rotor(ax, ay, az, th);
        double c1x, c1y, c1z, c2x, c2y, c2z;
        Down(Sandwich(V2, Sandwich(V1, P)), c1x, c1y, c1z);
        Down(Sandwich(Gp(V2, V1), P), c2x, c2y, c2z);
        nearly(c1x, c2x, 1e-11, "sandwich(V2 V1) = sandwich V2 of sandwich V1");
        nearly(c1y, c2y, 1e-11, "sandwich composes (y)");
        nearly(c1z, c2z, 1e-11, "sandwich composes (z)");
    }

    // ---- 5. ROUNDS AND FLATS, and the ONE incidence test. A point lies on any IPNS object --
    // sphere, plane, or the CIRCLE that is their meet -- iff the left contraction vanishes.
    {
        const Mv C = Up(1.0, -2.0, 0.5);
        const double R = 3.0;
        const Mv S = DualSphere(C, R);
        for (int i = 0; i < 16; ++i) {
            double ux = rnd(), uy = rnd(), uz = rnd() + 0.3;
            const double ul = std::sqrt(ux * ux + uy * uy + uz * uz);
            ux /= ul; uy /= ul; uz /= ul;
            nearly(Lc(Up(1.0 + R * ux, -2.0 + R * uy, 0.5 + R * uz), S).Max(), 0.0, 1e-11,
                 "point on sphere: P _| S = 0");
            ++checks;
            if (Lc(Up(1.0 + 1.01 * R * ux, -2.0 + 1.01 * R * uy, 0.5 + 1.01 * R * uz), S).Max() <
                1e-3) {
                Log("[cga] FAIL a point off the sphere still satisfies P _| S = 0");
                ok = false;
            }
        }
        // THE MEET: the circle where the sphere cuts the plane z = 1.7 is S ^ pi, and the points
        // of that circle are exactly the ones both objects accept.
        const double zc = 0.5 + 1.2;
        const Mv Pi = DualPlane(0.0, 0.0, 1.0, zc);
        const Mv Circle = Op(S, Pi);
        const double rr = std::sqrt(R * R - 1.2 * 1.2);
        for (int i = 0; i < 16; ++i) {
            const double ph = double(i) * 0.3926990816987241;
            const Mv Q = Up(1.0 + rr * std::cos(ph), -2.0 + rr * std::sin(ph), zc);
            nearly(Lc(Q, Pi).Max(), 0.0, 1e-12, "point on plane: P _| pi = 0");
            nearly(Lc(Q, S).Max(), 0.0, 1e-10, "circle point is on the sphere");
            nearly(Lc(Q, Circle).Max(), 0.0, 1e-10, "point on the MEET: P _| (S ^ pi) = 0");
        }
        ++checks;
        if (Lc(Up(1.0, -2.0 + R, 0.5), Circle).Max() < 1e-3) {   // on the sphere, off the plane
            Log("[cga] FAIL a point off the circle still satisfies P _| (S ^ pi) = 0");
            ok = false;
        }
    }

    // ---- 6. WHY THE UNIT LENGTH IS PART OF THE MODEL (docs/ALGEBRA.md priors). This is not a
    // rounding argument, it is a TYPE argument. The origin n0 carries +-1/2 on e4 and e5, and
    // the embedding's ni term adds x^2/2 to BOTH of those same coefficients. Once x^2/2 passes
    // 1/(2 eps) the 1/2 is annihilated outright, P.ni goes to exactly 0 -- and a conformal
    // vector with P.ni = 0 is not an inaccurate point, it IS a point at infinity. The model
    // collapses, silently, back into the direction it exists to improve on.
    //
    // The threshold in metres is |x| = 9.49e7 m: geostationary orbit still works, the Moon
    // does not, and the sun is long gone. That single number is why every space in this engine
    // declares its own unit length instead of "just using metres".
    {
        const double au = 1.495978707e11;
        nearly(Dot(Up(4.2164e7, 0.0, 0.0), Ni()), -1.0, 0.0,
               "geostationary orbit in METRES is still a place (P.ni = -1)");
        nearly(Dot(Up(3.844e8, 0.0, 0.0), Ni()), 0.0, 0.0,
               "the Moon in METRES has already collapsed (P.ni = 0)");
        nearly(Dot(Up(au, 0.0, 0.0), Ni()), 0.0, 0.0,
               "the sun in METRES is a DIRECTION, not a point (P.ni = 0)");
        nearly(Dot(Up(1.0, 0.0, 0.0), Ni()), -1.0, 0.0,
               "the sun at unit length 1 AU is a place (P.ni = -1)");
        nearly(Dot(Up(au / 6371000.0, 0.0, 0.0), Ni()), -1.0, 0.0,
               "the sun at unit length R_earth is a place (P.ni = -1)");
        Log("[cga] unit length: in metres a conformal point stops being a point past 9.49e7 m "
            "(geostationary survives, the Moon does not, the sun reads P.ni = 0 = infinity). "
            "At unit length 1 AU or 1 R_earth the same sun reads P.ni = -1. The SPACE is part "
            "of the model, and the dilator is how you change it.");
    }

    if (ok) {
        Log("[cga] Cl(4,1) self-test: PASS (%d checks -- null pair, up/down + homogeneity, "
            "P.Q distance law, translator/rotor/dilator sandwiches and their composition, "
            "sphere/plane/meet incidence, unit-length conditioning)", checks);
    }
    return ok;
}

}  // namespace ga::cga
