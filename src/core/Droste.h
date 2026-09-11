// ================================================================================================
//  Droste.h - M10: THE GLOBE WITHIN THE GLOBE. The root's address, hung as a leaf.
//
//  The planet is a sparse quadtree over six cube faces; every node is an ADDRESS (face, level,
//  x, y) and the residency manager streams whatever the walk asks for at that address. The
//  Droste effect here is one edit to that structure: a single leaf gets the ROOT as its child.
//  The tree becomes a graph with one cycle, and descending through the cycle arrives back at
//  the whole planet -- smaller, sitting in the leaf, and carrying its own leaf, forever.
//
//  WHY IT COSTS NOTHING TO STORE. The inner planet is not a copy. Its tiles ARE the root's
//  tiles -- the same addresses in the same tenants -- so the residency manager sees one set of
//  wants from every level at once, and a small globe simply asks for coarser mips of the same
//  tiles. The recursion is bounded by the screen, not by memory: once a level is smaller than
//  a pixel its walk emits nothing and the cycle is never taken again. That is what "sparse" was
//  for (the user, 2026-09-10: "it's sparse for a reason").
//
//  THE ALGEBRA. The link from the root to the leaf is ONE conformal versor of Cl(4,1) (Cga.h):
//
//      S = T(p) R(axis, twist) D(s) T(-p),      x  ->  p + s Q (x - p)
//
//  a dilation by s about the FIXED POINT p, with a twist Q about an axis through it. PGA cannot
//  write this -- a motor has no scale -- and that is the whole reason the link lives in the
//  conformal model: the root's sphere and the leaf's globe are the same DualSphere pushed
//  through one sandwich, radius included. Level k is S^k; fractional powers S^t = exp(t log S)
//  are the one-parameter subgroup whose orbits are LOGARITHMIC SPIRALS converging on p -- the
//  camera path of the infinite dive is one of them, and nothing else.
//
//  THE GAUGE. Angles are invariant under a similarity, so rendering level k from the eye C is
//  EXACTLY rendering the root from S^-k(C):
//
//      S^k(y) - C  =  s^k Q^k (y - S^-k(C))
//
//  (camera-relative vectors of level k are the root's own, rotated and scaled). The walk runs
//  the same tree again under the transformed eye, the shading runs unchanged in the level's own
//  frame, and only the clip position is mapped back. The floating origin grows a FLOATING
//  SCALE: when the nearest ground belongs to another level the frame re-roots there,
//  C <- S^-1(C), which changes nothing on screen -- a gauge change, exactly as the floating
//  origin's translation is -- and keeps every double near the scale of the world it describes.
//
//  Gates: RunDrosteSelfTest (from gatest) -- the closed form against the sandwich, the portal
//  sphere's radius read back out of the transformed DualSphere, the fixed point, the inverse,
//  powers and the fractional subgroup, the rest contact, and the gauge identity above.
// ================================================================================================
#pragma once

#include "core/Cga.h"
#include "core/Common.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <vector>

namespace ga::droste {

// Rodrigues: the rotation by `t` radians, right-handed about the unit axis `a`. Row-major.
inline void RotMat(const double a[3], double t, double m[3][3]) {
    const double c = std::cos(t), s = std::sin(t), k = 1.0 - c;
    m[0][0] = c + a[0] * a[0] * k;
    m[0][1] = a[0] * a[1] * k - a[2] * s;
    m[0][2] = a[0] * a[2] * k + a[1] * s;
    m[1][0] = a[1] * a[0] * k + a[2] * s;
    m[1][1] = c + a[1] * a[1] * k;
    m[1][2] = a[1] * a[2] * k - a[0] * s;
    m[2][0] = a[2] * a[0] * k - a[1] * s;
    m[2][1] = a[2] * a[1] * k + a[0] * s;
    m[2][2] = c + a[2] * a[2] * k;
}
inline void MatVec(const double m[3][3], const double v[3], double o[3]) {
    const double x = m[0][0] * v[0] + m[0][1] * v[1] + m[0][2] * v[2];
    const double y = m[1][0] * v[0] + m[1][1] * v[1] + m[1][2] * v[2];
    const double z = m[2][0] * v[0] + m[2][1] * v[1] + m[2][2] * v[2];
    o[0] = x;
    o[1] = y;
    o[2] = z;
}

// The link, resolved. Everything is in the ROOT'S tangent frame (the engine's one-world frame:
// metres, x east, y up, z north at the anchor, planet centre at (0, -R, 0)) -- and, by the
// self-similarity, in EVERY level's own tangent frame too: the numbers below describe how level
// k+1 sits inside level k for every k.
struct Portal {
    // THE ADDRESS: the leaf of the planet's quadtree whose child is the root.
    int face = -1, level = 0;
    uint32_t ix = 0, iy = 0;
    double R = 6371000.0;
    double s = 1.0;                   // scale per level: the inner globe's radius / R
    double axis[3] = {0.0, 0.0, 1.0};  // the twist axis (unit)
    double twist = 0.0;               // radians per level
    double p[3] = {0.0, 0.0, 0.0};    // THE FIXED POINT: S(p) = p -- where the tower converges
    double centre[3] = {0.0, 0.0, 0.0};   // the inner globe S(root), in its parent's frame
    double radius = 0.0;
    double groundUp[3] = {0.0, 1.0, 0.0};   // the leaf's local up (the rest contact's normal)
    cga::Mv S, Sinv;                  // the versor and its inverse, at unit length R
    bool Valid() const { return face >= 0 && s > 0.0 && s < 1.0; }

    // The scale and rotation of S^k (k real: fractional k is the one-parameter subgroup).
    double Scale(double k) const { return std::pow(s, k); }
    void Rot(double k, double m[3][3]) const { RotMat(axis, twist * k, m); }
    // S^k on a point: p + s^k Q^k (x - p).
    void Apply(double k, const double x[3], double out[3]) const {
        double m[3][3];
        Rot(k, m);
        const double d[3] = {x[0] - p[0], x[1] - p[1], x[2] - p[2]};
        double r[3];
        MatVec(m, d, r);
        const double sk = Scale(k);
        out[0] = p[0] + sk * r[0];
        out[1] = p[1] + sk * r[1];
        out[2] = p[2] + sk * r[2];
    }
    // S^k on a free direction: rotation only (a similarity keeps directions unit).
    void ApplyDir(double k, const double d[3], double out[3]) const {
        double m[3][3];
        Rot(k, m);
        MatVec(m, d, out);
    }
    // Level k's planet in the frame of level 0: S^k of the sphere (centre (0,-R,0), R).
    void Globe(double k, double c[3], double& r) const {
        const double c0[3] = {0.0, -R, 0.0};
        Apply(k, c0, c);
        r = R * Scale(k);
    }
};

// ---- the versor ------------------------------------------------------------------------------
// Built at unit length R (priors 32: a conformal point in METRES stops being a point past
// 9.49e7 m; the scene's objects sit near |x| ~ 1 R here).
inline cga::Mv SimilarityVersor(const double p[3], double s, const double axis[3], double twist,
                                double R) {
    using namespace cga;
    const Mv Tp = Translator(p[0] / R, p[1] / R, p[2] / R);
    const Mv Tm = Translator(-p[0] / R, -p[1] / R, -p[2] / R);
    const Mv Rq = Rotor(axis[0], axis[1], axis[2], twist);
    const Mv Ds = Dilator(s);
    // Rightmost acts first: T(-p), then the dilator and the rotor (they commute -- both are
    // generated at the origin, in orthogonal planes), then T(p).
    return Gp(Gp(Gp(Tp, Rq), Ds), Tm);
}

// A point through a versor, with the two priors-33 repairs every dilated point needs: the
// dilator does not keep P.ni = -1 (Normalize), and it multiplies the ni coefficient's error by
// scale^2 (Reproject rebuilds it from the Euclidean part).
inline void SandwichPoint(const cga::Mv& V, const double x[3], double R, double out[3]) {
    using namespace cga;
    const Mv P = Reproject(Normalize(Sandwich(V, Up(x[0] / R, x[1] / R, x[2] / R))));
    Down(P, out[0], out[1], out[2]);
    out[0] *= R;
    out[1] *= R;
    out[2] *= R;
}

// THE PORTAL AT A LEAF. `dirPlanet` is the leaf centre's unit direction in the PLANET frame
// (the quad-sphere's own CubeDir), the frame rows E/U/N take planet -> tangent, `ground` is the
// composed height at the leaf centre (the inner globe RESTS on it), `fill` is the globe's
// diameter over the leaf's ground span (1 = the leaf exactly). The twist is (axis, radians).
//
// The rest contact decides the centre; the scale is radius / R; and the fixed point is then
// FORCED -- solve (I - sQ) p = c1 - sQ c0 -- because S must take the root's centre c0 onto the
// globe's centre c1. Nothing about p is chosen: it is where the geometry says the tower ends.
inline Portal BuildPortal(int face, int level, uint32_t ix, uint32_t iy, const double dirPlanet[3],
                          const double E[3], const double U[3], const double N[3], double R,
                          double ground, double fill, const double axisT[3], double twistRad) {
    Portal pt;
    pt.face = face;
    pt.level = level;
    pt.ix = ix;
    pt.iy = iy;
    pt.R = R;
    // The leaf centre on the sphere, and its local up, in the tangent frame.
    const double up[3] = {E[0] * dirPlanet[0] + E[1] * dirPlanet[1] + E[2] * dirPlanet[2],
                          U[0] * dirPlanet[0] + U[1] * dirPlanet[1] + U[2] * dirPlanet[2],
                          N[0] * dirPlanet[0] + N[1] * dirPlanet[1] + N[2] * dirPlanet[2]};
    for (int i = 0; i < 3; ++i) pt.groundUp[i] = up[i];
    const double span = (3.14159265358979323846 / 2.0) * R / double(1u << level);
    pt.radius = 0.5 * fill * span;
    pt.s = pt.radius / R;
    const double lift = R + ground + pt.radius;   // centre: the ground point + one radius of up
    pt.centre[0] = up[0] * lift;
    pt.centre[1] = up[1] * lift - R;
    pt.centre[2] = up[2] * lift;
    // The twist axis, unit.
    const double al =
        std::sqrt(axisT[0] * axisT[0] + axisT[1] * axisT[1] + axisT[2] * axisT[2]);
    for (int i = 0; i < 3; ++i) pt.axis[i] = (al > 1e-12) ? axisT[i] / al : (i == 2 ? 1.0 : 0.0);
    pt.twist = twistRad;
    // THE FIXED POINT: (I - sQ) p = c1 - sQ c0, c0 = (0, -R, 0). Solved by Cramer on the 3x3 --
    // well conditioned for any s < 1 (the eigenvalues of sQ all have modulus s).
    double Q[3][3];
    RotMat(pt.axis, pt.twist, Q);
    const double c0[3] = {0.0, -R, 0.0};
    double qc0[3];
    MatVec(Q, c0, qc0);
    const double b[3] = {pt.centre[0] - pt.s * qc0[0], pt.centre[1] - pt.s * qc0[1],
                         pt.centre[2] - pt.s * qc0[2]};
    double A[3][3];
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) A[r][c] = (r == c ? 1.0 : 0.0) - pt.s * Q[r][c];
    }
    auto det3 = [](const double M[3][3]) {
        return M[0][0] * (M[1][1] * M[2][2] - M[1][2] * M[2][1]) -
               M[0][1] * (M[1][0] * M[2][2] - M[1][2] * M[2][0]) +
               M[0][2] * (M[1][0] * M[2][1] - M[1][1] * M[2][0]);
    };
    const double dA = det3(A);
    for (int k = 0; k < 3; ++k) {
        double Ak[3][3];
        for (int r = 0; r < 3; ++r) {
            for (int c = 0; c < 3; ++c) Ak[r][c] = (c == k) ? b[r] : A[r][c];
        }
        pt.p[k] = det3(Ak) / dA;
    }
    pt.S = SimilarityVersor(pt.p, pt.s, pt.axis, pt.twist, R);
    pt.Sinv = SimilarityVersor(pt.p, 1.0 / pt.s, pt.axis, -pt.twist, R);
    return pt;
}

// ---- the floating scale ----------------------------------------------------------------------
// Distance from a point (current frame) to level k's planet surface, sphere-approximate: the
// ground the camera would fall onto. Positive outside.
inline double SurfaceDistance(const Portal& pt, double k, const double x[3]) {
    double c[3], r;
    pt.Globe(k, c, r);
    const double dx = x[0] - c[0], dy = x[1] - c[1], dz = x[2] - c[2];
    return std::sqrt(dx * dx + dy * dy + dz * dz) - r;
}

// THE WEIGHT OF A GROUND, as the eye at height h above it feels it (planet radius r): 1/h^2 near
// the surface -- the ground you would fall onto dominates, which is what the roll, the speed and
// the backdrop all need -- and (r/h)^2 less again once the planet is small in the sky. One
// expression. The second factor is what makes the field GAUGE-INVARIANT: every inner globe nests
// at the same spot, so seen from farther away than they are wide they all stand at the same h,
// and 1/h^2 alone counts each of them -- as many as a camera-relative window happens to include.
// MEASURED (M10): an eye over the inner globe's own Merrimack re-rooted TWO levels (the next
// globe's top stands a hair nearer than the ground it rests on), its window (-1..+1) lost the
// root it stood 71 m above, and the appealing sky went black. With the size in, the tower sums
// to its largest member, whatever the window. Homogeneous of degree -2: a gauge change scales
// every weight alike, and every rule below normalizes it away.
inline double GroundWeight(double h, double r) { return (r * r) / (h * h * (r * r + h * h)); }

// THE OWNING GROUND: which of the levels (camLevel - 1, camLevel, camLevel + 1) owns the eye.
// This is the gravity analog: the camera belongs to the ground it would fall onto, and its
// frame, speed and roll reference follow that ground. "Would fall onto" is the ground that
// WEIGHS most -- near any surface simply the nearest one, but a globe smaller than its distance
// never takes the eye. MEASURED (M10, the gate below): by bare nearest surface, an eye 40 m over
// the inner globe's own Merrimack stepped in to level 2 under the quarter twist, and without the
// twist -- on the tower's axis, where every level's next globe stands a hair nearer than the
// ground it rests on -- to level 4 in one frame, the guard's limit, and on again the next.
// Returns -1, 0 or +1 (relative), never a level above the root (camLevelAbs is the camera's
// absolute level).
inline int NearestLevel(const Portal& pt, int camLevelAbs, const double x[3]) {
    auto weight = [&](double k) {
        double c[3], r;
        pt.Globe(k, c, r);
        const double dx = x[0] - c[0], dy = x[1] - c[1], dz = x[2] - c[2];
        const double h = (std::max)(std::fabs(std::sqrt(dx * dx + dy * dy + dz * dz) - r), 1e-6 * r);
        return GroundWeight(h, r);
    };
    int best = 0;
    double wBest = weight(0.0);
    const double wIn = weight(1.0);
    if (wIn > wBest) {
        best = 1;
        wBest = wIn;
    }
    if (camLevelAbs > 0 && weight(-1.0) > wBest) best = -1;
    return best;
}

// The levels every camera-derived rule reads, relative to the camera's: two out (never above the
// root) and two in -- wide enough that a re-root of one step, or two over a Merrimack, drops no
// ground that still weighs anything.
inline int LevelLo(int camLevelAbs) { return -(std::min)(camLevelAbs, 2); }
constexpr int kLevelHi = 2;

// THE GROUND FIELD at x: one law for everything the camera reads from the tower.
struct GroundField {
    double up[3] = {0.0, 1.0, 0.0};   // anti-gravity: each planet's radial by its weight (the roll)
    double lambda = 1.0;              // (sum w)^-1/2, the local length scale (current-frame units)
    double W = 0.0;                   // the SPACE backdrop's weight: sum w sp / sum w
    double ownShare = 1.0;            // the camera level's own part of that space
    // WHOSE sky each backdrop is (relative levels): the dome belongs to the ground that calls
    // for a dome the loudest (max w (1 - sp)), space to the one that calls for space (max w sp).
    // Either choice changes hands only where its own weight is ~0 -- and neither is "the
    // camera's level", which is the gauge (the dome drawn in the camera's frame swung 60 deg at
    // the re-root, MEASURED as a grey below-horizon sky in the inner globe's orbit).
    int domeRel = 0;
    int spaceRel = 0;
};
inline GroundField Grounds(const Portal& pt, int camLevelAbs, const double x[3]) {
    GroundField f;
    double gSum = 0.0, wSum = 0.0, wOwn = 0.0, u[3] = {0.0, 0.0, 0.0};
    double domeBest = -1.0, spaceBest = -1.0;
    for (int k = LevelLo(camLevelAbs); k <= kLevelHi; ++k) {
        double c[3], r;
        pt.Globe(double(k), c, r);
        const double d[3] = {x[0] - c[0], x[1] - c[1], x[2] - c[2]};
        const double dl = std::sqrt(d[0] * d[0] + d[1] * d[1] + d[2] * d[2]);
        const double h = (std::max)(std::fabs(dl - r), 1e-6 * r);
        const double g = GroundWeight(h, r);
        for (int i = 0; i < 3; ++i) u[i] += d[i] / (std::max)(dl, 1e-300) * g;
        // The backdrop that level's OWN altitude calls for: its dome below ~6 km of its own
        // units, space above ~12 km -- an altitude in own units is a ratio, blind to the gauge.
        double sp = std::clamp(((dl - r) * pt.R / r - 6000.0) / 6000.0, 0.0, 1.0);
        sp = sp * sp * (3.0 - 2.0 * sp);
        gSum += g;
        wSum += g * sp;
        if (k == 0) wOwn = g * sp;
        if (g * (1.0 - sp) > domeBest) {
            domeBest = g * (1.0 - sp);
            f.domeRel = k;
        }
        if (g * sp > spaceBest) {
            spaceBest = g * sp;
            f.spaceRel = k;
        }
    }
    const double un = std::sqrt(u[0] * u[0] + u[1] * u[1] + u[2] * u[2]);
    if (un > 0.0) {
        for (int i = 0; i < 3; ++i) f.up[i] = u[i] / un;
    }
    f.lambda = 1.0 / std::sqrt((std::max)(gSum, 1e-300));
    f.W = wSum / (std::max)(gSum, 1e-300);
    f.ownShare = (wSum > 0.0) ? wOwn / wSum : 1.0;
    return f;
}

// The local length scale of the Droste world at x: the soft-min of the distances to every
// ground within reach, by the same weights. One continuous expression (no switch), homogeneous
// of degree one -- scaling the world scales it -- and its limits are the physics: near any
// surface it IS the distance to that surface, so a camera moving at speed ~ lambda approaches
// every ground exponentially and never touches it. The logarithm the user asked for lives
// here: d(ln lambda)/dt is constant on an approach.
inline double LocalScale(const Portal& pt, int camLevelAbs, const double x[3]) {
    return Grounds(pt, camLevelAbs, x).lambda;
}

// ---- the gate --------------------------------------------------------------------------------
inline bool RunDrosteSelfTest() {
    bool ok = true;
    int checks = 0;
    auto nearly = [&](double a, double b, double tol, const char* what) {
        ++checks;
        if (std::fabs(a - b) <= tol) return;
        Log("[droste] FAIL %s: %.17g vs %.17g (tol %g)", what, a, b, tol);
        ok = false;
    };
    // A leaf near the Merrimack mouth, as main builds it: face 5 (lon -70 lives there, per the
    // composetest note), level 16, and a frame at 42.8 N 70.8 W. The numbers only need to be
    // representative; the identities are exact for any portal.
    const double R = 6371000.0;
    const double lat = 42.8183 * 3.14159265358979 / 180.0, lon = -70.81 * 3.14159265358979 / 180.0;
    // The engine's planet frame: x at 0N 0E, y at the pole, z at 90E (LEFT-handed, priors 34).
    const double o[3] = {std::cos(lat) * std::cos(lon), std::sin(lat), std::cos(lat) * std::sin(lon)};
    const double yl = std::sqrt(o[0] * o[0] + o[2] * o[2]);
    const double E[3] = {-o[2] / yl, 0.0, o[0] / yl};
    const double N[3] = {E[1] * o[2] - E[2] * o[1], E[2] * o[0] - E[0] * o[2],
                         E[0] * o[1] - E[1] * o[0]};
    // A leaf direction ~780 m east of the anchor.
    const double d0 = 780.0 / R;
    double dl[3] = {o[0] + E[0] * d0, o[1] + E[1] * d0, o[2] + E[2] * d0};
    const double dn = std::sqrt(dl[0] * dl[0] + dl[1] * dl[1] + dl[2] * dl[2]);
    for (double& v : dl) v /= dn;
    const double axisN[3] = {0.0, 0.0, 1.0};
    for (int variant = 0; variant < 2; ++variant) {
        const double twist = variant ? 1.5707963267948966 : 0.0;
        const Portal pt = BuildPortal(5, 16, 0, 0, dl, E, o, N, R, -5.0, 1.0, axisN, twist);
        const double c0[3] = {0.0, -R, 0.0};

        // 1. THE LINK TAKES THE ROOT'S CENTRE ONTO THE LEAF'S GLOBE -- closed form AND sandwich.
        double c1[3], c1v[3];
        pt.Apply(1.0, c0, c1);
        SandwichPoint(pt.S, c0, R, c1v);
        for (int i = 0; i < 3; ++i) {
            nearly(c1[i], pt.centre[i], 1e-6, "S(root centre) = the leaf globe's centre");
            nearly(c1v[i], pt.centre[i], 1e-4, "the versor agrees with the closed form (centre)");
        }
        // 2. ONE OBJECT: the root's DualSphere through the sandwich IS the leaf's globe. The
        // radius is read back out of the transformed sphere (after the priors-33 Normalize:
        // sigma.ni = -1 makes sigma^2 = r^2), not computed beside it.
        {
            using namespace cga;
            const Mv Sig0 = DualSphere(Up(c0[0] / R, c0[1] / R, c0[2] / R), 1.0);
            Mv Sig1 = Sandwich(pt.S, Sig0);
            Sig1 = Sig1 * (1.0 / -Dot(Sig1, Ni()));
            const double r2 = Dot(Sig1, Sig1);
            nearly(std::sqrt(r2 > 0 ? r2 : 0) * R, pt.radius, 1e-3,
                   "the transformed DualSphere's own radius = the leaf globe's radius");
        }
        // 3. THE FIXED POINT: the tower converges on p, and the versor leaves it alone.
        {
            double pp[3], pv[3];
            pt.Apply(1.0, pt.p, pp);
            SandwichPoint(pt.S, pt.p, R, pv);
            for (int i = 0; i < 3; ++i) {
                nearly(pp[i], pt.p[i], 1e-6, "S(p) = p (closed form)");
                nearly(pv[i], pt.p[i], 1e-4, "S(p) = p (sandwich)");
            }
        }
        // 4. THE REST CONTACT: the globe touches the leaf's ground exactly once, from above.
        {
            const double lo[3] = {pt.centre[0] - pt.groundUp[0] * pt.radius,
                                  pt.centre[1] - pt.groundUp[1] * pt.radius,
                                  pt.centre[2] - pt.groundUp[2] * pt.radius};
            const double gr = std::sqrt(lo[0] * lo[0] + (lo[1] + R) * (lo[1] + R) + lo[2] * lo[2]) - R;
            nearly(gr, -5.0, 1e-6, "the inner globe rests on the leaf's ground");
        }
        // 5. INVERSE, POWERS, AND THE ONE-PARAMETER SUBGROUP -- on scattered points.
        uint32_t st = 0x6d2b79f5u + variant;
        auto rnd = [&] {
            st = st * 1664525u + 1013904223u;
            return (double(st >> 8) / double(1u << 24)) * 2.0 - 1.0;
        };
        for (int n = 0; n < 24; ++n) {
            const double x[3] = {pt.p[0] + rnd() * 900.0, pt.p[1] + rnd() * 400.0,
                                 pt.p[2] + rnd() * 900.0};
            double a[3], b[3], c[3], v[3];
            pt.Apply(1.0, x, a);
            pt.Apply(-1.0, a, b);
            for (int i = 0; i < 3; ++i) nearly(b[i], x[i], 1e-7, "S^-1 S = 1");
            // the versor on a scattered point, and its inverse versor back
            SandwichPoint(pt.S, x, R, v);
            for (int i = 0; i < 3; ++i) nearly(v[i], a[i], 2e-4, "sandwich = closed form");
            SandwichPoint(pt.Sinv, v, R, c);
            for (int i = 0; i < 3; ++i) nearly(c[i], x[i], 2e-3, "Sinv sandwich undoes S");
            // S^2 = S S, and S^(1/2) S^(1/2) = S: the fractional powers are a subgroup, which
            // is what makes the logarithmic-spiral camera path exp(t log S) well defined.
            double h[3], hh[3], s2[3], ss[3];
            pt.Apply(0.5, x, h);
            pt.Apply(0.5, h, hh);
            for (int i = 0; i < 3; ++i) nearly(hh[i], a[i], 1e-7, "S^1/2 S^1/2 = S");
            pt.Apply(2.0, x, s2);
            pt.Apply(1.0, a, ss);
            for (int i = 0; i < 3; ++i) nearly(s2[i], ss[i], 1e-7, "S^2 = S S");
            // 6. THE GAUGE the renderer rests on: level k seen from C is the root seen from
            // S^-k(C), rotated by Q^k and scaled by s^k. Every inner/outer level is drawn by
            // this identity and nothing else.
            const double C[3] = {pt.p[0] + rnd() * 700.0, pt.p[1] + 10.0 + std::fabs(rnd()) * 300.0,
                                 pt.p[2] + rnd() * 700.0};
            for (int k = -1; k <= 2; ++k) {
                double Sk[3], Ck[3], Qk[3][3];
                pt.Apply(double(k), x, Sk);          // a point of level k, in the current frame
                pt.Apply(-double(k), C, Ck);         // the eye in level k's own frame
                pt.Rot(double(k), Qk);
                const double rel[3] = {x[0] - Ck[0], x[1] - Ck[1], x[2] - Ck[2]};
                double qr[3];
                MatVec(Qk, rel, qr);
                const double sk = pt.Scale(double(k));
                for (int i = 0; i < 3; ++i) {
                    const double lhs = Sk[i] - C[i], rhs = sk * qr[i];
                    nearly(lhs, rhs, 1e-6 * (std::max)(1.0, std::fabs(lhs)),
                           "S^k(y) - C = s^k Q^k (y - S^-k(C))");
                }
            }
        }
        // 7. THE FLOATING SCALE: re-rooting the eye one level in and drawing everything one
        // level shallower is the same picture -- the renormalization is a gauge change.
        {
            const double C[3] = {pt.p[0] - 3.0, pt.p[1] + 1.0, pt.p[2] + 0.5};   // near the tower
            double Cin[3];
            pt.Apply(-1.0, C, Cin);    // the eye in the inner level's frame
            double Q1[3][3];
            pt.Rot(1.0, Q1);
            for (int k = 1; k <= 2; ++k) {
                const double y[3] = {rnd() * 5e5, -3e6 + rnd() * 1e6, rnd() * 5e5};
                double a[3], b[3];
                pt.Apply(double(k), y, a);          // level k from the old frame
                pt.Apply(double(k - 1), y, b);      // level k-1 from the new frame
                const double relOld[3] = {a[0] - C[0], a[1] - C[1], a[2] - C[2]};
                const double relNew[3] = {b[0] - Cin[0], b[1] - Cin[1], b[2] - Cin[2]};
                double mapped[3];
                MatVec(Q1, relNew, mapped);
                for (int i = 0; i < 3; ++i) {
                    nearly(relOld[i], pt.s * mapped[i], 1e-6 * (std::max)(1.0, std::fabs(relOld[i])),
                           "renormalized: old view = s Q (new view)");
                }
            }
        }
        // 8. THE GROUND FIELD IS BLIND TO THE GAUGE. Everything the camera reads from the tower
        // (roll, speed, backdrop) must come out the same whichever level the eye is rooted in.
        // The eyes: one over the inner globe's own Merrimack at 40 m -- where the nearest-ground
        // rule steps TWO levels in (the next globe's top stands a hair nearer than the ground it
        // rests on) -- and a scatter around the tower. Each is read at the root, then re-rooted
        // exactly as main.cpp does it, read again, and mapped back: up by Q^k, lambda by s^k.
        {
            double upIn[3];
            const double upT[3] = {0.0, 1.0, 0.0};
            pt.ApplyDir(1.0, upT, upIn);   // the inner globe's own zenith at its Merrimack
            const double hover = pt.radius + 40.0 * pt.radius / 76.35;
            std::vector<std::array<double, 3>> eyes;
            eyes.push_back({pt.centre[0] + upIn[0] * hover, pt.centre[1] + upIn[1] * hover,
                            pt.centre[2] + upIn[2] * hover});
            for (int n = 0; n < 24; ++n) {
                eyes.push_back({pt.centre[0] + rnd() * 400.0, pt.centre[1] + std::fabs(rnd()) * 200.0,
                                pt.centre[2] + rnd() * 400.0});
            }
            for (size_t e = 0; e < eyes.size(); ++e) {
                const double* x0 = eyes[e].data();
                const GroundField f0 = Grounds(pt, 0, x0);
                // Any gauge, not only the one the rule would pick: the field must not care.
                for (int L = 1; L <= 2; ++L) {
                    double x[3];
                    pt.Apply(-double(L), x0, x);
                    const GroundField fL = Grounds(pt, L, x);
                    double upBack[3];
                    pt.ApplyDir(double(L), fL.up, upBack);   // level L's frame -> the root's
                    for (int i = 0; i < 3; ++i) {
                        nearly(upBack[i], f0.up[i], 1e-9, "ground up: gauge-blind");
                    }
                    nearly(fL.lambda * pt.Scale(double(L)), f0.lambda, 1e-9 * f0.lambda,
                           "ground scale: gauge-blind");
                    nearly(fL.W, f0.W, 1e-9, "space backdrop weight: gauge-blind");
                    // whose dome, whose space: the same ABSOLUTE level from every gauge
                    ++checks;
                    if (L + fL.domeRel != f0.domeRel || L + fL.spaceRel != f0.spaceRel) {
                        Log("[droste] FAIL backdrop owners: from level %d the dome is level %d "
                            "and space level %d; from the root, %d and %d",
                            L, L + fL.domeRel, L + fL.spaceRel, f0.domeRel, f0.spaceRel);
                        ok = false;
                    }
                }
            }
            // THE OWNING LEVEL over the inner Merrimack, as main.cpp settles it (4 steps a frame).
            {
                double x[3] = {eyes[0][0], eyes[0][1], eyes[0][2]};
                int L = 0;
                for (int guard = 0; guard < 4; ++guard) {
                    const int step = NearestLevel(pt, L, x);
                    if (step == 0) break;
                    double xn[3];
                    pt.Apply(-double(step), x, xn);
                    for (int i = 0; i < 3; ++i) x[i] = xn[i];
                    L += step;
                }
                // The heavier ground owns it: one step in, to the globe it hovers over, and no
                // further -- the next globe down is a speck at that distance.
                ++checks;
                if (L != 1) {
                    Log("[droste] FAIL owning level: the eye 40 m over the inner Merrimack (%s) "
                        "settles at level %d, expected 1 -- a globe smaller than its distance "
                        "took the eye",
                        variant ? "quarter twist" : "no twist", L);
                    ok = false;
                }
            }
        }
        if (variant == 1) {
            // The twist variant's orientation: with a quarter turn about north, the inner
            // globe's anchor (its Merrimack) faces WEST in its parent -- back toward the helm.
            double upIn[3];
            const double upT[3] = {0.0, 1.0, 0.0};
            pt.ApplyDir(1.0, upT, upIn);
            nearly(upIn[0], -1.0, 1e-12, "a quarter twist about north turns up into west");
        }
        Log("[droste] portal %s: s %.3e (radius %.2f m), fixed point (%.3f, %.3f, %.3f), "
            "%.2f m above the leaf's ground",
            variant ? "(quarter twist)" : "(no twist)", pt.s, pt.radius, pt.p[0], pt.p[1],
            pt.p[2], pt.p[1] - pt.centre[1] + pt.radius - 5.0);
    }
    if (ok) {
        Log("[droste] ---- PASS (%d checks): the root's centre lands on the leaf's globe, the "
            "DualSphere carries the radius through one sandwich, the fixed point is fixed, the "
            "inner globe rests on its leaf, S^-1 S = 1, the fractional powers are a subgroup, "
            "S^k(y) - C = s^k Q^k (y - S^-k(C)) (the gauge every level is drawn by), and "
            "re-rooting the eye is a gauge change ----", checks);
    }
    return ok;
}

}  // namespace ga::droste
