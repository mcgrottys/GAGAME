// ================================================================================================
//  Space.h - M12: THE FRAME CALCULUS AS CODE. A space is where a node lives; a placement is how
//  a space sits in its parent; resolution is ONE fold along the parent chain.
//
//  THE CONTRACT, EXACTLY (sharpened after an outside review, 2026-09-13): a Placement supports
//  RIGID MOTION, UNIFORM SCALE AND PARITY -- a similarity, proper or improper -- and nothing
//  else. Nonuniform scale, shear and the projection morphs are other contracts with other
//  types. Parity is the sign of s: the engine's planet frame is LEFT-handed (priors 34), so
//  ECEF -> planet is a reflection, and any improper orthogonal map is (-I) times a rotation --
//  the sign on s and the rotor of that rotation say it exactly (Frame() factors it so), the
//  conformal versor carries an odd factor (the Euclidean pseudoscalar), and directions flip
//  with it. A fractional power of a reflection has no principal branch and is refused. Its one
//  authoritative representation is the rotor + scale + translation below; the conformal versor
//  and the 4x4 are DERIVED execution forms. The invariants: the rotor is unit (Normalize()
//  re-unitizes a long product, as Motor::Normalize does) and "inverse is reverse" holds only
//  under it; s > 0; Pow(k) takes the PRINCIPAL branch (the rotor's angle in (-pi, pi]), so a
//  quarter twist per level means a quarter twist per level and never its complement.
//
//  WHICH ALGEBRA. A placement is a SIMILARITY -- the group the engine already needs and no more.
//  The tangent frame under the planet is a rigid motion (Pga.h's motor, exactly); the Droste
//  link is a dilation with a twist about a fixed point (Droste.h's Cl(4,1) versor, exactly); a
//  child whose metres are a scaled copy of its parent's (an inner planet) is the same object.
//  A similarity is x -> t + s R x: a scale, a rotation, a translation. Its rotation is stored
//  as a ROTOR -- the same four numbers Pga.h's Motor keeps as its real part, rotated with the
//  same QRotate and composed with the same QMul, so this file adds no second convention to
//  the engine and RunPgaSelfTest's orientation pins carry over unchanged. The conformal versor
//  of the same map, T(t/L) R D(s) at a declared unit length L, is built ON DEMAND (Versor) for
//  the sandwiches that need it (a sphere through a portal, a plane into a level's frame), and
//  spacetest holds the two forms against each other. No matrix exists in this file except at
//  its last function, ToMatrix, which is the rasterizer boundary and says so.
//
//  ONE LAW. Space::ToRoot() is a right fold over the parent chain with the identity placement
//  at the root. There is no branch for rigid vs similar: every link is a similarity and PGA is
//  the s = 1 representation. A space's UNIT LENGTH (priors 32) is not a placement at all -- it
//  is the length the conformal embedding is taken at, declared per space and checked by
//  Declare() against the space's extent (Cga.h: a point past |x|/L = 9.49e7 IS a point at
//  infinity). Metres are metres in every space; L only decides how well the versor is
//  conditioned. The Droste cycle (Space::Cycle) is a link from the root's tangent frame back
//  to the root whose placement is the portal's S; Level(k) is the k-th power of that ONE link,
//  and the gauge identity of Droste.h is then a statement about Level(k), unchanged.
//
//  THE POWER. Pow(k) is exp(k log S), the one-parameter subgroup. A similarity with s != 1 has
//  a fixed point p and its orbits are the logarithmic spirals of Droste.h; a rigid motion is
//  the s -> 1 limit, where p recedes to infinity along the screw axis and the orbit is the helix
//  Motor::Log/Exp already integrate. Both are the same subgroup; the code asks which form is
//  well posed (does (I - sR) invert?) and takes it. That is the algebra's own case split, not a
//  special case bolted on.
//
//  Gates: RunSpaceSelfTest (spacetest, --selftest): composition and inverse against the CGA
//  sandwich; the fold; Pow against Droste.h's Portal closed forms; PullPlane against the
//  hand-written transport in GlobeLayer AND the DualPlane sandwich; the unit-length refusal;
//  Frame() rows round-tripping; Versor(R) coefficient-equal to droste::SimilarityVersor.
// ================================================================================================
#pragma once

#include "core/Cga.h"
#include "core/Common.h"
#include "core/Pga.h"

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <string>

namespace ga {

// ------------------------------------------------------------------ Placement

struct Placement {
    // x_parent = t + s * R(x_own). r is the rotor in Pga.h's real-part layout (w, x, y, z).
    double s = 1.0;
    double r[4] = {1.0, 0.0, 0.0, 0.0};
    double t[3] = {0.0, 0.0, 0.0};

    static Placement Identity() { return Placement{}; }

    // A rigid frame given by its ROWS: east / up / north are the parent-frame directions of the
    // own x / y / z axes (GlobeLayer::SetFrame's planet -> tangent rows), `origin` the own
    // origin in parent coordinates. The declaration that owns the rows keeps them exactly (the
    // constant-buffer rows are casts of the declaration, never of this rotor); the rotor is
    // derived here by Shepperd's method, and Rows() gives them back to ~1e-16.
    static Placement Frame(const double east[3], const double up[3], const double north[3],
                           const double origin[3]) {
        Placement p;
        // R's columns are the own axes in parent coordinates: R[i][j] = axis_j[i]. A left-handed
        // triple (east . (up x north) < 0) is (-I) times a rotation: keep the sign in s and
        // derive the rotor from the negated columns, which ARE a rotation.
        const double det = east[0] * (up[1] * north[2] - up[2] * north[1]) -
                           east[1] * (up[0] * north[2] - up[2] * north[0]) +
                           east[2] * (up[0] * north[1] - up[1] * north[0]);
        const double sg = det < 0.0 ? -1.0 : 1.0;
        p.s = sg;
        const double R00 = sg * east[0], R01 = sg * up[0], R02 = sg * north[0];
        const double R10 = sg * east[1], R11 = sg * up[1], R12 = sg * north[1];
        const double R20 = sg * east[2], R21 = sg * up[2], R22 = sg * north[2];
        const double tr = R00 + R11 + R22;
        double w, x, y, z;
        if (tr > 0.0) {
            const double S = std::sqrt(tr + 1.0) * 2.0;
            w = 0.25 * S;
            x = (R21 - R12) / S;
            y = (R02 - R20) / S;
            z = (R10 - R01) / S;
        } else if (R00 > R11 && R00 > R22) {
            const double S = std::sqrt(1.0 + R00 - R11 - R22) * 2.0;
            w = (R21 - R12) / S;
            x = 0.25 * S;
            y = (R01 + R10) / S;
            z = (R02 + R20) / S;
        } else if (R11 > R22) {
            const double S = std::sqrt(1.0 + R11 - R00 - R22) * 2.0;
            w = (R02 - R20) / S;
            x = (R01 + R10) / S;
            y = 0.25 * S;
            z = (R12 + R21) / S;
        } else {
            const double S = std::sqrt(1.0 + R22 - R00 - R11) * 2.0;
            w = (R10 - R01) / S;
            x = (R02 + R20) / S;
            y = (R12 + R21) / S;
            z = 0.25 * S;
        }
        const double n = std::sqrt(w * w + x * x + y * y + z * z);
        p.r[0] = w / n;
        p.r[1] = x / n;
        p.r[2] = y / n;
        p.r[3] = z / n;
        p.t[0] = origin[0];
        p.t[1] = origin[1];
        p.t[2] = origin[2];
        return p;
    }

    // A rigid placement from a motor: the rotor is its real part, the translation is where it
    // sends the origin. Pga.h -> here, with nothing re-derived.
    static Placement Rigid(const Motor& m) {
        Placement p;
        m.Real(p.r);
        double x = 0.0, y = 0.0, z = 0.0;
        m.TransformPoint(x, y, z);
        p.t[0] = x;
        p.t[1] = y;
        p.t[2] = z;
        return p;
    }

    // The Droste link: a dilation by s about the fixed point p with a twist about `axis`
    // through it -- x -> p + s Q (x - p) = (p - s Q p) + s Q x. Same numbers as Droste.h's
    // Portal (Scale, Rot, Apply), same rotor as Motor::Rotation's real part.
    static Placement Similar(const double p[3], double s, const double axis[3], double twist) {
        Placement o;
        const double h = 0.5 * twist, sh = std::sin(h);
        o.r[0] = std::cos(h);
        o.r[1] = sh * axis[0];
        o.r[2] = sh * axis[1];
        o.r[3] = sh * axis[2];
        o.s = s;
        double qx = p[0], qy = p[1], qz = p[2];
        Motor::QRotate(o.r, qx, qy, qz);
        o.t[0] = p[0] - s * qx;
        o.t[1] = p[1] - s * qy;
        o.t[2] = p[2] - s * qz;
        return o;
    }

    // ---- the group ---------------------------------------------------------------------------
    // (this after inner): x -> t + s R (inner.t + inner.s inner.R x).
    Placement Then(const Placement& inner) const {
        Placement o;
        o.s = s * inner.s;
        Motor::QMul(r, inner.r, o.r);
        double ix = inner.t[0], iy = inner.t[1], iz = inner.t[2];
        Motor::QRotate(r, ix, iy, iz);
        o.t[0] = t[0] + s * ix;
        o.t[1] = t[1] + s * iy;
        o.t[2] = t[2] + s * iz;
        return o;
    }
    Placement Inverse() const {
        Placement o;
        o.s = 1.0 / s;
        o.r[0] = r[0];
        o.r[1] = -r[1];
        o.r[2] = -r[2];
        o.r[3] = -r[3];
        double x = t[0], y = t[1], z = t[2];
        Motor::QRotate(o.r, x, y, z);   // R^T t
        o.t[0] = -o.s * x;
        o.t[1] = -o.s * y;
        o.t[2] = -o.s * z;
        return o;
    }
    bool IsRigid(double eps = 1e-15) const { return std::fabs(s - 1.0) <= eps; }
    // RE-UNITIZE. Composing placements is exact in principle and drifts in practice (a rail or a
    // body composes one per tick), and "inverse is reverse" assumes |r| = 1. Same law as
    // Motor::Normalize, on the rotor alone: the scale and the translation carry no invariant.
    void Normalize() {
        const double n = std::sqrt(r[0] * r[0] + r[1] * r[1] + r[2] * r[2] + r[3] * r[3]);
        if (n > 0.0) {
            for (double& c : r) c /= n;
        }
    }

    // exp(k log S). See the banner: the fixed-point form when it is well posed, the screw
    // (Motor::Log / Exp) when the map is rigid. THE BRANCH: AxisAngle reads the rotor's angle
    // as 2 atan2(|v|, w), in (-pi, pi] when w >= 0 -- the principal log. A rotor with w < 0 is
    // the same rotation's other representative; Similar() and Frame() never produce one, and a
    // product that does is one sign flip away (the double cover Motor::Slerp already handles).
    Placement Pow(double k) const {
        if (s < 0.0) {
            // A reflection has no principal logarithm: only whole powers mean anything.
            const double ki = std::round(k);
            if (std::fabs(k - ki) > 1e-12) {
                Log("[space] Pow(%g) of an improper placement (s = %g) has no branch; refusing "
                    "(identity)", k, s);
                return Placement::Identity();
            }
            Placement acc = Placement::Identity();
            const Placement step = ki < 0 ? Inverse() : *this;
            for (int i = 0; i < static_cast<int>(std::fabs(ki)); ++i) acc = step.Then(acc);
            return acc;
        }
        if (IsRigid()) {
            const Motor m = ToMotor();
            double a[3], b[3];
            m.Log(a, b);
            for (int i = 0; i < 3; ++i) {
                a[i] *= k;
                b[i] *= k;
            }
            return Rigid(Motor::Exp(a, b));
        }
        double p[3];
        FixedPoint(p);
        Placement o;
        o.s = std::pow(s, k);
        RotorPow(r, k, o.r);
        double qx = p[0], qy = p[1], qz = p[2];
        Motor::QRotate(o.r, qx, qy, qz);
        o.t[0] = p[0] - o.s * qx;
        o.t[1] = p[1] - o.s * qy;
        o.t[2] = p[2] - o.s * qz;
        return o;
    }

    // ---- actions -----------------------------------------------------------------------------
    void Apply(const double x[3], double out[3]) const {
        double px = x[0], py = x[1], pz = x[2];
        Motor::QRotate(r, px, py, pz);
        out[0] = t[0] + s * px;
        out[1] = t[1] + s * py;
        out[2] = t[2] + s * pz;
    }
    // A direction feels the rotation and the PARITY (a reflection flips it), not the scale.
    void ApplyDir(const double d[3], double out[3]) const {
        double x = d[0], y = d[1], z = d[2];
        Motor::QRotate(r, x, y, z);
        const double sg = s < 0.0 ? -1.0 : 1.0;
        out[0] = sg * x;
        out[1] = sg * y;
        out[2] = sg * z;
    }
    // A parent-frame plane (n . x = d, n unit) pulled into the own frame: n' = R^T n,
    // d' = (d - n . t) / s. This IS GlobeLayer's hand-written frustum transport
    // (n' = Q^T n, d' = d / sigma, its planes being camera-relative so t = 0), and it is the
    // DualPlane sandwich through Versor() -- spacetest holds all three together.
    void PullPlane(const double n[3], double d, double outN[3], double& outD) const {
        const double rc[4] = {r[0], -r[1], -r[2], -r[3]};
        double x = n[0], y = n[1], z = n[2];
        Motor::QRotate(rc, x, y, z);
        outN[0] = x;
        outN[1] = y;
        outN[2] = z;
        outD = (d - (n[0] * t[0] + n[1] * t[1] + n[2] * t[2])) / s;
    }
    // The own axes in parent coordinates (the columns of R), for a declaration that wants its
    // rows back.
    void Rows(double east[3], double up[3], double north[3]) const {
        const double ex[3] = {1, 0, 0}, ey[3] = {0, 1, 0}, ez[3] = {0, 0, 1};
        ApplyDir(ex, east);
        ApplyDir(ey, up);
        ApplyDir(ez, north);
    }
    // The rigid part as a motor (s ignored): the rails, the cameras and the bodies speak Motor.
    Motor ToMotor() const {
        Motor rot;
        const double du0[4] = {0, 0, 0, 0};
        rot.SetParts(r, du0);
        return Motor::Translation(t[0], t[1], t[2]) * rot;
    }
    // The fixed point of a non-rigid similarity: (I - s R) p = t. Well posed whenever s != 1
    // (1 - s e^{i theta} never vanishes); Cramer on the 3x3, as Droste.h's BuildPortal does.
    void FixedPoint(double p[3]) const {
        double c0[3], c1[3], c2[3];
        Rows(c0, c1, c2);   // the columns of R
        double A[3][3];     // A = I - s R, A[i][j] = delta_ij - s * R[i][j] = delta_ij - s * c_j[i]
        for (int i = 0; i < 3; ++i) {
            A[i][0] = (i == 0 ? 1.0 : 0.0) - s * c0[i];
            A[i][1] = (i == 1 ? 1.0 : 0.0) - s * c1[i];
            A[i][2] = (i == 2 ? 1.0 : 0.0) - s * c2[i];
        }
        auto det3 = [](const double m[3][3]) {
            return m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1]) -
                   m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0]) +
                   m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]);
        };
        const double det = det3(A);
        for (int j = 0; j < 3; ++j) {
            double M[3][3];
            for (int i = 0; i < 3; ++i) {
                for (int k = 0; k < 3; ++k) M[i][k] = (k == j) ? t[i] : A[i][k];
            }
            p[j] = det3(M) / det;
        }
    }

    // The conformal versor of this map at unit length L: T(t/L) R D(s), rightmost acting first.
    // Equal to droste::SimilarityVersor(p, s, axis, twist, L) for the same map (T(p) R D T(-p)
    // = T(p - s R p) R D), which spacetest pins coefficient by coefficient.
    cga::Mv Versor(double L) const {
        double axis[3], angle;
        AxisAngle(r, axis, angle);
        const cga::Mv T = cga::Translator(t[0] / L, t[1] / L, t[2] / L);
        const cga::Mv R = cga::Rotor(axis[0], axis[1], axis[2], angle);
        const cga::Mv D = cga::Dilator(std::fabs(s));
        cga::Mv V = cga::Gp(cga::Gp(T, R), D);
        if (s < 0.0) {
            // The point reflection x -> -x is the sandwich with the Euclidean pseudoscalar
            // e1 e2 e3 -- an ODD versor, applied first (rightmost). spacetest pins it.
            V = cga::Gp(V, cga::Mv::Basis(0b111u));
        }
        return V;
    }

    // THE RASTERIZER BOUNDARY: the affine map as the row-major, row-vector 4x4 that
    // DirectXMath and Common.hlsli use (mul(float4(p, 1), M)). Derived here, never stored,
    // never composed -- compose placements, then ask for the matrix once.
    void ToMatrix(float m[16]) const {
        double c0[3], c1[3], c2[3];
        Rows(c0, c1, c2);
        const double* c[3] = {c0, c1, c2};   // row i of M = s * (column i of R)
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) m[i * 4 + j] = static_cast<float>(s * c[i][j]);
            m[i * 4 + 3] = 0.0f;
        }
        m[12] = static_cast<float>(t[0]);
        m[13] = static_cast<float>(t[1]);
        m[14] = static_cast<float>(t[2]);
        m[15] = 1.0f;
    }

    // ---- rotor helpers (Pga.h's layout) --------------------------------------------------------
    static void AxisAngle(const double q[4], double axis[3], double& angle) {
        const double vn = std::sqrt(q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
        angle = 2.0 * std::atan2(vn, q[0]);
        if (vn < 1e-300) {
            axis[0] = 0.0;
            axis[1] = 0.0;
            axis[2] = 1.0;
            return;
        }
        axis[0] = q[1] / vn;
        axis[1] = q[2] / vn;
        axis[2] = q[3] / vn;
    }
    static void RotorPow(const double q[4], double k, double out[4]) {
        double axis[3], angle;
        AxisAngle(q, axis, angle);
        const double h = 0.5 * angle * k, sh = std::sin(h);
        out[0] = std::cos(h);
        out[1] = sh * axis[0];
        out[2] = sh * axis[1];
        out[3] = sh * axis[2];
    }
};

// ------------------------------------------------------------------ Space

struct Space {
    std::string name;             // "planet.re", "tangent.act0816", "droste.tower", "solar.au"
    double unitM = 1.0;           // the conformal embedding's unit length (priors 32)
    double extentM = 0.0;         // the farthest point this space will ever name; 0 = unchecked
    const Space* parent = nullptr;
    Placement link;               // own -> parent

    // The chart onto the exchange frame (ATLAS tier 1) where this space has one: the
    // anchor-linear world.flat map of BathyModel.h (lat = orgLat + z / mPerLat, lon = orgLon +
    // x / mPerLon, mPerLon frozen at the anchor) -- what the kernels' geoA row is made of.
    struct Anchor {
        double latDeg = 0.0, lonDeg = 0.0, mPerLat = 0.0, mPerLon = 0.0;
        bool linear = false;
        void LatLonOf(double x, double z, double& outLatDeg, double& outLonDeg) const {
            outLatDeg = latDeg + z / mPerLat;
            outLonDeg = lonDeg + x / mPerLon;
        }
        void FlatOf(double inLatDeg, double inLonDeg, double& x, double& z) const {
            x = (inLonDeg - lonDeg) * mPerLon;
            z = (inLatDeg - latDeg) * mPerLat;
        }
    } anchor;

    // Cga.h: past |x| / L = 9.49e7 the embedding's origin term is annihilated and the point is
    // exactly a point at infinity. Measured there, gated in RunCgaSelfTest.
    static constexpr double kCollapseRatio = 9.49e7;

    // The unit-length rule, said once: the extent must sit inside the embedding's reach.
    bool Declare(std::string* why = nullptr) const {
        if (unitM <= 0.0) {
            if (why) *why = name + ": unit length must be positive";
            return false;
        }
        if (extentM > 0.0 && extentM / unitM >= kCollapseRatio) {
            if (why) {
                char b[256];
                snprintf(b, sizeof(b),
                         "%s: extent %.3g m at unit length %.3g m is %.3g units -- past the "
                         "conformal collapse at %.3g, a point there is a point at infinity",
                         name.c_str(), extentM, unitM, extentM / unitM, kCollapseRatio);
                *why = b;
            }
            return false;
        }
        return true;
    }

    // THE fold: the placement of this space in the root's frame, identity at the root.
    Placement ToRoot() const {
        return parent ? parent->ToRoot().Then(link) : Placement::Identity();
    }
    // This space's placement in `other`'s frame, evaluated through the NEAREST COMMON ANCESTOR:
    // the fold from the ancestor down each branch, `other`'s inverted -- never root-relative-
    // then-cancel. Two boats 40 m apart in one tangent space must never see the planet's
    // radius in their arithmetic; extentM / unitM guards the embedding, this guards the
    // doubles. Two spaces with no common ancestor are two worlds: refused (identity, logged).
    Placement To(const Space& other) const {
        const Space* lca = nullptr;
        for (const Space* p = this; p && !lca; p = p->parent) {
            for (const Space* q = &other; q; q = q->parent) {
                if (q == p) {
                    lca = p;
                    break;
                }
            }
        }
        if (!lca) {
            Log("[space] %s -> %s: no common ancestor -- two worlds; refusing (identity)",
                name.c_str(), other.name.c_str());
            return Placement::Identity();
        }
        auto up = [lca](const Space* from) {   // the fold from `from` up to (not including) lca
            Placement acc = Placement::Identity();
            for (const Space* p = from; p != lca; p = p->parent) acc = p->link.Then(acc);
            return acc;
        };
        return up(&other).Inverse().Then(up(this));
    }
    // A point of this space (metres) as a conformal point at this space's unit length.
    cga::Mv Embed(const double xM[3]) const {
        return cga::Up(xM[0] / unitM, xM[1] / unitM, xM[2] / unitM);
    }
    // THE CYCLE: the root hung under a frame of itself. `within` is the space S is expressed in
    // (the root's tangent frame for the Droste portal); Level(k) is S^k, and the gauge identity
    // of Droste.h reads: drawing level k from the eye C is drawing level 0 from
    // Level(k).Inverse().Apply(C).
    static Space Cycle(const char* name, const Space& within, const Placement& S) {
        Space c;
        c.name = name;
        c.unitM = within.unitM;
        c.extentM = within.extentM;
        c.parent = &within;
        c.link = S;
        return c;
    }
    Placement Level(double k) const { return link.Pow(k); }
};

// The gate (src/core/SpaceTest.cpp), run from --selftest after gatest.
bool RunSpaceSelfTest();

}  // namespace ga
