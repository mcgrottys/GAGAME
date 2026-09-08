// ================================================================================================
//  Pga.h - 3D plane-based geometric algebra Cl(3,0,1): MOTORS, the first CPU-side GA in the
//  engine (GA.hlsli carries the GPU side; M7's vessels will build on this file).
//
//  A motor is an even-grade element
//      M = s + r23 e23 + r31 e31 + r12 e12  +  t01 e01 + t02 e02 + t03 e03 + q e0123
//  and every rigid motion is one sandwich X' = M X ~M. The payoff over matrices for a camera:
//  the Google-Earth gestures -- tilt and rotate about the point you grabbed on the ground --
//  are each ONE rotation about an arbitrary world LINE, which is a single motor (no
//  translate-rotate-translate bookkeeping), and motors compose by the geometric product and
//  interpolate cleanly when we get to animating them.
//
//  Coordinates: the even subalgebra of Cl(3,0,1) is isomorphic to the dual quaternions; the
//  fields below are exactly the dual-quaternion coordinates of the motor under that
//  isomorphism (real part (s, r23, r31, r12), dual part (q, t01, t02, t03)), and the products
//  are the motor geometric product written through it. RunPgaSelfTest pins the orientation
//  conventions numerically: right-handed rotations in the engine frame (x east, y up, z north).
// ================================================================================================
#pragma once

#include "core/Common.h"

#include <cmath>

namespace ga {

struct Motor {
    double s = 1, r23 = 0, r31 = 0, r12 = 0;   // Euclidean rotor part
    double t01 = 0, t02 = 0, t03 = 0, q = 0;   // ideal (translation-carrying) part

    // ---- quaternion arithmetic on the two halves (the isomorphism made executable) ----------
    static void QMul(const double a[4], const double b[4], double out[4]) {
        out[0] = a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3];
        out[1] = a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2];
        out[2] = a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1];
        out[3] = a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0];
    }
    static void QRotate(const double r[4], double& x, double& y, double& z) {
        const double v[4] = {0, x, y, z};
        const double rc[4] = {r[0], -r[1], -r[2], -r[3]};
        double t[4], o[4];
        QMul(r, v, t);
        QMul(t, rc, o);
        x = o[1];
        y = o[2];
        z = o[3];
    }

    void Real(double out[4]) const { out[0] = s; out[1] = r23; out[2] = r31; out[3] = r12; }
    void Dual(double out[4]) const { out[0] = q; out[1] = t01; out[2] = t02; out[3] = t03; }
    void SetParts(const double re[4], const double du[4]) {
        s = re[0]; r23 = re[1]; r31 = re[2]; r12 = re[3];
        q = du[0]; t01 = du[1]; t02 = du[2]; t03 = du[3];
    }

    static Motor Identity() { return Motor{}; }

    // Pure translation by (tx, ty, tz).
    static Motor Translation(double tx, double ty, double tz) {
        Motor m;
        const double re[4] = {1, 0, 0, 0};
        const double du[4] = {0, 0.5 * tx, 0.5 * ty, 0.5 * tz};
        m.SetParts(re, du);
        return m;
    }

    // Rotation by `angle` (right-handed) about the LINE through point p with unit direction d --
    // the whole point of using PGA: an offset axis is one primitive, not a composition.
    static Motor Rotation(const double p[3], const double d[3], double angle) {
        const double h = 0.5 * angle;
        const double sh = std::sin(h);
        const double re[4] = {std::cos(h), sh * d[0], sh * d[1], sh * d[2]};
        // Ideal part: the axis moment. Built as T(p) R T(-p) collapsed analytically:
        // t = p - R(p); dual = 0.5 * (0, t) * re.
        double rx = p[0], ry = p[1], rz = p[2];
        QRotate(re, rx, ry, rz);
        const double tv[4] = {0, 0.5 * (p[0] - rx), 0.5 * (p[1] - ry), 0.5 * (p[2] - rz)};
        double du[4];
        QMul(tv, re, du);
        Motor m;
        m.SetParts(re, du);
        return m;
    }

    // Geometric product: (*this) after `o` (matches matrix convention: (A*B)(X) = A(B(X))).
    Motor operator*(const Motor& o) const {
        double ar[4], ad[4], br[4], bd[4];
        Real(ar); Dual(ad);
        o.Real(br); o.Dual(bd);
        double re[4], d1[4], d2[4], du[4];
        QMul(ar, br, re);
        QMul(ar, bd, d1);
        QMul(ad, br, d2);
        for (int i = 0; i < 4; ++i) du[i] = d1[i] + d2[i];
        Motor m;
        m.SetParts(re, du);
        return m;
    }

    // Sandwich on a point: X' = M X ~M.
    void TransformPoint(double& x, double& y, double& z) const {
        double re[4], du[4];
        Real(re); Dual(du);
        QRotate(re, x, y, z);
        const double rc[4] = {re[0], -re[1], -re[2], -re[3]};
        double t[4];
        QMul(du, rc, t);   // translation = 2 * Vec(dual * ~real)
        x += 2.0 * t[1];
        y += 2.0 * t[2];
        z += 2.0 * t[3];
    }

    // Directions feel only the rotor part (ideal translations act trivially on the horizon).
    void TransformDir(double& x, double& y, double& z) const {
        double re[4];
        Real(re);
        QRotate(re, x, y, z);
    }

    // Reverse ~M; for a unit motor this is the inverse.
    Motor Inverse() const {
        Motor m = *this;
        m.r23 = -r23; m.r31 = -r31; m.r12 = -r12;
        m.t01 = -t01; m.t02 = -t02; m.t03 = -t03;
        return m;
    }

    // M9bq: RE-UNITIZE. A motor is unit iff BOTH conditions hold -- |real| = 1 (it is a
    // rotation) and real . dual = 0 (the Study condition; a dual quaternion violating it is
    // not a rigid motion at all). Composing motors is exact in principle and drifts in
    // practice, and a rigid-body integrator composes one per tick forever -- ~10^7 over an
    // hour at 240 Hz -- so the drift is not hypothetical. Nothing needed this before the
    // integrator: every previous motor in the engine was built fresh from Rotation() or
    // Translation() each frame.
    //
    // Both violations are removed in the order they must be: scale to unit rotation first,
    // then project the Study component OUT of the dual part rather than scaling it, because
    // the violation is a direction in dual space and not a magnitude error.
    void Normalize() {
        double re[4], du[4];
        Real(re); Dual(du);
        const double n = std::sqrt(re[0]*re[0] + re[1]*re[1] + re[2]*re[2] + re[3]*re[3]);
        if (n < 1e-12) { *this = Identity(); return; }   // degenerate: no rotation to recover
        const double inv = 1.0 / n;
        for (int i = 0; i < 4; ++i) { re[i] *= inv; du[i] *= inv; }
        const double dot = re[0]*du[0] + re[1]*du[1] + re[2]*du[2] + re[3]*du[3];
        for (int i = 0; i < 4; ++i) du[i] -= dot * re[i];
        SetParts(re, du);
    }

    // ---- the screw calculus: log / exp on the motor manifold --------------------------------
    // A motor IS a screw motion: rotation theta about a line plus translation d along it.
    // Log unpacks the screw into a bivector (a = (theta/2) l, b = (theta/2) m + (d/2) l with m
    // the axis moment); Exp packs it back; scaling the bivector scales the MOTION. That is what
    // puts a camera on rails: Slerp(M0, M1, u) = M0 Exp(u Log(~M0 M1)) is the constant-velocity
    // screw between two poses -- no per-axis lerp artifacts, position and aim carried together.
    void Log(double a[3], double b[3]) const {
        const double s = std::sqrt(r23 * r23 + r31 * r31 + r12 * r12);
        if (s < 1e-12) {            // pure translation: t = 2 * dual vector
            a[0] = a[1] = a[2] = 0.0;
            b[0] = t01; b[1] = t02; b[2] = t03;
            return;
        }
        const double half = std::atan2(s, this->s);          // theta/2
        const double l[3] = {r23 / s, r31 / s, r12 / s};
        const double dHalf = -q / s;                          // d/2  (from q = -(d/2) sin)
        const double m[3] = {(t01 - dHalf * this->s * l[0]) / s,
                             (t02 - dHalf * this->s * l[1]) / s,
                             (t03 - dHalf * this->s * l[2]) / s};
        for (int i = 0; i < 3; ++i) {
            a[i] = half * l[i];
            b[i] = half * m[i] + dHalf * l[i];
        }
    }

    static Motor Exp(const double a[3], const double b[3]) {
        Motor out;
        const double half = std::sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]);
        if (half < 1e-12) {         // pure translation
            const double re[4] = {1, 0, 0, 0};
            const double du[4] = {0, b[0], b[1], b[2]};
            out.SetParts(re, du);
            return out;
        }
        const double l[3] = {a[0] / half, a[1] / half, a[2] / half};
        const double dHalf = b[0] * l[0] + b[1] * l[1] + b[2] * l[2];
        const double m[3] = {(b[0] - dHalf * l[0]) / half, (b[1] - dHalf * l[1]) / half,
                             (b[2] - dHalf * l[2]) / half};
        const double c = std::cos(half), sn = std::sin(half);
        const double re[4] = {c, sn * l[0], sn * l[1], sn * l[2]};
        const double du[4] = {-dHalf * sn, dHalf * c * l[0] + sn * m[0],
                              dHalf * c * l[1] + sn * m[1], dHalf * c * l[2] + sn * m[2]};
        out.SetParts(re, du);
        return out;
    }

    // Screw interpolation between two poses (shortest arc). u in [0,1].
    static Motor Slerp(const Motor& m0, Motor m1, double u) {
        // Double cover: pick the representative of m1 nearest m0.
        if (m0.s * m1.s + m0.r23 * m1.r23 + m0.r31 * m1.r31 + m0.r12 * m1.r12 < 0.0) {
            m1.s = -m1.s; m1.r23 = -m1.r23; m1.r31 = -m1.r31; m1.r12 = -m1.r12;
            m1.q = -m1.q; m1.t01 = -m1.t01; m1.t02 = -m1.t02; m1.t03 = -m1.t03;
        }
        double a[3], b[3];
        (m0.Inverse() * m1).Log(a, b);
        for (int i = 0; i < 3; ++i) {
            a[i] *= u;
            b[i] *= u;
        }
        return m0 * Exp(a, b);
    }
};

// ================================================================================================
//  Bivector - M9bq: THE SCREW. Six doubles, and the only kinematic/dynamic type the vessels need.
//
//  A grade-2 element of Cl(3,0,1) is six-dimensional -- three Euclidean (e23, e31, e12) and three
//  ideal (e01, e02, e03) -- and it is the SAME object for a velocity and for a force. That is not
//  a coincidence to be admired; it is the reason this type exists, because it removes the two
//  bookkeeping errors a boat physics engine otherwise makes forever:
//
//    * there is no separate force and torque to keep in step -- a wrench IS one bivector, so a
//      thrust applied at a transom mount cannot lose its moment arm on the way to the solver;
//    * there is no separate linear and angular velocity -- a twist IS one bivector, so a hull
//      point's velocity is one expression and not a sum somebody has to remember to write.
//
//  THE CONVENTION, derived rather than adopted (the derivation is a gate below, not a comment).
//  Conjugating a bivector by a pure translation t through the dual-quaternion product gives
//
//      a' = a,        b' = b + t x a
//
//  and BOTH physical readings satisfy exactly that, which is why one sandwich serves both:
//
//      TWIST    a = angular velocity omega        b = linear velocity AT THE WORLD ORIGIN
//               (translating a body leaves omega alone and slides v by t x omega)
//      WRENCH   a = force f                       b = torque about the WORLD ORIGIN
//               (translating a line of action leaves f alone and adds t x f to the torque)
//
//  So the Euclidean part is always the free vector and the ideal part is always its moment. The
//  power delivered by a wrench W on a twist T is then the symmetric pairing
//  T.a . W.b + T.b . W.a = omega . tau + v . f, which is invariant under transport -- gated.
//
//  Motor::Log already returns its screw in these same two slots, so Log/Exp need no adapter.
// ================================================================================================
struct Bivector {
    double a[3] = {0, 0, 0};   // Euclidean part: e23, e31, e12 -- omega (twist) or f (wrench)
    double b[3] = {0, 0, 0};   // ideal part:     e01, e02, e03 -- v_O   (twist) or tau_O (wrench)

    static Bivector Zero() { return Bivector{}; }

    // A twist: angular velocity, and the velocity of the body point at the world origin.
    static Bivector Twist(const double omega[3], const double vAtOrigin[3]) {
        Bivector t;
        for (int i = 0; i < 3; ++i) { t.a[i] = omega[i]; t.b[i] = vAtOrigin[i]; }
        return t;
    }
    // A wrench: a force, and where it acts. The moment arm is taken HERE, once, which is the
    // whole point -- no caller ever computes a torque.
    static Bivector ForceAt(const double f[3], const double p[3]) {
        Bivector w;
        for (int i = 0; i < 3; ++i) w.a[i] = f[i];
        w.b[0] = p[1]*f[2] - p[2]*f[1];      // tau_O = p x f
        w.b[1] = p[2]*f[0] - p[0]*f[2];
        w.b[2] = p[0]*f[1] - p[1]*f[0];
        return w;
    }
    // A pure couple: a torque with no net force. Free vectors have no line of action, so this
    // is the one case where the Euclidean part is empty.
    static Bivector Couple(const double tau[3]) {
        Bivector w;
        for (int i = 0; i < 3; ++i) w.b[i] = tau[i];
        return w;
    }

    Bivector operator+(const Bivector& o) const {
        Bivector r;
        for (int i = 0; i < 3; ++i) { r.a[i] = a[i] + o.a[i]; r.b[i] = b[i] + o.b[i]; }
        return r;
    }
    Bivector operator-(const Bivector& o) const {
        Bivector r;
        for (int i = 0; i < 3; ++i) { r.a[i] = a[i] - o.a[i]; r.b[i] = b[i] - o.b[i]; }
        return r;
    }
    Bivector operator*(double k) const {
        Bivector r;
        for (int i = 0; i < 3; ++i) { r.a[i] = a[i] * k; r.b[i] = b[i] * k; }
        return r;
    }
    Bivector& operator+=(const Bivector& o) { *this = *this + o; return *this; }

    // The velocity of the body point currently at world position p -- v(p) = v_O + omega x p.
    // Reading a twist at a point is one call, which is what the hull panel sampler wants.
    void VelocityAt(const double p[3], double out[3]) const {
        out[0] = b[0] + a[1]*p[2] - a[2]*p[1];
        out[1] = b[1] + a[2]*p[0] - a[0]*p[2];
        out[2] = b[2] + a[0]*p[1] - a[1]*p[0];
    }

    // Power of a wrench (this) acting on a twist: omega . tau + v . f. Frame-invariant.
    double Power(const Bivector& twist) const {
        double s = 0.0;
        for (int i = 0; i < 3; ++i) s += twist.a[i]*b[i] + twist.b[i]*a[i];
        return s;
    }

    // THE SANDWICH: B' = M B ~M. A bivector is a dual quaternion with both scalar slots empty,
    // so the motor product already written above IS this operation -- no second algebra.
    Bivector Transform(const Motor& m) const {
        Motor e;                              // embed (scalar parts stay 0)
        e.s = 0; e.r23 = a[0]; e.r31 = a[1]; e.r12 = a[2];
        e.q = 0; e.t01 = b[0]; e.t02 = b[1]; e.t03 = b[2];
        const Motor r = m * e * m.Inverse();
        Bivector o;
        o.a[0] = r.r23; o.a[1] = r.r31; o.a[2] = r.r12;
        o.b[0] = r.t01; o.b[1] = r.t02; o.b[2] = r.t03;
        return o;                             // r.s and r.q come back 0 -- gated, not assumed
    }

    // The algebraic exponential and its inverse. These pair EXACTLY with Motor::Log/Exp, which
    // means the half-angle convention lives in one place: `a` carries theta/2, not theta. Do not
    // hand a raw angular velocity to Exp() -- use IntegrateStep below, which is named for it.
    Motor Exp() const { return Motor::Exp(a, b); }
    static Bivector Log(const Motor& m) {
        Bivector v;
        m.Log(v.a, v.b);
        return v;
    }

    double NormSq() const {                   // the Euclidean (rotational) magnitude squared
        return a[0]*a[0] + a[1]*a[1] + a[2]*a[2];
    }
};

// The integrator's step: the motor that advances a pose by TWIST for dt seconds. The 0.5 is the
// half-angle -- Motor::Exp reads `a` as theta/2 -- and it is written once, here, so no caller can
// drop it. Pose update is then M <- IntegrateStep(B, dt) * M for a twist in WORLD coordinates.
inline Motor IntegrateStep(const Bivector& twist, double dt) {
    return (twist * (0.5 * dt)).Exp();
}

// Numeric gate for the conventions above. Pure CPU; runs inside --selftest.
inline bool RunPgaSelfTest() {
    auto near3 = [](double x, double y, double z, double ex, double ey, double ez) {
        return std::abs(x - ex) < 1e-9 && std::abs(y - ey) < 1e-9 && std::abs(z - ez) < 1e-9;
    };
    bool ok = true;
    const double org[3] = {0, 0, 0}, up[3] = {0, 1, 0};

    // Right-handed quarter turn about +y through the origin: east goes to south (+x -> -z).
    {
        double x = 1, y = 0, z = 0;
        Motor::Rotation(org, up, 1.5707963267948966).TransformPoint(x, y, z);
        ok &= near3(x, y, z, 0, 0, -1);
    }
    // Offset axis -- the PGA point: half turn about the vertical line through (5,0,0).
    {
        const double p[3] = {5, 0, 0};
        const Motor m = Motor::Rotation(p, up, 3.141592653589793);
        double x = 6, y = 0, z = 0;
        m.TransformPoint(x, y, z);
        ok &= near3(x, y, z, 4, 0, 0);
        double ax = 5, ay = 0, az = 0;
        m.TransformPoint(ax, ay, az);
        ok &= near3(ax, ay, az, 5, 0, 0);         // the axis is pointwise fixed
        double dx = 1, dy = 0, dz = 0;
        m.TransformDir(dx, dy, dz);
        ok &= near3(dx, dy, dz, -1, 0, 0);        // directions ignore the offset
    }
    // Composition = sequential application, and the offset rotation IS T(p) R T(-p).
    {
        const double p[3] = {5, 0, 0};
        const Motor direct = Motor::Rotation(p, up, 0.7);
        const Motor composed = Motor::Translation(5, 0, 0) *
                               Motor::Rotation(org, up, 0.7) * Motor::Translation(-5, 0, 0);
        double ax = 2, ay = 3, az = -4, bx = 2, by = 3, bz = -4;
        direct.TransformPoint(ax, ay, az);
        composed.TransformPoint(bx, by, bz);
        ok &= near3(ax, ay, az, bx, by, bz);
    }
    // Rigidity: a screwy motor preserves distances.
    {
        const double p[3] = {3, -2, 7}, d[3] = {0.6, 0.48, 0.64};
        const Motor m = Motor::Translation(1, 2, 3) * Motor::Rotation(p, d, 1.1);
        double ax = 1, ay = 2, az = 3, bx = -4, by = 0, bz = 5;
        const double before = std::sqrt(25 + 4 + 4);
        m.TransformPoint(ax, ay, az);
        m.TransformPoint(bx, by, bz);
        const double after =
            std::sqrt((ax - bx) * (ax - bx) + (ay - by) * (ay - by) + (az - bz) * (az - bz));
        ok &= std::abs(before - after) < 1e-9;
    }
    // Screw calculus: Log/Exp round-trip, Slerp endpoints, and the screw property -- the
    // midpoint of a pure rotation about an offset axis keeps the axis pointwise fixed.
    {
        const double p[3] = {5, 1, -2}, d[3] = {0, 1, 0};
        const Motor m = Motor::Translation(0.5, -0.25, 2.0) * Motor::Rotation(p, d, 0.9);
        double a[3], b[3];
        m.Log(a, b);
        const Motor rt = Motor::Exp(a, b);
        ok &= std::abs(rt.s - m.s) < 1e-9 && std::abs(rt.r12 - m.r12) < 1e-9 &&
              std::abs(rt.t01 - m.t01) < 1e-9 && std::abs(rt.q - m.q) < 1e-9;

        const Motor m0 = Motor::Rotation(p, d, 0.3);
        const Motor m1 = Motor::Rotation(p, d, 1.7);
        const Motor half = Motor::Slerp(m0, m1, 0.5);
        double x = 5, y = 1, z = -2;                       // a point ON the axis
        half.TransformPoint(x, y, z);
        ok &= near3(x, y, z, 5, 1, -2);
        double e1x = 6, e1y = 1, e1z = -2, e2x = 6, e2y = 1, e2z = -2;
        Motor::Slerp(m0, m1, 1.0).TransformPoint(e1x, e1y, e1z);
        m1.TransformPoint(e2x, e2y, e2z);
        ok &= near3(e1x, e1y, e1z, e2x, e2y, e2z);         // Slerp(...,1) == endpoint
    }
    // ---- M9bq: NORMALIZE. Perturb a motor off the unit sphere AND off the Study quadric,
    // renormalize, and demand both conditions back -- plus rigidity, which is the property
    // the integrator actually cares about losing.
    {
        const double p[3] = {2, -1, 4}, d[3] = {0, 0.6, 0.8};
        Motor m = Motor::Translation(3, 1, -2) * Motor::Rotation(p, d, 0.8);
        m.s *= 1.004; m.r12 -= 0.003; m.t02 += 0.002; m.q += 0.0015;   // drift, both kinds
        m.Normalize();
        double re[4], du[4];
        m.Real(re); m.Dual(du);
        const double n2 = re[0]*re[0] + re[1]*re[1] + re[2]*re[2] + re[3]*re[3];
        const double study = re[0]*du[0] + re[1]*du[1] + re[2]*du[2] + re[3]*du[3];
        ok &= std::abs(n2 - 1.0) < 1e-12;      // |real| = 1
        ok &= std::abs(study) < 1e-12;         // real . dual = 0
        double ax = 1, ay = 2, az = 3, bx = -4, by = 0, bz = 5;
        m.TransformPoint(ax, ay, az);
        m.TransformPoint(bx, by, bz);
        const double after =
            std::sqrt((ax-bx)*(ax-bx) + (ay-by)*(ay-by) + (az-bz)*(az-bz));
        ok &= std::abs(after - std::sqrt(25.0 + 4.0 + 4.0)) < 1e-9;   // rigidity restored
    }

    // ---- M9bq: THE BIVECTOR SANDWICH IS PURE. M B ~M must land back in grade 2: if the
    // scalar or pseudoscalar slot came back non-zero the embedding would be wrong and every
    // transported wrench would carry silent garbage.
    {
        const double f[3] = {1.5, -2.0, 0.5}, at[3] = {0.3, 1.1, -0.7};
        const Bivector w = Bivector::ForceAt(f, at);
        Motor e;
        e.s = 0; e.r23 = w.a[0]; e.r31 = w.a[1]; e.r12 = w.a[2];
        e.q = 0; e.t01 = w.b[0]; e.t02 = w.b[1]; e.t03 = w.b[2];
        const Motor m = Motor::Translation(2, -3, 1) * Motor::Rotation(org, up, 0.7);
        const Motor r = m * e * m.Inverse();
        ok &= std::abs(r.s) < 1e-12 && std::abs(r.q) < 1e-12;
    }

    // ---- M9bq: WRENCH TRANSPORT. THE gate for the convention. Transporting the wrench of a
    // force must equal the wrench of the transported force at the transported point -- for a
    // general screw motion, not just a translation. If the Euclidean/ideal slots were swapped
    // this fails on the moment arm, which is the error that would otherwise show up as a boat
    // that yaws when it should surge.
    {
        const double f[3] = {12.0, -4.0, 7.5}, at[3] = {1.4, -0.8, 2.2};
        const double p[3] = {0.5, 2.0, -1.0}, d[3] = {0.36, 0.48, 0.8};
        const Motor m = Motor::Translation(-2, 5, 3) * Motor::Rotation(p, d, 1.3);

        const Bivector moved = Bivector::ForceAt(f, at).Transform(m);

        double fx = f[0], fy = f[1], fz = f[2];
        m.TransformDir(fx, fy, fz);                       // the force is a direction
        double px = at[0], py = at[1], pz = at[2];
        m.TransformPoint(px, py, pz);                     // its point of application is a point
        const double f2[3] = {fx, fy, fz}, p2[3] = {px, py, pz};
        const Bivector direct = Bivector::ForceAt(f2, p2);

        for (int i = 0; i < 3; ++i) {
            ok &= std::abs(moved.a[i] - direct.a[i]) < 1e-9;
            ok &= std::abs(moved.b[i] - direct.b[i]) < 1e-9;
        }
    }

    // ---- M9bq: TWIST TRANSPORT, the same statement for velocities. The velocity the
    // transported twist reports at the transported point must be the transported velocity.
    {
        const double om[3] = {0.2, -0.5, 0.9}, vo[3] = {1.0, 0.25, -3.0};
        const double q[3] = {2.5, -1.5, 0.75};
        const double p[3] = {-1, 0.5, 2}, d[3] = {0.6, -0.64, 0.48};
        const Motor m = Motor::Translation(4, -1, 2) * Motor::Rotation(p, d, 0.95);

        const Bivector t = Bivector::Twist(om, vo);
        double v0[3];
        t.VelocityAt(q, v0);
        m.TransformDir(v0[0], v0[1], v0[2]);              // velocity is a direction

        double qx = q[0], qy = q[1], qz = q[2];
        m.TransformPoint(qx, qy, qz);
        const double q2[3] = {qx, qy, qz};
        double v1[3];
        t.Transform(m).VelocityAt(q2, v1);
        ok &= near3(v1[0], v1[1], v1[2], v0[0], v0[1], v0[2]);
    }

    // ---- M9bq: POWER IS FRAME-INVARIANT. omega . tau + v . f under simultaneous transport --
    // one number, and it is what says the twist and wrench conventions are DUAL to each other
    // rather than merely each self-consistent.
    {
        const double om[3] = {0.3, 1.1, -0.4}, vo[3] = {2.0, -1.0, 0.5};
        const double f[3] = {5.0, 2.5, -1.5}, at[3] = {0.9, -2.2, 1.7};
        const double p[3] = {1, 1, 1}, d[3] = {0.8, 0.0, 0.6};
        const Motor m = Motor::Translation(-3, 2, 6) * Motor::Rotation(p, d, 2.1);
        const Bivector t = Bivector::Twist(om, vo), w = Bivector::ForceAt(f, at);
        ok &= std::abs(w.Power(t) - w.Transform(m).Power(t.Transform(m))) < 1e-9;
    }

    // ---- M9bq: CHASLES. Every rigid motion is a rotation about ONE line plus a translation
    // ALONG it. Read the screw axis out of Log and check both halves: the motor maps the axis
    // line onto itself, and the displacement of an axis point is pure pitch (parallel to the
    // axis, zero perpendicular component).
    {
        const double p[3] = {3, -1, 2}, d[3] = {0.48, 0.6, 0.64};
        const Motor m = Motor::Translation(0.48*2.5, 0.6*2.5, 0.64*2.5) *
                        Motor::Rotation(p, d, 1.15);        // 1.15 rad about d, 2.5 m along it
        Bivector sc = Bivector::Log(m);
        const double half = std::sqrt(sc.NormSq());
        ok &= half > 1e-9;
        const double l[3] = {sc.a[0]/half, sc.a[1]/half, sc.a[2]/half};
        ok &= near3(l[0], l[1], l[2], d[0], d[1], d[2]);    // the axis direction is recovered

        double ax = p[0], ay = p[1], az = p[2];             // a point ON the axis
        m.TransformPoint(ax, ay, az);
        const double dsp[3] = {ax - p[0], ay - p[1], az - p[2]};
        const double along = dsp[0]*l[0] + dsp[1]*l[1] + dsp[2]*l[2];
        ok &= std::abs(along - 2.5) < 1e-9;                 // pitch is the declared 2.5 m
        for (int i = 0; i < 3; ++i) {
            ok &= std::abs(dsp[i] - along * l[i]) < 1e-9;   // and nothing perpendicular
        }
    }

    // ---- M9bq: THE INTEGRATOR HOLDS. A free body spinning at constant twist, stepped 100000
    // times at 240 Hz with a renormalize each tick (exactly what RigidBody will do), must keep
    // |omega| -- the twist is constant, so any drift here is the motor chain's, not physics --
    // and must stay rigid. This is the gate that says a boat can be sailed for an hour.
    {
        const double om[3] = {0.0, 0.9, 0.0}, vo[3] = {0.0, 0.0, 0.0};
        const Bivector t = Bivector::Twist(om, vo);
        Motor m = Motor::Rotation(org, up, 0.0);
        const double dt = 1.0 / 240.0;
        for (int i = 0; i < 100000; ++i) {
            m = IntegrateStep(t, dt) * m;
            m.Normalize();
        }
        double re[4], du[4];
        m.Real(re); m.Dual(du);
        const double n2 = re[0]*re[0] + re[1]*re[1] + re[2]*re[2] + re[3]*re[3];
        const double study = re[0]*du[0] + re[1]*du[1] + re[2]*du[2] + re[3]*du[3];
        ok &= std::abs(n2 - 1.0) < 1e-12 && std::abs(study) < 1e-12;
        // The twist read back off the accumulated motor still has the SAME magnitude: 100000
        // steps of 0.9 rad/s at 1/240 s is 375 rad, and the axis must not have wandered.
        double x = 1, y = 0, z = 0;
        m.TransformPoint(x, y, z);
        ok &= std::abs(std::sqrt(x*x + y*y + z*z) - 1.0) < 1e-9;   // still on the unit circle
        ok &= std::abs(y) < 1e-9;                                  // still in the xz plane
    }

    // ---- M9bq: EXP/LOG ROUND-TRIP ON THE TYPE, and the half-angle written down once.
    // IntegrateStep(twist, dt) must equal a rotation of |omega|*dt about the twist's axis.
    {
        const double om[3] = {0.0, 2.0, 0.0}, vo[3] = {0.0, 0.0, 0.0};
        const Motor step = IntegrateStep(Bivector::Twist(om, vo), 0.25);   // 0.5 rad about +y
        double x = 1, y = 0, z = 0;
        step.TransformPoint(x, y, z);
        ok &= near3(x, y, z, std::cos(0.5), 0.0, -std::sin(0.5));
    }

    Log("[pga] motor self-test: %s (offset-axis rotation, composition, rigidity, screw "
        "log/exp + slerp, normalize, wrench/twist transport, power invariance, Chasles, "
        "100k-step integrator)",
        ok ? "PASS" : "FAIL");
    return ok;
}

}  // namespace ga
