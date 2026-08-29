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
    Log("[pga] motor self-test: %s (offset-axis rotation, composition, rigidity, screw "
        "log/exp + slerp)",
        ok ? "PASS" : "FAIL");
    return ok;
}

}  // namespace ga
