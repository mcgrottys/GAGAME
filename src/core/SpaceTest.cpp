// RunSpaceSelfTest -- the gate on Space.h (M12 step 2c): the frame calculus as code.
//
// What is pinned, and against what:
//   1. The group: Then / Inverse / Identity on points, associativity, the fold vs stepwise
//      application -- pure algebra, exact to rounding.
//   2. The two conventions this file must NOT add to: Rigid(motor) acts exactly as the motor
//      does (Pga.h), and Similar(p, s, axis, twist) acts exactly as Droste.h's Portal does --
//      the same portal RunDrosteSelfTest builds, the same closed forms, at every power k.
//   3. The versor: Placement::Versor(R) equals droste::SimilarityVersor coefficient by
//      coefficient, and the point sandwich through it agrees with the closed form.
//   4. The gauge identity S^k(y) - C = s^k Q^k (y - S^-k(C)) through Level()/Inverse(), and the
//      plane transport: PullPlane against GlobeLayer's hand arithmetic (t = 0) AND against the
//      general point test n'.x - d' = (n.S(x) - d)/s.
//   5. The unit-length rule (priors 32): Declare() refuses an extent past the conformal collapse
//      and accepts inside it; Embed keeps P.ni = -1.
//   6. Frame() rows round-trip; the screw power of a rigid placement equals Motor::Slerp.
// A defect planted in Then (a dropped scale factor) was seen to fail blocks 1, 2 and 4 before
// this gate was trusted (priors 22: a check that cannot fail has not been asked the question).
#include "core/Space.h"

#include "core/Cga.h"
#include "core/Common.h"
#include "core/Droste.h"
#include "core/Pga.h"

#include <cmath>
#include <string>

namespace ga {

namespace {

struct Gate {
    bool ok = true;
    int checks = 0;
    void Near(double a, double b, double tol, const char* what) {
        ++checks;
        if (std::fabs(a - b) <= tol) return;
        Log("[space] FAIL %s: %.17g vs %.17g (tol %g)", what, a, b, tol);
        ok = false;
    }
    void Near3(const double a[3], const double b[3], double tol, const char* what) {
        for (int i = 0; i < 3; ++i) Near(a[i], b[i], tol, what);
    }
    void True(bool v, const char* what) {
        ++checks;
        if (v) return;
        Log("[space] FAIL %s", what);
        ok = false;
    }
};

// A deterministic scatter of points at planet scale (the Droste gate's own habit).
void Scatter(int i, double R, double out[3]) {
    const double a = 0.731 * i + 0.2, b = 1.37 * i + 0.9;
    out[0] = R * 0.8 * std::cos(a) * std::sin(b);
    out[1] = R * 0.8 * std::sin(a) * std::sin(b) - R;
    out[2] = R * 0.8 * std::cos(b);
}

}  // namespace

bool RunSpaceSelfTest() {
    Gate g;
    const double R = 6371000.0;

    // ---- the portal, built as RunDrosteSelfTest builds it (Droste.h) -----------------------
    const double lat = 42.8183 * 3.14159265358979 / 180.0, lon = -70.81 * 3.14159265358979 / 180.0;
    const double o[3] = {std::cos(lat) * std::cos(lon), std::sin(lat), std::cos(lat) * std::sin(lon)};
    const double yl = std::sqrt(o[0] * o[0] + o[2] * o[2]);
    const double E[3] = {-o[2] / yl, 0.0, o[0] / yl};
    const double N[3] = {E[1] * o[2] - E[2] * o[1], E[2] * o[0] - E[0] * o[2],
                         E[0] * o[1] - E[1] * o[0]};
    const double d0 = 780.0 / R;
    double dl[3] = {o[0] + E[0] * d0, o[1] + E[1] * d0, o[2] + E[2] * d0};
    const double dn = std::sqrt(dl[0] * dl[0] + dl[1] * dl[1] + dl[2] * dl[2]);
    for (double& v : dl) v /= dn;
    const double axisN[3] = {0.0, 0.0, 1.0};
    const droste::Portal pt =
        droste::BuildPortal(5, 16, 0, 0, dl, E, o, N, R, -5.0, 1.0, axisN, 1.5707963267948966);
    const Placement S = Placement::Similar(pt.p, pt.s, pt.axis, pt.twist);

    // ---- 1. the group ----------------------------------------------------------------------
    {
        const double origin[3] = {0.0, 0.0, 0.0};
        const Placement F = Placement::Frame(E, o, N, origin);   // planet -> tangent rows, no offset
        const Placement T = Placement::Rigid(Motor::Translation(120.0, 7.0, -10.0));
        const Placement A = F.Then(S), B = S.Then(T);
        for (int i = 0; i < 12; ++i) {
            double x[3], y1[3], y2[3], z1[3], z2[3];
            Scatter(i, R, x);
            // (A.Then(B))(x) == A(B(x))
            A.Then(B).Apply(x, y1);
            B.Apply(x, z1);
            A.Apply(z1, y2);
            g.Near3(y1, y2, 1e-6 * R, "Then composes: (A.B)(x) = A(B(x))");
            // associativity
            F.Then(S.Then(T)).Apply(x, y1);
            (F.Then(S)).Then(T).Apply(x, y2);
            g.Near3(y1, y2, 1e-6 * R, "Then is associative");
            // inverse
            S.Then(S.Inverse()).Apply(x, z2);
            g.Near3(z2, x, 1e-6 * R, "S . S^-1 = 1 on points");
            S.Inverse().Then(S).Apply(x, z2);
            g.Near3(z2, x, 1e-6 * R, "S^-1 . S = 1 on points");
            Placement::Identity().Apply(x, z2);
            g.Near3(z2, x, 0.0, "the identity is exact");
        }
    }

    // ---- 2. no second convention: Rigid == the motor, Similar == the portal ---------------
    {
        const double p0[3] = {40.0, 2.0, -15.0}, ax[3] = {0.0, 1.0, 0.0};
        const Motor m = Motor::Translation(3.0, -1.0, 8.0) * Motor::Rotation(p0, ax, 0.7);
        const Placement P = Placement::Rigid(m);
        for (int i = 0; i < 8; ++i) {
            double x[3], a[3];
            Scatter(i, 1000.0, x);
            P.Apply(x, a);
            double bx = x[0], by = x[1], bz = x[2];
            m.TransformPoint(bx, by, bz);
            const double b[3] = {bx, by, bz};
            g.Near3(a, b, 1e-9, "Rigid(motor) acts as the motor acts");
            double dd[3], da[3];
            P.ApplyDir(x, dd);
            double cx = x[0], cy = x[1], cz = x[2];
            m.TransformDir(cx, cy, cz);
            da[0] = cx; da[1] = cy; da[2] = cz;
            g.Near3(dd, da, 1e-9, "Rigid(motor) rotates directions as the motor does");
        }
        const double ks[] = {-2.0, -1.0, 0.0, 0.5, 1.0, 2.0, 3.0};
        for (double k : ks) {
            const Placement Sk = S.Pow(k);
            g.Near(Sk.s, pt.Scale(k), 1e-12 * std::fabs(pt.Scale(k)), "Pow(k).s = Scale(k)");
            for (int i = 0; i < 8; ++i) {
                double x[3], a[3], b[3];
                Scatter(i, R, x);
                Sk.Apply(x, a);
                pt.Apply(k, x, b);
                const double mag = R + std::fabs(b[0]) + std::fabs(b[1]) + std::fabs(b[2]);
                g.Near3(a, b, 1e-9 * mag, "Level(k) = the portal's closed form S^k");
                double da[3], db[3];
                Sk.ApplyDir(x, da);
                pt.ApplyDir(k, x, db);
                g.Near3(da, db, 1e-9 * R, "Level(k) on directions = the portal's Q^k");
            }
        }
        // Pow composes: S^0.5 . S^0.5 = S, S^2 = S . S, S^-1 = Inverse.
        for (int i = 0; i < 6; ++i) {
            double x[3], a[3], b[3];
            Scatter(i, R, x);
            S.Pow(0.5).Then(S.Pow(0.5)).Apply(x, a);
            S.Apply(x, b);
            g.Near3(a, b, 1e-6 * R, "S^1/2 . S^1/2 = S");
            S.Pow(2.0).Apply(x, a);
            S.Then(S).Apply(x, b);
            g.Near3(a, b, 1e-6 * R, "S^2 = S . S");
            S.Pow(-1.0).Apply(x, a);
            S.Inverse().Apply(x, b);
            g.Near3(a, b, 1e-6 * R, "S^-1 = Inverse");
        }
        // The fixed point is fixed, and it is the portal's.
        double fp[3], fpS[3];
        S.FixedPoint(fp);
        g.Near3(fp, pt.p, 1e-6 * R, "FixedPoint = the portal's p");
        S.Apply(fp, fpS);
        g.Near3(fpS, fp, 1e-6 * R, "S(p) = p");
    }

    // ---- 3. the versor -----------------------------------------------------------------------
    {
        const cga::Mv V = S.Versor(R);
        for (uint32_t b = 0; b < cga::kBlades; ++b) {
            g.Near(V.c[b], pt.S.c[b], 1e-12, "Versor(R) = droste::SimilarityVersor, per coefficient");
        }
        for (int i = 0; i < 8; ++i) {
            double x[3], a[3], b[3];
            Scatter(i, R, x);
            S.Apply(x, a);
            droste::SandwichPoint(V, x, R, b);
            g.Near3(a, b, 2e-4, "the versor's sandwich agrees with the closed form");
        }
    }

    // ---- 4. the gauge identity and the plane transport ---------------------------------------
    {
        const double C[3] = {632.0, 71.0, 30.0};   // the droste still's eye
        for (double k : {-1.0, 0.0, 1.0, 2.0}) {
            const Placement Sk = S.Pow(k);
            double Ck[3];
            Sk.Inverse().Apply(C, Ck);   // the eye in level k's frame: S^-k(C)
            for (int i = 0; i < 6; ++i) {
                double y[3], lhs[3], rhs[3], ymC[3], rot[3];
                Scatter(i, R, y);
                Sk.Apply(y, lhs);
                for (int j = 0; j < 3; ++j) { lhs[j] -= C[j]; ymC[j] = y[j] - Ck[j]; }
                Sk.ApplyDir(ymC, rot);
                for (int j = 0; j < 3; ++j) rhs[j] = Sk.s * rot[j];
                const double mag = R + std::fabs(rhs[0]) + std::fabs(rhs[1]) + std::fabs(rhs[2]);
                g.Near3(lhs, rhs, 1e-9 * mag, "S^k(y) - C = s^k Q^k (y - S^-k(C))");
            }
        }
        // PullPlane, camera-relative (t = 0): n' = Q^T n, d' = d / sigma -- GlobeLayer's form.
        {
            Placement Q = S;
            Q.t[0] = Q.t[1] = Q.t[2] = 0.0;
            const double n[3] = {0.6, 0.0, 0.8};
            double m[3][3], nq[3], np[3], dp;
            pt.Rot(1.0, m);
            nq[0] = m[0][0] * n[0] + m[1][0] * n[1] + m[2][0] * n[2];   // Q^T n
            nq[1] = m[0][1] * n[0] + m[1][1] * n[1] + m[2][1] * n[2];
            nq[2] = m[0][2] * n[0] + m[1][2] * n[1] + m[2][2] * n[2];
            Q.PullPlane(n, 5.0, np, dp);
            g.Near3(np, nq, 1e-12, "PullPlane normal = Q^T n (GlobeLayer's transport)");
            g.Near(dp, 5.0 / pt.Scale(1.0), 1e-9 * (5.0 / pt.Scale(1.0)), "PullPlane d' = d / sigma");
        }
        // PullPlane, general: n'.x - d' = (n.S(x) - d) / s for every x in the own frame.
        {
            const double n[3] = {0.0, 0.7071067811865476, 0.7071067811865476}, d = 12.5;
            double np[3], dp;
            S.PullPlane(n, d, np, dp);
            for (int i = 0; i < 8; ++i) {
                double x[3], sx[3];
                Scatter(i, R, x);
                S.Apply(x, sx);
                const double lhs = np[0] * x[0] + np[1] * x[1] + np[2] * x[2] - dp;
                const double rhs = (n[0] * sx[0] + n[1] * sx[1] + n[2] * sx[2] - d) / S.s;
                g.Near(lhs, rhs, 1e-6 * R / S.s, "PullPlane: the pulled plane is the plane's preimage");
            }
        }
    }

    // ---- 5. the unit-length rule -------------------------------------------------------------
    {
        Space planet;
        planet.name = "planet.re";
        planet.unitM = R;
        planet.extentM = 2.0 * R;
        std::string why;
        g.True(planet.Declare(&why), "a planet at unit length R declares");
        Space sun;
        sun.name = "sun.in.metres";
        sun.unitM = 1.0;
        sun.extentM = 1.496e11;
        g.True(!sun.Declare(&why), "the sun in metres is refused (past 9.49e7 units)");
        g.True(why.find("point at infinity") != std::string::npos, "the refusal says why");
        sun.unitM = 1.495978707e11;
        g.True(sun.Declare(&why), "the sun at unit length 1 AU declares");
        const double x[3] = {2.0 * R, 0.0, 0.0};
        const cga::Mv P = planet.Embed(x);
        g.Near(cga::Dot(P, cga::Ni()), -1.0, 1e-9, "Embed keeps P.ni = -1 inside the reach");
    }

    // ---- 6. the fold, Frame() round trip, the screw power ------------------------------------
    {
        Space root;
        root.name = "planet.re";
        root.unitM = R;
        Space tangent;
        tangent.name = "tangent.act0816";
        tangent.unitM = 1.0;
        tangent.parent = &root;
        const double origin[3] = {0.0, -R, 0.0};   // the planet's centre sits at (0,-R,0) in the tangent frame
        tangent.link = Placement::Frame(E, o, N, origin).Inverse();
        Space tower = Space::Cycle("droste.tower", tangent, S);
        for (int i = 0; i < 6; ++i) {
            double x[3], a[3], b[3], c[3];
            Scatter(i, 1000.0, x);
            tower.ToRoot().Apply(x, a);                      // the fold
            S.Apply(x, b);                                    // stepwise
            tangent.link.Apply(b, c);
            g.Near3(a, c, 1e-6 * R, "ToRoot() is the stepwise product");
            tower.To(tangent).Apply(x, b);
            S.Apply(x, c);
            g.Near3(b, c, 1e-6 * R, "To(parent) is the link itself");
            tangent.To(tower).Then(tower.To(tangent)).Apply(x, b);
            g.Near3(b, x, 1e-6 * R, "To() round-trips");
            tower.Level(2.0).Apply(x, b);
            S.Then(S).Apply(x, c);
            g.Near3(b, c, 1e-6 * R, "Level(2) = S . S");
        }
        // Frame(): rows in, rows out.
        double e2[3], u2[3], n2[3];
        Placement::Frame(E, o, N, origin).Rows(e2, u2, n2);
        g.Near3(e2, E, 1e-14, "Frame() east row round-trips");
        g.Near3(u2, o, 1e-14, "Frame() up row round-trips");
        g.Near3(n2, N, 1e-14, "Frame() north row round-trips");
        // The screw power: Rigid(m).Pow(u) = Rigid(Slerp(1, m, u)).
        const double p0[3] = {10.0, 0.0, 5.0}, ax[3] = {0.0, 1.0, 0.0};
        const Motor m = Motor::Translation(50.0, 4.0, -20.0) * Motor::Rotation(p0, ax, 1.1);
        for (double u : {0.25, 0.5, 0.75}) {
            const Placement Pu = Placement::Rigid(m).Pow(u);
            const Placement Su = Placement::Rigid(Motor::Slerp(Motor::Identity(), m, u));
            for (int i = 0; i < 4; ++i) {
                double x[3], a[3], b[3];
                Scatter(i, 1000.0, x);
                Pu.Apply(x, a);
                Su.Apply(x, b);
                g.Near3(a, b, 1e-6, "a rigid placement's Pow is the screw (Motor::Slerp)");
            }
        }
        // The anchor chart round-trips.
        Space::Anchor an;
        an.latDeg = 42.81833; an.lonDeg = -70.81; an.mPerLat = 110574.0; an.mPerLon = 81660.0;
        an.linear = true;
        double la, lo, fx, fz;
        an.LatLonOf(1234.5, -678.9, la, lo);
        an.FlatOf(la, lo, fx, fz);
        g.Near(fx, 1234.5, 1e-9, "anchor chart x round-trips");
        g.Near(fz, -678.9, 1e-9, "anchor chart z round-trips");
    }

    if (g.ok) {
        Log("[space] ---- PASS (%d checks): the group and the fold, Rigid = the motor, Similar = "
            "the portal at every power, Versor = SimilarityVersor, the gauge identity, the plane "
            "transport (GlobeLayer's form and the general preimage), the unit-length refusal, "
            "Frame() rows, the screw power ----",
            g.checks);
    } else {
        Log("[space] ---- FAIL (%d checks) ----", g.checks);
    }
    return g.ok;
}

}  // namespace ga
