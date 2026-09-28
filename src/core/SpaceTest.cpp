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
//   7. (step 4d-2) PowApply / PowApplyDir -- the power applied about its fixed point -- against
//      the portal's closed forms at block 2's tolerance and against Pow(k).Apply (the same map
//      about the origin), the fixed point fixed at every power, the rigid fallback Pow(u).Apply
//      bit for bit; the largest ulp distance seen against the portal is printed in the PASS
//      line, a record and not a gate.
// A defect planted in Then (a dropped scale factor) was seen to fail blocks 1, 2 and 4 before
// this gate was trusted (priors 22: a check that cannot fail has not been asked the question).
//
// RunFaceWindowSelfTest -- HIERARCHY step 3's gate on core/Lattice.h's FaceWindow, the address
// of a face-plane window, beside this one because its planes move by PullPlane. It runs AFTER
// spacetest's verdict, never inside it, so that verdict's lines and its check count stay what
// they were. What is pinned, and against what:
//   a. CubeFaceAxes against CubeFaceOfDir (20000 random directions: the (s, t) of the axes is
//      the inverse's to 1e-12, and the face's normal is the one the direction leans on most)
//      and against ComposeCubeDir (20000 random face points: back to their own (s, t), 1e-12).
//   b. tools/hierarchy/uv_precision.py's experiment, point for point -- its random.Random(7),
//      place, reaches and window: the float32 twin of PageTexel at rungs 6, 9, 12 and 15 against
//      the double reference, the eye 3 m and 10 km over the Merrimack. THE GATE is the helm's:
//      under 0.01 texel at every rung (10 km is a record: nothing that high wants rung 15). The
//      script's numbers are printed beside, and so is the script's own spelling evaluated here,
//      which must reproduce them -- the harness is its experiment, so what differs is the
//      spelling (three dots and w, k folded into the rows) and the anchor (the multiple of 16384
//      nearest the eye, not the script's half-page origin).
//   c. the same points in the eye's tangent frame (east, up, north): the rows' normals general
//      vectors, not axes. Same bound.
//   d. that frame scaled by 0.001 and by 1000, a Droste level's case: same bound, so the scale
//      cancels.
//   e. THE PLANT: the rows built the wrong way, w's cancellation taken in float32 (m and the
//      eye cast first, dotted after). Past the bound at rungs 12 and 15 it is CAUGHT; anything
//      less and the instrument is blind, which fails the gate.
#include "core/Space.h"

#include "core/Cga.h"
#include "core/Common.h"
#include "core/Droste.h"
#include "core/Lattice.h"
#include "core/Pga.h"

#include <algorithm>
#include <cmath>
#include <random>
#include <string>
#include <vector>

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
    uint64_t maxUlpPt = 0, maxUlpDir = 0;   // 4d-2: PowApply against the portal, the record

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
                // 4d-2: the power applied about its fixed point -- the form the tower is drawn
                // by -- against the portal's closed form at this block's tolerance, against
                // Pow(k).Apply (the same map about the origin), and its ulp distance recorded.
                double c[3], dc[3];
                S.PowApply(k, x, c);
                g.Near3(c, b, 1e-9 * mag, "PowApply(k) = the portal's closed form about p");
                g.Near3(c, a, 1e-9 * mag, "PowApply(k) = Pow(k).Apply, the same map about the origin");
                S.PowApplyDir(k, x, dc);
                g.Near3(dc, db, 1e-9 * R, "PowApplyDir(k) = the portal's Q^k");
                for (int j = 0; j < 3; ++j) {
                    const uint64_t up = UlpDistance(c[j], b[j]), ud = UlpDistance(dc[j], db[j]);
                    if (up > maxUlpPt) maxUlpPt = up;
                    if (ud > maxUlpDir) maxUlpDir = ud;
                }
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
        for (double k : ks) {
            double fk[3];
            S.PowApply(k, fp, fk);
            g.Near3(fk, fp, 1e-6 * R, "PowApply(k) leaves p fixed at every power");
        }
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
                // 4d-2: no fixed point at s = 1 -- PowApply is Pow(u).Apply there, bit for bit,
                // and PowApplyDir the rotor's own power.
                double c[3], dc[3], dd[3];
                Placement::Rigid(m).PowApply(u, x, c);
                g.Near3(c, a, 0.0, "a rigid PowApply is Pow(u).Apply, bit for bit");
                Placement::Rigid(m).PowApplyDir(u, x, dc);
                Pu.ApplyDir(x, dd);
                g.Near3(dc, dd, 1e-9, "a rigid PowApplyDir is the screw's rotation");
            }
        }
        // To() through the nearest common ancestor: equal to the root form where the root form is
        // exact, and better where it is not -- two siblings 40 m apart under a tangent frame that
        // sits a planet radius from the root must see 40 m, not 40 m +- the radius' ulp.
        Space boatA, boatB;
        boatA.name = "boat.a";
        boatA.parent = &tangent;
        boatA.link = Placement::Rigid(Motor::Translation(100.0, 0.0, -20.0));
        boatB.name = "boat.b";
        boatB.parent = &tangent;
        boatB.link = Placement::Rigid(Motor::Translation(140.0, 0.0, -20.0));
        const Placement ab = boatA.To(boatB);   // a in b's frame
        g.Near(ab.t[0], -40.0, 1e-12, "To() through the LCA: 40 m apart is exactly 40 m");
        g.Near(ab.t[1], 0.0, 1e-12, "To() through the LCA: no vertical leak");
        g.Near(ab.t[2], 0.0, 1e-12, "To() through the LCA: no lateral leak");
        const Placement abRoot = boatB.ToRoot().Inverse().Then(boatA.ToRoot());
        g.True(std::fabs(abRoot.t[0] + 40.0) <= 1e-6 * R, "the root form agrees to the radius' ulp");
        g.True(std::fabs(ab.t[0] + 40.0) <= std::fabs(abRoot.t[0] + 40.0) + 1e-12,
               "the LCA form is never worse than the root form");
        for (int i = 0; i < 4; ++i) {
            double x[3], a[3], b[3];
            Scatter(i, 1000.0, x);
            tower.To(tangent).Apply(x, a);
            S.Apply(x, b);
            g.Near3(a, b, 1e-6 * R, "To(parent) through the LCA is the link");
        }
        Space lone;
        lone.name = "another.world";
        g.True(lone.To(tangent).IsRigid() && lone.To(tangent).t[0] == 0.0,
               "two worlds with no common ancestor are refused (identity)");
        // Normalize: a long product drifts off the unit rotor; Normalize puts it back.
        Placement drift = Placement::Rigid(m);
        for (int i = 0; i < 2000; ++i) drift = drift.Then(Placement::Rigid(m).Pow(0.001));
        const double n0 = std::sqrt(drift.r[0] * drift.r[0] + drift.r[1] * drift.r[1] +
                                    drift.r[2] * drift.r[2] + drift.r[3] * drift.r[3]);
        drift.Normalize();
        const double n1 = std::sqrt(drift.r[0] * drift.r[0] + drift.r[1] * drift.r[1] +
                                    drift.r[2] * drift.r[2] + drift.r[3] * drift.r[3]);
        g.True(std::fabs(n1 - 1.0) <= 1e-15, "Normalize re-unitizes the rotor");
        g.True(std::fabs(n1 - 1.0) <= std::fabs(n0 - 1.0), "Normalize never makes it worse");

        // PARITY. The engine's planet frame is left-handed (priors 34): ECEF (X, Y, Z) -> planet
        // (X, Z, Y) is the axis swap, det -1. Frame() must carry it, not silently rotate.
        {
            const double ex[3] = {1, 0, 0}, ey[3] = {0, 0, 1}, ez[3] = {0, 1, 0}, o0[3] = {0, 0, 0};
            const Placement F = Placement::Frame(ex, ey, ez, o0);   // columns: e_x, e_z, e_y
            g.True(F.s < 0.0, "a left-handed frame is an improper placement (s < 0)");
            double pe[3], pu[3], pn[3];
            F.Rows(pe, pu, pn);
            g.Near3(pe, ex, 1e-14, "improper Frame() east row round-trips");
            g.Near3(pu, ey, 1e-14, "improper Frame() up row round-trips");
            g.Near3(pn, ez, 1e-14, "improper Frame() north row round-trips");
            for (int i = 0; i < 6; ++i) {
                double x[3], a[3], b[3];
                Scatter(i, 1000.0, x);
                F.Apply(x, a);
                const double want[3] = {x[0], x[2], x[1]};   // the swap
                g.Near3(a, want, 1e-9, "the reflection swaps Y and Z");
                droste::SandwichPoint(F.Versor(1000.0), x, 1000.0, b);
                g.Near3(a, b, 1e-6, "the odd versor's sandwich is the reflection");
                double d[3];
                F.ApplyDir(x, d);
                g.Near3(d, want, 1e-9, "a direction flips with the parity");
                F.Then(F).Apply(x, b);
                g.Near3(b, x, 1e-9, "two reflections are the identity");
            }
            g.True(F.Then(F).s > 0.0, "two reflections compose to a proper placement");
            g.True(F.Inverse().s < 0.0, "the inverse of a reflection is a reflection");
            g.True(F.Pow(0.5).IsRigid() && F.Pow(0.5).t[0] == 0.0, "a half reflection is refused");
            double p2[3], q2[3];
            F.Pow(2.0).Apply(p2, q2);   // whole powers compose
            g.Near(F.Pow(3.0).s, -1.0, 0.0, "an odd whole power keeps the parity");
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
            "the portal at every power, PowApply = the portal's closed form about p (within %llu "
            "ulps on points, %llu on directions), Versor = SimilarityVersor, the gauge identity, "
            "the plane transport (GlobeLayer's form and the general preimage), the unit-length "
            "refusal, Frame() rows, the screw power, To() through the nearest common ancestor, "
            "Normalize, parity (the left-handed planet frame as an improper placement) ----",
            g.checks, static_cast<unsigned long long>(maxUlpPt),
            static_cast<unsigned long long>(maxUlpDir));
    } else {
        Log("[space] ---- FAIL (%d checks) ----", g.checks);
    }
    return g.ok;
}

// ================================================================ HIERARCHY step 3: the address
namespace {

// Python's random.Random(seed).random(), so that this gate walks the very points
// uv_precision.py walks: MT19937 seeded by init_by_array with the one-word key {seed} (CPython's
// seeding of an int), and 53 bits from two draws.
struct PyRandom {
    uint32_t mt[624];
    int at = 624;
    explicit PyRandom(uint32_t seed) {
        mt[0] = 19650218u;
        for (uint32_t k = 1; k < 624; ++k) mt[k] = 1812433253u * (mt[k - 1] ^ (mt[k - 1] >> 30)) + k;
        uint32_t i = 1;
        for (int k = 624; k > 0; --k) {   // the key has one word, so its index stays 0
            mt[i] = (mt[i] ^ ((mt[i - 1] ^ (mt[i - 1] >> 30)) * 1664525u)) + seed;
            if (++i >= 624) {
                mt[0] = mt[623];
                i = 1;
            }
        }
        for (int k = 623; k > 0; --k) {
            mt[i] = (mt[i] ^ ((mt[i - 1] ^ (mt[i - 1] >> 30)) * 1566083941u)) - i;
            if (++i >= 624) {
                mt[0] = mt[623];
                i = 1;
            }
        }
        mt[0] = 0x80000000u;
    }
    uint32_t Next() {
        if (at >= 624) {
            for (int k = 0; k < 624; ++k) {
                const uint32_t y = (mt[k] & 0x80000000u) | (mt[(k + 1) % 624] & 0x7fffffffu);
                mt[k] = mt[(k + 397) % 624] ^ (y >> 1) ^ ((y & 1u) ? 0x9908b0dfu : 0u);
            }
            at = 0;
        }
        uint32_t y = mt[at++];
        y ^= y >> 11;
        y ^= (y << 7) & 0x9d2c5680u;
        y ^= (y << 15) & 0xefc60000u;
        y ^= y >> 18;
        return y;
    }
    double Random() {
        const uint32_t a = Next() >> 5, b = Next() >> 6;
        return (a * 67108864.0 + b) * (1.0 / 9007199254740992.0);
    }
};

// The script's place, sphere and reaches (a rung's ground points are drawn within its reach of
// the eye), and its RELATIVE column as it printed it, eye 3 m and eye 10 km, for the lines
// beside this gate's own.
constexpr double kHelmLatDeg = 42.816, kHelmLonDeg = -70.8125, kSphereR = 6371000.0;
constexpr int kRungs[4] = {6, 9, 12, 15};
constexpr double kReachM[4] = {60000.0, 8000.0, 1000.0, 120.0};
constexpr double kScriptHelm[4] = {0.002594, 0.002472, 0.002508, 0.002919};
constexpr double kScriptHigh[4] = {0.001980, 0.002772, 0.007176, 0.045293};
constexpr double kTexelBound = 0.01;   // THE GATE, in texels of the rung

// A max that keeps a NaN: a twin that returned one must fail the bound, not vanish in a max.
void Worse(double& worst, double e) {
    if (!(e <= worst)) worst = e;
}

// One rung of the script's run(): the eye, the script's window origin (the half-page lattice,
// the eye in its central half), the window this code anchors nearest the eye, and the points
// that fall inside the script's window -- planet frame, each with its global texel.
struct AddressRun {
    FaceWindow win;
    double eye[3] = {};
    double org[2] = {};
    std::vector<double> P;   // x y z per point
    std::vector<double> X;   // the texel anchored at the face's corner, x y per point
    size_t Count() const { return P.size() / 3; }
    // Relative to an anchor: exact, the difference of two doubles a window apart.
    double Ref(size_t i, int axis, const FaceWindow& w) const {
        return X[2 * i + axis] - double(axis == 0 ? w.anchorX : w.anchorY);
    }
    // The script's origin as a window's anchor. It is a multiple of 8192, not of 16384, so no
    // WRAP sampler could read it: it is here only to separate the spelling from the anchor.
    FaceWindow AtOrigin() const {
        FaceWindow w = win;
        w.anchorX = static_cast<long long>(org[0]);
        w.anchorY = static_cast<long long>(org[1]);
        return w;
    }
};

AddressRun RunAt(int rung, double eyeAltM, double reachM) {
    const double kDegToRad = 3.141592653589793 / 180.0;   // math.radians' own constant
    auto dirOf = [](double lat, double lon, double d[3]) {   // the script's dir_of
        const double cl = std::cos(lat);
        d[0] = cl * std::cos(lon);
        d[1] = std::sin(lat);
        d[2] = cl * std::sin(lon);
    };
    AddressRun r;
    const double lat0 = kHelmLatDeg * kDegToRad, lon0 = kHelmLonDeg * kDegToRad;
    double d0[3], uv[2];
    dirOf(lat0, lon0, d0);
    for (int i = 0; i < 3; ++i) r.eye[i] = d0[i] * (kSphereR + eyeAltM);
    const FaceWindow corner{CubeFaceOfDir(d0, uv), rung, 0, 0};
    double eu = 0.0, ev = 0.0;
    corner.TexelOf(r.eye, eu, ev);
    r.org[0] = std::floor(eu / 8192.0 - 0.5) * 8192.0;
    r.org[1] = std::floor(ev / 8192.0 - 0.5) * 8192.0;
    r.win = corner.Nearest(r.eye);
    PyRandom rnd(7);
    for (int k = 0; k < 4000; ++k) {
        const double dn = (rnd.Random() * 2 - 1) * reachM;
        const double de = (rnd.Random() * 2 - 1) * reachM;
        const double lat = lat0 + dn / kSphereR;
        const double lon = lon0 + de / (kSphereR * std::cos(lat0));
        double d[3];
        dirOf(lat, lon, d);
        const double P[3] = {d[0] * kSphereR, d[1] * kSphereR, d[2] * kSphereR};
        double X = 0.0, Y = 0.0;
        corner.TexelOf(P, X, Y);
        const double wu = X - r.org[0], wv = Y - r.org[1];
        if (!(0.0 <= wu && wu < 16384.0 && 0.0 <= wv && wv < 16384.0)) continue;
        r.P.insert(r.P.end(), P, P + 3);
        r.X.push_back(X);
        r.X.push_back(Y);
    }
    return r;
}

// A planet-frame point in `own`'s frame as the vertex path carries it: the difference from own's
// origin first, in doubles (the float wall's form), then R^T and the scale, then float32.
void OwnOf(const Placement& own, const double P[3], float out[3]) {
    const double rc[4] = {own.r[0], -own.r[1], -own.r[2], -own.r[3]};
    double x = P[0] - own.t[0], y = P[1] - own.t[1], z = P[2] - own.t[2];
    Motor::QRotate(rc, x, y, z);
    out[0] = static_cast<float>(x / own.s);
    out[1] = static_cast<float>(y / own.s);
    out[2] = static_cast<float>(z / own.s);
}

// The worst |twin - reference| over a run's points, given in `own`'s frame, read through `pl`,
// relative to w's anchor.
double WorstTexel(const AddressRun& r, const FaceWindow& w, const Placement& own,
                  const FaceWindow::Planes& pl) {
    double worst = 0.0;
    for (size_t i = 0; i < r.Count(); ++i) {
        float p[3];
        OwnOf(own, &r.P[3 * i], p);
        float x = 0.0f, y = 0.0f;
        FaceWindow::PageTexel(p, pl, x, y);
        Worse(worst, std::fabs(x - r.Ref(i, 0, w)));
        Worse(worst, std::fabs(y - r.Ref(i, 1, w)));
    }
    return worst;
}
double WorstTexel(const AddressRun& r, const FaceWindow& w, const Placement& own) {
    return WorstTexel(r, w, own, w.PlanesIn(own));
}

// uv_precision.py's texel_relative_f32 op for op, relative to w's anchor: (A + p_a - s0 p_n) /
// (E_n + p_n) * k, A = E_a - s0 E_n in doubles -- face 5's coordinates as the script writes them
// (s = x / z, t = y / z). Not the address under test: at the script's own origin it proves this
// harness IS the script's experiment, and at the twin's anchor it separates the two changes.
double WorstScript(const AddressRun& r, const FaceWindow& w) {
    const double N = w.FaceTexels();
    double worst = 0.0;
    for (size_t i = 0; i < r.Count(); ++i) {
        const double* P = &r.P[3 * i];
        for (int axis = 0; axis < 2; ++axis) {
            const double s0 = 2.0 * double(axis == 0 ? w.anchorX : w.anchorY) / N - 1.0;
            const float A = static_cast<float>(r.eye[axis] - s0 * r.eye[2]);
            const float k = static_cast<float>(0.5 * N);
            const float s0f = static_cast<float>(s0);
            const float pa = static_cast<float>(P[axis] - r.eye[axis]);
            const float pn = static_cast<float>(P[2] - r.eye[2]);
            const float en = static_cast<float>(r.eye[2]);
            const float sum = A + pa;
            const float lean = s0f * pn;
            const float num = sum - lean;
            const float den = en + pn;
            const float q = num / den;
            const float texel = q * k;
            Worse(worst, std::fabs(texel - r.Ref(i, axis, w)));
        }
    }
    return worst;
}

// THE PLANT: PlanesIn's rows with each w rebuilt the wrong way -- the plane m and own.t cast to
// float32 first and dotted after, over own.s in float32: the cancellation the rows exist to keep
// in doubles, taken where it cannot be.
FaceWindow::Planes Planted(const FaceWindow& w, const Placement& own) {
    FaceWindow::Planes pl = w.PlanesIn(own);
    double n[3], a[3], b[3];
    CubeFaceAxes(w.face, n, a, b);
    const double N = w.FaceTexels(), k = 0.5 * N;
    const double s0 = 2.0 * double(w.anchorX) / N - 1.0;
    const double t0 = 2.0 * double(w.anchorY) / N - 1.0;
    const double m[3][3] = {{k * (a[0] - s0 * n[0]), k * (a[1] - s0 * n[1]), k * (a[2] - s0 * n[2])},
                            {k * (b[0] - t0 * n[0]), k * (b[1] - t0 * n[1]), k * (b[2] - t0 * n[2])},
                            {n[0], n[1], n[2]}};
    const float t[3] = {static_cast<float>(own.t[0]), static_cast<float>(own.t[1]),
                        static_cast<float>(own.t[2])};
    const float s = static_cast<float>(own.s);
    float* const rows[3] = {pl.u, pl.v, pl.w};
    for (int i = 0; i < 3; ++i) {
        const float m0 = static_cast<float>(m[i][0]) * t[0];
        const float m1 = static_cast<float>(m[i][1]) * t[1];
        const float m2 = static_cast<float>(m[i][2]) * t[2];
        const float d01 = m0 + m1;
        const float d = d01 + m2;
        rows[i][3] = d / s;
    }
    return pl;
}

// The frames a run is read in: the planet's own axes at the eye (a translation), and the eye's
// tangent frame -- the engine's rows (FrameLoop: up the eye's direction, east d(dir)/dlon, north
// = east x up, a proper rotation in this left-handed planet frame).
Placement AxesAt(const double eye[3]) {
    Placement o;
    for (int i = 0; i < 3; ++i) o.t[i] = eye[i];
    return o;
}
Placement TangentAt(const double eye[3]) {
    const double l = std::sqrt(eye[0] * eye[0] + eye[1] * eye[1] + eye[2] * eye[2]);
    const double up[3] = {eye[0] / l, eye[1] / l, eye[2] / l};
    const double yl = std::sqrt(up[0] * up[0] + up[2] * up[2]);
    const double east[3] = {-up[2] / yl, 0.0, up[0] / yl};
    const double north[3] = {east[1] * up[2] - east[2] * up[1], east[2] * up[0] - east[0] * up[2],
                             east[0] * up[1] - east[1] * up[0]};
    return Placement::Frame(east, up, north, eye);
}

std::string Four(const double v[4], const char* fmt) {
    std::string s;
    char b[32];
    for (int i = 0; i < 4; ++i) {
        snprintf(b, sizeof b, fmt, v[i]);
        s += (i ? " " : "") + std::string(b);
    }
    return s;
}

}  // namespace

bool RunFaceWindowSelfTest() {
    Gate g;

    // ---- a. the face axes against the cube's two maps ---------------------------------------
    {
        std::mt19937 rng(0x3add5u);
        std::uniform_real_distribution<double> U(-1.0, 1.0), V(0.0, 1.0);
        auto dot = [](const double x[3], const double y[3]) {
            return x[0] * y[0] + x[1] * y[1] + x[2] * y[2];
        };
        double worstInv = 0.0, worstFwd = 0.0;
        uint32_t faces[6] = {0, 0, 0, 0, 0, 0}, leansElsewhere = 0, directions = 0;
        while (directions < 20000) {
            double d[3] = {U(rng), U(rng), U(rng)};
            const double l = std::sqrt(dot(d, d));
            if (l < 1e-6) continue;
            for (double& c : d) c /= l;
            ++directions;
            double uv[2], n[3], a[3], b[3];
            const uint32_t f = CubeFaceOfDir(d, uv);
            ++faces[f];
            CubeFaceAxes(f, n, a, b);
            const double dn = dot(d, n);
            Worse(worstInv, std::fabs(dot(d, a) / dn - (uv[0] * 2.0 - 1.0)));
            Worse(worstInv, std::fabs(dot(d, b) / dn - (uv[1] * 2.0 - 1.0)));
            for (uint32_t o = 0; o < 6; ++o) {   // the face the inverse chose is the one it leans on
                double no[3], ao[3], bo[3];
                CubeFaceAxes(o, no, ao, bo);
                if (dot(d, no) > dn) ++leansElsewhere;
            }
        }
        for (int i = 0; i < 20000; ++i) {
            const uint32_t f = static_cast<uint32_t>(i % 6);
            const double u = V(rng), v = V(rng);
            double d[3], n[3], a[3], b[3];
            ComposeCubeDir(f, u, v, d);
            CubeFaceAxes(f, n, a, b);
            const double dn = dot(d, n);
            if (!(dn > 0.0)) ++leansElsewhere;
            Worse(worstFwd, std::fabs(dot(d, a) / dn - (u * 2.0 - 1.0)));
            Worse(worstFwd, std::fabs(dot(d, b) / dn - (v * 2.0 - 1.0)));
        }
        g.Near(worstInv, 0.0, 1e-12, "address a: (s, t) from CubeFaceAxes = CubeFaceOfDir's");
        g.Near(worstFwd, 0.0, 1e-12,
               "address a: ComposeCubeDir's point, back through CubeFaceAxes, is its own (s, t)");
        g.True(leansElsewhere == 0, "address a: every direction leans most on its own face's normal");
        Log("[space] address a. the face axes: 20000 directions through CubeFaceOfDir (faces "
            "%u/%u/%u/%u/%u/%u), (s, t) from CubeFaceAxes off the inverse's by at most %.1e; "
            "20000 points of ComposeCubeDir back to their own (s, t) within %.1e; %u directions "
            "lean harder on another face's normal",
            faces[0], faces[1], faces[2], faces[3], faces[4], faces[5], worstInv, worstFwd,
            leansElsewhere);
    }

    // ---- b to e. the script's experiment -----------------------------------------------------
    // Per eye (3 m, then 10 km) and rung: the twin at the anchor nearest the eye (b), the script's
    // spelling at its own origin and at that anchor, the twin at the script's origin -- the two
    // changes apart -- then, at the helm, the tangent frame (c), its scalings (d), the plant (e).
    struct Eye {
        double altM;
        const double* printed;
        double twin[4], scriptOrg[4], scriptNear[4], twinOrg[4], count[4];
    } eyes[2] = {{3.0, kScriptHelm, {}, {}, {}, {}, {}}, {10000.0, kScriptHigh, {}, {}, {}, {}, {}}};
    double tangentW[4], milliW[4], kiloW[4], plantW[4];
    uint32_t face = 0;
    size_t helmPoints = 0;
    for (Eye& e : eyes) {
        for (int r = 0; r < 4; ++r) {
            const AddressRun run = RunAt(kRungs[r], e.altM, kReachM[r]);
            const Placement axes = AxesAt(run.eye);
            e.twin[r] = WorstTexel(run, run.win, axes);
            e.scriptOrg[r] = WorstScript(run, run.AtOrigin());
            e.scriptNear[r] = WorstScript(run, run.win);
            e.twinOrg[r] = WorstTexel(run, run.AtOrigin(), axes);
            e.count[r] = double(run.Count());
            if (&e != &eyes[0]) continue;
            face = run.win.face;
            helmPoints += run.Count();
            const Placement tangent = TangentAt(run.eye);
            Placement milli = tangent, kilo = tangent;   // (not `small`: the RPC headers define it)
            milli.s *= 0.001;
            kilo.s *= 1000.0;
            tangentW[r] = WorstTexel(run, run.win, tangent);
            milliW[r] = WorstTexel(run, run.win, milli);
            kiloW[r] = WorstTexel(run, run.win, kilo);
            plantW[r] = WorstTexel(run, run.win, axes, Planted(run.win, axes));
        }
    }
    g.True(face == 5, "address: the helm is on face 5 (the script's own assertion)");
    bool within = true, caught = true;
    for (int r = 0; r < 4; ++r) {
        within = within && eyes[0].twin[r] < kTexelBound && tangentW[r] < kTexelBound &&
                 milliW[r] < kTexelBound && kiloW[r] < kTexelBound;
        if (kRungs[r] >= 12) caught = caught && plantW[r] > kTexelBound;
    }
    g.True(within, "address b-d: the twin within 0.01 texel of the doubles at the helm");
    g.True(caught, "address e: the plant past 0.01 texel at rungs 12 and 15");
    // The script's spelling at its origin against its printed digits: a record, not the gate (a
    // different libm could move a last digit without the address being any worse). The spread
    // of the four at the helm is what the two changes can do to a worst of thousands of roundings.
    int reproduced = 0;
    double lo = 1e300, hi = 0.0;
    for (const Eye& e : eyes) {
        for (int r = 0; r < 4; ++r) {
            char a[32], b[32];
            snprintf(a, sizeof a, "%.6f", e.scriptOrg[r]);
            snprintf(b, sizeof b, "%.6f", e.printed[r]);
            reproduced += std::string(a) == b ? 1 : 0;
            if (&e != &eyes[0]) continue;
            for (double v : {e.twin[r], e.scriptOrg[r], e.scriptNear[r], e.twinOrg[r]}) {
                lo = (std::min)(lo, v);
                hi = (std::max)(hi, v);
            }
        }
    }

    for (const Eye& e : eyes) {
        const bool helmEye = &e == &eyes[0];
        Log("[space] address b. the planet's axes, eye %s: rungs 6 9 12 15 over %s points, the "
            "twin's worst %s texel at the anchor nearest the eye",
            helmEye ? "3 m (THE GATE, under 0.01 texel)" : "10 km (a record: nothing that high "
                                                           "wants rung 15)",
            Four(e.count, "%.0f").c_str(), Four(e.twin, "%.6f").c_str());
        Log("[space] address b.   beside it: uv_precision.py printed %s; on these points its "
            "spelling gives %s at its origin and %s at the twin's anchor, and the twin %s at the "
            "script's origin",
            Four(e.printed, "%.6f").c_str(), Four(e.scriptOrg, "%.6f").c_str(),
            Four(e.scriptNear, "%.6f").c_str(), Four(e.twinOrg, "%.6f").c_str());
    }
    Log("[space] address b. the script's spelling %s its printed digits here at %d of 8: the same "
        "points, reference and float32 rounding. What differs is the spelling (three dots and w, "
        "N/2 folded into the rows) and the anchor (the multiple of 16384 nearest the eye, not the "
        "script's half-page origin); at the helm the four combinations span %.6f to %.6f texel",
        reproduced == 8 ? "reproduces" : "does NOT reproduce", reproduced, lo, hi);
    Log("[space] address c. the eye's tangent frame (east, up, north), eye 3 m: worst %s texel",
        Four(tangentW, "%.6f").c_str());
    Log("[space] address d. that frame scaled by 0.001, eye 3 m: worst %s texel",
        Four(milliW, "%.6f").c_str());
    Log("[space] address d. that frame scaled by 1000, eye 3 m: worst %s texel",
        Four(kiloW, "%.6f").c_str());
    Log("[space] address e. PLANTED, w's cancellation in float32 (the planet's axes, eye 3 m): "
        "worst %s texel -- %s",
        Four(plantW, "%.3g").c_str(),
        caught ? "CAUGHT: past 0.01 at rungs 12 and 15"
               : "NOT CAUGHT: the instrument cannot see a row built in float32");

    if (g.ok) {
        Log("[space] address ---- PASS (%d checks over 40000 face directions and %zu points in "
            "four frames): the face axes against CubeFaceOfDir and ComposeCubeDir; the float32 "
            "twin of PageTexel within %.2f texel of the doubles at rungs 6, 9, 12 and 15 at the "
            "helm, in the planet's axes, the eye's tangent frame and that frame scaled by 0.001 "
            "and 1000; the plant caught ----",
            g.checks, helmPoints, kTexelBound);
    } else {
        Log("[space] address ---- FAIL (%d checks) ----", g.checks);
    }
    return g.ok;
}

FaceWindowSample FaceWindowHelmSample(int rung) {
    FaceWindowSample s;
    for (int r = 0; r < 4; ++r) {
        if (kRungs[r] != rung) continue;
        const AddressRun run = RunAt(rung, 3.0, kReachM[r]);
        const Placement axes = AxesAt(run.eye);
        s.win = run.win;
        s.planes = run.win.PlanesIn(axes);
        s.planted = Planted(run.win, axes);
        for (size_t i = 0; i < run.Count(); ++i) {
            float p[3];
            OwnOf(axes, &run.P[3 * i], p);
            s.p.insert(s.p.end(), p, p + 3);
            s.ref.push_back(run.Ref(i, 0, run.win));
            s.ref.push_back(run.Ref(i, 1, run.win));
        }
    }
    return s;
}

}  // namespace ga
