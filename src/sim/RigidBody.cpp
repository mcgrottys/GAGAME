// ================================================================================================
//  RigidBody.cpp - the step, and the gates that say it is a rigid body and not a plausible
//  animation. See RigidBody.h for the four structural decisions.
// ================================================================================================
#include "sim/RigidBody.h"

#include "core/Common.h"

#include <algorithm>

namespace ga {

namespace {

void Mul3(const double m[3][3], const double v[3], double out[3]) {
    for (int i = 0; i < 3; ++i) out[i] = m[i][0] * v[0] + m[i][1] * v[1] + m[i][2] * v[2];
}

// Symmetric 3x3 inverse by cofactors. Returns false on a singular tensor -- which for an inertia
// means the spec described a body with no extent about some axis, and that is a data error worth
// hearing about rather than a NaN to discover later.
bool Inv3(const double m[3][3], double out[3][3]) {
    const double c00 = m[1][1] * m[2][2] - m[1][2] * m[2][1];
    const double c01 = m[1][2] * m[2][0] - m[1][0] * m[2][2];
    const double c02 = m[1][0] * m[2][1] - m[1][1] * m[2][0];
    const double det = m[0][0] * c00 + m[0][1] * c01 + m[0][2] * c02;
    if (std::abs(det) < 1e-18) return false;
    const double id = 1.0 / det;
    out[0][0] = c00 * id;
    out[0][1] = (m[0][2] * m[2][1] - m[0][1] * m[2][2]) * id;
    out[0][2] = (m[0][1] * m[1][2] - m[0][2] * m[1][1]) * id;
    out[1][0] = c01 * id;
    out[1][1] = (m[0][0] * m[2][2] - m[0][2] * m[2][0]) * id;
    out[1][2] = (m[0][2] * m[1][0] - m[0][0] * m[1][2]) * id;
    out[2][0] = c02 * id;
    out[2][1] = (m[0][1] * m[2][0] - m[0][0] * m[2][1]) * id;
    out[2][2] = (m[0][0] * m[1][1] - m[0][1] * m[1][0]) * id;
    return true;
}

}  // namespace

void RigidBody::EffectiveInertia(double Ie[3][3]) const {
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) Ie[i][j] = mp.I[i][j];
        Ie[i][i] += mp.addedI[i];
    }
}

// Momenta live in the world frame; velocity is read in the body frame. This is the one place the
// two meet, and it is a pure rotation each way -- no moment arms, so no planetary numbers.
void RigidBody::TwistAt(const Motor& at, Bivector& out) const {
    const Motor inv = at.Inverse();
    double Lb[3] = {m_Lw[0], m_Lw[1], m_Lw[2]};
    double Pb[3] = {m_Pw[0], m_Pw[1], m_Pw[2]};
    inv.TransformDir(Lb[0], Lb[1], Lb[2]);
    inv.TransformDir(Pb[0], Pb[1], Pb[2]);

    double Ie[3][3], IeInv[3][3];
    EffectiveInertia(Ie);
    if (!Inv3(Ie, IeInv)) {
        out = Bivector::Zero();
        return;
    }
    Mul3(IeInv, Lb, out.a);
    for (int i = 0; i < 3; ++i) {
        // The effective mass is diagonal in BODY axes, so this division happens here, after the
        // rotation into the body frame -- doing it in world coordinates would apply the surge
        // added mass to whichever way the boat happened to be pointing.
        const double me = mp.mass + mp.addedM[i];
        out.b[i] = Pb[i] / ((me > 1e-12) ? me : 1e-12);
    }
}

void RigidBody::DeriveTwist() {
    double Ie[3][3], IeInv[3][3];
    EffectiveInertia(Ie);
    if (!Inv3(Ie, IeInv)) {
        Log("[rigid] singular inertia tensor -- body held at rest (check the spec)");
        Rest();
        return;
    }
    TwistAt(pose, m_twist);
}

void RigidBody::Carry(const Motor& K) {
    pose = K * pose;
    K.TransformDir(m_Lw[0], m_Lw[1], m_Lw[2]);
    K.TransformDir(m_Pw[0], m_Pw[1], m_Pw[2]);
    DeriveTwist();
}

void RigidBody::SetTwist(const Bivector& t) {
    double Ie[3][3];
    EffectiveInertia(Ie);
    double Lb[3], Pb[3];
    Mul3(Ie, t.a, Lb);
    for (int i = 0; i < 3; ++i) Pb[i] = (mp.mass + mp.addedM[i]) * t.b[i];
    pose.TransformDir(Lb[0], Lb[1], Lb[2]);
    pose.TransformDir(Pb[0], Pb[1], Pb[2]);
    for (int i = 0; i < 3; ++i) { m_Lw[i] = Lb[i]; m_Pw[i] = Pb[i]; }
    DeriveTwist();
}

void RigidBody::Rest() {
    for (int i = 0; i < 3; ++i) { m_Lw[i] = 0.0; m_Pw[i] = 0.0; }
    m_twist = Bivector::Zero();
}

void RigidBody::Step(const Bivector& bodyWrench, double dt) {
    // ---- 1. the wrench, body -> world. A force and a torque are both directions here (the
    // moment arm was already taken, about the CG, when the element built the wrench), so this
    // is the rotor half of the pose and nothing else.
    double fw[3] = {bodyWrench.a[0], bodyWrench.a[1], bodyWrench.a[2]};
    double tw[3] = {bodyWrench.b[0], bodyWrench.b[1], bodyWrench.b[2]};
    pose.TransformDir(fw[0], fw[1], fw[2]);
    pose.TransformDir(tw[0], tw[1], tw[2]);

    // ---- 2. THE INTEGRATION, and the whole reason the state is momentum. Newton and Euler in
    // the world frame are simply dP/dt = f and dL/dt = tau -- no gyroscopic term to get wrong,
    // because there is no rotating frame here. A torque-free body adds zero and its angular
    // momentum is conserved to the last bit rather than to a tolerance.
    for (int i = 0; i < 3; ++i) {
        m_Pw[i] += fw[i] * dt;
        m_Lw[i] += tw[i] * dt;
    }

    // ---- 3. the body twist follows from the momenta and the CURRENT attitude. (The tumbling
    // an asymmetric body does in its own frame is not integrated at all: it falls out of
    // re-reading a constant world momentum through a rotating body.)
    DeriveTwist();

    // ---- 4. THE MOVE, BY THE MIDPOINT RULE. Taking the whole step on the twist read at the
    // START of it is first order, and for a tumbling body that error goes straight into the
    // kinetic energy: with angular momentum pinned exactly, T depends only on which way the
    // hull is FACING, so an attitude error IS an energy error. Measured that way: 18% in a
    // minute. Reading the twist again at the half-step attitude and taking the full step on
    // THAT is second order and costs one extra rotor read.
    //
    // The oscillator is untouched by this: a pure translation does not change the attitude, so
    // the midpoint twist equals the start twist and the velocity-before-position ordering that
    // makes the move symplectic still holds exactly.
    Bivector mid;
    TwistAt(pose * IntegrateStep(m_twist, 0.5 * dt), mid);
    pose = pose * IntegrateStep(mid, dt);
    pose.Normalize();   // 240 Hz forever: the drift is not hypothetical
    DeriveTwist();      // the attitude moved, so the body-frame reading of the momenta moved
}

bool RigidBody::Sane() {
    auto bad = [](double x) { return !(x > -1e12 && x < 1e12); };   // catches NaN too
    for (int i = 0; i < 3; ++i) {
        if (bad(m_Lw[i]) || bad(m_Pw[i])) {
            Log("[rigid] state left the domain (P = %g %g %g, L = %g %g %g) -- reset to rest",
                m_Pw[0], m_Pw[1], m_Pw[2], m_Lw[0], m_Lw[1], m_Lw[2]);
            Rest();
            return false;
        }
    }
    if (bad(pose.s) || bad(pose.t01) || bad(pose.t02) || bad(pose.t03)) {
        Log("[rigid] pose left the domain -- reset to identity");
        pose = Motor::Identity();
        Rest();
        return false;
    }
    return true;
}

double RigidBody::KineticEnergy() const {
    double Ie[3][3], Iw[3];
    EffectiveInertia(Ie);
    Mul3(Ie, m_twist.a, Iw);
    double e = 0.0;
    for (int i = 0; i < 3; ++i) {
        e += 0.5 * (mp.mass + mp.addedM[i]) * m_twist.b[i] * m_twist.b[i];
        e += 0.5 * m_twist.a[i] * Iw[i];
    }
    return e;
}

// ================================================================================================
//  The gate.
// ================================================================================================
bool RunRigidBodySelfTest() {
    bool ok = true;
    auto fail = [&](const char* what, double got, double want) {
        Log("[rigid] FAIL %s: %.12g vs %.12g", what, got, want);
        ok = false;
    };
    auto spin = [](double x, double y, double z) {
        Bivector t;
        t.a[0] = x; t.a[1] = y; t.a[2] = z;
        return t;
    };
    auto vel = [](double x, double y, double z) {
        Bivector t;
        t.b[0] = x; t.b[1] = y; t.b[2] = z;
        return t;
    };

    // ---- 1. PARALLEL AXIS. A box's inertia about a corner, transported to the centre, must be
    // the box formula. Specs quote inertia about whatever point was convenient, so this is the
    // conversion every vessel spec will lean on.
    {
        const MassProps ref = MassProps::Box(1000.0, 2.0, 1.0, 5.0);
        MassProps p = ref;
        const double d[3] = {1.0, 0.5, 2.5};
        const double d2 = d[0]*d[0] + d[1]*d[1] + d[2]*d[2];
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                p.I[i][j] += p.mass * ((i == j ? d2 : 0.0) - d[i]*d[j]);
            }
        }
        p.ShiftToCg(d);
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                if (std::abs(p.I[i][j] - ref.I[i][j]) > 1e-9) {
                    fail("parallel axis", p.I[i][j], ref.I[i][j]);
                }
            }
        }
    }

    // ---- 2. FREE TRANSLATION IS EXACT. No wrench, no rotation: the CG travels v*t on the nose.
    // If the body-frame pose update were composed on the wrong side this drifts immediately.
    {
        RigidBody b;
        b.mp = MassProps::Box(500.0, 2, 1, 5);
        b.pose = Motor::Translation(10.0, -3.0, 7.0);
        b.SetTwist(vel(1.5, 0.0, -2.5));
        const double dt = 1.0 / 240.0;
        const int n = 2400;                                // 10 s
        for (int i = 0; i < n; ++i) b.Step(Bivector::Zero(), dt);
        double x = 0, y = 0, z = 0;
        b.pose.TransformPoint(x, y, z);
        const double t = n * dt;
        if (std::abs(x - (10.0 + 1.5 * t)) > 1e-9) fail("free translation x", x, 10.0 + 1.5 * t);
        if (std::abs(z - (7.0 - 2.5 * t)) > 1e-9) fail("free translation z", z, 7.0 - 2.5 * t);
    }

    // ---- 3. TORQUE-FREE ASYMMETRIC BODY: L_world is EXACT, T is a constant of the motion.
    // An asymmetric body's omega swings around in the body frame -- it is supposed to -- while
    // world angular momentum does not move at all. Written as velocity this drifted 2.6% in a
    // minute; written as momentum the conservation is structural and only the energy, which
    // depends on the ATTITUDE, carries any integration error at all.
    {
        RigidBody b;
        b.mp = MassProps::Box(900.0, 2.3, 1.1, 5.5);       // three distinct principal moments
        b.pose = Motor::Identity();
        b.SetTwist(spin(0.7, 1.9, 0.4));
        double L0[3];
        b.AngularMomentumWorld(L0);
        const double T0 = b.KineticEnergy();
        const double L0m = std::sqrt(L0[0]*L0[0] + L0[1]*L0[1] + L0[2]*L0[2]);
        const double dt = 1.0 / 240.0;
        for (int i = 0; i < 240 * 60; ++i) b.Step(Bivector::Zero(), dt);   // one minute
        double L1[3];
        b.AngularMomentumWorld(L1);
        const double T1 = b.KineticEnergy();
        double dL = 0.0;
        for (int i = 0; i < 3; ++i) dL = (std::max)(dL, std::abs(L1[i] - L0[i]));
        const double relL = dL / L0m, relT = std::abs(T1 - T0) / T0;
        Log("[rigid] torque-free asymmetric, 14400 steps: |dL|/|L| = %.3e, dT/T = %.3e",
            relL, relT);
        if (relL > 1.0e-14) fail("angular momentum is not exactly conserved", relL, 1.0e-14);
        // 5e-3 was the placeholder before the midpoint rule landed. MEASURED at 4.0e-6;
        // held at 1e-4 so a regression to a first-order attitude update (which measured
        // 1.8e-1 here) fails loudly instead of passing inside a generous bound.
        if (relT > 1.0e-4) fail("kinetic energy drift", relT, 1.0e-4);
    }

    // ---- 3b. AND IT REALLY IS TUMBLING. The gate above would also pass for a body that simply
    // never rotated, so demand that omega in the BODY frame actually swings: an asymmetric body
    // spun about all three axes must show a large excursion. This is the discriminator.
    {
        RigidBody b;
        b.mp = MassProps::Box(900.0, 2.3, 1.1, 5.5);
        b.pose = Motor::Identity();
        b.SetTwist(spin(0.7, 1.9, 0.4));
        double swing = 0.0;
        const double dt = 1.0 / 240.0;
        for (int i = 0; i < 240 * 20; ++i) {
            b.Step(Bivector::Zero(), dt);
            swing = (std::max)(swing, std::abs(b.Twist().a[0] - 0.7));
        }
        if (swing < 0.2) fail("asymmetric body is not tumbling (gate would be vacuous)",
                              swing, 0.2);
    }

    // ---- 4. THE MOMENT ARM, END TO END. A world force applied off the CG must produce exactly
    // the angular acceleration the arm implies. This exercises the whole chain -- BodyWrench
    // rotates the force in, ForceAt takes the arm, Euler turns it into alpha -- and it is the
    // path every thruster and every buoyancy panel will take.
    {
        RigidBody b;
        b.mp = MassProps::Box(1000.0, 2.0, 1.0, 5.0);
        b.pose = Motor::Identity();                        // body == world, so the arm is plain
        const double f[3] = {0.0, 0.0, 400.0};             // +z (north) force ...
        const double at[3] = {1.5, 0.0, 0.0};              // ... applied 1.5 m to starboard
        const double dt = 1e-4;
        b.Step(b.BodyWrench(f, at), dt);
        // tau = r x f = (1.5,0,0) x (0,0,400) = (0, -600, 0)
        const double expectAlphaY = -600.0 / b.mp.I[1][1];
        const double gotAlphaY = b.Twist().a[1] / dt;
        if (std::abs(gotAlphaY - expectAlphaY) > 1e-6 * std::abs(expectAlphaY)) {
            fail("moment arm -> yaw acceleration", gotAlphaY, expectAlphaY);
        }
        const double expectAz = 400.0 / b.mp.mass;         // and the force still accelerates it
        if (std::abs(b.Twist().b[2] / dt - expectAz) > 1e-9) {
            fail("off-CG force still accelerates", b.Twist().b[2] / dt, expectAz);
        }
    }

    // ---- 4b. THE SAME ARM, ROTATED. With the hull yawed 90 degrees the SAME world force must
    // give the same BODY-frame answer -- which is what says BodyWrench rotates the force in
    // rather than assuming body and world agree. Passing 4 alone would not say that.
    {
        RigidBody b;
        b.mp = MassProps::Box(1000.0, 2.0, 1.0, 5.0);
        const double org[3] = {0, 0, 0}, up[3] = {0, 1, 0};
        b.pose = Motor::Rotation(org, up, 1.5707963267948966);   // +x -> -z
        const double fWorld[3] = {0.0, 0.0, 400.0};
        const double atBody[3] = {1.5, 0.0, 0.0};
        const double dt = 1e-4;
        b.Step(b.BodyWrench(fWorld, atBody), dt);
        // In the body frame the world +z force reads as +x, so tau = (1.5,0,0) x (400,0,0) = 0:
        // no yaw at all, and pure surge. A BodyWrench that forgot to rotate would yaw here.
        if (std::abs(b.Twist().a[1] / dt) > 1e-9) {
            fail("rotated hull: force along the arm makes no torque", b.Twist().a[1] / dt, 0.0);
        }
        // ...and the surge is NEGATIVE: Pga.h pins a right-handed quarter turn about +y as
        // body +x -> world -z, so the inverse reads a world +z force as body -x. Expecting
        // +0.4 here was this gate's own first bug -- the code was right and the test was not.
        if (std::abs(b.Twist().b[0] / dt + 400.0 / b.mp.mass) > 1e-9) {
            fail("rotated hull: surge", b.Twist().b[0] / dt, -400.0 / b.mp.mass);
        }
    }

    // ---- 5. THE OSCILLATOR, AND WHY THE MOVE IS SYMPLECTIC. A linear spring gives a known
    // period; run it for 200 of them and demand the AMPLITUDE has not grown. Explicit Euler on
    // this problem gains energy every single step -- a boat left bobbing overnight would climb
    // out of the sea -- so an amplitude bound over many periods is the discriminating test, not
    // a period check on its own.
    {
        const double m = 800.0, k = 5000.0;                // omega = 2.5 rad/s, T = 2.513 s
        const double period = 2.0 * 3.14159265358979323846 * std::sqrt(m / k);
        RigidBody b;
        b.mp = MassProps::Box(m, 2, 1, 5);
        b.pose = Motor::Translation(0.0, 1.0, 0.0);        // pulled 1 m up, released
        const double dt = 1.0 / 240.0;
        const int n = static_cast<int>(200.0 * period / dt);
        double maxY = 0.0, prevY = 1.0;
        int crossings = 0;
        double firstCross = -1.0, lastCross = -1.0;
        const double org[3] = {0, 0, 0};
        for (int i = 0; i < n; ++i) {
            double x = 0, y = 0, z = 0;
            b.pose.TransformPoint(x, y, z);
            const double fy[3] = {0.0, -k * y, 0.0};       // through the CG: no torque
            b.Step(b.BodyWrench(fy, org), dt);
            double x2 = 0, y2 = 0, z2 = 0;
            b.pose.TransformPoint(x2, y2, z2);
            if (i > 10) maxY = (std::max)(maxY, std::abs(y2));
            if (prevY < 0.0 && y2 >= 0.0) {
                if (firstCross < 0.0) firstCross = i * dt;
                lastCross = i * dt;
                ++crossings;
            }
            prevY = y2;
        }
        const double measured =
            (crossings > 1) ? (lastCross - firstCross) / (crossings - 1) : 0.0;
        const double relT = std::abs(measured - period) / period;
        const double growth = maxY - 1.0;
        Log("[rigid] oscillator: period %.6f s (analytic %.6f, rel %.2e), amplitude growth "
            "over 200 periods %.3e m", measured, period, relT, growth);
        if (relT > 2.0e-3) fail("oscillator period", measured, period);
        if (growth > 1.0e-3) fail("oscillator amplitude growth (move is not symplectic)",
                                  growth, 1.0e-3);
    }

    // ---- 6. ADDED MASS IS A MASS. Same force, added mass equal to the dry mass, half the
    // acceleration -- on the AXIS it was declared for, and not on the others. An added-mass
    // tensor applied isotropically by mistake would pass the first half and fail the second.
    {
        RigidBody b;
        b.mp = MassProps::Box(1000.0, 2, 1, 5);
        b.mp.addedM[1] = 1000.0;                           // heave only
        b.pose = Motor::Identity();
        const double dt = 1e-4, org[3] = {0, 0, 0};
        const double fy[3] = {0.0, 2000.0, 0.0};
        b.Step(b.BodyWrench(fy, org), dt);
        if (std::abs(b.Twist().b[1] / dt - 1.0) > 1e-9) {
            fail("added mass halves heave accel", b.Twist().b[1] / dt, 1.0);
        }
        RigidBody c;
        c.mp = b.mp;
        c.pose = Motor::Identity();
        const double fx[3] = {2000.0, 0.0, 0.0};
        c.Step(c.BodyWrench(fx, org), dt);
        if (std::abs(c.Twist().b[0] / dt - 2.0) > 1e-9) {
            fail("added mass does NOT affect surge", c.Twist().b[0] / dt, 2.0);
        }
    }

    // ---- 7. THE GUARD REPORTS. A wrench that is not a number must leave the body at rest and
    // say so, not propagate a NaN into the pose where it would silently delete the boat.
    {
        RigidBody b;
        b.mp = MassProps::Box(100.0, 1, 1, 1);
        Bivector w;
        w.a[0] = std::sqrt(-1.0);                          // NaN force
        b.Step(w, 1.0 / 240.0);
        if (b.Sane()) fail("Sane() missed a NaN", 1.0, 0.0);
        double p[3];
        b.LinearMomentumWorld(p);
        if (p[0] != 0.0) fail("Sane() did not reset", p[0], 0.0);
    }

    Log("[rigid] ---- %s: parallel axis, exact free translation, torque-free L exact + real "
        "tumbling, moment arm -> alpha (upright and yawed), symplectic oscillator over 200 "
        "periods, anisotropic added mass, NaN guard ----",
        ok ? "PASS" : "FAIL");
    return ok;
}

}  // namespace ga
