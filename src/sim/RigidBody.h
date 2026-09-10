// ================================================================================================
//  RigidBody - M9bq: THE FIRST THING IN THIS ENGINE WITH MOMENTUM.
//
//  Everything else here is f(simUnix). That is why time-scrub scrubs the traffic, why headless
//  rails are byte-reproducible, and why the whole engine could be restarted at any instant and
//  agree with itself. A boat breaks that on purpose: it is a state that remembers.
//
//  Forces arrive as a WRENCH and velocity is read as a TWIST -- both the same six-double
//  Bivector, so there is no separate force and torque to keep in step and a thrust applied at a
//  transom mount cannot lose its moment arm on the way in. See core/Pga.h for the convention and
//  the gate that shows it discriminates.
//
//  FOUR DECISIONS. Each was taken the other way first, and the gate caught it.
//
//  1. THE INTEGRATOR'S STATE IS MOMENTUM IN THE WORLD FRAME (Pw, Lw), not velocity.
//     A torque-free body must conserve angular momentum EXACTLY, and a free body must conserve
//     linear momentum exactly. Written as velocities they are conserved only as well as the
//     integrator is accurate -- measured here at 2.6% angular-momentum and 6.8% energy drift in
//     ONE MINUTE of tumbling, which over an afternoon is not a boat. Written as momenta, `no
//     torque` means `Lw += 0` and conservation stops being a numerical achievement: it is the
//     representation. This is the same move the fold and the telescope make elsewhere in this
//     engine -- put the invariant in the form, not in the tolerance.
//
//  2. VELOCITY IS READ IN THE BODY FRAME. A world-frame twist stores `v at the world origin`,
//     and this engine's world origin can be the centre of the planet -- so v_O would be
//     omega x r with r ~ 6.4e6 m, a number ~1e7 times larger than the velocity it describes,
//     differenced back down every time anyone asked how fast the boat was going. Momenta have
//     no moment arm and are safe in world coordinates; a twist is not, so the twist is body
//     frame and is DERIVED from the momenta each step.
//
//  3. THE WRENCH IS IN THE BODY FRAME TOO, for exactly the same reason: a torque about the
//     WORLD origin is r x f with a planetary r, so the hull-scale moment that actually matters
//     would be the difference of two enormous numbers. Elements are mounted by body-frame
//     motors and know their attach points in metres, so BodyWrench() below is the only bridge
//     any of them needs.
//
//  4. THE MOVE IS SEMI-IMPLICIT (SYMPLECTIC): derive the new velocity, then move on it.
//     Explicit Euler pumps energy into an oscillator without bound, and a hull in waves is
//     nothing but oscillators -- a boat left floating overnight would climb out of the water.
//     Measured over 200 periods: amplitude growth 1.4e-5 m on a 1 m swing.
//
//  Stepped once per SimClock quantum (1/240 s) from the Advance() loop -- never on a wall-clock
//  dt, or the boat handles differently at 60 fps and at 140. See sim/SimClock.h.
// ================================================================================================
#pragma once

#include "core/Common.h"
#include "core/Pga.h"

#include <cmath>

namespace ga {

// ================================================================================================
//  MassProps - what a body weighs and how it resists being turned.
//
//  ADDED MASS is carried here too, and it is not an optional refinement for a boat. A hull
//  accelerating in water drags a volume of water with it; for a small planing hull the added
//  mass in heave is a large fraction of the displacement, and in sway larger still. Omit it and
//  the boat is far too lively -- it snaps to wave slopes like a cork instead of settling in.
//
//  It is stored DIAGONAL, in the body frame, and that is a stated approximation rather than an
//  oversight: the full potential-flow added-mass tensor is 6x6 with off-diagonal coupling, and
//  nothing in this engine could supply those coefficients honestly for a RHIB. A diagonal
//  tensor is one law applied to six axes; the couplings are declared absent, not silently zero.
// ================================================================================================
struct MassProps {
    double mass = 1.0;             // kg, dry + loaded
    double I[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};   // kg m^2, body frame, ABOUT THE CG

    // Diagonal added mass/inertia, body frame: surge, sway, heave, then roll, pitch, yaw.
    double addedM[3] = {0, 0, 0};      // kg
    double addedI[3] = {0, 0, 0};      // kg m^2

    static MassProps Box(double mass, double sx, double sy, double sz) {
        MassProps p;
        p.mass = mass;
        const double k = mass / 12.0;
        p.I[0][0] = k * (sy * sy + sz * sz);
        p.I[1][1] = k * (sx * sx + sz * sz);
        p.I[2][2] = k * (sx * sx + sy * sy);
        p.I[0][1] = p.I[0][2] = p.I[1][0] = p.I[1][2] = p.I[2][0] = p.I[2][1] = 0.0;
        return p;
    }

    // Parallel-axis transport: an inertia measured about a point `d` away from the CG, moved to
    // the CG. Specs quote inertia about whatever point was convenient; this is where that gets
    // fixed once instead of at every call site.
    void ShiftToCg(const double d[3]) {
        const double d2 = d[0] * d[0] + d[1] * d[1] + d[2] * d[2];
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                I[i][j] -= mass * ((i == j ? d2 : 0.0) - d[i] * d[j]);
            }
        }
    }
};

// ================================================================================================
//  The body. The body origin IS the centre of mass, by construction and not by convention: put
//  it anywhere else and the linear and angular equations couple through a term everyone forgets,
//  which shows up as a hull that yaws when it heaves. Vessel places every element relative to
//  the CG; MassProps::ShiftToCg re-expresses an inertia measured about some other point.
// ================================================================================================
class RigidBody {
public:
    Motor pose;          // body -> world. Its translation is the CG's world position.
    MassProps mp;

    // ---- state ---------------------------------------------------------------------------
    // The twist is BODY frame: a = angular velocity (rad/s), b = velocity of the CG (m/s).
    // Read-only, because it is derived from the momenta -- writing it directly would leave the
    // two descriptions disagreeing, which is exactly the class of bug this engine keeps
    // catching in its own compositor.
    const Bivector& Twist() const { return m_twist; }
    void SetTwist(const Bivector& t);            // sets the momenta to match, then re-derives
    void SetPose(const Motor& m) { pose = m; Rest(); }
    // Rest, at the current pose. The clock policy calls this when the scene time jumps: a boat
    // cannot be integrated across a scrub, and pretending otherwise would launch it.
    void Rest();

    // ---- frame helpers -------------------------------------------------------------------
    void ToWorld(const double bodyPt[3], double out[3]) const {
        out[0] = bodyPt[0]; out[1] = bodyPt[1]; out[2] = bodyPt[2];
        pose.TransformPoint(out[0], out[1], out[2]);
    }
    // The world velocity of the material point currently at body-frame `bodyPt`. One call,
    // because the twist carries both halves -- this is the hull panel sampler's whole question.
    void VelocityAtBody(const double bodyPt[3], double out[3]) const {
        m_twist.VelocityAt(bodyPt, out);                // body-frame velocity of that point
        pose.TransformDir(out[0], out[1], out[2]);      // a velocity is a direction
    }

    // THE BRIDGE EVERY ELEMENT USES: the body-frame wrench of a force computed in WORLD
    // coordinates (which is how the water and the wind hand it over) applied at a point known
    // in BODY coordinates (which is how the spec mounts it). No planetary number is touched.
    Bivector BodyWrench(const double fWorld[3], const double pBody[3]) const {
        double fb[3] = {fWorld[0], fWorld[1], fWorld[2]};
        pose.Inverse().TransformDir(fb[0], fb[1], fb[2]);
        return Bivector::ForceAt(fb, pBody);
    }

    // ---- the step ------------------------------------------------------------------------
    // `bodyWrench`: a = net force in BODY coordinates (N), b = net torque about the CG in BODY
    // coordinates (N m). Build it by summing BodyWrench(...) over the elements. Gravity is NOT
    // added here -- it is a force like any other and the caller owns whether it applies.
    void Step(const Bivector& bodyWrench, double dt);

    // NaN / blow-up guard. Not physics -- diagnostics. Returns false and leaves the body at rest
    // if the state has left the domain, so a bad wrench reports itself instead of quietly
    // teleporting the boat to infinity three frames later.
    bool Sane();

    double KineticEnergy() const;
    // The stored world angular momentum. For a torque-free body this is constant to the last
    // bit, which is the gate that catches a wrong gyroscopic term or a wrong frame.
    void AngularMomentumWorld(double out[3]) const {
        out[0] = m_Lw[0]; out[1] = m_Lw[1]; out[2] = m_Lw[2];
    }
    void LinearMomentumWorld(double out[3]) const {
        out[0] = m_Pw[0]; out[1] = m_Pw[1]; out[2] = m_Pw[2];
    }

private:
    void EffectiveInertia(double Ie[3][3]) const;   // I + diag(addedI)
    void DeriveTwist();                             // momenta (world) -> twist (body)
    // The same reading, at an ARBITRARY attitude -- the midpoint rule needs the twist the body
    // WILL have half a step from now, which is a different reading of the same momenta.
    void TwistAt(const Motor& at, Bivector& out) const;

    double m_Lw[3] = {0, 0, 0};    // world angular momentum about the CG, kg m^2/s
    double m_Pw[3] = {0, 0, 0};    // world linear momentum, kg m/s
    Bivector m_twist;              // derived, body frame
};

// Numeric gate. Pure CPU; runs inside --selftest.
bool RunRigidBodySelfTest();

}  // namespace ga
