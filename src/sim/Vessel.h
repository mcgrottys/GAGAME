// ================================================================================================
//  Vessel - M9bq: THE ONE RUNTIME TYPE. It consumes any VesselSpec.
//
//  There is no Rhib class. Adding a hull is adding a spec (VesselSpec.h); adding a NEW KIND OF
//  PHYSICS is adding an element kind here, and there are six, and they already span outboards,
//  inboards, rudders, keels, sails and windage.
//
//  ---- THE ONE PHYSICS DECISION WORTH READING BEFORE THE CODE.
//
//  Buoyancy acts along the LOCAL SURFACE NORMAL, not along world up. That looks like a liberty
//  and it is the opposite: it is the first-order-correct law, and taking the "obvious" one would
//  have been the approximation.
//
//  Archimedes gives rho g V straight up only in a HYDROSTATIC pressure field. Under a wave the
//  field is not hydrostatic -- there is a dynamic term, and its gradient is what actually pushes
//  a boat down a wave face. The clean way to see the answer: the free surface of a wave is an
//  equipotential of the effective gravity (that is WHY it has the shape it has), so the effective
//  gravity is normal to it, and buoyancy opposes the effective gravity. On flat water the normal
//  IS up and nothing changes; on a wave face the same expression tilts and the hull accelerates
//  down-slope.
//
//  So surfing is not a feature that was implemented. It is what this law does on a sloped
//  surface, and there is no branch anywhere that mentions waves. Same for a long swell: the
//  hull rides it because SurfaceSample carries a plane and a particle velocity instead of a
//  height. That is the whole of "leave it flexible for rolling waves".
//
//  ---- IMMERSION IS A POLYGON CLIP, NOT A DEPTH LOOKUP.
//
//  Each station's section is clipped against the water plane and the exact area and centroid of
//  what remains is used. That is exact for a box, exact for a deep-V, correct under heel (which
//  a depth-based sectional-area curve is NOT -- it would give no righting moment at all), and
//  needs no case for "fully submerged": the clip returns the whole polygon and the law does not
//  notice it happened.
// ================================================================================================
#pragma once

#include <atomic>
#include "core/Common.h"
#include "sim/Medium.h"
#include "sim/RigidBody.h"
#include "sim/VesselSpec.h"
#include "sim/WaterSurface.h"

#include <vector>

namespace ga {

// What the helm is asking for. Filled from the keyboard/pad; see the control mode in main.
struct VesselControls {
    static constexpr int kMaxThrusters = 4;
    double throttle[kMaxThrusters] = {};   // -1 astern .. +1 ahead, PER THRUSTER (the twin
                                           // levers, and why a pivot is a squeeze not a mode)
    double steer = 0.0;                    // commanded steering angle, rad (the outboard, never
                                           // the hull heading)
    double tilt = 0.0;                     // commanded trim/tilt, rad
    bool   running[kMaxThrusters] = {true, true, true, true};
};

// One frame's worth of what the hull is doing, for the HUD and for gates.
struct VesselTelemetry {
    double speedKn = 0.0;
    double headingRad = 0.0;
    double heelRad = 0.0, trimRad = 0.0;
    double draughtM = 0.0;          // mean immersion at the stations
    double immersedVol = 0.0;       // m^3
    // Split, because it answers the question the owner asked: are the TUBES still in the water
    // at planing trim? All of this hull's roll damping and most of its roll stiffness live in
    // the collar, so a collar that lifts clear at speed is a boat with nothing damping its roll
    // -- which is exactly the symptom.
    double hullVol = 0.0, collarVol = 0.0;
    double wettedLambda = 0.0;      // Savitsky's mean wetted-length / beam -- the planing state
    double copZ = 0.0;              // where the planing pressure acts, body z
    double depthM = 0.0;            // water depth under the hull
    bool   aground = false;         // a LABEL (depth under the CG < 0.3 m); the force is below
    // THE GROUND'S ANSWER (beaching): the bed's normal force summed over the station outline
    // points it is pushing on, and how many of them. Zero and zero is a hull the bed does not
    // touch; the weight on the bed is what buoyancy is no longer carrying.
    double contactN = 0.0;          // N
    int    contactPts = 0;
    bool   waterValid = false;      // false = no coverage; NOT the same as flat water
    int    thrustersWet = 0;
};

class Vessel {
public:
    // Build from a spec. False (and a log line) if the spec is empty or uses an element kind
    // this build has not implemented yet -- never a silent partial hull, because a hull missing
    // its buoyancy would simply sink and look like a physics bug.
    bool Build(const VesselSpec& spec, const Motor& pose);

    void Step(const WaterSurface& sea, const VesselControls& c, double simUnix, double dt);
    // F22: THE STEP ASKS ITS WATER ONCE. The points the force loops will ask (the stations, the
    // collar's slices, the mounts, the CG -- by the same ToWorld of the same body points, so the
    // doubles are the loops' own) are listed, answered by one WaterSurface::AtMany, and the loops
    // read the table; a point the table lacks (the planing panels, whose place is the step's own
    // state) is asked live and counted. Off only in the selftest, which holds the two bit for bit.
    static bool s_batchWater;
    static std::atomic<uint64_t> s_batchSteps, s_batchPoints, s_batchMisses, s_batchHits;
    // ...and the audit (the residency audit's flag): every table entry re-asked live, compared bit
    // for bit; a difference is the batch answering other than the point would.
    static bool s_batchAudit;
    static bool s_gripTrace;   // a test's per-point trace of the ground's grip
    static std::atomic<uint64_t> s_auditChecks, s_auditFails;

    const RigidBody& Body() const { return m_body; }
    RigidBody& Body() { return m_body; }
    const VesselSpec& Spec() const { return m_spec; }
    const VesselTelemetry& Telemetry() const { return m_tel; }

    // Gravity is applied inside Step, but exposed so a gate can hold a hull in a vice.
    bool applyGravity = true;

    // The total body-frame wrench at the current state, without stepping. This is what the
    // gates read: a floating body at equilibrium has a wrench of zero, and that is checkable
    // to many more digits than "watch it settle" ever could be.
    Bivector NetWrench(const WaterSurface& sea, const VesselControls& c, double simUnix);

private:
    void WaterPoints(const VesselControls& c, std::vector<double>& xz) const;   // F22
    // THE BED PLANE (beaching, 2026-10-11): one plane per hull per wrench, its normal the secant
    // over the hull -- the bed's height at the CG and at CG + (h, 0) and CG + (0, h), h half the
    // length overall, through the same WaterSurface::At the stations ask (so the F22 table serves
    // it). Every contact point then takes its OWN station's bed height and this normal. The
    // three world points, in the order WaterPoints lists them (the same doubles: a table hit).
    void BedProbePoints(double xz[6]) const;
    void BedPlane(const WaterSurface& sea, double simUnix);
    Bivector Buoyancy(const Element& e, const WaterSurface& sea, double simUnix);
    Bivector Collar(const Element& e, const WaterSurface& sea, double simUnix);
    Bivector Planing(const Element& e, const WaterSurface& sea, double simUnix);
    // One half of the running surface, offset across the beam. Roll stiffness and roll damping
    // both come from evaluating this at two places rather than one -- see the note in the .cpp.
    // The aftmost station, in body coordinates -- the transom the wetted length is measured
    // forward from. Found once at Build, after the CG solve has moved the stations.
    double m_transomZ = 0.0;

    // THE WETTED LENGTH, AS A STATE. Savitsky's lambda -- mean wetted length over beam -- is the
    // variable everything else keys on, and the reason it must be integrated rather than
    // evaluated is that the spray root does not teleport. Taken as an instantaneous function of
    // trim it swung the centre of pressure between the transom and mid-length every step and
    // threw the hull; lagged, it is the negative feedback that sets the running trim.
    // The spec's added mass at FULL immersion. What the body carries is this scaled by how much
    // of the hull is actually in the water -- see the note in Step.
    double m_addedM0[3] = {0, 0, 0};
    double m_addedI0[3] = {0, 0, 0};
    double m_dispVol = 1.0;        // static displaced volume, the scale for "fully immersed"

    double m_lambda = 1.5;
    double m_dt = 1.0 / 60.0;   // set by Step, read by the lambda relaxation

    Bivector Foil(const Element& e, const WaterSurface& sea, const VesselControls& c,
                  double simUnix);
    Bivector Drag(const Element& e, const WaterSurface& sea, double simUnix);
    Bivector Hover(const Element& e, const VesselControls& c);
    Bivector Wheel(const Element& e, const WaterSurface& sea, const VesselControls& c, double simUnix);

    VesselSpec m_spec;
    RigidBody  m_body;
    VesselTelemetry m_tel;
    // Accumulated across one wrench evaluation, so telemetry does not need a second pass.
    double m_accVol = 0.0;
    // The immersed fraction, established by the first pass of NetWrench and read by the second.
    // One number, two customers: the added mass (entrained water there is no longer any of) and
    // the axial resistance (a hull that is not in the water does not make any).
    double m_wetFrac = 1.0;
    double m_accDraught = 0.0;
    int    m_accStations = 0;

    // ---- the bed (beaching): the plane's normal in the hull's frame, this wrench; the contact
    // stiffness and damping PER KEEL POINT (Build: the hull's weight over `contactSettle` on the
    // points at each station's lowest y -- one for a V, three for a flat bottom); the friction.
    double m_bedN[3] = {0.0, 1.0, 0.0};
    double m_kContact = 0.0;       // N/m per point
    double m_cContact = 0.0;       // N s/m per point: critical against the mass share
    double m_mContact = 1.0;       // kg: the mass share of one point
    double m_muContact = 0.0;
    // THE GRIP IS A POSITION, NOT A VELOCITY. Coulomb's static friction holds a hull at REST on a
    // slope, and no function of velocity can: at 60 Hz a velocity law removes at most what gravity
    // added that step, so it creeps at ~g sin(theta) dt (measured 2.36 m in 10 s on a 20 deg slope
    // under the smooth sign with eps 0.5 mm/s in its implicit form; the brief's eps of 0.05 m/s
    // creeps 0.36 m; an eps small enough to hold is 55x critical and explicit integration chatters).
    // So each outline point keeps the ground point it GRIPPED: a tangential spring to its anchor
    // (the bed's own rate and damping) saturated at mu N, the anchor dragged to the cone when it
    // saturates. Below the cone the hull stands still to the spring's deflection; above it, it
    // slides at exactly mu N. One expression; the cone is the same min(1, mu N / |F|) a tyre uses.
    // Anchors live in the hull's FRAME (world points), so a carry moves them with the body (Carry).
    std::vector<double> m_anchor;      // 3 per outline point, in Buoyancy/station/outline order
    std::vector<uint8_t> m_anchorOn;   // the point is gripping (its anchor is live)
    size_t m_anchorIdx = 0;            // the running index this wrench
    bool m_groundAsks = false;         // any ground law at all: the bed plane is built, its probes listed

public:
    // CARRIED BY A MOTOR: the body (RigidBody::Carry) and the ground it grips, together.
    void Carry(const Motor& K);

    // THE WHEELS' LIVE STATE (the car, 2026-10-11): one per Wheel element in spec order, written by
    // every wrench -- where the strut put the wheel's centre (BODY frame), its steer, its load and
    // compression -- so the layer draws each wheel where the suspension has it, turned as it is.
    struct WheelState {
        double centre[3] = {0.0, 0.0, 0.0};
        double steer = 0.0;
        double load = 0.0;
        double comp = 0.0;
    };
    const std::vector<WheelState>& Wheels() const { return m_wheels; }
    // The lowest point of the body at rest, body frame: the keel of the stations and the tyres at
    // full droop. The set-down puts THIS on the surface the body rides (Entity::Update).
    double KeelY() const;

private:
    std::vector<WheelState> m_wheels;
    size_t m_wheelIdx = 0;
    // The braked tyre's grip of its ground point, per wheel (Build: its share of the weight over
    // the wheel element's contactSettle), 0.4 of critical as the hull's.
    double m_kGrip = 0.0, m_cGrip = 0.0;
    double m_mWheel = 1.0;   // kg: one wheel's share of the mass (the lateral spring's damping)
};

// ---- the section clip, exposed because it has a closed-form gate of its own ------------------
// Clips a closed polygon to the half-plane a*x + b*y + c <= 0 and returns the area and centroid
// of what remains. Returns 0 area when nothing is inside. `n` points in, arbitrary winding.
double ClipSectionArea(const double* px, const double* py, int n, double a, double b, double c,
                       double* cxOut, double* cyOut, double* exOut = nullptr,
                       double* eyOut = nullptr);

bool RunVesselSelfTest();

}  // namespace ga
