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
    bool   aground = false;
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

    VesselSpec m_spec;
    RigidBody  m_body;
    VesselTelemetry m_tel;
    // Accumulated across one wrench evaluation, so telemetry does not need a second pass.
    double m_accVol = 0.0;
    double m_accDraught = 0.0;
    int    m_accStations = 0;
};

// ---- the section clip, exposed because it has a closed-form gate of its own ------------------
// Clips a closed polygon to the half-plane a*x + b*y + c <= 0 and returns the area and centroid
// of what remains. Returns 0 area when nothing is inside. `n` points in, arbitrary winding.
double ClipSectionArea(const double* px, const double* py, int n, double a, double b, double c,
                       double* cxOut, double* cyOut);

bool RunVesselSelfTest();

}  // namespace ga
