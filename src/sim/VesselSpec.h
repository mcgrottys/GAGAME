// ================================================================================================
//  VesselSpec / VesselRegistry - M9bq: THE FACTORY. A new boat costs a SPEC, not a class.
//
//  This file is deliberately the same shape as core/FieldLoader.h's LoaderRegistry, which is
//  this codebase's factory idiom: a descriptor that IS the identity, a std::function builder
//  registered under a name, composition over inheritance because what varies between products
//  is DATA and not behaviour. GradeField.h states the value outright for datasets --
//
//      "a new dataset costs a loader and a grade declaration, not a bespoke pipeline"
//
//  -- and vessels get the same sentence. There is no Rhib class here and there will be no
//  Sailboat class. One runtime type (Vessel) consumes any spec.
//
//  ---- WHAT ACTUALLY VARIES: ELEMENTS, NOT VESSELS.
//
//  A hull is an assembly of force-producing elements, each mounted by a MOTOR and each returning
//  a WRENCH. Six kinds span everything on the list:
//
//      Buoyancy   immersed section area along a station table    any hull shape
//      Collar     a pressurised tube, per chamber                RHIB tubes, fenders
//      Planing    dynamic lift, centre of pressure moving fwd    any planing hull
//      Thruster   thrust along a line from its mount             outboard, inboard, pod, jet
//      Foil       lift + drag from a section vs relative flow    rudder, skeg, keel, SAIL
//      Drag       quadratic resistance on an area/Cd law         hull drag, WINDAGE
//
//  So the propulsion arrangement is data: an outboard is a steerable, tiltable Thruster mounted
//  at the transom; an inboard is a fixed Thruster low and forward plus a Foil for the rudder;
//  single/twin/triple is one/two/three entries; and a sailboat is zero Thrusters and some Foils
//  whose medium is Air. Counter-rotating props are `rotation: +1, -1` instead of `+1, +1` --
//  one character, because prop walk and torque roll are COMPUTED from that field rather than
//  written into a hull-specific branch.
//
//  ---- THE GA TWIST, AND WHY IT IS NOT DECORATION.
//
//  Every part is mounted by a Motor, and every articulation is a LINE, not an axis-and-pivot
//  pair. Pga.h's header advertises exactly this payoff -- "an offset axis is one primitive, not
//  a composition" -- and a boat is the case that collects on it:
//
//      outboard steering  = rotation about the near-vertical line through the transom mount
//      engine tilt/trim   = rotation about the transverse pin line
//      rudder stock, centreboard pivot, boom gooseneck = the same call, a different line
//
//  Motor::Rotation(p, d, angle) is that, once, for all of them. No translate-rotate-translate
//  bookkeeping and no per-joint moment-arm arithmetic to get wrong.
//
//  ---- BODY FRAME: x STARBOARD, y UP, z FORWARD (the bow).
//
//  Right-handed, and consistent with the engine's world frame (x east, y up, z north) when the
//  hull heads north. Every element position in a spec is in these axes, in metres, from the
//  CENTRE OF MASS -- which is the body origin by construction (see RigidBody.h).
//
//  ---- PROVENANCE IS A FIELD, NOT A COMMENT.
//
//  A hull spec is a table of numbers whose units and sources are exactly what a reader cannot
//  check by eye: a displacement in pounds read as kilograms is a boat that floats 2.2x too high
//  and looks entirely plausible doing it. Num carries the raw value, its unit, the canonical SI
//  conversion (through GaUnits' UnitSpec, so there is no second parser) and where it came from.
//  A number nobody sourced says ASSUMED out loud, on every run, in the spawn report.
// ================================================================================================
#pragma once

#include "core/Common.h"
#include "core/GaUnits.h"
#include "core/Pga.h"

#include <functional>
#include <map>
#include <memory>
#include <string>
#include <vector>

namespace ga {

// ================================================================================================
//  Num - a physical number that knows what it is and where it came from.
//  `v` is ALWAYS canonical SI; `raw`/`unit` are what was written, kept for the report.
// ================================================================================================
struct Num {
    double v = 0.0;               // canonical SI -- what the physics reads
    double raw = 0.0;             // as written in the spec
    std::string unit;             // as written ("lb", "kn", "deg", ...)
    std::string src;              // a source, or "ASSUMED: <reasoning>"
    Quantity quantity = Quantity::Unknown;

    operator double() const { return v; }   // so the physics reads naturally

    static Num Of(double raw, const char* unit, const char* src) {
        Num n;
        n.raw = raw;
        n.unit = unit ? unit : "";
        n.src = src ? src : "";
        const UnitSpec u = UnitSpec::Parse(n.unit.c_str());
        n.quantity = u.quantity;
        n.v = raw * u.toCanonical + u.datumShiftM;
        return n;
    }
    bool Assumed() const { return src.rfind("ASSUMED", 0) == 0; }
};

// ================================================================================================
//  Attach - where a part lives, and the LINE it swings about.
// ================================================================================================
struct Attach {
    Motor at;                                  // the mount, body frame
    bool jointed = false;
    double axisP[3] = {0, 0, 0};               // a point on the joint line
    double axisD[3] = {0, 1, 0};               // its unit direction
    double lo = 0.0, hi = 0.0;                 // travel limits, radians
    double rate = 1.0;                         // max slew, rad/s (mechanical, not a filter)

    // The TRIM/TILT joint: a second line, transverse. Trimming an outboard OUT angles its thrust
    // downward at the transom, which pushes the stern down and lifts the bow -- it is the
    // helmsman's only direct control over running attitude, and on a planing hull that is most
    // of the difference between a boat that skips and one that tracks.
    bool tilted = false;
    double tiltP[3] = {0, 0, 0};
    double tiltD[3] = {1, 0, 0};
    double tiltLo = 0.0, tiltHi = 0.0, tiltRate = 0.35;
    // The tilt channel's RETURN when no key holds it: 0 is a trim pump (it stays where it was
    // left), a brake pedal or a drone's climb lever comes back at this rate. The steer channel
    // always returns at `rate` (the wheel centres). The helm reads these joints: the first
    // jointed element's limits and rate are the steer's, the first tilted element's the tilt's.
    double tiltReturn = 0.0;

    // The joint at angle `q`, composed onto the mount. ONE PGA primitive for every articulation
    // on every hull; the offset axis costs nothing extra.
    // TWO joints, because an outboard has two: it swings about a near-vertical line to steer
    // and about a transverse pin to trim. Both are composed in the mount's LOCAL frame, so
    // `axisP` is measured from the mount itself -- putting the body-frame position there makes
    // the rotation swing the whole leg off the transom instead of turning it in place, which is
    // what it did.
    Motor Posed(double q, double t = 0.0) const {
        Motor m = at;
        if (jointed) {
            const double c = (q < lo) ? lo : ((q > hi) ? hi : q);
            m = m * Motor::Rotation(axisP, axisD, c);
        }
        if (tilted) {
            const double c = (t < tiltLo) ? tiltLo : ((t > tiltHi) ? tiltHi : t);
            m = m * Motor::Rotation(tiltP, tiltD, c);
        }
        return m;
    }
    // The mount's origin in body coordinates -- where a force from this part is applied.
    void Origin(double q, double out[3], double t = 0.0) const {
        out[0] = out[1] = out[2] = 0.0;
        Posed(q, t).TransformPoint(out[0], out[1], out[2]);
    }
    // The mount's forward axis (body +z) in body coordinates -- a thruster's line of action.
    void Forward(double q, double out[3], double t = 0.0) const {
        out[0] = 0.0; out[1] = 0.0; out[2] = 1.0;
        Posed(q, t).TransformDir(out[0], out[1], out[2]);
    }
};

// Ballast is the odd one out and earns its place: it exerts NO force of its own, it only sits
// somewhere and weighs something. Fuel, batteries, crew, a bag of gear. It exists because where a
// boat carries its weight is a real design fact -- tanks forward instead of aft is worth degrees
// of running trim -- and because the alternative is a single hand-tuned "the CG is here" number
// that stops being true the moment anything is added.
// Hover (2026-10-11) is the recovery drone's one element: a velocity servo at the CG and an
// attitude servo about it, in no medium at all -- it flies the same over land and sea because it
// asks the water nothing.
// Wheel (2026-10-11) is the car's: a strut cast down onto the SAME bed the hull beaches on, a
// progressive spring and damper along it, and a tyre at its foot -- drive, rolling resistance, the
// brake's grip and the slip-angle lateral law through one friction cone. Four of them are a car.
enum class ElementKind : uint8_t { Buoyancy, Collar, Planing, Thruster, Foil, Drag, Ballast, Hover, Wheel };
enum class MediumKind : uint8_t { Water, Air };

// ================================================================================================
//  A hull section: the outline of one station, in the body x-y plane, as a polygon.
//
//  Stored as the STARBOARD half from keel to sheer; the port half is mirrored, because a hull
//  that is not symmetric about its centreline is a different problem and this engine does not
//  have one. Immersion is computed by clipping this polygon against the water plane, which is
//  exact for a box, exact for a deep-V, correct under heel, and needs no case for "fully
//  submerged" -- the clip returns the whole polygon and the law does not notice.
// ================================================================================================
struct Section {
    double z = 0.0;                          // longitudinal station, body z (+ forward)
    std::vector<double> ox, oy;              // outline points, starboard half, keel -> sheer
};

// ================================================================================================
//  One element. Composition over inheritance: every kind's data is present, the tag says which
//  is read. A spec is a handful of these -- the memory does not matter and the absence of a
//  class hierarchy very much does.
// ================================================================================================
struct Element {
    ElementKind kind = ElementKind::Drag;
    std::string name;
    MediumKind medium = MediumKind::Water;   // THE wind hook: the same kind in the other fluid
    Attach mount;

    // What this part WEIGHS, and therefore where it drags the centre of mass to. Optional: a
    // zero here means "not a located mass", not "weightless" -- the hull's own mass is carried
    // by the spec's massDry/massLoaded and placed at hullMassCentre.
    //
    // This is the field that makes the factory's promise real. Two hundred kilos of outboard
    // hanging off a transom is not a detail of the RHIB, it is WHY a RHIB squats when you open
    // the throttles and why its pitch inertia is aft-dominated; and adding a third engine has to
    // move the CG without anyone editing a number that says where the CG is.
    Num mass;

    // ---- Buoyancy
    std::vector<Section> stations;           // ordered by z
    // ---- ...and the BED under the same stations (2026-10-11: beaching). The bed is the second
    // surface every SurfaceSample already carries (`bedNavd`), and a station's outline points
    // that lie below it are pushed back along the bed's normal: a penalty spring sized so the
    // hull's own weight compresses it `contactSettle` metres on its keel points, critically
    // damped per point, with Coulomb friction `contactMu` in the contact plane. Zero settle is
    // no contact at all (the water-only hull: the gate's control), never a stiffer one.
    Num contactSettle;                       // m: compression under the hull's weight (ASSUMED)
    Num contactMu;                           // the bed's friction coefficient

    // ---- Collar (one chamber of a tube)
    Num tubeZ0, tubeZ1;                      // longitudinal extent, body z
    Num tubeR0, tubeR1;                      // radius at each end (linear between)
    Num tubeXOffset;                         // centreline offset from the hull centreplane (+stbd)
    Num tubeYOffset;                         // and its height above the CG
    Num tubeDamping;                         // N s/m PER SQUARE METRE of engaged tube area --
                                             // the membrane. Per-area so the total is a property
                                             // of the collar, not of the slice count.

    // ---- Planing
    Num deadriseDeg;                         // transom deadrise
    Num planingBeam;                         // chine beam
    Num planingArea;                         // reference wetted area at speed
    // The running surface's OWN angle to the hull datum. Without it a planing model cannot start:
    // level hull -> zero angle of attack -> no lift -> no bow-up trim -> level hull, forever, and
    // the boat sits in displacement mode at any throttle. A real after body is not parallel to
    // the waterline, and this is that fact as a number.
    Num planingIncidence;

    // ---- Thruster
    Num maxThrust;                           // static thrust at full throttle
    Num propRadius;                          // for the submersion test
    int  rotation = +1;                      // +1 right-hand, -1 left-hand. Prop walk and torque
                                             // roll are computed FROM this -- flipping it is the
                                             // whole counter-rotating change.
    Num propTorqueArm;                       // effective arm of the torque reaction
    // The speed at which this prop runs out of thrust. It belongs to the GEARCASE AND THE
    // WHEEL, not to the boat: a high-thrust leg swinging a big low-pitch prop makes far more
    // static thrust and gives up earlier, and a speed prop does the opposite. Hard-coding it
    // would have made every engine in every spec the same engine.
    Num freeRunSpeed;

    // ---- Foil (rudder, skeg, keel, sail)
    Num foilArea;
    Num foilLiftSlope;                       // per radian, pre-stall
    Num foilStallDeg;
    Num foilCd0;

    // ---- Drag (hull resistance, windage)
    Num dragArea[3];                         // reference area per BODY axis (x, y, z)
    Num dragCd[3];

    // ---- Hover (the drone). The commanded velocity is the helm's: W/S along the heading and
    // Q/E across it at hoverSpeedH, SHIFT/CTRL up and down at hoverSpeedV (the mount's tilt
    // channel, -1..1), A/D a yaw RATE of hoverYawRate (the mount's steer channel, -1..1). The
    // wrench is m (g up + (v_cmd - v) / tau) at the CG and a critically damped PD on the
    // attitude (body up -> world up, yaw rate -> commanded) with the same tau.
    Num hoverSpeedH, hoverSpeedV, hoverYawRate, hoverTau;

    // ---- Wheel (the car). The mount is the STRUT TOP; the strut runs down the mount's -y and the
    // kingpin is the mount's joint line (jointed: it steers by the helm's angle, sign-flipped,
    // because the helm's steer is the outboard's). `suspRest` is the strut's length at full droop
    // (top to wheel centre), `wheelRadius` the tyre's; the strut meets the bed plane at t and the
    // compression is max(0, suspRest + wheelRadius - t). The spring is progressive, suspK x
    // (1 + (x / suspTravel)^2), damped by suspC on the strut's rate, and never pulls. At the tyre:
    // drive = throttle[0] x driveForce x driven, falling to zero at driveSpeed (the thruster's
    // shape); rolling resistance rollRes x N; the brake (the helm's tilt channel, 0..1) scales
    // the tyre's grip of its ground point along the rolling direction (the hull's anchor law,
    // `contactSettle` its compliance). LATERAL: the contact patch is a spring across the rolling
    // direction -- the anchor's lateral deflection times tyreCorner x N / tyreRelax -- and the
    // anchor relaxes toward the contact as the patch rolls through its relaxation length
    // tyreRelax. At rest that is a spring that holds a parked car across a camber; rolling, its
    // steady state is exactly -tyreCorner x N x (slip angle): the slip law, as the limit of the
    // one law rather than a second one (a velocity law at 60 Hz chattered and let the brake go).
    // The whole tangential force goes through min(1, tyreMu N / |F|).
    Num wheelRadius, suspRest, suspTravel, suspK, suspC;
    Num driveForce, driveSpeed, driven, tyreMu, tyreCorner, tyreRelax, rollRes;
};

// ================================================================================================
//  The spec. Pure data. Everything here is either a Num (with provenance) or geometry.
// ================================================================================================
// ================================================================================================
//  Form - PARTS PLUS FORMS (2026-10-12). A spec is a list of PARTS; a form is how much of each
//  part is engaged (0..1, a weight on its wrench) and the body's own numbers in that form. Three
//  vehicles with no parts in common are three forms of one body whose engagements are 0 or 1; a
//  transformer is the same table with parts in common and mounts that move between forms along
//  their screws (Motor::Log/Exp per part -- Brockett's product of exponentials -- the follow-on).
//  Every part's law already knows absence as zero (a wheel in the air carries nothing, a station
//  out of the water displaces nothing, a rotor at zero throttle lifts nothing), so a form is a
//  weighting and never a branch: the boat with its wheels folded IS the car with its wheels in
//  the air. The body's origin is the form's CG (RigidBody.h), so taking a form moves the origin
//  and carries the pose by the same shift; every part stands where it stood.
// ================================================================================================
struct Form {
    std::string name;                    // "boat", "drone", "car"
    std::string kind;                    // the source spec's kind: the ledger's and the logs'
    std::string display;
    std::vector<double> engage;          // one weight per element of the spec, 0..1
    Num loa, beam, draftStatic;
    Num massDry, massLoaded;
    double hullMassCentre[3] = {0, 0, 0};
    Num inertiaRoll, inertiaPitch, inertiaYaw;
    Num addedMassSurge, addedMassSway, addedMassHeave;
    Num addedInertiaRoll, addedInertiaPitch, addedInertiaYaw;
    Num seatHeight;
    bool recovery = false;
};

struct VesselSpec {
    std::string kind;                        // "rhib.novurania18"
    std::string display;                     // "1998 Novurania 18' RHIB, twin Yamaha F50"

    Num loa, beam, draftStatic;
    Num massDry, massLoaded;
    // Where the hull's OWN mass (structure, collar, deck, tanks, crew) acts, in the spec's
    // geometry datum. Located element masses are added to this; Vessel::Build solves for the
    // combined centroid and re-references every station and mount to it, because RigidBody's
    // body origin IS the centre of mass and nothing else will do.
    double hullMassCentre[3] = {0, 0, 0};
    double cgFromTransom[3] = {0, 0, 0};     // SOLVED at Build, for the record and the ledger
    Num inertiaRoll, inertiaPitch, inertiaYaw;   // about the CG, body axes
    Num addedMassSurge, addedMassSway, addedMassHeave;
    Num addedInertiaRoll, addedInertiaPitch, addedInertiaYaw;

    // THE SEAT (2026-10-11): where the set-down puts the keel (the lowest station point) ABOVE
    // the surface the body rides -- the top of the water-over-bed stack. A boat's is minus its
    // draught (so it floats where it floats; undeclared, that is what it was), a car's is zero
    // (the tyres touch and settle into their springs), a drone's is its hover height.
    Num seatHeight;
    // The recovery body: switched TO, it is placed over the vehicle it was switched from;
    // switched FROM, the next vehicle is placed under it and seats itself (FrameLoop::CycleVehicle).
    bool recovery = false;

    std::vector<Element> elements;
    // THE FORMS. Empty: one implicit form engaging every part with the numbers above (every
    // hull that ever was). Build refuses a form whose engagement table is not the parts'.
    std::vector<Form> forms;

    // Every Num in the spec, for the report. Built by the factory; the registry prints it.
    std::vector<std::pair<std::string, const Num*>> Ledger() const;
    void PrintLedger() const;                // one line per number: value, unit, source
    int AssumedCount() const;
};

// ================================================================================================
//  The registry. LoaderRegistry's shape, for hulls.
// ================================================================================================
using VesselFactory = std::function<VesselSpec()>;

class VesselRegistry {
public:
    void Register(const std::string& kind, VesselFactory make) {
        m_byKind[kind] = std::move(make);
    }
    bool Knows(const std::string& kind) const { return m_byKind.count(kind) != 0; }

    // Returns an empty-kind spec when nothing is registered under that name -- the caller checks
    // `kind.empty()`, because a silently-substituted default hull would be the worst possible
    // failure here (it would float, and it would be the wrong boat).
    VesselSpec Build(const std::string& kind) const {
        auto it = m_byKind.find(kind);
        if (it == m_byKind.end()) {
            Log("[vessel] no such kind '%s' -- %zu registered", kind.c_str(), m_byKind.size());
            return VesselSpec{};
        }
        return it->second();
    }

    std::vector<std::string> Kinds() const {
        std::vector<std::string> v;
        v.reserve(m_byKind.size());
        for (const auto& kv : m_byKind) v.push_back(kv.first);
        return v;
    }

private:
    std::map<std::string, VesselFactory> m_byKind;
};

// The kinds this build ships. Registering here (rather than in main) keeps the list next to the
// specs and makes "what boats exist" one grep.
void RegisterBuiltinVessels(VesselRegistry& reg);

// ---- the built-in specs -----------------------------------------------------------------------
// A 2 x 2 x 2 m cube of known mass. It exists because a factory with one product is not a
// factory, and because a box is the only hull whose draught, heave period and metacentric height
// are all exact closed forms to gate the element code against.
VesselSpec MakeTestBox();

// A 1998 Novurania 18' RHIB on twin Yamaha F50s -- the first hull meant to be SAILED
// rather than gated. Deep-V stations, five collar chambers, two steered outboards, a
// skeg, hull resistance and windage. Most of its numbers are ASSUMED and say so.
VesselSpec MakeRhib18();

// The recovery drone: one Hover element, no water. Switched to when a vehicle flips; flies
// anywhere; the vehicle switched back to is set down under it.
VesselSpec MakeRecoveryDrone();

// The car: a 2003 Subaru Baja (its published figures and its model's measured wheels), lifted
// 0.25 m and sprung long and soft so it drives like the Warthog -- four Wheel elements, windage,
// and a drag box low down for when it is driven into the sea. No buoyancy: it sinks, and the
// drone is the recovery.
VesselSpec MakeCarBaja();

// ONE BODY FROM SEVERAL SPECS: their parts concatenated (each part's name prefixed by its form's),
// one form per spec engaging only its own parts and carrying its spec's own numbers; `kind` the
// kinds joined by '+', the top-level numbers the first form's (the ledger before Build). Empty
// when any input is empty.
VesselSpec CombineForms(const std::vector<std::pair<std::string, VesselSpec>>& named);

}  // namespace ga
