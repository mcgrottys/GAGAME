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

    // The joint at angle `q`, composed onto the mount. ONE PGA primitive for every articulation
    // on every hull; the offset axis costs nothing extra.
    Motor Posed(double q) const {
        if (!jointed) return at;
        const double c = (q < lo) ? lo : ((q > hi) ? hi : q);
        return at * Motor::Rotation(axisP, axisD, c);
    }
    // The mount's origin in body coordinates -- where a force from this part is applied.
    void Origin(double q, double out[3]) const {
        out[0] = out[1] = out[2] = 0.0;
        Posed(q).TransformPoint(out[0], out[1], out[2]);
    }
    // The mount's forward axis (body +z) in body coordinates -- a thruster's line of action.
    void Forward(double q, double out[3]) const {
        out[0] = 0.0; out[1] = 0.0; out[2] = 1.0;
        Posed(q).TransformDir(out[0], out[1], out[2]);
    }
};

enum class ElementKind : uint8_t { Buoyancy, Collar, Planing, Thruster, Foil, Drag };
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

    // ---- Buoyancy
    std::vector<Section> stations;           // ordered by z

    // ---- Collar (one chamber of a tube)
    Num tubeZ0, tubeZ1;                      // longitudinal extent, body z
    Num tubeR0, tubeR1;                      // radius at each end (linear between)
    Num tubeXOffset;                         // centreline offset from the hull centreplane (+stbd)
    Num tubeYOffset;                         // and its height above the CG
    Num tubeDamping;                         // N per (m/s) of immersion rate -- the membrane

    // ---- Planing
    Num deadriseDeg;                         // transom deadrise
    Num planingBeam;                         // chine beam
    Num planingArea;                         // reference wetted area at speed

    // ---- Thruster
    Num maxThrust;                           // static thrust at full throttle
    Num propRadius;                          // for the submersion test
    int  rotation = +1;                      // +1 right-hand, -1 left-hand. Prop walk and torque
                                             // roll are computed FROM this -- flipping it is the
                                             // whole counter-rotating change.
    Num propTorqueArm;                       // effective arm of the torque reaction

    // ---- Foil (rudder, skeg, keel, sail)
    Num foilArea;
    Num foilLiftSlope;                       // per radian, pre-stall
    Num foilStallDeg;
    Num foilCd0;

    // ---- Drag (hull resistance, windage)
    Num dragArea[3];                         // reference area per BODY axis (x, y, z)
    Num dragCd[3];
};

// ================================================================================================
//  The spec. Pure data. Everything here is either a Num (with provenance) or geometry.
// ================================================================================================
struct VesselSpec {
    std::string kind;                        // "rhib.novurania18"
    std::string display;                     // "1998 Novurania 18' RHIB, twin Yamaha F50"

    Num loa, beam, draftStatic;
    Num massDry, massLoaded;
    double cgFromTransom[3] = {0, 0, 0};     // where the CG sits, for the record
    Num inertiaRoll, inertiaPitch, inertiaYaw;   // about the CG, body axes
    Num addedMassSurge, addedMassSway, addedMassHeave;
    Num addedInertiaRoll, addedInertiaPitch, addedInertiaYaw;

    std::vector<Element> elements;

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

}  // namespace ga
