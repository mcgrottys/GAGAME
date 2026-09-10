// ================================================================================================
//  VesselSpecs.cpp - the hulls this build ships. One function per kind, registered by name.
//
//  A new boat belongs HERE, as a function returning data, and nowhere else. If adding one ever
//  requires touching Vessel.cpp, the element set was wrong and that is the thing to fix.
// ================================================================================================
#include "sim/VesselSpec.h"

namespace ga {

// ================================================================================================
//  box.test -- a 2 m x 2 m x 5 m rectangular barge, 2000 kg.
//
//  It exists because a factory with one product is not a factory, and because a box is the only
//  hull whose draught, heave period AND roll period are all exact closed forms:
//
//      draught      d  = m / (rho * B * L)
//      heave        T  = 2 pi sqrt(m / (rho g B L))
//      roll         T  = 2 pi sqrt(I_roll / (m g GM)),  GM = d/2 + (L B^3 / 12) / V - KG
//
//  Deliberately NOT a cube: with B != L the roll and pitch inertias differ, so the roll-period
//  gate discriminates the body-axis mapping. A cube would have passed it either way round, and
//  the mapping was in fact written the wrong way round first.
// ================================================================================================
VesselSpec MakeTestBox() {
    VesselSpec s;
    s.kind = "box.test";
    s.display = "2 x 2 x 5 m test barge (analytic gate hull)";

    const double B = 2.0, H = 2.0, L = 5.0, M = 2000.0;

    s.beam = Num::Of(B, "m", "DECLARED: this hull is a definition, not a measurement");
    s.loa = Num::Of(L, "m", "DECLARED: this hull is a definition, not a measurement");
    s.massDry = Num::Of(M, "kg", "DECLARED: this hull is a definition, not a measurement");
    s.massLoaded = s.massDry;

    // Solid box about its centre. Roll is about the LONGITUDINAL axis, pitch about the
    // TRANSVERSE one -- named for the motion they resist.
    s.inertiaRoll = Num::Of(M / 12.0 * (B * B + H * H), "kg", "DERIVED: solid box, m/12 (B^2+H^2)");
    s.inertiaPitch = Num::Of(M / 12.0 * (H * H + L * L), "kg", "DERIVED: solid box, m/12 (H^2+L^2)");
    s.inertiaYaw = Num::Of(M / 12.0 * (B * B + L * L), "kg", "DERIVED: solid box, m/12 (B^2+L^2)");
    // Added mass stays zero here on purpose: every closed form above is the dry-inertia one, and
    // a gate that had to model entrained water would not be a closed form any more.

    Element hull;
    hull.kind = ElementKind::Buoyancy;
    hull.name = "hull";
    hull.medium = MediumKind::Water;
    // Two stations (a prism needs no more -- the trapezoidal rule is exact on it). Starboard
    // half outline, keel to sheer: up the centreline, out along the bottom, up the side, in
    // along the deck.
    for (double z : {-0.5 * L, 0.5 * L}) {
        Section st;
        st.z = z;
        st.ox = {0.0, 0.5 * B, 0.5 * B, 0.0};
        st.oy = {-0.5 * H, -0.5 * H, 0.5 * H, 0.5 * H};
        hull.stations.push_back(st);
    }
    s.elements.push_back(hull);

    // Hull drag. MOUNTED LOW, at body y = -0.9: a Drag element only acts while its mount is
    // immersed, and the CG of a floating barge sits well ABOVE the waterline -- mounted at the
    // origin it would never once engage, which is exactly what the first version of this spec
    // did and what made the wave gate ring undamped at three times the wave height.
    Element drag;
    drag.kind = ElementKind::Drag;
    drag.name = "hull.drag";
    drag.medium = MediumKind::Water;
    drag.mount.at = Motor::Translation(0.0, -0.45 * H, 0.0);
    // Bluff-body coefficients on the projected area of each face. A barge is the one hull where
    // these are honestly just that: no fairing, no form factor to argue about.
    drag.dragArea[0] = Num::Of(H * L, "1", "DERIVED: side projected area, H x L");
    drag.dragArea[1] = Num::Of(B * L, "1", "DERIVED: plan area, B x L");
    drag.dragArea[2] = Num::Of(B * H, "1", "DERIVED: end projected area, B x H");
    drag.dragCd[0] = Num::Of(1.2, "1", "ASSUMED: flat plate normal to flow");
    drag.dragCd[1] = Num::Of(1.5, "1", "ASSUMED: bluff vertical, heave damping");
    drag.dragCd[2] = Num::Of(0.9, "1", "ASSUMED: square-ended barge");
    s.elements.push_back(drag);

    return s;
}

// ================================================================================================
//  MakeRhib18 -- a 1998 Novurania 18' RHIB on twin Yamaha F50s.
//
//  THE PROVENANCE RULE IS THE POINT. Every number below carries where it came from, and the ones
//  I could not source say ASSUMED and say WHY. Novurania has never published a full spec sheet
//  for a 1998 hull -- the model designation itself is uncertain (their metric names put an 18
//  footer near a "550") -- so most of the geometry here is DERIVED from the two dimensions that
//  are certain (18 ft LOA, the beam a trailerable RHIB of that length carries) using ordinary
//  small-craft practice. That is honest guessing, labelled as guessing. The registry prints the
//  ledger on every spawn, so what is assumed is in front of you rather than buried here.
//
//  1998 matters for two of these: the tubes are Hypalon/CSM rather than the PVC a modern boat
//  would use (heavier per unit area, hence the collar mass), and the hull is a conventional
//  moulded deep-V rather than anything stepped.
//
//  TWIN F50s ON AN 18 FOOTER IS A LOT OF ENGINE. ~110 kg each, hung aft of and below the CG:
//  the boat squats under power, the pitch inertia is aft-dominated, and with BOTH props turning
//  the same way (the user's rigging -- `rotation` is +1 on both) there is a standing prop walk
//  and a torque roll that a counter-rotating pair would cancel. That is character, not error;
//  flipping one to -1 is the whole counter-rotating change.
// ================================================================================================
VesselSpec MakeRhib18() {
    VesselSpec s;
    s.kind = "rhib.novurania18";
    s.display = "1998 Novurania 18' RHIB, twin Yamaha T50TLRW (High Thrust, 4-stroke)";

    // ---- the two dimensions that are actually known -----------------------------------------
    const double LOA = 5.49;      // 18 ft, tube tip to tube tip
    const double BOA = 2.30;      // overall, tube to tube
    const double RT = 0.25;       // tube radius

    // ---- and the ones the owner's own photographs corrected --------------------------------
    // THE RIGID HULL IS NEITHER THE BOAT'S LENGTH NOR ITS BEAM, and that was the loft's biggest
    // error. Astern, the tubes finish in flat caps standing well proud of the transom; forward
    // they run out past the stem into the bow cone. So the GRP is shorter than LOA at both ends.
    //
    // Across is worse. The tubes are bonded ONTO the hull's topsides and OVERLAP them, so the
    // transom between them is barely half the overall beam -- not "BOA less two tube diameters",
    // which is what this file assumed and which made the running surface 30% too wide. On a
    // planing hull that is not a detail: Savitsky's lift goes as beam SQUARED.
    //
    // Read off the stern-on and deck photographs; both agree on a chine beam near 1.3 m against
    // the 2.3 m overall.
    const double HL = 4.90;       // rigid hull length; the tubes overhang ~0.3 m aft, ~0.3 fwd
    const double BH = 0.65;       // half-beam at the chine -- MEASURED off the transom photo
    const double KEEL = -0.45;    // deepest point of the V below the CG datum
    const double SHEER = 0.28;    // gunwale height above the same datum
    const double RAIL = 0.055;    // the spray rail's flat
    const double DEAD = 22.0 * 3.14159265358979323846 / 180.0;   // transom deadrise

    s.loa = Num::Of(18.0, "ft", "Novurania 18 -- the model name IS the length");
    s.beam = Num::Of(BOA, "m", "ASSUMED: typical overall beam for a trailerable 18 ft RHIB "
                               "(2.2-2.4 m); no 1998 spec sheet found");
    s.draftStatic = Num::Of(0.30, "m", "ASSUMED: hull only, drives nothing -- the clip computes "
                                       "the real draught");

    // ---- masses ------------------------------------------------------------------------------
    const double M_HULL = 430.0, M_ENG = 220.0, M_LOAD = 250.0;
    s.massDry = Num::Of(M_HULL + M_ENG, "kg",
                        "ASSUMED: ~430 kg GRP hull + Hypalon collar (1998 CSM is heavier than "
                        "modern PVC) + 220 kg engines");
    s.massLoaded = Num::Of(M_HULL + M_ENG + M_LOAD, "kg",
                           "ASSUMED: + 250 kg of fuel, gear and two crew");
    const double M = M_HULL + M_ENG + M_LOAD;

    // Radii of gyration as fractions of the principal dimension -- standard small-craft practice
    // when no inclining experiment exists. Pitch and yaw are inflated over a bare hull because
    // 220 kg of outboard hangs at the transom, 2.5 m aft of the CG.
    const double kRoll = 0.38 * BOA, kPitch = 0.30 * LOA, kYaw = 0.30 * LOA;
    s.inertiaRoll = Num::Of(M * kRoll * kRoll, "kg",
                            "DERIVED: radius of gyration 0.38 x beam (small-craft practice)");
    s.inertiaPitch = Num::Of(M * kPitch * kPitch, "kg",
                             "DERIVED: radius of gyration 0.30 x LOA, aft-loaded by the engines");
    s.inertiaYaw = Num::Of(M * kYaw * kYaw, "kg", "DERIVED: radius of gyration 0.30 x LOA");

    // Added mass. Heave is the big one for a planing hull; sway larger still because the hull
    // presents its whole side. Surge is small -- a hull is fine in that direction, which is the
    // entire point of a hull.
    // The hull's own mass acts a little aft of mid-length and low: a RHIB carries its console,
    // tanks and crew abaft midships, and the collar and deck are low. The ENGINES are placed as
    // element masses below, which is what actually drags the CG aft.
    // The hull STRUCTURE alone -- moulding, deck, collar. Symmetric about mid-length, and low.
    // Everything else that weighs anything is placed as a Ballast element below, where it can be
    // argued with individually.
    //
    // MEASURED BY ITS CONSEQUENCE. Set 0.48 m aft first, on top of the engines' own moment,
    // and the solved CG landed 1.06 m abaft mid-length -- only 31% of LOA from the transom. The
    // hull then ran at +22 deg of trim on plane, where a real deep-V sits at 3-6, because the
    // running surface was being asked to lift a boat balanced almost on its transom.
    //
    // The structure and collar ARE symmetric about mid-length, and the console, tank and crew
    // sit close to it, so the hull's own mass belongs near zero. The engines alone then pull the
    // CG to ~2.05 m from the transom, 37% of LOA, which is where a RHIB of this size actually
    // balances -- and that is the number doing the work, which is the whole point.
    s.hullMassCentre[0] = 0.0;
    s.hullMassCentre[1] = -0.05;
    s.hullMassCentre[2] = 0.0;

    s.addedMassSurge = Num::Of(0.05 * M, "kg", "ASSUMED: 5% of displacement (fine entry)");
    s.addedMassSway = Num::Of(0.80 * M, "kg", "ASSUMED: 80% -- broadside, the classic figure");
    s.addedMassHeave = Num::Of(0.55 * M, "kg", "ASSUMED: 55% for a deep-V at rest");
    s.addedInertiaRoll = Num::Of(0.25 * M * kRoll * kRoll, "kg", "ASSUMED: 25% of dry roll");
    s.addedInertiaPitch = Num::Of(0.30 * M * kPitch * kPitch, "kg", "ASSUMED: 30% of dry pitch");
    s.addedInertiaYaw = Num::Of(0.30 * M * kYaw * kYaw, "kg", "ASSUMED: 30% of dry yaw");

    // ---- the rigid hull: a deep-V, lofted from stations ---------------------------------------
    // Nine stations, transom to stem. Half-beam tapers forward and the deadrise SHARPENS toward
    // the bow (a constant-deadrise hull pounds; the warp is what makes a V ride). Each station is
    // the starboard half from keel to sheer: keel, chine, sheer.
    Element hull;
    hull.kind = ElementKind::Buoyancy;
    hull.name = "hull";
    hull.medium = MediumKind::Water;
    for (int i = 0; i < 9; ++i) {
        const double u = double(i) / 8.0;            // 0 transom .. 1 stem
        const double z = -0.5 * HL + u * HL;
        // Beam: full aft, holding to about two thirds forward, then a fine entry.
        const double taper = (u < 0.62) ? (1.0 - 0.10 * (u / 0.62))
                                        : (0.90 - 0.86 * ((u - 0.62) / 0.38));
        const double b = BH * ((taper > 0.06) ? taper : 0.06);
        // Deadrise warps from 22 deg aft to ~46 deg at the stem.
        const double dead = DEAD * (1.0 + 1.1 * u * u);
        // KEEL ROCKER, and the forefoot is DEEP. The rocker used to start at 62% and lift the
        // stem 0.30 m, which drew a shallow, full bow; the beached photographs show the opposite
        // -- the V carries deep and sharp right to a raked stem, which is what lets the boat put
        // its nose into a sea instead of slapping over it. Straight further aft, and less lift.
        const double lift = (u < 0.74) ? 0.0 : 0.38 * ((u - 0.74) / 0.26) * ((u - 0.74) / 0.26);
        const double yKeel = KEEL + lift * (-KEEL + 0.10);
        const double yChine = (std::min)(yKeel + b * std::tan(dead), SHEER);
        Section st;
        st.z = z;
        // THE SPRAY RAIL. It is the bright line running the whole length of the hull in every
        // photograph, and it is not trim: it is the widest part of the section at very nearly
        // the depth the boat floats at. It knocks the sheet down, it carries dynamic pressure,
        // and -- now that resistance is measured off these polygons rather than declared -- it
        // is a good part of where the roll damping comes from. Angled slightly down and out,
        // which is what throws the spray clear instead of letting it climb the topside.
        // It fades out forward, where the stem is too fine to carry one.
        const double rail = (yChine < SHEER - 0.02) ? RAIL * ((u < 0.88) ? 1.0 : 0.0) : 0.0;
        if (rail > 0.0) {
            st.ox = {0.0, b, b + rail, b + rail};
            st.oy = {yKeel, yChine, yChine - 0.30 * rail, SHEER};
        } else {
            st.ox = {0.0, b, b};
            st.oy = {yKeel, yChine, SHEER};
        }
        hull.stations.push_back(st);
    }
    // THE HULL'S OWN RESISTANCE, on the sections above. Only the coefficients are declared --
    // the areas are whatever the clip finds wet this instant, per station, so they follow the
    // hull onto the plane, into the air and upside down without anybody writing those cases.
    // These are the same two numbers the single-point element used to carry; what changed is
    // that they are now applied where the water actually is, which is where roll and yaw
    // damping come from.
    hull.dragCd[0] = Num::Of(1.1, "1", "ASSUMED: hull broadside, on the immersed section's own "
                                       "projected side area");
    hull.dragCd[1] = Num::Of(1.3, "1", "ASSUMED: bluff in heave, on the immersed section's own "
                                       "projected plan area");
    s.elements.push_back(hull);

    // ---- the collar: five independent chambers ------------------------------------------------
    // Independent because a puncture must be asymmetric and survivable rather than a game over,
    // and because the immersion law is per-chamber: A(h) is soft at first touch, very stiff at
    // half immersion and saturating once under -- that curve IS the RHIB's character, and the
    // damping term is what turns a slam into a thump (a collar is a ~0.25 bar membrane, not a
    // rigid float).
    // THE TUBES SIT IN THE WATER, not on it. At +0.10 the tube's underside sat exactly ON the
    // waterline, so the collar contributed nothing until the hull already heeled -- and the
    // measured consequence was a boat more stable INVERTED than upright: floating upside down it
    // had 0.41-0.48 m^3 of collar immersed against 0.04-0.29 upright at plane, because the
    // tubes are above the CG one way up and below it the other.
    //
    // A real RIB carries the lower part of its collar in the water at rest -- the tubes ARE the
    // beam, which is where "wonderful lateral stability... prevents the vessel from tipping or
    // capsizing even when deliberately overloaded to one side" comes from. At -0.05 about a
    // third of the tube is immersed at rest, so the collar is load-bearing form stability from
    // the first degree of heel instead of a fender that only wakes up once the boat is already
    // going over.
    const double tubeY = -0.05, tubeX = 0.5 * BOA - RT;
    const double chamberZ[6] = {-0.5 * LOA, -0.28 * LOA, -0.06 * LOA,
                                0.16 * LOA, 0.34 * LOA, 0.46 * LOA};
    for (int side = 0; side < 2; ++side) {
        const double sx = (side == 0) ? -tubeX : tubeX;
        for (int c = 0; c < 5; ++c) {
            Element tube;
            tube.kind = ElementKind::Collar;
            tube.name = (side == 0 ? "tube.port." : "tube.stbd.") + std::to_string(c);
            tube.medium = MediumKind::Water;
            tube.tubeZ0 = Num::Of(chamberZ[c], "m", "DERIVED: five equal-ish chambers over LOA");
            tube.tubeZ1 = Num::Of(chamberZ[c + 1], "m", "DERIVED: five equal-ish chambers");
            // Constant radius over most of the length, tapering only in the forward-most
            // chamber where the tube sweeps up to the squared-off nose.
            tube.tubeR0 = Num::Of(RT, "m", "ASSUMED: 0.50 m tube diameter, typical for an 18 ft "
                                           "RHIB of this era");
            tube.tubeR1 = Num::Of((c == 4) ? 0.72 * RT : RT, "m",
                                  "ASSUMED: constant aft, tapering into the bow cone");
            tube.tubeXOffset = Num::Of(sx, "m", "DERIVED: overall beam less one tube radius");
            tube.tubeYOffset = Num::Of(tubeY, "m", "ASSUMED: collar sits just above the sheer");
            // SIZED AGAINST CRITICAL, not picked. The collar's wetted area at rest is ~4 m^2,
            // so 1400 N s/m per m^2 gives ~5.6 kN s/m of heave damping against a critical
            // 2*sqrt(k m) ~ 18 kN s/m -- about 0.3 of critical, which is a membrane that
            // absorbs a slam without turning the boat into a brick.
            tube.tubeDamping = Num::Of(2400.0, "1",
                                       "DERIVED: ~0.3 of critical heave damping over the collar's "
                                       "wetted area; N s/m per m^2 of engaged tube");
            s.elements.push_back(tube);
        }
    }

    // ---- planing ------------------------------------------------------------------------------
    Element plane;
    plane.kind = ElementKind::Planing;
    plane.name = "running.surface";
    plane.medium = MediumKind::Water;
    // WHERE THE PRESSURE ACTS, which is not the middle of the boat. A planing hull carries its
    // dynamic load on the after third of the running surface -- the water leaves the hull at the
    // transom and the pressure peak sits just forward of it -- and that is the whole reason a
    // boat trims BOW-UP as it accelerates instead of simply rising.
    //
    // WHICH WAY THE MOMENT GOES, worked out the hard way. Moved to -0.30 LOA first on the
    // reasoning that a planing hull "rides on its after sections" -- but an upward force AFT of
    // the CG lifts the STERN and drives the bow DOWN, and it measured exactly that: 3070 N at
    // 1.1 m aft is 3400 N m of bow-down, and the hull ran at -1.6 deg no matter the throttle.
    //
    // The pressure peak on a planing surface is at the SPRAY ROOT -- the forward edge of the
    // wetted length, where the flow first meets the hull -- not back at the transom, where the
    // water is leaving and the pressure has already returned to atmospheric. Forward of the CG
    // is where it belongs, and then the centre-of-pressure march with speed carries it further
    // forward and gives porpoising something to be.
    plane.mount.at = Motor::Translation(0.0, KEEL, -0.05 * HL);
    plane.deadriseDeg = Num::Of(22.0, "deg", "ASSUMED: transom deadrise for a 1998 deep-V RHIB");
    plane.planingBeam = Num::Of(2.0 * BH, "m",
                                "MEASURED off the owner's stern-on photo: the tubes overlap the "
                                "topsides, so the running surface is ~1.3 m, not the 1.7 m that "
                                "BOA-less-two-tube-diameters implies. Savitsky's lift goes as "
                                "beam SQUARED, so this was the loft's most expensive error");
    plane.planingIncidence = Num::Of(3.0, "deg",
                                     "ASSUMED: the after buttocks sit ~3 deg to the datum, which "
                                     "is the ordinary range for a moulded deep-V; TUNED to put "
                                     "the running trim in the real 3-6 deg band");
    plane.planingArea = Num::Of(0.62 * HL * 2.0 * BH, "1",
                                "DERIVED: 62% of the chine rectangle wets at speed");
    s.elements.push_back(plane);

    // ---- twin outboards -----------------------------------------------------------------------
    // BOTH standard rotation, per the user's boat. Prop walk and torque roll are computed FROM
    // `rotation`, so a counter-rotating pair is these two characters and nothing else.
    for (int side = 0; side < 2; ++side) {
        const double sx = (side == 0) ? -0.34 : 0.34;
        Element eng;
        eng.kind = ElementKind::Thruster;
        eng.name = side == 0 ? "outboard.port" : "outboard.stbd";
        eng.medium = MediumKind::Water;
        // Mounted at the transom, and the prop sits BELOW the CG -- which is why a RHIB squats
        // when you open the throttles instead of simply going forwards.
        eng.mount.at = Motor::Translation(sx, -0.38, -0.5 * LOA - 0.10);
        // Steering is a rotation about the near-vertical line through the mount. ONE PGA
        // primitive; the offset axis is free, which is the whole reason the joint is a LINE.
        // The steering line, in the mount's OWN frame: through its origin, near-vertical. It
        // was written with the mount's body-frame position here, which made Posed() translate
        // the offset twice and swing the leg ~2.9 m off the transom every time the wheel moved.
        eng.mount.jointed = true;
        eng.mount.axisP[0] = 0.0; eng.mount.axisP[1] = 0.0; eng.mount.axisP[2] = 0.0;
        eng.mount.axisD[0] = 0.0; eng.mount.axisD[1] = 1.0; eng.mount.axisD[2] = 0.0;
        eng.mount.lo = -0.60; eng.mount.hi = 0.60;     // ~34 deg either side, hard over
        eng.mount.rate = 1.2;                          // rad/s at the helm pump

        // The TRIM line: the transverse pin, also through the mount. Positive rotation about
        // local +x carries the thrust axis (+z) toward -y, i.e. angles it DOWN at the transom,
        // which pushes the stern down and the bow up -- trimming out. -5 to +15 degrees is the
        // ordinary working range of an outboard's trim before it starts ventilating.
        eng.mount.tilted = true;
        eng.mount.tiltD[0] = 1.0; eng.mount.tiltD[1] = 0.0; eng.mount.tiltD[2] = 0.0;
        eng.mount.tiltLo = -0.0873;                    // -5 deg, tucked under
        eng.mount.tiltHi = 0.2618;                     // +15 deg, trimmed out
        eng.mount.tiltRate = 0.20;                     // rad/s -- a trim pump is not fast
        // THESE ARE HIGH THRUST ENGINES, which is a different machine from an F50 and not a
        // trim level. Yamaha's T prefix is the High Thrust series: a lower gear ratio (~2.3:1
        // against ~1.85) swinging a larger-diameter, lower-pitch wheel. It converts the same
        // 50 hp into markedly more STATIC thrust and gives it up at a lower speed -- the leg is
        // built to move weight, not to chase a top end.
        //
        // So the pair of numbers moves together, and moving only one would be the mistake:
        // ~11 kgf/hp static instead of ~8, and a free-running speed near 15 m/s (29 kn) instead
        // of 19. On a 900 kg hull that is MORE hump-crossing authority and a lower terminal
        // speed, which is exactly the trade the owner of a high-thrust leg has made.
        eng.maxThrust = Num::Of(5400.0, "n",
                                "USER: T50TLRW is Yamaha HIGH THRUST -- ~11 kgf/hp static on a "
                                "big low-pitch wheel, vs ~8 for a standard F50 leg");
        // CORRECTED BY THE BOAT ITSELF. I inferred 15 m/s (~29 kn) from the high-thrust label,
        // and the owner reports 35-40 mph -- 30-35 kn -- so the real leg pulls well past that.
        // The static-thrust half of the high-thrust reading stands; the top-end penalty I
        // deduced from it does not, and a measurement outranks an inference from a model name.
        eng.freeRunSpeed = Num::Of(20.0, "m/s",
                                   "USER: boat measures 35-40 mph, so the leg pulls past 18 m/s");
        eng.propRadius = Num::Of(0.1905, "m",
                                 "USER: high-thrust leg swings a bigger wheel -- 15 in class");
        eng.rotation = +1;   // USER: this boat's pair are BOTH standard rotation
        // THE TORQUE REACTION IS SHAFT TORQUE, not thrust times a moment arm -- getting that
        // wrong is worth a factor of four and it showed as a hull heeling 42 degrees under
        // power. For a propeller the torque and thrust coefficients give tau ~ T * R / 4, so
        // the effective arm is a QUARTER OF THE PROP RADIUS, and at 4 kN that is ~165 N m per
        // engine. Cross-check: 50 hp at ~5000 rpm through a 2:1 gearcase is ~280 N m of shaft
        // torque per engine at full noise, so this is the right order and slightly under.
        // tau ~ T*R/4 from the propeller's KQ/KT ratio -- and BOTH factors just went up: a
        // high-thrust leg makes more thrust on a bigger wheel, so its torque reaction is larger
        // too. With both engines turning the same way (this boat's rigging) that is a standing
        // roll the helmsman lives with, and it is the one thing a counter-rotating pair would
        // have cancelled for free.
        eng.propTorqueArm = Num::Of(0.1905 / 4.0, "m",
                                    "DERIVED: tau = T*R/4, on the high-thrust wheel's radius");
        // 110 kg each, hung at the transom. This is the number that makes the boat squat when
        // you open the throttles: Build solves the CG from it, so the hull is trimmed stern-down
        // before it has moved at all, and the planing surface therefore meets the flow at a
        // positive angle from the first metre.
        eng.mass = Num::Of(112.0, "kg",
                           "USER: T50TLRW, four-stroke -- ~112 kg dry; the high-thrust gearcase "
                           "is slightly heavier than the standard leg");
        s.elements.push_back(eng);

        // THE LOWER UNIT, as its own steered foil. It shares the engine's mount and steering
        // line, so it swings with the leg and its side force lands at the transom -- abaft the
        // CG, which is what turns a boat, and only ~0.4 m below it, which is why it turns
        // without rolling the hull the way a deep keel skeg would.
        //
        // This is where an outboard boat's turning force actually comes from. The thrust vector
        // alone only yaws the hull; the leg is what it pushes against.
        Element leg;
        leg.kind = ElementKind::Foil;
        leg.name = side == 0 ? "gearcase.port" : "gearcase.stbd";
        leg.medium = MediumKind::Water;
        leg.mount = eng.mount;                     // same place, same steering and trim lines
        leg.foilArea = Num::Of(0.055, "1",
                               "ASSUMED: one gearcase + anti-ventilation plate in profile");
        leg.foilLiftSlope = Num::Of(2.4, "1", "DERIVED: low-aspect strut, dCl/dalpha ~ 2.4 /rad");
        leg.foilStallDeg = Num::Of(28.0, "deg", "ASSUMED: struts stall late and softly");
        leg.foilCd0 = Num::Of(0.03, "1", "ASSUMED: gearcase profile drag");
        s.elements.push_back(leg);
    }

    // ---- the skeg: what makes it track -------------------------------------------------------
    // Without this a RHIB is a dinner plate. It is a small area a long way aft, which is exactly
    // the geometry that turns yaw rate into a restoring moment.
    Element skeg;
    skeg.kind = ElementKind::Foil;
    skeg.name = "skeg";
    skeg.medium = MediumKind::Water;
    skeg.mount.at = Motor::Translation(0.0, KEEL - 0.05, -0.34 * LOA);
    // Measured down from 0.16 by its consequence: a skeg is a small plate and two gearcases,
    // not a keel. At 0.16 it made more side force than the boat weighed and rolled it over in
    // every turn. 0.06 leaves it doing what a skeg does -- resisting yaw -- without steering
    // the boat from below its centre of mass.
    skeg.foilArea = Num::Of(0.05, "1",
                            "ASSUMED: the HULL's keel skeg alone. The gearcases used to be "
                            "lumped in here, on the centreline and fixed, which put their area "
                            "in the wrong place and denied it to the helm -- they are their own "
                            "steered foils now");
    skeg.foilLiftSlope = Num::Of(2.6, "1",
                                 "DERIVED: low-aspect lifting surface, dCl/dalpha ~ 2.6 /rad");
    skeg.foilStallDeg = Num::Of(22.0, "deg", "ASSUMED: low-aspect stall");
    skeg.foilCd0 = Num::Of(0.02, "1", "ASSUMED: appendage profile drag");
    s.elements.push_back(skeg);

    // ---- hull resistance ---------------------------------------------------------------------
    Element drag;
    drag.kind = ElementKind::Drag;
    drag.name = "hull.drag";
    drag.medium = MediumKind::Water;
    drag.mount.at = Motor::Translation(0.0, KEEL + 0.15, 0.0);
    // AXIAL ONLY. The side and plan areas moved onto the hull's own stations, where they are
    // measured rather than estimated and where they can damp rotation; leaving them here as
    // well would count the same water twice. What stays is the one direction a station-by-
    // station projection would get WRONG: axial resistance on a slender body is a whole-hull
    // quantity -- the frontal form drag plus the friction of the whole wetted length, felt once
    // -- not the sum of nine section areas, which would be nine times the hull's own frontal
    // area and would stop the boat dead.
    drag.dragArea[0] = Num::Of(0.0, "1", "the immersed side area is now measured per station");
    drag.dragArea[1] = Num::Of(0.0, "1", "the immersed plan area is now measured per station");
    drag.dragArea[2] = Num::Of(2.0 * BH * 0.35, "1", "DERIVED: immersed transom section");
    drag.dragCd[0] = Num::Of(0.0, "1", "see dragArea[0]");
    drag.dragCd[1] = Num::Of(0.0, "1", "see dragArea[1]");
    // BACK-SOLVED FROM A KNOWN RATIO, not guessed. A planing hull of this size runs at a
    // resistance/weight ratio near 0.13 at 16 kn, so ~1150 N for this boat. At 8.4 m/s and 57%
    // wetted that requires Cd*A ~ 0.056 m^2; on the 0.595 m^2 transom section that is Cd 0.10.
    //
    // It was 0.35 -- a flat-plate figure -- which put 4300 N of resistance on the hull, 3.7x
    // reality, and that single number is what pinned the boat at 16 kn: thrust could never
    // exceed it, so the hull never reached the speed where the running surface makes lift, so
    // it never trimmed up, so it never planed. A hull is FINE fore-and-aft; that is what a hull
    // IS, and 0.35 was describing a barn door.
    // The back-solve fixes the PRODUCT Cd*A -- R/W ~ 0.13 at 16 kn is a whole-boat figure and
    // does not care how it is split. The transom section shrank with the corrected chine beam,
    // so the coefficient rises to hold the same resistance rather than quietly making the boat
    // 24% slipperier because a photograph got measured.
    drag.dragCd[2] = Num::Of(0.13, "1",
                             "DERIVED: back-solved from R/W ~ 0.13 at 16 kn for a planing hull "
                             "of this displacement, holding Cd*A across the beam correction");
    s.elements.push_back(drag);

    // ---- windage: the same Drag kind, in the other fluid --------------------------------------
    // This is the wind hook and it costs one element. A RHIB's collar and console present a lot
    // of area for 900 kg, which is why they blow downwind so readily alongside.
    // ---- the load, placed. THIS IS WHERE A REAL BOAT'S DETAILS LIVE. --------------------------
    // The user's boat carries its fuel FORWARD, which is a genuine design choice and worth real
    // degrees of running trim: 70 kg at +1.6 m is a 112 kg m moment against the engines' 626, so
    // it pulls the balance point forward by about an eighth of a metre and takes the bow down
    // with it. Aft tanks would trim the boat very differently and this is the one line that
    // would say so.
    {
        Element fuel;
        fuel.kind = ElementKind::Ballast;
        fuel.name = "fuel";
        fuel.mount.at = Motor::Translation(0.0, -0.20, 1.60);
        fuel.mass = Num::Of(70.0, "kg",
                            "USER: this boat's tanks are FORWARD -- ~90 L of petrol plus tankage");
        s.elements.push_back(fuel);

        Element crew;
        crew.kind = ElementKind::Ballast;
        crew.name = "crew.console";
        crew.mount.at = Motor::Translation(0.0, 0.35, 0.15);
        crew.mass = Num::Of(180.0, "kg",
                            "ASSUMED: two crew standing at the console, plus gear; high up, "
                            "which is where a small boat's roll inertia mostly comes from");
        s.elements.push_back(crew);
    }

    Element windage;
    windage.kind = ElementKind::Drag;
    windage.name = "windage";
    windage.medium = MediumKind::Air;
    windage.mount.at = Motor::Translation(0.0, 0.55, -0.05 * LOA);
    windage.dragArea[0] = Num::Of(LOA * 0.75, "1", "DERIVED: collar + console profile, broadside");
    windage.dragArea[1] = Num::Of(LOA * BOA * 0.35, "1", "DERIVED: plan area above water");
    windage.dragArea[2] = Num::Of(BOA * 0.70, "1", "DERIVED: frontal area");
    windage.dragCd[0] = Num::Of(1.0, "1", "ASSUMED: bluff broadside");
    windage.dragCd[1] = Num::Of(1.0, "1", "ASSUMED");
    windage.dragCd[2] = Num::Of(0.8, "1", "ASSUMED: console + screen");
    s.elements.push_back(windage);

    return s;
}

void RegisterBuiltinVessels(VesselRegistry& reg) {
    reg.Register("box.test", MakeTestBox);
    reg.Register("rhib.novurania18", MakeRhib18);
}

}  // namespace ga
