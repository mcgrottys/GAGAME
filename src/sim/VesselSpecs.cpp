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

void RegisterBuiltinVessels(VesselRegistry& reg) {
    reg.Register("box.test", MakeTestBox);
}

}  // namespace ga
