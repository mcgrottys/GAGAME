// ================================================================================================
//  Vessel.cpp - the element laws and the spec ledger. See Vessel.h for the buoyancy argument and
//  VesselSpec.h for why there is no per-hull class.
// ================================================================================================
#include "sim/Vessel.h"

#include <algorithm>
#include <cmath>

namespace ga {

// ================================================================================================
//  The section clip. Sutherland-Hodgman against one half-plane, then the shoelace area and
//  centroid of the survivor. Exact, general, and with no special case for a fully immersed
//  section -- which is the point: a depth-based sectional-area curve would need one, and would
//  still give a heeled hull no righting moment at all.
// ================================================================================================
double ClipSectionArea(const double* px, const double* py, int n, double a, double b, double c,
                       double* cxOut, double* cyOut) {
    if (cxOut) *cxOut = 0.0;
    if (cyOut) *cyOut = 0.0;
    if (n < 3) return 0.0;

    // inside = a*x + b*y + c <= 0
    double ox[64], oy[64];
    int m = 0;
    auto emit = [&](double x, double y) {
        if (m < 64) { ox[m] = x; oy[m] = y; ++m; }
    };
    for (int i = 0; i < n; ++i) {
        const int j = (i + 1) % n;
        const double di = a * px[i] + b * py[i] + c;
        const double dj = a * px[j] + b * py[j] + c;
        const bool ini = di <= 0.0, inj = dj <= 0.0;
        if (ini) emit(px[i], py[i]);
        if (ini != inj) {
            // the crossing point, by linear interpolation on the signed distance
            const double t = di / (di - dj);
            emit(px[i] + t * (px[j] - px[i]), py[i] + t * (py[j] - py[i]));
        }
    }
    if (m < 3) return 0.0;

    double area2 = 0.0, cx = 0.0, cy = 0.0;
    for (int i = 0; i < m; ++i) {
        const int j = (i + 1) % m;
        const double cr = ox[i] * oy[j] - ox[j] * oy[i];
        area2 += cr;
        cx += (ox[i] + ox[j]) * cr;
        cy += (oy[i] + oy[j]) * cr;
    }
    if (std::abs(area2) < 1e-15) return 0.0;
    if (cxOut) *cxOut = cx / (3.0 * area2);
    if (cyOut) *cyOut = cy / (3.0 * area2);
    return 0.5 * std::abs(area2);
}

namespace {

// Build the full closed section from the starboard half (keel -> sheer), mirrored to port.
// A hull that is not symmetric about its centreline is a different problem and this engine does
// not have one; asserting the symmetry here is cheaper than carrying two halves everywhere.
int FullSection(const Section& s, double* px, double* py, int cap) {
    const int h = static_cast<int>(s.ox.size());
    int n = 0;
    for (int i = 0; i < h && n < cap; ++i) { px[n] = s.ox[i]; py[n] = s.oy[i]; ++n; }
    for (int i = h - 1; i >= 0 && n < cap; --i) {
        if (std::abs(s.ox[i]) < 1e-12) continue;      // on the centreline: do not duplicate
        px[n] = -s.ox[i]; py[n] = s.oy[i]; ++n;
    }
    return n;
}

// The trapezoidal longitudinal weight of station i: half the gap either side.
double StationSpan(const std::vector<Section>& st, size_t i) {
    const size_t n = st.size();
    if (n < 2) return 1.0;
    if (i == 0) return 0.5 * (st[1].z - st[0].z);
    if (i == n - 1) return 0.5 * (st[n - 1].z - st[n - 2].z);
    return 0.5 * (st[i + 1].z - st[i - 1].z);
}

}  // namespace

// ================================================================================================
//  Buoyancy.
// ================================================================================================
Bivector Vessel::Buoyancy(const Element& e, const WaterSurface& sea, double simUnix) {
    Bivector w = Bivector::Zero();
    const Motor inv = m_body.pose.Inverse();

    for (size_t i = 0; i < e.stations.size(); ++i) {
        const Section& st = e.stations[i];
        const double span = StationSpan(e.stations, i);

        // Where this station is in the world, so the sea is asked about THIS station and not
        // about some notional hull centre. That is what gives pitch in a wave whose length is
        // comparable to the hull -- the bow and the stern are on different parts of the wave.
        const double bodyC[3] = {0.0, 0.0, st.z};
        double wc[3];
        m_body.ToWorld(bodyC, wc);
        const SurfaceSample ss = sea.At(wc[0], wc[2], simUnix);
        if (!ss.valid) { m_tel.waterValid = false; continue; }
        // ONE evaluation, not two. Both of these used to call At() -- WaterAt does so
        // internally -- so every station and every tube slice summed the whole retained
        // spectrum twice for the same point.
        const Medium med = sea.WaterFrom(ss, wc[0], wc[2]);

        // The water plane, in body coordinates. A plane is a point and a normal; both come
        // across by the pose's inverse, and (point - station) stays hull-sized.
        double pb[3] = {med.sp[0], med.sp[1], med.sp[2]};
        inv.TransformPoint(pb[0], pb[1], pb[2]);
        double nb[3] = {med.nx, med.ny, med.nz};
        inv.TransformDir(nb[0], nb[1], nb[2]);

        // Restricted to this station's plane (body z = st.z), "below the water" is the
        // half-plane  nb.x (x - pb.x) + nb.y (y - pb.y) + nb.z (st.z - pb.z) <= 0.
        const double a = nb[0], b = nb[1];
        const double c = -(nb[0] * pb[0] + nb[1] * pb[1]) + nb[2] * (st.z - pb[2]);

        double px[64], py[64];
        const int n = FullSection(st, px, py, 64);
        double cx = 0.0, cy = 0.0;
        const double area = ClipSectionArea(px, py, n, a, b, c, &cx, &cy);
        if (area <= 0.0) continue;

        const double vol = area * span;
        m_accVol += vol;
        // Immersion depth at this station, for the telemetry only -- never for the force.
        const double keelPt[3] = {0.0, st.oy.empty() ? 0.0 : st.oy[0], st.z};
        m_accDraught += -(nb[0] * (keelPt[0] - pb[0]) + nb[1] * (keelPt[1] - pb[1]) +
                          nb[2] * (keelPt[2] - pb[2]));
        ++m_accStations;

        // The force: rho g V along the LOCAL SURFACE NORMAL (Vessel.h explains why that, and
        // not world up), applied at the centre of buoyancy of the immersed section.
        const double mag = med.rho * kG * vol;
        const double fWorld[3] = {med.nx * mag, med.ny * mag, med.nz * mag};
        const double atBody[3] = {cx, cy, st.z};
        w += m_body.BodyWrench(fWorld, atBody);
    }
    return w;
}

// ================================================================================================
//  Drag -- and windage, which is the same law in the other fluid.
// ================================================================================================
Bivector Vessel::Drag(const Element& e, const WaterSurface& sea, double simUnix) {
    double atBody[3];
    e.mount.Origin(0.0, atBody);
    double wc[3];
    m_body.ToWorld(atBody, wc);

    const Medium med = (e.medium == MediumKind::Air) ? sea.AirAt(wc[0], wc[2], simUnix)
                                                     : sea.WaterAt(wc[0], wc[2], simUnix);
    // Immersion gates the element in ITS OWN medium: a hull panel contributes when it is under
    // water, a windage panel when it is above it, and `up` in the Medium is the only difference
    // between those two sentences.
    const double imm = med.ImmersionAt(wc);
    if (imm <= 0.0) return Bivector::Zero();

    // WETTED AREA IS NOT CONSTANT, and a planing hull is the case that proves it. Held fixed,
    // hull drag rises as v^2 forever and the boat asymptotes at displacement speed no matter how
    // much thrust it has -- it can never climb its own hump. Scaling the reference area by how
    // deep this panel actually sits is the cheapest honest statement of "lift reduces wetted
    // area", and it is what turns the planing law above into an actual transition: lift lowers
    // immersion, immersion lowers drag, drag lowers resistance, speed rises, lift rises.
    // kRefImmersion is the panel's own static depth, so the factor is 1 at rest by construction.
    constexpr double kRefImmersion = 0.35;
    const double wetted = std::min(1.0, imm / kRefImmersion);

    // RELATIVE velocity: the hull through the fluid, not the hull through the ground. This is
    // where a current sets a boat sideways and where being carried by a wave stops being drag.
    double vb[3];
    m_body.VelocityAtBody(atBody, vb);
    double rel[3] = {vb[0] - med.vx, vb[1] - med.vy, vb[2] - med.vz};
    // into body axes, because the areas are declared per body axis
    m_body.pose.Inverse().TransformDir(rel[0], rel[1], rel[2]);

    double fb[3];
    for (int i = 0; i < 3; ++i) {
        // Quadratic in speed, signed: -1/2 rho Cd A |v| v. Written with |v|*v rather than v^2
        // so the sign is carried by the law instead of by a branch on the direction.
        fb[i] = -0.5 * med.rho * e.dragCd[i].v * e.dragArea[i].v * wetted *
                std::abs(rel[i]) * rel[i];
    }
    double fWorld[3] = {fb[0], fb[1], fb[2]};
    m_body.pose.TransformDir(fWorld[0], fWorld[1], fWorld[2]);
    return m_body.BodyWrench(fWorld, atBody);
}

// ================================================================================================
//  Build / Step.
// ================================================================================================
bool Vessel::Build(const VesselSpec& spec, const Motor& pose) {
    if (spec.kind.empty()) {
        Log("[vessel] refusing to build from an empty spec");
        return false;
    }

    m_spec = spec;

    // ---- SOLVE THE CENTRE OF MASS, then move the whole hull onto it. -----------------------
    // RigidBody's body origin is the CG by construction (put it anywhere else and the linear and
    // angular equations couple through a term everyone forgets). A spec, though, is naturally
    // written about a GEOMETRY datum -- mid-length, on the keel line -- because that is where a
    // draughtsman measures from. So the two have to be reconciled exactly once, here.
    //
    // The hull's own mass acts at hullMassCentre; every element carrying a mass acts at its
    // mount. The combined centroid is the CG, and then every station and every mount is
    // translated by -CG so the body origin lands on it. Nothing downstream needs to know.
    {
        double sumM = 0.0, mom[3] = {0.0, 0.0, 0.0};
        double located = 0.0;
        for (const Element& e : m_spec.elements) {
            if (!(e.mass.v > 0.0)) continue;
            double at[3];
            e.mount.Origin(0.0, at);
            for (int i = 0; i < 3; ++i) mom[i] += e.mass.v * at[i];
            located += e.mass.v;
        }
        const double total = (m_spec.massLoaded.v > 0.0) ? m_spec.massLoaded.v
                                                         : m_spec.massDry.v;
        const double hullM = total - located;
        if (hullM < 0.0) {
            Log("[vessel] spec '%s': located element masses (%.0f kg) exceed the loaded "
                "displacement (%.0f kg) -- refusing, because the remainder would be a NEGATIVE "
                "hull and it would float upside down rather than fail",
                m_spec.kind.c_str(), located, total);
            return false;
        }
        for (int i = 0; i < 3; ++i) mom[i] += hullM * m_spec.hullMassCentre[i];
        sumM = located + hullM;
        double cg[3] = {0.0, 0.0, 0.0};
        if (sumM > 1e-9) for (int i = 0; i < 3; ++i) cg[i] = mom[i] / sumM;

        // Re-reference the geometry. Stations carry (x, y) outlines at a longitudinal z; mounts
        // carry a motor and, when jointed, a line to swing about.
        for (Element& e : m_spec.elements) {
            for (Section& st : e.stations) {
                st.z -= cg[2];
                for (double& y : st.oy) y -= cg[1];
                for (double& x : st.ox) x -= cg[0];
            }
            e.mount.at = Motor::Translation(-cg[0], -cg[1], -cg[2]) * e.mount.at;
            if (e.mount.jointed) for (int i = 0; i < 3; ++i) e.mount.axisP[i] -= cg[i];
            e.tubeZ0.v -= cg[2];
            e.tubeZ1.v -= cg[2];
            e.tubeXOffset.v -= cg[0];
            e.tubeYOffset.v -= cg[1];
        }
        for (int i = 0; i < 3; ++i) m_spec.cgFromTransom[i] = cg[i];
        Log("[vessel] %s: CG solved at (%+.3f, %+.3f, %+.3f) m from the geometry datum -- "
            "%.0f kg located in elements, %.0f kg hull",
            m_spec.kind.c_str(), cg[0], cg[1], cg[2], located, hullM);
    }

    m_body = RigidBody{};
    m_body.mp.mass = spec.massLoaded.v > 0.0 ? spec.massLoaded.v : spec.massDry.v;
    // AXES. A moment of inertia is named for the motion it resists, and that is the axis it
    // is taken ABOUT -- so ROLL is about the LONGITUDINAL axis (body z, forward) and PITCH is
    // about the TRANSVERSE one (body x, starboard). Written the other way round first, which the
    // roll-period gate catches because a hull's roll and pitch inertias are nothing alike.
    m_body.mp.I[0][0] = spec.inertiaPitch.v;    // about body x (starboard) = PITCH
    m_body.mp.I[1][1] = spec.inertiaYaw.v;      // about body y (up)        = YAW
    m_body.mp.I[2][2] = spec.inertiaRoll.v;     // about body z (forward)   = ROLL
    m_body.mp.addedM[0] = spec.addedMassSway.v;     // body x = starboard -> sway
    m_body.mp.addedM[1] = spec.addedMassHeave.v;    // body y = up        -> heave
    m_body.mp.addedM[2] = spec.addedMassSurge.v;    // body z = forward   -> surge
    m_body.mp.addedI[0] = spec.addedInertiaPitch.v;
    m_body.mp.addedI[1] = spec.addedInertiaYaw.v;
    m_body.mp.addedI[2] = spec.addedInertiaRoll.v;
    m_body.SetPose(pose);
    m_tel = VesselTelemetry{};
    return true;
}

// ================================================================================================
//  Collar - ONE TUBE CHAMBER, and the curve that IS a RHIB.
//
//  The immersed area of a circle of radius r flooded to depth h is closed form:
//
//      A(h) = r^2 acos((r-h)/r) - (r-h) sqrt(2rh - h^2),      0 <= h <= 2r
//
//  and its derivative is the whole character of the boat. dA/dh is ZERO at first touch, MAXIMAL
//  at half immersion, and zero again once the tube is under: a collar meets the water softly,
//  then very stiffly, then stops caring. That is why a RHIB lands on its tubes instead of
//  slamming on its chines, and it is why this is a law and not a spring constant.
//
//  THE DAMPING TERM IS NOT OPTIONAL. A collar is a ~0.25 bar membrane, not a rigid float: it
//  absorbs on the way in. Without this the stiffness above is a pure spring and the hull rings
//  like a bell on every wave. With it, a slam becomes a thump. It is proportional to the rate of
//  immersion, which is the relative normal velocity -- no state, no history, no filter.
//
//  Chambers are independent by construction, so a puncture is asymmetric and survivable rather
//  than a game over, and so a heeled hull gets its righting moment from the lee tube alone.
// ================================================================================================
Bivector Vessel::Collar(const Element& e, const WaterSurface& sea, double simUnix) {
    Bivector w = Bivector::Zero();
    const double z0 = e.tubeZ0.v, z1 = e.tubeZ1.v;
    const double len = std::abs(z1 - z0);
    if (len < 1e-6) return w;

    // Five slices is enough: the tube is slender and the immersion varies smoothly along it.
    // More would buy precision the sea state does not have.
    constexpr int kSlices = 5;
    const double dz = len / kSlices;
    for (int i = 0; i < kSlices; ++i) {
        const double u = (i + 0.5) / kSlices;
        const double z = z0 + (z1 - z0) * u;
        const double r = e.tubeR0.v + (e.tubeR1.v - e.tubeR0.v) * u;
        if (r <= 1e-6) continue;

        const double atBody[3] = {e.tubeXOffset.v, e.tubeYOffset.v, z};
        double wc[3];
        m_body.ToWorld(atBody, wc);
        const SurfaceSample ss = sea.At(wc[0], wc[2], simUnix);
        if (!ss.valid) { m_tel.waterValid = false; continue; }
        // ONE evaluation, not two. Both of these used to call At() -- WaterAt does so
        // internally -- so every station and every tube slice summed the whole retained
        // spectrum twice for the same point.
        const Medium med = sea.WaterFrom(ss, wc[0], wc[2]);

        // Depth of the tube's AXIS below the surface; the tube spans axis-r .. axis+r, so the
        // flooded depth measured from its underside is that plus r, clamped to the diameter.
        const double axisDepth = med.ImmersionAt(wc);
        double hh = axisDepth + r;
        if (hh <= 0.0) continue;                       // clear of the water: contributes nothing
        if (hh > 2.0 * r) hh = 2.0 * r;

        const double rm = r - hh;
        const double disc = 2.0 * r * hh - hh * hh;
        const double area = r * r * std::acos(std::max(-1.0, std::min(1.0, rm / r))) -
                            rm * std::sqrt(std::max(disc, 0.0));
        if (area <= 0.0) continue;

        const double vol = area * dz;
        m_accVol += vol;

        // Buoyancy along the free surface's NORMAL -- the surface is an equipotential of the
        // effective gravity, so that direction IS the first-order law; world up would be the
        // approximation.
        const double mag = med.rho * kG * vol;
        double f[3] = {med.nx * mag, med.ny * mag, med.nz * mag};

        // The membrane. Relative velocity along the same normal, scaled by how much of the tube
        // is engaged (dA/dh is the engaged width, and that is what the water has to push).
        double vb[3];
        m_body.VelocityAtBody(atBody, vb);
        const double rel[3] = {vb[0] - med.vx, vb[1] - med.vy, vb[2] - med.vz};
        const double vn = rel[0] * med.nx + rel[1] * med.ny + rel[2] * med.nz;
        const double engaged = 2.0 * std::sqrt(std::max(disc, 0.0));   // chord width at h
        // Damping per unit of ENGAGED AREA, so the total does not depend on how finely the
        // chamber happens to be sliced. Written as a per-slice force it was multiplied by the
        // slice count: 50 slices x 9000 N s/m came to ~450 kN s/m against a critical damping
        // for this hull's heave of 2*sqrt(k m) ~ 18 kN s/m. Twenty-four times critical, taken
        // explicitly at 60 Hz where c*dt/m ~ 8 against a stability limit of 2 -- so the moment
        // the tubes engaged the term diverged and threw the boat over. It flipped on spawn.
        const double damp = -e.tubeDamping.v * vn * (engaged * dz);
        f[0] += med.nx * damp;
        f[1] += med.ny * damp;
        f[2] += med.nz * damp;

        w += m_body.BodyWrench(f, atBody);
    }
    return w;
}

// ================================================================================================
//  Planing - Savitsky's SHAPE, not Savitsky.
//
//  Dynamic lift goes as rho v^2 sin(alpha) x wetted area. That is the whole model, and everything
//  a planing hull is famous for is supposed to FALL OUT of it rather than be written down:
//
//    * the HUMP -- at low speed there is no lift, the hull is deep, drag is high; push through
//      and lift reduces immersion, which reduces drag, which raises speed, which raises lift;
//    * TRIM SENSITIVITY -- alpha is the angle the bottom meets the flow, so bow attitude IS the
//      throttle's partner;
//    * PORPOISING -- the centre of pressure moves FORWARD with speed, so the lift and the weight
//      chase each other in pitch. If this model porpoises when badly trimmed, that is evidence
//      it is right, not a bug to damp away.
//
//  Deadrise costs lift: a deep-V pays for its soft ride with a lower lift coefficient than a flat
//  plate, which is why 22 degrees rides well and needs the horsepower.
// ================================================================================================
Bivector Vessel::Planing(const Element& e, const WaterSurface& sea, double simUnix) {
    double atBody[3];
    e.mount.Origin(0.0, atBody);
    double wc[3];
    m_body.ToWorld(atBody, wc);
    const Medium med = sea.WaterAt(wc[0], wc[2], simUnix);
    if (med.ImmersionAt(wc) <= 0.0) return Bivector::Zero();   // airborne: no lift, only gravity

    double vb[3];
    m_body.VelocityAtBody(atBody, vb);
    double rel[3] = {vb[0] - med.vx, vb[1] - med.vy, vb[2] - med.vz};
    m_body.pose.Inverse().TransformDir(rel[0], rel[1], rel[2]);

    const double u = rel[2];                       // forward component, body axes
    if (u <= 0.1) return Bivector::Zero();         // astern or stopped: a planing surface does
                                                   // nothing at all, which is why boats back slowly
    const double spd = std::sqrt(rel[0] * rel[0] + rel[1] * rel[1] + rel[2] * rel[2]);
    if (spd < 1e-6) return Bivector::Zero();

    // The angle the bottom meets the flow. rel[1] > 0 is the hull RISING relative to the water,
    // which unloads the running surface; the loading case is the hull descending into the flow.
    // So the angle of attack is positive exactly when rel[1] is negative.
    // The hull's trim relative to the flow, PLUS the running surface's built-in incidence. The
    // second term is what lets the boat get onto plane at all: without it a level hull meets the
    // flow at zero degrees, makes no lift, never trims up, and never reaches the angle that
    // would have made lift -- measured as a hull stuck at 17 kn with the throttles buried.
    const double alpha = -rel[1] / spd + e.planingIncidence.v;
    if (alpha <= 0.0) return Bivector::Zero();

    const double dead = e.deadriseDeg.v;           // radians (Num parsed "deg")
    const double deadLoss = std::max(0.25, std::cos(dead));
    const double lift = 0.5 * med.rho * spd * spd * e.planingArea.v * alpha * deadLoss;

    // The centre of pressure marches FORWARD as speed rises -- the mechanism behind porpoising.
    // Referenced to a nominal planing speed so the term is bounded and dimensionless.
    const double cpShift = 0.35 * e.planingBeam.v * std::min(1.0, spd / 12.0);
    const double applyAt[3] = {atBody[0], atBody[1], atBody[2] + cpShift};

    // Lift acts normal to the RUNNING SURFACE, which is the hull's own up -- not the world's.
    // That is what makes a heeled planing hull carve instead of skid.
    double fWorld[3] = {0.0, lift, 0.0};
    m_body.pose.TransformDir(fWorld[0], fWorld[1], fWorld[2]);
    return m_body.BodyWrench(fWorld, applyAt);
}

// ================================================================================================
//  Foil - a lifting surface in whatever fluid it is mounted in.
//
//  One kind covers the skeg, a rudder, a centreboard and (in air) a sail: they differ by area,
//  slope, stall and WHICH MEDIUM, and nothing else. This is the element that makes a hull track
//  instead of skate -- a small area a long way aft turns sideslip into a restoring yaw moment,
//  and a RHIB without it is a dinner plate.
// ================================================================================================
Bivector Vessel::Foil(const Element& e, const WaterSurface& sea, double simUnix) {
    double atBody[3];
    e.mount.Origin(0.0, atBody);
    double wc[3];
    m_body.ToWorld(atBody, wc);
    const Medium med = (e.medium == MediumKind::Air) ? sea.AirAt(wc[0], wc[2], simUnix)
                                                     : sea.WaterAt(wc[0], wc[2], simUnix);
    if (med.ImmersionAt(wc) <= 0.0) return Bivector::Zero();

    double vb[3];
    m_body.VelocityAtBody(atBody, vb);
    double rel[3] = {vb[0] - med.vx, vb[1] - med.vy, vb[2] - med.vz};
    m_body.pose.Inverse().TransformDir(rel[0], rel[1], rel[2]);

    // Sideslip in the horizontal plane: the angle between where the foil points (body +z) and
    // where the water is actually going.
    const double u = std::abs(rel[2]);
    const double spd2 = rel[0] * rel[0] + rel[2] * rel[2];
    if (spd2 < 1e-6 || u < 1e-3) return Bivector::Zero();
    const double alpha = std::atan2(rel[0], u);

    const double stall = e.foilStallDeg.v;                       // radians
    const double aEff = std::max(-stall, std::min(stall, alpha));
    // Past stall the lift does not vanish, it plateaus -- clamping the ANGLE rather than zeroing
    // the force is what keeps a hard-over turn from going weightless at the skeg.
    const double cl = e.foilLiftSlope.v * aEff;
    const double q = 0.5 * med.rho * spd2 * e.foilArea.v;

    // Lift opposes the sideslip; profile drag opposes the motion.
    double fb[3] = {-q * cl, 0.0, -q * e.foilCd0.v * ((rel[2] > 0.0) ? 1.0 : -1.0)};
    double fWorld[3] = {fb[0], fb[1], fb[2]};
    m_body.pose.TransformDir(fWorld[0], fWorld[1], fWorld[2]);
    return m_body.BodyWrench(fWorld, atBody);
}

Bivector Vessel::NetWrench(const WaterSurface& sea, const VesselControls& c, double simUnix) {
    Bivector w = Bivector::Zero();
    m_accVol = 0.0;
    m_accDraught = 0.0;
    m_accStations = 0;
    m_tel.waterValid = true;
    m_tel.thrustersWet = 0;

    int thrusterIdx = 0;
    for (const Element& e : m_spec.elements) {
        switch (e.kind) {
            case ElementKind::Buoyancy:
                w += Buoyancy(e, sea, simUnix);
                break;
            case ElementKind::Drag:
                w += Drag(e, sea, simUnix);
                break;
            case ElementKind::Collar:
                w += Collar(e, sea, simUnix);
                break;
            case ElementKind::Planing:
                w += Planing(e, sea, simUnix);
                break;
            case ElementKind::Foil:
                w += Foil(e, sea, simUnix);
                break;
            case ElementKind::Thruster: {
                const int i = thrusterIdx++;
                if (i >= VesselControls::kMaxThrusters || !c.running[i]) break;
                // The steering joint rotates the LINE of action -- one PGA primitive, an offset
                // axis, exactly what Motor::Rotation was written for.
                double atBody[3], fwdBody[3];
                e.mount.Origin(c.steer, atBody);
                e.mount.Forward(c.steer, fwdBody);
                double wc[3];
                m_body.ToWorld(atBody, wc);
                const Medium med = sea.WaterAt(wc[0], wc[2], simUnix);
                // PER-ENGINE submersion: in a heeled turn the outside engine lifts and
                // ventilates while the inside one stays buried, and that asymmetry is a yaw
                // moment that falls out of the sum rather than being written anywhere.
                const double imm = med.ImmersionAt(wc);
                const double wet = (imm <= 0.0)
                                       ? 0.0
                                       : std::min(1.0, imm / std::max(e.propRadius.v, 1e-3));
                if (wet > 0.0) ++m_tel.thrustersWet;
                // THRUST FALLS OFF WITH SPEED. A propeller's thrust is highest at zero
                // advance and vanishes as the boat approaches the speed its pitch can sustain;
                // held constant instead, the hull would accelerate until drag alone stopped it
                // and would feel like a rocket at 30 kn rather than a boat running out of prop.
                // Linear is the honest first shape -- the real curve depends on pitch, slip and
                // cavitation, none of which this spec knows.
                double vFwd[3];
                m_body.VelocityAtBody(atBody, vFwd);
                double fwdW[3] = {fwdBody[0], fwdBody[1], fwdBody[2]};
                m_body.pose.TransformDir(fwdW[0], fwdW[1], fwdW[2]);
                const double along = vFwd[0] * fwdW[0] + vFwd[1] * fwdW[1] + vFwd[2] * fwdW[2];
                const double freeRun = (e.freeRunSpeed.v > 0.1) ? e.freeRunSpeed.v : 19.0;
                const double fall = std::max(0.0, 1.0 - std::max(along, 0.0) / freeRun);
                const double mag = c.throttle[i] * e.maxThrust.v * wet * fall;
                double fWorld[3] = {fwdBody[0] * mag, fwdBody[1] * mag, fwdBody[2] * mag};
                m_body.pose.TransformDir(fWorld[0], fWorld[1], fWorld[2]);
                w += m_body.BodyWrench(fWorld, atBody);
                // Prop torque reaction: a right-hand wheel rolls the hull to port under power.
                // Computed FROM `rotation`, so a counter-rotating pair cancels without anyone
                // writing a rule that says it should.
                Bivector couple;
                couple.b[2] = -e.rotation * std::abs(mag) * e.propTorqueArm.v;
                w += couple;
                break;
            }
            default:
                break;   // Build() already refused these
        }
    }

    if (applyGravity) {
        const double fW[3] = {0.0, -m_body.mp.mass * kG, 0.0};
        const double cg[3] = {0.0, 0.0, 0.0};          // the body origin IS the CG
        w += m_body.BodyWrench(fW, cg);
    }

    m_tel.immersedVol = m_accVol;
    m_tel.draughtM = (m_accStations > 0) ? m_accDraught / m_accStations : 0.0;
    return w;
}

void Vessel::Step(const WaterSurface& sea, const VesselControls& c, double simUnix, double dt) {
    const Bivector w = NetWrench(sea, c, simUnix);
    m_body.Step(w, dt);
    if (!m_body.Sane()) return;

    // ---- telemetry, read off the state rather than tracked alongside it
    const Bivector& t = m_body.Twist();
    m_tel.speedKn = std::sqrt(t.b[0] * t.b[0] + t.b[2] * t.b[2]) / 0.514444;
    double fwd[3] = {0.0, 0.0, 1.0};
    m_body.pose.TransformDir(fwd[0], fwd[1], fwd[2]);
    m_tel.headingRad = std::atan2(fwd[0], fwd[2]);          // 0 = north (+z), + toward east
    double up[3] = {0.0, 1.0, 0.0};
    m_body.pose.TransformDir(up[0], up[1], up[2]);
    m_tel.heelRad = std::atan2(up[0], up[1]);
    m_tel.trimRad = std::asin(std::max(-1.0, std::min(1.0, fwd[1])));

    double wc[3] = {0, 0, 0};
    m_body.pose.TransformPoint(wc[0], wc[1], wc[2]);
    const SurfaceSample ss = sea.At(wc[0], wc[2], simUnix);
    m_tel.depthM = ss.depthM;
    m_tel.waterValid = ss.valid;
    m_tel.aground = ss.valid && ss.depthM < 0.3;
}

// ================================================================================================
//  The spec ledger.
// ================================================================================================
std::vector<std::pair<std::string, const Num*>> VesselSpec::Ledger() const {
    std::vector<std::pair<std::string, const Num*>> v;
    auto add = [&](const char* n, const Num& x) {
        if (!x.unit.empty() || !x.src.empty()) v.emplace_back(n, &x);
    };
    add("loa", loa);
    add("beam", beam);
    add("draft.static", draftStatic);
    add("mass.dry", massDry);
    add("mass.loaded", massLoaded);
    add("inertia.roll", inertiaRoll);
    add("inertia.pitch", inertiaPitch);
    add("inertia.yaw", inertiaYaw);
    add("added.surge", addedMassSurge);
    add("added.sway", addedMassSway);
    add("added.heave", addedMassHeave);
    add("added.roll", addedInertiaRoll);
    add("added.pitch", addedInertiaPitch);
    add("added.yaw", addedInertiaYaw);
    for (const Element& e : elements) {
        const std::string p = e.name + ".";
        add((p + "tube.z0").c_str(), e.tubeZ0);
        add((p + "tube.r0").c_str(), e.tubeR0);
        add((p + "thrust.max").c_str(), e.maxThrust);
        add((p + "prop.radius").c_str(), e.propRadius);
        add((p + "foil.area").c_str(), e.foilArea);
        add((p + "deadrise").c_str(), e.deadriseDeg);
    }
    return v;
}

int VesselSpec::AssumedCount() const {
    int n = 0;
    for (const auto& kv : Ledger()) {
        if (kv.second->Assumed()) ++n;
    }
    return n;
}

void VesselSpec::PrintLedger() const {
    const auto led = Ledger();
    Log("[vessel] %s -- '%s': %zu elements, %zu declared numbers, %d ASSUMED",
        kind.c_str(), display.c_str(), elements.size(), led.size(), AssumedCount());
    for (const auto& kv : led) {
        const Num& n = *kv.second;
        Log("[vessel]   %-22s %12.4g %-8s (SI %12.6g)  %s", kv.first.c_str(), n.raw,
            n.unit.c_str(), n.v, n.src.c_str());
    }
}

}  // namespace ga
