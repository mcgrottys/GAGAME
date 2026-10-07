// ================================================================================================
//  Vessel.cpp - the element laws and the spec ledger. See Vessel.h for the buoyancy argument and
//  VesselSpec.h for why there is no per-hull class.
// ================================================================================================
#include "sim/Vessel.h"

#include <algorithm>
#include <cmath>
#include <cstring>

namespace ga {

// ================================================================================================
//  The section clip. Sutherland-Hodgman against one half-plane, then the shoelace area and
//  centroid of the survivor. Exact, general, and with no special case for a fully immersed
//  section -- which is the point: a depth-based sectional-area curve would need one, and would
//  still give a heeled hull no righting moment at all.
// ================================================================================================
double ClipSectionArea(const double* px, const double* py, int n, double a, double b, double c,
                       double* cxOut, double* cyOut, double* exOut, double* eyOut) {
    if (cxOut) *cxOut = 0.0;
    if (cyOut) *cyOut = 0.0;
    // The EXTENTS of what survives the clip, which for a slice ARE its projected areas: the
    // projection of a section onto the plane normal to body x is exactly its y-extent times the
    // slice length, and likewise for y. So the resistance law downstream needs no reference area
    // of its own -- the wetted shape reports its own frontal areas, and they vanish with it.
    if (exOut) *exOut = 0.0;
    if (eyOut) *eyOut = 0.0;
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
    if (exOut || eyOut) {
        double xlo = ox[0], xhi = ox[0], ylo = oy[0], yhi = oy[0];
        for (int i = 1; i < m; ++i) {
            xlo = (std::min)(xlo, ox[i]); xhi = (std::max)(xhi, ox[i]);
            ylo = (std::min)(ylo, oy[i]); yhi = (std::max)(yhi, oy[i]);
        }
        if (exOut) *exOut = xhi - xlo;
        if (eyOut) *eyOut = yhi - ylo;
    }
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
        double cx = 0.0, cy = 0.0, ex = 0.0, ey = 0.0;
        const double area = ClipSectionArea(px, py, n, a, b, c, &cx, &cy, &ex, &ey);
        if (area <= 0.0) continue;

        const double vol = area * span;
        m_accVol += vol;
        m_tel.hullVol += vol;
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

        // ---- RESISTANCE, WHERE THE HULL ACTUALLY IS ------------------------------------------
        // The hull resistance used to be ONE force at ONE point, and that single choice was
        // three bugs wearing a coat:
        //
        //   * a point force has NO rotational damping worth the name. The lever was 0.55 m, so
        //     the yaw damping went as that squared -- against a distributed hull's L^2/12 = 2.5
        //     m^2, about eight times short. The heading wandered because nothing opposed yaw.
        //   * roll damping was worse: a force ON THE CENTRELINE CANNOT DAMP ROLL AT ALL.
        //   * and it was gated on the immersion of that one point, so the instant the hull
        //     inverted -- the point now in the air -- every bit of hull resistance switched off
        //     together. A capsized boat kept its angular momentum and coasted: measured at 53
        //     degrees of heading per telemetry line, dead constant, for as long as the log ran.
        //     A CONSTANT RATE IS ZERO DAMPING, and that is the spin.
        //
        // Resistance is a property of the WETTED SURFACE, and the wetted surface is exactly what
        // the clip above just computed. So it is evaluated here, per station, at that station's
        // own immersed centroid, against that station's own local velocity -- and then rotational
        // damping is not a term anybody writes down. It falls out of the fact that a rotating
        // hull moves faster through the water at its ends than at its middle.
        //
        // The areas are not declared either: `ey * span` IS the projection of this immersed slice
        // normal to body x, and `ex * span` its projection normal to body y. A hull that lifts
        // out sheds them continuously; a hull that swamps gains them. There is no inversion case,
        // because inversion was never a case -- it was a point that went dry.
        //
        // Axial (body z) resistance is deliberately NOT here: on a slender body that is a
        // whole-hull quantity -- frontal form drag plus friction over the wetted length -- not a
        // sum of station projections, and it stays on the one element where it was calibrated.
        if (e.dragCd[0].v > 0.0 || e.dragCd[1].v > 0.0) {
            // Two-point Gauss across the immersed width. +-w/(2 sqrt 3) with half the area each
            // integrates x^2 exactly for a uniform strip, so the roll damping this produces is
            // the strip's true second moment rather than the 25% short a quarter-beam split
            // would give.
            const double arm = ex * 0.2886751345948129;
            const double halfLat = 0.5 * e.dragCd[0].v * ey * span;
            const double halfVert = 0.5 * e.dragCd[1].v * ex * span;
            for (int sgn = -1; sgn <= 1; sgn += 2) {
                const double pt[3] = {cx + sgn * arm, cy, st.z};
                double vb[3];
                m_body.VelocityAtBody(pt, vb);
                double rel[3] = {vb[0] - med.vx, vb[1] - med.vy, vb[2] - med.vz};
                inv.TransformDir(rel[0], rel[1], rel[2]);
                // Quadratic and signed, |v| v, so the direction is carried by the law.
                double fb[3] = {-0.5 * med.rho * halfLat * std::abs(rel[0]) * rel[0],
                                -0.5 * med.rho * halfVert * std::abs(rel[1]) * rel[1], 0.0};
                m_body.pose.TransformDir(fb[0], fb[1], fb[2]);
                w += m_body.BodyWrench(fb, pt);
            }
        }
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

    // WETTED AREA IS NOT CONSTANT, and a planing hull is the case that proves it. Held fixed,
    // hull drag rises as v^2 forever and the boat asymptotes at displacement speed no matter how
    // much thrust it has -- it can never climb its own hump. Scaling by how much hull is actually
    // in the water is what turns the planing law into an actual transition: lift lowers
    // immersion, immersion lowers drag, drag lowers resistance, speed rises, lift rises.
    //
    // THE MEASURE HAS TO BE THE HULL, NOT A POINT ON IT. Asking one mount whether IT is wet
    // reads correctly upright and reads zero the moment the hull rolls past its beam ends --
    // which is how a capsized boat came to coast with no resistance whatever. The immersed
    // volume over the static displaced volume asks the whole hull instead: it is exactly 1 at
    // rest by construction rather than by a tuned reference depth, it falls as the hull climbs
    // onto the plane, it is 0 airborne, and inverted-and-swamped it is 1. Nothing about it knows
    // which way up the boat is, which is the point.
    double wetted;
    if (e.medium == MediumKind::Air) {
        // Windage is the mirror question and a point still answers it: the console and collar
        // are either in the air or they are not, and there is no "partly" worth modelling.
        const double imm = med.ImmersionAt(wc);
        if (imm <= 0.0) return Bivector::Zero();
        wetted = std::min(1.0, imm / 0.35);
    } else {
        wetted = m_wetFrac;
        if (wetted <= 1e-4) return Bivector::Zero();
    }

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

    // The transom: the aftmost station, after the CG solve has moved them. The planing law
    // measures its wetted length forward from here.
    m_transomZ = 0.0;
    for (const Element& el : m_spec.elements) {
        for (const Section& st : el.stations) m_transomZ = (std::min)(m_transomZ, st.z);
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
    // Keep the FULL-IMMERSION figures; Step scales them by how wet the hull actually is.
    m_addedM0[0] = spec.addedMassSway.v;
    m_addedM0[1] = spec.addedMassHeave.v;
    m_addedM0[2] = spec.addedMassSurge.v;
    m_addedI0[0] = spec.addedInertiaPitch.v;
    m_addedI0[1] = spec.addedInertiaYaw.v;
    m_addedI0[2] = spec.addedInertiaRoll.v;
    m_dispVol = std::max(m_body.mp.mass / 1025.0, 1e-6);

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
        m_tel.collarVol += vol;

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
// The running surface is evaluated as TWO half-panels, port and starboard, each at a quarter
// beam. That is not a refinement, it is what makes a planing hull stable in roll at all:
//
// A single centreline force cannot produce a righting moment however large it is, so at planing
// trim the model had NO roll stiffness and NO roll damping -- both live in the buoyancy stations
// and the collar, and both are out of the water once the hull is flying. Measured: +-50 degrees
// of heel in a turn and +76 running straight. Split across the beam, a heeled hull puts the low
// panel deeper into the flow and at a larger angle, so it makes more lift and pushes back; a
// rolling hull sees a different vertical velocity on each side, which is the damping. Both fall
// out of asking the same question at two places instead of one.
// ================================================================================================
//  SAVITSKY (1964), and why the empirical form beats the one this file had.
//
//  The hand-rolled law here was lift ~ rho v^2 sin(alpha) x area at a centre of pressure that
//  marched FORWARD with speed. That is unconditionally unstable in trim: alpha = sin(trim) +
//  incidence, the lift acts forward of the CG, so more trim makes more lift makes more trim, and
//  the hull settled at +17 to +29 degrees bow-up where a real deep-V runs at 3-6.
//
//  Savitsky's method closes that loop through the WETTED LENGTH. Writing lambda for the mean
//  wetted-length-to-beam ratio, Cv = V/sqrt(g b) for the speed coefficient, tau for trim in
//  DEGREES and beta for deadrise in degrees:
//
//      CL0  = tau^1.1 (0.0120 lambda^0.5 + 0.0055 lambda^2.5 / Cv^2)
//      CLb  = CL0 - 0.0065 beta CL0^0.60                       (deadrise sheds lift)
//      lift = CLb * 0.5 rho V^2 b^2
//      Cp   = 0.75 - 1 / (5.236 Cv^2 / lambda^2 + 2.40)        (fraction of lambda*b fwd of transom)
//
//  The last line is the one that matters. Cp is bounded in [0.33, 0.75] -- it cannot run away --
//  and the centre of pressure sits at Cp * lambda * b forward of the TRANSOM, so it is the
//  wetted length that carries it. Trim up, the bow lifts clear, lambda shrinks, and the pressure
//  centre walks aft toward the transom, killing the bow-up moment that raised the trim. That is
//  the negative feedback, and it is geometry rather than a tuned gain.
//
//  LAMBDA IS A STATE. Evaluated instantaneously from immersion/tan(tau) it is stiffer than what
//  it replaces -- measured, the hull went from a steady +17 deg to tumbling airborne. The spray
//  root migrates at a finite rate, so lambda relaxes toward its geometric target with a time
//  constant. That lag IS the model.
//
//  Sources: Savitsky, "Hydrodynamic Design of Planing Hulls", Marine Technology 1(4), 1964.
//  The Cp coefficients (5.236, 2.40) are as quoted in the standard reproductions of the method.
//
//  WHAT IS KEPT FROM BEFORE: the split across the beam. Savitsky is a whole-surface method and
//  says nothing about roll, but a single centreline force cannot make a righting moment at all,
//  and at planing trim the stations and the collar are out of the water. So Savitsky sets the
//  MAGNITUDE and the LONGITUDINAL position, and the two panels distribute it athwartships by
//  their own local angle of attack -- which is where roll stiffness and roll damping come from.
// ================================================================================================
Bivector Vessel::Planing(const Element& e, const WaterSurface& sea, double simUnix) {
    const double b = (e.planingBeam.v > 0.0) ? e.planingBeam.v : 1.0;

    // The reference query, at the centreline mount: attitude, speed and immersion.
    double atBody[3];
    e.mount.Origin(0.0, atBody);
    double wc[3];
    m_body.ToWorld(atBody, wc);
    const Medium med = sea.WaterAt(wc[0], wc[2], simUnix);
    const double imm = med.ImmersionAt(wc);

    double vb[3];
    m_body.VelocityAtBody(atBody, vb);
    double rel[3] = {vb[0] - med.vx, vb[1] - med.vy, vb[2] - med.vz};
    m_body.pose.Inverse().TransformDir(rel[0], rel[1], rel[2]);
    const double spd = std::sqrt(rel[0]*rel[0] + rel[1]*rel[1] + rel[2]*rel[2]);

    // LAMBDA RELAXES whether or not the surface is making lift this instant -- a hull coming off
    // the plane has to let its wetted length grow back, and freezing the state while airborne
    // would have it land with the wrong one.
    const double alpha0 = (spd > 1e-6) ? (-rel[1] / spd + e.planingIncidence.v) : 0.0;
    const double lamMax = (m_spec.loa.v > 0.0) ? m_spec.loa.v / b : 4.0;
    const double lamGeom =
        (imm > 0.0 && alpha0 > 1e-3)
            ? std::min(lamMax, (imm / std::max(std::tan(alpha0), 0.02)) / b)
            : lamMax;
    constexpr double kSprayLagS = 0.35;    // how long the spray root takes to migrate
    m_lambda += (lamGeom - m_lambda) * std::min(1.0, m_dt / kSprayLagS);
    m_lambda = std::clamp(m_lambda, 0.05, lamMax);
    m_tel.wettedLambda = m_lambda;

    if (imm <= 0.0) return Bivector::Zero();        // airborne: gravity only
    if (rel[2] <= 0.1 || spd < 1e-6) return Bivector::Zero();   // astern or stopped

    const double tauDeg = std::asin(std::clamp(alpha0, -1.0, 1.0)) * 57.2957795;
    const double cv = spd / std::sqrt(kG * b);
    if (tauDeg <= 0.05 || cv < 0.3) return Bivector::Zero();

    const double lam = m_lambda;
    const double cl0 = std::pow(tauDeg, 1.1) *
                       (0.0120 * std::sqrt(lam) + 0.0055 * std::pow(lam, 2.5) / (cv * cv));
    const double betaDeg = e.deadriseDeg.v * 57.2957795;   // Num parsed "deg" into radians
    const double clb = std::max(0.0, cl0 - 0.0065 * betaDeg * std::pow(cl0, 0.60));
    const double lift = clb * 0.5 * med.rho * spd * spd * b * b;
    if (!(lift > 0.0)) return Bivector::Zero();

    // Cp in [0.33, 0.75] of the wetted length, forward of the transom. Bounded by construction.
    const double cp = 0.75 - 1.0 / (5.236 * cv * cv / (lam * lam) + 2.40);
    const double zCop = m_transomZ + cp * lam * b;
    m_tel.copZ = zCop;

    // Athwartships: half each, weighted by each panel's own angle of attack AGAINST THE
    // REFERENCE -- never against the pair's sum. A heeled hull loads the low panel harder; a
    // rolling one sees different vertical velocity on each side. Stiffness and damping, from
    // one split.
    //
    // NORMALISING TO THE SUM IS WHAT MADE IT WALK. Written as share = a_i / (aP + aS), the two
    // shares add to 1 BY CONSTRUCTION, so the pair always delivered the whole of Savitsky's
    // lift no matter how little running surface was left in the water. Lift one panel clear --
    // ordinary at 29 kn, where the hull carries 0.05 m^3 of the 0.88 it displaces -- and the
    // survivor took share = 1.0: the ENTIRE lift, moved to a quarter beam off the centreline.
    // On the corrected 1.30 m beam that is 8800 N at 0.325 m = 2860 N m against a roll inertia
    // of 859 kg m^2, or 190 deg/s^2, and then the same thing mirrored on the way back. A limit
    // cycle, and it read as the boat riding a sea that was not there: MEASURED at +-18 deg of
    // heel on water of Hs 0.05 m, which is a mirror.
    //
    // Savitsky's b is the beam of the WETTED surface. Half the surface out of the water is not
    // the same lift relocated, it is about half the lift -- so each panel carries its own half
    // scaled by its own alpha over the alpha the lift was computed at. Both panels at the
    // reference angle sum to exactly 1, so the calibrated level-trim case is untouched; a heeled
    // hull makes the low panel do more and the high panel less, which is the righting moment;
    // and a hull with one chine flying makes LESS TOTAL LIFT and settles back onto the other,
    // which is a restoring loop instead of a divergent one.
    const double halfB = 0.25 * b;
    double aP = 0.0, aS = 0.0;
    for (int side = 0; side < 2; ++side) {
        double pb[3] = {(side == 0) ? -halfB : halfB, atBody[1], zCop};
        double pw[3];
        m_body.ToWorld(pb, pw);
        const Medium pm = sea.WaterAt(pw[0], pw[2], simUnix);
        if (pm.ImmersionAt(pw) <= 0.0) continue;
        double pv[3];
        m_body.VelocityAtBody(pb, pv);
        double pr[3] = {pv[0] - pm.vx, pv[1] - pm.vy, pv[2] - pm.vz};
        m_body.pose.Inverse().TransformDir(pr[0], pr[1], pr[2]);
        const double ps = std::sqrt(pr[0]*pr[0] + pr[1]*pr[1] + pr[2]*pr[2]);
        const double a = (ps > 1e-6) ? std::max(0.0, -pr[1] / ps + e.planingIncidence.v) : 0.0;
        (side == 0 ? aP : aS) = a;
    }
    if (aP + aS <= 1e-9) return Bivector::Zero();
    const double aRef = std::max(alpha0, 1e-4);   // the angle `lift` above was computed at

    Bivector w = Bivector::Zero();
    for (int side = 0; side < 2; ++side) {
        // Capped at 1.0: one panel can at most deliver the reference lift on its own, because
        // the wetted beam cannot exceed the beam.
        const double share = 0.5 * std::min(((side == 0) ? aP : aS) / aRef, 2.0);
        if (share <= 0.0) continue;
        const double applyAt[3] = {(side == 0) ? -halfB : halfB, atBody[1], zCop};
        double fWorld[3] = {0.0, lift * share, 0.0};
        m_body.pose.TransformDir(fWorld[0], fWorld[1], fWorld[2]);
        w += m_body.BodyWrench(fWorld, applyAt);
    }
    return w;
}

// ================================================================================================
//  Foil - a lifting surface in whatever fluid it is mounted in.
//
//  One kind covers the skeg, a rudder, a centreboard and (in air) a sail: they differ by area,
//  slope, stall and WHICH MEDIUM, and nothing else. This is the element that makes a hull track
//  instead of skate -- a small area a long way aft turns sideslip into a restoring yaw moment,
//  and a RHIB without it is a dinner plate.
// ================================================================================================
Bivector Vessel::Foil(const Element& e, const WaterSurface& sea, const VesselControls& c,
                      double simUnix) {
    // A JOINTED FOIL FOLLOWS THE HELM. On an outboard boat the lower unit IS a rudder -- a big
    // lateral area at the transom that swings with the leg -- and leaving it out was leaving out
    // most of where the turn comes from. The thrust vector alone yaws the hull without giving it
    // anything to push against, which is a boat that pirouettes rather than turns.
    const double q = e.mount.jointed ? c.steer : 0.0;
    double atBody[3];
    e.mount.Origin(q, atBody, c.tilt);
    double wc[3];
    m_body.ToWorld(atBody, wc);
    const Medium med = (e.medium == MediumKind::Air) ? sea.AirAt(wc[0], wc[2], simUnix)
                                                     : sea.WaterAt(wc[0], wc[2], simUnix);
    if (med.ImmersionAt(wc) <= 0.0) return Bivector::Zero();

    double vb[3];
    m_body.VelocityAtBody(atBody, vb);
    double rel[3] = {vb[0] - med.vx, vb[1] - med.vy, vb[2] - med.vz};
    m_body.pose.Inverse().TransformDir(rel[0], rel[1], rel[2]);
    // ...and then into the FOIL's own frame, so a steered leg measures its angle of attack
    // against where it is actually pointing rather than against the hull's centreline.
    const Motor posed = e.mount.Posed(q, c.tilt);
    posed.Inverse().TransformDir(rel[0], rel[1], rel[2]);

    // Sideslip in the horizontal plane: the angle between where the foil points (local +z) and
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

    // VENTILATION. A lifting surface close under the free surface does not make its full
    // coefficient: it draws air down the low-pressure face and the lift collapses. Without this
    // the skeg was making ~14 kN at 25 kn -- one and a half times the boat's weight -- half a
    // metre below the CG, and since a planing hull has lifted its bottom clear the skeg was the
    // ONLY lateral force acting. The boat rolled onto its back in every hard turn.
    //
    // Scaled by immersion over the foil's own span, which is the ratio that decides whether a
    // surface is working in solid water or drawing air. A real RHIB turns on its banked hull
    // bottom, with the skeg damping yaw rather than providing the turn.
    const double span = std::sqrt((std::max)(e.foilArea.v, 1e-6));
    const double wet = (std::min)(1.0, med.ImmersionAt(wc) / (std::max)(span, 1e-3));
    const double qDyn = 0.5 * med.rho * spd2 * e.foilArea.v * wet * wet;

    // Lift opposes the sideslip; profile drag opposes the motion.
    double fb[3] = {-qDyn * cl, 0.0, -qDyn * e.foilCd0.v * ((rel[2] > 0.0) ? 1.0 : -1.0)};
    // Out of the foil's frame, then out of the body's.
    posed.TransformDir(fb[0], fb[1], fb[2]);
    double fWorld[3] = {fb[0], fb[1], fb[2]};
    m_body.pose.TransformDir(fWorld[0], fWorld[1], fWorld[2]);
    return m_body.BodyWrench(fWorld, atBody);
}

Bivector Vessel::NetWrench(const WaterSurface& sea, const VesselControls& c, double simUnix) {
    Bivector w = Bivector::Zero();
    m_accVol = 0.0;
    m_tel.hullVol = 0.0;
    m_tel.collarVol = 0.0;
    m_accDraught = 0.0;
    m_accStations = 0;
    m_tel.waterValid = true;
    m_tel.thrustersWet = 0;

    // TWO PASSES. Resistance and added mass are both properties of the WETTED HULL, and the
    // wetted hull is not known until the buoyancy clip and the collar have run. Letting that
    // depend on where a spec happens to list its elements is the kind of coupling that works
    // until somebody reorders a table, so the volume is established first and read second.
    for (const Element& e : m_spec.elements) {
        switch (e.kind) {
            case ElementKind::Buoyancy:
                w += Buoyancy(e, sea, simUnix);
                break;
            case ElementKind::Collar:
                w += Collar(e, sea, simUnix);
                break;
            default:
                break;
        }
    }
    // How much hull is in the water, as a fraction of what it displaces at rest: 1 at rest by
    // construction, 0 airborne, and clamped at 1 when swamped.
    m_wetFrac = std::clamp(m_accVol / m_dispVol, 0.0, 1.0);

    int thrusterIdx = 0;
    for (const Element& e : m_spec.elements) {
        switch (e.kind) {
            case ElementKind::Drag:
                w += Drag(e, sea, simUnix);
                break;
            case ElementKind::Planing:
                w += Planing(e, sea, simUnix);
                break;
            case ElementKind::Foil:
                w += Foil(e, sea, c, simUnix);
                break;
            case ElementKind::Thruster: {
                const int i = thrusterIdx++;
                if (i >= VesselControls::kMaxThrusters || !c.running[i]) break;
                // The steering joint rotates the LINE of action -- one PGA primitive, an offset
                // axis, exactly what Motor::Rotation was written for.
                double atBody[3], fwdBody[3];
                e.mount.Origin(c.steer, atBody, c.tilt);
                e.mount.Forward(c.steer, fwdBody, c.tilt);
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

// ================================================================================================
//  F22: the step's water, asked once.
// ================================================================================================
bool Vessel::s_batchWater = true;
bool Vessel::s_batchAudit = false;
std::atomic<uint64_t> Vessel::s_batchSteps{0}, Vessel::s_batchPoints{0}, Vessel::s_batchMisses{0},
    Vessel::s_batchHits{0}, Vessel::s_auditChecks{0}, Vessel::s_auditFails{0};

namespace {
// THE TABLE: the batch's answers served by the point's own doubles; a point not in it is asked of
// the sea behind it and counted (the planing panels). Air and wind go to the sea behind it.
class TableSea : public WaterSurface {
public:
    TableSea(const WaterSurface& sea, const std::vector<double>& xz, const std::vector<SurfaceSample>& out)
        : m_sea(sea), m_xz(xz), m_out(out) {}
    const char* Name() const override { return m_sea.Name(); }
    SurfaceSample At(double wx, double wz, double simUnix) const override {
        const size_t n = m_out.size();
        for (size_t i = 0; i < n; ++i) {
            if (m_xz[2 * i] == wx && m_xz[2 * i + 1] == wz) {
                ++Vessel::s_batchHits;
                return m_out[i];
            }
        }
        ++Vessel::s_batchMisses;
        return m_sea.At(wx, wz, simUnix);
    }
    void WindAt(double wx, double wz, double simUnix, double out[3]) const override {
        m_sea.WindAt(wx, wz, simUnix, out);
    }
private:
    const WaterSurface& m_sea;
    const std::vector<double>& m_xz;
    const std::vector<SurfaceSample>& m_out;
};
}  // namespace

// The points the force loops ask, each by the loop's own body point and the same ToWorld, so the
// doubles match the loops' to the bit: Buoyancy's stations (0, 0, z); Collar's five slices at the
// tube's offsets; Drag's, Planing's and Foil's mounts (the foil at the helm's angle and tilt);
// the running thrusters at the helm's; and the CG the telemetry asks. Not listed: the planing
// panels (their place is the step's lambda state): asked live, counted as misses.
void Vessel::WaterPoints(const VesselControls& c, std::vector<double>& xz) const {
    xz.clear();
    auto add = [&](const double atBody[3]) {
        double wc[3];
        m_body.ToWorld(atBody, wc);
        xz.push_back(wc[0]);
        xz.push_back(wc[2]);
    };
    int thrusterIdx = 0;
    for (const Element& e : m_spec.elements) {
        switch (e.kind) {
            case ElementKind::Buoyancy:
                for (const Section& st : e.stations) {
                    const double bodyC[3] = {0.0, 0.0, st.z};
                    add(bodyC);
                }
                break;
            case ElementKind::Collar: {
                const double z0 = e.tubeZ0.v, z1 = e.tubeZ1.v;
                if (std::abs(z1 - z0) < 1e-6) break;
                constexpr int kSlices = 5;
                for (int i = 0; i < kSlices; ++i) {
                    const double u = (i + 0.5) / kSlices;
                    const double z = z0 + (z1 - z0) * u;
                    const double r = e.tubeR0.v + (e.tubeR1.v - e.tubeR0.v) * u;
                    if (r <= 1e-6) continue;
                    const double atBody[3] = {e.tubeXOffset.v, e.tubeYOffset.v, z};
                    add(atBody);
                }
                break;
            }
            case ElementKind::Drag:
            case ElementKind::Planing: {
                double atBody[3];
                e.mount.Origin(0.0, atBody);
                add(atBody);
                break;
            }
            case ElementKind::Foil: {
                const double q = e.mount.jointed ? c.steer : 0.0;
                double atBody[3];
                e.mount.Origin(q, atBody, c.tilt);
                add(atBody);
                break;
            }
            case ElementKind::Thruster: {
                const int i = thrusterIdx++;
                if (i >= VesselControls::kMaxThrusters || !c.running[i]) break;
                double atBody[3], fwdBody[3];
                e.mount.Origin(c.steer, atBody, c.tilt);
                e.mount.Forward(c.steer, fwdBody, c.tilt);
                add(atBody);
                break;
            }
            default:
                break;
        }
    }
    const double cg[3] = {0.0, 0.0, 0.0};
    add(cg);
}

void Vessel::Step(const WaterSurface& sea, const VesselControls& c, double simUnix, double dt) {
    m_dt = dt;
    Bivector w;
    if (s_batchWater) {
        std::vector<double> xz;
        WaterPoints(c, xz);
        std::vector<SurfaceSample> out(xz.size() / 2);
        sea.AtMany(xz.data(), static_cast<int>(out.size()), simUnix, out.data());
        ++s_batchSteps;
        s_batchPoints += out.size();
        if (s_batchAudit) {   // every answer against the point asked live, bit for bit
            for (size_t i = 0; i < out.size(); ++i) {
                const SurfaceSample live = sea.At(xz[2 * i], xz[2 * i + 1], simUnix);
                const double* a = &out[i].heightNavd;
                const double* b = &live.heightNavd;
                bool same = out[i].valid == live.valid && out[i].foam == live.foam && out[i].sigma2 == live.sigma2;
                for (int k = 0; k < 11 && same; ++k) same = std::memcmp(&a[k], &b[k], sizeof(double)) == 0;
                ++s_auditChecks;
                if (!same) ++s_auditFails;
            }
        }
        const TableSea table(sea, xz, out);
        w = NetWrench(table, c, simUnix);
    } else {
        w = NetWrench(sea, c, simUnix);
    }

    // ADDED MASS IS ENTRAINED WATER, so a hull with no water around it has none. It was applied
    // unconditionally, which meant an AIRBORNE boat still carried 495 kg of heave added mass
    // against a gravity of only m*g -- it fell at 900*9.81/1395 = 6.3 m/s^2, 65% of gravity, and
    // hung in the air. Scaled by how much of the hull is actually immersed, a launched hull
    // falls at g and lands like it means it.
    //
    // The scale is the immersed volume over the static displaced volume, which is 1 at rest by
    // construction and goes to 0 as the hull flies. NetWrench has just accumulated it.
    for (int i = 0; i < 3; ++i) {
        m_body.mp.addedM[i] = m_addedM0[i] * m_wetFrac;
        m_body.mp.addedI[i] = m_addedI0[i] * m_wetFrac;
    }
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
