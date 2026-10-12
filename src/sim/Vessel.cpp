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
        double px[64], py[64];
        const int n = FullSection(st, px, py, 64);
        const SurfaceSample ss = sea.At(wc[0], wc[2], simUnix);
        if (!ss.valid) {   // no water answered: no force, and this station's anchors stand
            m_tel.waterValid = false;
            m_anchorIdx += static_cast<size_t>(n);
            continue;
        }
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

        // The bed under the same station (beaching): GroundAtStation, the law Shell shares.
        w += GroundAtStation(st, px, py, n, med, ss);

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
// ================================================================================================
//  The solids (sim/Solids.h): a prism's holds test, its nearest side, and the deepest surface.
// ================================================================================================
bool PrismHolds(const std::vector<double>& xz, double x, double z) {
    const size_t n = xz.size() / 2;
    bool in = false;
    for (size_t i = 0, j = n - 1; i < n; j = i++) {
        const double xi = xz[2 * i], zi = xz[2 * i + 1], xj = xz[2 * j], zj = xz[2 * j + 1];
        if (((zi > z) != (zj > z)) && (x < (xj - xi) * (z - zi) / (zj - zi) + xi)) in = !in;
    }
    return in;
}

void PrismNearestSide(const std::vector<double>& xz, double x, double z, double& dist, double n[2]) {
    const size_t cnt = xz.size() / 2;
    dist = 1e300;
    n[0] = 1.0; n[1] = 0.0;
    // The footprint's winding, so the outward side of an edge is known.
    double area2 = 0.0;
    for (size_t i = 0, j = cnt - 1; i < cnt; j = i++) area2 += xz[2 * j] * xz[2 * i + 1] - xz[2 * i] * xz[2 * j + 1];
    const double sgn = (area2 >= 0.0) ? 1.0 : -1.0;
    for (size_t i = 0, j = cnt - 1; i < cnt; j = i++) {
        const double ax = xz[2 * j], az = xz[2 * j + 1], bx = xz[2 * i], bz = xz[2 * i + 1];
        const double ex = bx - ax, ez = bz - az;
        const double el2 = ex * ex + ez * ez;
        if (el2 < 1e-18) continue;
        double t = ((x - ax) * ex + (z - az) * ez) / el2;
        t = (t < 0.0) ? 0.0 : ((t > 1.0) ? 1.0 : t);
        const double qx = ax + t * ex, qz = az + t * ez;
        const double d = std::sqrt((x - qx) * (x - qx) + (z - qz) * (z - qz));
        if (d < dist) {
            dist = d;
            // Outward: the edge's left normal for a counter-clockwise ring (x east, z north), flipped
            // for a clockwise one.
            const double el = std::sqrt(el2);
            n[0] = sgn * (-ez / el);
            n[1] = sgn * (ex / el);
            // The normal must point from the point outward: check against (point - foot).
            if (d > 1e-9 && (n[0] * (x - qx) + n[1] * (z - qz)) > 0.0) { n[0] = -n[0]; n[1] = -n[1]; }
        }
    }
}

void Vessel::DeepestSurface(const double pw[3], double bedPen, const double bedN[3], double& pen, double n[3]) const {
    pen = bedPen;
    n[0] = bedN[0]; n[1] = bedN[1]; n[2] = bedN[2];
    if (!m_solids) return;
    for (const SolidPrism& sp : *m_solids) {
        const double dx = pw[0] - sp.cx, dz = pw[2] - sp.cz;
        if (dx * dx + dz * dz > sp.radius * sp.radius) continue;
        const double top = sp.TopAt(pw[0], pw[2]);
        if (pw[1] < sp.BottomAt(pw[0], pw[2]) || pw[1] > top) continue;
        if (!PrismHolds(sp.xz, pw[0], pw[2])) continue;
        // Inside: the way out is the nearest face -- the top plane, or a side.
        const double ny = 1.0 / std::sqrt(1.0 + sp.gx * sp.gx + sp.gz * sp.gz);
        const double dTop = (top - pw[1]) * ny;
        double dSide, ns[2];
        PrismNearestSide(sp.xz, pw[0], pw[2], dSide, ns);
        if (dTop <= dSide) {
            if (dTop > pen) { pen = dTop; n[0] = -sp.gx * ny; n[1] = ny; n[2] = -sp.gz * ny; }
        } else {
            if (dSide > pen) { pen = dSide; n[0] = ns[0]; n[1] = 0.0; n[2] = ns[1]; }
        }
    }
}

Bivector Vessel::GroundAtStation(const Section& st, const double* px, const double* py, int n,
                                 const Medium& med, const SurfaceSample& ss) {
    Bivector w = Bivector::Zero();
    {
    // ---- THE BED, under the same station (beaching, 2026-10-11) -------------------------------
    // The second surface. The sample carries the bed as a DATUM height and the water as a
    // plane in the hull's frame (PlaneOf: the sphere under a flat frame); the bed in that
    // frame is the water's plane point dropped by the depth, so both surfaces move together
    // with the frame and nothing differences a planetary number. The normal is the hull's
    // one bed plane (BedPlane, the secant over the hull); the height is this station's own.
    // Every outline point below it is pushed back along it: a spring whose rate the hull's
    // weight sets (Build), damped critically on the way in, and Coulomb friction across it.
    //
    // The friction is Coulomb's, as a GRIP (Vessel.h, the anchors): a tangential spring from
    // the point to the ground point it gripped, the bed's own rate and damping, saturated at
    // mu N by the friction cone -- and the anchor dragged onto the cone when it saturates, so
    // a sliding hull feels exactly mu N opposite its motion and a standing one feels what
    // holds it, to the spring's deflection. A point that lifts lets go of its anchor.
    //
    // The water keeps acting as it does: the clip below sees less water as the hull rides
    // up, buoyancy falls, the bed carries what it dropped, friction holds it. That is a
    // beaching, and nothing here reads depthM to decide anything.
    if (m_kContact > 0.0) {
        const double bedPt[3] = {med.sp[0], med.sp[1] - ss.depthM, med.sp[2]};
        for (int i = 0; i < n; ++i, ++m_anchorIdx) {
            const double pBody[3] = {px[i], py[i], st.z};
            double pw[3];
            m_body.ToWorld(pBody, pw);
            const double h = (pw[0] - bedPt[0]) * m_bedN[0] + (pw[1] - bedPt[1]) * m_bedN[1] +
                             (pw[2] - bedPt[2]) * m_bedN[2];
            // THE CONTACT PLANE: the bed's, or a solid's nearest face when the point is deeper in
            // one (sim/Solids.h) -- the same law from here on, whichever surface it is.
            double pen, nrm[3];
            DeepestSurface(pw, std::max(0.0, -h), m_bedN, pen, nrm);
            const double* n = nrm;
            double* anc = &m_anchor[3 * m_anchorIdx];
            uint8_t& on = m_anchorOn[m_anchorIdx];
            if (pen <= 0.0) { on = 0; continue; }   // the domain: a bed pushes, never pulls
            if (!on) { anc[0] = pw[0]; anc[1] = pw[1]; anc[2] = pw[2]; on = 1; }
            double v[3];
            m_body.VelocityAtBody(pBody, v);
            const double vn = v[0] * n[0] + v[1] * n[1] + v[2] * n[2];
            // HUNT-CROSSLEY (2026-10-12): the damping grows with the penetration, c (pen / settle)
            // (-vn) -- the resting contact at pen = settle keeps the damping it had, a crash ten
            // settles deep dissipates ten times as hard, and a touch starts from zero (no impulse
            // at first contact, no pull at the exit). Measured before it: a car on its roof at
            // 20 kn took 135 kN on 10 points and left the ground with its CG 7 m up.
            const double fn = (std::max)(0.0, m_kContact * pen +
                                              m_cContact * (pen / m_settleContact) * (std::max)(0.0, -vn));
            // The grip: (point - anchor) and the point's velocity, both in the contact plane.
            double d[3] = {pw[0] - anc[0], pw[1] - anc[1], pw[2] - anc[2]};
            const double dn = d[0] * n[0] + d[1] * n[1] + d[2] * n[2];
            double ft[3];
            for (int k = 0; k < 3; ++k) {
                d[k] -= dn * n[k];
                const double vt = v[k] - vn * n[k];
                ft[k] = -m_kContact * d[k] - m_cContact * vt;
            }
            const double ftMag = std::sqrt(ft[0] * ft[0] + ft[1] * ft[1] + ft[2] * ft[2]);
            const double cone = std::min(1.0, m_muContact * fn / std::max(ftMag, 1e-12));
            for (int k = 0; k < 3; ++k) {
                ft[k] *= cone;
                // The anchor rides the cone: the deflection scaled onto it, in the plane.
                anc[k] = pw[k] - d[k] * cone;
            }
            const double f[3] = {fn * n[0] + ft[0], fn * n[1] + ft[1], fn * n[2] + ft[2]};
            if (s_gripTrace) {
                Log("[grip] pt %zu pen %.5f fn %.0f |d| %.5f |ft| %.0f cone %.3f v (%+.4f %+.4f %+.4f) anc (%+.4f %+.4f %+.4f) pw (%+.4f %+.4f %+.4f)",
                    m_anchorIdx, pen, fn, std::sqrt(d[0]*d[0]+d[1]*d[1]+d[2]*d[2]), ftMag, cone, v[0], v[1], v[2],
                    anc[0], anc[1], anc[2], pw[0], pw[1], pw[2]);
            }
            w += m_body.BodyWrench(f, pBody);
            m_tel.contactN += fn;
            ++m_tel.contactPts;
        }
    } else {
        m_anchorIdx += static_cast<size_t>(n);
    }

    }
    return w;
}

// ================================================================================================
//  Shell -- a body's skin on the ground: the stations' outline points against the bed, with the
//  grip, and no water clip. A rolled car lies on its roof; a boat's would be its deck. The same
//  GroundAtStation as Buoyancy's, so the two cannot disagree about the ground.
// ================================================================================================
Bivector Vessel::Shell(const Element& e, const WaterSurface& sea, double simUnix) {
    Bivector w = Bivector::Zero();
    for (size_t i = 0; i < e.stations.size(); ++i) {
        const Section& st = e.stations[i];
        double px[64], py[64];
        const int n = FullSection(st, px, py, 64);
        const double bodyC[3] = {0.0, 0.0, st.z};
        double wc[3];
        m_body.ToWorld(bodyC, wc);
        const SurfaceSample ss = sea.At(wc[0], wc[2], simUnix);
        if (!ss.valid) {
            m_tel.waterValid = false;
            m_anchorIdx += static_cast<size_t>(n);
            continue;
        }
        const Medium med = sea.WaterFrom(ss, wc[0], wc[2]);
        w += GroundAtStation(st, px, py, n, med, ss);
    }
    return w;
}

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

    // ---- THE FORMS. A spec without any has one: every part engaged, the spec's own numbers.
    if (m_spec.forms.empty()) {
        Form f;
        f.name = "the one form";
        f.kind = m_spec.kind;
        f.display = m_spec.display;
        f.engage.assign(m_spec.elements.size(), 1.0);
        f.loa = m_spec.loa; f.beam = m_spec.beam; f.draftStatic = m_spec.draftStatic;
        f.massDry = m_spec.massDry; f.massLoaded = m_spec.massLoaded;
        for (int i = 0; i < 3; ++i) f.hullMassCentre[i] = m_spec.hullMassCentre[i];
        f.inertiaRoll = m_spec.inertiaRoll; f.inertiaPitch = m_spec.inertiaPitch; f.inertiaYaw = m_spec.inertiaYaw;
        f.addedMassSurge = m_spec.addedMassSurge; f.addedMassSway = m_spec.addedMassSway; f.addedMassHeave = m_spec.addedMassHeave;
        f.addedInertiaRoll = m_spec.addedInertiaRoll; f.addedInertiaPitch = m_spec.addedInertiaPitch; f.addedInertiaYaw = m_spec.addedInertiaYaw;
        f.seatHeight = m_spec.seatHeight;
        f.recovery = m_spec.recovery;
        m_spec.forms.push_back(f);
    }
    for (const Form& f : m_spec.forms) {
        if (f.engage.size() != m_spec.elements.size()) {
            Log("[vessel] spec '%s': form '%s' engages %zu parts of %zu -- refusing (a form is a weight on EVERY part)",
                m_spec.kind.c_str(), f.name.c_str(), f.engage.size(), m_spec.elements.size());
            return false;
        }
    }
    m_form = 0;
    m_formCg.clear();
    {
        for (const Form& f : m_spec.forms) {
            double mom[3] = {0.0, 0.0, 0.0};
            double located = 0.0;
            for (size_t i = 0; i < m_spec.elements.size(); ++i) {
                const Element& e = m_spec.elements[i];
                const double g = f.engage[i];
                if (!(e.mass.v > 0.0) || g <= 0.0) continue;
                double at[3];
                e.mount.Origin(0.0, at);
                for (int k = 0; k < 3; ++k) mom[k] += g * e.mass.v * at[k];
                located += g * e.mass.v;
            }
            const double total = (f.massLoaded.v > 0.0) ? f.massLoaded.v : f.massDry.v;
            const double hullM = total - located;
            if (hullM < 0.0) {
                Log("[vessel] spec '%s' form '%s': located element masses (%.0f kg) exceed the loaded "
                    "displacement (%.0f kg) -- refusing, because the remainder would be a NEGATIVE "
                    "hull and it would float upside down rather than fail",
                    m_spec.kind.c_str(), f.name.c_str(), located, total);
                return false;
            }
            for (int k = 0; k < 3; ++k) mom[k] += hullM * f.hullMassCentre[k];
            const double sumM = located + hullM;
            std::array<double, 3> cg = {0.0, 0.0, 0.0};
            if (sumM > 1e-9) for (int k = 0; k < 3; ++k) cg[k] = mom[k] / sumM;
            m_formCg.push_back(cg);
            Log("[vessel] %s: form '%s' CG solved at (%+.3f, %+.3f, %+.3f) m from the geometry datum -- "
                "%.0f kg located in elements, %.0f kg hull",
                m_spec.kind.c_str(), f.name.c_str(), cg[0], cg[1], cg[2], located, hullM);
        }
        // Re-reference the geometry onto the FIRST form's CG (the body origin); SetForm moves it.
        const double cg0[3] = {m_formCg[0][0], m_formCg[0][1], m_formCg[0][2]};
        ShiftGeometry(cg0);
        for (int i = 0; i < 3; ++i) m_cgNow[i] = m_spec.cgFromTransom[i] = cg0[i];
    }

    // (The transom, the aftmost station, is found by ShiftGeometry wherever the origin stands.)

    m_body = RigidBody{};
    // The body's numbers are the form's: ApplyFormNumbers, after the anchors below are sized.
    m_body.SetPose(pose);
    m_tel = VesselTelemetry{};

    // ---- THE ANCHORS AND THE WHEEL SLOTS, per part: a Buoyancy part owns one grip anchor per
    // outline point, a Wheel part one anchor and one wheel slot; each part's first slot is kept so
    // a form that skips parts leaves every other part's grips where they were.
    m_anchorBase.assign(m_spec.elements.size(), 0);
    m_wheelBase.assign(m_spec.elements.size(), 0);
    {
        size_t anchors = 0, wheels = 0;
        for (size_t i = 0; i < m_spec.elements.size(); ++i) {
            const Element& el = m_spec.elements[i];
            m_anchorBase[i] = anchors;
            m_wheelBase[i] = wheels;
            if (el.kind == ElementKind::Buoyancy || el.kind == ElementKind::Shell) {
                for (const Section& st : el.stations) {
                    double px[64], py[64];
                    anchors += static_cast<size_t>(FullSection(st, px, py, 64));
                }
            } else if (el.kind == ElementKind::Wheel) {
                ++anchors;
                ++wheels;
            }
        }
        m_anchor.assign(3 * anchors, 0.0);
        m_anchorOn.assign(anchors, 0);
        m_wheels.assign(wheels, WheelState{});
    }
    ApplyFormNumbers(m_spec.forms[m_form]);
    m_body.SetPose(pose);
    m_tel = VesselTelemetry{};
    return true;
}

void Vessel::ShiftGeometry(const double d[3]) {
    // Stations carry (x, y) outlines at a longitudinal z; mounts carry a motor and, when jointed,
    // a line to swing about; the tubes their own offsets. Every one of them minus d.
    for (Element& e : m_spec.elements) {
        for (Section& st : e.stations) {
            st.z -= d[2];
            for (double& y : st.oy) y -= d[1];
            for (double& x : st.ox) x -= d[0];
        }
        e.mount.at = Motor::Translation(-d[0], -d[1], -d[2]) * e.mount.at;
        if (e.mount.jointed) for (int i = 0; i < 3; ++i) e.mount.axisP[i] -= d[i];
        e.tubeZ0.v -= d[2];
        e.tubeZ1.v -= d[2];
        e.tubeXOffset.v -= d[0];
        e.tubeYOffset.v -= d[1];
    }
    // The transom: the aftmost station, where it is now.
    m_transomZ = 0.0;
    for (const Element& el : m_spec.elements) {
        for (const Section& st : el.stations) m_transomZ = (std::min)(m_transomZ, st.z);
    }
}

void Vessel::ApplyFormNumbers(const Form& f) {
    // Spec()'s top level reads as the form: the ledger, the seat, the reach, the logs.
    m_spec.kind = f.kind;
    m_spec.display = f.display;
    m_spec.loa = f.loa; m_spec.beam = f.beam; m_spec.draftStatic = f.draftStatic;
    m_spec.massDry = f.massDry; m_spec.massLoaded = f.massLoaded;
    for (int i = 0; i < 3; ++i) m_spec.hullMassCentre[i] = f.hullMassCentre[i];
    m_spec.inertiaRoll = f.inertiaRoll; m_spec.inertiaPitch = f.inertiaPitch; m_spec.inertiaYaw = f.inertiaYaw;
    m_spec.addedMassSurge = f.addedMassSurge; m_spec.addedMassSway = f.addedMassSway; m_spec.addedMassHeave = f.addedMassHeave;
    m_spec.addedInertiaRoll = f.addedInertiaRoll; m_spec.addedInertiaPitch = f.addedInertiaPitch; m_spec.addedInertiaYaw = f.addedInertiaYaw;
    m_spec.seatHeight = f.seatHeight;
    m_spec.recovery = f.recovery;

    m_body.mp.mass = (f.massLoaded.v > 0.0) ? f.massLoaded.v : f.massDry.v;
    // AXES. A moment of inertia is named for the motion it resists, and that is the axis it
    // is taken ABOUT -- so ROLL is about the LONGITUDINAL axis (body z, forward) and PITCH is
    // about the TRANSVERSE one (body x, starboard). Written the other way round first, which the
    // roll-period gate catches because a hull's roll and pitch inertias are nothing alike.
    for (int i = 0; i < 3; ++i) for (int j = 0; j < 3; ++j) m_body.mp.I[i][j] = 0.0;
    m_body.mp.I[0][0] = f.inertiaPitch.v;    // about body x (starboard) = PITCH
    m_body.mp.I[1][1] = f.inertiaYaw.v;      // about body y (up)        = YAW
    m_body.mp.I[2][2] = f.inertiaRoll.v;     // about body z (forward)   = ROLL
    // Keep the FULL-IMMERSION figures; Step scales them by how wet the hull actually is.
    m_addedM0[0] = f.addedMassSway.v;
    m_addedM0[1] = f.addedMassHeave.v;
    m_addedM0[2] = f.addedMassSurge.v;
    m_addedI0[0] = f.addedInertiaPitch.v;
    m_addedI0[1] = f.addedInertiaYaw.v;
    m_addedI0[2] = f.addedInertiaRoll.v;
    for (int i = 0; i < 3; ++i) {
        m_body.mp.addedM[i] = m_addedM0[i];
        m_body.mp.addedI[i] = m_addedI0[i];
    }
    m_dispVol = std::max(m_body.mp.mass / 1025.0, 1e-6);

    // ---- THE BED'S SPRING, from the hull (beaching). A penalty contact needs a rate, and the
    // only honest one is the hull's own: the stiffness that lets its weight sink `contactSettle`
    // into the ground on the points that meet flat ground first -- each station's lowest outline
    // points (the keel of a V, the three bottom corners of a barge section). Spread over those,
    // flat ground compresses exactly the settle; a heeled hull on its chine finds more points
    // and a stiffer answer, continuously. Damped against each point's share of the mass, so a
    // landing thumps and does not ring. The first ENGAGED Buoyancy element's numbers speak for
    // the hull; a form engaging no settle has no ground through its stations.
    m_kContact = m_cContact = m_muContact = 0.0;
    m_mContact = m_body.mp.mass;
    m_kGrip = m_cGrip = 0.0;
    size_t nWheels = 0;
    const Element* by = nullptr;
    const Element* wheel = nullptr;
    for (size_t i = 0; i < m_spec.elements.size(); ++i) {
        const Element& el = m_spec.elements[i];
        if (!(f.engage[i] > 0.0)) continue;
        if ((el.kind == ElementKind::Buoyancy || el.kind == ElementKind::Shell) && !by) by = &el;
        if (el.kind == ElementKind::Wheel) {
            if (!wheel) wheel = &el;
            ++nWheels;
        }
    }
    if (by && by->contactSettle.v > 0.0) {
        int keelPts = 0;
        for (const Section& st : by->stations) {
            double px[64], py[64];
            const int n = FullSection(st, px, py, 64);
            double lo = 1e30;
            for (int i = 0; i < n; ++i) lo = (std::min)(lo, py[i]);
            for (int i = 0; i < n; ++i) if (py[i] <= lo + 1e-3) ++keelPts;
        }
        if (keelPts > 0) {
            m_mContact = m_body.mp.mass / keelPts;
            m_settleContact = by->contactSettle.v;
            m_kContact = m_mContact * kG / by->contactSettle.v;
            // 0.4 OF CRITICAL PER POINT, not 1.0. The points act TOGETHER about the roll axis
            // (a lever of the keel's depth under the CG), where their summed damping reached 1.6x
            // critical at omega dt = 0.67 and the explicit 60 Hz step crossed its limit
            // (2 zeta omega dt = 2.1 > 2): measured as a period-two rock that saturated the cone on
            // every fast half-cycle and crept a held hull 0.037 m/s down a 20 deg slope.
            m_cContact = 0.8 * std::sqrt(m_kContact * m_mContact);
            m_muContact = (std::max)(0.0, by->contactMu.v);
            Log("[vessel] %s: ground contact on %d keel points: %.3g N/m and %.3g N s/m each "
                "(settle %.3f m, mu %.2f)", m_spec.kind.c_str(), keelPts, m_kContact, m_cContact,
                by->contactSettle.v, m_muContact);
        }
    }
    // ...and the wheels' grip: each braked tyre's hold, its share of the weight over the wheel
    // element's contactSettle, 0.4 of critical as the hull's.
    m_mWheel = (nWheels > 0) ? m_body.mp.mass / double(nWheels) : m_body.mp.mass;
    if (wheel && nWheels > 0 && wheel->contactSettle.v > 0.0) {
        m_kGrip = m_mWheel * kG / wheel->contactSettle.v;
        m_cGrip = 0.8 * std::sqrt(m_kGrip * m_mWheel);
        Log("[vessel] %s: %zu wheels, the braked tyre's grip %.3g N/m and %.3g N s/m each (settle %.3f m)",
            m_spec.kind.c_str(), nWheels, m_kGrip, m_cGrip, wheel->contactSettle.v);
    }
    m_groundAsks = (m_kContact > 0.0) || (nWheels > 0);
}

bool Vessel::SetForm(size_t f) {
    if (f >= m_spec.forms.size()) return false;
    if (f == m_form) return true;
    // The body origin is the CG by construction: the new form's CG is another point of the datum,
    // so the geometry is re-referenced onto it and the pose carried by the same shift -- the new
    // origin is the body point d of the frame it leaves -- and every part stands where it stood.
    const double d[3] = {m_formCg[f][0] - m_cgNow[0], m_formCg[f][1] - m_cgNow[1], m_formCg[f][2] - m_cgNow[2]};
    ShiftGeometry(d);
    m_body.pose = m_body.pose * Motor::Translation(d[0], d[1], d[2]);
    for (int i = 0; i < 3; ++i) m_cgNow[i] = m_spec.cgFromTransom[i] = m_formCg[f][i];
    const std::string was = m_spec.forms[m_form].name;
    m_form = f;
    ApplyFormNumbers(m_spec.forms[f]);
    // The discrete switch: the grips let go, the planing state and the momenta rested (a body
    // whose mass changed under its momentum would jump; the continuous transformer's parts move
    // along screws with the momenta transported -- the follow-on).
    std::fill(m_anchorOn.begin(), m_anchorOn.end(), 0);
    m_lambda = 1.5;
    m_body.Rest();
    m_tel = VesselTelemetry{};
    Log("[vessel] form '%s' -> '%s' (%s): the origin moved (%+.3f, %+.3f, %+.3f) m to the new CG, %.0f kg",
        was.c_str(), m_spec.forms[f].name.c_str(), m_spec.kind.c_str(), d[0], d[1], d[2], m_body.mp.mass);
    return true;
}

double Vessel::KeelY() const {
    double keel = 0.0;
    for (size_t i = 0; i < m_spec.elements.size(); ++i) {
        const Element& el = m_spec.elements[i];
        if (!(Engage(i) > 0.0)) continue;
        for (const Section& st : el.stations) {
            for (double y : st.oy) keel = (std::min)(keel, y);
        }
        if (el.kind == ElementKind::Wheel) {
            double top[3];
            el.mount.Origin(0.0, top);
            keel = (std::min)(keel, top[1] - el.suspRest.v - el.wheelRadius.v);
        }
    }
    return keel;
}

void Vessel::Carry(const Motor& K) {
    m_body.Carry(K);
    for (size_t i = 0; i < m_anchorOn.size(); ++i) {
        if (!m_anchorOn[i]) continue;
        double* a = &m_anchor[3 * i];
        K.TransformPoint(a[0], a[1], a[2]);
    }
}

// ================================================================================================
//  The bed plane (beaching): the secant over the hull, one normal per wrench.
// ================================================================================================
void Vessel::BedProbePoints(double xz[6]) const {
    const double cg[3] = {0.0, 0.0, 0.0};
    double wc[3];
    m_body.ToWorld(cg, wc);
    const double h = (m_spec.loa.v > 0.0) ? 0.5 * m_spec.loa.v : 1.0;
    xz[0] = wc[0];     xz[1] = wc[2];
    xz[2] = wc[0] + h; xz[3] = wc[2];
    xz[4] = wc[0];     xz[5] = wc[2] + h;
}

void Vessel::BedPlane(const WaterSurface& sea, double simUnix) {
    m_bedN[0] = 0.0; m_bedN[1] = 1.0; m_bedN[2] = 0.0;
    if (!m_groundAsks) return;
    double xz[6];
    BedProbePoints(xz);
    const double h = xz[2] - xz[0];
    // The bed's height in the hull's FRAME at each probe: the water's plane point there
    // (PlaneOf, the sphere under the flat frame) less the depth -- the same construction the
    // stations use for their own bed point, so the plane and the points agree.
    double b[3];
    bool ok[3];
    for (int i = 0; i < 3; ++i) {
        const SurfaceSample s = sea.At(xz[2 * i], xz[2 * i + 1], simUnix);
        ok[i] = s.valid;
        double sp[3], n[3];
        sea.PlaneOf(s, xz[2 * i], xz[2 * i + 1], sp, n);
        b[i] = sp[1] - s.depthM;
    }
    if (!ok[0]) return;
    // A probe no layer covers (the edge of coverage) reads as the CG's height: a level bed in
    // that direction rather than a slope invented from nothing.
    const double dx = ok[1] ? (b[1] - b[0]) / h : 0.0;
    const double dz = ok[2] ? (b[2] - b[0]) / h : 0.0;
    const double inv = 1.0 / std::sqrt(1.0 + dx * dx + dz * dz);
    m_bedN[0] = -dx * inv;
    m_bedN[1] = inv;
    m_bedN[2] = -dz * inv;
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

// ================================================================================================
//  Hover - the recovery drone. Two servos and no fluid.
//
//  The force is m (g up + (v_cmd - v) / tau) at the CG: the weight carried, and the velocity
//  relaxed to the command with one time constant. The torque is a PD on the attitude bivector:
//  the axis that turns body-up onto world-up (their cross product, |e| = sin of the tilt) with
//  gain 1/tau^2, and the rate error against the commanded yaw rate about world up with gain
//  2/tau -- critically damped, each axis scaled by its own inertia so the three settle alike.
//  Written in the body frame, which is the wrench's frame, so no planetary number appears.
// ================================================================================================
Bivector Vessel::Hover(const Element& e, const VesselControls& c) {
    const double tau = (e.hoverTau.v > 1e-3) ? e.hoverTau.v : 0.4;
    // The helm's channels, as the drone reads them: W/S = both levers (demand), Q/E = their split.
    const double demand = 0.5 * (c.throttle[0] + c.throttle[1]);
    const double split = 0.5 * (c.throttle[1] - c.throttle[0]);
    const double sHi = (e.mount.hi > 0.0) ? e.mount.hi : 1.0;
    const double tHi = (e.mount.tiltHi > 0.0) ? e.mount.tiltHi : 1.0;
    // THE HELM'S STEER IS THE OUTBOARD'S ANGLE: D commands a NEGATIVE angle (Entity::Helm, the
    // linkage kicks the stern the other way). A drone has no stern: D must turn it RIGHT, which is
    // a positive rate about +y (Motor::Rotation takes +z toward +x). Hence the sign here.
    const double yawCmd = -std::clamp(c.steer / sHi, -1.0, 1.0) * e.hoverYawRate.v;
    const double climb = std::clamp(c.tilt / tHi, -1.0, 1.0) * e.hoverSpeedV.v;

    // The heading in the world, horizontal; its right-hand neighbour; world up.
    double fwd[3] = {0.0, 0.0, 1.0};
    m_body.pose.TransformDir(fwd[0], fwd[1], fwd[2]);
    const double fl = std::sqrt(fwd[0] * fwd[0] + fwd[2] * fwd[2]);
    const double fx = (fl > 1e-9) ? fwd[0] / fl : 0.0, fz = (fl > 1e-9) ? fwd[2] / fl : 1.0;
    const double rx = fz, rz = -fx;   // right = up x forward, in (x east, y up, z north)
    const double vCmd[3] = {(fx * demand + rx * split) * e.hoverSpeedH.v, climb,
                            (fz * demand + rz * split) * e.hoverSpeedH.v};
    const double cg[3] = {0.0, 0.0, 0.0};
    double v[3];
    m_body.VelocityAtBody(cg, v);
    const double m = m_body.mp.mass;
    const double f[3] = {m * (vCmd[0] - v[0]) / tau, m * (kG + (vCmd[1] - v[1]) / tau),
                         m * (vCmd[2] - v[2]) / tau};
    Bivector w = m_body.BodyWrench(f, cg);

    // Attitude: world up in the body frame; the error axis e = y_body x up; the commanded rate.
    double u[3] = {0.0, 1.0, 0.0};
    m_body.pose.Inverse().TransformDir(u[0], u[1], u[2]);
    const double err[3] = {u[2], 0.0, -u[0]};
    const Bivector& t = m_body.Twist();
    double tauB[3];
    for (int i = 0; i < 3; ++i) {
        const double wCmd = yawCmd * u[i];
        tauB[i] = m_body.mp.I[i][i] * (err[i] / (tau * tau) + 2.0 * (wCmd - t.a[i]) / tau);
    }
    w += Bivector::Couple(tauB);
    return w;
}

// ================================================================================================
//  Wheel - the car's corner: a strut cast onto the bed, and a tyre at its foot.
//
//  The raycast vehicle, in this engine's terms (it is what the Warthog is). The strut is a LINE
//  down the mount's -y; the kingpin is the mount's joint line, so a steered wheel is Attach::Posed
//  like a steered outboard -- one primitive. The ground is the hull's bed plane (BedPlane: the
//  secant over the body) with this wheel's own sample for its height, exactly the stations' law.
//  Where the strut meets it is the contact; the spring, the damper, the drive, the rolling
//  resistance, the brake's grip and the lateral slip law all act THERE, in the ground's plane,
//  through one friction cone. Nothing here reads depthM: under water the wheel finds the same bed.
// ================================================================================================
Bivector Vessel::Wheel(const Element& e, const WaterSurface& sea, const VesselControls& c, double simUnix) {
    const size_t wi = m_wheelIdx++;
    const size_t ai = m_anchorIdx++;
    if (wi >= m_wheels.size() || ai >= m_anchorOn.size()) return Bivector::Zero();
    WheelState& ws = m_wheels[wi];
    uint8_t& on = m_anchorOn[ai];
    double* anc = &m_anchor[3 * ai];
    const double R = e.wheelRadius.v;
    const double droop = e.suspRest.v;
    // THE HELM'S STEER IS THE OUTBOARD'S ANGLE (Entity::Helm): D commands a negative angle because
    // the leg kicks the stern the other way. A wheel turns the car the way it points, so D must be
    // a positive angle about +y here. The same sign the drone's yaw takes.
    const double q = e.mount.jointed ? -c.steer : 0.0;
    const Motor posed = e.mount.Posed(q, 0.0);
    double top[3] = {0.0, 0.0, 0.0}, dB[3] = {0.0, -1.0, 0.0}, fB[3] = {0.0, 0.0, 1.0};
    posed.TransformPoint(top[0], top[1], top[2]);
    posed.TransformDir(dB[0], dB[1], dB[2]);
    posed.TransformDir(fB[0], fB[1], fB[2]);
    ws.steer = q;
    ws.load = 0.0;
    ws.comp = 0.0;
    for (int k = 0; k < 3; ++k) ws.centre[k] = top[k] + dB[k] * droop;   // full droop until the ground says
    double topW[3], dW[3] = {dB[0], dB[1], dB[2]}, fW[3] = {fB[0], fB[1], fB[2]};
    m_body.ToWorld(top, topW);
    m_body.pose.TransformDir(dW[0], dW[1], dW[2]);
    m_body.pose.TransformDir(fW[0], fW[1], fW[2]);
    const SurfaceSample ss = sea.At(topW[0], topW[2], simUnix);
    if (!ss.valid) {
        on = 0;
        m_tel.waterValid = false;
        return Bivector::Zero();
    }
    // The bed under this wheel in the body's frame: the water's plane point less the depth (the
    // stations' own construction), the normal the body's one bed plane.
    double sp[3], sn[3];
    sea.PlaneOf(ss, topW[0], topW[2], sp, sn);
    const double bedPt[3] = {sp[0], sp[1] - ss.depthM, sp[2]};
    double nn[3] = {m_bedN[0], m_bedN[1], m_bedN[2]};
    const double* n = nn;
    double dn = dW[0] * n[0] + dW[1] * n[1] + dW[2] * n[2];
    if (dn >= -1e-6) {   // the strut does not point at the ground (the car is on its side or roof)
        on = 0;
        return Bivector::Zero();
    }
    double t = ((bedPt[0] - topW[0]) * n[0] + (bedPt[1] - topW[1]) * n[1] + (bedPt[2] - topW[2]) * n[2]) / dn;
    // THE SOLIDS' TOPS (sim/Solids.h): the ray meets a roof or a deck before the bed when the strut
    // top is above it and the hit lies in its footprint -- the car drives onto it.
    if (m_solids && dW[1] < -1e-6) {
        for (const SolidPrism& sp : *m_solids) {
            // The ray against the top PLANE: f(t) = y(t) - top(x(t), z(t)) = 0.
            const double f0 = topW[1] - sp.TopAt(topW[0], topW[2]);
            const double df = dW[1] - sp.gx * dW[0] - sp.gz * dW[2];
            if (f0 <= 0.0 || df >= -1e-9) continue;   // the strut top is under the plane, or runs along it
            const double tp = f0 / (-df);
            if (tp >= t) continue;
            const double hx = topW[0] + dW[0] * tp, hz = topW[2] + dW[2] * tp;
            const double ddx = hx - sp.cx, ddz = hz - sp.cz;
            if (ddx * ddx + ddz * ddz > sp.radius * sp.radius) continue;
            if (!PrismHolds(sp.xz, hx, hz)) continue;
            t = tp;
            const double ny = 1.0 / std::sqrt(1.0 + sp.gx * sp.gx + sp.gz * sp.gz);
            nn[0] = -sp.gx * ny; nn[1] = ny; nn[2] = -sp.gz * ny;
            dn = dW[0] * nn[0] + dW[1] * nn[1] + dW[2] * nn[2];
        }
    }
    // Bounded by the strut itself: the wheel cannot pass its top. A strut grazing the ground plane
    // (dn -> 0: the car on its side) made t enormous and the cubic spring 10^12 N; the shell
    // carries a rolled car, the strut only what a strut can.
    const double comp = std::clamp(droop + R - t, 0.0, droop);
    if (comp <= 0.0) {   // the domain: a tyre in the air carries nothing and grips nothing
        on = 0;
        return Bivector::Zero();
    }
    for (int k = 0; k < 3; ++k) ws.centre[k] = top[k] + dB[k] * (t - R);
    // THE SPRING IS PROGRESSIVE: k x (1 + (x / travel)^2). The bump stop is the law's own cube, not
    // a second spring at a threshold. Damped on the strut's rate -- the top's velocity along the
    // normal over the ray's cosine -- and it pushes, never pulls.
    double vTop[3];
    m_body.VelocityAtBody(top, vTop);
    const double rate = (vTop[0] * n[0] + vTop[1] * n[1] + vTop[2] * n[2]) / dn;
    const double travel = (e.suspTravel.v > 1e-3) ? e.suspTravel.v : 0.3;
    const double fn = (std::max)(0.0, e.suspK.v * comp * (1.0 + (comp / travel) * (comp / travel)) + e.suspC.v * rate);
    // The contact point, and the tyre's frame there: rolling direction in the ground plane, lateral
    // across it (n x forward).
    const double pcB[3] = {top[0] + dB[0] * t, top[1] + dB[1] * t, top[2] + dB[2] * t};
    double pcW[3];
    m_body.ToWorld(pcB, pcW);
    const double fdn = fW[0] * n[0] + fW[1] * n[1] + fW[2] * n[2];
    double fL[3] = {fW[0] - fdn * n[0], fW[1] - fdn * n[1], fW[2] - fdn * n[2]};
    const double fl = std::sqrt(fL[0] * fL[0] + fL[1] * fL[1] + fL[2] * fL[2]);
    if (fl < 1e-9) {   // the tyre points straight at the ground: it carries, it cannot roll
        on = 0;
        const double fOnly[3] = {n[0] * fn, n[1] * fn, n[2] * fn};
        return m_body.BodyWrench(fOnly, pcB);
    }
    for (int k = 0; k < 3; ++k) fL[k] /= fl;
    const double lat[3] = {n[1] * fL[2] - n[2] * fL[1], n[2] * fL[0] - n[0] * fL[2], n[0] * fL[1] - n[1] * fL[0]};
    double vc[3];
    m_body.VelocityAtBody(pcB, vc);
    const double vL = vc[0] * fL[0] + vc[1] * fL[1] + vc[2] * fL[2];
    const double vLat = vc[0] * lat[0] + vc[1] * lat[1] + vc[2] * lat[2];
    const double v0 = 0.3;   // m/s: the slip angle's and the rolling sign's softening at rest
    // DRIVE: the one lever (throttle[0]), falling to zero at driveSpeed in the direction it pushes
    // -- the thruster's shape, the honest first form of a drivetrain's curve.
    const double thr = c.throttle[0];
    const double vMax = (e.driveSpeed.v > 0.1) ? e.driveSpeed.v : 30.0;
    const double along = (thr >= 0.0) ? vL : -vL;
    const double fDrive = thr * e.driveForce.v * e.driven.v * (std::max)(0.0, 1.0 - (std::max)(along, 0.0) / vMax);
    const double fRoll = -e.rollRes.v * fn * vL / std::sqrt(vL * vL + v0 * v0);
    // THE PATCH GRIPS A GROUND POINT (Vessel.h, the anchors) -- in both directions, by two laws
    // that are one mechanism. Along the rolling direction the BRAKE (the helm's tilt channel,
    // 0..1) scales the hold, so a braked car stands on a hill instead of creeping down it a step a
    // frame; released, the anchor follows the contact. Across it the patch is the tyre's carcass:
    // a spring of tyreCorner x N over the relaxation length sigma, and the anchor relaxes toward
    // the contact by |vL| dt / sigma as the patch rolls through. At rest that spring holds a
    // parked car across a camber; rolling, the deflection settles at sigma vLat / |vL| and the
    // force at -tyreCorner x N x (slip angle): the slip law is this law's limit, not a second one.
    // (The slip law written as a velocity law chattered at 60 Hz -- its slope at rest was 3.5x the
    // explicit limit -- saturated the cone at 0.2 and let the brake go: the car crept at 0.37 m/s.)
    const double b = std::clamp(c.tilt, 0.0, 1.0);
    if (!on) {
        for (int k = 0; k < 3; ++k) anc[k] = pcW[k];
        on = 1;
    }
    const double dL = (pcW[0] - anc[0]) * fL[0] + (pcW[1] - anc[1]) * fL[1] + (pcW[2] - anc[2]) * fL[2];
    const double dLat = (pcW[0] - anc[0]) * lat[0] + (pcW[1] - anc[1]) * lat[1] + (pcW[2] - anc[2]) * lat[2];
    const double fBrake = -b * (m_kGrip * dL + m_cGrip * vL);
    const double sigma = (e.tyreRelax.v > 1e-3) ? e.tyreRelax.v : 0.3;
    const double relax = (std::min)(1.0, std::abs(vL) * m_dt / sigma);
    const double kLat = e.tyreCorner.v * fn / sigma;
    // Damped at 0.4 of critical against the wheel's share of the mass while the patch stands, the
    // damping rolling away with the relaxation (a rolling patch's force is its deflection's).
    const double cLat = 0.8 * std::sqrt(kLat * m_mWheel) * (1.0 - relax);
    const double fLat = -(kLat * dLat + cLat * vLat);
    double ft[3];
    for (int k = 0; k < 3; ++k) ft[k] = fL[k] * (fDrive + fRoll + fBrake) + lat[k] * fLat;
    const double ftMag = std::sqrt(ft[0] * ft[0] + ft[1] * ft[1] + ft[2] * ft[2]);
    const double cone = (std::min)(1.0, e.tyreMu.v * fn / (std::max)(ftMag, 1e-12));
    for (int k = 0; k < 3; ++k) {
        ft[k] *= cone;
        // The anchor rides the cone (along, by the pedal; across, always) and relaxes across.
        anc[k] = pcW[k] - fL[k] * dL * cone * b - lat[k] * dLat * cone * (1.0 - relax);
    }
    const double f[3] = {n[0] * fn + ft[0], n[1] * fn + ft[1], n[2] * fn + ft[2]};
    if (s_gripTrace) {
        Log("[grip] wheel %zu comp %.4f rate %+.4f fn %.0f | vL %+.4f vLat %+.4f dL %+.5f dLat %+.5f relax %.3f | drive %+.0f roll %+.0f brake %+.0f lat %+.0f cone %.3f",
            wi, comp, rate, fn, vL, vLat, dL, dLat, relax, fDrive, fRoll, fBrake, fLat, cone);
    }
    m_tel.contactN += fn;
    ++m_tel.contactPts;
    ws.load = fn;
    ws.comp = comp;
    return m_body.BodyWrench(f, pcB);
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
    m_tel.contactN = 0.0;
    m_tel.contactPts = 0;
    // (Each part's anchor and wheel slots are its own: m_anchorBase / m_wheelBase, set per part.)
    // The bed under the hull, before the stations ask it (beaching): one plane per wrench.
    BedPlane(sea, simUnix);

    // TWO PASSES. Resistance and added mass are both properties of the WETTED HULL, and the
    // wetted hull is not known until the buoyancy clip and the collar have run. Letting that
    // depend on where a spec happens to list its elements is the kind of coupling that works
    // until somebody reorders a table, so the volume is established first and read second.
    // EVERY PART, BY ITS FORM'S WEIGHT (VesselSpec.h Form): a part at weight zero asks nothing
    // and adds nothing -- zero times its wrench, which its own law would make zero anyway once
    // its mount is where the form folds it; each part's grips start at its own anchor slot.
    for (size_t ei = 0; ei < m_spec.elements.size(); ++ei) {
        const Element& e = m_spec.elements[ei];
        const double g = Engage(ei);
        if (!(g > 0.0)) continue;
        m_anchorIdx = m_anchorBase[ei];
        switch (e.kind) {
            case ElementKind::Buoyancy:
                w += Buoyancy(e, sea, simUnix) * g;
                break;
            case ElementKind::Shell:
                w += Shell(e, sea, simUnix) * g;
                break;
            case ElementKind::Collar:
                w += Collar(e, sea, simUnix) * g;
                break;
            default:
                break;
        }
    }
    // How much hull is in the water, as a fraction of what it displaces at rest: 1 at rest by
    // construction, 0 airborne, and clamped at 1 when swamped.
    m_wetFrac = std::clamp(m_accVol / m_dispVol, 0.0, 1.0);

    int thrusterIdx = 0;
    for (size_t ei = 0; ei < m_spec.elements.size(); ++ei) {
        const Element& e = m_spec.elements[ei];
        const double g = Engage(ei);
        if (!(g > 0.0)) continue;
        m_anchorIdx = m_anchorBase[ei];
        m_wheelIdx = m_wheelBase[ei];
        switch (e.kind) {
            case ElementKind::Drag:
                w += Drag(e, sea, simUnix) * g;
                break;
            case ElementKind::Planing:
                w += Planing(e, sea, simUnix) * g;
                break;
            case ElementKind::Foil:
                w += Foil(e, sea, c, simUnix) * g;
                break;
            case ElementKind::Hover:
                w += Hover(e, c) * g;
                break;
            case ElementKind::Wheel:
                w += Wheel(e, sea, c, simUnix) * g;
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
                w += m_body.BodyWrench(fWorld, atBody) * g;
                // Prop torque reaction: a right-hand wheel rolls the hull to port under power.
                // Computed FROM `rotation`, so a counter-rotating pair cancels without anyone
                // writing a rule that says it should.
                Bivector couple;
                couple.b[2] = -e.rotation * std::abs(mag) * e.propTorqueArm.v;
                w += couple * g;
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
bool Vessel::s_gripTrace = false;
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
    // The plane is the sea's own (WaterSurface::PlaneOf): the table serves answers, it does not
    // flatten them -- the hull floated a metre over its water through this wrapper's default.
    void PlaneOf(const SurfaceSample& s, double wx, double wz, double sp[3], double n[3]) const override {
        m_sea.PlaneOf(s, wx, wz, sp, n);
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
    for (size_t ei = 0; ei < m_spec.elements.size(); ++ei) {
        const Element& e = m_spec.elements[ei];
        if (!(Engage(ei) > 0.0)) continue;   // a part at weight zero asks no water
        switch (e.kind) {
            case ElementKind::Buoyancy:
            case ElementKind::Shell:
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
            case ElementKind::Wheel: {
                // The strut top's (x, z): the kingpin passes through it, so the steer moves nothing.
                double atBody[3];
                e.mount.Origin(e.mount.jointed ? -c.steer : 0.0, atBody);
                add(atBody);
                break;
            }
            default:
                break;
        }
    }
    const double cg[3] = {0.0, 0.0, 0.0};
    add(cg);
    // ...and the bed plane's three probes (beaching), the same doubles BedPlane asks with.
    if (m_groundAsks) {
        double bp[6];
        BedProbePoints(bp);
        for (int i = 0; i < 6; ++i) xz.push_back(bp[i]);
    }
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
    add("seat.height", seatHeight);
    for (const Element& e : elements) {
        const std::string p = e.name + ".";
        add((p + "tube.z0").c_str(), e.tubeZ0);
        add((p + "tube.r0").c_str(), e.tubeR0);
        add((p + "thrust.max").c_str(), e.maxThrust);
        add((p + "prop.radius").c_str(), e.propRadius);
        add((p + "foil.area").c_str(), e.foilArea);
        add((p + "deadrise").c_str(), e.deadriseDeg);
        add((p + "contact.settle").c_str(), e.contactSettle);
        add((p + "contact.mu").c_str(), e.contactMu);
        add((p + "hover.speedH").c_str(), e.hoverSpeedH);
        add((p + "hover.speedV").c_str(), e.hoverSpeedV);
        add((p + "hover.yawRate").c_str(), e.hoverYawRate);
        add((p + "hover.tau").c_str(), e.hoverTau);
        add((p + "wheel.radius").c_str(), e.wheelRadius);
        add((p + "susp.rest").c_str(), e.suspRest);
        add((p + "susp.travel").c_str(), e.suspTravel);
        add((p + "susp.k").c_str(), e.suspK);
        add((p + "susp.c").c_str(), e.suspC);
        add((p + "drive.force").c_str(), e.driveForce);
        add((p + "drive.speed").c_str(), e.driveSpeed);
        add((p + "drive.share").c_str(), e.driven);
        add((p + "tyre.mu").c_str(), e.tyreMu);
        add((p + "tyre.corner").c_str(), e.tyreCorner);
        add((p + "tyre.relax").c_str(), e.tyreRelax);
        add((p + "roll.res").c_str(), e.rollRes);
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
