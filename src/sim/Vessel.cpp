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
        const Medium med = sea.WaterAt(wc[0], wc[2], simUnix);
        const SurfaceSample ss = sea.At(wc[0], wc[2], simUnix);
        if (!ss.valid) { m_tel.waterValid = false; continue; }

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
        fb[i] = -0.5 * med.rho * e.dragCd[i].v * e.dragArea[i].v * std::abs(rel[i]) * rel[i];
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
    for (const Element& e : spec.elements) {
        if (e.kind == ElementKind::Collar || e.kind == ElementKind::Planing ||
            e.kind == ElementKind::Foil) {
            // Declared but not yet implemented (they arrive with the RHIB). Refuse rather than
            // contribute zero: a hull silently missing its tubes would float low and look like a
            // mass error, and this engine's ingest rule is that absence is never a value.
            Log("[vessel] spec '%s' uses element kind %d ('%s') which this build does not "
                "implement yet -- refusing to build a partial hull",
                spec.kind.c_str(), static_cast<int>(e.kind), e.name.c_str());
            return false;
        }
    }

    m_spec = spec;
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
                const double mag = c.throttle[i] * e.maxThrust.v * wet;
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
