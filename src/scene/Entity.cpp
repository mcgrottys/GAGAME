// Entity - the vessel node: the session's boat block and stepBoat, verbatim (M12 step 5e).
#include "scene/Entity.h"
#include "scene/Gateway.h"
#include "scene/Pose.h"

#include "core/Common.h"
#include "core/SceneConfig.h"
#include "core/Window.h"
#include "hal/Gpu.h"
#include "scene/SeaLayer.h"
#include "scene/VesselLayer.h"
#include "sim/SeaState.h"
#include "sim/SimClock.h"
#include "sim/VesselSpec.h"
#include "sim/WaveField.h"
#include "sim/WeatherManager.h"

#include <algorithm>
#include <chrono>
#include <cmath>

namespace ga::scene {

std::vector<std::string> Entity::Configure(const Wiring& w) {
    (void)w;   // a hull owns no device object: nothing to wire on this side
    return {};
}

bool Entity::Init(Gpu& gpu) {
    (void)gpu;
    return true;
}

void Entity::Apply(const PropSet& props) {
    // THE ONE PATH for load and hot-reload (Component.h's law). The spawn (`at`) needs no
    // frame: {x, alt, z} is a translation, and the session hands it in through SetSpawn.
    std::string why;
    EntityProps p = m_props;
    if (!props.ApplyTo(&p, nullptr, &why)) {
        Log("[entity] '%s' apply refused: %s", m_props.name.c_str(), why.c_str());
        return;
    }
    p.at = m_props.at;
    m_props = p;
}

void Entity::SetSpawn(double x, double y, double z) {
    m_spawn = Motor::Translation(x, y, z);
    m_spawnTurned = false;
    m_spawnHeadingRad = 0.0;
}

void Entity::SetSpawn(double x, double y, double z, double headingDeg) {
    // A hull is built bow along +z (north). The compass heading is a turn about +y, which takes
    // +z toward +x (east) in Motor::Rotation's right-handed sense (RunPgaSelfTest pins it), then
    // the translation to the spawn: one motor, the turn applied first.
    const double o[3] = {0.0, 0.0, 0.0}, upY[3] = {0.0, 1.0, 0.0};
    m_spawnTurned = true;
    m_spawnHeadingRad = headingDeg * 3.14159265358979323846 / 180.0;
    m_spawn = Motor::Translation(x, y, z) * Motor::Rotation(o, upY, m_spawnHeadingRad);
}

bool Entity::Spawn(const VesselRegistry& vesselReg) {
    auto& boat = m_boat;
    auto& helming = m_helming;
    const VesselSpec spec = vesselReg.Build(m_props.vessel);
    if (spec.kind.empty()) {
        Log("[vessel] '%s' is not a registered kind; known:", m_props.vessel.c_str());
        for (const std::string& k : vesselReg.Kinds()) Log("[vessel]   %s", k.c_str());
        return false;
    }
    spec.PrintLedger();
    boat = std::make_unique<Vessel>();
    // Spawn in the FLAT world frame, at --campos. NOT at cam.px/cam.pz: on a
    // rail the camera starts in the PLANET frame, and a hull built there lands at
    // (5.3e6, -2.4e6) where the bed lookup is meaningless -- it reported AGROUND at
    // depth -278 m, which is exactly what a frame confusion looks like from inside.
    const Motor start = m_spawn;
    double sx = 0.0, sy = 0.0, sz = 0.0;
    start.TransformPoint(sx, sy, sz);
    if (!boat->Build(spec, start)) {
        Log("[vessel] build FAILED -- not spawning (a partial hull would sink and "
            "look like a physics bug)");
        boat.reset();
        return false;
    }
    helming = true;
    // (The hand code printed the CAMERA's x and z here; --campos set both, so the numbers
    // were the spawn's. The spawn is what is printed now, in every case.)
    Log("[vessel] '%s' spawned at (%.1f, %.1f): %s", spec.kind.c_str(), sx, sz,
        spec.display.c_str());
    return true;
}

std::vector<std::string> Entity::Configure(const Observers& o) {
    m_o = o;
    m_wired = true;
    std::vector<std::string> missing;
    if (!o.weather) missing.push_back("weather");
    if (!o.waveField) missing.push_back("waveField");
    if (!o.sea) missing.push_back("sea");
    if (!o.seaState) missing.push_back("seaState");
    if (!o.waterScene) missing.push_back("waterScene");
    if (!o.vesselLayer) missing.push_back("vesselLayer");
    if (!o.gpu) missing.push_back("gpu");
    if (!missing.empty()) {
        std::string list;
        for (const std::string& m : missing) list += (list.empty() ? "" : ", ") + m;
        Log("[entity] '%s' NOT wired: %s (each absent term is reported by the water, never zero)",
            m_props.name.c_str(), list.c_str());
    }
    return missing;
}

void Entity::Record(const ViewContext& v) {
    (void)v;   // the vessel layer draws the hull; the entity steps it
}

void Entity::Helm(const InputState& in, float dt) {
    auto& boat = m_boat;
    auto& helming = m_helming;
    auto& boatCtl = m_ctl;
    if (!boat) return;
    if (in.keyPressed['T']) {
        helming = !helming;
        Log("[vessel] %s", helming ? "helm" : "camera detached (the boat "
                                             "keeps sailing)");
    }
    if (helming) {
        // Keyboard for now; the analog triggers are the twin-lever binnacle
        // and land with XInput. W/S drive BOTH levers, Q/E split them, which
        // is what makes a pivot a squeeze rather than a mode.
        const double rate = dt * 1.5;
        double demand = 0.0;
        if (in.keyDown['W']) demand += 1.0;
        if (in.keyDown['S']) demand -= 1.0;
        double split = 0.0;
        if (in.keyDown['E']) split += 1.0;
        if (in.keyDown['Q']) split -= 1.0;
        for (int t = 0; t < VesselControls::kMaxThrusters; ++t) {
            const double want =
                std::clamp(demand + ((t % 2 == 0) ? -split : split),
                           -1.0, 1.0);
            boatCtl.throttle[t] +=
                std::clamp(want - boatCtl.throttle[t], -rate, rate);
        }
        // D IS STARBOARD, and the sign is the outboard's, not the
        // wheel's. Motor::Rotation about +y takes +z to +x (RunPgaSelfTest
        // pins it), so a POSITIVE steer swings the thrust to starboard --
        // and that thrust acts at the transom, ABAFT the CG, so it pushes
        // the stern to starboard and the bow to PORT. A helm that turns the
        // boat to starboard therefore commands a NEGATIVE angle here, which
        // is exactly what the real linkage does: the leg kicks the stern the
        // opposite way to the turn.
        double sd = 0.0;
        if (in.keyDown['D']) sd -= 1.0;
        if (in.keyDown['A']) sd += 1.0;
        // A mechanical steering RATE limit, not a snap: the outboards swing
        // at a finite speed and that lag is a real part of how a boat feels.
        const double sMax = 0.6;
        boatCtl.steer += std::clamp(sd * sMax - boatCtl.steer,
                                    -dt * 1.2, dt * 1.2);
        boatCtl.steer = std::clamp(boatCtl.steer, -sMax, sMax);

        // TRIM. Shift trims OUT (bow up), Ctrl trims IN (bow down). It is
        // slow on purpose -- a trim pump takes seconds to sweep its range --
        // and it is the helmsman's only direct hold on running attitude.
        double td = 0.0;
        if (in.keyDown[VK_SHIFT]) td += 1.0;
        if (in.keyDown[VK_CONTROL]) td -= 1.0;
        boatCtl.tilt = std::clamp(boatCtl.tilt + td * dt * 0.20,
                                  -0.0873, 0.2618);
    }
}

void Entity::ResetAtRest() {
    auto& boat = m_boat;
    auto& boatCtl = m_ctl;
    auto& boatPlaced = m_placed;
    // THE CLOCK POLICY, which a stateful body needs and nothing else in this
    // engine does. Everything else here is f(simUnix) and simply re-evaluates;
    // a hull carries momentum, so jumping an hour teleports the sea out from
    // under it while it keeps the velocity it had. It came back as the boat
    // being flung. A deliberate jump resets it to rest at its last pose.
    if (boat) {
        boat->Body().Rest();
        boatCtl = VesselControls{};
        boatPlaced = false;   // re-seat it on the new instant's surface
        m_reseatKeepsYaw = true;   // ...facing the way it was
        Log("[vessel] time jumped -- hull reset to rest (a boat cannot be "
            "integrated across a scrub)");
    }
}

// THE PER-SUBJECT FLOATING ORIGIN (M13 step 2). A space is flat; the planet is not. A hull sitting
// d from its space's origin stands d^2/2R above the sphere its water is drawn on -- 8 cm at a
// kilometre, 31 cm at two, 300 m at sixty, which is a hull flying over its own sea. So the hull
// carries its space with it: past `recentreM` the space is rebuilt as the tangent frame AT THE
// HULL'S OWN PLACE and the pose is carried into it by the relative placement, momenta and all.
//
// What does NOT happen: the world frame does not move, no other body or view is touched, and the
// hull's state is unchanged in its own terms -- this is the floating origin done per subject,
// which is the thing a world re-anchor was rejected for.
void Entity::Recentre(double simUnix) {
    (void)simUnix;
    if (!m_boat || m_props.recentreM <= 0.0 || !m_o.rootChart || !m_o.rootChart->Exact()) return;
    if (!m_o.planet || !m_o.rootSpace) return;
    RigidBody& b = m_boat->Body();
    double cg[3] = {0.0, 0.0, 0.0};
    b.pose.TransformPoint(cg[0], cg[1], cg[2]);
    const double reach = std::sqrt(cg[0] * cg[0] + cg[2] * cg[2]);
    if (reach < m_props.recentreM) return;

    // Where the hull is, exactly (its own space's rows, on the sphere).
    const Space::Anchor& chart = m_ownValid ? m_ownChart : (m_space ? m_gateChart : *m_o.rootChart);
    double lat = 0.0, lon = 0.0;
    chart.PlaceOf(cg[0], cg[1], cg[2], lat, lon);
    // The new space: the tangent frame there, built by the rule every other frame in the engine
    // is built by, hung under the planet.
    const double R = chart.planetR;
    const PoseFrame fr = FrameFromAnchor(lat, lon, R);
    if (!fr.valid) return;
    Space next;
    next.name = m_props.name + ".origin";
    next.unitM = R;
    next.extentM = 2.0 * R;
    next.parent = m_o.planet;
    const double anchor[3] = {fr.up[0] * R, fr.up[1] * R, fr.up[2] * R};
    next.link = Placement::Frame(fr.east, fr.up, fr.north, anchor);
    std::string why;
    if (!next.Declare(&why)) {
        Log("[vessel] '%s' could not re-centre: %s", m_props.name.c_str(), why.c_str());
        m_props.recentreM = 0.0;   // said once, then left alone
        return;
    }
    // The pose, carried: the hull's own space into the new one, through the common ancestor.
    const Space& from = m_space ? *m_space : *m_o.rootSpace;
    const Placement toNew = from.To(next);
    b.Carry(toNew.ToMotor());
    m_ownSpace = next;
    m_ownChart = Space::Anchor{};
    m_ownChart.latDeg = lat;
    m_ownChart.lonDeg = lon;
    m_ownChart.mPerLat = 110574.0;
    m_ownChart.mPerLon = 111320.0 * std::cos(lat * 3.14159265358979323846 / 180.0);
    m_ownChart.linear = true;
    for (int i = 0; i < 3; ++i) {
        m_ownChart.east[i] = fr.east[i];
        m_ownChart.up[i] = fr.up[i];
        m_ownChart.north[i] = fr.north[i];
    }
    m_ownChart.planetR = R;
    m_ownValid = true;
    m_space = &m_ownSpace;
    m_spaceInRoot = m_ownSpace.To(*m_o.rootSpace).ToMotor();
    m_sea.SetChart(&m_ownChart);
    double after[3] = {0.0, 0.0, 0.0};
    b.pose.TransformPoint(after[0], after[1], after[2]);
    Log("[vessel] '%s' re-centred its origin at %.5f N %.5f E: it stood %.0f m out, where its flat "
        "frame rides %.2f m over the sphere; now %.1f m out",
        m_props.name.c_str(), lat, lon, reach, reach * reach / (2.0 * R),
        std::sqrt(after[0] * after[0] + after[2] * after[2]));
}

bool Entity::Teleport(const Gateway& gate, double simUnix) {
    if (!m_boat) return false;
    RigidBody& b = m_boat->Body();
    double c0[3] = {0.0, 0.0, 0.0};
    b.pose.TransformPoint(c0[0], c0[1], c0[2]);
    double la0 = 0.0, lo0 = 0.0;
    m_sea.PlaceOf(c0[0], c0[2], la0, lo0);
    const SurfaceSample s0 = m_sea.At(c0[0], c0[2], simUnix);
    // ONE PRODUCT: the pose and the momenta, by the gate's motor.
    b.Carry(gate.Carry());
    m_space = &gate.Destination();
    m_lastGate = &gate;   // the view follows its subject through THIS window
    ++m_carries;
    m_spaceInRoot = gate.DestinationInSource();
    m_gateChart = gate.Chart();      // M13: kept, so a later re-centre knows this hull's chart
    m_ownValid = false;              // the gate's space replaces any the hull carried
    m_sea.SetChart(&gate.Chart());   // the same sparse water, read at the destination's places
    double c1[3] = {0.0, 0.0, 0.0};
    b.pose.TransformPoint(c1[0], c1[1], c1[2]);
    const SurfaceSample s1 = m_sea.At(c1[0], c1[2], simUnix);
    // THE FREE SURFACE ONTO THE FREE SURFACE. Both boxes sit at their own spaces' datum, and the
    // two seas stand at their own heights this instant (two tides, two sea states): the hull keeps
    // its height above ITS water, or it arrives as a drop or a plunge and the buoyancy impulse
    // capsizes it (the set-down's own finding). A translation of the pose; the momenta stand.
    double lift = 0.0;
    if (s0.valid && s1.valid) {
        lift = s1.heightNavd - s0.heightNavd;
        b.pose = Motor::Translation(0.0, lift, 0.0) * b.pose;
    }
    double la1 = 0.0, lo1 = 0.0;
    m_sea.PlaceOf(c1[0], c1[2], la1, lo1);
    Log("[gate] '%s' carried '%s' from %.5f N %.5f E to %.5f N %.5f E (%s): surface %+.3f -> %+.3f m, "
        "lifted %+.3f m%s",
        gate.Declared().name.c_str(), m_props.name.c_str(), la0, lo0, la1, lo1,
        gate.Destination().name.c_str(), s0.valid ? s0.heightNavd : 0.0,
        s1.valid ? s1.heightNavd : 0.0, lift,
        (s0.valid && s1.valid) ? "" : " (a surface did not answer: no lift applied)");
    return true;
}

void Entity::ChaseFrame(double p[3], double f[3]) {
    auto& boat = m_boat;
    auto& helmYawRef = m_helmYawRef;
    const RigidBody& b = boat->Body();
    p[0] = p[1] = p[2] = 0.0;
    b.pose.TransformPoint(p[0], p[1], p[2]);
    f[0] = 0.0;
    f[1] = 0.0;
    f[2] = 1.0;
    b.pose.TransformDir(f[0], f[1], f[2]);
    const double fl = std::sqrt(f[0] * f[0] + f[2] * f[2]);
    if (fl > 1e-6) { f[0] /= fl; f[2] /= fl; }
    // The look-steer's reference is THIS camera's heading, kept here rather than
    // read off `cam`, so detaching the view (T, or a spectator flying to orbit in a
    // future multiplayer) cannot feed the assist a heading from the other side of
    // the planet.
    helmYawRef = std::atan2(f[2], f[0]);
}

// ONE step path, called from BOTH clock branches. The windowed clock advances in whole
// quanta off SimClock and the headless clock is frame-indexed, but a boat that
// integrated differently between them could not be gated by any rail -- and the rails
// are the only reproducible instrument this engine has. So the quanta COUNT differs and
// nothing else does.
void Entity::Update(const FrameInfo& fi) {
    auto& boat = m_boat;
    auto& boatSea = m_sea;
    auto& boatCtl = m_ctl;
    auto& boatPlaced = m_placed;
    auto& quantaOwed = m_quantaOwed;
    auto& telTick = m_telTick;
    const int quanta = fi.quanta;
    const double simUnix = fi.simUnix;
    WeatherManager* weather = m_o.weather;
    const WaveField* waveField = m_o.waveField;
    SeaLayer* sea = m_o.sea;
    const SeaState* seaState = m_o.seaState;
    const WaterSceneConfig* waterScene = m_o.waterScene;
    VesselLayer* vesselLayer = m_o.vesselLayer;
    m_stepMs = 0.0;
    if (!boat) return;
    boatSea.Configure(weather, waveField, sea ? &sea->Ocean() : nullptr,
                      seaState, sea ? double(sea->heightScale) : 1.0,
                      waterScene ? waterScene->wfExag : 1.0f,
                      waterScene ? waterScene->wfChop : 1.0f);
    // The cascade sea's context, as the bank kernel is handed it this frame (one wave rule).
    if (sea) {
        boatSea.SetCascadeSea(sea->PeakDirX(), sea->PeakDirZ(), sea->PeakDirValid(), sea->StormOn());
    }
    boatSea.SetSwellShadow(m_o.swellShadow);
    boatSea.SetBed(m_o.bed);
    // M13 step 2: the root space's exact chart, for a hull that has not been carried (a carried
    // hull holds its gate's, set at the carry and not overwritten here), and the root's chart
    // itself for the steps that cross spaces (the solved window, the wake table).
    if (!m_space) boatSea.SetChart(m_o.rootChart);
    boatSea.SetRootChart(m_o.rootChart);
    // THE SOLVER IS TRUTH, AND THE HULL ASKS FOR IT (the water match, step 1). Every frame, the
    // solver's region around the hull: its reach from the CG (the spec's length overall, which
    // bounds every station wherever the CG sits) plus the distance it covers before the answer
    // arrives -- the frame ring's latency and this frame, at its speed. Answered two frames on,
    // without a stall; until the first answer the solver's water reports no level and the set-down
    // below waits for it, as it waits for any water that has not answered.
    if (weather) {
        const double bodyCg[3] = {0.0, 0.0, 0.0};   // the body origin IS the CG (RigidBody.h)
        double cg[3] = {0.0, 0.0, 0.0}, v[3] = {0.0, 0.0, 0.0};
        boat->Body().pose.TransformPoint(cg[0], cg[1], cg[2]);
        boat->Body().VelocityAtBody(bodyCg, v);
        const double frameS = double((std::max)(quanta, 1)) * SimClock::kDt;
        const double leadS = double(Gpu::kFrameCount + 1u) * frameS;
        const double reach = (std::max)(boat->Spec().loa.v, boat->Spec().beam.v);
        const double speed = std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
        double latDeg = 0.0, lonDeg = 0.0;
        boatSea.PlaceOf(cg[0], cg[2], latDeg, lonDeg);
        weather->RequestRegion(latDeg, lonDeg, reach + speed * leadS);
    }
    // M13 step 2: THE HULL'S SPACE FOLLOWS THE HULL -- and it does so BEFORE the set-down and
    // before the step, so the hull is seated on the water of the space it is actually in. Done
    // after the step instead, a spawn 60 km from the anchor is set down on its old frame's flat
    // sea, the re-centring then says honestly that this is 300 m of air, and the hull falls.
    Recentre(simUnix);

    // PLACE THE HULL ON THE WATER, ONCE. A vessel is built before the weather
    // manager exists, so it cannot be spawned at the right height -- and NAVD 0 is half
    // a metre under the surface here at this tide. Dropped in submerged, the hull takes
    // a buoyancy impulse of its own displacement in one quantum and the RHIB simply
    // capsized: heel 179 deg, floating inverted, never recovering. So the first step
    // sets it down gently instead of the scene throwing it in.
    if (!boatPlaced) {
        double bp0[3] = {0, 0, 0};
        boat->Body().pose.TransformPoint(bp0[0], bp0[1], bp0[2]);
        const SurfaceSample ss = boatSea.At(bp0[0], bp0[2], simUnix);
        if (ss.valid) {
            // Put the KEEL at its static draught, not the CG on the waterline. The CG
            // sits well above the keel, so placing it at the surface immersed the hull
            // half a metre deeper than it floats and it came up like a cork. The keel
            // depth comes from the spec's own stations, after Build re-referenced them
            // onto the CG, so it is whatever this hull actually is.
            double keel = 0.0;
            for (const Element& el : boat->Spec().elements) {
                for (const Section& st : el.stations) {
                    for (double y : st.oy) keel = (std::min)(keel, y);
                }
            }
            const double draft = (boat->Spec().draftStatic.v > 0.0)
                                     ? boat->Spec().draftStatic.v : -keel;
            const double y0 = ss.heightNavd - keel - draft;
            // THE SEAT KEEPS THE HEADING (the Haulover portal demo, 2026-09-14). The set-down
            // levels the hull at its draught; it never meant to turn it -- but a bare translation
            // turned every hull to north, which nothing noticed while no spawn declared a heading
            // and a time jump (which re-seats, "at its last pose") snapped a helmed boat north.
            // First placement: the spawn's declared heading. A re-seat: the heading it had. A spawn
            // that declared none, first placement: the translation alone, byte for byte as before
            // (a composed identity turn could flip a zero's sign, and the telemetry prints it).
            Motor seat = Motor::Translation(bp0[0], y0, bp0[2]);
            if (m_reseatKeepsYaw || m_spawnTurned) {
                double yaw = m_spawnHeadingRad;
                if (m_reseatKeepsYaw) {
                    double fx = 0.0, fy = 0.0, fz = 1.0;
                    boat->Body().pose.TransformDir(fx, fy, fz);
                    yaw = std::atan2(fx, fz);
                }
                const double o[3] = {0.0, 0.0, 0.0}, upY[3] = {0.0, 1.0, 0.0};
                seat = seat * Motor::Rotation(o, upY, yaw);
            }
            m_reseatKeepsYaw = false;
            boat->Body().SetPose(seat);
            boatPlaced = true;
            Log("[vessel] set down: surface %+.3f, keel %.3f below CG, draught %.2f "
                "-> CG at %+.3f m NAVD", ss.heightNavd, -keel, draft, y0);
            // The snapshot the hull reads, and its age: what answered, and the instant the
            // solver's answers are coherent at (the frame's asOf, declared before this step).
            char asOf[48];
            if (fi.asOf <= WeatherManager::kNeverRead) snprintf(asOf, sizeof(asOf), "never");
            else snprintf(asOf, sizeof(asOf), "t=%.0f", fi.asOf);
            Log("[entity] '%s' reads %s (the frame's declared asOf: %s)", m_props.name.c_str(),
                boatSea.Describe(bp0[0], bp0[2], simUnix).c_str(), asOf);
        } else {
            return;   // no water yet: do not integrate a hull that has nothing to float on
        }
    }
    if (m_props.controller == 1) {   // fixed: the declared throttles and helm (--boat-drive)
        for (int t = 0; t < VesselControls::kMaxThrusters; ++t) {
            boatCtl.throttle[t] = m_props.throttle;
        }
        boatCtl.steer = m_props.steer;
    }
    // THE HULL STEPS AT 60 Hz, NOT 240. SimClock's quantum is 240 Hz because that is
    // what the SCENE needed; nothing in a hull's dynamics asks for 4 ms resolution. Its
    // fastest modes are heave at ~1 s, roll at ~1.8 s, and the collar's slam at ~10 Hz
    // -- 60 Hz resolves the quickest of those by six to one.
    //
    // It matters because a step is not cheap: ~73 water queries for this hull, and each
    // one sums the solved field's 32 components and the retained cascade bins. At 240 Hz
    // that measured 26 ms a frame, which is more than the entire renderer costs.
    //
    // Determinism is untouched -- this is still a FIXED step off the same clock, just a
    // coarser multiple of it, so the boat is as frame-rate independent as before. The
    // leftover quanta are carried, never dropped, so no owed time is lost.
    constexpr int kPerStep = 4;                     // 240 / 4 = 60 Hz
    quantaOwed += quanta;
    const int steps = quantaOwed / kPerStep;
    quantaOwed -= steps * kPerStep;
    const auto t0 = std::chrono::steady_clock::now();
    for (int q = 0; q < steps; ++q) {
        boat->Step(boatSea, boatCtl, simUnix, SimClock::kDt * kPerStep);
    }
    if (!boat->Body().Sane()) boatCtl = VesselControls{};
    m_stepMs = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count() * 1000.0;

    // THE GATES: a hull whose centre of gravity is inside a box is carried. The test is made in
    // the GATE's own source space, so a hull that has re-centred (or been carried before) still
    // meets the gates standing in that space -- the special case for "in the root space" went
    // out with the floating origin, which gives every hull a space of its own.
    if (m_o.gates) {
        for (const auto& g : *m_o.gates) {
            if (!g || !g->Valid()) continue;
            double cg[3] = {0.0, 0.0, 0.0};
            boat->Body().pose.TransformPoint(cg[0], cg[1], cg[2]);
            if (m_space) {   // into the gate's source space, through the common ancestor
                const Placement toSrc = m_space->To(g->Source());
                double q[3] = {cg[0], cg[1], cg[2]};
                toSrc.Apply(q, cg);
            }
            if (g->Inside(cg[0], cg[1], cg[2])) {
                Teleport(*g, simUnix);
                break;
            }
        }
    }
    PublishDraw();
    StepTelemetry(boatSea, simUnix);
}

// THE DRAW, published after the step -- and after the VIEW has decided what it is looking
// through this frame (FrameLoop's chase block sets the carry). It used to happen inside the step,
// which meant the frame a gate carried the hull it was still drawn in the frame the view had
// finished with: one frame of a vanished boat, exactly at the moment you are watching it go.
void Entity::PublishDraw() {
    VesselLayer* vesselLayer = m_o.vesselLayer;
    if (!vesselLayer || !m_boat) return;
    const std::unique_ptr<Vessel>& boat = m_boat;
    {
        const Vessel* vs[1] = {boat.get()};
        // THE FRAME THE HULL IS DRAWN IN. Its own space's placement in the root -- and, while the
        // view still stands on this side of a gate this hull went through, that placement pulled
        // back through the window (m_viewPull, the gate's motor inverted: the same map the
        // window's own geometry is drawn by, so the hull lands on the destination's sea exactly
        // where the window shows it). `through` carries to the shader which side of the window's
        // slab test keeps the pixel.
        const uint8_t through = m_viewThrough ? 1u : 0u;
        const Motor frame = m_space ? (m_viewPullOn ? m_viewPull * m_spaceInRoot : m_spaceInRoot)
                                    : (m_viewPullOn ? m_viewPull : Motor::Identity());
        vesselLayer->SetVessels(vs, &frame, &through, 1);
    }
}

void Entity::StepTelemetry(TreeWater& boatSea, double simUnix) {
    VesselLayer* vesselLayer = m_o.vesselLayer;
    const std::unique_ptr<Vessel>& boat = m_boat;
    if (!boat) return;
    int& telTick = m_telTick;
    // One telemetry line a second. Cheap, and it is the only way to tell a hull that is
    // floating wrong from one that is not being DRAWN.
    if ((telTick++ % 60) == 0) {
        const VesselTelemetry& t = boat->Telemetry();
        double bp[3] = {0, 0, 0};
        boat->Body().pose.TransformPoint(bp[0], bp[1], bp[2]);
        // The water the hull is standing on, at the hull. Buoyancy acts along this
        // NORMAL, so a wrong slope is not a cosmetic error -- it is a horizontal force.
        const SurfaceSample ws = boatSea.At(bp[0], bp[2], simUnix);
        const double slope = std::sqrt(ws.nx * ws.nx + ws.nz * ws.nz) /
                             ((std::abs(ws.ny) > 1e-9) ? std::abs(ws.ny) : 1e-9);
        Log("[vessel]   water: eta %+.2f n (%+.3f, %+.3f, %+.3f) |slope| %.3f = %.1f deg"
            "  orbital (%+.2f, %+.2f, %+.2f) m/s",
            ws.heightNavd, ws.nx, ws.ny, ws.nz, slope,
            std::atan(slope) * 57.2957795, ws.vx, ws.vy, ws.vz);
        Log("[vessel] pos (%.1f, %+.2f, %.1f) %.1f kn hdg %.0f heel %+.1f trim %+.1f "
            "draught %.3f vol %.2f (hull %.2f collar %.2f) lam %.2f cop %+.2f "
            "depth %.1f%s%s | parts %u",
            bp[0], bp[1], bp[2], t.speedKn, t.headingRad * 57.2957795,
            t.heelRad * 57.2957795, t.trimRad * 57.2957795, t.draughtM, t.immersedVol,
            t.hullVol, t.collarVol, t.wettedLambda, t.copZ, t.depthM,
            (t.immersedVol < 1e-6) ? " AIRBORNE" : (t.aground ? " AGROUND" : ""),
            t.waterValid ? "" : " NO-WATER",
            vesselLayer ? vesselLayer->PartCount() : 0u);
    }
}

}  // namespace ga::scene
