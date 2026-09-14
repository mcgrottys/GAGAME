// Entity - the vessel node: the session's boat block and stepBoat, verbatim (M12 step 5e).
#include "scene/Entity.h"

#include "core/Common.h"
#include "core/SceneConfig.h"
#include "core/Window.h"
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
    if (m_wired && m_o.weather) m_o.weather->SetMirrorCadence(m_props.mirrorCadence);
}

void Entity::SetSpawn(double x, double y, double z) {
    m_spawn = Motor::Translation(x, y, z);
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
    // THE FRESHNESS CONTRACT's declaration reaches the manager here; 0 = never, today's cost.
    if (o.weather) o.weather->SetMirrorCadence(m_props.mirrorCadence);
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
        Log("[vessel] time jumped -- hull reset to rest (a boat cannot be "
            "integrated across a scrub)");
    }
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
    // THE FRESHNESS CONTRACT: the entity drives the mirror, on its declared cadence -- a
    // readback only when the mirror is older than the cadence, and never one per hull step.
    // At the default (0 = never) this is exactly the hand code: no reader in the loop.
    if (weather && m_o.gpu) weather->RefreshOnCadence(*m_o.gpu, simUnix);
    boatSea.Configure(weather, waveField, sea ? &sea->Ocean() : nullptr,
                      seaState, sea ? double(sea->heightScale) : 1.0,
                      waterScene ? waterScene->wfExag : 1.0f,
                      waterScene ? waterScene->wfChop : 1.0f);
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
            boat->Body().SetPose(Motor::Translation(bp0[0], y0, bp0[2]));
            boatPlaced = true;
            Log("[vessel] set down: surface %+.3f, keel %.3f below CG, draught %.2f "
                "-> CG at %+.3f m NAVD", ss.heightNavd, -keel, draft, y0);
            // The snapshot the hull reads, and its age: what answered, and whether the
            // solver's mirror was ever read (the freshness contract's own line). The frame's
            // asOf is the instant declared BEFORE this step's refresh -- on the first step,
            // never.
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

    const Vessel* vs[1] = {boat.get()};
    if (vesselLayer) vesselLayer->SetVessels(vs, 1);

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
