// Entity - the vessel node: the session's boat block and stepBoat, verbatim (M12 step 5e).
#include "scene/Entity.h"
#include "scene/Gateway.h"
#include "scene/Pose.h"

#include "compose/BuildingSolids.h"
#include "scene/BuildingLayer.h"
#include "core/Common.h"
#include "core/SceneConfig.h"
#include "core/ThreadManager.h"
#include "core/Window.h"
#include "hal/Gpu.h"
#include "scene/SeaLayer.h"
#include "scene/VesselLayer.h"
#include "sim/SeaState.h"
#include "sim/SimClock.h"
#include "sim/VesselSpec.h"
#include "sim/WaveField.h"
#include "sim/WaterManager.h"

#include <algorithm>
#include <atomic>
#include <cstdlib>
#include <chrono>
#include <cmath>
#include <mutex>

namespace ga::scene {

struct Entity::SolidsJob {
    std::atomic<bool> busy{false};
    bool asked = false;
    double askLat = 0.0, askLon = 0.0;
    std::mutex mx;
    std::shared_ptr<const std::vector<BuildingSolid>> result;   // the latest set, whole
    double ms = 0.0;
    uint32_t composes = 0;
};

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
    // ONE BODY, ITS FORMS: the scene's `forms` are several specs' parts in one body (CombineForms),
    // each form engaging its own; without `forms`, the one kind as it always was.
    VesselSpec spec;
    if (!m_props.forms.empty()) {
        std::vector<std::pair<std::string, VesselSpec>> named;
        for (const FormProps& f : m_props.forms) {
            VesselSpec one = vesselReg.Build(f.vessel);
            if (one.kind.empty()) {
                Log("[vessel] '%s': form '%s' is not a registered kind; known:", m_props.name.c_str(), f.vessel.c_str());
                for (const std::string& k : vesselReg.Kinds()) Log("[vessel]   %s", k.c_str());
                return false;
            }
            // The form's name: the kind's first word ("rhib", "drone", "car").
            const size_t dot = f.vessel.find('.');
            named.emplace_back(f.vessel.substr(0, dot), std::move(one));
        }
        spec = CombineForms(named);
    } else {
        spec = vesselReg.Build(m_props.vessel);
    }
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
    helming = false;   // the loop seats ONE helm: the subject it follows (FrameLoop)
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
    // (T used to detach the camera here. T/Y now cycle the helm between the scene's vehicles and
    // F frees the camera: FrameLoop, where the subject is chosen.)
    // THE HELM MOVES THE JOINTS AT THEIR OWN RATES WITHIN THEIR OWN LIMITS: the first jointed
    // element's are the steer's, the first tilted element's the tilt's -- the RHIB's outboard
    // (0.60 rad at 1.2 rad/s; -5..+15 deg at 0.20 rad/s, a trim pump that stays put), which are
    // the numbers this block held as literals, so the boat is byte for byte what it was; a car's
    // kingpin and brake pedal and a drone's levers declare theirs. A hull with no joint keeps
    // the outboard's numbers.
    double sMax = 0.6, sRate = 1.2, tLo = -0.0873, tHi = 0.2618, tRate = 0.20, tReturn = 0.0;
    {
        const Attach* j = nullptr;
        const Attach* t = nullptr;
        for (const Element& el : boat->Spec().elements) {
            if (!j && el.mount.jointed) j = &el.mount;
            if (!t && el.mount.tilted) t = &el.mount;
        }
        if (j) { sMax = j->hi; sRate = j->rate; }
        if (t) { tLo = t->tiltLo; tHi = t->tiltHi; tRate = t->tiltRate; tReturn = t->tiltReturn; }
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
        boatCtl.steer += std::clamp(sd * sMax - boatCtl.steer,
                                    -dt * sRate, dt * sRate);
        boatCtl.steer = std::clamp(boatCtl.steer, -sMax, sMax);

        // TRIM. Shift trims OUT (bow up), Ctrl trims IN (bow down). It is
        // slow on purpose -- a trim pump takes seconds to sweep its range --
        // and it is the helmsman's only direct hold on running attitude.
        // (A pedal or a lever declares a return rate and comes back to zero
        // when released; the pump's is 0 and it stays where it was left.)
        double td = 0.0;
        if (in.keyDown[VK_SHIFT]) td += 1.0;
        if (in.keyDown[VK_CONTROL]) td -= 1.0;
        const double tWant = (td > 0.0) ? tHi : ((td < 0.0) ? tLo : 0.0);
        const double tStep = dt * ((td != 0.0) ? tRate : tReturn);
        boatCtl.tilt += std::clamp(tWant - boatCtl.tilt, -tStep, tStep);
        boatCtl.tilt = std::clamp(boatCtl.tilt, tLo, tHi);
    }
}

void Entity::PlaceAt(const Motor& pose) {
    if (!m_boat) return;
    m_boat->Body().SetPose(pose);   // at rest there
    m_ctl = VesselControls{};
    m_placed = false;               // seated on its surface by the next Update...
    m_reseatKeepsYaw = true;        // ...facing the way the pose faces
}

bool Entity::CycleForm(int dir) {
    if (!m_boat || m_boat->FormCount() < 2) return false;
    const size_t n = m_boat->FormCount();
    const size_t f = (m_boat->FormIndex() + n + (dir < 0 ? n - 1 : 1)) % n;
    if (!m_boat->SetForm(f)) return false;
    // At rest in the new form, seated by the next Update at the form's own seat height over the
    // surface there, facing the way it faced -- the whole of the recovery law.
    m_ctl = VesselControls{};
    m_placed = false;
    m_reseatKeepsYaw = true;
    Log("[vehicle] '%s' takes form '%s' (%s)%s", m_props.name.c_str(), m_boat->CurrentForm().name.c_str(),
        m_boat->Spec().kind.c_str(), m_boat->Spec().recovery ? ": the recovery form, seated over where it stood" : "");
    return true;
}

size_t Entity::FormCount() const { return m_boat ? m_boat->FormCount() : 0; }

const std::string& Entity::MeshFile() const {
    if (m_boat && !m_props.forms.empty() && m_boat->FormIndex() < m_props.forms.size()) {
        return m_props.forms[m_boat->FormIndex()].mesh;
    }
    return m_props.mesh;
}
double Entity::MeshScale() const {
    if (m_boat && !m_props.forms.empty() && m_boat->FormIndex() < m_props.forms.size()) {
        return m_props.forms[m_boat->FormIndex()].meshScale;
    }
    return m_props.meshScale;
}
double Entity::MeshYaw() const {
    if (m_boat && !m_props.forms.empty() && m_boat->FormIndex() < m_props.forms.size()) {
        return m_props.forms[m_boat->FormIndex()].meshYaw;
    }
    return m_props.meshYaw;
}

void Entity::ResetAtRest() {
    // THE CLOCK POLICY, which a stateful body needs and nothing else in this
    // engine does. Everything else here is f(simUnix) and simply re-evaluates;
    // a hull carries momentum, so jumping an hour teleports the sea out from
    // under it while it keeps the velocity it had. It came back as the boat
    // being flung. A deliberate jump resets it to rest at its last pose.
    if (m_boat) {
        PlaceAt(m_boat->Body().pose);
        Log("[vessel] time jumped -- hull reset to rest (a boat cannot be "
            "integrated across a scrub)");
    }
}

bool Entity::FrameIn(const Space* s, double cg[3], double fwd[3]) const {
    if (!m_boat) return false;
    const RigidBody& b = m_boat->Body();
    double c[3] = {0.0, 0.0, 0.0}, f[3] = {0.0, 0.0, 1.0};
    b.pose.TransformPoint(c[0], c[1], c[2]);
    b.pose.TransformDir(f[0], f[1], f[2]);
    const Space* here = m_space ? m_space : m_o.rootSpace;
    if (!here || !s || here == s) {
        for (int i = 0; i < 3; ++i) { cg[i] = c[i]; fwd[i] = f[i]; }
        return true;
    }
    const Placement P = here->To(*s);
    P.Apply(c, cg);
    P.ApplyDir(f, fwd);
    return true;
}

void Entity::PlaceOver(const Entity& other) {
    if (!m_boat) return;
    double cg[3], f[3];
    if (!other.FrameIn(m_space ? m_space : m_o.rootSpace, cg, f)) return;
    // Upright, at its heading: the seat keeps the yaw and sets the height.
    const double o[3] = {0.0, 0.0, 0.0}, upY[3] = {0.0, 1.0, 0.0};
    PlaceAt(Motor::Translation(cg[0], cg[1], cg[2]) * Motor::Rotation(o, upY, std::atan2(f[0], f[2])));
}

// THE HULL'S OWN SPACE IS A POINT OF THE CLOUD (M13 step 2, restated on the cloud 2026-10-11).
// A space is flat; the planet is not: a hull d from its space's origin stands d^2/2R above the
// sphere its water is drawn on, and the water it is seated on is the sphere's own plane in that
// frame (WaterSurface::PlaneOf). The hull takes the cloud's point under it, by its position and
// nothing else (OriginLattice::PickSurface, with the lattice's own hysteresis), and moves to the
// next when the lattice says so: the pose carried by the one motor between the two spaces,
// momenta and all. No other body or view is touched: the floating origin done per subject.
bool Entity::StandOn(const Space& s) {
    if (!s.stand.valid || !m_o.rootSpace || !m_o.rootChart || !m_o.rootChart->Exact()) return false;
    OriginLattice L;
    L.R = m_o.rootChart->planetR;
    m_origin = L.At(s.stand.shell, s.stand.index);
    m_ownSpace = s;
    m_ownChart = m_origin.anchor;   // the point's tangent frame, its radius on it (Exact)
    m_ownValid = true;
    m_space = &m_ownSpace;
    m_spaceInRoot = m_ownSpace.To(*m_o.rootSpace).ToMotor();
    m_sea.SetChart(&m_ownChart);
    ++m_stands;
    return true;
}

void Entity::SolidsStep(double simUnix) {
    if (!m_o.buildings || !m_boat) {
        m_prisms.clear();
        return;
    }
    if (!m_solidsJob) m_solidsJob = std::make_shared<SolidsJob>();
    SolidsJob& J = *m_solidsJob;
    double cg[3] = {0.0, 0.0, 0.0};
    m_boat->Body().pose.TransformPoint(cg[0], cg[1], cg[2]);
    double lat = 0.0, lon = 0.0;
    m_sea.PlaceOf(cg[0], cg[2], lat, lon);
    const double d2r = 3.14159265358979323846 / 180.0;
    const double mPerLat = 111320.0, mPerLon = 111320.0 * std::cos(lat * d2r);
    // The reach: the declared one, or the gate harness's (GAGAME_SOLIDS_REACH in the environment --
    // a probe that wants to see a ramp's foot from the river; not a flag).
    static const double reachM = [] {
        const char* e = std::getenv("GAGAME_SOLIDS_REACH");
        return (e && std::atof(e) > 1.0) ? std::atof(e) : kSolidsReachM;
    }();
    // Ask again when the hull has moved a third of the reach from the last ask (one job at a time).
    const double dx = (lon - J.askLon) * mPerLon, dz = (lat - J.askLat) * mPerLat;
    if ((!J.asked || std::sqrt(dx * dx + dz * dz) > reachM / 3.0) && !J.busy.load()) {
        J.busy = true;
        J.asked = true;
        J.askLat = lat;
        J.askLon = lon;
        const BuildingStack* stack = m_o.buildings;
        std::shared_ptr<SolidsJob> job = m_solidsJob;
        const double dLat = reachM / mPerLat, dLon = reachM / mPerLon;
        Threads().Submit(Lane::Io, "vehicle.solids", [stack, job, lat, lon, dLat, dLon] {
            const auto t0 = std::chrono::steady_clock::now();
            auto out = std::make_shared<std::vector<BuildingSolid>>(
                stack->Compose(lon - dLon, lat - dLat, lon + dLon, lat + dLat, 0.0));
            const double ms = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count() * 1000.0;
            {
                std::lock_guard<std::mutex> lk(job->mx);
                job->result = std::move(out);
                job->ms = ms;
                ++job->composes;
            }
            job->busy = false;
        });
    }
    std::shared_ptr<const std::vector<BuildingSolid>> raw;
    double composeMs = 0.0;
    uint32_t composes = 0;
    {
        std::lock_guard<std::mutex> lk(J.mx);
        raw = J.result;
        composeMs = J.ms;
        composes = J.composes;
    }
    const uint32_t chartGen = m_stands;
    // Re-made when the composed set, the hull's chart, or the layer's resident pages change (a
    // deck's page streams in seconds after the hull stops under it).
    const uint64_t pagesGen = m_o.buildingLayer ? m_o.buildingLayer->PagesVersion() : 0;
    if (raw == m_solidsSeen && chartGen == m_prismsChart && pagesGen == m_prismsPages) return;
    m_solidsSeen = raw;
    m_prismsChart = chartGen;
    m_prismsPages = pagesGen;
    m_prisms.clear();
    if (!raw) return;
    // INTO THE HULL'S FRAME: each footprint vertex by the chart (FlatOf), its ground the bed under
    // its centre (the bed page the water reads; the slow field when none), its bottom and top above
    // that, and the sphere's drop at its distance so a far roof stands where the frame has it.
    const Space::Anchor& chart = m_ownValid ? m_ownChart : *m_o.rootChart;
    const double R = chart.planetR > 0.0 ? chart.planetR : 6371000.0;
    for (const BuildingSolid& b : *raw) {
        if (b.remove || b.rings.empty() || b.rings[0].size() < 6) continue;
        SolidPrism pz;
        const std::vector<double>& ring = b.rings[0];
        double sx = 0.0, sz = 0.0;
        int cnt = 0;
        bool ok = true;
        for (size_t i = 0; i + 1 < ring.size(); i += 2) {
            double x = 0.0, z = 0.0;
            if (!chart.FlatOf(ring[i + 1], ring[i], x, z)) { ok = false; break; }
            pz.xz.push_back(x);
            pz.xz.push_back(z);
            sx += x;
            sz += z;
            ++cnt;
        }
        if (!ok || cnt < 3) continue;
        pz.cx = sx / cnt;
        pz.cz = sz / cnt;
        for (size_t i = 0; i < pz.xz.size(); i += 2) {
            const double ex = pz.xz[i] - pz.cx, ez = pz.xz[i + 1] - pz.cz;
            pz.radius = (std::max)(pz.radius, std::sqrt(ex * ex + ez * ez));
        }
        pz.radius += 0.5;
        double ground = 0.0;
        double gLat = 0.0, gLon = 0.0;
        chart.LatLonOf(pz.cx, pz.cz, gLat, gLon);
        if (!(m_o.bed && m_o.bed->Read(gLat, gLon, ground))) {
            const SurfaceSample ss = m_sea.At(pz.cx, pz.cz, simUnix);
            if (!ss.valid) continue;
            ground = ss.bedNavd;
        }
        const double drop = (pz.cx * pz.cx + pz.cz * pz.cz) / (2.0 * R);
        pz.yBottom = ground - drop + b.bottom;
        pz.yTop = ground - drop + b.top;
        m_prisms.push_back(std::move(pz));
    }
    const size_t buildingsN = m_prisms.size();
    // THE DECKS AND RAMPS (BuildingLayer::DecksNear): each ribbon segment a prism whose top plane
    // runs through the deck's height at its two ends -- the surveyed clearance over the higher of
    // the ground and the sea at each vertex -- a metre deep (the LOD's deck depth), the segment's
    // width across. The same solid a building is, with a slope.
    if (m_o.buildingLayer) {
        std::vector<BuildingLayer::DeckStrip> strips;
        m_o.buildingLayer->DecksNear(lat, lon, reachM, strips);
        for (const BuildingLayer::DeckStrip& st : strips) {
            const double half = (std::max)(st.widthM, 2.0) * 0.5;
            std::vector<double> vx(st.lat.size()), vz(st.lat.size()), vy(st.lat.size()), gv(st.lat.size());
            bool ok = true;
            double gMin = 1e300;
            for (size_t i = 0; i < st.lat.size() && ok; ++i) {
                ok = chart.FlatOf(st.lat[i], st.lon[i], vx[i], vz[i]);
                double ground = 0.0;
                if (!(m_o.bed && m_o.bed->Read(st.lat[i], st.lon[i], ground))) {
                    const SurfaceSample ss = m_sea.At(vx[i], vz[i], simUnix);
                    ground = ss.valid ? ss.bedNavd : 0.0;
                }
                gv[i] = ground;
                gMin = (std::min)(gMin, (std::max)(ground, 0.0));
            }
            if (!ok) continue;
            // The deck at its clearance over the lowest surface its way crosses, riding the ground
            // where the ground is higher -- the stand's law (BuildingLayer::StandBatch), word for word.
            for (size_t i = 0; i < st.lat.size(); ++i) {
                const double drop = (vx[i] * vx[i] + vz[i] * vz[i]) / (2.0 * R);
                vy[i] = (st.over[i] > 0.0 ? (std::max)(gv[i], gMin + st.over[i]) : gv[i]) - drop;
            }
            for (size_t i = 0; i + 1 < vx.size(); ++i) {
                const double ex = vx[i + 1] - vx[i], ez = vz[i + 1] - vz[i];
                const double len = std::sqrt(ex * ex + ez * ez);
                if (len < 0.05) continue;
                const double ux = ex / len, uz = ez / len;
                const double px = -uz * half, pz = ux * half;   // across the run
                SolidPrism pr;
                pr.xz = {vx[i] + px, vz[i] + pz, vx[i + 1] + px, vz[i + 1] + pz,
                         vx[i + 1] - px, vz[i + 1] - pz, vx[i] - px, vz[i] - pz};
                pr.cx = 0.5 * (vx[i] + vx[i + 1]);
                pr.cz = 0.5 * (vz[i] + vz[i + 1]);
                pr.yTop = 0.5 * (vy[i] + vy[i + 1]);
                pr.yBottom = pr.yTop - 1.0;
                const double grade = (vy[i + 1] - vy[i]) / len;
                pr.gx = grade * ux;
                pr.gz = grade * uz;
                pr.radius = 0.5 * len + half + 0.5;
                m_prisms.push_back(std::move(pr));
            }
        }
    }
    // The nearest deck segment, said from the hull: where to drive to find the bridge.
    double nearD = 1e300, nearDx = 0.0, nearDz = 0.0, nearTop = 0.0;
    double lowTop = 1e300, lowLat = 0.0, lowLon = 0.0;   // the ramp's foot: the lowest deck top in reach
    for (size_t i = buildingsN; i < m_prisms.size(); ++i) {
        const double ddx = m_prisms[i].cx - cg[0], ddz = m_prisms[i].cz - cg[2];
        const double d = std::sqrt(ddx * ddx + ddz * ddz);
        if (d < nearD) { nearD = d; nearDx = ddx; nearDz = ddz; nearTop = m_prisms[i].yTop - cg[1]; }
        const double over = m_prisms[i].yTop - cg[1];
        if (over < lowTop) { lowTop = over; chart.LatLonOf(m_prisms[i].cx, m_prisms[i].cz, lowLat, lowLon); }
    }
    // Said when the counts move (the page set moves often while a place streams in).
    static thread_local size_t saidSolids = SIZE_MAX, saidDecks = SIZE_MAX;
    if (raw->size() == saidSolids && m_prisms.size() - buildingsN == saidDecks) return;
    saidSolids = raw->size();
    saidDecks = m_prisms.size() - buildingsN;
    if (m_prisms.size() > buildingsN) {
        Log("[solids] '%s': the lowest deck top in reach stands %+.2f m from the CG at %.6f N %.6f E", m_props.name.c_str(),
            lowTop, lowLat, lowLon);
    }
    Log("[solids] '%s': %zu solids within %.0f m -> %zu prisms in the hull's frame, %zu of them deck segments%s (compose %u took %.1f ms on the pool)",
        m_props.name.c_str(), raw->size(), reachM, m_prisms.size(), m_prisms.size() - buildingsN,
        (m_prisms.size() > buildingsN)
            ? (" -- the nearest deck " + std::to_string(int(nearD)) + " m off at (" + std::to_string(int(nearDx)) + " east, " +
               std::to_string(int(nearDz)) + " north), its top " + std::to_string(nearTop).substr(0, 5) + " m from the CG").c_str()
            : "",
        composes, composeMs);
}

void Entity::Recentre(double simUnix) {
    (void)simUnix;
    if (!m_boat || m_standRefused || !m_o.rootChart || !m_o.rootChart->Exact()) return;
    if (!m_o.planet || !m_o.rootSpace) return;
    RigidBody& b = m_boat->Body();
    double cg[3] = {0.0, 0.0, 0.0};
    b.pose.TransformPoint(cg[0], cg[1], cg[2]);
    const double reach = std::sqrt(cg[0] * cg[0] + cg[2] * cg[2]);
    // Where the hull is, exactly (its frame's rows, on the sphere), and the cloud's point under it.
    const Space::Anchor& chart = m_ownValid ? m_ownChart : *m_o.rootChart;
    double P[3];
    chart.PlanetOf(cg[0], cg[1], cg[2], P);
    OriginLattice L;
    L.R = chart.planetR;
    const OriginLattice::Origin o = L.PickSurface(P, m_ownValid ? &m_origin : nullptr);
    if (m_ownValid && o.Same(m_origin)) return;
    if (!(std::fabs(o.anchor.latDeg) < 89.0)) return;
    Space next = SpaceAt(*m_o.planet, o, m_props.name + ".origin");   // on the cloud, by the one factory
    next.unitM = chart.planetR;
    next.extentM = 2.0 * chart.planetR;
    std::string why;
    if (!next.Declare(&why)) {
        Log("[vessel] '%s' could not stand on the cloud: %s", m_props.name.c_str(), why.c_str());
        m_standRefused = true;   // said once, then left alone
        return;
    }
    // The pose, carried: the space it stands in into the new one, through the common ancestor.
    const Space& from = m_space ? *m_space : *m_o.rootSpace;
    m_boat->Carry(from.To(next).ToMotor());   // the body and the ground it grips
    StandOn(next);
    double after[3] = {0.0, 0.0, 0.0};
    b.pose.TransformPoint(after[0], after[1], after[2]);
    Log("[vessel] '%s' stands on the cloud's origin %lld at %.5f N %.5f E: it stood %.0f m out, where its flat "
        "frame rides %.2f m over the sphere; now %.1f m out",
        m_props.name.c_str(), static_cast<long long>(o.index), o.anchor.latDeg, o.anchor.lonDeg, reach,
        reach * reach / (2.0 * chart.planetR), std::sqrt(after[0] * after[0] + after[2] * after[2]));
}

bool Entity::CgIn(const Space& s, double out[3]) const {
    if (!m_boat) return false;
    double cg[3] = {0.0, 0.0, 0.0};
    m_boat->Body().pose.TransformPoint(cg[0], cg[1], cg[2]);
    const Space* here = m_space ? m_space : m_o.rootSpace;
    if (!here || here == &s) {
        for (int i = 0; i < 3; ++i) out[i] = cg[i];
        return true;
    }
    here->To(s).Apply(cg, out);
    return true;
}

double Entity::Reach() const {
    if (!m_boat) return 0.0;
    return (std::max)(m_boat->Spec().loa.v, m_boat->Spec().beam.v);
}

double Entity::SpeedMps() const {
    if (!m_boat) return 0.0;
    const double bodyCg[3] = {0.0, 0.0, 0.0};
    double v[3] = {0.0, 0.0, 0.0};
    m_boat->Body().VelocityAtBody(bodyCg, v);
    return std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
}

bool Entity::Carry(const Space& into, const Motor& m, double simUnix) {
    if (!m_boat) return false;
    RigidBody& b = m_boat->Body();
    double c0[3] = {0.0, 0.0, 0.0};
    b.pose.TransformPoint(c0[0], c0[1], c0[2]);
    double la0 = 0.0, lo0 = 0.0;
    m_sea.PlaceOf(c0[0], c0[2], la0, lo0);
    // The surface in the frame it is LEAVING (WaterSurface::PlaneOf), before any chart changes.
    const SurfaceSample s0 = m_sea.At(c0[0], c0[2], simUnix);
    double sp0[3] = {c0[0], 0.0, c0[2]}, sn0[3];
    m_sea.PlaneOf(s0, c0[0], c0[2], sp0, sn0);
    // ONE PRODUCT: the pose and the momenta, by the motor.
    m_boat->Carry(m);   // the body and the ground it grips
    // It stands on `into`'s point of the cloud now -- and then on its own, by its position.
    if (!StandOn(into)) {
        Log("[vessel] '%s' was carried into '%s', which stands on no cloud point: it keeps its frame",
            m_props.name.c_str(), into.name.c_str());
    }
    ++m_carries;
    Recentre(simUnix);
    // THE FREE SURFACE ONTO THE FREE SURFACE. The two seas stand at their own heights this
    // instant (two tides, two sea states): the hull keeps its height above ITS water, or it
    // arrives as a drop or a plunge and the buoyancy impulse capsizes it (the set-down's own
    // finding). A translation of the pose; the momenta stand.
    double c1[3] = {0.0, 0.0, 0.0};
    b.pose.TransformPoint(c1[0], c1[1], c1[2]);
    const SurfaceSample s1 = m_sea.At(c1[0], c1[2], simUnix);
    double lift = 0.0;
    if (s0.valid && s1.valid) {
        double sp1[3], sn1[3];
        m_sea.PlaneOf(s1, c1[0], c1[2], sp1, sn1);
        // THE KEEL OVER THE TOP OF THE STACK (fast travel, 2026-10-12), the seat's own law
        // (Update's set-down: the surface a body rides is the top of the water-over-bed stack,
        // max(water, bed), and the keel sits a seat height above it). The carry keeps the keel's
        // height over that top -- a floating boat keeps its draught, a car on the river bed keeps
        // its 11 m under the water -- FLOORED BY THE BED: the keel may stand below the top only as
        // far as there is water there to stand in, so a boat sent to a beach lands with its keel
        // on the sand and a car from the river bed too. One expression; before it a body kept its
        // height over a water plane that stood metres under the sand, and the ground law threw it
        // out (the car at 1100 m/s).
        const double keelY = m_boat->KeelY();
        const double top0 = sp0[1] + (std::max)(0.0, -s0.depthM);
        const double top1 = sp1[1] + (std::max)(0.0, -s1.depthM);
        const double keelOver0 = c0[1] + keelY - top0;
        const double keelOver1 = (std::max)(keelOver0, -(std::max)(s1.depthM, 0.0));
        lift = (top1 + keelOver1 - keelY) - c1[1];
        b.pose = Motor::Translation(0.0, lift, 0.0) * b.pose;
    }
    double la1 = 0.0, lo1 = 0.0;
    m_sea.PlaceOf(c1[0], c1[2], la1, lo1);
    Log("[vessel] '%s' carried from %.5f N %.5f E to %.5f N %.5f E (into %s): surface %+.3f -> %+.3f m "
        "(depth %+.2f -> %+.2f m), lifted %+.3f m%s",
        m_props.name.c_str(), la0, lo0, la1, lo1, into.name.c_str(), s0.valid ? s0.heightNavd : 0.0,
        s1.valid ? s1.heightNavd : 0.0, s0.valid ? s0.depthM : 0.0, s1.valid ? s1.depthM : 0.0, lift,
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
    WaterManager* weather = m_o.weather;
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
        const double bandRms[3] = {sea->BandRmsPhysical(0), sea->BandRmsPhysical(1), sea->BandRmsPhysical(2)};
        boatSea.SetCascadeSea(sea->PeakDirX(), sea->PeakDirZ(), sea->PeakDirValid(), sea->Scale(), bandRms,
                              sea->CascadeCycleUnix());
    }
    boatSea.SetSwellShadow(m_o.swellShadow);
    boatSea.SetBed(m_o.bed);
    // The root space's exact chart until the hull stands on a point of its own (its first step),
    // and the root's chart itself for the steps that cross spaces (the solved window, the wakes).
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
        // (Where a gate is about to put it is the GATE's residency: FrameLoop::StepEntities.)
    }
    // THE HULL'S SPACE FOLLOWS THE HULL -- BEFORE the set-down and before the step, so the hull is
    // seated on the water of the space it is actually in (its first step puts it on the cloud).
    Recentre(simUnix);
    // THE SOLIDS about the hull, in its frame -- before the set-down (the seat reads them) and
    // before the step (the ground law reads them).
    SolidsStep(simUnix);
    boat->SetSolids(m_prisms.empty() ? nullptr : &m_prisms);

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
            // onto the CG, so it is whatever this hull actually is -- and a car's is its tyres
            // at full droop (Vessel::KeelY), so it lands on them and settles into its springs.
            const double keel = boat->KeelY();
            const double draft = (boat->Spec().draftStatic.v > 0.0)
                                     ? boat->Spec().draftStatic.v : -keel;
            // The surface in the HULL'S FRAME (WaterSurface::PlaneOf): the sphere under the hull,
            // not the datum height on a flat plane -- a metre apart 3.6 km from the frame's origin.
            double sp[3], sn[3];
            boatSea.PlaneOf(ss, bp0[0], bp0[2], sp, sn);
            // THE SURFACE THE BODY RIDES IS THE TOP OF THE STACK (2026-10-11): the water where
            // there is water, the bed where it stands proud -- max(height, bed), which is what
            // `depthM <= 0 is dry` already said, written as the water plane raised by the dry
            // part. The keel is set `seatHeight` above it: a boat's minus its draught (the
            // undeclared default, the number this line always used), a car's zero, a drone's
            // its hover height. On water the raise is +0.0 and the boat seats as it did.
            double top = sp[1] + (std::max)(0.0, -ss.depthM);
            // ...and the solids: a deck or a roof under the keel is the top of the stack there, so a
            // body set down over a bridge lands on the bridge (the drone's return to the car on the
            // span). What the solids know by now: a deck whose page has not streamed in yet is not
            // in the list, and the body lands on what is under the deck.
            for (const SolidPrism& pr : m_prisms) {
                const double ddx = bp0[0] - pr.cx, ddz = bp0[2] - pr.cz;
                if (ddx * ddx + ddz * ddz > pr.radius * pr.radius) continue;
                if (!PrismHolds(pr.xz, bp0[0], bp0[2])) continue;
                top = (std::max)(top, pr.TopAt(bp0[0], bp0[2]));
            }
            const double seatH = boat->Spec().seatHeight.src.empty() ? -draft : boat->Spec().seatHeight.v;
            const double y0 = top - keel + seatH;
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
                "-> CG at %+.3f m NAVD (seat %+.2f over the %s)", ss.heightNavd, -keel, draft, y0,
                seatH, (top > sp[1] + (std::max)(0.0, -ss.depthM) + 1e-9) ? "solid" : ((ss.depthM > 0.0) ? "water" : "ground"));
            // The snapshot the hull reads, and its age: what answered, and the instant the
            // solver's answers are coherent at (the frame's asOf, declared before this step).
            char asOf[48];
            if (fi.asOf <= WaterManager::kNeverRead) snprintf(asOf, sizeof(asOf), "never");
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
    // HELD WHERE IT STANDS WHILE ITS WATER IS ABSENT (fast travel, 2026-10-12). Absence is not
    // flat water, and it is not air either: a hull whose sample answers nothing -- a place a solver
    // owns and has not answered yet (a window that spun up under it: Boston's, when the eye
    // arrived after the carry), or a place no tree covers -- gets no buoyancy and no bed, and
    // fell through the sea and the planet for as long as the solver took. The set-down already
    // waits for the water to answer before it places a hull; the step waits the same way: no
    // time passes for the body until its medium exists, and its owed quanta are dropped, not
    // banked (a minute of owed steps would be a minute of physics in one frame).
    {
        double hp[3] = {0.0, 0.0, 0.0};
        boat->Body().pose.TransformPoint(hp[0], hp[1], hp[2]);
        const SurfaceSample here = boatSea.At(hp[0], hp[2], simUnix);
        if (here.valid != !m_heldNoWater) {
            m_heldNoWater = !here.valid;
            Log("[vessel] '%s' %s", m_props.name.c_str(),
                m_heldNoWater ? "is held where it stands: the water under it answers nothing (a solver "
                                "owns the place and has not answered, or nothing covers it)"
                              : "steps again: its water answers");
        }
        if (m_heldNoWater) {
            quantaOwed = 0;
            m_stepMs = 0.0;
            StepTelemetry(boatSea, simUnix);
            return;
        }
    }
    quantaOwed += quanta;
    const int steps = quantaOwed / kPerStep;
    quantaOwed -= steps * kPerStep;
    const auto t0 = std::chrono::steady_clock::now();
    for (int q = 0; q < steps; ++q) {
        boat->Step(boatSea, boatCtl, simUnix, SimClock::kDt * kPerStep);
    }
    if (!boat->Body().Sane()) boatCtl = VesselControls{};
    m_stepMs = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count() * 1000.0;

    // (The gates' boxes are the frame loop's test, after this step: StepEntities.)
    StepTelemetry(boatSea, simUnix);
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
            "depth %.1f ground %.0f N/%d pts%s%s | parts %u",
            bp[0], bp[1], bp[2], t.speedKn, t.headingRad * 57.2957795,
            t.heelRad * 57.2957795, t.trimRad * 57.2957795, t.draughtM, t.immersedVol,
            t.hullVol, t.collarVol, t.wettedLambda, t.copZ, t.depthM, t.contactN, t.contactPts,
            (t.immersedVol < 1e-6 && t.contactPts == 0) ? " AIRBORNE" : (t.aground ? " AGROUND" : ""),
            t.waterValid ? "" : " NO-WATER",
            vesselLayer ? vesselLayer->PartCount() : 0u);
    }
}

}  // namespace ga::scene
