// Rail - the rail file read into segments, and At(t) (M12 step 5e). See Rail.h.
#include "scene/Rail.h"

#include "core/Common.h"
#include "core/Droste.h"
#include "scene/Pose.h"
#include "scene/SceneBuilder.h"
#include "scene/SceneSchema.h"
#include "sim/GlobeModel.h"

#include <algorithm>
#include <cmath>
#include <cstring>

namespace ga::scene {

namespace {

bool Refuse(std::string* why, const std::string& s) {
    if (why) *why = s;
    return false;
}

// ---- the vocabulary: one prototype per element kind, one Schema over each (Props.h's law) --
struct RailFileProto {
    std::string name;
    bool tower = false;
};
struct RailPoseProto {
    std::string name;
    Motor at;
    int aim = 0;                        // none | fixedPoint
    double standOffBelowTwistDeg = 0.0;
    double standOff[3] = {0.0, 0.0, 0.0};
    std::string turnOf;
    double turnYaw = 0.0, turnPitch = 0.0;
};
struct RailSegmentProto {
    int kind = 0;                       // keys | spiral | hold | turn
    double duration = 0.0;
    int levels = 0;
    double from = 0.0;
    int up = 0;                         // none | gravity
    std::string base, upOf, to;
    int law = 0;                        // dive | leg
    double u0 = 0.0, u1 = 0.0, delay = 0.0;
    int level = 0;
};
RailFileProto kFile;
RailPoseProto kPose;
RailSegmentProto kSeg;

using Q = Quantity;
constexpr Reload R = Reload::Restart;

// An element read through its schema by the one parser (PropSet::Merge): unknown keys and
// wrong units refuse naming `path`; a key the element does not carry keeps the prototype's
// value (the file is completed before it is read, so every declared key is there).
class Element {
public:
    Element(const Schema& s, const JsonValue& e, std::string path, std::string* why, bool* ok)
        : m_set(PropSet::Defaults(s, s.Prototype())), m_e(&e), m_path(std::move(path)) {
        if (*ok && !m_set.Merge(e, m_path, why)) *ok = false;
    }
    double N(const char* key) const { return m_set.Get(key)->n; }
    int E(const char* key) const { return m_set.Get(key)->e; }
    bool B(const char* key) const { return m_set.Get(key)->b; }
    const std::string& S(const char* key) const { return m_set.Get(key)->s; }
    const double* V(const char* key) const { return m_set.Get(key)->v; }
    // A Motor key kept AS WRITTEN (it resolves in a frame, later); false when absent.
    bool Raw(const char* key, JsonValue& out) const {
        const JsonValue* v = m_e->Get(key);
        if (!v) return false;
        out = *v;
        return true;
    }
    const JsonValue* List(const char* key) const {
        const JsonValue* v = m_e->Get(key);
        return (v && v->type == JsonValue::Type::Array) ? v : nullptr;
    }
    const std::string& Path() const { return m_path; }

private:
    PropSet m_set;
    const JsonValue* m_e;
    std::string m_path;
};

}  // namespace

// ---- the schemas ------------------------------------------------------------------------------

const Schema& Rail::PoseSchema() {
    static const Schema* s = [] {
        Schema* sc = new Schema("rail.pose", &kPose);
        sc->Bind("name", kPose.name, "the pose's name (a key's `pose`, a spiral's `base`)", R)
            .Bind("at", kPose.at, "the camera, in the placement sugar; absent = turned from `turnOf`", R)
            .Optional()
            .BindEnum("aim", kPose.aim, {"none", "fixedPoint"},
                      "re-aim the camera at the portal's fixed point (the tower's helm)", R)
            .Bind("standOffBelowTwistDeg", kPose.standOffBelowTwistDeg, Q::Angle, "deg",
                  "below this twist the eye stands at fixedPoint + standOff instead (M10: an untwisted tower dives from above); 0 = never", R)
            .Bind("standOff", kPose.standOff, Q::Length, "m", "the stand-off from the fixed point", R)
            .Bind("turnOf", kPose.turnOf, "the pose this one is turned from about the local vertical", R)
            .Bind("turnYaw", kPose.turnYaw, Q::Angle, "rad", "the turn about the up, from the base's compass yaw", R)
            .Bind("turnPitch", kPose.turnPitch, Q::Angle, "rad", "the pitch the turned pose takes", R);
        return sc;
    }();
    return *s;
}

const Schema& Rail::SegmentSchema() {
    static const Schema* s = [] {
        Schema* sc = new Schema("rail.segment", &kSeg);
        sc->BindEnum("kind", kSeg.kind, {"keys", "spiral", "hold", "turn"}, "the segment's law", R)
            .Bind("duration", kSeg.duration, Q::Time, "s", "seconds; 0 = the rest of the rail", R)
            .Bind("levels", kSeg.levels, Q::Dimensionless, "1", "spiral: the duration in levels of the dive's pace (0 = `duration`)", R)
            .Bind("from", kSeg.from, Q::Time, "s", "keys: the key time the segment starts at", R)
            .BindEnum("up", kSeg.up, {"none", "gravity"}, "keys: write the up (anti-gravity at the eye) -- a tower rail's keyed legs", R)
            .List("keys", &RailKeySchema(), "keys: the poses at times", false)
            .Bind("base", kSeg.base, "spiral/hold/turn: the pose S^u acts on (turn: from)", R)
            .Bind("upOf", kSeg.upOf, "spiral/turn: the pose whose up the sample carries (Q^f of it on a spiral)", R)
            .Bind("to", kSeg.to, "turn: the pose turned to", R)
            .BindEnum("law", kSeg.law, {"dive", "leg"}, "spiral: u(t) -- the dive's clock, or a leg u0 -> u1 with ramps", R)
            .Bind("u0", kSeg.u0, Q::Dimensionless, "1", "leg: u at the start", R)
            .Bind("u1", kSeg.u1, Q::Dimensionless, "1", "leg: u at the end", R)
            .Bind("delay", kSeg.delay, Q::Time, "s", "dive: seconds held on the base before the clock starts", R)
            .Bind("level", kSeg.level, Q::Dimensionless, "1", "turn: the level the pose is written in", R);
        return sc;
    }();
    return *s;
}

const Schema& Rail::FileSchema() {
    static const Schema* s = [] {
        Schema* sc = new Schema("rail", &kFile);
        sc->Bind("name", kFile.name, "the rail's name (rails.active)", R)
            .Bind("tower", kFile.tower, "flown in the Droste tower: writes the camera's level and its up", R)
            .List("poses", &PoseSchema(), "the tower poses, by name", true)
            .List("segments", &SegmentSchema(), "the segments, in order", false);
        return sc;
    }();
    return *s;
}

// ---- the read -----------------------------------------------------------------------------------

bool Rail::Load(const std::string& path, std::string* why) {
    std::string text;
    if (!SceneBuilder::ReadFile(path, text, why)) return false;
    std::string err;
    JsonValue doc = JsonParser::Parse(text, &err);
    if (!err.empty()) return Refuse(why, path + ": " + err);
    m_path = path;
    return FromJson(doc, path, why);
}

bool Rail::FromJson(const JsonValue& in, const std::string& from, std::string* why) {
    JsonValue doc = in;
    SchemaChain root;
    root.a = &FileSchema();
    Complete(doc, root);
    if (!Validate(doc, root, from, why)) return false;
    bool ok = true;
    m_segments.clear();
    m_poseDecls.clear();
    m_poses.clear();
    m_resolved = false;
    Element top(FileSchema(), doc, from, why, &ok);
    if (!ok) return false;
    m_name = top.S("name");
    m_tower = top.B("tower");
    if (const JsonValue* poses = top.List("poses")) {
        for (size_t i = 0; i < poses->arr.size(); ++i) {
            const JsonValue& e = poses->arr[i];
            Element p(PoseSchema(), e, from + ".poses[" + std::to_string(i) + "]", why, &ok);
            if (!ok) return false;
            PoseDecl d;
            d.name = p.S("name");
            d.hasAt = p.Raw("at", d.at);
            d.aim = p.E("aim");
            d.standOffBelowTwistDeg = p.N("standOffBelowTwistDeg");
            for (int k = 0; k < 3; ++k) d.standOff[k] = p.V("standOff")[k];
            d.turnOf = p.S("turnOf");
            d.turnYaw = p.N("turnYaw");
            d.turnPitch = p.N("turnPitch");
            if (!d.hasAt && d.turnOf.empty()) {
                return Refuse(why, p.Path() + ": a pose needs `at` or `turnOf`");
            }
            m_poseDecls.push_back(std::move(d));
        }
    }
    const JsonValue* segs = top.List("segments");
    if (!segs || segs->arr.empty()) return Refuse(why, from + ".segments: a rail needs a segment");
    for (size_t i = 0; i < segs->arr.size(); ++i) {
        const JsonValue& e = segs->arr[i];
        const std::string path = from + ".segments[" + std::to_string(i) + "]";
        Element g(SegmentSchema(), e, path, why, &ok);
        if (!ok) return false;
        Segment s;
        s.kind = g.E("kind");
        s.duration = g.N("duration");
        s.levels = static_cast<int>(g.N("levels"));
        s.from = g.N("from");
        s.up = g.E("up");
        s.base = g.S("base");
        s.upOf = g.S("upOf");
        s.to = g.S("to");
        s.law = g.E("law");
        s.u0 = g.N("u0");
        s.u1 = g.N("u1");
        s.delay = g.N("delay");
        s.level = static_cast<int>(g.N("level"));
        if (const JsonValue* keys = g.List("keys")) {
            for (size_t k = 0; k < keys->arr.size(); ++k) {
                const JsonValue& ke = keys->arr[k];
                Element key(RailKeySchema(), ke, path + ".keys[" + std::to_string(k) + "]", why, &ok);
                if (!ok) return false;
                JsonValue at;
                const bool hasAt = key.Raw("at", at);
                const std::string view = key.S("view"), pose = key.S("pose");
                if (!hasAt && view.empty() && pose.empty()) {
                    return Refuse(why, key.Path() + ": a key needs `at`, `view` or `pose`");
                }
                s.keyT.push_back(key.N("t"));
                s.keyAt.push_back(at);
                s.keyHasAt.push_back(hasAt);
                s.keyView.push_back(view);
                s.keyPose.push_back(pose);
            }
        }
        if (s.kind == kKeys && s.keyT.empty()) return Refuse(why, path + ": a keys segment needs keys");
        if (s.kind != kKeys && s.base.empty()) return Refuse(why, path + ": needs a `base` pose");
        if (s.kind == kTurn && s.to.empty()) return Refuse(why, path + ": a turn needs `to`");
        if ((s.kind == kSpiral || s.kind == kTurn) && s.upOf.empty()) s.upOf = s.base;
        if (s.kind == kHold && s.upOf.empty()) s.upOf = s.base;
        m_segments.push_back(std::move(s));
    }
    m_doc = std::move(doc);
    return true;
}

// ---- the resolution -------------------------------------------------------------------------------

const RailPose* Rail::Pose(const std::string& name) const {
    for (const RailPose& p : m_poses) {
        if (p.name == name) return &p;
    }
    return nullptr;
}

bool Rail::ResolvePose(const PoseDecl& d, const RailFrame& frame, RailPose& out,
                       std::string* why) const {
    out.name = d.name;
    if (!d.turnOf.empty()) {
        // The other face of a pose: turned about the local vertical to look back, a little
        // down at the water -- the hand code's `drosteHelmBack`, float arithmetic and all.
        const RailPose* b = Pose(d.turnOf);
        if (!b) return Refuse(why, m_path + ".poses." + d.name + ".turnOf: no pose '" + d.turnOf + "' before it");
        out = *b;
        out.name = d.name;
        // Said as before in the world's own axes: the base's compass yaw turned, the pitch set.
        const DirectX::XMFLOAT3 bf = b->cam.Forward();
        const double yaw = std::atan2(double(bf.z), double(bf.x)) + d.turnYaw, pitch = d.turnPitch;
        const double f[3] = {std::cos(yaw) * std::cos(pitch), std::sin(pitch), std::sin(yaw) * std::cos(pitch)};
        out.cam.Aim(f);
    } else {
        PoseSugar s;
        if (!ReadPoseSugar(d.at, m_path + ".poses." + d.name + ".at", s, why)) return false;
        Camera c;
        switch (s.kind) {
            case PoseSugar::Kind::Compass:
                c.SetFromCompass(s.x, s.alt, s.z, static_cast<float>(s.az), static_cast<float>(s.pitch));
                break;
            case PoseSugar::Kind::Orbit: {
                const Camera g = LatLonPose(s.lat, s.lon, s.alt, s.lookAt, s.tLat, s.tLon, s.heading, s.tilt,
                                            s.roll, s.range, frame.frame.planetR);
                c = PlanetToFlatPose(g, frame.frame.east, frame.frame.up, frame.frame.north,
                                     frame.frame.planetR);
                break;
            }
            default: {
                Motor m;
                if (!ResolveMotorSugar(d.at, frame.frame, m_path + ".poses." + d.name + ".at", m, why)) return false;
                ToCamera(m, c);
                break;
            }
        }
        c.fovY = frame.fovY;
        const bool portalOk = frame.portal && frame.portal->Valid();
        if (d.aim == 1) {
            // WHICH POSE THE SPIRAL RUNS THROUGH (M10, measured): the helm's spiral clears the
            // inner globe only if the twist lifts it up and over; below the declared twist the
            // eye stands off the fixed point instead and dives from above.
            if (portalOk && std::abs(frame.twistDeg) < d.standOffBelowTwistDeg) {
                c.px = frame.portal->p[0] + d.standOff[0];
                c.py = frame.portal->p[1] + d.standOff[1];
                c.pz = frame.portal->p[2] + d.standOff[2];
            }
            if (portalOk) {
                const double radial[3] = {c.px, c.py + frame.frame.planetR, c.pz};
                c.SetUp(radial);   // the up first: LookAt levels against it
                c.LookAt(frame.portal->p[0], frame.portal->p[1], frame.portal->p[2]);
                DirectX::XMFLOAT3 hf, hr, hu;
                c.ViewBasis(hf, hr, hu);   // the helm's TRUE up: the roll the spiral carries
                out.up0[0] = hu.x;
                out.up0[1] = hu.y;
                out.up0[2] = hu.z;
            }
        }
        out.cam = c;
    }
    out.motor = FromCamera(out.cam);
    out.c0[0] = out.cam.px;
    out.c0[1] = out.cam.py;
    out.c0[2] = out.cam.pz;
    const DirectX::XMFLOAT3 hf = out.cam.Forward();
    out.f0[0] = hf.x;
    out.f0[1] = hf.y;
    out.f0[2] = hf.z;
    return true;
}

bool Rail::KeyMotor(const Segment& s, size_t i, const RailFrame& frame, Motor& out,
                    std::string* why) const {
    const std::string path = m_path + ".segments.keys[" + std::to_string(i) + "]";
    if (!s.keyPose[i].empty()) {
        const RailPose* p = Pose(s.keyPose[i]);
        if (!p) return Refuse(why, path + ".pose: no pose '" + s.keyPose[i] + "'");
        out = p->motor;
        return true;
    }
    if (!s.keyView[i].empty()) {
        // The eye a view declares. The orbit view's engine default -- over the North Atlantic
        // at 2.1 planet radii, aimed at the centre -- stands in when it declares none, which is
        // what the session's camGlobe was before the view's own `at` overrode it.
        const JsonValue* at = frame.viewAt ? frame.viewAt(s.keyView[i]) : nullptr;
        if (at) return ResolveMotorSugar(*at, frame.frame, path + ".view", out, why);
        if (s.keyView[i] != "orbit") {
            return Refuse(why, path + ".view: view '" + s.keyView[i] + "' declares no eye");
        }
        const double R = frame.frame.planetR;
        const Camera g = GlobeCamera(34.0, -52.0, R * 2.1, R);
        out = FromCamera(PlanetToFlatPose(g, frame.frame.east, frame.frame.up, frame.frame.north, R));
        return true;
    }
    return ResolveMotorSugar(s.keyAt[i], frame.frame, path + ".at", out, why);
}

bool Rail::Resolve(const RailFrame& frame, std::string* why) {
    m_frame = frame;
    m_resolved = false;
    m_poses.clear();
    m_poses.reserve(m_poseDecls.size());   // the segments hold pointers into it
    for (const PoseDecl& d : m_poseDecls) {
        RailPose p;
        if (!ResolvePose(d, frame, p, why)) return false;
        m_poses.push_back(p);
    }
    double acc = 0.0;
    for (Segment& s : m_segments) {
        s.keys.clear();
        for (size_t i = 0; i < s.keyT.size(); ++i) {
            Motor m;
            if (!KeyMotor(s, i, frame, m, why)) return false;
            s.keys.push_back({s.keyT[i], m});
        }
        s.durationS = (s.levels > 0) ? static_cast<double>(s.levels) * frame.levelSec : s.duration;
        s.start = acc;
        if (s.durationS > 0.0) acc += s.durationS;
        auto find = [&](const std::string& name, const char* what, const RailPose*& out) {
            out = nullptr;
            if (name.empty()) return true;
            out = Pose(name);
            if (out) return true;
            return Refuse(why, m_path + ".segments." + what + ": no pose '" + name + "'");
        };
        if (!find(s.base, "base", s.basePose) || !find(s.upOf, "upOf", s.upPose) ||
            !find(s.to, "to", s.toPose)) {
            return false;
        }
    }
    m_resolved = true;
    return true;
}

// ---- the laws ----------------------------------------------------------------------------------

double Rail::DiveU(double tau, double levelSec, int levels, bool clamp) {
    const double T = levelSec, ramp = 3.0;
    if (tau <= 0.0) return 0.0;
    const double u = (tau < ramp) ? tau * tau / (2.0 * ramp * T) : (tau - 0.5 * ramp) / T;
    // The dive rail ends ON a helm (a whole level): it holds there for the last frames.
    return clamp ? (std::min)(u, double(levels)) : u;
}

double Rail::LegU(double tau, double D, double dU) {
    const double r = (std::min)(3.0, 0.5 * D);
    const double v = dU / (D - r);
    if (tau <= 0.0) return 0.0;
    if (tau >= D) return dU;
    if (tau < r) return v * tau * tau / (2.0 * r);
    if (tau <= D - r) return v * (tau - 0.5 * r);
    const double e = D - tau;
    return dU - v * e * e / (2.0 * r);
}

void Rail::KeyedPose(const std::vector<std::pair<double, Motor>>& keys, double t, double eye[3],
                     double fwd[3], Motor* motor) {
    size_t i = 0;
    while (i + 1 < keys.size() && keys[i + 1].first <= t) ++i;
    Motor m;
    if (i + 1 >= keys.size()) {
        m = keys.back().second;
    } else {
        const double t0 = keys[i].first, t1 = keys[i + 1].first;
        double u = (t - t0) / std::max(t1 - t0, 1e-6);
        u = u * u * (3.0 - 2.0 * u);   // ease both ends of every leg
        m = Motor::Slerp(keys[i].second, keys[i + 1].second, u);
    }
    double px = 0, py = 0, pz = 0;
    m.TransformPoint(px, py, pz);
    double fx = 1, fy = 0, fz = 0;
    m.TransformDir(fx, fy, fz);
    eye[0] = px;
    eye[1] = py;
    eye[2] = pz;
    fwd[0] = fx;
    fwd[1] = fy;
    fwd[2] = fz;
    if (motor) *motor = m;
}

void Rail::GravityUp(const double eye[3], double planetR, double up[3]) {
    const double gy = eye[1] + planetR;
    const double gl = std::sqrt(eye[0] * eye[0] + gy * gy + eye[2] * eye[2]);
    up[0] = eye[0] / gl;
    up[1] = gy / gl;
    up[2] = eye[2] / gl;
}

void Rail::AimCamera(const RailSample& s, Camera& cam) {
    if (s.spiral && s.base) cam = s.base->cam;
    cam.px = s.eye[0];
    cam.py = s.eye[1];
    cam.pz = s.eye[2];
    cam.Aim(s.fwd);
}

// ---- the evaluation ---------------------------------------------------------------------------------

RailSample Rail::Eval(const Segment& s, double t, double tau) const {
    RailSample r;
    switch (s.kind) {
        case kKeys:
            KeyedPose(s.keys, s.from + tau, r.eye, r.fwd, &r.motor);
            r.level = 0;
            if (s.up == 1) {
                GravityUp(r.eye, m_frame.frame.planetR, r.up);
                r.upWritten = true;
            }
            break;
        case kSpiral:
        case kHold: {
            // S^u(base), written in level floor(u): the pose, its level, and its up.
            double u = 0.0;
            if (s.kind == kSpiral) {
                u = (s.law == kDive)
                        ? DiveU(t - (s.start + s.delay), m_frame.levelSec, m_frame.levels,
                                m_frame.clampLevels)
                        : s.u0 - LegU(tau, s.durationS, s.u0 - s.u1);
            }
            const double n = std::floor(u);
            const double f = u - n;
            if (!m_frame.cycle) {
                // No tower to fly through: the base pose stands (the frame loop flies KeysAt
                // when the portal is invalid; this is the sample a caller gets regardless).
                for (int i = 0; i < 3; ++i) {
                    r.eye[i] = s.basePose->c0[i];
                    r.fwd[i] = s.basePose->f0[i];
                    r.up[i] = s.upPose->up0[i];
                }
                r.upWritten = true;
                r.base = s.basePose;
                r.upOf = s.upPose;
                break;
            }
            const Space& cycle = *m_frame.cycle;
            cycle.LevelApply(f, s.basePose->c0, r.eye);
            cycle.LevelApplyDir(f, s.basePose->f0, r.fwd);
            cycle.LevelApplyDir(f, s.upPose->up0, r.up);
            r.level = static_cast<int>(n);
            r.upWritten = true;
            r.spiral = true;
            r.u = u;
            r.f = f;
            r.base = s.basePose;
            r.upOf = s.upPose;
            break;
        }
        case kTurn: {
            const std::vector<std::pair<double, Motor>> two = {{0.0, s.basePose->motor},
                                                               {s.durationS, s.toPose->motor}};
            KeyedPose(two, tau, r.eye, r.fwd, &r.motor);
            r.level = s.level;
            for (int i = 0; i < 3; ++i) r.up[i] = s.upPose->up0[i];
            r.upWritten = true;
            break;
        }
        default: break;
    }
    return r;
}

RailSample Rail::At(double t) const {
    double tau = t;
    const size_t n = m_segments.size();
    for (size_t i = 0; i < n; ++i) {
        const Segment& s = m_segments[i];
        if (s.durationS > 0.0 && i + 1 < n && tau >= s.durationS) {
            tau -= s.durationS;
            continue;
        }
        return Eval(s, t, tau);
    }
    return RailSample{};
}

RailSample Rail::KeysAt(double t) const {
    RailSample r;
    for (const Segment& s : m_segments) {
        if (s.kind != kKeys || s.keys.empty()) continue;
        KeyedPose(s.keys, t, r.eye, r.fwd, &r.motor);
        return r;
    }
    return r;
}

}  // namespace ga::scene
