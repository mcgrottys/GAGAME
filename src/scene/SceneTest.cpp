// RunSceneSelfTest -- the gate on the scene's data structures (M12 step 5a): the registry
// template, the property table, the fold with override, the placement sugar, the writer's
// round trip, the refusals, the Composite, and the Options shim.
//
// What is pinned, and against what:
//   1. Registry<T> answers exactly as VesselRegistry and LoaderRegistry do on the built-in
//      kinds: the same names in the same order, the same product for the same inputs (every
//      ledger number of every hull; a loader's name, structure, grade and georeference), the
//      same empty answer for a miss.
//   2. Props: the current value is the default; "12 kn" converts through GaUnits; a Velocity
//      into a Length REFUSES with AcceptsFrom's own why and the key's path; a levelled height
//      without its datum refuses; {v, unit, src} keeps its provenance; an unknown key refuses
//      naming the path; ApplyTo writes every field kind; Diff names what changed and flags the
//      Restart-bound key; ToJson -> WriteJson -> Parse -> Merge is the identity.
//   3. The fold with override on an in-memory document: defaults < base < include < --set,
//      objects merging by key, a placement replaced WHOLE (the atomic law), scalars replacing,
//      a named array merging in place / removing / appending in file order, completion filling
//      an element's declared keys from its prototype.
//   4. The placement sugar's four spellings resolve to the SAME Placement as the session's own
//      functions (scene/Pose.h: the compass pose = SetFromCompass then FromCamera; the orbit
//      key = OrbitPose + PlanetToFlatPose + FromCamera; the globe start camera; a motor's own
//      coefficients; Placement::Similar), bit for bit -- they are the same code path.
//   5. WriteJson(Resolved()) round-trips byte for byte, on the test document, on the struct
//      defaults, and on scenes/merrimack.json when the file is present.
//   6. Refusals carry the node path: an unknown key, an unknown node type (through a test
//      component registered for the purpose), a unit refusal inside an effect, a malformed
//      placement, a nameless element of a named list, a --set into an include; a same-named
//      entry within one file merges in place (the named-array law, not a refusal).
//   7. Node: ResolveAlong is Space::ToRoot's fold, Walk is file order with pruning, Find and
//      Path, a component wired through Configure/Apply.
//   9. [view] (M12 step 5b) View::Level, the re-levelling of a motor against an up FIELD, is
//      introduced BESIDE the Euler extraction and must be proven equal to it before anything
//      reads it: (A) at the six recipe poses and a thousand random ones it reproduces the frame
//      the rasterizer builds today -- scene::ToCamera then Camera::ViewBasis -- to the float
//      basis' own resolution; (B) with the world's up as the field it reproduces `poseMotor`
//      itself, strictly, wherever the pose is outside ViewBasis's degeneracy blend; (C) it is
//      invariant to a roll of +-30 degrees about the aim, which is what "the roll is removed by
//      construction" means; (D) it is idempotent; (E) MotorOf inverts Placement::Rigid, which is
//      how the scene's placement sugar reaches a pose; (F) View::FromCamera then ToCamera is the
//      identity on a Camera, optics included.
//   8. The Options shim: --boat without --campos is REFUSED (the plan's one deliberate
//      non-identity); the implication laws of ParseArgs reach the sets (--rail-flood: headless,
//      1200 frames, the orbit view, the flood rail); a still's flags reach the active view.
// A defect planted in MergeInto (the atomic law dropped, so an overlay's {lat, lon, alt}
// merged into {x, alt, z, az, pitch} by key) was seen to fail block 3 before this gate was
// trusted (priors 22).
#include "app/Options.h"
#include "core/Common.h"
#include "core/CurrentFieldLoader.h"
#include "core/FieldLoader.h"
#include "core/GeoGridLoader.h"
#include "core/Registry.h"
#include "render/Camera.h"
#include "scene/Component.h"
#include "scene/Node.h"
#include "scene/Pose.h"
#include "scene/Props.h"
#include "scene/SceneBuilder.h"
#include "scene/SceneSchema.h"
#include "scene/View.h"
#include "sim/BathyModel.h"
#include "sim/GlobeModel.h"
#include "sim/VesselSpec.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <string>

namespace ga::scene {

namespace {

struct Gate {
    bool ok = true;
    int checks = 0;
    void True(bool v, const char* what) {
        ++checks;
        if (v) return;
        Log("[scene] FAIL %s", what);
        ok = false;
    }
    void Near(double a, double b, double tol, const char* what) {
        ++checks;
        if (std::fabs(a - b) <= tol) return;
        Log("[scene] FAIL %s: %.17g vs %.17g (tol %g)", what, a, b, tol);
        ok = false;
    }
    void Same(double a, double b, const char* what) {
        ++checks;
        if (a == b) return;
        Log("[scene] FAIL %s: %.17g vs %.17g (bitwise)", what, a, b);
        ok = false;
    }
    void Has(const std::string& text, const char* needle, const char* what) {
        ++checks;
        if (text.find(needle) != std::string::npos) return;
        Log("[scene] FAIL %s: '%s' does not mention '%s'", what, text.c_str(), needle);
        ok = false;
    }
};

JsonValue ParseOrDie(Gate& g, const char* text) {
    std::string err;
    JsonValue v = JsonParser::Parse(text, &err);
    g.True(err.empty(), "test JSON parses");
    return v;
}

void SamePlacement(Gate& g, const Placement& a, const Placement& b, const char* what) {
    g.Same(a.s, b.s, what);
    for (int i = 0; i < 4; ++i) g.Same(a.r[i], b.r[i], what);
    for (int i = 0; i < 3; ++i) g.Same(a.t[i], b.t[i], what);
}

// ---- a test type for the property table ------------------------------------------------------

struct TestProps {
    bool on = true;
    float f = 1.15f;
    double len = 7.0;          // m
    double speed = 12.0;       // kn
    double navd = -30.0;       // m NAVD88
    int mode = 1;              // a | b | c
    std::string label = "x";
    double v[3] = {1.0, 2.0, 3.0};
    Motor pose;
    Placement link;
    float color[4] = {1.0f, 0.0f, 0.0f, 1.0f};
    std::string path = "a/b";
    uint32_t frames = 240;
    struct Sub {
        double d = 2.0;
        bool r = false;
    } sub;
};

TestProps kProto;

const Schema& SubSchema() {
    static const Schema* s = [] {
        Schema* sc = new Schema("t.sub", &kProto.sub);
        sc->Bind("d", kProto.sub.d, Quantity::Length, "m", "a nested length", Reload::Hot)
            .Bind("r", kProto.sub.r, "a nested flag", Reload::Restart);
        return sc;
    }();
    return *s;
}

const Schema& TestSchema() {
    static const Schema* s = [] {
        Schema* sc = new Schema("t", &kProto);
        sc->Bind("on", kProto.on, "a flag", Reload::Hot)
            .Bind("f", kProto.f, Quantity::Dimensionless, "1", "a float", Reload::Hot)
            .Bind("len", kProto.len, Quantity::Length, "m", "a length", Reload::Hot)
            .Bind("speed", kProto.speed, Quantity::Velocity, "kn", "a speed in knots", Reload::Hot)
            .Bind("navd", kProto.navd, Quantity::Length, "m NAVD88", "a levelled height", Reload::Hot)
            .BindEnum("mode", kProto.mode, {"a", "b", "c"}, "an enum", Reload::Hot)
            .Bind("label", kProto.label, "a string", Reload::Hot)
            .Bind("v", kProto.v, Quantity::Length, "m", "a vector", Reload::Hot)
            .Bind("pose", kProto.pose, "a motor", Reload::Hot)
            .Bind("link", kProto.link, "a similarity", Reload::Hot)
            .BindColor("color", kProto.color, "a colour", Reload::Hot)
            .BindPath("path", kProto.path, "a path", Reload::Hot)
            .Bind("frames", kProto.frames, Quantity::Dimensionless, "frames", "a count", Reload::Restart)
            .Nest("sub", SubSchema(), &kProto.sub, "a nested section");
        return sc;
    }();
    return *s;
}

// ---- a test component ---------------------------------------------------------------------------

struct SpinProps {
    double rate = 1.0;   // deg/s
};
SpinProps kSpin;
const Schema& SpinSchema() {
    static const Schema* s = [] {
        Schema* sc = new Schema("test.spin", &kSpin);
        sc->Bind("rate", kSpin.rate, Quantity::Angle, "deg", "the spin per second", Reload::Hot);
        return sc;
    }();
    return *s;
}

class SpinComponent final : public Component {
public:
    const char* Name() const override { return "test.spin"; }
    const Schema& Props() const override { return SpinSchema(); }
    std::vector<std::string> Configure(const Wiring& w) override {
        std::vector<std::string> missing;
        if (!w.gpu) missing.push_back("gpu");
        return missing;
    }
    bool Init(Gpu&) override { return true; }
    void Apply(const PropSet& props) override {
        std::string why;
        props.ApplyTo(&p, nullptr, &why);
    }
    void Update(const FrameInfo& f) override { angle += p.rate * f.dt; }
    void Record(const ViewContext&) override {}
    void ReloadShaders() override {}
    SpinProps p;
    double angle = 0.0;
};

}  // namespace

bool RunSceneSelfTest() {
    Gate g;
    RegisterBuiltinSceneTypes();
    const double R = GlobeModel::kR;

    // ---- 1. Registry<T> against the two hand-written registries ----------------------------
    {
        VesselRegistry vr;
        RegisterBuiltinVessels(vr);
        Registry<VesselSpec> tr("vessel");
        tr.Register("box.test", MakeTestBox);
        tr.Register("rhib.novurania18", MakeRhib18);
        const auto kinds = vr.Kinds();
        g.True(kinds == tr.Names(), "Registry<VesselSpec>::Names() == VesselRegistry::Kinds()");
        g.True(kinds.size() == 2, "two built-in hulls");
        for (const std::string& k : kinds) {
            g.True(vr.Knows(k) && tr.Knows(k), "both registries know the kind");
            const VesselSpec a = vr.Build(k), b = tr.Make(k);
            g.True(a.kind == b.kind && a.display == b.display, "the same spec identity");
            g.True(a.elements.size() == b.elements.size(), "the same element count");
            const auto la = a.Ledger(), lb = b.Ledger();
            g.True(la.size() == lb.size(), "the same ledger length");
            for (size_t i = 0; i < la.size() && i < lb.size(); ++i) {
                g.True(la[i].first == lb[i].first && la[i].second->v == lb[i].second->v &&
                           la[i].second->raw == lb[i].second->raw &&
                           la[i].second->unit == lb[i].second->unit &&
                           la[i].second->src == lb[i].second->src,
                       "the same ledger number (value, raw, unit, source)");
            }
        }
        g.True(!vr.Knows("nope") && !tr.Knows("nope"), "neither knows a miss");
        g.True(vr.Build("nope").kind.empty() && tr.Make("nope").kind.empty(),
               "a miss is the empty spec from both");
    }
    {
        LoaderRegistry lr;
        lr.Register("json", CurrentFieldLoader::Open);
        lr.Register("f32", GeoGridLoader::Open);
        Registry<std::unique_ptr<FieldLoader>, const std::string&, const GeoRef*> tr("loader");
        tr.Register("json", CurrentFieldLoader::Open);
        tr.Register("f32", GeoGridLoader::Open);
        g.True(lr.Types() == tr.Names(), "Registry<loader>::Names() == LoaderRegistry::Types()");
        g.True(lr.Knows("JSON") && tr.Knows("json"), "both know json");
        const std::string path = "data/currents/currents.json";
        auto a = lr.Open(path);
        auto b = tr.Make("json", path, nullptr);
        g.True((a == nullptr) == (b == nullptr), "the same answer for the same file");
        if (a && b) {
            g.True(strcmp(a->Name(), b->Name()) == 0 && strcmp(a->Structure(), b->Structure()) == 0,
                   "the same loader name and structure");
            g.True(a->GradeSig() == b->GradeSig() && a->Channels() == b->Channels(),
                   "the same grade and channels");
            const GeoRef &ra = a->Ref(), &rb = b->Ref();
            g.True(ra.width == rb.width && ra.height == rb.height && ra.originX == rb.originX &&
                       ra.originY == rb.originY && ra.scaleX == rb.scaleX && ra.scaleY == rb.scaleY,
                   "the same georeference");
        } else {
            Log("[scene] note: %s absent here; the loader products compare as two nulls", path.c_str());
        }
        g.True(lr.Open("x.tif") == nullptr && tr.Make("tif", "x.tif", nullptr) == nullptr,
               "a miss is nullptr from both");
    }

    // ---- 2. Props: the table read four ways ------------------------------------------------
    {
        const Schema& s = TestSchema();
        TestProps inst;
        inst.f = 2.0f;   // the current value is the default: not the prototype's 1.15
        PropSet defaults = PropSet::Defaults(s, &inst);
        g.Same(defaults.Get("f")->n, 2.0, "Defaults reads the instance, not the prototype");
        g.Same(PropSet::Defaults(s, &kProto).Get("f")->n, 1.15, "a float default prints as the short decimal");
        g.True(defaults.Get("mode")->s == "b", "an enum default is its name");
        g.True(defaults.Object("sub") != nullptr, "a nested section is a PropSet");
        const JsonValue dj = defaults.ToJson();
        g.True(dj.obj.size() == s.Decls().size(), "ToJson has every declared key");

        PropSet ps = defaults;
        std::string why;
        const JsonValue doc = ParseOrDie(g,
            "{\"on\": false, \"f\": 2.5, \"len\": \"30 ft\", \"speed\": \"3 m/s\", \"mode\": \"c\","
            " \"label\": \"y\", \"v\": [4, 5, \"6 m\"],"
            " \"pose\": {\"x\": 1, \"alt\": 2, \"z\": 3, \"az\": 90, \"pitch\": 0},"
            " \"link\": {\"similarity\": {\"p\": [1, 2, 3], \"s\": 0.5, \"axis\": [0, 0, 1], \"twistDeg\": 90}},"
            " \"color\": [0, 1, 0], \"path\": \"c\", \"frames\": 12,"
            " \"sub\": {\"d\": 3, \"_note\": \"a comment\"}, \"_comment\": \"ignored\"}");
        const bool merged = ps.Merge(doc, "t", &why);
        g.True(merged, (std::string("Merge accepts the document: ") + why).c_str());
        g.Near(ps.Get("len")->n, 30.0 * 0.3048, 1e-12, "30 ft into a metre field is 9.144");
        g.Near(ps.Get("speed")->n, 3.0 / 0.514444, 1e-9, "3 m/s into a knot field converts");
        g.Same(ps.Get("v")->v[2], 6.0, "a vector element with a unit");
        g.True(ps.Get("mode")->e == 2 && ps.Get("mode")->s == "c", "an enum by name");
        g.Same(ps.Object("sub")->Get("d")->n, 3.0, "a nested key merged");

        PropSet bad = defaults;
        g.True(!bad.Merge(ParseOrDie(g, "{\"len\": \"5 kn\"}"), "t", &why), "a Velocity into a Length refuses");
        g.Has(why, "quantity velocity into length", "the refusal is AcceptsFrom's why");
        g.Has(why, "t.len", "the refusal names the path");
        g.True(!bad.Merge(ParseOrDie(g, "{\"navd\": \"-30 m\"}"), "t", &why), "a height without its datum refuses");
        g.Has(why, "vertical datum (none) into NAVD88", "the datum refusal is AcceptsFrom's");
        g.True(bad.Merge(ParseOrDie(g, "{\"navd\": \"-98.4 ft NAVD88\"}"), "t", &why), "the same datum converts");
        g.Near(bad.Get("navd")->n, -98.4 * 0.3048, 1e-12, "feet NAVD88 into metres NAVD88");
        g.True(!bad.Merge(ParseOrDie(g, "{\"bogus\": 1}"), "t", &why), "an unknown key refuses");
        g.Has(why, "t.bogus", "the unknown key names its path");
        g.Has(why, "unknown key", "and says so");
        g.True(!bad.Merge(ParseOrDie(g, "{\"mode\": \"z\"}"), "t", &why), "an enum outside its names refuses");
        g.True(!bad.Merge(ParseOrDie(g, "{\"pose\": {\"similarity\": {\"p\": [0,0,0], \"s\": 2, \"axis\": [0,1,0], \"twistDeg\": 0}}}"), "t", &why),
               "a motor refuses the similarity spelling");
        g.True(bad.Merge(ParseOrDie(g, "{\"len\": {\"v\": 30, \"unit\": \"ft\", \"src\": \"the survey\"}}"), "t", &why),
               "the {v, unit, src} spelling");
        g.Near(bad.Get("len")->n, 9.144, 1e-12, "and converts");
        g.True(bad.Get("len")->src == "the survey", "and keeps its provenance");

        TestProps out;
        PoseFrame fr = FrameFromAnchor(BathyModel::kOrgLat, BathyModel::kOrgLon, R);
        const bool applied = ps.ApplyTo(&out, &fr, &why);
        g.True(applied, (std::string("ApplyTo: ") + why).c_str());
        g.True(!out.on && out.f == 2.5f && out.mode == 2 && out.label == "y" && out.path == "c" &&
                   out.frames == 12u && out.sub.d == 3.0 && out.color[1] == 1.0f && out.color[3] == 1.0f,
               "ApplyTo wrote every field kind");
        Camera c;
        c.SetFromCompass(1.0, 2.0, 3.0, 90.0f, 0.0f);
        const Motor want = FromCamera(c);
        g.Same(out.pose.s, want.s, "the motor field is FromCamera's");
        g.Same(out.pose.t01, want.t01, "the motor field is FromCamera's (dual)");
        const double p3[3] = {1, 2, 3}, ax[3] = {0, 0, 1};
        SamePlacement(g, out.link, Placement::Similar(p3, 0.5, ax, 90.0 * 3.14159265358979 / 180.0),
                      "the similarity field is Placement::Similar");

        const auto changes = ps.Diff(defaults);
        bool sawFrames = false, sawSubD = false, sawRestart = false;
        for (const auto& ch : changes) {
            if (ch.key == "frames") { sawFrames = true; sawRestart = ch.reload == Reload::Restart; }
            if (ch.key == "sub.d") sawSubD = true;
        }
        g.True(sawFrames && sawRestart, "Diff flags the Restart-bound key");
        g.True(sawSubD, "Diff descends into a nested section");
        g.True(changes.size() == 13, "Diff counts exactly the keys that changed");

        const std::string text = SceneBuilder::WriteJson(ps.ToJson());
        PropSet again = PropSet::Defaults(s, &inst);
        const bool readBack = again.Merge(ParseOrDie(g, text.c_str()), "t", &why);
        g.True(readBack, (std::string("the written values read back: ") + why).c_str());
        g.True(again.Diff(ps).empty(), "ToJson -> WriteJson -> Parse -> Merge is the identity");
    }

    // ---- 3. The fold with override --------------------------------------------------------
    SceneBuilder built;
    {
        std::string why;
        const JsonValue base = ParseOrDie(g,
            "{\"scene\": {\"name\": \"t\"},"
            " \"views\": [{\"name\": \"a\", \"at\": {\"x\": 1, \"alt\": 2, \"z\": 3, \"az\": 4, \"pitch\": 5}, \"fovY\": 50},"
            "            {\"name\": \"b\"}],"
            " \"portals\": [{\"name\": \"p\", \"level\": 12}],"
            " \"water\": {\"foam\": 0.5, \"swe\": {\"gain\": 2}}}");
        const JsonValue overlay = ParseOrDie(g,
            "{\"views\": [{\"name\": \"a\", \"at\": {\"lat\": 1, \"lon\": 2, \"alt\": 3}},"
            "            {\"name\": \"b\", \"remove\": true}, {\"name\": \"c\"}],"
            " \"scene\": {\"planet\": \"mars\"}, \"water\": {\"swe\": {\"spinupH\": \"30 min\"}}}");
        g.True(built.Overlay(base, "base", "", &why), "the base overlays the defaults");
        g.True(built.Overlay(overlay, "include", "", &why), "the include overlays the base");
        g.True(built.Set("views.a.fovY", JsonNum(60), &why), "--set on a named element");
        g.True(built.SetText("scene.view=c", &why), "--set with a bare string");
        g.True(built.SetText("portals.p.twistDeg=45", &why), "--set into a portal");
        g.True(built.SetText("capture.headless=true", &why), "--set a bool");
        g.True(built.SetText("tools.selftest={}", &why), "--set creates a named element");
        g.True(built.SetText("effects.slice={\"type\": \"slice.plane\", \"d\": \"2 m\"}", &why),
               "--set a typed element whole");
        g.True(!built.SetText("include=[]", &why), "--set include is refused");
        const bool resolved = built.Resolve(&why);
        g.True(resolved, (std::string("Resolve: ") + why).c_str());
        const JsonValue& d = built.Resolved();
        g.True(d.obj.front().first == "base" && d.obj[1].first == "scene" && d.obj.back().first == "tools",
               "the sections keep the schema's order");
        const JsonValue* sc = d.Get("scene");
        g.True(sc && sc->Str("name") == "t" && sc->Str("planet") == "mars" && sc->Str("view") == "c",
               "scalars: base, include and --set each replaced their key");
        const JsonValue* views = d.Get("views");
        g.True(views && views->arr.size() == 2, "named array: b removed, c appended");
        if (views && views->arr.size() == 2) {
            const JsonValue& a = views->arr[0];
            g.True(a.Str("name") == "a" && views->arr[1].Str("name") == "c", "file order kept");
            const JsonValue* at = a.Get("at");
            g.True(at && at->obj.size() == 3 && at->Get("lat") && !at->Get("x"),
                   "a placement is replaced WHOLE by an overlay (the atomic law)");
            g.Same(a.Num("fovY", 0), 60.0, "--set beat the base");
            g.Same(views->arr[1].Num("fovY", 0), 55.0, "an appended element is completed from its prototype");
            g.True(views->arr[1].Get("at") == nullptr, "an optional key stays absent");
        }
        const JsonValue* portals = d.Get("portals");
        g.True(portals && portals->arr.size() == 1, "one portal");
        if (portals && portals->arr.size() == 1) {
            const JsonValue& p = portals->arr[0];
            g.Same(p.Num("level", 0), 12.0, "the base's key survives");
            g.Same(p.Num("twistDeg", 0), 45.0, "the --set's key wins");
            g.Same(p.Num("fill", 0), 1.0, "the prototype fills the rest");
        }
        const JsonValue* water = d.Get("water");
        g.True(water && water->Num("foam", 0) == 0.5 && water->Get("swe")->Num("gain", 0) == 2.0,
               "objects merge by key (base keys kept)");
        g.True(water && water->Get("swe")->Get("spinupH")->type == JsonValue::Type::String,
               "a unit string is kept as written in the document");
        g.True(water && water->Get("swe")->Num("riverQ", 0) == -1.0, "and the defaults fill in beside them");
        g.True(d.Get("capture")->Get("headless")->boolean, "--set reached a nested section");
        const JsonValue* tools = d.Get("tools");
        g.True(tools && tools->arr.size() == 1 && tools->arr[0].Str("name") == "selftest" &&
                   tools->arr[0].Get("args") != nullptr,
               "a --set-created element carries its name and its completed keys");
        const JsonValue* effects = d.Get("effects");
        g.True(effects && effects->arr.size() == 1 && effects->arr[0].Str("type") == "slice.plane",
               "a typed element by --set");
    }

    // ---- 4. The placement sugar against the session's functions -------------------------------
    {
        const PoseFrame fr = FrameFromAnchor(BathyModel::kOrgLat, BathyModel::kOrgLon, R);
        g.True(fr.valid, "the anchor's tangent rows are a proper rotation");
        std::string why;
        Placement got;
        // {x, alt, z, az, pitch} = SetFromCompass then FromCamera (the helm still's pose).
        g.True(ResolvePlacement(ParseOrDie(g, "{\"x\": 120, \"alt\": 7, \"z\": -10, \"az\": 92.5, \"pitch\": -1.5}"),
                                fr, got, &why), "the compass spelling resolves");
        Camera c;
        c.SetFromCompass(120.0, 7.0, -10.0, 92.5f, -1.5f);
        SamePlacement(g, got, Placement::Rigid(FromCamera(c)), "compass == SetFromCompass + FromCamera");
        // {lat, lon, alt, lookAt} = the orbit key.
        g.True(ResolvePlacement(ParseOrDie(g, "{\"lat\": 42.74, \"lon\": -70.87, \"alt\": \"7 km\", \"lookAt\": {\"lat\": 42.8183, \"lon\": -70.81}}"),
                                fr, got, &why), "the orbit spelling resolves");
        const Camera orb = PlanetToFlatPose(OrbitPose(42.74, -70.87, 7000.0, 42.8183, -70.81, R),
                                            fr.east, fr.up, fr.north, R);
        SamePlacement(g, got, Placement::Rigid(FromCamera(orb)), "orbit == OrbitPose + PlanetToFlatPose + FromCamera");
        // {lat, lon, alt} = the globe start camera (--globe-cam), aimed at the centre.
        g.True(ResolvePlacement(ParseOrDie(g, "{\"lat\": 34, \"lon\": -52, \"alt\": 13379100}"),
                                fr, got, &why), "the globe-cam spelling resolves");
        const Camera glb = PlanetToFlatPose(GlobeCamera(34.0, -52.0, 13379100.0, R), fr.east, fr.up,
                                            fr.north, R);
        SamePlacement(g, got, Placement::Rigid(FromCamera(glb)), "globe-cam == GlobeCamera + PlanetToFlatPose + FromCamera");
        // {motor: {re, du}} = the motor's own coefficients.
        const double org[3] = {0, 0, 0}, up[3] = {0, 1, 0};
        const Motor m = Motor::Translation(1.0, 2.0, 3.0) * Motor::Rotation(org, up, 0.3);
        double re[4], du[4];
        m.Real(re);
        m.Dual(du);
        char buf[512];
        snprintf(buf, sizeof(buf),
                 "{\"motor\": {\"re\": [%.17g, %.17g, %.17g, %.17g], \"du\": [%.17g, %.17g, %.17g, %.17g]}}",
                 re[0], re[1], re[2], re[3], du[0], du[1], du[2], du[3]);
        g.True(ResolvePlacement(ParseOrDie(g, buf), fr, got, &why), "the motor spelling resolves");
        SamePlacement(g, got, Placement::Rigid(m), "motor == Placement::Rigid(motor)");
        // {similarity: {p, s, axis, twistDeg}} = Placement::Similar.
        g.True(ResolvePlacement(ParseOrDie(g, "{\"similarity\": {\"p\": [10, 0, 5], \"s\": 0.25, \"axis\": [0, 0, 1], \"twistDeg\": 90}}"),
                                fr, got, &why), "the similarity spelling resolves");
        const double p[3] = {10, 0, 5}, ax[3] = {0, 0, 1};
        SamePlacement(g, got, Placement::Similar(p, 0.25, ax, 90.0 * 3.14159265358979 / 180.0),
                      "similarity == Placement::Similar");
        // {x, alt, z} alone: az 90 / pitch 0 = yaw 0, level -- a pure translation for a spawn.
        g.True(ResolvePlacement(ParseOrDie(g, "{\"x\": 120, \"alt\": 0, \"z\": -10}"), fr, got, &why),
               "the compass spelling without az/pitch resolves");
        SamePlacement(g, got, Placement::Rigid(Motor::Translation(120.0, 0.0, -10.0)),
                      "{x, alt, z} == Motor::Translation (the boat's spawn)");
        g.True(!ResolvePlacement(ParseOrDie(g, "{\"x\": 1, \"yaw\": 2}"), fr, got, &why),
               "a foreign key in a placement refuses");
        g.True(!ResolvePlacement(ParseOrDie(g, "{\"lat\": 1, \"lon\": 2, \"alt\": \"3 kn\"}"), fr, got, &why),
               "a unit refusal inside a placement");
        g.Has(why, "velocity into length", "with AcceptsFrom's why");
    }

    // ---- 5. WriteJson round-trips byte for byte -------------------------------------------
    {
        const std::string s1 = SceneBuilder::WriteJson(built.Resolved());
        std::string err;
        const JsonValue p = JsonParser::Parse(s1, &err);
        g.True(err.empty(), "the written document parses");
        g.True(SceneBuilder::WriteJson(p) == s1, "WriteJson(Parse(WriteJson(doc))) == WriteJson(doc)");
        const std::string s2 = SceneBuilder::WriteJson(DefaultDocument());
        g.True(SceneBuilder::WriteJson(JsonParser::Parse(s2, &err)) == s2, "the defaults document round-trips");
        std::string text;
        if (SceneBuilder::ReadFile("scenes/merrimack.json", text, nullptr)) {
            SceneBuilder b;
            std::string why;
            const bool loaded = b.Load("scenes/merrimack.json", &why) && b.Resolve(&why);
            g.True(loaded, (std::string("scenes/merrimack.json loads and resolves: ") + why).c_str());
            const std::string s3 = SceneBuilder::WriteJson(b.Resolved());
            g.True(SceneBuilder::WriteJson(JsonParser::Parse(s3, &err)) == s3,
                   "the shipped scene's resolved document round-trips");
            g.True(b.Resolved().Get("views")->arr.size() >= 3, "the shipped scene carries its views");
        } else {
            Log("[scene] note: scenes/merrimack.json absent here (not the repo root?); its round trip skipped");
        }
    }

    // ---- 6. Refusals name the node path -----------------------------------------------------
    {
        ComponentSchemas().Register("test.spin", [] { return &SpinSchema(); });
        auto refuse = [&](const char* doc, const char* needle, const char* what) {
            SceneBuilder b;
            std::string why;
            const bool ok = b.Overlay(ParseOrDie(g, doc), "t", "", &why) && b.Resolve(&why);
            g.True(!ok, what);
            g.Has(why, needle, what);
        };
        refuse("{\"capture\": {\"frmes\": 3}}", "capture.frmes", "an unknown key names its path");
        refuse("{\"nodes\": [{\"name\": \"buoy\", \"type\": \"nope\"}]}", "nodes.buoy.type",
               "an unknown node type names the node path");
        refuse("{\"nodes\": [{\"name\": \"buoy\", \"type\": \"nope\"}]}", "'nope'", "and the type");
        refuse("{\"nodes\": [{\"name\": \"n\", \"type\": \"test.spin\", \"rte\": 3}]}", "nodes.n.rte",
               "an unknown key of a typed node");
        refuse("{\"nodes\": [{\"name\": \"n\", \"type\": \"test.spin\", \"children\": [{\"name\": \"k\", \"type\": \"x\"}]}]}",
               "nodes.n.children.k.type", "a child's refusal carries the whole path");
        refuse("{\"effects\": [{\"name\": \"s\", \"type\": \"slice.plane\", \"d\": \"3 kn\"}]}",
               "effects.s.d", "a unit refusal inside a typed element");
        refuse("{\"effects\": [{\"name\": \"s\", \"type\": \"slice.plane\", \"d\": \"3 kn\"}]}",
               "velocity into length", "with AcceptsFrom's why");
        refuse("{\"views\": [{\"name\": \"x\", \"at\": {\"x\": 1}}]}", "views.x.at", "a malformed placement");
        refuse("{\"views\": [{\"fovY\": 3}]}", "views[0]", "a nameless element of a named list");
        {
            // Within ONE file a later same-named entry merges into the earlier (the named-array
            // law applies inside an array as between files): one view x, the later keys winning.
            SceneBuilder b;
            std::string why;
            const bool ok = b.Overlay(ParseOrDie(g,
                "{\"views\": [{\"name\": \"x\", \"fovY\": 40}, {\"name\": \"x\", \"nearZ\": 1}]}"), "t", "", &why) && b.Resolve(&why);
            const JsonValue* views = ok ? b.Resolved().Get("views") : nullptr;
            g.True(views && views->arr.size() == 1, "a same-named entry in one file merges in place");
            g.True(views && views->arr[0].Num("fovY", 0) == 40.0 && views->arr[0].Num("nearZ", 0) == 1.0,
                   "the earlier keys kept, the later keys added");
        }
        refuse("{\"layers\": [{\"name\": \"fog\"}]}", "layers.fog.name", "an unknown layer");
        refuse("{\"scene\": {\"mode\": \"orbit\"}}", "scene.mode", "an enum outside its names");
        {
            SceneBuilder b;
            std::string why;
            const bool ok = b.Overlay(ParseOrDie(g,
                "{\"nodes\": [{\"name\": \"n\", \"type\": \"test.spin\", \"rate\": \"0.5 rad\","
                " \"children\": [{\"name\": \"k\", \"type\": \"test.spin\"}]}]}"), "t", "", &why) && b.Resolve(&why);
            g.True(ok, (std::string("a known node type with its keys resolves: ") + why).c_str());
            const JsonValue* n = ok ? &b.Resolved().Get("nodes")->arr[0] : nullptr;
            g.True(n && n->Get("children")->arr.size() == 1 && n->Get("children")->arr[0].Num("rate", 0) == 1.0,
                   "a child is completed from its type's prototype");
            g.True(n && n->Get("enabled") && n->Get("at") == nullptr, "a node's optional placement stays absent");
        }
    }

    // ---- 7. Node: the Composite, the fold, the walk -----------------------------------------
    {
        Node root("root");
        auto a = std::make_unique<Node>("a");
        auto b = std::make_unique<Node>("b");
        const double org[3] = {0, 0, 0}, up[3] = {0, 1, 0}, p[3] = {5, 0, 0}, ax[3] = {0, 0, 1};
        a->Placement().link = Placement::Rigid(Motor::Translation(1, 2, 3) * Motor::Rotation(org, up, 0.4));
        b->Placement().link = Placement::Similar(p, 0.5, ax, 0.2);
        Node& na = root.AddChild(std::move(a));
        Node& nb = na.AddChild(std::move(b));
        root.AddChild(std::make_unique<Node>("c"));
        SamePlacement(g, ResolveAlong(nb), na.Placement().link.Then(nb.Placement().link),
                      "ResolveAlong is the fold root->node (identity at the root)");
        g.True(nb.Path() == "root.a.b" && root.Find("b") == &nb && root.Find("z") == nullptr,
               "Path and Find");
        std::string order;
        const Node& croot = root;
        Walk(croot, [&](const Node& n, int depth) {
            order += n.Name() + std::to_string(depth) + " ";
            return true;
        });
        g.True(order == "root0 a1 b2 c1 ", "Walk is depth-first in file order");
        order.clear();
        Walk(croot, [&](const Node& n, int) {
            order += n.Name() + " ";
            return n.Name() != "a";   // prune a's subtree
        });
        g.True(order == "root a c ", "a visitor prunes by returning false");
        auto comp = std::make_unique<SpinComponent>();
        SpinComponent& spin = *comp;
        na.AddComponent(std::move(comp));
        g.True(na.Components().size() == 1 && strcmp(na.Components()[0]->Name(), "test.spin") == 0,
               "a component is owned by its node");
        Wiring w;
        g.True(spin.Configure(w).size() == 1 && spin.Configure(w)[0] == "gpu", "Configure reports what is missing");
        PropSet ps = PropSet::Defaults(SpinSchema(), &kSpin);
        std::string why;
        g.True(ps.Merge(ParseOrDie(g, "{\"rate\": \"0.5 rad\"}"), "n", &why), "a component prop with a unit");
        spin.Apply(ps);
        g.Near(spin.p.rate, 0.5 * 180.0 / 3.14159265358979, 1e-12, "Apply wrote the field in its declared unit");
        FrameInfo f;
        f.dt = 2.0;
        spin.Update(f);
        g.Near(spin.angle, 2.0 * spin.p.rate, 1e-12, "Update ran on the clock");
    }

    // ---- 8. The Options shim ---------------------------------------------------------------
    {
        using ga::app::Options;
        using ga::app::ParseArgs;
        auto parse = [](std::initializer_list<const char*> args) {
            std::vector<char*> argv;
            argv.push_back(const_cast<char*>("gagame"));
            for (const char* a : args) argv.push_back(const_cast<char*>(a));
            return ParseArgs(static_cast<int>(argv.size()), argv.data());
        };
        auto find = [](const ga::app::SceneArgs& s, const char* path) -> const JsonValue* {
            for (const auto& st : s.sets) {
                if (st.path == path) return &st.value;
            }
            return nullptr;
        };
        {
            const auto s = Options::ToSets(parse({"--sea", "--boat", "rhib.novurania18"}));
            g.True(!s.ok, "--boat without --campos is refused (the plan's deliberate non-identity)");
            g.Has(s.why, "--campos", "and the refusal says why");
        }
        {
            const auto s = Options::ToSets(parse({"--sea", "--boat", "rhib.novurania18", "--campos", "120,-10",
                                                  "--boat-drive", "1,0"}));
            g.True(s.ok, "--boat with --campos maps");
            const JsonValue* at = find(s, "entities.boat.at");
            g.True(at && at->Num("x", 1) == 120.0 && at->Num("z", 1) == -10.0 && at->Num("alt", 1) == 0.0,
                   "the spawn is {x, alt: 0, z}");
            const JsonValue* ctl = find(s, "entities.boat.controller");
            g.True(ctl && ctl->str == "fixed", "--boat-drive is the fixed controller");
        }
        {
            const auto s = Options::ToSets(parse({"--sea", "--one-water", "--pixel-water", "--headless",
                                                  "--rail-flood", "out/rail", "--frames", "300",
                                                  "--storm", "3.0,10,95", "--start", "2026-08-28T14:00:00",
                                                  "--tile-budget", "3000"}));
            g.True(s.ok && s.scene == "scenes/merrimack.json", "the rail is the merrimack scene");
            const JsonValue* v = find(s, "scene.view");
            g.True(v && v->str == "orbit", "--rail-flood starts in orbit (the implication law)");
            v = find(s, "rails.active");
            g.True(v && v->str == "flood", "rails.active = flood");
            v = find(s, "capture.frames");
            g.True(v && v->number == 300.0, "an explicit --frames beats the rail's 1200");
            v = find(s, "capture.headless");
            g.True(v && v->boolean, "a rail is headless");
            v = find(s, "capture.railDir");
            g.True(v && v->str == "out/rail", "the rail's directory");
            v = find(s, "time.start");
            g.True(v && v->str == "2026-08-28T14:00:00Z", "the start as civil time");
            v = find(s, "sea.storm.dir");
            g.True(v && v->number == 95.0, "the storm's direction");
            v = find(s, "streaming.tileBudget");
            g.True(v && v->number == 3000.0, "the tile budget");
            g.True(s.tools.empty() && s.raw.empty(), "no tool, no raw instrument");
        }
        {
            const auto s = Options::ToSets(parse({"--rail-flood", "out/r"}));
            const JsonValue* v = find(s, "capture.frames");
            g.True(v && v->number == 1200.0, "the flood rail's 1200 frames");
        }
        {
            const auto s = Options::ToSets(parse({"--sea", "--campos", "120,-10", "--cam", "7,92.5,-1.5",
                                                  "--dump", "out/x.png", "--globe-cam", "42.74,-70.87,7",
                                                  "--pix", "3", "--selftest"}));
            const JsonValue* at = find(s, "views.sea.at");
            g.True(at && at->Num("x", 0) == 120.0 && at->Num("az", 0) == 92.5 && at->Num("pitch", 0) == -1.5,
                   "--cam/--campos reach the active view");
            const JsonValue* orb = find(s, "views.orbit.at");
            g.True(orb && orb->Num("alt", 0) == 7000.0 && orb->Get("lat") && orb->Get("lon"), "--globe-cam sets the orbit view");
            g.True(orb && orb->Num("lat", 0) == static_cast<double>(42.74f), "a float flag value, widened, is what the code held");
            const JsonValue* fr = find(s, "capture.frames");
            g.True(fr && fr->number == 1.0 && find(s, "capture.headless"), "--dump implies one headless frame");
            g.True(s.tools.size() == 1 && s.tools[0] == "selftest", "--selftest is the selftest tool");
            g.True(s.raw.size() == 1 && s.raw[0] == "--pix 3", "--pix stays a raw instrument");
        }
        {
            const auto s = Options::ToSets(parse({}));
            g.True(s.ok && s.scene == "scenes/chart.json", "a bare launch is the chart scene");
            const JsonValue* v = find(s, "scene.mode");
            g.True(v && v->str == "chart", "in chart mode");
        }
    }

    // ---- 9. [view] the re-levelling (M12 step 5b) -------------------------------------------
    {
        // The same six rows as src/hal/DxTest.cpp's kViewPoses (tools/stills.sh + the droste
        // pose). key7km is the sea default: --globe-cam without --globe never moved the eye.
        struct VP { const char* name; double e, alt, n; float az, pitch; };
        const VP poses[] = {
            {"helm", 120.0, 7.0, -10.0, 92.5f, -1.5f},
            {"helm_ebb", 120.0, 7.0, -10.0, 92.5f, -1.5f},
            {"bird", 380.0, 1500.0, 10.0, 272.0f, -88.0f},
            {"key7km", 522.0, 7.0, 72.0, 246.0f, -4.0f},
            {"globe", 0.0, 200000.0, 0.0, 0.0f, 0.0f},
            {"droste", 632.0, 71.0, 30.0, 55.0f, 8.0f},
        };
        constexpr double kPi = 3.14159265358979323846;
        auto axesOf = [](const Motor& m, double x[3], double y[3], double z[3]) {
            x[0] = 1; x[1] = 0; x[2] = 0; m.TransformDir(x[0], x[1], x[2]);
            y[0] = 0; y[1] = 1; y[2] = 0; m.TransformDir(y[0], y[1], y[2]);
            z[0] = 0; z[1] = 0; z[2] = 1; m.TransformDir(z[0], z[1], z[2]);
        };
        auto eyeOf = [](const Motor& m, double p[3]) {
            p[0] = p[1] = p[2] = 0.0;
            m.TransformPoint(p[0], p[1], p[2]);
        };
        auto maxAbs3 = [](const double a[3], const double b[3]) {
            double e = 0.0;
            for (int i = 0; i < 3; ++i) e = (std::max)(e, std::fabs(a[i] - b[i]));
            return e;
        };
        auto mag3 = [](const double a[3]) {
            return (std::max)(1.0, std::sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]));
        };
        // A rotor and its negative are ONE rotation, so the sign is aligned before the compare.
        auto rotorErr = [](const Motor& a, const Motor& b) {
            double ra[4], rb[4];
            a.Real(ra);
            b.Real(rb);
            double dot = 0.0;
            for (int i = 0; i < 4; ++i) dot += ra[i] * rb[i];
            const double sg = dot < 0.0 ? -1.0 : 1.0;
            double e = 0.0;
            for (int i = 0; i < 4; ++i) e = (std::max)(e, std::fabs(ra[i] - sg * rb[i]));
            return e;
        };
        auto wrapPi = [kPi](double a) {
            while (a > kPi) a -= 2.0 * kPi;
            while (a < -kPi) a += 2.0 * kPi;
            return a;
        };
        // THE FINDING THIS GATE TURNED UP, and the reference the strict half needs.
        // scene::FromCamera -- the session's poseMotor, moved verbatim in 5a -- writes
        //     const double rAxis[3] = {std::sin(c.yaw), 0.0, -std::cos(c.yaw)};
        // and c.yaw is a FLOAT, so std::sin resolves to the float overload: the roll axis is
        // float-rounded, |rAxis| is 1 +- 1.2e-7, and the motor that comes back is not a unit
        // rotor. Two consequences, both measured below and both pre-existing: the pose carries
        // ~1e-7 rad of spurious ROLL (the axis is not quite perpendicular to the aim), and
        // TransformPoint scales the eye by |r|^2, which moves the position by up to 1e-5 m at
        // the bird pose's 1500 m altitude on one poseMotor/motorPose round trip. Nothing on the
        // render path reads that motor today (the rasterizer takes yaw and pitch), so this step
        // does NOT touch it -- the camera path is moved, not rewritten. The lambda below is the
        // same construction with the axis in double: the pose the session means, and what the
        // strict half of the gate holds View::Level against.
        auto poseMotorExact = [kPi](const Camera& c) {
            (void)kPi;
            const double org[3] = {0.0, 0.0, 0.0};
            const double yAxis[3] = {0.0, 1.0, 0.0};
            const double y = static_cast<double>(c.yaw);
            const double rAxis[3] = {std::sin(y), 0.0, -std::cos(y)};
            return Motor::Translation(c.px, c.py, c.pz) *
                   Motor::Rotation(org, rAxis, -static_cast<double>(c.pitch)) *
                   Motor::Rotation(org, yAxis, -y);
        };
        double maxAout = 0.0, maxAband = 0.0, maxAeye = 0.0, maxArel = 0.0;
        double maxB = 0.0, maxBeye = 0.0, maxBrel = 0.0, maxBfloat = 0.0;
        double maxC = 0.0, maxCeye = 0.0, maxCrel = 0.0, maxD = 0.0;
        int nA = 0, nB = 0, nBand = 0;
        auto probe = [&](const Camera& cam, double rollDeg) {
            const double up[3] = {cam.upHint[0], cam.upHint[1], cam.upHint[2]};
            const Motor base = scene::FromCamera(cam);
            const double org[3] = {0.0, 0.0, 0.0}, ex[3] = {1.0, 0.0, 0.0};
            // A BODY-FRAME roll about the aim: the thing the levelling must not be able to see.
            const Motor rolled = base * Motor::Rotation(org, ex, rollDeg * kPi / 180.0);
            const Motor lv = View::Level(rolled, up);

            // (A) the frame the RASTERIZER builds today, for the same pose: the Euler round trip
            // and then Camera::ViewBasis. That reference is float throughout (Forward() and the
            // basis are XMFLOAT3), so the residue is the boundary's, not the algebra's -- and
            // INSIDE the degeneracy blend the float error is amplified by the blend's own
            // divisor (1 / 0.0145 = 69x), which is why the two bands are reported apart.
            Camera c2 = cam;
            scene::ToCamera(rolled, c2);
            DirectX::XMFLOAT3 ff, rr, uu;
            c2.ViewBasis(ff, rr, uu);
            const double rf[3] = {ff.x, ff.y, ff.z};
            const double ru[3] = {uu.x, uu.y, uu.z};
            const double rz[3] = {-rr.x, -rr.y, -rr.z};   // the motor's +Z is -right
            double x[3], y[3], z[3], p[3];
            axesOf(lv, x, y, z);
            eyeOf(lv, p);
            const double ax =
                (std::max)(maxAbs3(x, rf), (std::max)(maxAbs3(y, ru), maxAbs3(z, rz)));
            double d[3] = {1.0, 0.0, 0.0};
            rolled.TransformDir(d[0], d[1], d[2]);
            const double dn = std::sqrt(d[0] * d[0] + d[1] * d[1] + d[2] * d[2]);
            double align = 0.0;
            for (int i = 0; i < 3; ++i) align += (d[i] / dn) * up[i];
            const bool inBand = std::fabs(align) >= 0.985;
            if (inBand) maxAband = (std::max)(maxAband, ax);
            else maxAout = (std::max)(maxAout, ax);
            const double re[3] = {c2.px, c2.py, c2.pz};
            const double de = maxAbs3(p, re);
            maxAeye = (std::max)(maxAeye, de);
            maxArel = (std::max)(maxArel, de / mag3(re));
            ++nA;

            // (B) with the world's up as the field and OUTSIDE the blend, Level reproduces the
            // pose motor itself -- the strict half, against the double-axis construction; the
            // same comparison against the session's own float-axis motor is reported beside it,
            // and the gap IS the spurious roll named above.
            const Motor exact = poseMotorExact(cam);
            double de3[3] = {1.0, 0.0, 0.0};
            exact.TransformDir(de3[0], de3[1], de3[2]);
            if (std::fabs(de3[1]) < 0.985) {
                const double w[3] = {0.0, 1.0, 0.0};
                const Motor lw = View::Level(exact, w);
                maxB = (std::max)(maxB, rotorErr(lw, exact));
                maxBfloat = (std::max)(maxBfloat, rotorErr(View::Level(base, w), base));
                double pb[3], rb[3];
                eyeOf(lw, pb);
                eyeOf(exact, rb);
                const double db = maxAbs3(pb, rb);
                maxBeye = (std::max)(maxBeye, db);
                maxBrel = (std::max)(maxBrel, db / mag3(rb));
                ++nB;
            } else {
                ++nBand;
            }

            // (C) the roll is removed by construction: levelling a rolled pose and an unrolled
            // one give the same motor.
            const Motor l0 = View::Level(base, up);
            maxC = (std::max)(maxC, rotorErr(lv, l0));
            double pc[3], p0[3];
            eyeOf(lv, pc);
            eyeOf(l0, p0);
            const double dc = maxAbs3(pc, p0);
            maxCeye = (std::max)(maxCeye, dc);
            maxCrel = (std::max)(maxCrel, dc / mag3(p0));

            // (D) idempotent.
            maxD = (std::max)(maxD, rotorErr(View::Level(lv, up), lv));
        };
        auto gravityCam = [](double e, double alt, double n, float az, float pitch) {
            Camera c;
            c.SetFromCompass(e, alt, n, az, pitch);
            c.fovY = 55.0f * 3.14159265f / 180.0f;
            c.nearZ = 0.25f;
            const double R = 6371000.0, gy = c.py + R;
            const double gl = std::sqrt(c.px * c.px + gy * gy + c.pz * c.pz);
            c.upHint[0] = static_cast<float>(c.px / gl);
            c.upHint[1] = static_cast<float>(gy / gl);
            c.upHint[2] = static_cast<float>(c.pz / gl);
            return c;
        };
        for (const VP& vp : poses) {
            const Camera c = gravityCam(vp.e, vp.alt, vp.n, vp.az, vp.pitch);
            probe(c, 0.0);
            probe(c, 30.0);
            probe(c, -30.0);
        }
        // A thousand random poses, from tens of metres to planet radii, with a random up FIELD:
        // the generator is a fixed LCG, so the sweep is the same sweep on every machine.
        uint64_t rng = 0x9e3779b97f4a7c15ull;
        auto next01 = [&rng] {
            rng = rng * 6364136223846793005ull + 1442695040888963407ull;
            return static_cast<double>((rng >> 11) & ((1ull << 53) - 1)) / 9007199254740992.0;
        };
        for (int i = 0; i < 1000; ++i) {
            Camera c;
            const double scale = std::pow(10.0, 1.0 + 6.1 * next01());   // 10 m .. 1.3e7 m
            c.px = (next01() * 2.0 - 1.0) * scale;
            c.py = (next01() * 2.0 - 1.0) * scale;
            c.pz = (next01() * 2.0 - 1.0) * scale;
            c.yaw = static_cast<float>((next01() * 2.0 - 1.0) * kPi);
            c.pitch = static_cast<float>((next01() * 2.0 - 1.0) * 89.0 * kPi / 180.0);
            c.fovY = 55.0f * 3.14159265f / 180.0f;
            double u[3] = {next01() * 2.0 - 1.0, next01() * 2.0 - 1.0, next01() * 2.0 - 1.0};
            const double un = std::sqrt(u[0] * u[0] + u[1] * u[1] + u[2] * u[2]);
            if (un < 1e-6) continue;
            for (int k = 0; k < 3; ++k) c.upHint[k] = static_cast<float>(u[k] / un);
            probe(c, (next01() * 2.0 - 1.0) * 30.0);
        }
        // (E) MotorOf inverts Placement::Rigid -- the sugar's road from a file to a pose. The
        // identity holds for a UNIT rotor, which is what Placement declares (Space.h), so the
        // motor is re-unitized first: with the float-axis rotor it is off by |r|^2, which is
        // the same 1e-5 m at the bird pose that the round trip below reports.
        double maxE = 0.0;
        for (const VP& vp : poses) {
            const Camera c = gravityCam(vp.e, vp.alt, vp.n, vp.az, vp.pitch);
            Motor m = scene::FromCamera(c);
            m.Normalize();
            const Placement p = Placement::Rigid(m);
            const Placement q = Placement::Rigid(MotorOf(p));
            for (int k = 0; k < 4; ++k) maxE = (std::max)(maxE, std::fabs(p.r[k] - q.r[k]));
            for (int k = 0; k < 3; ++k) maxE = (std::max)(maxE, std::fabs(p.t[k] - q.t[k]));
        }
        // (F) the rasterizer boundary round-trips a Camera. The OPTICS cross it verbatim and are
        // compared bitwise; the eye and the aim cross it through the motor, so they carry
        // poseMotor's own float-axis residue and are measured.
        bool opticsExact = true;
        double maxFang = 0.0, maxFeye = 0.0, maxFrel = 0.0;
        for (const VP& vp : poses) {
            const Camera c = gravityCam(vp.e, vp.alt, vp.n, vp.az, vp.pitch);
            View v("probe");
            v.FromCamera(c);
            Camera back = c;
            v.ToCamera(back);
            opticsExact = opticsExact && back.fovY == c.fovY && back.nearZ == c.nearZ;
            maxFang = (std::max)(maxFang,
                                 std::fabs(wrapPi(static_cast<double>(back.yaw) - c.yaw)));
            maxFang = (std::max)(maxFang,
                                 std::fabs(wrapPi(static_cast<double>(back.pitch) - c.pitch)));
            const double a[3] = {back.px, back.py, back.pz};
            const double b[3] = {c.px, c.py, c.pz};
            const double df = maxAbs3(a, b);
            maxFeye = (std::max)(maxFeye, df);
            maxFrel = (std::max)(maxFrel, df / mag3(b));
        }
        const View probeView("probe");
        Log("[view] Level over %d poses (%d random, %d strict, %d inside ViewBasis's blend):",
            nA, nA - 18, nB, nBand);
        Log("[view]   vs the rasterizer's own frame (ToCamera + Camera::ViewBasis, float): max "
            "axis %.3g outside the blend, %.3g inside it (the blend divides by 0.0145, so it "
            "multiplies the float basis' error by 69); max eye %.3g m (%.3g relative)",
            maxAout, maxAband, maxAeye, maxArel);
        Log("[view]   vs poseMotor at the world-up field: max rotor %.3g, max eye %.3g m (%.3g "
            "relative) -- against the SESSION's float-axis motor it is %.3g, which is the "
            "~1e-7 rad of spurious roll std::sin(float) puts in scene::FromCamera's axis",
            maxB, maxBeye, maxBrel, maxBfloat);
        Log("[view]   roll +-30 deg invariance: max rotor %.3g, max eye %.3g m (%.3g relative); "
            "idempotent max rotor %.3g; Rigid(MotorOf(p)) max %.3g; Camera round trip max angle "
            "%.3g rad, max eye %.3g m (%.3g relative, poseMotor's |r|^2)",
            maxC, maxCeye, maxCrel, maxD, maxE, maxFang, maxFeye, maxFrel);
        g.True(maxAout <= 1e-6, "[view] Level reproduces Camera::ViewBasis to the float basis");
        g.True(maxAband <= 4e-5, "[view] Level reproduces Camera::ViewBasis inside the blend");
        g.True(maxArel <= 1e-14, "[view] Level's eye IS the pose's eye");
        g.True(maxB <= 1e-12, "[view] Level at the world-up field == poseMotor (rotor)");
        g.True(maxBrel <= 1e-14, "[view] Level at the world-up field == poseMotor (eye)");
        g.True(maxC <= 1e-12, "[view] Level is invariant to roll (rotor)");
        g.True(maxCrel <= 1e-14, "[view] Level is invariant to roll (eye)");
        g.True(maxD <= 1e-12, "[view] Level is idempotent");
        g.True(maxE <= 1e-9, "[view] Placement::Rigid(MotorOf(p)) == p for a unit rotor");
        g.True(opticsExact, "[view] FromCamera then ToCamera keeps the optics bit for bit");
        g.True(maxFang <= 1e-6, "[view] FromCamera then ToCamera keeps the aim to a float ulp");
        g.True(maxFrel <= 1e-6, "[view] FromCamera then ToCamera keeps the eye to poseMotor's own residue");
        g.True(&probeView.Props() == &ViewSchema(),
               "[view] a View's Schema IS the views section's table");
    }

    if (g.ok) {
        Log("[scene] ---- PASS (%d checks): Registry<T> == VesselRegistry / LoaderRegistry on the "
            "built-in kinds, the property table (defaults, units through GaUnits, the Velocity-"
            "into-Length and datum refusals with AcceptsFrom's why, unknown keys, ApplyTo, Diff, "
            "the round trip), the fold with override (defaults < base < include < --set; atomic "
            "placements; named arrays in place / remove / append; completion), the placement "
            "sugar's four spellings == the session's pose maps bit for bit, WriteJson round-trips, "
            "refusals carry the node path, Node (the fold, the walk, a component), the Options "
            "shim (the --boat refusal, the rail's implication laws), and [view] View::Level "
            "against the rasterizer's own frame, against poseMotor, under a roll and under "
            "itself ----",
            g.checks);
    } else {
        Log("[scene] ---- FAIL (%d checks) ----", g.checks);
    }
    return g.ok;
}

}  // namespace ga::scene
