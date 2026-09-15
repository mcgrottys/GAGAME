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
//  10. [water] (M12 step 5c) THE ONE APPLY and the reload law, on an in-memory document: the
//      component's table agrees with the `water` section's own key for key, type for type, unit
//      for unit and Hot/Restart for Hot/Restart (two tables over two structs, one vocabulary);
//      a key the document no longer carries reverts to its DECLARED DEFAULT and not to the value
//      the run was carrying (the step's second deliberate change); an unknown key refuses with
//      its path and the previous set stands; a Restart-flagged key that changed is reported and
//      NOT applied, and does not revert either -- the run keeps what it was BUILT with; a unit
//      refusal carries GaUnits' own why; and Apply is IDEMPOTENT, the same set twice leaving
//      the same fields and the same fingerprint.
//  11. (M12 step 5d) THE FOLD'S FIXED POINT, on real files, because both laws live in
//      SceneBuilder::Load: an UNNAMED list (include, water.fleet.boats, rails.keys) is a VALUE
//      and an overlay REPLACES it whole; `"base": ""` is NO base (a complete document always
//      carries the key); and therefore resolution is IDEMPOTENT -- the text --print-scene
//      writes IS a scene file, and loading it back gives the same document, which is what the
//      scene spelling of every recipe stands on.
//  12. (M12 step 5e) THE NODES. [rail] the five hand tables and the six rail lambdas of the
//      session (c2813b8) are transcribed verbatim as the reference: printed through the rail
//      file's writer they equal scenes/rails/<name>.json byte for byte (the printed text is
//      written to out/rails_printed/, which is how the files were made), and Rail::At over the
//      file equals the lambdas' camera, level and up BITWISE at a thousand instants plus every
//      recorded frame of a 100 s flight, under both twists; [portal] the node's cycle is
//      Space::Cycle over the tangent space with Similar(p, s, axis, twist) of the link
//      BuildPortal resolved from the declaration; [effect] the slice plane's edge is registered
//      and the AST validates with it; [entity] the freshness fields default to today's
//      behaviour -- no cadence, no mirror read, the age reported as never.
// A defect planted in MergeInto (the atomic law dropped, so an overlay's {lat, lon, alt}
// merged into {x, alt, z, az, pitch} by key) was seen to fail block 3 before this gate was
// trusted (priors 22). The three checks of block 11 were seen to fail with BOTH step 5d fixes
// reverted (the unnamed list appended, `"base": ""` refused) before they were trusted.
#include "app/Options.h"
#include "scene/Gateway.h"
#include "sim/RigidBody.h"
#include "core/Common.h"
#include "core/CurrentFieldLoader.h"
#include "core/Droste.h"
#include "core/GaAst.h"
#include "core/FieldLoader.h"
#include "core/GeoGridLoader.h"
#include "core/Registry.h"
#include "render/Camera.h"
#include "scene/Component.h"
#include "scene/Entity.h"
#include "scene/GlobeLayer.h"
#include "scene/Node.h"
#include "scene/Portal.h"
#include "scene/Rail.h"
#include "scene/effects/SlicePlane.h"
#include "scene/Pose.h"
#include "scene/Props.h"
#include "scene/SceneBuilder.h"
#include "scene/SceneSchema.h"
#include "scene/View.h"
#include "scene/WaterComponent.h"
#include "scene/SceneReload.h"
#include "sim/BathyModel.h"
#include "sim/GlobeModel.h"
#include "sim/VesselSpec.h"
#include "sim/WaterSurfaceTree.h"
#include "sim/WeatherManager.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <filesystem>
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

    // ---- 10. [water] the one Apply and the reload law (M12 step 5c) --------------------------
    {
        // (A) TWO TABLES, ONE VOCABULARY. The component binds the live WaterSceneConfig; the
        // scene file's `water` section binds its own structs. Different layouts, so they cannot
        // be one Schema -- and they must not drift, because a file written against one is read
        // by the other. Key, type, field width, quantity, canonical unit, datum and the
        // Hot/Restart flag are held equal; the doc strings are not (the section's are the
        // one-liners --print-scene prints, the component's the declaration's own comments).
        const PropDecl* waterDecl = SceneFileSchema().Find("water");
        g.True(waterDecl && waterDecl->sub, "[water] the scene file has a water section");
        int compared = 0;
        if (waterDecl && waterDecl->sub) {
            for (const char* section : {"wavefield", "closures", "fleet"}) {
                const PropDecl* a = WaterSchema().Find(section);
                const PropDecl* b = waterDecl->sub->Find(section);
                g.True(a && a->sub && b && b->sub, "[water] both tables carry the section");
                if (!a || !a->sub || !b || !b->sub) continue;
                g.True(a->sub->Decls().size() == b->sub->Decls().size(),
                       "[water] the section's key count agrees");
                for (const PropDecl& da : a->sub->Decls()) {
                    const PropDecl* db = b->sub->Find(da.key);
                    if (!db) {
                        g.True(false, "[water] the section's key is in both tables");
                        continue;
                    }
                    ++compared;
                    g.True(da.type == db->type && da.field == db->field,
                           "[water] the key's type and field width agree");
                    g.True(da.quantity == db->quantity, "[water] the key's quantity agrees");
                    g.Same(da.unit.toCanonical, db->unit.toCanonical, "[water] the key's unit agrees");
                    g.Same(da.unit.datumShiftM, db->unit.datumShiftM, "[water] the key's datum agrees");
                    g.True(da.reload == db->reload, "[water] the key's Hot/Restart flag agrees");
                }
            }
        }

        // The component wired to a live config and NOTHING else: every fan-out subject is
        // nullable and every absence is reported, so the whole law runs without a device.
        WaterSceneConfig live;
        WaterComponent water;
        WaterComponent::Observers o;
        o.config = &live;
        const std::vector<std::string> missing = water.Configure(o);
        g.True(missing.size() == 8, "[water] Configure reports every observer it was not given");
        g.True(std::find(missing.begin(), missing.end(), std::string("config")) == missing.end(),
               "[water] the one observer it WAS given is not reported missing");
        auto setText = [](const WaterSceneConfig& c) {
            return WaterComponent::SetText(PropSet::Defaults(WaterSchema(), &c));
        };

        // A run that is NOT at the declared defaults: two closures retuned and a bank built at
        // a different ring density (the Restart key).
        live.foamOpacity = 0.90f;
        live.windSeaFill = 0.25f;
        live.bankTexelM = 2.4f;
        water.Apply(water.BootSet());
        g.Same(live.foamOpacity, 0.90f, "[water] the boot set is the live config's own values");
        g.Same(live.windSeaFill, 0.25f, "[water] the boot set is the live config's own values");
        g.Same(live.bankTexelM, 2.4f, "[water] the boot set applies its Restart key (it is boot)");
        const uint64_t bootHash = water.AppliedHash();

        // (B) THE RELOAD LAW. A document naming ONE closure is a COMPLETE candidate: that key
        // takes the document's value and every other HOT key reverts to its DECLARED DEFAULT --
        // 0.72 for foamOpacity, 1.0 for windSeaFill, NOT the 0.90 / 0.25 this run carried. That
        // is the step's second deliberate change; the old reader kept the run's value.
        PropSet cand(WaterSchema());
        WaterComponent::Fleet fleet;
        std::string why;
        const JsonValue one = ParseOrDie(g, "{\"closures\": {\"churnGain\": 0.5}}");
        g.True(water.ReadJson(one, cand, fleet, &why), "[water] a one-key document resolves");
        water.StageFleet(fleet);
        water.Apply(cand);
        g.Same(live.churnGain, 0.5f, "[water] the document's key is applied");
        g.Same(live.foamOpacity, 0.72f, "[water] a key the document drops reverts to its DEFAULT");
        g.Same(live.windSeaFill, 1.0f, "[water] a key the document drops reverts to its DEFAULT");
        g.Same(live.wfExag, WaterSceneConfig{}.wfExag,
               "[water] a whole dropped section reverts to its defaults");
        g.Same(live.bankTexelM, 2.4f,
               "[water] a Restart key does NOT revert: the run keeps what it was built with");

        // (C) IDEMPOTENCE: the same set again leaves the same fields and the same fingerprint.
        const uint64_t h1 = water.AppliedHash();
        const std::string state1 = setText(live);
        water.Apply(cand);
        g.True(water.AppliedHash() == h1, "[water] Apply is idempotent (the set's fingerprint)");
        g.True(setText(live) == state1, "[water] Apply is idempotent (every declared field)");

        // (D) AN UNKNOWN KEY REFUSES WITH ITS PATH, and the previous set stands.
        PropSet bad(WaterSchema());
        why.clear();
        const JsonValue unknown = ParseOrDie(g, "{\"closures\": {\"foamOpacty\": 0.1}}");
        g.True(!water.ReadJson(unknown, bad, fleet, &why), "[water] an unknown key refuses");
        g.Has(why, "water.closures.foamOpacty", "[water] the refusal names the key's path");
        g.True(setText(live) == state1, "[water] a refused candidate changes nothing");

        // A unit refusal is GaUnits' own, through the same table: a velocity into a length.
        why.clear();
        const JsonValue wrongUnit = ParseOrDie(g, "{\"wavefield\": {\"cellM\": \"2 kn\"}}");
        g.True(!water.ReadJson(wrongUnit, bad, fleet, &why), "[water] a wrong unit refuses");
        g.Has(why, "wavefield.cellM", "[water] the unit refusal names the key's path");

        // (E) THE RESTART LAW. bankTexelM is read where the bank's rings are BUILT, so a changed
        // value is reported and NOT applied; the Hot keys beside it are.
        PropSet restart(WaterSchema());
        const JsonValue texel = ParseOrDie(
            g, "{\"wavefield\": {\"bankTexelM\": 4.8}, \"closures\": {\"churnGain\": 0.25}}");
        g.True(water.ReadJson(texel, restart, fleet, &why), "[water] the Restart document resolves");
        water.StageFleet(fleet);
        water.Apply(restart);
        g.Same(live.bankTexelM, 2.4f, "[water] a changed Restart key is NOT applied");
        g.Same(live.churnGain, 0.25f, "[water] the Hot keys beside it ARE applied");

        // And the whole law at its limit: the EMPTY document is the declared defaults, every
        // Hot key, with the Restart key the run was built with left standing.
        PropSet empty(WaterSchema());
        g.True(water.ReadJson(ParseOrDie(g, "{}"), empty, fleet, &why),
               "[water] the empty document resolves");
        water.StageFleet(fleet);
        water.Apply(empty);
        WaterSceneConfig declared;
        declared.bankTexelM = 2.4f;   // the Restart key this run keeps
        g.True(setText(live) == setText(declared),
               "[water] the empty document IS the declared defaults, field for field");
        g.True(live.fleetCount == WaterSceneConfig{}.fleetCount &&
                   live.fleet[2].offsetS == WaterSceneConfig{}.fleet[2].offsetS,
               "[water] the boats -- a LIST, beside the set -- follow the same law");
        Log("[water] two tables held equal on %d keys; the reload law, the Restart refusal, the "
            "unknown-key and unit refusals and idempotence on in-memory documents; boot set "
            "FNV-1a %016llx",
            compared, static_cast<unsigned long long>(bootHash));
    }

    // ---- 11. THE FOLD'S FIXED POINT (M12 step 5d) -----------------------------------------
    // The two laws that make resolution idempotent, on real files -- both live in Load(), so an
    // in-memory Overlay cannot see them. The include names TWO boats and the file that includes
    // it names THREE: appending gives five (what it did), replacing gives the overlay's two.
    {
        const char* kInc = "gagame_scenetest_include.json";
        const char* kDoc = "gagame_scenetest_scene.json";
        const char* kFp = "gagame_scenetest_printed.json";
        auto writeFile = [](const char* path, const std::string& text) {
            FILE* f = fopen(path, "wb");
            if (!f) return false;
            fwrite(text.data(), 1, text.size(), f);
            fclose(f);
            return true;
        };
        const bool wrote =
            writeFile(kInc, "{\"fleet\": {\"enabled\": true, \"boats\": ["
                            "{\"speed\": 3, \"halfLen\": 6}, {\"speed\": 4}]}}") &&
            writeFile(kDoc, std::string("{\"base\": \"\", \"scene\": {\"name\": \"fixedpoint\"},"
                                        " \"include\": [{\"file\": \"") + kInc +
                                "\", \"at\": \"water\"}], \"water\": {\"fleet\": {\"boats\": "
                                "[{\"speed\": 9}, {\"speed\": 9}, {\"speed\": 9}]}}}");
        g.True(wrote, "[fold] the temporary scene files are written");
        SceneBuilder fa;
        std::string fwhy;
        g.True(fa.Load(kDoc, &fwhy),
               (std::string("[fold] `\"base\": \"\"` loads as NO base: ") + fwhy).c_str());
        g.True(fa.Resolve(&fwhy), (std::string("[fold] it resolves: ") + fwhy).c_str());
        const JsonValue* fw = fa.Resolved().Get("water");
        const JsonValue* ffl = fw ? fw->Get("fleet") : nullptr;
        const JsonValue* fbs = ffl ? ffl->Get("boats") : nullptr;
        g.True(fbs && fbs->arr.size() == 2,
               "[fold] an overlay's UNNAMED list REPLACES the base's whole (water.fleet.boats: "
               "the include's two over the file's three is two, not five)");
        g.True(fbs && fbs->arr.size() == 2 && fbs->arr[0].Num("speed", 0.0) == 3.0,
               "[fold] ...and what stands is the OVERLAY's list, in the overlay's order");
        // THE FIXED POINT: the text --print-scene writes, loaded back, is the same document.
        const std::string printed = SceneBuilder::WriteJson(fa.Resolved());
        g.True(writeFile(kFp, printed), "[fold] the printed document is written back as a file");
        SceneBuilder fb;
        g.True(fb.Load(kFp, &fwhy),
               (std::string("[fold] the printed document loads: ") + fwhy).c_str());
        g.True(fb.Resolve(&fwhy),
               (std::string("[fold] the printed document resolves: ") + fwhy).c_str());
        g.True(SceneBuilder::WriteJson(fb.Resolved()) == printed,
               "[fold] THE FIXED POINT: Resolved -> WriteJson -> Load -> Resolve is the identity "
               "(the scene spelling of every recipe stands on it)");
        remove(kInc);
        remove(kDoc);
        remove(kFp);
    }

    // ---- 12. [rail] [portal] [effect] [entity] -- the nodes of step 5e ---------------------
    // (A) THE RAILS. The hand tables and the six lambdas that flew them (FrameLoop::Session at
    // c2813b8: railKeys, railPose, diveFrom/diveAt, legU, diveU, keyedPose, gravityUp,
    // drosteRailPose) are transcribed VERBATIM below -- literals and float arithmetic included
    // -- as the reference, and held two ways against the data form: the tables PRINTED through
    // the rail file's own writer equal scenes/rails/<name>.json byte for byte (the printed
    // text is also written to out/rails_printed/, which is how the files were made), and
    // Rail::At (the file read back and resolved in the same frame) equals the lambdas' camera,
    // level and up BITWISE at a thousand instants plus every recorded frame of a 100 s flight,
    // under both twists (the quarter twist the recipes fly and the untwisted tower, which
    // dives from above). The two Droste rails ride the FLOOD keys to the helm, as the flag
    // implied the flood table before 5d compared the enum (the 5d parent flew the classic keys
    // there); the reference says what the design and the pre-5d binary said.
    {
        constexpr double kPiL = 3.14159265358979;
        struct Legacy {
            double planetR = 0.0;
            double east0[3], oDir[3], north0[3];
            float fovY = 0.9f;
            const droste::Portal* portal = nullptr;
            const Space* drosteLeaf = nullptr;
            double twistDeg = 90.0;
            double levelSec = 16.0;
            int levels = 3;
            bool railDroste = false, railDrosteOut = false;
            Camera camGlobe;
            std::vector<std::pair<double, Motor>> railKeys, climbKeys;
            Camera drosteHelm, drosteHelmBack;
            double drosteHelmUp[3] = {0.0, 1.0, 0.0};
            bool diveFromAbove = false;
            const double kDiveT0 = 40.0 + 2.0;

            Motor poseMotor(const Camera& c) const { return scene::FromCamera(c); }
            void motorPose(const Motor& m, Camera& c) const { scene::ToCamera(m, c); }
            Camera planetToFlatPose(const Camera& g) const {
                return scene::PlanetToFlatPose(g, east0, oDir, north0, planetR);
            }
            Camera orbPose(double lat, double lon, double altM, double tLat, double tLon) const {
                return scene::OrbitPose(lat, lon, altM, tLat, tLon, planetR);
            }
            Motor orbKey(double lat, double lon, double altM, double tLat, double tLon) const {
                return poseMotor(planetToFlatPose(orbPose(lat, lon, altM, tLat, tLon)));
            }
            void railPose(double t, Camera& out) const {
                size_t i = 0;
                while (i + 1 < railKeys.size() && railKeys[i + 1].first <= t) ++i;
                if (i + 1 >= railKeys.size()) {
                    motorPose(railKeys.back().second, out);
                    return;
                }
                const double t0 = railKeys[i].first, t1 = railKeys[i + 1].first;
                double u = (t - t0) / std::max(t1 - t0, 1e-6);
                u = u * u * (3.0 - 2.0 * u);   // ease both ends of every leg
                motorPose(Motor::Slerp(railKeys[i].second, railKeys[i + 1].second, u), out);
            }
            void diveFrom(const Camera& base, double u, Camera& out, int& level, double up[3]) const {
                const double n = std::floor(u);
                const double f = u - n;
                const double c0[3] = {base.px, base.py, base.pz};
                const DirectX::XMFLOAT3 hf = base.Forward();
                const double f0[3] = {hf.x, hf.y, hf.z};
                double c[3], fw[3];
                drosteLeaf->LevelApply(f, c0, c);
                drosteLeaf->LevelApplyDir(f, f0, fw);
                drosteLeaf->LevelApplyDir(f, drosteHelmUp, up);
                out = base;
                out.px = c[0];
                out.py = c[1];
                out.pz = c[2];
                out.yaw = static_cast<float>(std::atan2(fw[2], fw[0]));
                const float lim = 3.14159265f / 2.0f - 0.0017f;
                out.pitch = std::clamp(
                    static_cast<float>(std::atan2(fw[1], std::sqrt(fw[0] * fw[0] + fw[2] * fw[2]))),
                    -lim, lim);
                level = static_cast<int>(n);
            }
            void diveAt(double u, Camera& out, int& level, double up[3]) const {
                diveFrom(drosteHelm, u, out, level, up);
            }
            static double legU(double tau, double D, double dU) {
                const double r = (std::min)(3.0, 0.5 * D);
                const double v = dU / (D - r);
                if (tau <= 0.0) return 0.0;
                if (tau >= D) return dU;
                if (tau < r) return v * tau * tau / (2.0 * r);
                if (tau <= D - r) return v * (tau - 0.5 * r);
                const double e = D - tau;
                return dU - v * e * e / (2.0 * r);
            }
            double diveU(double tau) const {
                const double T = levelSec, ramp = 3.0;
                if (tau <= 0.0) return 0.0;
                const double u = (tau < ramp) ? tau * tau / (2.0 * ramp * T) : (tau - 0.5 * ramp) / T;
                return railDroste ? (std::min)(u, double(levels)) : u;
            }
            void keyedPose(const std::vector<std::pair<double, Motor>>& keys, double t,
                           Camera& out) const {
                size_t i = 0;
                while (i + 1 < keys.size() && keys[i + 1].first <= t) ++i;
                if (i + 1 >= keys.size()) {
                    motorPose(keys.back().second, out);
                    return;
                }
                double u = (t - keys[i].first) / std::max(keys[i + 1].first - keys[i].first, 1e-6);
                u = u * u * (3.0 - 2.0 * u);
                motorPose(Motor::Slerp(keys[i].second, keys[i + 1].second, u), out);
            }
            void gravityUp(const Camera& c, double up[3]) const {
                const double gy = c.py + planetR;
                const double gl = std::sqrt(c.px * c.px + gy * gy + c.pz * c.pz);
                up[0] = c.px / gl;
                up[1] = gy / gl;
                up[2] = c.pz / gl;
            }
            void drosteRailPose(double t, Camera& out, int& level, double up[3]) const {
                level = 0;
                if (railDroste) {
                    if (t < 40.0) {
                        railPose(t, out);   // the storm rail (its last key aimed at the fixed point)
                        gravityUp(out, up);
                        return;
                    }
                    diveAt(diveU(t - kDiveT0), out, level, up);
                    return;
                }
                // --rail-droste-out
                const double T = levelSec, Din = 2.0 * T, Dout = 2.0 * T, Tturn = 3.0;
                double tau = t;
                if (tau < 14.0) {   // the storm rail's last leg: 1.5 km over the harbor -> the helm
                    railPose(26.0 + tau, out);
                    gravityUp(out, up);
                    return;
                }
                tau -= 14.0;
                if (tau < 2.0) {    // a breath at the helm, the tower dead ahead
                    diveAt(0.0, out, level, up);
                    return;
                }
                tau -= 2.0;
                if (tau < Din) {    // IN: u 0 -> 2
                    diveAt(legU(tau, Din, 2.0), out, level, up);
                    return;
                }
                tau -= Din;
                if (tau < Tturn) {  // THE TURN, at the second level's helm
                    double w = tau / Tturn;
                    w = w * w * (3.0 - 2.0 * w);
                    motorPose(Motor::Slerp(poseMotor(drosteHelm), poseMotor(drosteHelmBack), w), out);
                    level = 2;
                    for (int i = 0; i < 3; ++i) up[i] = drosteHelmUp[i];
                    return;
                }
                tau -= Tturn;
                if (tau < Dout) {   // OUT: u 2 -> 0, facing outward
                    diveFrom(drosteHelmBack, 2.0 - legU(tau, Dout, 2.0), out, level, up);
                    return;
                }
                tau -= Dout;
                keyedPose(climbKeys, tau, out);   // the climb to orbit, looking back at the tower
                gravityUp(out, up);
            }
            // THE TABLES, verbatim, selected by the rail's name (the flag's own selection law).
            void Build(const char* name, bool marsMode, bool bathy, bool sea, bool globe) {
                railKeys.clear();
                climbKeys.clear();
                Camera cam;   // the session camera the helm copies its optics from
                cam.fovY = fovY;
                camGlobe = scene::GlobeCamera(34.0, -52.0, planetR * 2.1, planetR);
                camGlobe.fovY = cam.fovY;
                camGlobe.speed = 800000.0f;
                camGlobe = planetToFlatPose(camGlobe);
                const bool flood = strcmp(name, "flood") == 0 || strcmp(name, "droste") == 0 ||
                                   strcmp(name, "droste-out") == 0;
                railDroste = strcmp(name, "droste") == 0;
                railDrosteOut = strcmp(name, "droste-out") == 0;
                if (globe && marsMode) {
                    railKeys.push_back({0.0, orbKey(10, -25, planetR * 1.1, -12, -58)});
                    railKeys.push_back({7.0, orbKey(-7, -40, 800e3, -13, -62)});
                    railKeys.push_back({13.0, orbKey(-11, -52, 220e3, -13, -72)});
                    railKeys.push_back({19.0, orbKey(-13, -68, 150e3, -11, -90)});
                    railKeys.push_back({26.0, orbKey(-6, -98, 600e3, 18.6, -133.8)});
                    railKeys.push_back({30.0, orbKey(-4, -104, 900e3, 18.6, -133.8)});
                } else if (globe && flood && bathy) {
                    Camera cOver;    // 1.5 km over the harbor, aimed down-channel at the gap
                    cOver.SetFromCompass(-1400.0, 1500.0, 0.0, 93.0f, -40.0f);
                    Camera cHelmIn;  // helm height in the channel, the entrance dead ahead
                    cHelmIn.SetFromCompass(-250.0, 9.0, -15.0, 92.0f, -2.0f);
                    Camera cHelmGap; // ...then a ~3 kn push to between the jetty roots, gap 500 m out
                    cHelmGap.SetFromCompass(120.0, 7.0, -10.0, 92.5f, -1.5f);
                    railKeys.push_back({0.0, poseMotor(camGlobe)});
                    railKeys.push_back({8.0, orbKey(41.9, -71.6, 800e3, 42.8183, -70.81)});
                    railKeys.push_back({15.0, orbKey(42.55, -70.98, 80e3, 42.8183, -70.81)});
                    railKeys.push_back({21.0, orbKey(42.74, -70.87, 7e3, 42.8183, -70.81)});
                    railKeys.push_back({26.0, poseMotor(cOver)});
                    railKeys.push_back({32.0, poseMotor(cHelmIn)});
                    railKeys.push_back({40.0, poseMotor(cHelmGap)});
                } else if (globe && strcmp(name, "jetty") == 0 && bathy) {
                    Camera cHelm;    // on the water mid-channel, the entrance dead ahead
                    cHelm.SetFromCompass(250.0, 5.0, 40.0, 94.0f, -1.0f);
                    Camera cNTip;    // over the north tip, the gap ahead
                    cNTip.SetFromCompass(680.0, 14.0, 200.0, 192.0f, -10.0f);
                    Camera cMid;     // mid-gap, swung to look west up the channel
                    cMid.SetFromCompass(650.0, 12.0, -30.0, 262.0f, -8.0f);
                    Camera cSTip;    // over the south tip, looking back northwest across the gap
                    cSTip.SetFromCompass(680.0, 16.0, -290.0, 300.0f, -13.0f);
                    Camera cRise;    // climbing, the whole entrance opening below
                    cRise.SetFromCompass(520.0, 420.0, -140.0, 284.0f, -56.0f);
                    Camera cBird;    // bird's eye over the entrance: depth as color
                    cBird.SetFromCompass(380.0, 1500.0, 10.0, 272.0f, -88.0f);
                    railKeys.push_back({0.0, poseMotor(cHelm)});
                    railKeys.push_back({4.0, poseMotor(cHelm)});
                    railKeys.push_back({9.0, poseMotor(cNTip)});
                    railKeys.push_back({14.0, poseMotor(cMid)});
                    railKeys.push_back({18.0, poseMotor(cSTip)});
                    railKeys.push_back({21.5, poseMotor(cRise)});
                    railKeys.push_back({25.0, poseMotor(cBird)});
                    railKeys.push_back({28.0, poseMotor(cBird)});
                    railKeys.push_back({31.0, orbKey(42.79, -70.84, 7e3, 42.8183, -70.81)});
                    railKeys.push_back({34.5, orbKey(42.62, -70.95, 80e3, 42.8183, -70.81)});
                    railKeys.push_back({38.0, orbKey(41.9, -71.6, 800e3, 42.8183, -70.81)});
                    railKeys.push_back({40.0, orbKey(41.9, -71.6, 800e3, 42.8183, -70.81)});
                } else if (globe && strcmp(name, "zoom") == 0 && bathy) {
                    railKeys.push_back({0.0, poseMotor(camGlobe)});
                    railKeys.push_back({8.0, orbKey(41.9, -71.6, 800e3, 42.8183, -70.81)});
                    railKeys.push_back({16.0, orbKey(42.55, -70.98, 80e3, 42.8183, -70.81)});
                    railKeys.push_back({23.0, orbKey(42.74, -70.87, 7e3, 42.8183, -70.81)});
                    railKeys.push_back({27.0, orbKey(42.79, -70.84, 2400.0, 42.8183, -70.81)});
                    railKeys.push_back({30.0, orbKey(42.80, -70.835, 2200.0, 42.8183, -70.81)});
                } else if (globe && sea && bathy && !marsMode) {
                    Camera cHover;
                    cHover.SetFromCompass(-200.0, 1800.0, -2500.0, 22.0f, -46.0f);
                    Camera cHelm;
                    cHelm.SetFromCompass(522.0, 7.0, 72.0, 246.0f, -4.0f);
                    railKeys.push_back({0.0, poseMotor(camGlobe)});
                    railKeys.push_back({3.0, orbKey(41.2, -66.5, 500000.0,
                                                    BathyModel::kOrgLat, BathyModel::kOrgLon)});
                    railKeys.push_back({5.0, poseMotor(cHover)});
                    railKeys.push_back({10.0, poseMotor(cHover)});
                    railKeys.push_back({15.0, poseMotor(cHelm)});
                    railKeys.push_back({25.0, poseMotor(cHelm)});
                }
                drosteHelm = Camera();
                drosteHelm.SetFromCompass(120.0, 7.0, -10.0, 92.5f, -1.5f);   // the storm rail's helm
                drosteHelm.fovY = cam.fovY;
                diveFromAbove = portal->Valid() && std::abs(twistDeg) < 75.0;
                if (diveFromAbove) {
                    drosteHelm.px = portal->p[0] - 300.0;
                    drosteHelm.py = portal->p[1] + 300.0;
                    drosteHelm.pz = portal->p[2];
                }
                drosteHelmUp[0] = 0.0;
                drosteHelmUp[1] = 1.0;
                drosteHelmUp[2] = 0.0;
                if (portal->Valid()) {
                    drosteHelm.LookAt(portal->p[0], portal->p[1], portal->p[2]);
                    const double gy = drosteHelm.py + planetR;
                    const double gl = std::sqrt(drosteHelm.px * drosteHelm.px + gy * gy +
                                                drosteHelm.pz * drosteHelm.pz);
                    drosteHelm.upHint[0] = static_cast<float>(drosteHelm.px / gl);
                    drosteHelm.upHint[1] = static_cast<float>(gy / gl);
                    drosteHelm.upHint[2] = static_cast<float>(drosteHelm.pz / gl);
                    DirectX::XMFLOAT3 hf, hr, hu;
                    drosteHelm.ViewBasis(hf, hr, hu);   // the helm's TRUE up: the roll the spiral carries
                    drosteHelmUp[0] = hu.x;
                    drosteHelmUp[1] = hu.y;
                    drosteHelmUp[2] = hu.z;
                    if ((railDroste || railDrosteOut) && !railKeys.empty()) {
                        railKeys.back().second = poseMotor(drosteHelm);
                    }
                }
                drosteHelmBack = drosteHelm;
                drosteHelmBack.yaw = drosteHelm.yaw + 3.14159265f;
                drosteHelmBack.pitch = -0.07f;
                if (portal->Valid() && railDrosteOut) {
                    Camera cRise;   // rising over the harbor, looking back east at the entrance
                    cRise.SetFromCompass(-900.0, 450.0, -60.0, 84.0f, -17.0f);
                    climbKeys.push_back({0.0, poseMotor(drosteHelmBack)});
                    climbKeys.push_back({7.0, poseMotor(cRise)});
                    climbKeys.push_back({13.0, orbKey(42.74, -70.87, 7e3, 42.8183, -70.81)});
                    climbKeys.push_back({18.5, orbKey(42.55, -70.98, 80e3, 42.8183, -70.81)});
                    climbKeys.push_back({23.5, orbKey(41.9, -71.6, 800e3, 42.8183, -70.81)});
                    climbKeys.push_back({28.0, poseMotor(camGlobe)});
                }
            }
        };

        // ---- THE DATA FORM of the same tables, as the rail file's writer prints it. Each
        // key's numbers are the literals above, said once more; the gate is that both say the
        // same thing bit for bit.
        auto compass = [](double x, double alt, double z, double az, double pitch) {
            JsonValue o = JsonObj();
            JsonSet(o, "x", JsonNum(x));
            JsonSet(o, "alt", JsonNum(alt));
            JsonSet(o, "z", JsonNum(z));
            JsonSet(o, "az", JsonNum(az));
            JsonSet(o, "pitch", JsonNum(pitch));
            return o;
        };
        auto orbit = [](double lat, double lon, double alt, double tLat, double tLon) {
            JsonValue o = JsonObj();
            JsonSet(o, "lat", JsonNum(lat));
            JsonSet(o, "lon", JsonNum(lon));
            JsonSet(o, "alt", JsonNum(alt));
            JsonValue look = JsonObj();
            JsonSet(look, "lat", JsonNum(tLat));
            JsonSet(look, "lon", JsonNum(tLon));
            JsonSet(o, "lookAt", look);
            return o;
        };
        auto keyAt = [](double t, JsonValue at) {
            JsonValue k = JsonObj();
            JsonSet(k, "t", JsonNum(t));
            JsonSet(k, "at", std::move(at));
            return k;
        };
        auto keyView = [](double t, const char* view) {
            JsonValue k = JsonObj();
            JsonSet(k, "t", JsonNum(t));
            JsonSet(k, "view", JsonStr(view));
            return k;
        };
        auto keyPose = [](double t, const char* pose) {
            JsonValue k = JsonObj();
            JsonSet(k, "t", JsonNum(t));
            JsonSet(k, "pose", JsonStr(pose));
            return k;
        };
        auto keysSeg = [](std::vector<JsonValue> keys, double duration, double from, bool gravity) {
            JsonValue s = JsonObj();
            JsonSet(s, "kind", JsonStr("keys"));
            if (duration > 0.0) JsonSet(s, "duration", JsonNum(duration));
            if (from > 0.0) JsonSet(s, "from", JsonNum(from));
            if (gravity) JsonSet(s, "up", JsonStr("gravity"));
            JsonValue a = JsonArr();
            for (JsonValue& k : keys) a.arr.push_back(std::move(k));
            JsonSet(s, "keys", a);
            return s;
        };
        auto floodKeys = [&](bool helmLast) {
            std::vector<JsonValue> k;
            k.push_back(keyView(0.0, "orbit"));
            k.push_back(keyAt(8.0, orbit(41.9, -71.6, 800e3, 42.8183, -70.81)));
            k.push_back(keyAt(15.0, orbit(42.55, -70.98, 80e3, 42.8183, -70.81)));
            k.push_back(keyAt(21.0, orbit(42.74, -70.87, 7e3, 42.8183, -70.81)));
            k.push_back(keyAt(26.0, compass(-1400.0, 1500.0, 0.0, 93.0, -40.0)));
            k.push_back(keyAt(32.0, compass(-250.0, 9.0, -15.0, 92.0, -2.0)));
            if (helmLast) k.push_back(keyPose(40.0, "helm"));
            else k.push_back(keyAt(40.0, compass(120.0, 7.0, -10.0, 92.5, -1.5)));
            return k;
        };
        auto helmPoses = [&](bool back) {
            JsonValue a = JsonArr();
            JsonValue helm = JsonObj();
            JsonSet(helm, "name", JsonStr("helm"));
            JsonSet(helm, "at", compass(120.0, 7.0, -10.0, 92.5, -1.5));
            JsonSet(helm, "aim", JsonStr("fixedPoint"));
            JsonSet(helm, "standOffBelowTwistDeg", JsonNum(75.0));
            JsonValue off = JsonArr();
            off.arr.push_back(JsonNum(-300.0));
            off.arr.push_back(JsonNum(300.0));
            off.arr.push_back(JsonNum(0.0));
            JsonSet(helm, "standOff", off);
            a.arr.push_back(helm);
            if (back) {
                JsonValue hb = JsonObj();
                JsonSet(hb, "name", JsonStr("helmBack"));
                JsonSet(hb, "turnOf", JsonStr("helm"));
                JsonSet(hb, "turnYaw", JsonNum(3.14159265));
                JsonSet(hb, "turnPitch", JsonNum(-0.07));
                a.arr.push_back(hb);
            }
            return a;
        };
        auto railDoc = [&](const char* name, const char* readme, double marsR) {
            JsonValue d = JsonObj();
            JsonSet(d, "_readme", JsonStr(readme));
            JsonSet(d, "name", JsonStr(name));
            JsonValue segs = JsonArr();
            if (strcmp(name, "mars") == 0) {
                std::vector<JsonValue> k;
                k.push_back(keyAt(0.0, orbit(10, -25, marsR * 1.1, -12, -58)));
                k.push_back(keyAt(7.0, orbit(-7, -40, 800e3, -13, -62)));
                k.push_back(keyAt(13.0, orbit(-11, -52, 220e3, -13, -72)));
                k.push_back(keyAt(19.0, orbit(-13, -68, 150e3, -11, -90)));
                k.push_back(keyAt(26.0, orbit(-6, -98, 600e3, 18.6, -133.8)));
                k.push_back(keyAt(30.0, orbit(-4, -104, 900e3, 18.6, -133.8)));
                segs.arr.push_back(keysSeg(std::move(k), 0.0, 0.0, false));
            } else if (strcmp(name, "flood") == 0) {
                segs.arr.push_back(keysSeg(floodKeys(false), 0.0, 0.0, false));
            } else if (strcmp(name, "jetty") == 0) {
                std::vector<JsonValue> k;
                k.push_back(keyAt(0.0, compass(250.0, 5.0, 40.0, 94.0, -1.0)));
                k.push_back(keyAt(4.0, compass(250.0, 5.0, 40.0, 94.0, -1.0)));
                k.push_back(keyAt(9.0, compass(680.0, 14.0, 200.0, 192.0, -10.0)));
                k.push_back(keyAt(14.0, compass(650.0, 12.0, -30.0, 262.0, -8.0)));
                k.push_back(keyAt(18.0, compass(680.0, 16.0, -290.0, 300.0, -13.0)));
                k.push_back(keyAt(21.5, compass(520.0, 420.0, -140.0, 284.0, -56.0)));
                k.push_back(keyAt(25.0, compass(380.0, 1500.0, 10.0, 272.0, -88.0)));
                k.push_back(keyAt(28.0, compass(380.0, 1500.0, 10.0, 272.0, -88.0)));
                k.push_back(keyAt(31.0, orbit(42.79, -70.84, 7e3, 42.8183, -70.81)));
                k.push_back(keyAt(34.5, orbit(42.62, -70.95, 80e3, 42.8183, -70.81)));
                k.push_back(keyAt(38.0, orbit(41.9, -71.6, 800e3, 42.8183, -70.81)));
                k.push_back(keyAt(40.0, orbit(41.9, -71.6, 800e3, 42.8183, -70.81)));
                segs.arr.push_back(keysSeg(std::move(k), 0.0, 0.0, false));
            } else if (strcmp(name, "zoom") == 0) {
                std::vector<JsonValue> k;
                k.push_back(keyView(0.0, "orbit"));
                k.push_back(keyAt(8.0, orbit(41.9, -71.6, 800e3, 42.8183, -70.81)));
                k.push_back(keyAt(16.0, orbit(42.55, -70.98, 80e3, 42.8183, -70.81)));
                k.push_back(keyAt(23.0, orbit(42.74, -70.87, 7e3, 42.8183, -70.81)));
                k.push_back(keyAt(27.0, orbit(42.79, -70.84, 2400.0, 42.8183, -70.81)));
                k.push_back(keyAt(30.0, orbit(42.80, -70.835, 2200.0, 42.8183, -70.81)));
                segs.arr.push_back(keysSeg(std::move(k), 0.0, 0.0, false));
            } else if (strcmp(name, "classic") == 0) {
                std::vector<JsonValue> k;
                k.push_back(keyView(0.0, "orbit"));
                k.push_back(keyAt(3.0, orbit(41.2, -66.5, 500000.0, BathyModel::kOrgLat, BathyModel::kOrgLon)));
                k.push_back(keyAt(5.0, compass(-200.0, 1800.0, -2500.0, 22.0, -46.0)));
                k.push_back(keyAt(10.0, compass(-200.0, 1800.0, -2500.0, 22.0, -46.0)));
                k.push_back(keyAt(15.0, compass(522.0, 7.0, 72.0, 246.0, -4.0)));
                k.push_back(keyAt(25.0, compass(522.0, 7.0, 72.0, 246.0, -4.0)));
                segs.arr.push_back(keysSeg(std::move(k), 0.0, 0.0, false));
            } else if (strcmp(name, "droste") == 0) {
                JsonSet(d, "tower", JsonBool(true));
                JsonSet(d, "poses", helmPoses(false));
                segs.arr.push_back(keysSeg(floodKeys(true), 40.0, 0.0, true));
                JsonValue sp = JsonObj();
                JsonSet(sp, "kind", JsonStr("spiral"));
                JsonSet(sp, "base", JsonStr("helm"));
                JsonSet(sp, "upOf", JsonStr("helm"));
                JsonSet(sp, "law", JsonStr("dive"));
                JsonSet(sp, "delay", JsonNum(2.0));
                segs.arr.push_back(sp);
            } else if (strcmp(name, "droste-out") == 0) {
                JsonSet(d, "tower", JsonBool(true));
                JsonSet(d, "poses", helmPoses(true));
                segs.arr.push_back(keysSeg(floodKeys(true), 14.0, 26.0, true));
                JsonValue hold = JsonObj();
                JsonSet(hold, "kind", JsonStr("hold"));
                JsonSet(hold, "duration", JsonNum(2.0));
                JsonSet(hold, "base", JsonStr("helm"));
                segs.arr.push_back(hold);
                JsonValue in = JsonObj();
                JsonSet(in, "kind", JsonStr("spiral"));
                JsonSet(in, "levels", JsonNum(2));
                JsonSet(in, "base", JsonStr("helm"));
                JsonSet(in, "upOf", JsonStr("helm"));
                JsonSet(in, "law", JsonStr("leg"));
                JsonSet(in, "u0", JsonNum(0.0));
                JsonSet(in, "u1", JsonNum(2.0));
                segs.arr.push_back(in);
                JsonValue turn = JsonObj();
                JsonSet(turn, "kind", JsonStr("turn"));
                JsonSet(turn, "duration", JsonNum(3.0));
                JsonSet(turn, "base", JsonStr("helm"));
                JsonSet(turn, "upOf", JsonStr("helm"));
                JsonSet(turn, "to", JsonStr("helmBack"));
                JsonSet(turn, "level", JsonNum(2));
                segs.arr.push_back(turn);
                JsonValue out = JsonObj();
                JsonSet(out, "kind", JsonStr("spiral"));
                JsonSet(out, "levels", JsonNum(2));
                JsonSet(out, "base", JsonStr("helmBack"));
                JsonSet(out, "upOf", JsonStr("helm"));
                JsonSet(out, "law", JsonStr("leg"));
                JsonSet(out, "u0", JsonNum(2.0));
                JsonSet(out, "u1", JsonNum(0.0));
                segs.arr.push_back(out);
                std::vector<JsonValue> k;
                k.push_back(keyPose(0.0, "helmBack"));
                k.push_back(keyAt(7.0, compass(-900.0, 450.0, -60.0, 84.0, -17.0)));
                k.push_back(keyAt(13.0, orbit(42.74, -70.87, 7e3, 42.8183, -70.81)));
                k.push_back(keyAt(18.5, orbit(42.55, -70.98, 80e3, 42.8183, -70.81)));
                k.push_back(keyAt(23.5, orbit(41.9, -71.6, 800e3, 42.8183, -70.81)));
                k.push_back(keyView(28.0, "orbit"));
                segs.arr.push_back(keysSeg(std::move(k), 0.0, 0.0, true));
            }
            JsonSet(d, "segments", segs);
            SchemaChain chain;
            chain.a = &Rail::FileSchema();
            Complete(d, chain);
            return d;
        };
        struct RailCase { const char* name; const char* readme; double length; bool tower; bool mars; };
        const RailCase cases[] = {
            {"classic", "The classic flight (--rail): orbit to a hover over the mouth, then the north jetty tip. M6b.", 25.0, false, false},
            {"zoom", "The inlet zoom (--rail-zoom): orbit -> the Google pyramid -> the CUDEM estuary, no handoff. M6g.", 30.0, false, false},
            {"flood", "The flood ride (--rail-flood, the storm rail): orbit to the throat, ending at a helm mid-channel west of the gap. M6s.", 40.0, false, false},
            {"jetty", "The jetty pass (--rail-jetty): tip to tip at helm height, a climb to a bird's eye, then orbit. M7c/M7e.", 40.0, false, false},
            {"mars", "The Mars flyover (--planet mars): Valles Marineris west along its 4000 km, then up toward Tharsis.", 30.0, false, true},
            {"droste", "The dive (--rail-droste): the flood keys to the helm re-aimed at the fixed point, then the tower's spiral S^u(helm). M10.", 40.0 + 2.0 + 1.5 + 48.0 + 2.0, true, false},
            {"droste-out", "The out-and-back (--rail-droste-out): the flood's last leg, a breath, two levels in, the turn, two levels out, the climb to orbit. M10.", 111.0, true, false},
        };
        // The frame and the tower, as the session builds them at the recipes' leaf.
        const double kMarsR = 3389500.0;
        auto buildPortal = [&](const PoseFrame& fr, double twistDeg) {
            double pd[3];
            GlobeModel::LatLonDir(42.81826, -70.80045, pd);
            int lf = 0;
            uint32_t lix = 0, liy = 0;
            GlobeLayer::LeafOf(pd, 16, lf, lix, liy);
            double ld[3];
            GlobeLayer::LeafDir(lf, 16, lix, liy, ld);
            const double axisN[3] = {0.0, 0.0, 1.0};
            return droste::BuildPortal(lf, 16, lix, liy, ld, fr.east, fr.up, fr.north, fr.planetR,
                                       -5.0, 1.0, axisN, twistDeg * kPiL / 180.0);
        };
        int railCases = 0, railInstants = 0;
        uint64_t railBits = 0;
        for (const RailCase& rc : cases) {
            const double planetR = rc.mars ? kMarsR : R;
            const PoseFrame fr = rc.mars ? FrameFromAnchor(0.0, 0.0, kMarsR)
                                         : FrameFromAnchor(BathyModel::kOrgLat, BathyModel::kOrgLon, R);
            // (1) THE PRINTED TABLE vs THE CHECKED-IN FILE.
            const JsonValue doc = railDoc(rc.name, rc.readme, kMarsR);
            const std::string printed = SceneBuilder::WriteJson(doc);
            {
                std::string outDir = "out/rails_printed";
                std::filesystem::create_directories(outDir);
                FILE* f = fopen((outDir + "/" + rc.name + ".json").c_str(), "wb");
                if (f) {
                    fwrite(printed.data(), 1, printed.size(), f);
                    fclose(f);
                }
                std::string text;
                const std::string path = std::string("scenes/rails/") + rc.name + ".json";
                const bool have = SceneBuilder::ReadFile(path, text, nullptr);
                g.True(have, (std::string("[rail] ") + path + " is checked in").c_str());
                if (have) {
                    std::string norm;
                    for (char c : text) if (c != '\r') norm.push_back(c);
                    g.True(norm == printed, (std::string("[rail] ") + path +
                                             " equals the table printed through the writer (out/rails_printed/)").c_str());
                }
            }
            // (2) Rail::At vs the lambdas, bitwise, under both twists.
            for (double twist : {90.0, 0.0}) {
                if (!rc.tower && twist != 90.0) continue;
                const droste::Portal portal = rc.mars ? droste::Portal{} : buildPortal(fr, twist);
                Space planet, tangent;
                planet.name = "planet.re";
                planet.unitM = planetR;
                tangent.name = "tangent.test";
                tangent.unitM = planetR;
                tangent.parent = &planet;
                const double anchor[3] = {fr.up[0] * planetR, fr.up[1] * planetR, fr.up[2] * planetR};
                tangent.link = Placement::Frame(fr.east, fr.up, fr.north, anchor);
                Space cycle;
                if (portal.Valid()) {
                    cycle = Space::Cycle("droste.leaf", tangent,
                                         Placement::Similar(portal.p, portal.s, portal.axis, portal.twist));
                }
                Legacy L;
                L.planetR = planetR;
                for (int k = 0; k < 3; ++k) {
                    L.east0[k] = fr.east[k];
                    L.oDir[k] = fr.up[k];
                    L.north0[k] = fr.north[k];
                }
                L.fovY = 55.0f * 3.14159265f / 180.0f;
                L.portal = &portal;
                L.drosteLeaf = &cycle;
                L.twistDeg = twist;
                L.Build(rc.name, rc.mars, !rc.mars, !rc.mars, true);
                Rail rail;
                std::string why;
                const bool read = rail.FromJson(doc, std::string("rails/") + rc.name, &why);
                g.True(read, (std::string("[rail] ") + rc.name + " reads: " + why).c_str());
                RailFrame rf;
                rf.frame = fr;
                rf.fovY = L.fovY;
                rf.portal = &portal;
                rf.cycle = portal.Valid() ? &cycle : nullptr;
                rf.twistDeg = twist;
                rf.levelSec = 16.0;
                rf.levels = 3;
                rf.clampLevels = L.railDroste;
                rf.viewAt = [](const std::string&) -> const JsonValue* { return nullptr; };
                const bool resolved = read && rail.Resolve(rf, &why);
                g.True(resolved, (std::string("[rail] ") + rc.name + " resolves: " + why).c_str());
                if (!resolved) continue;
                g.True(rail.Tower() == rc.tower, "[rail] the file says whether it is a tower rail");
                const bool tower = rail.Tower() && portal.Valid();
                auto sameCam = [&](const Camera& a, const Camera& b) {
                    return a.px == b.px && a.py == b.py && a.pz == b.pz && a.yaw == b.yaw &&
                           a.pitch == b.pitch && a.fovY == b.fovY && a.nearZ == b.nearZ &&
                           a.speed == b.speed && a.upHint[0] == b.upHint[0] &&
                           a.upHint[1] == b.upHint[1] && a.upHint[2] == b.upHint[2];
                };
                uint64_t bad = 0, n = 0;
                auto probeAt = [&](double t) {
                    Camera cam0 = L.camGlobe;   // the live camera: what the rail writes into
                    Camera a = cam0, b = cam0;
                    int levelA = 0, levelB = 0;
                    double upA[3] = {0.0, 1.0, 0.0}, upB[3] = {0.0, 1.0, 0.0};
                    bool upWA = false;
                    if (tower) {
                        L.drosteRailPose(t, a, levelA, upA);
                        upWA = true;
                    } else {
                        L.railPose(t, a);
                    }
                    const RailSample s = tower ? rail.At(t) : rail.KeysAt(t);
                    Rail::AimCamera(s, b);
                    if (tower) {
                        levelB = s.level;
                        for (int k = 0; k < 3; ++k) upB[k] = s.up[k];
                    }
                    ++n;
                    const bool same = sameCam(a, b) && levelA == levelB && upWA == s.upWritten &&
                                      (!tower || (upA[0] == upB[0] && upA[1] == upB[1] && upA[2] == upB[2]));
                    if (!same) ++bad;
                    if (!tower) {
                        // For a plain rail At and KeysAt are one law (one open keys segment).
                        Camera c = cam0;
                        Rail::AimCamera(rail.At(t), c);
                        if (!sameCam(b, c)) ++bad;
                    }
                };
                for (int i = 0; i < 1000; ++i) probeAt(double(i) * (rc.length + 5.0) / 999.0);
                for (int k = 0; k < 3000; ++k) probeAt(double(k) / 30.0);   // the recorded instants
                railInstants += static_cast<int>(n);
                railBits += bad;
                char what[160];
                snprintf(what, sizeof what, "[rail] %s (twist %.0f): Rail::At == the hand tables' pose, level and up at %llu instants, bitwise",
                         rc.name, twist, static_cast<unsigned long long>(n));
                g.True(bad == 0, what);
                ++railCases;
            }
        }
        // The laws' own identities: a leg run down is the same leg with the ends swapped, and
        // the dive's clock clamps only when told to.
        for (int i = 0; i <= 64; ++i) {
            const double tau = 32.0 * double(i) / 64.0;
            g.Same(0.0 - Rail::LegU(tau, 32.0, 0.0 - 2.0), Rail::LegU(tau, 32.0, 2.0), "[rail] a leg 0 -> 2 is legU");
            g.Same(2.0 - Rail::LegU(tau, 32.0, 2.0 - 0.0), 2.0 - Rail::LegU(tau, 32.0, 2.0), "[rail] a leg 2 -> 0 is 2 - legU");
        }
        g.Same(Rail::DiveU(100.0, 16.0, 3, true), 3.0, "[rail] the dive clamps at its levels when the rail ends on a helm");
        g.True(Rail::DiveU(100.0, 16.0, 3, false) > 3.0, "[rail] ...and runs on when it does not");
        Log("[rail] %d rail cases held against the hand tables at %d instants (%llu differed); the printed "
            "tables written to out/rails_printed/", railCases, railInstants,
            static_cast<unsigned long long>(railBits));

        // (B) THE PORTAL NODE: its cycle IS the session's declaration -- Space::Cycle over the
        // tangent space with Placement::Similar of the link BuildPortal resolved.
        {
            const PoseFrame fr = FrameFromAnchor(BathyModel::kOrgLat, BathyModel::kOrgLon, R);
            Space planet, tangent;
            planet.name = "planet.re";
            planet.unitM = R;
            tangent.name = "tangent.merrimack";
            tangent.unitM = R;
            tangent.parent = &planet;
            const double anchor[3] = {fr.up[0] * R, fr.up[1] * R, fr.up[2] * R};
            tangent.link = Placement::Frame(fr.east, fr.up, fr.north, anchor);
            Portal node;
            node.Declared().name = "droste";
            Portal::Observers po;
            po.east = fr.east;
            po.up = fr.up;
            po.north = fr.north;
            po.planetR = R;
            po.compositor = nullptr;   // no ground: the leaf's ground reads 0 here
            po.hgtCh = -1;
            po.tangent = &tangent;
            po.globe = true;
            po.marsMode = false;
            g.True(node.Configure(po).size() == 1, "[portal] Configure reports the one observer it was not given (the ground)");
            g.True(node.Build() && node.Valid(), "[portal] the node builds the link from its declaration");
            const droste::Portal& link = node.Link();
            const droste::Portal ref = [&] {
                double pd[3];
                GlobeModel::LatLonDir(42.81826, -70.80045, pd);
                int lf = 0;
                uint32_t lix = 0, liy = 0;
                GlobeLayer::LeafOf(pd, 16, lf, lix, liy);
                double ld[3];
                GlobeLayer::LeafDir(lf, 16, lix, liy, ld);
                const double axisN[3] = {0.0, 0.0, 1.0};
                return droste::BuildPortal(lf, 16, lix, liy, ld, fr.east, fr.up, fr.north, R, 0.0, 1.0,
                                           axisN, 90.0 * kPiL / 180.0);
            }();
            g.Same(link.s, ref.s, "[portal] the link IS BuildPortal's (s)");
            for (int k = 0; k < 3; ++k) {
                g.Same(link.p[k], ref.p[k], "[portal] the link IS BuildPortal's (fixed point)");
                g.Same(link.centre[k], ref.centre[k], "[portal] the link IS BuildPortal's (centre)");
            }
            g.True(node.Cycle().parent == &tangent, "[portal] the cycle hangs under the tangent space");
            g.Same(node.Cycle().unitM, tangent.unitM, "[portal] the cycle inherits the tangent's unit length");
            SamePlacement(g, node.Cycle().link,
                          Placement::Similar(link.p, link.s, link.axis, link.twist),
                          "[portal] the cycle's link is Similar(p, s, axis, twist) of the portal");
            g.True(&node.Props() == &PortalSchema(), "[portal] a Portal's Schema IS the portals section's table");
            Portal off;
            off.Declared().enabled = false;
            off.Configure(po);
            g.True(!off.Build() && !off.Valid(), "[portal] a disabled portal builds no link");
            Portal mars;
            Portal::Observers pm = po;
            pm.marsMode = true;
            mars.Configure(pm);
            g.True(!mars.Build() && !mars.Valid(), "[portal] Mars has no tower");

            // THE DESTINATION (the Haulover portal demo): the root turned by the shortest arc
            // that carries the destination onto the leaf's place, then twisted. Its defining
            // property: the destination lands on the inner globe exactly where the leaf's own
            // place lands in the portal without one.
            g.True(!node.HasDestination(), "[portal] a declared portal has no destination until one is set");
            const double hLat = 25.8997, hLon = -80.1239;   // Baker's Haulover Inlet
            Portal to;
            to.Declared().name = "droste";
            to.SetDestination(hLat, hLon);
            to.Configure(po);
            g.True(to.Build() && to.Valid() && to.HasDestination(), "[portal] a portal with a destination builds");
            const droste::Portal& tl = to.Link();
            g.Same(tl.s, ref.s, "[portal] a destination does not change the scale");
            for (int k = 0; k < 3; ++k) {
                g.Same(tl.centre[k], ref.centre[k], "[portal] ...nor where the inner globe rests (its centre)");
            }
            double hd[3], pd0[3];
            GlobeModel::LatLonDir(hLat, hLon, hd);
            GlobeModel::LatLonDir(42.81826, -70.80045, pd0);
            int lf0 = 0;
            uint32_t lix0 = 0, liy0 = 0;
            GlobeLayer::LeafOf(pd0, 16, lf0, lix0, liy0);
            double ld0[3];
            GlobeLayer::LeafDir(lf0, 16, lix0, liy0, ld0);
            auto rows = [&](const double v[3], double o[3]) {
                o[0] = fr.east[0] * v[0] + fr.east[1] * v[1] + fr.east[2] * v[2];
                o[1] = fr.up[0] * v[0] + fr.up[1] * v[1] + fr.up[2] * v[2];
                o[2] = fr.north[0] * v[0] + fr.north[1] * v[1] + fr.north[2] * v[2];
                const double n = std::sqrt(o[0] * o[0] + o[1] * o[1] + o[2] * o[2]);
                for (int i = 0; i < 3; ++i) o[i] /= n;
            };
            double hT[3], lT[3];
            rows(hd, hT);
            rows(ld0, lT);
            // The root's surface points, tangent frame (the planet's centre is (0, -R, 0)).
            const double xH[3] = {R * hT[0], R * hT[1] - R, R * hT[2]};
            const double xL[3] = {R * lT[0], R * lT[1] - R, R * lT[2]};
            double imgH[3], imgL[3];
            tl.Apply(1.0, xH, imgH);
            ref.Apply(1.0, xL, imgL);
            double gap = 0.0, onGlobe = 0.0;
            for (int k = 0; k < 3; ++k) {
                gap = (std::max)(gap, std::fabs(imgH[k] - imgL[k]));
                onGlobe += (imgH[k] - tl.centre[k]) * (imgH[k] - tl.centre[k]);
            }
            g.Near(gap, 0.0, 1e-9, "[portal] the destination lands where the leaf's place lands without one (1e-9 m)");
            g.Near(std::sqrt(onGlobe), tl.radius, 1e-9, "[portal] ...on the inner globe's surface");
            // The arc alone turns the destination onto the leaf's place (QRotate, the motor's own).
            double ax[3], ang = 0.0;
            g.True(Portal::Carry(hd, ld0, fr.east, fr.up, fr.north, 0.0, ax, ang, nullptr),
                   "[portal] Carry builds the arc");
            {
                const double sh = std::sin(0.5 * ang);
                const double qArc[4] = {std::cos(0.5 * ang), sh * ax[0], sh * ax[1], sh * ax[2]};
                double x = hT[0], y = hT[1], z = hT[2];
                Motor::QRotate(qArc, x, y, z);
                g.Near(std::fabs(x - lT[0]) + std::fabs(y - lT[1]) + std::fabs(z - lT[2]), 0.0, 1e-14,
                       "[portal] the arc's rotor turns the destination's radial onto the leaf's");
                const double arcDeg = std::acos(hT[0] * lT[0] + hT[1] * lT[1] + hT[2] * lT[2]) * 180.0 / kPiL;
                g.Near(ang * 180.0 / kPiL, arcDeg, 1e-9, "[portal] ...by the great-circle angle between them");
            }
            g.True(ang > 0.0 && ang <= kPiL, "[portal] the rotation is the principal branch (0, pi]");
            const double antiPlanet[3] = {-ld0[0], -ld0[1], -ld0[2]};
            std::string awhy;
            g.True(!Portal::Carry(antiPlanet, ld0, fr.east, fr.up, fr.north, 0.0, ax, ang, &awhy) &&
                       awhy.find("antipode") != std::string::npos,
                   "[portal] the leaf's antipode is refused, with the reason");
        }

        // (C) THE EFFECT: the slice plane's edge registered and validated with the rest of the AST.
        {
            SlicePlane fx;
            fx.Declare("slice", true, 2.0);
            g.True(fx.Inputs().size() == 1 && fx.Inputs()[0].name == "user.plane" &&
                       fx.Outputs().size() == 1 && fx.Outputs()[0].name == "slice",
                   "[effect] slice.plane declares its ports");
            fx.RegisterEdges();
            bool found = false;
            for (const ga::ast::Edge& e : ga::ast::Edges()) {
                if (strcmp(e.from, "user.plane") == 0 && strcmp(e.to, "globe.ps") == 0 &&
                    strcmp(e.field, "slice") == 0) {
                    found = true;
                    g.True(!e.flip && strcmp(e.src.space, "world.m") == 0 && e.src.vNorth && e.dst.vNorth,
                           "[effect] the edge is the hand table's row (frames, no flip)");
                }
            }
            g.True(found, "[effect] RegisterEdges registered user.plane -> globe.ps 'slice'");
            g.True(ga::ast::Validate(), "[effect] the AST validates with the effect's edge in it");
            g.True(&fx.Props() == &SlicePlaneSchema(), "[effect] a SlicePlane's Schema IS the effects section's typed table");
            g.Same(fx.Declared().d, 2.0, "[effect] Declare carries the offset");
        }

        // (D) THE ENTITY: the spawn heading, and the solver it reads (by region, never on a clock).
        {
            Entity e;
            // THE SPAWN HEADING: a spawn without `az` is the translation it always was (the bow
            // north); with `az` the bow turns to that compass heading about the spawn point.
            e.SetSpawn(120.0, 0.0, -10.0);
            {
                const Motor ref = Motor::Translation(120.0, 0.0, -10.0);
                double a[8], b[8];
                double r1[4], d1[4], r2[4], d2[4];
                e.Spawn().Real(r1); e.Spawn().Dual(d1); ref.Real(r2); ref.Dual(d2);
                for (int k = 0; k < 4; ++k) { a[k] = r1[k]; a[4 + k] = d1[k]; b[k] = r2[k]; b[4 + k] = d2[k]; }
                bool same = true;
                for (int k = 0; k < 8; ++k) same = same && (a[k] == b[k]);
                g.True(same, "[entity] a spawn without az is bitwise the translation it always was");
            }
            e.SetSpawn(120.0, 0.0, -10.0, 90.0);
            {
                double bx = 0.0, by = 0.0, bz = 1.0;   // the bow at build: +z
                e.Spawn().TransformDir(bx, by, bz);
                g.Near(std::atan2(bx, bz) * 180.0 / kPiL, 90.0, 1e-9, "[entity] az 90 turns the bow east (the telemetry's heading)");
                double px = 0.0, py = 0.0, pz = 0.0;
                e.Spawn().TransformPoint(px, py, pz);
                g.Near(std::fabs(px - 120.0) + std::fabs(py) + std::fabs(pz + 10.0), 0.0, 1e-12,
                       "[entity] ...about the spawn point, which stays where the sugar put it");
            }
            g.True(!e.Active() && !e.Helming(), "[entity] a node without a hull is inert");
            g.True(EntitySchema().Find("mirrorCadence") == nullptr,
                   "[entity] the entities section has no mirror cadence: the hull reads the solver by region every frame");
            WeatherManager wm;
            g.Same(wm.SolverAsOf(), WeatherManager::kNeverRead, "[entity] no solver has answered: asOf is never");
            FrameInfo fi;
            g.True(fi.asOf == 0.0 && fi.quanta == 0, "[entity] FrameInfo carries asOf and the clock's quanta");
            TreeWater tw;
            tw.Configure(&wm, nullptr, nullptr, nullptr, 1.0, 1.0, 1.0);
            g.Has(tw.Describe(120.0, -10.0, 0.0), "no solver has answered", "[entity] TreeWater::Describe reports that no solver has answered");
        }

        // (E) THE GATE: one motor carries a body from a box to a place on the same planet.
        {
            const PoseFrame frM = FrameFromAnchor(BathyModel::kOrgLat, BathyModel::kOrgLon, R);
            Space planetG, rootG;
            planetG.name = "planet.re";
            planetG.unitM = R;
            planetG.extentM = 2.0 * R;
            rootG.name = "tangent.merrimack";
            rootG.unitM = R;
            rootG.extentM = 2.0 * R;
            rootG.parent = &planetG;
            const double anchorM[3] = {frM.up[0] * R, frM.up[1] * R, frM.up[2] * R};
            rootG.link = Placement::Frame(frM.east, frM.up, frM.north, anchorM);
            Gateway gate;
            gate.Declared().name = "haulover";
            gate.Declared().toLat = 25.8997;
            gate.Declared().toLon = -80.1239;
            gate.Declared().toAz = 90.0;
            g.True(gate.Build(planetG, rootG, R, 600.0, 0.0, -10.0, 90.0) && gate.Valid(),
                   "[gate] a box at the inlet builds its carry to Haulover");
            // The box: its centre and its forward face.
            g.True(gate.Inside(600.0, 0.0, -10.0), "[gate] the box's centre is inside");
            g.True(gate.Inside(603.9, 14.9, 19.9) && !gate.Inside(604.1, 0.0, -10.0) &&
                       !gate.Inside(600.0, 15.1, -10.0) && !gate.Inside(600.0, 0.0, 20.1),
                   "[gate] ...and the default 60 x 30 x 8 m box ends where its half-sizes do (heading east: depth along x, width along z)");
            // THE CARRY: the entry box's centre lands on the destination's origin, and the box's
            // forward (east, at az 90) leaves along the exit heading (east, toAz 90).
            double cx = 600.0, cy = 0.0, cz = -10.0;
            gate.Carry().TransformPoint(cx, cy, cz);
            g.Near(std::fabs(cx) + std::fabs(cy) + std::fabs(cz), 0.0, 1e-9, "[gate] the box's centre lands on the destination's origin");
            double fx = 1.0, fy = 0.0, fz = 0.0;
            gate.Carry().TransformDir(fx, fy, fz);
            g.Near(std::atan2(fx, fz) * 180.0 / kPiL, 90.0, 1e-9, "[gate] the box's forward leaves along the exit heading");
            g.Near(fy, 0.0, 1e-12, "[gate] ...and stays level: both boxes are y-up in their own spaces");
            // A body keeps its motion: momentum turned with the pose, the body twist unchanged.
            RigidBody body;
            body.pose = Motor::Translation(600.0, 0.2, -10.0);
            Bivector tw0 = Bivector::Zero();
            tw0.b[2] = 7.5;    // 7.5 m/s along its own bow (+z body)
            tw0.a[1] = 0.1;    // yawing
            body.SetTwist(tw0);
            double p0[3];
            body.LinearMomentumWorld(p0);
            body.Carry(gate.Carry());
            double p1[3];
            body.LinearMomentumWorld(p1);
            const Motor& K = gate.Carry();
            double pk[3] = {p0[0], p0[1], p0[2]};
            K.TransformDir(pk[0], pk[1], pk[2]);
            g.Near(std::fabs(p1[0] - pk[0]) + std::fabs(p1[1] - pk[1]) + std::fabs(p1[2] - pk[2]), 0.0, 1e-9,
                   "[gate] the carried body's world momentum is the old one turned by the carry");
            const Bivector& tw1 = body.Twist();
            g.Near(std::fabs(tw1.b[2] - 7.5) + std::fabs(tw1.a[1] - 0.1), 0.0, 1e-9,
                   "[gate] ...and its body twist is unchanged: it moves in its own frame as it did");
            // THE DESTINATION SPACE: its up at its origin is the planet's radial at the place.
            const Placement W = gate.Destination().To(rootG);
            double upD[3];
            const double ey[3] = {0.0, 1.0, 0.0};
            W.ApplyDir(ey, upD);
            const PoseFrame frH = FrameFromAnchor(25.8997, -80.1239, R);
            double upH[3];
            rootG.link.Inverse().ApplyDir(frH.up, upH);
            g.Near(std::fabs(upD[0] - upH[0]) + std::fabs(upD[1] - upH[1]) + std::fabs(upD[2] - upH[2]), 0.0, 1e-12,
                   "[gate] the destination space's up is the planet's radial at Haulover");
            const double tilt = std::acos((std::min)(1.0, upD[1])) * 180.0 / kPiL;
            g.True(tilt > 17.0 && tilt < 20.0, "[gate] ...18-19 degrees from the Merrimack's (the arc between the inlets)");
            double ox = 0.0, oy = 0.0, oz = 0.0;
            gate.DestinationInSource().TransformPoint(ox, oy, oz);
            double orig[3];
            const double z0[3] = {0.0, 0.0, 0.0};
            W.Apply(z0, orig);
            g.Near(std::fabs(ox - orig[0]) + std::fabs(oy - orig[1]) + std::fabs(oz - orig[2]), 0.0, 1e-6,
                   "[gate] the destination's motor places its origin where its placement does");
            // THE CHART: the destination's places, round trip.
            const Space::Anchor& ch = gate.Chart();
            double la = 0.0, lo = 0.0, rx = 0.0, rz = 0.0;
            ch.LatLonOf(1000.0, -500.0, la, lo);
            ch.FlatOf(la, lo, rx, rz);
            g.Near(std::fabs(rx - 1000.0) + std::fabs(rz + 500.0), 0.0, 1e-9, "[gate] the destination's chart round-trips");
            g.Near(la, 25.8997 - 500.0 / 110574.0, 1e-12, "[gate] ...by the engine's law at the place");
            // THE WATER: with no chart a TreeWater places a point by the root's constants, bitwise.
            TreeWater twG;
            double la0 = 0.0, lo0 = 0.0;
            twG.PlaceOf(120.0, -10.0, la0, lo0);
            g.True(la0 == BathyModel::kOrgLat + (-10.0) / BathyModel::kMPerLat &&
                       lo0 == BathyModel::kOrgLon + 120.0 / BathyModel::kMPerLon,
                   "[gate] a TreeWater with no chart places a point by the root's constants, bitwise");
            twG.SetChart(&ch);
            twG.PlaceOf(0.0, 0.0, la0, lo0);
            // M13 step 2: the chart's own origin IS the destination -- but the answer now comes
            // through the frame's rows and two transcendentals (Space::Anchor::PlaceOf, the place
            // on the sphere the mesh is drawn on) rather than off the declaration, so it is exact
            // to a nanodegree (a tenth of a millimetre of ground) and not bitwise.
            g.Near(la0, 25.8997, 1e-9, "[gate] ...and through a gate's chart, at the destination's place");
            g.Near(lo0, -80.1239, 1e-9, "[gate] ...in longitude too");
            // And away from the anchor the exact map is what it is FOR: the linear chart drifts
            // 5.6 m per km north of the sphere, which is what the water used to read.
            double laE = 0.0, loE = 0.0, laL = 0.0, loL = 0.0;
            twG.PlaceOf(0.0, 5000.0, laE, loE);
            ch.LatLonOf(0.0, 5000.0, laL, loL);
            const double driftM = (laL - laE) * 111195.0;
            g.Near(driftM, 28.1, 1.5, "[gate] ...and stands 28 m off the linear chart at 5 km north");
            // THE WINDOW'S TEST (Gateway::SeenThrough, the shader's GateThrough line for line). The
            // box at (600, 0, -10) heading east: 8 m deep along x, 60 m across along z, 30 m tall.
            const double eyeW[3] = {500.0, 2.0, -10.0};
            const double farBehind[3] = {900.0, 0.0, -10.0};
            const double besideP[3] = {900.0, 0.0, 200.0};
            const double beforeBox[3] = {560.0, 1.0, -10.0};
            const double inBox[3] = {601.0, 0.0, -5.0};
            g.True(gate.SeenThrough(eyeW, farBehind), "[gate] a point beyond the box, straight through it, is seen through the window");
            g.True(gate.SeenThrough(eyeW, inBox), "[gate] ...and so is a point inside the box");
            g.True(!gate.SeenThrough(eyeW, beforeBox), "[gate] a point between the eye and the box is not");
            g.True(!gate.SeenThrough(eyeW, besideP), "[gate] a point whose ray passes beside the box is not");
            const double eyeIn[3] = {600.0, 0.0, -10.0};
            g.True(gate.SeenThrough(eyeIn, besideP) && gate.SeenThrough(eyeIn, beforeBox),
                   "[gate] from inside the box every ray starts in the window");
            const double eyeAbove[3] = {600.0, 100.0, -10.0};
            const double belowBox[3] = {600.0, -40.0, -10.0};
            g.True(gate.SeenThrough(eyeAbove, belowBox), "[gate] the box is a box: looking down through its top works too");
        }
    }

    // ---- 13. [reload] THE WHOLE SCENE, DIFFED AND APPLIED (M12 step 5f) ---------------------
    // 5c pinned the reload law for ONE file; this pins it for the document. The four cases the
    // law has to get right are one document apart: a Hot key that MOVED, a Restart key that
    // moved (reported, NOT applied, and not reverted either -- the run keeps what it was built
    // with), a key the candidate no longer CARRIES (which must come back as its DECLARED
    // default, never as the value the run happens to hold), and a named-array entry the overlay
    // REMOVED (5a's `"remove": true`, which no property write can stand in for and which the
    // residue walk therefore has to name). Beside them: the address of a named element, the
    // pick that lets a typed element answer to two schemas without either seeing the other's
    // keys, and idempotence -- the same candidate twice is zero fields changed.
    {
        std::string why;
        const JsonValue base = ParseOrDie(g,
            "{\"scene\": {\"name\": \"r\"},"
            " \"views\": [{\"name\": \"sea\", \"fovY\": 50}, {\"name\": \"spare\"}],"
            " \"effects\": [{\"name\": \"cut\", \"type\": \"slice.plane\", \"d\": 3}],"
            " \"water\": {\"closures\": {\"foamOpacity\": 0.5},"
            "            \"wavefield\": {\"cellM\": 3, \"bankTexelM\": 1.2}}}");
        SceneBuilder was;
        g.True(was.Overlay(base, "base", "", &why) && was.Resolve(&why),
               (std::string("[reload] the document the run is in resolves: ") + why).c_str());
        // The candidate: the same base, then ONE overlay that moves a Hot key, moves a Restart
        // key, drops a key entirely and removes a named entry. The fold is the real one.
        const JsonValue edit = ParseOrDie(g,
            "{\"views\": [{\"name\": \"spare\", \"remove\": true}],"
            " \"water\": {\"wavefield\": {\"cellM\": 4, \"bankTexelM\": 2}}}");
        const JsonValue baseNoFoam = ParseOrDie(g,
            "{\"scene\": {\"name\": \"r\"},"
            " \"views\": [{\"name\": \"sea\", \"fovY\": 50}, {\"name\": \"spare\"}],"
            " \"effects\": [{\"name\": \"cut\", \"type\": \"slice.plane\", \"d\": 3}],"
            " \"water\": {\"wavefield\": {\"cellM\": 3, \"bankTexelM\": 1.2}}}");
        SceneBuilder now;
        g.True(now.Overlay(baseNoFoam, "base", "", &why) && now.Overlay(edit, "edit", "", &why) &&
                   now.Resolve(&why),
               (std::string("[reload] the candidate resolves: ") + why).c_str());

        // (A) THE ADDRESS: a section, a nested key, and a NAMED array element by its name.
        g.True(SceneReload::At(was.Resolved(), "water.wavefield.cellM") != nullptr &&
                   SceneReload::At(was.Resolved(), "water.wavefield.cellM")->number == 3.0,
               "[reload] At walks a dotted section path");
        g.True(SceneReload::At(was.Resolved(), "views.sea") != nullptr,
               "[reload] At addresses a named array element by its name");
        g.True(SceneReload::At(was.Resolved(), "views.nope") == nullptr,
               "[reload] At answers null for an element that is not there");
        g.True(SceneReload::At(now.Resolved(), "views.spare") == nullptr,
               "[reload] the named-array remove took the element out of the fold");

        // (B) THE PICK: a typed element answers to TWO schemas and neither sees the other's keys.
        const Schema* effectS = SceneFileSchema().Find("effects")->sub;
        const JsonValue* fx = SceneReload::At(now.Resolved(), "effects.cut");
        g.True(fx != nullptr, "[reload] the typed element is in the document");
        if (fx) {
            const JsonValue own = SceneReload::Pick(*fx, *effectS);
            const JsonValue typed = SceneReload::Pick(*fx, SlicePlaneSchema());
            g.True(own.Get("enabled") && !own.Get("d"),
                   "[reload] the element's own half keeps enabled and drops the type's d");
            g.True(typed.Get("d") && !typed.Get("name"),
                   "[reload] the type's half keeps d and drops the element's name");
        }

        // (C) THE DIFF AND THE APPLY, on the water section.
        const Schema* waterS = SceneFileSchema().Find("water")->sub;
        PropSet before = PropSet::Defaults(*waterS, waterS->Prototype());
        g.True(before.Merge(SceneReload::Pick(*SceneReload::At(was.Resolved(), "water"), *waterS),
                            "water", &why),
               "[reload] the run's water set reads back from its document");
        PropSet acc(*waterS);
        SceneReload::Verdict v;
        g.True(SceneReload::Accept(*waterS, *SceneReload::At(now.Resolved(), "water"), before,
                                   "water", acc, v, &why),
               (std::string("[reload] the candidate's water section accepts: ") + why).c_str());
        g.True(v.hot == 2, "[reload] two hot keys: the moved cellM and the dropped foamOpacity");
        g.True(v.restart == 1 && !v.restartKeys.empty() &&
                   v.restartKeys[0] == "wavefield.bankTexelM",
               "[reload] one Restart key, named");
        g.Same(acc.ValueAt("wavefield.cellM").number, 4.0, "[reload] a Hot key is applied");
        g.Same(acc.ValueAt("wavefield.bankTexelM").number, FloatAsDouble(1.2f),
               "[reload] a Restart key is NOT applied: the run keeps what it was built with");
        g.Same(acc.ValueAt("closures.foamOpacity").number, FloatAsDouble(0.72f),
               "[reload] a key the candidate no longer carries reverts to its DECLARED default");

        // (D) IDEMPOTENCE: the same candidate over the accepted set is zero fields changed.
        PropSet again(*waterS);
        SceneReload::Verdict v2;
        g.True(SceneReload::Accept(*waterS, *SceneReload::At(now.Resolved(), "water"), acc,
                                   "water", again, v2, &why),
               "[reload] the same candidate accepts a second time");
        g.True(v2.hot == 0, "[reload] idempotent: nothing hot moves the second time");
        g.True(v2.restart == 1,
               "[reload] the Restart key is reported every time, because it is still not applied");
        g.True(SceneBuilder::WriteJson(again.ToJson()) == SceneBuilder::WriteJson(acc.ToJson()),
               "[reload] and the accepted set is the same set");

        // (E) THE RESIDUE: what no target carries is NAMED, and what a target carries is not.
        std::vector<std::string> covered{"water", "views.sea"};
        std::vector<std::string> res;
        SceneReload::Residue(was.Resolved(), now.Resolved(), "", covered, res);
        bool sawGone = false, sawWater = false, sawSeaFov = false;
        for (const std::string& p : res) {
            if (p == "views.spare (gone)") sawGone = true;
            if (p.rfind("water", 0) == 0) sawWater = true;
            if (p.rfind("views.sea", 0) == 0) sawSeaFov = true;
        }
        g.True(sawGone, "[reload] a named-array entry the overlay removed is reported by name");
        g.True(!sawWater, "[reload] a covered section is the target's business, not the residue's");
        g.True(!sawSeaFov, "[reload] a covered element likewise");
        std::vector<std::string> none;
        SceneReload::Residue(was.Resolved(), was.Resolved(), "", none, none);
        g.True(none.empty(), "[reload] a document against itself has no residue");

        // (E2) THE LIST LAW. A PropSet carries no list, so a list under a target is a VALUE
        // compared whole: Hot where the target's fan-out consumes it, reported as needing a
        // restart where nothing does, and silent where it did not change. (The defect this pins:
        // the water fan-out once rebuilt its section from a PropSet and handed the component
        // an EMPTY fleet on every reload.)
        {
            const JsonValue a = ParseOrDie(g,
                "{\"fleet\": {\"enabled\": false, \"boats\": [{\"speed\": 4}]}}");
            const JsonValue b = ParseOrDie(g,
                "{\"fleet\": {\"enabled\": false, \"boats\": [{\"speed\": 4}, {\"speed\": 5}]}}");
            SceneReload::Verdict eat, skip, same;
            SceneReload::Lists(*waterS, &a, b, {"fleet.boats"}, eat);
            SceneReload::Lists(*waterS, &a, b, {}, skip);
            SceneReload::Lists(*waterS, &a, a, {"fleet.boats"}, same);
            g.True(eat.hot == 1 && eat.restart == 0 && !eat.hotKeys.empty() &&
                       eat.hotKeys[0] == "fleet.boats=[2 elements]",
                   "[reload] a changed list the fan-out consumes is hot, named with its size");
            g.True(skip.hot == 0 && skip.restart == 1,
                   "[reload] a changed list nothing consumes is reported as needing a restart");
            g.True(same.hot == 0 && same.restart == 0, "[reload] an unchanged list is silent");
        }

        // (F) AN UNKNOWN KEY REFUSES THE WHOLE RELOAD -- at the fold, before any target is
        // reached, which is why a refused candidate cannot half-apply.
        SceneBuilder bad;
        std::string badWhy;
        g.True(bad.Overlay(base, "base", "", &badWhy) &&
                   bad.Overlay(ParseOrDie(g, "{\"water\": {\"nope\": 1}}"), "edit", "", &badWhy),
               "[reload] the bad overlay applies to the document");
        g.True(!bad.Resolve(&badWhy), "[reload] an unknown key refuses the candidate");
        g.Has(badWhy, "water.nope", "[reload] and the refusal names the path");
    }

    if (g.ok) {
        Log("[scene] ---- PASS (%d checks): Registry<T> == VesselRegistry / LoaderRegistry on the "
            "built-in kinds, the property table (defaults, units through GaUnits, the Velocity-"
            "into-Length and datum refusals with AcceptsFrom's why, unknown keys, ApplyTo, Diff, "
            "the round trip), the fold with override (defaults < base < include < --set; atomic "
            "placements; named arrays in place / remove / append; completion), the placement "
            "sugar's four spellings == the session's pose maps bit for bit, WriteJson round-trips, "
            "refusals carry the node path, Node (the fold, the walk, a component), the Options "
            "shim (the --boat refusal, the rail's implication laws), [view] View::Level "
            "against the rasterizer's own frame, against poseMotor, under a roll and under "
            "itself, and [water] the one Apply (the two tables held equal, the reload law's "
            "revert-to-default, the refusals, the Restart key, idempotence), and [fold] the "
            "fixed point (an unnamed list replaced whole, `\"base\": \"\"` as no base, "
            "Resolved -> WriteJson -> Load -> Resolve the identity), and the nodes of 5e -- "
            "[rail] the shipped rails printed equal to their files and Rail::At bitwise against "
            "the hand tables, [portal] the cycle as the session declared it, [effect] the slice "
            "plane's edge in a valid AST, [entity] the freshness defaults -- and [reload] the "
            "whole scene of 5f (the address and the pick, a Hot key applied, a Restart key "
            "reported and not applied, a removed key back to its declared default, a named-array "
            "remove named by the residue, a list under a target as a value -- hot where consumed, "
            "reported where not --, idempotence, and an unknown key refusing the candidate whole) "
            "----",
            g.checks);
    } else {
        Log("[scene] ---- FAIL (%d checks) ----", g.checks);
    }
    return g.ok;
}

}  // namespace ga::scene
