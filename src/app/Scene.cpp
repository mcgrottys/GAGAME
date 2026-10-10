// Scene - M12 step 5d: the resolved document, read into the typed scene. See Scene.h.
#include "app/Scene.h"

#include "app/Options.h"
#include "scene/SceneBuilder.h"

#include <cstdio>
#include <cstdlib>
#include <string>
#include <utility>

namespace ga::app {
namespace {

using ga::scene::PropDecl;
using ga::scene::PropSet;
using ga::scene::PropValue;
using ga::scene::Schema;
using ga::scene::SchemaChain;

bool Refuse(std::string* why, const std::string& s) {
    if (why) *why = s;
    return false;
}

const JsonValue& Empty() {
    static const JsonValue v;
    return v;
}

// ================================================================================================
//  ElementReader - one list element (or one nested object of it) read through its schema chain by
//  the ONE parser, scene::ParseProp: the number law, the declared unit, the enum names and every
//  refusal are the table's, so a list element is read exactly as a section is. A key the element
//  does not carry leaves the caller's default standing (the document is COMPLETE after Resolve,
//  so in practice every declared key is there); the first refusal latches `ok` and names its path.
// ================================================================================================
class ElementReader {
public:
    ElementReader(SchemaChain chain, const JsonValue& e, std::string path, std::string* why,
                  bool* ok)
        : m_chain(chain), m_e(&e), m_path(std::move(path)), m_why(why), m_ok(ok) {}

    void Str(const char* key, std::string& f) {
        PropValue v;
        if (Read(key, v)) f = v.s;
    }
    void Bool(const char* key, bool& f) {
        PropValue v;
        if (Read(key, v)) f = v.b;
    }
    void F32(const char* key, float& f) {
        PropValue v;
        if (Read(key, v)) f = static_cast<float>(v.n);
    }
    void F64(const char* key, double& f) {
        PropValue v;
        if (Read(key, v)) f = v.n;
    }
    void I32(const char* key, int& f) {
        PropValue v;
        if (Read(key, v)) f = static_cast<int>(v.n);
    }
    void U32(const char* key, uint32_t& f) {
        PropValue v;
        if (Read(key, v)) f = static_cast<uint32_t>(v.n);
    }
    void D3(const char* key, double f[3]) {
        PropValue v;
        if (Read(key, v)) {
            for (int k = 0; k < 3; ++k) f[k] = v.v[k];
        }
    }
    void Enum(const char* key, int& f) {
        PropValue v;
        if (Read(key, v)) f = v.e;
    }
    // A nested Object (a view's viewport and follow). An absent key gives a reader over nothing,
    // which reads nothing -- the caller's defaults stand.
    ElementReader Nested(const char* key) const {
        SchemaChain sub;
        const PropDecl* d = m_chain.Find(key);
        const JsonValue* v = m_e->Get(key);
        if (d && d->sub) sub.a = d->sub;
        return ElementReader(sub, (v && v->type == JsonValue::Type::Object) ? *v : Empty(),
                             m_path + "." + key, m_why, m_ok);
    }

private:
    bool Read(const char* key, PropValue& out) {
        if (!*m_ok) return false;
        const PropDecl* d = m_chain.Find(key);
        const JsonValue* v = m_e->Get(key);
        if (!d || !v) return false;
        if (!ga::scene::ParseProp(*d, *v, m_path + "." + key, out, m_why)) {
            *m_ok = false;
            return false;
        }
        return true;
    }

    SchemaChain m_chain;
    const JsonValue* m_e;
    std::string m_path;
    std::string* m_why;
    bool* m_ok;
};

const JsonValue* ListOf(const JsonValue& doc, const char* key) {
    const JsonValue* v = doc.Get(key);
    return (v && v->type == JsonValue::Type::Array) ? v : nullptr;
}

}  // namespace

// ---- the lookups ------------------------------------------------------------------------------

const SceneView* Scene::View(const std::string& name) const {
    for (const SceneView& v : views) {
        if (v.p.name == name) return &v;
    }
    return nullptr;
}
const SceneView* Scene::Start() const { return View(scene.view); }

const ScenePortal* Scene::Portal(const char* name) const {
    for (const ScenePortal& p : portals) {
        if (p.p.name == name) return &p;
    }
    return nullptr;
}
const SceneEntity* Scene::Entity(const char* name) const {
    for (const SceneEntity& e : entities) {
        if (e.p.name == name) return &e;
    }
    return nullptr;
}
const SceneEffect* Scene::Effect(const char* name) const {
    for (const SceneEffect& e : effects) {
        if (e.p.name == name) return &e;
    }
    return nullptr;
}
const SceneLayer* Scene::Layer(const char* name) const {
    for (const SceneLayer& l : layers) {
        if (l.p.name == name) return &l;
    }
    return nullptr;
}
const SceneTool* Scene::Tool(const char* name) const {
    for (const SceneTool& t : tools) {
        if (t.name == name) return &t;
    }
    return nullptr;
}
const SceneEffect* Scene::EffectOfType(const char* type) const {
    for (const SceneEffect& e : effects) {
        if (e.p.enabled && e.p.type == type) return &e;
    }
    return nullptr;
}
bool Scene::LayerOn(const char* name) const {
    const SceneLayer* l = Layer(name);
    return l && l->p.enabled;
}
std::vector<std::string> Scene::LayerOrder() const {
    std::vector<std::string> names;
    for (const SceneLayer& l : layers) {
        if (l.p.enabled) names.push_back(l.p.name);
    }
    return names;
}

// ---- the read ---------------------------------------------------------------------------------

bool ReadScene(const JsonValue& doc, Scene& out, std::string* why) {
    if (doc.type != JsonValue::Type::Object) return Refuse(why, "the scene: expected an object");
    out = Scene{};
    scene::SceneDocument& sections = out;
    const Schema& root = scene::SceneFileSchema();
    // THE SECTIONS: one PropSet over the file's own table, merged and applied. No second reader.
    PropSet set = PropSet::Defaults(root, &sections);
    if (!set.Merge(doc, "scene", why)) return false;
    if (!set.ApplyTo(&sections, nullptr, why)) return false;
    // The `water` section as written: WaterComponent's table takes three of its keys (the M8
    // water scene, which reaches the document as merrimack.json's `include` overlay).
    if (const JsonValue* w = doc.Get("water")) out.waterDoc = *w;
    out.shadersW = Widen(out.data.shaders.c_str());
    out.dumpW = Widen(out.capture.dump.c_str());
    out.hdrW = Widen(out.capture.hdr.c_str());
    out.railDirW = Widen(out.capture.railDir.c_str());

    bool ok = true;
    // THE LISTS. Each element through its own schema chain (its own table, plus its type's for a
    // typed element), the `at` kept as written.
    if (const JsonValue* a = ListOf(doc, "views")) {
        const PropDecl* decl = root.Find("views");
        for (const JsonValue& e : a->arr) {
            SceneView v;
            ElementReader r(scene::ElementChain(*decl, e), e, "views", why, &ok);
            r.Str("name", v.p.name);
            r.F32("fovY", v.p.fovY);
            r.Str("gauge", v.p.gauge);
            r.F32("nearZ", v.p.nearZ);
            r.Bool("reversedZ", v.p.reversedZ);
            r.Str("target", v.p.target);
            ElementReader vp = r.Nested("viewport");
            vp.U32("x", v.p.viewport.x);
            vp.U32("y", v.p.viewport.y);
            vp.U32("w", v.p.viewport.w);
            vp.U32("h", v.p.viewport.h);
            ElementReader fw = r.Nested("follow");
            fw.Str("target", v.p.follow.target);
            fw.F64("back", v.p.follow.back);
            fw.F64("up", v.p.follow.up);
            fw.F64("aimLift", v.p.follow.aimLift);
            if (const JsonValue* at = e.Get("at")) {
                v.hasAt = true;
                v.at = *at;
            }
            if (const JsonValue* ia = ListOf(e, "interests")) {
                for (const JsonValue& ie : ia->arr) {
                    const std::string in = ie.Str("name");
                    if (in.empty()) {
                        return Refuse(why, "views." + v.p.name + ".interests: an entry needs a name");
                    }
                    v.interests.push_back(in);
                }
            }
            if (!ok) return false;
            out.views.push_back(std::move(v));
        }
    }
    if (const JsonValue* a = ListOf(doc, "portals")) {
        const PropDecl* decl = root.Find("portals");
        for (const JsonValue& e : a->arr) {
            ScenePortal p;
            ElementReader r(scene::ElementChain(*decl, e), e, "portals", why, &ok);
            r.Str("name", p.p.name);
            r.Bool("enabled", p.p.enabled);
            r.F64("lat", p.p.lat);
            r.F64("lon", p.p.lon);
            r.F64("toLat", p.p.toLat);
            r.F64("toLon", p.p.toLon);
            {
                const bool hasLat = e.Get("toLat") != nullptr, hasLon = e.Get("toLon") != nullptr;
                if (hasLat != hasLon) {
                    return Refuse(why, "portals." + p.p.name +
                                           ": a destination is a place -- declare toLat and toLon together");
                }
                p.hasTo = hasLat;
            }
            r.I32("level", p.p.level);
            r.F64("fill", p.p.fill);
            r.F64("twistDeg", p.p.twistDeg);
            r.Enum("lighting", p.p.lighting);
            if (!ok) return false;
            out.portals.push_back(std::move(p));
        }
    }
    if (const JsonValue* a = ListOf(doc, "gates")) {
        const PropDecl* decl = root.Find("gates");
        for (const JsonValue& e : a->arr) {
            SceneGate g;
            ElementReader r(scene::ElementChain(*decl, e), e, "gates", why, &ok);
            r.Str("name", g.p.name);
            r.Bool("enabled", g.p.enabled);
            r.D3("size", g.p.size);
            r.F64("fromLat", g.p.fromLat);
            r.F64("fromLon", g.p.fromLon);
            r.F64("toLat", g.p.toLat);
            r.F64("toLon", g.p.toLon);
            r.F64("toAz", g.p.toAz);
            // A place is a latitude AND a longitude, or none (the scene's own frame).
            {
                const bool hasLat = e.Get("fromLat") != nullptr, hasLon = e.Get("fromLon") != nullptr;
                if (hasLat != hasLon) {
                    return Refuse(why, "gates." + g.p.name +
                                           ": the box's place is a place -- declare fromLat and fromLon together");
                }
                g.hasFrom = hasLat;
            }
            {
                const bool hasLat = e.Get("toLat") != nullptr, hasLon = e.Get("toLon") != nullptr;
                if (hasLat != hasLon) {
                    return Refuse(why, "gates." + g.p.name +
                                           ": a destination is a place -- declare toLat and toLon together");
                }
                g.hasTo = hasLat;
            }
            if (const JsonValue* at = e.Get("at")) {
                g.hasAt = true;
                g.at = *at;
            }
            if (const JsonValue* ta = e.Get("toAt")) {
                g.hasToAt = true;
                g.toAt = *ta;
            }
            if (!ok) return false;
            out.gates.push_back(std::move(g));
        }
    }
    if (const JsonValue* a = ListOf(doc, "entities")) {
        const PropDecl* decl = root.Find("entities");
        for (const JsonValue& e : a->arr) {
            SceneEntity n;
            ElementReader r(scene::ElementChain(*decl, e), e, "entities", why, &ok);
            r.Str("name", n.p.name);
            r.Str("vessel", n.p.vessel);
            r.Enum("controller", n.p.controller);
            r.F64("throttle", n.p.throttle);
            r.F64("steer", n.p.steer);
            if (const JsonValue* at = e.Get("at")) {
                n.hasAt = true;
                n.at = *at;
            }
            if (!ok) return false;
            out.entities.push_back(std::move(n));
        }
    }
    if (const JsonValue* a = ListOf(doc, "interests")) {
        const PropDecl* decl = root.Find("interests");
        for (const JsonValue& e : a->arr) {
            SceneInterest n;
            ElementReader r(scene::ElementChain(*decl, e), e, "interests", why, &ok);
            r.Str("name", n.p.name);
            r.Str("target", n.p.target);
            r.F64("radius", n.p.radius);
            if (const JsonValue* at = e.Get("at")) {
                n.hasAt = true;
                n.at = *at;
            }
            if (!ok) return false;
            if (n.p.target.empty() && !n.hasAt) {
                return Refuse(why, "interests." + n.p.name + ": an interest follows a `target` or stands `at` a place");
            }
            out.interests.push_back(std::move(n));
        }
    }
    if (const JsonValue* a = ListOf(doc, "effects")) {
        const PropDecl* decl = root.Find("effects");
        for (const JsonValue& e : a->arr) {
            SceneEffect f;
            ElementReader r(scene::ElementChain(*decl, e), e, "effects", why, &ok);
            r.Str("name", f.p.name);
            r.Str("type", f.p.type);
            r.Bool("enabled", f.p.enabled);
            r.F64("d", f.d);   // slice.plane's, through the typed half of the chain
            if (!ok) return false;
            out.effects.push_back(std::move(f));
        }
    }
    if (const JsonValue* a = ListOf(doc, "layers")) {
        const PropDecl* decl = root.Find("layers");
        for (const JsonValue& e : a->arr) {
            SceneLayer l;
            ElementReader r(scene::ElementChain(*decl, e), e, "layers", why, &ok);
            r.Str("name", l.p.name);
            r.Bool("enabled", l.p.enabled);
            r.F32("exaggeration", l.exaggeration);   // the tide layer's
            r.F64("levelHeight", l.levelHeight);     // the buildings layer's three
            r.F64("defaultHeight", l.defaultHeight);
            r.F64("radius", l.radius);
            r.Str("lod", l.lod);
            r.F64("lodPixels", l.lodPixels);
            r.Bool("field", l.field);
            r.F64("laneWidth", l.laneWidth);         // the roads in the tree
            r.F64("defaultLanes", l.defaultLanes);
            r.F64("pathWidth", l.pathWidth);
            r.F64("kerb", l.kerb);
            if (!ok) return false;
            out.layers.push_back(std::move(l));
        }
    }
    if (const JsonValue* a = ListOf(doc, "tools")) {
        const PropDecl* decl = root.Find("tools");
        for (const JsonValue& e : a->arr) {
            SceneTool t;
            ElementReader r(scene::ElementChain(*decl, e), e, "tools", why, &ok);
            r.Str("name", t.name);
            r.Str("args", t.args);
            if (!ok) return false;
            out.tools.push_back(std::move(t));
        }
    }
    if (const JsonValue* a = ListOf(doc, "sources")) {
        const PropDecl* decl = root.Find("sources");
        for (const JsonValue& e : a->arr) {
            scene::SourceProps s;
            ElementReader r(scene::ElementChain(*decl, e), e, "sources", why, &ok);
            r.Str("name", s.name);
            r.Str("file", s.file);
            r.Str("folder", s.folder);
            r.Str("match", s.match);
            r.Str("manifest", s.manifest);
            r.Str("kind", s.kind);
            r.Str("crs", s.crs);
            r.F64("over", s.over);
            r.F64("feather", s.feather);
            r.Str("unit", s.unit);
            r.Str("datum", s.datum);
            r.F64("offset", s.offset);
            s.hasOffset = e.Get("offset") != nullptr;
            if (!ok) return false;
            if (s.file.empty() + s.folder.empty() + s.manifest.empty() != 2) {
                return Refuse(why, "sources: an entry names a file, a folder and a match, or a manifest -- one of the three");
            }
            out.sources.push_back(std::move(s));
        }
    }
    if (const JsonValue* a = ListOf(doc, "stations")) {
        const PropDecl* decl = root.Find("stations");
        for (const JsonValue& e : a->arr) {
            scene::StationProps s;
            ElementReader r(scene::ElementChain(*decl, e), e, "stations", why, &ok);
            r.Str("name", s.name);
            r.F64("lat", s.lat);
            r.F64("lon", s.lon);
            if (!ok) return false;
            out.stations.push_back(std::move(s));
        }
    }
    return true;
}

JsonValue WaterSceneDoc(const JsonValue& waterSection) {
    JsonValue o = scene::JsonObj();
    for (const char* k : {"wavefield", "closures", "fleet"}) {
        if (const JsonValue* v = waterSection.Get(k)) scene::JsonSet(o, k, *v);
    }
    return o;
}

void ToolArgs(const Scene& S, Options& o) {
    // Each tool that takes one: the field the legacy flag filled, filled from `--tool name:args`
    // when the flag did not. A tool named with neither leaves the field as it was.
    auto arg = [&](const char* name) -> std::string {
        const SceneTool* t = S.Tool(name);
        return t ? t->args : std::string();
    };
    auto text = [&](const char* name, std::string& f) {
        const std::string a = arg(name);
        if (!a.empty() && f.empty()) f = a;
    };
    auto wide = [&](const char* name, std::wstring& f) {
        const std::string a = arg(name);
        if (!a.empty() && f.empty()) f = Widen(a.c_str());
    };
    text("load-field", o.loadField);
    wide("water-map", o.waterMap);
    wide("bathy-map", o.bathyMap);
    text("gis-dump", o.gisDump);
    wide("fidelity-map", o.fidelityMap);
    text("ocean-probe", o.oceanProbe);
    wide("swe-uv", o.sweUvDump);
    wide("wave-map", o.waveMap);
    if (const SceneTool* t = S.Tool("tree-audit")) {
        if (!o.treeAudit && !t->args.empty()) {
            o.treeAudit = static_cast<uint32_t>(atoi(t->args.c_str()));
        }
    }
    // The audit's other two modes, as their flags set them (ParseArgs): warm every address the
    // incumbent holds, or pack what the trees hold. Named by `--tool` or a scene's tools[] alone,
    // the fields were never set, and RunTreeAudit would have audited instead.
    if (S.Tool("warm-trees")) {
        o.warmTrees = true;
        if (!o.treeAudit) o.treeAudit = 1000000u;
    }
    if (S.Tool("pack-trees")) {
        o.packTrees = true;
        if (!o.treeAudit) o.treeAudit = 1u;
    }
    if (const SceneTool* t = S.Tool("swe-cycle")) {
        if (o.sweCycleH <= 0.0 && !t->args.empty()) o.sweCycleH = atof(t->args.c_str());
    }
    if (const SceneTool* t = S.Tool("trace")) {
        if (!t->args.empty()) sscanf_s(t->args.c_str(), "%lf,%lf", &o.traceLat, &o.traceLon);
    }
    if (const SceneTool* t = S.Tool("export")) {
        if (o.exportSpec.empty() && !t->args.empty()) {
            const size_t comma = t->args.rfind(',');
            o.exportSpec = t->args.substr(0, comma);
            if (comma != std::string::npos) o.exportOut = Widen(t->args.c_str() + comma + 1);
        }
    }
}

bool CheckLayerOrder(const Scene& S, const std::vector<std::string>& registered,
                     std::string* why) {
    // THE ORDER IS DATA. The Assembly builds each layer where its data is ready (the lifetime
    // law), so what is checked is that the order it REGISTERED in is the declared order with the
    // layers this run could not build left out -- a subsequence. A repeat of the previous name is
    // a second instance of one declaration (the Droste outer bank's set B) and keeps its place.
    const std::vector<std::string> want = S.LayerOrder();
    size_t i = 0;
    std::string prev;
    for (const std::string& name : registered) {
        if (name == prev) continue;
        prev = name;
        if (!S.Layer(name.c_str())) continue;   // undeclared: not the list's business
        size_t j = i;
        while (j < want.size() && want[j] != name) ++j;
        if (j >= want.size()) {
            return Refuse(why, "layers: '" + name + "' registered out of the declared order (" +
                                   (i < want.size() ? want[i] : std::string("the list ends")) +
                                   " was next)");
        }
        i = j + 1;
    }
    return true;
}

}  // namespace ga::app
