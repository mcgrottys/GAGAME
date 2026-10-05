// ================================================================================================
//  SchemaDoc - M12 step 5f: THE TWO DATA CONTRACTS THE UI CONSUMES, emitted at boot beside
//  docs/ga_ast.json.
//
//  The plan's section F says the WPF/WinUI editor consumes three files and links no C++:
//  `scenes/*.json` with `docs/scene_schema.json` (what a scene file may say), `docs/ga_ast.json`
//  (the Blueprint contract, M7u) and `docs/registries.json` (what names exist, and what each
//  named type's properties are). It writes scene files and `--set` lines, launches
//  `gagame scene.json`, and checks itself against `--print-scene`.
//
//  WHY THEY ARE GENERATED AND NOT WRITTEN. Every fact in them already exists exactly once, in
//  the Schema tables (scene/SceneSchema.cpp, scene/Props.h) and the registries
//  (core/Registry.h). A hand-written contract is the drift Props.h's banner describes -- the
//  water scene's reader and writer listing the same keys twice and disagreeing about the
//  defaults. So these are a THIRD reader of the same table, written the way ga_ast.json is:
//  at boot, every boot, deterministic order (declaration order inside a type, std::map order
//  for a registry), LF (fopen "wb", as GaAst::WriteJson opens it -- git's autocrlf keeps the
//  checked-in blob LF too), and checked in, so a second boot with nothing changed is a
//  zero-line diff and a boot that DID change the tables shows exactly what moved.
//
//  WHAT A KEY CARRIES: its type, the live field's width, the quantity and unit it is declared in
//  (so an editor can offer "12 kn" and refuse a velocity into a length, as the engine does), the
//  DEFAULT an absent key means, the doc line, and Hot|Restart -- which is the one thing an
//  editor cannot infer and the whole reason a live scene editor is possible at all. The section
//  map says which node type each top-level section is sugar for, and the placement block names
//  the sugar's four spellings, because a placement is the one value whose file form is not its
//  type's field list.
//
//  Prior art, named: USD's schema registry (generatedSchema.usda / plugInfo.json describe the
//  typed schemas an application can author) and Godot's ClassDB + `PropertyInfo` (the inspector
//  is drawn from the registered property list, with hints and usage flags). Blender's RNA is the
//  same idea a third time. The engine's own contribution is the UNIT and the reload flag.
// ================================================================================================
#include "core/Common.h"
#include "scene/Props.h"
#include "scene/SceneBuilder.h"
#include "scene/SceneSchema.h"
#include "sim/VesselSpec.h"

#include <cstdio>
#include <map>
#include <set>
#include <string>
#include <vector>

namespace ga::scene {

namespace {

std::string Esc(const std::string& s) {
    std::string o;
    for (char c : s) {
        if (c == '"' || c == '\\') {
            o += '\\';
            o += c;
        } else if (c == '\n') {
            o += "\\n";
        } else {
            o += c;
        }
    }
    return o;
}

const char* TypeName(PropType t) {
    switch (t) {
        case PropType::Bool: return "bool";
        case PropType::Number: return "number";
        case PropType::String: return "string";
        case PropType::Enum: return "enum";
        case PropType::Vec3: return "vec3";
        case PropType::Motor: return "motor";
        case PropType::Similarity: return "similarity";
        case PropType::DualSphere: return "dualSphere";
        case PropType::Color: return "color";
        case PropType::Box: return "latLonBox";
        case PropType::LonLat: return "lonLat";
        case PropType::Path: return "path";
        case PropType::Object: return "object";
        case PropType::List: return "list";
    }
    return "?";
}

const char* FieldName(Field f) {
    switch (f) {
        case Field::None: return "none";
        case Field::Bool: return "bool";
        case Field::F32: return "f32";
        case Field::F64: return "f64";
        case Field::I32: return "i32";
        case Field::U32: return "u32";
        case Field::Str: return "string";
        case Field::D2: return "double2";
        case Field::D3: return "double3";
        case Field::D4: return "double4";
        case Field::F4: return "float4";
        case Field::Motor: return "motor";
        case Field::Placement: return "placement";
    }
    return "?";
}

// The canonical one-line text of a value (SceneBuilder's writer, trimmed).
std::string Line(const JsonValue& v) {
    std::string t = SceneBuilder::WriteJson(v);
    while (!t.empty() && (t.back() == '\n' || t.back() == '\r')) t.pop_back();
    return t;
}

// Every schema reachable from a root, in DECLARATION order, each once.
void Reach(const Schema& s, std::vector<const Schema*>& out, std::set<std::string>& seen) {
    if (!seen.insert(s.Type()).second) return;
    out.push_back(&s);
    for (const PropDecl& d : s.Decls()) {
        if (d.sub) Reach(*d.sub, out, seen);
    }
}

void WriteKeys(FILE* f, const Schema& s, const char* indent) {
    const auto& decls = s.Decls();
    for (size_t i = 0; i < decls.size(); ++i) {
        const PropDecl& d = decls[i];
        fprintf(f, "%s  { \"key\": \"%s\", \"type\": \"%s\", \"field\": \"%s\"", indent,
                Esc(d.key).c_str(), TypeName(d.type), FieldName(d.field));
        if (d.type == PropType::Object) {
            fprintf(f, ", \"schema\": \"%s\"", d.sub ? Esc(d.sub->Type()).c_str() : "");
        } else if (d.type == PropType::List) {
            fprintf(f, ", \"element\": \"%s\", \"named\": %s", d.sub ? Esc(d.sub->Type()).c_str() : "",
                    d.named ? "true" : "false");
            if (d.poly) {
                fprintf(f, ", \"registry\": \"%s\", \"typeKey\": \"%s\"", Esc(d.poly).c_str(),
                        Esc(d.polyKey ? d.polyKey : "type").c_str());
            }
        } else {
            fprintf(f, ", \"quantity\": \"%s\", \"unit\": \"%s\", \"default\": %s",
                    QuantityName(d.quantity), Esc(d.unit.raw).c_str(),
                    Line(s.DefaultOf(d)).c_str());
            if (d.type == PropType::Enum) {
                std::string names;
                for (const std::string& n : d.names) {
                    names += (names.empty() ? "" : ", ") + std::string("\"") + Esc(n) + "\"";
                }
                fprintf(f, ", \"names\": [%s]", names.c_str());
            }
        }
        if (d.optional) fprintf(f, ", \"optional\": true");
        fprintf(f, ", \"reload\": \"%s\", \"doc\": \"%s\" }%s\n",
                d.reload == Reload::Restart ? "restart" : "hot", Esc(d.doc).c_str(),
                i + 1 < decls.size() ? "," : "");
    }
}

// The section -> node type map: what each top-level section IS sugar for. The type a section's
// schema carries is its own name; this says what the ENGINE builds from it, which is the thing
// an editor's palette needs and the schema name alone does not say.
struct SectionNote {
    const char* key;
    const char* node;
    const char* note;
};
const SectionNote kSections[] = {
    {"base", "scene-file", "the scene this one inherits (loaded first; this file overrides it)"},
    {"scene", "scene", "the scene's identity: name, mode, planet, start view"},
    {"include", "include", "overlays applied over this file, in order (stronger than it)"},
    {"data", "data", "the data files the assembly opens"},
    {"time", "sim.clock", "the scene clock (sim/SimClock.h)"},
    {"sun", "solar.sun", "the sun: the ephemeris, or pinned at az/el"},
    {"sea", "sea", "the sea state and the MLLW -> NAVD88 datum link"},
    {"water", "water", "the water component (scene/WaterComponent.h) and the layers it fans out to"},
    {"streaming", "residency", "residency: scene state, because it changes the picture"},
    {"capture", "capture", "the headless capture and the still's hold"},
    {"views", "view", "the cameras (scene/View.h), by name; scene.view names the one that starts"},
    {"rails", "rail", "the rail flown (scene/Rail.h; scenes/rails/<active>.json)"},
    {"portals", "portal", "the Droste links (scene/Portal.h), by name"},
    {"entities", "entity", "the vessels (scene/Entity.h), by name"},
    {"effects", "effect", "the paper visuals (scene/Effect.h), by name, typed by EffectSchemas"},
    {"layers", "layer", "the layers in registration (draw) order, typed by name"},
    {"nodes", "component", "typed nodes (scene/Component.h), by name, typed by ComponentSchemas"},
    {"prune", "tool", "the tree-prune tool's keys (compose/TreePrune.h): list, or retire and purge on confirm"},
    {"tools", "tool", "the one-shot modes this scene runs (--tool)"},
    {"stations", "station", "where a station of the data stands when its file does not say (PHASE C3)"},
};

// The placement sugar: ONE parser, four spellings (scene/Props.h ResolvePlacement).
struct Spelling {
    const char* name;
    const char* keys;
    const char* doc;
};
const Spelling kSpellings[] = {
    {"compass", "\"x\", \"alt\", \"z\", \"az\", \"pitch\"",
     "the flat world frame in metres and compass degrees: Camera::SetFromCompass, then FromCamera"},
    {"orbit", "\"lat\", \"lon\", \"alt\", \"lookAt\"",
     "the orbit key: degrees and metres above the planet; lookAt aims at a second lat/lon"},
    {"motor", "\"motor.re\", \"motor.du\"",
     "the PGA motor outright (core/Pga.h), for a pose no sugar spells"},
    {"similarity", "\"similarity.p\", \"similarity.s\", \"similarity.axis\", "
                   "\"similarity.twistDeg\"",
     "the Cl(4,1) similarity (core/Space.h Placement), the Droste link's own spelling"},
};

}  // namespace

void Schema::WriteSchema(const char* path) {
    RegisterBuiltinSceneTypes();
    FILE* f = fopen(path, "wb");
    if (!f) {
        Log("[scene] cannot write %s", path);
        return;
    }
    std::vector<const Schema*> types;
    std::set<std::string> seen;
    Reach(SceneFileSchema(), types, seen);
    // The typed elements' schemas, which hang off the registries rather than off a Nest.
    for (Registry<const Schema*>* r : {&ComponentSchemas(), &EffectSchemas(), &LayerSchemas()}) {
        for (const std::string& n : r->Names()) {
            if (const Schema* s = r->Make(n)) Reach(*s, types, seen);
        }
    }

    fprintf(f, "{\n  \"version\": 1,\n");
    fprintf(f, "  \"_doc\": \"%s\",\n",
            Esc("the scene file's vocabulary, generated at boot from the Schema tables "
                "(src/scene/SceneSchema.cpp); reload hot|restart says what a running engine "
                "can take; --print-scene is the round trip")
                .c_str());
    fprintf(f, "  \"sections\": [\n");
    for (size_t i = 0; i < sizeof(kSections) / sizeof(kSections[0]); ++i) {
        const SectionNote& s = kSections[i];
        const PropDecl* d = SceneFileSchema().Find(s.key);
        fprintf(f, "    { \"key\": \"%s\", \"kind\": \"%s\", \"node\": \"%s\", \"doc\": \"%s\" }%s\n",
                Esc(s.key).c_str(), d ? TypeName(d->type) : "?", Esc(s.node).c_str(),
                Esc(s.note).c_str(), i + 1 < sizeof(kSections) / sizeof(kSections[0]) ? "," : "");
    }
    fprintf(f, "  ],\n  \"placement\": [\n");
    for (size_t i = 0; i < sizeof(kSpellings) / sizeof(kSpellings[0]); ++i) {
        const Spelling& s = kSpellings[i];
        fprintf(f, "    { \"spelling\": \"%s\", \"keys\": [%s], \"doc\": \"%s\" }%s\n",
                Esc(s.name).c_str(), s.keys, Esc(s.doc).c_str(),
                i + 1 < sizeof(kSpellings) / sizeof(kSpellings[0]) ? "," : "");
    }
    fprintf(f, "  ],\n  \"types\": [\n");
    for (size_t i = 0; i < types.size(); ++i) {
        fprintf(f, "    { \"type\": \"%s\", \"keys\": [\n", Esc(types[i]->Type()).c_str());
        WriteKeys(f, *types[i], "    ");
        fprintf(f, "    ] }%s\n", i + 1 < types.size() ? "," : "");
    }
    fprintf(f, "  ]\n}\n");
    fclose(f);
    Log("[scene] wrote %s: %zu types, %zu sections (the UI's scene contract)", path, types.size(),
        sizeof(kSections) / sizeof(kSections[0]));
}

void WriteRegistries(const char* path, const std::vector<RegistryDoc>& extra) {
    RegisterBuiltinSceneTypes();
    FILE* f = fopen(path, "wb");
    if (!f) {
        Log("[scene] cannot write %s", path);
        return;
    }
    struct Entry {
        std::string name, what;
        std::vector<std::string> names;
        Registry<const Schema*>* schemas = nullptr;
    };
    std::vector<Entry> all;
    all.push_back({"component", "a typed node in `nodes[]` (scene/Component.h)",
                   ComponentSchemas().Names(), &ComponentSchemas()});
    all.push_back({"effect", "a paper visual in `effects[]` (scene/Effect.h)",
                   EffectSchemas().Names(), &EffectSchemas()});
    all.push_back({"layer", "a draw layer in `layers[]`, in registration order (scene/Layer.h)",
                   LayerSchemas().Names(), &LayerSchemas()});
    // THE HULLS: registered exactly as the engine registers them, and each kind's property table
    // is its LEDGER -- read-only from a scene (entities[] chooses a kind by name; its numbers are
    // the spec's), which is why it is written as a ledger and not as settable keys.
    VesselRegistry vreg;
    RegisterBuiltinVessels(vreg);
    all.push_back({"vessel", "a hull kind in `entities[].vessel` (sim/VesselSpec.h)", vreg.Kinds(),
                   nullptr});
    size_t vesselAt = all.size() - 1;
    for (const RegistryDoc& r : extra) all.push_back({r.name, r.what, r.names, nullptr});

    fprintf(f, "{\n  \"version\": 1,\n");
    fprintf(f, "  \"_doc\": \"%s\",\n",
            Esc("every registry's names, with the property schema of each type that has one; "
                "generated at boot beside docs/scene_schema.json (src/scene/SchemaDoc.cpp)")
                .c_str());
    fprintf(f, "  \"registries\": [\n");
    for (size_t i = 0; i < all.size(); ++i) {
        const Entry& e = all[i];
        fprintf(f, "    { \"registry\": \"%s\", \"selects\": \"%s\", \"names\": [", Esc(e.name).c_str(),
                Esc(e.what).c_str());
        for (size_t k = 0; k < e.names.size(); ++k) {
            fprintf(f, "%s\"%s\"", k ? ", " : "", Esc(e.names[k]).c_str());
        }
        fprintf(f, "]");
        if (e.schemas) {
            fprintf(f, ", \"types\": [\n");
            for (size_t k = 0; k < e.names.size(); ++k) {
                const Schema* s = e.schemas->Make(e.names[k]);
                fprintf(f, "      { \"name\": \"%s\", \"type\": \"%s\", \"keys\": [\n",
                        Esc(e.names[k]).c_str(), s ? Esc(s->Type()).c_str() : "");
                if (s) WriteKeys(f, *s, "      ");
                fprintf(f, "      ] }%s\n", k + 1 < e.names.size() ? "," : "");
            }
            fprintf(f, "    ]");
        } else if (i == vesselAt) {
            fprintf(f, ", \"types\": [\n");
            for (size_t k = 0; k < e.names.size(); ++k) {
                const VesselSpec spec = vreg.Build(e.names[k]);
                fprintf(f, "      { \"name\": \"%s\", \"display\": \"%s\", \"ledger\": [\n",
                        Esc(spec.kind).c_str(), Esc(spec.display).c_str());
                const auto ledger = spec.Ledger();
                for (size_t n = 0; n < ledger.size(); ++n) {
                    const Num& num = *ledger[n].second;
                    fprintf(f,
                            "        { \"key\": \"%s\", \"quantity\": \"%s\", \"value\": %s, "
                            "\"raw\": %s, \"unit\": \"%s\", \"src\": \"%s\" }%s\n",
                            Esc(ledger[n].first).c_str(), QuantityName(num.quantity),
                            Line(JsonNum(num.v)).c_str(), Line(JsonNum(num.raw)).c_str(),
                            Esc(num.unit).c_str(), Esc(num.src).c_str(),
                            n + 1 < ledger.size() ? "," : "");
                }
                fprintf(f, "      ] }%s\n", k + 1 < e.names.size() ? "," : "");
            }
            fprintf(f, "    ]");
        }
        fprintf(f, " }%s\n", i + 1 < all.size() ? "," : "");
    }
    fprintf(f, "  ]\n}\n");
    fclose(f);
    Log("[scene] wrote %s: %zu registries (the UI's name contract)", path, all.size());
}

}  // namespace ga::scene
