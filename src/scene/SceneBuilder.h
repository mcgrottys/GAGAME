// ================================================================================================
//  SceneBuilder - M12 step 5a: RESOLUTION IS A FOLD WITH OVERRIDE.
//
//      struct defaults  <  the base file  <  each `include` overlay, in order  <  every --set,
//                                                                              in command-line order
//
//  The merge law, said once (MergeInto): objects merge by key; a Vec3, Motor, Similarity,
//  DualSphere or Color is a VALUE and is replaced whole (a versor is not a namespace: merging
//  {lat, lon, alt} into {x, alt, z, az, pitch} by key would breed a spelling that means
//  nothing); scalars replace; a named array (views, portals, entities, effects, layers,
//  nodes, tools) merges element by element -- an overlay entry with the same `name` merges in
//  place, `"remove": true` deletes it, an entry without a name (or with a new one) is appended;
//  an UNNAMED array (include, fleet.boats, rails.keys) is a VALUE and is replaced whole, so the
//  fold is idempotent and a resolved document reloads as itself (M12 step 5d: appending doubled
//  the fleet when a recipe file re-applied its own `include`); order is file order throughout.
//  A --set is `assoc`: `a.b.c=value` walks the path (an array segment names an element, or
//  indexes an unnamed one), creating what is missing, and merges the value by the same law,
//  so `views.sea.at={...}` replaces the eye whole and `portals.droste.twistDeg=45` edits one
//  key of one element. A value is JSON text; text that is not JSON is a string.
//
//  Resolve() then COMPLETES the document (every declared key present, list elements filled
//  from their element schema's prototype, a typed element from its type's schema too; keys
//  ordered comments-first then schema order; an included file's root comments stay with it) and
//  VALIDATES it against scene/SceneSchema.h: an unknown key, a wrong type, a unit refusal, an
//  unnamed element of a named list, a duplicate name, or an unknown node/effect/layer type is
//  refused with the NODE PATH in the message ("nodes.buoy.type: unknown component 'x'") --
//  the caller exits 2 with it. A file may name a `base` scene it INHERITS -- loaded first, so
//  the file's own keys override it (Godot's inherited scene, USD's `inherits`: weaker than the
//  local opinion, where `include` is stronger) -- which is how scenes/chart.json is three lines
//  over scenes/merrimack.json. Resolved() is the folded document: what --print-scene prints,
//  the memento a recipe is judged by; WriteJson is canonical (2-space indent, short objects
//  and scalar arrays inline, numbers shortest round-trip), so Parse(WriteJson(d)) writes back
//  byte for byte, which scenetest holds.
//
//  THE RELOAD LAW (for step 5f, stated here because the fold is its half): a reload resolves
//  a COMPLETE candidate scene from the stable defaults and the authored inputs, validates it,
//  and applies the accepted diff -- so a property removed from a file reverts to its declared
//  default, never to the previous runtime value; "the current value is the default" describes
//  the read of one file over a resolved base, not a memory.
//
//  Prior art, named: USD's layer stack (opinions compose strongest-last over a root layer;
//  a sublayer overlays; `over` edits in place, the prim path is the identity) and Godot's
//  inherited scenes (a derived .tscn overrides property by property and keeps the base's
//  children in order); "remove": true is USD's deactivation and Godot's node deletion in a
//  derived scene. The include here is an OVERLAY (stronger than the file that names it),
//  because that is what data/wave_scene.json has always been to the water section.
// ================================================================================================
#pragma once

#include "core/Json.h"
#include "scene/Props.h"

#include <string>
#include <vector>

namespace ga::scene {

class SceneBuilder {
public:
    // Starts from the struct defaults (SceneSchema.h DefaultDocument).
    SceneBuilder();

    // The file's `base` first (recursively), then the file, then its `include` list in order
    // (each overlay's own includes after it, depth first, a cycle refused). Paths are relative
    // to the working directory, as every data path in this engine is.
    bool Load(const std::string& path, std::string* why);
    // One overlay document at a section path ("" = the root).
    bool Overlay(const JsonValue& doc, const std::string& from, const std::string& at,
                 std::string* why);
    // --set a.b.c=value, as a path and a parsed value; SetText takes the assignment as typed.
    bool Set(const std::string& path, const JsonValue& value, std::string* why);
    bool SetText(const std::string& assignment, std::string* why);
    // Complete and validate. False = refused, `why` names the path.
    bool Resolve(std::string* why);

    const JsonValue& Resolved() const { return m_doc; }
    JsonValue& Document() { return m_doc; }
    const std::string& Base() const { return m_base; }
    // M12 step 5f: every file this fold actually READ, in the order it read them (the base
    // chain, then each include depth first) -- the chain the hot reload watches. An optional
    // include that was absent is not in it: there is nothing to watch.
    const std::vector<std::string>& Files() const { return m_files; }

    // The canonical text of a document.
    static std::string WriteJson(const JsonValue& v);
    static bool ReadFile(const std::string& path, std::string& text, std::string* why);
    // The value of a --set: JSON if it parses, else the text as a string.
    static JsonValue SetValue(const std::string& text);

private:
    bool ApplyIncludes(const JsonValue& doc, std::string* why);

    JsonValue m_doc;
    std::string m_base;
    std::vector<std::string> m_loading;   // the include chain, for the cycle guard
    std::vector<std::string> m_files;     // every file read, in read order (Files())
};

// ---- the pure laws, exported for the tests --------------------------------------------------

// The schemas a value at one path answers to: its own, plus its type's for a typed element.
struct SchemaChain {
    const Schema* a = nullptr;
    const Schema* b = nullptr;
    const PropDecl* Find(const std::string& key) const;
    const char* Type() const;
};

// THE MERGE LAW: overlay onto base, at `path`, under `chain` (may be empty for comment trees).
void MergeInto(JsonValue& base, const JsonValue& overlay, const SchemaChain& chain,
               const std::string& path);
// Every declared key present, elements completed, `remove` entries dropped, keys ordered
// (comments first, then the schema's order).
void Complete(JsonValue& doc, const SchemaChain& chain);
// The validation walk: false with `why` naming the path.
bool Validate(const JsonValue& doc, const SchemaChain& chain, const std::string& path,
              std::string* why);
// The schema chain of a typed list element (the element's own + its type's, if known).
SchemaChain ElementChain(const PropDecl& list, const JsonValue& element);

// The gate (src/scene/SceneTest.cpp), run from --selftest.
bool RunSceneSelfTest();

}  // namespace ga::scene
