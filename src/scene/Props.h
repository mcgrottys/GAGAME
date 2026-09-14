// ================================================================================================
//  Props - M12 step 5a: ONE TABLE PER COMPONENT TYPE, and everything a property needs falls
//  out of it -- reading, writing (--print-scene), the unit refusal, hot-reload diffing.
//
//  THE IDIOM STAYS SceneConfig.h's: "the current value is the default". LoadWaterScene reads
//  every key as `out.x = w->Num("x", out.x)` -- the live field is the fallback, a file is a
//  set of overrides -- and that is the right law for a scene. What it lacked was ONE place
//  that knows the keys: the reader listed them, the writer (WriteDefaultWaterScene) listed
//  them again by hand with its own defaults (comps 16 in the text, 32 in the struct -- the
//  drift the banner there admits), the hot-reload compared nothing and re-applied everything,
//  and no unit was ever checked because no key knew its unit. So a Schema binds each key ONCE
//  to a live field (an offset from a prototype instance, so the table is one per TYPE -- a
//  Flyweight -- while the values are per instance in a PropSet), with its type, its quantity
//  and unit (core/GaUnits.h: a Velocity into a Length refuses with AcceptsFrom's own why; a
//  datum mismatch refuses as the compositor's normalization does), its doc line and whether a
//  change is Hot or needs a Restart. Defaults() reads the fields; Merge() reads a JSON object
//  and refuses an unknown key (except "_"-prefixed comments, data/wave_scene.json's convention);
//  ApplyTo() writes the fields; ToJson() writes the values back; Diff() names what changed and
//  whether the change needs a restart. Four readers of one table cannot drift from each other.
//
//  THE NUMBER LAW. A Number is written as a bare value IN THE DECLARED UNIT, or as "12 kn" (a
//  value and a unit, converted through UnitSpec -- no second parser), or as VesselSpec.h's
//  {v, unit, src} object, whose src is provenance kept for the report. The value is STORED in
//  the declared unit, because that is the unit the live field is in (degrees for a compass
//  azimuth, hours for a spin-up), and the conversion is UnitSpec's canonical factor both ways.
//  Legacy float fields print with the shortest text that narrows back to the same float
//  (never more than %.9g), so a double scene reproduces the legacy value bit for bit; doubles
//  print with the shortest text that round-trips (Steele & White / Grisu's contract, met here
//  by trying %.15g, %.16g, %.17g in turn).
//
//  THE TYPES: Bool, Number, String, Enum (a name from a declared list, stored as its index),
//  Vec3, Motor (Pga.h: a rigid pose), Similarity (Space.h's Placement: a rigid pose or the
//  Droste link), DualSphere (a CGA sphere {c, r}), Color, Path, and the two structural ones
//  a scene file needs -- Object (a nested section with its own schema) and List (a named array
//  of one element type, or of types chosen by a `type`/`name` key through a schema registry).
//  A Motor or a Similarity is written in the placement SUGAR -- one parser, four spellings:
//  {x, alt, z, az, pitch} (SetFromCompass then FromCamera), {lat, lon, alt[, lookAt]} (the
//  orbit key), {motor: {re, du}} and {similarity: {p, s, axis, twistDeg}} -- validated for
//  shape when read and resolved to a Placement where it is consumed (ResolvePlacement, which
//  needs the tangent frame), never cached into a matrix.
//
//  Prior art, named: USD's property schemas (typed attributes with metadata, the schema one
//  per prim type, values per prim) and Godot's exported properties (the editor reads one
//  table for the inspector, the file format and the defaults). Neither carries units; this
//  table does, because this engine's failures have been unit failures (GaUnits.h's banner).
// ================================================================================================
#pragma once

#include "core/GaUnits.h"
#include "core/Json.h"
#include "core/Pga.h"
#include "core/Space.h"

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace ga::scene {

enum class PropType : uint8_t {
    Bool, Number, String, Enum, Vec3, Motor, Similarity, DualSphere, Color, Path, Object, List
};
enum class Reload : uint8_t { Hot, Restart };
// What the bound field IS, so ApplyTo writes the right width and ToJson prints by the number
// law (F32 fields through FloatText).
enum class Field : uint8_t { None, Bool, F32, F64, I32, U32, Str, D3, D4, F4, Motor, Placement };

class Schema;

struct PropDecl {
    std::string key;
    PropType type = PropType::Bool;
    Field field = Field::None;
    std::ptrdiff_t offset = 0;          // from the instance base (the prototype's layout)
    Quantity quantity = Quantity::Unknown;
    UnitSpec unit;                      // the declared unit; the value is stored in it
    std::string doc;
    Reload reload = Reload::Hot;
    std::vector<std::string> names;     // Enum: index -> name
    const Schema* sub = nullptr;        // Object: its schema. List: the element schema (may be null)
    const char* poly = nullptr;         // List: the schema registry that types an element ("component",
                                        // "effect", "layer") beside `sub`, or null
    const char* polyKey = "type";       // List: which element key names the type ("type" | "name")
    bool named = true;                  // List: elements carry `name` and merge by it
    bool optional = false;              // absent means "the engine decides": not completed, not printed
};

struct PropValue {
    bool set = false;
    bool b = false;
    double n = 0.0;                     // Number, in the declared unit
    int e = 0;                          // Enum index
    std::string s;                      // String / Path / the Enum's name
    double v[4] = {0.0, 0.0, 0.0, 0.0}; // Vec3 / DualSphere {cx, cy, cz, r} / Color
    Motor m;                            // Motor, when resolved
    Placement p;                        // Similarity, when resolved
    JsonValue raw;                      // Motor / Similarity: the sugar as written
    std::string src;                    // Number provenance ({v, unit, src})
};

// The frame a placement's lat/lon spellings resolve in: the planet's radius and the tangent
// rows the session derives from the anchor (compose/SurfaceFrame.h east/up/north).
struct PoseFrame {
    double planetR = 0.0;
    double east[3] = {1.0, 0.0, 0.0};
    double up[3] = {0.0, 1.0, 0.0};
    double north[3] = {0.0, 0.0, 1.0};
    bool valid = false;
};

// ================================================================================================
//  Schema - the table, one per type. Built ONCE over a prototype instance (the offsets are what
//  bind a key to a field; the prototype's values are the defaults ToJson prints for an absent
//  key), then read by every PropSet of that type.
// ================================================================================================
class Schema {
public:
    Schema(const char* type, const void* prototype);
    Schema(const Schema&) = delete;
    Schema& operator=(const Schema&) = delete;

    const char* Type() const { return m_type; }
    const void* Prototype() const { return m_base; }

    // Bind(key, field&, [Quantity, unit,] doc, Hot|Restart): the key names the field once.
    Schema& Bind(const char* key, bool& f, const char* doc, Reload r = Reload::Hot);
    Schema& Bind(const char* key, double& f, Quantity q, const char* unit, const char* doc,
                 Reload r = Reload::Hot);
    Schema& Bind(const char* key, float& f, Quantity q, const char* unit, const char* doc,
                 Reload r = Reload::Hot);
    Schema& Bind(const char* key, int& f, Quantity q, const char* unit, const char* doc,
                 Reload r = Reload::Hot);
    Schema& Bind(const char* key, uint32_t& f, Quantity q, const char* unit, const char* doc,
                 Reload r = Reload::Hot);
    Schema& Bind(const char* key, std::string& f, const char* doc, Reload r = Reload::Hot);
    Schema& BindPath(const char* key, std::string& f, const char* doc, Reload r = Reload::Hot);
    Schema& BindEnum(const char* key, int& f, std::vector<std::string> names, const char* doc,
                     Reload r = Reload::Hot);
    Schema& Bind(const char* key, double (&f)[3], Quantity q, const char* unit, const char* doc,
                 Reload r = Reload::Hot);
    Schema& Bind(const char* key, Motor& f, const char* doc, Reload r = Reload::Hot);
    Schema& Bind(const char* key, Placement& f, const char* doc, Reload r = Reload::Hot);
    Schema& BindSphere(const char* key, double (&f)[4], Quantity q, const char* unit,
                       const char* doc, Reload r = Reload::Hot);
    Schema& BindColor(const char* key, float (&f)[4], const char* doc, Reload r = Reload::Hot);
    // A nested section: `sub` was built over `subPrototype`, which lies inside this prototype.
    Schema& Nest(const char* key, const Schema& sub, const void* subPrototype, const char* doc);
    // A named array of `element`s (null element = typed only by the registry `poly` names).
    Schema& List(const char* key, const Schema* element, const char* doc, bool named = true,
                 const char* poly = nullptr, const char* polyKey = "type");
    // Marks the last-declared key optional (absent = the engine decides; see PropDecl).
    Schema& Optional();

    const PropDecl* Find(const std::string& key) const;
    const std::vector<PropDecl>& Decls() const { return m_decls; }
    // The prototype's value of one key, as JSON -- the default an absent key is completed with.
    JsonValue DefaultOf(const PropDecl& d) const;

    static bool IsComment(const std::string& key) { return !key.empty() && key[0] == '_'; }
    // Replaced whole by an overlay (a versor or a colour is a value, not a namespace).
    static bool Atomic(PropType t) {
        return t == PropType::Vec3 || t == PropType::Motor || t == PropType::Similarity ||
               t == PropType::DualSphere || t == PropType::Color;
    }

private:
    PropDecl& Add(const char* key, PropType type, Field field, const void* fieldAddr,
                  const char* doc, Reload r);
    void Unit(PropDecl& d, Quantity q, const char* unit);

    const char* m_type;
    const char* m_base;
    std::vector<PropDecl> m_decls;
};

// ================================================================================================
//  PropSet - the values of one instance, indexed by the schema's declarations.
// ================================================================================================
class PropSet {
public:
    explicit PropSet(const Schema& s);
    PropSet(const PropSet& o);
    PropSet& operator=(const PropSet& o);

    // THE CURRENT VALUE IS THE DEFAULT: every bound field read from `instance`.
    static PropSet Defaults(const Schema& s, const void* instance);
    // One JSON object over this set. Unknown keys refuse (comments excepted), a wrong type or
    // unit refuses with the why, naming `path`.`key`. Nested Objects recurse; Lists are the
    // document's business (SceneBuilder) and are skipped here.
    bool Merge(const JsonValue& obj, const std::string& path, std::string* why);
    // Write every set value into the instance's fields (a Motor/Similarity through the frame).
    bool ApplyTo(void* instance, const PoseFrame* frame, std::string* why) const;
    // The values as one object, every declared key present (Lists as empty arrays; an unset
    // optional key omitted).
    JsonValue ToJson() const;

    struct Change {
        std::string key;
        Reload reload;
    };
    // The keys whose values differ from `before` (nested keys as "sub.key").
    std::vector<Change> Diff(const PropSet& before) const;

    const PropValue* Get(const char* key) const;
    const PropSet* Object(const char* key) const;
    const Schema& Type() const { return *m_schema; }

private:
    const Schema* m_schema;
    std::vector<PropValue> m_values;                 // parallel to Decls()
    std::vector<std::unique_ptr<PropSet>> m_objects; // parallel to Decls(); set for Object decls
};

// ---- the one parser for a declared property, and its helpers ----------------------------------

// JSON -> value by the declaration: the number law, the enum names, the sugar shapes.
bool ParseProp(const PropDecl& d, const JsonValue& v, const std::string& path, PropValue& out,
               std::string* why);
// A number in `declared`'s unit from a bare number, "12 kn" or {v, unit, src}.
bool NumberOf(const JsonValue& v, const UnitSpec& declared, const std::string& path, double& out,
              std::string* src, std::string* why);
// value -> JSON by the declaration (the inverse of ParseProp).
JsonValue PropToJson(const PropDecl& d, const PropValue& v);
bool PropEqual(const PropDecl& d, const PropValue& a, const PropValue& b);

// The placement sugar's shape, checked without a frame (what Merge does); `rigid` refuses the
// similarity spelling (a Motor is a rigid pose).
bool CheckPlacementSugar(const JsonValue& sugar, bool rigid, const std::string& path,
                         std::string* why);
// The placement sugar, resolved in a frame: ONE parser for the four spellings.
bool ResolvePlacement(const JsonValue& sugar, const PoseFrame& frame, Placement& out,
                      std::string* why);

// M12 step 5d: THE SUGAR'S NUMBERS, WITHOUT A FRAME. A scene's eye is a placement, and resolving
// one needs the tangent rows the boot only has after the surface is built -- but the session's
// own calls do not want a Placement, they want the numbers the flags used to carry
// (Camera::SetFromCompass's five, scene::GlobeCamera's three). This is the SAME read
// ResolvePlacement does, stopping one step earlier: one parser, the declared units, the same
// refusals naming `path`. The kinds are the four spellings, in the order the sugar declares them.
struct PoseSugar {
    enum class Kind { Compass, Orbit, MotorForm, SimilarityForm };
    Kind kind = Kind::Compass;
    double x = 0.0, alt = 0.0, z = 0.0, az = 90.0, pitch = 0.0;   // {x, alt, z, az, pitch}
    double lat = 0.0, lon = 0.0, tLat = 0.0, tLon = 0.0;          // {lat, lon, alt[, lookAt]}
    bool lookAt = false;
};
bool ReadPoseSugar(const JsonValue& sugar, const std::string& path, PoseSugar& out,
                   std::string* why);

// M12 step 5e: THE SUGAR AS THE MOTOR the session's pose maps make -- scene::FromCamera of
// the camera the rigid spelling builds (ResolveMotor, the step before Placement::Rigid). A
// rail key carries THIS motor: the session's tables were poseMotor(camera), and the round
// trip through Rigid and MotorOf is not the identity on a motor that is not unit (5b's
// finding), so a key resolved through a Placement would not be the key the table held.
bool ResolveMotorSugar(const JsonValue& sugar, const PoseFrame& frame, const std::string& path,
                       Motor& out, std::string* why);

// The number law's formatting.
std::string NumberText(double v);   // the shortest text that round-trips the double
std::string FloatText(float v);     // the shortest text that narrows back to the float (<= %.9g)
double FloatAsDouble(float v);      // strtod(FloatText(v)): the double a float scene value IS

// JSON built in code (the shim, the defaults, the tests): core/Json.h is a reader only.
JsonValue JsonNum(double v);
JsonValue JsonStr(const std::string& s);
JsonValue JsonBool(bool b);
JsonValue JsonObj();
JsonValue JsonArr();
JsonValue* JsonSet(JsonValue& obj, const std::string& key, JsonValue value);   // replace or append
JsonValue* JsonGet(JsonValue& obj, const std::string& key);
bool JsonEqual(const JsonValue& a, const JsonValue& b);   // structural, key order included

}  // namespace ga::scene
