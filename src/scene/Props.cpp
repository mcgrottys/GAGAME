// Props - the table's bodies (M12 step 5a). See Props.h.
#include "scene/Props.h"

#include "core/Common.h"
#include "render/Camera.h"
#include "scene/Pose.h"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <stdexcept>

namespace ga::scene {

// ---- the number law's formatting ---------------------------------------------------------------

std::string NumberText(double v) {
    if (std::isnan(v) || std::isinf(v)) return "0";   // never written by this engine; refused upstream
    if (v == 0.0) return "0";                          // -0 prints as 0
    char buf[64];
    for (int prec = 15; prec <= 17; ++prec) {
        snprintf(buf, sizeof(buf), "%.*g", prec, v);
        if (strtod(buf, nullptr) == v) break;
    }
    return buf;
}

std::string FloatText(float v) {
    if (std::isnan(v) || std::isinf(v)) return "0";
    if (v == 0.0f) return "0";
    char buf[64];
    for (int prec = 6; prec <= 9; ++prec) {
        snprintf(buf, sizeof(buf), "%.*g", prec, static_cast<double>(v));
        if (strtof(buf, nullptr) == v) break;
    }
    return buf;
}

double FloatAsDouble(float v) { return strtod(FloatText(v).c_str(), nullptr); }

// ---- JSON helpers -----------------------------------------------------------------------------

JsonValue JsonNum(double v) {
    JsonValue j;
    j.type = JsonValue::Type::Number;
    j.number = v;
    return j;
}
JsonValue JsonStr(const std::string& s) {
    JsonValue j;
    j.type = JsonValue::Type::String;
    j.str = s;
    return j;
}
JsonValue JsonBool(bool b) {
    JsonValue j;
    j.type = JsonValue::Type::Bool;
    j.boolean = b;
    return j;
}
JsonValue JsonObj() {
    JsonValue j;
    j.type = JsonValue::Type::Object;
    return j;
}
JsonValue JsonArr() {
    JsonValue j;
    j.type = JsonValue::Type::Array;
    return j;
}
JsonValue* JsonSet(JsonValue& obj, const std::string& key, JsonValue value) {
    for (auto& kv : obj.obj) {
        if (kv.first == key) {
            kv.second = std::move(value);
            return &kv.second;
        }
    }
    obj.obj.emplace_back(key, std::move(value));
    return &obj.obj.back().second;
}
JsonValue* JsonGet(JsonValue& obj, const std::string& key) {
    if (obj.type != JsonValue::Type::Object) return nullptr;
    for (auto& kv : obj.obj) {
        if (kv.first == key) return &kv.second;
    }
    return nullptr;
}
bool JsonEqual(const JsonValue& a, const JsonValue& b) {
    if (a.type != b.type) return false;
    switch (a.type) {
        case JsonValue::Type::Null: return true;
        case JsonValue::Type::Bool: return a.boolean == b.boolean;
        case JsonValue::Type::Number: return a.number == b.number;
        case JsonValue::Type::String: return a.str == b.str;
        case JsonValue::Type::Array:
            if (a.arr.size() != b.arr.size()) return false;
            for (size_t i = 0; i < a.arr.size(); ++i) {
                if (!JsonEqual(a.arr[i], b.arr[i])) return false;
            }
            return true;
        case JsonValue::Type::Object:
            if (a.obj.size() != b.obj.size()) return false;
            for (size_t i = 0; i < a.obj.size(); ++i) {
                if (a.obj[i].first != b.obj[i].first) return false;
                if (!JsonEqual(a.obj[i].second, b.obj[i].second)) return false;
            }
            return true;
    }
    return false;
}

static JsonValue NumArray(const double* v, int n) {
    JsonValue a = JsonArr();
    for (int i = 0; i < n; ++i) a.arr.push_back(JsonNum(v[i]));
    return a;
}

static bool Refuse(std::string* why, const std::string& text) {
    if (why) *why = text;
    return false;
}

// ---- Schema -----------------------------------------------------------------------------------

Schema::Schema(const char* type, const void* prototype)
    : m_type(type), m_base(static_cast<const char*>(prototype)) {}

PropDecl& Schema::Add(const char* key, PropType type, Field field, const void* fieldAddr,
                      const char* doc, Reload r) {
    for (const PropDecl& d : m_decls) {
        if (d.key == key) {
            throw std::logic_error(std::string("Schema ") + m_type + ": key '" + key +
                                   "' bound twice");
        }
    }
    PropDecl d;
    d.key = key;
    d.type = type;
    d.field = field;
    d.offset = fieldAddr ? static_cast<const char*>(fieldAddr) - m_base : 0;
    d.doc = doc ? doc : "";
    d.reload = r;
    m_decls.push_back(std::move(d));
    return m_decls.back();
}

void Schema::Unit(PropDecl& d, Quantity q, const char* unit) {
    d.quantity = q;
    if (unit && *unit) {
        d.unit = UnitSpec::Parse(unit);
        if (d.unit.quantity != q) {
            throw std::logic_error(std::string("Schema ") + m_type + "." + d.key + ": unit '" +
                                   unit + "' is not a " + QuantityName(q));
        }
    } else {
        d.unit = UnitSpec::Of(q);
    }
}

Schema& Schema::Bind(const char* key, bool& f, const char* doc, Reload r) {
    Add(key, PropType::Bool, Field::Bool, &f, doc, r);
    return *this;
}
Schema& Schema::Bind(const char* key, double& f, Quantity q, const char* unit, const char* doc,
                     Reload r) {
    Unit(Add(key, PropType::Number, Field::F64, &f, doc, r), q, unit);
    return *this;
}
Schema& Schema::Bind(const char* key, float& f, Quantity q, const char* unit, const char* doc,
                     Reload r) {
    Unit(Add(key, PropType::Number, Field::F32, &f, doc, r), q, unit);
    return *this;
}
Schema& Schema::Bind(const char* key, int& f, Quantity q, const char* unit, const char* doc,
                     Reload r) {
    Unit(Add(key, PropType::Number, Field::I32, &f, doc, r), q, unit);
    return *this;
}
Schema& Schema::Bind(const char* key, uint32_t& f, Quantity q, const char* unit, const char* doc,
                     Reload r) {
    Unit(Add(key, PropType::Number, Field::U32, &f, doc, r), q, unit);
    return *this;
}
Schema& Schema::Bind(const char* key, std::string& f, const char* doc, Reload r) {
    Add(key, PropType::String, Field::Str, &f, doc, r);
    return *this;
}
Schema& Schema::BindPath(const char* key, std::string& f, const char* doc, Reload r) {
    Add(key, PropType::Path, Field::Str, &f, doc, r);
    return *this;
}
Schema& Schema::BindEnum(const char* key, int& f, std::vector<std::string> names,
                         const char* doc, Reload r) {
    Add(key, PropType::Enum, Field::I32, &f, doc, r).names = std::move(names);
    return *this;
}
Schema& Schema::Bind(const char* key, double (&f)[3], Quantity q, const char* unit,
                     const char* doc, Reload r) {
    Unit(Add(key, PropType::Vec3, Field::D3, &f, doc, r), q, unit);
    return *this;
}
Schema& Schema::Bind(const char* key, Motor& f, const char* doc, Reload r) {
    Add(key, PropType::Motor, Field::Motor, &f, doc, r);
    return *this;
}
Schema& Schema::Bind(const char* key, Placement& f, const char* doc, Reload r) {
    Add(key, PropType::Similarity, Field::Placement, &f, doc, r);
    return *this;
}
Schema& Schema::BindSphere(const char* key, double (&f)[4], Quantity q, const char* unit,
                           const char* doc, Reload r) {
    Unit(Add(key, PropType::DualSphere, Field::D4, &f, doc, r), q, unit);
    return *this;
}
Schema& Schema::BindColor(const char* key, float (&f)[4], const char* doc, Reload r) {
    Add(key, PropType::Color, Field::F4, &f, doc, r);
    return *this;
}
Schema& Schema::Nest(const char* key, const Schema& sub, const void* subPrototype,
                     const char* doc) {
    Add(key, PropType::Object, Field::None, subPrototype, doc, Reload::Hot).sub = &sub;
    return *this;
}
Schema& Schema::List(const char* key, const Schema* element, const char* doc, bool named,
                     const char* poly, const char* polyKey) {
    PropDecl& d = Add(key, PropType::List, Field::None, nullptr, doc, Reload::Hot);
    d.sub = element;
    d.named = named;
    d.poly = poly;
    d.polyKey = polyKey;
    return *this;
}
Schema& Schema::Optional() {
    if (!m_decls.empty()) m_decls.back().optional = true;
    return *this;
}

const PropDecl* Schema::Find(const std::string& key) const {
    for (const PropDecl& d : m_decls) {
        if (d.key == key) return &d;
    }
    return nullptr;
}

JsonValue Schema::DefaultOf(const PropDecl& d) const {
    if (d.type == PropType::List) return JsonArr();
    if (d.type == PropType::Object) {
        return PropSet::Defaults(*d.sub, m_base + d.offset).ToJson();
    }
    PropSet ps = PropSet::Defaults(*this, m_base);
    const PropValue* v = ps.Get(d.key.c_str());
    return (v && v->set) ? PropToJson(d, *v) : JsonValue{};
}

// ---- the sugar --------------------------------------------------------------------------------

namespace {

const UnitSpec& kMetres() { static const UnitSpec u = UnitSpec::Parse("m"); return u; }
const UnitSpec& kDegrees() { static const UnitSpec u = UnitSpec::Parse("deg"); return u; }
const UnitSpec& kOne() { static const UnitSpec u = UnitSpec::Parse("1"); return u; }

bool NumField(const JsonValue& o, const char* key, const UnitSpec& unit, const std::string& path,
              double& out, std::string* why, bool required = true) {
    const JsonValue* v = o.Get(key);
    if (!v) {
        if (required) return Refuse(why, path + ": placement needs '" + key + "'");
        return true;
    }
    return NumberOf(*v, unit, path + "." + key, out, nullptr, why);
}

bool ReadNumArray(const JsonValue& o, const char* key, const UnitSpec& unit, int n,
                  const std::string& path, double* out, std::string* why) {
    const JsonValue* v = o.Get(key);
    if (!v) return Refuse(why, path + ": placement needs '" + key + "'");
    if (v->type != JsonValue::Type::Array || static_cast<int>(v->arr.size()) != n) {
        return Refuse(why, path + "." + key + ": expected an array of " + std::to_string(n));
    }
    for (int i = 0; i < n; ++i) {
        if (!NumberOf(v->arr[static_cast<size_t>(i)], unit,
                      path + "." + key + "[" + std::to_string(i) + "]", out[i], nullptr, why)) {
            return false;
        }
    }
    return true;
}

bool OnlyKeys(const JsonValue& o, std::initializer_list<const char*> allowed,
              const std::string& path, std::string* why) {
    for (const auto& kv : o.obj) {
        if (Schema::IsComment(kv.first)) continue;
        bool ok = false;
        for (const char* a : allowed) ok |= (kv.first == a);
        if (!ok) return Refuse(why, path + "." + kv.first + ": unknown key in a placement");
    }
    return true;
}

enum class Sugar { Compass, Orbit, MotorForm, SimilarityForm, Bad };

Sugar Spelling(const JsonValue& v) {
    if (v.type != JsonValue::Type::Object) return Sugar::Bad;
    if (v.Get("motor")) return Sugar::MotorForm;
    if (v.Get("similarity")) return Sugar::SimilarityForm;
    if (v.Get("lat")) return Sugar::Orbit;
    if (v.Get("x")) return Sugar::Compass;
    return Sugar::Bad;
}

// The four spellings read into their numbers (shape + units), with or without a frame.
struct SugarValues {
    Sugar kind = Sugar::Bad;
    double x = 0, alt = 0, z = 0, az = 90.0, pitch = 0.0;   // compass (az/pitch default: yaw 0, level)
    double lat = 0, lon = 0, tLat = 0, tLon = 0;            // orbit
    bool lookAt = false;
    double re[4] = {1, 0, 0, 0}, du[4] = {0, 0, 0, 0};     // motor
    double p[3] = {0, 0, 0}, s = 1.0, axis[3] = {0, 0, 1}, twistDeg = 0.0;   // similarity
};

bool ReadSugar(const JsonValue& v, const std::string& path, SugarValues& o, std::string* why) {
    o.kind = Spelling(v);
    switch (o.kind) {
        case Sugar::Compass:
            if (!OnlyKeys(v, {"x", "alt", "z", "az", "pitch"}, path, why)) return false;
            return NumField(v, "x", kMetres(), path, o.x, why) &&
                   NumField(v, "alt", kMetres(), path, o.alt, why) &&
                   NumField(v, "z", kMetres(), path, o.z, why) &&
                   NumField(v, "az", kDegrees(), path, o.az, why, false) &&
                   NumField(v, "pitch", kDegrees(), path, o.pitch, why, false);
        case Sugar::Orbit: {
            if (!OnlyKeys(v, {"lat", "lon", "alt", "lookAt"}, path, why)) return false;
            if (!NumField(v, "lat", kDegrees(), path, o.lat, why) ||
                !NumField(v, "lon", kDegrees(), path, o.lon, why) ||
                !NumField(v, "alt", kMetres(), path, o.alt, why)) {
                return false;
            }
            if (const JsonValue* t = v.Get("lookAt")) {
                if (t->type != JsonValue::Type::Object) {
                    return Refuse(why, path + ".lookAt: expected {lat, lon}");
                }
                if (!OnlyKeys(*t, {"lat", "lon"}, path + ".lookAt", why)) return false;
                o.lookAt = true;
                return NumField(*t, "lat", kDegrees(), path + ".lookAt", o.tLat, why) &&
                       NumField(*t, "lon", kDegrees(), path + ".lookAt", o.tLon, why);
            }
            return true;
        }
        case Sugar::MotorForm: {
            if (!OnlyKeys(v, {"motor"}, path, why)) return false;
            const JsonValue& m = *v.Get("motor");
            if (m.type != JsonValue::Type::Object) return Refuse(why, path + ".motor: expected {re, du}");
            if (!OnlyKeys(m, {"re", "du"}, path + ".motor", why)) return false;
            return ReadNumArray(m, "re", kOne(), 4, path + ".motor", o.re, why) &&
                   ReadNumArray(m, "du", kOne(), 4, path + ".motor", o.du, why);
        }
        case Sugar::SimilarityForm: {
            if (!OnlyKeys(v, {"similarity"}, path, why)) return false;
            const JsonValue& s = *v.Get("similarity");
            if (s.type != JsonValue::Type::Object) {
                return Refuse(why, path + ".similarity: expected {p, s, axis, twistDeg}");
            }
            if (!OnlyKeys(s, {"p", "s", "axis", "twistDeg"}, path + ".similarity", why)) return false;
            return ReadNumArray(s, "p", kMetres(), 3, path + ".similarity", o.p, why) &&
                   NumField(s, "s", kOne(), path + ".similarity", o.s, why) &&
                   ReadNumArray(s, "axis", kOne(), 3, path + ".similarity", o.axis, why) &&
                   NumField(s, "twistDeg", kDegrees(), path + ".similarity", o.twistDeg, why);
        }
        case Sugar::Bad: break;
    }
    return Refuse(why, path + ": a placement is {x, alt, z, az, pitch}, {lat, lon, alt[, lookAt]}, "
                       "{motor: {re, du}} or {similarity: {p, s, axis, twistDeg}}");
}

}  // namespace

bool CheckPlacementSugar(const JsonValue& sugar, bool rigid, const std::string& path,
                         std::string* why) {
    SugarValues o;
    if (!ReadSugar(sugar, path, o, why)) return false;
    if (rigid && o.kind == Sugar::SimilarityForm) {
        return Refuse(why, path + ": a motor is rigid; the similarity spelling needs a Similarity");
    }
    return true;
}

// The rigid spellings as the session's own pose maps (scene/Pose.h): the compass spelling is
// SetFromCompass then FromCamera, the orbit spelling the orbit key (OrbitPose, PlanetToFlatPose,
// FromCamera) or, without lookAt, the globe start camera aimed at the planet's centre.
static bool ResolveMotor(const SugarValues& o, const PoseFrame& frame, const std::string& path,
                         Motor& out, std::string* why) {
    switch (o.kind) {
        case Sugar::Compass: {
            Camera c;
            c.SetFromCompass(o.x, o.alt, o.z, static_cast<float>(o.az), static_cast<float>(o.pitch));
            out = FromCamera(c);
            return true;
        }
        case Sugar::Orbit: {
            if (!frame.valid) return Refuse(why, path + ": the lat/lon spelling needs the tangent frame");
            const Camera g = o.lookAt ? OrbitPose(o.lat, o.lon, o.alt, o.tLat, o.tLon, frame.planetR)
                                      : GlobeCamera(o.lat, o.lon, o.alt, frame.planetR);
            out = FromCamera(PlanetToFlatPose(g, frame.east, frame.up, frame.north, frame.planetR));
            return true;
        }
        case Sugar::MotorForm:
            out.SetParts(o.re, o.du);
            return true;
        default: break;
    }
    return Refuse(why, path + ": not a rigid spelling");
}

bool ResolvePlacement(const JsonValue& sugar, const PoseFrame& frame, Placement& out,
                      std::string* why) {
    SugarValues o;
    if (!ReadSugar(sugar, "placement", o, why)) return false;
    if (o.kind == Sugar::SimilarityForm) {
        out = Placement::Similar(o.p, o.s, o.axis, o.twistDeg * 3.14159265358979 / 180.0);
        return true;
    }
    Motor m;
    if (!ResolveMotor(o, frame, "placement", m, why)) return false;
    out = Placement::Rigid(m);
    return true;
}

// M12 step 5d: the same read, stopping before the frame. PoseSugar::Kind mirrors Sugar's first
// four spellings (Bad never reaches here: ReadSugar refuses it).
bool ReadPoseSugar(const JsonValue& sugar, const std::string& path, PoseSugar& out,
                   std::string* why) {
    SugarValues o;
    if (!ReadSugar(sugar, path, o, why)) return false;
    out.kind = static_cast<PoseSugar::Kind>(static_cast<int>(o.kind));
    out.x = o.x;
    out.alt = o.alt;
    out.z = o.z;
    out.az = o.az;
    out.pitch = o.pitch;
    out.lat = o.lat;
    out.lon = o.lon;
    out.tLat = o.tLat;
    out.tLon = o.tLon;
    out.lookAt = o.lookAt;
    return true;
}

// The default sugar for a value the code holds: the motor spelling for a rigid placement,
// the similarity spelling otherwise (its fixed point and twist read back off the placement).
static JsonValue MotorSugar(const Motor& m) {
    double re[4], du[4];
    m.Real(re);
    m.Dual(du);
    JsonValue inner = JsonObj();
    JsonSet(inner, "re", NumArray(re, 4));
    JsonSet(inner, "du", NumArray(du, 4));
    JsonValue o = JsonObj();
    JsonSet(o, "motor", inner);
    return o;
}
static JsonValue PlacementSugar(const Placement& p) {
    if (p.IsRigid()) return MotorSugar(p.ToMotor());
    double fp[3], axis[3], angle;
    p.FixedPoint(fp);
    Placement::AxisAngle(p.r, axis, angle);
    JsonValue inner = JsonObj();
    JsonSet(inner, "p", NumArray(fp, 3));
    JsonSet(inner, "s", JsonNum(p.s));
    JsonSet(inner, "axis", NumArray(axis, 3));
    JsonSet(inner, "twistDeg", JsonNum(angle * 180.0 / 3.14159265358979));
    JsonValue o = JsonObj();
    JsonSet(o, "similarity", inner);
    return o;
}

// ---- the number law ---------------------------------------------------------------------------

bool NumberOf(const JsonValue& v, const UnitSpec& declared, const std::string& path, double& out,
              std::string* src, std::string* why) {
    double raw = 0.0;
    std::string unitText;
    if (v.type == JsonValue::Type::Number) {
        out = v.number;
        return true;
    } else if (v.type == JsonValue::Type::String) {
        const char* s = v.str.c_str();
        char* end = nullptr;
        raw = strtod(s, &end);
        if (end == s) return Refuse(why, path + ": expected a number, got '" + v.str + "'");
        while (*end == ' ' || *end == '\t') ++end;
        unitText = end;
        if (unitText.empty()) {
            out = raw;
            return true;
        }
    } else if (v.type == JsonValue::Type::Object) {
        const JsonValue* vv = v.Get("v");
        if (!vv || vv->type != JsonValue::Type::Number) {
            return Refuse(why, path + ": a {v, unit, src} number needs a numeric v");
        }
        raw = vv->number;
        unitText = v.Str("unit");
        if (src) *src = v.Str("src");
        if (unitText.empty()) {
            out = raw;
            return true;
        }
    } else {
        return Refuse(why, path + ": expected a number (bare, \"12 kn\" or {v, unit, src})");
    }
    const UnitSpec given = UnitSpec::Parse(unitText.c_str());
    std::string reason;
    if (!declared.AcceptsFrom(given, &reason)) return Refuse(why, path + ": " + reason);
    // given -> canonical -> declared: the canonical factor both ways, the datum offsets with it.
    const double canonical = raw * given.toCanonical + given.datumShiftM;
    out = (canonical - declared.datumShiftM) / declared.toCanonical;
    return true;
}

bool ParseProp(const PropDecl& d, const JsonValue& v, const std::string& path, PropValue& out,
               std::string* why) {
    out = PropValue{};
    switch (d.type) {
        case PropType::Bool:
            if (v.type == JsonValue::Type::Bool) out.b = v.boolean;
            else if (v.type == JsonValue::Type::Number) out.b = v.number != 0.0;   // SceneConfig's
                                                                                   // "both spellings"
            else return Refuse(why, path + ": expected true or false");
            break;
        case PropType::Number:
            if (!NumberOf(v, d.unit, path, out.n, &out.src, why)) return false;
            break;
        case PropType::String:
        case PropType::Path:
            if (v.type != JsonValue::Type::String) return Refuse(why, path + ": expected a string");
            out.s = v.str;
            break;
        case PropType::Enum: {
            if (v.type != JsonValue::Type::String) return Refuse(why, path + ": expected a name");
            int idx = -1;
            for (size_t i = 0; i < d.names.size(); ++i) {
                if (d.names[i] == v.str) idx = static_cast<int>(i);
            }
            if (idx < 0) {
                std::string all;
                for (const std::string& n : d.names) all += (all.empty() ? "" : " | ") + n;
                return Refuse(why, path + ": '" + v.str + "' is not one of " + all);
            }
            out.e = idx;
            out.s = v.str;
            break;
        }
        case PropType::Vec3:
        case PropType::Color: {
            const int need = d.type == PropType::Vec3 ? 3 : 3;
            const int most = d.type == PropType::Vec3 ? 3 : 4;
            if (v.type != JsonValue::Type::Array || static_cast<int>(v.arr.size()) < need ||
                static_cast<int>(v.arr.size()) > most) {
                return Refuse(why, path + ": expected an array of " + std::to_string(most));
            }
            out.v[3] = 1.0;
            for (size_t i = 0; i < v.arr.size(); ++i) {
                if (!NumberOf(v.arr[i], d.unit, path + "[" + std::to_string(i) + "]", out.v[i],
                              nullptr, why)) {
                    return false;
                }
            }
            break;
        }
        case PropType::DualSphere: {
            if (v.type != JsonValue::Type::Object) return Refuse(why, path + ": expected {c, r}");
            const JsonValue* c = v.Get("c");
            const JsonValue* r = v.Get("r");
            if (!c || c->type != JsonValue::Type::Array || c->arr.size() != 3 || !r) {
                return Refuse(why, path + ": a dual sphere is {c: [x, y, z], r}");
            }
            for (size_t i = 0; i < 3; ++i) {
                if (!NumberOf(c->arr[i], d.unit, path + ".c[" + std::to_string(i) + "]", out.v[i],
                              nullptr, why)) {
                    return false;
                }
            }
            if (!NumberOf(*r, d.unit, path + ".r", out.v[3], nullptr, why)) return false;
            break;
        }
        case PropType::Motor:
        case PropType::Similarity:
            if (!CheckPlacementSugar(v, d.type == PropType::Motor, path, why)) return false;
            out.raw = v;
            break;
        case PropType::Object:
            if (v.type != JsonValue::Type::Object) return Refuse(why, path + ": expected an object");
            break;
        case PropType::List:
            if (v.type != JsonValue::Type::Array) return Refuse(why, path + ": expected an array");
            break;
    }
    out.set = true;
    return true;
}

JsonValue PropToJson(const PropDecl& d, const PropValue& v) {
    switch (d.type) {
        case PropType::Bool: return JsonBool(v.b);
        case PropType::Number: return JsonNum(v.n);
        case PropType::String:
        case PropType::Path: return JsonStr(v.s);
        case PropType::Enum: return JsonStr(v.s);
        case PropType::Vec3: return NumArray(v.v, 3);
        case PropType::Color: return NumArray(v.v, 4);
        case PropType::DualSphere: {
            JsonValue o = JsonObj();
            JsonSet(o, "c", NumArray(v.v, 3));
            JsonSet(o, "r", JsonNum(v.v[3]));
            return o;
        }
        case PropType::Motor: return v.raw.type == JsonValue::Type::Object ? v.raw : MotorSugar(v.m);
        case PropType::Similarity:
            return v.raw.type == JsonValue::Type::Object ? v.raw : PlacementSugar(v.p);
        case PropType::Object: return JsonObj();
        case PropType::List: return JsonArr();
    }
    return JsonValue{};
}

bool PropEqual(const PropDecl& d, const PropValue& a, const PropValue& b) {
    if (a.set != b.set) return false;
    if (!a.set) return true;
    switch (d.type) {
        case PropType::Bool: return a.b == b.b;
        case PropType::Number: return a.n == b.n;
        case PropType::String:
        case PropType::Path:
        case PropType::Enum: return a.s == b.s;
        case PropType::Vec3: return a.v[0] == b.v[0] && a.v[1] == b.v[1] && a.v[2] == b.v[2];
        case PropType::Color:
        case PropType::DualSphere:
            return a.v[0] == b.v[0] && a.v[1] == b.v[1] && a.v[2] == b.v[2] && a.v[3] == b.v[3];
        case PropType::Motor:
        case PropType::Similarity: return JsonEqual(PropToJson(d, a), PropToJson(d, b));
        case PropType::Object:
        case PropType::List: return true;
    }
    return false;
}

// ---- PropSet ----------------------------------------------------------------------------------

PropSet::PropSet(const Schema& s) : m_schema(&s) {
    m_values.resize(s.Decls().size());
    m_objects.resize(s.Decls().size());
}

PropSet::PropSet(const PropSet& o) : m_schema(o.m_schema), m_values(o.m_values) {
    m_objects.resize(o.m_objects.size());
    for (size_t i = 0; i < o.m_objects.size(); ++i) {
        if (o.m_objects[i]) m_objects[i] = std::make_unique<PropSet>(*o.m_objects[i]);
    }
}

PropSet& PropSet::operator=(const PropSet& o) {
    if (this == &o) return *this;
    m_schema = o.m_schema;
    m_values = o.m_values;
    m_objects.clear();
    m_objects.resize(o.m_objects.size());
    for (size_t i = 0; i < o.m_objects.size(); ++i) {
        if (o.m_objects[i]) m_objects[i] = std::make_unique<PropSet>(*o.m_objects[i]);
    }
    return *this;
}

PropSet PropSet::Defaults(const Schema& s, const void* instance) {
    PropSet ps(s);
    const char* base = static_cast<const char*>(instance);
    const auto& decls = s.Decls();
    for (size_t i = 0; i < decls.size(); ++i) {
        const PropDecl& d = decls[i];
        PropValue& v = ps.m_values[i];
        const char* at = base + d.offset;
        switch (d.field) {
            case Field::Bool: v.b = *reinterpret_cast<const bool*>(at); break;
            case Field::F32: v.n = FloatAsDouble(*reinterpret_cast<const float*>(at)); break;
            case Field::F64: v.n = *reinterpret_cast<const double*>(at); break;
            case Field::I32: {
                const int iv = *reinterpret_cast<const int*>(at);
                if (d.type == PropType::Enum) {
                    v.e = iv;
                    v.s = (iv >= 0 && static_cast<size_t>(iv) < d.names.size())
                              ? d.names[static_cast<size_t>(iv)] : std::string();
                } else {
                    v.n = iv;
                }
                break;
            }
            case Field::U32: v.n = *reinterpret_cast<const uint32_t*>(at); break;
            case Field::Str: v.s = *reinterpret_cast<const std::string*>(at); break;
            case Field::D3: {
                const double* p = reinterpret_cast<const double*>(at);
                v.v[0] = p[0]; v.v[1] = p[1]; v.v[2] = p[2];
                break;
            }
            case Field::D4: {
                const double* p = reinterpret_cast<const double*>(at);
                for (int k = 0; k < 4; ++k) v.v[k] = p[k];
                break;
            }
            case Field::F4: {
                const float* p = reinterpret_cast<const float*>(at);
                for (int k = 0; k < 4; ++k) v.v[k] = FloatAsDouble(p[k]);
                break;
            }
            case Field::Motor: v.m = *reinterpret_cast<const Motor*>(at); v.raw = MotorSugar(v.m); break;
            case Field::Placement:
                v.p = *reinterpret_cast<const Placement*>(at);
                v.raw = PlacementSugar(v.p);
                break;
            case Field::None: break;
        }
        if (d.type == PropType::Object) {
            ps.m_objects[i] = std::make_unique<PropSet>(Defaults(*d.sub, at));
        } else if (d.type != PropType::List) {
            v.set = !d.optional;
        }
    }
    return ps;
}

bool PropSet::Merge(const JsonValue& obj, const std::string& path, std::string* why) {
    if (obj.type != JsonValue::Type::Object) return Refuse(why, path + ": expected an object");
    const auto& decls = m_schema->Decls();
    for (const auto& kv : obj.obj) {
        if (Schema::IsComment(kv.first)) continue;
        const PropDecl* d = m_schema->Find(kv.first);
        if (!d) {
            std::string known;
            for (const PropDecl& k : decls) known += (known.empty() ? "" : ", ") + k.key;
            return Refuse(why, path + "." + kv.first + ": unknown key for " + m_schema->Type() +
                                   " (known: " + known + ")");
        }
        const size_t i = static_cast<size_t>(d - decls.data());
        const std::string sub = path + "." + kv.first;
        if (d->type == PropType::Object) {
            if (!m_objects[i]) m_objects[i] = std::make_unique<PropSet>(*d->sub);
            if (!m_objects[i]->Merge(kv.second, sub, why)) return false;
            continue;
        }
        if (d->type == PropType::List) {
            if (kv.second.type != JsonValue::Type::Array) return Refuse(why, sub + ": expected an array");
            continue;   // the document's business (SceneBuilder)
        }
        if (!ParseProp(*d, kv.second, sub, m_values[i], why)) return false;
    }
    return true;
}

bool PropSet::ApplyTo(void* instance, const PoseFrame* frame, std::string* why) const {
    char* base = static_cast<char*>(instance);
    const auto& decls = m_schema->Decls();
    for (size_t i = 0; i < decls.size(); ++i) {
        const PropDecl& d = decls[i];
        char* at = base + d.offset;
        if (d.type == PropType::Object) {
            if (m_objects[i] && !m_objects[i]->ApplyTo(at, frame, why)) return false;
            continue;
        }
        const PropValue& v = m_values[i];
        if (!v.set) continue;
        switch (d.field) {
            case Field::Bool: *reinterpret_cast<bool*>(at) = v.b; break;
            case Field::F32: *reinterpret_cast<float*>(at) = static_cast<float>(v.n); break;
            case Field::F64: *reinterpret_cast<double*>(at) = v.n; break;
            case Field::I32:
                *reinterpret_cast<int*>(at) = d.type == PropType::Enum ? v.e : static_cast<int>(v.n);
                break;
            case Field::U32: *reinterpret_cast<uint32_t*>(at) = static_cast<uint32_t>(v.n); break;
            case Field::Str: *reinterpret_cast<std::string*>(at) = v.s; break;
            case Field::D3: {
                double* p = reinterpret_cast<double*>(at);
                p[0] = v.v[0]; p[1] = v.v[1]; p[2] = v.v[2];
                break;
            }
            case Field::D4: {
                double* p = reinterpret_cast<double*>(at);
                for (int k = 0; k < 4; ++k) p[k] = v.v[k];
                break;
            }
            case Field::F4: {
                float* p = reinterpret_cast<float*>(at);
                for (int k = 0; k < 4; ++k) p[k] = static_cast<float>(v.v[k]);
                break;
            }
            case Field::Motor: {
                if (!frame) return Refuse(why, d.key + ": a motor needs the pose frame to resolve");
                SugarValues o;
                if (!ReadSugar(v.raw, d.key, o, why)) return false;
                Motor m;
                if (!ResolveMotor(o, *frame, d.key, m, why)) return false;
                *reinterpret_cast<Motor*>(at) = m;
                break;
            }
            case Field::Placement: {
                if (!frame) return Refuse(why, d.key + ": a placement needs the pose frame to resolve");
                Placement p;
                if (!ResolvePlacement(v.raw, *frame, p, why)) return false;
                *reinterpret_cast<Placement*>(at) = p;
                break;
            }
            case Field::None: break;
        }
    }
    return true;
}

JsonValue PropSet::ToJson() const {
    JsonValue o = JsonObj();
    const auto& decls = m_schema->Decls();
    for (size_t i = 0; i < decls.size(); ++i) {
        const PropDecl& d = decls[i];
        if (d.type == PropType::Object) {
            JsonSet(o, d.key, m_objects[i] ? m_objects[i]->ToJson()
                                           : PropSet::Defaults(*d.sub, static_cast<const char*>(
                                                 m_schema->Prototype()) + d.offset).ToJson());
            continue;
        }
        if (d.type == PropType::List) {
            JsonSet(o, d.key, JsonArr());
            continue;
        }
        const PropValue& v = m_values[i];
        if (!v.set) {
            if (d.optional) continue;
            JsonSet(o, d.key, m_schema->DefaultOf(d));
            continue;
        }
        JsonSet(o, d.key, PropToJson(d, v));
    }
    return o;
}

std::vector<PropSet::Change> PropSet::Diff(const PropSet& before) const {
    std::vector<Change> out;
    const auto& decls = m_schema->Decls();
    for (size_t i = 0; i < decls.size(); ++i) {
        const PropDecl& d = decls[i];
        if (d.type == PropType::Object) {
            if (m_objects[i] && before.m_objects[i]) {
                for (const Change& c : m_objects[i]->Diff(*before.m_objects[i])) {
                    out.push_back({d.key + "." + c.key, c.reload});
                }
            }
            continue;
        }
        if (d.type == PropType::List) continue;
        if (!PropEqual(d, m_values[i], before.m_values[i])) out.push_back({d.key, d.reload});
    }
    return out;
}

const PropValue* PropSet::Get(const char* key) const {
    const PropDecl* d = m_schema->Find(key);
    if (!d) return nullptr;
    return &m_values[static_cast<size_t>(d - m_schema->Decls().data())];
}

const PropSet* PropSet::Object(const char* key) const {
    const PropDecl* d = m_schema->Find(key);
    if (!d || d->type != PropType::Object) return nullptr;
    return m_objects[static_cast<size_t>(d - m_schema->Decls().data())].get();
}

}  // namespace ga::scene
