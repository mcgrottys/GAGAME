// SceneBuilder - the fold's bodies (M12 step 5a). See SceneBuilder.h.
#include "scene/SceneBuilder.h"

#include "core/Common.h"
#include "scene/SceneSchema.h"

#include <cstdio>
#include <cstdlib>
#include <cstring>

namespace ga::scene {

namespace {

bool Refuse(std::string* why, const std::string& text) {
    if (why) *why = text;
    return false;
}

bool IsObject(const JsonValue& v) { return v.type == JsonValue::Type::Object; }
bool IsArray(const JsonValue& v) { return v.type == JsonValue::Type::Array; }

const JsonValue* NameOf(const JsonValue& elem, const char* key) {
    const JsonValue* n = elem.Get(key);
    return (n && n->type == JsonValue::Type::String) ? n : nullptr;
}

bool RemoveFlag(const JsonValue& elem) {
    const JsonValue* r = elem.Get("remove");
    return r && r->type == JsonValue::Type::Bool && r->boolean;
}

Registry<const Schema*>* PolyRegistry(const char* poly) {
    if (!poly) return nullptr;
    if (strcmp(poly, "component") == 0) return &ComponentSchemas();
    if (strcmp(poly, "effect") == 0) return &EffectSchemas();
    if (strcmp(poly, "layer") == 0) return &LayerSchemas();
    return nullptr;
}

std::string Join(const std::vector<std::string>& v) {
    std::string s;
    for (const std::string& x : v) s += (s.empty() ? "" : ", ") + x;
    return s;
}

// Named-array merge: in place by name, remove, append (the banner's law).
void MergeList(JsonValue& base, const JsonValue& overlay, const PropDecl& decl,
               const std::string& path) {
    // M12 step 5d: AN UNNAMED LIST IS A VALUE, AND AN OVERLAY REPLACES IT WHOLE. `include`,
    // `water.fleet.boats` and `rails.keys` carry no names to merge by, so appending was the only
    // thing the law could say about them -- and appending makes the fold NON-IDEMPOTENT: loading
    // an already-resolved document (`gagame scenes/recipes/helm.json`, whose own `include`
    // re-applies data/wave_scene.json at `water`) doubled the fleet, and a `base` chain doubled
    // the include list. A list of boats is the FLEET the way a versor is a pose -- a value, not a
    // namespace and not an accumulation -- which is the rule Schema::Atomic already states for
    // the other value shapes. Nothing the FLAG path resolves changes: those three lists are empty
    // in the defaults the fold starts from, so every recipe's --print-scene text is what it was.
    if (!decl.named) {
        base.arr = overlay.arr;
        return;
    }
    for (const JsonValue& e : overlay.arr) {
        const JsonValue* name = (decl.named && IsObject(e)) ? NameOf(e, "name") : nullptr;
        JsonValue* target = nullptr;
        size_t at = 0;
        if (name) {
            for (size_t i = 0; i < base.arr.size(); ++i) {
                const JsonValue* bn = IsObject(base.arr[i]) ? NameOf(base.arr[i], "name") : nullptr;
                if (bn && bn->str == name->str) {
                    target = &base.arr[i];
                    at = i;
                    break;
                }
            }
        }
        if (target) {
            if (RemoveFlag(e)) {
                base.arr.erase(base.arr.begin() + static_cast<std::ptrdiff_t>(at));
                continue;
            }
            MergeInto(*target, e, ElementChain(decl, *target), path + "." + name->str);
            continue;
        }
        if (IsObject(e) && RemoveFlag(e)) continue;   // nothing to remove
        base.arr.push_back(e);
    }
}

}  // namespace

// ---- SchemaChain --------------------------------------------------------------------------------

const PropDecl* SchemaChain::Find(const std::string& key) const {
    if (a) {
        if (const PropDecl* d = a->Find(key)) return d;
    }
    if (b) {
        if (const PropDecl* d = b->Find(key)) return d;
    }
    return nullptr;
}

const char* SchemaChain::Type() const { return a ? a->Type() : (b ? b->Type() : "(untyped)"); }

SchemaChain ElementChain(const PropDecl& list, const JsonValue& element) {
    SchemaChain c;
    c.a = list.sub;
    if (list.poly && IsObject(element)) {
        if (const JsonValue* t = NameOf(element, list.polyKey)) {
            Registry<const Schema*>* reg = PolyRegistry(list.poly);
            if (reg && reg->Knows(t->str)) c.b = reg->Make(t->str);
        }
    }
    return c;
}

// ---- the merge law ------------------------------------------------------------------------------

void MergeInto(JsonValue& base, const JsonValue& overlay, const SchemaChain& chain,
               const std::string& path) {
    if (!IsObject(base) || !IsObject(overlay)) {
        base = overlay;   // scalars (and mismatched shapes) replace
        return;
    }
    for (const auto& kv : overlay.obj) {
        const std::string& key = kv.first;
        const JsonValue& v = kv.second;
        JsonValue* b = JsonGet(base, key);
        if (!b) {
            JsonSet(base, key, v);   // a new key (a comment, an unknown key: validation's business)
            continue;
        }
        const PropDecl* d = chain.Find(key);
        if (d && Schema::Atomic(d->type)) {
            *b = v;
            continue;
        }
        if (d && d->type == PropType::List) {
            if (IsArray(*b) && IsArray(v)) MergeList(*b, v, *d, path + "." + key);
            else *b = v;
            continue;
        }
        if (IsObject(*b) && IsObject(v)) {
            SchemaChain sub;
            sub.a = (d && d->type == PropType::Object) ? d->sub : nullptr;
            MergeInto(*b, v, sub, path + "." + key);
            continue;
        }
        *b = v;
    }
}

// ---- completion ---------------------------------------------------------------------------------

static void CompleteWith(JsonValue& obj, const Schema* s) {
    if (!s) return;
    for (const PropDecl& d : s->Decls()) {
        JsonValue* v = JsonGet(obj, d.key);
        if (!v) {
            if (d.optional) continue;
            JsonSet(obj, d.key, s->DefaultOf(d));
            v = JsonGet(obj, d.key);
        }
        if (d.type == PropType::Object) {
            if (IsObject(*v)) {
                SchemaChain sub;
                sub.a = d.sub;
                Complete(*v, sub);
            }
        } else if (d.type == PropType::List && IsArray(*v)) {
            for (size_t i = 0; i < v->arr.size();) {
                JsonValue& e = v->arr[i];
                if (IsObject(e) && RemoveFlag(e)) {
                    v->arr.erase(v->arr.begin() + static_cast<std::ptrdiff_t>(i));
                    continue;
                }
                if (IsObject(e)) Complete(e, ElementChain(d, e));
                ++i;
            }
        }
    }
}

// The resolved order of one object: comments first, the declared keys in schema order (the
// chain's first schema, then its second), then anything undeclared, each group in its own
// original order. Deterministic, so the document reads the same however it was folded.
static void Order(JsonValue& obj, const SchemaChain& chain) {
    if (!IsObject(obj)) return;
    std::vector<std::pair<std::string, JsonValue>> out;
    auto take = [&](const std::string& key) {
        for (size_t i = 0; i < obj.obj.size(); ++i) {
            if (obj.obj[i].first == key) {
                out.push_back(std::move(obj.obj[i]));
                obj.obj.erase(obj.obj.begin() + static_cast<std::ptrdiff_t>(i));
                return;
            }
        }
    };
    for (size_t i = 0; i < obj.obj.size();) {
        if (Schema::IsComment(obj.obj[i].first)) {
            out.push_back(std::move(obj.obj[i]));
            obj.obj.erase(obj.obj.begin() + static_cast<std::ptrdiff_t>(i));
            continue;
        }
        ++i;
    }
    for (const Schema* s : {chain.a, chain.b}) {
        if (!s) continue;
        for (const PropDecl& d : s->Decls()) take(d.key);
    }
    for (auto& kv : obj.obj) out.push_back(std::move(kv));
    obj.obj = std::move(out);
}

void Complete(JsonValue& doc, const SchemaChain& chain) {
    CompleteWith(doc, chain.a);
    CompleteWith(doc, chain.b);
    Order(doc, chain);
}

// ---- validation ---------------------------------------------------------------------------------

bool Validate(const JsonValue& doc, const SchemaChain& chain, const std::string& path,
              std::string* why) {
    if (!IsObject(doc)) return Refuse(why, path + ": expected an object");
    std::vector<std::string> names;
    for (const auto& kv : doc.obj) {
        const std::string& key = kv.first;
        const JsonValue& v = kv.second;
        if (Schema::IsComment(key)) continue;
        const std::string sub = path.empty() ? key : path + "." + key;
        const PropDecl* d = chain.Find(key);
        if (!d) {
            std::vector<std::string> known;
            if (chain.a) for (const PropDecl& k : chain.a->Decls()) known.push_back(k.key);
            if (chain.b) for (const PropDecl& k : chain.b->Decls()) known.push_back(k.key);
            return Refuse(why, sub + ": unknown key for " + chain.Type() + " (known: " +
                                   Join(known) + ")");
        }
        if (d->type == PropType::Object) {
            SchemaChain c;
            c.a = d->sub;
            if (!Validate(v, c, sub, why)) return false;
            continue;
        }
        if (d->type == PropType::List) {
            if (!IsArray(v)) return Refuse(why, sub + ": expected an array");
            std::vector<std::string> seen;
            for (size_t i = 0; i < v.arr.size(); ++i) {
                const JsonValue& e = v.arr[i];
                const std::string idx = sub + "[" + std::to_string(i) + "]";
                if (!IsObject(e)) return Refuse(why, idx + ": expected an object");
                std::string ename = idx;
                if (d->named) {
                    const JsonValue* n = NameOf(e, "name");
                    if (!n || n->str.empty()) return Refuse(why, idx + ": an element of " + sub +
                                                                 " needs a name");
                    for (const std::string& s : seen) {
                        if (s == n->str) return Refuse(why, sub + "." + n->str + ": named twice");
                    }
                    seen.push_back(n->str);
                    ename = sub + "." + n->str;
                }
                SchemaChain c = ElementChain(*d, e);
                if (d->poly) {
                    const JsonValue* t = NameOf(e, d->polyKey);
                    Registry<const Schema*>* reg = PolyRegistry(d->poly);
                    if (!t) return Refuse(why, ename + ": needs a '" + d->polyKey + "'");
                    if (!reg || !reg->Knows(t->str)) {
                        return Refuse(why, ename + "." + d->polyKey + ": unknown " + d->poly +
                                               " '" + t->str + "' (known: " +
                                               (reg ? Join(reg->Names()) : std::string()) + ")");
                    }
                }
                if (!Validate(e, c, ename, why)) return false;
            }
            continue;
        }
        PropValue pv;
        if (!ParseProp(*d, v, sub, pv, why)) return false;
    }
    (void)names;
    return true;
}

// ---- SceneBuilder ------------------------------------------------------------------------------

SceneBuilder::SceneBuilder() {
    RegisterBuiltinSceneTypes();
    m_doc = DefaultDocument();
}

bool SceneBuilder::ReadFile(const std::string& path, std::string& text, std::string* why) {
    FILE* f = fopen(path.c_str(), "rb");
    if (!f) return Refuse(why, path + ": cannot open");
    char buf[4096];
    size_t n;
    text.clear();
    while ((n = fread(buf, 1, sizeof(buf), f)) > 0) text.append(buf, n);
    fclose(f);
    return true;
}

JsonValue SceneBuilder::SetValue(const std::string& text) {
    std::string err;
    JsonValue v = JsonParser::Parse(text, &err);
    if (err.empty()) return v;
    return JsonStr(text);
}

bool SceneBuilder::Load(const std::string& path, std::string* why) {
    for (const std::string& p : m_loading) {
        if (p == path) return Refuse(why, path + ": included by itself (a cycle)");
    }
    std::string text;
    if (!ReadFile(path, text, why)) return false;
    std::string err;
    const JsonValue doc = JsonParser::Parse(text, &err);
    if (!err.empty()) return Refuse(why, path + ": " + err);
    if (!IsObject(doc)) return Refuse(why, path + ": a scene file is an object");
    if (m_base.empty()) m_base = path;
    m_files.push_back(path);
    m_loading.push_back(path);
    // The scene this one inherits: loaded first, so this file's keys win over it. M12 step 5d:
    // a COMPLETE document carries every declared key, so a resolved recipe says `"base": ""` --
    // which means NONE, not a broken path. Only a non-string is a refusal.
    const JsonValue* base = doc.Get("base");
    if (base && base->type != JsonValue::Type::String) {
        m_loading.pop_back();
        return Refuse(why, path + ": base must name a scene file");
    }
    if (base && !base->str.empty()) {
        if (!Load(base->str, why)) {
            m_loading.pop_back();
            return false;
        }
    }
    SchemaChain root;
    root.a = &SceneFileSchema();
    MergeInto(m_doc, doc, root, "");
    const bool ok = ApplyIncludes(doc, why);
    m_loading.pop_back();
    return ok;
}

bool SceneBuilder::ApplyIncludes(const JsonValue& doc, std::string* why) {
    const JsonValue* inc = doc.Get("include");
    if (!inc) return true;
    if (!IsArray(*inc)) return Refuse(why, "include: expected an array");
    // Normalize: a bare string is {file}. The document keeps the normalized form.
    JsonValue* mine = JsonGet(m_doc, "include");
    if (mine && IsArray(*mine)) {
        for (JsonValue& e : mine->arr) {
            if (e.type == JsonValue::Type::String) {
                JsonValue o = JsonObj();
                JsonSet(o, "file", e);
                e = o;
            }
        }
    }
    for (const JsonValue& e : inc->arr) {
        std::string file, at;
        bool optional = false;
        if (e.type == JsonValue::Type::String) {
            file = e.str;
        } else if (IsObject(e)) {
            file = e.Str("file");
            at = e.Str("at");
            const JsonValue* o = e.Get("optional");
            optional = o && o->type == JsonValue::Type::Bool && o->boolean;
        }
        if (file.empty()) return Refuse(why, "include: an entry needs a file");
        std::string text;
        if (!ReadFile(file, text, nullptr)) {
            if (optional) {
                // stderr, not Log: --print-scene owns stdout.
                fprintf(stderr, "[scene] include %s: absent, skipped (optional)\n", file.c_str());
                continue;
            }
            return Refuse(why, "include " + file + ": cannot open");
        }
        std::string err;
        const JsonValue sub = JsonParser::Parse(text, &err);
        if (!err.empty()) return Refuse(why, "include " + file + ": " + err);
        for (const std::string& p : m_loading) {
            if (p == file) return Refuse(why, file + ": included by itself (a cycle)");
        }
        m_loading.push_back(file);
        m_files.push_back(file);
        const bool ok = Overlay(sub, file, at, why) && ApplyIncludes(sub, why);
        m_loading.pop_back();
        if (!ok) return false;
    }
    return true;
}

bool SceneBuilder::Overlay(const JsonValue& doc, const std::string& from, const std::string& at,
                           std::string* why) {
    if (!IsObject(doc)) return Refuse(why, from + ": an overlay is an object");
    JsonValue* target = &m_doc;
    SchemaChain chain;
    chain.a = &SceneFileSchema();
    std::string path;
    if (!at.empty()) {
        size_t pos = 0;
        while (pos <= at.size()) {
            const size_t dot = at.find('.', pos);
            const std::string seg = at.substr(pos, dot == std::string::npos ? std::string::npos : dot - pos);
            const PropDecl* d = chain.Find(seg);
            if (!d || d->type != PropType::Object) {
                return Refuse(why, from + ": include at '" + at + "': '" + seg + "' is not a section");
            }
            JsonValue* next = JsonGet(*target, seg);
            if (!next) next = JsonSet(*target, seg, JsonObj());
            target = next;
            chain.a = d->sub;
            chain.b = nullptr;
            path = path.empty() ? seg : path + "." + seg;
            if (dot == std::string::npos) break;
            pos = dot + 1;
        }
    }
    // An overlay's own `include` is applied by the caller, not merged as data; its root-level
    // comments (_readme) are about the overlay file and stay with it -- a comment beside a
    // value (wave_scene.json's _exag under wavefield) travels with the value.
    JsonValue body = doc;
    for (size_t i = 0; i < body.obj.size();) {
        const std::string& k = body.obj[i].first;
        if ((at.empty() && k == "include") || Schema::IsComment(k)) {
            body.obj.erase(body.obj.begin() + static_cast<std::ptrdiff_t>(i));
            continue;
        }
        ++i;
    }
    MergeInto(*target, body, chain, path);
    return true;
}

bool SceneBuilder::Set(const std::string& path, const JsonValue& value, std::string* why) {
    if (path.empty()) return Refuse(why, "--set: an empty path");
    std::vector<std::string> segs;
    size_t pos = 0;
    while (true) {
        const size_t dot = path.find('.', pos);
        segs.push_back(path.substr(pos, dot == std::string::npos ? std::string::npos : dot - pos));
        if (dot == std::string::npos) break;
        pos = dot + 1;
    }
    if (segs.front() == "include" || segs.front() == "base") {
        return Refuse(why, "--set " + segs.front() + ": a file's key, not a property");
    }
    JsonValue* cur = &m_doc;
    SchemaChain chain;
    chain.a = &SceneFileSchema();
    const PropDecl* listDecl = nullptr;   // set when `cur` is a list's array
    std::string walked;
    for (size_t i = 0; i + 1 < segs.size(); ++i) {
        const std::string& seg = segs[i];
        if (IsArray(*cur)) {
            // An element by name (named list) or by index (unnamed).
            JsonValue* elem = nullptr;
            if (listDecl && listDecl->named) {
                for (JsonValue& e : cur->arr) {
                    const JsonValue* n = IsObject(e) ? NameOf(e, "name") : nullptr;
                    if (n && n->str == seg) {
                        elem = &e;
                        break;
                    }
                }
                if (!elem) {
                    JsonValue fresh = JsonObj();
                    JsonSet(fresh, "name", JsonStr(seg));
                    cur->arr.push_back(fresh);
                    elem = &cur->arr.back();
                }
            } else {
                char* end = nullptr;
                const long idx = strtol(seg.c_str(), &end, 10);
                if (!end || *end != '\0' || idx < 0 || static_cast<size_t>(idx) >= cur->arr.size()) {
                    return Refuse(why, "--set " + path + ": '" + seg + "' is not an index of " + walked);
                }
                elem = &cur->arr[static_cast<size_t>(idx)];
            }
            chain = listDecl ? ElementChain(*listDecl, *elem) : SchemaChain{};
            listDecl = nullptr;
            cur = elem;
        } else if (IsObject(*cur)) {
            const PropDecl* d = chain.Find(seg);
            JsonValue* next = JsonGet(*cur, seg);
            if (!next) {
                if (d && d->type == PropType::List) next = JsonSet(*cur, seg, JsonArr());
                else next = JsonSet(*cur, seg, JsonObj());
            }
            if (d && d->type == PropType::List) {
                listDecl = d;
            } else {
                SchemaChain sub;
                sub.a = (d && d->type == PropType::Object) ? d->sub : nullptr;
                chain = sub;
                listDecl = nullptr;
            }
            cur = next;
        } else {
            return Refuse(why, "--set " + path + ": '" + walked + "' is a scalar");
        }
        walked = walked.empty() ? seg : walked + "." + seg;
    }
    const std::string& last = segs.back();
    if (IsArray(*cur)) {
        if (!listDecl) return Refuse(why, "--set " + path + ": cannot address into an untyped array");
        JsonValue e = JsonObj();
        if (listDecl->named) JsonSet(e, "name", JsonStr(last));
        if (IsObject(value)) {
            for (const auto& kv : value.obj) JsonSet(e, kv.first, kv.second);
        }
        JsonValue one = JsonArr();
        one.arr.push_back(e);
        MergeList(*cur, one, *listDecl, walked);
        return true;
    }
    if (!IsObject(*cur)) return Refuse(why, "--set " + path + ": '" + walked + "' is a scalar");
    JsonValue one = JsonObj();
    JsonSet(one, last, value);
    MergeInto(*cur, one, chain, walked);
    return true;
}

bool SceneBuilder::SetText(const std::string& assignment, std::string* why) {
    const size_t eq = assignment.find('=');
    if (eq == std::string::npos) return Refuse(why, "--set " + assignment + ": expected path=value");
    return Set(assignment.substr(0, eq), SetValue(assignment.substr(eq + 1)), why);
}

bool SceneBuilder::Resolve(std::string* why) {
    SchemaChain root;
    root.a = &SceneFileSchema();
    Complete(m_doc, root);
    return Validate(m_doc, root, "", why);
}

// ---- the writer ---------------------------------------------------------------------------------

namespace {

void EscapeInto(std::string& out, const std::string& s) {
    out.push_back('"');
    for (char c : s) {
        switch (c) {
            case '"': out += "\\\""; break;
            case '\\': out += "\\\\"; break;
            case '\n': out += "\\n"; break;
            case '\r': out += "\\r"; break;
            case '\t': out += "\\t"; break;
            default:
                if (static_cast<unsigned char>(c) < 0x20) {
                    char b[8];
                    snprintf(b, sizeof(b), "\\u%04x", static_cast<unsigned>(c));
                    out += b;
                } else {
                    out.push_back(c);
                }
        }
    }
    out.push_back('"');
}

bool Scalar(const JsonValue& v) {
    return v.type != JsonValue::Type::Object && v.type != JsonValue::Type::Array;
}

void WriteScalar(std::string& out, const JsonValue& v) {
    switch (v.type) {
        case JsonValue::Type::Null: out += "null"; break;
        case JsonValue::Type::Bool: out += v.boolean ? "true" : "false"; break;
        case JsonValue::Type::Number: out += NumberText(v.number); break;
        case JsonValue::Type::String: EscapeInto(out, v.str); break;
        default: break;
    }
}

constexpr size_t kInlineWidth = 100;

// One line when every member is a scalar and it fits.
bool Inline(std::string& out, const JsonValue& v) {
    std::string s;
    if (v.type == JsonValue::Type::Object) {
        s = "{";
        for (size_t i = 0; i < v.obj.size(); ++i) {
            if (!Scalar(v.obj[i].second)) return false;
            if (i) s += ", ";
            EscapeInto(s, v.obj[i].first);
            s += ": ";
            WriteScalar(s, v.obj[i].second);
        }
        s += "}";
    } else {
        s = "[";
        for (size_t i = 0; i < v.arr.size(); ++i) {
            if (!Scalar(v.arr[i])) return false;
            if (i) s += ", ";
            WriteScalar(s, v.arr[i]);
        }
        s += "]";
    }
    if (s.size() > kInlineWidth) return false;
    out += s;
    return true;
}

void Write(std::string& out, const JsonValue& v, int indent) {
    if (Scalar(v)) {
        WriteScalar(out, v);
        return;
    }
    const bool obj = v.type == JsonValue::Type::Object;
    const size_t n = obj ? v.obj.size() : v.arr.size();
    if (n == 0) {
        out += obj ? "{}" : "[]";
        return;
    }
    if (Inline(out, v)) return;
    const std::string pad(static_cast<size_t>(indent + 2), ' ');
    out += obj ? "{\n" : "[\n";
    for (size_t i = 0; i < n; ++i) {
        out += pad;
        if (obj) {
            EscapeInto(out, v.obj[i].first);
            out += ": ";
            Write(out, v.obj[i].second, indent + 2);
        } else {
            Write(out, v.arr[i], indent + 2);
        }
        out += (i + 1 < n) ? ",\n" : "\n";
    }
    out += std::string(static_cast<size_t>(indent), ' ');
    out += obj ? "}" : "]";
}

}  // namespace

std::string SceneBuilder::WriteJson(const JsonValue& v) {
    std::string out;
    Write(out, v, 0);
    out += "\n";
    return out;
}

}  // namespace ga::scene
