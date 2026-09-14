// SceneReload - the watch set, the re-resolution, the diff and the apply (M12 step 5f).
#include "scene/SceneReload.h"

#include "core/Common.h"

#include <cstdio>

namespace ga::scene {

namespace {

bool IsObj(const JsonValue& v) { return v.type == JsonValue::Type::Object; }
bool IsArr(const JsonValue& v) { return v.type == JsonValue::Type::Array; }

// The canonical text one value hashes and prints: SceneBuilder's writer, trimmed of its end.
std::string Text(const JsonValue& v) {
    std::string t = SceneBuilder::WriteJson(v);
    while (!t.empty() && (t.back() == '\n' || t.back() == '\r')) t.pop_back();
    return t;
}

// An array element's address: its `name` where it carries one (the merge law's NAMED array,
// which merges by name), its index where it does not (include, rails.keys, water.fleet.boats --
// the unnamed arrays, which are values replaced whole). One rule, drawn where the law draws it.
std::string ElemKey(const JsonValue& e, size_t i) {
    if (IsObj(e)) {
        const JsonValue* n = e.Get("name");
        if (n && n->type == JsonValue::Type::String && !n->str.empty()) return n->str;
    }
    return std::to_string(i);
}

bool IsCovered(const std::vector<std::string>& covered, const std::string& path) {
    for (const std::string& c : covered) {
        if (path == c) return true;
        if (path.size() > c.size() && path.compare(0, c.size(), c) == 0 && path[c.size()] == '.') {
            return true;
        }
    }
    return false;
}

bool Refuse(std::string* why, const std::string& msg) {
    if (why) *why = msg;
    return false;
}

std::vector<std::string> Split(const std::string& path) {
    std::vector<std::string> out;
    size_t i = 0;
    while (i <= path.size()) {
        const size_t dot = path.find('.', i);
        out.push_back(path.substr(i, dot == std::string::npos ? std::string::npos : dot - i));
        if (dot == std::string::npos) break;
        i = dot + 1;
    }
    return out;
}

}  // namespace

// ---- the pure laws ---------------------------------------------------------------------------

const JsonValue* SceneReload::At(const JsonValue& doc, const std::string& path) {
    if (path.empty()) return &doc;
    const JsonValue* v = &doc;
    for (const std::string& seg : Split(path)) {
        if (seg.empty()) return nullptr;
        if (IsObj(*v)) {
            const JsonValue* n = v->Get(seg.c_str());
            if (!n) return nullptr;
            v = n;
            continue;
        }
        if (IsArr(*v)) {
            const JsonValue* hit = nullptr;
            for (size_t k = 0; k < v->arr.size(); ++k) {
                if (ElemKey(v->arr[k], k) == seg) {
                    hit = &v->arr[k];
                    break;
                }
            }
            if (!hit) return nullptr;
            v = hit;
            continue;
        }
        return nullptr;
    }
    return v;
}

JsonValue SceneReload::Pick(const JsonValue& obj, const Schema& s) {
    JsonValue o = JsonObj();
    if (!IsObj(obj)) return o;
    for (const auto& kv : obj.obj) {
        if (Schema::IsComment(kv.first)) continue;
        if (!s.Find(kv.first)) continue;   // the other half of a typed element's chain
        JsonSet(o, kv.first, kv.second);
    }
    return o;
}

void SceneReload::Residue(const JsonValue& was, const JsonValue& now, const std::string& path,
                          const std::vector<std::string>& covered,
                          std::vector<std::string>& out) {
    if (!path.empty() && IsCovered(covered, path)) return;
    if (IsObj(was) && IsObj(now)) {
        for (const auto& kv : now.obj) {
            if (Schema::IsComment(kv.first)) continue;
            const std::string p = path.empty() ? kv.first : path + "." + kv.first;
            const JsonValue* b = was.Get(kv.first.c_str());
            if (!b) {
                if (!IsCovered(covered, p)) out.push_back(p + " (new)");
                continue;
            }
            Residue(*b, kv.second, p, covered, out);
        }
        for (const auto& kv : was.obj) {
            if (Schema::IsComment(kv.first) || now.Get(kv.first.c_str())) continue;
            const std::string p = path.empty() ? kv.first : path + "." + kv.first;
            if (!IsCovered(covered, p)) out.push_back(p + " (gone)");
        }
        return;
    }
    if (IsArr(was) && IsArr(now)) {
        for (size_t i = 0; i < now.arr.size(); ++i) {
            const std::string key = ElemKey(now.arr[i], i);
            const std::string p = path.empty() ? key : path + "." + key;
            const JsonValue* b = nullptr;
            for (size_t k = 0; k < was.arr.size(); ++k) {
                if (ElemKey(was.arr[k], k) == key) {
                    b = &was.arr[k];
                    break;
                }
            }
            if (!b) {
                if (!IsCovered(covered, p)) out.push_back(p + " (new)");
                continue;
            }
            Residue(*b, now.arr[i], p, covered, out);
        }
        for (size_t i = 0; i < was.arr.size(); ++i) {
            const std::string key = ElemKey(was.arr[i], i);
            bool still = false;
            for (size_t k = 0; k < now.arr.size(); ++k) {
                if (ElemKey(now.arr[k], k) == key) {
                    still = true;
                    break;
                }
            }
            if (still) continue;
            const std::string p = path.empty() ? key : path + "." + key;
            if (!IsCovered(covered, p)) out.push_back(p + " (gone)");
        }
        return;
    }
    if (Text(was) != Text(now)) out.push_back(path);
}

bool SceneReload::Accept(const Schema& s, const JsonValue& section, const PropSet& before,
                         const std::string& path, PropSet& accepted, Verdict& v,
                         std::string* why) {
    // THE COMPLETE CANDIDATE: the DECLARED defaults (the schema's prototype), then this
    // document's keys over them. A key the document no longer carries is the default here, so
    // it appears in the diff as the change to its default that it is.
    accepted = PropSet::Defaults(s, s.Prototype());
    if (!accepted.Merge(Pick(section, s), path, why)) return false;
    for (const PropSet::Change& c : accepted.Diff(before)) {
        if (c.reload == Reload::Restart) {
            // NOT APPLIED, and put back: the set the run carries is the set the run is in.
            accepted.Take(before, c.key);
            ++v.restart;
            v.restartKeys.push_back(c.key);
            continue;
        }
        ++v.hot;
        v.hotKeys.push_back(c.key + "=" + Text(accepted.ValueAt(c.key)));
    }
    return true;
}

namespace {
void ListKeys(const Schema& s, const std::string& prefix, std::vector<std::string>& out) {
    for (const PropDecl& d : s.Decls()) {
        const std::string k = prefix.empty() ? d.key : prefix + "." + d.key;
        if (d.type == PropType::List) out.push_back(k);
        else if (d.type == PropType::Object && d.sub) ListKeys(*d.sub, k, out);
    }
}
}  // namespace

void SceneReload::Lists(const Schema& s, const JsonValue* was, const JsonValue& now,
                        const std::vector<std::string>& consumed, Verdict& v) {
    std::vector<std::string> keys;
    ListKeys(s, "", keys);
    for (const std::string& k : keys) {
        const JsonValue* a = was ? At(*was, k) : nullptr;
        const JsonValue* b = At(now, k);
        if ((a ? Text(*a) : std::string()) == (b ? Text(*b) : std::string())) continue;
        bool eats = false;
        for (const std::string& c : consumed) eats = eats || c == k;
        const std::string n = std::to_string(b && IsArr(*b) ? b->arr.size() : 0);
        if (eats) {
            ++v.hot;
            v.hotKeys.push_back(k + "=[" + n + " elements]");
        } else {
            ++v.restart;
            v.restartKeys.push_back(k + " (a list nothing reads live this run)");
        }
    }
}

// ---- the wiring ------------------------------------------------------------------------------

void SceneReload::Configure(Resolve resolve, const JsonValue& booted) {
    m_resolve = std::move(resolve);
    m_doc = booted;
}

void SceneReload::Watch(const std::string& path, WaterSceneWatch* adopt, long long* mtime) {
    for (const auto& w : m_files) {
        if (w->path == path) return;   // ONE WATCHER PER FILE
    }
    auto w = std::make_unique<Watched>();
    w->path = path;
    if (adopt) {
        // The water's, already started on data/wave_scene.json with its own mtime cell: the
        // reload drives it from here instead of starting a second thread on one file.
        w->watch = adopt;
        w->mtime = mtime ? mtime : &w->ownMtime;
    } else {
        w->own = std::make_unique<WaterSceneWatch>();
        w->own->Start(path.c_str());
        w->watch = w->own.get();
        w->mtime = &w->ownMtime;
        WaterSceneChanged(path.c_str(), &w->ownMtime);   // the boot-time stat, as the water's is
    }
    m_files.push_back(std::move(w));
}

void SceneReload::Add(Target t) {
    if (!t.schema) return;
    const Schema& s = *t.schema;
    m_covered.push_back(t.path);
    m_live.push_back(Live{std::move(t), PropSet(s)});
}

bool SceneReload::Ready(std::string* why) {
    for (Live& l : m_live) {
        const JsonValue* sec = At(m_doc, l.t.path);
        if (!sec) {
            return Refuse(why, l.t.path + ": no such section in the resolved scene");
        }
        PropSet boot = PropSet::Defaults(*l.t.schema, l.t.schema->Prototype());
        if (!boot.Merge(Pick(*sec, *l.t.schema), l.t.path, why)) return false;
        l.applied = boot;   // the set the run was BUILT from: the first diff's `before`
    }
    Rehash();
    std::string files;
    for (const auto& w : m_files) files += (files.empty() ? "" : ", ") + w->path;
    Log("[scene] hot reload armed: %zu files (%s), %zu live targets; state FNV-1a %016llx",
        m_files.size(), files.c_str(), m_live.size(),
        static_cast<unsigned long long>(m_hash));
    m_armed = true;
    return true;
}

void SceneReload::Stop() {
    for (auto& w : m_files) {
        if (w->own) w->own->Stop();
    }
    m_armed = false;
}

void SceneReload::Rehash() {
    uint64_t h = 14695981039346656037ull;
    for (const Live& l : m_live) {
        h = Fnv1aBytes(l.t.path.data(), l.t.path.size(), h);
        const std::string t = Text(l.applied.ToJson());
        h = Fnv1aBytes(t.data(), t.size(), h);
        for (const std::string& k : l.t.lists) {
            const JsonValue* v = At(m_doc, l.t.path + "." + k);
            const std::string lt = v ? Text(*v) : std::string("null");
            h = Fnv1aBytes(lt.data(), lt.size(), h);
        }
    }
    m_hash = h;
}

// ---- the reload ------------------------------------------------------------------------------

bool SceneReload::Poll(uint32_t frame) {
    if (!m_armed || !m_resolve) return false;
    // The watcher says the directory moved; the mtime says the FILE did. Both as 5c's, per file.
    std::string moved, trigger;
    for (auto& w : m_files) {
        if (!w->watch || !w->watch->Poll(frame)) continue;
        if (!WaterSceneChanged(w->path.c_str(), w->mtime)) continue;
        moved += (moved.empty() ? "" : ", ") + w->path;
        if (trigger.empty()) trigger = w->watch->Trigger();
    }
    if (moved.empty()) return false;
    // 5c's own line, generalized: WHICH file, at which frame, and the latency from the
    // directory's notification -- the "within one frame" the gate reads.
    Log("[scene] %s hot-reloaded at frame %u, %s", moved.c_str(), frame, trigger.c_str());

    // THE CANDIDATE IS THE WHOLE FOLD, run again from the boot's own inputs.
    SceneBuilder b;
    std::string why;
    if (!m_resolve(b, &why)) {
        Log("[scene] reload REFUSED at frame %u (%s): %s -- the previous scene stands, nothing "
            "applied", frame, moved.c_str(), why.c_str());
        return false;
    }
    const JsonValue cand = b.Resolved();

    // PASS 1: accept every target, or refuse the whole reload. A half-applied scene is the one
    // state this must never reach, so nothing is written until every section has parsed.
    std::vector<PropSet> accepted;
    std::vector<Verdict> verdicts(m_live.size());
    std::vector<bool> present(m_live.size(), true);
    accepted.reserve(m_live.size());
    for (size_t i = 0; i < m_live.size(); ++i) {
        Live& l = m_live[i];
        const JsonValue* sec = At(cand, l.t.path);
        if (!sec) {
            present[i] = false;
            accepted.push_back(l.applied);
            continue;
        }
        PropSet acc(*l.t.schema);
        if (!Accept(*l.t.schema, *sec, l.applied, l.t.path, acc, verdicts[i], &why)) {
            Log("[scene] reload REFUSED at frame %u (%s): %s -- the previous scene stands, "
                "nothing applied", frame, moved.c_str(), why.c_str());
            return false;
        }
        Lists(*l.t.schema, At(m_doc, l.t.path), *sec, l.t.lists, verdicts[i]);
        accepted.push_back(acc);
    }

    // PASS 2: report, then apply. The report is complete by construction -- what a target does
    // not carry, the residue walk names. The residue reads the OLD document; the apply reads the
    // candidate (a fan-out that consumes a list reads it from Document()).
    int hot = 0, restart = 0;
    for (size_t i = 0; i < m_live.size(); ++i) {
        const Live& l = m_live[i];
        if (!present[i]) {
            // A typed element is two targets at one path: one node, counted once.
            if (i > 0 && !present[i - 1] && m_live[i - 1].t.path == l.t.path) continue;
            ++restart;
            Log("[scene] %s is gone from the scene: needs a restart -- a live node is destroyed "
                "at shutdown, not at reload", l.t.path.c_str());
            continue;
        }
        for (const std::string& k : verdicts[i].restartKeys) {
            Log("[scene] %s.%s needs a restart: it is read where the object is built, and this "
                "run's is -- not applied", l.t.path.c_str(), k.c_str());
        }
        hot += verdicts[i].hot;
        restart += verdicts[i].restart;
    }
    std::vector<std::string> residue;
    Residue(m_doc, cand, "", m_covered, residue);
    for (const std::string& p : residue) {
        Log("[scene] %s changed: needs a restart -- no live target for it this run", p.c_str());
    }
    restart += static_cast<int>(residue.size());
    m_doc = cand;

    for (size_t i = 0; i < m_live.size(); ++i) {
        Live& l = m_live[i];
        const Verdict& v = verdicts[i];
        if (!present[i] || v.hot == 0) continue;
        std::string list;
        for (const std::string& k : v.hotKeys) list += (list.empty() ? "" : ", ") + k;
        Log("[scene] %s apply: %d hot -- %s", l.t.path.c_str(), v.hot, list.c_str());
        if (l.t.apply) {
            l.t.apply(accepted[i]);
        } else if (l.t.instance) {
            std::string aw;
            if (!accepted[i].ApplyTo(l.t.instance, l.t.frame, &aw)) {
                Log("[scene] %s apply refused: %s", l.t.path.c_str(), aw.c_str());
                continue;   // the previous set stands for THIS target; the rest still apply
            }
        }
        if (l.t.fanOut) l.t.fanOut();
        l.applied = accepted[i];
    }
    Rehash();
    ++m_reloads;
    Log("[scene] reload: %s %d fields changed (%d hot, %d restart) FNV-1a %016llx", moved.c_str(),
        hot + restart, hot, restart, static_cast<unsigned long long>(m_hash));
    return true;
}

}  // namespace ga::scene
