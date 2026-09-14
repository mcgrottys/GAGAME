// ================================================================================================
//  SceneReload - M12 step 5f: THE WHOLE SCENE HOT-RELOADS, BY THE LAW 5c WROTE FOR THE WATER.
//
//  WHAT 5c ESTABLISHED, FOR ONE FILE. data/wave_scene.json is watched; when it moves, the
//  component resolves a COMPLETE candidate over the declared defaults, validates it, holds back
//  the Restart keys and applies the accepted diff through the SAME Apply the boot used
//  (scene/WaterComponent.h). Three properties made that safe: the candidate is complete (so a
//  key the file no longer carries reverts to its DECLARED DEFAULT, never to the value the run
//  happens to be carrying), the apply is idempotent, and load and reload are one path.
//
//  WHAT THIS GENERALIZES. The scene is not one file: it is a FOLD over a chain -- the struct
//  defaults, the base scene (and the scene IT inherits), each `include` overlay in order, then
//  every `--set` in command-line order (scene/SceneBuilder.h) -- plus the rail the session flies
//  from scenes/rails/<active>.json. So:
//
//    THE WATCH      one watcher per FILE in that chain, and the water's existing watcher is
//                   ADOPTED rather than duplicated (it already watches data/wave_scene.json,
//                   and two threads on one file is two answers to one question).
//    THE CANDIDATE  not the file that moved -- the WHOLE fold, run again from the boot's own
//                   inputs. The session hands in the closure that knows them (app::BuildScene
//                   over the parsed flags), so a reload cannot resolve a different fold from
//                   the boot's, and an edit to an overlay composes with its base exactly as it
//                   did at boot.
//    THE VALIDATION SceneBuilder::Resolve's, whole: an unknown key, a wrong type, a unit
//                   refusal or an unknown node type REFUSES THE RELOAD -- the previous scene
//                   stands, nothing is applied, and the log names the path. A half-applied
//                   scene is the one state this must never reach.
//    THE DIFF       per TARGET: a live struct, its Schema, and the push that carries the
//                   written fields to the objects that are not that struct. The accepted set is
//                   diffed against the last ACCEPTED set (not against the file), so a removed
//                   key shows up as the change to its default that it is.
//    THE RESTART    a Restart-flagged key that changed is REPORTED and NOT applied, and its
//                   previous value is put back into the accepted set, so the set the run
//                   carries is always the set the run is actually in.
//    THE RESIDUE    every path NOT under a target is walked structurally and reported as
//                   needing a restart, naming itself. That is what makes the report complete:
//                   a change is applied, or it is named. A NEW node or list element, and a
//                   REMOVED one, are reported this way too -- instantiating and destroying at
//                   run time is a follow-on step (it needs the Assembly's construction order,
//                   its device objects and the residency manager, none of which a property
//                   write can stand in for), and until it exists the honest answer is "needs a
//                   restart", not a silent nothing.
//
//  ONE LINE PER RELOAD: `[scene] reload: <file> <n> fields changed (<k> hot, <r> restart)
//  FNV-1a <hash>`. The hash is the STATE's, not the document's: the canonical text of every
//  target's accepted set, in registration order. So "the run came back to where it started"
//  is a comparison against the line the boot printed, which is what the reload probe reads.
//
//  WHAT A TARGET COSTS, AND WHY Hot IS NOT A PROMISE ON ITS OWN. A key is applied only where a
//  live target covers its path; `Hot` in the table says the ENGINE can take the change without
//  rebuilding, and a target says THIS RUN has somewhere to put it. Where the two disagree the
//  log says which -- "needs a restart: no live target" -- rather than writing a field nothing
//  reads and calling it applied.
//
//  Prior art, named: this is the ordinary asset hot-reload of Godot (a .tscn re-imported and the
//  live nodes' exported properties re-set) and USD's stage recomposition (an edited layer
//  recomposes the stage and the renderer re-reads the composed values; USD likewise distinguishes
//  what can be re-authored live from what forces a re-stage). The engine's own contribution is
//  the completeness of the report, which comes from the property table carrying the Hot|Restart
//  flag in the first place.
// ================================================================================================
#pragma once

#include "core/Json.h"
#include "core/SceneConfig.h"
#include "scene/Props.h"
#include "scene/SceneBuilder.h"

#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <vector>

namespace ga::scene {

class SceneReload {
public:
    // The boot's own resolution, run again. False with `why` = the candidate is refused.
    using Resolve = std::function<bool(SceneBuilder&, std::string*)>;
    // How an accepted set reaches the run. Null = write the bound fields of `instance`.
    using Apply = std::function<void(const PropSet&)>;

    // One live subject of the scene: the struct a section's (or an element's) schema addresses,
    // and the push that makes the write reach whatever is not that struct.
    struct Target {
        std::string path;                  // "water", "views.sea", "effects.cutaway"
        const Schema* schema = nullptr;
        void* instance = nullptr;          // the LIVE struct (S.water, m_startView.p, ...)
        Apply apply;                       // null = ApplyTo(instance)
        std::function<void()> fanOut;      // after the write: push into what is not the struct
        // The LIST keys (dotted, relative to `path`) the fan-out reads from Document(). A
        // PropSet carries no list, so a list under a target is compared as a VALUE: Hot where it
        // is named here, reported as needing a restart where it is not.
        std::vector<std::string> lists;
        const PoseFrame* frame = nullptr;  // for a placement key; null = `at` cannot resolve
    };

    SceneReload() = default;
    SceneReload(const SceneReload&) = delete;
    SceneReload& operator=(const SceneReload&) = delete;
    ~SceneReload() { Stop(); }

    // The inputs (the boot's fold) and the document the boot actually resolved.
    void Configure(Resolve resolve, const JsonValue& booted);
    // One watcher per file. `adopt` is an already-started watcher for this path (the water's),
    // with the mtime cell its owner keeps; null starts one here.
    void Watch(const std::string& path, WaterSceneWatch* adopt = nullptr,
               long long* mtime = nullptr);
    void Add(Target t);
    // Reads each target's set out of the booted document and logs the armed line. False with
    // `why` when a target's own path does not read back (a wiring error, not a file error).
    bool Ready(std::string* why);
    // Once a frame, from the frame loop. True when a candidate was accepted and applied.
    bool Poll(uint32_t frame);
    void Stop();

    // The state's fingerprint: every target's accepted set and consumed lists, in registration
    // order. During a reload's apply pass Document() is already the CANDIDATE, so a fan-out
    // reads the lists it consumes from the document being applied.
    uint64_t StateHash() const { return m_hash; }
    int Reloads() const { return m_reloads; }
    size_t Files() const { return m_files.size(); }
    size_t Targets() const { return m_live.size(); }
    const JsonValue& Document() const { return m_doc; }

    // ---- the pure laws, exported for scenetest ---------------------------------------------
    // The value at a dotted path: an object key, or a NAMED array element ("views.sea").
    static const JsonValue* At(const JsonValue& doc, const std::string& path);
    // `obj` reduced to the keys `s` declares (a typed element answers to two schemas; each
    // target reads its own half, and the document as a whole was validated against both).
    static JsonValue Pick(const JsonValue& obj, const Schema& s);
    // Every leaf path where `now` differs from `was`, skipping the subtrees `covered` names.
    // An array element is addressed by its `name` where it carries one and by its index where
    // it does not -- which is exactly the named/unnamed split the merge law draws.
    static void Residue(const JsonValue& was, const JsonValue& now, const std::string& path,
                        const std::vector<std::string>& covered, std::vector<std::string>& out);
    // THE DIFF AND APPLY OF ONE TARGET, pure enough to test on an in-memory document: the
    // accepted set (Restart keys held back at their previous values), and the two counts.
    struct Verdict {
        int hot = 0, restart = 0;
        std::vector<std::string> hotKeys, restartKeys;
    };
    static bool Accept(const Schema& s, const JsonValue& section, const PropSet& before,
                       const std::string& path, PropSet& accepted, Verdict& v, std::string* why);
    // The lists under a schema (recursing through its objects), as dotted relative keys, whose
    // value differs between two sections; `consumed` splits them into hot and restart.
    static void Lists(const Schema& s, const JsonValue* was, const JsonValue& now,
                      const std::vector<std::string>& consumed, Verdict& v);

private:
    struct Watched {
        std::string path;
        WaterSceneWatch* watch = nullptr;             // adopted or owned
        std::unique_ptr<WaterSceneWatch> own;
        long long* mtime = nullptr;                   // an adopted cell, or &ownMtime
        long long ownMtime = 0;
    };
    struct Live {
        Target t;
        PropSet applied;
    };

    void Rehash();

    Resolve m_resolve;
    JsonValue m_doc;                                  // the document the run is in
    std::vector<std::unique_ptr<Watched>> m_files;
    std::vector<Live> m_live;
    std::vector<std::string> m_covered;               // the target paths, for Residue
    uint64_t m_hash = 0;
    int m_reloads = 0;
    bool m_armed = false;
};

}  // namespace ga::scene
