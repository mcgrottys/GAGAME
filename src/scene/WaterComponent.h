// ================================================================================================
//  WaterComponent - M12 step 5c: THE WATER SCENE'S FAN-OUT, SAID ONCE.
//
//  THE DEFECT THIS CLOSES. The M8 water scene (data/wave_scene.json: the solved wave field's
//  window, and the closures that used to be shader literals) is read into ONE WaterSceneConfig
//  and then handed to the layers BY HAND, in TWO places, with TWO DIFFERENT SETS:
//
//    boot   (FrameLoop::Session)  the bank's wave field and its live config pointer, the outer
//                                 level's bank likewise, the globe's four water fields, THE AUTO
//                                 EDIT FLOOR derived from the datum envelope, and the sea's four
//                                 closures
//    reload (FrameLoop::Frame)    the bank's wave field, the globe's four fields, and the jetty
//                                 floor ONLY when it is above the -90 sentinel -- no sea
//                                 closures, no auto floor
//
//  So an operator who edited windSeaFill and saved watched nothing happen until the next run,
//  and one who deleted jettyCrestNavd kept whatever floor the run was already carrying. The
//  component is that fan-out written ONCE: Apply(const PropSet&) is the BOOT set, it is
//  idempotent, and it is called at load and on every reload -- one path, so the two can never
//  again apply different things.
//
//  TWO DELIBERATE CHANGES land with it, and they are the step:
//    1. THE RELOAD APPLIES THE SAME SET AS BOOT (the plan states this one). A saved edit to
//       windSeaFill, bandFoldWeight, buoyAssimAgeH or buoyAssimGainMax now reaches the sea, and
//       a jettyCrestNavd at or below the -90 sentinel now recomputes the AUTO floor from the
//       datum envelope instead of leaving the floor where it was.
//    2. A KEY REMOVED FROM THE FILE REVERTS TO ITS DECLARED DEFAULT. The candidate is a COMPLETE
//       resolved set -- the struct defaults, then this file's keys over them (SceneBuilder.h's
//       reload law) -- not a patch onto the values the run happens to be carrying. "The current
//       value is the default" describes the read of one file over a RESOLVED BASE, not a memory.
//
//  IDEMPOTENCE is the property that makes one path safe: applying the same set twice writes the
//  same fields to the same values, recomputes the same floor, and rebuilds nothing (the solver's
//  re-Configure is gated on a CHANGE, not on a call). The [water] apply line carries the FNV-1a
//  of the applied set, so "the run came back to where it started" is a hash comparison.
//
//  HOT AND RESTART, and why each is what it is. bankTexelM is the one RESTART key: the ring
//  spans, the kernel's fold thresholds and the globe's gBankA.x all derive from it at
//  CONSTRUCTION (WaterBankLayer::SetBaseTexel must precede Init, and the Assembly calls it
//  there), so a changed value is REPORTED -- "needs a restart" -- and NOT applied. Every other
//  key is HOT, including the wave field's window, because that is what the engine does today:
//  the reload re-Configures the solver, whose bucket key rolls and whose next Update re-solves
//  or hits the cache (SceneConfig.h's banner: "a changed wavefield geometry rolls the solver's
//  bucket key"). What a window move does NOT do today, and does not do here, is move the wave
//  field's PAGE TENANT: WaveFieldSource and its z16 lattice were built from the boot window and
//  the bank's SetWavePages origin with them, so a run-time window move re-solves onto pages that
//  still sit where they were. That is a residue of the boot-time binding, recorded and logged
//  when it happens -- not something this step changes.
//
//  THE OBSERVERS. Configure(const Observers&) is the house Configure(const T*) idiom widened
//  once for this component: a plain struct of NULLABLE pointers -- the live config, the bank,
//  the outer bank, the sea, the globe, the wave field, the water atlas, the rebuilder, and the
//  file's watch. What is absent is REPORTED once at Configure and then skipped; nothing is
//  assumed. (scene::Wiring stays the generic plugin surface: it carries the device side, which
//  this component does not use.)
//
//  Prior art, named: this is the ordinary "apply a property set to live objects" of USD (a
//  composed value written through to the renderer's state) and Godot (`_set` on an exported
//  property, called identically when the scene loads and when the inspector changes it) -- the
//  only thing worth naming here is the discipline the engine lacked, which is that LOAD and
//  RELOAD are the same call.
// ================================================================================================
#pragma once

#include "core/SceneConfig.h"
#include "scene/Component.h"

#include <cstdint>
#include <string>
#include <vector>

namespace ga {
class GlobeLayer;
class SeaLayer;
class WaterAtlas;
class WaterBankLayer;
class WaveField;
}  // namespace ga

namespace ga::scene {

// THE TABLE: every WaterSceneConfig field by the key data/wave_scene.json uses, in the file's
// three sections, with the quantity and unit the field is in, the doc its declaration carries
// (SceneConfig.h's comments) and Hot|Restart. One per type (a Flyweight), built over a prototype
// whose values ARE the declared defaults -- which is what a key the file no longer carries
// reverts to. It agrees key for key, type for type, unit for unit and flag for flag with the
// `water` section's own table (SceneSchema.h's WavefieldSchema / ClosuresSchema / FleetSchema);
// [scene] holds them equal, because the two bind different structs and could otherwise drift.
const Schema& WaterSchema();

class WaterComponent final : public Component {
public:
    // What a hot change needs when a field write is not enough. The solved wave field's solver
    // re-Configures from objects the SESSION owns -- the compositor, the water atlas, the tide
    // and current models, the height channel -- which is construction, not a property fan-out,
    // so the component names the NEED and the session does the work.
    class Rebuild {
    public:
        virtual ~Rebuild() = default;
        // Re-Configure the solved wave field at this config's window (its bucket key rolls).
        virtual void ReconfigureWaveField(const WaterSceneConfig& cfg) = 0;
    };

    // The fan-out's subjects. Every one nullable: a chart run has no globe, a run without
    // --droste has no outer bank, a run whose height channel never registered has no wave field.
    struct Observers {
        WaterSceneConfig* config = nullptr;   // THE live config -- what the bank reads per frame
        WaterBankLayer* bank = nullptr;
        WaterBankLayer* bankB = nullptr;      // M10: the outer Droste level's rings (set B)
        SeaLayer* sea = nullptr;
        GlobeLayer* globe = nullptr;
        WaveField* waveField = nullptr;       // non-null exactly when the session built one
        const WaterAtlas* atlas = nullptr;    // the datum envelope the AUTO edit floor reads
        Rebuild* rebuild = nullptr;
        const char* path = nullptr;           // the scene file (Assembly::kScenePath)
        WaterSceneWatch* watch = nullptr;     // the directory watcher (Assembly::sceneWatch)
        long long* mtime = nullptr;           // the mtime the watcher's poll is checked against
    };

    // ---- Component -------------------------------------------------------------------------
    const char* Name() const override { return "water"; }
    const Schema& Props() const override { return WaterSchema(); }
    // The device side: the water component owns no GPU object of its own (the bank, the sea and
    // the globe are layers the Assembly builds), so nothing here is required.
    std::vector<std::string> Configure(const Wiring& w) override;
    bool Init(Gpu& gpu) override;
    // THE ONE FAN-OUT. Idempotent; called at load and on every reload.
    void Apply(const PropSet& props) override;
    void Update(const FrameInfo& f) override;
    void Record(const ViewContext& v) override;
    void ReloadShaders() override {}

    // ---- the wiring, the boot set and the reload ---------------------------------------------

    // The subjects. Returns what it was NOT given (logged here too, once).
    std::vector<std::string> Configure(const Observers& o);

    // The set the engine boots with: the live config's values, as the loader left them
    // (core/SceneConfig.h LoadWaterScene, which reads the file over the struct's defaults --
    // the same fold this component's reload does, by a reader that predates it).
    PropSet BootSet() const;

    // THE HOT RELOAD, whole. The watcher fired (or the no-watch cadence came round), the file's
    // mtime moved, the file resolves into a COMPLETE candidate over the declared defaults, the
    // candidate VALIDATES (an unknown key refuses, naming its path, and the previous set stays),
    // and the accepted diff goes through Apply. True when a set was applied.
    bool Reload(uint32_t frame);

    // The file (or a document already parsed) as a complete candidate: the declared defaults,
    // then this document's keys over them. `fleet` carries the boats, which the property table
    // declares as a LIST and a PropSet does not carry (5a's law: lists are the document's
    // business) -- they are resolved beside the set and staged into the same Apply.
    struct Fleet {
        bool staged = false;
        int count = 0;
        WaterSceneConfig::Boat boats[8];
    };
    bool Read(const char* path, PropSet& out, Fleet& fleet, std::string* why) const;
    bool ReadJson(const JsonValue& doc, PropSet& out, Fleet& fleet, std::string* why) const;
    // The boats the next Apply writes with its set (cleared by nothing: a repeat Apply writes
    // the same boats, which is what idempotent means).
    void StageFleet(const Fleet& fleet);

    // The set the last Apply applied, and its FNV-1a -- the instrument the reload probe reads.
    const PropSet& Applied() const { return m_applied; }
    uint64_t AppliedHash() const { return m_hash; }
    // The canonical text one set hashes: the declared keys in the table's order, one per line.
    static std::string SetText(const PropSet& set);
    static uint64_t SetHash(const PropSet& set);

private:
    void FanOut();                       // the boot fan-out, verbatim, over m_o
    float EditFloorNavd() const;         // the AUTO edit floor, as Session computes it

    Observers m_o;
    WaterSceneConfig m_defaults;         // the declared defaults: what a removed key reverts to
    PropSet m_applied{WaterSchema()};    // the last applied set (the diff's `before`)
    Fleet m_fleet;
    uint64_t m_hash = 0;
    bool m_have = false;                 // an Apply has run: the next one is a RELOAD
    bool m_wired = false;
};

}  // namespace ga::scene
