// WaterComponent - the table, the one Apply, and the reload law (M12 step 5c). See the header.
#include "scene/WaterComponent.h"

#include "compose/WaterAtlas.h"
#include "core/Common.h"
#include "core/Json.h"
#include "scene/GlobeLayer.h"
#include "scene/SceneBuilder.h"
#include "scene/SeaLayer.h"
#include "scene/WaterBankLayer.h"

#include <cstring>

namespace ga::scene {

namespace {

using Q = Quantity;
constexpr Reload H = Reload::Hot;
constexpr Reload R = Reload::Restart;

// THE PROTOTYPE: static storage, so the table's offsets and DefaultOf() read from a living
// instance for the program's life, and its values ARE the declared defaults. The three sections
// are Nested over the SAME prototype -- WaterSceneConfig is flat and the file is not, so a
// section is a VIEW of the struct, its own offsets folded through a zero-offset nest.
WaterSceneConfig kCfg;
WaterSceneConfig::Boat kBoat;

const Schema& WavefieldTable() {
    static const Schema* s = [] {
        Schema* sc = new Schema("water.wavefield", &kCfg);
        sc->Bind("enabled", kCfg.wfEnabled, "the solved wave field, blended inside its feathered window", H)
            .Bind("orgX", kCfg.wfOrgX, Q::Length, "m", "window origin, world x", H)
            .Bind("orgZ", kCfg.wfOrgZ, Q::Length, "m", "window origin, world z", H)
            .Bind("nx", kCfg.wfNx, Q::Dimensionless, "1", "cells east", H)
            .Bind("ny", kCfg.wfNy, Q::Dimensionless, "1", "cells north", H)
            .Bind("cellM", kCfg.wfCellM, Q::Length, "m", "cell size", H)
            .Bind("comps", kCfg.wfComps, Q::Dimensionless, "1",
                  "spectral components (M9bp: 32 -- sixteen 1.6 deg apart sum to a fixed "
                  "interference lattice, the comb of straight ridges down a storm face)", H)
            .Bind("spreadDeg", kCfg.wfSpreadDeg, Q::Angle, "deg", "directional spread", H)
            .Bind("barNormalDeg", kCfg.wfBarNormalDeg, Q::Angle, "deg",
                  "the entrance bar's normal, compass", H)
            .Bind("gammaHs", kCfg.wfGammaHs, Q::Dimensionless, "1",
                  "Hs <= gamma * h (the total limiter)", H)
            .Bind("minSamplesPerLambda", kCfg.wfMinSamplesPerLambda, Q::Dimensionless, "1",
                  "upload gate; the cascades keep the shorter bands", H)
            .Bind("tideBucketM", kCfg.wfTideBucketM, Q::Length, "m", "re-solve quantization", H)
            .Bind("currentBucketMs", kCfg.wfCurrentBucketMs, Q::Velocity, "m/s",
                  "re-solve quantization", H)
            .Bind("featherM", kCfg.wfFeatherM, Q::Length, "m",
                  "window edge blend into the cascades", H)
            .Bind("chop", kCfg.wfChop, Q::Dimensionless, "1",
                  "Gerstner horizontal displacement (the ambient sea's lambda; one look for both)", H)
            .Bind("exag", kCfg.wfExag, Q::Dimensionless, "1",
                  "solved-field DISPLAY exaggeration, applied to the dequant tables at upload "
                  "(the cache stays raw physics)", H)
            // THE ONE RESTART KEY: the ring spans, the kernel's fold thresholds, the 9b
            // emulator and the globe's gBankA.x all derive from it at CONSTRUCTION
            // (WaterBankLayer::SetBaseTexel, which must precede Init and does, in the Assembly).
            .Bind("bankTexelM", kCfg.bankTexelM, Q::Length, "m",
                  "ring-0 texel: caps which wavelengths become GEOMETRY (the fold law sheds any "
                  "band the ring undersamples); ring construction, so a change needs a restart", R);
        return sc;
    }();
    return *s;
}

const Schema& ClosuresTable() {
    static const Schema* s = [] {
        Schema* sc = new Schema("water.closures", &kCfg);
        sc->Bind("shedSteepCap", kCfg.shedSteepCap, Q::Dimensionless, "1",
                 "Miche: shed slope variance rides ak <= this", H)
            .Bind("shedMssCeil", kCfg.shedMssCeil, Q::Dimensionless, "1",
                  "storm-sea mss ceiling on the solved shed", H)
            .Bind("churnGain", kCfg.churnGain, Q::Dimensionless, "1",
                  "churn read gain (0 = off)", H)
            .Bind("crestLo", kCfg.crestLo, Q::Dimensionless, "1", "the crest gate, eta/rms units", H)
            .Bind("crestHi", kCfg.crestHi, Q::Dimensionless, "1", "the crest gate, eta/rms units", H)
            .Bind("depthLo", kCfg.depthLo, Q::Dimensionless, "1",
                  "the depth-excess trigger band", H)
            .Bind("depthHi", kCfg.depthHi, Q::Dimensionless, "1",
                  "the depth-excess trigger band", H)
            .Bind("foamOpacity", kCfg.foamOpacity, Q::Dimensionless, "1",
                  "peak foam opacity (a thin aerated layer, not paint)", H)
            .Bind("ringBlendTexels", kCfg.ringBlendTexels, Q::Dimensionless, "1",
                  "bank ring cross-fade width, in texels", H)
            .Bind("bandFoldWeight", kCfg.bandFoldWeight, Q::Dimensionless, "1",
                  "how far the FOLD's per-band wavelength follows the live spectrum instead of "
                  "the band's geometric midpoint (0 = the shipped constant, byte for byte)", H)
            .Bind("windSeaFill", kCfg.windSeaFill, Q::Dimensionless, "1",
                  "gain on the Pierson-Moskowitz wind sea synthesised for hours whose partitions "
                  "are all swell (0 = off, byte for byte)", H)
            .Bind("buoyAssimAgeH", kCfg.buoyAssimAgeH, Q::Time, "h",
                  "buoy Hs assimilation: max observation age", H)
            .Bind("buoyAssimGainMax", kCfg.buoyAssimGainMax, Q::Dimensionless, "1",
                  "and the gain clamp (past it the forecast and the buoy disagree about the WORLD)", H)
            .Bind("causticStrength", kCfg.causticStrength, Q::Dimensionless, "1",
                  "the bed dapple (0 = off)", H)
            .Bind("waterOptics", kCfg.waterOptics,
                  "K_d and the deep colour from the NOAA ocean-colour fields; false restores the "
                  "M7c constants byte for byte", H)
            .Bind("jettyCrestNavd", kCfg.jettyCrestNavd, Q::Length, "m NAVD88",
                  "edit-land geometry floor; <= -90 = AUTO, the datum envelope at the structure", H);
        return sc;
    }();
    return *s;
}

const Schema& BoatTable() {
    static const Schema* s = [] {
        Schema* sc = new Schema("water.fleet.boat", &kBoat);
        sc->Bind("speed", kBoat.speed, Q::Velocity, "m/s", "AIS per-class mean SOG", H)
            .Bind("halfLen", kBoat.halfLen, Q::Length, "m", "half length", H)
            .Bind("wakeAmp", kBoat.wakeAmp, Q::Length, "m", "wake amplitude", H)
            .Bind("offsetS", kBoat.offsetS, Q::Length, "m",
                  "starting arc-length offset along the lane", H)
            .Bind("dir", kBoat.dir, Q::Dimensionless, "1", "+1 outbound, -1 inbound", H);
        return sc;
    }();
    return *s;
}

const Schema& FleetTable() {
    static const Schema* s = [] {
        Schema* sc = new Schema("water.fleet", &kCfg);
        sc->Bind("enabled", kCfg.fleetEnabled, "the AIS-lane floats", H)
            .List("boats", &BoatTable(), "the boats, in lane order", false);
        return sc;
    }();
    return *s;
}

// The declared scalars of a table, counting through its nested sections (a List is not one).
int CountFields(const Schema& s) {
    int n = 0;
    for (const PropDecl& d : s.Decls()) {
        if (d.type == PropType::Object) n += d.sub ? CountFields(*d.sub) : 0;
        else if (d.type != PropType::List) ++n;
    }
    return n;
}

// A dotted key ("wavefield.bankTexelM") resolved to its declaration and its offset from the
// table's base -- the nests' offsets folded in, which for this table are zero because a section
// is a view of the same struct.
bool FindDotted(const Schema& s, const std::string& key, const PropDecl** out,
                std::ptrdiff_t* off) {
    const size_t dot = key.find('.');
    if (dot == std::string::npos) {
        const PropDecl* d = s.Find(key);
        if (!d) return false;
        *out = d;
        *off += d->offset;
        return true;
    }
    const PropDecl* d = s.Find(key.substr(0, dot));
    if (!d || d->type != PropType::Object || !d->sub) return false;
    *off += d->offset;
    return FindDotted(*d->sub, key.substr(dot + 1), out, off);
}

// One declared field's bytes, copied from `from` to `to` (the kinds this table uses).
void CopyField(const PropDecl& d, std::ptrdiff_t off, const void* from, void* to) {
    const char* a = static_cast<const char*>(from) + off;
    char* b = static_cast<char*>(to) + off;
    switch (d.field) {
        case Field::Bool: *reinterpret_cast<bool*>(b) = *reinterpret_cast<const bool*>(a); break;
        case Field::F32: *reinterpret_cast<float*>(b) = *reinterpret_cast<const float*>(a); break;
        case Field::F64: *reinterpret_cast<double*>(b) = *reinterpret_cast<const double*>(a); break;
        case Field::I32: *reinterpret_cast<int*>(b) = *reinterpret_cast<const int*>(a); break;
        case Field::U32:
            *reinterpret_cast<uint32_t*>(b) = *reinterpret_cast<const uint32_t*>(a);
            break;
        default: break;
    }
}

// One value as the text the [water] line prints: the number law's shortest round trip for the
// width the field actually is, the two spellings of a bool, a name for an enum.
std::string ValueText(const PropDecl& d, const PropValue& v) {
    switch (d.type) {
        case PropType::Bool: return v.b ? "true" : "false";
        case PropType::Number:
            return d.field == Field::F32 ? FloatText(static_cast<float>(v.n)) : NumberText(v.n);
        default: return v.s;
    }
}

// The value at a dotted key of a set (the sections are one level deep).
std::string ValueTextOf(const PropSet& set, const std::string& key) {
    const size_t dot = key.find('.');
    if (dot == std::string::npos) {
        const PropDecl* d = set.Type().Find(key);
        const PropValue* v = set.Get(key.c_str());
        return (d && v) ? ValueText(*d, *v) : std::string("?");
    }
    const PropSet* sub = set.Object(key.substr(0, dot).c_str());
    return sub ? ValueTextOf(*sub, key.substr(dot + 1)) : std::string("?");
}

// The wavefield keys the SOLVER reads (FrameLoop's sceneToWaveCfg): a change to one of these is
// what re-Configures the field, and the first five are the WINDOW, whose page tenant does not
// move at run time (the header's residue).
bool IsSolverKey(const std::string& key) {
    static const char* kKeys[] = {"wavefield.orgX", "wavefield.orgZ", "wavefield.nx",
                                  "wavefield.ny", "wavefield.cellM", "wavefield.comps",
                                  "wavefield.spreadDeg", "wavefield.barNormalDeg",
                                  "wavefield.gammaHs", "wavefield.minSamplesPerLambda",
                                  "wavefield.tideBucketM", "wavefield.currentBucketMs",
                                  "wavefield.featherM", "wavefield.exag"};
    for (const char* k : kKeys) {
        if (key == k) return true;
    }
    return false;
}
bool IsWindowKey(const std::string& key) {
    return key == "wavefield.orgX" || key == "wavefield.orgZ" || key == "wavefield.nx" ||
           key == "wavefield.ny" || key == "wavefield.cellM";
}

}  // namespace

const Schema& WaterSchema() {
    static const Schema* s = [] {
        Schema* sc = new Schema("water", &kCfg);
        sc->Nest("wavefield", WavefieldTable(), &kCfg, "the solved wave field's window")
            .Nest("closures", ClosuresTable(), &kCfg, "the kernel closures")
            .Nest("fleet", FleetTable(), &kCfg, "the AIS-lane floats");
        return sc;
    }();
    return *s;
}

// ---- wiring -------------------------------------------------------------------------------

std::vector<std::string> WaterComponent::Configure(const Wiring&) {
    return {};   // no device, no shader, no field table: the layers own all three
}

std::vector<std::string> WaterComponent::Configure(const Observers& o) {
    m_o = o;
    m_defaults = WaterSceneConfig{};
    m_applied = PropSet::Defaults(WaterSchema(), &m_defaults);
    m_hash = SetHash(m_applied);
    m_have = false;
    m_fleet = Fleet{};
    std::vector<std::string> missing;
    std::string have;
    auto note = [&](const void* p, const char* name) {
        if (p) have += (have.empty() ? "" : ", ") + std::string(name);
        else missing.push_back(name);
    };
    note(o.config, "config");
    note(o.bank, "bank");
    note(o.bankB, "bank B");
    note(o.sea, "sea");
    note(o.globe, "globe");
    note(o.waveField, "wave field");
    note(o.atlas, "atlas");
    note(o.rebuild, "rebuild");
    note((o.watch && o.path && o.mtime) ? static_cast<const void*>(o.watch) : nullptr, "watch");
    std::string absent;
    for (const std::string& m : missing) absent += (absent.empty() ? "" : ", ") + m;
    Log("[water] wired: %s%s%s", have.empty() ? "nothing" : have.c_str(),
        absent.empty() ? "" : "; NOT wired (reported, never assumed): ",
        absent.empty() ? "" : absent.c_str());
    m_wired = true;
    return missing;
}

bool WaterComponent::Init(Gpu&) { return m_wired && m_o.config != nullptr; }
void WaterComponent::Update(const FrameInfo&) {}
void WaterComponent::Record(const ViewContext&) {}

// ---- the set ------------------------------------------------------------------------------

PropSet WaterComponent::BootSet() const {
    return PropSet::Defaults(WaterSchema(), m_o.config ? m_o.config : &m_defaults);
}

std::string WaterComponent::SetText(const PropSet& set) {
    return SceneBuilder::WriteJson(set.ToJson());
}

uint64_t WaterComponent::SetHash(const PropSet& set) {
    const std::string t = SetText(set);
    return Fnv1aBytes(t.data(), t.size());
}

void WaterComponent::StageFleet(const Fleet& fleet) { m_fleet = fleet; }

// ---- THE ONE FAN-OUT ------------------------------------------------------------------------

float WaterComponent::EditFloorNavd() const {
    // M8g THE ORIGIN PLANES: the edit-land geometry floor comes from the datum envelope
    // (MLLW + margin at the structure), not a tide-relative constant -- the old floor tracked
    // the live waterline, which made the jetty unsinkable.
    float floorNavd = m_o.config->jettyCrestNavd;
    if (floorNavd <= -90.0f) {
        floorNavd = 1.8f;
        if (m_o.atlas && m_o.atlas->Ready()) {
            float elo = 0.0f, ehi = 0.0f;
            m_o.atlas->EnvelopeNavd(42.8190, -70.8031, 0.0, &elo, &ehi);
            // Anchored to the TOP plane: a decayed structure is awash at spring high but a
            // continuous ridge below mid-tide. (lo + margin was the first draft -- that floors
            // at MLLW, which rescues the smear only at dead low.) Surveyed crests taller than
            // the floor still win.
            floorNavd = ehi - 0.45f;
            Log("[datum] envelope at north jetty: lo %+.2f hi %+.2f m NAVD "
                "(synodic-month min/max) -> edit floor %+.2f",
                elo, ehi, floorNavd);
        }
    }
    return floorNavd;
}

void WaterComponent::FanOut() {
    const WaterSceneConfig& cfg = *m_o.config;
    // The bank reads the LIVE config every frame, so an edit lands on the next recompose; the
    // wave field is what it blends inside the window (null when the scene turns it off). Set B
    // is the outer Droste level's ladder -- the same solve, the same closures, its own rings.
    // Gated on the wave field because that is the block boot builds them in: no field, no
    // bank wiring, exactly as before.
    if (m_o.bank && m_o.waveField) {
        m_o.bank->SetWaveField(cfg.wfEnabled ? m_o.waveField : nullptr);
        m_o.bank->SetScene(m_o.config);
        if (m_o.bankB) {
            m_o.bankB->SetWaveField(cfg.wfEnabled ? m_o.waveField : nullptr);
            m_o.bankB->SetScene(m_o.config);
        }
    }
    if (m_o.globe) {
        m_o.globe->foamOpacity = cfg.foamOpacity;
        m_o.globe->ringBlendTexels = cfg.ringBlendTexels;
        m_o.globe->causticStrength = cfg.causticStrength;
        m_o.globe->waterOptics = cfg.waterOptics;
        m_o.globe->editFloorNavd = EditFloorNavd();
    }
    if (m_o.sea) {
        m_o.sea->windSeaFill = cfg.windSeaFill;
        m_o.sea->bandFoldWeight = cfg.bandFoldWeight;
        m_o.sea->buoyAssimAgeH = cfg.buoyAssimAgeH;
        m_o.sea->buoyAssimGainMax = cfg.buoyAssimGainMax;
    }
}

void WaterComponent::Apply(const PropSet& props) {
    if (!m_o.config) {
        Log("[water] apply: no live config wired -- nothing applied");
        return;
    }
    const Schema& table = WaterSchema();
    const PropSet before = m_applied;

    // 1. The candidate, written onto a copy so a refusal leaves the run untouched.
    WaterSceneConfig cand = *m_o.config;
    std::string why;
    if (!props.ApplyTo(&cand, nullptr, &why)) {
        Log("[water] apply REFUSED: %s -- the previous set stands", why.c_str());
        return;
    }
    if (m_fleet.staged) {
        cand.fleetCount = m_fleet.count;
        for (int i = 0; i < 8; ++i) cand.fleet[i] = m_fleet.boats[i];
    }

    // 2. THE RESTART LAW. A Restart-flagged key that changed is REPORTED and NOT applied -- it
    //    is read where the object is built, and this run's object is already built. The BOOT
    //    set is exempt by construction: the session built the bank and the field with it.
    const std::vector<PropSet::Change> asked = props.Diff(before);
    bool solverMoved = false, windowMoved = false;
    for (const PropSet::Change& c : asked) {
        if (c.reload == Reload::Restart) {
            if (!m_have) continue;   // the boot set: applied at construction, not here
            const PropDecl* d = nullptr;
            std::ptrdiff_t off = 0;
            if (FindDotted(table, c.key, &d, &off)) CopyField(*d, off, m_o.config, &cand);
            Log("[water] %s needs a restart: it is read where the object is built, and this "
                "run's is -- not applied", c.key.c_str());
            continue;
        }
        solverMoved = solverMoved || IsSolverKey(c.key);
        windowMoved = windowMoved || IsWindowKey(c.key);
    }

    // 3. Commit, then the fan-out -- in Session's own order: the solver first (a moved window
    //    rolls its bucket key), then the bank, the globe and the sea.
    *m_o.config = cand;
    if (m_have && solverMoved && m_o.rebuild && m_o.waveField) {
        m_o.rebuild->ReconfigureWaveField(cand);
        if (windowMoved) {
            Log("[water] the wave field's PAGES keep the boot window: WaveFieldSource and the "
                "bank's page origin were bound at construction, so the re-solve lands on the "
                "lattice the run started with (restart to move the pages)");
        }
    }
    FanOut();

    // 4. The instrument: what was applied, what moved, and the set's fingerprint.
    m_applied = PropSet::Defaults(table, m_o.config);
    m_hash = SetHash(m_applied);
    m_have = true;
    const std::vector<PropSet::Change> moved = m_applied.Diff(before);
    std::string list;
    for (const PropSet::Change& c : moved) {
        list += (list.empty() ? "" : ", ") + c.key + "=" + ValueTextOf(m_applied, c.key);
    }
    Log("[water] apply: %d fields (%d changed)%s%s | set FNV-1a %016llx",
        CountFields(table), static_cast<int>(moved.size()), moved.empty() ? "" : ": ",
        list.c_str(), static_cast<unsigned long long>(m_hash));
}

// ---- the reload law --------------------------------------------------------------------------

bool WaterComponent::ReadJson(const JsonValue& doc, PropSet& out, Fleet& fleet,
                              std::string* why) const {
    // THE RELOAD LAW: a COMPLETE candidate -- the declared defaults, then THIS document's keys
    // over them. A key the document no longer carries reverts to its declared default, never to
    // the value the run happens to be carrying.
    out = PropSet::Defaults(WaterSchema(), &m_defaults);
    if (!out.Merge(doc, "water", why)) return false;
    // The boats are a LIST, which a PropSet does not carry (5a: lists are the document's
    // business). Same law: the declared fleet, then this document's boats over it, each
    // element's absent keys from the element table's prototype.
    fleet = Fleet{};
    fleet.staged = true;
    fleet.count = m_defaults.fleetCount;
    for (int i = 0; i < 8; ++i) fleet.boats[i] = m_defaults.fleet[i];
    if (const JsonValue* fl = doc.Get("fleet")) {
        if (const JsonValue* bs = fl->Get("boats")) {
            if (bs->type != JsonValue::Type::Array) {
                if (why) *why = "water.fleet.boats: expected an array";
                return false;
            }
            fleet.count = 0;
            for (const JsonValue& b : bs->arr) {
                if (fleet.count >= 8) break;
                WaterSceneConfig::Boat& o = fleet.boats[fleet.count++];
                o = WaterSceneConfig::Boat{};
                o.speed = b.Num("speed", o.speed);
                o.halfLen = b.Num("halfLen", o.halfLen);
                o.wakeAmp = b.Num("wakeAmp", o.wakeAmp);
                o.offsetS = b.Num("offsetS", o.offsetS);
                o.dir = (b.Num("dir", o.dir) < 0.0) ? -1 : 1;
            }
        }
    }
    return true;
}

bool WaterComponent::Read(const char* path, PropSet& out, Fleet& fleet, std::string* why) const {
    std::string text;
    if (!SceneBuilder::ReadFile(path, text, why)) return false;
    std::string err;
    const JsonValue v = JsonParser::Parse(text, &err);
    if (!err.empty()) {
        if (why) *why = std::string(path) + ": " + err;
        return false;
    }
    return ReadJson(v, out, fleet, why);
}

bool WaterComponent::Reload(uint32_t frame) {
    if (!m_o.config || !m_o.path || !m_o.watch || !m_o.mtime) return false;
    // Step 3 (docs/PERF_EXPERIMENT.md): the stat runs only when the directory watcher says the
    // file moved (every 30 frames with no watch); the mtime compare is unchanged.
    if (!m_o.watch->Poll(frame)) return false;
    if (!WaterSceneChanged(m_o.path, m_o.mtime)) return false;
    PropSet cand(WaterSchema());
    Fleet fleet;
    std::string why;
    if (!Read(m_o.path, cand, fleet, &why)) {
        Log("[scene] %s REFUSED at frame %u: %s -- the previous set stands, nothing applied",
            m_o.path, frame, why.c_str());
        return false;
    }
    StageFleet(fleet);
    Log("[scene] %s hot-reloaded at frame %u, %s", m_o.path, frame,
        m_o.watch->Trigger().c_str());
    Apply(cand);
    return true;
}

}  // namespace ga::scene
