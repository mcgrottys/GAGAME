// ================================================================================================
//  SceneConfig - M8: THE WATER SCENE AS DATA. The user's ask, verbatim: "create a scene config
//  so we can hot swap datasets and types without recompiling."
//
//  data/wave_scene.json configures the solved wave field's window (move it to another inlet by
//  editing four numbers), its spectrum/bucket parameters, and the water closures that were
//  literals in the shaders (shed caps, foam bands, churn gain, chop, foam opacity). The file is
//  AUTHORED IF ABSENT and never clobbered (the M6p hand-edits law); it hot-reloads on mtime
//  change every frame -- edit, save, watch the water change. A changed wavefield geometry rolls
//  the solver's bucket key, so the re-solve (or its cache hit) follows automatically.
//
//  This is the water leg of the program-file doctrine (bed_rules.json, edits.geojson, the
//  Blueprint contract): programs in data files, identity follows content, engine restarts are
//  for CODE. Stack composition (which height/color sources, in what order) is the named
//  follow-on -- it lives in main today because source construction still needs C++.
// ================================================================================================
#pragma once

#include <cstdio>
#include <string>
#include <sys/stat.h>

#include "core/Common.h"
#include "core/Json.h"

namespace ga {

struct WaterSceneConfig {
    // ---- the solved wave field's window (world metres, BathyModel frame) ----
    bool wfEnabled = true;
    double wfOrgX = -1400.0, wfOrgZ = -800.0;
    int wfNx = 1600, wfNy = 1000;
    double wfCellM = 2.0;
    int wfComps = 16;
    double wfSpreadDeg = 26.0;
    double wfBarNormalDeg = 285.0;      // the entrance bar's normal, compass
    double wfGammaHs = 0.60;            // Hs <= gamma * h (the total limiter)
    double wfMinSamplesPerLambda = 8.0; // upload gate; the cascades keep shorter bands
    double wfTideBucketM = 0.25;        // re-solve quantization
    double wfCurrentBucketMs = 0.10;
    float wfFeatherM = 120.0f;          // window edge blend into the cascades
    float wfChop = 1.1f;                // Gerstner horizontal displacement (the ambient
                                        // sea's lambda; one look for both)
    float wfExag = 1.15f;               // M8g: solved-field DISPLAY exaggeration, applied
                                        // to the dequant tables at upload (cache stays
                                        // raw physics; ProbeAt reads the same table so
                                        // the 9b/9c twin holds). vqview shipped 1.15.
    float bankTexelM = 1.2f;            // M9c: 1.2, not 2.4. The fold law
                                        // sheds any band the ring undersamples, so this
                                        // caps which wavelengths become GEOMETRY, and a band
                                        // contributes nothing at all until texel < lambda/2.
                                        // 4.8 half-folded the 16 m solved comps; 2.4 left the
                                        // CHOP band at exactly zero even with the M9c
                                        // energy-weighted fold wavelength (3.55 m needs
                                        // < 1.77 m); 1.2 matches the mesh's own ~1.19 m
                                        // vertex spacing, so neither side wastes the other,
                                        // and puts 39% of the chop into geometry.
                                        // Construction-time: needs a restart, not a
                                        // hot-reload. Ring 0 span = 512 * this.

    // ---- kernel closures (were literals; every one pinned by gates/proofs) ----
    float shedSteepCap = 0.44f;   // Miche: shed slope variance rides ak <= this
    float shedMssCeil = 0.09f;    // storm-sea mss ceiling on the solved shed
    float churnGain = 0.12f;      // churn read gain (0 = off); a whisper until the deposit-side discipline lands
    float crestLo = 0.28f, crestHi = 0.80f;   // the crest gate, eta/rms units
    float depthLo = 1.05f, depthHi = 1.95f;   // the depth-excess trigger band

    // ---- shading ----
    float foamOpacity = 0.72f;    // peak foam opacity (a thin aerated layer, not paint)
    float ringBlendTexels = 48.0f;   // bank ring cross-fade width (the interpolation
                                     // that keeps ring handovers from tiling visibly)
    float bandFoldWeight = 1.0f;     // M9c: how far the FOLD's per-band wavelength follows
                                     // the live spectrum instead of the band's geometric
                                     // midpoint. Cascade 2 spans lambda 0.41..12 m; judging
                                     // a 4.6 m wind sea as 2.2 m throws it out of geometry.
                                     // 0 = the shipped constant, byte for byte.
    float windSeaFill = 1.0f;        // M9a: gain on the Pierson-Moskowitz wind sea
                                     // synthesised from the GFS wind on hours where the
                                     // forecast's partitioning reports NO wind sea. Those
                                     // hours hand back swell only -- a ~4 mHz Gaussian with
                                     // no tail -- so cascades 1-2 realize EXACTLY zero and
                                     // the inlet renders as glass. 0 = off, byte for byte.
    float buoyAssimAgeH = 6.0f;      // buoy Hs assimilation: max observation age
    float buoyAssimGainMax = 1.8f;   // and the gain clamp (past it the forecast and
                                     // the buoy disagree about the WORLD)
    float causticStrength = 0.6f;    // 0 = off; the bed dapple, softened by default
    bool waterOptics = true;         // M9: drive K_d and the deep colour from the NOAA
                                     // ocean-colour fields (docs/ALGEBRA.md "optics").
                                     // false restores the M7c constants byte for byte --
                                     // the A/B, and the fallback when the fields are absent
    float jettyCrestNavd = -99.0f;   // M8g: edit-land geometry floor, NAVD m.
                                     // <= -90 = AUTO: datum envelope MLLW + 0.35 at the
                                     // structure (the origin planes decide; high water
                                     // then drowns the outer jetty as it should)

    // ---- the fleet (M8 floats): boats shuttling the AIS lane, ping-pong at the ends.
    // Classes carry the AIS climatology's mean speeds; wake character follows k = g/U^2.
    struct Boat {
        double speed = 4.86;      // m/s (AIS per-class mean SOG)
        double halfLen = 7.5;     // m
        double wakeAmp = 0.55;    // m
        double offsetS = 0.0;     // starting arc-length offset along the lane
        int dir = 1;              // +1 outbound (toward the sea), -1 inbound
    };
    bool fleetEnabled = false;   // floats stand down for now: waves and flows first
    int fleetCount = 5;
    Boat fleet[8] = {
        {4.86, 7.5, 0.55, 400.0, 1},     // the recreational hero
        {4.62, 9.0, 0.47, 1300.0, 1},    // fishing
        {3.83, 13.0, 0.63, 2100.0, -1},  // tug, inbound
        {3.34, 6.0, 0.38, 900.0, 1},     // commercial skiff
        {4.86, 6.5, 0.44, 1800.0, -1},   // second recreational, inbound
    };
};

inline void WriteDefaultWaterScene(const char* path) {
    FILE* f = fopen(path, "wb");
    if (!f) return;
    const char* text =
        "{\n"
        "  \"_readme\": [\"The water scene as data (M8): the solved wave field's window and\",\n"
        "              \"the water closures. Hot-reloads on save; a changed window rolls the\",\n"
        "              \"solver's cache key. Closures are engineering constants pinned by\",\n"
        "              \"gates -- change them against evidence (ALGEBRA.md, priors #12).\"],\n"
        "  \"wavefield\": {\n"
        "    \"enabled\": true,\n"
        "    \"orgX\": -1400.0, \"orgZ\": -800.0,\n"
        "    \"nx\": 1600, \"ny\": 1000, \"cellM\": 2.0,\n"
        "    \"comps\": 16, \"spreadDeg\": 26.0, \"barNormalDeg\": 285.0,\n"
        "    \"gammaHs\": 0.60, \"minSamplesPerLambda\": 8.0,\n"
        "    \"tideBucketM\": 0.25, \"currentBucketMs\": 0.10,\n"
        "    \"featherM\": 120.0, \"chop\": 1.1, \"exag\": 1.15,\n"
        "    \"bankTexelM\": 1.2\n"
        "  },\n"
        "  \"closures\": {\n"
        "    \"shedSteepCap\": 0.44, \"shedMssCeil\": 0.09,\n"
        "    \"churnGain\": 0.12,\n"
        "    \"crestLo\": 0.28, \"crestHi\": 0.80,\n"
        "    \"depthLo\": 1.05, \"depthHi\": 1.95,\n"
        "    \"foamOpacity\": 0.72,\n"
        "    \"ringBlendTexels\": 48.0,\n"
        "    \"windSeaFill\": 1.0,\n"
        "    \"bandFoldWeight\": 1.0,\n"
        "    \"buoyAssimAgeH\": 6.0, \"buoyAssimGainMax\": 1.8,\n"
        "    \"causticStrength\": 0.6,\n"
        "    \"waterOptics\": true,\n"
        "    \"jettyCrestNavd\": -99.0\n"
        "  },\n"
        "  \"fleet\": {\n"
        "    \"enabled\": false,\n"
        "    \"boats\": [\n"
        "      {\"speed\": 4.86, \"halfLen\": 7.5,  \"wakeAmp\": 0.55, \"offsetS\": 400.0,  \"dir\": 1},\n"
        "      {\"speed\": 4.62, \"halfLen\": 9.0,  \"wakeAmp\": 0.47, \"offsetS\": 1300.0, \"dir\": 1},\n"
        "      {\"speed\": 3.83, \"halfLen\": 13.0, \"wakeAmp\": 0.63, \"offsetS\": 2100.0, \"dir\": -1},\n"
        "      {\"speed\": 3.34, \"halfLen\": 6.0,  \"wakeAmp\": 0.38, \"offsetS\": 900.0,  \"dir\": 1},\n"
        "      {\"speed\": 4.86, \"halfLen\": 6.5,  \"wakeAmp\": 0.44, \"offsetS\": 1800.0, \"dir\": -1}\n"
        "    ]\n"
        "  }\n"
        "}\n";
    fwrite(text, 1, strlen(text), f);
    fclose(f);
    Log("[scene] authored default %s (edit + save to hot-swap; never clobbered)", path);
}

// Load (authoring the default first if absent). Returns false only on a parse error, in
// which case `out` keeps its previous values -- a broken edit must never flat-line the sea.
inline bool LoadWaterScene(const char* path, WaterSceneConfig& out) {
    {
        struct _stat64 st;
        if (_stat64(path, &st) != 0) WriteDefaultWaterScene(path);
    }
    FILE* f = fopen(path, "rb");
    if (!f) return false;
    std::string text;
    char buf[4096];
    size_t n;
    while ((n = fread(buf, 1, sizeof(buf), f)) > 0) text.append(buf, n);
    fclose(f);
    std::string err;
    const JsonValue v = JsonParser::Parse(text, &err);
    if (!err.empty()) {
        Log("[scene] %s parse error (%s) -- keeping the previous config", path, err.c_str());
        return false;
    }
    if (const JsonValue* w = v.Get("wavefield")) {
        const JsonValue* en = w->Get("enabled");
        out.wfEnabled = !en || en->type != JsonValue::Type::Bool || en->boolean;
        out.wfOrgX = w->Num("orgX", out.wfOrgX);
        out.wfOrgZ = w->Num("orgZ", out.wfOrgZ);
        out.wfNx = static_cast<int>(w->Num("nx", out.wfNx));
        out.wfNy = static_cast<int>(w->Num("ny", out.wfNy));
        out.wfCellM = w->Num("cellM", out.wfCellM);
        out.wfComps = static_cast<int>(w->Num("comps", out.wfComps));
        out.wfSpreadDeg = w->Num("spreadDeg", out.wfSpreadDeg);
        out.wfBarNormalDeg = w->Num("barNormalDeg", out.wfBarNormalDeg);
        out.wfGammaHs = w->Num("gammaHs", out.wfGammaHs);
        out.wfMinSamplesPerLambda = w->Num("minSamplesPerLambda", out.wfMinSamplesPerLambda);
        out.wfTideBucketM = w->Num("tideBucketM", out.wfTideBucketM);
        out.wfCurrentBucketMs = w->Num("currentBucketMs", out.wfCurrentBucketMs);
        out.wfFeatherM = static_cast<float>(w->Num("featherM", out.wfFeatherM));
        out.wfChop = static_cast<float>(w->Num("chop", out.wfChop));
        out.wfExag = static_cast<float>(w->Num("exag", out.wfExag));
        out.bankTexelM = static_cast<float>(w->Num("bankTexelM", out.bankTexelM));
    }
    if (const JsonValue* c = v.Get("closures")) {
        out.shedSteepCap = static_cast<float>(c->Num("shedSteepCap", out.shedSteepCap));
        out.shedMssCeil = static_cast<float>(c->Num("shedMssCeil", out.shedMssCeil));
        out.churnGain = static_cast<float>(c->Num("churnGain", out.churnGain));
        out.crestLo = static_cast<float>(c->Num("crestLo", out.crestLo));
        out.crestHi = static_cast<float>(c->Num("crestHi", out.crestHi));
        out.depthLo = static_cast<float>(c->Num("depthLo", out.depthLo));
        out.depthHi = static_cast<float>(c->Num("depthHi", out.depthHi));
        out.foamOpacity = static_cast<float>(c->Num("foamOpacity", out.foamOpacity));
        out.ringBlendTexels =
            static_cast<float>(c->Num("ringBlendTexels", out.ringBlendTexels));
        out.windSeaFill = static_cast<float>(c->Num("windSeaFill", out.windSeaFill));
        out.bandFoldWeight =
            static_cast<float>(c->Num("bandFoldWeight", out.bandFoldWeight));
        out.buoyAssimAgeH = static_cast<float>(c->Num("buoyAssimAgeH", out.buoyAssimAgeH));
        out.buoyAssimGainMax =
            static_cast<float>(c->Num("buoyAssimGainMax", out.buoyAssimGainMax));
        out.causticStrength =
            static_cast<float>(c->Num("causticStrength", out.causticStrength));
        // Num() only reads NUMBERS -- a JSON `false` here silently returned the default and
        // the A/B rendered identically twice. Accept both spellings, explicitly.
        if (const JsonValue* wo = c->Get("waterOptics")) {
            out.waterOptics = (wo->type == JsonValue::Type::Bool) ? wo->boolean
                                                                  : wo->number != 0.0;
        }
        out.jettyCrestNavd =
            static_cast<float>(c->Num("jettyCrestNavd", out.jettyCrestNavd));
    }
    if (const JsonValue* fl = v.Get("fleet")) {
        const JsonValue* en = fl->Get("enabled");
        out.fleetEnabled = !en || en->type != JsonValue::Type::Bool || en->boolean;
        if (const JsonValue* bs = fl->Get("boats")) {
            if (bs->type == JsonValue::Type::Array) {
                out.fleetCount = 0;
                for (const JsonValue& b : bs->arr) {
                    if (out.fleetCount >= 8) break;
                    WaterSceneConfig::Boat& o = out.fleet[out.fleetCount++];
                    o.speed = b.Num("speed", o.speed);
                    o.halfLen = b.Num("halfLen", o.halfLen);
                    o.wakeAmp = b.Num("wakeAmp", o.wakeAmp);
                    o.offsetS = b.Num("offsetS", o.offsetS);
                    o.dir = (b.Num("dir", o.dir) < 0.0) ? -1 : 1;
                }
            }
        }
    }
    return true;
}

// Per-frame mtime watch: true when the file changed since *lastMtime (which updates).
inline bool WaterSceneChanged(const char* path, long long* lastMtime) {
    struct _stat64 st;
    if (_stat64(path, &st) != 0) return false;
    if (static_cast<long long>(st.st_mtime) == *lastMtime) return false;
    *lastMtime = static_cast<long long>(st.st_mtime);
    return true;
}

}  // namespace ga
