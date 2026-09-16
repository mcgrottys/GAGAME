// SceneSchema - the sections' schemas, built once over static prototypes (M12 step 5a).
#include "scene/SceneSchema.h"

namespace ga::scene {

namespace {

// THE PROTOTYPES: static storage, so a Schema's offsets and DefaultOf() read from a living
// instance for the program's life. One document holds every section; the list elements have
// prototypes of their own.
SceneDocument kDoc;
IncludeEntry kInclude;
FleetBoat kBoat;
ViewProps kView;   // its viewport and follow prototypes live INSIDE it (Nest takes the address)
RailKey kRailKey;
PortalProps kPortal;
GateProps kGate;
EntityProps kEntity;
InterestProps kInterest;
InterestRef kInterestRef;
EffectProps kEffect;
SlicePlaneProps kSlice;
LayerEntry kLayer;
TideLayerProps kTide;
NodeProps kNode;
ToolProps kTool;

using Q = Quantity;
constexpr Reload H = Reload::Hot;
constexpr Reload R = Reload::Restart;

// M12 step 5f: WHAT Hot MEANS HERE. A reload applies a Hot key through a LIVE TARGET
// (scene/SceneReload.h) -- a struct the run reads after the write. Seven keys were flagged Hot
// with nowhere honest to land and are Restart now, for two reasons. THE SESSION OWNS THE VALUE
// at run time: `scene.view` and `views[].at` name the start camera and its eye, which WASD, the
// rails and the chase camera then move; `views[].gauge` is the Droste level the session re-roots
// as the eye crosses a link; `views[].viewport` is not read at all (the renderer records the
// whole target, Renderer::OneView); `scene.name` is a label nothing reads after the boot's
// [scene] line. THE BOOT DERIVED A DECLARATION FROM IT: `sun.source` decides whether the AST
// registers the solar edges, and `effects[].enabled` whether it registers the effect's edge, so
// a live switch would leave docs/GA_AST.md describing a path the run does not walk. A key with
// no live target is REPORTED at reload ("needs a restart"), never silently written.

const Schema& SceneSchema_() {
    static const Schema* s = [] {
        auto& p = kDoc.scene;
        Schema* sc = new Schema("scene", &p);
        sc->Bind("name", p.name, "the scene's name (a label, read into the boot's [scene] line)", R)
            .BindEnum("mode", p.mode, {"chart", "world", "gulf"},
                      "the layer-enable law: the M1 chart, the one world (estuary + planet), the gulf map", R)
            .Bind("planet", p.planet, "earth | mars", R)
            .Bind("view", p.view, "the start camera: a name in views[]", R);
        return sc;
    }();
    return *s;
}

const Schema& IncludeSchema() {
    static const Schema* s = [] {
        Schema* sc = new Schema("include", &kInclude);
        sc->BindPath("file", kInclude.file, "a scene file overlaid on the base, in order", R)
            .Bind("at", kInclude.at, "the section it overlays (\"\" = the root)", R)
            .Bind("optional", kInclude.optional, "a missing file is skipped", R);
        return sc;
    }();
    return *s;
}

const Schema& DataSchema() {
    static const Schema* s = [] {
        auto& p = kDoc.data;
        Schema* sc = new Schema("data", &p);
        sc->BindPath("shaders", p.shaders, "the shader directory (--shaders)", R)
            .BindPath("tides", p.tides, "the tide stations (--tides)", R)
            .BindPath("seastate", p.seastate, "the sea state (--seastate)", R)
            .BindPath("currents", p.currents, "the currents (--currents)", R)
            .BindPath("bathy", p.bathy, "the bathymetry (--bathy)", R);
        return sc;
    }();
    return *s;
}

const Schema& TimeSchema() {
    static const Schema* s = [] {
        auto& p = kDoc.time;
        Schema* sc = new Schema("time", &p);
        sc->Bind("start", p.start, "\"now\", \"YYYY-MM-DDTHH:MM:SSZ\" or unix seconds (--start)", R)
            .Bind("timeScale", p.timeScale, Q::Dimensionless, "1", "sim seconds per wall second (--scale)", H)
            .Bind("paused", p.paused, "start paused", H)
            .Bind("windowDays", p.windowDays, Q::Time, "day", "the tide plot's window (--window-days)", H);
        return sc;
    }();
    return *s;
}

const Schema& EarthSchema() {
    static const Schema* s = [] {
        auto& p = kDoc.sun.earth;
        Schema* sc = new Schema("sun.earth", &p);
        sc->Bind("at", p.at, Q::Length, "m", "the Earth's centre; the sun is a light at 0,0,0", R)
            .Bind("axis", p.axis, Q::Dimensionless, "", "the Earth's spin axis in that frame", R)
            .Bind("spin", p.spin, Q::Angle, "deg", "the Earth's turn about its axis", R);
        return sc;
    }();
    return *s;
}

const Schema& SunSchema() {
    static const Schema* s = [] {
        auto& p = kDoc.sun;
        Schema* sc = new Schema("sun", &p);
        sc->BindEnum("source", p.source, {"ephemeris", "pinned", "earth"},
                     "the ephemeris at the scene's time, pinned at az/el, or the Earth placed "
                     "around a sun at the origin (sun.earth) with no clock at all", R)
            .Bind("az", p.az, Q::Angle, "deg", "pinned azimuth, compass", H)
            .Bind("el", p.el, Q::Angle, "deg", "pinned elevation", H)
            .Nest("earth", EarthSchema(), &p.earth, "the Earth's place around the sun");
        return sc;
    }();
    return *s;
}

const Schema& StormSchema() {
    static const Schema* s = [] {
        auto& p = kDoc.sea.storm;
        Schema* sc = new Schema("sea.storm", &p);
        sc->Bind("hs", p.hs, Q::Length, "m", "significant height; 0 = the forecast sea (--storm)", R)
            .Bind("tp", p.tp, Q::Time, "s", "peak period", R)
            .Bind("dir", p.dir, Q::Angle, "deg", "from-direction, compass", R);
        return sc;
    }();
    return *s;
}
const Schema& DatumSchema() {
    static const Schema* s = [] {
        auto& p = kDoc.sea.datum;
        Schema* sc = new Schema("sea.datum", &p);
        sc->Bind("fromStation", p.fromStation, "CO-OPS resolves MLLW -> NAVD88 per station; false = mllwToNavd", R)
            .Bind("mllwToNavd", p.mllwToNavd, Q::Length, "m", "tide (m MLLW) + this = NAVD88 (--datum)", R);
        return sc;
    }();
    return *s;
}
const Schema& SeaSchema() {
    static const Schema* s = [] {
        auto& p = kDoc.sea;
        Schema* sc = new Schema("sea", &p);
        sc->Nest("storm", StormSchema(), &p.storm, "the sandbox sea-state override")
            .Nest("datum", DatumSchema(), &p.datum, "the MLLW -> NAVD88 link");
        return sc;
    }();
    return *s;
}

const Schema& SweSchema() {
    static const Schema* s = [] {
        auto& p = kDoc.water.swe;
        Schema* sc = new Schema("water.swe", &p);
        sc->Bind("enabled", p.enabled, "the shallow-water solver; false = the analytic tide plane (--swe-off)", R)
            .Bind("westBoundary", p.westBoundary, "the west-boundary deviation; false = zeroed (--swe-west-off)", R)
            .Bind("spinupH", p.spinupH, Q::Time, "h", "history integrated before the first frame (--swe-spinup)", R)
            .Bind("gain", p.gain, Q::Dimensionless, "1", "solved-current gain (--swe-gain)", H)
            .Bind("riverQ", p.riverQ, Q::Dimensionless, "1", "river discharge, m^3/s; < 0 = data/river/river.json (--river)", R);
        return sc;
    }();
    return *s;
}
const Schema& BankSchema() {
    static const Schema* s = [] {
        auto& p = kDoc.water.bank;
        Schema* sc = new Schema("water.bank", &p);
        sc->Bind("flatBed", p.flatBed, "replace the bed with a flat floor (--flat-bed)", R)
            .Bind("flatBedNavd", p.flatBedNavd, Q::Length, "m NAVD88", "the floor's height", R);
        return sc;
    }();
    return *s;
}
const Schema& WavefieldSchema() {
    static const Schema* s = [] {
        auto& p = kDoc.water.wavefield;
        Schema* sc = new Schema("water.wavefield", &p);
        sc->Bind("enabled", p.enabled, "the solved wave field", H)
            .Bind("orgX", p.orgX, Q::Length, "m", "window origin, world x", H)
            .Bind("orgZ", p.orgZ, Q::Length, "m", "window origin, world z", H)
            .Bind("nx", p.nx, Q::Dimensionless, "1", "cells east", H)
            .Bind("ny", p.ny, Q::Dimensionless, "1", "cells north", H)
            .Bind("cellM", p.cellM, Q::Length, "m", "cell size", H)
            .Bind("comps", p.comps, Q::Dimensionless, "1", "spectral components", H)
            .Bind("spreadDeg", p.spreadDeg, Q::Angle, "deg", "directional spread", H)
            .Bind("barNormalDeg", p.barNormalDeg, Q::Angle, "deg", "the entrance bar's normal, compass", H)
            .Bind("gammaHs", p.gammaHs, Q::Dimensionless, "1", "Hs <= gamma * h", H)
            .Bind("minSamplesPerLambda", p.minSamplesPerLambda, Q::Dimensionless, "1", "upload gate", H)
            .Bind("tideBucketM", p.tideBucketM, Q::Length, "m", "re-solve quantization", H)
            .Bind("currentBucketMs", p.currentBucketMs, Q::Velocity, "m/s", "re-solve quantization", H)
            .Bind("featherM", p.featherM, Q::Length, "m", "window edge blend", H)
            .Bind("chop", p.chop, Q::Dimensionless, "1", "Gerstner horizontal displacement", H)
            .Bind("exag", p.exag, Q::Dimensionless, "1", "display exaggeration on the solved comps", H)
            .Bind("bankTexelM", p.bankTexelM, Q::Length, "m", "ring-0 texel (ring construction)", R);
        return sc;
    }();
    return *s;
}
const Schema& ClosuresSchema() {
    static const Schema* s = [] {
        auto& p = kDoc.water.closures;
        Schema* sc = new Schema("water.closures", &p);
        sc->Bind("shedSteepCap", p.shedSteepCap, Q::Dimensionless, "1", "Miche cap on the shed slope variance", H)
            .Bind("shedMssCeil", p.shedMssCeil, Q::Dimensionless, "1", "storm-sea mss ceiling", H)
            .Bind("churnGain", p.churnGain, Q::Dimensionless, "1", "churn read gain (0 = off)", H)
            .Bind("crestLo", p.crestLo, Q::Dimensionless, "1", "crest gate, eta/rms", H)
            .Bind("crestHi", p.crestHi, Q::Dimensionless, "1", "crest gate, eta/rms", H)
            .Bind("depthLo", p.depthLo, Q::Dimensionless, "1", "depth-excess trigger band", H)
            .Bind("depthHi", p.depthHi, Q::Dimensionless, "1", "depth-excess trigger band", H)
            .Bind("foamOpacity", p.foamOpacity, Q::Dimensionless, "1", "peak foam opacity", H)
            .Bind("ringBlendTexels", p.ringBlendTexels, Q::Dimensionless, "1", "bank ring cross-fade width", H)
            .Bind("bandFoldWeight", p.bandFoldWeight, Q::Dimensionless, "1", "the fold's per-band wavelength follows the spectrum", H)
            .Bind("windSeaFill", p.windSeaFill, Q::Dimensionless, "1", "gain on the synthesised wind sea", H)
            .Bind("buoyAssimAgeH", p.buoyAssimAgeH, Q::Time, "h", "buoy assimilation: max observation age", H)
            .Bind("buoyAssimGainMax", p.buoyAssimGainMax, Q::Dimensionless, "1", "and its gain clamp", H)
            .Bind("causticStrength", p.causticStrength, Q::Dimensionless, "1", "bed dapple (0 = off)", H)
            .Bind("waterOptics", p.waterOptics, "K_d and the deep colour from the ocean-colour fields", H)
            .Bind("jettyCrestNavd", p.jettyCrestNavd, Q::Length, "m NAVD88", "edit-land floor; <= -90 = auto", H);
        return sc;
    }();
    return *s;
}
const Schema& FleetBoatSchema() {
    static const Schema* s = [] {
        Schema* sc = new Schema("water.fleet.boat", &kBoat);
        sc->Bind("speed", kBoat.speed, Q::Velocity, "m/s", "AIS per-class mean SOG", H)
            .Bind("halfLen", kBoat.halfLen, Q::Length, "m", "half length", H)
            .Bind("wakeAmp", kBoat.wakeAmp, Q::Length, "m", "wake amplitude", H)
            .Bind("offsetS", kBoat.offsetS, Q::Length, "m", "starting arc-length offset along the lane", H)
            .Bind("dir", kBoat.dir, Q::Dimensionless, "1", "+1 outbound, -1 inbound", H);
        return sc;
    }();
    return *s;
}
const Schema& FleetSchema() {
    static const Schema* s = [] {
        auto& p = kDoc.water.fleet;
        Schema* sc = new Schema("water.fleet", &p);
        sc->Bind("enabled", p.enabled, "the AIS-lane floats", H)
            .List("boats", &FleetBoatSchema(), "the boats, in lane order", false);
        return sc;
    }();
    return *s;
}
const Schema& WaterSchema() {
    static const Schema* s = [] {
        auto& p = kDoc.water;
        Schema* sc = new Schema("water", &p);
        sc->Bind("oneWater", p.oneWater, "water geometry from the wave bank alone (--no-one-water)", R)
            .Bind("pixelWater", p.pixelWater, "shade the water per pixel (--no-pixel-water)", R)
            .Bind("foam", p.foam, Q::Dimensionless, "1", "sea-mode whitecap intensity (--foam)", H)
            .Bind("edgePx", p.edgePx, Q::Dimensionless, "px", "tessellated edge target (--edge-px)", H)
            .Bind("heightScale", p.heightScale, Q::Dimensionless, "1", "wave vertical exaggeration (--height-scale)", H)
            .Nest("swe", SweSchema(), &p.swe, "the shallow-water solver")
            .Nest("bank", BankSchema(), &p.bank, "the water bank")
            .Nest("wavefield", WavefieldSchema(), &p.wavefield, "the solved wave field's window (data/wave_scene.json)")
            .Nest("closures", ClosuresSchema(), &p.closures, "the kernel closures (data/wave_scene.json)")
            .Nest("fleet", FleetSchema(), &p.fleet, "the AIS fleet (data/wave_scene.json)");
        return sc;
    }();
    return *s;
}

const Schema& StreamingSchema() {
    static const Schema* s = [] {
        auto& p = kDoc.streaming;
        Schema* sc = new Schema("streaming", &p);
        sc->Bind("tileBudget", p.tileBudget, Q::Dimensionless, "1", "hard cap on Google fetches per run (--tile-budget)", R)
            .Bind("predictEvery", p.predictEvery, Q::Dimensionless, "frames", "prefetch-walk cadence (--predict-every)", H)
            .Bind("directStorage", p.directStorage, "NVMe -> GPU tile reads (--no-direct-storage)", R)
            .Bind("colorTrees", p.colorTrees, "colour and height pages from the trees (--no-color-trees)", R)
            .Bind("gisGate", p.gisGate, "the vector land/sea gate on the bed (--no-gis-gate)", R)
            .Bind("seafloor", p.seafloor, "the global seafloor relief source (--no-seafloor)", R)
            .Bind("exposure", p.exposure, "the swell-exposure page (--no-exposure)", R)
            .Bind("ringLoads", p.ringLoads, "the ring gate (--no-ring-loads)", R);
        return sc;
    }();
    return *s;
}

const Schema& SettleSchema() {
    static const Schema* s = [] {
        auto& p = kDoc.capture.settle;
        Schema* sc = new Schema("capture.settle", &p);
        sc->Bind("sync", p.sync, "hold the dump instant until the residency queues drain (--settle-sync)", R)
            .Bind("hold", p.hold, Q::Dimensionless, "frames", "hold it for exactly N frames (--settle-hold)", R)
            .Bind("exact", p.exact, "hold until the resident set IS the want set (--settle-exact)", R)
            .Bind("clearChurn", p.clearChurn, "zero the churn atlas at the first held frame (--settle-clear-churn)", R);
        return sc;
    }();
    return *s;
}
const Schema& CaptureSchema() {
    static const Schema* s = [] {
        auto& p = kDoc.capture;
        Schema* sc = new Schema("capture", &p);
        sc->Bind("headless", p.headless, "no window (--headless)", R)
            .Bind("width", p.width, Q::Dimensionless, "px", "the frame's width (--width)", R)
            .Bind("height", p.height, Q::Dimensionless, "px", "the frame's height (--height)", R)
            .Bind("frames", p.frames, Q::Dimensionless, "frames", "frames to run; 0 = until the window closes (--frames)", R)
            .BindPath("dump", p.dump, "the still's PNG (--dump)", R)
            .BindPath("hdr", p.hdr, "the RGBA16F radiance before the tonemap (--dump-hdr)", R)
            .BindPath("mp4", p.mp4, "pipe rail frames to an encoder (--mp4)", R)
            .BindPath("railDir", p.railDir, "the rail's PNG directory (--rail*)", R)
            .Nest("settle", SettleSchema(), &p.settle, "the still's hold");
        return sc;
    }();
    return *s;
}

const Schema& ViewportSchema() {
    static const Schema* s = [] {
        auto& p = kView.viewport;
        Schema* sc = new Schema("view.viewport", &p);
        sc->Bind("x", p.x, Q::Dimensionless, "px", "left edge in the target", R)
            .Bind("y", p.y, Q::Dimensionless, "px", "top edge in the target", R)
            .Bind("w", p.w, Q::Dimensionless, "px", "width; 0 = the whole target", R)
            .Bind("h", p.h, Q::Dimensionless, "px", "height; 0 = the whole target", R);
        return sc;
    }();
    return *s;
}

const Schema& FollowSchema() {
    static const Schema* s = [] {
        auto& p = kView.follow;
        Schema* sc = new Schema("view.follow", &p);
        sc->Bind("target", p.target, "the entity this view chases; \"\" = none", H)
            .Bind("back", p.back, Q::Length, "m", "the eye, behind the target's heading", H)
            .Bind("up", p.up, Q::Length, "m", "the eye, above the target", H)
            .Bind("aimLift", p.aimLift, Q::Length, "m", "the aim, above the target's origin", H);
        return sc;
    }();
    return *s;
}

const Schema& InterestRefSchema() {
    static const Schema* s = [] {
        Schema* sc = new Schema("view.interest", &kInterestRef);
        sc->Bind("name", kInterestRef.name, "an interest this view keeps resident (interests[].name)", R);
        return sc;
    }();
    return *s;
}

const Schema& ViewSchema_() {
    static const Schema* s = [] {
        Schema* sc = new Schema("view", &kView);
        sc->Bind("name", kView.name, "the view's name", R)
            .Bind("at", kView.at, "the eye: {x, alt, z, az, pitch} | {lat, lon, alt[, lookAt]} | {motor}; absent = the mode's default", R)
            .Optional()
            .Bind("fovY", kView.fovY, Q::Angle, "deg", "vertical field of view (--fov)", H)
            .Bind("gauge", kView.gauge, "the space the eye is expressed in", R)
            .Bind("nearZ", kView.nearZ, Q::Length, "m", "the near plane", H)
            // M12 step 5b: what scene/View.h declares beside the eye -- the third optic, where
            // the recording lands, and the chase camera.
            .Bind("reversedZ", kView.reversedZ, "depth 1 at the near plane falling to 0 at infinity (Camera.h)", R)
            .Bind("target", kView.target, "the target chain this view records into (\"main\" = the renderer's own)", R)
            .Nest("viewport", ViewportSchema(), &kView.viewport, "the rectangle of the target")
            .Nest("follow", FollowSchema(), &kView.follow, "the chase camera")
            .List("interests", &InterestRefSchema(),
                  "the subjects and places whose water this view keeps resident, by name");
        return sc;
    }();
    return *s;
}

const Schema& RailKeySchema_() {
    static const Schema* s = [] {
        Schema* sc = new Schema("rails.key", &kRailKey);
        sc->Bind("t", kRailKey.t, Q::Time, "s", "the key's time", R)
            .Bind("at", kRailKey.at, "the key's pose, in the placement sugar (rigid)", R)
            .Optional()
            .Bind("view", kRailKey.view, "the eye a named view declares (the orbit start)", R)
            .Bind("pose", kRailKey.pose, "a named pose of the tower (scenes/rails/*.json poses)", R);
        return sc;
    }();
    return *s;
}
const Schema& DrosteRailSchema() {
    static const Schema* s = [] {
        auto& p = kDoc.rails.droste;
        Schema* sc = new Schema("rails.droste", &p);
        sc->Bind("levelSec", p.levelSec, Q::Time, "s", "rail seconds per level (--droste-level-sec)", R)
            .Bind("levels", p.levels, Q::Dimensionless, "1", "how deep the dive goes (--droste-levels)", R);
        return sc;
    }();
    return *s;
}
const Schema& RailsSchema() {
    static const Schema* s = [] {
        auto& p = kDoc.rails;
        Schema* sc = new Schema("rails", &p);
        sc->BindEnum("active", p.active,
                     {"none", "classic", "zoom", "flood", "jetty", "droste", "droste-out"},
                     "the rail flown (--rail, --rail-zoom, --rail-flood, --rail-jetty, --rail-droste[-out])", R)
            .List("keys", &RailKeySchema_(), "an authored keyed flight, inline (the shipped rails are scenes/rails/<active>.json)", false)
            .Nest("droste", DrosteRailSchema(), &p.droste, "the dive");
        return sc;
    }();
    return *s;
}

const Schema& PortalSchema_() {
    static const Schema* s = [] {
        Schema* sc = new Schema("portal", &kPortal);
        sc->Bind("name", kPortal.name, "the portal's name", R)
            .Bind("enabled", kPortal.enabled, "link the root under a leaf of itself (--droste)", R)
            .Bind("lat", kPortal.lat, Q::Angle, "deg", "the leaf's place (--droste-at)", R)
            .Bind("lon", kPortal.lon, Q::Angle, "deg", "the leaf's place", R)
            .Bind("toLat", kPortal.toLat, Q::Angle, "deg",
                  "the DESTINATION: the place the inner globe presents where the root shows this leaf "
                  "(the root turned by the shortest arc carrying it onto the leaf's place, then twisted); "
                  "with toLon; absent = the leaf's own place", R)
            .Optional()
            .Bind("toLon", kPortal.toLon, Q::Angle, "deg", "the destination's longitude (with toLat)", R)
            .Optional()
            .Bind("level", kPortal.level, Q::Dimensionless, "1", "the quadtree level (16 = a 153 m leaf)", R)
            .Bind("fill", kPortal.fill, Q::Dimensionless, "1", "the inner globe's diameter / leaf span (--droste-fill)", R)
            .Bind("twistDeg", kPortal.twistDeg, Q::Angle, "deg", "the twist per level about north (--droste-twist)", R)
            .BindEnum("lighting", kPortal.lighting, {"realistic", "appealing"}, "the lighting A/B (--droste-light)", H);
        return sc;
    }();
    return *s;
}

const Schema& GateSchema_() {
    static const Schema* s = [] {
        Schema* sc = new Schema("gate", &kGate);
        sc->Bind("name", kGate.name, "the gate's name", R)
            .Bind("enabled", kGate.enabled, "carry bodies through it", R)
            .Bind("at", kGate.at, "the box's centre and heading in the flat world frame: {x, alt, z, az}", R)
            .Optional()
            .Bind("size", kGate.size, Q::Length, "m", "the box: across, up, and along its heading", R)
            .Bind("toLat", kGate.toLat, Q::Angle, "deg", "the destination's latitude", R)
            .Bind("toLon", kGate.toLon, Q::Angle, "deg", "the destination's longitude", R)
            .Bind("toAz", kGate.toAz, Q::Angle, "deg",
                  "the compass heading the box's forward face leaves along at the destination", R);
        return sc;
    }();
    return *s;
}

const Schema& InterestSchema_() {
    static const Schema* s = [] {
        Schema* sc = new Schema("interest", &kInterest);
        sc->Bind("name", kInterest.name, "the interest's name (views[].interests names it)", R)
            .Bind("target", kInterest.target, "the entity whose surroundings stay resident (\"\" = the fixed `at`)", R)
            .Bind("at", kInterest.at, "a fixed place in the flat world frame: {x, alt, z}", R)
            .Optional()
            .Bind("radius", kInterest.radius, Q::Length, "m",
                  "how far around the subject its water's pages stay resident", R);
        return sc;
    }();
    return *s;
}

const Schema& EntitySchema_() {
    static const Schema* s = [] {
        Schema* sc = new Schema("entity", &kEntity);
        sc->Bind("name", kEntity.name, "the entity's name", R)
            .Bind("vessel", kEntity.vessel, "the hull kind (VesselRegistry; --boat)", R)
            .Bind("at", kEntity.at, "the spawn, in the flat world frame (--campos); `az` is the bow's compass heading (absent: north, as every hull always spawned)", R)
            .BindEnum("controller", kEntity.controller, {"helm", "fixed"},
                      "the keyboard helm, or fixed throttles and helm (--boat-drive)", H)
            .Bind("throttle", kEntity.throttle, Q::Dimensionless, "1", "fixed: every thruster's throttle", H)
            .Bind("steer", kEntity.steer, Q::Dimensionless, "1", "fixed: the commanded steering", H)
            .Bind("recentreM", kEntity.recentreM, Q::Length, "m",
                  "how far this hull may sail from its space's origin before the space moves to "
                  "its place (the per-subject floating origin; 0 = never, and it rides d^2/2R "
                  "above the sphere)", H);
        return sc;
    }();
    return *s;
}

const Schema& EffectSchema() {
    static const Schema* s = [] {
        Schema* sc = new Schema("effect", &kEffect);
        sc->Bind("name", kEffect.name, "the effect's name", R)
            .Bind("type", kEffect.type, "the effect kind (EffectSchemas)", R)
            .Bind("enabled", kEffect.enabled, "apply it", R);
        return sc;
    }();
    return *s;
}
const Schema& SlicePlaneSchema_() {
    static const Schema* s = [] {
        Schema* sc = new Schema("slice.plane", &kSlice);
        sc->Bind("d", kSlice.d, Q::Length, "m", "the cutaway plane's offset, world z (--slice)", H);
        return sc;
    }();
    return *s;
}

const Schema& LayerEntrySchema() {
    static const Schema* s = [] {
        Schema* sc = new Schema("layer", &kLayer);
        sc->Bind("name", kLayer.name, "the layer (Layer::Name)", R)
            .Bind("enabled", kLayer.enabled, "draw it", H);
        return sc;
    }();
    return *s;
}
const Schema& TideLayerSchema() {
    static const Schema* s = [] {
        Schema* sc = new Schema("layer.tide", &kTide);
        sc->Bind("exaggeration", kTide.exaggeration, Q::Dimensionless, "1", "ribbon vertical exaggeration (--exagg)", R);
        return sc;
    }();
    return *s;
}
const Schema& EmptySchema() {
    static const Schema* s = [] { return new Schema("layer.plain", &kLayer); }();
    return *s;
}

const Schema& NodeSchema_() {
    static const Schema* s = [] {
        Schema* sc = new Schema("node", &kNode);
        sc->Bind("name", kNode.name, "the node's name", R)
            .Bind("type", kNode.type, "the component type (ComponentSchemas)", R)
            .Bind("enabled", kNode.enabled, "walked and drawn", H)
            .Bind("at", kNode.at, "its placement in the parent (any spelling); absent = identity", H)
            .Optional()
            .List("children", sc, "child nodes, in file order", true, "component", "type");
        return sc;
    }();
    return *s;
}

const Schema& ToolSchema() {
    static const Schema* s = [] {
        Schema* sc = new Schema("tool", &kTool);
        sc->Bind("name", kTool.name, "the one-shot mode (--tool name[:args])", R)
            .Bind("args", kTool.args, "its arguments, as one string", R);
        return sc;
    }();
    return *s;
}

}  // namespace

const Schema& ViewSchema() { return ViewSchema_(); }
const Schema& PortalSchema() { return PortalSchema_(); }
const Schema& GateSchema() { return GateSchema_(); }
const Schema& EntitySchema() { return EntitySchema_(); }
const Schema& InterestSchema() { return InterestSchema_(); }
const Schema& NodeSchema() { return NodeSchema_(); }
const Schema& RailKeySchema() { return RailKeySchema_(); }
const Schema& SlicePlaneSchema() { return SlicePlaneSchema_(); }

const Schema& SceneFileSchema() {
    static const Schema* s = [] {
        Schema* sc = new Schema("scene-file", &kDoc);
        sc->BindPath("base", kDoc.base, "the scene this one inherits (loaded first; this file overrides it)", R)
            .Nest("scene", SceneSchema_(), &kDoc.scene, "the scene: name, mode, planet, start view")
            .List("include", &IncludeSchema(), "overlays applied over this file, in order", false)
            .Nest("data", DataSchema(), &kDoc.data, "the data files")
            .Nest("time", TimeSchema(), &kDoc.time, "the scene clock")
            .Nest("sun", SunSchema(), &kDoc.sun, "the sun")
            .Nest("sea", SeaSchema(), &kDoc.sea, "the sea state and the datum")
            .Nest("water", WaterSchema(), &kDoc.water, "the water")
            .Nest("streaming", StreamingSchema(), &kDoc.streaming, "residency: scene state")
            .Nest("capture", CaptureSchema(), &kDoc.capture, "headless capture")
            .List("views", &ViewSchema_(), "the cameras, by name")
            .Nest("rails", RailsSchema(), &kDoc.rails, "the camera rails")
            .List("portals", &PortalSchema_(), "the Droste links, by name")
            .List("gates", &GateSchema_(), "the cuboid gates to other places, by name")
            .List("entities", &EntitySchema_(), "the vessels, by name")
            .List("interests", &InterestSchema_(), "the subjects and places whose water stays resident for the views that name them, by name")
            .List("effects", &EffectSchema(), "the paper visuals, by name, typed", true, "effect", "type")
            .List("layers", &LayerEntrySchema(), "the layers in registration order, typed by name", true, "layer", "name")
            .List("nodes", &NodeSchema_(), "typed nodes (plugins)", true, "component", "type")
            .List("tools", &ToolSchema(), "the one-shot modes this scene runs");
        return sc;
    }();
    return *s;
}

Registry<const Schema*>& ComponentSchemas() {
    static Registry<const Schema*> r("component");
    return r;
}
Registry<const Schema*>& EffectSchemas() {
    static Registry<const Schema*> r("effect");
    return r;
}
Registry<const Schema*>& LayerSchemas() {
    static Registry<const Schema*> r("layer");
    return r;
}

void RegisterBuiltinSceneTypes() {
    static bool done = false;
    if (done) return;
    done = true;
    EffectSchemas().Register("slice.plane", [] { return &SlicePlaneSchema_(); });
    for (const char* name : {"sky", "sea", "terrain", "gulf", "waterbank", "globe", "gis",
                             "vessels", "markers"}) {
        LayerSchemas().Register(name, [] { return &EmptySchema(); });
    }
    LayerSchemas().Register("tide", [] { return &TideLayerSchema(); });
}

JsonValue DefaultDocument() {
    return PropSet::Defaults(SceneFileSchema(), &kDoc).ToJson();
}

}  // namespace ga::scene
