// ================================================================================================
//  SceneSchema - M12 step 5a: THE SCENE FILE'S VOCABULARY. Every top-level section of a scene
//  file is a struct here with a Schema over it (scene/Props.h), so the file format, its
//  defaults, its validation and its --print-scene output are one table read four ways.
//
//  THE SECTIONS, in the file's order (the plan's list, plus `tools`):
//      base       "scenes/x.json"              -- the scene this one INHERITS (weaker: loaded
//                                                first, this file's keys override it)
//      scene      {name, mode, planet, view}   -- the mode is the layer-enable law (chart |
//                                                world | gulf); `view` names the start camera
//      include    [{file, at, optional}]       -- overlays, applied in order over the base
//      data       {shaders, tides, seastate, currents, bathy}
//      time       {start, timeScale, paused, windowDays}
//      sun        {source, az, el}             -- ephemeris, or pinned at az/el
//      sea        {storm{hs, tp, dir}, datum{fromStation, mllwToNavd}}
//      water      {oneWater, pixelWater, foam, edgePx, heightScale, swe{...}, bank{...},
//                  wavefield{...}, closures{...}, fleet{...}}   -- the last three are what
//                  data/wave_scene.json overlays through `include` (the M8 law survives:
//                  authored if absent, never clobbered, hot-reloaded)
//      streaming  {tileBudget, predictEvery, directStorage, colorTrees, gisGate, seafloor,
//                  exposure, ringLoads}       -- scene state: they change the picture through
//                                                residency
//      capture    {headless, width, height, frames, dump, hdr, mp4, railDir,
//                  settle{sync, hold, exact, clearChurn}}
//      views      [{name, at, fovY, gauge, nearZ}]   -- `at` in the placement sugar; absent
//                                                       means the engine's default for the mode
//      rails      {active, keys[{t, at}], droste{levelSec, levels}}
//      portals    [{name, enabled, lat, lon, level, fill, twistDeg, lighting}]
//      entities   [{name, vessel, at, controller, throttle, steer}]
//      effects    [{name, type, enabled, ...}]  -- typed through EffectSchemas (slice.plane)
//      layers     [{name, enabled, ...}]        -- the registration order; typed by name
//      nodes      [{name, type, enabled, at, children, ...}]   -- typed through ComponentSchemas
//      tools      [{name, args}]                -- the one-shot modes a scene runs (--tool)
//
//  `tools` is the one section past the plan's list: the recipe table's selftest row is
//  "--tool selftest", and a scene file that runs a tool is that recipe's file form.
//
//  Every Number here carries the unit its legacy field is in (degrees for a compass azimuth,
//  hours for a spin-up, frames for a hold), so a file may say "7 km" or "12 kn" and the value
//  lands in the field's unit through GaUnits' one converter; a wrong quantity refuses. The
//  struct defaults ARE Options' defaults, field for field, so a scene that says nothing means
//  what the flags meant.
//
//  Prior art, named: USD (a layer stack composes prims by path; the root layer is the scene,
//  sublayers overlay it) and Godot (a .tscn is the unit of reuse; an inherited scene overrides
//  its base property by property) for the inherit-unless-override fold; the sections here are
//  the engine's own nouns.
// ================================================================================================
#pragma once

#include "core/Registry.h"
#include "core/Space.h"
#include "scene/Props.h"

#include <cstdint>
#include <string>

namespace ga::scene {

struct SceneSection {
    std::string name = "merrimack";
    int mode = 1;                     // chart | world | gulf
    std::string planet = "earth";
    std::string view = "sea";         // the start camera: a name in `views`
};
struct IncludeEntry {
    std::string file;
    std::string at;                   // the section the file overlays ("" = the root)
    bool optional = false;            // a missing file is skipped, logged
};
struct DataSection {
    std::string shaders = "shaders";
    std::string tides = "data/tides/stations.json";
    std::string seastate = "data/sea/seastate.json";
    std::string currents = "data/currents/currents.json";
    std::string bathy = "data/bathy/merrimack.json";
};
struct TimeSection {
    std::string start = "now";        // "now", "YYYY-MM-DDTHH:MM:SSZ" or unix seconds
    double timeScale = 1.0;
    bool paused = false;
    double windowDays = 7.0;
};
struct SunSection {
    int source = 0;                   // ephemeris | pinned
    float az = 112.0f, el = 26.0f;
};
struct StormProps {
    float hs = 0.0f, tp = 10.0f, dir = 90.0f;
};
struct DatumProps {
    bool fromStation = true;          // CO-OPS resolves MLLW -> NAVD88; false = mllwToNavd
    float mllwToNavd = -1.30f;
};
struct SeaSection {
    StormProps storm;
    DatumProps datum;
};
struct SweProps {
    bool enabled = true;
    bool westBoundary = true;
    double spinupH = 1.0;
    float gain = 1.0f;
    double riverQ = -1.0;             // m^3/s; < 0 = data/river/river.json
};
struct BankProps {
    bool flatBed = false;
    float flatBedNavd = -30.0f;
};
struct WavefieldProps {
    bool enabled = true;
    double orgX = -1400.0, orgZ = -800.0;
    int nx = 1600, ny = 1000;
    double cellM = 2.0;
    int comps = 32;
    double spreadDeg = 26.0, barNormalDeg = 285.0;
    double gammaHs = 0.60, minSamplesPerLambda = 8.0;
    double tideBucketM = 0.25, currentBucketMs = 0.10;
    float featherM = 120.0f, chop = 1.1f, exag = 1.15f;
    float bankTexelM = 1.2f;          // Restart: ring construction
};
struct ClosuresProps {
    float shedSteepCap = 0.44f, shedMssCeil = 0.09f;
    float churnGain = 0.12f;
    float crestLo = 0.28f, crestHi = 0.80f;
    float depthLo = 1.05f, depthHi = 1.95f;
    float foamOpacity = 0.72f;
    float ringBlendTexels = 48.0f;
    float bandFoldWeight = 1.0f, windSeaFill = 1.0f;
    float buoyAssimAgeH = 6.0f, buoyAssimGainMax = 1.8f;
    float causticStrength = 0.6f;
    bool waterOptics = true;
    float jettyCrestNavd = -99.0f;
};
struct FleetBoat {
    double speed = 4.86, halfLen = 7.5, wakeAmp = 0.55, offsetS = 0.0;
    int dir = 1;
};
struct FleetProps {
    bool enabled = false;             // boats: List(FleetBoat), unnamed
};
struct WaterSection {
    bool oneWater = true, pixelWater = true;
    float foam = 1.0f, edgePx = 12.0f, heightScale = 1.15f;
    SweProps swe;
    BankProps bank;
    WavefieldProps wavefield;
    ClosuresProps closures;
    FleetProps fleet;
};
struct StreamingSection {
    uint32_t tileBudget = 1000, predictEvery = 3;
    bool directStorage = true, colorTrees = true, gisGate = true, seafloor = true,
         exposure = true, ringLoads = true;
};
struct SettleProps {
    bool sync = false;
    uint32_t hold = 0;
    bool exact = false, clearChurn = false;
};
struct CaptureSection {
    bool headless = false;
    uint32_t width = 1600, height = 900, frames = 0;
    std::string dump, hdr, mp4, railDir;
    SettleProps settle;
};
struct ViewProps {
    std::string name;
    Placement at;                     // optional: absent = the engine's default for the mode
    float fovY = 55.0f;
    std::string gauge = "root";
    float nearZ = 0.25f;
};
struct RailKey {
    double t = 0.0;
    Placement at;
};
struct DrosteRail {
    double levelSec = 16.0;
    int levels = 3;
};
struct RailsSection {
    int active = 0;                   // none | classic | zoom | flood | jetty | droste | droste-out
    DrosteRail droste;                // keys: List(RailKey), unnamed
};
struct PortalProps {
    std::string name;
    bool enabled = true;
    double lat = 42.81826, lon = -70.80045;
    int level = 16;
    double fill = 1.0, twistDeg = 90.0;
    int lighting = 0;                 // realistic | appealing
};
struct EntityProps {
    std::string name;
    std::string vessel;
    Placement at;                     // the spawn (required)
    int controller = 0;               // helm | fixed
    double throttle = 0.0, steer = 0.0;
};
struct EffectProps {
    std::string name;
    std::string type;
    bool enabled = true;
};
struct SlicePlaneProps {
    double d = 0.0;                   // the cutaway plane's offset, world z metres
};
struct LayerEntry {
    std::string name;
    bool enabled = true;
};
struct TideLayerProps {
    float exaggeration = 60.0f;
};
struct NodeProps {
    std::string name;
    std::string type;
    bool enabled = true;
    Placement at;                     // optional: identity
};
struct ToolProps {
    std::string name;
    std::string args;
};

// The whole document as one prototype (the lists carry no fields).
struct SceneDocument {
    std::string base;                 // the inherited scene file; "" = none
    SceneSection scene;
    DataSection data;
    TimeSection time;
    SunSection sun;
    SeaSection sea;
    WaterSection water;
    StreamingSection streaming;
    CaptureSection capture;
    RailsSection rails;
};

// The root schema: every section as an Object or a List, in the file's order.
const Schema& SceneFileSchema();
// The typed lists' registries of schemas: nodes[].type, effects[].type, layers[].name.
Registry<const Schema*>& ComponentSchemas();
Registry<const Schema*>& EffectSchemas();
Registry<const Schema*>& LayerSchemas();
// In-tree registration: the slice.plane effect, the ten layers (tide with its exaggeration).
void RegisterBuiltinSceneTypes();
// The struct defaults as a document: the bottom of the fold.
JsonValue DefaultDocument();
// The element schemas, for a caller that validates one element (the tests).
const Schema& ViewSchema();
const Schema& PortalSchema();
const Schema& EntitySchema();
const Schema& NodeSchema();

}  // namespace ga::scene
