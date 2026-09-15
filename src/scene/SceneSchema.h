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
//      views      [{name, at, fovY, gauge, nearZ, reversedZ, target, viewport{}, follow{}}]
//                                                    -- `at` in the placement sugar; absent
//                                                       means the engine's default for the mode.
//                                                       M12 step 5b: the optics, the target and
//                                                       its rectangle, and the chase camera as
//                                                       data (scene/View.h)
//      rails      {active, keys[{t, at | view | pose}], droste{levelSec, levels}}
//                                                    -- M12 step 5e: `active` names the rail
//                                                       flown from scenes/rails/<active>.json
//                                                       (scene/Rail.h: segments as data); an
//                                                       inline `keys` list is an authored
//                                                       keyed flight in the same key spelling
//      portals    [{name, enabled, lat, lon, level, fill, twistDeg, lighting}]
//      entities   [{name, vessel, at, controller, throttle, steer}]
//                                                    -- the hull reads the solver's surface by
//                                                       region, every frame (scene/Entity.h)
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
#include <vector>

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
// M12 step 5b: the rectangle of the target a view records into. Zero width or height = the
// whole target, which is what every recorded frame means; an OFFSET is declarable and the
// renderer reports that it cannot honour one yet (it needs a hal::CommandContext overload).
struct ViewportProps {
    uint32_t x = 0, y = 0, w = 0, h = 0;
};
// THE CHASE CAMERA, as data: the four numbers FrameLoop's `if (helming)` block holds as
// literals (15 m back, 5 m up, aimed 0.6 m above the hull's origin). An empty target is "this
// view follows nothing", which is every recorded recipe but the boat's.
struct FollowProps {
    std::string target;
    double back = 15.0, up = 5.0, aimLift = 0.6;
};
struct ViewProps {
    std::string name;
    Placement at;                     // optional: absent = the engine's default for the mode
    float fovY = 55.0f;
    std::string gauge = "root";
    float nearZ = 0.25f;
    bool reversedZ = true;            // Camera.h's law: 1 at the near plane falling to 0
    std::string target = "main";      // the named target chain; "main" is the renderer's own
    ViewportProps viewport;
    FollowProps follow;
};
// M12 step 5e: a key is a pose at a time -- a rigid placement in the sugar, or the eye a
// named view declares (`view`), or a named pose of the tower (`pose`, scene/Rail.h).
struct RailKey {
    double t = 0.0;
    Motor at;                         // optional
    std::string view;                 // "" = none
    std::string pose;                 // "" = none
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
    // THE DESTINATION: the place the inner globe presents where the root shows this leaf. Both
    // keys are optional and read only when BOTH are declared (ScenePortal::hasTo) -- 0 N 0 E is a
    // real place, so presence is carried beside the numbers, never inferred from them.
    double toLat = 0.0, toLon = 0.0;
    int level = 16;
    double fill = 1.0, twistDeg = 90.0;
    int lighting = 0;                 // realistic | appealing
};
// THE GATE (scene/Gateway.h): a cuboid in the root's flat frame whose far side is a place on the
// same planet; a body whose centre of gravity enters it leaves from the destination.
struct GateProps {
    std::string name;
    bool enabled = true;
    Placement at;                          // the box's centre and heading: {x, alt, z, az}
    double size[3] = {60.0, 30.0, 8.0};    // across, up, along the heading -- metres
    double toLat = 0.0, toLon = 0.0;       // the destination
    double toAz = 0.0;                     // the heading the box's forward face leaves along
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

// M12 step 5f: THE UI'S NAME CONTRACT (docs/registries.json, src/scene/SchemaDoc.cpp). The three
// schema registries above and the hulls (with each kind's ledger) are read here; the loaders and
// the one-shot tools are passed in by their owner, because there is no one live registry of
// either to read -- the owner is the only place that knows what this build registers.
struct RegistryDoc {
    std::string name;                   // "loader" | "tool"
    std::string what;                   // one line: what a name in it selects
    std::vector<std::string> names;
};
void WriteRegistries(const char* path, const std::vector<RegistryDoc>& extra);
// The element schemas, for a caller that validates one element (the tests).
const Schema& ViewSchema();
const Schema& PortalSchema();
const Schema& GateSchema();
const Schema& EntitySchema();
const Schema& NodeSchema();
// M12 step 5e: the rail key (scene/Rail.h reads a rail file's keys through it) and the slice
// plane's typed table (scene/effects/SlicePlane.h's Props()).
const Schema& RailKeySchema();
const Schema& SlicePlaneSchema();

}  // namespace ga::scene
