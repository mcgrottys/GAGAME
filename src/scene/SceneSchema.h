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
//      water      {pixelWater, foam, edgePx, heightScale, swe{...}, bank{...},
//                  wavefield{...}, closures{...}, fleet{...}}   -- the last three are what
//                  data/wave_scene.json overlays through `include` (the M8 law survives:
//                  authored if absent, never clobbered, hot-reloaded), and their table is
//                  WaterComponent's own (scene/WaterComponent.h), nested over `wave`
//      streaming  {tileBudget, dayTiles, dayBytes, googleZoom, predictEvery, directStorage,
//                  colorTrees, gisGate, seafloor, exposure, holdMargin, treeRoot}
//                                                -- scene state: they change the picture
//                                                through residency (treeRoot: where the tile
//                                                trees are, cache/trees unless a tool's scratch
//                                                folder is named; dayTiles/dayBytes: the day's
//                                                cap on Google requests, core/DayLedger.h;
//                                                googleZoom: the Google source's finest zoom)
//      capture    {headless, width, height, frames, dump, hdr, mp4, railDir,
//                  settle{sync, hold, exact, clearChurn}, residencyAudit}
//      hud        {fps, minimap{enabled, follow, size, margin, homeAltM, minAltM}}
//                                                    -- the windshield (scene/HudLayer.h)
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
//      prune      {mode, root, ageDays, purgeDays, confirm}  -- the tree-prune tool's keys
//      tools      [{name, args}]                -- the one-shot modes a scene runs (--tool)
//
//  `tools` is the one section past the plan's list: the recipe table's selftest row is
//  "--tool selftest", and a scene file that runs a tool is that recipe's file form. `prune` is
//  the first tool whose configuration is scene keys rather than one args string, because one of
//  its keys is a confirmation (compose/TreePrune.h) and an args string has no names to confirm
//  with.
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
#include "core/SceneConfig.h"
#include "core/Space.h"
#include "scene/Props.h"

#include <cstdint>
#include <string>
#include <vector>

namespace ga::scene {

struct SceneSection {
    std::string name;                 // PHASE C5: no place's name; every shipped root scene says its own
    int mode = 1;                     // chart | world | gulf
    std::string planet = "earth";
    std::string view = "sea";         // the start camera: a name in `views`
    int windowDepth = 0;              // at most this many windows deep (0: as deep as the screen shows)
};
struct IncludeEntry {
    std::string file;
    std::string at;                   // the section the file overlays ("" = the root)
    bool optional = false;            // a missing file is skipped, logged
};
struct DataSection {
    std::string shaders = "shaders";
    std::string tides;                // the water's products, as the harvesters write them: no
    std::string seastate;             // place's files by default -- "" = none, and every shipped
    std::string currents;             // root scene names its own
    std::string route;                // PHASE C5: the chart's channel route (geojson); "" = none
    std::string tideFocus;            // PHASE C3: a tide station's id; "" = the nearest the water's anchor
    std::string river;                // the river gauges' discharges (harvest river); "" = none
    std::string edits;                // the owner's hand polygons (the law over every survey); "" = none
};
// PHASE C4: THE SCENE'S PLACE. world.flat is the tangent plane about `anchor` ([lon, lat]), exact;
// every {x, alt, z} the scene writes without a place of its own stands in it. No default place:
// [0, 0] is the planet frame's own origin (the Mars scene's frame), and every shipped Earth
// scene names its anchor.
struct PlaceSection {
    double anchor[2] = {0.0, 0.0};
};
struct TimeSection {
    std::string start = "now";        // "now", "YYYY-MM-DDTHH:MM:SSZ" or unix seconds
    double timeScale = 1.0;
    bool paused = false;
    double windowDays = 7.0;
};
// A CLOCK OF ITS OWN (docs: the collage binds a clock per producer; scene/SceneClocks.h resolves it).
// A constant or a binding: t = anchor + offset + rate (B(scene time) - B(the scene's start)), B the
// clock it binds ("" = the scene's own, `time`), the anchor its start ("" = B at the scene's start).
// rate 1 keeps B's pace, 0 holds the instant. A cycle of bindings is refused at load.
struct ClockProps {
    std::string start;
    double rate = 1.0;
    std::string bind;     // time | water.clock | sun.clock | sun.earth.clock ("" = time)
    std::string offset;   // seconds or a run of number+unit: "-3h", "1h30m", "2d"
};
// THE EARTH, PLACED AROUND A SUN THAT IS A LIGHT AT THE ORIGIN. Position in metres (the sun is
// at 0,0,0), and a turn of `spin` degrees about `axis`. Nothing here reads the clock: a scene can
// hold a night storm at noon if it wants to, and coupling this to time is a later, separate step.
struct EarthPlacement {
    double at[3] = {-1.495978707e11, 0.0, 0.0};
    double axis[3] = {0.0, 0.0, 1.0};
    double spin = 0.0;
    ClockProps clock;                 // ephemeris: the Earth's turn (where a place faces: the time of day)
};
struct SunSection {
    int source = 0;                   // ephemeris | pinned | earth
    float az = 112.0f, el = 26.0f;
    EarthPlacement earth;
    ClockProps clock;                 // ephemeris: the sun's place on the ecliptic (declination, distance: the season)
};
// THE AIR OF THE DAY (src/scene/Air.h): the planet's air is rows of measurements; what changes from
// day to day is the aerosol, and the picture's white follows from the one sun through that air.
struct AirSection {
    float aod550 = -1.0f;   // the aerosol optical depth at 550 nm; < 0 = the planet's typical
                            // value (Earth 0.10, the maritime mean; Mars's dust 0.5)
    float angstrom = -1.0f; // the aerosol's Angstrom exponent: its column scales as
                            // (lambda / 550 nm)^-angstrom; < 0 = the planet's typical value
                            // (Earth 0.6, the maritime mean; Mars's dust 0)
    float exposure = 0.0f;  // the tonemap's scale; 0 = THE LAW: 1 / the luminance of a white level
                            // surface under a zenith sun through this air (sun + sky), so a
                            // sunlit white is the display's white
};
struct StormProps {
    float hs = 0.0f, tp = 10.0f, dir = 90.0f;
};
struct DatumProps {
    bool fromStation = true;          // CO-OPS resolves MLLW -> NAVD88; false = mllwToNavd
    float mllwToNavd = 0.0f;          // PHASE C3: no place's number; the scene declares its own
};
// AN ASTEROID INTO THE SEA (2026-10-25; laws/Impact.hlsli, sim/ImpactWave.h): the impactor and where
// and when it lands; the wave is the one law's, evaluated everywhere (the globe's vertices, the
// bank, the hull, the solver's faces). The scene's `impacts` list is the RECORD of events (each with
// its `at`; "" = the water clock's first instant); `sea.impactor` is the one the I key drops at the
// point under the cursor, at the clock's instant (sim/ImpactField.h holds the record).
// `exag` exaggerates the DRAWN wave alone (the hull and the solver read it raw); `reachM` is how far
// the sum must hold (the discrete sum's image returns at 2 pi / dk, dk = 12 / (R_D terms)): the terms
// follow from it (12 reach / (pi R_D), at most 4096), or `terms` names them.
struct ImpactProps {
    bool enabled = true;
    double lat = 35.0, lon = -42.0;
    double diameterM = 10000.0, speedMs = 20000.0, densityKgM3 = 3000.0, angleDeg = 45.0;
    std::string at;
    double basinDepthM = 4000.0;
    double energyShare = 0.094;
    double terms = 0.0;        // 0 = by the reach: 12 reach / (pi R_D), the alias past the reach
    double reachM = 6.0e6;     // how far the sum must hold before its image returns (the Atlantic)
    double exag = 1.0;
};
struct SeaSection {
    StormProps storm;
    ImpactProps impactor;   // the impactor the I key drops at the point under the cursor
    DatumProps datum;
    double box[4] = {0.0, 0.0, 0.0, 0.0};   // PHASE C2: data.seastate's place when its file says none
};
struct SweProps {
    bool enabled = true;
    bool riverTide = true;            // the river faces' tide deviation; false = zeroed (--swe-river-off)
    double spinupH = 1.0;
    float gain = 1.0f;
    // THE WAVE SOLVE'S READ OF THE SOLVER'S CURRENT, every this many sim seconds (WaveField::RefreshSweCurrent;
    // 90 since F13): a new current is a new key and a new solve (~3 min), and a headless record waits for
    // each. A scene run fast (the asteroid's arrival at 30x) sets it past its length and keeps the first.
    double currentRefreshS = 90.0;
    // THE SOLVER'S SUBSTEPS A FRAME (SweSolver::SetStepBudget; 16 since M5): past it the solver starves and
    // runs time-dilated. A record at 30x with a surge's 0.012 s step needs ~90; the descent at 130x ~360.
    double stepsPerFrame = 16.0;
    // STAGE 1d OF ONE WATER ON THE TREE (2026-10-26, WATER_V2.md 14): the reach of the eye's demand for the
    // solver's finest mip, m about its ground point; 0 = the whole window at mip 0. Beyond it the coarse twin
    // carries the water (SweSolver::SetEye).
    double fineReachM = 0.0;
    // m^3/s at the first river face; < 0 = its gauge's, from data.river (sim/RiverModel.h DischargeAt)
    double riverQ = -1.0;
    int bedWait = 2;                  // none | map | whole: what the spin-up waits for (finding 48)
    // STAGE 2 OF ONE WATER ON THE TREE (2026-10-26, WATER_V2.md 14): THE SOLVER STANDS WHERE THE EYE IS.
    // No box: its window is this many tiles of 128 cells a side about the eye's start (the place's
    // anchor), its corner on the global lattice of the cell's angle (so a place falls in the same cells
    // whichever eye came), its bed the one height stack's, its open sea the tide atlas's. 0 = no solver.
    // This is a budget (the tail's extent at two mips), not a place; the chain's coarse mips carrying the
    // planet is what retires it. The cell: one angle both ways, cellM metres north-south. Where rivers
    // enter the window is data (data.river's reaches, sim/RiverModel.h).
    uint32_t windowTiles = 16;
    float cellM = 13.728f;
    // STAGE 2b: THE TAIL -- the coarse lattice across the ocean about the same anchor: tailCells a side of cells
    // 2^tailRank of the window's (2048 of 2^9: 5 x 7 km at the Merrimack, 10,000 x 14,000 km on the plane, the
    // window whole in its central cells); 0 = no tail.
    uint32_t tailCells = 2048;
    uint32_t tailRank = 9;
    // STAGE 2c: THE TELESCOPE -- the anchor levels between the tail and the window: this many, telescopeCells a
    // side, their ranks telescopeStep apart below the tail's (3 of 512 cells, 2 apart: ranks 7, 5, 3 -- 1.76 km,
    // 440 m, 110 m cells, 900, 225 and 56 km across).
    uint32_t telescopeLevels = 3;
    uint32_t telescopeCells = 512;
    uint32_t telescopeStep = 2;
    double patchCellsPerLip = 8.0;   // an event's patch: its lip's radius over this many cells (16 = one rank finer)
    double levelTiles = 1024.0;      // a sparse level's tile budget (0 = unbounded): above it, the tiles whose field the parent holds best are released
    double dampH = 0.0;              // the solver's linear damping as the velocity's e-folding time (h); 0 = none
};
struct BankProps {
    bool flatBed = false;
    float flatBedNavd = -30.0f;
    bool column = true;   // the Eulerian column plane: overturning crests as geometry (WaterBank.hlsl)
    float columnTexel = 0.3f;   // the finest texel the column ladder reaches (m): rings under ring 0, resident under a fold
};
struct WaterSection {
    // THE DATA'S CLOCK (scene/SceneClocks.h): the tide, the solver, the currents, the sea state's hour,
    // the wind and the cascades read it. Unset, it is the scene's own.
    ClockProps clock;
    bool pixelWater = true;
    float foam = 1.0f, edgePx = 12.0f, heightScale = 1.15f;
    SweProps swe;
    BankProps bank;
    // wavefield, closures, fleet: the struct WaterComponent reads, under WaterComponent's own three
    // tables (scene/WaterComponent.h), nested here as views of it -- one table of those keys.
    WaterSceneConfig wave;
};
struct StreamingSection {
    uint32_t tileBudget = 1000, predictEvery = 3;
    // The residency pool's ceiling in 64 KB tiles (ResidencyManager::kPoolCapTiles = 32768, 2 GB):
    // smaller, the order's cut releases sooner -- a test's way to make the discard-and-replace path run.
    uint32_t poolTiles = 32768;
    // F2/D1: the turns a landing tile takes to come into focus (ResidencyManager::fadeTurns); 0 snaps.
    uint32_t fadeTurns = 6;
    // THE DAY'S CAP on Google, every engine on the machine together, per UTC day, whichever is
    // met first (core/DayLedger.h): dayTiles bounds the requests SENT, whatever came back;
    // dayBytes the bytes landed. Zero is no request that day, not "no limit".
    uint32_t dayTiles = 100000;
    double dayBytes = 5.0e9;          // a double: 5 GB does not fit the u32 field
    // The finest zoom the Google source may be asked for, held to 0..19; each step finer is four
    // times the tiles. 14 is the source as it was; any other value names itself in the source's
    // structure, so it paints its own tree (compose/Sources.h).
    uint32_t googleZoom = 14;
    // The Google fetch threads off the loaders (core/TileProviders.h StartNetwork): -1, four in a
    // window and none headless (a still's frames must see their tiles); 0, the fetch in the paint.
    int fetchThreads = -1;
    // Sources paint a tile at a time where they can (ColorSource::PaintTile); false, texel by texel
    // (the A/B: the two must be byte for byte one).
    bool tilePaint = true;
    // No network this run: every provider serves its cache and asks nothing (a session that cannot be
    // made means the same without being asked).
    bool offline = false;
    bool directStorage = true, colorTrees = true, gisGate = true, seafloor = true,
         exposure = true;
    // THE GLOBAL COAST: OSM's land polygons as GALAND01 (harvester/harvest_land_polygons.py);
    // "" = the New England survey alone (compose/GisMask.h).
    std::string gisLand;
    std::string gisWater;   // the planet's inland water, GAWATR01 ("" = New England's NHD carve)
    // F16 (HIERARCHY 4.20, the third clause): THE SURVEY'S GRAIN, metres -- the finest level the
    // land/sea mask has anything of its own for. A mask tile finer than it is its parent,
    // magnified: never painted, stored or loaded. The survey's shoreline (NOAA CUSP, NAD83 against
    // WGS84 "~1 m, documented not corrected") and the NHD water polygons (1:24,000) have nothing
    // below a metre; before this the mask held 913,717 tiles at 1.9-7.5 cm texels. 0 = every grain.
    double gisGrainM = 1.0;
    double holdMargin = 1.41421356;   // order: a held tile counts for this times its measure
    std::string treeRoot = "cache/trees";   // the folder the tile trees live in (TileTree.h)
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
    // The residency audit every N turns (hal/ResidencyAudit.h); 0 = off, and then it costs one
    // branch a turn. What the run records about its residency, beside what it records on screen.
    uint32_t residencyAudit = 0;
};
// THE WINDSHIELD (scene/HudLayer.h): what is drawn on the glass over the picture. A scene that
// says nothing draws nothing there, and records what it recorded before the glass existed.
struct MinimapProps {
    bool enabled = false;
    std::string follow = "boat";   // the entity the marker stands on and Reset centres over
    double size = 0.34;            // the square's side, as a fraction of the target's height
    uint32_t margin = 16;          // px from the bottom-right corner
    double homeAltM = 6.0e6;       // Reset: this far above the followed entity, looking down
    std::string layers;            // the layers this eye draws, by name, space-separated; empty = its own list
    double minAltM = 8.0;          // the zoom's floor above the sea (over a hull's console)
    // FAST TRAVEL (2026-10-12): with the minimap's travel toggle on, a click on the map opens a
    // spherical gate (scene/Gateway.h, radius > 0) `travelAheadM` ahead of the followed vehicle,
    // leading to the clicked place at the vehicle's heading. Its radius grows from nothing to
    // `travelRadiusM` over `travelGrowSec` (smoothstep in time) and the gate stands until
    // `travelLifeSec` after it opened, then is gone -- every number read off the scene clock, so a
    // second player given the same record (FrameLoop::TravelRecord) sees the same sphere.
    double travelAheadM = 8.0;
    double travelRadiusM = 25.0;
    double travelGrowSec = 7.0;
    double travelLifeSec = 12.0;
};
struct HudSection {
    bool fps = false;              // the frame rate, top left
    MinimapProps minimap;          // the second eye, bottom right
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
    std::string file;                 // a rail file of the scene's own, flown instead of the named one
    DrosteRail droste;                // keys: List(RailKey), unnamed
};
// A RASTER IS A SOURCE BY BEING A FILE (compose/RasterFileSource.h): one file, a folder and a
// pattern, or a manifest of raw rows -- one source each. `kind` and `crs` are read only where the
// file cannot say (a CRS it carries wins); `over` is the stack order's first key, the grain its
// second; `feather` (m) softens the edge of the whole. A height's `unit` and `datum` are read only
// where the file names none; `offset` (m) is read only where it is written (hasOffset: 0 is a
// separation too), and a height off the engine's datum is taken only with it. A photo's
// `captureLevel` (m above `captureDatum`; the datum alone is 0 m on it) is the water level it was
// flown at: its pixels are the ground only above it (compose/Sources.h LevelGateSource).
struct SourceProps {
    std::string name, file, folder, match, manifest, kind, crs, unit, datum, captureDatum;
    double over = 0.0, feather = 0.0, offset = 0.0, captureLevel = 0.0;
    bool hasOffset = false, hasCaptureLevel = false;
};
struct PortalProps {
    std::string name;
    bool enabled = true;
    double lat = 0.0, lon = 0.0;      // PHASE C5: the shipped portals name their leaf
    // THE DESTINATION: the place the inner globe presents where the root shows this leaf. Both
    // keys are optional and read only when BOTH are declared (ScenePortal::hasTo) -- 0 N 0 E is a
    // real place, so presence is carried beside the numbers, never inferred from them.
    double toLat = 0.0, toLon = 0.0;
    int level = 16;
    double fill = 1.0, twistDeg = 90.0;
    int lighting = 0;                 // realistic | appealing
};
// THE GATE (scene/Gateway.h): a cuboid standing at one place whose far side is another place on
// the same planet; a body whose centre of gravity enters it leaves from the destination. A place is
// a latitude and a longitude, carried with its presence (SceneGate) because 0 N 0 E is a real
// one; a gate that names none stands at, or leads to, the scene's own flat frame.
struct GateProps {
    std::string name;
    bool enabled = true;
    double fromLat = 0.0, fromLon = 0.0;   // where the box stands (absent: the scene's frame)
    // ONE BOX, POSED TWICE (scene/Gateway.h BoxPose). Each end is the placement sugar in its own
    // place's flat frame: {x, alt, z, az[, pitch]} -- the heading about that place's up and the
    // lean about the box's own across-axis, nose up positive -- or the motor spelling, which
    // carries any orientation. No `pitch` is no lean, which is the only reason a box is y-up.
    Placement at;                          // where it stands, and how it faces, at `from`
    double size[3] = {60.0, 30.0, 8.0};    // across, up, along the heading -- metres
    // THE SHAPE (2026-10-12): a radius > 0 makes the gate a SPHERE of that radius about the box's
    // centre, and `size` is not read; 0 is the cuboid of `size`. A sphere is the fast-travel
    // portal's shape (FrameLoop::SpawnTravel grows it), read live every frame as `size` is.
    double radius = 0.0;
    double toLat = 0.0, toLon = 0.0;       // the destination (absent: the scene's frame)
    Placement toAt;                        // the same box, posed again at the destination
    double toAz = 0.0;                     // its heading there when `toAt` names none
};
// THE INTERESTS (the water match): a subject or a place whose surroundings stay resident at the
// grain its water's kernels read, for the views that name one -- so an eye arriving there, or a
// hull floating there, finds its data landed rather than landing. `target` names an entity (the
// interest follows it); otherwise `at` is a fixed place in the placement sugar. A view keeps
// resident only the interests it lists (views[].interests): one camera can hold a boat's water
// while another holds none.
struct InterestProps {
    std::string name;
    std::string target;               // an entity's name; "" = the fixed place `at`
    Placement at;                     // optional
    double radius = 200.0;            // m: how far around the subject
};
// A view's reference to one of the scene's interests, by name.
struct InterestRef {
    std::string name;
};
// ONE FORM OF AN ENTITY (parts plus forms, 2026-10-12): a hull kind and, if the scene has one,
// the reference model drawn over it. An entity with `forms` is ONE body (sim/VesselSpec.h Form)
// whose T / Y cycle takes the next form; `vessel` is then the first form's.
struct FormProps {
    std::string vessel;
    std::string mesh;
    double meshScale = 1.0, meshYaw = 0.0;
};
struct EntityProps {
    std::string name;
    std::string vessel;
    std::vector<FormProps> forms;
    Placement at;                     // the spawn (required)
    int controller = 0;               // helm | fixed
    double throttle = 0.0, steer = 0.0;
    // THE REFERENCE MODEL (2026-10-11): a glTF binary the vessel layer draws over the physics --
    // the Baja's body over the car's own wheels. Its frame is the spec's geometry datum (y up, +z
    // the bow) after `meshScale` to metres and `meshYaw` about +y. Empty: the spec's boxes.
    std::string mesh;
    double meshScale = 1.0, meshYaw = 0.0;
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
// The buildings layer's DECLARED assumptions (compose/BuildingSolids.h): what a floor and an
// untagged building stand for, and how far about the eye its cells are streamed.
struct BuildingsLayerProps {
    double levelHeight = 3.0, defaultHeight = 6.0, radius = 3000.0;
    std::string lod;                 // the size-stratified pyramid's folder ("" = no far boxes)
    double lodPixels = 1.0;
    // THREE FORMS BY SIZE (the owner, 2026-10-15): a building narrower than boxPixels on screen is a
    // POINT (one vertex, lit by its rotor), from boxPixels its moment BOX (a hexahedron), from
    // shapePixels its own form (the mesh). Opaque at every size: no form fades into another.
    double boxPixels = 2.0;
    double shapePixels = 8.0;
    double pageMB = 1024.0;          // the loaded pages' budget (CPU); the shape pool on the GPU is fixed
    double standMB = 256.0;          // the stands' record pool on the GPU (64 B a building stood): what the walks can name
    double marginDeg = 5.0;          // the walk's cone past the view's edge: what a turn finds already asked
    bool field = true;               // the buildings under a pixel as scalars (compose/BuildingField.h)
    // THE ROADS IN THE SAME TREE (compose/BuildingLod.h RoadRibbonDefaults): the scene's assumptions
    // where a way's file is silent.
    double laneWidth = 3.5, defaultLanes = 2.0, pathWidth = 2.0, kerb = 0.12;
    double rampGrade = 0.06, deckDepth = 1.0, bridgeClearance = 5.0;   // the decks and their ramps (docs/ROADS.md)
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
// PHASE C3: where a station of the scene's data stands when its own file does not say.
struct StationProps {
    std::string name;                 // the station's id in its file
    double lat = 0.0, lon = 0.0;
};
// THE PRUNE TOOL (compose/TreePrune.h, --tool tree-prune). The mode is `list` unless the scene
// says otherwise, and `retire` and `purge` act only when `confirm` names the root's full path as
// the listing printed it; the ages are days. `root` "" is cache\trees.
struct PruneSection {
    int mode = 0;                     // list | retire | purge
    std::string root;
    double ageDays = 30.0;            // a stamp older than this is STALE (unstamped: twice it)
    double purgeDays = 7.0;           // a retired batch older than this is deleted by purge
    std::string confirm;
    std::string node;                 // retire: only this node's folders; "" = every node
};

// THE RENDER section: the kept frame (scene/GlobeLayer.h keepOn) and the grains the picture is
// redrawn at when the sun or the tide moves by more than them.
struct RenderSection {
    bool keep = true;
    double keepSunGrainDeg = 0.01;
    double keepTideGrainM = 0.001;
    // THE SUN'S RAY OVER THE GROUND (shaders/TerrainRay.hlsli; hal/MaxLane.h): the ray from every lit
    // point cast toward the sun over the height's max lane, so a hillside, a dune or a jetty shades
    // what stands behind it -- every texel of the level the pixel may read, passed a block at a time
    // where nothing in the block can reach the ray. `shadowSteps` caps the traversal's steps
    // (0 = no terrain shadow); `shadowRangeM` is how far the ray reaches.
    int shadowSteps = 128;
    double shadowRangeM = 8000.0;
    // The water's mirror ray marched over the same field: where it meets land the reflection
    // is that land, not the sky behind it. A test (2026-10-15), off until the owner's eye says.
    bool reflectLand = false;
    // VARIABLE RATE SHADING on the water (2026-10-25; shaders/GlobeMesh.hlsl SV_ShadingRate): the
    // sea's pixels shared four a side where the pixel's footprint is wider than the chop texture's
    // texel, two a side nearer; foam, a held fold, the shore and the land keep their pixels. On a
    // device of tier 2; the A/B is `--set render.vrs=false`.
    bool vrs = true;
    // THE DRAWN BUILDINGS' SHADOW (shaders/TerrainRay.hlsli SunMapClear; BuildingLayer::RenderSunMap;
    // docs/VISIBILITY_FIELD.md 9): the solids drawn about the eye, seen once more from the sun, a
    // depth map per CELL of the origin lattice -- the eye's ground cell at the finest lattice level
    // whose cells are no wider than buildingShadowM (2000 m: level 12, 1.7 km), then its parents,
    // the span doubling each level, buildingShadowLevels of them (5: out to 27 km), every one at
    // the same texel count, so a tower shades the street, the quay and the tower beside it with the
    // sun's own penumbra, at a grain that grows with the distance as the pixel's does.
    // buildingShadowM 0 or buildingShadowLevels 0 = no maps, and the buildings cast nothing.
    // buildingShadowView: an instrument -- the buildings' faces drawn as their maps' clear share
    // (red; blue marks a face), for the gate's coverage count.
    double buildingShadowM = 2000.0;
    int buildingShadowLevels = 5;
    bool buildingShadowView = false;
};

// THE DEBUG VIEWS of the globe's surface (shaders/DebugView.hlsl, GlobeLayer.h DebugView): the
// mesh and what it carries read as geometry and as numbers. `none` and no wire draws the shipped
// picture through the shipped pipeline; anything else draws through its own, built only then.
// G in the window steps through the views.
struct DebugSection {
    int view = 0;                     // none | flat | edges | normals | height | depth | meshlets |
                                      // albedo | residency | residency.height | residency.landsea
    float wirePx = 0.0f;              // the triangles' edges drawn this wide (pixels); 0 = none
    float wireColor[4] = {1.0f, 0.85f, 0.2f, 1.0f};
    int wireBy = 0;                   // colour | edge | depth: the wire's own colour or a hue
    float contourM = 1.0f;            // height: a line every this many metres, a heavier every fifth
    float depthNearM = 10.0f, depthFarM = 1.0e6f;   // depth: the log ramp's ends
    float edgeRefM = 1.0f;            // edges: the length the hues are centred on; one octave a hue step
};

// The whole document as one prototype (the lists carry no fields).
struct SceneDocument {
    std::string base;                 // the inherited scene file; "" = none
    SceneSection scene;
    DataSection data;
    PlaceSection place;
    TimeSection time;
    SunSection sun;
    AirSection air;
    SeaSection sea;
    WaterSection water;
    StreamingSection streaming;
    RenderSection render;
    DebugSection debug;
    CaptureSection capture;
    HudSection hud;
    RailsSection rails;
    PruneSection prune;
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
const Schema& InterestSchema();
const Schema& NodeSchema();
// M12 step 5e: the rail key (scene/Rail.h reads a rail file's keys through it) and the slice
// plane's typed table (scene/effects/SlicePlane.h's Props()).
const Schema& RailKeySchema();
const Schema& SlicePlaneSchema();

}  // namespace ga::scene
