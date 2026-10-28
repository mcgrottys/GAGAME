// SceneSchema - the sections' schemas, built once over static prototypes (M12 step 5a).
#include "scene/SceneSchema.h"

#include "scene/WaterComponent.h"

namespace ga::scene {

namespace {

// THE PROTOTYPES: static storage, so a Schema's offsets and DefaultOf() read from a living
// instance for the program's life. One document holds every section; the list elements have
// prototypes of their own.
SceneDocument kDoc;
IncludeEntry kInclude;
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
BuildingsLayerProps kBuildings;
NodeProps kNode;
ToolProps kTool;
StationProps kStation;
SourceProps kSource;

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
            .Bind("view", p.view, "the start camera: a name in views[]", R)
            .Bind("windowDepth", p.windowDepth, Q::Dimensionless, "1",
                  "how many windows deep the view reaches through facing gates at most (0: as deep as "
                  "the screen shows, a window under two pixels ending it): each world seen is walked and "
                  "its tiles wanted, so this is the gates' frame cost", R);
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
            .BindPath("river", p.river, "the rivers: the water composite's river layer (reaches meeting the sea, their gauges and tide stations; sim/RiverModel.h; \"\" = none)", R)
            .BindPath("edits", p.edits,
                      "the hand polygons, a geojson: land, water and structures that override every survey, in the "
                      "height stack, the land/sea mask and the solver's bed (\"\" = none)", R)
            .BindPath("route", p.route, "the chart's channel route, a geojson (\"\" = none)", R)
            .Bind("tideFocus", p.tideFocus,
                  "the tide station whose clock and datum link the water takes (\"\" = the nearest the solver's anchor)", R);
        return sc;
    }();
    return *s;
}

const Schema& PlaceSchema() {
    static const Schema* s = [] {
        auto& p = kDoc.place;
        Schema* sc = new Schema("place", &p);
        sc->BindLonLat("anchor", p.anchor, "the place world.flat stands about, [lon, lat] degrees: every {x, alt, z} without a place of its own is in its tangent plane, exactly", R);
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

const Schema& ClockSchema(const char* name, ClockProps& p) {
    Schema* sc = new Schema(name, &p);
    sc->Bind("start", p.start, "its instant at the scene's start: \"now\", UTC text or unix seconds; \"\" = the bound clock's", R)
        .Bind("rate", p.rate, Q::Dimensionless, "1", "its pace against the bound clock's (1 = the same, 0 = held)", R)
        .Bind("bind", p.bind, "the clock it follows (scene/SceneClocks.h): time, water.clock, sun.clock or sun.earth.clock; \"\" = time", R)
        .Bind("offset", p.offset, "added to its instant: seconds or a run of number+unit (\"-3h\", \"1h30m\", \"2d\")", R);
    return *sc;
}

const Schema& EarthSchema() {
    static const Schema* s = [] {
        auto& p = kDoc.sun.earth;
        Schema* sc = new Schema("sun.earth", &p);
        sc->Bind("at", p.at, Q::Length, "m", "the Earth's centre; the sun is a light at 0,0,0", R)
            .Bind("axis", p.axis, Q::Dimensionless, "", "the Earth's spin axis in that frame", R)
            .Bind("spin", p.spin, Q::Angle, "deg", "the Earth's turn about its axis", R)
            .Nest("clock", ClockSchema("sun.earth.clock", p.clock), &p.clock,
                  "ephemeris: the clock of the Earth's turn (the time of day), apart from the sun's");
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
            .Nest("earth", EarthSchema(), &p.earth, "the Earth's place around the sun")
            .Nest("clock", ClockSchema("sun.clock", p.clock), &p.clock,
                  "ephemeris: the clock of the sun's place on the ecliptic (declination and distance: the season), "
                  "apart from the Earth's turn");
        return sc;
    }();
    return *s;
}

const Schema& AirSchema() {
    static const Schema* s = [] {
        auto& p = kDoc.air;
        Schema* sc = new Schema("air", &p);
        sc->Bind("aod550", p.aod550, Q::Dimensionless, "",
                 "the day's aerosol optical depth at 550 nm, the air's one row that changes with the "
                 "day (src/scene/Air.h); < 0 = the planet's typical value (Earth 0.10, the "
                 "Maritime Aerosol Network's open-ocean mean; Mars's dust 0.5)", R)
            .Bind("angstrom", p.angstrom, Q::Dimensionless, "",
                  "the aerosol's Angstrom exponent: its optical depth scales as (lambda / 550 nm)"
                  "^-angstrom per channel, single-scattering albedo and phase unchanged; < 0 = the "
                  "planet's typical value (Earth 0.6, the maritime mean; Mars's dust 0)", R)
            .Bind("exposure", p.exposure, Q::Dimensionless, "",
                  "the tonemap's scale; 0 = the law: 1 / the luminance of a white level surface "
                  "under a zenith sun through this air (its sun and its sky), so a sunlit white "
                  "is the display's white; > 0 overrides it", R);
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
            .Bind("mllwToNavd", p.mllwToNavd, Q::Length, "m", "tide (m MLLW) + this = NAVD88", R);
        return sc;
    }();
    return *s;
}
const Schema& ImpactSchema() {
    static const Schema* s = [] {
        auto& p = kDoc.sea.impactor;
        Schema* sc = new Schema("sea.impactor", &p);
        sc->Bind("enabled", p.enabled, "an asteroid into the sea: its cavity's linear dispersive wave, everywhere the water is read (in `impacts`, the event stands; in `sea.impactor`, the I key is armed)", R)
            .Bind("lat", p.lat, Q::Angle, "deg", "where it lands", R)
            .Bind("lon", p.lon, Q::Angle, "deg", "where it lands", R)
            .Bind("diameterM", p.diameterM, Q::Length, "m", "the impactor's diameter", R)
            .Bind("speedMs", p.speedMs, Q::Dimensionless, "1", "its speed, m/s", R)
            .Bind("densityKgM3", p.densityKgM3, Q::Dimensionless, "1", "its density, kg/m^3", R)
            .Bind("angleDeg", p.angleDeg, Q::Angle, "deg", "its angle from the horizontal", R)
            .Bind("at", p.at, "the impact's instant: UTC text or unix seconds; \"\" = the water clock's first instant", R)
            .Bind("basinDepthM", p.basinDepthM, Q::Length, "m", "the ocean's depth at the impact and across the basin (the dispersion's one depth; the cavity cannot be deeper)", R)
            .Bind("energyShare", p.energyShare, Q::Dimensionless, "", "the share of the impactor's kinetic energy the cavity holds (Ward & Asphaug 2003: 0.094)", R)
            .Bind("terms", p.terms, Q::Dimensionless, "", "the wavenumber table's size, 16..4096; 0 = from reachM", R)
            .Bind("reachM", p.reachM, Q::Length, "m", "how far the sum must hold before its image returns (sets the terms: 12 reach / (pi R_D))", R)
            .Bind("exag", p.exag, Q::Dimensionless, "", "display exaggeration of the drawn wave (the globe, the bank); the hull and the solver read it raw", R);
        return sc;
    }();
    return *s;
}
const Schema& SeaSchema() {
    static const Schema* s = [] {
        auto& p = kDoc.sea;
        Schema* sc = new Schema("sea", &p);
        sc->Nest("storm", StormSchema(), &p.storm, "the sandbox sea-state override")
            .Nest("impactor", ImpactSchema(), &p.impactor, "the asteroid the I key drops at the point under the cursor, at the clock's instant (its lat, lon and at are ignored)")
            .Nest("datum", DatumSchema(), &p.datum, "the MLLW -> NAVD88 link")
            .BindBox("box", p.box, "where data.seastate's forecast stands when its file carries no box, [lon0, lat0, lon1, lat1] degrees; zeros = nowhere (the grid alone)", R);
        return sc;
    }();
    return *s;
}

const Schema& SweSchema() {
    static const Schema* s = [] {
        auto& p = kDoc.water.swe;
        Schema* sc = new Schema("water.swe", &p);
        sc->Bind("enabled", p.enabled, "the shallow-water solver; false = the analytic tide plane (--swe-off)", R)
            .Bind("riverTide", p.riverTide, "the river faces' tide deviation; false = zeroed (--swe-river-off)", R)
            .Bind("spinupH", p.spinupH, Q::Time, "h", "history integrated before the first frame (--swe-spinup)", R)
            .Bind("gain", p.gain, Q::Dimensionless, "1", "solved-current gain (--swe-gain)", H)
            .Bind("stepsPerFrame", p.stepsPerFrame, Q::Dimensionless, "1",
                  "the solver's substeps a frame before it starves (its clock re-anchors, its dynamics run time-dilated); a fast clock's record sets what it needs", R)
            .Bind("fineReachM", p.fineReachM, Q::Length, "m",
                  "the eye's demand for the solver's finest mip: its tiles within this reach of the eye's ground point; 0 = the whole window (stage 1d of one water on the tree)", R)
            .Bind("currentRefreshS", p.currentRefreshS, Q::Time, "s",
                  "how often (sim s) the wave solve reads the solver's current and re-keys (a solve each); past the run's length = the first current throughout", R)
            .Bind("riverQ", p.riverQ, Q::Dimensionless, "1", "the discharge at the first river face, m^3/s; < 0 = its gauge's in data.river (--river)", R)
            .BindEnum("bedWait", p.bedWait, {"none", "map", "whole"},
                      "what the spin-up waits for before its hour: nothing (the kernel reads whatever it reads before the first residency turn), the residency map (one turn: the coarsest resident mip), or the whole bed (every solver's domain read at mip 0)", R)
            .Bind("windowTiles", p.windowTiles, Q::Dimensionless, "1",
                  "the solver's window: this many tiles of 128 cells a side about the eye's start (the place's anchor), its corner on the global lattice of the cell's angle, its bed the one height stack's, its open sea the tide atlas's; 0 = no solver (stage 2 of one water on the tree)", R)
            .Bind("cellM", p.cellM, Q::Length, "m", "the solver's cell: one angle east and north, this many metres north-south (13.728 m = 4/9 arc-second)", R)
            .Bind("tailCells", p.tailCells, Q::Dimensionless, "1", "the tail across the ocean (stage 2b of one water on the tree): this many cells a side about the same anchor, the window whole in its central cells; 0 = no tail", R)
            .Bind("tailRank", p.tailRank, Q::Dimensionless, "1", "the tail's cell as a power of two of the window's (9: 512 window cells, 5 x 7 km at the Merrimack)", R)
            .Bind("telescopeLevels", p.telescopeLevels, Q::Dimensionless, "1", "the telescope (stage 2c): this many anchor levels between the tail and the window, each about the window's centre", R)
            .Bind("telescopeCells", p.telescopeCells, Q::Dimensionless, "1", "an anchor level's cells a side", R)
            .Bind("telescopeStep", p.telescopeStep, Q::Dimensionless, "1", "the anchor levels' ranks this far apart below the tail's (2: ranks 7, 5, 3 under a rank-9 tail)", R);
        return sc;
    }();
    return *s;
}
const Schema& BankSchema() {
    static const Schema* s = [] {
        auto& p = kDoc.water.bank;
        Schema* sc = new Schema("water.bank", &p);
        sc->Bind("flatBed", p.flatBed, "replace the bed with a flat floor (--flat-bed)", R)
            .Bind("flatBedNavd", p.flatBedNavd, Q::Length, "m NAVD88", "the floor's height", R)
            .Bind("column", p.column,
                  "the column plane: the sheet's labels scattered into Eulerian columns, a reader of an "
                  "overturning crest (WaterBank.hlsl CsBankScatter; the crest is drawn by the sheet itself)",
                  R)
            .Bind("columnTexel", p.columnTexel, Q::Length, "m",
                  "the finest texel the column ladder reaches: rings under ring 0 halve its 1.2 m until they reach it "
                  "(0.3 = two rings), each resident only where the ring above witnessed a fold",
                  R);
        return sc;
    }();
    return *s;
}
// THE WATER SECTION. Its wavefield, closures and fleet are WaterComponent's own tables nested over
// the document's WaterSceneConfig (`wave`): the component reads that struct, so the section and the
// component cannot say two different things about one key.
const Schema& WaterSectionSchema() {
    static const Schema* s = [] {
        auto& p = kDoc.water;
        Schema* sc = new Schema("water", &p);
        sc->Nest("clock", ClockSchema("water.clock", p.clock), &p.clock,
                 "the data's clock: the tide, the solver, the currents, the sea state, the wind, the cascades")
            .Bind("pixelWater", p.pixelWater, "shade the water per pixel (--no-pixel-water)", R)
            .Bind("foam", p.foam, Q::Dimensionless, "1", "sea-mode whitecap intensity (--foam)", H)
            .Bind("edgePx", p.edgePx, Q::Dimensionless, "px", "tessellated edge target (--edge-px)", H)
            .Bind("heightScale", p.heightScale, Q::Dimensionless, "1", "wave vertical exaggeration (--height-scale)", H)
            .Nest("swe", SweSchema(), &p.swe, "the shallow-water solver")
            .Nest("bank", BankSchema(), &p.bank, "the water bank")
            .Nest("wavefield", WavefieldTable(), &p.wave, "the solved wave field's window (data/wave_scene.json)")
            .Nest("closures", ClosuresTable(), &p.wave, "the kernel closures (data/wave_scene.json)")
            .Nest("fleet", FleetTable(), &p.wave, "the AIS fleet (data/wave_scene.json)");
        return sc;
    }();
    return *s;
}

const Schema& RenderSchema() {
    static const Schema* s = [] {
        auto& p = kDoc.render;
        Schema* sc = new Schema("render", &p);
        sc->Bind("keep", p.keep,
                 "the globe's picture kept while its inputs stand: a kept frame composites the cached land and "
                 "draws only the meshlets that had water (GlobeLayer.h keepOn)",
                 H)
            .Bind("keepSunGrainDeg", p.keepSunGrainDeg, Q::Angle, "deg",
                  "the sun's grain for the kept frame: the globe is redrawn when the sun has moved by more", H)
            .Bind("keepTideGrainM", p.keepTideGrainM, Q::Length, "m",
                  "the tide's grain for the kept frame: the globe is redrawn when the water level has moved by more", H)
            .Bind("shadowSteps", p.shadowSteps, Q::Dimensionless, "1",
                  "the terrain shadow's cast toward the sun over the height's max lane: the traversal's cap in "
                  "steps (0 = no terrain shadow)", H)
            .Bind("shadowRangeM", p.shadowRangeM, Q::Length, "m",
                  "how far the terrain shadow's cast reaches from the lit point", H)
            .Bind("reflectLand", p.reflectLand,
                  "the water's mirror ray marched over the height field: land it meets is reflected in place of the sky", H)
            .Bind("vrs", p.vrs,
                  "variable rate shading on the water: the sea's pixels shared four a side where the pixel's footprint "
                  "is wider than the chop texture's texel, two a side nearer; foam, a held fold, the shore and the land "
                  "keep their pixels (a device of tier 2)", H)
            .Bind("buildingShadowM", p.buildingShadowM, Q::Length, "m",
                  "the drawn buildings' shadow maps: the finest map is the eye's cell of the origin lattice at the "
                  "finest level whose cells are no wider than this (0 = the buildings cast no shadow)", H)
            .Bind("buildingShadowLevels", p.buildingShadowLevels, Q::Dimensionless, "1",
                  "how many levels of the lattice the shadow maps climb from the finest cell, the span doubling each "
                  "(0 = the buildings cast no shadow)", H)
            .Bind("buildingShadowView", p.buildingShadowView,
                  "an instrument: the buildings' faces drawn as their shadow maps' clear share (red) over a blue mark", H);
        return sc;
    }();
    return *s;
}

const Schema& StreamingSchema() {
    static const Schema* s = [] {
        auto& p = kDoc.streaming;
        Schema* sc = new Schema("streaming", &p);
        sc->Bind("tileBudget", p.tileBudget, Q::Dimensionless, "1", "hard cap on Google fetches per run (--tile-budget)", R)
            .Bind("poolTiles", p.poolTiles, Q::Dimensionless, "1", "the residency pool's ceiling in 64 KB tiles (32768 = 2 GB); smaller, the order's cut releases sooner", R)
            .Bind("fadeTurns", p.fadeTurns, Q::Dimensionless, "1",
                  "the turns a landing tile takes to come into focus: its map byte steps from its parent's "
                  "level to its own, and trilinear blends the two held mips (0 = the snap; at most 15)", R)
            .Bind("dayTiles", p.dayTiles, Q::Dimensionless, "1",
                  "cap on Google requests a UTC day, every request sent counted whatever came "
                  "back, every engine on the machine together (cache/google/day_<date>.json); "
                  "0 = no request that day",
                  R)
            .Bind("dayBytes", p.dayBytes, Q::Dimensionless, "1",
                  "cap on the bytes of Google tiles landed a UTC day, every engine on the machine "
                  "together; 0 = no request that day",
                  R)
            .Bind("googleZoom", p.googleZoom, Q::Dimensionless, "1",
                  "the finest zoom the Google source may be asked for, held to 0..19; each step "
                  "finer is four times the tiles (14 = the source as it was; any other value "
                  "paints its own tree)",
                  R)
            .Bind("fetchThreads", p.fetchThreads, Q::Dimensionless, "1",
                  "the Google fetch threads off the loaders: a tile not on disk is queued and the paint goes on "
                  "without it; -1 = four in a window, none headless; 0 = the fetch in the paint", H)
            .Bind("offline", p.offline,
                  "no network this run: the cached tiles serve and nothing is asked (also what a run "
                  "without a session falls back to)", R)
            .Bind("tilePaint", p.tilePaint,
                  "sources paint a tile at a time where they can; false = texel by texel (the A/B: byte for byte one)", H)
            .Bind("predictEvery", p.predictEvery, Q::Dimensionless, "frames", "prefetch-walk cadence (--predict-every)", H)
            .Bind("directStorage", p.directStorage, "NVMe -> GPU tile reads (--no-direct-storage)", R)
            .Bind("colorTrees", p.colorTrees, "colour and height pages from the trees (--no-color-trees)", R)
            .Bind("gisGate", p.gisGate, "the vector land/sea gate on the bed (--no-gis-gate)", R)
            .BindPath("gisWater", p.gisWater,
                      "the planet's inland water as GAWATR01 (harvester/harvest_osm_water.py: OSM water polygons classed sea, impounded, reach), read by 1-degree cell -- in place of the New England NHD carve (\"\" = that carve)", R)
            .BindPath("gisLand", p.gisLand,
                      "the global coast: OSM's land polygons as GALAND01 (harvester/harvest_land_polygons.py); "
                      "\"\" = the New England survey alone", R)
            .Bind("gisGrainM", p.gisGrainM, Q::Length, "m",
                  "the survey's grain: the finest level the land/sea mask has anything of its own "
                  "for; a mask tile finer than it is its parent, magnified -- never painted, stored "
                  "or loaded (HIERARCHY 4.20); 0 = every grain. Also the survey's standing as a "
                  "WITNESS of the water's edge: a height layer finer than it draws the edge by its "
                  "sign against the live level wherever it has a value (GisMaskSource SetHeight)",
                  R)
            .Bind("seafloor", p.seafloor, "the global seafloor relief source (--no-seafloor)", R)
            .Bind("exposure", p.exposure, "the swell-exposure page (--no-exposure)", R)
            .Bind("holdMargin", p.holdMargin, Q::Dimensionless, "1",
                  "the order's hold margin: a held tile, and a tile above a held tile, count for "
                  "this times their measure, so a tile gives its slot up only to one larger on the "
                  "screen by more (HIERARCHY 4.19); 1 is the order without it",
                  R)
            .Bind("treeRoot", p.treeRoot,
                  "the folder the tile trees live in; a scratch folder keeps a tool's paints and "
                  "packs out of the real cache",
                  R);
        return sc;
    }();
    return *s;
}

const Schema& SourceSchema() {
    static const Schema* s = [] {
        Schema* sc = new Schema("source", &kSource);
        sc->Bind("name", kSource.name, "a label (\"\" = the file's own name)", R)
            .BindPath("file", kSource.file, "one raster: a GeoTIFF, or a PNG / JPEG beside its world file", R)
            .BindPath("folder", kSource.folder, "a folder whose files matching `match` are one source", R)
            .Bind("match", kSource.match, "the folder's pattern, * and ? (\"\" = every file)", R)
            .BindPath("manifest", kSource.manifest,
                      "a manifest of raw rows (the harvester's form: crs, tiles, their bounds), one source", R)
            .Bind("kind", kSource.kind,
                  "colour | height | seastate (a sea-state file: its forecast and buoys at its box) | composite (a GACOMP01 "
                  "product of many sources, docs/FIDELITY.md: its tiers join as rasters); \"\" = the pixels decide (8-bit, 3 or 4 channels: colour; one "
                  "channel of 16-bit or float: height)", R)
            .Bind("crs", kSource.crs, "EPSG:nnnn, read only where the file carries none", R)
            .Bind("over", kSource.over, Q::Dimensionless, "1",
                  "the stack order's first key, ascending upward (every built-in source is 0); "
                  "the second is the grain, the coarser under the finer", R)
            .Bind("feather", kSource.feather, Q::Length, "m",
                  "the edge of the whole softened over this far (smoothstep); 0 = a hard edge", R)
            .Bind("unit", kSource.unit, "a height's unit (m, ft, ftUS), read only where the file names none", R)
            .Bind("datum", kSource.datum,
                  "a height's vertical datum (NAVD88, MSL, EGM2008 ...), read only where the file names none", R)
            .Bind("offset", kSource.offset, Q::Length, "m",
                  "added to a height whose datum is not the engine's (NAVD88): the separation at the "
                  "place; such a file is refused without it, and a file on NAVD88 takes none", R)
            .Optional()   // absent is not 0: the builder must not complete it (SourceProps::hasOffset)
            .Bind("captureLevel", kSource.captureLevel, Q::Length, "m",
                  "THE WATER LEVEL A PHOTO WAS FLOWN AT, metres above captureDatum (compose/Sources.h "
                  "LevelGateSource): its pixel is a picture of the ground only where the ground stands above "
                  "it, so under today's water the tree shows only such pixels -- the photo's weight is gated by "
                  "the height tree (1 above, 0 below, a feather of the height's own spread between), its colour "
                  "untouched; absent, with no captureDatum, the photo is not gated (today's weight)", R)
            .Optional()   // absent is not 0 (SourceProps::hasCaptureLevel)
            .Bind("captureDatum", kSource.captureDatum,
                  "the datum captureLevel is measured from, resolved at the place as the tides are: \"\" or "
                  "NAVD88 (the engine's frame), MLLW (sea.datum: the place's link), MSL (the focus station's "
                  "fitted mean above MLLW); alone = 0 m on it; a datum the ladder does not hold refuses the "
                  "photo by name", R);
        return sc;
    }();
    return *s;
}

const Schema& DebugSchema() {
    static const Schema* s = [] {
        auto& p = kDoc.debug;
        Schema* sc = new Schema("debug", &p);
        sc->BindEnum("view", p.view,
                     {"none", "flat", "edges", "normals", "height", "depth", "meshlets", "albedo", "residency",
                      "residency.height", "residency.landsea"},
                     "the surface drawn as: the picture (none), flat grey, its triangles' size (one hue an octave "
                     "about edgeRefM), its geometric normal in east/north/up, its height (hypsometric, a line every "
                     "contourM), its distance (a log ramp), the meshlets, the raw colour, or what the residency lets "
                     "the colour / height / land-sea mask read (shaders/ResidencyLens.hlsl)",
                     R)
            .Bind("wirePx", p.wirePx, Q::Dimensionless, "1", "pixels: the triangles' edges drawn over the view this wide; 0 = none", R)
            .BindColor("wireColor", p.wireColor, "the wire's colour when wireBy is colour", R)
            .BindEnum("wireBy", p.wireBy, {"colour", "edge", "depth"},
                      "the wire's colour: its own, a hue by its triangle's size, or by distance", R)
            .Bind("contourM", p.contourM, Q::Length, "m", "height: the contour interval", R)
            .Bind("depthNearM", p.depthNearM, Q::Length, "m", "depth: where the log ramp starts", R)
            .Bind("depthFarM", p.depthFarM, Q::Length, "m", "depth: where it ends", R)
            .Bind("edgeRefM", p.edgeRefM, Q::Length, "m", "edges: the triangle size the hue wheel is centred on", R);
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
            .Nest("settle", SettleSchema(), &p.settle, "the still's hold")
            .Bind("residencyAudit", p.residencyAudit, Q::Dimensionless, "frames",
                  "the residency audit every N turns: each page tenant's residency bytes against "
                  "the tiles mapped and landed, and the invalidations applied (--res-audit; 0 = off)", R);
        return sc;
    }();
    return *s;
}

const Schema& MinimapSchema() {
    static const Schema* s = [] {
        auto& p = kDoc.hud.minimap;
        Schema* sc = new Schema("hud.minimap", &p);
        sc->Bind("enabled", p.enabled, "the second eye in the bottom-right corner: drag to turn the planet, wheel to zoom, Reset to return", R)
            .Bind("follow", p.follow, "the entity the marker stands on and Reset centres over", R)
            .Bind("size", p.size, Q::Dimensionless, "1", "the square's side as a fraction of the target's height", R)
            .Bind("margin", p.margin, Q::Dimensionless, "px", "the gap to the bottom-right corner", R)
            .Bind("homeAltM", p.homeAltM, Q::Length, "m", "Reset: the eye this far above the followed entity", R)
            .Bind("layers", p.layers, "the layers this eye draws, by name, space-separated (globe vessels buildings sky); empty: that list", R)
            .Bind("minAltM", p.minAltM, Q::Length, "m", "the zoom's floor above the sea", R)
            .Bind("travelAheadM", p.travelAheadM, Q::Length, "m",
                  "fast travel: the sphere opens this far ahead of the vehicle", R)
            .Bind("travelRadiusM", p.travelRadiusM, Q::Length, "m", "fast travel: the sphere's full radius", R)
            .Bind("travelGrowSec", p.travelGrowSec, Q::Time, "s", "fast travel: the sphere reaches its full radius in this long", R)
            .Bind("travelLifeSec", p.travelLifeSec, Q::Time, "s", "fast travel: the sphere is gone this long after it opened", R);
        return sc;
    }();
    return *s;
}
const Schema& HudSchema() {
    static const Schema* s = [] {
        auto& p = kDoc.hud;
        Schema* sc = new Schema("hud", &p);
        sc->Bind("fps", p.fps, "the frame rate, top left", R)
            .Nest("minimap", MinimapSchema(), &p.minimap, "the globe in the corner");
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
            .BindPath("file", p.file, "a rail file of the scene's own (scenes/rails/*.json's shape), flown instead of the named one when capture.railDir is set (2026-10-19: scenes/rails/mavericks_lineup.json)", R)
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
            .Bind("fromLat", kGate.fromLat, Q::Angle, "deg",
                  "the latitude of the place the box stands at (with fromLon; absent: the scene's own frame)", R)
            .Optional()
            .Bind("fromLon", kGate.fromLon, Q::Angle, "deg", "the longitude of that place", R)
            .Optional()
            .Bind("at", kGate.at, "the box's centre and heading in its place's flat frame: {x, alt, z, az}", R)
            .Optional()
            .Bind("size", kGate.size, Q::Length, "m", "the box: across, up, and along its heading", R)
            .Bind("radius", kGate.radius, Q::Length, "m",
                  "a sphere of this radius about the centre instead of the box (0: the box)", R)
            .Optional()
            .Bind("toLat", kGate.toLat, Q::Angle, "deg",
                  "the destination's latitude (with toLon; absent: the scene's own frame)", R)
            .Optional()
            .Bind("toLon", kGate.toLon, Q::Angle, "deg", "the destination's longitude", R)
            .Optional()
            .Bind("toAt", kGate.toAt, "where the box's centre comes out, in the destination's flat frame: {x, alt, z}", R)
            .Optional()
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

static scene::FormProps kForm;
const Schema& FormSchema() {
    static const Schema* s = [] {
        Schema* sc = new Schema("entity.form", &kForm);
        sc->Bind("vessel", kForm.vessel, "this form's hull kind (VesselRegistry)", R)
            .Bind("mesh", kForm.mesh, "a glTF binary (.glb) drawn over this form's physics; empty: the spec's boxes", R)
            .Bind("meshScale", kForm.meshScale, Q::Dimensionless, "1", "the mesh file's units to metres", R)
            .Bind("meshYaw", kForm.meshYaw, Q::Angle, "deg", "the turn about +y that puts the model's front on +z", R);
        return sc;
    }();
    return *s;
}

const Schema& EntitySchema_() {
    static const Schema* s = [] {
        Schema* sc = new Schema("entity", &kEntity);
        sc->Bind("name", kEntity.name, "the entity's name", R)
            .Bind("vessel", kEntity.vessel, "the hull kind (VesselRegistry; --boat); with `forms`, the first form's", R)
            .List("forms", &FormSchema(), "ONE body with several forms (sim/VesselSpec.h Form): T / Y take the next; each a hull kind and its model", false)
            .Bind("at", kEntity.at, "the spawn, in the flat world frame (--campos); `az` is the bow's compass heading (absent: north, as every hull always spawned)", R)
            .BindEnum("controller", kEntity.controller, {"helm", "fixed"},
                      "the keyboard helm, or fixed throttles and helm (--boat-drive)", H)
            .Bind("throttle", kEntity.throttle, Q::Dimensionless, "1", "fixed: every thruster's throttle", H)
            .Bind("steer", kEntity.steer, Q::Dimensionless, "1", "fixed: the commanded steering", H)
            .Bind("mesh", kEntity.mesh, "a glTF binary (.glb) drawn over the physics, its frame the spec's datum (y up, +z the bow); empty: the spec's own boxes", R)
            .Bind("meshScale", kEntity.meshScale, Q::Dimensionless, "1", "the mesh file's units to metres (a centimetre model: 0.01)", R)
            .Bind("meshYaw", kEntity.meshYaw, Q::Angle, "deg", "the turn about +y that puts the model's front on +z", R);
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
const Schema& BuildingsLayerSchema() {
    static const Schema* s = [] {
        Schema* sc = new Schema("layer.buildings", &kBuildings);
        sc->Bind("levelHeight", kBuildings.levelHeight, Q::Length, "m",
                 "what one tagged floor (building:levels) stands for, where no height is tagged", R)
            .Bind("defaultHeight", kBuildings.defaultHeight, Q::Length, "m",
                  "a building with neither height nor floors tagged: the scene's assumption", R)
            .Bind("radius", kBuildings.radius, Q::Length, "m",
                  "cells of solids are streamed in within this distance of the eye (its altitude counted) and "
                  "dropped beyond 1.25 times it", R)
            .BindPath("lod", kBuildings.lod,
                      "the folded tree of buildings (docs/BUILDING_LOD.md), built by --tool building-lod; \"\" = none", R)
            .Bind("lodPixels", kBuildings.lodPixels, Q::Dimensionless, "px",
                  "a building is drawn on its own from this many pixels across (faded in to twice that); narrower ones are the field's", R)
            .Bind("boxPixels", kBuildings.boxPixels, Q::Dimensionless, "px",
                  "a building is drawn as its moment box (a hexahedron) from this many pixels across, and as one lit "
                  "point below (from lodPixels)", R)
            .Bind("shapePixels", kBuildings.shapePixels, Q::Dimensionless, "px",
                  "a building or road piece is drawn as its own form from this many pixels across, and as its "
                  "moment box (the same volume, centroid and heading) below; 1 = always its form. Tokyo at 9 km: 16.5 ms of buildings at 1, 2.3 ms at 6, the stills alike", R)
            .Bind("pageMB", kBuildings.pageMB, Q::Dimensionless, "1",
                  "the tree's loaded pages' budget on the CPU; over it, the page worth the fewest pixels on screen "
                  "goes first, and never to make room for one worth fewer (the coarse levels are so the floor)", R)
            .Bind("standMB", kBuildings.standMB, Q::Dimensionless, "1",
                  "the stands' record pool on the GPU: every building a landed stand holds, 64 bytes each, written once; "
                  "a walk names ranges of it and the GPU decides each record's form. Full: a stand waits for room", R)
            .Bind("marginDeg", kBuildings.marginDeg, Q::Angle, "deg",
                  "the walk's cone reaches this far past the view's edge, so what a turn brings into view has been "
                  "asked for and stood already", R)
            .Bind("field", kBuildings.field,
                  "the buildings narrower than a pixel as scalars on the ground (compose/BuildingField.h): their walls "
                  "seen at an angle; needs `lod`", R)
            .Bind("laneWidth", kBuildings.laneWidth, Q::Length, "m",
                  "a road in the tree (sources of kind \"roads\", docs/ROADS.md) is as wide as its file says, else its "
                  "lanes times this", R)
            .Bind("defaultLanes", kBuildings.defaultLanes, Q::Dimensionless, "1",
                  "a road with neither width nor lanes tagged: the scene's assumption", R)
            .Bind("pathWidth", kBuildings.pathWidth, Q::Length, "m",
                  "a footway, path, cycleway, track or steps with no width tagged", R)
            .Bind("rampGrade", kBuildings.rampGrade, Q::Dimensionless, "1",
                  "the approaches to a bridge fall at this grade (rise over run) from the deck to the ground, along the "
                  "ways joining its ends through junctions of two", R)
            .Bind("deckDepth", kBuildings.deckDepth, Q::Length, "m",
                  "a bridge's deck is this deep: its surveyed clearance is to the underside, the road rides on top", R)
            .Bind("bridgeClearance", kBuildings.bridgeClearance, Q::Length, "m",
                  "a bridge no clearance survey (sources of kind \"bridges\") knows stands this high over what it crosses", R)
            .Bind("kerb", kBuildings.kerb, Q::Length, "m",
                  "a road's surface stands this much over the composed ground, and its mass as much under it", R);
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

const Schema& StationSchema() {
    static const Schema* s = [] {
        Schema* sc = new Schema("station", &kStation);
        sc->Bind("name", kStation.name, "the station's id in its file", R)
            .Bind("lat", kStation.lat, Q::Angle, "deg", "its latitude", R)
            .Bind("lon", kStation.lon, Q::Angle, "deg", "its longitude", R);
        return sc;
    }();
    return *s;
}

const Schema& PruneSchema() {
    static const Schema* s = [] {
        auto& p = kDoc.prune;
        Schema* sc = new Schema("prune", &p);
        sc->BindEnum("mode", p.mode, {"list", "retire", "purge"},
                     "the tree-prune tool: list changes nothing; retire moves stale tag folders "
                     "into <root>\\.retired\\<UTC stamp>; purge deletes retired batches", R)
            .BindPath("root", p.root, "the trees folder (\"\" = cache\\trees)", R)
            .Bind("ageDays", p.ageDays, Q::Time, "day",
                  "a stamp older than this is STALE; an unstamped folder gets twice it", R)
            .Bind("purgeDays", p.purgeDays, Q::Time, "day",
                  "purge deletes the retired batches older than this", R)
            .BindPath("confirm", p.confirm,
                      "retire and purge act only when this is the root's full path as the "
                      "listing printed it", R)
            .Bind("node", p.node, "retire only this node's stale folders (wave.field, earth.color ...); \"\" = every node", R);
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
            .Nest("place", PlaceSchema(), &kDoc.place, "the scene's place: the anchor world.flat stands about")
            .Nest("time", TimeSchema(), &kDoc.time, "the scene clock")
            .Nest("sun", SunSchema(), &kDoc.sun, "the sun")
            .Nest("air", AirSchema(), &kDoc.air, "the air of the day: its aerosol, the picture's white")
            .Nest("sea", SeaSchema(), &kDoc.sea, "the sea state and the datum")
            .Nest("water", WaterSectionSchema(), &kDoc.water, "the water")
            .Nest("streaming", StreamingSchema(), &kDoc.streaming, "residency: scene state")
            .Nest("render", RenderSchema(), &kDoc.render, "the renderer: the kept frame and its grains")
            .Nest("debug", DebugSchema(), &kDoc.debug, "the debug views of the globe's surface")
            .List("sources", &SourceSchema(), "the scene's sources by name: a derived scene ADDS to its base's (merged in place by name, `remove: true` drops one), as every named list; their order is StackOrder's (`over`, then the coarser under the finer), not the list's")
            .List("stations", &StationSchema(), "where a station of the data stands when its file does not say; by name, merged as every named list")
            .List("impacts", &ImpactSchema(), "the asteroids into the sea, each an event with its own instant (sim/ImpactField.h: the record the I key appends to)", false)
            .Nest("capture", CaptureSchema(), &kDoc.capture, "headless capture")
            .Nest("hud", HudSchema(), &kDoc.hud, "the windshield: readouts and the minimap on the glass")
            .List("views", &ViewSchema_(), "the cameras, by name")
            .Nest("rails", RailsSchema(), &kDoc.rails, "the camera rails")
            .List("portals", &PortalSchema_(), "the Droste links, by name")
            .List("gates", &GateSchema_(), "the gates to other places, by name: a cuboid of `size`, or a sphere by `radius`")
            .List("entities", &EntitySchema_(), "the vessels, by name")
            .List("interests", &InterestSchema_(), "the subjects and places whose water stays resident for the views that name them, by name")
            .List("effects", &EffectSchema(), "the paper visuals, by name, typed", true, "effect", "type")
            .List("layers", &LayerEntrySchema(), "the layers in registration order, typed by name", true, "layer", "name")
            .List("nodes", &NodeSchema_(), "typed nodes (plugins)", true, "component", "type")
            .Nest("prune", PruneSchema(), &kDoc.prune, "the tree-prune tool's keys")
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
    LayerSchemas().Register("buildings", [] { return &BuildingsLayerSchema(); });
}

JsonValue DefaultDocument() {
    return PropSet::Defaults(SceneFileSchema(), &kDoc).ToJson();
}

}  // namespace ga::scene
