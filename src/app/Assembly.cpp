// ================================================================================================
//  Assembly - Assemble(): main()'s boot sequence as one function (M12 step 1c).
//
//  The body is main.cpp's assembly span at 712679a, VERBATIM, under the aliases Assembly.h
//  describes. The only lines that changed shape: the block-level declarations (now members, so
//  a declaration line is gone and a runtime initialiser is an assignment), the four early exits
//  (exitCode + nullptr instead of `return`), activeGlobe's pointer hand-off, and the two
//  std::optional sources (emplaced). The three helpers in the anonymous namespace moved with the
//  span because only this code calls them. Comments stayed with the code they describe.
// ================================================================================================
#include "core/Planet.h"
#include "app/Assembly.h"

#include <filesystem>

#include "compose/ColorStackSource.h"
#include "compose/ExposureSource.h"
#include "compose/GisMask.h"
#include "compose/HeightStackSource.h"
#include "compose/TileTree.h"
#include "compose/TileArchive.h"
#include "compose/TileIndex.h"
#include "hal/TileStream.h"
#include "compose/ComposeTree.h"
#include "compose/DomainSource.h"
#include "core/CurrentFieldLoader.h"
#include "core/GeoGridLoader.h"
#include "hal/Gpu.h"
#include "core/Image.h"
#include "hal/PixEvents.h"
#include "hal/Tenant.h"
#include "hal/TileAtlas.h"
#include "core/Window.h"
#include "render/Renderer.h"
#include "scene/FieldSet.h"
#include "scene/GisLayer.h"
#include "scene/GlobeLayer.h"
#include "scene/HudLayer.h"
#include "scene/MarkerLayer.h"
#include "scene/GulfLayer.h"
#include "scene/SeaLayer.h"
#include "scene/SkyLayer.h"
#include "scene/WaterBankLayer.h"
#include "scene/TideLayer.h"
#include "compose/Compositor.h"
#include "compose/Exchange.h"
#include "compose/GisStencil.h"
#include "compose/Projections.h"
#include "compose/Sources.h"
#include "compose/WaterAtlas.h"
#include "core/Json.h"
#include "core/GaAst.h"
#include "scene/BuildingLayer.h"
#include "scene/VesselLayer.h"
#include "core/TileProviders.h"
#include "core/SceneConfig.h"
#include "scene/SceneReload.h"
#include "sim/GlobeModel.h"
#include "sim/CurrentModel.h"
#include "sim/SeaState.h"
#include "sim/Stations.h"
#include "sim/SweSolver.h"
#include "sim/TideModel.h"
#include "sim/WaterTerms.h"
#include "sim/WaveField.h"
#include "app/Options.h"
#include "app/Tools.h"

#include <algorithm>
#include <map>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

using namespace ga;
using namespace ga::app;

namespace {

// THE COMPOSITE'S TIERS (docs/FIDELITY.md section 3): a `sources` entry of kind composite names a
// GACOMP01 file; a raster layer's tiers become raster entries -- a folder (relative to the file),
// its pattern, its unit and datum and offset, its `over` -- named "<entry or layer>.t<grain>". A
// tier that declares none of its own takes the composite's unit and datum. A point or field layer
// carries no tiers: the scene's data key names its `file`.
std::vector<RasterEntry> CompositeTiers(const scene::SourceProps& s) {
    std::vector<RasterEntry> out;
    std::ifstream f(s.file, std::ios::binary);
    std::string text((std::istreambuf_iterator<char>(f)), std::istreambuf_iterator<char>());
    std::string err;
    const JsonValue root = JsonParser::Parse(text, &err);
    if (!f || !err.empty() || root.Str("format") != "GACOMP01") {
        Log("[composite] %s: not a GACOMP01 composite (%s); the scene goes on without it", s.file.c_str(),
            !f ? "cannot be read" : !err.empty() ? err.c_str() : "its format says otherwise");
        return out;
    }
    const std::string dir = s.file.substr(0, s.file.find_last_of("/\\") + 1);
    const std::string layer = s.name.empty() ? root.Str("layer") : s.name;
    const JsonValue* tiers = root.Get("tiers");
    if (!tiers) return out;
    for (const JsonValue& t : tiers->arr) {
        RasterEntry e;
        e.folder = dir + t.Str("folder");
        e.match = t.Str("match", "*.tif");
        char g[32];
        snprintf(g, sizeof(g), ".t%g", t.Num("grain_m", 0));
        e.name = layer + g;
        e.kind = root.Str("kind") == "height" ? "height" : t.Str("kind");
        // The scene's `over` alone: a tier stacks by its grain among every height the scene reads
        // (StackOrder, the coarser under the finer). A tier's own rank (`over`, which files made before
        // 2026-10-14 carry) put the composite's 463 m gap fill over the scene's 13.7 m survey, and the
        // river showed ETOPO's 15" blocks wherever the 3.4 m tier had no data (Mark's flats and wall).
        e.over = s.over;
        e.feather = s.feather;
        e.unit = t.Str("unit", root.Str("unit").c_str());
        e.datum = t.Str("datum", root.Str("datum").c_str());
        // A tier off the engine's datum carries the separation it measured (offset_m); one on it, none.
        e.hasOffset = t.Get("offset_m") != nullptr && e.datum != "NAVD88";
        e.offset = e.hasOffset ? t.Num("offset_m", 0) : 0.0;
        // The entry's capture level is every tier's: one flight, one water.
        e.captureLevel = s.captureLevel;
        e.captureDatum = s.captureDatum;
        e.hasCaptureLevel = s.hasCaptureLevel;
        out.push_back(std::move(e));
    }
    Log("[composite] %s: layer %s, %zu tiers", s.file.c_str(), layer.c_str(), out.size());
    return out;
}

// M5c: the MLLW -> NAVD88 join from CO-OPS station datums instead of a hand constant. Use the
// focus station's own NAVD link when CO-OPS publishes one; otherwise transfer via the relation
// NAVD88 = local MSL + delta, calibrated from the NEAREST station on the open coast that carries
// both datums (NavdAboveMsl).

// M9n: NAVD88 zero above LOCAL MSL, in metres, from published CO-OPS station datums -- the same
// staff carries MSL and NAVD88, so their difference is data, not an assumption. Boston: MSL
// 2.660, NAVD88 2.752 -> +0.092. Returns false when no station in the table publishes both.
//
// The bed's ETOPO layers are MSL-referenced and need the NEGATIVE of this to reach NAVD88; the
// tide's MLLW link is a different offset from the same table. One derivation, two consumers.
//
// WHICH STATION (2026-10-21, the one atlas): the link is regional -- NAVD88 departs from local MSL by
// a few centimetres that change along a coast (Boston +0.09 m, Sewells Point -0.08) -- so it is the
// NEAREST witness to the place that publishes both, on the open coast: a station up a river
// (riverKm > 0) stands on a river's mean, which drifts with the discharge. The table was New
// England's alone until the stations became one global set; "the smallest |delta|" it chose by then
// picked Virginia's for the Merrimack.
bool NavdAboveMsl(const TideModel& model, double latDeg, double lonDeg, double* outM, std::string* from) {
    constexpr double kD2R = 3.14159265358979323846 / 180.0;
    double bestD2 = 0.0, best = 0.0;
    bool got = false;
    for (size_t i = 0; i < model.Count(); ++i) {
        const TideStation& s = model.S(i);
        if (s.mllwMinusNavdM <= -900.0 || s.riverKm > 0.0) continue;
        const double dLat = s.lat - latDeg, dLon = (s.lon - lonDeg) * std::cos(latDeg * kD2R);
        const double d2 = dLat * dLat + dLon * dLon;
        if (!got || d2 < bestD2) {
            bestD2 = d2;
            best = -s.mllwMinusNavdM - s.meanMllwM;   // NAVD88 zero above local MSL
            got = true;
            if (from) *from = s.name;
        }
    }
    if (got && outM) *outM = best;
    return got;
}

// THE CAPTURE LEVEL ON THE ENGINE'S FRAME (compose/Sources.h LevelGateSource): a scene source's
// `captureLevel` metres above its `captureDatum`, resolved at the place as the tides are -- MLLW by
// the place's link (`datumOff`: a height in MLLW + it = NAVD88, ResolveDatum below), MSL by the focus
// station's fitted mean above MLLW (TideStation::meanMllwM), NAVD88 as it is -- and refused by name
// for a datum the ladder does not hold (MHW is not in stations.json yet). `said`: the declaration
// in words, for the log and the gate's identity.
bool LevelOnFrame(const RasterEntry& e, const TideModel& model, double datumOff, double& levelM,
                  std::string& said, std::string* why) {
    const std::string& d = e.captureDatum;
    double zero = 0.0;   // the datum's zero on NAVD88
    if (d.empty() || d == "NAVD88") {
        zero = 0.0;
    } else if (d == "MLLW") {
        zero = datumOff;
    } else if (d == "MSL") {
        if (model.Count() == 0) {
            if (why) *why = "captureDatum MSL: no tide station to take the fitted mean from";
            return false;
        }
        zero = datumOff + model.S(model.Focus()).meanMllwM;
    } else {
        if (why) *why = "captureDatum '" + d + "' is not on the place's ladder (NAVD88, MLLW, MSL)";
        return false;
    }
    levelM = e.captureLevel + zero;
    char b[128];
    snprintf(b, sizeof(b), "%g m %s = %+.3f m NAVD88", e.captureLevel, d.empty() ? "NAVD88" : d.c_str(), levelM);
    said = b;
    return true;
}

float ResolveDatum(const TideModel& model) {
    const TideStation& fs = model.S(model.Focus());
    if (fs.mllwMinusNavdM > -900.0) {
        Log("[datum] %s CO-OPS datums: MLLW - NAVD88 = %+.3f m", fs.name.c_str(),
            fs.mllwMinusNavdM);
        return static_cast<float>(fs.mllwMinusNavdM);
    }
    double delta = 0.0;
    std::string from = "NAVD=MSL assumption";
    NavdAboveMsl(model, fs.lat, fs.lon, &delta, &from);
    const float off = static_cast<float>(-(fs.meanMllwM + delta));
    Log("[datum] no NAVD link at %s; MLLW - NAVD88 = %+.3f m via NAVD=MSL%+.3f (from %s)",
        fs.name.c_str(), off, delta, from.c_str());
    return off;
}

}  // namespace

namespace ga::app {

std::unique_ptr<Assembly> Assemble(const Options& opt, const Scene& S, int& exitCode) {
    auto A = std::make_unique<Assembly>();
    Compositor::s_tilePaint = S.streaming.tilePaint;   // tile at a time where a source can (the A/B key)
    // ---- The aliases: one reference per member, in order, under main()'s names, so the body
    // below is main()'s code unchanged. A closure that captures one of these by reference
    // captures the member it is bound to (CWG 2011) and stays valid after this returns.
    // activeGlobe, srcEtopo and srcMola are bound at the line that used to declare them.
    auto& model = A->model;
    auto& window = A->window;
    auto& gpu = A->gpu;
    auto& surface = A->surface;
    auto& rd = A->rd;
    auto& renderer = A->renderer;
    auto& fields = A->fields;
    auto& skyOwned = A->skyOwned;
    auto& sky = A->sky;
    auto& tideOwned = A->tideOwned;
    auto& tide = A->tide;
    auto& seaState = A->seaState;
    auto& sea = A->sea;
    auto& currents = A->currents;
    auto& haveCurrents = A->haveCurrents;
    auto& datumOff = A->datumOff;
    auto& marsMode = A->marsMode;
    auto& globeModel = A->globeModel;
    auto& marsModel = A->marsModel;
    auto& globeDataOk = A->globeDataOk;
    auto& compositor = A->compositor;
    auto& navdAboveMsl = A->navdAboveMsl;
    auto& datumFrom = A->datumFrom;
    auto& haveMslLink = A->haveMslLink;
    auto& mslToNavd = A->mslToNavd;
    auto& srcEdits = A->srcEdits;
    auto& hgtCh = A->hgtCh;
    auto& waterAtlas = A->waterAtlas;
    auto& sweDomain = A->sweDomain;
    auto& swe = A->swe;
    auto& rivers = A->rivers;
    auto& gulf = A->gulf;
    auto& waterScene = A->waterScene;
    auto& waterSceneMtime = A->waterSceneMtime;
    auto& kScenePath = A->kScenePath;
    auto& sceneWatch = A->sceneWatch;
    auto& waterBank = A->waterBank;
    auto& waterBankB = A->waterBankB;
    auto& waterBankEye = A->waterBankEye;
    auto& globe = A->globe;
    auto& vesselLayer = A->vesselLayer;
    auto& buildingLayer = A->buildingLayer;
    auto& planetR = A->planetR;
    auto& resMgr = A->resMgr;
    auto& marsDiff = A->marsDiff;
    auto& marsNorm = A->marsNorm;
    auto& googleTiles = A->googleTiles;
    // The Google source takes the scene's finest zoom at construction, held to 0..19 (14, the
    // default, is the source that was a literal).
    if (S.streaming.googleZoom > static_cast<uint32_t>(GoogleColorSource::kMaxZoom)) {
        Log("[google] streaming.googleZoom %u is past %d: held to %d", S.streaming.googleZoom,
            GoogleColorSource::kMaxZoom, GoogleColorSource::kMaxZoom);
    }
    auto& srcGoogle = A->srcGoogle.emplace(
        &A->googleTiles, static_cast<int>((std::min)(S.streaming.googleZoom, 19u)));
    auto& srcBed = A->srcBed;
    auto& srcRelief = A->srcRelief;
    auto& gisStencil = A->gisStencil;
    auto& gisMask = A->gisMask;
    auto& srcGisMask = A->srcGisMask;
    auto& vectors = A->vectors;
    auto& gisLayer = A->gisLayer;
    auto& hud = A->hud;
    auto& exchange = A->exchange;
    auto& colorCubeT = A->colorCubeT;
    auto& hgtTenant = A->hgtTenant;
    auto& maskTenant = A->maskTenant;
    auto& colCh = A->colCh;
    auto& megaKeep = A->megaKeep;
    auto& megaTree = A->megaTree;
    auto& heightRoot = A->heightRoot;
    auto& heightTree = A->heightTree;
    auto& exposureSrc = A->exposureSrc;
    auto& exposureRoot = A->exposureRoot;
    auto& exposureTree = A->exposureTree;
    auto& exposureT = A->exposureT;
    auto& heightTenant = A->heightTenant;
    auto& exposureTenant = A->exposureTenant;
    auto& exposureShadow = A->exposureShadow;
    auto& heightBed = A->heightBed;
    auto& colorTenant = A->colorTenant;
    auto& landseaTenant = A->landseaTenant;
    auto& idxColorCube = A->idxColorCube;
    auto& idxHeightCube = A->idxHeightCube;
    auto& tileStream = A->tileStream;

    // ---- M12 step 5d: the scene's spellings this span reads more than once. `shaderDir` was
    // opt.shaderDir (the wide form the shader compiler takes, widened once at the read); the
    // portal and the slice effect are named list elements, absent in every scene that does not
    // declare them.
    const std::wstring& shaderDir = S.shadersW;
    const ScenePortal* drostePortal = S.Portal("droste");
    const bool droste = drostePortal && drostePortal->p.enabled;
    const SceneEffect* sliceFx = S.EffectOfType("slice.plane");
    const bool sliceOn = sliceFx != nullptr;

    // A RASTER IS A SOURCE BY BEING A FILE (compose/RasterFileSource.h): the scene's `sources`,
    // opened before the device. Then THE PASS (HIERARCHY 4.20): each paints its own level and the
    // tree folds the rest, on every lattice the colour binds below, before the first frame -- or
    // alone, and out, as the `ingest` tool. (PHASE A1: the windows are the eye's, not the
    // sources'; nothing here chooses one.)
    // THE PLANET (core/Planet.h): its radius and its gravity from its own body, set once for every solver
    SetThePlanet(PlanetByName(S.scene.planet));
    planetR = ThePlanet().R;
    surface = SurfaceFrame::About(planetR, opt.stencil, S.place.anchor[1], S.place.anchor[0]);
    // PHASE B2 (D1): THE WINDOWS' STEP, a whole tile at the floor of every tenant that shares them --
    // the colour, the mask and the height (128 x 128), the exposure (256 x 128): 2048 x 1024.
    surface.ShareWindows(128, 128);
    surface.ShareWindows(256, 128);
    Log("[surface] the windows step by %u x %u texels of their rank: a whole tile at the floor (mip 3) "
        "of every tenant sharing them",
        surface.step[0], surface.step[1]);
    if (S.scene.planet != "mars") {
        std::vector<RasterEntry> entries;
        for (const scene::SourceProps& s : S.sources) {
            if (s.kind == "seastate") continue;   // PHASE C2: a sea state, not a raster (below)
            if (s.kind == "buildings") continue;  // solids, not a raster: the buildings layer's (below)
            if (s.kind == "roads") continue;      // ways, not a raster: the tree's ribbons (--tool building-lod)
            if (s.kind == "bridges") continue;    // spans, not a raster: the decks' clearances (--tool building-lod)
            if (s.kind == "composite") {
                // A COMPOSITE (docs/FIDELITY.md section 3, GACOMP01): many sources made into one
                // product by a harvester; a raster layer is its tiers, each a folder of one grain,
                // read as any folder is. The engine reads the product, never its inputs.
                for (RasterEntry& t : CompositeTiers(s)) entries.push_back(std::move(t));
                continue;
            }
            entries.push_back({s.file, s.folder, s.match, s.manifest, s.name, s.kind, s.crs, s.over,
                               s.feather, s.unit, s.datum, s.offset, s.hasOffset, s.captureLevel,
                               s.captureDatum, s.hasCaptureLevel});
        }
        A->sceneSources = LoadRasterSources(entries);
    }
    // The folder every tile tree of this run lives in (streaming.treeRoot), before the first
    // tree is built. A tool pointed at a scratch folder paints, packs and reads there alone.
    TileTree::SetTreeRoot(S.streaming.treeRoot);
    if (S.streaming.colorTrees || S.Tool("ingest")) {
        // The heights' pyramid at boot too (2026-10-17): a height source's own level and its folds, on
        // the lattice the Float root's children answer in (TileTree::KidFrame: the pyramid twice as
        // wide a tile). Painted on demand, each coarse level the terrain or the solver asked for was
        // an area mean of the whole file -- 655 ms a height read, cold, at the jetty with a 62 Mpx bed
        // and a 25 Mpx lidar on the tree, against 2 ms warm; folds kept at boot make it the read of
        // a tile. (PHASE B3 deleted the z14 height page; this is the pyramid, not a page.) A height leaf
        // is the Float root's child, so its frames are the root's twice as wide a tile (FloatKidFrame):
        // until 2026-10-21 the cube's was surface.cubeH itself, 128 x 128, in the folder the root's
        // 256 x 128 reads share (TileTree::Fits).
        IngestSources(A->sceneSources,
                      std::vector<Lattice>{surface.cube, hal::BlockBinding::Pyramid(128, 128)},
                      {TileTree::FloatKidFrame(hal::BlockBinding::Pyramid(128, 128)),
                       TileTree::FloatKidFrame(surface.cubeH)});
    }
    if (S.Tool("ingest")) return nullptr;

    // ---- M1: the tide viewer.
    if (!model.Load(S.data.tides)) {
        Log("FATAL: no tide data at '%s'.", S.data.tides.c_str());
        Log("Run the harvester once (network, cached forever after):");
        Log("    py -3 harvester\\harvest_tides.py");
        exitCode = 1;
        return nullptr;
    }

    if (!S.capture.headless) {
        if (!window.Create(S.capture.width, S.capture.height, L"GAGAME")) {
            exitCode = 1;
            return nullptr;
        }
    }

    // M7k: the PIX capturer must be resident BEFORE device creation.
    if (opt.pixFrames > 0) PixLoadGpuCapturer();
    gpu.Init(S.capture.headless ? nullptr : window.Handle(), S.capture.width, S.capture.height,
             opt.debugLayer, opt.noVsync && !S.capture.headless);
    if (!S.capture.headless) {
        Log("[window] client area %ux%u (requested %ux%u; the swapchain matches the client "
            "rect, not the outer window)",
            window.Width(), window.Height(), S.capture.width, S.capture.height);
    }

    rd.shaderDir = shaderDir;
    renderer.Init(gpu, rd);
    if (opt.gpuTime) renderer.EnableGpuProfiler();
    // M12 step 4a: THE SHIPPED SURFACE, declared once (compose/SurfaceFrame.h): the planet's
    // radius, the cube and the Merrimack windows the tenants below are declared on, the
    // tenants themselves once they exist (Declare, at the old SetComposed site), and the
    // tangent frame's rows the session writes into it. Both fills read it.
    // (planetR and the surface were declared before the device, with the sources. PHASE A1: the
    // eye's windows are the one compiled path; Compose.hlsli holds GA_BLOCK_RANKS = 5 itself.)

    fields.Init(gpu, L".", 1.0f, 1.0f);

    // Registration order IS draw order: sky (backdrop, depth off), then the tide product.
    skyOwned = std::make_unique<SkyLayer>();
    sky = skyOwned.get();
    sky->Configure(shaderDir, opt.skyProbe);
    sky->Init(gpu, renderer.Shaders(), fields, renderer.RootSignature());
    renderer.AddLayer(std::move(skyOwned));

    tideOwned = std::make_unique<TideLayer>();
    tide = tideOwned.get();
    // THE PLACE'S RIVER (2026-10-21): data.river's reach whose mouth is nearest the place gives the tide
    // profile -- its own stations at their kilometres -- for the ribbon and the chart.
    rivers.Load(S.data.river);   // the reaches meeting the sea, their gauges and their tide stations
    {
        int best = -1;
        double bestD2 = 0.0;
        const double cl = std::cos(S.place.anchor[1] * 3.14159265358979 / 180.0);
        for (size_t r = 0; r < rivers.Reaches().size(); ++r) {
            const RiverModel::Vertex& m0 = rivers.Reaches()[r].line.front();
            const double dLat = m0.lat - S.place.anchor[1], dLon = (m0.lon - S.place.anchor[0]) * cl;
            const double d2 = dLat * dLat + dLon * dLon;
            if (best < 0 || d2 < bestD2) { best = int(r); bestD2 = d2; }
        }
        std::vector<TideModel::ProfileStation> prof;
        if (best >= 0) {
            for (const RiverModel::Tide& t : rivers.R(best).tides) {
                for (size_t i = 0; i < model.Count(); ++i) {
                    if (model.S(i).id == t.id) prof.push_back({int(i), t.km});
                }
            }
            Log("[tide] the place's river: %s, its mouth %.1f km from the place, %zu of its stations in the tides",
                rivers.R(best).name.c_str(), std::sqrt(bestD2) * 111.19, prof.size());
        }
        model.SetProfile(std::move(prof));
    }
    tide->Configure(shaderDir, &model,
                    S.Layer("tide") ? S.Layer("tide")->exaggeration : 60.0f, S.place.anchor[1], S.place.anchor[0]);
    tide->Init(gpu, renderer.Shaders(), fields, renderer.RootSignature());
    renderer.AddLayer(std::move(tideOwned));

    // The open sea (M2) is optional until harvest_waves.py has run once.
    // A file of many points (a composite) stands the point nearest the scene's place as the sea's
    // own, and every other point as one more source (below).
    const std::vector<std::pair<double, double>> seaPoints = SeaState::PointsOf(S.data.seastate);
    int seaPoint = -1;
    double seaBestD2 = 0.0;
    for (size_t i = 0; i < seaPoints.size(); ++i) {
        const double dLat = seaPoints[i].first - S.place.anchor[1];
        const double dLon = (seaPoints[i].second - S.place.anchor[0]) * std::cos(S.place.anchor[1] * 3.14159265358979 / 180.0);
        const double d2 = dLat * dLat + dLon * dLon;
        if (seaPoint < 0 || d2 < seaBestD2) { seaPoint = int(i); seaBestD2 = d2; }
    }
    if (seaState.Load(S.data.seastate, seaPoint)) {
        // THE GLOBE'S SEA STATE (sim/SeaGrid.h, 2026-10-21): the composite's grid beside its points, when
        // it carries one. Then the synthesis is the eye's own sea and the field is the grid's; the other
        // points are not sources over it (the grid is their measurement, every node of it).
        const bool seaGridOn = A->seaGrid.Load(S.data.seastate);
        // PHASE C2: THE SEA STATE IS A FIELD OF SOURCES. data.seastate's file stands where it says,
        // or where sea.box places a file that says nothing; every `sources` entry of kind "seastate"
        // is one more, a source by being a file.
        if (!seaState.HasBox()) {   // the key is [lon0, lat0, lon1, lat1]; the box lat0, lon0, lat1, lon1
            const double* k = S.sea.box;
            const double b[4] = {k[1], k[0], k[3], k[2]};
            seaState.SetBox(b);
        }
        auto seaOwned = std::make_unique<SeaLayer>();
        sea = seaOwned.get();
        sea->Configure(shaderDir, &seaState);
        for (const scene::SourceProps& s : S.sources) {
            if (s.kind != "seastate") continue;
            auto src = std::make_unique<SeaState>();
            if (!src->Load(s.file)) {
                Log("[sea] source %s: %s unreadable -- not a source", s.name.c_str(), s.file.c_str());
                continue;
            }
            sea->AddSource(src.get());
            A->seaSources.push_back(std::move(src));
        }
        for (int i = 0; i < int(seaPoints.size()) && !seaGridOn; ++i) {
            if (i == seaPoint) continue;
            auto src = std::make_unique<SeaState>();
            if (!src->Load(S.data.seastate, i)) continue;
            sea->AddSource(src.get());
            A->seaSources.push_back(std::move(src));
        }
        sea->SetSurface(&surface);   // the world.flat chart, for the churn's tangent rows (PHASE C5)
        sea->SetGrid(seaGridOn ? &A->seaGrid : nullptr);
        sea->sweCurrentGain = S.water.swe.gain;
        sea->heightScale = S.water.heightScale;
        sea->Init(gpu, renderer.Shaders(), fields, renderer.RootSignature());
        if (S.sea.storm.hs > 0.01f) sea->SetStorm(S.sea.storm.hs, S.sea.storm.tp, S.sea.storm.dir);
        renderer.AddLayer(std::move(seaOwned));
    } else {
        Log("[main] no sea state (run: py -3 harvester\\harvest_waves.py)");
    }

    // M3: currents -- the ACT tidal clock for the sea's jet, the GoMOFS field for the gulf.
    haveCurrents = currents.Load(S.data.currents);
    for (const scene::StationProps& st : S.stations) currents.Place(st.name, st.lat, st.lon);
    if (!haveCurrents) {
        Log("[main] no currents (run: py -3 harvester\\harvest_currents.py)");
    }

    // ---- M6w: THE ONE BED. The planets' CPU models, the raw CUDEM planes, and the
    // composed HEIGHT channel all come up BEFORE the solver -- because the solver's bed
    // is no longer a private file: it is REALIZED from the same painted stack the
    // renderer's tiles come from (ETOPO <- NE-15s <- the CUDEM windows <- hand edits).
    marsMode = (S.scene.planet == "mars");
    if (marsMode) marsModel.LoadMars("data/globe/globe.json");
    globeDataOk = globeModel.Load("data/globe/globe.json");
    GlobeModel& activeGlobe =
        (marsMode && marsModel.Ready()) ? marsModel : globeModel;
    A->activeGlobe = &activeGlobe;   // the loop reads the same choice through the Assembly

    // M9n: THE HEIGHT STACK'S VERTICAL DATUM, resolved once and applied at the sources.
    // ETOPO is MSL/geoid referenced and everything above it in the stack is NAVD88; the two
    // were blended as one height for as long as the stack has existed. The link is published
    // -- CO-OPS carries MSL and NAVD88 on one station staff -- so it is derived, logged, and
    // handed to the ETOPO sources, which means the CORRECTION REACHES EVERY CONSUMER of the
    // stack rather than only the products that happen to know about it.
    haveMslLink = NavdAboveMsl(model, S.place.anchor[1], S.place.anchor[0], &navdAboveMsl, &datumFrom);
    mslToNavd = haveMslLink ? -navdAboveMsl : 0.0;
    if (haveMslLink) {
        Log("[datum] height stack: MSL -> NAVD88 = %+.3f m, from CO-OPS published datums at "
            "%s (regional, not GEOID18) -- applied to the ETOPO layers at the source",
            mslToNavd, datumFrom.c_str());
    } else {
        Log("[datum] height stack: NO station publishes both MSL and NAVD88 -- ETOPO stays "
            "in its own datum and the stack blends two frames (pre-M9n behaviour)");
    }
    EquirectHeightSource& srcEtopo = A->srcEtopo.emplace(
        "noaa.etopo2022", "equirect-grid int16 8192x4096", 489200.0, &globeModel.Elev(),
        globeModel.Nx(), globeModel.Ny(), mslToNavd);
    EquirectHeightSource& srcMola = A->srcMola.emplace(
        "nasa.mola.megdr16", "equirect-grid int16 5760x2880", 369700.0, &marsModel.Elev(),
        marsModel.Nx(), marsModel.Ny());
    if (marsMode && marsModel.Ready()) {
        hgtCh = compositor.AddHeightChannel("mars.height", {&srcMola});
    } else if (globeDataOk) {
        std::vector<HeightSource*> hstack{&srcEtopo};
        // The scene's heights by file join in the default order (StackOrder: `over`, then the
        // coarser under the finer), and the hand edits, the owner's own corrections, stay on top.
        for (const auto& s : A->sceneSources) {
            if (s->Height()) hstack.push_back(s.get());
        }
        StackOrder(hstack);
        if (!S.data.edits.empty() && srcEdits.Load(S.data.edits, 2.5f)) {
            hstack.push_back(&srcEdits);
        }
        hgtCh = compositor.AddHeightChannel("earth.height", std::move(hstack));
        // THE HAND EDIT YIELDS TO A FINER WITNESS (Sources.cpp, EditsHeightSource::Uncovered): the
        // channel it stands in names the witnesses; its tiles' identity folds theirs (HeightSource::Law).
        srcEdits.Witnesses(&compositor, hgtCh);
        // THE HEIGHT TREE (M9as), built as soon as the channel stands: every CPU reader after this line --
        // the solver's bed first -- reads it through Compositor::SampleHeight (compose/HeightTree.h), and
        // the GPU's tenant below is fed from it. Before 2026-10-14 it stood after the solver's bed was
        // realized, so that bed alone still read the stack.
        if (S.streaming.colorTrees || S.Tool("tree-audit") || S.Tool("warm-trees") || S.Tool("pack-trees")) {
            // The channel's own order, the one the CPU's stack reads (the height stack above:
            // StackOrder, the hand edits last), so the GPU's bed and the CPU's are one over.
            auto layers = BuildHeightStack(compositor, hgtCh);
            const Compositor::Channel& hch = compositor.ChannelAt(hgtCh);
            auto dc = std::make_shared<DomainCompositor>();
            dc->SetBlend(DomainCompositor::Blend::LayeredOver);
            std::string order;
            for (size_t i = 0; i < layers.size() && i < hch.height.size(); ++i) {
                if (dc->Add(layers[i])) order += " " + std::string(layers[i]->Name()) + "(" +
                                                 std::to_string(int(hch.height[i]->Info().cmPerPixel)) + "cm)";
            }
            heightRoot = std::make_shared<CompositeSource>("earth.height", dc);
            for (auto& l : layers) megaKeep.push_back(l);
            Log("[height-tree] compose order, bottom to top (the channel's: `over`, the coarser "
                "under the finer, the hand edits last):%s",
                order.c_str());
            heightTree = std::make_unique<TileTree>(heightRoot.get(), TileTree::Fmt::Float);
            heightTree->Print();
            // THE CPU'S READ OF IT (compose/HeightTree.h): the composed root on the pyramid, bound
            // to the channel; Compositor::SampleHeight answers from it.
            {
                const Lattice pyr = hal::BlockBinding::Pyramid(128, 128);
                compositor.BindHeightTree(hgtCh, std::make_shared<HeightTree>(heightTree->Reader(pyr),
                                                                              heightTree->MagnifiedQuery(pyr), pyr));
            }
            if (S.Tool("exposure-ab")) {
                exitCode = tools::RunExposureAB(compositor, hgtCh, S.place.anchor[1], S.place.anchor[0]);
                return nullptr;
            }
            if (S.Tool("fetch-probe")) {
                exitCode = tools::RunFetchProbe(*compositor.HeightTreeOf(hgtCh), S.place.anchor[1], S.place.anchor[0]);
                return nullptr;
            }
            if (const SceneTool* t = S.Tool("height-tree-probe")) {
                exitCode = tools::RunHeightTreeProbe(compositor, hgtCh, S.place.anchor[0], S.place.anchor[1], t->args);
                return nullptr;
            }
        }
    }

    // M6v/M8i: THE WATER ATLAS -- water parameters through the same registry. 18 phasor
    // channels (water.tide.M2..O1): equilibrium base <- EOT20 global medium <- the NE
    // station field (20 CO-OPS fits, Merrimack to Scituate), epoch-laddered, cached as
    // RG16F window tiles on demand. The sim manager consumes these next.
    waterAtlas.Init(compositor, model, "data/water");
    if (S.Tool("water-map") || S.Tool("bathy-map")) {
        exitCode = tools::RunWaterMap(opt, gpu, compositor, hgtCh, waterAtlas, surface.flat);
        return nullptr;
    }

    // THE SOLVER'S DOMAIN (PHASE C1; 2026-10-21, Mark: "Data sets should never be specific to place,
    // only the measurement type"). The scene's lat/lon box (water.swe.box) and the solver's cell
    // (water.swe.cellM: one angle both ways, its north-south side in metres) -- its anchor the box's
    // centre, its cells in true metres at the anchor (SweDomain: a ratio of planes about the anchor,
    // HIERARCHY 4.4), its CPU bed the one height stack at every cell, whatever surveys the stack holds
    // there. The box is the place's; the bed is the world's. Until this date the domain stood on a
    // per-place survey file (data.bathy: the cell, the default window, and the water bank's gate), so a
    // place no survey file named had no waves at all. No box, no solver. PHASE B2 (D4): ONE STANDING
    // RANK-2 WINDOW about the anchor (rank 2: the finest whose texel is at most the solver's cell), held
    // whole before the solver starts.
    if (hgtCh >= 0 && !marsMode) {
        const double* bx = S.water.swe.box;
        const double lon0 = bx[0], lat0 = bx[1], lon1 = bx[2], lat1 = bx[3];
        if (lon0 != 0.0 || lat0 != 0.0 || lon1 != 0.0 || lat1 != 0.0) {
            constexpr double kD2Rd = 3.14159265358979323846 / 180.0;
            const double cellDeg = double(S.water.swe.cellM) / (planetR * kD2Rd);
            const auto cx = static_cast<uint32_t>(std::lround((lon1 - lon0) / cellDeg));
            const auto cy = static_cast<uint32_t>(std::lround((lat1 - lat0) / cellDeg));
            if (lon1 > lon0 && lat1 > lat0 && cx > 1 && cy > 1) {
                sweDomain.Place(lat0, lon0, lat1, lon1, cx, cy, planetR);
                sweDomain.FillBed([&](double la, double lo, double resM) {
                    return compositor.SampleHeight(hgtCh, la * kD2Rd, lo * kD2Rd, resM);
                });
                surface.StandAbout(sweDomain.latC, sweDomain.lonC, 2u);
            } else {
                Log("[swe] water.swe.box [%.5f, %.5f, %.5f, %.5f] REFUSED: not a box of at least "
                    "two cells of %.2f m -- no solver", lon0, lat0, lon1, lat1, double(S.water.swe.cellM));
            }
        }
    }
    // PHASE C3: THE TIDE FOCUS is the station the water's point asks for: its reader is the
    // solver (its ocean clock, its NAVD88 bed), so the point is its anchor and the stations those
    // its domain holds (sim/Stations.h); data.tideFocus names one instead.
    if (sweDomain.Ready()) {
        const std::string& id = S.data.tideFocus;
        const int fi = NearestStation(model, sweDomain.latC, sweDomain.lonC,
                                      [&](double la, double lo) { return sweDomain.Holds(la, lo); });
        for (size_t i = 0; i < model.Count(); ++i) {
            if (id.empty() ? int(i) == fi : model.S(i).id == id) model.SetFocus(int(i));
        }
        const TideStation& f = model.S(model.Focus());
        Log("[tide] focus %s '%s' at %.5f N %.5f E (%s)", f.id.c_str(), f.name.c_str(), f.lat, f.lon,
            id.empty() ? "the nearest the solver's anchor in its domain" : "data.tideFocus");
    }
    // M9ar: the height megatexture (the height page tenant, the cube and the eye's windows) is the
    // only bed on the GPU; the solver, the sea shader, the water bank and the globe all read it.
    if (sea) Log("[bed] the sea and the solver read the height megatexture -- the only bed");

    // M5c: the MLLW -> NAVD88 join, the focus station's CO-OPS link unless the scene declares one.
    datumOff = !S.sea.datum.fromStation ? S.sea.datum.mllwToNavd : ResolveDatum(model);

    // M5c: the sparse shallow-water solver -- the estuary's own hydrodynamics, tide-forced
    // offshore and river-forced upstream, feeding the sea's mean surface and currents.
    if (sea && S.water.swe.enabled && sweDomain.Ready()) {
        SweConfig sweCfg;
        sweCfg.name = S.scene.name.c_str();
        sweCfg.spongeM = S.water.swe.sponge;
        // THE RIVERS THAT ENTER IT (sim/RiverModel.h): every reach of data.river whose mouth the box
        // holds, a Flather face where its line leaves the box. Their clocks are the frame loop's.
        for (const RiverModel::Crossing& c : rivers.EntersBox(sweDomain.lat0, sweDomain.lon0, sweDomain.lat1, sweDomain.lon1)) {
            sweCfg.faces.push_back({c.side, c.lat, c.lon, rivers.R(c.reach).name});
        }
        swe.Init(gpu, renderer.Shaders(), shaderDir, sweDomain, sweCfg);   // bed bound below
        sea->SetSwe(&swe);
    }
    if (haveCurrents && currents.FieldAt(S.place.anchor[0], S.place.anchor[1]).Valid()) {
        auto gulfOwned = std::make_unique<GulfLayer>();
        gulf = gulfOwned.get();
        gulf->Configure(shaderDir, &currents, &currents.FieldAt(S.place.anchor[0], S.place.anchor[1]));
        gulf->Init(gpu, renderer.Shaders(), fields, renderer.RootSignature());
        renderer.AddLayer(std::move(gulfOwned));
    }

    // M7: THE WAVE VERTEX BANK -- the water's geometry as ONE tiled resource, filled
    // per frame from everything the weather manager federates, sampled by the globe in
    // one-water mode. Registered BEFORE the planet so the rings are recomposed (and the
    // cascades already advanced by SeaLayer) when the globe's meshlets sample them.
    // M8h: the water scene loads BEFORE the layers stand up -- the bank's ring
    // density (bankTexelM) is a construction-time choice. Hot-reload still lives
    // in the frame loop; geometry-of-the-bank changes need a restart (logged).
    LoadWaterScene(kScenePath, waterScene);
    WaterSceneChanged(kScenePath, &waterSceneMtime);
    // M12 step 5d: ...and the RESOLVED SCENE's `water` section over it. The file above reaches
    // the document as merrimack.json's own `include` overlay, so for a shipped run these are the
    // same values by the same fold (LoadWaterScene still AUTHORS the file when it is absent --
    // the M6p law) -- but a scene file, an overlay or a --set now says them too, through the ONE
    // table (scene/WaterComponent.h) whose Apply the reload uses.
    {
        scene::PropSet wset(scene::WaterSchema());
        scene::WaterComponent::Fleet wfleet;
        std::string why;
        if (!A->water.ReadJson(WaterSceneDoc(S.waterDoc), wset, wfleet, &why) ||
            !wset.ApplyTo(&waterScene, nullptr, &why)) {
            Log("FATAL: [scene] %s", why.c_str());
            exitCode = 2;
            return nullptr;
        }
        waterScene.fleetCount = wfleet.count;
        for (int i = 0; i < 8; ++i) waterScene.fleet[i] = wfleet.boats[i];
    }
    // Step 3 (docs/PERF_EXPERIMENT.md): the directory watches the file; the frame polls
    // one atomic instead of paying the 0.14-0.19 ms stat through the data/ junction.
    sceneWatch.Start(kScenePath);

    if (sea && !marsMode) {
        auto wbOwned = std::make_unique<WaterBankLayer>();
        waterBank = wbOwned.get();
        waterBank->Configure(shaderDir, sea, &swe, &waterAtlas, &compositor, hgtCh, &globeModel);
        waterBank->SetBaseTexel(waterScene.bankTexelM);   // M8h ring density (scene)
        waterBank->SetSurface(&surface);   // the world.flat chart: the rings' places (PHASE C5)
        waterBank->SetMask(&gisMask);
        A->bankMaskClose.jobs.push_back(waterBank->MaskJobsOf());      // the coast's water makes a ring tile wet
        waterBank->flatBed = S.water.bank.flatBed;
        waterBank->flatBedNavd = S.water.bank.flatBedNavd;
        waterBank->column = S.water.bank.column;   // the first eye's rings carry the column plane
        waterBank->columnTexel = S.water.bank.columnTexel;
        if (S.water.bank.flatBed) {
            Log("[bed] --flat-bed %.1f m NAVD: the bank fills against a CONSTANT floor. Diff "
                "this run's wireframe against a normal one -- whatever differs is what "
                "bathymetry does to the MESH, with shading held out of it.",
                S.water.bank.flatBedNavd);
        }
        waterBank->Init(gpu, renderer.Shaders(), fields, renderer.RootSignature());
        renderer.AddLayer(std::move(wbOwned));
        // M10: SET B -- the rings of the level the camera's planet FLOATS IN (Droste.h). The
        // bank is a camera-anchored ladder and under the gauge every level has its own eye;
        // the outer sea is seen from S(C), so its waves need rings anchored there. Stateless
        // like the first bank (every tile recomputed each frame), so it costs one more fill
        // and a second small atlas, and the fill runs only in a frame a world reads it.
        // (And for a scene with GATES: the sea seen through a gate's window is seen from the
        // carried eye, so it needs rings anchored there -- the same second bank, the same law.
        // With a gate in reach and no window in view the rings stand anchored and unfilled.)
        if (droste || !S.gates.empty()) {
            auto wbB = std::make_unique<WaterBankLayer>();
            waterBankB = wbB.get();
            waterBankB->Configure(shaderDir, sea, &swe, &waterAtlas, &compositor, hgtCh, &globeModel);
            waterBankB->SetBaseTexel(waterScene.bankTexelM);
            waterBankB->SetSurface(&surface);
            waterBankB->SetMask(&gisMask);
            A->bankMaskClose.jobs.push_back(waterBankB->MaskJobsOf());
            waterBankB->flatBed = S.water.bank.flatBed;
            waterBankB->flatBedNavd = S.water.bank.flatBedNavd;
            waterBankB->column = false;   // set B's sea is read through the gauge; no lip sheets there yet
            waterBankB->Init(gpu, renderer.Shaders(), fields, renderer.RootSignature());
            waterBankB->enabled = false;   // until the camera has a level above it
            renderer.AddLayer(std::move(wbB));
        }
        // ANOTHER EYE'S RINGS (scene hud.minimap): the same ladder anchored at the second eye, so
        // the sea it looks down on has the waves the first eye's has, wherever it looks. The same
        // stateless fill, run only in a frame that eye stands low enough to read it.
        if (S.hud.minimap.enabled) {
            auto wbE = std::make_unique<WaterBankLayer>();
            waterBankEye = wbE.get();
            waterBankEye->Configure(shaderDir, sea, &swe, &waterAtlas, &compositor, hgtCh, &globeModel);
            waterBankEye->SetBaseTexel(waterScene.bankTexelM);
            waterBankEye->SetSurface(&surface);
            waterBankEye->SetMask(&gisMask);
            A->bankMaskClose.jobs.push_back(waterBankEye->MaskJobsOf());
            waterBankEye->flatBed = S.water.bank.flatBed;
            waterBankEye->flatBedNavd = S.water.bank.flatBedNavd;
            waterBankEye->column = false;   // the minimap's rings draw no lip sheets
            waterBankEye->Init(gpu, renderer.Shaders(), fields, renderer.RootSignature());
            waterBankEye->enabled = false;   // until that eye stands low enough to read it
            renderer.AddLayer(std::move(wbE));
        }
    }

    // M6: the planet (registered LAST -- it borrows root param 2 for its node list).
    // M6f: which planet is a MODEL choice -- Mars loads MOLA into the same relief fields
    // and the whole pipeline (texture, mips, camera clamps) serves it unchanged.
    // (M6w: the models + the compositor's height side were HOISTED above the bathy/solver
    // block -- the solver's bed realizes from the channel.)
    if (globeDataOk) {
        auto globeOwned = std::make_unique<GlobeLayer>();
        globe = globeOwned.get();
        globe->Configure(shaderDir, &activeGlobe);
        globe->marsReliefValid = marsMode && marsModel.Ready();
        globe->windOverlay = opt.viz;
        globe->surfaceDebug = opt.surfaceDebug;
        globe->meshStats = opt.meshStats;
        globe->msSurface = opt.msSurface;
        globe->albedoLens = opt.albedo;
        // Before Init: the residency lens's pipelines are built there, and only when asked for.
        globe->debugLens = opt.lens;
        // ...and the scene's debug view, before Init for the same reason.
        globe->SetDebug(S.debug);
        // ...and the cache lens's, only for a scene whose minimap can turn it on (Minimap::Cache).
        globe->cacheLensBuild = S.hud.minimap.enabled;
        if (!opt.groundProbe.empty()) {   // Phase A0: lat,lon in degrees -> the planet direction
            double lat = 0.0, lon = 0.0;
            if (sscanf_s(opt.groundProbe.c_str(), " %lf , %lf", &lat, &lon) == 2) {
                const double k = 3.141592653589793 / 180.0;
                globe->groundProbeOn = true;
                globe->groundProbeDir[0] = std::cos(lat * k) * std::cos(lon * k);
                globe->groundProbeDir[1] = std::sin(lat * k);
                globe->groundProbeDir[2] = std::cos(lat * k) * std::sin(lon * k);
            } else {
                Log("[ground-probe] '%s' is not lat,lon -- the probe is off", opt.groundProbe.c_str());
            }
        }
        globe->Init(gpu, renderer.Shaders(), fields, renderer.RootSignature());
        renderer.AddLayer(std::move(globeOwned));
    } else {
        Log("[main] no globe data (run: py -3 harvester\\harvest_globe.py)");
    }

    // ---- M6e: the unified residency manager + the streamed planet surfaces. Two worlds,
    // one machine: Mars streams the rescued sample's BC1/BC5 pyramids from disk; Earth
    // streams Google 2D tiles (cache-first, throttled, budget-capped) reprojected onto the
    // same cube faces. Residency is driven by the CDLOD walk, clamped by residency-map
    // cubes, prefetched along the camera's screw.
    // planetR and THE SHIPPED SURFACE were declared before the device, and the tree root with
    // them (HIERARCHY 4.17: the key decides what the layers compile).
    // --warm-trees and --pack-trees are tree-audit's other two modes (Tools/TreeAudit.cpp):
    // each needs the trees built and ends the run where the audit does.
    const bool treeTool = S.Tool("tree-audit") || S.Tool("warm-trees") || S.Tool("pack-trees");

    if (globe) {
        resMgr.Init(gpu);
        resMgr.holdMargin = static_cast<float>(S.streaming.holdMargin);   // H2: 1 is F's order
        resMgr.dsSerial = opt.dsSerial;
        resMgr.traceRes = opt.resTrace;
        WaveField::SetSolveAudit(opt.waveAudit);   // F25: the solve's instrument
        CbTrace() = opt.cbTrace;   // F9: the CB fingerprints print when asked for
        resMgr.pagesEvery = opt.pagesEvery;
        resMgr.auditEvery = S.capture.residencyAudit;   // the scene's (--res-audit N)
        resMgr.poolCapTiles = (std::max)(S.streaming.poolTiles, 1024u);   // streaming.poolTiles: the reserve (384) and a few chunks at least
        resMgr.starvePlant = opt.starvePlant;           // the watchdog's plant (0 = off)
        int surf = -1, norm = -1;
        if (marsMode) {
            // Mars: color/normal stay NATIVE streams (the rescued sample's pyramids are
            // already tile-shaped truth); height composes from MOLA through the same
            // machinery Earth uses -- one relief path in the shader for both planets.
            if (marsDiff.Open(L"data/earth/diffuse.bin", DXGI_FORMAT_BC1_UNORM_SRGB)) {
                surf = resMgr.AddTextureCube(gpu, L"mars.diffuse (the rescued sample)",
                                             16384, DXGI_FORMAT_BC1_UNORM_SRGB,
                                             marsDiff.Fn());
            }
            if (marsNorm.Open(L"data/earth/normal.bin", DXGI_FORMAT_BC5_SNORM)) {
                norm = resMgr.AddTextureCube(gpu, L"mars.normal", 16384,
                                             DXGI_FORMAT_BC5_SNORM, marsNorm.Fn());
            }
            if (marsModel.Ready() && hgtCh >= 0) {
                hgtTenant = resMgr.AddTextureCube(gpu, L"mars.height (composed: MOLA)",
                                                  Compositor::kFaceDim,
                                                  DXGI_FORMAT_R16_FLOAT,
                                                  compositor.CubeHeight(hgtCh));
            }
            // M12 step 4a: Mars's height cube into the surface's declaration -- an int, not
            // a hal::Tenant (AddTextureCube), with no windows.
            surface.hgtT = hgtTenant;
        } else {
            // earth.height REGISTERED above the solver (M6w) -- here it becomes GPU
            // tenants: the global cube and the eye's windows.
            // M9aq: ONE height tenant -- pages 0..5 the cube faces, then the eye's windows
            // (the colour's slices, so the near-field land/sea gate and the normals ride
            // CUDEM truth).
            // M9as: fed by the height TileTree when --color-trees is on.
            {
                // M12 step 3e: THE DECLARATION (hal/Tenant.h). Slices 0..5 the cube faces on
                // the 16k quad-sphere, then the eye's windows; R16F metres NAVD88 in 256x128
                // tiles; a missing tile is not loaded yet, and the residency clamp reads the
                // coarser level. The providers are the height tree on each binding's lattice,
                // or the incumbent compositor's when the trees are off.
                hal::TenantDesc hd;
                hd.name = L"earth.height (megatexture pages)";
                hd.astNode = "height.pages";
                hd.fiber = {DXGI_FORMAT_R32_FLOAT, 128, 128, "m NAVD88 (float)"};
                hd.semantics = hal::Semantics::Texture;
                hd.residence = hal::Residence::Streamable;
                hd.absence = hal::Absence::Unloaded;
                hd.slices = SurfaceFrame::WindowSlices();
                const Lattice& hCubeL = surface.cubeH;   // M12 step 4a: the surface's lattices
                // The pages are the height tree's float root: without the tree there is no earth
                // height to page (the compositor's direct cube is Mars's half-float form).
                if (!(S.streaming.colorTrees && heightTree)) {
                    Log("[height] no height tree (streaming.colorTrees off): the earth is drawn without relief");
                }
                hd.bindings.push_back({0, 6, hCubeL,
                                       (S.streaming.colorTrees && heightTree) ? heightTree->Provider(hCubeL)
                                                                              : TileProviderFn{},
                                       "paint cube faces"});
                if (S.streaming.colorTrees && heightTree) hd.bindings.back().magnified = heightTree->MagnifiedQuery(hCubeL);   // F14
                // PHASE B2: THE EYE'S WINDOWS -- the colour's slices, the
                // same ground (HIERARCHY 4.1) -- and the standing window, painted by the height tree
                // on the pyramid (it is asked the global tile; the binding keeps the slot).
                if (heightTree) {
                    surface.WindowBlocks(hd.blocks,
                                         heightTree->Provider(hal::BlockBinding::Pyramid(128, 128)),
                                         "paint pyramid windows");
                    const TileQueryFn q = heightTree->MagnifiedQuery(hal::BlockBinding::Pyramid(128, 128));   // F14
                    for (hal::BlockSlice& k : hd.blocks) k.magnified = q;
                }
                heightTenant = hal::Tenant::Sparse(gpu, resMgr, std::move(hd));
                hgtTenant = heightTenant.Id();
                // PHASE B2: the hull's bed is the pyramid at the grain of the finest ring (the ring
                // law's rung: the finest whose texel is at most the ring's), by direction anywhere.
                {
                    const double base = waterBank ? waterBank->BaseTexelM() : 1.2;
                    const uint32_t rung = uint32_t(std::clamp(
                        int(std::ceil(std::log2(surface.cube.GroundRes(0) / (std::max)(base, 1e-3)))), 0, 15));
                    heightBed = std::make_unique<HeightPage>(&compositor, hgtCh, rung);
                    Log("[height] the hull's bed: the pyramid at rung %u (the finest ring's %.2f m)", rung, base);
                }
                // M9bb: a fold or a drop below changed a root tile: the tenant refetches that
                // address (the tree's tag names the slice) -- the one law, Tenant::Bind.
                if (heightTree) heightTenant.Bind(*heightTree);
                // THE HEIGHT'S MAX LANE (hal/MaxLane.h): what the terrain shadow's cast reads.
                if (S.render.shadowSteps > 0) {
                    resMgr.EnableMaxLane(hgtTenant, gpu, renderer.Shaders(), shaderDir);
                }
                // PHASE B2 (D4): the solver's bed is the standing window's chain.
                if (swe.Ready() && surface.standingRank) {
                    const Placement own = surface.StandingFrame();
                    const SurfaceFrame::ChainRows rows = surface.StandingRows(own, surface.standingCentre);
                    SurfaceFrame::KernelWindowRows kw;
                    SurfaceFrame::KernelRows(rows, kw);
                    SweSolver::BedWindow& bw = A->bedWindow;
                    memcpy(bw.rows, &kw, sizeof(kw));
                    bw.slice = SurfaceFrame::kStandingSlice;
                    swe.SetBed(gpu, resMgr.TextureRes(hgtTenant), resMgr.ResidencyRes(hgtTenant),
                               resMgr.Mips(hgtTenant), bw);
                }
                    if (sea && hgtCh >= 0 && S.streaming.exposure) {
                        exposureSrc = std::make_shared<ExposureSource>(&compositor, hgtCh);
                        auto xdc = std::make_shared<DomainCompositor>();
                        xdc->SetBlend(DomainCompositor::Blend::LayeredOver);
                        if (!xdc->Add(exposureSrc)) Log("[exposure] compose REFUSED the node");
                        exposureRoot = std::make_shared<CompositeSource>("swell.exposure", xdc);
                        exposureTree = std::make_shared<std::shared_ptr<TileTree>>(
                            std::make_shared<TileTree>(exposureRoot.get(), TileTree::Fmt::Half));
                        // M12 step 3e: THE DECLARATION, painted by the tree in the holder (a
                        // bucket roll swaps it; the dispatcher reads it per request). The cube
                        // faces are not this node's frame: an unbound slice answers "fully exposed" (R16F
                        // 1.0 = 0x3C00) so the boot's coarsest loads and any stray want land
                        // once instead of retrying forever.
                        hal::TenantDesc xd;
                        xd.name = L"swell.exposure (pages)";
                        xd.astNode = "exposure.node";
                        xd.fiber = {DXGI_FORMAT_R16_FLOAT, 256, 128,
                                    "swell exposure 0..1 (half float)"};
                        xd.semantics = hal::Semantics::Texture;
                        xd.residence = hal::Residence::Recomputable;
                        xd.absence = hal::Absence::OutOfDomain;
                        xd.absentTile.assign(65536, 0);
                        {
                            uint16_t* h = reinterpret_cast<uint16_t*>(xd.absentTile.data());
                            for (size_t i = 0; i < 32768; ++i) h[i] = 0x3C00u;
                        }
                        // PHASE B2 (D2): window slices alone -- the exposure is read from the eye's
                        // windows and never from the cube, so it declares no lattice; its tree paints
                        // on the pyramid (Tenant::Bind), the cube's slices answer "exposed".
                        xd.slices = SurfaceFrame::WindowSlices();
                        xd.holder = exposureTree;
                        surface.WindowBlocks(xd.blocks, nullptr, "exposure");
                        exposureTenant = hal::Tenant::Sparse(gpu, resMgr, std::move(xd));
                        exposureT = exposureTenant.Id();
                        exposureTenant.Bind(**exposureTree);   // its folds invalidate the windows' tiles
                        sea->SetExposurePage(resMgr.TextureSrv(exposureT),
                                             resMgr.ResidencySrv(exposureT), exposureSrc.get());
                        // The same texels for whoever floats in them, at the floor the bank reads
                        // them at (WaterTerms.h kSwellShadowMipFloor): the node asked where the
                        // painter asks it, quantized as the page stores it.
                        // PHASE B2: on the pyramid at rung 3 (76 m: the grain the bank reads it at).
                        exposureShadow = std::make_unique<ExposurePage>(exposureSrc.get(), 3u);
                        Log("[exposure] swell.exposure is page tenant %d: the LOS march over "
                            "the height stack, cached per (direction, level) bucket, on the eye's "
                            "windows alone, read at rung 3 (%.1f m nominal)",
                            exposureT, surface.cube.GroundRes(0) / 8.0);
                    }
                if (sea) {
                    sea->SetHeightPage(resMgr.TextureRes(hgtTenant),
                                       resMgr.ResidencyRes(hgtTenant), resMgr.Mips(hgtTenant));
                }
            }
            // earth.color: the Google mercator tree, realized on the global cube and the
            // eye's windows (same stack, deeper footprint).
            DayCaps dayCaps;   // the scene's day caps (streaming.dayTiles / dayBytes)
            dayCaps.tiles = S.streaming.dayTiles;
            dayCaps.bytes = DayCaps::FromScene(S.streaming.dayBytes);
            if (googleTiles.Init("satellite", S.streaming.tileBudget, dayCaps, S.streaming.offline)) {
                googleTiles.SetFetchCounter(&resMgr.fetchesThisRun);
                // THE NETWORK OFF THE LOADERS (core/TileProviders.h StartNetwork): a window's paints
                // never wait on a fetch. A headless still keeps the fetch in the paint: its frames
                // must see the tiles they ask for.
                const int fetchThreads = S.streaming.fetchThreads >= 0 ? S.streaming.fetchThreads
                                                                       : (S.capture.headless ? 0 : 4);
                googleTiles.StartNetwork(fetchThreads, [&resMgr](int t, const TileRequest& r) { resMgr.Again(t, r); });
                std::vector<ColorSource*> colorStack{&srcGoogle};
                size_t bedLayer = SIZE_MAX, maskLayer = SIZE_MAX;
                // M7x (user catch): the ortho was painting its capture-day WATER over the
                // drained-bed albedo -- a hard-edged dark rectangle the sea shader then
                // attenuated AGAIN. Photos are LAND authorities; the bed classifier is
                // the WATER authority. The stack order encodes that ranking: google under
                // the scene's photos (the plane orthos among them: finer wins), bed above
                // both (its height-band alpha reclaims everything below the intertidal ramp
                // and hands land back to the photos above +1.2 m NAVD).
                // The scene's `sources` join the photos, and the photos stand in their default
                // order (StackOrder: `over`, then the coarser grain under the finer).
                // THE CAPTURE LEVEL (compose/Sources.h LevelGateSource): a photo that declares the
                // water level it was flown at is gated, in the tree below, by the ground above that
                // level -- the edge the survey gates the seafloor with (GateSource): the pixel's colour
                // is its own, only the weight is the product. The level is resolved as the tides are
                // (LevelOnFrame); a datum the ladder does not hold refuses the photo by name, as a
                // height off the frame is refused without its offset (HIERARCHY 4.11: declared, never
                // guessed). The flat channel below carries no value gate (its gate is a weight,
                // PaintColorTile), so the photo stands ungated there as it always did.
                std::map<const ColorSource*, LevelGateSource*> levelGateOf;
                for (const auto& s : A->sceneSources) {
                    if (s->Height()) continue;
                    const RasterEntry& e = s->Entry();
                    if (e.hasCaptureLevel || !e.captureDatum.empty()) {
                        double levelM = 0.0;
                        std::string said, why;
                        if (hgtCh < 0) why = "no height channel stands to read the ground from";
                        if (!why.empty() || !LevelOnFrame(e, model, datumOff, levelM, said, &why)) {
                            Log("[capture] %s REFUSED: %s -- the photo is not drawn",
                                s->Info().name.c_str(), why.c_str());
                            continue;
                        }
                        auto g = std::make_unique<LevelGateSource>();
                        g->Load(*s, &compositor, hgtCh, levelM, said);
                        levelGateOf[s.get()] = g.get();
                        A->levelGates.push_back(std::move(g));
                    }
                    colorStack.push_back(s.get());
                }
                StackOrder(colorStack);
                const std::vector<ColorSource*> photos = colorStack;   // (the stack moves below)
                // M9av: the GLOBAL seafloor under the bed classifier -- the ingested
                // bathymetry's hillshade x sediment ramp, every ocean texel; the classifier
                // keeps its authority inside its own box by painting over it.
                size_t reliefLayer = SIZE_MAX;
                if (S.streaming.seafloor &&
                    srcRelief.Load("data/bed/seafloor_rules.json", &compositor, hgtCh)) {
                    colorStack.push_back(&srcRelief);
                    reliefLayer = colorStack.size() - 1;
                }
                if (srcBed.Load("data/bed/bed_rules.json", &compositor, hgtCh)) {
                    colorStack.push_back(&srcBed);
                    bedLayer = colorStack.size() - 1;
                }
                if (S.Tool("paint-ab")) {   // every colour source's two paints, byte for byte
                    int bad = 0;
                    for (ColorSource* c : colorStack) bad |= tools::RunPaintAB(*c, S.place.anchor[1], S.place.anchor[0]);
                    exitCode = bad;
                    return nullptr;
                }
                // M9ak: THE GATE. The user's rule: "the GIS mask gates, the height band
                // refines". srcBed is the WATER authority and its alpha is a height band,
                // which cannot tell an inland hollow below +1.2 m NAVD from the sea; the
                // survey can, and cannot place a waterline to the metre. So the survey
                // multiplies the bed's weight and the height band decides where inside it.
                //
                // The mask is a LAYER (so it earns a cache identity and its own tree on the
                // same addresses as the imagery) that PAINTS NOTHING. Its footprint is the
                // rings' own bounds, so every tile outside New England keeps the identity
                // it already has -- the global cube is not repainted for this.
                const size_t bedIdx = bedLayer;
                if (S.streaming.gisGate && (bedIdx != SIZE_MAX || reliefLayer != SIZE_MAX) &&
                    gisMask.Load("data/gis/", S.streaming.gisLand, S.streaming.gisWater, S.data.edits)) {
                    // A river reach is the sea's where the solver stands (GisMask.h SetTidalBox): the
                    // carve's FCode says river, the solver's domain says tidal. Before Refresh (the box
                    // joins the mask's fingerprint).
                    if (swe.Ready()) {
                        const SweDomain& td = swe.Domain();
                        gisMask.SetTidalBox(td.lat0, td.lat1, td.lon0, td.lon1);
                    }
                    srcGisMask.SetGrain(S.streaming.gisGrainM);   // F16: the survey's grain
                    // The height's witness: a height layer finer than that grain draws the water's
                    // edge where it has a value (GisMask.h SetHeight; the shipped scenes have none).
                    if (hgtCh >= 0) srcGisMask.SetHeight(&compositor, hgtCh);
                    srcGisMask.Refresh();   // the rings are loaded: declare the real box and the finer layers
                    if (S.Tool("gis-dump")) tools::RunGisDump(opt, gisMask);
                    if (S.Tool("gis-sweep-test")) tools::RunGisSweepTest(gisMask);   // F15
                    colorStack.push_back(&srcGisMask);
                    maskLayer = colorStack.size() - 1;
                }
                colCh = compositor.AddColorChannel("earth.color", std::move(colorStack));
                if (maskLayer != SIZE_MAX) {
                    if (bedIdx != SIZE_MAX) compositor.SetColorGate(colCh, bedIdx, maskLayer);
                    if (reliefLayer != SIZE_MAX) {
                        compositor.SetColorGate(colCh, reliefLayer, maskLayer);
                    }
                }
                // M9am: THE MEGATEXTURE, AS THE WATER'S GRAPH. The same nodes the bed and
                // the tide compose through -- ColorLayerSource, NormalizeToSi,
                // DomainCompositor::LayeredOver, CompositeSource -- with TileTree caching
                // every node's output on the NVMe on the shared tile addresses:
                //
                //     google ---+
                //               +-> earth.land ----------------+
                //     aerial ---+                              |
                //                                              +-> earth.color (mega)
                //     relief --+                              |
                //              +-> earth.seafloor --+          |
                //     bed -----+                    +-> gate --+
                //     gis.landsea (vector) ---------+
                //
                // Land is the base; the seafloor paints over it with weight = its own height
                // band x the survey's water coverage ("the GIS mask gates, the height band
                // refines"), so over surveyed water the land goes transparent per pixel and
                // the seafloor shows; inland the seafloor is gated out and the mega tile is a
                // stored REFERENCE to the land tree's. The flat incumbent channel above stays
                // as the definition the audit compares against.
                std::vector<std::shared_ptr<DomainSource>>& keepAlive = megaKeep;
                auto leaf = [&](ColorSource* s) -> std::shared_ptr<DomainSource> {
                    auto l = std::make_shared<ColorLayerSource>(s);
                    auto n = NormalizeToSi(l);
                    keepAlive.push_back(l);
                    return n ? n : l;
                };
                auto over = [&](const char* name,
                                std::vector<std::shared_ptr<DomainSource>> in)
                    -> std::shared_ptr<DomainSource> {
                    auto dc = std::make_shared<DomainCompositor>();
                    dc->SetBlend(DomainCompositor::Blend::LayeredOver);
                    for (auto& s : in) {
                        if (!dc->Add(s)) Log("[mega] %s: input %s refused", name, s->Name());
                    }
                    return std::make_shared<CompositeSource>(name, dc);
                };
                std::vector<std::shared_ptr<DomainSource>> landIn;
                for (ColorSource* s : photos) {
                    std::shared_ptr<DomainSource> l = leaf(s);
                    // The capture level's gate (above): the photo's weight times the ground above it.
                    const auto g = levelGateOf.find(s);
                    if (g != levelGateOf.end()) {
                        l = std::make_shared<GateSource>(s->Info().name + "<ground", l, leaf(g->second));
                    }
                    landIn.push_back(l);
                }
                std::shared_ptr<DomainSource> land = over("earth.land", landIn);
                std::shared_ptr<DomainSource> mega;
                std::vector<std::shared_ptr<DomainSource>> seaIn;
                if (reliefLayer != SIZE_MAX) seaIn.push_back(leaf(&srcRelief));
                if (bedLayer != SIZE_MAX) seaIn.push_back(leaf(&srcBed));
                if (!seaIn.empty()) {
                    std::shared_ptr<DomainSource> sea = over("earth.seafloor", seaIn);
                    std::shared_ptr<DomainSource> seaGated = sea;
                    if (maskLayer != SIZE_MAX) {
                        seaGated = std::make_shared<GateSource>("seafloor<gis", sea,
                                                                leaf(&srcGisMask));
                    }
                    mega = over("earth.color", {land, seaGated});
                } else {
                    mega = land;
                }
                keepAlive.push_back(land);
                keepAlive.push_back(mega);
                PrintTree("earth.color (megatexture)", mega.get());
                if (S.streaming.colorTrees || treeTool) {
                    megaTree = std::make_unique<TileTree>(mega.get());
                    megaTree->Print();
                }
                auto mkColor = [&](const ColorFrame& f) -> TileProviderFn {
                    return (S.streaming.colorTrees && megaTree) ? megaTree->Provider(f)
                                                        : compositor.ColorRealization(colCh, f);
                };
                // M9ap: NO INSET TEXTURES. The planet's colour is ONE tenant -- a reserved
                // Texture2DArray of pages: slices 0..5 the cube faces, then the eye's
                // windows -- with one SRV, one residency map, one budget, and
                // one provider that dispatches on the slice. The three tenants this
                // replaces were three pages of a ladder with hand-off fades between them.
                // M12 step 3e: THE DECLARATION -- the cube as a slice binding on its lattice,
                // the windows as blocks; sRGB colour with the coverage in the alpha,
                // 128x128 tiles; a missing tile is not loaded yet (the residency map clamps).
                const Lattice& cCubeL = surface.cube;
                hal::TenantDesc cd;
                cd.name = L"earth.color (megatexture pages)";
                cd.astNode = "color.pages";
                cd.fiber = {DXGI_FORMAT_R8G8B8A8_UNORM_SRGB, 128, 128,
                            "sRGB colour, alpha = coverage"};
                cd.semantics = hal::Semantics::Texture;
                cd.residence = hal::Residence::Streamable;
                cd.absence = hal::Absence::Unloaded;
                cd.slices = SurfaceFrame::WindowSlices();
                cd.bindings.push_back({0, 6, cCubeL, mkColor(cCubeL), "paint cube faces"});
                // F14: what the tree knows of a tile without painting it -- its parent, magnified
                // (HIERARCHY 4.20, the third clause) -- declared beside the provider so the order
                // never spends a load to be told.
                if (S.streaming.colorTrees && megaTree) cd.bindings.back().magnified = megaTree->MagnifiedQuery(cCubeL);
                // PHASE A1: THE EYE'S WINDOWS, slot s's rank k at
                // SurfaceFrame::WindowSlice(s, k), painted by the tree on the pyramid's lattice
                // (it is asked the global tile; the binding keeps the slot). Each is born on its
                // rung's first block of face 0 and moved to its eye by the frame loop's step
                // (SurfaceFrame::Follow, hal::Tenant::Move) before its first want.
                surface.WindowBlocks(cd.blocks,
                                     mkColor(hal::BlockBinding::Pyramid(cd.fiber.texW, cd.fiber.texH)),
                                     "paint pyramid windows");
                if (S.streaming.colorTrees && megaTree) {
                    const TileQueryFn q = megaTree->MagnifiedQuery(hal::BlockBinding::Pyramid(cd.fiber.texW, cd.fiber.texH));
                    for (hal::BlockSlice& k : cd.blocks) k.magnified = q;
                }
                colorTenant = hal::Tenant::Sparse(gpu, resMgr, std::move(cd));
                colorCubeT = colorTenant.Id();
                // M9bb: a fold or a drop below changed a root tile: the colour tenant
                // refetches that address (the frame's tag names the page slice) -- Tenant::Bind.
                if (megaTree) colorTenant.Bind(*megaTree);
                // M9ay: THE SURVEY AS A PAGE TENANT. gis.landsea's own tree -- the vector
                // rings swept per tile on the SAME addresses as the imagery and the bed --
                // feeds a third page tenant (r = water coverage, b = edited, a = surveyed).
                // The classifier reads it; the three committed rasters GisStencil built
                // from the .raw parity fills are gone (AUDIT_WATER item 2). Addresses the
                // survey has no opinion about have no tile: the loader marks them NULL after
                // its retries and the shader falls back to the height sign there.
                if (S.streaming.colorTrees && megaTree && maskLayer != SIZE_MAX) {
                    if (TileTree* gt = megaTree->Find("gis.landsea")) {
                        // M12 step 3e: THE DECLARATION -- the survey on the colour's three
                        // lattices, painted by its own node of the megatexture tree. Where the
                        // survey has no opinion there is no tile, and the shader falls back to
                        // the height sign.
                        hal::TenantDesc md;
                        md.name = L"gis.landsea (survey mask pages)";
                        md.astNode = "mask.pages";
                        md.fiber = {DXGI_FORMAT_R8G8B8A8_UNORM, 128, 128,
                                    "r = water coverage, b = edited, a = surveyed; no tile = "
                                    "no opinion, the shader falls back to the height sign"};
                        md.semantics = hal::Semantics::Texture;
                        md.residence = hal::Residence::Streamable;
                        md.absence = hal::Absence::Unloaded;
                        md.slices = SurfaceFrame::WindowSlices();
                        md.bindings.push_back({0, 6, cCubeL, gt->Provider(cCubeL),
                                               "paint survey mask (cube faces)"});
                        // PHASE A1: the colour's windows, its slices (PHASE B2: one declaration).
                        surface.WindowBlocks(md.blocks,
                                             gt->Provider(hal::BlockBinding::Pyramid(md.fiber.texW, md.fiber.texH)),
                                             "paint survey mask (pyramid windows)");
                        // F16: the mask knows its finer tiles are the parent magnified (the
                        // survey's grain), so the question is declared here too (F14).
                        md.bindings.back().magnified = gt->MagnifiedQuery(cCubeL);
                        {
                            const TileQueryFn q = gt->MagnifiedQuery(hal::BlockBinding::Pyramid(md.fiber.texW, md.fiber.texH));
                            for (hal::BlockSlice& k : md.blocks) k.magnified = q;
                        }
                        landseaTenant = hal::Tenant::Sparse(gpu, resMgr, std::move(md));
                        maskTenant = landseaTenant.Id();
                        landseaTenant.Bind(*gt);   // its folds invalidate the slice its tag names
                        Log("[gis] the survey is page tenant %d: r = water coverage, b = "
                            "edited, a = surveyed -- no .raw raster is opened",
                            maskTenant);
                    } else {
                        Log("[gis] no gis.landsea node in the megatexture tree: the "
                            "classifier falls back to the height sign");
                    }
                }
                // THE BUILDINGS UNDER A PIXEL (compose/BuildingField.h): the folded tree as three
                // additive scalars a texel -- lambda_p, volume per area, lambda_f -- of the buildings
                // narrower than the texel (the wider are the prisms'), on the colour's addresses
                // and windows, its own tree on the NVMe. No tile = no building there.
                if (S.streaming.colorTrees && S.LayerOn("buildings") && !marsMode) {
                    const SceneLayer* bl = S.Layer("buildings");
                    if (bl && !bl->lod.empty() && bl->field) {
                        auto lodf = std::make_shared<BuildingLodFile>();
                        std::string why;
                        std::error_code ec;
                        const auto man = std::filesystem::path(bl->lod) / "lod.json";
                        const auto sz = std::filesystem::file_size(man, ec);
                        const auto tm = ec ? 0 : std::filesystem::last_write_time(man, ec).time_since_epoch().count();
                        if (lodf->Open(bl->lod, &why)) {
                            A->bldLod = lodf;
                            A->srcBldField = std::make_unique<BuildingFieldSource>(
                                lodf, bl->lod + " (lod.json " + std::to_string(ec ? 0 : sz) + " B, " + std::to_string(tm) + ")");
                            auto l = std::make_shared<ColorLayerSource>(A->srcBldField.get());
                            keepAlive.push_back(l);
                            A->bldFieldLeaf = l;
                            A->bldFieldTree = std::make_unique<TileTree>(l.get());
                            A->bldFieldTree->Print();
                            TileTree* bt = A->bldFieldTree.get();
                            hal::TenantDesc bd;
                            bd.name = L"buildings.field (pages)";
                            bd.astNode = "buildings.field";
                            bd.fiber = {DXGI_FORMAT_R8G8B8A8_UNORM, 128, 128,
                                        "r = lambda_p, g = volume per area / 64 m, b = lambda_f / 4: the "
                                        "buildings narrower than the texel; no tile = none"};
                            bd.semantics = hal::Semantics::Texture;
                            bd.residence = hal::Residence::Streamable;
                            bd.absence = hal::Absence::Unloaded;
                            bd.slices = SurfaceFrame::WindowSlices();
                            bd.bindings.push_back({0, 6, cCubeL, bt->Provider(cCubeL), "paint building field (cube faces)"});
                            surface.WindowBlocks(bd.blocks,
                                                 bt->Provider(hal::BlockBinding::Pyramid(bd.fiber.texW, bd.fiber.texH)),
                                                 "paint building field (pyramid windows)");
                            A->buildingTenant = hal::Tenant::Sparse(gpu, resMgr, std::move(bd));
                            A->buildingTenant.Bind(*bt);
                            Log("[buildings] the field under a pixel is page tenant %d (lambda_p, volume, lambda_f) "
                                "from %s", A->buildingTenant.Id(), bl->lod.c_str());
                        } else {
                            Log("[buildings] no field under a pixel: %s", why.c_str());
                        }
                    }
                }
                if (treeTool) {
                    exitCode = tools::RunTreeAudit(opt, compositor, hgtCh, resMgr, colCh,
                                                   megaTree, heightTree, surface);
                    return nullptr;
                }
            }
            // M12 step 4a: the tenants, now that they exist, into the surface's declaration
            // (ids and page slices read off hal::Tenant); the globe takes the surface itself
            // beside SetResidency below, where the old SetPlanetRadius was.
            surface.Declare(colorTenant, heightTenant, landseaTenant, A->buildingTenant);

            // ---- M9ae: WHAT THE DISK ALREADY HOLDS, in memory, once.
            //
            // The composed cache is the tree's backing store -- 64 KB tiles laid out for
            // CopyTiles -- but nothing in RAM knew what was in it, so "is a finer level
            // ready here?" could only be answered by building a filename and asking the
            // filesystem. Residency could request a level; it could not prefer the levels
            // that would be a cheap READ over the ones that mean a paint or a fetch.
            //
            // The index is one entry per TILE, not per texel, so the asymmetry that makes
            // NVMe paging worth doing shows up directly: terabytes of tree, megabytes of map.
            {
                idxColorCube.Scan("earth.color", "cube16k");
                idxHeightCube.Scan("earth.height", "cube16k");
                idxColorCube.Report();
                idxHeightCube.Report();
                // Each tenant gets the index of its OWN realization -- the scheduler then
                // prefers loads that are a read over loads that are a paint.
                resMgr.SetTileIndex(colorCubeT, &idxColorCube);
                resMgr.SetTileIndex(hgtTenant, &idxHeightCube);
                // M9ag: the NVMe -> GPU reader. Created once; a machine without the
                // redist or with a driver that declines keeps the ReadFile path.
                // M9ao: ON by default. Through the packed trees the streamed path is
                // pixel-identical to the upload ring (section 33); the user made it the
                // default. --no-direct-storage is the A/B.
                if (S.streaming.directStorage) {
                    if (tileStream.Init(gpu)) resMgr.SetTileStream(&tileStream);
                } else {
                    Log("[dstorage] OFF by request (--no-direct-storage): every tile takes "
                        "the upload ring");
                }
            }

        }
        if (opt.lens) globe->debugLens = opt.lens;   // (the scene's residency view, set above, stands otherwise)
        globe->probeCullFar = opt.probeCullFar;
        globe->waterTileCount = opt.waterTiles;   // M13 step 0
        // M9h: the grad(flow) bank plus the grid it lives on, for --lens velgrad. The
        // SWE solver owns the bank; the bathy model owns the world mapping.
        if (swe.Ready()) {
            // The residency map and the bank's texel size ride along: the lens picks the
            // level its footprint wants and the map clamps it to what has arrived.
            // ---- M9i: CLOSE THE LOOP. The regional current becomes a GA object through
            // the plugin seam, composes into a page, and lands in the bank's COARSE levels.
            // Nothing here reaches into another layer: a loader answers where/what/where-not,
            // a compositor answers value-and-weight, and the bank takes texels. The solver
            // never learns that GoMOFS exists, which is exactly what the backed-out version
            // got wrong.
            //
            // grad() is taken on the composed page rather than on the source, because
            // divergence and vorticity are properties of the COMPOSITE -- taking them
            // per-source and blending afterwards would average two different derivatives.
            if (swe.Ready()) {
                // EVERY field of the scene's currents, coarsest first, so the finer stand over the coarser
                // where they overlap and a page no field reaches composes nothing (CurrentFieldLoader::
                // OpenAll). A composite holds many regions' fields; one of them -- its finest anywhere, or
                // the one holding the solver's anchor -- is not the solver's whole box.
                if (auto fieldLoaders = CurrentFieldLoader::OpenAll(S.data.currents); !fieldLoaders.empty()) {
                    DomainCompositor fc;
                    LevelLadder lad;
                    lad.level0MetersPerTexel = swe.CellM();
                    fc.SetLadder(lad);
                    std::vector<std::shared_ptr<RasterSource>> rasters;
                    for (auto& cld : fieldLoaders) {
                        rasters.push_back(std::make_shared<RasterSource>(std::move(cld), 0));
                        fc.Add(rasters.back());
                    }

                    GradeBank& vb = swe.VelGradBank();
                    uint32_t wrote = 0;
                    // Levels coarse enough that a ~700 m model is honest there.
                    for (uint32_t lvl = 3; lvl < vb.MipCount(); ++lvl) {
                        const uint32_t w = (std::max)(1u, swe.Nx() >> lvl);
                        const uint32_t h = (std::max)(1u, swe.Ny() >> lvl);
                        // PHASE C1: the page about the solver's own anchor -- its box's south-west
                        // corner, the level's texel in degrees at the anchor's latitude (the domain's
                        // chart to second order across the box; these levels are 80 m and up).
                        DomainCompositor::PageGeo geo;
                        const SweDomain& sd = swe.Domain();
                        const double mpt = swe.CellM() * double(1u << lvl);
                        const double mPerLat = sd.R * 3.14159265358979323846 / 180.0;
                        const double mPerLon = mPerLat * std::cos(sd.latC * 3.14159265358979323846 / 180.0);
                        geo.lon0 = sd.lon0 + 0.5 * mpt / mPerLon;
                        geo.lat0 = sd.lat0 + 0.5 * mpt / mPerLat;
                        geo.dLon = mpt / mPerLon;
                        geo.dLat = mpt / mPerLat;

                        std::vector<float> uv, cov;
                        const uint32_t covered =
                            fc.ComposePage(PageAddr{lvl, 0, 0}, geo, w, h, 2, uv, cov);
                        if (!covered) continue;

                        // grad(flow) on the composed page: divergence to grade 0, vorticity
                        // to grade 2, exactly what the type says kG1 * kG1 produces. Coverage
                        // rides in .z so the shader composite can weight it, and a texel with
                        // no current stays absent rather than reading as still water.
                        std::vector<float> rgba(size_t(w) * h * 4, 0.0f);
                        const double dxm = mpt, dym = mpt;
                        for (uint32_t y = 0; y < h; ++y) {
                            for (uint32_t x = 0; x < w; ++x) {
                                const size_t i = size_t(y) * w + x;
                                if (cov[i] <= 0.0f) continue;
                                const uint32_t xm = (x > 0) ? x - 1 : x;
                                const uint32_t xp = (x + 1 < w) ? x + 1 : x;
                                const uint32_t ym = (y > 0) ? y - 1 : y;
                                const uint32_t yp = (y + 1 < h) ? y + 1 : y;
                                const size_t a = (size_t(y) * w + xm) * 2;
                                const size_t b = (size_t(y) * w + xp) * 2;
                                const size_t c = (size_t(ym) * w + x) * 2;
                                const size_t d = (size_t(yp) * w + x) * 2;
                                const float dudx = (uv[b + 0] - uv[a + 0]) / float(2 * dxm);
                                const float dvdx = (uv[b + 1] - uv[a + 1]) / float(2 * dxm);
                                const float dudy = (uv[d + 0] - uv[c + 0]) / float(2 * dym);
                                const float dvdy = (uv[d + 1] - uv[c + 1]) / float(2 * dym);
                                rgba[i * 4 + 0] = dudx + dvdy;   // grade 0: divergence
                                rgba[i * 4 + 1] = dvdx - dudy;   // grade 2: vorticity
                                rgba[i * 4 + 2] = cov[i];        // coverage
                            }
                        }
                        vb.UploadLevel(gpu, lvl, rgba.data(), w * 4 * sizeof(float), w, h);
                        ++wrote;
                    }
                    Log("[compose] grad(current, %zu fields) -> swe.velgrad slice 0 levels 3..%u: %u "
                        "composed (loader -> source -> page -> bank)",
                        rasters.size(), vb.MipCount() - 1, wrote);

                    // ---- SLICE 1: THE REGION. Same sources, same compositor, a page over
                    // the REGION's own ground instead of the window's. This is what makes
                    // the wind fallback unnecessary: outside the solve there is now a page
                    // of the SAME quantity in the SAME units, so the consumer composites
                    // two pages of one bank instead of blending in a foreign field.
                    // The region is the union of the fields that reach the solver's box -- not every
                    // field of a composite, whose union may span a coast -- and none reaching it is
                    // no region.
                    const SweDomain& sdr = swe.Domain();
                    double rl0 = 1e9, ra0 = 1e9, rl1 = -1e9, ra1 = -1e9;
                    for (const auto& r : rasters) {
                        const GeoRef& g = r->Ref();
                        const double l0 = g.originX, l1 = g.originX + g.scaleX * g.width;
                        const double a0 = (std::min)(g.originY, g.originY + g.scaleY * g.height);
                        const double a1 = (std::max)(g.originY, g.originY + g.scaleY * g.height);
                        if (l1 < sdr.lon0 || l0 > sdr.lon1 || a1 < sdr.lat0 || a0 > sdr.lat1) continue;
                        rl0 = (std::min)(rl0, l0); rl1 = (std::max)(rl1, l1);
                        ra0 = (std::min)(ra0, a0); ra1 = (std::max)(ra1, a1);
                    }
                    if (vb.Slices() > 1 && rl1 > rl0 && ra1 > ra0) {
                        // Every level, not just the floor: an upload into an unmapped tile
                        // is discarded and the page would simply not be there.
                        vb.MapAllLevels(gpu, 1);
                        GeoRef rg;
                        rg.originX = rl0;
                        rg.originY = ra0;
                        rg.width = rg.height = 1;
                        rg.scaleX = rl1 - rl0;
                        rg.scaleY = ra1 - ra0;
                        uint32_t rwrote = 0;
                        // Start where the bank's texels approach the source's own scale.
                        // Differentiating a field upsampled 8x invents structure that is
                        // not in it -- the derivative of the interpolation, not of the
                        // current -- so the region owns only levels it can honestly fill,
                        // and the residency clamp keeps a sampler from asking for finer.
                        for (uint32_t lvl = 3; lvl < vb.MipCount(); ++lvl) {
                            const uint32_t w = (std::max)(1u, swe.Nx() >> lvl);
                            const uint32_t h = (std::max)(1u, swe.Ny() >> lvl);
                            // The page spans the SOURCE's extent, sampled onto this
                            // level's grid -- the region gets its own geography, not the
                            // window's stretched over it.
                            DomainCompositor::PageGeo rgeo;
                            rgeo.dLon = rg.scaleX * double(rg.width) / double(w);
                            rgeo.dLat = rg.scaleY * double(rg.height) / double(h);
                            rgeo.lon0 = rg.originX + 0.5 * rgeo.dLon;
                            rgeo.lat0 = rg.originY + 0.5 * rgeo.dLat;

                            std::vector<float> ruv, rcov;
                            const uint32_t rc =
                                fc.ComposePage(PageAddr{lvl, 0, 0}, rgeo, w, h, 2, ruv, rcov);
                            if (!rc) continue;
                            std::vector<float> rr(size_t(w) * h * 4, 0.0f);
                            // Metres per texel of THIS page: the region's degrees converted
                            // at its own latitude, not the window's frozen anchor.
                            const double mx = std::abs(rgeo.dLon) * 111319.49 *
                                              std::cos(rgeo.lat0 * 3.14159265 / 180.0);
                            const double my = std::abs(rgeo.dLat) * 110574.0;
                            for (uint32_t y = 0; y < h; ++y) {
                                for (uint32_t x = 0; x < w; ++x) {
                                    const size_t i = size_t(y) * w + x;
                                    if (rcov[i] <= 0.0f) continue;
                                    const uint32_t xm = (x > 0) ? x - 1 : x;
                                    const uint32_t xp = (x + 1 < w) ? x + 1 : x;
                                    const uint32_t ym = (y > 0) ? y - 1 : y;
                                    const uint32_t yp = (y + 1 < h) ? y + 1 : y;
                                    const size_t a = (size_t(y) * w + xm) * 2;
                                    const size_t b = (size_t(y) * w + xp) * 2;
                                    const size_t c2 = (size_t(ym) * w + x) * 2;
                                    const size_t d2 = (size_t(yp) * w + x) * 2;
                                    const float dudx = float((ruv[b + 0] - ruv[a + 0]) / (2 * mx));
                                    const float dvdx = float((ruv[b + 1] - ruv[a + 1]) / (2 * mx));
                                    const float dudy = float((ruv[d2 + 0] - ruv[c2 + 0]) / (2 * my));
                                    const float dvdy = float((ruv[d2 + 1] - ruv[c2 + 1]) / (2 * my));
                                    rr[i * 4 + 0] = dudx + dvdy;
                                    rr[i * 4 + 1] = dvdx - dudy;
                                    rr[i * 4 + 2] = rcov[i];
                                }
                            }
                            vb.UploadLevel(gpu, lvl, rr.data(), w * 4 * sizeof(float), w, h,
                                           1);
                            ++rwrote;
                        }
                        // The lens needs the region page's geography to sample it.
                        globe->SetVelGradRegion(
                            rg.originX, rg.originY, rg.scaleX * double(rg.width),
                            rg.scaleY * double(rg.height));
                        Log("[compose] grad(current) -> swe.velgrad slice 1 (REGION): %u "
                            "levels over %.2f x %.2f deg -- the wind fallback is now "
                            "unnecessary",
                            rwrote, std::abs(rg.scaleX) * rg.width,
                            std::abs(rg.scaleY) * rg.height);
                    }
                }
            }
            // The lens draws in world.flat (the globe's chart, C4/C5's): the domain's box through it.
            double lx0 = 0.0, lz0 = 0.0, lx1 = 0.0, lz1 = 0.0;
            surface.flat.FlatOf(swe.Domain().lat0, swe.Domain().lon0, lx0, lz0);
            surface.flat.FlatOf(swe.Domain().lat1, swe.Domain().lon1, lx1, lz1);
            globe->SetVelGradLens(
                swe.VelGradSrv(), float(lx0), float(lz0), float(lx1 - lx0), float(lz1 - lz0),
                swe.VelGradResMapSrv(),
                static_cast<float>((lx1 - lx0) / (std::max)(1u, swe.Nx())),
                static_cast<float>(swe.VelGradResMapW()),
                static_cast<float>(swe.VelGradResMapH()), swe.VelGradMips());
        }
        // M12 step 5e: the cutaway plane is an EFFECT NODE (scene/effects/SlicePlane.h): the
        // declaration from the `effects` section, the globe as its observer, the fan-out its
        // own -- the two lines that wrote the globe's fields by hand.
        A->slice.Declare(sliceFx ? sliceFx->p.name : std::string("slice"), sliceOn,
                         sliceOn ? sliceFx->d : 0.0);
        {
            scene::SlicePlane::Observers so;
            so.globe = globe;
            A->slice.Configure(so);
        }
        A->slice.FanOut();
        compositor.LogRegistry();
        // M8j: --fidelity-map draws that same registry. It runs HERE, not at the
        // --water-map exit, because earth.color is registered 200 lines later than
        // earth.height -- the first cut rendered a sheet with the skin channel simply
        // missing, which is the exact class of error this picture exists to catch.
        if (S.Tool("fidelity-map")) tools::RunFidelityMap(opt, compositor, surface.flat);
        // M7j: the GA AST -- the state diagram printed and validated EVERY run, so a
        // frame mismatch or an orphaned field is a boot-time report, not a debugging
        // session. (The workflow as an AST: domains, axes, units, scales, ranges.)
        // M12 step 4c: the compose pillar's paint rows are the shipped surface's declaration
        // -- the tenants' nodes and edges, their slices, the lattices' frames, the flip
        // derived -- registered here: after Declare() (the tenants exist), before the hand
        // table (the rows keep the head of the diagram, where they have always printed) and
        // before the validator below (the flip rule is checked on rows that exist). Mars
        // declares no page tenant and gets no paint row, which is the truth of it.
        surface.RegisterEdges();
        ga::ast::RegisterKnownWaterEdges();
        // M9bh: --pixel-water re-opens eight edges into the pixel stage (the two rays
        // and what they read). Declared only when the flag is on -- an edge for a mode
        // the run is not in is graph rot wearing the other sign.
        if (S.water.pixelWater) ga::ast::RegisterPixelWaterEdges();
        // M9bi: the sun's own edges, when the ephemeris is the one driving it.
        if (S.sun.source != Scene::kPinned) ga::ast::RegisterSolarEdges();
        if (sliceOn) {
            // M7o: the demo node registers its edge like any other -- the AST is how
            // features arrive now. One blade, one inner product, one discard. M12 step 5e:
            // the effect node registers it (Effect::RegisterEdges -- the same row).
            A->slice.RegisterEdges();
        }
        ga::ast::Print();
        ga::ast::Validate();
        ga::ast::WriteMarkdown("docs/GA_AST.md");   // the scriptorium indexes this
        ga::ast::WriteJson("docs/ga_ast.json");     // the Blueprint contract (M7u)
        // M12 step 5f: THE OTHER TWO CONTRACTS THE UI CONSUMES (the plan's section F), written
        // here because this is where a generated document is written -- same boot, same
        // determinism, same LF, checked in, a zero-line diff when nothing moved. The scene
        // contract is read off the Schema tables; the name contract adds the registries that
        // live outside scene/ and whose owner is the only honest source of "what this build
        // registered".
        {
            scene::Schema::WriteSchema("docs/scene_schema.json");
            std::vector<scene::RegistryDoc> extra;   // (the hulls: WriteRegistries builds them)
            // The loader registries are built where a file is opened (the bathy grid and the
            // current field, Assembly.cpp; --load-field's own), each registering the types it
            // can answer for -- so this is the union of what those sites register, named here
            // because there is no one live registry to read.
            extra.push_back({"loader", "a field file type (core/FieldLoader.h LoaderRegistry)",
                             {"f32", "json"}});
            extra.push_back({"tool", "a one-shot mode in `tools[]` (--tool name[:args])",
                             {"bathy-map", "building-lod", "dump-water-state", "export", "fidelity-map",
                              "gis-dump", "gis-sweep-test", "height-tree-probe", "fetch-probe", "exposure-ab", "ingest", "paint-ab", "load-field", "ocean-probe", "pack-tiles",
                              "pack-trees", "rastertest", "roads-probe", "sea-verify", "selftest", "swe-cycle",
                              "swe-uv", "trace",
                              "tree-audit", "tree-prune", "twin-surface", "warm-inlet", "warm-trees",
                              "water-map", "wave-blend-test", "wave-converge", "wave-map", "wave-recheck"}});
            scene::WriteRegistries("docs/registries.json", extra);
        }
        // The survey pack loads whenever it exists: the land MASKS are the default
        // classifier (always on); the VECTOR overlay draws only under --stencil.
        if (!marsMode && gisStencil.Load("data/gis/gis.json")) {
            // M9ay: no raster is built here any more; the classifier reads the mask pages.
            vectors.Load("data/vectors/vectors.vpack");
            auto gisOwned = std::make_unique<GisLayer>();
            gisLayer = gisOwned.get();
            gisLayer->Configure(shaderDir, &gisStencil, &exchange, &vectors);
            gisLayer->Init(gpu, renderer.Shaders(), fields, renderer.RootSignature());
            gisLayer->enabled = opt.stencil;
            renderer.AddLayer(std::move(gisOwned));
        }
        if (!marsMode && !S.entities.empty()) {
            auto vlOwned = std::make_unique<VesselLayer>();
            vlOwned->Configure(shaderDir);
            vlOwned->Init(gpu, renderer.Shaders(), fields, renderer.RootSignature());
            vesselLayer = vlOwned.get();
            renderer.AddLayer(std::move(vlOwned));
        }
        if (!marsMode) {
            auto mkOwned = std::make_unique<MarkerLayer>();
            mkOwned->Configure(shaderDir, &exchange, "markers.stations");
            mkOwned->Init(gpu, renderer.Shaders(), fields, renderer.RootSignature());
            mkOwned->enabled = !opt.albedo;   // the lens shows textures, nothing else
            renderer.AddLayer(std::move(mkOwned));
        }
        // THE BUILDINGS (compose/BuildingSolids.h), registered last (a scene that names them
        // appends them to its base's list): the scene's sources of kind "buildings", a
        // stack by `over` like the earth's, composed into solids and drawn by their own layer --
        // never into earth.height, so the water solver's bed does not see them (2026-10-09).
        if (!marsMode && S.LayerOn("buildings")) {
            const SceneLayer* bl = S.Layer("buildings");
            // The stack is OPENED here (each harvest's cell index, each GeoJSON read whole) and
            // composed cell by cell about the eye as it moves (scene/BuildingLayer: the streaming).
            std::vector<BuildingSourceSpec> specs;
            for (const scene::SourceProps& s : S.sources) {
                if (s.kind == "buildings") specs.push_back({s.name, s.manifest.empty() ? s.file : s.manifest, s.over});
            }
            auto stack = std::make_shared<BuildingStack>();
            std::string log;
            stack->Open(std::move(specs), {bl->levelHeight, bl->defaultHeight}, &log);
            for (size_t p0 = 0, p1; (p1 = log.find('\n', p0)) != std::string::npos; p0 = p1 + 1) {
                Log("[buildings] %s", log.substr(p0, p1 - p0).c_str());
            }
            Log("[buildings] streamed by %.2f deg cell within %.0f m of the eye (floor %.2f m, untagged %.2f m)",
                BuildingLayer::kCellDeg, bl->radius, bl->levelHeight, bl->defaultHeight);
            const SurfaceFrame* sf = &surface;
            const Compositor* comp = &compositor;
            const int ch = hgtCh;
            const double R = planetR;
            auto blOwned = std::make_unique<BuildingLayer>();
            A->buildingStack = stack;   // the vehicles' ground law composes the same solids about the hull
            blOwned->Configure(
                shaderDir, stack, bl->radius,
                // The marker pylons' placement (FrameLoop): the planet point on the sphere of the
                // globe, rotated into the flat frame whose origin is the anchor on the surface.
                [sf, R](double lat, double lon, double h, double out[3]) {
                    double d[3];
                    GlobeModel::LatLonDir(lat, lon, d);
                    const double r = R + h;
                    out[0] = (sf->east[0] * d[0] + sf->east[1] * d[1] + sf->east[2] * d[2]) * r;
                    out[1] = (sf->up[0] * d[0] + sf->up[1] * d[1] + sf->up[2] * d[2]) * r - R;
                    out[2] = (sf->north[0] * d[0] + sf->north[1] * d[1] + sf->north[2] * d[2]) * r;
                },
                // ...and its inverse: the planet point is east x + up (y + R) + north z; LatLonDir's
                // own convention (y at the pole, z at 90 E) reads latitude and longitude off it.
                [sf, R](const double f[3], double& lat, double& lon, double& h) {
                    constexpr double kDegPerRad = 180.0 / 3.14159265358979323846;
                    double P[3];
                    for (int i = 0; i < 3; ++i) P[i] = sf->east[i] * f[0] + sf->up[i] * (f[1] + R) + sf->north[i] * f[2];
                    const double r = std::sqrt(P[0] * P[0] + P[1] * P[1] + P[2] * P[2]);
                    lat = std::asin(P[1] / r) * kDegPerRad;
                    lon = std::atan2(P[2], P[0]) * kDegPerRad;
                    h = r - R;
                },
                // THE GROUND AT A GRAIN: the pyramid at the rung whose texel is at most the grain asked
                // (the hull's bed law, above), memoised -- a field per job (BuildingLayer::Ground).
                // (Measured 2026-10-11, docs/BUILDING_LOD.md section 11: the tree's own tiles read here
                // paint on a miss, seconds a page where the archive is cold; the stack it is.)
                [comp, ch, g0 = surface.cube.GroundRes(0)](double resM) -> std::unique_ptr<PlaceField> {
                    if (ch < 0) return nullptr;
                    const uint32_t rung = uint32_t(std::clamp(int(std::ceil(std::log2(g0 / (std::max)(resM, 1e-3)))), 0, 15));
                    return std::make_unique<HeightPage>(comp, ch, rung, true);   // the stack: never a paint on Io
                });
            if (!bl->lod.empty()) {   // the folded tree (docs/BUILDING_LOD.md): a missing one is named, not fatal
                auto lodf = std::make_shared<BuildingLodFile>();
                std::string why;
                if (lodf->Open(bl->lod, &why)) {
                    std::string per;
                    for (int L = lod::kLmin; L <= lod::kLmax; ++L) {
                        per += (per.empty() ? "" : ", ") + std::to_string(L) + ":" + std::to_string(lodf->Pages(L));
                    }
                    Log("[buildings] the ranked tree from %s: each building from %.1f px, faded in to twice that; pages by "
                        "level %s", bl->lod.c_str(), bl->lodPixels, per.c_str());
                    blOwned->ConfigureFar(std::move(lodf), bl->lodPixels, bl->shapePixels, bl->pageMB, bl->marginDeg, bl->boxPixels, bl->standMB);
                } else {
                    Log("[buildings] no far boxes: %s", why.c_str());
                }
            }
            blOwned->Init(gpu, renderer.Shaders(), fields, renderer.RootSignature());
            buildingLayer = blOwned.get();
            renderer.AddLayer(std::move(blOwned));
        }
        if (S.hud.fps || S.hud.minimap.enabled) {
            auto hudOwned = std::make_unique<HudLayer>();
            hudOwned->Configure(shaderDir);
            hudOwned->Init(gpu, renderer.Shaders(), fields, renderer.RootSignature());
            hud = hudOwned.get();
            renderer.AddOverlay(std::move(hudOwned));
        }
        globe->SetResidency(&resMgr, surf, norm, marsMode);
        globe->SetSurface(&surface);   // M12 step 4a: its radius, frame, lattices and tenants

        // Field adapters: the whole tiled economy reports on one stats line.
        if (swe.Ready()) {
            resMgr.RegisterField("swe", [&swe] { return swe.ResidentBytes(); },
                                 [&swe] { return swe.ResidentTiles(); });
        }
        if (sea) {
            resMgr.RegisterField("churn", [sea] { return sea->ChurnBytes(); },
                                 [sea] { return sea->ChurnTiles(); });
            resMgr.RegisterField("air", [globe] { return globe->AirBytes(); },
                                 [globe] { return globe->AirTiles(); });
        }

        // The grade-signature registry's first demand derivation: wind published as a
        // Cl(2) field (vector where it blows, bivector where it curls), and the Cayley
        // closure says WHERE any wind-product field (enstrophy, OW, advection terms) can
        // be non-zero -- residency for derived tenants decided by algebra, no data read.
        if (!marsMode && globeModel.WindNx() > 0) {
            const int tx = 6, ty = 6;
            std::vector<uint8_t> sig(tx * ty, 0);
            const int nx = globeModel.WindNx(), ny = globeModel.WindNy();
            for (int y = 0; y < ny; ++y) {
                for (int x = 0; x < nx; ++x) {
                    const float u = globeModel.WindU()[y * nx + x];
                    const float v = globeModel.WindV()[y * nx + x];
                    const int i = (y * ty / ny) * tx + (x * tx / nx);
                    if (u * u + v * v > 36.0f) sig[i] |= 0b010;          // grade 1: wind
                    const int xr = (std::min)(nx - 1, x + 1);
                    const int yd = (std::min)(ny - 1, y + 1);
                    const float curl = (globeModel.WindV()[y * nx + xr] - v) -
                                       (globeModel.WindU()[yd * nx + x] - u);
                    if (std::abs(curl) > 2.5f) sig[i] |= 0b100;          // grade 2: curl
                }
            }
            resMgr.PublishSignatures("wind10m", tx, ty, sig);
            std::vector<uint8_t> derived;
            uint32_t dx = 0, dy = 0;
            if (resMgr.DeriveDemand("wind10m", "wind10m", derived, dx, dy)) {
                int need = 0;
                for (uint8_t s : derived) {
                    if (s) ++need;
                }
                Log("[residency] Cayley-derived demand for wind-product fields: %d/%d "
                    "tiles (the rest are ALGEBRAICALLY zero -- never allocated, never "
                    "dispatched)",
                    need, dx * dy);
                // M9h: and now it DRIVES. Until this call the closure was a report; the
                // bank's residency came from a CPU curl scan alone.
                if (globe) globe->ApplyWindDemand(gpu, derived, dx, dy);
            }
        }
    }

    // ---- M12 step 5d: THE `layers` LIST, APPLIED AND CHECKED. The list is the registration
    // order and the standing draw declaration (Layer::declared): a layer the scene does not carry
    // -- or carries `"enabled": false` -- is built exactly as it was (the construction order IS
    // the lifetime law and does not move) and never drawn. What the list cannot do silently is
    // disagree with the code: the order the span above registered in must be the declared order
    // with the layers this run could not build left out, or the boot refuses naming the layer.
    // An unknown layer NAME never reaches here -- SceneSchema's registry refuses it at Resolve.
    {
        std::vector<std::string> registered;
        std::string order;
        for (const std::unique_ptr<Layer>& l : renderer.Layers()) {
            l->declared = S.LayerOn(l->Name());
            registered.push_back(l->Name());
            order += (order.empty() ? "" : " ") + std::string(l->Name()) +
                     (l->declared ? "" : "(off)");
        }
        std::string why;
        if (!CheckLayerOrder(S, registered, &why)) {
            Log("FATAL: [scene] %s", why.c_str());
            exitCode = 2;
            return nullptr;
        }
        Log("[scene] draw order (%zu registered, the scene's list): %s", registered.size(),
            order.c_str());
    }
    return A;
}

}  // namespace ga::app
