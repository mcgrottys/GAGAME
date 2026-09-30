// ================================================================================================
//  Assembly - the scene as one object: everything main() builds before the first frame.
//
//  M12 step 1c. The "assembly" span of main() -- from the tide-model load through the residency
//  wiring, ending just before the mode selection -- moved VERBATIM into Assemble(). The
//  struct's members are that span's block-level locals, SAME NAMES, SAME TYPES, IN MAIN'S
//  DECLARATION ORDER, and the order is the design:
//
//  THE LIFETIME LAW. Declaration order is destruction order, and main()'s order is load-bearing.
//  megaKeep/megaTree are declared beside the tenants that hold providers into them "so they
//  cannot die first"; the GlobeLayer's worker joins inside renderer.Shutdown(); five
//  TileTree::onChanged closures capture &resMgr and a tenant id BY REFERENCE and run on the
//  loader threads (heightTree, exposureTree, megaTree, the gis.landsea node, and the loop's
//  exposure re-roll); the page providers capture their sub-providers by value; the
//  resMgr.RegisterField adapters capture &swe, sea and globe. All of it was safe only because
//  main()'s frame never moved. So the Assembly is heap-allocated ONCE (std::unique_ptr), never
//  copied, never moved: every reference a closure took keeps pointing at the object it was bound
//  to, and the members destruct in exactly the reverse of the order main()'s locals did. main()
//  declares the Assembly BEFORE every session and frame-loop local, so it also outlives them,
//  and the explicit shutdown sequence at the end of main() runs through the aliases unchanged.
//
//  THE ALIAS TECHNIQUE keeps the moved bodies verbatim. Assemble() opens with one reference per
//  member (`auto& model = A->model;` ...) and then runs main()'s code unchanged; a lambda that
//  captures `[&resMgr, &hgtTenant]` captures the object the reference is bound to (C++14 CWG
//  2011), so the closures outlive Assemble() safely. main() declares the same aliases after
//  Assemble() returns, so the session and frame-loop code is verbatim too (step 1d absorbs
//  those into the frame-loop class).
//
//  WHAT IS NOT THE LOCAL'S EXACT SHAPE, and why a member cannot be. activeGlobe was a
//  `GlobeModel&` chosen at run time -- a reference member cannot be late-bound -- so it is a
//  pointer here, aliased back to a reference on both sides. srcEtopo and srcMola take the
//  LOADED globe's dimensions as constructor arguments, so they are std::optional, emplaced at
//  the line that used to construct them. A `const` local whose initialiser was code
//  (haveCurrents, datumOff, marsMode, globeDataOk, haveBathyRaw, haveMslLink, mslToNavd,
//  planetR) is a plain member assigned at that line. The four TileIndex objects and the
//  TileStream were function-local statics inside main()'s residency block: as the last members
//  they now destruct FIRST, before gpu -- the statics used to outlive the device. TileStream
//  has no destructor of its own: its members release a device-local staging buffer, the
//  DirectStorage factory/queue/fence and the per-batch file handles, all of which want the
//  device alive, and ResidencyManager::Shutdown() (also run by its destructor, after the
//  stream is gone) touches neither the stream nor the tile indices.
//
//  EARLY EXITS inside the span (the tide-data fatal, a failed window, --water-map, --tree-audit)
//  set `exitCode` and return nullptr; main() returns that code at the same point, and the
//  Assembly unwinds inside Assemble() in the order main()'s locals would have. --gis-dump's
//  std::exit stays a std::exit.
// ================================================================================================
#pragma once

#include "core/ExitTrail.h"
#include "compose/ExposurePage.h"
#include "compose/HeightPage.h"
#include "compose/ExposureSource.h"
#include "compose/GisMask.h"
#include "compose/TileTree.h"
#include "compose/TileIndex.h"
#include "hal/TileStream.h"
#include "compose/DomainSource.h"
#include "hal/Gpu.h"
#include "hal/Residency.h"
#include "hal/Tenant.h"
#include "core/Window.h"
#include "render/Renderer.h"
#include "scene/FieldSet.h"
#include "scene/GisLayer.h"
#include "scene/GlobeLayer.h"
#include "scene/GulfLayer.h"
#include "scene/SeaLayer.h"
#include "scene/SkyLayer.h"
#include "scene/WaterBankLayer.h"
#include "scene/TideLayer.h"
#include "compose/Compositor.h"
#include "compose/Exchange.h"
#include "compose/SurfaceFrame.h"
#include "compose/GisStencil.h"
#include "compose/Sources.h"
#include "compose/VectorPack.h"
#include "compose/WaterAtlas.h"
#include "scene/VesselLayer.h"
#include "core/TileProviders.h"
#include "core/SceneConfig.h"
#include "scene/WaterComponent.h"
#include "scene/effects/SlicePlane.h"   // M12 step 5e: the cutaway plane as an effect node
#include "sim/BathyModel.h"
#include "sim/GlobeModel.h"
#include "sim/CurrentModel.h"
#include "sim/SeaState.h"
#include "sim/SweSolver.h"
#include "sim/TideModel.h"
#include "app/Options.h"
#include "app/Scene.h"

#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace ga::app {

struct Assembly {
    // ---- main()'s locals, in main()'s order (712679a lines 264..709, then the two statics).
    TideModel model;
    Window window;   // Create()d only when not headless
    Gpu gpu;
    // THE SHUTDOWN TRAIL (core/ExitTrail.h): each mark is declared right after the member it
    // names, so it says so right before that member's destructor runs.
    ExitMark exitGpu{"assembly: ~Gpu -- the device, its queue and what it still owns"};
    // M12 step 4a: THE SHIPPED SURFACE (compose/SurfaceFrame.h) -- the planet radius, the
    // tangent frame's rows, the five lattices, the tenants' ids and page slices, the stencil
    // flag -- built once in Assemble() (SurfaceFrame::Merrimack) and read by both fills.
    // Declared BEFORE the renderer because a layer the renderer owns (the globe) holds a
    // pointer into it until renderer.Shutdown() joins its worker -- the lifetime law above --
    // and nothing else keeps a reference: a Tenant copies its lattices, the frame loop dies
    // first, the tools take it by reference for the length of a call.
    SurfaceFrame surface;
    GpuTexture surfaceDirectory;   // HIERARCHY 4.17 commit 4: the blocks' directory (Walk.hlsli)
    RendererDesc rd;
    Renderer renderer;
    ExitMark exitRenderer{"assembly: ~Renderer -- the layers it owns and their atlases' heaps"};
    FieldSet fields;
    std::unique_ptr<SkyLayer> skyOwned;   // handed to the renderer; `sky` keeps the handle
    SkyLayer* sky = nullptr;
    std::unique_ptr<TideLayer> tideOwned;
    TideLayer* tide = nullptr;
    SeaState seaState;
    SeaLayer* sea = nullptr;
    CurrentModel currents;
    bool haveCurrents = false;
    float datumOff = 0.0f;
    bool marsMode = false;
    GlobeModel globeModel;
    GlobeModel marsModel;
    bool globeDataOk = false;
    GlobeModel* activeGlobe = nullptr;   // main's `GlobeModel&`: marsModel when ready, else globeModel
    BathyModel bathyRaw, bathyCapeAnn, bathyBoston;
    bool haveBathyRaw = false;
    Compositor compositor;
    double navdAboveMsl = 0.0;
    std::string datumFrom = "none";
    bool haveMslLink = false;
    double mslToNavd = 0.0;
    std::optional<EquirectHeightSource> srcEtopo;   // emplaced once globeModel is loaded: the
    std::optional<EquirectHeightSource> srcMola;    // grid dimensions are constructor arguments
    std::unique_ptr<WindowHeightSource> srcNe15;
    std::unique_ptr<CudemHeightSource> srcCudem, srcCudemCA, srcCudemBos;
    EditsHeightSource srcEdits;   // M6w: the hand edits ARE a stack layer now -- their
                                  // cache identity is the geojson content, so an operator
                                  // edit repaints exactly the touched tiles at every rung
    int hgtCh = -1;
    WaterAtlas waterAtlas;
    BathyModel bathy;
    // THE SOLVERS' GRIDS (HIERARCHY 4.17): the survey windows drawn in to where their sources
    // paint at full weight (BathyModel::DrawFrom). `bathy` stays the survey's whole window for
    // what draws or stands on the ground (the foundation sink, the camera's clamp);
    // every reader of a solver's fields reads it by these.
    BathyModel bathySwe, bathyBostonSwe;
    SweSolver swe;
    double riverQ = 70.0;
    GulfLayer* gulf = nullptr;
    WaterSceneConfig waterScene;
    long long waterSceneMtime = 0;
    const char* kScenePath = "data/wave_scene.json";
    WaterSceneWatch sceneWatch;
    // M12 step 5c: THE WATER SCENE'S ONE APPLY (scene/WaterComponent.h). Declared beside the
    // config it writes and the watch it polls, and BEFORE the layers it fans out to, because it
    // holds nothing but observers: it owns no GPU object, joins no thread and frees nothing, so
    // its position in the lifetime order is about reading well, not about destruction. The
    // session Configures it once the layers exist and calls Apply at load; the frame loop calls
    // Reload, which calls the SAME Apply.
    scene::WaterComponent water;
    // M12 step 5e: THE EFFECT NODE (scene/effects/SlicePlane.h) -- the cutaway plane owns the
    // globe's sliceOn/sliceD and registers its AST edge; declared beside the water component
    // for the same reason (observers only, no GPU object).
    scene::SlicePlane slice;
    WaterBankLayer* waterBank = nullptr;
    WaterBankLayer* waterBankB = nullptr;   // M10: the outer level's rings (set B)
    GlobeLayer* globe = nullptr;
    VesselLayer* vesselLayer = nullptr;   // M9bq: the hulls, drawn from their specs
    double planetR = 0.0;
    ResidencyManager resMgr;
    ExitMark exitResMgr{"assembly: ~ResidencyManager -- Shutdown again (waits for any load still "
                        "inside it), then the tenants' reserved resources and the pool's heaps"};
    MarsBinProvider marsDiff, marsNorm;
    GoogleTileProvider googleTiles;
    // ---- M6i: THE LAYER COMPOSITOR's color side (the height side moved above the
    // solver, M6w). Sources register their schemas; channels stack them in order;
    // realizations paint composed quadtrees ONCE and cache every 64KB tile.
    // Made in Assemble() with the scene's finest zoom (streaming.googleZoom): the zoom is part of
    // the source's identity, so it is fixed when the source is made.
    std::optional<GoogleColorSource> srcGoogle;
    BedSynthSource srcBed;          // M7d: the bed classifier -- the first synthesis
    SeafloorReliefSource srcRelief; // M9av: the seafloor's appearance from the ingested bathymetry
                                    // node; its program is data/bed/bed_rules.json
    AerialOrthoSource srcAerial;    // M6l: MassGIS 15 cm orthos (loads if harvested)
    AerialOrthoSource srcOverlay;   // M6o: user GeoTIFF overlays -- ALPHA IS FIBER: a
                                    // mostly-transparent highlights plane bleeds through
                                    // the composed quadtree pixel by pixel
    GisStencil gisStencil;   // survey vectors + mask realizations (GSHHG/WDBII)
    // M9ak: the SAME survey, as rings rather than as a parity fill -- the compositor's
    // land/sea gate. Neither .raw mask is opened by this one.
    GisVectorMask gisMask;
    GisMaskSource srcGisMask{&gisMask};
    VectorPack vectors;      // M6p: lossless vector layers, LOD by wedge importance
    GisLayer* gisLayer = nullptr;

    Exchange exchange;       // M6j: the plugin bus -- named GA buffer channels
    // M12 step 4a: the z14 window origin and the z17 detail origin are `surface`'s
    // (SurfaceFrame::Merrimack writes them down, once); the ids below are the tenants'.
    int colorCubeT = -1, winTenant = -1, hgtTenant = -1, hgtWinTenant = -1;
    int maskTenant = -1;   // M9ay: the survey mask pages (gis.landsea's tree)
    int detTenant = -1;   // M7f: z17 detail color window
    int colCh = -1;   // color channel id (hgtCh registered above the solver, M6w)
    // M9am: the megatexture graph and its on-disk tile cache. Declared HERE, beside the
    // tenants that hold providers into them, so they cannot die first.
    std::vector<std::shared_ptr<DomainSource>> megaKeep;
    std::unique_ptr<TileTree> megaTree;
    // M9as: THE HEIGHT GRAPH AND ITS DISK TREE. BuildHeightStack's six layers, each its own
    // sparse tree on the NVMe (FloatW: value + coverage), composed LayeredOver into ONE root
    // whose tiles are the R16F pages the height tenant reads. Inputs are ordered by
    // fidelity -- coarsest first, so the deepest tree paints over everything it intersects
    // (the user's rule) -- and the order is printed, not assumed.
    std::shared_ptr<DomainSource> heightRoot;
    std::unique_ptr<TileTree> heightTree;
    // M9ba: THE SWELL EXPOSURE AS A TREE NODE. A DomainSource (the LOS march over the height
    // stack), a one-input compose root so the tree materializes R16F, the tree behind a
    // holder so a bucket roll swaps it under the tenant's provider, and the tenant itself.
    std::shared_ptr<ExposureSource> exposureSrc;
    std::shared_ptr<DomainSource> exposureRoot;
    // shared_ptr, swapped ATOMICALLY: loader threads are inside the old tree's provider
    // when a bucket rolls; they hold their own reference until their paint returns.
    std::shared_ptr<std::shared_ptr<TileTree>> exposureTree;
    int exposureT = -1;
    ExitMark exitTrees{"assembly: the tile trees -- the exposure holder, the height tree, the "
                       "megatexture tree (a load still painting holds only its own reference)"};
    // M12 step 3e: THE FOUR DECLARATIONS (hal/Tenant.h) -- the height, exposure, colour and
    // survey tenants as TenantDescs, each Sparse()'d into resMgr and Bind()'d to the tree that
    // feeds it. Declared after the trees they bind (their providers and the exposure holder
    // point into them) and before the indices; a Tenant is a handle on state the manager's own
    // dispatcher shares, so destroying one here releases nothing the manager still reads. The
    // int ids above keep the values Tenant::Id() gave them, so no consumer changed. (The wave's
    // lives in the frame loop beside its tree.)
    hal::Tenant heightTenant, exposureTenant, colorTenant, landseaTenant;
    // THE SWELL SHADOW A HULL READS (the water match, step 3): the exposure page's texels at the
    // kernel's floor, evaluated from the node on the CPU (compose/ExposurePage) -- the numbers the
    // page holds, without the page's residency timing.
    std::unique_ptr<ExposurePage> exposureShadow;
    // THE BED THE WATER KERNELS READ, for a hull's depth laws (compose/HeightPage): the height page's
    // finest texels, evaluated from the stack where the painter evaluates it.
    std::unique_ptr<HeightPage> heightBed;
    // M9ae: the composed cache's index (one entry per TILE), and M9ag: the NVMe -> GPU reader.
    // Both were function-local statics inside main()'s residency block; as the last members
    // they now destruct first, before gpu (the statics used to outlive the device).
    TileIndex idxColorCube, idxColorWin, idxColorDet, idxHeightCube;
    TileStream tileStream;
    // The last member dies first: the trail's first line inside ~Assembly.
    ExitMark exitStream{"assembly destructs, last member first: ~TileStream -- the DirectStorage "
                        "queue, its staging buffer and file handles (nothing drains its reads)"};

    Assembly() = default;
    Assembly(const Assembly&) = delete;
    Assembly& operator=(const Assembly&) = delete;
    Assembly(Assembly&&) = delete;
    Assembly& operator=(Assembly&&) = delete;
};

// main()'s assembly span (Assembly.cpp). M12 step 5d: every value that is scene state comes from
// `S`, the resolved document (app/Scene.h) -- the data files, the mode and planet, the water, the
// sea state, the streaming switches, the capture size, and the `layers` list, which is the
// registration order and the standing draw declaration. `opt` is read for the pure instruments
// and the one-shot tools' own arguments, and for nothing else. Returns the built scene, or
// nullptr when the span exited the process early: `exitCode` then carries the code main returns.
std::unique_ptr<Assembly> Assemble(const Options& opt, const Scene& S, int& exitCode);

}  // namespace ga::app
