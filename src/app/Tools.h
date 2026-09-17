// ================================================================================================
//  Tools - the engine's one-shot commands.
//
//  Every mode that runs once and exits (or runs once and falls through to the normal boot)
//  used to be an `if (opt.flag)` block in main(). M12 step 1b moves each of them here VERBATIM:
//  the function body is the block's body, its parameters are exactly the main() locals the
//  block touched (const Options& first, then the rest in main's declaration order), and the
//  call site keeps its place in the boot sequence, its condition and its return. Nothing about
//  behaviour, log text or exit codes changed; the gate diffs the logs.
//
//  Step 5 puts these behind a Tool registry (`--tool name[:args]`); until then main() calls
//  them by name. --swe-cycle is a template (SweCycle.h) because the solver's boundary clocks
//  are main()'s lambdas and SweSolver::Spinup takes them as template parameters.
// ================================================================================================
#pragma once

#include "app/Options.h"

#include <functional>
#include <memory>
#include <optional>
#include <vector>

namespace ga {
class BathyModel;
class Camera;
class Compositor;
class ExposureSource;
class GisVectorMask;
class GlobeLayer;
class GlobeModel;
class Gpu;
class Renderer;
class ResidencyManager;
class SeaLayer;
class SeaState;
class SweSolver;
class TideModel;
class TileTree;
class WaterAtlas;
class WaterBankLayer;
class VesselLayer;
class WaveField;
class WeatherManager;
struct SurfaceFrame;
struct WaterSceneConfig;
namespace scene {
class Entity;
}
}  // namespace ga

namespace ga::app::tools {

// ---- Before the scene exists (no device, or a 64x64 device of the tool's own).

// --pack-tiles: pack the composed cache into per-realization archives; exit 0. (M12 step
// 4a: the realizations are the surface's lattices; no scene exists, so main() declares it.)
int RunPackTiles(const Options& opt, const SurfaceFrame& surface);
// --load-field PATH: a file through the loader plugin into a sparse bank; report; exit.
int RunLoadField(const Options& opt);
// --selftest: the twelve gates; exit 0 (pass) / 1 (fail).
int RunSelfTest(const Options& opt);

// ---- During assembly (the call keeps its exact place in the boot sequence).

// --water-map / --bathy-map: the reprojection proof (Lambert conformal sheet); exit 0.
int RunWaterMap(const Options& opt, Gpu& gpu, const GlobeModel& globeModel, Compositor& compositor,
                int hgtCh, const WaterAtlas& waterAtlas);
// --gis-dump PATH: the survey gate over its box as a PGM; std::exit(0) from inside.
void RunGisDump(const Options& opt, const GisVectorMask& gisMask);
// --tree-audit N (--pack-trees, --warm-trees): compare / pack / warm the tile trees; exit 0.
int RunTreeAudit(const Options& opt, Compositor& compositor, int hgtCh, ResidencyManager& resMgr,
                 int colCh, const std::unique_ptr<TileTree>& megaTree,
                 const std::unique_ptr<TileTree>& heightTree, const SurfaceFrame& surface);
// --fidelity-map PATH: the heterogeneity sheet. Falls through (no early return, as before).
void RunFidelityMap(const Options& opt, Compositor& compositor);
// --ocean-probe lat,lon: the weather manager's verification harness. The exit code when
// "lat,lon" parsed; nullopt when it did not, and the run goes on exactly as it always has.
std::optional<int> RunOceanProbe(const Options& opt, const TideModel& model, Gpu& gpu,
                                 Renderer& renderer, SweSolver& swe, ResidencyManager& resMgr,
                                 double simUnix, const std::function<double(double)>& oceanAt,
                                 const std::function<double(double)>& southAt,
                                 const std::function<double(double)>& westAt,
                                 const std::function<double(double)>& westQAt,
                                 WeatherManager& weather);
// --swe-cycle N: RunSweCycleMode, a template -- see Tools/SweCycle.h (included below).
// --swe-uv PATH: the solved current field after spin-up, as pictures. Falls through.
void RunSweUv(const Options& opt, Gpu& gpu, const BathyModel& bathy, SweSolver& swe);
// --export SPEC: a composed channel out through the manager; exit with the export's code.
int RunExport(const Options& opt, Gpu& gpu, Compositor& compositor, int hgtCh,
              ResidencyManager& resMgr, int colCh, const SurfaceFrame& surface);
// --warm-inlet: pre-cache the composed pyramids. Falls through into the frame loop.
void RunWarmInlet(const Options& opt, Gpu& gpu, const Compositor& compositor,
                  ResidencyManager& resMgr, int winTenant, int hgtTenant, int hgtWinTenant);

// ---- After the frame loop (the scene is live; the run tears down normally afterwards).

// --dump-water-state: the inlet box's fields as ws_*.f32 + ws_meta.json for proofs/.
void RunDumpWaterState(const Options& opt, Gpu& gpu, SeaLayer* sea, double simUnix,
                       WeatherManager& weather);
// --twin-surface: the keystone gate -- CPU TreeWater against the GPU bank at range rings.
void RunTwinSurface(const Options& opt, Gpu& gpu, const SeaState& seaState, SeaLayer* sea,
                    const WaterSceneConfig& waterScene, WaterBankLayer* waterBank,
                    const Camera& cam, double simUnix, WeatherManager& weather,
                    const std::unique_ptr<WaveField>& waveField);
// --trace lat,lon: one sample through the state diagram, eleven steps to the GPU texel.
void RunTrace(const Options& opt, Gpu& gpu, SeaLayer* sea, const Compositor& compositor, int hgtCh,
              const WaterAtlas& waterAtlas, WaterBankLayer* waterBank, GlobeLayer* globe,
              const ResidencyManager& resMgr, double winOrgX, double winOrgY, int hgtTenant,
              int hgtWinTenant, double simUnix, WeatherManager& weather);
// --sea-verify: rendered Hs from the displacement textures against the model's.
void RunSeaVerify(const Options& opt, Gpu& gpu, SeaLayer* sea);

// ---- Inside the frame loop (an instrument: it reads back and waits).

// --water-probe N: the DRAWN sea (the scene depth lifted to the planet in doubles) against the
// water each hull reads, with the bank between them, binned by range from the hull.
void RunWaterProbe(Gpu& gpu, Renderer& renderer, const Camera& cam, double planetR,
                   const std::vector<std::unique_ptr<scene::Entity>>& entities,
                   WaterBankLayer* waterBank, const VesselLayer* vessels,
                   const WaterAtlas* atlas, const ExposureSource* exposure,
                   const WaveField* waveField, const SeaLayer* sea, const SeaState* seaState,
                   const SurfaceFrame& surface, double oceanNow, double simUnix,
                   uint32_t recFrame);

}  // namespace ga::app::tools

#include "app/Tools/SweCycle.h"   // RunSweCycle + RunSweCycleMode: templates over the clocks
