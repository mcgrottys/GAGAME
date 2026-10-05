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
#include "core/Space.h"
#include "sim/SweSolver.h"

#include <functional>
#include <memory>
#include <optional>
#include <string>
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
struct PruneSection;
struct WindowLink;
}
}  // namespace ga

namespace ga::app::tools {

// ---- Before the scene exists (no device, or a 64x64 device of the tool's own).

// --pack-tiles: pack the composed cache into per-realization archives; exit 0. (M12 step
// 4a: the realizations are the surface's lattices; no scene exists, so main() declares it.)
int RunPackTiles(const Options& opt, const SurfaceFrame& surface);
// --tree-prune (--tool tree-prune): the tile trees' tag folders listed by last use, and retired
// or purged only when the scene's prune.confirm names the root (compose/TreePrune.h). 0 listed
// or done, 2 refused with nothing changed, 1 stopped at a failed move or delete.
int RunTreePrune(const scene::PruneSection& prune);
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
// PHASE C1: THE TOOLS' GRID. The solver's cells are its own chart (SweDomain); the tools' points still
// stand in world.flat (their keys are C4's), so a tool takes a point to its place through the flat
// chart and then to the solver's cell, and back. The one place the tools meet both charts.
struct SweToolGrid {
    const SweDomain* dom = nullptr;
    Space::Anchor flat;
    bool CellOfFlat(double x, double z, double& tx, double& ty) const {
        double la = 0.0, lo = 0.0;
        flat.LatLonOf(x, z, la, lo);
        return dom->CellOf(la, lo, tx, ty);
    }
    void FlatOfCell(double tx, double ty, double& x, double& z) const {
        double la = 0.0, lo = 0.0;
        dom->LatLonOf(tx, ty, la, lo);
        flat.FlatOf(la, lo, x, z);
    }
    // Points of world.flat (x, z pairs) turned into cells in place (-1 where the chart has none).
    void CellsOfFlat(float* xz, int n) const {
        for (int i = 0; i < n; ++i) {
            double tx = -1.0, ty = -1.0;
            if (!CellOfFlat(xz[2 * i], xz[2 * i + 1], tx, ty)) tx = ty = -1.0;
            xz[2 * i] = float(tx);
            xz[2 * i + 1] = float(ty);
        }
    }
    // The CPU bed at a point of world.flat (the nearest cell's); -9999 off the grid.
    float BedAtFlat(double x, double z) const {
        double tx = 0.0, ty = 0.0;
        if (!CellOfFlat(x, z, tx, ty) || tx < 0.0 || ty < 0.0 || tx >= dom->nx || ty >= dom->ny) {
            return -9999.0f;
        }
        return dom->elev[size_t(ty) * dom->nx + size_t(tx)];
    }
};

// PHASE C1: the tide focus station as a probe of --swe-cycle -- the solver's level at the station's
// own place (by the solver's chart) beside the station's own prediction (TideModel), NAVD m.
struct SweFocusProbe {
    std::string id;
    double lat = 0.0, lon = 0.0;
    std::function<double(double)> pred;
};

// --swe-cycle N: RunSweCycleMode, a template -- see Tools/SweCycle.h (included below).
// --swe-uv PATH: the solved current field after spin-up, as pictures. Falls through.
void RunSweUv(const Options& opt, Gpu& gpu, const SweToolGrid& grid, SweSolver& swe);
// --export SPEC: a composed channel out through the manager; exit with the export's code.
int RunExport(const Options& opt, Gpu& gpu, Compositor& compositor, int hgtCh,
              ResidencyManager& resMgr, int colCh);
// --warm-inlet: pre-cache the height cube. Falls through into the frame loop.
void RunWarmInlet(const Options& opt, Gpu& gpu, const Compositor& compositor,
                  ResidencyManager& resMgr, int hgtTenant);

// ---- After the frame loop (the scene is live; the run tears down normally afterwards).

// --dump-water-state: the inlet box's fields as ws_*.f32 + ws_meta.json for proofs/.
void RunDumpWaterState(const Options& opt, Gpu& gpu, SeaLayer* sea, double simUnix,
                       WeatherManager& weather);
// --twin-surface: the keystone gate -- CPU TreeWater against the GPU bank at range rings.
// `classifierNavd` is the level the surface classifier holds the bed against (the tide plane).
void RunTwinSurface(const Options& opt, Gpu& gpu, const SeaState& seaState, SeaLayer* sea,
                    const WaterSceneConfig& waterScene, WaterBankLayer* waterBank,
                    const Camera& cam, double simUnix, WeatherManager& weather,
                    const std::unique_ptr<WaveField>& waveField, double classifierNavd);
// THE THREE LEVELS THAT MUST AGREE at the camera (the startup transient's measurement): the
// classifier's, the bank's and the CPU's. RunTwinSurface prints it first; --bed-trace prints it
// at its readings. Reads back and waits.
void LogLevelsAtCamera(Gpu& gpu, WaterBankLayer* waterBank, const Camera& cam, double simUnix,
                       WeatherManager& weather, double classifierNavd);
// --trace lat,lon: one sample through the state diagram, eleven steps to the GPU texel.
void RunTrace(const Options& opt, Gpu& gpu, SeaLayer* sea, const Compositor& compositor, int hgtCh,
              const WaterAtlas& waterAtlas, WaterBankLayer* waterBank, GlobeLayer* globe,
              const ResidencyManager& resMgr, double simUnix, WeatherManager& weather);
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
                   uint32_t recFrame, const std::vector<scene::WindowLink>* windows);

// --bed-trace N: THE BED THE SOLVER READS (app/Tools/BedTrace.cpp). The solver's own BedAt read
// back through its own kernel (SweSolver::TraceBed) just before the spin-up, just after it, every
// N frames, and once more at the end of the run with the domain made whole. Each reading logs the
// mips the rule read at, the cells reading exactly 0.0 (the value of a tile that is not there),
// and the cells of the inlet's throat whose bed lies below the tide plane -- whether the inlet
// is open; the end compares every reading with the whole one. `--bed-trace-plant` adds one
// reading with the rule's residency floor forced to the coarsest mip: the instrument must be seen
// to call that bed coarse.
class BedTracer {
public:
    // What the residency manager CLAIMS over the solver's domain: its own map, by mip, and the
    // samples taken (WeatherManager::ClaimedMips). Printed beside what the kernel read.
    using ClaimFn = std::function<uint64_t(uint64_t hist[16])>;
    // The throat's cross-section, found once on the CPU bed, which is geometry here and nothing
    // else (the count reads the trace): the narrowest run of wet-capable cells that crosses the
    // solver's throat point, looked for within kThroatSearchM of it along the channel.
    void Configure(const SweToolGrid& grid, const std::string& dir, ClaimFn claim = {});
    // One reading, logged under `label` and kept for the comparison at the end. `tideNavd` is the
    // plane the throat's cells are held against; `floorMip` the rule's floor (the planted read).
    bool Read(Gpu& gpu, SweSolver& swe, const std::string& label, double tideNavd,
              float floorMip = 0.0f);
    // The reading taken with the domain made whole, and every kept reading against it.
    void CompareWithWhole(const std::string& wholeLabel) const;
    bool Configured() const { return m_nx > 0; }

private:
    static constexpr double kThroatX = 250.0, kThroatZ = 60.0;   // the solver's own probe
    static constexpr double kThroatSearchM = 300.0;
    static constexpr float kWetCapableNavd = 1.2f;   // SweSolver::Init's wet-capable bed
    struct Reading {
        std::string label;
        std::vector<float> bed;
    };
    std::vector<Reading> m_kept;
    std::string m_dir;
    ClaimFn m_claim;
    uint32_t m_nx = 0, m_ny = 0;
    double m_dxM = 0.0, m_dyM = 0.0, m_worldX0 = 0.0, m_worldZ1 = 0.0;
    uint32_t m_throatCol = 0, m_throatRow0 = 0, m_throatRow1 = 0;   // inclusive rows
};

}  // namespace ga::app::tools

#include "app/Tools/SweCycle.h"   // RunSweCycle + RunSweCycleMode: templates over the clocks
