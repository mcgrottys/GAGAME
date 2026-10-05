// SelfTest - --selftest.
// Moved verbatim from main.cpp (M12 step 1b); declared in app/Tools.h.
#include "app/Tools.h"

#include "compose/Compositor.h"
#include "compose/RasterFileSource.h"
#include "compose/TreePrune.h"
#include "compose/TileTree.h"
#include "compose/WaterAtlas.h"
#include "hal/DxTest.h"
#include "core/DayLedger.h"
#include "core/GaAst.h"
#include "core/Lattice.h"
#include "hal/Gpu.h"
#include "hal/ResidencyAudit.h"
#include "core/Pga.h"
#include "hal/Residency.h"
#include "hal/Shader.h"
#include "hal/Tenant.h"
#include "core/Space.h"
#include "core/ThreadAudit.h"
#include "scene/SceneBuilder.h"
#include "hal/TileAtlas.h"
#include "sim/RigidBody.h"
#include "sim/WaveChart.h"
#include "sim/WaveField.h"
#include "sim/SimClock.h"
#include "sim/SweSolver.h"
#include "sim/Vessel.h"

namespace ga::app::tools {

int RunSelfTest(const Options& opt) {
    Gpu gpu;
    gpu.Init(nullptr, 64, 64, opt.debugLayer);
    ShaderCompiler sc;
    sc.Init();
    bool ok = RunPgaSelfTest();   // pure CPU: the motor conventions, pinned first
    ok &= RunDxSelfTest();        // M7v: the DX12 contract gate (CB parity via
                                  // reflection, the sampler law, AST anchors)
    ok &= RunGaSelfTest();        // pure CPU: GA products + the frame/orientation
                                  // ledger as executable contract (M7j)
    ok &= RunDayLedgerSelfTest();   // the day's cap on Google fetches: the engine's ledger, lock
                                    // and date, a stand-in request and clock, out\daytest, no
                                    // network
    ok &= RunSpaceSelfTest();     // M12: the frame calculus -- placements, the fold, the
                                  // Droste link through Space, the lattice against ColorFrame
    ok &= RunFaceWindowSelfTest();   // HIERARCHY step 3: a face-plane window's address, the
                                     // float32 twin of PageTexel against doubles, and a plant
    ok &= hal::RunTenantBindingSelfTest();   // HIERARCHY 4.17 commit 1: a slice bound to one
                                             // aligned block of the pyramid, and three plants
    ok &= RunComposeSelfTest();   // pure CPU: the layer compositor's contracts
    ok &= RunRasterFileSelfTest();   // a raster is a source by being a file: its own GeoTIFFs and
                                     // PNGs under out\rastertest, placement, order, identity, plants
                                     // twins of the same ground (containment), a planted origin caught
    ok &= RunWaterSelfTest();     // pure CPU: the water atlas' datum/epoch/field gates
    ok &= RunWaveFieldSelfTest();   // F8: the eikonal sweep and its gauge, out of the solve
    ok &= RunWaveChartSelfTest();   // M13: the cascade sea's lattice-generated planes -- the
                                    // partition, the variance-preserving blend, the metric
    ok &= RunTileSelfTest(gpu, sc, opt.shaderDir);
    ok &= RunAtlasSelfTest(gpu, sc, opt.shaderDir);
    ok &= RunResidencyAuditSelfTest();   // the residency bytes against the mapped set, on
                                         // constructed tenants through the real byte rule
    ok &= RunThreadSelfTest();    // the thread instrument's own gate: it must SEE a race
    ok &= RunTileTreeSelfTest();  // HIERARCHY 4a: the tree fit for a deep pyramid, on scratch
                                  // roots in out\treetest -- nothing lands in the real cache
    ok &= RunSimClockSelfTest();  // the scene clock: whole quanta, framing-independent
    ok &= RunRigidBodySelfTest();  // M9bq: the body with momentum -- L, T, moment arms
    ok &= RunVesselSelfTest();     // M9bq: the factory + the element laws
    ok &= RunWaterHoldSelfTest();  // a solver's window holds its water: a cut channel is
                                   // reported, a whole one is not, the plant is caught
    ok &= scene::RunSceneSelfTest();   // M12 step 5a: the scene's data structures -- the
                                       // registry template, the property table, the fold with
                                       // override, the placement sugar, the shim
    ok &= RunPruneSelfTest();          // the tree-prune tool's refusals, planted and caught on a
                                       // scratch root under out\prunetest, never the cache
    ok &= RunResidencySelfTest(gpu, sc, opt.shaderDir);   // the floor law: its arithmetic on the
                                                          // CPU, its reads on this GPU's samplers
    gpu.Shutdown();
    return ok ? 0 : 1;
}

}  // namespace ga::app::tools
