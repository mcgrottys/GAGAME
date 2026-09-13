// SelfTest - --selftest.
// Moved verbatim from main.cpp (M12 step 1b); declared in app/Tools.h.
#include "app/Tools.h"

#include "compose/Compositor.h"
#include "compose/WaterAtlas.h"
#include "core/DxTest.h"
#include "core/GaAst.h"
#include "core/Gpu.h"
#include "core/Pga.h"
#include "core/Shader.h"
#include "core/ThreadAudit.h"
#include "core/TileAtlas.h"
#include "sim/RigidBody.h"
#include "sim/SimClock.h"
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
    ok &= RunComposeSelfTest();   // pure CPU: the layer compositor's contracts
    ok &= RunWaterSelfTest();     // pure CPU: the water atlas' datum/epoch/field gates
    ok &= RunTileSelfTest(gpu, sc, opt.shaderDir);
    ok &= RunAtlasSelfTest(gpu, sc, opt.shaderDir);
    ok &= RunThreadSelfTest();    // the thread instrument's own gate: it must SEE a race
    ok &= RunSimClockSelfTest();  // the scene clock: whole quanta, framing-independent
    ok &= RunRigidBodySelfTest();  // M9bq: the body with momentum -- L, T, moment arms
    ok &= RunVesselSelfTest();     // M9bq: the factory + the element laws
    gpu.Shutdown();
    return ok ? 0 : 1;
}

}  // namespace ga::app::tools
