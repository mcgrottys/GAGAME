// ================================================================================================
//  gagame - M0 + M1.
//
//  Modes:
//    (default)     windowed viewer. WASD/QE fly, right-drag look, wheel speed, R reload shaders,
//                  F5 save the camera to data/views.json (relaunch into it with --view NAME).
//                  Time controls: SPACE pause, UP/DOWN time-scale x10, LEFT/RIGHT nudge -/+ 1 h
//                  (SHIFT: 1 day), HOME or N back to now, [ ] halve/double the plot window.
//    --selftest    M0 gate: run the reserved-resource null-tile test suite headless and exit
//                  with 0 (pass) / 1 (fail). See src/core/TileAtlas.cpp.
//    --headless    no window; render --frames frames and write --dump to a PNG. The verification
//                  path: a renderer change is provable from a shell.
//
//  The window title is the HUD: sim clock (UTC), time scale, focus-station tide, fit RMS.
// ================================================================================================
#include "core/BuildInfo.h"
#include "core/CrashTrace.h"
#include "core/ExitTrail.h"
#include "core/ThreadAudit.h"
#include "core/ThreadManager.h"
#include "app/Assembly.h"
#include "app/FrameLoop.h"
#include "app/Options.h"
#include "app/Scene.h"
#include "app/Tools.h"
#include "compose/SurfaceFrame.h"
#include "compose/TileTree.h"
#include "scene/SceneBuilder.h"
#include "sim/GlobeModel.h"
#include "sim/WaveField.h"

#include <exception>
#include <memory>
#include <string>

using namespace ga;
using namespace ga::app;

int main(int argc, char** argv) {
    ga::InstallCrashTrace();   // M7v: symbolized stacks on any crash, headless
    // Before any pool exists: this is the thread that owns the frame, the device, the queue and
    // the command list, and GA_MAIN_THREAD_ONLY() is measured against it.
    ga::threadaudit::SetMainThread();
    try {
        const Options opt = ParseArgs(argc, argv);
        // M12 step 5a: the scene's front door. --print-scene resolves the flags' scene form
        // (Options::ToSets over scenes/*.json, app/Options.cpp) and prints it -- before the
        // pool, the boot line and any device work; a refusal is exit 2 with the reason.
        if (opt.printScene) return PrintScene(opt, argc, argv);
        if (opt.threadAudit) ga::threadaudit::Enable();
        // The process pool, before anything can submit to it. Everything that used to spawn its
        // own threads is a client of this (core/ThreadManager.h).
        ga::Threads().Init();
        ga::Threads().SetInline(opt.jobsInline);
        {
            // The first line of every log: which binary, which flags. A baseline still or a
            // bench log is otherwise unmatchable to the recipe that made it (out/baseline/*.log
            // named neither; the memory note carried the command instead).
            std::string args;
            for (int i = 1; i < argc; ++i) {
                if (i > 1) args += ' ';
                args += argv[i];
            }
            Log("[boot] gagame rev %s | argv: %s", BuildGitRev(), args.c_str());
        }

        // ---- M12 step 5d: THE SCENE, RESOLVED ONCE (app/Scene.h). The flags reach it through
        // the same shim --print-scene prints (Options::ToSets -> SceneBuilder: defaults < the
        // base file < each include overlay < every --set, in order), so `gagame <flags>` and
        // `gagame <the scene file those flags print>` resolve to ONE document -- and from here
        // down the boot reads THAT, not the flags. A refusal is exit 2, naming the path.
        scene::SceneBuilder builder;
        SceneArgs sargs;
        std::string why;
        if (!BuildScene(opt, builder, sargs, &why)) {
            Log("FATAL: [scene] refused: %s", why.c_str());
            return 2;
        }
        Scene S;
        if (!ReadScene(builder.Resolved(), S, &why)) {
            Log("FATAL: [scene] refused: %s", why.c_str());
            return 2;
        }
        S.path = sargs.scene;
        // Every tree folder this run ensures is stamped with the scene that used it
        // (compose/TileTree.h, StampLive): named here, before anything can build a tree.
        tree_detail::SetLiveScene(S.scene.name, S.path);
        {
            std::string order;
            for (const std::string& n : S.LayerOrder()) order += (order.empty() ? "" : " ") + n;
            Log("[scene] %s -> '%s': mode %s, planet %s, start view '%s'; layers: %s",
                S.path.c_str(), S.scene.name.c_str(),
                S.scene.mode == Scene::kWorld ? "world"
                : S.scene.mode == Scene::kGulf ? "gulf" : "chart",
                S.scene.planet.c_str(), S.scene.view.c_str(), order.c_str());
            for (const std::string& r : sargs.raw) {
                Log("[scene] raw instrument, not scene data: %s", r.c_str());
            }
        }
        // A one-shot mode is a TOOL, named in the document, whether the flag or --tool said it
        // (Options::ToSets writes both into `tools`). Its arguments stay the flag's.
        Options topt = opt;
        ToolArgs(S, topt);

        // M12 step 4a: a disk job with no scene; the shipped surface's declaration (its
        // realization tags) is all the packer reads.
        if (S.Tool("pack-tiles")) {
            return tools::RunPackTiles(topt, SurfaceFrame::About(GlobeModel::kR, false,
                                                                 S.place.anchor[1], S.place.anchor[0]));
        }
        // The prune tool: the tile trees' folders by last use, before any device and before any
        // tree exists -- it builds none, so it stamps none. Its keys are the scene's prune.*.
        if (S.Tool("tree-prune")) return tools::RunTreePrune(S.prune);
        // The building pyramid: the stack's every cell, boxed, on the CPU (docs/BUILDING_LOD.md).
        if (const SceneTool* t = S.Tool("building-lod")) return tools::RunBuildingLod(S, t->args);
        // One block of --selftest that needs no device: a raster by file and its own level; with
        // `:real`, the real height files against the harvester's grids (slice 3, part C).
        if (const SceneTool* t = S.Tool("rastertest")) {
            return (t->args == "real" ? ga::RunRealHeightsCheck() : ga::RunRasterFileSelfTest()) ? 0 : 1;
        }

        // ---- M0 + M4: the self-test path needs a device and the shader compiler, nothing else.
        if (!topt.loadField.empty()) return tools::RunLoadField(topt);

        if (S.Tool("selftest")) return tools::RunSelfTest(topt);
        // F25: a wave cache entry's own planes solved again by this binary, compared byte for byte.
        if (const SceneTool* t = S.Tool("wave-recheck")) return WaveField::RecheckCache(t->args) ? 0 : 1;
        if (const SceneTool* t = S.Tool("wave-converge")) return WaveField::ConvergeCache(t->args) ? 0 : 1;   // F27

        // ---- M12 step 1c: the scene, built as one object (app/Assembly.h). Its members
        // are the locals that used to stand here, under the same names, so what follows is
        // unchanged; the Assembly is declared before every session local and outlives them.
        int exitCode = 0;
        auto A = Assemble(topt, S, exitCode);
        if (!A) return exitCode;
        // ---- M12 step 1d: the session and the frame loop as one object (app/FrameLoop.h),
        // declared AFTER the Assembly so it destructs first, as the session locals did before
        // the assembly locals. Run() is main()'s remaining span: Session(), the loop, Finish().
        auto loop = std::make_unique<FrameLoop>(topt, S, *A);
        const int rc = loop->Run();
        // THE SHUTDOWN TRAIL (core/ExitTrail.h): the frame loop and then the assembly, the order
        // their scope always ended them in, each teardown marked from inside by its members.
        ExitStep("main: the frame loop destructs");
        loop.reset();
        ExitStep("main: the assembly destructs");
        A.reset();
        Log("[exit] main returns %d; what follows is the C runtime's own teardown (the static "
            "job pool, the log)", rc);
        return rc;
    } catch (const std::exception& e) {
        Log("FATAL: %s", e.what());
        return 1;
    }
}
