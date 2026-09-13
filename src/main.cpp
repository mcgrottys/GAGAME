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
#include "core/ThreadAudit.h"
#include "core/ThreadManager.h"
#include "app/Assembly.h"
#include "app/FrameLoop.h"
#include "app/Options.h"
#include "app/Tools.h"

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

        if (opt.packTiles) return tools::RunPackTiles(opt);

        // ---- M0 + M4: the self-test path needs a device and the shader compiler, nothing else.
        if (!opt.loadField.empty()) return tools::RunLoadField(opt);

        if (opt.selftest) return tools::RunSelfTest(opt);

        // ---- M12 step 1c: the scene, built as one object (app/Assembly.h). Its members
        // are the locals that used to stand here, under the same names, so what follows is
        // unchanged; the Assembly is declared before every session local and outlives them.
        int exitCode = 0;
        auto A = Assemble(opt, exitCode);
        if (!A) return exitCode;
        // ---- M12 step 1d: the session and the frame loop as one object (app/FrameLoop.h),
        // declared AFTER the Assembly so it destructs first, as the session locals did before
        // the assembly locals. Run() is main()'s remaining span: Session(), the loop, Finish().
        auto loop = std::make_unique<FrameLoop>(opt, *A);
        return loop->Run();
    } catch (const std::exception& e) {
        Log("FATAL: %s", e.what());
        return 1;
    }
}
