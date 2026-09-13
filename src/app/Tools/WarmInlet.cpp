// WarmInlet - --warm-inlet.
// Moved verbatim from main.cpp (M12 step 1b); declared in app/Tools.h.
// Falls through, as before: the pre-warm is a statement in the boot sequence, not an exit.
#include "app/Tools.h"

#include "compose/Compositor.h"
#include "core/Common.h"
#include "hal/Context.h"
#include "hal/Gpu.h"
#include "hal/Residency.h"

#include <chrono>
#include <cstdint>
#include <thread>

namespace ga::app::tools {

// M6f/M6i: pre-warm the composed pyramids -- the Merrimack window coarse-to-z14 in
// rings, plus the planet-wide height cube at a working mip (height paints are LOCAL:
// no network, just source reads). Everything lands in the composed forever-cache:
// warming is a once-per-machine cost (re-runs drain instantly from disk).
void RunWarmInlet(const Options& opt, Gpu& gpu, const Compositor& compositor,
                  ResidencyManager& resMgr, int winTenant, int hgtTenant, int hgtWinTenant) {
    struct WarmRing {
        float a, b;
        uint32_t mip;
    };
    // Mip 2 covers the WHOLE window (one zoom level = one color grading across the
    // view -- the patchwork of per-zoom gradings was half of the "uneven shading"
    // report); deeper rings tighten on the inlet.
    const WarmRing rings[] = {
        {0.00f, 1.00f, 3}, {0.00f, 1.00f, 2}, {0.38f, 0.62f, 1}, {0.44f, 0.56f, 0}};
    if (winTenant >= 0) {
        for (const auto& w : rings) {
            resMgr.Want(winTenant, 0, w.mip, w.a, w.a, w.b, w.b);
        }
    }
    if (hgtWinTenant >= 0) {
        // Height paints are pure local math: warm the WHOLE window at mip 2 (~32 MB)
        // so land/sea classification is never a coarse-mip smear anywhere in view.
        const uint32_t hwf = (hgtWinTenant == hgtTenant) ? 6u : 0u;   // M9aq slice
        resMgr.Want(hgtWinTenant, hwf, 2, 0, 0, 1, 1);
        for (const auto& w : rings) {
            resMgr.Want(hgtWinTenant, hwf, w.mip, w.a, w.a, w.b, w.b);
        }
    }
    if (hgtTenant >= 0) {
        for (uint32_t f = 0; f < 6; ++f) resMgr.Want(hgtTenant, f, 4, 0, 0, 1, 1);
    }
    Log("[warm] pre-caching composed pyramids (%u fetch budget; composed tiles land "
        "in cache/composed forever)",
        opt.tileBudget);
    for (int it = 0; it < 12000 && resMgr.PendingCount() > 0; ++it) {
        // M12 step 3f: the residency turn records through an Upload context (the 3b pattern);
        // the manager still takes the raw list, named as the escape hatch it is.
        hal::CommandContext up(gpu, gpu.BeginUpload(), hal::Owner::Upload);
        resMgr.ProcessQueues(gpu, up.Native());
        gpu.EndUpload();
        std::this_thread::sleep_for(std::chrono::milliseconds(20));   // loads need time
        if (it % 150 == 0) {
            Log("[warm] pending %u, fetched %u, painted %u (+%u cached)",
                resMgr.PendingCount(), resMgr.fetchesThisRun,
                compositor.painted.load(), compositor.cacheHits.load());
        }
    }
    Log("[warm] done: %u fetches, %u tiles painted, %u from composed cache, %s",
        resMgr.fetchesThisRun, compositor.painted.load(), compositor.cacheHits.load(),
        resMgr.stats.c_str());
}

}  // namespace ga::app::tools
