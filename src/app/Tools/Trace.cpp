// Trace - --trace.
// Moved verbatim from main.cpp (M12 step 1b); declared in app/Tools.h.
#include "app/Tools.h"

#include "compose/Compositor.h"
#include "compose/WaterAtlas.h"
#include "core/Common.h"
#include "hal/Gpu.h"
#include "hal/Residency.h"
#include "scene/GlobeLayer.h"
#include "scene/SeaLayer.h"
#include "scene/WaterBankLayer.h"
#include "sim/BathyModel.h"
#include "sim/WaterTerms.h"
#include "sim/WaveScale.h"
#include "sim/WeatherManager.h"

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace ga::app::tools {

void RunTrace(const Options& opt, Gpu& gpu, SeaLayer* sea, const Compositor& compositor,
              int hgtCh, const WaterAtlas& waterAtlas, WaterBankLayer* waterBank,
              GlobeLayer* globe, const ResidencyManager& resMgr, double simUnix,
              WeatherManager& weather, const Space::Anchor& chart) {
    weather.RefreshMirrorsTo(gpu, simUnix);   // a no-op after the export above
    // PHASE C4: no point given = the scene's place.anchor.
    const bool given = !std::isnan(opt.traceLat) && !std::isnan(opt.traceLon);
    const double tlat = given ? opt.traceLat : chart.latDeg, tlon = given ? opt.traceLon : chart.lonDeg;
    double wx = 0.0, wz = 0.0;
    chart.FlatOf(tlat, tlon, wx, wz);
    Log("[trace] ==== ONE SAMPLE THROUGH THE STATE DIAGRAM ====");
    Log("[trace] input       lat %.5f lon %.5f  t %.0f unix", tlat, tlon,
        simUnix);
    Log("[trace] 1 frame     latlon.deg -> world.m: x %+.1f z %+.1f  "
        "(the tangent plane about %.5f,%.5f, exact; +x=east +z=north)",
        wx, wz, chart.latDeg, chart.lonDeg);
    const WeatherSample wq = weather.Query(tlat, tlon, simUnix, 30.0);
    Log("[trace] 2 bed       compose.stack SampleHeightStack: %+.2f m NAVD "
        "[%s]  (edge compose.stack->water.bank corners)",
        wq.bedNavd, wq.bedSrc);
    Log("[trace] 3 level     water.atlas rotors + swe mirror: %+.2f m NAVD "
        "[%s]", wq.levelNavd, wq.levelSrc);
    if (waterAtlas.Ready()) {
        float elo = 0.0f, ehi = 0.0f;
        waterAtlas.EnvelopeNavd(tlat, tlon, simUnix, &elo, &ehi);
        Log("[trace] 3b envelope water.atlas origin planes: lo %+.2f hi "
            "%+.2f m NAVD (synodic-month min/max; live level must sit "
            "inside; edit floor %+.2f)",
            elo, ehi, globe ? globe->editFloorNavd : 0.0f);
    }
    Log("[trace] 4 current   swe.solver (row0N raster, FLIP into +v=N): "
        "u %+.2f v %+.2f m/s [%s]  (edge swe.solver->water.bank uv)",
        wq.u, wq.v, wq.currentSrc);
    Log("[trace] 5 depth     level - bed = %.2f m  dry %.2f  breaking clamp "
        "0.55*depth = %.2f m", wq.depthM,
        wt::Smoothstep(0.05, 0.65, double(wq.depthM)),
        0.55 * (std::max)(static_cast<double>(wq.depthM), 0.05));
    // The one law the bank's tile corners and the hull's twin read (sim/WaveScale.h).
    static const WaveScale kNone{};
    const WaveScale& scaleT = sea ? sea->Scale() : kNone;
    Log("[trace] 6 sea state Hs %.2f m Tp %.1f s dir %.0f [%s] -> hsScale "
        "%.2f  (the grid's nodes bilinear, %zu source(s) over them, over the reference %.2f m, "
        "clamp 0.15..3)",
        wq.hs, wq.tp, wq.dirDeg, wq.waveSrc, scaleT.At(weather.Globe(), tlat, tlon),
        scaleT.sources.size(), scaleT.hsRef);
    const float expoT = sea->ShadowAtWorld(static_cast<float>(wx),
                                           static_cast<float>(wz));
    Log("[trace] 7 exposure  swell.exposure node (page z14 mips >= 3, no flip, "
        "floor 0.18): %.2f  (edge exposure.node->water.bank exposure)",
        (std::max)(expoT, 0.18f));
    for (int m = 0; m < 3; ++m) {
        const double texel = 4.8 * (1 << m);
        const double lam[3] = {213.0, 26.9, 2.2};
        Log("[trace] 8 fold r%d  texel %.1f m: w(213m) %.2f  w(26.9m) %.2f  "
            "w(2.2m) %.2f  (geometry vs sigma2 split, M6t)",
            m, texel,
            1.0 - std::clamp((texel - lam[0] * 0.12) / (lam[0] * 0.38), 0.0, 1.0),
            1.0 - std::clamp((texel - lam[1] * 0.12) / (lam[1] * 0.38), 0.0, 1.0),
            1.0 - std::clamp((texel - lam[2] * 0.12) / (lam[2] * 0.38), 0.0, 1.0));
    }
    Log("[trace] 9 gpu fibers cascades (patch.wrap, no flip) + churn "
        "(atlas.texel flat, x1.05) + bank write ring texels: GPU-resident; "
        "contracts printed by [gaast] at boot");
    Log("[trace] 10 render   bank -> globe BankSample (no flip) -> two rays "
        "(sandwich -n d n, refraction rotor; gatest-pinned)");
    Log("[trace] ==== cross-check: NOAA tides at 8440452, GoMOFS currents, "
        "GFS-Wave Hs -- the provenance strings above name the rungs ====");
    waterBank->TraceProbe(gpu, wx, wz);
    // Step 11 (the z14 height page's texel against the CPU stack) is deleted with the Mercator
    // pages (PHASE B3).
    // M9ba: the exposure field is the tree folder cache\trees\swell.exposure.*
}

}  // namespace ga::app::tools
