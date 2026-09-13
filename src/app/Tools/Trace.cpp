// Trace - --trace.
// Moved verbatim from main.cpp (M12 step 1b); declared in app/Tools.h.
#include "app/Tools.h"

#include "compose/Compositor.h"
#include "compose/WaterAtlas.h"
#include "core/Common.h"
#include "core/Gpu.h"
#include "core/Residency.h"
#include "scene/GlobeLayer.h"
#include "scene/SeaLayer.h"
#include "scene/WaterBankLayer.h"
#include "sim/BathyModel.h"
#include "sim/WeatherManager.h"

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace ga::app::tools {

void RunTrace(const Options& opt, Gpu& gpu, SeaLayer* sea, const Compositor& compositor,
              int hgtCh, const WaterAtlas& waterAtlas, WaterBankLayer* waterBank,
              GlobeLayer* globe, const ResidencyManager& resMgr, double winOrgX,
              double winOrgY, int hgtTenant, int hgtWinTenant, double simUnix,
              WeatherManager& weather) {
    weather.RefreshMirrorsTo(gpu, simUnix);   // a no-op after the export above
    const double tlat = opt.traceLat, tlon = opt.traceLon;
    const double wx = (tlon - BathyModel::kOrgLon) * BathyModel::kMPerLon;
    const double wz = (tlat - BathyModel::kOrgLat) * BathyModel::kMPerLat;
    Log("[trace] ==== ONE SAMPLE THROUGH THE STATE DIAGRAM ====");
    Log("[trace] input       lat %.5f lon %.5f  t %.0f unix", tlat, tlon,
        simUnix);
    Log("[trace] 1 frame     latlon.deg -> world.m: x %+.1f z %+.1f  "
        "(org %.5f,%.5f; mPerLon %.0f mPerLat %.0f; +x=east +z=north)",
        wx, wz, BathyModel::kOrgLat, BathyModel::kOrgLon,
        BathyModel::kMPerLon, BathyModel::kMPerLat);
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
        std::clamp((wq.depthM - 0.05) / 0.6, 0.0, 1.0),
        0.55 * (std::max)(static_cast<double>(wq.depthM), 0.05));
    const double hsRefT = 0.8;
    const double hsScaleT =
        wq.hs > 0.0f ? std::clamp(wq.hs / hsRefT, 0.15, 3.0) : 1.0;
    Log("[trace] 6 sea state Hs %.2f m Tp %.1f s dir %.0f [%s] -> hsScale "
        "%.2f  (Hs/gulfRef %.2f, clamp 0.15..3)",
        wq.hs, wq.tp, wq.dirDeg, wq.waveSrc, hsScaleT, hsRefT);
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
    // Step 11: THE COMPOSED TILES THEMSELVES. Same point, three answers
    // that must agree: the CPU stack (the law), the resident GPU texel of
    // the height window (what the renderer actually reads), and the
    // residency map that says which mip that is. stack -> cache -> GPU,
    // end to end.
    if (hgtWinTenant >= 0 && hgtCh >= 0) {
        const double piT = 3.14159265358979;
        const double n14 = 16384.0 * 256.0;
        const double mxT = (tlon + 180.0) / 360.0 * n14;
        const double myT =
            (0.5 - std::log(std::tan(piT * 0.25 + tlat * piT / 360.0)) /
                       (2.0 * piT)) *
            n14;
        const double uT = (mxT - winOrgX) / 16384.0;
        const double vT = (myT - winOrgY) / 16384.0;
        if (uT > 0.0 && uT < 1.0 && vT > 0.0 && vT < 1.0) {
            const uint32_t hwfT = (hgtWinTenant == hgtTenant) ? 6u : 0u;
            const uint32_t mipT = resMgr.ResidentMipAt(
                hgtWinTenant, hwfT, static_cast<float>(uT),
                static_cast<float>(vT));
            if (mipT <= 7) {
                const uint32_t dimT = 16384u >> mipT;
                uint32_t txT = static_cast<uint32_t>(uT * dimT);
                uint32_t tyT = static_cast<uint32_t>(vT * dimT);
                if (txT >= dimT) txT = dimT - 1;
                if (tyT >= dimT) tyT = dimT - 1;
                uint8_t pxT[16] = {};
                float gpuH = 0.0f;
                if (gpu.ReadbackTexel(resMgr.TextureRes(hgtWinTenant),
                                      hwfT * resMgr.Mips(hgtWinTenant) + mipT,
                                      txT, tyT,
                                      resMgr.TextureState(hgtWinTenant),
                                      pxT)) {
                    const uint16_t h16 =
                        static_cast<uint16_t>(pxT[0] | (pxT[1] << 8));
                    const uint32_t sT = (h16 >> 15) & 1u,
                                   eT = (h16 >> 10) & 31u,
                                   mT2 = h16 & 1023u;
                    gpuH = (eT == 0)
                               ? 0.0f
                               : std::ldexp(1.0f + mT2 / 1024.0f,
                                            static_cast<int>(eT) - 15) *
                                     (sT ? -1.0f : 1.0f);
                }
                // CPU stack at the TEXEL CENTRE, at the texel's own res.
                const double pxC = winOrgX + (txT + 0.5) * (1 << mipT);
                const double pyC = winOrgY + (tyT + 0.5) * (1 << mipT);
                const double lonC = pxC / n14 * 360.0 - 180.0;
                const double latC =
                    std::atan(std::sinh(piT * (1.0 - 2.0 * pyC / n14)));
                const float cpuH = compositor.SampleHeightStack(
                    hgtCh, latC, lonC * piT / 180.0,
                    9.55 * (1 << mipT));
                const float dH = std::abs(gpuH - cpuH);
                const float tol =
                    0.06f + 0.02f * std::abs(cpuH);
                Log("[trace] 11 compose  height.pages z14 mip %u texel "
                    "(%u,%u): GPU %+.2f m vs CPU stack %+.2f m  %s "
                    "(edge compose.stack->height.pages, |d| %.3f tol %.3f)",
                    mipT, txT, tyT, gpuH, cpuH,
                    dH <= tol ? "MATCH" : "MISMATCH", dH, tol);
            } else {
                Log("[trace] 11 compose  height.pages z14: nothing resident "
                    "at this uv yet");
            }
        }
    }
    // M9ba: the exposure field is the tree folder cache\trees\swell.exposure.*
}

}  // namespace ga::app::tools
