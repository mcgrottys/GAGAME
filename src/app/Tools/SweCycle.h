// ================================================================================================
//  SweCycle - --swe-cycle N: the headless tidal-cycle validation run, moved verbatim from
//  main.cpp (M12 step 1b). See app/Tools.h.
//
//  Header-only on purpose. The solver's boundary clocks (oceanAt / southAt / westAt / westQAt)
//  are lambdas that stay in main(), and SweSolver::Spinup / AdvanceTo take them as template
//  parameters; so RunSweCycle (the CSV integrator, unchanged) and RunSweCycleMode (the block
//  that ran under `if (opt.sweCycleH > 0)`) are templates too, and the lambdas bind exactly as
//  they did -- no std::function in between.
// ================================================================================================
#pragma once

#include "app/Options.h"
#include "core/Common.h"
#include "core/Gpu.h"
#include "core/Residency.h"
#include "sim/BathyModel.h"
#include "sim/CurrentModel.h"
#include "sim/SweSolver.h"

#include <cstdio>

namespace ga::app::tools {

// M5c validation harness: integrate a full tidal cycle headless and log probes to CSV --
// ocean sponge (must track the analytic tide), the entrance throat AT the ACT0816 station
// (solved current vs the CO-OPS prediction, the real gate), the Joppa Flats basin (lag +
// attenuation must EMERGE), and the river's standing slope upstream.
template <typename F, typename G, typename H, typename Q>
void RunSweCycle(Gpu& gpu, SweSolver& swe, F oceanAt, G westAt, H southAt, Q westQAt,
                 const CurrentModel* currents, int ctSta, const BathyModel& bathy,
                 double startUnix, double hours) {
    auto tideAt = oceanAt;

    // River probe: the deepest channel cell near x = -5000 m.
    const int nx = bathy.Nx(), ny = bathy.Ny();
    const int ix = static_cast<int>((-5000.0f - bathy.WorldX0()) / bathy.WorldSizeX() * nx);
    int iyBest = ny / 2;
    float eBest = 1e9f;
    for (int iy = 0; iy < ny; ++iy) {
        const float e = bathy.Elev()[iy * nx + ix];
        if (e > -9000.0f && e < eBest) {
            eBest = e;
            iyBest = iy;
        }
    }
    const float riverZ = bathy.WorldZ0() + bathy.WorldSizeZ() * (1.0f - (iyBest + 0.5f) / ny);
    Log("[swe-cycle] river probe (-5000, %.0f), bed %.1f m", riverZ, eBest);

    // Throat probe: the deepest channel cell within 150 m of the ACT0816 station -- the jet
    // core, not whatever shoal the exact origin lands on.
    float throatX = 0, throatZ = 0, tBest = 1e9f;
    for (float wz = -150; wz <= 150; wz += 10) {
        for (float wx = -150; wx <= 150; wx += 10) {
            const float e = bathy.SampleWorld(wx, wz);
            if (e > -9000.0f && e < tBest) {
                tBest = e;
                throatX = wx;
                throatZ = wz;
            }
        }
    }
    Log("[swe-cycle] throat probe (%.0f, %.0f), bed %.1f m", throatX, throatZ, tBest);

    const char* path = "data/swe_cycle.csv";
    FILE* f = nullptr;
    fopen_s(&f, path, "w");
    if (!f) {
        Log("[swe-cycle] cannot write %s", path);
        return;
    }
    fprintf(f,
            "unix,tide_navd,act_pred_ms,ocean_deta,throat_deta,throat_u,throat_v,basin_deta,"
            "river_deta,west_target,gap_u\n");
    const double endUnix = startUnix + hours * 3600.0;
    int rows = 0;
    for (double t = startUnix; t <= endUnix; t += 120.0) {
        swe.AdvanceTo(gpu, t, tideAt, westAt, southAt, westQAt);
        // 4 named probes + a 6-point transect across the jetty gap (x = 350) whose valid-point
        // mean is the fair section current to hold against ACT0816.
        const float pts[20] = {2600.0f, 0.0f,   throatX, throatZ, -2500.0f, -900.0f,
                               -5000.0f, riverZ, 350.0f, -350.0f, 350.0f,  -250.0f,
                               350.0f,  -150.0f, 350.0f, -50.0f,  350.0f,  50.0f,
                               350.0f,  150.0f};
        SweSolver::Probe pr[10];
        swe.ReadProbes(gpu, pts, 10, pr);
        const SweSolver::Probe &ocean = pr[0], &throat = pr[1], &basin = pr[2], &river = pr[3];
        float gapU = 0;
        int gapN = 0;
        for (int gi = 4; gi < 10; ++gi) {
            if (pr[gi].valid) {
                gapU += pr[gi].u;
                ++gapN;
            }
        }
        gapU = gapN ? gapU / gapN : 0.0f;
        const double act =
            (currents && ctSta >= 0) ? currents->SignedSpeed(static_cast<size_t>(ctSta), t) : 0.0;
        fprintf(f, "%.0f,%.4f,%.3f,%.4f,%.4f,%.3f,%.3f,%.4f,%.4f,%.4f,%.3f\n", t, tideAt(t),
                act, ocean.dEta, throat.dEta, throat.u, throat.v, basin.dEta, river.dEta,
                westAt(t), gapU);
        if (++rows % 30 == 0) {
            Log("[swe-cycle] +%.1f h  tide %+.2f  gap u %+.2f (ACT %+.2f)  basin dEta %+.3f",
                (t - startUnix) / 3600.0, tideAt(t), gapU, act, basin.dEta);
        }
    }
    fclose(f);
    Log("[swe-cycle] wrote %s (%d rows)", path, rows);
}

// The mode: spin the solver up two hours of history, integrate opt.sweCycleH hours to
// data/swe_cycle.csv, tear down, exit 0.
template <typename Ocean, typename South, typename West, typename WestQ>
int RunSweCycleMode(const Options& opt, Gpu& gpu, const CurrentModel& currents, bool haveCurrents,
                    const BathyModel& bathy, SweSolver& swe, ResidencyManager& resMgr,
                    double simUnix, Ocean oceanAt, South southAt, West westAt, WestQ westQAt) {
    swe.Spinup(gpu, simUnix, 2.0, oceanAt, westAt, southAt, westQAt);
    const int ctSta = haveCurrents ? currents.StationIndex("ACT0816") : -1;
    RunSweCycle(gpu, swe, oceanAt, westAt, southAt, westQAt,
                haveCurrents ? &currents : nullptr, ctSta, bathy, simUnix,
                opt.sweCycleH);
    gpu.WaitIdle();
    resMgr.Shutdown();
    gpu.Shutdown();
    return 0;
}

}  // namespace ga::app::tools
