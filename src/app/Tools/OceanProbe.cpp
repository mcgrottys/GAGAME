// OceanProbe - --ocean-probe lat,lon.
// Moved verbatim from main.cpp (M12 step 1b); declared in app/Tools.h.
// nullopt = "lat,lon" did not parse. In main() that branch never returned and the run
// went on; the call site falls through on nullopt to keep that exact behaviour.
#include "app/Tools.h"

#include "core/Common.h"
#include "core/Gpu.h"
#include "core/Residency.h"
#include "render/Renderer.h"
#include "sim/SweSolver.h"
#include "sim/TideModel.h"
#include "sim/WeatherManager.h"

#include <cmath>
#include <cstdio>
#include <functional>
#include <optional>

namespace ga::app::tools {

// --ocean-probe lat,lon: the manager's verification harness. Standing at the point
// below the activation altitude IS the zoom -- dormant windows containing it spin up
// through the same rule the camera uses; then the sample prints at three rungs with
// full provenance, plus consistency gates against the station truth.
std::optional<int> RunOceanProbe(const Options& opt, const TideModel& model, Gpu& gpu,
                                 Renderer& renderer, SweSolver& swe,
                                 ResidencyManager& resMgr, double simUnix,
                                 const std::function<double(double)>& oceanAt,
                                 const std::function<double(double)>& southAt,
                                 const std::function<double(double)>& westAt,
                                 const std::function<double(double)>& westQAt,
                                 WeatherManager& weather) {
    double plat = 0, plon = 0;
    if (sscanf_s(opt.oceanProbe.c_str(), "%lf,%lf", &plat, &plon) == 2) {
        // The external merrimack window needs its history before it can be mirrored
        // (interactive runs spin it up right after this block).
        Log("[wx] probe: spinning the merrimack window");
        if (swe.Ready()) swe.Spinup(gpu, simUnix, 0.5, oceanAt, westAt, southAt,
                                    westQAt);
        Log("[wx] probe: manager update at the probe point");
        weather.Update(gpu, renderer.Shaders(), opt.shaderDir, simUnix, plat, plon,
                       1000.0);
        // The probe reads the windows' CPU mirrors: fill them once, at its instant.
        weather.RefreshMirrorsTo(gpu, simUnix);
        auto show = [&](double res) {
            const WeatherSample ws = weather.Query(plat, plon, simUnix, res);
            Log("[wx] res %6.0f m: level %+6.2f  cur %+5.2f,%+5.2f  Hs %4.2f Tp %4.1f "
                "dir %3.0f  wind %+5.1f,%+5.1f  bed %+7.1f  depth %6.1f",
                res, ws.levelNavd, ws.u, ws.v, ws.hs, ws.tp, ws.dirDeg, ws.windU,
                ws.windV, ws.bedNavd, ws.depthM);
            Log("[wx]   level=%s | current=%s | waves=%s | wind=%s | bed=%s",
                ws.levelSrc, ws.currentSrc, ws.waveSrc, ws.windSrc, ws.bedSrc);
        };
        Log("[wx] probe (%.4f, %.4f) t=now -- three rungs:", plat, plon);
        show(20000.0);
        show(500.0);
        show(15.0);
        // Consistency gates: the manager's level vs the fitted station truth.
        bool ok = true;
        for (const char* sid : {"8443970", "8441841"}) {
            for (size_t i = 0; i < model.Count(); ++i) {
                if (model.S(i).id != sid) continue;
                const TideStation& st = model.S(i);
                if (st.mllwMinusNavdM < -900.0) continue;
                const double direct = model.Height(i, simUnix) + st.mllwMinusNavdM;
                const WeatherSample q = weather.Query(st.lat, st.lon, simUnix, 200.0);
                const double err = std::abs(q.levelNavd - direct);
                Log("[wx] gate %s (%s): manager %+.3f vs station %+.3f -- err %.0f mm "
                    "(%s)",
                    sid, st.name.c_str(), q.levelNavd, direct, err * 1000.0,
                    q.levelSrc);
                if (err > 0.25) ok = false;
            }
        }
        Log("[wx] ---- %s: %d windows active ----", ok ? "PASS" : "FAIL",
            weather.ActiveWindows());
        gpu.WaitIdle();
        resMgr.Shutdown();
        gpu.Shutdown();
        return ok ? 0 : 1;
    }
    return std::nullopt;
}

}  // namespace ga::app::tools
