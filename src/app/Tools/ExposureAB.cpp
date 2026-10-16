// ================================================================================================
//  --tool exposure-ab: THE OPEN SEA'S SHORTCUT AGAINST THE MARCH (compose/ExposureSource.h).
//
//  Where the stack's ceiling over a texel's reach is under the shadow's 0.15 m (Compositor::
//  HeightStackCeiling), the exposure is answered 1 without its 750 reads -- a claim of EQUALITY, so
//  this holds every texel of a patch at each place, both ways, value for value: the scene's place, two
//  coasts the travels reach and two seas offshore, at the page's finest grain (76 m) and a coarser one,
//  two swell directions. The time each way and the share answered open. THE PLANT: the ceiling's
//  limit raised 10 m, which must part from the march somewhere near a coast.
// ================================================================================================
#include "app/Tools.h"

#include "compose/Compositor.h"
#include "compose/ExposureSource.h"
#include "core/Common.h"

#include <chrono>
#include <cmath>
#include <string>

namespace ga::app::tools {

int RunExposureAB(const Compositor& comp, int hgtCh, double anchorLatDeg, double anchorLonDeg) {
    struct Place {
        const char* name;
        double lat, lon;
    };
    const Place places[] = {{"the scene's place", anchorLatDeg, anchorLonDeg},
                            {"Haulover inlet", 25.90015, -80.12236},
                            {"the Merrimack mouth", 42.8189, -70.8066},
                            {"off Virginia", 36.76346, -74.31662},
                            {"the Gulf of Mexico", 26.27188, -87.75406},
                            {"off the Chandeleur Islands", 29.82, -88.83},
                            {"off the Dry Tortugas", 24.60, -82.90}};
    ExposureSource src(&comp, hgtCh);
    constexpr double kR = 6371008.8, kD2R = 3.14159265358979323846 / 180.0;
    bool ok = true, caught = false;
    size_t allTexels = 0, allOpen = 0;
    double allMarch = 0.0, allFast = 0.0;
    const auto forget = [] {
        for (auto& c : expo_detail::t_open) c = {};
    };
    for (const Place& pl : places) {
        {   // what each source says of the place's box (a 10 km square): its range, or why it has none
            const double dLat = 5000.0 / kR / kD2R, dLon = dLat / std::cos(pl.lat * kD2R);
            const Compositor::Channel& ch = comp.ChannelAt(hgtCh);
            for (size_t i = ch.height.size(); i-- > 0;) {
                float lo = 0.0f, hi = 0.0f;
                bool full = false;
                const auto r = ch.height[i]->RangeIn(pl.lat - dLat, pl.lon - dLon, pl.lat + dLat, pl.lon + dLon, lo, hi, full);
                Log("[exposure-ab]   %s: %s %s", ch.height[i]->Info().name.c_str(),
                    r == HeightSource::Range::None ? "none" : r == HeightSource::Range::Unknown ? "UNKNOWN" : "range",
                    r == HeightSource::Range::Known ? (std::to_string(lo) + " .. " + std::to_string(hi) + (full ? " (owns the box)" : "")).c_str() : "");
            }
        }
        for (const double groundM : {76.0, 305.0}) {
            for (const float dirDeg : {0.0f, 135.0f}) {
                src.Set(std::cos(dirDeg * float(kD2R)), std::sin(dirDeg * float(kD2R)), 0.0, true);
                const int n = groundM < 100.0 ? 64 : 32;
                size_t diff = 0, open = 0, diffPlant = 0;
                double tMarch = 0.0, tFast = 0.0;
                for (int j = 0; j < n; ++j) {
                    for (int i = 0; i < n; ++i) {
                        DomainQuery q;
                        q.lat = pl.lat + (j - n / 2) * groundM / kR / kD2R;
                        q.lon = pl.lon + (i - n / 2) * groundM / (kR * std::cos(pl.lat * kD2R)) / kD2R;
                        q.groundM = groundM;
                        DomainValue a, b, c;
                        ExposureSource::s_openSea = false;
                        auto t0 = std::chrono::steady_clock::now();
                        const bool ra = src.SampleAt(q, a);
                        tMarch += std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
                        ExposureSource::s_openSea = true;
                        t0 = std::chrono::steady_clock::now();
                        const bool rb = src.SampleAt(q, b);
                        tFast += std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
                        open += ExposureSource::s_lastOpen ? 1 : 0;
                        if (ra != rb || a.weight != b.weight || a.c[0] != b.c[0]) ++diff;
                        forget();
                        ExposureSource::s_openSlack = 100.0f;   // the plant
                        src.SampleAt(q, c);
                        ExposureSource::s_openSlack = 0.0f;
                        forget();
                        if (c.c[0] != a.c[0]) ++diffPlant;
                    }
                }
                Log("[exposure-ab] %s (%.4f, %.4f) at %.0f m, swell toward %.0f deg: %d x %d texels -- %zu differ "
                    "from the march, %zu answered open; the march %.1f ms, with the shortcut %.1f ms; the plant "
                    "parts at %zu",
                    pl.name, pl.lat, pl.lon, groundM, double(dirDeg), n, n, diff, open, tMarch, tFast, diffPlant);
                ok &= diff == 0;
                caught |= diffPlant > 0;
                allTexels += size_t(n) * size_t(n);
                allOpen += open;
                allMarch += tMarch;
                allFast += tFast;
            }
        }
    }
    ExposureSource::s_openSea = true;
    Log("[exposure-ab] %zu texels, %zu answered open (%.1f %%): the march %.0f ms, with the shortcut %.0f ms (%.1fx) "
        "-- %s; the plant %s",
        allTexels, allOpen, 100.0 * double(allOpen) / double((std::max)(allTexels, size_t(1))), allMarch, allFast,
        allMarch / (std::max)(allFast, 1e-6), ok ? "EQUAL everywhere" : "DIFFERENT", caught ? "CAUGHT" : "NOT CAUGHT");
    return ok && caught ? 0 : 1;
}

}  // namespace ga::app::tools
