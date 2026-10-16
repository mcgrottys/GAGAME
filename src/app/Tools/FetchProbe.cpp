// ================================================================================================
//  --tool fetch-probe: THE FETCH ON THE HEIGHT TREE (HeightTree::Fetch).
//
//  The fetch -- how far the water runs upwind before the first ground above the water line, the
//  input of fetch-limited wind-sea growth -- is the ray walk the swell shadow takes, stopped at the
//  first texel above the line: every texel the great circle crosses, the blocks below the line
//  passed over whole. This holds it against a brute march along the same great circle (a point
//  every quarter texel, the tree's own sample at the walk's grain) at water points about the scene's
//  place, eight directions out to 50 km: the share within one texel and the worst, and the cost of
//  each. THE PLANT: the walk with the water line five metres up, which must not agree.
//
//  Measured first (2026-10-14) and dropped: a sweep of a tile's parallel lines (Stewart 1998's
//  horizon sweeps). The walk's block skip already costs open water nothing, so the sweep was 1.5x
//  for the swell shadow (and 2 % of texels apart, by up to 0.88, at wall edges) and 470x SLOWER for
//  the fetch. One exact walk serves both.
// ================================================================================================
#include "app/Tools.h"

#include "compose/HeightTree.h"
#include "core/Common.h"
#include "core/Lattice.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <vector>

namespace ga::app::tools {

namespace {
double Q(std::vector<double> v, double f) {
    if (v.empty()) return 0.0;
    std::sort(v.begin(), v.end());
    return v[size_t(f * double(v.size() - 1))];
}
}  // namespace

int RunFetchProbe(HeightTree& tree, double anchorLatDeg, double anchorLonDeg) {
    constexpr double kD2R = 3.14159265358979323846 / 180.0;
    constexpr double R = Lattice::kMercCirc / (2.0 * 3.14159265358979323846);
    constexpr double kRange = 50000.0, kGrain = 13.0;
    const float level = 0.0f;
    const double la = anchorLatDeg * kD2R, lo = anchorLonDeg * kD2R;
    // the texel the walk reads at its grain: the tolerance
    const double texel = kGrain;
    struct P { double lat, lon; };
    std::vector<P> pts;
    for (int j = -8; j <= 8; ++j) {
        for (int i = -8; i <= 8; ++i) {
            const double plat = la + j * 120.0 / R, plon = lo + i * 120.0 / (R * std::cos(la));
            if (tree.Sample(plat, plon, kGrain) <= level) pts.push_back({plat, plon});
        }
    }
    std::vector<double> err, errPlant;
    size_t within = 0, withinPlant = 0, n = 0, reach = 0, walkShort = 0, walkLong = 0;
    double usWalk = 0.0, usMarch = 0.0;
    for (int k = 0; k < 8; ++k) {
        const double a = k * 45.0 * kD2R, tE = std::cos(a), tN = std::sin(a);
        for (const P& p : pts) {
            auto t0 = std::chrono::steady_clock::now();
            const double walk = tree.Fetch(p.lat, p.lon, tE, tN, kRange, kGrain, level);
            auto t1 = std::chrono::steady_clock::now();
            const double plant = tree.Fetch(p.lat, p.lon, tE, tN, kRange, kGrain, level + 5.0f);
            // the brute march: the same great circle, a point every quarter texel
            const double cl = std::cos(p.lat), sl = std::sin(p.lat), co = std::cos(p.lon), so = std::sin(p.lon);
            const double q0[3] = {cl * co, sl, cl * so}, E[3] = {-so, 0.0, co}, N[3] = {-sl * co, cl, -sl * so};
            double t[3];
            for (int c = 0; c < 3; ++c) t[c] = tE * E[c] + tN * N[c];
            auto t2 = std::chrono::steady_clock::now();
            double march = kRange;
            for (double s = texel * 0.25; s < kRange; s += texel * 0.25) {
                const double ang = s / R;
                double d[3];
                for (int c = 0; c < 3; ++c) d[c] = q0[c] * std::cos(ang) + t[c] * std::sin(ang);
                const double plat = std::asin(std::clamp(d[1], -1.0, 1.0)), plon = std::atan2(d[2], d[0]);
                if (tree.Sample(plat, plon, kGrain) > level) {
                    march = s;
                    break;
                }
            }
            auto t3 = std::chrono::steady_clock::now();
            usWalk += std::chrono::duration<double, std::micro>(t1 - t0).count();
            usMarch += std::chrono::duration<double, std::micro>(t3 - t2).count();
            reach += march < kRange ? 1 : 0;
            const double e = std::abs(walk - march), ep = std::abs(plant - march);
            err.push_back(e);
            errPlant.push_back(ep);
            within += e <= texel ? 1 : 0;
            walkShort += walk < march - texel ? 1 : 0;   // the walk met ground the march's bilinear did not
            walkLong += walk > march + texel ? 1 : 0;
            withinPlant += ep <= texel ? 1 : 0;
            ++n;
        }
    }
    const double pct = 100.0 * double(within) / double((std::max)(size_t(1), n));
    const double pctPlant = 100.0 * double(withinPlant) / double((std::max)(size_t(1), n));
    Log("[fetch] %zu water points x 8 directions to %.0f km about %.5f N %.5f E (%zu reach ground) -- |walk - march| "
        "median %.1f m, p95 %.1f m, worst %.1f m; within one texel (%.0f m) at %.2f %% | the walk %.1f us, the march "
        "%.1f us a fetch",
        pts.size(), kRange / 1000.0, anchorLatDeg, anchorLonDeg, reach, Q(err, 0.5), Q(err, 0.95), Q(err, 1.0), texel,
        pct, usWalk / double((std::max)(size_t(1), n)), usMarch / double((std::max)(size_t(1), n)));
    Log("[fetch] apart by more than a texel: the walk shorter at %zu (ground the march's bilinear samples smooth "
        "below the line: a texel above it between two below), the walk longer at %zu", walkShort, walkLong);
    Log("[fetch] PLANT the water line five metres up: within one texel at %.2f %% -- %s", pctPlant,
        pctPlant < pct ? "CAUGHT" : "NOT CAUGHT: this instrument cannot see a fetch gone wrong");
    return pctPlant < pct ? 0 : 1;
}

}  // namespace ga::app::tools
