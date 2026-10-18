// HeightTreeProbe - --tool height-tree-probe: the CPU's read of the height tree (compose/HeightTree.h)
// against the stack it is painted from, before any reader is moved onto it. Declared in app/Tools.h.
//
// At random points about the scene's place (a tenth of a degree: the ground one reader works over)
// at grains from a metre and a half to six kilometres, and over the whole sphere at 400 m and
// coarser (finer would paint thousands of scattered tiles into the cache): the tree's answer and the
// stack's, the difference's median, 95th percentile and worst, and what one sample costs -- the
// tree COLD (its first pass paints the tiles it lacks, as a reader's first visit does) and WARM (the
// same points again, from its cache), the stack each time. They are not equal by
// construction -- the tree is the stack at texel centres one level finer, bilinear between them and
// stored as half floats (the GPU's quantization), where the stack is an area mean at the point --
// so this measures the difference; it does not assume it. THE PLANT: the tree read three levels too
// coarse, at the place, must differ from the stack far more than the law does, or this instrument
// cannot see a level gone wrong.
#include "app/Tools.h"

#include "compose/Compositor.h"
#include "compose/HeightTree.h"
#include "compose/TileTree.h"
#include "core/Common.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <random>
#include <vector>

namespace ga::app::tools {

namespace {

struct Stat {
    double median = 0.0, p95 = 0.0, worst = 0.0, treeNs = 0.0, stackNs = 0.0;
    size_t n = 0;
};

Stat Measure(Compositor& comp, int ch, HeightTree& tree, const std::vector<std::pair<double, double>>& pts,
             double grainM, int levelOffset) {
    std::vector<double> d;
    d.reserve(pts.size());
    double tTree = 0.0, tStack = 0.0;
    for (const auto& [la, lo] : pts) {
        const auto t0 = std::chrono::steady_clock::now();
        const float a = tree.Sample(la, lo, grainM, levelOffset);
        const auto t1 = std::chrono::steady_clock::now();
        const float b = comp.SampleHeightStack(ch, la, lo, grainM);
        const auto t2 = std::chrono::steady_clock::now();
        tTree += std::chrono::duration<double, std::nano>(t1 - t0).count();
        tStack += std::chrono::duration<double, std::nano>(t2 - t1).count();
        d.push_back(std::abs(double(a) - double(b)));
    }
    std::sort(d.begin(), d.end());
    Stat s;
    s.n = d.size();
    if (d.empty()) return s;
    s.median = d[d.size() / 2];
    s.p95 = d[size_t(0.95 * double(d.size() - 1))];
    s.worst = d.back();
    s.treeNs = tTree / double(d.size());
    s.stackNs = tStack / double(d.size());
    return s;
}

}  // namespace

int RunHeightTreeProbe(Compositor& comp, int ch, double anchorLonDeg, double anchorLatDeg, const std::string& args) {
    constexpr double kD2R = 3.14159265358979 / 180.0;
    if (args.rfind("transect=", 0) == 0) {   // THE TRANSECT: the stack and each source along a line
        double la0 = 0, lo0 = 0, la1 = 0, lo1 = 0;
        int n = 0;
        if (sscanf_s(args.c_str() + 9, "%lf,%lf,%lf,%lf,%d", &la0, &lo0, &la1, &lo1, &n) != 5 || n < 2) {
            Log("[height-tree] transect wants lat0,lon0,lat1,lon1,n");
            return 1;
        }
        const Compositor::Channel& chan = comp.ChannelAt(ch);
        const double c = std::cos(0.5 * (la0 + la1) * kD2R), R = 6378137.0;
        const double lenM = std::hypot((lo1 - lo0) * kD2R * R * c, (la1 - la0) * kD2R * R), step = lenM / (n - 1);
        Log("[height-tree] transect %.6f,%.6f -> %.6f,%.6f: %.0f m in %d points (%.2f m), read at %.2f m; the stack, then "
            "each source with a weight: name w value", la0, lo0, la1, lo1, lenM, n, step, step);
        for (int i = 0; i < n; ++i) {
            const double f = double(i) / (n - 1), la = (la0 + (la1 - la0) * f) * kD2R, lo = (lo0 + (lo1 - lo0) * f) * kD2R;
            const double g = step / std::cos(la);
            std::string row;
            char b[160];
            snprintf(b, sizeof(b), "[height-tree] %4.0f m (%.6f, %.6f): stack %+8.3f |", f * lenM, la / kD2R, lo / kD2R,
                     comp.SampleHeightStack(ch, la, lo, g));
            row = b;
            for (HeightSource* h : chan.height) {
                float m = 0.0f;
                const float w = h ? h->Sample(la, lo, g, m) : 0.0f;
                if (w <= 0.0f) continue;
                snprintf(b, sizeof(b), " %s %.3f %+.3f", h->Info().name.c_str(), w, m);
                row += b;
            }
            Log("%s", row.c_str());
        }
        return 0;
    }
    HeightTree* tree = comp.HeightTreeOf(ch);
    if (!tree) {
        Log("[height-tree] no tree is bound to the height channel (streaming.colorTrees is off): nothing to probe");
        return 1;
    }
    std::mt19937_64 rng(20261014);
    std::uniform_real_distribution<double> u01(0.0, 1.0);
    std::vector<std::pair<double, double>> place, sphere;
    for (size_t i = 0; i < 2000; ++i) {
        place.push_back({(anchorLatDeg + 0.1 * (u01(rng) - 0.5)) * kD2R,
                         (anchorLonDeg + 0.1 * (u01(rng) - 0.5) / std::max(0.2, std::cos(anchorLatDeg * kD2R))) * kD2R});
    }
    for (size_t i = 0; i < 500; ++i) sphere.push_back({std::asin(2.0 * u01(rng) - 1.0), (2.0 * u01(rng) - 1.0) * 3.14159265358979});
    Log("[height-tree] the tree against the stack: %zu points about the place (%.4f, %.4f) +-0.05 deg, %zu over the "
        "sphere; |tree - stack| in metres (warm), and a sample's cost: the tree cold / warm, the stack",
        place.size(), anchorLatDeg, anchorLonDeg, sphere.size());
    for (const double g : {1.5, 6.0, 25.0, 100.0, 400.0, 1600.0, 6400.0}) {
        for (int where = 0; where < 2; ++where) {
            if (where == 1 && g < 400.0) continue;
            const uint64_t r0 = tree->tileReads.load();
            const Stat cold = Measure(comp, ch, *tree, where ? sphere : place, g, 0);
            const uint64_t r1 = tree->tileReads.load();
            const Stat s = Measure(comp, ch, *tree, where ? sphere : place, g, 0);
            Log("[height-tree] %-6s %6.1f m (level %2u): median %.3f  p95 %.3f  worst %8.3f m | tree %9.0f / %5.0f ns  stack %8.0f ns | "
                "%llu tiles read cold, %llu warm",
                where ? "sphere" : "place", g, tree->LevelFor(g), s.median, s.p95, s.worst, cold.treeNs, s.treeNs, s.stackNs,
                static_cast<unsigned long long>(r1 - r0), static_cast<unsigned long long>(tree->tileReads.load() - r1));
        }
    }
    // THE TEXEL'S CENTRE AT ITS OWN LEVEL (HeightPage's read, the hull's bed): the tree's answer there is
    // the texel itself -- the bytes the GPU loads. Where every source paints that level directly it is the
    // stack at the centre through the GPU's quantization; where a source's own level is finer, it is the
    // FOLD of the four texels beneath (HIERARCHY 4.20), and the stack's direct read is not what is drawn.
    // MEASURED on the Merrimack: equal at 93 % of texels at level 9, 68 % at 11, 49 % at 13 (worst 2 cm,
    // 25 cm, 2.7 m): the folds' distance from the stack, which a twin of the GPU must not pretend away.
    {
        const Lattice pyr = Lattice::Cube(Lattice::kFaceDim << 17, 128, 128);
        for (const uint32_t mip : {9u, 11u, 13u}) {
            double worst = 0.0;
            size_t same = 0, n = 0;
            for (const auto& [la, lo] : place) {
                const double cl = std::cos(la);
                const double d[3] = {cl * std::cos(lo), std::sin(la), cl * std::sin(lo)};
                double uv[2];
                const uint32_t face = CubeFaceOfDir(d, uv);
                const uint32_t dim = pyr.faceDim >> mip;
                const uint32_t ix = uint32_t(uv[0] * dim), iy = uint32_t(uv[1] * dim);
                const TileRequest r{face, mip, ix / pyr.texW, iy / pyr.texH};
                double cla = 0.0, clo = 0.0;
                pyr.Texel(r, ix % pyr.texW, iy % pyr.texH, cla, clo);
                const float t = tree->Sample(cla, clo, pyr.GroundRes(mip));
                const float h = comp.SampleHeightStack(ch, cla, clo, pyr.GroundRes(mip));   // R32F: as composed
                worst = (std::max)(worst, std::abs(double(t) - double(h)));
                same += t == h ? 1 : 0;
                ++n;
            }
            Log("[height-tree] texel centres at level %2u (%.2f m): the tree (what the GPU loads) equals the stack's half float at %zu of %zu, worst %.4f m",
                mip, pyr.GroundRes(mip), same, n, worst);
        }
    }
    // THE RAY (HeightTree::MaxAlongRay, the exposure's): the highest ground a ray toward the swell crosses,
    // against the march it replaced -- the tree sampled every step along a line of constant bearing in
    // degrees -- as the excess over the water line (0 NAVD here) the exposure reads, at water points
    // about the place, five swell directions, 4 km at 13 m. Not equal by construction: the ray is the
    // great circle and takes every texel it crosses, the march a point every step, bilinear. THE PLANT:
    // the ray walked the other way, which must not agree.
    {
        constexpr double kRange = 4000.0, kStep = 13.0, kLevel = 0.0, kR = 6371000.0;
        std::vector<double> d, dPlant;
        double tMarch = 0.0, tRay = 0.0;
        size_t rays = 0;
        for (const auto& [la, lo] : place) {
            if (tree->Sample(la, lo, kStep) > kLevel) continue;   // the exposure marches from water only
            for (const double azDeg : {0.0, 72.0, 144.0, 216.0, 288.0}) {
                const double ang = azDeg * kD2R, dirX = std::cos(ang), dirZ = std::sin(ang);
                const auto t0 = std::chrono::steady_clock::now();
                double ex = 0.0;
                double lat = la, lon = lo;
                const double dLat = -dirZ * kStep / kR, dLon = -dirX * kStep / (kR * std::cos(la));
                for (int k = 0; k < int(kRange / kStep); ++k) {
                    lat += dLat;
                    lon += dLon;
                    ex = (std::max)(ex, double(tree->Sample(lat, lon, kStep)) - kLevel);
                    if (ex > 1.2) break;
                }
                const auto t1 = std::chrono::steady_clock::now();
                const double top = tree->MaxAlongRay(la, lo, -dirX, -dirZ, kRange, kStep, float(kLevel), float(kLevel + 1.2000001));
                const auto t2 = std::chrono::steady_clock::now();
                const double back = tree->MaxAlongRay(la, lo, dirX, dirZ, kRange, kStep, float(kLevel), float(kLevel + 1.2000001));
                const double exRay = (std::min)(1.2000001, (std::max)(0.0, top - kLevel));
                const double exBack = (std::min)(1.2000001, (std::max)(0.0, back - kLevel));
                ex = (std::min)(1.2000001, ex);
                d.push_back(std::abs(exRay - ex));
                dPlant.push_back(std::abs(exBack - ex));
                tMarch += std::chrono::duration<double, std::nano>(t1 - t0).count();
                tRay += std::chrono::duration<double, std::nano>(t2 - t1).count();
                ++rays;
            }
        }
        std::sort(d.begin(), d.end());
        std::sort(dPlant.begin(), dPlant.end());
        const auto q = [](const std::vector<double>& v, double f) { return v.empty() ? 0.0 : v[size_t(f * double(v.size() - 1))]; };
        size_t agree = 0, agreePlant = 0;
        for (double x : d) agree += x < 0.05 ? 1 : 0;
        for (double x : dPlant) agreePlant += x < 0.05 ? 1 : 0;
        Log("[height-tree] ray: %zu rays from water, |excess(ray) - excess(march)| median %.3f p95 %.3f m, within 5 cm at %.1f %% | "
            "march %.1f us, ray %.1f us a ray",
            rays, q(d, 0.5), q(d, 0.95), rays ? 100.0 * double(agree) / double(rays) : 0.0,
            rays ? tMarch / double(rays) / 1000.0 : 0.0, rays ? tRay / double(rays) / 1000.0 : 0.0);
        Log("[height-tree] PLANT the ray walked the other way: within 5 cm at %.1f %%, p95 %.3f m -- %s",
            rays ? 100.0 * double(agreePlant) / double(rays) : 0.0, q(dPlant, 0.95),
            agreePlant < agree ? "CAUGHT" : "NOT CAUGHT: this instrument cannot see a ray gone wrong");
    }
    // THE PLANT: three levels too coarse, at the place, at 6 m.
    const Stat law = Measure(comp, ch, *tree, place, 6.0, 0);
    const Stat plant = Measure(comp, ch, *tree, place, 6.0, 3);
    const bool caught = plant.p95 > 2.0 * law.p95 + 0.05;
    Log("[height-tree] PLANT three levels coarse at the place, 6 m: p95 %.3f against the law's %.3f m -- %s",
        plant.p95, law.p95, caught ? "CAUGHT" : "NOT CAUGHT: this instrument cannot see a level gone wrong");
    Log("[height-tree] the cache: %zu tiles held, %llu tile reads in %.1f s", tree->Held(),
        static_cast<unsigned long long>(tree->tileReads.load()), double(tree->readUs.load()) / 1.0e6);
    return caught ? 0 : 1;
}

}  // namespace ga::app::tools
