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

int RunHeightTreeProbe(Compositor& comp, int ch, double anchorLonDeg, double anchorLatDeg) {
    HeightTree* tree = comp.HeightTreeOf(ch);
    if (!tree) {
        Log("[height-tree] no tree is bound to the height channel (streaming.colorTrees is off): nothing to probe");
        return 1;
    }
    constexpr double kD2R = 3.14159265358979 / 180.0;
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
    // THE PLANT: three levels too coarse, at the place, at 6 m.
    const Stat law = Measure(comp, ch, *tree, place, 6.0, 0);
    const Stat plant = Measure(comp, ch, *tree, place, 6.0, 3);
    const bool caught = plant.p95 > 2.0 * law.p95 + 0.05;
    Log("[height-tree] PLANT three levels coarse at the place, 6 m: p95 %.3f against the law's %.3f m -- %s",
        plant.p95, law.p95, caught ? "CAUGHT" : "NOT CAUGHT: this instrument cannot see a level gone wrong");
    Log("[height-tree] the cache: %zu tiles held, %llu tile reads, %llu texel reads", tree->Held(),
        static_cast<unsigned long long>(tree->tileReads.load()), static_cast<unsigned long long>(tree->texelReads.load()));
    return caught ? 0 : 1;
}

}  // namespace ga::app::tools
