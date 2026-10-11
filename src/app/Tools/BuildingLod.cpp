// BuildingLod - --tool building-lod[:lon0,lat0,lon1,lat1]: the scene's building stack, every cell
// composed, boxed and folded into the tree at layers.buildings.lod (compose/
// BuildingLod.h, docs/BUILDING_LOD.md). On the CPU, before any device. Declared in app/Tools.h.
#include "app/Tools.h"

#include "app/Scene.h"
#include "compose/BuildingLod.h"
#include "core/Common.h"

#include <cstdio>
#include <thread>

namespace ga::app::tools {

int RunBuildingLod(const Scene& S, const std::string& args) {
    const SceneLayer* bl = S.Layer("buildings");
    if (!bl || bl->lod.empty()) {
        Log("[lod] refused: the scene names no layers.buildings.lod folder");
        return 2;
    }
    double box[4];
    const bool boxed = !args.empty() && sscanf_s(args.c_str(), "%lf,%lf,%lf,%lf", &box[0], &box[1], &box[2], &box[3]) == 4;
    if (!args.empty() && !boxed) {
        Log("[lod] refused: the argument '%s' is not lon0,lat0,lon1,lat1", args.c_str());
        return 2;
    }
    std::vector<BuildingSourceSpec> specs;
    for (const scene::SourceProps& s : S.sources) {
        if (s.kind == "buildings") specs.push_back({s.name, s.manifest.empty() ? s.file : s.manifest, s.over});
    }
    BuildingStack stack;
    std::string log;
    stack.Open(std::move(specs), {bl->levelHeight, bl->defaultHeight}, &log);
    for (size_t p0 = 0, p1; (p1 = log.find('\n', p0)) != std::string::npos; p0 = p1 + 1) Log("[lod] %s", log.substr(p0, p1 - p0).c_str());
    // The roads (sources of kind "roads"), filed in the same tree as ribbons (docs/ROADS.md).
    std::vector<RoadFile> roads;
    for (const scene::SourceProps& s : S.sources) {
        if (s.kind != "roads") continue;
        RoadFile rf;
        std::string why;
        if (rf.Open(s.manifest.empty() ? s.file : s.manifest, &why)) {
            Log("[lod] roads '%s': %zu cells of %.2f deg", s.name.c_str(), rf.Cells().size(), rf.CellDeg());
            roads.push_back(std::move(rf));
        } else {
            Log("[lod] roads '%s' refused: %s", s.name.c_str(), why.c_str());
        }
    }
    const RoadRibbonDefaults rd{bl->laneWidth, bl->defaultLanes, bl->pathWidth, bl->kerb};
    const int threads = static_cast<int>((std::max)(1u, std::thread::hardware_concurrency()));
    Log("[lod] folding every cell%s into the tree at %s (%d threads)", boxed ? " in the box" : "", bl->lod.c_str(), threads);
    LodBuildStats st;
    log.clear();
    if (!BuildBuildingLod(stack, roads, rd, bl->lod, boxed ? box : nullptr, threads, &st, &log)) {
        Log("[lod] refused: %s", log.c_str());
        return 2;
    }
    Log("[lod] %llu cells, %llu solids (%llu with volume) in %.1f s", static_cast<unsigned long long>(st.cells),
        static_cast<unsigned long long>(st.solids), static_cast<unsigned long long>(st.kept), st.seconds);
    if (!roads.empty()) {
        Log("[lod] roads: %llu ways -> %llu ribbon pieces (%llu tunnels left out); lane %.2f m x %.0f, path %.2f m, kerb %.2f m",
            static_cast<unsigned long long>(st.ways), static_cast<unsigned long long>(st.ribbons),
            static_cast<unsigned long long>(st.tunnels), rd.laneWidth, rd.defaultLanes, rd.pathWidth, rd.kerb);
    }
    for (int L = lod::kLmin; L <= lod::kLmax; ++L) {
        Log("[lod]   level %+d (quads of %.0f m): %llu nodes, %llu buildings of their own", L,
            lod::QuadDeg(L) * lod::kMetresPerDeg, static_cast<unsigned long long>(st.nodes[L - lod::kLmin]),
            static_cast<unsigned long long>(st.owned[L - lod::kLmin]));
    }
    return 0;
}

}  // namespace ga::app::tools
