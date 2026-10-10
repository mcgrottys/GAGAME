// BuildingLod - --tool building-lod[:lon0,lat0,lon1,lat1]: the scene's building stack, every cell
// composed and boxed into the size-stratified pyramid at layers.buildings.lod (compose/
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
    const int threads = static_cast<int>((std::max)(1u, std::thread::hardware_concurrency()));
    Log("[lod] boxing every cell%s into %s (rho0 %.2f m, %d threads)", boxed ? " in the box" : "", bl->lod.c_str(),
        bl->lodRho0, threads);
    LodBuildStats st;
    log.clear();
    if (!BuildBuildingLod(stack, bl->lod, bl->lodRho0, boxed ? box : nullptr, threads, &st, &log)) {
        Log("[lod] refused: %s", log.c_str());
        return 2;
    }
    Log("[lod] %llu cells, %llu solids in %.1f s", static_cast<unsigned long long>(st.cells),
        static_cast<unsigned long long>(st.solids), st.seconds);
    for (int k = lod::kMinLevel; k <= lod::kMaxLevel; ++k) {
        Log("[lod]   level %d (rho >= %.0f m, tiles of %.2f deg): %llu boxes in %llu tiles", k, bl->lodRho0 * double(1 << k),
            lod::TileDeg(k), static_cast<unsigned long long>(st.kept[k]), static_cast<unsigned long long>(st.tiles[k]));
    }
    return 0;
}

}  // namespace ga::app::tools
