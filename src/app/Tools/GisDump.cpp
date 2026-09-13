// GisDump - --gis-dump PATH.
// Moved verbatim from main.cpp (M12 step 1b); declared in app/Tools.h.
// The body ends in std::exit(0), exactly as it did in main(); the call site is a statement.
#include "app/Tools.h"

#include "compose/GisMask.h"
#include "core/Common.h"

#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <vector>

namespace ga::app::tools {

void RunGisDump(const Options& opt, const GisVectorMask& gisMask) {
    // M9av: LOOK AT THE GATE. 255 = water (or no opinion), 0 = land.
    double b0, a0, b1, a1;
    gisMask.Bounds(b0, a0, b1, a1);
    const uint32_t dim = 1024;
    std::vector<uint8_t> g;
    gisMask.RasterizeGate(a0, a1, b0, b1, dim, g);
    std::vector<uint8_t> v(static_cast<size_t>(dim) * dim);
    for (size_t i = 0; i < v.size(); ++i) {
        // value where surveyed; 128 (grey) where the mask has no opinion
        v[i] = (g[i * 2 + 1] & 1u) ? g[i * 2] : 128u;
    }
    std::ofstream pf(opt.gisDump, std::ios::binary);
    pf << "P5\n" << dim << " " << dim << "\n255\n";
    pf.write(reinterpret_cast<const char*>(v.data()), v.size());
    Log("[gismask] --gis-dump: %s (%ux%u over %.3f,%.3f..%.3f,%.3f) %s",
        opt.gisDump.c_str(), dim, dim, b0, a0, b1, a1,
        pf ? "written" : "FAILED TO WRITE");
    std::exit(0);
}

}  // namespace ga::app::tools
