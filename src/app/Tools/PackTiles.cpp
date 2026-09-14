// PackTiles - --pack-tiles.
// Moved verbatim from main.cpp (M12 step 1b); declared in app/Tools.h.
#include "app/Tools.h"

#include "compose/SurfaceFrame.h"
#include "compose/TileArchive.h"
#include "core/Common.h"

#include <cstdint>
#include <string>

namespace ga::app::tools {

// M9ah: pack the composed cache into per-realization archives and exit. No device, no
// scene -- this is a disk-to-disk job. The loose tiles are kept: the archive is derived,
// and the compositor keeps writing loose files as it paints, so anything painted after a
// pack must still be findable the old way.
int RunPackTiles(const Options&, const SurfaceFrame& surface) {
    // M12 step 4a: the window realizations are the surface's lattices' own tags
    // (Lattice::Tag); the z19 inlet is Tools/Export.cpp's export-only realization.
    const std::string winTag = surface.win.Tag(), detTag = surface.det.Tag(),
                      winHTag = surface.winH.Tag();
    const char* jobs[][2] = {
        {"earth.color", "cube16k"},
        {"earth.color", winTag.c_str()},
        {"earth.color", detTag.c_str()},
        {"earth.color", "window_z19_40699567_49405858"},
        {"earth.height", "cube16k"},
        {"earth.height", winHTag.c_str()},
        {"mars.height", "cube16k"},
    };
    uint32_t total = 0;
    for (const auto& j : jobs) total += TileArchive::Pack(j[0], j[1]);
    Log("[tilearch] %u tiles packed across %zu realizations", total,
        sizeof(jobs) / sizeof(jobs[0]));
    return 0;
}

}  // namespace ga::app::tools
