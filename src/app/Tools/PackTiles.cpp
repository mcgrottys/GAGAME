// PackTiles - --pack-tiles.
// Moved verbatim from main.cpp (M12 step 1b); declared in app/Tools.h.
#include "app/Tools.h"

#include "compose/TileArchive.h"
#include "core/Common.h"

#include <cstdint>

namespace ga::app::tools {

// M9ah: pack the composed cache into per-realization archives and exit. No device, no
// scene -- this is a disk-to-disk job. The loose tiles are kept: the archive is derived,
// and the compositor keeps writing loose files as it paints, so anything painted after a
// pack must still be findable the old way.
int RunPackTiles(const Options&) {
    const char* jobs[][2] = {
        {"earth.color", "cube16k"},
        {"earth.color", "window_z14_1263360_1538048"},
        {"earth.color", "window_z17_10168820_12344774"},
        {"earth.color", "window_z19_40699567_49405858"},
        {"earth.height", "cube16k"},
        {"earth.height", "window_z14_1263360_1538048"},
        {"mars.height", "cube16k"},
    };
    uint32_t total = 0;
    for (const auto& j : jobs) total += TileArchive::Pack(j[0], j[1]);
    Log("[tilearch] %u tiles packed across %zu realizations", total,
        sizeof(jobs) / sizeof(jobs[0]));
    return 0;
}

}  // namespace ga::app::tools
