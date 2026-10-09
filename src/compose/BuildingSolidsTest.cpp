// ================================================================================================
//  BuildingSolidsTest - the [buildings] block of --selftest: the stack's laws, each planted and
//  caught on solids made here (no file, no fetch).
//
//  A lower source of seven solids and an upper one of three, each placed so exactly one law decides
//  its fate: a footprint above covers a centre (10 goes), a `remove` ring clears (11 goes), an id
//  above names it (12 goes, the upper 12 stands elsewhere), floors tagged (13 stands at 4 x level),
//  an outline holding a part (14 goes, part 15 stands at its own height), untagged (16 stands at the
//  default), and a footprint above that covers NO centre (17 stays: a source wins where it stands,
//  not near it).
// ================================================================================================
#include "compose/BuildingSolids.h"

#include "core/Common.h"

#include <cmath>
#include <cstdio>

namespace ga {

namespace {

// A square of side d degrees with its south-west corner at (lon, lat).
BuildingSolid Square(int64_t id, double lon, double lat, double d, uint8_t kind = 0) {
    BuildingSolid s;
    s.id = id;
    s.kind = kind;
    s.rings.push_back({lon, lat, lon + d, lat, lon + d, lat + d, lon, lat + d});
    s.outer.push_back(1);
    return s;
}

}  // namespace

bool RunBuildingSelfTest() {
    bool ok = true;
    auto check = [&ok](bool c, const char* what) {
        Log("[buildings] %s %s", c ? "ok  " : "FAIL", what);
        ok &= c;
    };
    const double u = 1e-4;   // ~10 m

    std::vector<BuildingSolid> lower;
    lower.push_back(Square(10, 0 * u, 0, u));
    lower.push_back(Square(11, 2 * u, 0, u));
    lower.push_back(Square(12, 4 * u, 0, u));
    lower.push_back(Square(13, 6 * u, 0, u));
    lower.back().levels = 4.0f;
    lower.push_back(Square(14, 8 * u, 0, 2 * u));                // an outline...
    lower.push_back(Square(15, 8.5 * u, 0.5 * u, 0.5 * u, 1));   // ...holding a part
    lower.back().height = 30.0f;
    lower.push_back(Square(16, 12 * u, 0, u));
    lower.push_back(Square(17, 14 * u, 0, u));

    std::vector<BuildingSolid> upper;
    upper.push_back(Square(0, -0.2 * u, -0.2 * u, 1.4 * u));     // covers 10's centre
    upper.back().height = 20.0f;
    upper.push_back(Square(0, 1.9 * u, -0.1 * u, 1.2 * u));      // clears 11
    upper.back().remove = true;
    upper.push_back(Square(12, 4 * u, 3 * u, u));                // names 12, stands elsewhere
    upper.back().height = 9.0f;
    upper.push_back(Square(0, 14.6 * u, 0.6 * u, 0.3 * u));      // inside 17, not over its centre
    upper.back().height = 5.0f;

    uint64_t idA = 0, idB = 0;
    const BuildingDefaults d{3.0, 6.0};
    const std::vector<BuildingSolid> out = ComposeBuildings({lower, upper}, d, &idA);
    auto find = [&out](int64_t id, double lon) -> const BuildingSolid* {
        for (const BuildingSolid& s : out) {
            if (s.id == id && std::abs(s.rings[0][0] - lon) < 1e-9) return &s;
        }
        return nullptr;
    };
    check(out.size() == 7, "seven solids drawn (3 of the upper's, 4 of the lower's)");
    check(!find(10, 0), "10: a footprint above covers its centre");
    check(!find(11, 2 * u), "11: a remove ring clears it and draws nothing itself");
    // Both 12s share a west edge; the lower stands at lat 0, the upper at 3u.
    int n12 = 0;
    const BuildingSolid* s12 = nullptr;
    for (const BuildingSolid& s : out) {
        if (s.id == 12) {
            ++n12;
            s12 = &s;
        }
    }
    check(n12 == 1 && std::abs(s12->rings[0][1] - 3 * u) < 1e-12, "12: the id above names it");
    check(s12 && s12->top == 9.0, "12 above stands at 9 m");
    const BuildingSolid* s13 = find(13, 6 * u);
    check(s13 && s13->top == 12.0 && s13->bottom == 0.0, "13: 4 floors x 3 m");
    check(!find(14, 8 * u), "14: an outline holding a part is drawn by its parts");
    const BuildingSolid* s15 = find(15, 8.5 * u);
    check(s15 && s15->top == 30.0, "15: the part at its own 30 m");
    const BuildingSolid* s16 = find(16, 12 * u);
    check(s16 && s16->top == 6.0, "16: untagged at the scene's default");
    check(find(17, 14 * u) != nullptr, "17: a footprint above that covers no centre leaves it");
    // The identity moves with a declared assumption and with nothing else.
    ComposeBuildings({lower, upper}, d, &idB);
    check(idA == idB, "identity repeats on the same inputs");
    ComposeBuildings({lower, upper}, {3.0, 7.0}, &idB);
    check(idA != idB, "identity moves with the declared default");
    Log("[buildings] selftest %s", ok ? "PASS" : "FAIL");
    return ok;
}

}  // namespace ga
