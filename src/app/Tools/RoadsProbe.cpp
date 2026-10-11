// RoadsProbe - `--tool roads-probe:at=LAT,LON;km=R;[point=M;shape=M;]roads=PATH;bridges=PATH[;bridges=PATH...]`.
// One box of the road and clearance harvests (compose/RoadWays.h) read and described: the ways
// by highway / structure / form, the junction count, and every bridge way with the surveyed
// spans MatchBridges gives it within the reach of its survey (point= default 60 m, shape= default
// 5 m). No device, no scene data; exit 0 when every named
// file opened, 2 on a refusal or a bad argument.
#include "app/Tools.h"

#include "compose/RoadWays.h"
#include "core/Common.h"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <map>
#include <string>
#include <vector>

namespace ga::app::tools {

namespace {

const char* StructureName(RoadStructure s) {
    switch (s) {
        case RoadStructure::None: return "none";
        case RoadStructure::Bridge: return "bridge";
        case RoadStructure::Tunnel: return "tunnel";
        case RoadStructure::Ford: return "ford";
        case RoadStructure::Culvert: return "culvert";
        default: return "untagged";
    }
}
const char* FormName(RoadForm f) {
    static const char* k[] = {"carriageway", "dual", "ramp", "roundabout", "service", "parking-aisle",
                              "path", "track", "steps", "other"};
    const unsigned i = static_cast<unsigned>(f);
    return i < 10 ? k[i] : "untagged";
}
const char* SourceName(BridgeSource s) {
    switch (s) {
        case BridgeSource::Nbi: return "NBI";
        case BridgeSource::Enc: return "ENC";
        case BridgeSource::Osm: return "OSM";
        default: return "other";
    }
}

}  // namespace

int RunRoadsProbe(const std::string& args) {
    double lat = NAN, lon = NAN, km = 2.0, pointM = 60.0, shapeM = 5.0;
    std::string roads;
    std::vector<std::string> bridges;
    size_t at = 0;
    while (at <= args.size()) {
        const size_t semi = args.find(';', at);
        const std::string kv = args.substr(at, semi == std::string::npos ? std::string::npos : semi - at);
        at = semi == std::string::npos ? args.size() + 1 : semi + 1;
        const size_t eq = kv.find('=');
        if (eq == std::string::npos) continue;
        const std::string k = kv.substr(0, eq), v = kv.substr(eq + 1);
        if (k == "at") {
            char* end = nullptr;
            lat = std::strtod(v.c_str(), &end);
            if (end && *end == ',') {
                lon = std::strtod(end + 1, &end);
            } else {
                lat = NAN;
            }
        } else if (k == "point") {
            pointM = std::atof(v.c_str());
        } else if (k == "shape") {
            shapeM = std::atof(v.c_str());
        } else if (k == "km") {
            km = std::atof(v.c_str());
        } else if (k == "roads") {
            roads = v;
        } else if (k == "bridges") {
            bridges.push_back(v);
        } else {
            Log("[roads-probe] unknown key '%s'", k.c_str());
            return 2;
        }
    }
    if (std::isnan(lat) || !(km > 0.0) || (roads.empty() && bridges.empty())) {
        Log("[roads-probe] usage: --tool roads-probe:at=LAT,LON;km=R;roads=X.roads.json;bridges=Y.bridges.json");
        return 2;
    }
    const double dLat = km * 1000.0 / 110574.0;
    const double dLon = km * 1000.0 / (111320.0 * std::cos(lat * 3.14159265358979323846 / 180.0));
    const double lon0 = lon - dLon, lon1 = lon + dLon, lat0 = lat - dLat, lat1 = lat + dLat;
    Log("[roads-probe] box %.5f..%.5f E, %.5f..%.5f N (%.2f km about %.5f N, %.5f E)", lon0, lon1, lat0, lat1, km,
        lat, lon);
    int rc = 0;
    std::vector<RoadWay> ways;
    std::string why;
    if (!roads.empty()) {
        if (!LoadRoadSource(roads, lon0, lat0, lon1, lat1, ways, &why)) {
            Log("[roads-probe] roads '%s' refused: %s", roads.c_str(), why.c_str());
            rc = 2;
        }
    }
    std::vector<BridgeSpan> spans;
    for (const std::string& b : bridges) {
        const size_t before = spans.size();
        if (!LoadBridgeSource(b, lon0, lat0, lon1, lat1, spans, &why)) {
            Log("[roads-probe] bridges '%s' refused: %s", b.c_str(), why.c_str());
            rc = 2;
            continue;
        }
        for (size_t i = before; i < spans.size(); ++i) spans[i].sourceName = b.substr(b.find_last_of("/\\") + 1);
        Log("[roads-probe] %s: %zu spans", b.c_str(), spans.size() - before);
    }
    std::map<std::string, int> byHighway, byStructure, byForm;
    for (const RoadWay& w : ways) {
        ++byHighway[w.highwayTag];
        ++byStructure[StructureName(w.structure)];
        ++byForm[FormName(w.form)];
    }
    Log("[roads-probe] %zu ways", ways.size());
    for (const auto& [k, n] : byHighway) Log("[roads-probe]   highway %-16s %6d", k.c_str(), n);
    for (const auto& [k, n] : byStructure) Log("[roads-probe]   structure %-14s %6d", k.c_str(), n);
    for (const auto& [k, n] : byForm) Log("[roads-probe]   form %-19s %6d", k.c_str(), n);
    Log("[roads-probe] %zu junctions", JunctionsOf(ways).size());
    const std::vector<BridgeMatch> match = MatchBridges(ways, spans, pointM, shapeM);
    auto spanLine = [](const BridgeSpan& s) {
        char buf[640];
        std::snprintf(buf, sizeof(buf),
                      "%s %lld '%s' note '%s' origin '%s' (%s) %s: vertClr %g m, closed %g, open %g, horClr %g m, "
                      "category %u, year %u",
                      SourceName(s.source), static_cast<long long>(s.id), s.name.c_str(), s.note.c_str(),
                      s.origin.c_str(), s.sourceName.c_str(),
                      s.primitive == 0 ? "point" : s.primitive == 1 ? "line" : "area", s.vertClr, s.vertClrClosed,
                      s.vertClrOpen, s.horClr, s.category, s.yearBuilt);
        return std::string(buf);
    };
    // Each bridge way, with the spans it stands on.
    for (size_t i = 0; i < ways.size(); ++i) {
        const RoadWay& w = ways[i];
        if (w.structure != RoadStructure::Bridge) continue;
        char layer[8] = "-";
        if (w.levelFrom != kLevelUntagged) std::snprintf(layer, sizeof(layer), "%d", w.levelFrom);
        Log("[roads-probe] bridge way %lld '%s' ref '%s' layer %s highway %s rank %u, %zu points",
            static_cast<long long>(w.id), w.name.c_str(), w.ref.c_str(), layer, w.highwayTag.c_str(), w.rank,
            w.points.size());
        for (const BridgeMatch& b : match) {
            if (b.way != i) continue;
            Log("[roads-probe]     overlap %7.1f m  span %s", b.overlapM, spanLine(spans[b.span]).c_str());
        }
    }
    // Each span, with the ways on its deck in the law's order.
    size_t matched = 0;
    for (size_t si = 0; si < spans.size(); ++si) {
        const BridgeSpan& s = spans[si];
        std::string list;
        for (const BridgeMatch& b : match) {
            if (b.span != si) continue;
            char buf[128];
            std::snprintf(buf, sizeof(buf), "%s%lld (%s, %.1f m)", list.empty() ? "" : ", ",
                          static_cast<long long>(ways[b.way].id), ways[b.way].highwayTag.c_str(), b.overlapM);
            list += buf;
        }
        if (!list.empty()) ++matched;
        Log("[roads-probe] span %s at %.5f N %.5f E -> %s", spanLine(s).c_str(), s.parts[0][1], s.parts[0][0],
            list.empty() ? "no bridge way within reach" : list.c_str());
    }
    Log("[roads-probe] %zu of %zu spans pair with a bridge way (point reach %g m, shape reach %g m; %zu pairs)",
        matched, spans.size(), pointM, shapeM,
        match.size());
    return rc;
}

}  // namespace ga::app::tools
