// ================================================================================================
//  BuildingSolids - BUILDINGS ARE SOLIDS, AND A STACK OF SOURCES OF THEM COMPOSES LIKE THE EARTH'S.
//
//  A solid is a prism: its rings (lon/lat degrees, doubles, the first ring outer), from a bottom to
//  a top in metres ABOVE THE GROUND at its footprint -- OSM's own datum for `height`, carried by
//  the harvest's manifest as "datum": "ground". The ground is not a number here: the reader that
//  stands a solid on the earth asks the composed height stack for it. One record serves the picture
//  now (scene/BuildingLayer) and the water's sparse voxels later: the solid, not a bake of it.
//
//  THE SOURCES (the scene's `sources`, kind "buildings"), each one file:
//    a harvest manifest   <stem>.buildings.json + .bin (harvester/harvest_buildings.py): OSM ways
//                         and multipolygons, heights normalized to metres, the untagged left NaN
//    a GeoJSON            Polygon / MultiPolygon features, the owner's own: properties `height`,
//                         `min_height`, `building:levels`, `building:min_level` (metres and counts,
//                         as OSM spells them), `id` (an OSM way id to replace; a relation's negated),
//                         and `remove`: true for a footprint that only clears what lies beneath.
//
//  THE STACK (StackOrder's first key, `over`, ascending upward; file order breaks a tie). A source
//  wins wherever it stands, as a higher height or photo does: every solid beneath whose footprint's
//  centre lies inside a footprint above, or whose id a solid above names, is not drawn. So a hand
//  model replaces the OSM block it sits on, a `remove` ring clears a lot, and a whole town's file
//  can be laid over the state's without touching it.
//
//  THE HEIGHT (normalize -> compose). A tagged height is the file's. An untagged one is the scene's
//  DECLARED assumption: levels x levelHeight where the floors are tagged, else defaultHeight -- one
//  place, visible as an assumption, replaced the day a measured height (lidar, Overture) is a source.
//  Within one source an outline whose footprint holds a building:part is not drawn: the parts are
//  the building (OSM's Simple 3D Buildings rule).
// ================================================================================================
#pragma once

#include <cstdint>
#include <limits>
#include <string>
#include <vector>

namespace ga {

struct BuildingSolid {
    int64_t id = 0;     // OSM way id; a relation's negated; 0 = none (a hand feature)
    uint8_t kind = 0;   // 0 building, 1 building:part
    static constexpr float kUntagged = std::numeric_limits<float>::quiet_NaN();
    float height = kUntagged, minHeight = kUntagged, levels = kUntagged, minLevel = kUntagged;
    bool remove = false;   // a footprint that only clears what lies beneath it
    // Rings in degrees (lon, lat); rings[0] the outer, the rest inner unless outer[i] says so.
    std::vector<std::vector<double>> rings;   // each: lon0, lat0, lon1, lat1, ...
    std::vector<uint8_t> outer;
    double bottom = 0.0, top = 0.0;   // metres above the ground: resolved by Compose

};

struct BuildingSourceSpec {
    std::string name, path;   // a .buildings.json manifest, or a .geojson
    double over = 0.0;
};

struct BuildingDefaults {
    double levelHeight = 3.0;     // metres a tagged floor stands for
    double defaultHeight = 6.0;   // metres for a building with neither height nor floors
};

// Every solid of one source whose outer ring's first point lies within radiusM of (latDeg, lonDeg)
// (radiusM <= 0: the whole file). False, and why, when the file is refused.
bool LoadBuildingSource(const std::string& path, double latDeg, double lonDeg, double radiusM,
                        std::vector<BuildingSolid>& out, std::string* why);

// The stack, bottom to top, composed by the laws above into the solids to draw, bottom and top
// resolved. `identity` is FNV-1a over the inputs that decided them, for a cache key; `drawn[k]`,
// how many of source k's solids were drawn (the instrument that sees which claim won where).
std::vector<BuildingSolid> ComposeBuildings(std::vector<std::vector<BuildingSolid>> stack,
                                            const BuildingDefaults& d, uint64_t* identity = nullptr,
                                            std::vector<size_t>* drawn = nullptr);

// The [buildings] block of --selftest (compose/BuildingSolidsTest.cpp): the stack's laws, planted.
bool RunBuildingSelfTest();

// Point in polygon over one ring (even-odd), degrees.
bool RingHolds(const std::vector<double>& ring, double lon, double lat);
// The area centre of a ring (degrees), its vertex mean where the area is degenerate.
void RingCentre(const std::vector<double>& ring, double& lon, double& lat);

}  // namespace ga
