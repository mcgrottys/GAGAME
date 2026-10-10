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

#include <array>
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
    int source = -1;                  // the stack's index of the source that drew it: by Compose
};

struct BuildingSourceSpec {
    std::string name, path;   // a .buildings.json manifest, or a .geojson
    double over = 0.0;
};

struct BuildingDefaults {
    double levelHeight = 3.0;     // metres a tagged floor stands for
    double defaultHeight = 6.0;   // metres for a building with neither height nor floors
};

// The stack, bottom to top, composed by the laws above into the solids to draw, bottom and top
// resolved, each naming its source. `identity` is FNV-1a over the inputs that decided them.
std::vector<BuildingSolid> ComposeBuildings(std::vector<std::vector<BuildingSolid>> stack,
                                            const BuildingDefaults& d, uint64_t* identity = nullptr);

// THE STACK, OPEN AND READ BY BOX: what streams. A harvest keeps only its cell index in memory (the
// manifest's "cells", or at planet scale the binary sidecar its "cellIndex" names: int32 ix, int32
// iy, int64 offset, int64 count) and finds a box's cells by binary search on (iy, ix), then reads
// their records by seek; a GeoJSON (an owner's file, small) is read whole once. A
// box is composed from every source's solids in the box grown by `margin` degrees, so a footprint
// just over the edge still covers, and the box keeps the solids whose outer ring's first point lies
// inside it: each solid is drawn by exactly one box. Compose is const and opens its own file
// handles, so boxes may be composed on several threads at once.
class BuildingStack {
public:
    // Every source of `specs` (any order: sorted here by `over`, file order breaking a tie); a
    // refused one is named in `log` and the stack goes on without it.
    void Open(std::vector<BuildingSourceSpec> specs, const BuildingDefaults& d, std::string* log);
    std::vector<BuildingSolid> Compose(double lon0, double lat0, double lon1, double lat1, double margin,
                                       uint64_t* identity = nullptr) const;
    size_t Sources() const { return m_src.size(); }
    // Every cell of `cellDeg` (as ix, iy: [ix, ix+1) x [iy, iy+1)) that any source holds a solid in,
    // sorted by (iy, ix): a harvest's own index where its grain is cellDeg, a GeoJSON's solids by
    // their first point. What a pass over the whole stack walks (the LOD pyramid's tool).
    std::vector<std::pair<int, int>> Cells(double cellDeg) const;
    const std::string& Name(size_t k) const { return m_src[k].name; }

private:
    struct Source {
        std::string name, bin;
        double cellDeg = 0.0;
        std::vector<std::array<int64_t, 4>> cells;   // iy, ix, offset, count, sorted by (iy, ix)
        std::vector<BuildingSolid> whole;            // a GeoJSON, read once
        bool harvest = false;
    };
    void Read(const Source& s, double lon0, double lat0, double lon1, double lat1,
              std::vector<BuildingSolid>& out) const;
    std::vector<Source> m_src;
    BuildingDefaults m_d;
};

// The [buildings] block of --selftest (compose/BuildingSolidsTest.cpp): the stack's laws, planted.
bool RunBuildingSelfTest();

// Point in polygon over one ring (even-odd), degrees.
bool RingHolds(const std::vector<double>& ring, double lon, double lat);
// The area centre of a ring (degrees), its vertex mean where the area is degenerate.
void RingCentre(const std::vector<double>& ring, double& lon, double& lat);

}  // namespace ga
