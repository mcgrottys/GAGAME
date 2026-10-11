// ================================================================================================
//  BuildingLod - THE FOLDED TREE OF BUILDINGS (docs/BUILDING_LOD.md).
//
//  Every composed solid of the stack becomes its moment box (compose/BuildingMoments.h) and lives
//  in ONE node of a quadtree on the harvest's own degrees: quads of QuadDeg(L) = 0.05 x 2^L for
//  L = kLmin..kLmax (87 m to 89 km), the node chosen by the box's size (the smallest quad at least
//  four of its radii across: Ulrich's loose tree) and its centroid. Every node also keeps the
//  FOLD of what lies under it: the moments of all its descendants' boxes summed (BuildingMoments
//  FOLD + FRAME), as one moment box. So a node can stand for everything beneath it with one box
//  that keeps their total volume, centroid, height and heading.
//
//  The reader (scene/BuildingLayer) walks the tree from the roots: a node smaller than a few
//  pixels draws as its fold; otherwise its own buildings are drawn one by one where each covers a
//  pixel (folded where they do not), and its children are visited. Nothing is dropped, every
//  choice is per node by its own distance (no ring, no tile edge), and the boxes drawn are bounded
//  by the screen's pixels, not by the city's size.
//
//  On disk, a folder: lod.json (the manifest) and per level L: N<L>.bin (nodes), B<L>.bin (the
//  nodes' own buildings, in node order) and P<L>.idx (pages). A PAGE is the unit read: kPageX
//  quads across and PageY(L) down (a page never crosses a build band), its nodes sorted by
//  (y, x). Pages sorted by (py, px).
// ================================================================================================
#pragma once

#include "compose/BuildingSolids.h"
#include "compose/RoadWays.h"

#include <cstdint>
#include <string>
#include <vector>

namespace ga {

#pragma pack(push, 1)
struct LodBox {             // 32 bytes: a FOLD placed on the planet -- the spread of its mass, and
                            // how much of the spread's rectangle its footprints cover
    int32_t lat7, lon7;     // centroid, degrees x 1e7
    float zc;               // centroid height above the ground (m)
    float hz, a1, a2;       // half-extents: up, and the spread's long and short, sqrt(3 lambda) (m)
    float heading;          // radians, the long axis from east toward north
    float cover;            // footprint area / (4 a1 a2): the volume is 8 a1 a2 hz cover
};
struct LodBuilding {        // 20 bytes: one building's own box
    int32_t lat7, lon7;
    uint16_t zc, hz, a1, a2;   // half floats (m)
    int16_t heading;        // radians x 1e4
    int8_t dcx, dcy;        // its detail cell (the streaming's 0.05 deg cell, by its first point) minus
                            // the centroid's cell
};
struct LodNode {            // 88 bytes
    int32_t x, y;           // the quad: [x, x+1) x [y, y+1) of QuadDeg(L)
    int64_t ownFirst;       // its own buildings in B<L>.bin
    uint32_t ownCount;
    float ownRhoMin;        // the smallest own building's circumscribed radius (m)
    LodBox own;             // the fold of its own buildings (hz 0 = none)
    LodBox desc;            // the fold of every descendant's (hz 0 = none)
};
struct LodPage {            // 56 bytes (GALOD04; a GALOD03 page is the first 40, with no shapes)
    int32_t py, px;
    int64_t nodeFirst, nodeCount;
    int64_t bldFirst, bldCount;
    int64_t shapeFirst, shapeBytes;   // its buildings' shapes in S<L>.bin, one record each, in order
};
#pragma pack(pop)

namespace lod {
inline constexpr int kLmin = -6, kLmax = 4;   // 87 m .. 89 km quads (at the equator, north-south)
inline constexpr int kLevels = kLmax - kLmin + 1;
inline constexpr int kPageX = 32;
inline constexpr double kDetailDeg = 0.05;    // the streaming's cell (BuildingLayer::kCellDeg)
inline constexpr double kMetresPerDeg = 111195.0;
inline double QuadDeg(int L) { return kDetailDeg * (L >= 0 ? double(1 << L) : 1.0 / double(1 << -L)); }
// A band is one root quad high; a page never crosses one.
inline int PageY(int L) { const int rows = 1 << (kLmax - L); return rows < 32 ? rows : 32; }
// The level a box of circumscribed radius rho lives at: the smallest quad four radii across.
int LevelOf(double rho);
}  // namespace lod

struct LodBuildStats {
    uint64_t cells = 0, solids = 0, kept = 0, nodes[lod::kLevels] = {}, owned[lod::kLevels] = {};
    uint64_t ways = 0, ribbons = 0, tunnels = 0;   // the roads: ways read, pieces filed, tunnels left out
    double seconds = 0.0;
};
// THE ROADS IN THE SAME TREE (docs/ROADS.md): a way of a roads harvest (compose/RoadWays.h) is
// filed as RIBBON pieces beside the buildings -- each piece's mass the sum of its segments' slabs
// (a rectangle of the road's width, from `kerb` under the ground to `kerb` over it, through the
// prisms' own PrismMoments), so it folds into a node exactly as a building does and sits at the
// level its length puts it; its record the polyline (BuildingShape.h EncodeRibbon). The width is
// the file's where tagged, else lanes x laneWidth, else defaultLanes x laneWidth; a path class
// (rank 255: footway, path, cycleway, track, steps) is pathWidth. A tunnel is not filed: nothing of
// it stands on the ground. Every number here is the scene's declared assumption (layer.buildings).
struct RoadRibbonDefaults {
    double laneWidth = 3.5, defaultLanes = 2.0, pathWidth = 2.0, kerb = 0.12;
};
// THE PASS: every cell of the stack (inside lon0..lon1 x lat0..lat1 when given), composed and boxed
// on `threads` workers, root band by root band, into `dir`, the roads' pieces beside them. False
// (with `log`) on a refusal.
bool BuildBuildingLod(const BuildingStack& stack, const std::vector<RoadFile>& roads, const RoadRibbonDefaults& rd,
                      const std::string& dir, const double* box, int threads, LodBuildStats* stats, std::string* log);

// The tree open for reading: each level's page index in memory, a page's nodes and buildings read
// by seek (const, its own handle: pool threads read pages at once).
class BuildingLodFile {
public:
    bool Open(const std::string& dir, std::string* why);
    bool Valid() const { return m_ok; }
    const LodPage* Find(int L, int px, int py) const;
    bool Read(int L, const LodPage& p, std::vector<LodNode>& nodes, std::vector<LodBuilding>& blds) const;
    // GALOD04: the page's shape records (compose/BuildingShape.h), one a building in B order.
    bool ReadShapes(int L, const LodPage& p, std::vector<uint8_t>& shapes) const;
    bool HasShapes() const { return m_format >= 4; }
    size_t Pages(int L) const { return m_idx[L - lod::kLmin].size(); }

private:
    bool m_ok = false;
    int m_format = 0;
    std::string m_dir;
    std::vector<LodPage> m_idx[lod::kLevels];
};

// Decoding, shared by the reader and the gate.
void LodUnpack(const LodBuilding& b, double& lat, double& lon, double& zc, double& hz, double& a1, double& a2,
               double& heading, int& cx, int& cy);
// A box's own moments in a frame about (latc, lonc): x east, y north (metres), z up. A FOLD's (a1, a2
// its spread, `cover` below 1) is the mass of its footprints spread so: volume 8 a1 a2 hz cover,
// variance a^2/3 -- exact, where the area-kept box would fold a narrower mass than the real one.
struct Moments;
Moments LodBoxMoments(double lat, double lon, double zc, double hz, double a1, double a2, double heading,
                      double latc, double lonc, double cover = 1.0);

}  // namespace ga
