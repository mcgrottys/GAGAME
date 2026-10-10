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

#include <cstdint>
#include <string>
#include <vector>

namespace ga {

#pragma pack(push, 1)
struct LodBox {             // 28 bytes: a moment box placed on the planet
    int32_t lat7, lon7;     // centroid, degrees x 1e7
    float zc;               // centroid height above the ground (m)
    float hz, a1, a2;       // half-extents: up, long, short (m)
    float heading;          // radians, the long axis from east toward north
};
struct LodBuilding {        // 20 bytes: one building's own box
    int32_t lat7, lon7;
    uint16_t zc, hz, a1, a2;   // half floats (m)
    int16_t heading;        // radians x 1e4
    int8_t dcx, dcy;        // its detail cell (the streaming's 0.05 deg cell, by its first point) minus
                            // the centroid's cell
};
struct LodNode {            // 80 bytes
    int32_t x, y;           // the quad: [x, x+1) x [y, y+1) of QuadDeg(L)
    int64_t ownFirst;       // its own buildings in B<L>.bin
    uint32_t ownCount;
    float ownRhoMin;        // the smallest own building's circumscribed radius (m)
    LodBox own;             // the fold of its own buildings (hz 0 = none)
    LodBox desc;            // the fold of every descendant's (hz 0 = none)
};
struct LodPage {            // 40 bytes
    int32_t py, px;
    int64_t nodeFirst, nodeCount;
    int64_t bldFirst, bldCount;
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
    double seconds = 0.0;
};
// THE PASS: every cell of the stack (inside lon0..lon1 x lat0..lat1 when given), composed and boxed
// on `threads` workers, root band by root band, into `dir`. False (with `log`) on a refusal.
bool BuildBuildingLod(const BuildingStack& stack, const std::string& dir, const double* box, int threads,
                      LodBuildStats* stats, std::string* log);

// The tree open for reading: each level's page index in memory, a page's nodes and buildings read
// by seek (const, its own handle: pool threads read pages at once).
class BuildingLodFile {
public:
    bool Open(const std::string& dir, std::string* why);
    bool Valid() const { return m_ok; }
    const LodPage* Find(int L, int px, int py) const;
    bool Read(int L, const LodPage& p, std::vector<LodNode>& nodes, std::vector<LodBuilding>& blds) const;
    size_t Pages(int L) const { return m_idx[L - lod::kLmin].size(); }

private:
    bool m_ok = false;
    std::string m_dir;
    std::vector<LodPage> m_idx[lod::kLevels];
};

// Decoding, shared by the reader and the gate.
void LodUnpack(const LodBuilding& b, double& lat, double& lon, double& zc, double& hz, double& a1, double& a2,
               double& heading, int& cx, int& cy);
// A box's own moments in a frame about (latc, lonc): x east, y north (metres), z up.
struct Moments;
Moments LodBoxMoments(double lat, double lon, double zc, double hz, double a1, double a2, double heading,
                      double latc, double lonc);

}  // namespace ga
