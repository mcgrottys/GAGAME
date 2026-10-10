// ================================================================================================
//  BuildingLod - THE SIZE-STRATIFIED PYRAMID OF MOMENT BOXES (docs/BUILDING_LOD.md, step 2).
//
//  Every composed solid of the stack becomes its moment box (compose/BuildingMoments.h) and lives
//  at ONE level, the one its size picks: k = floor(log2(rho / rho0)), clamped to kMaxLevel. Levels
//  0 and 1 reach no farther than 8 km, inside any detail radius, so they are drawn only as the full
//  prisms of the detail cells about the eye and the pyramid keeps levels 2 and up (Massachusetts:
//  level 1 alone was 58% of the solids). Level k is a grid of tiles TileDeg(k) on a side, so each
//  level holds about the same number of tiles within its own reach. A record is filed by its DETAIL cell (the 0.05 deg cell the streaming draws it in,
//  chosen by its outer ring's first point), so a far box and its detail prisms always agree on
//  where the building is, and the layer can skip exactly the boxes whose detail cell is drawn.
//
//  On disk, a folder: lod.json (the manifest: format, rho0, levels, the stack's sources) and per
//  level k, L<k>.bin (records) and L<k>.idx (tiles: ty, tx, offset, count; sorted by (ty, tx)).
//  Records within a tile are sorted by their detail cell (cy, cx).
// ================================================================================================
#pragma once

#include "compose/BuildingSolids.h"

#include <cstdint>
#include <string>
#include <vector>

namespace ga {

#pragma pack(push, 1)
struct LodRecord {          // 36 bytes
    int32_t lat7, lon7;     // the box's centroid, degrees x 1e7
    int32_t cx, cy;         // its detail cell (0.05 deg): [cx, cx+1) x [cy, cy+1)
    float zc;               // the centroid's height above the ground (m)
    float hz;               // vertical half-extent (m)
    float a1, a2;           // horizontal half-extents (m), long then short
    float heading;          // radians, the long axis from east toward north
};
struct LodTile {            // 24 bytes
    int32_t ty, tx;
    int64_t offset, count;  // records, in L<k>.bin
};
#pragma pack(pop)

namespace lod {
inline constexpr int kMinLevel = 2, kMaxLevel = 8;   // level 1 reaches 8 km: inside any detail radius
inline constexpr double kDetailDeg = 0.05;   // the streaming's cell (BuildingLayer::kCellDeg)
// A level's tile: the detail cell doubled until it holds about a quarter of the level's reach.
inline double TileDeg(int k) { return k <= 2 ? kDetailDeg : kDetailDeg * double(1 << (k - 2)); }
}  // namespace lod

// THE PASS: every cell of the stack (inside lon0..lon1 x lat0..lat1 when given), composed and
// boxed on `threads` workers, latitude band by band (TileDeg(kMaxLevel) a band, so memory holds
// one band), into `dir`. Returns false (with `log`) on a refusal.
struct LodBuildStats {
    uint64_t cells = 0, solids = 0, kept[lod::kMaxLevel + 1] = {}, tiles[lod::kMaxLevel + 1] = {};
    double seconds = 0.0;
};
bool BuildBuildingLod(const BuildingStack& stack, const std::string& dir, double rho0, const double* box,
                      int threads, LodBuildStats* stats, std::string* log);

// The pyramid open for reading: the manifest and every level's tile index in memory, records read
// by seek. Read is const and opens its own handle (pool threads read tiles at once).
class BuildingLodFile {
public:
    bool Open(const std::string& dir, std::string* why);
    bool Valid() const { return m_ok; }
    double Rho0() const { return m_rho0; }
    // The tile (tx, ty) of level k, or nullptr.
    const LodTile* Find(int k, int tx, int ty) const;
    std::vector<LodRecord> Read(int k, const LodTile& t) const;
    size_t Tiles(int k) const { return (k >= 0 && k <= lod::kMaxLevel) ? m_idx[k].size() : 0; }

private:
    bool m_ok = false;
    double m_rho0 = 4.0;
    std::string m_dir;
    std::vector<LodTile> m_idx[lod::kMaxLevel + 1];
};

}  // namespace ga
