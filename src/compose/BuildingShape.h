// ================================================================================================
//  BuildingShape - ONE BUILDING'S OWN FORM, stored beside its box in the folded tree (GALOD04,
//  docs/BUILDING_LOD.md): the footprint the drawing extrudes, in place of the box the box drew.
//
//  A record is the solid in the tangent plane at its own centroid (the LodBuilding's lat7/lon7):
//  its rings as east/north offsets, normalised so every outer ring runs counter-clockwise and every
//  hole clockwise -- then a wall's outward normal is always the right of its edge, and the drawing
//  needs no winding of its own -- its bottom and top above the ground, and its roof as triangles
//  over the rings' own vertices (the holes bridged in, ear-clipped: the near prisms' own roof).
//
//  Layout (little-endian, a multiple of 4 bytes, self-sized so a page's records are read in order):
//      ShapeHead                        16 B
//      uint16 ringLen[nRings]           vertices per ring, the rings one after another
//      int16  xy[2 nVerts]              east, north: decimetres (kMetres: metres) from the centroid
//      uint16 tri[3 nTris]              the roof, indices into xy
//      pad to 4
// ================================================================================================
#pragma once

#include "compose/BuildingSolids.h"

#include <cstdint>
#include <vector>

namespace ga {

#pragma pack(push, 1)
struct ShapeHead {
    uint16_t nVerts, nTris;
    uint8_t nRings, flags;   // flags: kPart, kMetres
    uint16_t pad;
    float bottom, top;       // metres above the ground under the centroid
};
#pragma pack(pop)

namespace shape {
inline constexpr uint8_t kPart = 1;     // a building:part
inline constexpr uint8_t kMetres = 2;   // xy in metres, not decimetres: a footprint over 3.2 km
inline size_t Bytes(const ShapeHead& h) {
    const size_t n = sizeof(ShapeHead) + 2u * h.nRings + 4u * h.nVerts + 6u * h.nTris;
    return (n + 3u) & ~size_t(3);
}
}  // namespace shape

// The solid's record, about (latc, lonc) in degrees. False (nothing appended) for a solid whose
// rings are degenerate or too many vertices for a record (the drawing keeps its box).
bool EncodeShape(const BuildingSolid& s, double latc, double lonc, std::vector<uint8_t>& out);

// A record decoded: the rings' vertices in metres east/north of the centroid.
struct ShapeView {
    ShapeHead head{};
    const uint16_t* ringLen = nullptr;
    const int16_t* xy = nullptr;
    const uint16_t* tri = nullptr;
    float unit = 0.1f;   // metres per xy step
};
// The record at p (inside [p, end)); false if it does not fit.
bool ReadShape(const uint8_t* p, const uint8_t* end, ShapeView& v);

// The roof of rings in a plane, outer counter-clockwise and holes clockwise (outer[i] marks the
// outer ones): triangles as indices into the rings' concatenated vertices. Shared by the record and
// the near prisms.
void TriangulateRoof(const std::vector<std::vector<double>>& xy, const std::vector<uint8_t>& outer,
                     std::vector<uint32_t>& tris);

}  // namespace ga
