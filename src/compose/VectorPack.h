// ================================================================================================
//  VectorPack - M6p: the survey order's manuscript reader. Lossless vector layers whose every
//  vertex carries its Visvalingam importance (sqrt of the wedge-product effective area, in
//  meters) -- so ONE structure serves every LOD: Query(tolMeters) filters vertices whose
//  importance >= tol and returns exactly the decimation VW would have produced, without a
//  single precomputed level. Points/tracks/depth-profiles ride the same format (kind + flag
//  fields); ships, wildlife, and planes are point/track layers with attributes in sidecars.
// ================================================================================================
#pragma once

#include "core/Gpu.h"

#include <string>
#include <vector>

namespace ga {

class VectorPack {
public:
    struct Vert {
        float lon, lat, imp;   // degrees, degrees, meters-of-detail
    };
    struct Layer {
        std::string name;
        uint8_t kind = 0;      // 0 polyline, 1 polygon, 2 points
        uint8_t flags = 0;     // 1 has z, 2 has t (reserved; parsed, skipped)
        std::vector<uint32_t> polyStart;   // offsets into verts, +1 sentinel at end
        std::vector<Vert> verts;
    };

    bool Load(const std::string& path);
    const Layer* Find(const std::string& name) const;

    // Line-list segments (lon,lat pairs, two verts per segment) at the given tolerance:
    // endpoints always survive (importance +inf), interior vertices by the wedge filter.
    // The count at tol 0 is the LOSSLESS geometry, bit-for-bit what the harvester read.
    std::vector<float> Segments(const Layer& layer, float tolMeters) const;

    const std::vector<Layer>& Layers() const { return m_layers; }

private:
    std::vector<Layer> m_layers;
};

}  // namespace ga
