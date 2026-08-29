#include "compose/VectorPack.h"

#include <cstring>
#include <fstream>

namespace ga {

bool VectorPack::Load(const std::string& path) {
    std::ifstream f(path, std::ios::binary);
    if (!f) {
        Log("[vectors] no %s (run: py -3 harvester\\harvest_vectors.py)", path.c_str());
        return false;
    }
    std::vector<uint8_t> d((std::istreambuf_iterator<char>(f)),
                           std::istreambuf_iterator<char>());
    if (d.size() < 8 || memcmp(d.data(), "VPK1", 4) != 0) return false;
    uint32_t layerCount;
    memcpy(&layerCount, d.data() + 4, 4);
    size_t off = 8;
    auto need = [&](size_t n) { return off + n <= d.size(); };
    for (uint32_t li = 0; li < layerCount; ++li) {
        if (!need(2)) return false;
        uint16_t nameLen;
        memcpy(&nameLen, d.data() + off, 2);
        off += 2;
        if (!need(nameLen + 6u)) return false;
        Layer L;
        L.name.assign(reinterpret_cast<const char*>(d.data() + off), nameLen);
        off += nameLen;
        L.kind = d[off];
        L.flags = d[off + 1];
        uint32_t polyCount;
        memcpy(&polyCount, d.data() + off + 2, 4);
        off += 6;
        const uint32_t stride = 3 + ((L.flags & 1) ? 1 : 0) + ((L.flags & 2) ? 1 : 0);
        for (uint32_t p = 0; p < polyCount; ++p) {
            if (!need(4)) return false;
            uint32_t c;
            memcpy(&c, d.data() + off, 4);
            off += 4;
            if (!need(static_cast<size_t>(c) * stride * 4)) return false;
            L.polyStart.push_back(static_cast<uint32_t>(L.verts.size()));
            for (uint32_t i = 0; i < c; ++i) {
                Vert v;
                memcpy(&v, d.data() + off, 12);   // lon, lat, imp (z/t skipped)
                off += stride * 4;
                L.verts.push_back(v);
            }
        }
        L.polyStart.push_back(static_cast<uint32_t>(L.verts.size()));
        m_layers.push_back(std::move(L));
    }
    size_t verts = 0;
    for (const auto& L : m_layers) verts += L.verts.size();
    Log("[vectors] %zu layers, %zu vertices, lossless with per-vertex wedge importance",
        m_layers.size(), verts);
    return true;
}

const VectorPack::Layer* VectorPack::Find(const std::string& name) const {
    for (const auto& L : m_layers) {
        if (L.name == name) return &L;
    }
    return nullptr;
}

std::vector<float> VectorPack::Segments(const Layer& layer, float tolMeters) const {
    std::vector<float> out;
    for (size_t p = 0; p + 1 < layer.polyStart.size(); ++p) {
        float px = 0, py = 0;
        bool have = false;
        for (uint32_t i = layer.polyStart[p]; i < layer.polyStart[p + 1]; ++i) {
            const Vert& v = layer.verts[i];
            if (v.imp < tolMeters) continue;   // the wedge filter: below tolerance, decimated
            if (have) {
                out.push_back(px);
                out.push_back(py);
                out.push_back(v.lon);
                out.push_back(v.lat);
            }
            px = v.lon;
            py = v.lat;
            have = true;
        }
    }
    return out;
}

}  // namespace ga
