#include "compose/GisStencil.h"

#include "core/Json.h"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <utility>

namespace ga {

bool GisStencil::ReadBin(const std::string& path, std::vector<Polyline>& out) {
    std::ifstream f(path, std::ios::binary);
    if (!f) return false;
    uint32_t n = 0;
    f.read(reinterpret_cast<char*>(&n), 4);
    out.reserve(n);
    for (uint32_t i = 0; i < n && f; ++i) {
        uint32_t c = 0;
        f.read(reinterpret_cast<char*>(&c), 4);
        if (!f || c > 20000000u) return false;
        Polyline line(c);
        f.read(reinterpret_cast<char*>(line.data()), static_cast<std::streamsize>(c) * 8);
        out.push_back(std::move(line));
    }
    return !out.empty();
}

bool GisStencil::Load(const std::string& jsonPath) {
    std::ifstream f(jsonPath, std::ios::binary);
    if (!f) {
        Log("[gis] no %s (run: py -3 harvester\\harvest_gis.py); stencil is height+graticule "
            "only",
            jsonPath.c_str());
        return false;
    }
    std::string text((std::istreambuf_iterator<char>(f)), std::istreambuf_iterator<char>());
    std::string err;
    const JsonValue v = JsonParser::Parse(text, &err);
    m_dir = jsonPath.substr(0, jsonPath.find_last_of("/\\") + 1);
    ReadBin(m_dir + v.Str("coast_ne"), m_coastNe);
    ReadBin(m_dir + v.Str("rivers_ne"), m_riversNe);
    ReadBin(m_dir + v.Str("coast_global"), m_coastGlob);
    Log("[gis] survey vectors: %zu NE coast lines, %zu NE rivers, %zu global coast lines (%s)",
        m_coastNe.size(), m_riversNe.size(), m_coastGlob.size(),
        v.Str("source", "GSHHG").c_str());
    m_ready = !m_coastNe.empty() || !m_coastGlob.empty();
    return m_ready;
}

}  // namespace ga
