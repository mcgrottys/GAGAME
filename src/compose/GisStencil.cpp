#include "compose/GisStencil.h"

#include "core/Json.h"

#include <fstream>

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
    m_maskNePath = v.Str("landmask_ne");
    m_maskNeDim = static_cast<uint32_t>(v.Num("landmask_ne_dim", 0));
    m_maskGlobPath = v.Str("landmask_global");
    m_maskGw = static_cast<uint32_t>(v.Num("landmask_global_w", 0));
    m_maskGh = static_cast<uint32_t>(v.Num("landmask_global_h", 0));
    Log("[gis] survey vectors: %zu NE coast lines, %zu NE rivers, %zu global coast lines (%s)",
        m_coastNe.size(), m_riversNe.size(), m_coastGlob.size(),
        v.Str("source", "GSHHG").c_str());
    return !m_coastNe.empty() || !m_coastGlob.empty();
}

void GisStencil::BuildMasks(Gpu& gpu) {
    auto loadRaw = [&](const std::string& rel, size_t expect,
                       std::vector<uint8_t>& out) -> bool {
        if (rel.empty()) return false;
        std::ifstream f(m_dir + rel, std::ios::binary);
        if (!f) return false;
        out.assign((std::istreambuf_iterator<char>(f)), std::istreambuf_iterator<char>());
        return out.size() == expect;
    };
    std::vector<uint8_t> raw;
    if (m_maskNeDim > 0 &&
        loadRaw(m_maskNePath, static_cast<size_t>(m_maskNeDim) * m_maskNeDim, raw)) {
        m_maskWin = gpu.CreateTexture2D(m_maskNeDim, m_maskNeDim, DXGI_FORMAT_R8_UNORM,
                                        D3D12_RESOURCE_FLAG_NONE,
                                        D3D12_RESOURCE_STATE_COPY_DEST,
                                        L"gis.landmaskWindow (GSHHG f parity fill)");
        gpu.UploadTexture(m_maskWin, raw.data(), m_maskNeDim);
        m_maskWin.srv = gpu.CreateSrv(m_maskWin.res.Get(), DXGI_FORMAT_R8_UNORM);
    }
    if (m_maskGw > 0 &&
        loadRaw(m_maskGlobPath, static_cast<size_t>(m_maskGw) * m_maskGh, raw)) {
        m_maskGlob = gpu.CreateTexture2D(m_maskGw, m_maskGh, DXGI_FORMAT_R8_UNORM,
                                         D3D12_RESOURCE_FLAG_NONE,
                                         D3D12_RESOURCE_STATE_COPY_DEST,
                                         L"gis.landmaskGlobal (GSHHG l parity fill)");
        gpu.UploadTexture(m_maskGlob, raw.data(), m_maskGw);
        m_maskGlob.srv = gpu.CreateSrv(m_maskGlob.res.Get(), DXGI_FORMAT_R8_UNORM);
    }
    m_ready = m_maskWin.Valid() || m_maskGlob.Valid();
    Log("[gis] land masks: window %s (%u^2), global %s (%ux%u) -- classification by SURVEY",
        m_maskWin.Valid() ? "ok" : "absent", m_maskNeDim,
        m_maskGlob.Valid() ? "ok" : "absent", m_maskGw, m_maskGh);
}

}  // namespace ga
