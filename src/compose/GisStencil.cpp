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

// Even-odd ray cast in window-pixel space (the mercator frame is conformal; rings convert
// once, texels test cheap).
static bool PointInRing(double x, double y, const std::vector<std::pair<double, double>>& r) {
    bool in = false;
    for (size_t i = 0, j = r.size() - 1; i < r.size(); j = i++) {
        if ((r[i].second > y) != (r[j].second > y) &&
            x < (r[j].first - r[i].first) * (y - r[i].second) /
                    (r[j].second - r[i].second + 1e-12) +
                r[i].first) {
            in = !in;
        }
    }
    return in;
}

void GisStencil::BuildMasks(Gpu& gpu, double orgPxX, double orgPxY, double sizePx) {
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
        const uint32_t dim = m_maskNeDim;
        // R8G8: r = mask (edits baked over survey), g = edit flag (the override is law).
        std::vector<uint8_t> rg(static_cast<size_t>(dim) * dim * 2);
        for (size_t i = 0; i < raw.size(); ++i) rg[i * 2] = raw[i];

        std::ifstream ef(m_dir + "edits.geojson", std::ios::binary);
        int nEdits = 0;
        if (ef) {
            std::string text((std::istreambuf_iterator<char>(ef)),
                             std::istreambuf_iterator<char>());
            std::string err;
            const JsonValue gj = JsonParser::Parse(text, &err);
            const double n14 = 16384.0 * 256.0;
            const double scale = dim / sizePx;
            if (const JsonValue* feats = gj.Get("features")) {
                for (const JsonValue& ft : feats->arr) {
                    const JsonValue* props = ft.Get("properties");
                    const JsonValue* geom = ft.Get("geometry");
                    if (!props || !geom) continue;
                    const uint8_t val =
                        props->Str("mask") == "water" ? 0 : 255;
                    const JsonValue* coords = geom->Get("coordinates");
                    if (!coords || coords->arr.empty()) continue;
                    // Outer ring only (holes are a later refinement), lon/lat -> window px.
                    std::vector<std::pair<double, double>> ring;
                    double x0 = 1e18, x1 = -1e18, y0 = 1e18, y1 = -1e18;
                    for (const JsonValue& pt : coords->arr[0].arr) {
                        if (pt.arr.size() < 2) continue;
                        const double lon = pt.arr[0].number;
                        const double lat = pt.arr[1].number * 3.14159265358979 / 180.0;
                        const double mx = (lon + 180.0) / 360.0 * n14;
                        const double my =
                            (0.5 - std::log(std::tan(0.7853981634 + lat * 0.5)) /
                                       (2.0 * 3.14159265358979)) *
                            n14;
                        const double px = (mx - orgPxX) * scale;
                        const double py = (my - orgPxY) * scale;
                        ring.push_back({px, py});
                        x0 = (std::min)(x0, px); x1 = (std::max)(x1, px);
                        y0 = (std::min)(y0, py); y1 = (std::max)(y1, py);
                    }
                    if (ring.size() < 3) continue;
                    ++nEdits;
                    for (int y = (std::max)(0, static_cast<int>(y0));
                         y <= (std::min)(static_cast<int>(dim) - 1, static_cast<int>(y1));
                         ++y) {
                        for (int x = (std::max)(0, static_cast<int>(x0));
                             x <= (std::min)(static_cast<int>(dim) - 1,
                                             static_cast<int>(x1));
                             ++x) {
                            if (!PointInRing(x + 0.5, y + 0.5, ring)) continue;
                            rg[(static_cast<size_t>(y) * dim + x) * 2] = val;
                            rg[(static_cast<size_t>(y) * dim + x) * 2 + 1] = 255;
                        }
                    }
                }
            }
            if (nEdits) {
                Log("[gis] %d mask edit polygon%s baked (edits.geojson overrides survey AND "
                    "the live-tide classifier)",
                    nEdits, nEdits == 1 ? "" : "s");
            }
        }
        m_maskWin = gpu.CreateTexture2D(dim, dim, DXGI_FORMAT_R8G8_UNORM,
                                        D3D12_RESOURCE_FLAG_NONE,
                                        D3D12_RESOURCE_STATE_COPY_DEST,
                                        L"gis.landmaskWindow (survey + hand edits)");
        gpu.UploadTexture(m_maskWin, rg.data(), dim * 2);
        m_maskWin.srv = gpu.CreateSrv(m_maskWin.res.Get(), DXGI_FORMAT_R8G8_UNORM);
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
