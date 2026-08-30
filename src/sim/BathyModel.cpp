#include "sim/BathyModel.h"

#include "core/Common.h"
#include "core/Json.h"

#include <cmath>
#include <fstream>
#include <sstream>

namespace ga {

namespace {
std::string ReadFile(const std::string& path) {
    std::ifstream f(path, std::ios::binary);
    if (!f) return {};
    std::ostringstream ss;
    ss << f.rdbuf();
    return ss.str();
}
std::string DirOf(const std::string& path) {
    const size_t slash = path.find_last_of("/\\");
    return (slash == std::string::npos) ? std::string(".") : path.substr(0, slash);
}
}  // namespace

bool BathyModel::Load(const std::string& jsonPath) {
    const std::string text = ReadFile(jsonPath);
    if (text.empty()) {
        Log("[bathy] cannot read %s", jsonPath.c_str());
        return false;
    }
    std::string err;
    const JsonValue root = JsonParser::Parse(text, &err);
    if (!err.empty()) {
        Log("[bathy] %s: JSON parse failed: %s", jsonPath.c_str(), err.c_str());
        return false;
    }
    m_nx = static_cast<int>(root.Num("nx", 0));
    m_ny = static_cast<int>(root.Num("ny", 0));
    m_lon0 = root.Num("lon0", 0);
    m_lat1 = root.Num("lat1", 0);
    m_dlon = root.Num("dlon", 0);
    m_dlat = root.Num("dlat", 0);
    const std::string file = DirOf(jsonPath) + "/" + root.Str("file", "merrimack.f32");
    const size_t n = static_cast<size_t>(m_nx) * m_ny;
    std::ifstream f(file, std::ios::binary);
    if (!f || n == 0 || n > (1u << 26)) {
        Log("[bathy] %s unreadable", file.c_str());
        m_nx = 0;
        return false;
    }
    m_elev.resize(n);
    f.read(reinterpret_cast<char*>(m_elev.data()), static_cast<std::streamsize>(n * 4));
    if (!f) {
        m_nx = 0;
        return false;
    }

    m_worldX0 = static_cast<float>((m_lon0 - kOrgLon) * kMPerLon);
    m_worldSizeX = static_cast<float>(m_nx * m_dlon * kMPerLon);
    const double latSouth = m_lat1 - m_ny * m_dlat;
    m_worldZ0 = static_cast<float>((latSouth - kOrgLat) * kMPerLat);
    m_worldSizeZ = static_cast<float>(m_ny * m_dlat * kMPerLat);
    Log("[bathy] %dx%d, world [%.0f..%.0f] east x [%.0f..%.0f] north (m), range %.1f..%.1f m "
        "NAVD88",
        m_nx, m_ny, m_worldX0, m_worldX0 + m_worldSizeX, m_worldZ0, m_worldZ0 + m_worldSizeZ,
        root.Num("min_m", 0), root.Num("max_m", 0));
    return true;
}

namespace {
bool PointInRing(double x, double y, const std::vector<std::pair<double, double>>& r) {
    bool in = false;
    for (size_t i = 0, j = r.size() - 1; i < r.size(); j = i++) {
        if ((r[i].second > y) != (r[j].second > y) &&
            x < (r[j].first - r[i].first) * (y - r[i].second) / (r[j].second - r[i].second) +
                    r[i].first) {
            in = !in;
        }
    }
    return in;
}
}  // namespace

int BathyModel::ApplyMaskEdits(const std::string& geojsonPath, float crestNavd) {
    if (!Ready()) return 0;
    const std::string text = ReadFile(geojsonPath);
    if (text.empty()) return 0;
    std::string err;
    const JsonValue root = JsonParser::Parse(text, &err);
    if (!err.empty()) return 0;
    const JsonValue* feats = root.Get("features");
    if (!feats) return 0;

    int cells = 0;
    for (const JsonValue& ft : feats->arr) {
        const JsonValue* props = ft.Get("properties");
        const JsonValue* geom = ft.Get("geometry");
        if (!props || !geom) continue;
        if (props->Str("mask") != "land") continue;   // water edits stay classifier-only
        const JsonValue* coords = geom->Get("coordinates");
        if (!coords || coords->arr.empty()) continue;
        // Outer ring, lon/lat -> grid cells (row 0 = north).
        std::vector<std::pair<double, double>> ring;
        double x0 = 1e18, x1 = -1e18, y0 = 1e18, y1 = -1e18;
        for (const JsonValue& pt : coords->arr[0].arr) {
            if (pt.arr.size() < 2) continue;
            const double px = (pt.arr[0].number - m_lon0) / m_dlon;
            const double py = (m_lat1 - pt.arr[1].number) / m_dlat;
            ring.push_back({px, py});
            x0 = (std::min)(x0, px); x1 = (std::max)(x1, px);
            y0 = (std::min)(y0, py); y1 = (std::max)(y1, py);
        }
        if (ring.size() < 3) continue;
        for (int y = (std::max)(0, static_cast<int>(y0));
             y <= (std::min)(m_ny - 1, static_cast<int>(y1)); ++y) {
            for (int x = (std::max)(0, static_cast<int>(x0));
                 x <= (std::min)(m_nx - 1, static_cast<int>(x1)); ++x) {
                if (!PointInRing(x + 0.5, y + 0.5, ring)) continue;
                float& e = m_elev[static_cast<size_t>(y) * m_nx + x];
                if (e > -9000.0f && e < crestNavd) {
                    e = crestNavd;
                    ++cells;
                }
            }
        }
    }
    if (cells) {
        Log("[bathy] %d cells walled to %+.1f m NAVD from mask=land edits (%s -- the survey "
            "law reaches the solver)",
            cells, crestNavd, geojsonPath.c_str());
    }
    return cells;
}

float BathyModel::SampleWorld(float x, float z) const {
    if (!Ready()) return -9999.0f;
    // Row 0 = north: v grows southward.
    const double fx = (x - m_worldX0) / m_worldSizeX * m_nx - 0.5;
    const double fy = (1.0 - (z - m_worldZ0) / m_worldSizeZ) * m_ny - 0.5;
    const int x0 = static_cast<int>(std::floor(fx));
    const int y0 = static_cast<int>(std::floor(fy));
    if (x0 < 0 || y0 < 0 || x0 + 1 >= m_nx || y0 + 1 >= m_ny) return -9999.0f;
    const float tx = static_cast<float>(fx - x0), ty = static_cast<float>(fy - y0);
    const float v00 = m_elev[static_cast<size_t>(y0) * m_nx + x0];
    const float v10 = m_elev[static_cast<size_t>(y0) * m_nx + x0 + 1];
    const float v01 = m_elev[static_cast<size_t>(y0 + 1) * m_nx + x0];
    const float v11 = m_elev[static_cast<size_t>(y0 + 1) * m_nx + x0 + 1];
    if (v00 < -9000 || v10 < -9000 || v01 < -9000 || v11 < -9000) return -9999.0f;
    return (v00 * (1 - tx) + v10 * tx) * (1 - ty) + (v01 * (1 - tx) + v11 * tx) * ty;
}

}  // namespace ga
