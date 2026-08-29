#include "sim/CurrentModel.h"

#include "core/Common.h"
#include "core/Json.h"

#include <algorithm>
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

bool CurrentField::Sample(double lon, double lat, float& outU, float& outV) const {
    if (!Valid()) return false;
    const double fx = (lon - lon0) / dlon - 0.5;
    const double fy = (lat - lat0) / dlat - 0.5;
    const int x0 = static_cast<int>(std::floor(fx));
    const int y0 = static_cast<int>(std::floor(fy));
    if (x0 < 0 || y0 < 0 || x0 + 1 >= nx || y0 + 1 >= ny) return false;
    const double tx = fx - x0, ty = fy - y0;
    const auto at = [&](int x, int y, float& uu, float& vv) {
        const size_t k = static_cast<size_t>(y) * nx + x;
        uu = u[k];
        vv = v[k];
        return uu > -900.0f;
    };
    float u00, v00, u10, v10, u01, v01, u11, v11;
    const bool a = at(x0, y0, u00, v00), b = at(x0 + 1, y0, u10, v10);
    const bool c = at(x0, y0 + 1, u01, v01), d = at(x0 + 1, y0 + 1, u11, v11);
    if (a && b && c && d) {
        outU = static_cast<float>((u00 * (1 - tx) + u10 * tx) * (1 - ty) +
                                  (u01 * (1 - tx) + u11 * tx) * ty);
        outV = static_cast<float>((v00 * (1 - tx) + v10 * tx) * (1 - ty) +
                                  (v01 * (1 - tx) + v11 * tx) * ty);
        return true;
    }
    if (a) { outU = u00; outV = v00; return true; }   // nearest-ish fallback at coasts
    return false;
}

bool CurrentModel::Load(const std::string& jsonPath) {
    const std::string text = ReadFile(jsonPath);
    if (text.empty()) {
        Log("[currents] cannot read %s", jsonPath.c_str());
        return false;
    }
    std::string err;
    const JsonValue root = JsonParser::Parse(text, &err);
    if (!err.empty()) {
        Log("[currents] %s: JSON parse failed: %s", jsonPath.c_str(), err.c_str());
        return false;
    }

    if (const JsonValue* stations = root.Get("tidal_stations");
        stations && stations->type == JsonValue::Type::Array) {
        for (const JsonValue& js : stations->arr) {
            TidalCurrentStation s;
            s.id = js.Str("id");
            s.name = js.Str("name", s.id.c_str());
            s.floodDeg = js.Num("flood_deg", 285);
            s.ebbDeg = js.Num("ebb_deg", 105);
            if (const JsonValue* evs = js.Get("events");
                evs && evs->type == JsonValue::Type::Array) {
                for (const JsonValue& je : evs->arr) {
                    s.events.push_back({je.Num("t", 0), je.Num("ms", 0)});
                }
            }
            Log("[currents] %s %-28s %zu events, flood %.0f / ebb %.0f degT", s.id.c_str(),
                s.name.c_str(), s.events.size(), s.floodDeg, s.ebbDeg);
            if (s.events.size() >= 4) m_stations.push_back(std::move(s));
        }
    }

    if (const JsonValue* adcp = root.Get("buoy_adcp");
        adcp && adcp->type == JsonValue::Type::Object) {
        adcpValid = true;
        adcpId = adcp->Str("id", "44029");
        adcpMs = adcp->Num("ms", 0);
        adcpTowardDeg = adcp->Num("toward_deg", 0);
        adcpObsUnix = adcp->Num("obs_unix", 0);
    }

    if (const JsonValue* jf = root.Get("field"); jf && jf->type == JsonValue::Type::Object) {
        m_field.nx = static_cast<int>(jf->Num("nx", 0));
        m_field.ny = static_cast<int>(jf->Num("ny", 0));
        m_field.lon0 = jf->Num("lon0", 0);
        m_field.lat0 = jf->Num("lat0", 0);
        m_field.dlon = jf->Num("dlon", 0);
        m_field.dlat = jf->Num("dlat", 0);
        m_field.source = jf->Str("source");
        const std::string file = DirOf(jsonPath) + "/" + jf->Str("file", "gomofs_uv.f32");
        std::ifstream f(file, std::ios::binary);
        const size_t n = static_cast<size_t>(m_field.nx) * m_field.ny;
        if (f && n > 0 && n < (1u << 24)) {
            m_field.u.resize(n);
            m_field.v.resize(n);
            f.read(reinterpret_cast<char*>(m_field.u.data()),
                   static_cast<std::streamsize>(n * 4));
            f.read(reinterpret_cast<char*>(m_field.v.data()),
                   static_cast<std::streamsize>(n * 4));
            if (!f) { m_field.u.clear(); m_field.v.clear(); m_field.nx = 0; }
        } else {
            m_field.nx = 0;
        }
        if (m_field.Valid()) {
            size_t water = 0;
            for (float x : m_field.u) water += (x > -900.0f) ? 1 : 0;
            Log("[currents] field %dx%d from %s (%.0f%% water)", m_field.nx, m_field.ny,
                m_field.source.c_str(), 100.0 * water / n);
        } else {
            Log("[currents] field metadata present but %s unreadable", file.c_str());
        }
    }
    return Ready();
}

int CurrentModel::StationIndex(const char* id) const {
    for (size_t i = 0; i < m_stations.size(); ++i) {
        if (m_stations[i].id == id) return static_cast<int>(i);
    }
    return -1;
}

double CurrentModel::SignedSpeed(size_t sta, double unixT) const {
    const TidalCurrentStation& s = m_stations[sta];
    const auto& e = s.events;
    if (e.size() < 2 || unixT <= e.front().t || unixT >= e.back().t) return 0.0;
    size_t hi = 1;
    while (hi < e.size() && e[hi].t < unixT) ++hi;
    const TidalEvent& a = e[hi - 1];
    const TidalEvent& b = e[hi];
    const double f = (unixT - a.t) / std::max(b.t - a.t, 1.0);
    // Cosine blend: flat at each event, sinusoidal between slack and max -- NOAA's own guidance
    // for interpolating subordinate-station tables.
    return a.ms + (b.ms - a.ms) * 0.5 * (1.0 - std::cos(3.14159265358979 * f));
}

}  // namespace ga
