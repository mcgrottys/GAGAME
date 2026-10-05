#include "sim/TideModel.h"

#include "core/Common.h"
#include "core/Json.h"

#include <cmath>
#include <fstream>
#include <limits>
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

bool TideModel::Load(const std::string& jsonPath) {
    const std::string text = ReadFile(jsonPath);
    if (text.empty()) {
        Log("[tide] cannot read %s", jsonPath.c_str());
        return false;
    }
    std::string err;
    const JsonValue root = JsonParser::Parse(text, &err);
    if (!err.empty()) {
        Log("[tide] %s: JSON parse failed: %s", jsonPath.c_str(), err.c_str());
        return false;
    }

    m_epochUnix = root.Num("epoch_unix", 0);
    const JsonValue* stations = root.Get("stations");
    if (!stations || stations->type != JsonValue::Type::Array || stations->arr.empty()) {
        Log("[tide] %s has no stations", jsonPath.c_str());
        return false;
    }

    const std::string dir = DirOf(jsonPath);
    m_stations.clear();
    for (const JsonValue& js : stations->arr) {
        TideStation s;
        s.id = js.Str("id");
        s.name = js.Str("name", s.id.c_str());
        s.lat = js.Num("lat", 0);
        s.lon = js.Num("lon", 0);
        s.riverKm = js.Num("river_km", -1);
        s.meanMllwM = js.Num("mean_mllw_m", 0);
        s.mllwMinusNavdM = js.Num("mllw_minus_navd_m", -999);
        s.fitRmsM = js.Num("fit_rms_m", 0);
        s.fitMaxM = js.Num("fit_max_m", 0);

        if (const JsonValue* coeffs = js.Get("coeffs");
            coeffs && coeffs->type == JsonValue::Type::Array) {
            for (const JsonValue& jc : coeffs->arr) {
                TideCoeff c;
                c.ampM = jc.Num("amp_m", 0);
                // Stored as degrees/hour (NOAA's native unit for constituent speeds) so the JSON
                // is human-checkable against the harcon tables; converted here exactly once.
                c.omegaRadS = jc.Num("speed_dph", 0) * (3.14159265358979323846 / 180.0) / 3600.0;
                c.phaseRad = jc.Num("phase_rad", 0);
                if (c.ampM > 0) s.coeffs.push_back(c);
            }
        }

        const std::string offFile = js.Str("official_file");
        if (!offFile.empty()) {
            s.officialStartUnix = js.Num("official_start_unix", 0);
            s.officialDtS = js.Num("official_dt_s", 3600);
            const size_t n = static_cast<size_t>(js.Num("official_n", 0));
            std::ifstream f(dir + "/" + offFile, std::ios::binary);
            if (f && n > 0 && n < (1u << 24)) {
                s.officialM.resize(n);
                f.read(reinterpret_cast<char*>(s.officialM.data()),
                       static_cast<std::streamsize>(n * sizeof(float)));
                if (!f) s.officialM.clear();
            }
            if (s.officialM.empty()) Log("[tide] official series missing for %s", s.id.c_str());
        }

        Log("[tide] %s %-28s %2zu coeffs  mean %.3f m  fit rms %.1f mm  river %.1f km  official %zu h",
            s.id.c_str(), s.name.c_str(), s.coeffs.size(), s.meanMllwM, s.fitRmsM * 1000.0,
            s.riverKm, s.officialM.size());
        m_stations.push_back(std::move(s));
    }

    m_totalKm = 0;
    m_focus = 0;
    for (size_t i = 0; i < m_stations.size(); ++i) {
        if (m_stations[i].riverKm > m_totalKm) m_totalKm = m_stations[i].riverKm;
    }
    Log("[tide] %zu stations, epoch %.0f, river profile %.1f km", m_stations.size(), m_epochUnix,
        m_totalKm);
    return !m_stations.empty();
}

double TideModel::Height(size_t i, double unixT) const {
    const TideStation& s = m_stations[i];
    const double dt = unixT - m_epochUnix;
    double h = s.meanMllwM;
    for (const TideCoeff& c : s.coeffs) {
        h += c.ampM * std::cos(c.omegaRadS * dt + c.phaseRad);
    }
    return h;
}

double TideModel::Official(size_t i, double unixT) const {
    const TideStation& s = m_stations[i];
    if (s.officialM.size() < 2) return std::numeric_limits<double>::quiet_NaN();
    const double pos = (unixT - s.officialStartUnix) / s.officialDtS;
    if (pos < 0 || pos > static_cast<double>(s.officialM.size() - 1)) {
        return std::numeric_limits<double>::quiet_NaN();
    }
    const size_t k = static_cast<size_t>(pos);
    const double f = pos - static_cast<double>(k);
    const size_t k1 = (k + 1 < s.officialM.size()) ? k + 1 : k;
    return s.officialM[k] * (1.0 - f) + s.officialM[k1] * f;
}

}  // namespace ga
