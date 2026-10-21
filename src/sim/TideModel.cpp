#include "sim/TideModel.h"

#include <algorithm>
#include <cmath>

#include "core/Common.h"
#include "core/FileText.h"
#include "core/Json.h"

#include <cmath>
#include <fstream>
#include <limits>
#include <sstream>

namespace ga {

bool TideModel::Load(const std::string& jsonPath) {
    const std::string text = ReadFileText(jsonPath);
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
    m_profile.clear();
    m_focus = 0;
    Log("[tide] %zu stations, epoch %.0f", m_stations.size(), m_epochUnix);
    return !m_stations.empty();
}

void TideModel::SetProfile(std::vector<ProfileStation> p) {
    std::sort(p.begin(), p.end(), [](const ProfileStation& a, const ProfileStation& b) { return a.km < b.km; });
    m_profile = std::move(p);
    m_totalKm = m_profile.empty() ? 0.0 : m_profile.back().km;
}

double TideModel::ProfileKm(int index) const {
    for (const ProfileStation& p : m_profile) {
        if (p.index == index) return p.km;
    }
    return -1.0;
}

std::vector<int> TideModel::Nearest(double latDeg, double lonDeg, size_t k) const {
    constexpr double kD2R = 3.14159265358979323846 / 180.0;
    const double cp = std::cos(latDeg * kD2R);
    const double pt[3] = {cp * std::cos(lonDeg * kD2R), std::sin(latDeg * kD2R), cp * std::sin(lonDeg * kD2R)};
    std::vector<std::pair<double, int>> d;
    d.reserve(m_stations.size());
    for (size_t i = 0; i < m_stations.size(); ++i) {
        const TideStation& s = m_stations[i];
        const double cs = std::cos(s.lat * kD2R);
        d.push_back({-(pt[0] * cs * std::cos(s.lon * kD2R) + pt[1] * std::sin(s.lat * kD2R) + pt[2] * cs * std::sin(s.lon * kD2R)),
                     int(i)});
    }
    const size_t n = (std::min)(k, d.size());
    std::partial_sort(d.begin(), d.begin() + std::ptrdiff_t(n), d.end());
    std::vector<int> out;
    for (size_t i = 0; i < n; ++i) out.push_back(d[i].second);
    return out;
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
