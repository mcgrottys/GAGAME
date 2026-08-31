#include "sim/GlobeModel.h"

#include "core/Common.h"
#include "core/Json.h"

#include <cmath>
#include <fstream>

namespace ga {

namespace {

bool ReadAll(const std::string& path, std::string* out) {
    std::ifstream f(path, std::ios::binary);
    if (!f) return false;
    out->assign(std::istreambuf_iterator<char>(f), std::istreambuf_iterator<char>());
    return true;
}

template <typename T>
bool ReadBin(const std::string& path, size_t count, std::vector<T>* out) {
    std::ifstream f(path, std::ios::binary);
    if (!f) return false;
    out->resize(count);
    f.read(reinterpret_cast<char*>(out->data()), count * sizeof(T));
    return static_cast<size_t>(f.gcount()) == count * sizeof(T);
}

}  // namespace

bool GlobeModel::LoadMars(const std::string& jsonPath) {
    std::string text;
    if (!ReadAll(jsonPath, &text)) return false;
    std::string err;
    const JsonValue js = JsonParser::Parse(text, &err);
    if (!err.empty()) return false;
    const std::string dir = jsonPath.substr(0, jsonPath.find_last_of("/\\") + 1);
    m_nx = static_cast<int>(js.Num("mars_nx", 0));
    m_ny = static_cast<int>(js.Num("mars_ny", 0));
    const std::string relief = js.Str("mars_file");
    if (m_nx <= 0 || m_ny <= 0 || relief.empty() ||
        !ReadBin(dir + relief, static_cast<size_t>(m_nx) * m_ny, &m_elev)) {
        Log("[globe] no MOLA relief (run: py -3 harvester\\harvest_globe.py); Mars stays a "
            "textured sphere");
        m_nx = 0;
        return false;
    }
    Log("[globe] MOLA relief %dx%d loaded (%s)", m_nx, m_ny,
        js.Str("mars_source", "MOLA").c_str());
    return true;
}

bool GlobeModel::Load(const std::string& jsonPath) {
    std::string text;
    if (!ReadAll(jsonPath, &text)) return false;
    std::string err;
    const JsonValue js = JsonParser::Parse(text, &err);
    if (!err.empty()) {
        Log("[globe] %s: %s", jsonPath.c_str(), err.c_str());
        return false;
    }
    const std::string dir = jsonPath.substr(0, jsonPath.find_last_of("/\\") + 1);

    m_nx = static_cast<int>(js.Num("nx", 0));
    m_ny = static_cast<int>(js.Num("ny", 0));
    const std::string relief = js.Str("relief_file");
    if (m_nx <= 0 || m_ny <= 0 || relief.empty() ||
        !ReadBin(dir + relief, static_cast<size_t>(m_nx) * m_ny, &m_elev)) {
        Log("[globe] relief missing or short at '%s%s'", dir.c_str(), relief.c_str());
        m_nx = 0;
        return false;
    }

    m_wnx = static_cast<int>(js.Num("waves_nx", 0));
    m_wny = static_cast<int>(js.Num("waves_ny", 0));
    m_wlat1 = js.Num("waves_lat1", 90.0);
    m_wlon1 = js.Num("waves_lon1", 0.0);
    m_wdlat = js.Num("waves_dlat", 0.25);
    m_wdlon = js.Num("waves_dlon", 0.25);
    m_cycle = js.Str("waves_cycle", "none");
    if (m_wnx > 0) {
        const size_t n = static_cast<size_t>(m_wnx) * m_wny;
        if (!ReadBin(dir + js.Str("hs_file", "hs.f32"), n, &m_hs)) m_hs.clear();
        const std::string wf = js.Str("wind_file");
        if (!wf.empty() && !ReadBin(dir + wf, n, &m_wind)) m_wind.clear();
    }

    // M6c: the live 3D sky.
    m_cnx = static_cast<int>(js.Num("clouds_nx", 0));
    m_cny = static_cast<int>(js.Num("clouds_ny", 0));
    m_cnz = static_cast<int>(js.Num("clouds_nz", 0));
    m_cloudsCycle = js.Str("clouds_cycle", "none");
    if (m_cnx > 0) {
        const size_t n = static_cast<size_t>(m_cnx) * m_cny * m_cnz;
        if (!ReadBin(dir + js.Str("clouds_file", "cloud_iso.f32"), n, &m_clouds)) {
            m_clouds.clear();
            m_cnx = 0;
        }
        if (const JsonValue* alts = js.Get("clouds_alt_m")) {
            for (const auto& a : alts->arr) m_cloudAlts.push_back(static_cast<float>(a.number));
        }
    }

    // M6d: NE 15s window + global wind vector.
    m_nenx = static_cast<int>(js.Num("ne_nx", 0));
    m_neny = static_cast<int>(js.Num("ne_ny", 0));
    m_neLon0 = js.Num("ne_lon0", -72.0);
    m_neLat1 = js.Num("ne_lat1", 45.0);
    m_neDlon = js.Num("ne_dlon", 15.0 / 3600.0);
    m_neDlat = js.Num("ne_dlat", -15.0 / 3600.0);
    if (m_nenx > 0 &&
        !ReadBin(dir + js.Str("ne_file", "ne_15s.i16"),
                 static_cast<size_t>(m_nenx) * m_neny, &m_neElev)) {
        m_nenx = 0;
    }
    m_wvnx = static_cast<int>(js.Num("windvec_nx", 0));
    m_wvny = static_cast<int>(js.Num("windvec_ny", 0));
    m_wvlat1 = js.Num("windvec_lat1", 90.0);
    m_wvlon1 = js.Num("windvec_lon1", 0.0);
    m_wvdlat = js.Num("windvec_dlat", 0.5);
    m_wvdlon = js.Num("windvec_dlon", 0.5);
    if (m_wvnx > 0) {
        const size_t n = static_cast<size_t>(m_wvnx) * m_wvny;
        if (!ReadBin(dir + "wind_u.f32", n, &m_windU) ||
            !ReadBin(dir + "wind_v.f32", n, &m_windV)) {
            m_windU.clear();
            m_windV.clear();
            m_wvnx = 0;
        }
    }

    // M9: the water's quality. All three fields share one grid; any one missing drops the
    // whole family, because the optics model consumes them together (a Kd490 with no
    // backscatter would render a black ocean, not a partial answer).
    m_ocnx = static_cast<int>(js.Num("oc_nx", 0));
    m_ocny = static_cast<int>(js.Num("oc_ny", 0));
    m_oclat1 = js.Num("oc_lat1", 90.0);
    m_oclon1 = js.Num("oc_lon1", -180.0);
    m_ocdlat = js.Num("oc_dlat", -0.25);
    m_ocdlon = js.Num("oc_dlon", 0.25);
    m_ocnull = static_cast<float>(js.Num("oc_null", -9.0));
    m_ocEpoch = js.Str("oc_epoch_utc", "none");
    if (m_ocnx > 0) {
        const size_t n = static_cast<size_t>(m_ocnx) * m_ocny;
        if (!ReadBin(dir + js.Str("chlor_a_file", "oc_chl.f32"), n, &m_ocChl) ||
            !ReadBin(dir + js.Str("kd_490_file", "oc_kd490.f32"), n, &m_ocKd) ||
            !ReadBin(dir + js.Str("spm_file", "oc_spm.f32"), n, &m_ocSpm)) {
            m_ocChl.clear();
            m_ocKd.clear();
            m_ocSpm.clear();
            m_ocnx = 0;
        }
    }
    // Sea ice rides the GFS grid the waves already use; refuse it if the dims disagree,
    // rather than sampling one grid's field through another's mapping.
    m_iceCycle = js.Str("ice_cycle", "none");
    const int inx = static_cast<int>(js.Num("ice_nx", 0));
    const int iny = static_cast<int>(js.Num("ice_ny", 0));
    if (inx > 0 && inx == m_wnx && iny == m_wny) {
        if (!ReadBin(dir + js.Str("ice_file", "ice.f32"),
                     static_cast<size_t>(inx) * iny, &m_ice)) {
            m_ice.clear();
        }
    } else if (inx > 0) {
        Log("[globe] sea ice %dx%d does not match the wave grid %dx%d -- dropped", inx, iny,
            m_wnx, m_wny);
    }

    Log("[globe] ocean colour %dx%d (%s) %s; sea ice %s %s", m_ocnx, m_ocny, m_ocEpoch.c_str(),
        m_ocChl.empty() ? "ABSENT (optics fall back to pure water)" : "loaded",
        m_iceCycle.c_str(), m_ice.empty() ? "absent" : "loaded");

    Log("[globe] relief %dx%d (~%.1f km/texel at the equator); waves %s %dx%d%s; clouds %s "
        "%dx%dx%d; NE 15s %dx%d; wind vec %dx%d",
        m_nx, m_ny, 2.0 * 3.14159265358979 * kR / m_nx / 1000.0, m_cycle.c_str(), m_wnx, m_wny,
        m_wind.empty() ? " (no wind)" : " + wind", m_cloudsCycle.c_str(), m_cnx, m_cny, m_cnz,
        m_nenx, m_neny, m_wvnx, m_wvny);
    return true;
}

double GlobeModel::ElevAt(double latDeg, double lonDeg) const {
    if (!Ready()) return 0.0;
    double x = (lonDeg + 180.0) / 360.0 * m_nx - 0.5;
    double y = (90.0 - latDeg) / 180.0 * m_ny - 0.5;
    x = std::fmod(x + m_nx, static_cast<double>(m_nx));
    y = std::fmin(std::fmax(y, 0.0), m_ny - 1.001);
    const int x0 = static_cast<int>(x), y0 = static_cast<int>(y);
    const int x1 = (x0 + 1) % m_nx, y1 = y0 + 1;
    const double fx = x - x0, fy = y - y0;
    auto at = [&](int xx, int yy) {
        return static_cast<double>(m_elev[static_cast<size_t>(yy) * m_nx + xx]);
    };
    return (at(x0, y0) * (1 - fx) + at(x1, y0) * fx) * (1 - fy) +
           (at(x0, y1) * (1 - fx) + at(x1, y1) * fx) * fy;
}

void GlobeModel::LatLonDir(double latDeg, double lonDeg, double out[3]) {
    const double d2r = 3.14159265358979 / 180.0;
    const double cl = std::cos(latDeg * d2r);
    out[0] = cl * std::cos(lonDeg * d2r);
    out[1] = std::sin(latDeg * d2r);
    out[2] = cl * std::sin(lonDeg * d2r);
}

}  // namespace ga
