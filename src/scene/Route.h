// ================================================================================================
//  Route - M8: the AIS traffic lane as an arc-length-parameterized polyline. Ported from the
//  vqview-inlet Route contract with its lesson intact: waypoints are evenly spaced in
//  LONGITUDE, not distance, so stepping by index makes a vessel surge on bends -- At(s) walks
//  by cumulative arc length and the per-class AIS speed means what it says. The lane itself
//  is data/gis/route_merrimack.json (harvest_route.py: 9 days / 7087 AIS reports distilled to
//  where boats actually run, georeferenced into the engine's world frame).
// ================================================================================================
#pragma once

#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

#include "core/Common.h"
#include "core/Json.h"

namespace ga {

class Route {
public:
    bool Load(const char* path) {
        FILE* f = fopen(path, "rb");
        if (!f) {
            Log("[route] %s missing -- run harvester/harvest_route.py", path);
            return false;
        }
        std::string text;
        char buf[4096];
        size_t n;
        while ((n = fread(buf, 1, sizeof(buf), f)) > 0) text.append(buf, n);
        fclose(f);
        std::string err;
        const JsonValue v = JsonParser::Parse(text, &err);
        const JsonValue* wp = v.Get("waypoints");
        if (!err.empty() || !wp || wp->type != JsonValue::Type::Array || wp->arr.size() < 2) {
            Log("[route] %s unusable (%s)", path, err.empty() ? "no waypoints" : err.c_str());
            return false;
        }
        m_x.clear();
        m_z.clear();
        m_cum.clear();
        for (const JsonValue& p : wp->arr) {
            m_x.push_back(p.Num("x", 0.0));
            m_z.push_back(p.Num("z", 0.0));
        }
        m_cum.push_back(0.0);
        for (size_t i = 1; i < m_x.size(); ++i) {
            m_cum.push_back(m_cum[i - 1] +
                            std::hypot(m_x[i] - m_x[i - 1], m_z[i] - m_z[i - 1]));
        }
        // per-class mean speed over ground, straight from the AIS climatology
        if (const JsonValue* sp = v.Get("speeds_ms")) {
            speedRec = sp->Num("recreational", speedRec);
            speedFish = sp->Num("fishing", speedFish);
            speedTug = sp->Num("tug_other", speedTug);
            speedCom = sp->Num("commercial", speedCom);
        }
        Log("[route] %s: %zu waypoints, %.0f m lane (AIS speeds rec %.2f fish %.2f tug "
            "%.2f com %.2f m/s)",
            path, m_x.size(), Length(), speedRec, speedFish, speedTug, speedCom);
        return true;
    }

    double Length() const { return m_cum.empty() ? 0.0 : m_cum.back(); }
    bool Ready() const { return m_cum.size() >= 2; }

    // Clamped arc-length lookup: position + the segment's heading (rad, atan2(dz, dx)).
    void At(double s, double& x, double& z, double& heading) const {
        if (!Ready()) {
            x = z = heading = 0.0;
            return;
        }
        s = (std::max)(0.0, (std::min)(s, Length()));
        size_t i = 1;
        while (i + 1 < m_cum.size() && m_cum[i] < s) ++i;
        const double seg = (std::max)(m_cum[i] - m_cum[i - 1], 1e-4);
        const double u = (s - m_cum[i - 1]) / seg;
        x = m_x[i - 1] + (m_x[i] - m_x[i - 1]) * u;
        z = m_z[i - 1] + (m_z[i] - m_z[i - 1]) * u;
        heading = std::atan2(m_z[i] - m_z[i - 1], m_x[i] - m_x[i - 1]);
    }

    double speedRec = 4.86, speedFish = 4.62, speedTug = 3.83, speedCom = 3.34;

private:
    std::vector<double> m_x, m_z, m_cum;
};

}  // namespace ga
