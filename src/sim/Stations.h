// PHASE C3: A STATION IS A SOURCE AT ITS OWN PLACE. A reader asking at a point gets the station
// nearest it (the largest dot of unit directions) among those `holds` admits; unplaced: none; -1.
#pragma once

#include <cmath>

namespace ga {

template <class Model, class Holds>
int NearestStation(const Model& m, double latDeg, double lonDeg, Holds holds) {
    constexpr double k = 3.14159265358979323846 / 180.0;
    const double cp = std::cos(latDeg * k);
    const double p[3] = {cp * std::cos(lonDeg * k), std::sin(latDeg * k), cp * std::sin(lonDeg * k)};
    int best = -1;
    double bestDot = -2.0;
    for (size_t i = 0; i < m.Count(); ++i) {
        const auto& s = m.S(i);
        if (!std::isfinite(s.lat) || !std::isfinite(s.lon) || !holds(s.lat, s.lon)) continue;
        const double cs = std::cos(s.lat * k);
        const double d = p[0] * cs * std::cos(s.lon * k) + p[1] * std::sin(s.lat * k) +
                         p[2] * cs * std::sin(s.lon * k);
        if (d > bestDot) {
            bestDot = d;
            best = static_cast<int>(i);
        }
    }
    return best;
}

}  // namespace ga
