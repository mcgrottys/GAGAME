#include "sim/RiverModel.h"

#include "core/Common.h"
#include "core/Json.h"
#include "sim/TideModel.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <iterator>

namespace ga {

namespace {
constexpr double kD2R = 3.14159265358979323846 / 180.0;
}

bool RiverModel::Load(const std::string& path) {
    m_reaches.clear();
    m_sites.clear();
    std::ifstream f(path, std::ios::binary);
    if (path.empty() || !f) {
        Log("[river] no river layer at '%s': no river enters any solver", path.c_str());
        return false;
    }
    const std::string text((std::istreambuf_iterator<char>(f)), std::istreambuf_iterator<char>());
    std::string err;
    const JsonValue root = JsonParser::Parse(text, &err);
    if (const JsonValue* sites = root.Get("sites")) {
        for (const JsonValue& s : sites->arr) m_sites.push_back({s.Str("id"), s.Str("name"), s.Num("discharge_cms", -1.0)});
    }
    if (const JsonValue* reaches = root.Get("reaches")) {
        for (const JsonValue& r : reaches->arr) {
            Reach R;
            R.id = r.Str("id");
            R.name = r.Str("name");
            R.tidalKm = r.Num("tidal_km", -1.0);
            if (const JsonValue* line = r.Get("line")) {
                for (const JsonValue& v : line->arr) {
                    if (v.arr.size() < 4) continue;
                    R.line.push_back({v.arr[0].number, v.arr[1].number, v.arr[2].number, v.arr[3].number,
                                      v.arr.size() > 4 && v.arr[4].number != 0.0});
                }
            }
            if (const JsonValue* g = r.Get("gauges")) {
                for (const JsonValue& v : g->arr) R.gauges.push_back({v.Str("id"), v.Num("km", 0.0), v.Num("drainage_km2", 0.0)});
            }
            if (const JsonValue* t = r.Get("tides")) {
                for (const JsonValue& v : t->arr) R.tides.push_back({v.Str("id"), v.Num("km", 0.0)});
            }
            if (R.line.size() >= 2) m_reaches.push_back(std::move(R));
        }
    }
    Log("[river] %s: %zu reaches meeting the sea, %zu gauges", path.c_str(), m_reaches.size(), m_sites.size());
    return true;
}

const RiverModel::Site* RiverModel::SiteOf(const std::string& id) const {
    for (const Site& s : m_sites) {
        if (s.id == id) return &s;
    }
    return nullptr;
}

std::vector<RiverModel::Crossing> RiverModel::EntersBox(double lat0, double lon0, double lat1, double lon1) const {
    std::vector<Crossing> out;
    const auto in = [&](const Vertex& v) { return v.lon > lon0 && v.lon < lon1 && v.lat > lat0 && v.lat < lat1; };
    for (size_t r = 0; r < m_reaches.size(); ++r) {
        const auto& L = m_reaches[r].line;
        if (!in(L[0])) continue;   // the box must hold its mouth
        for (size_t i = 0; i + 1 < L.size(); ++i) {
            if (!in(L[i]) || in(L[i + 1])) continue;
            // The segment's exit: the smallest parameter at which it meets an edge.
            const Vertex& a = L[i];
            const Vertex& b = L[i + 1];
            double tBest = 2.0;
            int side = -1;
            const auto edge = [&](double a0, double b0, double e, int s) {
                if (b0 == a0) return;
                const double t = (e - a0) / (b0 - a0);
                if (t >= 0.0 && t <= 1.0 && t < tBest) {
                    tBest = t;
                    side = s;
                }
            };
            edge(a.lat, b.lat, lat1, 0);   // N
            edge(a.lat, b.lat, lat0, 1);   // S
            edge(a.lon, b.lon, lon0, 2);   // W
            edge(a.lon, b.lon, lon1, 3);   // E
            if (side < 0) break;
            Crossing c;
            c.reach = int(r);
            c.side = side;
            c.lat = a.lat + tBest * (b.lat - a.lat);
            c.lon = a.lon + tBest * (b.lon - a.lon);
            c.km = a.km + tBest * (b.km - a.km);
            c.areaKm2 = a.areaKm2 + tBest * (b.areaKm2 - a.areaKm2);
            out.push_back(c);
            break;
        }
    }
    std::stable_sort(out.begin(), out.end(), [](const Crossing& x, const Crossing& y) { return x.areaKm2 > y.areaKm2; });
    if (out.size() > size_t(kMaxRiverFaces)) {
        for (size_t k = kMaxRiverFaces; k < out.size(); ++k) {
            Log("[river] %s enters the box too (%.0f km^2 drained): past the solver's %d faces, a wall",
                m_reaches[size_t(out[k].reach)].name.c_str(), out[k].areaKm2, kMaxRiverFaces);
        }
        out.resize(kMaxRiverFaces);
    }
    return out;
}

double RiverModel::DischargeAt(const Crossing& c, std::string* said) const {
    if (c.reach < 0) return -1.0;
    const Reach& R = m_reaches[size_t(c.reach)];
    const Gauge* best = nullptr;
    double bestD = 1e300, q = -1.0;
    for (const Gauge& g : R.gauges) {
        const Site* s = SiteOf(g.id);
        if (!s || s->dischargeCms < 0.0 || g.areaKm2 <= 0.0 || c.areaKm2 <= 0.0) continue;
        const double d = std::abs(std::log(c.areaKm2 / g.areaKm2));
        if (d < bestD) {
            bestD = d;
            best = &g;
            q = s->dischargeCms;
        }
    }
    if (!best) {
        if (said) *said = "no gauge on " + R.name + " reports";
        return -1.0;
    }
    const double scale = c.areaKm2 / best->areaKm2;
    if (said) {
        char b[256];
        snprintf(b, sizeof(b), "USGS %s %s %.2f m^3/s x %.3f (the face drains %.0f km^2, the gauge %.0f)",
                 best->id.c_str(), SiteOf(best->id)->name.c_str(), q, scale, c.areaKm2, best->areaKm2);
        *said = b;
    }
    return q * scale;
}

double RiverModel::PrismArea(const Crossing& c, double highWaterNavd, const std::function<float(double, double)>& bedAt,
                             double planetR, double stepM, std::string* said) const {
    if (c.reach < 0) return 0.0;
    const Reach& R = m_reaches[size_t(c.reach)];
    const double kmEnd = R.tidalKm >= 0.0 ? (std::min)(R.tidalKm, R.line.back().km) : R.line.back().km;
    const double mLat = planetR * kD2R;
    const double reachMax = 3000.0;   // a guard: a cross-section no river's tidal reach exceeds
    double area = 0.0, kmWet = c.km, widest = 0.0;
    size_t seg = 0;
    for (double km = c.km + 0.5 * stepM / 1000.0; km < kmEnd; km += stepM / 1000.0) {
        while (seg + 2 < R.line.size() && R.line[seg + 1].km < km) ++seg;
        const Vertex& a = R.line[seg];
        const Vertex& b = R.line[seg + 1];
        const double t = (b.km > a.km) ? std::clamp((km - a.km) / (b.km - a.km), 0.0, 1.0) : 0.0;
        const double lat = a.lat + t * (b.lat - a.lat), lon = a.lon + t * (b.lon - a.lon);
        const double mLon = mLat * std::cos(lat * kD2R);
        double ex = (b.lon - a.lon) * mLon, ny = (b.lat - a.lat) * mLat;
        const double len = std::sqrt(ex * ex + ny * ny);
        if (len <= 0.0) continue;
        ex /= len;
        ny /= len;
        // the cross-section: perpendicular to the line, (-ny, ex) in metres east and north
        double width = 0.0;
        if (bedAt(lat, lon) < highWaterNavd) {
            const double ds = 0.5 * stepM;
            for (int dir = -1; dir <= 1; dir += 2) {
                for (double s = ds; s <= reachMax; s += ds) {
                    const double pe = -ny * s * dir, pn = ex * s * dir;
                    if (bedAt(lat + pn / mLat, lon + pe / mLon) >= highWaterNavd) break;
                    width += ds;
                }
            }
            width += 0.5 * stepM;
        }
        if (width > 0.0) kmWet = km;
        widest = (std::max)(widest, width);
        area += width * stepM;
    }
    if (said) {
        char b[320];
        snprintf(b, sizeof(b),
                 "%.2f km^2 of %s below %+.2f m NAVD88 from km %.2f to km %.2f (water to km %.2f, widest %.0f m; "
                 "the head of tide %s)",
                 area / 1.0e6, R.name.c_str(), highWaterNavd, c.km, kmEnd, kmWet, widest,
                 R.tidalKm >= 0.0 ? "the hydrography's" : "where the line ends: the bed ends the water");
        *said = b;
    }
    return area;
}

// ------------------------------------------------------------------------------- the clocks

double RiverFaceClock::DEta(const TideModel& m, int f, double t) const {
    if (!tide || f < 0 || f >= int(faces.size()) || oceanSta < 0) return 0.0;
    const Face& F = faces[size_t(f)];
    if (F.staA < 0) return 0.0;
    const auto own = [&](int s) { return m.Height(size_t(s), t) - m.S(size_t(s)).meanMllwM; };
    const double tw = own(F.staA) * (1.0 - F.w) + own(F.staB) * F.w;
    return tw - own(oceanSta);
}

double RiverFaceClock::Q(const TideModel& m, int f, double t) const {
    if (f < 0 || f >= int(faces.size())) return 0.0;
    const Face& F = faces[size_t(f)];
    const auto eta = [&](double u) { return (oceanSta >= 0 ? m.Height(size_t(oceanSta), u) : 0.0) + DEta(m, f, u); };
    const double dh = (eta(t + 300.0) - eta(t - 300.0)) / 600.0;
    return F.q - F.prismM2 * dh;
}

std::string RiverFaceClock::Describe(const TideModel& m, int f, double t, double datumOff) const {
    if (f < 0 || f >= int(faces.size())) return "no face";
    const Face& F = faces[size_t(f)];
    char b[512];
    if (F.staA < 0 || oceanSta < 0) {
        snprintf(b, sizeof(b), "%s face at km %.2f: no tide station on the river (its deviation 0)", F.name.c_str(), F.at.km);
        return b;
    }
    const auto& A = m.S(size_t(F.staA));
    const auto& B = m.S(size_t(F.staB));
    const auto& O = m.S(size_t(oceanSta));
    snprintf(b, sizeof(b),
             "%s face at river km %.2f: %s %+.3f less mean %.3f | %s %+.3f less mean %.3f | weight %.3f | "
             "entrance %s %+.3f less mean %.3f, MLLW->NAVD %+.3f",
             F.name.c_str(), F.at.km, A.name.c_str(), m.Height(size_t(F.staA), t), A.meanMllwM, B.name.c_str(),
             m.Height(size_t(F.staB), t), B.meanMllwM, F.w, O.name.c_str(), m.Height(size_t(oceanSta), t), O.meanMllwM,
             datumOff);
    return b;
}

RiverFaceClock BuildRiverClock(const TideModel& m, const RiverModel& rivers,
                               const std::vector<RiverModel::Crossing>& at, double riverQ) {
    RiverFaceClock c;
    const auto indexOf = [&](const std::string& id) {
        for (size_t i = 0; i < m.Count(); ++i) {
            if (m.S(i).id == id) return int(i);
        }
        return -1;
    };
    for (size_t k = 0; k < at.size(); ++k) {
        const RiverModel::Reach& R = rivers.R(at[k].reach);
        RiverFaceClock::Face F;
        F.at = at[k];
        F.name = R.name;
        // the stations on the reach the tide model holds, by kilometre
        std::vector<std::pair<double, int>> on;
        for (const RiverModel::Tide& t : R.tides) {
            const int i = indexOf(t.id);
            if (i >= 0) on.push_back({t.km, i});
        }
        std::sort(on.begin(), on.end());
        if (!on.empty()) {
            if (k == 0) c.oceanSta = on.front().second;   // the mouth's: the open water's clock
            size_t j = 0;
            while (j + 1 < on.size() && on[j + 1].first <= F.at.km) ++j;
            F.staA = on[j].second;
            F.staB = (j + 1 < on.size()) ? on[j + 1].second : on[j].second;
            const double kA = on[j].first, kB = (j + 1 < on.size()) ? on[j + 1].first : kA;
            F.w = kB > kA ? std::clamp((F.at.km - kA) / (kB - kA), 0.0, 1.0) : 0.0;
        }
        std::string said;
        const double q = (k == 0 && riverQ > 0.0) ? riverQ : rivers.DischargeAt(F.at, &said);
        if (k == 0 && riverQ > 0.0) said = "water.swe.riverQ (--river)";
        F.q = (std::max)(q, 0.0);
        Log("[river] %s enters by the %c side at %.5f N %.5f E, river km %.2f, %.0f km^2 drained: %.1f m^3/s (%s); "
            "its tide %s",
            R.name.c_str(), "NSWE"[F.at.side], F.at.lat, F.at.lon, F.at.km, F.at.areaKm2, F.q,
            said.c_str(), F.staA < 0 ? "none on the river: the ocean's" : (m.S(size_t(F.staA)).name + " / " + m.S(size_t(F.staB)).name).c_str());
        c.faces.push_back(F);
    }
    return c;
}

}  // namespace ga
