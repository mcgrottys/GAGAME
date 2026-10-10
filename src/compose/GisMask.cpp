#include "compose/GisMask.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <fstream>

#include "core/Json.h"

namespace ga {

namespace {

constexpr double kPi = 3.14159265358979;
constexpr double kD2R = kPi / 180.0;
// F15: an arc's exact latitude bounds (defined with the index below; used by the edits' loader).
void ArcLatBounds(double ax, double ay, double az, double bx, double by, double bz, double& lo, double& hi);

}  // namespace

// ---- loading --------------------------------------------------------------------------------

namespace {

using GisLine = std::vector<std::pair<float, float>>;

// M9av: CLOSE THE CLIP. A coastline clipped to a survey box arrives as OPEN polylines whose ends
// lie on the box's edges. A ring test that closes each piece with a chord back to its own start
// draws that chord across open water: the mainland's ran from New Jersey to Maine, and even-odd
// parity called the whole Gulf of Maine inside it LAND (the wedge in the seafloor albedo). The
// closure the data means is along the BOX: walk its boundary counter-clockwise (interior on the
// left -- the coastline convention, land on the left of digitization) from each piece's end to
// the next piece's start, chaining until the chain returns to where it began. Disjoint land
// polygons come out, and their parity is the survey's.
double PerimT(double lon, double lat, double lon0, double lat0, double lon1, double lat1) {
    const double W = lon1 - lon0, H = lat1 - lat0, eps = 2e-3;
    if (std::fabs(lat - lat0) < eps) return lon - lon0;                     // south, eastward
    if (std::fabs(lon - lon1) < eps) return W + (lat - lat0);               // east, northward
    if (std::fabs(lat - lat1) < eps) return W + H + (lon1 - lon);           // north, westward
    if (std::fabs(lon - lon0) < eps) return 2.0 * W + H + (lat1 - lat);     // west, southward
    return -1.0;
}

size_t StitchAlongBox(std::vector<GisLine>& lines, std::string& note) {
    double lon0 = 1e9, lat0 = 1e9, lon1 = -1e9, lat1 = -1e9;
    for (const GisLine& l : lines) {
        for (const auto& p : l) {
            lon0 = (std::min)(lon0, double(p.first));
            lon1 = (std::max)(lon1, double(p.first));
            lat0 = (std::min)(lat0, double(p.second));
            lat1 = (std::max)(lat1, double(p.second));
        }
    }
    if (lon1 <= lon0 || lat1 <= lat0) return 0;
    const double W = lon1 - lon0, H = lat1 - lat0, P = 2.0 * (W + H);
    struct Open {
        size_t idx;
        double tStart, tEnd;
    };
    std::vector<Open> opens;
    for (size_t i = 0; i < lines.size(); ++i) {
        const GisLine& l = lines[i];
        if (l.size() < 2) continue;
        const auto& a = l.front();
        const auto& b = l.back();
        const double chordKm =
            std::hypot((b.first - a.first) * std::cos(a.second * kD2R), b.second - a.second) *
            111.0;
        if (chordKm < 0.05) continue;   // closed already
        const double ts = PerimT(a.first, a.second, lon0, lat0, lon1, lat1);
        const double te = PerimT(b.first, b.second, lon0, lat0, lon1, lat1);
        if (ts < 0.0 || te < 0.0) continue;   // open, but not on the clip: the chord stands
        opens.push_back({i, ts, te});
    }
    if (opens.empty()) return 0;
    auto ccw = [&](double from, double to) {
        double d = to - from;
        if (d < 0.0) d += P;
        return d;
    };
    auto corner = [&](double t) -> std::pair<float, float> {
        if (std::fabs(t - W) < 1e-9) return {float(lon1), float(lat0)};
        if (std::fabs(t - (W + H)) < 1e-9) return {float(lon1), float(lat1)};
        if (std::fabs(t - (2.0 * W + H)) < 1e-9) return {float(lon0), float(lat1)};
        return {float(lon0), float(lat0)};
    };
    const double cornersT[4] = {W, W + H, 2.0 * W + H, 0.0};
    std::vector<uint8_t> used(opens.size(), 0);
    std::vector<GisLine> polys;
    size_t broken = 0;
    for (size_t s = 0; s < opens.size(); ++s) {
        if (used[s]) continue;
        GisLine poly;
        size_t cur = s, guard = 0;
        for (;;) {
            used[cur] = 1;
            const GisLine& l = lines[opens[cur].idx];
            poly.insert(poly.end(), l.begin(), l.end());
            size_t nxt = SIZE_MAX;
            double best = 1e18;
            for (size_t j = 0; j < opens.size(); ++j) {
                double d = ccw(opens[cur].tEnd, opens[j].tStart);
                if (j == cur && d <= 1e-12) d = P;
                if (d < best) {
                    best = d;
                    nxt = j;
                }
            }
            // The box corners passed on the way, in walking order.
            std::vector<std::pair<double, int>> pass;
            for (int k = 0; k < 4; ++k) {
                const double d = ccw(opens[cur].tEnd, cornersT[k]);
                if (d > 1e-9 && d < best - 1e-9) pass.push_back({d, k});
            }
            std::sort(pass.begin(), pass.end());
            for (const auto& pk : pass) poly.push_back(corner(cornersT[pk.second]));
            if (nxt == s) break;
            if (nxt == SIZE_MAX || used[nxt] || ++guard > opens.size()) {
                ++broken;
                break;
            }
            cur = nxt;
        }
        polys.push_back(std::move(poly));
    }
    std::vector<uint8_t> drop(lines.size(), 0);
    for (const Open& o : opens) drop[o.idx] = 1;
    std::vector<GisLine> outL;
    for (size_t i = 0; i < lines.size(); ++i) {
        if (!drop[i]) outL.push_back(std::move(lines[i]));
    }
    const size_t made = polys.size();
    for (GisLine& p : polys) outL.push_back(std::move(p));
    lines.swap(outL);
    char b[200];
    snprintf(b, sizeof(b),
             "%zu clipped pieces stitched along box %.3f,%.3f..%.3f,%.3f into %zu land polygons"
             "%s",
             opens.size(), lon0, lat0, lon1, lat1, made,
             broken ? " (A CHAIN BROKE -- inspect)" : "");
    note = b;
    return made;
}

}  // namespace

bool GisVectorMask::ReadRings(const std::string& path, std::vector<Ring>& out, bool stitchClip) {
    std::ifstream f(path, std::ios::binary);
    if (!f) return false;
    uint32_t n = 0;
    f.read(reinterpret_cast<char*>(&n), 4);
    if (!f || n == 0 || n > 5000000u) return false;
    std::vector<GisLine> lines;
    lines.reserve(n);
    for (uint32_t i = 0; i < n && f; ++i) {
        uint32_t c = 0;
        f.read(reinterpret_cast<char*>(&c), 4);
        if (!f || c > 20000000u) return false;
        GisLine line(c);
        f.read(reinterpret_cast<char*>(line.data()), static_cast<std::streamsize>(c) * 8);
        lines.push_back(std::move(line));
    }
    if (stitchClip) {
        std::string note;
        if (StitchAlongBox(lines, note)) Log("[gismask] %s: %s", path.c_str(), note.c_str());
    }
    for (const GisLine& line : lines) {
        const uint32_t c = static_cast<uint32_t>(line.size());
        if (c < 3) continue;   // not a ring; a two-point line encloses nothing
        Ring r;
        r.first = static_cast<uint32_t>(m_pts.size());
        r.count = c;
        r.lon0 = r.lon1 = line[0].first;
        r.lat0 = r.lat1 = line[0].second;
        for (uint32_t k = 0; k < c; ++k) {
            const double lon = line[k].first * kD2R, lat = line[k].second * kD2R;
            const double cl = std::cos(lat);
            // Unit vectors, once. Every test below is a dot or a cross on the sphere, and
            // converting per query would cost more than the tests do.
            m_pts.push_back({cl * std::cos(lon), cl * std::sin(lon), std::sin(lat)});
            r.lon0 = (std::min)(r.lon0, line[k].first);
            r.lon1 = (std::max)(r.lon1, line[k].first);
            r.lat0 = (std::min)(r.lat0, line[k].second);
            r.lat1 = (std::max)(r.lat1, line[k].second);
        }
        m_lon0 = (std::min)(m_lon0, double(r.lon0));
        m_lon1 = (std::max)(m_lon1, double(r.lon1));
        m_lat0 = (std::min)(m_lat0, double(r.lat0));
        m_lat1 = (std::max)(m_lat1, double(r.lat1));
        out.push_back(r);
    }
    return !out.empty();
}

void GisVectorMask::LoadEdits(const std::string& path) {
    std::ifstream f(path, std::ios::binary);
    if (!f) return;
    const std::string text((std::istreambuf_iterator<char>(f)), std::istreambuf_iterator<char>());
    std::string err;
    const JsonValue root = JsonParser::Parse(text, &err);
    if (!err.empty()) return;
    const JsonValue* feats = root.Get("features");
    if (!feats) return;
    for (const JsonValue& ft : feats->arr) {
        const JsonValue* props = ft.Get("properties");
        const JsonValue* geom = ft.Get("geometry");
        if (!props || !geom) continue;
        const uint8_t val = props->Str("mask") == "water" ? 255u : 0u;
        const JsonValue* coords = geom->Get("coordinates");
        if (!coords) continue;
        // Polygon: [ring][pt][lon,lat]. MultiPolygon nests one deeper; walk to the first array
        // of numbers either way rather than trusting a type string.
        for (const JsonValue& a : coords->arr) {
            const JsonValue* ringArr = &a;
            if (!a.arr.empty() && !a.arr[0].arr.empty() && a.arr[0].arr[0].arr.size() >= 2) {
                ringArr = &a.arr[0];   // MultiPolygon: descend once
            }
            if (ringArr->arr.size() < 3) continue;
            Ring r;
            r.first = static_cast<uint32_t>(m_pts.size());
            r.count = 0;
            r.waterValue = val;
            bool first = true;
            for (const JsonValue& p : ringArr->arr) {
                if (p.arr.size() < 2) continue;
                const double lonD = p.arr[0].number, latD = p.arr[1].number;
                const double lon = lonD * kD2R, lat = latD * kD2R;
                const double cl = std::cos(lat);
                m_pts.push_back({cl * std::cos(lon), cl * std::sin(lon), std::sin(lat)});
                ++r.count;
                if (first) {
                    r.lon0 = r.lon1 = float(lonD);
                    r.lat0 = r.lat1 = float(latD);
                    first = false;
                } else {
                    r.lon0 = (std::min)(r.lon0, float(lonD));
                    r.lon1 = (std::max)(r.lon1, float(lonD));
                    r.lat0 = (std::min)(r.lat0, float(latD));
                    r.lat1 = (std::max)(r.lat1, float(latD));
                }
            }
            if (r.count >= 3) {
                // F15: the ring's arcs' latitudes, for the edge test (Touches): the vertices'
                // box, widened where an arc passes its circle's vertex.
                double lo = r.lat0, hi = r.lat1;
                for (uint32_t i = 0; i < r.count; ++i) {
                    const Vec3& A = m_pts[r.first + i];
                    const Vec3& B = m_pts[r.first + ((i + 1) % r.count)];
                    double t0 = 0.0, t1 = 0.0;
                    ArcLatBounds(A.x, A.y, A.z, B.x, B.y, B.z, t0, t1);
                    lo = (std::min)(lo, t0);
                    hi = (std::max)(hi, t1);
                }
                r.alat0 = float(lo - 1e-6);
                r.alat1 = float(hi + 1e-6);
                m_edits.push_back(r);
            }
        }
    }
}

bool GisVectorMask::Load(const std::string& dir, const std::string& landPath) {
    m_pts.reserve(1400000);
    if (!landPath.empty()) {   // THE GLOBAL COAST: its cell index only (see the header)
        std::ifstream f(landPath, std::ios::binary);
        char magic[8] = {};
        uint32_t n = 0;
        if (f && f.read(magic, 8) && std::memcmp(magic, "GALAND01", 8) == 0 && f.read(reinterpret_cast<char*>(&n), 4)) {
            m_landIdx.resize(n);
            for (uint32_t i = 0; i < n && f; ++i) {
                f.read(reinterpret_cast<char*>(&m_landIdx[i].cx), 2);
                f.read(reinterpret_cast<char*>(&m_landIdx[i].cy), 2);
                f.read(reinterpret_cast<char*>(&m_landIdx[i].rings), 4);
                f.read(reinterpret_cast<char*>(&m_landIdx[i].off), 8);
            }
            if (!f) m_landIdx.clear();
            m_landPath = landPath;
        }
        if (m_landIdx.empty()) {
            Log("[gismask] the global coast %s is not GALAND01 -- the New England survey alone", landPath.c_str());
        } else {
            uint64_t rings = 0;
            for (const LandIndex& c : m_landIdx) rings += c.rings;
            Log("[gismask] THE GLOBAL COAST: %s, %llu OSM land rings in %zu cells of 1 deg (index only; a cell is "
                "read when a tile that needs it is painted)", landPath.c_str(), static_cast<unsigned long long>(rings),
                m_landIdx.size());
        }
    }
    // The finer survey wins: with OSM's coast, GSHHG's New England coast is not the coast.
    const bool coast = !m_landIdx.empty() || ReadRings(dir + "coast_ne.bin", m_coast, /*stitchClip=*/true);
    // The NHD carve. survey.json: "NHD open-water even-odd per feature", with marsh and wetland
    // excluded because the live tide owns that call -- which is the same split as this gate.
    ReadRings(dir + "nhd_water_ne.bin", m_water);
    LoadEdits(dir + "edits.geojson");
    if (!coast) {
        Log("[gismask] no coast rings at %s -- the gate will not exist, and nothing that does "
            "not depend on it changes",
            dir.c_str());
        return false;
    }
    if (!m_landIdx.empty()) {   // the coast's opinion is the globe's
        m_lon0 = -180.0;
        m_lon1 = 180.0;
        m_lat0 = -90.0;
        m_lat1 = 90.0;
    }
    Log("[gismask] VECTOR land/sea: %zu coast rings, %zu NHD open-water rings, %zu hand-edit "
        "rings, %zu points -- no .raw parity fill opened",
        m_coast.size(), m_water.size(), m_edits.size(), m_pts.size());
    // ~0.005 degrees a bucket over the loaded span: small enough that a NHD pond lands in one
    // or two, large enough that the CSR stays a few megabytes.
    m_buckets = static_cast<uint32_t>((std::max)(64.0, (std::min)(16384.0,
                                                                  (m_lon1 - m_lon0) * 200.0)));
    BuildIndex(m_coast, m_coastIdx);
    BuildIndex(m_water, m_waterIdx);
    // The identity of the SURVEY AS LOADED. It rides in the source's `structure`, so
    // re-harvesting the GIS changes this mask's tree identity -- and therefore every composite
    // that consumed it -- and changes nothing else.
    char fp[160];
    snprintf(fp, sizeof(fp), "sweep v3 (per-feature carve) rings %zu/%zu/%zu pts %zu box %.4f,%.4f,%.4f,%.4f%s",
             m_coast.size(), m_water.size(), m_edits.size(), m_pts.size(), m_lon0, m_lat0,
             m_lon1, m_lat1, m_landIdx.empty() ? "" : " + OSM land GALAND01 v1");
    if (!m_landIdx.empty()) {   // the coast file's own size, so a new harvest repaints
        std::error_code ec;
        std::ifstream f(m_landPath, std::ios::binary | std::ios::ate);
        m_fingerprint = std::string(fp) + " " + std::to_string(static_cast<long long>(f.tellg()));
    }
    if (m_landIdx.empty()) m_fingerprint = fp;
    Log("[gismask] bounds %.3f..%.3f lon, %.3f..%.3f lat -- OUTSIDE this the gate is not in the "
        "subset, so every tile it cannot affect keeps the identity it already has",
        m_lon0, m_lon1, m_lat0, m_lat1);
    Log("[gismask] meridian index: %u longitude buckets, %zu coast edges, %zu water edges "
        "(%.1f MB) -- a column touches ~%zu edges, not %zu",
        m_buckets, m_coastIdx.a.size(), m_waterIdx.a.size(),
        (m_coastIdx.edge.size() + m_waterIdx.edge.size() + m_coastIdx.a.size() * 2 +
         m_waterIdx.a.size() * 2) * 4.0 / 1048576.0,
        m_buckets ? (m_coastIdx.edge.size() + m_waterIdx.edge.size()) / m_buckets : 0,
        m_coastIdx.a.size() + m_waterIdx.a.size());
    return true;
}

// ---- the index ------------------------------------------------------------------------------

int GisVectorMask::Bucket(double lonDeg) const {
    if (m_buckets == 0 || m_lon1 <= m_lon0) return -1;
    const double t = (lonDeg - m_lon0) / (m_lon1 - m_lon0);
    if (t < 0.0 || t >= 1.0) return -1;
    return static_cast<int>(t * m_buckets);
}

// F15: THE ARC'S LATITUDES. The ends bound an arc's longitudes (under 180 degrees it is monotone
// in longitude) but not its latitudes: the great circle through A and B has a vertex -- the
// projection of the pole onto its plane, h = z - (z.n)n -- and an arc that passes h reaches the
// vertex's latitude, poleward of both ends. h lies on the minor arc A->B iff (A x h).n > 0 and
// (h x B).n > 0 (the same side test the meet uses). The same for -h, the southern vertex.
namespace {
void ArcLatBounds(double ax, double ay, double az, double bx, double by, double bz, double& lo,
                  double& hi) {
    const double la = std::asin((std::max)(-1.0, (std::min)(1.0, az))) / kD2R;
    const double lb = std::asin((std::max)(-1.0, (std::min)(1.0, bz))) / kD2R;
    lo = (std::min)(la, lb);
    hi = (std::max)(la, lb);
    const double nx = ay * bz - az * by, ny = az * bx - ax * bz, nz = ax * by - ay * bx;
    const double n2 = nx * nx + ny * ny + nz * nz;
    if (n2 < 1e-30) return;   // A and B coincide: no arc
    const double hx = -nz * nx, hy = -nz * ny, hz = n2 - nz * nz;   // z - (z.n)n, scaled by n2
    const double h2 = hx * hx + hy * hy + hz * hz;
    if (h2 < 1e-30) return;   // the circle is the equator: its latitude is its ends'
    const double vertex = std::asin((std::max)(-1.0, (std::min)(1.0, hz / std::sqrt(h2)))) / kD2R;
    auto onArc = [&](double px, double py, double pz) {
        const double c1 = (ay * pz - az * py) * nx + (az * px - ax * pz) * ny + (ax * py - ay * px) * nz;
        const double c2 = (py * bz - pz * by) * nx + (pz * bx - px * bz) * ny + (px * by - py * bx) * nz;
        return c1 > 0.0 && c2 > 0.0;
    };
    if (onArc(hx, hy, hz)) hi = (std::max)(hi, vertex);
    if (onArc(-hx, -hy, -hz)) lo = (std::min)(lo, -vertex);
}
}  // namespace

// Longitude only. Parity along a meridian depends on every edge that crosses it however far
// south of the tile, so latitude cannot prune here and must not be allowed to.
void GisVectorMask::BuildIndex(const std::vector<Ring>& rings, EdgeIndex& idx) {
    std::vector<uint32_t> lo, hi;
    for (size_t ri = 0; ri < rings.size(); ++ri) {
        const Ring& r = rings[ri];
        for (uint32_t i = 0; i < r.count; ++i) {
            const uint32_t ai = r.first + i;
            const uint32_t bi = r.first + ((i + 1) % r.count);
            idx.a.push_back(ai);
            idx.b.push_back(bi);
            idx.ring.push_back(static_cast<uint32_t>(ri));
            const double la = std::atan2(m_pts[ai].y, m_pts[ai].x) / kD2R;
            const double lb = std::atan2(m_pts[bi].y, m_pts[bi].x) / kD2R;
            double l0 = (std::min)(la, lb), l1 = (std::max)(la, lb);
            if (l1 - l0 > 180.0) { l0 = m_lon0; l1 = m_lon1; }   // dateline-spanning: take all
            {   // F15: the edge's exact bounds, widened by a float's grain (0.1 m) so a rounding
                // can only make a tile sweep in full, never miss an edge.
                double t0 = 0.0, t1 = 0.0;
                ArcLatBounds(m_pts[ai].x, m_pts[ai].y, m_pts[ai].z, m_pts[bi].x, m_pts[bi].y,
                             m_pts[bi].z, t0, t1);
                idx.lonLo.push_back(float(l0 - 1e-6));
                idx.lonHi.push_back(float(l1 + 1e-6));
                idx.latLo.push_back(float(t0 - 1e-6));
                idx.latHi.push_back(float(t1 + 1e-6));
            }
            const int b0 = Bucket((std::max)(l0, m_lon0));
            const int b1 = Bucket((std::min)(l1, m_lon1 - 1e-9));
            lo.push_back(b0 < 0 ? 0u : uint32_t(b0));
            hi.push_back(b1 < 0 ? 0u : uint32_t(b1));
        }
    }
    idx.start.assign(m_buckets + 1, 0u);
    for (size_t e = 0; e < lo.size(); ++e) {
        for (uint32_t b = lo[e]; b <= hi[e] && b < m_buckets; ++b) ++idx.start[b + 1];
    }
    for (uint32_t b = 0; b < m_buckets; ++b) idx.start[b + 1] += idx.start[b];
    idx.edge.resize(idx.start[m_buckets]);
    std::vector<uint32_t> cur(idx.start.begin(), idx.start.end() - 1);
    for (size_t e = 0; e < lo.size(); ++e) {
        for (uint32_t b = lo[e]; b <= hi[e] && b < m_buckets; ++b) idx.edge[cur[b]++] = uint32_t(e);
    }
}

// ---- the global coast, by cell --------------------------------------------------------------

std::shared_ptr<const GisVectorMask::LandCell> GisVectorMask::LandCellAt(int cx, int cy) const {
    const auto it = std::lower_bound(m_landIdx.begin(), m_landIdx.end(), std::make_pair(cy, cx),
                                     [](const LandIndex& e, const std::pair<int, int>& k) {
                                         return std::make_pair(int(e.cy), int(e.cx)) < k;
                                     });
    if (it == m_landIdx.end() || it->cy != cy || it->cx != cx) return nullptr;
    const int32_t key = (cy + 90) * 360 + (cx + 180);
    {
        std::lock_guard<std::mutex> lk(m_landMx);
        auto c = m_landCache.find(key);
        if (c != m_landCache.end()) {
            m_landUse[key] = ++m_landTick;
            return c->second;
        }
    }
    auto cell = std::make_shared<LandCell>();
    std::ifstream f(m_landPath, std::ios::binary);
    f.seekg(static_cast<std::streamoff>(it->off));
    for (uint32_t r = 0; r < it->rings && f; ++r) {
        float box[4];
        uint32_t n = 0;
        f.read(reinterpret_cast<char*>(box), 16);
        f.read(reinterpret_cast<char*>(&n), 4);
        LandRing lr{box[0], box[1], box[2], box[3], static_cast<uint32_t>(cell->ll.size() / 2), n};
        cell->ll.resize(cell->ll.size() + 2 * size_t(n));
        f.read(reinterpret_cast<char*>(cell->ll.data() + 2 * size_t(lr.first)), static_cast<std::streamsize>(n) * 8);
        cell->rings.push_back(lr);
    }
    cell->bytes = cell->ll.size() * sizeof(float) + cell->rings.size() * sizeof(LandRing);
    std::lock_guard<std::mutex> lk(m_landMx);
    ++m_landReads;
    m_landCache[key] = cell;
    m_landUse[key] = ++m_landTick;
    m_landBytes += cell->bytes;
    while (m_landBytes > kLandCacheBytes && m_landCache.size() > 1) {   // the least recently used out
        auto old = m_landUse.begin();
        for (auto u = m_landUse.begin(); u != m_landUse.end(); ++u) {
            if (u->second < old->second) old = u;
        }
        auto c = m_landCache.find(old->first);
        if (c != m_landCache.end()) {
            m_landBytes -= c->second->bytes;
            m_landCache.erase(c);
        }
        m_landUse.erase(old);
    }
    return cell;
}

void GisVectorMask::GatherLand(double latMin, double latMax, double lonMin, double lonMax, double eps,
                               TileLand& t) const {
    // The cells about the tile: a ring is filed by its box's centre and reaches ~0.1 deg past it.
    const int x0 = static_cast<int>(std::floor(lonMin)) - 1, x1 = static_cast<int>(std::floor(lonMax)) + 1;
    const int y0 = (std::max)(-90, static_cast<int>(std::floor(latMin)) - 1);
    const int y1 = (std::min)(89, static_cast<int>(std::floor(latMax)) + 1);
    for (int cy = y0; cy <= y1; ++cy) {
        for (int cx = x0; cx <= x1; ++cx) {
            const int wx = ((cx + 180) % 360 + 360) % 360 - 180;
            const std::shared_ptr<const LandCell> cell = LandCellAt(wx, cy);
            if (!cell) continue;
            for (const LandRing& lr : cell->rings) {
                if (lr.lon1 < lonMin || lr.lon0 > lonMax || lr.lat1 < latMin || lr.lat0 > latMax) continue;
                Ring r;
                r.first = static_cast<uint32_t>(t.pts.size());
                r.lon0 = lr.lon0;
                r.lat0 = lr.lat0;
                r.lon1 = lr.lon1;
                r.lat1 = lr.lat1;
                // Decimated to the tile's grain: a vertex is kept a quarter texel from the last kept.
                const float* p = cell->ll.data() + 2 * size_t(lr.first);
                double plon = 1e9, plat = 1e9;
                for (uint32_t i = 0; i < lr.count; ++i) {
                    const double lon = p[2 * i], lat = p[2 * i + 1];
                    if (i + 1 < lr.count && std::abs(lon - plon) < eps && std::abs(lat - plat) < eps) continue;
                    plon = lon;
                    plat = lat;
                    const double cl = std::cos(lat * kD2R);
                    t.pts.push_back({cl * std::cos(lon * kD2R), cl * std::sin(lon * kD2R), std::sin(lat * kD2R)});
                    if (!t.touches && lon >= lonMin && lon <= lonMax && lat >= latMin && lat <= latMax) t.touches = true;
                }
                r.count = static_cast<uint32_t>(t.pts.size()) - r.first;
                if (r.count < 3) {
                    t.pts.resize(r.first);
                    continue;
                }
                // An edge can cross the box with neither end inside it: a ring whose box meets the
                // tile's and is not wholly around it is swept in full, to be sure.
                if (!t.touches && !(lr.lon0 <= lonMin && lr.lon1 >= lonMax && lr.lat0 <= latMin && lr.lat1 >= latMax)) {
                    t.touches = true;
                }
                if (!t.touches) {   // the box is wholly inside this ring's box: does an edge cross it?
                    for (uint32_t i = 0; i < r.count && !t.touches; ++i) {
                        const Vec3& A = t.pts[r.first + i];
                        const Vec3& B = t.pts[r.first + (i + 1) % r.count];
                        double a0 = 0.0, a1 = 0.0;
                        ArcLatBounds(A.x, A.y, A.z, B.x, B.y, B.z, a0, a1);
                        const double la = std::atan2(A.y, A.x) / kD2R, lb = std::atan2(B.y, B.x) / kD2R;
                        if ((std::max)(la, lb) >= lonMin && (std::min)(la, lb) <= lonMax && a1 >= latMin && a0 <= latMax) {
                            t.touches = true;
                        }
                    }
                }
                t.rings.push_back(r);
                t.south.push_back(lr.lat0 <= -89.999f ? 1u : 0u);
            }
        }
    }
}

// ---- the sweep ------------------------------------------------------------------------------

// THE MEET. An edge A->B spans the great circle n = A ^ B; the meridian at L is the great circle
// with normal m; their meet d = n x m is the antipodal pair of points where they cross, and `e`
// picks the half we are actually on. Exact on the sphere -- see the header.
namespace {
inline bool MeridianCross(const double ax, const double ay, const double az, const double bx,
                          const double by, const double bz, double mx, double my, double ex,
                          double ey, double& latDeg) {
    const double sa = mx * ax + my * ay;
    const double sb = mx * bx + my * by;
    if ((sa > 0.0) == (sb > 0.0)) return false;
    const double nx = ay * bz - az * by;
    const double ny = az * bx - ax * bz;
    const double nz = ax * by - ay * bx;
    double dx = -nz * my, dy = nz * mx, dz = nx * my - ny * mx;
    const double len = std::sqrt(dx * dx + dy * dy + dz * dz);
    if (len < 1e-15) return false;   // the edge lies in the meridian plane: no single crossing
    dx /= len;
    dy /= len;
    dz /= len;
    if (dx * ex + dy * ey < 0.0) dz = -dz;   // take the antipodal meet instead
    latDeg = std::asin((std::max)(-1.0, (std::min)(1.0, dz))) / kD2R;
    return true;
}
}  // namespace

void GisVectorMask::Crossings(const EdgeIndex& idx, double lonDeg, std::vector<double>& xs) const {
    const int b = Bucket(lonDeg);
    if (b < 0 || idx.start.empty()) return;
    const double L = lonDeg * kD2R;
    const double mx = -std::sin(L), my = std::cos(L);
    const double ex = std::cos(L), ey = std::sin(L);
    double lat = 0;
    for (uint32_t k = idx.start[b]; k < idx.start[b + 1]; ++k) {
        const uint32_t e = idx.edge[k];
        const Vec3& A = m_pts[idx.a[e]];
        const Vec3& B = m_pts[idx.b[e]];
        if (MeridianCross(A.x, A.y, A.z, B.x, B.y, B.z, mx, my, ex, ey, lat)) xs.push_back(lat);
    }
}

void GisVectorMask::CrossingsTagged(const EdgeIndex& idx, double lonDeg,
                                    std::vector<std::pair<uint32_t, double>>& xs) const {
    const int b = Bucket(lonDeg);
    if (b < 0 || idx.start.empty()) return;
    const double L = lonDeg * kD2R;
    const double mx = -std::sin(L), my = std::cos(L);
    const double ex = std::cos(L), ey = std::sin(L);
    double lat = 0;
    for (uint32_t k = idx.start[b]; k < idx.start[b + 1]; ++k) {
        const uint32_t e = idx.edge[k];
        const Vec3& A = m_pts[idx.a[e]];
        const Vec3& B = m_pts[idx.b[e]];
        if (MeridianCross(A.x, A.y, A.z, B.x, B.y, B.z, mx, my, ex, ey, lat)) {
            xs.push_back({idx.ring[e], lat});
        }
    }
}

void GisVectorMask::CrossingsRing(const Ring& r, double lonDeg, std::vector<double>& xs) const {
    if (lonDeg < r.lon0 || lonDeg > r.lon1) return;
    const double L = lonDeg * kD2R;
    const double mx = -std::sin(L), my = std::cos(L);
    const double ex = std::cos(L), ey = std::sin(L);
    double lat = 0;
    for (uint32_t i = 0; i < r.count; ++i) {
        const Vec3& A = m_pts[r.first + i];
        const Vec3& B = m_pts[r.first + ((i + 1) % r.count)];
        if (MeridianCross(A.x, A.y, A.z, B.x, B.y, B.z, mx, my, ex, ey, lat)) xs.push_back(lat);
    }
}

// Parity from the south. Valid because no ring here encloses the south pole -- these are New
// England coastlines and New England ponds. A polar ring set would need the count taken from a
// point known to be outside instead, which is a change to this function and nothing else.
void GisVectorMask::FillParity(std::vector<double>& xs, double latMin, double latMax, uint32_t dim,
                               std::vector<uint8_t>& col, uint8_t inside) {
    if (xs.empty()) return;
    std::sort(xs.begin(), xs.end());
    const double dLat = (latMax - latMin) / double(dim);
    size_t j = 0;
    bool in = false;
    for (uint32_t row = dim; row-- > 0;) {   // row 0 is latMax, so walk upward from the bottom
        const double lat = latMin + (double(dim - 1 - row) + 0.5) * dLat;
        while (j < xs.size() && xs[j] < lat) {
            in = !in;
            ++j;
        }
        if (in) col[row] = inside;
    }
}

bool GisVectorMask::Touches(const EdgeIndex& idx, double latMin, double latMax, double lonMin,
                            double lonMax) const {
    if (idx.start.empty()) return false;
    const int b0 = Bucket(lonMin), b1 = Bucket(lonMax);
    if (b0 < 0 || b1 < 0) return true;
    for (int b = b0; b <= b1; ++b) {
        for (uint32_t k = idx.start[b]; k < idx.start[b + 1]; ++k) {
            const uint32_t e = idx.edge[k];
            if (idx.lonHi[e] >= lonMin && idx.lonLo[e] <= lonMax && idx.latHi[e] >= latMin &&
                idx.latLo[e] <= latMax) {
                return true;
            }
        }
    }
    return false;
}

// F15: THE SWEEP IS OVER THE EDGES THAT MEET THE TILE. Parity down a meridian changes only at a
// crossing, and across the tile's columns the set of crossings inside the tile's band changes
// only where an edge enters the box: a tile no edge meets has the same parity in every column
// and every row, and one column decides it (the crossings south of the tile are counted in that
// column as in any other). A tile some edge meets is swept as before, every column. The two are
// the same function; the second is the first with its 255 other columns computed and discarded
// (--tool gis-sweep-test holds them equal, byte for byte).
void GisVectorMask::RasterizeGateImpl(double latMin, double latMax, double lonMin, double lonMax,
                                      uint32_t dim, std::vector<uint8_t>& out,
                                      bool allowOneColumn) const {
    // 255 everywhere: water, or no survey here. A gate's default must be PERMISSIVE -- the one
    // thing it must never do is delete a layer because it had no opinion.
    // M9ay: TWO BYTES A CELL -- [value, flags]. value: 255 water, 0 land. flags bit 0 =
    // SURVEYED (the column and row lie inside the rings' box: the mask has an opinion here);
    // bit 1 = EDITED (a hand ring from edits.geojson decided this cell). Unsurveyed cells
    // carry no opinion, which the source reports as weight 0 -- never as "water".
    out.assign(static_cast<size_t>(dim) * dim * 2u, 0u);
    if (m_coast.empty() && m_landIdx.empty()) return;
    // THE GLOBAL COAST for this tile: its rings, at its grain, bucketed by column (an edge is in the
    // columns its longitudes span), each ring's parity its own -- neighbouring pieces overlap a
    // little at the split grid's seams, and a union's parity would cut a line of sea along them.
    TileLand land;
    std::vector<uint32_t> colStart, colEdge;   // CSR: column -> edges (ring << 0 | index)
    std::vector<std::pair<uint32_t, uint32_t>> edges;   // (ring, first point of the edge)
    if (!m_landIdx.empty()) {
        GatherLand(latMin, latMax, lonMin, lonMax, 0.25 * (lonMax - lonMin) / double(dim), land);
        std::vector<std::pair<uint32_t, uint32_t>> span;   // per edge: first and last column
        for (uint32_t ri = 0; ri < land.rings.size(); ++ri) {
            const Ring& r = land.rings[ri];
            for (uint32_t i = 0; i < r.count; ++i) {
                const Vec3& A = land.pts[r.first + i];
                const Vec3& B = land.pts[r.first + (i + 1) % r.count];
                double la = std::atan2(A.y, A.x) / kD2R, lb = std::atan2(B.y, B.x) / kD2R;
                if (la > lb) std::swap(la, lb);
                if (lb - la > 180.0 || lb < lonMin || la > lonMax) continue;   // no column of the tile between its ends
                const double w = (lonMax - lonMin) / double(dim);
                const int c0 = (std::max)(0, static_cast<int>(std::floor((la - lonMin) / w - 0.5)));
                const int c1 = (std::min)(int(dim) - 1, static_cast<int>(std::ceil((lb - lonMin) / w - 0.5)));
                if (c1 < c0) continue;
                edges.push_back({ri, r.first + i});
                span.push_back({uint32_t(c0), uint32_t(c1)});
            }
        }
        colStart.assign(dim + 1, 0u);
        for (const auto& s : span) {
            for (uint32_t c = s.first; c <= s.second; ++c) ++colStart[c + 1];
        }
        for (uint32_t c = 0; c < dim; ++c) colStart[c + 1] += colStart[c];
        colEdge.resize(colStart[dim]);
        std::vector<uint32_t> cur(colStart.begin(), colStart.end() - 1);
        for (size_t e = 0; e < span.size(); ++e) {
            for (uint32_t c = span[e].first; c <= span[e].second; ++c) colEdge[cur[c]++] = uint32_t(e);
        }
    }
    bool one = allowOneColumn && lonMin >= m_lon0 && lonMax <= m_lon1 && !land.touches &&
               !Touches(m_coastIdx, latMin, latMax, lonMin, lonMax) &&
               !Touches(m_waterIdx, latMin, latMax, lonMin, lonMax);
    for (size_t i = 0; one && i < m_edits.size(); ++i) {
        const Ring& e = m_edits[i];
        if (e.lon1 >= lonMin && e.lon0 <= lonMax && e.alat1 >= latMin && e.alat0 <= latMax) one = false;
    }
    {
        const uint32_t n = 1u + (one ? m_sweptOne.fetch_add(1u) + m_sweptFull.load()
                                     : m_sweptFull.fetch_add(1u) + m_sweptOne.load());
        if ((n & 4095u) == 0u) {
            Log("[gismask] %u tiles swept: %u by one column (no ring edge meets the tile), %u in full",
                n, m_sweptOne.load(), m_sweptFull.load());
        }
    }
    std::vector<uint8_t> col(dim), ecol(dim);
    std::vector<double> xs;
    std::vector<std::pair<uint32_t, double>> tagged;
    const double dLat = (latMax - latMin) / double(dim);
    const uint32_t columns = one ? 1u : dim;
    for (uint32_t cx = 0; cx < columns; ++cx) {
        const double lon = one ? 0.5 * (lonMin + lonMax)
                               : lonMin + (double(cx) + 0.5) * (lonMax - lonMin) / double(dim);
        if (lon < m_lon0 || lon > m_lon1) continue;   // outside the survey: no opinion
        std::fill(col.begin(), col.end(), 255u);
        std::fill(ecol.begin(), ecol.end(), 0u);
        xs.clear();
        Crossings(m_coastIdx, lon, xs);
        FillParity(xs, latMin, latMax, dim, col, 0u);     // inside the coast: LAND
        if (!land.rings.empty()) {   // the global coast: each ring's own parity, LAND inside
            const uint32_t c = one ? dim / 2 : cx;
            const double L = lon * kD2R;
            const double mx = -std::sin(L), my = std::cos(L), ex = std::cos(L), ey = std::sin(L);
            tagged.clear();
            for (uint32_t k = colStart[c]; k < colStart[c + 1]; ++k) {
                const auto& [ri, ai] = edges[colEdge[k]];
                const Ring& r = land.rings[ri];
                const uint32_t bi = r.first + ((ai - r.first) + 1) % r.count;
                const Vec3& A = land.pts[ai];
                const Vec3& B = land.pts[bi];
                double lat = 0.0;
                if (MeridianCross(A.x, A.y, A.z, B.x, B.y, B.z, mx, my, ex, ey, lat)) tagged.push_back({ri, lat});
            }
            for (uint32_t ri = 0; ri < land.rings.size(); ++ri) {   // a ring around the south pole starts inside
                if (land.south[ri] && lon >= land.rings[ri].lon0 && lon <= land.rings[ri].lon1) tagged.push_back({ri, -91.0});
            }
            std::sort(tagged.begin(), tagged.end());
            for (size_t s0 = 0; s0 < tagged.size();) {
                size_t s1 = s0;
                xs.clear();
                while (s1 < tagged.size() && tagged[s1].first == tagged[s0].first) xs.push_back(tagged[s1++].second);
                FillParity(xs, latMin, latMax, dim, col, 0u);
                s0 = s1;
            }
        }
        // M9az: THE CARVE IS PER FEATURE, THEN OR -- the harvester's rule ("NHD open-water
        // even-odd per feature"). Parity over the UNION of the water rings cancels wherever
        // two water polygons overlap, and at the Merrimack mouth SeaOcean overlaps the
        // estuary/river polygon: the channel between the jetties came out LAND, the helm
        // stood on drained bed, and the bed classifier had been silently gated out of the
        // river since M9ak. Group the crossings by ring and fill each ring's parity alone.
        tagged.clear();
        CrossingsTagged(m_waterIdx, lon, tagged);
        std::sort(tagged.begin(), tagged.end());
        for (size_t s0 = 0; s0 < tagged.size();) {
            size_t s1 = s0;
            xs.clear();
            while (s1 < tagged.size() && tagged[s1].first == tagged[s0].first) {
                xs.push_back(tagged[s1].second);
                ++s1;
            }
            FillParity(xs, latMin, latMax, dim, col, 255u);   // this feature carves WATER
            s0 = s1;
        }
        for (const Ring& e : m_edits) {                   // and the hand edits are law
            xs.clear();
            CrossingsRing(e, lon, xs);
            FillParity(xs, latMin, latMax, dim, col, e.waterValue);
            FillParity(xs, latMin, latMax, dim, ecol, 255u);   // (xs stays sorted)
        }
        for (uint32_t row = 0; row < dim; ++row) {
            const double lat = latMax - (double(row) + 0.5) * dLat;   // row 0 = latMax
            const bool surveyed = lat >= m_lat0 && lat <= m_lat1;
            const uint8_t v = col[row];
            const uint8_t f = static_cast<uint8_t>((surveyed ? 1u : 0u) | (ecol[row] ? 2u : 0u));
            const uint32_t c0 = one ? 0u : cx, c1 = one ? dim : cx + 1u;
            for (uint32_t c = c0; c < c1; ++c) {
                const size_t i = (static_cast<size_t>(row) * dim + c) * 2u;
                out[i] = v;
                out[i + 1] = f;
            }
        }
    }
}

// ---- the source -----------------------------------------------------------------------------

void GisMaskSource::Refresh() {
    const GisVectorMask* mask = (m_mask && m_mask->Ready()) ? m_mask : nullptr;
    double lon0 = -180, lat0 = -90, lon1 = 180, lat1 = 90;
    if (mask) mask->Bounds(lon0, lat0, lon1, lat1);
    m_info = {"gis.landsea",
              "vector rings, even-odd by great-arc meet, no raster, value+flags: " +
                  (mask ? mask->Fingerprint() : std::string("none")),
              "EPSG:4326 (rings are the authority; the .raw parity fills are a realization)",
              1000.0,
              lon0,
              lat0,
              lon1,
              lat1};
}

void GisMaskSource::BeginTile(double latMin, double latMax, double lonMin, double lonMax,
                              double, PaintCtx& ctx) {
    if (!m_mask) return;
    m_mask->RasterizeGate(latMin / kD2R, latMax / kD2R, lonMin / kD2R, lonMax / kD2R, kGridDim,
                          ctx.scratch);
    ctx.scratchDim = kGridDim;
    ctx.sLat0 = latMin;
    ctx.sLat1 = latMax;
    ctx.sLon0 = lonMin;
    ctx.sLon1 = lonMax;
}

float GisMaskSource::Sample(double latRad, double lonRad, double, const PaintCtx& ctx,
                            uint8_t rgba[4]) {
    if (ctx.scratchDim == 0 || ctx.scratch.empty()) return 1.0f;   // no sweep: no opinion
    const double u = (lonRad - ctx.sLon0) / (ctx.sLon1 - ctx.sLon0);
    const double v = (ctx.sLat1 - latRad) / (ctx.sLat1 - ctx.sLat0);   // row 0 = latMax
    const uint32_t d = ctx.scratchDim;
    const int32_t x = static_cast<int32_t>(u * d);
    const int32_t y = static_cast<int32_t>(v * d);
    if (x < 0 || y < 0 || uint32_t(x) >= d || uint32_t(y) >= d) return 0.0f;   // no opinion
    const size_t i = (static_cast<size_t>(y) * d + x) * 2u;
    const uint8_t g = ctx.scratch[i], fl = ctx.scratch[i + 1];
    // Nearest, never interpolated: this is a classification, and a gate that blurs is not a
    // gate. The height band is what refines the waterline inside it.
    // M9ay: the mask is now READABLE as well as a gate. Weight = "surveyed" (an opinion
    // exists); the VALUE carries it: r = water coverage (1 water, 0 land), b = edited. The
    // gate multiplies by the value (ComposeTree.h GateSource); the classifier reads the pages.
    if (!(fl & 1u)) return 0.0f;
    rgba[0] = g;
    rgba[1] = g;
    rgba[2] = (fl & 2u) ? 255 : 0;
    rgba[3] = 255;
    return 1.0f;
}

}  // namespace ga
