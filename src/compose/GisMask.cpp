#include "compose/GisMask.h"

#include <algorithm>
#include <cmath>
#include <fstream>

#include "core/Json.h"

namespace ga {

namespace {

constexpr double kPi = 3.14159265358979;
constexpr double kD2R = kPi / 180.0;

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
            if (r.count >= 3) m_edits.push_back(r);
        }
    }
}

bool GisVectorMask::Load(const std::string& dir) {
    m_pts.reserve(1400000);
    const bool coast = ReadRings(dir + "coast_ne.bin", m_coast, /*stitchClip=*/true);
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
    snprintf(fp, sizeof(fp), "sweep v3 (per-feature carve) rings %zu/%zu/%zu pts %zu box %.4f,%.4f,%.4f,%.4f",
             m_coast.size(), m_water.size(), m_edits.size(), m_pts.size(), m_lon0, m_lat0,
             m_lon1, m_lat1);
    m_fingerprint = fp;
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

void GisVectorMask::RasterizeGate(double latMin, double latMax, double lonMin, double lonMax,
                                  uint32_t dim, std::vector<uint8_t>& out) const {
    // 255 everywhere: water, or no survey here. A gate's default must be PERMISSIVE -- the one
    // thing it must never do is delete a layer because it had no opinion.
    // M9ay: TWO BYTES A CELL -- [value, flags]. value: 255 water, 0 land. flags bit 0 =
    // SURVEYED (the column and row lie inside the rings' box: the mask has an opinion here);
    // bit 1 = EDITED (a hand ring from edits.geojson decided this cell). Unsurveyed cells
    // carry no opinion, which the source reports as weight 0 -- never as "water".
    out.assign(static_cast<size_t>(dim) * dim * 2u, 0u);
    if (m_coast.empty()) return;
    std::vector<uint8_t> col(dim), ecol(dim);
    std::vector<double> xs;
    std::vector<std::pair<uint32_t, double>> tagged;
    const double dLat = (latMax - latMin) / double(dim);
    for (uint32_t cx = 0; cx < dim; ++cx) {
        const double lon = lonMin + (double(cx) + 0.5) * (lonMax - lonMin) / double(dim);
        if (lon < m_lon0 || lon > m_lon1) continue;   // outside the survey: no opinion
        std::fill(col.begin(), col.end(), 255u);
        std::fill(ecol.begin(), ecol.end(), 0u);
        xs.clear();
        Crossings(m_coastIdx, lon, xs);
        FillParity(xs, latMin, latMax, dim, col, 0u);     // inside the coast: LAND
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
            const size_t i = (static_cast<size_t>(row) * dim + cx) * 2u;
            out[i] = col[row];
            out[i + 1] = static_cast<uint8_t>((surveyed ? 1u : 0u) | (ecol[row] ? 2u : 0u));
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
