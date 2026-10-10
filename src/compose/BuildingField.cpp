// BuildingField - the buildings under a pixel, as scalars (BuildingField.h).
#include "compose/BuildingField.h"

#include <algorithm>
#include <cmath>
#include <cstring>

namespace ga {

namespace {
constexpr double kPi = 3.14159265358979323846;
constexpr double kD2R = kPi / 180.0;
constexpr double kR = 6371008.8;
int FloorDiv(int a, int n) { return a >= 0 ? a / n : -((-a + n - 1) / n); }
}  // namespace

BuildingFieldSource::BuildingFieldSource(std::shared_ptr<const BuildingLodFile> tree, const std::string& id)
    : m_tree(std::move(tree)) {
    m_info = {"buildings.field",
              "buildings under a texel as (lambda_p, lambda_f under half the texel, lambda_f), narrower than the texel only, "
              "from the folded tree " + id + "; v3 (two cutoffs: the size law)",
              "EPSG:4326 (the tree's centroids)",
              1000.0, -180.0, -90.0, 180.0, 90.0};
}

std::shared_ptr<const BuildingFieldSource::PageData> BuildingFieldSource::Page(int L, int px, int py) const {
    const PageKey k{L, px, py};
    {
        std::lock_guard<std::mutex> lk(m_mx);
        auto it = m_cache.find(k);
        if (it != m_cache.end()) {
            m_lru.splice(m_lru.begin(), m_lru, it->second.second);
            return it->second.first;
        }
    }
    const LodPage* lp = m_tree->Find(L, px, py);
    if (!lp) return nullptr;
    auto p = std::make_shared<PageData>();
    if (!m_tree->Read(L, *lp, p->nodes, p->blds)) return nullptr;
    p->bytes = p->nodes.size() * sizeof(LodNode) + p->blds.size() * sizeof(LodBuilding) + 64;
    std::lock_guard<std::mutex> lk(m_mx);
    auto it = m_cache.find(k);   // another tile read it meanwhile
    if (it != m_cache.end()) return it->second.first;
    m_lru.push_front(k);
    m_cache[k] = {p, m_lru.begin()};
    m_bytes += p->bytes;
    while (m_bytes > kCacheBytes && m_lru.size() > 1) {
        auto old = m_cache.find(m_lru.back());
        m_bytes -= old->second.first->bytes;
        m_cache.erase(old);
        m_lru.pop_back();
    }
    return p;
}

void BuildingFieldSource::BeginTile(double latMin, double latMax, double lonMin, double lonMax, double groundResM,
                                    PaintCtx& ctx) {
    const uint32_t d = kGridDim;
    ctx.scratch.assign(size_t(d) * d * 3 * sizeof(float), 0);
    ctx.scratchDim = 0;   // nothing splatted yet: the tile is void
    ctx.sLat0 = latMin;
    ctx.sLat1 = latMax;
    ctx.sLon0 = lonMin;
    ctx.sLon1 = lonMax;
    if (!m_tree || !(groundResM > 0.0) || !(latMax > latMin) || !(lonMax > lonMin)) return;
    float* g = reinterpret_cast<float*>(ctx.scratch.data());
    const double lat0 = latMin / kD2R, lat1 = latMax / kD2R, lon0 = lonMin / kD2R, lon1 = lonMax / kD2R;
    const double t = groundResM;
    bool any = false;
    // THE SPLAT: a building (or a fold) at its centroid adds its area, volume and frontal measure
    // to the cell it stands in.
    auto splat = [&](double lat, double lon, double area, double frontHalf, double front) {
        if (lat < lat0 || lat >= lat1 || lon < lon0 || lon >= lon1) return;
        const int x = (std::min)(int((lon - lon0) / (lon1 - lon0) * d), int(d) - 1);
        const int y = (std::min)(int((lat1 - lat) / (lat1 - lat0) * d), int(d) - 1);   // row 0 = north
        float* c = g + (size_t(y) * d + x) * 3;
        c[0] += float(area);
        c[1] += float(frontHalf);
        c[2] += float(front);
        any = true;
    };
    // One building alone: its box's footprint, volume and mean projected width x height.
    auto alone = [&](const LodBuilding& b) {
        double lat, lon, zc, hz, a1, a2, hd;
        int cx, cy;
        LodUnpack(b, lat, lon, zc, hz, a1, a2, hd, cx, cy);
        if (2.0 * std::sqrt(a1 * a1 + a2 * a2 + hz * hz) >= t) return;   // the prisms' (wider than a texel)
        const double rho = std::sqrt(a1 * a1 + a2 * a2 + hz * hz);
        const double front = 2.0 * hz * 4.0 * (a1 + a2) / kPi;
        splat(lat, lon, 4.0 * a1 * a2, 2.0 * rho < 0.5 * t ? front : 0.0, front);
    };
    // The finest level whose quads are no wider than a texel: its descendants are one fold.
    int Lf = lod::kLmin - 1;
    for (int L = lod::kLmin; L <= lod::kLmax; ++L) {
        if (lod::QuadDeg(L) * lod::kMetresPerDeg <= t) Lf = L;
    }
    for (int L = (std::max)(Lf, lod::kLmin); L <= lod::kLmax; ++L) {
        const double Q = lod::QuadDeg(L);
        const int qx0 = int(std::floor(lon0 / Q)), qx1 = int(std::floor(lon1 / Q));
        const int qy0 = int(std::floor(lat0 / Q)), qy1 = int(std::floor(lat1 / Q));
        const int py0 = FloorDiv(qy0, lod::PageY(L)), py1 = FloorDiv(qy1, lod::PageY(L));
        const int px0 = FloorDiv(qx0, lod::kPageX), px1 = FloorDiv(qx1, lod::kPageX);
        for (int py = py0; py <= py1; ++py) {
            for (int px = px0; px <= px1; ++px) {
                const auto p = Page(L, px, py);
                if (!p) continue;
                if (L == Lf) {
                    // A node's descendants as their fold (its own buildings are below, alone).
                    for (const LodNode& n : p->nodes) {
                        if (n.x < qx0 || n.x > qx1 || n.y < qy0 || n.y > qy1) continue;
                        const LodBox& f = n.desc;
                        if (f.hz > 0.0f) {
                            const double area = 4.0 * double(f.a1) * f.a2 * f.cover;
                            const double vol = area * 2.0 * f.hz;
                            // A fold's buildings are under Q / 2 <= t / 2 wide: under both cutoffs.
                            splat(f.lat7 * 1e-7, f.lon7 * 1e-7, area, vol / kFoldWidthM, vol / kFoldWidthM);
                        }
                    }
                }
                for (const LodBuilding& b : p->blds) alone(b);   // every level's own, one by one
            }
        }
    }
    if (!any) return;
    // Sums to densities: each cell's area on the sphere.
    const double dlon = (lon1 - lon0) * kD2R / d;
    for (uint32_t y = 0; y < d; ++y) {
        const double la = (lat1 - (lat1 - lat0) * y / d) * kD2R, lb = (lat1 - (lat1 - lat0) * (y + 1) / d) * kD2R;
        const double cellA = kR * kR * dlon * (std::sin(la) - std::sin(lb));
        const float inv = cellA > 0.0 ? float(1.0 / cellA) : 0.0f;
        for (uint32_t x = 0; x < d; ++x) {
            float* c = g + (size_t(y) * d + x) * 3;
            c[0] *= inv;
            c[1] *= inv;
            c[2] *= inv;
        }
    }
    ctx.scratchDim = d;
}

float BuildingFieldSource::Sample(double latRad, double lonRad, double, const PaintCtx& ctx, uint8_t rgba[4]) {
    if (ctx.scratchDim == 0) return 0.0f;   // no building in the tile: void
    const uint32_t d = ctx.scratchDim;
    const double u = (lonRad - ctx.sLon0) / (ctx.sLon1 - ctx.sLon0);
    const double v = (ctx.sLat1 - latRad) / (ctx.sLat1 - ctx.sLat0);
    const int x = std::clamp(int(u * d), 0, int(d) - 1), y = std::clamp(int(v * d), 0, int(d) - 1);
    const float* c = reinterpret_cast<const float*>(ctx.scratch.data()) + (size_t(y) * d + x) * 3;
    auto byte = [](double q) { return static_cast<uint8_t>(std::lround(std::clamp(q, 0.0, 1.0) * 255.0)); };
    rgba[0] = byte(c[0]);
    rgba[1] = byte(c[1] / kFrontal);
    rgba[2] = byte(c[2] / kFrontal);
    rgba[3] = 255;
    return 1.0f;
}

}  // namespace ga
