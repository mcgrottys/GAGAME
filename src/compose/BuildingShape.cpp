// BuildingShape - one building's own form in the folded tree (BuildingShape.h).
#include "compose/BuildingShape.h"

#include <algorithm>
#include <cmath>
#include <cstring>

namespace ga {

namespace {

constexpr double kDeg = 3.14159265358979323846 / 180.0;
constexpr double kR = 6371008.8;   // the prisms' own (BuildingLayer.cpp)

struct P2 {
    double x, y;
};
double Cross(const P2& a, const P2& b, const P2& c) { return (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x); }
bool InTri(const P2& p, const P2& a, const P2& b, const P2& c) {
    return Cross(a, b, p) >= 0.0 && Cross(b, c, p) >= 0.0 && Cross(c, a, p) >= 0.0;
}
double Area2(const std::vector<double>& r) {
    double s = 0.0;
    const size_t n = r.size() / 2;
    for (size_t i = 0; i < n; ++i) {
        const size_t j = (i + 1) % n;
        s += r[2 * i] * r[2 * j + 1] - r[2 * j] * r[2 * i + 1];
    }
    return s;
}

// Ear clipping of one counter-clockwise polygon (indices into pts) -> triangles. A polygon that
// stops yielding ears (self-touching) is closed by a fan of what is left, so no roof goes missing.
void EarClip(const std::vector<P2>& pts, std::vector<uint32_t> idx, std::vector<uint32_t>& tris) {
    while (idx.size() > 3) {
        bool clipped = false;
        for (size_t i = 0; i < idx.size(); ++i) {
            const size_t ip = (i + idx.size() - 1) % idx.size(), in = (i + 1) % idx.size();
            const P2 &a = pts[idx[ip]], &b = pts[idx[i]], &c = pts[idx[in]];
            if (Cross(a, b, c) <= 0.0) continue;   // reflex or flat
            bool empty = true;
            for (size_t k = 0; k < idx.size() && empty; ++k) {
                if (k == ip || k == i || k == in) continue;
                const P2& q = pts[idx[k]];
                if ((q.x == a.x && q.y == a.y) || (q.x == b.x && q.y == b.y) || (q.x == c.x && q.y == c.y)) continue;
                empty = !InTri(q, a, b, c);
            }
            if (!empty) continue;
            tris.insert(tris.end(), {idx[ip], idx[i], idx[in]});
            idx.erase(idx.begin() + static_cast<std::ptrdiff_t>(i));
            clipped = true;
            break;
        }
        if (!clipped) break;
    }
    for (size_t i = 1; i + 1 < idx.size(); ++i) tris.insert(tris.end(), {idx[0], idx[i], idx[i + 1]});
}

}  // namespace

void TriangulateRoof(const std::vector<std::vector<double>>& xy, const std::vector<uint8_t>& outer,
                     std::vector<uint32_t>& tris) {
    if (xy.empty()) return;
    std::vector<P2> pts;
    std::vector<uint32_t> base(xy.size());
    for (size_t r = 0; r < xy.size(); ++r) {
        base[r] = static_cast<uint32_t>(pts.size());
        for (size_t k = 0; k + 1 < xy[r].size(); k += 2) pts.push_back({xy[r][k], xy[r][k + 1]});
    }
    // The first outer ring with each hole bridged in from its nearest vertex, then ear-clipped.
    std::vector<uint32_t> poly;
    for (size_t i = 0; i < xy[0].size() / 2; ++i) poly.push_back(base[0] + static_cast<uint32_t>(i));
    for (size_t r = 1; r < xy.size(); ++r) {
        if (outer[r]) continue;
        const size_t n = xy[r].size() / 2;
        size_t bestH = 0, bestP = 0;
        double best = 1e300;
        for (size_t h = 0; h < n; ++h) {
            const P2& q = pts[base[r] + h];
            for (size_t p = 0; p < poly.size(); ++p) {
                const double dd = std::hypot(pts[poly[p]].x - q.x, pts[poly[p]].y - q.y);
                if (dd < best) {
                    best = dd;
                    bestH = h;
                    bestP = p;
                }
            }
        }
        std::vector<uint32_t> hole;
        for (size_t k = 0; k <= n; ++k) hole.push_back(base[r] + static_cast<uint32_t>((bestH + k) % n));   // clockwise
        hole.push_back(poly[bestP]);
        poly.insert(poly.begin() + static_cast<std::ptrdiff_t>(bestP) + 1, hole.begin(), hole.end());
    }
    EarClip(pts, poly, tris);
    // A second outer ring (a multipolygon's other piece): its own roof.
    for (size_t r = 1; r < xy.size(); ++r) {
        if (!outer[r]) continue;
        std::vector<uint32_t> ix;
        for (size_t i = 0; i < xy[r].size() / 2; ++i) ix.push_back(base[r] + static_cast<uint32_t>(i));
        EarClip(pts, ix, tris);
    }
}

bool EncodeShape(const BuildingSolid& s, double latc, double lonc, std::vector<uint8_t>& out) {
    const double mx = std::cos(latc * kDeg) * kR * kDeg, my = kR * kDeg;
    std::vector<std::vector<double>> xy;
    std::vector<uint8_t> outer;
    double ext = 0.0;
    size_t nVerts = 0;
    for (size_t r = 0; r < s.rings.size(); ++r) {
        const std::vector<double>& g = s.rings[r];
        std::vector<double> ring;
        for (size_t k = 0; k + 1 < g.size(); k += 2) {
            const double x = (g[k] - lonc) * mx, y = (g[k + 1] - latc) * my;
            ring.push_back(x);
            ring.push_back(y);
            ext = (std::max)(ext, (std::max)(std::abs(x), std::abs(y)));
        }
        if (ring.size() < 6) {
            if (r == 0) return false;
            continue;
        }
        const bool isOuter = r == 0 || (r < s.outer.size() && s.outer[r] != 0);
        // Outer counter-clockwise, holes clockwise: the wall's outward normal is the edge's right.
        if ((Area2(ring) > 0.0) != isOuter) {
            const size_t n = ring.size() / 2;
            for (size_t i = 0; i < n / 2; ++i) {
                std::swap(ring[2 * i], ring[2 * (n - 1 - i)]);
                std::swap(ring[2 * i + 1], ring[2 * (n - 1 - i) + 1]);
            }
        }
        nVerts += ring.size() / 2;
        xy.push_back(std::move(ring));
        outer.push_back(isOuter ? 1 : 0);
    }
    if (xy.empty() || xy.size() > 255 || nVerts > 65535 || ext > 32000.0) return false;
    const bool metres = ext > 3200.0;
    const double unit = metres ? 1.0 : 0.1;
    // Quantised first, so the roof is triangulated over the very corners the walls stand on.
    for (std::vector<double>& ring : xy) {
        for (double& c : ring) c = std::lround(c / unit) * unit;
    }
    std::vector<uint32_t> tris;
    TriangulateRoof(xy, outer, tris);
    if (tris.size() / 3 > 65535) return false;
    ShapeHead h{};
    h.nVerts = static_cast<uint16_t>(nVerts);
    h.nTris = static_cast<uint16_t>(tris.size() / 3);
    h.nRings = static_cast<uint8_t>(xy.size());
    h.flags = static_cast<uint8_t>((s.kind == 1 ? shape::kPart : 0) | (metres ? shape::kMetres : 0));
    h.bottom = static_cast<float>(s.bottom);
    h.top = static_cast<float>(s.top);
    const size_t at = out.size();
    out.resize(at + shape::Bytes(h), 0);
    uint8_t* p = out.data() + at;
    std::memcpy(p, &h, sizeof(h));
    p += sizeof(h);
    for (const std::vector<double>& ring : xy) {
        const uint16_t n = static_cast<uint16_t>(ring.size() / 2);
        std::memcpy(p, &n, 2);
        p += 2;
    }
    for (const std::vector<double>& ring : xy) {
        for (double c : ring) {
            const int16_t q = static_cast<int16_t>(std::lround(c / unit));
            std::memcpy(p, &q, 2);
            p += 2;
        }
    }
    for (uint32_t t : tris) {
        const uint16_t q = static_cast<uint16_t>(t);
        std::memcpy(p, &q, 2);
        p += 2;
    }
    return true;
}

bool ReadShape(const uint8_t* p, const uint8_t* end, ShapeView& v) {
    if (end - p < static_cast<std::ptrdiff_t>(sizeof(ShapeHead))) return false;
    std::memcpy(&v.head, p, sizeof(ShapeHead));
    if (end - p < static_cast<std::ptrdiff_t>(shape::Bytes(v.head))) return false;
    const uint8_t* q = p + sizeof(ShapeHead);
    v.ringLen = reinterpret_cast<const uint16_t*>(q);
    q += 2u * v.head.nRings;
    v.xy = reinterpret_cast<const int16_t*>(q);
    q += 4u * v.head.nVerts;
    v.tri = reinterpret_cast<const uint16_t*>(q);
    v.unit = (v.head.flags & shape::kMetres) ? 1.0f : 0.1f;
    return true;
}

}  // namespace ga
