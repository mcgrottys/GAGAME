#include "compose/BuildingLod.h"

#include "compose/BuildingMoments.h"
#include "compose/BuildingShape.h"
#include "core/Common.h"
#include "core/Json.h"

#include <DirectXPackedVector.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <mutex>
#include <sstream>
#include <thread>
#include <set>
#include <tuple>

namespace ga {

namespace {

constexpr double kDeg = 3.14159265358979323846 / 180.0;
constexpr double kR = 6371008.8;   // the prisms' own (BuildingLayer.cpp)

int FloorDiv(int a, int n) { return a >= 0 ? a / n : -((-a + n - 1) / n); }
uint16_t Half(double v) { return DirectX::PackedVector::XMConvertFloatToHalf(static_cast<float>(v)); }
double Unhalf(uint16_t h) { return DirectX::PackedVector::XMConvertHalfToFloat(h); }
int Idx(int L) { return L - lod::kLmin; }

struct Item {
    int L, x, y;
    LodBuilding rec;
    float rho;
    uint32_t arena = 0;           // its shape: arenas[arena][off, off + len); len 0 = none
    uint32_t len = 0;
    uint64_t off = 0;
};

// A moment box in a local frame about (lon0, lat0) placed as an item: its level by its radius, its
// quad by its centroid, its record. Shared by the buildings' prisms and the roads' ribbons.
void PlaceItem(const MomentBox& b, double lon0, double lat0, double mx, double my, int cx, int cy, Item& it) {
    const double lat = lat0 + b.c[1] / my, lon = lon0 + b.c[0] / mx;
    it.rho = static_cast<float>(b.Radius());
    it.L = lod::LevelOf(b.Radius());
    const double Q = lod::QuadDeg(it.L);
    it.x = static_cast<int>(std::floor(lon / Q));
    it.y = static_cast<int>(std::floor(lat / Q));
    LodBuilding& r = it.rec;
    r.lat7 = static_cast<int32_t>(std::lround(lat * 1e7));
    r.lon7 = static_cast<int32_t>(std::lround(lon * 1e7));
    r.zc = Half(b.c[2]);
    r.hz = Half(b.half[2]);
    r.a1 = Half(b.half[0]);
    r.a2 = Half(b.half[1]);
    r.heading = static_cast<int16_t>(std::lround(b.heading * 1e4));
    const int ccx = static_cast<int>(std::floor(lon / lod::kDetailDeg)), ccy = static_cast<int>(std::floor(lat / lod::kDetailDeg));
    r.dcx = static_cast<int8_t>(std::clamp(cx - ccx, -127, 127));
    r.dcy = static_cast<int8_t>(std::clamp(cy - ccy, -127, 127));
}

// One solid's moment box and its record; false for a solid with no volume.
bool BoxItem(const BuildingSolid& s, int cx, int cy, Item& it) {
    if (s.rings.empty() || s.rings[0].size() < 6) return false;
    const double lon0 = s.rings[0][0], lat0 = s.rings[0][1];
    const double mx = std::cos(lat0 * kDeg) * kR * kDeg, my = kR * kDeg;
    std::vector<std::vector<double>> xy(s.rings.size());
    for (size_t k = 0; k < s.rings.size(); ++k) {
        const std::vector<double>& g = s.rings[k];
        for (size_t i = 0; i + 1 < g.size(); i += 2) {
            xy[k].push_back((g[i] - lon0) * mx);
            xy[k].push_back((g[i + 1] - lat0) * my);
        }
    }
    const Moments m = PrismMoments(xy, s.bottom, s.top, &s.outer);
    if (m.Empty()) return false;
    PlaceItem(BoxOf(m), lon0, lat0, mx, my, cx, cy, it);
    return true;
}

// ---- the roads (BuildingLod.h: RoadRibbonDefaults) --------------------------------------------
double RoadWidth(const RoadWay& w, const RoadRibbonDefaults& d) {
    if (w.widthM == w.widthM && w.widthM > 0.0 && w.widthM < 1e4) return w.widthM;
    if (w.rank == 255) return d.pathWidth;
    if (w.lanes == w.lanes && w.lanes > 0.0 && w.lanes < 1e3) return w.lanes * d.laneWidth;
    return d.defaultLanes * d.laneWidth;
}
constexpr double kPieceM = 2500.0;   // a piece fits a decimetre record (3.2 km from its centroid)

// A way as ribbon pieces, each at most kPieceM long (sharing its end point with the next, so no
// gap): the piece's mass is a slab a segment -- the segment's rectangle of the road's width from
// kerb under the ground to kerb over it -- summed; its record the polyline about the box's centroid.
void RibbonItems(const RoadWay& w, const RoadRibbonDefaults& d, int cx, int cy, uint32_t arenaIdx,
                 std::vector<uint8_t>& arena, std::vector<Item>& out) {
    const size_t n = w.points.size();
    if (n < 2) return;
    const double width = RoadWidth(w, d), hw = 0.5 * width;
    const bool path = w.rank == 255;
    const double lon0 = w.points[0].lon, lat0 = w.points[0].lat;
    const double mx = std::cos(lat0 * kDeg) * kR * kDeg, my = kR * kDeg;
    std::vector<double> xy(2 * n);
    for (size_t i = 0; i < n; ++i) {
        xy[2 * i] = (w.points[i].lon - lon0) * mx;
        xy[2 * i + 1] = (w.points[i].lat - lat0) * my;
    }
    auto seg = [&](size_t i) { return std::hypot(xy[2 * i + 2] - xy[2 * i], xy[2 * i + 3] - xy[2 * i + 1]); };
    size_t a = 0;
    while (a + 1 < n) {
        size_t b = a + 1;
        double len = seg(a);
        while (b + 1 < n && len + seg(b) <= kPieceM) {
            len += seg(b);
            ++b;
        }
        Moments m;
        for (size_t i = a; i < b; ++i) {
            const double dx = xy[2 * i + 2] - xy[2 * i], dy = xy[2 * i + 3] - xy[2 * i + 1], l = std::hypot(dx, dy);
            if (!(l > 1e-3)) continue;
            const double nx = dy / l * hw, ny = -dx / l * hw;
            std::vector<std::vector<double>> ring{{xy[2 * i] + nx, xy[2 * i + 1] + ny, xy[2 * i + 2] + nx, xy[2 * i + 3] + ny,
                                                   xy[2 * i + 2] - nx, xy[2 * i + 3] - ny, xy[2 * i] - nx, xy[2 * i + 1] - ny}};
            m += PrismMoments(ring, -d.kerb, d.kerb);
        }
        if (!m.Empty()) {
            const MomentBox box = BoxOf(m);
            Item it{};
            PlaceItem(box, lon0, lat0, mx, my, cx, cy, it);
            std::vector<double> local(2 * (b - a + 1));
            for (size_t i = a; i <= b; ++i) {
                local[2 * (i - a)] = xy[2 * i] - box.c[0];
                local[2 * (i - a) + 1] = xy[2 * i + 1] - box.c[1];
            }
            it.arena = arenaIdx;
            it.off = arena.size();
            if (EncodeRibbon(local, width, d.kerb, d.kerb, path, arena)) it.len = static_cast<uint32_t>(arena.size() - it.off);
            out.push_back(it);
        }
        a = b;
    }
}

LodBox ToBox(const Moments& m, double latc, double lonc) {
    LodBox o{};
    if (m.Empty()) return o;
    const MomentBox b = BoxOf(m);
    const double mx = std::cos(latc * kDeg) * kR * kDeg, my = kR * kDeg;
    o.lat7 = static_cast<int32_t>(std::lround((latc + b.c[1] / my) * 1e7));
    o.lon7 = static_cast<int32_t>(std::lround((lonc + b.c[0] / mx) * 1e7));
    o.zc = static_cast<float>(b.c[2]);
    o.hz = static_cast<float>(b.half[2]);
    o.a1 = static_cast<float>(b.spread[0]);
    o.a2 = static_cast<float>(b.spread[1]);
    o.heading = static_cast<float>(b.heading);
    o.cover = static_cast<float>(b.cover);
    return o;
}

std::string LevelPath(const std::string& dir, char kind, int L, const char* ext) {
    return dir + "/" + kind + std::to_string(L) + ext;
}

}  // namespace

int lod::LevelOf(double rho) {
    for (int L = kLmin; L < kLmax; ++L) {
        if (QuadDeg(L) * kMetresPerDeg >= 4.0 * rho) return L;
    }
    return kLmax;
}

void LodUnpack(const LodBuilding& b, double& lat, double& lon, double& zc, double& hz, double& a1, double& a2,
               double& heading, int& cx, int& cy) {
    lat = b.lat7 * 1e-7;
    lon = b.lon7 * 1e-7;
    zc = Unhalf(b.zc);
    hz = Unhalf(b.hz);
    a1 = Unhalf(b.a1);
    a2 = Unhalf(b.a2);
    heading = b.heading * 1e-4;
    cx = static_cast<int>(std::floor(lon / lod::kDetailDeg)) + b.dcx;
    cy = static_cast<int>(std::floor(lat / lod::kDetailDeg)) + b.dcy;
}

Moments LodBoxMoments(double lat, double lon, double zc, double hz, double a1, double a2, double heading,
                      double latc, double lonc, double cover) {
    // The box's own law (BuildingMoments: a uniform box of half-width a has variance a^2/3), set
    // at its centroid in the frame about (latc, lonc).
    Moments m;
    m.m = 8.0 * a1 * a2 * hz * cover;
    if (!(m.m > 0.0)) return Moments{};
    const double mx = std::cos(latc * kDeg) * kR * kDeg, my = kR * kDeg;
    const double c[3] = {(lon - lonc) * mx, (lat - latc) * my, zc};
    const double l1 = a1 * a1 / 3.0, l2 = a2 * a2 / 3.0, ch = std::cos(heading), sh = std::sin(heading);
    const double C[6] = {ch * ch * l1 + sh * sh * l2, sh * sh * l1 + ch * ch * l2, hz * hz / 3.0,
                         ch * sh * (l1 - l2), 0.0, 0.0};
    static const int I[6] = {0, 1, 2, 0, 0, 1}, J[6] = {0, 1, 2, 1, 2, 2};
    for (int k = 0; k < 3; ++k) m.s[k] = m.m * c[k];
    for (int k = 0; k < 6; ++k) m.S[k] = m.m * (C[k] + c[I[k]] * c[J[k]]);
    return m;
}

bool BuildBuildingLod(const BuildingStack& stack, const std::vector<RoadFile>& roads, const RoadRibbonDefaults& rd,
                      const std::string& dir, const double* box, int threads, LodBuildStats* stats, std::string* log) {
    const auto t0 = std::chrono::steady_clock::now();
    LodBuildStats st;
    std::error_code ec;
    std::filesystem::create_directories(dir, ec);
    std::filesystem::remove(dir + "/lod.json", ec);   // the manifest last: a half tree is no tree
    std::vector<std::pair<int, int>> cells = stack.Cells(lod::kDetailDeg);
    // The roads' cells too: a way lives in its first point's cell of the same 0.05 degree grid.
    if (!roads.empty()) {
        std::vector<RoadWay> probe;
        std::set<std::pair<int, int>> have(cells.begin(), cells.end());
        for (const RoadFile& rf : roads) {
            for (const std::pair<int, int>& c : rf.Cells()) {
                if (have.insert(c).second) cells.push_back(c);
            }
        }
        std::sort(cells.begin(), cells.end(), [](const std::pair<int, int>& p, const std::pair<int, int>& q) {
            return std::tie(p.second, p.first) < std::tie(q.second, q.first);
        });
    }
    if (box) {
        cells.erase(std::remove_if(cells.begin(), cells.end(),
                                   [box](const std::pair<int, int>& c) {
                                       const double lon = (c.first + 0.5) * lod::kDetailDeg, lat = (c.second + 0.5) * lod::kDetailDeg;
                                       return lon < box[0] || lat < box[1] || lon > box[2] || lat > box[3];
                                   }),
                    cells.end());
    }
    if (cells.empty()) {
        if (log) *log += "no cells to box\n";
        return false;
    }
    std::ofstream nf[lod::kLevels], bf[lod::kLevels], sf[lod::kLevels];
    std::vector<LodPage> pages[lod::kLevels];
    int64_t nodesOut[lod::kLevels] = {}, bldsOut[lod::kLevels] = {}, shapesOut[lod::kLevels] = {};
    for (int L = lod::kLmin; L <= lod::kLmax; ++L) {
        nf[Idx(L)].open(LevelPath(dir, 'N', L, ".bin"), std::ios::binary | std::ios::trunc);
        bf[Idx(L)].open(LevelPath(dir, 'B', L, ".bin"), std::ios::binary | std::ios::trunc);
        sf[Idx(L)].open(LevelPath(dir, 'S', L, ".bin"), std::ios::binary | std::ios::trunc);
        if (!nf[Idx(L)] || !bf[Idx(L)] || !sf[Idx(L)]) {
            if (log) *log += "cannot write in " + dir + "\n";
            return false;
        }
    }
    const int cellsPerRoot = static_cast<int>(std::lround(lod::QuadDeg(lod::kLmax) / lod::kDetailDeg));
    threads = (std::max)(1, threads);
    size_t a = 0;
    while (a < cells.size()) {
        const int band = FloorDiv(cells[a].second, cellsPerRoot);
        size_t b = a;
        while (b < cells.size() && FloorDiv(cells[b].second, cellsPerRoot) == band) ++b;
        // 1. Every solid of the band's cells, boxed and placed.
        std::atomic<size_t> next{a};
        std::atomic<uint64_t> solids{0}, waysN{0}, ribbonsN{0}, tunnelsN{0};
        std::mutex mx;
        std::vector<Item> items;
        std::vector<std::thread> pool;
        std::vector<std::vector<uint8_t>> arenas(threads);   // each thread's shapes, kept to the band's end
        for (int t = 0; t < threads; ++t) {
            pool.emplace_back([&, t] {
                std::vector<Item> mine;
                std::vector<uint8_t>& arena = arenas[t];
                for (size_t i; (i = next.fetch_add(1)) < b;) {
                    const int cx = cells[i].first, cy = cells[i].second;
                    const double x0 = cx * lod::kDetailDeg, y0 = cy * lod::kDetailDeg;
                    // The streaming's own box and margin, so each solid is boxed exactly once.
                    const std::vector<BuildingSolid> ss = stack.Compose(x0, y0, x0 + lod::kDetailDeg, y0 + lod::kDetailDeg, 0.002);
                    solids += ss.size();
                    for (const BuildingSolid& s : ss) {
                        Item it{};
                        if (!BoxItem(s, cx, cy, it)) continue;
                        // Its own form about the centroid its record carries (BuildingShape.h).
                        it.arena = static_cast<uint32_t>(t);
                        it.off = arena.size();
                        if (EncodeShape(s, it.rec.lat7 * 1e-7, it.rec.lon7 * 1e-7, arena)) {
                            it.len = static_cast<uint32_t>(arena.size() - it.off);
                        }
                        // A centroid just over the band's edge stays in the band (the tree is loose).
                        const int rows = 1 << (lod::kLmax - it.L);
                        it.y = std::clamp(it.y, band * rows, (band + 1) * rows - 1);
                        mine.push_back(it);
                    }
                    // The roads of the cell (by first point, as the harvest files them), as ribbons.
                    if (!roads.empty()) {
                        std::vector<RoadWay> ways;
                        for (const RoadFile& rf : roads) rf.Read(x0, y0, x0 + lod::kDetailDeg, y0 + lod::kDetailDeg, ways);
                        waysN += ways.size();
                        const size_t first = mine.size();
                        for (const RoadWay& w : ways) {
                            if (w.structure == RoadStructure::Tunnel) {
                                ++tunnelsN;
                                continue;
                            }
                            RibbonItems(w, rd, cx, cy, static_cast<uint32_t>(t), arena, mine);
                        }
                        for (size_t k = first; k < mine.size(); ++k) {
                            const int rows = 1 << (lod::kLmax - mine[k].L);
                            mine[k].y = std::clamp(mine[k].y, band * rows, (band + 1) * rows - 1);
                        }
                        ribbonsN += mine.size() - first;
                    }
                }
                std::lock_guard<std::mutex> lk(mx);
                items.insert(items.end(), mine.begin(), mine.end());
            });
        }
        for (std::thread& t : pool) t.join();
        st.solids += solids.load();
        st.ways += waysN.load();
        st.ribbons += ribbonsN.load();
        st.tunnels += tunnelsN.load();
        st.cells += b - a;
        st.kept += items.size();
        // By node, and within a node the largest first: the walk stops at the first under a pixel.
        std::sort(items.begin(), items.end(), [](const Item& p, const Item& q) {
            return std::make_tuple(p.L, p.y, p.x, -p.rho) < std::make_tuple(q.L, q.y, q.x, -q.rho);
        });
        // 2. The tree, finest level first: a node's own fold from its items, its descendants' from
        // its children's (own + desc) moved into its frame -- FOLD and FRAME.
        struct Acc {
            int x, y;
            size_t i0, i1;        // its own items
            Moments all;          // about its own centre
            LodNode node;
        };
        std::vector<Acc> prev, cur;
        std::vector<std::vector<Acc>> levels(lod::kLevels);
        size_t ip = 0;
        for (int L = lod::kLmin; L <= lod::kLmax; ++L) {
            const double Q = lod::QuadDeg(L), Qc = lod::QuadDeg(L - 1);
            // Children grouped by their parent's quad.
            std::vector<std::tuple<int, int, size_t>> kids;   // (py, px, index into prev)
            for (size_t k = 0; k < prev.size(); ++k) kids.push_back({FloorDiv(prev[k].y, 2), FloorDiv(prev[k].x, 2), k});
            std::sort(kids.begin(), kids.end());
            size_t i0 = ip;
            while (ip < items.size() && items[ip].L == L) ++ip;
            size_t oi = i0, ki = 0;
            cur.clear();
            while (oi < ip || ki < kids.size()) {
                int y, x;
                if (ki >= kids.size() || (oi < ip && std::tie(items[oi].y, items[oi].x) <= std::tie(std::get<0>(kids[ki]), std::get<1>(kids[ki])))) {
                    y = items[oi].y;
                    x = items[oi].x;
                } else {
                    y = std::get<0>(kids[ki]);
                    x = std::get<1>(kids[ki]);
                }
                Acc acc{};
                acc.x = x;
                acc.y = y;
                const double latc = (y + 0.5) * Q, lonc = (x + 0.5) * Q;
                Moments own, desc;
                float rhoMin = 1e30f;
                acc.i0 = oi;
                for (; oi < ip && items[oi].y == y && items[oi].x == x; ++oi) {
                    double lat, lon, zc, hz, a1, a2, h;
                    int cx, cy;
                    LodUnpack(items[oi].rec, lat, lon, zc, hz, a1, a2, h, cx, cy);
                    own += LodBoxMoments(lat, lon, zc, hz, a1, a2, h, latc, lonc);
                    rhoMin = (std::min)(rhoMin, items[oi].rho);
                }
                acc.i1 = oi;
                for (; ki < kids.size() && std::get<0>(kids[ki]) == y && std::get<1>(kids[ki]) == x; ++ki) {
                    const Acc& c = prev[std::get<2>(kids[ki])];
                    const double clat = (c.y + 0.5) * Qc, clon = (c.x + 0.5) * Qc;
                    const double cmx = std::cos(clat * kDeg) * kR * kDeg, my = kR * kDeg;
                    const double o[3] = {(lonc - clon) * cmx, (latc - clat) * my, 0.0};
                    desc += c.all.About(o);
                }
                acc.all = own;
                acc.all += desc;
                acc.node.x = x;
                acc.node.y = y;
                acc.node.ownCount = static_cast<uint32_t>(acc.i1 - acc.i0);
                acc.node.ownRhoMin = acc.node.ownCount ? rhoMin : 0.0f;
                acc.node.own = ToBox(own, latc, lonc);
                acc.node.desc = ToBox(desc, latc, lonc);
                cur.push_back(acc);
            }
            levels[Idx(L)] = cur;
            prev.swap(cur);
        }
        // 3. Written level by level in page order, each node's own buildings beside it.
        for (int L = lod::kLmin; L <= lod::kLmax; ++L) {
            std::vector<Acc>& v = levels[Idx(L)];
            const int PY = lod::PageY(L);
            auto key = [PY](const Acc& n) { return std::make_tuple(FloorDiv(n.y, PY), FloorDiv(n.x, lod::kPageX), n.y, n.x); };
            std::sort(v.begin(), v.end(), [&](const Acc& p, const Acc& q) { return key(p) < key(q); });
            for (size_t i = 0; i < v.size();) {
                const int py = FloorDiv(v[i].y, PY), px = FloorDiv(v[i].x, lod::kPageX);
                LodPage pg{py, px, nodesOut[Idx(L)], 0, bldsOut[Idx(L)], 0, shapesOut[Idx(L)], 0};
                for (; i < v.size() && FloorDiv(v[i].y, PY) == py && FloorDiv(v[i].x, lod::kPageX) == px; ++i) {
                    v[i].node.ownFirst = bldsOut[Idx(L)];
                    for (size_t k = v[i].i0; k < v[i].i1; ++k) {
                        bf[Idx(L)].write(reinterpret_cast<const char*>(&items[k].rec), sizeof(LodBuilding));
                        // One shape record a building: its own, or an empty head (it keeps its box).
                        static const ShapeHead kNone{};
                        if (items[k].len) {
                            sf[Idx(L)].write(reinterpret_cast<const char*>(arenas[items[k].arena].data() + items[k].off), items[k].len);
                            shapesOut[Idx(L)] += items[k].len;
                        } else {
                            sf[Idx(L)].write(reinterpret_cast<const char*>(&kNone), sizeof(kNone));
                            shapesOut[Idx(L)] += sizeof(kNone);
                        }
                    }
                    bldsOut[Idx(L)] += v[i].i1 - v[i].i0;
                    nf[Idx(L)].write(reinterpret_cast<const char*>(&v[i].node), sizeof(LodNode));
                    ++nodesOut[Idx(L)];
                    st.owned[Idx(L)] += v[i].i1 - v[i].i0;
                }
                pg.nodeCount = nodesOut[Idx(L)] - pg.nodeFirst;
                pg.bldCount = bldsOut[Idx(L)] - pg.bldFirst;
                pg.shapeBytes = shapesOut[Idx(L)] - pg.shapeFirst;
                pages[Idx(L)].push_back(pg);
            }
            st.nodes[Idx(L)] += v.size();
        }
        const double sec = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
        Log("[lod] band %.1f..%.1f N: %zu cells, %llu solids so far, %.0f s (%zu of %zu cells)",
            band * lod::QuadDeg(lod::kLmax), (band + 1) * lod::QuadDeg(lod::kLmax), b - a,
            static_cast<unsigned long long>(st.solids), sec, b, cells.size());
        a = b;
    }
    for (int L = lod::kLmin; L <= lod::kLmax; ++L) {
        nf[Idx(L)].close();
        bf[Idx(L)].close();
        sf[Idx(L)].close();
        std::ofstream fi(LevelPath(dir, 'P', L, ".idx"), std::ios::binary | std::ios::trunc);
        fi.write(reinterpret_cast<const char*>(pages[Idx(L)].data()),
                 static_cast<std::streamsize>(pages[Idx(L)].size() * sizeof(LodPage)));
        if (!fi) {
            if (log) *log += "cannot write " + LevelPath(dir, 'P', L, ".idx") + "\n";
            return false;
        }
    }
    std::ostringstream m;
    m << "{\n  \"format\": \"GALOD04\",\n  \"lmin\": " << lod::kLmin << ",\n  \"lmax\": " << lod::kLmax
      << ",\n  \"detailDeg\": " << lod::kDetailDeg << ",\n  \"sources\": [";
    for (size_t k = 0; k < stack.Sources(); ++k) m << (k ? ", " : "") << "\"" << stack.Name(k) << "\"";
    m << "],\n  \"roads\": [";
    for (size_t k = 0; k < roads.size(); ++k) m << (k ? ", " : "") << "\"" << roads[k].Path() << "\"";
    m << "],\n  \"ways\": " << st.ways << ",\n  \"ribbons\": " << st.ribbons << ",\n  \"tunnels\": " << st.tunnels
      << ",\n  \"laneWidth\": " << rd.laneWidth << ",\n  \"defaultLanes\": " << rd.defaultLanes << ",\n  \"pathWidth\": " << rd.pathWidth
      << ",\n  \"kerb\": " << rd.kerb
      << ",\n  \"cells\": " << st.cells << ",\n  \"solids\": " << st.solids << ",\n  \"kept\": " << st.kept << "\n}\n";
    std::ofstream(dir + "/lod.json", std::ios::binary | std::ios::trunc) << m.str();
    st.seconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
    if (stats) *stats = st;
    return true;
}

bool BuildingLodFile::Open(const std::string& dir, std::string* why) {
    m_ok = false;
    m_dir = dir;
    std::ifstream f(dir + "/lod.json", std::ios::binary);
    if (!f) {
        if (why) *why = "no tree at " + dir + " (build it: --tool building-lod)";
        return false;
    }
    std::stringstream ss;
    ss << f.rdbuf();
    std::string err;
    const JsonValue m = JsonParser::Parse(ss.str(), &err);
    const std::string fmt = m.Str("format");
    m_format = fmt == "GALOD04" ? 4 : fmt == "GALOD03" ? 3 : 0;
    if (!err.empty() || !m_format) {
        if (why) *why = "not a GALOD03/04 manifest (rebuild it: --tool building-lod)" + (err.empty() ? "" : ": " + err);
        return false;
    }
    // A GALOD03 page is a GALOD04 page's first 40 bytes: read as such, it has no shapes.
    const size_t rec = m_format >= 4 ? sizeof(LodPage) : offsetof(LodPage, shapeFirst);
    for (int L = lod::kLmin; L <= lod::kLmax; ++L) {
        std::ifstream fi(LevelPath(dir, 'P', L, ".idx"), std::ios::binary);
        LodPage p{};
        m_idx[Idx(L)].clear();
        while (fi.read(reinterpret_cast<char*>(&p), static_cast<std::streamsize>(rec))) m_idx[Idx(L)].push_back(p);
    }
    m_ok = true;
    return true;
}

const LodPage* BuildingLodFile::Find(int L, int px, int py) const {
    if (L < lod::kLmin || L > lod::kLmax) return nullptr;
    const std::vector<LodPage>& v = m_idx[Idx(L)];
    auto it = std::lower_bound(v.begin(), v.end(), std::make_pair(py, px), [](const LodPage& t, const std::pair<int, int>& q) {
        return std::make_pair(t.py, t.px) < q;
    });
    return (it != v.end() && it->py == py && it->px == px) ? &*it : nullptr;
}

bool BuildingLodFile::Read(int L, const LodPage& p, std::vector<LodNode>& nodes, std::vector<LodBuilding>& blds) const {
    nodes.resize(static_cast<size_t>(p.nodeCount));
    blds.resize(static_cast<size_t>(p.bldCount));
    std::ifstream fn(LevelPath(m_dir, 'N', L, ".bin"), std::ios::binary), fb(LevelPath(m_dir, 'B', L, ".bin"), std::ios::binary);
    fn.seekg(static_cast<std::streamoff>(p.nodeFirst * sizeof(LodNode)));
    fb.seekg(static_cast<std::streamoff>(p.bldFirst * sizeof(LodBuilding)));
    return fn.read(reinterpret_cast<char*>(nodes.data()), static_cast<std::streamsize>(nodes.size() * sizeof(LodNode))) &&
           (blds.empty() || fb.read(reinterpret_cast<char*>(blds.data()), static_cast<std::streamsize>(blds.size() * sizeof(LodBuilding))));
}

bool BuildingLodFile::ReadShapes(int L, const LodPage& p, std::vector<uint8_t>& shapes) const {
    shapes.clear();
    if (m_format < 4 || p.shapeBytes <= 0) return m_format >= 4;
    shapes.resize(static_cast<size_t>(p.shapeBytes));
    std::ifstream fs(LevelPath(m_dir, 'S', L, ".bin"), std::ios::binary);
    fs.seekg(static_cast<std::streamoff>(p.shapeFirst));
    return static_cast<bool>(fs.read(reinterpret_cast<char*>(shapes.data()), static_cast<std::streamsize>(shapes.size())));
}

}  // namespace ga
