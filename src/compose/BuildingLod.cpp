#include "compose/BuildingLod.h"

#include "compose/BuildingMoments.h"
#include "core/Common.h"
#include "core/Json.h"

#include <DirectXPackedVector.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <mutex>
#include <sstream>
#include <thread>
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
};

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
    const MomentBox b = BoxOf(m);
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
    return true;
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

bool BuildBuildingLod(const BuildingStack& stack, const std::string& dir, const double* box, int threads,
                      LodBuildStats* stats, std::string* log) {
    const auto t0 = std::chrono::steady_clock::now();
    LodBuildStats st;
    std::error_code ec;
    std::filesystem::create_directories(dir, ec);
    std::filesystem::remove(dir + "/lod.json", ec);   // the manifest last: a half tree is no tree
    std::vector<std::pair<int, int>> cells = stack.Cells(lod::kDetailDeg);
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
    std::ofstream nf[lod::kLevels], bf[lod::kLevels];
    std::vector<LodPage> pages[lod::kLevels];
    int64_t nodesOut[lod::kLevels] = {}, bldsOut[lod::kLevels] = {};
    for (int L = lod::kLmin; L <= lod::kLmax; ++L) {
        nf[Idx(L)].open(LevelPath(dir, 'N', L, ".bin"), std::ios::binary | std::ios::trunc);
        bf[Idx(L)].open(LevelPath(dir, 'B', L, ".bin"), std::ios::binary | std::ios::trunc);
        if (!nf[Idx(L)] || !bf[Idx(L)]) {
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
        std::atomic<uint64_t> solids{0};
        std::mutex mx;
        std::vector<Item> items;
        std::vector<std::thread> pool;
        for (int t = 0; t < threads; ++t) {
            pool.emplace_back([&] {
                std::vector<Item> mine;
                for (size_t i; (i = next.fetch_add(1)) < b;) {
                    const int cx = cells[i].first, cy = cells[i].second;
                    const double x0 = cx * lod::kDetailDeg, y0 = cy * lod::kDetailDeg;
                    // The streaming's own box and margin, so each solid is boxed exactly once.
                    const std::vector<BuildingSolid> ss = stack.Compose(x0, y0, x0 + lod::kDetailDeg, y0 + lod::kDetailDeg, 0.002);
                    solids += ss.size();
                    for (const BuildingSolid& s : ss) {
                        Item it{};
                        if (!BoxItem(s, cx, cy, it)) continue;
                        // A centroid just over the band's edge stays in the band (the tree is loose).
                        const int rows = 1 << (lod::kLmax - it.L);
                        it.y = std::clamp(it.y, band * rows, (band + 1) * rows - 1);
                        mine.push_back(it);
                    }
                }
                std::lock_guard<std::mutex> lk(mx);
                items.insert(items.end(), mine.begin(), mine.end());
            });
        }
        for (std::thread& t : pool) t.join();
        st.solids += solids.load();
        st.cells += b - a;
        st.kept += items.size();
        std::sort(items.begin(), items.end(),
                  [](const Item& p, const Item& q) { return std::tie(p.L, p.y, p.x) < std::tie(q.L, q.y, q.x); });
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
                LodPage pg{py, px, nodesOut[Idx(L)], 0, bldsOut[Idx(L)], 0};
                for (; i < v.size() && FloorDiv(v[i].y, PY) == py && FloorDiv(v[i].x, lod::kPageX) == px; ++i) {
                    v[i].node.ownFirst = bldsOut[Idx(L)];
                    for (size_t k = v[i].i0; k < v[i].i1; ++k) {
                        bf[Idx(L)].write(reinterpret_cast<const char*>(&items[k].rec), sizeof(LodBuilding));
                    }
                    bldsOut[Idx(L)] += v[i].i1 - v[i].i0;
                    nf[Idx(L)].write(reinterpret_cast<const char*>(&v[i].node), sizeof(LodNode));
                    ++nodesOut[Idx(L)];
                    st.owned[Idx(L)] += v[i].i1 - v[i].i0;
                }
                pg.nodeCount = nodesOut[Idx(L)] - pg.nodeFirst;
                pg.bldCount = bldsOut[Idx(L)] - pg.bldFirst;
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
        std::ofstream fi(LevelPath(dir, 'P', L, ".idx"), std::ios::binary | std::ios::trunc);
        fi.write(reinterpret_cast<const char*>(pages[Idx(L)].data()),
                 static_cast<std::streamsize>(pages[Idx(L)].size() * sizeof(LodPage)));
        if (!fi) {
            if (log) *log += "cannot write " + LevelPath(dir, 'P', L, ".idx") + "\n";
            return false;
        }
    }
    std::ostringstream m;
    m << "{\n  \"format\": \"GALOD03\",\n  \"lmin\": " << lod::kLmin << ",\n  \"lmax\": " << lod::kLmax
      << ",\n  \"detailDeg\": " << lod::kDetailDeg << ",\n  \"sources\": [";
    for (size_t k = 0; k < stack.Sources(); ++k) m << (k ? ", " : "") << "\"" << stack.Name(k) << "\"";
    m << "],\n  \"cells\": " << st.cells << ",\n  \"solids\": " << st.solids << ",\n  \"kept\": " << st.kept << "\n}\n";
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
    if (!err.empty() || m.Str("format") != "GALOD03") {
        if (why) *why = "not a GALOD03 manifest (rebuild it: --tool building-lod)" + (err.empty() ? "" : ": " + err);
        return false;
    }
    for (int L = lod::kLmin; L <= lod::kLmax; ++L) {
        std::ifstream fi(LevelPath(dir, 'P', L, ".idx"), std::ios::binary);
        LodPage p{};
        m_idx[Idx(L)].clear();
        while (fi.read(reinterpret_cast<char*>(&p), sizeof(p))) m_idx[Idx(L)].push_back(p);
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

}  // namespace ga
