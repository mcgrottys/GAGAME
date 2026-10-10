#include "compose/BuildingLod.h"

#include "compose/BuildingMoments.h"
#include "core/Common.h"
#include "core/Json.h"

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

namespace ga {

namespace {

constexpr double kDeg = 3.14159265358979323846 / 180.0;
constexpr double kR = 6371008.8;   // the prisms' own (BuildingLayer.cpp)

int FloorDiv(int a, int n) { return a >= 0 ? a / n : -((-a + n - 1) / n); }
int CellsPerTile(int k) { return static_cast<int>(std::lround(lod::TileDeg(k) / lod::kDetailDeg)); }

// One solid's moment box, in degrees and metres; false for a solid with no volume.
bool BoxRecord(const BuildingSolid& s, double rho0, LodRecord& r, int& level) {
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
    level = (std::min)(LodLevel(b.Radius(), rho0), lod::kMaxLevel);
    r.lat7 = static_cast<int32_t>(std::lround((lat0 + b.c[1] / my) * 1e7));
    r.lon7 = static_cast<int32_t>(std::lround((lon0 + b.c[0] / mx) * 1e7));
    r.cx = static_cast<int32_t>(std::floor(lon0 / lod::kDetailDeg));
    r.cy = static_cast<int32_t>(std::floor(lat0 / lod::kDetailDeg));
    r.zc = static_cast<float>(b.c[2]);
    r.hz = static_cast<float>(b.half[2]);
    r.a1 = static_cast<float>(b.half[0]);
    r.a2 = static_cast<float>(b.half[1]);
    r.heading = static_cast<float>(b.heading);
    return true;
}

std::string LevelPath(const std::string& dir, int k, const char* ext) {
    return dir + "/L" + std::to_string(k) + ext;
}

}  // namespace

bool BuildBuildingLod(const BuildingStack& stack, const std::string& dir, double rho0, const double* box,
                      int threads, LodBuildStats* stats, std::string* log) {
    const auto t0 = std::chrono::steady_clock::now();
    LodBuildStats st;
    std::error_code ec;
    std::filesystem::create_directories(dir, ec);
    std::filesystem::remove(dir + "/lod.json", ec);   // the manifest last: a half pyramid is no pyramid
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
    std::ofstream bin[lod::kMaxLevel + 1];
    std::vector<LodTile> idx[lod::kMaxLevel + 1];
    int64_t written[lod::kMaxLevel + 1] = {};
    for (int k = lod::kMinLevel; k <= lod::kMaxLevel; ++k) {
        bin[k].open(LevelPath(dir, k, ".bin"), std::ios::binary | std::ios::trunc);
        if (!bin[k]) {
            if (log) *log += "cannot write " + LevelPath(dir, k, ".bin") + "\n";
            return false;
        }
    }
    // Bands of rows that no level's tile crosses: the coarsest tile's height.
    const int bandRows = CellsPerTile(lod::kMaxLevel);
    threads = (std::max)(1, threads);
    size_t a = 0;
    while (a < cells.size()) {
        const int band = FloorDiv(cells[a].second, bandRows);
        size_t b = a;
        while (b < cells.size() && FloorDiv(cells[b].second, bandRows) == band) ++b;
        std::atomic<size_t> next{a};
        std::mutex mx;
        std::vector<LodRecord> recs[lod::kMaxLevel + 1];
        std::atomic<uint64_t> solids{0};
        std::vector<std::thread> pool;
        for (int t = 0; t < threads; ++t) {
            pool.emplace_back([&] {
                std::vector<LodRecord> mine[lod::kMaxLevel + 1];
                for (size_t i; (i = next.fetch_add(1)) < b;) {
                    const int cx = cells[i].first, cy = cells[i].second;
                    const double x0 = cx * lod::kDetailDeg, y0 = cy * lod::kDetailDeg;
                    // The streaming's own box and margin, so each solid is boxed exactly once.
                    const std::vector<BuildingSolid> ss = stack.Compose(x0, y0, x0 + lod::kDetailDeg, y0 + lod::kDetailDeg, 0.002);
                    solids += ss.size();
                    for (const BuildingSolid& s : ss) {
                        LodRecord r{};
                        int k = 0;
                        if (!BoxRecord(s, rho0, r, k) || k < lod::kMinLevel) continue;
                        r.cx = cx;   // the cell that composed it IS its detail cell
                        r.cy = cy;
                        mine[k].push_back(r);
                    }
                }
                std::lock_guard<std::mutex> lk(mx);
                for (int k = lod::kMinLevel; k <= lod::kMaxLevel; ++k) recs[k].insert(recs[k].end(), mine[k].begin(), mine[k].end());
            });
        }
        for (std::thread& t : pool) t.join();
        st.solids += solids.load();
        st.cells += b - a;
        for (int k = lod::kMinLevel; k <= lod::kMaxLevel; ++k) {
            const int n = CellsPerTile(k);
            auto key = [n](const LodRecord& r) {
                return std::make_tuple(FloorDiv(r.cy, n), FloorDiv(r.cx, n), r.cy, r.cx);
            };
            std::sort(recs[k].begin(), recs[k].end(), [&](const LodRecord& p, const LodRecord& q) { return key(p) < key(q); });
            for (size_t i = 0; i < recs[k].size();) {
                const int ty = FloorDiv(recs[k][i].cy, n), tx = FloorDiv(recs[k][i].cx, n);
                size_t j = i;
                while (j < recs[k].size() && FloorDiv(recs[k][j].cy, n) == ty && FloorDiv(recs[k][j].cx, n) == tx) ++j;
                idx[k].push_back({ty, tx, written[k], static_cast<int64_t>(j - i)});
                bin[k].write(reinterpret_cast<const char*>(&recs[k][i]), static_cast<std::streamsize>((j - i) * sizeof(LodRecord)));
                written[k] += static_cast<int64_t>(j - i);
                i = j;
            }
            st.kept[k] += recs[k].size();
        }
        const double sec = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
        Log("[lod] band %.1f..%.1f N: %zu cells, %llu solids so far, %.0f s (%zu of %zu cells)",
            band * bandRows * lod::kDetailDeg, (band + 1) * bandRows * lod::kDetailDeg, b - a,
            static_cast<unsigned long long>(st.solids), sec, b, cells.size());
        a = b;
    }
    for (int k = lod::kMinLevel; k <= lod::kMaxLevel; ++k) {
        bin[k].close();
        std::ofstream fi(LevelPath(dir, k, ".idx"), std::ios::binary | std::ios::trunc);
        fi.write(reinterpret_cast<const char*>(idx[k].data()), static_cast<std::streamsize>(idx[k].size() * sizeof(LodTile)));
        st.tiles[k] = idx[k].size();
        if (!fi) {
            if (log) *log += "cannot write " + LevelPath(dir, k, ".idx") + "\n";
            return false;
        }
    }
    std::ostringstream m;
    m << "{\n  \"format\": \"GALOD01\",\n  \"rho0\": " << rho0 << ",\n  \"minLevel\": " << lod::kMinLevel
      << ",\n  \"maxLevel\": " << lod::kMaxLevel << ",\n  \"detailDeg\": " << lod::kDetailDeg << ",\n  \"sources\": [";
    for (size_t k = 0; k < stack.Sources(); ++k) m << (k ? ", " : "") << "\"" << stack.Name(k) << "\"";
    m << "],\n  \"cells\": " << st.cells << ",\n  \"solids\": " << st.solids << ",\n  \"kept\": [";
    for (int k = lod::kMinLevel; k <= lod::kMaxLevel; ++k) m << (k > lod::kMinLevel ? ", " : "") << st.kept[k];
    m << "]\n}\n";
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
        if (why) *why = "no pyramid at " + dir + " (build it: --tool building-lod)";
        return false;
    }
    std::stringstream ss;
    ss << f.rdbuf();
    std::string err;
    const JsonValue m = JsonParser::Parse(ss.str(), &err);
    if (!err.empty() || m.Str("format") != "GALOD01") {
        if (why) *why = "not a GALOD01 manifest" + (err.empty() ? "" : ": " + err);
        return false;
    }
    m_rho0 = m.Num("rho0", 4.0);
    for (int k = lod::kMinLevel; k <= lod::kMaxLevel; ++k) {
        std::ifstream fi(LevelPath(dir, k, ".idx"), std::ios::binary);
        LodTile t{};
        m_idx[k].clear();
        while (fi.read(reinterpret_cast<char*>(&t), sizeof(t))) m_idx[k].push_back(t);
    }
    m_ok = true;
    return true;
}

const LodTile* BuildingLodFile::Find(int k, int tx, int ty) const {
    if (k < lod::kMinLevel || k > lod::kMaxLevel) return nullptr;
    const std::vector<LodTile>& v = m_idx[k];
    auto it = std::lower_bound(v.begin(), v.end(), std::make_pair(ty, tx), [](const LodTile& t, const std::pair<int, int>& q) {
        return std::make_pair(t.ty, t.tx) < q;
    });
    return (it != v.end() && it->ty == ty && it->tx == tx) ? &*it : nullptr;
}

std::vector<LodRecord> BuildingLodFile::Read(int k, const LodTile& t) const {
    std::vector<LodRecord> out(static_cast<size_t>(t.count));
    std::ifstream f(LevelPath(m_dir, k, ".bin"), std::ios::binary);
    f.seekg(static_cast<std::streamoff>(t.offset * sizeof(LodRecord)));
    if (!f.read(reinterpret_cast<char*>(out.data()), static_cast<std::streamsize>(out.size() * sizeof(LodRecord)))) out.clear();
    return out;
}

}  // namespace ga
