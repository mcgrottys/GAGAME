// ================================================================================================
//  planet_buildings.cpp - harvest_planet_buildings.py in C++: the same solids, the same bytes.
//
//  The Python harvester decodes ~630 M ways one at a time and runs ~3 hours on 14 processes; this
//  reads the same osmium output (building ways carrying their own coordinates) through libosmium's
//  threaded PBF reader and does the per-way work in C++. Every law is the Python's, line for line,
//  so the gate is byte equality with it (harvester/cpp/README.md):
//    heights to metres by the same patterns, refused not guessed; floors as counts; the FIXED roof
//    vocabulary (254 = other); closed ways only, the closing node dropped; multipolygons stitched by
//    harvest_buildings.stitch's exact order; records bucketed by 2.5 degree tile, each bucket sorted
//    by (iy, ix, id), the buckets concatenated in the order of their names; the cell index as the
//    binary sidecar (int32 ix, int32 iy, int64 offset, int64 count, by (iy, ix)).
//
//  Usage: planet_buildings <buildings-with-locations.osm.pbf> <out dir> [threads]
//  Licence of the data: ODbL 1.0, "(c) OpenStreetMap contributors".
// ================================================================================================
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <osmium/handler.hpp>
#include <osmium/io/pbf_input.hpp>
#include <osmium/osm/relation.hpp>
#include <osmium/osm/way.hpp>
#include <osmium/visitor.hpp>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <limits>
#include <map>
#include <mutex>
#include <regex>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

namespace fs = std::filesystem;

namespace {

constexpr double kCellDeg = 0.05;
constexpr int kBucket = 50;   // cells a bucket spans on each axis: 2.5 degrees
constexpr float kNaN = std::numeric_limits<float>::quiet_NaN();
const char* const kRoofs[] = {"flat", "skillion", "gabled", "half-hipped", "hipped", "pyramidal", "gambrel",
                              "mansard", "dome", "onion", "round", "saltbox", "quadruple_saltbox", "sawtooth",
                              "cone", "crosspitched", "side_hipped", "side_half-hipped", "double_saltbox",
                              "gabled_height_moved", "butterfly", "many"};

auto t0 = std::chrono::steady_clock::now();
double Secs() { return std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count(); }

// ---- normalize: harvest_buildings.metres / count, the same patterns -----------------------------
std::string Strip(const std::string& s) {   // Python's str.strip(): whitespace both ends
    size_t a = 0, b = s.size();
    while (a < b && std::isspace(static_cast<unsigned char>(s[a]))) ++a;
    while (b > a && std::isspace(static_cast<unsigned char>(s[b - 1]))) --b;
    return s.substr(a, b - a);
}
double PyFloat(std::string s) {   // float() of a matched number: "," already replaced where Python does
    return std::strtod(s.c_str(), nullptr);
}
bool Metres(const char* text, double& out) {
    static const std::string num = R"([-+]?\d+(?:[.,]\d+)?)";
    static const std::regex m1("(" + num + R"()\s*(m|meters?|metres?)?)");
    static const std::regex m2("(" + num + R"()\s*(ft|feet|foot|'))");
    static const std::regex m3("(" + num + R"()\s*'\s*()" + num + R"()\s*("|in)?)");
    std::string t = Strip(text);
    std::transform(t.begin(), t.end(), t.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    t.erase(std::remove(t.begin(), t.end(), '~'), t.end());
    std::smatch g;
    auto comma = [](std::string x) { std::replace(x.begin(), x.end(), ',', '.'); return x; };
    if (std::regex_match(t, g, m1)) { out = PyFloat(comma(g[1])); return true; }
    if (std::regex_match(t, g, m2)) { out = PyFloat(comma(g[1])) * 0.3048; return true; }
    // The feet-and-inches form: Python's float() is given the groups as written (a comma fails it).
    if (std::regex_match(t, g, m3)) {
        const std::string a = g[1], b = g[2];
        if (a.find(',') != std::string::npos || b.find(',') != std::string::npos) return false;   // ValueError
        out = PyFloat(a) * 0.3048 + PyFloat(b) * 0.0254;
        return true;
    }
    return false;
}
bool Count(const char* text, double& out) {
    std::string t = Strip(text);
    std::replace(t.begin(), t.end(), ',', '.');
    if (t.empty()) return false;
    char* end = nullptr;
    const double v = std::strtod(t.c_str(), &end);
    if (end != t.c_str() + t.size()) return false;
    if (!(v >= 0)) return false;   // negative, or NaN
    out = v;
    return true;
}

struct Refused {
    uint64_t height = 0, min_height = 0, roof_height = 0, levels = 0, min_level = 0;
};
struct Attrs {
    uint8_t roof = 255;
    float h = kNaN, mh = kNaN, lv = kNaN, mlv = kNaN, rh = kNaN;
};
Attrs AttrsOf(const osmium::TagList& t, Refused& r) {
    Attrs a;
    auto length = [&](const char* k, uint64_t& refused) -> float {
        const char* v = t[k];
        if (!v) return kNaN;
        double m;
        if (!Metres(v, m)) { ++refused; return kNaN; }
        return static_cast<float>(m);
    };
    auto levels = [&](const char* k, uint64_t& refused) -> float {
        const char* v = t[k];
        if (!v) return kNaN;
        double c;
        if (!Count(v, c)) { ++refused; return kNaN; }
        return static_cast<float>(c);
    };
    if (const char* rs = t["roof:shape"]) {
        a.roof = 254;
        for (uint8_t i = 0; i < sizeof(kRoofs) / sizeof(kRoofs[0]); ++i) {
            if (std::strcmp(rs, kRoofs[i]) == 0) { a.roof = i; break; }
        }
    }
    a.h = length("height", r.height);
    a.mh = length("min_height", r.min_height);
    a.lv = levels("building:levels", r.levels);
    a.mlv = levels("building:min_level", r.min_level);
    a.rh = length("roof:height", r.roof_height);
    return a;
}
int KindOf(const osmium::TagList& t) {   // harvest_buildings.is_building
    const char* b = t["building"];
    if (b && std::strcmp(b, "no") != 0) return 0;
    const char* p = t["building:part"];
    if (p && std::strcmp(p, "no") != 0) return 1;
    return -1;
}

// ---- records and buckets ----------------------------------------------------------------------
struct XY {
    int32_t x, y;
};
int64_t FloorDiv(int64_t a, int64_t b) { return a / b - ((a % b != 0) && ((a < 0) != (b < 0))); }
void CellOf(int32_t x, int32_t y, int32_t& iy, int32_t& ix) {   // harvest_buildings.cell_of
    iy = static_cast<int32_t>(std::floor(static_cast<double>(y) * 1e-7 / kCellDeg));
    ix = static_cast<int32_t>(std::floor(static_cast<double>(x) * 1e-7 / kCellDeg));
}
std::string BucketOf(int32_t iy, int32_t ix) {   // f"{iy // B:+04d}_{ix // B:+04d}"
    char s[32];
    snprintf(s, sizeof(s), "%+04lld_%+04lld", static_cast<long long>(FloorDiv(iy, kBucket)),
             static_cast<long long>(FloorDiv(ix, kBucket)));
    return s;
}
template <class T>
void Put(std::string& s, const T& v) { s.append(reinterpret_cast<const char*>(&v), sizeof(T)); }

std::string Record(int64_t id, int kind, const Attrs& a, const std::vector<std::pair<int, std::vector<XY>>>& geo) {
    std::string r;
    Put(r, id);
    Put(r, static_cast<uint8_t>(kind));
    Put(r, a.roof);
    Put(r, static_cast<uint16_t>(geo.size()));
    for (float f : {a.h, a.mh, a.lv, a.mlv, a.rh}) Put(r, f);
    for (const auto& [outer, xy] : geo) {
        Put(r, static_cast<uint32_t>(xy.size()));
        Put(r, static_cast<uint8_t>(outer));
        r.append(3, '\0');
        r.append(reinterpret_cast<const char*>(xy.data()), xy.size() * sizeof(XY));
    }
    return r;
}

class Buckets {
public:
    Buckets(fs::path dir, std::string tag) : m_dir(std::move(dir)), m_tag(std::move(tag)) {}
    ~Buckets() { Flush(); }
    void Add(int32_t iy, int32_t ix, int64_t id, const std::string& rec) {
        std::string& b = m_buf[BucketOf(iy, ix)];
        Put(b, iy);
        Put(b, ix);
        Put(b, id);
        Put(b, static_cast<uint32_t>(rec.size()));
        b += rec;
        m_size += rec.size() + 20;
        if (m_size > (512u << 20)) Flush();
    }
    void Flush() {
        for (auto& [name, data] : m_buf) {
            std::ofstream f(m_dir / (name + "." + m_tag + ".part"), std::ios::binary | std::ios::app);
            f.write(data.data(), static_cast<std::streamsize>(data.size()));
        }
        m_buf.clear();
        m_size = 0;
    }

private:
    fs::path m_dir;
    std::string m_tag;
    std::unordered_map<std::string, std::string> m_buf;
    size_t m_size = 0;
};

// ---- pass 1: building ways, and the building multipolygons -----------------------------------
struct Rel {
    int64_t id;
    int kind;
    Attrs attrs;
    std::vector<std::pair<int64_t, bool>> members;   // way id, outer
};
struct Pass1 : osmium::handler::Handler {
    Buckets& out;
    Refused refused;
    uint64_t ways = 0, openWays = 0, noLocations = 0;
    std::vector<Rel> rels;
    explicit Pass1(Buckets& b) : out(b) {}
    void way(const osmium::Way& w) {
        const int kind = KindOf(w.tags());
        if (kind < 0) return;
        const auto& nr = w.nodes();
        if (nr.size() < 4 || nr.front().ref() != nr.back().ref()) { ++openWays; return; }
        std::vector<XY> xy;
        xy.reserve(nr.size() - 1);
        for (size_t i = 0; i + 1 < nr.size(); ++i) {   // drop the closing node
            const osmium::Location& l = nr[i].location();
            if (!l.valid()) { ++noLocations; return; }
            xy.push_back({l.x(), l.y()});
        }
        const Attrs a = AttrsOf(w.tags(), refused);
        int32_t iy, ix;
        CellOf(xy[0].x, xy[0].y, iy, ix);
        out.Add(iy, ix, w.id(), Record(w.id(), kind, a, {{1, std::move(xy)}}));
        if (++ways % 50000000 == 0) printf("[planet++] pass 1: %llu building ways (%.0f s)\n", (unsigned long long)ways, Secs());
    }
    void relation(const osmium::Relation& r) {
        const int kind = KindOf(r.tags());
        const char* type = r.tags()["type"];
        if (kind < 0 || !type || std::strcmp(type, "multipolygon") != 0) return;
        Rel rel{r.id(), kind, AttrsOf(r.tags(), refused), {}};
        for (const osmium::RelationMember& m : r.members()) {
            if (m.type() == osmium::item_type::way) rel.members.push_back({m.ref(), std::strcmp(m.role(), "inner") != 0});
        }
        rels.push_back(std::move(rel));
    }
};

// ---- pass 2: the member ways' coordinates, and the stitch ------------------------------------
struct Node {
    int64_t id;
    int32_t x, y;
    bool operator==(const Node& o) const { return id == o.id && x == o.x && y == o.y; }
};
using Piece = std::vector<Node>;
struct Pass2 : osmium::handler::Handler {
    const std::vector<int64_t>& want;   // sorted
    std::unordered_map<int64_t, Piece> got;
    explicit Pass2(const std::vector<int64_t>& w) : want(w) {}
    void way(const osmium::Way& w) {
        if (!std::binary_search(want.begin(), want.end(), w.id())) return;
        Piece p;
        for (const osmium::NodeRef& n : w.nodes()) {
            if (!n.location().valid()) return;   // Python: a way without locations is not taken
            p.push_back({n.ref(), n.location().x(), n.location().y()});
        }
        got[w.id()] = std::move(p);
    }
};
// harvest_buildings.stitch, the same order of every choice.
std::vector<Piece> Stitch(const std::vector<const Piece*>& ways) {
    std::vector<Piece> rings, open;
    for (const Piece* w : ways) {
        if (w->front() == w->back() && w->size() >= 4) rings.push_back(*w);
        else open.push_back(*w);
    }
    while (!open.empty()) {
        Piece cur = std::move(open.back());
        open.pop_back();
        bool grown = true;
        while (!(cur.front() == cur.back()) && grown) {
            grown = false;
            for (size_t i = 0; i < open.size(); ++i) {
                const Piece& w = open[i];
                if (w.front() == cur.back()) {
                    cur.insert(cur.end(), w.begin() + 1, w.end());
                } else if (w.back() == cur.back()) {
                    cur.insert(cur.end(), w.rbegin() + 1, w.rend());
                } else if (w.back() == cur.front()) {
                    Piece n(w.begin(), w.end() - 1);
                    n.insert(n.end(), cur.begin(), cur.end());
                    cur = std::move(n);
                } else if (w.front() == cur.front()) {
                    Piece n(w.rbegin(), w.rend() - 1);
                    n.insert(n.end(), cur.begin(), cur.end());
                    cur = std::move(n);
                } else {
                    continue;
                }
                open.erase(open.begin() + static_cast<std::ptrdiff_t>(i));
                grown = true;
                break;
            }
        }
        if (cur.front() == cur.back() && cur.size() >= 4) rings.push_back(std::move(cur));
    }
    return rings;
}

// ---- pass 3: a bucket sorted into a chunk ----------------------------------------------------
struct CellRow {
    int32_t iy, ix;
    int64_t off, n;
};
uint64_t SortBucket(const std::vector<fs::path>& parts, const fs::path& chunk, std::vector<CellRow>& cells) {
    std::string data;
    for (const fs::path& p : parts) {
        std::ifstream f(p, std::ios::binary);
        data.append(std::istreambuf_iterator<char>(f), std::istreambuf_iterator<char>());
    }
    struct Head {
        int32_t iy, ix;
        int64_t id;
        uint64_t start;
        uint32_t n;
    };
    std::vector<Head> heads;
    for (size_t p = 0; p < data.size();) {
        Head h;
        std::memcpy(&h.iy, &data[p], 4);
        std::memcpy(&h.ix, &data[p + 4], 4);
        std::memcpy(&h.id, &data[p + 8], 8);
        std::memcpy(&h.n, &data[p + 16], 4);
        h.start = p + 20;
        heads.push_back(h);
        p += 20 + h.n;
    }
    std::sort(heads.begin(), heads.end(), [](const Head& a, const Head& b) {   // Python's tuple order
        if (a.iy != b.iy) return a.iy < b.iy;
        if (a.ix != b.ix) return a.ix < b.ix;
        if (a.id != b.id) return a.id < b.id;
        return a.start < b.start;
    });
    std::ofstream f(chunk, std::ios::binary);
    int64_t off = 0;
    for (const Head& h : heads) {
        if (cells.empty() || cells.back().iy != h.iy || cells.back().ix != h.ix) cells.push_back({h.iy, h.ix, off, 0});
        ++cells.back().n;
        f.write(&data[h.start], h.n);
        off += h.n;
    }
    for (const fs::path& p : parts) fs::remove(p);
    return heads.size();
}

}  // namespace

int main(int argc, char** argv) {
    if (argc < 3) {
        fprintf(stderr, "usage: planet_buildings <buildings-with-locations.osm.pbf> <out dir> [threads]\n");
        return 2;
    }
    const fs::path in = argv[1], outDir = argv[2];
    const unsigned threads = argc > 3 ? static_cast<unsigned>(std::atoi(argv[3])) : std::max(1u, std::thread::hardware_concurrency() - 2);
    const std::string stem = in.filename().string().substr(0, in.filename().string().find('.'));
    const fs::path tmp = outDir / (stem + ".tmp");
    fs::create_directories(tmp);
    for (const auto& e : fs::directory_iterator(tmp)) fs::remove(e.path());
    setvbuf(stdout, nullptr, _IONBF, 0);

    osmium::io::File file{in.string()};
    {
        osmium::io::Reader probe{file, osmium::osm_entity_bits::nothing};
        const osmium::io::Header h = probe.header();
        if (h.get("pbf_optional_feature_0") != "LocationsOnWays" && h.get("pbf_optional_feature_1") != "LocationsOnWays" &&
            h.get("pbf_optional_feature_2") != "LocationsOnWays") {
            fprintf(stderr, "[planet++] no LocationsOnWays in the header: run osmium add-locations-to-ways first\n");
            return 1;
        }
        probe.close();
    }

    Pass1 p1{*new Buckets(tmp, "w")};
    {
        osmium::io::Reader r{file, osmium::osm_entity_bits::way | osmium::osm_entity_bits::relation};
        osmium::apply(r, p1);
        r.close();
    }
    delete &p1.out;   // flushes the ways' buckets
    printf("[planet++] pass 1 done: %llu ways, %zu multipolygons (%.0f s)\n", (unsigned long long)p1.ways, p1.rels.size(), Secs());

    std::vector<int64_t> want;
    for (const Rel& r : p1.rels)
        for (const auto& m : r.members) want.push_back(m.first);
    std::sort(want.begin(), want.end());
    want.erase(std::unique(want.begin(), want.end()), want.end());
    Pass2 p2{want};
    {
        osmium::io::Reader r{file, osmium::osm_entity_bits::way};
        osmium::apply(r, p2);
        r.close();
    }
    uint64_t noRing = 0;
    {
        Buckets own(tmp, "rel");
        for (const Rel& r : p1.rels) {
            std::vector<std::pair<int, std::vector<XY>>> geo;
            for (bool outer : {true, false}) {
                std::vector<const Piece*> ws;
                for (const auto& [m, o] : r.members) {
                    auto it = p2.got.find(m);
                    if (o == outer && it != p2.got.end()) ws.push_back(&it->second);
                }
                for (const Piece& ring : Stitch(ws)) {
                    std::vector<XY> xy;
                    for (size_t i = 0; i + 1 < ring.size(); ++i) xy.push_back({ring[i].x, ring[i].y});
                    geo.push_back({outer ? 1 : 0, std::move(xy)});
                }
            }
            std::stable_sort(geo.begin(), geo.end(), [](const auto& a, const auto& b) { return a.first > b.first; });
            if (geo.empty() || !geo[0].first) { ++noRing; continue; }
            int32_t iy, ix;
            CellOf(geo[0].second[0].x, geo[0].second[0].y, iy, ix);
            own.Add(iy, ix, -r.id, Record(-r.id, r.kind, r.attrs, geo));
        }
    }
    printf("[planet++] pass 2 done: relations stitched (%.0f s)\n", Secs());

    std::map<std::string, std::vector<fs::path>> parts;   // ordered by name, as Python's sorted()
    for (const auto& e : fs::directory_iterator(tmp)) {
        const std::string n = e.path().filename().string();
        if (n.size() > 5 && n.substr(n.size() - 5) == ".part") parts[n.substr(0, n.find('.'))].push_back(e.path());
    }
    for (auto& [b, ps] : parts) std::sort(ps.begin(), ps.end());
    std::vector<std::string> order;
    for (const auto& [b, ps] : parts) order.push_back(b);
    std::vector<std::vector<CellRow>> cells(order.size());
    std::vector<uint64_t> counts(order.size());
    std::atomic<size_t> next{0};
    std::vector<std::thread> pool;
    for (unsigned t = 0; t < std::min(threads, 6u); ++t) {
        pool.emplace_back([&] {
            for (size_t i; (i = next++) < order.size();) {
                counts[i] = SortBucket(parts[order[i]], tmp / (order[i] + ".chunk"), cells[i]);
            }
        });
    }
    for (std::thread& t : pool) t.join();
    const fs::path bin = outDir / (stem + ".buildings.bin");
    std::vector<CellRow> all;
    uint64_t total = 0;
    {
        std::ofstream f(bin, std::ios::binary);
        f.write("GABLDG01", 8);
        std::vector<char> buf(64u << 20);
        for (size_t i = 0; i < order.size(); ++i) {
            const int64_t base = f.tellp();
            const fs::path ch = tmp / (order[i] + ".chunk");
            {
                std::ifstream g(ch, std::ios::binary);
                while (g.read(buf.data(), static_cast<std::streamsize>(buf.size())) || g.gcount() > 0) f.write(buf.data(), g.gcount());
            }
            fs::remove(ch);
            for (CellRow c : cells[i]) {
                c.off += base;
                all.push_back(c);
            }
            total += counts[i];
        }
    }
    std::sort(all.begin(), all.end(), [](const CellRow& a, const CellRow& b) {
        return std::tie(a.iy, a.ix, a.off) < std::tie(b.iy, b.ix, b.off);
    });
    {
        std::ofstream f(outDir / (stem + ".buildings.idx"), std::ios::binary);
        for (const CellRow& c : all) {
            f.write(reinterpret_cast<const char*>(&c.ix), 4);
            f.write(reinterpret_cast<const char*>(&c.iy), 4);
            f.write(reinterpret_cast<const char*>(&c.off), 8);
            f.write(reinterpret_cast<const char*>(&c.n), 8);
        }
    }
    {
        std::ofstream f(outDir / (stem + ".buildings.json"));
        f << "{\n \"format\": \"GABLDG01\",\n \"source\": {\"file\": \"" << in.filename().string() << "\", \"bytes\": "
          << fs::file_size(in) << ", \"tool\": \"osmium tags-filter + add-locations-to-ways; harvester/cpp/planet_buildings\"},\n"
          << " \"licence\": \"ODbL 1.0\", \"attribution\": \"(c) OpenStreetMap contributors\",\n"
          << " \"units\": {\"coordinates\": \"int32, 1e-7 degree, WGS84 lon/lat (OSM's own precision)\", "
             "\"height\": \"metres above the ground at the footprint\", \"levels\": \"count\"},\n"
          << " \"datum\": \"ground\",\n"
          << " \"counts\": {\"records\": " << total << ", \"ways\": " << p1.ways << ", \"multipolygons\": " << p1.rels.size() << "},\n"
          << " \"refused\": {\"height\": " << p1.refused.height << ", \"min_height\": " << p1.refused.min_height
          << ", \"roof:height\": " << p1.refused.roof_height << ", \"building:levels\": " << p1.refused.levels
          << ", \"building:min_level\": " << p1.refused.min_level << "},\n"
          << " \"skipped\": {\"open_way\": " << p1.openWays << ", \"no_locations\": " << p1.noLocations
          << ", \"relation_no_ring\": " << noRing << "},\n \"roofShapes\": [";
        for (size_t i = 0; i < sizeof(kRoofs) / sizeof(kRoofs[0]); ++i) f << "\"" << kRoofs[i] << "\", ";
        f << "\"(other: 254)\"],\n \"cellDeg\": 0.05,\n \"cellIndex\": \"" << stem << ".buildings.idx\",\n \"cellCount\": "
          << all.size() << "\n}\n";
    }
    std::error_code ec;
    fs::remove(tmp, ec);
    printf("[planet++] %llu solids in %zu cells -> %s (%.1f GB) in %.1f min\n", (unsigned long long)total, all.size(),
           bin.string().c_str(), fs::file_size(bin) / 1073741824.0, Secs() / 60.0);
    printf("[planet++] refused height %llu, min_height %llu, roof:height %llu, levels %llu, min_level %llu; open %llu, "
           "no locations %llu, relation no ring %llu\n",
           (unsigned long long)p1.refused.height, (unsigned long long)p1.refused.min_height,
           (unsigned long long)p1.refused.roof_height, (unsigned long long)p1.refused.levels,
           (unsigned long long)p1.refused.min_level, (unsigned long long)p1.openWays,
           (unsigned long long)p1.noLocations, (unsigned long long)noRing);
    return 0;
}
