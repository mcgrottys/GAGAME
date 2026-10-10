// ================================================================================================
//  planet_roads.cpp - OSM highway=* ways into GAROAD01 (src/compose/RoadWays.h defines the bytes).
//
//  The input is osmium's output: `osmium tags-filter <planet> w/highway` then
//  `osmium add-locations-to-ways`, so every way carries its own coordinates and one pass over the
//  ways suffices (no relations). The design is planet_buildings.cpp's: libosmium's threaded PBF
//  reader, records bucketed by 2.5 degree tile into temp files, each bucket sorted by (iy, ix, id),
//  the buckets concatenated in the order of their names, the binary cell index beside them
//  (int32 ix, int32 iy, int64 offset, int64 count, by (iy, ix)), and a manifest.
//
//  NORMALIZE is this harvester's own stage: every table it applies is written into the manifest, so
//  the assumption is visible; a value that does not parse is REFUSED (counted per tag, left
//  untagged), never guessed. Defaults (a motorway's oneway, a residential street's lanes) are
//  compose's declared assumptions, not the harvest's.
//
//  Usage: planet_roads <roads-with-locations.osm.pbf> <out dir> [threads]
//         planet_roads --probe <stem.roads.json> <lon> <lat> <radiusKm>   (bridges / tunnels near)
//  Licence of the data: ODbL 1.0, "(c) OpenStreetMap contributors".
// ================================================================================================
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <osmium/handler.hpp>
#include <osmium/io/pbf_input.hpp>
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
#include <memory>
#include <regex>
#include <string>
#include <thread>
#include <tuple>
#include <unordered_map>
#include <vector>

namespace fs = std::filesystem;

namespace {

constexpr double kCellDeg = 0.05;
constexpr int kBucket = 50;   // cells a bucket spans on each axis: 2.5 degrees
constexpr float kNaN = std::numeric_limits<float>::quiet_NaN();
constexpr float kInf = std::numeric_limits<float>::infinity();
constexpr uint8_t kOther = 254, kUntagged = 255;
constexpr int8_t kLevelUntagged = -128;

// RoadWays.h's enums, by value (the harvester does not include src/).
enum Form : uint8_t { Carriageway = 0, DualCarriageway = 1, Ramp = 2, Roundabout = 3, Service = 4, ParkingAisle = 5,
                      PathForm = 6, Track = 7, Steps = 8, OtherForm = 9 };
enum Ramp_ : uint8_t { RampNone = 0, RampExit = 1, RampEntrance = 2, RampDirectionUntagged = 3 };
enum Structure : uint8_t { SNone = 0, Bridge = 1, Tunnel = 2, Ford = 3, Culvert = 4 };
enum Oneway : uint8_t { Both = 0, Forward = 1, Backward = 2, Closed = 3 };
enum Access : uint8_t { Public = 0, Private = 1, No = 2, Permissive = 3, Destination = 4, AccessOther = 5 };
const char* const kFormNames[] = {"Carriageway", "DualCarriageway", "Ramp", "Roundabout", "Service", "ParkingAisle",
                                  "Path", "Track", "Steps", "Other"};
const char* const kStructNames[] = {"None", "Bridge", "Tunnel", "Ford", "Culvert"};

// The FIXED highway vocabulary: index = position, 254 = other. Never reorder: the files store the index.
const char* const kHighways[] = {"motorway", "trunk", "primary", "secondary", "tertiary", "unclassified",
                                 "residential", "motorway_link", "trunk_link", "primary_link", "secondary_link",
                                 "tertiary_link", "living_street", "service", "pedestrian", "track", "busway",
                                 "bus_guideway", "escape", "raceway", "road", "footway", "bridleway", "steps",
                                 "corridor", "path", "cycleway", "construction", "proposed", "platform",
                                 "elevator", "via_ferrata"};
constexpr size_t kHighwayN = sizeof(kHighways) / sizeof(kHighways[0]);
// rank per vocabulary entry (255 untagged): FRC-like, 0 motorway .. 8 track.
const uint8_t kRank[kHighwayN] = {0, 1, 2, 3, 4, 5, 6,            // motorway .. residential
                                  0, 1, 2, 3, 4,                  // the links take their class's rank
                                  6, 7, 255, 8, 3,                // living_street, service, pedestrian, track, busway
                                  255, 255, 255, 5,               // bus_guideway, escape, raceway, road
                                  255, 255, 255, 255, 255, 255,   // footway, bridleway, steps, corridor, path, cycleway
                                  255, 255, 255, 255, 255};       // construction, proposed, platform, elevator, via_ferrata
// The FIXED surface vocabulary: 254 other, 255 untagged.
const char* const kSurfaces[] = {"asphalt", "paved", "unpaved", "concrete", "concrete:plates", "concrete:lanes",
                                 "paving_stones", "sett", "cobblestone", "unhewn_cobblestone", "gravel",
                                 "fine_gravel", "compacted", "dirt", "earth", "ground", "grass", "grass_paver", "mud",
                                 "sand", "wood", "metal", "woodchips", "pebblestone", "rock", "chipseal", "tartan",
                                 "artificial_turf", "clay", "ice", "snow", "salt"};
constexpr size_t kSurfaceN = sizeof(kSurfaces) / sizeof(kSurfaces[0]);

auto t0 = std::chrono::steady_clock::now();
double Secs() { return std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count(); }

// ---- normalize: planet_buildings.cpp's Metres / Count, verbatim -----------------------------
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

std::string Lower(const char* s) {
    std::string t = Strip(s);
    std::transform(t.begin(), t.end(), t.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return t;
}
bool Is(const char* v, const char* w) { return v && std::strcmp(v, w) == 0; }

// maxspeed: "NN" km/h, "NN mph", "NN knots"; "none" = no limit (+inf); anything else refused.
bool Speed(const char* text, float& out) {
    static const std::regex re(R"((\d+(?:\.\d+)?)\s*(mph|knots)?)");
    const std::string t = Lower(text);
    if (t == "none") { out = kInf; return true; }
    std::smatch g;
    if (!std::regex_match(t, g, re)) return false;
    double v = std::strtod(g[1].str().c_str(), nullptr);
    if (g[2] == "mph") v *= 1.609344;
    else if (g[2] == "knots") v *= 1.852;
    out = static_cast<float>(v);
    return true;
}
// maxweight in tonnes: "N", "N t", "N tonnes"; "N lbs"/"N lb"; "N st" (short tons); else refused.
bool Tonnes(const char* text, float& out) {
    static const std::regex re(R"((\d+(?:[.,]\d+)?)\s*(t|tonnes|lbs|lb|st)?)");
    const std::string t = Lower(text);
    std::smatch g;
    if (!std::regex_match(t, g, re)) return false;
    std::string n = g[1];
    std::replace(n.begin(), n.end(), ',', '.');
    double v = std::strtod(n.c_str(), nullptr);
    if (g[2] == "lbs" || g[2] == "lb") v *= 0.00045359237;
    else if (g[2] == "st") v *= 0.90718474;
    out = static_cast<float>(v);
    return true;
}
// layer: an integer -5..5.
bool Layer(const char* text, int8_t& out) {
    const std::string t = Strip(text);
    if (t.empty()) return false;
    char* end = nullptr;
    const long v = std::strtol(t.c_str(), &end, 10);
    if (end != t.c_str() + t.size() || v < -5 || v > 5) return false;
    out = static_cast<int8_t>(v);
    return true;
}

struct Counts {
    uint64_t lanes = 0, lanesForward = 0, lanesBackward = 0, width = 0, maxspeed = 0, maxheight = 0, maxweight = 0,
             layer = 0, oneway = 0;   // refused
    uint64_t maxheightLegalDefault = 0, bridgeTunnelConflict = 0;
    uint64_t form[10] = {}, structure[5] = {};
};

struct Attrs {
    uint8_t highway = kOther, rank = kUntagged, form = OtherForm, ramp = RampNone, structure = SNone;
    int8_t level = kLevelUntagged;
    uint8_t oneway = kUntagged, surface = kUntagged, access = kUntagged;
    float lanes = kNaN, lanesF = kNaN, lanesB = kNaN, width = kNaN, speed = kNaN, height = kNaN, weight = kNaN;
};

bool RoadClass(const char* hw) {   // the classes whose form is Carriageway unless stated otherwise
    for (const char* c : {"trunk", "primary", "secondary", "tertiary", "unclassified", "residential", "living_street",
                          "road", "busway"})
        if (std::strcmp(hw, c) == 0) return true;
    return false;
}

Attrs AttrsOf(const char* hw, const osmium::TagList& t, Counts& c) {
    Attrs a;
    for (uint8_t i = 0; i < kHighwayN; ++i)
        if (std::strcmp(hw, kHighways[i]) == 0) { a.highway = i; break; }
    a.rank = a.highway < kHighwayN ? kRank[a.highway] : kUntagged;

    // form: link > roundabout > class; dual_carriageway=yes upgrades a road class. Never from oneway.
    const bool link = a.highway >= 7 && a.highway <= 11;   // the vocabulary's five *_link (residential_link etc. are "other")
    const char* junction = t["junction"];
    if (link) { a.form = Ramp; a.ramp = RampDirectionUntagged; }
    else if (Is(junction, "roundabout") || Is(junction, "circular")) a.form = Roundabout;
    else if (std::strcmp(hw, "motorway") == 0) a.form = DualCarriageway;
    else if (std::strcmp(hw, "service") == 0) a.form = Is(t["service"], "parking_aisle") ? ParkingAisle : Service;
    else if (Is(hw, "footway") || Is(hw, "path") || Is(hw, "cycleway") || Is(hw, "bridleway") || Is(hw, "pedestrian") ||
             Is(hw, "corridor")) a.form = PathForm;
    else if (Is(hw, "track")) a.form = Track;
    else if (Is(hw, "steps")) a.form = Steps;
    else if (RoadClass(hw)) a.form = Is(t["dual_carriageway"], "yes") ? DualCarriageway : Carriageway;
    else a.form = OtherForm;
    ++c.form[a.form];

    // structure: absence of the tags = None (OSM's meaning), not untagged.
    const char* br = t["bridge"];
    const char* tu = t["tunnel"];
    const bool isBridge = br && std::strcmp(br, "no") != 0, isTunnel = tu && std::strcmp(tu, "no") != 0;
    if (isBridge) { a.structure = Bridge; if (isTunnel) ++c.bridgeTunnelConflict; }
    else if (isTunnel) a.structure = std::strcmp(tu, "culvert") == 0 ? Culvert : Tunnel;
    else if (Is(t["ford"], "yes")) a.structure = Ford;
    ++c.structure[a.structure];

    if (const char* v = t["layer"]) { if (!Layer(v, a.level)) ++c.layer; }

    if (const char* v = t["oneway"]) {
        if (Is(v, "yes") || Is(v, "true") || Is(v, "1")) a.oneway = Forward;
        else if (Is(v, "-1") || Is(v, "reverse")) a.oneway = Backward;
        else if (Is(v, "no") || Is(v, "false") || Is(v, "0")) a.oneway = Both;
        else ++c.oneway;
    }

    auto count = [&](const char* k, uint64_t& refused) -> float {
        const char* v = t[k];
        if (!v) return kNaN;
        double x;
        if (!Count(v, x)) { ++refused; return kNaN; }
        return static_cast<float>(x);
    };
    a.lanes = count("lanes", c.lanes);
    a.lanesF = count("lanes:forward", c.lanesForward);
    a.lanesB = count("lanes:backward", c.lanesBackward);
    if (const char* v = t["width"]) {
        double m;
        if (Metres(v, m)) a.width = static_cast<float>(m); else ++c.width;
    }
    if (const char* v = t["maxspeed"]) { if (!Speed(v, a.speed)) ++c.maxspeed; }
    if (const char* v = t["maxheight"]) {
        const std::string l = Lower(v);
        double m;
        if (l == "none" || l == "unlimited") a.height = kInf;
        else if (l == "default" || l == "below_default") ++c.maxheightLegalDefault;
        else if (Metres(v, m)) a.height = static_cast<float>(m);
        else ++c.maxheight;
    }
    if (const char* v = t["maxweight"]) { if (!Tonnes(v, a.weight)) ++c.maxweight; }

    if (const char* v = t["surface"]) {
        a.surface = kOther;
        for (uint8_t i = 0; i < kSurfaceN; ++i)
            if (std::strcmp(v, kSurfaces[i]) == 0) { a.surface = i; break; }
    }
    if (const char* v = t["access"]) {
        if (Is(v, "yes") || Is(v, "public")) a.access = Public;
        else if (Is(v, "private")) a.access = Private;
        else if (Is(v, "no")) a.access = No;
        else if (Is(v, "permissive")) a.access = Permissive;
        else if (Is(v, "destination") || Is(v, "customers") || Is(v, "delivery")) a.access = Destination;
        else a.access = AccessOther;
    }
    return a;
}

// ---- records and buckets (planet_buildings.cpp's) ---------------------------------------------
int64_t FloorDiv(int64_t a, int64_t b) { return a / b - ((a % b != 0) && ((a < 0) != (b < 0))); }
void CellOf(int32_t x, int32_t y, int32_t& iy, int32_t& ix) {
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

#pragma pack(push, 1)
struct Pt {
    int64_t node;
    int32_t x, y;
};
#pragma pack(pop)
static_assert(sizeof(Pt) == 16, "GAROAD01 point layout");

void StrLen(const char* s, uint16_t& len) {
    const size_t n = s ? std::strlen(s) : 0;
    len = static_cast<uint16_t>(std::min<size_t>(n, 65535));
}
std::string Record(int64_t id, const Attrs& a, const char* name, const char* ref, const std::vector<Pt>& pts) {
    std::string r;
    Put(r, id);
    for (uint8_t b : {a.highway, a.rank, a.form, a.ramp, a.structure}) Put(r, b);
    Put(r, a.level);   // levelFrom
    Put(r, a.level);   // levelTo: OSM's one `layer` to both ends
    for (uint8_t b : {a.oneway, a.surface, a.access}) Put(r, b);
    r.append(2, '\0');
    for (float f : {a.lanes, a.lanesF, a.lanesB, a.width, a.speed, a.height, a.weight}) Put(r, f);
    uint16_t nl, rl;
    StrLen(name, nl);
    StrLen(ref, rl);
    Put(r, nl);
    Put(r, rl);
    Put(r, static_cast<uint16_t>(0));   // nativeIdLen: OSM's id is the int64
    if (nl) r.append(name, nl);
    if (rl) r.append(ref, rl);
    Put(r, static_cast<uint32_t>(pts.size()));
    r.append(reinterpret_cast<const char*>(pts.data()), pts.size() * sizeof(Pt));
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

// ---- the one pass: highway ways ---------------------------------------------------------------
struct Pass : osmium::handler::Handler {
    Buckets& out;
    Counts c;
    uint64_t ways = 0, records = 0, noHighway = 0, highwayNo = 0, noLocations = 0, tooShort = 0;
    std::map<std::string, uint64_t> perHighway, perHighwayRecords;   // every value, "other" ones too
    explicit Pass(Buckets& b) : out(b) {}
    void way(const osmium::Way& w) {
        ++ways;
        const char* hw = w.tags()["highway"];
        if (!hw) { ++noHighway; return; }
        ++perHighway[hw];
        if (std::strcmp(hw, "no") == 0) { ++highwayNo; return; }
        const auto& nr = w.nodes();
        if (nr.size() < 2) { ++tooShort; return; }
        std::vector<Pt> pts;
        pts.reserve(nr.size());
        for (const osmium::NodeRef& n : nr) {
            const osmium::Location& l = n.location();
            if (!l.valid()) { ++noLocations; return; }
            pts.push_back({n.ref(), l.x(), l.y()});
        }
        const Attrs a = AttrsOf(hw, w.tags(), c);
        int32_t iy, ix;
        CellOf(pts[0].x, pts[0].y, iy, ix);
        out.Add(iy, ix, w.id(), Record(w.id(), a, w.tags()["name"], w.tags()["ref"], pts));
        ++perHighwayRecords[hw];
        if (++records % 50000000 == 0) printf("[roads] %llu records (%.0f s)\n", (unsigned long long)records, Secs());
    }
};

// ---- a bucket sorted into a chunk (planet_buildings.cpp's) ------------------------------------
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
    std::sort(heads.begin(), heads.end(), [](const Head& a, const Head& b) {
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

std::string Json(const std::string& s) {   // a JSON string literal; bytes >= 0x80 pass as UTF-8
    std::string o = "\"";
    for (unsigned char ch : s) {
        if (ch == '"') o += "\\\"";
        else if (ch == '\\') o += "\\\\";
        else if (ch < 0x20) { char b[8]; snprintf(b, sizeof(b), "\\u%04x", ch); o += b; }
        else o += static_cast<char>(ch);
    }
    return o + "\"";
}

// ---- --probe: the bridges and tunnels near a point, read back from the files ------------------
int Probe(const fs::path& manifest, double lon, double lat, double radiusKm) {
    std::string stem = manifest.filename().string();
    stem = stem.substr(0, stem.find('.'));
    const fs::path dir = manifest.parent_path();
    std::ifstream bin(dir / (stem + ".roads.bin"), std::ios::binary), idx(dir / (stem + ".roads.idx"), std::ios::binary);
    char magic[8];
    if (!bin || !bin.read(magic, 8) || std::memcmp(magic, "GAROAD01", 8) != 0) {
        fprintf(stderr, "no GAROAD01 data for %s\n", manifest.string().c_str());
        return 1;
    }
    const double kmLat = 111.32, kmLon = 111.32 * std::cos(lat * 3.14159265358979 / 180.0);
    const double dLat = radiusKm / kmLat, dLon = radiusKm / kmLon;
    // A way is indexed by its FIRST point: widen the cell box by a margin so ways that start outside
    // the circle but pass through it are seen (0.1 degree, ~11 km: longer bridge ways are rare).
    const double margin = 0.1;
    const int32_t iy0 = static_cast<int32_t>(std::floor((lat - dLat - margin) / kCellDeg)),
                  iy1 = static_cast<int32_t>(std::floor((lat + dLat + margin) / kCellDeg)),
                  ix0 = static_cast<int32_t>(std::floor((lon - dLon - margin) / kCellDeg)),
                  ix1 = static_cast<int32_t>(std::floor((lon + dLon + margin) / kCellDeg));
    int32_t row[2];
    int64_t on[2];
    int found = 0;
    printf("%-11s %-14s %-32s %-10s %5s %5s %7s %7s %5s %6s\n", "id", "highway", "name", "ref", "layer", "lanes",
           "maxspd", "oneway", "struc", "points");
    while (idx.read(reinterpret_cast<char*>(row), 8) && idx.read(reinterpret_cast<char*>(on), 16)) {
        const int32_t ix = row[0], iy = row[1];
        if (iy < iy0 || iy > iy1 || ix < ix0 || ix > ix1) continue;
        bin.seekg(on[0]);
        for (int64_t k = 0; k < on[1]; ++k) {
            char h[54];
            bin.read(h, 54);
            int64_t id;
            std::memcpy(&id, h, 8);
            const uint8_t hw = h[8], st = h[12], ow = h[15];
            const int8_t lv = h[13];
            float f[7];
            std::memcpy(f, h + 20, 28);
            uint16_t L[3];
            std::memcpy(L, h + 48, 6);
            std::string name(L[0], '\0'), ref(L[1], '\0'), nat(L[2], '\0');
            bin.read(name.data(), L[0]);
            bin.read(ref.data(), L[1]);
            bin.read(nat.data(), L[2]);
            uint32_t n;
            bin.read(reinterpret_cast<char*>(&n), 4);
            std::vector<Pt> pts(n);
            bin.read(reinterpret_cast<char*>(pts.data()), n * sizeof(Pt));
            if (st != Bridge && st != Tunnel) continue;
            bool inside = false;
            for (const Pt& p : pts) {
                const double ex = (p.x * 1e-7 - lon) * kmLon, ny = (p.y * 1e-7 - lat) * kmLat;
                if (ex * ex + ny * ny <= radiusKm * radiusKm) { inside = true; break; }
            }
            if (!inside) continue;
            ++found;
            char lanes[16], speed[16];
            snprintf(lanes, sizeof(lanes), std::isnan(f[0]) ? "-" : "%g", f[0]);
            snprintf(speed, sizeof(speed), std::isnan(f[4]) ? "-" : "%.0f", f[4]);
            const char* ows = ow == Forward ? "fwd" : ow == Backward ? "back" : ow == Both ? "both" : "-";
            printf("%-11lld %-14s %-32s %-10s %5s %5s %7s %7s %5s %6u\n", (long long)id,
                   hw < kHighwayN ? kHighways[hw] : "(other)", name.c_str(), ref.c_str(),
                   lv == kLevelUntagged ? "-" : std::to_string(lv).c_str(), lanes, speed, ows, kStructNames[st], n);
        }
    }
    printf("%d bridge/tunnel ways within %.2f km of %.5f, %.5f\n", found, radiusKm, lat, lon);
    return 0;
}

}  // namespace

int main(int argc, char** argv) {
    if (argc >= 6 && std::strcmp(argv[1], "--probe") == 0)
        return Probe(argv[2], std::atof(argv[3]), std::atof(argv[4]), std::atof(argv[5]));
    if (argc < 3) {
        fprintf(stderr, "usage: planet_roads <roads-with-locations.osm.pbf> <out dir> [threads]\n"
                        "       planet_roads --probe <stem.roads.json> <lon> <lat> <radiusKm>\n");
        return 2;
    }
    const fs::path in = argv[1], outDir = argv[2];
    const unsigned threads = argc > 3 ? static_cast<unsigned>(std::atoi(argv[3])) : std::max(1u, std::thread::hardware_concurrency() - 2);
    const std::string stem = in.filename().string().substr(0, in.filename().string().find('.'));
    const fs::path tmp = outDir / (stem + ".roads.tmp");
    fs::create_directories(tmp);
    for (const auto& e : fs::directory_iterator(tmp)) fs::remove(e.path());
    setvbuf(stdout, nullptr, _IONBF, 0);

    osmium::io::File file{in.string()};
    {
        osmium::io::Reader probe{file, osmium::osm_entity_bits::nothing};
        const osmium::io::Header h = probe.header();
        if (h.get("pbf_optional_feature_0") != "LocationsOnWays" && h.get("pbf_optional_feature_1") != "LocationsOnWays" &&
            h.get("pbf_optional_feature_2") != "LocationsOnWays") {
            fprintf(stderr, "[roads] no LocationsOnWays in the header: run osmium add-locations-to-ways first\n");
            return 1;
        }
        probe.close();
    }

    auto buckets = std::make_unique<Buckets>(tmp, "w");
    Pass p{*buckets};
    {
        osmium::io::Reader r{file, osmium::osm_entity_bits::way};
        osmium::apply(r, p);
        r.close();
    }
    buckets.reset();   // flushes the buckets
    printf("[roads] pass done: %llu ways, %llu records (%.0f s)\n", (unsigned long long)p.ways,
           (unsigned long long)p.records, Secs());

    std::map<std::string, std::vector<fs::path>> parts;   // ordered by name
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
    const fs::path bin = outDir / (stem + ".roads.bin");
    std::vector<CellRow> all;
    uint64_t total = 0;
    {
        std::ofstream f(bin, std::ios::binary);
        f.write("GAROAD01", 8);
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
        std::ofstream f(outDir / (stem + ".roads.idx"), std::ios::binary);
        for (const CellRow& c : all) {
            f.write(reinterpret_cast<const char*>(&c.ix), 4);
            f.write(reinterpret_cast<const char*>(&c.iy), 4);
            f.write(reinterpret_cast<const char*>(&c.off), 8);
            f.write(reinterpret_cast<const char*>(&c.n), 8);
        }
    }
    {
        const Counts& c = p.c;
        std::ofstream f(outDir / (stem + ".roads.json"));
        f << "{\n \"format\": \"GAROAD01\",\n \"source\": {\"file\": " << Json(in.filename().string()) << ", \"bytes\": "
          << fs::file_size(in) << ", \"tool\": \"osmium tags-filter w/highway + add-locations-to-ways; harvester/cpp/planet_roads\"},\n"
          << " \"licence\": \"ODbL 1.0\", \"attribution\": \"(c) OpenStreetMap contributors\",\n"
          << " \"units\": {\"coordinates\": \"int32, 1e-7 degree, WGS84 lon/lat (OSM's own precision)\", "
             "\"lengths\": \"float32 metres (width, maxheight)\", \"maxspeed\": \"float32 km/h\", "
             "\"maxweight\": \"float32 tonnes\", \"lanes\": \"float32 count\", "
             "\"untagged\": \"NaN; enums 255; level -128\", \"noRestriction\": \"+inf\"},\n"
          << " \"highway\": [";
        for (size_t i = 0; i < kHighwayN; ++i) f << "\"" << kHighways[i] << "\", ";
        f << "\"(other: 254)\"],\n \"surface\": [";
        for (size_t i = 0; i < kSurfaceN; ++i) f << "\"" << kSurfaces[i] << "\", ";
        f << "\"(other: 254, untagged: 255)\"],\n \"rank\": {";
        for (size_t i = 0; i < kHighwayN; ++i) f << (i ? ", " : "") << "\"" << kHighways[i] << "\": " << int(kRank[i]);
        f << ", \"(other)\": 255},\n"
          << " \"form\": {\"values\": [";
        for (size_t i = 0; i < 10; ++i) f << (i ? ", " : "") << "\"" << kFormNames[i] << "\"";
        f << "], \"untagged\": 255, \"rules\": [\"*_link -> Ramp (ramp DirectionUntagged; others ramp None)\", "
             "\"junction=roundabout|circular -> Roundabout\", \"motorway -> DualCarriageway\", "
             "\"service -> Service; service=parking_aisle -> ParkingAisle\", "
             "\"footway|path|cycleway|bridleway|pedestrian|corridor -> Path\", \"track -> Track\", \"steps -> Steps\", "
             "\"trunk|primary|secondary|tertiary|unclassified|residential|living_street|road|busway -> Carriageway, "
             "DualCarriageway when dual_carriageway=yes\", \"the rest -> Other\", "
             "\"first rule that matches wins; never inferred from oneway\"]},\n"
          << " \"ramp\": [\"None\", \"Exit\", \"Entrance\", \"DirectionUntagged\"],\n"
          << " \"structure\": {\"values\": [\"None\", \"Bridge\", \"Tunnel\", \"Ford\", \"Culvert\"], \"untagged\": 255, "
             "\"rules\": [\"bridge present and not no -> Bridge (wins over tunnel: conflict counted)\", "
             "\"tunnel=culvert -> Culvert\", \"tunnel present and not no -> Tunnel\", \"ford=yes -> Ford\", "
             "\"none of them -> None (OSM: no tag, no structure)\"]},\n"
          << " \"level\": \"layer, an integer -5..5, to levelFrom and levelTo; absent or refused -128\",\n"
          << " \"oneway\": {\"values\": [\"Both\", \"Forward\", \"Backward\", \"Closed\"], \"untagged\": 255, "
             "\"rules\": [\"yes|true|1 -> Forward\", \"-1|reverse -> Backward\", \"no|false|0 -> Both\", "
             "\"absent -> untagged\", \"other -> refused, untagged\"]},\n"
          << " \"access\": {\"values\": [\"Public\", \"Private\", \"No\", \"Permissive\", \"Destination\", \"Other\"], "
             "\"untagged\": 255, \"rules\": [\"yes|public -> Public\", \"private -> Private\", \"no -> No\", "
             "\"permissive -> Permissive\", \"destination|customers|delivery -> Destination\", \"other -> Other\"]},\n"
          << " \"parse\": {\"lanes\": \"count: a number >= 0 (',' read as '.'), else refused\", "
             "\"width\": \"metres: N [m], N ft|feet|', N'M\\\"\", "
             "\"maxspeed\": \"N -> km/h; N mph x 1.609344; N knots x 1.852; none -> +inf; else refused\", "
             "\"maxheight\": \"as width; none|unlimited -> +inf; default|below_default -> untagged (legal default)\", "
             "\"maxweight\": \"N|N t|N tonnes -> t; N lbs|N lb x 0.00045359237; N st x 0.90718474; else refused\"},\n"
          << " \"counts\": {\"ways\": " << p.ways << ", \"records\": " << total << ",\n  \"highway\": {";
        bool first = true;
        for (const auto& [k, v] : p.perHighway) { f << (first ? "" : ", ") << Json(k) << ": " << v; first = false; }
        f << "},\n  \"highwayRecords\": {";
        first = true;
        for (const auto& [k, v] : p.perHighwayRecords) { f << (first ? "" : ", ") << Json(k) << ": " << v; first = false; }
        f << "},\n  \"structure\": {";
        for (size_t i = 0; i < 5; ++i) f << (i ? ", " : "") << "\"" << kStructNames[i] << "\": " << c.structure[i];
        f << "}, \"bridgeTunnelConflict\": " << c.bridgeTunnelConflict << ",\n  \"form\": {";
        for (size_t i = 0; i < 10; ++i) f << (i ? ", " : "") << "\"" << kFormNames[i] << "\": " << c.form[i];
        f << "},\n  \"maxheightLegalDefault\": " << c.maxheightLegalDefault << "},\n"
          << " \"refused\": {\"lanes\": " << c.lanes << ", \"lanes:forward\": " << c.lanesForward
          << ", \"lanes:backward\": " << c.lanesBackward << ", \"width\": " << c.width << ", \"maxspeed\": " << c.maxspeed
          << ", \"maxheight\": " << c.maxheight << ", \"maxweight\": " << c.maxweight << ", \"layer\": " << c.layer
          << ", \"oneway\": " << c.oneway << "},\n"
          << " \"skipped\": {\"no_highway\": " << p.noHighway << ", \"highway_no\": " << p.highwayNo
          << ", \"no_locations\": " << p.noLocations << ", \"too_short\": " << p.tooShort << "},\n"
          << " \"cellDeg\": 0.05,\n \"cellIndex\": " << Json(stem + ".roads.idx") << ",\n \"cellCount\": " << all.size()
          << "\n}\n";
    }
    std::error_code ec;
    fs::remove(tmp, ec);
    printf("[roads] %llu records in %zu cells -> %s (%.3f GB) in %.1f min\n", (unsigned long long)total, all.size(),
           bin.string().c_str(), fs::file_size(bin) / 1073741824.0, Secs() / 60.0);
    printf("[roads] skipped: no highway %llu, highway=no %llu, no locations %llu, too short %llu\n",
           (unsigned long long)p.noHighway, (unsigned long long)p.highwayNo, (unsigned long long)p.noLocations,
           (unsigned long long)p.tooShort);
    return 0;
}
