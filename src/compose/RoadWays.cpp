// RoadWays - the readers of GAROAD01 and GABRDG01 (the layouts are RoadWays.h's header comment),
// the junction table, and the bridge match. The harvest is read as the building stack reads its
// own (compose/BuildingSolids.cpp, BuildingStack::Open / Read): manifest, cell-index sidecar,
// the .bin beside it, a row of cells at a time by binary search, records kept by first point.
#include "compose/RoadWays.h"

#include "core/Common.h"
#include "core/Json.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <fstream>
#include <iterator>
#include <unordered_map>

namespace ga {

namespace {

std::string ReadAll(const std::string& path) {
    std::ifstream f(path, std::ios::binary);
    if (!f) return {};
    return std::string((std::istreambuf_iterator<char>(f)), std::istreambuf_iterator<char>());
}

bool EndsWith(const std::string& s, const char* tail) {
    const size_t n = std::strlen(tail);
    return s.size() >= n && s.compare(s.size() - n, n, tail) == 0;
}

// A record's point count or part count past this is a corrupt file, not a road: refuse the cell
// rather than allocate what the bytes say.
constexpr uint32_t kMaxCount = 1u << 22;

// One harvest opened: the manifest checked, the cell rows read, the .bin's magic seen.
struct Harvest {
    JsonValue manifest;
    double cellDeg = 0.0;
    std::string bin;
    std::vector<std::array<int64_t, 4>> cells;   // (iy, ix, offset, count), sorted
};

bool OpenHarvest(const std::string& path, const char* magicWant, const char* tail, Harvest& h, std::string* why) {
    auto refuse = [why](const std::string& w) {
        if (why) *why = w;
        return false;
    };
    if (!EndsWith(path, tail)) return refuse(std::string("not a ") + tail + " manifest: " + path);
    const std::string text = ReadAll(path);
    if (text.empty()) return refuse("cannot read " + path);
    std::string err;
    h.manifest = JsonParser::Parse(text, &err);
    if (!err.empty() || h.manifest.Str("format") != magicWant) {
        return refuse(std::string("not a ") + magicWant + " manifest" + (err.empty() ? "" : ": " + err));
    }
    h.cellDeg = h.manifest.Num("cellDeg", 0.0);
    const std::string idxName = h.manifest.Str("cellIndex");
    if (idxName.empty() || !(h.cellDeg > 0.0)) return refuse("the manifest has no cell index");
    h.bin = path.substr(0, path.size() - 5) + ".bin";
    {
        std::ifstream f(h.bin, std::ios::binary);
        char magic[8] = {};
        if (!f || !f.read(magic, 8) || std::memcmp(magic, magicWant, 8) != 0) {
            return refuse(std::string("no ") + magicWant + " data beside the manifest (" + h.bin + ")");
        }
    }
    // The sidecar beside the manifest: 24 bytes a cell.
    const std::string idx = path.substr(0, path.find_last_of("/\\") + 1) + idxName;
    std::ifstream fi(idx, std::ios::binary);
#pragma pack(push, 1)
    struct {
        int32_t ix, iy;
        int64_t off, n;
    } row;
#pragma pack(pop)
    static_assert(sizeof(row) == 24, "cell index row");
    while (fi.read(reinterpret_cast<char*>(&row), sizeof(row))) h.cells.push_back({row.iy, row.ix, row.off, row.n});
    if (h.cells.empty()) return refuse("no cell index at " + idx);
    std::sort(h.cells.begin(), h.cells.end());
    return true;
}

// Every cell of the index the box touches, a row at a time (BuildingStack::Read's walk).
template <class Fn>
void ForCells(const Harvest& h, double lon0, double lat0, double lon1, double lat1, Fn&& fn) {
    const int64_t ix0 = static_cast<int64_t>(std::floor(lon0 / h.cellDeg));
    const int64_t ix1 = static_cast<int64_t>(std::floor(lon1 / h.cellDeg));
    const int64_t iy0 = static_cast<int64_t>(std::floor(lat0 / h.cellDeg));
    const int64_t iy1 = static_cast<int64_t>(std::floor(lat1 / h.cellDeg));
    for (int64_t iy = iy0; iy <= iy1; ++iy) {
        auto it = std::lower_bound(h.cells.begin(), h.cells.end(), std::array<int64_t, 4>{iy, ix0, INT64_MIN, INT64_MIN});
        for (; it != h.cells.end() && (*it)[0] == iy && (*it)[1] <= ix1; ++it) fn((*it)[2], (*it)[3]);
    }
}

bool InBox(double x, double y, double lon0, double lat0, double lon1, double lat1) {
    return x >= lon0 && x < lon1 && y >= lat0 && y < lat1;
}

bool ReadString(std::ifstream& f, uint16_t len, std::string& s) {
    s.resize(len);
    return len == 0 || static_cast<bool>(f.read(s.data(), len));
}

// The manifest's vocabulary for one key: index < 254 is the array's word, 254 "other", 255 "".
std::string Word(const std::vector<std::string>& vocab, uint8_t i) {
    if (i == 255) return {};
    if (i == 254 || i >= vocab.size()) return "other";
    return vocab[i];
}

std::vector<std::string> Vocabulary(const JsonValue& m, const char* key) {
    std::vector<std::string> v;
    if (const JsonValue* a = m.Get(key)) {
        for (const JsonValue& w : a->arr) v.push_back(w.type == JsonValue::Type::String ? w.str : std::string());
    }
    return v;
}

bool ReadRoads(std::ifstream& f, int64_t offset, int64_t n, double lon0, double lat0, double lon1, double lat1,
               const std::vector<std::string>& highways, const std::vector<std::string>& surfaces,
               std::vector<RoadWay>& out) {
    f.clear();
    f.seekg(static_cast<std::streamoff>(offset));
#pragma pack(push, 1)
    struct {
        int64_t id;
        uint8_t highway, rank, form, ramp, structure;
        int8_t levelFrom, levelTo;
        uint8_t oneway, surface, access, pad[2];
        float lanes, lanesForward, lanesBackward, widthM, maxspeedKph, maxheightM, maxweightT;
        uint16_t nameLen, refLen, nativeIdLen;
    } rec;
    struct {
        int64_t nodeId;
        int32_t x, y;
    } pt;
#pragma pack(pop)
    static_assert(sizeof(rec) == 54 && sizeof(pt) == 16, "GAROAD01 record layout");
    for (int64_t i = 0; i < n; ++i) {
        if (!f.read(reinterpret_cast<char*>(&rec), sizeof(rec))) return false;
        RoadWay w;
        w.id = rec.id;
        w.highway = rec.highway;
        w.highwayTag = Word(highways, rec.highway);
        w.rank = rec.rank;
        w.form = static_cast<RoadForm>(rec.form);
        w.ramp = static_cast<RoadRamp>(rec.ramp);
        w.structure = static_cast<RoadStructure>(rec.structure);
        w.levelFrom = rec.levelFrom;
        w.levelTo = rec.levelTo;
        w.oneway = static_cast<RoadOneway>(rec.oneway);
        w.surface = Word(surfaces, rec.surface);
        w.access = static_cast<RoadAccess>(rec.access);
        w.lanes = rec.lanes;
        w.lanesForward = rec.lanesForward;
        w.lanesBackward = rec.lanesBackward;
        w.widthM = rec.widthM;
        w.maxspeedKph = rec.maxspeedKph;
        w.maxheightM = rec.maxheightM;
        w.maxweightT = rec.maxweightT;
        if (!ReadString(f, rec.nameLen, w.name) || !ReadString(f, rec.refLen, w.ref) ||
            !ReadString(f, rec.nativeIdLen, w.nativeId)) {
            return false;
        }
        uint32_t np = 0;
        if (!f.read(reinterpret_cast<char*>(&np), 4) || np > kMaxCount) return false;
        w.points.resize(np);
        for (uint32_t k = 0; k < np; ++k) {
            if (!f.read(reinterpret_cast<char*>(&pt), sizeof(pt))) return false;
            w.points[k].nodeId = pt.nodeId;
            w.points[k].lon = pt.x * 1e-7;   // the file's own 1e-7 degree
            w.points[k].lat = pt.y * 1e-7;
        }
        if (w.points.empty()) continue;
        if (InBox(w.points[0].lon, w.points[0].lat, lon0, lat0, lon1, lat1)) out.push_back(std::move(w));
    }
    return true;
}

bool ReadSpans(std::ifstream& f, int64_t offset, int64_t n, double lon0, double lat0, double lon1, double lat1,
               std::vector<BridgeSpan>& out) {
    f.clear();
    f.seekg(static_cast<std::streamoff>(offset));
#pragma pack(push, 1)
    struct {
        int64_t id;
        uint8_t source, kind, category, primitive, verdat, clrUnderRef, serviceOn, serviceUnder, nbiKind, nbiType,
            navigation, pad;
        float vertClr, vertClrClosed, vertClrOpen, vertClrSafe, horClr, clrOverDeck, clrUnder, minVertClrRoute,
            lengthM, maxSpanM, deckWidthM, heightM;
        uint16_t spansMain, spansApproach, yearBuilt, nameLen, crossesLen, originLen, noteLen;
    } rec;
    struct {
        uint32_t n;
        uint8_t outer, pad[3];
    } part;
#pragma pack(pop)
    static_assert(sizeof(rec) == 82 && sizeof(part) == 8, "GABRDG01 record layout");
    std::vector<int32_t> xy;
    for (int64_t i = 0; i < n; ++i) {
        if (!f.read(reinterpret_cast<char*>(&rec), sizeof(rec))) return false;
        BridgeSpan s;
        s.id = rec.id;
        s.source = static_cast<BridgeSource>(rec.source);
        s.kind = static_cast<BridgeKind>(rec.kind);
        s.category = rec.category;
        s.primitive = rec.primitive;
        s.verdat = rec.verdat;
        s.clrUnderRef = rec.clrUnderRef;
        s.serviceOn = rec.serviceOn;
        s.serviceUnder = rec.serviceUnder;
        s.nbiKind = rec.nbiKind;
        s.nbiType = rec.nbiType;
        s.navigation = rec.navigation;
        s.vertClr = rec.vertClr;
        s.vertClrClosed = rec.vertClrClosed;
        s.vertClrOpen = rec.vertClrOpen;
        s.vertClrSafe = rec.vertClrSafe;
        s.horClr = rec.horClr;
        s.clrOverDeck = rec.clrOverDeck;
        s.clrUnder = rec.clrUnder;
        s.minVertClrRoute = rec.minVertClrRoute;
        s.lengthM = rec.lengthM;
        s.maxSpanM = rec.maxSpanM;
        s.deckWidthM = rec.deckWidthM;
        s.heightM = rec.heightM;
        s.spansMain = rec.spansMain;
        s.spansApproach = rec.spansApproach;
        s.yearBuilt = rec.yearBuilt;
        if (!ReadString(f, rec.nameLen, s.name) || !ReadString(f, rec.crossesLen, s.crosses) ||
            !ReadString(f, rec.originLen, s.origin) || !ReadString(f, rec.noteLen, s.note)) {
            return false;
        }
        uint32_t parts = 0;
        if (!f.read(reinterpret_cast<char*>(&parts), 4) || parts > kMaxCount) return false;
        for (uint32_t p = 0; p < parts; ++p) {
            if (!f.read(reinterpret_cast<char*>(&part), sizeof(part)) || part.n > kMaxCount) return false;
            xy.resize(2 * size_t(part.n));
            if (!xy.empty() && !f.read(reinterpret_cast<char*>(xy.data()), std::streamsize(xy.size() * 4))) return false;
            std::vector<double> pts(xy.size());
            for (size_t k = 0; k < xy.size(); ++k) pts[k] = xy[k] * 1e-7;
            s.parts.push_back(std::move(pts));
            s.outer.push_back(part.outer);
        }
        if (s.parts.empty() || s.parts[0].size() < 2) continue;
        if (InBox(s.parts[0][0], s.parts[0][1], lon0, lat0, lon1, lat1)) out.push_back(std::move(s));
    }
    return true;
}

}  // namespace

struct RoadFile::Impl {
    Harvest h;
    std::vector<std::string> highways, surfaces;
};

bool RoadFile::Open(const std::string& manifestPath, std::string* why) {
    auto impl = std::make_shared<Impl>();
    if (!OpenHarvest(manifestPath, "GAROAD01", ".roads.json", impl->h, why)) return false;
    impl->highways = Vocabulary(impl->h.manifest, "highway");
    impl->surfaces = Vocabulary(impl->h.manifest, "surface");
    m_impl = std::move(impl);
    m_path = manifestPath;
    return true;
}

bool RoadFile::Read(double lon0, double lat0, double lon1, double lat1, std::vector<RoadWay>& out) const {
    if (!m_impl) return false;
    const Impl& im = *m_impl;
    std::ifstream f(im.h.bin, std::ios::binary);   // this call's own handle: const, so threads may share the file
    bool ok = true;
    ForCells(im.h, lon0, lat0, lon1, lat1, [&](int64_t off, int64_t n) {
        if (!ReadRoads(f, off, n, lon0, lat0, lon1, lat1, im.highways, im.surfaces, out)) {
            Log("[roads] '%s': a truncated cell at offset %lld", m_path.c_str(), static_cast<long long>(off));
            ok = false;
        }
    });
    return ok;
}

std::vector<std::pair<int, int>> RoadFile::Cells() const {
    std::vector<std::pair<int, int>> out;
    if (!m_impl) return out;
    out.reserve(m_impl->h.cells.size());
    for (const std::array<int64_t, 4>& c : m_impl->h.cells) out.push_back({static_cast<int>(c[1]), static_cast<int>(c[0])});
    return out;
}
double RoadFile::CellDeg() const { return m_impl ? m_impl->h.cellDeg : 0.0; }

bool LoadRoadSource(const std::string& manifestPath, double lon0, double lat0, double lon1, double lat1,
                    std::vector<RoadWay>& out, std::string* why) {
    RoadFile rf;
    if (!rf.Open(manifestPath, why)) return false;
    rf.Read(lon0, lat0, lon1, lat1, out);
    return true;
}

bool LoadBridgeSource(const std::string& manifestPath, double lon0, double lat0, double lon1, double lat1,
                      std::vector<BridgeSpan>& out, std::string* why) {
    Harvest h;
    if (!OpenHarvest(manifestPath, "GABRDG01", ".bridges.json", h, why)) return false;
    std::ifstream f(h.bin, std::ios::binary);
    ForCells(h, lon0, lat0, lon1, lat1, [&](int64_t off, int64_t n) {
        if (!ReadSpans(f, off, n, lon0, lat0, lon1, lat1, out)) {
            Log("[roads] '%s': a truncated cell at offset %lld", manifestPath.c_str(), static_cast<long long>(off));
        }
    });
    return true;
}

std::vector<RoadNode> JunctionsOf(const std::vector<RoadWay>& ways) {
    struct Seen {
        double lon, lat;
        std::vector<int64_t> ways;
        bool end = false;
    };
    std::unordered_map<int64_t, Seen> at;
    for (const RoadWay& w : ways) {
        for (size_t k = 0; k < w.points.size(); ++k) {
            const RoadPoint& p = w.points[k];
            auto [it, fresh] = at.try_emplace(p.nodeId, Seen{p.lon, p.lat, {}, false});
            // A closed way passes its first node twice: one way, named once.
            if (it->second.ways.empty() || it->second.ways.back() != w.id) it->second.ways.push_back(w.id);
            if (k == 0 || k + 1 == w.points.size()) it->second.end = true;
        }
    }
    std::vector<RoadNode> out;
    for (auto& [id, s] : at) {
        std::sort(s.ways.begin(), s.ways.end());
        s.ways.erase(std::unique(s.ways.begin(), s.ways.end()), s.ways.end());
        if (s.ways.size() < 2 && !s.end) continue;
        RoadNode n;
        n.id = id;
        n.lon = s.lon;
        n.lat = s.lat;
        n.ways = std::move(s.ways);
        out.push_back(std::move(n));
    }
    std::sort(out.begin(), out.end(), [](const RoadNode& a, const RoadNode& b) { return a.id < b.id; });
    return out;
}

namespace {

// ---- the overlap law's geometry, on the span's local chart (metres). A way's segment is
// P(t) = A + t E, t in [0, 1]; each piece of the span's buffer (a disc about a vertex, a slab about
// an edge, the area's inside) cuts the segment in intervals of t, and the overlap is the length of
// their union. Exact: no sampling.
struct V2 {
    double x, y;
};
using Interval = std::pair<double, double>;

// The disc of radius r about C.
void DiscCut(V2 a, V2 e, V2 c, double r, std::vector<Interval>& out) {
    const double dx = a.x - c.x, dy = a.y - c.y;
    const double qa = e.x * e.x + e.y * e.y, qb = dx * e.x + dy * e.y, qc = dx * dx + dy * dy - r * r;
    if (qa <= 0.0) {
        if (qc <= 0.0) out.push_back({0.0, 1.0});
        return;
    }
    const double disc = qb * qb - qa * qc;
    if (disc < 0.0) return;
    const double s = std::sqrt(disc);
    out.push_back({(-qb - s) / qa, (-qb + s) / qa});
}

// The rectangle of half-width r about the edge C0 -> C1 (a capsule's body; its caps are discs).
void SlabCut(V2 a, V2 e, V2 c0, V2 c1, double r, std::vector<Interval>& out) {
    const double lx = c1.x - c0.x, ly = c1.y - c0.y, len = std::hypot(lx, ly);
    if (len <= 0.0) return;
    const double ux = lx / len, uy = ly / len;
    double lo = 0.0, hi = 1.0;
    // f(t) = f0 + t f1 kept within [fa, fb].
    auto clip = [&](double f0, double f1, double fa, double fb) {
        if (f1 == 0.0) {
            if (f0 < fa || f0 > fb) hi = -1.0;
            return;
        }
        double t0 = (fa - f0) / f1, t1 = (fb - f0) / f1;
        if (t0 > t1) std::swap(t0, t1);
        lo = (std::max)(lo, t0);
        hi = (std::min)(hi, t1);
    };
    const double px = a.x - c0.x, py = a.y - c0.y;
    clip(px * ux + py * uy, e.x * ux + e.y * uy, 0.0, len);
    clip(-px * uy + py * ux, -e.x * uy + e.y * ux, -r, r);
    if (lo <= hi) out.push_back({lo, hi});
}

// Even-odd over every ring: inside the area.
bool AreaHolds(const std::vector<std::vector<V2>>& rings, V2 p) {
    bool in = false;
    for (const std::vector<V2>& r : rings) {
        for (size_t i = 0, j = r.size() - 1; i < r.size(); j = i++) {
            if ((r[i].y > p.y) != (r[j].y > p.y) &&
                p.x < (r[j].x - r[i].x) * (p.y - r[i].y) / (r[j].y - r[i].y) + r[i].x) {
                in = !in;
            }
        }
    }
    return in;
}

// The stretches of the segment inside the area: cut at every edge crossing, each piece tested at
// its middle.
void AreaCut(V2 a, V2 e, const std::vector<std::vector<V2>>& rings, std::vector<Interval>& out) {
    std::vector<double> ts = {0.0, 1.0};
    for (const std::vector<V2>& r : rings) {
        for (size_t i = 0, j = r.size() - 1; i < r.size(); j = i++) {
            const double fx = r[i].x - r[j].x, fy = r[i].y - r[j].y;
            const double den = e.x * fy - e.y * fx;
            if (den == 0.0) continue;
            const double gx = r[j].x - a.x, gy = r[j].y - a.y;
            const double t = (gx * fy - gy * fx) / den, u = (gx * e.y - gy * e.x) / den;
            if (t > 0.0 && t < 1.0 && u >= 0.0 && u <= 1.0) ts.push_back(t);
        }
    }
    std::sort(ts.begin(), ts.end());
    for (size_t k = 0; k + 1 < ts.size(); ++k) {
        const double m = 0.5 * (ts[k] + ts[k + 1]);
        if (ts[k + 1] > ts[k] && AreaHolds(rings, {a.x + m * e.x, a.y + m * e.y})) out.push_back({ts[k], ts[k + 1]});
    }
}

// One span on its chart: every part as a polyline (an area's rings closed, and its inside).
struct SpanShape {
    std::vector<std::vector<V2>> parts;
    bool area = false;
    double r = 0.0;
    // The metres of the polyline `w` within r of the shape (or inside it, for an area).
    double Overlap(const std::vector<V2>& w) const {
        double total = 0.0;
        std::vector<Interval> iv;
        for (size_t j = 0; j + 1 < w.size(); ++j) {
            const V2 a = w[j], e{w[j + 1].x - a.x, w[j + 1].y - a.y};
            const double len = std::hypot(e.x, e.y);
            if (len <= 0.0) continue;
            iv.clear();
            for (const std::vector<V2>& p : parts) {
                const size_t n = p.size();
                for (size_t k = 0; k < n; ++k) DiscCut(a, e, p[k], r, iv);
                for (size_t k = 0; k + 1 < n; ++k) SlabCut(a, e, p[k], p[k + 1], r, iv);
                if (area && n > 2) SlabCut(a, e, p[n - 1], p[0], r, iv);
            }
            if (area) AreaCut(a, e, parts, iv);
            for (Interval& i : iv) {
                i.first = (std::max)(i.first, 0.0);
                i.second = (std::min)(i.second, 1.0);
            }
            std::sort(iv.begin(), iv.end());
            double lo = 0.0, hi = -1.0, covered = 0.0;
            for (const Interval& i : iv) {
                if (i.second <= i.first) continue;
                if (i.first > hi) {
                    if (hi > lo) covered += hi - lo;
                    lo = i.first;
                    hi = i.second;
                } else {
                    hi = (std::max)(hi, i.second);
                }
            }
            if (hi > lo) covered += hi - lo;
            total += covered * len;
        }
        return total;
    }
    // The nearest distance from the polyline `w` to the shape's first vertex (a point span).
    double DistanceToPoint(const std::vector<V2>& w) const {
        const V2 c = parts[0][0];
        double best = std::hypot(w[0].x - c.x, w[0].y - c.y);
        for (size_t j = 0; j + 1 < w.size(); ++j) {
            const double ex = w[j + 1].x - w[j].x, ey = w[j + 1].y - w[j].y, ee = ex * ex + ey * ey;
            const double t = ee > 0.0 ? std::clamp(((c.x - w[j].x) * ex + (c.y - w[j].y) * ey) / ee, 0.0, 1.0) : 0.0;
            best = (std::min)(best, std::hypot(c.x - (w[j].x + t * ex), c.y - (w[j].y + t * ey)));
        }
        return best;
    }
};

}  // namespace

std::vector<BridgeMatch> MatchBridges(const std::vector<RoadWay>& ways, const std::vector<BridgeSpan>& spans,
                                      double withinPointM, double withinShapeM) {
    constexpr double kLonM = 111320.0, kLatM = 110574.0;   // metres a degree of longitude (equator) / latitude
    struct Box {
        size_t way;
        double lon0, lat0, lon1, lat1;
    };
    std::vector<Box> boxes;
    for (size_t i = 0; i < ways.size(); ++i) {
        const RoadWay& w = ways[i];
        if (w.structure != RoadStructure::Bridge || w.points.empty()) continue;
        Box b{i, 1e9, 1e9, -1e9, -1e9};
        for (const RoadPoint& p : w.points) {
            b.lon0 = (std::min)(b.lon0, p.lon);
            b.lon1 = (std::max)(b.lon1, p.lon);
            b.lat0 = (std::min)(b.lat0, p.lat);
            b.lat1 = (std::max)(b.lat1, p.lat);
        }
        boxes.push_back(b);
    }
    std::vector<BridgeMatch> out;
    std::vector<V2> wl;
    for (size_t s = 0; s < spans.size(); ++s) {
        const BridgeSpan& sp = spans[s];
        if (sp.parts.empty() || sp.parts[0].size() < 2) continue;
        // The local chart about the span's first vertex.
        const double lonC = sp.parts[0][0], latC = sp.parts[0][1];
        const double mx = kLonM * std::cos(latC * 3.14159265358979323846 / 180.0), my = kLatM;
        SpanShape shape;
        shape.area = sp.primitive == 2;
        const bool point = sp.primitive == 0;
        // The reach is the survey's own uncertainty: an NBI point is one coordinate per bridge, an
        // ENC line or area the deck charted to the metre.
        const double withinM = point ? withinPointM : withinShapeM;
        shape.r = withinM;
        double vx0 = 1e9, vy0 = 1e9, vx1 = -1e9, vy1 = -1e9;
        for (const std::vector<double>& part : sp.parts) {
            std::vector<V2> p;
            for (size_t k = 0; k + 1 < part.size(); k += 2) {
                p.push_back({(part[k] - lonC) * mx, (part[k + 1] - latC) * my});
                vx0 = (std::min)(vx0, part[k]);
                vx1 = (std::max)(vx1, part[k]);
                vy0 = (std::min)(vy0, part[k + 1]);
                vy1 = (std::max)(vy1, part[k + 1]);
            }
            // A ring that repeats its first vertex last is closed already.
            if (shape.area && p.size() > 3 && p.front().x == p.back().x && p.front().y == p.back().y) p.pop_back();
            if (!p.empty()) shape.parts.push_back(std::move(p));
        }
        const double padLon = withinM / (std::max)(mx, 1e-6), padLat = withinM / my;
        double nearest = withinM;
        BridgeMatch pick{SIZE_MAX, s, 0.0};
        for (const Box& b : boxes) {
            if (b.lon0 > vx1 + padLon || b.lon1 < vx0 - padLon || b.lat0 > vy1 + padLat || b.lat1 < vy0 - padLat) continue;
            wl.clear();
            for (const RoadPoint& p : ways[b.way].points) wl.push_back({(p.lon - lonC) * mx, (p.lat - latC) * my});
            if (point) {
                // A point span names no deck: only the nearest bridge way within withinM takes it
                // (ties to the lower index, so the answer does not depend on order).
                const double d = shape.DistanceToPoint(wl);
                if (d < nearest || (d == nearest && pick.way != SIZE_MAX && b.way < pick.way)) {
                    const double ov = shape.Overlap(wl);
                    if (ov > 0.0) {
                        nearest = d;
                        pick = {b.way, s, ov};
                    }
                }
                continue;
            }
            const double ov = shape.Overlap(wl);
            if (ov > 0.0) out.push_back({b.way, s, ov});
        }
        if (point && pick.way != SIZE_MAX) out.push_back(pick);
    }
    std::sort(out.begin(), out.end(), [&ways](const BridgeMatch& a, const BridgeMatch& b) {
        if (a.span != b.span) return a.span < b.span;
        if (a.overlapM != b.overlapM) return a.overlapM > b.overlapM;
        if (ways[a.way].rank != ways[b.way].rank) return ways[a.way].rank < ways[b.way].rank;
        return a.way < b.way;
    });
    return out;
}

}  // namespace ga
