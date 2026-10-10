#include "compose/BuildingSolids.h"

#include "core/Common.h"
#include "core/Json.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <fstream>
#include <iterator>

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

// ---- the harvest: <stem>.buildings.json (the manifest and its cell index) + <stem>.buildings.bin.
// One cell's `n` records from `offset`, keeping those whose first point lies in the box.
bool ReadRecords(std::ifstream& f, int64_t offset, int64_t n, double lon0, double lat0, double lon1,
                 double lat1, std::vector<BuildingSolid>& out) {
    f.clear();
    f.seekg(static_cast<std::streamoff>(offset));
    std::vector<int32_t> xy;
    for (int64_t i = 0; i < n; ++i) {
#pragma pack(push, 1)
        struct {
            int64_t id;
            uint8_t kind, roof;
            uint16_t rings;
            float h, mh, lv, mlv, rh;
        } rec;
        struct {
            uint32_t n;
            uint8_t outer, pad[3];
        } ring;
#pragma pack(pop)
        static_assert(sizeof(rec) == 32 && sizeof(ring) == 8, "GABLDG01 record layout");
        if (!f.read(reinterpret_cast<char*>(&rec), sizeof(rec))) return false;
        BuildingSolid s;
        s.id = rec.id;
        s.kind = rec.kind;
        s.height = rec.h;
        s.minHeight = rec.mh;
        s.levels = rec.lv;
        s.minLevel = rec.mlv;
        for (uint16_t r = 0; r < rec.rings; ++r) {
            f.read(reinterpret_cast<char*>(&ring), sizeof(ring));
            xy.resize(2 * size_t(ring.n));
            f.read(reinterpret_cast<char*>(xy.data()), std::streamsize(xy.size() * 4));
            std::vector<double> pts(xy.size());
            for (size_t k = 0; k < xy.size(); ++k) pts[k] = xy[k] * 1e-7;   // the file's own 1e-7 degree
            s.rings.push_back(std::move(pts));
            s.outer.push_back(ring.outer);
        }
        if (!f) return false;
        if (s.rings.empty() || s.rings[0].size() < 6) continue;
        const double x = s.rings[0][0], y = s.rings[0][1];
        if (x >= lon0 && x < lon1 && y >= lat0 && y < lat1) out.push_back(std::move(s));
    }
    return true;
}

float Num(const JsonValue* props, const char* key) {
    const JsonValue* v = props ? props->Get(key) : nullptr;
    if (!v) return BuildingSolid::kUntagged;
    if (v->type == JsonValue::Type::Number) return static_cast<float>(v->number);
    if (v->type == JsonValue::Type::String) {   // OSM spells numbers as strings; metres only here
        char* end = nullptr;
        const float x = std::strtof(v->str.c_str(), &end);
        if (end && end != v->str.c_str() && (*end == 0 || std::strcmp(end, " m") == 0 || std::strcmp(end, "m") == 0)) {
            return x;
        }
    }
    return BuildingSolid::kUntagged;
}

void AddPolygon(const JsonValue& poly, BuildingSolid& s) {
    for (size_t r = 0; r < poly.arr.size(); ++r) {
        std::vector<double> pts;
        for (const JsonValue& p : poly.arr[r].arr) {
            if (p.arr.size() < 2) continue;
            pts.push_back(p.arr[0].number);
            pts.push_back(p.arr[1].number);
        }
        // GeoJSON repeats the first position last; a ring here is implicitly closed.
        if (pts.size() >= 4 && pts[0] == pts[pts.size() - 2] && pts[1] == pts[pts.size() - 1]) {
            pts.resize(pts.size() - 2);
        }
        if (pts.size() < 6) continue;
        s.rings.push_back(std::move(pts));
        s.outer.push_back(r == 0 ? 1 : 0);
    }
}

bool LoadGeoJson(const std::string& path, std::vector<BuildingSolid>& out, std::string* why) {
    std::string err;
    const std::string text = ReadAll(path);
    if (text.empty()) {
        if (why) *why = "cannot read " + path;
        return false;
    }
    const JsonValue root = JsonParser::Parse(text, &err);
    const JsonValue* feats = root.Get("features");
    if (!err.empty() || !feats) {
        if (why) *why = "not a GeoJSON FeatureCollection" + (err.empty() ? "" : ": " + err);
        return false;
    }
    for (const JsonValue& ft : feats->arr) {
        const JsonValue* props = ft.Get("properties");
        const JsonValue* geom = ft.Get("geometry");
        const JsonValue* coords = geom ? geom->Get("coordinates") : nullptr;
        if (!coords) continue;
        BuildingSolid s;
        s.kind = props && props->Get("building:part") ? 1 : 0;
        s.height = Num(props, "height");
        s.minHeight = Num(props, "min_height");
        s.levels = Num(props, "building:levels");
        s.minLevel = Num(props, "building:min_level");
        if (const JsonValue* id = props ? props->Get("id") : nullptr) {
            s.id = id->type == JsonValue::Type::Number ? static_cast<int64_t>(id->number)
                                                       : std::strtoll(id->str.c_str(), nullptr, 10);
        }
        const JsonValue* rm = props ? props->Get("remove") : nullptr;
        s.remove = rm && rm->type == JsonValue::Type::Bool && rm->boolean;
        const std::string type = geom->Str("type");
        if (type == "Polygon") {
            AddPolygon(*coords, s);
        } else if (type == "MultiPolygon") {
            for (const JsonValue& poly : coords->arr) AddPolygon(poly, s);
        }
        if (s.rings.empty()) continue;
        out.push_back(std::move(s));
    }
    return true;
}

double Resolve(float metres, float count, double perCount, double otherwise) {
    if (!std::isnan(metres)) return metres;
    if (!std::isnan(count)) return count * perCount;
    return otherwise;
}

// A footprint set, searched by box first: the cover a source lays over everything beneath it.
struct Cover {
    struct Ring {
        double lon0, lat0, lon1, lat1;
        const std::vector<double>* pts;
    };
    std::vector<Ring> rings;
    std::vector<int64_t> ids;
    void Add(const BuildingSolid& s) {
        for (size_t r = 0; r < s.rings.size(); ++r) {
            if (!s.outer[r]) continue;
            Ring g{1e9, 1e9, -1e9, -1e9, &s.rings[r]};
            for (size_t k = 0; k + 1 < s.rings[r].size(); k += 2) {
                g.lon0 = (std::min)(g.lon0, s.rings[r][k]);
                g.lon1 = (std::max)(g.lon1, s.rings[r][k]);
                g.lat0 = (std::min)(g.lat0, s.rings[r][k + 1]);
                g.lat1 = (std::max)(g.lat1, s.rings[r][k + 1]);
            }
            rings.push_back(g);
        }
        if (s.id != 0) ids.push_back(s.id);
    }
    void Seal() {
        std::sort(rings.begin(), rings.end(), [](const Ring& a, const Ring& b) { return a.lon0 < b.lon0; });
        std::sort(ids.begin(), ids.end());
    }
    bool Holds(const BuildingSolid& s) const {
        if (s.id != 0 && std::binary_search(ids.begin(), ids.end(), s.id)) return true;
        double lon, lat;
        RingCentre(s.rings[0], lon, lat);
        // Rings sorted by their west edge: only those starting west of the point can hold it.
        const auto end = std::upper_bound(rings.begin(), rings.end(), lon,
                                          [](double x, const Ring& r) { return x < r.lon0; });
        for (auto it = rings.begin(); it != end; ++it) {
            if (lon <= it->lon1 && lat >= it->lat0 && lat <= it->lat1 && RingHolds(*it->pts, lon, lat)) {
                return true;
            }
        }
        return false;
    }
};

}  // namespace

bool RingHolds(const std::vector<double>& p, double lon, double lat) {
    bool in = false;
    const size_t n = p.size() / 2;
    for (size_t i = 0, j = n - 1; i < n; j = i++) {
        const double xi = p[2 * i], yi = p[2 * i + 1], xj = p[2 * j], yj = p[2 * j + 1];
        if ((yi > lat) != (yj > lat) && lon < (xj - xi) * (lat - yi) / (yj - yi) + xi) in = !in;
    }
    return in;
}

void RingCentre(const std::vector<double>& p, double& lon, double& lat) {
    const size_t n = p.size() / 2;
    // About the first vertex, so the products stay small (a ring is metres across in degrees).
    const double x0 = p[0], y0 = p[1];
    double a = 0.0, cx = 0.0, cy = 0.0, mx = 0.0, my = 0.0;
    for (size_t i = 0; i < n; ++i) {
        const size_t j = (i + 1) % n;
        const double xi = p[2 * i] - x0, yi = p[2 * i + 1] - y0, xj = p[2 * j] - x0, yj = p[2 * j + 1] - y0;
        const double c = xi * yj - xj * yi;
        a += c;
        cx += (xi + xj) * c;
        cy += (yi + yj) * c;
        mx += xi;
        my += yi;
    }
    if (std::abs(a) > 1e-18) {
        lon = x0 + cx / (3.0 * a);
        lat = y0 + cy / (3.0 * a);
    } else {
        lon = x0 + mx / n;
        lat = y0 + my / n;
    }
}

void BuildingStack::Open(std::vector<BuildingSourceSpec> specs, const BuildingDefaults& d, std::string* log) {
    m_d = d;
    m_src.clear();
    std::stable_sort(specs.begin(), specs.end(),
                     [](const BuildingSourceSpec& a, const BuildingSourceSpec& b) { return a.over < b.over; });
    for (const BuildingSourceSpec& sp : specs) {
        Source src;
        src.name = sp.name;
        std::string why;
        if (EndsWith(sp.path, ".buildings.json")) {
            std::string err;
            const JsonValue m = JsonParser::Parse(ReadAll(sp.path), &err);
            const JsonValue* cells = m.Get("cells");
            src.cellDeg = m.Get("cellDeg") ? m.Get("cellDeg")->number : 0.0;
            src.bin = sp.path.substr(0, sp.path.size() - 5) + ".bin";
            std::ifstream f(src.bin, std::ios::binary);
            char magic[8] = {};
            if (!err.empty() || m.Str("format") != "GABLDG01") {
                why = "not a GABLDG01 manifest" + (err.empty() ? "" : ": " + err);
            } else if (m.Str("datum") != "ground") {   // the only datum a solid here is stood on
                why = "datum '" + m.Str("datum") + "' (only \"ground\" is read)";
            } else if ((!cells && m.Str("cellIndex").empty()) || !(src.cellDeg > 0.0)) {
                why = "the manifest has no cell index";
            } else if (!f || !f.read(magic, 8) || std::memcmp(magic, "GABLDG01", 8) != 0) {
                why = "no GABLDG01 data beside the manifest (" + src.bin + ")";
            } else {
                src.harvest = true;
                if (cells) {
                    for (const JsonValue& c : cells->arr) {
                        if (c.arr.size() < 4) continue;
                        src.cells.push_back({static_cast<int64_t>(c.arr[1].number), static_cast<int64_t>(c.arr[0].number),
                                             static_cast<int64_t>(c.arr[2].number), static_cast<int64_t>(c.arr[3].number)});
                    }
                } else {
                    // The sidecar beside the manifest: 24 bytes a cell.
                    const std::string idx = sp.path.substr(0, sp.path.find_last_of("/\\") + 1) + m.Str("cellIndex");
                    std::ifstream fi(idx, std::ios::binary);
#pragma pack(push, 1)
                    struct {
                        int32_t ix, iy;
                        int64_t off, n;
                    } row;
#pragma pack(pop)
                    while (fi.read(reinterpret_cast<char*>(&row), sizeof(row))) src.cells.push_back({row.iy, row.ix, row.off, row.n});
                    if (src.cells.empty()) why = "no cell index at " + idx;
                }
                std::sort(src.cells.begin(), src.cells.end());
            }
        } else if (EndsWith(sp.path, ".geojson") || EndsWith(sp.path, ".json")) {
            LoadGeoJson(sp.path, src.whole, &why);
        } else {
            why = "neither a .buildings.json harvest nor a .geojson";
        }
        char line[512];
        if (!why.empty()) {
            snprintf(line, sizeof(line), "source '%s' refused: %s\n", sp.name.c_str(), why.c_str());
        } else if (src.harvest) {
            snprintf(line, sizeof(line), "source '%s' (over %g): %zu cells of %.2f deg, read by box\n",
                     sp.name.c_str(), sp.over, src.cells.size(), src.cellDeg);
        } else {
            snprintf(line, sizeof(line), "source '%s' (over %g): %zu solids, read whole\n", sp.name.c_str(),
                     sp.over, src.whole.size());
        }
        if (log) *log += line;
        if (why.empty()) m_src.push_back(std::move(src));
    }
}

std::vector<std::pair<int, int>> BuildingStack::Cells(double cellDeg) const {
    std::vector<std::pair<int, int>> out;   // (iy, ix) while gathering, for the sort
    for (const Source& s : m_src) {
        if (s.harvest) {
            const double f = s.cellDeg / cellDeg;
            const int n = static_cast<int>(std::lround(f));
            for (const auto& c : s.cells) {   // a coarser harvest cell covers n x n of ours
                for (int dy = 0; dy < (std::max)(n, 1); ++dy) {
                    for (int dx = 0; dx < (std::max)(n, 1); ++dx) {
                        out.push_back({static_cast<int>(std::floor(c[0] * f)) + dy, static_cast<int>(std::floor(c[1] * f)) + dx});
                    }
                }
            }
        } else {
            for (const BuildingSolid& b : s.whole) {
                if (b.rings.empty() || b.rings[0].size() < 2) continue;
                out.push_back({static_cast<int>(std::floor(b.rings[0][1] / cellDeg)),
                               static_cast<int>(std::floor(b.rings[0][0] / cellDeg))});
            }
        }
    }
    std::sort(out.begin(), out.end());
    out.erase(std::unique(out.begin(), out.end()), out.end());
    for (auto& c : out) std::swap(c.first, c.second);
    return out;
}

void BuildingStack::Read(const Source& s, double lon0, double lat0, double lon1, double lat1,
                         std::vector<BuildingSolid>& out) const {
    if (!s.harvest) {
        for (const BuildingSolid& b : s.whole) {
            const double x = b.rings[0][0], y = b.rings[0][1];
            if (x >= lon0 && x < lon1 && y >= lat0 && y < lat1) out.push_back(b);
        }
        return;
    }
    std::ifstream f(s.bin, std::ios::binary);   // this call's own handle: Compose runs on many threads
    const int64_t ix0 = static_cast<int64_t>(std::floor(lon0 / s.cellDeg));
    const int64_t ix1 = static_cast<int64_t>(std::floor(lon1 / s.cellDeg));
    const int64_t iy0 = static_cast<int64_t>(std::floor(lat0 / s.cellDeg));
    const int64_t iy1 = static_cast<int64_t>(std::floor(lat1 / s.cellDeg));
    // A row of the box at a time: the first cell at or after (iy, ix0), then along the row. The
    // planet is millions of cells; a box is a handful.
    for (int64_t iy = iy0; iy <= iy1; ++iy) {
        auto it = std::lower_bound(s.cells.begin(), s.cells.end(), std::array<int64_t, 4>{iy, ix0, INT64_MIN, INT64_MIN});
        for (; it != s.cells.end() && (*it)[0] == iy && (*it)[1] <= ix1; ++it) {
            if (!ReadRecords(f, (*it)[2], (*it)[3], lon0, lat0, lon1, lat1, out)) {
                Log("[buildings] '%s': a truncated cell at offset %lld", s.name.c_str(), static_cast<long long>((*it)[2]));
            }
        }
    }
}

std::vector<BuildingSolid> BuildingStack::Compose(double lon0, double lat0, double lon1, double lat1, double margin,
                                                  uint64_t* identity) const {
    std::vector<std::vector<BuildingSolid>> stack(m_src.size());
    for (size_t k = 0; k < m_src.size(); ++k) {
        Read(m_src[k], lon0 - margin, lat0 - margin, lon1 + margin, lat1 + margin, stack[k]);
    }
    std::vector<BuildingSolid> all = ComposeBuildings(std::move(stack), m_d, identity);
    std::vector<BuildingSolid> mine;
    for (BuildingSolid& s : all) {
        const double x = s.rings[0][0], y = s.rings[0][1];
        if (x >= lon0 && x < lon1 && y >= lat0 && y < lat1) mine.push_back(std::move(s));
    }
    return mine;
}

std::vector<BuildingSolid> ComposeBuildings(std::vector<std::vector<BuildingSolid>> stack,
                                            const BuildingDefaults& d, uint64_t* identity) {
    uint64_t h = 14695981039346656037ull;
    auto mix = [&h](const void* p, size_t n) {
        for (size_t i = 0; i < n; ++i) h = (h ^ static_cast<const uint8_t*>(p)[i]) * 1099511628211ull;
    };
    mix(&d.levelHeight, sizeof(double));
    mix(&d.defaultHeight, sizeof(double));
    std::vector<BuildingSolid> out;
    Cover above;   // every footprint of every source above the one being read
    for (size_t k = stack.size(); k-- > 0;) {
        std::vector<BuildingSolid>& src = stack[k];
        // Simple 3D Buildings: an outline whose footprint holds a part is drawn by its parts.
        Cover parts;
        for (const BuildingSolid& s : src) {
            if (s.kind == 1) parts.Add(s);
        }
        parts.Seal();
        std::vector<char> drawn(src.size(), 0);
        for (size_t i = 0; i < src.size(); ++i) {
            const BuildingSolid& s = src[i];
            if (s.remove || above.Holds(s)) continue;
            if (s.kind == 0 && !parts.rings.empty()) {
                bool holdsPart = false;
                for (const auto& r : parts.rings) {
                    double lon, lat;
                    RingCentre(*r.pts, lon, lat);
                    if (RingHolds(s.rings[0], lon, lat)) {
                        holdsPart = true;
                        break;
                    }
                }
                if (holdsPart) continue;
            }
            drawn[i] = 1;
        }
        for (size_t i = 0; i < src.size(); ++i) above.Add(src[i]);   // removals cover too
        above.Seal();
        for (size_t i = 0; i < src.size(); ++i) {
            if (!drawn[i]) continue;
            BuildingSolid s = src[i];   // a copy: `above` points into src for the sources beneath
            s.top = Resolve(s.height, s.levels, d.levelHeight, d.defaultHeight);
            s.bottom = Resolve(s.minHeight, s.minLevel, d.levelHeight, 0.0);
            if (!(s.top > s.bottom)) continue;
            mix(&s.id, sizeof(s.id));
            mix(&s.top, sizeof(s.top));
            mix(&s.bottom, sizeof(s.bottom));
            s.source = static_cast<int>(k);
            for (const auto& r : s.rings) mix(r.data(), r.size() * sizeof(double));
            out.push_back(std::move(s));
        }
    }
    if (identity) *identity = h;
    return out;
}

}  // namespace ga
