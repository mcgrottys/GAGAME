#include <limits>
#include "compose/RasterFileSource.h"

#include "compose/ColorStackSource.h"
#include "compose/DomainSource.h"
#include "compose/HeightStackSource.h"
#include "compose/TileTree.h"
#include "core/Common.h"
#include "core/ImageLoader.h"
#include "core/ThreadManager.h"

#include <algorithm>
#include <atomic>
#include <cctype>
#include <chrono>
#include <cmath>
#include <cstring>
#include <filesystem>

namespace ga {

std::atomic<uint64_t> RasterFileSource::s_decodes{0}, RasterFileSource::s_decodeUs{0}, RasterFileSource::s_edges{0},
    RasterFileSource::s_edgeUs{0};

namespace {

constexpr double kPi = 3.14159265358979323846, kD2R = kPi / 180.0;
constexpr double kR = 6378137.0;   // WGS84 a: Web Mercator's sphere, and a degree's metres of arc
constexpr uint32_t kWin = 256;     // a decoded window's side, texels
constexpr size_t kWindows = 1024;  // the windows a source keeps: 256 MB at most, and only what was touched.
// MEASURED 2026-10-17 at 256: a cold view over a 62 Mpx bed (980 windows) and 25 lidar files (400) decoded
// 12,971 windows for 934 tile loads of 1.4 s each -- fourteen decodes a load, the same windows over and
// over, the picture never settling; the cache held a fifth of the working set.

bool Fail(std::string* why, std::string w) {
    if (why) *why = std::move(w);
    return false;
}

// A folder entry's pattern: `*` and `?`, case-insensitive.
bool Matches(const char* p, const char* s) {
    if (*p == '*') return Matches(p + 1, s) || (*s && Matches(p, s + 1));
    if (!*s) return !*p;
    return (*p == '?' || std::tolower(static_cast<unsigned char>(*p)) ==
                             std::tolower(static_cast<unsigned char>(*s))) &&
           Matches(p + 1, s + 1);
}

// One file through the registry, the entry's `crs` and `kind` read only where the file cannot say.
std::unique_ptr<FieldLoader> OpenOne(const LoaderRegistry& reg, const std::string& path,
                                     const RasterEntry& e, std::string* why) {
    GeoRef said;   // the entry's word, for a file that has none of its own
    if (!e.crs.empty() && (said.epsg = ParseEpsg(e.crs)) == 0) {
        Fail(why, "its entry's crs '" + e.crs + "' is not EPSG:nnnn");
        return nullptr;
    }
    if (e.kind == "colour" || e.kind == "color") said.valueUnit = "sRGB byte";
    else if (e.kind == "height") said.valueUnit = "m";
    else if (!e.kind.empty()) {
        Fail(why, "its entry's kind '" + e.kind + "' is neither colour nor height");
        return nullptr;
    }
    const std::string ext = std::filesystem::path(path).extension().string();
    std::unique_ptr<FieldLoader> L =
        reg.Open(path, (said.epsg > 0 || !e.kind.empty()) ? &said : nullptr);
    if (!L) {
        Fail(why, reg.Knows(ext.empty() ? ext : ext.substr(1)) ? ImageLoaderWhy()
                                                               : "no loader reads " + ext);
    } else if (L->Channels() != 4 && L->Channels() != 1) {
        Fail(why, "its loader gives neither colour (RGBA) nor a height (one channel)");
        L.reset();
    }
    return L;
}

// The windows a painting thread used last: a texel read is then no lock, and four hold every
// window one bilinear read can touch. Keyed by the source's serial, never its address. The edges
// beside them, the same way.
struct Recent {
    uint64_t owner = 0, key = ~0ull;
    std::shared_ptr<const std::vector<uint8_t>> px;
};
thread_local Recent tRecent[4];
thread_local unsigned tNext = 0;
struct RecentEdge {
    uint64_t owner = 0, key = ~0ull;
    std::shared_ptr<const void> e;
};
thread_local RecentEdge tRecentEdge[4];
thread_local unsigned tNextEdge = 0;
std::atomic<uint64_t> gSerial{0};

// ONE AXIS OF THE DISTANCE TRANSFORM (Felzenszwalb and Huttenlocher 2004, the lower envelope of
// parabolas): d[q] = min over p of s2 (q - p)^2 + f[p], exact, n steps; s2 is the axis's texel
// squared in metres. f is 0 at an absent texel and `far` elsewhere; v and z are scratch of n and
// n + 1. Each axis in turn gives the squared Euclidean distance in metres.
void Dt1(const double* f, double* d, int n, double s2, int* v, double* z) {
    int k = 0;
    v[0] = 0;
    z[0] = -1e300;
    z[1] = 1e300;
    const auto meet = [&](int q, int r) {   // where parabola q overtakes parabola r (r < q)
        return ((f[q] + s2 * q * q) - (f[r] + s2 * r * r)) / (2.0 * s2 * (q - r));
    };
    for (int q = 1; q < n; ++q) {
        double s = meet(q, v[k]);
        while (s <= z[k]) s = meet(q, v[--k]);   // z[0] is -1e300: k never falls under 0
        ++k;
        v[k] = q;
        z[k] = s;
        z[k + 1] = 1e300;
    }
    k = 0;
    for (int q = 0; q < n; ++q) {
        while (z[k + 1] < q) ++k;
        d[q] = s2 * (q - v[k]) * (q - v[k]) + f[v[k]];
    }
}

}  // namespace

RasterFileSource::RasterFileSource() : m_serial(++gSerial) {}

bool RasterFileSource::Load(const LoaderRegistry& reg, const std::string& path,
                            const RasterEntry& e, std::string* why) {
    std::unique_ptr<FieldLoader> L = OpenOne(reg, path, e, why);
    if (!L) return false;
    std::vector<std::unique_ptr<FieldLoader>> one;
    one.push_back(std::move(L));
    return Open(std::move(one), e, path, why);
}

bool RasterFileSource::Open(std::vector<std::unique_ptr<FieldLoader>> files, const RasterEntry& e,
                            const std::string& label, std::string* why) {
    if (files.empty()) return Fail(why, "no file");
    m_files.clear();
    for (auto& L : files) {
        auto f = std::make_unique<File>();
        f->ref = L->Ref();
        f->loader = std::move(L);
        m_files.push_back(std::move(f));
    }
    m_over = e.over;
    m_feather = e.feather;
    m_entry = e;
    if (!Placed(why)) return false;
    // Whether a texel inside the box can be absent -- a file's own (its loader says), or ground
    // the files do not tile. Only then does the feather reach absence, and only then is it named.
    m_absence = !Tiled();
    for (const auto& f : m_files) m_absence = m_absence || f->loader->MayBeAbsent();
    // THE IDENTITY: one file's structure, or the set's, and the feather that weighs it.
    char b[96];
    if (m_files.size() == 1) {
        m_info.structure = m_files[0]->loader->Structure();
    } else {
        uint64_t h = 1469598103934665603ull;
        for (const auto& f : m_files) {
            for (const char* c = f->loader->Structure(); *c; ++c) h = (h ^ uint8_t(*c)) * 1099511628211ull;
        }
        snprintf(b, sizeof(b), ", fnv64 %016llx (r2)", static_cast<unsigned long long>(h));
        m_info.structure = "set of " + std::to_string(m_files.size()) + " files, " + m_info.crs + b;
    }
    if (m_feather > 0.0) {
        snprintf(b, sizeof(b), ", feather %g m%s", m_feather, m_absence ? " at the box and at absence" : "");
        m_info.structure += b;
    }
    if (m_height) m_info.structure += ", " + m_valued;
    m_info.name = e.name.empty() ? std::filesystem::path(label).stem().string() : e.name;
    Log("[raster] %s: %s -- grain %.4g m, %.6f..%.6f E, %.6f..%.6f N, %zu file(s), over %g",
        m_info.name.c_str(), m_info.structure.c_str(), m_grainM, m_info.lon0, m_info.lon1,
        m_info.lat0, m_info.lat1, m_files.size(), m_over);
    return true;
}

bool RasterFileSource::Place(const GeoRef& ref, std::string* why) {
    m_files[0]->ref = ref;
    return Placed(why);
}

bool RasterFileSource::Placed(std::string* why) {
    const GeoRef& r0 = m_files[0]->ref;
    int zone = 0;
    bool south = false;
    std::string w;
    if (!CrsOfEpsg(r0.epsg, m_kind, zone, south, &w)) return Fail(why, w);
    if (zone) {
        m_tm = TransverseMercator::Utm(zone);
        m_tm.falseN = south ? 10000000.0 : 0.0;
    }
    m_ux0 = m_uy0 = 1e300;
    m_ux1 = m_uy1 = -1e300;
    double lo0 = 1e9, la0 = 1e9, lo1 = -1e9, la1 = -1e9;
    for (const auto& fp : m_files) {
        File& f = *fp;
        const GeoRef& g = f.ref;
        if (g.epsg != r0.epsg) {
            return Fail(why, "it mixes EPSG:" + std::to_string(r0.epsg) + " and EPSG:" +
                                 std::to_string(g.epsg) + " -- one source is one CRS");
        }
        if (std::abs(std::abs(g.scaleX) / std::abs(r0.scaleX) - 1.0) > 1e-9 ||
            std::abs(std::abs(g.scaleY) / std::abs(r0.scaleY) - 1.0) > 1e-9) {
            return Fail(why, "it mixes grains -- one source is one grain");
        }
        if (f.loader->Channels() != m_files[0]->loader->Channels() || std::strcmp(g.valueUnit, r0.valueUnit)) {
            return Fail(why, "it mixes kinds or units -- one source is one kind of value in one unit");
        }
        f.w = g.width;
        f.h = g.height;
        f.x0 = g.originX;
        f.x1 = g.originX + g.scaleX * g.width;
        const double yb = g.originY + g.scaleY * g.height;
        f.down = g.scaleY < 0.0;
        f.yLo = (std::min)(g.originY, yb);
        f.yHi = (std::max)(g.originY, yb);
        f.kx = g.width / (f.x1 - f.x0);
        f.ky = g.height / (f.yHi - f.yLo);
        m_ux0 = (std::min)(m_ux0, f.x0);
        m_ux1 = (std::max)(m_ux1, f.x1);
        m_uy0 = (std::min)(m_uy0, f.yLo);
        m_uy1 = (std::max)(m_uy1, f.yHi);
        // The footprint in the exchange frame: each file's corners and midpoints, and its centre.
        for (int i = 0; i < 9; ++i) {
            double la = 0.0, lo = 0.0;
            if (!ToLatLon(f.x0 + (f.x1 - f.x0) * 0.5 * (i % 3), f.yLo + (f.yHi - f.yLo) * 0.5 * (i / 3),
                          la, lo)) {
                return Fail(why, "its corners do not resolve to WGS84");
            }
            lo0 = (std::min)(lo0, lo / kD2R);
            lo1 = (std::max)(lo1, lo / kD2R);
            la0 = (std::min)(la0, la / kD2R);
            la1 = (std::max)(la1, la / kD2R);
        }
    }
    m_info.lon0 = lo0;
    m_info.lat0 = la0;
    m_info.lon1 = lo1;
    m_info.lat1 = la1;
    m_info.crs = r0.Describe();
    // THE GRAIN: one texel's ground at the centre, each axis, metres; the finer is the grain.
    const double c = std::cos(0.5 * (la0 + la1) * kD2R), deg = kR * kD2R;
    const bool geo = m_kind == CrsKind::Geographic, merc = m_kind == CrsKind::WebMercator;
    m_mx = geo ? deg * c : merc ? c : 1.0;
    m_my = geo ? deg : merc ? c : 1.0;
    m_gx = std::abs(r0.scaleX) * m_mx;
    m_gy = std::abs(r0.scaleY) * m_my;
    m_grainM = (std::min)(m_gx, m_gy);
    m_info.cmPerPixel = 100.0 * m_grainM;
    m_height = m_files[0]->loader->Channels() == 1;
    return !m_height || Valued(why);
}

// A HEIGHT'S VALUE, declared and never guessed (HIERARCHY 4.11): the unit and the datum the file
// names (its GeoKeys, as the loader reads them into valueUnit), else the entry's -- where both
// speak and differ, the file's stands and that is said. On the engine's datum it is taken as it
// is; on another, only with the entry's offset, the separation at the place, added here once.
bool RasterFileSource::Valued(std::string* why) {
    const UnitSpec file = UnitSpec::Parse(m_files[0]->ref.valueUnit), frame = UnitSpec::Parse(kHeightFrame);
    const UnitSpec unit = UnitSpec::Parse(m_entry.unit.c_str());
    const UnitSpec datum = UnitSpec::Parse(("m " + m_entry.datum).c_str());
    const bool fileUnit = file.quantity == Quantity::Length;
    const char* n = m_files[0]->loader->Name();
    if (fileUnit && !m_entry.unit.empty() && unit.toCanonical != file.toCanonical) {
        Log("[raster] %s: the file's own unit (%s) stands; the entry's '%s' is not read", n, file.raw.c_str(),
            m_entry.unit.c_str());
    }
    if (!file.datum.empty() && !m_entry.datum.empty() && datum.datum != file.datum) {
        Log("[raster] %s: the file's own datum %s stands; the entry's '%s' is not read", n, file.datum.c_str(),
            m_entry.datum.c_str());
    }
    const double scale = fileUnit ? file.toCanonical : unit.quantity == Quantity::Length ? unit.toCanonical : 0.0;
    const std::string d = !file.datum.empty() ? file.datum : datum.datum;
    if (scale <= 0.0) {
        return Fail(why, "its unit is not declared: the file names none, and the entry's unit '" + m_entry.unit +
                             "' is not a length (m, ft, ftUS)");
    }
    if (d.empty()) {
        return Fail(why, "its datum is not declared: the file names none, and the entry's datum '" +
                             m_entry.datum + "' is none this engine names (NAVD88, MSL, EGM96, EGM2008, WGS84 ...)");
    }
    if (d != frame.datum && !m_entry.hasOffset) {
        return Fail(why, "its datum is " + d + " and the engine's is " + frame.datum +
                             ": it is taken only with the entry's offset, the separation at the place, in metres");
    }
    if (d == frame.datum && m_entry.hasOffset) {
        Log("[raster] %s: on %s already, so the entry's offset %+g m is not added", n, d.c_str(), m_entry.offset);
    }
    m_vScale = scale;
    m_vOffset = d == frame.datum ? 0.0 : m_entry.offset;
    char b[160];
    snprintf(b, sizeof(b), "height on %s x%.10g %+.6g m -> %s", d.c_str(), m_vScale, m_vOffset, kHeightFrame);
    m_valued = b;
    return true;
}

// Do the files tile the set's box? The files' edges cut the box into cells; every cell's centre
// must lie in some file. N files cut at most (2N)^2 cells, each asked of N files.
bool RasterFileSource::Tiled() const {
    std::vector<double> xs, ys;
    for (const auto& f : m_files) {
        xs.push_back(f->x0);
        xs.push_back(f->x1);
        ys.push_back(f->yLo);
        ys.push_back(f->yHi);
    }
    std::sort(xs.begin(), xs.end());
    std::sort(ys.begin(), ys.end());
    for (size_t j = 0; j + 1 < ys.size(); ++j) {
        for (size_t i = 0; i + 1 < xs.size(); ++i) {
            if (xs[i + 1] <= xs[i] || ys[j + 1] <= ys[j]) continue;
            const double x = 0.5 * (xs[i] + xs[i + 1]), y = 0.5 * (ys[j] + ys[j + 1]);
            bool held = false;
            for (const auto& f : m_files) held = held || (x >= f->x0 && x < f->x1 && y >= f->yLo && y < f->yHi);
            if (!held) return false;
        }
    }
    return true;
}

bool RasterFileSource::Footprint(double& lon0, double& lat0, double& lon1, double& lat1) const {
    lon0 = m_info.lon0;
    lat0 = m_info.lat0;
    lon1 = m_info.lon1;
    lat1 = m_info.lat1;
    return !m_files.empty();
}

int RasterFileSource::OwnMip(const Lattice& lattice) const {
    if (m_files.empty()) return -1;
    const double lat = 0.5 * (m_info.lat0 + m_info.lat1) * kD2R;
    const double lon = 0.5 * (m_info.lon0 + m_info.lon1) * kD2R;
    return (std::max)(lattice.LevelOf(m_grainM, lat, lon), 0);
}

bool RasterFileSource::ToCrs(double lat, double lon, double& x, double& y) const {
    switch (m_kind) {
        case CrsKind::Geographic:
            x = lon / kD2R;
            y = lat / kD2R;
            return true;
        case CrsKind::WebMercator:
            if (std::abs(lat) > 85.05112878 * kD2R) return false;
            x = kR * lon;
            y = kR * std::log(std::tan(0.25 * kPi + 0.5 * lat));
            return true;
        case CrsKind::TransverseMercator:
            m_tm.Forward(lat, lon, x, y);
            return true;
        default:
            return false;
    }
}

bool RasterFileSource::ToLatLon(double x, double y, double& lat, double& lon) const {
    if (m_kind == CrsKind::Geographic) {
        lat = y * kD2R;
        lon = x * kD2R;
        return std::abs(y) <= 90.0;
    }
    if (m_kind == CrsKind::WebMercator) {
        lon = x / kR;
        lat = 2.0 * std::atan(std::exp(y / kR)) - 0.5 * kPi;
        return true;
    }
    // Transverse Mercator: Newton on the forward form the samples use, so the footprint and the
    // samples cannot disagree about where the file is.
    lat = (y - m_tm.falseN) / (kR * 0.9996);
    lon = m_tm.lon0Rad + (x - m_tm.falseE) / (kR * 0.9996 * std::cos(lat));
    for (int it = 0; it < 20; ++it) {
        const double h = 1e-7;
        double e, n, e1, n1, e2, n2;
        m_tm.Forward(lat, lon, e, n);
        m_tm.Forward(lat + h, lon, e1, n1);
        m_tm.Forward(lat, lon + h, e2, n2);
        const double a = (e1 - e) / h, b = (e2 - e) / h, c = (n1 - n) / h, d = (n2 - n) / h;
        const double det = a * d - b * c, de = x - e, dn = y - n;
        if (det == 0.0 || !std::isfinite(det)) return false;
        lat += (d * de - b * dn) / det;
        lon += (a * dn - c * de) / det;
        if (std::abs(de) + std::abs(dn) < 1e-6) return true;
    }
    return std::abs(lat) < 0.5 * kPi;
}

// ---- BY THE WINDOW: 256 x 256 texels of one file, RGBA8 as the seam gives them (0 where the
// file said nodata), decoded once and kept while used. ONE DECODE A WINDOW (2026-10-17): the loader
// serializes its decoder, and the ingest's pool threads paint neighbouring tiles, so a window two
// threads wanted at once was decoded twice behind that lock -- measured as 44,347 decodes of a 5,430-
// window file, eight times over, with the decoder the whole of the ingest's time. A thread that finds
// a window being decoded waits for it (m_decoding, m_decoded) rather than decoding it again.
std::shared_ptr<const std::vector<uint8_t>> RasterFileSource::Window(size_t file, uint32_t wx,
                                                                     uint32_t wy) {
    const uint64_t key = (uint64_t(file) << 48) | (uint64_t(wy) << 24) | wx;
    {
        std::unique_lock<std::mutex> lk(m_cacheMx);
        for (;;) {
            auto it = m_index.find(key);
            if (it != m_index.end()) {
                m_lru.splice(m_lru.begin(), m_lru, it->second);
                return it->second->second;
            }
            if (m_decoding.insert(key).second) break;   // nobody is on it: this thread decodes it
            m_decoded.wait(lk);                          // another thread is: its window lands below
        }
    }
    auto px = std::make_shared<std::vector<uint8_t>>(size_t(kWin) * kWin * 4, uint8_t(m_height ? 0xFF : 0));
    TilePayload tp;
    const auto tDec = std::chrono::steady_clock::now();
    const bool got = m_files[file]->loader->LoadTile(wx, wy, kWin, kWin, tp);
    s_decodes.fetch_add(1, std::memory_order_relaxed);
    s_decodeUs.fetch_add(uint64_t(std::chrono::duration<double, std::micro>(std::chrono::steady_clock::now() - tDec).count()), std::memory_order_relaxed);
    if (m_height && tp.data.size() * 4 == px->size()) {
        std::memcpy(px->data(), tp.data.data(), px->size());   // a height's floats; NaN (0xFF..) is absence
    } else if (got && !m_height) {
        for (size_t k = 0; k < px->size(); ++k) (*px)[k] = static_cast<uint8_t>(tp.data[k]);
    }
    std::lock_guard<std::mutex> lk(m_cacheMx);
    m_decoding.erase(key);
    m_decoded.notify_all();
    m_lru.emplace_front(key, px);
    m_index[key] = m_lru.begin();
    if (m_lru.size() > kWindows) {
        m_index.erase(m_lru.back().first);
        m_lru.pop_back();
    }
    return m_lru.front().second;
}

std::shared_ptr<const RasterFileSource::Sat> RasterFileSource::WindowSat(size_t file, uint32_t wx, uint32_t wy) {
    const uint64_t key = (uint64_t(file) << 48) | (uint64_t(wy) << 24) | wx;
    {
        std::lock_guard<std::mutex> lk(m_cacheMx);
        auto it = m_satIndex.find(key);
        if (it != m_satIndex.end()) {
            m_satLru.splice(m_satLru.begin(), m_satLru, it->second);
            return it->second->second;
        }
    }
    const auto px = Window(file, wx, wy);
    const auto edge = WindowEdge(file, wx, wy);
    const float* ef = edge && !edge->f.empty() ? edge->f.data() : nullptr;
    auto s = std::make_shared<Sat>();
    const size_t N = kWin + 1;
    s->v.assign(N * N, 0.0);
    s->n.assign(N * N, 0.0);
    for (uint32_t j = 0; j < kWin; ++j) {
        double rowV = 0.0, rowN = 0.0;
        for (uint32_t i = 0; i < kWin; ++i) {
            float t;
            std::memcpy(&t, &(*px)[(size_t(j) * kWin + i) * 4], 4);
            if (!std::isnan(t)) {   // a texel weighs its edge factor (1 with none): the table carries the law
                const double e = ef ? double(ef[size_t(j) * kWin + i]) : 1.0;
                rowV += e * t;
                rowN += e;
            }
            s->v[(j + 1) * N + i + 1] = s->v[j * N + i + 1] + rowV;
            s->n[(j + 1) * N + i + 1] = s->n[j * N + i + 1] + rowN;
        }
    }
    std::lock_guard<std::mutex> lk(m_cacheMx);
    auto it = m_satIndex.find(key);
    if (it != m_satIndex.end()) return it->second->second;
    m_satLru.emplace_front(key, s);
    m_satIndex[key] = m_satLru.begin();
    if (m_satLru.size() > kWindows / 4) {   // a table is four times a window's bytes
        m_satIndex.erase(m_satLru.back().first);
        m_satLru.pop_back();
    }
    return m_satLru.front().second;
}

std::shared_ptr<const RasterFileSource::Blocks> RasterFileSource::WindowBlocks(size_t file, uint32_t wx, uint32_t wy) {
    const uint64_t key = (uint64_t(file) << 48) | (uint64_t(wy) << 24) | wx;
    {
        std::lock_guard<std::mutex> lk(m_cacheMx);
        auto it = m_blkIndex.find(key);
        if (it != m_blkIndex.end()) {
            m_blkLru.splice(m_blkLru.begin(), m_blkLru, it->second);
            return it->second->second;
        }
    }
    const auto px = Window(file, wx, wy);
    const auto edge = WindowEdge(file, wx, wy);
    const float* ef = edge && !edge->f.empty() ? edge->f.data() : nullptr;
    auto b = std::make_shared<Blocks>();
    for (int k = 0; k < 256; ++k) {
        b->mn[k] = std::numeric_limits<float>::infinity();
        b->mx[k] = -std::numeric_limits<float>::infinity();
        b->nan[k] = 0;
    }
    for (uint32_t j = 0; j < kWin; ++j) {
        for (uint32_t i = 0; i < kWin; ++i) {
            float t;
            std::memcpy(&t, &(*px)[(size_t(j) * kWin + i) * 4], 4);
            const int k = int(j / 16) * 16 + int(i / 16);
            if (std::isnan(t)) {
                b->nan[k] = 1;
                continue;
            }
            if (ef && ef[size_t(j) * kWin + i] < 1.0f) b->nan[k] = 1;   // under the feather: not the block's whole
            b->mn[k] = (std::min)(b->mn[k], t);
            b->mx[k] = (std::max)(b->mx[k], t);
        }
    }
    std::lock_guard<std::mutex> lk(m_cacheMx);
    auto it = m_blkIndex.find(key);
    if (it != m_blkIndex.end()) return it->second->second;
    m_blkLru.emplace_front(key, b);
    m_blkIndex[key] = m_blkLru.begin();
    if (m_blkLru.size() > kWindows) {   // a few KB each
        m_blkIndex.erase(m_blkLru.back().first);
        m_blkLru.pop_back();
    }
    return m_blkLru.front().second;
}

// THE EDGE OF THE DATA (header): the window's texels, each the smoothstep of its distance in metres to
// the nearest absent texel over the feather. The seed is the window and a ring of `reach` texels about
// it (the feather in texels, so every absent texel that can lower a factor is in it), read through
// Absent: this file's texels, or the file of the set that holds the ground beyond this one's edge.
// (kWin + 2 reach)^2 cells, two axes of Dt1 each: a window of a 10 m feather at 1 m costs 76k cells.
std::shared_ptr<const RasterFileSource::Edge> RasterFileSource::WindowEdge(size_t file, uint32_t wx, uint32_t wy) {
    if (m_feather <= 0.0 || !m_absence) return nullptr;
    const uint64_t key = (uint64_t(file) << 48) | (uint64_t(wy) << 24) | wx;
    {
        std::lock_guard<std::mutex> lk(m_cacheMx);
        auto it = m_edgeIndex.find(key);
        if (it != m_edgeIndex.end()) {
            m_edgeLru.splice(m_edgeLru.begin(), m_edgeLru, it->second);
            return it->second->second;
        }
    }
    const int reach = (std::max)(1, int(std::ceil(m_feather / (std::min)(m_gx, m_gy))));
    const int P = int(kWin) + 2 * reach, X0 = int(wx * kWin) - reach, Y0 = int(wy * kWin) - reach;
    constexpr double kFar = 1e30;   // (`far` is a Windows macro)
    const auto tEdge = std::chrono::steady_clock::now();
    std::vector<double> g(size_t(P) * P, kFar), d(size_t(P) * P);
    bool any = false;
    WinCache wc;
    for (int j = 0; j < P; ++j) {
        for (int i = 0; i < P; ++i) {
            if (Absent(file, X0 + i, Y0 + j, wc)) {
                g[size_t(j) * P + i] = 0.0;
                any = true;
            }
        }
    }
    auto e = std::make_shared<Edge>();
    if (any) {   // (a window whose present texels all weigh 1 stays empty below: every absent texel weighs 0 anyway)
        std::vector<int> v(P);
        std::vector<double> z(P + 1), col(P), out(P);
        for (int i = 0; i < P; ++i) {   // down each column, the window's rows in metres
            for (int j = 0; j < P; ++j) col[j] = g[size_t(j) * P + i];
            Dt1(col.data(), out.data(), P, m_gy * m_gy, v.data(), z.data());
            for (int j = 0; j < P; ++j) d[size_t(j) * P + i] = out[j];
        }
        for (int j = 0; j < P; ++j) Dt1(&d[size_t(j) * P], &g[size_t(j) * P], P, m_gx * m_gx, v.data(), z.data());
        e->f.resize(size_t(kWin) * kWin);
        bool trivial = true;
        for (uint32_t j = 0; j < kWin; ++j) {
            for (uint32_t i = 0; i < kWin; ++i) {
                const double m2 = g[size_t(j + reach) * P + i + reach];
                const double t = std::clamp(std::sqrt(m2) / m_feather, 0.0, 1.0);
                const float f = float(t * t * (3.0 - 2.0 * t));
                e->f[size_t(j) * kWin + i] = f;
                if (m2 > 0.0 && f < 1.0f) trivial = false;   // a present texel under the feather
            }
        }
        if (trivial) e->f.clear();
    }
    s_edges.fetch_add(1, std::memory_order_relaxed);
    s_edgeUs.fetch_add(uint64_t(std::chrono::duration<double, std::micro>(std::chrono::steady_clock::now() - tEdge).count()),
                       std::memory_order_relaxed);
    std::lock_guard<std::mutex> lk(m_cacheMx);
    auto it = m_edgeIndex.find(key);
    if (it != m_edgeIndex.end()) return it->second->second;
    m_edgeLru.emplace_front(key, e);
    m_edgeIndex[key] = m_edgeLru.begin();
    // As many as the windows: at a quarter of them the edges of a 7,400-window run were built 53,483
    // times (686 s) and the picture never settled (2026-10-17). A window's bytes each where it holds
    // anything; most hold nothing (no absent texel within the feather, or none present).
    if (m_edgeLru.size() > kWindows) {
        m_edgeIndex.erase(m_edgeLru.back().first);
        m_edgeLru.pop_back();
    }
    return m_edgeLru.front().second;
}

// A texel of this file is absent where it holds NaN (a height) or alpha 0 (a colour). Ground beyond
// this file's edge is the texel of the file of the set that holds it, absent where none does -- and
// present beyond the set's box, whose edge is Feathered's.
bool RasterFileSource::Absent(size_t file, int x, int y, WinCache& wc) {
    const File& f = *m_files[file];
    if (x < 0 || y < 0 || x >= int(f.w) || y >= int(f.h)) {
        const double X = f.x0 + (x + 0.5) / f.kx;
        const double Y = f.down ? f.yHi - (y + 0.5) / f.ky : f.yLo + (y + 0.5) / f.ky;
        if (X < m_ux0 || X >= m_ux1 || Y < m_uy0 || Y >= m_uy1) return false;
        for (size_t j = 0; j < m_files.size(); ++j) {
            const File& o = *m_files[j];
            if (j == file || X < o.x0 || X >= o.x1 || Y < o.yLo || Y >= o.yHi) continue;
            const int u = std::clamp(int((X - o.x0) * o.kx), 0, int(o.w) - 1);
            const int v = std::clamp(int((o.down ? o.yHi - Y : Y - o.yLo) * o.ky), 0, int(o.h) - 1);
            return Absent(j, u, v, wc);
        }
        return true;
    }
    const uint64_t key = (uint64_t(file) << 48) | (uint64_t(y / kWin) << 24) | uint64_t(x / kWin);
    if (key != wc.lastKey) {
        auto it = wc.held.find(key);
        if (it == wc.held.end()) it = wc.held.emplace(key, Window(file, uint32_t(x / kWin), uint32_t(y / kWin))).first;
        wc.lastKey = key;
        wc.last = it->second.get();
    }
    const uint8_t* p = &(*wc.last)[(size_t(y % kWin) * kWin + x % kWin) * 4];
    if (!m_height) return p[3] == 0;
    float t;
    std::memcpy(&t, p, 4);
    return std::isnan(t);
}

// A texel's edge factor: 1 where the feather does not reach absence, else its window's (held
// beside the thread's recent windows, as Texel holds those).
float RasterFileSource::EdgeAt(size_t file, int x, int y) {
    if (m_feather <= 0.0 || !m_absence) return 1.0f;
    const uint64_t key = (uint64_t(file) << 48) | (uint64_t(y / kWin) << 24) | uint64_t(x / kWin);
    const size_t at = size_t(y % kWin) * kWin + x % kWin;
    for (const RecentEdge& r : tRecentEdge) {
        if (r.owner == m_serial && r.key == key) {
            const Edge* e = static_cast<const Edge*>(r.e.get());
            return e->f.empty() ? 1.0f : e->f[at];
        }
    }
    RecentEdge& r = tRecentEdge[tNextEdge++ & 3u];
    const auto e = WindowEdge(file, uint32_t(x / kWin), uint32_t(y / kWin));
    r.e = e;
    r.owner = m_serial;
    r.key = key;
    return e->f.empty() ? 1.0f : e->f[at];
}

// THE RANGE OVER A BOX. Sample's value is an area mean of the texels its box touches, times the scale
// plus the offset, so it lies in the range of those texels; the box asked for is taken to the file's
// texels through the CRS (its corners and edges, a margin for the projection's bend, two texels for
// the sample's own box at the source's grain -- a coarser box is the caller's to add) and read in blocks.
HeightSource::Range RasterFileSource::RangeIn(double lat0, double lon0, double lat1, double lon1, float& lo,
                                              float& hi, bool& full) {
    full = false;
    if (!m_height || m_files.empty()) return Range::None;
    double xa = 1e300, xb = -1e300, ya = 1e300, yb = -1e300;
    for (int j = 0; j <= 4; ++j) {
        for (int i = 0; i <= 4; ++i) {
            double x, y;
            if (!ToCrs((lat0 + (lat1 - lat0) * j / 4.0) * kD2R, (lon0 + (lon1 - lon0) * i / 4.0) * kD2R, x, y)) {
                return Range::Unknown;
            }
            xa = (std::min)(xa, x); xb = (std::max)(xb, x);
            ya = (std::min)(ya, y); yb = (std::max)(yb, y);
        }
    }
    const double bx = 0.01 * (xb - xa), by = 0.01 * (yb - ya);
    xa -= bx; xb += bx; ya -= by; yb += by;
    if (xb <= m_ux0 || xa >= m_ux1 || yb <= m_uy0 || ya >= m_uy1) return Range::None;
    const bool insideWhole = xa > m_ux0 && xb < m_ux1 && ya > m_uy0 && yb < m_uy1;
    bool any = false, nan = false;
    int files = 0;
    double tlo = 0.0, thi = 0.0;
    for (size_t fi = 0; fi < m_files.size(); ++fi) {
        const File& f = *m_files[fi];
        if (xb < f.x0 || xa >= f.x1 || yb < f.yLo || ya >= f.yHi) continue;
        ++files;
        const double ua = (xa - f.x0) * f.kx - 2.0, ub = (xb - f.x0) * f.kx + 2.0;
        const double va = (f.down ? f.yHi - yb : ya - f.yLo) * f.ky - 2.0;
        const double vb = (f.down ? f.yHi - ya : yb - f.yLo) * f.ky + 2.0;
        if (ua < 0.0 || va < 0.0 || ub > double(f.w) - 1.0 || vb > double(f.h) - 1.0) nan = true;   // the edge
        const int u0 = (std::max)(0, int(std::floor(ua))), u1 = (std::min)(int(f.w) - 1, int(std::floor(ub)));
        const int v0 = (std::max)(0, int(std::floor(va))), v1 = (std::min)(int(f.h) - 1, int(std::floor(vb)));
        if (u0 > u1 || v0 > v1) continue;
        // a survey finer than the box is worth: past ~16k blocks (a 2k x 2k texel box) the march is the
        // cheaper answer, and near such a survey there is ground above the water anyway
        if (double(v1 / 16 - v0 / 16 + 1) * double(u1 / 16 - u0 / 16 + 1) > 16384.0) return Range::Unknown;
        for (int by_ = v0 / 16; by_ <= v1 / 16; ++by_) {
            for (int bx_ = u0 / 16; bx_ <= u1 / 16; ++bx_) {
                const auto b = WindowBlocks(fi, uint32_t(bx_ * 16 / int(kWin)), uint32_t(by_ * 16 / int(kWin)));
                const int k = (by_ % int(kWin / 16)) * 16 + (bx_ % int(kWin / 16));
                if (b->nan[k]) nan = true;
                if (!(b->mn[k] <= b->mx[k])) continue;   // a block of NaN alone
                tlo = any ? (std::min)(tlo, double(b->mn[k])) : double(b->mn[k]);
                thi = any ? (std::max)(thi, double(b->mx[k])) : double(b->mx[k]);
                any = true;
            }
        }
    }
    if (!any) return Range::None;
    const double a = tlo * m_vScale + m_vOffset, c = thi * m_vScale + m_vOffset;
    lo = float((std::min)(a, c));
    hi = float((std::max)(a, c));
    full = files == 1 && !nan && insideWhole && Feathered(xa, ya) >= 1.0f && Feathered(xa, yb) >= 1.0f &&
           Feathered(xb, ya) >= 1.0f && Feathered(xb, yb) >= 1.0f;
    return Range::Known;
}

void RasterFileSource::BoxIntegral(size_t file, double u0, double u1, double v0, double v1, double& value,
                                   double& have) {
    value = have = 0.0;
    const size_t N = kWin + 1;
    // The table's bilinear at a fractional corner (a, b) of the window, 0..kWin.
    const auto at = [N](const std::vector<double>& t, double a, double b) {
        const int i = (std::min)(static_cast<int>(a), int(kWin) - 1), j = (std::min)(static_cast<int>(b), int(kWin) - 1);
        const double s = a - i, r = b - j;
        const double t00 = t[j * N + i], t10 = t[j * N + i + 1], t01 = t[(j + 1) * N + i], t11 = t[(j + 1) * N + i + 1];
        return t00 * (1 - s) * (1 - r) + t10 * s * (1 - r) + t01 * (1 - s) * r + t11 * s * r;
    };
    const uint32_t wx0 = uint32_t(u0 / kWin), wx1 = uint32_t((std::max)(u0, u1 - 1e-9) / kWin);
    const uint32_t wy0 = uint32_t(v0 / kWin), wy1 = uint32_t((std::max)(v0, v1 - 1e-9) / kWin);
    for (uint32_t wy = wy0; wy <= wy1; ++wy) {
        const double b0 = std::clamp(v0 - double(wy) * kWin, 0.0, double(kWin));
        const double b1 = std::clamp(v1 - double(wy) * kWin, 0.0, double(kWin));
        for (uint32_t wx = wx0; wx <= wx1; ++wx) {
            const double a0 = std::clamp(u0 - double(wx) * kWin, 0.0, double(kWin));
            const double a1 = std::clamp(u1 - double(wx) * kWin, 0.0, double(kWin));
            if (a1 <= a0 || b1 <= b0) continue;
            const auto s = WindowSat(file, wx, wy);
            value += at(s->v, a1, b1) - at(s->v, a0, b1) - at(s->v, a1, b0) + at(s->v, a0, b0);
            have += at(s->n, a1, b1) - at(s->n, a0, b1) - at(s->n, a1, b0) + at(s->n, a0, b0);
        }
    }
}

const uint8_t* RasterFileSource::Texel(size_t file, int x, int y) {
    const uint64_t key = (uint64_t(file) << 48) | (uint64_t(y / kWin) << 24) | uint64_t(x / kWin);
    const size_t at = (size_t(y % kWin) * kWin + x % kWin) * 4;
    for (const Recent& r : tRecent) {
        if (r.owner == m_serial && r.key == key) return &(*r.px)[at];
    }
    Recent& r = tRecent[tNext++ & 3u];
    r.px = Window(file, uint32_t(x / kWin), uint32_t(y / kWin));
    r.owner = m_serial;
    r.key = key;
    return &(*r.px)[at];
}

// A box of one texel: the bilinear read between texel centres, taps clamped at the file's edge,
// in the plane orthos' own arithmetic. A tap's alpha is the file's times its edge factor (1 where
// the feather reaches no absence, so the bytes are what they were). Where the four are one and the
// same, the alpha cancels and this is their plain bilinear; else the colour is premultiplied.
// Returns the alpha, 0..1.
double RasterFileSource::Bilinear(size_t file, double fx, double fy, uint8_t rgba[4]) {
    const File& f = *m_files[file];
    const int W = int(f.w), H = int(f.h);
    const int x0 = std::clamp(static_cast<int>(std::floor(fx)), 0, (std::max)(0, W - 2));
    const int y0 = std::clamp(static_cast<int>(std::floor(fy)), 0, (std::max)(0, H - 2));
    const double tx = std::clamp(fx - x0, 0.0, 1.0), ty = std::clamp(fy - y0, 0.0, 1.0);
    const int x1 = (std::min)(x0 + 1, W - 1), y1 = (std::min)(y0 + 1, H - 1);
    uint8_t q[4][4];
    std::memcpy(q[0], Texel(file, x0, y0), 4);
    std::memcpy(q[1], Texel(file, x1, y0), 4);
    std::memcpy(q[2], Texel(file, x0, y1), 4);
    std::memcpy(q[3], Texel(file, x1, y1), 4);
    const double al[4] = {q[0][3] * EdgeAt(file, x0, y0), q[1][3] * EdgeAt(file, x1, y0),
                          q[2][3] * EdgeAt(file, x0, y1), q[3][3] * EdgeAt(file, x1, y1)};
    if (al[0] == al[1] && al[0] == al[2] && al[0] == al[3]) {
        for (int c = 0; c < 3; ++c) {
            const double a = q[0][c] * (1 - tx) + q[1][c] * tx;
            const double b = q[2][c] * (1 - tx) + q[3][c] * tx;
            rgba[c] = static_cast<uint8_t>(a * (1 - ty) + b * ty + 0.5);
        }
        return al[0] / 255.0;
    }
    const double w[4] = {(1 - tx) * (1 - ty), tx * (1 - ty), (1 - tx) * ty, tx * ty};
    double acc[3] = {0.0, 0.0, 0.0}, a = 0.0;
    for (int k = 0; k < 4; ++k) {
        const double wa = w[k] * al[k];
        for (int c = 0; c < 3; ++c) acc[c] += wa * q[k][c];
        a += wa;
    }
    if (a <= 0.0) return 0.0;
    for (int c = 0; c < 3; ++c) rgba[c] = static_cast<uint8_t>(acc[c] / a + 0.5);
    return a / 255.0;
}

// A larger box (su x sv texels about (uc, vc), texel i covering [i, i + 1)): the area mean, each
// texel weighted by its part inside the box and by its alpha and edge factor, the box clipped to
// the file. It is the same law as the bilinear: a box one texel wide is the linear read between
// centres.
double RasterFileSource::BoxMean(size_t file, double uc, double vc, double su, double sv,
                                 uint8_t rgba[4]) {
    const File& f = *m_files[file];
    const double u0 = (std::max)(0.0, uc - 0.5 * su), u1 = (std::min)(double(f.w), uc + 0.5 * su);
    const double v0 = (std::max)(0.0, vc - 0.5 * sv), v1 = (std::min)(double(f.h), vc + 0.5 * sv);
    double acc[3] = {0.0, 0.0, 0.0}, a = 0.0, area = 0.0;
    for (int j = static_cast<int>(v0); j < v1; ++j) {
        const double wy = (std::min)(v1, j + 1.0) - (std::max)(v0, double(j));
        for (int i = static_cast<int>(u0); i < u1; ++i) {
            const double wxy = ((std::min)(u1, i + 1.0) - (std::max)(u0, double(i))) * wy;
            const uint8_t* p = Texel(file, i, j);
            const double wa = wxy * (p[3] * EdgeAt(file, i, j));
            for (int c = 0; c < 3; ++c) acc[c] += wa * p[c];
            a += wa;
            area += wxy;
        }
    }
    if (a <= 0.0 || area <= 0.0) return 0.0;
    for (int c = 0; c < 3; ++c) rgba[c] = static_cast<uint8_t>(acc[c] / a + 0.5);
    return a / (255.0 * area);
}

float RasterFileSource::Sample(double latRad, double lonRad, double groundResM, const PaintCtx&,
                               uint8_t rgba[4]) {
    double x = 0.0, y = 0.0;
    if (m_files.empty() || !ToCrs(latRad, lonRad, x, y)) return 0.0f;
    if (x <= m_ux0 || x >= m_ux1 || y <= m_uy0 || y >= m_uy1) return 0.0f;   // outside the whole
    for (size_t i = 0; i < m_files.size(); ++i) {
        const File& f = *m_files[i];
        if (x < f.x0 || x >= f.x1 || y < f.yLo || y >= f.yHi) continue;
        // Texels from the outer corner of texel (0, 0), a sample at a texel's centre at integers.
        const double fx = (x - f.x0) * f.kx - 0.5;
        const double fy = (f.down ? f.yHi - y : y - f.yLo) * f.ky - 0.5;
        // THE BOX: the ground asked for (groundResM is Mercator-equatorial; x cos(lat) is metres
        // on the ground, the aerial source's rule), in this file's texels, never under one.
        const double s = groundResM * std::cos(latRad), su = s / m_gx, sv = s / m_gy;
        const double alpha = (su <= 1.0 && sv <= 1.0)
                                 ? Bilinear(i, fx, fy, rgba)
                                 : BoxMean(i, fx + 0.5, fy + 0.5, (std::max)(1.0, su),
                                           (std::max)(1.0, sv), rgba);
        if (alpha <= 0.0) return 0.0f;
        rgba[3] = 255;
        return Feathered(x, y) * static_cast<float>(alpha);
    }
    return 0.0f;
}

// The entry's feather against the whole's box, the orthos' 25 m smoothstep; 1 with none.
float RasterFileSource::Feathered(double x, double y) const {
    if (m_feather <= 0.0) return 1.0f;
    const double dm = (std::min)((std::min)(x - m_ux0, m_ux1 - x) * m_mx,
                                 (std::min)(y - m_uy0, m_uy1 - y) * m_my);
    const double t = std::clamp(dm / m_feather, 0.0, 1.0);
    return static_cast<float>(t * t * (3.0 - 2.0 * t));
}

// A HEIGHT: the colour's area mean in one channel -- a texel weighs its edge factor where the file
// has a number (1 away from every absence) and 0 where it said nodata (NaN in the window) -- over
// the box asked for, never under one texel, so a box of one texel is the bilinear between centres,
// taps clamped at the file's edge. The weight is the weighed part of the box the file has, times
// the box's feather; the number is in kHeightFrame.
float RasterFileSource::Sample(double latRad, double lonRad, double groundResM, float& metres) {
    double x = 0.0, y = 0.0;
    if (!m_height || m_files.empty() || !ToCrs(latRad, lonRad, x, y)) return 0.0f;
    if (x <= m_ux0 || x >= m_ux1 || y <= m_uy0 || y >= m_uy1) return 0.0f;   // outside the whole
    for (size_t i = 0; i < m_files.size(); ++i) {
        const File& f = *m_files[i];
        if (x < f.x0 || x >= f.x1 || y < f.yLo || y >= f.yHi) continue;
        const double s = groundResM * std::cos(latRad);
        const double su = (std::max)(1.0, s / m_gx), sv = (std::max)(1.0, s / m_gy);
        const double uc = (x - f.x0) * f.kx, vc = (f.down ? f.yHi - y : y - f.yLo) * f.ky;
        const double u0 = (std::max)(0.0, uc - 0.5 * su), u1 = (std::min)(double(f.w), uc + 0.5 * su);
        const double v0 = (std::max)(0.0, vc - 0.5 * sv), v1 = (std::min)(double(f.h), vc + 0.5 * sv);
        double acc = 0.0, have = 0.0, area = 0.0;
        if (su > 2.0 || sv > 2.0) {   // past two texels a side, by the integral (WindowSat)
            BoxIntegral(i, u0, u1, v0, v1, acc, have);
            area = (u1 - u0) * (v1 - v0);
            if (have <= 1e-9 || area <= 0.0) return 0.0f;
            metres = static_cast<float>(acc / have * m_vScale + m_vOffset);
            return Feathered(x, y) * static_cast<float>((std::min)(1.0, have / area));
        }
        for (int j = static_cast<int>(v0); j < v1; ++j) {
            const double wy = (std::min)(v1, j + 1.0) - (std::max)(v0, double(j));
            for (int k = static_cast<int>(u0); k < u1; ++k) {
                const double w = ((std::min)(u1, k + 1.0) - (std::max)(u0, double(k))) * wy;
                float t;
                std::memcpy(&t, Texel(i, k, j), 4);
                area += w;
                if (std::isnan(t)) continue;
                const double we = w * EdgeAt(i, k, j);
                acc += we * t;
                have += we;
            }
        }
        if (have <= 0.0 || area <= 0.0) return 0.0f;
        metres = static_cast<float>(acc / have * m_vScale + m_vOffset);
        return Feathered(x, y) * static_cast<float>(have / area);
    }
    return 0.0f;
}

std::vector<std::unique_ptr<RasterFileSource>> LoadRasterSources(
    const std::vector<RasterEntry>& entries) {
    LoaderRegistry reg;
    RegisterImageLoaders(reg);
    std::vector<std::unique_ptr<RasterFileSource>> out;
    for (const RasterEntry& e : entries) {
        const std::string label = !e.manifest.empty() ? e.manifest : !e.file.empty() ? e.file : e.folder;
        std::vector<std::unique_ptr<FieldLoader>> files;
        std::string why;
        if (!e.manifest.empty()) {
            files = OpenManifest(e.manifest, &why);
        } else if (!e.file.empty()) {
            if (auto L = OpenOne(reg, e.file, e, &why)) files.push_back(std::move(L));
        } else {
            std::vector<std::string> paths;
            std::error_code ec;
            for (auto it = std::filesystem::directory_iterator(e.folder, ec);
                 !ec && it != std::filesystem::directory_iterator(); it.increment(ec)) {
                const std::string n = it->path().filename().string();
                if (it->is_regular_file(ec) && Matches(e.match.empty() ? "*" : e.match.c_str(), n.c_str())) {
                    paths.push_back(it->path().generic_string());
                }
            }
            std::sort(paths.begin(), paths.end());   // by name, not by disk
            why = "'" + e.match + "' matches no file that loads";
            for (const std::string& p : paths) {
                std::string w;
                if (auto L = OpenOne(reg, p, e, &w)) files.push_back(std::move(L));
                else Log("[raster] %s REFUSED: %s -- the set goes on without it", p.c_str(), w.c_str());
            }
        }
        auto s = std::make_unique<RasterFileSource>();
        if (!files.empty() && s->Open(std::move(files), e, label, &why)) {
            out.push_back(std::move(s));
        } else {
            Log("[raster] %s REFUSED: %s -- the run goes on without it", label.c_str(), why.c_str());
        }
    }
    return out;
}

// ---- THE PASS ---------------------------------------------------------------------------------
IngestStats Ingest(TileTree& tree, const Lattice& lattice, const std::vector<Lattice>& realized) {
    IngestStats st;
    const DomainSource* node = tree.Node();
    double lo0 = 0, la0 = 0, lo1 = 0, la1 = 0;
    if (!tree.Fits(lattice)) {   // a tile of this frame is not this tree's tile (TileTree::Fits)
        Log("[ingest] %s on %s: REFUSED -- a %ux%u tile is not this tree's (%zu bytes)", node->Name(),
            lattice.Tag().c_str(), lattice.texW, lattice.texH, tree.TileBytes());
        return st;
    }
    st.own = node->OwnMip(lattice);
    if (st.own < 0 || !node->Footprint(lo0, la0, lo1, la1)) return st;
    const std::string tag = lattice.Tag();
    tree.EnsureFrame(tag);
    const std::string marker = tree.Folder() + "\\" + tag + "\\.whole";
    if (tree_detail::Exists(marker)) {
        st.whole = true;
        return st;
    }
    // THE TILES, per level, from the lattice's coarsest down to the own mip: every tile whose cap
    // meets the footprint's is descended into, and those the compositor's rule counts in are the
    // level's. A cap (a centre and the angle to the farthest corner) and not the lat/lon box: a
    // cube tile across the dateline gets a box of every longitude from atan2, the compositor's
    // own caution, and the descent would walk the dateline at every level.
    auto dir = [](double la, double lo, double d[3]) {
        d[0] = std::cos(la) * std::cos(lo);
        d[1] = std::sin(la);
        d[2] = std::cos(la) * std::sin(lo);
    };
    auto angle = [](const double a[3], const double b[3]) {
        const double c[3] = {a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]};
        return std::atan2(std::sqrt(c[0] * c[0] + c[1] * c[1] + c[2] * c[2]), a[0] * b[0] + a[1] * b[1] + a[2] * b[2]);
    };
    double fc[3], fr = 0.0;
    dir(0.5 * (la0 + la1) * kD2R, 0.5 * (lo0 + lo1) * kD2R, fc);
    for (int k = 0; k < 4; ++k) {
        double p[3];
        dir(((k & 1) ? la1 : la0) * kD2R, ((k & 2) ? lo1 : lo0) * kD2R, p);
        fr = (std::max)(fr, angle(fc, p));
    }
    auto meets = [&](const TileRequest& r) {
        double c[3], p[3], la = 0.0, lo = 0.0, rr = 0.0;
        lattice.Texel(r, lattice.texW / 2, lattice.texH / 2, la, lo);
        dir(la, lo, c);
        for (int k = 0; k < 4; ++k) {   // corner texels' centres, and a texel's margin below
            lattice.Texel(r, (k & 1) ? lattice.texW - 1 : 0, (k & 2) ? lattice.texH - 1 : 0, la, lo);
            dir(la, lo, p);
            rr = (std::max)(rr, angle(c, p));
        }
        return angle(c, fc) <= fr + rr * (1.0 + 4.0 / lattice.texW);
    };
    const uint32_t top = lattice.MaxMip();
    std::vector<std::vector<TileRequest>> in(top + 1);
    std::vector<TileRequest> frontier;
    // A level's tiles a side, by columns and by rows apart: a 256 x 128 tile (a height's) makes a face
    // twice as many rows as columns, and the top level two tiles, one above the other. (Until
    // 2026-10-21 both were the columns' count and the top one tile: no height's pass reached the
    // lower half of a face, which the readers then painted cold.)
    auto across = [&](int m) { return (std::max)(1u, (lattice.faceDim >> m) / lattice.texW); };
    auto down = [&](int m) { return (std::max)(1u, (lattice.faceDim >> m) / lattice.texH); };
    for (uint32_t f = 0; f < (lattice.kind == Lattice::Kind::Cube ? 6u : 1u); ++f) {
        for (uint32_t y = 0; y < down(int(top)); ++y) frontier.push_back({f, top, 0, y});
    }
    for (int m = int(top); m >= st.own && !frontier.empty(); --m) {
        std::vector<TileRequest> next;
        for (const TileRequest& r : frontier) {
            if (r.x >= across(m) || r.y >= down(m) || !meets(r)) continue;
            TileBox b{};
            lattice.Box(r, b);
            if (Compositor::Touches(lo0, la0, lo1, la1, b)) in[m].push_back(r);
            for (uint32_t k = 0; m > st.own && k < 4; ++k) {
                next.push_back({r.face, r.mip - 1, r.x * 2 + (k & 1), r.y * 2 + (k >> 1)});
            }
        }
        frontier.swap(next);
    }
    size_t total = 0;
    for (const auto& v : in) total += v.size();
    // THE PARTNER LEVEL: a level whose texel grid a finer realization of this tree already holds is
    // taken from it, tile for tile (TileTree::Adopt), and neither painted nor folded here.
    std::vector<int> partnerMip(top + 1, -1), partnerOf(top + 1, -1);
    for (uint32_t m = 0; m <= top; ++m) {
        for (size_t i = 0; i < realized.size() && partnerMip[m] < 0; ++i) {
            partnerMip[m] = TileTree::PartnerMip(lattice, m, realized[i]);
            if (partnerMip[m] >= 0) partnerOf[m] = int(i);
        }
    }
    // Painted at the own mip, then each level folded from the one below, on the pool.
    const auto t0 = std::chrono::steady_clock::now();
    const uint64_t dec0 = RasterFileSource::s_decodes.load(), us0 = RasterFileSource::s_decodeUs.load();
    const uint64_t edge0 = RasterFileSource::s_edges.load();
    auto since = [&] {
        return std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
    };
    std::atomic<uint32_t> done{0}, adopted{0};
    std::atomic<bool> transient{false};
    std::atomic<long long> nextLog{5};
    for (uint32_t m = uint32_t(st.own); m <= top; ++m) {
        const std::vector<TileRequest>& lv = in[m];
        if (lv.empty()) continue;
        ++st.levels;
        const int pi = partnerOf[m];
        const std::string ptag = pi >= 0 ? realized[pi].Tag() : std::string();
        Threads().ParallelFor(Lane::Compute, "ingest", int(lv.size()), 4, [&](int a, int b) {
            std::vector<uint8_t> bytes;
            for (int k = a; k < b; ++k) {
                const TileTree::Status s =
                    pi >= 0 ? tree.Adopt(tag, lv[k], realized[pi], ptag, uint32_t(partnerMip[m]), bytes)
                            : tree.Tile(lattice, tag, lv[k], bytes, nullptr);
                if (s == TileTree::Status::Transient) transient = true;
                if (pi >= 0) ++adopted;
                const uint32_t d = ++done;
                long long due = nextLog.load();
                if (since() >= double(due) && nextLog.compare_exchange_strong(due, due + 5)) {
                    Log("[ingest] %s on %s: mip %u, %u of %zu tiles, %.0f s; %llu windows decoded, %llu data edges built",
                        node->Name(), tag.c_str(), m, d, total, since(),
                        static_cast<unsigned long long>(RasterFileSource::s_decodes.load() - dec0),
                        static_cast<unsigned long long>(RasterFileSource::s_edges.load() - edge0));
                }
            }
        });
    }
    st.tiles = done.load();
    st.adopted = adopted.load();
    st.seconds = since();
    st.windows = RasterFileSource::s_decodes.load() - dec0;
    st.decodeSec = double(RasterFileSource::s_decodeUs.load() - us0) * 1e-6;
    if (!transient) {
        char b[256];
        snprintf(b, sizeof(b), "{\"tree\": \"%s\", \"own\": %d, \"tiles\": %u, \"levels\": %u}\n",
                 tree.Id().c_str(), st.own, st.tiles, st.levels);
        const std::string s = b;
        tree_detail::WriteTile(marker, std::vector<uint8_t>(s.begin(), s.end()), false);
    }
    return st;
}

void IngestSources(const std::vector<std::unique_ptr<RasterFileSource>>& sources,
                   const std::vector<Lattice>& colour, const std::vector<Lattice>& height) {
    for (const auto& src : sources) {
        // The leaf folder the stack's tree keeps for it: the megatexture's (a colour, normalized),
        // or the height tree's (FloatW, in the frame).
        std::shared_ptr<DomainSource> layer, leaf;
        if (src->Height()) layer = std::make_shared<HeightLayerSource>(src.get(), kHeightFrame);
        else leaf = NormalizeToSi(layer = std::make_shared<ColorLayerSource>(src.get()));
        TileTree tree(leaf ? leaf.get() : layer.get(), src->Height() ? TileTree::Fmt::FloatW : TileTree::Fmt::Rgba8);
        // The finest lattice first: a coarser one takes its equal-grid levels from the folds the
        // finer one kept (TileTree::Adopt) instead of asking the file for them.
        std::vector<Lattice> order = src->Height() ? height : colour;
        std::stable_sort(order.begin(), order.end(),
                         [](const Lattice& a, const Lattice& b) { return a.faceDim > b.faceDim; });
        std::vector<Lattice> realized;
        for (const Lattice& l : order) {
            const IngestStats st = Ingest(tree, l, realized);
            realized.push_back(l);
            if (st.whole) {
                Log("[ingest] %s on %s: whole (own mip %d)", src->Info().name.c_str(), l.Tag().c_str(), st.own);
            } else if (st.own >= 0) {
                Log("[ingest] %s on %s: own mip %d, %u tiles on %u levels in %.1f s (%u taken from a finer "
                    "realization); %llu windows decoded, %.1f s in the decoder",
                    src->Info().name.c_str(), l.Tag().c_str(), st.own, st.tiles, st.levels, st.seconds,
                    st.adopted, static_cast<unsigned long long>(st.windows), st.decodeSec);
            }
        }
    }
}

}  // namespace ga
