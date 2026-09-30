#include "compose/RasterFileSource.h"

#include "compose/ColorStackSource.h"
#include "compose/DomainSource.h"
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

namespace {

constexpr double kPi = 3.14159265358979323846, kD2R = kPi / 180.0;
constexpr double kR = 6378137.0;   // WGS84 a: Web Mercator's sphere, and a degree's metres of arc
constexpr uint32_t kWin = 256;     // a decoded window's side, texels
constexpr size_t kWindows = 256;   // the windows a source keeps: 64 MB

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
    } else if (L->Channels() != 4) {
        Fail(why, "its loader gives no colour (RGBA)");
        L.reset();
    }
    return L;
}

// The windows a painting thread used last: a texel read is then no lock, and four hold every
// window one bilinear read can touch. Keyed by the source's serial, never its address.
struct Recent {
    uint64_t owner = 0, key = ~0ull;
    std::shared_ptr<const std::vector<uint8_t>> px;
};
thread_local Recent tRecent[4];
thread_local unsigned tNext = 0;
std::atomic<uint64_t> gSerial{0};

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
    if (!Placed(why)) return false;
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
        snprintf(b, sizeof(b), ", feather %g m", m_feather);
        m_info.structure += b;
    }
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
    return true;
}

int RasterFileSource::OwnMip(const Lattice& lattice) const {
    if (m_files.empty()) return -1;
    const double lat = 0.5 * (m_info.lat0 + m_info.lat1) * kD2R;
    const double lon = 0.5 * (m_info.lon0 + m_info.lon1) * kD2R;
    int own = 0;
    for (uint32_t m = 0; m <= lattice.MaxMip(); ++m) {
        double a = 0.0, b = 0.0;
        lattice.TexelGround(m, lat, lon, a, b);
        if ((std::max)(a, b) > m_grainM * (1.0 + 1e-9)) break;
        own = static_cast<int>(m);
    }
    return own;
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
// file said nodata), decoded once and kept while used. Two threads may decode one window at once;
// the second keeps the first's, and the loader serializes its own decoder.
std::shared_ptr<const std::vector<uint8_t>> RasterFileSource::Window(size_t file, uint32_t wx,
                                                                     uint32_t wy) {
    const uint64_t key = (uint64_t(file) << 48) | (uint64_t(wy) << 24) | wx;
    {
        std::lock_guard<std::mutex> lk(m_cacheMx);
        auto it = m_index.find(key);
        if (it != m_index.end()) {
            m_lru.splice(m_lru.begin(), m_lru, it->second);
            return it->second->second;
        }
    }
    auto px = std::make_shared<std::vector<uint8_t>>(size_t(kWin) * kWin * 4, uint8_t(0));
    TilePayload tp;
    if (m_files[file]->loader->LoadTile(wx, wy, kWin, kWin, tp)) {
        for (size_t k = 0; k < px->size(); ++k) (*px)[k] = static_cast<uint8_t>(tp.data[k]);
    }
    std::lock_guard<std::mutex> lk(m_cacheMx);
    auto it = m_index.find(key);
    if (it != m_index.end()) return it->second->second;
    m_lru.emplace_front(key, px);
    m_index[key] = m_lru.begin();
    if (m_lru.size() > kWindows) {
        m_index.erase(m_lru.back().first);
        m_lru.pop_back();
    }
    return m_lru.front().second;
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
// in the plane orthos' own arithmetic. Where the four alphas are one, the alpha cancels and this
// is their plain bilinear; else the colour is premultiplied. Returns the alpha, 0..1.
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
    if (q[0][3] == q[1][3] && q[0][3] == q[2][3] && q[0][3] == q[3][3]) {
        for (int c = 0; c < 3; ++c) {
            const double a = q[0][c] * (1 - tx) + q[1][c] * tx;
            const double b = q[2][c] * (1 - tx) + q[3][c] * tx;
            rgba[c] = static_cast<uint8_t>(a * (1 - ty) + b * ty + 0.5);
        }
        return q[0][3] / 255.0;
    }
    const double w[4] = {(1 - tx) * (1 - ty), tx * (1 - ty), (1 - tx) * ty, tx * ty};
    double acc[3] = {0.0, 0.0, 0.0}, a = 0.0;
    for (int k = 0; k < 4; ++k) {
        const double wa = w[k] * q[k][3];
        for (int c = 0; c < 3; ++c) acc[c] += wa * q[k][c];
        a += wa;
    }
    if (a <= 0.0) return 0.0;
    for (int c = 0; c < 3; ++c) rgba[c] = static_cast<uint8_t>(acc[c] / a + 0.5);
    return a / 255.0;
}

// A larger box (su x sv texels about (uc, vc), texel i covering [i, i + 1)): the area mean, each
// texel weighted by its part inside the box and by its alpha, the box clipped to the file. It is
// the same law as the bilinear: a box one texel wide is the linear read between centres.
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
            const double wa = wxy * p[3];
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
        float w = 1.0f;
        if (m_feather > 0.0) {   // against the whole's box, the orthos' 25 m smoothstep
            const double dm = (std::min)((std::min)(x - m_ux0, m_ux1 - x) * m_mx,
                                         (std::min)(y - m_uy0, m_uy1 - y) * m_my);
            const double t = std::clamp(dm / m_feather, 0.0, 1.0);
            w = static_cast<float>(t * t * (3.0 - 2.0 * t));
        }
        return w * static_cast<float>(alpha);
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

void StackOrder(std::vector<ColorSource*>& stack) {
    auto over = [](const ColorSource* s) {
        const auto* r = dynamic_cast<const RasterFileSource*>(s);
        return r ? r->Over() : 0.0;
    };
    std::stable_sort(stack.begin(), stack.end(), [&](const ColorSource* a, const ColorSource* b) {
        if (over(a) != over(b)) return over(a) < over(b);
        return a->Info().cmPerPixel > b->Info().cmPerPixel;   // the coarser under the finer
    });
}

// ---- THE PASS ---------------------------------------------------------------------------------
IngestStats Ingest(TileTree& tree, const Lattice& lattice) {
    IngestStats st;
    const DomainSource* node = tree.Node();
    double lo0 = 0, la0 = 0, lo1 = 0, la1 = 0;
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
    for (uint32_t f = 0; f < (lattice.kind == Lattice::Kind::Cube ? 6u : 1u); ++f) {
        frontier.push_back({f, top, 0, 0});
    }
    for (int m = int(top); m >= st.own && !frontier.empty(); --m) {
        const uint32_t n = (std::max)(1u, (lattice.faceDim >> m) / lattice.texW);
        std::vector<TileRequest> next;
        for (const TileRequest& r : frontier) {
            if (r.x >= n || r.y >= n || !meets(r)) continue;
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
    // Painted at the own mip, then each level folded from the one below, on the pool.
    const auto t0 = std::chrono::steady_clock::now();
    auto since = [&] {
        return std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
    };
    std::atomic<uint32_t> done{0};
    std::atomic<bool> transient{false};
    std::atomic<long long> nextLog{5};
    for (uint32_t m = uint32_t(st.own); m <= top; ++m) {
        const std::vector<TileRequest>& lv = in[m];
        if (lv.empty()) continue;
        ++st.levels;
        Threads().ParallelFor(Lane::Compute, "ingest", int(lv.size()), 4, [&](int a, int b) {
            std::vector<uint8_t> bytes;
            for (int k = a; k < b; ++k) {
                if (tree.Tile(lattice, tag, lv[k], bytes, nullptr) == TileTree::Status::Transient) {
                    transient = true;
                }
                const uint32_t d = ++done;
                long long due = nextLog.load();
                if (since() >= double(due) && nextLog.compare_exchange_strong(due, due + 5)) {
                    Log("[ingest] %s on %s: mip %u, %u of %zu tiles, %.0f s", node->Name(),
                        tag.c_str(), m, d, total, since());
                }
            }
        });
    }
    st.tiles = done.load();
    st.seconds = since();
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
                   const std::vector<Lattice>& lattices) {
    for (const auto& src : sources) {
        auto layer = std::make_shared<ColorLayerSource>(src.get());
        const std::shared_ptr<DomainSource> leaf = NormalizeToSi(layer);
        TileTree tree(leaf ? leaf.get() : layer.get());   // the megatexture's leaf folder
        for (const Lattice& l : lattices) {
            const IngestStats st = Ingest(tree, l);
            if (st.whole) {
                Log("[ingest] %s on %s: whole (own mip %d)", src->Info().name.c_str(), l.Tag().c_str(), st.own);
            } else if (st.own >= 0) {
                Log("[ingest] %s on %s: own mip %d, %u tiles on %u levels in %.1f s", src->Info().name.c_str(),
                    l.Tag().c_str(), st.own, st.tiles, st.levels, st.seconds);
            }
        }
    }
}

}  // namespace ga
