#include "compose/RasterFileSource.h"

#include "core/Common.h"
#include "core/ImageLoader.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <filesystem>

namespace ga {

namespace {

constexpr double kPi = 3.14159265358979323846, kD2R = kPi / 180.0;
constexpr double kR = 6378137.0;   // WGS84 a: Web Mercator's sphere, and a degree's metres of arc

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

// A loader's texels as RGBA8, pulled through the seam's one door (LoadTile).
void Pull(FieldLoader& L, std::vector<uint8_t>& rgba) {
    const GeoRef& g = L.Ref();
    const uint32_t T = 256;
    rgba.assign(size_t(g.width) * g.height * 4, 0);
    TilePayload tp;
    for (uint32_t ty = 0; ty * T < g.height; ++ty) {
        for (uint32_t tx = 0; tx * T < g.width; ++tx) {
            if (!L.LoadTile(tx, ty, T, T, tp)) continue;   // all absent: alpha 0 already
            for (uint32_t r = 0; r < T && ty * T + r < g.height; ++r) {
                for (uint32_t c = 0; c < T && tx * T + c < g.width; ++c) {
                    uint8_t* d = &rgba[(size_t(ty * T + r) * g.width + tx * T + c) * 4];
                    const float* s = &tp.data[(size_t(r) * T + c) * 4];
                    for (int k = 0; k < 4; ++k) d[k] = static_cast<uint8_t>(s[k]);
                }
            }
        }
    }
}

}  // namespace

bool RasterFileSource::Load(const LoaderRegistry& reg, const std::string& path,
                            const RasterEntry& e, std::string* why) {
    GeoRef said;   // the entry's word, for a file that has none of its own
    if (!e.crs.empty() && (said.epsg = ParseEpsg(e.crs)) == 0) {
        return Fail(why, "its entry's crs '" + e.crs + "' is not EPSG:nnnn");
    }
    if (e.kind == "colour" || e.kind == "color") said.valueUnit = "sRGB byte";
    else if (e.kind == "height") said.valueUnit = "m";
    else if (!e.kind.empty()) return Fail(why, "its entry's kind '" + e.kind + "' is neither colour nor height");
    const std::string ext = std::filesystem::path(path).extension().string();
    std::unique_ptr<FieldLoader> L =
        reg.Open(path, (said.epsg > 0 || !e.kind.empty()) ? &said : nullptr);
    if (!L) {
        return Fail(why, reg.Knows(ext.empty() ? ext : ext.substr(1)) ? ImageLoaderWhy()
                                                                   : "no loader reads " + ext);
    }
    if (L->Channels() != 4) return Fail(why, "its loader gives no colour (RGBA)");
    if (!Place(L->Ref(), why)) return false;

    // THE MIP CHAIN: level 0 is the file; each coarser level the alpha-weighted mean of the 2x2
    // above it (those that exist). A GeoTIFF's own overviews are not reachable through WIC
    // (core/ImageLoader.h), so the chain is always computed.
    m_levels.assign(1, Level{m_ref.width, m_ref.height, {}});
    Pull(*L, m_levels[0].rgba);
    while (m_levels.back().w > 1 || m_levels.back().h > 1) {
        const Level& up = m_levels.back();
        Level lv{(up.w + 1) / 2, (up.h + 1) / 2, {}};
        lv.rgba.assign(size_t(lv.w) * lv.h * 4, 0);
        for (uint32_t y = 0; y < lv.h; ++y) {
            for (uint32_t x = 0; x < lv.w; ++x) {
                double acc[3] = {0.0, 0.0, 0.0}, a = 0.0;
                int n = 0;
                for (uint32_t k = 0; k < 4; ++k) {
                    const uint32_t sx = 2 * x + (k & 1), sy = 2 * y + (k >> 1);
                    if (sx >= up.w || sy >= up.h) continue;
                    const uint8_t* p = &up.rgba[(size_t(sy) * up.w + sx) * 4];
                    for (int c = 0; c < 3; ++c) acc[c] += double(p[c]) * p[3];
                    a += p[3];
                    ++n;
                }
                uint8_t* d = &lv.rgba[(size_t(y) * lv.w + x) * 4];
                for (int c = 0; c < 3 && a > 0.0; ++c) d[c] = static_cast<uint8_t>(acc[c] / a + 0.5);
                d[3] = static_cast<uint8_t>(a / n + 0.5);
            }
        }
        m_levels.push_back(std::move(lv));
    }
    m_over = e.over;
    const std::string stem = std::filesystem::path(path).stem().string();
    m_info.name = e.name.empty() ? stem : (e.file.empty() ? e.name + "." + stem : e.name);
    m_info.structure = L->Structure();
    Log("[raster] %s: %s -- grain %.4g m, %.6f..%.6f E, %.6f..%.6f N, %zu levels, over %g",
        m_info.name.c_str(), m_info.structure.c_str(), m_grainM, m_info.lon0, m_info.lon1,
        m_info.lat0, m_info.lat1, m_levels.size(), m_over);
    return true;
}

bool RasterFileSource::Place(const GeoRef& ref, std::string* why) {
    int zone = 0;
    bool south = false;
    CrsKind kind = CrsKind::Unknown;
    std::string w;
    if (!CrsOfEpsg(ref.epsg, kind, zone, south, &w)) return Fail(why, w);
    m_ref = ref;
    m_ref.kind = kind;
    if (zone) {
        m_tm = TransverseMercator::Utm(zone);
        m_tm.falseN = south ? 10000000.0 : 0.0;
    }
    // The footprint in the exchange frame: the outer edges' corners and midpoints, and the centre.
    double lo0 = 1e9, la0 = 1e9, lo1 = -1e9, la1 = -1e9;
    for (int i = 0; i < 9; ++i) {
        double la = 0.0, lo = 0.0;
        if (!ToLatLon(m_ref.originX + m_ref.scaleX * m_ref.width * 0.5 * (i % 3),
                      m_ref.originY + m_ref.scaleY * m_ref.height * 0.5 * (i / 3), la, lo)) {
            return Fail(why, "its corners do not resolve to WGS84");
        }
        lo0 = (std::min)(lo0, lo / kD2R);
        lo1 = (std::max)(lo1, lo / kD2R);
        la0 = (std::min)(la0, la / kD2R);
        la1 = (std::max)(la1, la / kD2R);
    }
    m_info.lon0 = lo0;
    m_info.lat0 = la0;
    m_info.lon1 = lo1;
    m_info.lat1 = la1;
    m_info.crs = m_ref.Describe();
    // THE GRAIN: one texel's ground at the centre, the finer of its two sides, metres.
    const double c = std::cos(0.5 * (la0 + la1) * kD2R), deg = kR * kD2R;
    const bool geo = kind == CrsKind::Geographic, merc = kind == CrsKind::WebMercator;
    const double gx = std::abs(m_ref.scaleX) * (geo ? deg * c : merc ? c : 1.0);
    const double gy = std::abs(m_ref.scaleY) * (geo ? deg : merc ? c : 1.0);
    m_grainM = (std::min)(gx, gy);
    m_info.cmPerPixel = 100.0 * m_grainM;
    return true;
}

bool RasterFileSource::ToCrs(double lat, double lon, double& x, double& y) const {
    switch (m_ref.kind) {
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
    if (m_ref.kind == CrsKind::Geographic) {
        lat = y * kD2R;
        lon = x * kD2R;
        return std::abs(y) <= 90.0;
    }
    if (m_ref.kind == CrsKind::WebMercator) {
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

float RasterFileSource::Sample(double latRad, double lonRad, double groundResM, const PaintCtx&,
                               uint8_t rgba[4]) {
    double x = 0.0, y = 0.0;
    if (m_levels.empty() || !ToCrs(latRad, lonRad, x, y)) return 0.0f;
    // Texels of level 0 from the outer corner of texel (0, 0); the scale's sign is the direction.
    const double u = (x - m_ref.originX) / m_ref.scaleX;
    const double v = (y - m_ref.originY) / m_ref.scaleY;
    if (!(u >= 0.0 && v >= 0.0 && u <= m_ref.width && v <= m_ref.height)) return 0.0f;
    // THE LEVEL: its own grain against the ground asked for (groundResM is Mercator-equatorial;
    // x cos(lat) is metres on the ground -- the aerial source's rule).
    const double want = groundResM * std::cos(latRad);
    const int L = std::clamp(static_cast<int>(std::floor(std::log2((std::max)(want, m_grainM) / m_grainM))),
                             0, static_cast<int>(m_levels.size()) - 1);
    const Level& lv = m_levels[L];
    // Bilinear between texel CENTRES (at integers here), premultiplied by the file's alpha.
    const double fx = u * lv.w / m_ref.width - 0.5, fy = v * lv.h / m_ref.height - 0.5;
    const int x0 = static_cast<int>(std::floor(fx)), y0 = static_cast<int>(std::floor(fy));
    const double tx = fx - x0, ty = fy - y0;
    double acc[3] = {0.0, 0.0, 0.0}, a = 0.0;
    for (int k = 0; k < 4; ++k) {
        const int xi = x0 + (k & 1), yi = y0 + (k >> 1);
        if (xi < 0 || yi < 0 || xi >= int(lv.w) || yi >= int(lv.h)) continue;
        const uint8_t* p = &lv.rgba[(size_t(yi) * lv.w + xi) * 4];
        const double wa = ((k & 1) ? tx : 1.0 - tx) * ((k >> 1) ? ty : 1.0 - ty) * p[3];
        for (int c = 0; c < 3; ++c) acc[c] += wa * p[c];
        a += wa;
    }
    if (a <= 0.0) return 0.0f;
    for (int c = 0; c < 3; ++c) rgba[c] = static_cast<uint8_t>(acc[c] / a + 0.5);
    rgba[3] = 255;
    return static_cast<float>(a / 255.0);
}

std::vector<std::unique_ptr<RasterFileSource>> LoadRasterSources(
    const std::vector<RasterEntry>& entries) {
    LoaderRegistry reg;
    RegisterImageLoaders(reg);
    std::vector<std::unique_ptr<RasterFileSource>> out;
    for (const RasterEntry& e : entries) {
        std::vector<std::string> files;
        if (!e.file.empty()) files.push_back(e.file);
        std::error_code ec;
        for (auto it = std::filesystem::directory_iterator(e.file.empty() ? e.folder : "", ec);
             !ec && it != std::filesystem::directory_iterator(); it.increment(ec)) {
            const std::string n = it->path().filename().string();
            if (it->is_regular_file(ec) && Matches(e.match.empty() ? "*" : e.match.c_str(), n.c_str())) {
                files.push_back(it->path().generic_string());
            }
        }
        std::sort(files.begin() + (e.file.empty() ? 0 : 1), files.end());   // by name, not by disk
        if (files.empty()) Log("[raster] sources: '%s' matches nothing in '%s'", e.match.c_str(), e.folder.c_str());
        for (const std::string& f : files) {
            auto s = std::make_unique<RasterFileSource>();
            std::string why;
            if (s->Load(reg, f, e, &why)) {
                out.push_back(std::move(s));
            } else {
                Log("[raster] %s REFUSED: %s -- the run goes on without it", f.c_str(), why.c_str());
            }
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

}  // namespace ga
