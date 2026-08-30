#include "compose/Sources.h"

#include "core/Json.h"
#include "core/TileProviders.h"

#include <windows.h>

#include <algorithm>
#include <cmath>
#include <fstream>

namespace ga {

namespace {
constexpr double kPi = 3.14159265358979;
constexpr double kMercCirc = 40075016.686;

float Feather(double edge, double featherFrac) {
    const double t = std::clamp(edge / featherFrac, 0.0, 1.0);
    return static_cast<float>(t * t * (3.0 - 2.0 * t));
}

// Bilinear over an int16 grid, x optionally wrapped.
float Bilinear(const std::vector<int16_t>& g, int nx, int ny, double fx, double fy, bool wrapX) {
    fx -= 0.5;
    fy -= 0.5;
    int x0 = static_cast<int>(std::floor(fx));
    int y0 = static_cast<int>(std::floor(fy));
    const double tx = fx - x0, ty = fy - y0;
    int x1 = x0 + 1, y1 = y0 + 1;
    if (wrapX) {
        x0 = ((x0 % nx) + nx) % nx;
        x1 = ((x1 % nx) + nx) % nx;
    } else {
        x0 = std::clamp(x0, 0, nx - 1);
        x1 = std::clamp(x1, 0, nx - 1);
    }
    y0 = std::clamp(y0, 0, ny - 1);
    y1 = std::clamp(y1, 0, ny - 1);
    const double a = g[static_cast<size_t>(y0) * nx + x0] * (1 - tx) +
                     g[static_cast<size_t>(y0) * nx + x1] * tx;
    const double b = g[static_cast<size_t>(y1) * nx + x0] * (1 - tx) +
                     g[static_cast<size_t>(y1) * nx + x1] * tx;
    return static_cast<float>(a * (1 - ty) + b * ty);
}
}  // namespace

// ------------------------------------------------------------------------------ Google

GoogleColorSource::GoogleColorSource(GoogleTileProvider* prov) : m_prov(prov) {
    m_info = {"google.satellite", "mercator-tile-tree jpeg 256px (sessioned, cache-first)",
              "EPSG:3857 web-mercator",
              955.0 /* z14 politeness cap at the equator */, -180, -85, 180, 85};
}

// Zoom from the requested footprint: z such that Mercator metres/px matches groundResM.
// Capped at 14 -- deeper zooms are a budget decision a REALIZATION makes by asking for a
// finer groundRes only inside a window it owns; the global cube can never demand them.
static int ZoomFor(double groundResM) {
    constexpr double kCirc = 40075016.686;
    return std::clamp(static_cast<int>(std::lround(std::log2(kCirc / (256.0 * groundResM)))),
                      0, 14);
}

bool GoogleColorSource::Pixel(int z, double latRad, double lonRad, uint8_t rgb[3]) {
    const double n = static_cast<double>(1u << z) * 256.0;
    const double latC = std::clamp(latRad, -1.4844, 1.4844);   // Mercator band; poles clamp
    const double mx = (lonRad / kPi * 0.5 + 0.5) * n;
    const double my = (0.5 - std::log(std::tan(kPi * 0.25 + latC * 0.5)) / (2.0 * kPi)) * n;
    const int gx = std::clamp(static_cast<int>(mx), 0, static_cast<int>(n) - 1);
    const int gy = std::clamp(static_cast<int>(my), 0, static_cast<int>(n) - 1);
    const auto tile = m_prov->Decoded(z, gx / 256, gy / 256);
    if (!tile) return false;
    const uint8_t* s = &(*tile)[((gy & 255) * 256 + (gx & 255)) * 4];
    rgb[0] = s[0];
    rgb[1] = s[1];
    rgb[2] = s[2];
    return true;
}

void GoogleColorSource::BeginTile(double, double, double, double, double, PaintCtx&) {
    // M6j postmortem: a cross-zoom "grade normalization" lived here for one day. Measured
    // same-footprint deltas are ~2.5% (the loud banding had been our OWN lighting bugs), but
    // the z10 reference is a different CAPTURE over seasonal land cover -- the marsh got
    // bleached toward another season's mosaic, clamped at +35%. The user called it both
    // times. The pixels ship AS GOOGLE MADE THEM; per-tile PaintCtx stays for compositor ops
    // that earn their place.
}

float GoogleColorSource::Sample(double latRad, double lonRad, double groundResM,
                                const PaintCtx&, uint8_t rgba[4]) {
    if (!m_prov || !m_prov->Ready()) return 0.0f;
    uint8_t rgb[3];
    if (!Pixel(ZoomFor(groundResM), latRad, lonRad, rgb)) {
        return -1.0f;   // TRANSIENT: coverage exists, fetch failed -- do not cache
    }
    rgba[0] = rgb[0];
    rgba[1] = rgb[1];
    rgba[2] = rgb[2];
    rgba[3] = 255;
    return 1.0f;
}

// ------------------------------------------------------------------------------ equirect

EquirectHeightSource::EquirectHeightSource(const char* name, const char* structure,
                                           double cmPerPixel, const std::vector<int16_t>* elev,
                                           int nx, int ny)
    : m_elev(elev), m_nx(nx), m_ny(ny) {
    m_info = {name, structure, "EPSG:4326 equirect (plate carree)", cmPerPixel, -180, -90, 180,
              90};
}

float EquirectHeightSource::Sample(double latRad, double lonRad, double, float& metres) {
    if (!m_elev || m_nx <= 0) return 0.0f;
    const double u = lonRad / (2.0 * kPi) + 0.5;
    const double v = std::clamp(0.5 - latRad / kPi, 0.0, 1.0);
    metres = Bilinear(*m_elev, m_nx, m_ny, u * m_nx, v * m_ny, true);
    return 1.0f;
}

// ------------------------------------------------------------------------------ window grid

WindowHeightSource::WindowHeightSource(const char* name, const char* structure,
                                       double cmPerPixel, const std::vector<int16_t>* elev,
                                       int nx, int ny, double lon0, double lat1, double dLon,
                                       double dLat, double featherFrac)
    : m_elev(elev), m_nx(nx), m_ny(ny), m_lon0(lon0), m_lat1(lat1), m_dLon(dLon),
      m_dLat(std::abs(dLat)), m_feather(featherFrac) {
    m_info = {name, structure, "EPSG:4326 window (row 0 north)", cmPerPixel, lon0,
              lat1 - ny * std::abs(dLat), lon0 + nx * dLon, lat1};
}

float WindowHeightSource::Sample(double latRad, double lonRad, double, float& metres) {
    if (!m_elev || m_nx <= 0) return 0.0f;
    const double lonDeg = lonRad * 180.0 / kPi, latDeg = latRad * 180.0 / kPi;
    const double u = (lonDeg - m_lon0) / (m_nx * m_dLon);
    const double v = (m_lat1 - latDeg) / (m_ny * m_dLat);
    if (u < 0.0 || u > 1.0 || v < 0.0 || v > 1.0) return 0.0f;
    metres = Bilinear(*m_elev, m_nx, m_ny, u * m_nx, v * m_ny, false);
    const double edge = (std::min)((std::min)(u, 1.0 - u), (std::min)(v, 1.0 - v));
    return Feather(edge, m_feather);   // the old render-time NeWeight feather, at paint time
}

// ------------------------------------------------------------------------------ aerial ortho

AerialOrthoSource::~AerialOrthoSource() {
    // Handles were pushed as (view, mapping, file) triples per tile.
    for (size_t i = 0; i + 2 < m_handles.size(); i += 3) {
        UnmapViewOfFile(m_handles[i]);
        CloseHandle(m_handles[i + 1]);
        CloseHandle(m_handles[i + 2]);
    }
}

bool AerialOrthoSource::Load(const std::string& jsonPath) {
    std::ifstream f(jsonPath, std::ios::binary);
    if (!f) return false;
    std::string text((std::istreambuf_iterator<char>(f)), std::istreambuf_iterator<char>());
    std::string err;
    const JsonValue v = JsonParser::Parse(text, &err);
    const JsonValue* tiles = v.Get("tiles");
    if (!tiles || tiles->arr.empty()) return false;
    const std::string dir = jsonPath.substr(0, jsonPath.find_last_of("/\\") + 1);

    double lonMin = 180, lonMax = -180, latMin = 90, latMax = -90;
    for (const JsonValue& t : tiles->arr) {
        Tile tile;
        tile.e0 = t.Num("utm_e0", 0);
        tile.n0 = t.Num("utm_n0", 0);
        tile.e1 = t.Num("utm_e1", 0);
        tile.n1 = t.Num("utm_n1", 0);
        tile.channels = static_cast<uint32_t>(t.Num("channels", 3));
        const std::string path = dir + t.Str("file");
        const std::wstring wpath(path.begin(), path.end());
        HANDLE file = CreateFileW(wpath.c_str(), GENERIC_READ, FILE_SHARE_READ, nullptr,
                                  OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, nullptr);
        if (file == INVALID_HANDLE_VALUE) {
            Log("[aerial] missing %s; skipping tile", path.c_str());
            continue;
        }
        HANDLE mapping = CreateFileMappingW(file, nullptr, PAGE_READONLY, 0, 0, nullptr);
        const void* view =
            mapping ? MapViewOfFile(mapping, FILE_MAP_READ, 0, 0, 0) : nullptr;
        if (!view) {
            CloseHandle(mapping);
            CloseHandle(file);
            continue;
        }
        m_handles.push_back(const_cast<void*>(view));
        m_handles.push_back(mapping);
        m_handles.push_back(file);
        tile.data = static_cast<const uint8_t*>(view);
        if (const JsonValue* mips = t.Get("mips")) {
            for (const JsonValue& m : mips->arr) {
                tile.mips.push_back({static_cast<uint32_t>(m.Num("px", 0)),
                                     static_cast<uint64_t>(m.Num("offset", 0))});
            }
        }
        if (tile.mips.empty()) continue;
        m_ue0 = (std::min)(m_ue0, tile.e0);
        m_un0 = (std::min)(m_un0, tile.n0);
        m_ue1 = (std::max)(m_ue1, tile.e1);
        m_un1 = (std::max)(m_un1, tile.n1);
        if (const JsonValue* bb = t.Get("wgs84_bbox"); bb && bb->arr.size() == 4) {
            lonMin = (std::min)(lonMin, bb->arr[0].number);
            latMin = (std::min)(latMin, bb->arr[1].number);
            lonMax = (std::max)(lonMax, bb->arr[2].number);
            latMax = (std::max)(latMax, bb->arr[3].number);
        }
        m_tiles.push_back(std::move(tile));
    }
    if (m_tiles.empty()) return false;
    // Identity from the json when present, so one class serves both the MassGIS orthos and
    // any user GeoTIFF overlay ("highlights") dropped through harvest_overlay.py.
    m_info = {v.Str("name", "massgis.coq2023"),
              v.Str("structure", "jp2 ortho, 1500 m USNG tiles, plane-flown leaf-off"),
              v.Str("crs", "EPSG:6348 NAD83(2011)/UTM 19N (~1 m vs WGS84, uncorrected)"),
              v.Num("cm_per_px", 15.0), lonMin, latMin, lonMax, latMax};
    Log("[aerial] %s: %zu tiles mapped (UTM E %.0f..%.0f, N %.0f..%.0f, %uch)",
        m_info.name.c_str(), m_tiles.size(), m_ue0, m_ue1, m_un0, m_un1,
        m_tiles[0].channels);
    return true;
}

float AerialOrthoSource::Sample(double latRad, double lonRad, double groundResM,
                                const PaintCtx&, uint8_t rgba[4]) {
    if (m_tiles.empty()) return 0.0f;
    static const TransverseMercator kUtm19 = TransverseMercator::Utm(19);
    double E, N;
    kUtm19.Forward(latRad, lonRad, E, N);
    if (E <= m_ue0 || E >= m_ue1 || N <= m_un0 || N >= m_un1) return 0.0f;
    for (const Tile& t : m_tiles) {
        if (E < t.e0 || E >= t.e1 || N < t.n0 || N >= t.n1) continue;
        // Meters-per-pixel contract: pick the mip whose GROUND resolution matches the paint
        // footprint (groundResM is Mercator-equatorial; x cos(lat) makes it true metres).
        // The tile's own native m/px anchors the pick -- 15 cm orthos and 50 cm overlays
        // ride the same code.
        const double nativeM = (t.e1 - t.e0) / t.mips[0].px;
        const double wantM = groundResM * std::cos(latRad);
        const int lvl = std::clamp(
            static_cast<int>(std::floor(std::log2((std::max)(wantM, nativeM) / nativeM))),
            0, static_cast<int>(t.mips.size()) - 1);
        const MipLevel& mp = t.mips[lvl];
        const double pxPerM = mp.px / (t.e1 - t.e0);
        const double fx = (E - t.e0) * pxPerM - 0.5;
        const double fy = (t.n1 - N) * pxPerM - 0.5;   // row 0 = north
        const int x0 = std::clamp(static_cast<int>(std::floor(fx)), 0,
                                  static_cast<int>(mp.px) - 2);
        const int y0 = std::clamp(static_cast<int>(std::floor(fy)), 0,
                                  static_cast<int>(mp.px) - 2);
        const double tx = std::clamp(fx - x0, 0.0, 1.0), ty = std::clamp(fy - y0, 0.0, 1.0);
        const uint8_t* base = t.data + mp.offset;
        const uint32_t nc = t.channels;
        auto texel = [&](int x, int y) {
            return base + (static_cast<size_t>(y) * mp.px + x) * nc;
        };
        const uint8_t* p00 = texel(x0, y0);
        const uint8_t* p10 = texel(x0 + 1, y0);
        const uint8_t* p01 = texel(x0, y0 + 1);
        const uint8_t* p11 = texel(x0 + 1, y0 + 1);
        auto bilerp = [&](uint32_t c) {
            const double a = p00[c] * (1 - tx) + p10[c] * tx;
            const double b = p01[c] * (1 - tx) + p11[c] * tx;
            return a * (1 - ty) + b * ty;
        };
        for (uint32_t c = 0; c < 3; ++c) {
            rgba[c] = static_cast<uint8_t>(bilerp(c) + 0.5);
        }
        rgba[3] = 255;
        // Feather against the UNION boundary (interior tile seams stay seamless -- one
        // flight, one capture), 25 m wide.
        const double dm = (std::min)((std::min)(E - m_ue0, m_ue1 - E),
                                     (std::min)(N - m_un0, m_un1 - N));
        const double tfe = std::clamp(dm / 25.0, 0.0, 1.0);
        float w = static_cast<float>(tfe * tfe * (3.0 - 2.0 * tfe));
        // ALPHA IS FIBER: a 4-channel plane's per-pixel alpha multiplies the paint weight,
        // so a mostly-transparent overlay (a GeoTIFF of highlights) bleeds through the
        // composed quadtree pixel by pixel -- flat image and quad tree meet in the paint
        // loop's lerp, never in a special case.
        if (nc >= 4) w *= static_cast<float>(bilerp(3) / 255.0);
        return w;
    }
    return 0.0f;
}

// ------------------------------------------------------------------------------ CUDEM

CudemHeightSource::CudemHeightSource(const BathyModel* bathy, double featherFrac,
                                     const char* name)
    : m_bathy(bathy), m_feather(featherFrac) {
    const double lon0 = BathyModel::kOrgLon + bathy->WorldX0() / BathyModel::kMPerLon;
    const double lat0 = BathyModel::kOrgLat + bathy->WorldZ0() / BathyModel::kMPerLat;
    m_info = {name,
              "geotiff-window float32 (thalweg-preserving resample)",
              "local tangent metres @ ACT0816 (from EPSG:4326 GeoTIFF)", 1370.0, lon0, lat0,
              lon0 + bathy->WorldSizeX() / BathyModel::kMPerLon,
              lat0 + bathy->WorldSizeZ() / BathyModel::kMPerLat};
}

// ------------------------------------------------------------------------------ hand edits

bool EditsHeightSource::Load(const std::string& geojsonPath, float crestNavd) {
    m_crest = crestNavd;
    std::ifstream f(geojsonPath, std::ios::binary);
    if (!f) return false;
    const std::string text((std::istreambuf_iterator<char>(f)),
                           std::istreambuf_iterator<char>());
    std::string err;
    const JsonValue root = JsonParser::Parse(text, &err);
    if (!err.empty()) return false;
    const JsonValue* feats = root.Get("features");
    if (!feats) return false;
    double L0 = 1e9, A0 = 1e9, L1 = -1e9, A1 = -1e9;
    for (const JsonValue& ft : feats->arr) {
        const JsonValue* props = ft.Get("properties");
        const JsonValue* geom = ft.Get("geometry");
        if (!props || !geom) continue;
        const JsonValue* coords = geom->Get("coordinates");
        if (!coords || coords->arr.empty()) continue;
        Ring r;
        r.land = props->Str("mask") != "water";
        for (const JsonValue& pt : coords->arr[0].arr) {
            if (pt.arr.size() < 2) continue;
            const double lon = pt.arr[0].number, lat = pt.arr[1].number;
            r.pts.push_back({lon, lat});
            r.lon0 = (std::min)(r.lon0, lon); r.lon1 = (std::max)(r.lon1, lon);
            r.lat0 = (std::min)(r.lat0, lat); r.lat1 = (std::max)(r.lat1, lat);
        }
        if (r.pts.size() < 3) continue;
        L0 = (std::min)(L0, r.lon0); L1 = (std::max)(L1, r.lon1);
        A0 = (std::min)(A0, r.lat0); A1 = (std::max)(A1, r.lat1);
        m_rings.push_back(std::move(r));
    }
    if (m_rings.empty()) return false;
    // THE IDENTITY IS THE CONTENT: hash the file bytes into the structure string, so an
    // operator edit changes exactly the touched tiles' cache identity (the soak rule then
    // repaints them at every rung and leaves the rest of the planet immortal).
    uint64_t h = 14695981039346656037ull;
    for (const char c : text) h = (h ^ static_cast<uint8_t>(c)) * 1099511628211ull;
    char structure[96];
    snprintf(structure, sizeof(structure), "hand-edit polygons (crest %.1f m) v%08x",
             crestNavd, static_cast<uint32_t>(h & 0xFFFFFFFFu));
    m_info = {"survey.edits", structure, "EPSG:4326 (edits.geojson -- THE LAW)", 500.0,
              L0 - 0.001, A0 - 0.001, L1 + 0.001, A1 + 0.001};
    return true;
}

float EditsHeightSource::Sample(double latRad, double lonRad, double, float& metres) {
    const double lon = lonRad * 180.0 / kPi, lat = latRad * 180.0 / kPi;
    for (const Ring& r : m_rings) {
        if (lon < r.lon0 || lon > r.lon1 || lat < r.lat0 || lat > r.lat1) continue;
        bool in = false;
        const auto& p = r.pts;
        for (size_t i = 0, j = p.size() - 1; i < p.size(); j = i++) {
            if ((p[i].second > lat) != (p[j].second > lat) &&
                lon < (p[j].first - p[i].first) * (lat - p[i].second) /
                          (p[j].second - p[i].second) +
                          p[i].first) {
                in = !in;
            }
        }
        if (in) {
            metres = r.land ? m_crest : -1.0f;   // riprap crest, or dredged subtidal
            return 1.0f;
        }
    }
    return 0.0f;
}

float CudemHeightSource::Sample(double latRad, double lonRad, double, float& metres) {
    if (!m_bathy || !m_bathy->Ready()) return 0.0f;
    const double x = (lonRad * 180.0 / kPi - BathyModel::kOrgLon) * BathyModel::kMPerLon;
    const double z = (latRad * 180.0 / kPi - BathyModel::kOrgLat) * BathyModel::kMPerLat;
    const float v = m_bathy->SampleWorld(static_cast<float>(x), static_cast<float>(z));
    if (v < -9000.0f) return 0.0f;   // outside the grid, or nodata
    const double ex = (std::min)(x - m_bathy->WorldX0(),
                                 m_bathy->WorldX0() + m_bathy->WorldSizeX() - x) /
                      m_bathy->WorldSizeX();
    const double ez = (std::min)(z - m_bathy->WorldZ0(),
                                 m_bathy->WorldZ0() + m_bathy->WorldSizeZ() - z) /
                      m_bathy->WorldSizeZ();
    metres = v;
    return Feather((std::min)(ex, ez), m_feather);
}

}  // namespace ga
