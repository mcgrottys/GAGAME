#include "compose/Sources.h"

#include "core/Json.h"
#include "core/TileProviders.h"

#include <windows.h>

#include <algorithm>
#include <cmath>
#include <filesystem>
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

bool GoogleColorSource::Refusals(uint32_t& refused) const {
    if (!m_prov || !m_prov->Ready()) return false;
    refused = m_prov->Refused();
    return true;
}

// ------------------------------------------------------------------------------ equirect

EquirectHeightSource::EquirectHeightSource(const char* name, const char* structure,
                                           double cmPerPixel, const std::vector<int16_t>* elev,
                                           int nx, int ny, double datumShiftM)
    : m_elev(elev), m_nx(nx), m_ny(ny), m_datumShift(datumShiftM) {
    // A plain local. This was a pair of function-local STATIC buffers picked by a
    // non-atomic `slot++`, so two sources constructed at once raced on both the index and the
    // buffer -- and it was never needed: SourceInfo::crs is a std::string and copies.
    char crs[96];
    if (datumShiftM != 0.0) {
        snprintf(crs, sizeof(crs), "EPSG:4326 equirect, vdatum %+.3f m -> NAVD88", datumShiftM);
    } else {
        snprintf(crs, sizeof(crs), "EPSG:4326 equirect (plate carree)");
    }
    m_info = {name, structure, crs, cmPerPixel, -180, -90, 180, 90};
}

float EquirectHeightSource::Sample(double latRad, double lonRad, double, float& metres) {
    if (!m_elev || m_nx <= 0) return 0.0f;
    const double u = lonRad / (2.0 * kPi) + 0.5;
    const double v = std::clamp(0.5 - latRad / kPi, 0.0, 1.0);
    metres = Bilinear(*m_elev, m_nx, m_ny, u * m_nx, v * m_ny, true) +
             static_cast<float>(m_datumShift);
    return 1.0f;
}

// ------------------------------------------------------------------------------ window grid

WindowHeightSource::WindowHeightSource(const char* name, const char* structure,
                                       double cmPerPixel, const std::vector<int16_t>* elev,
                                       int nx, int ny, double lon0, double lat1, double dLon,
                                       double dLat, double featherFrac, double datumShiftM)
    : m_elev(elev), m_nx(nx), m_ny(ny), m_lon0(lon0), m_lat1(lat1), m_dLon(dLon),
      m_dLat(std::abs(dLat)), m_feather(featherFrac), m_datumShift(datumShiftM) {
    char crs[96];   // was a function-local static shared by every instance
    if (datumShiftM != 0.0) {
        snprintf(crs, sizeof(crs), "EPSG:4326 window (row 0 N), vdatum %+.3f m -> NAVD88",
                 datumShiftM);
    } else {
        snprintf(crs, sizeof(crs), "EPSG:4326 window (row 0 north)");
    }
    m_info = {name, structure, crs, cmPerPixel, lon0, lat1 - ny * std::abs(dLat),
              lon0 + nx * dLon, lat1};
}

float WindowHeightSource::Sample(double latRad, double lonRad, double, float& metres) {
    if (!m_elev || m_nx <= 0) return 0.0f;
    const double lonDeg = lonRad * 180.0 / kPi, latDeg = latRad * 180.0 / kPi;
    const double u = (lonDeg - m_lon0) / (m_nx * m_dLon);
    const double v = (m_lat1 - latDeg) / (m_ny * m_dLat);
    if (u < 0.0 || u > 1.0 || v < 0.0 || v > 1.0) return 0.0f;
    metres = Bilinear(*m_elev, m_nx, m_ny, u * m_nx, v * m_ny, false) +
             static_cast<float>(m_datumShift);
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
    metres = v;
    return EdgeWeight(x, z);
}

float CudemHeightSource::EdgeWeight(double x, double z) const {
    const double ex = (std::min)(x - m_bathy->WorldX0(),
                                 m_bathy->WorldX0() + m_bathy->WorldSizeX() - x) /
                      m_bathy->WorldSizeX();
    const double ez = (std::min)(z - m_bathy->WorldZ0(),
                                 m_bathy->WorldZ0() + m_bathy->WorldSizeZ() - z) /
                      m_bathy->WorldSizeZ();
    return Feather((std::min)(ex, ez), m_feather);
}

bool CudemHeightSource::FullWeightCells(int& c0, int& r0, int& c1, int& r1) const {
    if (!m_bathy || !m_bathy->Ready()) return false;
    const int nx = m_bathy->Nx(), ny = m_bathy->Ny();
    // A cell's centre on the lattice, in the world frame Sample puts a sample point in (the
    // linear lon/lat map both frames were built with); the other axis held at the window's
    // middle, where only the axis being walked can bring the weight under one.
    auto x = [&](int c) {
        return (m_bathy->Lon0() + (c + 0.5) * m_bathy->Dlon() - BathyModel::kOrgLon) *
               BathyModel::kMPerLon;
    };
    auto z = [&](int r) {
        return (m_bathy->Lat1() - (r + 0.5) * m_bathy->Dlat() - BathyModel::kOrgLat) *
               BathyModel::kMPerLat;
    };
    const double xMid = x(nx / 2), zMid = z(ny / 2);
    c0 = 0;
    while (c0 < nx && EdgeWeight(x(c0), zMid) < 1.0f) ++c0;
    c1 = nx - 1;
    while (c1 >= c0 && EdgeWeight(x(c1), zMid) < 1.0f) --c1;
    r0 = 0;
    while (r0 < ny && EdgeWeight(xMid, z(r0)) < 1.0f) ++r0;
    r1 = ny - 1;
    while (r1 >= r0 && EdgeWeight(xMid, z(r1)) < 1.0f) --r1;
    return c0 <= c1 && r0 <= r1;
}

// ------------------------------------------------------------------- the bed classifier

namespace {

// Deterministic smooth value noise on a metric lattice: the bed's grain. Two octaves,
// each FOLDED by the texel footprint (the M6t rule wearing a different hat): a rung too
// coarse to resolve an octave sheds it instead of aliasing it.
float BedHash(long long ix, long long iy) {
    uint64_t v = static_cast<uint64_t>(ix) * 0x9E3779B97F4A7C15ull ^
                 static_cast<uint64_t>(iy) * 0xC2B2AE3D27D4EB4Full;
    v ^= v >> 29; v *= 0xBF58476D1CE4E5B9ull; v ^= v >> 32;
    return static_cast<float>(v & 0xFFFFFFu) / static_cast<float>(0xFFFFFFu);
}

float BedValNoise(double x, double y) {
    const double fx = std::floor(x), fy = std::floor(y);
    const long long ix = static_cast<long long>(fx), iy = static_cast<long long>(fy);
    float tx = static_cast<float>(x - fx), ty = static_cast<float>(y - fy);
    tx = tx * tx * (3.0f - 2.0f * tx);
    ty = ty * ty * (3.0f - 2.0f * ty);
    const float a = BedHash(ix, iy), b = BedHash(ix + 1, iy);
    const float c = BedHash(ix, iy + 1), d = BedHash(ix + 1, iy + 1);
    return (a + (b - a) * tx) + ((c + (d - c) * tx) - (a + (b - a) * tx)) * ty;
}

const char* kBedRulesDefault = R"JSON({
  "version": 1,
  "comment": "synth.bed -- the compositor's first synthesis node. Colors are DRY bed albedo, sRGB 0..1: the renderer's refracted ray applies the water's own attenuation, so author the bed as if drained. Rules run in order; the first whose slope gates pass paints. Add polygons to bed_zones.geojson (properties.bed = a rule name, optional properties.albedo = [r,g,b]) to override by survey or by hand; every edit repaints exactly the touched tiles.",
  "bounds": {"lon0": -71.15, "lat0": 42.15, "lon1": -70.25, "lat1": 43.10},
  "alpha": {"full0": -9.0, "full1": 0.4, "off0": -14.0, "off1": 1.2},
  "rules": [
    {"name": "rock", "slopeMin": 0.05, "shallow": [0.37, 0.35, 0.31], "deep": [0.28, 0.28, 0.26], "noise": 0.16},
    {"name": "sand", "shallow": [0.74, 0.68, 0.53], "deep": [0.51, 0.49, 0.40], "noise": 0.10}
  ],
  "zones": "data/bed/bed_zones.geojson"
})JSON";

const char* kBedZonesDefault =
    "{\"type\": \"FeatureCollection\", \"features\": []}\n";

}  // namespace

bool BedSynthSource::Load(const std::string& rulesPath, const Compositor* comp,
                          int hgtChannel) {
    m_comp = comp;
    m_hgtCh = hgtChannel;
    if (!comp || hgtChannel < 0) return false;
    // Author the default program if absent -- never clobber an edit (M6p law).
    if (!std::ifstream(rulesPath)) {
        std::filesystem::create_directories(
            std::filesystem::path(rulesPath).parent_path());
        std::ofstream(rulesPath, std::ios::binary) << kBedRulesDefault;
    }
    std::ifstream f(rulesPath, std::ios::binary);
    if (!f) return false;
    const std::string text((std::istreambuf_iterator<char>(f)),
                           std::istreambuf_iterator<char>());
    std::string err;
    const JsonValue root = JsonParser::Parse(text, &err);
    if (!err.empty()) return false;
    if (const JsonValue* b = root.Get("bounds")) {
        m_info.lon0 = b->Num("lon0", -71.15); m_info.lat0 = b->Num("lat0", 42.15);
        m_info.lon1 = b->Num("lon1", -70.25); m_info.lat1 = b->Num("lat1", 43.10);
    }
    if (const JsonValue* a = root.Get("alpha")) {
        m_full0 = a->Num("full0", -9.0); m_full1 = a->Num("full1", 0.4);
        m_off0 = a->Num("off0", -14.0); m_off1 = a->Num("off1", 1.2);
    }
    m_rules.clear();
    if (const JsonValue* rules = root.Get("rules")) {
        for (const JsonValue& rv : rules->arr) {
            Rule r;
            r.name = rv.Str("name", "sand");
            r.slopeMin = rv.Num("slopeMin", -1.0);
            r.slopeMax = rv.Num("slopeMax", 1e9);
            r.noise = static_cast<float>(rv.Num("noise", 0.10));
            if (const JsonValue* c = rv.Get("shallow")) {
                for (int i = 0; i < 3 && i < static_cast<int>(c->arr.size()); ++i)
                    r.shallow[i] = static_cast<float>(c->arr[i].number);
            }
            if (const JsonValue* c = rv.Get("deep")) {
                for (int i = 0; i < 3 && i < static_cast<int>(c->arr.size()); ++i)
                    r.deep[i] = static_cast<float>(c->arr[i].number);
            }
            m_rules.push_back(std::move(r));
        }
    }
    if (m_rules.empty()) return false;
    // The zone overlay: survey or hand polygons, same grammar as edits.geojson.
    std::string zonesText;
    const std::string zonesPath = root.Str("zones", "data/bed/bed_zones.geojson");
    if (!std::ifstream(zonesPath)) {
        std::filesystem::create_directories(
            std::filesystem::path(zonesPath).parent_path());
        std::ofstream(zonesPath, std::ios::binary) << kBedZonesDefault;
    }
    m_zones.clear();
    if (std::ifstream zf(zonesPath, std::ios::binary); zf) {
        zonesText.assign((std::istreambuf_iterator<char>(zf)),
                         std::istreambuf_iterator<char>());
        std::string zerr;
        const JsonValue zroot = JsonParser::Parse(zonesText, &zerr);
        const JsonValue* feats = zerr.empty() ? zroot.Get("features") : nullptr;
        if (feats) {
            for (const JsonValue& ft : feats->arr) {
                const JsonValue* props = ft.Get("properties");
                const JsonValue* geom = ft.Get("geometry");
                if (!props || !geom) continue;
                const JsonValue* coords = geom->Get("coordinates");
                if (!coords || coords->arr.empty()) continue;
                Zone z;
                z.bed = props->Str("bed", "sand");
                if (const JsonValue* c = props->Get("albedo")) {
                    for (int i = 0; i < 3 && i < static_cast<int>(c->arr.size()); ++i)
                        z.albedo[i] = static_cast<float>(c->arr[i].number);
                    z.hasAlbedo = true;
                }
                for (const JsonValue& pt : coords->arr[0].arr) {
                    if (pt.arr.size() < 2) continue;
                    const double lon = pt.arr[0].number, lat = pt.arr[1].number;
                    z.pts.push_back({lon, lat});
                    z.lon0 = (std::min)(z.lon0, lon); z.lon1 = (std::max)(z.lon1, lon);
                    z.lat0 = (std::min)(z.lat0, lat); z.lat1 = (std::max)(z.lat1, lat);
                }
                if (z.pts.size() >= 3) m_zones.push_back(std::move(z));
            }
        }
    }
    // THE IDENTITY IS THE PROGRAM + ITS INPUTS: hash the rules, the zones, and the height
    // stack's signature (this node READS that channel -- if the bed data changes shape, the
    // synthesized tiles must change identity with it).
    uint64_t h = 14695981039346656037ull;
    auto mix = [&h](const std::string& t) {
        for (const char c : t) h = (h ^ static_cast<uint8_t>(c)) * 1099511628211ull;
    };
    mix(text);
    mix(zonesText);
    for (const HeightSource* hs : m_comp->ChannelAt(m_hgtCh).height) {
        mix(hs->Info().name);
        mix(hs->Info().structure);
    }
    char structure[96];
    snprintf(structure, sizeof(structure), "bed-classifier fold2 (rules+zones+bed) v%08x",
             static_cast<uint32_t>(h & 0xFFFFFFFFu));
    m_info.name = "synth.bed";
    m_info.structure = structure;
    m_info.crs = "wgs84.synthesis (height-stack product; bed_rules.json -- THE PROGRAM)";
    m_info.cmPerPixel = 1000.0;
    return true;
}

const BedSynthSource::Rule* BedSynthSource::PickRule(double slope) const {
    for (const Rule& r : m_rules) {
        if (slope >= r.slopeMin && slope <= r.slopeMax) return &r;
    }
    return &m_rules.back();
}

float BedSynthSource::Sample(double latRad, double lonRad, double groundResM,
                             const PaintCtx&, uint8_t rgba[4]) {
    if (!m_comp || m_rules.empty()) return 0.0f;
    const double lon = lonRad * 180.0 / kPi, lat = latRad * 180.0 / kPi;
    // Soft edge at the program's bounds (~700 m) so the node never cuts a seam.
    const double fd = 0.008;
    const double fEdge =
        (std::min)((std::min)(lon - m_info.lon0, m_info.lon1 - lon),
                   (std::min)(lat - m_info.lat0, m_info.lat1 - lat)) / fd;
    if (fEdge <= 0.0) return 0.0f;
    const float wBox = static_cast<float>((std::min)(fEdge, 1.0));
    const float bed = m_comp->SampleHeightStack(m_hgtCh, latRad, lonRad, groundResM);
    // Paint the band the refracted ray can SEE: fade out where the water is deep enough
    // that transmittance kills the bed term anyway, and above the waterline band where
    // land imagery is authoritative.
    //
    // M7s: THE ALPHA FOLDS AS COVERAGE, not as a threshold of the box-average. At a 600 m
    // texel the averaged bed of a marsh plain (+1 crossed with -2 creeks) slid under the
    // +1.2 cutoff and the classifier painted DRY SAND over low-lying LAND at coarse LODs
    // only -- the "smooth brown earth" the albedo lens convicted (and the M6t law again:
    // never threshold a box-averaged field; average the thresholded field). Coarse texels
    // now subsample the alpha decision at ~45 m and average the ANSWERS.
    auto alphaAt = [&](float b) {
        double av = 1.0;
        if (b < m_full0) {
            av = (b - m_off0) / (std::max)(m_full0 - m_off0, 1e-6);
        } else if (b > m_full1) {
            av = (m_off1 - b) / (std::max)(m_off1 - m_full1, 1e-6);
        }
        return (std::min)((std::max)(av, 0.0), 1.0);
    };
    float a;
    if (groundResM > 90.0) {
        const int n = (std::min)(4, static_cast<int>(groundResM / 45.0));
        const double dLat = groundResM / 6371000.0 / n;
        const double dLon = dLat / (std::max)(std::cos(latRad), 0.2);
        double acc = 0.0;
        for (int sy = 0; sy < n; ++sy) {
            for (int sx = 0; sx < n; ++sx) {
                const double la = latRad + (sy - (n - 1) * 0.5) * dLat;
                const double lo = lonRad + (sx - (n - 1) * 0.5) * dLon;
                acc += alphaAt(m_comp->SampleHeightStack(m_hgtCh, la, lo, 45.0));
            }
        }
        a = static_cast<float>(acc / (n * n)) * wBox;
    } else {
        a = static_cast<float>(alphaAt(bed)) * wBox;
    }
    if (a <= 0.004f) return 0.0f;
    // Slope from the SAME stack, at the texel's own scale: the classifier's second input.
    const double dM = (std::max)(groundResM, 8.0);
    const double dLatR = dM / 6371000.0;
    const double dLonR = dM / (6371000.0 * (std::max)(std::cos(latRad), 0.2));
    const float bE = m_comp->SampleHeightStack(m_hgtCh, latRad, lonRad + dLonR, groundResM);
    const float bN = m_comp->SampleHeightStack(m_hgtCh, latRad + dLatR, lonRad, groundResM);
    const double slope = std::sqrt(static_cast<double>(bE - bed) * (bE - bed) +
                                   static_cast<double>(bN - bed) * (bN - bed)) / dM;
    const Rule* rule = PickRule(slope);
    const float* shallow = rule->shallow;
    const float* deep = rule->deep;
    float zoneCol[3];
    for (const Zone& z : m_zones) {
        if (lon < z.lon0 || lon > z.lon1 || lat < z.lat0 || lat > z.lat1) continue;
        bool in = false;
        const auto& p = z.pts;
        for (size_t i = 0, j = p.size() - 1; i < p.size(); j = i++) {
            if ((p[i].second > lat) != (p[j].second > lat) &&
                lon < (p[j].first - p[i].first) * (lat - p[i].second) /
                          (p[j].second - p[i].second) +
                          p[i].first) {
                in = !in;
            }
        }
        if (!in) continue;
        if (z.hasAlbedo) {
            for (int i = 0; i < 3; ++i) zoneCol[i] = z.albedo[i];
            shallow = zoneCol;
            deep = zoneCol;
        } else {
            for (const Rule& r : m_rules) {
                if (r.name == z.bed) { shallow = r.shallow; deep = r.deep; break; }
            }
        }
        break;
    }
    // Depth tone + folded grain: the dry albedo the refracted ray will attenuate.
    const float t = (std::min)((std::max)(-bed / 12.0f, 0.0f), 1.0f);
    const double mx = lonRad * 6371000.0 * std::cos(latRad);
    const double my = latRad * 6371000.0;
    float grain = 0.0f;
    const double wl[2] = {90.0, 22.0};
    const float amp[2] = {1.0f, 0.6f};
    for (int o = 0; o < 2; ++o) {
        const float fold =
            static_cast<float>((std::min)((std::max)(wl[o] / dM, 0.0), 1.0));
        grain += (BedValNoise(mx / wl[o], my / wl[o]) * 2.0f - 1.0f) * amp[o] * fold;
    }
    for (int i = 0; i < 3; ++i) {
        const float c =
            (shallow[i] + (deep[i] - shallow[i]) * t) * (1.0f + rule->noise * grain);
        rgba[i] = static_cast<uint8_t>(
            (std::min)((std::max)(c, 0.0f), 1.0f) * 255.0f + 0.5f);
    }
    rgba[3] = 255;
    return a;
}

// ---- M9av: the seafloor relief source ---------------------------------------------------------
namespace {

const char* kSeafloorRulesDefault = R"JSON({
  "_": "synth.seafloor.relief -- THE PROGRAM. Hillshade of the ingested bathymetry times a dry sediment ramp. The ramp itself is code (Sources.cpp SeafloorRamp), mirrored in shaders/Compose.hlsli SeafloorRampLuma, which the renderer divides back out to recover the shading through opaque water.",
  "alpha": {"full1": 0.4, "off1": 1.2},
  "sun": {"azimuthDeg": 315.0, "elevationDeg": 45.0},
  "exaggeration": 25.0,
  "ambient": 0.45
}
)JSON";

// Dry sediment albedo by datum depth: sand on the shelf, silt down the slope, clay on the plain.
// MIRRORED in shaders/Compose.hlsli SeafloorRampLuma -- change both, or the renderer's shading
// recovery drifts.
void SeafloorRamp(float depthM, float rgb[3]) {
    static const float kD[5] = {0.0f, 40.0f, 200.0f, 1000.0f, 4000.0f};
    static const float kC[5][3] = {{0.66f, 0.60f, 0.46f}, {0.56f, 0.52f, 0.42f},
                                   {0.46f, 0.44f, 0.39f}, {0.39f, 0.37f, 0.34f},
                                   {0.32f, 0.31f, 0.30f}};
    const float d = (std::min)((std::max)(depthM, 0.0f), 4000.0f);
    int i = 0;
    while (i < 3 && d > kD[i + 1]) ++i;
    const float t = (d - kD[i]) / (kD[i + 1] - kD[i]);
    for (int c = 0; c < 3; ++c) rgb[c] = kC[i][c] + (kC[i + 1][c] - kC[i][c]) * t;
}

}  // namespace

bool SeafloorReliefSource::Load(const std::string& rulesPath, const Compositor* comp,
                                int hgtChannel) {
    m_comp = comp;
    m_hgtCh = hgtChannel;
    if (!comp || hgtChannel < 0) return false;
    if (!std::ifstream(rulesPath)) {
        std::filesystem::create_directories(
            std::filesystem::path(rulesPath).parent_path());
        std::ofstream(rulesPath, std::ios::binary) << kSeafloorRulesDefault;
    }
    std::ifstream f(rulesPath, std::ios::binary);
    if (!f) return false;
    const std::string text((std::istreambuf_iterator<char>(f)),
                           std::istreambuf_iterator<char>());
    std::string err;
    const JsonValue root = JsonParser::Parse(text, &err);
    if (!err.empty()) return false;
    if (const JsonValue* a = root.Get("alpha")) {
        m_full1 = a->Num("full1", 0.4);
        m_off1 = a->Num("off1", 1.2);
    }
    if (const JsonValue* s = root.Get("sun")) {
        m_azDeg = s->Num("azimuthDeg", 315.0);
        m_elDeg = s->Num("elevationDeg", 45.0);
    }
    m_exagg = root.Num("exaggeration", 25.0);
    m_ambient = root.Num("ambient", 0.45);
    // THE IDENTITY IS THE PROGRAM + ITS INPUTS (as synth.bed): the rules, the ramp's version,
    // and the height stack's signature. The product's grain is its finest input's.
    uint64_t h = 14695981039346656037ull;
    auto mix = [&h](const std::string& t) {
        for (const char c : t) h = (h ^ static_cast<uint8_t>(c)) * 1099511628211ull;
    };
    mix(text);
    mix("ramp v1 grad central-at-grain");
    double finest = 1e9;
    for (const HeightSource* hs : m_comp->ChannelAt(m_hgtCh).height) {
        mix(hs->Info().name);
        mix(hs->Info().structure);
        if (hs->Info().cmPerPixel > 0.0) finest = (std::min)(finest, hs->Info().cmPerPixel);
    }
    char structure[96];
    snprintf(structure, sizeof(structure),
             "seafloor-relief (bathy hillshade x sediment ramp) v%08x",
             static_cast<uint32_t>(h & 0xFFFFFFFFu));
    m_info.name = "synth.seafloor.relief";
    m_info.structure = structure;
    m_info.crs = "wgs84.synthesis (height-stack product; seafloor_rules.json -- THE PROGRAM)";
    m_info.cmPerPixel = finest < 1e9 ? finest : 1000.0;
    m_info.lon0 = -180.0;
    m_info.lat0 = -90.0;
    m_info.lon1 = 180.0;
    m_info.lat1 = 90.0;
    return true;
}

float SeafloorReliefSource::Sample(double latRad, double lonRad, double groundResM,
                                   const PaintCtx&, uint8_t rgba[4]) {
    if (!m_comp) return 0.0f;
    const float bed = m_comp->SampleHeightStack(m_hgtCh, latRad, lonRad, groundResM);
    // The waterline band, folded as COVERAGE at coarse texels (M7s, as synth.bed): average the
    // thresholded field, never threshold the average. No deep cutoff: the ocean is all ours.
    auto alphaAt = [&](float b) {
        double av = 1.0;
        if (b > m_full1) av = (m_off1 - b) / (std::max)(m_off1 - m_full1, 1e-6);
        return (std::min)((std::max)(av, 0.0), 1.0);
    };
    float a;
    if (groundResM > 90.0) {
        const int n = (std::min)(4, static_cast<int>(groundResM / 45.0));
        const double dLat = groundResM / 6371000.0 / n;
        const double dLon = dLat / (std::max)(std::cos(latRad), 0.2);
        double acc = 0.0;
        for (int sy = 0; sy < n; ++sy) {
            for (int sx = 0; sx < n; ++sx) {
                const double la = latRad + (sy - (n - 1) * 0.5) * dLat;
                const double lo = lonRad + (sx - (n - 1) * 0.5) * dLon;
                acc += alphaAt(m_comp->SampleHeightStack(m_hgtCh, la, lo, 45.0));
            }
        }
        a = static_cast<float>(acc / (n * n));
    } else {
        a = static_cast<float>(alphaAt(bed));
    }
    if (a <= 0.004f) return 0.0f;
    // The gradient at the texel's own scale (forward differences east and north), exaggerated
    // and lit by the fixed sun. A flat bed shades to exactly 1, so the ramp's luminance is the
    // reference the renderer divides out to get the hillshade back.
    // Central differences at the LARGER of the texel and the data's grain under this point:
    // a step inside a bilinear cell measures the interpolant's facet, not the seabed.
    const double dM = (std::max)((std::max)(groundResM, 8.0),
                                 m_comp->HeightGrainM(m_hgtCh, latRad, lonRad));
    const double dLatR = 0.5 * dM / 6371000.0;
    const double dLonR = 0.5 * dM / (6371000.0 * (std::max)(std::cos(latRad), 0.2));
    const float bE = m_comp->SampleHeightStack(m_hgtCh, latRad, lonRad + dLonR, groundResM);
    const float bW = m_comp->SampleHeightStack(m_hgtCh, latRad, lonRad - dLonR, groundResM);
    const float bN = m_comp->SampleHeightStack(m_hgtCh, latRad + dLatR, lonRad, groundResM);
    const float bS = m_comp->SampleHeightStack(m_hgtCh, latRad - dLatR, lonRad, groundResM);
    double nx = -m_exagg * (bE - bW) / dM, ny = -m_exagg * (bN - bS) / dM, nz = 1.0;
    const double nl = std::sqrt(nx * nx + ny * ny + nz * nz);
    nx /= nl;
    ny /= nl;
    nz /= nl;
    const double az = m_azDeg * kPi / 180.0, el = m_elDeg * kPi / 180.0;
    const double lx = std::cos(el) * std::sin(az), ly = std::cos(el) * std::cos(az),
                 lz = std::sin(el);
    const double ndl = (std::max)(nx * lx + ny * ly + nz * lz, 0.0);
    const double flat = m_ambient + (1.0 - m_ambient) * lz;
    const double shade =
        (std::min)((std::max)((m_ambient + (1.0 - m_ambient) * ndl) / flat, 0.35), 1.6);
    float ramp[3];
    SeafloorRamp((std::max)(-bed, 0.0f), ramp);
    for (int i = 0; i < 3; ++i) {
        const float c = static_cast<float>(ramp[i] * shade);
        rgba[i] = static_cast<uint8_t>(
            (std::min)((std::max)(c, 0.0f), 1.0f) * 255.0f + 0.5f);
    }
    rgba[3] = 255;
    return a;
}

}  // namespace ga
