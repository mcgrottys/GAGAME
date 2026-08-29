#include "compose/Sources.h"

#include "core/TileProviders.h"

#include <algorithm>
#include <cmath>

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

void GoogleColorSource::BeginTile(double latMin, double latMax, double lonMin, double lonMax,
                                  double groundResM, PaintCtx& ctx) {
    if (!m_prov || !m_prov->Ready()) return;
    const int z = ZoomFor(groundResM);
    // Grade normalization: mean color of this zoom vs ONE ABSOLUTE REFERENCE zoom (z10) over
    // a sparse grid of the tile footprint, so every deep zoom shares a single grading and
    // resident-mip boundaries stop being color steps. (Normalizing to the immediate parent
    // would only SHIFT the step down a level -- the reference must be common.) Clamped so a
    // real land-cover change cannot over-drive the correction.
    if (z <= 10) return;
    const int zr = 10;
    double sumC[3] = {0, 0, 0}, sumP[3] = {0, 0, 0};
    int n = 0;
    for (int gy = 0; gy < 5; ++gy) {
        for (int gx = 0; gx < 5; ++gx) {
            const double lat = latMin + (latMax - latMin) * (gy + 0.5) / 5.0;
            const double lon = lonMin + (lonMax - lonMin) * (gx + 0.5) / 5.0;
            uint8_t c[3], p[3];
            if (!Pixel(z, lat, lon, c) || !Pixel(zr, lat, lon, p)) continue;
            for (int k = 0; k < 3; ++k) {
                sumC[k] += c[k];
                sumP[k] += p[k];
            }
            ++n;
        }
    }
    if (n < 8) return;
    for (int k = 0; k < 3; ++k) {
        ctx.gain[k] = static_cast<float>(
            std::clamp((sumP[k] + 4.0) / (sumC[k] + 4.0), 0.75, 1.35));
    }
}

float GoogleColorSource::Sample(double latRad, double lonRad, double groundResM,
                                const PaintCtx& ctx, uint8_t rgba[4]) {
    if (!m_prov || !m_prov->Ready()) return 0.0f;
    uint8_t rgb[3];
    if (!Pixel(ZoomFor(groundResM), latRad, lonRad, rgb)) {
        return -1.0f;   // TRANSIENT: coverage exists, fetch failed -- do not cache
    }
    for (int k = 0; k < 3; ++k) {
        rgba[k] = static_cast<uint8_t>(
            (std::min)(255.0f, static_cast<float>(rgb[k]) * ctx.gain[k]));
    }
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

// ------------------------------------------------------------------------------ CUDEM

CudemHeightSource::CudemHeightSource(const BathyModel* bathy, double featherFrac)
    : m_bathy(bathy), m_feather(featherFrac) {
    const double lon0 = BathyModel::kOrgLon + bathy->WorldX0() / BathyModel::kMPerLon;
    const double lat0 = BathyModel::kOrgLat + bathy->WorldZ0() / BathyModel::kMPerLat;
    m_info = {"noaa.cudem.merrimack",
              "geotiff-window float32 (thalweg-preserving resample)",
              "local tangent metres @ ACT0816 (from EPSG:4326 GeoTIFF)", 1370.0, lon0, lat0,
              lon0 + bathy->WorldSizeX() / BathyModel::kMPerLon,
              lat0 + bathy->WorldSizeZ() / BathyModel::kMPerLat};
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
