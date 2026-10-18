#include <cstdint>
#include "compose/Sources.h"
#include "core/Common.h"

#include "core/Json.h"
#include "core/TileProviders.h"


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

GoogleColorSource::GoogleColorSource(GoogleTileProvider* prov, int zoomCap)
    : m_prov(prov), m_zoomCap(std::clamp(zoomCap, 0, kMaxZoom)) {
    // The structure is half the source's identity (name|structure, ColorStackSource.h): at the
    // default it is today's string exactly; at any other cap it names the cap, so a scene that
    // raises the zoom paints its own tree. cmPerPixel is the cap's grain at the equator (955 at
    // z14, halved a step finer); only the registry log and the fidelity map read it.
    std::string structure = "mercator-tile-tree jpeg 256px (sessioned, cache-first)";
    if (m_zoomCap != kDefaultZoom) structure += ", to z" + std::to_string(m_zoomCap);
    m_info = {"google.satellite", structure, "EPSG:3857 web-mercator",
              std::ldexp(955.0, kDefaultZoom - m_zoomCap), -180, -85, 180, 85};
}

// Zoom from the requested footprint: z such that Mercator metres/px matches groundResM.
// Capped at the scene's streaming.googleZoom (14 unless a scene says otherwise) -- deeper
// zooms are a budget decision a REALIZATION makes by asking for a finer groundRes only inside
// a window it owns; the global cube can never demand them, and the day's ledger bounds the count.
int GoogleColorSource::ZoomFor(double groundResM) const {
    constexpr double kCirc = 40075016.686;
    return std::clamp(static_cast<int>(std::lround(std::log2(kCirc / (256.0 * groundResM)))),
                      0, m_zoomCap);
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

// TILE AT A TIME: Pixel's arithmetic texel for texel, but each Google tile the paint meets is asked
// of the provider once -- a texel at a time, every one took the provider's lock, a map lookup and a
// shared count, for the one to four tiles a paint reads. A tile not ready answers its texels as
// Sample does (not whole, nothing written).
bool GoogleColorSource::PaintTile(const ColorFrame& frame, const TileRequest& r, double groundResM, PaintCtx&,
                                  std::vector<uint8_t>& out, bool& complete, bool& anyCover, bool& fullCover) {
    if (!m_prov || !m_prov->Ready()) return false;
    const int z = ZoomFor(groundResM);
    const double n = static_cast<double>(1u << z) * 256.0;
    struct Held {
        int tx, ty;
        std::shared_ptr<std::vector<uint8_t>> tile;
    };
    std::vector<Held> held;
    size_t last = 0;
    out.assign(65536, 0);
    complete = true;
    anyCover = false;
    fullCover = true;
    for (uint32_t py = 0; py < frame.texH; ++py) {
        for (uint32_t px = 0; px < frame.texW; ++px) {
            double latRad = 0, lonRad = 0;
            frame.Texel(r, px, py, latRad, lonRad);
            const double latC = std::clamp(latRad, -1.4844, 1.4844);   // as Pixel
            const double mx = (lonRad / kPi * 0.5 + 0.5) * n;
            const double my = (0.5 - std::log(std::tan(kPi * 0.25 + latC * 0.5)) / (2.0 * kPi)) * n;
            const int gx = std::clamp(static_cast<int>(mx), 0, static_cast<int>(n) - 1);
            const int gy = std::clamp(static_cast<int>(my), 0, static_cast<int>(n) - 1);
            const int tx = gx / 256, ty = gy / 256;
            if (last >= held.size() || held[last].tx != tx || held[last].ty != ty) {
                last = held.size();
                for (size_t k = 0; k < held.size(); ++k) {
                    if (held[k].tx == tx && held[k].ty == ty) { last = k; break; }
                }
                if (last == held.size()) held.push_back({tx, ty, m_prov->Decoded(z, tx, ty)});
            }
            const std::vector<uint8_t>* tile = held[last].tile.get();
            if (!tile) {   // TRANSIENT, as Sample's -1: coverage exists, the fetch is not here
                complete = false;
                fullCover = false;
                continue;
            }
            const uint8_t* s = &(*tile)[((gy & 255) * 256 + (gx & 255)) * 4];
            uint8_t* d = &out[(size_t(py) * frame.texW + px) * 4];
            d[0] = s[0];
            d[1] = s[1];
            d[2] = s[2];
            d[3] = 255;   // weight 1, quantized as the per-texel loop does
            anyCover = true;
        }
    }
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

int GoogleColorSource::FinestMip(const Lattice& l, double lat0, double lat1, double lon0,
                                 double lon1) const {
    constexpr double kCirc = 40075016.686;
    const double lat = 0.5 * (lat0 + lat1);
    return l.LevelOf(std::ldexp(kCirc / 256.0, -m_zoomCap) * std::cos(lat), lat, 0.5 * (lon0 + lon1));
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

HeightSource::Range EquirectHeightSource::RangeIn(double lat0, double lon0, double lat1, double lon1, float& lo,
                                                  float& hi, bool& full) {
    full = false;
    if (!m_elev || m_nx <= 0) return Range::None;
    // Sample's own texel coordinates (Bilinear subtracts the half): u * nx, v * ny
    const double fx0 = (lon0 / 360.0 + 0.5) * m_nx - 0.5, fx1 = (lon1 / 360.0 + 0.5) * m_nx - 0.5;
    const double fy0 = std::clamp(0.5 - lat1 / 180.0, 0.0, 1.0) * m_ny - 0.5,
                 fy1 = std::clamp(0.5 - lat0 / 180.0, 0.0, 1.0) * m_ny - 0.5;
    const int x0 = int(std::floor(fx0)), x1 = int(std::floor(fx1)) + 1;
    const int y0 = std::clamp(int(std::floor(fy0)), 0, m_ny - 1), y1 = std::clamp(int(std::floor(fy1)) + 1, 0, m_ny - 1);
    if (double(x1 - x0 + 1) * double(y1 - y0 + 1) > 1048576.0) return Range::Unknown;
    int16_t mn = INT16_MAX, mx = INT16_MIN;
    for (int y = y0; y <= y1; ++y) {
        for (int x = x0; x <= x1; ++x) {
            const int xw = ((x % m_nx) + m_nx) % m_nx;
            const int16_t v = (*m_elev)[size_t(y) * size_t(m_nx) + size_t(xw)];
            mn = (std::min)(mn, v);
            mx = (std::max)(mx, v);
        }
    }
    lo = float(mn) + float(m_datumShift);
    hi = float(mx) + float(m_datumShift);
    full = true;
    return Range::Known;
}

// THE IDENTITY OF WHAT A BUILT-IN LAYER PAINTS (2026-10-18). A layer by file carries its bytes in its
// structure string and the tree keys its tiles by them (HeightLayerSource::Identity); these two read
// grids the engine loaded for other reasons -- the globe's ETOPO, the survey's CUDEM -- and carried
// only their names, so a regenerated grid, or another station's datum link, would have met its old
// tiles in the cache. The law is the bytes and the numbers the paint depends on: the grid's FNV-64
// and size, the datum link, the box and the feather. The paint's own FORM (the texel centre, the fold
// of a parent from its children) is kTileTreeVersion's, which TileTree folds into every tree's name;
// a layer does not repeat it. The hash is taken once, at the first ask, which comes when the tree is
// built -- after the grid is loaded -- and costs the bytes once (ETOPO's 64 MB, some 50 ms).
std::string EquirectHeightSource::Law() const {
    if (!m_elev || m_elev->empty()) return std::string();
    if (!m_hashed) {
        m_bytesFnv = Fnv1aBytes(m_elev->data(), m_elev->size() * sizeof(int16_t));
        m_hashed = true;
    }
    char b[128];
    snprintf(b, sizeof(b), "grid int16 %dx%d fnv64 %016llx, vdatum %+.3f m", m_nx, m_ny,
             static_cast<unsigned long long>(m_bytesFnv), m_datumShift);
    return b;
}

// ------------------------------------------------------------------------------ CUDEM

std::string CudemHeightSource::Law() const {
    if (!m_bathy || !m_bathy->Ready()) return std::string();
    if (!m_hashed) {
        const std::vector<float>& e = m_bathy->Elev();
        m_bytesFnv = Fnv1aBytes(e.data(), e.size() * sizeof(float));
        m_hashed = true;
    }
    char b[192];
    snprintf(b, sizeof(b), "grid float32 %dx%d fnv64 %016llx at %.6f,%.6f..%.6f,%.6f, feather %.3f",
             m_bathy->Nx(), m_bathy->Ny(), static_cast<unsigned long long>(m_bytesFnv), m_bathy->Lon0(),
             m_bathy->Lat0(), m_bathy->Lon1(), m_bathy->Lat1(), m_feather);
    return b;
}

CudemHeightSource::CudemHeightSource(const BathyModel* bathy, double featherFrac,
                                     const char* name)
    : m_bathy(bathy), m_feather(featherFrac) {
    m_info = {name,
              "geotiff-window float32 (thalweg-preserving resample)",
              "EPSG:4326 grid (the harvester's resample of its GeoTIFF)", 1370.0, bathy->Lon0(),
              bathy->Lat0(), bathy->Lon1(), bathy->Lat1()};
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

float EditsHeightSource::Sample(double latRad, double lonRad, double groundResM, float& metres) {
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
            return Uncovered(latRad, lonRad, groundResM);
        }
    }
    return 0.0f;
}

// THE HAND EDIT YIELDS TO A FINER WITNESS (2026-10-18). A polygon's crest is a stand-in for ground the
// surveys never measured: OSM's jetty ways over CUDEM's 10 m cells, where the laser and the sounder
// have no value. Measured at the north jetty with the transect tool, the edit painted its +2.5 m over
// the 1 m lidar's +2.77 m and the 1.5 m composite's +3.03 m on the same crest, a flat slab in every
// still. So the edit's weight at a point is the share the finer witnesses leave uncovered:
//
//     w = prod_j (1 - w_j)   over the channel's layers with a grain finer than this one's (5 m),
//
// the complement of their over-accumulated coverage -- one law at every grain, since each w_j at a
// coarse grain is already that source's share of the box. Its limits: no finer layer (or no channel
// named) = 1, the value the edit always had; a finer layer at full weight = 0, the edit gone and the
// data standing; across a finer layer's feather the edit fades as the witness rises, the same metres.
// The VALUE is never touched, only the weight: a floor would hold the lidar's slopes at the crest, and
// the slopes are the data. Coarser layers (CUDEM, ETOPO) are not asked: the edit exists to stand over
// them. One source asking the others for their weight is the gate's shape (GateSource) read inside
// the layer, so the CPU's stack, the tree's point sample and its tile compose inherit it from this
// one Sample and cannot drift.
float EditsHeightSource::Uncovered(double latRad, double lonRad, double groundResM) const {
    if (!m_comp || m_channel < 0) return 1.0f;
    const double lon = lonRad * 180.0 / kPi, lat = latRad * 180.0 / kPi;
    double u = 1.0;
    for (HeightSource* hs : m_comp->ChannelAt(m_channel).height) {
        if (hs == this) continue;
        const SourceInfo& s = hs->Info();
        if (s.cmPerPixel <= 0.0 || s.cmPerPixel >= m_info.cmPerPixel) continue;
        if (lon < s.lon0 || lon > s.lon1 || lat < s.lat0 || lat > s.lat1) continue;
        float m = 0.0f;
        const float w = hs->Sample(latRad, lonRad, groundResM, m);
        if (w > 0.0f) u *= 1.0 - (std::min)(1.0, double(w));
        if (u <= 0.0) return 0.0f;
    }
    return float(u);
}

// The identity of what this layer paints: its own content (the polygons' bytes and the crest, which
// the structure string hashes) and the finer witnesses it yields to, by name and structure, so the
// tree repaints the edits' tiles when the polygons OR a witness changes, and at no other time.
std::string EditsHeightSource::Law() const {
    std::string law = m_info.structure;
    if (!m_comp || m_channel < 0) return law;
    bool first = true;
    for (HeightSource* hs : m_comp->ChannelAt(m_channel).height) {
        if (hs == this) continue;
        const SourceInfo& s = hs->Info();
        if (s.cmPerPixel <= 0.0 || s.cmPerPixel >= m_info.cmPerPixel) continue;
        law += (first ? "|yields<" : ",") + s.name + "|" + s.structure;
        first = false;
    }
    return law;
}

HeightSource::Range EditsHeightSource::RangeIn(double lat0, double lon0, double lat1, double lon1, float& lo,
                                              float& hi, bool& full) {
    full = false;
    bool any = false;
    for (const Ring& r : m_rings) {
        if (lon1 < r.lon0 || lon0 > r.lon1 || lat1 < r.lat0 || lat0 > r.lat1) continue;
        const float v = r.land ? m_crest : -1.0f;
        lo = any ? (std::min)(lo, v) : v;
        hi = any ? (std::max)(hi, v) : v;
        any = true;
    }
    return any ? Range::Known : Range::None;
}

HeightSource::Range CudemHeightSource::RangeIn(double lat0, double lon0, double lat1, double lon1, float& lo,
                                               float& hi, bool& full) {
    full = false;
    if (!m_bathy || !m_bathy->Ready()) return Range::None;
    const int nx = m_bathy->Nx(), ny = m_bathy->Ny();
    const int x0 = (std::max)(0, int(std::floor((lon0 - m_bathy->Lon0()) / m_bathy->Dlon() - 0.5))),
              x1 = (std::min)(nx - 1, int(std::floor((lon1 - m_bathy->Lon0()) / m_bathy->Dlon() - 0.5)) + 1);
    const int y0 = (std::max)(0, int(std::floor((m_bathy->Lat1() - lat1) / m_bathy->Dlat() - 0.5))),
              y1 = (std::min)(ny - 1, int(std::floor((m_bathy->Lat1() - lat0) / m_bathy->Dlat() - 0.5)) + 1);
    if (x0 > x1 || y0 > y1) return Range::None;
    if (double(x1 - x0 + 1) * double(y1 - y0 + 1) > 1048576.0) return Range::Unknown;   // the march is cheaper
    const std::vector<float>& e = m_bathy->Elev();
    bool any = false;
    for (int y = y0; y <= y1; ++y) {
        for (int x = x0; x <= x1; ++x) {
            const float v = e[size_t(y) * size_t(nx) + size_t(x)];
            if (v < -9000.0f) continue;
            lo = any ? (std::min)(lo, v) : v;
            hi = any ? (std::max)(hi, v) : v;
            any = true;
        }
    }
    return any ? Range::Known : Range::None;
}

float CudemHeightSource::Sample(double latRad, double lonRad, double, float& metres) {
    if (!m_bathy || !m_bathy->Ready()) return 0.0f;
    const double lon = lonRad * 180.0 / kPi, lat = latRad * 180.0 / kPi;
    const float v = m_bathy->SampleLatLon(lat, lon);
    if (v < -9000.0f) return 0.0f;   // outside the grid, or nodata
    metres = v;
    return EdgeWeight(lon, lat);
}

// PHASE C5: the feather in the survey's own fractions of its extent, by lon and lat.
float CudemHeightSource::EdgeWeight(double lon, double lat) const {
    const double sx = m_bathy->Lon1() - m_bathy->Lon0(), sz = m_bathy->Lat1() - m_bathy->Lat0();
    const double ex = (std::min)(lon - m_bathy->Lon0(), m_bathy->Lon1() - lon) / sx;
    const double ez = (std::min)(lat - m_bathy->Lat0(), m_bathy->Lat1() - lat) / sz;
    return Feather((std::min)(ex, ez), m_feather);
}

bool CudemHeightSource::FullWeightCells(int& c0, int& r0, int& c1, int& r1) const {
    if (!m_bathy || !m_bathy->Ready()) return false;
    const int nx = m_bathy->Nx(), ny = m_bathy->Ny();
    // A cell's centre on the lattice, in the world frame Sample puts a sample point in (the
    // linear lon/lat map both frames were built with); the other axis held at the window's
    // middle, where only the axis being walked can bring the weight under one.
    auto x = [&](int c) { return m_bathy->Lon0() + (c + 0.5) * m_bathy->Dlon(); };
    auto z = [&](int r) { return m_bathy->Lat1() - (r + 0.5) * m_bathy->Dlat(); };
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
  "bounds": {"lon0": -180, "lat0": -90, "lon1": 180, "lat1": 90},
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
        m_info.lon0 = b->Num("lon0", -180.0); m_info.lat0 = b->Num("lat0", -90.0);
        m_info.lon1 = b->Num("lon1", 180.0); m_info.lat1 = b->Num("lat1", 90.0);
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
    // it reads the height TREE when one is bound (compose/HeightTree.h): another reading, another tile
    if (m_comp->HeightTreeOf(m_hgtCh)) mix("height tree v2");
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

int BedSynthSource::FinestMip(const Lattice& l, double lat0, double lat1, double lon0,
                              double lon1) const {
    if (!m_comp) return -1;
    for (const Zone& z : m_zones) {
        if (!(z.lon1 * kPi / 180.0 < lon0 || z.lon0 * kPi / 180.0 > lon1 || z.lat1 * kPi / 180.0 < lat0 ||
              z.lat0 * kPi / 180.0 > lat1)) {
            return -1;
        }
    }
    const double lat = 0.5 * (lat0 + lat1), lon = 0.5 * (lon0 + lon1);
    return l.LevelOf(m_comp->HeightGrainM(m_hgtCh, lat, lon), lat, lon);
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
    const float bed = m_comp->SampleHeight(m_hgtCh, latRad, lonRad, groundResM);
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
                acc += alphaAt(m_comp->SampleHeight(m_hgtCh, la, lo, 45.0));
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
    const float bE = m_comp->SampleHeight(m_hgtCh, latRad, lonRad + dLonR, groundResM);
    const float bN = m_comp->SampleHeight(m_hgtCh, latRad + dLatR, lonRad, groundResM);
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
// MIRRORED in shaders/Compose.hlsli SeafloorRampLuma -- change both.
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
    if (m_comp->HeightTreeOf(m_hgtCh)) mix("height tree v2");   // as synth.bed
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

int SeafloorReliefSource::FinestMip(const Lattice& l, double lat0, double lat1, double lon0,
                                    double lon1) const {
    if (!m_comp) return -1;
    const double lat = 0.5 * (lat0 + lat1), lon = 0.5 * (lon0 + lon1);
    return l.LevelOf(m_comp->HeightGrainM(m_hgtCh, lat, lon), lat, lon);
}

float SeafloorReliefSource::Sample(double latRad, double lonRad, double groundResM,
                                   const PaintCtx&, uint8_t rgba[4]) {
    if (!m_comp) return 0.0f;
    const float bed = m_comp->SampleHeight(m_hgtCh, latRad, lonRad, groundResM);
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
                acc += alphaAt(m_comp->SampleHeight(m_hgtCh, la, lo, 45.0));
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
    const float bE = m_comp->SampleHeight(m_hgtCh, latRad, lonRad + dLonR, groundResM);
    const float bW = m_comp->SampleHeight(m_hgtCh, latRad, lonRad - dLonR, groundResM);
    const float bN = m_comp->SampleHeight(m_hgtCh, latRad + dLatR, lonRad, groundResM);
    const float bS = m_comp->SampleHeight(m_hgtCh, latRad - dLatR, lonRad, groundResM);
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

// ---- THE CAPTURE LEVEL (Sources.h LevelGateSource) --------------------------------------------

void LevelGateSource::Load(const ColorSource& layer, const Compositor* comp, int hgtChannel,
                           double levelM, const std::string& said) {
    m_comp = comp;
    m_hgtCh = hgtChannel;
    m_level = levelM;
    const SourceInfo& li = layer.Info();
    // THE IDENTITY FOLDS THE HEIGHT STACK'S: every height layer's name|structure in the channel's
    // order -- what the height tree's own identity is made of -- so an edited bed (a new survey, a
    // moved hand edit) repaints the gated tiles, and a bed unchanged keeps them.
    uint64_t h = 14695981039346656037ull;
    if (comp && hgtChannel >= 0) {
        for (const HeightSource* hs : comp->ChannelAt(hgtChannel).height) {
            const std::string id = hs->Info().name + "|" + hs->Info().structure + ";";
            for (unsigned char c : id) h = (h ^ c) * 1099511628211ull;
        }
    }
    char b[256];
    snprintf(b, sizeof(b),
             "ground above %.4f m NAVD88 (%s): 1 above, 0 below, smoothstep across the height's own "
             "spread; height stack fnv64 %016llx",
             levelM, said.c_str(), static_cast<unsigned long long>(h));
    m_info = {li.name + ".ground", b, "EPSG:4326 (a product of the height tree)", li.cmPerPixel,
              li.lon0, li.lat0, li.lon1, li.lat1};
    Log("[capture] %s gates %s: %s", m_info.name.c_str(), li.name.c_str(), b);
}

float LevelGateSource::Weight(float h, float level, float s) {
    // ONE LAW: the smoothstep of (h - level) over [-s, s]. s -> 0 is the step, and at the level
    // itself a half either way -- the step's own middle, the limit from both sides.
    const double d = static_cast<double>(h) - static_cast<double>(level);
    const double t = s > 0.0f ? std::clamp(0.5 + 0.5 * d / static_cast<double>(s), 0.0, 1.0)
                              : (d > 0.0 ? 1.0 : d < 0.0 ? 0.0 : 0.5);
    return static_cast<float>(t * t * (3.0 - 2.0 * t));
}

float LevelGateSource::Sample(double latRad, double lonRad, double groundResM, const PaintCtx&,
                              uint8_t rgba[4]) {
    if (!m_comp || m_hgtCh < 0) return 0.0f;   // no height to read: no opinion, the layer passes
    const double lon = lonRad * 180.0 / kPi, lat = latRad * 180.0 / kPi;
    if (lon < m_info.lon0 || lon > m_info.lon1 || lat < m_info.lat0 || lat > m_info.lat1) {
        return 0.0f;   // outside the layer: no opinion
    }
    float s = 0.0f;
    const float h = m_comp->SampleHeight(m_hgtCh, latRad, lonRad, groundResM, &s);
    const uint8_t v = static_cast<uint8_t>(Weight(h, static_cast<float>(m_level), s) * 255.0f + 0.5f);
    // The mask's encoding (GisMask.cpp): the weight says an opinion exists, r carries the factor;
    // the gate multiplies by the value (ComposeTree.h GateSource; TileTree GateTile per texel).
    rgba[0] = v;
    rgba[1] = v;
    rgba[2] = v;
    rgba[3] = 255;
    return 1.0f;
}

}  // namespace ga
