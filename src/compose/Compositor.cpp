#include "compose/Compositor.h"

#include <windows.h>

#include <cmath>
#include <cstdio>
#include <fstream>

namespace ga {

namespace {

constexpr double kPi = 3.14159265358979;
constexpr double kMercCirc = 40075016.686;   // Web-Mercator world metres (equator)
constexpr int kComposeVersion = 2;           // bump on any paint-math change: new cache tag
                                             // (v2: per-tile grade normalization)

// Changing a stack (order, membership, version) must never serve stale composed tiles: the
// hash lands in the cache directory name, so an edit simply starts a fresh folder.
uint64_t Fnv1a(uint64_t h, const std::string& s) {
    for (const char c : s) {
        h ^= static_cast<uint8_t>(c);
        h *= 1099511628211ull;
    }
    return h;
}

uint16_t FloatToHalf(float f) {
    const uint32_t x = *reinterpret_cast<const uint32_t*>(&f);
    const uint32_t sign = (x >> 16) & 0x8000u;
    int32_t e = static_cast<int32_t>((x >> 23) & 0xFF) - 127 + 15;
    uint32_t m = (x >> 13) & 0x3FFu;
    if (e <= 0) return static_cast<uint16_t>(sign);
    if (e >= 31) return static_cast<uint16_t>(sign | 0x7BFFu);
    return static_cast<uint16_t>(sign | (static_cast<uint32_t>(e) << 10) | m);
}

}  // namespace

void ComposeCubeDir(uint32_t face, double u, double v, double out[3]) {
    const double s = u * 2.0 - 1.0, t = v * 2.0 - 1.0;
    double p[3];
    switch (face) {
        case 0: p[0] = 1;  p[1] = -t; p[2] = -s; break;
        case 1: p[0] = -1; p[1] = -t; p[2] = s;  break;
        case 2: p[0] = s;  p[1] = 1;  p[2] = t;  break;
        case 3: p[0] = s;  p[1] = -1; p[2] = -t; break;
        case 4: p[0] = s;  p[1] = -t; p[2] = 1;  break;
        default: p[0] = -s; p[1] = -t; p[2] = -1; break;
    }
    const double l = std::sqrt(p[0] * p[0] + p[1] * p[1] + p[2] * p[2]);
    out[0] = p[0] / l;
    out[1] = p[1] / l;
    out[2] = p[2] / l;
}

int Compositor::AddColorChannel(const std::string& name, std::vector<ColorSource*> stack) {
    Channel ch;
    ch.name = name;
    ch.color = std::move(stack);
    uint64_t h = Fnv1a(14695981039346656037ull, name + "#" + std::to_string(kComposeVersion));
    for (const auto* s : ch.color) h = Fnv1a(h, s->Info().name + "|" + s->Info().structure);
    ch.stackHash = h;
    m_channels.push_back(std::move(ch));
    return static_cast<int>(m_channels.size()) - 1;
}

int Compositor::AddHeightChannel(const std::string& name, std::vector<HeightSource*> stack) {
    Channel ch;
    ch.name = name;
    ch.height = std::move(stack);
    uint64_t h = Fnv1a(14695981039346656037ull, name + "#" + std::to_string(kComposeVersion));
    for (const auto* s : ch.height) h = Fnv1a(h, s->Info().name + "|" + s->Info().structure);
    ch.stackHash = h;
    m_channels.push_back(std::move(ch));
    return static_cast<int>(m_channels.size()) - 1;
}

std::string Compositor::CachePath(const Channel& ch, const char* realization,
                                  const TileRequest& r) const {
    char buf[320];
    snprintf(buf, sizeof(buf), "cache\\composed\\%s\\%s_v%08x\\f%u_m%u_x%u_y%u.bin",
             ch.name.c_str(), realization, static_cast<uint32_t>(ch.stackHash & 0xFFFFFFFFu),
             r.face, r.mip, r.x, r.y);
    return buf;
}

bool Compositor::ReadCached(const std::string& path, std::vector<uint8_t>& out) {
    std::ifstream f(path, std::ios::binary);
    if (!f) return false;
    out.assign((std::istreambuf_iterator<char>(f)), std::istreambuf_iterator<char>());
    if (out.size() != 65536) {
        out.clear();
        return false;   // torn write (crash mid-save): repaint it
    }
    ++cacheHits;
    return true;
}

void Compositor::WriteCached(const std::string& path, const std::vector<uint8_t>& data) {
    std::ofstream f(path, std::ios::binary);
    f.write(reinterpret_cast<const char*>(data.data()), data.size());
}

void Compositor::EnsureCacheDir(const Channel& ch, const char* realization) {
    char buf[320];
    CreateDirectoryA("cache", nullptr);
    CreateDirectoryA("cache\\composed", nullptr);
    snprintf(buf, sizeof(buf), "cache\\composed\\%s", ch.name.c_str());
    CreateDirectoryA(buf, nullptr);
    snprintf(buf, sizeof(buf), "cache\\composed\\%s\\%s_v%08x", ch.name.c_str(), realization,
             static_cast<uint32_t>(ch.stackHash & 0xFFFFFFFFu));
    CreateDirectoryA(buf, nullptr);
}

// ---- realizations --------------------------------------------------------------------------

TileProviderFn Compositor::CubeColor(int channel) {
    EnsureCacheDir(m_channels[channel], "cube16k");
    return [this, channel](const TileRequest& r, std::vector<uint8_t>& out) {
        const Channel& ch = m_channels[channel];
        const std::string path = CachePath(ch, "cube16k", r);
        if (ReadCached(path, out)) return true;

        const uint32_t faceTexels = kFaceDim >> r.mip;
        const double invFace = 1.0 / faceTexels;
        const double groundRes = kMercCirc / (4.0 * faceTexels);
        // Tile lat/lon box (corners + centre: a gnomonic tile's extremes live on its edge
        // midpoints only at the poles, where a loose box is harmless) for BeginTile.
        double latMin = 10, latMax = -10, lonMin = 10, lonMax = -10;
        for (int cy = 0; cy < 3; ++cy) {
            for (int cx = 0; cx < 3; ++cx) {
                double d[3];
                ComposeCubeDir(r.face, (r.x * 128.0 + cx * 64.0) * invFace,
                               (r.y * 128.0 + cy * 64.0) * invFace, d);
                const double la = std::asin((std::max)(-1.0, (std::min)(1.0, d[1])));
                const double lo = std::atan2(d[2], d[0]);
                latMin = (std::min)(latMin, la);
                latMax = (std::max)(latMax, la);
                lonMin = (std::min)(lonMin, lo);
                lonMax = (std::max)(lonMax, lo);
            }
        }
        std::vector<PaintCtx> ctxs(ch.color.size());
        for (size_t i = 0; i < ch.color.size(); ++i) {
            ch.color[i]->BeginTile(latMin, latMax, lonMin, lonMax, groundRes, ctxs[i]);
        }
        out.assign(65536, 0);
        bool complete = true;   // a transient source failure (fetch budget/network) shows as
                                // a hole THIS run but is never cached: the next run repaints
        for (uint32_t py = 0; py < 128; ++py) {
            for (uint32_t px = 0; px < 128; ++px) {
                const double u = (r.x * 128.0 + px + 0.5) * invFace;
                const double v = (r.y * 128.0 + py + 0.5) * invFace;
                double d[3];
                ComposeCubeDir(r.face, u, v, d);
                const double lat = std::asin((std::max)(-1.0, (std::min)(1.0, d[1])));
                const double lon = std::atan2(d[2], d[0]);
                uint8_t* dst = &out[(py * 128 + px) * 4];
                float acc[3] = {0, 0, 0};
                float cover = 0.0f;
                for (size_t i = 0; i < ch.color.size(); ++i) {   // bottom -> top
                    uint8_t rgba[4];
                    const float w = ch.color[i]->Sample(lat, lon, groundRes, ctxs[i], rgba);
                    if (w < 0.0f) { complete = false; continue; }
                    if (w == 0.0f) continue;
                    for (int c = 0; c < 3; ++c) acc[c] += (rgba[c] - acc[c]) * w;
                    cover = (std::max)(cover, w);
                }
                dst[0] = static_cast<uint8_t>(acc[0]);
                dst[1] = static_cast<uint8_t>(acc[1]);
                dst[2] = static_cast<uint8_t>(acc[2]);
                dst[3] = cover > 0.0f ? 255 : 0;
            }
        }
        if (complete) {
            WriteCached(path, out);
            ++painted;
        }
        return true;
    };
}

TileProviderFn Compositor::WindowColor(int channel, long long orgPxX, long long orgPxY,
                                       uint32_t sizePx, int zBase) {
    EnsureCacheDir(m_channels[channel], "window");
    (void)sizePx;
    return [this, channel, orgPxX, orgPxY, zBase](const TileRequest& r,
                                                  std::vector<uint8_t>& out) {
        const Channel& ch = m_channels[channel];
        const std::string path = CachePath(ch, "window", r);
        if (ReadCached(path, out)) return true;

        // Window texels ARE Mercator pixels of zoom (zBase - mip): the lat/lon roundtrip
        // through a Mercator-tree source lands back on the same pixel, so fills remain the
        // straight pixel moves the M6f DetailFn did -- now with the whole stack above them.
        const double worldPx = static_cast<double>((1ll << zBase) * 256ll >> r.mip);
        const double groundRes = kMercCirc / worldPx;
        const long long gx0 = (orgPxX >> r.mip) + static_cast<long long>(r.x) * 128;
        const long long gy0 = (orgPxY >> r.mip) + static_cast<long long>(r.y) * 128;
        const double lat0 = std::atan(std::sinh(kPi * (1.0 - 2.0 * (gy0 + 128.0) / worldPx)));
        const double lat1 = std::atan(std::sinh(kPi * (1.0 - 2.0 * gy0 / worldPx)));
        const double lon0 = (gx0 / worldPx - 0.5) * 2.0 * kPi;
        const double lon1 = ((gx0 + 128.0) / worldPx - 0.5) * 2.0 * kPi;
        std::vector<PaintCtx> ctxs(ch.color.size());
        for (size_t i = 0; i < ch.color.size(); ++i) {
            ch.color[i]->BeginTile(lat0, lat1, lon0, lon1, groundRes, ctxs[i]);
        }
        out.assign(65536, 0);
        bool complete = true;
        for (uint32_t py = 0; py < 128; ++py) {
            const double Y = (gy0 + py + 0.5) / worldPx;
            const double lat = std::atan(std::sinh(kPi * (1.0 - 2.0 * Y)));
            for (uint32_t px = 0; px < 128; ++px) {
                const double X = (gx0 + px + 0.5) / worldPx;
                const double lon = (X - 0.5) * 2.0 * kPi;
                uint8_t* dst = &out[(py * 128 + px) * 4];
                float acc[3] = {0, 0, 0};
                float cover = 0.0f;
                for (size_t i = 0; i < ch.color.size(); ++i) {
                    uint8_t rgba[4];
                    const float w = ch.color[i]->Sample(lat, lon, groundRes, ctxs[i], rgba);
                    if (w < 0.0f) { complete = false; continue; }
                    if (w == 0.0f) continue;
                    for (int c = 0; c < 3; ++c) acc[c] += (rgba[c] - acc[c]) * w;
                    cover = (std::max)(cover, w);
                }
                dst[0] = static_cast<uint8_t>(acc[0]);
                dst[1] = static_cast<uint8_t>(acc[1]);
                dst[2] = static_cast<uint8_t>(acc[2]);
                dst[3] = cover > 0.0f ? 255 : 0;
            }
        }
        if (complete) {
            WriteCached(path, out);
            ++painted;
        }
        return true;
    };
}

TileProviderFn Compositor::CubeHeight(int channel) {
    EnsureCacheDir(m_channels[channel], "cube16k");
    return [this, channel](const TileRequest& r, std::vector<uint8_t>& out) {
        const Channel& ch = m_channels[channel];
        const std::string path = CachePath(ch, "cube16k", r);
        if (ReadCached(path, out)) return true;

        // R16F: 256x128 texels per 64KB tile. Every mip is painted straight from the sources
        // at its own ground resolution (independent paints; trilinear blends cousins -- close
        // enough for display relief, and each level is honest about its own footprint).
        const uint32_t faceTexels = kFaceDim >> r.mip;
        const double invFace = 1.0 / faceTexels;
        const double groundRes = kMercCirc / (4.0 * faceTexels);
        out.assign(65536, 0);
        uint16_t* dst16 = reinterpret_cast<uint16_t*>(out.data());
        for (uint32_t py = 0; py < 128; ++py) {
            for (uint32_t px = 0; px < 256; ++px) {
                const double u = (r.x * 256.0 + px + 0.5) * invFace;
                const double v = (r.y * 128.0 + py + 0.5) * invFace;
                double d[3];
                ComposeCubeDir(r.face, u, v, d);
                const double lat = std::asin((std::max)(-1.0, (std::min)(1.0, d[1])));
                const double lon = std::atan2(d[2], d[0]);
                float h = 0.0f;
                for (HeightSource* s : ch.height) {   // bottom -> top
                    float m = 0.0f;
                    const float w = s->Sample(lat, lon, groundRes, m);
                    if (w <= 0.0f) continue;
                    h += (m - h) * w;
                }
                dst16[py * 256 + px] = FloatToHalf(h);
            }
        }
        WriteCached(path, out);
        ++painted;
        return true;
    };
}

TileProviderFn Compositor::WindowHeight(int channel, long long orgPxX, long long orgPxY,
                                        uint32_t sizePx, int zBase) {
    EnsureCacheDir(m_channels[channel], "windowH");
    (void)sizePx;
    return [this, channel, orgPxX, orgPxY, zBase](const TileRequest& r,
                                                  std::vector<uint8_t>& out) {
        const Channel& ch = m_channels[channel];
        const std::string path = CachePath(ch, "windowH", r);
        if (ReadCached(path, out)) return true;

        // R16F 256x128 tiles of the same Mercator window frame the color window uses.
        const double worldPx = static_cast<double>((1ll << zBase) * 256ll >> r.mip);
        const double groundRes = kMercCirc / worldPx;
        const long long gx0 = (orgPxX >> r.mip) + static_cast<long long>(r.x) * 256;
        const long long gy0 = (orgPxY >> r.mip) + static_cast<long long>(r.y) * 128;
        out.assign(65536, 0);
        uint16_t* dst16 = reinterpret_cast<uint16_t*>(out.data());
        for (uint32_t py = 0; py < 128; ++py) {
            const double Y = (gy0 + py + 0.5) / worldPx;
            const double lat = std::atan(std::sinh(kPi * (1.0 - 2.0 * Y)));
            for (uint32_t px = 0; px < 256; ++px) {
                const double X = (gx0 + px + 0.5) / worldPx;
                const double lon = (X - 0.5) * 2.0 * kPi;
                float h = 0.0f;
                for (HeightSource* s : ch.height) {
                    float m = 0.0f;
                    const float w = s->Sample(lat, lon, groundRes, m);
                    if (w <= 0.0f) continue;
                    h += (m - h) * w;
                }
                dst16[py * 256 + px] = FloatToHalf(h);
            }
        }
        WriteCached(path, out);
        ++painted;
        return true;
    };
}

void Compositor::LogRegistry() const {
    for (const auto& ch : m_channels) {
        const size_t n = ch.color.size() + ch.height.size();
        Log("[compose] channel %s: %zu layer%s (stack v%08x)", ch.name.c_str(), n,
            n == 1 ? "" : "s", static_cast<uint32_t>(ch.stackHash & 0xFFFFFFFFu));
        int i = 0;
        auto row = [&](const SourceInfo& s) {
            Log("[compose]   L%d %-22s %-38s %-26s %9.0f cm/px  [%.2f..%.2f]x[%.2f..%.2f]",
                i++, s.name.c_str(), s.structure.c_str(), s.crs.c_str(), s.cmPerPixel,
                s.lon0, s.lon1, s.lat0, s.lat1);
        };
        for (const auto* s : ch.color) row(s->Info());
        for (const auto* s : ch.height) row(s->Info());
    }
}

void FillComposedCb(ComposedSurfaceCb& cb, const ResidencyManager* rm, int colorCube,
                    int window, int heightCube, int heightWindow, double orgPxX,
                    double orgPxY, double sizePx, int zBase, double planetR,
                    const double east[3], const double up[3], const double north[3],
                    bool stencilOverlay, uint32_t gisWinSrv, uint32_t gisGlobSrv) {
    const bool cubeOn = rm && colorCube >= 0;
    const bool winOn = rm && window >= 0;
    const bool hgtOn = rm && heightCube >= 0;
    const bool hgtWinOn = rm && heightWindow >= 0;
    cb.u[0] = cubeOn ? rm->TextureSrv(colorCube) : UINT32_MAX;
    cb.u[1] = cubeOn ? rm->ResidencySrv(colorCube) : UINT32_MAX;
    cb.u[2] = winOn ? rm->TextureSrv(window) : UINT32_MAX;
    cb.u[3] = winOn ? rm->ResidencySrv(window) : UINT32_MAX;
    cb.u2[0] = hgtOn ? rm->TextureSrv(heightCube) : UINT32_MAX;
    cb.u2[1] = hgtOn ? rm->ResidencySrv(heightCube) : UINT32_MAX;
    cb.u2[2] = hgtWinOn ? rm->TextureSrv(heightWindow) : UINT32_MAX;
    cb.u2[3] = hgtWinOn ? rm->ResidencySrv(heightWindow) : UINT32_MAX;
    cb.u3[0] = gisWinSrv;
    cb.u3[1] = gisGlobSrv;
    cb.u3[2] = cb.u3[3] = UINT32_MAX;
    cb.f[0] = cubeOn ? 1.0f : 0.0f;
    cb.f[1] = winOn ? 1.0f : 0.0f;
    cb.f[2] = hgtOn ? 1.0f : 0.0f;
    cb.f[3] = static_cast<float>(planetR);
    cb.merc[0] = static_cast<float>(orgPxX);
    cb.merc[1] = static_cast<float>(orgPxY);
    cb.merc[2] = static_cast<float>(sizePx > 0.0 ? 1.0 / sizePx : 0.0);
    cb.merc[3] = static_cast<float>((1ll << zBase) * 256ll);
    // R16F pyramids stop at the one-tile-ish mip (16384 -> 256 = 7 levels, max lod 6); one
    // cube texel spans (pi/2)/16384 radians of arc along a face's midline.
    cb.g[0] = 6.0f;
    cb.g[1] = static_cast<float>(3.14159265358979 * 0.5 / Compositor::kFaceDim);
    cb.g[2] = 6.0f;
    cb.g[3] = stencilOverlay ? 1.0f : 0.0f;
    for (int i = 0; i < 3; ++i) {
        cb.r0[i] = static_cast<float>(east[i]);
        cb.r1[i] = static_cast<float>(up[i]);
        cb.r2[i] = static_cast<float>(north[i]);
    }
    cb.r0[3] = cb.r1[3] = cb.r2[3] = 0.0f;
}

}  // namespace ga
