#include "compose/Compositor.h"

#include <windows.h>

#include <array>
#include <cmath>
#include <cstdio>
#include <fstream>

namespace ga {

namespace {

constexpr double kPi = 3.14159265358979;
constexpr double kMercCirc = 40075016.686;   // Web-Mercator world metres (equator)
constexpr int kComposeVersion = 4;           // bump on any paint-math change: new cache tag
                                             // v4 (M9n): ETOPO carries the MSL -> NAVD88 link,
                                             // so every composed height tile repaints once
                                             // (v3: grade normalization REMOVED -- it
                                             // bleached seasonal land cover toward the z10
                                             // reference capture; pixels ship as Google
                                             // made them)

// Changing a stack (order, membership, version) must never serve stale composed tiles: the
// hash lands in the cache directory name, so an edit simply starts a fresh folder.
uint64_t Fnv1a(uint64_t h, const std::string& s) {
    for (const char c : s) {
        h ^= static_cast<uint8_t>(c);
        h *= 1099511628211ull;
    }
    return h;
}

// The frame IS the identity: a window realization's cache folder carries its zoom base and
// Mercator origin, so two windows of one channel can never serve each other's tiles.
std::string WindowTag(const char* kind, long long orgPxX, long long orgPxY, int zBase) {
    char buf[96];
    snprintf(buf, sizeof(buf), "%s_z%d_%lld_%lld", kind, zBase, orgPxX, orgPxY);
    return buf;
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

int Compositor::AddColorChannel(const std::string& name, std::vector<ColorSource*> stack) {
    Channel ch;
    ch.name = name;
    ch.color = std::move(stack);
    ch.gateOf.assign(ch.color.size(), -1);
    ch.gateOnly.assign(ch.color.size(), 0u);
    m_channels.push_back(std::move(ch));
    return static_cast<int>(m_channels.size()) - 1;
}

void Compositor::SetColorGate(int channel, size_t layer, size_t gate) {
    Channel& ch = m_channels[channel];
    if (layer >= ch.color.size() || gate >= ch.color.size() || layer == gate) {
        Log("[compose] %s: REFUSED gate %zu -> %zu (out of range, or a layer gating itself)",
            ch.name.c_str(), gate, layer);
        return;
    }
    ch.gateOf[layer] = static_cast<int>(gate);
    ch.gateOnly[gate] = 1u;
    Log("[compose] %s: %s GATES %s -- weight multiplied per texel; the gate paints nothing",
        ch.name.c_str(), ch.color[gate]->Info().name.c_str(),
        ch.color[layer]->Info().name.c_str());
}

int Compositor::AddHeightChannel(const std::string& name, std::vector<HeightSource*> stack) {
    Channel ch;
    ch.name = name;
    ch.height = std::move(stack);
    m_channels.push_back(std::move(ch));
    return static_cast<int>(m_channels.size()) - 1;
}

int Compositor::AddFieldChannel(const std::string& name, std::vector<FieldSource*> stack) {
    Channel ch;
    ch.name = name;
    ch.field = std::move(stack);
    m_channels.push_back(std::move(ch));
    return static_cast<int>(m_channels.size()) - 1;
}

// The soak rule's membership test (see Compositor.h). Global sources always belong; a window
// source belongs where its footprint overlaps the tile by at least ~2 texels in SOME axis (a
// one-texel-wide strip 100 texels long is a meaningful paint; a sub-texel speck is not).
// Dateline caution: cube tiles on the +-pi faces get loose lon boxes from atan2 -- global
// sources are immune (always in) and every regional source of ours sits far from the seam.
namespace {
bool SourceTouches(const SourceInfo& si, const Compositor::TileBox& b) {
    const double d2r = kPi / 180.0;
    const double sLatMin = si.lat0 * d2r, sLatMax = si.lat1 * d2r;
    const double sLonMin = si.lon0 * d2r, sLonMax = si.lon1 * d2r;
    if (si.lon1 - si.lon0 >= 359.0 && si.lat1 - si.lat0 >= 179.0) return true;   // global
    const double iLat = (std::min)(sLatMax, b.latMax) - (std::max)(sLatMin, b.latMin);
    const double iLon = (std::min)(sLonMax, b.lonMax) - (std::max)(sLonMin, b.lonMin);
    if (iLat <= 0.0 || iLon <= 0.0) return false;
    return iLat >= 2.0 * b.texLat || iLon >= 2.0 * b.texLon;
}
uint64_t SubsetSeed(const std::string& channelName) {
    return Fnv1a(14695981039346656037ull,
                 channelName + "#" + std::to_string(kComposeVersion));
}
}  // namespace

bool Compositor::Touches(double lon0, double lat0, double lon1, double lat1, const TileBox& b) {
    SourceInfo si;
    si.lon0 = lon0; si.lat0 = lat0; si.lon1 = lon1; si.lat1 = lat1;
    return SourceTouches(si, b);
}

uint64_t Compositor::ColorSubset(const Channel& ch, const TileBox& box,
                                 std::vector<size_t>& included) const {
    uint64_t h = SubsetSeed(ch.name);
    for (size_t i = 0; i < ch.color.size(); ++i) {
        const SourceInfo& si = ch.color[i]->Info();
        if (!SourceTouches(si, box)) continue;
        included.push_back(i);
        h = Fnv1a(h, si.name + "|" + si.structure);
    }
    return h;
}

uint64_t Compositor::HeightSubset(const Channel& ch, const TileBox& box,
                                  std::vector<size_t>& included) const {
    uint64_t h = SubsetSeed(ch.name);
    for (size_t i = 0; i < ch.height.size(); ++i) {
        const SourceInfo& si = ch.height[i]->Info();
        if (!SourceTouches(si, box)) continue;
        included.push_back(i);
        h = Fnv1a(h, si.name + "|" + si.structure);
    }
    return h;
}

uint64_t Compositor::FieldSubset(const Channel& ch, const TileBox& box,
                                 std::vector<size_t>& included) const {
    uint64_t h = SubsetSeed(ch.name);
    for (size_t i = 0; i < ch.field.size(); ++i) {
        const SourceInfo& si = ch.field[i]->Info();
        if (!SourceTouches(si, box)) continue;
        included.push_back(i);
        h = Fnv1a(h, si.name + "|" + si.structure);
    }
    return h;
}

// M9ai: resolve a tile to a place in an archive rather than to bytes.
//
// Opened lazily and cached per realization, under a lock because providers run on the loader
// pool. The lock is taken only on the FIRST touch of a realization -- after that the map lookup
// is read-only and the archive's own directory is immutable for the run.
bool Compositor::TryArchive(const Channel& ch, const char* realization, const TileRequest& r,
                            uint64_t subset, TileLoc* loc) {
    if (!loc) return false;
    TileArchive* arc = nullptr;
    {
        std::lock_guard<std::mutex> lk(m_archiveMx);
        auto it = m_archives.find(realization);
        if (it == m_archives.end()) {
            TileArchive a;
            const bool ok = a.Open(ch.name, realization);
            it = m_archives.emplace(realization, std::move(a)).first;
            if (ok) {
                Log("[tilearch] %s/%s: %zu tiles available for direct read", ch.name.c_str(),
                    realization, it->second.Count());
            }
        }
        arc = &it->second;
    }
    if (!arc->Valid()) return false;
    const uint64_t key = (uint64_t(r.face) << 61) | (uint64_t(r.mip) << 56) |
                         (uint64_t(r.y & 0xFFFFFFFull) << 28) | uint64_t(r.x & 0xFFFFFFFull);
    const TileArchive::Rec* rec = arc->Find(key, static_cast<uint32_t>(subset & 0xFFFFFFFFu));
    if (!rec) return false;
    loc->path = arc->WPath().c_str();
    loc->offset = rec->offset;
    loc->size = rec->size;
    return true;
}

std::string Compositor::CachePath(const Channel& ch, const char* realization,
                                  const TileRequest& r, uint64_t subset) const {
    char buf[320];
    snprintf(buf, sizeof(buf), "cache\\composed\\%s\\%s\\f%u_m%u_x%u_y%u_%08x.bin",
             ch.name.c_str(), realization, r.face, r.mip, r.x, r.y,
             static_cast<uint32_t>(subset & 0xFFFFFFFFu));
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
    // M6k: identity moved from the DIRECTORY (whole-stack hash) into each tile's FILENAME
    // (per-tile source-subset hash) -- the soak rule's short circuit lives in that rename.
    char buf[320];
    CreateDirectoryA("cache", nullptr);
    CreateDirectoryA("cache\\composed", nullptr);
    snprintf(buf, sizeof(buf), "cache\\composed\\%s", ch.name.c_str());
    CreateDirectoryA(buf, nullptr);
    snprintf(buf, sizeof(buf), "cache\\composed\\%s\\%s", ch.name.c_str(), realization);
    CreateDirectoryA(buf, nullptr);
}

// ---- realizations --------------------------------------------------------------------------

// THE ONE COLOUR PAINT LOOP. Both realizations differed only in their frame, and the frame is
// now an argument -- so the bottom-to-top stack walk, the lerp that IS "paint over", the cover
// rule and the transient-never-cached rule exist in exactly one place. A source returning -1
// (coverage exists, the fetch failed) leaves a hole THIS run and is never written: the next run
// repaints it.
void Compositor::PaintColorTile(const Channel& ch, const ColorFrame& frame, const TileRequest& r,
                                const TileBox& box, const std::vector<size_t>& inc,
                                std::vector<uint8_t>& out, bool& complete) const {
    const double groundRes = frame.GroundRes(r.mip);
    std::vector<PaintCtx> ctxs(inc.size());
    for (size_t k = 0; k < inc.size(); ++k) {
        ch.color[inc[k]]->BeginTile(box.latMin, box.latMax, box.lonMin, box.lonMax, groundRes,
                                    ctxs[k]);
    }
    // Where each source sits in `inc`, so a gate can be found without a search per texel.
    // -1 means the gate is not in this tile's subset, which is NO OPINION (factor 1) -- a gate
    // whose footprint misses must never delete the layer it was meant to qualify.
    std::vector<int> posOf(ch.color.size(), -1);
    for (size_t k = 0; k < inc.size(); ++k) posOf[inc[k]] = static_cast<int>(k);

    out.assign(65536, 0);
    complete = true;
    std::vector<float> w(inc.size());
    std::vector<std::array<uint8_t, 4>> rgba(inc.size());
    for (uint32_t py = 0; py < frame.texH; ++py) {
        for (uint32_t px = 0; px < frame.texW; ++px) {
            double lat = 0, lon = 0;
            frame.Texel(r, px, py, lat, lon);
            // Sample every layer in the subset ONCE, gates included -- a gate is a source and
            // costs exactly what it did before it was given that role.
            for (size_t k = 0; k < inc.size(); ++k) {
                w[k] = ch.color[inc[k]]->Sample(lat, lon, groundRes, ctxs[k], rgba[k].data());
                if (w[k] < 0.0f) complete = false;
            }
            uint8_t* dst = &out[(py * frame.texW + px) * 4];
            float acc[3] = {0, 0, 0};
            float cover = 0.0f;
            for (size_t k = 0; k < inc.size(); ++k) {   // bottom -> top, subset only
                if (ch.gateOnly[inc[k]]) continue;      // gates paint nothing
                float wk = w[k];
                if (wk <= 0.0f) continue;               // (-1 = transient, already recorded)
                const int g = ch.gateOf[inc[k]];
                if (g >= 0 && posOf[g] >= 0) {
                    const float gw = w[posOf[g]];
                    if (gw <= 0.0f) continue;           // gated out here entirely
                    wk *= gw;
                }
                for (int c = 0; c < 3; ++c) acc[c] += (rgba[k][c] - acc[c]) * wk;
                cover = (std::max)(cover, wk);
            }
            dst[0] = static_cast<uint8_t>(acc[0]);
            dst[1] = static_cast<uint8_t>(acc[1]);
            dst[2] = static_cast<uint8_t>(acc[2]);
            dst[3] = cover > 0.0f ? 255 : 0;
        }
    }
}

// ONE source, over the same addresses. The difference from the loop above is the whole point of
// the split: there is no stack walk and no lerp, so nothing here can be affected by what else is
// in the channel. That independence IS the tree's cache identity -- a source's tiles do not
// change when the blend above them changes, which is why they never have to be repainted for it.
//
// Alpha carries the paint WEIGHT, quantized to 8 bits. That quantization is the one place this
// path is not exact, and it is bounded: the composite lerps by w, so an error of half an alpha
// step moves a channel by at most |rgba - acc| / 510 -- under one LSB, and only on a feather
// ramp, where w is strictly between 0 and 1. Every source that answers 0 or 1 (google always;
// an ortho everywhere but its edge) round-trips exactly, and an overlay whose weight IS a byte
// of per-pixel alpha round-trips exactly too.
void Compositor::PaintSourceTile(ColorSource* src, const ColorFrame& frame, const TileRequest& r,
                                 const TileBox& box, std::vector<uint8_t>& out, bool& complete,
                                 bool& anyCover, bool& fullCover) {
    const double groundRes = frame.GroundRes(r.mip);
    PaintCtx ctx;
    src->BeginTile(box.latMin, box.latMax, box.lonMin, box.lonMax, groundRes, ctx);
    out.assign(65536, 0);
    complete = true;
    anyCover = false;
    fullCover = true;
    for (uint32_t py = 0; py < frame.texH; ++py) {
        for (uint32_t px = 0; px < frame.texW; ++px) {
            double lat = 0, lon = 0;
            frame.Texel(r, px, py, lat, lon);
            uint8_t* dst = &out[(py * frame.texW + px) * 4];
            uint8_t rgba[4] = {0, 0, 0, 0};
            const float w = src->Sample(lat, lon, groundRes, ctx, rgba);
            if (w < 0.0f) {   // TRANSIENT: coverage exists, the fetch failed. Cache nothing.
                complete = false;
                fullCover = false;
                continue;
            }
            if (w <= 0.0f) { fullCover = false; continue; }
            const uint8_t a = static_cast<uint8_t>(w * 255.0f + 0.5f);
            dst[0] = rgba[0];
            dst[1] = rgba[1];
            dst[2] = rgba[2];
            dst[3] = a;
            anyCover = true;
            if (a != 255) fullCover = false;
        }
    }
}

// One realization = one frame + the cache identity that frame demands. THE REALIZATION'S NAME
// IS ITS FRAME (M7x): three colour windows once shared the literal folder "window", so the z17
// paint of tile (x,y) was served as the z14 tile (x,y) -- the patchwork of displaced,
// wrong-scale imagery on the flood rail. A tile's content is a pure function of
// (org, zBase, mip, x, y) plus the stack, so org and zBase are IN the folder name and distinct
// frames cannot collide by construction. Identical frames still share, which is how the export
// warms the live window.
TileProviderFn Compositor::ColorRealization(int channel, const ColorFrame& frame) {
    const std::string tag = frame.Tag();
    EnsureCacheDir(m_channels[channel], tag.c_str());
    return [this, channel, frame, tag](const TileRequest& r, std::vector<uint8_t>& out,
                                       TileLoc* loc) {
        const Channel& ch = m_channels[channel];
        TileBox box{};
        frame.Box(r, box);
        std::vector<size_t> inc;
        const uint64_t subset = ColorSubset(ch, box, inc);
        // M9ai: an archived tile resolves to a PLACE, and the bytes never enter this process.
        if (TryArchive(ch, tag.c_str(), r, subset, loc)) return true;
        const std::string path = CachePath(ch, tag.c_str(), r, subset);
        if (ReadCached(path, out)) return true;
        bool complete = true;
        PaintColorTile(ch, frame, r, box, inc, out, complete);
        if (complete) {
            WriteCached(path, out);
            ++painted;
        }
        return true;
    };
}

TileProviderFn Compositor::CubeColor(int channel) {
    return ColorRealization(channel, ColorFrame::Cube(kFaceDim));
}

TileProviderFn Compositor::WindowColor(int channel, long long orgPxX, long long orgPxY,
                                       uint32_t sizePx, int zBase) {
    (void)sizePx;
    return ColorRealization(channel, ColorFrame::Window(orgPxX, orgPxY, zBase));
}

TileProviderFn Compositor::CubeHeight(int channel) {
    EnsureCacheDir(m_channels[channel], "cube16k");
    return [this, channel](const TileRequest& r, std::vector<uint8_t>& out,
                           TileLoc* loc) {
        const Channel& ch = m_channels[channel];
        // R16F: 256x128 texels per 64KB tile. Every mip is painted straight from the sources
        // at its own ground resolution (independent paints; trilinear blends cousins -- close
        // enough for display relief, and each level is honest about its own footprint).
        const uint32_t faceTexels = kFaceDim >> r.mip;
        const double invFace = 1.0 / faceTexels;
        const double groundRes = kMercCirc / (4.0 * faceTexels);
        TileBox box{10, -10, 10, -10, 0, 0};
        for (int cy = 0; cy < 3; ++cy) {
            for (int cx = 0; cx < 3; ++cx) {
                double d[3];
                ComposeCubeDir(r.face, (r.x * 256.0 + cx * 128.0) * invFace,
                               (r.y * 128.0 + cy * 64.0) * invFace, d);
                const double la = std::asin((std::max)(-1.0, (std::min)(1.0, d[1])));
                const double lo = std::atan2(d[2], d[0]);
                box.latMin = (std::min)(box.latMin, la);
                box.latMax = (std::max)(box.latMax, la);
                box.lonMin = (std::min)(box.lonMin, lo);
                box.lonMax = (std::max)(box.lonMax, lo);
            }
        }
        box.texLat = (box.latMax - box.latMin) / 128.0;
        box.texLon = (box.lonMax - box.lonMin) / 256.0;
        std::vector<size_t> inc;
        const uint64_t subset = HeightSubset(ch, box, inc);
        // M9ai: an archived tile resolves to a PLACE, and the bytes never enter this process.
        if (TryArchive(ch, "cube16k", r, subset, loc)) return true;
        const std::string path = CachePath(ch, "cube16k", r, subset);
        if (ReadCached(path, out)) return true;

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
                for (size_t k = 0; k < inc.size(); ++k) {   // bottom -> top, subset only
                    float m = 0.0f;
                    const float w = ch.height[inc[k]]->Sample(lat, lon, groundRes, m);
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
    const std::string tag = WindowTag("windowH", orgPxX, orgPxY, zBase);
    EnsureCacheDir(m_channels[channel], tag.c_str());
    (void)sizePx;
    return [this, channel, orgPxX, orgPxY, zBase, tag](const TileRequest& r,
                                                       std::vector<uint8_t>& out,
                                                       TileLoc* loc) {
        const Channel& ch = m_channels[channel];
        // R16F 256x128 tiles of the same Mercator window frame the color window uses.
        const double worldPx = static_cast<double>((1ll << zBase) * 256ll >> r.mip);
        const double groundRes = kMercCirc / worldPx;
        const long long gx0 = (orgPxX >> r.mip) + static_cast<long long>(r.x) * 256;
        const long long gy0 = (orgPxY >> r.mip) + static_cast<long long>(r.y) * 128;
        TileBox box{};
        box.latMin = std::atan(std::sinh(kPi * (1.0 - 2.0 * (gy0 + 128.0) / worldPx)));
        box.latMax = std::atan(std::sinh(kPi * (1.0 - 2.0 * gy0 / worldPx)));
        box.lonMin = (gx0 / worldPx - 0.5) * 2.0 * kPi;
        box.lonMax = ((gx0 + 256.0) / worldPx - 0.5) * 2.0 * kPi;
        box.texLat = (box.latMax - box.latMin) / 128.0;
        box.texLon = (box.lonMax - box.lonMin) / 256.0;
        std::vector<size_t> inc;
        const uint64_t subset = HeightSubset(ch, box, inc);
        // M9ai: an archived tile resolves to a PLACE, and the bytes never enter this process.
        if (TryArchive(ch, tag.c_str(), r, subset, loc)) return true;
        const std::string path = CachePath(ch, tag.c_str(), r, subset);
        if (ReadCached(path, out)) return true;

        out.assign(65536, 0);
        uint16_t* dst16 = reinterpret_cast<uint16_t*>(out.data());
        for (uint32_t py = 0; py < 128; ++py) {
            const double Y = (gy0 + py + 0.5) / worldPx;
            const double lat = std::atan(std::sinh(kPi * (1.0 - 2.0 * Y)));
            for (uint32_t px = 0; px < 256; ++px) {
                const double X = (gx0 + px + 0.5) / worldPx;
                const double lon = (X - 0.5) * 2.0 * kPi;
                float h = 0.0f;
                for (size_t k = 0; k < inc.size(); ++k) {
                    float m = 0.0f;
                    const float w = ch.height[inc[k]]->Sample(lat, lon, groundRes, m);
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

TileProviderFn Compositor::WindowField(int channel, long long orgPxX, long long orgPxY,
                                       uint32_t sizePx, int zBase) {
    const std::string tag = WindowTag("windowF", orgPxX, orgPxY, zBase);
    EnsureCacheDir(m_channels[channel], tag.c_str());
    (void)sizePx;
    return [this, channel, orgPxX, orgPxY, zBase, tag](const TileRequest& r,
                                                       std::vector<uint8_t>& out,
                                                       TileLoc* loc) {
        const Channel& ch = m_channels[channel];
        // RG16F 128x128 tiles: the phasor fiber (re, im) in the Mercator window frame.
        const double worldPx = static_cast<double>((1ll << zBase) * 256ll >> r.mip);
        const double groundRes = kMercCirc / worldPx;
        const long long gx0 = (orgPxX >> r.mip) + static_cast<long long>(r.x) * 128;
        const long long gy0 = (orgPxY >> r.mip) + static_cast<long long>(r.y) * 128;
        TileBox box{};
        box.latMin = std::atan(std::sinh(kPi * (1.0 - 2.0 * (gy0 + 128.0) / worldPx)));
        box.latMax = std::atan(std::sinh(kPi * (1.0 - 2.0 * gy0 / worldPx)));
        box.lonMin = (gx0 / worldPx - 0.5) * 2.0 * kPi;
        box.lonMax = ((gx0 + 128.0) / worldPx - 0.5) * 2.0 * kPi;
        box.texLat = (box.latMax - box.latMin) / 128.0;
        box.texLon = (box.lonMax - box.lonMin) / 128.0;
        std::vector<size_t> inc;
        const uint64_t subset = FieldSubset(ch, box, inc);
        // M9ai: an archived tile resolves to a PLACE, and the bytes never enter this process.
        if (TryArchive(ch, tag.c_str(), r, subset, loc)) return true;
        const std::string path = CachePath(ch, tag.c_str(), r, subset);
        if (ReadCached(path, out)) return true;

        out.assign(65536, 0);
        uint16_t* dst16 = reinterpret_cast<uint16_t*>(out.data());
        for (uint32_t py = 0; py < 128; ++py) {
            const double Y = (gy0 + py + 0.5) / worldPx;
            const double lat = std::atan(std::sinh(kPi * (1.0 - 2.0 * Y)));
            for (uint32_t px = 0; px < 128; ++px) {
                const double X = (gx0 + px + 0.5) / worldPx;
                const double lon = (X - 0.5) * 2.0 * kPi;
                float v[2] = {0.0f, 0.0f};
                for (size_t k = 0; k < inc.size(); ++k) {
                    float s[2] = {0.0f, 0.0f};
                    const float w = ch.field[inc[k]]->Sample(lat, lon, groundRes, s);
                    if (w <= 0.0f) continue;
                    v[0] += (s[0] - v[0]) * w;   // per-pixel paint in the PLANE: re and im
                    v[1] += (s[1] - v[1]) * w;   // blend together -- amplitude survives
                }
                dst16[(py * 128 + px) * 2 + 0] = FloatToHalf(v[0]);
                dst16[(py * 128 + px) * 2 + 1] = FloatToHalf(v[1]);
            }
        }
        WriteCached(path, out);
        ++painted;
        return true;
    };
}

double Compositor::HeightGrainM(int channel, double latRad, double lonRad) const {
    const Channel& ch = m_channels[channel];
    const double lon = lonRad * 180.0 / 3.14159265358979323846, lat = latRad * 180.0 / 3.14159265358979323846;
    double finest = 0.0;
    for (const HeightSource* hs : ch.height) {
        const SourceInfo& s = hs->Info();
        if (s.cmPerPixel <= 0.0) continue;
        if (lon < s.lon0 || lon > s.lon1 || lat < s.lat0 || lat > s.lat1) continue;
        const double m = s.cmPerPixel * 0.01;
        if (finest <= 0.0 || m < finest) finest = m;
    }
    return finest;
}

float Compositor::SampleHeightStack(int channel, double latRad, double lonRad,
                                    double groundResM) const {
    const Channel& ch = m_channels[channel];
    float h = 0.0f;
    for (size_t i = 0; i < ch.height.size(); ++i) {
        float m = 0.0f;
        const float w = ch.height[i]->Sample(latRad, lonRad, groundResM, m);
        if (w <= 0.0f) continue;
        h += (m - h) * w;
    }
    return h;
}

void Compositor::SampleFieldStack(int channel, double latRad, double lonRad,
                                  double groundResM, float out[2]) const {
    const Channel& ch = m_channels[channel];
    out[0] = out[1] = 0.0f;
    for (size_t i = 0; i < ch.field.size(); ++i) {
        float s[2] = {0.0f, 0.0f};
        const float w = ch.field[i]->Sample(latRad, lonRad, groundResM, s);
        if (w <= 0.0f) continue;
        out[0] += (s[0] - out[0]) * w;
        out[1] += (s[1] - out[1]) * w;
    }
}

void Compositor::LogRegistry() const {
    for (const auto& ch : m_channels) {
        const size_t n = ch.color.size() + ch.height.size();
        Log("[compose] channel %s: %zu layer%s (identity: per-tile source subsets, v%d)",
            ch.name.c_str(), n, n == 1 ? "" : "s", kComposeVersion);
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
                    bool stencilOverlay, int maskPages, int detailWin,
                    const double* detOrgPx, int detailZ, uint32_t winSlice,
                    uint32_t detSlice, uint32_t hgtWinSlice) {
    const bool cubeOn = rm && colorCube >= 0;
    const bool winOn = rm && window >= 0;
    const bool hgtOn = rm && heightCube >= 0;
    const bool hgtWinOn = rm && heightWindow >= 0;
    // M9ap: the pages path. One tenant; the cube views cover slices 0..5, the array view
    // carries the Mercator pages. The old window/detail SRVs are left unset so nothing can
    // read a second texture by accident.
    const bool pages = cubeOn && winOn && window == colorCube && winSlice != UINT32_MAX;
    cb.u5[0] = pages ? rm->TextureSrv(colorCube) : UINT32_MAX;
    cb.u5[1] = pages ? rm->ResidencySrv(colorCube) : UINT32_MAX;
    cb.u5[2] = pages ? winSlice : UINT32_MAX;
    cb.u5[3] = pages ? detSlice : UINT32_MAX;
    cb.u[0] = cubeOn ? (pages ? rm->TextureSrvCube(colorCube) : rm->TextureSrv(colorCube))
                     : UINT32_MAX;
    cb.u[1] = cubeOn ? (pages ? rm->ResidencySrvCube(colorCube) : rm->ResidencySrv(colorCube))
                     : UINT32_MAX;
    cb.u[2] = (winOn && !pages) ? rm->TextureSrv(window) : UINT32_MAX;
    cb.u[3] = (winOn && !pages) ? rm->ResidencySrv(window) : UINT32_MAX;
    // M9aq: height pages -- one tenant, cube views over slices 0..5, the array view carrying
    // the z14 page. The old window SRVs are left unset so nothing can read a second texture.
    const bool hpages = hgtOn && hgtWinOn && heightWindow == heightCube && hgtWinSlice != UINT32_MAX;
    cb.u6[0] = hpages ? rm->TextureSrv(heightCube) : UINT32_MAX;
    cb.u6[1] = hpages ? rm->ResidencySrv(heightCube) : UINT32_MAX;
    cb.u6[2] = hpages ? hgtWinSlice : UINT32_MAX;
    cb.u6[3] = UINT32_MAX;
    cb.u2[0] = hgtOn ? (hpages ? rm->TextureSrvCube(heightCube) : rm->TextureSrv(heightCube))
                     : UINT32_MAX;
    cb.u2[1] = hgtOn ? (hpages ? rm->ResidencySrvCube(heightCube) : rm->ResidencySrv(heightCube))
                     : UINT32_MAX;
    cb.u2[2] = (hgtWinOn && !hpages) ? rm->TextureSrv(heightWindow) : UINT32_MAX;
    cb.u2[3] = (hgtWinOn && !hpages) ? rm->ResidencySrv(heightWindow) : UINT32_MAX;
    // M9ay: the survey MASK PAGES (gis.landsea's tree as a page tenant): array SRV + residency
    // for the Mercator pages (slices 6, 7), cube views for the faces. r = water coverage,
    // b = edited, a = surveyed. UINT32_MAX = no survey: the classifier uses the height sign.
    const bool maskOn = rm && maskPages >= 0;
    cb.u3[0] = maskOn ? rm->TextureSrv(maskPages) : UINT32_MAX;
    cb.u3[1] = maskOn ? rm->ResidencySrv(maskPages) : UINT32_MAX;
    cb.u3[2] = maskOn ? rm->TextureSrvCube(maskPages) : UINT32_MAX;
    cb.u3[3] = maskOn ? rm->ResidencySrvCube(maskPages) : UINT32_MAX;
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
    // M7f: the DETAIL color window (z17) -- the third rung of the one ladder -- plus the
    // fine edit mask (surveyed structures at ~1 m over their own bbox).
    const bool detOn = rm && detailWin >= 0 && detOrgPx;
    cb.u4[0] = (detOn && !pages) ? rm->TextureSrv(detailWin) : UINT32_MAX;
    cb.u4[1] = (detOn && !pages) ? rm->ResidencySrv(detailWin) : UINT32_MAX;
    cb.u4[2] = UINT32_MAX;   // M9ay: the fine edit raster is gone; edits ride the mask pages
    cb.u4[3] = UINT32_MAX;
    cb.det[0] = cb.det[1] = cb.det[2] = 0.0f;
    if (detOn && zBase > 0 && sizePx > 0.0) {
        const double f = static_cast<double>(1ll << (detailZ - zBase));
        cb.det[0] = static_cast<float>((orgPxX * f - detOrgPx[0]) / 16384.0);
        cb.det[1] = static_cast<float>((orgPxY * f - detOrgPx[1]) / 16384.0);
        cb.det[2] = static_cast<float>(sizePx * f / 16384.0);
    }
    cb.det[3] = 0.0f;
    for (int i = 0; i < 4; ++i) cb.ed[i] = 0.0f;
}

}  // namespace ga
