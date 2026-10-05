#include "scene/WaterBankLayer.h"

#include "hal/GpuProfiler.h"

#include "core/Image.h"
#include "hal/PixEvents.h"
#include "hal/Pipeline.h"
#include "hal/Root.h"
#include "scene/SeaLayer.h"
#include "core/SceneConfig.h"
#include "sim/WaveField.h"
#include "sim/WaveScale.h"

#include "hal/Views.h"

#include <algorithm>
#include <chrono>
#include <filesystem>
#include <map>
#include <cmath>
#include <cstdio>
#include <cstring>

namespace ga {

namespace {
constexpr double kPiD = 3.14159265358979;
constexpr double kD2R = kPiD / 180.0;
}  // namespace

void WaterBankLayer::Configure(const std::wstring& shaderDir, SeaLayer* sea, SweSolver* swe,
                               const WaterAtlas* atlas,
                               Compositor* comp, int hgtCh, const GlobeModel* globe,
                               const SeaState* seaState) {
    m_shaderDir = shaderDir;
    m_sea = sea;
    m_swe = swe;
    m_atlas = atlas;
    m_comp = comp;
    m_hgtCh = hgtCh;
    m_globe = globe;
    m_seaState = seaState;
}

void WaterBankLayer::Init(Gpu& gpu, ShaderCompiler& sc, FieldSet&, hal::RootSignature) {
    if (!m_sea) return;
    m_sc = &sc;
    // The mip ladder side by side: ring m occupies texels [m*512, (m+1)*512) x [0, 512).
    m_disp.Init(gpu, kMips * kRingTexels, kRingTexels, DXGI_FORMAT_R16G16B16A16_FLOAT,
                L"water.disp (the wave vertex bank)");
    m_param.Init(gpu, kMips * kRingTexels, kRingTexels, DXGI_FORMAT_R16G16B16A16_FLOAT,
                 L"water.param (level + sigma2 + current)");
    m_detail.Init(gpu, kMips * kRingTexels, kRingTexels, DXGI_FORMAT_R16G16B16A16_FLOAT,
                  L"water.detail (hsScale + sparkle context)");

    // Root signature: b0 CB, t0 tile list, then the BINDLESS pair -- one unbounded SRV range
    // and one unbounded UAV range over the shared heap, so this kernel reaches every texture
    // by slot exactly the way the render path does.
    // M9aq: t0, space5 -- the heap as Texture2DArray, so the bed can be read from slice 6 of
    // the height PAGE tenant (one height texture; the window is a page of it).
    // The two samplers are this kernel's own -- MaxLOD 0 and no comparison function at all
    // (the desc's zero, not NEVER), unlike the house sampler -- and dead: WaterBank.hlsl
    // declares no SamplerState, its reads are manual bilinear loads, so nothing samples through
    // them (step 3d's finding, kept as found; step 3f's gate serialized them EQUAL to the descs
    // they replace).
    const hal::SamplerFields wrap{0,
                                  D3D12_FILTER_MIN_MAG_MIP_LINEAR,
                                  D3D12_TEXTURE_ADDRESS_MODE_WRAP,
                                  D3D12_TEXTURE_ADDRESS_MODE_WRAP,
                                  D3D12_TEXTURE_ADDRESS_MODE_WRAP,
                                  0.0f,
                                  static_cast<D3D12_COMPARISON_FUNC>(0),
                                  D3D12_SHADER_VISIBILITY_ALL,
                                  0};
    const hal::SamplerFields clamp{1,
                                   D3D12_FILTER_MIN_MAG_MIP_LINEAR,
                                   D3D12_TEXTURE_ADDRESS_MODE_CLAMP,
                                   D3D12_TEXTURE_ADDRESS_MODE_CLAMP,
                                   D3D12_TEXTURE_ADDRESS_MODE_CLAMP,
                                   0.0f,
                                   static_cast<D3D12_COMPARISON_FUNC>(0),
                                   D3D12_SHADER_VISIBILITY_ALL,
                                   0};
    m_rs = hal::RootLayout{}
               .Cbv(0)
               .Srv(0)
               .Table({hal::SrvRange(0, hal::kUnbounded, 1), hal::SrvRange(0, hal::kUnbounded, 5)})
               .Table({hal::UavRange(0, hal::kUnbounded, 2)})
               .Sampler(wrap)
               .Sampler(clamp)
               .Build(gpu, "waterbank");
    m_rs->SetName(L"water bank root signature");

    m_fill = hal::Require(
        hal::BuildCompute(gpu, m_rs.Get(),
                          sc.Compile(m_shaderDir + L"/WaterBank.hlsl", L"CsBankFill", L"cs_6_0"),
                          "waterbank"),
        "WaterBank kernel");
    m_fill->SetName(L"CsBankFill");
    m_ready = true;
    Log("[waterbank] %d mip rings x %dx%d tiles (%.1f m .. %.0f m texels, %.1f .. %.0f km "
        "spans); land tiles NULL",
        kMips, kRingTiles, kRingTiles, m_baseTexelM,
        m_baseTexelM * (1 << (kMips - 1)),
        kRingTexels * m_baseTexelM / 1000.0,
        kRingTexels * m_baseTexelM * (1 << (kMips - 1)) / 1000.0);
}

void WaterBankLayer::ReloadShaders(Gpu& gpu, ShaderCompiler& sc) {
    const ShaderBlob cs =
        sc.Compile(m_shaderDir + L"/WaterBank.hlsl", L"CsBankFill", L"cs_6_0");
    hal::Reload(m_fill, [&] { return hal::BuildCompute(gpu, m_rs.Get(), cs, "waterbank"); },
                "waterbank");
}

void WaterBankLayer::SetFrame(Gpu& gpu, double simUnix, double camX, double camZ) {
    m_simUnix = simUnix;
    m_camX = camX;
    m_camZ = camZ;
    // Where the rings stand is not a drawing decision: `enabled` says whether this frame FILLS
    // them (Render), and a bank anchored while nothing reads it is mapped and ready for the
    // frame that does (set B ahead of a window).
    if (!m_ready) return;
    for (int m = 0; m < kMips; ++m) ReanchorRing(gpu, m, camX, camZ);
}

// M13 step 2: the place a ring point holds, on the sphere the mesh is drawn on. The rings are a
// RADIAL PROJECTION onto the root's tangent plane -- the mesh reads them at (R d.x, R d.z) of a
// vertex's direction -- so the place of (wx, wz) is the direction that projects there
// (Space::Anchor::PlaceOfProjected). Where the surface never wrote its rows this is the
// anchor-linear chart, exactly as every line here used to be.
void WaterBankLayer::PlaceOfRing(double wx, double wz, double& latDeg, double& lonDeg) const {
    if (m_surface && m_surface->flat.Exact() &&
        m_surface->flat.PlaceOfProjected(wx, wz, latDeg, lonDeg)) {
        return;
    }
    latDeg = BathyModel::kOrgLat + wz / BathyModel::kMPerLat;
    lonDeg = BathyModel::kOrgLon + wx / BathyModel::kMPerLon;
}

bool WaterBankLayer::TileWet(double wx0, double wz0, double spanM) const {
    if (!m_comp || m_hgtCh < 0) return true;
    // Five-point test against the one height stack: any point at or below the high-water
    // margin keeps the tile; a tile of pure upland stays NULL.
    for (int k = 0; k < 5; ++k) {
        const double fx = (k == 4) ? 0.5 : (k & 1) ? 0.96 : 0.04;
        const double fz = (k == 4) ? 0.5 : (k & 2) ? 0.96 : 0.04;
        double lat = 0.0, lon = 0.0;
        PlaceOfRing(wx0 + fx * spanM, wz0 + fz * spanM, lat, lon);
        if (m_comp->SampleHeightStack(m_hgtCh, lat * kD2R, lon * kD2R, spanM * 0.25) < 2.5f) {
            return true;
        }
    }
    return false;
}

void WaterBankLayer::CornerParams(double wx, double wz, float& lvl, float& bed) const {
    double lat = 0.0, lon = 0.0;
    PlaceOfRing(wx, wz, lat, lon);
    lvl = 0.0f;
    if (m_atlas && m_atlas->Ready()) {
        lvl = static_cast<float>(m_atlas->MslNavd(lat, lon) +
                                 m_atlas->Level(lat, lon, m_simUnix, 500.0));
    }
    bed = (m_comp && m_hgtCh >= 0)
              ? m_comp->SampleHeightStack(m_hgtCh, lat * kD2R, lon * kD2R, 100.0)
              : -30.0f;
}

void WaterBankLayer::ReanchorRing(Gpu& gpu, int m, double camX, double camZ) {
    const double texel = m_baseTexelM * (1 << m);
    const double tileSpan = kTileTexels * texel;
    const double half = 0.5 * kRingTexels * texel;
    const float ox = static_cast<float>(std::floor((camX - half) / tileSpan) * tileSpan);
    const float oz = static_cast<float>(std::floor((camZ - half) / tileSpan) * tileSpan);
    if (m_orgValid[m] && ox == m_orgX[m] && oz == m_orgZ[m]) return;
    m_orgX[m] = ox;
    m_orgZ[m] = oz;
    m_orgValid[m] = true;

    // Land tiles NULL: RGBA16F hardware tiles are 128x64, so one logical 128^2 tile is a
    // 1x2 hardware pair in each bank.
    bool changed = false;
    for (int ty = 0; ty < kRingTiles; ++ty) {
        for (int tx = 0; tx < kRingTiles; ++tx) {
            const bool wet =
                TileWet(ox + tx * tileSpan, oz + ty * tileSpan, tileSpan);
            uint8_t& state = m_wet[m][ty * kRingTiles + tx];
            if (state == (wet ? 1 : 0) && m_orgValid[m]) {
                // state carries over; mapping only changes on wet/dry flips
            }
            const uint32_t htx = static_cast<uint32_t>(
                (m * kRingTexels + tx * kTileTexels) / m_disp.TileW());
            const uint32_t hty0 = static_cast<uint32_t>(ty * kTileTexels / m_disp.TileH());
            const uint32_t hcount = kTileTexels / m_disp.TileH();
            for (uint32_t k = 0; k < hcount; ++k) {
                if (wet) {
                    m_disp.RequestMap(htx, hty0 + k);
                    m_param.RequestMap(htx, hty0 + k);
                    m_detail.RequestMap(htx, hty0 + k);
                } else {
                    m_disp.RequestUnmap(htx, hty0 + k);
                    m_param.RequestUnmap(htx, hty0 + k);
                    m_detail.RequestUnmap(htx, hty0 + k);
                }
            }
            if (state != (wet ? 1 : 0)) changed = true;
            state = wet ? 1 : 0;
        }
    }
    std::vector<uint32_t> fresh;
    m_disp.CommitMappings(gpu, &fresh);
    fresh.clear();
    m_param.CommitMappings(gpu, &fresh);
    fresh.clear();
    m_detail.CommitMappings(gpu, &fresh);
    (void)changed;
}

namespace {
float HalfF(uint16_t h) {
    const uint32_t s = (h >> 15) & 1u, e = (h >> 10) & 31u, m = h & 1023u;
    if (e == 0) return (s ? -1.0f : 1.0f) * m * 5.9604645e-8f;
    if (e == 31) return s ? -1e30f : 1e30f;
    float v = std::ldexp(1.0f + m / 1024.0f, static_cast<int>(e) - 15);
    return s ? -v : v;
}
}  // namespace

// M9bq: the twin gate's reader. ONE readback per plane, then N points addressed out of the
// snapshot -- see the header for why TraceProbe cannot serve this.
//
// A point is answered by the FINEST resident ring that contains it, which is the same rule the
// renderer's BankSample walks; a point outside every ring comes back valid = false rather than
// clamped to the coarsest, because "off the bank" and "on the bank's edge" are different
// statements and a gate that confused them would report a fictitious disagreement.
void WaterBankLayer::ReadBankPoints(Gpu& gpu, const double* worldXz, int n, BankPoint* out) {
    for (int i = 0; i < n; ++i) out[i] = BankPoint{};
    if (n <= 0) return;

    auto snap = [&](TileAtlas2D& bank, std::vector<uint8_t>& data, uint32_t& pitch) {
        GpuTexture wrap;
        wrap.res = bank.Res();
        wrap.format = DXGI_FORMAT_R16G16B16A16_FLOAT;
        wrap.width = 3072;
        wrap.height = 512;
        wrap.state = m_state;
        data = gpu.ReadbackTexture(wrap, &pitch);
    };
    std::vector<uint8_t> dData, pData, tData;
    uint32_t dPitch = 0, pPitch = 0, tPitch = 0;
    snap(m_disp, dData, dPitch);
    snap(m_param, pData, pPitch);
    snap(m_detail, tData, tPitch);
    if (dData.empty() || pData.empty() || tData.empty()) return;

    // THE MESH'S RECONSTRUCTION, not the texel under the point (the water match, step 2): a texel
    // holds its field at its CENTRE, and the mesh reads the bank through Globe.hlsl's BankFetch --
    // the Catmull-Rom cubic over the 4x4 support for disp, the tent for param. The nearest texel
    // stood up to half a 1.2 m texel off the point, which on a 0.2 slope is 6 cm of "disagreement"
    // the instrument manufactured itself. (The ring cross-fade is not applied: the finest ring that
    // holds the whole support answers, as it does 48 texels inside every border.)
    const auto catmull = [](double t, double w[4]) {
        const double t2 = t * t, t3 = t2 * t;
        w[0] = -0.5 * t3 + t2 - 0.5 * t;
        w[1] = 1.5 * t3 - 2.5 * t2 + 1.0;
        w[2] = -1.5 * t3 + 2.0 * t2 + 0.5 * t;
        w[3] = 0.5 * t3 - 0.5 * t2;
    };
    for (int i = 0; i < n; ++i) {
        const double wx = worldXz[i * 2 + 0], wz = worldXz[i * 2 + 1];
        for (int m = 0; m < kMips; ++m) {
            if (!m_orgValid[m]) continue;
            const double texel = m_baseTexelM * (1 << m);
            const double lx = (wx - m_orgX[m]) / texel, lz = (wz - m_orgZ[m]) / texel;
            if (lx < 2.0 || lz < 2.0 || lx > 510.0 || lz > 510.0) continue;   // BankFetch's window
            const int tx0 = static_cast<int>(std::floor(lx - 0.5));
            const int tz0 = static_cast<int>(std::floor(lz - 0.5));
            const double fx = (lx - 0.5) - tx0, fz = (lz - 0.5) - tz0;
            const auto at = [&](const std::vector<uint8_t>& data, uint32_t pitch, int tx, int tz,
                                int c) {
                const uint16_t* px = reinterpret_cast<const uint16_t*>(
                    data.data() + static_cast<size_t>(pitch) * static_cast<size_t>(tz)) +
                    (m * 512 + tx) * 4;
                return double(HalfF(px[c]));
            };
            double d[4] = {0, 0, 0, 0}, p[4] = {0, 0, 0, 0}, g[4] = {0, 0, 0, 0};
            for (int c = 0; c < 4; ++c) {
                for (int k = 0; k < 4; ++k) {   // the tent
                    const double wgt = ((k & 1) ? fx : 1.0 - fx) * ((k >> 1) ? fz : 1.0 - fz);
                    p[c] += wgt * at(pData, pPitch, tx0 + (k & 1), tz0 + (k >> 1), c);
                    g[c] += wgt * at(tData, tPitch, tx0 + (k & 1), tz0 + (k >> 1), c);
                }
            }
            double wxc[4], wzc[4];
            catmull(fx, wxc);
            catmull(fz, wzc);
            for (int c = 0; c < 4; ++c) {
                for (int j = 0; j < 4; ++j) {   // the cubic
                    for (int k = 0; k < 4; ++k) {
                        d[c] += wzc[j] * wxc[k] * at(dData, dPitch, tx0 + k - 1, tz0 + j - 1, c);
                    }
                }
            }
            BankPoint& b = out[i];
            b.valid = true;
            b.ring = m;
            b.texelM = static_cast<float>(texel);
            b.dispX = float(d[0]); b.dispY = float(d[1]); b.dispZ = float(d[2]); b.foam = float(d[3]);
            b.level = float(p[0]); b.sigma2 = float(p[1]); b.curU = float(p[2]); b.curV = float(p[3]);
            // The detail layout: (gain1, dry, gain0, gain2).
            b.gain1 = float(g[0]); b.dry = float(g[1]); b.gain0 = float(g[2]); b.gain2 = float(g[3]);
            break;
        }
    }
}

void WaterBankLayer::TraceProbe(Gpu& gpu, double wx, double wz) {
    for (int m = 0; m < kMips; ++m) {
        if (!m_orgValid[m]) continue;
        const double texel = m_baseTexelM * (1 << m);
        const int tx = static_cast<int>((wx - m_orgX[m]) / texel);
        const int ty = static_cast<int>((wz - m_orgZ[m]) / texel);
        if (tx < 1 || ty < 1 || tx >= 511 || ty >= 511) continue;
        auto read4 = [&](TileAtlas2D& bank, float out[4]) {
            GpuTexture wrap;
            wrap.res = bank.Res();
            wrap.format = DXGI_FORMAT_R16G16B16A16_FLOAT;
            wrap.width = 3072;
            wrap.height = 512;
            wrap.state = m_state;
            uint32_t pitch = 0;
            const std::vector<uint8_t> data = gpu.ReadbackTexture(wrap, &pitch);
            const uint16_t* px = reinterpret_cast<const uint16_t*>(
                data.data() + static_cast<size_t>(pitch) * ty) + (m * 512 + tx) * 4;
            for (int i = 0; i < 4; ++i) out[i] = HalfF(px[i]);
        };
        float d[4], p[4], det[4];
        read4(m_disp, d);
        read4(m_param, p);
        read4(m_detail, det);
        Log("[trace] 9b bank ring %d texel %.1f m at (%d,%d): disp (%+.2f,%+.2f,%+.2f) "
            "foam %.2f | level %+.2f sigma2 %.4f cur (%+.2f,%+.2f) | hsScale*expo %.2f "
            "dry %.2f",
            m, texel, tx, ty, d[0], d[1], d[2], d[3], p[0], p[1], p[2], p[3], det[0],
            det[1]);
        return;
    }
    Log("[trace] 9b bank: point outside every resident ring");
}

void WaterBankLayer::DumpFibers(Gpu& gpu) {
    auto read = [&](TileAtlas2D& bank, std::vector<float>& out) {
        GpuTexture wrap;
        wrap.res = bank.Res();
        wrap.format = DXGI_FORMAT_R16G16B16A16_FLOAT;
        wrap.width = 3072;
        wrap.height = 512;
        wrap.state = m_state;
        uint32_t pitch = 0;
        const std::vector<uint8_t> data = gpu.ReadbackTexture(wrap, &pitch);
        out.resize(3072ull * 512 * 4);
        for (uint32_t y = 0; y < 512; ++y) {
            const uint16_t* px =
                reinterpret_cast<const uint16_t*>(data.data() + static_cast<size_t>(pitch) * y);
            for (uint32_t x = 0; x < 3072u * 4; ++x) {
                out[static_cast<size_t>(y) * 3072 * 4 + x] = HalfF(px[x]);
            }
        }
    };
    std::vector<float> disp, param, det;
    read(m_disp, disp);
    read(m_param, param);
    read(m_detail, det);
    auto save = [&](const wchar_t* path, auto shade) {
        std::vector<uint8_t> img(3072ull * 512 * 4);
        for (size_t i = 0; i < 3072ull * 512; ++i) {
            shade(i, &img[i * 4]);
            img[i * 4 + 3] = 255;
        }
        SavePng(path, img.data(), 3072, 512, 3072 * 4, img.size());
    };
    auto tone = [](float v, float scale) {
        return static_cast<uint8_t>(
            std::clamp(0.5f + 0.5f * v / scale, 0.0f, 1.0f) * 255.0f + 0.5f);
    };
    save(L"fiber_disp.png", [&](size_t i, uint8_t* o) {
        o[0] = tone(disp[i * 4 + 0], 2.0f);
        o[1] = tone(disp[i * 4 + 1], 2.0f);
        o[2] = tone(disp[i * 4 + 2], 2.0f);
    });
    save(L"fiber_foam.png", [&](size_t i, uint8_t* o) {
        const uint8_t g = static_cast<uint8_t>(std::clamp(disp[i * 4 + 3], 0.0f, 1.0f) * 255);
        o[0] = o[1] = o[2] = g;
    });
    save(L"fiber_param.png", [&](size_t i, uint8_t* o) {
        o[0] = tone(param[i * 4 + 0], 3.0f);
        o[1] = static_cast<uint8_t>(std::clamp(param[i * 4 + 1] * 24.0f, 0.0f, 1.0f) * 255);
        o[2] = tone(param[i * 4 + 2], 2.5f);
    });
    save(L"fiber_detail.png", [&](size_t i, uint8_t* o) {
        o[0] = static_cast<uint8_t>(std::clamp(det[i * 4] / 3.0f, 0.0f, 1.0f) * 255);
        o[1] = static_cast<uint8_t>(std::clamp(det[i * 4 + 1], 0.0f, 1.0f) * 255);
        o[2] = 0;
    });
    // The AST's declared ranges, enforced over the whole plane (zeros are NULL tiles and
    // legal everywhere; the check watches the extremes).
    struct RangeCheck {
        const char* what;
        float lo, hi, seenLo, seenHi;
        int idx;
        const std::vector<float>* buf;
    } checks[] = {
        {"disp.y (m, +-Hs/2*3)", -6.0f, 6.0f, 0, 0, 1, &disp},
        {"param.level (m NAVD)", -4.0f, 4.0f, 0, 0, 0, &param},
        {"param.sigma2", 0.0f, 0.12f, 0, 0, 1, &param},
        {"param.u (m/s)", -4.0f, 4.0f, 0, 0, 2, &param},
        {"detail.hsScale*expo", 0.0f, 3.0f, 0, 0, 0, &det},
        // M8: bands 0 and 2 join the detail plane (caustic Jacobian/Laplacian scales).
        // Gains carry shoaling (<=1.7) and current amplification (<=2.0) on top of
        // hsScale*expo <= 3 -- the declared ceiling is their product's practical bound.
        {"detail.gain0", 0.0f, 8.0f, 0, 0, 2, &det},
        {"detail.gain2", 0.0f, 8.0f, 0, 0, 3, &det},
    };
    bool ok = true;
    for (auto& c : checks) {
        c.seenLo = 1e9f;
        c.seenHi = -1e9f;
        for (size_t i = 0; i < 3072ull * 512; ++i) {
            const float v = (*c.buf)[i * 4 + c.idx];
            c.seenLo = (std::min)(c.seenLo, v);
            c.seenHi = (std::max)(c.seenHi, v);
        }
        const bool bad = c.seenLo < c.lo || c.seenHi > c.hi;
        if (bad) ok = false;
        Log("[fibers] %-22s seen [%+.3f, %+.3f]  declared [%+.1f, %+.1f]  %s", c.what,
            c.seenLo, c.seenHi, c.lo, c.hi, bad ? "OUT OF RANGE" : "ok");
    }
    Log("[fibers] exported fiber_disp/foam/param/detail.png (3072x512, rings left to "
        "right)%s", ok ? "" : " -- RANGE VIOLATIONS above");
    // M7p: raw planes + ring geometry, for proofs/inlet_storm.py's match report.
    if (FILE* fr = fopen("fiber_detail.f32", "wb")) {
        std::vector<float> plane(3072ull * 512);
        for (size_t i = 0; i < plane.size(); ++i) plane[i] = det[i * 4];
        fwrite(plane.data(), sizeof(float), plane.size(), fr);
        fclose(fr);
    }
    // M9c: the VERTICAL displacement plane, raw. The 8-bit fiber_disp.png quantizes a +-6 m
    // declared range to 47 mm a step, which cannot resolve a 14 cm chop band, and the
    // range statistic printed above is a MAX over every ring -- it is owned by the swell
    // and blind to whether short waves reached geometry at all. This is the plane to
    // measure texel-scale slope on when asking "did the fold let the chop through".
    if (FILE* fr = fopen("fiber_dispy.f32", "wb")) {
        std::vector<float> plane(3072ull * 512);
        for (size_t i = 0; i < plane.size(); ++i) plane[i] = disp[i * 4 + 1];
        fwrite(plane.data(), sizeof(float), plane.size(), fr);
        fclose(fr);
    }
    if (FILE* fj = fopen("fiber_meta.json", "wb")) {
        fprintf(fj, "{ \"baseTexelM\": %.3f, \"rings\": [", m_baseTexelM);
        for (int m = 0; m < kMips; ++m) {
            fprintf(fj, "%s{ \"org\": [%.1f, %.1f], \"texelM\": %.3f }",
                    m ? ", " : "", m_orgX[m], m_orgZ[m], m_baseTexelM * (1 << m));
        }
        fprintf(fj, "] }");
        fclose(fj);
    }
}

void WaterBankLayer::Render(const FrameContext& ctx) {
    if (!m_ready || !enabled || !m_sea) return;
    PixScope scope(ctx.cmd->Native(),
                   "waterbank (the wave vertex bank: rings recomposed per frame)");

    // The tile list: every wet tile in every ring, with its corner params from the stacks.
    const auto tileList0 = std::chrono::steady_clock::now();   // tileListMs bracket
    std::vector<BankTile> tiles;
    tiles.reserve(kMips * kRingTiles * kRingTiles);
    // The local sea state: the global Hs grid over the reference the cascades were synthesised
    // for (the Gulf point) -- mid-ocean waves track their OWN storm, not ours. One law with the
    // hull's twin (sim/WaveScale.h), carried at the tile's corners and lerped per texel like the
    // level and the bed, so the scale is continuous across tiles and across the grid's nodes.
    static const WaveScale kNone{};
    const WaveScale& waveScale = m_sea ? m_sea->Scale() : kNone;   // PHASE C2: the field
    for (int m = 0; m < kMips; ++m) {
        const double texel = m_baseTexelM * (1 << m);
        const double tileSpan = kTileTexels * texel;
        for (int ty = 0; ty < kRingTiles; ++ty) {
            for (int tx = 0; tx < kRingTiles; ++tx) {
                if (!m_wet[m][ty * kRingTiles + tx]) continue;
                BankTile t{};
                t.orgXZ[0] = static_cast<float>(m_orgX[m] + tx * tileSpan);
                t.orgXZ[1] = static_cast<float>(m_orgZ[m] + ty * tileSpan);
                t.texelM = static_cast<float>(texel);
                t.dstX = static_cast<uint32_t>(m * kRingTexels + tx * kTileTexels);
                t.dstY = static_cast<uint32_t>(ty * kTileTexels);
                for (int k = 0; k < 4; ++k) {
                    const double cx = t.orgXZ[0] + (k & 1) * tileSpan;
                    const double cz = t.orgXZ[1] + ((k >> 1) & 1) * tileSpan;
                    CornerParams(cx, cz, t.lvl[k], t.bed[k]);
                    // M8: under a --storm override the grid is STALE by definition -- the
                    // law answers 1 (the storm IS the reference). M7j/THE LOST LINE: every
                    // corner is written unconditionally; a tile that shipped 0 once flattened
                    // the whole sea, and only a CPU-vs-GPU cross-check found it.
                    double cLat = 0.0, cLon = 0.0;
                    PlaceOfRing(cx, cz, cLat, cLon);
                    t.hs[k] = static_cast<float>(waveScale.At(m_globe, cLat, cLon));
                }
                // M13 step 2: the tile's place rows -- the exact map at its origin, its tangent
                // map at its centre (the midpoint rule; see BankTile). The kernel forms every
                // texel's lat/lon from these instead of from the anchor-linear chart.
                {
                    const double ox = t.orgXZ[0], oz = t.orgXZ[1];   // the FLOAT origin the mesh
                    const double cx = ox + 0.5 * tileSpan;           // reads with, in doubles
                    const double cz = oz + 0.5 * tileSpan;
                    double la0 = 0.0, lo0 = 0.0, dLatDx = 0.0, dLonDx = 0.0, dLatDz = 0.0,
                           dLonDz = 0.0;
                    const Space::Anchor& chart = m_surface->flat;
                    const bool ok = chart.Exact() && chart.PlaceOfProjected(ox, oz, la0, lo0) &&
                                    chart.PlaceJacobianProjected(cx, cz, 0.5 * texel, dLatDx,
                                                                 dLonDx, dLatDz, dLonDz);
                    t.placeA[0] = static_cast<float>(la0);
                    t.placeA[1] = static_cast<float>(lo0);
                    t.placeA[2] = static_cast<float>(dLatDx);
                    t.placeA[3] = static_cast<float>(dLonDx);
                    t.placeB[0] = static_cast<float>(dLatDz);
                    t.placeB[1] = static_cast<float>(dLonDz);
                    t.placeB[2] = ok ? 1.0f : 0.0f;
                    t.placeB[3] = 0.0f;
                    // PHASE B2: THE TILE'S POINT for the windows' address, in doubles: the ground
                    // point (the radial through the ring point, at the planet's radius) less the
                    // rows' eye, in the tangent axes; at the origin, and its derivative at the
                    // centre by the midpoint rule.
                    {
                        const SurfaceFrame& sf = *m_surface;
                        auto pointOf = [&](double x, double z, double p[3]) {
                            double d[3];
                            if (!chart.DirOfProjected(x, z, d)) return false;
                            const double q[3] = {d[0] * sf.planetR - m_hwEye[0], d[1] * sf.planetR - m_hwEye[1],
                                                 d[2] * sf.planetR - m_hwEye[2]};
                            const double* ax[3] = {sf.east, sf.up, sf.north};
                            for (int c = 0; c < 3; ++c) p[c] = q[0] * ax[c][0] + q[1] * ax[c][1] + q[2] * ax[c][2];
                            return true;
                        };
                        double p0[3], pxp[3], pxm[3], pzp[3], pzm[3];
                        const double h = 0.5 * tileSpan;
                        const bool okP = ok && pointOf(ox, oz, p0) && pointOf(cx + h, cz, pxp) &&
                                         pointOf(cx - h, cz, pxm) && pointOf(cx, cz + h, pzp) &&
                                         pointOf(cx, cz - h, pzm);
                        for (int c = 0; c < 3; ++c) {
                            t.pointA[c] = okP ? static_cast<float>(p0[c]) : 0.0f;
                            t.pointX[c] = okP ? static_cast<float>((pxp[c] - pxm[c]) / (2.0 * h)) : 0.0f;
                            t.pointZ[c] = okP ? static_cast<float>((pzp[c] - pzm[c]) / (2.0 * h)) : 0.0f;
                        }
                        t.pointA[3] = okP ? 1.0f : 0.0f;
                        t.pointX[3] = t.pointZ[3] = 0.0f;
                    }

                    // ---- THE CASCADE SEA'S PLANES for this tile (sim/WaveChart.h). The
                    // neighbourhood is found at the tile's centre -- a tile is at most 5 km and a
                    // cell hundreds of km, so the four planes are the same for every texel in it
                    // -- and each plane's coordinate is sampled at the tile's origin and
                    // differenced across the tile, about its centre (the midpoint rule again).
                    // The shares are NOT baked: their two inputs are, so the kernel computes the
                    // same weights per texel that TreeWater computes per point.
                    double dC[3];
                    const bool haveCharts = ok && chart.DirOfProjected(cx, cz, dC);
                    t.bandY[3] = haveCharts ? 1.0f : 0.0f;
                    if (haveCharts) {
                        const WaveChart::Cell cell = m_waveChart.CellAt(dC);
                        WaveChart::Chart cc[WaveChart::kMax];
                        m_waveChart.At(cell, dC, cc);   // for the rotations at the tile's centre
                        const double hx = 0.5 * tileSpan;
                        // One plane's coordinate at a point of the ring, through the exact place.
                        auto uAt = [&](int k, double x, double z, double u[2]) {
                            double d[3];
                            if (!chart.DirOfProjected(x, z, d)) {
                                u[0] = u[1] = 0.0;
                                return;
                            }
                            const double p[3] = {d[0] * chart.planetR, d[1] * chart.planetR,
                                                 d[2] * chart.planetR};
                            WaveChart::UOf(cell.f[k], p, u);
                        };
                        for (int k = 0; k < WaveChart::kMax; ++k) {
                            double u0[2], uxp[2], uxm[2], uzp[2], uzm[2];
                            uAt(k, ox, oz, u0);
                            uAt(k, cx + hx, cz, uxp);
                            uAt(k, cx - hx, cz, uxm);
                            uAt(k, cx, cz + hx, uzp);
                            uAt(k, cx, cz - hx, uzm);
                            float* row = t.chart[k];
                            // The origin's coordinate, wrapped into each cascade's own patch: a
                            // float then carries metres inside a 756 m period instead of the
                            // hundreds of kilometres a cell can be across. Rows [0..3] hold
                            // cascades 0 and 1, [4..5] cascade 2 -- the float4 layout the kernel
                            // reads (ChartUv).
                            for (int c = 0; c < 3; ++c) {
                                const double L = m_sea ? double(m_sea->FftPatchL(c)) : 1.0;
                                const double Lc = (L > 1.0) ? L : 1.0;
                                row[c * 2 + 0] =
                                    static_cast<float>(u0[0] - std::floor(u0[0] / Lc) * Lc);
                                row[c * 2 + 1] =
                                    static_cast<float>(u0[1] - std::floor(u0[1] / Lc) * Lc);
                            }
                            row[6] = row[7] = 0.0f;
                            row[8] = static_cast<float>((uxp[0] - uxm[0]) / (2.0 * hx));   // du/dex
                            row[9] = static_cast<float>((uzp[0] - uzm[0]) / (2.0 * hx));   // du/dez
                            row[10] = static_cast<float>((uxp[1] - uxm[1]) / (2.0 * hx));  // dv/dex
                            row[11] = static_cast<float>((uzp[1] - uzm[1]) / (2.0 * hx));  // dv/dez
                            for (int r = 0; r < 4; ++r) {
                                row[12 + r] = static_cast<float>(cc[k].rot[r]);
                            }
                        }
                        // The shares' two inputs, as an affine map over the tile.
                        auto edgeAt = [&](double x, double z, double& ex, double& ey) {
                            double d[3];
                            if (!chart.DirOfProjected(x, z, d)) {
                                ex = ey = 0.0;
                                return;
                            }
                            m_waveChart.EdgeM(cell, d, ex, ey);
                        };
                        double e0x = 0.0, e0y = 0.0, exp1 = 0.0, eyp1 = 0.0, exm1 = 0.0, eym1 = 0.0;
                        double ezp1 = 0.0, ezpy = 0.0, ezm1 = 0.0, ezmy = 0.0;
                        edgeAt(ox, oz, e0x, e0y);
                        edgeAt(cx + hx, cz, exp1, eyp1);
                        edgeAt(cx - hx, cz, exm1, eym1);
                        edgeAt(cx, cz + hx, ezp1, ezpy);
                        edgeAt(cx, cz - hx, ezm1, ezmy);
                        t.bandX[0] = static_cast<float>(e0x);
                        t.bandX[1] = static_cast<float>((exp1 - exm1) / (2.0 * hx));
                        t.bandX[2] = static_cast<float>((ezp1 - ezm1) / (2.0 * hx));
                        t.bandX[3] = static_cast<float>(m_waveChart.bandM);
                        t.bandY[0] = static_cast<float>(e0y);
                        t.bandY[1] = static_cast<float>((eyp1 - eym1) / (2.0 * hx));
                        t.bandY[2] = static_cast<float>((ezpy - ezmy) / (2.0 * hx));
                    }
                }
                // M7j: exposure moved to the KERNEL, from the solver's own swell-shadow
                // field (the M7i x-ramp killed the channel and the open beaches -- a
                // hand-drawn boundary where a marched line-of-sight field already existed).
                tiles.push_back(t);
            }
        }
    }
    tileListMs = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() -
                                                           tileList0)
                     .count();
    if (tiles.empty()) return;

    BankCbData cb{};
    cb.org[0] = m_orgX[0];
    cb.org[1] = m_orgZ[0];
    cb.org[2] = m_baseTexelM;
    cb.org[3] = static_cast<float>(m_simUnix);
    for (int c = 0; c < 3; ++c) cb.patch[c] = m_sea->FftPatchL(c);
    cb.patch[3] = m_sea->heightScale;
    const double kCut[4] = {2.0 * kPiD / 756.0, 2.0 * kPiD / 60.0, 2.0 * kPiD / 12.0,
                            0.9 * kPiD * 256.0 / 47.0};
    for (int c = 0; c < 3; ++c) {
        cb.bandK[c] = static_cast<float>(std::sqrt(kCut[c] * kCut[c + 1]));
    }
    if (m_swe && m_swe->Ready()) {
        cb.sweDims[0] = static_cast<float>(m_swe->Nx());
        cb.sweDims[1] = static_cast<float>(m_swe->Ny());
        cb.sweDims[2] = 1.0f / m_swe->PadW();
        cb.sweDims[3] = 1.0f / m_swe->PadH();
        cb.slotsA[3] = m_swe->EtaSrv();
        cb.slotsB[0] = m_swe->UvSrv();
    }
    cb.misc[0] = static_cast<float>(kTileTexels);
    cb.misc[2] = static_cast<float>(injectPattern);
    for (int c = 0; c < 3; ++c) cb.slotsA[c] = m_sea->FftDispSrv(c);
    cb.slotsB[1] = m_disp.Uav();
    cb.slotsB[2] = m_param.Uav();
    cb.slotsB[3] = m_detail.Uav();
    // M7e: the foam memory -- the churn atlas joins the bank's inputs (16384 m domain
    // centred on the anchor, 2 m texels, no padding: 8192 square).
    cb.slotsC[0] = m_sea ? m_sea->ChurnAtlasSrv() : 0xFFFFFFFFu;
    // M9ba: the swell exposure PAGES (array SRV + residency map); the z14 slice, mips >= 3.
    cb.slotsC[1] = m_sea ? m_sea->ExposureSrv() : 0xFFFFFFFFu;
    cb.slotsC[2] = m_sea ? m_sea->ExposureResSrv() : 0xFFFFFFFFu;
    cb.slotsC[3] = 0xFFFFFFFFu;
    // M9az: the churn window follows the camera; the sea owns its origin.
    cb.churn[0] = m_sea ? m_sea->ChurnOriginX() : 0.0f;
    cb.churn[1] = m_sea ? m_sea->ChurnOriginZ() : 0.0f;
    cb.churn[2] = 1.0f / SeaLayer::ChurnDomainM();
    cb.churn[3] = SeaLayer::ChurnDomainM() / 2.0f;   // atlas texels along one axis
    cb.peakDir[0] = m_sea ? m_sea->PeakDirX() : 0.0f;
    cb.peakDir[1] = m_sea ? m_sea->PeakDirZ() : 0.0f;
    cb.peakDir[2] = (m_sea && m_sea->PeakDirValid()) ? 1.0f : 0.0f;
    cb.peakDir[3] = 0.0f;
    cb.slotsD[0] = m_hgtWinSrv;
    cb.slotsD[1] = m_hgtWinResSrv;
    cb.slotsD[2] = 0xFFFFFFFFu;   // PHASE B3: the page's slice lane, no page
    cb.slotsD[3] = 0xFFFFFFFFu;
    // M12 step 4b: the world.flat chart's row and the height window's row come from the
    // surface and the window's lattice (the old eight casts bit for bit; the [kernel] hash
    // below is the gate).
    m_surface->FlatRows(cb.geoA);
    // M8 foamlaw: the deriv fibers carry the Jacobian foam (the crest's area 2-blade
    // degenerating -- provably the same event the Miche steepness names), and the band
    // rms envelopes let the kernel normalize eta for the crest gate and depth excess.
    for (int c = 0; c < 3; ++c) cb.slotsE[c] = m_sea->FftDerivSrv(c);
    cb.slotsE[3] = 0xFFFFFFFFu;
    for (int c = 0; c < 3; ++c) cb.rmsRef[c] = m_sea->BandRms(c);
    cb.rmsRef[3] = 0.0f;
    for (int c = 0; c < 3; ++c) cb.bandKFold[c] = m_sea->BandKFold(c);
    cb.bandKFold[3] = 0.0f;
    for (int c = 0; c < 3; ++c) cb.bandKSpread[c] = m_sea->BandKSpread(c);
    cb.bandKSpread[3] = 0.0f;
    cb.sweB[0] = m_tidePlane;   // the solver is truth: its level is this plane plus its deviation
    cb.sweB[1] = cb.sweB[2] = cb.sweB[3] = 0.0f;
    static_assert(sizeof(SurfaceFrame::KernelWindowRows) == sizeof(float) * 80, "the kernels' rows");
    memcpy(cb.hwU, &m_hw, sizeof(m_hw));   // PHASE B2: hwU..hwS, KernelRows' packing
    memcpy(cb.svU, m_sv, sizeof(m_sv));   // svU..svO, SweDomain::KernelRows' packing
    cb.debugA[0] = flatBed ? 1.0f : 0.0f;   // M9p: the A/B that proves the bed moves geometry
    cb.debugA[1] = flatBedNavd;
    cb.debugA[2] = cb.debugA[3] = 0.0f;
    // M8 wavefield: the solved field's window + per-component table. The time rotor
    // (cos, sin)(sigma t) is computed HERE in doubles and reduced mod 2 pi -- sigma t
    // at unix scale would shred float precision in the kernel (the phase never wraps
    // on the GPU; the spinor arrives pre-advanced-ready, the cl2 law).
    // Scene closures (data/wave_scene.json, live): defaults when no scene is wired.
    const WaterSceneConfig defScene{};
    const WaterSceneConfig& sc2 = m_scene ? *m_scene : defScene;
    cb.foamA[0] = sc2.churnGain;
    cb.foamA[1] = sc2.shedSteepCap;
    cb.foamA[2] = sc2.shedMssCeil;
    cb.foamA[3] = sc2.crestLo;
    cb.foamB[0] = sc2.crestHi;
    cb.foamB[1] = sc2.depthLo;
    cb.foamB[2] = sc2.depthHi;
    cb.foamB[3] = m_sea->WindGate();   // Monahan: wind owns whitecap coverage
    memcpy(cb.boatA, m_boatA, sizeof(cb.boatA));   // M8: the fleet (zeros = no wake)
    memcpy(cb.boatB, m_boatB, sizeof(cb.boatB));
    cb.waveU[0] = 0xFFFFFFFFu;
    if (m_wave && m_wave->Ready() && m_wavePages != UINT32_MAX) {
        const WaveField::GpuTable& wt = m_wave->Table();
        cb.waveU[0] = m_wavePages;
        cb.waveU[1] = m_wavePagesRes;
        cb.waveU[2] = wt.nUsed;
        cb.waveU[3] = wt.envSlice;
        cb.waveA[0] = wt.orgX;
        cb.waveA[1] = wt.orgZ;
        cb.waveA[2] = wt.invCell;
        cb.waveA[3] = wt.feather;
        cb.waveP[0] = static_cast<float>(m_waveOrgPx[0]);
        cb.waveP[1] = static_cast<float>(m_waveOrgPx[1]);
        cb.waveP[2] = 1.0f / 16384.0f;
        cb.waveP[3] = static_cast<float>(65536.0 * 256.0);
        cb.waveD[0] = static_cast<float>(m_waveNx);
        cb.waveD[1] = static_cast<float>(m_waveNy);
        cb.waveD[2] = static_cast<float>(m_waveWinPx[0] - m_waveOrgPx[0]);   // the window's NW texel
        cb.waveD[3] = static_cast<float>(m_waveWinPx[1] - m_waveOrgPx[1]);   // in the page frame
        cb.waveB[0] = wt.envMax;
        cb.waveB[1] = wt.sumMax;
        cb.waveB[2] = sc2.wfChop;   // chop (scene cfg; ambient-sea lambda by default)
        cb.waveB[3] = wt.level;
        for (int c2 = 0; c2 < WaveField::kMaxComp; ++c2) {
            const double ang =
                std::fmod(static_cast<double>(wt.sigma[c2]) * m_simUnix, 2.0 * kPiD);
            // M9bl: comps 0..15 ride the original rows, 16..31 the appended ones -- the
            // same split the HLSL's WaveSigRow/WaveDirRow/WaveScaleRow read back.
            const int half = (c2 < 16) ? 0 : 1;
            const int cH = c2 - half * 16;
            const int r4 = (cH >> 1) * 4 + (cH & 1) * 2;
            float* sig = half ? cb.waveSig2 : cb.waveSig;
            float* dir = half ? cb.waveDir2 : cb.waveDir;
            float* scl = half ? cb.waveScale2 : cb.waveScale;
            sig[r4] = static_cast<float>(std::cos(ang));
            sig[r4 + 1] = static_cast<float>(std::sin(ang));
            dir[r4] = wt.dirX[c2];
            dir[r4 + 1] = wt.dirZ[c2];
            scl[r4] = wt.aMax[c2];
            scl[r4 + 1] = wt.kMax[c2];
        }
    }

    GpuScope gscope(ctx.prof, ctx.cmd->Native(), "waterbank.fill");   // barriers + the one dispatch
    auto toUav = [&](TileAtlas2D& bank) {
        if (m_state == D3D12_RESOURCE_STATE_UNORDERED_ACCESS) return;
        ctx.cmd->Barrier(bank.Res(), m_state, D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
    };
    toUav(m_disp);
    toUav(m_param);
    toUav(m_detail);
    m_state = D3D12_RESOURCE_STATE_UNORDERED_ACCESS;

    {   // M12 step 4b instrument: the bank's constant buffer, fingerprinted at its upload --
        // the gate for the lattice-row moves (winA, geoA from the surface) and for the fills
        // of 4e/4f. Logs when the hash changes, as the [surface] fills do; a member, not a
        // static, because the Droste set-B bank is a second instance.
        const uint64_t h = Fnv1aBytes(&cb, sizeof(cb));
        if (h != m_cbFp) {
            m_cbFp = h;
            Log("[kernel] waterbank cb FNV-1a %016llx", static_cast<unsigned long long>(h));
        }
    }
    ctx.cmd->ComputeRoot(m_rs.Get());
    ctx.cmd->ComputeConstants(0, cb);
    ctx.cmd->ComputeSrvAt(
        1, ctx.gpu->PushConstants(tiles.data(), tiles.size() * sizeof(BankTile)));
    ctx.cmd->ComputeBindless(2);
    ctx.cmd->ComputeBindless(3);
    ctx.cmd->Pipeline(m_fill.Get());
    ctx.cmd->Dispatch(kTileTexels / 16, kTileTexels / 16,
                      static_cast<UINT>(tiles.size()));
    // PHASE B0: THE BANK'S BED TRACE (the header): the same kernel on the same rows and tiles,
    // compiled with HP_TRACE and BANK_TRACE, writing its readings to a target of its own.
    if (traceOn && m_sc) {
        if (!m_traceK) {
            m_traceK = hal::BuildCompute(
                *ctx.gpu, m_rs.Get(),
                m_sc->Compile(m_shaderDir + L"/WaterBank.hlsl", L"CsBankFill", L"cs_6_0",
                              {L"HP_TRACE=1", L"BANK_TRACE=1"}),
                "waterbank.trace");
            m_traceTex = ctx.gpu->CreateTexture2D(kMips * kRingTexels, kRingTexels,
                                                  DXGI_FORMAT_R32G32B32A32_FLOAT,
                                                  D3D12_RESOURCE_FLAG_ALLOW_UNORDERED_ACCESS,
                                                  D3D12_RESOURCE_STATE_UNORDERED_ACCESS,
                                                  L"waterbank.trace (bed, slice*16+mip, depth, |dy|)");
            m_traceUav = hal::Uav2D(*ctx.gpu, m_traceTex.res.Get(), DXGI_FORMAT_R32G32B32A32_FLOAT);
        }
        if (m_traceK) {
            BankCbData tcb = cb;
            tcb.slotsC[3] = m_traceUav;
            tcb.debugA[3] = traceFloor;
            ctx.cmd->ComputeConstants(0, tcb);
            ctx.cmd->Pipeline(m_traceK.Get());
            ctx.cmd->Dispatch(kTileTexels / 16, kTileTexels / 16, static_cast<UINT>(tiles.size()));
            m_traceTiles = tiles;
            m_traceFloorUsed = traceFloor;
        }
    }

    auto toSrv = [&](TileAtlas2D& bank) {
        ctx.cmd->Barrier(bank.Res(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS,
                         D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE |
                             D3D12_RESOURCE_STATE_PIXEL_SHADER_RESOURCE);
    };
    toSrv(m_disp);
    toSrv(m_param);
    toSrv(m_detail);
    m_state = D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE |
              D3D12_RESOURCE_STATE_PIXEL_SHADER_RESOURCE;

    char s[96];
    snprintf(s, sizeof(s), "bank %zu/%d t %.0f KB", tiles.size(),
             kMips * kRingTiles * kRingTiles,
             tiles.size() * 2.0 * 64.0 * 2.0);
    stats = s;
}

// PHASE B0: THE BANK'S BED TRACE, read back (the header). Per ring: where it stands (its centre's
// place), the bed's sources and mips by share of the ring's texels, the bed, the depth, and the
// breaking clamp's hmax = 0.55 max(depth, 0.05) over the wet texels with the share it cut (|dy|
// past hmax); then the centre texel of ring 0, the eye's own. Lens images: <label>_hmax.png (0 m
// dark blue .. 4 m red, no tile black) and <label>_source.png (the cube violet, the z14 page green,
// the z17 page red, a window by its rank's hue -- cyan, green, red, yellow, white for ranks 1..5 --
// the corner lerp grey; darker by the mip read); the raw planes <label>_trace.f32 (3072 x 512 x 4).
bool WaterBankLayer::TraceRead(Gpu& gpu, const std::string& dir, const std::string& label) {
    if (!m_traceK || m_traceTiles.empty()) {
        Log("[banktrace] %s: no trace was dispatched", label.c_str());
        return false;
    }
    uint32_t pitch = 0;
    const std::vector<uint8_t> raw = gpu.ReadbackTexture(m_traceTex, &pitch);
    const uint32_t W = kMips * kRingTexels, H = kRingTexels;
    std::vector<float> plane(size_t(W) * H * 4, 0.0f);
    std::vector<uint8_t> have(size_t(W) * H, 0);
    for (const BankTile& t : m_traceTiles) {
        for (uint32_t y = 0; y < uint32_t(kTileTexels); ++y) {
            const float* row = reinterpret_cast<const float*>(&raw[size_t(t.dstY + y) * pitch]);
            for (uint32_t x = 0; x < uint32_t(kTileTexels); ++x) {
                const size_t i = size_t(t.dstY + y) * W + (t.dstX + x);
                for (int c = 0; c < 4; ++c) plane[i * 4 + c] = row[(t.dstX + x) * 4 + c];
                have[i] = 1;
            }
        }
    }
    std::error_code ec;
    std::filesystem::create_directories(dir, ec);
    if (FILE* f = nullptr; fopen_s(&f, (dir + "/" + label + "_trace.f32").c_str(), "wb") == 0 && f) {
        fwrite(plane.data(), sizeof(float), plane.size(), f);
        fclose(f);
    }
    auto sourceName = [&](uint32_t slice) {
        char b[48];
        if (slice == 255u) return std::string("corner-lerp");
        if (slice < 6u) snprintf(b, sizeof(b), "cube f%u", slice);
        else if (slice == SurfaceFrame::kStandingSlice) snprintf(b, sizeof(b), "standing window");
        else snprintf(b, sizeof(b), "window set %u r%u", (slice - 6u) / SurfaceFrame::kMaxRanks,
                      (slice - 6u) % SurfaceFrame::kMaxRanks + 1u);
        return std::string(b);
    };
    auto pct = [](std::vector<float>& v, double q) {
        if (v.empty()) return 0.0f;
        const size_t k = size_t(q * double(v.size() - 1));
        std::nth_element(v.begin(), v.begin() + k, v.end());
        return v[k];
    };
    Log("[banktrace] %s%s: %zu tiles traced", label.c_str(),
        m_traceFloorUsed > 0.0f ? " [PLANTED: the rule's residency floor forced to mip 6]" : "",
        m_traceTiles.size());
    for (int m = 0; m < kMips; ++m) {
        const double texel = m_baseTexelM * (1 << m);
        const double cx = m_orgX[m] + 0.5 * kRingTexels * texel;
        const double cz = m_orgZ[m] + 0.5 * kRingTexels * texel;
        double lat = 0.0, lon = 0.0;
        PlaceOfRing(cx, cz, lat, lon);
        std::map<uint32_t, uint64_t> src;
        std::vector<float> bed, depth, hmax;
        uint64_t n = 0, cut = 0, wet = 0;
        for (uint32_t y = 0; y < H; ++y) {
            for (uint32_t x = uint32_t(m) * kRingTexels; x < uint32_t(m + 1) * kRingTexels; ++x) {
                const size_t i = size_t(y) * W + x;
                if (!have[i]) continue;
                ++n;
                const float* p = &plane[i * 4];
                const uint32_t code = uint32_t((std::max)(p[1], 0.0f) + 0.5f);
                ++src[code];
                bed.push_back(p[0]);
                depth.push_back(p[2]);
                const float hm = 0.55f * (std::max)(p[2], 0.05f);
                if (p[2] > 0.05f) {
                    ++wet;
                    hmax.push_back(hm);
                }
                if (p[3] > hm) ++cut;
            }
        }
        if (!n) {
            Log("[banktrace]   ring %d (%.1f m texels): centre %.5f N %.5f E -- no wet tile", m,
                texel, lat, lon);
            continue;
        }
        std::string s;
        for (const auto& [code, c] : src) {
            char b[96];
            snprintf(b, sizeof(b), " %s m%u %.1f%%", sourceName(code / 16u).c_str(), code % 16u,
                     100.0 * double(c) / double(n));
            s += b;
        }
        Log("[banktrace]   ring %d (%.1f m texels): centre %.5f N %.5f E, %llu texels | the bed "
            "read%s | bed p5 %+.2f p50 %+.2f p95 %+.2f m | depth p50 %.2f m | hmax over the wet "
            "(%llu) p5 %.2f p50 %.2f p95 %.2f m | the clamp cut %.2f%% of the texels",
            m, texel, lat, lon, static_cast<unsigned long long>(n), s.c_str(), pct(bed, 0.05),
            pct(bed, 0.5), pct(bed, 0.95), pct(depth, 0.5), static_cast<unsigned long long>(wet),
            pct(hmax, 0.05), pct(hmax, 0.5), pct(hmax, 0.95), 100.0 * double(cut) / double(n));
    }
    {   // ring 0's centre texel: the eye's own
        const size_t i = size_t(H / 2) * W + kRingTexels / 2;
        const float* p = &plane[i * 4];
        const uint32_t code = uint32_t((std::max)(p[1], 0.0f) + 0.5f);
        Log("[banktrace]   the eye's texel (ring 0 centre): %s, bed %+.3f m (%s m%u), depth %.3f "
            "m, hmax %.3f m, |dy| before the clamp %.3f m",
            have[i] ? "wet tile" : "NO TILE", p[0], sourceName(code / 16u).c_str(), code % 16u,
            p[2], 0.55f * (std::max)(p[2], 0.05f), p[3]);
    }
    // The lens images.
    std::vector<uint8_t> hm(size_t(W) * H * 4, 0), sc(size_t(W) * H * 4, 0);
    const float ramp[5][3] = {{20, 30, 120}, {20, 190, 220}, {40, 200, 60}, {240, 220, 30},
                              {230, 40, 30}};
    const float hue[9][3] = {{0.60f, 0.25f, 1.00f}, {0.15f, 1.00f, 0.30f}, {1.00f, 0.30f, 0.10f},
                             {0.10f, 0.75f, 1.00f}, {0.15f, 1.00f, 0.30f}, {1.00f, 0.30f, 0.10f},
                             {1.00f, 0.92f, 0.15f}, {1.00f, 1.00f, 1.00f}, {0.5f, 0.5f, 0.5f}};
    for (size_t i = 0; i < size_t(W) * H; ++i) {
        hm[i * 4 + 3] = sc[i * 4 + 3] = 255;
        if (!have[i]) continue;
        const float* p = &plane[i * 4];
        const float t = std::clamp(0.55f * (std::max)(p[2], 0.05f) / 4.0f, 0.0f, 1.0f) * 4.0f;
        const int a = (std::min)(int(t), 3);
        const float f = t - float(a);
        for (int c = 0; c < 3; ++c) {
            hm[i * 4 + c] = uint8_t(ramp[a][c] + (ramp[a + 1][c] - ramp[a][c]) * f);
        }
        const uint32_t code = uint32_t((std::max)(p[1], 0.0f) + 0.5f);
        const uint32_t slice = code / 16u, mip = code % 16u;
        int h = 8;   // the corner lerp
        if (slice < 6u) h = 0;
        else if (slice != 255u) h = 3 + int((slice - 6u) % SurfaceFrame::kMaxRanks);
        const float b = 1.0f - float(mip) / 9.0f;
        for (int c = 0; c < 3; ++c) sc[i * 4 + c] = uint8_t(hue[h][c] * b * 255.0f);
    }
    const std::wstring wdir(dir.begin(), dir.end()), wl(label.begin(), label.end());
    SavePng(wdir + L"/" + wl + L"_hmax.png", hm.data(), W, H, W * 4, hm.size());
    SavePng(wdir + L"/" + wl + L"_source.png", sc.data(), W, H, W * 4, sc.size());
    return true;
}

}  // namespace ga
