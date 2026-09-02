#include "scene/WaterBankLayer.h"

#include "core/Image.h"
#include "core/PixEvents.h"
#include "scene/SeaLayer.h"
#include "core/SceneConfig.h"
#include "sim/WaveField.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>

namespace ga {

namespace {
constexpr double kPiD = 3.14159265358979;
constexpr double kD2R = kPiD / 180.0;
}  // namespace

void WaterBankLayer::Configure(const std::wstring& shaderDir, SeaLayer* sea, SweSolver* swe,
                               const BathyModel* sweBathy, const WaterAtlas* atlas,
                               Compositor* comp, int hgtCh, const GlobeModel* globe,
                               const SeaState* seaState) {
    m_shaderDir = shaderDir;
    m_sea = sea;
    m_swe = swe;
    m_sweBathy = sweBathy;
    m_atlas = atlas;
    m_comp = comp;
    m_hgtCh = hgtCh;
    m_globe = globe;
    m_seaState = seaState;
}

void WaterBankLayer::Init(Gpu& gpu, ShaderCompiler& sc, FieldSet&, ID3D12RootSignature*) {
    if (!m_sea) return;
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
    D3D12_DESCRIPTOR_RANGE1 rs[2]{}, ru[1]{};
    rs[0].RangeType = D3D12_DESCRIPTOR_RANGE_TYPE_SRV;
    rs[0].NumDescriptors = UINT_MAX;
    rs[0].BaseShaderRegister = 0;
    rs[0].RegisterSpace = 1;
    rs[0].Flags = D3D12_DESCRIPTOR_RANGE_FLAG_DESCRIPTORS_VOLATILE;
    rs[0].OffsetInDescriptorsFromTableStart = 0;
    ru[0].RangeType = D3D12_DESCRIPTOR_RANGE_TYPE_UAV;
    ru[0].NumDescriptors = UINT_MAX;
    ru[0].BaseShaderRegister = 0;
    ru[0].RegisterSpace = 2;
    // M9aq: t0, space5 -- the heap as Texture2DArray, so the bed can be read from slice 6 of
    // the height PAGE tenant (one height texture; the window is a page of it).
    rs[1] = rs[0];
    rs[1].RegisterSpace = 5;
    ru[0].Flags = D3D12_DESCRIPTOR_RANGE_FLAG_DESCRIPTORS_VOLATILE;
    ru[0].OffsetInDescriptorsFromTableStart = 0;
    D3D12_ROOT_PARAMETER1 params[4]{};
    params[0].ParameterType = D3D12_ROOT_PARAMETER_TYPE_CBV;
    params[0].Descriptor.ShaderRegister = 0;
    params[1].ParameterType = D3D12_ROOT_PARAMETER_TYPE_SRV;
    params[1].Descriptor.ShaderRegister = 0;
    params[2].ParameterType = D3D12_ROOT_PARAMETER_TYPE_DESCRIPTOR_TABLE;
    params[2].DescriptorTable.NumDescriptorRanges = 2;
    params[2].DescriptorTable.pDescriptorRanges = rs;
    params[3].ParameterType = D3D12_ROOT_PARAMETER_TYPE_DESCRIPTOR_TABLE;
    params[3].DescriptorTable.NumDescriptorRanges = 1;
    params[3].DescriptorTable.pDescriptorRanges = ru;
    D3D12_STATIC_SAMPLER_DESC samp[2]{};
    samp[0].Filter = D3D12_FILTER_MIN_MAG_MIP_LINEAR;
    samp[0].AddressU = samp[0].AddressV = samp[0].AddressW =
        D3D12_TEXTURE_ADDRESS_MODE_WRAP;
    samp[0].ShaderRegister = 0;
    samp[0].ShaderVisibility = D3D12_SHADER_VISIBILITY_ALL;
    samp[1] = samp[0];
    samp[1].AddressU = samp[1].AddressV = samp[1].AddressW =
        D3D12_TEXTURE_ADDRESS_MODE_CLAMP;
    samp[1].ShaderRegister = 1;
    D3D12_VERSIONED_ROOT_SIGNATURE_DESC vd{};
    vd.Version = D3D_ROOT_SIGNATURE_VERSION_1_1;
    vd.Desc_1_1.NumParameters = _countof(params);
    vd.Desc_1_1.pParameters = params;
    vd.Desc_1_1.NumStaticSamplers = 2;
    vd.Desc_1_1.pStaticSamplers = samp;
    Com<ID3DBlob> blob, err;
    GA_CHECK(D3D12SerializeVersionedRootSignature(&vd, &blob, &err));
    GA_CHECK(gpu.Device()->CreateRootSignature(0, blob->GetBufferPointer(),
                                              blob->GetBufferSize(), IID_PPV_ARGS(&m_rs)));
    m_rs->SetName(L"water bank root signature");

    ShaderBlob cs = sc.Compile(m_shaderDir + L"/WaterBank.hlsl", L"CsBankFill", L"cs_6_0");
    if (!cs.Valid()) throw std::runtime_error("WaterBank kernel failed");
    D3D12_COMPUTE_PIPELINE_STATE_DESC d{};
    d.pRootSignature = m_rs.Get();
    d.CS = {cs.Data(), cs.Size()};
    GA_CHECK(gpu.Device()->CreateComputePipelineState(&d, IID_PPV_ARGS(&m_fill)));
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
    ShaderBlob cs = sc.Compile(m_shaderDir + L"/WaterBank.hlsl", L"CsBankFill", L"cs_6_0");
    if (!cs.Valid()) return;
    D3D12_COMPUTE_PIPELINE_STATE_DESC d{};
    d.pRootSignature = m_rs.Get();
    d.CS = {cs.Data(), cs.Size()};
    Com<ID3D12PipelineState> pso;
    if (SUCCEEDED(gpu.Device()->CreateComputePipelineState(&d, IID_PPV_ARGS(&pso)))) {
        m_fill = pso;
    }
}

void WaterBankLayer::SetFrame(Gpu& gpu, double simUnix, double camX, double camZ) {
    m_simUnix = simUnix;
    m_camX = camX;
    m_camZ = camZ;
    if (!m_ready || !enabled) return;
    for (int m = 0; m < kMips; ++m) ReanchorRing(gpu, m, camX, camZ);
}

bool WaterBankLayer::TileWet(double wx0, double wz0, double spanM) const {
    if (!m_comp || m_hgtCh < 0) return true;
    // Five-point test against the one height stack: any point at or below the high-water
    // margin keeps the tile; a tile of pure upland stays NULL.
    for (int k = 0; k < 5; ++k) {
        const double fx = (k == 4) ? 0.5 : (k & 1) ? 0.96 : 0.04;
        const double fz = (k == 4) ? 0.5 : (k & 2) ? 0.96 : 0.04;
        const double lat = BathyModel::kOrgLat + (wz0 + fz * spanM) / BathyModel::kMPerLat;
        const double lon = BathyModel::kOrgLon + (wx0 + fx * spanM) / BathyModel::kMPerLon;
        if (m_comp->SampleHeightStack(m_hgtCh, lat * kD2R, lon * kD2R, spanM * 0.25) < 2.5f) {
            return true;
        }
    }
    return false;
}

void WaterBankLayer::CornerParams(double wx, double wz, float& lvl, float& bed) const {
    const double lat = BathyModel::kOrgLat + wz / BathyModel::kMPerLat;
    const double lon = BathyModel::kOrgLon + wx / BathyModel::kMPerLon;
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
    PixScope scope(ctx.cl, "waterbank (the wave vertex bank: rings recomposed per frame)");

    // The tile list: every wet tile in every ring, with its corner params from the stacks.
    std::vector<BankTile> tiles;
    tiles.reserve(kMips * kRingTiles * kRingTiles);
    const double hsRef =
        (m_seaState && m_seaState->Ready())
            ? (std::max)(m_seaState->Hour(m_seaState->HourIndex(m_simUnix)).combinedHs, 0.3)
            : 1.0;
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
                    CornerParams(t.orgXZ[0] + (k & 1) * tileSpan,
                                 t.orgXZ[1] + ((k >> 1) & 1) * tileSpan, t.lvl[k], t.bed[k]);
                }
                // The local sea state: the global Hs grid over the reference the cascades
                // were synthesised for (the Gulf point) -- mid-ocean waves track their OWN
                // storm, not ours.
                float hsScale = 1.0f;
                // M8: under a --storm override the grid is STALE by definition (the
                // override rewrote the partitions, not the product) -- ratioing grid Hs
                // against the storm reference painted a 17x tile-quantized staircase
                // along the grid's land/sea edge (the pale rectangles; the top-down
                // lens convicted it). The storm IS the reference: hsScale = 1.
                if (!(m_sea && m_sea->StormOn()) && m_globe && m_globe->WavesNx() > 0) {
                    const double lat =
                        BathyModel::kOrgLat + (t.orgXZ[1] + tileSpan * 0.5) /
                                                  BathyModel::kMPerLat;
                    double lon = BathyModel::kOrgLon + (t.orgXZ[0] + tileSpan * 0.5) /
                                                           BathyModel::kMPerLon;
                    if (m_globe->WavesLon1() > 180.0 && lon < 0.0) lon += 360.0;
                    const int nx = m_globe->WavesNx(), ny = m_globe->WavesNy();
                    const int ix = static_cast<int>(
                        (lon - (m_globe->WavesLon1() - nx * m_globe->WavesDLon())) /
                        m_globe->WavesDLon());
                    const int iy = static_cast<int>((m_globe->WavesLat1() - lat) /
                                                    m_globe->WavesDLat());
                    if (ix >= 0 && iy >= 0 && ix < nx && iy < ny) {
                        const float hs = m_globe->Hs()[static_cast<size_t>(iy) * nx + ix];
                        if (hs >= 0.0f) {
                            hsScale = static_cast<float>(
                                std::clamp(hs / hsRef, 0.15, 3.0));
                        }
                    }
                }
                // M7j: exposure moved to the KERNEL, from the solver's own swell-shadow
                // field (the M7i x-ramp killed the channel and the open beaches -- a
                // hand-drawn boundary where a marched line-of-sight field already existed).
                // THE LOST LINE: M7i's ramp edit swallowed this unconditional assignment
                // and every tile shipped hsScale 0 -- the "water seems worse" report was
                // a dead-flat sea, found by the hypervisor's CPU-vs-GPU cross-check
                // (expected 0.42, bank said 0.00) in one probe.
                t.hsScale = hsScale;
                tiles.push_back(t);
            }
        }
    }
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
    if (m_swe && m_swe->Ready() && m_sweBathy) {
        cb.swe[0] = m_sweBathy->WorldX0();
        cb.swe[1] = m_sweBathy->WorldZ0();
        cb.swe[2] = 1.0f / m_sweBathy->WorldSizeX();
        cb.swe[3] = 1.0f / m_sweBathy->WorldSizeZ();
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
    cb.slotsD[2] = m_hgtWinSlice;   // M9aq: the page, or ~0 for the old window
    cb.slotsD[3] = 0xFFFFFFFFu;
    cb.geoA[0] = static_cast<float>(BathyModel::kOrgLat);
    cb.geoA[1] = static_cast<float>(BathyModel::kOrgLon);
    cb.geoA[2] = static_cast<float>(1.0 / BathyModel::kMPerLat);
    cb.geoA[3] = static_cast<float>(1.0 / BathyModel::kMPerLon);
    cb.winA[0] = static_cast<float>(m_hgtWinOrg[0]);
    cb.winA[1] = static_cast<float>(m_hgtWinOrg[1]);
    cb.winA[2] = 1.0f / 16384.0f;
    cb.winA[3] = 16384.0f * 256.0f;
    // M8 foamlaw: the deriv fibers carry the Jacobian foam (the crest's area 2-blade
    // degenerating -- provably the same event the Miche steepness names), and the band
    // rms envelopes let the kernel normalize eta for the crest gate and depth excess.
    for (int c = 0; c < 3; ++c) cb.slotsE[c] = m_sea->FftDerivSrv(c);
    cb.slotsE[3] = 0xFFFFFFFFu;
    for (int c = 0; c < 3; ++c) cb.rmsRef[c] = m_sea->BandRms(c);
    cb.rmsRef[3] = 0.0f;
    for (int c = 0; c < 3; ++c) cb.bandKFold[c] = m_sea->BandKFold(c);
    cb.bandKFold[3] = 0.0f;
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
    if (m_wave && m_wave->Ready()) {
        const WaveField::GpuTable& wt = m_wave->Table();
        cb.waveU[0] = m_wave->Srv();
        cb.waveU[1] = wt.nx;
        cb.waveU[2] = wt.ny;
        cb.waveU[3] = WaveField::kMaxComp;
        cb.waveA[0] = wt.orgX;
        cb.waveA[1] = wt.orgZ;
        cb.waveA[2] = wt.invCell;
        cb.waveA[3] = wt.feather;
        cb.waveB[0] = wt.envMax;
        cb.waveB[1] = wt.sumMax;
        cb.waveB[2] = sc2.wfChop;   // chop (scene cfg; ambient-sea lambda by default)
        cb.waveB[3] = wt.level;
        for (int c2 = 0; c2 < WaveField::kMaxComp; ++c2) {
            const double ang =
                std::fmod(static_cast<double>(wt.sigma[c2]) * m_simUnix, 2.0 * kPiD);
            const int r4 = (c2 >> 1) * 4 + (c2 & 1) * 2;
            cb.waveSig[r4] = static_cast<float>(std::cos(ang));
            cb.waveSig[r4 + 1] = static_cast<float>(std::sin(ang));
            cb.waveDir[r4] = wt.dirX[c2];
            cb.waveDir[r4 + 1] = wt.dirZ[c2];
            cb.waveScale[r4] = wt.aMax[c2];
            cb.waveScale[r4 + 1] = wt.kMax[c2];
        }
    }

    auto toUav = [&](TileAtlas2D& bank) {
        if (m_state == D3D12_RESOURCE_STATE_UNORDERED_ACCESS) return;
        D3D12_RESOURCE_BARRIER b{};
        b.Type = D3D12_RESOURCE_BARRIER_TYPE_TRANSITION;
        b.Transition.pResource = bank.Res();
        b.Transition.Subresource = D3D12_RESOURCE_BARRIER_ALL_SUBRESOURCES;
        b.Transition.StateBefore = m_state;
        b.Transition.StateAfter = D3D12_RESOURCE_STATE_UNORDERED_ACCESS;
        ctx.cl->ResourceBarrier(1, &b);
    };
    toUav(m_disp);
    toUav(m_param);
    toUav(m_detail);
    m_state = D3D12_RESOURCE_STATE_UNORDERED_ACCESS;

    ctx.cl->SetComputeRootSignature(m_rs.Get());
    ctx.cl->SetComputeRootConstantBufferView(0, ctx.gpu->PushConstants(&cb, sizeof(cb)));
    ctx.cl->SetComputeRootShaderResourceView(
        1, ctx.gpu->PushConstants(tiles.data(), tiles.size() * sizeof(BankTile)));
    ctx.cl->SetComputeRootDescriptorTable(2, ctx.gpu->SrvHeap().Gpu(0));
    ctx.cl->SetComputeRootDescriptorTable(3, ctx.gpu->SrvHeap().Gpu(0));
    ctx.cl->SetPipelineState(m_fill.Get());
    ctx.cl->Dispatch(kTileTexels / 16, kTileTexels / 16,
                     static_cast<UINT>(tiles.size()));

    auto toSrv = [&](TileAtlas2D& bank) {
        D3D12_RESOURCE_BARRIER b{};
        b.Type = D3D12_RESOURCE_BARRIER_TYPE_TRANSITION;
        b.Transition.pResource = bank.Res();
        b.Transition.Subresource = D3D12_RESOURCE_BARRIER_ALL_SUBRESOURCES;
        b.Transition.StateBefore = D3D12_RESOURCE_STATE_UNORDERED_ACCESS;
        b.Transition.StateAfter = D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE |
                                  D3D12_RESOURCE_STATE_PIXEL_SHADER_RESOURCE;
        ctx.cl->ResourceBarrier(1, &b);
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

}  // namespace ga
