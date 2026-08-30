#include "scene/WaterBankLayer.h"

#include "core/PixEvents.h"
#include "scene/SeaLayer.h"

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
    D3D12_DESCRIPTOR_RANGE1 rs[1]{}, ru[1]{};
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
    ru[0].Flags = D3D12_DESCRIPTOR_RANGE_FLAG_DESCRIPTORS_VOLATILE;
    ru[0].OffsetInDescriptorsFromTableStart = 0;
    D3D12_ROOT_PARAMETER1 params[4]{};
    params[0].ParameterType = D3D12_ROOT_PARAMETER_TYPE_CBV;
    params[0].Descriptor.ShaderRegister = 0;
    params[1].ParameterType = D3D12_ROOT_PARAMETER_TYPE_SRV;
    params[1].Descriptor.ShaderRegister = 0;
    params[2].ParameterType = D3D12_ROOT_PARAMETER_TYPE_DESCRIPTOR_TABLE;
    params[2].DescriptorTable.NumDescriptorRanges = 1;
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
                if (m_globe && m_globe->WavesNx() > 0) {
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
                // M7i: EXPOSURE. The FFT sea is fetch-blind -- without this, the harbor
                // basin whitecaps as hard as the open bar, and sheltered-water foam reads
                // as a mirrored ocean (the user's "is the water inverted?" report: it was
                // not inverted, it was un-sheltered). Inside the solver window, ocean
                // swell attenuates west of the throat on the same x-ramp the jet and the
                // churn already use; locally generated chop keeps an 0.18 floor.
                if (m_sweBathy && m_sweBathy->Ready()) {
                    const double cx = t.orgXZ[0] + tileSpan * 0.5;
                    const double cz = t.orgXZ[1] + tileSpan * 0.5;
                    if (cx > m_sweBathy->WorldX0() &&
                        cx < m_sweBathy->WorldX0() + m_sweBathy->WorldSizeX() &&
                        cz > m_sweBathy->WorldZ0() &&
                        cz < m_sweBathy->WorldZ0() + m_sweBathy->WorldSizeZ()) {
                        const double s = std::clamp((cx - 250.0) / 800.0, 0.0, 1.0);
                        t.hsScale = hsScale * static_cast<float>(
                            0.18 + 0.82 * s * s * (3.0 - 2.0 * s));
                    }
                }
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
    for (int c = 0; c < 3; ++c) cb.slotsA[c] = m_sea->FftDispSrv(c);
    cb.slotsB[1] = m_disp.Uav();
    cb.slotsB[2] = m_param.Uav();
    cb.slotsB[3] = m_detail.Uav();
    // M7e: the foam memory -- the churn atlas joins the bank's inputs (16384 m domain
    // centred on the anchor, 2 m texels, no padding: 8192 square).
    cb.slotsC[0] = m_sea ? m_sea->ChurnAtlasSrv() : 0xFFFFFFFFu;
    cb.slotsC[1] = cb.slotsC[2] = cb.slotsC[3] = 0xFFFFFFFFu;
    cb.churn[0] = -8192.0f;
    cb.churn[1] = -8192.0f;
    cb.churn[2] = 1.0f / 16384.0f;
    cb.churn[3] = 8192.0f;

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
