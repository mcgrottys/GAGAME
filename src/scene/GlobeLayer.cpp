#include "scene/GlobeLayer.h"

#include "core/PixEvents.h"
#include "core/Shader.h"

#include <algorithm>
#include <cmath>
#include <cstring>

using namespace DirectX;

namespace ga {

namespace {

constexpr double kPi = 3.14159265358979;

void CubeDirD(int face, double u, double v, double out[3]) {
    const double cx = u * 2.0 - 1.0, cy = v * 2.0 - 1.0;
    double p[3];
    switch (face) {
        case 0: p[0] = 1;   p[1] = cy;  p[2] = -cx; break;
        case 1: p[0] = -1;  p[1] = cy;  p[2] = cx;  break;
        case 2: p[0] = cx;  p[1] = 1;   p[2] = -cy; break;
        case 3: p[0] = cx;  p[1] = -1;  p[2] = cy;  break;
        case 4: p[0] = cx;  p[1] = cy;  p[2] = 1;   break;
        default: p[0] = -cx; p[1] = cy; p[2] = -1;  break;
    }
    const double len = std::sqrt(p[0] * p[0] + p[1] * p[1] + p[2] * p[2]);
    out[0] = p[0] / len;
    out[1] = p[1] / len;
    out[2] = p[2] / len;
}

}  // namespace

void GlobeLayer::Init(Gpu& gpu, ShaderCompiler& sc, FieldSet&, ID3D12RootSignature* rootSig) {
    m_rootSig = rootSig;
    if (!m_globe || !m_globe->Ready()) throw std::runtime_error("GlobeLayer needs globe data");
    if (!BuildPso(gpu, sc)) throw std::runtime_error("globe PSO failed");

    // M6j: the unified mesh-shader surface (orbit to helm, one pipeline). Falls back to the
    // classic VS path -- and the terrain layer -- if the device or compile says no.
    if (msSurface && BuildMeshPso(gpu, sc)) {
        for (uint32_t i = 0; i < Gpu::kFrameCount; ++i) {
            m_recBuf[i] = gpu.CreateUploadBuffer(
                static_cast<uint64_t>(kMaxMeshlets) * sizeof(MeshletRec),
                L"globe.meshlets (per-frame records)");
        }
        m_msPath = true;
        Log("[globe] mesh-shader surface ACTIVE (unified orbit-to-helm; terrain layer "
            "retires as a renderer)");
    } else if (msSurface) {
        Log("[globe] mesh-shader surface unavailable; classic VS path + terrain layer");
    }

    // M6i: no committed relief texture any more -- the composed height cube (ETOPO + NE 15s +
    // CUDEM, or MOLA) streams the same data through the residency manager, painted once and
    // cached. The CPU-side equirect grid stays in GlobeModel for picking and camera clamps.
    const int nx = m_globe->Nx(), ny = m_globe->Ny();
    if (m_globe->WavesNx() > 0 && !m_globe->Hs().empty()) {
        const int wn = m_globe->WavesNx(), wm = m_globe->WavesNy();
        m_hs = gpu.CreateTexture2D(wn, wm, DXGI_FORMAT_R32_FLOAT, D3D12_RESOURCE_FLAG_NONE,
                                   D3D12_RESOURCE_STATE_COPY_DEST, L"globe.hs (gfswave)");
        gpu.UploadTexture(m_hs, m_globe->Hs().data(), wn * 4);
        m_hs.srv = gpu.CreateSrv(m_hs.res.Get(), DXGI_FORMAT_R32_FLOAT);
        if (!m_globe->Wind().empty()) {
            m_wind = gpu.CreateTexture2D(wn, wm, DXGI_FORMAT_R32_FLOAT, D3D12_RESOURCE_FLAG_NONE,
                                         D3D12_RESOURCE_STATE_COPY_DEST, L"globe.wind (gfswave)");
            gpu.UploadTexture(m_wind, m_globe->Wind().data(), wn * 4);
            m_wind.srv = gpu.CreateSrv(m_wind.res.Get(), DXGI_FORMAT_R32_FLOAT);
        }
    }
    if (m_globe->CloudsNx() > 0) InitClouds(gpu, sc);
    InitNeAndWind(gpu, sc);

    Log("[globe] layer ready (relief %dx%d via composed height cube, waves %s, clouds %s)",
        nx, ny, m_hs.Valid() ? m_globe->WavesCycle().c_str() : "absent",
        m_cloudReady ? m_globe->CloudsCycle().c_str() : "absent");
}

// M6c: the sky volume. GFS isobaric cloud fraction rides up as a small dense 3D source
// texture; the CPU decides residency (a tile with no cloud in its footprint stays NULL = the
// hardware's clear air); one list-driven build pass writes density into the resident tiles.
void GlobeLayer::InitClouds(Gpu& gpu, ShaderCompiler& sc) {
    const int snx = m_globe->CloudsNx(), sny = m_globe->CloudsNy(), snz = m_globe->CloudsNz();
    const auto& src = m_globe->Clouds();
    const auto& alts = m_globe->CloudAltsM();
    if (alts.size() < static_cast<size_t>(snz)) return;

    // ---- dense source volume (720x361x10 R32F, ~10 MB)
    {
        D3D12_RESOURCE_DESC rd{};
        rd.Dimension = D3D12_RESOURCE_DIMENSION_TEXTURE3D;
        rd.Width = snx;
        rd.Height = sny;
        rd.DepthOrArraySize = static_cast<UINT16>(snz);
        rd.MipLevels = 1;
        rd.Format = DXGI_FORMAT_R32_FLOAT;
        rd.SampleDesc.Count = 1;
        D3D12_HEAP_PROPERTIES hp{};
        hp.Type = D3D12_HEAP_TYPE_DEFAULT;
        GA_CHECK(gpu.Device()->CreateCommittedResource(&hp, D3D12_HEAP_FLAG_NONE, &rd,
                                                      D3D12_RESOURCE_STATE_COPY_DEST, nullptr,
                                                      IID_PPV_ARGS(&m_cloudSrc.res)));
        m_cloudSrc.res->SetName(L"globe.cloudSrc (GFS isobaric TCDC)");
        m_cloudSrc.state = D3D12_RESOURCE_STATE_COPY_DEST;

        D3D12_PLACED_SUBRESOURCE_FOOTPRINT fp{};
        UINT numRows = 0;
        UINT64 rowBytes = 0, total = 0;
        gpu.Device()->GetCopyableFootprints(&rd, 0, 1, 0, &fp, &numRows, &rowBytes, &total);
        GpuBuffer staging = gpu.CreateUploadBuffer(total, L"cloudSrc staging");
        for (UINT r = 0; r < numRows; ++r) {
            memcpy(staging.cpu + fp.Offset + static_cast<uint64_t>(r) * fp.Footprint.RowPitch,
                   reinterpret_cast<const uint8_t*>(src.data()) +
                       static_cast<uint64_t>(r) * rowBytes,
                   static_cast<size_t>(rowBytes));
        }
        ID3D12GraphicsCommandList* cl = gpu.BeginUpload();
        D3D12_TEXTURE_COPY_LOCATION dst{}, sl{};
        dst.pResource = m_cloudSrc.res.Get();
        dst.Type = D3D12_TEXTURE_COPY_TYPE_SUBRESOURCE_INDEX;
        sl.pResource = staging.res.Get();
        sl.Type = D3D12_TEXTURE_COPY_TYPE_PLACED_FOOTPRINT;
        sl.PlacedFootprint = fp;
        cl->CopyTextureRegion(&dst, 0, 0, 0, &sl, nullptr);
        D3D12_RESOURCE_BARRIER br{};
        br.Type = D3D12_RESOURCE_BARRIER_TYPE_TRANSITION;
        br.Transition.pResource = m_cloudSrc.res.Get();
        br.Transition.Subresource = D3D12_RESOURCE_BARRIER_ALL_SUBRESOURCES;
        br.Transition.StateBefore = D3D12_RESOURCE_STATE_COPY_DEST;
        br.Transition.StateAfter = D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE;
        cl->ResourceBarrier(1, &br);
        gpu.EndUpload();
        m_cloudSrc.state = D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE;
    }

    // ---- the sparse bank + residency from the CPU field
    m_cloud.Init(gpu, kVolNx, kVolNy, kVolNz, DXGI_FORMAT_R16_FLOAT,
                 L"globe.cloud (sparse VOLUME bank: NULL tile = clear air)");
    const float dAlt = kShellTopM / kVolNz;
    for (uint32_t tz = 0; tz < m_cloud.TilesZ(); ++tz) {
        const float altLo = tz * m_cloud.TileD() * dAlt - 1800.0f;   // Gaussian slab reach
        const float altHi = (tz + 1) * m_cloud.TileD() * dAlt + 1800.0f;
        for (uint32_t ty = 0; ty < m_cloud.TilesY(); ++ty) {
            for (uint32_t tx = 0; tx < m_cloud.TilesX(); ++tx) {
                const int sx0 = tx * m_cloud.TileW() * snx / kVolNx;
                const int sx1 = ((tx + 1) * m_cloud.TileW() * snx + kVolNx - 1) / kVolNx;
                const int sy0 = ty * m_cloud.TileH() * sny / kVolNy;
                const int sy1 = ((ty + 1) * m_cloud.TileH() * sny + kVolNy - 1) / kVolNy;
                bool cloudy = false;
                for (int L = 0; L < snz && !cloudy; ++L) {
                    if (alts[L] < altLo || alts[L] > altHi) continue;
                    const size_t base = static_cast<size_t>(L) * snx * sny;
                    for (int y = sy0; y < sy1 && !cloudy; ++y) {
                        for (int x = sx0; x < sx1; ++x) {
                            if (src[base + static_cast<size_t>(y) * snx + x] > 12.0f) {
                                cloudy = true;
                                break;
                            }
                        }
                    }
                }
                if (cloudy) m_cloud.RequestMap(tx, ty, tz);
            }
        }
    }
    std::vector<uint32_t> fresh;
    m_cloud.CommitMappings(gpu, &fresh);
    Log("[globe] cloud residency: %u/%u tiles (%.0f/%.0f MB) -- the rest IS clear air",
        m_cloud.ResidentCount(), m_cloud.TilesX() * m_cloud.TilesY() * m_cloud.TilesZ(),
        m_cloud.ResidentBytes() / 1048576.0, m_cloud.VirtualBytes() / 1048576.0);

    // ---- build kernel (b0 CBV, t0 list root SRV, table [t1 src, u0 vol], s0 clamp)
    D3D12_DESCRIPTOR_RANGE1 ranges[2]{};
    ranges[0].RangeType = D3D12_DESCRIPTOR_RANGE_TYPE_SRV;
    ranges[0].NumDescriptors = 1;
    ranges[0].BaseShaderRegister = 1;
    ranges[0].Flags = D3D12_DESCRIPTOR_RANGE_FLAG_DESCRIPTORS_VOLATILE;
    ranges[0].OffsetInDescriptorsFromTableStart = 0;
    ranges[1].RangeType = D3D12_DESCRIPTOR_RANGE_TYPE_UAV;
    ranges[1].NumDescriptors = 1;
    ranges[1].BaseShaderRegister = 0;
    ranges[1].Flags = D3D12_DESCRIPTOR_RANGE_FLAG_DESCRIPTORS_VOLATILE;
    ranges[1].OffsetInDescriptorsFromTableStart = 1;
    D3D12_ROOT_PARAMETER1 params[3]{};
    params[0].ParameterType = D3D12_ROOT_PARAMETER_TYPE_CBV;
    params[0].Descriptor.ShaderRegister = 0;
    params[1].ParameterType = D3D12_ROOT_PARAMETER_TYPE_SRV;
    params[1].Descriptor.ShaderRegister = 0;
    params[2].ParameterType = D3D12_ROOT_PARAMETER_TYPE_DESCRIPTOR_TABLE;
    params[2].DescriptorTable.NumDescriptorRanges = 2;
    params[2].DescriptorTable.pDescriptorRanges = ranges;
    D3D12_STATIC_SAMPLER_DESC samp{};
    samp.Filter = D3D12_FILTER_MIN_MAG_MIP_LINEAR;
    samp.AddressU = samp.AddressV = samp.AddressW = D3D12_TEXTURE_ADDRESS_MODE_CLAMP;
    samp.MaxLOD = D3D12_FLOAT32_MAX;
    samp.ShaderRegister = 0;
    samp.ComparisonFunc = D3D12_COMPARISON_FUNC_NEVER;
    D3D12_VERSIONED_ROOT_SIGNATURE_DESC vd{};
    vd.Version = D3D_ROOT_SIGNATURE_VERSION_1_1;
    vd.Desc_1_1.NumParameters = _countof(params);
    vd.Desc_1_1.pParameters = params;
    vd.Desc_1_1.NumStaticSamplers = 1;
    vd.Desc_1_1.pStaticSamplers = &samp;
    Com<ID3DBlob> blob, err;
    GA_CHECK(D3D12SerializeVersionedRootSignature(&vd, &blob, &err));
    GA_CHECK(gpu.Device()->CreateRootSignature(0, blob->GetBufferPointer(), blob->GetBufferSize(),
                                              IID_PPV_ARGS(&m_cloudRs)));
    ShaderBlob cs = sc.Compile(m_shaderDir + L"/CloudVol.hlsl", L"CsCloudBuild", L"cs_6_0");
    if (!cs.Valid()) throw std::runtime_error("CloudVol kernel failed");
    D3D12_COMPUTE_PIPELINE_STATE_DESC cd{};
    cd.pRootSignature = m_cloudRs.Get();
    cd.CS = {cs.Data(), cs.Size()};
    GA_CHECK(gpu.Device()->CreateComputePipelineState(&cd, IID_PPV_ARGS(&m_cloudBuild)));

    m_cloudTable = gpu.SrvHeap().Alloc(2);
    D3D12_SHADER_RESOURCE_VIEW_DESC sv{};
    sv.Shader4ComponentMapping = D3D12_DEFAULT_SHADER_4_COMPONENT_MAPPING;
    sv.Format = DXGI_FORMAT_R32_FLOAT;
    sv.ViewDimension = D3D12_SRV_DIMENSION_TEXTURE3D;
    sv.Texture3D.MipLevels = 1;
    gpu.Device()->CreateShaderResourceView(m_cloudSrc.res.Get(), &sv,
                                           gpu.SrvHeap().Cpu(m_cloudTable + 0));
    D3D12_UNORDERED_ACCESS_VIEW_DESC uv{};
    uv.Format = DXGI_FORMAT_R16_FLOAT;
    uv.ViewDimension = D3D12_UAV_DIMENSION_TEXTURE3D;
    uv.Texture3D.WSize = UINT(-1);
    gpu.Device()->CreateUnorderedAccessView(m_cloud.Res(), nullptr, &uv,
                                            gpu.SrvHeap().Cpu(m_cloudTable + 1));

    // ---- one build pass (clouds are static per forecast cycle)
    {
        CloudCbData cb{};
        cb.volNx = kVolNx;
        cb.volNy = kVolNy;
        cb.volNz = kVolNz;
        cb.listCount = m_cloud.ResidentCount();
        cb.tilesX = m_cloud.TilesX();
        cb.tilesY = m_cloud.TilesY();
        cb.tileW = m_cloud.TileW();
        cb.tileH = m_cloud.TileH();
        cb.tileD = m_cloud.TileD();
        cb.srcNx = static_cast<uint32_t>(snx);
        cb.srcNy = static_cast<uint32_t>(sny);
        cb.srcNz = static_cast<uint32_t>(snz);
        for (int i = 0; i < 4; ++i) cb.altA[i] = alts[i];
        for (int i = 0; i < 4; ++i) cb.altB[i] = alts[4 + i];
        cb.altC[0] = alts[8];
        cb.altC[1] = alts[9];
        cb.altC[2] = kShellTopM;
        cb.altC[3] = 1.0f;    // density gamma (linear: GFS fraction is already conservative)

        ID3D12GraphicsCommandList* cl = gpu.BeginUpload();
        ID3D12DescriptorHeap* heaps[] = {gpu.SrvHeap().Heap()};   // raw upload list: bind it
        cl->SetDescriptorHeaps(1, heaps);
        cl->SetComputeRootSignature(m_cloudRs.Get());
        cl->SetComputeRootConstantBufferView(0, gpu.PushConstants(&cb, sizeof(cb)));
        const auto& list = m_cloud.ResidentList();
        cl->SetComputeRootShaderResourceView(1, gpu.PushConstants(list.data(), list.size() * 4));
        cl->SetComputeRootDescriptorTable(2, gpu.SrvHeap().Gpu(m_cloudTable));
        cl->SetPipelineState(m_cloudBuild.Get());
        cl->Dispatch(m_cloud.TileW() / 8, m_cloud.TileH() / 8, cb.listCount);
        D3D12_RESOURCE_BARRIER br{};
        br.Type = D3D12_RESOURCE_BARRIER_TYPE_TRANSITION;
        br.Transition.pResource = m_cloud.Res();
        br.Transition.Subresource = D3D12_RESOURCE_BARRIER_ALL_SUBRESOURCES;
        br.Transition.StateBefore = D3D12_RESOURCE_STATE_UNORDERED_ACCESS;
        br.Transition.StateAfter = D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE |
                                   D3D12_RESOURCE_STATE_PIXEL_SHADER_RESOURCE;
        cl->ResourceBarrier(1, &br);
        gpu.EndUpload();
        gpu.ResetConstantArenaAfterIdle();
    }
    m_cloudReady = true;
}

// M6d: the sparse Mv2 wind bank. (The NE 15s relief window that used to load here is a LAYER
// in the composed earth.height stack now -- painted at compose time, not blended per pixel.)
void GlobeLayer::InitNeAndWind(Gpu& gpu, ShaderCompiler& sc) {
    if (m_globe->WindNx() <= 0 || m_globe->WindU().empty()) return;

    const int wn = m_globe->WindNx(), wm = m_globe->WindNy();
    {
        std::vector<float> uv(static_cast<size_t>(wn) * wm * 2);
        for (size_t i = 0; i < static_cast<size_t>(wn) * wm; ++i) {
            uv[i * 2 + 0] = m_globe->WindU()[i];
            uv[i * 2 + 1] = m_globe->WindV()[i];
        }
        m_windSrc = gpu.CreateTexture2D(wn, wm, DXGI_FORMAT_R32G32_FLOAT,
                                        D3D12_RESOURCE_FLAG_NONE,
                                        D3D12_RESOURCE_STATE_COPY_DEST,
                                        L"globe.windSrc (GFS 10 m u,v)");
        gpu.UploadTexture(m_windSrc, uv.data(), wn * 8);
        // Compute reads it: PIXEL alone is not legal for that (the M5c lesson).
        ID3D12GraphicsCommandList* cl = gpu.BeginUpload();
        gpu.Transition(cl, m_windSrc, D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE |
                                          D3D12_RESOURCE_STATE_PIXEL_SHADER_RESOURCE);
        gpu.EndUpload();
    }

    m_windBank.Init(gpu, wn, wm, DXGI_FORMAT_R16G16B16A16_FLOAT,
                    L"globe.windMv2 (sparse: resident where storms live)");

    // Residency from a CPU curl estimate: the sparsity pattern IS the weather.
    const auto& U = m_globe->WindU();
    const auto& V = m_globe->WindV();
    const double dLat = std::abs(m_globe->WindDLat());
    const double dyM = GlobeModel::kR * dLat * 3.14159265358979 / 180.0;
    uint32_t resident = 0;
    for (uint32_t ty = 0; ty < m_windBank.TilesY(); ++ty) {
        for (uint32_t tx = 0; tx < m_windBank.TilesX(); ++tx) {
            bool active = false;
            const uint32_t y1 = (std::min)((ty + 1) * m_windBank.TileH(),
                                           static_cast<uint32_t>(wm));
            const uint32_t x1 = (std::min)((tx + 1) * m_windBank.TileW(),
                                           static_cast<uint32_t>(wn));
            for (uint32_t y = ty * m_windBank.TileH() + 1; y + 1 < y1 && !active; y += 2) {
                const double latDeg = m_globe->WindLat1() - (y + 0.5) * dLat;
                const double dxM =
                    dyM * (std::max)(std::cos(latDeg * 3.14159265358979 / 180.0), 0.05);
                for (uint32_t x = tx * m_windBank.TileW(); x < x1; x += 2) {
                    const size_t i = static_cast<size_t>(y) * wn + x;
                    const uint32_t xm = (x + wn - 1) % wn, xp = (x + 1) % wn;
                    const double curl =
                        (V[static_cast<size_t>(y) * wn + xp] -
                         V[static_cast<size_t>(y) * wn + xm]) / (2.0 * dxM) -
                        (U[i - wn] - U[i + wn]) / (2.0 * dyM);
                    const double spd = std::sqrt(U[i] * U[i] + V[i] * V[i]);
                    if (std::abs(curl) > 6.0e-5 || spd > 17.0) {
                        active = true;
                        break;
                    }
                }
            }
            if (active) {
                m_windBank.RequestMap(tx, ty);
                ++resident;
            }
        }
    }
    std::vector<uint32_t> fresh;
    m_windBank.CommitMappings(gpu, &fresh);
    Log("[globe] wind Mv2 residency: %u/%u tiles -- calm air stays NULL", resident,
        m_windBank.TilesX() * m_windBank.TilesY());

    // Build root signature + kernel (b0 CBV, t0 list, table [t1 src, u0 bank]).
    D3D12_DESCRIPTOR_RANGE1 ranges[2]{};
    ranges[0].RangeType = D3D12_DESCRIPTOR_RANGE_TYPE_SRV;
    ranges[0].NumDescriptors = 1;
    ranges[0].BaseShaderRegister = 1;
    ranges[0].Flags = D3D12_DESCRIPTOR_RANGE_FLAG_DESCRIPTORS_VOLATILE;
    ranges[0].OffsetInDescriptorsFromTableStart = 0;
    ranges[1].RangeType = D3D12_DESCRIPTOR_RANGE_TYPE_UAV;
    ranges[1].NumDescriptors = 1;
    ranges[1].BaseShaderRegister = 0;
    ranges[1].Flags = D3D12_DESCRIPTOR_RANGE_FLAG_DESCRIPTORS_VOLATILE;
    ranges[1].OffsetInDescriptorsFromTableStart = 1;
    D3D12_ROOT_PARAMETER1 params[3]{};
    params[0].ParameterType = D3D12_ROOT_PARAMETER_TYPE_CBV;
    params[0].Descriptor.ShaderRegister = 0;
    params[1].ParameterType = D3D12_ROOT_PARAMETER_TYPE_SRV;
    params[1].Descriptor.ShaderRegister = 0;
    params[2].ParameterType = D3D12_ROOT_PARAMETER_TYPE_DESCRIPTOR_TABLE;
    params[2].DescriptorTable.NumDescriptorRanges = 2;
    params[2].DescriptorTable.pDescriptorRanges = ranges;
    D3D12_VERSIONED_ROOT_SIGNATURE_DESC vd{};
    vd.Version = D3D_ROOT_SIGNATURE_VERSION_1_1;
    vd.Desc_1_1.NumParameters = _countof(params);
    vd.Desc_1_1.pParameters = params;
    Com<ID3DBlob> blob, err;
    GA_CHECK(D3D12SerializeVersionedRootSignature(&vd, &blob, &err));
    GA_CHECK(gpu.Device()->CreateRootSignature(0, blob->GetBufferPointer(), blob->GetBufferSize(),
                                              IID_PPV_ARGS(&m_windRs)));
    ShaderBlob cs = sc.Compile(m_shaderDir + L"/GlobeWind.hlsl", L"CsWindGrad", L"cs_6_0");
    if (!cs.Valid()) throw std::runtime_error("GlobeWind kernel failed");
    D3D12_COMPUTE_PIPELINE_STATE_DESC cd{};
    cd.pRootSignature = m_windRs.Get();
    cd.CS = {cs.Data(), cs.Size()};
    GA_CHECK(gpu.Device()->CreateComputePipelineState(&cd, IID_PPV_ARGS(&m_windBuild)));

    m_windTable = gpu.SrvHeap().Alloc(2);
    D3D12_SHADER_RESOURCE_VIEW_DESC sv{};
    sv.Shader4ComponentMapping = D3D12_DEFAULT_SHADER_4_COMPONENT_MAPPING;
    sv.Format = DXGI_FORMAT_R32G32_FLOAT;
    sv.ViewDimension = D3D12_SRV_DIMENSION_TEXTURE2D;
    sv.Texture2D.MipLevels = 1;
    gpu.Device()->CreateShaderResourceView(m_windSrc.res.Get(), &sv,
                                           gpu.SrvHeap().Cpu(m_windTable + 0));
    D3D12_UNORDERED_ACCESS_VIEW_DESC uav{};
    uav.Format = DXGI_FORMAT_R16G16B16A16_FLOAT;
    uav.ViewDimension = D3D12_UAV_DIMENSION_TEXTURE2D;
    gpu.Device()->CreateUnorderedAccessView(m_windBank.Res(), nullptr, &uav,
                                            gpu.SrvHeap().Cpu(m_windTable + 1));

    // One-shot build (static per forecast cycle).
    {
        WindCbData cb{};
        cb.nx = static_cast<uint32_t>(wn);
        cb.ny = static_cast<uint32_t>(wm);
        cb.listCount = m_windBank.ResidentCount();
        cb.tilesX = m_windBank.TilesX();
        cb.tileW = m_windBank.TileW();
        cb.tileH = m_windBank.TileH();
        cb.lat1 = static_cast<float>(m_globe->WindLat1());
        cb.dLatDeg = static_cast<float>(dLat);
        cb.radius = static_cast<float>(GlobeModel::kR);
        cb.scale = 1.0e4f;

        ID3D12GraphicsCommandList* cl = gpu.BeginUpload();
        ID3D12DescriptorHeap* heaps[] = {gpu.SrvHeap().Heap()};
        cl->SetDescriptorHeaps(1, heaps);
        cl->SetComputeRootSignature(m_windRs.Get());
        cl->SetComputeRootConstantBufferView(0, gpu.PushConstants(&cb, sizeof(cb)));
        const auto& list = m_windBank.ResidentList();
        cl->SetComputeRootShaderResourceView(1, gpu.PushConstants(list.data(), list.size() * 4));
        cl->SetComputeRootDescriptorTable(2, gpu.SrvHeap().Gpu(m_windTable));
        cl->SetPipelineState(m_windBuild.Get());
        cl->Dispatch(m_windBank.TileW() / 16, m_windBank.TileH() / 16, cb.listCount);
        D3D12_RESOURCE_BARRIER br{};
        br.Type = D3D12_RESOURCE_BARRIER_TYPE_TRANSITION;
        br.Transition.pResource = m_windBank.Res();
        br.Transition.Subresource = D3D12_RESOURCE_BARRIER_ALL_SUBRESOURCES;
        br.Transition.StateBefore = D3D12_RESOURCE_STATE_UNORDERED_ACCESS;
        br.Transition.StateAfter = D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE |
                                   D3D12_RESOURCE_STATE_PIXEL_SHADER_RESOURCE;
        cl->ResourceBarrier(1, &br);
        gpu.EndUpload();
        gpu.ResetConstantArenaAfterIdle();
    }
    m_windReady = true;
}

bool GlobeLayer::BuildPso(Gpu& gpu, ShaderCompiler& sc) {
    const std::wstring path = m_shaderDir + L"/Globe.hlsl";
    ShaderBlob vs = sc.Compile(path, L"VsMain", L"vs_6_0");
    ShaderBlob ps = sc.Compile(path, L"PsMain", L"ps_6_0");
    ShaderBlob vsk = sc.Compile(path, L"VsSky", L"vs_6_0");
    ShaderBlob psk = sc.Compile(path, L"PsSky", L"ps_6_0");
    if (!vs.Valid() || !ps.Valid() || !vsk.Valid() || !psk.Valid()) return false;

    // The atmosphere backdrop: fullscreen, no depth involvement; the surface overdraws it.
    {
        D3D12_GRAPHICS_PIPELINE_STATE_DESC d{};
        d.pRootSignature = m_rootSig;
        d.VS = {vsk.Data(), vsk.Size()};
        d.PS = {psk.Data(), psk.Size()};
        d.BlendState.RenderTarget[0].RenderTargetWriteMask = D3D12_COLOR_WRITE_ENABLE_ALL;
        d.SampleMask = UINT_MAX;
        d.RasterizerState.FillMode = D3D12_FILL_MODE_SOLID;
        d.RasterizerState.CullMode = D3D12_CULL_MODE_NONE;
        // M6g: the backdrop touches ONLY untouched pixels (VsSky emits z=0 = reversed-Z
        // infinity; cleared depth is 0, drawn geometry is > 0, so GREATER_EQUAL passes only
        // where the frame is still empty). No write: it stays a backdrop.
        d.DepthStencilState.DepthEnable = TRUE;
        d.DepthStencilState.DepthWriteMask = D3D12_DEPTH_WRITE_MASK_ZERO;
        d.DepthStencilState.DepthFunc = D3D12_COMPARISON_FUNC_GREATER_EQUAL;
        d.DSVFormat = DXGI_FORMAT_D32_FLOAT;
        d.PrimitiveTopologyType = D3D12_PRIMITIVE_TOPOLOGY_TYPE_TRIANGLE;
        d.NumRenderTargets = 1;
        d.RTVFormats[0] = DXGI_FORMAT_R16G16B16A16_FLOAT;
        d.SampleDesc.Count = 1;
        Com<ID3D12PipelineState> pso;
        if (FAILED(gpu.Device()->CreateGraphicsPipelineState(&d, IID_PPV_ARGS(&pso)))) {
            Log("[globe] sky PSO failed");
            return false;
        }
        m_skyPso = pso;
    }

    D3D12_GRAPHICS_PIPELINE_STATE_DESC d{};
    d.pRootSignature = m_rootSig;
    d.VS = {vs.Data(), vs.Size()};
    d.PS = {ps.Data(), ps.Size()};
    d.BlendState.RenderTarget[0].RenderTargetWriteMask = D3D12_COLOR_WRITE_ENABLE_ALL;
    d.SampleMask = UINT_MAX;
    d.RasterizerState.FillMode = D3D12_FILL_MODE_SOLID;
    d.RasterizerState.CullMode = D3D12_CULL_MODE_NONE;   // cube faces mix winding
    d.RasterizerState.DepthClipEnable = TRUE;
    d.DepthStencilState.DepthEnable = TRUE;
    d.DepthStencilState.DepthWriteMask = D3D12_DEPTH_WRITE_MASK_ALL;
    d.DepthStencilState.DepthFunc = D3D12_COMPARISON_FUNC_GREATER;   // reversed-Z
    d.DSVFormat = DXGI_FORMAT_D32_FLOAT;
    d.PrimitiveTopologyType = D3D12_PRIMITIVE_TOPOLOGY_TYPE_TRIANGLE;
    d.NumRenderTargets = 1;
    d.RTVFormats[0] = DXGI_FORMAT_R16G16B16A16_FLOAT;
    d.SampleDesc.Count = 1;

    Com<ID3D12PipelineState> pso;
    if (FAILED(gpu.Device()->CreateGraphicsPipelineState(&d, IID_PPV_ARGS(&pso)))) {
        Log("[globe] PSO failed");
        return false;
    }
    m_pso = pso;
    return true;
}

void GlobeLayer::ReloadShaders(Gpu& gpu, ShaderCompiler& sc) {
    BuildPso(gpu, sc);
    if (m_msPath) BuildMeshPso(gpu, sc);
}

// The mesh PSO rides the subobject STREAM (no d3dx12 in this repo): every subobject is
// { type, payload }, each aligned to a pointer boundary -- exactly what the runtime parses.
namespace {
template <typename T, D3D12_PIPELINE_STATE_SUBOBJECT_TYPE Tag>
struct alignas(void*) StreamSub {
    D3D12_PIPELINE_STATE_SUBOBJECT_TYPE type = Tag;
    T val{};
};
}  // namespace

bool GlobeLayer::BuildMeshPso(Gpu& gpu, ShaderCompiler& sc) {
    Com<ID3D12Device2> dev2;
    if (FAILED(gpu.Device()->QueryInterface(IID_PPV_ARGS(&dev2)))) return false;
    D3D12_FEATURE_DATA_D3D12_OPTIONS7 o7{};
    if (FAILED(gpu.Device()->CheckFeatureSupport(D3D12_FEATURE_D3D12_OPTIONS7, &o7,
                                                 sizeof(o7))) ||
        o7.MeshShaderTier == D3D12_MESH_SHADER_TIER_NOT_SUPPORTED) {
        return false;
    }
    const std::wstring path = m_shaderDir + L"/GlobeMesh.hlsl";
    ShaderBlob ms = sc.Compile(path, L"MsMain", L"ms_6_5");
    ShaderBlob ps = sc.Compile(m_shaderDir + L"/Globe.hlsl", L"PsMain", L"ps_6_5");
    if (!ms.Valid() || !ps.Valid()) return false;

    struct {
        StreamSub<ID3D12RootSignature*, D3D12_PIPELINE_STATE_SUBOBJECT_TYPE_ROOT_SIGNATURE> rs;
        StreamSub<D3D12_SHADER_BYTECODE, D3D12_PIPELINE_STATE_SUBOBJECT_TYPE_MS> ms;
        StreamSub<D3D12_SHADER_BYTECODE, D3D12_PIPELINE_STATE_SUBOBJECT_TYPE_PS> ps;
        StreamSub<D3D12_BLEND_DESC, D3D12_PIPELINE_STATE_SUBOBJECT_TYPE_BLEND> blend;
        StreamSub<UINT, D3D12_PIPELINE_STATE_SUBOBJECT_TYPE_SAMPLE_MASK> mask;
        StreamSub<D3D12_RASTERIZER_DESC, D3D12_PIPELINE_STATE_SUBOBJECT_TYPE_RASTERIZER> rast;
        StreamSub<D3D12_DEPTH_STENCIL_DESC, D3D12_PIPELINE_STATE_SUBOBJECT_TYPE_DEPTH_STENCIL>
            ds;
        StreamSub<DXGI_FORMAT, D3D12_PIPELINE_STATE_SUBOBJECT_TYPE_DEPTH_STENCIL_FORMAT> dsv;
        StreamSub<D3D12_RT_FORMAT_ARRAY, D3D12_PIPELINE_STATE_SUBOBJECT_TYPE_RENDER_TARGET_FORMATS>
            rtv;
        StreamSub<DXGI_SAMPLE_DESC, D3D12_PIPELINE_STATE_SUBOBJECT_TYPE_SAMPLE_DESC> sample;
    } stream;
    stream.rs.val = m_rootSig;
    stream.ms.val = {ms.Data(), ms.Size()};
    stream.ps.val = {ps.Data(), ps.Size()};
    stream.blend.val.RenderTarget[0].RenderTargetWriteMask = D3D12_COLOR_WRITE_ENABLE_ALL;
    stream.mask.val = UINT_MAX;
    stream.rast.val.FillMode = D3D12_FILL_MODE_SOLID;
    stream.rast.val.CullMode = D3D12_CULL_MODE_NONE;   // cube faces mix winding
    stream.rast.val.DepthClipEnable = TRUE;
    stream.ds.val.DepthEnable = TRUE;
    stream.ds.val.DepthWriteMask = D3D12_DEPTH_WRITE_MASK_ALL;
    stream.ds.val.DepthFunc = D3D12_COMPARISON_FUNC_GREATER;   // reversed-Z
    stream.dsv.val = DXGI_FORMAT_D32_FLOAT;
    stream.rtv.val.NumRenderTargets = 1;
    stream.rtv.val.RTFormats[0] = DXGI_FORMAT_R16G16B16A16_FLOAT;
    stream.sample.val = {1, 0};

    D3D12_PIPELINE_STATE_STREAM_DESC sd{sizeof(stream), &stream};
    Com<ID3D12PipelineState> pso;
    if (FAILED(dev2->CreatePipelineState(&sd, IID_PPV_ARGS(&pso)))) {
        Log("[globe] mesh PSO creation failed");
        return false;
    }
    m_msPso = pso;
    return true;
}

void GlobeLayer::SelectNode(int face, int level, double u0, double v0, double size) {
    const double R = m_radius;
    double dir[3];
    CubeDirD(face, u0 + size * 0.5, v0 + size * 0.5, dir);
    const double arc = (kPi / 2.0) * R / (1 << level);   // ground span of this node, m

    // M6g: node position in the TANGENT frame (doubles; the same frame the camera lives in).
    const double px = dir[0] * R, py = dir[1] * R, pz = dir[2] * R;
    const double tx = m_frameE[0] * px + m_frameE[1] * py + m_frameE[2] * pz;
    const double ty = m_frameU[0] * px + m_frameU[1] * py + m_frameU[2] * pz - R;
    const double tz = m_frameN[0] * px + m_frameN[1] * py + m_frameN[2] * pz;
    const double rel[3] = {tx - m_camPos[0], ty - m_camPos[1], tz - m_camPos[2]};
    const double dist =
        std::sqrt(rel[0] * rel[0] + rel[1] * rel[1] + rel[2] * rel[2]);

    // Horizon cull in the planet frame (angles, not dots: both can exceed 90 degrees).
    const double r =
        std::sqrt(m_camPlanet[0] * m_camPlanet[0] + m_camPlanet[1] * m_camPlanet[1] +
                  m_camPlanet[2] * m_camPlanet[2]);
    if (r > R + 10000.0) {
        const double cosA = (dir[0] * m_camPlanet[0] + dir[1] * m_camPlanet[1] +
                             dir[2] * m_camPlanet[2]) / r;
        const double ang = std::acos(std::clamp(cosA, -1.0, 1.0));
        const double horizon = std::acos(std::clamp(R / r, 0.0, 1.0));
        const double nodeAng = arc * 0.80 / R;   // generous half-diagonal
        if (ang > horizon + nodeAng + 0.02) return;
    }

    // Frustum cull: bounding sphere in camera-relative space. Radius covers the node's ground
    // extent, its relief, and the display exaggeration.
    const double radius = arc * 0.75 + 9000.0 * (std::max)(1.0f, reliefExagg);
    for (int p = 0; p < m_planeCount; ++p) {
        const double d = m_frustum[p][0] * rel[0] + m_frustum[p][1] * rel[1] +
                         m_frustum[p][2] * rel[2] + m_frustum[p][3];
        if (d < -radius) return;
    }

    // M6j: the mesh path walks all the way to CUDEM scale (level 16 = 4.75 m vertex spacing);
    // the fallback VS path keeps its classic depth (the terrain layer covers the near field).
    const int maxDepth = m_msPath ? 16 : kMaxDepth;
    if (level < maxDepth && dist < arc * kLodFactor) {
        const double h = size * 0.5;
        SelectNode(face, level + 1, u0, v0, h);
        SelectNode(face, level + 1, u0 + h, v0, h);
        SelectNode(face, level + 1, u0, v0 + h, h);
        SelectNode(face, level + 1, u0 + h, v0 + h, h);
        return;
    }

    // M6e: this leaf's on-screen span decides which streamed-texture mip it WANTS; the
    // residency manager turns wants into loads/mappings on its own budgets. The CDLOD walk IS
    // the sampling feedback -- deterministic, no readback pass (the classic had to render one).
    // M6i: the same rects feed every tenant riding this planet -- Mars's native pyramids and
    // the composed color/height cubes alike (Want clamps to each tenant's own mip count).
    if (m_res && (m_surfT >= 0 || m_colorT >= 0 || m_hgtT >= 0)) {
        const double px = arc / ((std::max)(dist, 1.0) * (std::max)(m_pixAng, 1e-6f));
        const double texAtMip0 = size * 16384.0;
        const int mip = (std::max)(
            0, static_cast<int>(std::ceil(std::log2((std::max)(texAtMip0 / (std::max)(px, 16.0),
                                                               1.0)))));
        // Node rects live in OUR CubeDir face-uv; texture tiles live in the HARDWARE cube
        // convention. Deriving the two per face collapses to one universal rule: v -> 1 - v.
        const float tu0 = static_cast<float>(u0), tu1 = static_cast<float>(u0 + size);
        const float tv0 = static_cast<float>(1.0 - (v0 + size));
        const float tv1 = static_cast<float>(1.0 - v0);
        if (m_surfT >= 0) m_res->Want(m_surfT, face, mip, tu0, tv0, tu1, tv1, m_predictPass);
        if (m_normT >= 0) m_res->Want(m_normT, face, mip, tu0, tv0, tu1, tv1, m_predictPass);
        if (m_colorT >= 0) m_res->Want(m_colorT, face, mip, tu0, tv0, tu1, tv1, m_predictPass);
        if (m_hgtT >= 0) m_res->Want(m_hgtT, face, mip, tu0, tv0, tu1, tv1, m_predictPass);
        // M6f: the window's demand -- the node's corners in Mercator z14-pixel space,
        // intersected with the window; its mip matches the same on-screen texel math against
        // the window's OWN pyramid (mip 0 = z14). Color and height windows share the frame,
        // so one rect feeds both.
        if (m_winT >= 0 || m_hgtWinT >= 0) {
            double mmin[2] = {1e18, 1e18}, mmax[2] = {-1e18, -1e18};
            for (int cy = 0; cy < 3; ++cy) {
                for (int cx = 0; cx < 3; ++cx) {
                    double d[3];
                    CubeDirD(face, u0 + size * cx * 0.5, v0 + size * cy * 0.5, d);
                    const double lat = std::asin(std::clamp(d[1], -1.0, 1.0));
                    const double lon = std::atan2(d[2], d[0]);
                    const double n14 = 16384.0 * 256.0;
                    const double mx = (lon / 3.14159265358979 * 0.5 + 0.5) * n14;
                    const double latC = std::clamp(lat, -1.4844, 1.4844);
                    const double my =
                        (0.5 - std::log(std::tan(0.7853981634 + latC * 0.5)) /
                                   (2.0 * 3.14159265358979)) *
                        n14;
                    mmin[0] = (std::min)(mmin[0], mx);
                    mmin[1] = (std::min)(mmin[1], my);
                    mmax[0] = (std::max)(mmax[0], mx);
                    mmax[1] = (std::max)(mmax[1], my);
                }
            }
            const double du0 = (mmin[0] - m_detOrg[0]) / m_detSize;
            const double dv0 = (mmin[1] - m_detOrg[1]) / m_detSize;
            const double du1 = (mmax[0] - m_detOrg[0]) / m_detSize;
            const double dv1 = (mmax[1] - m_detOrg[1]) / m_detSize;
            if (du1 > 0.0 && dv1 > 0.0 && du0 < 1.0 && dv0 < 1.0) {
                const double px = arc / ((std::max)(dist, 1.0) * (std::max)(m_pixAng, 1e-6f));
                const double span = (std::max)(du1 - du0, dv1 - dv0);
                const double texAtMip0 = span * 16384.0;
                const int dmip = (std::max)(
                    0, static_cast<int>(std::ceil(
                           std::log2((std::max)(texAtMip0 / (std::max)(px, 16.0), 1.0)))));
                const float wu0 = static_cast<float>((std::max)(du0, 0.0));
                const float wv0 = static_cast<float>((std::max)(dv0, 0.0));
                const float wu1 = static_cast<float>((std::min)(du1, 1.0));
                const float wv1 = static_cast<float>((std::min)(dv1, 1.0));
                if (m_winT >= 0) {
                    m_res->Want(m_winT, 0, dmip, wu0, wv0, wu1, wv1, m_predictPass);
                }
                if (m_hgtWinT >= 0) {
                    m_res->Want(m_hgtWinT, 0, dmip, wu0, wv0, wu1, wv1, m_predictPass);
                }
            }
            // M7f: the z17 DETAIL window rides the same node box, 8x finer frame.
            if (m_detWinT >= 0) {
                const double eu0 = (mmin[0] * 8.0 - m_det17Org[0]) / 16384.0;
                const double ev0 = (mmin[1] * 8.0 - m_det17Org[1]) / 16384.0;
                const double eu1 = (mmax[0] * 8.0 - m_det17Org[0]) / 16384.0;
                const double ev1 = (mmax[1] * 8.0 - m_det17Org[1]) / 16384.0;
                if (eu1 > 0.0 && ev1 > 0.0 && eu0 < 1.0 && ev0 < 1.0) {
                    const double px =
                        arc / ((std::max)(dist, 1.0) * (std::max)(m_pixAng, 1e-6f));
                    const double spanD = (std::max)(eu1 - eu0, ev1 - ev0);
                    const int emip = (std::max)(
                        0, static_cast<int>(std::ceil(std::log2(
                               (std::max)(spanD * 16384.0 / (std::max)(px, 16.0), 1.0)))));
                    m_res->Want(m_detWinT, 0, emip,
                                static_cast<float>((std::max)(eu0, 0.0)),
                                static_cast<float>((std::max)(ev0, 0.0)),
                                static_cast<float>((std::min)(eu1, 1.0)),
                                static_cast<float>((std::min)(ev1, 1.0)), m_predictPass);
                }
            }
        }
    }
    if (m_predictPass) return;   // prefetch walk: wants only, no draw nodes

    // Per-LEVEL morph ramp (identical on both sides of every seam = crack-free): fade this LOD
    // out across the band where its parent would still be split.
    const float morphStart = static_cast<float>(arc * kLodFactor * 1.35);
    const float morphEnd = static_cast<float>(arc * kLodFactor * 1.95);
    if (m_msPath) {
        EmitMeshlets(face, u0, v0, size, arc, morphStart, morphEnd);
        return;
    }
    NodeData nd{};
    nd.uv0[0] = static_cast<float>(u0);
    nd.uv0[1] = static_cast<float>(v0);
    nd.uvStep[0] = static_cast<float>(size / 32.0);
    nd.uvStep[1] = static_cast<float>(size / 32.0);
    nd.face = static_cast<uint32_t>(face);
    nd.morphStart = morphStart;
    nd.morphEnd = morphEnd;
    m_nodes.push_back(nd);
}

// M6j: one leaf -> 16 mesh-shader records (4x4 sub-meshlets of 8x8 cells). Fine meshlets
// (arc <= 650 m) get a DOUBLE-precision camera-relative anchor + the position Jacobian, so
// vertices reconstruct from small numbers only -- the float wall the estuary layers dodged by
// staying flat falls here, and the one surface reaches walking height.
void GlobeLayer::EmitMeshlets(int face, double u0, double v0, double size, double arc,
                              float morphStart, float morphEnd) {
    if (m_meshlets.size() + 16 > kMaxMeshlets) return;
    const double R = m_radius;
    const double step = size / 32.0;
    auto tangent = [&](const double d[3], double out[3]) {
        const double px = d[0] * R, py = d[1] * R, pz = d[2] * R;
        out[0] = m_frameE[0] * px + m_frameE[1] * py + m_frameE[2] * pz;
        out[1] = m_frameU[0] * px + m_frameU[1] * py + m_frameU[2] * pz - R;
        out[2] = m_frameN[0] * px + m_frameN[1] * py + m_frameN[2] * pz;
    };
    for (int my = 0; my < 4; ++my) {
        for (int mx = 0; mx < 4; ++mx) {
            MeshletRec rec{};
            rec.uv0[0] = static_cast<float>(u0);
            rec.uv0[1] = static_cast<float>(v0);
            rec.uvStepCell[0] = static_cast<float>(step);
            rec.uvStepCell[1] = static_cast<float>(step);
            rec.face = static_cast<uint32_t>(face);
            rec.cell0 = static_cast<uint32_t>(mx * 8) | (static_cast<uint32_t>(my * 8) << 8);
            rec.morphStart = morphStart;
            rec.morphEnd = morphEnd;
            rec.arc = static_cast<float>(arc);
            const double cu = u0 + (mx * 8 + 4) * step;
            const double cv = v0 + (my * 8 + 4) * step;
            double dc[3], tc[3];
            CubeDirD(face, cu, cv, dc);
            tangent(dc, tc);
            rec.anchorRel[0] = static_cast<float>(tc[0] - m_camPos[0]);
            rec.anchorRel[1] = static_cast<float>(tc[1] - m_camPos[1]);
            rec.anchorRel[2] = static_cast<float>(tc[2] - m_camPos[2]);
            // Jacobian by central difference in doubles; per face-uv UNIT.
            const double e = (std::max)(step * 0.25, 1e-7);
            double dA[3], dB[3], tA[3], tB[3];
            CubeDirD(face, cu + e, cv, dA);
            CubeDirD(face, cu - e, cv, dB);
            tangent(dA, tA);
            tangent(dB, tB);
            for (int i = 0; i < 3; ++i) {
                rec.dPdu[i] = static_cast<float>((tA[i] - tB[i]) / (2.0 * e));
            }
            CubeDirD(face, cu, cv + e, dA);
            CubeDirD(face, cu, cv - e, dB);
            tangent(dA, tA);
            tangent(dB, tB);
            for (int i = 0; i < 3; ++i) {
                rec.dPdv[i] = static_cast<float>((tA[i] - tB[i]) / (2.0 * e));
            }
            rec.upT[0] = static_cast<float>(m_frameE[0] * dc[0] + m_frameE[1] * dc[1] +
                                            m_frameE[2] * dc[2]);
            rec.upT[1] = static_cast<float>(m_frameU[0] * dc[0] + m_frameU[1] * dc[1] +
                                            m_frameU[2] * dc[2]);
            rec.upT[2] = static_cast<float>(m_frameN[0] * dc[0] + m_frameN[1] * dc[1] +
                                            m_frameN[2] * dc[2]);
            m_meshlets.push_back(rec);
        }
    }
}

// M6e screw-prefetch: the same walk under the PREDICTED pose, emitting predicted wants only.
void GlobeLayer::PredictWants(const Camera& cam, float aspect) {
    if (!m_res || (m_surfT < 0 && m_colorT < 0 && m_hgtT < 0)) return;
    double savedPos[3] = {m_camPos[0], m_camPos[1], m_camPos[2]};
    double savedFrustum[6][4];
    memcpy(savedFrustum, m_frustum, sizeof(m_frustum));
    const int savedPlanes = m_planeCount;

    m_camPos[0] = cam.px;
    m_camPos[1] = cam.py;
    m_camPos[2] = cam.pz;
    m_planeCount = 0;   // no frustum cull: the predicted view is approximate by nature
    m_predictPass = true;
    for (int f = 0; f < 6; ++f) SelectNode(f, 0, 0.0, 0.0, 1.0);
    m_predictPass = false;

    memcpy(m_frustum, savedFrustum, sizeof(m_frustum));
    m_planeCount = savedPlanes;
    m_camPos[0] = savedPos[0];
    m_camPos[1] = savedPos[1];
    m_camPos[2] = savedPos[2];
}

void GlobeLayer::SetView(const Camera& cam, float aspect, float viewportH, double simTime) {
    if (!m_globe || !m_globe->Ready()) return;
    m_viewportH = viewportH;
    m_pixAng = cam.fovY / (std::max)(viewportH, 1.0f);   // the walk needs it BEFORE wavesB
    // M6g: cam is the FLAT (tangent-frame) camera. Keep it for node culling, and derive the
    // planet-frame position (doubles) for the horizon test.
    m_camPos[0] = cam.px;
    m_camPos[1] = cam.py;
    m_camPos[2] = cam.pz;
    const double ry = m_radius + cam.py;
    for (int i = 0; i < 3; ++i) {
        m_camPlanet[i] = m_frameU[i] * ry + m_frameE[i] * cam.px + m_frameN[i] * cam.pz;
    }

    // Frustum planes from the camera-relative view-projection (reversed-Z: use the 4 side
    // planes + near; there is no far plane).
    const XMMATRIX vp = cam.ViewRelative() * cam.Projection(aspect);
    XMFLOAT4X4 m;
    XMStoreFloat4x4(&m, vp);
    auto plane = [&](int idx, double a, double b, double c, double d) {
        const double len = std::sqrt(a * a + b * b + c * c);
        m_frustum[idx][0] = a / len;
        m_frustum[idx][1] = b / len;
        m_frustum[idx][2] = c / len;
        m_frustum[idx][3] = d / len;
    };
    plane(0, m._14 + m._11, m._24 + m._21, m._34 + m._31, m._44 + m._41);   // left
    plane(1, m._14 - m._11, m._24 - m._21, m._34 - m._31, m._44 - m._41);   // right
    plane(2, m._14 + m._12, m._24 + m._22, m._34 + m._32, m._44 + m._42);   // bottom
    plane(3, m._14 - m._12, m._24 - m._22, m._34 - m._32, m._44 - m._42);   // top
    plane(4, m._13, m._23, m._33, m._43);                                    // near (z >= 0)
    m_planeCount = 5;

    m_nodes.clear();
    m_meshlets.clear();
    for (int f = 0; f < 6; ++f) SelectNode(f, 0, 0.0, 0.0, 1.0);

    const double r = std::sqrt(m_camPos[0] * m_camPos[0] + m_camPos[1] * m_camPos[1] +
                               m_camPos[2] * m_camPos[2]);
    m_cb.glo[0] = static_cast<float>(m_radius);
    m_cb.glo[1] = reliefExagg;
    m_cb.glo[2] = static_cast<float>(m_nodes.size());
    m_cb.glo[3] = static_cast<float>(std::fmod(simTime, 3600.0));
    // M6g: gCamAbs = sphere-CENTRED tangent camera (flat + (0,R,0); the sum in doubles).
    m_cb.camAbs[0] = static_cast<float>(m_camPos[0]);
    m_cb.camAbs[1] = static_cast<float>(m_camPos[1] + m_radius);
    m_cb.camAbs[2] = static_cast<float>(m_camPos[2]);
    m_cb.estGeo[0] = static_cast<float>(m_estGeo[0]);
    m_cb.estGeo[1] = static_cast<float>(m_estGeo[1]);
    m_cb.estGeo[2] =
        m_estGeo[2] > 0.0 ? static_cast<float>(1.0 / m_estGeo[2]) : 0.0f;
    m_cb.estGeo[3] =
        (m_estGeo[3] > 0.0 && !m_streamMars) ? static_cast<float>(1.0 / m_estGeo[3]) : 0.0f;
    // M7: the wave vertex bank rows (one-water mode).
    m_cb.bankU[0] = m_bankSrv[0];
    m_cb.bankU[1] = m_bankSrv[1];
    m_cb.bankU[2] = (m_oneWater && m_bankSrv[0] != 0xFFFFFFFFu) ? 1u : 0u;
    m_cb.bankU[3] = 512u;
    m_cb.bankA[0] = m_bankBase;
    m_cb.bankA[1] = 6.0f;
    memcpy(m_cb.bankOrg01, &m_bankOrg[0], 16);
    memcpy(m_cb.bankOrg23, &m_bankOrg[4], 16);
    memcpy(m_cb.bankOrg45, &m_bankOrg[8], 16);
    m_cb.bankU2[0] = m_bankSrv[2];
    for (int i = 0; i < 3; ++i) {
        m_cb.bankU2[i + 1] = m_bankDeriv[i];
        m_cb.bankB[i] = m_bankPatch[i];
        m_cb.bankC[i] = m_bankK[i];
    }
    m_cb.bankB[3] = m_bankExag;
    m_cb.texIdx[0] = UINT32_MAX;   // was the equirect relief; the composed height cube owns it
    m_cb.texIdx[1] = m_hs.Valid() ? m_hs.srv : UINT32_MAX;
    m_cb.texIdx[2] = m_wind.Valid() ? m_wind.srv : UINT32_MAX;
    m_cb.wavesA[0] = static_cast<float>(m_globe->WavesLat1());
    m_cb.wavesA[1] = static_cast<float>(m_globe->WavesLon1());
    m_cb.wavesA[2] = static_cast<float>(1.0 / m_globe->WavesDLat());
    m_cb.wavesA[3] = static_cast<float>(1.0 / m_globe->WavesDLon());
    m_cb.wavesB[0] = static_cast<float>(m_globe->WavesNx());
    m_cb.wavesB[1] = static_cast<float>(m_globe->WavesNy());
    m_cb.wavesB[2] = cam.fovY / (std::max)(m_viewportH, 1.0f);   // pixel angular size
    m_cb.wavesB[3] = waterNavd;   // M6j: the live waterline for the close-up material model
    m_cb.texIdx[3] = m_cloudReady ? m_cloud.Srv() : UINT32_MAX;
    m_cb.cloudA[0] = 2.4e-3f;      // extinction /m at density 1 (stratiform-effective)
    m_cb.cloudA[1] = kShellTopM;
    m_cb.cloudA[2] = 1.0f;         // sun boost
    m_cb.cloudA[3] = 0.75f;        // ground-shadow strength
    m_cb.texIdx2[0] = UINT32_MAX;  // was the NE 15s window; a composed height LAYER now
    m_cb.texIdx2[1] = m_windReady ? m_windBank.Srv() : UINT32_MAX;
    m_cb.texIdx2[2] = windOverlay ? 1u : 0u;
    m_cb.texIdx2[3] = albedoLens ? 1u : 0u;
    m_cb.windGeo[0] = static_cast<float>(m_globe->WindLat1());
    m_cb.windGeo[1] = static_cast<float>(m_globe->WindLon1());
    m_cb.windGeo[2] = static_cast<float>(1.0 / std::abs(m_globe->WindDLat()));
    m_cb.windGeo[3] = static_cast<float>(1.0 / m_globe->WindDLon());
    m_cb.windB[0] = static_cast<float>(m_globe->WindNx());
    m_cb.windB[1] = static_cast<float>(m_globe->WindNy());

    // ---- M6e: Mars's native streams. On Mars, the live-Earth fields all stand down.
    const bool surfOn = m_res && m_surfT >= 0;
    const bool normOn = m_res && m_normT >= 0;
    m_cb.streamU[0] = surfOn ? m_res->TextureSrv(m_surfT) : UINT32_MAX;
    m_cb.streamU[1] = normOn ? m_res->TextureSrv(m_normT) : UINT32_MAX;
    m_cb.streamU[2] = surfOn ? m_res->ResidencySrv(m_surfT) : UINT32_MAX;
    m_cb.streamU[3] = normOn ? m_res->ResidencySrv(m_normT) : UINT32_MAX;
    m_cb.streamF[0] = surfOn ? 1.0f : 0.0f;
    m_cb.streamF[1] = normOn ? 1.0f : 0.0f;
    m_cb.streamF[2] = m_streamMars ? 1.0f : 0.0f;
    m_cb.streamF[3] = 0.0f;
    // ---- M6i: the composed channels + the one-world frame, through the ONE fill function
    // the terrain also uses -- the two layers cannot disagree about this math.
    FillComposedCb(m_cb.cs, m_res, m_colorT, m_winT, m_hgtT, m_hgtWinT, m_detOrg[0],
                   m_detOrg[1], m_detSize, 14, m_radius, m_frameE, m_frameU, m_frameN,
                   stencilOverlay, m_gisWinSrv, m_gisGlobSrv, m_detWinT, m_det17Org, 17,
                   m_gisEditSrv, m_gisEditOn ? m_gisEditBox : nullptr);
    if (m_streamMars) {
        m_cb.texIdx[1] = m_cb.texIdx[2] = m_cb.texIdx[3] = UINT32_MAX;   // waves/wind/clouds
        m_cb.texIdx2[1] = UINT32_MAX;                                    // wind bank
        m_cb.texIdx2[2] = 0;
    }

    // The sky pass rebuilds pixel rays from this basis (b2). M6j: the ONE render basis --
    // the shell can no longer roll apart from the surface it wraps.
    XMFLOAT3 cf, cr, cu;
    cam.ViewBasis(cf, cr, cu);
    m_skyCb.fwd[0] = cf.x; m_skyCb.fwd[1] = cf.y; m_skyCb.fwd[2] = cf.z;
    m_skyCb.fwd[3] = std::tan(cam.fovY * 0.5f);
    m_skyCb.right[0] = cr.x; m_skyCb.right[1] = cr.y; m_skyCb.right[2] = cr.z;
    m_skyCb.right[3] = aspect;
    m_skyCb.up[0] = cu.x; m_skyCb.up[1] = cu.y; m_skyCb.up[2] = cu.z;
    m_skyCb.up[3] = 0.0f;
    // (M6h: the M6b destination beacon is fully retired -- row and shader block deleted.)

    char s[96];
    if (m_msPath) {
        snprintf(s, sizeof(s), "globe %zu meshlets (MS)  alt %.0f km", m_meshlets.size(),
                 (r - m_radius) / 1000.0);
    } else {
        snprintf(s, sizeof(s), "globe %zu nodes  alt %.0f km", m_nodes.size(),
                 (r - m_radius) / 1000.0);
    }
    stats = s;
    if (m_res) stats += "  " + m_res->stats;
}

void GlobeLayer::Render(const FrameContext& ctx) {
    if (!m_pso || (m_nodes.empty() && m_meshlets.empty())) return;
    PixScope scope(ctx.cl, "globe (atmosphere shell + quad-sphere CDLOD + sparse cloud volume)");

    // M6e: the residency manager's per-frame turn -- loads started, budgeted tiles mapped and
    // filled, residency maps refreshed -- BEFORE the surface samples any of it.
    if (m_res) m_res->ProcessQueues(*ctx.gpu, ctx.cl);

    const D3D12_GPU_VIRTUAL_ADDRESS cbVa = ctx.gpu->PushConstants(&m_cb, sizeof(m_cb));

    // 1) The atmosphere backdrop: limb scatter + sun for every ray that misses the planet.
    if (m_skyPso && skyPassEnabled) {
        PixMarker(ctx.cl, "globe.sky (single-scatter shell: the limb past the disc)");
        ctx.cl->SetPipelineState(m_skyPso.Get());
        ctx.cl->IASetPrimitiveTopology(D3D_PRIMITIVE_TOPOLOGY_TRIANGLELIST);
        ctx.cl->SetGraphicsRootConstantBufferView(1, cbVa);
        ctx.cl->SetGraphicsRootConstantBufferView(
            4, ctx.gpu->PushConstants(&m_skyCb, sizeof(m_skyCb)));
        ctx.cl->DrawInstanced(3, 1, 0, 0);
    }

    // 2) The surface (which marches the sparse cloud bank on its way down).
    if (m_msPath && m_msPso && !m_meshlets.empty()) {
        // M6j: the unified surface -- meshlet records ride a frame-indexed upload buffer,
        // one DispatchMesh amplifies them from the composed channels.
        if (!m_cl6 && FAILED(ctx.cl->QueryInterface(IID_PPV_ARGS(&m_cl6)))) {
            m_msPath = false;
            return;
        }
        GpuBuffer& rec = m_recBuf[ctx.gpu->FrameIndex()];
        const size_t bytes = m_meshlets.size() * sizeof(MeshletRec);
        memcpy(rec.cpu, m_meshlets.data(), bytes);
        ctx.cl->SetPipelineState(m_msPso.Get());
        ctx.cl->SetGraphicsRootConstantBufferView(1, cbVa);
        ctx.cl->SetGraphicsRootShaderResourceView(2, rec.res->GetGPUVirtualAddress());
        m_cl6->DispatchMesh(static_cast<UINT>(m_meshlets.size()), 1, 1);
        return;
    }
    ctx.cl->SetPipelineState(m_pso.Get());
    ctx.cl->IASetPrimitiveTopology(D3D_PRIMITIVE_TOPOLOGY_TRIANGLELIST);
    ctx.cl->SetGraphicsRootConstantBufferView(1, cbVa);
    // Root param 2 normally carries the FieldSet; the renderer re-binds it every frame and the
    // globe draws last, so the CDLOD node list borrows the slot for this draw.
    ctx.cl->SetGraphicsRootShaderResourceView(
        2, ctx.gpu->PushConstants(m_nodes.data(), m_nodes.size() * sizeof(NodeData)));
    ctx.cl->DrawInstanced(32 * 32 * 6, static_cast<UINT>(m_nodes.size()), 0, 0);
}

}  // namespace ga
