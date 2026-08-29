#include "sim/SweSolver.h"

#include "core/PixEvents.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>

namespace ga {

void SweSolver::Init(Gpu& gpu, ShaderCompiler& sc, const std::wstring& shaderDir,
                     const BathyModel& bathy, ID3D12Resource* bathyRes) {
    m_bathy = &bathy;
    m_bathyRes = bathyRes;
    const uint32_t nx = bathy.Nx(), ny = bathy.Ny();
    const float dx = bathy.WorldSizeX() / nx;

    // Two grade banks over the bathy grid, padded up to tile multiples internally by the atlas.
    m_eta.Init(gpu, nx, ny, DXGI_FORMAT_R32_FLOAT, L"swe.eta (dEta from the tide plane)");
    m_flux.Init(gpu, nx, ny, DXGI_FORMAT_R32G32_FLOAT, L"swe.flux (signed face fluxes)");

    // Static residency: everything that can ever be wet -- bed below max tide + surge + wave
    // margin. Land and dune tiles stay NULL forever; their reads are the hardware zero.
    const auto& elev = bathy.Elev();
    auto mapWet = [&](TileAtlas2D& bank) {
        for (uint32_t ty = 0; ty < bank.TilesY(); ++ty) {
            for (uint32_t tx = 0; tx < bank.TilesX(); ++tx) {
                bool wet = false;
                const uint32_t x1 = (std::min)((tx + 1) * bank.TileW(), nx);
                const uint32_t y1 = (std::min)((ty + 1) * bank.TileH(), ny);
                for (uint32_t y = ty * bank.TileH(); y < y1 && !wet; ++y) {
                    for (uint32_t x = tx * bank.TileW(); x < x1; ++x) {
                        const float e = elev[y * nx + x];
                        if (e > -9000.0f && e < 4.0f) {
                            wet = true;
                            break;
                        }
                    }
                }
                if (wet) bank.RequestMap(tx, ty);
            }
        }
        std::vector<uint32_t> fresh;
        bank.CommitMappings(gpu, &fresh);
    };
    mapWet(m_eta);
    mapWet(m_flux);
    for (uint32_t ty = 0; ty < m_eta.TilesY(); ++ty) {
        for (uint32_t tx = 0; tx < m_eta.TilesX(); ++tx) {
            if (!m_eta.IsResident(tx, ty)) Log("[swe] eta NULL tile (%u,%u)", tx, ty);
        }
    }

    // Timestep from the deepest resident water (pipe scheme + damping tolerate ~0.6 dx / c).
    float deepest = 0.0f;
    for (float e : elev) {
        if (e > -9000.0f) deepest = (std::min)(deepest, e);
    }
    const float hmax = -deepest + 3.0f;
    m_dt = 0.45f * dx / std::sqrt(9.81f * (std::max)(hmax, 5.0f));

    m_cb.nx = nx;
    m_cb.ny = ny;
    m_cb.etaTilesX = m_eta.TilesX();
    m_cb.fluxTilesX = m_flux.TilesX();
    m_cb.etaTileW = m_eta.TileW();
    m_cb.etaTileH = m_eta.TileH();
    m_cb.fluxTileW = m_flux.TileW();
    m_cb.fluxTileH = m_flux.TileH();
    m_cb.worldX0 = bathy.WorldX0();
    m_cb.dx = dx;
    m_cb.dt = m_dt;
    m_cb.damp = 0.99995f;   // background linear part only; the real friction is quadratic drag
    m_cb.spongeX0 = 1400.0f;              // ramp start: past the bar, before the open sea
    m_cb.spongeRate = m_dt / 10.0f;       // full-strength deviations die in ~10 s
    m_cb.gravity = 9.81f;
    m_cb.riverBox[0] = 24.0f;             // west Dirichlet strip width, texels (~120 m)
    m_cb.riverBox[1] = m_dt / 2.0f;       // hard clamp: the strip must SUPPLY the upriver
                                          // prism's demand (hundreds of m^3/s), not trickle
    // M6d: the south strip exists only when the window actually reaches Plum Island Sound
    // (the sound's real entrance is south of the window; its tide must enter as data).
    const bool hasSound = bathy.WorldZ0() < -9000.0f;
    m_cb.riverBox[2] = hasSound ? static_cast<float>(ny - 8) : 1.0e9f;
    if (hasSound) {
        Log("[swe] south boundary strip active (Plum Island Sound rows %u..%u)", ny - 8, ny - 1);
    }

    // Dense derived-current texture (exact bathy dims; no padding).
    m_uv = gpu.CreateTexture2D(nx, ny, DXGI_FORMAT_R16G16B16A16_FLOAT,
                               D3D12_RESOURCE_FLAG_ALLOW_UNORDERED_ACCESS,
                               D3D12_RESOURCE_STATE_UNORDERED_ACCESS, L"swe.uv (derived currents)");
    m_uv.srv = gpu.CreateSrv(m_uv.res.Get(), DXGI_FORMAT_R16G16B16A16_FLOAT);
    m_uvUav = gpu.CreateTextureUav(m_uv.res.Get(), DXGI_FORMAT_R16G16B16A16_FLOAT,
                                   D3D12_UAV_DIMENSION_TEXTURE2D);

    // Root signature: b0 CBV, t0 root SRV (tile list), table [t1 bathy, u0 eta, u1 flux, u2 uv].
    D3D12_DESCRIPTOR_RANGE1 ranges[2]{};
    ranges[0].RangeType = D3D12_DESCRIPTOR_RANGE_TYPE_SRV;
    ranges[0].NumDescriptors = 1;
    ranges[0].BaseShaderRegister = 1;
    ranges[0].Flags = D3D12_DESCRIPTOR_RANGE_FLAG_DESCRIPTORS_VOLATILE;
    ranges[0].OffsetInDescriptorsFromTableStart = 0;
    ranges[1].RangeType = D3D12_DESCRIPTOR_RANGE_TYPE_UAV;
    ranges[1].NumDescriptors = 3;
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
                                              IID_PPV_ARGS(&m_rs)));
    m_rs->SetName(L"swe root signature");

    auto makePso = [&](const wchar_t* entry, Com<ID3D12PipelineState>& out) {
        ShaderBlob cs = sc.Compile(shaderDir + L"/Swe.hlsl", entry, L"cs_6_0");
        if (!cs.Valid()) throw std::runtime_error("Swe kernel failed");
        D3D12_COMPUTE_PIPELINE_STATE_DESC d{};
        d.pRootSignature = m_rs.Get();
        d.CS = {cs.Data(), cs.Size()};
        GA_CHECK(gpu.Device()->CreateComputePipelineState(&d, IID_PPV_ARGS(&out)));
        out->SetName(entry);
    };
    makePso(L"CsSweClearEta", m_clearEta);
    makePso(L"CsSweClearFlux", m_clearFlux);
    makePso(L"CsSweUvClear", m_uvClear);
    makePso(L"CsSweFlux", m_fluxK);
    makePso(L"CsSweHeight", m_heightK);
    makePso(L"CsSweDerive", m_deriveK);

    m_table = gpu.SrvHeap().Alloc(4);
    D3D12_SHADER_RESOURCE_VIEW_DESC sv{};
    sv.Shader4ComponentMapping = D3D12_DEFAULT_SHADER_4_COMPONENT_MAPPING;
    sv.Format = DXGI_FORMAT_R32_FLOAT;
    sv.ViewDimension = D3D12_SRV_DIMENSION_TEXTURE2D;
    sv.Texture2D.MipLevels = 1;
    gpu.Device()->CreateShaderResourceView(bathyRes, &sv, gpu.SrvHeap().Cpu(m_table + 0));
    D3D12_UNORDERED_ACCESS_VIEW_DESC uv{};
    uv.ViewDimension = D3D12_UAV_DIMENSION_TEXTURE2D;
    uv.Format = DXGI_FORMAT_R32_FLOAT;
    gpu.Device()->CreateUnorderedAccessView(m_eta.Res(), nullptr, &uv,
                                            gpu.SrvHeap().Cpu(m_table + 1));
    uv.Format = DXGI_FORMAT_R32G32_FLOAT;
    gpu.Device()->CreateUnorderedAccessView(m_flux.Res(), nullptr, &uv,
                                            gpu.SrvHeap().Cpu(m_table + 2));
    uv.Format = DXGI_FORMAT_R16G16B16A16_FLOAT;
    gpu.Device()->CreateUnorderedAccessView(m_uv.res.Get(), nullptr, &uv,
                                            gpu.SrvHeap().Cpu(m_table + 3));

    Log("[swe] grid %ux%u dx %.2f m  dt %.3f s  eta %u/%u t  flux %u/%u t  resident %.1f MB",
        nx, ny, dx, m_dt, m_eta.ResidentCount(), m_eta.TilesX() * m_eta.TilesY(),
        m_flux.ResidentCount(), m_flux.TilesX() * m_flux.TilesY(),
        (m_eta.ResidentBytes() + m_flux.ResidentBytes()) / 1048576.0);
    m_ready = true;
    m_pendingReset = true;
}

void SweSolver::RecordReset(ID3D12GraphicsCommandList* cl, Gpu& gpu) {
    PixScope scope(cl, "swe.reset (state -> the analytic tide plane)");
    cl->SetComputeRootSignature(m_rs.Get());
    cl->SetComputeRootConstantBufferView(0, gpu.PushConstants(&m_cb, sizeof(m_cb)));
    cl->SetComputeRootDescriptorTable(2, gpu.SrvHeap().Gpu(m_table));

    const auto& el = m_eta.ResidentList();
    cl->SetComputeRootShaderResourceView(1, gpu.PushConstants(el.data(), el.size() * 4));
    cl->SetPipelineState(m_clearEta.Get());
    cl->Dispatch(m_eta.TileW() / 16, m_eta.TileH() / 16, static_cast<UINT>(el.size()));

    const auto& fl = m_flux.ResidentList();
    cl->SetComputeRootShaderResourceView(1, gpu.PushConstants(fl.data(), fl.size() * 4));
    cl->SetPipelineState(m_clearFlux.Get());
    cl->Dispatch(m_flux.TileW() / 16, m_flux.TileH() / 16, static_cast<UINT>(fl.size()));

    // Full-surface uv clear (fresh texture memory is undefined; NULL-region texels must read
    // invalid so the sea falls back to the analytic jet there).
    cl->SetPipelineState(m_uvClear.Get());
    cl->Dispatch((m_cb.nx + 15) / 16, (m_cb.ny + 15) / 16, 1);

    D3D12_RESOURCE_BARRIER uav[3]{};
    for (int i = 0; i < 3; ++i) uav[i].Type = D3D12_RESOURCE_BARRIER_TYPE_UAV;
    uav[0].UAV.pResource = m_eta.Res();
    uav[1].UAV.pResource = m_flux.Res();
    uav[2].UAV.pResource = m_uv.res.Get();
    cl->ResourceBarrier(3, uav);
}

int SweSolver::Record(ID3D12GraphicsCommandList* cl, Gpu& gpu, double simUnix, float tideNavd) {
    return Record(cl, gpu, simUnix, tideNavd, kMaxSubsteps);
}

int SweSolver::Record(ID3D12GraphicsCommandList* cl, Gpu& gpu, double simUnix, float tideNavd,
                      int maxSub) {
    if (!m_ready) return 0;
    PixScope scope(cl, "swe (sparse shallow-water: flux/height substeps + derive)");

    // Spin-up runs on the raw upload list, which has no descriptor heap bound; the frame list
    // already has this exact heap, so re-setting it is harmless there.
    ID3D12DescriptorHeap* heaps[] = {gpu.SrvHeap().Heap()};
    cl->SetDescriptorHeaps(1, heaps);

    auto etaTo = [&](D3D12_RESOURCE_STATES to) {
        if (m_etaState == to) return;
        D3D12_RESOURCE_BARRIER b{};
        b.Type = D3D12_RESOURCE_BARRIER_TYPE_TRANSITION;
        b.Transition.pResource = m_eta.Res();
        b.Transition.Subresource = D3D12_RESOURCE_BARRIER_ALL_SUBRESOURCES;
        b.Transition.StateBefore = m_etaState;
        b.Transition.StateAfter = to;
        cl->ResourceBarrier(1, &b);
        m_etaState = to;
    };
    auto uvTo = [&](D3D12_RESOURCE_STATES to) {
        if (m_uvState == to) return;
        gpu.Transition(cl, m_uv, to);
        m_uvState = to;
    };
    etaTo(D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
    uvTo(D3D12_RESOURCE_STATE_UNORDERED_ACCESS);

    // Clock policy. A true scrub (arrow keys, --start) means the state belongs to a time that
    // no longer exists: snap to the analytic plane. Mere STARVATION (a fast time scale asking
    // for more substeps than the frame budget allows) keeps the state -- the field stays
    // continuous, boundary-forced by the current tide -- and just re-anchors the clock, i.e.
    // time-dilated hydrodynamics instead of a visible reset flicker.
    const double drift = std::abs(simUnix - m_simTime);
    if (m_pendingReset || drift > 3600.0) {
        RecordReset(cl, gpu);
        m_simTime = simUnix;
        m_pendingReset = false;
    } else if (drift > 900.0) {
        m_simTime = simUnix - maxSub * static_cast<double>(m_dt);
    }

    int n = static_cast<int>((simUnix - m_simTime) / m_dt);
    n = (std::max)(0, (std::min)(n, maxSub));

    m_cb.tideNavd = tideNavd;
    m_cb.riverDEta = m_westDEta;   // west-boundary target deviation (river tide - ocean tide)
    m_cb.riverBox[3] = m_southDEta;

    cl->SetComputeRootSignature(m_rs.Get());
    cl->SetComputeRootConstantBufferView(0, gpu.PushConstants(&m_cb, sizeof(m_cb)));
    cl->SetComputeRootDescriptorTable(2, gpu.SrvHeap().Gpu(m_table));
    const auto& el = m_eta.ResidentList();
    const auto& fl = m_flux.ResidentList();
    const D3D12_GPU_VIRTUAL_ADDRESS elVa = gpu.PushConstants(el.data(), el.size() * 4);
    const D3D12_GPU_VIRTUAL_ADDRESS flVa = gpu.PushConstants(fl.data(), fl.size() * 4);

    D3D12_RESOURCE_BARRIER uavEta{}, uavFlux{};
    uavEta.Type = uavFlux.Type = D3D12_RESOURCE_BARRIER_TYPE_UAV;
    uavEta.UAV.pResource = m_eta.Res();
    uavFlux.UAV.pResource = m_flux.Res();

    for (int i = 0; i < n; ++i) {
        cl->SetComputeRootShaderResourceView(1, flVa);
        cl->SetPipelineState(m_fluxK.Get());
        cl->Dispatch(m_flux.TileW() / 16, m_flux.TileH() / 16, static_cast<UINT>(fl.size()));
        cl->ResourceBarrier(1, &uavFlux);

        cl->SetComputeRootShaderResourceView(1, elVa);
        cl->SetPipelineState(m_heightK.Get());
        cl->Dispatch(m_eta.TileW() / 16, m_eta.TileH() / 16, static_cast<UINT>(el.size()));
        cl->ResourceBarrier(1, &uavEta);
    }
    m_simTime += n * m_dt;

    cl->SetComputeRootShaderResourceView(1, elVa);
    cl->SetPipelineState(m_deriveK.Get());
    cl->Dispatch(m_eta.TileW() / 16, m_eta.TileH() / 16, static_cast<UINT>(el.size()));

    // Leave eta + uv sampleable by the domain and pixel shaders.
    etaTo(D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE |
          D3D12_RESOURCE_STATE_PIXEL_SHADER_RESOURCE);
    uvTo(D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE |
         D3D12_RESOURCE_STATE_PIXEL_SHADER_RESOURCE);

    char s[96];
    snprintf(s, sizeof(s), "swe %ut %.0f/%.0fMB %dss", m_eta.ResidentCount() + m_flux.ResidentCount(),
             (m_eta.ResidentBytes() + m_flux.ResidentBytes()) / 1048576.0,
             (m_eta.VirtualBytes() + m_flux.VirtualBytes()) / 1048576.0, n);
    stats = s;
    return n;
}

std::vector<uint8_t> SweSolver::ReadFluxRaw(Gpu& gpu, uint32_t* outW, uint32_t* outH,
                                            uint32_t* outPitch) {
    GpuTexture wrap;
    wrap.res = m_flux.Res();
    wrap.format = DXGI_FORMAT_R32G32_FLOAT;
    wrap.width = m_flux.TilesX() * m_flux.TileW();
    wrap.height = m_flux.TilesY() * m_flux.TileH();
    wrap.state = D3D12_RESOURCE_STATE_UNORDERED_ACCESS;   // flux stays UA between frames
    *outW = wrap.width;
    *outH = wrap.height;
    return gpu.ReadbackTexture(wrap, outPitch);
}

void SweSolver::ReadProbes(Gpu& gpu, const float* xzPairs, int count, Probe* out) {
    for (int i = 0; i < count; ++i) out[i] = Probe{0, 0, 0, false};
    if (!m_ready || !m_bathy || count <= 0) return;

    uint32_t etaPitch = 0, uvPitch = 0;
    GpuTexture wrap;
    wrap.res = m_eta.Res();
    wrap.format = DXGI_FORMAT_R32_FLOAT;
    wrap.width = m_eta.TilesX() * m_eta.TileW();
    wrap.height = m_eta.TilesY() * m_eta.TileH();
    wrap.state = m_etaState;
    const std::vector<uint8_t> etaData = gpu.ReadbackTexture(wrap, &etaPitch);
    m_etaState = wrap.state;
    const std::vector<uint8_t> uvData = gpu.ReadbackTexture(m_uv, &uvPitch);
    m_uvState = m_uv.state;

    for (int i = 0; i < count; ++i) {
        const float wx = xzPairs[i * 2], wz = xzPairs[i * 2 + 1];
        const int tx =
            static_cast<int>((wx - m_bathy->WorldX0()) / m_bathy->WorldSizeX() * m_cb.nx);
        // Bathy row 0 is the NORTHERN edge; world z grows north.
        const int ty = static_cast<int>((m_bathy->WorldZ0() + m_bathy->WorldSizeZ() - wz) /
                                        m_bathy->WorldSizeZ() * m_cb.ny);
        if (tx < 0 || ty < 0 || tx >= static_cast<int>(m_cb.nx) ||
            ty >= static_cast<int>(m_cb.ny)) {
            continue;
        }
        out[i].dEta = *reinterpret_cast<const float*>(&etaData[ty * etaPitch + tx * 4]);
        const uint16_t* px = reinterpret_cast<const uint16_t*>(&uvData[ty * uvPitch + tx * 8]);
        out[i].u = HalfToFloat(px[0]);
        out[i].v = HalfToFloat(px[1]);
        out[i].valid = HalfToFloat(px[3]) > 0.5f;
    }
}

}  // namespace ga
