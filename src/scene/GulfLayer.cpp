#include "scene/GulfLayer.h"

#include "core/PixEvents.h"
#include "scene/FieldSet.h"

#include <cmath>
#include <vector>

namespace ga {

namespace {
constexpr double kPi = 3.14159265358979323846;

void Barrier(ID3D12GraphicsCommandList* cl, ID3D12Resource* res, D3D12_RESOURCE_STATES from,
             D3D12_RESOURCE_STATES to) {
    D3D12_RESOURCE_BARRIER b{};
    b.Type = D3D12_RESOURCE_BARRIER_TYPE_TRANSITION;
    b.Transition.pResource = res;
    b.Transition.Subresource = D3D12_RESOURCE_BARRIER_ALL_SUBRESOURCES;
    b.Transition.StateBefore = from;
    b.Transition.StateAfter = to;
    cl->ResourceBarrier(1, &b);
}
}  // namespace

void GulfLayer::Init(Gpu& gpu, ShaderCompiler& sc, FieldSet& fields,
                     ID3D12RootSignature* rootSig) {
    (void)fields;
    m_rootSig = rootSig;
    if (!m_currents || !m_currents->Field().Valid()) {
        throw std::runtime_error("GulfLayer needs a current field");
    }
    const CurrentField& f = m_currents->Field();

    if (!BuildDrawPso(gpu, sc)) throw std::runtime_error("gulf draw PSO failed");

    // ---- upload the field as (u, v, mask, 0) RGBA32F
    std::vector<float> texels(static_cast<size_t>(f.nx) * f.ny * 4);
    for (int i = 0; i < f.nx * f.ny; ++i) {
        const bool water = f.u[i] > -900.0f;
        texels[i * 4 + 0] = water ? f.u[i] : 0.0f;
        texels[i * 4 + 1] = water ? f.v[i] : 0.0f;
        texels[i * 4 + 2] = water ? 1.0f : 0.0f;
        texels[i * 4 + 3] = 0.0f;
    }
    m_uvTex = gpu.CreateTexture2D(f.nx, f.ny, DXGI_FORMAT_R32G32B32A32_FLOAT,
                                  D3D12_RESOURCE_FLAG_ALLOW_UNORDERED_ACCESS,
                                  D3D12_RESOURCE_STATE_COPY_DEST, L"gulf.uvField");
    gpu.UploadTexture(m_uvTex, texels.data(), f.nx * 16);
    m_uvSrv = gpu.CreateSrv(m_uvTex.res.Get(), DXGI_FORMAT_R32G32B32A32_FLOAT);

    auto makeOut = [&](const wchar_t* name) {
        D3D12_RESOURCE_DESC d{};
        d.Dimension = D3D12_RESOURCE_DIMENSION_TEXTURE2D;
        d.Width = static_cast<UINT64>(f.nx);
        d.Height = static_cast<UINT>(f.ny);
        d.DepthOrArraySize = 1;
        d.MipLevels = 1;
        d.Format = DXGI_FORMAT_R16G16B16A16_FLOAT;
        d.SampleDesc.Count = 1;
        d.Flags = D3D12_RESOURCE_FLAG_ALLOW_UNORDERED_ACCESS;
        D3D12_HEAP_PROPERTIES hp{};
        hp.Type = D3D12_HEAP_TYPE_DEFAULT;
        Com<ID3D12Resource> res;
        GA_CHECK(gpu.Device()->CreateCommittedResource(&hp, D3D12_HEAP_FLAG_NONE, &d,
                                                      D3D12_RESOURCE_STATE_UNORDERED_ACCESS,
                                                      nullptr, IID_PPV_ARGS(&res)));
        res->SetName(name);
        return res;
    };
    m_mvTex = makeOut(L"gulf.mv2 (div, u, v, curl)");
    m_owTex = makeOut(L"gulf.okuboWeiss");
    m_mvSrv = gpu.CreateSrv(m_mvTex.Get(), DXGI_FORMAT_R16G16B16A16_FLOAT);
    m_owSrv = gpu.CreateSrv(m_owTex.Get(), DXGI_FORMAT_R16G16B16A16_FLOAT);

    RunVelGrad(gpu, sc);

    // ---- static CB parts
    const double midLat = f.lat0 + 0.5 * f.dlat * f.ny;
    const double lonSpanKm = f.dlon * f.nx * 111.320 * std::cos(midLat * kPi / 180.0);
    const double latSpanKm = f.dlat * f.ny * 110.574;
    m_aspectWoverH = static_cast<float>(lonSpanKm / latSpanKm);
    m_cb.srvs[0] = m_uvSrv;
    m_cb.srvs[1] = m_mvSrv;
    m_cb.srvs[2] = m_owSrv;
    m_cb.params[0] = 0.55f;      // colour ramp full scale, m/s
    m_cb.params[1] = 3.0f;       // |OW| threshold in the STORED 1e-10 1/s^2 units (= 3e-10)
    m_cb.params[2] = 1.7f;       // brightness into the ACES curve
    m_cb.params[3] = 0.5f;       // graticule spacing, degrees
    m_cb.geo[0] = static_cast<float>(f.lon0);
    m_cb.geo[1] = static_cast<float>(f.lat0);
    m_cb.geo[2] = static_cast<float>(f.dlon * f.nx);
    m_cb.geo[3] = static_cast<float>(f.dlat * f.ny);
    m_cb.marker[0] = static_cast<float>((kBuoyLon - f.lon0) / (f.dlon * f.nx));
    m_cb.marker[1] = static_cast<float>((kBuoyLat - f.lat0) / (f.dlat * f.ny));
    m_cb.marker[2] = 0.012f;
    m_cb.marker[3] = 1.0f;

    // ---- numeric gate: field at the buoy vs its ADCP
    float mu = 0, mv = 0;
    char buf[160];
    if (m_currents->Field().Sample(kBuoyLon, kBuoyLat, mu, mv)) {
        const double ms = std::sqrt(mu * mu + mv * mv);
        const double toward = std::fmod(std::atan2(mu, mv) * 180.0 / kPi + 360.0, 360.0);
        if (m_currents->adcpValid) {
            snprintf(buf, sizeof(buf),
                     "44029 obs %.2f m/s @%03.0f | GoMOFS %.2f @%03.0f",
                     m_currents->adcpMs, m_currents->adcpTowardDeg, ms, toward);
        } else {
            snprintf(buf, sizeof(buf), "GoMOFS at 44029: %.2f m/s @%03.0f", ms, toward);
        }
        validation = buf;
        Log("[gulf] %s", buf);
    }
    m_haveField = true;
}

bool GulfLayer::BuildDrawPso(Gpu& gpu, ShaderCompiler& sc) {
    const std::wstring path = m_shaderDir + L"/Gulf.hlsl";
    ShaderBlob vs = sc.Compile(path, L"VsMain", L"vs_6_0");
    ShaderBlob ps = sc.Compile(path, L"PsMain", L"ps_6_0");
    if (!vs.Valid() || !ps.Valid()) return false;

    D3D12_GRAPHICS_PIPELINE_STATE_DESC d{};
    d.pRootSignature = m_rootSig;
    d.VS = {vs.Data(), vs.Size()};
    d.PS = {ps.Data(), ps.Size()};
    d.BlendState.RenderTarget[0].SrcBlend = D3D12_BLEND_ONE;
    d.BlendState.RenderTarget[0].DestBlend = D3D12_BLEND_ZERO;
    d.BlendState.RenderTarget[0].BlendOp = D3D12_BLEND_OP_ADD;
    d.BlendState.RenderTarget[0].SrcBlendAlpha = D3D12_BLEND_ONE;
    d.BlendState.RenderTarget[0].DestBlendAlpha = D3D12_BLEND_ZERO;
    d.BlendState.RenderTarget[0].BlendOpAlpha = D3D12_BLEND_OP_ADD;
    d.BlendState.RenderTarget[0].RenderTargetWriteMask = D3D12_COLOR_WRITE_ENABLE_ALL;
    d.SampleMask = UINT_MAX;
    d.RasterizerState.FillMode = D3D12_FILL_MODE_SOLID;
    d.RasterizerState.CullMode = D3D12_CULL_MODE_NONE;
    d.DepthStencilState.DepthEnable = FALSE;
    d.DepthStencilState.DepthWriteMask = D3D12_DEPTH_WRITE_MASK_ZERO;
    d.DSVFormat = DXGI_FORMAT_D32_FLOAT;
    d.PrimitiveTopologyType = D3D12_PRIMITIVE_TOPOLOGY_TYPE_TRIANGLE;
    d.NumRenderTargets = 1;
    d.RTVFormats[0] = DXGI_FORMAT_R16G16B16A16_FLOAT;
    d.SampleDesc.Count = 1;

    Com<ID3D12PipelineState> pso;
    if (FAILED(gpu.Device()->CreateGraphicsPipelineState(&d, IID_PPV_ARGS(&pso)))) return false;
    m_drawPso = pso;
    return true;
}

void GulfLayer::RunVelGrad(Gpu& gpu, ShaderCompiler& sc) {
    // Tiny dedicated compute root signature: constants + one 3-UAV table.
    if (!m_csRootSig) {
        D3D12_DESCRIPTOR_RANGE1 range{};
        range.RangeType = D3D12_DESCRIPTOR_RANGE_TYPE_UAV;
        range.NumDescriptors = 3;
        range.BaseShaderRegister = 0;
        range.Flags = D3D12_DESCRIPTOR_RANGE_FLAG_DESCRIPTORS_VOLATILE;

        D3D12_ROOT_PARAMETER1 params[2]{};
        params[0].ParameterType = D3D12_ROOT_PARAMETER_TYPE_32BIT_CONSTANTS;
        params[0].Constants.ShaderRegister = 0;
        params[0].Constants.Num32BitValues = 4;
        params[1].ParameterType = D3D12_ROOT_PARAMETER_TYPE_DESCRIPTOR_TABLE;
        params[1].DescriptorTable.NumDescriptorRanges = 1;
        params[1].DescriptorTable.pDescriptorRanges = &range;

        D3D12_VERSIONED_ROOT_SIGNATURE_DESC vd{};
        vd.Version = D3D_ROOT_SIGNATURE_VERSION_1_1;
        vd.Desc_1_1.NumParameters = _countof(params);
        vd.Desc_1_1.pParameters = params;
        Com<ID3DBlob> blob, err;
        GA_CHECK(D3D12SerializeVersionedRootSignature(&vd, &blob, &err));
        GA_CHECK(gpu.Device()->CreateRootSignature(0, blob->GetBufferPointer(),
                                                  blob->GetBufferSize(),
                                                  IID_PPV_ARGS(&m_csRootSig)));
        m_csRootSig->SetName(L"gulf velgrad root signature");

        m_csTable = gpu.SrvHeap().Alloc(3);
        D3D12_UNORDERED_ACCESS_VIEW_DESC uv{};
        uv.ViewDimension = D3D12_UAV_DIMENSION_TEXTURE2D;
        uv.Format = DXGI_FORMAT_R32G32B32A32_FLOAT;
        gpu.Device()->CreateUnorderedAccessView(m_uvTex.res.Get(), nullptr, &uv,
                                                gpu.SrvHeap().Cpu(m_csTable + 0));
        uv.Format = DXGI_FORMAT_R16G16B16A16_FLOAT;
        gpu.Device()->CreateUnorderedAccessView(m_mvTex.Get(), nullptr, &uv,
                                                gpu.SrvHeap().Cpu(m_csTable + 1));
        gpu.Device()->CreateUnorderedAccessView(m_owTex.Get(), nullptr, &uv,
                                                gpu.SrvHeap().Cpu(m_csTable + 2));
    }

    ShaderBlob cs = sc.Compile(m_shaderDir + L"/VelGrad.hlsl", L"CsVelGrad", L"cs_6_0");
    if (!cs.Valid()) throw std::runtime_error("VelGrad.hlsl failed to compile");
    D3D12_COMPUTE_PIPELINE_STATE_DESC cd{};
    cd.pRootSignature = m_csRootSig.Get();
    cd.CS = {cs.Data(), cs.Size()};
    GA_CHECK(gpu.Device()->CreateComputePipelineState(&cd, IID_PPV_ARGS(&m_csPso)));
    m_csPso->SetName(L"CsVelGrad");

    const CurrentField& f = m_currents->Field();
    struct { uint32_t nx, ny; float cx, cy; } c{};
    c.nx = static_cast<uint32_t>(f.nx);
    c.ny = static_cast<uint32_t>(f.ny);
    const double midLat = f.lat0 + 0.5 * f.dlat * f.ny;
    c.cx = static_cast<float>(f.dlon * 111320.0 * std::cos(midLat * kPi / 180.0));
    c.cy = static_cast<float>(f.dlat * 110574.0);

    auto* cl = gpu.BeginUpload();
    ID3D12DescriptorHeap* heaps[] = {gpu.SrvHeap().Heap()};
    cl->SetDescriptorHeaps(1, heaps);
    {
        PixScope scope(cl, "gulf.velgrad (grad(U): div->grade0, vorticity->grade2, OW)");
        Barrier(cl, m_uvTex.res.Get(), m_uvTex.state, D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
        cl->SetComputeRootSignature(m_csRootSig.Get());
        cl->SetComputeRoot32BitConstants(0, 4, &c, 0);
        cl->SetComputeRootDescriptorTable(1, gpu.SrvHeap().Gpu(m_csTable));
        cl->SetPipelineState(m_csPso.Get());
        cl->Dispatch((c.nx + 7) / 8, (c.ny + 7) / 8, 1);
        Barrier(cl, m_uvTex.res.Get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS,
                D3D12_RESOURCE_STATE_PIXEL_SHADER_RESOURCE);
        Barrier(cl, m_mvTex.Get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS,
                D3D12_RESOURCE_STATE_PIXEL_SHADER_RESOURCE);
        Barrier(cl, m_owTex.Get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS,
                D3D12_RESOURCE_STATE_PIXEL_SHADER_RESOURCE);
    }
    gpu.EndUpload();
    m_uvTex.state = D3D12_RESOURCE_STATE_PIXEL_SHADER_RESOURCE;
}

void GulfLayer::ReloadShaders(Gpu& gpu, ShaderCompiler& sc) {
    Com<ID3D12PipelineState> keep = m_drawPso;
    if (!BuildDrawPso(gpu, sc)) {
        m_drawPso = keep;
        Log("[gulf] reload failed; keeping the previous PSO");
    }
}

void GulfLayer::Render(const FrameContext& ctx) {
    if (!m_haveField || !m_drawPso) return;
    PixScope scope(ctx.cl, "gulf.map (speed ramp + OW eddies by bivector sign)");

    // Letterbox the geographic aspect into the viewport.
    const float vpAspect = static_cast<float>(ctx.width) / static_cast<float>(ctx.height);
    float halfW = 0.94f, halfH = 0.94f;
    const float panelAspect = m_aspectWoverH / vpAspect;   // ndc width per ndc height
    if (panelAspect > 1.0f) halfH = 0.94f / panelAspect;
    else halfW = 0.94f * panelAspect;
    m_cb.panel[0] = 0.0f;
    m_cb.panel[1] = 0.0f;
    m_cb.panel[2] = halfW;
    m_cb.panel[3] = halfH;

    ctx.cl->SetPipelineState(m_drawPso.Get());
    ctx.cl->IASetPrimitiveTopology(D3D_PRIMITIVE_TOPOLOGY_TRIANGLELIST);
    ctx.cl->SetGraphicsRootConstantBufferView(1, ctx.gpu->PushConstants(&m_cb, sizeof(m_cb)));
    ctx.cl->DrawInstanced(6, 1, 0, 0);
}

}  // namespace ga
