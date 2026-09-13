#include "scene/SkyLayer.h"

#include "scene/FieldSet.h"

namespace ga {

void SkyLayer::Init(Gpu& gpu, ShaderCompiler& sc, FieldSet& fields,
                    ID3D12RootSignature* rootSig) {
    (void)fields;   // the sky reads no field data
    m_rootSig = rootSig;
    if (!BuildPso(gpu, sc)) throw std::runtime_error("sky PSO could not be created");
}

bool SkyLayer::BuildPso(Gpu& gpu, ShaderCompiler& sc) {
    const std::wstring path = m_shaderDir + L"/Sky.hlsl";
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
    // Depth OFF for both test and write. The sky is a backdrop: it must not occlude anything, and
    // it must not leave depth values behind.
    d.DepthStencilState.DepthEnable = FALSE;
    d.DepthStencilState.DepthWriteMask = D3D12_DEPTH_WRITE_MASK_ZERO;
    d.DSVFormat = DXGI_FORMAT_D32_FLOAT;
    d.PrimitiveTopologyType = D3D12_PRIMITIVE_TOPOLOGY_TYPE_TRIANGLE;
    d.NumRenderTargets = 1;
    d.RTVFormats[0] = DXGI_FORMAT_R16G16B16A16_FLOAT;
    d.SampleDesc.Count = 1;

    Com<ID3D12PipelineState> pso;
    const HRESULT hr = gpu.Device()->CreateGraphicsPipelineState(&d, IID_PPV_ARGS(&pso));
    if (FAILED(hr)) {
        Log("[sky] PSO: %s", HrString(hr).c_str());
        return false;
    }
    m_pso = pso;
    return true;
}

void SkyLayer::ReloadShaders(Gpu& gpu, ShaderCompiler& sc) {
    Com<ID3D12PipelineState> keep = m_pso;
    if (!BuildPso(gpu, sc)) {
        m_pso = keep;
        Log("[sky] reload failed; keeping the previous PSO");
    }
}

void SkyLayer::Render(const FrameContext& ctx) {
    if (!m_pso) return;
    // M10: the sky's frame (Sky.hlsl SkyFrameCb, b1): three rotation rows and the sun.
    struct { float r0[4], r1[4], r2[4], sun[4]; } cb{};
    for (int i = 0; i < 3; ++i) {
        cb.r0[i] = m_rot[i];
        cb.r1[i] = m_rot[3 + i];
        cb.r2[i] = m_rot[6 + i];
        cb.sun[i] = m_sun[i];
    }
    ctx.cmd->Pipeline(m_pso.Get());
    ctx.cmd->GraphicsConstants(1, cb);
    ctx.cmd->DrawFullscreen();
}

}  // namespace ga
