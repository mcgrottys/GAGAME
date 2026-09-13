#include "scene/MarkerLayer.h"

#include "hal/PixEvents.h"
#include "hal/Shader.h"

namespace ga {

void MarkerLayer::Init(Gpu& gpu, ShaderCompiler& sc, FieldSet&, ID3D12RootSignature* rootSig) {
    m_rootSig = rootSig;
    if (!BuildPso(gpu, sc)) throw std::runtime_error("marker PSO failed");
}

bool MarkerLayer::BuildPso(Gpu& gpu, ShaderCompiler& sc) {
    const std::wstring path = m_shaderDir + L"/Markers.hlsl";
    ShaderBlob vs = sc.Compile(path, L"VsMain", L"vs_6_0");
    ShaderBlob ps = sc.Compile(path, L"PsMain", L"ps_6_0");
    if (!vs.Valid() || !ps.Valid()) return false;

    D3D12_GRAPHICS_PIPELINE_STATE_DESC d{};
    d.pRootSignature = m_rootSig;
    d.VS = {vs.Data(), vs.Size()};
    d.PS = {ps.Data(), ps.Size()};
    d.BlendState.RenderTarget[0].RenderTargetWriteMask = D3D12_COLOR_WRITE_ENABLE_ALL;
    d.SampleMask = UINT_MAX;
    d.RasterizerState.FillMode = D3D12_FILL_MODE_SOLID;
    d.RasterizerState.CullMode = D3D12_CULL_MODE_BACK;
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
    if (FAILED(gpu.Device()->CreateGraphicsPipelineState(&d, IID_PPV_ARGS(&pso)))) return false;
    m_pso = pso;
    return true;
}

void MarkerLayer::ReloadShaders(Gpu& gpu, ShaderCompiler& sc) {
    Com<ID3D12PipelineState> keep = m_pso;
    if (!BuildPso(gpu, sc)) m_pso = keep;
}

void MarkerLayer::Render(const FrameContext& ctx) {
    if (!m_pso || !m_exchange) return;
    const Exchange::View v = m_exchange->Query(m_channel);
    if (!v.valid || v.elements == 0) return;
    PixScope scope(ctx.cl, "markers (GA product buffer -> motor sandwich on the GPU)");
    float cb[4] = {static_cast<float>(v.elements), 1.0f, 0.0f, 0.0f};
    ctx.cl->SetPipelineState(m_pso.Get());
    ctx.cl->IASetPrimitiveTopology(D3D_PRIMITIVE_TOPOLOGY_TRIANGLELIST);
    ctx.cl->SetGraphicsRootConstantBufferView(1, ctx.gpu->PushConstants(cb, sizeof(cb)));
    ctx.cl->SetGraphicsRootShaderResourceView(2, v.va);
    ctx.cl->DrawInstanced(v.elements * 36, 1, 0, 0);
}

}  // namespace ga
