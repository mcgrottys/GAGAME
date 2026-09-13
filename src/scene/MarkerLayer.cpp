#include "scene/MarkerLayer.h"

#include "hal/PixEvents.h"
#include "hal/Pipeline.h"
#include "hal/Shader.h"

namespace ga {

void MarkerLayer::Init(Gpu& gpu, ShaderCompiler& sc, FieldSet&, ID3D12RootSignature* rootSig) {
    m_rootSig = rootSig;
    if (!BuildPso(gpu, sc)) throw std::runtime_error("marker PSO failed");
}

bool MarkerLayer::BuildPso(Gpu& gpu, ShaderCompiler& sc) {
    const std::wstring path = m_shaderDir + L"/Markers.hlsl";
    hal::GraphicsPipelineDesc d;
    d.rootSig = m_rootSig;
    d.vs = sc.Compile(path, L"VsMain", L"vs_6_0");
    d.ps = sc.Compile(path, L"PsMain", L"ps_6_0");
    d.cull = D3D12_CULL_MODE_BACK;
    d.depthClip = TRUE;
    d.depthTest = true;
    d.depthWrite = true;   // reversed-Z GREATER, the default comparison
    return hal::Reload(m_pso, [&] { return hal::BuildGraphics(gpu, d, "marker"); }, "marker");
}

void MarkerLayer::ReloadShaders(Gpu& gpu, ShaderCompiler& sc) {
    BuildPso(gpu, sc);   // the reload law lives in BuildPso: swap only on success
}

void MarkerLayer::Render(const FrameContext& ctx) {
    if (!m_pso || !m_exchange) return;
    const Exchange::View v = m_exchange->Query(m_channel);
    if (!v.valid || v.elements == 0) return;
    PixScope scope(ctx.cmd->Native(), "markers (GA product buffer -> motor sandwich on the GPU)");
    float cb[4] = {static_cast<float>(v.elements), 1.0f, 0.0f, 0.0f};
    ctx.cmd->Pipeline(m_pso.Get());
    ctx.cmd->Topology(D3D_PRIMITIVE_TOPOLOGY_TRIANGLELIST);
    ctx.cmd->GraphicsConstants(1, cb);
    ctx.cmd->GraphicsSrvAt(2, v.va);
    ctx.cmd->Draw(v.elements * 36, 1, 0, 0);
}

}  // namespace ga
