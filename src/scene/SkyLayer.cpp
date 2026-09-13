#include "scene/SkyLayer.h"

#include "hal/Pipeline.h"
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
    // THE DEFAULTS, as the builder states them, were read from this block: opaque, cull none,
    // depth OFF for both test and write (the sky is a backdrop: it must not occlude anything,
    // and it must not leave depth values behind), the HDR target, triangles.
    hal::GraphicsPipelineDesc d;
    d.rootSig = m_rootSig;
    d.vs = sc.Compile(path, L"VsMain", L"vs_6_0");
    d.ps = sc.Compile(path, L"PsMain", L"ps_6_0");
    return hal::Reload(m_pso, [&] { return hal::BuildGraphics(gpu, d, "sky"); }, "sky");
}

void SkyLayer::ReloadShaders(Gpu& gpu, ShaderCompiler& sc) {
    BuildPso(gpu, sc);   // the reload law lives in BuildPso: swap only on success
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
