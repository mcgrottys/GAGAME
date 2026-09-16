#include "scene/SkyLayer.h"

#include "hal/Pipeline.h"
#include "scene/FieldSet.h"

namespace ga {

void SkyLayer::Init(Gpu& gpu, ShaderCompiler& sc, FieldSet& fields,
                    hal::RootSignature rootSig) {
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
    struct {
        float r0[4], r1[4], r2[4], sun[4];
        float boxR0[4], boxR1[4], boxR2[4], boxC[4];   // M13: the gate's window (Sky.hlsl)
        float skyR0[4], skyR1[4], skyR2[4], winSun[4];
    } cb{};
    for (int i = 0; i < 3; ++i) {
        cb.r0[i] = m_rot[i];
        cb.r1[i] = m_rot[3 + i];
        cb.r2[i] = m_rot[6 + i];
        cb.sun[i] = m_sun[i];
        cb.boxR0[i] = m_winBox[i];
        cb.boxR1[i] = m_winBox[3 + i];
        cb.boxR2[i] = m_winBox[6 + i];
        cb.boxC[i] = m_winC[i];
        cb.skyR0[i] = m_winSky[i];
        cb.skyR1[i] = m_winSky[3 + i];
        cb.skyR2[i] = m_winSky[6 + i];
        cb.winSun[i] = m_winSun[i];
    }
    cb.boxR0[3] = m_winHalf[0];
    cb.boxR1[3] = m_winHalf[1];
    cb.boxR2[3] = m_winHalf[2];
    cb.boxC[3] = m_winOn ? 1.0f : 0.0f;   // the one flag the shader tests
    ctx.cmd->Pipeline(m_pso.Get());
    ctx.cmd->GraphicsConstants(1, cb);
    ctx.cmd->DrawFullscreen();
}

}  // namespace ga
