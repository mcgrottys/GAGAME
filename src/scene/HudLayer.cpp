#include "scene/HudLayer.h"

#include "core/Font5x7.h"
#include "hal/PixEvents.h"
#include "hal/Pipeline.h"
#include "hal/Shader.h"

namespace ga {

void HudLayer::Init(Gpu& gpu, ShaderCompiler& sc, FieldSet&, hal::RootSignature rootSig) {
    m_rootSig = rootSig;
    if (!BuildPso(gpu, sc)) throw std::runtime_error("hud PSO failed");
}

bool HudLayer::BuildPso(Gpu& gpu, ShaderCompiler& sc) {
    const std::wstring path = m_shaderDir + L"/Hud.hlsl";
    hal::GraphicsPipelineDesc d;
    d.rootSig = m_rootSig;
    d.vs = sc.Compile(path, L"VsMain", L"vs_6_0");
    d.ps = sc.Compile(path, L"PsMain", L"ps_6_0");
    d.blend = true;   // over the picture by the element's alpha
    d.srcBlend = D3D12_BLEND_SRC_ALPHA;
    d.dstBlend = D3D12_BLEND_INV_SRC_ALPHA;
    d.dstBlendAlpha = D3D12_BLEND_ONE;
    d.dsvFormat = DXGI_FORMAT_UNKNOWN;           // the glass has no depth
    d.rtvFormat = DXGI_FORMAT_R8G8B8A8_UNORM;    // the display-referred target, after the curve
    return hal::Reload(m_pso, [&] { return hal::BuildGraphics(gpu, d, "hud"); }, "hud");
}

void HudLayer::ReloadShaders(Gpu& gpu, ShaderCompiler& sc) {
    BuildPso(gpu, sc);   // the reload law lives in BuildPso: swap only on success
}

void HudLayer::Rect(float x, float y, float w, float h, Kind kind, Rgba c, float edge) {
    Element e{};
    e.rect[0] = x;
    e.rect[1] = y;
    e.rect[2] = x + w;
    e.rect[3] = y + h;
    e.color[0] = c.r;
    e.color[1] = c.g;
    e.color[2] = c.b;
    e.color[3] = c.a;
    e.kind = kind;
    e.edge = edge;
    m_el.push_back(e);
}

float HudLayer::Text(float x, float y, float px, const std::string& s, Rgba c) {
    // The shadow first, one font pixel down and right, then the face over it.
    for (int pass = 0; pass < 2; ++pass) {
        const float o = pass == 0 ? px : 0.0f;
        const Rgba col = pass == 0 ? Rgba{0.0f, 0.0f, 0.0f, c.a * 0.6f} : c;
        float cx = x;
        for (char ch : s) {
            if (const GlyphRow* g = Glyph5x7(ch)) {
                Rect(cx + o, y + o, 5.0f * px, 7.0f * px, Glyph, col);
                Element& e = m_el.back();
                e.glyphLo = g->col[0] | (g->col[1] << 8) | (g->col[2] << 16) |
                            (uint32_t(g->col[3]) << 24);
                e.glyphHi = g->col[4];
            }
            cx += 6.0f * px;
        }
    }
    return TextWidth(s, px);
}

void HudLayer::Render(const FrameContext& ctx) {
    if (!m_pso || m_el.empty()) return;
    PixScope scope(ctx.cmd->Native(), "hud (the windshield: rectangles on the glass)");
    const float cb[4] = {float(ctx.width), float(ctx.height), 1.0f / float(ctx.width),
                         1.0f / float(ctx.height)};
    ctx.cmd->Pipeline(m_pso.Get());
    ctx.cmd->Topology(D3D_PRIMITIVE_TOPOLOGY_TRIANGLELIST);
    ctx.cmd->GraphicsConstants(1, cb);
    ctx.cmd->GraphicsSrvAt(2, ctx.gpu->PushConstants(m_el.data(), m_el.size() * sizeof(Element)));
    ctx.cmd->Draw(6, static_cast<UINT>(m_el.size()), 0, 0);
}

}  // namespace ga
