#include "scene/GisLayer.h"

#include "core/PixEvents.h"
#include "core/Shader.h"

#include <cstring>

namespace ga {

GisLayer::Batch GisLayer::MakeBatch(Gpu& gpu, const std::vector<GisStencil::Polyline>& lines,
                                    const wchar_t* name, float r, float g, float b,
                                    float lift) {
    Batch batch{};
    batch.color[0] = r;
    batch.color[1] = g;
    batch.color[2] = b;
    batch.color[3] = lift;
    size_t segs = 0;
    for (const auto& line : lines) {
        if (line.size() >= 2) segs += line.size() - 1;
    }
    if (segs == 0) return batch;
    std::vector<float> pts;
    pts.reserve(segs * 4);
    for (const auto& line : lines) {
        for (size_t i = 0; i + 1 < line.size(); ++i) {
            pts.push_back(line[i].first);
            pts.push_back(line[i].second);
            pts.push_back(line[i + 1].first);
            pts.push_back(line[i + 1].second);
        }
    }
    batch.buf = gpu.CreateUploadBuffer(pts.size() * sizeof(float), name);
    memcpy(batch.buf.cpu, pts.data(), pts.size() * sizeof(float));
    batch.verts = static_cast<uint32_t>(segs * 2);
    return batch;
}

void GisLayer::Init(Gpu& gpu, ShaderCompiler& sc, FieldSet&, ID3D12RootSignature* rootSig) {
    m_rootSig = rootSig;
    if (!m_gis) return;
    if (!BuildPso(gpu, sc)) throw std::runtime_error("gis PSO failed");
    m_coast = MakeBatch(gpu, m_gis->CoastNe(), L"gis.coastNe (GSHHG f)", 0.15f, 1.0f, 0.25f,
                        4.0f);
    m_rivers = MakeBatch(gpu, m_gis->RiversNe(), L"gis.riversNe (WDBII)", 1.0f, 0.20f, 1.0f,
                         4.0f);
    m_global = MakeBatch(gpu, m_gis->CoastGlobal(), L"gis.coastGlobal (GSHHG l)", 0.15f, 1.0f,
                         0.25f, 40.0f);
    Log("[gis] vector layer: %u + %u + %u line vertices", m_coast.verts, m_rivers.verts,
        m_global.verts);
}

bool GisLayer::BuildPso(Gpu& gpu, ShaderCompiler& sc) {
    const std::wstring path = m_shaderDir + L"/GisVec.hlsl";
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
    d.RasterizerState.CullMode = D3D12_CULL_MODE_NONE;
    // The overlay is X-RAY on purpose (alignment truth over every layer's claim); the far
    // side of the planet is culled in the vertex shader instead of by depth.
    d.DepthStencilState.DepthEnable = FALSE;
    d.DSVFormat = DXGI_FORMAT_D32_FLOAT;
    d.PrimitiveTopologyType = D3D12_PRIMITIVE_TOPOLOGY_TYPE_LINE;
    d.NumRenderTargets = 1;
    d.RTVFormats[0] = DXGI_FORMAT_R16G16B16A16_FLOAT;
    d.SampleDesc.Count = 1;

    Com<ID3D12PipelineState> pso;
    if (FAILED(gpu.Device()->CreateGraphicsPipelineState(&d, IID_PPV_ARGS(&pso)))) return false;
    m_pso = pso;
    return true;
}

void GisLayer::ReloadShaders(Gpu& gpu, ShaderCompiler& sc) {
    Com<ID3D12PipelineState> keep = m_pso;
    if (!BuildPso(gpu, sc)) m_pso = keep;
}

void GisLayer::Render(const FrameContext& ctx) {
    if (!enabled || !m_pso) return;
    PixScope scope(ctx.cl, "gis (survey vectors: the authority, drawn as vectors)");
    ctx.cl->SetPipelineState(m_pso.Get());
    ctx.cl->IASetPrimitiveTopology(D3D_PRIMITIVE_TOPOLOGY_LINELIST);
    auto draw = [&](const Batch& b) {
        if (b.verts == 0) return;
        GisCbData cb{};
        cb.cs = m_cs;
        memcpy(cb.color, b.color, sizeof(cb.color));
        ctx.cl->SetGraphicsRootConstantBufferView(1, ctx.gpu->PushConstants(&cb, sizeof(cb)));
        ctx.cl->SetGraphicsRootShaderResourceView(2, b.buf.res->GetGPUVirtualAddress());
        ctx.cl->DrawInstanced(b.verts, 1, 0, 0);
    };
    draw(m_global);
    draw(m_coast);
    draw(m_rivers);
}

}  // namespace ga
