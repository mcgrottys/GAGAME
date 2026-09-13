#include "scene/GisLayer.h"

#include "hal/PixEvents.h"
#include "hal/Pipeline.h"
#include "hal/Shader.h"

#include <cstring>

namespace ga {

GisLayer::Batch GisLayer::MakeBatch(Gpu& gpu, const std::vector<GisStencil::Polyline>& lines,
                                    const char* channel, float r, float g, float b,
                                    float lift) {
    Batch batch{};
    batch.channel = channel;
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
    const int ch = m_exchange->Register(
        channel, {8, 0b010, "lonlat-degrees line-list (float2 per vertex, 2 per segment)"},
        "gis.vectors");
    m_exchange->Publish(gpu, ch, pts.data(), pts.size() * sizeof(float));
    return batch;
}

void GisLayer::Init(Gpu& gpu, ShaderCompiler& sc, FieldSet&, hal::RootSignature rootSig) {
    m_rootSig = rootSig;
    if (!m_exchange) return;
    if (!BuildPso(gpu, sc)) throw std::runtime_error("gis PSO failed");
    if (m_pack && m_pack->Find("coast_ne")) {
        // M6p: the vpack path -- lossless layers with per-vertex wedge importance; buffers
        // are (re)published per tolerance bucket in Render.
        auto reg = [&](const char* channel, float r, float g, float b, float lift) {
            Batch batch{};
            batch.channel = channel;
            batch.color[0] = r; batch.color[1] = g; batch.color[2] = b; batch.color[3] = lift;
            return batch;
        };
        m_coast = reg("gis.coast.ne", 0.15f, 1.0f, 0.25f, 4.0f);
        m_rivers = reg("gis.rivers.ne", 1.0f, 0.20f, 1.0f, 4.0f);
        m_global = reg("gis.coast.global", 0.15f, 1.0f, 0.25f, 40.0f);
        // Charted coastal STRUCTURES (jetties, groins, dikes -- KML-ingested footprints):
        // amber, so surveyed riprap reads apart from the natural shoreline.
        m_structs = reg("gis.structures", 1.0f, 0.65f, 0.10f, 4.0f);
        const GaBufferLayout lay{8, 0b010,
                                 "lonlat-degrees line-list (wedge-filtered at view LOD)"};
        m_coastCh = m_exchange->Register(m_coast.channel, lay, "gis.vpack");
        m_riversCh = m_exchange->Register(m_rivers.channel, lay, "gis.vpack");
        m_globalCh = m_exchange->Register(m_global.channel, lay, "gis.vpack");
        if (m_pack->Find("structures"))
            m_structsCh = m_exchange->Register(m_structs.channel, lay, "gis.vpack");
        PublishAtTolerance(gpu, 0.0f);
        return;
    }
    if (!m_gis) return;   // fallback: the raw GSHHG polylines, published once, no LOD
    m_coast = MakeBatch(gpu, m_gis->CoastNe(), "gis.coast.ne", 0.15f, 1.0f, 0.25f, 4.0f);
    m_rivers = MakeBatch(gpu, m_gis->RiversNe(), "gis.rivers.ne", 1.0f, 0.20f, 1.0f, 4.0f);
    m_global = MakeBatch(gpu, m_gis->CoastGlobal(), "gis.coast.global", 0.15f, 1.0f, 0.25f,
                         40.0f);
}

void GisLayer::PublishAtTolerance(Gpu& gpu, float tol) {
    auto pub = [&](int ch, const char* layer) {
        const VectorPack::Layer* L = m_pack->Find(layer);
        if (!L || ch < 0) return;
        const std::vector<float> segs = m_pack->Segments(*L, tol);
        m_exchange->Publish(gpu, ch, segs.data(), segs.size() * sizeof(float));
    };
    pub(m_coastCh, "coast_ne");
    pub(m_riversCh, "rivers_ne");
    pub(m_globalCh, "coast_global");
    pub(m_structsCh, "structures");
    m_bucket = tol;
}

bool GisLayer::BuildPso(Gpu& gpu, ShaderCompiler& sc) {
    const std::wstring path = m_shaderDir + L"/GisVec.hlsl";
    hal::GraphicsPipelineDesc d;
    d.rootSig = m_rootSig;
    d.vs = sc.Compile(path, L"VsMain", L"vs_6_0");
    d.ps = sc.Compile(path, L"PsMain", L"ps_6_0");
    // The overlay is X-RAY on purpose (alignment truth over every layer's claim): depth stays
    // OFF, and the far side of the planet is culled in the vertex shader instead.
    d.topology = D3D12_PRIMITIVE_TOPOLOGY_TYPE_LINE;
    return hal::Reload(m_pso, [&] { return hal::BuildGraphics(gpu, d, "gis"); }, "gis");
}

void GisLayer::ReloadShaders(Gpu& gpu, ShaderCompiler& sc) {
    BuildPso(gpu, sc);   // the reload law lives in BuildPso: swap only on success
}

void GisLayer::Render(const FrameContext& ctx) {
    if (!enabled || !m_pso) return;
    PixScope scope(ctx.cmd->Native(), "gis (survey vectors: the authority, drawn as vectors)");
    if (m_pack && m_coastCh >= 0) {
        // Quantize the view's ground-pixel size to x8 buckets; republish only on change.
        float bucket = 0.0f;
        for (float b = 8.0f; b <= 4096.0f; b *= 8.0f) {
            if (tolMeters >= b) bucket = b;
        }
        if (bucket != m_bucket) PublishAtTolerance(*ctx.gpu, bucket);
    }
    ctx.cmd->Pipeline(m_pso.Get());
    ctx.cmd->Topology(D3D_PRIMITIVE_TOPOLOGY_LINELIST);
    auto draw = [&](const Batch& b) {
        const Exchange::View v = m_exchange->Query(b.channel);
        if (!v.valid || v.elements == 0) return;
        GisCbData cb{};
        cb.cs = m_cs;
        memcpy(cb.color, b.color, sizeof(cb.color));
        ctx.cmd->GraphicsConstants(1, cb);
        ctx.cmd->GraphicsSrvAt(2, v.va);
        ctx.cmd->Draw(v.elements, 1, 0, 0);
    };
    draw(m_global);
    draw(m_coast);
    draw(m_rivers);
    draw(m_structs);
}

}  // namespace ga
