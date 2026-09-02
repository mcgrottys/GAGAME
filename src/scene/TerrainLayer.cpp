#include "scene/TerrainLayer.h"

#include "core/PixEvents.h"
#include "scene/FieldSet.h"

#include <vector>

namespace ga {

void TerrainLayer::Init(Gpu& gpu, ShaderCompiler& sc, FieldSet& fields,
                        ID3D12RootSignature* rootSig) {
    (void)fields;
    m_rootSig = rootSig;
    if (!m_bathy || !m_bathy->Ready()) throw std::runtime_error("TerrainLayer needs bathymetry");
    if (!BuildPso(gpu, sc)) throw std::runtime_error("terrain PSO failed");


    // Half-resolution grid geometry over a full-resolution normal texture: ~13 m quads carry the
    // shape, per-pixel normals carry the 6.8 m detail (the jetty riprap edge in particular).
    m_quadsX = static_cast<uint32_t>(m_bathy->Nx() / 2);
    m_quadsZ = static_cast<uint32_t>(m_bathy->Ny() / 2);
    Log("[terrain] %ux%u quads over %dx%d texels", m_quadsX, m_quadsZ, m_bathy->Nx(),
        m_bathy->Ny());
}

bool TerrainLayer::BuildPso(Gpu& gpu, ShaderCompiler& sc) {
    const std::wstring path = m_shaderDir + L"/Terrain.hlsl";
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

void TerrainLayer::ReloadShaders(Gpu& gpu, ShaderCompiler& sc) {
    Com<ID3D12PipelineState> keep = m_pso;
    if (!BuildPso(gpu, sc)) {
        m_pso = keep;
        Log("[terrain] reload failed; keeping the previous PSO");
    }
}

void TerrainLayer::Render(const FrameContext& ctx) {
    (void)ctx;
    if (!m_pso || !renderEnabled) return;
    // M9aw: THIS LAYER HAS NO BED. The bank it used to build (BuildBedBank, M9k..M9an) was a
    // second GPU realization of the height stack the megatexture already carries, with no
    // caller since the planet mesh took over the ground; it is deleted. The ground is drawn by
    // GlobeMesh from the height pages. Until this layer reads those pages itself it draws
    // nothing, and says so once -- no fallback (M9an's rule stands).
    static bool warned = false;
    if (!warned) {
        warned = true;
        Log("[terrain] draws nothing: no bed of its own (the height pages are the bed; the "
            "planet mesh draws the ground)");
    }
    return;
}

}  // namespace ga
