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

    m_tex = gpu.CreateTexture2D(m_bathy->Nx(), m_bathy->Ny(), DXGI_FORMAT_R32_FLOAT,
                                D3D12_RESOURCE_FLAG_NONE, D3D12_RESOURCE_STATE_COPY_DEST,
                                L"terrain.height (CUDEM NAVD88 m)");
    gpu.UploadTexture(m_tex, m_bathy->Elev().data(), m_bathy->Nx() * 4);
    m_tex.srv = gpu.CreateSrv(m_tex.res.Get(), DXGI_FORMAT_R32_FLOAT);
    // The sea's DOMAIN shader and the M5c SWE compute kernels read this too; PIXEL alone was
    // never a legal state for those reads (a latent M5b bug this driver forgave).
    {
        ID3D12GraphicsCommandList* cl = gpu.BeginUpload();
        gpu.Transition(cl, m_tex, D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE |
                                      D3D12_RESOURCE_STATE_PIXEL_SHADER_RESOURCE);
        gpu.EndUpload();
    }

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
    if (!m_pso || !renderEnabled) return;
    PixScope scope(ctx.cl, "terrain (CUDEM topobathy at true scale)");

    TerrainCbData cb{};
    cb.geo[0] = m_bathy->WorldX0();
    cb.geo[1] = m_bathy->WorldZ0();
    cb.geo[2] = m_bathy->WorldSizeX();
    cb.geo[3] = m_bathy->WorldSizeZ();
    cb.srv[0] = m_tex.srv;
    cb.srv[1] = m_quadsX;
    cb.srv[2] = m_quadsZ;
    cb.params[0] = waterNavd;
    cb.params[1] = m_bathy->WorldSizeX() / m_bathy->Nx();   // texel world size
    cb.cs = m_cs;
    ctx.cl->SetPipelineState(m_pso.Get());
    ctx.cl->IASetPrimitiveTopology(D3D_PRIMITIVE_TOPOLOGY_TRIANGLELIST);
    ctx.cl->SetGraphicsRootConstantBufferView(1, ctx.gpu->PushConstants(&cb, sizeof(cb)));
    ctx.cl->DrawInstanced(6 * m_quadsX * m_quadsZ, 1, 0, 0);
}

}  // namespace ga
