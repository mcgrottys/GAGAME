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



// ================================================================================================
//  M9k: THE BED THROUGH THE GA PATH.
//
//    GA LOAD     GeoGridLoader over the harvester's grid -- the GeoRef comes from the file's
//                own sidecar (row 0 north, nodata -9999), and nodata is ABSENCE, so a tile of
//                pure nodata is never allocated.
//    GA COMPOSE  RasterSource through DomainCompositor: value and weight in the WGS84 exchange
//                frame, resolved into the source's CRS exactly, per page and per level.
//    SPARSE      a paged GradeBank -- reserved array, mip chain, pinned floor, residency map.
//
//  The bed is grade 0: a scalar height. What a PRODUCT of it with anything else becomes is the
//  Cayley closure's business, not this function's.
//
//  Returns the worst disagreement with the committed texture, in metres. That number is the
//  point: nothing switches to this path until it is small, because the bed feeds the solver,
//  the sea shader, the water bank and the globe, and a silent half-texel slide here would move
//  a coastline everywhere at once.
// ================================================================================================
double TerrainLayer::BuildBedBank(Gpu& gpu, const std::string& gridJson) {
    LoaderRegistry reg;
    reg.Register("json", GeoGridLoader::Open);
    reg.Register("f32", GeoGridLoader::Open);
    auto ld = reg.Open(gridJson);
    if (!ld) {
        Log("[bed] %s: no loader -- GA path not built", gridJson.c_str());
        return -1.0;
    }
    const GeoRef gref = ld->Ref();
    const uint32_t nx = m_bathy->Nx(), ny = m_bathy->Ny();

    auto ras = std::make_shared<RasterSource>(std::move(ld), 0);
    if (!ras->Valid()) {
        Log("[bed] loader produced no samples -- GA path not built");
        return -1.0;
    }
    DomainCompositor dc;
    LevelLadder lad;
    lad.level0MetersPerTexel = double(m_bathy->WorldSizeX()) / double(nx);
    dc.SetLadder(lad);
    dc.Add(ras);

    GradeBankDesc d;
    d.name = "terrain.bed (GA path: load -> compose -> sparse)";
    d.width = nx;
    d.height = ny;
    d.fmt = DXGI_FORMAT_R32_FLOAT;   // the bed is metres NAVD; half would cost 0.03 m at 30 m
    d.gradeSig = kG0;                // a scalar height
    d.metersPerTexel = double(m_bathy->WorldSizeX()) / double(nx);
    d.units = "m NAVD88";
    d.range = "-80..95";
    d.mipLevels = 6;
    d.arraySlices = 2;               // page 0 this survey; page 1 reserved for a wider region
    m_bedBank.Init(gpu, d, policy::None());
    m_bedBank.ActivateSlice(gpu, 0);
    m_bedBank.MapAllLevels(gpu, 0);

    // Compose every level of page 0 from the same source. The compositor is asked in lat/lon
    // and resolves into the grid's own CRS, so this is not a resample of m_tex -- it is an
    // independent path to the same ground, which is what makes the comparison mean anything.
    double worst = 0.0;
    uint32_t levels = 0, coveredL0 = 0;
    const std::vector<float>& truth = m_bathy->Elev();
    for (uint32_t lvl = 0; lvl < d.mipLevels; ++lvl) {
        const uint32_t w = (nx >> lvl) ? (nx >> lvl) : 1u;
        const uint32_t h = (ny >> lvl) ? (ny >> lvl) : 1u;
        DomainCompositor::PageGeo geo;
        geo.dLon = gref.scaleX * double(gref.width) / double(w);
        geo.dLat = gref.scaleY * double(gref.height) / double(h);
        geo.lon0 = gref.originX + 0.5 * geo.dLon;
        geo.lat0 = gref.originY + 0.5 * geo.dLat;

        std::vector<float> page, cov;
        const uint32_t covered = dc.ComposePage(PageAddr{lvl, 0, 0}, geo, w, h, 1, page, cov);
        if (!covered) continue;
        if (lvl == 0) {
            coveredL0 = covered;
            // The equivalence check, at full resolution against the model the committed
            // texture is uploaded from.
            if (truth.size() == size_t(nx) * ny) {
                for (size_t i = 0; i < page.size(); ++i) {
                    if (cov[i] <= 0.0f) continue;   // absence is not a disagreement
                    const double e = std::abs(double(page[i]) - double(truth[i]));
                    if (e > worst) worst = e;
                }
            }
        }
        m_bedBank.UploadLevel(gpu, lvl, page.data(), w * sizeof(float), w, h, 0);
        ++levels;
    }
    m_bedReady = levels > 0;
    Log("[bed] GA path: %ux%u, %u levels, %u/%u texels covered at L0, worst |GA - committed| "
        "= %.4f m (%s)",
        nx, ny, levels, coveredL0, nx * ny, worst,
        (worst < 0.01) ? "equivalent" : "DIVERGENT -- do not switch consumers");
    return worst;
}
}  // namespace ga
