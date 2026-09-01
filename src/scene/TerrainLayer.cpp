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
double TerrainLayer::BuildBedBank(Gpu& gpu, const Compositor& comp, int heightChannel,
                                  double mslToNavd88M, const char* datumProv) {
    const uint32_t nx = m_bathy->Nx(), ny = m_bathy->Ny();
    if (nx == 0 || ny == 0) return -1.0;

    // THE STACK, not one file. Every layer of the height channel becomes a DomainSource through
    // the same Sample() the renderer already calls, so the resample, the feather and the edit
    // polygons have exactly one implementation between the two paths.
    auto layers = BuildHeightStack(comp, heightChannel, mslToNavd88M, datumProv);
    if (layers.empty()) {
        Log("[bed] height channel %d has no layers -- GA path not built", heightChannel);
        return -1.0;
    }

    // The rung RealizeFromChannel samples on: the grid's own latitude cell size, constant for
    // every cell. Matching it exactly is what makes the comparison a comparison.
    LevelLadder lad;
    lad.level0MetersPerTexel = m_bathy->Dlat() * BathyModel::kMPerLat;

    DomainCompositor dc;
    dc.SetLadder(lad);
    dc.SetBlend(DomainCompositor::Blend::LayeredOver);   // the stack rule, not the rival rule
    for (auto& l : layers) dc.Add(l);

    GradeBankDesc d;
    d.name = "terrain.bed (GA path: stack -> compose -> sparse)";
    d.width = nx;
    d.height = ny;
    d.fmt = DXGI_FORMAT_R32_FLOAT;   // the bed is metres NAVD; half would cost 0.03 m at 30 m
    d.gradeSig = kG0;                // a scalar height
    d.metersPerTexel = lad.level0MetersPerTexel;
    d.units = "m NAVD88";
    d.range = "-80..95";
    d.mipLevels = 6;
    d.arraySlices = 2;               // page 0 this survey; page 1 reserved for a wider region
    m_bedBank.Init(gpu, d, policy::None());
    m_bedBank.ActivateSlice(gpu, 0);
    m_bedBank.MapAllLevels(gpu, 0);

    // The grid's lattice, exactly as RealizeFromChannel walks it: row 0 is NORTH, so latitude
    // steps negatively from the north edge and dLat carries the sign.
    auto pageGeo = [&](uint32_t w, uint32_t h) {
        DomainCompositor::PageGeo g;
        g.dLon = m_bathy->Dlon() * double(nx) / double(w);
        g.dLat = -m_bathy->Dlat() * double(ny) / double(h);
        g.lon0 = m_bathy->Lon0() + 0.5 * g.dLon;
        g.lat0 = m_bathy->Lat1() + 0.5 * g.dLat;
        return g;
    };

    // ---- FIDELITY FIRST, and separately from correctness. Two different questions hide in one
    // number here, and answering them together would let each excuse the other:
    //
    //   1. did the realization MOVE faithfully?   compose with the datum link set to ZERO --
    //      i.e. blend the layers exactly as the engine does -- and the answer must be 0.0000.
    //   2. is the engine's bed RIGHT?             that is what the real link then changes, and
    //      its size is a finding about the committed bed, not an error in this path.
    //
    // Checking (1) with the correction applied would have reported 0.092 m and left it ambiguous
    // whether the stack was reproduced or merely close.
    double worstRaw = 0.0;
    const std::vector<float>& truth = m_bathy->Elev();
    if (truth.size() == size_t(nx) * ny) {
        auto raw = BuildHeightStack(comp, heightChannel, 0.0, "fidelity probe: engine's own rule");
        DomainCompositor rc;
        rc.SetLadder(lad);
        rc.SetBlend(DomainCompositor::Blend::LayeredOver);
        for (auto& l : raw) rc.Add(l);
        std::vector<float> page, cov;
        rc.ComposePage(PageAddr{0, 0, 0}, pageGeo(nx, ny), nx, ny, 1, page, cov);
        for (size_t i = 0; i < page.size(); ++i) {
            if (cov[i] <= 0.0f) continue;
            worstRaw = (std::max)(worstRaw, std::abs(double(page[i]) - double(truth[i])));
        }
        Log("[bed] realization fidelity: worst |GA(link=0) - committed| = %.4f m (%s)", worstRaw,
            (worstRaw < 0.01) ? "EQUIVALENT -- the six-layer stack is reproduced"
                              : "the stack is NOT reproduced; the datum is not the issue");
    }

    double worst = 0.0;
    uint32_t levels = 0, coveredL0 = 0;
    std::vector<float> l0page;
    for (uint32_t lvl = 0; lvl < d.mipLevels; ++lvl) {
        const uint32_t w = (nx >> lvl) ? (nx >> lvl) : 1u;
        const uint32_t h = (ny >> lvl) ? (ny >> lvl) : 1u;
        std::vector<float> page, cov;
        const uint32_t covered =
            dc.ComposePage(PageAddr{lvl, 0, 0}, pageGeo(w, h), w, h, 1, page, cov);
        if (!covered) continue;
        if (lvl == 0) {
            coveredL0 = covered;
            l0page = page;
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
    // The shipped bed carries the correction, so it is EXPECTED to differ from the committed one.
    // The verdict below is about fidelity; the difference is reported as what it is.
    Log("[bed] GA path: %ux%u, %u levels, %u layers, %u/%u texels covered at L0 -- %s",
        nx, ny, levels, uint32_t(layers.size()), coveredL0, nx * ny,
        (worstRaw < 0.01)
            ? "SAFE TO SWITCH: stack reproduced exactly, and the shipped bed adds the datum link"
            : "DO NOT SWITCH: the stack is not reproduced");
    Log("[bed] shipped GA bed vs committed: %.4f m, all of it the published MSL -> NAVD88 link "
        "the committed bed omits (it blends MSL-referenced ETOPO as if it were NAVD88)", worst);

    // ---- THE DATUM SENSITIVITY PROBE. The MSL -> NAVD88 link is a real published number but a
    // regional one, so rather than argue about its accuracy, measure what it can move. The same
    // stack is composed again with the MSL-referenced layers displaced a whole metre; the answer
    // is how many metres of the finished bed follow. Where CUDEM and the edits paint at full
    // weight it is zero, and the question is settled THERE by measurement rather than by essay.
    if (!l0page.empty()) {
        auto probed = BuildHeightStack(comp, heightChannel, mslToNavd88M, datumProv, 1.0);
        DomainCompositor pc;
        pc.SetLadder(lad);
        pc.SetBlend(DomainCompositor::Blend::LayeredOver);
        for (auto& l : probed) pc.Add(l);
        std::vector<float> page, cov;
        pc.ComposePage(PageAddr{0, 0, 0}, pageGeo(nx, ny), nx, ny, 1, page, cov);
        double maxInf = 0.0, sumInf = 0.0;
        size_t n = 0;
        for (size_t i = 0; i < page.size() && i < l0page.size(); ++i) {
            if (cov[i] <= 0.0f) continue;
            const double inf = std::abs(double(page[i]) - double(l0page[i]));
            maxInf = (std::max)(maxInf, inf);
            sumInf += inf;
            ++n;
        }
        Log("[bed] datum sensitivity: 1.000 m of MSL error moves the bed by at most %.4f m "
            "(mean %.4f m) -- the %.3f m link can shift this bed by <= %.4f m",
            maxInf, n ? sumInf / double(n) : 0.0, mslToNavd88M,
            std::abs(mslToNavd88M) * maxInf);
    }
    return worst;
}
}  // namespace ga
