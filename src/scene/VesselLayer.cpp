#include "scene/VesselLayer.h"

#include "hal/PixEvents.h"
#include "hal/Shader.h"
#include "sim/Vessel.h"

#include <algorithm>

namespace ga {

void VesselLayer::Init(Gpu& gpu, ShaderCompiler& sc, FieldSet&, ID3D12RootSignature* rootSig) {
    m_rootSig = rootSig;
    if (!BuildPso(gpu, sc)) throw std::runtime_error("vessel PSO failed");
}

bool VesselLayer::BuildPso(Gpu& gpu, ShaderCompiler& sc) {
    const std::wstring path = m_shaderDir + L"/Vessel.hlsl";
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
    d.RasterizerState.CullMode = D3D12_CULL_MODE_BACK;
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

void VesselLayer::ReloadShaders(Gpu& gpu, ShaderCompiler& sc) {
    Com<ID3D12PipelineState> keep = m_pso;
    if (!BuildPso(gpu, sc)) m_pso = keep;
}

namespace {

// One box, in the parent's frame: centre and half-extents, composed onto the hull's pose.
void Emit(std::vector<VesselLayer::PartGpu>& out, const Motor& hull, const Motor& mount,
          double cx, double cy, double cz, double hx, double hy, double hz, int palette);

}  // namespace

void VesselLayer::SetVessels(const Vessel* const* vessels, int count) {
    m_parts.clear();
    if (!vessels) return;
    for (int i = 0; i < count; ++i) {
        const Vessel* v = vessels[i];
        if (!v) continue;
        const Motor hull = v->Body().pose;
        const VesselSpec& spec = v->Spec();

        for (const Element& e : spec.elements) {
            const int pal = static_cast<int>(e.kind);
            switch (e.kind) {
                case ElementKind::Buoyancy: {
                    // One box per adjacent station PAIR, sized by that pair's own outline. A
                    // single box round the whole hull would hide exactly what the stations are
                    // for -- that the shape changes along the length -- and for a deep-V the
                    // forefoot and the transom are not remotely the same section.
                    for (size_t s = 0; s + 1 < e.stations.size(); ++s) {
                        const Section& a = e.stations[s];
                        const Section& b = e.stations[s + 1];
                        double bx = 0.0, y0 = 1e30, y1 = -1e30;
                        for (const Section* q : {&a, &b}) {
                            for (size_t p = 0; p < q->ox.size(); ++p) {
                                bx = (std::max)(bx, std::abs(q->ox[p]));
                                y0 = (std::min)(y0, q->oy[p]);
                                y1 = (std::max)(y1, q->oy[p]);
                            }
                        }
                        if (!(bx > 0.0) || y1 <= y0) continue;
                        Emit(m_parts, hull, e.mount.at, 0.0, 0.5 * (y0 + y1),
                             0.5 * (a.z + b.z), bx, 0.5 * (y1 - y0),
                             0.5 * std::abs(b.z - a.z), pal);
                    }
                    break;
                }
                case ElementKind::Collar: {
                    const double z0 = e.tubeZ0.v, z1 = e.tubeZ1.v;
                    const double r = 0.5 * (e.tubeR0.v + e.tubeR1.v);
                    if (!(r > 0.0)) break;
                    Emit(m_parts, hull, e.mount.at, e.tubeXOffset.v, e.tubeYOffset.v,
                         0.5 * (z0 + z1), r, r, 0.5 * std::abs(z1 - z0), pal);
                    break;
                }
                case ElementKind::Thruster: {
                    // Drawn at its MOUNT and at rest: the articulation (steer, tilt) lives in
                    // the element's joint lines and the layer is not told the current angles
                    // yet, so a steered outboard will draw straight until it is. Called out
                    // because a wrong-looking engine should read as a missing wire, not physics.
                    const double r = (e.propRadius.v > 0.0) ? e.propRadius.v : 0.15;
                    Emit(m_parts, hull, e.mount.at, 0.0, 0.0, 0.0, r, 2.0 * r, 0.9 * r, pal);
                    break;
                }
                case ElementKind::Foil: {
                    const double area = (e.foilArea.v > 0.0) ? e.foilArea.v : 0.1;
                    const double h = std::sqrt(area);
                    Emit(m_parts, hull, e.mount.at, 0.0, -0.5 * h, 0.0, 0.02, 0.5 * h,
                         0.5 * h, pal);
                    break;
                }
                case ElementKind::Ballast: {
                    // Drawn small and deliberately visible. Where a boat carries its weight is
                    // the difference between a hull that planes at 4 degrees and one that runs
                    // bow-high at 12, and being able to SEE that the tanks are forward is worth
                    // the six triangles.
                    const double m = (e.mass.v > 0.0) ? e.mass.v : 1.0;
                    const double h = 0.5 * std::cbrt(m / 700.0);   // ~size by mass, gently
                    Emit(m_parts, hull, e.mount.at, 0.0, 0.0, 0.0, h, h, h, pal);
                    break;
                }
                // Planing and Drag are LAWS over surfaces the other elements already draw --
                // a running surface and a windage area are not solids, and boxing them would
                // double-draw the hull.
                default: break;
            }
        }
    }
}

namespace {

void Emit(std::vector<VesselLayer::PartGpu>& out, const Motor& hull, const Motor& mount,
          double cx, double cy, double cz, double hx, double hy, double hz, int palette) {
    const Motor world = hull * mount * Motor::Translation(cx, cy, cz);
    VesselLayer::PartGpu p{};
    double re[4], du[4];
    world.Real(re);
    world.Dual(du);
    for (int i = 0; i < 4; ++i) {
        p.re[i] = static_cast<float>(re[i]);
        p.du[i] = static_cast<float>(du[i]);
    }
    p.half[0] = static_cast<float>((std::max)(hx, 0.01));
    p.half[1] = static_cast<float>((std::max)(hy, 0.01));
    p.half[2] = static_cast<float>((std::max)(hz, 0.01));
    p.half[3] = static_cast<float>(palette);
    out.push_back(p);
}

}  // namespace

void VesselLayer::Render(const FrameContext& ctx) {
    if (!m_pso || m_parts.empty()) return;
    PixScope scope(ctx.cl, "vessels (spec -> boxes, motor sandwich on the GPU)");
    const float cb[4] = {static_cast<float>(m_parts.size()), 1.0f, 0.0f, 0.0f};
    ctx.cl->SetPipelineState(m_pso.Get());
    ctx.cl->IASetPrimitiveTopology(D3D_PRIMITIVE_TOPOLOGY_TRIANGLELIST);
    ctx.cl->SetGraphicsRootConstantBufferView(1, ctx.gpu->PushConstants(cb, sizeof(cb)));
    ctx.cl->SetGraphicsRootShaderResourceView(
        2, ctx.gpu->PushConstants(m_parts.data(), m_parts.size() * sizeof(PartGpu)));
    ctx.cl->DrawInstanced(static_cast<uint32_t>(m_parts.size()) * 36u, 1, 0, 0);
}

}  // namespace ga
