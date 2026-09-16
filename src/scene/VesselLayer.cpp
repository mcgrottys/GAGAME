#include "scene/VesselLayer.h"

#include "hal/PixEvents.h"
#include "hal/Pipeline.h"
#include "hal/Shader.h"
#include "sim/Vessel.h"

#include <algorithm>
#include "render/Camera.h"

namespace ga {

void VesselLayer::Init(Gpu& gpu, ShaderCompiler& sc, FieldSet&, hal::RootSignature rootSig) {
    m_rootSig = rootSig;
    if (!BuildPso(gpu, sc)) throw std::runtime_error("vessel PSO failed");
}

bool VesselLayer::BuildPso(Gpu& gpu, ShaderCompiler& sc) {
    const std::wstring path = m_shaderDir + L"/Vessel.hlsl";
    hal::GraphicsPipelineDesc d;
    d.rootSig = m_rootSig;
    d.vs = sc.Compile(path, L"VsMain", L"vs_6_0");
    d.ps = sc.Compile(path, L"PsMain", L"ps_6_0");
    d.cull = D3D12_CULL_MODE_BACK;
    d.depthClip = TRUE;
    d.depthTest = true;
    d.depthWrite = true;   // reversed-Z GREATER, the default comparison
    return hal::Reload(m_pso, [&] { return hal::BuildGraphics(gpu, d, "vessel"); }, "vessel");
}

void VesselLayer::ReloadShaders(Gpu& gpu, ShaderCompiler& sc) {
    BuildPso(gpu, sc);   // the reload law lives in BuildPso: swap only on success
}

namespace {

// One box, in the parent's frame: centre and half-extents, composed onto the hull's pose.
void Emit(std::vector<VesselLayer::PartCpu>& out, const Motor& hull, const Motor& mount,
          double cx, double cy, double cz, double hx, double hy, double hz, int palette);

}  // namespace

void VesselLayer::SetVessels(const Vessel* const* vessels, int count) {
    SetVessels(vessels, nullptr, nullptr, count);
}

void VesselLayer::SetVessels(const Vessel* const* vessels, const Motor* frames, int count) {
    SetVessels(vessels, frames, nullptr, count);
}

void VesselLayer::SetVessels(const Vessel* const* vessels, const Motor* frames,
                             const uint8_t* through, int count) {
    m_cpu.clear();
    if (!vessels) return;
    for (int i = 0; i < count; ++i) {
        const Vessel* v = vessels[i];
        if (!v) continue;
        const size_t first = m_cpu.size();
        const Motor hull = frames ? frames[i] * v->Body().pose : v->Body().pose;
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
                        Emit(m_cpu, hull, e.mount.at, 0.0, 0.5 * (y0 + y1),
                             0.5 * (a.z + b.z), bx, 0.5 * (y1 - y0),
                             0.5 * std::abs(b.z - a.z), pal);
                    }
                    break;
                }
                case ElementKind::Collar: {
                    const double z0 = e.tubeZ0.v, z1 = e.tubeZ1.v;
                    const double r = 0.5 * (e.tubeR0.v + e.tubeR1.v);
                    if (!(r > 0.0)) break;
                    Emit(m_cpu, hull, e.mount.at, e.tubeXOffset.v, e.tubeYOffset.v,
                         0.5 * (z0 + z1), r, r, 0.5 * std::abs(z1 - z0), pal);
                    break;
                }
                case ElementKind::Thruster: {
                    // Drawn at its MOUNT and at rest: the articulation (steer, tilt) lives in
                    // the element's joint lines and the layer is not told the current angles
                    // yet, so a steered outboard will draw straight until it is. Called out
                    // because a wrong-looking engine should read as a missing wire, not physics.
                    const double r = (e.propRadius.v > 0.0) ? e.propRadius.v : 0.15;
                    Emit(m_cpu, hull, e.mount.at, 0.0, 0.0, 0.0, r, 2.0 * r, 0.9 * r, pal);
                    break;
                }
                case ElementKind::Foil: {
                    const double area = (e.foilArea.v > 0.0) ? e.foilArea.v : 0.1;
                    const double h = std::sqrt(area);
                    Emit(m_cpu, hull, e.mount.at, 0.0, -0.5 * h, 0.0, 0.02, 0.5 * h,
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
                    Emit(m_cpu, hull, e.mount.at, 0.0, 0.0, 0.0, h, h, h, pal);
                    break;
                }
                // Planing and Drag are LAWS over surfaces the other elements already draw --
                // a running surface and a windage area are not solids, and boxing them would
                // double-draw the hull.
                default: break;
            }
        }
        // Every box this hull emitted belongs to the same side of the window as the hull does.
        if (through && through[i]) {
            for (size_t k = first; k < m_cpu.size(); ++k) m_cpu[k].through = 1.0f;
        }
    }
}

namespace {

void Emit(std::vector<VesselLayer::PartCpu>& out, const Motor& hull, const Motor& mount,
          double cx, double cy, double cz, double hx, double hy, double hz, int palette) {
    VesselLayer::PartCpu p{};
    p.world = hull * mount * Motor::Translation(cx, cy, cz);
    p.half[0] = static_cast<float>((std::max)(hx, 0.01));
    p.half[1] = static_cast<float>((std::max)(hy, 0.01));
    p.half[2] = static_cast<float>((std::max)(hz, 0.01));
    p.half[3] = static_cast<float>(palette);
    out.push_back(p);
}

}  // namespace

bool VesselLayer::Occupies(double x, double y, double z, double margin) const {
    for (const PartCpu& p : m_cpu) {
        double lx = x, ly = y, lz = z;
        p.world.Inverse().TransformPoint(lx, ly, lz);   // the point in the box's own frame
        if (std::abs(lx) <= p.half[0] + margin && std::abs(ly) <= p.half[1] + margin &&
            std::abs(lz) <= p.half[2] + margin) {
            return true;
        }
    }
    return false;
}

void VesselLayer::Render(const FrameContext& ctx) {
    if (!m_pso || m_cpu.empty()) return;
    // THE BOUNDARY: each box's motor relative to THIS view's eye, composed in doubles, then float.
    // (The shader no longer subtracts the eye -- it receives the eye-relative position directly.)
    const Motor toEye = ctx.camera ? Motor::Translation(-ctx.camera->px, -ctx.camera->py, -ctx.camera->pz)
                                   : Motor::Identity();
    m_parts.resize(m_cpu.size());
    for (size_t k = 0; k < m_cpu.size(); ++k) {
        const Motor rel = toEye * m_cpu[k].world;
        double re[4], du[4];
        rel.Real(re);
        rel.Dual(du);
        for (int i = 0; i < 4; ++i) {
            m_parts[k].re[i] = static_cast<float>(re[i]);
            m_parts[k].du[i] = static_cast<float>(du[i]);
            m_parts[k].half[i] = m_cpu[k].half[i];
            m_parts[k].opt[i] = 0.0f;
        }
        m_parts[k].opt[0] = m_cpu[k].through;
    }
    PixScope scope(ctx.cmd->Native(), "vessels (spec -> boxes, motor sandwich on the GPU)");
    // The constants: the part count and brightness as before, then the gate's window (M13) --
    // the box's rows with its half extents in w, its eye-relative centre, and the one flag the
    // shader tests. Appended at the END, mirroring Vessel.hlsl's cbuffer (priors 22).
    struct {
        float params[4];
        float r0[4], r1[4], r2[4], c[4];
    } cb{};
    cb.params[0] = static_cast<float>(m_parts.size());
    cb.params[1] = 1.0f;
    float* rows[3] = {cb.r0, cb.r1, cb.r2};
    for (int r = 0; r < 3; ++r) {
        for (int i = 0; i < 3; ++i) rows[r][i] = m_winRows[r * 3 + i];
        rows[r][3] = m_winHalf[r];
    }
    for (int i = 0; i < 3; ++i) cb.c[i] = m_winC[i];
    cb.c[3] = m_winOn ? 1.0f : 0.0f;
    ctx.cmd->Pipeline(m_pso.Get());
    ctx.cmd->Topology(D3D_PRIMITIVE_TOPOLOGY_TRIANGLELIST);
    ctx.cmd->GraphicsConstants(1, cb);
    ctx.cmd->GraphicsSrvAt(
        2, ctx.gpu->PushConstants(m_parts.data(), m_parts.size() * sizeof(PartGpu)));
    ctx.cmd->Draw(static_cast<uint32_t>(m_parts.size()) * 36u, 1, 0, 0);
}

}  // namespace ga
