#include "scene/VesselLayer.h"

#include "hal/PixEvents.h"
#include "hal/Pipeline.h"
#include "hal/Shader.h"
#include "scene/MeshFile.h"
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
    const bool boxes = hal::Reload(m_pso, [&] { return hal::BuildGraphics(gpu, d, "vessel"); }, "vessel");
    // The mesh pass: the same state, the same pixel shader, the file's vertices (VsMesh). A glb's
    // front faces wind counter-clockwise about the outward normal, which under this chain (the
    // box table's own note) is clockwise on screen: D3D's front, so the same cull.
    hal::GraphicsPipelineDesc m = d;
    m.vs = sc.Compile(path, L"VsMesh", L"vs_6_0");
    const bool mesh = hal::Reload(m_meshPso, [&] { return hal::BuildGraphics(gpu, m, "vessel.mesh"); }, "vessel.mesh");
    return boxes && mesh;
}

void VesselLayer::DeclareMesh(const Vessel* v, const std::string& file, double scale, double yawDeg) {
    if (!v) return;
    if (file.empty()) {   // the form has no model: the spec's boxes
        m_meshDecl.erase(v);
        return;
    }
    auto it = m_meshDecl.find(v);
    if (it != m_meshDecl.end() && it->second.file == file && it->second.scale == scale && it->second.yawDeg == yawDeg) {
        return;   // declared every publish; nothing changed
    }
    MeshDecl d;
    d.file = file;
    d.scale = scale;
    d.yawDeg = yawDeg;
    m_meshDecl[v] = d;
    Log("[vessels] '%s' draws %s (scale %.4g, yaw %.1f deg) over its physics; its wheels stay the struts' own",
        v->Spec().kind.c_str(), file.c_str(), scale, yawDeg);
}

void VesselLayer::UploadMesh(Gpu& gpu, const MeshDecl& d, MeshGpu& g) {
    g.tried = true;
    MeshData data;
    std::string why;
    // Left out of the picture: the model's own tyres, rims and brake discs (the physics draws its
    // wheels where the struts put them) and its glass (no transparency pass: an opaque window
    // would hide the interior the model has). Both declared here, counted by the reader.
    const std::vector<std::string> skips = {"tire", "rim", "brakedisk"};
    if (!LoadGlbMesh(d.file, d.scale, d.yawDeg, skips, 0.6, data, &why)) {
        Log("[vessels] mesh %s NOT drawn: %s (the spec's boxes stand in)", d.file.c_str(), why.c_str());
        return;
    }
    g.vb = gpu.CreateDefaultBuffer(data.verts.data(), data.verts.size() * sizeof(uint32_t), L"vessel.mesh.vb");
    g.ib = gpu.CreateDefaultBuffer(data.idx.data(), data.idx.size() * sizeof(uint32_t), L"vessel.mesh.ib");
    if (!g.vb.Valid() || !g.ib.Valid()) {
        Log("[vessels] mesh %s NOT drawn: the buffers could not be made", d.file.c_str());
        return;
    }
    g.vbSlot = gpu.CreateStructuredBufferSrv(g.vb.res.Get(), static_cast<uint32_t>(data.verts.size()), 4);
    g.ibSlot = gpu.CreateStructuredBufferSrv(g.ib.res.Get(), static_cast<uint32_t>(data.idx.size()), 4);
    g.indexCount = static_cast<uint32_t>(data.idx.size());
    Log("[vessels] mesh %s on the GPU: %zu vertices (%.1f MB), %zu triangles (%.1f MB), slots %u / %u",
        d.file.c_str(), data.VertexCount(), data.verts.size() * 4.0 / 1048576.0, data.TriangleCount(),
        data.idx.size() * 4.0 / 1048576.0, g.vbSlot, g.ibSlot);
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
                             const uint8_t* depth, int count) {
    m_cpu.clear();
    m_meshDraws.clear();
    if (!vessels) return;
    for (int i = 0; i < count; ++i) {
        const Vessel* v = vessels[i];
        if (!v) continue;
        const size_t first = m_cpu.size();
        const Motor hull = frames ? frames[i] * v->Body().pose : v->Body().pose;
        const VesselSpec& spec = v->Spec();
        size_t wheelIdx = 0;
        // The reference model, when the scene declared one: the file's frame is the spec's datum,
        // and Build moved the datum onto the CG (cgFromTransom), so datum -> body is that shift.
        const auto meshIt = m_meshDecl.find(v);
        const bool hasMesh = meshIt != m_meshDecl.end();
        if (hasMesh) {
            MeshDraw d;
            d.world = hull * Motor::Translation(-spec.cgFromTransom[0], -spec.cgFromTransom[1], -spec.cgFromTransom[2]);
            d.depth = depth ? static_cast<float>(depth[i]) : 0.0f;
            d.decl = &meshIt->second;
            m_meshDraws.push_back(d);
        }

        for (size_t ei = 0; ei < spec.elements.size(); ++ei) {
            const Element& e = spec.elements[ei];
            // A part its form does not engage is not drawn (its wheel slot still counts).
            if (!(v->Engage(ei) > 0.0)) {
                if (e.kind == ElementKind::Wheel) ++wheelIdx;
                continue;
            }
            const int pal = static_cast<int>(e.kind);
            switch (e.kind) {
                case ElementKind::Wheel: {
                    // THE WHEEL WHERE THE STRUT PUT IT: the live centre and steer from the physics
                    // (Vessel::Wheels), not the mount at rest -- a tyre is drawn compressed, drooped
                    // or in the air as it is, and turned as it is. A slab the tyre's radius and
                    // width, turned about the kingpin by the steer.
                    const auto& wheels = v->Wheels();
                    if (wheelIdx >= wheels.size()) break;
                    const Vessel::WheelState& ws = wheels[wheelIdx];
                    const double R = (e.wheelRadius.v > 0.0) ? e.wheelRadius.v : 0.3;
                    const double o[3] = {0.0, 0.0, 0.0}, upY[3] = {0.0, 1.0, 0.0};
                    const Motor at = Motor::Translation(ws.centre[0], ws.centre[1], ws.centre[2]) *
                                     Motor::Rotation(o, upY, ws.steer);
                    Emit(m_cpu, hull, at, 0.0, 0.0, 0.0, 0.33 * R, R, R, pal);
                    if (wheelIdx == 0 && !hasMesh) {
                        // The body over the wheels when no mesh is declared (a box the spec's
                        // length and beam, standing on the strut tops): a car with no art.
                        double top[3];
                        e.mount.Origin(0.0, top);
                        const double L = (spec.loa.v > 0.0) ? spec.loa.v : 4.0;
                        const double B = (spec.beam.v > 0.0) ? spec.beam.v : 1.8;
                        Emit(m_cpu, hull, Motor::Identity(), 0.0, top[1] + 0.45, 0.0, 0.5 * B, 0.45,
                             0.5 * L, static_cast<int>(ElementKind::Buoyancy));
                    }
                    ++wheelIdx;
                    break;
                }
                case ElementKind::Shell:
                    if (hasMesh) break;   // the model is the body
                    [[fallthrough]];
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
                case ElementKind::Hover: {
                    // The drone: a body box and four rotor discs on the frame's corners, sized
                    // from the spec's length and beam. No art, like everything else here.
                    const double L = (spec.loa.v > 0.0) ? spec.loa.v : 0.6;
                    const double B = (spec.beam.v > 0.0) ? spec.beam.v : 0.6;
                    Emit(m_cpu, hull, e.mount.at, 0.0, 0.0, 0.0, 0.18 * B, 0.05, 0.18 * L, pal);
                    for (int sx = -1; sx <= 1; sx += 2) {
                        for (int sz = -1; sz <= 1; sz += 2) {
                            Emit(m_cpu, hull, e.mount.at, sx * 0.4 * B, 0.04, sz * 0.4 * L,
                                 0.17 * B, 0.008, 0.17 * L, pal);
                        }
                    }
                    break;
                }
                // Planing and Drag are LAWS over surfaces the other elements already draw --
                // a running surface and a windage area are not solids, and boxing them would
                // double-draw the hull.
                default: break;
            }
        }
        // Every box this hull emitted belongs to the same world of the window chain as the hull.
        if (depth && depth[i]) {
            for (size_t k = first; k < m_cpu.size(); ++k) m_cpu[k].depth = float(depth[i]);
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
        if (p.depth != 0.0f) continue;   // a hull seen through a window does not stand here
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
    // ANOTHER EYE (the minimap's): the hulls' depths are the first eye's chain, so it draws the
    // hulls that stand in its own world -- depth 0, each in the place it really is -- kept by its
    // own chain of windows (its world table).
    const bool firstEye = ctx.viewIndex == 0;
    m_parts.clear();
    m_parts.reserve(m_cpu.size());
    for (size_t k = 0; k < m_cpu.size(); ++k) {
        if (!firstEye && m_cpu[k].depth != 0.0f) continue;
        m_parts.emplace_back();
        const size_t n = m_parts.size() - 1;
        const Motor rel = toEye * m_cpu[k].world;
        double re[4], du[4];
        rel.Real(re);
        rel.Dual(du);
        for (int i = 0; i < 4; ++i) {
            m_parts[n].re[i] = static_cast<float>(re[i]);
            m_parts[n].du[i] = static_cast<float>(du[i]);
            m_parts[n].half[i] = m_cpu[k].half[i];
            m_parts[n].opt[i] = 0.0f;
        }
        m_parts[n].opt[0] = m_cpu[k].depth;
    }
    // The constants: the part count and brightness (the view's chain of windows and the light in
    // each world are its world table's), and the mesh pass's slots. Mirrors Vessel.hlsl's cbuffer
    // (priors 22).
    struct {
        float params[4];
        uint32_t mesh[4];
    } cb{};
    cb.params[0] = static_cast<float>(m_parts.size());
    cb.params[1] = 1.0f;
    if (!m_parts.empty()) {
        PixScope scope(ctx.cmd->Native(), "vessels (spec -> boxes, motor sandwich on the GPU)");
        ctx.cmd->Pipeline(m_pso.Get());
        ctx.cmd->Topology(D3D_PRIMITIVE_TOPOLOGY_TRIANGLELIST);
        ctx.cmd->GraphicsConstants(1, cb);
        ctx.cmd->GraphicsSrvAt(
            2, ctx.gpu->PushConstants(m_parts.data(), m_parts.size() * sizeof(PartGpu)));
        ctx.cmd->Draw(static_cast<uint32_t>(m_parts.size()) * 36u, 1, 0, 0);
    }
    // THE REFERENCE MODELS: one draw each, the hull's motor eye-relative in doubles exactly as a
    // part's, the file's vertices through the bindless uints (VsMesh), the same PsMain -- lit where
    // it stands and cut by the view's windows like every box.
    if (m_meshDraws.empty() || !m_meshPso || !ctx.gpu) return;
    PixScope meshScope(ctx.cmd->Native(), "vessels (reference models)");
    for (const MeshDraw& d : m_meshDraws) {
        if (!firstEye && d.depth != 0.0f) continue;
        MeshGpu& g = m_meshes[d.decl->file];
        if (!g.tried) UploadMesh(*ctx.gpu, *d.decl, g);
        if (g.indexCount == 0) continue;
        PartGpu part{};
        const Motor rel = toEye * d.world;
        double re[4], du[4];
        rel.Real(re);
        rel.Dual(du);
        for (int i = 0; i < 4; ++i) {
            part.re[i] = static_cast<float>(re[i]);
            part.du[i] = static_cast<float>(du[i]);
        }
        part.opt[0] = d.depth;
        cb.mesh[0] = g.vbSlot;
        cb.mesh[1] = g.ibSlot;
        cb.mesh[2] = g.indexCount;
        ctx.cmd->Pipeline(m_meshPso.Get());
        ctx.cmd->Topology(D3D_PRIMITIVE_TOPOLOGY_TRIANGLELIST);
        ctx.cmd->GraphicsConstants(1, cb);
        ctx.cmd->GraphicsSrvAt(2, ctx.gpu->PushConstants(&part, sizeof(part)));
        ctx.cmd->Draw(g.indexCount, 1, 0, 0);
    }
}

}  // namespace ga
