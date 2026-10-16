#include "scene/BuildingLayer.h"
#include "compose/RasterFileSource.h"

#include "compose/BuildingShape.h"

#include "compose/BuildingMoments.h"

#include "core/Common.h"
#include "core/ThreadManager.h"
#include "hal/GpuProfiler.h"
#include "hal/PixEvents.h"
#include "hal/Pipeline.h"
#include "hal/Shader.h"
#include "hal/Views.h"
#include "core/Image.h"
#include "render/Renderer.h"   // SceneConstants: the sun pass's b0 carries the sun's own cast
#include <DirectXMath.h>
#include "render/Camera.h"

#include <algorithm>
#include <thread>
#include <array>
#include <cmath>
#include <chrono>
#include <cstring>
#include <map>
#include <thread>

namespace ga {

BuildingLayer::~BuildingLayer() {
    // A job holds the stack and the ground's sources: none may outlive the layer's owner. Ask them
    // to stop and wait for the ones already running (a cell is tens of milliseconds).
    if (m_stands) {
        Log("[buildings] at the end: %llu stands made, %.1f ms each on the pool; the stands' stream took %.3f ms a frame on the frame thread",
            static_cast<unsigned long long>(m_stands), m_standMs / double(m_stands), m_frame ? m_streamMs / double(m_frame) : 0.0);
    }
    for (uint32_t k = 0; k < kEyes * kWorlds; ++k) {   // the instrument: what each walker's last walk drew
        const Eye& e = m_eyes[k];
        if (e.walks) {
            Log("[buildings] eye %u world %u at the end: %llu walks, the last %u nodes named (%u records) from %zu pages, the GPU drew %u boxes + %u shapes + %u points of the last it counted, %llu nodes hidden by what the frame drew", k / kWorlds, k % kWorlds,
                static_cast<unsigned long long>(e.walks), e.drawn.named, e.drawn.cap, e.drawnUsed.size(), e.lastDrawn[0], e.lastDrawn[1], e.lastDrawn[2], static_cast<unsigned long long>(m_hidden[k]));
        }
    }
    m_shared->cancel = true;
    for (int i = 0; i < 2000 && (m_shared->inflight.load() > 0 || m_shared->pageInflight.load() > 0 || [this] { for (const auto& w : m_shared->walking) if (w.load() > 0) return true; return m_shared->standInflight.load() > 0; }()); ++i) {
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
}

void BuildingLayer::Configure(const std::wstring& shaderDir, std::shared_ptr<const BuildingStack> stack,
                              double radiusM, Place place, Locate locate, Ground ground) {
    m_shaderDir = shaderDir;
    m_stack = std::move(stack);
    m_radius = radiusM;
    m_place = std::move(place);
    m_locate = std::move(locate);
    m_ground = std::move(ground);
}

void BuildingLayer::ConfigureFar(std::shared_ptr<const BuildingLodFile> lod, double pixels, double shapePixels,
                                 double pageMB, double marginDeg, double boxPixels, double standMB) {
    m_marginDeg = marginDeg;
    m_stoodRecords = static_cast<uint32_t>((std::max)(standMB, 1.0) * 1048576.0 / sizeof(StoodRec));
    m_shapePixels = (std::max)(shapePixels, 0.0);
    m_boxPixels = (std::max)(boxPixels, 0.0);
    m_pageBudget = static_cast<uint64_t>((std::max)(pageMB, 1.0) * 1048576.0);
    m_lod = std::move(lod);
    m_lodPixels = pixels > 0.0 ? pixels : 1.0;
}

void BuildingLayer::Init(Gpu& gpu, ShaderCompiler& sc, FieldSet&, hal::RootSignature rootSig) {
    m_rootSig = rootSig;
    m_gpu = &gpu;
    // THE CLASSIFY PASS: its constants, the walk's node list, the stands' pool, the walk's buffer.
    m_clsRs = hal::RootLayout().Cbv(0).Srv(0).Srv(1).Uav(0).Build(gpu, "buildings.classify");
    if (!BuildPso(gpu, sc)) throw std::runtime_error("building PSO failed");
    gpu.onLostDispatch = [this](uint32_t k) { DumpChunk(k); };
    if (m_lod && m_stoodRecords) {
        m_stood = gpu.CreateUavBuffer(uint64_t(m_stoodRecords) * sizeof(StoodRec), L"buildings.stood");
        m_stoodFree.Reset(m_stoodRecords);
        m_sigMesh = gpu.CreateCommandSignature(D3D12_INDIRECT_ARGUMENT_TYPE_DISPATCH_MESH, 16);
        m_sigDraw = gpu.CreateCommandSignature(D3D12_INDIRECT_ARGUMENT_TYPE_DRAW, 16);
        m_countsRb = gpu.CreateReadbackBuffer(uint64_t(kCountsRing) * 16u, L"buildings.counts");
        Log("[buildings] the stands' pool: %u records of %zu bytes (%.0f MB) on the GPU; the walk names nodes, the GPU decides each record's form",
            m_stoodRecords, sizeof(StoodRec), double(m_stoodRecords) * sizeof(StoodRec) / 1048576.0);
    }
    // THE SUN MAP: a depth kSunTexels a side, cleared to 0 (nothing: the far face of the cast) and
    // tested GREATER as the frame's depth is, so it keeps what stands nearest the sun.
    D3D12_CLEAR_VALUE dcv{};
    dcv.Format = DXGI_FORMAT_D32_FLOAT;
    dcv.DepthStencil.Depth = 0.0f;
    m_sunTex = gpu.CreateTexture2D(kSunTexels, kSunTexels, DXGI_FORMAT_D32_FLOAT, D3D12_RESOURCE_FLAG_ALLOW_DEPTH_STENCIL,
                                   D3D12_RESOURCE_STATE_DEPTH_WRITE, L"buildings.sunmap", &dcv);
    m_sunDsv = hal::Dsv(gpu, m_sunTex.res.Get(), DXGI_FORMAT_D32_FLOAT);
    m_sunSrv = gpu.CreateSrv(m_sunTex.res.Get(), DXGI_FORMAT_R32_FLOAT);
}

bool BuildingLayer::BuildPso(Gpu& gpu, ShaderCompiler& sc) {
    const std::wstring path = m_shaderDir + L"/Buildings.hlsl";
    hal::GraphicsPipelineDesc d;
    d.rootSig = m_rootSig;
    d.vs = sc.Compile(path, L"VsMain", L"vs_6_0");
    d.ps = sc.Compile(path, L"PsMain", L"ps_6_0");
    d.cull = D3D12_CULL_MODE_NONE;
    d.depthClip = TRUE;
    d.depthTest = true;
    d.depthWrite = true;   // reversed-Z GREATER, the default comparison
    const bool ok = hal::Reload(m_pso, [&] { return hal::BuildGraphics(gpu, d, "buildings"); }, "buildings");
    d.vs = sc.Compile(path, L"VsBox", L"vs_6_0");   // the tree's boxes: the same light, the same depth,
    // THE OUTSIDE IS THE FRONT (2026-10-10): every surface of the tree winds counter-clockwise seen
    // from outside -- the records' rings (BuildingShape.h), their walls to the edge's right, the
    // roof's ears, a ribbon's quads, a box's six faces -- so the back faces are the insides and the
    // undersides, which nothing sees, and the rasterizer drops them before a pixel is shaded.
    d.cull = D3D12_CULL_MODE_BACK;
    d.frontCcw = TRUE;
    // OPAQUE (the owner, 2026-10-15): a box fading in, or faded by its fold's coverage, was a
    // transparent block over the imagery; a form is drawn whole or not at all.
    const bool okBox = hal::Reload(m_psoBox, [&] { return hal::BuildGraphics(gpu, d, "buildings.boxes"); }, "buildings.boxes");
    // THE POINTS: one vertex a building, the smallest of the three forms (VsPoint), opaque, no cull.
    hal::GraphicsPipelineDesc pd = d;
    pd.vs = sc.Compile(path, L"VsPoint", L"vs_6_0");
    pd.cull = D3D12_CULL_MODE_NONE;
    pd.topology = D3D12_PRIMITIVE_TOPOLOGY_TYPE_POINT;
    hal::Reload(m_psoPoint, [&] { return hal::BuildGraphics(gpu, pd, "buildings.points"); }, "buildings.points");
    // THE SUN'S PIPELINES (RenderSunMap): the same stages with no pixel stage and no colour target
    // -- the depth the sun sees is the whole picture -- and the same cull as the frame's, so the
    // solid the sun meets is the solid the eye sees, winding and all, and the map and the drawn
    // faces agree about what is a surface (a face the sun lights is in the map at its own depth;
    // the receiver's plane absorbs that in SunMapClear).
    hal::GraphicsPipelineDesc sd = d;
    sd.ps = ShaderBlob{};
    sd.numRenderTargets = 0;
    sd.rtvFormat = DXGI_FORMAT_UNKNOWN;
    hal::Reload(m_psoSunBox, [&] { return hal::BuildGraphics(gpu, sd, "buildings.sun.boxes"); }, "buildings.sun.boxes");
    sd.vs = sc.Compile(path, L"VsMain", L"vs_6_0");
    sd.cull = D3D12_CULL_MODE_NONE;
    sd.frontCcw = FALSE;
    hal::Reload(m_psoSun, [&] { return hal::BuildGraphics(gpu, sd, "buildings.sun.cells"); }, "buildings.sun.cells");
    // THE SHAPES' PIPELINE: MsShape extrudes a building's own footprint (needs mesh shaders; without
    // them the walk draws boxes, as before).
    D3D12_FEATURE_DATA_D3D12_OPTIONS7 o7{};
    if (SUCCEEDED(gpu.Device()->CheckFeatureSupport(D3D12_FEATURE_D3D12_OPTIONS7, &o7, sizeof(o7))) &&
        o7.MeshShaderTier != D3D12_MESH_SHADER_TIER_NOT_SUPPORTED) {
        hal::MeshPipelineDesc md;
        md.rootSig = m_rootSig;
        md.as = sc.Compile(path, L"AsShape", L"as_6_5");   // the streams: 64 instances -> their edge and roof groups
        md.ms = sc.Compile(path, L"MsShape", L"ms_6_5");
        md.ps = sc.Compile(path, L"PsMain", L"ps_6_5");
        md.cull = D3D12_CULL_MODE_BACK;   // the outside is the front (above)
        md.frontCcw = TRUE;
        md.depthClip = TRUE;
        md.depthTest = true;
        md.depthWrite = true;
        md.blend = true;   // a ribbon thinner than a pixel carries its width as alpha (MsShape); the buildings are opaque
        md.srcBlend = D3D12_BLEND_SRC_ALPHA;
        md.dstBlend = D3D12_BLEND_INV_SRC_ALPHA;
        if (md.as.Valid() && md.ms.Valid() && md.ps.Valid()) {
            hal::Reload(m_psoShape, [&] { return hal::BuildMesh(gpu, md, "buildings.shapes"); }, "buildings.shapes");
            hal::MeshPipelineDesc sm = md;   // ...and the sun's: depth only (above)
            sm.ps = ShaderBlob{};
            sm.blend = false;
            sm.numRenderTargets = 0;
            sm.rtvFormat = DXGI_FORMAT_UNKNOWN;
            hal::Reload(m_psoSunShape, [&] { return hal::BuildMesh(gpu, sm, "buildings.sun.shapes"); }, "buildings.sun.shapes");
        }
        // The boxes by mesh shader too: 8 vertices and 12 triangles a box, 16 a group, no amplification.
        hal::MeshPipelineDesc mb = md;
        mb.as = ShaderBlob{};
        mb.blend = false;   // the boxes are opaque (above)
        mb.ms = sc.Compile(path, L"MsBox", L"ms_6_5");
        if (mb.ms.Valid() && mb.ps.Valid()) {
            hal::Reload(m_psoBoxMesh, [&] { return hal::BuildMesh(gpu, mb, "buildings.boxes.mesh"); }, "buildings.boxes.mesh");
            hal::MeshPipelineDesc sb = mb;
            sb.ps = ShaderBlob{};
            sb.blend = false;
            sb.numRenderTargets = 0;
            sb.rtvFormat = DXGI_FORMAT_UNKNOWN;
            hal::Reload(m_psoSunBoxMesh, [&] { return hal::BuildMesh(gpu, sb, "buildings.sun.boxes.mesh"); }, "buildings.sun.boxes.mesh");
        }
    }
    // THE CLASSIFY KERNEL (shaders/BuildingsClassify.hlsl): the per-building law, on the GPU.
    hal::Reload(m_clsPso, [&] {
        return hal::BuildCompute(gpu, m_clsRs.Get(), sc.Compile(m_shaderDir + L"/BuildingsClassify.hlsl", L"CsClassify", L"cs_6_0"),
                                 "buildings.classify");
    }, "buildings.classify");
    return okBox && ok;
}

void BuildingLayer::ReloadShaders(Gpu& gpu, ShaderCompiler& sc) {
    BuildPso(gpu, sc);   // the reload law lives in BuildPso: swap only on success
}

// ---- the stands' pool: free runs of records ---------------------------------------------------
void BuildingLayer::Ranges::Reset(uint32_t n) {
    free.clear();
    byLen.clear();
    total = n;
    used = 0;
    if (n) {
        free[0] = n;
        byLen.insert({n, 0});
    }
}
bool BuildingLayer::Ranges::Take(uint32_t n, uint32_t& off) {
    if (!n) return false;
    auto it = byLen.lower_bound(n);   // the smallest run that holds it
    if (it == byLen.end()) return false;
    const uint32_t len = it->first;
    off = it->second;
    byLen.erase(it);
    free.erase(off);
    if (len > n) {
        free[off + n] = len - n;
        byLen.insert({len - n, off + n});
    }
    used += n;
    return true;
}
void BuildingLayer::Ranges::Give(uint32_t off, uint32_t n) {
    if (!n) return;
    used -= (std::min)(used, n);
    auto drop = [&](uint32_t o, uint32_t l) {   // a run out of both maps
        free.erase(o);
        auto r = byLen.equal_range(l);
        for (auto it = r.first; it != r.second; ++it) {
            if (it->second == o) { byLen.erase(it); break; }
        }
    };
    auto next = free.find(off + n);   // the run after it
    if (next != free.end()) {
        const uint32_t l = next->second;
        drop(off + n, l);
        n += l;
    }
    auto after = free.lower_bound(off);   // the run before it ends where this begins
    if (after != free.begin()) {
        auto prev = std::prev(after);
        if (prev->first + prev->second == off) {
            const uint32_t o = prev->first, l = prev->second;
            drop(o, l);
            off = o;
            n += l;
        }
    }
    free[off] = n;
    byLen.insert({n, off});
}

namespace {

struct P2 {
    double x, y;
};

double Cross(const P2& o, const P2& a, const P2& b) {
    return (a.x - o.x) * (b.y - o.y) - (a.y - o.y) * (b.x - o.x);
}

double Area2(const std::vector<P2>& r) {
    double a = 0.0;
    for (size_t i = 0, j = r.size() - 1; i < r.size(); j = i++) a += r[j].x * r[i].y - r[i].x * r[j].y;
    return a;
}

bool InTri(const P2& p, const P2& a, const P2& b, const P2& c) {
    return Cross(a, b, p) >= 0.0 && Cross(b, c, p) >= 0.0 && Cross(c, a, p) >= 0.0;
}

// Ear clipping of one counter-clockwise polygon (indices into pts) -> triangles. A polygon that
// stops yielding ears (self-touching) is closed by a fan of what is left, so no roof goes missing.
void EarClip(const std::vector<P2>& pts, std::vector<uint32_t> idx, std::vector<uint32_t>& tris) {
    size_t guard = 0;
    while (idx.size() > 3 && guard < idx.size()) {
        bool clipped = false;
        for (size_t i = 0; i < idx.size(); ++i) {
            const size_t ip = (i + idx.size() - 1) % idx.size(), in = (i + 1) % idx.size();
            const P2 &a = pts[idx[ip]], &b = pts[idx[i]], &c = pts[idx[in]];
            if (Cross(a, b, c) <= 0.0) continue;   // reflex or flat
            bool empty = true;
            for (size_t k = 0; k < idx.size() && empty; ++k) {
                if (k == ip || k == i || k == in) continue;
                const P2& q = pts[idx[k]];
                if ((q.x == a.x && q.y == a.y) || (q.x == b.x && q.y == b.y) || (q.x == c.x && q.y == c.y)) continue;
                empty = !InTri(q, a, b, c);
            }
            if (!empty) continue;
            tris.insert(tris.end(), {idx[ip], idx[i], idx[in]});
            idx.erase(idx.begin() + static_cast<std::ptrdiff_t>(i));
            clipped = true;
            guard = 0;
            break;
        }
        if (!clipped) ++guard;
        if (!clipped) break;
    }
    for (size_t i = 1; i + 1 < idx.size(); ++i) tris.insert(tris.end(), {idx[0], idx[i], idx[i + 1]});
}

}  // namespace

// One solid's prisms into `out`, about its cell's origin `org` (flat frame, doubles).
static void Prisms(const BuildingSolid& s, const double org[3], const BuildingLayer::Place& place,
            const PlaceField* gnd, std::vector<BuildingLayer::Vertex>& out) {
    using Vertex = BuildingLayer::Vertex;
    auto ground = [gnd](double la, double lo) { double v = 0.0; if (gnd) gnd->Read(la, lo, v); return v; };
    constexpr double kDeg = 3.14159265358979323846 / 180.0;
    {
        const std::vector<double>& o = s.rings[0];
        const double lon0 = o[0], lat0 = o[1];

        // The ground: the lowest composed height under the footprint's outer ring.
        double base = 1e30;
        for (size_t k = 0; k + 1 < o.size(); k += 2) base = (std::min)(base, ground(o[k + 1], o[k]));
        const double zb = base + s.bottom, zt = base + s.top;

        // The footprint in metres on its own tangent plane (east, north), and that plane's
        // directions in the flat frame for the walls' normals.
        const double mx = std::cos(lat0 * kDeg) * 6371008.8 * kDeg, my = 6371008.8 * kDeg;
        double e3[3], n3[3], p0[3], pe[3], pn[3];
        place(lat0, lon0, zb, p0);
        place(lat0, lon0 + 1e-4, zb, pe);
        place(lat0 + 1e-4, lon0, zb, pn);
        double le = 0.0, ln = 0.0;
        for (int i = 0; i < 3; ++i) {
            e3[i] = pe[i] - p0[i];
            n3[i] = pn[i] - p0[i];
            le += e3[i] * e3[i];
            ln += n3[i] * n3[i];
        }
        for (int i = 0; i < 3; ++i) {
            e3[i] /= std::sqrt(le);
            n3[i] /= std::sqrt(ln);
        }
        double u3[3] = {e3[1] * n3[2] - e3[2] * n3[1], e3[2] * n3[0] - e3[0] * n3[2],
                        e3[0] * n3[1] - e3[1] * n3[0]};
        if (u3[0] * (p0[0]) + u3[1] * (p0[1] + 6.4e6) + u3[2] * p0[2] < 0.0) {   // point it away from the centre
            for (double& c : u3) c = -c;
        }
        const uint8_t part = s.kind == 1 ? 2 : 0;
        auto emit = [&](double lat, double lon, double z, const double nrm[3], float kind) {
            double p[3];
            place(lat, lon, z, p);
            Vertex v{};
            for (int i = 0; i < 3; ++i) {
                v.pos[i] = static_cast<float>(p[i] - org[i]);
                v.n[i] = static_cast<float>(nrm[i]);
            }
            v.kind = kind + part;
            out.push_back(v);
        };

        // Rings in the plane, each oriented: the outer counter-clockwise, the holes clockwise.
        std::vector<std::vector<P2>> rings2(s.rings.size());
        for (size_t r = 0; r < s.rings.size(); ++r) {
            const std::vector<double>& g = s.rings[r];
            for (size_t k = 0; k + 1 < g.size(); k += 2) rings2[r].push_back({(g[k] - lon0) * mx, (g[k + 1] - lat0) * my});
        }
        // Walls: every edge of every ring, its outward normal from the ring's own winding.
        for (size_t r = 0; r < s.rings.size(); ++r) {
            const std::vector<double>& g = s.rings[r];
            const size_t n = g.size() / 2;
            const double sgn = (Area2(rings2[r]) >= 0.0) == (s.outer[r] != 0) ? 1.0 : -1.0;
            for (size_t i = 0; i < n; ++i) {
                const size_t j = (i + 1) % n;
                const double dx = rings2[r][j].x - rings2[r][i].x, dy = rings2[r][j].y - rings2[r][i].y;
                const double len = std::hypot(dx, dy);
                if (!(len > 1e-6)) continue;
                const double ox = sgn * dy / len, oy = -sgn * dx / len;   // right of a CCW edge: out
                const double nrm[3] = {ox * e3[0] + oy * n3[0], ox * e3[1] + oy * n3[1], ox * e3[2] + oy * n3[2]};
                const double ai = g[2 * i + 1], oi = g[2 * i], aj = g[2 * j + 1], oj = g[2 * j];
                emit(ai, oi, zb, nrm, 0.0f);
                emit(aj, oj, zb, nrm, 0.0f);
                emit(aj, oj, zt, nrm, 0.0f);
                emit(ai, oi, zb, nrm, 0.0f);
                emit(aj, oj, zt, nrm, 0.0f);
                emit(ai, oi, zt, nrm, 0.0f);
            }
        }
        // The roof: the outer ring with each hole bridged in from its nearest outer vertex, then
        // ear-clipped. Positions index back into the solid's own degrees, so the roof's corners are
        // the walls' corners exactly.
        std::vector<P2> pts;
        std::vector<std::pair<double, double>> geo;   // lon, lat per pts entry
        std::vector<uint32_t> poly;
        auto push = [&](size_t r, size_t k) {
            pts.push_back(rings2[r][k]);
            geo.push_back({s.rings[r][2 * k], s.rings[r][2 * k + 1]});
            return static_cast<uint32_t>(pts.size() - 1);
        };
        {
            const size_t n = rings2[0].size();
            const bool ccw = Area2(rings2[0]) >= 0.0;
            for (size_t i = 0; i < n; ++i) poly.push_back(push(0, ccw ? i : n - 1 - i));
        }
        for (size_t r = 1; r < s.rings.size(); ++r) {
            if (s.outer[r]) continue;   // a second outer of a multipolygon: its own walls, no roof bridge
            const size_t n = rings2[r].size();
            const bool cw = Area2(rings2[r]) < 0.0;
            // The hole's vertex nearest any vertex of the polygon so far, and that vertex.
            size_t bestH = 0, bestP = 0;
            double best = 1e300;
            for (size_t h = 0; h < n; ++h) {
                for (size_t q = 0; q < poly.size(); ++q) {
                    const double dd = std::hypot(pts[poly[q]].x - rings2[r][h].x, pts[poly[q]].y - rings2[r][h].y);
                    if (dd < best) {
                        best = dd;
                        bestH = h;
                        bestP = q;
                    }
                }
            }
            std::vector<uint32_t> hole;
            for (size_t k = 0; k <= n; ++k) {
                const size_t h = cw ? (bestH + k) % n : (bestH + n - k) % n;
                hole.push_back(push(r, h));
            }
            hole.push_back(poly[bestP]);
            poly.insert(poly.begin() + static_cast<std::ptrdiff_t>(bestP) + 1, hole.begin(), hole.end());
        }
        std::vector<uint32_t> tris;
        EarClip(pts, poly, tris);
        for (uint32_t t : tris) emit(geo[t].second, geo[t].first, zt, u3, 1.0f);
        // A second outer ring (a multipolygon's other piece) gets its own roof.
        for (size_t r = 1; r < s.rings.size(); ++r) {
            if (!s.outer[r]) continue;
            std::vector<P2> rp = rings2[r];
            std::vector<uint32_t> ix(rp.size());
            for (size_t i = 0; i < ix.size(); ++i) ix[i] = static_cast<uint32_t>(Area2(rp) >= 0.0 ? i : ix.size() - 1 - i);
            std::vector<uint32_t> rt;
            EarClip(rp, ix, rt);
            for (uint32_t t : rt) emit(s.rings[r][2 * t + 1], s.rings[r][2 * t], zt, u3, 1.0f);
        }
    }

}

double BuildingLayer::Reach(const Key& k, double latDeg, double lonDeg, double h) const {
    return ReachBox(k.first * kCellDeg, k.second * kCellDeg, kCellDeg, latDeg, lonDeg, h);
}

double BuildingLayer::ReachBox(double x0, double y0, double deg, double latDeg, double lonDeg, double h) {
    constexpr double kDeg = 3.14159265358979323846 / 180.0, kR = 6371008.8;
    const double nx = (std::clamp)(lonDeg, x0, x0 + deg), ny = (std::clamp)(latDeg, y0, y0 + deg);
    const double sa = std::sin(0.5 * (ny - latDeg) * kDeg), so = std::sin(0.5 * (nx - lonDeg) * kDeg);
    const double q = sa * sa + std::cos(latDeg * kDeg) * std::cos(ny * kDeg) * so * so;
    const double ground = 2.0 * kR * std::asin(std::sqrt((std::min)(1.0, q)));
    return std::hypot(ground, (std::max)(h, 0.0));
}

void BuildingLayer::Want(Gpu* gpu, double latDeg, double lonDeg, double h) {
    // The cells about the eye, nearest first; those not resident and not building are asked for
    // until kInFlight are building.
    constexpr double kDeg = 3.14159265358979323846 / 180.0;
    const int ex = static_cast<int>(std::floor(lonDeg / kCellDeg)), ey = static_cast<int>(std::floor(latDeg / kCellDeg));
    const double cellM = kCellDeg * 111195.0 * (std::max)(0.2, std::cos(latDeg * kDeg));
    const int span = static_cast<int>(std::ceil(m_radius / cellM)) + 1;
    std::vector<std::pair<double, Key>> want;
    const uint64_t rewarmed0 = m_rewarmed;
    for (int dy = -span; dy <= span; ++dy) {
        for (int dx = -span; dx <= span; ++dx) {
            const Key k{ex + dx, ey + dy};
            const double r = Reach(k, latDeg, lonDeg, h);
            if (r > m_radius || m_cells.count(k) || m_pending.count(k)) continue;
            auto w = m_warm.find(k);
            if (w != m_warm.end()) {   // back from the warm pool: drawn this frame, nothing built
                m_warmBytes -= uint64_t(w->second.cell.count) * sizeof(Vertex);
                m_cells[k] = std::move(w->second.cell);
                ++m_cellsVersion;
                m_warm.erase(w);
                ++m_rewarmed;
                continue;
            }
            want.push_back({r, k});
        }
    }
    if (m_rewarmed != rewarmed0) {
        Log("[buildings] frame %llu: %llu cells back warm, drawn this frame (%zu resident, %zu warm, %.0f MB warm)",
            static_cast<unsigned long long>(m_frame), static_cast<unsigned long long>(m_rewarmed - rewarmed0),
            m_cells.size(), m_warm.size(), m_warmBytes / 1048576.0);
    }
    std::sort(want.begin(), want.end());
    for (const auto& [r, k] : want) {
        if (m_shared->inflight.load() >= kInFlight) break;
        m_pending.insert(k);
        m_shared->inflight.fetch_add(1);
        Threads().Submit(Lane::Io, "buildings.cell",
                         [shared = m_shared, stack = m_stack, place = m_place, ground = m_ground, gpu, k] {
            const std::unique_ptr<PlaceField> fine = ground ? ground(1.0) : nullptr;   // the prisms' ground, at a metre
            // Still wanted: the cell within the keep reach of where the eye is NOW, not where it was asked.
            auto wanted = [&] {
                std::lock_guard<std::mutex> lk(shared->mx);
                return ReachBox(k.first * kCellDeg, k.second * kCellDeg, kCellDeg, shared->eyeLat, shared->eyeLon,
                                shared->eyeH) <= shared->keepM;
            };
            if (!shared->cancel.load()) {
                const auto t0 = std::chrono::steady_clock::now();
                Built b;
                b.key = k;
                b.stale = !wanted();
                const double x0 = k.first * kCellDeg, y0 = k.second * kCellDeg;
                place(y0 + 0.5 * kCellDeg, x0 + 0.5 * kCellDeg, 0.0, b.origin);
                // The margin: a footprint whose first point is a little over the edge still covers.
                const std::vector<BuildingSolid> solids =
                    b.stale ? std::vector<BuildingSolid>{} : stack->Compose(x0, y0, x0 + kCellDeg, y0 + kCellDeg, 0.002);
                b.stale = b.stale || !wanted();
                b.drawn.assign(stack->Sources(), 0);
                std::vector<Vertex> verts;
                for (size_t si = 0; si < solids.size() && !b.stale; ++si) {
                    const BuildingSolid& s = solids[si];
                    if (shared->cancel.load()) break;
                    if ((si & 1023u) == 1023u && !wanted()) {
                        b.stale = true;
                        break;
                    }
                    Prisms(s, b.origin, place, fine.get(), verts);
                    if (s.source >= 0) ++b.drawn[static_cast<size_t>(s.source)];
                }
                b.count = b.stale ? 0u : static_cast<uint32_t>(verts.size());
                if (b.count && !shared->cancel.load()) {
                    const uint64_t bytes = verts.size() * sizeof(Vertex);
                    b.staging = gpu->CreateUploadBuffer(bytes, L"buildings.staging");
                    std::memcpy(b.staging.cpu, verts.data(), bytes);
                    b.vb = gpu->CreateDefaultBuffer(nullptr, bytes, L"buildings.cell");
                }
                b.ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
                std::lock_guard<std::mutex> lk(shared->mx);
                shared->done.push_back(std::move(b));
            }
            shared->inflight.fetch_sub(1);
        });
    }
}

void BuildingLayer::Upload(const FrameContext& ctx) {
    std::vector<Built> done;
    {
        std::lock_guard<std::mutex> lk(m_shared->mx);
        const size_t n = (std::min)(m_shared->done.size(), static_cast<size_t>(kUploadsPerFrame));
        done.assign(std::make_move_iterator(m_shared->done.begin()),
                    std::make_move_iterator(m_shared->done.begin() + static_cast<std::ptrdiff_t>(n)));
        m_shared->done.erase(m_shared->done.begin(), m_shared->done.begin() + static_cast<std::ptrdiff_t>(n));
    }
    for (Built& b : done) {
        const auto t0 = std::chrono::steady_clock::now();
        m_pending.erase(b.key);
        if (b.stale) {   // the eye left before it was built: nothing to keep, asked again if it returns
            ++m_staleBuilds;
            Log("[buildings] frame %llu cell %d,%d dropped unbuilt after %.0f ms: the eye left its reach (%llu so far)",
                static_cast<unsigned long long>(m_frame), b.key.first, b.key.second, b.ms,
                static_cast<unsigned long long>(m_staleBuilds));
            continue;
        }
        Cell c{};
        for (int i = 0; i < 3; ++i) c.origin[i] = b.origin[i];
        c.count = b.vb.Valid() ? b.count : 0;
        if (c.count) {
            // Copied on THIS frame's list: the cell's buffer is born COMMON, the copy promotes it to
            // COPY_DEST, and one barrier hands it to the vertex shader's reads.
            c.vb = std::move(b.vb);
            ID3D12GraphicsCommandList* cl = ctx.cmd->Native();
            cl->CopyBufferRegion(c.vb.res.Get(), 0, b.staging.res.Get(), 0, b.staging.size);
            D3D12_RESOURCE_BARRIER br{};
            br.Type = D3D12_RESOURCE_BARRIER_TYPE_TRANSITION;
            br.Transition.pResource = c.vb.res.Get();
            br.Transition.Subresource = D3D12_RESOURCE_BARRIER_ALL_SUBRESOURCES;
            br.Transition.StateBefore = D3D12_RESOURCE_STATE_COPY_DEST;
            br.Transition.StateAfter = D3D12_RESOURCE_STATE_ALL_SHADER_RESOURCE;   // the root slot is seen by every stage
            cl->ResourceBarrier(1, &br);
            m_retired.push_back({std::move(b.staging), m_frame});   // freed once this frame has retired
        }
        std::string tally;
        for (size_t k = 0; k < b.drawn.size(); ++k) {
            if (!b.drawn[k]) continue;
            tally += (tally.empty() ? "" : ", ") + m_stack->Name(k) + " " + std::to_string(b.drawn[k]);
        }
        m_cells[b.key] = std::move(c);
        ++m_cellsVersion;
        ++m_built;
        const double up = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
        Log("[buildings] frame %llu cell %d,%d (%.2f, %.2f): %u vertices (%.1f MB), built %.0f ms, frame's upload %.2f ms, %zu "
            "resident (%llu built, %llu back warm) -- %s", static_cast<unsigned long long>(m_frame), b.key.first, b.key.second, b.key.second * kCellDeg, b.key.first * kCellDeg,
            m_cells[b.key].count, m_cells[b.key].count * sizeof(Vertex) / 1048576.0, b.ms, up, m_cells.size(),
            static_cast<unsigned long long>(m_built), static_cast<unsigned long long>(m_rewarmed),
            tally.empty() ? "no solids" : tally.c_str());
    }
}

namespace {
// The ground's frame at a point, flat frame: p, unit east, north and up (away from the centre) --
// the prisms' own construction, so a box and its building stand on the same axes.
void PointFrame(const BuildingLayer::Place& place, double lat, double lon, double z, double p0[3], double e3[3],
                double n3[3], double u3[3]) {
    double pe[3], pn[3];
    place(lat, lon, z, p0);
    place(lat, lon + 1e-4, z, pe);
    place(lat + 1e-4, lon, z, pn);
    double le = 0.0, ln = 0.0;
    for (int i = 0; i < 3; ++i) {
        e3[i] = pe[i] - p0[i];
        n3[i] = pn[i] - p0[i];
        le += e3[i] * e3[i];
        ln += n3[i] * n3[i];
    }
    for (int i = 0; i < 3; ++i) {
        e3[i] /= std::sqrt(le);
        n3[i] /= std::sqrt(ln);
    }
    // Up is asked of the placement too (a metre higher), never e x n: the flat frame's handedness
    // would decide that one's sign, and a shape extruded along it stood upside down in the ground.
    double pu[3], lu = 0.0;
    place(lat, lon, z + 1.0, pu);
    for (int i = 0; i < 3; ++i) {
        u3[i] = pu[i] - p0[i];
        lu += u3[i] * u3[i];
    }
    for (int i = 0; i < 3; ++i) u3[i] /= std::sqrt(lu);
}
}  // namespace

static int FloorDivI(int a, int n) { return a >= 0 ? a / n : -((-a + n - 1) / n); }

// THE ATTITUDE AS A ROTOR: the rotor taking the flat frame's axes (x east, y up, z north: a
// right-handed triple) to the local east, up and north, as its four components (x, y, z, w) -- the
// GPU rotates the axes back out (Buildings.hlsl Rotate). The order matters: (east, north, up) is
// LEFT-handed in this frame, and a rotor cannot reach a mirror -- built from it, the footprints came
// out mirrored, their near walls culled as backs and their far walls lit (2026-10-10). Shepperd's
// choice of the largest diagonal keeps it exact for every attitude.
static void RotorOf(const double E[3], const double U[3], const double N[3], float q[4]) {
    // The matrix whose columns are E, U, N: m[row][col].
    const double m00 = E[0], m10 = E[1], m20 = E[2], m01 = U[0], m11 = U[1], m21 = U[2], m02 = N[0], m12 = N[1], m22 = N[2];
    const double tr = m00 + m11 + m22;
    double x, y, z, w;
    if (tr > 0.0) {
        const double s = std::sqrt(tr + 1.0) * 2.0;
        w = 0.25 * s; x = (m21 - m12) / s; y = (m02 - m20) / s; z = (m10 - m01) / s;
    } else if (m00 > m11 && m00 > m22) {
        const double s = std::sqrt(1.0 + m00 - m11 - m22) * 2.0;
        w = (m21 - m12) / s; x = 0.25 * s; y = (m01 + m10) / s; z = (m02 + m20) / s;
    } else if (m11 > m22) {
        const double s = std::sqrt(1.0 + m11 - m00 - m22) * 2.0;
        w = (m02 - m20) / s; x = (m01 + m10) / s; y = 0.25 * s; z = (m12 + m21) / s;
    } else {
        const double s = std::sqrt(1.0 + m22 - m00 - m11) * 2.0;
        w = (m10 - m01) / s; x = (m02 + m20) / s; y = (m12 + m21) / s; z = 0.25 * s;
    }
    q[0] = float(x); q[1] = float(y); q[2] = float(z); q[3] = float(w);
}
static double Len3(const float v[3]) { return std::sqrt(double(v[0]) * v[0] + double(v[1]) * v[1] + double(v[2]) * v[2]); }
// The rotor applying b then a (Buildings.hlsl QMul, w last).
static void QMul(const float a[4], const float b[4], float o[4]) {
    o[0] = a[3] * b[0] + b[3] * a[0] + (a[1] * b[2] - a[2] * b[1]);
    o[1] = a[3] * b[1] + b[3] * a[1] + (a[2] * b[0] - a[0] * b[2]);
    o[2] = a[3] * b[2] + b[3] * a[2] + (a[0] * b[1] - a[1] * b[0]);
    o[3] = a[3] * b[3] - (a[0] * b[0] + a[1] * b[1] + a[2] * b[2]);
}
// A detail cell as one word (the classify pass compares words): ix, iy as int16s.
static uint32_t PackCell(const std::pair<int, int>& c) {
    return (uint32_t(uint16_t(int16_t(c.second))) << 16) | uint32_t(uint16_t(int16_t(c.first)));
}
// A STOOD BOX AS ITS RECORD (StoodRec): about the stand's centre, the half-axes as ONE ROTOR and
// three extents -- an axis of no length takes its direction from the other two (a fold of no
// spread is still a box with an attitude) -- and what the per-building law reads. A box that
// stands nothing claims no size (rho 0), so the pass draws it as nothing: the one law.
void BuildingLayer::RecordOf(const RtBox& b, const double c[3], float rho, uint32_t addr, float heading, uint32_t cell,
                             float landed, float zc, StoodRec& r) {
    r = StoodRec{};
    r.addr = addr;
    r.heading = heading;
    r.cell = cell;
    r.landed = landed;
    r.zc = zc;
    if (!b.valid) return;
    const double a1 = Len3(b.u), a2 = Len3(b.v), hz = Len3(b.w);
    if (hz <= 1e-9 || (a1 <= 1e-9 && a2 <= 1e-9)) return;   // nothing stands
    double E[3], N[3], U[3];
    for (int i = 0; i < 3; ++i) {
        E[i] = a1 > 1e-9 ? b.u[i] / a1 : 0.0;
        N[i] = a2 > 1e-9 ? b.v[i] / a2 : 0.0;
        U[i] = b.w[i] / hz;
    }
    if (a1 <= 1e-9) { E[0] = N[1] * U[2] - N[2] * U[1]; E[1] = N[2] * U[0] - N[0] * U[2]; E[2] = N[0] * U[1] - N[1] * U[0]; }
    if (a2 <= 1e-9) { N[0] = U[1] * E[2] - U[2] * E[1]; N[1] = U[2] * E[0] - U[0] * E[2]; N[2] = U[0] * E[1] - U[1] * E[0]; }
    for (int i = 0; i < 3; ++i) r.p[i] = static_cast<float>(b.p[i] - c[i]);
    RotorOf(E, U, N, r.q);
    r.a1 = static_cast<float>(a1);
    r.a2 = static_cast<float>(a2);
    r.hz = static_cast<float>(hz);
    r.rho = rho;
}

void BuildingLayer::LoadPage(const PageKey& k) {
    const int L = std::get<0>(k);
    const LodPage* lp = m_lod->Find(L, std::get<1>(k), std::get<2>(k));
    if (!lp) return;
    m_pagePending.insert(k);
    m_pendingSince[k] = m_frame;
    m_shared->pageInflight.fetch_add(1);
    Threads().Submit(Lane::Io, "buildings.page",
                     [shared = m_shared, lodf = m_lod, place = m_place, ground = m_ground, gpu = m_gpu,
                      shapes = m_psoShape != nullptr, k, lp = *lp, L, tAsk = std::chrono::steady_clock::now()] {
        auto page = std::make_shared<Page>();
        std::vector<LodNode> nodes;
        std::vector<LodBuilding> blds;
        const auto tRun = std::chrono::steady_clock::now();
        const uint64_t dec0 = RasterFileSource::s_decodes.load(), decUs0 = RasterFileSource::s_decodeUs.load();
        page->waitMs = std::chrono::duration<double, std::milli>(tRun - tAsk).count();
        const bool read = !shared->cancel.load() && lodf->Read(L, lp, nodes, blds);
        page->readMs = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - tRun).count();
        if (!read) Log("[buildings] page L%d (%d,%d) could not be read from the tree", L, std::get<1>(k), std::get<2>(k));
        if (read) {
            page->L = L;
            page->blds.resize(blds.size());
            // The shapes (GALOD04): one record a building, in order -- packed here into whole pool slots,
            // a record never crossing one, each building addressed by (slot ordinal, offset in it).
            std::vector<uint8_t> shapeBytes, packed;
            if (shapes && lodf->HasShapes()) lodf->ReadShapes(L, lp, shapeBytes);
            size_t at = 0;
            uint32_t fill = kSlotBytes;   // the open slot's bytes used (full: the next record opens one)
            for (size_t i = 0; i < blds.size(); ++i) {
                double lat, lon, zc, hz, a1, a2, hd;
                int cx, cy;
                LodUnpack(blds[i], lat, lon, zc, hz, a1, a2, hd, cx, cy);
                RtBld& b = page->blds[i];
                ShapeView sv;
                if (at < shapeBytes.size() && ReadShape(shapeBytes.data() + at, shapeBytes.data() + shapeBytes.size(), sv)) {
                    const size_t rb = shape::Bytes(sv.head);
                    const uint32_t minV = (sv.head.flags & shape::kRibbon) ? 2u : 3u;   // a ribbon is a line of two
                    if (sv.head.nVerts >= minV && rb <= kSlotBytes) {
                        if (fill + rb > kSlotBytes) {
                            page->slotUsed.push_back(0);
                            packed.resize(page->slotUsed.size() * size_t(kSlotBytes), 0);
                            fill = 0;
                        }
                        const size_t addr = (page->slotUsed.size() - 1) * size_t(kSlotBytes) + fill;
                        std::memcpy(packed.data() + addr, shapeBytes.data() + at, rb);
                        fill += static_cast<uint32_t>(rb);
                        page->slotUsed.back() = fill;
                        b.shapeOff = static_cast<uint32_t>(addr);   // (packed is kept: see Page::packed)
                        b.nV = sv.head.nVerts;
                        b.nT = sv.head.nTris;
                    } else if (sv.head.nVerts >= minV) {
                        ++page->tooBig;
                    }
                    at += rb;
                }
                b.cell = {cx, cy};
                b.rho = static_cast<float>(std::sqrt(a1 * a1 + a2 * a2 + hz * hz));
            }
            const double Q = lod::QuadDeg(L), Qm = Q * lod::kMetresPerDeg;
            page->nodes.resize(nodes.size());
            for (size_t i = 0; i < nodes.size(); ++i) {
                const LodNode& n = nodes[i];
                RtNode& r = page->nodes[i];
                r.x = n.x;
                r.y = n.y;
                r.own0 = static_cast<uint32_t>(n.ownFirst - lp.bldFirst);
                r.ownN = n.ownCount;
                r.kids = n.desc.hz > 0.0f;
                const double latc = (n.y + 0.5) * Q, lonc = (n.x + 0.5) * Q;
                place(latc, lonc, 0.0, r.c);   // at sea level: its stand knows the ground
                r.own = n.own;
                r.desc = n.desc;
                // The quad's corners and what stands on it, from its centre.
                r.rad = static_cast<float>(0.75 * Qm + (std::max)(n.own.zc + n.own.hz, n.desc.zc + n.desc.hz) + 50.0);
                page->at[Page::QuadKey(n.x, n.y)] = static_cast<uint32_t>(i);
                // THE RANK WITHIN A NODE: its own buildings largest first, so the walk stops at the
                // first one under the threshold at the node's nearest distance (all after are smaller).
                if (r.own0 < page->blds.size()) {
                    const auto b0 = page->blds.begin() + r.own0;
                    const auto b1 = page->blds.begin() + (std::min<size_t>)(size_t(r.own0) + r.ownN, page->blds.size());
                    std::sort(b0, b1, [](const RtBld& a, const RtBld& b) { return a.rho > b.rho; });
                }
            }
            page->raw = blds;   // the stands read the records
            page->stood = std::make_unique<std::atomic<std::shared_ptr<const Stood>>[]>(nodes.size());
            if (L == lod::kLmin) {
                for (const RtNode& r : page->nodes) {
                    if (!r.ownN || r.own0 >= page->blds.size()) continue;   // sorted: the first is the largest
                    float& m = page->parentRho[Page::QuadKey(r.x >> 1, r.y >> 1)];
                    m = (std::max)(m, page->blds[r.own0].rho);
                }
            }
            page->bytes = nodes.size() * (sizeof(RtNode) + 48) + blds.size() * sizeof(RtBld);
            if (!packed.empty() && gpu) {   // staged here, copied slot by slot into the pool when the page lands
                page->shapeStaging = gpu->CreateUploadBuffer(packed.size(), L"buildings.shapes.staging");
                std::memcpy(page->shapeStaging.cpu, packed.data(), packed.size());
                page->bytes += packed.size();
                page->packed = std::move(packed);
            }
        }
        // THE INSTRUMENT (docs/BUILDING_LOD.md section 11): a page that took long to stand says where
        // the time went -- its buildings' ground, and the raster windows every thread decoded meanwhile.
        const double parseMs = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - tRun).count() - page->readMs;
        if (parseMs > 100.0) {
            Log("[buildings] page L%d (%d,%d): read %.1f ms, then %.0f ms parsing %zu buildings (nothing stood: the stands are the walk's); meanwhile %llu raster windows decoded in %.0f ms (every source, every thread)",
                L, std::get<1>(k), std::get<2>(k), page->readMs, parseMs, page->blds.size(),
                static_cast<unsigned long long>(RasterFileSource::s_decodes.load() - dec0), double(RasterFileSource::s_decodeUs.load() - decUs0) / 1000.0);
        }
        std::lock_guard<std::mutex> lk(shared->mx);
        shared->pagesDone.push_back({k, page});
        shared->pageInflight.fetch_sub(1);
    });
}

void BuildingLayer::TreeFrame(const FrameContext& ctx, double latDeg, double lonDeg, double h) {
    // 1. What the pool finished: pages into the tree, a walk's boxes onto the GPU.
    std::vector<std::pair<PageKey, std::shared_ptr<Page>>> pages;
    std::vector<Walked> walked;
    std::vector<Shared::StandDone> stands;
    {
        std::lock_guard<std::mutex> lk(m_shared->mx);
        pages.swap(m_shared->pagesDone);
        walked.swap(m_shared->walked);
        stands.swap(m_shared->standDone);
    }
    // THE STANDS THAT LANDED: a ribbon's heights into the pool's slots (its stand found the
    // ground under each vertex), the page's pointer already published by the job itself.
    if (!stands.empty() && m_frame - m_lastStandWalk >= uint64_t(kStandWalkFrames)) {
        ++m_pagesVersion;   // what a walk depends on changed: the next walk draws what stood
        m_lastStandWalk = m_frame;
    }
    {
        // ONE staging buffer for the frame's stands: their records into the stands' pool and a
        // ribbon's heights into the shape pool's slots, however many landed (a buffer a ribbon
        // stalled the frame: hundreds of resource creations a frame over a road tree). A stand is
        // PUBLISHED here, once its records are recorded for the GPU, so a walk that reads it names
        // records the pass will find; the stand it replaces gives its range back when nothing
        // holds it and no pass in flight can read it.
        size_t total = 0;
        const bool zOk = m_pool.Valid(), rOk = m_stood.Valid();
        for (const Shared::StandDone& sd : stands) {
            if (zOk) for (const auto& [off, bytes] : sd.zWrites) total += (bytes.size() + 15u) & ~size_t(15);
            if (rOk) total += (sd.recs.size() + 15u) & ~size_t(15);
        }
        GpuBuffer st;
        size_t at = 0;
        bool zCopied = false, rCopied = false;
        if (total && ctx.gpu) st = ctx.gpu->CreateUploadBuffer(total, L"buildings.stand.staging");
        for (Shared::StandDone& sd : stands) {
            ++m_stands;
            m_standMs += sd.ms;
            m_standPending.erase({sd.key, sd.node});
            if (st.Valid()) {
                if (zOk) {
                    for (auto& [off, bytes] : sd.zWrites) {
                        if (bytes.empty()) continue;
                        std::memcpy(st.cpu + at, bytes.data(), bytes.size());
                        ctx.cmd->Native()->CopyBufferRegion(m_pool.res.Get(), off, st.res.Get(), at, bytes.size());
                        at += (bytes.size() + 15u) & ~size_t(15);
                        zCopied = true;
                    }
                }
                if (rOk && !sd.recs.empty() && sd.stood) {
                    std::memcpy(st.cpu + at, sd.recs.data(), sd.recs.size());
                    ctx.cmd->Native()->CopyBufferRegion(m_stood.res.Get(), uint64_t(sd.stood->gpuOff) * sizeof(StoodRec), st.res.Get(), at,
                                                        sd.recs.size());
                    at += (sd.recs.size() + 15u) & ~size_t(15);
                    rCopied = true;
                }
            }
            if (sd.stood && sd.page && sd.page->stood && sd.node < sd.page->nodes.size()) {
                const std::shared_ptr<const Stood> old = sd.page->stood[sd.node].exchange(sd.stood, std::memory_order_acq_rel);
                if (old && old->gpuN) m_stoodRetired.push_back({old->gpuOff, old->gpuN, m_frame, old});
            }
        }
        if (st.Valid()) m_retired.push_back({std::move(st), m_frame});
        if (zCopied) ctx.cmd->Barrier(m_pool.res.Get(), D3D12_RESOURCE_STATE_COPY_DEST, D3D12_RESOURCE_STATE_ALL_SHADER_RESOURCE);
        if (rCopied) ctx.cmd->Barrier(m_stood.res.Get(), D3D12_RESOURCE_STATE_COPY_DEST, D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);
    }
    for (auto it = m_stoodRetired.begin(); it != m_stoodRetired.end();) {   // a replaced stand's records, freed once nothing reads them
        if (!it->stood.expired()) {
            it->frame = m_frame;
            ++it;
        } else if (m_frame - it->frame > kRetireFrames) {
            m_stoodFree.Give(it->off, it->n);
            it = m_stoodRetired.erase(it);
        } else {
            ++it;
        }
    }
    if (m_psoShape && !m_pool.Valid() && ctx.gpu && m_lod && m_lod->HasShapes()) {
        m_pool = ctx.gpu->CreateDefaultBuffer(nullptr, kPoolBytes, L"buildings.shapes");
        m_poolSrv = ctx.gpu->CreateStructuredBufferSrv(m_pool.res.Get(), static_cast<uint32_t>(kPoolBytes / 16), 16);
        m_freeSlots.clear();
        for (uint32_t i = kSlots; i-- > 0;) m_freeSlots.push_back(i);   // the lowest on top
    }
    for (auto it = m_slotsRetired.begin(); it != m_slotsRetired.end();) {   // freed once no frame reads them
        if (!it->page.expired()) {   // a walk in flight still holds the page and will draw from its slots
            it->frame = m_frame;
            ++it;
        } else if (m_frame - it->frame > kRetireFrames) {
            m_freeSlots.insert(m_freeSlots.end(), it->slots.begin(), it->slots.end());
            it = m_slotsRetired.erase(it);
        } else {
            ++it;
        }
    }
    // THE LANDING: a page joins the tree when its slots are free; until then it waits, in order.
    for (auto& kp : pages) m_roomWait.push_back(std::move(kp));
    bool shapesCopied = false;
    size_t stillWaiting = 0;
    for (auto it = m_roomWait.begin(); it != m_roomWait.end();) {
        const PageKey k = it->first;
        std::shared_ptr<Page>& p = it->second;
        const size_t need = m_pool.Valid() ? p->slotUsed.size() : 0;
        if (need > m_freeSlots.size()) {
            ++stillWaiting;
            ++it;
            continue;
        }
        for (size_t j = 0; j < need; ++j) {
            const uint32_t slot = m_freeSlots.back();
            m_freeSlots.pop_back();
            p->slots.push_back(slot);
            ctx.cmd->Native()->CopyBufferRegion(m_pool.res.Get(), uint64_t(slot) * kSlotBytes, p->shapeStaging.res.Get(),
                                                uint64_t(j) * kSlotBytes, p->slotUsed[j]);
            shapesCopied = true;
        }
        if (p->shapeStaging.Valid()) m_retired.push_back({std::move(p->shapeStaging), m_frame});
        if (p->tooBig) {
            m_tooBig += p->tooBig;
            Log("[buildings] frame %llu: %u shape records over a %u KB slot keep their boxes (%llu so far)",
                static_cast<unsigned long long>(m_frame), p->tooBig, kSlotBytes >> 10,
                static_cast<unsigned long long>(m_tooBig));
        }
        m_pagePending.erase(k);
        m_pendingSince.erase(k);
        m_pageBytes += p->bytes;
        ++m_pagesLoaded;
        m_pageUsed[k] = m_frame;
        p->landed = Now();
        ++m_landN;
        m_landWait += p->waitMs;
        m_landWaitMax = (std::max)(m_landWaitMax, p->waitMs);
        m_landRead += p->readMs;
        m_landReadMax = (std::max)(m_landReadMax, p->readMs);
        m_pages[k] = std::move(p);
        ++m_pagesVersion;
        it = m_roomWait.erase(it);
    }
    if (stillWaiting) {
        if (m_roomWaitFrames++ % 60 == 0) {
            size_t want = 0, coming = 0;
            for (const auto& w : m_roomWait) want += w.second->slotUsed.size();
            for (const SlotsRetired& r : m_slotsRetired) coming += r.slots.size();
            Log("[buildings] frame %llu: %zu pages wait for room in the shape pool -- %zu slots wanted, %zu free, %zu "
                "retiring (of %u slots of %u KB); no page is asked for until they land",
                static_cast<unsigned long long>(m_frame), stillWaiting, want, m_freeSlots.size(), coming, kSlots,
                kSlotBytes >> 10);
        }
    } else {
        m_roomWaitFrames = 0;
    }
    if (shapesCopied) {
        D3D12_RESOURCE_BARRIER br{};
        br.Type = D3D12_RESOURCE_BARRIER_TYPE_TRANSITION;
        br.Transition.pResource = m_pool.res.Get();
        br.Transition.Subresource = D3D12_RESOURCE_BARRIER_ALL_SUBRESOURCES;
        br.Transition.StateBefore = D3D12_RESOURCE_STATE_COPY_DEST;
        br.Transition.StateAfter = D3D12_RESOURCE_STATE_ALL_SHADER_RESOURCE;   // the root slot is seen by every stage
        ctx.cmd->Native()->ResourceBarrier(1, &br);
    }
    for (Walked& w : walked) {
        Eye& e = m_eyes[w.eye < kEyes * kWorlds ? w.eye : 0];
        // THE PASS: the node list expanded into the three forms' lists on the GPU, the counts and
        // the draws' arguments in the buffer's header; the counts copied back for the log line.
        if (w.cap && w.out.b.Valid() && w.list.Valid() && m_clsPso && m_stood.Valid()) {
            Classify(ctx, w);
            if (m_countsPending.size() < kCountsRing) {
                m_countsPending.push_back({m_frame, m_countsNext, w.eye, e.walks + 1, w.named, w.cap, w.ms});
                m_countsNext = (m_countsNext + 1) % kCountsRing;
            }
        } else {
            w.out = ListBuf{};
        }
        if (w.list.Valid()) m_retired.push_back({std::move(w.list), m_frame, 1});
        if (e.drawn.vb.Valid()) m_retired.push_back({std::move(e.drawn.vb), m_frame, 2});
        for (int i = 0; i < 3; ++i) e.drawn.origin[i] = w.origin[i];
        e.drawn.named = w.out.b.Valid() ? w.named : 0;
        e.drawn.cap = w.out.b.Valid() ? w.cap : 0;
        e.drawn.vb = std::move(w.out.b);
        w.stands.clear();   // the pass is recorded: the stands' ranges may go when nothing else holds them
        e.drawnUsed = std::set<PageKey>(w.used.begin(), w.used.end());
        e.drawnHold = std::move(w.hold);
        e.drawnCells = w.cellsVersion;
        for (const PageKey& k : w.used) m_pageUsed[k] = m_frame;
        m_hidden[w.eye < kEyes * kWorlds ? w.eye : 0] = w.hidden;
        // The pages the walk wanted, the largest on screen first; the stands likewise.
        e.wants = std::move(w.wants);
        e.stands = std::move(w.asks);
        m_standQueueStale = true;   // the stream's order is remade from every eye's asks
        ++e.walks;
        if (e.walks <= 3 || e.walks % 50 == 0) {
            Log("[buildings] frame %llu eye %u world %u walk %llu: %u nodes named, %u records sent (%llu buildings) over %llu nodes visited, %llu hidden, %llu cut by the air, in %.1f ms (visit %.1f: "
                "roots %.1f, %u subtrees on %u threads %.1f, merge %.1f; list %.1f, buffers %.1f); %zu pages (%.0f MB), %zu asked; the stands' pool %u of %u records",
                static_cast<unsigned long long>(m_frame), w.eye / kWorlds, w.eye % kWorlds,
                static_cast<unsigned long long>(e.walks), w.named, w.cap, static_cast<unsigned long long>(w.singles),
                static_cast<unsigned long long>(w.nodes), static_cast<unsigned long long>(w.hidden),
                static_cast<unsigned long long>(w.aired), w.ms, w.msVisit, w.msRoots, w.subtrees, w.threads, w.msDrain,
                w.msVisit - w.msRoots - w.msDrain, w.msList,
                w.msBuf, m_pages.size(), m_pageBytes / 1048576.0, e.wants.size(), m_stoodFree.used, m_stoodFree.total);
        }
    }
    // THE COUNTS THAT CAME BACK: what the GPU drew of what each walk sent, kCountsLag frames on.
    for (auto it = m_countsPending.begin(); it != m_countsPending.end();) {
        if (m_frame < it->frame + kCountsLag || !m_countsRb.Valid()) { ++it; continue; }
        uint32_t c[4];
        std::memcpy(c, m_countsRb.cpu + size_t(it->slot) * 16u, 16);
        for (int i = 0; i < 3; ++i) m_eyes[it->eye < kEyes * kWorlds ? it->eye : 0].lastDrawn[i] = c[i];
        if (it->walk <= 3 || it->walk % 50 == 0) {
            const float ms = ctx.prof ? ctx.prof->LastMs("buildings.classify") : -1.0f;
            Log("[buildings] frame %llu eye %u world %u walk %llu drew %u boxes + %u shapes + %u points of %u records over %u nodes named; classify %.2f ms on the GPU",
                static_cast<unsigned long long>(it->frame), it->eye / kWorlds, it->eye % kWorlds, static_cast<unsigned long long>(it->walk),
                c[0], c[1], c[2], it->cap, it->named, ms);
        }
        it = m_countsPending.erase(it);
    }
    const std::vector<std::pair<double, PageKey>> wants = Wants();   // every eye's, each page once
    // THE TWO STREAMS. The coarse level is the coarsest still wanted or loading; the coarse stream
    // sends its pages only, the fine stream every finer one by size on screen, each up to its own
    // slots. A new walk (the eye moved) replaces the list: what it no longer wants is never sent.
    // While a landed page waits for pool slots, nothing more is sent: no page is read that cannot land.
    if (m_roomWait.empty()) {
        int level = INT32_MIN;
        for (const PageKey& k : m_pagePending) level = (std::max)(level, std::get<0>(k));
        for (const auto& [px, k] : wants) {
            if (!m_pages.count(k)) level = (std::max)(level, std::get<0>(k));
        }
        int coarse = 0, fine = 0;
        for (const PageKey& k : m_pagePending) ++(std::get<0>(k) == level ? coarse : fine);
        std::vector<std::pair<double, PageKey>> finer;
        for (const auto& [px, k] : wants) {
            if (m_pages.count(k) || m_pagePending.count(k)) continue;
            if (std::get<0>(k) == level) {
                if (coarse < kCoarseInFlight) {
                    LoadPage(k);
                    ++coarse;
                }
            } else if (std::get<0>(k) >= level - 1) {   // one level behind the coarse fill, no deeper
                finer.push_back({px, k});
            }
        }
        std::sort(finer.begin(), finer.end(), [](const auto& a, const auto& b) { return a.first > b.first; });
        for (const auto& [px, k] : finer) {
            if (fine >= kFineInFlight) break;
            LoadPage(k);
            ++fine;
        }
    }
    // THE LOADS' INSTRUMENT, every two seconds while anything is wanted or on its way.
    if (m_frame % 120 == 0) {
        size_t unloaded = 0;
        for (const auto& [px, k] : wants) if (!m_pages.count(k) && !m_pagePending.count(k)) ++unloaded;
        if (unloaded || !m_pagePending.empty() || m_landN) {
            Log("[buildings] frame %llu loads: %zu wanted and not asked, %zu asked and on their way, %zu waiting for room; "
                "%u landed since the last line, their wait in the lane %.1f ms (max %.1f), their read %.1f ms (max %.1f)",
                static_cast<unsigned long long>(m_frame), unloaded, m_pagePending.size(), m_roomWait.size(), m_landN,
                m_landN ? m_landWait / m_landN : 0.0, m_landWaitMax, m_landN ? m_landRead / m_landN : 0.0, m_landReadMax);
            m_landN = 0;
            m_landWait = m_landWaitMax = m_landRead = m_landReadMax = 0.0;
            for (const auto& [k, since] : m_pendingSince) {
                if (m_frame - since > 120) {
                    Log("[buildings]   page L%d (%d,%d) asked at frame %llu is still on its way %llu frames on (the tree %s it)",
                        std::get<0>(k), std::get<1>(k), std::get<2>(k), static_cast<unsigned long long>(since),
                        static_cast<unsigned long long>(m_frame - since),
                        m_lod->Find(std::get<0>(k), std::get<1>(k), std::get<2>(k)) ? "holds" : "does not hold");
                }
            }
        }
    }
    // THE STANDS' STREAM: every eye's asks merged (a node once: the finest grain, buildings if any
    // eye wants them), the largest on screen first, up to kStandInFlight on the pool at a time.
    // The merged order is made ONCE when a walk lands and consumed from a cursor frame by frame:
    // remade every frame it was a map of 4,096 asks an eye, 1.0 ms a frame on the frame thread
    // (measured at Tokyo city).
    {
        const auto tStream = std::chrono::steady_clock::now();
        if (m_standQueueStale) {
            m_standQueueStale = false;
            std::map<std::pair<PageKey, uint32_t>, StandAsk> asks;
            for (const Eye& e : m_eyes) {
                for (const StandAsk& s : e.stands) {
                    StandAsk& t = asks[{s.page, s.node}];
                    if (t.px <= 0.0) {
                        t = s;
                    } else {
                        t.blds = t.blds || s.blds;
                        t.grainM = (std::min)(t.grainM, s.grainM);
                        t.px = (std::max)(t.px, s.px);
                    }
                }
            }
            m_standQueue.clear();
            m_standQueue.reserve(asks.size());
            for (const auto& [k, s] : asks) m_standQueue.push_back(s);
            std::sort(m_standQueue.begin(), m_standQueue.end(), [](const StandAsk& a, const StandAsk& b) { return a.px > b.px; });
            m_standNext = 0;
        }
        std::vector<std::pair<std::shared_ptr<const Page>, StandAsk>> batch;
        int jobs = m_shared->standInflight.load();
        for (; m_standNext < m_standQueue.size() && jobs < kStandInFlight; ++m_standNext) {
            const StandAsk& s = m_standQueue[m_standNext];
            if (m_standPending.count({s.page, s.node})) continue;
            auto it = m_pages.find(s.page);
            if (it == m_pages.end() || !it->second->stood || s.node >= it->second->nodes.size()) continue;
            // ITS RANGE IN THE STANDS' POOL: one record a building (the job writes exactly these); a
            // stand without its buildings takes none. No room: it is not asked now and the next walk asks again.
            StandAsk ask = s;
            const Page& pg = *it->second;
            const RtNode& n = pg.nodes[s.node];
            uint32_t nb = 0;
            if (s.blds && n.own0 < pg.blds.size() && n.own0 < pg.raw.size()) {
                nb = static_cast<uint32_t>((std::min)(size_t(n.ownN), (std::min)(pg.blds.size() - n.own0, pg.raw.size() - n.own0)));
            }
            ask.gpuN = m_stood.Valid() ? nb : 0u;
            if (ask.gpuN && !m_stoodFree.Take(ask.gpuN, ask.gpuOff)) {
                ++m_stoodFull;
                continue;
            }
            batch.push_back({it->second, ask});
            if (batch.size() >= size_t(kStandBatch)) {
                StandBatch(std::move(batch));
                batch.clear();
                ++jobs;
            }
        }
        if (!batch.empty() && jobs < kStandInFlight) StandBatch(std::move(batch));
        m_streamMs += std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - tStream).count();
        if (m_stoodFull && m_frame % 300 == 0) {
            uint32_t retiring = 0;
            for (const StoodRetired& r : m_stoodRetired) retiring += r.n;
            Log("[buildings] frame %llu: the stands' pool is full -- %u of %u records used, %u retiring; %llu stands have waited for room (layers.buildings.standMB)",
                static_cast<unsigned long long>(m_frame), m_stoodFree.used, m_stoodFree.total, retiring, static_cast<unsigned long long>(m_stoodFull));
        }
    }
    // 2. THE BUDGET, BY VALUE (2026-10-10, the owner's distinction: a coarse page IS the far picture
    // and must stay while the fine ones fill in; a texture's fine tile supersedes its coarse one, a
    // building's fine page only adds). A page's VALUE is its width on screen were the eye to look
    // at it: its quads in metres over its distance, in pixels -- one law at every level, so a
    // coarse page near the horizon is worth thousands of pixels, a fine page under the eye
    // hundreds, a fine page fifty kilometres off a fraction. Over the budget (or short of slots
    // for the pages waiting to land) the page worth the FEWEST pixels goes first, and no page goes
    // to make room for one worth fewer than it: the coarse levels are so the floor, with no case
    // for them. The walk's want order (pixels on screen, largest first) and this agree.
    auto slotsShort = [this] {
        size_t want = 0, coming = m_freeSlots.size();
        for (const auto& w : m_roomWait) want += w.second->slotUsed.size();
        for (const SlotsRetired& r : m_slotsRetired) coming += r.slots.size();
        return want > coming;
    };
    if (!m_pages.empty() && (m_pageBytes > m_pageBudget || slotsShort()) && ctx.camera && ctx.height) {
        const double eye[3] = {ctx.camera->px, ctx.camera->py, ctx.camera->pz};
        const double pixAng = double(ctx.camera->fovY) / double(ctx.height);
        auto value = [&](const PageKey& k) {
            const int L = std::get<0>(k);
            const double Q = lod::QuadDeg(L), PY = lod::PageY(L);
            const double lat = (std::get<2>(k) + 0.5) * PY * Q, lon = (std::get<1>(k) + 0.5) * lod::kPageX * Q;
            double p[3];
            m_place(lat, lon, 0.0, p);
            const double d = std::sqrt((p[0] - eye[0]) * (p[0] - eye[0]) + (p[1] - eye[1]) * (p[1] - eye[1]) +
                                       (p[2] - eye[2]) * (p[2] - eye[2]));
            const double widthM = (std::max)(lod::kPageX * Q, PY * Q) * lod::kMetresPerDeg * std::cos(lat * 3.14159265358979323846 / 180.0);
            return widthM / (std::max)(d - 0.5 * widthM, 1.0) / pixAng;
        };
        // What the room is for: the least valuable of the pages landing or wanted and not loaded.
        double need = 1e300;
        for (const auto& w : m_roomWait) need = (std::min)(need, value(w.first));
        for (const PageKey& k : m_pagePending) need = (std::min)(need, value(k));
        for (const auto& [px, k] : wants) {
            if (!m_pages.count(k) && !m_pagePending.count(k)) need = (std::min)(need, value(k));
        }
        std::vector<std::pair<double, PageKey>> byValue;
        byValue.reserve(m_pages.size());
        for (const auto& [k, pg] : m_pages) byValue.push_back({value(k), k});
        std::sort(byValue.begin(), byValue.end());
        uint32_t gone = 0, goneByLevel[lod::kLevels] = {};
        double lowestKept = 1e300, highestGone = 0.0;
        for (const auto& [v, k] : byValue) {
            if (!(m_pageBytes > m_pageBudget || slotsShort())) break;
            if (v >= need) break;   // the room would be made for a page worth less: over budget stands
            if (DrawnUses(k)) continue;   // an eye's drawn walk reads its shapes
            auto it = m_pages.find(k);
            if (!it->second->slots.empty()) m_slotsRetired.push_back({it->second->slots, m_frame, it->second});
            if (it->second->stood) {   // its stands' records go back to the pool when nothing holds them
                for (size_t i = 0; i < it->second->nodes.size(); ++i) {
                    const std::shared_ptr<const Stood> sp = it->second->stood[i].load(std::memory_order_acquire);
                    if (sp && sp->gpuN) m_stoodRetired.push_back({sp->gpuOff, sp->gpuN, m_frame, sp});
                }
            }
            m_pageBytes -= it->second->bytes;
            m_pageUsed.erase(k);
            m_pages.erase(it);
            ++m_pagesVersion;
            ++gone;
            ++goneByLevel[std::get<0>(k) - lod::kLmin];
            highestGone = (std::max)(highestGone, v);
        }
        for (const auto& [v, k] : byValue) {
            if (m_pages.count(k)) lowestKept = (std::min)(lowestKept, v);
        }
        if (gone && (m_evictions++ < 8 || m_evictions % 50 == 0)) {
            std::string per;
            for (int L = lod::kLmin; L <= lod::kLmax; ++L) {
                if (goneByLevel[L - lod::kLmin]) per += " L" + std::to_string(L) + ":" + std::to_string(goneByLevel[L - lod::kLmin]);
            }
            Log("[buildings] frame %llu: %u pages let go (%s) worth up to %.1f px, for pages worth %.1f px and more; %zu kept "
                "(%.0f MB of %.0f), the least worth %.1f px",
                static_cast<unsigned long long>(m_frame), gone, per.c_str(), highestGone, need >= 1e300 ? 0.0 : need,
                m_pages.size(), m_pageBytes / 1048576.0, m_pageBudget / 1048576.0, lowestKept >= 1e300 ? 0.0 : lowestKept);
        }
    }
    // 3. The first eye's walks -- its own world and each window world it reaches.
    WalkEye(ctx, 0);
}

void BuildingLayer::WalkEye(const FrameContext& ctx, uint32_t eye) {
    if (!ctx.camera || eye >= kEyes) return;
    const double at[3] = {ctx.camera->px, ctx.camera->py, ctx.camera->pz};
    const DirectX::XMFLOAT3 f3 = ctx.camera->Forward();
    const double fwd[3] = {f3.x, f3.y, f3.z};
    double lat = 0.0, lon = 0.0, h = 0.0;
    m_locate(at, lat, lon, h);
    Walk(ctx, WalkerOf(eye, 0), at, fwd, lat, lon, h);
    for (uint32_t k = 0; k + 1 < kWorlds; ++k) {
        Eye& e = m_eyes[WalkerOf(eye, k + 1)];
        if (k >= m_worlds[eye].size()) {   // no such world this frame: what it drew is let go
            if (e.drawn.vb.Valid()) m_retired.push_back({std::move(e.drawn.vb), m_frame, 2});
            e.drawn = Drawn{};
            e.drawnUsed.clear();
            e.drawnHold.clear();
            e.wants.clear();
            e.stands.clear();
            e.walkEye[0] = e.walkEye[1] = e.walkEye[2] = 1e30;
            continue;
        }
        const WorldEye& w = m_worlds[eye][k];
        m_locate(w.cam, lat, lon, h);
        Walk(ctx, WalkerOf(eye, k + 1), w.cam, w.fwd, lat, lon, h);
    }
}

std::vector<std::pair<double, BuildingLayer::PageKey>> BuildingLayer::Wants() const {
    std::vector<std::pair<double, PageKey>> out;
    std::map<PageKey, double> best;
    for (const Eye& e : m_eyes) {
        for (const auto& [px, k] : e.wants) {
            auto it = best.find(k);
            if (it == best.end()) {
                best.emplace(k, px);
                out.push_back({px, k});
            } else if (px > it->second) {
                it->second = px;
                for (auto& w : out) if (w.second == k) w.first = px;
            }
        }
    }
    return out;
}

bool BuildingLayer::DrawnUses(const PageKey& k) const {
    for (const Eye& e : m_eyes) if (e.drawnUsed.count(k)) return true;
    return false;
}

void BuildingLayer::SimulateEye(const FrameContext& ctx) {
    if (!m_pso || !m_stack || !ctx.camera || !ctx.gpu || !m_lod || !m_lod->Valid() || !ctx.height) return;
    const uint32_t eye = ctx.viewIndex < kEyes ? ctx.viewIndex : 0;
    if (eye == 0) return;   // the first eye walks in Simulate
    WalkEye(ctx, eye);
}

void BuildingLayer::Walk(const FrameContext& ctx, uint32_t eyeIndex, const double eye[3], const double fwd[3], double latDeg, double lonDeg, double h) {
    Eye& E = m_eyes[eyeIndex];
    if (m_shared->walking[eyeIndex].load() || !ctx.height) return;
    const double moved = std::sqrt((eye[0] - E.walkEye[0]) * (eye[0] - E.walkEye[0]) + (eye[1] - E.walkEye[1]) * (eye[1] - E.walkEye[1]) +
                                   (eye[2] - E.walkEye[2]) * (eye[2] - E.walkEye[2]));
    const double turned = fwd[0] * E.walkFwd[0] + fwd[1] * E.walkFwd[1] + fwd[2] * E.walkFwd[2];
    if (moved < (std::max)(1.0, 0.02 * (std::max)(h, 0.0)) && turned > 0.99998 && E.walkCells == m_cellsVersion &&
        E.walkPages == m_pagesVersion) {
        return;
    }
    for (int i = 0; i < 3; ++i) {
        E.walkEye[i] = eye[i];
        E.walkFwd[i] = fwd[i];
    }
    E.walkCells = m_cellsVersion;
    E.walkPages = m_pagesVersion;
    struct In {
        double eye[3], fwd[3], lat, lon, h, pixAng, halfDiag, cosHalf, sinHalf, tauB, tauS, tauP;
        double extinct = 0.0;   // the air's extinction (1/m) at the path's least dense point: T(r) = exp(-extinct r)
        double org[3];   // the walk's origin (the cloud's, at the eye), flat frame: what the instances are about
        std::shared_ptr<const Occluder> occ;   // the eye's farthest depth by cell, or none
        Motor pull;                            // a window world's instances into the true frame (identity for the own)
        bool pulled = false;
        struct KeyHash {
            size_t operator()(const PageKey& k) const {
                return std::hash<uint64_t>()((uint64_t(uint32_t(std::get<0>(k) + 64)) << 56) ^
                                             (uint64_t(uint32_t(std::get<1>(k))) << 28) ^ uint64_t(uint32_t(std::get<2>(k))));
            }
        };
        std::unordered_map<PageKey, const Page*, KeyHash> pages;
        std::vector<std::shared_ptr<const Page>> hold;   // the walk's references
        bool pool = false;                               // the stands' pool stands: nodes can be named
        std::set<Key> cells;
        uint64_t cellsVersion;
    } in;
    for (int i = 0; i < 3; ++i) {
        in.eye[i] = eye[i];
        in.fwd[i] = fwd[i];
    }
    {   // THE ORIGIN: the cloud's point at the eye (its shell at the eye's altitude), with hysteresis.
        constexpr double kDeg = 3.14159265358979323846 / 180.0, kR = 6371008.8;
        const double d[3] = {std::cos(latDeg * kDeg) * std::cos(lonDeg * kDeg), std::sin(latDeg * kDeg), std::cos(latDeg * kDeg) * std::sin(lonDeg * kDeg)};
        const double P[3] = {d[0] * (kR + h), d[1] * (kR + h), d[2] * (kR + h)};
        OriginLattice L;
        L.R = kR;
        const OriginLattice::Origin o = L.Pick(P, E.walkOriginSet ? &E.walkOrigin : nullptr);
        if (!E.walkOriginSet || !o.Same(E.walkOrigin)) {
            Log("[buildings] eye %u world %u: the walk's origin is the cloud's point %lld on shell %d (%.5f N %.5f E, %.0f m up): the eye stands %.1f km from it%s",
                eyeIndex / kWorlds, eyeIndex % kWorlds, static_cast<long long>(o.index), o.shell, o.anchor.latDeg, o.anchor.lonDeg, o.r - kR,
                std::sqrt((P[0] - o.c[0]) * (P[0] - o.c[0]) + (P[1] - o.c[1]) * (P[1] - o.c[1]) + (P[2] - o.c[2]) * (P[2] - o.c[2])) / 1000.0,
                E.walkOriginSet ? " (a new cell)" : "");
            E.walkOrigin = o;
            E.walkOriginSet = true;
        }
        m_place(E.walkOrigin.anchor.latDeg, E.walkOrigin.anchor.lonDeg, E.walkOrigin.r - kR, in.org);
    }
    in.lat = latDeg;
    in.lon = lonDeg;
    in.h = h;
    in.pixAng = double(ctx.camera->fovY) / double(ctx.height);
    const double aspect = ctx.width ? double(ctx.width) / double(ctx.height) : 16.0 / 9.0;
    in.halfDiag = std::atan(std::tan(0.5 * double(ctx.camera->fovY)) * std::sqrt(1.0 + aspect * aspect)) +
                  m_marginDeg * 3.14159265358979323846 / 180.0;   // the cone's margin: a turn finds its buildings asked
    in.occ = m_occ[eyeIndex / kWorlds];
    if (eyeIndex % kWorlds) {
        const uint32_t k = eyeIndex % kWorlds - 1;
        if (k < m_worlds[eyeIndex / kWorlds].size()) {
            in.pull = m_worlds[eyeIndex / kWorlds][k].pull;
            in.pulled = true;
        }
    }
    in.pool = m_stood.Valid() && m_clsPso != nullptr;
    in.cosHalf = std::cos(in.halfDiag);
    in.sinHalf = std::sin(in.halfDiag);
    in.tauB = m_lodPixels;
    {   // THE AIR (SetAir): Rayleigh + aerosol extinction at 550 nm, thinned by their scale heights to
        // the eye's altitude -- the least dense point of a path from the eye to the ground, so T is
        // the most the air could leave, and the law cuts only what is surely gone.
        const double hA = (std::max)(h, 0.0);
        const double rayH = (std::max)(double(m_air.row[0][3]), 1.0), mieH = (std::max)(double(m_air.row[1][3]), 1.0);
        in.extinct = double(m_air.row[0][1]) * std::exp(-hA / rayH) + double(m_air.row[2][1]) * std::exp(-hA / mieH);
    }
    in.tauS = m_shapePixels;   // its own form from here, its moment box below
    in.tauP = m_boxPixels;     // its box from here, a point below
    in.pages.reserve(m_pages.size());
    in.hold.reserve(m_pages.size());
    for (const auto& [k, p] : m_pages) {
        in.pages.emplace(k, p.get());
        in.hold.push_back(p);
    }
    for (const auto& [k, c] : m_cells) in.cells.insert(k);
    in.cellsVersion = m_cellsVersion;
    m_shared->walking[eyeIndex].store(1);
    Threads().Submit(Lane::Io, "buildings.walk", [shared = m_shared, lodf = m_lod, gpu = ctx.gpu, eyeIndex, in = std::move(in)] {
        const auto t0 = std::chrono::steady_clock::now();
        Walked w;
        w.eye = eyeIndex;
        w.cellsVersion = in.cellsVersion;
        for (int i = 0; i < 3; ++i) w.origin[i] = in.org[i];   // the instances are about the cloud's point
        // THE WALK IN PARALLEL: the coarse levels walked here down to kSplit, the subtrees below it
        // shared out to threads, each into its own accumulator; merged after. A pure function of `in`.
        struct Acc {
            std::vector<NodeEntry> nodes;                        // the nodes named for the GPU (one range each)
            std::vector<std::shared_ptr<const Stood>> stands;   // their stands, held until the pass is recorded
            uint64_t cap = 0;                                   // the records those ranges send
            std::set<PageKey> used;
            std::map<PageKey, double> wants;
            std::vector<StandAsk> asks;   // the stands asked for (each node once: the tree is visited once)
            uint64_t visited = 0, singles = 0, hidden = 0, aired = 0;
            PageKey lastUsed{INT32_MIN, 0, 0};
        };
        struct Frontier {
            int L;
            const Page* pg;
            const RtNode* n;
        };
        constexpr int kSplit = -2;   // quads of 0.0125 deg (1.4 km): thousands of subtrees over a city
        std::vector<Frontier> frontier;
        constexpr double kR = 6371008.8;
        // The horizon, and what stands above it a little beyond.
        const double dh = std::sqrt((std::max)(in.h, 0.0) * (2.0 * kR + (std::max)(in.h, 0.0))) + 120000.0;
        auto pageOf = [](int L, int x, int y) { return PageKey{L, FloorDivI(x, lod::kPageX), FloorDivI(y, lod::PageY(L))}; };
        auto loaded = [&](const PageKey& k) -> const Page* {
            auto it = in.pages.find(k);
            return it == in.pages.end() ? nullptr : it->second;
        };
        // THE NODE NAMED (BuildingsClassify.hlsl): one entry for the GPU -- the stand's centre about
        // the walk's origin (the difference in doubles, cast once), its range in the stands' pool,
        // how many of its buildings to classify and the air's factor on every threshold at this
        // node. The form of each building (none, a point, a box, its own shape) is the pass's
        // verdict, by the law that stood here: the walk's cost is the nodes it names, not the
        // buildings they hold. NO FOLDS (the owner, 2026-10-15): a node's mass as one box stood in
        // for a page on its way and for buildings not yet stood, and on a fast zoom-out those
        // stand-ins were the folds of 10 km nodes, slabs over the land that shrank as the pages
        // landed. Every building is its own point, hexahedron or mesh, and a page that has not
        // landed shows nothing until it does.
        auto name = [&](Acc& acc, const std::shared_ptr<const Stood>& sp, uint32_t count, double air) {
            if (!count || acc.cap + count > kMaxRecords) return;
            NodeEntry e{};
            for (int i = 0; i < 3; ++i) e.off[i] = static_cast<float>(sp->c[i] - in.org[i]);
            e.poolOff = sp->gpuOff;
            e.count = count;
            e.air = static_cast<float>(air);
            acc.nodes.push_back(e);
            acc.stands.push_back(sp);
            acc.cap += count;
            acc.singles += count;
        };
        std::function<void(Acc&, bool, int, const Page&, const RtNode&)> visit = [&](Acc& acc, bool split, int L,
                                                                                       const Page& pg, const RtNode& n) {
            if (split && L == kSplit) {
                frontier.push_back({L, &pg, &n});
                return;
            }
            ++acc.visited;
            // ITS STAND, or none yet: the node's centre on the ground and its folds come from the
            // stand; without one the node is tested at sea level with the worst of elevations.
            const uint32_t ni = static_cast<uint32_t>(&n - pg.nodes.data());
            const std::shared_ptr<const Stood> sp = pg.stood ? pg.stood[ni].load(std::memory_order_acquire) : nullptr;
            const double* nc = sp ? sp->c : n.c;
            const double nrad = sp ? double(n.rad) : double(n.rad) + kUnstoodM;
            double v[3], dist = 0.0;
            for (int i = 0; i < 3; ++i) {
                v[i] = nc[i] - in.eye[i];
                dist += v[i] * v[i];
            }
            dist = std::sqrt(dist);
            if (dist - nrad > dh) return;   // under the horizon
            if (dist > nrad) {              // outside the view's cone
                // Outside when the angle to it, less its sphere's half-angle b, passes the half-diagonal:
                // cos(angle) < cos(halfDiag + b), formed without a trig call (sin b = rad / dist).
                const double ca = (v[0] * in.fwd[0] + v[1] * in.fwd[1] + v[2] * in.fwd[2]) / dist;
                const double sb = nrad / dist, cb = std::sqrt((std::max)(0.0, 1.0 - sb * sb));
                if (ca < in.cosHalf * cb - in.sinHalf * sb) return;
            }
            // BEHIND WHAT THE FRAME DREW: the node's sphere on the occluder's grid (in the true frame,
            // by the camera the grid was drawn by); hidden when its nearest point lies beyond the
            // farthest surface of every cell it covers. A sphere reaching past the grid's edge, or
            // the near plane, is not judged.
            if (in.occ && in.occ->w) {
                const Occluder& oc = *in.occ;
                double p[3] = {nc[0], nc[1], nc[2]};
                if (in.pulled) in.pull.TransformPoint(p[0], p[1], p[2]);
                const double q[3] = {p[0] - oc.eye[0], p[1] - oc.eye[1], p[2] - oc.eye[2]};
                const double dz = q[0] * oc.fwd[0] + q[1] * oc.fwd[1] + q[2] * oc.fwd[2];
                if (dz - nrad > oc.nearZ) {
                    const double sx = (q[0] * oc.right[0] + q[1] * oc.right[1] + q[2] * oc.right[2]) / dz;
                    const double sy = (q[0] * oc.up[0] + q[1] * oc.up[1] + q[2] * oc.up[2]) / dz;
                    const double hs = 1.2 * nrad / (dz - nrad);   // the sphere's half-size in tan units, a little over
                    if (sx - hs > -oc.tanX && sx + hs < oc.tanX && sy - hs > -oc.tanY && sy + hs < oc.tanY) {
                        const int cx0 = (std::max)(0, int(std::floor((sx - hs) / oc.tanX * 0.5 * oc.w + 0.5 * oc.w)));
                        const int cx1 = (std::min)(int(oc.w) - 1, int(std::floor((sx + hs) / oc.tanX * 0.5 * oc.w + 0.5 * oc.w)));
                        const int cy0 = (std::max)(0, int(std::floor((-sy - hs) / oc.tanY * 0.5 * oc.h + 0.5 * oc.h)));
                        const int cy1 = (std::min)(int(oc.h) - 1, int(std::floor((-sy + hs) / oc.tanY * 0.5 * oc.h + 0.5 * oc.h)));
                        const double nearest = dz - nrad;
                        bool hidden = true;
                        for (int cy = cy0; cy <= cy1 && hidden; ++cy) {
                            for (int cx = cx0; cx <= cx1; ++cx) {
                                if (double(oc.dist[size_t(cy) * oc.w + cx]) > nearest) { hidden = false; break; }
                            }
                        }
                        if (hidden) {
                            ++acc.hidden;
                            return;
                        }
                    }
                }
            }
            // THE RANK BOUND: a building filed here has radius at most Q/4 (BuildingLod's LevelOf), so
            // nothing here or below is wider than half the quad: under lodPixels, the walk stops.
            const double dn = (std::max)(dist - nrad, 1.0);
            const double px = lod::QuadDeg(L) * lod::kMetresPerDeg / (dn * in.pixAng);
            // THE THRESHOLD THROUGH THE AIR (SetAir): what the picture holds of a building at this
            // distance is its contrast times T(dn), so a pixel of it is lodPixels / T pixels of size.
            const double tau = in.tauB * std::exp(in.extinct * dn);
            if (L < lod::kLmax && 0.5 * px < tau) {
                if (0.5 * px >= in.tauB) ++acc.aired;
                return;
            }
            // THE STAND IT NEEDS: its buildings once the largest of them is a pixel here, at the grain
            // of two pixels of ground at its distance (a metre at least); asked when there is none,
            // when the folds alone stand where buildings are wanted, or when a stand twice as fine
            // is earned. What stands meanwhile is what the last stand made.
            const bool needBlds = n.ownN && n.own0 < pg.blds.size() && 2.0 * pg.blds[n.own0].rho / (dn * in.pixAng) >= tau;
            const double grain = (std::max)(1.0, dn * in.pixAng * 2.0);
            if (!sp || (needBlds && !sp->withBlds) || double(sp->grainM) > 2.0 * grain) {
                // A node is visited once per walk (the tree), so the asks are a list, not a map:
                // keyed per node they were a 100k-entry merge, 60 ms of an 80 ms walk (measured).
                StandAsk ask;
                ask.page = pageOf(L, n.x, n.y);
                ask.node = ni;
                ask.blds = needBlds;
                ask.grainM = grain;
                ask.px = px;
                acc.asks.push_back(ask);
            }
            // THE NODE'S RANGE: its buildings within the node's size bound -- the page ranks them
            // largest first, so the bound is one partition point (the walk's O(log) share of the
            // per-building law; the rest is the pass's at each building's own distance).
            uint32_t nb = 0;
            if (sp && sp->withBlds) {
                const size_t end = (std::min)(size_t(n.own0) + (std::min)(n.ownN, sp->gpuN), pg.blds.size());
                if (n.own0 < end) {
                    const auto b0 = pg.blds.begin() + n.own0, b1 = pg.blds.begin() + end;
                    nb = static_cast<uint32_t>(std::partition_point(b0, b1, [&](const RtBld& b) { return 2.0 * b.rho / (dn * in.pixAng) >= tau; }) - b0);
                }
            }
            // The children's pages: the loaded ones are walked; the missing ones are asked for, by
            // the quad's size on screen, and their buildings appear when they land.
            // The four children share one page (a page is kPageX wide and PageY(L) tall, both even).
            bool recurse = false;
            const Page* cp = nullptr;
            if (L > lod::kLmin && n.kids) {
                const PageKey k = pageOf(L - 1, 2 * n.x, 2 * n.y);
                cp = loaded(k);
                if (!cp) {
                    if (lodf->Find(L - 1, std::get<1>(k), std::get<2>(k))) {
                        double& want = acc.wants[k];
                        want = (std::max)(want, px);   // (nothing stands in: its buildings come when it lands)
                    }
                } else {
                    if (k != acc.lastUsed) {
                        acc.used.insert(k);
                        acc.lastUsed = k;
                    }
                    recurse = true;
                    if (L - 1 == lod::kLmin) {   // the finest children: their largest, at this node's nearest distance
                        auto pm = cp->parentRho.find(Page::QuadKey(n.x, n.y));
                        if (pm == cp->parentRho.end() || 2.0 * pm->second / (dn * in.pixAng) < tau) recurse = false;
                    }
                }
            }
            if (sp && nb && in.pool) name(acc, sp, nb, tau / in.tauB);
            if (!recurse) return;
            for (int j = 0; j < 2; ++j) {
                for (int i = 0; i < 2; ++i) {
                    auto it = cp->at.find(Page::QuadKey(2 * n.x + i, 2 * n.y + j));
                    if (it != cp->at.end()) visit(acc, split, L - 1, *cp, cp->nodes[it->second]);
                }
            }
        };
        Acc top;
        // THE ROOTS: the coarsest quads within the horizon.
        {
            const int L = lod::kLmax;
            const double Q = lod::QuadDeg(L), span = dh / lod::kMetresPerDeg;
            constexpr double kDeg = 3.14159265358979323846 / 180.0;
            const double lat0 = (std::max)(-90.0, in.lat - span), lat1 = (std::min)(90.0, in.lat + span);
            const double cl = (std::max)(0.05, std::cos((std::min)(89.0, std::abs(in.lat) + span) * kDeg));
            const double lspan = (std::min)(180.0, span / cl);
            const int y0 = static_cast<int>(std::floor(lat0 / Q)), y1 = static_cast<int>(std::floor(lat1 / Q));
            const int x0 = static_cast<int>(std::floor((in.lon - lspan) / Q)), x1 = static_cast<int>(std::floor((in.lon + lspan) / Q));
            const int wrap = static_cast<int>(std::lround(360.0 / Q));
            for (int y = y0; y <= y1; ++y) {
                for (int xx = x0; xx <= x1; ++xx) {
                    // Longitude wraps at the date line: the quad's own index.
                    const int x = ((xx + wrap / 2) % wrap + wrap) % wrap - wrap / 2;
                    const PageKey k = pageOf(L, x, y);
                    if (!lodf->Find(L, std::get<1>(k), std::get<2>(k))) continue;
                    const Page* p = loaded(k);
                    if (!p) {
                        top.wants[k] = 1e12;   // the roots before anything
                        continue;
                    }
                    top.used.insert(k);
                    auto it = p->at.find(Page::QuadKey(x, y));
                    if (it != p->at.end()) visit(top, true, L, *p, p->nodes[it->second]);
                }
            }
        }
        const auto tR = std::chrono::steady_clock::now();
        w.msRoots = std::chrono::duration<double, std::milli>(tR - t0).count();
        {   // the frontier's subtrees, drained by a few threads, each into its own accumulator
            const size_t nT = (std::min)(frontier.size(), size_t((std::max)(1u, (std::min)(8u, std::thread::hardware_concurrency() / 2))));
            std::vector<Acc> accs(nT);
            std::atomic<size_t> next{0};
            auto drain = [&](size_t t) {
                for (size_t i; (i = next.fetch_add(1)) < frontier.size();) {
                    const Frontier& f = frontier[i];
                    visit(accs[t], false, f.L, *f.pg, *f.n);
                }
            };
            std::vector<std::thread> threads;
            for (size_t t = 1; t < nT; ++t) threads.emplace_back(drain, t);
            if (nT) drain(0);
            for (std::thread& t : threads) t.join();
            w.msDrain = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - tR).count();
            w.threads = static_cast<uint32_t>(nT);
            w.subtrees = static_cast<uint32_t>(frontier.size());
            for (Acc& a : accs) {
                top.nodes.insert(top.nodes.end(), a.nodes.begin(), a.nodes.end());
                top.stands.insert(top.stands.end(), std::make_move_iterator(a.stands.begin()), std::make_move_iterator(a.stands.end()));
                top.cap += a.cap;
                top.used.insert(a.used.begin(), a.used.end());
                for (const auto& [k, px] : a.wants) {
                    double& want = top.wants[k];
                    want = (std::max)(want, px);
                }
                top.asks.insert(top.asks.end(), a.asks.begin(), a.asks.end());
                top.visited += a.visited;
                top.singles += a.singles;
                top.hidden += a.hidden;
                top.aired += a.aired;
            }
        }
        std::set<PageKey>& used = top.used;
        std::map<PageKey, double>& wants = top.wants;
        w.nodes = top.visited;
        w.singles = top.singles;
        w.hidden = top.hidden;
        w.aired = top.aired;
        const auto tV = std::chrono::steady_clock::now();
        w.msVisit = std::chrono::duration<double, std::milli>(tV - t0).count();
        // THE LIST: each node's first thread as a running sum (the pass finds a thread's node by it);
        // the eye about the origin for the pass's distances. No sort: every form is opaque and depth
        // tested, so the order the pass writes is as good as any.
        std::vector<NodeEntry>& nodes = top.nodes;
        uint32_t first = 0;
        for (NodeEntry& e : nodes) {
            e.first = first;
            first += e.count;
        }
        w.named = static_cast<uint32_t>(nodes.size());
        w.cap = first;
        w.cellsN = static_cast<uint32_t>(in.cells.size());
        w.stands = std::move(top.stands);
        for (int i = 0; i < 3; ++i) w.eo[i] = static_cast<float>(in.eye[i] - in.org[i]);
        w.pixAng = static_cast<float>(in.pixAng);
        const auto tS = std::chrono::steady_clock::now();
        w.msList = std::chrono::duration<double, std::milli>(tS - tV).count();
        if (w.cap && !shared->cancel.load()) {
            // Two buffers: the node list with the resident cells after it (upload heap: the pass reads
            // it as it is), and the walk's buffer the pass fills (the header, then cap records).
            const uint64_t listBytes = (uint64_t(nodes.size()) * sizeof(NodeEntry) + uint64_t(w.cellsN) * 16u + 15u) & ~uint64_t(15);
            const uint64_t outBytes = uint64_t(kWalkHeader) * 16u + uint64_t(w.cap) * sizeof(Box);
            {
                std::lock_guard<std::mutex> lk(shared->mx);
                auto best = shared->freeStaging.end();
                for (auto it = shared->freeStaging.begin(); it != shared->freeStaging.end(); ++it) {
                    if (it->size >= listBytes && (best == shared->freeStaging.end() || it->size < best->size)) best = it;
                }
                if (best != shared->freeStaging.end()) {
                    w.list = std::move(*best);
                    shared->freeStaging.erase(best);
                }
                auto bestOut = shared->freeBoxes.end();
                for (auto it = shared->freeBoxes.begin(); it != shared->freeBoxes.end(); ++it) {
                    if (it->b.size >= outBytes && (bestOut == shared->freeBoxes.end() || it->b.size < bestOut->b.size)) bestOut = it;
                }
                if (bestOut != shared->freeBoxes.end()) {
                    w.out = std::move(*bestOut);
                    shared->freeBoxes.erase(bestOut);
                }
            }
            if (!w.list.Valid()) w.list = gpu->CreateUploadBuffer(listBytes + listBytes / 4, L"buildings.walk.nodes");   // a little over, so the next walk fits
            uint8_t* dst = w.list.cpu;
            std::memcpy(dst, nodes.data(), nodes.size() * sizeof(NodeEntry));
            dst += nodes.size() * sizeof(NodeEntry);
            for (const Key& c : in.cells) {
                const uint32_t row[4] = {PackCell(c), 0u, 0u, 0u};
                std::memcpy(dst, row, 16);
                dst += 16;
            }
            if (!w.out.b.Valid()) w.out = ListBuf{gpu->CreateUavBuffer(outBytes + outBytes / 4, L"buildings.walk"), true};
        }
        w.msBuf = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - tS).count();
        for (const auto& [k, px] : wants) w.wants.push_back({px, k});
        w.asks = std::move(top.asks);
        auto byPx = [](const StandAsk& a, const StandAsk& b) { return a.px > b.px; };
        if (w.asks.size() > kStandAskMax) {   // the most worth on screen; the rest are asked again when these have landed
            std::nth_element(w.asks.begin(), w.asks.begin() + kStandAskMax, w.asks.end(), byPx);
            w.asks.resize(kStandAskMax);
        }
        std::sort(w.asks.begin(), w.asks.end(), byPx);
        // COARSE BEFORE FINE: the coarsest level first, the largest on screen first within it.
        std::sort(w.wants.begin(), w.wants.end(), [](const auto& a, const auto& b) {
            const int la = std::get<0>(a.second), lb = std::get<0>(b.second);
            return la != lb ? la > lb : a.first > b.first;
        });
        w.used.assign(used.begin(), used.end());
        w.hold = in.hold;
        w.ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
        std::lock_guard<std::mutex> lk(shared->mx);
        shared->walked.push_back(std::move(w));
        shared->walking[eyeIndex].store(0);
    });
}

void BuildingLayer::StandBox(const Place& place, double lat, double lon, double base, double zc, double hz,
                             double a1, double a2, double hd, RtBox& o, StoodFrame* frame) {
    if (!(hz > 0.0)) return;
    double p[3], e[3], n[3], u[3];
    PointFrame(place, lat, lon, base + zc, p, e, n, u);
    if (frame) {
        for (int i = 0; i < 3; ++i) {
            frame->g[i] = p[i] - u[i] * zc;
            frame->e[i] = static_cast<float>(e[i]);
            frame->n[i] = static_cast<float>(n[i]);
            frame->up[i] = static_cast<float>(u[i]);
        }
    }
    const double ch = std::cos(hd), sh = std::sin(hd);
    for (int i = 0; i < 3; ++i) {
        o.p[i] = p[i];
        o.u[i] = static_cast<float>((ch * e[i] + sh * n[i]) * a1);
        o.v[i] = static_cast<float>((-sh * e[i] + ch * n[i]) * a2);
        o.w[i] = static_cast<float>(u[i] * hz);
    }
    o.valid = true;
}

// THE STAND, on the pool: the node's folds and (asked) its buildings on the composed ground at the
// grain the walk earned; a ribbon's profile at the same grain, its heights handed back for the pool.
// Published whole by the job (an atomic store with release; the walk loads with acquire).
void BuildingLayer::StandBatch(std::vector<std::pair<std::shared_ptr<const Page>, StandAsk>> batch) {
    for (const auto& [page, ask] : batch) m_standPending.insert({ask.page, ask.node});
    m_shared->standInflight.fetch_add(1);
    Threads().Submit(Lane::Io, "buildings.stand", [shared = m_shared, place = m_place, ground = m_ground, batch = std::move(batch)] {
        // One ground field for the batch, at the finest grain any of its nodes earned (a batch is the
        // walk's neighbours in worth, so their grains are alike).
        double grainM = 1e300;
        for (const auto& [page, ask] : batch) grainM = (std::min)(grainM, ask.grainM);
        const std::unique_ptr<PlaceField> gnd = ground ? ground(grainM) : nullptr;
        auto gAt = [&](double la, double lo) { double v = 0.0; if (gnd) gnd->Read(la, lo, v); return v; };
        std::vector<Shared::StandDone> dones;
        dones.reserve(batch.size());
        for (const auto& [page, ask] : batch) {
            const auto t0 = std::chrono::steady_clock::now();
            auto s = std::make_shared<Stood>();
            s->grainM = static_cast<float>(grainM);
            s->withBlds = ask.blds;
            const RtNode& n = page->nodes[ask.node];
            const double Q = lod::QuadDeg(page->L);
            const double latc = (n.y + 0.5) * Q, lonc = (n.x + 0.5) * Q;
            place(latc, lonc, gAt(latc, lonc), s->c);
            Shared::StandDone done;
            // ITS RECORDS FOR THE GPU (StoodRec): its buildings, each about the stand's centre;
            // exactly the range StandBatch took (a building past it claims nothing).
            s->gpuOff = ask.gpuOff;
            s->gpuN = ask.gpuN;
            done.recs.assign(size_t(ask.gpuN) * sizeof(StoodRec), 0);
            auto recAt = [&](uint32_t i) -> StoodRec* {
                return i < ask.gpuN ? reinterpret_cast<StoodRec*>(done.recs.data() + size_t(i) * sizeof(StoodRec)) : nullptr;
            };
            if (ask.blds && !shared->cancel.load()) {
                constexpr double kDeg = 3.14159265358979323846 / 180.0, kR = 6371008.8;
                for (uint32_t k = n.own0; k < n.own0 + n.ownN && k < page->raw.size() && k < page->blds.size(); ++k) {
                    double lat, lon, zc, hz, a1, a2, hd;
                    int cx, cy;
                    LodUnpack(page->raw[k], lat, lon, zc, hz, a1, a2, hd, cx, cy);
                    const RtBld& b = page->blds[k];
                    RtBox box;
                    StoodFrame frame;
                    // THE GROUND BASE: a building stands on the LOWEST ground under its footprint -- its walls
                    // reach the ground on the downhill side, the uphill side is buried, the roof stays level.
                    double base = gAt(lat, lon);
                    ShapeView sv;
                    const bool shaped = b.shapeOff != UINT32_MAX && b.shapeOff < page->packed.size() &&
                                        ReadShape(page->packed.data() + b.shapeOff, page->packed.data() + page->packed.size(), sv);
                    if (shaped) {
                        const double mx = std::cos(lat * kDeg) * kR * kDeg, my = kR * kDeg;
                        if (sv.head.flags & shape::kRibbon) {
                            // A RIBBON RIDES THE GROUND PLUS ITS OWN PROFILE (compose/BuildingShape.h kDeck): each
                            // vertex's height over the centroid's ground, for the pool -- a deck's or a ramp's
                            // profile stands over the higher of the ground and the sea, the surface it is
                            // measured from; a road's (profile zero) is the ground itself.
                            std::vector<uint8_t> z(size_t(sv.head.nVerts) * 2);
                            const bool deck = (sv.head.flags & shape::kDeck) != 0 && sv.z;
                            // THE DECK STANDS AT ITS CLEARANCE OVER THE LOWEST SURFACE ITS WAY CROSSES
                            // (2026-10-12): the water of a river bridge, the cutting under an overpass --
                            // the lowest of max(ground, sea) along the ribbon -- and rides the ground
                            // wherever the ground is higher. Added to the ground under EACH vertex it stood
                            // 12 m over a 9 m bank: a 21 m deck with a cliff at either abutment, which the
                            // car met as a wall and the picture as a bridge in the sky.
                            std::vector<double> gv(sv.head.nVerts);
                            double gMin = 1e300;
                            for (uint32_t v = 0; v < sv.head.nVerts; ++v) {
                                const double x = sv.xy[2 * v] * double(sv.unit), y = sv.xy[2 * v + 1] * double(sv.unit);
                                gv[v] = gAt(lat + y / my, lon + x / mx);
                                gMin = (std::min)(gMin, (std::max)(gv[v], 0.0));
                            }
                            for (uint32_t v = 0; v < sv.head.nVerts; ++v) {
                                const double g = gv[v];
                                const double zp = deck ? double(sv.z[v]) * 0.1 : 0.0;
                                const double dz = (zp > 0.0 ? (std::max)(g, gMin + zp) : g) - base;
                                const int16_t q = static_cast<int16_t>(std::clamp(std::lround(dz * 10.0), -32767L, 32767L));
                                std::memcpy(&z[2 * v], &q, 2);
                            }
                            const uint32_t slot = b.shapeOff / kSlotBytes;
                            if (slot < page->slots.size()) {
                                done.zWrites.push_back({uint64_t(page->slots[slot]) * kSlotBytes + b.shapeOff % kSlotBytes + shape::ZOffset(sv.head), std::move(z)});
                            }
                        } else {
                            for (uint32_t v = 0; v < sv.head.nVerts; ++v) {
                                const double x = sv.xy[2 * v] * double(sv.unit), y = sv.xy[2 * v + 1] * double(sv.unit);
                                base = (std::min)(base, gAt(lat + y / my, lon + x / mx));
                            }
                        }
                    }
                    StandBox(place, lat, lon, base, zc, hz, a1, a2, hd, box, shaped ? &frame : nullptr);
                    if (StoodRec* r = recAt(k - n.own0)) {
                        // Its shape's pool address, as the page's slots place it (none: its box is drawn).
                        uint32_t addr = UINT32_MAX;
                        if (shaped && b.shapeOff / kSlotBytes < page->slots.size()) {
                            addr = static_cast<uint32_t>(uint64_t(page->slots[b.shapeOff / kSlotBytes]) * kSlotBytes + b.shapeOff % kSlotBytes);
                        }
                        RecordOf(box, s->c, box.valid ? b.rho : 0.0f, addr, static_cast<float>(hd), PackCell(b.cell), page->landed,
                                 static_cast<float>(zc), *r);
                        // THE ROTOR LAW, CHECKED ONCE (the first shaped buildings): the shape's ground
                        // frame must be the box's rotor with the heading taken back out -- what the pass
                        // computes -- or the footprints would stand turned.
                        static std::atomic<int> checked{0};
                        if (shaped && box.valid && checked.load() < 32 && checked.fetch_add(1) < 32) {
                            float qg[4], qp[4];
                            const double E[3] = {frame.e[0], frame.e[1], frame.e[2]}, N[3] = {frame.n[0], frame.n[1], frame.n[2]},
                                         U[3] = {frame.up[0], frame.up[1], frame.up[2]};
                            RotorOf(E, U, N, qg);
                            const float back[4] = {0.0f, std::sin(0.5f * float(hd)), 0.0f, std::cos(0.5f * float(hd))};
                            QMul(r->q, back, qp);
                            float dPlus = 0.0f, dMinus = 0.0f;
                            for (int i = 0; i < 4; ++i) {
                                dPlus = (std::max)(dPlus, std::fabs(qg[i] - qp[i]));
                                dMinus = (std::max)(dMinus, std::fabs(qg[i] + qp[i]));
                            }
                            if ((std::min)(dPlus, dMinus) > 1e-3f) {
                                Log("[buildings] THE SHAPE ROTOR DISAGREES: ground frame (%.4f %.4f %.4f %.4f) vs box x back (%.4f %.4f %.4f %.4f), heading %.3f",
                                    qg[0], qg[1], qg[2], qg[3], qp[0], qp[1], qp[2], qp[3], hd);
                            }
                        }
                    }
                    ++done.blds;
                }
            }
            done.stood = s;   // published by Simulate, with its records
            done.page = page;
            done.key = ask.page;
            done.node = ask.node;
            done.ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
            dones.push_back(std::move(done));
        }
        std::lock_guard<std::mutex> lk(shared->mx);
        for (auto& d : dones) shared->standDone.push_back(std::move(d));
        shared->standInflight.fetch_sub(1);
    });
}

void BuildingLayer::DecksNear(double latDeg, double lonDeg, double rM, std::vector<DeckStrip>& out) const {
    out.clear();
    constexpr double kDeg = 3.14159265358979323846 / 180.0, kR = 6371008.8;
    const double my = kR * kDeg, mx = std::cos(latDeg * kDeg) * kR * kDeg;
    const double r2 = rM * rM;
    for (const auto& [key, page] : m_pages) {
        if (!page || page->packed.empty()) continue;
        for (size_t k = 0; k < page->blds.size() && k < page->raw.size(); ++k) {
            const RtBld& b = page->blds[k];
            if (b.shapeOff == UINT32_MAX || b.shapeOff >= page->packed.size()) continue;
            ShapeView sv;
            if (!ReadShape(page->packed.data() + b.shapeOff, page->packed.data() + page->packed.size(), sv)) continue;
            if (!(sv.head.flags & shape::kRibbon) || !(sv.head.flags & shape::kDeck) || !sv.z || sv.head.nVerts < 2) continue;
            double lat, lon, zc, hz, a1, a2, hd;
            int cx, cy;
            LodUnpack(page->raw[k], lat, lon, zc, hz, a1, a2, hd, cx, cy);
            // Within reach: any vertex within rM of the place.
            bool within = false;
            DeckStrip strip;
            strip.widthM = sv.head.pad * 0.1;
            for (uint32_t v = 0; v < sv.head.nVerts; ++v) {
                const double x = sv.xy[2 * v] * double(sv.unit), y = sv.xy[2 * v + 1] * double(sv.unit);
                const double vla = lat + y / my, vlo = lon + x / mx;
                const double dx = (vlo - lonDeg) * mx, dz = (vla - latDeg) * my;
                if (dx * dx + dz * dz <= r2) within = true;
                strip.lat.push_back(vla);
                strip.lon.push_back(vlo);
                strip.over.push_back(double(sv.z[v]) * 0.1);
            }
            if (within) out.push_back(std::move(strip));
        }
    }
}

void BuildingLayer::Simulate(const FrameContext& ctx) {
    ++m_frame;
    // Buffers no frame in flight can still read.
    {
        std::lock_guard<std::mutex> lk(m_shared->mx);
        for (Retired& r : m_retired) {
            if (r.pool == 0 || m_frame - r.frame <= kRetireFrames || !r.buf.Valid()) continue;
            // The two largest are kept: a walk only ever needs one, and a small one would never fit.
            if (r.pool == 1) {
                std::vector<GpuBuffer>& list = m_shared->freeStaging;
                if (list.size() < 2) {
                    list.push_back(std::move(r.buf));
                } else {
                    auto least = std::min_element(list.begin(), list.end(),
                                                  [](const GpuBuffer& a, const GpuBuffer& b) { return a.size < b.size; });
                    if (least->size < r.buf.size) *least = std::move(r.buf);
                }
            } else {   // a walk's buffer: in kListRead since its pass
                std::vector<ListBuf>& list = m_shared->freeBoxes;
                if (list.size() < 2) {
                    list.push_back({std::move(r.buf), false});
                } else {
                    auto least = std::min_element(list.begin(), list.end(),
                                                  [](const ListBuf& a, const ListBuf& b) { return a.b.size < b.b.size; });
                    if (least->b.size < r.buf.size) *least = {std::move(r.buf), false};
                }
            }
        }
    }
    m_retired.erase(std::remove_if(m_retired.begin(), m_retired.end(),
                                   [this](const Retired& r) { return m_frame - r.frame > kRetireFrames; }),
                    m_retired.end());
    if (!m_pso || !m_stack || !ctx.camera || !ctx.gpu || !ctx.cmd) return;
    const double eye[3] = {ctx.camera->px, ctx.camera->py, ctx.camera->pz};
    double lat = 0.0, lon = 0.0, h = 0.0;
    m_locate(eye, lat, lon, h);
    {
        std::lock_guard<std::mutex> lk(m_shared->mx);
        m_shared->eyeLat = lat;
        m_shared->eyeLon = lon;
        m_shared->eyeH = h;
        m_shared->keepM = kKeep * m_radius;
    }
    {   // The instrument: where the layer believes the eye is, each time it enters another cell.
        const Key here{static_cast<int>(std::floor(lon / kCellDeg)), static_cast<int>(std::floor(lat / kCellDeg))};
        if (here != m_eyeCell) {
            m_eyeCell = here;
            Log("[buildings] frame %llu: eye at %.5f, %.5f, %.0f m (flat %.0f, %.0f, %.0f)",
                static_cast<unsigned long long>(m_frame), lat, lon, h, eye[0], eye[1], eye[2]);
        }
    }
    // Drop what has fallen behind kKeep x radius into the warm pool, and the pool's oldest out past
    // its budget: a freed buffer waits out the frames in flight.
    for (auto it = m_cells.begin(); it != m_cells.end();) {
        if (Reach(it->first, lat, lon, h) > kKeep * m_radius) {
            m_warmBytes += uint64_t(it->second.count) * sizeof(Vertex);
            const Key k = it->first;
            Cell cell = std::move(it->second);
            it = m_cells.erase(it);
            ++m_cellsVersion;
            m_warm[k] = {std::move(cell), m_frame, m_cellsVersion};
        } else {
            ++it;
        }
    }
    while (m_warmBytes > kWarmBytes && !m_warm.empty()) {
        // A cell still handing off (its boxes not yet on screen) is drawn: it is not dropped.
        auto old = m_warm.end();
        for (auto it = m_warm.begin(); it != m_warm.end(); ++it) {
            if (it->second.leftAt > m_eyes[WalkerOf(0, 0)].drawnCells) continue;
            if (old == m_warm.end() || it->second.frame < old->second.frame) old = it;
        }
        if (old == m_warm.end()) break;
        m_warmBytes -= uint64_t(old->second.cell.count) * sizeof(Vertex);
        if (old->second.cell.vb.Valid()) m_retired.push_back({std::move(old->second.cell.vb), m_frame});
        m_warm.erase(old);
    }
    // ONE SOURCE OF SOLIDS: a tree with shapes, drawn by the mesh pipeline, draws every building at
    // every distance in its own footprint -- the near cells would draw the same buildings again. They
    // stream only for a tree of boxes (GALOD03) or none, or without mesh shaders.
    const bool treeDrawsAll = m_lod && m_lod->Valid() && m_lod->HasShapes() && m_psoShape;
    if (!m_cellsSaid) {
        m_cellsSaid = true;
        Log("[buildings] near cells: %s", treeDrawsAll ? "none -- the tree's shapes draw every building, near and far"
                                                       : "streamed (the tree has no shapes, or there is no tree)");
    }
    Upload(ctx);
    if (!treeDrawsAll) Want(ctx.gpu, lat, lon, h);
    if (m_lod && m_lod->Valid() && ctx.height > 0) TreeFrame(ctx, lat, lon, h);
    // THE SUN MAP, after this frame's cells and walk have landed: what the eye draws this frame is
    // what casts this frame; recorded here, before any view, so the globe's surface reads it.
    RenderSunMap(ctx);
}

void BuildingLayer::DumpChunk(uint32_t k) const {
    // The instances are the GPU's (the classify pass made them from the stands' records); what a
    // lost device can be told is what each walker's drawn walk sent.
    for (uint32_t i = 0; i < kEyes * kWorlds; ++i) {
        const Eye& e = m_eyes[i];
        if (!e.drawn.vb.Valid()) continue;
        Log("[crash] buildings, dispatch %u of its scope: eye %u world %u's drawn walk %llu named %u nodes, %u records to classify, from %zu pages",
            k, i / kWorlds, i % kWorlds, static_cast<unsigned long long>(e.walks), e.drawn.named, e.drawn.cap, e.drawnHold.size());
    }
}

// THE PASS (shaders/BuildingsClassify.hlsl), recorded when a walk lands: four dispatches over the
// walk's buffer -- the header cleared, each record's form counted, the lists' bases and the draws'
// arguments written, the records written -- then the buffer left in its read states, and its first
// row copied back for the log line.
void BuildingLayer::Classify(const FrameContext& ctx, Walked& w) {
    GpuScope scope(ctx.prof, ctx.cmd->Native(), "buildings.classify");
    hal::CommandContext& cmd = *ctx.cmd;
    hal::Resource out = w.out.b.res.Get();
    cmd.Barrier(out, w.out.fresh ? D3D12_RESOURCE_STATE_COMMON : kListRead, D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
    w.out.fresh = false;
    struct {   // mirrors ClassifyCb
        float eye[4];
        float tau[4];
        uint32_t n[4];
        uint32_t pass[4];
    } cb{};
    for (int i = 0; i < 3; ++i) cb.eye[i] = w.eo[i];
    cb.eye[3] = w.pixAng;   // the walk's own: a pixel's angle from the eye the walk was asked from
    cb.tau[0] = static_cast<float>(m_lodPixels);
    cb.tau[1] = static_cast<float>(m_shapePixels);
    cb.tau[2] = static_cast<float>(m_boxPixels);
    cb.tau[3] = (m_psoShape && m_pool.Valid()) ? 1.0f : 0.0f;
    cb.n[0] = w.named;
    cb.n[1] = w.cap;
    cb.n[2] = w.named * 2u;   // the cells' first row, after the nodes
    cb.n[3] = w.cellsN;
    cmd.ComputeRoot(m_clsRs.Get());
    cmd.Pipeline(m_clsPso.Get());
    cmd.ComputeSrvAt(1, w.list.gpu);
    cmd.ComputeSrvAt(2, m_stood.gpu);
    cmd.ComputeUavAt(3, w.out.b.gpu);
    const uint32_t groups = (w.cap + 63u) / 64u;
    for (uint32_t pass = 0; pass < 4; ++pass) {
        cb.pass[0] = pass;
        cmd.ComputeConstants(0, cb);
        cmd.Dispatch((pass == 0u || pass == 2u) ? 1u : groups);
        cmd.UavBarrier(out);
    }
    cmd.Barrier(out, D3D12_RESOURCE_STATE_UNORDERED_ACCESS, kListRead);
    if (m_countsRb.Valid()) cmd.Native()->CopyBufferRegion(m_countsRb.res.Get(), uint64_t(m_countsNext) * 16u, out, 0, 16);
}

void BuildingLayer::DrawSolids(const FrameContext& ctx, uint32_t eyeIndex, uint32_t worlds, const Pipes& p, const char* tag) {
    GpuScope gscope(ctx.prof, ctx.cmd->Native(), tag);
    ctx.cmd->Pipeline(p.cells);
    ctx.cmd->Topology(D3D_PRIMITIVE_TOPOLOGY_TRIANGLELIST);
    // Mirrors Buildings.hlsl's cbuffer (priors 22).
    struct {
        float origin[4];   // xyz the cell's origin relative to the eye, w = brightness
        float time[4];     // x the layer's now (s), y the pages' fade-in (s)
        uint32_t mesh[4];  // a draw of instances: the first's row, how many, the world's depth, the pool's heap slot
        float world[4];    // the world's turn in the true frame (a rotor; identity for the eye's own)
    } cb{};
    cb.world[3] = 1.0f;
    // The resident cells, and the ones leaving whose boxes the drawn walk does not hold yet.
    std::vector<const Cell*> draw;
    for (const auto& [key, c] : m_cells) draw.push_back(&c);
    for (const auto& [key, wc] : m_warm) {
        if (wc.leftAt > m_eyes[WalkerOf(0, 0)].drawnCells) draw.push_back(&wc.cell);
    }
    {
    GpuScope cellScope(ctx.prof, ctx.cmd->Native(), "buildings.cells");
    for (const Cell* cp : draw) {
        const Cell& c = *cp;
        if (!c.count) continue;
        // THE BOUNDARY: the difference taken in doubles, then cast.
        cb.origin[0] = static_cast<float>(c.origin[0] - ctx.camera->px);
        cb.origin[1] = static_cast<float>(c.origin[1] - ctx.camera->py);
        cb.origin[2] = static_cast<float>(c.origin[2] - ctx.camera->pz);
        cb.origin[3] = 1.0f;
        ctx.cmd->GraphicsConstants(1, cb);
        ctx.cmd->GraphicsSrvAt(2, c.vb.gpu);
        ctx.cmd->Draw(c.count, 1, 0, 0);
    }
    }
    for (uint32_t world = 0; world < worlds; ++world) {
    const Eye& e = m_eyes[WalkerOf(eyeIndex, world)];
    const Drawn& m_drawn = e.drawn;
    if (!m_drawn.vb.Valid() || !m_sigMesh || !m_sigDraw) continue;
    if (world > 0 && world - 1 >= m_worlds[eyeIndex].size()) continue;
        // THE WORLD'S PLACE IN THE TRUE FRAME: the walk's origin pulled through the window, the eye off
    // it in doubles; its turn as the rotor the instances' own rotors are composed with.
    double org[3] = {m_drawn.origin[0], m_drawn.origin[1], m_drawn.origin[2]};
    if (world > 0) {
        const Motor& pull = m_worlds[eyeIndex][world - 1].pull;
        pull.TransformPoint(org[0], org[1], org[2]);
        double E[3] = {1.0, 0.0, 0.0}, U[3] = {0.0, 1.0, 0.0}, N[3] = {0.0, 0.0, 1.0};
        pull.TransformDir(E[0], E[1], E[2]);
        pull.TransformDir(U[0], U[1], U[2]);
        pull.TransformDir(N[0], N[1], N[2]);
        RotorOf(E, U, N, cb.world);
    } else {
        cb.world[0] = cb.world[1] = cb.world[2] = 0.0f;
        cb.world[3] = 1.0f;
    }
    cb.origin[0] = static_cast<float>(org[0] - ctx.camera->px);
    cb.origin[1] = static_cast<float>(org[1] - ctx.camera->py);
    cb.origin[2] = static_cast<float>(org[2] - ctx.camera->pz);
    cb.origin[3] = 1.0f;
    cb.time[0] = Now();
    cb.time[1] = kPageFadeS;
    // z: a pixel's angle (rad), the walk's own -- the ribbons' width on screen (Buildings.hlsl MsShape).
    cb.time[2] = static_cast<float>(double(ctx.camera->fovY) / double(ctx.height));
    cb.mesh[2] = world;   // the depth the pixels must reach through the windows
    // THE THREE FORMS, EACH BY ITS OWN COUNT IN THE BUFFER'S HEADER (Buildings.hlsl FormBase,
    // FormCount): the classify pass wrote the lists and the draws' arguments; ExecuteIndirect reads
    // them. The CPU binds the buffer and names the form; it never knows how many.
    cb.mesh[0] = cb.mesh[1] = 0;
    cb.mesh[3] = m_poolSrv;
    ctx.cmd->GraphicsConstants(1, cb);
    ctx.cmd->GraphicsSrvAt(2, m_drawn.vb.gpu);
    hal::Resource args = m_drawn.vb.res.Get();
    if (p.boxMesh || p.box) {
        // THE TREE'S BOXES: 8 vertices a box through the mesh stage (16 a group; the groups in two
        // dimensions past 65535), or 36 through the vertex stage.
        GpuScope boxScope(ctx.prof, ctx.cmd->Native(), "buildings.boxes");
        if (p.boxMesh) {
            ctx.cmd->Pipeline(p.boxMesh);
            ctx.cmd->ExecuteIndirect(m_sigMesh.Get(), args, 16);
        } else {
            ctx.cmd->Pipeline(p.box);
            ctx.cmd->Topology(D3D_PRIMITIVE_TOPOLOGY_TRIANGLELIST);
            ctx.cmd->ExecuteIndirect(m_sigDraw.Get(), args, 64);
        }
    }
    if (p.shape && m_pool.Valid()) {
        // THE SHAPES: an amplification group a kAsInsts instances (Buildings.hlsl AsShape) cuts their
        // edges and roofs into full mesh groups.
        GpuScope shapeScope(ctx.prof, ctx.cmd->Native(), "buildings.shapes");
        ctx.cmd->Pipeline(p.shape);
        ctx.cmd->ExecuteIndirect(m_sigMesh.Get(), args, 32);
    }
    if (p.point) {
        // THE POINTS: the buildings under boxPixels, one vertex each through the vertex stage, lit by
        // their rotors (Buildings.hlsl VsPoint). The sun's pass draws none: a thing under two pixels
        // casts nothing a texel could hold.
        GpuScope ptScope(ctx.prof, ctx.cmd->Native(), "buildings.points");
        ctx.cmd->Pipeline(p.point);
        ctx.cmd->Topology(D3D_PRIMITIVE_TOPOLOGY_POINTLIST);
        ctx.cmd->ExecuteIndirect(m_sigDraw.Get(), args, 48);
    }
    }
}

void BuildingLayer::Render(const FrameContext& ctx) {
    // THIS VIEW'S EYE: its own walk's boxes (the minimap's from its camera), the near cells the first's,
    // and each window world it reaches, drawn pulled through the window.
    const uint32_t eyeIndex = ctx.viewIndex < kEyes ? ctx.viewIndex : 0;
    bool any = !m_cells.empty() || !m_warm.empty();
    for (uint32_t k = 0; k < kWorlds; ++k) any = any || m_eyes[WalkerOf(eyeIndex, k)].drawn.vb.Valid();
    if (!m_pso || !any || !ctx.camera) return;
    PixScope scope(ctx.cmd->Native(), "buildings (solids -> prisms, cell origins about the eye)");
    Pipes p;
    p.cells = m_pso.Get();
    p.box = m_psoBox.Get();
    p.boxMesh = m_psoBoxMesh.Get();
    p.shape = m_psoShape.Get();
    p.point = m_psoPoint.Get();
    DrawSolids(ctx, eyeIndex, kWorlds, p, "buildings");
}

// ================================================================================================
//  THE SUN MAP (scene render.buildingShadowM; shaders/TerrainRay.hlsli SunMapClear).
//
//  The cast is an orthographic camera standing on the sun's side of the focus, looking down the
//  sun: its view the sun's own axes (right = up-hint x down-sun, as the frame's camera builds its
//  basis), its projection a cube `side` metres on every edge, depth 1 at the sun's face and 0 at the
//  far face -- the frame's reversed convention, so the buildings' pipelines test GREATER unchanged
//  and the clear (0) is "nothing". One matrix, DirectXMath's as the frame's gViewProj is, stored
//  row-major as Common.hlsli reads it; the receivers read their place on the map from the same
//  bytes, so the cast and the lookup cannot disagree about where a texel is. The centre is snapped
//  to the texel grid in the sun's own axes: an eye that drifts within a texel moves no shadow.
// ================================================================================================
void BuildingLayer::PlanSunMap(const Camera& cam, const float sunDir[3], double sideM, const double focus[3], ComposedSurfaceCb& cb) {
    using namespace DirectX;
    const bool stood = m_sun.on;   // a centre stands from the frame before
    m_sun.on = false;
    cb.sunMapU[0] = UINT32_MAX;
    cb.sunMapU[1] = kSunTexels;
    cb.sunMapU[2] = cb.sunMapU[3] = 0;
    cb.shadow[3] = 0.0f;
    memset(cb.sunVP, 0, sizeof(cb.sunVP));
    if (!m_sunPlanSaid) {   // the instrument: once, why the map stands or does not
        m_sunPlanSaid = true;
        Log("[buildings] sun map planned: side %.0f m, srv %u, pipeline %d, focus %.0f %.0f %.0f (eye-relative), sun %.3f %.3f %.3f",
            sideM, m_sunSrv, m_psoSun ? 1 : 0, focus[0] - cam.px, focus[1] - cam.py, focus[2] - cam.pz, sunDir[0], sunDir[1], sunDir[2]);
    }
    if (sideM <= 0.0 || m_sunSrv == UINT32_MAX || !m_psoSun) return;
    // The sun's axes, in doubles: down-sun forward, the world up as the hint (north where the sun
    // stands near the zenith), right = hint x forward (DirectXMath's LookToLH law), up = forward x right.
    double S[3] = {sunDir[0], sunDir[1], sunDir[2]};
    const double sl = std::sqrt(S[0] * S[0] + S[1] * S[1] + S[2] * S[2]);
    if (sl < 1e-9) return;
    for (double& v : S) v /= sl;
    const double F[3] = {-S[0], -S[1], -S[2]};
    const bool steep = std::abs(F[1]) > 0.99;
    const double hint[3] = {0.0, steep ? 0.0 : 1.0, steep ? 1.0 : 0.0};
    double R[3] = {hint[1] * F[2] - hint[2] * F[1], hint[2] * F[0] - hint[0] * F[2], hint[0] * F[1] - hint[1] * F[0]};
    const double rl = std::sqrt(R[0] * R[0] + R[1] * R[1] + R[2] * R[2]);
    for (double& v : R) v /= rl;
    const double U[3] = {F[1] * R[2] - F[2] * R[1], F[2] * R[0] - F[0] * R[2], F[0] * R[1] - F[1] * R[0]};
    // THE CENTRE STANDS IN THE WORLD. It is kept in the flat frame's doubles, snapped to the texel
    // grid of the sun's axes THERE, and taken eye-relative only for the matrix: snapped about the
    // eye instead (the first form), the grid slid with the eye and every shadow edge crawled as the
    // camera moved. It moves only when the focus has left the map's middle (a quarter side from
    // the centre), so a focus that wanders with the eye's forward does not slide the map's edge
    // through the shadows every frame; and it moves when the side or the sun's axes change by more
    // than a texel's worth, so the grid is the sun's own.
    const double texel = sideM / double(kSunTexels);
    double fR = focus[0] * R[0] + focus[1] * R[1] + focus[2] * R[2];
    double fU = focus[0] * U[0] + focus[1] * U[1] + focus[2] * U[2];
    const double fS = focus[0] * S[0] + focus[1] * S[1] + focus[2] * S[2];
    bool move = !stood || m_sun.side != static_cast<float>(sideM);
    if (!move) {
        const double dR = m_sun.centre[0] * R[0] + m_sun.centre[1] * R[1] + m_sun.centre[2] * R[2] - fR;
        const double dU = m_sun.centre[0] * U[0] + m_sun.centre[1] * U[1] + m_sun.centre[2] * U[2] - fU;
        const double dS = m_sun.centre[0] * S[0] + m_sun.centre[1] * S[1] + m_sun.centre[2] * S[2] - fS;
        move = std::abs(dR) > 0.25 * sideM || std::abs(dU) > 0.25 * sideM || std::abs(dS) > 0.25 * sideM;
    }
    if (move) {
        fR = std::round(fR / texel) * texel;
        fU = std::round(fU / texel) * texel;
        for (int i = 0; i < 3; ++i) m_sun.centre[i] = fR * R[i] + fU * U[i] + fS * S[i];
    }
    const double c[3] = {m_sun.centre[0] - cam.px, m_sun.centre[1] - cam.py, m_sun.centre[2] - cam.pz};
    // The camera on the sun's side, half a side out, looking down the sun; near = side (the far
    // face -> 0), far = 0 (the sun's face -> 1): reversed, as the frame's depth is.
    const XMVECTOR pos = XMVectorSet(float(c[0] + S[0] * 0.5 * sideM), float(c[1] + S[1] * 0.5 * sideM), float(c[2] + S[2] * 0.5 * sideM), 1.0f);
    const XMVECTOR fwd = XMVectorSet(float(F[0]), float(F[1]), float(F[2]), 0.0f);
    const XMVECTOR upv = XMVectorSet(float(U[0]), float(U[1]), float(U[2]), 0.0f);
    const XMMATRIX view = XMMatrixLookToLH(pos, fwd, upv);
    const XMMATRIX proj = XMMatrixOrthographicLH(float(sideM), float(sideM), float(sideM), 0.0f);
    XMStoreFloat4x4(reinterpret_cast<XMFLOAT4X4*>(m_sun.vp), XMMatrixMultiply(view, proj));
    memcpy(cb.sunVP, m_sun.vp, sizeof(m_sun.vp));
    m_sun.side = static_cast<float>(sideM);
    cb.sunMapU[0] = m_sunSrv;
    cb.shadow[3] = static_cast<float>(sideM);
    m_sun.on = true;
}

void BuildingLayer::DumpSunMap(const std::wstring& path) {
    if (!m_sun.on || !m_gpu || !m_sunTex.res) return;
    m_gpu->WaitIdle();
    uint32_t pitch = 0;
    const std::vector<uint8_t> px = m_gpu->ReadbackTexture(m_sunTex, &pitch);
    if (px.empty()) return;
    std::vector<uint8_t> rgba(size_t(kSunTexels) * kSunTexels * 4);
    uint32_t hit = 0;
    for (uint32_t y = 0; y < kSunTexels; ++y) {
        const float* row = reinterpret_cast<const float*>(px.data() + size_t(y) * pitch);
        for (uint32_t x = 0; x < kSunTexels; ++x) {
            const float z = row[x];
            const uint8_t g = static_cast<uint8_t>((std::min)((std::max)(z, 0.0f), 1.0f) * 255.0f);
            uint8_t* o = &rgba[(size_t(y) * kSunTexels + x) * 4];
            o[0] = o[1] = o[2] = g;
            o[3] = 255;
            hit += z > 0.0f ? 1u : 0u;
        }
    }
    const bool ok = SavePng(path, rgba.data(), kSunTexels, kSunTexels, kSunTexels * 4, rgba.size());
    Log("[buildings] sun map dump %S: %s, %u of %u texels hold a solid (%.1f %%), side %.0f m",
        path.c_str(), ok ? "ok" : "FAILED", hit, kSunTexels * kSunTexels, 100.0 * double(hit) / (double(kSunTexels) * double(kSunTexels)), double(m_sun.side));
}

void BuildingLayer::RenderSunMap(const FrameContext& ctx) {
    if (!m_sun.on || !ctx.cmd || !ctx.camera || !ctx.gpu || m_sunDsv == UINT32_MAX) return;
    const Eye& e = m_eyes[WalkerOf(0, 0)];
    const bool any = !m_cells.empty() || !m_warm.empty() || e.drawn.vb.Valid();   // (the points cast nothing)
    PixScope scope(ctx.cmd->Native(), "buildings.sun (the solids as the sun sees them)");
    GpuScope gscope(ctx.prof, ctx.cmd->Native(), "buildings.sun");
    // The map is the pass's own target; the frame's targets, root and viewport are set by the
    // renderer after every layer's Simulate (Renderer::RenderFrame), so nothing is put back here.
    ctx.cmd->Barrier(m_sunTex, D3D12_RESOURCE_STATE_DEPTH_WRITE);
    const D3D12_CPU_DESCRIPTOR_HANDLE dsv = ctx.gpu->DsvHeap().Cpu(m_sunDsv);
    ctx.cmd->DepthTarget(dsv);
    ctx.cmd->ClearDepth(dsv, 0.0f);
    ctx.cmd->Viewport(kSunTexels, kSunTexels);
    if (!m_sunSaid) {   // the instrument: once, what the pass draws and where the map stands
        m_sunSaid = true;
        Log("[buildings] sun map: %u texels a side, drawn %s: %zu cells, %u nodes named (%u records) of eye 0's own world; "
            "pipelines cells %d boxes %d boxes.mesh %d shapes %d; srv %u dsv %u",
            kSunTexels, any ? "yes" : "nothing yet", m_cells.size(), e.drawn.named, e.drawn.cap,
            m_psoSun ? 1 : 0, m_psoSunBox ? 1 : 0, m_psoSunBoxMesh ? 1 : 0, m_psoSunShape ? 1 : 0, m_sunSrv, m_sunDsv);
    }
    if (any) {
        // b0 for the cast: the vertex and mesh stages read gViewProj and nothing else of the scene's
        // rows (the light is the pixel stage's, and this pass has none).
        SceneConstants sc{};
        memcpy(sc.viewProj, m_sun.vp, sizeof(sc.viewProj));
        ctx.cmd->GraphicsRoot(m_rootSig);
        ctx.cmd->GraphicsConstantsAt(0, ctx.cmd->Push(sc));
        ctx.cmd->GraphicsBindless(3);   // the shape pool (space7) through the heap's table
        Pipes p;
        p.cells = m_psoSun.Get();
        p.box = m_psoSunBox.Get();
        p.boxMesh = m_psoSunBoxMesh.Get();
        p.shape = m_psoSunShape.Get();
        DrawSolids(ctx, 0, 1, p, "buildings.sun.draw");
    }
    ctx.cmd->Barrier(m_sunTex, D3D12_RESOURCE_STATE_PIXEL_SHADER_RESOURCE);
}

}  // namespace ga
