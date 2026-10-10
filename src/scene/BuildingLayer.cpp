#include "scene/BuildingLayer.h"

#include "compose/BuildingShape.h"

#include "compose/BuildingMoments.h"

#include "core/Common.h"
#include "core/ThreadManager.h"
#include "hal/GpuProfiler.h"
#include "hal/PixEvents.h"
#include "hal/Pipeline.h"
#include "hal/Shader.h"
#include "render/Camera.h"

#include <algorithm>
#include <execution>
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
    m_shared->cancel = true;
    for (int i = 0; i < 2000 && (m_shared->inflight.load() > 0 || m_shared->pageInflight.load() > 0 || m_shared->walking.load() > 0); ++i) {
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

void BuildingLayer::ConfigureFar(std::shared_ptr<const BuildingLodFile> lod, double pixels) {
    m_lod = std::move(lod);
    m_lodPixels = pixels > 0.0 ? pixels : 1.0;
}

void BuildingLayer::Init(Gpu& gpu, ShaderCompiler& sc, FieldSet&, hal::RootSignature rootSig) {
    m_rootSig = rootSig;
    m_gpu = &gpu;
    if (!BuildPso(gpu, sc)) throw std::runtime_error("building PSO failed");
    gpu.onLostDispatch = [this](uint32_t k) { DumpChunk(k); };
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
    d.blend = true;                                  // a fold's coverage as its alpha
    d.srcBlend = D3D12_BLEND_SRC_ALPHA;
    d.dstBlend = D3D12_BLEND_INV_SRC_ALPHA;
    const bool okBox = hal::Reload(m_psoBox, [&] { return hal::BuildGraphics(gpu, d, "buildings.boxes"); }, "buildings.boxes");
    // THE SHAPES' PIPELINE: MsShape extrudes a building's own footprint (needs mesh shaders; without
    // them the walk draws boxes, as before).
    D3D12_FEATURE_DATA_D3D12_OPTIONS7 o7{};
    if (SUCCEEDED(gpu.Device()->CheckFeatureSupport(D3D12_FEATURE_D3D12_OPTIONS7, &o7, sizeof(o7))) &&
        o7.MeshShaderTier != D3D12_MESH_SHADER_TIER_NOT_SUPPORTED) {
        hal::MeshPipelineDesc md;
        md.rootSig = m_rootSig;
        md.ms = sc.Compile(path, L"MsShape", L"ms_6_5");
        md.ps = sc.Compile(path, L"PsMain", L"ps_6_5");
        md.cull = D3D12_CULL_MODE_NONE;
        md.depthClip = TRUE;
        md.depthTest = true;
        md.depthWrite = true;
        md.blend = true;   // a building fading in, as the boxes
        md.srcBlend = D3D12_BLEND_SRC_ALPHA;
        md.dstBlend = D3D12_BLEND_INV_SRC_ALPHA;
        if (md.ms.Valid() && md.ps.Valid()) {
            hal::Reload(m_psoShape, [&] { return hal::BuildMesh(gpu, md, "buildings.shapes"); }, "buildings.shapes");
        }
    }
    return okBox && ok;
}

void BuildingLayer::ReloadShaders(Gpu& gpu, ShaderCompiler& sc) {
    BuildPso(gpu, sc);   // the reload law lives in BuildPso: swap only on success
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
            const BuildingLayer::Ground& ground, std::vector<BuildingLayer::Vertex>& out) {
    using Vertex = BuildingLayer::Vertex;
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
            if (!shared->cancel.load()) {
                const auto t0 = std::chrono::steady_clock::now();
                Built b;
                b.key = k;
                const double x0 = k.first * kCellDeg, y0 = k.second * kCellDeg;
                place(y0 + 0.5 * kCellDeg, x0 + 0.5 * kCellDeg, 0.0, b.origin);
                // The margin: a footprint whose first point is a little over the edge still covers.
                const std::vector<BuildingSolid> solids =
                    stack->Compose(x0, y0, x0 + kCellDeg, y0 + kCellDeg, 0.002);
                b.drawn.assign(stack->Sources(), 0);
                std::vector<Vertex> verts;
                for (const BuildingSolid& s : solids) {
                    if (shared->cancel.load()) break;
                    Prisms(s, b.origin, place, ground, verts);
                    if (s.source >= 0) ++b.drawn[static_cast<size_t>(s.source)];
                }
                b.count = static_cast<uint32_t>(verts.size());
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
            br.Transition.StateAfter = D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE;
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

static std::pair<int, int> CellOf(double lat, double lon) {
    return {static_cast<int>(std::floor(lon / BuildingLayer::kCellDeg)), static_cast<int>(std::floor(lat / BuildingLayer::kCellDeg))};
}

void BuildingLayer::LoadPage(const PageKey& k) {
    const int L = std::get<0>(k);
    const LodPage* lp = m_lod->Find(L, std::get<1>(k), std::get<2>(k));
    if (!lp) return;
    m_pagePending.insert(k);
    m_shared->pageInflight.fetch_add(1);
    Threads().Submit(Lane::Io, "buildings.page",
                     [shared = m_shared, lodf = m_lod, place = m_place, ground = m_ground, gpu = m_gpu,
                      shapes = m_psoShape != nullptr, k, lp = *lp, L] {
        auto page = std::make_shared<Page>();
        std::vector<LodNode> nodes;
        std::vector<LodBuilding> blds;
        if (!shared->cancel.load() && lodf->Read(L, lp, nodes, blds)) {
            page->L = L;
            // A box stood on the composed ground at its centroid, on the prisms' own axes.
            auto stand = [&](double lat, double lon, double base, double zc, double hz, double a1, double a2, double hd,
                             RtBox& o, RtBld* frame) {
                if (!(hz > 0.0)) return;
                double p[3], e[3], n[3], u[3];
                PointFrame(place, lat, lon, base + zc, p, e, n, u);
                if (frame) {   // the shape's own frame: the ground under the centroid, east, north, up
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
            };
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
                    if (sv.head.nVerts >= 3 && rb <= kSlotBytes) {
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
                    } else if (sv.head.nVerts >= 3) {
                        ++page->tooBig;
                    }
                    at += rb;
                }
                stand(lat, lon, ground(lat, lon), zc, hz, a1, a2, hd, b.box, b.shapeOff != UINT32_MAX ? &b : nullptr);
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
                place(latc, lonc, ground(latc, lonc), r.c);
                if (r.kids) {
                    const double dlat = n.desc.lat7 * 1e-7, dlon = n.desc.lon7 * 1e-7;
                    stand(dlat, dlon, ground(dlat, dlon), n.desc.zc, n.desc.hz, n.desc.a1, n.desc.a2, n.desc.heading, r.desc, nullptr);
                    r.descCover = std::clamp(n.desc.cover, 0.0f, 1.0f);
                    r.descCell = CellOf(dlat, dlon);
                }
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
        std::lock_guard<std::mutex> lk(shared->mx);
        shared->pagesDone.push_back({k, page});
        shared->pageInflight.fetch_sub(1);
    });
}

void BuildingLayer::TreeFrame(const FrameContext& ctx, double latDeg, double lonDeg, double h) {
    // 1. What the pool finished: pages into the tree, a walk's boxes onto the GPU.
    std::vector<std::pair<PageKey, std::shared_ptr<Page>>> pages;
    std::vector<Walked> walked;
    {
        std::lock_guard<std::mutex> lk(m_shared->mx);
        pages.swap(m_shared->pagesDone);
        walked.swap(m_shared->walked);
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
        m_pageBytes += p->bytes;
        ++m_pagesLoaded;
        m_pageUsed[k] = m_frame;
        p->landed = Now();
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
        br.Transition.StateAfter = D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE;
        ctx.cmd->Native()->ResourceBarrier(1, &br);
    }
    for (Walked& w : walked) {
        if ((w.count || w.insts) && w.vb.Valid()) {   // the cells' own copy and barrier (Upload)
            ID3D12GraphicsCommandList* cl = ctx.cmd->Native();
            const uint64_t bytes = (uint64_t(w.count + w.insts) * sizeof(Box) + uint64_t(w.tasks) * 8u + 15u) & ~uint64_t(15);
            cl->CopyBufferRegion(w.vb.res.Get(), 0, w.staging.res.Get(), 0, bytes);
            D3D12_RESOURCE_BARRIER br{};
            br.Type = D3D12_RESOURCE_BARRIER_TYPE_TRANSITION;
            br.Transition.pResource = w.vb.res.Get();
            br.Transition.Subresource = D3D12_RESOURCE_BARRIER_ALL_SUBRESOURCES;
            br.Transition.StateBefore = D3D12_RESOURCE_STATE_COPY_DEST;
            br.Transition.StateAfter = D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE;
            cl->ResourceBarrier(1, &br);
            m_retired.push_back({std::move(w.staging), m_frame, 1});
        }
        if (m_drawn.vb.Valid()) m_retired.push_back({std::move(m_drawn.vb), m_frame, 2});
        for (int i = 0; i < 3; ++i) m_drawn.origin[i] = w.origin[i];
        m_drawn.count = w.vb.Valid() ? w.count : 0;
        m_drawn.insts = w.vb.Valid() ? w.insts : 0;
        m_drawn.tasks = w.vb.Valid() ? w.tasks : 0;
        m_drawnUsed = std::set<PageKey>(w.used.begin(), w.used.end());
        m_drawnHold = std::move(w.hold);
        m_drawn.cpu = std::move(w.cpu);
        m_drawn.vb = std::move(w.vb);
        m_drawnCells = w.cellsVersion;
        for (const PageKey& k : w.used) m_pageUsed[k] = m_frame;
        // The pages the walk wanted, the largest on screen first.
        m_wants = std::move(w.wants);
        ++m_walks;
        const bool largest = m_drawn.tasks > m_walkTasksMax;
        if (largest) m_walkTasksMax = m_drawn.tasks;
        if (m_walks <= 3 || m_walks % 50 == 0 || (largest && m_drawn.tasks > 500000u)) {
            Log("[buildings] frame %llu walk %llu: %u boxes + %u shapes in %u mesh tasks (%llu buildings, %llu fading in) over %llu nodes in %.1f ms (visit %.1f, "
                "sort %.1f, buffers %.1f); %zu pages (%.0f MB), %zu asked", static_cast<unsigned long long>(m_frame),
                static_cast<unsigned long long>(m_walks), m_drawn.count, m_drawn.insts, m_drawn.tasks, static_cast<unsigned long long>(w.singles),
                static_cast<unsigned long long>(w.fading), static_cast<unsigned long long>(w.nodes), w.ms, w.msVisit, w.msSort,
                w.msBuf, m_pages.size(), m_pageBytes / 1048576.0, m_wants.size());
        }
    }
    // THE TWO STREAMS. The coarse level is the coarsest still wanted or loading; the coarse stream
    // sends its pages only, the fine stream every finer one by size on screen, each up to its own
    // slots. A new walk (the eye moved) replaces the list: what it no longer wants is never sent.
    // While a landed page waits for pool slots, nothing more is sent: no page is read that cannot land.
    if (m_roomWait.empty()) {
        int level = INT32_MIN;
        for (const PageKey& k : m_pagePending) level = (std::max)(level, std::get<0>(k));
        for (const auto& [px, k] : m_wants) {
            if (!m_pages.count(k)) level = (std::max)(level, std::get<0>(k));
        }
        int coarse = 0, fine = 0;
        for (const PageKey& k : m_pagePending) ++(std::get<0>(k) == level ? coarse : fine);
        std::vector<std::pair<double, PageKey>> finer;
        for (const auto& [px, k] : m_wants) {
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
    // 2. The budget: the page no walk has used for longest goes first.
    // ONE EVICTION FOR BOTH LIMITS: the page no walk has used for longest goes, while the pages'
    // memory is over its budget OR the pages waiting to land want more slots than will be free.
    auto slotsShort = [this] {
        size_t want = 0, coming = m_freeSlots.size();
        for (const auto& w : m_roomWait) want += w.second->slotUsed.size();
        for (const SlotsRetired& r : m_slotsRetired) coming += r.slots.size();
        return want > coming;
    };
    while (!m_pages.empty() && (m_pageBytes > kPageBytes || slotsShort())) {
        auto old = m_pages.begin();
        for (auto it = m_pages.begin(); it != m_pages.end(); ++it) {
            if (m_pageUsed[it->first] < m_pageUsed[old->first]) old = it;
        }
        if (m_pageUsed[old->first] + 2 >= m_frame) break;   // all in use: over budget, not thrashing
        if (m_drawnUsed.count(old->first)) break;   // the drawn walk reads its shapes
        if (!old->second->slots.empty()) m_slotsRetired.push_back({old->second->slots, m_frame, old->second});
        m_pageBytes -= old->second->bytes;
        m_pageUsed.erase(old->first);
        m_pages.erase(old);
        ++m_pagesVersion;
    }
    // 3. A new walk, when its inputs have moved and none is running.
    if (m_shared->walking.load() || !ctx.height) return;
    const double eye[3] = {ctx.camera->px, ctx.camera->py, ctx.camera->pz};
    const DirectX::XMFLOAT3 f3 = ctx.camera->Forward();
    const double fwd[3] = {f3.x, f3.y, f3.z};
    const double moved = std::sqrt((eye[0] - m_walkEye[0]) * (eye[0] - m_walkEye[0]) + (eye[1] - m_walkEye[1]) * (eye[1] - m_walkEye[1]) +
                                   (eye[2] - m_walkEye[2]) * (eye[2] - m_walkEye[2]));
    const double turned = fwd[0] * m_walkFwd[0] + fwd[1] * m_walkFwd[1] + fwd[2] * m_walkFwd[2];
    if (moved < (std::max)(1.0, 0.002 * (std::max)(h, 0.0)) && turned > 0.99998 && m_walkCells == m_cellsVersion &&
        m_walkPages == m_pagesVersion) {
        return;
    }
    for (int i = 0; i < 3; ++i) {
        m_walkEye[i] = eye[i];
        m_walkFwd[i] = fwd[i];
    }
    m_walkCells = m_cellsVersion;
    m_walkPages = m_pagesVersion;
    struct In {
        double eye[3], fwd[3], lat, lon, h, pixAng, halfDiag, cosHalf, sinHalf, tauB;
        struct KeyHash {
            size_t operator()(const PageKey& k) const {
                return std::hash<uint64_t>()((uint64_t(uint32_t(std::get<0>(k) + 64)) << 56) ^
                                             (uint64_t(uint32_t(std::get<1>(k))) << 28) ^ uint64_t(uint32_t(std::get<2>(k))));
            }
        };
        std::unordered_map<PageKey, const Page*, KeyHash> pages;
        std::vector<std::shared_ptr<const Page>> hold;   // the walk's references
        bool shapes = false;                             // the shape pool and its pipeline stand
        int cx0 = 1, cx1 = 0, cy0 = 1, cy1 = 0;          // the resident cells' bounding box (empty: x0 > x1)
        std::set<Key> cells;
        uint64_t cellsVersion;
    } in;
    for (int i = 0; i < 3; ++i) {
        in.eye[i] = eye[i];
        in.fwd[i] = fwd[i];
    }
    in.lat = latDeg;
    in.lon = lonDeg;
    in.h = h;
    in.pixAng = double(ctx.camera->fovY) / double(ctx.height);
    const double aspect = ctx.width ? double(ctx.width) / double(ctx.height) : 16.0 / 9.0;
    in.halfDiag = std::atan(std::tan(0.5 * double(ctx.camera->fovY)) * std::sqrt(1.0 + aspect * aspect));
    in.shapes = m_psoShape && m_pool.Valid();
    in.cosHalf = std::cos(in.halfDiag);
    in.sinHalf = std::sin(in.halfDiag);
    in.tauB = m_lodPixels;
    in.pages.reserve(m_pages.size());
    in.hold.reserve(m_pages.size());
    for (const auto& [k, p] : m_pages) {
        in.pages.emplace(k, p.get());
        in.hold.push_back(p);
    }
    for (const auto& [k, c] : m_cells) {
        in.cells.insert(k);
        if (in.cx0 > in.cx1) {
            in.cx0 = in.cx1 = k.first;
            in.cy0 = in.cy1 = k.second;
        }
        in.cx0 = (std::min)(in.cx0, k.first);
        in.cx1 = (std::max)(in.cx1, k.first);
        in.cy0 = (std::min)(in.cy0, k.second);
        in.cy1 = (std::max)(in.cy1, k.second);
    }
    in.cellsVersion = m_cellsVersion;
    m_shared->walking.store(1);
    Threads().Submit(Lane::Io, "buildings.walk", [shared = m_shared, lodf = m_lod, gpu = ctx.gpu, in = std::move(in)] {
        const auto t0 = std::chrono::steady_clock::now();
        Walked w;
        w.cellsVersion = in.cellsVersion;
        for (int i = 0; i < 3; ++i) w.origin[i] = in.eye[i];
        // THE WALK IN PARALLEL: the coarse levels walked here down to kSplit, the subtrees below it
        // shared out to threads, each into its own accumulator; merged after. A pure function of `in`.
        struct Inst {   // a shaped instance and the counts its tasks are cut from
            Box b;
            uint16_t nV, nT;
        };
        struct Acc {
            std::vector<Box> out;
            std::vector<Inst> inst;
            std::set<PageKey> used;
            std::map<PageKey, double> wants;
            uint64_t nodes = 0, singles = 0, fading = 0;
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
        auto emit = [&](Acc& acc, const RtBox& b, double alpha, float landed) {
            if (!b.valid || acc.out.size() >= kMaxBoxes) return;
            Box x{};
            for (int i = 0; i < 3; ++i) {
                x.c[i] = static_cast<float>(b.p[i] - in.eye[i]);
                x.u[i] = b.u[i];
                x.v[i] = b.v[i];
                x.w[i] = b.w[i];
            }
            x.alpha = static_cast<float>(alpha);
            x.pad1 = landed;   // Buildings.hlsl VsBox: its page's landing, for the fade-in
            acc.out.push_back(x);
        };
        // ...or its own form: the ground frame under its centroid and its record in the pool (MsShape).
        auto emitShape = [&](Acc& acc, const RtBld& b, double alpha, float landed, uint64_t addr) {
            if (acc.out.size() + acc.inst.size() >= kMaxBoxes) return;
            Inst x{};
            for (int i = 0; i < 3; ++i) {
                x.b.c[i] = static_cast<float>(b.g[i] - in.eye[i]);
                x.b.u[i] = b.e[i];
                x.b.v[i] = b.n[i];
                x.b.w[i] = b.up[i];
            }
            x.b.alpha = static_cast<float>(alpha);
            x.b.pad1 = landed;
            const uint32_t a32 = static_cast<uint32_t>(addr);
            std::memcpy(&x.b.pad2, &a32, 4);
            x.nV = b.nV;
            x.nT = b.nT;
            acc.inst.push_back(x);
        };
        std::function<void(Acc&, bool, int, const Page&, const RtNode&)> visit = [&](Acc& acc, bool split, int L,
                                                                                       const Page& pg, const RtNode& n) {
            if (split && L == kSplit) {
                frontier.push_back({L, &pg, &n});
                return;
            }
            ++acc.nodes;
            double v[3], dist = 0.0;
            for (int i = 0; i < 3; ++i) {
                v[i] = n.c[i] - in.eye[i];
                dist += v[i] * v[i];
            }
            dist = std::sqrt(dist);
            if (dist - n.rad > dh) return;   // under the horizon
            if (dist > n.rad) {              // outside the view's cone
                // Outside when the angle to it, less its sphere's half-angle b, passes the half-diagonal:
                // cos(angle) < cos(halfDiag + b), formed without a trig call (sin b = rad / dist).
                const double ca = (v[0] * in.fwd[0] + v[1] * in.fwd[1] + v[2] * in.fwd[2]) / dist;
                const double sb = n.rad / dist, cb = std::sqrt((std::max)(0.0, 1.0 - sb * sb));
                if (ca < in.cosHalf * cb - in.sinHalf * sb) return;
            }
            // THE RANK BOUND: a building filed here has radius at most Q/4 (BuildingLod's LevelOf), so
            // nothing here or below is wider than half the quad: under lodPixels, the walk stops.
            const double dn = (std::max)(dist - n.rad, 1.0);
            const double px = lod::QuadDeg(L) * lod::kMetresPerDeg / (dn * in.pixAng);
            if (L < lod::kLmax && 0.5 * px < in.tauB) return;
            // ONE LAW PER BUILDING: its diameter in pixels at its own distance, faded in over one
            // threshold's width above the threshold.
            for (uint32_t k = n.own0; k < n.own0 + n.ownN && k < pg.blds.size(); ++k) {
                const RtBld& b = pg.blds[k];
                if (2.0 * b.rho / (dn * in.pixAng) < in.tauB) break;   // this and every smaller one: under a pixel
                if (!b.box.valid) continue;
                double d = 0.0;
                for (int i = 0; i < 3; ++i) d += (b.box.p[i] - in.eye[i]) * (b.box.p[i] - in.eye[i]);
                const double sz = 2.0 * b.rho / ((std::max)(std::sqrt(d), 1.0) * in.pixAng);
                const double a = (sz - in.tauB) / in.tauB;
                if (a <= 0.0) continue;   // too small here
                if (b.cell.first >= in.cx0 && b.cell.first <= in.cx1 && b.cell.second >= in.cy0 && b.cell.second <= in.cy1 &&
                    in.cells.count(b.cell)) {
                    continue;   // its prisms are drawn
                }
                if (in.shapes && b.shapeOff != UINT32_MAX && b.shapeOff / kSlotBytes < pg.slots.size()) {
                    emitShape(acc, b, (std::min)(a, 1.0), pg.landed,
                              uint64_t(pg.slots[b.shapeOff / kSlotBytes]) * kSlotBytes + b.shapeOff % kSlotBytes);
                } else {
                    emit(acc, b.box, (std::min)(a, 1.0), pg.landed);
                }
                ++acc.singles;
                if (a < 1.0) ++acc.fading;
            }
            if (L == lod::kLmin || !n.kids) return;
            // The children's pages: the loaded ones are walked; the missing ones are asked for, by
            // the quad's size on screen, and their buildings appear when they land.
            // The four children share one page (a page is kPageX wide and PageY(L) tall, both even).
            const PageKey k = pageOf(L - 1, 2 * n.x, 2 * n.y);
            const Page* cp = loaded(k);
            if (!cp) {
                if (lodf->Find(L - 1, std::get<1>(k), std::get<2>(k))) {
                    double& want = acc.wants[k];
                    want = (std::max)(want, px);
                    // The fold stands in while the page is on its way (unless its cell's prisms are drawn).
                    if (!(n.descCell.first >= in.cx0 && n.descCell.first <= in.cx1 && n.descCell.second >= in.cy0 &&
                          n.descCell.second <= in.cy1 && in.cells.count(n.descCell))) {
                        emit(acc, n.desc, n.descCover, pg.landed);
                    }
                }
                return;
            }
            if (k != acc.lastUsed) {
                acc.used.insert(k);
                acc.lastUsed = k;
            }
            if (L - 1 == lod::kLmin) {   // the finest children: their largest, at this node's nearest distance
                auto pm = cp->parentRho.find(Page::QuadKey(n.x, n.y));
                if (pm == cp->parentRho.end() || 2.0 * pm->second / (dn * in.pixAng) < in.tauB) return;
            }
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
            for (Acc& a : accs) {
                top.out.insert(top.out.end(), a.out.begin(), a.out.end());
                top.inst.insert(top.inst.end(), a.inst.begin(), a.inst.end());
                top.used.insert(a.used.begin(), a.used.end());
                for (const auto& [k, px] : a.wants) {
                    double& want = top.wants[k];
                    want = (std::max)(want, px);
                }
                top.nodes += a.nodes;
                top.singles += a.singles;
                top.fading += a.fading;
            }
        }
        std::vector<Box>& out = top.out;
        std::set<PageKey>& used = top.used;
        std::map<PageKey, double>& wants = top.wants;
        w.nodes = top.nodes;
        w.singles = top.singles;
        w.fading = top.fading;
        const auto tV = std::chrono::steady_clock::now();
        w.msVisit = std::chrono::duration<double, std::milli>(tV - t0).count();
        // Far to near: a building fading in blends over what stands behind it (painter's order, depth tested).
        std::sort(std::execution::par, out.begin(), out.end(), [](const Box& a, const Box& b) {
            return a.c[0] * a.c[0] + a.c[1] * a.c[1] + a.c[2] * a.c[2] > b.c[0] * b.c[0] + b.c[1] * b.c[1] + b.c[2] * b.c[2];
        });
        std::vector<Inst>& inst = top.inst;
        std::sort(std::execution::par, inst.begin(), inst.end(), [](const Inst& a, const Inst& b) {
            return a.b.c[0] * a.b.c[0] + a.b.c[1] * a.b.c[1] + a.b.c[2] * a.b.c[2] >
                   b.b.c[0] * b.b.c[0] + b.b.c[1] * b.b.c[1] + b.b.c[2] * b.b.c[2];
        });
        // THE TASKS, in the instances' order (the raster keeps the groups' order: far to near).
        std::vector<uint32_t> tasks;
        tasks.reserve(inst.size() * 4);
        for (size_t i = 0; i < inst.size(); ++i) {
            for (uint32_t c = 0; c * kTaskEdges < inst[i].nV; ++c) tasks.insert(tasks.end(), {uint32_t(i), c});
            for (uint32_t c = 0; c * kTaskTris < inst[i].nT; ++c) tasks.insert(tasks.end(), {uint32_t(i), 0x80000000u | c});
        }
        const auto tS = std::chrono::steady_clock::now();
        w.msSort = std::chrono::duration<double, std::milli>(tS - tV).count();
        w.count = static_cast<uint32_t>(out.size());
        w.insts = static_cast<uint32_t>(inst.size());
        w.tasks = static_cast<uint32_t>(tasks.size() / 2);
        {
            auto cpu = std::make_shared<DrawCpu>();
            cpu->inst.reserve(inst.size());
            for (const Inst& x : inst) cpu->inst.push_back(x.b);
            cpu->tasks = tasks;
            w.cpu = std::move(cpu);
        }
        if ((w.count || w.insts) && !shared->cancel.load()) {
            // One buffer: the boxes, the shaped instances, then the tasks (two uints each).
            const uint64_t bytes = (uint64_t(w.count + w.insts) * sizeof(Box) + tasks.size() * 4u + 15u) & ~uint64_t(15);
            auto take = [&](std::vector<GpuBuffer>& list) {
                std::lock_guard<std::mutex> lk(shared->mx);
                auto best = list.end();
                for (auto it = list.begin(); it != list.end(); ++it) {
                    if (it->size >= bytes && (best == list.end() || it->size < best->size)) best = it;
                }
                GpuBuffer b;
                if (best != list.end()) {
                    b = std::move(*best);
                    list.erase(best);
                }
                return b;
            };
            const uint64_t room = bytes + bytes / 4;   // a little over, so the next walk fits
            w.staging = take(shared->freeStaging);
            if (!w.staging.Valid()) w.staging = gpu->CreateUploadBuffer(room, L"buildings.walk.staging");
            uint8_t* dst = w.staging.cpu;
            std::memcpy(dst, out.data(), out.size() * sizeof(Box));
            dst += out.size() * sizeof(Box);
            for (const Inst& x : inst) {
                std::memcpy(dst, &x.b, sizeof(Box));
                dst += sizeof(Box);
            }
            if (!tasks.empty()) std::memcpy(dst, tasks.data(), tasks.size() * 4u);
            w.vb = take(shared->freeBoxes);
            if (!w.vb.Valid()) w.vb = gpu->CreateDefaultBuffer(nullptr, room, L"buildings.walk");
        }
        w.msBuf = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - tS).count();
        for (const auto& [k, px] : wants) w.wants.push_back({px, k});
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
        shared->walking.store(0);
    });
}

void BuildingLayer::Simulate(const FrameContext& ctx) {
    ++m_frame;
    // Buffers no frame in flight can still read.
    {
        std::lock_guard<std::mutex> lk(m_shared->mx);
        for (Retired& r : m_retired) {
            if (r.pool == 0 || m_frame - r.frame <= kRetireFrames || !r.buf.Valid()) continue;
            std::vector<GpuBuffer>& list = r.pool == 1 ? m_shared->freeStaging : m_shared->freeBoxes;
            // The two largest are kept: a walk only ever needs one, and a small one would never fit.
            if (list.size() < 2) {
                list.push_back(std::move(r.buf));
            } else {
                auto least = std::min_element(list.begin(), list.end(),
                                              [](const GpuBuffer& a, const GpuBuffer& b) { return a.size < b.size; });
                if (least->size < r.buf.size) *least = std::move(r.buf);
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
            if (it->second.leftAt > m_drawnCells) continue;
            if (old == m_warm.end() || it->second.frame < old->second.frame) old = it;
        }
        if (old == m_warm.end()) break;
        m_warmBytes -= uint64_t(old->second.cell.count) * sizeof(Vertex);
        if (old->second.cell.vb.Valid()) m_retired.push_back({std::move(old->second.cell.vb), m_frame});
        m_warm.erase(old);
    }
    Upload(ctx);
    Want(ctx.gpu, lat, lon, h);
    if (m_lod && m_lod->Valid() && ctx.height > 0) TreeFrame(ctx, lat, lon, h);
}

void BuildingLayer::DumpChunk(uint32_t k) const {
    // Which page holds a pool address: its slot, found among the pages any frame could have drawn.
    std::map<uint32_t, std::pair<const Page*, uint32_t>> bySlot;   // slot -> (page, its ordinal there)
    auto index = [&](const Page& p) {
        for (uint32_t j = 0; j < p.slots.size(); ++j) bySlot[p.slots[j]] = {&p, j};
    };
    for (const auto& [key, p] : m_pages) index(*p);
    for (const auto& p : m_drawnHold) index(*p);
    for (const DrawLog& d : m_drawLog) {
        if (!d.cpu) continue;
        const DrawCpu& c = *d.cpu;
        const size_t nTasks = c.tasks.size() / 2;
        const size_t t0 = size_t(k) * kChunkGroups, t1 = (std::min)(nTasks, t0 + kChunkGroups);
        if (t0 >= nTasks) {
            Log("[crash] buildings, frame %llu's draw: chunk %u is past its %zu tasks", static_cast<unsigned long long>(d.frame), k, nTasks);
            continue;
        }
        std::set<uint32_t> insts;
        for (size_t t = t0; t < t1; ++t) insts.insert(c.tasks[2 * t]);
        size_t found = 0, bad = 0, tall = 0, wide = 0;
        float maxTop = -1e30f, minBottom = 1e30f, maxExt = 0.0f, maxDist = 0.0f, minDist = 1e30f;
        uint32_t maxV = 0, maxT = 0;
        struct Worst { float score; uint32_t inst, nV, nT, nR; float bottom, top, ext, dist; };
        std::vector<Worst> worst;
        for (uint32_t i : insts) {
            if (i >= c.inst.size()) { ++bad; continue; }
            const Box& b = c.inst[i];
            const float dist = std::sqrt(b.c[0] * b.c[0] + b.c[1] * b.c[1] + b.c[2] * b.c[2]);
            minDist = (std::min)(minDist, dist);
            maxDist = (std::max)(maxDist, dist);
            uint32_t addr;
            std::memcpy(&addr, &b.pad2, 4);
            const auto it = bySlot.find(addr / kSlotBytes);
            if (it == bySlot.end()) { ++bad; continue; }
            const std::vector<uint8_t>& pk = it->second.first->packed;
            const size_t at = size_t(it->second.second) * kSlotBytes + addr % kSlotBytes;
            ShapeView sv;
            if (at >= pk.size() || !ReadShape(pk.data() + at, pk.data() + pk.size(), sv)) { ++bad; continue; }
            ++found;
            const ShapeHead& h = sv.head;
            uint32_t ringSum = 0, triMax = 0;
            for (uint32_t r = 0; r < h.nRings; ++r) ringSum += sv.ringLen[r];
            for (uint32_t t = 0; t < 3u * h.nTris; ++t) triMax = (std::max)(triMax, uint32_t(sv.tri[t]));
            float ext = 0.0f;
            for (uint32_t v = 0; v < 2u * h.nVerts; ++v) ext = (std::max)(ext, std::fabs(float(sv.xy[v]) * sv.unit));
            const bool broken = ringSum != h.nVerts || (h.nTris && triMax >= h.nVerts) || !std::isfinite(h.top) ||
                                !std::isfinite(h.bottom) || h.top < h.bottom;
            bad += broken;
            tall += h.top - h.bottom > 300.0f;
            wide += ext > 500.0f;
            maxTop = (std::max)(maxTop, h.top);
            minBottom = (std::min)(minBottom, h.bottom);
            maxExt = (std::max)(maxExt, ext);
            maxV = (std::max)(maxV, uint32_t(h.nVerts));
            maxT = (std::max)(maxT, uint32_t(h.nTris));
            worst.push_back({(h.top - h.bottom) * ext / (std::max)(dist, 1.0f) + (broken ? 1e9f : 0.0f), i, h.nVerts, h.nTris,
                             h.nRings, h.bottom, h.top, ext, dist});
        }
        Log("[crash] buildings, frame %llu's draw, chunk %u = tasks %zu..%zu of %zu: %zu instances, %zu records read, %zu "
            "unreadable or inconsistent; top up to %.1f m, bottom down to %.1f m, footprint out to %.1f m, up to %u vertices "
            "and %u roof triangles; %zu over 300 m tall, %zu over 500 m wide; %.0f..%.0f m from the walk's eye",
            static_cast<unsigned long long>(d.frame), k, t0, t1, nTasks, insts.size(), found, bad, maxTop, minBottom, maxExt, maxV,
            maxT, tall, wide, minDist, maxDist);
        std::sort(worst.begin(), worst.end(), [](const Worst& a, const Worst& b) { return a.score > b.score; });
        for (size_t j = 0; j < (std::min<size_t>)(5, worst.size()); ++j) {
            const Worst& x = worst[j];
            Log("[crash]   instance %u: %u vertices, %u roof triangles, %u rings, bottom %.1f top %.1f m, footprint %.1f m, "
                "%.0f m away", x.inst, x.nV, x.nT, x.nR, x.bottom, x.top, x.ext, x.dist);
        }
    }
}

void BuildingLayer::Render(const FrameContext& ctx) {
    if (!m_pso || (m_cells.empty() && m_warm.empty() && !m_drawn.count && !m_drawn.tasks) || !ctx.camera) return;
    PixScope scope(ctx.cmd->Native(), "buildings (solids -> prisms, cell origins about the eye)");
    GpuScope gscope(ctx.prof, ctx.cmd->Native(), "buildings");
    ctx.cmd->Pipeline(m_pso.Get());
    ctx.cmd->Topology(D3D_PRIMITIVE_TOPOLOGY_TRIANGLELIST);
    // Mirrors Buildings.hlsl's cbuffer (priors 22).
    struct {
        float origin[4];   // xyz the cell's origin relative to the eye, w = brightness
        float time[4];     // x the layer's now (s), y the pages' fade-in (s)
        uint32_t mesh[4];  // the shapes' draw: first instance row, first task row, tasks, pool slot
    } cb{};
    // The resident cells, and the ones leaving whose boxes the drawn walk does not hold yet.
    std::vector<const Cell*> draw;
    for (const auto& [key, c] : m_cells) draw.push_back(&c);
    for (const auto& [key, wc] : m_warm) {
        if (wc.leftAt > m_drawnCells) draw.push_back(&wc.cell);
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
    if (!m_drawn.vb.Valid()) return;
    cb.origin[0] = static_cast<float>(m_drawn.origin[0] - ctx.camera->px);
    cb.origin[1] = static_cast<float>(m_drawn.origin[1] - ctx.camera->py);
    cb.origin[2] = static_cast<float>(m_drawn.origin[2] - ctx.camera->pz);
    cb.origin[3] = 1.0f;
    cb.time[0] = Now();
    cb.time[1] = kPageFadeS;
    if (m_psoBox && m_drawn.count) {
        // THE TREE'S BOXES: the buildings with no shape, one buffer about the walk's eye, 36 vertices a box.
        GpuScope boxScope(ctx.prof, ctx.cmd->Native(), "buildings.boxes");
        ctx.cmd->Pipeline(m_psoBox.Get());
        ctx.cmd->Topology(D3D_PRIMITIVE_TOPOLOGY_TRIANGLELIST);
        ctx.cmd->GraphicsConstants(1, cb);
        ctx.cmd->GraphicsSrvAt(2, m_drawn.vb.gpu);
        ctx.cmd->Draw(m_drawn.count * 36u, 1, 0, 0);
    }
    if (m_psoShape && m_drawn.tasks && m_pool.Valid()) {
        // THE SHAPES: one mesh group a task (Buildings.hlsl MsShape), the groups in the walk's order.
        GpuScope shapeScope(ctx.prof, ctx.cmd->Native(), "buildings.shapes");
        ctx.cmd->Pipeline(m_psoShape.Get());
        cb.mesh[0] = m_drawn.count * 4u;                          // the first instance's row
        cb.mesh[1] = (m_drawn.count + m_drawn.insts) * 4u;        // the first task's row
        cb.mesh[2] = m_drawn.tasks;
        cb.mesh[3] = m_poolSrv;
        ctx.cmd->GraphicsConstants(1, cb);
        ctx.cmd->GraphicsSrvAt(2, m_drawn.vb.gpu);
        // One DispatchMesh may launch at most 2^22 groups (65535 x 64 here, an even count, so a
        // chunk starts on a whole row of two tasks): a walk with more is drawn in chunks, each
        // its first task row advanced -- one call past the limit is an invalid command.
        constexpr uint32_t kChunk = kChunkGroups;   // even: a chunk starts on a whole row of two tasks
        m_drawLog[m_frame % 3] = {m_frame, m_drawn.cpu};
        for (uint32_t done = 0; done < m_drawn.tasks; done += kChunk) {
            const uint32_t n = (std::min)(m_drawn.tasks - done, kChunk);
            cb.mesh[1] = (m_drawn.count + m_drawn.insts) * 4u + done / 2u;
            cb.mesh[2] = n;
            ctx.cmd->GraphicsConstants(1, cb);
            const uint32_t gx = (std::min)(n, 65535u), gy = (n + 65534u) / 65535u;
            ctx.cmd->DispatchMesh(gx, gy, 1);
        }
    }
}

}  // namespace ga
