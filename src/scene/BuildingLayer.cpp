#include "scene/BuildingLayer.h"

#include "compose/BuildingMoments.h"

#include "core/Common.h"
#include "core/ThreadManager.h"
#include "hal/GpuProfiler.h"
#include "hal/PixEvents.h"
#include "hal/Pipeline.h"
#include "hal/Shader.h"
#include "render/Camera.h"

#include <algorithm>
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
    if (!BuildPso(gpu, sc)) throw std::runtime_error("building PSO failed");
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
    return hal::Reload(m_psoBox, [&] { return hal::BuildGraphics(gpu, d, "buildings.boxes"); }, "buildings.boxes") && ok;
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
    u3[0] = e3[1] * n3[2] - e3[2] * n3[1];
    u3[1] = e3[2] * n3[0] - e3[0] * n3[2];
    u3[2] = e3[0] * n3[1] - e3[1] * n3[0];
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
                     [shared = m_shared, lodf = m_lod, place = m_place, ground = m_ground, k, lp = *lp, L] {
        auto page = std::make_shared<Page>();
        std::vector<LodNode> nodes;
        std::vector<LodBuilding> blds;
        if (!shared->cancel.load() && lodf->Read(L, lp, nodes, blds)) {
            page->L = L;
            // A box stood on the composed ground at its centroid, on the prisms' own axes.
            auto stand = [&](double lat, double lon, double zc, double hz, double a1, double a2, double hd, RtBox& o,
                             double alpha = 1.0) {
                o.alpha = static_cast<float>(std::clamp(alpha, 0.0, 1.0));
                if (!(hz > 0.0)) return;
                double p[3], e[3], n[3], u[3];
                PointFrame(place, lat, lon, ground(lat, lon) + zc, p, e, n, u);
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
            for (size_t i = 0; i < blds.size(); ++i) {
                double lat, lon, zc, hz, a1, a2, hd;
                int cx, cy;
                LodUnpack(blds[i], lat, lon, zc, hz, a1, a2, hd, cx, cy);
                RtBld& b = page->blds[i];
                stand(lat, lon, zc, hz, a1, a2, hd, b.box);
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
                r.rhoMin = n.ownRhoMin;
                const double latc = (n.y + 0.5) * Q, lonc = (n.x + 0.5) * Q;
                place(latc, lonc, ground(latc, lonc), r.c);
                // THE FOLDS: its own, its descendants', and both together -- the moments added
                // (FOLD), the sum's box (BoxOf), in the node's own frame.
                Moments all;
                auto fold = [&](const LodBox& b, RtBox& o, Key& cell) {
                    if (!(b.hz > 0.0f)) return;
                    const double lat = b.lat7 * 1e-7, lon = b.lon7 * 1e-7;
                    const double s = std::pow((std::max)(double(b.cover), 1e-6), kFoldShrink);
                    stand(lat, lon, b.zc, b.hz, b.a1 * s, b.a2 * s, b.heading, o, b.cover / (s * s));
                    cell = CellOf(lat, lon);
                    all += LodBoxMoments(lat, lon, b.zc, b.hz, b.a1, b.a2, b.heading, latc, lonc, b.cover);
                };
                fold(n.own, r.own, r.ownCell);
                fold(n.desc, r.desc, r.descCell);
                float top = 0.0f;
                if (!all.Empty()) {
                    const MomentBox mb = BoxOf(all);
                    constexpr double kDeg = 3.14159265358979323846 / 180.0;
                    const double mx = std::cos(latc * kDeg) * 6371008.8 * kDeg, my = 6371008.8 * kDeg;
                    const double lat = latc + mb.c[1] / my, lon = lonc + mb.c[0] / mx;
                    const double s = std::pow((std::max)(mb.cover, 1e-6), kFoldShrink);
                    stand(lat, lon, mb.c[2], mb.half[2], mb.spread[0] * s, mb.spread[1] * s, mb.heading, r.all, mb.cover / (s * s));
                    r.allCell = CellOf(lat, lon);
                    top = static_cast<float>(mb.c[2] + mb.half[2]);
                }
                // The quad's corners and what stands on it, from its centre.
                r.rad = static_cast<float>(0.75 * Qm + (std::max)(top, (std::max)(n.own.zc + n.own.hz, n.desc.zc + n.desc.hz)) + 50.0);
                page->at[{n.x, n.y}] = static_cast<uint32_t>(i);
            }
            page->bytes = nodes.size() * (sizeof(RtNode) + 48) + blds.size() * sizeof(RtBld);
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
    for (auto& [k, p] : pages) {
        m_pagePending.erase(k);
        m_pageBytes += p->bytes;
        ++m_pagesLoaded;
        m_pageUsed[k] = m_frame;
        m_pages[k] = std::move(p);
        ++m_pagesVersion;
    }
    for (Walked& w : walked) {
        if (w.count && w.vb.Valid()) {   // the cells' own copy and barrier (Upload)
            ID3D12GraphicsCommandList* cl = ctx.cmd->Native();
            cl->CopyBufferRegion(w.vb.res.Get(), 0, w.staging.res.Get(), 0, w.staging.size);
            D3D12_RESOURCE_BARRIER br{};
            br.Type = D3D12_RESOURCE_BARRIER_TYPE_TRANSITION;
            br.Transition.pResource = w.vb.res.Get();
            br.Transition.Subresource = D3D12_RESOURCE_BARRIER_ALL_SUBRESOURCES;
            br.Transition.StateBefore = D3D12_RESOURCE_STATE_COPY_DEST;
            br.Transition.StateAfter = D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE;
            cl->ResourceBarrier(1, &br);
            m_retired.push_back({std::move(w.staging), m_frame});
        }
        if (m_drawn.vb.Valid()) m_retired.push_back({std::move(m_drawn.vb), m_frame});
        for (int i = 0; i < 3; ++i) m_drawn.origin[i] = w.origin[i];
        m_drawn.count = w.vb.Valid() ? w.count : 0;
        m_drawn.vb = std::move(w.vb);
        for (const PageKey& k : w.used) m_pageUsed[k] = m_frame;
        // The pages the walk wanted, the largest on screen first.
        for (const auto& [px, k] : w.wants) {
            if (m_shared->pageInflight.load() >= kPageInFlight) break;
            if (m_pages.count(k) || m_pagePending.count(k)) continue;
            LoadPage(k);
        }
        ++m_walks;
        if (m_walks <= 3 || m_walks % 50 == 0) {
            Log("[buildings] frame %llu walk %llu: %u boxes (%llu buildings, %llu folds) over %llu nodes in %.1f ms; %zu pages "
                "(%.0f MB), %zu asked", static_cast<unsigned long long>(m_frame), static_cast<unsigned long long>(m_walks),
                m_drawn.count, static_cast<unsigned long long>(w.singles), static_cast<unsigned long long>(w.folds),
                static_cast<unsigned long long>(w.nodes), w.ms, m_pages.size(), m_pageBytes / 1048576.0, w.wants.size());
        }
    }
    // 2. The budget: the page no walk has used for longest goes first.
    while (m_pageBytes > kPageBytes && !m_pages.empty()) {
        auto old = m_pages.begin();
        for (auto it = m_pages.begin(); it != m_pages.end(); ++it) {
            if (m_pageUsed[it->first] < m_pageUsed[old->first]) old = it;
        }
        if (m_pageUsed[old->first] + 2 >= m_frame) break;   // all in use: over budget, not thrashing
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
        double eye[3], fwd[3], lat, lon, h, pixAng, halfDiag, tauB;
        std::map<PageKey, std::shared_ptr<const Page>> pages;
        std::set<Key> cells;
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
    in.tauB = m_lodPixels;
    in.pages = m_pages;
    for (const auto& [k, c] : m_cells) in.cells.insert(k);
    m_shared->walking.store(1);
    Threads().Submit(Lane::Io, "buildings.walk", [shared = m_shared, lodf = m_lod, gpu = ctx.gpu, in = std::move(in)] {
        const auto t0 = std::chrono::steady_clock::now();
        Walked w;
        for (int i = 0; i < 3; ++i) w.origin[i] = in.eye[i];
        std::vector<Box> out;
        std::set<PageKey> used;
        std::map<PageKey, double> wants;
        constexpr double kR = 6371008.8;
        // The horizon, and what stands above it a little beyond.
        const double dh = std::sqrt((std::max)(in.h, 0.0) * (2.0 * kR + (std::max)(in.h, 0.0))) + 120000.0;
        auto pageOf = [](int L, int x, int y) { return PageKey{L, FloorDivI(x, lod::kPageX), FloorDivI(y, lod::PageY(L))}; };
        auto loaded = [&](const PageKey& k) -> const Page* {
            auto it = in.pages.find(k);
            return it == in.pages.end() ? nullptr : it->second.get();
        };
        auto emit = [&](const RtBox& b) {
            if (!b.valid || out.size() >= kMaxBoxes) return;
            Box x{};
            for (int i = 0; i < 3; ++i) {
                x.c[i] = static_cast<float>(b.p[i] - in.eye[i]);
                x.alpha = b.alpha;
                x.u[i] = b.u[i];
                x.v[i] = b.v[i];
                x.w[i] = b.w[i];
            }
            out.push_back(x);
        };
        std::function<void(int, const Page&, const RtNode&)> visit = [&](int L, const Page& pg, const RtNode& n) {
            ++w.nodes;
            double v[3], dist = 0.0;
            for (int i = 0; i < 3; ++i) {
                v[i] = n.c[i] - in.eye[i];
                dist += v[i] * v[i];
            }
            dist = std::sqrt(dist);
            if (dist - n.rad > dh) return;   // under the horizon
            if (dist > n.rad) {              // outside the view's cone
                const double ca = (v[0] * in.fwd[0] + v[1] * in.fwd[1] + v[2] * in.fwd[2]) / dist;
                if (std::acos(std::clamp(ca, -1.0, 1.0)) - std::asin((std::min)(1.0, n.rad / dist)) > in.halfDiag) return;
            }
            const double dn = (std::max)(dist - n.rad, 1.0);
            const double px = lod::QuadDeg(L) * lod::kMetresPerDeg / (dn * in.pixAng);
            if (px < kQuadPixels) {          // the node is a few pixels: its fold stands for it all
                if (!in.cells.count(n.allCell)) {
                    emit(n.all);
                    ++w.folds;
                }
                return;
            }
            if (n.ownN) {
                if (2.0 * n.rhoMin / (dn * in.pixAng) >= in.tauB) {   // each covers a pixel: one by one
                    for (uint32_t k = n.own0; k < n.own0 + n.ownN && k < pg.blds.size(); ++k) {
                        if (in.cells.count(pg.blds[k].cell)) continue;   // its prisms are drawn
                        emit(pg.blds[k].box);
                        ++w.singles;
                    }
                } else if (!in.cells.count(n.ownCell)) {
                    emit(n.own);
                    ++w.folds;
                }
            }
            if (L == lod::kLmin || !n.desc.valid) return;
            // The children's pages: all loaded, the children are walked; else the descendants'
            // fold stands in while the missing pages are asked for.
            PageKey kids[4];
            int nk = 0;
            bool ready = true;
            for (int j = 0; j < 2; ++j) {
                for (int i = 0; i < 2; ++i) {
                    const PageKey k = pageOf(L - 1, 2 * n.x + i, 2 * n.y + j);
                    if (std::find(kids, kids + nk, k) != kids + nk) continue;
                    kids[nk++] = k;
                    if (!lodf->Find(L - 1, std::get<1>(k), std::get<2>(k))) continue;   // nothing there
                    if (!loaded(k)) {
                        ready = false;
                        double& want = wants[k];
                        want = (std::max)(want, px);
                    }
                }
            }
            if (!ready) {
                if (!in.cells.count(n.descCell)) {
                    emit(n.desc);
                    ++w.folds;
                }
                return;
            }
            for (int j = 0; j < 2; ++j) {
                for (int i = 0; i < 2; ++i) {
                    const int cx = 2 * n.x + i, cy = 2 * n.y + j;
                    const PageKey k = pageOf(L - 1, cx, cy);
                    const Page* cp = loaded(k);
                    if (!cp) continue;
                    auto it = cp->at.find({cx, cy});
                    if (it == cp->at.end()) continue;
                    used.insert(k);
                    visit(L - 1, *cp, cp->nodes[it->second]);
                }
            }
        };
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
                        wants[k] = 1e12;   // the roots before anything
                        continue;
                    }
                    used.insert(k);
                    auto it = p->at.find({x, y});
                    if (it != p->at.end()) visit(L, *p, p->nodes[it->second]);
                }
            }
        }
        // Far to near: the folds blend over what stands behind them (painter's order, depth still tested).
        std::sort(out.begin(), out.end(), [](const Box& a, const Box& b) {
            return a.c[0] * a.c[0] + a.c[1] * a.c[1] + a.c[2] * a.c[2] > b.c[0] * b.c[0] + b.c[1] * b.c[1] + b.c[2] * b.c[2];
        });
        w.count = static_cast<uint32_t>(out.size());
        if (w.count && !shared->cancel.load()) {
            const uint64_t bytes = out.size() * sizeof(Box);
            w.staging = gpu->CreateUploadBuffer(bytes, L"buildings.walk.staging");
            std::memcpy(w.staging.cpu, out.data(), bytes);
            w.vb = gpu->CreateDefaultBuffer(nullptr, bytes, L"buildings.walk");
        }
        for (const auto& [k, px] : wants) w.wants.push_back({px, k});
        std::sort(w.wants.begin(), w.wants.end(), [](const auto& a, const auto& b) { return a.first > b.first; });
        w.used.assign(used.begin(), used.end());
        w.ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
        std::lock_guard<std::mutex> lk(shared->mx);
        shared->walked.push_back(std::move(w));
        shared->walking.store(0);
    });
}

void BuildingLayer::Simulate(const FrameContext& ctx) {
    ++m_frame;
    // Buffers no frame in flight can still read.
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
            m_warm[it->first] = {std::move(it->second), m_frame};
            it = m_cells.erase(it);
            ++m_cellsVersion;
        } else {
            ++it;
        }
    }
    while (m_warmBytes > kWarmBytes && !m_warm.empty()) {
        auto old = std::min_element(m_warm.begin(), m_warm.end(),
                                    [](const auto& a, const auto& b) { return a.second.frame < b.second.frame; });
        m_warmBytes -= uint64_t(old->second.cell.count) * sizeof(Vertex);
        if (old->second.cell.vb.Valid()) m_retired.push_back({std::move(old->second.cell.vb), m_frame});
        m_warm.erase(old);
    }
    Upload(ctx);
    Want(ctx.gpu, lat, lon, h);
    if (m_lod && m_lod->Valid() && ctx.height > 0) TreeFrame(ctx, lat, lon, h);
}

void BuildingLayer::Render(const FrameContext& ctx) {
    if (!m_pso || (m_cells.empty() && !m_drawn.count) || !ctx.camera) return;
    PixScope scope(ctx.cmd->Native(), "buildings (solids -> prisms, cell origins about the eye)");
    GpuScope gscope(ctx.prof, ctx.cmd->Native(), "buildings");
    ctx.cmd->Pipeline(m_pso.Get());
    ctx.cmd->Topology(D3D_PRIMITIVE_TOPOLOGY_TRIANGLELIST);
    // Mirrors Buildings.hlsl's cbuffer (priors 22).
    struct {
        float origin[4];   // xyz the cell's origin relative to the eye, w = brightness
    } cb{};
    for (const auto& [key, c] : m_cells) {
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
    if (!m_psoBox || !m_drawn.count) return;
    // THE TREE'S BOXES: the last walk's, one buffer about the eye it stood at, 36 vertices a box.
    ctx.cmd->Pipeline(m_psoBox.Get());
    cb.origin[0] = static_cast<float>(m_drawn.origin[0] - ctx.camera->px);
    cb.origin[1] = static_cast<float>(m_drawn.origin[1] - ctx.camera->py);
    cb.origin[2] = static_cast<float>(m_drawn.origin[2] - ctx.camera->pz);
    cb.origin[3] = 1.0f;
    ctx.cmd->GraphicsConstants(1, cb);
    ctx.cmd->GraphicsSrvAt(2, m_drawn.vb.gpu);
    ctx.cmd->Draw(m_drawn.count * 36u, 1, 0, 0);
}

}  // namespace ga
