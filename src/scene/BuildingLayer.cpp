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
    for (int i = 0; i < 2000 && (m_shared->inflight.load() > 0 || m_shared->farInflight.load() > 0); ++i) {
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
    d.vs = sc.Compile(path, L"VsBox", L"vs_6_0");   // the far boxes: the same light, the same depth
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

void BuildingLayer::WantFar(Gpu* gpu, double latDeg, double lonDeg, double h, double pixAng) {
    constexpr double kDeg = 3.14159265358979323846 / 180.0;
    std::vector<std::pair<double, FarKey>> want;
    for (int k = lod::kMinLevel; k <= lod::kMaxLevel; ++k) {
        if (!m_lod->Tiles(k)) continue;
        const double reach = LodReach(k, m_lod->Rho0(), m_lodPixels, pixAng);
        if (h > reach) continue;   // the eye's altitude alone puts the whole level under its pixels
        const double T = lod::TileDeg(k);
        const int ex = static_cast<int>(std::floor(lonDeg / T)), ey = static_cast<int>(std::floor(latDeg / T));
        const double tileM = T * 111195.0 * (std::max)(0.2, std::cos(latDeg * kDeg));
        const int spanX = (std::min)(static_cast<int>(std::ceil(reach / tileM)) + 1, 400);
        const int spanY = (std::min)(static_cast<int>(std::ceil(reach / (T * 111195.0))) + 1, 400);
        for (int dy = -spanY; dy <= spanY; ++dy) {
            for (int dx = -spanX; dx <= spanX; ++dx) {
                const int tx = ex + dx, ty = ey + dy;
                const FarKey key{k, tx, ty};
                if (m_far.count(key) || m_farPending.count(key) || !m_lod->Find(k, tx, ty)) continue;
                const double r = ReachBox(tx * T, ty * T, T, latDeg, lonDeg, h);
                if (r <= reach) want.push_back({r / reach, key});   // nearest within its own reach first
            }
        }
    }
    std::sort(want.begin(), want.end());
    for (const auto& [r, key] : want) {
        if (m_shared->farInflight.load() >= kFarInFlight) break;
        const LodTile* tile = m_lod->Find(std::get<0>(key), std::get<1>(key), std::get<2>(key));
        if (!tile) continue;
        m_farPending.insert(key);
        m_shared->farInflight.fetch_add(1);
        Threads().Submit(Lane::Io, "buildings.far",
                         [shared = m_shared, lodf = m_lod, place = m_place, ground = m_ground, gpu, key, t = *tile] {
            if (!shared->cancel.load()) {
                const auto t0 = std::chrono::steady_clock::now();
                const int k = std::get<0>(key);
                const double T = lod::TileDeg(k);
                FarBuilt b;
                b.key = key;
                place((std::get<2>(key) + 0.5) * T, (std::get<1>(key) + 0.5) * T, 0.0, b.origin);
                const std::vector<LodRecord> recs = lodf->Read(k, t);
                std::vector<Box> boxes;
                boxes.reserve(recs.size());
                for (const LodRecord& r : recs) {
                    if (shared->cancel.load()) break;
                    const double lat = r.lat7 * 1e-7, lon = r.lon7 * 1e-7;
                    double p[3], e[3], n[3], u[3];
                    PointFrame(place, lat, lon, ground(lat, lon) + r.zc, p, e, n, u);
                    const double ch = std::cos(r.heading), sh = std::sin(r.heading);
                    Box x{};
                    for (int i = 0; i < 3; ++i) {
                        x.c[i] = static_cast<float>(p[i] - b.origin[i]);
                        x.u[i] = static_cast<float>((ch * e[i] + sh * n[i]) * r.a1);
                        x.v[i] = static_cast<float>((-sh * e[i] + ch * n[i]) * r.a2);
                        x.w[i] = static_cast<float>(u[i] * r.hz);
                    }
                    const Key cell{r.cx, r.cy};
                    if (b.runs.empty() || b.runs.back().cell != cell) b.runs.push_back({cell, static_cast<uint32_t>(boxes.size()), 0});
                    ++b.runs.back().count;
                    boxes.push_back(x);
                }
                b.count = static_cast<uint32_t>(boxes.size());
                if (b.count && !shared->cancel.load()) {
                    const uint64_t bytes = boxes.size() * sizeof(Box);
                    b.staging = gpu->CreateUploadBuffer(bytes, L"buildings.far.staging");
                    std::memcpy(b.staging.cpu, boxes.data(), bytes);
                    b.vb = gpu->CreateDefaultBuffer(nullptr, bytes, L"buildings.far");
                }
                b.ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
                std::lock_guard<std::mutex> lk(shared->mx);
                shared->farDone.push_back(std::move(b));
            }
            shared->farInflight.fetch_sub(1);
        });
    }
}

void BuildingLayer::UploadFar(const FrameContext& ctx) {
    std::vector<FarBuilt> done;
    {
        std::lock_guard<std::mutex> lk(m_shared->mx);
        const size_t n = (std::min)(m_shared->farDone.size(), static_cast<size_t>(kUploadsPerFrame));
        done.assign(std::make_move_iterator(m_shared->farDone.begin()),
                    std::make_move_iterator(m_shared->farDone.begin() + static_cast<std::ptrdiff_t>(n)));
        m_shared->farDone.erase(m_shared->farDone.begin(), m_shared->farDone.begin() + static_cast<std::ptrdiff_t>(n));
    }
    for (FarBuilt& b : done) {
        m_farPending.erase(b.key);
        Far f{};
        for (int i = 0; i < 3; ++i) f.origin[i] = b.origin[i];
        f.count = b.vb.Valid() ? b.count : 0;
        f.runs = std::move(b.runs);
        if (f.count) {   // the cells' own copy and barrier (Upload)
            f.vb = std::move(b.vb);
            ID3D12GraphicsCommandList* cl = ctx.cmd->Native();
            cl->CopyBufferRegion(f.vb.res.Get(), 0, b.staging.res.Get(), 0, b.staging.size);
            D3D12_RESOURCE_BARRIER br{};
            br.Type = D3D12_RESOURCE_BARRIER_TYPE_TRANSITION;
            br.Transition.pResource = f.vb.res.Get();
            br.Transition.Subresource = D3D12_RESOURCE_BARRIER_ALL_SUBRESOURCES;
            br.Transition.StateBefore = D3D12_RESOURCE_STATE_COPY_DEST;
            br.Transition.StateAfter = D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE;
            cl->ResourceBarrier(1, &br);
            m_retired.push_back({std::move(b.staging), m_frame});
        }
        ++m_farBuilt;
        Log("[buildings] frame %llu far level %d tile %d,%d: %u boxes in %zu detail cells, built %.0f ms (%zu far tiles)",
            static_cast<unsigned long long>(m_frame), std::get<0>(b.key), std::get<1>(b.key), std::get<2>(b.key), f.count,
            f.runs.size(), b.ms, m_far.size() + 1);
        m_far[b.key] = std::move(f);
    }
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
    if (m_lod && m_lod->Valid() && ctx.height > 0) {
        // The far boxes: each level's reach at this camera; tiles past kKeep x it dropped (small: no
        // warm pool); each kept tile told whether a detail cell can stand in it, so the draw splits
        // it into runs only where it must.
        const double pixAng = double(ctx.camera->fovY) / double(ctx.height);
        for (auto it = m_far.begin(); it != m_far.end();) {
            const int k = std::get<0>(it->first);
            const double T = lod::TileDeg(k);
            const double r = ReachBox(std::get<1>(it->first) * T, std::get<2>(it->first) * T, T, lat, lon, h);
            if (r > kKeep * LodReach(k, m_lod->Rho0(), m_lodPixels, pixAng)) {
                if (it->second.vb.Valid()) m_retired.push_back({std::move(it->second.vb), m_frame});
                it = m_far.erase(it);
                continue;
            }
            it->second.nearDetail = r <= kKeep * m_radius + 1.0;
            ++it;
        }
        UploadFar(ctx);
        WantFar(ctx.gpu, lat, lon, h, pixAng);
    }
}

void BuildingLayer::Render(const FrameContext& ctx) {
    if (!m_pso || (m_cells.empty() && m_far.empty()) || !ctx.camera) return;
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
    if (!m_psoBox || m_far.empty()) return;
    // THE FAR BOXES: 36 vertices a box; where a tile can hold a resident detail cell, only the runs
    // of the cells that are not resident (the cells' prisms draw the rest).
    ctx.cmd->Pipeline(m_psoBox.Get());
    uint64_t boxes = 0, tiles = 0;
    for (const auto& [key, f] : m_far) {
        if (!f.count) continue;
        cb.origin[0] = static_cast<float>(f.origin[0] - ctx.camera->px);
        cb.origin[1] = static_cast<float>(f.origin[1] - ctx.camera->py);
        cb.origin[2] = static_cast<float>(f.origin[2] - ctx.camera->pz);
        cb.origin[3] = 1.0f;
        ctx.cmd->GraphicsConstants(1, cb);
        ctx.cmd->GraphicsSrvAt(2, f.vb.gpu);
        ++tiles;
        if (!f.nearDetail) {
            ctx.cmd->Draw(f.count * 36u, 1, 0, 0);
            boxes += f.count;
            continue;
        }
        uint32_t first = 0, n = 0;   // consecutive drawable runs, merged into one draw
        for (const Run& r : f.runs) {
            if (m_cells.count(r.cell)) {
                if (n) ctx.cmd->Draw(n * 36u, 1, first * 36u, 0);
                boxes += n;
                n = 0;
                continue;
            }
            if (!n) first = r.first;
            n += r.count;
        }
        if (n) ctx.cmd->Draw(n * 36u, 1, first * 36u, 0);
        boxes += n;
    }
    m_farBoxesDrawn = boxes;
    m_farTilesDrawn = tiles;
}

}  // namespace ga
