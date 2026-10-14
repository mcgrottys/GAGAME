// ================================================================================================
//  HeightTree - THE CPU'S READ OF THE HEIGHT TREE. A reader that wants the ground at a point (the
//  swell's exposure marching a ray, the bed classifier's slope, a solver's bed) reads the tile tree
//  the GPU reads -- the composed root on the pyramid, the half floats the sampler is handed -- and
//  not the sources one by one. So a reader's cost is a cached array read whatever the stack holds
//  (2026-10-14: a 3.4 m composite read through its files made an exposure tile 31 s), and the CPU
//  and the picture stand on one ground.
//
//  THE LAW is HeightPage's (the hull's bed, compose/HeightPage.h), its texels taken from the tree
//  instead of recomputed from the stack: the pyramid by direction (CubeFaceOfDir), PageLoad4's
//  bilinear in doubles -- centres at -0.5, four taps clamped at the face's edge -- over the texels
//  of one level. The level is the coarsest whose texel is no larger than the ground asked: a texel of
//  the tree at that grain IS the area mean the stack answered (the tree folds its children into it),
//  and the bilinear between such texels is the stack's own read at that grain. MEASURED, one level
//  finer (texel at most half the ground) quadrupled the tiles a march touches -- the exposure read
//  4,114 tiles and thrashed a 4,096-tile cache -- for nothing the reader asked for. A level finer than
//  every source's own is the level above, magnified (HIERARCHY 4.20); the reader climbs until a level
//  holds bytes.
//
//  WHAT IT DOES NOT DO: decide what is stored (the tree paints a missing tile as the loader would get
//  it, synchronously, on the caller's thread), or keep anything but decoded tiles: an LRU of at most
//  `maxTiles` (64 KB each, half floats as stored), shared by every thread under one mutex; a tile
//  two threads miss at once is read twice and the first kept. Each thread keeps its last four tiles
//  beside it, so a sample's four taps -- nearly always one tile -- take no lock: MEASURED on the
//  Merrimack scene, a warm read was 0.4-1.3 us through the mutex and the map for every tap.
// ================================================================================================
#pragma once

#include "core/Common.h"
#include "core/Lattice.h"
#include "core/ThreadManager.h"
#include "core/TileAddress.h"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <functional>
#include <condition_variable>
#include <list>
#include <memory>
#include <mutex>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace ga {

class HeightTree {
public:
    // The tree's bytes of a tile on the pyramid -- 1: its half floats; 0: nothing covers it; 2: not now
    // (TileTree::Reader) -- and whether a tile is its parent magnified.
    using ReadFn = std::function<int(const TileRequest&, std::vector<uint8_t>&)>;

    HeightTree(ReadFn read, TileQueryFn magnified, const Lattice& pyramid, size_t maxTiles = 4096)
        : m_read(std::move(read)), m_magnified(std::move(magnified)), m_lat(pyramid), m_max(maxTiles),
          m_serial(NextSerial()) {
        m_mips = 0;
        for (uint32_t d = m_lat.faceDim; d >= m_lat.texW && d >= m_lat.texH; d >>= 1) ++m_mips;
    }

    // The level a reader of `groundResM` reads: the coarsest whose texel is no larger than it.
    uint32_t LevelFor(double groundResM) const {
        const double g0 = m_lat.GroundRes(0);
        const double m = std::floor(std::log2((std::max)(groundResM, g0) / g0));
        return uint32_t(std::clamp(m, 0.0, double(m_mips - 1)));
    }

    // The ground at a point for a reader of `groundResM` metres, in the height's frame (m NAVD88).
    // `levelOffset` moves the level (the probe's planted fault); 0 is the law.
    float Sample(double latRad, double lonRad, double groundResM, int levelOffset = 0) {
        const double cl = std::cos(latRad);
        const double d[3] = {cl * std::cos(lonRad), std::sin(latRad), cl * std::sin(lonRad)};
        double uv[2] = {0.0, 0.0};
        const uint32_t face = CubeFaceOfDir(d, uv);
        uint32_t mip = uint32_t(std::clamp(int(LevelFor(groundResM)) + levelOffset, 0, int(m_mips) - 1));
        // climb while the level is the one above, magnified, at this point's tile
        while (mip + 1 < m_mips) {
            const uint32_t dim = m_lat.faceDim >> mip;
            const TileRequest r{face, mip, uint32_t((std::min)(uv[0] * dim, dim - 1.0)) / m_lat.texW,
                                uint32_t((std::min)(uv[1] * dim, dim - 1.0)) / m_lat.texH};
            if (!TileOf(r).magnified) break;
            ++mip;
        }
        const uint32_t dim = m_lat.faceDim >> mip;
        const double tx = uv[0] * dim - 0.5, ty = uv[1] * dim - 0.5;
        const double fx0 = std::floor(tx), fy0 = std::floor(ty), frx = tx - fx0, fry = ty - fy0;
        const double dmax = double(dim) - 1.0;
        const auto clampI = [&](double t) { return uint32_t((std::min)((std::max)(t, 0.0), dmax)); };
        const uint32_t x0 = clampI(fx0), x1 = clampI(fx0 + 1.0), y0 = clampI(fy0), y1 = clampI(fy0 + 1.0);
        return float((double(Texel(face, mip, x0, y0)) * (1.0 - frx) + double(Texel(face, mip, x1, y0)) * frx) * (1.0 - fry) +
                     (double(Texel(face, mip, x0, y1)) * (1.0 - frx) + double(Texel(face, mip, x1, y1)) * frx) * fry);
    }

    // THE FRAME'S READ: never waits on a paint. The answer is the finest level HELD at or coarser than
    // the one `groundResM` asks for -- the GPU's own rule, the finest resident mip -- and a level not
    // held is asked of a loader thread (Lane::Io) and answered next time. The coarsest level is read
    // in place when it is not held (one tile a face, once a run), so an answer always exists.
    // MEASURED (2026-10-14, flood rail on the composite Merrimack): the water bank's land test and
    // corners read through Sample() on the frame thread waited 69.8 s on 1,024 cold tiles (68 ms a
    // tile, a coarse tile is the fold of its children), 55 ms a frame. Landed() counts the loads
    // that arrived, so a reader that kept an answer knows when a finer one may stand.
    // `provisional`, when given, is set where the answer came from a level coarser than asked because
    // the asked one was not held (a level above, magnified, is the asked level's own answer).
    float SampleResident(double latRad, double lonRad, double groundResM, bool* provisional = nullptr) {
        const double cl = std::cos(latRad);
        const double d[3] = {cl * std::cos(lonRad), std::sin(latRad), cl * std::sin(lonRad)};
        double uv[2] = {0.0, 0.0};
        const uint32_t face = CubeFaceOfDir(d, uv);
        for (uint32_t mip = LevelFor(groundResM); mip < m_mips; ++mip) {
            const uint32_t dim = m_lat.faceDim >> mip;
            const double tx = uv[0] * dim - 0.5, ty = uv[1] * dim - 0.5;
            const double fx0 = std::floor(tx), fy0 = std::floor(ty), frx = tx - fx0, fry = ty - fy0;
            const double dmax = double(dim) - 1.0;
            const auto clampI = [&](double t) { return uint32_t((std::min)((std::max)(t, 0.0), dmax)); };
            const uint32_t xs[2] = {clampI(fx0), clampI(fx0 + 1.0)}, ys[2] = {clampI(fy0), clampI(fy0 + 1.0)};
            const bool last = mip + 1 == m_mips;
            float v[4];
            bool held = true;
            for (int k = 0; k < 4 && held; ++k) {
                const uint32_t ix = xs[k & 1], iy = ys[k >> 1];
                const TileRequest r{face, mip, ix / m_lat.texW, iy / m_lat.texH};
                const Cached* t = last ? &TileOf(r) : HeldOf(r);
                if (!t) {
                    Request(r);
                    held = false;
                    if (provisional) *provisional = true;
                } else if (t->magnified) {
                    held = false;   // the level above, magnified: the answer is the one above
                } else {
                    v[k] = TexelIn(*t, ix, iy);
                }
            }
            if (held) {
                return float((double(v[0]) * (1.0 - frx) + double(v[1]) * frx) * (1.0 - fry) +
                             (double(v[2]) * (1.0 - frx) + double(v[3]) * frx) * fry);
            }
        }
        return 0.0f;   // unreachable: the coarsest level is read in place
    }
    uint64_t Landed() const { return m_landed.load(std::memory_order_acquire); }

    // No load starts after this, and it returns when none is running: the tree a load reads must not
    // die under it (the Assembly closes this before its height TileTree destructs).
    void Close() {
        std::unique_lock<std::mutex> lk(m_mx);
        m_closed = true;
        m_idle.wait(lk, [&] { return m_inflight == 0; });
    }

    // THE HIGHEST GROUND ALONG A RAY (the swell's exposure, compose/ExposureSource.h): the maximum of
    // the tree's texels a great circle crosses from a point, `towardE`/`towardN` the unit direction it
    // leaves in, out to `rangeM`, at the level for `groundM`; never below `floorM` (the reader's own
    // floor: nothing beneath it can matter to it); and it may stop once it reaches `stopAtM`.
    //   STRAIGHT IN THE FACE: a face of the cube is a gnomonic projection, so a great circle is a
    // straight line in the face's coordinates and the texels it crosses are a grid traversal
    // (Amanatides & Woo 1987) -- no trigonometry per step, only at the ends and where it leaves a face.
    //   SKIPPED BY THE BLOCK'S MAXIMUM: each cached tile keeps the maximum of every 16 x 16 block, and
    // the walk crosses blocks first; a block whose maximum cannot raise the running one is passed over
    // whole (the maximum mipmap's skip, Tevs et al. 2008, at one level). The answer is exactly the
    // maximum over every texel the line crosses: a skipped block holds none above it.
    float MaxAlongRay(double latRad, double lonRad, double towardE, double towardN, double rangeM, double groundM,
                      float floorM, float stopAtM) {
        const double R = Lattice::kMercCirc / (2.0 * 3.14159265358979323846);
        const double cl = std::cos(latRad), sl = std::sin(latRad), co = std::cos(lonRad), so = std::sin(lonRad);
        double p[3] = {cl * co, sl, cl * so};
        const double E[3] = {-so, 0.0, co}, N[3] = {-sl * co, cl, -sl * so};
        double t[3];
        for (int k = 0; k < 3; ++k) t[k] = towardE * E[k] + towardN * N[k];
        float best = floorM;
        double left = rangeM / R;   // the arc still to walk, radians
        const uint32_t mip0 = LevelFor(groundM);
        for (int piece = 0; piece < 4 && left > 0.0 && best < stopAtM; ++piece) {
            double uv0[2], q[3];
            const uint32_t face = CubeFaceOfDir(p, uv0);
            // how far along the arc the ray stays on this face: the whole rest, or the crossing found
            // by bisection on the arc (a face is 10,000 km across; a ray of kilometres leaves rarely)
            double s = left;
            const auto at = [&](double a, double out[3]) {
                for (int k = 0; k < 3; ++k) out[k] = p[k] * std::cos(a) + t[k] * std::sin(a);
            };
            at(s, q);
            double uvq[2];
            if (CubeFaceOfDir(q, uvq) != face) {
                double lo = 0.0, hi = s;
                for (int it = 0; it < 40; ++it) {
                    const double mid = 0.5 * (lo + hi);
                    at(mid, q);
                    (CubeFaceOfDir(q, uvq) == face ? lo : hi) = mid;
                }
                s = lo;
                at(s, q);
                FaceUv(face, q, uvq);   // the face's own coordinates of the last point on it
            }
            best = (std::max)(best, WalkFace(face, mip0, uv0, uvq, best, stopAtM));
            // on into the next face: the point past the crossing and the direction the arc has there
            const double a = (std::min)(left, s + 1e-9);
            double p2[3], t2[3];
            for (int k = 0; k < 3; ++k) {
                p2[k] = p[k] * std::cos(a) + t[k] * std::sin(a);
                t2[k] = -p[k] * std::sin(a) + t[k] * std::cos(a);
            }
            for (int k = 0; k < 3; ++k) { p[k] = p2[k]; t[k] = t2[k]; }
            left -= a;
        }
        return best;
    }

    // The instrument: tiles read from the tree (misses) and the time inside those reads. Counted per
    // miss only: MEASURED, a counter per texel and a shared_ptr copy per tap -- two cache lines every
    // loader thread writes -- made a warm read 600 ns in the exposure's march, the stack's 160.
    std::atomic<uint64_t> tileReads{0}, readUs{0}, transient{0};   // transient: reads answered "not now"
    // INSTRUMENT: the reads made on the calling thread (any tree), so a frame loop can see what it waited on
    static inline thread_local uint64_t tlReads = 0, tlReadUs = 0;
    size_t Held() const {
        std::lock_guard<std::mutex> lk(m_mx);
        return m_lru.size();
    }

private:
    // A tile as cached: whether it is its parent magnified (then no bytes: nothing finer than every
    // source is painted, HIERARCHY 4.20), else its half floats as the tree stores them. MEASURED: the
    // magnified question asked per sample instead of per tile made a warm read 3-4x the stack's.
    static constexpr uint32_t kBlock = 16;   // the side of a block whose maximum a tile keeps
    struct Cached {
        bool magnified = false;
        bool kept = true;   // false: a "not now" read, answered once and remembered nowhere
        std::vector<uint8_t> bytes;
        std::vector<float> blockMax;   // (texW / kBlock) x (texH / kBlock), row-major; empty with no bytes
    };

    // A point's coordinates in a given face's plane (the gnomonic projection onto it), for a point
    // that CubeFaceOfDir may have put on its neighbour at the very edge.
    static void FaceUv(uint32_t face, const double d[3], double uv[2]) {
        double n[3], a[3], b[3];
        CubeFaceAxes(face, n, a, b);
        const double pn = d[0] * n[0] + d[1] * n[1] + d[2] * n[2];
        uv[0] = ((d[0] * a[0] + d[1] * a[1] + d[2] * a[2]) / pn) * 0.5 + 0.5;
        uv[1] = ((d[0] * b[0] + d[1] * b[1] + d[2] * b[2]) / pn) * 0.5 + 0.5;
    }

    // The cells of side `cell` a segment (x0, y0) -> (x1, y1) crosses for t in [ta, tb], in order, each
    // with the t it enters and leaves by; `visit` returns false to stop.
    template <class Visit>
    static void Traverse(double x0, double y0, double x1, double y1, double cell, double ta, double tb, Visit&& visit) {
        const double dx = x1 - x0, dy = y1 - y0;
        long long cx = (long long)std::floor((x0 + dx * ta) / cell), cy = (long long)std::floor((y0 + dy * ta) / cell);
        const int sx = dx > 0 ? 1 : dx < 0 ? -1 : 0, sy = dy > 0 ? 1 : dy < 0 ? -1 : 0;
        constexpr double kInf = 1e300;
        double tx = sx ? ((double(cx + (sx > 0 ? 1 : 0)) * cell) - x0) / dx : kInf;
        double ty = sy ? ((double(cy + (sy > 0 ? 1 : 0)) * cell) - y0) / dy : kInf;
        const double ddx = sx ? cell / std::abs(dx) : kInf, ddy = sy ? cell / std::abs(dy) : kInf;
        double t = ta;
        while (t < tb) {
            const double tn = (std::min)((std::min)(tx, ty), tb);
            if (!visit(cx, cy, t, tn)) return;
            if (tx < ty) { cx += sx; tx += ddx; } else { cy += sy; ty += ddy; }
            t = tn;
        }
    }

    // The highest texel the segment uv0 -> uv1 crosses on one face, at `mip` (climbing where the level is
    // magnified at the start), above `running`; blocks whose maximum cannot raise it are passed over.
    float WalkFace(uint32_t face, uint32_t mip, const double uv0[2], const double uv1[2], float running, float stopAtM) {
        while (mip + 1 < m_mips) {
            const uint32_t dim = m_lat.faceDim >> mip;
            const TileRequest r{face, mip, uint32_t((std::min)(uv0[0] * dim, dim - 1.0)) / m_lat.texW,
                                uint32_t((std::min)(uv0[1] * dim, dim - 1.0)) / m_lat.texH};
            if (!TileOf(r).magnified) break;
            ++mip;
        }
        const uint32_t dim = m_lat.faceDim >> mip;
        const double x0 = uv0[0] * dim, y0 = uv0[1] * dim, x1 = uv1[0] * dim, y1 = uv1[1] * dim;
        const long long last = (long long)dim - 1;
        const uint32_t bw = m_lat.texW / kBlock;
        Traverse(x0, y0, x1, y1, double(kBlock), 0.0, 1.0, [&](long long bx, long long by, double ta, double tb) {
            if (bx < 0 || by < 0 || bx * kBlock > last || by * kBlock > last) return true;
            const uint64_t ux = uint64_t(bx) * kBlock, uy = uint64_t(by) * kBlock;
            const TileRequest r{face, mip, uint32_t(ux / m_lat.texW), uint32_t(uy / m_lat.texH)};
            const Cached& c = TileOf(r);
            if (c.blockMax.empty()) {   // nothing covers it (the stack's 0), or the level is magnified there
                if (c.magnified) {
                    const double mu = (x0 + (x1 - x0) * 0.5 * (ta + tb)) / dim, mv = (y0 + (y1 - y0) * 0.5 * (ta + tb)) / dim;
                    double d[3];
                    ComposeCubeDir(face, mu, mv, d);
                    const double lat = std::asin((std::max)(-1.0, (std::min)(1.0, d[1]))), lon = std::atan2(d[2], d[0]);
                    running = (std::max)(running, Sample(lat, lon, m_lat.GroundRes(mip + 1)));
                } else {
                    running = (std::max)(running, 0.0f);
                }
                return running < stopAtM;
            }
            const uint32_t lbx = uint32_t(ux % m_lat.texW) / kBlock, lby = uint32_t(uy % m_lat.texH) / kBlock;
            if (c.blockMax[size_t(lby) * bw + lbx] <= running) return true;   // nothing here can raise it
            Traverse(x0, y0, x1, y1, 1.0, ta, tb, [&](long long ix, long long iy, double, double) {
                if (ix < 0 || iy < 0 || ix > last || iy > last) return true;
                uint16_t h;
                std::memcpy(&h, c.bytes.data() + (size_t(uint64_t(iy) % m_lat.texH) * m_lat.texW + size_t(uint64_t(ix) % m_lat.texW)) * 2, 2);
                running = (std::max)(running, HalfToFloat(h));
                return running < stopAtM;
            });
            return running < stopAtM;
        });
        return running;
    }
    using Tile = std::shared_ptr<const Cached>;

    float Texel(uint32_t face, uint32_t mip, uint32_t ix, uint32_t iy) {
        return TexelIn(TileOf(TileRequest{face, mip, ix / m_lat.texW, iy / m_lat.texH}), ix, iy);
    }
    float TexelIn(const Cached& t, uint32_t ix, uint32_t iy) const {
        if (t.bytes.size() < size_t(m_lat.texW) * m_lat.texH * 2) return 0.0f;   // nothing covers: the stack's 0
        uint16_t h;
        std::memcpy(&h, t.bytes.data() + (size_t(iy % m_lat.texH) * m_lat.texW + ix % m_lat.texW) * 2, 2);
        return HalfToFloat(h);
    }

    struct Recent {
        uint64_t owner = 0, key = 0;
        Tile tile;
    };
    static uint64_t NextSerial() {
        static std::atomic<uint64_t> n{0};
        return ++n;
    }

    // The tile, held alive by this thread's slot until the slot is reused: the four taps of a sample
    // read through the reference, never touching a count another thread writes.
    const Cached& TileOf(const TileRequest& r) {
        const uint64_t key = KeyOf(r);
        thread_local Recent recent[4];
        thread_local uint32_t next = 0;
        for (const Recent& e : recent) {
            if (e.owner == m_serial && e.key == key) return *e.tile;
        }
        Recent& slot = recent[next++ & 3u];
        Tile t = Shared(key, r);
        slot.owner = t->kept ? m_serial : 0;   // a "not now" answers this tap and is not remembered
        slot.key = key;
        slot.tile = std::move(t);
        return *slot.tile;
    }

    static uint64_t KeyOf(const TileRequest& r) {
        return (uint64_t(r.face) << 61) | (uint64_t(r.mip) << 56) | (uint64_t(r.y) << 28) | r.x;
    }

    // The tile if this reader holds it, else null; never reads.
    const Cached* HeldOf(const TileRequest& r) {
        const uint64_t key = KeyOf(r);
        thread_local Recent recent[4];
        thread_local uint32_t next = 0;
        for (const Recent& e : recent) {
            if (e.owner == m_serial && e.key == key) return e.tile.get();
        }
        Tile t;
        {
            std::lock_guard<std::mutex> lk(m_mx);
            auto it = m_index.find(key);
            if (it == m_index.end()) return nullptr;
            m_lru.splice(m_lru.begin(), m_lru, it->second);
            t = it->second->second;
        }
        Recent& slot = recent[next++ & 3u];
        slot.owner = m_serial;
        slot.key = key;
        slot.tile = std::move(t);
        return slot.tile.get();
    }

    // Ask a loader thread for a tile not held: once while it is asked, never after Close().
    void Request(const TileRequest& r) {
        const uint64_t key = KeyOf(r);
        {
            std::lock_guard<std::mutex> lk(m_mx);
            if (m_closed || m_index.count(key) || !m_pending.insert(key).second) return;
            ++m_inflight;
        }
        Threads().Submit(Lane::Io, "height.tree", [this, key, r] {
            bool closed;
            {
                std::lock_guard<std::mutex> lk(m_mx);
                closed = m_closed;
            }
            if (!closed) Shared(key, r);
            {
                std::lock_guard<std::mutex> lk(m_mx);
                m_pending.erase(key);
                --m_inflight;
            }
            m_landed.fetch_add(1, std::memory_order_release);
            m_idle.notify_all();
        });
    }

    Tile Shared(uint64_t key, const TileRequest& r) {
        {
            std::lock_guard<std::mutex> lk(m_mx);
            auto it = m_index.find(key);
            if (it != m_index.end()) {
                m_lru.splice(m_lru.begin(), m_lru, it->second);
                return it->second->second;
            }
        }
        auto c = std::make_shared<Cached>();
        c->magnified = m_magnified && m_magnified(r);
        int answer = 1;
        if (!c->magnified) {
            const auto t0 = std::chrono::steady_clock::now();
            answer = m_read(r, c->bytes);
            if (answer != 1) c->bytes.clear();
            if (c->bytes.size() >= size_t(m_lat.texW) * m_lat.texH * 2) {
                // the block maxima, once a tile: what MaxAlongRay passes over
                const uint32_t bw = m_lat.texW / kBlock, bh = m_lat.texH / kBlock;
                c->blockMax.assign(size_t(bw) * bh, -1e30f);
                for (uint32_t y = 0; y < m_lat.texH; ++y) {
                    for (uint32_t x = 0; x < m_lat.texW; ++x) {
                        uint16_t h;
                        std::memcpy(&h, c->bytes.data() + (size_t(y) * m_lat.texW + x) * 2, 2);
                        float& m = c->blockMax[size_t(y / kBlock) * bw + x / kBlock];
                        m = (std::max)(m, HalfToFloat(h));
                    }
                }
            }
            const uint64_t us = uint64_t(std::chrono::duration<double, std::micro>(std::chrono::steady_clock::now() - t0).count());
            readUs.fetch_add(us, std::memory_order_relaxed);
            ++tlReads;
            tlReadUs += us;
            tileReads.fetch_add(1, std::memory_order_relaxed);
        }
        // Not now is an answer for this read alone: kept by no cache, so the next read asks again.
        if (answer == 2) {
            transient.fetch_add(1, std::memory_order_relaxed);
            c->kept = false;
            return c;
        }
        std::lock_guard<std::mutex> lk(m_mx);
        auto it = m_index.find(key);
        if (it != m_index.end()) return it->second->second;
        m_lru.emplace_front(key, std::move(c));
        m_index[key] = m_lru.begin();
        if (m_lru.size() > m_max) {
            m_index.erase(m_lru.back().first);
            m_lru.pop_back();
        }
        return m_lru.front().second;
    }

    ReadFn m_read;
    TileQueryFn m_magnified;
    Lattice m_lat;
    size_t m_max;
    const uint64_t m_serial;   // names this reader's tiles in the threads' own four
    uint32_t m_mips = 1;
    mutable std::mutex m_mx;
    std::condition_variable m_idle;
    std::unordered_set<uint64_t> m_pending;   // tiles asked of a loader and not yet back
    uint32_t m_inflight = 0;
    bool m_closed = false;
    std::atomic<uint64_t> m_landed{0};
    std::list<std::pair<uint64_t, Tile>> m_lru;
    std::unordered_map<uint64_t, std::list<std::pair<uint64_t, Tile>>::iterator> m_index;
};

}  // namespace ga
