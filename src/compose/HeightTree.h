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
#include "core/TileAddress.h"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <functional>
#include <list>
#include <memory>
#include <mutex>
#include <unordered_map>
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

    // The instrument: tiles read from the tree (misses) and the time inside those reads. Counted per
    // miss only: MEASURED, a counter per texel and a shared_ptr copy per tap -- two cache lines every
    // loader thread writes -- made a warm read 600 ns in the exposure's march, the stack's 160.
    std::atomic<uint64_t> tileReads{0}, readUs{0}, transient{0};   // transient: reads answered "not now"
    size_t Held() const {
        std::lock_guard<std::mutex> lk(m_mx);
        return m_lru.size();
    }

private:
    // A tile as cached: whether it is its parent magnified (then no bytes: nothing finer than every
    // source is painted, HIERARCHY 4.20), else its half floats as the tree stores them. MEASURED: the
    // magnified question asked per sample instead of per tile made a warm read 3-4x the stack's.
    struct Cached {
        bool magnified = false;
        bool kept = true;   // false: a "not now" read, answered once and remembered nowhere
        std::vector<uint8_t> bytes;
    };
    using Tile = std::shared_ptr<const Cached>;

    float Texel(uint32_t face, uint32_t mip, uint32_t ix, uint32_t iy) {
        const TileRequest r{face, mip, ix / m_lat.texW, iy / m_lat.texH};
        const Cached& t = TileOf(r);
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
        const uint64_t key = (uint64_t(r.face) << 61) | (uint64_t(r.mip) << 56) | (uint64_t(r.y) << 28) | r.x;
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
            readUs.fetch_add(uint64_t(std::chrono::duration<double, std::micro>(std::chrono::steady_clock::now() - t0).count()),
                             std::memory_order_relaxed);
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
    std::list<std::pair<uint64_t, Tile>> m_lru;
    std::unordered_map<uint64_t, std::list<std::pair<uint64_t, Tile>>::iterator> m_index;
};

}  // namespace ga
