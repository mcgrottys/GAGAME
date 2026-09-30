// ResidencyTest.cpp - the residency manager's own gate (--selftest, [restest]).
//
// THE FLOOR LAW (Residency.h; docs/HIERARCHY.md 4.6) held to what it claims, each check first seen
// to catch a defect planted for it:
//   1. SOUND. At every point of a 16 x 16 sub-grid of every cell of random true maps -- the six
//      faces of a cube and a window -- the GPU's bilinear read of the floor is at least the
//      largest of the four nearest TRUE bytes. Across a face's edge those four are found by
//      directions (ComposeCubeDir, CubeFaceOfDir), never through CubeFloorRing. Planted: the floor
//      without its cross-face neighbours, and a 3 x 3 that forgets its diagonals.
//   2. THE KERNELS' TAPS (review finding 9). For random uv, every texel PageLoad4 reads at the
//      rounded mip of PageHaveLoad's one byte lies in a cell whose true byte is no larger than
//      that byte. Planted: the undilated map, which is the finding as it shipped.
//   3. THE HARNESS'S NUMBERS. porch_floor.py's 200 maps, drawn from Python's own generator (seed
//      3) draw for draw and dilated by FloorMap: its samples, its largest steps, its mean cost;
//      then the same statistics on the engine's 128-cell maps.
//   4. THE GPU (shaders/ResidencyFloor.hlsl). The M6h scenario, measured: a reserved array with
//      tiles mapped and filled at every resident mip and their neighbours NULL, sampled across the
//      frontier through PageSample with the clamp from PageHave, by the engine's two samplers,
//      under the law in effect, the floor, the floor twice, M6h itself (the true map read
//      bilinear) and the two floors read by the law in effect's gather + max, point for point
//      against the law in effect; the MARGIN law (MarginMap) at M = 2, 4, 6, 8, 10 and 12 texels
//      of the level read, each read by the gather + max in effect and bilinear over its own 3 x 3;
//      the mechanism, measured alone, at whole and at fractional clamps; and the law in effect and
//      the margins again at footprint sizes BETWEEN powers of two, tallied apart (Fractional). The
//      margin law's cost is gate 3's too, on maps closed as the manager holds them (Closed).
//
// The suite fails when an instrument is broken: a planted defect not caught, a probe that cannot
// see its own fill, a construction that does not hold its arithmetic. What each law lets in on
// this GPU -- the law in effect included -- is a FINDING, printed every run and not failed: the
// floor is not staged (Residency.h says why), and the law in effect is what has always shipped.
#include "hal/Residency.h"

#include "core/Lattice.h"
#include "hal/Pipeline.h"
#include "hal/PixEvents.h"
#include "hal/Root.h"
#include "hal/Shader.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <iterator>
#include <set>
#include <string>
#include <vector>

namespace ga {

namespace {

using Maps = std::vector<std::vector<uint8_t>>;

// ============================================================================ Python's generator
// random.Random as far as porch_floor.py uses it (CPython 3.14's _randommodule.c and random.py):
// MT19937 seeded by init_by_array from the seed's one 32-bit word; randrange(n) is getrandbits of
// n's bit length, redrawn until it falls below n; random() is 53 bits from two draws. Enough to
// walk the harness's 200 maps draw for draw; the first draws are held to Python's own below.
class PyRandom {
public:
    explicit PyRandom(uint32_t seed) {
        m_mt[0] = 19650218u;   // init_genrand(19650218)
        for (uint32_t i = 1; i < kN; ++i) {
            m_mt[i] = 1812433253u * (m_mt[i - 1] ^ (m_mt[i - 1] >> 30)) + i;
        }
        uint32_t i = 1;   // init_by_array({seed}): with one key word, j is 0 at every use
        for (uint32_t k = kN; k; --k) {
            m_mt[i] = (m_mt[i] ^ ((m_mt[i - 1] ^ (m_mt[i - 1] >> 30)) * 1664525u)) + seed;
            if (++i >= kN) {
                m_mt[0] = m_mt[kN - 1];
                i = 1;
            }
        }
        for (uint32_t k = kN - 1; k; --k) {
            m_mt[i] = (m_mt[i] ^ ((m_mt[i - 1] ^ (m_mt[i - 1] >> 30)) * 1566083941u)) - i;
            if (++i >= kN) {
                m_mt[0] = m_mt[kN - 1];
                i = 1;
            }
        }
        m_mt[0] = 0x80000000u;
        m_index = kN;
    }
    uint32_t Next32() {
        if (m_index >= kN) Twist();
        uint32_t y = m_mt[m_index++];
        y ^= y >> 11;
        y ^= (y << 7) & 0x9d2c5680u;
        y ^= (y << 15) & 0xefc60000u;
        y ^= y >> 18;
        return y;
    }
    double Random() {
        const uint32_t a = Next32() >> 5, b = Next32() >> 6;
        return (a * 67108864.0 + b) * (1.0 / 9007199254740992.0);
    }
    int RandRange(int n) {
        int k = 0;
        while ((n >> k) != 0) ++k;   // n.bit_length()
        uint32_t r;
        do {
            r = Next32() >> (32 - k);
        } while (r >= static_cast<uint32_t>(n));
        return static_cast<int>(r);
    }
    int RandInt(int a, int b) { return a + RandRange(b - a + 1); }

private:
    static constexpr uint32_t kN = 624, kM = 397;
    void Twist() {
        for (uint32_t k = 0; k < kN; ++k) {
            const uint32_t y = (m_mt[k] & 0x80000000u) | (m_mt[(k + 1) % kN] & 0x7fffffffu);
            m_mt[k] = m_mt[(k + kM) % kN] ^ (y >> 1) ^ ((y & 1u) ? 0x9908b0dfu : 0u);
        }
        m_index = 0;
    }
    uint32_t m_mt[kN] = {};
    uint32_t m_index = kN;
};

// ============================================================================ the maps
constexpr uint32_t kR = 128;       // the engine's map: a byte per 128 texels of a 16384 page
constexpr uint32_t kFaces = 6;     // slices 0..5 of a set are a cube; slice 6 is a window
constexpr uint32_t kSetSlices = 7;

uint8_t At(const Maps& m, uint32_t flat) { return m[flat / (kR * kR)][flat % (kR * kR)]; }

// The cell under face f's plane point (u, v) -- 0..1 across the face, and past it beyond the
// edge -- as the direction through it falls: face and cell, flattened as CubeFloorRing does.
uint32_t CellOfDir(uint32_t f, double u, double v) {
    double d[3], uv[2];
    ComposeCubeDir(f, u, v, d);
    const uint32_t g = CubeFaceOfDir(d, uv);
    const auto index = [](double c) {
        return static_cast<uint32_t>(
            std::clamp(static_cast<int>(std::floor(c * kR)), 0, static_cast<int>(kR) - 1));
    };
    return (g * kR + index(uv[1])) * kR + index(uv[0]);
}

// The texel the hardware's seamless cube filter reads for texel (i, j) of face f when exactly one
// of i, j lies past the square: the neighbouring face's texel at the same place along the shared
// edge. Found by directions -- the point ON the edge at that place, nudged across it.
uint32_t SeamTexel(uint32_t f, int i, int j) {
    const auto coord = [](int k) {
        return k < 0 ? -1e-9 : k >= static_cast<int>(kR) ? 1.0 + 1e-9 : (k + 0.5) / kR;
    };
    return CellOfDir(f, coord(i), coord(j));
}

// A true map like a real one, porch_floor.py's recipe at the engine's size: the coarsest mip
// everywhere (the floor the manager maps at a tenant's birth), then square patches of finer
// mips, each cell keeping the finest patch over it. Overlapping patches make the concave
// corners a floor without diagonals would miss.
void RandomTrueMap(PyRandom& r, std::vector<uint8_t>& m) {
    m.assign(size_t(kR) * kR, uint8_t(7 * 16));
    const int n = static_cast<int>(kR);
    const int patches = r.RandInt(1, 12);
    for (int k = 0; k < patches; ++k) {
        const int cx = r.RandRange(n), cy = r.RandRange(n), h = r.RandInt(1, n / 3);
        const uint8_t v = static_cast<uint8_t>(r.RandInt(0, 6) * 16);
        for (int y = (std::max)(cy - h, 0); y < (std::min)(cy + h, n); ++y) {
            for (int x = (std::max)(cx - h, 0); x < (std::min)(cx + h, n); ++x) {
                uint8_t& b = m[size_t(y) * kR + x];
                b = (std::min)(b, v);
            }
        }
    }
}

// One test set: the six faces and a window. With `deadFace`, face 3 is all 255 -- a coarsest
// tile that failed at boot, "nothing here" -- so its edges spread 255 onto its neighbours.
Maps RandomSet(PyRandom& r, bool deadFace) {
    Maps t(kSetSlices);
    for (uint32_t s = 0; s < kSetSlices; ++s) RandomTrueMap(r, t[s]);
    if (deadFace) t[3].assign(size_t(kR) * kR, uint8_t(255));
    return t;
}

// THE PLANTED DIAGONAL: FloorPad's ring, then the largest of the centre and its four edge
// neighbours only -- a 3 x 3 that forgets its corners.
void PlusFloor(const Maps& t, const std::vector<uint32_t>& ring, Maps& out) {
    const ptrdiff_t p = kR + 2;
    std::vector<uint8_t> pad;
    out.resize(t.size());
    for (uint32_t s = 0; s < t.size(); ++s) {
        ResidencyManager::FloorPad(t, kR, s, s < kFaces ? &ring : nullptr, pad);
        out[s].resize(size_t(kR) * kR);
        for (uint32_t y = 0; y < kR; ++y) {
            for (uint32_t x = 0; x < kR; ++x) {
                const uint8_t* c = &pad[size_t(y + 1) * p + (x + 1)];
                out[s][size_t(y) * kR + x] = (std::max)({c[0], c[-1], c[1], c[-p], c[p]});
            }
        }
    }
}

// THE MARGIN LAW (tested here and by the probe; staged nowhere): a level may be read at a place
// only where the place stands at least M of THAT level's texels inside what is resident at that
// level, and the same holds at every coarser level. The unit is the mechanism's ([restest] gpu
// mechanism): where the clamp decides, an anisotropic footprint keeps its ratio N and is scaled
// to the clamped level, reaching about N / 2 of that level's texels along its long axis and one
// more for the bilinear tap -- a constant count of texels of the level read, where a fixed count
// of cells is 128 texels of mip 0 and 2 of mip 6.
//
// ok(c, L): every mip-L tile meeting cell c's square grown by M texels of mip L on every side
// (M 2^L / 128 cells) is resident. The byte is F(c) * 16, F the smallest L with ok(c, L') at every
// L' from L to the coarsest; 255 ("nothing here") where even the coarsest fails -- beside a dead
// face. A mip-L tile counts as resident when every cell under it holds mip L or finer: for a map
// made from a resident set closed upward that is the tile being mapped; for a random map it is
// the cautious reading. A window's grown square is clamped at the slice's edge, as the sampler
// clamps; a cube face's continues past the edge by CubeFloorRing's geometry (the cell under a
// point's direction), sampled a quarter of a tile of that level apart (never coarser than a
// quarter of a cell), its edges included, so every tile it meets on a neighbour is seen. Square
// maps only (rdim a side); with `cube`, rdim is kR, CellOfDir's size.
void MarginMap(const Maps& t, uint32_t rdim, uint32_t levels, bool cube, double margin, Maps& out) {
    const int n = static_cast<int>(rdim);
    struct Level {
        int span = 1, nt = 1;
        std::vector<uint8_t> mx;     // each mip-L tile's coarsest cell
        std::vector<uint32_t> sum;   // prefix sums of "not resident", (nt + 1)^2
    };
    std::vector<std::vector<Level>> lv(t.size(), std::vector<Level>(levels));
    for (size_t s = 0; s < t.size(); ++s) {
        for (uint32_t L = 0; L < levels; ++L) {
            Level& e = lv[s][L];
            e.span = 1 << L;
            e.nt = (n + e.span - 1) / e.span;
            e.mx.assign(size_t(e.nt) * e.nt, uint8_t(0));
            for (int y = 0; y < n; ++y) {
                for (int x = 0; x < n; ++x) {
                    uint8_t& m = e.mx[size_t(y / e.span) * e.nt + x / e.span];
                    m = (std::max)(m, t[s][size_t(y) * rdim + x]);
                }
            }
            const int w = e.nt + 1;
            e.sum.assign(size_t(w) * w, 0u);
            for (int y = 0; y < e.nt; ++y) {
                for (int x = 0; x < e.nt; ++x) {
                    const uint32_t bad = e.mx[size_t(y) * e.nt + x] > L * 16 ? 1u : 0u;
                    e.sum[size_t(y + 1) * w + x + 1] = bad + e.sum[size_t(y) * w + x + 1] +
                                                       e.sum[size_t(y + 1) * w + x] -
                                                       e.sum[size_t(y) * w + x];
                }
            }
        }
    }
    const auto ok = [&](size_t s, int cx, int cy, uint32_t L) {
        const Level& e = lv[s][L];
        const double g = margin * std::ldexp(1.0, static_cast<int>(L)) / 128.0;
        const double x0 = cx - g, x1 = cx + 1 + g, y0 = cy - g, y1 = cy + 1 + g;
        const auto lo = [&](double v) {
            return std::clamp(static_cast<int>(std::floor((std::max)(v, 0.0) / e.span)), 0,
                              e.nt - 1);
        };
        const auto hi = [&](double v) {
            return std::clamp(static_cast<int>(std::ceil((std::min)(v, double(n)) / e.span)) - 1, 0,
                              e.nt - 1);
        };
        const int tx0 = lo(x0), ty0 = lo(y0), tx1 = hi(x1), ty1 = hi(y1), w = e.nt + 1;
        if (e.sum[size_t(ty1 + 1) * w + tx1 + 1] - e.sum[size_t(ty0) * w + tx1 + 1] -
                e.sum[size_t(ty1 + 1) * w + tx0] + e.sum[size_t(ty0) * w + tx0] != 0) {
            return false;
        }
        if (!cube || s >= kFaces || (x0 >= 0 && y0 >= 0 && x1 <= n && y1 <= n)) return true;
        const double step = (std::max)(0.25, e.span / 4.0);
        const int kx = (std::max)(2, static_cast<int>(std::ceil((x1 - x0) / step)));
        const int ky = (std::max)(2, static_cast<int>(std::ceil((y1 - y0) / step)));
        for (int j = 0; j <= ky; ++j) {
            const double y = y0 + (y1 - y0) * j / ky;
            for (int i = 0; i <= kx; ++i) {
                const double x = x0 + (x1 - x0) * i / kx;
                if (x >= 0 && x <= n && y >= 0 && y <= n) continue;
                const uint32_t flat = CellOfDir(static_cast<uint32_t>(s), x / n, y / n);
                const uint32_t f = flat / (rdim * rdim), c = flat % (rdim * rdim);
                const Level& o = lv[f][L];
                const uint32_t span = static_cast<uint32_t>(o.span);
                if (o.mx[size_t((c / rdim) / span) * o.nt + (c % rdim) / span] > L * 16) return false;
            }
        }
        return true;
    };
    out.assign(t.size(), std::vector<uint8_t>(size_t(rdim) * rdim, uint8_t(255)));
    for (size_t s = 0; s < t.size(); ++s) {
        for (int cy = 0; cy < n; ++cy) {
            for (int cx = 0; cx < n; ++cx) {
                uint8_t f = 255;
                for (int L = static_cast<int>(levels) - 1; L >= 0 && ok(s, cx, cy, uint32_t(L)); --L) {
                    f = static_cast<uint8_t>(L * 16);
                }
                out[s][size_t(cy) * rdim + cx] = f;
            }
        }
    }
}

// A random map read as the manager would hold it: the tiles it names (each cell's finest mip) and
// every ancestor -- the manager never maps a tile under a NULL parent -- and the true map of that
// set. A random map's cells are not consistent with its own tiles (a mip-6 tile over a mip-3
// patch is resident, so every cell under it holds mip 6 or finer); the margin law reads tiles, so
// its cost is measured on the closed map, and today's gather and the floors beside it too.
Maps Closed(const Maps& t, uint32_t rdim, uint32_t levels) {
    const int n = static_cast<int>(rdim);
    Maps c = t;
    for (size_t s = 0; s < t.size(); ++s) {
        for (uint32_t L = 0; L < levels; ++L) {
            const int span = 1 << L, nt = (n + span - 1) / span;
            std::vector<uint8_t> mn(size_t(nt) * nt, uint8_t(255));
            for (int y = 0; y < n; ++y) {
                for (int x = 0; x < n; ++x) {
                    uint8_t& m = mn[size_t(y / span) * nt + x / span];
                    m = (std::min)(m, t[s][size_t(y) * rdim + x]);
                }
            }
            for (int y = 0; y < n; ++y) {
                for (int x = 0; x < n; ++x) {
                    if (mn[size_t(y / span) * nt + x / span] <= L * 16) {
                        uint8_t& b = c[s][size_t(y) * rdim + x];
                        b = (std::min)(b, static_cast<uint8_t>(L * 16));
                    }
                }
            }
        }
    }
    return c;
}

// The GPU's bilinear read of map `m` at point (px, py) -- in cells -- of slice s: the four texels
// around it, weighted as the filter weights them. A window clamps at its edge (sLinearClamp). On a
// cube face a texel past an edge is the neighbour's texel at the same place along it (SeamTexel),
// and the texel past a CORNER, which no face has, is taken as the least of the three that meet
// there: whatever the hardware synthesizes it from, a mix of the three reads no less.
double BilinearAt(const Maps& m, uint32_t s, double px, double py, bool cube) {
    const double tx = px - 0.5, ty = py - 0.5;
    const int x0 = static_cast<int>(std::floor(tx)), y0 = static_cast<int>(std::floor(ty));
    const double fx = tx - x0, fy = ty - y0;
    const int n = static_cast<int>(kR);
    double v[4] = {};
    int corner = -1;
    for (int k = 0; k < 4; ++k) {
        const int i = x0 + (k & 1), j = y0 + (k >> 1);
        const bool oi = i < 0 || i >= n, oj = j < 0 || j >= n;
        if (!cube) {
            v[k] = m[s][size_t(std::clamp(j, 0, n - 1)) * kR + std::clamp(i, 0, n - 1)];
        } else if (!oi && !oj) {
            v[k] = m[s][size_t(j) * kR + i];
        } else if (oi && oj) {
            corner = k;
        } else {
            v[k] = At(m, SeamTexel(s, i, j));
        }
    }
    if (corner >= 0) {
        double lo = 1e9;
        for (int k = 0; k < 4; ++k) {
            if (k != corner) lo = (std::min)(lo, v[k]);
        }
        v[corner] = lo;
    }
    return (1 - fx) * (1 - fy) * v[0] + fx * (1 - fy) * v[1] + (1 - fx) * fy * v[2] +
           fx * fy * v[3];
}

// The largest of the four nearest TRUE bytes of point (px, py): the cells under the point nudged
// half a cell each way. On a window they clamp at its edge. Across a face's edge they are found
// by directions and never through CubeFloorRing, in two ways, because the cube's geometry gives
// two answers there. SEAM keeps the place along the edge: the texel the hardware's own filter
// reads (SeamTexel); past a CORNER it has no texel (the hardware makes one from the three the
// other nudges already name). PLANE is the direction of the point half a cell further along THIS
// face's plane: the gnomonic map shrinks the along-edge coordinate by 1 / (1 + 2d / rdim) for a
// point d cells past the edge -- a shift toward the edge's middle of up to half a cell, larger
// the farther from that middle -- so on the neighbour it can fall into the next cell over, which
// is none of the four texels the filter reads there.
uint8_t TrueMax(const Maps& t, uint32_t s, double px, double py, bool cube, bool plane) {
    const int n = static_cast<int>(kR);
    uint8_t best = 0;
    for (int k = 0; k < 4; ++k) {
        const double qx = px + ((k & 1) ? 0.5 : -0.5), qy = py + ((k >> 1) ? 0.5 : -0.5);
        const int i = static_cast<int>(std::floor(qx)), j = static_cast<int>(std::floor(qy));
        const bool oi = i < 0 || i >= n, oj = j < 0 || j >= n;
        uint8_t b;
        if (!cube) {
            b = t[s][size_t(std::clamp(j, 0, n - 1)) * kR + std::clamp(i, 0, n - 1)];
        } else if (!oi && !oj) {
            b = t[s][size_t(j) * kR + i];
        } else if (plane) {
            b = At(t, CellOfDir(s, qx / kR, qy / kR));
        } else if (oi && oj) {
            continue;
        } else {
            b = At(t, SeamTexel(s, i, j));
        }
        best = (std::max)(best, b);
    }
    return best;
}

// ============================================================================ 1. sound
struct SoundResult {
    uint64_t samples = 0;
    uint64_t seamBad = 0;     // below the largest of the four true bytes the seam names
    uint64_t atEdge = 0;      // ... of which within half a cell of a cube face's edge
    uint64_t planeBad = 0;    // below the largest of the four the plane nudge names
    uint64_t planeOnly = 0;   // ... where the seam's four are covered
    double worst = 0.0;       // the largest seam shortfall, in mips
    std::string first;        // the first seam failure
};

void CheckSound(const Maps& t, const Maps& d, SoundResult& r) {
    const int n = static_cast<int>(kR);
    for (uint32_t s = 0; s < t.size(); ++s) {
        const bool cube = s < kFaces;
        const std::vector<uint8_t>& T = t[s];
        const std::vector<uint8_t>& D = d[s];
        for (uint32_t cy = 0; cy < kR; ++cy) {
            for (uint32_t cx = 0; cx < kR; ++cx) {
                for (uint32_t sy = 0; sy < 16; ++sy) {
                    const double py = cy + (sy + 0.5) / 16.0;
                    for (uint32_t sx = 0; sx < 16; ++sx) {
                        const double px = cx + (sx + 0.5) / 16.0;
                        const double tx = px - 0.5, ty = py - 0.5;
                        const int x0 = static_cast<int>(std::floor(tx));
                        const int y0 = static_cast<int>(std::floor(ty));
                        double b;
                        uint8_t seam, plane;
                        if (x0 >= 0 && y0 >= 0 && x0 + 1 < n && y0 + 1 < n) {
                            // The four the filter mixes ARE the four nearest: the one 2 x 2.
                            const double fx = tx - x0, fy = ty - y0;
                            const size_t i = size_t(y0) * kR + x0;
                            b = (1 - fx) * (1 - fy) * D[i] + fx * (1 - fy) * D[i + 1] +
                                (1 - fx) * fy * D[i + kR] + fx * fy * D[i + kR + 1];
                            seam = plane = (std::max)({T[i], T[i + 1], T[i + kR], T[i + kR + 1]});
                        } else {
                            b = BilinearAt(d, s, px, py, cube);
                            seam = TrueMax(t, s, px, py, cube, false);
                            plane = cube ? TrueMax(t, s, px, py, true, true) : seam;
                        }
                        ++r.samples;
                        const bool bs = b < seam - 1e-9, bp = b < plane - 1e-9;
                        if (bs) {
                            if (r.seamBad++ == 0) {
                                char w[200];
                                snprintf(w, sizeof w,
                                         "slice %u (%s) cell (%u,%u) at (%.4f, %.4f): read %.3f "
                                         "mips, a true byte there %.3f",
                                         s, cube ? "a cube face" : "the window", cx, cy, px, py,
                                         b / 16.0, seam / 16.0);
                                r.first = w;
                            }
                            if (cube && (x0 < 0 || y0 < 0 || x0 + 1 >= n || y0 + 1 >= n)) {
                                ++r.atEdge;
                            }
                            r.worst = (std::max)(r.worst, (seam - b) / 16.0);
                        }
                        if (bp) {
                            ++r.planeBad;
                            if (!bs) ++r.planeOnly;
                        }
                    }
                }
            }
        }
    }
}

// ============================================================================ 2. the kernels' taps
struct TapResult {
    uint64_t reads = 0, taps = 0, bad = 0;
    std::string first;
};

// PageHaveLoad's one byte, then PageLoad4's four texels at that byte's rounded mip, in the
// shader's float arithmetic (PageSample.hlsli: kPageDim 16384, kPageResDim 128). A kernel reads a
// cube face through that face's own uv (HeightPages.hlsli's HpCubeFace) and PageLoad4 clamps its
// taps inside the slice, so every slice is read here as the 2D page it is to a kernel.
void CheckTaps(const Maps& t, const Maps& d, PyRandom& rnd, uint32_t perSlice, TapResult& r) {
    for (uint32_t s = 0; s < t.size(); ++s) {
        for (uint32_t k = 0; k < perSlice; ++k) {
            const float u = static_cast<float>(rnd.Random()), v = static_cast<float>(rnd.Random());
            const int cx = static_cast<int>(std::clamp(u * 128.0f, 0.0f, 127.0f));
            const int cy = static_cast<int>(std::clamp(v * 128.0f, 0.0f, 127.0f));
            const uint8_t byte = d[s][size_t(cy) * kR + cx];
            const float mip = std::nearbyint(static_cast<float>(byte) / 255.0f * 15.9375f);
            if (mip > 7.5f) continue;   // "nothing here": a kernel reads no texel at all
            const uint32_t m = static_cast<uint32_t>(mip);
            const float dim = 16384.0f / std::exp2(mip);
            const int t0x = static_cast<int>(std::floor(u * dim - 0.5f));
            const int t0y = static_cast<int>(std::floor(v * dim - 0.5f));
            const int top = static_cast<int>(dim) - 1;
            ++r.reads;
            for (int q = 0; q < 4; ++q) {
                const uint32_t tx = static_cast<uint32_t>(std::clamp(t0x + (q & 1), 0, top));
                const uint32_t ty = static_cast<uint32_t>(std::clamp(t0y + (q >> 1), 0, top));
                const uint32_t ex = (tx << m) / 128u, ey = (ty << m) / 128u;   // its cell
                ++r.taps;
                if (t[s][size_t(ey) * kR + ex] > byte) {
                    if (r.bad++ == 0) {
                        char w[200];
                        snprintf(w, sizeof w,
                                 "slice %u uv (%.6f, %.6f): the byte read says mip %u, a tap at "
                                 "texel (%u,%u) lies in cell (%u,%u) whose true mip is %u",
                                 s, u, v, m, tx, ty, ex, ey, t[s][size_t(ey) * kR + ex] / 16u);
                        r.first = w;
                    }
                }
            }
        }
    }
}

// ============================================================================ 3. the harness
struct Steps {
    uint64_t samples = 0, unsound = 0;
    double stepOld = 0.0, stepNew = 0.0, extra = 0.0;
};

// porch_floor.py's gather_max and bilinear, statement for statement, on bytes; n cells a side.
double GatherMax(const std::vector<uint8_t>& m, int n, double x, double y) {
    const int i0 = static_cast<int>((std::min)((std::max)(x - 0.5, 0.0), n - 1.0));
    const int j0 = static_cast<int>((std::min)((std::max)(y - 0.5, 0.0), n - 1.0));
    const int i1 = (std::min)(i0 + 1, n - 1), j1 = (std::min)(j0 + 1, n - 1);
    return (std::max)({m[size_t(j0) * n + i0], m[size_t(j0) * n + i1], m[size_t(j1) * n + i0],
                       m[size_t(j1) * n + i1]});
}
double Bilinear(const std::vector<uint8_t>& d, int n, double x, double y) {
    const double fx = (std::min)((std::max)(x - 0.5, 0.0), n - 1.0);
    const double fy = (std::min)((std::max)(y - 0.5, 0.0), n - 1.0);
    const int i0 = static_cast<int>(fx), j0 = static_cast<int>(fy);
    const int i1 = (std::min)(i0 + 1, n - 1), j1 = (std::min)(j0 + 1, n - 1);
    const double tx = fx - i0, ty = fy - j0;
    return (1 - tx) * (1 - ty) * d[size_t(j0) * n + i0] + tx * (1 - ty) * d[size_t(j0) * n + i1] +
           (1 - tx) * ty * d[size_t(j1) * n + i0] + tx * ty * d[size_t(j1) * n + i1];
}

// porch_floor.py's floor_law(), on its own 200 maps, with the dilation FloorMap's -- and the same
// statistics for the other reads: the floor read by the gather + max, and the margin law's two.
enum class Read { FloorBilinear, FloorGather, MarginGather, MarginBilinear };
Steps PorchFloor(Read how, uint32_t rings, double margin = 0.0, bool closed = false) {
    constexpr int N = 24;
    PyRandom rnd(3);
    Steps st;
    Maps m(1), d, d2;
    for (int trial = 0; trial < 200; ++trial) {
        m[0].assign(size_t(N) * N, uint8_t(7 * 16));
        const int patches = rnd.RandInt(1, 6);
        for (int p = 0; p < patches; ++p) {
            const int cx = rnd.RandRange(N), cy = rnd.RandRange(N), r = rnd.RandInt(1, 8);
            const uint8_t v = static_cast<uint8_t>(rnd.RandInt(0, 6) * 16);
            for (int j = (std::max)(cy - r, 0); j < (std::min)(cy + r, N); ++j) {
                for (int i = (std::max)(cx - r, 0); i < (std::min)(cx + r, N); ++i) {
                    m[0][size_t(j) * N + i] = (std::min)(m[0][size_t(j) * N + i], v);
                }
            }
        }
        if (closed) m = Closed(m, N, 8);
        if (how == Read::MarginGather || how == Read::MarginBilinear) {
            MarginMap(m, N, 8, false, margin, d);
            if (how == Read::MarginBilinear) {
                ResidencyManager::FloorMap(d, N, nullptr, d2);
                d.swap(d2);
            }
        } else {
            ResidencyManager::FloorMap(m, N, nullptr, d);
            for (uint32_t k = 1; k < rings; ++k) {
                ResidencyManager::FloorMap(d, N, nullptr, d2);
                d.swap(d2);
            }
        }
        const bool bilinear = how == Read::FloorBilinear || how == Read::MarginBilinear;
        const double y = rnd.Random() * N;
        double prevOld = 0.0, prevNew = 0.0;
        for (int k = 0; k < 16 * N; ++k) {   // x += 1/16, exact, from 0 while x < N
            const double x = k / 16.0;
            const double old = GatherMax(m[0], N, x, y) / 16.0;
            const double neu = (bilinear ? Bilinear(d[0], N, x, y) : GatherMax(d[0], N, x, y)) / 16.0;
            if (neu < old - 1e-9) ++st.unsound;
            st.extra += neu - old;
            ++st.samples;
            if (k > 0) {
                st.stepOld = (std::max)(st.stepOld, std::fabs(old - prevOld));
                st.stepNew = (std::max)(st.stepNew, std::fabs(neu - prevNew));
            }
            prevOld = old;
            prevNew = neu;
        }
    }
    return st;
}

// The same statistics on the engine's maps: rows a sixteenth of a cell a step at random heights
// over every slice of a set, where the gather's 2 x 2 stays inside its slice (a face's edge
// belongs to gate 1). `d` is read bilinear (the floor's read) or by the gather + max (the read in
// effect, over another map).
void EngineSteps(const Maps& t, const Maps& d, bool bilinear, PyRandom& rnd, Steps& st) {
    const int n = static_cast<int>(kR);
    for (uint32_t s = 0; s < t.size(); ++s) {
        for (int row = 0; row < 16; ++row) {
            const double y = 0.5 + rnd.Random() * (n - 1.0);
            double prevOld = 0.0, prevNew = 0.0;
            bool prev = false;
            for (int k = 8; k <= 16 * n - 8; ++k) {
                const double x = k / 16.0;
                const double old = GatherMax(t[s], n, x, y) / 16.0;
                const double neu =
                    (bilinear ? Bilinear(d[s], n, x, y) : GatherMax(d[s], n, x, y)) / 16.0;
                if (neu < old - 1e-9) ++st.unsound;
                st.extra += neu - old;
                ++st.samples;
                if (prev) {
                    st.stepOld = (std::max)(st.stepOld, std::fabs(old - prevOld));
                    st.stepNew = (std::max)(st.stepNew, std::fabs(neu - prevNew));
                }
                prevOld = old;
                prevNew = neu;
                prev = true;
            }
        }
    }
}

// ============================================================================ 4. the GPU
// A page tenant's array at the colour tenant's shape -- 16384 texels a slice, RGBA8 in 128 x 128
// tiles, the chain stopped at the one-tile level -- so its cells, its mips and a footprint's size
// against both are the engine's: the danger this measures lives at mips 5 and 6, where one texel
// is a quarter or half a cell and an 8x footprint spans several. Small in what it maps.
constexpr uint32_t kPDim = 16384, kPMips = 8, kPSlices = 7, kPWindow = 6;
constexpr uint32_t kPTileBytes = 65536;
// The margin law's M, in texels of the level read. At a whole level the mechanism's reach under 8x
// is about N / 2 = 4 texels of that level and one more for the bilinear tap, so 2 and 4 are plants
// below it. Between levels the finer of the pair is read with the footprint 2^f longer in its
// texels, (N / 2) 2^f + 1, up to N + 1 = 9: 6 is the plant for that, 8 its test, 10 = N + 2, and
// 12 lies above it.
constexpr int kNumMargins = 6;
constexpr double kMargins[kNumMargins] = {2.0, 4.0, 6.0, 8.0, 10.0, 12.0};
constexpr int kLaws = 7 + 2 * kNumMargins, kShapes = 7, kScales = 12;
constexpr int kMaps = 3 + 2 * kNumMargins;   // true, floor, floor twice, each margin's F, F's floor
constexpr uint32_t kQuadsMax = 384;
constexpr uint32_t kRowsPerScan = kLaws * 2 * kShapes * kScales;   // 3192

struct FloorCb {   // shaders/ResidencyFloor.hlsl's FloorCb, as root constants
    uint32_t arr, map, slice, law, aniso, cube;
    float start[2], step[2], jx[2], jy[2];
    float fixed;
};
static_assert(sizeof(FloorCb) == 60, "FloorCb is fifteen root constants");

struct ProbeLaw {
    const char* name;
    uint32_t map;   // 0 the true map, 1 the floor, 2 the floor of the floor, 3 + k margin k's F,
                    // 3 + kNumMargins + k F's own 3 x 3 floor (k over kMargins)
    uint32_t law;   // FloorCb::law: 0 PageHave (gather + max), 1 the floor's bilinear read, 2 fixed
};
// The law in effect first: every other law is held against it point for point. The last two
// are the floor maps read by the law in effect -- no law of the brief, measured as data.
const ProbeLaw kProbeLaws[kLaws] = {
    {"the law in effect (PageHave: gather + max, true map)", 0, 0},
    {"THE FLOOR (bilinear, the 3 x 3 floor)", 1, 1},
    {"the floor twice (bilinear, a 5 x 5 floor)", 2, 1},
    {"M6h, planted (bilinear, the true map)", 0, 1},
    {"no clamp at all (the fill's own check)", 0, 2},
    {"gather + max over the 3 x 3 floor", 1, 0},
    {"gather + max over the 5 x 5 floor", 2, 0},
    // THE MARGIN LAW (MarginMap) at each M of kMargins: F read by the gather + max in effect, then
    // F's own 3 x 3 read bilinear. M = 2, 4 and 6 are plants; 8 is the fractional level's test.
    {"margin M=2, planted (gather + max over F)", 3, 0},
    {"margin M=4, planted (gather + max over F)", 4, 0},
    {"margin M=6, planted (gather + max over F)", 5, 0},
    {"margin M=8 (gather + max over F)", 6, 0},
    {"margin M=10 (gather + max over F)", 7, 0},
    {"margin M=12 (gather + max over F)", 8, 0},
    {"margin M=2, planted (bilinear, F's 3 x 3)", 9, 1},
    {"margin M=4, planted (bilinear, F's 3 x 3)", 10, 1},
    {"margin M=6, planted (bilinear, F's 3 x 3)", 11, 1},
    {"margin M=8 (bilinear, F's 3 x 3)", 12, 1},
    {"margin M=10 (bilinear, F's 3 x 3)", 13, 1},
    {"margin M=12 (bilinear, F's 3 x 3)", 14, 1},
};
constexpr int kInEffect = 0, kFloor = 1, kFloor2 = 2, kPlanted = 3, kNoClamp = 4;
constexpr int kGatherFloor = 5, kGatherFloor2 = 6;
constexpr int kMarginGather = 7, kMarginBilinear = 7 + kNumMargins;   // + k over kMargins
const int kShapeRatio[kShapes] = {1, 4, 4, 4, 8, 8, 8};
const int kShapeAngle[kShapes] = {0, 0, 90, 45, 0, 90, 45};   // the major axis against the scan

// A row's footprint, in uv a pixel: the long axis 2^(scale - 1) texels of mip 0 at the shape's
// angle to the scan, the short one that over the shape's ratio.
void RowJ(int shape, int scale, float jx[2], float jy[2]) {
    const double kPi = 3.14159265358979323846;
    const double major = std::ldexp(1.0, scale - 1) / kPDim;
    const double minor = major / kShapeRatio[shape];
    const double a = kShapeAngle[shape] * kPi / 180.0;
    jx[0] = static_cast<float>(std::cos(a) * major);
    jx[1] = static_cast<float>(std::sin(a) * major);
    jy[0] = static_cast<float>(-std::sin(a) * minor);
    jy[1] = static_cast<float>(std::cos(a) * minor);
}

// RowJ at any size: the long axis 2^log2Texels texels of mip 0 (RowJ's scale is log2Texels + 1).
void RowJf(int shape, double log2Texels, float jx[2], float jy[2]) {
    const double kPi = 3.14159265358979323846;
    const double major = std::exp2(log2Texels) / kPDim;
    const double minor = major / kShapeRatio[shape];
    const double a = kShapeAngle[shape] * kPi / 180.0;
    jx[0] = static_cast<float>(std::cos(a) * major);
    jx[1] = static_cast<float>(std::sin(a) * major);
    jy[0] = static_cast<float>(-std::sin(a) * minor);
    jy[1] = static_cast<float>(std::cos(a) * minor);
}

struct ProbeScan {
    const char* name;
    bool cube;
    uint32_t slice;
    double x0, y;     // cells: where the scan starts, and the row it walks along
    uint32_t quads;   // sixteen a cell
};
const ProbeScan kProbeScans[] = {
    {"the window, through its block", false, kPWindow, 48.0, 63.7, 384},
    {"the window, 0.7 cell from the block's lower edge", false, kPWindow, 48.0, 71.3, 384},
    {"cube face 0, across its +u edge onto face 5", true, 0, 116.0, 63.7, 320},
    {"cube face 0, across that edge 1.6 cells from its corner", true, 0, 116.0, 126.4, 320},
};

struct ProbeTile {
    uint32_t slice, mip, x, y;
};

// THE RESIDENT SET, closed upward (the manager never maps a tile under a NULL parent). Three
// blocks of mip-0 tiles and their ancestors: the window's, cells 60-63 x 56-71, whose right side
// is a 0 -> 7 cliff on a mip-6 tile line (nothing finer than mip 7 at x >= 64) and whose left is
// the stair its ancestors make (3, 4, 5, 6 going left); cube face 0's at its +u edge, the same
// block against face 5, where only mip 7 is resident; and face 0's corner cells 124-127 squared.
// Every slice keeps its coarsest tile, the floor a tenant is born with.
std::vector<ProbeTile> ProbeResidentSet() {
    std::set<uint64_t> seen;
    std::vector<ProbeTile> tiles;
    const auto add = [&](uint32_t s, uint32_t m, uint32_t x, uint32_t y) {
        for (;; ++m, x >>= 1, y >>= 1) {
            const uint64_t key = (uint64_t(s) << 48) | (uint64_t(m) << 40) | (uint64_t(y) << 20) | x;
            if (!seen.insert(key).second) return;
            tiles.push_back({s, m, x, y});
            if (m + 1 == kPMips) return;
        }
    };
    for (uint32_t y = 56; y < 72; ++y) {
        for (uint32_t x = 60; x < 64; ++x) add(kPWindow, 0, x, y);
        for (uint32_t x = 124; x < 128; ++x) add(0, 0, x, y);
    }
    for (uint32_t y = 124; y < 128; ++y) {
        for (uint32_t x = 124; x < 128; ++x) add(0, 0, x, y);
    }
    for (uint32_t s = 0; s < kPSlices; ++s) add(s, kPMips - 1, 0, 0);
    return tiles;
}

// One sample of the target: the value through PageSample, the clamp, the hardware's LOD, and
// whether the hardware says every texel the footprint touched was mapped.
struct Px {
    float r, have, lod, mapped;
};

class FloorProbe {
public:
    // False when an instrument is broken: the array refused, the fill not what the probe
    // believes, the planted M6h not caught. Every law's standing is printed, not returned.
    bool Run(Gpu& gpu, ShaderCompiler& sc, const std::wstring& shaderDir);

private:
    bool CreateArray(Gpu& gpu);
    bool CreateMaps(Gpu& gpu);
    bool CreatePipeline(Gpu& gpu, ShaderCompiler& sc, const std::wstring& shaderDir);
    void DrawScan(Gpu& gpu, const ProbeScan& scan);
    void Mechanism(Gpu& gpu);
    void MechanismFrac(Gpu& gpu);
    void DrawRows(Gpu& gpu, const std::vector<FloorCb>& rows, uint32_t quads);
    std::vector<uint64_t> Fractional(Gpu& gpu);
    static size_t Row(int law, int aniso, int shape, int scale) {
        return ((size_t(law) * 2 + aniso) * kShapes + shape) * kScales + scale;
    }

    // The residency along a scan's row about cell x: seven cells' true finest mips, then the same
    // cells of the map a law read ('-' nothing here; '|' past the slice).
    std::string Around(const ProbeScan& s, double x, uint32_t map) const {
        const int cy = static_cast<int>(std::floor(s.y)), c0 = static_cast<int>(std::floor(x)) - 3;
        std::string out = "cells " + std::to_string(c0) + ".." + std::to_string(c0 + 6) +
                          " of row " + std::to_string(cy) + ": true";
        for (int k = 0; k < 2; ++k) {
            if (k) out += ", read";
            for (int cx = c0; cx <= c0 + 6; ++cx) {
                if (cx < 0 || cx >= static_cast<int>(kR)) {
                    out += " |";
                    continue;
                }
                const uint8_t b = m_cpu[k ? map : 0][s.slice][size_t(cy) * kR + cx];
                out += b == 255 ? std::string(" -") : " " + std::to_string(b / 16);
            }
        }
        return out;
    }

    std::vector<ProbeTile> m_tiles;
    Maps m_true;
    Maps m_cpu[kMaps];   // every map a law reads, as uploaded (m_cpu[0] is the true map)
    Com<ID3D12Resource> m_arr;
    Com<ID3D12Heap> m_heap;
    uint32_t m_arrSrv = 0, m_arrSrvCube = 0;
    Com<ID3D12Resource> m_maps[kMaps];
    uint32_t m_mapSrv[kMaps] = {}, m_mapSrvCube[kMaps] = {};
    Com<ID3D12RootSignature> m_rs;
    Com<ID3D12PipelineState> m_pso;
    GpuTexture m_rt;
    D3D12_CPU_DESCRIPTOR_HANDLE m_rtv{};
};

bool FloorProbe::CreateArray(Gpu& gpu) {
    D3D12_RESOURCE_DESC rd{};
    rd.Dimension = D3D12_RESOURCE_DIMENSION_TEXTURE2D;
    rd.Width = kPDim;
    rd.Height = kPDim;
    rd.DepthOrArraySize = kPSlices;
    rd.MipLevels = kPMips;
    rd.Format = DXGI_FORMAT_R8G8B8A8_UNORM;
    rd.SampleDesc.Count = 1;
    rd.Layout = D3D12_TEXTURE_LAYOUT_64KB_UNDEFINED_SWIZZLE;
    HRESULT hr = gpu.Device()->CreateReservedResource(&rd, D3D12_RESOURCE_STATE_COPY_DEST, nullptr,
                                                      IID_PPV_ARGS(&m_arr));
    if (FAILED(hr)) {
        Log("[restest] gpu: FAIL -- the probe array was refused: %s", HrString(hr).c_str());
        return false;
    }
    m_arr->SetName(L"restest.floor array");
    UINT numTiles = 0, numSub = kPSlices * kPMips;
    D3D12_PACKED_MIP_INFO packed{};
    D3D12_TILE_SHAPE shape{};
    std::vector<D3D12_SUBRESOURCE_TILING> sub(numSub);
    gpu.Device()->GetResourceTiling(m_arr.Get(), &numTiles, &packed, &shape, &numSub, 0, sub.data());
    // Every address below assumes the colour tenant's tiling; on any other the probe would
    // measure a shape it does not describe, so it refuses.
    if (packed.NumPackedMips != 0 || shape.WidthInTexels != 128 || shape.HeightInTexels != 128 ||
        sub[0].WidthInTiles != kR || sub[kPMips - 1].WidthInTiles != 1) {
        Log("[restest] gpu: FAIL -- not the tiling the probe is written for (%u packed mips, tile "
            "%ux%u, mip 0 %u tiles wide); refusing to run it",
            packed.NumPackedMips, shape.WidthInTexels, shape.HeightInTexels, sub[0].WidthInTiles);
        return false;
    }

    m_tiles = ProbeResidentSet();
    const uint32_t n = static_cast<uint32_t>(m_tiles.size());
    D3D12_HEAP_DESC hd{};
    hd.SizeInBytes = uint64_t(n) * kPTileBytes;
    hd.Properties.Type = D3D12_HEAP_TYPE_DEFAULT;
    hd.Flags = D3D12_HEAP_FLAG_ALLOW_ONLY_NON_RT_DS_TEXTURES;
    hr = gpu.Device()->CreateHeap(&hd, IID_PPV_ARGS(&m_heap));
    if (FAILED(hr)) {
        Log("[restest] gpu: FAIL -- the probe heap was refused: %s", HrString(hr).c_str());
        return false;
    }
    m_heap->SetName(L"restest.floor heap");

    // Each resident tile onto its own heap tile, in one UpdateTileMappings; everything else stays
    // as a reserved resource is born, NULL.
    std::vector<D3D12_TILED_RESOURCE_COORDINATE> at(n);
    std::vector<D3D12_TILE_REGION_SIZE> sizes(n, D3D12_TILE_REGION_SIZE{1, FALSE, 0, 0, 0});
    std::vector<D3D12_TILE_RANGE_FLAGS> flags(n, D3D12_TILE_RANGE_FLAG_NONE);
    std::vector<UINT> starts(n), counts(n, 1);
    for (uint32_t k = 0; k < n; ++k) {
        const ProbeTile& t = m_tiles[k];
        at[k] = D3D12_TILED_RESOURCE_COORDINATE{t.x, t.y, 0, t.slice * kPMips + t.mip};
        starts[k] = k;
    }
    gpu.Queue()->UpdateTileMappings(m_arr.Get(), n, at.data(), sizes.data(), m_heap.Get(), n,
                                    flags.data(), starts.data(), counts.data(),
                                    D3D12_TILE_MAPPING_FLAG_NONE);
    // The fill: 255 in every channel of every texel of every resident tile, at every mip.
    GpuBuffer bright = gpu.CreateUploadBuffer(kPTileBytes, L"restest.floor bright tile");
    memset(bright.cpu, 0xFF, kPTileBytes);
    auto* cl = gpu.BeginUpload();
    {
        PixScope scope(cl, "restest.floor fill (CopyTiles, one bright tile everywhere resident)");
        const D3D12_TILE_REGION_SIZE one{1, FALSE, 0, 0, 0};
        for (uint32_t k = 0; k < n; ++k) {
            cl->CopyTiles(m_arr.Get(), &at[k], &one, bright.res.Get(), 0,
                          D3D12_TILE_COPY_FLAG_LINEAR_BUFFER_TO_SWIZZLED_TILED_RESOURCE);
        }
        D3D12_RESOURCE_BARRIER b{};
        b.Type = D3D12_RESOURCE_BARRIER_TYPE_TRANSITION;
        b.Transition.pResource = m_arr.Get();
        b.Transition.Subresource = D3D12_RESOURCE_BARRIER_ALL_SUBRESOURCES;
        b.Transition.StateBefore = D3D12_RESOURCE_STATE_COPY_DEST;
        b.Transition.StateAfter = D3D12_RESOURCE_STATE_PIXEL_SHADER_RESOURCE;
        cl->ResourceBarrier(1, &b);
    }
    gpu.EndUpload();

    m_arrSrv = gpu.CreateSrvArray(m_arr.Get(), DXGI_FORMAT_R8G8B8A8_UNORM, kPMips, kPSlices);
    m_arrSrvCube = gpu.SrvHeap().Alloc();
    D3D12_SHADER_RESOURCE_VIEW_DESC cv{};
    cv.Shader4ComponentMapping = D3D12_DEFAULT_SHADER_4_COMPONENT_MAPPING;
    cv.Format = DXGI_FORMAT_R8G8B8A8_UNORM;
    cv.ViewDimension = D3D12_SRV_DIMENSION_TEXTURECUBEARRAY;
    cv.TextureCubeArray.MipLevels = kPMips;
    cv.TextureCubeArray.First2DArrayFace = 0;
    cv.TextureCubeArray.NumCubes = 1;
    gpu.Device()->CreateShaderResourceView(m_arr.Get(), &cv, gpu.SrvHeap().Cpu(m_arrSrvCube));

    // The true map, by UpdateResidencyByte's rule for 128-texel tiles: a mapped mip-m tile holds
    // its 2^m x 2^m cells to at most m * 16.
    m_true.assign(kPSlices, std::vector<uint8_t>(size_t(kR) * kR, uint8_t(255)));
    for (const ProbeTile& t : m_tiles) {
        const uint32_t span = 1u << t.mip;
        for (uint32_t y = t.y * span; y < (t.y + 1) * span; ++y) {
            for (uint32_t x = t.x * span; x < (t.x + 1) * span; ++x) {
                uint8_t& b = m_true[t.slice][size_t(y) * kR + x];
                b = (std::min)(b, static_cast<uint8_t>(t.mip * 16));
            }
        }
    }
    return true;
}

bool FloorProbe::CreateMaps(Gpu& gpu) {
    // The maps a law reads: the true one, its floor and the floor of the floor -- each FloorMap
    // with the cube's own border, the engine's call -- and the margin law's F at each margin, alone
    // and under its own 3 x 3 floor.
    const std::vector<uint32_t> ring = ResidencyManager::CubeFloorRing(kR);
    m_cpu[0] = m_true;
    ResidencyManager::FloorMap(m_true, kR, &ring, m_cpu[1]);
    ResidencyManager::FloorMap(m_cpu[1], kR, &ring, m_cpu[2]);
    for (int k = 0; k < kNumMargins; ++k) {
        MarginMap(m_true, kR, kPMips, true, kMargins[k], m_cpu[3 + k]);
        ResidencyManager::FloorMap(m_cpu[3 + k], kR, &ring, m_cpu[3 + kNumMargins + k]);
        unsigned long long coarser = 0, levels = 0;
        for (uint32_t s = 0; s < kPSlices; ++s) {
            for (size_t i = 0; i < size_t(kR) * kR; ++i) {
                if (m_cpu[3 + k][s][i] > m_true[s][i]) {
                    ++coarser;
                    levels += (m_cpu[3 + k][s][i] - m_true[s][i]) / 16;
                }
            }
        }
        Log("[restest] gpu margin M=%.0f: F is coarser than the true map at %llu of %u cells, by "
            "%llu levels in all",
            kMargins[k], coarser, kPSlices * kR * kR, levels);
    }
    const uint32_t pitch = 256;
    GpuBuffer stage =
        gpu.CreateUploadBuffer(uint64_t(kMaps) * kPSlices * kR * pitch, L"restest.floor maps");
    auto* cl = gpu.BeginUpload();
    for (int k = 0; k < kMaps; ++k) {
        D3D12_RESOURCE_DESC md{};
        md.Dimension = D3D12_RESOURCE_DIMENSION_TEXTURE2D;
        md.Width = kR;
        md.Height = kR;
        md.DepthOrArraySize = kPSlices;
        md.MipLevels = 1;
        md.Format = DXGI_FORMAT_R8_UNORM;
        md.SampleDesc.Count = 1;
        D3D12_HEAP_PROPERTIES hp{};
        hp.Type = D3D12_HEAP_TYPE_DEFAULT;
        const HRESULT hr = gpu.Device()->CreateCommittedResource(
            &hp, D3D12_HEAP_FLAG_NONE, &md, D3D12_RESOURCE_STATE_COPY_DEST, nullptr,
            IID_PPV_ARGS(&m_maps[k]));
        if (FAILED(hr)) {
            Log("[restest] gpu: FAIL -- a residency map was refused: %s", HrString(hr).c_str());
            gpu.EndUpload();
            return false;
        }
        m_maps[k]->SetName((L"restest.floor map " + std::to_wstring(k)).c_str());
        for (uint32_t s = 0; s < kPSlices; ++s) {
            const uint64_t off = ((uint64_t(k) * kPSlices + s) * kR) * pitch;
            for (uint32_t y = 0; y < kR; ++y) {
                memcpy(stage.cpu + off + uint64_t(y) * pitch, &m_cpu[k][s][size_t(y) * kR], kR);
            }
            D3D12_TEXTURE_COPY_LOCATION dl{}, sl{};
            dl.pResource = m_maps[k].Get();
            dl.Type = D3D12_TEXTURE_COPY_TYPE_SUBRESOURCE_INDEX;
            dl.SubresourceIndex = s;
            sl.pResource = stage.res.Get();
            sl.Type = D3D12_TEXTURE_COPY_TYPE_PLACED_FOOTPRINT;
            sl.PlacedFootprint.Offset = off;
            sl.PlacedFootprint.Footprint = {DXGI_FORMAT_R8_UNORM, kR, kR, 1, pitch};
            cl->CopyTextureRegion(&dl, 0, 0, 0, &sl, nullptr);
        }
        D3D12_RESOURCE_BARRIER b{};
        b.Type = D3D12_RESOURCE_BARRIER_TYPE_TRANSITION;
        b.Transition.pResource = m_maps[k].Get();
        b.Transition.Subresource = D3D12_RESOURCE_BARRIER_ALL_SUBRESOURCES;
        b.Transition.StateBefore = D3D12_RESOURCE_STATE_COPY_DEST;
        b.Transition.StateAfter = D3D12_RESOURCE_STATE_PIXEL_SHADER_RESOURCE;
        cl->ResourceBarrier(1, &b);
        m_mapSrv[k] = gpu.CreateSrvArray(m_maps[k].Get(), DXGI_FORMAT_R8_UNORM, 1, kPSlices);
        m_mapSrvCube[k] = gpu.SrvHeap().Alloc();
        D3D12_SHADER_RESOURCE_VIEW_DESC cv{};
        cv.Shader4ComponentMapping = D3D12_DEFAULT_SHADER_4_COMPONENT_MAPPING;
        cv.Format = DXGI_FORMAT_R8_UNORM;
        cv.ViewDimension = D3D12_SRV_DIMENSION_TEXTURECUBEARRAY;
        cv.TextureCubeArray.MipLevels = 1;
        cv.TextureCubeArray.First2DArrayFace = 0;
        cv.TextureCubeArray.NumCubes = 1;
        gpu.Device()->CreateShaderResourceView(m_maps[k].Get(), &cv,
                                               gpu.SrvHeap().Cpu(m_mapSrvCube[k]));
    }
    gpu.EndUpload();
    return true;
}

bool FloorProbe::CreatePipeline(Gpu& gpu, ShaderCompiler& sc, const std::wstring& shaderDir) {
    // Fifteen root constants, the heap as Texture2DArray[] and TextureCubeArray[] (spaces 5 and
    // 6, as the shared layout has them), and the shared layout's s0 and s3 field for field
    // (Renderer.cpp): the engine's trilinear and anisotropic samplers.
    hal::RootLayout rl;
    rl.Constants(0, static_cast<uint32_t>(sizeof(FloorCb) / 4))
        .Table({hal::SrvRange(0, hal::kUnbounded, 5), hal::SrvRange(0, hal::kUnbounded, 6)})
        .Sampler(hal::StaticSampler(0, D3D12_FILTER_MIN_MAG_MIP_LINEAR,
                                    D3D12_TEXTURE_ADDRESS_MODE_CLAMP))
        .Sampler(hal::StaticSampler(3, D3D12_FILTER_ANISOTROPIC, D3D12_TEXTURE_ADDRESS_MODE_CLAMP,
                                    8));
    m_rs = rl.Build(gpu, "restest.floor");
    hal::GraphicsPipelineDesc pd;
    pd.rootSig = m_rs.Get();
    pd.vs = sc.Compile(shaderDir + L"/ResidencyFloor.hlsl", L"VsFloor", L"vs_6_0");
    pd.ps = sc.Compile(shaderDir + L"/ResidencyFloor.hlsl", L"PsFloor", L"ps_6_0");
    pd.rtvFormat = DXGI_FORMAT_R32G32B32A32_FLOAT;
    pd.dsvFormat = DXGI_FORMAT_UNKNOWN;
    m_pso = hal::BuildGraphics(gpu, pd, "restest.floor");
    if (!m_pso) {
        Log("[restest] gpu: FAIL -- the probe's pipeline did not build "
            "(shaders/ResidencyFloor.hlsl)");
        return false;
    }
    // A float target, cleared to -1: a pixel no draw reached can never pass for a sample.
    D3D12_CLEAR_VALUE clear{};
    clear.Format = DXGI_FORMAT_R32G32B32A32_FLOAT;
    for (float& c : clear.Color) c = -1.0f;
    m_rt = gpu.CreateTexture2D(2 * kQuadsMax, 2 * kRowsPerScan, clear.Format,
                               D3D12_RESOURCE_FLAG_ALLOW_RENDER_TARGET,
                               D3D12_RESOURCE_STATE_RENDER_TARGET, L"restest.floor rows", &clear);
    m_rtv = gpu.RtvHeap().Cpu(gpu.RtvHeap().Alloc());
    gpu.Device()->CreateRenderTargetView(m_rt.res.Get(), nullptr, m_rtv);
    return true;
}

void FloorProbe::DrawScan(Gpu& gpu, const ProbeScan& scan) {
    const float clearColor[4] = {-1.0f, -1.0f, -1.0f, -1.0f};
    auto* cl = gpu.BeginUpload();
    {
        PixScope scope(cl, "restest.floor scan (every law x 2 samplers x 7 shapes x 12 scales)");
        ID3D12DescriptorHeap* heaps[] = {gpu.SrvHeap().Heap()};
        cl->SetDescriptorHeaps(1, heaps);
        cl->ClearRenderTargetView(m_rtv, clearColor, 0, nullptr);
        cl->OMSetRenderTargets(1, &m_rtv, FALSE, nullptr);
        cl->SetGraphicsRootSignature(m_rs.Get());
        cl->SetPipelineState(m_pso.Get());
        cl->SetGraphicsRootDescriptorTable(1, gpu.SrvHeap().Gpu(0));
        cl->IASetPrimitiveTopology(D3D_PRIMITIVE_TOPOLOGY_TRIANGLELIST);
        for (int law = 0; law < kLaws; ++law) {
            for (int aniso = 0; aniso < 2; ++aniso) {
                for (int shape = 0; shape < kShapes; ++shape) {
                    for (int scale = 0; scale < kScales; ++scale) {
                        const ProbeLaw& pl = kProbeLaws[law];
                        FloorCb c{};
                        c.arr = scan.cube ? m_arrSrvCube : m_arrSrv;
                        c.map = scan.cube ? m_mapSrvCube[pl.map] : m_mapSrv[pl.map];
                        c.slice = scan.slice;
                        c.law = pl.law;
                        c.aniso = static_cast<uint32_t>(aniso);
                        c.cube = scan.cube ? 1u : 0u;
                        c.start[0] = static_cast<float>(scan.x0 / kR);
                        c.start[1] = static_cast<float>(scan.y / kR);
                        c.step[0] = static_cast<float>(1.0 / (16.0 * kR));
                        c.step[1] = 0.0f;
                        RowJ(shape, scale, c.jx, c.jy);
                        c.fixed = 0.0f;
                        const size_t row = Row(law, aniso, shape, scale);
                        const D3D12_VIEWPORT vp{0.0f, float(2 * row), float(2 * scan.quads), 2.0f,
                                                0.0f, 1.0f};
                        const D3D12_RECT sr{0, LONG(2 * row), LONG(2 * scan.quads),
                                            LONG(2 * row + 2)};
                        cl->RSSetViewports(1, &vp);
                        cl->RSSetScissorRects(1, &sr);
                        cl->SetGraphicsRoot32BitConstants(
                            0, static_cast<UINT>(sizeof(FloorCb) / 4), &c, 0);
                        cl->DrawInstanced(3, 1, 0, 0);
                    }
                }
            }
        }
    }
    gpu.EndUpload();
}

// THE MECHANISM, measured on its own: a FIXED clamp under a small footprint along the scan,
// walking toward the window's 0 -> 7 cliff at x = 64 cells, whose left side is resident at every
// mip. How far before the cliff do the samples touching a NULL tile begin -- the run of touching
// points that ends at the cliff -- by sampler, ratio, footprint and clamp? One bilinear tap at
// mip m reaches 2^m / 128 of a cell from its centre; a reach beyond that, growing with the RATIO
// under a footprint far smaller than a cell, is the anisotropic filter spreading its taps by the
// CLAMPED mip's texels rather than by the footprint's own length.
void FloorProbe::Mechanism(Gpu& gpu) {
    const float kClamp[] = {3.0f, 4.0f, 5.0f, 5.5f, 6.0f, 6.5f};
    struct Kind {
        const char* name;
        uint32_t aniso;
        int ratio;
        double texels;   // the long axis, mip-0 texels a pixel
    };
    const Kind kKind[] = {{"trilinear, 1:1, 2 texels", 0, 1, 2.0},
                          {"anisotropic, 1:1, 2 texels", 1, 1, 2.0},
                          {"anisotropic, 1:1, 0.5 texel", 1, 1, 0.5},
                          {"anisotropic, 4:1, 0.5 texel", 1, 4, 0.5},
                          {"anisotropic, 4:1, 2 texels", 1, 4, 2.0},
                          {"anisotropic, 8:1, 2 texels", 1, 8, 2.0},
                          {"anisotropic, 4:1, 8 texels", 1, 4, 8.0},
                          {"anisotropic, 8:1, 8 texels", 1, 8, 8.0}};
    constexpr uint32_t kQ = 160;   // x 56 .. 66 cells, a sixteenth of a cell a quad
    constexpr double kX0 = 56.0, kY = 63.7;
    const float clearColor[4] = {-1.0f, -1.0f, -1.0f, -1.0f};
    const size_t nRows = std::size(kKind) * std::size(kClamp);
    auto* cl = gpu.BeginUpload();
    {
        PixScope scope(cl, "restest.floor mechanism (fixed clamps, a half-texel footprint)");
        ID3D12DescriptorHeap* heaps[] = {gpu.SrvHeap().Heap()};
        cl->SetDescriptorHeaps(1, heaps);
        cl->ClearRenderTargetView(m_rtv, clearColor, 0, nullptr);
        cl->OMSetRenderTargets(1, &m_rtv, FALSE, nullptr);
        cl->SetGraphicsRootSignature(m_rs.Get());
        cl->SetPipelineState(m_pso.Get());
        cl->SetGraphicsRootDescriptorTable(1, gpu.SrvHeap().Gpu(0));
        cl->IASetPrimitiveTopology(D3D_PRIMITIVE_TOPOLOGY_TRIANGLELIST);
        for (size_t row = 0; row < nRows; ++row) {
            const Kind& k = kKind[row / std::size(kClamp)];
            FloorCb c{};
            c.arr = m_arrSrv;
            c.map = m_mapSrv[0];
            c.slice = kPWindow;
            c.law = 2;
            c.aniso = k.aniso;
            c.start[0] = static_cast<float>(kX0 / kR);
            c.start[1] = static_cast<float>(kY / kR);
            c.step[0] = static_cast<float>(1.0 / (16.0 * kR));
            c.jx[0] = static_cast<float>(k.texels / kPDim);
            c.jy[1] = static_cast<float>(k.texels / kPDim / k.ratio);
            c.fixed = kClamp[row % std::size(kClamp)];
            const D3D12_VIEWPORT vp{0.0f, float(2 * row), float(2 * kQ), 2.0f, 0.0f, 1.0f};
            const D3D12_RECT sr{0, LONG(2 * row), LONG(2 * kQ), LONG(2 * row + 2)};
            cl->RSSetViewports(1, &vp);
            cl->RSSetScissorRects(1, &sr);
            cl->SetGraphicsRoot32BitConstants(0, static_cast<UINT>(sizeof(FloorCb) / 4), &c, 0);
            cl->DrawInstanced(3, 1, 0, 0);
        }
    }
    gpu.EndUpload();
    uint32_t pitch = 0;
    const std::vector<uint8_t> px = gpu.ReadbackTexture(m_rt, &pitch);
    Log("[restest] gpu mechanism: a FIXED clamp under a small footprint along the scan, toward "
        "the window's 0 -> 7 cliff at x = 64 (its left resident at every mip): cells before the "
        "cliff where the run of samples touching a NULL tile begins");
    std::string head;
    for (float c : kClamp) {
        char b[16];
        snprintf(b, sizeof b, " %6.1f", c);
        head += b;
    }
    Log("[restest]   %-30s%s", "clamp", head.c_str());
    // Quad q stands at x = kX0 + q / 16; the one just left of the cliff is q = 16 (64 - kX0) - 1.
    const uint32_t qCliff = static_cast<uint32_t>(16.0 * (64.0 - kX0)) - 1;
    for (size_t ki = 0; ki < std::size(kKind); ++ki) {
        std::string line;
        for (size_t ci = 0; ci < std::size(kClamp); ++ci) {
            const size_t row = ki * std::size(kClamp) + ci;
            const auto touched = [&](uint32_t q) {
                for (uint32_t k = 0; k < 4; ++k) {
                    const uint32_t x = 2 * q + (k & 1), y = 2 * uint32_t(row) + (k >> 1);
                    const float* f =
                        reinterpret_cast<const float*>(px.data() + size_t(y) * pitch) + 4 * x;
                    if (f[3] == 0.0f) return true;
                }
                return false;
            };
            uint32_t q = qCliff + 1;
            while (q > 0 && touched(q - 1)) --q;
            char b[16];
            snprintf(b, sizeof b, " %6.2f", 64.0 - (kX0 + q / 16.0));
            line += b;
        }
        Log("[restest]   %-30s%s", kKind[ki].name, line.c_str());
    }
    std::string tap;
    for (float c : kClamp) {
        char b[16];
        snprintf(b, sizeof b, " %6.2f", std::ldexp(1.0, static_cast<int>(c)) / kR);
        tap += b;
    }
    Log("[restest]   %-30s%s", "one bilinear tap's reach", tap.c_str());
}

// Rows of any kind, one quad a point, row i at target rows 2i and 2i + 1 (the target is cleared).
void FloorProbe::DrawRows(Gpu& gpu, const std::vector<FloorCb>& rows, uint32_t quads) {
    const float clearColor[4] = {-1.0f, -1.0f, -1.0f, -1.0f};
    auto* cl = gpu.BeginUpload();
    {
        PixScope scope(cl, "restest.floor rows");
        ID3D12DescriptorHeap* heaps[] = {gpu.SrvHeap().Heap()};
        cl->SetDescriptorHeaps(1, heaps);
        cl->ClearRenderTargetView(m_rtv, clearColor, 0, nullptr);
        cl->OMSetRenderTargets(1, &m_rtv, FALSE, nullptr);
        cl->SetGraphicsRootSignature(m_rs.Get());
        cl->SetPipelineState(m_pso.Get());
        cl->SetGraphicsRootDescriptorTable(1, gpu.SrvHeap().Gpu(0));
        cl->IASetPrimitiveTopology(D3D_PRIMITIVE_TOPOLOGY_TRIANGLELIST);
        for (size_t row = 0; row < rows.size(); ++row) {
            const D3D12_VIEWPORT vp{0.0f, float(2 * row), float(2 * quads), 2.0f, 0.0f, 1.0f};
            const D3D12_RECT sr{0, LONG(2 * row), LONG(2 * quads), LONG(2 * row + 2)};
            cl->RSSetViewports(1, &vp);
            cl->RSSetScissorRects(1, &sr);
            cl->SetGraphicsRoot32BitConstants(0, static_cast<UINT>(sizeof(FloorCb) / 4), &rows[row],
                                              0);
            cl->DrawInstanced(3, 1, 0, 0);
        }
    }
    gpu.EndUpload();
}

// THE MECHANISM AT A FRACTIONAL CLAMP L + f: the same walk toward the window's cliff at x = 64.
// At 6 + f mip 7 is resident on both sides, so the run of points touching a NULL tile is mip 6's
// reach alone, in texels of mip 6 (a cell is 2 of them); at 5 + f mips 5 and 6 are both NULL past
// the cliff and the run is the longer of the two, counted in texels of mip 5. Beside each, the
// bound (N / 2) 2^f + 1 texels of the finer level. The run is measured a sixteenth of a cell at a
// time: an eighth of a texel of mip 6, a quarter of mip 5.
void FloorProbe::MechanismFrac(Gpu& gpu) {
    const float kClamp[] = {5.5f, 5.9f, 6.0f, 6.25f, 6.5f, 6.75f, 6.9f};
    struct Kind {
        const char* name;
        int ratio;
        double texels;   // the long axis, mip-0 texels a pixel
    };
    const Kind kKind[] = {{"anisotropic, 4:1, 8 texels", 4, 8.0},
                          {"anisotropic, 8:1, 8 texels", 8, 8.0},
                          {"anisotropic, 8:1, 64 texels", 8, 64.0}};
    constexpr uint32_t kQ = 160;   // x 56 .. 66 cells, a sixteenth of a cell a quad
    constexpr double kX0 = 56.0, kY = 63.7;
    std::vector<FloorCb> rows;
    for (const Kind& k : kKind) {
        for (float clampValue : kClamp) {
            FloorCb c{};
            c.arr = m_arrSrv;
            c.map = m_mapSrv[0];
            c.slice = kPWindow;
            c.law = 2;
            c.aniso = 1;
            c.start[0] = static_cast<float>(kX0 / kR);
            c.start[1] = static_cast<float>(kY / kR);
            c.step[0] = static_cast<float>(1.0 / (16.0 * kR));
            c.jx[0] = static_cast<float>(k.texels / kPDim);
            c.jy[1] = static_cast<float>(k.texels / kPDim / k.ratio);
            c.fixed = clampValue;
            rows.push_back(c);
        }
    }
    DrawRows(gpu, rows, kQ);
    uint32_t pitch = 0;
    const std::vector<uint8_t> px = gpu.ReadbackTexture(m_rt, &pitch);
    Log("[restest] gpu mechanism at a FRACTIONAL clamp L + f, toward the same cliff: the run of "
        "samples touching a NULL tile, in cells and in texels of mip L, the finer level read, beside "
        "the bound (N / 2) 2^f + 1 of them (at 6 + f the run is mip 6's alone; at 5 + f the longer "
        "of mips 5 and 6)");
    const uint32_t qCliff = static_cast<uint32_t>(16.0 * (64.0 - kX0)) - 1;
    for (size_t ki = 0; ki < std::size(kKind); ++ki) {
        std::string line;
        for (size_t ci = 0; ci < std::size(kClamp); ++ci) {
            const size_t row = ki * std::size(kClamp) + ci;
            const auto touched = [&](uint32_t q) {
                for (uint32_t k = 0; k < 4; ++k) {
                    const uint32_t x = 2 * q + (k & 1), y = 2 * uint32_t(row) + (k >> 1);
                    const float* f =
                        reinterpret_cast<const float*>(px.data() + size_t(y) * pitch) + 4 * x;
                    if (f[3] == 0.0f) return true;
                }
                return false;
            };
            uint32_t q = qCliff + 1;
            while (q > 0 && touched(q - 1)) --q;
            const double cells = 64.0 - (kX0 + q / 16.0);
            const double lv = std::floor(double(kClamp[ci])), fr = double(kClamp[ci]) - lv;
            const double texels = cells * kR / std::ldexp(1.0, static_cast<int>(lv));
            const double bound = kKind[ki].ratio / 2.0 * std::exp2(fr) + 1.0;
            char b[96];
            snprintf(b, sizeof b, "%s%.2f: %.2f cells = %.2f texels (bound %.2f)",
                     line.empty() ? "" : "; ", kClamp[ci], cells, texels, bound);
            line += b;
        }
        Log("[restest]   %-28s %s", kKind[ki].name, line.c_str());
    }
}

// THE FRACTIONAL SIZES. Every size of the scans above is a whole power of two, so wherever the
// hardware's own LOD decides it is a whole level, which reads one level's taps. A footprint of
// 2^(k + f) texels reads levels L and L + 1 with its taps at the same places in uv, so at L, the
// finer of the pair, its long axis is 2^f times longer in that level's texels -- and a clamp of
// L + f, which the bilinear reads make on every ramp, reads L too with weight 1 - f. Measured here:
// the law in effect and the margins M = 4 .. 12 both ways, at 2^(k + 0.5) and 2^(k + 0.9) texels
// for every 2^k of the scans, ratios 4 and 8, the same four scans -- tallied apart, so the rows
// above stay comparable number for number. Returns each law's touched points, both samplers.
std::vector<uint64_t> FloorProbe::Fractional(Gpu& gpu) {
    const auto U = [](uint64_t v) { return static_cast<unsigned long long>(v); };
    std::vector<int> laws = {kInEffect};
    for (int k = 1; k < kNumMargins; ++k) laws.push_back(kMarginGather + k);
    for (int k = 1; k < kNumMargins; ++k) laws.push_back(kMarginBilinear + k);
    constexpr int kFr = 2 * kScales, kFShapes = kShapes - 1;   // shapes 1 .. 6: ratios 4 and 8
    const double kFrac[2] = {0.5, 0.9};
    const auto sizeLog2 = [&](int fs) { return (fs / 2) - 1 + kFrac[fs % 2]; };
    // Residency per level, the manager's closure: a mip-L tile is mapped when any cell under it
    // holds mip L or finer.
    std::vector<std::vector<std::vector<uint8_t>>> res(
        kPSlices, std::vector<std::vector<uint8_t>>(kPMips));
    for (uint32_t s = 0; s < kPSlices; ++s) {
        for (uint32_t L = 0; L < kPMips; ++L) {
            const uint32_t span = 1u << L, nt = kR / span;
            res[s][L].assign(size_t(nt) * nt, uint8_t(0));
            for (uint32_t y = 0; y < kR; ++y) {
                for (uint32_t x = 0; x < kR; ++x) {
                    if (m_true[s][size_t(y) * kR + x] <= L * 16) {
                        res[s][L][size_t(y / span) * nt + x / span] = 1;
                    }
                }
            }
        }
    }
    // From a point (cells) to the nearest NULL tile of mip L, in texels of mip L. Past a cube
    // face's edge the probe's neighbours hold mip 7 alone; a window clamps at its edge.
    const auto nullDist = [&](const ProbeScan& s, double px, double py, uint32_t L) {
        const uint32_t span = 1u << L, nt = kR / span;
        double best = 1e9;
        for (uint32_t ty = 0; ty < nt; ++ty) {
            for (uint32_t tx = 0; tx < nt; ++tx) {
                if (res[s.slice][L][size_t(ty) * nt + tx]) continue;
                const double x0 = double(tx * span), x1 = x0 + span;
                const double y0 = double(ty * span), y1 = y0 + span;
                const double dx = (std::max)({x0 - px, px - x1, 0.0});
                const double dy = (std::max)({y0 - py, py - y1, 0.0});
                best = (std::min)(best, std::sqrt(dx * dx + dy * dy));
            }
        }
        if (s.cube && L + 1 < kPMips) {
            const double n = double(kR);
            if (px < 0.0 || py < 0.0 || px > n || py > n) {
                best = 0.0;
            } else {
                best = (std::min)({best, px, n - px, py, n - py});
            }
        }
        return best * double(kR) / double(span);
    };
    struct FT {
        uint64_t n = 0, nClamp = 0, touched = 0, below = 0, clampT = 0, hwT = 0;
        uint64_t byRatio[2] = {};
        float worst = 0.0f;
        std::string first;
        double fracMax = -1.0, fracMaxDist = 0.0;
        bool fracMaxTouched = false;
        int fracMaxRatio = 0;
        double reachLB = -1.0, reachLBFrac = 0.0;
        int reachLBRatio = 0;
    };
    std::vector<FT> ft(laws.size() * 2);
    const size_t nRows = laws.size() * 2 * kFShapes * kFr;
    if (nRows > kRowsPerScan) {
        Log("[restest] gpu fractional: FAIL -- %zu rows do not fit the probe's target", nRows);
        return std::vector<uint64_t>(kLaws, 0);
    }
    for (const ProbeScan& s : kProbeScans) {
        std::vector<FloorCb> rows;
        rows.reserve(nRows);
        for (size_t li = 0; li < laws.size(); ++li) {
            const ProbeLaw& pl = kProbeLaws[laws[li]];
            for (int an = 0; an < 2; ++an) {
                for (int sh = 1; sh < kShapes; ++sh) {
                    for (int fs = 0; fs < kFr; ++fs) {
                        FloorCb c{};
                        c.arr = s.cube ? m_arrSrvCube : m_arrSrv;
                        c.map = s.cube ? m_mapSrvCube[pl.map] : m_mapSrv[pl.map];
                        c.slice = s.slice;
                        c.law = pl.law;
                        c.aniso = static_cast<uint32_t>(an);
                        c.cube = s.cube ? 1u : 0u;
                        c.start[0] = static_cast<float>(s.x0 / kR);
                        c.start[1] = static_cast<float>(s.y / kR);
                        c.step[0] = static_cast<float>(1.0 / (16.0 * kR));
                        c.step[1] = 0.0f;
                        RowJf(sh, sizeLog2(fs), c.jx, c.jy);
                        c.fixed = 0.0f;
                        rows.push_back(c);
                    }
                }
            }
        }
        DrawRows(gpu, rows, s.quads);
        uint32_t pitch = 0;
        const std::vector<uint8_t> px = gpu.ReadbackTexture(m_rt, &pitch);
        for (size_t row = 0; row < rows.size(); ++row) {
            const size_t li = row / (size_t(2) * kFShapes * kFr);
            const int an = static_cast<int>((row / (size_t(kFShapes) * kFr)) % 2);
            const int sh = 1 + static_cast<int>((row / kFr) % kFShapes);
            const int fs = static_cast<int>(row % kFr);
            const FloorCb& c = rows[row];
            FT& t = ft[li * 2 + an];
            for (uint32_t q = 0; q < s.quads; ++q) {
                for (uint32_t k = 0; k < 4; ++k) {
                    const uint32_t x = 2 * q + (k & 1), y = 2 * uint32_t(row) + (k >> 1);
                    const float* f =
                        reinterpret_cast<const float*>(px.data() + size_t(y) * pitch) + 4 * x;
                    const Px p{f[0], f[1], f[2], f[3]};
                    ++t.n;
                    const double ox = (k & 1) - 0.5, oy = (k >> 1) - 0.5;
                    const double pxc = s.x0 + q / 16.0 + (ox * c.jx[0] + oy * c.jy[0]) * kR;
                    const double pyc = s.y + (ox * c.jx[1] + oy * c.jy[1]) * kR;
                    const bool touched = p.mapped == 0.0f;
                    const bool clampDecides = p.have > p.lod;
                    if (clampDecides) {
                        ++t.nClamp;
                        const double fr = p.have - std::floor(p.have);
                        if (fr > t.fracMax) {
                            t.fracMax = fr;
                            t.fracMaxTouched = touched;
                            t.fracMaxRatio = kShapeRatio[sh];
                            t.fracMaxDist = nullDist(
                                s, pxc, pyc,
                                static_cast<uint32_t>(
                                    std::clamp(std::floor(p.have), 0.0f, float(kPMips - 1))));
                        }
                    }
                    if (p.r < 1.0f) {
                        ++t.below;
                        t.worst = (std::max)(t.worst, 1.0f - p.r);
                    }
                    if (!touched) continue;
                    ++t.touched;
                    ++t.byRatio[kShapeRatio[sh] == 8 ? 1 : 0];
                    ++(clampDecides ? t.clampT : t.hwT);
                    const float level =
                        std::clamp((std::max)(p.have, p.lod), 0.0f, float(kPMips - 1));
                    const uint32_t finer = static_cast<uint32_t>(std::floor(level));
                    const double d = nullDist(s, pxc, pyc, finer);
                    if (clampDecides && d > t.reachLB) {
                        t.reachLB = d;
                        t.reachLBFrac = p.have - std::floor(p.have);
                        t.reachLBRatio = kShapeRatio[sh];
                    }
                    if (t.first.empty()) {
                        char w[440];
                        snprintf(w, sizeof w,
                                 "[%s | %s, 2^%.1f texels, ratio %d at %d deg | (%.3f, %.3f) cells: "
                                 "the hardware's LOD %.3f, clamp %.3f, read %.5f; the nearest NULL "
                                 "tile of mip %u, the finer level read, %.2f texels of it away]",
                                 s.name, an ? "anisotropic 8x" : "trilinear", sizeLog2(fs),
                                 kShapeRatio[sh], kShapeAngle[sh], pxc, pyc, p.lod, p.have, p.r,
                                 finer, d);
                        t.first = w;
                    }
                }
            }
        }
    }
    Log("[restest] gpu FRACTIONAL SIZES, tallied apart from the rows above: %zu scans x 2 samplers "
        "x 6 footprints (ratios 4 and 8; the long axis along the scan, across it and at 45 "
        "degrees) x 24 sizes, 2^(k + 0.5) and 2^(k + 0.9) texels a pixel for k = -1 .. 10. Per "
        "law: points whose footprint touched a NULL tile, and points that read below 1:",
        std::size(kProbeScans));
    for (size_t li = 0; li < laws.size(); ++li) {
        for (int an = 0; an < 2; ++an) {
            const FT& t = ft[li * 2 + an];
            Log("[restest]   %-46s %-20s touched %6llu of %llu (ratio 4: %llu, 8: %llu; the clamp "
                "deciding %llu of %llu, the hardware %llu) | below 1: %llu, worst %.4f",
                an == 0 ? kProbeLaws[laws[li]].name : "",
                an ? "anisotropic 8x (s3)" : "trilinear (s0)", U(t.touched), U(t.n),
                U(t.byRatio[0]), U(t.byRatio[1]), U(t.clampT), U(t.nClamp), U(t.hwT), U(t.below),
                t.worst);
        }
    }
    for (size_t li = 0; li < laws.size(); ++li) {
        const FT& tri = ft[li * 2];
        const FT& ani = ft[li * 2 + 1];
        const uint64_t all = tri.touched + ani.touched;
        const std::string& first = !ani.first.empty() ? ani.first : tri.first;
        Log("[restest] gpu fractional verdict: %s is %s at every fractional size%s%s",
            kProbeLaws[laws[li]].name, all == 0 ? "SOUND" : "NOT SOUND",
            all ? " -- the first failing point: " : "", all ? first.c_str() : "");
        if (ani.nClamp > 0) {
            char reach[240] = "";
            if (ani.reachLB >= 0.0) {
                snprintf(reach, sizeof reach,
                         "; of those that touched, the farthest from a NULL tile of its finer "
                         "level: %.2f texels of it, at f = %.3f, ratio %d, where (N / 2) 2^f + 1 "
                         "= %.2f",
                         ani.reachLB, ani.reachLBFrac, ani.reachLBRatio,
                         ani.reachLBRatio / 2.0 * std::exp2(ani.reachLBFrac) + 1.0);
            }
            char dist[48];
            if (ani.fracMaxDist > 1e8) {
                snprintf(dist, sizeof dist, "none at that level");
            } else {
                snprintf(dist, sizeof dist, "%.2f texels", ani.fracMaxDist);
            }
            Log("[restest]   ... anisotropic, where the clamp decided (%llu points): the largest "
                "fractional part met %.3f (ratio %d; it %s; the nearest NULL tile of its finer "
                "level %s away)%s",
                U(ani.nClamp), ani.fracMax, ani.fracMaxRatio,
                ani.fracMaxTouched ? "touched one" : "touched none", dist, reach);
        }
    }
    std::vector<uint64_t> out(kLaws, 0);
    for (size_t li = 0; li < laws.size(); ++li) {
        out[laws[li]] = ft[li * 2].touched + ft[li * 2 + 1].touched;
    }
    return out;
}

bool FloorProbe::Run(Gpu& gpu, ShaderCompiler& sc, const std::wstring& shaderDir) {
    if (gpu.TiledTier() < D3D12_TILED_RESOURCES_TIER_2) {
        Log("[restest] gpu: FAIL -- below tier 2 a NULL tile reads undefined bytes, so no sample "
            "here can be judged");
        return false;
    }
    if (!CreateArray(gpu) || !CreateMaps(gpu) || !CreatePipeline(gpu, sc, shaderDir)) return false;
    Log("[restest] gpu array: reserved %ux%u x%u slices x%u mips RGBA8 (the colour tenant's "
        "page; slices 0-5 a cube, 6 a window), %zu tiles mapped and filled with 255 at every "
        "resident mip, the rest NULL. The window's block (cells 60-63 x 56-71) meets a 0 -> 7 "
        "cliff on a mip-6 tile line at x = 64 and a 3-4-5-6 stair to its left; face 0 has the "
        "same block at its +u edge against face 5 (mip 7 only there) and another at its corner",
        kPDim, kPDim, kPSlices, kPMips, m_tiles.size());

    // Every sample of every scan, kept: the laws are compared point for point.
    const size_t perScan = size_t(kRowsPerScan) * kQuadsMax * 4;
    std::vector<std::vector<Px>> all;
    for (const ProbeScan& scan : kProbeScans) {
        DrawScan(gpu, scan);
        uint32_t pitch = 0;
        const std::vector<uint8_t> px = gpu.ReadbackTexture(m_rt, &pitch);
        std::vector<Px> v(perScan, Px{-1, -1, -1, -1});
        for (uint32_t row = 0; row < kRowsPerScan; ++row) {
            for (uint32_t q = 0; q < scan.quads; ++q) {
                for (uint32_t k = 0; k < 4; ++k) {
                    const uint32_t x = 2 * q + (k & 1), y = 2 * row + (k >> 1);
                    const float* f = reinterpret_cast<const float*>(px.data() + size_t(y) * pitch) +
                                     4 * x;
                    v[(size_t(row) * kQuadsMax + q) * 4 + k] = Px{f[0], f[1], f[2], f[3]};
                }
            }
        }
        all.push_back(std::move(v));
    }
    const HRESULT removed = gpu.Device()->GetDeviceRemovedReason();
    if (FAILED(removed)) {
        Log("[restest] gpu: FAIL -- the device was removed during the probe: %s",
            HrString(removed).c_str());
        return false;
    }
    // The x of quad q along a scan, in cells, and of its pixel k: the four pixels sit half a
    // footprint about the quad's point, so under a long footprint a pixel's own place is what a
    // report must name.
    const auto cellX = [](const ProbeScan& s, uint32_t q) { return s.x0 + q / 16.0; };
    const auto pixelX = [&](const ProbeScan& s, uint32_t q, uint32_t k, int shape, int scale) {
        float jx[2], jy[2];
        RowJ(shape, scale, jx, jy);
        return cellX(s, q) + ((k & 1) - 0.5) * jx[0] * kR + ((k >> 1) - 0.5) * jy[0] * kR;
    };

    // ---- THE FILL'S OWN CHECK: no clamp, the finest footprint (2^-1 texel), the trilinear
    // sampler, ratio 1 -- mip 0 alone. A point well inside a block must read 1 and be called
    // mapped; a point in a NULL cell beside it 0 and not. Anything else and the probe's picture
    // of the array is not the array's, and nothing below can be judged.
    uint32_t fillIn = 0, fillOut = 0, fillBad = 0;
    float cleanMin = 2.0f;
    for (size_t si = 0; si < std::size(kProbeScans); ++si) {
        const ProbeScan& s = kProbeScans[si];
        const size_t row = Row(kNoClamp, 0, 0, 0);
        const double edge = s.cube ? 128.0 : 64.0;   // the block's right side
        for (uint32_t q = 0; q < s.quads; ++q) {
            const double x = cellX(s, q);
            const bool in = x > edge - 3.9 && x < edge - 0.1;
            const bool out = (x > edge + 0.1 && x < edge + 7.9) || x < edge - 4.1;
            if (!in && !out) continue;
            for (uint32_t k = 0; k < 4; ++k) {
                const Px& p = all[si][(row * kQuadsMax + q) * 4 + k];
                if (in) {
                    ++fillIn;
                    cleanMin = (std::min)(cleanMin, p.r);
                    if (p.r != 1.0f || p.mapped != 1.0f) ++fillBad;
                } else {
                    ++fillOut;
                    if (p.r != 0.0f || p.mapped != 0.0f) ++fillBad;
                }
            }
        }
    }
    Log("[restest] gpu fill: with no clamp at mip 0, %u points inside the blocks read %s and are "
        "called mapped, %u points in the NULL cells beside them read 0 and are not -- %s",
        fillIn, fillBad == 0 ? "1" : "NOT all 1", fillOut,
        fillBad == 0 ? "the probe's picture of the array is the array's"
                     : "BROKEN: the fill or the NULLs are not what the probe believes");
    if (fillBad != 0) {
        Log("[restest] gpu: FAIL -- %u of %u fill points disagree (the smallest inside reads %.7f)",
            fillBad, fillIn + fillOut, cleanMin);
        return false;
    }
    Mechanism(gpu);
    MechanismFrac(gpu);

    // ---- PER LAW: how many points touched a NULL tile (the hardware's word) and how many read
    // below 1 (the value), by sampler and ratio, and whether the clamp or the hardware's own LOD
    // decided the level there. Every point of every scan, footprint and scale counts.
    struct Tally {
        uint64_t n = 0, touched = 0, below = 0, touchedClamp = 0, touchedHw = 0, nClamp = 0;
        uint64_t byRatio[3] = {};
        float worst = 0.0f;   // the largest admixture, 1 - r
    };
    const auto ratioIdx = [](int shape) { return kShapeRatio[shape] == 1 ? 0 : kShapeRatio[shape] == 4 ? 1 : 2; };
    Tally tally[kLaws][2];
    for (size_t si = 0; si < std::size(kProbeScans); ++si) {
        const ProbeScan& s = kProbeScans[si];
        for (int law = 0; law < kLaws; ++law) {
            if (law == kNoClamp) continue;
            for (int an = 0; an < 2; ++an) {
                for (int shape = 0; shape < kShapes; ++shape) {
                    for (int scale = 0; scale < kScales; ++scale) {
                        const size_t row = Row(law, an, shape, scale);
                        for (uint32_t q = 0; q < s.quads; ++q) {
                            for (uint32_t k = 0; k < 4; ++k) {
                                const Px& p = all[si][(row * kQuadsMax + q) * 4 + k];
                                Tally& t = tally[law][an];
                                ++t.n;
                                const bool clampDecides = p.have > p.lod;
                                if (clampDecides) ++t.nClamp;
                                if (p.mapped == 0.0f) {
                                    ++t.touched;
                                    ++t.byRatio[ratioIdx(shape)];
                                    ++(clampDecides ? t.touchedClamp : t.touchedHw);
                                }
                                if (p.r < 1.0f) {
                                    ++t.below;
                                    t.worst = (std::max)(t.worst, 1.0f - p.r);
                                }
                            }
                        }
                    }
                }
            }
        }
    }
    const char* kSampler[2] = {"trilinear (s0)", "anisotropic 8x (s3)"};
    Log("[restest] gpu: %zu scans x 2 samplers x 7 footprints (ratios 1, 4 and 8; the long axis "
        "along the scan, across it, and at 45 degrees) x 12 sizes (2^-1 .. 2^10 texels a pixel "
        "along the long axis), a point every sixteenth of a cell. Per law: points whose footprint "
        "the hardware says touched a NULL tile (CheckAccessFullyMapped), and points that read "
        "below 1:",
        std::size(kProbeScans));
    for (int law = 0; law < kLaws; ++law) {
        if (law == kNoClamp) continue;
        for (int an = 0; an < 2; ++an) {
            const Tally& t = tally[law][an];
            Log("[restest]   %-46s %-20s touched %6llu of %llu (ratio 1: %llu, 4: %llu, 8: %llu; "
                "the clamp deciding %llu of %llu, the hardware %llu) | below 1: %llu, worst %.4f",
                an == 0 ? kProbeLaws[law].name : "", kSampler[an],
                static_cast<unsigned long long>(t.touched), static_cast<unsigned long long>(t.n),
                static_cast<unsigned long long>(t.byRatio[0]),
                static_cast<unsigned long long>(t.byRatio[1]),
                static_cast<unsigned long long>(t.byRatio[2]),
                static_cast<unsigned long long>(t.touchedClamp),
                static_cast<unsigned long long>(t.nClamp),
                static_cast<unsigned long long>(t.touchedHw),
                static_cast<unsigned long long>(t.below), t.worst);
        }
    }

    // ---- WHERE THE LAW IN EFFECT ITSELF LETS A NULL TILE IN, scan by scan: what has always
    // shipped, and so a finding, not a failure.
    for (size_t si = 0; si < std::size(kProbeScans); ++si) {
        const ProbeScan& s = kProbeScans[si];
        uint64_t n = 0;
        double xLo = 1e9, xHi = -1e9, lodLo = 1e9, lodHi = -1e9;
        int scaleLo = 99, scaleHi = -1, ratioLo = 99, ratioHi = 0;
        bool clampDecided = false;
        for (int an = 0; an < 2; ++an) {
            for (int shape = 0; shape < kShapes; ++shape) {
                for (int scale = 0; scale < kScales; ++scale) {
                    const size_t row = Row(kInEffect, an, shape, scale);
                    for (uint32_t q = 0; q < s.quads; ++q) {
                        for (uint32_t k = 0; k < 4; ++k) {
                            const Px& p = all[si][(row * kQuadsMax + q) * 4 + k];
                            if (p.mapped != 0.0f) continue;
                            ++n;
                            xLo = (std::min)(xLo, pixelX(s, q, k, shape, scale));
                            xHi = (std::max)(xHi, pixelX(s, q, k, shape, scale));
                            lodLo = (std::min)(lodLo, double(p.lod));
                            lodHi = (std::max)(lodHi, double(p.lod));
                            scaleLo = (std::min)(scaleLo, scale);
                            scaleHi = (std::max)(scaleHi, scale);
                            ratioLo = (std::min)(ratioLo, kShapeRatio[shape]);
                            ratioHi = (std::max)(ratioHi, kShapeRatio[shape]);
                            clampDecided = clampDecided || p.have > p.lod;
                        }
                    }
                }
            }
        }
        if (n == 0) {
            Log("[restest] gpu the law in effect, %s: no NULL tile touched", s.name);
            continue;
        }
        Log("[restest] gpu the law in effect, %s: %llu points touched a NULL tile -- x %.2f to %.2f "
            "cells, long axis 2^%d to 2^%d texels, ratio %d to %d, the hardware's LOD %.2f to %.2f%s",
            s.name, static_cast<unsigned long long>(n), xLo, xHi, scaleLo - 1, scaleHi - 1, ratioLo,
            ratioHi, lodLo, lodHi, clampDecided ? " (the clamp deciding at some)" : "");
    }

    // ---- WHERE A LAW TOUCHES WHAT THE LAW IN EFFECT DID NOT, point for point. The planted M6h
    // must: the law in effect admits NULL tiles of its own under a long anisotropic footprint
    // (the hardware's LOD deciding), so a plant is CAUGHT only by what it alone lets in.
    bool caught = false;
    std::vector<int> compared = {kFloor, kFloor2, kGatherFloor, kGatherFloor2, kPlanted};
    for (int k = 0; k < 2 * kNumMargins; ++k) compared.push_back(kMarginGather + k);
    for (int law : compared) {
        uint64_t newTouched = 0, newBelow = 0, lostTouched = 0, bySampler[2] = {};
        uint64_t byScan[std::size(kProbeScans)] = {};
        std::string where, shared;   // shared: the first point the law in effect touched as well
        int shown = 0;
        double haveLo = 1e9, haveHi = -1e9, xLo = 1e9, xHi = -1e9;
        int scaleLo = 99, scaleHi = -1, ratioLo = 99;
        for (size_t si = 0; si < std::size(kProbeScans); ++si) {
            const ProbeScan& s = kProbeScans[si];
            for (int an = 0; an < 2; ++an) {
                for (int shape = 0; shape < kShapes; ++shape) {
                    for (int scale = 0; scale < kScales; ++scale) {
                        const size_t rowL = Row(law, an, shape, scale);
                        const size_t rowR = Row(kInEffect, an, shape, scale);
                        for (uint32_t q = 0; q < s.quads; ++q) {
                            for (uint32_t k = 0; k < 4; ++k) {
                                const Px& p = all[si][(rowL * kQuadsMax + q) * 4 + k];
                                const Px& r = all[si][(rowR * kQuadsMax + q) * 4 + k];
                                if (p.mapped == 0.0f && r.mapped != 0.0f) {
                                    const double x = pixelX(s, q, k, shape, scale);
                                    ++newTouched;
                                    ++bySampler[an];
                                    ++byScan[si];
                                    haveLo = (std::min)(haveLo, double(p.have));
                                    haveHi = (std::max)(haveHi, double(p.have));
                                    xLo = (std::min)(xLo, x);
                                    xHi = (std::max)(xHi, x);
                                    scaleLo = (std::min)(scaleLo, scale);
                                    scaleHi = (std::max)(scaleHi, scale);
                                    ratioLo = (std::min)(ratioLo, kShapeRatio[shape]);
                                    if (shown++ < 3) {
                                        char w[320];
                                        snprintf(w, sizeof w,
                                                 "%s[%s | %s, ratio %d at %d deg, 2^%d texels | x "
                                                 "%.3f cells: clamp %.3f against %.3f in effect, "
                                                 "the hardware's LOD %.3f, read %.5f]",
                                                 where.empty() ? "" : " ", s.name, kSampler[an],
                                                 kShapeRatio[shape], kShapeAngle[shape], scale - 1,
                                                 x, p.have, r.have, p.lod, p.r);
                                        where += w;
                                    }
                                }
                                if (p.r < 1.0f && r.r >= 1.0f) ++newBelow;
                                if (p.mapped != 0.0f && r.mapped == 0.0f) ++lostTouched;
                                if (p.mapped == 0.0f && r.mapped == 0.0f && shared.empty()) {
                                    const double x = pixelX(s, q, k, shape, scale);
                                    char w[480];
                                    snprintf(w, sizeof w,
                                             "[%s | %s, ratio %d at %d deg, 2^%d texels | x %.3f "
                                             "cells: clamp %.3f (in effect %.3f), the hardware's "
                                             "LOD %.3f, read %.5f | %s]",
                                             s.name, kSampler[an], kShapeRatio[shape],
                                             kShapeAngle[shape], scale - 1, x, p.have, r.have,
                                             p.lod, p.r,
                                             Around(s, x, kProbeLaws[law].map).c_str());
                                    shared = w;
                                }
                            }
                        }
                    }
                }
            }
        }
        Log("[restest] gpu %s against the law in effect: touched a NULL tile where it did not at "
            "%llu points, read below 1 where it read 1 at %llu; kept out %llu that it let in",
            kProbeLaws[law].name, static_cast<unsigned long long>(newTouched),
            static_cast<unsigned long long>(newBelow), static_cast<unsigned long long>(lostTouched));
        if (newTouched > 0) {
            std::string scans;
            for (size_t si = 0; si < std::size(kProbeScans); ++si) {
                char b[160];
                snprintf(b, sizeof b, "%s%s %llu", scans.empty() ? "" : "; ", kProbeScans[si].name,
                         static_cast<unsigned long long>(byScan[si]));
                scans += b;
            }
            Log("[restest]   ... trilinear %llu, anisotropic %llu; ratio %d and up; at clamps %.3f to "
                "%.3f, footprints 2^%d to 2^%d texels, x %.2f to %.2f cells (the pixels' own "
                "places); by scan: %s",
                static_cast<unsigned long long>(bySampler[0]),
                static_cast<unsigned long long>(bySampler[1]), ratioLo, haveLo, haveHi, scaleLo - 1,
                scaleHi - 1, xLo, xHi, scans.c_str());
            Log("[restest]   ... the first: %s", where.c_str());
        }
        if (law == kFloor || law == kFloor2) {
            Log("[restest] gpu verdict: %s is %s", kProbeLaws[law].name,
                newTouched == 0 && newBelow == 0
                    ? "SOUND against the law in effect on this GPU, at every footprint measured"
                    : "UNSOUND on this GPU: it lets in NULL tiles the law in effect keeps out "
                      "(and it is not staged)");
        }
        if (law >= kMarginGather) {
            const unsigned long long tT = tally[law][0].touched, tA = tally[law][1].touched;
            const unsigned long long own = tally[kInEffect][0].touched + tally[kInEffect][1].touched;
            Log("[restest] gpu verdict: %s touched %llu points under trilinear and %llu under "
                "anisotropic 8x -- %s; of the law in effect's own %llu it kept out %llu",
                kProbeLaws[law].name, tT, tA,
                tT + tA == 0 ? "SOUND at every whole-power size" : "NOT SOUND",
                own, static_cast<unsigned long long>(lostTouched));
            if (!shared.empty()) {
                Log("[restest]   ... the first point the law in effect touched as well: %s",
                    shared.c_str());
            }
            const int mk = (law - kMarginGather) % kNumMargins;
            if (kMargins[mk] < 7.0) {
                Log("[restest] gpu planted: margin M=%.0f (%s) %s", kMargins[mk],
                    law < kMarginBilinear ? "gather" : "bilinear",
                    tT + tA > 0 ? "touches NULL tiles -- CAUGHT"
                                : "touches nothing -- NOT CAUGHT: enough here, or the probe cannot "
                                  "see what it lets in");
            }
        }
        if (law == kPlanted) {
            caught = newTouched > 0 && newBelow > 0;
            Log("[restest] gpu planted: M6h %s",
                caught ? "reads NULL tiles the law in effect keeps out -- CAUGHT"
                       : "reads nothing the law in effect keeps out -- NOT CAUGHT: the probe cannot "
                         "see the failure it exists for");
        }
    }
    // ---- THE FRACTIONAL SIZES, tallied apart; then, per read, the smallest margin that reads zero
    // at every size, whole and fractional.
    const std::vector<uint64_t> fracTouched = Fractional(gpu);
    for (int v = 0; v < 2; ++v) {
        std::string line;
        int smallest = -1;
        for (int k = 0; k < kNumMargins; ++k) {
            const int law = (v ? kMarginBilinear : kMarginGather) + k;
            const unsigned long long whole = tally[law][0].touched + tally[law][1].touched;
            const bool run = k > 0;   // M = 2 is not run at the fractional sizes
            char b[80];
            if (run) {
                snprintf(b, sizeof b, "%sM=%.0f %llu / %llu", line.empty() ? "" : "; ", kMargins[k],
                         whole, static_cast<unsigned long long>(fracTouched[law]));
            } else {
                snprintf(b, sizeof b, "%sM=%.0f %llu / not run", line.empty() ? "" : "; ",
                         kMargins[k], whole);
            }
            line += b;
            if (smallest < 0 && run && whole == 0 && fracTouched[law] == 0) smallest = k;
        }
        char m[16] = "none";
        if (smallest >= 0) snprintf(m, sizeof m, "M=%.0f", kMargins[smallest]);
        Log("[restest] gpu margin, %s: points touched at the whole-power sizes / at the fractional "
            "sizes: %s -- the smallest M reading zero at every size: %s",
            v ? "bilinear over F's 3 x 3" : "gather + max over F", line.c_str(), m);
    }
    return caught;
}

}  // namespace

bool RunResidencySelfTest(Gpu& gpu, ShaderCompiler& sc, const std::wstring& shaderDir) {
    Log("[restest] ---- the floor law (docs/HIERARCHY.md 4.6), proposed: the map the GPU reads "
        "the true one with each byte the largest of its 3 x 3, read bilinear -- its construction, "
        "and on this GPU against the law in effect ----");
    const auto t0 = std::chrono::steady_clock::now();
    bool ok = true;

    // Python's generator, held to Python: random.Random(3) draws randint(1, 6), randrange(24)
    // twice, randint(1, 8), randint(0, 6), random() = 2 18 17 3 2 0.9159448117309811 (py -3,
    // CPython 3.14.3). Gate 3 walks porch_floor.py's maps on this generator.
    {
        PyRandom r(3);
        const int a = r.RandInt(1, 6), b = r.RandRange(24), c = r.RandRange(24),
                  d = r.RandInt(1, 8), e = r.RandInt(0, 6);
        const double f = r.Random();
        const bool same = a == 2 && b == 18 && c == 17 && d == 3 && e == 2 &&
                          f == 0.9159448117309811;
        Log("[restest] Python's random.Random(3), reproduced: %d %d %d %d %d %.16f -- %s", a, b,
            c, d, e, f, same ? "the draws CPython makes" : "NOT CPython's draws");
        ok &= same;
    }

    // ---- 1. SOUND, on the six faces and a window of four random sets (one with a dead face).
    const std::vector<uint32_t> ring = ResidencyManager::CubeFloorRing(kR);
    PyRandom rnd(20260928);
    std::vector<Maps> sets;
    for (int k = 0; k < 4; ++k) sets.push_back(RandomSet(rnd, k == 3));
    SoundResult sound, noCross, noDiag;
    Maps floorMap;
    for (const Maps& t : sets) {
        ResidencyManager::FloorMap(t, kR, &ring, floorMap);
        CheckSound(t, floorMap, sound);
    }
    Log("[restest] sound: at %llu points (16 x 16 a cell, every cell of 4 sets of the six faces "
        "and a window) the bilinear read of the floor is below the largest of the four nearest "
        "true bytes at %llu%s",
        static_cast<unsigned long long>(sound.samples),
        static_cast<unsigned long long>(sound.seamBad),
        sound.seamBad ? (" -- FAIL, the first: " + sound.first).c_str() : "");
    Log("[restest] sound: across a face's edge the four are found by directions, not through "
        "CubeFloorRing -- keeping the place along the edge, as the hardware's seamless filter "
        "reads (the verdict above); by the literal nudge half a cell along this face's plane "
        "(which lands up to half a cell nearer the edge's middle on the neighbour) the read falls "
        "short at %llu points, %llu of them where the seam's four are covered",
        static_cast<unsigned long long>(sound.planeBad),
        static_cast<unsigned long long>(sound.planeOnly));
    ok &= sound.seamBad == 0;
    // Planted: the floor without its cross-face neighbours (every slice clamps at its edge), and
    // a 3 x 3 without its diagonals. The first two sets are enough for either to be seen.
    for (int k = 0; k < 2; ++k) {
        ResidencyManager::FloorMap(sets[k], kR, nullptr, floorMap);
        CheckSound(sets[k], floorMap, noCross);
        PlusFloor(sets[k], ring, floorMap);
        CheckSound(sets[k], floorMap, noDiag);
    }
    const bool caughtCross = noCross.seamBad > 0 && noCross.atEdge == noCross.seamBad;
    const bool caughtDiag = noDiag.seamBad > 0;
    Log("[restest] planted: the floor without its cross-face neighbours falls short at %llu of "
        "%llu points, %llu of them within half a cell of a face's edge (worst %.3f mips) -- %s",
        static_cast<unsigned long long>(noCross.seamBad),
        static_cast<unsigned long long>(noCross.samples),
        static_cast<unsigned long long>(noCross.atEdge), noCross.worst,
        caughtCross ? "CAUGHT, at the edges and only there" : "NOT CAUGHT as it must be");
    Log("[restest] planted: a 3 x 3 without its diagonals falls short at %llu points (worst %.3f "
        "mips; the first: %s) -- %s",
        static_cast<unsigned long long>(noDiag.seamBad), noDiag.worst, noDiag.first.c_str(),
        caughtDiag ? "CAUGHT" : "NOT CAUGHT");
    ok &= caughtCross && caughtDiag;

    // ---- 2. THE KERNELS' TAPS: the floor, then the finding as it shipped, then no diagonals.
    TapResult taps, shipped, plusTaps;
    PyRandom uvs(9);
    for (const Maps& t : sets) {
        ResidencyManager::FloorMap(t, kR, &ring, floorMap);
        CheckTaps(t, floorMap, uvs, 1u << 17, taps);
        CheckTaps(t, t, uvs, 1u << 17, shipped);
        PlusFloor(t, ring, floorMap);
        CheckTaps(t, floorMap, uvs, 1u << 17, plusTaps);
    }
    Log("[restest] taps: %llu kernel reads (PageHaveLoad's one byte, PageLoad4's four texels at "
        "its rounded mip, in the shader's float arithmetic): %llu of %llu taps lie in a cell "
        "coarser than the byte read%s",
        static_cast<unsigned long long>(taps.reads), static_cast<unsigned long long>(taps.bad),
        static_cast<unsigned long long>(taps.taps),
        taps.bad ? (" -- FAIL, the first: " + taps.first).c_str() : "");
    Log("[restest] planted: the same reads of the TRUE map (review finding 9, as it shipped): "
        "%llu of %llu taps in a coarser cell -- %s; the first: %s",
        static_cast<unsigned long long>(shipped.bad), static_cast<unsigned long long>(shipped.taps),
        shipped.bad ? "CAUGHT" : "NOT CAUGHT", shipped.first.c_str());
    Log("[restest] planted: of a 3 x 3 without its diagonals: %llu of %llu taps -- %s",
        static_cast<unsigned long long>(plusTaps.bad), static_cast<unsigned long long>(plusTaps.taps),
        plusTaps.bad ? "CAUGHT" : "not seen by this gate (the sound gate above is the one)");
    ok &= taps.bad == 0 && shipped.bad > 0;

    // ---- 3. THE HARNESS'S NUMBERS: porch_floor.py printed samples 76800, unsound 0, largest
    // steps 7.000 and 0.438 mips, mean cost 0.151 mips.
    // Held to the figures as printed (%.3f both sides): the floor's largest step is 7/16 =
    // 0.4375, which a tolerance of half a unit in the third figure would call different.
    const Steps porch = PorchFloor(Read::FloorBilinear, 1);
    char printed[64];
    snprintf(printed, sizeof printed, "%.3f %.3f %.3f", porch.stepOld, porch.stepNew,
             porch.extra / double(porch.samples));
    const bool agree = porch.samples == 76800 && porch.unsound == 0 &&
                       std::string(printed) == "7.000 0.438 0.151";
    Log("[restest] harness: porch_floor.py's 200 maps (its generator, seed 3) through FloorMap: "
        "samples %llu; finer than the gather's clamp %llu; largest step between samples 1/16 cell "
        "apart: gather %.3f mips, floor %.3f mips; mean cost %.3f mips -- %s",
        static_cast<unsigned long long>(porch.samples),
        static_cast<unsigned long long>(porch.unsound), porch.stepOld, porch.stepNew,
        porch.extra / double(porch.samples),
        agree ? "EQUAL to its printed figures" : "DIFFERENT from its printed 76800 / 0 / 7.000 / "
                                                 "0.438 / 0.151");
    ok &= agree;
    {
        Steps eng;
        PyRandom rows(11);
        for (const Maps& t : sets) {
            ResidencyManager::FloorMap(t, kR, &ring, floorMap);
            EngineSteps(t, floorMap, true, rows, eng);
        }
        const Steps porch2 = PorchFloor(Read::FloorBilinear, 2);
        Log("[restest] harness: the same on the engine's 128-cell maps (%llu samples, 16 rows a "
            "slice): finer than the gather's clamp %llu; largest step gather %.3f, floor %.3f "
            "mips; mean cost %.3f mips. The floor twice (5 x 5) on porch_floor.py's maps: largest "
            "step %.3f, mean cost %.3f mips",
            static_cast<unsigned long long>(eng.samples),
            static_cast<unsigned long long>(eng.unsound), eng.stepOld, eng.stepNew,
            eng.extra / double(eng.samples), porch2.stepNew, porch2.extra / double(porch2.samples));
        ok &= eng.unsound == 0;
    }
    // ---- THE MARGIN LAW at M = 10 (8x + 2), both reads, beside the floors, on the same rows:
    // never finer than the gather in effect, the largest step, and the mean cost against it --
    // over all four sets, and over the three with no dead face (beside one, F is "nothing here").
    // Every map is first CLOSED (Closed): the margin law reads tiles, and a random map's cells are
    // not its own tiles'; read raw, F is the coarsest level almost everywhere.
    {
        struct Variant {
            const char* name;
            Steps all, live, porch;
        };
        Variant v[4] = {{"the 3 x 3 floor, bilinear"},
                        {"gather + max over the 5 x 5 floor"},
                        {"margin M=10, gather + max over F"},
                        {"margin M=10, bilinear over F's 3 x 3"}};
        const auto add = [](Steps& a, const Steps& b) {
            a.samples += b.samples;
            a.unsound += b.unsound;
            a.extra += b.extra;
            a.stepOld = (std::max)(a.stepOld, b.stepOld);
            a.stepNew = (std::max)(a.stepNew, b.stepNew);
        };
        PyRandom rowsOf[4] = {PyRandom(11), PyRandom(11), PyRandom(11), PyRandom(11)};
        Maps f1, f2, marg, margFloor;
        for (size_t k = 0; k < sets.size(); ++k) {
            const Maps t = Closed(sets[k], kR, 8);
            ResidencyManager::FloorMap(t, kR, &ring, f1);
            ResidencyManager::FloorMap(f1, kR, &ring, f2);
            MarginMap(t, kR, 8, true, 10.0, marg);
            ResidencyManager::FloorMap(marg, kR, &ring, margFloor);
            const Maps* d[4] = {&f1, &f2, &marg, &margFloor};
            const bool bilinear[4] = {true, false, false, true};
            for (int j = 0; j < 4; ++j) {
                Steps one;
                EngineSteps(t, *d[j], bilinear[j], rowsOf[j], one);
                add(v[j].all, one);
                if (k < 3) add(v[j].live, one);
            }
        }
        v[0].porch = PorchFloor(Read::FloorBilinear, 1, 0.0, true);
        v[1].porch = PorchFloor(Read::FloorGather, 2, 0.0, true);
        v[2].porch = PorchFloor(Read::MarginGather, 1, 10.0, true);
        v[3].porch = PorchFloor(Read::MarginBilinear, 1, 10.0, true);
        for (const Variant& x : v) {
            Log("[restest] harness, closed maps: %-37s | engine maps: finer than the gather at %llu of %llu, "
                "largest step %.3f mips, mean cost %.3f mips (%.3f on the three sets with no dead "
                "face) | porch_floor.py's maps: finer at %llu, largest step %.3f, mean cost %.3f",
                x.name, static_cast<unsigned long long>(x.all.unsound),
                static_cast<unsigned long long>(x.all.samples), x.all.stepNew,
                x.all.extra / double(x.all.samples), x.live.extra / double(x.live.samples),
                static_cast<unsigned long long>(x.porch.unsound), x.porch.stepNew,
                x.porch.extra / double(x.porch.samples));
            ok &= x.all.unsound == 0 && x.porch.unsound == 0;
        }
    }
    const double cpuMs =
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
    Log("[restest] the CPU gates took %.0f ms", cpuMs);

    // ---- 4. THE GPU.
    FloorProbe probe;
    ok &= probe.Run(gpu, sc, shaderDir);

    Log("[restest] ---- %s ----",
        ok ? "PASS: every instrument sees what it exists to see -- the floor's construction holds "
             "its arithmetic on the cube's faces, a window and the kernels' taps, porch_floor.py's "
             "numbers are reproduced, every planted defect was caught -- and the GPU verdicts "
             "above say what each law lets in on this GPU"
           : "FAIL: see above");
    return ok;
}

}  // namespace ga
