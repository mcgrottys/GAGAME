// RunTenantBindingSelfTest -- HIERARCHY 4.17 commit 1's gate on hal/Tenant.h's BlockBinding and on
// the block half of the tenant's dispatcher and change law, run from --selftest after the address
// block. Pure CPU, and it writes nothing: no TileTree is built (its constructor makes folders
// under cache\trees), so the tree here is a fake that fires the hook Bind(tree) hangs.
//
// Every expectation is computed apart from the code under test, from texel coordinates in doubles:
// a slot's tile from the block's origin (16384 bx texels of its rung, halved per mip) plus x tile
// widths, and the pyramid's mip from the face dimension of Lattice::Cube(16384 << 17) itself --
// never from X = bx tilesX(m) + x or from 17 - r + m. What is pinned, and against what:
//   1. The round trip, at rungs 0 3 6 9 12 17 on every face (the four corner blocks and two random
//      ones), in both tile shapes (128x128, and the height's 256x128), at every mip 0 to 7: the four
//      corner slots and 1000 random ones go to the doubles' global tile and back to themselves; the
//      tiles just outside the block at each mip, the block's tile on another face, and the mips
//      beside the slice's chain are refused; a mip the shape does not carry is refused both ways.
//   2. The ground: the block's corner directions, ComposeCubeDir of its uv box, against
//      ComposeCubeDir at the corner's texel coordinates in doubles (1e-12) and against step 3's
//      address (FaceWindow::TexelOf puts them on the corner's texels); the pyramid's own
//      Lattice::Texel at the outer texels of every mip's corner slots against the texel centres
//      the box gives (1e-12 rad); GroundRes bitwise the pyramid's at the doubles' mip, and
//      kMercCirc / (4 N) there.
//   3. Nesting: every slot of a rung-6 block at its mip m + 3 and the slot the rung-3 block over
//      the same ground holds at its mip m go to ONE pyramid tile, the doubles' one, and each is
//      found back from the other's; the same for rungs 9 and 6.
//   4. The Merrimack mouth (42.816 N, 70.8125 W, face 5): its blocks at rungs 3, 6 and 9, where in
//      them it stands, and the blocks a viewer there needs within 1, 8 and 66 km -- by exact
//      great-circle distance to each candidate block's four edge arcs. A record, with the checks
//      that tie it to the binding (the block holds the mouth, its slot is found back from its tile).
//   5. Dispatch and routing through Tenant::Unregistered, per tile shape: the cube binding and
//      three block slices over the mouth (rungs 3, 6 and 9) dispatched, each block's provider
//      asked exactly the doubles' global tile; then a fake tree announces pyramid tiles at every
//      mip from one finer than the finest slice's chain to one coarser than the coarsest's, and
//      each reaches exactly the slices that hold it, at the doubles' slot, once.
//   6. Three plants, each seen by the gate that must see it: tilesX on both axes (gate 1 on the
//      256x128 shape; on 128x128 it is the same function, and passing there is recorded), the
//      block's origin one tile off (gates 2 and 3), and a change law that stops at the first slice
//      that holds the tile (gate 5).
#include "hal/Tenant.h"

#include "core/Common.h"
#include "core/Lattice.h"

#include <algorithm>
#include <bit>
#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <functional>
#include <map>
#include <random>
#include <set>
#include <string>
#include <tuple>
#include <vector>

namespace ga::hal {

namespace {

constexpr double kPi = 3.141592653589793;
constexpr double kSphereR = 6371000.0;   // the address block's sphere (SpaceTest.cpp)

std::string Fmt(const char* fmt, ...) {
    char buf[640];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    return buf;
}
std::string Req(const TileRequest& r) { return Fmt("f%u m%u (%u,%u)", r.face, r.mip, r.x, r.y); }
std::string Blk(const BlockBinding& b) { return Fmt("f%u r%d (%u,%u)", b.face, b.rung, b.bx, b.by); }
bool Same(const TileRequest& a, const TileRequest& b) {
    return a.face == b.face && a.mip == b.mip && a.x == b.x && a.y == b.y;
}

// A gate's count: its checks, its failures, and the first failure named.
struct Verdict {
    uint64_t checks = 0, fails = 0;
    std::string first;
    template <class Why>
    bool Check(bool ok, Why&& why) {
        ++checks;
        if (!ok && fails++ == 0) first = why();
        return ok;
    }
    std::string Said() const {
        return fails ? Fmt("%llu of %llu checks FAIL, first: %s", (unsigned long long)fails,
                           (unsigned long long)checks, first.c_str())
                     : Fmt("%llu checks", (unsigned long long)checks);
    }
};

struct Shape {
    uint32_t w, h;
};
constexpr Shape kShapes[2] = {{128, 128}, {256, 128}};
constexpr int kRungs[6] = {0, 3, 6, 9, 12, 17};

// ---- the law under test, or a planted copy of it ----------------------------------------------
// Every gate reads the binding through one of these, so a plant is seen by the very instrument
// that passes the binding. The ground takes the tile shape only so that plant B can move the
// origin by one tile of it; the binding's own ground has no use for it.
struct Law {
    const char* name;
    bool (*global)(const BlockBinding&, const TileRequest&, uint32_t, uint32_t, TileRequest&);
    bool (*slot)(const BlockBinding&, const TileRequest&, uint32_t, uint32_t, TileRequest&);
    void (*ground)(const BlockBinding&, uint32_t, uint32_t, double&, double&, double&, double&);
    double (*groundRes)(const BlockBinding&, uint32_t);
};

const Law kBinding = {
    "the binding",
    [](const BlockBinding& b, const TileRequest& s, uint32_t w, uint32_t h, TileRequest& g) {
        return b.Global(s, w, h, g);
    },
    [](const BlockBinding& b, const TileRequest& g, uint32_t w, uint32_t h, TileRequest& s) {
        return b.Slot(g, w, h, s);
    },
    [](const BlockBinding& b, uint32_t, uint32_t, double& u0, double& v0, double& u1, double& v1) {
        b.Ground(u0, v0, u1, v1);
    },
    [](const BlockBinding& b, uint32_t m) { return b.GroundRes(m); },
};

// PLANT A: tilesX on both axes. On a square tile it IS the binding; on the height's 256x128 it
// puts Y at half its row and refuses the lower half of every grid.
bool GlobalSquare(const BlockBinding& b, const TileRequest& s, uint32_t w, uint32_t h,
                  TileRequest& g) {
    if (s.mip >= BlockBinding::Mips(w, h)) return false;
    const uint32_t tw = (Lattice::kFaceDim >> s.mip) / w, th = tw;   // the plant
    if (s.x >= tw || s.y >= th) return false;
    g = TileRequest{b.face, uint32_t(BlockBinding::kFinestRung - b.rung) + s.mip, b.bx * tw + s.x,
                    b.by * th + s.y};
    return true;
}
bool SlotSquare(const BlockBinding& b, const TileRequest& g, uint32_t w, uint32_t h,
                TileRequest& s) {
    const uint32_t top = uint32_t(BlockBinding::kFinestRung - b.rung);
    if (g.face != b.face || g.mip < top || g.mip - top >= BlockBinding::Mips(w, h)) return false;
    const uint32_t m = g.mip - top;
    const uint32_t tw = (Lattice::kFaceDim >> m) / w, th = tw;   // the plant
    const uint32_t x = g.x - b.bx * tw, y = g.y - b.by * th;
    if (x >= tw || y >= th) return false;
    s = TileRequest{0, m, x, y};
    return true;
}
const Law kPlantSquare = {"tilesX on both axes", GlobalSquare, SlotSquare, kBinding.ground,
                          kBinding.groundRes};

// PLANT B: the block's origin one tile off along u -- texW texels of its rung -- carried the way an
// origin kept in texels would be: halved per mip, then divided into tiles. The ground moves with it.
bool GlobalShifted(const BlockBinding& b, const TileRequest& s, uint32_t w, uint32_t h,
                   TileRequest& g) {
    if (s.mip >= BlockBinding::Mips(w, h)) return false;
    const uint32_t tw = (Lattice::kFaceDim >> s.mip) / w, th = (Lattice::kFaceDim >> s.mip) / h;
    if (s.x >= tw || s.y >= th) return false;
    const uint64_t ox = uint64_t(b.bx) * Lattice::kFaceDim + w;   // the plant
    const uint64_t oy = uint64_t(b.by) * Lattice::kFaceDim;
    g = TileRequest{b.face, uint32_t(BlockBinding::kFinestRung - b.rung) + s.mip,
                    uint32_t((ox >> s.mip) / w) + s.x, uint32_t((oy >> s.mip) / h) + s.y};
    return true;
}
bool SlotShifted(const BlockBinding& b, const TileRequest& g, uint32_t w, uint32_t h,
                 TileRequest& s) {
    const uint32_t top = uint32_t(BlockBinding::kFinestRung - b.rung);
    if (g.face != b.face || g.mip < top || g.mip - top >= BlockBinding::Mips(w, h)) return false;
    const uint32_t m = g.mip - top;
    const uint32_t tw = (Lattice::kFaceDim >> m) / w, th = (Lattice::kFaceDim >> m) / h;
    const uint64_t ox = uint64_t(b.bx) * Lattice::kFaceDim + w;   // the plant
    const uint64_t oy = uint64_t(b.by) * Lattice::kFaceDim;
    const uint32_t x = g.x - uint32_t((ox >> m) / w), y = g.y - uint32_t((oy >> m) / h);
    if (x >= tw || y >= th) return false;
    s = TileRequest{0, m, x, y};
    return true;
}
void GroundShifted(const BlockBinding& b, uint32_t w, uint32_t, double& u0, double& v0, double& u1,
                   double& v1) {
    b.Ground(u0, v0, u1, v1);
    const double d = double(w) / std::ldexp(16384.0, b.rung);   // the plant: one tile of the rung
    u0 += d;
    u1 += d;
}
const Law kPlantShifted = {"the block's origin one tile off", GlobalShifted, SlotShifted,
                           GroundShifted, kBinding.groundRes};

// ---- the expectation, from texel coordinates in doubles --------------------------------------
// The pyramid as the brief names it, and the mip at which its face is `texels` across.
const Lattice& Pyramid() {
    static const Lattice p = Lattice::Cube(16384u << 17);
    return p;
}
int PyramidMipOf(double texels) {
    for (uint32_t M = 0; M < 32; ++M) {
        if (double(Pyramid().faceDim >> M) == texels) return int(M);
    }
    return -1;
}
// Texels across the face at a rung's slice mip m: 16384 2^r, halved m times.
double FaceTexels(int rung, uint32_t m) { return std::ldexp(16384.0, rung - int(m)); }
// The slice's side at its mip m, and the block's origin there, in texels of that mip.
double Side(uint32_t m) { return std::ldexp(16384.0, -int(m)); }
double Origin(uint32_t b, uint32_t m) { return double(b) * Side(m); }
// A slice carries mip m in a tile shape when a whole tile fits across each axis.
bool Carries(uint32_t m, const Shape& s) { return Side(m) / s.w >= 1.0 && Side(m) / s.h >= 1.0; }
uint32_t CarriedMips(const Shape& s) {
    uint32_t n = 0;
    while (n < 8 && Carries(n, s)) ++n;
    return n;
}
// The global tile of slot (x, y) at the slice's mip m; false where a number is not whole.
bool Expect(const BlockBinding& b, uint32_t m, double x, double y, const Shape& s, TileRequest& g) {
    const int M = PyramidMipOf(FaceTexels(b.rung, m));
    const double X = (Origin(b.bx, m) + x * s.w) / s.w;
    const double Y = (Origin(b.by, m) + y * s.h) / s.h;
    if (M < 0 || X != std::floor(X) || Y != std::floor(Y)) return false;
    g = TileRequest{b.face, uint32_t(M), uint32_t(X), uint32_t(Y)};
    return true;
}

// The blocks gates 1 and 2 walk: at each rung and face, the face's four corner blocks and two
// drawn at random.
std::vector<BlockBinding> Walked(std::mt19937& rng) {
    std::vector<BlockBinding> out;
    for (const int r : kRungs) {
        const uint32_t n = 1u << r;
        std::uniform_int_distribution<uint32_t> any(0, n - 1);
        for (uint32_t f = 0; f < 6; ++f) {
            for (int c = 0; c < 4; ++c) out.push_back({f, r, (c & 1) ? n - 1 : 0, (c & 2) ? n - 1 : 0});
            for (int k = 0; k < 2; ++k) out.push_back({f, r, any(rng), any(rng)});
        }
    }
    return out;
}

// ---- 1. the round trip -----------------------------------------------------------------------
struct TripCount {
    uint64_t slots = 0, refused = 0;
};
Verdict GateRoundTrip(const Law& law, const std::vector<BlockBinding>& blocks, const Shape* shapes,
                      int nShapes, std::mt19937& rng, TripCount& n) {
    Verdict v;
    for (const BlockBinding& b : blocks) {
        for (int si = 0; si < nShapes; ++si) {
            const Shape& s = shapes[si];
            auto at = [&] { return Blk(b) + Fmt(" %ux%u", s.w, s.h); };
            auto refused = [&](const TileRequest& g) {
                TileRequest q{};
                ++n.refused;
                v.Check(!law.slot(b, g, s.w, s.h, q), [&] {
                    return at() + ": " + Req(g) + " lies outside, taken as slot " + Req(q);
                });
            };
            for (uint32_t m = 0; m < 8; ++m) {
                const int M = PyramidMipOf(FaceTexels(b.rung, m));
                if (!Carries(m, s)) {
                    // Not carried: no slot of it has a tile, and the tile it would hold is not its.
                    TileRequest g{};
                    ++n.refused;
                    v.Check(!law.global(b, {6, m, 0, 0}, s.w, s.h, g), [&] {
                        return at() + Fmt(": mip %u is not carried, yet slot (0,0) went to ", m) + Req(g);
                    });
                    refused({b.face, uint32_t(M), uint32_t(std::floor(Origin(b.bx, m) / s.w)),
                             uint32_t(std::floor(Origin(b.by, m) / s.h))});
                    continue;
                }
                const double tw = Side(m) / s.w, th = Side(m) / s.h;
                std::uniform_int_distribution<uint32_t> rx(0, uint32_t(tw) - 1), ry(0, uint32_t(th) - 1);
                for (int k = 0; k < 1004; ++k) {
                    const uint32_t x = k < 4 ? ((k & 1) ? uint32_t(tw) - 1 : 0u) : rx(rng);
                    const uint32_t y = k < 4 ? ((k & 2) ? uint32_t(th) - 1 : 0u) : ry(rng);
                    TileRequest want{}, g{}, back{};
                    const bool e = Expect(b, m, x, y, s, want);
                    const bool there = e && law.global(b, {6, m, x, y}, s.w, s.h, g);
                    const bool home = there && law.slot(b, g, s.w, s.h, back);
                    v.Check(home && Same(g, want) && Same(back, TileRequest{0, m, x, y}), [&] {
                        return at() + Fmt(": slot m%u (%u,%u) -> ", m, x, y) +
                               (there ? Req(g) : std::string("refused")) + " (the doubles: " +
                               (e ? Req(want) : std::string("not whole")) + ") -> " +
                               (home ? Req(back) : std::string("refused"));
                    });
                }
                n.slots += 1004;
                // The tiles just outside the block at this mip, where the face has such a tile.
                const double X0 = Origin(b.bx, m) / s.w, X1 = X0 + tw;
                const double Y0 = Origin(b.by, m) / s.h, Y1 = Y0 + th;
                const double FX = FaceTexels(b.rung, m) / s.w, FY = FaceTexels(b.rung, m) / s.h;
                const double xs[3] = {X0, std::floor((X0 + X1) / 2), X1 - 1};
                const double ys[3] = {Y0, std::floor((Y0 + Y1) / 2), Y1 - 1};
                std::vector<std::pair<double, double>> outside;
                for (const double y : ys) {
                    outside.push_back({X0 - 1, y});
                    outside.push_back({X1, y});
                }
                for (const double x : xs) {
                    outside.push_back({x, Y0 - 1});
                    outside.push_back({x, Y1});
                }
                for (const double x : {X0 - 1, X1}) {
                    for (const double y : {Y0 - 1, Y1}) outside.push_back({x, y});
                }
                for (const auto& [x, y] : outside) {
                    if (x < 0 || y < 0 || x >= FX || y >= FY) continue;   // past the face: no tile
                    refused({b.face, uint32_t(M), uint32_t(x), uint32_t(y)});
                }
                refused({(b.face + 1) % 6, uint32_t(M), uint32_t(X0), uint32_t(Y0)});   // not its face
            }
            // The pyramid's mips beside the slice's chain: one finer than its mip 0, one coarser
            // than its last -- each at the tile holding the block's origin corner.
            const int top = PyramidMipOf(FaceTexels(b.rung, 0));
            if (top >= 1) {
                refused({b.face, uint32_t(top - 1), uint32_t(2.0 * Origin(b.bx, 0) / s.w),
                         uint32_t(2.0 * Origin(b.by, 0) / s.h)});
            }
            const uint32_t past = CarriedMips(s);
            refused({b.face, uint32_t(PyramidMipOf(FaceTexels(b.rung, past))),
                     uint32_t(std::floor(Origin(b.bx, past) / s.w)),
                     uint32_t(std::floor(Origin(b.by, past) / s.h))});
        }
    }
    return v;
}

// ---- 2. the ground ---------------------------------------------------------------------------
struct GroundWorst {
    double dir = 0.0, window = 0.0, texel = 0.0;
    uint64_t corners = 0, texels = 0, res = 0;
};
void Worse(double& worst, double e) {
    if (!(e <= worst)) worst = e;   // a NaN is kept, not lost in a max
}
Verdict GateGround(const Law& law, const std::vector<BlockBinding>& blocks, GroundWorst& w) {
    Verdict v;
    for (const BlockBinding& b : blocks) {
        const double N = FaceTexels(b.rung, 0);
        for (const Shape& s : kShapes) {
            double u0, v0, u1, v1;
            law.ground(b, s.w, s.h, u0, v0, u1, v1);
            // (a) the corners: the box through ComposeCubeDir against the corner's texels in doubles,
            // and against step 3's address of the same rung anchored at the face's corner.
            for (int c = 0; c < 4; ++c) {
                const bool right = (c & 1) != 0, down = (c & 2) != 0;
                double dB[3], dE[3];
                ComposeCubeDir(b.face, right ? u1 : u0, down ? v1 : v0, dB);
                const double Xc = double(right ? b.bx + 1 : b.bx) * 16384.0;
                const double Yc = double(down ? b.by + 1 : b.by) * 16384.0;
                ComposeCubeDir(b.face, Xc / N, Yc / N, dE);
                double e = 0.0;
                for (int i = 0; i < 3; ++i) Worse(e, std::fabs(dB[i] - dE[i]));
                Worse(w.dir, e);
                v.Check(e <= 1e-12, [&] {
                    return Blk(b) + Fmt(": corner %d's direction off the texel corner's by %.3g", c, e);
                });
                double X = 0.0, Y = 0.0;
                FaceWindow{b.face, b.rung, 0, 0}.TexelOf(dB, X, Y);
                const double ew = (std::max)(std::fabs(X - Xc), std::fabs(Y - Yc));
                Worse(w.window, ew);
                v.Check(ew <= 1e-5, [&] {
                    return Blk(b) + Fmt(": corner %d at texel (%.6f, %.6f) of the rung, not (%.0f, %.0f)",
                                        c, X, Y, Xc, Yc);
                });
                ++w.corners;
            }
            // (b) the pyramid's own texel centres at the outer texels of every mip's corner slots,
            // against the centres the box gives half a texel of that mip inside its corners.
            const Lattice pyr = Lattice::Cube(16384u << 17, s.w, s.h);
            for (uint32_t m = 0; m < 8; ++m) {
                if (!Carries(m, s)) continue;
                const uint32_t tw = uint32_t(Side(m) / s.w), th = uint32_t(Side(m) / s.h);
                const double du = (u1 - u0) / Side(m), dv = (v1 - v0) / Side(m);
                for (int c = 0; c < 4; ++c) {
                    const bool right = (c & 1) != 0, down = (c & 2) != 0;
                    const TileRequest slot{6, m, right ? tw - 1 : 0u, down ? th - 1 : 0u};
                    TileRequest g{};
                    if (!v.Check(law.global(b, slot, s.w, s.h, g), [&] {
                            return Blk(b) + " refused its own corner slot " + Req(slot);
                        })) {
                        continue;
                    }
                    double lat = 0.0, lon = 0.0;
                    pyr.Texel(g, right ? s.w - 1 : 0u, down ? s.h - 1 : 0u, lat, lon);
                    double d[3];
                    ComposeCubeDir(b.face, right ? u1 - 0.5 * du : u0 + 0.5 * du,
                                   down ? v1 - 0.5 * dv : v0 + 0.5 * dv, d);
                    const double latB = std::asin((std::max)(-1.0, (std::min)(1.0, d[1])));
                    const double lonB = std::atan2(d[2], d[0]);
                    const double e = (std::max)(std::fabs(lat - latB),
                                                std::fabs(std::remainder(lon - lonB, 2.0 * kPi)));
                    Worse(w.texel, e);
                    v.Check(e <= 1e-12, [&] {
                        return Blk(b) + Fmt(" %ux%u m%u corner %d: the pyramid's texel off the box's by "
                                            "%.3g rad", s.w, s.h, m, c, e);
                    });
                    ++w.texels;
                }
            }
        }
        // (c) GroundRes, bitwise: the pyramid's at the doubles' mip, and kMercCirc / (4 N) there.
        for (uint32_t m = 0; m < 8; ++m) {
            const int M = PyramidMipOf(FaceTexels(b.rung, m));
            const double got = law.groundRes(b, m);
            const double pyr = M >= 0 ? Pyramid().GroundRes(uint32_t(M)) : -1.0;
            const double own = Lattice::kMercCirc / (4.0 * FaceTexels(b.rung, m));
            ++w.res;
            v.Check(M >= 0 && std::bit_cast<uint64_t>(got) == std::bit_cast<uint64_t>(pyr) &&
                        std::bit_cast<uint64_t>(got) == std::bit_cast<uint64_t>(own),
                    [&] {
                        return Blk(b) + Fmt(": GroundRes(%u) = %.17g, the pyramid's at mip %d %.17g, "
                                            "kMercCirc / 4N %.17g", m, got, M, pyr, own);
                    });
        }
    }
    return v;
}

// ---- 3. nesting ------------------------------------------------------------------------------
Verdict GateNesting(const Law& law, std::mt19937& rng, uint64_t& pairs) {
    Verdict v;
    const int ladder[2][2] = {{6, 3}, {9, 6}};   // {finer, the rung three above it}
    for (const auto& lr : ladder) {
        const int rf = lr[0], rc = lr[1];
        const uint32_t n = 1u << rf;
        std::uniform_int_distribution<uint32_t> any(0, n - 1);
        for (uint32_t f = 0; f < 6; ++f) {
            for (int c = 0; c < 6; ++c) {
                const BlockBinding fine =
                    c < 4 ? BlockBinding{f, rf, (c & 1) ? n - 1 : 0u, (c & 2) ? n - 1 : 0u}
                          : BlockBinding{f, rf, any(rng), any(rng)};
                // The coarser block over the same ground: the one whose uv box holds the fine
                // block's origin, found in doubles.
                const double k = std::ldexp(1.0, rc), fw = std::ldexp(1.0, -rf);
                const BlockBinding coarse{f, rc, uint32_t(std::floor(fine.bx * fw * k)),
                                          uint32_t(std::floor(fine.by * fw * k))};
                for (const Shape& s : kShapes) {
                    for (uint32_t m = 0; m + 3 < 8 && Carries(m + 3, s); ++m) {
                        const uint32_t tw = uint32_t(Side(m + 3) / s.w), th = uint32_t(Side(m + 3) / s.h);
                        for (uint32_t y = 0; y < th; ++y) {
                            for (uint32_t x = 0; x < tw; ++x) {
                                // The same texel of the face at both slices' mips (one face size).
                                const double gx = Origin(fine.bx, m + 3) + double(x) * s.w;
                                const double gy = Origin(fine.by, m + 3) + double(y) * s.h;
                                const double cx = (gx - Origin(coarse.bx, m)) / s.w;
                                const double cy = (gy - Origin(coarse.by, m)) / s.h;
                                TileRequest want{}, wantC{}, gF{}, gC{}, backF{}, backC{};
                                const bool e = Expect(fine, m + 3, x, y, s, want) &&
                                               Expect(coarse, m, cx, cy, s, wantC) && Same(want, wantC);
                                const bool okF = law.global(fine, {6, m + 3, x, y}, s.w, s.h, gF);
                                const bool okC = law.global(coarse, {7, m, uint32_t(cx), uint32_t(cy)},
                                                            s.w, s.h, gC);
                                const bool found = okF && okC && law.slot(fine, gC, s.w, s.h, backF) &&
                                                   law.slot(coarse, gF, s.w, s.h, backC);
                                ++pairs;
                                v.Check(e && found && Same(gF, gC) && Same(gF, want) &&
                                            Same(backF, TileRequest{0, m + 3, x, y}) &&
                                            Same(backC, TileRequest{0, m, uint32_t(cx), uint32_t(cy)}),
                                        [&] {
                                            return Fmt("%ux%u: ", s.w, s.h) + Blk(fine) +
                                                   Fmt(" m%u (%u,%u) -> ", m + 3, x, y) +
                                                   (okF ? Req(gF) : std::string("refused")) + ", " +
                                                   Blk(coarse) + Fmt(" m%u (%.0f,%.0f) -> ", m, cx, cy) +
                                                   (okC ? Req(gC) : std::string("refused")) +
                                                   "; the doubles " + (e ? Req(want) : std::string("disagree"));
                                        });
                            }
                        }
                    }
                }
            }
        }
    }
    return v;
}

// ---- 4. the Merrimack mouth ------------------------------------------------------------------
double Dot(const double a[3], const double b[3]) { return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]; }
void Cross(const double a[3], const double b[3], double o[3]) {
    o[0] = a[1] * b[2] - a[2] * b[1];
    o[1] = a[2] * b[0] - a[0] * b[2];
    o[2] = a[0] * b[1] - a[1] * b[0];
}
double Angle(const double a[3], const double b[3]) {
    double c[3];
    Cross(a, b, c);
    return std::atan2(std::sqrt(Dot(c, c)), Dot(a, b));
}
// The block's corner directions from the binding's ground, in order around it:
// (u0, v0) (u1, v0) (u1, v1) (u0, v1).
void Corners(const BlockBinding& b, double C[4][3]) {
    double u0, v0, u1, v1;
    b.Ground(u0, v0, u1, v1);
    ComposeCubeDir(b.face, u0, v0, C[0]);
    ComposeCubeDir(b.face, u1, v0, C[1]);
    ComposeCubeDir(b.face, u1, v1, C[2]);
    ComposeCubeDir(b.face, u0, v1, C[3]);
}
// The great-circle angle from unit P to the arc from A to B: to the foot of the perpendicular
// when it lies on the arc, else to the nearer end.
double AngleToArc(const double P[3], const double A[3], const double B[3], double foot[3]) {
    double m[3];
    Cross(A, B, m);
    const double l = std::sqrt(Dot(m, m));
    for (double& c : m) c /= l;
    const double pm = Dot(P, m);
    const double Q[3] = {P[0] - pm * m[0], P[1] - pm * m[1], P[2] - pm * m[2]};
    double aq[3], qb[3];
    Cross(A, Q, aq);
    Cross(Q, B, qb);
    if (Dot(aq, m) >= 0.0 && Dot(qb, m) >= 0.0 && Dot(Q, A) + Dot(Q, B) > 0.0) {
        for (int i = 0; i < 3; ++i) foot[i] = Q[i];
        return std::asin((std::min)(1.0, std::fabs(pm)));
    }
    const double a = Angle(P, A), b = Angle(P, B);
    for (int i = 0; i < 3; ++i) foot[i] = a <= b ? A[i] : B[i];
    return (std::min)(a, b);
}
// The great-circle angle from unit P to a block's ground: zero inside it, else the nearest of its
// four edges -- each a great circle, since an edge is a line of constant u or v on the face's
// plane, and so a plane through the centre.
double AngleToBlock(const double P[3], const BlockBinding& b) {
    double n[3], a[3], c[3];
    CubeFaceAxes(b.face, n, a, c);
    double u0, v0, u1, v1;
    b.Ground(u0, v0, u1, v1);
    const double pn = Dot(P, n);
    if (pn > 0.0) {
        const double u = Dot(P, a) / pn * 0.5 + 0.5, v = Dot(P, c) / pn * 0.5 + 0.5;
        if (u >= u0 && u <= u1 && v >= v0 && v <= v1) return 0.0;
    }
    double C[4][3], foot[3];
    Corners(b, C);
    double best = 1e300;
    for (int i = 0; i < 4; ++i) best = (std::min)(best, AngleToArc(P, C[i], C[(i + 1) % 4], foot));
    return best;
}
struct Needed {
    BlockBinding b;
    double m;
};
// The blocks of a rung that a viewer at P needs within `reach` metres: candidates from a dense
// sampling of the cap (129 rings, a point every reach / 128 of arc), widened by one block each way
// on the face each sample lands on, then each kept by its exact distance.
std::vector<Needed> Within(const double P[3], const double east[3], const double north[3],
                           int rung, double reach) {
    const double delta = reach / kSphereR;
    const uint32_t k = 1u << rung;
    std::set<std::tuple<uint32_t, uint32_t, uint32_t>> hit, cand;
    for (int i = 0; i <= 128; ++i) {
        const double rho = delta * i / 128.0;
        const int nb = (std::max)(8, int(2.0 * kPi * rho * kSphereR / (reach / 128.0)) + 1);
        for (int j = 0; j < nb; ++j) {
            const double th = 2.0 * kPi * j / nb;
            double d[3], uv[2];
            for (int c = 0; c < 3; ++c) {
                d[c] = std::cos(rho) * P[c] +
                       std::sin(rho) * (std::cos(th) * north[c] + std::sin(th) * east[c]);
            }
            const uint32_t f = CubeFaceOfDir(d, uv);
            hit.insert({f, (std::min)(k - 1, uint32_t(std::floor(uv[0] * k))),
                        (std::min)(k - 1, uint32_t(std::floor(uv[1] * k)))});
        }
    }
    for (const auto& [f, x, y] : hit) {
        for (int oy = -1; oy <= 1; ++oy) {
            for (int ox = -1; ox <= 1; ++ox) {
                const int cx = int(x) + ox, cy = int(y) + oy;
                if (cx >= 0 && cy >= 0 && cx < int(k) && cy < int(k)) cand.insert({f, uint32_t(cx), uint32_t(cy)});
            }
        }
    }
    std::vector<Needed> out;
    for (const auto& [f, x, y] : cand) {
        const BlockBinding b{f, rung, x, y};
        const double m = AngleToBlock(P, b) * kSphereR;
        if (m <= reach) out.push_back({b, m});
    }
    return out;
}
std::string SaidNeeded(const std::vector<Needed>& v) {
    std::map<uint32_t, std::vector<const Needed*>> byFace;
    for (const Needed& n : v) byFace[n.b.face].push_back(&n);
    std::string s = Fmt("%zu block%s", v.size(), v.size() == 1 ? "" : "s");
    const char* sep = ": ";
    for (const auto& [f, list] : byFace) {
        s += sep;
        sep = "; ";
        if (list.size() <= 4) {
            s += Fmt("f%u", f);
            for (size_t i = 0; i < list.size(); ++i) {
                s += Fmt("%s (%u,%u)", i ? "," : "", list[i]->b.bx, list[i]->b.by);
                if (list[i]->m > 0.0) s += Fmt(" at %.2f km", list[i]->m / 1000.0);
            }
        } else {
            uint32_t x0 = UINT32_MAX, x1 = 0, y0 = UINT32_MAX, y1 = 0;
            for (const Needed* n : list) {
                x0 = (std::min)(x0, n->b.bx);
                x1 = (std::max)(x1, n->b.bx);
                y0 = (std::min)(y0, n->b.by);
                y1 = (std::max)(y1, n->b.by);
            }
            s += Fmt("f%u %zu blocks, x %u..%u, y %u..%u", f, list.size(), x0, x1, y0, y1);
        }
    }
    return s;
}

// ---- 5. routing ------------------------------------------------------------------------------
// Which declared blocks hold pyramid tile T, and at which slot: T's texel square against each
// block's uv box, both in doubles, at the slice mip whose face is as many texels across as T's.
std::vector<TileRequest> Holders(const TileRequest& T, const std::vector<BlockSlice>& blocks,
                                 const Shape& s) {
    std::vector<TileRequest> out;
    const double FT = double(Pyramid().faceDim >> T.mip);
    for (const BlockSlice& k : blocks) {
        const BlockBinding& b = k.block;
        if (T.face != b.face) continue;
        int m = -1;
        for (uint32_t mm = 0; mm < 8; ++mm) {
            if (Carries(mm, s) && FaceTexels(b.rung, mm) == FT) m = int(mm);
        }
        if (m < 0) continue;
        const double w = std::ldexp(1.0, -b.rung);
        const double tu0 = T.x * double(s.w) / FT, tu1 = (T.x + 1.0) * s.w / FT;
        const double tv0 = T.y * double(s.h) / FT, tv1 = (T.y + 1.0) * s.h / FT;
        if (tu0 < b.bx * w || tu1 > (b.bx + 1.0) * w || tv0 < b.by * w || tv1 > (b.by + 1.0) * w) continue;
        out.push_back({k.slice, uint32_t(m), uint32_t((T.x * double(s.w) - Origin(b.bx, m)) / s.w),
                       uint32_t((T.y * double(s.h) - Origin(b.by, m)) / s.h)});
    }
    return out;
}
// The tiles a fake tree announces: at every pyramid mip from one finer than the finest slice's
// chain to one coarser than the coarsest's, the tiles at each block's four corners and just
// outside them, twenty at random inside each block, and the first block's corner tiles again on
// face 2.
std::vector<TileRequest> Announced(const std::vector<BlockSlice>& blocks, const Shape& s,
                                   std::mt19937& rng) {
    int lo = 99, hi = -1;
    for (const BlockSlice& k : blocks) {
        lo = (std::min)(lo, PyramidMipOf(FaceTexels(k.block.rung, 0)) - 1);
        hi = (std::max)(hi, PyramidMipOf(FaceTexels(k.block.rung, CarriedMips(s) - 1)) + 1);
    }
    std::set<std::tuple<uint32_t, uint32_t, uint32_t, uint32_t>> set;
    for (int M = (std::max)(lo, 0); M <= hi; ++M) {
        const double FT = double(Pyramid().faceDim >> M);
        const double TX = FT / s.w, TY = FT / s.h;   // the face's tiles a side at M
        if (TX < 1.0 || TY < 1.0) continue;
        for (size_t i = 0; i < blocks.size(); ++i) {
            const BlockBinding& b = blocks[i].block;
            const double span = FT / std::ldexp(1.0, b.rung);   // the block's texels a side at M
            const double x0 = std::floor(b.bx * span / s.w), x1 = std::floor(((b.bx + 1.0) * span - 1.0) / s.w);
            const double y0 = std::floor(b.by * span / s.h), y1 = std::floor(((b.by + 1.0) * span - 1.0) / s.h);
            auto add = [&](uint32_t f, double x, double y) {
                if (x >= 0 && y >= 0 && x < TX && y < TY) set.insert({f, uint32_t(M), uint32_t(x), uint32_t(y)});
            };
            for (const double x : {x0 - 1, x0, x1, x1 + 1}) {
                for (const double y : {y0 - 1, y0, y1, y1 + 1}) add(b.face, x, y);
            }
            std::uniform_int_distribution<uint32_t> rx{uint32_t(x0), uint32_t(x1)}, ry{uint32_t(y0), uint32_t(y1)};
            for (int k = 0; k < 20; ++k) add(b.face, rx(rng), ry(rng));
            if (i == 0) {
                for (const double x : {x0, x1}) {
                    for (const double y : {y0, y1}) add(2, x, y);
                }
            }
        }
    }
    std::vector<TileRequest> out;
    for (const auto& [f, m, x, y] : set) out.push_back({f, m, x, y});
    return out;
}
struct RouteCount {
    uint64_t tiles = 0, deliveries = 0, held[4] = {0, 0, 0, 0};
};
// Each announced tile reaches exactly the expected slots, each once.
Verdict GateRouting(const std::function<void(const std::string&, const TileRequest&)>& announce,
                    std::vector<TileRequest>& got, const std::string& tag,
                    const std::vector<TileRequest>& tiles, const std::vector<BlockSlice>& blocks,
                    const Shape& s, RouteCount& n) {
    Verdict v;
    for (const TileRequest& T : tiles) {
        const std::vector<TileRequest> want = Holders(T, blocks, s);
        got.clear();
        announce(tag, T);
        bool ok = got.size() == want.size();
        for (const TileRequest& w : want) {
            size_t seen = 0;
            for (const TileRequest& g : got) seen += Same(g, w) ? 1 : 0;
            ok = ok && seen == 1;
        }
        ++n.tiles;
        n.deliveries += want.size();
        ++n.held[(std::min)(want.size(), size_t(3))];
        v.Check(ok, [&] {
            std::string a, b;
            for (const TileRequest& w : want) a += " " + Req(w);
            for (const TileRequest& g : got) b += " " + Req(g);
            return Req(T) + Fmt(" should reach %zu:", want.size()) + a +
                   Fmt("; it reached %zu:", got.size()) + b;
        });
    }
    return v;
}

}  // namespace

bool RunTenantBindingSelfTest() {
    bool ok = true;
    std::mt19937 rng(0x4171u);
    const std::vector<BlockBinding> walked = Walked(rng);
    const std::string pyramidTag = Pyramid().Tag();
    Log("[tenant-binding] ---- HIERARCHY 4.17 commit 1: a slice bound to one aligned block of the "
        "pyramid, Lattice::Cube(16384 << 17) (tag %s) ----",
        pyramidTag.c_str());

    // ---- 1. the round trip
    TripCount trip;
    const Verdict v1 = GateRoundTrip(kBinding, walked, kShapes, 2, rng, trip);
    ok = ok && !v1.fails;
    Log("[tenant-binding] 1. round trip: %zu blocks (rungs 0 3 6 9 12 17, every face, its four "
        "corner blocks and two random) x 2 tile shapes (128x128: %u mips, 256x128: %u): %llu slots "
        "(four corners and 1000 random a mip) to the doubles' pyramid tile and back to themselves; "
        "%llu tiles just outside the block, on another face, beside the chain or at a mip the "
        "shape does not carry, refused -- %s",
        walked.size(), CarriedMips(kShapes[0]), CarriedMips(kShapes[1]),
        (unsigned long long)trip.slots, (unsigned long long)trip.refused, v1.Said().c_str());

    // ---- 2. the ground
    GroundWorst gw;
    const Verdict v2 = GateGround(kBinding, walked, gw);
    ok = ok && !v2.fails;
    Log("[tenant-binding] 2. ground: %llu corner directions from the uv box within %.1e of "
        "ComposeCubeDir at the texel corners in doubles (gate 1e-12), step 3's address "
        "(FaceWindow::TexelOf) puts them within %.1e texel of their texels; %llu texel centres of "
        "the pyramid's Lattice::Texel at every mip's corner slots within %.1e rad of the box's "
        "(gate 1e-12); GroundRes bitwise the pyramid's at %llu mips -- %s",
        (unsigned long long)gw.corners, gw.dir, gw.window, (unsigned long long)gw.texels, gw.texel,
        (unsigned long long)gw.res, v2.Said().c_str());

    // ---- 3. nesting
    uint64_t pairs = 0;
    const Verdict v3 = GateNesting(kBinding, rng, pairs);
    ok = ok && !v3.fails;
    Log("[tenant-binding] 3. nesting: %llu slot pairs (rung 6 at mip m + 3 against the rung-3 block "
        "over the same ground at m, and rung 9 against rung 6; every face, both shapes, every m "
        "both carry) name ONE pyramid tile, the doubles' one, and each slot is found back from the "
        "other's tile -- %s",
        (unsigned long long)pairs, v3.Said().c_str());

    // ---- 4. the Merrimack mouth
    Verdict v4;
    const double kDeg = kPi / 180.0;
    const double lat = 42.816 * kDeg, lon = -70.8125 * kDeg;
    const double P[3] = {std::cos(lat) * std::cos(lon), std::sin(lat), std::cos(lat) * std::sin(lon)};
    double uv[2];
    const uint32_t mouthFace = CubeFaceOfDir(P, uv);
    v4.Check(mouthFace == 5, [&] { return Fmt("the mouth is on face %u, not 5", mouthFace); });
    const double cl = std::sqrt(P[0] * P[0] + P[2] * P[2]);
    const double east[3] = {-P[2] / cl, 0.0, P[0] / cl};   // SpaceTest's TangentAt: +lon
    double north[3];
    Cross(east, P, north);                                  // east x up
    Log("[tenant-binding] 4. the Merrimack mouth, 42.816 N 70.8125 W: face %u, uv (%.9f, %.9f), "
        "the cube's texel (%.2f, %.2f); distances on the address block's sphere, %.0f km",
        mouthFace, uv[0], uv[1], uv[0] * 16384.0, uv[1] * 16384.0, kSphereR / 1000.0);
    BlockBinding mouth[3];
    const int mouthRungs[3] = {3, 6, 9};
    double nearest9 = 0.0, texels9 = 0.0, next9 = 0.0, nextTexels9 = 0.0;
    const char* nearestEdge9 = "";
    const char* nextEdge9 = "";
    for (int i = 0; i < 3; ++i) {
        const int r = mouthRungs[i];
        const double N = FaceTexels(r, 0), X = uv[0] * N, Y = uv[1] * N;
        const BlockBinding b{mouthFace, r, uint32_t(std::floor(X / 16384.0)), uint32_t(std::floor(Y / 16384.0))};
        mouth[i] = b;
        double bu0, bv0, bu1, bv1;
        b.Ground(bu0, bv0, bu1, bv1);
        v4.Check(b.Refusal(6).empty() && bu0 <= uv[0] && uv[0] < bu1 && bv0 <= uv[1] && uv[1] < bv1,
                 [&] { return Blk(b) + " does not hold the mouth"; });
        // The mouth's tile at the slice's mip 0, found back through the binding.
        const TileRequest t{mouthFace, uint32_t(PyramidMipOf(N)), uint32_t(std::floor(X / 128.0)),
                            uint32_t(std::floor(Y / 128.0))};
        TileRequest slot{};
        const bool found = b.Slot(t, 128, 128, slot);
        v4.Check(found && slot.mip == 0 && slot.x == uint32_t(std::floor((X - b.bx * 16384.0) / 128.0)) &&
                     slot.y == uint32_t(std::floor((Y - b.by * 16384.0) / 128.0)),
                 [&] { return Blk(b) + " did not find the mouth's tile " + Req(t) + " back"; });
        // Where in the block it stands: texels from each edge, and metres to each edge's nearest
        // point -- the foot on its great circle where that lies on the edge, else the nearer
        // corner -- with the bearing toward it.
        double C[4][3];
        Corners(b, C);
        const double tex[4] = {(uv[0] - bu0) * N, (bu1 - uv[0]) * N, (uv[1] - bv0) * N, (bv1 - uv[1]) * N};
        const int edge[4][2] = {{3, 0}, {1, 2}, {0, 1}, {2, 3}};   // u0, u1, v0, v1
        const char* names[4] = {"u0", "u1", "v0", "v1"};
        double metres[4], bearing[4];
        for (int e = 0; e < 4; ++e) {
            double foot[3];
            metres[e] = AngleToArc(P, C[edge[e][0]], C[edge[e][1]], foot) * kSphereR;
            const double fp = Dot(foot, P);
            const double t3[3] = {foot[0] - fp * P[0], foot[1] - fp * P[1], foot[2] - fp * P[2]};
            bearing[e] = std::fmod(std::atan2(Dot(t3, east), Dot(t3, north)) / kDeg + 360.0, 360.0);
        }
        Log("[tenant-binding] 4.   rung %d: block (%u,%u) of %u a side, the mouth in slot (%u,%u) "
            "of its mip 0; from the block's edges u0 %.2f, u1 %.2f, v0 %.2f, v1 %.2f texels, and "
            "to each edge's nearest point %.2f km (bearing %.0f), %.2f km (%.0f), %.2f km (%.0f), "
            "%.2f km (%.0f)",
            r, b.bx, b.by, 1u << r, slot.x, slot.y, tex[0], tex[1], tex[2], tex[3],
            metres[0] / 1000.0, bearing[0], metres[1] / 1000.0, bearing[1], metres[2] / 1000.0,
            bearing[2], metres[3] / 1000.0, bearing[3]);
        if (r == 9) {
            int order[4] = {0, 1, 2, 3};
            std::sort(order, order + 4, [&](int a, int c) { return metres[a] < metres[c]; });
            nearest9 = metres[order[0]];
            texels9 = tex[order[0]];
            nearestEdge9 = names[order[0]];
            next9 = metres[order[1]];
            nextTexels9 = tex[order[1]];
            nextEdge9 = names[order[1]];
        }
        std::string reach;
        for (const double km : {1.0, 8.0, 66.0}) {
            const std::vector<Needed> need = Within(P, east, north, r, km * 1000.0);
            bool own = false;
            for (const Needed& n : need) {
                own = own || (n.m == 0.0 && n.b.face == b.face && n.b.bx == b.bx && n.b.by == b.by);
            }
            v4.Check(own, [&] { return Blk(b) + Fmt(" is missing from its own %.0f km set", km); });
            reach += Fmt("%swithin %.0f km %s", reach.empty() ? "" : " | ", km, SaidNeeded(need).c_str());
        }
        Log("[tenant-binding] 4.   rung %d, what a viewer at the mouth needs: %s", r, reach.c_str());
    }
    Log("[tenant-binding] 4. the design guessed the mouth stands 2.3 km from its rung-9 block's edge: "
        "the nearest edge, %s, is %.2f km away (%.2f texels), the next, %s, %.2f km (%.2f texels)",
        nearestEdge9, nearest9 / 1000.0, texels9, nextEdge9, next9 / 1000.0, nextTexels9);
    ok = ok && !v4.fails;

    // ---- 5. dispatch and routing, through an Unregistered tenant per tile shape
    Verdict v5;
    std::string said5[2];
    std::vector<TileRequest> got;
    TileRequest lastCube{}, lastBlock{};
    uint32_t cubeCalls = 0, blockCalls = 0;
    const TileProviderFn cubeProvider = [&](const TileRequest& r, std::vector<uint8_t>&, TileLoc*) {
        ++cubeCalls;
        lastCube = r;
        return true;
    };
    const TileProviderFn blockProvider = [&](const TileRequest& r, std::vector<uint8_t>&, TileLoc*) {
        ++blockCalls;
        lastBlock = r;
        return true;
    };
    std::vector<BlockSlice> declared[2];
    std::vector<TileRequest> tiles[2];
    Tenant tenants[2];
    for (int si = 0; si < 2; ++si) {
        const Shape& s = kShapes[si];
        TenantDesc d;
        d.name = si == 0 ? L"tenant-binding test (the colour's tiles)" : L"tenant-binding test (the height's tiles)";
        d.astNode = "test.pages";
        d.fiber = {si == 0 ? DXGI_FORMAT_R8G8B8A8_UNORM_SRGB : DXGI_FORMAT_R16_FLOAT, s.w, s.h,
                   "the selftest's"};
        d.semantics = Semantics::Texture;
        d.residence = Residence::Streamable;
        d.absence = Absence::Unloaded;
        d.slices = 10;   // 0..5 the cube, 6..8 the mouth's blocks, 9 unbound
        d.bindings.push_back({0, 6, Lattice::Cube(Lattice::kFaceDim, s.w, s.h), cubeProvider,
                              "paint cube faces"});
        for (int i = 0; i < 3; ++i) {
            d.blocks.push_back({6u + uint32_t(i), mouth[i], blockProvider, "paint pyramid block"});
        }
        declared[si] = d.blocks;
        tenants[si] = Tenant::Unregistered(std::move(d), [&](const TileRequest& q) { got.push_back(q); });
        const Tenant& t = tenants[si];
        std::vector<uint8_t> out;
        TileLoc loc;
        uint64_t asked = 0;
        // The cube binding hands its faces on unchanged, as it always did.
        for (uint32_t f = 0; f < 6; ++f) {
            cubeCalls = 0;
            const TileRequest r{f, 3, 1, 2};
            const bool answered = t.Dispatch(r, out, &loc);
            v5.Check(answered && cubeCalls == 1 && Same(lastCube, r),
                     [&] { return "the cube binding handed on " + Req(lastCube) + " for " + Req(r); });
        }
        // Each block slice: every carried mip, its four corner slots and twenty at random.
        for (const BlockSlice& k : declared[si]) {
            for (uint32_t m = 0; m < CarriedMips(s); ++m) {
                const uint32_t tw = uint32_t(Side(m) / s.w), th = uint32_t(Side(m) / s.h);
                std::uniform_int_distribution<uint32_t> rx(0, tw - 1), ry(0, th - 1);
                for (int c = 0; c < 24; ++c) {
                    const uint32_t x = c < 4 ? ((c & 1) ? tw - 1 : 0u) : rx(rng);
                    const uint32_t y = c < 4 ? ((c & 2) ? th - 1 : 0u) : ry(rng);
                    TileRequest want{};
                    blockCalls = 0;
                    const bool e = Expect(k.block, m, x, y, s, want);
                    const bool answered = t.Dispatch({k.slice, m, x, y}, out, &loc);
                    ++asked;
                    v5.Check(e && answered && blockCalls == 1 && Same(lastBlock, want), [&] {
                        return Fmt("slice %u m%u (%u,%u) asked its provider for ", k.slice, m, x, y) +
                               Req(lastBlock) + ", the doubles' " + Req(want);
                    });
                }
                // Off the grid: no global tile, and the provider is not asked.
                blockCalls = 0;
                const bool off = t.Dispatch({k.slice, m, tw, 0}, out, &loc);
                v5.Check(!off && blockCalls == 0,
                         [&] { return Fmt("slice %u m%u (%u,0), off its grid, was answered", k.slice, m, tw); });
            }
            blockCalls = 0;
            const bool past = t.Dispatch({k.slice, CarriedMips(s), 0, 0}, out, &loc);
            v5.Check(!past && blockCalls == 0,
                     [&] { return Fmt("slice %u answered a mip past its chain", k.slice); });
        }
        cubeCalls = blockCalls = 0;
        const bool unbound = t.Dispatch({9, 0, 0, 0}, out, &loc);
        v5.Check(!unbound && cubeCalls == 0 && blockCalls == 0,
                 [&] { return std::string("the unbound slice 9 was answered"); });
        // THE ROUTING: a fake tree's onChanged, the hook Bind(tree) hangs, fired by hand.
        struct FakeTree {
            std::function<void(const std::string&, const TileRequest&)> onChanged;
        } tree;
        tree.onChanged = [&t](const std::string& tag, const TileRequest& r) { t.Changed(tag, r); };
        tiles[si] = Announced(declared[si], s, rng);
        RouteCount rc;
        const Verdict vr = GateRouting(tree.onChanged, got, pyramidTag, tiles[si], declared[si], s, rc);
        v5.checks += vr.checks;
        if (vr.fails && !v5.fails) v5.first = vr.first;
        v5.fails += vr.fails;
        // A change on the cube's own lattice takes the one law it always took.
        got.clear();
        tree.onChanged(Lattice::Cube(Lattice::kFaceDim, s.w, s.h).Tag(), {3, 2, 5, 7});
        v5.Check(got.size() == 1 && Same(got[0], TileRequest{3, 2, 5, 7}),
                 [&] { return std::string("a change on the cube's lattice left the old law"); });
        uint32_t mipLo = UINT32_MAX, mipHi = 0;
        for (const TileRequest& T : tiles[si]) {
            mipLo = (std::min)(mipLo, T.mip);
            mipHi = (std::max)(mipHi, T.mip);
        }
        said5[si] = Fmt("%ux%u: %llu slots of the three block slices asked their provider once, for "
                        "the doubles' pyramid tile; %llu pyramid tiles announced at mips %u..%u "
                        "made %llu deliveries -- %llu tiles held by no slice, %llu by one, %llu by "
                        "two, %llu by three",
                        s.w, s.h, (unsigned long long)asked, (unsigned long long)rc.tiles,
                        mipLo, mipHi, (unsigned long long)rc.deliveries,
                        (unsigned long long)rc.held[0], (unsigned long long)rc.held[1],
                        (unsigned long long)rc.held[2], (unsigned long long)rc.held[3]);
    }
    ok = ok && !v5.fails;
    Log("[tenant-binding] 5. dispatch and routing, slices 6 7 8 the mouth's blocks at rungs 3 6 9: "
        "%s; %s. The cube's faces handed on unchanged, slots off the grid and mips past the chain "
        "answered false unasked, the unbound slice false, a change on cube16k the old law -- %s",
        said5[0].c_str(), said5[1].c_str(), v5.Said().c_str());

    // ---- 6. the plants: each must be seen by the gate named for it
    {
        std::mt19937 prng(0x4171u);
        const std::vector<BlockBinding> pw = Walked(prng);
        TripCount pt;
        const Verdict a256 = GateRoundTrip(kPlantSquare, pw, &kShapes[1], 1, prng, pt);
        const Verdict a128 = GateRoundTrip(kPlantSquare, pw, &kShapes[0], 1, prng, pt);
        const bool caughtA = a256.fails > 0;
        ok = ok && caughtA && !a128.fails;
        Log("[tenant-binding] 6. PLANTED %s: gate 1 on 256x128 %s; on 128x128, where it is the same "
            "function, %s -- %s",
            kPlantSquare.name, a256.Said().c_str(), a128.Said().c_str(),
            caughtA ? "CAUGHT" : "NOT CAUGHT: the round trip cannot see a transposed tile count");

        GroundWorst pg;
        const Verdict b2 = GateGround(kPlantShifted, pw, pg);
        uint64_t pp = 0;
        const Verdict b3 = GateNesting(kPlantShifted, prng, pp);
        const bool caughtB = b2.fails > 0 && b3.fails > 0;
        ok = ok && caughtB;
        Log("[tenant-binding] 6. PLANTED %s: gate 2 %s; gate 3 %s -- %s", kPlantShifted.name,
            b2.Said().c_str(), b3.Said().c_str(),
            caughtB ? "CAUGHT by both" : "NOT CAUGHT by both: an origin error could pass");

        // The change law as a first-match lookup -- SliceOf's shape, carried over -- which stops
        // at the first block slice that holds the tile.
        const Tenant& t = tenants[0];
        const std::vector<BlockSlice>& blocks = t.Desc().blocks;
        auto firstOnly = [&](const std::string& tag, const TileRequest& r) {
            if (tag != pyramidTag) return;
            for (const BlockSlice& k : blocks) {
                TileRequest q{};
                if (!k.block.Slot(r, kShapes[0].w, kShapes[0].h, q)) continue;
                q.face = k.slice;
                got.push_back(q);
                return;   // the plant
            }
        };
        RouteCount prc;
        const Verdict c5 = GateRouting(firstOnly, got, pyramidTag, tiles[0], declared[0], kShapes[0], prc);
        const bool caughtC = c5.fails > 0;
        ok = ok && caughtC;
        Log("[tenant-binding] 6. PLANTED a change law that stops at the first slice holding the tile: "
            "gate 5 %s -- %s",
            c5.Said().c_str(), caughtC ? "CAUGHT" : "NOT CAUGHT: a shared tile would reach one slice");
    }

    // ---- the refusals: the binding's own words, and the declaration's
    {
        Verdict vr;
        const struct {
            BlockBinding b;
            uint32_t slice;
        } bad[6] = {{{5, -1, 0, 0}, 6}, {{5, 18, 0, 0}, 6}, {{6, 3, 0, 0}, 6},
                    {{5, 3, 8, 0}, 6},  {{5, 3, 0, 8}, 6},  {{5, 3, 7, 7}, 5}};
        std::string words;
        for (const auto& c : bad) {
            const std::string why = c.b.Refusal(c.slice);
            vr.Check(!why.empty(), [&] { return Blk(c.b) + " at a slice was not refused"; });
            words += (words.empty() ? "" : "; ") + why;
        }
        for (const BlockBinding& good : {BlockBinding{0, 0, 0, 0}, BlockBinding{5, 17, 131071, 131071}}) {
            vr.Check(good.Refusal(6).empty(), [&] { return Blk(good) + " was refused: " + good.Refusal(6); });
        }
        // The declaration refuses a malformed block slice before anything is built.
        auto declare = [&](const std::vector<BlockSlice>& blocks, uint32_t slices) {
            TenantDesc d;
            d.name = L"tenant-binding refusal";
            d.fiber = {DXGI_FORMAT_R8G8B8A8_UNORM, 128, 128, ""};
            d.slices = slices;
            d.bindings.push_back({0, 6, Lattice::Cube(Lattice::kFaceDim), cubeProvider, ""});
            d.bindings.push_back({6, 1, Lattice::Window(1263360, 1538048, 14), cubeProvider, ""});
            d.blocks = blocks;
            try {
                Tenant::Unregistered(std::move(d), [](const TileRequest&) {});
            } catch (const std::exception& e) {
                return std::string(e.what());
            }
            return std::string();
        };
        const BlockSlice ok7{7, {5, 9, 166, 4}, blockProvider, ""};
        const std::string threw[4] = {
            declare({BlockSlice{7, {5, 18, 0, 0}, blockProvider, ""}}, 8),   // the binding's refusal
            declare({BlockSlice{8, {5, 9, 166, 4}, blockProvider, ""}}, 8),  // past the slices
            declare({ok7, ok7}, 8),                                          // bound twice
            declare({BlockSlice{6, {5, 9, 166, 4}, blockProvider, ""}}, 8)}; // the page's slice
        std::string declWords;
        for (const std::string& w : threw) {
            vr.Check(!w.empty(), [&] { return std::string("a malformed block slice was declared"); });
            declWords += (declWords.empty() ? "" : "; ") + w;
        }
        vr.Check(threw[0].find(BlockBinding{5, 18, 0, 0}.Refusal(7)) != std::string::npos,
                 [&] { return "the declaration did not say the binding's reason: " + threw[0]; });
        ok = ok && !vr.fails;
        Log("[tenant-binding] refusals: %s", words.c_str());
        Log("[tenant-binding] refusals, declared: %s -- %s", declWords.c_str(), vr.Said().c_str());
    }

    if (ok) {
        Log("[tenant-binding] ---- PASS: the block binding's round trip at every mip in both tile "
            "shapes, its ground against the pyramid's own lattice, the nesting the phase law stands "
            "on, the dispatch and the change routing through the tenant, and every plant caught ----");
    } else {
        Log("[tenant-binding] ---- FAIL ----");
    }
    return ok;
}

}  // namespace ga::hal
