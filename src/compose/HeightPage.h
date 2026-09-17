// ================================================================================================
//  HeightPage - the bed the water kernels read, evaluated on the CPU for a hull (the water match,
//  step 3).
//
//  THE KERNELS READ THE HEIGHT TENANT (HeightPages.hlsli HpHeightAt): the z14 page by containment where
//  its texel is at least as fine as the cube's, else the cube face -- bilinear over texel centres at the
//  resident level, held to the consumer's floor. The bank's floor is now its ring's own grain, so the
//  rings a hull and an eye stand in read the page's finest level (mip 0: 9.55 m of Mercator, ~7 m at the
//  Merrimack). That level's texels are the height stack painted at their centres with the level's
//  ground resolution and stored through the one quantization; --trace step 11 holds a GPU texel against
//  exactly this CPU evaluation (Compositor::SampleHeightStack at the centre, GroundRes). So a hull's
//  depth laws -- the dry weight, shoaling, the wave-current gain's phase speed, the breaking cap -- can
//  stand on the bed the drawn sea stands on, without reading a single texel back.
//
//  WHY THE FINEST LEVEL, NOT THE RESIDENT ONE: residency is I/O timing, and a hull's physics must be a
//  function of the scene and the clock (compose/ExposurePage says how that was measured). The GPU
//  converges to this level as it lands.
//
//  WHAT IT REPLACES under a hull: the stack sampled at 1 m at the centre of an 8 m memo cell -- a bed
//  constant over each cell and stepping between them, which the bank never read. Beside the north
//  jetty that disagreement put the kernel's water at a dry weight of 0.14 and the hull's at 1.0, and the
//  drawn sea stood 0.41 m rms off the hull's.
//
//  Outside the page: the cube face at its finest level (611 m), as the kernels fall back. Texels are
//  memoised (the bed does not change under a hull); single-threaded, like TreeWater's memo.
// ================================================================================================
#pragma once

#include "compose/Compositor.h"
#include "compose/TileTree.h"
#include "core/Common.h"
#include "core/Lattice.h"
#include "sim/PlaceField.h"

#include <cmath>
#include <cstdint>
#include <unordered_map>

namespace ga {

class HeightPage : public PlaceField {
public:
    static constexpr size_t kMaxHeld = 1u << 16;   // texels memoised before the memo starts over

    // `page` the Mercator window the tenant's page slice sits on, `cube` its cube faces' lattice.
    HeightPage(const Compositor* comp, int channel, const Lattice& page, const Lattice& cube)
        : m_comp(comp), m_ch(channel), m_page(page), m_cube(cube) {}

    bool Read(double latDeg, double lonDeg, double& value) const override {
        if (!m_comp || m_ch < 0) return false;
        if (m_memo.size() > kMaxHeld) m_memo.clear();
        constexpr double kD2R = 3.14159265358979 / 180.0;
        // The page by containment (the kernels' test: strictly inside).
        if (m_page.kind == Lattice::Kind::Window) {
            double px = 0.0, py = 0.0;
            m_page.PxOf(latDeg, lonDeg, px, py);
            const double u = (px - double(m_page.orgPxX)) / double(m_page.faceDim);
            const double v = (py - double(m_page.orgPxY)) / double(m_page.faceDim);
            if (u > 0.0 && u < 1.0 && v > 0.0 && v < 1.0) {
                value = Bilinear(kPageFace, m_page, u, v);
                return true;
            }
        }
        // The cube face by direction (HpCubeFace's CPU twin).
        const double la = latDeg * kD2R, lo = lonDeg * kD2R;
        const double cl = std::cos(la);
        const double d[3] = {cl * std::cos(lo), std::sin(la), cl * std::sin(lo)};
        double uv[2] = {0.0, 0.0};
        const uint32_t face = CubeFaceOfDir(d, uv);
        value = Bilinear(face, m_cube, uv[0], uv[1]);
        return true;
    }

    size_t Held() const { return m_memo.size(); }

private:
    static constexpr uint32_t kPageFace = 7u;   // a memo key's face for the page, past the cube's 0..5

    // PageLoad4's law at the finest level, in doubles: centres at -0.5, four taps clamped.
    double Bilinear(uint32_t face, const Lattice& lat, double u, double v) const {
        const uint32_t dim = lat.faceDim;
        const double tx = u * dim - 0.5, ty = v * dim - 0.5;
        const double fx0 = std::floor(tx), fy0 = std::floor(ty);
        const double frx = tx - fx0, fry = ty - fy0;
        const double dmax = double(dim) - 1.0;
        const auto clampI = [&](double t) {
            return static_cast<uint32_t>((std::min)((std::max)(t, 0.0), dmax));
        };
        const uint32_t x0 = clampI(fx0), x1 = clampI(fx0 + 1.0);
        const uint32_t y0 = clampI(fy0), y1 = clampI(fy0 + 1.0);
        return (double(Texel(face, lat, x0, y0)) * (1.0 - frx) +
                double(Texel(face, lat, x1, y0)) * frx) * (1.0 - fry) +
               (double(Texel(face, lat, x0, y1)) * (1.0 - frx) +
                double(Texel(face, lat, x1, y1)) * frx) * fry;
    }

    float Texel(uint32_t face, const Lattice& lat, uint32_t ix, uint32_t iy) const {
        const uint64_t key = (uint64_t(face) << 56) | (uint64_t(ix) << 28) | uint64_t(iy);
        const auto it = m_memo.find(key);
        if (it != m_memo.end()) return it->second;
        // The painter's address at level 0: the tile holding the texel and its pixel inside.
        const TileRequest r{face == kPageFace ? 0u : face, 0u, ix / lat.texW, iy / lat.texH};
        double latR = 0.0, lonR = 0.0;
        lat.Texel(r, ix % lat.texW, iy % lat.texH, latR, lonR);
        const float h = m_comp->SampleHeightStack(m_ch, latR, lonR, lat.GroundRes(0));
        const float q = HalfToFloat(tree_detail::F2H(h));   // the one quantization: the GPU's
        m_memo.emplace(key, q);
        return q;
    }

    const Compositor* m_comp = nullptr;
    int m_ch = -1;
    Lattice m_page, m_cube;
    mutable std::unordered_map<uint64_t, float> m_memo;
};

}  // namespace ga
