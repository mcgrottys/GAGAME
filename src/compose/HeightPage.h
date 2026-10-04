// ================================================================================================
//  HeightPage - the bed the water kernels read, evaluated on the CPU for a hull (the water match,
//  step 3).
//
//  THE KERNELS READ THE HEIGHT TENANT'S WINDOWS (HeightPages.hlsli HpHeightChain): the eye's windows
//  of the one pyramid, each ring at its own grain. A window's texels ARE the pyramid's, painted at
//  their centres with the level's ground resolution and stored through the one quantization, so the
//  twin is the pyramid's lattice at the rung the finest ring reads, by direction, anywhere. So a hull's
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
//  Texels are memoised (the bed does not change under a hull); single-threaded, like TreeWater's memo.
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

    // PHASE B1 (out/integration/plan_phase_b.md): THE TWIN ON THE PYRAMID. The kernels read the
    // windows of the one pyramid (HeightPages.hlsli HpHeightChain), and a window's texels ARE the
    // pyramid's (HIERARCHY 4.1: slice i at the global texel modulo 16384), so the twin needs no
    // window and no eye: the pyramid's lattice at `rung` (16384 2^rung texels a face, in the height's
    // 256 x 128 tiles), each texel the stack at its centre with that rung's ground, through the one
    // quantization -- a function of the place alone, at the grain the rings a hull stands in read
    // (the rung whose texel the finest ring's is: WaterBankLayer's base texel).
    HeightPage(const Compositor* comp, int channel, uint32_t rung)
        : m_comp(comp), m_ch(channel), m_cube(Lattice::Cube(Lattice::kFaceDim << 17, 256, 128)),
          m_rung(int(rung)) {}
    int Rung() const { return m_rung; }

    bool Read(double latDeg, double lonDeg, double& value) const override {
        if (!m_comp || m_ch < 0) return false;
        if (m_memo.size() > kMaxHeld) m_memo.clear();
        constexpr double kD2R = 3.14159265358979 / 180.0;
        // The pyramid at its rung, by direction (HpCubeFace's CPU twin).
        // The cube face by direction (HpCubeFace's CPU twin).
        const double la = latDeg * kD2R, lo = lonDeg * kD2R;
        const double cl = std::cos(la);
        const double d[3] = {cl * std::cos(lo), std::sin(la), cl * std::sin(lo)};
        double uv[2] = {0.0, 0.0};
        const uint32_t face = CubeFaceOfDir(d, uv);
        value = BilinearAt(face, m_cube, 17u - uint32_t(m_rung), uv[0], uv[1]);
        return true;
    }

    size_t Held() const { return m_memo.size(); }

private:
    // PageLoad4's law in doubles at a lattice's mip (the pyramid's rungs are its mips 17 - rung):
    // centres at -0.5, four taps clamped.
    double BilinearAt(uint32_t face, const Lattice& lat, uint32_t mip, double u, double v) const {
        const uint32_t dim = lat.faceDim >> mip;
        const double tx = u * dim - 0.5, ty = v * dim - 0.5;
        const double fx0 = std::floor(tx), fy0 = std::floor(ty);
        const double frx = tx - fx0, fry = ty - fy0;
        const double dmax = double(dim) - 1.0;
        const auto clampI = [&](double t) {
            return static_cast<uint32_t>((std::min)((std::max)(t, 0.0), dmax));
        };
        const uint32_t x0 = clampI(fx0), x1 = clampI(fx0 + 1.0);
        const uint32_t y0 = clampI(fy0), y1 = clampI(fy0 + 1.0);
        return (double(Texel(face, lat, x0, y0, mip)) * (1.0 - frx) +
                double(Texel(face, lat, x1, y0, mip)) * frx) * (1.0 - fry) +
               (double(Texel(face, lat, x0, y1, mip)) * (1.0 - frx) +
                double(Texel(face, lat, x1, y1, mip)) * frx) * fry;
    }

    float Texel(uint32_t face, const Lattice& lat, uint32_t ix, uint32_t iy, uint32_t mip = 0) const {
        // (face 3 bits, x and y 30 bits each: a rung-15 face is 2^29 texels a side)
        const uint64_t key = (uint64_t(face) << 60) | (uint64_t(ix) << 30) | uint64_t(iy);
        const auto it = m_memo.find(key);
        if (it != m_memo.end()) return it->second;
        // The painter's address at the level: the tile holding the texel and its pixel inside.
        const TileRequest r{face, mip, ix / lat.texW, iy / lat.texH};
        double latR = 0.0, lonR = 0.0;
        lat.Texel(r, ix % lat.texW, iy % lat.texH, latR, lonR);
        const float h = m_comp->SampleHeightStack(m_ch, latR, lonR, lat.GroundRes(mip));
        const float q = HalfToFloat(tree_detail::F2H(h));   // the one quantization: the GPU's
        m_memo.emplace(key, q);
        return q;
    }

    const Compositor* m_comp = nullptr;
    int m_ch = -1;
    Lattice m_cube;    // the pyramid
    int m_rung = 0;    // the rung the twin reads
    mutable std::unordered_map<uint64_t, float> m_memo;
};

}  // namespace ga
