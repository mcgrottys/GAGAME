// ================================================================================================
//  ExposurePage - the swell shadow's page texels, evaluated on the CPU for a hull (the water match,
//  step 3).
//
//  THE KERNEL READS THE EYE'S WINDOWS: the exposure tenant's windows of the one pyramid at rung 3 (rank
//  1's mip 0), bilinear over texel centres, floored at kSwellShadowFloor. Its texels are the node's own answers:
//  TileTree paints texel (i, j) of mip m as ExposureSource::SampleAt at the lattice's texel centre with
//  groundM = GroundRes(m), and the Half root stores it through the one quantization (tree_detail::F2H).
//  So the CPU does not need the GPU's copy of a texel to hold the same number -- it can ask the node at
//  the same centre with the same footprint and quantize it the same way, and the two are one value.
//
//  WHY NOT READ THE GPU'S TEXELS BACK. It was tried (a region readback of the page, delivered through
//  the frame ring): the numbers agreed, but a copy carries RESIDENCY, and residency is I/O timing. At
//  start-up the only resident mip under the helm was 6, reading 0.0 until mip 3 landed a timing-
//  dependent number of frames later, and the helm_boat recipe stopped being identical run to run. A
//  hull's physics must be a function of the scene and the clock, never of when a tile arrived. The
//  truth is the finest level the kernel ever reads (the floor, mip 3); the GPU converges to it as that
//  level lands, and what it draws before then is a streaming transient the probe can see.
//
//  COST: one texel is one five-ray march over the height stack at 26.7 m steps -- measured 0.04-0.06 ms
//  at the Merrimack (--water-probe's exposure column). A hull touches four texels and crosses into new
//  ones every ~56 m; texels are memoised per (direction, level) bucket, dropped when the bucket rolls.
//
//  Absence: no valid swell direction is no field (the node refuses to march toward a default, and the
//  page reads nothing resident). A texel the node could not answer is
//  the Half compose's zero, exactly as the page stores it. Single-threaded, like TreeWater's memo.
// ================================================================================================
#pragma once

#include "compose/ExposureSource.h"
#include "compose/TileTree.h"
#include "core/Common.h"
#include "core/Lattice.h"
#include "sim/PlaceField.h"

#include <cmath>
#include <cstdint>
#include <unordered_map>

namespace ga {

class ExposurePage : public PlaceField {
public:
    static constexpr size_t kMaxHeld = 4096;   // texels memoised before the memo starts over

    // PHASE B1 (out/integration/plan_phase_b.md): THE TWIN ON THE PYRAMID, as HeightPage's: the
    // exposure's windows are the pyramid's texels, so the twin is the pyramid's lattice at `rung` (the
    // floor the bank reads the exposure at: 76 m, rung 3, rank 1's mip 0), by direction, anywhere.
    ExposurePage(const ExposureSource* src, uint32_t rung)
        : m_src(src), m_page(Lattice::Cube(Lattice::kFaceDim << 17, 256, 128)),
          m_mip(17u - rung) {}

    bool Read(double latDeg, double lonDeg, double& value) const override {
        if (!m_src || !m_src->Valid()) return false;
        // The pyramid's face by direction.
        constexpr double kD2R = 3.14159265358979 / 180.0;
        const double la = latDeg * kD2R, lo = lonDeg * kD2R;
        const double cl = std::cos(la);
        const double d[3] = {cl * std::cos(lo), std::sin(la), cl * std::sin(lo)};
        double uv[2] = {0.0, 0.0};
        m_slice = CubeFaceOfDir(d, uv);
        const double u = uv[0], v = uv[1];
        const uint64_t params = m_src->Params();
        if (params != m_params || m_memo.size() > kMaxHeld) {
            m_memo.clear();
            m_params = params;
        }
        // PageLoad4's texel law, in doubles: centres at -0.5, four taps clamped to the page.
        const uint32_t dim = m_page.faceDim >> m_mip;
        const double tx = u * dim - 0.5, ty = v * dim - 0.5;
        const double fx0 = std::floor(tx), fy0 = std::floor(ty);
        const double frx = tx - fx0, fry = ty - fy0;
        const double dmax = double(dim) - 1.0;
        const auto clampI = [&](double t) {
            return static_cast<uint32_t>((std::min)((std::max)(t, 0.0), dmax));
        };
        const uint32_t x0 = clampI(fx0), x1 = clampI(fx0 + 1.0);
        const uint32_t y0 = clampI(fy0), y1 = clampI(fy0 + 1.0);
        value = (double(Texel(x0, y0)) * (1.0 - frx) + double(Texel(x1, y0)) * frx) * (1.0 - fry) +
                (double(Texel(x0, y1)) * (1.0 - frx) + double(Texel(x1, y1)) * frx) * fry;
        return true;
    }

    uint64_t Evaluations() const { return m_evaluations; }
    size_t Held() const { return m_memo.size(); }
    uint32_t Mip() const { return m_mip; }

private:
    float Texel(uint32_t ix, uint32_t iy) const {
        // (the pyramid's face in the top 3 bits; x and y 30 bits each)
        const uint64_t key = (uint64_t(m_slice) << 60) | (uint64_t(ix) << 30) | uint64_t(iy);
        const auto it = m_memo.find(key);
        if (it != m_memo.end()) return it->second;
        // The painter's own address: the tile (texW x texH) holding the texel, and its pixel inside.
        const TileRequest r{m_slice, m_mip, ix / m_page.texW, iy / m_page.texH};
        double latR = 0.0, lonR = 0.0;
        m_page.Texel(r, ix % m_page.texW, iy % m_page.texH, latR, lonR);
        constexpr double kR2D = 180.0 / 3.14159265358979;
        DomainQuery q;
        q.lat = latR * kR2D;
        q.lon = lonR * kR2D;
        q.groundM = m_page.GroundRes(m_mip);
        DomainValue dv;
        float val = 0.0f;   // an unanswered texel: the Half compose leaves it zero
        if (m_src->SampleAt(q, dv) && dv.weight > 0.0f) {
            val = HalfToFloat(tree_detail::F2H(dv.c[0]));   // the one quantization: the GPU's
        }
        ++m_evaluations;
        m_memo.emplace(key, val);
        return val;
    }

    const ExposureSource* m_src = nullptr;
    Lattice m_page;
    mutable uint32_t m_slice = 0;   // the face of the last read
    uint32_t m_mip = 0;
    mutable uint64_t m_params = ~0ull;
    mutable std::unordered_map<uint64_t, float> m_memo;
    mutable uint64_t m_evaluations = 0;
};

}  // namespace ga
