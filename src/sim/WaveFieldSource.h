// ================================================================================================
//  WaveFieldSource -- M9bc: THE SOLVED WAVE FIELD AS A TREE NODE.
//
//  The wave field is a stationary boundary-value solve over a window with a global phase gauge
//  (ALGEBRA `wavefield`): it cannot be asked one texel at a time, so it is the first TILE-NATIVE
//  node -- it paints whole tiles of a frame's addresses from a solve cached per identity, and the
//  tree treats the result exactly like any per-texel paint (same formats, identity, fold, refetch).
//  The same hook is what a stencil operator (grad, div, curl over a neighbourhood) needs.
//
//  Planes are the frame's FACES: face p of the z16 window frame is component p's (a, k, cos phi,
//  sin phi) -- the Cl(2)+ even part with its scalars, quantized as the solver always quantized it
//  -- and face nUsed is the envelope. The solver's grid IS the page's grid: cell = the z16 texel in
//  world metres at the window's latitude, origin on a texel corner, nx/ny tile multiples, so a
//  tile is exact bytes of the solve, never a resample. Every mip is answered: mip m is the 2^m box
//  mean of the solve (componentwise on the spinor -- the cl2 law), which is what the fold would
//  have produced, so parents and children agree by construction and the whole pyramid can be
//  PREFILLED into the disk cache the moment a solve lands -- the residency chain then reads files
//  and never waits on a paint: no waves popping in.
//
//  Identity = solver version + the bucket key the field was solved under (level, current, SWE
//  current signature, spectrum, height stack) + the page frame. A bucket roll re-keys the tree.
// ================================================================================================
#pragma once

#include "compose/Compositor.h"
#include "compose/DomainSource.h"
#include "core/GaUnits.h"
#include "core/Lattice.h"
#include "hal/Residency.h"
#include "core/Space.h"
#include "sim/WaveField.h"

#include <atomic>
#include <cmath>
#include <cstdio>
#include <memory>
#include <string>
#include <vector>

namespace ga {

class WaveFieldSource : public DomainSource {
public:
    static constexpr int kZoom = 16;
    static constexpr uint32_t kTile = 128;         // Raw4 tiles: 128 x 128 texels, 64 KB
    static constexpr uint32_t kMarginTiles = 4;    // frame origin sits this far NW of the window
    static constexpr double kWorldPx = double(1ll << kZoom) * 256.0;
    static constexpr double kMercCircM = 40075016.686;

    struct Frame {
        long long orgPxX = 0, orgPxY = 0;   // the ColorFrame::Window origin (z16 px)
        long long winPxX = 0, winPxY = 0;   // the window's NW texel (z16 px)
        uint32_t nx = 0, ny = 0;            // the window in texels (= solver cells)
        ColorFrame color;                   // the frame the tree paints in
    };

    // Align the solver's grid to the z16 page. cell = the page texel in WORLD metres at the
    // window's centre latitude (Mercator is conformal: x and y spacing agree to 0.01% over the
    // window); origin = the SW corner of a page texel; nx, ny rounded up to tile multiples.
    // PHASE C4: the request is metres about the scene's anchor (`chart`, exact); its corners'
    // places snap to the page lattice, and the grid's cells ARE that lattice's texels.
    static Frame Align(WaveFieldConfig& cfg, const Space::Anchor& chart) {
        constexpr double kD2R = 3.14159265358979 / 180.0;
        const double sizeX = cfg.nx * cfg.cellM, sizeZ = cfg.ny * cfg.cellM;
        double latC = 0.0, lonC = 0.0, lonW = 0.0, latN = 0.0;
        chart.LatLonOf(cfg.orgX + 0.5 * sizeX, cfg.orgZ + 0.5 * sizeZ, latC, lonC);
        chart.LatLonOf(cfg.orgX, cfg.orgZ + sizeZ, latN, lonW);
        const double cell = kMercCircM / kWorldPx * std::cos(latC * kD2R);
        // M12 step 4b: the page lattice's own closed form (Lattice::PxOf on a z16 window).
        // MercX / MercY were its text: a 44.6 M-input sweep (the window's span, a global
        // grid, a random global set, the shipped window's lonW / latN) found the doubles
        // bit-identical, so the origin is the lattice's and the two statics are gone.
        double mx = 0.0, my = 0.0;
        Lattice::Window(0, 0, kZoom, kTile, kTile).PxOf(latN, lonW, mx, my);
        const long long X0 = static_cast<long long>(std::floor(mx));
        const long long Y0 = static_cast<long long>(std::floor(my));
        cfg.nx = static_cast<int>(std::ceil(sizeX / cell / kTile) * kTile);
        cfg.ny = static_cast<int>(std::ceil(sizeZ / cell / kTile) * kTile);
        cfg.cellM = cell;
        // The SW corner of the window in world metres, from the page texel lattice.
        const double lon0 = double(X0) / kWorldPx * 360.0 - 180.0;
        const double latS = LatOfMercY(double(Y0 + cfg.ny));
        chart.FlatOf(latS, lon0, cfg.orgX, cfg.orgZ);   // the SW corner, about the anchor
        cfg.pxX0 = X0;
        cfg.pxY0 = Y0;
        Frame f;
        f.winPxX = X0;
        f.winPxY = Y0;
        f.nx = uint32_t(cfg.nx);
        f.ny = uint32_t(cfg.ny);
        f.orgPxX = X0 - static_cast<long long>(kMarginTiles * kTile);
        f.orgPxY = Y0 - static_cast<long long>(kMarginTiles * kTile);
        f.color = ColorFrame::Window(f.orgPxX, f.orgPxY, kZoom, kTile, kTile);
        return f;
    }
    // The inverse (page row -> latitude); Lattice has no counterpart, so it stays.
    static double LatOfMercY(double y) {
        const double n = 3.14159265358979 * (1.0 - 2.0 * y / kWorldPx);
        return (2.0 * std::atan(std::exp(n)) - 3.14159265358979 * 0.5) * 180.0 / 3.14159265358979;
    }

    WaveFieldSource(const WaveField* wf, const Frame& frame)
        : m_wf(wf), m_frame(frame), m_unit(UnitSpec::Parse("fraction")) {}

    // The bucket key this node stands for. The tree's identity folds it; main re-keys the tree
    // when the solver's live key moves.
    void SetKey(uint64_t k) { m_key.store(k); }
    uint64_t Key() const { return m_key.load(); }
    const Frame& PageFrame() const { return m_frame; }
    // The window's uv box inside the frame (for the residency wants).
    void WindowUv(float& u0, float& v0, float& u1, float& v1) const {
        u0 = float(double(m_frame.winPxX - m_frame.orgPxX) / 16384.0);
        v0 = float(double(m_frame.winPxY - m_frame.orgPxY) / 16384.0);
        u1 = float(double(m_frame.winPxX - m_frame.orgPxX + m_frame.nx) / 16384.0);
        v1 = float(double(m_frame.winPxY - m_frame.orgPxY + m_frame.ny) / 16384.0);
    }
    // The same window as whole tiles of mip 0, closed -- the box TileTree::Prefill takes. The
    // far bound is the tile that STARTS on the window's far edge: the uv box above, read as a
    // closed box, has always put that tile (void: none of the window is in it) in the prefill,
    // and a prefill that left it out would leave a different set of files.
    void WindowTiles(uint32_t& x0, uint32_t& y0, uint32_t& x1, uint32_t& y1) const {
        x0 = uint32_t((m_frame.winPxX - m_frame.orgPxX) / kTile);
        y0 = uint32_t((m_frame.winPxY - m_frame.orgPxY) / kTile);
        x1 = uint32_t((m_frame.winPxX - m_frame.orgPxX + m_frame.nx) / kTile);
        y1 = uint32_t((m_frame.winPxY - m_frame.orgPxY + m_frame.ny) / kTile);
    }

    // ---- DomainSource
    const char* Name() const override { return "wave.field"; }
    SourceDomain Domain() const override { return SourceDomain::Raster; }
    uint8_t GradeSig() const override { return kG0 | kG2; }   // scalars + the phase spinor
    uint32_t Channels() const override { return 4; }
    const UnitSpec& Unit() const override { return m_unit; }
    const char* NodeKind() const override { return "load"; }
    const char* Cadence() const override { return "on wave bucket"; }
    bool TileNative() const override { return true; }
    bool Footprint(double& lon0, double& lat0, double& lon1, double& lat1) const override {
        lon0 = double(m_frame.winPxX) / kWorldPx * 360.0 - 180.0;
        lon1 = double(m_frame.winPxX + m_frame.nx) / kWorldPx * 360.0 - 180.0;
        lat1 = LatOfMercY(double(m_frame.winPxY));
        lat0 = LatOfMercY(double(m_frame.winPxY + m_frame.ny));
        return true;
    }
    std::string Identity() const override {
        char b[160];
        snprintf(b, sizeof(b), "wave.field|solver v%u|key %016llx|z%d win %lld,%lld %ux%u|",
                 WaveField::kSolverVersion, static_cast<unsigned long long>(m_key.load()), kZoom,
                 m_frame.winPxX, m_frame.winPxY, m_frame.nx, m_frame.ny);
        return b;
    }
    bool SampleAt(const DomainQuery&, DomainValue& out) const override {
        out.weight = 0.0f;   // regional: tiles only
        return false;
    }

    // The tile: plane r.face, any mip. Answers false (Transient) until the solver's live field
    // carries this node's key -- the tree never caches a tile of the wrong identity.
    bool PaintTile(const ColorFrame& frame, const TileRequest& r, uint32_t texW, uint32_t texH,
                   std::vector<DomainValue>& vals) const override {
        // The solve of THIS node's key: next (its pages are being painted) or live.
        std::shared_ptr<const WaveField::Solved> s = m_wf ? m_wf->SolvedFor(m_key.load()) : nullptr;
        if (!s) return false;
        const WaveField::GpuTable& t = s->table;
        const uint32_t nx = t.nx, ny = t.ny;
        if (nx == 0 || ny == 0 || r.face > t.nUsed) return false;
        const uint32_t plane = r.face;
        const size_t aw = size_t(nx) * 2;
        const size_t sx0 = size_t(plane & 1u) * nx, sy0 = size_t(plane >> 1u) * ny;
        const long long step = 1ll << r.mip;
        const long long X = (frame.orgPxX >> r.mip) + static_cast<long long>(r.x) * texW;
        const long long Y = (frame.orgPxY >> r.mip) + static_cast<long long>(r.y) * texH;
        vals.assign(size_t(texW) * texH, DomainValue{});
        for (uint32_t py = 0; py < texH; ++py) {
            for (uint32_t px = 0; px < texW; ++px) {
                // The block of solve cells this texel covers at this mip (mip 0: one cell).
                const long long cx0 = ((X + px) << r.mip) - m_frame.winPxX;
                const long long cy0 = ((Y + py) << r.mip) - m_frame.winPxY;
                double acc[4] = {0, 0, 0, 0};
                uint32_t n = 0;
                for (long long by = 0; by < step; ++by) {
                    const long long row = cy0 + by;
                    if (row < 0 || row >= ny) continue;
                    const size_t j = size_t(ny - 1 - row);   // solver row 0 = SOUTH
                    for (long long bx = 0; bx < step; ++bx) {
                        const long long i = cx0 + bx;
                        if (i < 0 || i >= nx) continue;
                        const uint8_t* p = &s->atlas[((sy0 + j) * aw + sx0 + size_t(i)) * 4];
                        for (int q = 0; q < 4; ++q) acc[q] += p[q];
                        ++n;
                    }
                }
                if (!n) continue;
                DomainValue& v = vals[size_t(py) * texW + px];
                for (int q = 0; q < 4; ++q) v.c[q] = float(acc[q] / (255.0 * n));
                v.weight = float(double(n) / double(step * step));
            }
        }
        return true;
    }

private:
    const WaveField* m_wf = nullptr;
    Frame m_frame;
    UnitSpec m_unit;
    std::atomic<uint64_t> m_key{0};
};

}  // namespace ga
