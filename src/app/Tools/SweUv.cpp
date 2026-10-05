// SweUv - --swe-uv.
// Moved verbatim from main.cpp (M12 step 1b); declared in app/Tools.h.
#include "app/Tools.h"

#include "core/Common.h"
#include "hal/Gpu.h"
#include "core/Image.h"
#include "sim/BathyModel.h"
#include "sim/SweSolver.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <string>
#include <vector>

namespace ga::app::tools {

namespace {

// M5c debug: the solved surface-current field as a picture. u east = red, u west = blue,
// v north = green tint, brightness = speed; land/invalid = dark grey. The fastest way to SEE
// whether the throat jet, the eddies, and the boundary plumbing are doing physics or nonsense.
void DumpSweUv(Gpu& gpu, SweSolver& swe, const SweToolGrid& grid, const std::wstring& path) {
    const uint32_t nx = swe.Nx(), ny = swe.Ny();
    std::vector<float> pts(static_cast<size_t>(nx) * ny * 2);
    // One giant batch is wasteful; sample the texture directly through ReadProbes' machinery
    // instead: probe every texel via a single readback by asking for row-major world points.
    for (uint32_t y = 0; y < ny; ++y) {
        for (uint32_t x = 0; x < nx; ++x) {
            const size_t i = (static_cast<size_t>(y) * nx + x) * 2;
            pts[i + 0] = x + 0.5f;   // the cell's centre (ReadProbes reads cells)
            pts[i + 1] = y + 0.5f;
        }
    }
    std::vector<SweSolver::Probe> pr(static_cast<size_t>(nx) * ny);
    swe.ReadProbes(gpu, pts.data(), static_cast<int>(nx * ny), pr.data());

    std::vector<uint8_t> rgba(static_cast<size_t>(nx) * ny * 4);
    for (size_t i = 0; i < pr.size(); ++i) {
        uint8_t* px = &rgba[i * 4];
        if (!pr[i].valid) {
            px[0] = px[1] = px[2] = 46;
        } else {
            const float u = pr[i].u, v = pr[i].v;
            const float s = std::sqrt(u * u + v * v);
            const float b = std::min(1.0f, s / 1.5f);   // full brightness at 1.5 m/s
            const float uf = std::clamp(u / std::max(s, 1e-4f), -1.0f, 1.0f);
            const float vf = std::clamp(v / std::max(s, 1e-4f), -1.0f, 1.0f);
            px[0] = static_cast<uint8_t>(40 + 215 * b * std::max(0.0f, uf));
            px[2] = static_cast<uint8_t>(40 + 215 * b * std::max(0.0f, -uf));
            px[1] = static_cast<uint8_t>(40 + 130 * b * std::abs(vf));
        }
        px[3] = 255;
    }
    SavePng(path, rgba.data(), nx, ny, nx * 4, rgba.size());
    Log("[swe] uv field dumped to %S", path.c_str());

    // Companion picture: the deviation field itself. Red = above the tide plane, blue = below
    // (full at 25 cm); land grey. At max ebb the west strip must glow red and slope away east.
    for (size_t i = 0; i < pr.size(); ++i) {
        uint8_t* px = &rgba[i * 4];
        const float e = pr[i].dEta;
        const float b = std::min(1.0f, std::abs(e) / 0.25f);
        px[0] = static_cast<uint8_t>(40 + (e > 0 ? 215 * b : 0));
        px[1] = 40;
        px[2] = static_cast<uint8_t>(40 + (e < 0 ? 215 * b : 0));
        px[3] = 255;
    }
    SavePng(path + L".eta.png", rgba.data(), nx, ny, nx * 4, rgba.size());

    // Third picture: raw flux magnitude (log scale). Distinguishes "flux kernel never ran here"
    // (exact zero, black) from "ran but weak" (dim) at a glance.
    uint32_t fw = 0, fh = 0, fpitch = 0;
    const std::vector<uint8_t> flux = swe.ReadFluxRaw(gpu, &fw, &fh, &fpitch);
    for (uint32_t y = 0; y < ny; ++y) {
        for (uint32_t x = 0; x < nx; ++x) {
            const float* f = reinterpret_cast<const float*>(&flux[y * fpitch + x * 8]);
            const float mag = std::abs(f[0]) + std::abs(f[1]);
            uint8_t* px = &rgba[(static_cast<size_t>(y) * nx + x) * 4];
            const float v = (mag > 0) ? std::min(1.0f, 0.18f * std::log2(1.0f + mag)) : 0.0f;
            px[0] = static_cast<uint8_t>(20 + 235 * v);
            px[1] = static_cast<uint8_t>(20 + 90 * v);
            px[2] = (mag == 0.0f) ? 20 : 60;
            px[3] = 255;
        }
    }
    SavePng(path + L".flux.png", rgba.data(), nx, ny, nx * 4, rgba.size());

    // M6r continuity audit: NET eastward transport through full N-S sections (sum of raw east
    // face fluxes down a column, m^3/s -- land and NULL faces are exact zeros). Continuity says
    // consecutive sections differ only by the storage filling between them; a jump reveals a
    // leak, a flat profile with a weak gap says the demand never concentrated.
    for (const float wx : {-12000.0f, -5000.0f, -2500.0f, 0.0f, 350.0f, 900.0f, 1600.0f}) {
        double sx = -1.0, sy = 0.0;
        grid.CellOfFlat(wx, 0.0, sx, sy);
        const int ix = static_cast<int>(std::floor(sx));
        if (ix < 0 || ix >= static_cast<int>(nx)) continue;
        double q = 0.0;
        for (uint32_t y = 0; y < ny; ++y) {
            q += reinterpret_cast<const float*>(&flux[y * fpitch + ix * 8])[0];
        }
        Log("[swe] section x=%+6.0f m: net east transport %+8.1f m^3/s", wx, q);
    }
}

}  // namespace

void RunSweUv(const Options& opt, Gpu& gpu, const SweToolGrid& grid, SweSolver& swe) {
    DumpSweUv(gpu, swe, grid, opt.sweUvDump);
}

}  // namespace ga::app::tools
