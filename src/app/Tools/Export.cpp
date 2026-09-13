// Export - --export.
// Moved verbatim from main.cpp (M12 step 1b); declared in app/Tools.h.
#include "app/Tools.h"

#include "compose/Compositor.h"
#include "core/Common.h"
#include "core/Gpu.h"
#include "core/Image.h"
#include "core/Residency.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

namespace ga::app::tools {

namespace {

// ================================================================================================
// M6j --export: the manager as a DATA INTERFACE, not just a renderer's feeder. A composed
// channel is pulled tile by tile through the exact provider path the renderer streams
// (cache-first: a warmed machine exports offline) and written as:
//   .png  color RGBA, or height normalized to grayscale
//   .raw  height as row-major float32 (plus dims in the log)
//   .obj  height triangulated as a mesh in local metres -- the same tiles the mesh-shader
//         surface consumes, hand-inspectable in any DCC tool
// ================================================================================================
static int RunChannelExport(const std::string& spec, const std::wstring& outPath,
                            Compositor& comp, int colCh, int hgtCh) {
    std::string name = spec;
    uint32_t mip = 2;
    if (const size_t c = spec.find(':'); c != std::string::npos) {
        name = spec.substr(0, c);
        mip = static_cast<uint32_t>(atoi(spec.c_str() + c + 1));
    }
    TileProviderFn fn;
    bool height = false;
    uint32_t tileW = 128, tileH = 128, face = 0;
    if (name == "earth.color.window" && colCh >= 0) {
        fn = comp.WindowColor(colCh, 1263360, 1538048, 16384, 14);
    } else if (name == "earth.color.inlet" && colCh >= 0) {
        // M6l: a z19 export-only realization (~22 cm ground at this latitude) centred on the
        // MassGIS ortho coverage -- deep enough to JUDGE the 15 cm aerial layer's painting.
        fn = comp.WindowColor(colCh, 40699567, 49405858, 16384, 19);
    } else if (name == "earth.height.window" && hgtCh >= 0) {
        fn = comp.WindowHeight(hgtCh, 1263360, 1538048, 16384, 14);
        height = true;
        tileW = 256;
    } else if (name.rfind("earth.color.cube.f", 0) == 0 && colCh >= 0) {
        face = static_cast<uint32_t>(name.back() - '0') % 6;
        fn = comp.CubeColor(colCh);
    } else if ((name.rfind("earth.height.cube.f", 0) == 0 ||
                name.rfind("mars.height.cube.f", 0) == 0) &&
               hgtCh >= 0) {
        face = static_cast<uint32_t>(name.back() - '0') % 6;
        fn = comp.CubeHeight(hgtCh);
        height = true;
        tileW = 256;
    } else {
        Log("[export] unknown channel '%s' (or its stack is not configured). Channels: "
            "earth.color.window, earth.color.inlet (z19), earth.height.window, "
            "earth.color.cube.f0..5, earth|mars.height.cube.f0..5",
            name.c_str());
        return 1;
    }
    const uint32_t dim = Compositor::kFaceDim >> mip;
    if (dim > 4096) {
        Log("[export] mip %u is %ux%u -- use mip >= 2 (<= 4096 wide)", mip, dim, dim);
        return 1;
    }
    const uint32_t tilesX = dim / tileW, tilesY = dim / tileH;
    Log("[export] %s mip %u: %ux%u texels, %u tiles (cache-first through the manager)",
        name.c_str(), mip, dim, dim, tilesX * tilesY);

    std::vector<float> hgtData;
    std::vector<uint8_t> rgba;
    if (height) hgtData.resize(static_cast<size_t>(dim) * dim);
    else rgba.resize(static_cast<size_t>(dim) * dim * 4);
    std::vector<uint8_t> tile;
    for (uint32_t ty = 0; ty < tilesY; ++ty) {
        for (uint32_t tx = 0; tx < tilesX; ++tx) {
            if (!fn({face, mip, tx, ty}, tile, nullptr) || tile.size() != 65536) continue;
            for (uint32_t py = 0; py < tileH; ++py) {
                const size_t row = static_cast<size_t>(ty) * tileH + py;
                if (height) {
                    const uint16_t* src = reinterpret_cast<const uint16_t*>(tile.data());
                    for (uint32_t px = 0; px < tileW; ++px) {
                        hgtData[row * dim + tx * tileW + px] =
                            HalfToFloat(src[py * tileW + px]);
                    }
                } else {
                    memcpy(&rgba[(row * dim + tx * tileW) * 4], &tile[py * tileW * 4],
                           static_cast<size_t>(tileW) * 4);
                }
            }
        }
    }

    const std::wstring ext =
        outPath.size() > 4 ? outPath.substr(outPath.size() - 4) : std::wstring();
    if (ext == L".obj") {
        if (!height) {
            Log("[export] .obj export needs a HEIGHT channel");
            return 1;
        }
        if (dim > 1024) {
            Log("[export] .obj at %u^2 would be %u M verts -- use mip >= 4", dim,
                dim * dim / 1000000);
            return 1;
        }
        const double worldPx = static_cast<double>((1ll << 14) * 256ll >> mip);
        const double ground = 40075016.686 / worldPx * std::cos(42.8 * 3.14159265 / 180.0);
        FILE* f = nullptr;
        _wfopen_s(&f, outPath.c_str(), L"w");
        if (!f) return 1;
        fprintf(f, "# GAGAME composed-channel export: %s mip %u (%u^2, %.2f m/px)\n",
                name.c_str(), mip, dim, ground);
        for (uint32_t y = 0; y < dim; ++y) {
            for (uint32_t x = 0; x < dim; ++x) {
                fprintf(f, "v %.2f %.2f %.2f\n", (static_cast<double>(x) - dim / 2.0) * ground,
                        hgtData[static_cast<size_t>(y) * dim + x],
                        (dim / 2.0 - static_cast<double>(y)) * ground);
            }
        }
        for (uint32_t y = 0; y + 1 < dim; ++y) {
            for (uint32_t x = 0; x + 1 < dim; ++x) {
                const uint32_t a = y * dim + x + 1, b = a + 1, c = a + dim, d = c + 1;
                fprintf(f, "f %u %u %u\nf %u %u %u\n", a, b, c, b, d, c);
            }
        }
        fclose(f);
        Log("[export] wrote %S (%u verts, %u tris)", outPath.c_str(), dim * dim,
            (dim - 1) * (dim - 1) * 2);
    } else if (ext == L".raw") {
        if (!height) {
            Log("[export] .raw export is for height channels; color goes to .png");
            return 1;
        }
        FILE* f = nullptr;
        _wfopen_s(&f, outPath.c_str(), L"wb");
        if (!f) return 1;
        fwrite(hgtData.data(), 4, hgtData.size(), f);
        fclose(f);
        Log("[export] wrote %S (float32 row-major, %ux%u, row 0 north)", outPath.c_str(), dim,
            dim);
    } else {
        if (height) {
            float lo = 1e9f, hi = -1e9f;
            for (const float v : hgtData) {
                lo = (std::min)(lo, v);
                hi = (std::max)(hi, v);
            }
            rgba.resize(static_cast<size_t>(dim) * dim * 4);
            const float inv = hi > lo ? 255.0f / (hi - lo) : 0.0f;
            for (size_t i = 0; i < hgtData.size(); ++i) {
                const uint8_t g = static_cast<uint8_t>((hgtData[i] - lo) * inv);
                rgba[i * 4] = rgba[i * 4 + 1] = rgba[i * 4 + 2] = g;
                rgba[i * 4 + 3] = 255;
            }
            Log("[export] height range %.1f .. %.1f m (normalized to grayscale)", lo, hi);
        }
        SavePng(outPath, rgba.data(), dim, dim, dim * 4, rgba.size());
        Log("[export] wrote %S", outPath.c_str());
    }
    return 0;
}

}  // namespace

int RunExport(const Options& opt, Gpu& gpu, Compositor& compositor, int hgtCh,
              ResidencyManager& resMgr, int colCh) {
    const int rc =
        RunChannelExport(opt.exportSpec, opt.exportOut, compositor, colCh, hgtCh);
    gpu.WaitIdle();
    resMgr.Shutdown();
    return rc;
}

}  // namespace ga::app::tools
