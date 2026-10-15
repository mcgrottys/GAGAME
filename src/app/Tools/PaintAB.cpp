// ================================================================================================
//  --tool paint-ab: A SOURCE'S TWO PAINTS, BYTE FOR BYTE (ColorSource::PaintTile).
//
//  A source that paints a tile at a time must paint the per-texel loop's bytes exactly: the same
//  RGBA, the same alpha, the same `complete` / `anyCover` / `fullCover`. This tool paints the same
//  addresses both ways through Compositor::PaintSourceTile (s_tilePaint off, then on) and compares:
//  the tile holding the scene's place and its eight neighbours, at every level of the cube the
//  colour tree reads (cube16k, mips 0..7) and of the eye windows' deep cube (2^31 a face, mips
//  6..18), and of those only the levels the tree paints (none above a source's own). It times both. Run it with the scene's own sources, cached tiles only: a tile the
//  provider would fetch answers "not whole" in both paths alike.
// ================================================================================================
#include "app/Tools.h"

#include "compose/Compositor.h"
#include "core/Common.h"
#include "core/Lattice.h"

#include <chrono>
#include <cmath>
#include <vector>

namespace ga::app::tools {

int RunPaintAB(ColorSource& src, double anchorLatDeg, double anchorLonDeg) {
    constexpr double kD2R = 3.14159265358979323846 / 180.0;
    const double la = anchorLatDeg * kD2R, lo = anchorLonDeg * kD2R;
    const double d[3] = {std::cos(la) * std::cos(lo), std::sin(la), std::cos(la) * std::sin(lo)};
    double uv[2];
    const uint32_t face = CubeFaceOfDir(d, uv);
    struct Frame {
        Lattice lat;
        uint32_t mip0, mip1;
        const char* name;
    };
    const Frame frames[] = {{Lattice::Cube(16384), 0, 7, "cube16k"}, {Lattice::Cube(1u << 31), 6, 18, "cube2^31"}};
    uint32_t tiles = 0, same = 0, flagsSame = 0, notWhole = 0;
    double usTexel = 0.0, usTile = 0.0;
    std::string first;
    const bool was = Compositor::s_tilePaint;
    for (const Frame& f : frames) {
        // what the tree paints: a source with a level of its own is folded above it, never painted
        const int own = src.OwnMip(f.lat);
        for (uint32_t mip = f.mip0; mip <= f.mip1; ++mip) {
            if (own >= 0 && int(mip) > own) continue;
            const double dim = double(f.lat.faceDim >> mip);
            const long long cx = (long long)(uv[0] * dim) / f.lat.texW, cy = (long long)(uv[1] * dim) / f.lat.texH;
            const long long nTiles = (long long)(dim / f.lat.texW);
            for (long long dy = -1; dy <= 1; ++dy) {
                for (long long dx = -1; dx <= 1; ++dx) {
                    const long long x = cx + dx, y = cy + dy;
                    if (x < 0 || y < 0 || x >= nTiles || y >= nTiles) continue;
                    const TileRequest r{face, mip, uint32_t(x), uint32_t(y)};
                    Compositor::TileBox box{};
                    f.lat.Box(r, box);
                    std::vector<uint8_t> a, b;
                    bool ca = false, aa = false, fa = false, cb = false, ab = false, fb = false;
                    Compositor::PaintSourceTile(&src, f.lat, r, box, a, ca, aa, fa);   // warm: the provider's tiles held
                    Compositor::s_tilePaint = false;
                    auto t0 = std::chrono::steady_clock::now();
                    Compositor::PaintSourceTile(&src, f.lat, r, box, a, ca, aa, fa);
                    auto t1 = std::chrono::steady_clock::now();
                    Compositor::s_tilePaint = true;
                    Compositor::PaintSourceTile(&src, f.lat, r, box, b, cb, ab, fb);
                    auto t2 = std::chrono::steady_clock::now();
                    usTexel += std::chrono::duration<double, std::micro>(t1 - t0).count();
                    usTile += std::chrono::duration<double, std::micro>(t2 - t1).count();
                    ++tiles;
                    notWhole += ca ? 0 : 1;
                    const bool eq = a == b;
                    same += eq ? 1 : 0;
                    flagsSame += (ca == cb && aa == ab && fa == fb) ? 1 : 0;
                    if ((!eq || ca != cb || aa != ab || fa != fb) && first.empty()) {
                        size_t at = 0;
                        while (at < a.size() && at < b.size() && a[at] == b[at]) ++at;
                        char m[200];
                        snprintf(m, sizeof m, "%s f%u m%u (%lld,%lld): first byte apart %zu, flags %d%d%d vs %d%d%d",
                                 f.name, face, mip, x, y, at, ca, aa, fa, cb, ab, fb);
                        first = m;
                    }
                }
            }
        }
    }
    Compositor::s_tilePaint = was;
    const bool ok = tiles > 0 && same == tiles && flagsSame == tiles;
    Log("[paint-ab] %s: %u tiles about %.5f N %.5f E (%u not whole, alike in both) -- bytes equal %u, flags equal %u; "
        "texel by texel %.2f ms a tile, a tile at a time %.2f ms a tile (%.1fx)%s%s -- %s",
        src.Info().name.c_str(), tiles, anchorLatDeg, anchorLonDeg, notWhole, same, flagsSame,
        usTexel / 1000.0 / (std::max)(1u, tiles), usTile / 1000.0 / (std::max)(1u, tiles),
        usTile > 0.0 ? usTexel / usTile : 0.0, first.empty() ? "" : "; first apart: ", first.c_str(),
        ok ? "PASS" : "FAIL");
    return ok ? 0 : 1;
}

}  // namespace ga::app::tools
