// ================================================================================================
//  Lattice.h - M12 step 2b: THE LATTICE, said once. Was Compositor.h's ColorFrame (M9aj).
//
//  A raster is a plane (ATLAS.md section 3): a frame, a metric, a LATTICE and a fiber. This is
//  the lattice -- the sample grid a realization sits on and the projection that pinned it flat:
//  the quad-sphere cube of `faceDim` texels per face, or a Web-Mercator window of `texW x texH`
//  tiles from an origin pixel at zoom `zBase`. It is the whole of what a realization's
//  addressing means: which ground a tile (face, mip, x, y) covers, what a texel's centre is in
//  the WGS84 exchange frame, what a source is asked at, and the cache folder name (Tag) a
//  tile's content is a pure function of.
//
//  WHY IT MOVED. ColorFrame was threaded as a parameter through every TileTree call and passed
//  unchanged to children, while five page providers and three kernels re-spelled the same
//  window origin by hand. The name was wrong too: height, mask, exposure and wave trees all
//  sit on this. The tree now INHERITS its lattice from its parent unless a node declares its
//  own (step 2d, the same fold TileTree already applies to its tile format), and the tenants
//  declare theirs per slice (step 3e). Nothing moved in the numbers: GroundRes, Box, Texel and
//  Tag are the M9aj bodies verbatim, and Tag() -- THE CACHE IDENTITY -- reads no new member,
//  ever. The additions (Crs, VNorth, Addr/Request, SameGround, Rows, PxOf, Ladder) are the
//  declarations a frame carried nowhere: what the AST, the page table and the kernel rows used
//  to restate from literals.
//
//  Gates: composetest (cube/window addressing vs closed forms, unchanged) and spacetest block 5
//  (GroundRes bitwise against the two old expressions, Tag strings, Addr/Request round trip).
// ================================================================================================
#pragma once

#include "core/GeoRef.h"
#include "core/PageTable.h"
#include "core/TileAddress.h"

#include <cstdint>
#include <string>

namespace ga {

// The quad-sphere's face direction (D3D cubemap convention): face, u, v in [0,1] -> unit vector
// in the planet frame. Defined in Lattice.cpp; pinned by composetest.
void ComposeCubeDir(uint32_t face, double u, double v, double out[3]);

// One paint tile's angular footprint (radians) + per-texel span, for the soak rule.
struct TileBox {
    double latMin, latMax, lonMin, lonMax;
    double texLat, texLon;
};

struct Lattice {
    static constexpr uint32_t kFaceDim = 16384;   // every composed pyramid realization today
    static constexpr double kMercCirc = 40075016.686;   // Web-Mercator world metres (equator)

    enum class Kind : uint8_t { Cube, Window };
    Kind kind = Kind::Cube;
    uint32_t texW = 128, texH = 128;   // texels per 64 KB tile (RGBA8 128x128; R16F 256x128)
    uint32_t faceDim = kFaceDim;       // Cube: texels per face. Window: the page's texels
    long long orgPxX = 0, orgPxY = 0;  // Window: origin in zBase Mercator pixels
    int zBase = 14;

    static Lattice Cube(uint32_t faceDim, uint32_t texW = 128, uint32_t texH = 128) {
        Lattice f;
        f.kind = Kind::Cube;
        f.faceDim = faceDim;
        f.texW = texW;
        f.texH = texH;
        return f;
    }
    static Lattice Window(long long orgPxX, long long orgPxY, int zBase, uint32_t texW = 128,
                          uint32_t texH = 128) {
        Lattice f;
        f.kind = Kind::Window;
        f.orgPxX = orgPxX;
        f.orgPxY = orgPxY;
        f.zBase = zBase;
        f.texW = texW;
        f.texH = texH;
        return f;
    }

    // ---- the M9aj bodies, verbatim (Lattice.cpp) ----------------------------------------------
    // What a source is ASKED at. The cube's finest is bounded by the face dimension; a window's
    // is its zoom base, which is how a realization demands detail the cube can never demand.
    double GroundRes(uint32_t mip) const;
    // The tile's angular box + per-texel span -- the soak rule's input.
    void Box(const TileRequest& r, TileBox& box) const;
    // One texel's centre, in the WGS84 exchange frame every source answers in.
    void Texel(const TileRequest& r, uint32_t px, uint32_t py, double& latRad,
               double& lonRad) const;
    // The realization's cache folder name -- "cube16k", "window_z14_1263360_1538048".
    std::string Tag(const char* kindName = "window") const;

    // ---- the declarations a frame carried nowhere ---------------------------------------------
    // The projection family this lattice is flat in (Projections.h for the exact forms).
    CrsKind Crs() const { return kind == Kind::Cube ? CrsKind::Geographic : CrsKind::WebMercator; }
    // Does the second axis grow northward? Mercator rows grow SOUTH (GaAst.cpp's mercator.px).
    bool VNorth() const { return kind == Kind::Cube; }
    // The coarsest mip a tree on this lattice carries: faceDim halved down to one tile
    // (TileTree::MaxMip, moved).
    uint32_t MaxMip() const {
        uint32_t m = 0;
        for (uint32_t d = faceDim; d > texW; d >>= 1) ++m;
        return m;
    }
    // The bridge to the page table's address space (PageTable.h): the same tile, named as a
    // page. Levels count down in resolution there; mips count down here -- the same number.
    PageAddr Addr(const TileRequest& r) const { return PageAddr{r.mip, r.x, r.y}; }
    TileRequest Request(uint32_t face, const PageAddr& a) const {
        return TileRequest{face, a.level, a.x, a.y};
    }
    // The ladder this lattice IS: mip 0's ground resolution, halving per level.
    LevelLadder Ladder() const { return LevelLadder{GroundRes(0), faceDim}; }
    // Two lattices name the same ground iff they are the same POD: what the tree's fold refuses
    // to compose across (a resample node is the missing piece, not a special case here).
    bool SameGround(const Lattice& o) const {
        return kind == o.kind && texW == o.texW && texH == o.texH && faceDim == o.faceDim &&
               orgPxX == o.orgPxX && orgPxY == o.orgPxY && zBase == o.zBase;
    }
    // The constant-buffer row every kernel repeats by hand: origin px x, origin px y,
    // 1 / page texels, world px at zBase (ComposedSurfaceCb::merc, WaterBank's winA).
    void Rows(float out[4]) const {
        out[0] = static_cast<float>(orgPxX);
        out[1] = static_cast<float>(orgPxY);
        out[2] = static_cast<float>(faceDim > 0 ? 1.0 / double(faceDim) : 0.0);
        out[3] = static_cast<float>((1ll << zBase) * 256ll);
    }
    // World pixels at zBase (the Mercator closed form, WaveFieldSource.h's MercX/MercY).
    double WorldPx() const { return double((1ll << zBase) * 256ll); }
    void PxOf(double latDeg, double lonDeg, double& px, double& py) const {
        const double kPi = 3.14159265358979;
        const double l = latDeg * kPi / 180.0;
        px = (lonDeg + 180.0) / 360.0 * WorldPx();
        py = (0.5 - std::log(std::tan(kPi * 0.25 + l * 0.5)) / (2.0 * kPi)) * WorldPx();
    }
};

}  // namespace ga
