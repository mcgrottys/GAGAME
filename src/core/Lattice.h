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
//
//  HIERARCHY step 3 adds the FACE-PLANE WINDOW's address beside the cube it lies on (FaceWindow,
//  below): the face's plane and axes, the texel of a point relative to a window's anchor, and the
//  three plane rows a shader turns into that texel (PageSample.hlsli's PageTexel). Nothing reads
//  it yet; the Lattice struct and its bodies are untouched. Gates: the address block after
//  spacetest (core/SpaceTest.cpp) and the address block after tiletest's step 0
//  (hal/TileAtlas.cpp).
// ================================================================================================
#pragma once

#include "core/GaAst.h"
#include "core/GeoRef.h"
#include "core/PageTable.h"
#include "core/TileAddress.h"

#include <cstdint>
#include <string>
#include <vector>

namespace ga {

struct Placement;   // core/Space.h: a frame's placement in its parent, for FaceWindow::PlanesIn

// The quad-sphere's face direction (D3D cubemap convention): face, u, v in [0,1] -> unit vector
// in the planet frame. Defined in Lattice.cpp; pinned by composetest.
void ComposeCubeDir(uint32_t face, double u, double v, double out[3]);
// ...and its inverse: a unit direction -> (face, u, v). The CPU twin of HeightPages.hlsli's
// HpCubeFace, the kernels' hand inverse (no hardware cube sampler outside the pixel stage), in the
// shader's own arithmetic; gatest round-trips 20000 directions through ComposeCubeDir to 1e-9.
inline uint32_t CubeFaceOfDir(const double d[3], double uv[2]) {
    const double a0 = d[0] < 0.0 ? -d[0] : d[0];
    const double a1 = d[1] < 0.0 ? -d[1] : d[1];
    const double a2 = d[2] < 0.0 ? -d[2] : d[2];
    double sx = 0.0, t = 0.0;
    uint32_t face = 0;
    if (a0 >= a1 && a0 >= a2) {
        if (d[0] > 0) { face = 0; sx = -d[2] / a0; t = -d[1] / a0; }
        else          { face = 1; sx =  d[2] / a0; t = -d[1] / a0; }
    } else if (a1 >= a2) {
        if (d[1] > 0) { face = 2; sx =  d[0] / a1; t =  d[2] / a1; }
        else          { face = 3; sx =  d[0] / a1; t = -d[2] / a1; }
    } else {
        if (d[2] > 0) { face = 4; sx =  d[0] / a2; t = -d[1] / a2; }
        else          { face = 5; sx = -d[0] / a2; t = -d[1] / a2; }
    }
    uv[0] = sx * 0.5 + 0.5;
    uv[1] = t * 0.5 + 0.5;
    return face;
}

// ---- HIERARCHY step 3 (docs/HIERARCHY.md 4.4): THE ADDRESS IS A RATIO OF TWO PLANES -----------
//
// A face of the cube is a plane at unit distance from the body's centre: its unit normal n and
// its two in-plane axes a (along +u) and b (along +v) are signed axes of the planet frame, read
// off ComposeCubeDir's table (the point it normalizes is n + s a + t b). A point P of the body,
// planet frame, origin at the centre, has the face coordinates
//     s = (P . a) / (P . n),   t = (P . b) / (P . n),   u = s / 2 + 1 / 2,   v = t / 2 + 1 / 2,
// which is CubeFaceOfDir's inverse on its own face; the address block holds the axes against it
// and against ComposeCubeDir over 20000 directions each way.
void CubeFaceAxes(uint32_t face, double n[3], double a[3], double b[3]);

// A WINDOW: kFaceDim texels a side at its finest rung, in the plane of one face, addressed
// MODULO its size. Rung 0 is the cube's mip 0; rung r has N = kFaceDim 2^r texels across the
// face, and a point's global texel is X = u N (texel i covers [i, i + 1)). A WRAP sampler takes
// the modulo, so X lives at X mod kFaceDim of the window's texture and the address may be taken
// relative to any ANCHOR that is a multiple of kFaceDim texels of the rung; the anchor nearest
// the eye keeps the numbers small:
//     x = X - anchorX = (N / 2) (P . (a - s0 n)) / (P . n),   s0 = 2 anchorX / N - 1
// and y likewise with b and t0. Numerator and denominator are each a PLANE through the body's
// centre evaluated at the point, so they move into any frame the way every plane does
// (Placement::PullPlane), and the frame's scale, common to both, cancels in the ratio. The one
// large cancellation -- the numerator's plane at the eye, a planet's size times N / 2 against
// another -- is taken in doubles here, in PlanesIn; the shader adds only eye-relative terms to
// it. MEASURED on uv_precision.py's own points (the address blocks): at most 0.0033 texel at
// rungs 6 to 15 within reach of the helm in CPU float32, in four frames, and 0.0020 at rung 15
// on this GPU -- where the script finds the float32 direction's spelling 29 texels off at rung
// 15 and today's Mercator spelling 138.
struct FaceWindow {
    uint32_t face = 0;
    int rung = 0;                         // 0 = the cube's mip 0 (Lattice::kFaceDim a face)
    long long anchorX = 0, anchorY = 0;   // texels of the rung, multiples of kFaceDim

    // What a shader carries: the planes U, V and W of the address in the frame its points are
    // given in -- xyz the plane's normal there, w its value at that frame's origin -- so that
    // the texel is (U . p + U.w, V . p + V.w) / (W . p + W.w).
    struct Planes {
        float u[4], v[4], w[4];
    };

    // N: the texels across the face at this rung (exact: a power of two).
    double FaceTexels() const;
    // THE REFERENCE, in doubles: the texel (x, y) of a planet-frame point relative to the anchor,
    // from the definition (X = u N), not from the planes.
    void TexelOf(const double P[3], double& x, double& y) const;
    // This window anchored at the multiple of kFaceDim nearest a planet-frame point's texel, per
    // axis (face and rung kept).
    FaceWindow Nearest(const double P[3]) const;
    // The rows for points given in the frame `own` (x_planet = own.t + own.s R(x_own)): the
    // planes m = (N / 2)(a - s0 n), (N / 2)(b - t0 n) and n, each through PullPlane -- the
    // normal R^T m rotated in doubles and cast after, and w = (m . own.t) / own.s, THE
    // cancellation, in doubles and cast last.
    Planes PlanesIn(const Placement& own) const;
    // PageSample.hlsli's PageTexel on the CPU, in float32, op for op in the HLSL's order: each
    // dot left to right, its plane's w added, then the two divisions. Every product and sum is a
    // named float, so none is carried wider or fused into an FMA: CMakeLists.txt sets no /fp and
    // no /arch, and under MSVC's default /fp:precise nothing has been contracted since VS 2022
    // (/fp:contract or /fp:fast would), while x64's default SSE2 has no FMA to contract into.
    // DXC emits the same shape -- three dot3, three fadd, two fdiv -- but marks them `fast`,
    // which leaves a driver free to fuse inside a dot and to approximate a division: the GPU is
    // recorded against this twin, and gated only against the doubles.
    static void PageTexel(const float p[3], const Planes& pl, float& x, float& y);
};

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
    // M12 step 4c: THE LATTICE AS THE DIAGRAM'S FRAME (core/GaAst.h). The cube IS the table's
    // `cube.face` (+v north: D3D's per-face spec directions) and a window IS its `mercator.px`
    // (+v south: Mercator rows grow south) -- the space named from the kind, +v from VNorth(),
    // origin and pitch 0 as the table's frames have always carried them (the print shows those
    // for metre frames only), centres because Texel samples centres (the +0.5). An edge
    // registered from a lattice is then the hand frame's equal field by field; gatest asserts
    // it for the shipped surface's five lattices against the table's own page-sample row.
    ast::Frame AstFrame() const;
    // The coarsest mip a tree on this lattice carries: faceDim halved down to one tile
    // (TileTree::MaxMip, moved).
    uint32_t MaxMip() const {
        uint32_t m = 0;
        for (uint32_t d = faceDim; d > texW; d >>= 1) ++m;
        return m;
    }
    // The bridge to the page address space (PageTable.h): the same tile, named as a
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
    // World pixels at zBase (the Mercator closed form, the closed forms WaveFieldSource::Align used to carry).
    double WorldPx() const { return double((1ll << zBase) * 256ll); }
    void PxOf(double latDeg, double lonDeg, double& px, double& py) const {
        const double kPi = 3.14159265358979;
        const double l = latDeg * kPi / 180.0;
        px = (lonDeg + 180.0) / 360.0 * WorldPx();
        py = (0.5 - std::log(std::tan(kPi * 0.25 + l * 0.5)) / (2.0 * kPi)) * WorldPx();
    }
};

// HIERARCHY step 3's gate on FaceWindow (core/SpaceTest.cpp), run from --selftest after
// spacetest. Its helm sample is also what the GPU half draws (hal/TileAtlas.cpp, after tiletest's
// step 0), so the two halves judge the same points: tools/hierarchy/uv_precision.py's ground
// points around an eye 3 m over the Merrimack at one rung (6, 9, 12 or 15; any other is empty),
// the ones inside the script's window, with the rows for the planet's own axes.
bool RunFaceWindowSelfTest();
struct FaceWindowSample {
    FaceWindow win;                  // anchored nearest the eye
    std::vector<float> p;            // each point relative to the eye, float32, x y z per point
    std::vector<double> ref;         // its texel relative to the anchor (TexelOf), x y per point
    FaceWindow::Planes planes{};     // PlanesIn(the eye, the planet's axes)
    FaceWindow::Planes planted{};    // the same rows with w's cancellation taken in float32
};
FaceWindowSample FaceWindowHelmSample(int rung);

}  // namespace ga
