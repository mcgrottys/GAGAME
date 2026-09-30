// ================================================================================================
//  HeightPages.hlsli -- M9ax: THE BED FROM THE HEIGHT PAGE TENANT, FOR COMPUTE.
//
//  The pixel stage resolves the planet's height with ComposedHeightPages (Compose.hlsli): the
//  z14 Mercator page by containment where its resident texel is at least as fine as the cube's,
//  else the cube face by direction, the residency map clamping the mip. The three kernels that
//  simulate on the bed -- the SWE solver, the churn, the wave bank -- each carried their own copy
//  of HALF that rule: the page only, with a wall / -30 m / a CPU corner lerp outside it
//  (AUDIT_WATER item 5, "the keyhole"). This is the whole rule, once, for any stage: manual
//  bilinear Loads, because a bindless SampleLevel outside the pixel stage returns zero
//  (ALGEBRA priors 1), on the tenant's full array view (slices 0..5 the cube faces, 6 the z14
//  page) and its residency-map array. M12 step 4e: the reads are PageSample.hlsli's contract
//  (PageHaveLoad, PageLoad, PageWins) -- the same law the pixel stage binds its views to.
//
//  Frames (GaAst: height.pages -> {swe.solver, churn.kernel, water.bank}, merc-uv, no flip):
//  lat/lon -> Mercator px -> page uv is PageUvLatLon, the lat/lon spelling of the window frame
//  (measured NOT bit-identical to PageUv of the same direction: PageSample.hlsli's banner);
//  lat/lon -> direction -> cube face uv is ComposeCubeDir's inverse (the D3D cube convention
//  the tiles were painted in).
// ================================================================================================
#ifndef HEIGHT_PAGES_HLSLI
#define HEIGHT_PAGES_HLSLI

#include "PageSample.hlsli"

// M12 step 4f: the two ground resolutions are the lattices' own (Lattice::GroundRes(0):
// kMercCirc / (4 faceDim) for the cube, kMercCirc / world px at z14 for the page), folded at
// compile time from the constants this file declares -- the kernels' rows carry no ground and
// nothing else changes -- and bit-identical to the floats the surface's gCsGround row carries
// (a division by a power of two commutes with rounding: 0x4418dfc2 and 0x4118dfc2 both ways).
// Exact, the page's mip 6 equals the cube's mip 0 and PageWins takes the page there, where
// 611.0f and 9.55f (9.55 * 64 = 611.2 > 611) took the cube.
static const float kHpMercCirc = 40075016.686f;   // Web-Mercator equator, m (Lattice::kMercCirc)
static const float kHpWorldPxZ14 = 4194304.0f;    // (1 << 14) * 256 px (Lattice::WorldPx at z14)
static const float kHpCubeTexelM = kHpMercCirc / (4.0f * kPageDim);   // 611.496.. (was 611.0f)
static const float kHpPageTexelM = kHpMercCirc / kHpWorldPxZ14;        // 9.5546.. (was 9.55f)
static const float kHpMaxMip = 6.0f;           // 7 mips

// THE BED TRACE (an instrument: SweSolver::TraceBed, shaders/Swe.hlsl CsSweBedTrace). Compiled
// with HP_TRACE defined, the rule below also records what it chose -- the mip it read at and the
// slice it read -- and takes a floor under both residency reads, which the planted failure
// raises to the coarsest mip. Every other kernel compiles without it, and for them the two hooks
// are the expression they wrap and nothing: the solver, the churn and the bank read the bed they
// always read.
#ifdef HP_TRACE
static float gHpTraceMip = -1.0f;    // the mip the last HpHeightAt read at
static uint gHpTraceSlice = 0u;      // ...and the slice: a cube face 0..5, or the page's
static float gHpTraceFloor = 0.0f;   // a residency floor the trace imposes; 0 = the map's own
#define HP_HAVE(have) max((have), gHpTraceFloor)
#define HP_CHOSE(mip, slice) gHpTraceMip = (mip); gHpTraceSlice = (slice)
#else
#define HP_HAVE(have) (have)
#define HP_CHOSE(mip, slice)
#endif

// HpCubeFace (direction -> D3D cube face and its uv) lives in PageSample.hlsli, included above.

// The planet's height (m NAVD) at lat/lon (degrees). winA = the page's lattice row (org px x,
// org px y, 1/kPageDim, world px at z14: Lattice::Rows); winSlice the page's slice; pageMipMin
// the coarsest-allowed page mip a consumer wants to be held to (the bank rings ask their own
// grain -- mip 0 for the fine rings; the solver 0).
float HpHeightAt(Texture2DArray<float4> arr, Texture2DArray<float4> res, float latDeg,
                 float lonDeg, float4 winA, uint winSlice, float pageMipMin) {
    const float latR = latDeg * 0.01745329252f;
    const float lonR = lonDeg * 0.01745329252f;
    const float cl = cos(latR);
    const float3 dir = float3(cl * cos(lonR), sin(latR), cl * sin(lonR));
    float2 cuv;
    const uint face = HpCubeFace(dir, cuv);
    const float haveC = clamp(round(HP_HAVE(PageHaveLoad(res, cuv, face))), 0.0f, kHpMaxMip);
    // The page, by containment, where it is at least as fine as the cube. Two residency
    // reads decide; only ONE bilinear is paid (the bank kernel runs this per texel per ring).
    const float2 wuv = PageUvLatLon(latDeg, lonDeg, winA);
    if (all(wuv > 0.0f) && all(wuv < 1.0f)) {
        const float haveW =
            clamp(round(HP_HAVE(PageHaveLoad(res, wuv, winSlice))), pageMipMin, kHpMaxMip);
        if (PageWins(haveC, haveW, float2(kHpCubeTexelM, kHpPageTexelM))) {
            HP_CHOSE(haveW, winSlice);
            return PageLoad(arr, wuv, winSlice, haveW);
        }
    }
    HP_CHOSE(haveC, face);
    return PageLoad(arr, cuv, face, haveC);
}

#endif
