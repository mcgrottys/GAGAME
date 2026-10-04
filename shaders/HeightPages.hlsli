// ================================================================================================
//  HeightPages.hlsli -- M9ax: THE BED FROM THE HEIGHT PAGE TENANT, FOR COMPUTE. PHASE B3: on the
//  windows of the pyramid alone.
//
//  The three kernels that simulate on the bed -- the SWE solver, the churn, the wave bank -- read
//  the planet's height by one rule, HpHeightChain below: the cube by direction, then the ranks of
//  the kernel's own chain of windows (its rows, WindowRows.hlsli), each at the kernel's own grain,
//  by manual bilinear Loads (a bindless SampleLevel outside the pixel stage returns zero: ALGEBRA
//  priors 1). The pixel stage's form is Compose.hlsli's ComposedHeightChain. The Mercator page this
//  file read (HpHeightAt, PageUvLatLon, the z14 constants) is deleted.
// ================================================================================================
#ifndef HEIGHT_PAGES_HLSLI
#define HEIGHT_PAGES_HLSLI

#include "PageSample.hlsli"

// The cube's ground resolution, the lattice's own (Lattice::GroundRes(0): kMercCirc / (4 faceDim)),
// folded at compile time; a rank's is it over 8^k (HpRankGround).
static const float kHpMercCirc = 40075016.686f;   // Web-Mercator equator, m (Lattice::kMercCirc)
static const float kHpCubeTexelM = kHpMercCirc / (4.0f * kPageDim);   // 611.496..
static const float kHpMaxMip = 6.0f;           // 7 mips

// THE BED TRACE (an instrument: SweSolver::TraceBed, shaders/Swe.hlsl CsSweBedTrace). Compiled
// with HP_TRACE defined, the rule below also records what it chose -- the mip it read at and the
// slice it read -- and takes a floor under both residency reads, which the planted failure
// raises to the coarsest mip. Every other kernel compiles without it, and for them the two hooks
// are the expression they wrap and nothing: the solver, the churn and the bank read the bed they
// always read.
#ifdef HP_TRACE
static float gHpTraceMip = -1.0f;    // the mip the last read chose
static uint gHpTraceSlice = 0u;      // ...and the slice: a cube face 0..5, or a window's
static float gHpTraceFloor = 0.0f;   // a residency floor the trace imposes; 0 = the map's own
#define HP_HAVE(have) max((have), gHpTraceFloor)
#define HP_CHOSE(mip, slice) gHpTraceMip = (mip); gHpTraceSlice = (slice)
#else
#define HP_HAVE(have) (have)
#define HP_CHOSE(mip, slice)
#endif

// HpCubeFace (direction -> D3D cube face and its uv) lives in PageSample.hlsli, included above.

// ---- PHASE B1 (out/integration/plan_phase_b.md): THE BED BY THE CHAIN, the kernels' block form.
// The Mercator page's rule with the page replaced by the windows of the pyramid (HIERARCHY 4.1): the
// cube by direction, then the ranks of the kernel's own chain, coarsest first -- each read at the
// level of the kernel's own grain (texelM, metres: the bank's ring, the churn's texel, the solver's
// cell): the finest level of the rank no finer than that grain, m_k = max(floor(log2(texelM /
// g_k)), 0), g_k the rank's mip-0 ground -- and taken where it is resident to that level or finer
// and at least as fine as the ground held (PageWins). A grain past a window's floor (the coarsest
// mip a window holds, HP_WINDOW_FLOOR) leaves the point to the rank above, which holds that ground
// at its own finer mips, and every finer rank with it. One bilinear is paid, at the winner, by
// Loads with the taps modulo the page (priors 1: no sampler outside the pixel stage).
//
// The chain wc is Window.hlsli's, found by the kernel at ITS point from ITS rows (the kernel's
// cbuffer carries HP_WINDOW_ROWS_DECL; HP_WINDOW_ROWS turns the chain's accessors onto them) --
// the address is a ratio of planes at the kernel's own point, as every reader's is.
#ifndef HP_WINDOW_FLOOR
#define HP_WINDOW_FLOOR 3.0f
#endif
// The reads, as hooks: the tenant's own by default; the selftest's synthetic tenant names its own
// (TileBed.hlsl), so one body is held against its C++ twin.
#ifndef HP_FETCH_WIN
#define HP_FETCH_WIN(arr, uv, slice, mip) PageLoadWrap(arr, uv, slice, mip)
#define HP_HAVE_WIN(res, uv, slice) PageHaveLoadWrap(res, uv, slice)
#define HP_FETCH_CUBE(arr, uv, face, mip) PageLoad(arr, uv, face, mip)
#define HP_HAVE_CUBE(res, uv, face) PageHaveLoad(res, uv, face)
#endif
// The rows a kernel carries for its chain: WindowRows.hlsli's HP_WINDOW_ROWS_DECL, the last rows of
// its cbuffer.
#ifdef HP_WINDOW_ROWS
#define WIN_UV(s, i, p) PageTexelUv(p, gHwU[i], gHwV[i], gHwW[i])
#define WIN_OFF(s, i) ((((i) & 1u) != 0u) ? gHwO[(i) >> 1].zw : gHwO[(i) >> 1].xy)
#define WIN_SLICE(s, i) (((i) < 4u) ? gHwS[0][(i)] : gHwS[1].x)
#define WIN_K(s) gHwS[1].y
// PHASE B2: the rank of the chain's first entry, less one (0 for the eye's windows; a standing
// window's chain starts at its own rank).
#define HP_WIN_RANK0 gHwS[1].z
#include "Window.hlsli"
#endif
// A rank's mip-0 ground: the pyramid's nominal texel at rung 3 (k + 1) (BlockBinding::GroundRes(0)).
float HpRankGround(uint k) { return kHpCubeTexelM / exp2(3.0f * float(k + 1u)); }
#ifndef HP_WIN_RANK0
#define HP_WIN_RANK0 0u
#endif
#ifdef GA_WINDOW_HLSLI
// The rule, for a tenant with the cube (the height) or without it (the exposure: window slices
// alone, PHASE B2's D2 -- where no window answers, `none`).
float HpChainRead(Texture2DArray<float4> arr, Texture2DArray<float4> res, float3 dir, WalkChain wc,
                  float texelM, bool cube, float none) {
    float2 cuv;
    const uint face = HpCubeFace(dir, cuv);
    const float haveC = clamp(round(HP_HAVE(HP_HAVE_CUBE(res, cuv, face))), 0.0f, kHpMaxMip);
    float ground = cube ? PageGroundM(kHpCubeTexelM, haveC) : 3.0e38f;
    uint sl = face;
    float2 uv = cuv;
    float mip = haveC;
    bool win = false;
    [unroll] for (uint k = 0; k < GA_BLOCK_RANKS; ++k) {
        if (k >= wc.n) break;
        const float g0 = HpRankGround(k + HP_WIN_RANK0);
        const float want = max(floor(log2(max(texelM, 1e-6f) / g0)), 0.0f);
        if (want > HP_WINDOW_FLOOR) break;
        const uint s = WalkSlice(wc, k);
        const float2 u = WalkUv(wc, k);
        const float haveB = round(HP_HAVE(HP_HAVE_WIN(res, u, s)));
        if (haveB > HP_WINDOW_FLOOR) continue;
        const float m = max(haveB, want);
        const float gB = PageGroundM(g0, m);
        if (PageWins(gB, ground)) {
            sl = s;
            uv = u;
            mip = m;
            ground = gB;
            win = true;
        }
    }
    HP_CHOSE(win || cube ? mip : -1.0f, sl);
    if (win) return HP_FETCH_WIN(arr, uv, sl, mip);
    return cube ? HP_FETCH_CUBE(arr, cuv, face, haveC) : none;
}
float HpHeightChain(Texture2DArray<float4> arr, Texture2DArray<float4> res, float3 dir, WalkChain wc,
                    float texelM) {
    return HpChainRead(arr, res, dir, wc, texelM, true, 0.0f);
}
#endif

#endif
