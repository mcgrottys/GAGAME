// ================================================================================================
//  Window.hlsli - PHASE A1 (out/integration/plan_eye_windows.md): THE EYE'S CHAIN, its HLSL body.
//  The C++ body is SurfaceFrame::Chain; the selftest reads this one back through TileChain.hlsl
//  and holds the two equal, rank for rank, against the doubles too.
//
//  Rank k + 1's window is a box of 16384 texels of rung 3 (k + 1) about the eye (HIERARCHY 4.1).
//  A point's address in it is PageTexelUv by the rank's planes -- the texel less the multiple of
//  16384 nearest the eye, over 16384: what a WRAP sampler reads modulo 16384 -- and the point is in
//  the box where the address plus the box's origin less that anchor (WIN_OFF) lies in [0, 1). The
//  boxes nest about one eye, so the chain is the ranks from 1 up to the first that does not hold
//  the point: one ratio and one compare a rank, no table. Every rank's address is kept, live or
//  not, so a reader's footprint is the hardware's across a box's edge.
//
//  The includer names the rows of a level's slot s: WIN_UV(s, i, p) rank i + 1's address at p,
//  WIN_OFF(s, i) its box's origin less the anchor, WIN_SLICE(s, i) its slice, WIN_K(s) the ranks
//  live -- the surface's rows in Compose.hlsli (a slot a level, PHASE A2), the selftest's own in
//  TileChain.hlsl. p is relative to THAT slot's eye.
// ================================================================================================
#ifndef GA_WINDOW_HLSLI
#define GA_WINDOW_HLSLI

#if !defined(WIN_UV) || !defined(WIN_OFF) || !defined(WIN_SLICE) || !defined(WIN_K)
#error Window.hlsli needs WIN_UV(s, i, p), WIN_OFF(s, i), WIN_SLICE(s, i) and WIN_K(s)
#endif
#ifndef GA_BLOCK_RANKS
#define GA_BLOCK_RANKS 5
#endif
#include "Walk.hlsli"   // the chain's container and accessors (WalkChain)

WalkChain WindowChain(float3 p, uint s) {
    WalkChain w = (WalkChain)0;
    bool inside = true;
    [unroll] for (uint k = 0; k < GA_BLOCK_RANKS; ++k) {
        const float2 uv = WIN_UV(s, k, p);
        const float2 b = uv + WIN_OFF(s, k);
        inside = inside && k < (WIN_K(s)) && all(b >= 0.0f) && all(b < 1.0f);
        WalkSet(w, k, WIN_SLICE(s, k), uv);
        if (inside) w.n = k + 1u;
    }
    return w;
}

#endif  // GA_WINDOW_HLSLI
