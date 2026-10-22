// ================================================================================================
//  Walk.hlsli - THE CHAIN'S CONTAINER: the blocks a point lies in, one a rank, coarsest first, and
//  their accessors. PHASE B3: the directory walk that filled it from standing blocks is deleted; the
//  eye's windows fill it (Window.hlsli's WindowChain, SurfaceFrame::Chain its C++ body).
// ================================================================================================
#ifndef GA_WALK_HLSLI
#define GA_WALK_HLSLI

#if !GA_BLOCK_RANKS
#error Walk.hlsli: compile it with GA_BLOCK_RANKS, the ranks a chain carries
#endif

#include "PageSample.hlsli"

// The blocks met, coarsest rank first: n of them, rank k + 1's slice sk and its uv (its texel over
// 16384) at the point. NAMED VALUES, not arrays: a stage finds the chain once and hands it to
// every reader, whose loop over the ranks unrolls, so each rank's value is a register.
// ...and e, THE POINT'S DISTANCE TO ITS BOX'S EDGE in uv (2026-10-19, the Mavericks horizon): a
// window is a torus, addressed modulo its own 16384 texels, and a FILTERED read near the box's edge
// reaches across it into the box's opposite side -- land 19 km east blended into the sea 10 km west,
// a vertex at the far horizon lifted a hundred metres (Mark: "they are obviously on the edge of
// different mip levels"). A rank answers only where the footprint of the read lies within the box
// (WalkEdgeOk); the rank above holds the same ground at its mip 0 with the point deep inside.
struct WalkChain {
    uint n;
    uint s0, s1, s2, s3, s4;
    float2 uv0, uv1, uv2, uv3, uv4;
    float e0, e1, e2, e3, e4;
};
// Rank k + 1 of a chain, for a loop the compiler unrolls: k is a constant there and the selects
// fold to the one named value.
uint WalkSlice(WalkChain w, uint k) {
    return (k == 0u) ? w.s0 : (k == 1u) ? w.s1 : (k == 2u) ? w.s2 : (k == 3u) ? w.s3 : w.s4;
}
float2 WalkUv(WalkChain w, uint k) {
    return (k == 0u) ? w.uv0 : (k == 1u) ? w.uv1 : (k == 2u) ? w.uv2 : (k == 3u) ? w.uv3 : w.uv4;
}
float WalkEdge(WalkChain w, uint k) {
    return (k == 0u) ? w.e0 : (k == 1u) ? w.e1 : (k == 2u) ? w.e2 : (k == 3u) ? w.e3 : w.e4;
}
void WalkSet(inout WalkChain w, uint k, uint s, float2 uv, float e) {
    if (k == 0u) { w.s0 = s; w.uv0 = uv; w.e0 = e; }
    else if (k == 1u) { w.s1 = s; w.uv1 = uv; w.e1 = e; }
    else if (k == 2u) { w.s2 = s; w.uv2 = uv; w.e2 = e; }
    else if (k == 3u) { w.s3 = s; w.uv3 = uv; w.e3 = e; }
    else { w.s4 = s; w.uv4 = uv; w.e4 = e; }
}
// Does rank k's box hold the footprint of a filtered read at mip `mip` with `taps` texels of
// reach (2 for a bilinear or trilinear tap, 8 for an anisotropic one)? The footprint in uv is
// taps x 2^mip texels over the page's 16384.
bool WalkEdgeOk(WalkChain w, uint k, float mip, float taps) {
    return WalkEdge(w, k) >= taps * exp2(max(mip, 0.0f)) / kPageDim;
}

#endif  // GA_WALK_HLSLI
