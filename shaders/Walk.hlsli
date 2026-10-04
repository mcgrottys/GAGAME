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
struct WalkChain {
    uint n;
    uint s0, s1, s2, s3, s4;
    float2 uv0, uv1, uv2, uv3, uv4;
};
// Rank k + 1 of a chain, for a loop the compiler unrolls: k is a constant there and the selects
// fold to the one named value.
uint WalkSlice(WalkChain w, uint k) {
    return (k == 0u) ? w.s0 : (k == 1u) ? w.s1 : (k == 2u) ? w.s2 : (k == 3u) ? w.s3 : w.s4;
}
float2 WalkUv(WalkChain w, uint k) {
    return (k == 0u) ? w.uv0 : (k == 1u) ? w.uv1 : (k == 2u) ? w.uv2 : (k == 3u) ? w.uv3 : w.uv4;
}
void WalkSet(inout WalkChain w, uint k, uint s, float2 uv) {
    if (k == 0u) { w.s0 = s; w.uv0 = uv; }
    else if (k == 1u) { w.s1 = s; w.uv1 = uv; }
    else if (k == 2u) { w.s2 = s; w.uv2 = uv; }
    else if (k == 3u) { w.s3 = s; w.uv3 = uv; }
    else { w.s4 = s; w.uv4 = uv; }
}

#endif  // GA_WALK_HLSLI
