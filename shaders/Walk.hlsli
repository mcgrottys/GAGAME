// ================================================================================================
//  Walk.hlsli - HIERARCHY 4.17 commit 4: THE DIRECTORY WALK, its HLSL body. The C++ body is
//  SurfaceFrame::Walk; the selftest reads this one back through TileWalk.hlsl and holds the two
//  equal, slice for slice and to 0.01 texel.
//
//  From a ground point p (the undisplaced point relative to the eye, in the frame the rows were
//  pulled into) and its direction: the face, the face's directory cell, the block that cell
//  names, that block's uv by its rows, its directory cell, and on, to where no block is named. A
//  rank's block is an eighth of its parent and a cell a sixteenth, and blocks are aligned, so a
//  cell lies in one block of the next rank or in none (SurfaceFrame::DeclareBlocks refuses a key
//  where that does not hold) and the walk meets one block a rank. The directory is a function of
//  the ground and of the blocks that exist, never of a camera.
//
//  Compiled only where the scene's key stands: GA_BLOCK_RANKS is the key's ranks (the engine's
//  define, SurfaceFrame::Ranks), and the includer names a block's uv at a point, WALK_UV(i, p) --
//  the surface's rows in Compose.hlsli, the selftest's own in TileWalk.hlsl -- so both run this
//  one body.
// ================================================================================================
#ifndef GA_WALK_HLSLI
#define GA_WALK_HLSLI

#if !GA_BLOCK_RANKS
#error Walk.hlsli is the standing blocks' code: compile it with GA_BLOCK_RANKS, the key's ranks
#endif
#ifndef WALK_UV
#error Walk.hlsli needs WALK_UV(i, p), block i's uv at the point p
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

// One directory cell: 16 x 16 cells a slice, the slices stacked down one R16_UINT texture (slice s
// at rows 16 s to 16 s + 15). The face's cells name rank 1; a block's name the next rank's; a cell
// that names no block holds 0xFFFF.
uint WalkCell(Texture2D<uint> dir, uint slice, float2 uv) {
    const int2 c = clamp(int2(floor(uv * 16.0f)), int2(0, 0), int2(15, 15));
    return dir.Load(int3(c.x, c.y + 16 * int(slice), 0));
}

// The walk: the face's cell, then one a rank past it -- GA_BLOCK_RANKS loads at most, since the
// finest rank's cells name nothing further and are not read.
WalkChain Walk(float3 dirP, float3 p, Texture2D<uint> dir) {
    WalkChain w = (WalkChain)0;
    float2 fuv;
    const uint face = HpCubeFace(dirP, fuv);   // its own statement: fuv is HpCubeFace's out, and
    uint s = WalkCell(dir, face, fuv);         // an argument list sets no order (the selftest caught it)
    [unroll] for (uint k = 0; k < GA_BLOCK_RANKS; ++k) {
        if (s < 6u || s >= 14u) break;   // no block named (0xFFFF), or past the eight rows
        const float2 uv = WALK_UV(s - 6u, p);
        WalkSet(w, k, s, uv);
        w.n = k + 1u;
        if (k + 1u < GA_BLOCK_RANKS) s = WalkCell(dir, s, uv);
    }
    return w;
}

#endif  // GA_WALK_HLSLI
