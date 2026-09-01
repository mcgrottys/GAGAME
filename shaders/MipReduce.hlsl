// ================================================================================================
//  MipReduce - M9h: FILLING THE CHAIN. A grade bank's coarse mips are not decoration; they are
//  the floor that makes a sample impossible to miss (TileAtlas2D pins the packed tail for
//  exactly that reason). A pinned tail that reads zero is real memory pretending to be data, so
//  the levels have to be generated before the floor means anything.
//
//  A 2x2 box reduction, one level per dispatch, the coarse level written through its OWN mip UAV.
//
//  WHY AVERAGING IS CORRECT HERE, and why it would not be for a texture tenant: these are FIELD
//  banks, where a NULL tile MEANS the quantity is identically zero -- Tier-2's read-zero is the
//  semantics, not a fallback. A fine texel inside a null region genuinely reads 0.0, so
//  averaging it with its neighbours is an honest average OF THE FIELD, not a mix of data with
//  absence. A texture tenant, where null means ABSENT, would need coverage weighting instead:
//  reusing this kernel there would silently darken every partially-resident region.
//
//  Dispatching over the coarse level's FULL extent is deliberate. Writes into null tiles are
//  discarded by the hardware (GAMEPLAN 4.2, hazard 1), so a full dispatch cannot corrupt
//  anything -- only resident coarse tiles receive values -- and it costs nothing worth saving,
//  because every level is a quarter of the one beneath it.
//
//  The channel count arrives as a DEFINE rather than as three entry points: a typed UAV's
//  format must match the resource's family, and one parameterised kernel keeps the reduction
//  itself in a single place where its correctness argument lives.
// ================================================================================================

#ifndef GA_MIP_CH
#define GA_MIP_CH 4
#endif

#if GA_MIP_CH == 1
typedef float MipT;
#elif GA_MIP_CH == 2
typedef float2 MipT;
#else
typedef float4 MipT;
#endif

cbuffer MipCb : register(b0) {
    uint2 gDstDim;      // coarse level extent, texels
    uint2 gSrcDim;      // fine level extent, for the odd-size edge clamp
};

RWTexture2D<MipT> gSrc : register(u0);
RWTexture2D<MipT> gDst : register(u1);

[numthreads(8, 8, 1)]
void CsMipReduce(uint3 id : SV_DispatchThreadID) {
    if (id.x >= gDstDim.x || id.y >= gDstDim.y) return;
    // Clamped at an odd edge so the last column or row averages what actually exists rather
    // than folding a duplicate in and biasing the border.
    const uint2 s0 = id.xy * 2u;
    const uint2 s1 = uint2(min(s0.x + 1u, gSrcDim.x - 1u), min(s0.y + 1u, gSrcDim.y - 1u));
    gDst[id.xy] = (gSrc[s0] + gSrc[uint2(s1.x, s0.y)] + gSrc[uint2(s0.x, s1.y)] + gSrc[s1]) *
                  0.25f;
}
