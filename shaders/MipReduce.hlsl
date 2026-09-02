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

// Array views pinned to ONE slice by the caller (TileAtlas2D::BuildMips), so the reduction
// never crosses a page boundary -- pages are independent by construction.
RWTexture2DArray<MipT> gSrc : register(u0);
RWTexture2DArray<MipT> gDst : register(u1);

[numthreads(8, 8, 1)]
void CsMipReduce(uint3 id : SV_DispatchThreadID) {
    if (id.x >= gDstDim.x || id.y >= gDstDim.y) return;
    // Clamped at an odd edge so the last column or row averages what actually exists rather
    // than folding a duplicate in and biasing the border.
    const uint2 s0 = id.xy * 2u;
    const uint2 s1 = uint2(min(s0.x + 1u, gSrcDim.x - 1u), min(s0.y + 1u, gSrcDim.y - 1u));
    const MipT a = gSrc[uint3(s0, 0)];
    const MipT b = gSrc[uint3(s1.x, s0.y, 0)];
    const MipT c = gSrc[uint3(s0.x, s1.y, 0)];
    const MipT d = gSrc[uint3(s1, 0)];

#ifdef GA_MIP_COVCH
    // ---- COVERAGE-WEIGHTED: ABSENCE MUST NOT VOTE. -------------------------------------------
    // The plain box average above is correct only when a null texel MEANS the quantity is
    // identically zero. The moment a bank carries coverage -- because its sources do not reach
    // everywhere, which is the normal case for anything composed -- averaging four texels of
    // which two are absent halves the value and reports it as fact. The error compounds up the
    // chain: each level folds more absence in, so the coarsest mips (the PINNED floor, the ones
    // that can never be missing and are therefore what a distant sample actually reads) are the
    // most corrupted. That is backwards from every other error in the system.
    //
    // So the value channels are averaged weighted by coverage, and coverage itself is averaged
    // PLAIN -- it is the fraction of the coarse texel that had data, which is a mean of the four
    // fractions, not a weighted mean of itself.
    const float wa = a[GA_MIP_COVCH], wb = b[GA_MIP_COVCH];
    const float wc = c[GA_MIP_COVCH], wd = d[GA_MIP_COVCH];
    const float wsum = wa + wb + wc + wd;
    MipT o = (MipT)0;
    if (wsum > 0.0f) o = (a * wa + b * wb + c * wc + d * wd) / wsum;
    o[GA_MIP_COVCH] = wsum * 0.25f;   // fully absent -> 0: still absent, and says so
    gDst[uint3(id.xy, 0)] = o;
#else
    gDst[uint3(id.xy, 0)] = (a + b + c + d) * 0.25f;
#endif
}
