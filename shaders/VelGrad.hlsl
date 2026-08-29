// ================================================================================================
//  M3: THE FIRST LIVE GEOMETRIC ALGEBRA PASS.
//
//  One dispatch over the GoMOFS surface-current field computes the gradient's multivector:
//  divergence lands in grade 0, the current itself rides along as grade 1, and vorticity lands
//  in grade 2 -- one RGBA texel per cell, exactly the Multivector2 layout vqview's FieldSet
//  declared and never used. Okubo-Weiss (strain^2 - vorticity^2) drops out beside it; its
//  negative regions are rotation-dominated water: the eddies the gulf view lights up.
//
//  This file is the first call site GA.hlsli has ever had. GAMEPLAN.md section 5, application 2.
// ================================================================================================
#include "GA.hlsli"

cbuffer VelGradCb : register(b0) {
    uint  gNx, gNy;
    float gCellMx, gCellMy;   // metres per cell, east / north
};

RWTexture2D<float4> gUvIn  : register(u0);   // u east m/s, v north m/s, mask (1 water), 0
RWTexture2D<float4> gMvOut : register(u1);   // PackMv2: div, u, v, curl
RWTexture2D<float4> gOwOut : register(u2);   // Okubo-Weiss, speed, mask, 0

[numthreads(8, 8, 1)]
void CsVelGrad(uint3 id : SV_DispatchThreadID) {
    if (id.x >= gNx || id.y >= gNy) return;
    const float4 c = gUvIn[id.xy];
    if (c.z < 0.5f) {
        gMvOut[id.xy] = 0;
        gOwOut[id.xy] = float4(0, 0, 0, 0);
        return;
    }

    // Central differences where both neighbours are water; one-sided at coasts.
    float4 xm = (id.x > 0) ? gUvIn[uint2(id.x - 1, id.y)] : c;
    float4 xp = (id.x + 1 < gNx) ? gUvIn[uint2(id.x + 1, id.y)] : c;
    float4 ym = (id.y > 0) ? gUvIn[uint2(id.x, id.y - 1)] : c;
    float4 yp = (id.y + 1 < gNy) ? gUvIn[uint2(id.x, id.y + 1)] : c;
    if (xm.z < 0.5f) xm = c;
    if (xp.z < 0.5f) xp = c;
    if (ym.z < 0.5f) ym = c;
    if (yp.z < 0.5f) yp = c;

    const float sx = max((xp.z > 0.5f ? 1.0f : 0.0f) + (xm.z > 0.5f ? 1.0f : 0.0f), 1.0f);
    const float sy = max((yp.z > 0.5f ? 1.0f : 0.0f) + (ym.z > 0.5f ? 1.0f : 0.0f), 1.0f);
    const float dudx = (xp.x - xm.x) / (sx * gCellMx);
    const float dvdx = (xp.y - xm.y) / (sx * gCellMx);
    const float dudy = (yp.x - ym.x) / (sy * gCellMy);
    const float dvdy = (yp.y - ym.y) / (sy * gCellMy);

    // grad applied to the current: divergence is the grade-0 part, vorticity the grade-2 part;
    // the current itself sits in grade 1 so one texel carries the whole story.
    //
    // ** FP16 UNDERFLOW TRAP. ** Ocean vorticity is ~1e-5 1/s and Okubo-Weiss ~1e-9 1/s^2 --
    // below half-float's subnormal floor, so raw values flush to zero in the RGBA16F targets
    // and every eddy silently vanishes. Store scaled: grades 0/2 in 1e-5 1/s units, OW in
    // 1e-10 1/s^2 units. The map shader thresholds in the same scaled units.
    Mv2 m;
    m.s = (dudx + dvdy) * 1.0e5f;
    m.v = c.xy;
    m.b = (dvdx - dudy) * 1.0e5f;
    gMvOut[id.xy] = PackMv2(m);

    gOwOut[id.xy] = float4(OkuboWeiss(dudx, dudy, dvdx, dvdy) * 1.0e10f,
                           length(c.xy), 1.0f, 0.0f);
}
