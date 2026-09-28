// HIERARCHY step 0, probe C: does a WRAP tap of one slice of a reserved array filter across that
// slice's own edge -- the texel at u = 1 mixed with the texel at u = 0 of the SAME slice -- with
// plain SampleLevel and under Sample's min-LOD clamp? The harness is TileSelfTest::ProbeWrap in
// src/hal/TileAtlas.cpp; the row geometry below and its expected values there must stay in step.
//
// PIXEL STAGE ON PURPOSE: a bindless SampleLevel outside it returns zero on this adapter (ALGEBRA
// priors 1), and the windows are read here anyway. The clamp form is PageSample itself, included
// rather than copied, so the probe goes through the engine's own call.
//
// The row: pixel i of 16 samples u = (1022.5 + i / 4) / 1024, a quarter of a mip-0 texel a pixel
// straight across u = 1, at v = 64.5 / 1024 (inside the top row of tiles at mips 0 and 1). u is
// linear in x and constant in y, so the derivatives Sample takes are a quarter texel and zero:
// level -2, which reads mip 0 unclamped and lands exactly on mip 1 under a clamp of 1.
#include "PageSample.hlsli"

cbuffer WrapCb : register(b0) {
    uint gSrv;      // the probe array's slot in the heap
    uint gSlice;    // the slice whose two edge tiles are filled
    uint gMode;     // 0 SampleLevel(0); 1 PageSample with clamp 0; 2 PageSample with clamp 1
    uint gClamp;    // 0 the WRAP sampler; 1 the CLAMP sampler (the planted failure)
};

Texture2DArray gTexArr[] : register(t0, space5);   // the heap as arrays, as Common.hlsli has it
SamplerState sWrap : register(s0);
SamplerState sClamp : register(s1);

// One triangle over the whole viewport; the harness points the viewport at one row per draw.
float4 VsWrap(uint id : SV_VertexID) : SV_Position {
    const float2 t = float2((id << 1) & 2, id & 2);
    return float4(t * float2(2.0f, -2.0f) + float2(-1.0f, 1.0f), 0.0f, 1.0f);
}

// gMode and gClamp are root constants, so every branch below is uniform across the quad and
// Sample's derivatives are the row's own.
float4 Tap(SamplerState s, float2 uv) {
    if (gMode == 0) return gTexArr[gSrv].SampleLevel(s, float3(uv, gSlice), 0.0f);
    return PageSample(gTexArr[gSrv], s, uv, gSlice, gMode == 2 ? 1.0f : 0.0f);
}

float4 PsWrap(float4 pos : SV_Position) : SV_Target {
    const float2 uv = float2((1022.375f + 0.25f * pos.x) / 1024.0f, 64.5f / 1024.0f);
    if (gClamp != 0) return Tap(sClamp, uv);
    return Tap(sWrap, uv);
}
