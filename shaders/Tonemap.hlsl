// Fullscreen tonemap: linear HDR radiance -> display-referred 8-bit.
// A separate pass so anything volumetric can composite into LINEAR radiance before the curve.
#include "Common.hlsli"

cbuffer TonemapCb : register(b1) {
    uint gSceneColorSrv;
    uint3 _pad;
};

struct VsOut {
    float4 pos : SV_Position;
    float2 uv  : TEXCOORD0;
};

// One triangle covering the viewport, generated from the vertex id.
VsOut VsMain(uint vid : SV_VertexID) {
    VsOut o;
    const float2 xy = float2((vid == 1) ? 3.0f : -1.0f, (vid == 2) ? 3.0f : -1.0f);
    o.pos = float4(xy, 0.0f, 1.0f);
    o.uv = xy * float2(0.5f, -0.5f) + 0.5f;
    return o;
}

// M6j: ACES is GONE. Its filmic mid-tone boost (0.39 -> 0.52) and desaturation are built for
// scene-referred HDR photography -- applied to imagery Google's pipeline already tone-mapped,
// it bleached the whole planet toward chalk (the "washed out" report). The curve was habit,
// not need. Below the knee the data passes UNTOUCHED; only genuine HDR -- sun glint, the
// solar disc -- takes a smooth exponential shoulder to the display ceiling.
float3 TonemapShoulder(float3 x) {
    const float knee = 0.85f;
    const float3 excess = max(x - knee, 0.0f);
    return min(x, knee) + (1.0f - knee) * (1.0f - exp(-excess / (1.0f - knee)));
}

float4 PsMain(VsOut i) : SV_Target {
    float3 hdr = gTex[gSceneColorSrv].SampleLevel(sPointClamp, i.uv, 0).rgb;
    hdr *= gExposure;
    float3 ldr = TonemapShoulder(max(hdr, 0.0f));
    // The target is R8G8B8A8_UNORM, not _SRGB, so the transfer function is applied here.
    ldr = pow(ldr, 1.0f / 2.2f);
    return float4(ldr, 1.0f);
}
