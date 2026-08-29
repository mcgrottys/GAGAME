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

// ACES-ish filmic curve (Krzysztof Narkowicz's fit). Rolls highlights off instead of clipping.
float3 TonemapACES(float3 x) {
    const float a = 2.51f, b = 0.03f, c = 2.43f, d = 0.59f, e = 0.14f;
    return saturate((x * (a * x + b)) / (x * (c * x + d) + e));
}

float4 PsMain(VsOut i) : SV_Target {
    float3 hdr = gTex[gSceneColorSrv].SampleLevel(sPointClamp, i.uv, 0).rgb;
    hdr *= gExposure;
    float3 ldr = TonemapACES(hdr);
    // The target is R8G8B8A8_UNORM, not _SRGB, so the transfer function is applied here.
    ldr = pow(max(ldr, 0.0f), 1.0f / 2.2f);
    return float4(ldr, 1.0f);
}
