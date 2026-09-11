// Sky. A fullscreen pass with depth test and depth write both OFF, drawn before anything else.
// It uses the SAME SkyRadianceDir() every reflection uses, from Common.hlsli -- with two copies
// of a sky model, a reflection stops matching the sky above it.
//
// M10: the dome is evaluated in ITS OWN world's frame (SkyLayer::SetSkyFrame). The rows take the
// view ray from the camera's frame into the sky's, and the sun is the sky's sun. Identity and
// the scene's sun are the default -- the old pass exactly; inside a twisted Droste level under
// realistic lighting the backdrop is the root's sky, turned.
cbuffer SkyFrameCb : register(b1) {
    float4 gSkyR0;
    float4 gSkyR1;
    float4 gSkyR2;
    float4 gSkySun;
};
#define GA_SUN_DIR (gSkySun.xyz)
#include "Common.hlsli"

struct VsOut {
    float4 pos : SV_Position;
    float2 ndc : TEXCOORD0;
};

VsOut VsMain(uint vid : SV_VertexID) {
    VsOut o;
    const float2 xy = float2((vid == 1) ? 3.0f : -1.0f, (vid == 2) ? 3.0f : -1.0f);
    o.pos = float4(xy, 0.0f, 1.0f);
    o.ndc = xy;
    return o;
}

float4 PsMain(VsOut i) : SV_Target {
    const float3 r = ViewRay(i.ndc);
    const float3 d = float3(dot(gSkyR0.xyz, r), dot(gSkyR1.xyz, r), dot(gSkyR2.xyz, r));
    return float4(SkyRadianceDir(d), 1.0f);
}
