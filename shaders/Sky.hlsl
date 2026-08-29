// Sky. A fullscreen pass with depth test and depth write both OFF, drawn before anything else.
// It uses the SAME SkyRadianceDir() every reflection uses, from Common.hlsli -- with two copies
// of a sky model, a reflection stops matching the sky above it.
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
    return float4(SkyRadianceDir(ViewRay(i.ndc)), 1.0f);
}
