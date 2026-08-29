// M6i: VECTOR GIS on the GPU. The survey polylines ride up as raw (lon, lat) pairs and are
// projected onto the planet IN-SHADER through the same composed rows every layer shares --
// the projection is a realization, the vector stays the authority, and the lines are crisp at
// every zoom because there is no raster in the path at all. Depth-off overlay: alignment
// truth draws over whatever any other layer claimed.
#define GA_NO_FIELD_BUFFER
#include "Common.hlsli"

cbuffer GisCb : register(b1) {
    GA_COMPOSED_CB_ROWS
    float4 gGisColor;   // rgb = line color, w = lift above the geoid (m)
};

#include "Compose.hlsli"

StructuredBuffer<float2> gGisPts : register(t0, space0);   // lon/lat degrees, 2 per segment

struct VsOut {
    float4 pos : SV_Position;
    float3 col : COLOR0;
    float cull : TEXCOORD0;   // > 0 past the horizon; the PS discards (a vertex-side warp
                              // sprayed half-culled segments across the screen)
};

VsOut VsMain(uint vid : SV_VertexID) {
    const float2 ll = gGisPts[vid];
    const float lon = radians(ll.x), lat = radians(ll.y);
    const float3 dir = float3(cos(lat) * cos(lon), sin(lat), cos(lat) * sin(lon));
    // Planet -> tangent -> camera-relative: the globe's own one-world math, on a survey point.
    const float3 pT = CsToTangent(dir) * (gCsF.w + gGisColor.w);
    const float3 camT = float3(gEyeRel.x, gEyeRel.y + gCsF.w, gEyeRel.z);
    VsOut o;
    o.col = gGisColor.rgb;
    o.pos = mul(float4(pT - camT, 1.0f), gViewProj);
    const float camR = length(camT);
    o.cull = max(0.0f, gCsF.w / camR - 0.03f - dot(pT, camT) / (gCsF.w * camR));
    return o;
}

float4 PsMain(VsOut i) : SV_Target {
    if (i.cull > 1e-5f) discard;   // the far side of the planet, on a depth-off overlay
    return float4(i.col, 1.0f);
}
