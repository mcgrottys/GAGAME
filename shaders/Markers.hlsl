// M6j: the first GA-product-buffer plugin, end to end. The CPU builds one PGA MOTOR per tide
// station (Pga.h -- placing and orienting the marker on the planet is data organization, the
// CPU's job), publishes {motor, scale+color} through the Exchange, and THIS shader applies the
// sandwich (GA.hlsli MotorPoint -- the same formulas) to instance a pylon: GA on the CPU for
// organization, GA on the GPU for rendering, one buffer between them. The pattern a physics
// plugin will use for real geometry later (vessels, debris, gauge widgets).
#define GA_NO_FIELD_BUFFER
#include "Common.hlsli"

cbuffer MarkerCb : register(b1) {
    float4 gMkParams;   // x = instance count (informational), y = global brightness
};

struct MarkerRec {      // one element of the GA product buffer (Exchange "markers.stations")
    float4 re;          // motor real part: s, r23, r31, r12
    float4 du;          // motor dual part: q, t01, t02, t03
    float4 scaleColor;  // x = half-width m, y = height m, zw = packed color (z = idx)
};
StructuredBuffer<MarkerRec> gMarkers : register(t0, space0);

struct VsOut {
    float4 pos : SV_Position;
    float3 col : COLOR0;
    float3 n : NORMAL0;
};

static const float3 kCorners[8] = {
    float3(-1, 0, -1), float3(1, 0, -1), float3(1, 0, 1), float3(-1, 0, 1),
    float3(-1, 1, -1), float3(1, 1, -1), float3(1, 1, 1), float3(-1, 1, 1)
};
// CLOCKWISE SEEN FROM OUTSIDE, which is D3D's front face -- and the table used to be the other
// way round. Culling BACK then removed every face turned toward the camera and kept only the far
// walls, so a box drew as its own inside and anything sitting within one showed straight through
// (the RHIB's ballast, visible through its hull). The rule, checkable by hand: with the
// LookToLH view and this projection there is no mirror anywhere in the chain, so a triangle winds
// clockwise on screen exactly when (v1 - v0) x (v2 - v0) points TOWARD the eye -- i.e. every
// triangle here must have that cross product along its OUTWARD normal, kFaceN[tri]. Swapping the
// last two indices of each triangle is the whole fix; kFaceN is per-triangle and does not move.
static const uint kIdx[36] = {0,5,1, 0,4,5, 1,6,2, 1,5,6, 2,7,3, 2,6,7,
                              3,4,0, 3,7,4, 4,6,5, 4,7,6, 0,1,2, 0,2,3};
static const float3 kFaceN[12] = {
    float3(0,0,-1), float3(0,0,-1), float3(1,0,0), float3(1,0,0),
    float3(0,0,1),  float3(0,0,1),  float3(-1,0,0), float3(-1,0,0),
    float3(0,1,0),  float3(0,1,0),  float3(0,-1,0), float3(0,-1,0)
};
static const float3 kPalette[6] = {
    float3(1.0, 0.55, 0.1), float3(0.2, 0.8, 1.0), float3(0.9, 0.2, 0.6),
    float3(0.3, 1.0, 0.4), float3(1.0, 0.9, 0.2), float3(0.8, 0.5, 1.0)
};

VsOut VsMain(uint vid : SV_VertexID) {
    const uint inst = vid / 36u;
    const uint tri = (vid % 36u) / 3u;
    const MarkerRec m = gMarkers[inst];
    const float3 local = kCorners[kIdx[vid % 36u]] *
                         float3(m.scaleColor.x, m.scaleColor.y, m.scaleColor.x);
    const float3 world = MotorPoint(m.re, m.du, local);   // the sandwich, on the GPU
    VsOut o;
    o.n = MotorDir(m.re, kFaceN[tri]);
    o.col = kPalette[uint(m.scaleColor.z) % 6u] * gMkParams.y;
    o.pos = mul(float4(world - gEyeRel.xyz, 1.0f), gViewProj);
    return o;
}

float4 PsMain(VsOut i) : SV_Target {
    const float ndl = saturate(dot(normalize(i.n), gSunDir.xyz));
    return float4(i.col * (0.25f + 0.9f * ndl), 1.0f);
}
