// HIERARCHY 4.17 commit 4: the directory walk on the GPU, for the selftest's readback (the
// harness is ProbeWalk in src/hal/TileAtlas.cpp, which compiles this with GA_BLOCK_RANKS=5). Each
// pixel runs Walk.hlsli's own body -- the one Compose.hlsli runs -- for one point and writes one
// step of its chain: point j's five steps at x = 5 j to 5 j + 4 of its row, each (slice, u, v,
// steps), slice -1 past the chain's end.
#include "PageSample.hlsli"

cbuffer WalkCb : register(b0) {
    uint gPts;     // the points (RGBA32F): xyz relative to the eye, in the rows' tangent frame
    uint gDirs;    // their directions (RGBA32F), planet frame
    uint gRows;    // the blocks' rows (RGBA32F, 8 x 4): U, V, W a row each, then the offsets
    uint gDirTex;  // the directory (R16_UINT), through the heap's uint view
    uint gShift;   // how far below its points' rows this draw's target rows sit
};

Texture2D<float4> gTex[] : register(t0, space1);   // the heap
Texture2D<uint> gTexUW[] : register(t0, space2);   // the heap again, as uint

// Block i's uv at p by this harness's rows -- Compose.hlsli's CsBlockUv, the same arithmetic.
float2 WalkRowsUv(uint i, float3 p) {
    const float4 o = gTex[gRows].Load(int3(int(i >> 1), 3, 0));
    return PageTexelUv(p, gTex[gRows].Load(int3(int(i), 0, 0)), gTex[gRows].Load(int3(int(i), 1, 0)),
                       gTex[gRows].Load(int3(int(i), 2, 0))) +
           ((i & 1u) != 0u ? o.zw : o.xy);
}
#define WALK_UV(i, p) WalkRowsUv(i, p)
#include "Walk.hlsli"

float4 VsWalk(uint id : SV_VertexID) : SV_Position {
    const float2 t = float2((id << 1) & 2, id & 2);
    return float4(t * float2(2.0f, -2.0f) + float2(-1.0f, 1.0f), 0.0f, 1.0f);
}

float4 PsWalk(float4 pos : SV_Position) : SV_Target {
    const int2 px = int2(pos.xy);
    const int2 at = int2(px.x / 5, px.y - int(gShift));
    const uint k = uint(px.x % 5);
    const float3 p = gTex[gPts].Load(int3(at, 0)).xyz;
    const float3 d = gTex[gDirs].Load(int3(at, 0)).xyz;
    const WalkChain w = Walk(d, p, gTexUW[gDirTex]);
    return k < w.n ? float4(float(WalkSlice(w, k)), WalkUv(w, k), float(w.n))
                   : float4(-1.0f, 0.0f, 0.0f, float(w.n));
}
