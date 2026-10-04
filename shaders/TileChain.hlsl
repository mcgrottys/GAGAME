// PHASE A1: the eye's chain on the GPU, for the selftest's readback (the harness is ProbeChain in
// src/hal/TileAtlas.cpp). Each pixel runs Window.hlsli's own body -- the one Compose.hlsli runs --
// for one point and writes one rank of its chain: point j's five ranks at x = 5 j to 5 j + 4 of
// its row, each (slice, u, v, chain length) -- every rank's address, live or not.
#include "PageSample.hlsli"

cbuffer ChainCb : register(b0) {
    uint gPts;     // the points (RGBA32F): xyz relative to the eye, in the rows' tangent frame
    uint gRows;    // the windows' rows (RGBA32F, 5 x 4): U, V, W a row each, then (off.xy, slice, K)
    uint gShift;   // how far below its points' rows this draw's target rows sit
    uint gPad;
};

Texture2D<float4> gTex[] : register(t0, space1);   // the heap

float2 ChainRowsUv(uint i, float3 p) {
    return PageTexelUv(p, gTex[gRows].Load(int3(int(i), 0, 0)), gTex[gRows].Load(int3(int(i), 1, 0)),
                       gTex[gRows].Load(int3(int(i), 2, 0)));
}
#define WIN_UV(s, i, p) ChainRowsUv(i, p)
#define WIN_OFF(s, i) gTex[gRows].Load(int3(int(i), 3, 0)).xy
#define WIN_SLICE(s, i) uint(gTex[gRows].Load(int3(int(i), 3, 0)).z)
#define WIN_K(s) uint(gTex[gRows].Load(int3(0, 3, 0)).w)
#include "Window.hlsli"

float4 VsChain(uint id : SV_VertexID) : SV_Position {
    const float2 t = float2((id << 1) & 2, id & 2);
    return float4(t * float2(2.0f, -2.0f) + float2(-1.0f, 1.0f), 0.0f, 1.0f);
}

float4 PsChain(float4 pos : SV_Position) : SV_Target {
    const int2 px = int2(pos.xy);
    const int2 at = int2(px.x / 5, px.y - int(gShift));
    const uint k = uint(px.x % 5);
    const float3 p = gTex[gPts].Load(int3(at, 0)).xyz;
    const WalkChain w = WindowChain(p, 0u);
    return float4(float(WalkSlice(w, k)), WalkUv(w, k), float(w.n));
}
