// HIERARCHY step 3: the address of a face-plane window, on the GPU. Each pixel Loads one point --
// spacetest's helm sample (core/Lattice.h FaceWindowHelmSample), relative to the eye, float32 --
// and writes PageSample.hlsli's PageTexel and PageTexelUv of it: the engine's own functions,
// included rather than copied, so the gate judges the spelling the windows will read. The
// harness is the address block after step 0 in src/hal/TileAtlas.cpp; the constants below and
// its draws must stay in step.
//
// PIXEL STAGE, where the windows will be read. A float target keeps the texel's own bits, and a
// Load reads the bindless heap in any stage.
#include "PageSample.hlsli"

cbuffer TexelCb : register(b0) {
    float4 gPlaneU;   // FaceWindow::PlanesIn's three rows -- or the plant's
    float4 gPlaneV;
    float4 gPlaneW;
    uint gSrv;        // the points' slot in the heap
    uint gShift;      // how far below its points' rows this draw's target rows sit
};

Texture2D<float4> gTex[] : register(t0, space1);   // the heap, as Common.hlsli has it

// One triangle over the whole viewport; the harness points the viewport at one block of rows.
float4 VsTexel(uint id : SV_VertexID) : SV_Position {
    const float2 t = float2((id << 1) & 2, id & 2);
    return float4(t * float2(2.0f, -2.0f) + float2(-1.0f, 1.0f), 0.0f, 1.0f);
}

float4 PsTexel(float4 pos : SV_Position) : SV_Target {
    const float3 p = gTex[gSrv].Load(int3(int2(pos.xy) - int2(0, int(gShift)), 0)).xyz;
    return float4(PageTexel(p, gPlaneU, gPlaneV, gPlaneW),
                  PageTexelUv(p, gPlaneU, gPlaneV, gPlaneW));
}
