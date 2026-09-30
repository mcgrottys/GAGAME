// The residency manager's gate, part 4 (src/hal/ResidencyTest.cpp, FloorProbe): THE M6h SCENARIO,
// MEASURED. A reserved array whose resident tiles hold 1.0 at every resident mip and whose
// neighbours are NULL (tier 2: they read 0), sampled across the frontier between them through
// PageSample with the clamp from PageHave -- the engine's own calls, included rather than copied,
// so the probe reads through exactly what ships -- by the engine's two samplers. Any admixture of
// a NULL tile's zero shows as a red below 1. The harness's row geometry and its reading of the
// target must stay in step with this file.
//
// The law in effect is PageSample.hlsli's PageHave. The floor law's read (Residency.h: the floor
// map read bilinear) is not staged, so it is written here, as it would ship, for the gate to hold
// against the law in effect.
//
// PIXEL STAGE ON PURPOSE: PageSample is Sample() with a min-LOD clamp, which needs derivatives,
// and the footprint is the question. One QUAD is one point of a row: quad k stands at gStart + k
// gStep, and its four pixels sit half a footprint either side of it, so the derivatives Sample
// takes are exactly the row's Jacobian (gJx, gJy) -- the footprint is set independently of where
// the row walks, which is what lets a row walk a sixteenth of a cell at a time under a footprint
// of several cells.
#include "PageSample.hlsli"

cbuffer FloorCb : register(b0) {
    uint gArr;      // the probe array's slot: its Texture2DArray view, or (gCube) its cube view
    uint gMap;      // the residency map's slot, the same kind of view: the true map or a floor
    uint gSlice;    // the slice; under gCube, the face whose plane the row walks
    uint gLaw;      // 0 PageHave, the law in effect; 1 the floor's read (HaveFloor); 2 gFixed
    uint gAniso;    // 0 the trilinear sampler (the shared layout's s0); 1 its anisotropic s3
    uint gCube;     // 1: through the cube views (PageHaveCube, PageSampleCube)
    float2 gStart;  // uv of quad 0 -- a face's uv under gCube, and past [0, 1] it crosses the edge
    float2 gStep;   // uv from one quad to the next
    float2 gJx;     // d uv / d x: the footprint
    float2 gJy;     // d uv / d y
    float gFixed;   // gLaw 2: the clamp itself (0: no clamp; the fill's check and the mechanism)
};

Texture2DArray gTexArr[] : register(t0, space5);         // the heap as arrays, as Common.hlsli has it
TextureCubeArray gTexCubeArr[] : register(t0, space6);   // ...and as cube arrays
SamplerState sTrilinear : register(s0);   // sLinearClamp's fields (Renderer.cpp's s0)
SamplerState sAniso : register(s3);       // sAniso's fields (s3: ANISOTROPIC, CLAMP, 8x)

// One triangle over the whole viewport; the harness points the viewport at one row per draw.
float4 VsFloor(uint id : SV_VertexID) : SV_Position {
    const float2 t = float2((id << 1) & 2, id & 2);
    return float4(t * float2(2.0f, -2.0f) + float2(-1.0f, 1.0f), 0.0f, 1.0f);
}

// ComposeCubeDir (Lattice.cpp) without the normalization: the cube sampler takes any length, and a
// direction affine in uv keeps the derivatives the row's own.
float3 CubeDir(uint face, float2 uv) {
    const float s = uv.x * 2.0f - 1.0f, t = uv.y * 2.0f - 1.0f;
    switch (face) {
        case 0: return float3(1.0f, -t, -s);
        case 1: return float3(-1.0f, -t, s);
        case 2: return float3(s, 1.0f, t);
        case 3: return float3(s, -1.0f, -t);
        case 4: return float3(s, -t, 1.0f);
        default: return float3(-s, -t, -1.0f);
    }
}

// The floor law's read as it would ship in PageSample.hlsli: the map read BILINEAR, which is
// sound only over the floor map (read over the true map it is M6h, the probe's planted failure).
float HaveFloor(float2 uv) {
    return gTexArr[gMap].SampleLevel(sTrilinear, float3(uv, gSlice), 0.0f).x * 255.0f / 16.0f;
}
float HaveFloorCube(float3 dir) {
    return gTexCubeArr[gMap].SampleLevel(sTrilinear, float4(dir, 0.0f), 0.0f).x * 255.0f / 16.0f;
}

// gLaw, gAniso and gCube are root constants: every branch is uniform over the quad, so the
// derivatives are the row's own on both sides of it.
//
// TWO VERDICTS A SAMPLE. The value: a tap on a NULL texel pulls the red below 1 by its weight.
// And the hardware's own: the same Sample call once more with its status out, asked
// CheckAccessFullyMapped -- whether ANY texel the footprint touched lay in an unmapped tile,
// whatever its weight. The value is the brief's measure; the status does not depend on the
// filter's weights summing to exactly one, which nothing promises for an anisotropic footprint.
float4 PsFloor(float4 pos : SV_Position) : SV_Target {
    const uint2 p = uint2(pos.xy);
    const float2 uv = gStart + float(p.x >> 1) * gStep + (float(p.x & 1u) - 0.5f) * gJx +
                      (float(p.y & 1u) - 0.5f) * gJy;
    float have = gFixed, lod = 0.0f;
    float4 c = 0.0f;
    uint status = 0u;
    if (gCube != 0u) {
        const float3 d = CubeDir(gSlice, uv);
        if (gLaw == 0u) have = PageHaveCube(gTexCubeArr[gMap], sTrilinear, d);
        else if (gLaw == 1u) have = HaveFloorCube(d);
        if (gAniso != 0u) {
            c = PageSampleCube(gTexCubeArr[gArr], sAniso, d, have);
            c.a = gTexCubeArr[gArr].Sample(sAniso, float4(d, 0.0f), have, status).a;
            lod = gTexCubeArr[gArr].CalculateLevelOfDetailUnclamped(sAniso, d);
        } else {
            c = PageSampleCube(gTexCubeArr[gArr], sTrilinear, d, have);
            c.a = gTexCubeArr[gArr].Sample(sTrilinear, float4(d, 0.0f), have, status).a;
            lod = gTexCubeArr[gArr].CalculateLevelOfDetailUnclamped(sTrilinear, d);
        }
    } else {
        if (gLaw == 0u) have = PageHave(gTexArr[gMap], sTrilinear, uv, gSlice);
        else if (gLaw == 1u) have = HaveFloor(uv);
        if (gAniso != 0u) {
            c = PageSample(gTexArr[gArr], sAniso, uv, gSlice, have);
            c.a = gTexArr[gArr].Sample(sAniso, float3(uv, gSlice), int2(0, 0), have, status).a;
            lod = gTexArr[gArr].CalculateLevelOfDetailUnclamped(sAniso, uv);
        } else {
            c = PageSample(gTexArr[gArr], sTrilinear, uv, gSlice, have);
            c.a = gTexArr[gArr].Sample(sTrilinear, float3(uv, gSlice), int2(0, 0), have, status).a;
            lod = gTexArr[gArr].CalculateLevelOfDetailUnclamped(sTrilinear, uv);
        }
    }
    // r: the sample through PageSample (1 where every tap was resident); g: the clamp; b: the
    // hardware's own LOD before any clamp; a: 1 when the hardware says every texel the footprint
    // touched was mapped, 0 when it touched a NULL tile.
    return float4(c.r, have, lod, CheckAccessFullyMapped(status) ? 1.0f : 0.0f);
}
