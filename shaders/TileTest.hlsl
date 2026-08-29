// M0 self-test kernels for reserved-resource (tiled) semantics. See src/core/TileAtlas.cpp for
// the harness and the result encoding; the two must stay in step.
//
// This file deliberately does NOT include Common.hlsli: the self-test runs on its own compute
// root signature, before any renderer exists, and must not depend on the scene bindings.

cbuffer TestCb : register(b0) {
    uint gW, gH, gD;
    uint gTileW, gTileH;
    uint gTilesX, gTilesY;
    uint gPad;
};

Texture2D<float>   gSrc2D : register(t0);
Texture3D<float>   gSrc3D : register(t1);
RWTexture2D<float> gDst2D : register(u0);
RWTexture3D<float> gDst3D : register(u1);
// x = loads matching pattern, y = samples matching pattern, z = probes reported fully mapped,
// w = loadZeros | (sampleZeros << 8). Slots [0, tiles) are 2D tiles; 64/65 are the 3D probes.
RWStructuredBuffer<uint4> gResults : register(u2);

SamplerState sPoint : register(s0);

// The written pattern: nonzero everywhere and exactly representable in fp32 (integers < 2^24),
// so equality compares are legitimate and a null tile's 0.0 can never be mistaken for data.
float Pattern2D(uint x, uint y) { return 1.0f + (float)x + (float)y * (float)gW; }
float Pattern3D(uint x, uint y, uint z) {
    return 1.0f + (float)x + (float)y * 128.0f + (float)z * 128.0f * 128.0f;
}

[numthreads(8, 8, 1)]
void CsWrite2D(uint3 id : SV_DispatchThreadID) {
    if (id.x >= gW || id.y >= gH) return;
    // Writes cover EVERY texel, including NULL-mapped tiles. Tier 2 must discard those silently;
    // the read pass proves it.
    gDst2D[id.xy] = Pattern2D(id.x, id.y);
}

[numthreads(1, 1, 1)]
void CsRead2D(uint3 gid : SV_GroupID) {
    const uint tileX = gid.x, tileY = gid.y;
    const uint bx = tileX * gTileW, by = tileY * gTileH;

    // Five probes per tile: the four corners and the centre.
    const uint2 probes[5] = {
        uint2(bx, by),
        uint2(bx + gTileW - 1, by),
        uint2(bx, by + gTileH - 1),
        uint2(bx + gTileW - 1, by + gTileH - 1),
        uint2(bx + gTileW / 2, by + gTileH / 2),
    };

    uint loadPat = 0, samplePat = 0, mappedCnt = 0, loadZero = 0, sampleZero = 0;
    for (uint i = 0; i < 5; ++i) {
        const uint2 p = probes[i];
        const float expect = Pattern2D(p.x, p.y);

        uint statusL = 0;
        const float vL = gSrc2D.Load(int3(p, 0), int2(0, 0), statusL);
        uint statusS = 0;
        const float2 uv = (float2(p) + 0.5f) / float2(gW, gH);
        const float vS = gSrc2D.SampleLevel(sPoint, uv, 0, int2(0, 0), statusS);

        if (vL == expect) ++loadPat;
        if (vS == expect) ++samplePat;
        if (vL == 0.0f) ++loadZero;
        if (vS == 0.0f) ++sampleZero;
        // Both access paths must agree on residency; count the probe as mapped only if they do.
        if (CheckAccessFullyMapped(statusL) && CheckAccessFullyMapped(statusS)) ++mappedCnt;
    }
    gResults[tileY * gTilesX + tileX] = uint4(loadPat, samplePat, mappedCnt,
                                              loadZero | (sampleZero << 8));
}

[numthreads(4, 4, 4)]
void CsWrite3D(uint3 id : SV_DispatchThreadID) {
    if (id.x >= gW || id.y >= gH || id.z >= gD) return;
    gDst3D[id] = Pattern3D(id.x, id.y, id.z);
}

[numthreads(1, 1, 1)]
void CsRead3D(uint3 gid : SV_GroupID) {
    // One mapped probe (inside the single mapped origin tile) and one deep in NULL territory.
    {
        const uint3 p = uint3(1, 1, 1);
        uint statusL = 0;
        const float vL = gSrc3D.Load(int4(p, 0), int3(0, 0, 0), statusL);
        uint statusS = 0;
        const float3 uv = (float3(p) + 0.5f) / float3(gW, gH, gD);
        const float vS = gSrc3D.SampleLevel(sPoint, uv, 0, int3(0, 0, 0), statusS);
        const float expect = Pattern3D(p.x, p.y, p.z);
        gResults[64] = uint4(vL == expect ? 1u : 0u, vS == expect ? 1u : 0u,
                             (CheckAccessFullyMapped(statusL) && CheckAccessFullyMapped(statusS))
                                 ? 1u : 0u,
                             42u);
    }
    {
        const uint3 p = uint3(100, 100, 60);
        uint statusL = 0;
        const float vL = gSrc3D.Load(int4(p, 0), int3(0, 0, 0), statusL);
        uint statusS = 0;
        const float3 uv = (float3(p) + 0.5f) / float3(gW, gH, gD);
        const float vS = gSrc3D.SampleLevel(sPoint, uv, 0, int3(0, 0, 0), statusS);
        gResults[65] = uint4(vL == 0.0f ? 1u : 0u, vS == 0.0f ? 1u : 0u,
                             (CheckAccessFullyMapped(statusL) || CheckAccessFullyMapped(statusS))
                                 ? 1u : 0u,
                             42u);
    }
}
