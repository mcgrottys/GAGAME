// M5: the land and the bed -- the CUDEM topobathy surface at true scale. The jetties, the bar,
// the dunes and the marsh are all THIS surface; the sea layer draws over it wherever the water
// stands higher. Materials come from elevation relative to the CURRENT water level plus slope
// (steep + low = riprap: the jetties shade themselves correctly at any tide).
#include "Common.hlsli"

cbuffer TerrainCb : register(b1) {
    float4 gTGeo;      // world x0, z0, sizeX, sizeZ
    uint4  gTSrv;      // x = heightfield SRV, y = grid quads X, z = grid quads Z, w = unused
    float4 gTParams;   // x = water level (NAVD88 m), y = texel world size, zw = unused
};

struct VsOut {
    float4 pos : SV_Position;
    float3 rel : TEXCOORD0;
    float2 uv : TEXCOORD1;
    float elev : TEXCOORD2;
};

float HeightAt(float2 uv) {
    // Row 0 of the grid is NORTH; v = 0 at the south edge in world terms, so flip.
    return gTex[gTSrv.x].SampleLevel(sLinearClamp, float2(uv.x, 1.0f - uv.y), 0).x;
}

VsOut VsMain(uint vid : SV_VertexID) {
    VsOut o;
    const uint qx = gTSrv.y, qz = gTSrv.z;
    const uint quad = vid / 6;
    const uint corner = vid % 6;
    const uint2 cc[6] = {uint2(0,0), uint2(1,0), uint2(0,1), uint2(1,0), uint2(1,1), uint2(0,1)};
    const uint ix = quad % qx;
    const uint iz = quad / qx;
    const float2 uv = float2(ix + cc[corner].x, iz + cc[corner].y) / float2(qx, qz);
    float h = HeightAt(uv);
    if (h < -9000.0f) h = -35.0f;   // nodata: drop to deep floor, far below every camera
    const float3 world = float3(gTGeo.x + uv.x * gTGeo.z, h, gTGeo.y + uv.y * gTGeo.w);
    o.uv = uv;
    o.elev = h;
    o.rel = world - gEyeRel.xyz;
    o.pos = mul(float4(o.rel, 1.0f), gViewProj);
    return o;
}

float4 PsMain(VsOut i) : SV_Target {
    // Normal from the heightfield: central differences one texel apart, world-scaled.
    const float2 du = float2(gTParams.y / gTGeo.z, 0);
    const float2 dv = float2(0, gTParams.y / gTGeo.w);
    const float hx = (HeightAt(i.uv + du) - HeightAt(i.uv - du)) / (2.0f * gTParams.y);
    const float hz = (HeightAt(i.uv + dv) - HeightAt(i.uv - dv)) / (2.0f * gTParams.y);
    const float3 n = normalize(float3(-hx, 1.0f, -hz));
    const float slope = length(float2(hx, hz));

    const float water = gTParams.x;
    const float above = i.elev - water;

    // Materials, water-level-relative: submerged mud, wet sand at the waterline, dry sand,
    // vegetation above the reach of the tide, riprap on anything steep.
    float3 albedo;
    if (above < -0.3f) {
        albedo = float3(0.16f, 0.15f, 0.12f);                  // submerged bed (sea covers this)
    } else if (above < 0.35f) {
        albedo = float3(0.38f, 0.34f, 0.27f);                  // wet sand band
    } else if (i.elev < 3.6f) {
        albedo = float3(0.70f, 0.64f, 0.50f);                  // beach and flats
    } else {
        albedo = float3(0.28f, 0.37f, 0.20f);                  // dune grass / marsh / upland
    }
    if (slope > 0.42f && i.elev > water - 1.5f) {
        albedo = lerp(albedo, float3(0.36f, 0.35f, 0.34f), saturate((slope - 0.42f) * 3.0f));
    }

    const float ndl = saturate(dot(n, gSunDir.xyz));
    float3 col = albedo * (SUN_IRR_C * ndl + SkyRadiance(n.y) * 0.55f);

    col = AerialPerspective(col, normalize(i.rel), length(i.rel));
    return float4(col, 1.0f);
}
