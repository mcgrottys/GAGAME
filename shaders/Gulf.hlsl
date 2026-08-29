// M3: the Gulf of Maine as a live map. Background = GoMOFS surface-current speed; the coloured
// blooms are Okubo-Weiss-negative regions -- rotation-dominated water -- tinted by the SIGN of
// the vorticity bivector: violet = cyclonic (counter-clockwise), amber = anticyclonic. Land is
// the model's own mask. The ring marks buoy 44029 (NERACOOS A01), whose ADCP is the ground
// truth the title bar compares against.
#include "Common.hlsli"

cbuffer GulfCb : register(b1) {
    float4 gPanel;    // xy = centre ndc, zw = half extents ndc
    uint4  gSrvs;     // x = uv field, y = Mv2 field, z = Okubo-Weiss field
    float4 gParams;   // x maxSpeed m/s, y |OW| threshold 1/s^2, z brightness, w graticule deg
    float4 gGeo;      // lon0, lat0, lonSpan, latSpan
    float4 gMarker;   // xy = buoy uv, z = ring radius (uv units), w = enabled
};

struct VsOut {
    float4 pos : SV_Position;
    float2 uv : TEXCOORD0;    // x east, y north (row 0 = southern edge)
};

VsOut VsMain(uint vid : SV_VertexID) {
    const float2 c[6] = {float2(-1, -1), float2(1, -1), float2(-1, 1),
                         float2(1, -1), float2(1, 1), float2(-1, 1)};
    VsOut o;
    const float2 p = gPanel.xy + c[vid] * gPanel.zw;
    o.pos = float4(p, 0.5f, 1.0f);
    o.uv = c[vid] * 0.5f + 0.5f;
    return o;
}

float3 SpeedRamp(float t) {
    const float3 deep = float3(0.015f, 0.045f, 0.11f);
    const float3 mid  = float3(0.06f, 0.38f, 0.45f);
    const float3 hot  = float3(0.95f, 0.82f, 0.35f);
    return (t < 0.5f) ? lerp(deep, mid, t * 2.0f) : lerp(mid, hot, t * 2.0f - 1.0f);
}

float4 PsMain(VsOut i) : SV_Target {
    const float4 f = gTex[gSrvs.x].SampleLevel(sLinearClamp, i.uv, 0);
    float3 col;
    if (f.z < 0.5f) {
        col = float3(0.10f, 0.115f, 0.13f);   // land, from the model's own mask
    } else {
        const float speed = length(f.xy);
        col = SpeedRamp(saturate(speed / gParams.x));

        // The GA payoff: Okubo-Weiss < 0 marks rotation-dominated water; the bivector's sign
        // says which way it turns.
        const float ow = gTex[gSrvs.z].SampleLevel(sLinearClamp, i.uv, 0).x;
        const float curl = gTex[gSrvs.y].SampleLevel(sLinearClamp, i.uv, 0).w;
        if (ow < -gParams.y) {
            const float e = saturate((-ow - gParams.y) / (8.0f * gParams.y));
            const float3 tint = (curl > 0.0f) ? float3(0.62f, 0.30f, 1.05f)
                                              : float3(1.05f, 0.52f, 0.18f);
            col = lerp(col, tint, 0.28f + 0.50f * e);
        }
    }

    // Graticule, anti-aliased in degrees.
    const float lon = gGeo.x + i.uv.x * gGeo.z;
    const float lat = gGeo.y + i.uv.y * gGeo.w;
    const float dg = gParams.w;
    const float2 d = abs(frac(float2(lon, lat) / dg + 0.5f) - 0.5f) * dg;
    const float2 aa = float2(fwidth(lon), fwidth(lat)) * 1.2f;
    const float grat = max(1.0f - smoothstep(0, aa.x, d.x), 1.0f - smoothstep(0, aa.y, d.y));
    col = lerp(col, float3(0.5f, 0.55f, 0.6f), 0.18f * grat);

    // Buoy 44029 marker ring.
    if (gMarker.w > 0.5f) {
        const float r = length((i.uv - gMarker.xy) * float2(gGeo.z / gGeo.w, 1.0f));
        const float ring = 1.0f - smoothstep(0.0035f, 0.0055f, abs(r - gMarker.z));
        col = lerp(col, float3(1.4f, 1.4f, 1.4f), ring);
    }

    return float4(col * gParams.z, 1.0f);
}
