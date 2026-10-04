// M1: the lower Merrimack as a data sculpture. X = along-channel distance (100 scene metres per
// river km), height = the analytic tide interpolated between harmonic stations, exaggerated.
// Scrubbed fast, the tidal wave visibly propagates upriver -- the station phase lags are real.
//
// mode 0 draws the water ribbon (a vertex grid from SV_VertexID -- no vertex buffer, the vqview
// pattern); mode 1 draws one coloured pylon per station, whose top rides its station's tide.
#include "Common.hlsli"

cbuffer TideCb : register(b1) {
    float4 gRib;        // x sceneLenM, y halfWidthM, z heightExagg, w nRibbonStations
    float4 gStaX[2];    // station scene X, metres
    float4 gStaH[2];    // station tide height, metres MLLW
    float4 gRMisc;      // x mode (0 surface, 1 pylons), y contourStep m, z focusX m, w unused
    float4 gStaColor[8];
};

static const uint kQX = 384;
static const uint kQZ = 24;

float StaX(uint i) { return (i < 4) ? gStaX[0][i] : gStaX[1][i - 4]; }
float StaH(uint i) { return (i < 4) ? gStaH[0][i] : gStaH[1][i - 4]; }

// Piecewise-linear tide height along the channel, clamped at both ends.
float HeightAt(float x) {
    const uint n = (uint)gRib.w;
    if (n == 0) return 0.0f;
    if (x <= StaX(0)) return StaH(0);
    [loop] for (uint i = 0; i + 1 < n; ++i) {
        const float x1 = StaX(i + 1);
        if (x <= x1) {
            const float x0 = StaX(i);
            const float t = saturate((x - x0) / max(x1 - x0, 1e-3f));
            return lerp(StaH(i), StaH(i + 1), t);
        }
    }
    return StaH(n - 1);
}

struct VsOut {
    float4 pos : SV_Position;
    float3 rel : TEXCOORD0;                    // camera-relative position
    float3 n   : TEXCOORD1;
    float  hM  : TEXCOORD2;                    // metres MLLW, for contours
    nointerpolation float4 col : TEXCOORD3;    // a > 0.5 marks a pylon
};

// Unit cube as 12 triangles; +ve winding irrelevant (cull off).
static const uint3 kCubeTris[12] = {
    uint3(0,1,2), uint3(1,3,2),   // -z
    uint3(4,6,5), uint3(5,6,7),   // +z
    uint3(0,4,1), uint3(1,4,5),   // -y
    uint3(2,3,6), uint3(3,7,6),   // +y
    uint3(0,2,4), uint3(2,6,4),   // -x
    uint3(1,5,3), uint3(3,5,7),   // +x
};

VsOut VsMain(uint vid : SV_VertexID, uint inst : SV_InstanceID) {
    VsOut o;
    const float exagg = gRib.z;
    float3 world;

    if (gRMisc.x < 0.5f) {
        // ---- water ribbon
        const uint quad = vid / 6;
        const uint corner = vid % 6;
        const uint2 c[6] = {uint2(0,0), uint2(1,0), uint2(0,1), uint2(1,0), uint2(1,1), uint2(0,1)};
        const uint qx = quad % kQX;
        const uint qz = quad / kQX;
        const float u = (qx + c[corner].x) / (float)kQX;
        const float v = (qz + c[corner].y) / (float)kQZ;
        const float x = u * gRib.x;
        const float h = HeightAt(x);
        world = float3(x, h * exagg, (v - 0.5f) * 2.0f * gRib.y);
        const float dx = gRib.x / kQX;
        const float dhdx = (HeightAt(x + dx) - HeightAt(x - dx)) / (2.0f * dx);
        o.n = normalize(float3(-dhdx * exagg, 1.0f, 0.0f));
        o.hM = h;
        o.col = float4(0, 0, 0, 0);
    } else {
        // ---- station pylons: a slender box whose top surfs its station's tide
        const uint tri = vid / 3;
        const uint corner = kCubeTris[tri][vid % 3];
        const float3 unitc = float3((corner >> 0) & 1, (corner >> 2) & 1, (corner >> 1) & 1);
        const float x = StaX(inst);
        const float top = StaH(inst) * exagg + 10.0f;   // poke above the surface so it reads
        const float bottom = -0.4f * exagg;
        const float hw = 11.0f;   // pylon half-width, metres ('half' is a reserved HLSL type)
        world = float3(x - hw + unitc.x * 2.0f * hw,
                       lerp(bottom, top, unitc.y),
                       -hw + unitc.z * 2.0f * hw);
        o.n = float3(0, 1, 0);
        o.hM = 0;
        o.col = gStaColor[inst];
    }

    o.rel = world - gEyeRel.xyz;
    o.pos = mul(float4(o.rel, 1.0f), gViewProj);
    return o;
}

float4 PsMain(VsOut i) : SV_Target {
    if (i.col.a > 0.5f) {
        return float4(i.col.rgb, 1.0f);
    }
    const float3 v = normalize(-i.rel);
    const float3 n = normalize(i.n);

    // Deep-water colour from the shared optics: b/sigma is the asymptotic colour, same six
    // numbers vqview calibrated for the Merrimack. No colour ramp.
    const float3 deep = gBscat.rgb / max(gSigmaW.rgb, 1e-4f);
    const float ndl = saturate(dot(n, gSunDir.xyz));
    const float3 upR = float3(0.0f, 1.0f, 0.0f);
    float3 col = deep * (SkyAmbient(n, upR, 0.0f) + 0.65f * ndl * SunAt(upR, 0.0f));

    // Fresnel-weighted sky reflection, from the one shared sky model.
    const float3 r = reflect(-v, n);
    const float f = 0.02f + 0.98f * pow(1.0f - saturate(dot(n, v)), 5.0f);
    col = lerp(col, SkyRadianceDir(r), f);

    // Height contours every contourStep metres above MLLW: the ribbon doubles as its own gauge,
    // and the dark line sweeping along the river as the tide crosses each level is the point.
    // contourStep <= 0 turns the gauge off (the C key cycles it).
    if (gRMisc.y > 0.01f) {
        const float cstep = gRMisc.y;
        const float d = abs(frac(i.hM / cstep + 0.5f) - 0.5f) * cstep;   // m to nearest contour
        const float aa = max(fwidth(i.hM) * 1.5f, 0.004f);
        const float contour = 1.0f - smoothstep(0.0f, aa, d);
        col *= 1.0f - 0.45f * contour;
    }

    return float4(col, 1.0f);
}
