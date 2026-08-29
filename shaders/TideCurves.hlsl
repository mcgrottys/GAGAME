// M1's visible gate: each station's tide over the scrub window. Instances 0..7 draw OUR analytic
// sum-of-cosines (solid); instances 8..15 draw NOAA's official predictions (dashed, dimmed);
// 16 is the 'now' cursor; 17 the MLLW zero line. The solid curve lying exactly on the dashed one
// is the acceptance test, rendered every frame.
//
// Everything is evaluated in the vertex shader from per-frame constants: theta at the window
// centre comes in fp64-reduced from the CPU, so fp32 only ever sees omega * dt with small dt.
#include "Common.hlsli"

cbuffer CurvesCb : register(b1) {
    float4 gWin;     // x windowSec, y yMin m, z yMax m, w nStations
    float4 gRect;    // ndc x0, y0(bottom), x1, y1(top)
    float4 gCMisc;   // x nCoeffMax, y officialN, z focusStation, w unused
    float4 gCol[8];
    float4 gMean[2];
    float4 gNCo[2];
    float4 gCoef[320];   // [s*40 + c] = (amp m, omega rad/s, thetaAtCentre rad, 0)
    float4 gOff[256];    // [s*32 + k/4][k%4] = official sample m; -999 = outside coverage
};

static const uint kVerts = 384;

float MeanOf(uint s)  { return (s < 4) ? gMean[0][s] : gMean[1][s - 4]; }
float NCoefOf(uint s) { return (s < 4) ? gNCo[0][s] : gNCo[1][s - 4]; }
float OffSample(uint s, uint k) { return gOff[s * 32 + (k >> 2)][k & 3]; }

struct VsOut {
    float4 pos : SV_Position;
    nointerpolation float4 col : COLOR0;
};

VsOut VsMain(uint vid : SV_VertexID, uint inst : SV_InstanceID) {
    VsOut o;
    const uint nSta = (uint)gWin.w;
    const float u = vid / (float)(kVerts - 1);
    float x = lerp(gRect.x, gRect.z, u);
    float h = 0.0f;
    bool dead = false;
    float4 col = float4(1, 1, 1, 1);

    if (inst < 8) {
        // ---- analytic model curve
        const uint s = inst;
        dead = (s >= nSta);
        const float dt = (u - 0.5f) * gWin.x;
        float acc = MeanOf(s);
        const uint n = (uint)NCoefOf(s);
        [loop] for (uint c = 0; c < n; ++c) {
            const float4 k = gCoef[s * 40 + c];
            acc += k.x * cos(k.z + k.y * dt);
        }
        h = acc;
        col = gCol[s];
    } else if (inst < 16) {
        // ---- official NOAA predictions, dashed and dimmed
        const uint s = inst - 8;
        dead = (s >= nSta);
        const float pos = u * (gCMisc.y - 1.0f);
        const uint k0 = (uint)pos;
        const uint k1 = min(k0 + 1, (uint)gCMisc.y - 1);
        const float a = OffSample(s, k0);
        const float b = OffSample(s, k1);
        if (a < -900.0f || b < -900.0f) dead = true;
        h = lerp(a, b, pos - k0);
        if ((vid % 6) >= 4) dead = true;   // dashes: drop 2 of every 6 vertices
        col = float4(gCol[s].rgb * 0.40f, 1);
    } else if (inst == 16) {
        // ---- 'now' cursor at the window centre
        dead = (vid >= 2);
        x = lerp(gRect.x, gRect.z, 0.5f);
        h = (vid == 0) ? gWin.y : gWin.z;
        col = float4(1.6f, 1.6f, 1.6f, 1);
    } else {
        // ---- MLLW zero line
        dead = (vid >= 2);
        x = (vid == 0) ? gRect.x : gRect.z;
        h = 0.0f;
        col = float4(0.45f, 0.45f, 0.52f, 1);
    }

    const float t01 = saturate((h - gWin.y) / max(gWin.z - gWin.y, 1e-3f));
    o.pos = float4(x, lerp(gRect.y, gRect.w, t01), 0.5f, 1.0f);
    if (dead) {
        // NaN positions cull every line segment touching this vertex -- the standard trick for
        // per-vertex kill in a line strip without an index buffer.
        o.pos = asfloat(0x7fc00000u).xxxx;
    }
    o.col = col;
    return o;
}

float4 PsMain(VsOut i) : SV_Target {
    return float4(i.col.rgb, 1.0f);
}
