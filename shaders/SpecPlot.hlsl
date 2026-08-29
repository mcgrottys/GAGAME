// M2's visible gate, panel two: wave spectral density S(f). Solid = the model spectrum the FFT
// cascades were synthesised from (GFS-Wave partitions through the shared parameterization);
// dashed = buoy 44013's MEASURED spectral density. Same trick as TideCurves: everything evaluated
// in the vertex shader from per-frame constants, NaN positions cull dead segments.
#include "Common.hlsli"

cbuffer SpecCb : register(b1) {
    float4 gRect;     // ndc x0, y0(bottom), x1, y1(top)
    float4 gAxis;     // x fMax Hz, y sMax m^2/Hz, z nSamples, w hasBuoy (0/1)
    float4 gColM;     // model colour
    float4 gColB;     // buoy colour
    float4 gModelS[24];   // 96 samples of model S(f), packed 4-wide
    float4 gBuoyS[24];    // 96 samples of measured S(f); negative = missing
};

static const uint kVerts = 96;

float SampleOf(float4 arr[24], uint k) { return arr[k >> 2][k & 3]; }

struct VsOut {
    float4 pos : SV_Position;
    nointerpolation float4 col : COLOR0;
};

VsOut VsMain(uint vid : SV_VertexID, uint inst : SV_InstanceID) {
    VsOut o;
    const float u = vid / (float)(kVerts - 1);
    float x = lerp(gRect.x, gRect.z, u);
    float s = 0.0f;
    bool dead = false;
    float4 col = float4(1, 1, 1, 1);

    if (inst == 0) {
        s = SampleOf(gModelS, min(vid, kVerts - 1));
        col = gColM;
    } else if (inst == 1) {
        s = SampleOf(gBuoyS, min(vid, kVerts - 1));
        if (s < 0.0f || gAxis.w < 0.5f) dead = true;
        if ((vid % 6) >= 4) dead = true;   // dashes
        col = gColB;
    } else {
        // baseline (S = 0)
        dead = (vid >= 2);
        x = (vid == 0) ? gRect.x : gRect.z;
        s = 0.0f;
        col = float4(0.45f, 0.45f, 0.52f, 1);
    }

    const float t01 = saturate(s / max(gAxis.y, 1e-4f));
    o.pos = float4(x, lerp(gRect.y, gRect.w, t01), 0.5f, 1.0f);
    if (dead) o.pos = asfloat(0x7fc00000u).xxxx;
    o.col = col;
    return o;
}

float4 PsMain(VsOut i) : SV_Target {
    return float4(i.col.rgb, 1.0f);
}
