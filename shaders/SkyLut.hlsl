// ================================================================================================
//  SkyLut.hlsl - the air's one table, and the two --sky-probe holds against each other
//  (Atmosphere.hlsli holds the air itself).
//
//    CsMultiScatter     32 x 32   the isotropic estimate of every scattering order past the
//                                 first, summed as a geometric series (Hillaire 2020 s5).
//                                 Depends on the atmosphere alone: built ONCE.
//    CsTransmittance   256 x 64   PROBE ONLY: what survives a ray from an altitude to the top of
//    CsTransAnalytic              the air, marched (the table the sky used to read) and in the
//                                 closed form the sky reads now (Atmosphere.hlsli AtmSunT).
//  (There is no table of the VIEW. Every ray marches the air from where it is -- Atmosphere.hlsli
//  AtmRay -- and the sun's light at each step is a closed form, so the one table above, which
//  describes the air alone, is all the sky needs.)
//
//  The march is the ordinary radiative transfer integral along a ray: at each step, the light
//  the sun delivers (its transmittance to that point, times the phase functions) plus the light
//  every other order delivers (the table), attenuated by what the air between has already taken.
//  The step is integrated ANALYTICALLY rather than sampled (Hillaire's energy-conserving form,
//  S_int = (S - S e^{-sigma ds}) / sigma), which is what keeps 32 steps from banding.
// ================================================================================================
#define GA_NO_FIELD_BUFFER
#include "Common.hlsli"
#include "Atmosphere.hlsli"

cbuffer SkyLutCb : register(b0) {
    float4 gLutA;    // xyz = the sun, in the eye's own frame (+y is up there); w = eye radius r
    float4 gLutB;    // x = planet radius Rb, y = Rt, z = the solar irradiance scale, w = spare
    float4 gLutC;    // xy = this table's size in texels, zw = spare
    uint4  gLutD;    // x = the table being written (UAV slot), yzw = spare
};
#define ATM_RB (gLutB.x)
#define ATM_RT (gLutB.y)

// The heap, as this engine always reaches it: nothing names a texture, everything names a slot
// (Common.hlsli's header). gTex[] comes from there; the UAV side is the water bank's own
// spelling, u0 space2. The READS are SkyLutFetch's manual bilinear, never SampleLevel: this is
// a compute stage, where a bindless sample silently returns zero on this adapter (priors 1), and
// dxtest's sampler law fails the build if a kernel forgets.
RWTexture2D<float4> gU[] : register(u0, space2);

static const float3 kUpLocal = float3(0.0f, 1.0f, 0.0f);

// THE TRANSMITTANCE ALONG A RAY, marched -- the table the sky read before the closed form. 40 steps;
// --sky-probe measures what that bought against a brute-force integral in doubles.
float3 AtmTransmittanceTo(float r, float mu, float Rb, float Rt) {
    const float disc = r * r * (mu * mu - 1.0f) + Rt * Rt;
    const float top = max(0.0f, -r * mu + sqrt(max(disc, 0.0f)));
    const float ground = AtmRaySphere(r, mu, Rb);
    const float dist = (ground > 0.0f) ? ground : top;
    const uint kSteps = 40u;
    const float ds = dist / kSteps;
    float3 od = 0.0f;
    for (uint i = 0u; i < kSteps; ++i) {
        const float t = (i + 0.5f) * ds;
        const float rr = sqrt(max(r * r + t * t + 2.0f * r * mu * t, 0.0f));
        od += AtmExtinction(rr - Rb) * ds;
    }
    return exp(-od);
}

[numthreads(8, 8, 1)]
void CsTransmittance(uint3 id : SV_DispatchThreadID) {
    const float2 size = gLutC.xy;
    if (id.x >= uint(size.x) || id.y >= uint(size.y)) return;
    const float2 uv = (float2(id.xy) + 0.5f) / size;
    float r, mu;
    AtmTransParams(uv, ATM_RB, ATM_RT, r, mu);
    gU[gLutD.x][id.xy] = float4(AtmTransmittanceTo(r, mu, ATM_RB, ATM_RT), 1.0f);
}

// THE SAME QUANTITY AS THE SKY NOW ASKS FOR IT (Atmosphere.hlsli AtmSunT), on the same grid, so
// --sky-probe reads what this device actually computes from the closed form.
[numthreads(8, 8, 1)]
void CsTransAnalytic(uint3 id : SV_DispatchThreadID) {
    const float2 size = gLutC.xy;
    if (id.x >= uint(size.x) || id.y >= uint(size.y)) return;
    const float2 uv = (float2(id.xy) + 0.5f) / size;
    float r, mu;
    AtmTransParams(uv, ATM_RB, ATM_RT, r, mu);
    gU[gLutD.x][id.xy] = float4(AtmSunT(r, mu, ATM_RB), 1.0f);
}

// THE ORDERS PAST THE FIRST. For a point at (r, muS), light arriving from every direction is
// gathered by marching a sphere of directions; the second order is what those rays deliver, and
// f_ms is how much of an isotropic unit source comes back. A medium that returns f_ms of what it
// is given, over and over, returns 1/(1 - f_ms) in total -- the geometric series is the whole of
// Hillaire's trick, and it is why an infinite number of orders costs one texture read.
[numthreads(8, 8, 1)]
void CsMultiScatter(uint3 id : SV_DispatchThreadID) {
    const float2 size = gLutC.xy;
    if (id.x >= uint(size.x) || id.y >= uint(size.y)) return;
    const float2 uv = (float2(id.xy) + 0.5f) / size;
    float r, muS;
    AtmMsParams(uv, ATM_RB, ATM_RT, r, muS);
    const float3 up = kUpLocal;
    const float3 sunDir = normalize(float3(sqrt(max(1.0f - muS * muS, 0.0f)), muS, 0.0f));

    const uint kDirs = 64u;   // 8 x 8 over the sphere, the reference's own count
    float3 lum = 0.0f, fmsSum = 0.0f;
    for (uint i = 0u; i < 8u; ++i) {
        for (uint j = 0u; j < 8u; ++j) {
            // A uniform sphere: cos(theta) even in [-1, 1], phi even in [0, 2pi).
            const float cosT = 1.0f - 2.0f * (i + 0.5f) / 8.0f;
            const float sinT = sqrt(max(1.0f - cosT * cosT, 0.0f));
            const float phi = 6.28318531f * (j + 0.5f) / 8.0f;
            const float3 d = float3(sinT * cos(phi), cosT, sinT * sin(phi));
            float3 fms;
            const float3 L = AtmRay(r, d, up, sunDir, 20u, -1, ATM_RB, ATM_RT, fms);
            lum += L;
            fmsSum += fms;
        }
    }
    lum /= float(kDirs);
    fmsSum /= float(kDirs);
    // 1/(4 pi) is already in the direction average; the series is the isotropic feedback.
    const float3 series = 1.0f / max(1.0f - fmsSum, 1e-4f);
    gU[gLutD.x][id.xy] = float4(lum * series, 1.0f);
}
