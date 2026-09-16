// ================================================================================================
//  SkyLut.hlsl - the three tables the sky is made of (Atmosphere.hlsli holds the air itself).
//
//    CsTransmittance   256 x 64   what survives a ray from an altitude to the top of the air.
//                                 Depends on the atmosphere alone, so it is built ONCE.
//    CsMultiScatter     32 x 32   the isotropic estimate of every scattering order past the
//                                 first, summed as a geometric series (Hillaire 2020 s5).
//                                 Depends on the atmosphere alone: built ONCE.
//    CsSkyView        192 x 108   the radiance of the hemisphere from THIS eye with THIS sun.
//                                 One dispatch a frame, and every consumer reads it.
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
    uint4  gLutD;    // x = the table being written (UAV slot), y = transmittance, z = MS (SRVs)
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

// THE TRANSMITTANCE ALONG A RAY, marched. 40 steps is well past the point where the answer stops
// moving for a 100 km atmosphere -- this table is built once, so there is no reason to be mean.
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

float3 TransLookup(float r, float mu) {
    const float2 uv = AtmTransUv(r, mu, ATM_RB, ATM_RT);
    return SkyLutFetch(gLutD.y, uv, float2(256.0f, 64.0f));
}

// THE SUN'S LIGHT AT A POINT: what reaches it through the air, and nothing at all when the
// planet itself is in the way. The shadow test is the reason a low sun leaves the ground dark
// while the air above it is still lit -- which is what dusk IS.
float3 SunTransmittance(float r, float muS) {
    if (AtmRaySphere(r, muS, ATM_RB) > 0.0f) return 0.0f;
    return TransLookup(r, muS);
}

// ---- the march, shared by the multiple-scattering estimate and the sky view --------------------
// Returns the radiance along `dir` from a point at radius r, and (for the MS pass) the fraction
// of light that would come back if the medium scattered isotropically with unit strength.
float3 MarchSky(float r, float3 dir, float3 up, float3 sunDir, uint steps, bool withMs,
                out float3 fms) {
    fms = 0.0f;
    const float mu = dot(dir, up);
    const float muS = dot(sunDir, up);
    const float nu = dot(dir, sunDir);
    const float ground = AtmRaySphere(r, mu, ATM_RB);
    const float disc = r * r * (mu * mu - 1.0f) + ATM_RT * ATM_RT;
    const float top = max(0.0f, -r * mu + sqrt(max(disc, 0.0f)));
    const float dist = (ground > 0.0f) ? ground : top;
    if (dist <= 0.0f) return 0.0f;

    const float phaseR = AtmPhaseR(nu);
    const float phaseM = AtmPhaseM(nu);
    float3 L = 0.0f;
    float3 T = 1.0f;
    const float ds = dist / steps;
    for (uint i = 0u; i < steps; ++i) {
        const float t = (i + 0.5f) * ds;
        const float rr = sqrt(max(r * r + t * t + 2.0f * r * mu * t, 0.0f));
        const float h = rr - ATM_RB;
        float rayD, mieD, ozoD;
        AtmDensity(h, rayD, mieD, ozoD);
        const float3 sigS = kAtmRayS * rayD + kAtmMieS.xxx * mieD;
        const float3 sigE = max(kAtmRayS * rayD + kAtmMieE.xxx * mieD + kAtmOzoA * ozoD, 1e-12f);
        // The sun's zenith cosine AT THAT POINT, not at the eye: the whole geometry of a sunset
        // is that the air a hundred kilometres away stands under a different sun angle.
        const float muSp = (muS * r + t * dot(dir, sunDir)) / max(rr, 1e-6f);
        const float3 sunT = SunTransmittance(rr, clamp(muSp, -1.0f, 1.0f));
        float3 S = sunT * (kAtmRayS * rayD * phaseR + kAtmMieS.xxx * mieD * phaseM);
        if (withMs) {
            const float2 uv = AtmMsUv(rr, clamp(muSp, -1.0f, 1.0f), ATM_RB, ATM_RT);
            S += SkyLutFetch(gLutD.z, uv, float2(32.0f, 32.0f)) * sigS;
        }
        const float3 Tstep = exp(-sigE * ds);
        // The analytic integral of the step, so the answer does not depend on where the sample
        // landed inside it.
        const float3 Sint = (S - S * Tstep) / sigE;
        L += T * Sint;
        // The isotropic response the MS pass needs: the same integral with the scattering
        // coefficient alone and no phase, no sun.
        const float3 Fint = (sigS - sigS * Tstep) / sigE;
        fms += T * Fint;
        T *= Tstep;
    }
    // The ground, when the ray reaches it: a Lambert bounce of the direct sun. It is what lifts
    // the lower sky over a bright surface, and it is the only place the albedo enters.
    if (ground > 0.0f) {
        const float3 p = up * r + dir * ground;   // in the local frame, up is the eye's zenith
        const float3 n = normalize(p);
        const float muSg = dot(n, sunDir);
        if (muSg > 0.0f) {
            L += T * kAtmAlbedo * muSg * SunTransmittance(ATM_RB, muSg) * 0.3183099f;
        }
    }
    return L;
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
            const float3 L = MarchSky(r, d, up, sunDir, 20u, false, fms);
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

[numthreads(8, 8, 1)]
void CsSkyView(uint3 id : SV_DispatchThreadID) {
    const float2 size = gLutC.xy;
    if (id.x >= uint(size.x) || id.y >= uint(size.y)) return;
    const float2 uv = (float2(id.xy) + 0.5f) / size;
    const float r = gLutA.w;
    const float3 up = kUpLocal;
    const float3 sunDir = normalize(gLutA.xyz);
    float3 dir;
    AtmSkyViewParams(uv, r, up, sunDir, ATM_RB, dir);
    float3 fms;
    const float3 L = MarchSky(r, dir, up, sunDir, 32u, true, fms);
    gU[gLutD.x][id.xy] = float4(L * gLutB.z, 1.0f);
}
