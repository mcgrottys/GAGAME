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
    uint4  gLutD;    // x = the table being written (UAV slot), y = the air's table (SRV slot,
                     // the curve reads it), zw = spare
    float4 gLutAir[5];   // the planet's air, src/scene/Air.h's five rows (Atmosphere.hlsli)
};
#define ATM_RB (gLutB.x)
#define ATM_RT (gLutB.y)
#define ATM_AIR (AtmAirRows(gLutAir[0], gLutAir[1], gLutAir[2], gLutAir[3], gLutAir[4]))

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
        od += AtmExtinction(ATM_AIR, rr - Rb) * ds;
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
    gU[gLutD.x][id.xy] = float4(AtmSunT(ATM_AIR, r, mu, ATM_RB), 1.0f);
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
    const AtmAir air = ATM_AIR;

    const uint kDirs = 64u;   // 8 x 8 over the sphere, the reference's own count
    float3 lum = 0.0f, fmsSum = 0.0f, irr = 0.0f;
    for (uint i = 0u; i < 8u; ++i) {
        for (uint j = 0u; j < 8u; ++j) {
            // A uniform sphere: cos(theta) even in [-1, 1], phi even in [0, 2pi).
            const float cosT = 1.0f - 2.0f * (i + 0.5f) / 8.0f;
            const float sinT = sqrt(max(1.0f - cosT * cosT, 0.0f));
            const float phi = 6.28318531f * (j + 0.5f) / 8.0f;
            const float3 d = float3(sinT * cos(phi), cosT, sinT * sin(phi));
            float3 fms, T;
            float tG;
            float3 L = AtmRay(air, r, d, up, sunDir, 20u, -1, ATM_RB, 3.0e38f, T, fms, tG);
            if (tG >= 0.0f) L += T * AtmGroundBounce(air, r, d, up, sunDir, tG, ATM_RB);
            lum += L;
            fmsSum += fms;
            irr += L * max(cosT, 0.0f);   // what falls on a level surface from this direction
        }
    }
    lum /= float(kDirs);
    fmsSum /= float(kDirs);
    // 1/(4 pi) is already in the direction average; the series is the isotropic feedback.
    const float3 series = 1.0f / max(1.0f - fmsSum, 1e-4f);
    gU[gLutD.x][id.xy] = float4(lum * series, 1.0f);
    // THE SKY'S IRRADIANCE on a level surface here, per unit sun, every order: the hemisphere's
    // radiance times its cosine (4 pi / 64 per direction), the orders past the first by the same
    // series. The table's second half (x + 32): Common.hlsli SkyAmbient reads it.
    gU[gLutD.x][id.xy + uint2(uint(size.x), 0u)] =
        float4(irr * (12.5663706f / float(kDirs)) * series, 1.0f);
}

// ---- --sky-probe: THE SKY'S CURVE OVER THE EYE'S RADIUS ------------------------------------------
// One texel a (radius, row). The eye stands on the +y axis at radius r, from 1 m above the ground
// to three planet radii (log-spaced in altitude: kCurveW samples); the sun is fixed in space, so
// it stands at the same elevation over every eye on that axis. Rows: two suns x three rays x
// kCurveModels models.
//   suns   0: 20 deg up, the rays 90 deg of azimuth from it;  1: 2 deg up, the rays 30 deg from it
//   rays   0 ZENITH;  1 HORIZON: tangent to the ground 200 m up (level, for an eye below that);
//          2 LIMB: tangent 25 km up (level, for an eye below that)
//   models 0 the runtime's integral with 12 steps
//          1 the runtime's integral with 6 steps
//          2 the runtime's integral with 10 steps
//          3 THE RUNTIME'S INTEGRAL as the sky asks it (Common.hlsli SkyAlong, discless, the
//            ground's bounce): kSkySteps, exponential from the segment's start
//          (part 2's first cut crowded 6 + 6 steps at the segment's lowest point; it measured
//          2-8x worse on the horizon than this and is gone: out/sky/p2/probe.log)
//          4 THE REFERENCE: the same integral in kRefSteps even steps
// (Part 1's rows -- today's march, gradient and shell -- were recorded before those models were
// deleted: out/sky/p1/curve_today.csv, run from out/sky/P1.)
static const uint kCurveModels = 5u;
static const uint kRefSteps = 4096u;

// The ray at radius r on +y for row (sun s, ray k): its direction and its sun.
void CurveRay(float r, uint s, uint k, float Rb, out float3 dir, out float3 sun) {
    const float el = (s == 0u) ? radians(20.0f) : radians(2.0f);
    const float az = (s == 0u) ? radians(90.0f) : radians(30.0f);
    sun = float3(cos(el), sin(el), 0.0f);
    float mu = 1.0f;
    if (k > 0u) {
        const float b = min(Rb + ((k == 1u) ? 200.0f : 25000.0f), r);
        mu = -sqrt(max(1.0f - (b / r) * (b / r), 0.0f));
    }
    const float sh = sqrt(max(1.0f - mu * mu, 0.0f));
    dir = float3(sh * cos(az), mu, sh * sin(az));
}

// The integral along one segment in `n` steps, `even` over [t0, t1] (the reference) or exponential
// from t0. The integrand is AtmRay's, line for line.
float3 CurveMarch(AtmAir air, float r, float3 dir, float3 up, float3 sunDir, int msSlot, float Rb,
                  uint n, bool even) {
    const float Rt = Rb + air.top;
    const float mu = dot(dir, up), muS = dot(sunDir, up), nu = dot(dir, sunDir);
    const float b2 = r * r * max(1.0f - mu * mu, 0.0f);
    const float tc = -r * mu;
    if (b2 >= Rt * Rt) return 0.0f;
    const float hw = sqrt(Rt * Rt - b2);
    const float t0 = max(tc - hw, 0.0f);
    const bool hits = (b2 < Rb * Rb) && (tc > 0.0f);
    const float t1 = hits ? tc - sqrt(Rb * Rb - b2) : tc + hw;
    if (t1 <= t0) return 0.0f;
    const float phaseR = AtmPhaseR(nu), phaseM = AtmPhaseM(air.mieG, nu);
    const float eg = exp(3.0f) - 1.0f;
    float3 L = 0.0f, T = 1.0f;
    float tPrev = t0;
    for (uint i = 0u; i < n; ++i) {
        const float w = even ? float(i + 1u) / float(n) : (exp(3.0f * float(i + 1u) / float(n)) - 1.0f) / eg;
        const float tNext = t0 + (t1 - t0) * w;
        const float ds = tNext - tPrev;
        const float t = 0.5f * (tPrev + tNext);
        tPrev = tNext;
        const float rr = sqrt(b2 + (t - tc) * (t - tc));
        float rayD, mieD, ozoD;
        AtmDensity(air, rr - Rb, rayD, mieD, ozoD);
        const float3 sigS = air.rayS * rayD + air.mieS * mieD;
        const float3 sigE = max(air.rayS * rayD + air.mieE * mieD + air.ozoA * ozoD, 1e-12f);
        const float muSp = clamp((muS * r + t * nu) / max(rr, 1e-6f), -1.0f, 1.0f);
        float3 S = AtmSunT(air, rr, muSp, Rb) * (air.rayS * rayD * phaseR + air.mieS * mieD * phaseM);
        if (msSlot >= 0) S += AtmFetch(uint(msSlot), AtmMsUv(rr, muSp, Rb, Rt), kAtmMsDims) * sigS;
        const float3 Ts = exp(-sigE * ds);
        L += T * (S - S * Ts) / sigE;
        T *= Ts;
    }
    if (hits) L += T * AtmGroundBounce(air, r, dir, up, sunDir, t1, Rb);
    return L * kSunE;
}

// The runtime's integral as the sky asks it (Common.hlsli SkyAlong, without the disc).
float3 CurveRuntime(AtmAir air, float r, float3 dir, float3 up, float3 sunDir, int msSlot,
                    float Rb, uint steps) {
    float3 T, fms;
    float tG;
    float3 L = AtmRay(air, r, dir, up, sunDir, steps, msSlot, Rb, 3.0e38f, T, fms, tG);
    if (tG >= 0.0f) L += T * AtmGroundBounce(air, r, dir, up, sunDir, tG, Rb);
    return L * kSunE;
}

[numthreads(8, 8, 1)]
void CsSkyCurve(uint3 id : SV_DispatchThreadID) {
    const float2 size = gLutC.xy;
    if (id.x >= uint(size.x) || id.y >= uint(size.y)) return;
    const float Rb = ATM_RB;
    const AtmAir air = ATM_AIR;
    // Altitude 1 m .. 2 Rb, log-spaced: the radius from the ground to three planet radii.
    const float hAlt = exp(log(2.0f * Rb) * float(id.x) / max(size.x - 1.0f, 1.0f));
    const float r = Rb + hAlt;
    const uint model = id.y % kCurveModels;
    const uint ray = (id.y / kCurveModels) % 3u;
    const uint sunI = id.y / (kCurveModels * 3u);
    const float3 up = kUpLocal;
    float3 dir, sun;
    CurveRay(r, sunI, ray, Rb, dir, sun);
    const int ms = int(gLutD.y);
    float3 L = 0.0f;
    if (model == 0u) {
        L = CurveRuntime(air, r, dir, up, sun, ms, Rb, 12u);
    } else if (model == 1u) {
        L = CurveRuntime(air, r, dir, up, sun, ms, Rb, 6u);
    } else if (model == 2u) {
        L = CurveRuntime(air, r, dir, up, sun, ms, Rb, 10u);
    } else if (model == 3u) {
        L = CurveRuntime(air, r, dir, up, sun, ms, Rb, kSkySteps);
    } else {
        L = CurveMarch(air, r, dir, up, sun, ms, Rb, kRefSteps, true);
    }
    gU[gLutD.x][id.xy] = float4(L, hAlt);
}

// ---- --sky-probe: THE SKY AT THIS FRAME'S EYE ----------------------------------------------------
// One texel a (radius, row). x: 0 = gSkyLut.z (FrameLoop eyeRadiusM), 1 = the level row's radius
// (Globe.hlsl sLvlSkyEyeR), 2 = the eye's distance from the planet's centre in doubles.
// y: ray * 2 + model; rays 0 zenith, 1 level horizontal along the view's azimuth, 2 the dipped
// horizon (tangent 200 m above the ground) along it; models 0 = the dome and every reflection,
// 1 = the globe's backdrop -- both the one integral now (Common.hlsli SkyAlong, discless).
// Sun: gLutA.xyz, in the frame whose +y is the eye's zenith and +x the view's azimuth.
[numthreads(8, 8, 1)]
void CsSkyAt(uint3 id : SV_DispatchThreadID) {
    if (id.x >= 3u || id.y >= 6u) return;
    const float Rb = ATM_RB;
    const float r = (id.x == 0u) ? gLutB.z : ((id.x == 1u) ? gLutB.w : gLutC.z);
    const uint ray = id.y / 2u;
    const float3 up = kUpLocal, sun = normalize(gLutA.xyz);
    float mu = 1.0f;
    if (ray == 1u) mu = 0.0f;
    if (ray == 2u) {
        const float b = min(Rb + 200.0f, r);
        mu = -sqrt(max(1.0f - (b / r) * (b / r), 0.0f));
    }
    const float3 dir = float3(sqrt(max(1.0f - mu * mu, 0.0f)), mu, 0.0f);
    const float3 L = CurveRuntime(ATM_AIR, r, dir, up, sun, int(gLutD.y), Rb, kSkySteps);
    gU[gLutD.x][id.xy] = float4(L, r);
}

// ---- --sky-probe: ONE UNIT -- the sky against the ground it lights ----------------------------
// Row y: the sun's elevation (90, 60, 30, 10, 2, 0, -2 deg). Row 0, column 2 is the picture's
// white (SkyLayer::WhiteNoonY: the tonemap's exposure is its inverse). Columns, at sea level, in the engine's
// unit (SUN_IRR_C = E / pi): 0 the zenith's radiance, 1 the sky 3 deg above the horizon (90 deg
// from the sun), 2 a white level Lambertian ground's radiance -- the sun through the air times its
// cosine plus the sky's irradiance (Common.hlsli SunAt + SkyAmbient) -- 3 its sky part alone.
[numthreads(8, 8, 1)]
void CsSkyUnits(uint3 id : SV_DispatchThreadID) {
    if (id.x >= 4u || id.y >= 7u) return;
    const float els[7] = {90.0f, 60.0f, 30.0f, 10.0f, 2.0f, 0.0f, -2.0f};
    const float el = radians(els[id.y]);
    const float Rb = ATM_RB;
    const AtmAir air = ATM_AIR;
    const float3 up = kUpLocal;
    const float3 sun = float3(cos(el), sin(el), 0.0f);
    const int ms = int(gLutD.y);
    float3 L = 0.0f;
    if (id.x == 0u) {
        L = CurveRuntime(air, Rb + 1.0f, up, up, sun, ms, Rb, kSkySteps);
    } else if (id.x == 1u) {
        const float m = sin(radians(3.0f));
        L = CurveRuntime(air, Rb + 1.0f, float3(0.0f, m, sqrt(1.0f - m * m)), up, sun, ms, Rb,
                         kSkySteps);
    } else {
        const float3 sky = (ms >= 0)
            ? SUN_IRR_C * AtmFetch(uint(ms), AtmMsUv(Rb, sin(el), Rb, Rb + air.top), kAtmMsDims, 32)
            : float3(0.0f, 0.0f, 0.0f);
        const float3 sunL = SUN_IRR_C * AtmSunT(air, Rb, sin(el), Rb) * max(sin(el), 0.0f);
        L = (id.x == 2u) ? sunL + sky : sky;
    }
    gU[gLutD.x][id.xy] = float4(L, els[id.y]);
}

// ---- --sky-probe: THE SKY'S COLOUR -----------------------------------------------------------------
// The sun 60 deg up. Row y = aerosol a (0: none, 1: the air's own) * 6 + ray: 0 the zenith, 1 a ray
// 90 deg from the sun (30 deg up, opposite azimuth), 2..5 the aureole -- 2, 5, 10, 20 deg from the
// sun toward the zenith. Columns: 0 the runtime integral, 1 the reference (4096 even steps), 2 and
// 3 the same without the orders past the first (single scattering only). Sea level.
[numthreads(8, 8, 1)]
void CsSkyColour(uint3 id : SV_DispatchThreadID) {
    if (id.x >= 4u || id.y >= 12u) return;
    const float Rb = ATM_RB;
    AtmAir air = ATM_AIR;
    if (id.y < 6u) {
        air.mieS = 0.0f;
        air.mieE = 0.0f;
    }
    const uint ray = id.y % 6u;
    const float3 up = kUpLocal;
    const float sEl = radians(60.0f);
    const float3 sun = float3(cos(sEl), sin(sEl), 0.0f);
    float el = 90.0f, az = 0.0f;
    if (ray == 1u) { el = 30.0f; az = 180.0f; }
    if (ray >= 2u) {
        const float offs[4] = {2.0f, 5.0f, 10.0f, 20.0f};
        el = 60.0f + offs[ray - 2u];
    }
    const float e = radians(el), a = radians(az);
    const float3 dir = float3(cos(e) * cos(a), sin(e), cos(e) * sin(a));
    const int ms = (id.x >= 2u) ? -1 : int(gLutD.y);
    const float r = Rb + 1.0f;
    const float3 L = ((id.x % 2u) == 0u) ? CurveRuntime(air, r, dir, up, sun, ms, Rb, kSkySteps)
                                         : CurveMarch(air, r, dir, up, sun, ms, Rb, kRefSteps, true);
    gU[gLutD.x][id.xy] = float4(L, el);
}
