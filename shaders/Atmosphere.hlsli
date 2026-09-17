// ================================================================================================
//  Atmosphere.hlsli - THE AIR, from measurements rather than from two constants.
//
//  The sky in this engine was `lerp(horizon, zenith, smoothstep(0, 0.30, d.y))`: two colours and
//  an elevation. It could not know what time it was -- a sunset and a noon sky were the same
//  pixels -- and every mirror in the water inherited that, which is why the sea never reddened
//  either. What replaces it is the ordinary physics of a clear atmosphere, with the numbers taken
//  from published work rather than fitted by eye:
//
//    RAYLEIGH   molecular scattering, sea-level coefficients at 680/550/440 nm and an 8 km scale
//               height. beta = (5.802, 13.558, 33.100) x 1e-6 m^-1 -- Bruneton & Neyret 2008 and
//               Bruneton 2017's reference implementation, computed from the air's refractive
//               index and the US Standard Atmosphere 1976 density profile. (The engine's own
//               from-space shell already carried (5.8, 13.5, 33.1)e-6: the same measurement.)
//    MIE        aerosol scattering 3.996e-6 and extinction 4.440e-6 m^-1, 1.2 km scale height,
//               Henyey-Greenstein g = 0.8 -- the same reference's standard clear-air aerosol.
//    OZONE      ABSORPTION ONLY, (0.650, 1.881, 0.085) x 1e-6 m^-1 on a tent profile centred at
//               25 km and 15 km wide (the US Standard Atmosphere's ozone layer). This is the term
//               my first attempt at a physical sky left out, and leaving it out is exactly why
//               that sky went BROWN at dusk: with no ozone the long twilight path keeps its red
//               and loses its blue, and the Chappuis absorption band -- which eats 500-700 nm --
//               is the whole reason a real twilight sky stays blue overhead.
//
//  MULTIPLE SCATTERING is not optional either. Single scattering alone gives a sky that is too
//  dark and too saturated, and a twilight that dies instead of glowing. The second-order-and-
//  beyond term is carried the way Hillaire 2020 carries it ("A Scalable and Production Ready Sky
//  and Atmosphere Rendering Technique", EGSR): an isotropic estimate tabulated over (altitude,
//  sun elevation), summed as a geometric series so an infinite number of orders costs one lookup.
//
//  EVERY RAY IS MARCHED WHERE IT IS. There is no table of the view: a pixel's ray, a mirror's, and
//  a ray a gate carried to the other side of the planet each integrate the air from their own
//  origin under the one sun, so a window onto Haulover shows Haulover's sky without being told
//  to. The only tables are properties of the AIR, the same for every ray on the planet: what it
//  lets through toward the sun from a height at an angle, and what the scattering orders past the
//  first add there. (An earlier version of this file also tabulated the sky as seen from the
//  camera; that bakes the sky for ONE eye, and the portal showed the Merrimack's sunset at noon.)
// ================================================================================================
#ifndef GA_ATMOSPHERE_HLSLI
#define GA_ATMOSPHERE_HLSLI

// ---- the measured air ---------------------------------------------------------------------
static const float3 kAtmRayS = float3(5.802e-6f, 13.558e-6f, 33.100e-6f);   // scattering, 1/m
static const float  kAtmRayH = 8000.0f;                                     // scale height, m
static const float  kAtmMieS = 3.996e-6f;
static const float  kAtmMieE = 4.440e-6f;
static const float  kAtmMieH = 1200.0f;
static const float  kAtmMieG = 0.80f;
static const float3 kAtmOzoA = float3(0.650e-6f, 1.881e-6f, 0.085e-6f);     // absorption, 1/m
static const float  kAtmOzoC = 25000.0f;   // the layer's centre
static const float  kAtmOzoW = 15000.0f;   // its half-width (a tent, zero outside)
static const float  kAtmTopM = 100000.0f;  // the air ends here (the Karman line, near enough)
static const float  kAtmAlbedo = 0.1f;     // the ground under the multiple-scattering estimate

// The densities at a height above the ground, each 1 at sea level (ozone's tent peaks at 1).
void AtmDensity(float h, out float rayD, out float mieD, out float ozoD) {
    rayD = exp(-max(h, 0.0f) / kAtmRayH);
    mieD = exp(-max(h, 0.0f) / kAtmMieH);
    ozoD = max(0.0f, 1.0f - abs(h - kAtmOzoC) / kAtmOzoW);
}

// What a metre of air at that height takes out of a beam, per channel.
float3 AtmExtinction(float h) {
    float rayD, mieD, ozoD;
    AtmDensity(h, rayD, mieD, ozoD);
    return kAtmRayS * rayD + kAtmMieE.xxx * mieD + kAtmOzoA * ozoD;
}

// The two phase functions. Rayleigh is exact for molecules; Henyey-Greenstein is the standard
// one-parameter stand-in for the forward-thrown aerosol lobe.
float AtmPhaseR(float mu) { return 0.0596831f * (1.0f + mu * mu); }   // 3/(16 pi)
float AtmPhaseM(float mu) {
    const float g = kAtmMieG;
    const float d = 1.0f + g * g - 2.0f * g * mu;
    return 0.0795775f * (1.0f - g * g) / max(d * sqrt(max(d, 1e-6f)), 1e-6f);   // 1/(4 pi)
}

// Distance to a sphere of radius R from a point r along a unit ray with cos(zenith) = mu.
// Negative when the ray misses (or the root is behind): the callers test for that.
float AtmRaySphere(float r, float mu, float R) {
    const float disc = r * r * (mu * mu - 1.0f) + R * R;
    if (disc < 0.0f) return -1.0f;
    const float s = sqrt(disc);
    const float t0 = -r * mu - s, t1 = -r * mu + s;
    if (t1 < 0.0f) return -1.0f;
    return (t0 < 0.0f) ? t1 : t0;
}

// ---- the transmittance table's parameterisation (Bruneton & Neyret 2008) ---------------------
// r is the distance from the planet's centre, mu the cosine of the ray's zenith angle. The
// mapping spends its resolution where the air does: rho is the ground-parallel distance to the
// horizon, H the atmosphere's own, and the ray's length to the top is measured between its
// shortest and longest possible values at that altitude.
float2 AtmTransUv(float r, float mu, float Rb, float Rt) {
    const float H = sqrt(max(Rt * Rt - Rb * Rb, 1e-6f));
    const float rho = sqrt(max(r * r - Rb * Rb, 0.0f));
    const float disc = r * r * (mu * mu - 1.0f) + Rt * Rt;
    const float d = max(0.0f, -r * mu + sqrt(max(disc, 0.0f)));
    const float dMin = Rt - r;
    const float dMax = rho + H;
    return float2(saturate((d - dMin) / max(dMax - dMin, 1e-6f)), saturate(rho / H));
}

void AtmTransParams(float2 uv, float Rb, float Rt, out float r, out float mu) {
    const float H = sqrt(max(Rt * Rt - Rb * Rb, 1e-6f));
    const float rho = H * saturate(uv.y);
    r = sqrt(max(rho * rho + Rb * Rb, 0.0f));
    const float dMin = Rt - r;
    const float dMax = rho + H;
    const float d = dMin + saturate(uv.x) * (dMax - dMin);
    mu = (d == 0.0f) ? 1.0f : clamp((H * H - rho * rho - d * d) / (2.0f * r * d), -1.0f, 1.0f);
}

// ---- the multiple-scattering table: altitude and the sun's elevation, nothing else -----------
float2 AtmMsUv(float r, float muS, float Rb, float Rt) {
    return float2(saturate(0.5f + 0.5f * muS), saturate((r - Rb) / max(Rt - Rb, 1e-6f)));
}
void AtmMsParams(float2 uv, float Rb, float Rt, out float r, out float muS) {
    muS = clamp(saturate(uv.x) * 2.0f - 1.0f, -1.0f, 1.0f);
    r = Rb + saturate(uv.y) * (Rt - Rb);
}

// ---- THE RAY, MARCHED (M13). No table of the view: every ray -- a pixel's, a mirror's, one the
// gate carried to the other side of the planet -- integrates the air from wherever it actually is,
// under the one sun. What IS tabulated is a property of the air alone, identical for every ray on
// the planet: what it lets through toward the sun from a height at an angle (transmittance), and
// what every scattering order past the first adds there (multiple scattering).
//
// MANUAL BILINEAR, NOT SampleLevel -- priors 1: the sea shades in the DOMAIN stage and the tables
// are built in COMPUTE, and a bindless sample outside the pixel stage returns zero on this adapter.
float3 AtmFetch(uint slot, float2 uv, float2 dims) {
    const float2 tf = uv * dims - 0.5f;
    const float2 t0 = floor(tf);
    const float2 fr = tf - t0;
    float3 acc = 0.0f;
    [unroll] for (int k = 0; k < 4; ++k) {
        const int2 tc = clamp(int2(t0) + int2(k & 1, k >> 1), int2(0, 0), int2(dims) - int2(1, 1));
        acc += ((k & 1) ? fr.x : 1.0f - fr.x) * ((k >> 1) ? fr.y : 1.0f - fr.y) *
               gTex[slot][tc].rgb;
    }
    return acc;
}
static const float2 kAtmTransDims = float2(256.0f, 64.0f);
static const float2 kAtmMsDims = float2(32.0f, 32.0f);

// THE ONE DECLARED GAIN: the march answers in the model's units (per unit solar irradiance); the
// engine's radiance is a relative unit a tonemapper set. This carries one into the other, chosen
// against the sky the engine drew at noon. It is the only number here that is not a measurement.
static const float kAtmGain = 18.0f;

// What reaches a point at radius r from the sun at zenith cosine muS: the air's table, and nothing
// at all when the planet stands in the way -- which is what makes dusk.
float3 AtmSunT(uint transSlot, float r, float muS, float Rb, float Rt) {
    if (AtmRaySphere(r, muS, Rb) > 0.0f) return 0.0f;
    return AtmFetch(transSlot, AtmTransUv(r, muS, Rb, Rt), kAtmTransDims);
}

// The radiance along `dir` from a point at radius r whose zenith is `up`, lit by `sunDir`.
// msSlot < 0 marches single scattering only (the multiple-scattering table is built with it) and
// returns in `fms` the isotropic response that table needs.
float3 AtmRay(float r, float3 dir, float3 up, float3 sunDir, uint steps, uint transSlot,
              int msSlot, float Rb, float Rt, out float3 fms) {
    fms = 0.0f;
    const float mu = dot(dir, up);
    const float muS = dot(sunDir, up);
    const float nu = dot(dir, sunDir);
    const float ground = AtmRaySphere(r, mu, Rb);
    const float disc = r * r * (mu * mu - 1.0f) + Rt * Rt;
    const float top = max(0.0f, -r * mu + sqrt(max(disc, 0.0f)));
    const float dist = (ground > 0.0f) ? ground : top;
    if (dist <= 0.0f) return 0.0f;
    const float phaseR = AtmPhaseR(nu);
    const float phaseM = AtmPhaseM(nu);
    // The orders past the first are a smooth ambient: read once per ray, at its origin, and
    // weighted by each step's own scattering coefficient below.
    const float3 msHere = (msSlot >= 0)
        ? AtmFetch(uint(msSlot), AtmMsUv(r, muS, Rb, Rt), kAtmMsDims) : float3(0.0f, 0.0f, 0.0f);
    float3 L = 0.0f;
    float3 T = 1.0f;
    // STEPS CROWDED TOWARD THE ORIGIN. A ray from near the ground crosses most of its air in the
    // first few kilometres (Rayleigh's scale height is 8 km, the aerosols' 1.2), so the steps are
    // spaced exponentially from where the ray starts: t_i = d (e^{g i/n} - 1)/(e^g - 1). With the
    // in-step integral done analytically, a handful of such steps holds the column.
    const float g = 3.0f;
    const float eg = exp(g) - 1.0f;
    float tPrev = 0.0f;
    for (uint i = 0u; i < steps; ++i) {
        const float tNext = dist * (exp(g * float(i + 1u) / float(steps)) - 1.0f) / eg;
        const float ds = tNext - tPrev;
        const float t = 0.5f * (tPrev + tNext);
        tPrev = tNext;
        const float rr = sqrt(max(r * r + t * t + 2.0f * r * mu * t, 0.0f));
        float rayD, mieD, ozoD;
        AtmDensity(rr - Rb, rayD, mieD, ozoD);
        const float3 sigS = kAtmRayS * rayD + kAtmMieS.xxx * mieD;
        const float3 sigE = max(kAtmRayS * rayD + kAtmMieE.xxx * mieD + kAtmOzoA * ozoD, 1e-12f);
        // The sun's zenith cosine AT THAT POINT: air a hundred kilometres along the ray stands under
        // a different sun angle, and that is the whole geometry of a sunset.
        const float muSp = clamp((muS * r + t * nu) / max(rr, 1e-6f), -1.0f, 1.0f);
        float3 S = AtmSunT(transSlot, rr, muSp, Rb, Rt) *
                   (kAtmRayS * rayD * phaseR + kAtmMieS.xxx * mieD * phaseM);
        S += msHere * sigS;
        const float3 Tstep = exp(-sigE * ds);
        // The step integrated analytically (Hillaire), so few steps do not band.
        L += T * (S - S * Tstep) / sigE;
        fms += T * (sigS - sigS * Tstep) / sigE;
        T *= Tstep;
    }
    // The ground, when the ray reaches it: a Lambert bounce of the direct sun.
    if (ground > 0.0f) {
        const float3 n = normalize(up * r + dir * ground);
        const float muSg = dot(n, sunDir);
        if (muSg > 0.0f) {
            L += T * kAtmAlbedo * muSg * AtmSunT(transSlot, Rb, muSg, Rb, Rt) * 0.3183099f;
        }
    }
    return L;
}

#endif  // GA_ATMOSPHERE_HLSLI
