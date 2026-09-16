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
//  WHY LUTS AND NOT A MARCH PER PIXEL: every mirror in this engine asks the sky for radiance --
//  the water's Fresnel reflection is the sky, and it asks at every water pixel. A march there
//  would cost more than the rest of the frame. So the same three tables Hillaire uses are built
//  once (transmittance, multiple scattering) or once a frame (the sky view), and every consumer
//  -- the dome, the sea's mirror, the haze -- reads ONE of them. Two copies of a sky model is the
//  bug the sky pass's own header warns about.
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

// ---- the sky-view table: the hemisphere as the eye sees it (Hillaire 2020) --------------------
// The horizon gets the resolution, because that is where the air is thickest and the colour
// changes fastest: the vertical coordinate is square-rooted away from the horizon line, and the
// two halves of the texture are the sky and the ground-intersecting rays.
float2 AtmSkyViewUv(float r, float3 dir, float3 up, float3 sunDir, float Rb) {
    const float cosHorizon = sqrt(max(r * r - Rb * Rb, 0.0f)) / max(r, 1e-6f);
    const float beta = acos(clamp(cosHorizon, -1.0f, 1.0f));
    const float zenithHorizon = 3.14159265f - beta;
    const float vza = acos(clamp(dot(dir, up), -1.0f, 1.0f));
    float v;
    if (vza < zenithHorizon) {
        const float c = vza / max(zenithHorizon, 1e-6f);
        v = 0.5f * (1.0f - sqrt(max(1.0f - c, 0.0f)));
    } else {
        const float c = (vza - zenithHorizon) / max(beta, 1e-6f);
        v = 0.5f + 0.5f * sqrt(max(c, 0.0f));
    }
    // Azimuth measured FROM THE SUN, so the table is what the sky actually is: symmetric about
    // the sun's meridian, with the whole circle resolved for the halo and the anti-solar side.
    const float3 e = normalize(dir - up * dot(dir, up));
    const float3 s = normalize(sunDir - up * dot(sunDir, up));
    float3 t = cross(up, s);
    const float az = atan2(dot(e, t), dot(e, s));
    return float2(saturate(az * 0.15915494f + 0.5f), saturate(v));
}

void AtmSkyViewParams(float2 uv, float r, float3 up, float3 sunDir, float Rb, out float3 dir) {
    const float cosHorizon = sqrt(max(r * r - Rb * Rb, 0.0f)) / max(r, 1e-6f);
    const float beta = acos(clamp(cosHorizon, -1.0f, 1.0f));
    const float zenithHorizon = 3.14159265f - beta;
    float vza;
    if (uv.y < 0.5f) {
        const float c = 1.0f - 2.0f * uv.y;
        vza = zenithHorizon * (1.0f - c * c);
    } else {
        const float c = 2.0f * uv.y - 1.0f;
        vza = zenithHorizon + beta * c * c;
    }
    const float az = (uv.x - 0.5f) * 6.28318531f;
    const float3 s = normalize(sunDir - up * dot(sunDir, up));
    const float3 t = cross(up, s);
    const float sinV = sin(vza), cosV = cos(vza);
    dir = normalize(up * cosV + (s * cos(az) + t * sin(az)) * sinV);
}

#endif  // GA_ATMOSPHERE_HLSLI
