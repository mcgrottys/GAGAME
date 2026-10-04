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
//  to. What the sun delivers to each step is a CLOSED FORM -- Chapman's function for the two
//  exponential components, an exact shell integral for ozone's tent -- so the march reads no
//  texture for it. The one table left is a property of the AIR, the same for every ray on the
//  planet: what the scattering orders past the first add at a height under a sun angle. (An
//  earlier version of this file also tabulated the sky as seen from the camera; that bakes the
//  sky for ONE eye, and the portal showed the Merrimack's sunset at noon.)
// ================================================================================================
#ifndef GA_ATMOSPHERE_HLSLI
#define GA_ATMOSPHERE_HLSLI

// ---- the measured air, AS ROWS ----------------------------------------------------------------
// A planet enters the sky only through these numbers (src/scene/Air.h: Earth's are the reference
// air above, Mars's its CO2 and dust): there is no planet named anywhere below. The rows ride the
// scene constants (Common.hlsli gAir*) and the sky table's own (SkyLut.hlsl), in Air.h's order.
struct AtmAir {
    float3 rayS;  float rayH;    // Rayleigh scattering (1/m at the ground), scale height (m)
    float3 mieS;  float mieH;    // aerosol scattering, scale height
    float3 mieE;  float mieG;    // aerosol extinction, Henyey-Greenstein g
    float3 ozoA;  float ozoC;    // ozone absorption at the tent's peak, the tent's centre (m)
    float  ozoW;  float top;     // the tent's half-width; the top of the air (m above the ground)
    float  albedo;               // the ground under the multiple-scattering estimate
    float2 invH;                 // 1 / (Rayleigh, aerosol) scale heights
    float2 rootHalfInvH;         // sqrt(1 / 2H): Chapman's sqrt(x / 2) = sqrt(r) times this
};
AtmAir AtmAirRows(float4 r0, float4 r1, float4 r2, float4 r3, float4 r4) {
    AtmAir a;
    a.rayS = r0.xyz; a.rayH = r0.w;
    a.mieS = r1.xyz; a.mieH = r1.w;
    a.mieE = r2.xyz; a.mieG = r2.w;
    a.ozoA = r3.xyz; a.ozoC = r3.w;
    a.ozoW = r4.x;   a.top = r4.y;   a.albedo = r4.z;
    a.invH = 1.0f / max(float2(a.rayH, a.mieH), 1.0f);
    a.rootHalfInvH = sqrt(0.5f * a.invH);
    return a;
}

// The densities at a height above the ground, each 1 at sea level (ozone's tent peaks at 1).
void AtmDensity(AtmAir air, float h, out float rayD, out float mieD, out float ozoD) {
    rayD = exp(-max(h, 0.0f) * air.invH.x);
    mieD = exp(-max(h, 0.0f) * air.invH.y);
    ozoD = max(0.0f, 1.0f - abs(h - air.ozoC) / max(air.ozoW, 1.0f));
}

// What a metre of air at that height takes out of a beam, per channel.
float3 AtmExtinction(AtmAir air, float h) {
    float rayD, mieD, ozoD;
    AtmDensity(air, h, rayD, mieD, ozoD);
    return air.rayS * rayD + air.mieE * mieD + air.ozoA * ozoD;
}

// The two phase functions. Rayleigh is exact for molecules; Henyey-Greenstein is the standard
// one-parameter stand-in for the forward-thrown aerosol lobe.
float AtmPhaseR(float mu) { return 0.0596831f * (1.0f + mu * mu); }   // 3/(16 pi)
float AtmPhaseM(float g, float mu) {
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

// ---- the transmittance grid (Bruneton & Neyret 2008): --sky-probe's two tables ---------------
// r is the distance from the planet's centre, mu the cosine of the ray's zenith angle. The
// mapping spends its resolution where the air does: rho is the ground-parallel distance to the
// horizon, H the atmosphere's own, and the ray's length to the top is measured between its
// shortest and longest possible values at that altitude. Every texel is a ray that clears the
// ground. The runtime reads no such table: the sun's transmittance is a closed form (below).
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

// MANUAL BILINEAR, NOT SampleLevel -- priors 1: the sea shades in the DOMAIN stage and the table
// is built in COMPUTE, and a bindless sample outside the pixel stage returns zero on this adapter.
// xOff: the table's second half (x + 32) holds the SKY'S IRRADIANCE on a level surface.
float3 AtmFetch(uint slot, float2 uv, float2 dims, int xOff = 0) {
    const float2 tf = uv * dims - 0.5f;
    const float2 t0 = floor(tf);
    const float2 fr = tf - t0;
    float3 acc = 0.0f;
    [unroll] for (int k = 0; k < 4; ++k) {
        const int2 tc = clamp(int2(t0) + int2(k & 1, k >> 1), int2(0, 0), int2(dims) - int2(1, 1));
        acc += ((k & 1) ? fr.x : 1.0f - fr.x) * ((k >> 1) ? fr.y : 1.0f - fr.y) *
               gTex[slot][tc + int2(xOff, 0)].rgb;
    }
    return acc;
}
static const float2 kAtmMsDims = float2(32.0f, 32.0f);

// THE UNIT: the march answers PER UNIT OF THE SUN'S IRRADIANCE at the top of the air (radiance in
// sr^-1, irradiance dimensionless). The one sun multiplies it (Common.hlsli kSunE), the same sun
// every surface is lit by -- there is no second, declared gain.

// ---- WHAT THE SUN DELIVERS, IN CLOSED FORM -----------------------------------------------------
// What reaches a point from the sun is exp(-tau), tau the air's column along the sun ray, and the
// column has a closed form for each of the three components -- so no step of any ray reads a
// texture for it, and --sky-probe holds the form against the brute-force integral on the device.
//
// EXPONENTIAL AIR (Rayleigh, aerosols). The column above radius r along a ray at zenith angle z is
// H rho(r) ch(x, z), x = r / H: CHAPMAN's grazing-incidence function (Chapman 1931). Its standard
// closed form, the parabolic expansion about the ray's tangent point,
//     ch(x, z) = sqrt(pi x / 2) exp(x cos^2 z / 2) erfc(sqrt(x / 2) cos z),        z <= 90 deg,
// errs by -1/x at the zenith and -3/(8x) at the horizon: 0.13 % and 0.05 % for this air's
// Rayleigh x ~ 800, less for the aerosols' ~5300. The exponential and the erfc are never formed
// apart (each alone overflows or underflows a float); their product is the scaled complementary
// error function erfcx(y) = t exp(E(t)), t = 1/(1 + y/2), with E the Chebyshev polynomial of
// Numerical Recipes' erfcc (Press et al., 2nd ed., s6.2: fractional error under 1.2e-7
// everywhere), right in both limits -- and E is added to the density's own exponent, so the
// column costs one exponential. Past the horizontal the ray sinks to its tangent radius b and
// climbs again; its column is the published reflection -- twice the tangent point's whole
// grazing column, H rho(b) 2 sqrt(pi b / 2H), less what the reversed ray would see -- which meets
// the upper form exactly at z = 90 deg, so the column is one continuous function of the angle.
//
// OZONE is not exponential -- a tent, ozoW either side of ozoC -- and its column is closed too.
// Through spherical shells, a ray whose tangent radius is b crosses the shell at radius K with
// airmass K / sqrt(K^2 - b^2) = [K / sqrt(K + b)] (K - b)^(-1/2). The bracket moves by under
// 0.2 % across the layer for any ray, so it is taken at the layer's centre; what remains -- the
// tent against (K - b)^(-1/2) -- integrates exactly, because the tent is three ramps (weights
// 1, -2, 1 at its three knots) and each ramp's integral from b out to b + x is
//     psi = (2/3) e (d + e sqrt(p+) + 2 (-p)+),   p = K - b, d = x - p, e = sqrt(x) - sqrt(p+)
// clamped at zero. That holds for the steepest ray and the most grazing alike, where the usual
// single-shell airmass diverges. A rising ray meets the layer beyond its own radius (all of it
// past b, less the part between b and r); a sinking one crosses the part between b and r twice.
//
// THE PLANET blocks a sinking ray whose tangent radius is inside it. That is the whole shadow
// test -- which is what makes dusk -- and it needs no intersection.
static const float kAtmSqrtPi = 1.77245385f;

// E(t) in erfcx(y) = t exp(E(t)).
float2 AtmErfcxExponent(float2 t) {
    return -1.26551223f +
           t * (1.00002368f +
           t * (0.37409196f +
           t * (0.09678418f +
           t * (-0.18628806f +
           t * (0.27886807f +
           t * (-1.13520398f +
           t * (1.48851587f +
           t * (-0.82215223f +
           t * 0.17087277f))))))));
}

// What reaches a point at radius r from the sun at zenith cosine muS.
float3 AtmSunT(AtmAir air, float r, float muS, float Rb) {
    const float h = max(r - Rb, 0.0f);
    const float sink = (muS < 0.0f) ? 1.0f : 0.0f;
    const float s = sqrt(max(1.0f - muS * muS, 0.0f));
    const float b = r * s;                          // the ray's tangent radius
    const float drop = r * muS * muS / (1.0f + s);  // r - b, without cancellation
    const float2 hs = float2(air.rayH, air.mieH);

    // Rayleigh and aerosols together: H rho(r) ch(x, |z|), and the tangent point's full column
    // (whose exponent is never positive for a ray the planet lets through).
    const float2 sx = sqrt(r) * air.rootHalfInvH;   // sqrt(x / 2)
    const float2 t = 1.0f / (1.0f + 0.5f * abs(muS) * sx);
    const float2 up = hs * kAtmSqrtPi * sx * t * exp(AtmErfcxExponent(t) - h * air.invH);
    const float2 tangent = 2.0f * hs * kAtmSqrtPi * sqrt(b) * air.rootHalfInvH *
                           exp(min((drop - h) * air.invH, 0.0f));
    const float2 col = up + sink * (tangent - 2.0f * up);

    // Ozone: the three ramps from b out to the layer's top, and from b out to r.
    const float R2 = Rb + air.ozoC;
    const float3 knots = float3(R2 - air.ozoW, R2, R2 + air.ozoW);
    const float3 p = knots - b;
    const float3 lam = sqrt(max(p, 0.0f));
    const float3 neg = 2.0f * max(-p, 0.0f);
    const float3 eOut = max(lam.z - lam, 0.0f);
    const float3 eIn = max(sqrt(drop) - lam, 0.0f);
    const float3 psiOut = eOut * ((knots.z - knots) + lam * eOut + neg);
    const float3 psiIn = eIn * ((r - knots) + lam * eIn + neg);
    const float ozo = (0.6666667f / max(air.ozoW, 1.0f)) * R2 * rsqrt(R2 + b) *
                      dot(float3(1.0f, -2.0f, 1.0f), psiOut + (2.0f * sink - 1.0f) * psiIn);

    const float3 tau = air.rayS * col.x + air.mieE * col.y + air.ozoA * ozo;
    return (1.0f - sink * step(b, Rb)) * exp(-tau);
}

// ---- THE SKY: ONE LINE INTEGRAL, FROM WHEREVER THE EYE IS ---------------------------------------
// The light the air scatters toward the eye along `dir`, from where the ray ENTERS the air to
// where it LEAVES it, strikes the GROUND, or reaches tMax (a surface the caller is shading):
//
//     L = integral over [t0, t1] of T(t0, t) [ T_sun(t) (sigma_R p_R + sigma_M p_M) + sigma_s ms(t) ] dt
//
// The eye's position says only where the segment starts: inside the air t0 = 0, the eye itself;
// above it, t0 is the ray's entry into the shell. There is no ceiling and no second model -- the
// limb seen from orbit, the sky from the helm and the haze over the disc are this one segment.
// Out: T, what survives over the segment (the sun's disc and a surface behind are seen through
// it); tGround, the distance at which the ray met the ground, or -1.
//
// r is the eye's distance from the planet's centre and `up` its zenith; the geometry is carried
// by the tangent radius b and the distance tc to the tangent point (rr^2 = b^2 + (t - tc)^2), so a
// point a hundred kilometres along a grazing ray keeps its metres at three planet radii.
//
// STEPS CROWDED TOWARD THE START. `steps` steps spaced exponentially from where the segment
// begins, t = t0 + (t1 - t0) (e^{g i/n} - 1)/(e^g - 1) -- the old march's spacing, now from
// max(eye, entry). Measured against the even-stepped reference (--sky-probe), this beats
// crowding the steps at the segment's lowest (densest) point by 2-8x on the horizon: a grazing
// path is optically THICK (blue's tau along the horizon is ~19), so the light that reaches the eye
// comes from the first optical depth beside it, not from the dense air at the tangent, which the
// air in front has already put out.
//
// The orders past the first are read at EVERY step, under that step's own sun: the table is a
// property of the air (a height and a sun angle), not of the eye.
float3 AtmRay(AtmAir air, float r, float3 dir, float3 up, float3 sunDir, uint steps, int msSlot,
              float Rb, float tMax, out float3 T, out float3 fms, out float tGround) {
    T = 1.0f;
    fms = 0.0f;
    tGround = -1.0f;
    const float Rt = Rb + air.top;
    const float mu = dot(dir, up);
    const float muS = dot(sunDir, up);
    const float nu = dot(dir, sunDir);
    const float b2 = r * r * max(1.0f - mu * mu, 0.0f);   // the tangent radius, squared
    const float tc = -r * mu;                              // the distance to the tangent point
    if (b2 >= Rt * Rt) return 0.0f;                        // the ray never meets the air
    const float hw = sqrt(Rt * Rt - b2);
    const float t0 = max(tc - hw, 0.0f);                   // the eye, or the air's entry
    const float tTop = tc + hw;                            // the air's exit
    const bool hits = (b2 < Rb * Rb) && (tc > 0.0f);       // the ground stands in the way
    const float tG = hits ? tc - sqrt(Rb * Rb - b2) : tTop;
    const float t1 = min(tG, tMax);
    if (hits && tG <= tMax) tGround = max(tG, 0.0f);   // (an eye standing on the ground: 0)
    if (t1 <= t0) return 0.0f;
    const float phaseR = AtmPhaseR(nu);
    const float phaseM = AtmPhaseM(air.mieG, nu);
    const float g = 3.0f;
    const float eg = exp(g) - 1.0f;
    const float len = t1 - t0;
    float3 L = 0.0f;
    float tPrev = t0;
    for (uint i = 0u; i < steps; ++i) {
        const float tNext = t0 + len * (exp(g * float(i + 1u) / float(steps)) - 1.0f) / eg;
        const float ds = tNext - tPrev;
        const float t = 0.5f * (tPrev + tNext);
        tPrev = tNext;
        const float rr = sqrt(b2 + (t - tc) * (t - tc));
        float rayD, mieD, ozoD;
        AtmDensity(air, rr - Rb, rayD, mieD, ozoD);
        const float3 sigS = air.rayS * rayD + air.mieS * mieD;
        const float3 sigE = max(air.rayS * rayD + air.mieE * mieD + air.ozoA * ozoD, 1e-12f);
        // The sun's zenith cosine AT THAT POINT: air a hundred kilometres along the ray stands under
        // a different sun angle, and that is the whole geometry of a sunset.
        const float muSp = clamp((muS * r + t * nu) / max(rr, 1e-6f), -1.0f, 1.0f);
        float3 S = AtmSunT(air, rr, muSp, Rb) *
                   (air.rayS * rayD * phaseR + air.mieS * mieD * phaseM);
        if (msSlot >= 0) {
            S += AtmFetch(uint(msSlot), AtmMsUv(rr, muSp, Rb, Rt), kAtmMsDims) * sigS;
        }
        const float3 Tstep = exp(-sigE * ds);
        // The step integrated analytically (Hillaire), so few steps do not band.
        L += T * (S - S * Tstep) / sigE;
        fms += T * (sigS - sigS * Tstep) / sigE;
        T *= Tstep;
    }
    return L;
}

// The ground where a ray from the eye met it (AtmRay's tGround): a Lambert bounce of the direct
// sun, in the march's units (the caller carries it through the segment's T).
float3 AtmGroundBounce(AtmAir air, float r, float3 dir, float3 up, float3 sunDir, float tGround,
                       float Rb) {
    const float3 n = normalize(up * r + dir * tGround);
    const float muSg = dot(n, sunDir);
    return (muSg > 0.0f) ? air.albedo * muSg * AtmSunT(air, Rb, muSg, Rb) * 0.3183099f
                         : float3(0.0f, 0.0f, 0.0f);
}

#endif  // GA_ATMOSPHERE_HLSLI
