// ================================================================================================
//  laws/Dispersion.hlsli -- finite-depth linear wave theory for one band or component: the phase
//  speed, the wavenumber in this depth, the orbit's horizontal factor, Green's-law shoaling, and the
//  wave-current gain. Pure functions of their parameters; docs/ALGEBRA.md `physics` is the
//  derivation. Host contract: Laws.hlsli.
// ================================================================================================
#ifndef GA_LAWS_DISPERSION_HLSLI
#define GA_LAWS_DISPERSION_HLSLI

// The last-resort depth floor, 0.15 m. Every depth law below divides by tanh(k h) or a group speed
// built from it, and both go singular as h -> 0; the shoreline is a place where h really does reach
// zero. A DOMAIN guard, not physics, and the innermost one: callers impose larger floors where the
// physics wants them (0.3 m for the orbit, 0.4 m for the churn's chop).
static const real kDepthFloorM = 0.15;

// c(k, h) = sqrt(g/k tanh(kh)): exact linear theory for the band's REPRESENTATIVE wavenumber.
// Deep: tanh -> 1, c -> sqrt(g/k) (depth cannot be felt); shallow: tanh(kh) -> kh, c -> sqrt(g h)
// (wavenumber cannot be felt). THE 7-FOOT-STANDING-WAVE TERM: in 4 m of water an 11 s swell slows
// from ~17 m/s to ~6, so a 0.7-1 m/s ebb that deep water shrugs off drives the swell band a third
// of the way to blocking right where the bar is shallow -- where the entrance stands up and breaks.
inline real BandPhaseSpeed(real kBand, real depth) {
    const real kh = kBand * max(depth, kDepthFloorM);
    return sqrt(GA_PLANET_G / kBand * tanh(kh));
}

// THE WAVENUMBER IN THIS DEPTH (the crest law, 2026-10-19): a band's frequency is its deep-water
// k's (sigma^2 = g kDeep); in depth h the same frequency travels at the k that solves
// sigma^2 = g k tanh(kh). Guo's explicit form (Coastal Engineering 45, 2002; < 0.75 %):
// kh = x^2 (1 - exp(-x^(5/2)))^(-2/5), x = sigma sqrt(h/g) = sqrt(kDeep h). Deep: kh -> x^2 (k =
// kDeep); shallow: kh -> x (k = sigma / sqrt(g h)). Returns k h.
inline real ShoaledKh(real kDeep, real depth) {
    const real x = sqrt(max(kDeep * depth, 1e-6));
    return x * x * pow(1.0 - exp(-pow(x, 2.5)), -0.4);
}

// The Airy orbit's horizontal excursion over its vertical at the surface, coth(kh), with k the
// band's in this depth: 1 in deep water, the slosh of a long wave in the shallows. (coth of the
// DEEP k reached 113 on the 213 m band in the shallows: 52 m of slosh, a torn sheet -- priors 53 b.)
inline real OrbitCoth(real kDeep, real depth) {
    return 1.0 / tanh(ShoaledKh(kDeep, max(depth, 0.3)));
}
// ...and for a wavenumber already solved in this depth (a solved component's k).
inline real OrbitCothK(real k, real depth) {
    return 1.0 / tanh(k * max(depth, 0.3));
}

// Green's law: the ENERGY FLUX E c_g through the band is conserved and E ~ a^2, so a ~ c_g^(-1/2):
//     c_g = (c/2) (1 + 2kh / sinh 2kh),  S = sqrt(c_g^deep / c_g),  clamped [0.75, 1.7].
// The reference is the deep-water group speed AT THE SAME WAVENUMBER, (1/2) sqrt(g/k): the bank
// holds a fixed representative k per band and shoals THAT band. 1.7 binds in real water (at the
// 0.15 m floor the swell band asks for x2.75, which no wave that shallow carries: breaking owns it);
// 0.75 never binds (the law's minimum is the classical shoaling dip, ~0.93 near kh ~ 2).
inline real ShoalFactor(real kBand, real depth) {
    const real kh = kBand * max(depth, kDepthFloorM);
    const real th = tanh(kh);
    const real c = sqrt(GA_PLANET_G / kBand * th);
    const real cg = 0.5 * c * (1.0 + 2.0 * kh / max(sinh(2.0 * kh), 1e-3));
    const real cgDeep = 0.5 * sqrt(GA_PLANET_G / kBand);
    return clamp(sqrt(cgDeep / max(cg, 0.05)), 0.75, 1.7);
}

// What WaveCurrentAmp answers with: two numbers that are NOT interchangeable. `amp` multiplies the
// band's height; `blocked` drives breaking FOAM and is never an amplitude. Named so a caller cannot
// swap them and still compile.
struct WaveCurrentGain {
    real amp;       // the band's amplitude multiplier, in [0.55, 2.0]
    real blocked;   // 0 free .. 1 fully blocked
};

// Linear wave-action amplification over a COLLINEAR current. r = U . d^ / c0 is the current
// projected on the wave's own direction in units of the still-water phase speed; linear theory:
//     c'/c0 = (1 + sqrt(1 + 4r)) / 2,   A/A0 = 1 / sqrt((c'/c0)^2 (2 c'/c0 - 1))    (exact; GaTest
//     block 6 pins it equal to the action form 1 / sqrt(cr (cr + 2r)) to 1e-12).
// Opposing current (r < 0) shortens and steepens toward BLOCKING at r = -1/4 exactly, where the
// group speed against the stream is zero and the amplification diverges. Real water breaks: the
// closures are blocked = smoothstep(-0.16, -0.245, r) and a saturation at 1.45 past the band (both
// TUNED, ALGEBRA.md `physics` says so), and [0.55, 2.0] on the amplification. The -0.2499 floor,
// the 1e-4 and 1e-3 floors are never reached with a binding value (the early return comes first):
// belt and braces against a future edit, not physics. max(c0, 0.5) IS a domain guard: a band's
// phase speed goes to zero in vanishing depth and would send r to infinity for any current at all.
inline WaveCurrentGain WaveCurrentAmp(real curX, real curZ, real dirX, real dirZ, real c0) {
    const real r = (curX * dirX + curZ * dirZ) / max(c0, 0.5);
    WaveCurrentGain g;
    g.blocked = smoothstep(-0.16, -0.245, r);
    if (r <= -0.245) {
        g.amp = 1.45;
        g.blocked = 1.0;
        return g;
    }
    const real rc = clamp(r, -0.2499, 4.0);
    const real q = sqrt(max(1.0 + 4.0 * rc, 1e-4));
    const real cr = 0.5 * (1.0 + q);   // c'/c0
    const real amp = clamp(1.0 / sqrt(max(cr * cr * (2.0 * cr - 1.0), 1e-3)), 0.55, 2.0);
    g.amp = lerp(amp, 1.45, g.blocked * g.blocked);
    return g;
}

#endif
