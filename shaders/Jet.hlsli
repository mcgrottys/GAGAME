// The wave-current amplification, as PURE FUNCTIONS of their parameters -- shared by the water
// bank and SeaChurn.hlsl, which must agree exactly about where the water is breaking. PHASE C3b:
// the analytic entrance jet is deleted; the current is the solver's own (sim/WaterTerms.h twins these).
// THE PLANET'S GRAVITY (2026-10-20, core/Planet.h): GM / R^2, from the includer's constant buffer --
// WaterBank.hlsl's gPlanet.x, SeaChurn.hlsl's gMiscC.w. No default: a kernel that forgets it fails to
// compile rather than run on Earth's.
#ifndef GA_PLANET_G
#error "Jet.hlsli: define GA_PLANET_G (the planet's surface gravity, from the constant buffer) before including"
#endif
#ifndef GA_JET_HLSLI
#define GA_JET_HLSLI

// Finite-depth phase speed for a band's representative wavenumber: c = sqrt(g/k tanh(kh)).
// THE 7-FOOT-STANDING-WAVE TERM: in 4 m of water an 11 s swell slows from ~17 m/s to ~6, so a
// 0.7-1 m/s ebb that deep water shrugs off drives the SWELL band a third of the way to blocking
// right where the bar is shallow -- which is exactly where the entrance stands up and breaks.
float BandPhaseSpeed(float kBand, float depth) {
    const float kh = kBand * max(depth, 0.15f);
    return sqrt(GA_PLANET_G / kBand * tanh(kh));
}

// THE WAVENUMBER IN THIS DEPTH (the crest law, 2026-10-19): a band's frequency is its deep-water
// k's (sigma^2 = g kDeep); in depth h the same frequency travels at the k that solves
// sigma^2 = g k tanh(kh). Guo's explicit form (Coastal Engineering 45, 2002; < 0.75 %):
// kh = x^2 (1 - exp(-x^(5/2)))^(-2/5), x = sigma sqrt(h/g) = sqrt(kDeep h). Deep: kh -> x^2 (k =
// kDeep); shallow: kh -> x (k = sigma / sqrt(g h)).
float ShoaledKh(float kDeep, float depth) {
    const float x = sqrt(max(kDeep * depth, 1e-6f));
    return x * x * pow(1.0f - exp(-pow(x, 2.5f)), -0.4f);
}
// The Airy orbit's horizontal excursion over its vertical at the surface, coth(kh), with k the
// band's in this depth: 1 in deep water, the slosh of a long wave in the shallows.
float OrbitCoth(float kDeep, float depth) {
    return 1.0f / tanh(ShoaledKh(kDeep, max(depth, 0.3f)));
}

// THE LIP'S FALL (2026-10-20; docs/WATER_VOLUME.md 15). Water thrown e metres past the cusp has left
// the face at the crest's speed c and falls for t = e / c, a t^2 / 2. Two corrections from the
// literature (Mark: "review your math with online documentation"):
//   - c WITH AMPLITUDE: linear dispersion runs ~20 % slow at breaking and in the surf zone, where
//     amplitude dispersion dominates (coastalwiki, "Nonlinear wave dispersion relations"); the
//     cnoidal form c^2 = g h (1 + alpha H / h), alpha ~ 1, carried onto the full dispersion as
//     c^2 = (g / k) tanh(kh) (1 + H / h): linear theory again as H / h -> 0 in deep water;
//   - a, THE JET TIP'S MEASURED FALL: 0.6 - 0.8 g, not g (Shakeri, Tavakolinejad, Maxeiner and
//     Duncan, APS DFD 2007, plunging jets in a wave tank).
// So   fall = (a / g) k coth(kh) e^2 / (2 (1 + H / h)),   and g cancels: no planet's gravity in it,
// only the ratio, which is any planet's. Its limits: nothing at the cusp (e = 0); the launch speed
// is the crest's, which the literature disputes (below c, Melville & Rapp; to 1.3 c, Duncan); the
// loop the throw is bounded by is the trochoid's kinematic device, not a surface the Euler equations
// allow (Gerstner's waves are deep-water, rotational, and unstable past steepness 1/3).
static const float kJetFallRatio = 0.7f;   // a / g, measured (0.6 - 0.8)
float LipFall(float ePast, float k, float coth, float H, float h) {
    return kJetFallRatio * 0.5f * k * coth * ePast * ePast / (1.0f + max(H, 0.0f) / max(h, 0.3f));
}

// THE THROW'S BOUND IS THE LANDING (2026-10-20, Mark: "I see the lip but the waves don't seem to
// barrel correctly"). Measured at Mavericks' close pose: the lip's lowest point stood at -1.79 m over a
// trough at -3.61 m -- it hung at most 79 % of the wave's height (67 % at the median), so no tube
// closed. The throw was bounded by the trochoid's loop, pi / 2k, a kinematic device. The physical bound
// is the fall's own: the jet flies until it lands in the trough. LipFall(e*) = Hc, the crest's height
// over the trough, gives the flight
//     e* = sqrt(2 Hc (1 + Hc / h) / ((a / g) k coth(kh))),
// and the crest's reach is the cusp plus that flight, 1 / k + e*. No new constant: a set wave (more
// Hc) throws farther and lands, as it does. Tubes are measured by their vortex ratio, the length over
// the width of the tube's section (Mead & Black 2001, J. Coastal Research 29: ~1 a round tube, the
// most intense, ~3 an almond), which this law does not fix: the reef's slope and the swell set it.
float LipReach(float k, float coth, float Hc, float h) {
    const float H = max(Hc, 0.0f);
    const float eLand = sqrt(2.0f * H * (1.0f + H / max(h, 0.3f)) / max(kJetFallRatio * k * coth, 1e-6f));
    return 1.0f / max(k, 1e-4f) + eLand;
}

// Green's-law-style shoaling: amplitude grows as the group speed drops entering shallow water.
float ShoalFactor(float kBand, float depth) {
    const float kh = kBand * max(depth, 0.15f);
    const float th = tanh(kh);
    const float c = sqrt(GA_PLANET_G / kBand * th);
    const float cg = 0.5f * c * (1.0f + 2.0f * kh / max(sinh(2.0f * kh), 1e-3f));
    const float cgDeep = 0.5f * sqrt(GA_PLANET_G / kBand);
    return clamp(sqrt(cgDeep / max(cg, 0.05f)), 0.75f, 1.7f);
}

// F7: BREAKING AS A DISSIPATION LAW, the clipped Rayleigh sea (sim/WaterTerms.h BreakFactor):
// L = sqrt(1 - exp(-1/excess^2)) on every amplitude, excess = Hs_raw / (0.60 h). Deep: 1; the
// shore: rms -> the limit from below. No cap on a height anywhere.
// F7b: the height a wave can stand to (WaterTerms.h BreakHmax): Miche's 0.88/k in deep water,
// 0.60 h at the shore, one tanh between. k the sea's energy-weighted wavenumber at the point.
float BreakHmax(float k, float depth) {
    const float kk = max(k, 1e-6f);
    return (0.88f / kk) * tanh(0.60f * kk * max(depth, 0.05f) / 0.88f);
}

float BreakFactor(float excess) {
    if (!(excess > 1e-6f)) return 1.0f;
    return sqrt(1.0f - exp(-1.0f / (excess * excess)));
}

// Linear wave-action amplification over a collinear current: r = U_parallel / c0. Opposing
// (r < 0) shortens and STEEPENS toward blocking at r = -1/4; past it real waves BREAK, so the
// growth saturates and the second component reports the blocking fraction (drives breaking foam).
float2 WaveCurrentAmp(float2 U, float2 waveDir, float c0) {
    const float r = dot(U, waveDir) / max(c0, 0.5f);
    const float blocked = smoothstep(-0.16f, -0.245f, r);
    if (r <= -0.245f) return float2(1.45f, 1.0f);
    const float rc = clamp(r, -0.2499f, 4.0f);
    const float q = sqrt(max(1.0f + 4.0f * rc, 1e-4f));
    const float cr = 0.5f * (1.0f + q);             // c'/c0
    const float amp = clamp(1.0f / sqrt(max(cr * cr * (2.0f * cr - 1.0f), 1e-3f)), 0.55f, 2.0f);
    return float2(lerp(amp, 1.45f, blocked * blocked), blocked);
}

#endif  // GA_JET_HLSLI

// THE LIP'S FLIGHT AS A MOTOR (2026-10-20, Mark: "the lip's flight as a GA motor (the free-fall frame
// as a boost, the flight a CGA translator)"). In the free-fall frame -- Longuet-Higgins' frame for the
// plunging jet -- the thrown water moves on a straight line, and it leaves ALONG THE SHEET: the launch
// direction is the travel d projected into the sheet's tangent plane, (d . T) T^-1 for the tangent
// bivector T, which for the height field is d + (d . grad eta) up, normalised. The boost back to the
// planet's frame is the fall along -up (LipFall). Both are translators and translators commute, so the
// flight is ONE translator, tau = s u - fall up, applied to the label's point by the motor sandwich
// (GA.hlsli MotorPoint; Pga.h's Motor::Translation on the CPU). Where the sheet is level -- the crest's
// top -- u is the travel and this is the law of before; on the front face u turns down and the water
// is thrown along the face, on the back it turns up and the crest rises as it is thrown, as a crest
// does before it pitches. (du / dx is not carried into the twins' Jacobian, as dq1 / deta is not.)
// x east, y up, z north (the bank's displacement's axes). s: the throw; gradEta: the sheet's slope.
float3 LipFlightTau(float2 dBar, float2 gradEta, float s, float fall) {
    const float m = dot(dBar, gradEta);   // the sheet's rise along the travel
    const float3 u = float3(dBar.x, m, dBar.y) * rsqrt(1.0f + m * m);
    return u * s - float3(0.0f, fall, 0.0f);
}
