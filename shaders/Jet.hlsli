// The entrance tidal jet and the wave-current amplification, as PURE FUNCTIONS of their
// parameters -- shared by Sea.hlsl (display) and SeaChurn.hlsl (the sparse churn simulation),
// which must agree exactly about where the water is breaking.
#ifndef GA_JET_HLSLI
#define GA_JET_HLSLI

// Analytic ebb/flood jet: channel axis through its current station (xz: metres east and north of
// it) along the ebb direction, Gaussian across-channel, exponential decay seaward, plateau
// upstream. signedMs: + flood, - ebb.
float2 JetVelocity(float2 xz, float signedMs, float halfWidth, float seawardDecay,
                   float2 floodDir, float2 ebbDir) {
    const float along = dot(xz, ebbDir);
    const float crossd = length(xz - along * ebbDir);
    float env = exp(-(crossd * crossd) / (halfWidth * halfWidth));
    env *= (along > 0.0f) ? exp(-along / seawardDecay) : smoothstep(-4000.0f, -2500.0f, along);
    const float2 flow = (signedMs >= 0.0f) ? floodDir : ebbDir;
    return flow * abs(signedMs) * env;
}

// Finite-depth phase speed for a band's representative wavenumber: c = sqrt(g/k tanh(kh)).
// THE 7-FOOT-STANDING-WAVE TERM: in 4 m of water an 11 s swell slows from ~17 m/s to ~6, so a
// 0.7-1 m/s ebb that deep water shrugs off drives the SWELL band a third of the way to blocking
// right where the bar is shallow -- which is exactly where the entrance stands up and breaks.
float BandPhaseSpeed(float kBand, float depth) {
    const float kh = kBand * max(depth, 0.15f);
    return sqrt(9.81f / kBand * tanh(kh));
}

// Green's-law-style shoaling: amplitude grows as the group speed drops entering shallow water.
float ShoalFactor(float kBand, float depth) {
    const float kh = kBand * max(depth, 0.15f);
    const float th = tanh(kh);
    const float c = sqrt(9.81f / kBand * th);
    const float cg = 0.5f * c * (1.0f + 2.0f * kh / max(sinh(2.0f * kh), 1e-3f));
    const float cgDeep = 0.5f * sqrt(9.81f / kBand);
    return clamp(sqrt(cgDeep / max(cg, 0.05f)), 0.75f, 1.7f);
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
