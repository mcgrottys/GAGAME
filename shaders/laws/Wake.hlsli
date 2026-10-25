// ================================================================================================
//  laws/Wake.hlsli -- THE KELVIN WAKE (ALGEBRA.md `wake`; proofs/kelvin_wake.py). Station keeping
//  c = U cos theta gives k = K0 sec^2 theta, K0 = g/U^2; stationarity reduces to the quadratic
//  2 zeta t^2 + xi t + zeta = 0 (t = tan theta) and the 19.4712-degree wedge IS its discriminant.
//  THE SIGNED PHASE: the vqview shader this engine descends from folded |t| into a non-stationary
//  angle (measured |grad ph| / k up to 6.65 where the envelope theorem demands 1); the true phase at
//  the signed roots is ph = K0 sec theta (xi - zeta |t|), already mirror-symmetric, with slope
//  direction d = grad(ph) / k. Every bound is a closure with its argument recorded (ALGEBRA.md).
//  Host contract: Laws.hlsli; stands alone.
// ================================================================================================
#ifndef GA_LAWS_WAKE_HLSLI
#define GA_LAWS_WAKE_HLSLI

// One vessel, the eight numbers the bank's constant buffer carries per boat (WaterBankLayer::
// SetBoats: gBoatA = (x, z, heading, speed), gBoatB = (amp, half length, enabled, -)).
struct WakeVessel {
    real x;            // world east, m
    real z;            // world north, m
    real heading;      // radians; fwd = (cos hd, sin hd) in the (x, z) plane
    real speed;        // over ground, m/s: K0 = g/U^2, the one parameter that sets the wake's scale
    real wakeAmp;      // base wake amplitude, m
    real hullHalfLen;  // hull half-length, m
    bool enabled;
};

// What a sample point accumulates. eta and the slope SUM over vessels (linear superposition);
// akMax and stern are MAXIMA, which is physical: steepness and prop wash both feed the foam UNION.
struct WakeSample {
    real eta;      // m (SUM)
    real slopeX;   // d(eta)/dx (SUM)
    real slopeZ;   // d(eta)/dz (SUM)
    real akMax;    // the largest steepness a k contributed (MAX): the foam union's input
    real stern;    // stern turbulence envelope 0..1 (MAX): aerated prop wash
};

// ONE stationary-phase branch at t = tan(theta). The four bounds: the MESH-NYQUIST band limit
// smoothstep(2, 5, lambda / sampleM) (below two samples a wavelength the term is alias; sampleM is
// the caller's own grain: the bank's tile texel, a hull's panel spacing); DIVERGENT DAMPING
// exp(-(k / 5 K0)^2) (viscous removal of the short arm; 5 TUNED); the STEEPNESS CAP a k <= 0.30
// (pre-breaking Miche family; past it the energy is foam's, through akMax); the 1e-3 and 1e-6
// floors are division guards.
inline WakeSample WakeBranch(real t, real K0, real xi, real zeta, real amp, real sampleM, real fwdX, real fwdZ,
                             real rgtX, real rgtZ, real side, WakeSample acc) {
    const real sec = sqrt(1.0 + t * t);
    const real k = K0 * sec * sec;   // station keeping: k = K0 sec^2 th
    const real lam = 6.2831853 / k;
    const real w = smoothstep(2.0, 5.0, lam / max(sampleM, 1e-3));
    if (w <= 0.0) return acc;
    const real tAbs = abs(t);
    // THE SIGNED STATIONARY PHASE (a '+' here is the reference's bug)
    const real ph = K0 * sec * (xi - zeta * tAbs);
    const real ct = 1.0 / sec, st = tAbs / sec;
    // d = grad(ph) / k: consistent with ph by construction (they flip together)
    const real dX = -(fwdX * ct + rgtX * (side * st));
    const real dZ = -(fwdZ * ct + rgtZ * (side * st));
    const real kRel = k / max(K0, 1e-6);
    const real damp = exp(-(kRel / 5.0) * (kRel / 5.0));
    const real aMaxK = 0.30 / max(k, 1e-3);
    const real a = min(amp * w * damp, aMaxK);
    acc.eta += a * cos(ph);
    acc.slopeX -= (a * k) * sin(ph) * dX;
    acc.slopeZ -= (a * k) * sin(ph) * dZ;
    acc.akMax = max(acc.akMax, a * k);
    return acc;
}

// One vessel's whole wake at a point. THE WEDGE IS A DISCRIMINANT: with xi metres astern and zeta
// abeam the roots are real only while xi^2 - 8 zeta^2 >= 0, the 19.4712-degree half-angle, and
// nothing here imposes it. The amplitude closures, each with its argument: 1 / sqrt(0.6 + dRel)
// spreading (E ~ 1/r; the 0.6 regularises at the hull, TUNED); the NEAR FADE smoothstep(0.5, 2.5,
// dRel) in hull half-lengths (linear theory dies at the hull); the CUSP BOOST (an Airy caustic at
// the wedge edge; 1.8 and the cube TUNED, faded in over dRel 1..3.5); the DRAUGHT CAP 0.22 x 0.34
// halfLen (TUNED, unexplained in the source); GROWTH along the hull smoothstep(0, 2 halfLen, xi);
// the divergent branch x 0.85 (TUNED); the 1200 m cull is a COST bound the Nyquist limit happens
// to cover. The STERN TURBULENCE term is the reference's tuned closure, every constant in hull
// half-lengths; it joins the foam union by MAX and is not crest-gated.
inline WakeSample WakeOne(WakeVessel v, real px, real pz, real sampleM, WakeSample acc) {
    if (!v.enabled || v.speed < 0.5) return acc;   // K0 diverges as U -> 0; no steady wake below
    const real fwdX = cos(v.heading), fwdZ = sin(v.heading);
    const real rgtX = -fwdZ, rgtZ = fwdX;
    const real rX = px - v.x, rZ = pz - v.z;
    if (rX * rX + rZ * rZ > 1200.0 * 1200.0) return acc;
    const real xi = -(rX * fwdX + rZ * fwdZ);   // metres astern (positive behind)
    const real across = rX * rgtX + rZ * rgtZ;
    const real side = (across >= 0.0) ? 1.0 : -1.0;
    const real zeta = abs(across);
    const real halfLen = max(v.hullHalfLen, 1.0);
    if (xi > 0.0) {
        const real wid = halfLen * 0.34 * (1.0 + xi / (halfLen * 10.0));
        const real sN = smoothstep(0.0, halfLen * 0.3, xi) * exp(-xi / (halfLen * 3.2)) *
                        (1.0 - smoothstep(wid * 0.45, wid * 1.15, zeta));
        acc.stern = max(acc.stern, sN);
    }
    if (xi <= 0.5) return acc;   // nothing ahead of the bow
    const real disc = xi * xi - 8.0 * zeta * zeta;
    if (disc <= 0.0) return acc;   // outside the wedge: no stationary phase
    const real sq = sqrt(disc);
    const real K0 = GA_PLANET_G / (v.speed * v.speed);
    const real dist = max(sqrt(rX * rX + rZ * rZ), 1.0);
    const real dRel = dist / halfLen;
    const real cuspFar = 1.0 + 1.8 * pow(saturate(1.0 - sq / max(xi, 1e-3)), 3.0);
    const real cusp = 1.0 + (cuspFar - 1.0) * smoothstep(1.0, 3.5, dRel);
    const real nearFade = smoothstep(0.5, 2.5, dRel);
    const real ampCap = 0.22 * halfLen * 0.34;
    const real spread = 1.0 / sqrt(0.6 + dRel);
    const real amp = min(v.wakeAmp * smoothstep(0.0, halfLen * 2.0, xi) * cusp * spread, ampCap) * nearFade;
    if (amp < 1e-4) return acc;
    if (zeta < 1e-3) {   // on the track the quadratic degenerates to the single transverse root
        return WakeBranch(0.0, K0, xi, zeta, amp, sampleM, fwdX, fwdZ, rgtX, rgtZ, side, acc);
    }
    // 2 zeta t^2 + xi t + zeta = 0: t1 the TRANSVERSE branch (small |t|), t2 the DIVERGENT one
    const real t1 = (-xi + sq) / (4.0 * zeta);
    const real t2 = (-xi - sq) / (4.0 * zeta);
    acc = WakeBranch(abs(t1), K0, xi, zeta, amp, sampleM, fwdX, fwdZ, rgtX, rgtZ, side, acc);
    acc = WakeBranch(abs(t2), K0, xi, zeta, amp * 0.85, sampleM, fwdX, fwdZ, rgtX, rgtZ, side, acc);
    return acc;
}

#endif
