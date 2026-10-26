// ================================================================================================
//  laws/Impact.hlsli -- AN ASTEROID INTO THE OCEAN: THE CAVITY, AND ITS WAVE EVERYWHERE AFTER
//  (2026-10-25; docs/WATER_V2.md section 13). One text, two hosts (Laws.hlsli says the dialect).
//
//  Prior art: Ward & Asphaug, "Asteroid impact tsunami: a probabilistic hazard assessment" (Icarus
//  145, 2000) and "Asteroid impact tsunami of 2880 March 16" (GJI 153, 2003). The impact is a transient
//  CAVITY in the sea: a parabola of depth D_c and radius R_c with a raised lip out to R_D = sqrt(2) R_c,
//  net volume nought (the water is in the lip). Its evolution is LINEAR DISPERSIVE theory over a basin
//  of one depth h0: eta(r, t) = integral_0^inf F(k) J0(k r) cos(omega(k) t) k dk, with F the cavity's
//  Hankel transform and omega^2 = g k tanh(k h0). That integral is the one law here, as a SUM over a
//  table of wavenumbers the host builds once (sim/ImpactWave.h): eta(r, t) = sum_n A_n J0(k_n r)
//  cos(omega_n t), A_n = F(k_n) k_n dk. It holds at every r and t -- the cavity at t = 0, the ring
//  of the first wave, the dispersive train behind it -- with no stepping and no state. Near land the
//  wave shoals by Green's law from h0 to the depth here, bounded by that depth (a linear reader's
//  bore); the solver window carries the rest (its faces read this level).
//
//  The cavity from the impactor. Its DIAMETER is the pi-scaling form (Schmidt & Holsapple 1982;
//  Collins, Melosh & Marcus 2005 write it as 1.161 (rho_i/rho_t)^(1/3) L^0.78 V^0.44 g^-0.22
//  sin^(1/3) theta for rock) with the constant FITTED to Ward & Asphaug 2003's published case: a
//  1100 m, 2.2 g/cc impactor at 17.8 km/s makes a cavity 18.9 km across (kImpactCavityC = 1.374 at
//  vertical incidence). Its DEPTH is the energy argument: a share of the impactor's kinetic energy
//  (W&A 2003: 9.4 % of 2.46e20 J) is the cavity's potential energy, pi rho g D_c^2 R_c^2 / 3 for
//  the lipped parabola, and D_c cannot exceed the ocean's depth at the impact. The same case gives
//  D_c = 4990 m against their 6.5 km before capping; both cap at the 4998 m of water there.
//
//  Checked (scratch, numpy): the Bessel forms to 5e-8 against the integral definition; the sum at
//  the 1950 DA case against W&A's figures -- 14 m at 800 km after an hour (x4.7 by Green's law to a
//  25 m beach: their "120 m on the beaches from Cape Cod to Hatteras"), 3-5 m at 2800 km (their
//  "about 9 m in mid-ocean"). Within their factor of two; the form is theirs.
//
//  Bessel functions: Abramowitz & Stegun 9.4.1 / 9.4.3 (J0) and 9.4.4 / 9.4.6 (J1), 5e-8 absolute.
//  A special function's implementation, like a tanh's; the law is the sum above.
// ================================================================================================
#ifndef GA_LAW_IMPACT_HLSLI
#define GA_LAW_IMPACT_HLSLI
#include "Dispersion.hlsli"
#include "Breaking.hlsli"

static const real kImpactCavityC = 1.374;          // the pi-scaling constant, fitted (above)
static const real kImpactLipRatio = 1.4142135623730951;   // R_D / R_c: the lip holds the cavity's water
static const real kImpactKMaxOverRd = 12.0;        // the table's reach in k: 12 / R_D (the cavity's own scale)

inline real BesselJ0(real x) {
    const real ax = abs(x);
    const real y = (ax / 3.0) * (ax / 3.0);
    const real z = 3.0 / max(ax, 3.0);
    const real js = 1.0 + y * (-2.2499997 + y * (1.2656208 + y * (-0.3163866 + y * (0.0444479 + y * (-0.0039444 + y * 0.0002100)))));
    const real f0 = 0.79788456 + z * (-0.00000077 + z * (-0.00552740 + z * (-0.00009512 + z * (0.00137237 + z * (-0.00072805 + z * 0.00014476)))));
    const real t0 = ax - 0.78539816 + z * (-0.04166397 + z * (-0.00003954 + z * (0.00262573 + z * (-0.00054125 + z * (-0.00029333 + z * 0.00013558)))));
    const real ja = f0 * cos(t0) / sqrt(max(ax, 3.0));
    return (ax <= 3.0) ? js : ja;
}
inline real BesselJ1(real x) {
    const real ax = abs(x);
    const real y = (ax / 3.0) * (ax / 3.0);
    const real z = 3.0 / max(ax, 3.0);
    const real js = ax * (0.5 + y * (-0.56249985 + y * (0.21093573 + y * (-0.03954289 + y * (0.00443319 + y * (-0.00031761 + y * 0.00001109))))));
    const real f1 = 0.79788456 + z * (0.00000156 + z * (0.01659667 + z * (0.00017105 + z * (-0.00249511 + z * (0.00113653 + z * -0.00020033)))));
    const real t1 = ax - 2.35619449 + z * (0.12499612 + z * (0.00005650 + z * (-0.00637879 + z * (0.00074348 + z * (0.00079824 + z * -0.00029166)))));
    const real ja = f1 * cos(t1) / sqrt(max(ax, 3.0));
    const real j = (ax <= 3.0) ? js : ja;
    return (x < 0.0) ? -j : j;
}
inline real BesselJ2(real x) {
    const real ax = max(abs(x), 1e-12);
    return 2.0 * BesselJ1(ax) / ax - BesselJ0(ax);
}

// THE CAVITY'S DIAMETER (m) from the impactor: its diameter L (m), speed V (m/s), density, the
// water's density, and the angle from the horizontal (deg).
inline real ImpactCavityDiameter(real impactorDiamM, real speedMs, real rhoImpactor, real rhoWater, real angleDeg) {
    const real s = max(sin(angleDeg * 0.017453292519943295), 0.05);
    return kImpactCavityC * pow(rhoImpactor / rhoWater, 1.0 / 3.0) * pow(impactorDiamM, 0.78) *
           pow(speedMs, 0.44) * pow(GA_PLANET_G, -0.22) * pow(s, 1.0 / 3.0);
}
// THE CAVITY'S DEPTH (m) from the energy its wave carries (J) and its radius: the lipped parabola's
// potential energy is pi rho g D_c^2 R_c^2 / 3. The caller caps it at the ocean's depth.
inline real ImpactCavityDepth(real waveEnergyJ, real cavityRadiusM, real rhoWater) {
    return sqrt(3.0 * waveEnergyJ / (3.141592653589793 * rhoWater * GA_PLANET_G * cavityRadiusM * cavityRadiusM));
}
// THE CAVITY'S SPECTRUM F(k) (m^3): the Hankel transform of eta0(r) = D_c (r^2 / R_c^2 - 1) over
// r < R_D = sqrt(2) R_c, which is D_c [R_D J1(k R_D) / k - 4 J2(k R_D) / k^2].
inline real ImpactSpectrum(real k, real cavityRadiusM, real cavityDepthM) {
    const real RD = kImpactLipRatio * cavityRadiusM;
    const real x = k * RD;
    return cavityDepthM * (RD * BesselJ1(x) / k - 4.0 * BesselJ2(x) / (k * k));
}
// The basin's dispersion: omega(k) over the one depth the wave crosses.
inline real ImpactOmega(real k, real basinDepthM) {
    return sqrt(GA_PLANET_G * k * tanh(k * basinDepthM));
}
// ONE TERM of the sum at distance r (m) and time t (s) since the impact: the level, and its slope
// along r (d eta / d r).
inline real ImpactTerm(real A, real k, real omega, real r, real t) {
    return A * BesselJ0(k * r) * cos(omega * t);
}
inline real ImpactTermSlope(real A, real k, real omega, real r, real t) {
    return -A * k * BesselJ1(k * r) * cos(omega * t);
}
// THE SHOALING to the depth here: Green's law (h0 / h)^(1/4), the WKB limit of the same linear
// theory over a slowly varying bed, BOUNDED by the shore's saturation -- a wave's height stands no
// taller than kGammaHs times the water under it (Breaking.hlsli: the observed Hs <= gamma h), so its
// amplitude no taller than half that; past it the solver's columns say what happens. One odd, smooth
// law: a = b tanh(G eta / b), b = gamma h / 2; its limits are Green's law (small) and the saturation
// (large). The first bound was the depth itself, and a trough reaching -h drained the coarse globe's
// banks to their bed from orbit (2026-10-25); a trough now keeps 1 - gamma / 2 of the water.
inline real ImpactShoalBound(real depthM) { return 0.5 * kGammaHs * max(depthM, kDepthFloorM); }
inline real ImpactShoal(real etaDeep, real basinDepthM, real depthM) {
    const real h = max(depthM, kDepthFloorM);
    const real b = ImpactShoalBound(depthM);
    const real G = pow(basinDepthM / h, 0.25);
    return b * tanh(G * etaDeep / b);
}
// ...and the slope's factor at the same point: d/d(eta) of the law above, G sech^2(G eta / b).
inline real ImpactShoalSlope(real etaDeep, real slopeDeep, real basinDepthM, real depthM) {
    const real h = max(depthM, kDepthFloorM);
    const real b = ImpactShoalBound(depthM);
    const real G = pow(basinDepthM / h, 0.25);
    const real th = tanh(G * etaDeep / b);
    return slopeDeep * G * (1.0 - th * th);   // sech^2 = 1 - tanh^2 (the dialect has tanh, not cosh)
}
#endif
