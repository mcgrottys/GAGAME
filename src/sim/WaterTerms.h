// ================================================================================================
//  WaterTerms - the water bank's closed-form laws, in doubles, on the CPU.
//
//  WHY THIS FILE EXISTS.  Every law that shapes the water the player SEES lives in HLSL:
//  shaders/Jet.hlsli decides how a band's amplitude and phase speed answer to depth and to the
//  tidal jet, and shaders/WaterBank.hlsl adds each vessel's Kelvin wake on top.  A hull is
//  simulated on the CPU.  If the CPU applies a DIFFERENT law -- even a slightly different one --
//  the hull rides water the render is not drawing: it heaves over a crest the camera is looking
//  into the back of, and no amount of tuning ever fixes it, because nothing is wrong.  Same
//  trap OceanCpu.h exists to close for the cascade sea; this file closes it for the four terms
//  the bank applies AFTER the cascade -- shoaling, phase speed, wave-current gain, and wakes.
//
//  WHY IT CAN BE A HEADER OF FREE FUNCTIONS.  All four are pure, closed form and stateless:
//  they read nothing, remember nothing, and allocate nothing.  A Kelvin wake in particular is
//  STEADY in the ship's frame, so "what did this wake look like a second ago" is not a question
//  -- the field is a function of the offset alone.  That is why the GPU can evaluate it per
//  texel with no history and why the CPU can evaluate it per hull station for the same price.
//
//  THE PORTING RULE.  Every constant below is transcribed from the shader WITH the physical
//  argument that put it there.  Where the source records the argument, the argument is copied.
//  Where the source records only that a number was TUNED, this file says TUNED -- it does not
//  invent a derivation.  ALGEBRA.md `wake` calls this set "an engineering closure with its
//  physical argument recorded"; a number that arrives here without its argument has lost the
//  only thing that makes it reviewable.
//
//  THE ONE PLACE THIS PORT IS NOT A TRANSCRIPTION -- and it is not a deviation either.  The
//  wake's phase is the SIGNED stationary phase (ALGEBRA.md `wake`).  The vqview shader this
//  engine descends from folded |t| into the phase, which evaluates phi at +|t| where the
//  stationary root is -|t|: a non-stationary angle whose measured |grad phi| / k runs 1.03-6.65
//  where the envelope theorem demands exactly 1.  WaterBank.hlsl already ships the corrected,
//  signed form; this file mirrors WaterBank.hlsl, not the reference.  proofs/water_terms.py
//  re-measures that same gate against THIS header's own text (it transpiles the bodies below
//  and runs them), and re-measures it again with the sign flipped, which reproduces the
//  reference and its failure.  If the gate ever passes for both forms, the gate is broken.
//
//  PRECISION.  Doubles throughout, per the GPU-resident law's CPU half: the GPU may carry these
//  in fp32 because it is painting pixels; a hull integrating at 240 Hz may not.  The two answers
//  therefore differ at the fp32 rounding level BY DESIGN.  Anything larger is a port bug.
//
//  DEPENDENCIES: <cmath> and nothing else, deliberately.  This header is included by physics
//  code that must not drag in windows.h.
//
//  Source of record: shaders/Jet.hlsli (lines 23, 29, 41), shaders/WaterBank.hlsl (181, 202).
//  Spec: docs/ALGEBRA.md `physics` and `wake`.  Gates: proofs/water_terms.py,
//  proofs/kelvin_wake.py, src/core/GaTest.cpp block 10.
// ================================================================================================
#pragma once

#include <cmath>

namespace ga {

// Gravity, exactly the value every shader in this engine writes as 9.81f.  It is a CONSTANT OF
// THE PORT, not a physical measurement to be refined: if this and the HLSL literal ever differ,
// the CPU's dispersion relation is not the GPU's and every wavelength drifts.
//
// IT IS DELIBERATELY NOT sim/Medium.h's kG (9.80665, standard gravity), and the two must not be
// merged. That one is PHYSICS -- it weighs hulls, and the vessel gates pin draughts and periods
// against it. This one is PARITY -- it reproduces a shader literal. They differ by 0.03%, which
// is nothing next to the wake's own engineering closures and everything next to a silent
// divergence between the sea the hull feels and the sea the GPU draws. Use ga::kG when computing
// a force; use kGWave when reproducing a kernel.  (proofs/water_terms.py pins them APART.)
inline constexpr double kGWave = 9.81;

// The last-resort depth floor, 0.15 m.  Both depth laws below divide by tanh(k*h) or by a group
// speed built from it, and both go singular as h -> 0; the shoreline is a place where h really
// does reach zero.  This is a DOMAIN guard, not physics -- and it is the innermost one: callers
// impose their own, larger floors where the physics wants it (WaterBank.hlsl uses 0.3 m for
// phase speed, SeaChurn.hlsl 0.4 m), because at 15 cm nothing in a wave band is linear anyway.
inline constexpr double kDepthFloorM = 0.15;

// ------------------------------------------------------------------------------------------------
//  wt - the scalar shims HLSL hands a shader for free.  Written out rather than pulled from
//  <algorithm> so that Smoothstep is DEMONSTRABLY the HLSL curve: the same 3t^2 - 2t^3 Hermite
//  on the same saturated parameter, INCLUDING the descending-edge case (e0 > e1), which
//  WaveCurrentAmp relies on and which a "sorted edges" implementation would silently invert.
//  In their own namespace because `Smoothstep` and `Clamp` are names other translation units
//  already spell for themselves; nothing here may collide with them.
// ------------------------------------------------------------------------------------------------
namespace wt {

inline double Min(double a, double b) {
    if (a < b) return a;
    return b;
}

inline double Max(double a, double b) {
    if (a > b) return a;
    return b;
}

inline double Clamp(double x, double lo, double hi) {
    return Min(Max(x, lo), hi);
}

inline double Saturate(double x) {
    return Clamp(x, 0.0, 1.0);
}

// HLSL smoothstep verbatim.  Note the division by (e1 - e0) with NO ordering assumption: when
// e0 > e1 the ramp runs backwards, which is exactly how the blocking indicator is written.
inline double Smoothstep(double e0, double e1, double x) {
    const double t = Saturate((x - e0) / (e1 - e0));
    return t * t * (3.0 - 2.0 * t);
}

inline double Lerp(double a, double b, double t) {
    return a + t * (b - a);
}

}  // namespace wt

// ================================================================================================
//  1. BandPhaseSpeed -- shaders/Jet.hlsli:23.  c(k, h) = sqrt( g/k * tanh(k h) ).
//
//  The linear dispersion relation for a band's REPRESENTATIVE wavenumber (the geometric mean of
//  the band's two spectral cuts; SeaLayer/WaterBankLayer build it as sqrt(kCut[c]*kCut[c+1]),
//  giving k = 0.0295, 0.234, 2.84 /m -- 213 m swell, 27 m wind sea, 2.2 m chop).  It is exact
//  linear theory, not a fit: sigma^2 = g k tanh(k h), c = sigma / k.
//
//  Its two limits are the whole physical content, and proofs/water_terms.py measures both:
//    kh >> 1 (deep):    tanh -> 1,      c -> sqrt(g/k)   -- depth cannot be felt.
//    kh << 1 (shallow): tanh(kh) -> kh, c -> sqrt(g h)   -- wavenumber cannot be felt.
//
//  THE 7-FOOT-STANDING-WAVE TERM, carried over from the shader comment because it is the reason
//  this function is on the CPU at all: in 4 m of water an 11 s swell slows from ~17 m/s to ~6,
//  so a 0.7-1 m/s ebb that deep water shrugs off drives the SWELL band a third of the way to
//  blocking right where the bar is shallow -- which is exactly where the entrance stands up and
//  breaks.  A hull that used the deep-water c there would feel no current effect at all.
// ================================================================================================
inline double BandPhaseSpeed(double kBand, double depth) {
    const double kh = kBand * wt::Max(depth, kDepthFloorM);
    return std::sqrt(kGWave / kBand * std::tanh(kh));
}

// ================================================================================================
//  2. ShoalFactor -- shaders/Jet.hlsli:29.  Green's law: amplitude grows as the group speed
//  drops entering shallow water, because the ENERGY FLUX E*c_g through the band is conserved
//  and E ~ a^2, so a ~ c_g^(-1/2).
//
//      c_g = (c/2) (1 + 2kh / sinh 2kh),  S = sqrt( c_g^deep / c_g ),  clamped [0.75, 1.7].
//
//  NOTE WHICH c_g^deep.  The reference is the deep-water group speed AT THE SAME WAVENUMBER,
//  (1/2)sqrt(g/k), not at the same period.  That is the shader's choice and it is the right one
//  here: the bank holds a fixed representative k per band and shoals THAT band, so holding k is
//  what keeps the CPU and GPU talking about the same wave.  The consequence is worth stating
//  because it is easy to misread as a bug: at fixed k the shallow asymptote is
//  S -> (1/sqrt 2) * (k h)^(-1/4) -- the h^(-1/4) exponent of classical Green's law, but with a
//  1/sqrt(2) offset that the constant-PERIOD form does not have.  proofs/water_terms.py pins
//  both the exponent and the offset, and the exponent is the part that is physics.
//
//  THE CLAMPS.  [0.75, 1.7] is an engineering closure, and the two ends are not the same kind
//  of thing:
//    * 1.7 (upper) BINDS in real water and is doing work.  At the 0.15 m floor the swell band's
//      kh is 0.0044, for which the unclamped law asks for x2.75 -- an amplitude no wave that
//      shallow can carry, because it broke long before (the bank's own |eta| <= 0.55 h breaking
//      clamp is the term that should own it).  1.7 is where this law is told to stop and let
//      breaking take over.
//    * 0.75 (lower) does NOT bind: the unclamped law's minimum over all kh is the classical
//      shoaling DIP (waves shoal slightly DOWN before they shoal up, near kh ~ 2), and that
//      minimum is around 0.93, comfortably inside the clamp.  It is a guard against a caller
//      handing in a nonsense k or h, not a shaping constant.  proofs/water_terms.py MEASURES
//      the minimum rather than asserting it, so the day it moves the claim fails.
//
//  The 0.05 floor under c_g and the 1e-3 floor under sinh(2kh) are the same species of guard:
//  both denominators go to zero only at h = 0, which the depth floor already excludes.
// ================================================================================================
inline double ShoalFactor(double kBand, double depth) {
    const double kh = kBand * wt::Max(depth, kDepthFloorM);
    const double th = std::tanh(kh);
    const double c = std::sqrt(kGWave / kBand * th);
    const double cg = 0.5 * c * (1.0 + 2.0 * kh / wt::Max(std::sinh(2.0 * kh), 1e-3));
    const double cgDeep = 0.5 * std::sqrt(kGWave / kBand);
    return wt::Clamp(std::sqrt(cgDeep / wt::Max(cg, 0.05)), 0.75, 1.7);
}

// ------------------------------------------------------------------------------------------------
//  What WaveCurrentAmp answers with.  Two numbers that are NOT interchangeable: `amp` multiplies
//  the band's height, `blocked` drives breaking FOAM.  The shader returns them as a float2 and
//  its callers index .x and .y; a port that kept a bare pair would let a caller swap them and
//  still compile, which is the whole reason this is a named struct.
// ------------------------------------------------------------------------------------------------
struct WaveCurrentGain {
    double amp;      // amplitude multiplier on the band, dimensionless, in [0.55, 2.0]
    double blocked;  // 0 = free, 1 = fully blocked; the breaking-foam driver, never an amplitude
};

// ================================================================================================
//  3. WaveCurrentAmp -- shaders/Jet.hlsli:41.  Wave action conservation over a COLLINEAR current.
//
//  r = U . dHat / c0 is the current projected on the wave's own direction, in units of the
//  still-water phase speed.  Linear theory (deep water) gives the relative phase speed as the
//  root of c'^2 = c0 c' + ... i.e.
//      c'/c0 = (1 + sqrt(1 + 4r)) / 2                                              (exact)
//  and action transport A/A0 = 1/sqrt( (c'/c0)^2 (2 c'/c0 - 1) )                   (exact).
//  GaTest.cpp block 6 already pins that this closed form IS 1/sqrt(cr (cr + 2r)), the action
//  form, to 1e-12 -- so the two square roots below are theory, not a curve fit.
//
//  BLOCKING AT r = -1/4 IS EXACT.  Opposing current (r < 0) shortens and steepens the wave; at
//  r = -1/4 the discriminant 1 + 4r vanishes, c' = c0/2, the group speed hits zero against the
//  stream and the wave cannot propagate any further upstream.  The amplification diverges there.
//  Real water does not diverge -- it BREAKS.  Hence:
//
//    * blocked = smoothstep(-0.16, -0.245, r).  An engineering closure (ALGEBRA.md `physics`
//      says so in as many words): the band [-0.16, -0.245] is where the growth becomes steep
//      enough that breaking, not amplification, is the honest answer.  -0.245 stops just short
//      of the exact -0.25 singularity.
//    * the 1.45 saturation.  Also a closure: past the band the answer is pinned at 1.45 and ALL
//      the remaining energy is handed to foam.  1.45 is a tuned level, not a derived one; the
//      source records it as a closure and this port does not pretend otherwise.
//    * clamp [0.55, 2.0] on the amplification itself, for the same reason at both ends.
//
//  THREE GUARDS BELOW ARE DEAD CODE, AND SAYING SO IS THE POINT.  Because the r <= -0.245 branch
//  returns first, the clamp's -0.2499 floor, the 1e-4 floor under (1 + 4rc) and the 1e-3 floor
//  under cr^2(2cr - 1) are never reached with a binding value: at the tightest surviving r the
//  three arguments are 0.020 and 0.046, four and one and a half orders above their floors.
//  They are belt-and-braces against a future edit moving the early-out, not physics, and
//  proofs/water_terms.py measures the margin instead of taking that on trust.  The 4.0 upper
//  clamp on r is the same: the 0.55 amplitude floor already binds by r ~ 0.51.
//
//  max(c0, 0.5) IS a real domain guard: c0 is a band phase speed, and in vanishing depth it
//  goes to zero, which would send r to infinity for any current at all.
//
//  Arguments are components rather than a vector type on purpose -- this header owes nothing to
//  any 2-vector class, and a wrong-basis mistake is a compile error at the call site, not here.
// ================================================================================================
inline WaveCurrentGain WaveCurrentAmp(double curX, double curZ, double peakDirX, double peakDirZ,
                                      double c0) {
    const double r = (curX * peakDirX + curZ * peakDirZ) / wt::Max(c0, 0.5);
    const double blocked = wt::Smoothstep(-0.16, -0.245, r);
    if (r <= -0.245) return {1.45, 1.0};
    const double rc = wt::Clamp(r, -0.2499, 4.0);
    const double q = std::sqrt(wt::Max(1.0 + 4.0 * rc, 1e-4));
    const double cr = 0.5 * (1.0 + q);
    const double amp = wt::Clamp(1.0 / std::sqrt(wt::Max(cr * cr * (2.0 * cr - 1.0), 1e-3)),
                                 0.55, 2.0);
    return {wt::Lerp(amp, 1.45, blocked * blocked), blocked};
}

// ------------------------------------------------------------------------------------------------
//  One vessel, exactly the eight floats the bank's constant buffer carries per boat
//  (WaterBank.hlsl gBoatA/gBoatB; src/scene/WaterBankLayer.h:159).  Same fields, same units,
//  same meanings -- if the GPU table and this struct ever disagree about a field, the wake the
//  hull feels is not the wake on screen.
// ------------------------------------------------------------------------------------------------
struct WakeVessel {
    double x = 0.0;            // gBoatA.x -- world east, metres
    double z = 0.0;            // gBoatA.y -- world north, metres
    double heading = 0.0;      // gBoatA.z -- radians; fwd = (cos hd, sin hd) in the (x, z) plane
    double speed = 0.0;        // gBoatA.w -- speed over ground, m/s.  K0 = g/U^2, so this is the
                               //             single parameter that sets the wake's whole scale.
    double wakeAmp = 0.0;      // gBoatB.x -- base wake amplitude, metres
    double hullHalfLen = 0.0;  // gBoatB.y -- hull half-length, metres
    bool enabled = false;      // gBoatB.z >= 0.5 -- an off boat contributes nothing at all
};

// ------------------------------------------------------------------------------------------------
//  What a sample point accumulates.  eta and the two slope components SUM over vessels
//  (superposition: linear wave theory, and the shader's own comment notes there is no
//  interaction to resolve).  akMax and stern are MAXIMA, not sums, and that difference is
//  physical: steepness and prop wash both feed the foam UNION -- two wakes crossing do not make
//  water twice as foamy, the foamier one wins.
// ------------------------------------------------------------------------------------------------
struct WakeSample {
    double eta = 0.0;     // surface displacement, metres (SUM over vessels)
    double slopeX = 0.0;  // d(eta)/dx, world east  (SUM)
    double slopeZ = 0.0;  // d(eta)/dz, world north (SUM)
    double akMax = 0.0;   // largest steepness a*k contributed (MAX) -- the foam union's input
    double stern = 0.0;   // stern turbulence envelope, 0..1 (MAX) -- aerated prop wash
};

// ================================================================================================
//  4a. WakeBranch -- shaders/WaterBank.hlsl:181.  ONE stationary-phase branch of the Kelvin wake.
//
//  Station keeping is the whole derivation: a wave whose crest stands still relative to the ship
//  must have c = U cos(theta), theta measured from the track, so with deep-water c^2 = g/k,
//      k = K0 sec^2(theta),   K0 = g/U^2.
//  `t` is tan(theta) at a stationary root of the phase; WakeOne solves for it.  Everything here
//  is that root evaluated.
//
//  THE SIGNED STATIONARY PHASE (ALGEBRA.md `wake`; this is the corrected form):
//      phi = K0 sec(theta) * (xi - zeta |t|)
//  The stationary roots are NEGATIVE for xi, zeta > 0, so the phase AT them carries a minus
//  sign, and the expression is already mirror-symmetric across the track -- which is why `zeta`
//  is |across| and `side` carries the sign separately.  The reference shader instead folded |t|
//  in with a PLUS, phi = K0(xi sec + zeta sec |t|), evaluating the phase at +|t|: an angle that
//  is not stationary.  The test that separates them is the envelope theorem, which says
//  |grad phi| = k EXACTLY at a stationary point; measured by finite differences the folded form
//  runs 1.03-6.65 instead.  proofs/water_terms.py re-runs that measurement on this file.
//
//  ...and the SLOPE DIRECTION IS THE SAME OBJECT.  d = grad(phi)/k = -(cos th * fwd
//  + sin th * side * rgt).  It is not an independent choice: eta = a cos(phi) forces
//  grad(eta) = -a k sin(phi) d, which is exactly the line below.  phi and d flip together, so
//  eta is even in the pair and the analytic slope really is the derivative of the analytic eta.
//  (In the reference they were not, and the shading disagreed with the geometry.)
//
//  THE FOUR BOUNDS, each an engineering closure with its argument, per ALGEBRA.md `wake`:
//
//    * MESH-NYQUIST BAND LIMIT, w = smoothstep(2, 5, lambda / sampleM).  A wave is only real if
//      the surface carrying it can hold it, and the surface here is the MESH, never the pixel
//      grid: sampleM is the tile's texel size in metres.  Below two samples per wavelength the
//      term is pure alias and is cut entirely; it fades in over 2..5 samples.  This is why the
//      caller must pass its own texel size and not a constant.
//
//    * DIVERGENT DAMPING, exp(-(k/(5 K0))^2).  The divergent branch runs off to arbitrarily
//      large k (sec^2 theta is unbounded as the root steepens), and short gravity waves are
//      removed by viscosity and by the wind long before they reach the far field.  The Gaussian
//      cut at k = 5 K0 is the closure; 5 is TUNED (the source states the mechanism, not a
//      derivation of the number).
//
//    * STEEPNESS CAP, a*k <= 0.30.  A gravity wave steeper than roughly ak ~ 0.44 (the Miche /
//      Stokes limiting steepness) cannot exist -- it breaks.  0.30 is inside that family, the
//      pre-breaking bound; past it the energy belongs to foam, and the bank sends akMax to
//      exactly that union.  Enforced as an amplitude ceiling aMaxK = 0.30/k, which is why the
//      cap tightens automatically for the short divergent waves.
//
//    * the 1e-3 and 1e-6 floors under sampleM, k and K0 are division guards, nothing more.
// ================================================================================================
inline void WakeBranch(double t, double K0, double xi, double zeta, double amp, double sampleM,
                       double fwdX, double fwdZ, double rgtX, double rgtZ, double side,
                       WakeSample& acc) {
    const double sec = std::sqrt(1.0 + t * t);
    const double k = K0 * sec * sec;                          // station keeping: k = K0 sec^2 th
    const double lam = 6.283185307179586 / k;
    // the mesh's Nyquist, not the pixel's: below 2 samples per wavelength this term is alias
    const double w = wt::Smoothstep(2.0, 5.0, lam / wt::Max(sampleM, 1e-3));
    if (w <= 0.0) return;
    const double tAbs = std::abs(t);
    // THE SIGNED STATIONARY PHASE.  A '+' here is the reference's bug, and it is the one edit
    // proofs/water_terms.py makes to produce its negative control.
    const double ph = K0 * sec * (xi - zeta * tAbs);
    const double ct = 1.0 / sec;                              // cos theta
    const double st = tAbs / sec;                             // sin theta
    // d = grad(ph)/k.  Consistent with ph by construction -- see the note above.
    const double dX = -(fwdX * ct + rgtX * (side * st));
    const double dZ = -(fwdZ * ct + rgtZ * (side * st));
    const double kRel = k / wt::Max(K0, 1e-6);
    // viscous/wind removal of the short divergent arm; Gaussian cut at k = 5 K0 (TUNED width)
    const double damp = std::exp(-(kRel / 5.0) * (kRel / 5.0));
    // pre-breaking steepness ceiling ak <= 0.30 (Miche family); past it the energy is foam's
    const double aMaxK = 0.30 / wt::Max(k, 1e-3);
    const double a = wt::Min(amp * w * damp, aMaxK);
    acc.eta += a * std::cos(ph);
    acc.slopeX -= (a * k) * std::sin(ph) * dX;
    acc.slopeZ -= (a * k) * std::sin(ph) * dZ;
    acc.akMax = wt::Max(acc.akMax, a * k);
}

// ================================================================================================
//  4b. WakeOne -- shaders/WaterBank.hlsl:202.  One vessel's whole Kelvin wake at one point.
//
//  THE WEDGE IS A DISCRIMINANT, NOT A CONSTANT.  With xi metres astern and zeta metres abeam,
//  phi(theta) = K0(xi sec + zeta sec tan) is stationary where
//      2 zeta t^2 + xi t + zeta = 0,      t = tan(theta),
//  whose roots are real only while xi^2 - 8 zeta^2 >= 0, i.e. |zeta/xi| <= 1/(2 sqrt 2).  That
//  is the famous 19.4712-degree half-angle -- atan(1/(2 sqrt 2)) = arcsin(1/3) to the last bit
//  -- and NOTHING IN THIS FUNCTION IMPOSES IT.  It falls out of `disc <= 0`.  Vieta gives
//  t+ t- = zeta/(2 zeta) = 1/2 for free, and the two roots merge at the wedge edge on the double
//  root t = -1/sqrt(2), sec^2 = 3/2, so the cusp wave is k = 1.5 K0, lambda = (2/3) lambda_t.
//  Speed is the entire character of the picture: lambda_t = 2 pi U^2 / g, which for this
//  engine's AIS per-class speeds (SceneConfig.h:115-119, 4.86 / 4.62 / 3.83 / 3.34 m/s) is
//  15.1 / 13.7 / 9.4 / 7.1 m.  proofs/water_terms.py measures all of that off this code.
//
//  Solving the quadratic IS the stationary-phase method, exactly -- it is not an approximation
//  to a theta quadrature.  Brute-force quadrature fails STRUCTURALLY out here: far-field phase
//  decorrelation needs O(K0 R) samples, so at R = 1200 m it would want thousands per point.
//
//  THE AMPLITUDE CLOSURES, each with what is known about it:
//
//    * 1/sqrt(0.6 + dRel) SPREADING.  The sqrt is physics: a wave system spreading in two
//      dimensions conserves energy over a growing front, E ~ 1/r, so amplitude ~ r^(-1/2).  The
//      0.6 offset is a regulariser that keeps it finite at the hull -- TUNED.
//
//    * NEAR FADE, smoothstep(0.5, 2.5, dRel), in hull half-lengths.  Linear theory dies at the
//      hull: within a hull length the water is being pushed aside by a solid body, not carrying
//      a free wave, and the stationary-phase far field is simply not the right description.
//      This fades the whole model in as it becomes true.
//
//    * CUSP BOOST, faded in over dRel 1..3.5.  At the wedge edge sq -> 0, the two branches merge
//      and stationary phase degenerates (the second derivative vanishes: an Airy caustic, not a
//      Gaussian), so the true amplitude there is higher than either branch alone gives.
//      saturate(1 - sq/xi)^3 is 1 exactly at the edge and falls away inside.  The 1.8 gain and
//      the cubic are TUNED; the fade over dRel 1..3.5 is Airy validity -- the caustic
//      description only holds once you are several hull lengths out.
//
//    * DRAUGHT AMP CAP, ampCap = 0.22 * halfLen * 0.34.  A displacement hull cannot make a wave
//      taller than the water it is displacing.  0.34*halfLen is the hull's draught/half-beam
//      proxy (the same 0.34 the stern envelope uses for width) and 0.22 is the fraction of it
//      allowed as wake amplitude.  UNEXPLAINED IN THE SOURCE: neither shaders/WaterBank.hlsl nor
//      ALGEBRA.md derives 0.22 or 0.34; both are named ("draught amp cap") and TUNED.  They are
//      carried here unchanged rather than re-derived, and this comment says so.
//
//    * GROWTH, smoothstep(0, 2*halfLen, xi).  The wake is generated ALONG the hull, so it is not
//      at full strength until a hull length or so astern of the bow.
//
//    * DIVERGENT BRANCH x 0.85.  TUNED.  The source gives no argument; physically the divergent
//      arm is the weaker-looking one in photographs, but 0.85 is a number, not a derivation.
//
//    * 1200 m CULL.  "spread + damp are dead past this" -- at 1200 m, dRel for a 7.5 m hull is
//      160, so spreading alone is down to 1/12.7 and the term is below the amplitude early-out.
//
//  THE STERN TURBULENCE TERM is a different animal and is labelled as such in the shader: the
//  aerated prop wash, a short narrow decaying envelope, "the reference's tuned closure".  Every
//  number in it (0.34 width, the 1/10 widening rate, the 0.3 start, the 3.2 decay length, the
//  0.45/1.15 edge taper) is TUNED, all in hull half-lengths.  It joins the foam UNION -- hence
//  MAX, never a sum -- and unlike the wake proper it is NOT gated on crests: prop wash cares
//  nothing for where the crest is.  It is also evaluated for ANY xi > 0, before the bow gate and
//  before the wedge test, because it exists astern regardless of whether a stationary phase does.
//
//  DOMAIN GUARDS, not physics: halfLen >= 1 m and dist >= 1 m (both are divisors); xi > 0.5 m
//  (nothing ahead of the bow); zeta < 1e-3 collapses to the single on-track root t = 0 rather
//  than dividing by zero; amp < 1e-4 m is below anything the surface can show.
// ================================================================================================
inline void WakeOne(const WakeVessel& v, double px, double pz, double sampleM, WakeSample& acc) {
    // an off boat, or one barely moving: K0 = g/U^2 diverges as U -> 0 and there is no steady
    // wake to speak of below half a metre per second anyway
    if (!v.enabled) return;
    if (v.speed < 0.5) return;
    const double fwdX = std::cos(v.heading);
    const double fwdZ = std::sin(v.heading);
    const double rgtX = -fwdZ;                          // fwd rotated +90 deg in the (x, z) plane
    const double rgtZ = fwdX;
    const double rX = px - v.x;
    const double rZ = pz - v.z;
    if (rX * rX + rZ * rZ > 1200.0 * 1200.0) return;    // spread + damp are dead past this
    const double xi = -(rX * fwdX + rZ * fwdZ);         // metres ASTERN (positive behind)
    const double across = rX * rgtX + rZ * rgtZ;        // metres abeam, signed
    double side = 1.0;
    if (across < 0.0) side = -1.0;                      // the mirror sign, kept out of the phase
    const double zeta = std::abs(across);
    const double halfLen = wt::Max(v.hullHalfLen, 1.0);
    // stern turbulence envelope (the aerated prop wash): short, narrow, decaying -- the
    // reference's TUNED closure, every constant in hull half-lengths; joins the foam UNION,
    // never adds, and is not crest-gated.
    if (xi > 0.0) {
        const double wid = halfLen * 0.34 * (1.0 + xi / (halfLen * 10.0));
        const double sN = wt::Smoothstep(0.0, halfLen * 0.3, xi) *
                          std::exp(-xi / (halfLen * 3.2)) *
                          (1.0 - wt::Smoothstep(wid * 0.45, wid * 1.15, zeta));
        acc.stern = wt::Max(acc.stern, sN);
    }
    if (xi <= 0.5) return;                              // nothing ahead of the bow
    // THE WEDGE: real stationary roots need xi^2 - 8 zeta^2 >= 0.  19.4712 deg is this line.
    const double disc = xi * xi - 8.0 * zeta * zeta;
    if (disc <= 0.0) return;                            // outside the wedge: no stationary phase
    const double sq = std::sqrt(disc);
    const double K0 = kGWave / (v.speed * v.speed);     // the ONE scale: lambda_t = 2 pi / K0
    const double dist = wt::Max(std::sqrt(rX * rX + rZ * rZ), 1.0);
    const double dRel = dist / halfLen;                 // distance in hull half-lengths
    // Airy caustic at the wedge edge, where sq -> 0 and the two branches merge (1.8 and the
    // cube are TUNED); faded in over dRel 1..3.5 because the caustic description needs far field
    const double cuspFar = 1.0 + 1.8 * std::pow(wt::Saturate(1.0 - sq / wt::Max(xi, 1e-3)), 3.0);
    const double cusp = 1.0 + (cuspFar - 1.0) * wt::Smoothstep(1.0, 3.5, dRel);
    const double nearFade = wt::Smoothstep(0.5, 2.5, dRel);   // linear theory dies at the hull
    const double ampCap = 0.22 * halfLen * 0.34;              // draught amp cap (TUNED, see above)
    const double spread = 1.0 / std::sqrt(0.6 + dRel);        // 2-D energy spreading, a ~ r^-1/2
    const double amp = wt::Min(v.wakeAmp * wt::Smoothstep(0.0, halfLen * 2.0, xi) * cusp * spread,
                               ampCap) * nearFade;
    if (amp < 1e-4) return;
    if (zeta < 1e-3) {
        // on the track itself the quadratic degenerates to the single transverse root t = 0
        WakeBranch(0.0, K0, xi, zeta, amp, sampleM, fwdX, fwdZ, rgtX, rgtZ, side, acc);
        return;
    }
    // 2 zeta t^2 + xi t + zeta = 0.  t1 is the TRANSVERSE branch (small |t|), t2 the DIVERGENT
    // one; Vieta gives t1*t2 = 1/2 identically, which proofs/water_terms.py checks.
    const double t1 = (-xi + sq) / (4.0 * zeta);
    const double t2 = (-xi - sq) / (4.0 * zeta);
    WakeBranch(std::abs(t1), K0, xi, zeta, amp, sampleM, fwdX, fwdZ, rgtX, rgtZ, side, acc);
    WakeBranch(std::abs(t2), K0, xi, zeta, amp * 0.85, sampleM, fwdX, fwdZ, rgtX, rgtZ, side,
               acc);
}

}  // namespace ga
