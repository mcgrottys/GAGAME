// ================================================================================================
//  OceanCpu - the three FFT wave cascades, evaluated on the CPU, anywhere on the planet.
//
//  WHY THIS FILE EXISTS.  A hull needs the sea surface where the hull is, this frame, in doubles:
//  the heave under each station, the horizontal (choppy) offset that decides WHICH water the bow
//  is actually in, the SLOPE that turns that point into a free-surface normal, and the water's
//  own VELOCITY, without which every drag force is computed against a sea standing perfectly
//  still.  The cascades that carry that sea live only as GPU textures written by
//  shaders/OceanCompute.hlsl.  Inside WaveField's solved window the CPU has ProbeAt; OUTSIDE it --
//  which is to say everywhere in open ocean, which is almost everywhere -- the CPU knows nothing
//  about the sea at all.  This file closes that.  The alternative, reading the displacement
//  textures back, costs a fence per frame and then hands the physics a frame-late, fp16,
//  bilinearly-blurred answer that is worse than the one computed here from first principles.
//
//  WHY IT IS POSSIBLE AT ALL.  The cascade sea is PARAMETRIC, not data.  Two facts make it so:
//
//    * BinVariance(k) (OceanCompute.hlsl:100) is CLOSED FORM -- JONSWAP wind sea plus Gaussian
//      swells under cos^2s spreading -- with each bin's frequency and angular extent INTEGRATED by
//      erf rather than point-sampled.  Nothing is read from a texture to evaluate it.
//    * GaussianPair(texel) (OceanCompute.hlsl:65) is a pure integer pcg2d hash of the texel index,
//      the forecast seed and the cascade index.  It is reproducible exactly, anywhere, forever.
//
//  THE TRAP THIS AVOIDS.  Because of those two facts a truncated direct sum here reproduces THE
//  SAME REALIZATION the GPU draws: the same individual crests, in the same places, at the same
//  instants -- not a statistically similar sea.  That distinction is the whole point.  A sea built
//  from a fresh PRNG, or from a re-derived spectrum, would have the right Hs and the right
//  spectrum and still be a DIFFERENT sea: the hull would pitch over a crest the player is looking
//  into the back of, and no amount of tuning would ever fix it, because nothing would be wrong.
//  The identity is the integer hash; proofs/ocean_cpu.py pins it bit-for-bit, and pins the summed
//  field against an actual inverse FFT of the same packed spectrum (the GPU's own algorithm).
//
//  WHAT IS SUMMED.  Per cascade the GPU holds a 256^2 grid of spectral bins.  For every bin with
//  nonzero variance hk(-k) = conj(hk(k)) exactly, so the bins come in CONJUGATE PAIRS whose joint
//  contribution to the real field is 2*Re[hk(k) e^{ik.x}].  This class retains PAIRS, largest
//  variance first, until the dropped variance is under kResidualTarget of the cascade's total or
//  the count reaches kMaxBins -- and then REPORTS what it actually dropped (see
//  ResidualVarianceFraction; the plan requires that be measured, never asserted).  Summing pairs
//  rather than bins is not an optimisation: it is what keeps the reconstructed field exactly real.
//
//  (Why the pairing is exact and needs no special case: WaveK's mirror texel negates k for every
//  index except 128, whose signed value -128 is its own mirror.  The smallest |k| on those two
//  lines is exactly the grid's Nyquist wavenumber pi*N/L, and every cascade's kHi is the
//  0.9*pi*N/L GUARD BAND, strictly below it -- so BinVariance returns zero there and they never
//  enter the list.  SetSeaState tests kHi <= pi*N/L rather than trusting it, and logs if a future
//  band ever crosses the guard.)
//
//  THE SIGN LEDGER.  Every one of these was read off the HLSL and then CONFIRMED numerically in
//  proofs/ocean_cpu.py gate 4b, which compares this sum against N^2 * numpy.fft.ifft2 of the same
//  packed spectrum and reports how far each deliberately-flipped variant lands:
//
//    space   e^{+i k.x}.  CsFft's twiddle is (cos ang, sin ang) with ang = +2*pi*(i%hl)/len -- the
//            synthesis sign -- un-normalised, so f(n) = sum_m C(m) e^{+2*pi*i*m.n/N}; with
//            WaveK's k = 2*pi/L * s and the texel's world position x = n*L/N, 2*pi*m.n/N IS k.x.
//            Flipping it moves h by 0.586 m against a 0.490 m rms field, 4.0x the truncation floor.
//    time    e^{+i w t}.  CsModulate: rot = (cos wt, sin wt), and
//            hk = h0(k)*rot + conj(h0(-k))*conj(rot).  A 3 s offset moves h by 1.352 m, 9.2x.
//    chop    -i k^ .  CsModulate writes dx = float2(hk.y, -hk.x) * kn.x, and (b, -a) is exactly
//            (-i)*(a + ib).  Flipping it moves Dx by 1.361 m, 9.3x the floor.
//    pair    the factor 2 on every retained bin, because the list holds one texel of each
//            conjugate pair.  Halving it moves h by 0.367 m, 2.5x the floor.
//    lambda  horizontal ONLY.  CsAssemble writes float4(fLambda*dx, h, fLambda*dz, 0) -- the
//            height channel is never scaled by the choppy constant.
//    layout  the packed complex C = h_hat + i*Dx_hat lands h in .re and Dx in .im, which is why
//            the disp texture is (Dx, h, Dz) and why Displacement returns it in that order.
//    slope   +i k, and it is the SPATIAL sign ALONE.  CsModulate writes hx = float2(-hk.y, hk.x)
//            * k.x, and (-b, a) is exactly (+i)*(a + ib), so hx_hat = i*kx*hk -- carried to real
//            space by the same e^{+ik.x} synthesis as the height.  The rotor cannot enter it:
//            d/dx of hk*e^{+ik.x} brings down +i*kx because x appears in the phasor and nowhere
//            else, so 2026-09-08's e^{+iwt} -> e^{-iwt} moved the VALUE of every slope and not
//            its sign.  (Had CsFft synthesised with e^{-ik.x}, the factor would be -i*kx; that
//            pairing is what the sign is read from, not the rotor.)  Flipping it, and dropping
//            the k, are proofs/ocean_cpu.py gate 6's two negative controls.
//
//  TWO CONSEQUENCES OF THOSE CONVENTIONS, MEASURED RATHER THAN ASSERTED (proofs/ocean_cpu.py's
//  REPORT block prints both, and they are properties of the GPU shader, faithfully mirrored here,
//  NOT choices made by this file):
//
//    * A bin at wavevector k travels toward -k^.  With e^{+i(k.x + wt)} the phase fronts move
//      against k, so the synthesised sea runs OPPOSITE to each partition's dirTo.  The proof
//      cross-correlates a Tp 12 s swell built with dirTo = +east one second apart and measures the
//      pattern moving 19.6 m/s WEST (deep-water c at that peak is 18.7 m/s).  Tessendorf's own
//      equations have this property and it is invisible under his |k^.w^|^2 spectrum, which is
//      symmetric in k; the cos^2s lobe here is not, so here it shows.
//    * lambda*D points AWAY from crests, so the choppy term BROADENS crests and sharpens troughs.
//      Measured on the whole field: the Jacobian CsAssemble writes averages 1.028 over the top
//      decile of h and 0.972 over the bottom, i.e. area EXPANDS at crests -- the opposite of the
//      1 - s*a*k that ALGEBRA.md `caustics` states -- so the foam channel fires in the troughs.
//
//  Both are stated so that whoever fixes them in the shader fixes them here in the same commit;
//  what must never happen is the two drifting apart, because then the hull leaves the render.
//
//  KNOWN, DELIBERATE DIFFERENCES FROM WHAT THE GPU DISPLAYS (all sub-texel, all measured):
//    * The GPU stores the disp texture as fp16 and reads it with a bilinear tap; this evaluates
//      the continuous band-limited field in doubles.  This one is MORE accurate, not different.
//    * The sampler places texel n at uv = (n+0.5)/N while the transform puts that value at
//      x = n*L/N, so the RENDERED field is offset by half a texel, L/(2N) = 1.48 / 0.36 / 0.09 m
//      per cascade.  This class evaluates the unshifted field.  On cascade 0 that is 2.5 degrees
//      of phase on a 213 m wave.
//    * CsModulate computes w*fTime in fp32, and SeaLayer's fTime is seconds since the forecast
//      cycle (tens of thousands).  At t = 86400 s an fp32 product of order 1e5 rad has ~0.008 rad
//      of quantisation; this class carries it in doubles and will disagree by about that.
//    * The caller owns the cascade FADES and the world-space ADVECTION Sea.hlsl applies
//      (uv = (xz - adv)/L, CascadeFade, BandScale, the depth-limited breaking clamp).  This class
//      answers the one question the shader cannot: what is the undisturbed cascade field at this
//      point.  Anything shaping it belongs to the caller, and stays out of here.
//
//  THREADING: SetSeaState mutates, Displacement is const and touches nothing but its own immutable
//  bin lists -- safe to call from every physics thread at once, no locking, once the sea state is
//  set.
//
//  COST, measured (MSVC /O2, one core, 20000 calls per timing) on BOTH SIDES of the slope and
//  velocity being added -- the two binaries run INTERLEAVED, five runs of each, because this
//  machine is shared and timing one build after the other measures the load as much as the code.
//  Two independent interleaved sessions, median of 15 timings each:
//
//    Hs 2.5 m Gulf-of-Maine sea, 1319 retained pairs (164 + 166 + 989 across the cascades)
//        before, Displacement:  35.59 / 35.14 us   (spreads 35.21-37.43, 34.88-36.69)
//        after,  Sample:        40.35 / 39.81 us   =>  +13.4% and +13.3%
//        after,  Displacement:  40.40 / 39.80 us   -- it calls Sample, so it IS that query
//    Hs 6.5 m storm, 1374 pairs
//        before 37.88 / 37.37 us, after 42.47 / 42.44 us  =>  +12.1% and +13.6%
//
//  The RATIO is the measurement, ~13%; the absolute microseconds belong to the machine and moved
//  1.5% between the two sessions.  (An earlier run of the same pre-change code on a quieter
//  machine gave 36 us with a 35.8-37.2 spread -- the same number to within its own spread.)
//
//  That is ~30 ns per pair, and it is STILL two sincos: one for the rotor and one for the spatial
//  phasor.  The whole 13% is the second phasor Q and four more accumulators -- no transcendental
//  was added, which is why it is 13% and not 100%, and it is why a second PASS for the slope
//  would have cost the other 87%.  If a hull ever needs many stations per step, the ROTOR half is
//  the one to hoist -- e^{iwt} does not depend on
//  the query point, so a caller sampling P points at one instant can compute it once per bin
//  instead of P times and halve the transcendental count.
//
//  F21 (2026-10-06): THE MEASUREMENT CAME. At the helm the hull's step was 7.0 ms of a 17.2 ms CPU
//  frame ([cpu], FrameLoop): two 60 Hz steps a frame, each ~73 stations and tube slices, each
//  point up to four charts, every sample paying the rotor sincos per bin again at the same
//  instant. THE ROTOR IS HOISTED, per thread: a thread_local memo of (cos wt, sin wt) per bin,
//  keyed on (this, the bin list's epoch, tSec), built per cascade on first use at an instant and
//  read by every later sample at that instant. The values are the same expressions in the same
//  order, so every channel is bit for bit what the cold sum gives (RunOceanCpuSelfTest holds it,
//  and a planted instant off by a nanosecond is caught). Const stays const: the memo is the
//  thread's, not the object's; a thread never shares it, and a sea state's change moves the epoch.
// ================================================================================================
#pragma once

#include <atomic>
#include <cmath>
#include <cstdint>
#include <vector>

#include "sim/SeaState.h"

namespace ga {

// ------------------------------------------------------------------------------------------------
//  ONE POINT OF THE SEA -- everything a hull asks of it, from ONE pass over the retained bins.
//
//  Why one struct and not three calls: the cost of a query IS the two sincos per retained bin
//  (COST, above), and all eight numbers are real multiples of the same two phasors those sincos
//  build.  A second pass for the slope would have cost another 35 us to buy arithmetic already in
//  flight; carrying it in the first pass cost 4.7 us, measured either side (COST, above).
//  Nothing here is boat-shaped -- no station geometry, no fades, no policy; those belong to the
//  caller, as the last KNOWN DIFFERENCE above says.
//
//  SLOPE.  (sx, sz) = (dh/dx, dh/dz) of the height field with respect to the QUERY coordinate.
//  That is the shader's hx/hz channel exactly -- the one Sea.hlsl:316 turns into
//  normalize(float3(-hx, 1, -hz)) -- so a caller building a free-surface normal that way builds
//  the same vector the render lights with, and owns the cascade fades and the |slope| <= 1.1 clamp
//  Sea.hlsl applies just before it.
//
//  WHAT THE SLOPE IS NOT: the gradient with respect to WORLD position.  The choppy map sends
//  (x, z) to (x + lambda*Dx, z + lambda*Dz), so the exact tangents of the DRAWN surface carry the
//  chop's own gradient too, and the exact normal needs the Jacobian columns (Jxx, Jzz, Jxz).
//  Those fall out of the same phasor for no extra transcendental, and they are returned now (the
//  water match, step 3): the mesh draws the displaced surface through its tangent bivector
//  (GlobeMesh.hlsl, the fold guard), and --water-probe measured the hull reading the undisplaced
//  one 0.14 m low under half-metre waves. (jxx, jxz, jzz) = (dDx/dx, dDx/dz = dDz/dx, dDz/dz),
//  lambda in, like dx and dz.
//
//  PARTICLE VELOCITY.  (vx, vy, vz) is the water's velocity at the surface, m/s, of the particle
//  whose Lagrangian LABEL is (wx, wz) -- i.e. of the water drawn at (wx + dx, h, wz + dz).  It has
//  no shader equivalent, because the GPU never needed one, so it is DERIVED from linear
//  deep-water theory per bin rather than ported:
//
//    For one free wave of elevation eta = Re[H e^{i(k.x - wt)}] the potential satisfying Laplace
//    and the linearised free-surface conditions is phi = Re[-i (g H / w) e^{|k|y} e^{i(k.x - wt)}]
//    with y = 0 the mean surface.  w^2 = g|k| is EXACTLY the condition that makes d(phi)/dy at
//    y = 0 equal d(eta)/dt, so no dispersion assumption enters beyond the one Bin::w already
//    carries.  Then
//
//        horizontal   grad_xz phi = (g/w) k eta = w * k^ * eta        (g/w = w/|k| in deep water)
//        vertical     d(phi)/dy   = d(eta)/dt
//
//    Evaluating at y = 0 rather than at y = eta is a second-order difference, which is the order
//    the entire linear synthesis is written to; there is no depth factor to apply because
//    e^{|k| * 0} = 1.  The horizontal is IN PHASE with the elevation (fastest forward at the
//    crest, backward in the trough) and the vertical is in quadrature, which is what makes a deep
//    water orbit a circle of radius w*a -- gate 8 measures the circle rather than assuming it.
//
//  THE TRAP: a retained pair is not one wave, it is TWO.  hk = h0(+k) e^{-iwt} +
//  conj(h0(-k)) e^{+iwt}, and while the first term travels along +k, the second is a wave at
//  wavevector -k travelling along -k: 2 Re[conj(B) e^{i(k.x + wt)}] IS 2 Re[B e^{i((-k).x - wt)}].
//  Their k^ are OPPOSITE, so the pair's horizontal velocity is w*k^*(eta_A - eta_B), NEVER
//  w*k^*h.  A directional swell hides the difference (where A is large, B is nearly zero); a
//  cos^2 8 wind sea does not, so getting it wrong would have been a drag bias that appeared only
//  in a short sea and looked like a tuning problem.  The loop therefore carries a SECOND phasor Q
//  -- the same sum with the -k half negated -- for four extra multiplies and no extra
//  transcendental.  See OceanCpu.cpp.
//
//  WHAT THIS IS NOT: d/dt of the choppy position (x + lambda*Dx, h, z + lambda*Dz).  That would
//  inherit lambda, an artistic constant and not a fluid velocity, and it would inherit the chop's
//  sign -- which the ledger above records as pointing AWAY from crests, so for a single wave
//  d(lambda*Dx)/dt comes out as -lambda*w*k^*eta, exactly backward.  proofs/ocean_cpu.py MEASURES
//  that anti-correlation instead of asserting it, so the day the chop sign is fixed in the shader
//  the number moves and says so.
// ------------------------------------------------------------------------------------------------
struct OceanSample {
    double dx = 0, h = 0, dz = 0;   // displacement, m -- the disp texture's (Dx, h, Dz), lambda in
    double sx = 0, sz = 0;          // dh/dx, dh/dz, dimensionless -- the deriv texture's (hx, hz)
    double vx = 0, vy = 0, vz = 0;  // particle velocity at the surface, m/s -- no shader twin
    double jxx = 0, jxz = 0, jzz = 0;   // the lateral displacement's Jacobian, lambda in (above)
};

class OceanCpu {
public:
    static constexpr int kCascades = 3;      // OceanFft::kCascades
    // F21: the rotor memo (above). Off only in the selftest, which holds the memoed sum to the cold
    // one bit for bit. The counters are process-wide instruments: samples taken, and the instants a
    // thread built a cascade's rotors for ([cpu] at the exit).
    static bool s_rotorMemo;
    static std::atomic<uint64_t> s_samples, s_rotorBuilds;
    static constexpr int kN = 256;           // OceanFft::kN -- the spectral grid is kN x kN

    // Truncation policy. kResidualTarget is what we AIM to drop; kMaxBins is the hard cap that
    // keeps a storm from turning a query into a 32768-term sum.
    //
    // Why the cap sits where it does, measured rather than guessed. A cascade's band is an annulus
    // in a 256^2 grid, so how many bins CAN carry energy is fixed by the band, not the sea state:
    // cascade 0's band is a 12.6-bin-radius disc (248 pairs), cascade 1's is 3.1..15.5 bins (360),
    // and cascade 2's runs from 3.9 bins out past the grid edge (20822). Cascade 2 is the only one
    // that could ever be expensive, and its shell energy falls as k^-2 (a JONSWAP f^-5 tail
    // through f ~ sqrt(k)), so 99% of it lives in the inner ~1000 pairs regardless of amplitude:
    // 989 for a Hs 2.5 m sea, 1146 for a Hs 6.5 m storm. 4096 is therefore ~4x clear of anything
    // realistic while still bounding the pathological case -- and if it ever DOES bind, the
    // residual this class reports says so out loud instead of silently costing accuracy.
    static constexpr double kResidualTarget = 0.01;
    static constexpr int kMaxBins = 4096;

    // Mirrors OceanFft::SetSeaState, plus the cascade geometry that class keeps private.
    // patchL/kLo/kHi come from OceanFft (m_patchL, m_bandLo, m_bandHi, src/core/OceanFft.cpp:
    // 153-158) -- passed in rather than duplicated, because two copies of a band cut are two
    // copies that can disagree. lambda is OceanFft::m_lambda, which has no accessor yet; the
    // default is that member's value and the parameter exists so it can stop being a default the
    // day one is added.
    //
    // parts/count are the live partition set (SeaState::BuildParams). count is clamped to 4, as
    // OceanFft's constant buffer clamps it. seed keys the Gaussian draw and MUST be the same value
    // OceanFft was given, or this evaluates a different ocean than the one on screen.
    void SetSeaState(const PartParam* parts, int count, uint32_t seed, const float patchL[3],
                     const float kLo[3], const float kHi[3], double lambda = 1.1);

    // Displacement at a world point (x east, z north, metres) and time. tSec is seconds since the
    // forecast cycle, i.e. exactly the fTime OceanFft::Record is handed. out = (Dx, h, Dz), the
    // same channel order and the same lambda scaling as the GPU's disp texture: out[1] is the
    // unscaled surface elevation, out[0]/out[2] are the choppy horizontal offsets.
    //
    // The sampled surface point is (wx + out[0], out[1], wz + out[2]) -- the horizontal terms move
    // the water, they do not describe a slope. A hull station that wants "the height under this
    // world point" must therefore iterate, because the map is not the identity; one Newton step is
    // usually enough at kLambda 1.1 and that is the caller's business, not this file's.
    void Displacement(double wx, double wz, double tSec, double out[3]) const;

    // Displacement, slope and particle velocity, from ONE pass over the same bins. See the
    // OceanSample block above for what each channel means and exactly where its sign comes from.
    // Displacement() is this call with five of the eight numbers dropped, and costs what it costs:
    // there is no cheaper query, because the transcendentals are shared by all eight.
    void Sample(double wx, double wz, double tSec, OceanSample& out) const;

    // THE HULL'S QUERY. Identical to Sample except that waves shorter than `minLambda` are not
    // summed at all.
    //
    // This is a physical statement, not a speed hack, and the two happen to coincide. A hull
    // integrates PRESSURE OVER AREA: a 5.5 m boat resolving its buoyancy on ~0.6 m panels
    // cannot feel a 1 m wave, because that wave puts as much up-force on one half of a panel as
    // down-force on the other and the integral is what the panel returns. Summing those bins
    // costs a sincos each and contributes a force of zero.
    //
    // It is also most of the cost. A typical sea retains 29/237/1451 pairs across the three
    // cascades -- cascade 2, the short chop, is 85% of the bins and none of the hull forces.
    void SampleForHull(double wx, double wz, double tSec, double minLambda,
                       OceanSample& out) const;
    // ...with a gain PER CASCADE (the water match, step 2): the bank kernel multiplies each cascade by
    // its own amplitude law -- the local sea-state scale, the swell shadow, shoaling and the
    // wave-current gain at that band's wavenumber, and the solved window's stand-down on the structure
    // bands -- so the hull's twin must too. Every channel is linear in a cascade's bins, so a gain on
    // the cascade is a gain on all eight; a cascade of gain 0 is skipped without touching a bin.
    void SampleForHull(double wx, double wz, double tSec, double minLambda,
                       const double gain[kCascades], OceanSample& out) const;

    bool Ready() const { return m_ready; }

    // A cascade band's REPRESENTATIVE wavenumber, rad/m: the geometric mean of its two cuts, the
    // number the bank kernel's per-band laws (phase speed, shoaling, the wave-current gain) are
    // evaluated at (gBandK). From the cuts this twin was configured with, never a second table.
    double BandK(int cascade) const { return std::sqrt(m_kLo[cascade] * m_kHi[cascade]); }

    // --- reporting: what the truncation actually cost, per cascade ---------------------------
    // The fraction of this cascade's realized variance thrown away by keeping only the retained
    // pairs. Realized, not expected: it is measured on the actual Gaussian draws this seed
    // produced, so it is a statement about THIS ocean and not about an ensemble.
    double ResidualVarianceFraction(int cascade) const;

    // Retained CONJUGATE PAIRS for this cascade. Each pair is two texels of the GPU's 256^2 h0
    // grid, so the equivalent GPU bin count is twice this.
    int RetainedBins(int cascade) const;

    // The time-averaged variance this evaluator actually DELIVERS for a cascade, m^2. The
    // full-grid figure is Variance(c) / (1 - ResidualVarianceFraction(c)); the significant height
    // of the delivered sea is 4*sqrt(sum over cascades). This is the CPU twin of
    // OceanFft::MeasureHs, which reads the rendered texture back and computes the same number.
    double Variance(int cascade) const;

private:
    // One retained conjugate pair. Exactly 64 bytes -- one cache line, and the query loop is
    // pure streaming over these.
    struct Bin {
        double kx = 0, kz = 0;   // wavevector, rad/m (x east, z north -- BinVariance's frame)
        double w = 0;            // sqrt(g|k|), rad/s -- the deep-water rotor rate
        double invK = 0;         // 1/|k|, the k^ the choppy term needs
        double ar = 0, ai = 0;   // h0(+k) as CsInitSpectrum writes it into .xy
        double br = 0, bi = 0;   // h0(-k) as CsInitSpectrum writes it into .zw
    };

    // The whole-cascade early-out for SampleForHull: the LONGEST wave this cascade actually
    // retained. If even that is finer than what the caller can resolve, the cascade is skipped
    // without touching a single bin.
    double m_lambdaMax[kCascades] = {};

    void SampleBand(double wx, double wz, double tSec, double minLambda, const double* gain,
                    OceanSample& out) const;

    std::vector<Bin> m_bin[kCascades];
    uint64_t m_epoch = 0;               // F21: moves with every SetSeaState; the memo's key
    double m_varAll[kCascades] = {};    // realized variance over every pair with energy
    double m_varKept[kCascades] = {};   // realized variance of the retained pairs
    double m_kLo[kCascades] = {}, m_kHi[kCascades] = {};   // the band cuts, rad/m (OceanFft's)
    double m_lambda = 1.1;
    bool m_ready = false;
};

// F21: the memo against the cold sum, bit for bit, and the plant (an instant a nanosecond off).
bool RunOceanCpuSelfTest();

}  // namespace ga
