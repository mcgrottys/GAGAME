// ================================================================================================
//  WaterTerms - the water's closed-form laws on the CPU: THE KERNELS' OWN TEXT, compiled here.
//
//  WHY THIS FILE EXISTS.  Every law that shapes the water the player SEES is evaluated on the GPU
//  (WaterBank.hlsl, SeaChurn.hlsl, Swe.hlsl). A hull is simulated on the CPU. If the CPU applies a
//  DIFFERENT law -- even a slightly different one -- the hull rides water the render is not drawing,
//  and no amount of tuning ever fixes it, because nothing is wrong.
//
//  Until 2026-10-25 this file was a TRANSCRIPTION of the shaders' laws into doubles, 720 lines each
//  "with the physical argument that put it there", held to the kernels by gates. Four drifts between
//  the two bodies were measured in the inventory of 2026-10-22, and a fifth found by reading: the
//  twin's breaking excess judged on the exaggerated band rms. Now the laws have ONE text,
//  shaders/laws/*.hlsli (docs/WATER_LAWS.md), written in the subset HLSL and C++ both compile, and
//  this file is the CPU HOST of that text: it supplies what a kernel gets for free -- the `real`
//  type, the intrinsics, the planet's gravity -- includes the laws into ga::law, and re-exports them
//  into ga for the callers that always lived here (TreeWater, WaveField, GaTest, Trace).
//
//  PRECISION.  `real` is double here and float in a kernel: the host owns the type, the law never
//  names one. The two processors therefore differ at the fp32 rounding level BY DESIGN; anything
//  larger is a bug in a host, not in a law. (The planet's gravity is the float the kernels receive,
//  PlanetGWave, so the one constant both read is bit-identical.)
//
//  DEPENDENCIES: <cmath> and core/Planet.h, deliberately -- included by physics code that must not
//  drag in windows.h.
// ================================================================================================
#pragma once

#include "core/Planet.h"

#include <cmath>

namespace ga {
namespace law {

// ---- THE HOST CONTRACT (shaders/laws/Laws.hlsli): the type, the intrinsics, the gravity. --------
using real = double;
using std::abs;
using std::cos;
using std::exp;
using std::log;
using std::pow;
using std::sin;
using std::sinh;
using std::sqrt;
using std::tanh;
inline real max(real a, real b) { return (a > b) ? a : b; }
inline real min(real a, real b) { return (a < b) ? a : b; }
inline real clamp(real x, real lo, real hi) { return min(max(x, lo), hi); }
inline real saturate(real x) { return clamp(x, 0.0, 1.0); }
inline real lerp(real a, real b, real t) { return a + t * (b - a); }
// HLSL smoothstep verbatim: the same 3t^2 - 2t^3 Hermite on the same saturated parameter, INCLUDING
// the descending-edge case (e0 > e1), which the blocking indicator relies on.
inline real smoothstep(real e0, real e1, real x) {
    const real t = saturate((x - e0) / (e1 - e0));
    return t * t * (3.0 - 2.0 * t);
}
#define GA_PLANET_G (::ga::PlanetGWave())
#include "laws/Laws.hlsli"
#undef GA_PLANET_G

}  // namespace law

// ---- The laws, as the callers in ga have always spelled them. -------------------------------------
using law::BandLaw;
using law::BandPhaseSpeed;
using law::BandSea;
using law::BreakFactor;
using law::BreakHmax;
using law::CrestExcessShare;
using law::CrestLaw;
using law::CrestOut;
using law::CrestState;
using law::DryWeight;
using law::LipFall;
using law::LipFallDe;
using law::LipReach;
using law::OrbitCoth;
using law::OrbitCothK;
using law::PlungeShare;
using law::SeaBreak;
using law::SeaBreakAt;
using law::ShoalFactor;
using law::ShoaledKh;
using law::SpillReach;
using law::SwashFactor;
using law::SwashShare;
using law::TroughBound;
using law::WakeBranch;
using law::WakeOne;
using law::WakeSample;
using law::WakeVessel;
using law::WaveCurrentAmp;
using law::WaveCurrentGain;
using law::WindowEdgeM;
using law::WindowWeight;
using law::YieldShare;
using law::BesselJ0;
using law::BesselJ1;
using law::BesselJ2;
using law::ImpactCavityDiameter;
using law::ImpactCavityDepth;
using law::ImpactSpectrum;
using law::ImpactOmega;
using law::ImpactTerm;
using law::ImpactTermSlope;
using law::ImpactShoal;
using law::ImpactShoalSlope;
using law::kGammaHs;
using law::kJetFallRatio;

// The last-resort depth floor the laws divide above (laws/Dispersion.hlsli kDepthFloorM), for the
// callers that spell it in doubles.
inline constexpr double kDepthFloorM = 0.15;

// The swell shadow's two kernel constants, shaders/WaterBank.hlsl (M7j/M9ba): the exposure page is
// read at max(have, 3) -- mips >= 3 of the z14 page, ~76 m texels, the grain the line-of-sight march
// is honest at -- and floored at 0.18, the local chop a wind raises even in the deepest lee (the
// march's own deep-shadow floor is 0.12; the bank keeps a little more for the sea it makes itself).
// Carried with the kernel's arguments, not re-derived.
inline constexpr unsigned kSwellShadowMipFloor = 3;
inline constexpr double kSwellShadowFloor = 0.18;

}  // namespace ga
