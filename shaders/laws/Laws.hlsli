// ================================================================================================
//  shaders/laws -- THE WATER'S CLOSED-FORM LAWS, ONE TEXT, TWO HOSTS (2026-10-25, docs/WATER_LAWS.md).
//
//  Every law here is compiled by the kernels (WaterBank.hlsl, SeaChurn.hlsl, Swe.hlsl) AND by the CPU
//  (src/sim/WaterTerms.h includes these files into ga::law). There is no twin to transcribe and no
//  gate to hold the two bodies together: a law has one body. Before this the same laws lived in
//  shaders/Jet.hlsli and inline in WaterBank.hlsl, with a C++ transcription in WaterTerms.h; four
//  drifts between the two were measured, a fifth was found by reading (the twin's breaking excess on
//  the exaggerated band rms).
//
//  THE DIALECT: the subset HLSL 2021 (dxc) and C++20 (MSVC) both accept, held to both compilers:
//    - scalar `real` arithmetic. The HOST owns the type: `typedef float real;` in a kernel,
//      `using real = double;` on the CPU. A law never names float or double.
//    - `inline` free functions; structs returned by value; `static const real` constants.
//    - the intrinsics max min clamp saturate lerp smoothstep sqrt tanh sinh exp log pow cos sin abs.
//      A kernel has them; the CPU host defines them on real.
//    - GA_PLANET_G, the planet's surface gravity, from the host (a kernel's constant buffer;
//      PlanetGWave() on the CPU).
//    - no vector types, no inout, no [attributes], no texture reads: a law takes components and
//      returns a struct. The kernel packs and unpacks at the call.
//
//  THE HOST CONTRACT, before including any of these files:
//      typedef float real;            // or: using real = double;
//      #define GA_PLANET_G (...)
//      (CPU: max/min/clamp/saturate/lerp/smoothstep on real, and <cmath>'s functions in scope)
//
//  The files, in dependency order:
//      Dispersion.hlsli   phase speed, Guo's k in depth, the orbit's coth, Green's shoaling, the
//                         wave-current gain
//      Breaking.hlsli     Battjes-Janssen's Hmax, the clipped-Rayleigh factor, a sea's excess, the
//                         dry weight, the trough's bound
//      Bands.hlsli        the three cascades' band law, the cascades' yield to the solved field,
//                         the solved window's weight
//      Crest.hlsli        the lip's fall and flight, the landing, the spiller's roller, the swash,
//                         which crests plunge, THE CREST LAW
//      Wake.hlsli         the Kelvin wake, signed stationary phase
//      Impact.hlsli       an asteroid's cavity and its linear dispersive wave (the Bessel sum)
// ================================================================================================
#ifndef GA_LAWS_HLSLI
#define GA_LAWS_HLSLI
#ifndef GA_PLANET_G
#error "laws: define GA_PLANET_G (the planet's surface gravity) before including"
#endif
#include "Dispersion.hlsli"
#include "Breaking.hlsli"
#include "Bands.hlsli"
#include "Crest.hlsli"
#include "Wake.hlsli"
#include "Impact.hlsli"
#endif
