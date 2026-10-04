// ================================================================================================
//  Air.h - A PLANET'S AIR, AS ROWS. The sky is one integral (shaders/Atmosphere.hlsli AtmRay)
//  and a planet enters it only through these numbers: what its air scatters and absorbs per
//  metre at the ground, how fast each part thins with height, where it ends, what its ground
//  sends back. Earth and Mars are two rows of the same table -- there is no Mars branch in any
//  shader. The rows travel in the scene constants (SceneConstants::air, Common.hlsli gAir*) and
//  in the sky table's own constants (SkyLut.hlsl), in this one order:
//
//    row 0  Rayleigh scattering at 680/550/440 nm (1/m at the ground), its scale height (m)
//    row 1  aerosol scattering per channel (1/m), its scale height (m)
//    row 2  aerosol extinction per channel (1/m), Henyey-Greenstein g
//    row 3  ozone absorption per channel (1/m at the tent's peak), the tent's centre height (m)
//    row 4  the tent's half-width (m), the top of the air (m above the ground), the ground's
//           albedo under the multiple-scattering estimate, spare (0)
//
//  There is no gain. The air's coefficients are physical, the integral answers per unit of the
//  sun's irradiance, and the ONE sun (Common.hlsli SUN_IRR_C: the irradiance at the top of the
//  air, in the engine's unit) multiplies sky and ground alike -- so the sky's radiance and the lit
//  ground come out in one unit by construction (Common.hlsli kSunE).
// ================================================================================================
#pragma once

#include <cmath>
#include <string>

namespace ga {

struct AirRows {
    float row[5][4];
};

// The aerosol of a day: its optical depth at 550 nm `aod` and its spectral slope, the Angstrom
// exponent `alpha` -- the column at each channel's wavelength is aod (lambda / 550 nm)^-alpha. The
// profile's shape (scale height), the single-scattering albedo per channel and the phase's g stay
// the planet's row. The channels are the rows' own wavelengths: 680, 550, 440 nm.
inline void AirAerosol(AirRows& a, float aod, float alpha) {
    const float h = a.row[1][3];
    if (aod < 0.0f || h <= 0.0f) return;
    const float lam[3] = {680.0f, 550.0f, 440.0f};
    for (int c = 0; c < 3; ++c) {
        const float ssa = (a.row[2][c] > 0.0f) ? a.row[1][c] / a.row[2][c] : 1.0f;
        const float ext = aod * std::pow(lam[c] / 550.0f, -alpha) / h;   // 1/m at the ground
        a.row[2][c] = ext;          // extinction
        a.row[1][c] = ext * ssa;    // scattering
    }
}

// EARTH: Bruneton & Neyret 2008 / Bruneton 2017's reference air -- Rayleigh from the air's
// refractive index and the US Standard Atmosphere 1976, the ozone layer as a tent 15 km either
// side of 25 km, and the reference's aerosol SHAPE (1.2 km scale height, single-scattering albedo
// 0.90, Henyey-Greenstein g 0.8).
// THE AEROSOL'S COLUMN IS DATA (scene key air.aod550; AirOf below). The reference's own column,
// 4.44e-6 /m x 1.2 km = 0.0053, is a very clean marine air. The DEFAULT is a typical clear-sky
// marine day: an aerosol optical depth of 0.10 at 550 nm -- the Maritime Aerosol Network's
// open-ocean mean is 0.11 at 500 nm (Smirnov et al. 2011, Atmos. Meas. Tech. 4, 583-597), which
// the Angstrom exponent of maritime air (~0.6) carries to ~0.10 at 550 nm.
// THE SLOPE IS DATA TOO (scene key air.angstrom): the reference's aerosol is spectrally grey
// (alpha 0); the default is the maritime ~0.6, the open-ocean mean Angstrom exponent (440-870 nm)
// of the same Maritime Aerosol Network climatology (Smirnov et al. 2011) -- 1.14x the 550 nm
// column at 440 nm, 0.88x at 680 nm.
static constexpr float kEarthAod550 = 0.10f;
static constexpr float kEarthAngstrom = 0.6f;
inline AirRows EarthAir(float aod550 = kEarthAod550, float angstrom = kEarthAngstrom) {
    AirRows a{{{5.802e-6f, 13.558e-6f, 33.100e-6f, 8000.0f},
               {3.996e-6f, 3.996e-6f, 3.996e-6f, 1200.0f},
               {4.440e-6f, 4.440e-6f, 4.440e-6f, 0.80f},
               {0.650e-6f, 1.881e-6f, 0.085e-6f, 25000.0f},
               {15000.0f, 100000.0f, 0.1f, 0.0f}}};
    AirAerosol(a, aod550, angstrom);
    return a;
}

// MARS: the same integral with the planet's own air.
//   CO2 Rayleigh: the column is 610 Pa / 3.71 m s^-2 = 164 kg m^-2 against Earth's 10 330 (1.6 %),
//   and CO2 scatters 2.4x what air does per molecule (its refractive index), so the optical depth
//   is 3.8 % of Earth's -- 0.0037 at 550 nm -- spread over an 11.1 km scale height: Earth's
//   coefficients x 0.038 x 8 / 11.1.
//   Dust: a visible optical depth of 0.5 over an 11.1 km scale height (Lemmon et al. 2004, the
//   rovers' clear-season sky: 0.3-1.0), extinction 4.5e-5 /m at the ground; single-scattering
//   albedo 0.976 / 0.92 / 0.76 at 680 / 550 / 440 nm and g = 0.63 (Wolff et al. 2009, CRISM) --
//   the dust that eats blue is why the Martian day is butterscotch and its sunset blue.
//   No ozone to speak of. The ground's albedo 0.25 (the bright dusty regions). (The sun is the
//   engine's one SUN_IRR_C on both planets: Mars's 43 % is not modelled, sky and ground alike.)
// Mars's dust is spectrally flat in extinction across the visible (its colour is its single-
// scattering albedo, above): alpha 0.
static constexpr float kMarsAod550 = 0.5f;
static constexpr float kMarsAngstrom = 0.0f;
inline AirRows MarsAir(float aod550 = kMarsAod550, float angstrom = kMarsAngstrom) {
    const float k = 0.038f * 8000.0f / 11100.0f;
    const float e = 0.5f / 11100.0f;
    AirRows a{{{5.802e-6f * k, 13.558e-6f * k, 33.100e-6f * k, 11100.0f},
               {e * 0.976f, e * 0.92f, e * 0.76f, 11100.0f},
               {e, e, e, 0.63f},
               {0.0f, 0.0f, 0.0f, 25000.0f},
               {15000.0f, 100000.0f, 0.25f, 0.0f}}};
    AirAerosol(a, aod550, angstrom);
    return a;
}

// The air of the scene's planet (Scene::scene.planet, the names the radius is chosen by) on the
// scene's day: `aod550` and `angstrom` are the scene's air.aod550 and air.angstrom; below 0 each is
// the planet's typical value above.
inline AirRows AirOf(const std::string& planet, float aod550 = -1.0f, float angstrom = -1.0f) {
    if (planet == "mars") {
        return MarsAir(aod550 < 0.0f ? kMarsAod550 : aod550,
                       angstrom < 0.0f ? kMarsAngstrom : angstrom);
    }
    return EarthAir(aod550 < 0.0f ? kEarthAod550 : aod550,
                    angstrom < 0.0f ? kEarthAngstrom : angstrom);
}

}  // namespace ga
