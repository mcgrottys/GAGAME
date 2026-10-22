// ================================================================================================
//  Planet.h - THE PLANET'S OWN BODY (2026-10-20, Mark: "take surface gravity from the planet's
//  model (GM/R^2) into every wave kernel and solver, and into the wave cache's key"). Gravity was
//  a fixed number in ~50 places -- 9.81 in the wave kernels and their twins, 9.80665 in the hulls'
//  forces -- which is Earth's, and the engine draws Mars. A planet here is its mean radius and its
//  gravitational parameter GM, the two numbers its geodesy publishes; its surface gravity is
//  GM / R^2, the sphere's.
//
//  THE LIMIT, stated: GM / R^2 leaves out the planet's spin. Earth's effective gravity (the
//  centrifugal term with it, and the oblate figure) runs from 9.780 m/s^2 at the equator to 9.832
//  at the poles; the sphere's 9.820 is that range's middle. A wave's dispersion and a hull's weight
//  both see the local value; the latitude term is the next refinement, one line here.
//
//  The scene's planet is set once at assembly (SetThePlanet) and read by every solver and kernel
//  fill (ThePlanet). The kernels take it as a float through their constant buffers; the CPU twins
//  of those kernels compute with that same float (PlanetGWave), so the two processors agree to
//  the bit, as they did when both wrote 9.81.
// ================================================================================================
#pragma once

#include <atomic>
#include <string_view>

namespace ga {

struct PlanetBody {
    const char* name;
    double R;    // mean radius, m
    double GM;   // gravitational parameter, m^3 / s^2
    double SurfaceGravity() const { return GM / (R * R); }
};

// Earth: IERS / WGS 84 GM, the engine's mean radius. Mars: NASA's planetary fact sheet GM
// (42 828.37 km^3/s^2), the MOLA reference radius the relief is drawn on.
inline constexpr PlanetBody kEarthBody{"earth", 6371000.0, 3.986004418e14};
inline constexpr PlanetBody kMarsBody{"mars", 3389500.0, 4.282837e13};

inline const PlanetBody& PlanetByName(std::string_view n) { return (n == "mars") ? kMarsBody : kEarthBody; }

namespace detail {
inline std::atomic<const PlanetBody*> g_planet{&kEarthBody};
}
inline void SetThePlanet(const PlanetBody& p) { detail::g_planet.store(&p); }
inline const PlanetBody& ThePlanet() { return *detail::g_planet.load(std::memory_order_relaxed); }
// The planet's surface gravity (m/s^2): forces.
inline double PlanetG() { return ThePlanet().SurfaceGravity(); }
// ...as the kernels receive it (a float): the waves' twins compute with this one.
inline double PlanetGWave() { return double(float(PlanetG())); }

}  // namespace ga
