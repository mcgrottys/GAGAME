// ================================================================================================
//  Medium / SurfaceSample - M9bq: THE TWO INTERFACES A HULL READS THE WORLD THROUGH.
//
//  These are small structs and they are the most load-bearing decision in the vessel work,
//  because between them they make two whole features cost nothing.
//
//  ---- 1. A MEDIUM IS A MEDIUM. An element never learns whether it is in water or in air.
//
//  Forces are computed against the RELATIVE velocity v_body - v_medium, always, and the density
//  is a field of the medium rather than a constant of the element. So a windage panel and a
//  submerged drag panel are the same element kind with a different Medium, a sail is a foil in
//  air exactly as a rudder is a foil in water, and leeway, weathercocking and the RHIB's very
//  considerable windage all fall out of code that was written for the water.
//
//  There is no `if (wind)` anywhere in the vessel physics, because wind is not a special case of
//  anything. That is this repo's no-piecewise rule applied structurally rather than as a style
//  preference: a branch is a place where two descriptions of the same physics meet and are never
//  checked against each other.
//
//  ---- 2. A SURFACE IS A PLANE, NOT A HEIGHT.
//
//  SurfaceSample carries the free surface's NORMAL and the water's PARTICLE VELOCITY, not just
//  an elevation. Immersion is then measured against the local plane, and that single choice is
//  the entire answer to rolling swell: a sloped plane gives the buoyancy wrench a horizontal
//  component, which IS surfing, and particle velocity in the drag term IS being carried along by
//  the water. Long waves need no code -- they need us not to take the flat-plane shortcut, which
//  is why the shortcut is not available in this struct.
//
//  The normal comes free from the tangent bivector the caustics section already defines
//  (docs/ALGEBRA.md, `caustics`): one 2-blade whose dual is the normal.
// ================================================================================================
#pragma once

#include "core/Common.h"

namespace ga {

// ================================================================================================
//  What the water is doing at one point on the surface.
//
//  Heights are NAVD88 metres, the datum the whole height stack was normalised to (see
//  compose/HeightStackSource.h -- the ETOPO layers carry their own MSL->NAVD shift, so every
//  layer really is NAVD and this number is comparable to the bed without further thought).
// ================================================================================================
struct SurfaceSample {
    double heightNavd = 0.0;        // free surface elevation: level + tide + waves
    double dx = 0.0, dz = 0.0;      // horizontal (Gerstner) displacement of the sampled particle
    double nx = 0.0, ny = 1.0, nz = 0.0;   // unit surface normal, world frame
    double vx = 0.0, vy = 0.0, vz = 0.0;   // water velocity here: current + orbital, m/s
    double bedNavd = 0.0;           // the one bed, from the composed height stack
    double depthM = 0.0;            // heightNavd - bedNavd; <= 0 is dry
    float  foam = 0.0f;             // 0..1, for spray and wake seeding
    float  sigma2 = 0.0f;           // sub-texel slope variance -- the unresolved sea

    // THE COVERAGE GATE, and it is not a formality. Compositor::SampleHeightStack starts at
    // h = 0 and skips layers with zero weight, so a point no layer covers comes back as NAVD 0 --
    // which reads as "the seabed is exactly at the surface", which reads as aground. Offshore the
    // global ETOPO layer covers everything at weight 1, so this is true in practice; if it ever
    // is not, a boat in mid-Atlantic would silently believe it had run onto a beach. Absence is
    // not zero (core/GeoRef.h's ingest rule), so absence is carried explicitly instead.
    bool valid = false;
};

// ================================================================================================
//  A fluid, at a point. Water or air; an element cannot tell and must not care.
// ================================================================================================
struct Medium {
    double rho = 1025.0;                   // kg/m^3
    double vx = 0.0, vy = 0.0, vz = 0.0;   // fluid velocity, world frame, m/s

    // The free surface bounding this medium, as a PLANE: a world point it passes through and a
    // unit normal. The point is not optional -- a plane through (0, surfNavd, 0) is only the
    // right plane at the world origin, and this engine's world origin can be a thousand
    // kilometres away, which would tilt the sea under the boat by the whole lever arm.
    // It is stored as the point the surface was SAMPLED at, i.e. right under the hull, so
    // (p - sp) stays hull-sized and no planetary number is ever differenced.
    double sp[3] = {0.0, 0.0, 0.0};        // a world point ON the surface
    double nx = 0.0, ny = 1.0, nz = 0.0;   // the surface's unit normal
    double up = 1.0;                       // +1: this medium is BELOW the surface (water)
                                           // -1: this medium is ABOVE it (air)

    // Signed immersion of a world point in this medium, measured against the local surface PLANE
    // rather than a level. Positive = inside the medium. Using the plane is what makes a hull on
    // a wave face feel the slope; using the height alone would make every sea flat and every
    // boat a lift, never a surf. `up` is what lets water and air share the expression.
    double ImmersionAt(const double p[3]) const {
        const double d = (p[0] - sp[0]) * nx + (p[1] - sp[1]) * ny + (p[2] - sp[2]) * nz;
        return -up * d;
    }

    static Medium Seawater() {
        Medium m;
        m.rho = 1025.0;   // ~35 PSU at 10 C; fresh river water is 1000 and the estuary is between
        m.up = 1.0;
        return m;
    }
    static Medium Air() {
        Medium m;
        m.rho = 1.225;    // ISA sea level, 15 C
        m.up = -1.0;
        return m;
    }
};

// Standard gravity. One place, so a spec cannot disagree with a gate about it.
inline constexpr double kG = 9.80665;

}  // namespace ga
