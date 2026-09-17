// ================================================================================================
//  Ephemeris.h - M9bi: THE SUN, as a place instead of a preference.
//
//  Before this file the sun was two floats on the Renderer -- sunAzimuthDeg = 112, sunElevation
//  Deg = 26 -- with no date, no latitude and no distance. Every layer read the same unit vector,
//  so the scene was at least self-consistent, but it was self-consistently WRONG: the tide came
//  from EOT20 at the scene's real timestamp, the waves came from GFS-Wave at that timestamp, and
//  the light came from an art direction chosen once.
//
//  Now there is ONE global constant, and it is not a direction: it is the sun's PLACE.
//
//  ---- THE FRAME CHAIN, as versors (docs/ALGEBRA.md "cga") -------------------------------------
//  solar.hci   the heliocentric space. The SUN IS THE ORIGIN -- literally cga::N0(), a constant
//              this file never recomputes. Unit length 1 AU, axes equatorial mean-of-date
//              (x to the equinox, z to the celestial pole). Earth is a translator away.
//  planet.ecef the engine's planet frame (Compose.hlsli: +y = north pole, +x = lat 0 lon 0, and
//              lon = atan2(z, x)). Unit length R_earth.
//
//  and the map between them is a product of four versors, applied by the ONE sandwich:
//
//      T  translator   put the Earth's centre at the origin       (geocentric)
//      R  rotor        -GMST about the pole                       (inertial -> Earth-fixed)
//      M  reflection   swap the last two axes                     (ECEF -> the engine's frame)
//      D  dilator      1 AU -> 1 R_earth of unit length           (the SPACE change)
//
//  M is a REFLECTION and not a rotation, and that is not a convenience: the engine's planet
//  frame is LEFT-handed (at lat 0 lon 0, east = +z, north = +y, up = +x, so east x north = -up),
//  while ECEF is right-handed. A rotor cannot express that and would silently mirror the sky.
//  CGA does not care -- an odd versor is still a versor -- but the ledger has to say so.
//
//  D is the piece that only a conformal algebra offers, and it is not decoration. The engine
//  spans eleven orders of magnitude, from 1.5e11 m to the sub-metre estuary, and a conformal
//  point written in METRES stops being a point past 9.49e7 m -- its P.ni collapses to exactly
//  zero and it becomes a point at INFINITY (Cga.h, gated). The sun in metres is a direction.
//  So each space carries its own unit length, and moving between them is a versor.
//
//  ---- WHAT IS AND IS NOT MODELLED -------------------------------------------------------------
//  The apparent solar coordinates are the Astronomical Almanac's low-precision series (accuracy
//  ~0.01 deg = 36", stated valid 1950-2050; this engine's window is 2026). That is 20x finer
//  than the sun's own angular RADIUS, so it cannot be seen. NREL's SPA would give 0.0003 deg and
//  a thousand lines of periodic terms to no visible end. The series' mean longitude already
//  carries aberration, so what comes out is the APPARENT sun, not the geometric one.
//
//  UTC is used as UT1 (|dUT1| < 0.9 s = 0.004 deg of Earth rotation), no delta-T, no nutation, no
//  refraction, no topocentric parallax beyond what the finite distance gives on its own.
//
//  Gate: RunCgaSelfTest pins the versor chain against the independent trig closed form, and pins
//  SOLAR NOON at the Merrimack mouth on 2026-08-28 against an external ephemeris service
//  (16:44:25Z) -- an outside number, not this file marking its own homework.
// ================================================================================================
#pragma once

#include "core/Cga.h"

#include <algorithm>
#include <cmath>

namespace ga::sun {

inline constexpr double kAuM = 1.495978707e11;       // IAU 2012 astronomical unit, metres
inline constexpr double kSunRadiusM = 6.957e8;       // IAU 2015 nominal solar radius, metres
inline constexpr double kEarthRadiusM = 6371000.0;   // the engine's sphere (GlobeLayer gGlo.x)
inline constexpr double kTsi1AuWm2 = 1361.0;         // total solar irradiance at exactly 1 AU
inline constexpr double kJ2000Jd = 2451545.0;
inline constexpr double kDeg = 3.14159265358979323846 / 180.0;

inline double Wrap360(double d) { return d - 360.0 * std::floor(d / 360.0); }
inline double Wrap180(double d) { return Wrap360(d + 180.0) - 180.0; }

// The unix epoch is 1970-01-01T00:00:00Z = JD 2440587.5. Time is a double everywhere in this
// engine (TideModel, SweSolver, the rails), so no precision is lost getting here.
inline double JulianDay(double unixSeconds) { return unixSeconds / 86400.0 + 2440587.5; }

// Greenwich Mean Sidereal Time, degrees. The rate 360.98564736629 deg/day is a SIDEREAL day --
// the 3m56s/day by which the stars beat the sun is exactly what puts the subsolar point where
// it belongs at a given UTC.
inline double GmstDeg(double n) {
    const double T = n / 36525.0;
    return Wrap360(280.46061837 + 360.98564736629 * n + 0.000387933 * T * T -
                   T * T * T / 38710000.0);
}

struct Apparent {
    double n = 0;            // days since J2000.0 (TT taken as UTC)
    double eclLonDeg = 0;    // apparent ecliptic longitude (aberration already folded in)
    double raDeg = 0;        // apparent right ascension
    double decDeg = 0;       // apparent declination
    double distAu = 0;       // Earth-Sun distance
    double gmstDeg = 0;
    double subsolarLatDeg = 0;   // = declination
    double subsolarLonDeg = 0;   // = RA - GMST, the place the sun is overhead
    double angRadiusDeg = 0;     // the sun's own angular radius from here (0.262 .. 0.271)
    double irradianceWm2 = 0;    // TSI / distAu^2 -- the annual 3.4% the fixed sun never had
};

// The Astronomical Almanac's low-precision solar coordinates. Every coefficient below is from
// that source; change them there first, never here (the engine's rule for measured constants).
inline Apparent Solar(double unixSeconds) {
    Apparent a;
    a.n = JulianDay(unixSeconds) - kJ2000Jd;
    const double L = Wrap360(280.460 + 0.9856474 * a.n);          // mean longitude
    const double g = Wrap360(357.528 + 0.9856003 * a.n) * kDeg;   // mean anomaly
    a.eclLonDeg = Wrap360(L + 1.915 * std::sin(g) + 0.020 * std::sin(2.0 * g));
    const double eps = (23.439 - 0.0000004 * a.n) * kDeg;
    const double lam = a.eclLonDeg * kDeg;
    a.distAu = 1.00014 - 0.01671 * std::cos(g) - 0.00014 * std::cos(2.0 * g);
    a.raDeg = Wrap360(std::atan2(std::cos(eps) * std::sin(lam), std::cos(lam)) / kDeg);
    a.decDeg = std::asin(std::sin(eps) * std::sin(lam)) / kDeg;
    a.gmstDeg = GmstDeg(a.n);
    a.subsolarLatDeg = a.decDeg;
    a.subsolarLonDeg = Wrap180(a.raDeg - a.gmstDeg);
    a.angRadiusDeg = std::asin(kSunRadiusM / (a.distAu * kAuM)) / kDeg;
    a.irradianceWm2 = kTsi1AuWm2 / (a.distAu * a.distAu);
    return a;
}

// The engine's planet frame, spelled out once: this is the inverse of Compose.hlsli's
// lat = asin(dir.y), lon = atan2(dir.z, dir.x). Nothing else in this file may invent its own.
inline void PlanetDirFromLatLon(double latDeg, double lonDeg, double& x, double& y, double& z) {
    const double la = latDeg * kDeg, lo = lonDeg * kDeg;
    x = std::cos(la) * std::cos(lo);
    y = std::sin(la);
    z = std::cos(la) * std::sin(lo);
}

// ---- THE SOLAR SYSTEM, as conformal objects ---------------------------------------------------
// One struct, built once per frame, holding the sun as a PLACE plus the versor that carries it
// into the frame the renderer draws in. Nothing downstream re-derives any of it.
struct SolarSystem {
    // solar.hci, unit length 1 AU.
    cga::Mv sunPoint;      // == cga::N0(). THE global constant: the sun is the origin.
    cga::Mv sunSphere;     // the photosphere as a dual sphere -- what gives an angular radius
    cga::Mv earthPoint;    // T n0 ~T: the Earth's centre, one translator from the sun
    cga::Mv earthSphere;

    // The versor chain solar.hci -> planet.ecef, and the sun after it (unit length R_earth).
    cga::Mv toPlanetEven;      // T then R then D -- the even (rotor/translator/dilator) part
    cga::Mv axisMirror;        // M: the odd half, the left-handed frame's reflection
    cga::Mv sunPointPlanet;    // the sun, in the engine's planet frame, in Earth radii
    cga::Mv earthSpherePlanet;   // the unit sphere the engine actually draws
    // How far the Earth's centre MISSES the origin after the four-versor round trip, in metres.
    // In planet.ecef the Earth's centre IS the origin by definition of the frame, so the sphere
    // above is built from that definition and this number is the chain's measured error rather
    // than something the geometry has to absorb.
    double chainOriginResidM = 0;

    Apparent app;

    // Conveniences the renderer wants, all read OUT of the objects above rather than beside them.
    double sunDirPlanet[3] = {0, 0, 0};   // unit, planet frame, from the Earth's CENTRE
    double sunDistM = 0;
};

// Apply an odd (reflection) versor to a grade-1 IPNS object: X -> -n X n. Points, spheres and
// planes are all grade 1, and those are the only things this file pushes through the mirror.
inline cga::Mv Mirror(const cga::Mv& n, const cga::Mv& X) { return cga::Gp(cga::Gp(n, X), n) * -1.0; }

// THE CHAIN, from the Earth's place around the sun: its centre (AU, the sun at the origin) and
// the rotor that turns the heliocentric axes into the Earth's own. Everything below is the same
// versors whoever supplies those two -- the clock (Build) or a scene (BuildEarth).
inline SolarSystem BuildChain(const Apparent& app, double ex, double ey, double ez,
                              const cga::Mv& spinRotor);

inline SolarSystem Build(double unixSeconds) {
    const Apparent app = Solar(unixSeconds);
    const double ra = app.raDeg * kDeg, dec = app.decDeg * kDeg;
    // The sun seen FROM the Earth lies at (RA, dec) and distance R, so the Earth seen from the
    // SUN lies at exactly minus that.
    return BuildChain(app, -app.distAu * std::cos(dec) * std::cos(ra),
                      -app.distAu * std::cos(dec) * std::sin(ra), -app.distAu * std::sin(dec),
                      cga::Rotor(0.0, 0.0, 1.0, -app.gmstDeg * kDeg));   // about the pole (z_eq)
}

// THE EARTH, PLACED BY A SCENE. The sun is a light at 0,0,0; the Earth's centre is `atM` metres
// from it and the Earth is turned `spinDeg` about `axis`. No clock anywhere: the apparent
// quantities (distance, disc size, subsolar point) are read back OUT of the placement.
inline SolarSystem BuildEarth(const double atM[3], const double axis[3], double spinDeg) {
    Apparent app;
    const double ex = atM[0] / kAuM, ey = atM[1] / kAuM, ez = atM[2] / kAuM;
    app.distAu = std::sqrt(ex * ex + ey * ey + ez * ez);
    const double d = (app.distAu > 0.0) ? app.distAu : 1.0;
    // The sun from the Earth is the direction -at; in the frame's own terms that is an RA and a
    // declination, kept only for the log and the disc.
    app.raDeg = Wrap360(std::atan2(-ey, -ex) / kDeg);
    app.decDeg = std::asin(std::clamp(-ez / d, -1.0, 1.0)) / kDeg;
    app.gmstDeg = Wrap360(spinDeg);
    app.subsolarLatDeg = app.decDeg;
    app.subsolarLonDeg = Wrap180(app.raDeg - app.gmstDeg);
    app.angRadiusDeg = std::asin(std::min(1.0, kSunRadiusM / (d * kAuM))) / kDeg;
    app.irradianceWm2 = kTsi1AuWm2 / (d * d);
    double ax = axis[0], ay = axis[1], az = axis[2];
    const double al = std::sqrt(ax * ax + ay * ay + az * az);
    if (al > 0.0) { ax /= al; ay /= al; az /= al; } else { ax = 0.0; ay = 0.0; az = 1.0; }
    return BuildChain(app, ex, ey, ez, cga::Rotor(ax, ay, az, -spinDeg * kDeg));
}

inline SolarSystem BuildChain(const Apparent& app, double ex, double ey, double ez,
                              const cga::Mv& spinRotor) {
    SolarSystem s;
    s.app = app;

    // ---- solar.hci. The sun is the origin, and it is the origin for every consumer, forever.
    s.sunPoint = cga::N0();
    s.sunSphere = cga::DualSphere(s.sunPoint, kSunRadiusM / kAuM);

    // The Earth's centre, one translator from the sun.
    const cga::Mv T = cga::Translator(ex, ey, ez);
    s.earthPoint = cga::Sandwich(T, s.sunPoint);
    s.earthSphere = cga::DualSphere(s.earthPoint, kEarthRadiusM / kAuM);

    // ---- the chain into the frame the renderer draws in.
    // T^-1 puts the EARTH at the origin (geocentric); R turns the inertial equinox to Greenwich;
    // D restates the space in Earth radii. M (below) mirrors the last two axes.
    const cga::Mv Tinv = cga::Translator(-ex, -ey, -ez);
    const cga::Mv& R = spinRotor;
    const cga::Mv D = cga::Dilator(kAuM / kEarthRadiusM);
    s.toPlanetEven = cga::Gp(D, cga::Gp(R, Tinv));
    // The mirror that swaps Y and Z: reflection in the plane whose unit normal is (0,1,-1)/sqrt2.
    const double r2 = 1.0 / std::sqrt(2.0);
    s.axisMirror = cga::Dir(0.0, r2, -r2);

    // NORMALISE after the chain: the dilator is a versor that deliberately does NOT preserve
    // a conformal point's scale, and DualSphere below reads a radius out of the vector, so an
    // un-normalised centre silently produces a sphere of the wrong size (cga::Normalize).
    // Normalise (the dilator does not preserve P.ni) and RE-PROJECT onto the null cone (the
    // dilator multiplies the ni coefficient's error by scale^2). Two lines of conformal
    // hygiene, both for the same reason a long rotor product gets re-orthonormalised.
    s.sunPointPlanet = cga::Reproject(
        cga::Normalize(Mirror(s.axisMirror, cga::Sandwich(s.toPlanetEven, s.sunPoint))));
    // The Earth's sphere comes from the DEFINITION of planet.ecef -- unit sphere at the origin
    // -- not from transporting its centre 1 AU and back. Asking a float chain to reproduce a
    // definition is how the 0.2 m below would have become the sphere's radius error instead.
    s.earthSpherePlanet = cga::DualSphere(cga::Up(0.0, 0.0, 0.0), 1.0);
    {
        const cga::Mv ec = cga::Reproject(
            cga::Normalize(Mirror(s.axisMirror, cga::Sandwich(s.toPlanetEven, s.earthPoint))));
        s.chainOriginResidM = cga::Distance(ec, cga::Up(0.0, 0.0, 0.0)) * kEarthRadiusM;
    }

    // Read the direction back OUT of the conformal point -- the down map, normalised.
    double px, py, pz;
    cga::Down(s.sunPointPlanet, px, py, pz);
    const double len = std::sqrt(px * px + py * py + pz * pz);
    const double inv = (len > 0.0) ? 1.0 / len : 0.0;
    s.sunDirPlanet[0] = px * inv;
    s.sunDirPlanet[1] = py * inv;
    s.sunDirPlanet[2] = pz * inv;
    s.sunDistM = s.app.distAu * kAuM;
    return s;
}

// The sun's direction seen from an actual PLACE on the planet rather than from its centre: the
// finite-source parallax the old direction-only sun could not express. It is 8.8" at most (the
// Earth's radius over 1 AU), well under a pixel at any altitude this engine flies, so the
// renderer takes it once at the camera and never per pixel -- but it is the difference between
// a sun that is somewhere and a sun that is merely some way.
inline void SunDirFromPlanetPoint(const SolarSystem& s, const double posEarthRadii[3],
                                  double outDir[3]) {
    const cga::Mv P = cga::Up(posEarthRadii[0], posEarthRadii[1], posEarthRadii[2]);
    double sx, sy, sz, qx, qy, qz;
    cga::Down(s.sunPointPlanet, sx, sy, sz);
    cga::Down(P, qx, qy, qz);
    const double dx = sx - qx, dy = sy - qy, dz = sz - qz;
    const double len = std::sqrt(dx * dx + dy * dy + dz * dz);
    const double inv = (len > 0.0) ? 1.0 / len : 0.0;
    outDir[0] = dx * inv;
    outDir[1] = dy * inv;
    outDir[2] = dz * inv;
}

// ---- THE TERMINATOR, as a meet ----------------------------------------------------------------
// The day/night boundary on a sphere lit by a POINT at finite distance is the tangent cone's
// contact circle: not a great circle, but the one in the polar plane of the light -- offset
// R^2/d toward it, radius R sqrt(1 - R^2/d^2). In CGA the circle is the OUTER PRODUCT of the
// sphere with that plane, and "this point is on the terminator" is one left contraction against
// the result, whatever grade it happens to be.
//
// At 1 AU the offset is R^2/d = 0.27 km and the radius is short of R by 0.14 m, so this changes
// no pixel. It is here because the SOFTNESS does change pixels: the sun is a disc of angular
// radius 0.266 deg, so the geometric penumbra is a 59 km band -- and because a model that cannot
// say where the terminator is has not really placed the sun, it has only pointed at it.
struct Terminator {
    cga::Mv plane;      // the polar plane of the light w.r.t. the sphere (IPNS, grade 1)
    cga::Mv circle;     // sphere ^ plane: the contact circle (IPNS, grade 2)
    double offset = 0;  // centre offset from the sphere's centre toward the light, unit lengths
    double radius = 0;  // in unit lengths
};

inline Terminator TerminatorOf(const cga::Mv& sphere, const cga::Mv& lightPoint,
                               double sphereRadius) {
    Terminator t;
    // The sphere's centre is its dual vector plus the radius term put back.
    const cga::Mv centre = sphere + cga::Ni() * (0.5 * sphereRadius * sphereRadius);
    double cx, cy, cz, lx, ly, lz;
    cga::Down(centre, cx, cy, cz);
    cga::Down(lightPoint, lx, ly, lz);
    const double dx = lx - cx, dy = ly - cy, dz = lz - cz;
    const double d = std::sqrt(dx * dx + dy * dy + dz * dz);
    const double inv = (d > 0.0) ? 1.0 / d : 0.0;
    const double nx = dx * inv, ny = dy * inv, nz = dz * inv;
    t.offset = sphereRadius * sphereRadius / (d > 0.0 ? d : 1.0);
    const double s2 = 1.0 - (sphereRadius * sphereRadius) / (d > 0.0 ? d * d : 1.0);
    t.radius = sphereRadius * std::sqrt(s2 > 0.0 ? s2 : 0.0);
    // The plane through the contact circle: normal toward the light, through centre + offset*n.
    // IPNS plane n + d ni: P . pi = x.n - d, so d is the signed distance along n. The plane
    // passes through centre + offset*n.
    const double dist = nx * cx + ny * cy + nz * cz + t.offset;
    t.plane = cga::DualPlane(nx, ny, nz, dist);
    t.circle = cga::Op(sphere, t.plane);
    return t;
}

}  // namespace ga::sun
