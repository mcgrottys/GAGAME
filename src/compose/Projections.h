// ================================================================================================
//  Projections - M6l: exact map projections for the schema registry's alignment contract.
//
//  Every source declares its native CRS and resolves WGS84 lat/lon EXACTLY into it -- no
//  linear approximations ("painting will need to account for meters per pixel and projections
//  of the different earth texture layers so that they align properly" -- the user's spec,
//  verbatim). Snyder's formulas, GRS80 ellipsoid; forward transforms only (sources answer
//  point queries, the compositor never needs the inverse). Pinned by RunComposeSelfTest
//  against independently computed reference coordinates at the ACT0816 anchor.
//
//  NAD83 vs WGS84: treated as identical here (~1 m horizontal in New England). That offset is
//  REAL and visible at 15 cm -- it is part of what the stencil comparison measures, not
//  something to hide in the math. Documented in the registry's crs strings.
// ================================================================================================
#pragma once

#include <cmath>

namespace ga {

// Transverse Mercator (UTM zone template): GRS80, k0 = 0.9996.
struct TransverseMercator {
    double lon0Rad;                 // central meridian
    double falseE = 500000.0;
    double falseN = 0.0;

    static TransverseMercator Utm(int zone) {
        return {(-183.0 + 6.0 * zone) * 3.14159265358979 / 180.0, 500000.0, 0.0};
    }

    void Forward(double latRad, double lonRad, double& e, double& n) const {
        constexpr double a = 6378137.0, f = 1.0 / 298.257222101, k0 = 0.9996;
        constexpr double e2 = f * (2.0 - f);
        const double ep2 = e2 / (1.0 - e2);
        const double sinL = std::sin(latRad), cosL = std::cos(latRad);
        const double N = a / std::sqrt(1.0 - e2 * sinL * sinL);
        const double T = std::tan(latRad) * std::tan(latRad);
        const double C = ep2 * cosL * cosL;
        const double A = cosL * (lonRad - lon0Rad);
        const double M =
            a * ((1 - e2 / 4 - 3 * e2 * e2 / 64 - 5 * e2 * e2 * e2 / 256) * latRad -
                 (3 * e2 / 8 + 3 * e2 * e2 / 32 + 45 * e2 * e2 * e2 / 1024) *
                     std::sin(2 * latRad) +
                 (15 * e2 * e2 / 256 + 45 * e2 * e2 * e2 / 1024) * std::sin(4 * latRad) -
                 (35 * e2 * e2 * e2 / 3072) * std::sin(6 * latRad));
        const double A2 = A * A, A3 = A2 * A, A4 = A2 * A2, A5 = A4 * A, A6 = A4 * A2;
        e = falseE + k0 * N *
                         (A + (1 - T + C) * A3 / 6.0 +
                          (5 - 18 * T + T * T + 72 * C - 58 * ep2) * A5 / 120.0);
        n = falseN + k0 * (M + N * std::tan(latRad) *
                                   (A2 / 2.0 + (5 - T + 9 * C + 4 * C * C) * A4 / 24.0 +
                                    (61 - 58 * T + T * T + 600 * C - 330 * ep2) * A6 / 720.0));
    }
};

// Lambert Conformal Conic (2SP): Massachusetts State Plane Mainland (EPSG:26986) and kin.
struct LambertConformalConic {
    double sp1Rad, sp2Rad, lat0Rad, lon0Rad, falseE, falseN;
    // Derived once:
    double n = 0, aF = 0, rho0 = 0;

    static LambertConformalConic MassMainland() {
        LambertConformalConic p{41.71666666666667 * 3.14159265358979 / 180.0,
                                42.68333333333333 * 3.14159265358979 / 180.0,
                                41.0 * 3.14159265358979 / 180.0,
                                -71.5 * 3.14159265358979 / 180.0, 200000.0, 750000.0};
        p.Derive();
        return p;
    }

    void Derive() {
        constexpr double a = 6378137.0, f = 1.0 / 298.257222101;
        const double e = std::sqrt(f * (2.0 - f));
        auto m = [&](double p) {
            return std::cos(p) / std::sqrt(1.0 - e * e * std::sin(p) * std::sin(p));
        };
        auto t = [&](double p) {
            return std::tan(3.14159265358979 / 4.0 - p / 2.0) /
                   std::pow((1.0 - e * std::sin(p)) / (1.0 + e * std::sin(p)), e / 2.0);
        };
        n = (std::log(m(sp1Rad)) - std::log(m(sp2Rad))) / (std::log(t(sp1Rad)) -
                                                           std::log(t(sp2Rad)));
        aF = a * m(sp1Rad) / (n * std::pow(t(sp1Rad), n));
        rho0 = aF * std::pow(t(lat0Rad), n);
    }

    void Forward(double latRad, double lonRad, double& x, double& y) const {
        constexpr double f = 1.0 / 298.257222101;
        const double e = std::sqrt(f * (2.0 - f));
        const double tt = std::tan(3.14159265358979 / 4.0 - latRad / 2.0) /
                          std::pow((1.0 - e * std::sin(latRad)) / (1.0 + e * std::sin(latRad)),
                                   e / 2.0);
        const double rho = aF * std::pow(tt, n);
        const double th = n * (lonRad - lon0Rad);
        x = falseE + rho * std::sin(th);
        y = falseN + rho0 - rho * std::cos(th);
    }
};

}  // namespace ga
