// ================================================================================================
//  BuildingMoments - A SOLID'S MASS AS A CONFORMAL VECTOR, ITS FAR SHAPE AS A MOMENT BOX, ITS SIZE
//  AS A LEVEL. The algebra under the building LOD (docs/BUILDING_LOD.md); step 1 of its order.
//
//  A solid of unit density has moments about an origin: m = int dV, s = int x dV, S = int x x^T dV
//  (1 + 3 + 6 numbers: Garland-Heckbert's quadric count). The conformal embedding is linear in
//  (1, x, x^2), so its integral is one grade-1 vector of Cl(4,1) (core/Cga.h):
//
//      M = int X dV = m n0 + s + (1/2) tr(S) ni
//
//  and three facts hold, pinned by RunLodSelfTest:
//    FOLD      the union's M is the sum of the parts' (and so is the full S): a parent is its
//              children added -- the fold law's linear case, nothing thresholded before the sum.
//    FRAME     moving the origin is the versor sandwich T M ~T (the parallel-axis theorem on the
//              ni part); a change of unit is the dilator.
//    SPHERE    M = m (X(c) + (1/2) sigma^2 ni): centroid c = s/m and spread sigma^2 =
//              -M^2 / (M.ni)^2, an imaginary dual sphere -- the aggregate's bounding sphere with no
//              pass over its parts.
//
//  The moment BOX: the central tensor's horizontal block gives the heading and two variances, the
//  vertical one the height; a uniform box of half-width a has variance a^2/3, so half-extents are
//  sqrt(3 lambda), the horizontal pair scaled together to keep the footprint area m / (2 hz). One
//  law for one prism (it returns its own height, a rectangle returns itself) and for a crowd; every
//  box keeps volume, centroid and heading.
//
//  Frames: local metres, x east, y north, z up, about an origin the caller names (a cell's).
// ================================================================================================
#pragma once

#include "core/Cga.h"

#include <cstdint>
#include <vector>

namespace ga {

struct Moments {
    double m = 0.0;                 // volume (m^3)
    double s[3] = {0, 0, 0};        // first moment about the origin
    double S[6] = {0, 0, 0, 0, 0, 0};   // second: xx, yy, zz, xy, xz, yz

    Moments& operator+=(const Moments& o);
    // The same mass about an origin moved by o (the new origin's position in the old frame).
    Moments About(const double o[3]) const;
    // The conformal vector M = m n0 + s/L + (1/2) tr(S)/L^2 ni, lengths in units of L.
    cga::Mv Conformal(double unitL = 1.0) const;
    bool Empty() const { return !(m > 0.0); }
};

// A prism: rings in local metres (x0, y0, x1, y1, ...; any winding), rings[0] the outer and the
// rest holes, from z = bottom to z = top. Holes subtract whatever their winding. `outer`, when
// given (BuildingSolid's own flags), marks the further rings that ADD (a multipolygon's pieces).
Moments PrismMoments(const std::vector<std::vector<double>>& ringsXY, double bottom, double top,
                     const std::vector<uint8_t>* outer = nullptr);

struct MomentBox {
    double c[3] = {0, 0, 0};   // centroid
    double heading = 0.0;      // radians, of the long axis from +x toward +y
    double half[3] = {0, 0, 0};   // half-extents along (long, short, up), the footprint's area kept
    double spread[2] = {0, 0};    // the horizontal half-extents of the mass's own spread, sqrt(3 lambda)
    double cover = 1.0;           // the footprint's share of the spread's rectangle: half = spread sqrt(cover)
    double Radius() const;     // circumscribed: the size that picks the level
};
MomentBox BoxOf(const Moments& mo);

// The level a thing of circumscribed radius rho lives at: floor(log2(rho / rho0)), 0 below rho0.
int LodLevel(double rho, double rho0);
// How far level k is wanted: rho0 2^k / (pixels * pixAng), pixAng the radians a pixel subtends.
double LodReach(int level, double rho0, double pixels, double pixAng);

// The [lod] block of --selftest.
bool RunLodSelfTest();

}  // namespace ga
