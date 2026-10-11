// ================================================================================================
//  OriginLattice - EVERY FLOATING ORIGIN IS A POINT OF ONE LATTICE, AND A POINT PICKS ITS OWN.
//
//  The owner, 2026-10-11: "a point cloud of potential floating origins coming from the centre of
//  the earth", so that any leaf that needs an origin -- the root, a gate world, a bank's chart, a
//  portal made by a click on the minimap, the eye in orbit or inside the planet -- takes one by a
//  formula of its position alone, and two readers of one place agree without being told. The
//  night's bug was the counter-example: the bank's chart and the hull's chart at Haulover were
//  born by two rules and disagreed. Then: "every leaf should follow the appropriate floating
//  origin point of its nearest parent's cloud"; "the cloud isn't literal: an equation, indexed and
//  lazily made"; and "an equation that allows points to overlap, the preferred pass-through points
//  going from coarse to finer groups and back".
//
//  THE LATTICE IS DYADIC, SO IT NESTS. Directions: an icosahedron's 20 faces, each subdivided
//  2^L times along its edges at level L; a point is the normalised barycentric combination of its
//  face's corners. Every point of level L is a point of level L + 1 (its coordinates doubled), so a
//  coarse origin IS a fine origin: a child on a shared point keeps its parent's origin with no
//  motor, and a group folding up keeps its origin where it is shared. (Spherical Fibonacci sets,
//  the first form the same night, do not nest and were replaced by this.) Shells: radii
//  R exp(k eps_L) with eps_L = (the icosahedron's edge, 1.1072 rad) / 2^L, so a shell's neighbours
//  stand as far apart radially as laterally and every other shell of level L + 1 is a shell of L.
//  Under rMin the centre is the one origin: direction means nothing there.
//
//  THE IDENTITY IS LEVEL-FREE. An origin's `index` is its (face, i, j) at the finest level kLmax
//  (a level-L point is (f, i 2^(kLmax - L), j 2^(kLmax - L))), canonical for a point on an edge or
//  a corner (the lowest face that holds it owns it), and `shell` is in the finest level's units.
//  So Same() across levels is the geometric identity, and nothing is stored: Direction(index) is
//  the equation, InverseIndex(d) its closed-form inverse (the face by the largest dot with its
//  centre, the gnomonic barycentrics rounded to the grid, the nearest of the cell's corners), an
//  origin made when asked for, 100 bytes, held by whoever holds it.
//
//  THE VERSOR. An origin is a tangent frame (Space::Anchor: east, up, north at its direction) on
//  its shell: a point's floats are (east, up, north) . (P - c), the flat frame's own convention
//  (x east, y up, z north). Moving floats from origin i to origin j is ONE MOTOR, M_ij = A_j ~A_i,
//  taken in doubles (Between). Every frame change the engine makes between places is this product.
//
//  THE PICK keeps its origin until the point has left the cell by a margin (Pick): a point on a
//  boundary never flaps, and the switch, when it comes, is the same motor as any other.
//
//  WHOSE CLOUD: a lattice belongs to a PARENT body -- the Earth's about its centre at its radius,
//  Mars's its own, a Droste inner globe's its own at its scale -- and a leaf of the space tree
//  takes its origin from its nearest parent's cloud, the origin a property of a node inherited
//  down the tree. A node states its floats about its parent's origin until its content outruns
//  that origin's reach; then it takes the point of the cloud nearest to itself, and the motor
//  between the two (Between) IS its placement in the parent, which the tree carries already. The
//  cloud serves the GPU's floats: the CPU keeps doubles end to end and may use a coarser level or
//  none; the same equation on both sides, so handing the GPU a point is a cast.
//
//  Measured: --selftest [origin] (OriginLatticeTest.cpp), the numbers in docs/ORIGINS.md.
// ================================================================================================
#pragma once

#include "core/Pga.h"
#include "core/Space.h"

#include <cmath>
#include <cstdint>
#include <string>

namespace ga {

struct OriginLattice {
    double R = 6371008.8;        // the surface shell (k = 0)
    int level = 10;              // L: 2^L points along an icosahedron edge (level 10: 6.9 km apart at R)
    double rMin = 10000.0;       // under this radius the centre is the one origin
    static constexpr int kLmax = 24;                       // the finest level: the identity's units
    static constexpr double kEdgeRad = 1.1071487177940904; // the icosahedron's edge, radians (atan 2)
    double Eps() const { return kEdgeRad / double(1 << level); }   // this level's shell ratio

    struct Origin {
        bool centre = false;     // the planet's centre: the identity frame, c = 0
        int64_t shell = 0;       // k in the finest level's units: r = R exp(k eps_max)
        int64_t index = -1;      // the direction's canonical (face, i, j) at kLmax; -1 at the centre
        double r = 0.0;          // the shell's radius (m)
        double c[3] = {0, 0, 0}; // the origin's point, planet frame (m from the centre)
        Space::Anchor anchor;    // its tangent frame: east, up, north (unit, planet frame)
        bool Same(const Origin& o) const { return centre == o.centre && shell == o.shell && index == o.index; }
    };

    // The origin (shell, index); the centre for index < 0.
    Origin At(int64_t shell, int64_t index) const;
    // The origin nearest to P (planet frame, metres from the centre) at this level.
    Origin Nearest(const double P[3]) const;
    // THE GROUND'S ORIGIN under a point: the surface shell's point in P's direction -- what a chart
    // of the ground wants (the rings of the sea, a place, a hull), whatever the eye's altitude.
    Origin Surface(const double P[3]) const;
    Origin SurfaceAt(double latDeg, double lonDeg) const;
    // Nearest, with hysteresis: `current` (may be null) is kept while P stands within (1 + margin)
    // times its distance to the nearest -- one law, so a point on a cell's boundary never flaps.
    Origin Pick(const double P[3], const Origin* current, double margin = 0.25) const;
    // Surface, with the same hysteresis, judged along the ground.
    Origin PickSurface(const double P[3], const Origin* current, double margin = 0.25) const;
    // A point's floats about an origin: (east, up, north) . (P - c), in doubles for the caller to cast.
    static void Floats(const Origin& o, const double P[3], double out[3]);
    // ...and back: the planet point of floats about an origin.
    static void Point(const Origin& o, const double f[3], double P[3]);
    // THE MOTOR between two origins: floats about `from` -> floats about `to` (A_to^-1 A_from).
    static Motor Between(const Origin& from, const Origin& to);
    // This level's spacing on a shell: eps_L r.
    double Spacing(int64_t shell) const { return Eps() * RadiusOf(shell); }
    double RadiusOf(int64_t shell) const { return R * std::exp(double(shell) * (kEdgeRad / double(int64_t(1) << kLmax))); }
    int64_t ShellOf(double r) const {
        return int64_t(std::llround(std::log(r / R) / Eps())) * (int64_t(1) << (kLmax - level));
    }

    // THE DIRECTIONS. The direction of a canonical index, and the canonical index of the point of
    // THIS level nearest to a unit vector. (The index is level-free; the level says which points
    // are candidates.) Points(): how many this level has, 10 x 4^L + 2.
    static void Direction(int64_t index, double d[3]);
    int64_t InverseIndex(const double d[3]) const;
    int64_t Points() const { return int64_t(10) * (int64_t(1) << (2 * level)) + 2; }
    // Every point of this level by ordinal over the faces' (i, j) triangles, as its canonical
    // index -- for a gate that walks the whole lattice; an ordinal that lands on a shared point
    // another spot owns returns -1, so a walk sees each point once. Ordinals run to 20 (N+1)(N+2)/2.
    int64_t IndexOfOrdinal(int64_t ordinal) const;
    int64_t Ordinals() const { const int64_t N = int64_t(1) << level; return 20 * (N + 1) * (N + 2) / 2; }
    // The index of a level-L point (f, i, j), canonical; the decode at kLmax, for instruments.
    static int64_t Encode(int level, int face, int64_t i, int64_t j);
    static void Decode(int64_t index, int& face, int64_t& i, int64_t& j);
};

// A SPACE ON THE CLOUD: the child of `parent` (a body: the planet) standing on origin `o` --
// its frame the origin's tangent frame, its placement the origin's point, its stand recorded so
// Space::Declare accepts it. The only way a space gets a stand; the four spaces the engine makes
// under a planet (a gate's place, a hull's own, and in time the scene's root) are made here.
Space SpaceAt(const Space& parent, const OriginLattice::Origin& o, const std::string& name);

// The [origin] block of --selftest: the inverse against brute force, the nesting, the float bound
// on every shell, the motor between origins, the hysteresis, the centre, and a plant.
bool RunOriginSelfTest();

}  // namespace ga
