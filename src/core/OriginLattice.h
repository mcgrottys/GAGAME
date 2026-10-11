// ================================================================================================
//  OriginLattice - EVERY FLOATING ORIGIN IS A POINT OF ONE LATTICE, AND A POINT PICKS ITS OWN.
//
//  The owner, 2026-10-11: "a point cloud of potential floating origins coming from the centre of
//  the earth", so that any leaf that needs an origin -- the root, a gate world, a bank's chart, a
//  portal made by a click on the minimap, the eye in orbit or inside the planet -- takes one by a
//  formula of its position alone, and two readers of one place agree without being told. The
//  night's bug was the counter-example: the bank's chart and the hull's chart at Haulover were
//  born by two rules and disagreed.
//
//  THE LATTICE. Shells of radius r_k = R (1 + eps)^k about the planet's centre, k any integer with
//  r_k >= rMin (under that, the centre itself is the one origin: direction means nothing there);
//  on every shell n directions of a SPHERICAL FIBONACCI set, whose nearest point to a direction is
//  a closed form (Keinert, Innmann, Saenger, Stamminger, "Spherical Fibonacci Mapping", SIGGRAPH
//  Asia 2015): no search, no table. eps = sqrt(4 pi / n), so a shell's neighbours stand as far
//  apart radially as laterally, and the spacing is the shell's: s_k = eps r_k. A point is never
//  further than about s_k from its origin, so its floats carry a grain of s_k 2^-24: at the
//  surface with n = 2^22 the origins are 11 km apart and the grain is under a millimetre; in orbit
//  the shell is larger and so is the grain, relative precision being the planet's own law.
//
//  THE VERSOR. An origin is a tangent frame (Space::Anchor: east, up, north at its direction) on
//  its shell: a point's floats are (east, up, north) . (P - c), the flat frame's own convention
//  (x east, y up, z north). Moving floats from origin i to origin j is ONE MOTOR, M_ij = A_j ~A_i,
//  taken in doubles (Between). Every frame change the engine makes between places is this product.
//
//  THE PICK keeps its origin until the point has left the cell by a margin (Pick): a point on a
//  boundary never flaps, and the switch, when it comes, is the same motor as any other.
//
//  WHOSE CLOUD (the owner, the same night): a lattice belongs to a PARENT body -- the Earth's about
//  its centre at its radius, Mars's its own, a Droste inner globe's its own at its scale -- and a
//  leaf of the space tree takes its origin from its nearest parent's cloud, the origin a property
//  of a node inherited down the tree. A node states its floats about its parent's origin until its
//  content outruns that origin's reach; then it takes the point of the cloud nearest to itself, and
//  the motor between the two (Between) IS its placement in the parent, which the tree carries
//  already. So the cloud supplies the candidates, the tree says who follows whom, and no leaf --
//  a tile, a page of buildings, a gate world, a hull, a portal made by a click on the minimap --
//  chooses an origin by a rule of its own.
//
//  Measured (--selftest [origin], 2026-10-11): the inverse equals brute force on every point of a
//  4096-set and on 200 directions of the 2^22-set; floats stay within 0.67 of the bound on the
//  surface shell (11.0 km spacing, 0.93 mm grain), 100 km up, in orbit at 20,000 km (46 km, 3.9
//  mm) and 3,000 km down (5.8 km, 0.5 mm); M_ij to 7e-9 m; the hysteresis loop 0.19 spacings wide.
// ================================================================================================
#pragma once

#include "core/Pga.h"
#include "core/Space.h"

#include <cmath>
#include <cstdint>

namespace ga {

struct OriginLattice {
    double R = 6371008.8;        // the surface shell (k = 0)
    uint32_t n = 1u << 22;       // directions a shell (spherical Fibonacci)
    double rMin = 10000.0;       // under this radius the centre is the one origin
    double Eps() const { return std::sqrt(4.0 * 3.14159265358979323846 / double(n)); }

    struct Origin {
        bool centre = false;     // the planet's centre: the identity frame, c = 0
        int shell = 0;           // k: r = R (1 + eps)^k
        int64_t index = -1;      // the direction's Fibonacci index (-1 at the centre)
        double r = 0.0;          // the shell's radius (m)
        double c[3] = {0, 0, 0}; // the origin's point, planet frame (m from the centre)
        Space::Anchor anchor;    // its tangent frame: east, up, north (unit, planet frame)
        bool Same(const Origin& o) const { return centre == o.centre && shell == o.shell && index == o.index; }
    };

    // The origin (k, i); the centre for i < 0.
    Origin At(int shell, int64_t index) const;
    // The origin nearest to P (planet frame, metres from the centre).
    Origin Nearest(const double P[3]) const;
    // Nearest, with hysteresis: `current` (may be null) is kept while P stands within (1 + margin)
    // times its distance to the nearest -- one law, so a point on a cell's boundary never flaps.
    Origin Pick(const double P[3], const Origin* current, double margin = 0.25) const;
    // A point's floats about an origin: (east, up, north) . (P - c), in doubles for the caller to cast.
    static void Floats(const Origin& o, const double P[3], double out[3]);
    // ...and back: the planet point of floats about an origin.
    static void Point(const Origin& o, const double f[3], double P[3]);
    // THE MOTOR between two origins: floats about `from` -> floats about `to` (A_to^-1 A_from).
    static Motor Between(const Origin& from, const Origin& to);
    // A shell's spacing: eps r_k, the distance between neighbouring origins on it.
    double Spacing(int shell) const { return Eps() * RadiusOf(shell); }
    double RadiusOf(int shell) const { return R * std::pow(1.0 + Eps(), double(shell)); }
    int ShellOf(double r) const { return int(std::lround(std::log(r / R) / std::log(1.0 + Eps()))); }

    // THE SPHERICAL FIBONACCI SET, the polar axis on y as the planet frame has it. Direction i,
    // and the index of the direction nearest to a unit vector (Keinert 2015, closed form).
    void Direction(int64_t i, double d[3]) const;
    int64_t InverseIndex(const double d[3]) const;
};

// The [origin] block of --selftest: the inverse against brute force, the float bound on every
// shell, the motor between origins, the hysteresis, the centre, and a plant.
bool RunOriginSelfTest();

}  // namespace ga
