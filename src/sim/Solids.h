// ================================================================================================
//  Solids - what the ground law reads beside the bed (2026-10-12).
//
//  A right prism in the body's own flat frame: its footprint as (x, z) pairs, its bottom and top
//  as frame heights, a circle about it for the reach test. A building, a bridge deck, a kerb are
//  all this, and the law neither knows nor asks which: a contact point inside one is pushed out
//  through its NEAREST face by the same spring, grip and cone the bed uses, and a strut's ray
//  takes the nearer of the bed and a top. So a car stops at a wall and drives onto a roof by the
//  one expression it already had. Produced by whoever knows the data (scene/Entity.cpp composes
//  the buildings stack about the hull, in the hull's chart); the physics reads a list.
// ================================================================================================
#pragma once

#include <vector>

namespace ga {

struct SolidPrism {
    std::vector<double> xz;              // x0, z0, x1, z1, ... (any winding; closed implicitly)
    double yBottom = 0.0, yTop = 0.0;    // frame heights
    double cx = 0.0, cz = 0.0, radius = 0.0;
};

// Even-odd point in polygon over (x, z) pairs.
bool PrismHolds(const std::vector<double>& xz, double x, double z);
// From a point inside: the distance to the nearest side and that side's outward unit normal (x, z).
void PrismNearestSide(const std::vector<double>& xz, double x, double z, double& dist, double n[2]);

}  // namespace ga
