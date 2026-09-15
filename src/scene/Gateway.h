// ================================================================================================
//  Gateway - a cuboid whose far side is another place on the same planet (2026-09-14). A body whose
//  centre of gravity enters the box leaves from the destination.
//
//  ONE MOTOR. The box sits in its space -- the root tangent space, the Merrimack's flat frame --
//  at Entry, the motor of its sugar {x, alt, z, az}: a turn about +y to the compass heading, then
//  the translation. The destination is a place (toLat, toLon) with a heading toAz: its own tangent
//  space under the SAME planet (Space::To carries any point of it into the root's coordinates,
//  through the common ancestor), and Exit is that heading about its up at its origin. The carry
//  is K = Exit * Entry^-1: a pose relative to the entry box, expressed relative to the exit box.
//  Both boxes are y-up in their own spaces, so K is a turn about y and a translation; the planet's
//  curvature between the two places lives in the two spaces' placements, not in the carry.
//
//  NOTHING ELSE MOVES. The world frame, the other bodies and every view stay where they are; the
//  carried body changes its SPACE and its pose in it (Entity::Teleport). Two players on the two
//  sides of a gate, or two views of one split screen, see one consistent world.
//
//  ONE-WAY. The destination has no box, so a camera there sees the body appear.
//
//  THE SAME SPARSE WATER. The destination space carries its chart -- Space::Anchor at the place,
//  by the engine's own linear law (110574 m a degree of latitude, 111320 cos(lat) of longitude,
//  BathyModel's constants evaluated there) -- and a carried hull's TreeWater reads the ONE
//  weather manager through it: the planet's tiles, at the destination's lat/lon.
// ================================================================================================
#pragma once

#include "core/Pga.h"
#include "core/Space.h"
#include "scene/SceneSchema.h"

#include <string>

namespace ga::scene {

class Gateway {
public:
    GateProps& Declared() { return m_props; }
    const GateProps& Declared() const { return m_props; }

    // The box from its sugar's numbers (the flat frame's {x, alt, z, az}, in `source`), the
    // destination space under `planet`. False, logged, when the declaration names no place.
    bool Build(const Space& planet, const Space& source, double planetR, double x, double alt,
               double z, double azDeg);
    bool Valid() const { return m_valid; }

    const Space& Source() const { return *m_source; }
    const Space& Destination() const { return m_dest; }
    const Space::Anchor& Chart() const { return m_chart; }
    const Motor& Entry() const { return m_entry; }
    const Motor& Exit() const { return m_exit; }
    // K = Exit * Entry^-1: a pose of the source space carried to the destination space.
    const Motor& Carry() const { return m_carry; }
    // The destination space's placement in the source space, as a motor (the draw and the view
    // of a carried body): proper, because both spaces are frames of one planet.
    const Motor& DestinationInSource() const { return m_destInSource; }

    // A point of the SOURCE space inside the box: |local| <= half the size on every axis.
    bool Inside(double px, double py, double pz) const;

private:
    GateProps m_props;
    bool m_valid = false;
    const Space* m_source = nullptr;
    Space m_dest;
    Space::Anchor m_chart;
    Motor m_entry, m_entryInv, m_exit, m_carry, m_destInSource;
};

}  // namespace ga::scene
