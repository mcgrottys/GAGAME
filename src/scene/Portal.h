// ================================================================================================
//  Portal - M12 step 5e: THE LINK TO AN ANCESTOR SPACE, AS A SCENE NODE.
//
//  The Droste tower is one edit to the scene graph: a leaf of the planet's quadtree gets the
//  ROOT as its child (core/Droste.h). The session built it by hand -- `portal =
//  droste::BuildPortal(...)` from the flags, then (step 4d) `drosteLeaf = Space::Cycle(...)`
//  beside it -- and every Droste read in the frame went through those two members. This
//  component is that construction as a node: its props are the address (lat, lon, level), the
//  fill, the twist and the lighting (SceneSchema.h PortalSchema, the `portals` section); Init
//  runs BuildPortal UNCHANGED (it stays the builder: the address resolves into the leaf, the
//  rest contact, the scale and the forced fixed point there) and declares the cycle the tangent
//  space hangs under -- Space::Cycle("droste.leaf", tangent, Placement::Similar(p, s, axis,
//  twist)), the ONE place a scale enters the frame calculus. The frame loop reads Link() and
//  Cycle() where it read the members, and the two boot lines print as they did.
//
//  What it needs, and reports when it is not given: the tangent frame's rows and the planet's
//  radius (the leaf's direction and the twist axis are expressed in them), the compositor and
//  its height channel (the inner globe RESTS on the composed ground at the leaf's centre), and
//  the tangent Space (the cycle's `within`). No device object: Init(Gpu&) is the build.
//
//  Prior art, named: Hart & DeFanti 1991 (cyclic instancing, rendered until it is smaller than
//  a pixel), the portal recursion of Portal (Valve, 2007) and Minecraft's Immersive Portals, and
//  USD's `references` (a prim that stands for another subtree). The node form is Godot's: a
//  scene file instances the link by type and property. What is this engine's own is stated in
//  Droste.h: a whole streamed planet as a leaf of its own sparse address tree, one versor.
// ================================================================================================
#pragma once

#include "core/Droste.h"
#include "core/Space.h"
#include "scene/Component.h"
#include "scene/SceneSchema.h"

#include <string>
#include <vector>

namespace ga {
class Compositor;
}

namespace ga::scene {

class Portal final : public Component {
public:
    struct Observers {
        const double* east = nullptr;    // the tangent frame's rows (compose/SurfaceFrame.h)
        const double* up = nullptr;
        const double* north = nullptr;
        double planetR = 0.0;
        Compositor* compositor = nullptr;   // the composed ground the globe rests on
        int hgtCh = -1;
        const Space* tangent = nullptr;     // the cycle's `within`
        bool globe = false;                 // the world has a globe layer (no globe: no tower)
        bool marsMode = false;              // Mars has no estuary window and no tower
    };

    // ---- Component -------------------------------------------------------------------------
    const char* Name() const override { return m_props.name.c_str(); }
    const Schema& Props() const override { return PortalSchema(); }
    std::vector<std::string> Configure(const Wiring& w) override;
    bool Init(Gpu& gpu) override;   // the build (no device object)
    void Apply(const PropSet& props) override;
    void Update(const FrameInfo& f) override;
    void Record(const ViewContext& v) override;
    void ReloadShaders() override {}

    // ---- the declaration and the wiring -----------------------------------------------------
    PortalProps& Declared() { return m_props; }
    const PortalProps& Declared() const { return m_props; }
    std::vector<std::string> Configure(const Observers& o);
    // THE DESTINATION (the Haulover portal demo, 2026-09-14): the place the inner globe presents
    // where the root shows this leaf. BuildPortal's rotation is ONE rotor, and it was the twist
    // about north alone; with a destination it is the twist AFTER the shortest arc that carries
    // the destination onto the leaf's own place. That is the whole change: the inner planet is
    // the root, so an eye that dives into the inner globe at that spot and re-roots arrives at
    // the destination at full scale. Without one, BuildPortal receives exactly the numbers it
    // always did (absence is the identity, not a zero-length arc).
    void SetDestination(double latDeg, double lonDeg);
    bool HasDestination() const { return m_hasTo; }
    // The rotor, as the axis and angle BuildPortal and Placement::Similar take: the shortest arc
    // carrying `destPlanet` onto `leafPlanet` (both planet-frame radials, rotated into the tangent
    // rows first), then `twistRad` about north. False, with the reason, when the destination is
    // the leaf's antipode (every great circle is a shortest arc there).
    static bool Carry(const double destPlanet[3], const double leafPlanet[3], const double east[3],
                      const double up[3], const double north[3], double twistRad,
                      double axisOut[3], double& angleOut, std::string* why);

    // BuildPortal from the declaration, and the cycle. False (and Valid() false) when the
    // scene has no tower: disabled, no globe, or Mars.
    bool Build();

    // ---- what the frame reads ---------------------------------------------------------------
    const droste::Portal& Link() const { return m_link; }
    const Space& Cycle() const { return m_cycle; }
    bool Valid() const { return m_link.Valid(); }

private:
    PortalProps m_props;
    bool m_hasTo = false;
    double m_toLat = 0.0, m_toLon = 0.0;
    Observers m_o;
    droste::Portal m_link;
    Space m_cycle;
};

}  // namespace ga::scene
