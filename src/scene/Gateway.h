// ================================================================================================
//  Gateway - a cuboid whose far side is another place on the same planet (2026-09-14). A body whose
//  centre of gravity enters the box leaves from the destination.
//
//  TWO PLACES, ONE MOTOR. A place is a tangent frame of the planet and the chart that reads the
//  planet's data there (Place). The box stands at one -- the scene's own flat frame, unless the gate
//  names another (fromLat, fromLon) -- at Entry, the motor of its sugar {x, alt, z, az[, pitch]}:
//  the turn to the compass heading and the lean, then the translation (BoxPose). It comes out at
//  another place (toLat, toLon; the scene's own frame when none is named) at Exit, which is the SAME
//  structure posed again: `toAt`'s own sugar, or `toAz` alone for a box that only turns. The carry
//  is K = Exit * Entry^-1: a pose relative to the entry box, expressed relative to the exit box.
//  NEITHER END IS LOCKED UPRIGHT. A box with no pitch is y-up because its sugar declared no lean,
//  and then K is a turn about y and a translation, as the first gates were; give the exit a pitch
//  and K carries the lean with it -- a hull that drives in level leaves nose-down and keeps
//  falling (there is no air on it to right it), and the window, which carries ray directions the
//  same way, looks down on the far place from a level look at the box. The planet's curvature
//  between the two places lives in the two spaces' placements, not in the carry.
//
//  ONE SPACE PER PLACE. The caller keeps one space for each place (FrameLoop's registry), so a body
//  carried to a place stands in the very frame a gate standing there was built in: a gate at
//  Haulover back to the Merrimack meets the boat the first gate carried there, and nothing about
//  that is a special case.
//
//  NOTHING ELSE MOVES. The world frame, the other bodies and every view stay where they are; the
//  carried body changes its SPACE and its pose in it (Entity::Teleport). Two players on the two
//  sides of a gate, or two views of one split screen, see one consistent world.
//
//  ONE-WAY. A gate is a box on one side only; a way back is another gate. Two gates that see each
//  other through their far sides are the classic corridor of windows within windows -- and that is
//  not a feature either: WindowChain (below) follows a view into whatever box it enters and, from
//  that box's far side, into whatever box it meets there.
//
//  THE SAME SPARSE WATER. The destination carries its chart -- Space::Anchor at the place, by the
//  engine's own linear law (110574 m a degree of latitude, 111320 cos(lat) of longitude) with the
//  exact rows beside it -- and a carried hull's TreeWater reads the ONE weather manager through it:
//  the planet's tiles, at the destination's lat/lon.
// ================================================================================================
#pragma once

#include "core/Pga.h"
#include "core/Space.h"
#include "scene/SceneSchema.h"

#include <string>
#include <vector>

namespace ga::scene {

// A PLACE: a tangent frame of the planet (y up at its origin), hung under the planet by the rule
// every frame in the engine is built by, and the chart that reads the planet's data there.
struct Place {
    Space space;
    Space::Anchor chart;
    double latDeg = 0.0, lonDeg = 0.0;
    bool Build(const Space& planet, double planetR, double latDeg, double lonDeg,
               const std::string& name, std::string* why);
};

class Gateway {
public:
    Gateway() = default;
    Gateway(const Gateway&) = delete;              // the first form points at its own place
    Gateway& operator=(const Gateway&) = delete;

    GateProps& Declared() { return m_props; }
    const GateProps& Declared() const { return m_props; }

    // ONE BOX, POSED TWICE. The two ends are the same structure -- a cuboid of `size` -- put
    // somewhere and turned: this is that pose, from the compass sugar's numbers, in whichever
    // place's flat frame the end stands in. The heading turns it about the place's up and the
    // pitch about the box's own across-axis (nose up positive, a camera's sense), so a box with
    // no pitch is y-up because nothing asked it to lean, not because a gate must be. A pitch of
    // -90 stands the box's far face underfoot: what you drive into going east, you leave going
    // straight down. (Any orientation at all is expressible: the placement sugar's motor
    // spelling reaches this as a Motor, and Build takes the motors.)
    static Motor BoxPose(double x, double alt, double z, double azDeg, double pitchDeg);
    // The box at `source`, coming out as `exit` in `dest` -- both poses said in their own place's
    // flat frame. `root` is the frame the scene is drawn in -- every view's eye stands in it --
    // and the window's motor is said there. The spaces and the chart must outlive the gate.
    // False, logged, when the box has no size.
    bool Build(const Space& source, const Space& dest, const Space::Anchor& destChart,
               const Space& root, const Motor& entry, const Motor& exit);
    // The first gate's form: the box in the root frame `source`, the destination the place the
    // declaration names (toLat, toLon) -- built and owned here -- and the exit at its origin.
    bool Build(const Space& planet, const Space& source, double planetR, const Motor& entry);
    bool Valid() const { return m_valid; }

    const Space& Source() const { return *m_source; }
    const Space& Destination() const { return *m_dest; }
    const Space::Anchor& Chart() const { return *m_chart; }
    const Motor& Entry() const { return m_entry; }
    const Motor& Exit() const { return m_exit; }
    // K = Exit * Entry^-1: a pose of the source space carried to the destination space.
    const Motor& Carry() const { return m_carry; }
    // The destination space's placement in the source space, as a motor: proper, because both
    // spaces are frames of one planet.
    const Motor& DestinationInSource() const { return m_destInSource; }
    // THE SAME MAPS, SAID IN THE ROOT FRAME: the destination's placement, the box's, and THE
    // WINDOW -- the motor taking a point that enters the box to where it comes out, root to root:
    //     W = DestInRoot * K * RootInSource.
    // A view is carried through a gate by W, and the far side is drawn back through W^-1.
    const Motor& DestinationInRoot() const { return m_destInRoot; }
    const Motor& EntryInRoot() const { return m_entryInRoot; }
    const Motor& Window() const { return m_window; }

    // A point inside the box: |local| <= half the size on every axis -- in the SOURCE frame, and
    // the same said for a point of the ROOT frame.
    bool Inside(double px, double py, double pz) const;
    bool InsideRoot(double px, double py, double pz) const;

    // THE WINDOW'S ONE TEST. The segment from an eye to a point, both in the source frame, reaches
    // the box's entry at or before the point: the point is seen THROUGH the window. The
    // destination's level keeps exactly those points; every other level keeps exactly the rest --
    // two complementary discards, no stencil, no second target, and nothing about which camera is
    // asking.
    bool SeenThrough(const double eye[3], const double p[3]) const;
    // ...and the same slab walked from `tStart` on, with the box placed at `boxInRoot` and both
    // ends in the root frame -- a window seen through earlier windows stands where their pulls put
    // it. True when the segment's part beyond tStart enters the box at or before p; *tIn is where.
    // Common.hlsli GateSlabFrom is its line-for-line translation.
    bool SeenThroughFrom(const Motor& boxInRoot, const double eye[3], const double p[3],
                         double tStart, double* tIn) const;

private:
    GateProps m_props;
    bool m_valid = false;
    const Space* m_source = nullptr;
    const Space* m_dest = nullptr;
    const Space::Anchor* m_chart = nullptr;
    Place m_ownDest;   // the first form's destination
    Motor m_entry, m_entryInv, m_exit, m_carry, m_destInSource;
    Motor m_destInRoot, m_entryInRoot, m_entryInRootInv, m_window;
};

// ---- THE VIEW THROUGH THE GATES ------------------------------------------------------------------
// What an eye sees through the gates, as a chain: the first box its view enters, then -- from that
// box's far side, where the view carries on -- the next box the carried view meets, and so on. Each
// link is a place the view has reached: `carry` takes the true eye there (root to root) and its
// inverse `pull` draws that place's world in the true frame.
//
// A LINK IS A WINDOW, NOT A SCREEN. What a link shows is its world rasterized from the TRUE eye and
// kept, per pixel, where the segment from the eye passes every box of the chain so far in order
// (Common.hlsli GateSlabFrom, walked by ChainDepth below and by the shaders). So every depth moves
// with the eye exactly as the world behind a pane of glass does. `rect` and `zNear` only bound
// those rays -- a CULL for the walk, conservative by construction: every ray through the box meets
// the box, so its tan lies inside the corners' bounding rectangle, and it is seen only past its entry
// point, which lies no nearer along the view axis than the box's nearest corner.
struct ViewCone {
    double eye[3] = {0.0, 0.0, 0.0};   // root frame
    double fwd[3] = {0.0, 0.0, 1.0};   // the render basis
    double right[3] = {1.0, 0.0, 0.0};
    double up[3] = {0.0, 1.0, 0.0};
    double tanX = 1.0, tanY = 1.0;     // the view's half extents in tan-space
    double pixTan = 1.0e-3;            // one pixel, in tan-space
};
struct WindowLink {
    const Gateway* gate = nullptr;
    Motor carry;           // root -> root: the eye through this window and every one before it
    Motor pull;            // carry^-1: this link's world, drawn in the true frame
    Motor boxInRoot;       // this window's box where the true eye sees it
    double rect[4] = {0.0, 0.0, 0.0, 0.0};   // tan-space x0, x1, y0, y1 of the rays through it
    double zNear = 0.0;    // nothing nearer than this, along the view axis, is seen through it
    bool visible = false;  // some ray of the view passes through it
};
// `forced` are taken first, in order, seen or not -- the windows a view owes (its subject went
// through them). Past those, the nearest box the view still reaches, within `reachM` of the eye,
// until `maxDepth` links or a window narrower than two pixels: the screen ends the corridor.
std::vector<WindowLink> WindowChain(const std::vector<const Gateway*>& gates,
                                    const std::vector<const Gateway*>& forced,
                                    const ViewCone& view, int maxDepth, double reachM);
// How many links of `chain` the segment from the eye to p passes, in order: the depth of the world
// p belongs to (Globe.hlsl GateChainDepth).
int ChainDepth(const std::vector<WindowLink>& chain, const double eye[3], const double p[3]);
// The link's walk cull: four side planes through the eye and the near plane, eye-relative in the
// root frame's axes (a x + b y + c z + d >= 0 inside). A link no ray reaches culls everything.
void LinkPlanes(const WindowLink& link, const ViewCone& view, double planes[5][4]);
// WHAT A WINDOW HIDES: the cone from the eye through the face of the link's box the eye sees best,
// beyond that face (planes as LinkPlanes', inside = hidden). Every ray through that face enters the
// box after every window before it, so every point in this volume belongs to a deeper world than
// the one in front of the window -- the world in front need not walk there. False when the eye is
// in the box or no ray reaches it.
bool LinkHole(const WindowLink& link, const ViewCone& view, double planes[5][4]);

}  // namespace ga::scene
