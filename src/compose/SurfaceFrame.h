// ================================================================================================
//  SurfaceFrame.h - M12 step 4a: THE SHIPPED SURFACE, declared once.
//
//  WHAT THE SITES WERE DOING. The composed-surface rows (Compositor.h's ComposedSurfaceCb, the
//  constants every layer that samples the planet embeds) were filled twice a frame from two
//  copies of one declaration: the frame loop's, from the Assembly's ints and consts (the z14
//  window origin as `4935.0 * 256.0`, the z17 origin computed beside the colour tenant, the
//  planet radius, the tangent frame's rows the session derives from the anchor) for the
//  terrain, the sea and the GIS layer; and the globe's, from members that SetComposed,
//  SetFrame and SetPlanetRadius had copied out of the same numbers. Twenty-two positional
//  parameters each, and the z14 origin spelled as a literal in the Assembly's header and at
//  sixteen lines of Assembly.cpp, FrameLoop.cpp and three tools -- the tenants' lattices,
//  three SetHeightPage calls, a uv closure, two providers, three cache tags -- with the z17
//  origin computed at one place and spelled as a cache tag at two more. Step 0 planted a
//  FNV-1a fingerprint of the rows at both fills, and every render since has printed the same
//  hash from both at the helm, the globe and the droste pose: the two declarations agree, so
//  collapsing them is motion, and the two fingerprints stay as the gate.
//
//  THE DECLARATION. A SurfaceFrame is what the shipped surface IS: the planet's radius; the
//  tangent frame's rows (planet -> tangent: east, up at the anchor, north -- the doubles the
//  session computes from the anchor and writes here, once); the lattices every realization
//  sits on (core/Lattice.h): the 16k quad-sphere, the Merrimack z14 window and its z17 detail
//  window in the colour's 128x128 tiling, and the same cube and z14 window in the height's
//  256x128 tiling; the tenants realized on them, as the ids and the page slices read off
//  their declarations (hal::Tenant::Id, and SliceOf of the lattice's own Tag -- the numbers
//  the sites used to write as 6 and 7); and the stencil flag. Merrimack() writes the two
//  origins down -- the z14 window at tile (4935, 6008), the z17 window centred on 42.8160 N,
//  70.8125 W -- and no other line of the engine outside its tests does. Fill() is
//  FillComposedCb's body, moved: its former parameters are read from the members at the top
//  and the arithmetic below them is the old text.
//
//  WHAT `window` IS. The body tells a colour cube from a colour window and a height cube from
//  a height window, because they were once separate tenants; since M9ap/M9aq the window IS a
//  slice of the one page tenant (Assembly.cpp assigns winTenant = colorCubeT and hgtWinTenant
//  = hgtTenant right after Sparse() and nowhere else), so here the window is the colour tenant
//  at winSlice and the height window the height tenant at hgtWinSlice -- and there is NO
//  window (-1) where the slice is undeclared: Mars declares its MOLA height cube with no page
//  (Assembly.cpp's SetComposed(-1, -1, hgtTenant, -1, ...)), and Compose.hlsli turns the
//  height-window path on from gCsU2.z alone, so a window that is merely the tenant again would
//  have lit it. The body's `!pages` branches are then reachable only for Mars's cube, and are
//  kept as they were: this step moves, a later one prunes.
//
//  THE MERC ROW comes from Lattice::Rows(), which is the old four casts bit for bit: both
//  origins are below 2^24 and exact in float whether cast from the double or the integer,
//  1/16384 is 2^-14, and the world-pixel expression is the same text.
//
//  THE KERNEL ROWS (step 4b). The water bank, the churn and the solver each hand-filled a
//  winA row {org px x, org px y, 1/16384, 16384*256} from two origin doubles their
//  SetHeightPage / SetHeightWindow had been passed plus two literals, and the bank and the
//  churn a geoA row from BathyModel's four constants -- the world.flat chart in floats. The
//  three now take the height window's Lattice and fill winA from its Rows(); the chart lives
//  here as `flat` (core/Space.h's Anchor, written by Merrimack from the same constants) and
//  the bank and the churn read geoA through FlatRows(), the sites' own casts (a double
//  division, then one cast). The gate: a [kernel] FNV-1a of each kernel's constant buffer,
//  printed at its upload when it changes, equal before and after the move.
//
//  THE DIAGRAM'S PAINT ROWS (step 4c). GaAst.cpp's compose table hand-wrote the four rows
//  `compose.stack -> color.pages / height.pages` -- the z14 origin as a literal in a range
//  string, the frames, the flip flags, the slice numbers -- beside the tenants that declare
//  the same things. RegisterEdges() registers those rows from this declaration: the node and
//  the edge names Declare() reads off the tenants (TenantDesc::astNode, SliceBinding::astField,
//  kept here as `colorAst` / `hgtAst` with each binding's slices and lattice), the frames from
//  the lattices themselves (Lattice::AstFrame), the flip from the two frames' +v
//  (ast::NeedsFlip -- the rule GeoRef.h derives it by, never a literal), the range and the
//  anchor from the slices, the tags and the tile shape. The gate: the generated
//  docs/GA_AST.md differs from the checked-in one in those rows' range and code columns only,
//  and the probe frame's md5 does not move.
//
//  WHO WRITES IT. The Assembly builds it once (Merrimack), declares the tenants into it
//  (Declare) where it used to hand them to the globe, and hands the globe a pointer where it
//  used to hand it the radius; the session writes the frame rows through its aliases where it
//  used to compute them into its own members; the tools that name a realization take it. No
//  Direct3D type lives here (tools/hal_lint.py): the residency manager is read by tenant id.
// ================================================================================================
#pragma once

#include "core/Lattice.h"
#include "core/Space.h"

#include <cstdint>
#include <vector>

namespace ga {

struct ComposedSurfaceCb;
class ResidencyManager;
namespace hal {
class Tenant;
}

struct SurfaceFrame {
    double planetR = 0.0;
    // Planet -> tangent rotation rows: x east, y up at the anchor, z north (GlobeLayer's
    // frame; the session derives them from the anchor and checks the determinant).
    double east[3] = {1.0, 0.0, 0.0};
    double up[3] = {0.0, 1.0, 0.0};
    double north[3] = {0.0, 0.0, 1.0};
    // The lattices (core/Lattice.h). The colour and the survey sit on the first three; the
    // height and the exposure on the two 256x128 tilings of the same ground.
    Lattice cube;    // the 16k quad-sphere, 128x128 tiles
    Lattice win;     // the Merrimack z14 window
    Lattice det;     // its z17 detail window
    Lattice cubeH;   // the same cube, 256x128 tiles
    Lattice winH;    // the same z14 window, 256x128 tiles
    // The tenants realized on them (-1 = none) and the slices their pages are (UINT32_MAX =
    // none): the colour tenant holds the z14 page at winSlice and the z17 page at detSlice
    // (detT is the colour tenant: M9ap), the height tenant its z14 page at hgtWinSlice.
    int colorT = -1, hgtT = -1, maskT = -1, detT = -1;
    uint32_t winSlice = UINT32_MAX, detSlice = UINT32_MAX, hgtWinSlice = UINT32_MAX;
    // M12 step 4c: THE TENANTS' OWN WORDS FOR THE DIAGRAM, read off their declarations by
    // Declare() beside the ids and the slices: the node a tenant is (TenantDesc::astNode) and,
    // per binding, the edge it realizes (SliceBinding::astField), its slice range and its
    // lattice. RegisterEdges() registers the compose pillar's paint rows from these. An
    // undeclared tenant (Mars's height cube is an AddTextureCube; the packer and the selftest
    // declare none) has no node and no bindings, and gets no row. The survey's row is still
    // the hand table's: its tenant names three edges the table names as one (a later step).
    struct AstBinding {
        const char* field = "";
        uint32_t first = 0, count = 0;
        Lattice lattice;
    };
    struct AstTenant {
        const char* node = "";
        std::vector<AstBinding> bindings;
    };
    AstTenant colorAst, hgtAst;
    bool stencil = false;   // --stencil: the GIS alignment overlay
    // M12 step 4b: THE WORLD.FLAT CHART (core/Space.h's Anchor) -- the anchor-linear lat/lon
    // <-> metres map of BathyModel.h (lat = orgLat + z / mPerLat, lon = orgLon + x / mPerLon,
    // mPerLon frozen at the anchor) that the kernels' geoA row is made of. Merrimack writes it
    // from BathyModel's constants.
    Space::Anchor flat;

    // The Merrimack estuary's shipped surface on a planet of radius planetR: the lattices,
    // with the two window origins written here and nowhere else. The tenants come later
    // (Declare), the frame rows from the session.
    static SurfaceFrame Merrimack(double planetR, bool stencil);
    // The tenants, once they exist: ids and page slices read off the declarations. An empty
    // Tenant (never declared) leaves -1 / UINT32_MAX, as the ints did.
    void Declare(const hal::Tenant& color, const hal::Tenant& height, const hal::Tenant& mask);
    // M12 step 4c: the compose pillar's paint rows, registered from the declaration (the
    // banner). Called by the Assembly after Declare() and before the AST's hand table and its
    // validator; idempotent, as every registration is.
    void RegisterEdges() const;
    // THE ONE FILL of the composed-surface rows (was FillComposedCb, Compositor.cpp).
    void Fill(ComposedSurfaceCb& cb, const ResidencyManager& rm) const;
    // M12 step 4b: the kernels' geoA row -- {orgLat, orgLon, 1/mPerLat, 1/mPerLon} as floats,
    // the old four casts bit for bit (a double division, then one cast, as the sites wrote).
    void FlatRows(float geoA[4]) const {
        geoA[0] = static_cast<float>(flat.latDeg);
        geoA[1] = static_cast<float>(flat.lonDeg);
        geoA[2] = static_cast<float>(1.0 / flat.mPerLat);
        geoA[3] = static_cast<float>(1.0 / flat.mPerLon);
    }
};

}  // namespace ga
