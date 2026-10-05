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
//  sits on (core/Lattice.h): the 16k quad-sphere in the colour's 128x128 tiles and in the
//  height's 256x128; the tenants realized on them (hal::Tenant::Id); the eye's windows below;
//  and the stencil flag. PHASE B3: the Mercator z14 and z17 windows, their origins, their rows
//  (merc, det, u4) and their slices are deleted: every page past the faces is an eye's window.
//  Fill() writes the composed-surface rows.
//
//  THE KERNEL ROWS (step 4b). The churn reads the world.flat chart (`flat`, core/Space.h's
//  Anchor about the scene's place.anchor) through TangentRows(); their height reads carry the
//  windows' rows (KernelRows).
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
#include "hal/Tenant.h"

#include <cstdint>
#include <string>
#include <vector>

namespace ga {

struct ComposedSurfaceCb;
class ResidencyManager;

struct SurfaceFrame {
    double planetR = 0.0;
    // Planet -> tangent rotation rows: x east, y up at the anchor, z north (GlobeLayer's
    // frame; the session derives them from the anchor and checks the determinant).
    double east[3] = {1.0, 0.0, 0.0};
    double up[3] = {0.0, 1.0, 0.0};
    double north[3] = {0.0, 0.0, 1.0};
    // The lattices (core/Lattice.h): the colour and the survey on the cube's 128x128 tiles, the
    // height and the exposure on its 256x128 tiles.
    Lattice cube;    // the 16k quad-sphere, 128x128 tiles
    Lattice cubeH;   // the same cube, 256x128 tiles
    // The tenants realized on them (-1 = none).
    int colorT = -1, hgtT = -1, maskT = -1;
    bool hgtWindows = false;   // PHASE B2: the height tenant declares the eye's windows
    // HIERARCHY 4.17 commit 2: THE STANDING BLOCKS -- aligned blocks of the pyramid, each a
    // face-plane window anchored on a multiple of 16384 texels of its rung (FaceWindow,
    // core/Lattice.h), coarsest rung first, PHASE A1: the engine
    // declares none (the colour's and the mask's windows are the eye's); the selftests do.
    static constexpr uint32_t kMaxBlocks = 8;   // the rows ComposedSurfaceCb carries
    std::vector<FaceWindow> blocks;
    // HIERARCHY 4.17 commit 3: THE EYE the blocks' rows are taken about, planet frame, doubles --
    // the globe walk's own (GlobeLayer::CaptureWalk), written by the session before every Fill,
    // so the rows and the mesh records' eye-relative points share one origin.
    double eye[3] = {0.0, 0.0, 0.0};
    // One block's rows about a frame `own` whose origin is `eye` (planet frame): FaceWindow::
    // PlanesIn of the block re-anchored on the multiple of 16384 texels of its rung nearest the
    // eye, so every number a shader handles is small, and the whole blocks from that anchor to
    // the block's own origin (off), which the shader adds to the uv. A face the eye stands
    // behind keeps the block's own anchor.
    static void BlockRows(const FaceWindow& block, const Placement& own, const double eye[3],
                          FaceWindow::Planes& rows, float off[2]);
    static constexpr uint32_t kMaxRanks = 5;
    // A chain's step (SurfaceFrame::Chain): rank k + 1's slice and its own uv, in doubles.
    struct WalkStep {
        uint32_t slice;
        double u, v;
    };
    // STANDING blocks, declared by "lon,lat,rung" entries (degrees east and north) joined by ';'.
    // Each point is given the block of its rung that holds it, and the block is LOGGED with how
    // far the point stands inside it. A malformed list is refused aloud and leaves the blocks
    // empty. PHASE A1: the scene's key is gone and the engine declares none -- a following window
    // is the eye's (below) -- so these serve the selftests' standing windows
    // (and a solver's domain, later) alone.
    bool DeclareBlocks(const std::string& key);
    // Block i as the tenants declare it (hal/Tenant.h).
    hal::BlockBinding Block(size_t i) const;

    // ---- PHASE A1 (out/integration/plan_eye_windows.md): THE EYE'S WINDOWS. -------------------
    // Rank k's window about an eye is HIERARCHY 4.1's: a box of 16384 texels of rung 3k on the
    // face the eye stands over, about the eye's own address (4.4: X = u N, a ratio of two planes),
    // placed modulo 16384 (hal/Tenant.h's banner). Its origin is the eye's address less 8192,
    // rounded to kWindowStep -- so the box is whole tiles at the window's mips 0..3 -- and held on
    // the face. It STEPS when the eye's address has left the box's centre by more than
    // kWindowStep on either axis (4.19's held-set law on the box, the margin in texels of the rank:
    // the eye drifts that far before the box re-centres). K, the ranks live, is the eye's measure
    // (RanksAt, HIERARCHY 4.17): 1 + ceil(-L / 3), L = log2 of the pixel's footprint at the eye's
    // altitude over rank 1's texel on the ground there, clamped to 0..kMaxRanks.
    static constexpr uint32_t kWindowStep = 1024;   // texels of the rank: quantum and margin both
    // PHASE B2 (D1): THE STEP IS A WHOLE TILE AT THE FLOOR FOR EVERY TENANT SHARING THE WINDOWS, per
    // axis: step = the largest tile side of the sharing tenants x 2^3 -- with the height's 256 x 128,
    // 2048 texels in x and 1024 in y for all four. kWindowStep is the 128 x 128 value, the start.
    uint32_t step[2] = {kWindowStep, kWindowStep};
    void ShareWindows(uint32_t texW, uint32_t texH);
    // PHASE B2 (D4): THE STANDING WINDOW -- a solver's domain: one window of a rank about a fixed
    // place, held whole before the solver starts (water.swe.bedWait), at slice kStandingSlice of
    // every tenant. No blocks, no directory. Its rows about any frame `own` (origin `origin`, planet
    // frame): StandingRows -- one chain entry, its rank said by rank0.
    static constexpr uint32_t kStandingSlice = 6u + 8u * 5u;
    hal::BlockBinding standing{};
    uint32_t standingRank = 0;          // 0 = none
    double standingCentre[3] = {};      // the place it stands about, planet frame (on the sphere)
    bool StandAbout(double latDeg, double lonDeg, uint32_t rank);
    // PHASE B2: the window slices a tenant declares -- every set's ranks (WindowSlice) and the
    // standing window -- painted by `provider` on the pyramid. One declaration for the four tenants:
    // slice i is the same ground in all of them (HIERARCHY 4.1).
    void WindowBlocks(std::vector<hal::BlockSlice>& out, const TileProviderFn& provider,
                      const char* astField) const;
    // The standing window's frame: its centre on the sphere and its east / up / north there.
    Placement StandingFrame() const;
    // A2: one set a slot of the globe's level table (GlobeLayer::kMaxLevels, held equal there).
    static constexpr uint32_t kWindowSlots = 8;
    struct EyeWindows {
        uint32_t K = 0;
        hal::BlockBinding box[kMaxRanks];   // rank k + 1's window (rung 3 (k + 1))
    };
    // PHASE A3: A WINDOW SET IS THE GROUND'S, NOT THE TABLE'S. There are kWindowSlots sets of K
    // windows, set w at the slices WindowSlice(w, k); each frame every slot of the level table
    // claims a set (Assign) -- the one whose boxes already hold its eye, rank for rank, else the
    // one it held, else a free one -- and the set follows that slot's eye (Follow). A world that
    // changes its place in the table (the eye goes through a gate: the world that was slot 1 is now
    // slot 0) keeps its windows and their tiles; nothing is told, nothing is loaded again.
    // What the tenants are bound to and the walk wants (bound), and what the rows draw (drawn):
    // the bound windows of the frame before, so a reader never sees a box whose leaving tiles the
    // manager has not yet been told of (FrameLoop's step). Indexed by SET.
    EyeWindows bound[kWindowSlots], drawn[kWindowSlots];
    static constexpr uint32_t kNoSet = 0xFFFFFFFFu;
    uint32_t slotSet[kWindowSlots] = {0, 1, 2, 3, 4, 5, 6, 7};   // the set each slot reads
    // F9: THE SLICE SET w's RANK k READS. A window is an address (face, rung, origin); two claimed
    // sets whose boxes at a rank are one address are one window there, and the later reads the
    // earlier's slice (Share, after every Follow). Its own slice is then unread: no want, no
    // mapping, and the order releases what it held. sharedRanks counts the ranks read elsewhere.
    uint32_t slice[kWindowSlots][kMaxRanks] = {};
    uint32_t sharedRanks = 0;
    void Share();
    // The slice of the colour and the mask that holds set `w`'s rank `rank` (1..kMaxRanks).
    static uint32_t WindowSlice(uint32_t w, uint32_t rank) { return 6u + w * kMaxRanks + rank - 1u; }
    // The claim: slots 0..n-1 with their eyes (planet frame) take their sets; the other slots none.
    void Assign(uint32_t n, const double eyes[][3], double pixAng);
    static uint32_t WindowSlices() { return 6u + kWindowSlots * kMaxRanks + 1u; }   // + the standing
    // THE MEASURE: the ranks an eye (planet frame, metres) wants on a planet of radius R at a
    // pixel of angle pixAng; its face and L are handed back for the log.
    static int RanksAt(const double eye[3], double R, double pixAng, uint32_t* face = nullptr,
                       double* L = nullptr);
    // THE STEP: `slot`'s windows about `eye`. drawn takes the windows bound before; then K and the
    // boxes are found again. The slices whose box changed are appended to `moved` with their new
    // binding, for the tenants (hal::Tenant::Move).
    struct Moved {
        uint32_t slice;
        hal::BlockBinding to;
    };
    // Set `w` about `eye`. `eye` null: no slot claimed the set this frame; it holds no rank (its
    // slices keep their bindings and their tiles, for the order to release or a slot to claim).
    void Follow(uint32_t w, const double eye[3], double pixAng, std::vector<Moved>& moved);
    // A2: each slot's eye (planet frame, doubles) as Assign took it, for its rows (Fill).
    double slotEye[kWindowSlots][3] = {};
    uint32_t slotsLive = 1;
    // A slot's windows as the readers hold them: per rank its planes about the frame `own` (origin
    // the eye, BlockRows: anchored on the multiple of 16384 nearest it), its box's origin less that
    // anchor in 16384s, its slice and its mip-0 ground; and K. What Fill writes, and the selftest.
    struct ChainRows {
        uint32_t K = 0;
        FaceWindow::Planes pl[kMaxRanks] = {};
        float off[kMaxRanks][2] = {};
        uint32_t slice[kMaxRanks] = {};
        double ground[kMaxRanks] = {};
        uint32_t rank0 = 0;   // PHASE B2: entry k is rank rank0 + k + 1 (a standing window starts anywhere)
    };
    ChainRows StandingRows(const Placement& own, const double origin[3]) const;
    // PHASE B2: a kernel's rows -- slot `slot`'s windows as bound now (the boxes this frame's readers
    // draw), about that slot's eye (slotEye) in the tangent axes; and the slot whose eye stands
    // nearest a planet point (the bank's rings stand about one), within `reachM`, else ~0.
    ChainRows SlotRows(uint32_t slot) const;
    uint32_t SlotNear(const double p[3], double reachM) const;
    // PHASE B2: A WANT IN A WINDOW'S UV. The rectangles of a window slice (addressed modulo 16384)
    // that hold the global texels [x0, x1) x [y0, y1) of its rung, clipped to its box: one, two or
    // four -- what LeafWants does for a node, for any reader. Returns how many.
    static int SliceRects(const hal::BlockBinding& b, double x0, double y0, double x1, double y1,
                          float out[4][4]);
    // ...of the ground within `halfM` metres of a planet direction, or of a lat/lon box (degrees).
    static int SliceRectsAbout(const hal::BlockBinding& b, const double dir[3], double halfM,
                               double planetR, float out[4][4]);
    static int SliceRectsLL(const hal::BlockBinding& b, double lat0, double lon0, double lat1,
                            double lon1, float out[4][4]);
    // slices: the slice each rank reads (F9, SurfaceFrame::slice[set]); null = the set's own.
    static ChainRows RowsOf(const EyeWindows& w, uint32_t slot, const Placement& own, const double eye[3],
                            const uint32_t* slices = nullptr);
    // THE CHAIN, its C++ body (shaders/Window.hlsli is the HLSL one), op for op in float32: each
    // rank's address FaceWindow::PageTexel over 16384, plus its offset, in [0, 1); the ranks from 1
    // up to the first that does not hold p. Every rank's address is written; returns the chain's
    // length.
    static uint32_t Chain(const float p[3], const ChainRows& rows, WalkStep out[kMaxRanks]);
    // PHASE B1 (out/integration/plan_phase_b.md): THE ROWS A KERNEL CARRIES (shaders/WindowRows.hlsli
    // HP_WINDOW_ROWS_DECL, the last rows of the bank's, the churn's and the solver's cbuffers): the
    // same rows RowsOf gives a level, in the kernels' packing -- rank k + 1's planes U, V, W, the
    // box's origin less the anchor two ranks a row, the slices of ranks 1..4, then rank 5's and K.
    struct KernelWindowRows {
        float u[20], v[20], w[20];
        float o[12];
        uint32_t s[8];
    };
    static void KernelRows(const ChainRows& rows, KernelWindowRows& out);
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
    // THE WORLD.FLAT CHART (core/Space.h's Anchor): the tangent plane about the scene's
    // place.anchor, exact (PHASE C4/C5: the anchor-linear map and its geoA row are deleted).
    Space::Anchor flat;

    // The shipped surface on a planet of radius planetR: the lattices and the flat chart. The
    // tenants come later (Declare), the frame rows from the session.
    static SurfaceFrame About(double planetR, bool stencil, double latDeg, double lonDeg);
    // The tenants, once they exist: ids read off the declarations. An empty Tenant (never
    // declared) leaves -1.
    void Declare(const hal::Tenant& color, const hal::Tenant& height, const hal::Tenant& mask);
    // M12 step 4c: the compose pillar's paint rows, registered from the declaration (the
    // banner). Called by the Assembly after Declare() and before the AST's hand table and its
    // validator; idempotent, as every registration is.
    void RegisterEdges() const;
    // THE ONE FILL of the composed-surface rows (was FillComposedCb, Compositor.cpp).
    void Fill(ComposedSurfaceCb& cb, const ResidencyManager& rm) const;
    // PHASE C5: the kernels' tangent rows -- the chart's east, up and north in the planet frame
    // (w = R on the up row), for a kernel that turns a flat point into its direction.
    void TangentRows(float e[4], float u[4], float n[4]) const {
        for (int i = 0; i < 3; ++i) {
            e[i] = static_cast<float>(flat.east[i]);
            u[i] = static_cast<float>(flat.up[i]);
            n[i] = static_cast<float>(flat.north[i]);
        }
        e[3] = 0.0f;
        u[3] = static_cast<float>(flat.planetR);
        n[3] = 0.0f;
    }
};

}  // namespace ga
