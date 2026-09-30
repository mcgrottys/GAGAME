// ================================================================================================
//  Tenant.h - M12 step 3e: THE SPARSE DEFAULT. A GPU-resident field is ONE declaration -- a
//  fiber, the slices, a lattice per slice binding, the provider per binding, what absence
//  means -- and the streaming, the page dispatch and the invalidation follow from it.
//
//  WHAT THE SITES WERE DOING. Five page tenants (earth.height, swell.exposure, earth.color,
//  gis.landsea, wave.field) each hand-wrote the same two closures: a DISPATCHER that split a
//  TileRequest on its slice -- `face < 6` to the cube provider unchanged, a page slice to that
//  page's provider with `w.face = 0` (the wave: `w.face = r.face - 6`, the plane), the slices
//  nobody binds answering a literal pattern (the exposure's 32768 halves of 0x3C00, the wave's
//  65536 zeros) -- and an onChanged that mapped a tree's lattice TAG back to a slice
//  (`window_z14` -> 6, `window_z17` -> 7, else the face) and queued Invalidate(id, q). Two of
//  them read the tree through a shared_ptr holder with std::atomic_load, because a bucket roll
//  swaps the tree under the provider. The same shape five times, no two quite alike (one
//  ignored the tag, one had no onChanged at all, one answered ones where another answered
//  zeros), and the GPU-resident law -- what is resident, on which lattice, fed by whom, and
//  what a missing tile means -- was recoverable only by reading all five.
//
//  THE DECLARATION. TenantDesc says it once: the FIBER (the format, the tile's texel shape,
//  and what one texel holds, in words); the SEMANTICS (a NULL tile MEANS zero -- a field, which
//  a shader adds unconditionally -- or ABSENT -- a texture, whose residency map clamps the
//  sample to the finest resident ancestor); the RESIDENCE (what it costs to bring a tile back:
//  a stream from the NVMe, a recompute of a cached identity, or nothing -- it is volatile and
//  the next bucket repaints it); the ABSENCE (what an unbound slice or a missing tile means:
//  zero, not loaded yet, outside the domain, or "only the coarser level exists"); the bytes an
//  unbound slice answers; and the BINDINGS -- a contiguous slice range, the lattice it sits on,
//  the provider that paints it (or none: the tree in the holder, on that lattice), and the AST
//  edge it realizes. Sparse() gates the declaration, registers it with the residency manager
//  (AddTexturePages, unchanged) and builds the ONE dispatcher; Bind(tree) hangs the ONE
//  onChanged; the boot log prints the declaration as the law it is:
//      [tenant] <name>: id N, <fmt> <texW>x<texH> "<quantity>", <Semantics>/<Residence>/
//      <Absence>, slices S: [first..last] <lattice tag> <-- <node> (<edge>), ...
//
//  THE DISPATCHER IS THE OLD LAMBDA, SAID ONCE. A request on slice f finds the binding that
//  contains f and calls its provider with `w.face = f - binding.first`: the face itself for
//  the cube (first = 0: the request is unchanged), 0 for a single-slice page, the plane for
//  the wave -- exactly what the five closures computed by hand. An unbound slice answers
//  `absentTile` (a copy) with a default TileLoc and true, as the exposure's and the wave's
//  faces 0..5 did; a binding with no provider reads the holder per request (two atomic loads:
//  the holder, then the tree in it) and answers false when there is no tree, as the two holder
//  lambdas did. That the bytes are the same was step 3e's gate: the [settle-exact] mapped set
//  of every tenant at the helm, tenant by tenant, before and after.
//
//  THE HOLDER IS BOUND BEFORE THE FIRST REQUEST -- and the first request is INSIDE
//  AddTexturePages: the manager loads and maps every slice's coarsest tile synchronously at
//  registration (Residency.cpp AddTextureInternal, "THE CLAIM IS MADE TRUE AT BIRTH"), so a
//  holder handed over after Sparse() returned would have answered those loads with "no tree":
//  a failed boot tile, a 255 in the residency map and an async retry, which is not what the
//  lambdas did. So the holder rides the declaration (TenantDesc::holder) and Sparse() installs
//  it through Bind(holder) before it registers. Bind(holder) stays public for a site that
//  swaps HOLDERS; none does today -- the roll swaps the tree inside the holder, and the
//  dispatcher's atomic_load sees it, exactly as before.
//
//  ONE LAW FOR THE ONCHANGED CLOSURES. Bind(tree) sets the tree's bound lattice (the first
//  binding's: the cube for the page tenants, the page for the exposure; TileTree::Provider()
//  with no argument is then the tree on it) and hangs
//      onChanged(tag, r):   q = r;  q.face = SliceOf(tag) + r.face;  Invalidate(Id(), q)
//  where SliceOf(tag) is the FIRST SLICE of the binding whose lattice.Tag() the tag names. The
//  closures compared PREFIXES (`window_z14`, `window_z17`) because a tenant binds one window
//  per zoom, and the tag a tree announces IS its lattice's Tag() (TileTree::Provider and
//  Prefill both take it from the frame) -- so an exact compare against each binding's tag
//  names the same slice the prefix did, and the cube's tag ("cube16k") names slice 0, where
//  the closures left the face alone: 0 + r.face is r.face. A window tree paints face 0, so
//  6 + 0 and 7 + 0 are the 6 and 7 the closures wrote. A tag no binding declares is routed to
//  the cube binding (else the first) and said once, because it is a lattice this tenant never
//  declared. The wave tenant binds no tree: it never had an onChanged (its prefill fills a
//  tree nobody serves yet, and the roll Drops the tenant whole), and giving it one would queue
//  invalidations the old code never did.
//
//  THE TILED-RESOURCE CONTRACTS this declaration stands on, each where it is kept:
//    * the tier is stored and gated: Gpu::Init stores TiledResourcesTier (Gpu::TiledTier) and
//      warns below TIER_2; TileSelfTest::Run (TileAtlas.cpp) fails the suite below tier 2, so
//      --selftest is the refusal;
//    * NULL tiles read zero and swallow writes: TileSelfTest::Run phase 1 (RunWrite2D over the
//      whole surface, then Verify2D against the checkerboard) proves both on this adapter;
//    * CheckAccessFullyMapped is truthful: the same phase's status word (TileAtlas.cpp, the
//      "of 5 probes, how many reported CheckAccessFullyMapped == true" probe);
//    * the tile shape is queried, never assumed: TileAtlas2D::Init takes m_tileW/m_tileH from
//      GetResourceTiling's D3D12_TILE_SHAPE; ResidencyManager::AddTextureInternal derives its
//      virtual layout from the format and prints the queried shape beside it; Sparse() here
//      refuses a binding whose lattice's texel shape is not the fiber's;
//    * the packed-mip floor is pinned: TileAtlas2D::Init's m_pinnedFloor (the packed tail, or
//      the coarsest standard mip where a shape has no tail) and ActivateSlice pin it per
//      slice; the page tenants stop their chain at the one-tile level instead (no packed mips,
//      GA_CHECKed in AddTextureInternal), map that level at birth and never evict it
//      (MapAndFill's evictor skips mips - 1; the exact settle never drops it);
//    * coarse before fine: Want() enqueues the ancestor column first and the ring gate admits
//      a request only under a MAPPED parent; ProcessQueues' gather maps a tile only when its
//      parent is mapped (parentOk); the evictor never takes a tile with a mapped child
//      (childMapped) -- the classic sample's ordering invariant;
//    * linear -> swizzled only in CopyTiles: every fill is CopyTiles with
//      LINEAR_BUFFER_TO_SWIZZLED_TILED_RESOURCE (AddTextureInternal's boot fill, the landed
//      DirectStorage loop and MapAndFill's ring fill in Residency.cpp; GradeBank::
//      UploadDenseTiles), and TileStream.h says why a DirectStorage read lands in a buffer
//      rather than in the tile;
//    * the NULL map is delayed past the frame overlap: a dropped tile goes to m_retiring
//      (DropOne) and is NULL-mapped kEvictAgeFrames turns later (ProcessQueues' retire loop),
//      unless the coordinate was re-mapped meanwhile; the landing slots retire the same way
//      (m_stageRetire, kStageRetireFrames); the evictor takes only tiles unseen for
//      kEvictAgeFrames (MapAndFill);
//    * a residency claim waits on the DirectStorage fence: a direct-read tile is mapped but its
//      residency byte is written only once its batch's fence has signalled (MapAndFill's
//      claim-if-ring rule; the landed loop in ProcessQueues) -- until then the sampler reads
//      the coarser ancestor;
//    * the pool cap refuses rather than evicts what is still wanted: at kPoolCapTiles the
//      evictor takes only an aged, childless, non-floor tile and, finding none, breaks out of
//      the batch (MapAndFill); during an exact settle the cap test is skipped so the pool
//      holds the want set (Residency.h settleExact).
//
//  DX12-first: the fiber's format is DXGI's and the manager's texture is the reserved
//  Texture2DArray it always was; nothing here is virtual. The residency manager keeps the
//  temporal-ownership contract (Context.h) by TURN COUNT, not by fence: a dropped tile's NULL
//  map and a DirectStorage landing slot retire 4 turns later (kEvictAgeFrames,
//  kStageRetireFrames), longer than the frame ring's overlap.
// ================================================================================================
#pragma once

#include "core/Lattice.h"
#include "core/TileAddress.h"
#include "hal/Gpu.h"
#include "hal/Residency.h"

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace ga {
class TileTree;
}

namespace ga::hal {

// What one texel holds, and the tile's texel shape it is stored in: the LATTICE's texW x texH
// (Lattice.h) and the format's 64 KB tile agree by construction, and Sparse() checks it.
struct Fiber {
    DXGI_FORMAT fmt = DXGI_FORMAT_UNKNOWN;
    uint32_t texW = 0, texH = 0;
    const char* quantity = "";   // "m NAVD88 (half float)", "sRGB colour, alpha = coverage"
};
// A NULL tile MEANS zero (a field: the shader adds it) or absent (a texture: the residency
// map clamps to the finest resident ancestor).
enum class Semantics : uint8_t { Field, Texture };
// What it COSTS to bring a tile back.
enum class Residence : uint8_t { Streamable, Recomputable, Volatile };
// What an UNBOUND slice, or a missing tile, means.
enum class Absence : uint8_t { Zero, Unloaded, OutOfDomain, Coarse };

struct SliceBinding {
    uint32_t first = 0, count = 0;   // the slices [first, first + count)
    Lattice lattice;                 // the ground these slices tile
    TileProviderFn provider;         // paints one tile of it; empty = the tree in the holder
    const char* astField = "";       // the GA AST edge this binding realizes
};

struct TenantDesc {
    const wchar_t* name = L"";
    const char* astNode = "";
    Fiber fiber;
    Semantics semantics = Semantics::Field;
    Residence residence = Residence::Streamable;
    Absence absence = Absence::Zero;
    // The bytes an unbound slice answers -- the exposure's 0x3C00 halves, the wave's zeros.
    // Empty: an unbound slice answers false (honestly nothing).
    std::vector<uint8_t> absentTile;
    uint32_t slices = 0;
    std::vector<SliceBinding> bindings;
    // The swapped trees (exposure, wave): where the tree lives, read per request. Bound by
    // Sparse() before the first request -- see the banner on why it rides the declaration.
    std::shared_ptr<std::shared_ptr<TileTree>> holder;
};

// A handle on the registered tenant. Copyable: the state it names is shared with the
// dispatcher the residency manager holds, so a Tenant that dies releases nothing the manager
// still reads. Default-constructed it is empty (Id() -1).
class Tenant {
public:
    Tenant() = default;
    // AddTexturePages + the dispatcher, after the declaration's gates (bindings inside the
    // slices and disjoint, one face dimension, the lattices' tile shape the fiber's, a
    // 64 KB absentTile or none, a holder where a binding has no provider). Throws on a
    // malformed declaration, as every hal builder does at boot. Logs the [tenant] line.
    static Tenant Sparse(Gpu& gpu, ResidencyManager& mgr, TenantDesc desc);

    int Id() const;
    bool Valid() const { return m_s != nullptr; }
    // The lattice slice `slice` sits on; null for an unbound slice.
    const Lattice* LatticeOf(uint32_t slice) const;
    // The first slice of the binding whose lattice.Tag() is `latticeTag`; an undeclared tag
    // routes to the cube binding (else the first) and is said once.
    uint32_t SliceOf(const std::string& latticeTag) const;
    // tree.SetLattice(the first binding's lattice) and tree.onChanged = the one invalidation
    // law: Invalidate(Id(), {SliceOf(tag) + r.face, r.mip, r.x, r.y}).
    void Bind(TileTree& tree);
    // The holder the dispatcher reads per request (std::atomic_load, as the lambdas did).
    void Bind(std::shared_ptr<std::shared_ptr<TileTree>> holder);
    const TenantDesc& Desc() const;

private:
    struct State;
    std::shared_ptr<State> m_s;
};

}  // namespace ga::hal
