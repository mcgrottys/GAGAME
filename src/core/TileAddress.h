// ================================================================================================
//  TileAddress.h - M12 step 2a: a tile's ADDRESS, and how a provider answers for it, with no
//  Direct3D in sight.
//
//  TileRequest, TileLoc and TileProviderFn used to live in Residency.h beside the manager that
//  streams them. Everything above the streamer speaks them too -- the compositor's realizations,
//  the tile trees, the lattice a realization sits on -- and none of that needs d3d12.h. Now that
//  the residency manager is moving under src/hal/ (step 3), the address has to be a core type,
//  or the whole compositor would inherit the hardware header through one struct of four ints.
//  Moved verbatim; nothing about an address changed.
// ================================================================================================
#pragma once

#include <atomic>
#include <cstdint>
#include <functional>
#include <vector>

namespace ga {

// Step 5 E (review finding 83; HIERARCHY 4.19, "a tile is held whole or it is not held"): set on
// the calling thread by a tile tree that answers a tile without one of its sources (a refused
// fetch: the composite skips the child, `complete` is false, the tile is returned and not
// stored). The residency manager clears it before its provider call and reads it
// after, on the same thread; a tile not whole is not delivered.
inline thread_local bool g_tileIncomplete = false;
// PHASE A4 (HIERARCHY 4.20, the third clause), the same channel: set by a tree whose tile is
// finer than every source's own level, so the level above magnified -- no bytes; the manager
// maps nothing for it and the reader's residency byte names the parent.
inline thread_local bool g_tileMagnified = false;
// THE BOOT'S QUESTION (2026-10-17): set by the residency manager around its synchronous load of a
// tenant's coarsest tiles. A tile tree then answers only what it HOLDS (a file, an archive, its
// memory) and returns "not now" for anything it would have to paint, compose or fold. The boot was
// written for "a handful of cache-first tiles"; on a tree identity never built (the helm recipe's
// colour, its seafloor relief painting its own fine level and folding up) one coarse tile pulled a
// whole fine window through the main thread: 45 minutes with nothing logged. Not now is the boot's
// honest path: the map says nothing there and the async load paints it.
inline thread_local bool g_tileHeldOnly = false;
// THE LOADER'S STOP (2026-10-17): a loader points this at its own quit flag around a provider call
// (ResidencyManager::RunLoad, HeightTree's jobs). Once it is set the tree paints, composes and folds
// nothing more and answers "not now", so a load composing a cold pyramid -- minutes of work in one
// job -- leaves at once instead of holding the loader's Shutdown / Close (the helm bench's exit sat
// there). What the load already wrote stands: a tile on disk is whole, the next run resumes there.
inline thread_local const std::atomic<bool>* g_tileStop = nullptr;
// THE LANE'S PAINT PERMIT (2026-10-21): set by the residency manager around its provider call
// (ResidencyManager::RunLoad). A load that paints, composes or folds holds one of its tenant's
// permits from its first such step to its end; a tenant has `share` of them -- the Io lane's
// threads over the tenants painting and one more -- so a tenant whose tiles cost minutes cold
// (the swell's exposure) never holds the lane, and a load that only reads (milliseconds) needs
// none. A load refused one answers "not now", and its tile waits in the order for a permit.
struct PaintPermit {
    std::atomic<int>* painting = nullptr;   // the tenant's loads holding a permit now
    int share = 1;
    bool held = false, refused = false;
    bool Take() {
        if (held) return true;
        if (painting->fetch_add(1, std::memory_order_relaxed) < share) return held = true;
        painting->fetch_sub(1, std::memory_order_relaxed);
        refused = true;
        return false;
    }
};
inline thread_local PaintPermit* g_paintPermit = nullptr;
// Is the tree to answer only what it holds? Under the boot's question, and for a loader that is leaving.
inline bool TreeStopped() {
    return g_tileHeldOnly || (g_tileStop && g_tileStop->load(std::memory_order_relaxed));
}
// May a tree paint now? Not when stopped; under a loader's permit, only with one taken.
inline bool TreeMayPaint() { return !TreeStopped() && (!g_paintPermit || g_paintPermit->Take()); }
// Was this load refused a permit? Its "not now" is the permit's, not the tile's.
inline bool TreePaintRefused() { return g_paintPermit && g_paintPermit->refused; }

// A provider fills one 64KB tile's worth of LINEAR data for a texture tenant. Runs on a worker
// thread; must be self-contained and cache-first (HTTP providers throttle themselves and honor
// a hard per-run fetch budget). Returns false if the tile cannot be produced (kept NULL).
struct TileRequest {
    uint32_t face = 0, mip = 0, x = 0, y = 0;
};
// THE LOAD THIS THREAD IS MAKING (2026-10-14): the residency manager names its tenant and tile
// around its provider call, so a source that cannot answer now (a fetch queued on the network,
// GoogleTileProvider) can say whom to tell when it can. -1: no load (a tool, a CPU reader).
inline thread_local int g_loadTenant = -1;
inline thread_local TileRequest g_loadReq{};
// M9ai: WHERE A TILE IS, instead of what it contains.
//
// A provider that finds its tile in an archive fills this and returns true WITHOUT touching
// out64k. The bytes then never enter CPU address space at all: DirectStorage takes the path,
// the offset and the mapped tile, and the read goes NVMe -> GPU. A provider that has to paint,
// or whose tile is only a loose file, fills out64k as before and leaves this empty -- so the
// two paths coexist per TILE, not per build.
struct TileLoc {
    const wchar_t* path = nullptr;   // archive path; owned by the archive, outlives the request
    uint64_t offset = 0;
    uint32_t size = 0;
    bool Valid() const { return path != nullptr && size != 0; }
};

using TileProviderFn =
    std::function<bool(const TileRequest&, std::vector<uint8_t>& out64k, TileLoc* loc)>;
// F14: a question about a tile that needs no bytes to answer -- "is this tile its parent,
// magnified?" is arithmetic over the sources' own levels (HIERARCHY 4.20), and the order asks it
// on the main thread instead of spending a load to be told.
using TileQueryFn = std::function<bool(const TileRequest&)>;

}  // namespace ga
