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

// A provider fills one 64KB tile's worth of LINEAR data for a texture tenant. Runs on a worker
// thread; must be self-contained and cache-first (HTTP providers throttle themselves and honor
// a hard per-run fetch budget). Returns false if the tile cannot be produced (kept NULL).
struct TileRequest {
    uint32_t face = 0, mip = 0, x = 0, y = 0;
};
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
