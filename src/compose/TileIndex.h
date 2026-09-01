// ================================================================================================
//  TileIndex - M9ae: THE DISK IS THE TREE; MEMORY HOLDS THE INDEX.
//
//  The architecture was always disk -> CPU shard -> GPU reserved resource, and two thirds of it
//  are already built. The compositor transcodes and composes tiles ONCE and writes them to
//  cache/composed/<channel>/<realization>/ as 64 KB blobs "laid out exactly for CopyTiles, which
//  is the DirectStorage-ready folder" -- 20666 of them, 1.3 GB, on this machine right now.
//
//  What was missing is the middle: nothing in memory knew what the disk held. A tile was located
//  by BUILDING A PATH AND OPENING IT, so "is there a finer level here?" could only be answered by
//  guessing a filename and asking the filesystem -- per tile, per frame, per tenant. The residency
//  manager therefore could not prefer what was READY; it could only request a level and discover
//  afterwards whether that meant a cheap read or an expensive paint (and, for a tile tree source,
//  a network fetch).
//
//  That is the direct cause of the reported "medium detail then a drop to low": residency asks
//  for the level the walk wants, the fine tiles are not on disk yet, and the far field waits at
//  its coarsest resident ancestor while the budget goes to painting tiles nobody can see yet.
//
//  So: scan the cache once at boot, keep the sparse index in RAM, and let residency ask it. The
//  index is small by construction -- one entry per tile, not one texel -- so a tree whose DATA is
//  terabytes has an index in megabytes. That asymmetry is the entire point of paging to NVMe: the
//  tree can be as big as the disk, and what memory holds is the map, never the territory.
//
//  WHAT AN ENTRY IS NOT: it is not a promise that the tile is correct. The filename carries the
//  SOURCE-SUBSET hash the tile was painted from, so a tile painted before a source was added is
//  still on disk under its old hash and must not be served. The index keeps the hash and the
//  lookup checks it -- a stale tile is a MISS, exactly as if it were absent, which is what makes
//  adding a source safe rather than a cache-poisoning event.
// ================================================================================================
#pragma once
#include <cstdint>
#include <cstdio>
#include <string>
#include <unordered_map>
#include <vector>

#include "core/Common.h"

namespace ga {

class TileIndex {
public:
    // (face:3 | mip:5 | y:28 | x:28) -- the same decomposition CachePath prints, so an entry and
    // a filename are two renderings of one address.
    using Key = uint64_t;
    static Key MakeKey(uint32_t face, uint32_t mip, uint32_t x, uint32_t y) {
        return (uint64_t(face) << 61) | (uint64_t(mip) << 56) |
               (uint64_t(y & 0xFFFFFFFull) << 28) | uint64_t(x & 0xFFFFFFFull);
    }

    struct Entry {
        uint32_t subset = 0;   // source-subset hash the tile was painted from
        uint32_t bytes = 0;
    };

    // Scan one realization folder. Returns tiles indexed. Cheap: 20k filenames parse in
    // milliseconds, and nothing is opened -- the directory entry carries the size.
    uint32_t Scan(const std::string& channel, const std::string& realization) {
        const std::string dir = "cache\\composed\\" + channel + "\\" + realization;
        m_name = channel + "/" + realization;
        uint32_t added = 0;
        WIN32_FIND_DATAA fd{};
        HANDLE h = FindFirstFileA((dir + "\\*.bin").c_str(), &fd);
        if (h == INVALID_HANDLE_VALUE) return 0;
        do {
            uint32_t f = 0, m = 0, x = 0, y = 0, sub = 0;
            if (sscanf_s(fd.cFileName, "f%u_m%u_x%u_y%u_%8x.bin", &f, &m, &x, &y, &sub) == 5) {
                Entry e;
                e.subset = sub;
                e.bytes = fd.nFileSizeLow;
                m_tiles[MakeKey(f, m, x, y)] = e;
                if (m >= m_mipSeen.size()) m_mipSeen.resize(m + 1, 0);
                ++m_mipSeen[m];
                ++added;
            }
        } while (FindNextFileA(h, &fd));
        FindClose(h);
        return added;
    }

    // Is this exact tile on disk, painted from the subset we would paint it from now?
    bool Has(uint32_t face, uint32_t mip, uint32_t x, uint32_t y, uint32_t subset) const {
        const auto it = m_tiles.find(MakeKey(face, mip, x, y));
        return it != m_tiles.end() && it->second.subset == subset;
    }

    // THE QUESTION RESIDENCY ACTUALLY WANTS: what is the finest level already on disk covering
    // this ground? Walks from `finest` up its own ancestry, which is O(mips) pointer-free
    // lookups and no filesystem contact at all. Returns `coarsest + 1` when nothing is held.
    uint32_t FinestReady(uint32_t face, uint32_t finest, uint32_t coarsest, uint32_t x,
                         uint32_t y, uint32_t subset) const {
        for (uint32_t m = finest; m <= coarsest; ++m) {
            const uint32_t sx = x >> (m - finest), sy = y >> (m - finest);
            if (Has(face, m, sx, sy, subset)) return m;
        }
        return coarsest + 1;
    }

    size_t Count() const { return m_tiles.size(); }
    const std::string& Name() const { return m_name; }

    void Report() const {
        std::string per;
        char b[32];
        for (size_t m = 0; m < m_mipSeen.size(); ++m) {
            if (!m_mipSeen[m]) continue;
            snprintf(b, sizeof(b), " m%zu:%u", m, m_mipSeen[m]);
            per += b;
        }
        Log("[tileindex] %s: %zu tiles on disk (%.2f MB of index for %.0f MB of tiles)%s",
            m_name.c_str(), m_tiles.size(),
            m_tiles.size() * (sizeof(Key) + sizeof(Entry)) / 1048576.0,
            m_tiles.size() * 64.0 / 1024.0, per.c_str());
    }

private:
    std::string m_name;
    std::unordered_map<Key, Entry> m_tiles;
    std::vector<uint32_t> m_mipSeen;
};

}  // namespace ga
