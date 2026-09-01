// ================================================================================================
//  TileArchive - M9ah: ONE FILE PER REALIZATION, NOT ONE FILE PER TILE.
//
//  The composed cache is 20666 loose 64 KB files. That was the right shape while the compositor
//  was the only reader -- a tile is written once, read by path, and the filesystem is the index.
//  It is the wrong shape for DirectStorage, and for a reason that survives every other
//  optimisation: a request eliminates the COPY, not the OPEN. Per-file open is a system call and
//  a metadata lookup per tile, paid on the thread issuing reads, and at 48 loads in flight that
//  is the cost that remains after the copy is gone.
//
//  So a realization packs into one archive and the index carries (offset, size) into it. Then a
//  tile read is: one file handle held open for the whole run, one offset, one enqueue. Which is
//  exactly the request shape DirectStorage is built around, and exactly the two fields
//  TileIndex::Entry has been carrying since it was written.
//
//  THE LOOSE FILES REMAIN THE SOURCE OF TRUTH. Packing is explicit (--pack-tiles), the archive
//  is a derived artefact, and a stale or missing archive falls back to the loose scan rather
//  than serving wrong bytes. That matters because the compositor still WRITES loose files as it
//  paints -- an archive is a snapshot of what was painted up to a point, and tiles painted after
//  it must still be found. Any tile the archive does not carry is looked up the old way.
//
//  LAYOUT, deliberately boring:
//
//      magic "GAAR" | version | tileCount | reserved
//      tileCount x { key64, offset64, size32, subset32 }      sorted by key, bsearch-able
//      payloads, 64 KB each, in key order
//
//  The directory is read once at open and becomes the in-memory index; the payloads are never
//  touched by the CPU at all when DirectStorage is doing the reading.
// ================================================================================================
#pragma once
#include <cstdint>
#include <cstdio>
#include <string>
#include <vector>

#include "core/Common.h"

namespace ga {

class TileArchive {
public:
    static constexpr uint32_t kMagic = 0x52414147u;   // "GAAR"
    static constexpr uint32_t kVersion = 1u;
    static inline uint32_t m_lastSuperseded = 0;   // reported by the last Pack()

    struct Rec {
        uint64_t key = 0;
        uint64_t offset = 0;
        uint32_t size = 0;
        uint32_t subset = 0;
    };

    static std::string PathFor(const std::string& channel, const std::string& realization) {
        return "cache\\composed\\" + channel + "\\" + realization + ".gaa";
    }

    // ---- writing -------------------------------------------------------------------------
    // Pack every loose tile of a realization. Returns tiles written. The loose files are NOT
    // deleted: the archive is derived, and deleting the only copy of 1.3 GB of painted work to
    // save a directory entry is not a trade worth making silently.
    static uint32_t Pack(const std::string& channel, const std::string& realization) {
        const std::string dir = "cache\\composed\\" + channel + "\\" + realization;
        std::vector<Rec> recs;
        std::vector<std::string> names;
        std::vector<uint64_t> stamps;   // last-write time, to pick the survivor per address
        WIN32_FIND_DATAA fd{};
        HANDLE h = FindFirstFileA((dir + "\\*.bin").c_str(), &fd);
        if (h == INVALID_HANDLE_VALUE) return 0;
        do {
            uint32_t f = 0, m = 0, x = 0, y = 0, sub = 0;
            if (sscanf_s(fd.cFileName, "f%u_m%u_x%u_y%u_%8x.bin", &f, &m, &x, &y, &sub) != 5) {
                continue;
            }
            Rec r;
            r.key = (uint64_t(f) << 61) | (uint64_t(m) << 56) |
                    (uint64_t(y & 0xFFFFFFFull) << 28) | uint64_t(x & 0xFFFFFFFull);
            r.size = fd.nFileSizeLow;
            r.subset = sub;
            recs.push_back(r);
            names.push_back(fd.cFileName);
            stamps.push_back((uint64_t(fd.ftLastWriteTime.dwHighDateTime) << 32) |
                             fd.ftLastWriteTime.dwLowDateTime);
        } while (FindNextFileA(h, &fd));
        FindClose(h);
        if (recs.empty()) return 0;

        // Sort by key, then by subset, so equal-key runs are contiguous and the directory is
        // bsearch-able. Payloads land in the same order, which puts neighbouring tiles of a mip
        // adjacent on disk -- what makes a burst of reads for one region sequential rather than
        // scattered across a 1 GB file.
        std::vector<uint32_t> order(recs.size());
        for (uint32_t i = 0; i < order.size(); ++i) order[i] = i;
        std::sort(order.begin(), order.end(), [&](uint32_t a, uint32_t b) {
            if (recs[a].key != recs[b].key) return recs[a].key < recs[b].key;
            return recs[a].subset < recs[b].subset;
        });
        // ONE SURVIVOR PER ADDRESS. ~40% of this cache is tiles painted before a source was
        // added -- same address, older subset hash, superseded and never served. The loose files
        // keep them (they are the source of truth and deleting painted work to save a directory
        // entry is not a trade to make silently), but an ARCHIVE is derived, so carrying them
        // would be 464 MB of the 1.14 GB spent on bytes nothing can ask for.
        //
        // The survivor is the most recently WRITTEN, which is the only ordering the filesystem
        // actually knows: a subset hash is an identity, not a version, so it cannot say which of
        // two tiles is newer. Ties keep the first, which is arbitrary and harmless -- two tiles
        // written in the same 100 ns tick are the same paint.
        {
            std::vector<uint32_t> keep;
            keep.reserve(order.size());
            for (size_t i = 0; i < order.size();) {
                size_t j = i, best = i;
                while (j < order.size() && recs[order[j]].key == recs[order[i]].key) {
                    if (stamps[order[j]] > stamps[order[best]]) best = j;
                    ++j;
                }
                keep.push_back(order[best]);
                i = j;
            }
            m_lastSuperseded = uint32_t(order.size() - keep.size());
            order.swap(keep);
        }
        const uint32_t distinct = uint32_t(order.size());

        const std::string out = PathFor(channel, realization);
        FILE* fo = nullptr;
        if (fopen_s(&fo, out.c_str(), "wb") != 0 || !fo) {
            Log("[tilearch] cannot write %s", out.c_str());
            return 0;
        }
        const uint32_t n = uint32_t(order.size());
        const uint64_t dirBytes = uint64_t(n) * sizeof(Rec);
        const uint64_t payload0 = 16 + dirBytes;
        uint64_t off = payload0;
        std::vector<Rec> sorted;
        sorted.reserve(n);
        for (uint32_t i : order) {
            Rec r = recs[i];
            r.offset = off;
            off += r.size;
            sorted.push_back(r);
        }
        const uint32_t hdr[4] = {kMagic, kVersion, n, 0u};
        fwrite(hdr, sizeof(hdr), 1, fo);
        fwrite(sorted.data(), sizeof(Rec), n, fo);
        std::vector<uint8_t> buf;
        uint32_t wrote = 0;
        for (uint32_t k = 0; k < n; ++k) {
            const std::string src = dir + "\\" + names[order[k]];
            FILE* fi = nullptr;
            if (fopen_s(&fi, src.c_str(), "rb") != 0 || !fi) continue;
            buf.resize(sorted[k].size);
            const size_t got = fread(buf.data(), 1, buf.size(), fi);
            fclose(fi);
            if (got != buf.size()) continue;
            fwrite(buf.data(), 1, buf.size(), fo);
            ++wrote;
        }
        fclose(fo);
        Log("[tilearch] packed %s: %u tiles, %u superseded dropped (%.0f%% of the folder), "
            "%.1f MB, directory %.2f MB (loose files kept)",
            out.c_str(), wrote, m_lastSuperseded,
            (wrote + m_lastSuperseded)
                ? 100.0 * double(m_lastSuperseded) / double(wrote + m_lastSuperseded)
                : 0.0,
            double(off) / 1048576.0, double(dirBytes) / 1048576.0);
        return wrote;
    }

    // ---- reading -------------------------------------------------------------------------
    bool Open(const std::string& channel, const std::string& realization) {
        m_path = PathFor(channel, realization);
        FILE* f = nullptr;
        if (fopen_s(&f, m_path.c_str(), "rb") != 0 || !f) return false;
        uint32_t hdr[4] = {};
        if (fread(hdr, sizeof(hdr), 1, f) != 1 || hdr[0] != kMagic || hdr[1] != kVersion) {
            fclose(f);
            return false;
        }
        m_recs.resize(hdr[2]);
        const bool ok = hdr[2] == 0 || fread(m_recs.data(), sizeof(Rec), hdr[2], f) == hdr[2];
        fclose(f);
        if (!ok) {
            m_recs.clear();
            return false;
        }
        // The path the reader hands DirectStorage -- opened once for the whole run, not per tile.
        m_wpath.assign(m_path.begin(), m_path.end());
        return true;
    }

    // Directory lookup. Sorted by key, so this is a bsearch over a contiguous array -- no hash,
    // no pointer chase, and the whole directory for 20k tiles is under half a megabyte.
    //
    // THE SUBSET IS PART OF THE LOOKUP, not a check afterwards, because a key can legitimately
    // appear MORE THAN ONCE: the cache keeps every tile ever painted, including ones painted
    // before a source was added, and those differ only in their subset hash. Measured on this
    // cache: earth.color/cube16k packs 4340 tiles for 2473 distinct addresses, so 43% of that
    // archive is superseded. A bsearch on key alone lands on an arbitrary member of that run
    // and would have served stale imagery at random -- the exact cache-poisoning the loose-file
    // naming scheme was designed to prevent. So the search finds the run and walks it.
    const Rec* Find(uint64_t key, uint32_t subset) const {
        size_t lo = 0, hi = m_recs.size();
        while (lo < hi) {
            const size_t mid = (lo + hi) / 2;
            if (m_recs[mid].key < key) lo = mid + 1;
            else hi = mid;
        }
        for (size_t i = lo; i < m_recs.size() && m_recs[i].key == key; ++i) {
            if (m_recs[i].subset == subset) return &m_recs[i];
        }
        return nullptr;
    }

    // Address-only: is anything here at all? The scheduling question, same caveat as
    // TileIndex::HasAny -- a hit may be stale, which costs an ordering decision, never a pixel.
    bool HasAny(uint64_t key) const {
        size_t lo = 0, hi = m_recs.size();
        while (lo < hi) {
            const size_t mid = (lo + hi) / 2;
            if (m_recs[mid].key < key) lo = mid + 1;
            else hi = mid;
        }
        return lo < m_recs.size() && m_recs[lo].key == key;
    }

    bool Valid() const { return !m_recs.empty(); }
    size_t Count() const { return m_recs.size(); }
    const std::wstring& WPath() const { return m_wpath; }
    const std::string& Path() const { return m_path; }

private:
    std::string m_path;
    std::wstring m_wpath;
    std::vector<Rec> m_recs;
};

}  // namespace ga
