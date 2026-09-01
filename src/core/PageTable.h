// ================================================================================================
//  PageTable - M9h: where the (level, x, y) address space stops being a convention in the docs
//  and becomes a data structure.
//
//  A reserved ARRAY gives 1024 independently-resident slices in one resource and one descriptor
//  (measured, section 16 of docs/SPARSE_GA.md). A slice is a PAGE. This answers the only
//  question the renderer and the ingest path both need:
//
//      which slice holds level L at page (x, y)?
//
//  THE ALIGNMENT GUARANTEE LIVES HERE. Every GA tree on a body shares this address space, so
//  level L means the same ground resolution on every tree, and page (L, x, y) means the same
//  ground. Two trees asked the same address answer about the same place -- which is what lets
//  the atmosphere, the water and the crust compose without any of them knowing about the
//  others, and what makes a product of two trees well-defined tile by tile. The ladder below is
//  the whole of that guarantee: one level-0 resolution per body, halving every level.
//
//  WHAT THIS IS NOT. It is not a GPU indirection texture. Resolution happens CPU-side, where
//  residency decisions already live, and the slice index is handed to the shader like any other
//  bindless index. A GPU-side page table becomes worthwhile when a single draw must resolve
//  many pages per pixel; until then it would be machinery without a caller.
// ================================================================================================
#pragma once

#include "core/Common.h"

#include <cstdint>
#include <string>
#include <unordered_map>
#include <vector>

namespace ga {

// A page address in the shared space. Levels count DOWN in resolution: level 0 is the finest a
// given tree carries, each level up covers twice the ground per texel. x/y are page indices at
// that level, not texels.
struct PageAddr {
    uint32_t level = 0;
    uint32_t x = 0;
    uint32_t y = 0;

    uint64_t Key() const {
        return (static_cast<uint64_t>(level) << 56) | (static_cast<uint64_t>(x & 0xFFFFFFFu) << 28) |
               static_cast<uint64_t>(y & 0xFFFFFFFu);
    }
    bool operator==(const PageAddr& o) const {
        return level == o.level && x == o.x && y == o.y;
    }
    // The parent covering this page one level coarser -- the walk a lookup takes when a fine
    // page is absent, and the reason the ladder must be a strict halving.
    PageAddr Parent() const { return {level + 1, x >> 1, y >> 1}; }
};

// ================================================================================================
//  The ladder. One per body, shared by every tree on it. This is the "same scaling" requirement
//  in executable form: a tree that computed its own resolution per level could not be composed
//  with another, and nothing would catch the drift until two fields disagreed about where the
//  coast was.
// ================================================================================================
struct LevelLadder {
    double level0MetersPerTexel = 1.0;   // finest level's ground resolution
    uint32_t pageTexels = 16384;         // page extent, one slice's mip 0

    double MetersPerTexel(uint32_t level) const {
        return level0MetersPerTexel * double(1ull << level);
    }
    double PageGroundMeters(uint32_t level) const {
        return MetersPerTexel(level) * double(pageTexels);
    }
    // How many levels to get from the finest to something that spans `meters` in one page --
    // i.e. how deep this ladder has to be to reach a given scale.
    uint32_t LevelsToSpan(double meters) const {
        uint32_t l = 0;
        while (l < 63 && PageGroundMeters(l) < meters) ++l;
        return l;
    }
};

// ================================================================================================
//  PageTable -- address -> slice, with allocation and an honest refusal.
//
//  Eviction follows the pool cap's rule (section 19) rather than inventing a second policy: a
//  page may hold state nothing else can reproduce, so running out REFUSES and says so. A caller
//  that knows its pages are Streamable or Recomputable can Release() a victim itself and retry;
//  the table will not guess which pages are safe to destroy.
// ================================================================================================
class PageTable {
public:
    static constexpr uint32_t kNoSlice = 0xFFFFFFFFu;

    void Init(uint32_t sliceCount, const LevelLadder& ladder, const char* name) {
        m_ladder = ladder;
        m_name = name ? name : "pages";
        m_free.clear();
        m_free.reserve(sliceCount);
        // Hand out low slices first: easier to read in a capture, and keeps the active set
        // dense so a debug view of "slices 0..N" is the whole story.
        for (uint32_t i = sliceCount; i-- > 0;) m_free.push_back(i);
        m_addrOf.assign(sliceCount, PageAddr{});
        m_used.assign(sliceCount, 0);
        m_slices = sliceCount;
        m_map.clear();
        m_refused = 0;
    }

    const LevelLadder& Ladder() const { return m_ladder; }
    uint32_t Slices() const { return m_slices; }
    uint32_t Resident() const { return static_cast<uint32_t>(m_map.size()); }
    uint32_t Refused() const { return m_refused; }

    // Exact lookup. kNoSlice when this page is not resident -- the caller decides whether to
    // reserve it, walk to the parent, or accept the absence.
    uint32_t Find(const PageAddr& a) const {
        auto it = m_map.find(a.Key());
        return (it == m_map.end()) ? kNoSlice : it->second;
    }

    // THE LOOKUP THE RENDERER ACTUALLY WANTS: the finest resident page covering this address,
    // walking up the ladder. Returns kNoSlice only when nothing at any level covers it. This is
    // the CPU half of "no LOD popping" -- a missing fine page silently resolves to a coarser
    // one that describes the same ground, so detail changes and nothing else does.
    uint32_t FindCovering(PageAddr a, uint32_t* outLevel = nullptr) const {
        for (uint32_t guard = 0; guard < 64; ++guard) {
            const uint32_t s = Find(a);
            if (s != kNoSlice) {
                if (outLevel) *outLevel = a.level;
                return s;
            }
            if (a.x == 0 && a.y == 0 && a.level >= 63) break;
            a = a.Parent();
        }
        return kNoSlice;
    }

    // Reserve a slice for this address. Returns the existing slice if already resident.
    uint32_t Reserve(const PageAddr& a) {
        const uint64_t k = a.Key();
        auto it = m_map.find(k);
        if (it != m_map.end()) return it->second;
        if (m_free.empty()) {
            // Same discipline as the pool cap: refuse loudly rather than evict something whose
            // contents may be irreproducible. The caller owns the eviction decision because
            // only the caller knows its residence class.
            ++m_refused;
            if (!m_warned) {
                m_warned = true;
                Log("[pages] %s: OUT OF SLICES (%u in use). Refusing rather than evicting -- a "
                    "page may hold state nothing can rebuild. Release() a victim and retry.",
                    m_name.c_str(), m_slices);
            }
            return kNoSlice;
        }
        const uint32_t s = m_free.back();
        m_free.pop_back();
        m_map[k] = s;
        m_addrOf[s] = a;
        m_used[s] = 1;
        return s;
    }

    void Release(const PageAddr& a) {
        auto it = m_map.find(a.Key());
        if (it == m_map.end()) return;
        const uint32_t s = it->second;
        m_map.erase(it);
        m_used[s] = 0;
        m_free.push_back(s);
    }

    // Which address a slice currently holds -- for debug views, and for the fill path that has
    // a slice in hand and needs to know what ground it is writing.
    const PageAddr& AddrOf(uint32_t slice) const { return m_addrOf[slice]; }
    bool SliceInUse(uint32_t slice) const { return slice < m_slices && m_used[slice] != 0; }

    std::string Stats() const {
        char buf[192];
        snprintf(buf, sizeof(buf),
                 "%s: %u/%u slices, level0 %.3f m/texel, page %u texels (%.1f m at L0), "
                 "refused %u",
                 m_name.c_str(), Resident(), m_slices, m_ladder.level0MetersPerTexel,
                 m_ladder.pageTexels, m_ladder.PageGroundMeters(0), m_refused);
        return buf;
    }

private:
    LevelLadder m_ladder;
    std::string m_name;
    std::unordered_map<uint64_t, uint32_t> m_map;
    std::vector<PageAddr> m_addrOf;
    std::vector<uint8_t> m_used;
    std::vector<uint32_t> m_free;
    uint32_t m_slices = 0;
    uint32_t m_refused = 0;
    bool m_warned = false;
};

}   // namespace ga
