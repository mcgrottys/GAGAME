// ================================================================================================
//  PageTable.h - M9h: where the (level, x, y) address space stops being a convention in the docs
//  and becomes a data structure. PageAddr names a page; LevelLadder says what ground a level
//  covers.
//
//  THE ALIGNMENT GUARANTEE LIVES HERE. Every GA tree on a body shares this address space, so
//  level L means the same ground resolution on every tree, and page (L, x, y) means the same
//  ground. Two trees asked the same address answer about the same place -- which is what lets
//  the atmosphere, the water and the crust compose without any of them knowing about the
//  others, and what makes a product of two trees well-defined tile by tile. The ladder below is
//  the whole of that guarantee: one level-0 resolution per body, halving every level.
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

}   // namespace ga
