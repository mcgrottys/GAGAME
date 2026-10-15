// ================================================================================================
//  CacheIndex - WHAT THE HARVEST HOLDS OFFLINE, AS A MAP THE GPU CAN READ (docs/FIDELITY.md 7, the
//  cache lens on the minimap).
//
//  The harvest's storage index (`harvest storage`, harvest/Gagame.Harvest/Core/StorageIndex.cs,
//  magic GASTOR04) records the bytes and tiles stored per source / parameter / store / lattice
//  CELL, read off the disk and never estimated. Its cells are the engine's own cube lattice
//  (core/Lattice.h CubeFaceOfDir: face, then u along x and v along y, the D3D cube face's uv), keyed
//  face << 61 | level << 56 | y << 28 | x, level L cutting a face 2^L cells a side, 12 the deepest
//  kept. This reduces every entry to ONE grid of that lattice, level 10 (1024 cells a face side,
//  about 9.8 km at a face's centre), the six faces stacked into one 1024 x 6144 R8G8 image whose
//  row is face * 1024 + y -- so the shader addresses it by the lattice's own law (HpCubeFace) with
//  one Load, and no hardware cube filter ever mixes two cells.
//
//  ONE LAW FOR A CELL OF ANY LEVEL: an entry of level L covers the level-10 texels
//  [x 2^(10-L), (x + 1) 2^(10-L)) on each axis -- a finer entry's span rounds to its one ancestor
//  texel (x >> (L - 10)), a coarser one's to the 4^(10-L) texels under it. Per texel:
//      R = the finest level any entry over it holds, times 20 (12 -> 240);
//      G = log2(1 + the bytes stored per texel) over 0..40, scaled to 0..255 -- an entry's bytes are
//          spread evenly over the texels it covers, so a planet-wide base adds its share and no more.
//  A texel nothing covers is 0, 0; a level-0 entry is R = 0 with G > 0, so "nothing" is G = 0.
// ================================================================================================
#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace ga {

struct CacheIndex {
    static constexpr uint32_t kLevel = 10;
    static constexpr uint32_t kSide = 1u << kLevel;   // texels a face side

    std::vector<uint8_t> rg;   // kSide x (6 kSide) texels, two bytes each (R8G8_UNORM rows)
    uint64_t folders = 0, entries = 0, texelsLit = 0;
    double ms = 0.0;

    // %LOCALAPPDATA%\GAGAME\storage-index.bin, where StorageIndex.DefaultPath writes it.
    static std::wstring DefaultPath();
    // Reads the index and reduces it. False (and rg empty) when the file is missing or is not a
    // GASTOR04 index; `why` says which.
    bool Read(const std::wstring& path, std::string& why);
};

}  // namespace ga
