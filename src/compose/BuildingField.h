// ================================================================================================
//  BuildingField - THE BUILDINGS UNDER A PIXEL, AS SCALARS (docs/BUILDING_LOD.md, step 1 of the
//  scalar plan, 2026-10-10).
//
//  A building wider than a pixel is drawn as itself (scene/BuildingLayer). One narrower is no
//  geometry any more: it is a share of its pixel's colour, which is what a texel already is. So
//  below a texel each building is three numbers a texel adds up, all ADDITIVE (so any level is the
//  sum of what lies in it, and the tree's tiles at every grain mean the same thing):
//
//      c   the plan area index: footprint area / texel area           (urban morphometry's lambda_p)
//      f/2 the frontal area index of those narrower than HALF the texel
//      f   the frontal area index: sum of height x mean projected width / texel area
//                                                                     (lambda_f, Grimmond & Oke 1999)
//
//  A tile of ground texel t holds exactly the buildings narrower than t (2 rho < t): wider ones
//  are the prisms' (a building is counted once, the pixel's own scale decides which). It is
//  painted at every grain from the folded tree (compose/BuildingLod.h) without reading every
//  building under it: at the finest level L whose quads are no wider than a texel, a node's
//  DESCENDANTS are one fold (their area and volume exact, the fold's moments), its OWN buildings
//  one by one; every coarser level's buildings one by one. A tile finer than the finest quad
//  reads its buildings one by one.
//
//  f is exact for every building splatted alone. A fold carries area and volume but not its
//  buildings' count, so its f is its volume over kFoldWidthM (the stated assumption, 12 m: the
//  typical house); the footprint tree (step 3) will carry the count and retire it.
//
//  THE SIZE LAW. Building sizes run as a power law (urban scaling: Batty), so the frontal index of the
//  buildings under a cutoff s goes as f(<s) = f(<t) (s/t)^beta; the two cutoffs give beta per texel,
//  beta = log2(f / f/2). A texel read coarser than its pixel p (a level not yet resident) is then
//  cut to f (p/t)^beta: the buildings drawn as boxes are not counted twice while finer tiles stream.
//
//  Texel: r = c, g = f/2 / kFrontal, b = f / kFrontal, a = 255 (a tile with no building anywhere in
//  it is void: no tile, read as zero). Linear in every channel, so a texture filter between texels
//  and between mips is the sum's own average.
// ================================================================================================
#pragma once

#include "compose/BuildingLod.h"
#include "compose/Compositor.h"

#include <list>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <tuple>

namespace ga {

class BuildingFieldSource : public ColorSource {
public:
    static constexpr double kFrontal = 4.0;      // b = 1 at lambda_f 4
    static constexpr double kFoldWidthM = 12.0;  // a fold's mean building width (see above)
    static constexpr uint32_t kGridDim = 128;    // the tile's own texels
    // THE FIELD'S FLOOR: no solid of the stack is narrower than this (2 rho, rho the box's
    // circumscribed radius: a footprint of nothing still stands a floor's 3 m), so a texel finer
    // than it holds no building at all -- the field there is zero, never painted, never wanted.
    static constexpr double kMinDiameterM = 3.0;
    // ...as rungs below a window's mip 0 (rung r, mip m: a texel of cubeTexel 2^(m - r)): the field
    // is wanted at m >= r - FloorRungs, the finest mip whose texel is still at least kMinDiameterM.
    static int FloorRungs(double cubeTexelM) {
        int k = 0;
        while (cubeTexelM / double(1 << (k + 1)) >= kMinDiameterM && k < 30) ++k;
        return k;
    }

    // `tree` the folded tree (BuildingLodFile, opened); `id` what names its contents in the cache
    // key (the folder and its manifest's size and time).
    BuildingFieldSource(std::shared_ptr<const BuildingLodFile> tree, const std::string& id);
    const SourceInfo& Info() const override { return m_info; }
    // A tile holds what is narrower than ITS texel: its parent is no mean of it (DomainSource).
    bool LevelsOwn() const override { return true; }
    void BeginTile(double latMin, double latMax, double lonMin, double lonMax, double groundResM,
                   PaintCtx& ctx) override;
    float Sample(double latRad, double lonRad, double groundResM, const PaintCtx& ctx,
                 uint8_t rgba[4]) override;

private:
    struct PageData {
        std::vector<LodNode> nodes;
        std::vector<LodBuilding> blds;
        size_t bytes = 0;
    };
    using PageKey = std::tuple<int, int, int>;
    std::shared_ptr<const PageData> Page(int L, int px, int py) const;

    std::shared_ptr<const BuildingLodFile> m_tree;
    SourceInfo m_info;
    // The pages read, shared by the tiles that need them (the loader paints on many threads).
    mutable std::mutex m_mx;
    mutable std::map<PageKey, std::pair<std::shared_ptr<const PageData>, std::list<PageKey>::iterator>> m_cache;
    mutable std::list<PageKey> m_lru;   // front = newest
    mutable size_t m_bytes = 0;
    static constexpr size_t kCacheBytes = 512ull << 20;
};

}  // namespace ga
