// ================================================================================================
//  RasterFileSource - A RASTER IS A SOURCE BY BEING A FILE. What the scene's `sources` names -- a
//  file (a GeoTIFF, or a PNG / JPEG beside a world file), a folder and a pattern, or a manifest of
//  raw rows -- opened through the loader seam (core/ImageLoader.h), is ONE leaf of the colour
//  tree: no C++ per dataset.
//
//  WHICH FORM. A ColorSource (Compositor.h), not a DomainSource: the colour tree's leaf machinery
//  already runs on it -- ColorLayerSource gives it the tree's identity (name|structure),
//  footprint and unit, and the per-source tree paints it (PaintSourceTile).
//
//  A SOURCE PAINTS ITS OWN LEVEL, AND THE TREE MAKES THE OTHERS (HIERARCHY 4.20). On a lattice its
//  own mip (OwnMip) is the coarsest whose texel on the ground, measured at the source's centre with
//  the lattice's own texel (Lattice::TexelGround), is no coarser than its grain on either axis;
//  0 where even mip 0 is coarser. The tree folds every level above that and keeps the folds
//  (TileTree::LeafTile). So the source is asked only at its own grain or finer, and it keeps no
//  levels: it reads its files by the WINDOW, 256 x 256 texels decoded once and kept while used
//  (a few hundred, least recently used), and holds no copy of a file.
//
//  WHAT IT ANSWERS. The exchange frame (WGS84 lat/lon) resolved EXACTLY into the files' CRS
//  (Projections.h), then a file's affine: no row is flipped by hand, the sign of the scale is the
//  flip (priors 10), and a texel's sample stands at its centre (priors 7). The value at a texel is
//  the mean of the file over a box of the ground asked for (groundResM x cos lat, the metres every
//  source here reads it as) and never less than one of the file's texels, in the file's own axes.
//  A box of one texel IS the bilinear read, taps clamped at the file's edge as the plane orthos'
//  source did (its arithmetic, so their tiles are byte for byte what it painted); a larger one is
//  the area mean, each texel weighted by the part of it inside the box. The weight is the file's
//  alpha and nothing else (colour premultiplied by it, so a transparent texel never drags a
//  colour toward black), times the feather.
//
//  A SET. A folder, or a manifest, is one source of N files of one CRS and one grain (a mix is
//  refused by name): a point is read from the file that holds it, seamless inside, and the
//  entry's `feather` (metres, 0 = a hard edge) is taken against the bounding box of the whole.
//
//  ITS IDENTITY is the loaders' Structure(): FNV-1a 64 of every byte of each file and its
//  sidecars (core/ImageLoader.cpp), and the feather. A file replaced by one of the same size is a
//  new tree.
//
//  A HEIGHT (slice 3) is the same class and every line above: a file of one channel is a
//  HeightSource of the height stack, its window's four bytes a texel a float, NaN where the file
//  said nodata. Its value is declared, never guessed (HIERARCHY 4.11): the unit and the datum the
//  file names, else the entry's; a datum not the engine's (kHeightFrame) is taken only with the
//  entry's `offset`, the separation at the place, added here once -- else refused by name. The
//  read is the colour's area mean in one channel, a texel weighing 1 where the file has a value:
//  a box of one texel is the bilinear, taps clamped at the file's edge, and the weight is the part
//  of the box the file has. The tree's format for it is FloatW; the identity adds the conversion.
// ================================================================================================
#pragma once

#include "compose/Compositor.h"
#include "compose/Projections.h"
#include "core/FieldLoader.h"

#include <algorithm>
#include <list>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

namespace ga {

class TileTree;

// One entry of the scene's `sources` (app/Scene.h): a file, a folder and a pattern, or a manifest.
struct RasterEntry {
    std::string file, folder, match, manifest, name, kind, crs;
    double over = 0.0;      // the stack order's first key (StackOrder)
    double feather = 0.0;   // metres, against the bounding box of the whole; 0 = a hard edge
    std::string unit, datum;   // a height's, read only where the file names none
    double offset = 0.0;       // metres added to a height off kHeightFrame's datum: declared, or
    bool hasOffset = false;    // such a file is refused
};

class RasterFileSource : public ColorSource, public HeightSource {
public:
    RasterFileSource();
    // One file through the registry; false, and why, if it is refused.
    bool Load(const LoaderRegistry& reg, const std::string& path, const RasterEntry& e,
              std::string* why);
    // The entry's files as ONE source; `label` names it where the entry does not.
    bool Open(std::vector<std::unique_ptr<FieldLoader>> files, const RasterEntry& e,
              const std::string& label, std::string* why);
    const SourceInfo& Info() const override { return m_info; }
    float Sample(double latRad, double lonRad, double groundResM, const PaintCtx& ctx,
                 uint8_t rgba[4]) override;
    float Sample(double latRad, double lonRad, double groundResM, float& metres) override;
    int OwnMip(const Lattice& lattice) const override;
    bool Footprint(double& lon0, double& lat0, double& lon1, double& lat1) const override;
    // THE PLACEMENT of the first file from a georeference -- and a height's unit and datum from
    // its valueUnit; public so a test can plant a wrong one through the same door.
    bool Place(const GeoRef& ref, std::string* why);
    const GeoRef& Ref() const { return m_files[0]->ref; }
    double GrainM() const { return m_grainM; }   // ground sample distance: the finer axis, metres
    double Over() const { return m_over; }
    size_t Files() const { return m_files.size(); }
    bool Height() const { return m_height; }

private:
    struct File {
        std::unique_ptr<FieldLoader> loader;
        GeoRef ref;
        uint32_t w = 0, h = 0;
        double x0 = 0, x1 = 0, yLo = 0, yHi = 0, kx = 0, ky = 0;   // edges; texels per CRS unit
        bool down = true;                                          // row 0 at yHi
    };
    bool Placed(std::string* why);
    bool Valued(std::string* why);
    bool ToCrs(double latRad, double lonRad, double& x, double& y) const;
    bool ToLatLon(double x, double y, double& latRad, double& lonRad) const;
    std::shared_ptr<const std::vector<uint8_t>> Window(size_t file, uint32_t wx, uint32_t wy);
    const uint8_t* Texel(size_t file, int x, int y);
    double Bilinear(size_t file, double fx, double fy, uint8_t rgba[4]);
    double BoxMean(size_t file, double uc, double vc, double su, double sv, uint8_t rgba[4]);
    float Feathered(double x, double y) const;

    std::vector<std::unique_ptr<File>> m_files;
    CrsKind m_kind = CrsKind::Unknown;
    TransverseMercator m_tm{};
    double m_ux0 = 0, m_ux1 = 0, m_uy0 = 0, m_uy1 = 0;   // the whole's box, CRS units
    double m_mx = 1, m_my = 1;                           // metres per CRS unit at the centre
    double m_gx = 0, m_gy = 0, m_grainM = 0.0, m_over = 0.0, m_feather = 0.0;
    bool m_height = false;
    double m_vScale = 1.0, m_vOffset = 0.0;   // a height: metres = the file's number x scale + offset
    std::string m_valued;                     // ...and in words, for the identity
    RasterEntry m_entry;
    SourceInfo m_info;
    const uint64_t m_serial;   // names this source's windows in the painting threads' own few
    std::mutex m_cacheMx;
    std::list<std::pair<uint64_t, std::shared_ptr<const std::vector<uint8_t>>>> m_lru;
    std::unordered_map<uint64_t, decltype(m_lru)::iterator> m_index;
};

// Every entry, opened: one source each. A refused entry is logged by name with its reason, and
// the run goes on without it; a folder's refused file is logged and the set goes on without it.
std::vector<std::unique_ptr<RasterFileSource>> LoadRasterSources(
    const std::vector<RasterEntry>& entries);

// THE DEFAULT ORDER of a stack of photos or of heights, bottom to top: `over` ascending (a
// RasterFileSource's own, every other source's 0), then the coarser grain under the finer
// (SourceInfo::cmPerPixel). Stable: equal keys keep the order they came in.
template <class Source>
void StackOrder(std::vector<Source*>& stack) {
    auto over = [](const Source* s) {
        const auto* r = dynamic_cast<const RasterFileSource*>(s);
        return r ? r->Over() : 0.0;
    };
    std::stable_sort(stack.begin(), stack.end(), [&](const Source* a, const Source* b) {
        if (over(a) != over(b)) return over(a) < over(b);
        return a->Info().cmPerPixel > b->Info().cmPerPixel;   // the coarser under the finer
    });
}

// ---- THE PASS (HIERARCHY 4.20): one source's leaf tree on one lattice. Every tile of its own mip
// that the footprint touches (the compositor's rule, Compositor::Touches) is painted, then each
// level above is folded from the one below, up to the last level the footprint touches; each
// level's tiles on the engine's pool. Then `<tag>\.whole` says the tree is whole for this
// identity, and a second pass is one look at it.
struct IngestStats {
    int own = -1;
    uint32_t tiles = 0, levels = 0;
    double seconds = 0.0;
    bool whole = false;   // the marker was there: nothing was asked
};
IngestStats Ingest(TileTree& tree, const Lattice& lattice);
// Every source's pass on every lattice of its kind, logged: what the boot runs before the first
// frame and the `ingest` tool runs alone. A height's tree is the height stack's leaf (FloatW).
void IngestSources(const std::vector<std::unique_ptr<RasterFileSource>>& sources,
                   const std::vector<Lattice>& colour, const std::vector<Lattice>& height);

// --selftest's block (compose/RasterFileTest.cpp); `--tool rastertest:real`, the real files of
// slice 3's part C against the harvester's grids, on the CPU.
bool RunRasterFileSelfTest();
bool RunRealHeightsCheck();

}  // namespace ga
