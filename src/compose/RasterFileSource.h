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
// ================================================================================================
#pragma once

#include "compose/Compositor.h"
#include "compose/Projections.h"
#include "core/FieldLoader.h"

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
};

class RasterFileSource : public ColorSource {
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
    int OwnMip(const Lattice& lattice) const override;
    // THE PLACEMENT of the first file from a georeference; public so a test can plant a wrong one
    // through the same door.
    bool Place(const GeoRef& ref, std::string* why);
    const GeoRef& Ref() const { return m_files[0]->ref; }
    double GrainM() const { return m_grainM; }   // ground sample distance: the finer axis, metres
    double Over() const { return m_over; }
    size_t Files() const { return m_files.size(); }

private:
    struct File {
        std::unique_ptr<FieldLoader> loader;
        GeoRef ref;
        uint32_t w = 0, h = 0;
        double x0 = 0, x1 = 0, yLo = 0, yHi = 0, kx = 0, ky = 0;   // edges; texels per CRS unit
        bool down = true;                                          // row 0 at yHi
    };
    bool Placed(std::string* why);
    bool ToCrs(double latRad, double lonRad, double& x, double& y) const;
    bool ToLatLon(double x, double y, double& latRad, double& lonRad) const;
    std::shared_ptr<const std::vector<uint8_t>> Window(size_t file, uint32_t wx, uint32_t wy);
    const uint8_t* Texel(size_t file, int x, int y);
    double Bilinear(size_t file, double fx, double fy, uint8_t rgba[4]);
    double BoxMean(size_t file, double uc, double vc, double su, double sv, uint8_t rgba[4]);

    std::vector<std::unique_ptr<File>> m_files;
    CrsKind m_kind = CrsKind::Unknown;
    TransverseMercator m_tm{};
    double m_ux0 = 0, m_ux1 = 0, m_uy0 = 0, m_uy1 = 0;   // the whole's box, CRS units
    double m_mx = 1, m_my = 1;                           // metres per CRS unit at the centre
    double m_gx = 0, m_gy = 0, m_grainM = 0.0, m_over = 0.0, m_feather = 0.0;
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

// THE DEFAULT ORDER of a stack of photos, bottom to top: `over` ascending (a RasterFileSource's
// own, every other source's 0), then the coarser grain under the finer (SourceInfo::cmPerPixel).
// Stable: equal keys keep the order they came in.
void StackOrder(std::vector<ColorSource*>& stack);

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
// Every source's pass on every lattice, logged: what the boot runs before the first frame and
// the `ingest` tool runs alone.
void IngestSources(const std::vector<std::unique_ptr<RasterFileSource>>& sources,
                   const std::vector<Lattice>& lattices);

// --selftest's block (compose/RasterFileTest.cpp).
bool RunRasterFileSelfTest();

}  // namespace ga
