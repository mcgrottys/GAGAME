// ================================================================================================
//  RasterFileSource - A RASTER IS A SOURCE BY BEING A FILE. One file the scene's `sources` names
//  (a GeoTIFF, or a PNG / JPEG beside a world file), opened through the loader seam
//  (core/ImageLoader.h), is one more leaf of the colour tree -- no C++ per dataset.
//
//  WHICH FORM. A ColorSource (Compositor.h), the aerial orthos' form, not a DomainSource: the
//  colour tree's leaf machinery already runs on it -- ColorLayerSource gives it the tree's
//  identity (name|structure), footprint and unit, the flat incumbent channel the audit compares
//  against takes only ColorSource*, and the per-source tree paints it (PaintSourceTile). A
//  DomainSource would have restated all four for one leaf.
//
//  WHAT IT ANSWERS. The exchange frame (WGS84 lat/lon) resolved EXACTLY into the file's own CRS
//  (Projections.h), then the file's affine: no row is flipped by hand, the sign of the scale is
//  the flip (priors 10), and a texel's sample stands at its centre (priors 7). It answers at the
//  grain it is asked for from a mip chain of itself (as the aerial source does), each level a 2x2
//  box of the one above, alpha-weighted (a GeoTIFF's own overviews are not visible through WIC).
//  Bilinear, premultiplied: the weight is the file's alpha (0 where it said nodata), so a
//  transparent texel lets the layer beneath show and never drags a colour toward black.
//
//  ITS IDENTITY is the loader's Structure(): FNV-1a 64 of every byte of the file and its sidecars
//  (core/ImageLoader.cpp). A file replaced by one of the same size is a new tree.
// ================================================================================================
#pragma once

#include "compose/Compositor.h"
#include "compose/Projections.h"
#include "core/FieldLoader.h"

#include <memory>
#include <string>
#include <vector>

namespace ga {

// One entry of the scene's `sources` (app/Scene.h): a file, or a folder and a pattern.
struct RasterEntry {
    std::string file, folder, match, name, kind, crs;
    double over = 0.0;   // the stack order's first key (StackOrder)
};

class RasterFileSource : public ColorSource {
public:
    // Opens `path` through the registry; false, and why, if the file is refused.
    bool Load(const LoaderRegistry& reg, const std::string& path, const RasterEntry& e,
              std::string* why);
    const SourceInfo& Info() const override { return m_info; }
    float Sample(double latRad, double lonRad, double groundResM, const PaintCtx& ctx,
                 uint8_t rgba[4]) override;
    // THE PLACEMENT: the projection and the footprint from a georeference. Load calls it with the
    // file's; it is public so a test can plant a wrong one through the same door.
    bool Place(const GeoRef& ref, std::string* why);
    const GeoRef& Ref() const { return m_ref; }
    double GrainM() const { return m_grainM; }   // ground sample distance: the finer axis, metres
    double Over() const { return m_over; }

private:
    struct Level {
        uint32_t w = 0, h = 0;
        std::vector<uint8_t> rgba;
    };
    bool ToCrs(double latRad, double lonRad, double& x, double& y) const;
    bool ToLatLon(double x, double y, double& latRad, double& lonRad) const;
    std::vector<Level> m_levels;
    GeoRef m_ref;
    TransverseMercator m_tm{};
    double m_grainM = 0.0, m_over = 0.0;
    SourceInfo m_info;
};

// Every file the entries name, opened. A refused file is logged by name with its reason, and the
// run goes on without it.
std::vector<std::unique_ptr<RasterFileSource>> LoadRasterSources(
    const std::vector<RasterEntry>& entries);

// THE DEFAULT ORDER of a stack of photos, bottom to top: `over` ascending (a RasterFileSource's
// own, every other source's 0), then the coarser grain under the finer (SourceInfo::cmPerPixel).
// Stable: equal keys keep the order they came in.
void StackOrder(std::vector<ColorSource*>& stack);

// --selftest's block (compose/RasterFileTest.cpp).
bool RunRasterFileSelfTest();

}  // namespace ga
