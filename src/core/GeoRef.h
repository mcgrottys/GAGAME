// ================================================================================================
//  GeoRef - M9h: THE GEOREFERENCE AS A CHECKED TYPE, and the ingest rule that makes "lossless"
//  and "sparse" the same thing rather than opposites.
//
//  The user's requirement: a loader must narrow down whatever projection and unit information
//  it can actually find, and there are exactly three ways it can come:
//
//    EMBEDDED    the file carries it     GeoTIFF ModelPixelScale + ModelTiepoint + GeoKeys,
//                                        NetCDF CF grid_mapping, GRIB2 GDS
//    CONVENTION  the protocol fixes it   WMTS / Google tiles ARE EPSG:3857 on a fixed pyramid;
//                                        nothing inside a cached tile says so
//    DECLARED    the operator states it  a bare image plus explicit projection and units
//
//  Today SourceInfo::crs is free text ("EPSG:3857 web-mercator"). It documents the contract but
//  cannot be checked, and this project has been bitten precisely there:
//    * priors 10 -- the latlon->uv flip "obviously" needed inverting; the inversion was already
//      inside the mercator formula. The validator's first catch was its own author's
//      declaration. So the flip must be DERIVED from the georeference, never typed by hand.
//    * priors 2 / 7 -- padded vs logical dims, and corner vs centre sampling: a half-lattice
//      translation the flip rule cannot see.
//  Both become fields here, so the GA AST edge can be registered FROM the georeference instead
//  of being written out again at every call site.
//
//  ---- THE INGEST RULE ------------------------------------------------------------------------
//  NODATA IS NOT A VALUE. It is absence, and absence is a NULL TILE.
//
//  That single rule is what reconciles "lossless" with "sparse". A dense bathymetry raster is
//  dense -- storing it sparsely saves nothing and loses nothing. But a survey (eHydro, a
//  single-beam track, a cloud-free composite) is mostly nodata, and a tile that is entirely
//  nodata should never be allocated: the Tier-2 read-zero guarantee then MEANS "no survey
//  here", the consumer adds it unconditionally, and no sentinel value (-9999, NaN) ever
//  reaches a shader to be mistaken for terrain. Lossless because no real sample was touched;
//  sparse because absence costs nothing.
//
//  The second way a dense source becomes legitimately sparse is to store it as a DEVIATION from
//  a base field, so zero means "the base is exactly right here". The SWE eta plane already
//  reads that way ("NULL tiles read the hardware zero = the tide plane is exactly right"), and
//  it is the shape the CUDEM bed wants if it is ever to be sparse.
// ================================================================================================
#pragma once

#include <cmath>
#include <cstdint>
#include <string>

namespace ga {

// Where the georeference came from. Kept on the object because it decides how much to TRUST it:
// an embedded CRS is evidence, a convention is an assumption worth asserting, and a declared one
// is an operator's claim that nothing in the data can confirm.
enum class CrsProvenance : uint8_t { Embedded, Convention, Declared };

// The projections this engine resolves exactly (Projections.h holds the formulas). Anything
// else must be reprojected by the harvester before it becomes a GA object -- the engine does not
// guess at a CRS it cannot evaluate.
enum class CrsKind : uint8_t {
    Geographic,          // EPSG:4326 lat/lon degrees
    WebMercator,         // EPSG:3857 -- Google/WMTS pyramids
    TransverseMercator,  // UTM zones, EPSG:6348 (MassGIS orthos) etc.
    Equirect,            // plate carree grids (GFS, RTOFS, ocean colour)
    Unknown,
};

// ================================================================================================
//  GeoRef -- the affine from texel indices to the source's own world coordinates, plus the units
//  and the two sampling conventions that no flip rule can see.
//
//  world = origin + scale * index, evaluated in the CRS's own linear unit. scaleY is NEGATIVE
//  for the usual north-up raster (GeoTIFF writes it that way), which is exactly what makes
//  rowZeroNorth derivable instead of declared.
// ================================================================================================
struct GeoRef {
    CrsProvenance provenance = CrsProvenance::Declared;
    CrsKind kind = CrsKind::Unknown;
    int epsg = 0;

    double originX = 0.0, originY = 0.0;   // world coords of texel (0,0)'s CORNER (GeoTIFF tiepoint)
    double scaleX = 1.0, scaleY = -1.0;    // world units per texel; scaleY < 0 = row 0 is north
    uint32_t width = 0, height = 0;        // LOGICAL dims -- never the padded tile grid (priors 2)

    // Sample positions. GeoTIFF's tiepoint is a CORNER; hardware samplers and every kernel in
    // this engine assume CENTRES. Getting this wrong is a half-texel translation that reads as
    // a subtle misalignment, not as an error (priors 7).
    bool centers = true;

    const char* linearUnit = "m";      // the CRS's unit: "m" or "deg"
    const char* valueUnit = "";        // what a texel MEANS: "m NAVD88", "m/s", "sRGB byte"
    double valueMin = 0.0, valueMax = 0.0;   // declared range -> the AST edge's range check

    // Absence. hasNoData turns a fully-nodata tile into a tile that is never allocated.
    bool hasNoData = false;
    double noData = 0.0;

    // ---- the two questions the AST asks, ANSWERED rather than declared ----------------------
    // Does the second axis grow northward? Derived from the sign of scaleY, so a source cannot
    // disagree with its own affine.
    bool VNorth() const { return scaleY > 0.0; }
    // An edge into a +v=north consumer needs a flip exactly when this source is row-0-north.
    bool NeedsFlipInto(bool dstVNorth) const { return VNorth() != dstVNorth; }

    double MetersPerTexelX() const {
        return (kind == CrsKind::Geographic || kind == CrsKind::Equirect)
                   ? std::abs(scaleX) * 111319.49079327358   // deg -> m at the equator
                   : std::abs(scaleX);
    }

    bool IsNoData(double v) const {
        if (!hasNoData) return false;
        if (std::isnan(noData)) return std::isnan(v);
        return std::abs(v - noData) <= 1e-9 * (1.0 + std::abs(noData));
    }

    // Texel index -> the CRS's own world coordinates (centres or corners per `centers`).
    void TexelToWorld(double col, double row, double& x, double& y) const {
        const double o = centers ? 0.5 : 0.0;
        x = originX + scaleX * (col + o);
        y = originY + scaleY * (row + o);
    }

    // ---- the three provenance classes -------------------------------------------------------

    // EMBEDDED: GeoTIFF's ModelPixelScale (sx, sy, sz) + ModelTiepoint (i, j, k, X, Y, Z).
    // The tiepoint is a CORNER and sy is stored POSITIVE while the row axis runs south, which is
    // why scaleY is negated here -- the one place that sign convention should live.
    static GeoRef FromGeoTiff(int epsgCode, CrsKind k, double tiePixI, double tiePixJ,
                              double tieWorldX, double tieWorldY, double pixScaleX,
                              double pixScaleY, uint32_t w, uint32_t h) {
        GeoRef g;
        g.provenance = CrsProvenance::Embedded;
        g.kind = k;
        g.epsg = epsgCode;
        g.scaleX = pixScaleX;
        g.scaleY = -pixScaleY;                       // GeoTIFF: positive sy, south-running rows
        g.originX = tieWorldX - pixScaleX * tiePixI;
        g.originY = tieWorldY + pixScaleY * tiePixJ;
        g.width = w;
        g.height = h;
        g.centers = false;                            // tiepoint is a corner; callers add the half
        return g;
    }

    // CONVENTION: a Web-Mercator pyramid tile. Nothing inside the cached JPEG says any of this;
    // the protocol does. Asserting it here is what stops a cache from becoming unreadable when
    // the convention is the only record of what the bytes meant.
    static GeoRef WebMercatorTile(int z, int tileX, int tileY, uint32_t tilePx) {
        constexpr double kR = 6378137.0;
        constexpr double kHalf = 3.14159265358979323846 * kR;   // 20037508.342789244
        GeoRef g;
        g.provenance = CrsProvenance::Convention;
        g.kind = CrsKind::WebMercator;
        g.epsg = 3857;
        const double span = (2.0 * kHalf) / double(1u << z);
        g.scaleX = span / double(tilePx);
        g.scaleY = -span / double(tilePx);            // row 0 is the tile's NORTH edge
        g.originX = -kHalf + span * tileX;
        g.originY = kHalf - span * tileY;
        g.width = tilePx;
        g.height = tilePx;
        g.centers = false;
        g.linearUnit = "m";
        return g;
    }

    // DECLARED: a bare raster plus the operator's claim. Kept distinct from Embedded on purpose
    // -- a declared georeference is the one a validator should be most suspicious of, and the
    // provenance field is what lets a report say so.
    static GeoRef Declared(int epsgCode, CrsKind k, double ox, double oy, double sx, double sy,
                           uint32_t w, uint32_t h) {
        GeoRef g;
        g.provenance = CrsProvenance::Declared;
        g.kind = k;
        g.epsg = epsgCode;
        g.originX = ox;
        g.originY = oy;
        g.scaleX = sx;
        g.scaleY = sy;
        g.width = w;
        g.height = h;
        return g;
    }

    // Human-readable, for SourceInfo::crs and the boot report -- the free-text string stops
    // being the source of truth and becomes a rendering OF the truth.
    std::string Describe() const {
        const char* p = provenance == CrsProvenance::Embedded      ? "embedded"
                        : provenance == CrsProvenance::Convention  ? "convention"
                                                                   : "declared";
        return "EPSG:" + std::to_string(epsg) + " (" + p + ", row0 " +
               (VNorth() ? "south" : "north") + ", " + (centers ? "centres" : "corners") + ")";
    }
};

}   // namespace ga
