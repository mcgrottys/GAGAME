// ================================================================================================
//  GisMask - M9ak: THE LAND/SEA MASK, AS VECTORS. No raster is stored, read, or shipped.
//
//  `data/gis/landmask_ne.raw` (16 MB) and `landmask_global.raw` (8 MB) are 24 MB of PARITY FILL
//  -- a rasterization the harvester performed once, at one resolution, in one frame. They are a
//  realization of the survey, not the survey. `GisStencil.h` has said so since it was written:
//  "the GSHHG shorelines and WDBII rivers load as POLYLINES (EPSG:4326) and STAY RESIDENT AS THE
//  AUTHORITY", and the vector consumer is "crisp at every zoom, no raster ceiling".
//
//  So the compositor asks the RINGS, and neither .raw file is opened. What is on disk stays
//  vector; what gets rasterized is the composed tile, which is a tile like every other tile here.
//
//  THE RULE, and it is already written down -- `data/gis/survey.json` records how the raster was
//  built and therefore what the vector composition means:
//
//      base   GSHHG full-res coast rings      inside = LAND
//      carve  NHD open-water polygons         inside = WATER   (overrides the coast)
//      law    edits.geojson polygons          inside = its own declared value (overrides both)
//
//  with marsh and wetland deliberately EXCLUDED from the carve, because "the live tide owns that
//  call" -- the same division of labour the user stated for this gate: the survey says where
//  water CAN be; the bed classifier's height band says where the waterline actually falls.
//
//  WHY COLUMNS, AND WHERE THE ALGEBRA IS REAL.
//
//  A point-in-polygon test on a sphere is a crossing count along some arc from the query point.
//  The obvious raster order -- fill by ROWS of constant latitude -- casts the ray along a
//  PARALLEL, and a parallel is not a great circle, so every crossing would be an approximation
//  in the lon/lat chart, wrong by more the longer the edge and the higher the latitude.
//
//  Fill by COLUMNS and the ray is a MERIDIAN, which IS a great circle. Then the test is
//  incidence algebra on the sphere, and it is exact:
//
//      an edge A->B spans the great circle  n = A ^ B          (the join of two points)
//      the meridian at longitude L is       m = (-sin L, cos L, 0)
//      the edge crosses that meridian iff   sign(m . A) != sign(m . B)
//      and the crossing point is            d = n x m          (the MEET of the two circles)
//
//  Four dot products, one cross product, one asin per crossing -- no chart, no dateline special
//  case, no cos(lat) fudge. Parity then runs down the column in latitude. This is the "spherical
//  polygons are chains of great arcs; incidence and clipping are meets and joins" note that has
//  been sitting in GisStencil.h as a promissory note.
//
//  THE INDEX IS BY LONGITUDE, AND ONLY BY LONGITUDE. Parity along a meridian depends on every
//  edge that crosses it, however far south -- so latitude cannot prune, and must not. Bucketing
//  the 1.26 M edges by longitude turns a column from "walk every ring that overlaps" (the big
//  coast ring alone is 32727 points, walked 512 times a tile) into a few hundred edge tests.
// ================================================================================================
#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "compose/Compositor.h"
#include "core/Common.h"

namespace ga {

class GisVectorMask {
public:
    // Loads the rings themselves -- coast, NHD open water, and the hand edits. Returns false if
    // there is no coast ring set to be authoritative about, in which case the gate simply never
    // exists and nothing downstream changes.
    bool Load(const std::string& dir);
    bool Ready() const { return !m_coast.empty(); }

    // The union of the loaded rings' own bounding boxes, in degrees -- the source's declared
    // footprint, which is what keeps the gate OUT of the cache identity of every tile it cannot
    // affect. The global cube keeps the tiles it already has.
    void Bounds(double& lon0, double& lat0, double& lon1, double& lat1) const {
        lon0 = m_lon0;
        lat0 = m_lat0;
        lon1 = m_lon1;
        lat1 = m_lat1;
    }

    // Rasterize the WATER GATE over a lat/lon box (degrees) into a dim x dim grid of bytes:
    // 255 = the bed may paint here (water, or outside the survey: no opinion),
    //   0 = the survey says land.
    // Row 0 is latMax. Column-major internally, because a column is a meridian.
    // M9ay: out is dim*dim*2 bytes: [value (255 water / 0 land), flags (1 surveyed, 2 edited)].
    void RasterizeGate(double latMin, double latMax, double lonMin, double lonMax, uint32_t dim,
                       std::vector<uint8_t>& out) const;

    size_t RingCount() const { return m_coast.size() + m_water.size() + m_edits.size(); }
    size_t PointCount() const { return m_pts.size(); }
    // An identity for the LOADED SURVEY -- ring counts, point count, bounds. It rides in the
    // source's `structure`, so re-harvesting the GIS changes the mask's tree identity and every
    // composite that consumed it, and changes nothing else.
    const std::string& Fingerprint() const { return m_fingerprint; }

private:
    struct Ring {
        uint32_t first = 0, count = 0;
        float lon0 = 0, lat0 = 0, lon1 = 0, lat1 = 0;   // degrees
        uint8_t waterValue = 0;   // edits only: what "inside" asserts (0 land, 255 water)
    };
    struct Vec3 {
        double x, y, z;
    };
    // Edges of one ring set, bucketed by longitude (CSR). Coast and water get one each; the
    // hand edits do not, because each edit ring asserts its OWN value and so must keep its own
    // parity -- and there are a few dozen of them, not a million.
    struct EdgeIndex {
        std::vector<uint32_t> a, b;      // indices into m_pts
        std::vector<uint32_t> start;     // bucket -> first entry in `edge` (size buckets+1)
        std::vector<uint32_t> edge;      // entries: indices into a/b
    };

    // stitchClip: the file is a coastline CLIPPED to a box (open pieces ending on its edges);
    // close them along the boundary into land polygons before they become rings (M9av).
    bool ReadRings(const std::string& path, std::vector<Ring>& out, bool stitchClip = false);
    void LoadEdits(const std::string& path);
    void BuildIndex(const std::vector<Ring>& rings, EdgeIndex& idx);
    int Bucket(double lonDeg) const;
    // Crossing latitudes of one edge set with the meridian at `lonDeg`, appended to `xs`.
    void Crossings(const EdgeIndex& idx, double lonDeg, std::vector<double>& xs) const;
    void CrossingsRing(const Ring& r, double lonDeg, std::vector<double>& xs) const;
    // Parity-fill a column from the sorted crossing latitudes.
    static void FillParity(std::vector<double>& xs, double latMin, double latMax, uint32_t dim,
                           std::vector<uint8_t>& col, uint8_t inside);

    std::vector<Vec3> m_pts;
    std::vector<Ring> m_coast, m_water, m_edits;
    EdgeIndex m_coastIdx, m_waterIdx;
    double m_lon0 = 180, m_lat0 = 90, m_lon1 = -180, m_lat1 = -90;
    uint32_t m_buckets = 0;
    std::string m_fingerprint;
};

// ================================================================================================
//  GisMaskSource -- the mask wearing the ColorSource interface, so it is a LAYER: it earns a
//  cache identity, it gets its own sparse tree on disk, and the compositor can gate another
//  layer with it. It paints nothing (`gateOnly`); its weight IS the gate factor.
//
//  BeginTile is what makes this affordable. Answering per texel would be a point-in-polygon
//  against 1.26 M edges per pixel; instead the rings are swept ONCE per tile, by meridian, into
//  the per-tile scratch the realization already owns, and Sample is a lookup.
// ================================================================================================
class GisMaskSource : public ColorSource {
public:
    explicit GisMaskSource(const GisVectorMask* mask) : m_mask(mask) { Refresh(); }
    // A source's FOOTPRINT IS ITS ADMISSION TICKET, and this one's is not known until the rings
    // are loaded. Constructed before Load(), it declared the empty box GisVectorMask starts with
    // (lon 180..-180), SourceTouches rejected it against every tile, and the gate silently never
    // entered a single subset -- built, wired, logged, and never once asked a question. Call
    // this after Load; the zero-file tree directory is what caught it.
    void Refresh();
    const SourceInfo& Info() const override { return m_info; }
    void BeginTile(double latMin, double latMax, double lonMin, double lonMax, double groundResM,
                   PaintCtx& ctx) override;
    float Sample(double latRad, double lonRad, double groundResM, const PaintCtx& ctx,
                 uint8_t rgba[4]) override;

private:
    // 2x the 128-texel tile it gates, and no more. THE GRIDS LINE UP: this mask is a layer of
    // the same channel, so its tree lands on the same (face, mip, x, y) addresses as the imagery
    // and the bed, and the sweep happens ONCE PER ADDRESS EVER -- after that the gate is a 64 KB
    // read and combining it with the bed is a per-texel byte multiply with no resampling at all.
    // Oversampling past the tile's own texels buys nothing, because Sample only ever reads the
    // cells under texel centres; the 2x is for the latitude axis, where a Mercator tile's rows
    // are not quite uniform in latitude. Sampled NEAREST -- a gate that blurs is not a gate.
    static constexpr uint32_t kGridDim = 256;
    const GisVectorMask* m_mask = nullptr;
    SourceInfo m_info;
};

}  // namespace ga
