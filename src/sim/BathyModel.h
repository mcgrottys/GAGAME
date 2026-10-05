// ================================================================================================
//  BathyModel - M5: a CUDEM topobathy survey on its own lon/lat lattice (row 0 north), NAVD88.
//
//  PHASE C5: the survey knows no world frame. The frozen anchor and its two metres-per-degree
//  (the old world.flat) are DELETED; a reader asks at a place (SampleLatLon), and the scene's
//  place.anchor chart (core/Space.h) turns a flat point into its place where it needs one.
// ================================================================================================
#pragma once

#include <string>
#include <vector>

namespace ga {

class Compositor;
class CudemHeightSource;

class BathyModel {
public:

    bool Load(const std::string& jsonPath);
    bool Ready() const { return m_nx > 0; }

    // M6r: the survey law reaches the physics. mask=land polygons from edits.geojson (the
    // OSM-seeded jetty footprints) rasterize into this grid as riprap walls: the 3 m CUDEM
    // knows the structures but 13.7 m box means smear them into leaky sills, and the solver's
    // flood then crosses the crest line instead of concentrating through the gap. CPU-side
    // masking only, per the vector doctrine. Returns cells raised.
    // (M6w: superseded on the main path by RealizeFromChannel -- edits are a STACK SOURCE
    // now, so the walls arrive through the channel; this stays for tools/fallback.)
    int ApplyMaskEdits(const std::string& geojsonPath, float crestNavd);

    // M6w: THE ONE BED. Re-fill this grid (same georef) from the composed height channel --
    // CUDEM where it covers, NE-15s/ETOPO beyond, hand-edit structures on top. The solver's
    // bed, the renderer's tiles, and every physics product become the same stack sampled at
    // rungs; a bathymetry disagreement between them is no longer expressible.
    bool RealizeFromChannel(const Compositor& comp, int heightChannel);

    // THE SOLVER'S WINDOW (HIERARCHY 4.17: a solver's OPEN BOUNDARY stands where its sources
    // paint at full weight). This grid becomes a copy of `survey` -- the lattice `source` paints
    // from -- drawn in on the side of an open face that reads the bed (the west, `westOpen`) to
    // the cells where `source` paints at full weight (CudemHeightSource::FullWeightCells); the
    // other sides keep the survey's extent. The whole of `survey` when `drawIn` is false, when
    // no side is open, or when the source cannot say or stands on another lattice. Whole cells,
    // so every cell keeps its place on the survey's lattice. `name` labels the log line. True
    // when drawn in.
    bool DrawFrom(const BathyModel& survey, const CudemHeightSource* source, bool drawIn,
                  bool westOpen, const char* name);

    int Nx() const { return m_nx; }
    int Ny() const { return m_ny; }
    const std::vector<float>& Elev() const { return m_elev; }   // NAVD88 m; -9999 = nodata

    // M9n: the grid's own lat/lon lattice, so another path can sample the SAME points
    // RealizeFromChannel does. Row 0 is north, hence Lat1() (the north edge) and a positive
    // Dlat() that walks southward.
    double Lon0() const { return m_lon0; }
    double Lat1() const { return m_lat1; }
    double Dlon() const { return m_dlon; }
    double Dlat() const { return m_dlat; }



    double Lat0() const { return m_lat1 - m_ny * m_dlat; }   // the south edge
    double Lon1() const { return m_lon0 + m_nx * m_dlon; }   // the east edge
    // Bilinear sample at a place (degrees); -9999 outside the grid or over nodata.
    float SampleLatLon(double latDeg, double lonDeg) const;

private:
    int m_nx = 0, m_ny = 0;
    double m_lon0 = 0, m_lat1 = 0, m_dlon = 0, m_dlat = 0;   // row 0 = north
    std::vector<float> m_elev;
};

}  // namespace ga
