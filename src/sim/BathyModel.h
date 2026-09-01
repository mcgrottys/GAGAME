// ================================================================================================
//  BathyModel - M5: the CUDEM topobathy grid, georeferenced into the engine's world frame.
//
//  The world frame is anchored at the ACT0816 Merrimack-entrance current station -- the SAME
//  origin the tidal jet and churn atlas have used since M3, which is what makes the terrain
//  drop into place around the existing physics: x = metres east, z = metres north of
//  (42.81833 N, 70.81 W).
//
//  DATUM NOTE: CUDEM elevations are NAVD88; the tide model speaks MLLW. Near Newburyport MLLW
//  sits ~1.30 m below NAVD88 zero (the fitted MSL-above-MLLW, with NAVD88 ~ local MSL on this
//  coast), so water_NAVD = tide_MLLW - 1.30. Tunable via --datum until the proper CO-OPS NAVD
//  datum fetch lands in M5b.
// ================================================================================================
#pragma once

#include <string>
#include <vector>

namespace ga {

class Compositor;

class BathyModel {
public:
    static constexpr double kOrgLon = -70.81;
    static constexpr double kOrgLat = 42.81833;
    static constexpr double kMPerLon = 81660.0;    // 111320 * cos(42.818 deg)
    static constexpr double kMPerLat = 110574.0;

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

    // World-frame extent of the grid (x east, z north, metres from the origin).
    float WorldX0() const { return m_worldX0; }
    float WorldZ0() const { return m_worldZ0; }
    float WorldSizeX() const { return m_worldSizeX; }
    float WorldSizeZ() const { return m_worldSizeZ; }

    // Bilinear sample at a world position; -9999 outside the grid or over nodata.
    float SampleWorld(float x, float z) const;

private:
    int m_nx = 0, m_ny = 0;
    double m_lon0 = 0, m_lat1 = 0, m_dlon = 0, m_dlat = 0;   // row 0 = north
    std::vector<float> m_elev;
    float m_worldX0 = 0, m_worldZ0 = 0, m_worldSizeX = 1, m_worldSizeZ = 1;
};

}  // namespace ga
