// ================================================================================================
//  Sources - M6i: the first residents of the schema registry.
//
//  GoogleColorSource      mercator-tile-tree     ~30 cm/px    global      (cache-first HTTP)
//  EquirectHeightSource   equirect-grid int16    varies       global      (ETOPO 2022, MOLA)
//  WindowHeightSource     window-grid int16      ~46 m/px     regional    (ETOPO 15s NE ring)
//  CudemHeightSource      geotiff-window float   ~13.7 m/px   estuary     (NOAA CUDEM, via
//                                                                          BathyModel's grid)
//  Each wraps data the engine ALREADY loads -- no duplicate memory, no second fetch path.
// ================================================================================================
#pragma once

#include "compose/Compositor.h"
#include "sim/BathyModel.h"

#include <cstdint>
#include <vector>

namespace ga {

class GoogleTileProvider;

// Web-Mercator session tiles, resolved per query: the zoom is picked from the requested
// metres-per-texel (capped at z14 -- deeper zooms stay a deliberate, budgeted choice), the
// pixel is a nearest copy so a Mercator-aligned realization degenerates to exact pixel moves.
// BeginTile computes a per-tile GRADE gain (mean color of this zoom vs its parent zoom over
// the same footprint): each level inherits the grading of its ancestors, so resident-mip
// boundaries stop being color-grade seams -- normalization at PAINT time, never at render.
class GoogleColorSource : public ColorSource {
public:
    explicit GoogleColorSource(GoogleTileProvider* prov);
    const SourceInfo& Info() const override { return m_info; }
    void BeginTile(double latMin, double latMax, double lonMin, double lonMax,
                   double groundResM, PaintCtx& ctx) override;
    float Sample(double latRad, double lonRad, double groundResM, const PaintCtx& ctx,
                 uint8_t rgba[4]) override;

private:
    bool Pixel(int z, double latRad, double lonRad, uint8_t rgb[3]);
    GoogleTileProvider* m_prov;
    SourceInfo m_info;
};

// A global equirect int16 grid, row 0 = lat +90, column 0 = lon -180, bilinear, lon-wrapped.
// Serves ETOPO 2022 (Earth) and MEGDR MOLA (Mars) -- one structure, two planets.
class EquirectHeightSource : public HeightSource {
public:
    EquirectHeightSource(const char* name, const char* structure, double cmPerPixel,
                         const std::vector<int16_t>* elev, int nx, int ny);
    const SourceInfo& Info() const override { return m_info; }
    float Sample(double latRad, double lonRad, double groundResM, float& metres) override;

private:
    const std::vector<int16_t>* m_elev;
    int m_nx, m_ny;
    SourceInfo m_info;
};

// A regional equirect int16 window (row 0 = north), edge-feathered so it fades into whatever
// sits beneath it in the stack -- the render-time NeWeight feather, moved to paint time.
class WindowHeightSource : public HeightSource {
public:
    WindowHeightSource(const char* name, const char* structure, double cmPerPixel,
                       const std::vector<int16_t>* elev, int nx, int ny, double lon0,
                       double lat1, double dLon, double dLat, double featherFrac = 0.04);
    const SourceInfo& Info() const override { return m_info; }
    float Sample(double latRad, double lonRad, double groundResM, float& metres) override;

private:
    const std::vector<int16_t>* m_elev;
    int m_nx, m_ny;
    double m_lon0, m_lat1, m_dLon, m_dLat, m_feather;
    SourceInfo m_info;
};

// The CUDEM topobathy window through BathyModel's already-loaded, thalweg-preserving grid.
// The top of the earth.height stack: bathymetry overwrites everything else, softly.
class CudemHeightSource : public HeightSource {
public:
    explicit CudemHeightSource(const BathyModel* bathy, double featherFrac = 0.04);
    const SourceInfo& Info() const override { return m_info; }
    float Sample(double latRad, double lonRad, double groundResM, float& metres) override;

private:
    const BathyModel* m_bathy;
    double m_feather;
    SourceInfo m_info;
};

}  // namespace ga
