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
#include "compose/Projections.h"
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

// M6l: MassGIS 2023 15 cm plane-flown orthos of the inlet -- the compositor's first
// INDEPENDENT high-res layer, and the proof case for the alignment contract: the source
// declares its NATIVE projection (EPSG:6348, NAD83(2011)/UTM 19N, from the GeoJP2 header)
// and Sample() resolves WGS84 lat/lon into it with the EXACT transverse-Mercator forward --
// no linear approximations -- then picks the mip whose metres-per-pixel matches the
// requested paint footprint. Tiles are memory-mapped (a mip chain per 1500 m tile).
class AerialOrthoSource : public ColorSource {
public:
    ~AerialOrthoSource();
    bool Load(const std::string& jsonPath);
    const SourceInfo& Info() const override { return m_info; }
    float Sample(double latRad, double lonRad, double groundResM, const PaintCtx& ctx,
                 uint8_t rgba[4]) override;

private:
    struct MipLevel {
        uint32_t px = 0;
        uint64_t offset = 0;
    };
    struct Tile {
        double e0 = 0, n0 = 0, e1 = 0, n1 = 0;   // UTM19N bounds
        const uint8_t* data = nullptr;           // mapped view of the mip chain
        uint32_t channels = 3;                   // 3 = rgb; 4 = rgba -- ALPHA IS FIBER:
                                                 // per-pixel alpha multiplies the paint
                                                 // weight, so a mostly-transparent overlay
                                                 // (highlights) bleeds through the quadtree
                                                 // pixel by pixel
        std::vector<MipLevel> mips;
    };
    std::vector<Tile> m_tiles;
    std::vector<void*> m_handles;   // files + mappings + views, released in the dtor
    double m_ue0 = 1e18, m_un0 = 1e18, m_ue1 = -1e18, m_un1 = -1e18;
    SourceInfo m_info;
};

// A CUDEM topobathy window through an already-loaded, thalweg-preserving BathyModel grid.
// M6w: any number of focus windows (merrimack, capeann, boston...) stack as separate
// sources -- the HQ static insets over the NE-15s / ETOPO base.
class CudemHeightSource : public HeightSource {
public:
    explicit CudemHeightSource(const BathyModel* bathy, double featherFrac = 0.04,
                               const char* name = "noaa.cudem.merrimack");
    const SourceInfo& Info() const override { return m_info; }
    float Sample(double latRad, double lonRad, double groundResM, float& metres) override;

private:
    const BathyModel* m_bathy;
    double m_feather;
    SourceInfo m_info;
};

// M6w: HAND EDITS AS A STACK SOURCE. The mask=land polygons (the OSM-seeded jetty
// footprints) paint their crest INTO the height channel; mask=water polygons dredge. The
// source's cache identity hashes the geojson CONTENT, so editing the file repaints exactly
// the touched tiles at every rung -- the soak rule working FOR the operator. This retires
// the M6r pre-bake into BathyModel (which changed source content without changing cache
// identity: tiles painted before it silently served the un-walled bed).
class EditsHeightSource : public HeightSource {
public:
    bool Load(const std::string& geojsonPath, float crestNavd = 2.5f);
    const SourceInfo& Info() const override { return m_info; }
    float Sample(double latRad, double lonRad, double groundResM, float& metres) override;

private:
    struct Ring {
        bool land = true;
        double lon0 = 1e9, lat0 = 1e9, lon1 = -1e9, lat1 = -1e9;
        std::vector<std::pair<double, double>> pts;   // lon, lat (degrees)
    };
    std::vector<Ring> m_rings;
    float m_crest = 2.5f;
    SourceInfo m_info;
};

}  // namespace ga
