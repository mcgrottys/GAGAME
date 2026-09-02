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
    // datumShiftM: metres ADDED to every sample to express the grid in the stack's datum.
    // ETOPO is MSL/geoid referenced and the rest of the earth stack is NAVD88, so the earth's
    // ETOPO layers carry the published MSL -> NAVD88 link here rather than leaving every
    // consumer to blend two datums as if they were one. Mars passes 0: an areoid has no NAVD88.
    EquirectHeightSource(const char* name, const char* structure, double cmPerPixel,
                         const std::vector<int16_t>* elev, int nx, int ny,
                         double datumShiftM = 0.0);
    const SourceInfo& Info() const override { return m_info; }
    float Sample(double latRad, double lonRad, double groundResM, float& metres) override;

private:
    const std::vector<int16_t>* m_elev;
    int m_nx, m_ny;
    double m_datumShift = 0.0;
    SourceInfo m_info;
};

// A regional equirect int16 window (row 0 = north), edge-feathered so it fades into whatever
// sits beneath it in the stack -- the render-time NeWeight feather, moved to paint time.
class WindowHeightSource : public HeightSource {
public:
    WindowHeightSource(const char* name, const char* structure, double cmPerPixel,
                       const std::vector<int16_t>* elev, int nx, int ny, double lon0,
                       double lat1, double dLon, double dLat, double featherFrac = 0.04,
                       double datumShiftM = 0.0);
    const SourceInfo& Info() const override { return m_info; }
    float Sample(double latRad, double lonRad, double groundResM, float& metres) override;

private:
    const std::vector<int16_t>* m_elev;
    int m_nx, m_ny;
    double m_lon0, m_lat1, m_dLon, m_dLat, m_feather;
    double m_datumShift = 0.0;
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
// M7d: THE BED CLASSIFIER -- the compositor's first SYNTHESIS source: a state-diagram NODE
// whose edge pulls the HEIGHT stack (depth + slope) and writes the COLOR fiber. Its PROGRAM
// is a data file (data/bed/bed_rules.json + bed_zones.geojson, content-hashed into the cache
// identity): an agent or a future terrain-building UX edits the file, and the soak rule
// turns that edit into exactly the touched repaints -- no code, no rebuild, no manual
// invalidation. It paints ABOVE imagery of open water (google's photo of the surface) and
// BELOW surveyed orthos (massgis actually saw the bed) -- STACK ORDER IS THE AUTHORITY
// RANKING. Colors are authored as DRY bed albedo: the renderer's refracted ray applies the
// water's own attenuation, so the satellite look is REPRODUCED by physics, not quoted.
class BedSynthSource : public ColorSource {
public:
    // comp/hgtChannel: the height stack this node reads (the cross-channel edge). Authors
    // default rule/zone files if absent -- never clobbers an existing edit (M6p law).
    bool Load(const std::string& rulesPath, const Compositor* comp, int hgtChannel);
    const SourceInfo& Info() const override { return m_info; }
    float Sample(double latRad, double lonRad, double groundResM, const PaintCtx& ctx,
                 uint8_t rgba[4]) override;

private:
    struct Rule {
        std::string name;
        double slopeMin = -1.0, slopeMax = 1e9;
        float shallow[3] = {0.74f, 0.68f, 0.53f};
        float deep[3] = {0.51f, 0.49f, 0.40f};
        float noise = 0.10f;
    };
    struct Zone {
        double lon0 = 1e9, lat0 = 1e9, lon1 = -1e9, lat1 = -1e9;
        std::vector<std::pair<double, double>> pts;   // lon, lat (degrees)
        std::string bed;                              // names a rule...
        float albedo[3] = {0.5f, 0.5f, 0.4f};         // ...or carries its own paint
        bool hasAlbedo = false;
    };
    const Rule* PickRule(double slope) const;
    const Compositor* m_comp = nullptr;
    int m_hgtCh = -1;
    std::vector<Rule> m_rules;
    std::vector<Zone> m_zones;
    double m_full0 = -9.0, m_full1 = 0.4, m_off0 = -14.0, m_off1 = 1.2;
    SourceInfo m_info;
};

// M9av: THE SEAFLOOR'S APPEARANCE, FROM THE BATHYMETRY THE PROJECT ALREADY INGESTS. No seafloor
// imagery exists in data/ (the ocean floor has no photographs), so its texture is a PRODUCT of
// the height stack -- ETOPO, the NE 15 s grid and CUDEM, whichever is finest under the texel:
// relief = the gradient of the bed (the grade-1 part of its derivative) lit by one fixed
// cartographic sun, times a dry sediment ramp keyed on datum depth (sand on the shelf, silt on
// the slope, clay on the plain). Global footprint; the alpha is the same waterline band the bed
// classifier hands land back through, with NO deep cutoff: the tree paints every ocean texel.
// DRY albedo, like synth.bed: the water's optics (measured K_d, the two-flux endpoint) stay the
// renderer's, and where they make the water opaque the renderer shades the endpoint by this
// relief instead of replacing it (Compose.hlsli SeafloorReliefMod).
class SeafloorReliefSource : public ColorSource {
public:
    bool Load(const std::string& rulesPath, const Compositor* comp, int hgtChannel);
    const SourceInfo& Info() const override { return m_info; }
    float Sample(double latRad, double lonRad, double groundResM, const PaintCtx& ctx,
                 uint8_t rgba[4]) override;

private:
    const Compositor* m_comp = nullptr;
    int m_hgtCh = -1;
    double m_full1 = 0.4, m_off1 = 1.2;       // the waterline band (metres, datum)
    double m_azDeg = 315.0, m_elDeg = 45.0;   // the cartographic sun
    double m_exagg = 25.0;                    // vertical exaggeration on the gradient
    double m_ambient = 0.45;
    SourceInfo m_info;
};

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
