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
    // THE FINEST ZOOM IS THE SCENE'S (streaming.googleZoom), held to 0..kMaxZoom. At
    // kDefaultZoom the name and structure are byte for byte what they were when 14 was a
    // literal, so no tree moves; at any other cap the structure says so (", to z17").
    static constexpr int kDefaultZoom = 14, kMaxZoom = 19;
    explicit GoogleColorSource(GoogleTileProvider* prov, int zoomCap = kDefaultZoom);
    const SourceInfo& Info() const override { return m_info; }
    void BeginTile(double latMin, double latMax, double lonMin, double lonMax,
                   double groundResM, PaintCtx& ctx) override;
    float Sample(double latRad, double lonRad, double groundResM, const PaintCtx& ctx,
                 uint8_t rgba[4]) override;
    bool Refusals(uint32_t& refused) const override;
    bool PaintTile(const ColorFrame& frame, const TileRequest& r, double groundResM, PaintCtx& ctx,
                   std::vector<uint8_t>& out, bool& complete, bool& anyCover, bool& fullCover) override;
    // A4: its finest, the cap's pixel on the ground at the box's centre (Pixel reads that zoom
    // below it, and a finer tile is the level above magnified). Its coarser levels stay its own
    // zooms, folded as M9bb folds them (HIERARCHY 4.20: a source with levels of its own keeps them).
    int FinestMip(const Lattice& l, double lat0, double lat1, double lon0, double lon1) const override;
    // The zoom a footprint asks for, held to 0..the cap (public for the selftest).
    int ZoomFor(double groundResM) const;

private:
    bool Pixel(int z, double latRad, double lonRad, uint8_t rgb[3]);
    GoogleTileProvider* m_prov;
    int m_zoomCap;
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
    // The grid's cells under the box and the bilinear's one beyond, the longitude wrapped; it owns
    // every box (its weight is 1 everywhere).
    Range RangeIn(double lat0, double lon0, double lat1, double lon1, float& lo, float& hi, bool& full) override;

private:
    const std::vector<int16_t>* m_elev;
    int m_nx, m_ny;
    double m_datumShift = 0.0;
    SourceInfo m_info;
};

// (The plane orthos and the overlays are scene sources now, compose/RasterFileSource.h.)

// A CUDEM topobathy window through an already-loaded, thalweg-preserving BathyModel grid.
// M6w: any number of focus windows (merrimack, capeann, boston...) stack as separate
// sources -- the HQ static insets over the NE-15s / ETOPO base.
class CudemHeightSource : public HeightSource {
public:
    explicit CudemHeightSource(const BathyModel* bathy, double featherFrac = 0.04,
                               const char* name = "noaa.cudem");   // PHASE C5: callers name the survey
    const SourceInfo& Info() const override { return m_info; }
    float Sample(double latRad, double lonRad, double groundResM, float& metres) override;
    // The weight the feather gives a world point (x east, z north, metres) -- Sample's own,
    // before the data is asked: 1 inside the band the window fades over, 0 at its edge.
    float EdgeWeight(double lon, double lat) const;
    // The grid's cells under the box (and the one beyond, the bilinear's), nodata skipped.
    Range RangeIn(double lat0, double lon0, double lat1, double lon1, float& lo, float& hi, bool& full) override;
    // WHERE THIS SURVEY PAINTS AT FULL WEIGHT, in its own cells: the first and last column and
    // row whose centres EdgeWeight calls 1 (HIERARCHY 4.17: a solver stands there). False when
    // no cell does. The band follows the feather: ask again and a new feather gives a new box.
    bool FullWeightCells(int& c0, int& r0, int& c1, int& r1) const;
    const BathyModel* Grid() const { return m_bathy; }

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
    // A4: the height's grain at the box's centre (it is a product of the height), and every level
    // where a zone's ring touches the box (a vector, exact at any grain).
    int FinestMip(const Lattice& l, double lat0, double lat1, double lon0, double lon1) const override;

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
// renderer's.
class SeafloorReliefSource : public ColorSource {
public:
    bool Load(const std::string& rulesPath, const Compositor* comp, int hgtChannel);
    const SourceInfo& Info() const override { return m_info; }
    float Sample(double latRad, double lonRad, double groundResM, const PaintCtx& ctx,
                 uint8_t rgba[4]) override;
    // A4: a product of the height, so the height's grain at the box's centre.
    int FinestMip(const Lattice& l, double lat0, double lat1, double lon0, double lon1) const override;

private:
    const Compositor* m_comp = nullptr;
    int m_hgtCh = -1;
    double m_full1 = 0.4, m_off1 = 1.2;       // the waterline band (metres, datum)
    double m_azDeg = 315.0, m_elDeg = 45.0;   // the cartographic sun
    double m_exagg = 25.0;                    // vertical exaggeration on the gradient
    double m_ambient = 0.45;
    SourceInfo m_info;
};

// THE CAPTURE LEVEL (2026-10-17): an ortho's pixel is a picture of the GROUND only where the ground
// stood above the water when the picture was taken, and under today's water the tree may show only
// such pixels. NOAA's 2011 low-water orthomosaic of the Merrimack painted its channel black and its
// bars sand as the BED under today's water (Mark: "the water imagery is hijacking the water"), and
// Google's and MassGIS's water pixels do the same more quietly everywhere. So a colour source may
// declare the water level it was flown at (the scene entry's captureLevel / captureDatum, resolved
// at the place as the tides are), and this node is the gate that level makes: a layer of the colour
// channel that PAINTS NOTHING, whose value at a texel is the weight the ground earns there -- 1
// above the level, 0 below, a smoothstep across the height's own uncertainty between -- read from
// the height tree at the texel's grain (Compositor::SampleHeight: the tree the GPU reads, the stack
// where none is bound). GateSource multiplies the layer's weight by it (ComposeTree.h; the tree's
// GateTile does it per texel), the same edge the survey gates the seafloor with: the pixel's COLOUR
// is never touched, only its weight.
//   ONE CONTINUOUS LAW, its limits the physics. The uncertainty is half the spread of the four
// texels the tree's bilinear stands between (HeightTree::Sample): nought on a flat bed, where the
// gate is a step at the level; one grain of the level read times the slope on a slope, the
// shoreline's own uncertainty said as a height; wide where only a coarse level holds the point. A
// source that declares no level has no gate: today's weight, byte for byte.
//   ITS IDENTITY folds the height stack's (every height layer's name|structure), so an edited bed
// repaints the gated tiles and nothing else; the gated source's own tree never moves. Its footprint
// is its layer's: inside it an opinion at every texel (weight 1, r = the factor, the mask's
// encoding); outside, none.
class LevelGateSource : public ColorSource {
public:
    // `layer`: the source gated (its footprint and its name); `levelM` on kHeightFrame (m NAVD88);
    // `said`: the level as the scene declared it, for the log and the identity.
    void Load(const ColorSource& layer, const Compositor* comp, int hgtChannel, double levelM,
              const std::string& said);
    const SourceInfo& Info() const override { return m_info; }
    float Sample(double latRad, double lonRad, double groundResM, const PaintCtx& ctx,
                 uint8_t rgba[4]) override;
    // THE LAW as a function: the weight of ground at height h against the level, with the height's
    // uncertainty s (metres, >= 0). Public so the selftest pins it beside the painted tile.
    static float Weight(float h, float level, float s);
    double LevelM() const { return m_level; }

private:
    const Compositor* m_comp = nullptr;
    int m_hgtCh = -1;
    double m_level = 0.0;
    SourceInfo m_info;
};

class EditsHeightSource : public HeightSource {
public:
    bool Load(const std::string& geojsonPath, float crestNavd = 2.5f);
    const SourceInfo& Info() const override { return m_info; }
    float Sample(double latRad, double lonRad, double groundResM, float& metres) override;
    // The crest where a land ring's box meets the box, -1 where a water ring's does.
    Range RangeIn(double lat0, double lon0, double lat1, double lon1, float& lo, float& hi, bool& full) override;
    // THE HAND EDIT YIELDS TO A FINER WITNESS (2026-10-18): the channel whose finer measured layers
    // this one yields to. Unnamed, the edit paints at full weight wherever its polygons are (the law
    // before this date). Named, its weight is the share of the texel those layers leave uncovered.
    void Witnesses(const Compositor* comp, int channel) { m_comp = comp; m_channel = channel; }
    std::string Law() const override;

private:
    float Uncovered(double latRad, double lonRad, double groundResM) const;
    const Compositor* m_comp = nullptr;
    int m_channel = -1;
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
