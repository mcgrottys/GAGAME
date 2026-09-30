// ================================================================================================
//  Compositor - M6i: the layer compositor. "The logical evolution of the mars sample."
//
//  The problem it retires: every data source used to reach the renderer through its OWN path --
//  the global Google cube had one shader block, the Mercator detail window another, the ETOPO
//  equirect a third, the NE 15s ring a fourth -- each with its own uv math, feathering and
//  blend weights evaluated PER PIXEL, PER FRAME. Two layers (globe, terrain) each re-derived
//  that math independently, which is exactly how they came to disagree about what the Earth
//  looks like (the M6h glitch report).
//
//  The architecture (the user's design, near verbatim):
//    1. SCHEMA REGISTRY -- for each source, its structure (mercator tile tree, equirect grid,
//       geotiff-derived window), format, and centimetres per pixel. SourceInfo below.
//    2. CHANNELS -- per planet, per meaning: earth.color, earth.height, mars.height (water and
//       air follow the same shape later). A channel is an ORDERED layer stack, bottom to top;
//       an upper layer paints over a lower one wherever it has coverage, with paint-time edge
//       feathering (bathymetry overwrites imagery softly, the NE ring fades into ETOPO).
//    3. COMPOSED QUADTREES -- a channel is REALIZED as tile pyramids: a cube-face pyramid for
//       the whole planet, a Mercator-aligned window pyramid where one region needs depth the
//       16k cube cannot carry. Tiles are painted ONCE, on the residency manager's worker
//       threads (the provider fn IS the paint), and cached to disk forever:
//       cache/composed/<channel>/<realization>/... -- an ephemeral-but-cached store of raw
//       64KB tiles, laid out exactly for CopyTiles, which is the DirectStorage-ready folder.
//       A second run streams composed tiles straight off disk: no HTTP, no reprojection, no
//       resampling -- the paint cost is once per machine per stack version.
//    4. ONE RENDER PATH -- shaders/Compose.hlsli. The renderer knows CHANNELS, not sources:
//       earth color is earth color, earth height is earth height. Every layer that reads them
//       calls the SAME ComposedColor/ComposedHeight functions on the SAME constants
//       (ComposedSurfaceCb, filled by SurfaceFrame::Fill alone), so no two CAN disagree.
//
//  GA hook: a layer is any object with a Sample(); the stack walk is an ordered composition of
//  operators. Raster operators over multivector-valued layers (contrast amplification, fades,
//  past/present morphs) drop in as ordinary layers -- that is where the fun kernel work lands.
// ================================================================================================
#pragma once

#include "compose/TileArchive.h"
#include "hal/Residency.h"
#include "core/Lattice.h"

#include <atomic>
#include <map>
#include <mutex>
#include <memory>
#include <string>
#include <vector>

namespace ga {

// ---- 1. the schema registry entry ----------------------------------------------------------
// Structure is a human-readable contract ("what shape is this data"), resolution is honest
// (centimetres per pixel at the finest level), coverage is a lat/lon box in degrees.
struct SourceInfo {
    std::string name;         // "google.satellite", "noaa.cudem.merrimack", ...
    std::string structure;    // "mercator-tile-tree jpeg 256px", "equirect-grid int16", ...
    std::string crs;          // the source's native projection -- the ALIGNMENT contract.
                              // GeoTIFF-derived sources carry theirs from the file; every
                              // Sample() resolves it to WGS84 lat/lon, the exchange frame,
                              // so two sources can only disagree by being WRONG, not by
                              // speaking different coordinates.
    double cmPerPixel = 0;    // finest ground resolution
    double lon0 = -180, lat0 = -90, lon1 = 180, lat1 = 90;   // coverage (degrees)
};

// ---- 2. sources ----------------------------------------------------------------------------
// A source answers point queries in lat/lon and returns a WEIGHT: 0 = no coverage here, 1 =
// full ownership, in between = the paint-time feather at its edges. groundResM is the composed
// texel's Mercator-equatorial metres-per-texel, so tile-tree sources can pick a zoom level.
//
// PaintCtx is PER-TILE, PER-SOURCE state owned by the REALIZATION (sources are shared across
// worker threads and must stay stateless): BeginTile sees the tile's lat/lon box once and may
// stash tile-scoped corrections -- the grade-normalization gain lives here.
struct PaintCtx {
    float gain[3] = {1.0f, 1.0f, 1.0f};
    // M9ak: PER-TILE SCRATCH. A source may do work once per tile in BeginTile and read it back
    // per texel. The vector GIS mask sweeps its rings by meridian into here -- one sweep per
    // tile instead of a point-in-polygon against 1.26 million edges per texel -- and the sweep
    // itself then happens only once per tile ADDRESS ever, because the mask is a layer and so
    // gets its own tree on the same addresses as the imagery it gates.
    std::vector<uint8_t> scratch;
    uint32_t scratchDim = 0;
    double sLat0 = 0, sLat1 = 0, sLon0 = 0, sLon1 = 0;   // the box `scratch` spans (radians)
};

class ColorSource {
public:
    virtual ~ColorSource() = default;
    virtual const SourceInfo& Info() const = 0;
    virtual void BeginTile(double, double, double, double, double, PaintCtx&) {}
    // (latMin, latMax, lonMin, lonMax radians, groundResM)
    virtual float Sample(double latRad, double lonRad, double groundResM, const PaintCtx& ctx,
                         uint8_t rgba[4]) = 0;
    // A source that fetches: true, with the distinct source tiles it refused this run because
    // its fetch budget was spent -- the fetches it WOULD have made. False for every source that
    // never fetches, which is all of them but one.
    virtual bool Refusals(uint32_t& refused) const {
        (void)refused;
        return false;
    }
};

class HeightSource {
public:
    virtual ~HeightSource() = default;
    virtual const SourceInfo& Info() const = 0;
    virtual float Sample(double latRad, double lonRad, double groundResM, float& metres) = 0;
};

// M6v: the two-component fiber -- a WATER-parameter field sample. The value algebra is a
// Cl(2)+ SPINOR (re, im): a tidal constituent's phasor, a current ellipse component, any
// quantity whose interpolation must happen in the plane (bilinear on amp/phase collapses
// amplitude across a phase gradient; bilinear on re/im is the geometrically sound blend).
// Same contract as every source: declare CRS/coverage/resolution, answer in the WGS84
// exchange frame, return a paint weight.
class FieldSource {
public:
    virtual ~FieldSource() = default;
    virtual const SourceInfo& Info() const = 0;
    virtual float Sample(double latRad, double lonRad, double groundResM, float out[2]) = 0;
};

// ---- 3. the compositor ---------------------------------------------------------------------
struct Lattice;
using ColorFrame = Lattice;   // M12 step 2b: the frame is the lattice (core/Lattice.h)   // the realization's geometry, defined below the class it belongs to

class Compositor {
public:
    static constexpr uint32_t kFaceDim = Lattice::kFaceDim;   // core/Lattice.h

    // The tile footprint type lives with the lattice now (core/Lattice.h).
    using TileBox = ga::TileBox;

    // Channels: an ordered stack, bottom -> top. Pointers are borrowed (main owns sources).
    int AddColorChannel(const std::string& name, std::vector<ColorSource*> stack);
    int AddHeightChannel(const std::string& name, std::vector<HeightSource*> stack);
    int AddFieldChannel(const std::string& name, std::vector<FieldSource*> stack);

    // Realizations: each returns a TileProviderFn for one residency tenant. The fn paints
    // (or reads back from the composed cache) one 64KB tile on a worker thread.
    //  * CubeColor: RGBA8 tiles (128x128) on the HARDWARE cube convention, mip m at
    //    Mercator-equivalent zoom (8 - m) for a 16k face.
    //  * WindowColor: RGBA8 tiles of a Mercator-aligned window; org/size in zBase pixels,
    //    mip m == zoom (zBase - m), texels are exact Mercator pixels.
    //  * CubeHeight: R16F tiles (256x128), metres, same cube convention.
    TileProviderFn CubeColor(int channel);
    TileProviderFn WindowColor(int channel, long long orgPxX, long long orgPxY, uint32_t sizePx,
                               int zBase);
    // Both of the above ARE this, with a frame filled in. A caller that has a frame -- a page of
    // a ladder, a per-source tree, a test -- names it directly and gets the same paint.
    TileProviderFn ColorRealization(int channel, const ColorFrame& frame);
    TileProviderFn CubeHeight(int channel);
    //  * WindowHeight: R16F tiles of the SAME Mercator window frame the color window uses --
    //    one frame, two channels, so near-field land/sea gates and normals ride CUDEM truth.
    TileProviderFn WindowHeight(int channel, long long orgPxX, long long orgPxY,
                                uint32_t sizePx, int zBase);
    //  * WindowField (M6v): RG16F 128x128 tiles of a Mercator window -- the water-parameter
    //    realizations (per-constituent tide phasors today; current ellipses next). Same
    //    soak-rule cache identity, same frame math, a different fiber.
    TileProviderFn WindowField(int channel, long long orgPxX, long long orgPxY,
                               uint32_t sizePx, int zBase);
    // The CPU stack sample -- physics, audits, and print realizations walk the SAME per-texel
    // math the paint loop runs, so a field queried on the CPU cannot disagree with its tiles.
    void SampleFieldStack(int channel, double latRad, double lonRad, double groundResM,
                          float out[2]) const;
    // M6w: the same contract for HEIGHT -- the physics-facing bed query. The SWE lattice, the
    // weather manager's products, and the renderer's tiles all evaluate THIS stack; there is
    // one bed, sampled at rungs.
    // M9av: the finest declared grain (metres) of any height layer whose footprint holds the
    // point -- what a derivative of the stack may honestly be taken at. A gradient stepped finer
    // than the data's cell reads the interpolant's facets (bilinear ETOPO at 4.9 km stepped at
    // 1 km rendered the continental slope as terraces).
    double HeightGrainM(int channel, double latRad, double lonRad) const;
    float SampleHeightStack(int channel, double latRad, double lonRad,
                            double groundResM) const;

    // The schema table, logged at startup: what feeds each channel, in what structure.
    void LogRegistry() const;

    struct Channel {
        std::string name;
        std::vector<ColorSource*> color;
        std::vector<HeightSource*> height;
        std::vector<FieldSource*> field;
        // M9ak: THE GATE. Parallel to `color`. A layer's paint weight is MULTIPLIED by the
        // coverage of its gate, per texel, which is a different composition from the stack's
        // own paint-over: the stack resolves who is on top, a gate resolves whether a layer
        // is ALLOWED HERE AT ALL.
        //
        // The user's rule, in their words: "the GIS mask gates, the height band refines". The
        // survey says where water can be; the bed classifier's height-band alpha says where
        // exactly the waterline falls INSIDE that. Neither can do the other's job -- GSHHG at
        // 1:250k cannot place a waterline to the metre, and a height threshold alone cannot
        // tell an inland hollow below +1.2 m from the sea.
        //
        // A gate source is a member of `color` (so it earns a cache identity and its own tree)
        // but is never painted, and a gate that does not reach a tile has NO OPINION -- factor
        // 1, not 0. A missing gate must never delete data.
        std::vector<int> gateOf;         // index into `color`, or -1 for ungated
        std::vector<uint8_t> gateOnly;   // 1 = this layer only gates; it paints nothing
    };
    // layer's weight *= gate's coverage. Both are indices into the channel's colour stack.
    void SetColorGate(int channel, size_t layer, size_t gate);
    // Public for the selftest: the soak rule is a CONTRACT, and contracts get pinned.
    const Channel& ChannelAt(int id) const { return m_channels[id]; }
    int ChannelCount() const { return static_cast<int>(m_channels.size()); }
    // THE SOAK RULE'S MEMBERSHIP TEST, public so the tile trees apply the identical rule: a
    // footprint (degrees) belongs to a tile if it is global, or overlaps by >= ~2 texels in
    // some axis. A tree that used a looser test painted a speck of ortho into a coarse cube
    // tile the incumbent never painted -- 27/255 on 567 texels, found by the audit.
    static bool Touches(double lon0, double lat0, double lon1, double lat1, const TileBox& b);
    uint64_t ColorSubset(const Channel& ch, const TileBox& box,
                         std::vector<size_t>& included) const;
    // The one colour paint loop, exposed so anything that composes must go through THIS walk
    // rather than writing a second one that can drift from it.
    void PaintColorTile(const Channel& ch, const ColorFrame& frame, const TileRequest& r,
                        const TileBox& box, const std::vector<size_t>& inc,
                        std::vector<uint8_t>& out, bool& complete) const;
    // ONE source over the same addresses -- what a per-source tree stores (SourceTree.h). RGB
    // is the source's own bytes, untouched; ALPHA IS THE PAINT WEIGHT, quantized, because the
    // stack's feather lives in the weight and a tree that dropped it could not be composed
    // afterwards. `anyCover`/`fullCover` are what let the composite decide, per tile, between a
    // REFERENCE to this tree and a genuine composition.
    static void PaintSourceTile(ColorSource* src, const ColorFrame& frame, const TileRequest& r,
                                const TileBox& box, std::vector<uint8_t>& out, bool& complete,
                                bool& anyCover, bool& fullCover);
    uint64_t HeightSubset(const Channel& ch, const TileBox& box,
                          std::vector<size_t>& included) const;
    uint64_t FieldSubset(const Channel& ch, const TileBox& box,
                         std::vector<size_t>& included) const;

    std::atomic<uint32_t> painted{0};     // tiles composed this run
    std::atomic<uint32_t> cacheHits{0};   // tiles served from the composed cache

private:
    // M6k: THE SOAK RULE. A tile's cache identity hashes only the sources that MEANINGFULLY
    // touch it: footprint intersects the tile AND spans >= ~2 texels at this LOD (in at least
    // one axis). Everything the user asked of multi-LOD painting falls out by construction:
    //  * a new source repaints exactly the tiles it can change -- at every LOD, coarser
    //    (box-filtered 4->1, recursively: each LOD averages the source over its OWN texel
    //    footprint) and finer (bilinear 1->4) alike;
    //  * at a LOD so coarse the source would touch less than ~a texel, it drops OUT of the
    //    subset, the tag reverts to the without-it identity, and the repaint short-circuits
    //    into a cache HIT -- redundant paints never run;
    //  * tiles a source never touched keep their identity forever.
    // (ColorSubset/HeightSubset implement it; declared public above for the selftest.)
    // M9ai: the archive for one realization, opened once and cached. Returns true and fills
    // `loc` when the tile is present under the CURRENT subset -- the caller then returns
    // without reading, and the bytes stay on disk for DirectStorage. A missing archive, or a
    // tile painted after the last pack, simply misses and the loose-file path runs.
    bool TryArchive(const Channel& ch, const char* realization, const TileRequest& r,
                    uint64_t subset, TileLoc* loc);
    std::map<std::string, TileArchive> m_archives;
    std::mutex m_archiveMx;

    std::string CachePath(const Channel& ch, const char* realization, const TileRequest& r,
                          uint64_t subset) const;
    bool ReadCached(const std::string& path, std::vector<uint8_t>& out);
    void WriteCached(const std::string& path, const std::vector<uint8_t>& data);
    void EnsureCacheDir(const Channel& ch, const char* realization);

    std::vector<Channel> m_channels;
};

// The hardware cubemap convention (D3D spec) -- the ONE cube addressing every composed cube
// realization writes and every TextureCube sample reads. Moved here from the Google provider:
// it belongs to the ADDRESSING, not to any one source.
//   +X: dir = ( 1, -t, -s)   -X: dir = (-1, -t,  s)
//   +Y: dir = ( s,  1,  t)   -Y: dir = ( s, -1, -t)
//   +Z: dir = ( s, -t,  1)   -Z: dir = (-s, -t, -1)     with s = 2u-1, t = 2v-1.

// ---- 4. the one render path's constants ----------------------------------------------------
// Mirrors GA_COMPOSED_CB_ROWS in Common.hlsli (count rows on BOTH sides after any edit): the
// rows of SurfaceCb (b2), ONE buffer the frame loop fills through SurfaceFrame::Fill ALONE
// (compose/SurfaceFrame.h, M12 step 4a) and the renderer binds once a frame for every layer
// that samples a planet's composed channels (M12 step 4g), so no two can disagree on the math.
struct ComposedSurfaceCb {
    uint32_t u[4];    // color cube SRV, color cube residency map, window SRV, window res map
    uint32_t u2[4];   // height cube SRV + residency map, height WINDOW SRV + residency map
    uint32_t u3[4];   // GIS survey stencil: window SRV (R8G8 coast,river), global SRV (R8)
    float f[4];       // color cube on, window on, height on, planet radius (m)
    float merc[4];    // window org px x, org px y, 1/sizePx, full-world px at window zBase
    float g[4];       // height cube max lod, height texel arc (rad), height window max lod,
                      // stencil overlay on
    float r0[4];      // planet->tangent rotation rows (east / up / north)
    float r1[4];
    float r2[4];
    uint32_t u4[4];   // M7f: detail color window (z17) SRV + residency, fine edit mask SRV
    float det[4];     // detail uv from window uv: offset xy, scale z; w = fine edit mask on
    float ground[4];  // M12 step 4f: ground texel (m) at mip 0 -- the cube, the z14 window,
                      // the z17 detail (Lattice::GroundRes(0)); w spare. Was ed[4], the fine
                      // edit mask box, dead since M9ay.
    uint32_t u5[4];   // M9ap PAGES: colour array SRV, array residency SRV, window slice,
                      // detail slice. u5[0] == ~0 means the old three-tenant path.
    uint32_t u6[4];   // M9aq HEIGHT PAGES: height array SRV, array residency SRV, window
                      // slice. u6[0] == ~0 means the old cube + window tenants.
    // HIERARCHY 4.17: THE STANDING BLOCKS (SurfaceFrame::blocks), appended so no row above
    // moves; at most SurfaceFrame::kMaxBlocks, coarsest rung first. Block i: row i of blkU /
    // blkV / blkW is one of PageTexelUv's planes about the eye's own tangent frame, anchored on
    // the multiple of 16384 texels of its rung nearest the eye (SurfaceFrame::BlockRows, commit
    // 3); blkO[2i], blkO[2i + 1] the whole blocks from that anchor to the block's own origin;
    // blkG[i] its ground (m) at mip 0; blkS[i] its slice of the colour and the mask. blkE is the
    // eye in the tangent axes about the planet's centre, for a stage with only a direction.
    // blkN[0] is the count: 0 is today's Mercator pages.
    float blkU[32];
    float blkV[32];
    float blkW[32];
    float blkG[8];
    uint32_t blkS[8];
    float blkO[16];
    float blkE[4];
    uint32_t blkN[4];
};

// M12 step 2b: THE FRAME moved to core/Lattice.h and became the LATTICE every tree and
// tenant inherits (its M9aj banner went with it). The old name stays usable here so no
// call site changed in the move; new code says Lattice.
using ColorFrame = Lattice;

// The compositor's --selftest gate (ComposeTest.cpp): paint order, per-pixel weights, alpha,
// cache identity, transient-never-cached, cube/window addressing vs closed forms, stack-hash
// isolation. Returns false (and logs FAILs) if any contract is broken.
bool RunComposeSelfTest();

// M12 step 4a: the fill is SurfaceFrame::Fill (compose/SurfaceFrame.h) -- the surface
// declared once fills its own rows; the M9ap pages rule is stated in its banner.

}  // namespace ga
