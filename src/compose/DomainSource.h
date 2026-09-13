// ================================================================================================
//  DomainSource / DomainCompositor - M9i: composition generalized past imagery.
//
//  NAMED APART FROM Compositor.h's FieldSource ON PURPOSE. That one is the imagery/tide
//  compositor's two-component spinor source -- same contract shape (declare CRS, answer in the
//  WGS84 exchange frame, return a paint weight), narrower scope. This is the generalization
//  across DOMAINS, so it carries a distinct name rather than shadowing a working class.
//
//  Compositor.h already composes SOURCES into a raster: each answers a point query in lat/lon
//  and returns a WEIGHT -- 0 no coverage, 1 full ownership, between = the paint-time feather.
//  That model is right and this keeps it verbatim. What it cannot express is everything that is
//  not a colour on a 2-D mercator tile:
//
//      POINT      buoys, tide and current stations, ADCP -- scattered, no grid at all
//      PROFILE    a depth cast: one column, many depths
//      RASTER     a 2-D grid (GoMOFS currents, CUDEM bed, a GeoTIFF survey)
//      VOLUME     a 3-D field (cloud density, water column, crust composition)
//
//  All four can contribute to ONE sparse GA product, and often should: an ocean-state field
//  wants the model raster for shape and the buoy points for authority, exactly as the sea state
//  already assimilates buoy 44013 over GFS-Wave. So the interface is deliberately domain-neutral
//  -- a source answers "what is your value and your weight at this position", and the DOMAIN
//  only decides how it answers, never what the compositor does with the answer.
//
//  WHAT COMPOSES WITH WHAT. The output is a page of the shared (level, x, y) tree, so every
//  source is asked in the OUTPUT's frame and the alignment is structural rather than negotiated
//  (docs/SPARSE_GA.md 15, 21). A source that cannot answer somewhere returns weight 0, which is
//  absence -- and absence composes to a NULL tile, never to a zero pretending to be a
//  measurement (the ingest rule, GeoRef.h).
//
//  GRADES. Each source declares its grade signature; a compositor writing a grade-1 product
//  will not silently accept a grade-0 source. What a PRODUCT of two composed fields becomes is
//  still the Cayley closure's business (GradeField.h), not this file's.
//
//  A LOADER IS NOT A SOURCE. FieldLoader decodes a file into tiles -- storage. A DomainSource
//  answers queries in the output's frame -- composition. RasterSource below adapts one into the
//  other, which is the only place the two ideas meet.
// ================================================================================================
#pragma once

#include "core/FieldLoader.h"
#include "core/GaUnits.h"
#include "core/GeoRef.h"
#include "core/GradeField.h"
#include "compose/Projections.h"
#include "core/PageTable.h"

#include <algorithm>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

namespace ga {

struct Lattice;
using ColorFrame = Lattice;   // M12 step 2b: the frame is the lattice (core/Lattice.h)
struct TileRequest;

enum class SourceDomain : uint8_t { Point, Profile, Raster, Volume };

// Where a value is wanted. Depth and time are carried for every query even when a 2-D source
// ignores them: a compositor that had to know which fields a source reads could not mix
// domains, which is the whole point.
struct DomainQuery {
    double lon = 0.0, lat = 0.0;
    double depthM = 0.0;      // 0 = surface, positive downward
    double unixT = 0.0;
    double groundM = 1.0;     // the output texel's ground size -- a source may answer coarser
};

// Up to 4 channels, because a Cl(2) multivector is 4 and everything here is at most that.
struct DomainValue {
    float c[4] = {0, 0, 0, 0};
    float weight = 0.0f;      // 0 = no coverage. THE thing that makes absence composable.
};

// ================================================================================================
//  A source of field values, in any domain.
// ================================================================================================
class DomainSource {
public:
    virtual ~DomainSource() = default;
    virtual const char* Name() const = 0;
    virtual SourceDomain Domain() const = 0;
    virtual uint8_t GradeSig() const = 0;
    virtual uint32_t Channels() const = 0;
    // M9l: what the numbers MEAN. Grade says how a value transforms; this says what it IS.
    // Defaulted rather than pure so an existing source keeps compiling -- but the default is
    // Unknown, which the compositor refuses, so silence is not a way past the check.
    virtual const UnitSpec& Unit() const {
        static const UnitSpec kUnstated;
        return kUnstated;
    }
    // The value and weight here. False is identical to weight 0 and exists only for callers
    // that want to skip the copy.
    virtual bool SampleAt(const DomainQuery& q, DomainValue& out) const = 0;
    // M9bc: TILE-NATIVE nodes. Some answers are regional -- a boundary-value solve with a phase
    // gauge, a stencil operator with a margin -- and cannot be asked one texel at a time. Such a
    // node paints a whole tile of values for a frame's address (any mip); the tree treats the
    // result exactly as a per-texel paint: same formats, identity, fold, refetch. False = the
    // answer is not ready for this identity (the tree returns Transient and asks again).
    virtual bool TileNative() const { return false; }
    virtual bool PaintTile(const ColorFrame&, const TileRequest&, uint32_t, uint32_t,
                           std::vector<DomainValue>&) const {
        return false;
    }
    // ---- THE TILE SCOPE. A per-texel node whose INPUTS can move under it -- a bucket the frame
    // thread re-keys while a paint is running on a loader job -- snapshots them in BeginTile and
    // answers every SampleAt of THAT tile from the snapshot, so one tile is one field. EndTile
    // returns false when the snapshot went stale before the tile finished: the tree answers
    // Transient and asks again rather than storing two fields as one.
    //
    // This is the per-texel twin of the contract PaintTile already has for tile-native nodes
    // ("false = the answer is not ready for this identity"), and it is shaped like
    // ColorSource::BeginTile in Compositor.h, which the colour path has always had. Called on
    // the painting thread, once each, around the texel loop; the default pair is free.
    virtual void BeginTile() const {}
    virtual bool EndTile() const { return true; }
    // Cheap rejection so a compositor can skip a source over a whole page without querying it.
    virtual bool MayCover(double lon0, double lat0, double lon1, double lat1) const {
        (void)lon0; (void)lat0; (void)lon1; (void)lat1;
        return true;
    }
    // Higher wins where two sources both claim full weight -- a 1.5 m survey over a 700 m
    // model. Equal priority blends by weight, which is what a feathered edge wants.
    virtual int Priority() const { return 0; }

    // ---- M9m: THE TREE. Composition converges -- many leaves, fewer nodes, one product -- so a
    // source may itself have inputs. Defaulted to a leaf, which is what a loader-backed source
    // is, so nothing that exists today has to say anything. PrintTree walks these.
    virtual size_t InputCount() const { return 0; }
    virtual const DomainSource* Input(size_t) const { return nullptr; }
    // What this node DOES, for the printed tree: "load", "compose", "subtract", "normalize".
    virtual const char* NodeKind() const { return "load"; }
    // The gantt column. A bed composes once; a tide is a function of t and must be re-evaluated
    // every instant. Stating it is what lets one tree carry rows on different clocks.
    virtual const char* Cadence() const { return "static"; }
    // M12 step 2d: THE LATTICE A NODE DECLARES, or nullptr to INHERIT its parent's -- the
    // identity element of the fold TileTree::Resolve applies at every node, the way Footprint,
    // Unit and Cadence already flow through the graph. No shipped node overrides it yet; the
    // first that does (a window realization under a cube root) composes across lattices only
    // through a resample node, which the tree refuses to invent (TileTree::RefuseLattice).
    virtual const Lattice* OwnLattice() const { return nullptr; }
    // M9am: WHAT A CACHE MAY KEY THIS NODE'S OUTPUT ON. A leaf's tiles on disk are a function of
    // the data it reads and nothing else, so the default is name + unit; a loader that knows its
    // structure (a tile tree, a file version) says so, and a wrapper that changes nothing
    // identity-bearing forwards its inner's. Compose nodes derive theirs from their inputs.
    virtual std::string Identity() const { return std::string(Name()) + "|" + Unit().Describe(); }
    // M9am: the declared footprint in degrees, when there is one. MayCover answers "might this
    // touch"; this answers "how much", which the soak rule needs -- a source whose footprint
    // spans less than ~2 texels of a tile drops out of that tile's subset, so a coarse tile is
    // not repainted for a speck. False = unknown, which the caller treats as "always in".
    virtual bool Footprint(double& lon0, double& lat0, double& lon1, double& lat1) const {
        (void)lon0; (void)lat0; (void)lon1; (void)lat1;
        return false;
    }
};

// ================================================================================================
//  RASTER -- the adapter that turns a FieldLoader into a source. This is the only place the
//  storage idea and the composition idea meet.
// ================================================================================================
class RasterSource : public DomainSource {
public:
    RasterSource(std::unique_ptr<FieldLoader> loader, int priority = 0)
        : m_loader(std::move(loader)), m_priority(priority) {
        if (m_loader) {
            m_unit = UnitSpec::Parse(m_loader->Ref().valueUnit);
            CacheAll();
        }
    }
    bool Valid() const { return m_loader != nullptr && !m_cache.empty(); }
    // The source's own georeference -- a consumer placing it as a PAGE needs its extent, and
    // getting that from the source rather than restating it is what keeps the two aligned.
    const GeoRef& Ref() const { return m_loader->Ref(); }

    const char* Name() const override { return m_loader ? m_loader->Name() : "raster"; }
    SourceDomain Domain() const override { return SourceDomain::Raster; }
    // Straight from GeoRef::valueUnit -- the string the loader read out of the file, parsed
    // once here. The loader still reports; this stage still decides.
    const UnitSpec& Unit() const override { return m_unit; }
    uint8_t GradeSig() const override { return m_loader ? m_loader->GradeSig() : 0; }
    uint32_t Channels() const override { return m_chan; }
    int Priority() const override { return m_priority; }

    bool MayCover(double lon0, double lat0, double lon1, double lat1) const override {
        const GeoRef& g = m_loader->Ref();
        double x0, y0, x1, y1;
        g.TexelToWorld(0.0, 0.0, x0, y0);
        g.TexelToWorld(double(g.width), double(g.height), x1, y1);
        if (x0 > x1) std::swap(x0, x1);
        if (y0 > y1) std::swap(y0, y1);
        return !(lon1 < x0 || lon0 > x1 || lat1 < y0 || lat0 > y1);
    }

    bool SampleAt(const DomainQuery& q, DomainValue& out) const override {
        const GeoRef& g = m_loader->Ref();
        // THE EXCHANGE FRAME IS WGS84 LAT/LON (Compositor.h): a source resolves it into its
        // OWN projection EXACTLY, so two sources can only disagree by being wrong, not by
        // speaking different coordinates. Applying the affine straight to lat/lon would be
        // correct only for EPSG:4326 and would put a UTM survey or a 3857 tile somewhere else
        // entirely -- silently, because the answer is still a plausible number.
        double px = q.lon, py = q.lat;
        if (!ToSourceCrs(g, q.lon, q.lat, px, py)) return false;
        // Inverse of GeoRef's affine. centers is honoured rather than assumed: a
        // corner-anchored source read as centre-anchored is a half-texel slide that reads as
        // a subtle misalignment, never as an error (priors 7).
        const double o = g.centers ? 0.5 : 0.0;
        const double fx = (px - g.originX) / g.scaleX - o;
        const double fy = (py - g.originY) / g.scaleY - o;
        // BILINEAR, and it is not a quality nicety. A consumer that differentiates this
        // field -- and grad() is exactly that -- gets ZERO inside every source texel and a
        // spike at each boundary if the sample is nearest. Upsampled 280x200 to 1863x1174 that
        // renders as a grid of lines, which is the derivative of a staircase and not of the
        // current. Corners with no coverage drop out of the blend rather than pulling it to
        // zero, so a coastline stays a coastline instead of a ramp into land.
        const int x0 = int(std::floor(fx)), y0 = int(std::floor(fy));
        const double tx = fx - x0, ty = fy - y0;
        double acc[4] = {0, 0, 0, 0}, wsum = 0.0;
        for (int dy = 0; dy < 2; ++dy) {
            for (int dx = 0; dx < 2; ++dx) {
                const int xi = x0 + dx, yi = y0 + dy;
                if (xi < 0 || yi < 0 || xi >= int(g.width) || yi >= int(g.height)) continue;
                const float cv = m_cov[size_t(yi) * g.width + xi];
                if (cv <= 0.0f) continue;
                const double bw = (dx ? tx : 1.0 - tx) * (dy ? ty : 1.0 - ty);
                if (bw <= 0.0) continue;
                const size_t k = (size_t(yi) * g.width + xi) * m_chan;
                for (uint32_t c = 0; c < m_chan && c < 4; ++c) acc[c] += bw * m_cache[k + c];
                wsum += bw;
            }
        }
        if (wsum <= 0.0) return false;   // absence: the source simply is not here
        for (uint32_t c = 0; c < m_chan && c < 4; ++c) out.c[c] = float(acc[c] / wsum);
        out.weight = float(wsum);
        return true;
    }

    // WGS84 -> the source's declared CRS. Forward transforms only, which is all a point
    // query needs and all Projections.h offers.
    static bool ToSourceCrs(const GeoRef& g, double lon, double lat, double& x, double& y) {
        constexpr double kD2R = 3.14159265358979323846 / 180.0;
        switch (g.kind) {
            case CrsKind::Geographic:
            case CrsKind::Equirect:
                x = lon;
                y = lat;
                return true;
            case CrsKind::WebMercator: {
                constexpr double kR = 6378137.0;
                if (lat > 85.05112878 || lat < -85.05112878) return false;
                x = kR * lon * kD2R;
                y = kR * std::log(std::tan(0.78539816339744831 + 0.5 * lat * kD2R));
                return true;
            }
            case CrsKind::TransverseMercator: {
                // UTM zone from the EPSG code where it is one of the standard bands; the
                // engine's own orthos are EPSG:6348 (UTM 19N).
                int zone = 19;
                if (g.epsg >= 32601 && g.epsg <= 32660) zone = g.epsg - 32600;
                else if (g.epsg >= 32701 && g.epsg <= 32760) zone = g.epsg - 32700;
                else if (g.epsg == 6348) zone = 19;
                const TransverseMercator tm = TransverseMercator::Utm(zone);
                tm.Forward(lat * kD2R, lon * kD2R, x, y);
                return true;
            }
            default:
                return false;   // a CRS this engine cannot evaluate is refused, never guessed
        }
    }

private:
    void CacheAll() {
        const GeoRef& g = m_loader->Ref();
        m_chan = m_loader->Channels();
        m_cache.assign(size_t(g.width) * g.height * m_chan, 0.0f);
        m_cov.assign(size_t(g.width) * g.height, 0.0f);
        // Pull the whole grid once through the tile interface. These sources are forecast
        // grids -- hundreds of texels on a side -- so this is kilobytes, and it keeps the
        // loader's tile contract as the single way data leaves a file.
        const uint32_t tw = 64, th = 64;
        TilePayload tp;
        for (uint32_t ty = 0; ty * th < g.height; ++ty) {
            for (uint32_t tx = 0; tx * tw < g.width; ++tx) {
                if (!m_loader->LoadTile(tx, ty, tw, th, tp)) continue;   // all-absent tile
                for (uint32_t r = 0; r < th; ++r) {
                    const uint32_t sy = ty * th + r;
                    if (sy >= g.height) break;
                    for (uint32_t c = 0; c < tw; ++c) {
                        const uint32_t sx = tx * tw + c;
                        if (sx >= g.width) break;
                        const size_t src = (size_t(r) * tw + c) * tp.channels;
                        bool any = false;
                        for (uint32_t ch = 0; ch < tp.channels; ++ch) {
                            if (tp.data[src + ch] != 0.0f) any = true;
                        }
                        if (!any) continue;
                        const size_t dst = (size_t(sy) * g.width + sx) * m_chan;
                        for (uint32_t ch = 0; ch < m_chan && ch < tp.channels; ++ch) {
                            m_cache[dst + ch] = tp.data[src + ch];
                        }
                        m_cov[size_t(sy) * g.width + sx] = 1.0f;
                    }
                }
            }
        }
    }

    std::unique_ptr<FieldLoader> m_loader;
    std::vector<float> m_cache, m_cov;
    uint32_t m_chan = 1;
    int m_priority = 0;
    UnitSpec m_unit;
};

// ================================================================================================
//  POINT -- scattered observations. Inverse-distance within a radius, which is the same shape
//  the tide stations already use (StationFieldSource, IDW p=2), kept here so a buoy can join a
//  raster in one product rather than being assimilated by hand somewhere downstream.
// ================================================================================================
class PointSource : public DomainSource {
public:
    struct Obs {
        double lon, lat;
        float c[4];
    };
    PointSource(std::string name, uint8_t grade, uint32_t channels, double radiusDeg,
                int priority = 10, const char* valueUnit = "")
        : m_name(std::move(name)), m_grade(grade), m_chan(channels), m_radius(radiusDeg),
          m_priority(priority), m_unit(UnitSpec::Parse(valueUnit)) {}

    void Add(const Obs& o) { m_obs.push_back(o); }
    size_t Count() const { return m_obs.size(); }

    const char* Name() const override { return m_name.c_str(); }
    SourceDomain Domain() const override { return SourceDomain::Point; }
    const UnitSpec& Unit() const override { return m_unit; }
    uint8_t GradeSig() const override { return m_grade; }
    uint32_t Channels() const override { return m_chan; }
    int Priority() const override { return m_priority; }

    bool SampleAt(const DomainQuery& q, DomainValue& out) const override {
        double wsum = 0.0, acc[4] = {0, 0, 0, 0};
        for (const Obs& o : m_obs) {
            const double dx = o.lon - q.lon, dy = o.lat - q.lat;
            const double d2 = dx * dx + dy * dy;
            if (d2 > m_radius * m_radius) continue;
            const double w = 1.0 / (d2 + 1e-12);
            for (uint32_t c = 0; c < m_chan && c < 4; ++c) acc[c] += w * o.c[c];
            wsum += w;
        }
        if (wsum <= 0.0) return false;
        for (uint32_t c = 0; c < m_chan && c < 4; ++c) out.c[c] = float(acc[c] / wsum);
        // Weight falls off to the radius so a point set FEATHERS into whatever it sits on
        // instead of stamping a disc -- the same reason the imagery window feathers.
        double best = m_radius * m_radius;
        for (const Obs& o : m_obs) {
            const double dx = o.lon - q.lon, dy = o.lat - q.lat;
            best = (std::min)(best, dx * dx + dy * dy);
        }
        const double t = 1.0 - std::sqrt(best) / m_radius;
        out.weight = float((std::max)(0.0, (std::min)(1.0, t)));
        return out.weight > 0.0f;
    }

private:
    std::string m_name;
    std::vector<Obs> m_obs;
    uint8_t m_grade;
    uint32_t m_chan;
    double m_radius;
    int m_priority;
    UnitSpec m_unit;
};

// ================================================================================================
//  NormalizedSource -- THE NORMALIZATION STAGE, as a decorator.
//
//  A DomainSource wrapping a DomainSource, rescaling every channel on the way out. A decorator
//  rather than a mutation because the raw source stays valid and inspectable, and because it
//  COMPOSES: a normalized source is itself a source, so nothing downstream needs a second code
//  path and a normalized source can feed another compositor as easily as a raw one. The wrapper
//  is skipped entirely when the factor is 1 and the offset 0 -- normalization that costs nothing
//  when nothing is needed is normalization people leave switched on.
//
//  WEIGHT IS NOT TOUCHED. Coverage is a fraction in both frames; scaling it would darken every
//  partially covered texel, which is exactly the bug MipReduce.hlsl's comment warns about.
// ================================================================================================
class NormalizedSource : public DomainSource {
public:
    NormalizedSource(std::shared_ptr<DomainSource> inner, const UnitSpec& from, const UnitSpec& to)
        : m_inner(std::move(inner)),
          m_scale(to.toCanonical != 0.0 ? from.toCanonical / to.toCanonical : 1.0),
          m_offset(from.datumShiftM - to.datumShiftM),
          m_unit(to) {}

    const char* Name() const override { return m_inner->Name(); }
    SourceDomain Domain() const override { return m_inner->Domain(); }
    uint8_t GradeSig() const override { return m_inner->GradeSig(); }
    uint32_t Channels() const override { return m_inner->Channels(); }
    int Priority() const override { return m_inner->Priority(); }
    const UnitSpec& Unit() const override { return m_unit; }
    const char* NodeKind() const override { return "normalize"; }
    const char* Cadence() const override { return m_inner->Cadence(); }
    size_t InputCount() const override { return 1; }
    const DomainSource* Input(size_t i) const override { return i ? nullptr : m_inner.get(); }
    // The scale and offset are part of the tile FORMAT, not of the data, so the cache keys on
    // the inner identity plus what was done to it.
    std::string Identity() const override {
        char b[64];
        snprintf(b, sizeof(b), "|norm x%.9g %+.6g", m_scale, m_offset);
        return m_inner->Identity() + b;
    }
    bool MayCover(double a, double b, double c, double d) const override {
        return m_inner->MayCover(a, b, c, d);
    }
    bool Footprint(double& a, double& b, double& c, double& d) const override {
        return m_inner->Footprint(a, b, c, d);
    }
    bool SampleAt(const DomainQuery& q, DomainValue& out) const override {
        if (!m_inner->SampleAt(q, out)) return false;
        const uint32_t n = m_inner->Channels();
        for (uint32_t i = 0; i < n && i < 4; ++i) {
            out.c[i] = static_cast<float>(out.c[i] * m_scale + m_offset);
        }
        return true;
    }

private:
    std::shared_ptr<DomainSource> m_inner;
    double m_scale = 1.0;
    double m_offset = 0.0;
    UnitSpec m_unit;
};

// The one call a caller makes. Returns a source ready to compose, or NULL with a logged reason.
// Null is the point: an incommensurable source never reaches Compose, so no later stage has to
// carry a doubt about what its floats mean.
inline std::shared_ptr<DomainSource> Normalize(std::shared_ptr<DomainSource> src,
                                               const UnitSpec& target) {
    if (!src) return nullptr;
    const UnitSpec from = src->Unit();
    std::string why;
    if (!target.AcceptsFrom(from, &why)) {
        Log("[normalize] %s REFUSED: %s (source %s, target %s)", src->Name(), why.c_str(),
            from.Describe().c_str(), target.Describe().c_str());
        return nullptr;
    }
    const double scale = target.toCanonical != 0.0 ? from.toCanonical / target.toCanonical : 1.0;
    const double offset = from.datumShiftM - target.datumShiftM;
    if (scale == 1.0 && offset == 0.0) return src;   // already there; the stage is free
    Log("[normalize] %s: %s -> %s (x%.6g %+.4g)", src->Name(), from.Describe().c_str(),
        target.Describe().c_str(), scale, offset);
    return std::make_shared<NormalizedSource>(std::move(src), from, target);
}

// Normalize to the canonical form of whatever the source already is -- a file in feet becoming a
// field in metres without anyone having to name a target.
inline std::shared_ptr<DomainSource> NormalizeToSi(std::shared_ptr<DomainSource> src) {
    if (!src) return nullptr;
    UnitSpec t = src->Unit();
    t.toCanonical = 1.0;
    t.datumShiftM = 0.0;
    return Normalize(std::move(src), t);
}

// ================================================================================================
//  DomainCompositor -- N sources of any domain into ONE page of a GA product.
//
//  The rule, and it is the imagery compositor's rule unchanged: highest PRIORITY that has any
//  coverage wins the pixel; equal priorities blend by weight; nothing anywhere means the texel
//  is absent, and an entirely absent page is never allocated. Sources are asked in the output's
//  frame, so mixing a 700 m model raster with a scattered buoy set needs no agreement between
//  them beyond both being asked the same question.
// ================================================================================================
class DomainCompositor {
public:
    void SetLadder(const LevelLadder& l) { m_ladder = l; }

    // M9n: HOW SOURCES COMBINE, and it is not one rule.
    //
    //   PriorityAverage  the native rule: highest priority wins, ties blend by weight. Right
    //                    when sources are RIVALS describing one quantity -- a 1.5 m survey and
    //                    a 700 m model of the same current.
    //   LayeredOver      h += (m - h) * w, bottom to top. Right when sources are a STACK with
    //                    an authority order, each painting over what is under it. This is
    //                    Compositor::SampleHeightStack exactly, and reproducing the engine's
    //                    bed requires it: the two rules agree only where every weight is 0 or
    //                    1, and disagree precisely across a feather -- which is where a survey
    //                    edit meets the grid beneath it, and where the bed's 28 m lived.
    enum class Blend : uint8_t { PriorityAverage, LayeredOver };
    void SetBlend(Blend b) { m_blend = b; }
    Blend BlendMode() const { return m_blend; }
    // A page's geography, EXACTLY -- not the anchor-linear approximation. That form
    // (BathyModel::kOrgLat with a frozen mPerLon) is declared valid only NEAR ITS ANCHOR and
    // is what every local water consumer shares; a planet-scale tree cannot use it, because a
    // metres-per-degree frozen at 42.8N is wrong by 40% at the equator and unbounded at the
    // pole. So a page states its own lat/lon extent and the compositor interpolates within it.
    struct PageGeo {
        // M9ad: when true, lat0/dLat are NORMALIZED MERCATOR y (0 = north pole edge, 1 = south)
        // rather than degrees, and each row is converted back to latitude before the sources are
        // asked. A linear lat/lon page is fine over a survey and badly wrong over a continent --
        // the coarse rungs of a planet ladder span thousands of km, where the two frames
        // disagree by hundreds. Imagery is Mercator-native anyway, so this is also the frame
        // that costs the sources no reprojection.
        bool mercator = false;
        double lon0 = 0.0, lat0 = 0.0;   // texel (0,0) CENTRE
        double dLon = 0.0, dLat = 0.0;   // degrees per texel; dLat < 0 = row 0 north
    };
    // A product has ONE grade and ONE unit. Two sources that disagree about either are not
    // two views of a field, they are two fields -- and averaging them produces a number with
    // no meaning that no later stage can detect. So the first source sets the contract and
    // the rest are held to it, loudly. This is the composition-time half of the frame contract
    // the AST already enforces on every edge.
    bool Add(std::shared_ptr<DomainSource> s) {
        if (!s) return false;
        if (m_sources.empty()) {
            m_grade = s->GradeSig();
            m_chan = s->Channels();
            m_unit = s->Unit();
        } else if (s->GradeSig() != m_grade || s->Channels() != m_chan) {
            Log("[compose] %s REFUSED: grade %u/%u channels %u/%u -- a product carries one "
                "grade and one unit; mixing them makes a number nothing downstream can catch",
                s->Name(), s->GradeSig(), m_grade, s->Channels(), m_chan);
            return false;
        }
        // M9l: and the UNIT half, which this message has promised since it was written while
        // checking only the grade. A source must arrive already in the product's frame -- the
        // compositor does not convert, because by here the provenance needed to convert
        // honestly (which file, which datum) is gone. Normalize() is the door.
        {
            std::string why;
            if (!m_unit.AcceptsFrom(s->Unit(), &why)) {
                Log("[compose] %s REFUSED: %s -- run it through Normalize() first (product is "
                    "%s)", s->Name(), why.c_str(), m_unit.Describe().c_str());
                return false;
            }
            if (!s->Unit().IsCanonical()) {
                Log("[compose] %s REFUSED: %s is convertible but UNNORMALIZED -- the "
                    "normalization stage is not optional", s->Name(),
                    s->Unit().Describe().c_str());
                return false;
            }
        }
        m_sources.push_back(std::move(s));
        return true;
    }
    uint8_t Grade() const { return m_grade; }
    uint32_t Channels() const { return m_chan; }
    const UnitSpec& Unit() const { return m_unit; }
    size_t SourceCount() const { return m_sources.size(); }

    // M9m: ONE texel's worth of composition -- priority first, then weighted average within the
    // winning priority. Factored out of ComposePage so that CompositeSource (a compositor wearing
    // the source interface, which is what makes trees possible) answers a point query through the
    // exact same code that fills a page. Two implementations would drift, and the drift would
    // show up only where a composite fed another composite.
    // M9am: ONE STEP OF "OVER", as a function. The tile-wise compose in TileTree.h runs this on
    // bytes already on disk; the point-wise compose below runs it on fresh samples. Two loops,
    // one kernel -- the alternative is the drift that shows up only where a composite feeds a
    // composite.
    static void OverStep(double acc[4], double& cov, const float* c, float weight,
                         uint32_t chan) {
        const double w = (std::min)(1.0, double(weight));
        for (uint32_t k = 0; k < chan && k < 4; ++k) acc[k] += (double(c[k]) - acc[k]) * w;
        cov += (1.0 - cov) * w;
    }
    static void OverFinish(const double acc[4], double cov, uint32_t chan, DomainValue& out) {
        for (uint32_t k = 0; k < chan && k < 4; ++k) out.c[k] = float(acc[k] / cov);
        out.weight = float(cov);
    }

    bool SampleBlended(const DomainQuery& q, DomainValue& out) const {
        out = DomainValue{};
        if (m_blend == Blend::LayeredOver) {
            // Sequential over, in ADD order -- so a caller must add bottom-to-top or the law
            // ends up under the grid it is supposed to overrule. Starting the accumulator at
            // zero (rather than at "absent") is deliberate: it is what SampleHeightStack does,
            // so a lone partial-weight layer pulls toward zero in both paths identically.
            double acc[4] = {0, 0, 0, 0}, cov = 0.0;
            for (const auto& s : m_sources) {
                DomainValue v;
                if (!s->SampleAt(q, v) || v.weight <= 0.0f) continue;
                OverStep(acc, cov, v.c, v.weight, m_chan);
            }
            if (cov <= 0.0) return false;   // NODATA: absent, not zero. The caller fills.
            // UN-PREMULTIPLY. Accumulating from zero is compositing over TRANSPARENT BLACK: a
            // texel only 30% covered would come out as 0.3 x its value, i.e. dragged toward sea
            // level by the absence beneath it. Dividing by the accumulated coverage removes that
            // phantom layer and leaves "the value, known with 30% confidence", which is the only
            // reading that survives being mipped or being blended again upstream.
            //
            // This does NOT move the bed: its bottom layer (ETOPO) covers the globe at w=1, so
            // cov is 1 everywhere and the division is by one. It matters exactly where the old
            // form was silently wrong -- a product whose stack does not reach the ground.
            OverFinish(acc, cov, m_chan, out);
            return true;
        }
        int bestPri = INT32_MIN;
        double acc[4] = {0, 0, 0, 0}, wsum = 0.0;
        for (const auto& s : m_sources) {
            DomainValue v;
            if (!s->SampleAt(q, v) || v.weight <= 0.0f) continue;
            const int pri = s->Priority();
            if (pri < bestPri) continue;
            if (pri > bestPri) {   // a finer source appeared: discard the coarser mix
                bestPri = pri;
                wsum = 0.0;
                acc[0] = acc[1] = acc[2] = acc[3] = 0.0;
            }
            for (uint32_t c = 0; c < m_chan && c < 4; ++c) acc[c] += v.weight * v.c[c];
            wsum += v.weight;
        }
        if (wsum <= 0.0) return false;
        for (uint32_t c = 0; c < m_chan && c < 4; ++c) out.c[c] = float(acc[c] / wsum);
        out.weight = float((std::min)(1.0, wsum));
        return true;
    }

    // For the printed tree -- a composite reports its children.
    const DomainSource* SourceAt(size_t i) const {
        return i < m_sources.size() ? m_sources[i].get() : nullptr;
    }

    // Compose one page. `out` is width*height*channels, `cov` is width*height. Returns the
    // number of texels that got any coverage -- zero means "do not allocate this page", which
    // is the ingest rule arriving at the other end of the pipeline.
    // nodata: what an UNCOVERED texel gets. It is a parameter, not a constant zero, because
    // zero is a legal value for nearly every field this composes -- a bed at 0 m is sea level,
    // a current at 0 m/s is slack water -- so filling holes with it makes absence
    // indistinguishable from data and there is no way to recover the difference later. `cov`
    // remains the authority; this is what lands in the texel for consumers that only read the
    // value plane.
    uint32_t ComposePage(const PageAddr& addr, const PageGeo& geo, uint32_t width,
                         uint32_t height, uint32_t channels, std::vector<float>& out,
                         std::vector<float>& cov, double unixT = 0.0, double depthM = 0.0,
                         float nodata = 0.0f) const {
        out.assign(size_t(width) * height * channels, nodata);
        cov.assign(size_t(width) * height, 0.0f);
        const double mpt = m_ladder.MetersPerTexel(addr.level);
        uint32_t covered = 0;
        for (uint32_t y = 0; y < height; ++y) {
            for (uint32_t x = 0; x < width; ++x) {
                DomainQuery q;
                q.lon = geo.lon0 + x * geo.dLon;
                if (geo.mercator) {
                    // Inverse Web Mercator: lat = 2*atan(exp(pi*(1 - 2y))) - pi/2.
                    const double my = geo.lat0 + y * geo.dLat;
                    const double t = 3.14159265358979 * (1.0 - 2.0 * my);
                    q.lat = (2.0 * std::atan(std::exp(t)) - 1.57079632679490) * 57.2957795130823;
                } else {
                    q.lat = geo.lat0 + y * geo.dLat;
                }
                q.depthM = depthM;
                q.unixT = unixT;
                q.groundM = mpt;

                DomainValue blended;
                if (!SampleBlended(q, blended)) continue;   // absent: leave zero, count nothing
                const size_t o = (size_t(y) * width + x) * channels;
                for (uint32_t c = 0; c < channels && c < 4; ++c) out[o + c] = blended.c[c];
                cov[size_t(y) * width + x] = blended.weight;
                ++covered;
            }
        }
        return covered;
    }

private:
    LevelLadder m_ladder;
    std::vector<std::shared_ptr<DomainSource>> m_sources;
    uint8_t m_grade = 0;
    UnitSpec m_unit;   // the product frame: set by the first source, enforced on every other
    Blend m_blend = Blend::PriorityAverage;
    uint32_t m_chan = 0;
};

}   // namespace ga
