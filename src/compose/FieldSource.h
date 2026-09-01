// ================================================================================================
//  FieldSource / FieldCompositor - M9i: composition generalized past imagery.
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
//  A LOADER IS NOT A SOURCE. FieldLoader decodes a file into tiles -- storage. A FieldSource
//  answers queries in the output's frame -- composition. RasterSource below adapts one into the
//  other, which is the only place the two ideas meet.
// ================================================================================================
#pragma once

#include "core/FieldLoader.h"
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

enum class FieldDomain : uint8_t { Point, Profile, Raster, Volume };

// Where a value is wanted. Depth and time are carried for every query even when a 2-D source
// ignores them: a compositor that had to know which fields a source reads could not mix
// domains, which is the whole point.
struct FieldQuery {
    double lon = 0.0, lat = 0.0;
    double depthM = 0.0;      // 0 = surface, positive downward
    double unixT = 0.0;
    double groundM = 1.0;     // the output texel's ground size -- a source may answer coarser
};

// Up to 4 channels, because a Cl(2) multivector is 4 and everything here is at most that.
struct FieldValue {
    float c[4] = {0, 0, 0, 0};
    float weight = 0.0f;      // 0 = no coverage. THE thing that makes absence composable.
};

// ================================================================================================
//  A source of field values, in any domain.
// ================================================================================================
class FieldSource {
public:
    virtual ~FieldSource() = default;
    virtual const char* Name() const = 0;
    virtual FieldDomain Domain() const = 0;
    virtual uint8_t GradeSig() const = 0;
    virtual uint32_t Channels() const = 0;
    // The value and weight here. False is identical to weight 0 and exists only for callers
    // that want to skip the copy.
    virtual bool SampleAt(const FieldQuery& q, FieldValue& out) const = 0;
    // Cheap rejection so a compositor can skip a source over a whole page without querying it.
    virtual bool MayCover(double lon0, double lat0, double lon1, double lat1) const {
        (void)lon0; (void)lat0; (void)lon1; (void)lat1;
        return true;
    }
    // Higher wins where two sources both claim full weight -- a 1.5 m survey over a 700 m
    // model. Equal priority blends by weight, which is what a feathered edge wants.
    virtual int Priority() const { return 0; }
};

// ================================================================================================
//  RASTER -- the adapter that turns a FieldLoader into a source. This is the only place the
//  storage idea and the composition idea meet.
// ================================================================================================
class RasterSource : public FieldSource {
public:
    RasterSource(std::unique_ptr<FieldLoader> loader, int priority = 0)
        : m_loader(std::move(loader)), m_priority(priority) {
        if (m_loader) CacheAll();
    }
    bool Valid() const { return m_loader != nullptr && !m_cache.empty(); }

    const char* Name() const override { return m_loader ? m_loader->Name() : "raster"; }
    FieldDomain Domain() const override { return FieldDomain::Raster; }
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

    bool SampleAt(const FieldQuery& q, FieldValue& out) const override {
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
        const int x = int(std::floor(fx + 0.5));
        const int y = int(std::floor(fy + 0.5));
        if (x < 0 || y < 0 || x >= int(g.width) || y >= int(g.height)) return false;
        const size_t k = (size_t(y) * g.width + x) * m_chan;
        float w = m_cov[size_t(y) * g.width + x];
        if (w <= 0.0f) return false;   // absence: the source simply is not here
        for (uint32_t c = 0; c < m_chan && c < 4; ++c) out.c[c] = m_cache[k + c];
        out.weight = w;
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
};

// ================================================================================================
//  POINT -- scattered observations. Inverse-distance within a radius, which is the same shape
//  the tide stations already use (StationFieldSource, IDW p=2), kept here so a buoy can join a
//  raster in one product rather than being assimilated by hand somewhere downstream.
// ================================================================================================
class PointSource : public FieldSource {
public:
    struct Obs {
        double lon, lat;
        float c[4];
    };
    PointSource(std::string name, uint8_t grade, uint32_t channels, double radiusDeg,
                int priority = 10)
        : m_name(std::move(name)), m_grade(grade), m_chan(channels), m_radius(radiusDeg),
          m_priority(priority) {}

    void Add(const Obs& o) { m_obs.push_back(o); }
    size_t Count() const { return m_obs.size(); }

    const char* Name() const override { return m_name.c_str(); }
    FieldDomain Domain() const override { return FieldDomain::Point; }
    uint8_t GradeSig() const override { return m_grade; }
    uint32_t Channels() const override { return m_chan; }
    int Priority() const override { return m_priority; }

    bool SampleAt(const FieldQuery& q, FieldValue& out) const override {
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
};

// ================================================================================================
//  FieldCompositor -- N sources of any domain into ONE page of a GA product.
//
//  The rule, and it is the imagery compositor's rule unchanged: highest PRIORITY that has any
//  coverage wins the pixel; equal priorities blend by weight; nothing anywhere means the texel
//  is absent, and an entirely absent page is never allocated. Sources are asked in the output's
//  frame, so mixing a 700 m model raster with a scattered buoy set needs no agreement between
//  them beyond both being asked the same question.
// ================================================================================================
class FieldCompositor {
public:
    void SetLadder(const LevelLadder& l) { m_ladder = l; }
    // A page's geography, EXACTLY -- not the anchor-linear approximation. That form
    // (BathyModel::kOrgLat with a frozen mPerLon) is declared valid only NEAR ITS ANCHOR and
    // is what every local water consumer shares; a planet-scale tree cannot use it, because a
    // metres-per-degree frozen at 42.8N is wrong by 40% at the equator and unbounded at the
    // pole. So a page states its own lat/lon extent and the compositor interpolates within it.
    struct PageGeo {
        double lon0 = 0.0, lat0 = 0.0;   // texel (0,0) CENTRE
        double dLon = 0.0, dLat = 0.0;   // degrees per texel; dLat < 0 = row 0 north
    };
    // A product has ONE grade and ONE unit. Two sources that disagree about either are not
    // two views of a field, they are two fields -- and averaging them produces a number with
    // no meaning that no later stage can detect. So the first source sets the contract and
    // the rest are held to it, loudly. This is the composition-time half of the frame contract
    // the AST already enforces on every edge.
    bool Add(std::shared_ptr<FieldSource> s) {
        if (!s) return false;
        if (m_sources.empty()) {
            m_grade = s->GradeSig();
            m_chan = s->Channels();
        } else if (s->GradeSig() != m_grade || s->Channels() != m_chan) {
            Log("[compose] %s REFUSED: grade %u/%u channels %u/%u -- a product carries one "
                "grade and one unit; mixing them makes a number nothing downstream can catch",
                s->Name(), s->GradeSig(), m_grade, s->Channels(), m_chan);
            return false;
        }
        m_sources.push_back(std::move(s));
        return true;
    }
    uint8_t Grade() const { return m_grade; }
    uint32_t Channels() const { return m_chan; }
    size_t SourceCount() const { return m_sources.size(); }

    // Compose one page. `out` is width*height*channels, `cov` is width*height. Returns the
    // number of texels that got any coverage -- zero means "do not allocate this page", which
    // is the ingest rule arriving at the other end of the pipeline.
    uint32_t ComposePage(const PageAddr& addr, const PageGeo& geo, uint32_t width,
                         uint32_t height, uint32_t channels, std::vector<float>& out,
                         std::vector<float>& cov, double unixT = 0.0,
                         double depthM = 0.0) const {
        out.assign(size_t(width) * height * channels, 0.0f);
        cov.assign(size_t(width) * height, 0.0f);
        const double mpt = m_ladder.MetersPerTexel(addr.level);
        uint32_t covered = 0;
        for (uint32_t y = 0; y < height; ++y) {
            for (uint32_t x = 0; x < width; ++x) {
                FieldQuery q;
                q.lon = geo.lon0 + x * geo.dLon;
                q.lat = geo.lat0 + y * geo.dLat;
                q.depthM = depthM;
                q.unixT = unixT;
                q.groundM = mpt;

                int bestPri = INT32_MIN;
                double acc[4] = {0, 0, 0, 0}, wsum = 0.0;
                for (const auto& s : m_sources) {
                    FieldValue v;
                    if (!s->SampleAt(q, v) || v.weight <= 0.0f) continue;
                    const int pri = s->Priority();
                    if (pri < bestPri) continue;
                    if (pri > bestPri) {   // a finer source appeared: discard the coarser mix
                        bestPri = pri;
                        wsum = 0.0;
                        acc[0] = acc[1] = acc[2] = acc[3] = 0.0;
                    }
                    for (uint32_t c = 0; c < channels && c < 4; ++c) acc[c] += v.weight * v.c[c];
                    wsum += v.weight;
                }
                if (wsum <= 0.0) continue;   // absent: leave zero, and count nothing
                const size_t o = (size_t(y) * width + x) * channels;
                for (uint32_t c = 0; c < channels && c < 4; ++c) {
                    out[o + c] = float(acc[c] / wsum);
                }
                cov[size_t(y) * width + x] = float((std::min)(1.0, wsum));
                ++covered;
            }
        }
        return covered;
    }

private:
    LevelLadder m_ladder;
    std::vector<std::shared_ptr<FieldSource>> m_sources;
    uint8_t m_grade = 0;
    uint32_t m_chan = 0;
};

}   // namespace ga
