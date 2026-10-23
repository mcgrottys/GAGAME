// ================================================================================================
//  ExposureSource -- M9ba: THE SWELL SHADOW AS A TREE NODE.
//
//  Exposure is the fraction of the offshore sea that reaches a point: 1 in open water, ~0.12 in
//  the geometric shadow of a jetty, a dune, an island. It used to be a 160 x 160 CPU raster
//  marched over ONE copy of the Merrimack CUDEM and uploaded as a committed texture -- so it
//  existed inside one 18 km box and nowhere else, and the bank's rings whitecapped every other
//  harbour on the planet. It is a FIELD over the bed, so it is a DomainSource: sample = a
//  line-of-sight march toward the peak-wave source over the height STACK (whatever is finest
//  under the ray), graded blocking as before (an awash bar breaks and transmits; a 1.2 m wall
//  throws the deep shadow). Its identity is the program + its inputs: the height stack's
//  signature, the peak direction bucketed to 5 degrees, the water level bucketed to 0.25 m --
//  so a TileTree over it caches each (direction, level) answer on the NVMe on the shared
//  addresses, and the renderer reads it as a page tenant like the bed. When the bucket rolls,
//  the tree re-keys and the tenant drops its tiles; until the new ones land the page reads
//  "nothing resident" and the sea is exposed -- an honest absence, never a stale shadow.
// ================================================================================================
#pragma once

#include "compose/Compositor.h"
#include "compose/DomainSource.h"
#include "core/GaUnits.h"
#include "sim/GlobeModel.h"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <string>

namespace ga {

// The bucket the calling thread's current tile is being painted from. Namespace scope rather
// than a class member so it is one variable per THREAD, not one per source instance.
namespace expo_detail {
inline constexpr uint64_t kNoTile = ~0ull;
inline thread_local uint64_t t_tile = kNoTile;
// THE OPEN SEA, per thread: a cell whose ceiling the stack cannot rise above (Compositor::
// HeightStackCeiling), by (source, cell, grain, bucket); the last few, round robin.
struct OpenCell {
    uint64_t key = 0;
    bool open = false;
};
inline thread_local OpenCell t_open[8];
inline thread_local uint32_t t_openNext = 0;
}  // namespace expo_detail

class ExposureSource : public DomainSource {
public:
    ExposureSource(const Compositor* comp, int hgtChannel)
        : m_comp(comp), m_hgtCh(hgtChannel), m_unit(UnitSpec::Parse("fraction")) {}

    // The live inputs. Returns true when the BUCKET changed -- the identity moved and every
    // cached tile of the old identity is a different field. No valid direction = no field:
    // the march must never run toward a default (M7j: it walked west into the dunes from every
    // ocean cell and the whole sea read as deep shadow).
    bool Set(float dirX, float dirZ, double levelM, bool valid) {
        int32_t dirB = -1, lvlB = 0;
        if (valid && (dirX * dirX + dirZ * dirZ) > 1e-6f) {
            const double ang = std::atan2(double(dirZ), double(dirX)) * 180.0 / 3.14159265358979;
            dirB = static_cast<int32_t>(std::lround(ang / kDirBucketDeg));
            dirB = ((dirB % 72) + 72) % 72;
        }
        lvlB = static_cast<int32_t>(std::lround(levelM / kLevelBucketM));
        const uint64_t packed = (uint64_t(uint32_t(dirB)) << 32) | uint32_t(lvlB);
        const uint64_t prev = m_params.exchange(packed);
        return prev != packed;
    }
    bool Valid() const { return DirBucket(m_params.load()) >= 0; }
    // The live bucket word, (dirBucket << 32) | levelBucket: a field evaluated from this node is the
    // same field exactly while this is unchanged (ExposurePage keys its memo on it).
    uint64_t Params() const { return m_params.load(); }

    // ---- DomainSource
    const char* Name() const override { return "swell.exposure"; }
    SourceDomain Domain() const override { return SourceDomain::Raster; }
    uint8_t GradeSig() const override { return 1u; }   // a scalar
    uint32_t Channels() const override { return 1; }
    const UnitSpec& Unit() const override { return m_unit; }
    const char* NodeKind() const override { return "load"; }
    const char* Cadence() const override { return "on swell/level bucket"; }
    bool Footprint(double& lon0, double& lat0, double& lon1, double& lat1) const override {
        lon0 = -180.0; lat0 = -90.0; lon1 = 180.0; lat1 = 90.0;
        return true;
    }
    std::string Identity() const override {
        const uint64_t p = m_params.load();
        char b[96];
        snprintf(b, sizeof(b), "swell.exposure|v4 fan5x26 best-ray march %.0fm to %.0fm|dir%03d|lvl%+d|",
                 kStepMinM, kMaxRangeM, DirBucket(p), LevelBucket(p));
        std::string id = b;
        if (m_comp && m_hgtCh >= 0) {
            for (const HeightSource* hs : m_comp->ChannelAt(m_hgtCh).height) {
                id += hs->Info().name + ":" + hs->Info().structure + "|";
            }
        }
        return id;
    }

    // ---- THE TILE SCOPE (DomainSource::BeginTile). m_params is exchanged by the FRAME thread
    // every frame while this march runs per texel on a loader job for tens of milliseconds, so
    // without a snapshot a single tile could be marched from two different swell directions and
    // two different levels -- and then stored under ONE identity, because the tree's id is
    // computed when the tree is built. The window is real and named: Set() mutates the params
    // BEFORE main builds the replacement tree (main.cpp, the exposure roll), so a worker inside
    // the old tree paints with the new bucket.
    //
    // Snapshot per thread, answer every texel of the tile from it, and refuse the tile if the
    // bucket moved before it finished. Note EndTile compares VALUES, not exchange events: Set()
    // exchanges every frame but writes the same packed word unless the bucket really rolled, so
    // a still scene rejects nothing.
    //   A refused tile needs no Again of its own (F3's reopen, 2026-10-23): its refusal says the bucket
    // moved, the Set that moved it returned true, and the frame loop's roll then drops the tenant whole
    // (resMgr.Drop) -- every tile, the refused and those under it, is asked anew from the new tree.
    void BeginTile() const override { expo_detail::t_tile = m_params.load(); }
    bool EndTile() const override {
        const uint64_t painted = expo_detail::t_tile;
        expo_detail::t_tile = expo_detail::kNoTile;
        return painted == expo_detail::kNoTile || painted == m_params.load();
    }

    bool SampleAt(const DomainQuery& q, DomainValue& out) const override {
        out.weight = 0.0f;
        // Inside a tile: the snapshot. Outside one (the trace, a point query): live.
        const uint64_t p =
            (expo_detail::t_tile != expo_detail::kNoTile) ? expo_detail::t_tile : m_params.load();
        const int32_t dirB = DirBucket(p);
        if (!m_comp || m_hgtCh < 0 || dirB < 0) return false;
        const double level = LevelBucket(p) * kLevelBucketM;
        const double ang0 = dirB * kDirBucketDeg * 3.14159265358979 / 180.0;
        constexpr double kD2R = 3.14159265358979 / 180.0;
        const double lat0 = q.lat * kD2R, lon0 = q.lon * kD2R;
        // THE STACK, NOT THE TREE (2026-10-15): the tree's ray (HeightTree::MaxAlongRay) was 1.5x faster
        // where the tree was warm, but a node read on the frame thread (ExposurePage, a hull) and on the
        // loaders paints every cold tile its five 50 km rays cross -- at each place a travel reached, 3.2 s
        // a page on the loaders and 7.3 s of the frame thread's waits.
        out.c[0] = 1.0f;
        out.weight = 1.0f;
        // The step follows the texel: a 76 m texel does not need 13 m steps, a 9 m one does.
        const double stepM = (std::max)(kStepMinM, q.groundM * 0.35);
        // THE OPEN SEA (2026-10-15): where nothing the stack holds within the march's reach rises
        // past the water line by the 0.15 m that starts a shadow, every ray's excess is under it, its
        // transmission 1, and the answer the march would give is exactly 1 -- so it is given without the
        // march. The reach: the cell about this texel, out by the range, the sample's own box at the
        // step and at the texel, and a margin. Offshore this was 750 stack reads a texel, 3.2 s a page.
        s_lastOpen = s_openSea && OpenSea(q.lat, q.lon, q.groundM, stepM, level, p);
        if (s_lastOpen) return true;
        const float e0 = m_comp->SampleHeightStack(m_hgtCh, lat0, lon0, q.groundM);
        if (e0 > level) return true;   // land: exposed by convention; never sampled as water
        const int maxSteps = static_cast<int>(kMaxRangeM / stepM);
        // A FAN, not a ray -- and the BEST ray, not the mean. The sea has directional spread
        // (the wave-field spectrum's +-26 degrees, `wavefield`); a jettied channel takes
        // whatever part of that spread is aligned with it, because the waves that enter
        // refract along the channel and fill it (the Merrimack's standing waves stand a
        // kilometre inside the tips -- the user has ridden them on a clear day). The single
        // line of sight the old raster marched put the whole channel at the deep-shadow floor
        // once the survey edits made the jetties real walls; averaging a fan left it at 0.3.
        // So: five rays across the spread, each transmission weighted by its angular
        // distance from the peak, and the exposure is the MAXIMUM -- the channel is as open
        // as its most open direction. Diffraction proper is not modelled and not claimed.
        static const double kFanDeg[5] = {-26.0, -13.0, 0.0, 13.0, 26.0};
        static const double kFanW[5] = {0.6, 0.85, 1.0, 0.85, 0.6};
        double best = 0.0;
        for (int r = 0; r < 5; ++r) {
            const double ang = ang0 + kFanDeg[r] * kD2R;
            // Propagation direction (unit, world east/north); the march goes TOWARD the source.
            const double dirX = std::cos(ang), dirZ = std::sin(ang);
            // PHASE C5: the step's angle on the sphere at the march's own latitude (radians).
            const double dLat = -dirZ * stepM / GlobeModel::kR;
            const double dLon = -dirX * stepM / (GlobeModel::kR * std::cos(lat0));
            double lat = lat0, lon = lon0;
            double excess = 0.0;   // worst blocker height above the water line en route
            for (int s = 0; s < maxSteps; ++s) {
                lat += dLat;
                lon += dLon;
                const float e = m_comp->SampleHeightStack(m_hgtCh, lat, lon, stepM);
                if (e > -9000.0f) excess = (std::max)(excess, double(e) - level);
                if (excess > 1.2) break;   // a full wall already
            }
            double tr = 1.0;
            if (excess > 0.15) {
                // 0.15 m awash -> 0.5 transmission, down to the 0.12 deep-shadow floor by 1.2 m.
                const double t = (std::min)((excess - 0.15) / 1.05, 1.0);
                tr = 0.5 - 0.38 * t;
            }
            best = (std::max)(best, kFanW[r] * tr);
        }
        out.c[0] = static_cast<float>((std::min)(best, 1.0));
        return true;
    }

    // The CPU mirror for the hypervisor: the same answer the page carries, at ~76 m.
    float At(double latDeg, double lonDeg) const {
        DomainQuery q;
        q.lat = latDeg;
        q.lon = lonDeg;
        q.groundM = 76.0;
        DomainValue v;
        return SampleAt(q, v) ? v.c[0] : 1.0f;
    }

    static constexpr double kDirBucketDeg = 5.0;
    static constexpr double kLevelBucketM = 0.25;
    static constexpr double kStepMinM = 13.0;
    static constexpr double kMaxRangeM = 4000.0;
    // --tool exposure-ab: the shortcut on or off, the plant's slack on its limit, and whether the last
    // sample on this thread was answered open.
    static inline bool s_openSea = true;
    static inline float s_openSlack = 0.0f;
    static inline thread_local bool s_lastOpen = false;

private:
    static constexpr double kCellDeg = 0.05;
    bool OpenSea(double latDeg, double lonDeg, double groundM, double stepM, double level, uint64_t p) const {
        if (std::abs(latDeg) > 80.0) return false;
        const double cy = std::floor(latDeg / kCellDeg), cx = std::floor(lonDeg / kCellDeg);
        uint64_t key = 14695981039346656037ull;
        const auto mix = [&key](uint64_t v) {
            for (int b = 0; b < 8; ++b) key = (key ^ ((v >> (8 * b)) & 0xFFu)) * 1099511628211ull;
        };
        uint64_t g;
        std::memcpy(&g, &groundM, 8);
        mix(reinterpret_cast<uintptr_t>(this));
        mix(uint64_t(int64_t(cy)));
        mix(uint64_t(int64_t(cx)));
        mix(g);
        mix(p);
        for (const expo_detail::OpenCell& c : expo_detail::t_open) {
            if (c.key == key) return c.open;
        }
        const double marginM = kMaxRangeM + 2.0 * (std::max)(stepM, groundM) + 100.0;
        const double lat0 = cy * kCellDeg, lat1 = lat0 + kCellDeg;
        const double dLat = marginM / GlobeModel::kR * 180.0 / 3.14159265358979;
        const double cosMin = std::cos((std::min)(89.0, (std::max)(std::abs(lat0), std::abs(lat1)) + dLat) * 3.14159265358979 / 180.0);
        const double dLon = dLat / (std::max)(cosMin, 0.05);
        float hi = 0.0f;
        const float limit = float(level + 0.15) - 1e-3f + s_openSlack;
        const bool open = m_comp->HeightStackCeiling(m_hgtCh, lat0 - dLat, cx * kCellDeg - dLon, lat1 + dLat,
                                                     (cx + 1.0) * kCellDeg + dLon, limit, hi);
        expo_detail::t_open[expo_detail::t_openNext++ & 7u] = {key, open};
        return open;
    }
    static int32_t DirBucket(uint64_t p) { return static_cast<int32_t>(uint32_t(p >> 32)); }
    static int32_t LevelBucket(uint64_t p) { return static_cast<int32_t>(uint32_t(p & 0xFFFFFFFFu)); }

    const Compositor* m_comp = nullptr;
    int m_hgtCh = -1;
    UnitSpec m_unit;
    // (dirBucket << 32) | levelBucket; dirBucket = -1 (0xFFFFFFFF) when no direction is known.
    std::atomic<uint64_t> m_params{uint64_t(0xFFFFFFFFull) << 32};
};

}  // namespace ga
