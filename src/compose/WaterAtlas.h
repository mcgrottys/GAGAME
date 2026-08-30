// ================================================================================================
//  WaterAtlas - M6v: THE WATER ATLAS. Heterogeneous water-parameter data through the SAME
//  atlas grammar as the textures -- sources declare CRS/coverage/resolution and answer in the
//  WGS84 exchange frame; channels are ordered stacks; realizations are cached quadtree tiles.
//
//  The value fiber is the constituent PHASOR (re, im), a Cl(2)+ spinor: the tide at x is
//      h(x, t) = msl(x) + sum_c Re[ P_c(x) * e^{i w_c (t - T0)} ]
//  -- five rotor fields (M2 S2 N2 K1 O1) whose time advance is pure rotor application, the
//  same statement the wave cascades made in M6t. "NOAA carries the state" becomes literal:
//  the constituent fields ARE the state; evaluation is stateless.
//
//  THE STACK (the user's rule: low-medium global default, HQ New England enhancement):
//    L0  equilibrium tide     analytic, global, LOW    -- zero download, honest structure
//    L1  EOT20                1/8 deg, global, MEDIUM  -- satellite-altimetry empirical atlas
//                             (Hart-Davis et al. 2021, CC-BY 4.0), re/im native
//    L2  NE station field     ~2 km, regional, HIGH    -- the 20 fitted CO-OPS stations
//                             (Merrimack chain, Gloucester, Boston, Salem, ...), phasor IDW,
//                             feathered coverage
//
//  THE EPOCH LADDER (the phase version of the M6d datum lesson): station fits carry phases at
//  the FIT EPOCH; global atlases carry GREENWICH LAGS. Mixing them raw smuggles an arbitrary
//  phase offset into the seam, exactly like the 19 cm MLLW offset once smuggled a permanent
//  ebb. The atlas re-references every global source per constituent by the amplitude-weighted
//  circular consensus of the stations it overlaps: delta_c = arg( sum_s P_s * conj(Q_s) ).
//  No astronomy is transcribed; the stations ARE the epoch authority.
// ================================================================================================
#pragma once

#include "compose/Compositor.h"
#include "sim/TideModel.h"

#include <array>
#include <memory>
#include <string>
#include <vector>

namespace ga {

class WaterAtlas {
public:
    static constexpr int kCon = 5;
    static const char* const kConName[kCon];       // M2 S2 N2 K1 O1
    static const double kConSpeedDegH[kCon];       // NOAA constituent speeds, deg/hour

    // Registers water.tide.<C> field channels on the compositor. tides = the fitted station
    // model (data/tides); waterDir = data/water (registry + optional EOT20 grids).
    bool Init(Compositor& comp, const TideModel& tides, const std::string& waterDir);

    bool Ready() const { return m_ready; }
    int ChannelId(int c) const { return m_channel[c]; }

    // The tide about local MSL at any point, any time (NAVD88 via MslNavd below).
    double Level(double latDeg, double lonDeg, double unixT, double groundResM = 500.0) const;
    // MSL height in NAVD88: station-IDW inside the NE survey, 0 (geoid ~ MSL) beyond it.
    double MslNavd(double latDeg, double lonDeg) const;

    // The per-constituent phasor from the composed stack (exchange-frame query).
    void Phasor(int c, double latDeg, double lonDeg, double groundResM, float out[2]) const;

    // Audit hooks (the selftest walks these).
    const TideModel* Tides() const { return m_tides; }
    double EpochCalibrationDeg(int c) const { return m_deltaDeg[c]; }
    int StationsUsed() const { return m_nStations; }
    bool HasEot20() const { return m_hasEot20; }

private:
    struct Impl;
    std::shared_ptr<Impl> m_impl;   // owns the source objects the compositor borrows
    const TideModel* m_tides = nullptr;
    int m_channel[kCon] = {-1, -1, -1, -1, -1};
    double m_deltaDeg[kCon] = {0, 0, 0, 0, 0};
    int m_nStations = 0;
    bool m_hasEot20 = false;
    bool m_ready = false;
};

// --selftest gate: geopositions round-trip, datum ladder sane, field reproduces the station
// fits inside the HQ region, phase-seam continuity at the coverage feather, phasor-plane
// interpolation preserves amplitude. Skips (true) when data/tides or data/water is absent.
bool RunWaterSelfTest();

}  // namespace ga
