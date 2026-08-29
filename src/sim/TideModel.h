// ================================================================================================
//  TideModel - M1: the metronome. Water level as a pure function of time.
//
//  h(t) = mean + sum_i A_i * cos(omega_i * (t - epoch) + phi_i)        [metres above MLLW]
//
//  The coefficients come from harvester/harvest_tides.py: NOAA CO-OPS harmonic-constituent
//  FREQUENCIES (37 per station, exact) with amplitude/phase least-squares-fitted to one year of
//  NOAA's own official hourly predictions. That makes the engine's tide match the official tables
//  by construction (fit residuals are millimetres) while the runtime stays a stateless sum of
//  cosines -- no astronomy, no network, no state. Nodal drift costs a few cm/year outside the fit
//  year; refitting from cache is one script run.
//
//  The official hourly series (the fit input) rides along for the on-screen overlay: the visible
//  M1 gate is our curve lying on top of NOAA's, live, while you scrub.
// ================================================================================================
#pragma once

#include <cstddef>
#include <string>
#include <vector>

namespace ga {

struct TideCoeff {
    double ampM = 0;        // metres
    double omegaRadS = 0;   // rad/s
    double phaseRad = 0;    // phase at the fit epoch
};

struct TideStation {
    std::string id, name;
    double lat = 0, lon = 0;
    double riverKm = -1;    // along-channel km from the entrance; < 0 = off the river profile
    double meanMllwM = 0;   // fitted mean, metres above MLLW
    double mllwMinusNavdM = -999;   // CO-OPS datum link: MLLW - NAVD88, m; -999 = no NAVD at
                                    // this station (M5c datum fetch)
    double fitRmsM = 0;
    double fitMaxM = 0;
    std::vector<TideCoeff> coeffs;

    // Official CO-OPS hourly predictions for the fit year (overlay + validation).
    double officialStartUnix = 0;
    double officialDtS = 3600;
    std::vector<float> officialM;
};

class TideModel {
public:
    // Loads data/tides/stations.json (+ the official_*.f32 sidecars next to it).
    bool Load(const std::string& jsonPath);

    size_t Count() const { return m_stations.size(); }
    const TideStation& S(size_t i) const { return m_stations[i]; }
    double EpochUnix() const { return m_epochUnix; }
    int Focus() const { return m_focus; }            // Newburyport when present, else 0
    double TotalRiverKm() const { return m_totalKm; }

    // Metres above MLLW, analytic, valid for any t (accuracy degrades slowly outside fit year).
    double Height(size_t i, double unixT) const;
    // Official prediction by linear interpolation; NaN outside the cached year.
    double Official(size_t i, double unixT) const;

private:
    std::vector<TideStation> m_stations;
    double m_epochUnix = 0;
    double m_totalKm = 0;
    int m_focus = 0;
};

}  // namespace ga
