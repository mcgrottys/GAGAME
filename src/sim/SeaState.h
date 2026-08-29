// ================================================================================================
//  SeaState - M2: the forecast half of the wave system.
//
//  Loads data/sea/seastate.json (harvester/harvest_waves.py): hourly GFS-Wave wind + wave
//  PARTITIONS (wind sea + up to three swell trains) at a Gulf-of-Maine point, plus NDBC buoy
//  ground truth (44013 Hs/period/direction and its measured spectral density S(f)).
//
//  Its second job is the SPECTRUM PARAMETERIZATION, computed here exactly once and handed to the
//  GPU: each partition becomes {fp, energy scale, spreading}, where the energy scale is a NUMERIC
//  normalisation (integrate the unit-shape spectrum, scale so m0 hits the partition's Hs). The
//  same parameters drive the CPU-side S(f) used for the plot overlay and the model-Hs readout,
//  so "what the title bar claims" and "what the FFT synthesises" cannot drift apart.
//
//  Shapes: wind sea = JONSWAP (gamma 3.3); swell = narrow Gaussian bump. Spreading = cos^2s
//  about the propagation direction (s = 8 wind sea, 60 swell), normalised numerically.
//  Directions in the data are meteorological ("from"); everything the engine consumes is
//  converted here to a propagation-TOWARD unit vector in world (x = east, z = north).
// ================================================================================================
#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace ga {

// Mirrored in shaders/OceanCompute.hlsl (two float4s per partition).
struct PartParam {
    float fp = 0.1f;         // peak frequency, Hz
    float specScale = 0.0f;  // S(f) = specScale * shape(f); makes m0 match the partition Hs
    float sigF = 0.008f;     // Gaussian width for swell, Hz (unused for JONSWAP)
    float gamma = 3.3f;      // JONSWAP peak enhancement; 0 selects the Gaussian swell shape
    float dirToX = 1.0f;     // propagation-toward unit vector, world x (east)
    float dirToZ = 0.0f;     //                                world z (north)
    float spreadS = 8.0f;    // cos^2s spreading exponent
    float dirNorm = 1.0f;    // 1 / integral of cos^2s over theta, so D integrates to 1
};

struct SeaPartition {
    double hs = 0, tp = 0, fromDeg = 0;
    bool windsea = false;
};

struct SeaHour {
    double fh = 0;               // forecast hour offset from the cycle (not necessarily hourly)
    double windMs = 0, windFromDeg = 0;
    double combinedHs = 0, combinedTp = 0, combinedFromDeg = 0;
    std::vector<SeaPartition> parts;
};

struct BuoyObs {
    std::string id;
    double obsUnix = 0;
    double hs = 0, dpd = 0, apd = 0, mwd = 0, wspd = 0;
    std::vector<float> specFreqHz;      // measured spectral density, when the buoy provides it
    std::vector<float> specDens;        // m^2/Hz
    bool valid = false;
};

class SeaState {
public:
    bool Load(const std::string& jsonPath);
    bool Ready() const { return !m_hours.empty(); }

    double CycleUnix() const { return m_cycleUnix; }
    const std::string& CycleLabel() const { return m_cycleLabel; }
    const BuoyObs* Buoy(const char* id) const;

    // Nearest forecast entry for a sim time (entries may be several hours apart; clamped).
    int HourIndex(double simUnix) const;
    const SeaHour& Hour(int idx) const { return m_hours[idx]; }

    // The partition parameterization the GPU consumes, for a given hour. Returns count (<= 4).
    int BuildParams(int hourIdx, PartParam out[4]) const;

    // One partition from (Hs, Tp, direction-from): the normalisation lives here so forecast
    // partitions and sandbox overrides (--storm) go through identical math.
    static PartParam MakePartition(double hs, double tp, double fromDeg, bool windsea);

    // CPU-side S(f) from the same parameters (plot overlay + Hs readout).
    static double SpectrumAt(const PartParam* parts, int n, double fHz);
    static double SignificantHeight(const PartParam* parts, int n);

private:
    std::vector<SeaHour> m_hours;
    std::vector<BuoyObs> m_buoys;
    double m_cycleUnix = 0;
    std::string m_cycleLabel;
};

}  // namespace ga
