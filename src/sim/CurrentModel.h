// ================================================================================================
//  CurrentModel - M3: water in motion, two ways.
//
//  1. TIDAL CURRENT STATIONS (ACT0816 Merrimack entrance & friends): NOAA's max/slack event
//     tables turned into smooth signed speed curves -- + flood, - ebb, cosine-blended between
//     events. Analytic, cached-forever data, evaluated for any t: the ebb/flood clock that
//     drives the sea view's jet and its wave steepening.
//
//  2. THE FIELD (GoMOFS surface currents resampled to a regular lat/lon grid): the Gulf of
//     Maine's actual circulation, input to the GA velocity-gradient pass and the gulf map view.
//     Optional -- the harvester probes for it and the engine degrades gracefully without it.
// ================================================================================================
#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace ga {

struct TidalEvent {
    double t = 0;      // unix seconds
    double ms = 0;     // signed speed, m/s: + flood, - ebb, 0 slack
};

struct TidalCurrentStation {
    std::string id, name;
    double floodDeg = 285, ebbDeg = 105;   // compass, direction the water flows TOWARD
    std::vector<TidalEvent> events;
};

struct CurrentField {
    int nx = 0, ny = 0;
    double lon0 = 0, lat0 = 0, dlon = 0, dlat = 0;
    std::vector<float> u, v;               // row-major, y = latitude rows; <= -900 marks land
    std::string source;
    bool Valid() const { return nx > 0 && !u.empty(); }
    // Bilinear where all four corners are water; nearest otherwise. False on land/outside.
    bool Sample(double lon, double lat, float& outU, float& outV) const;
};

class CurrentModel {
public:
    bool Load(const std::string& jsonPath);
    bool Ready() const { return !m_stations.empty() || m_field.Valid(); }

    size_t StationCount() const { return m_stations.size(); }
    const TidalCurrentStation& S(size_t i) const { return m_stations[i]; }
    int StationIndex(const char* id) const;

    // Signed speed in m/s (+ flood, - ebb) at any time; 0 outside the fetched window.
    double SignedSpeed(size_t sta, double unixT) const;

    const CurrentField& Field() const { return m_field; }

    // NDBC ADCP ground truth (buoy 44029 / NERACOOS A01).
    bool adcpValid = false;
    std::string adcpId;
    double adcpMs = 0, adcpTowardDeg = 0, adcpObsUnix = 0;

private:
    std::vector<TidalCurrentStation> m_stations;
    CurrentField m_field;
};

}  // namespace ga
