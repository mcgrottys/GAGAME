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

#include <cmath>
#include <cstdint>
#include <limits>
#include <string>
#include <vector>

namespace ga {

struct TidalEvent {
    double t = 0;      // unix seconds
    double ms = 0;     // signed speed, m/s: + flood, - ebb, 0 slack
};

struct TidalCurrentStation {
    std::string id, name;
    double lat = NAN, lon = NAN;           // PHASE C3: its place -- the file's, else the scene's
    double floodDeg = NAN, ebbDeg = NAN;   // compass, direction the water flows TOWARD
    std::vector<TidalEvent> events;
};

struct CurrentField {
    int nx = 0, ny = 0;
    double lon0 = 0, lat0 = 0, dlon = 0, dlat = 0;
    std::vector<float> u, v;               // row-major, y = latitude rows; <= -900 marks land
    std::string source;
    double grainM = 0.0;                   // the model's own cell (the composite's grain_m); 0 = unsaid
    bool Valid() const { return nx > 0 && !u.empty(); }
    bool Holds(double lon, double lat) const {
        return Valid() && lon >= lon0 && lon <= lon0 + nx * dlon && lat >= lat0 && lat <= lat0 + ny * dlat;
    }
    // Bilinear where all four corners are water; nearest otherwise. False on land/outside.
    bool Sample(double lon, double lat, float& outU, float& outV) const;
};

class CurrentModel {
public:
    bool Load(const std::string& jsonPath);
    bool Ready() const { return !m_stations.empty() || !m_fields.empty(); }

    size_t Count() const { return m_stations.size(); }
    const TidalCurrentStation& S(size_t i) const { return m_stations[i]; }
    // PHASE C3: a station whose file says nothing of its place stands where the scene says.
    void Place(const std::string& id, double latDeg, double lonDeg);

    // Signed speed in m/s (+ flood, - ebb) at any time; 0 outside the fetched window.
    double SignedSpeed(size_t sta, double unixT) const;

    // THE FIELDS (a composite's `fields`, docs/FIDELITY.md; a file of one says `field`), finest
    // first. ONE RULE: a place is answered by the finest field whose box holds it -- FieldAt for a
    // reader that draws one grid about a place (the gulf map), SampleAt for a point (the finest
    // holding water there). The finest field anywhere answers nothing: in a composite it is another
    // region's (the coastal map's finest is the Chesapeake's, beside a scene at the Merrimack).
    const CurrentField& FieldAt(double lon, double lat) const {
        static const CurrentField none;
        for (const CurrentField& f : m_fields) {
            if (f.Holds(lon, lat)) return f;
        }
        return none;
    }
    size_t FieldCount() const { return m_fields.size(); }
    bool SampleAt(double lon, double lat, float& outU, float& outV) const;

    // NDBC ADCP ground truth (buoy 44029 / NERACOOS A01).
    bool adcpValid = false;
    std::string adcpId;
    double adcpMs = 0, adcpTowardDeg = 0, adcpObsUnix = 0;
    // Where the ADCP buoy stands: its file's `lat`/`lon`, else the scene's `stations` entry of its
    // id (Place); NaN = nowhere known (the gulf map draws no marker and no comparison).
    double adcpLat = std::numeric_limits<double>::quiet_NaN(), adcpLon = std::numeric_limits<double>::quiet_NaN();

private:
    std::vector<TidalCurrentStation> m_stations;
    std::vector<CurrentField> m_fields;
};

}  // namespace ga
