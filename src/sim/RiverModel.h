// RiverModel - THE RIVERS, AS A MEASUREMENT (2026-10-21).
//
// Mark: "I would have river gauges and the gis for them also be a global composite. In theory can't
// we handle the intersection of rivers and the sea as well as the river gauge data for said river
// if we have the data?" Until this date the river that entered the shallow-water solver was a scene's
// hand settings -- a west-side face, gauge ids with a hand ratio ("01100500 01100000*1.08"), an
// upriver area constant, the stations' river kilometres from a per-place harvest. Now it is the water
// composite's river layer (harvest compose-water; data.river), every reach that meets the sea:
//
//     "sites":   [{"id", "name", "discharge_cms", "lat", "lon", ...}]       the gauges' discharges
//     "reaches": [{"id", "name",
//                  "line":   [[lon, lat, km, drainage_km2, tidal], ...],     mouth first, upstream
//                  "gauges": [{"id", "km", "drainage_km2"}],                the main stem's gauges
//                  "tides":  [{"id", "km"}],                                the tide stations on it
//                  "tidal_km": km}]                                         the head of tide, if known
//
// and the solver's river faces follow from it at any box anywhere (EntersBox): a river enters a
// domain where its reach's line, walked up from a mouth the domain holds, first leaves the box. Its
// discharge there is its gauge's times the drainage between (DischargeAt: the gauge nearest the face
// in drainage, the flow scaling with the area it drains); its tide is the stations on it bracketing
// the face's kilometre; and the reach above the face, which fills and drains through it, is its
// surface below high water, read from the one bed along the line (PrismArea).
#pragma once

#include <functional>
#include <string>
#include <vector>

namespace ga {

class TideModel;

constexpr int kMaxRiverFaces = 4;   // Swe.hlsl gFaceA[4] / gFaceB[4], SweCbData's faceA / faceB

class RiverModel {
public:
    struct Vertex { double lon, lat, km, areaKm2; bool tidal; };
    struct Gauge { std::string id; double km, areaKm2; };
    struct Tide { std::string id; double km; };
    struct Reach {
        std::string id, name;
        std::vector<Vertex> line;
        std::vector<Gauge> gauges;
        std::vector<Tide> tides;
        double tidalKm = -1.0;   // the head of tide as the hydrography marks it; < 0 = not said
    };
    struct Site { std::string id, name; double dischargeCms = -1.0; };
    // WHERE A RIVER ENTERS A BOX: its reach, the side of the box it crosses (0 N, 1 S, 2 W, 3 E, the
    // solver's side bits), the crossing (degrees) and its kilometre and drainage there.
    struct Crossing {
        int reach = -1;
        int side = -1;
        double lat = 0.0, lon = 0.0, km = 0.0, areaKm2 = 0.0;
    };

    bool Load(const std::string& path);
    const std::vector<Reach>& Reaches() const { return m_reaches; }
    const Reach& R(int i) const { return m_reaches[size_t(i)]; }
    const Site* SiteOf(const std::string& id) const;

    // Every reach whose mouth the box [lon0, lat0, lon1, lat1] holds and whose line leaves it: where it
    // first does, the largest drainage first, at most kMaxRiverFaces.
    std::vector<Crossing> EntersBox(double lat0, double lon0, double lat1, double lon1) const;
    // The discharge at a crossing (m^3/s): the reach's reporting gauge nearest it in drainage, times
    // the face's drainage over the gauge's. -1 when no gauge on the reach reports; `said` names it.
    double DischargeAt(const Crossing& c, std::string* said) const;
    // THE TIDAL PRISM'S AREA above a crossing (m^2): the river's surface below `highWaterNavd` from the
    // face up the line to the head of tide -- the hydrography's (tidalKm) where it says one, else the
    // line's end, the bed itself ending the water where it rises above high water. Each step of
    // `stepM` along the line the cross-section is walked out both ways from the line while the bed is
    // under the level (`bedAt(lat, lon)`, m NAVD88); its width times the step is the area. `said`
    // reports how far the water reached and why it stopped.
    double PrismArea(const Crossing& c, double highWaterNavd, const std::function<float(double, double)>& bedAt,
                     double planetR, double stepM, std::string* said) const;

private:
    std::vector<Reach> m_reaches;
    std::vector<Site> m_sites;
};

// THE RIVER FACES' CLOCKS: each face's tide, as a DEVIATION from the ocean's (the stations on its
// reach bracketing the face's kilometre, each in its own MLLW less its own mean -- the datum
// offsets cancel), and the transport it must carry (+ into the domain): the discharge less the prism
// the reach above it fills, q - A d(eta_face)/dt.
struct RiverFaceClock {
    struct Face {
        RiverModel::Crossing at;
        std::string name;
        int staA = -1, staB = -1;   // the tide stations bracketing the face's kilometre (model indices)
        double w = 0.0;             // the weight of staB
        double q = 0.0;             // the discharge, m^3/s
        double prismM2 = 0.0;       // the reach above the face that fills through it
    };
    int oceanSta = -1;       // the river's mouth station: the open water's clock (the first face's)
    bool tide = true;        // water.swe.riverTide: false zeroes the deviation (--swe-river-off)
    std::vector<Face> faces;

    double DEta(const TideModel& m, int f, double t) const;
    double Q(const TideModel& m, int f, double t) const;
    std::string Describe(const TideModel& m, int f, double t, double datumOff) const;
};
// The faces' clocks from the crossings: the tide stations the reach names (by id in the model), the
// river's mouth station as the ocean's, each face's discharge (`riverQ` > 0 replaces the first's).
RiverFaceClock BuildRiverClock(const TideModel& m, const RiverModel& rivers,
                               const std::vector<RiverModel::Crossing>& at, double riverQ);

}  // namespace ga
