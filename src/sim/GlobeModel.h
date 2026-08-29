// ================================================================================================
//  GlobeModel - M6: the planet's data. ETOPO 2022 relief decimated to an 8192x4096 equirect
//  int16 grid (~5 km/texel) plus the latest GFS-Wave global Hs / 10 m wind fields, all loaded
//  from harvest_globe.py's output. CPU-side sampling exists for the camera (ground picking and
//  minimum-altitude clamps); the GPU gets the same data as textures.
//
//  Planet frame: centre at the origin, y through the north pole, x through (0N, 0E),
//  z through (0N, 90E). Radius 6 371 000 m.
// ================================================================================================
#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace ga {

class GlobeModel {
public:
    static constexpr double kR = 6371000.0;

    bool Load(const std::string& jsonPath);
    // M6f: MOLA relief into the SAME fields (nx/ny/elev) so the whole relief pipeline --
    // texture init, mips, ElevAt camera clamps -- serves Mars unchanged. Earth-only fields
    // (waves, clouds, NE ring, wind) stay empty and their consumers gate on that.
    bool LoadMars(const std::string& jsonPath);
    bool Ready() const { return m_nx > 0; }

    int Nx() const { return m_nx; }
    int Ny() const { return m_ny; }
    const std::vector<int16_t>& Elev() const { return m_elev; }

    // M6d: the New England 15-arc-second relief window (~460 m) and the global 10 m wind
    // vector (u east / v north, m/s, row 0 = north).
    int NeNx() const { return m_nenx; }
    int NeNy() const { return m_neny; }
    double NeLon0() const { return m_neLon0; }
    double NeLat1() const { return m_neLat1; }
    double NeDLon() const { return m_neDlon; }
    double NeDLat() const { return m_neDlat; }
    const std::vector<int16_t>& NeElev() const { return m_neElev; }
    int WindNx() const { return m_wvnx; }
    int WindNy() const { return m_wvny; }
    double WindLat1() const { return m_wvlat1; }
    double WindLon1() const { return m_wvlon1; }
    double WindDLat() const { return m_wvdlat; }
    double WindDLon() const { return m_wvdlon; }
    const std::vector<float>& WindU() const { return m_windU; }
    const std::vector<float>& WindV() const { return m_windV; }

    // M6c: isobaric cloud fraction (%, level-major, row 0 = north).
    int CloudsNx() const { return m_cnx; }
    int CloudsNy() const { return m_cny; }
    int CloudsNz() const { return m_cnz; }
    const std::vector<float>& Clouds() const { return m_clouds; }
    const std::vector<float>& CloudAltsM() const { return m_cloudAlts; }
    const std::string& CloudsCycle() const { return m_cloudsCycle; }

    int WavesNx() const { return m_wnx; }
    int WavesNy() const { return m_wny; }
    double WavesLat1() const { return m_wlat1; }
    double WavesLon1() const { return m_wlon1; }
    double WavesDLat() const { return m_wdlat; }
    double WavesDLon() const { return m_wdlon; }
    const std::vector<float>& Hs() const { return m_hs; }
    const std::vector<float>& Wind() const { return m_wind; }
    const std::string& WavesCycle() const { return m_cycle; }

    // Bilinear relief sample, metres (negative under the sea). lon in degrees east.
    double ElevAt(double latDeg, double lonDeg) const;

    // Unit direction in the planet frame for a lat/lon (degrees).
    static void LatLonDir(double latDeg, double lonDeg, double out[3]);

private:
    int m_nx = 0, m_ny = 0;
    std::vector<int16_t> m_elev;
    int m_wnx = 0, m_wny = 0;
    double m_wlat1 = 90, m_wlon1 = 0, m_wdlat = 0.25, m_wdlon = 0.25;
    std::vector<float> m_hs, m_wind;
    std::string m_cycle;
    int m_cnx = 0, m_cny = 0, m_cnz = 0;
    std::vector<float> m_clouds, m_cloudAlts;
    std::string m_cloudsCycle;
    int m_nenx = 0, m_neny = 0;
    double m_neLon0 = 0, m_neLat1 = 0, m_neDlon = 0, m_neDlat = 0;
    std::vector<int16_t> m_neElev;
    int m_wvnx = 0, m_wvny = 0;
    double m_wvlat1 = 90, m_wvlon1 = 0, m_wvdlat = 0.5, m_wvdlon = 0.5;
    std::vector<float> m_windU, m_windV;
};

}  // namespace ga
