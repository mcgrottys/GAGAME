// ================================================================================================
//  SeaGrid - THE GLOBE'S SEA STATE (2026-10-21, Mark: "harvest the global GFS-Wave partitions and
//  make absence read the nearest sea"). The world composite's sea state holds, beside its forecast
//  points, the model's every node: GFS-Wave's 0.25 degree grid, its partitions (the combined sea, the
//  wind sea, three swell trains) and its wind, at the cycle's thirteen forecast hours (harvest
//  compose-seastate, ComposeSeastate.WriteGrid; sea/grid.f16 beside seastate.json). Read in place, a
//  file mapping, half floats one plane after another: [hour][plane][row][col], row 0 at 90 N, column 0
//  at 0 E, the nodes' own (node-centred), NaN where the model has no value.
//
//  WHY: a coast the fidelity map does not paint borrowed the nearest forecast POINT's sea -- Cape
//  Hatteras synthesised the Gulf of Maine's swell (docs/REVIEW_2026-10-21.md, B1). Every place now
//  reads its own: SeaLayer synthesises the sea at the eye's own ground point, and the scale field
//  (sim/WaveScale.h, the globe's far sea) reads this grid's Hs at the same forecast hour, so the
//  reference and the field are one measurement at one time.
//
//  ABSENCE READS THE NEAREST SEA (B2). The model masks land, and at 0.25 degrees a coast's own water
//  falls inside that mask; the old law read "no opinion" there, and the sea there was the cascades'
//  reference -- another place's. A node with no value now reads the nearest node that has one, within
//  kReachNodes (the mask takes up to a node and a half from the sea at a coast, a cell's diagonal); past
//  that it has no opinion. Found once, at load, by a breadth-first walk from every node with sea over the
//  grid (longitude wraps), so a coastal water texel reads the offshore sea the bank's depth physics then
//  shoals and breaks -- deep water's value, the boundary condition those laws are written for.
// ================================================================================================
#pragma once

#include "sim/SeaState.h"

#include <cstdint>
#include <string>
#include <vector>

namespace ga {

class SeaGrid {
public:
    static constexpr int kReachNodes = 2;

    SeaGrid() = default;
    ~SeaGrid();
    SeaGrid(const SeaGrid&) = delete;
    SeaGrid& operator=(const SeaGrid&) = delete;

    // The composite's seastate.json: its `grid` object and the file it names (beside it). False when
    // the file carries no grid (a composite harvested before 2026-10-21) or it cannot be read.
    bool Load(const std::string& seastateJson);
    bool Ready() const { return m_data != nullptr; }

    double CycleUnix() const { return m_cycleUnix; }
    const std::string& CycleLabel() const { return m_cycleLabel; }
    // The forecast hour nearest simUnix (SeaState::HourIndex's law) and its offset from the cycle.
    int HourIndex(double simUnix) const;
    double HourFh(int hour) const { return (hour >= 0 && hour < int(m_fh.size())) ? m_fh[size_t(hour)] : 0.0; }

    // The grid's geometry, the gfswave convention GlobeModel's waves take (node-centred, 0..360).
    int Nx() const { return m_nx; }
    int Ny() const { return m_ny; }
    double Lat1() const { return m_lat1; }
    double Lon1() const { return m_lon1; }
    double DLat() const { return m_dlat; }
    double DLon() const { return m_dlon; }

    // One hour's significant wave height and 10 m wind over every node, absence read from the nearest
    // sea within reach and -1 past it (gfswave's own land value, which GlobeModel's readers keep).
    void HsPlane(int hour, std::vector<float>& out) const;
    void WindPlane(int hour, std::vector<float>& out) const;

    // The sea at a place in one hour: the four nodes about it, each read where it reads (itself, or its
    // nearest sea within reach), weighted bilinearly among those that have a value. A partition is
    // blended by its ENERGY: Hs^2 summed, its period and its direction (a unit vector) weighted by it.
    // False where no node about the place has a value within reach.
    bool HourAt(double latDeg, double lonDeg, int hour, SeaHour& out) const;
    // B2's walk for any node-centred grid closing the circle (row 0 north): per node itself where `wet`,
    // else its nearest wet node within `reach` (four neighbours, breadth first), else -1. GlobeModel's
    // own Hs (a composite without a grid) is filled by the same walk.
    static std::vector<int32_t> NearestWet(const std::vector<uint8_t>& wet, int nx, int ny, int reach, size_t* filled);
    // A place with no sea within reach, moved to the nearest node that has one (at any distance).
    bool NearestSea(double& latDeg, double& lonDeg) const;
    // Hs at a place as the scale field reads HsPlane (WaveScale::GridAt): bilinear over the four nodes,
    // among those with a value; -1 where none has one. The scale field's reference, so it is 1 there.
    double HsBilinear(double latDeg, double lonDeg, int hour) const;

private:
    float Val(int hour, int plane, int node) const;
    int PlaneOf(const char* name) const;
    void FillPlane(int hour, int plane, std::vector<float>& out) const;

    void* m_file = nullptr;
    void* m_map = nullptr;
    const uint16_t* m_data = nullptr;
    size_t m_nodes = 0;
    int m_nx = 0, m_ny = 0;
    double m_lat1 = 90.0, m_lon1 = 0.0, m_dlat = 0.25, m_dlon = 0.25;
    double m_cycleUnix = 0.0;
    std::string m_cycleLabel;
    std::vector<double> m_fh;
    std::vector<std::string> m_planes;
    std::vector<int32_t> m_src;   // per node: the node it reads (itself, or its nearest sea within reach); -1 none
    // The planes the readers name, found once by name (a file may carry them in any order).
    int m_pHs = -1, m_pTp = -1, m_pDir = -1, m_pWsHs = -1, m_pWsTp = -1, m_pWsDir = -1, m_pWind = -1, m_pWdir = -1;
    int m_pSwHs[3] = {-1, -1, -1}, m_pSwTp[3] = {-1, -1, -1}, m_pSwDir[3] = {-1, -1, -1};
};

}  // namespace ga
