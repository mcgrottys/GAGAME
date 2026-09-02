// ================================================================================================
//  GisStencil - M6i: surveyed ground truth, VECTOR-FIRST.
//
//  The GSHHG shorelines and WDBII rivers load as POLYLINES (EPSG:4326) and stay resident as
//  the authority. Two kinds of consumer:
//    * VECTOR: GisLayer renders the polylines as line geometry, projected onto the planet
//      in-shader from raw lon/lat -- crisp at every zoom, no raster ceiling. (The pattern the
//      compositor generalizes: one source, many realizations -- and the place where GA earns
//      its keep later: spherical polygons are chains of great arcs; incidence and clipping
//      are meets and joins.)
//    * RASTER: the harvester's parity-filled land masks (window R8 in the shared Mercator z14
//      frame + global R8 equirect) load as textures -- the sampled realization the per-pixel
//      land/sea CLASSIFIER consumes. Classification by survey, refined by heights, instead of
//      inferred from height sign alone.
// ================================================================================================
#pragma once

#include "core/Gpu.h"

#include <string>
#include <utility>
#include <vector>

namespace ga {

class GisStencil {
public:
    using Polyline = std::vector<std::pair<float, float>>;   // (lon, lat) degrees

    bool Load(const std::string& jsonPath);
    // M9ay: the raster realizations (window R8G8, global R8, fine edit R8G8) are gone. They
    // read the .raw parity fills; the classifier reads gis.landsea's own pages now
    // (GisMask + the mask page tenant). This class keeps the VECTORS for the overlay.
    bool Ready() const { return m_ready; }

    const std::vector<Polyline>& CoastNe() const { return m_coastNe; }
    const std::vector<Polyline>& RiversNe() const { return m_riversNe; }
    const std::vector<Polyline>& CoastGlobal() const { return m_coastGlob; }

private:
    static bool ReadBin(const std::string& path, std::vector<Polyline>& out);

    std::vector<Polyline> m_coastNe, m_riversNe, m_coastGlob;
    std::string m_dir;
    bool m_ready = false;
};

}  // namespace ga
