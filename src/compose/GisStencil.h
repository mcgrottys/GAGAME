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
    void BuildMasks(Gpu& gpu);   // the raster realizations (R8 textures + SRVs)
    bool Ready() const { return m_ready; }
    uint32_t MaskWinSrv() const { return m_maskWin.srv; }
    uint32_t MaskGlobSrv() const { return m_maskGlob.srv; }

    const std::vector<Polyline>& CoastNe() const { return m_coastNe; }
    const std::vector<Polyline>& RiversNe() const { return m_riversNe; }
    const std::vector<Polyline>& CoastGlobal() const { return m_coastGlob; }

private:
    static bool ReadBin(const std::string& path, std::vector<Polyline>& out);

    std::vector<Polyline> m_coastNe, m_riversNe, m_coastGlob;
    std::string m_dir, m_maskNePath, m_maskGlobPath;
    uint32_t m_maskNeDim = 0, m_maskGw = 0, m_maskGh = 0;
    GpuTexture m_maskWin, m_maskGlob;
    bool m_ready = false;
};

}  // namespace ga
