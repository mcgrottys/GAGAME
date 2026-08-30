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
    // The raster realizations. The WINDOW mask is R8G8: r = land mask with data/gis/
    // edits.geojson baked over the survey, g = EDIT FLAG -- flagged texels are law, they
    // override the survey and the live-tide classifier alike (the jetties came back from
    // the sea by three hand-typed polygons). Needs the shared Mercator window frame.
    void BuildMasks(Gpu& gpu, double orgPxX, double orgPxY, double sizePx);
    bool Ready() const { return m_ready; }
    uint32_t MaskWinSrv() const { return m_maskWin.srv; }
    // M7f: the FINE edit mask -- edits.geojson rasterized at ~1 m over the edits' own bbox
    // (the 4096^2 window mask answers at 38 m texels; a 25 m jetty is one texel there).
    uint32_t MaskEditSrv() const { return m_maskEdit.Valid() ? m_maskEdit.srv : 0xFFFFFFFFu; }
    const float* EditBox() const { return m_editBox; }   // window-uv offset xy, scale zw
    uint32_t MaskGlobSrv() const { return m_maskGlob.srv; }

    const std::vector<Polyline>& CoastNe() const { return m_coastNe; }
    const std::vector<Polyline>& RiversNe() const { return m_riversNe; }
    const std::vector<Polyline>& CoastGlobal() const { return m_coastGlob; }

private:
    static bool ReadBin(const std::string& path, std::vector<Polyline>& out);

    std::vector<Polyline> m_coastNe, m_riversNe, m_coastGlob;
    std::string m_dir, m_maskNePath, m_maskGlobPath;
    uint32_t m_maskNeDim = 0, m_maskGw = 0, m_maskGh = 0;
    GpuTexture m_maskWin, m_maskGlob, m_maskEdit;
    float m_editBox[4] = {0, 0, 0, 0};
    bool m_ready = false;
};

}  // namespace ga
