// ================================================================================================
//  GisLayer - M6i: the survey vectors, rendered as vectors.
//
//  GSHHG shorelines and WDBII rivers draw as LINE GEOMETRY (GisVec.hlsl): raw lon/lat pairs
//  projected onto the planet in-shader through the shared composed rows. No raster ceiling,
//  no per-zoom rebuild -- the vector is the authority and this layer is merely one of its
//  realizations (the land-mask textures are another). Registered after the globe; depth-off,
//  horizon-culled, --stencil-gated.
// ================================================================================================
#pragma once

#include "compose/Compositor.h"
#include "compose/Exchange.h"
#include "compose/GisStencil.h"
#include "compose/VectorPack.h"
#include "scene/Layer.h"

#include <string>

namespace ga {

class GisLayer : public Layer {
public:
    void Configure(const std::wstring& shaderDir, const GisStencil* gis, Exchange* exchange,
                   const VectorPack* pack) {
        m_shaderDir = shaderDir;
        m_gis = gis;
        m_exchange = exchange;
        m_pack = pack;
    }

    // M6p: LOD by the wedge filter. main sets the view's ground-pixel size per frame; the
    // layer republishes its Exchange buffers only when the TOLERANCE BUCKET changes (x8
    // steps + hysteresis by construction), so orbit views draw thousands of segments, not
    // hundreds of thousands, from the SAME lossless pack.
    float tolMeters = 0.0f;

    const char* Name() const override { return "gis"; }
    void Init(Gpu& gpu, ShaderCompiler& sc, FieldSet& fields,
              hal::RootSignature rootSig) override;
    void ReloadShaders(Gpu& gpu, ShaderCompiler& sc) override;
    void Render(const FrameContext& ctx) override;

    bool enabled = false;   // --stencil

private:
    // Mirrors GisCb in GisVec.hlsl (one color row; the composed rows the vectors project
    // through are the renderer's one surface buffer, b2, since M12 step 4g).
    struct GisCbData {
        float color[4];
    };
    // Each batch is an EXCHANGE CHANNEL: the polylines publish once as lon/lat segment
    // buffers (the vector authority), and this layer -- or any other consumer, including an
    // exporter -- resolves them by name.
    struct Batch {
        std::string channel;
        float color[4];     // rgb + lift m
    };
    bool BuildPso(Gpu& gpu, ShaderCompiler& sc);
    Batch MakeBatch(Gpu& gpu, const std::vector<GisStencil::Polyline>& lines,
                    const char* channel, float r, float g, float b, float lift);
    void PublishAtTolerance(Gpu& gpu, float tol);

    std::wstring m_shaderDir;
    const GisStencil* m_gis = nullptr;
    Exchange* m_exchange = nullptr;
    const VectorPack* m_pack = nullptr;
    hal::RootSignature m_rootSig = nullptr;
    hal::Pso m_pso;
    Batch m_coast, m_rivers, m_global, m_structs;
    int m_coastCh = -1, m_riversCh = -1, m_globalCh = -1,   // Exchange ids for republish
        m_structsCh = -1;
    float m_bucket = -1.0f;
};

}  // namespace ga
