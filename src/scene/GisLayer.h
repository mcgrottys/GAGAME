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
#include "compose/GisStencil.h"
#include "scene/Layer.h"

#include <string>

namespace ga {

class GisLayer : public Layer {
public:
    void Configure(const std::wstring& shaderDir, const GisStencil* gis) {
        m_shaderDir = shaderDir;
        m_gis = gis;
    }

    const char* Name() const override { return "gis"; }
    void Init(Gpu& gpu, ShaderCompiler& sc, FieldSet& fields,
              ID3D12RootSignature* rootSig) override;
    void ReloadShaders(Gpu& gpu, ShaderCompiler& sc) override;
    void Render(const FrameContext& ctx) override;

    void SetComposed(const ComposedSurfaceCb& cs) { m_cs = cs; }
    bool enabled = false;   // --stencil

private:
    // Mirrors GisCb in GisVec.hlsl (composed rows + one color row).
    struct GisCbData {
        ComposedSurfaceCb cs;
        float color[4];
    };
    struct Batch {
        GpuBuffer buf;      // float2 lon/lat, two per segment
        uint32_t verts = 0;
        float color[4];     // rgb + lift m
    };
    bool BuildPso(Gpu& gpu, ShaderCompiler& sc);
    static Batch MakeBatch(Gpu& gpu, const std::vector<GisStencil::Polyline>& lines,
                           const wchar_t* name, float r, float g, float b, float lift);

    std::wstring m_shaderDir;
    const GisStencil* m_gis = nullptr;
    ID3D12RootSignature* m_rootSig = nullptr;
    Com<ID3D12PipelineState> m_pso;
    Batch m_coast, m_rivers, m_global;
    ComposedSurfaceCb m_cs{};
};

}  // namespace ga
