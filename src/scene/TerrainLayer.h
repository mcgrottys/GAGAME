// ================================================================================================
//  TerrainLayer - M5: the CUDEM surface as geometry. One heightfield texture, one SV_VertexID
//  grid, true vertical scale; the jetties and the coast are simply what the data says they are.
// ================================================================================================
#pragma once

#include "scene/Layer.h"
#include "sim/BathyModel.h"

#include <string>

namespace ga {

class TerrainLayer : public Layer {
public:
    void Configure(const std::wstring& shaderDir, const BathyModel* bathy) {
        m_shaderDir = shaderDir;
        m_bathy = bathy;
    }

    const char* Name() const override { return "terrain"; }
    void Init(Gpu& gpu, ShaderCompiler& sc, FieldSet& fields,
              ID3D12RootSignature* rootSig) override;
    void ReloadShaders(Gpu& gpu, ShaderCompiler& sc) override;
    void Render(const FrameContext& ctx) override;

    float waterNavd = 0.0f;    // current water level in the terrain's datum, set per frame
    uint32_t HeightSrv() const { return m_tex.srv; }
    GpuTexture& HeightTex() { return m_tex; }   // M5c: the SWE solver reads the bed directly

private:
    bool BuildPso(Gpu& gpu, ShaderCompiler& sc);

    struct TerrainCbData {
        float geo[4];
        uint32_t srv[4];
        float params[4];
    };

    std::wstring m_shaderDir;
    const BathyModel* m_bathy = nullptr;
    ID3D12RootSignature* m_rootSig = nullptr;
    Com<ID3D12PipelineState> m_pso;
    GpuTexture m_tex;
    uint32_t m_quadsX = 0, m_quadsZ = 0;
};

}  // namespace ga
