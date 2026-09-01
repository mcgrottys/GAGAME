// ================================================================================================
//  TerrainLayer - M5: the CUDEM surface as geometry. One heightfield texture, one SV_VertexID
//  grid, true vertical scale; the jetties and the coast are simply what the data says they are.
// ================================================================================================
#pragma once

#include "compose/Compositor.h"
#include "scene/Layer.h"
#include "compose/DomainSource.h"
#include "compose/HeightStackSource.h"
#include "core/GeoGridLoader.h"
#include "core/GradeField.h"
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
    // M6j: when the mesh-shader planet surface is active, this layer keeps its OTHER jobs
    // (the heightfield texture physics reads, the sea's bed) but stops rendering -- one
    // planet, one description on screen.
    bool renderEnabled = true;
    uint32_t HeightSrv() const { return m_tex.srv; }
    GpuTexture& HeightTex() { return m_tex; }   // M5c: the SWE solver reads the bed directly

    // ---- M9k: THE BED AS A GA OBJECT -------------------------------------------------------
    // The same ground, arriving the way every 2D source is supposed to: GA Load (a
    // FieldLoader over the harvester's grid), GA Compose (a RasterSource through the
    // DomainCompositor, coverage and all), and a DirectX sparse structure (a paged GradeBank,
    // reserved array, mip chain, residency map).
    //
    // Built ALONGSIDE m_tex on purpose, and proved equal to it before anything switches. The
    // bed is the most load-bearing texture in the engine -- SweSolver, Sea.hlsl, WaterBank and
    // Globe.hlsl all read it -- so swapping consumers on an unverified path would risk every
    // one of them at once for no way to tell which broke.
    GradeBank& BedBank() { return m_bedBank; }
    bool BedBankReady() const { return m_bedReady; }
    // Returns the worst |GA path - committed texture| in metres, or -1 if it could not run.
    double BuildBedBank(Gpu& gpu, const Compositor& comp, int heightChannel,
                        double mslToNavd88M, const char* datumProv);

    // M6i: the composed color channel -- filled by FillComposedCb in main, the SAME function
    // and constants the globe uses, so the two layers agree texel for texel.
    void SetComposed(const ComposedSurfaceCb& cs) { m_cs = cs; }

private:
    bool BuildPso(Gpu& gpu, ShaderCompiler& sc);

    // Mirrored in shaders/Terrain.hlsl (count float4 rows on BOTH sides after any edit).
    struct TerrainCbData {
        float geo[4];
        uint32_t srv[4];      // heightfield, quadsX, quadsZ
        float params[4];
        ComposedSurfaceCb cs; // M6i: the composed channels (8 rows)
    };

    std::wstring m_shaderDir;
    const BathyModel* m_bathy = nullptr;
    ID3D12RootSignature* m_rootSig = nullptr;
    Com<ID3D12PipelineState> m_pso;
    GpuTexture m_tex;
    GradeBank m_bedBank;
    bool m_bedReady = false;
    uint32_t m_quadsX = 0, m_quadsZ = 0;
    ComposedSurfaceCb m_cs{};   // zero until SetComposed: every channel reads "off"
};

}  // namespace ga
