#include "scene/TerrainLayer.h"

#include "hal/PixEvents.h"
#include "hal/Pipeline.h"
#include "scene/FieldSet.h"

#include <vector>

namespace ga {

void TerrainLayer::Init(Gpu& gpu, ShaderCompiler& sc, FieldSet& fields,
                        hal::RootSignature rootSig) {
    (void)fields;
    m_rootSig = rootSig;
    if (!m_bathy || !m_bathy->Ready()) throw std::runtime_error("TerrainLayer needs bathymetry");
    if (!BuildPso(gpu, sc)) throw std::runtime_error("terrain PSO failed");


    // Half-resolution grid geometry over a full-resolution normal texture: ~13 m quads carry the
    // shape, per-pixel normals carry the 6.8 m detail (the jetty riprap edge in particular).
    m_quadsX = static_cast<uint32_t>(m_bathy->Nx() / 2);
    m_quadsZ = static_cast<uint32_t>(m_bathy->Ny() / 2);
    Log("[terrain] %ux%u quads over %dx%d texels", m_quadsX, m_quadsZ, m_bathy->Nx(),
        m_bathy->Ny());
}

bool TerrainLayer::BuildPso(Gpu& gpu, ShaderCompiler& sc) {
    const std::wstring path = m_shaderDir + L"/Terrain.hlsl";
    hal::GraphicsPipelineDesc d;
    d.rootSig = m_rootSig;
    d.vs = sc.Compile(path, L"VsMain", L"vs_6_0");
    d.ps = sc.Compile(path, L"PsMain", L"ps_6_0");
    d.depthClip = TRUE;
    d.depthTest = true;
    d.depthWrite = true;   // reversed-Z GREATER, the default comparison
    return hal::Reload(m_pso, [&] { return hal::BuildGraphics(gpu, d, "terrain"); }, "terrain");
}

void TerrainLayer::ReloadShaders(Gpu& gpu, ShaderCompiler& sc) {
    BuildPso(gpu, sc);   // the reload law lives in BuildPso: swap only on success
}

void TerrainLayer::Render(const FrameContext& ctx) {
    (void)ctx;
    if (!m_pso || !renderEnabled) return;
    // M9aw: THIS LAYER HAS NO BED. The bank it used to build (BuildBedBank, M9k..M9an) was a
    // second GPU realization of the height stack the megatexture already carries, with no
    // caller since the planet mesh took over the ground; it is deleted. The ground is drawn by
    // GlobeMesh from the height pages. Until this layer reads those pages itself it draws
    // nothing, and says so once -- no fallback (M9an's rule stands).
    static bool warned = false;
    if (!warned) {
        warned = true;
        Log("[terrain] draws nothing: no bed of its own (the height pages are the bed; the "
            "planet mesh draws the ground)");
    }
    return;
}

}  // namespace ga
