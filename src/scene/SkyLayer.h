// The sky, as a product. Evidence that the Layer contract is cheap: a header, a ~40-line source
// file, and a shader. It registers no fields, owns one PSO, and issues one three-vertex draw.
#pragma once

#include "hal/Gpu.h"
#include "scene/Layer.h"

#include <string>

namespace ga {

class SkyLayer : public Layer {
public:
    // `probe` (--sky-probe) also builds the two transmittance tables Probe() compares.
    void Configure(const std::wstring& shaderDir, bool probe = false) {
        m_shaderDir = shaderDir;
        m_probe = probe;
    }

    const char* Name() const override { return "sky"; }
    void Init(Gpu& gpu, ShaderCompiler& sc, FieldSet& fields,
              hal::RootSignature rootSig) override;
    void ReloadShaders(Gpu& gpu, ShaderCompiler& sc) override;
    void Render(const FrameContext& ctx) override;

    // M10: WHOSE sky. The dome is evaluated in its own world's frame: rows take a view ray from
    // the camera's frame into the sky's, and `sun` is the sun in the sky's frame. Identity and
    // the scene's sun (the default, and every run without a Droste link) reproduce the old pass
    // exactly; under realistic Droste lighting the camera sits inside a twisted level and the
    // backdrop is the ROOT's sky, turned by Q^L.
    void SetSkyFrame(const float rows[9], const float sun[3]) {
        for (int i = 0; i < 9; ++i) m_rot[i] = rows[i];
        for (int i = 0; i < 3; ++i) m_sun[i] = sun[i];
    }

    // M13: THE GATE'S WINDOW HAS A SKY -- the sky of the place its rays land in. A window is a
    // transform on the whole view, so the backdrop runs the same slab test the globe clips with,
    // and a pixel inside it marches the air from the far place: its zenith, the one sun as seen
    // from there, and its distance from the planet's centre, all said in this frame.
    // `on` false is the shipped pass, byte for byte.
    void SetGateWindow(const float boxRows[9], const float boxHalf[3], const float boxCentre[3],
                       const float up[3], float eyeRadiusM, const float sun[3], bool on) {
        m_winOn = on;
        if (!on) return;
        for (int i = 0; i < 9; ++i) m_winBox[i] = boxRows[i];
        for (int i = 0; i < 3; ++i) {
            m_winHalf[i] = boxHalf[i];
            m_winC[i] = boxCentre[i];
            m_winUp[i] = up[i];
            m_winSun[i] = sun[i];
        }
        m_winEyeR = eyeRadiusM;
    }

    // ---- THE AIR'S TABLE (M13, Atmosphere.hlsli / SkyLut.hlsl) -----------------------------
    // A property of the atmosphere alone -- what the scattering orders past the first add at a
    // height under a sun angle -- built once and the same for every ray on the planet. There is
    // no table of the VIEW (every ray marches the air from where it is) and none of the sun's
    // transmittance (a closed form). The renderer publishes the slot in the scene constants.
    uint32_t MultiScatterSrv() const { return m_msTex.srv; }
    void SetPlanetRadius(double planetR) { m_planetR = planetR; }
    // --sky-probe: the closed-form transmittance and the marched table it replaced, read back
    // and held against a brute-force integral and against published optical depths.
    void Probe(Gpu& gpu);

private:
    bool BuildPso(Gpu& gpu, ShaderCompiler& sc);
    bool BuildLutPsos(Gpu& gpu, ShaderCompiler& sc);
    void RunLuts(const FrameContext& ctx);

    std::wstring m_shaderDir;
    hal::RootSignature m_rootSig = nullptr;
    hal::Pso m_pso;
    float m_rot[9] = {1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f};
    float m_sun[3] = {0.0f, 1.0f, 0.0f};
    bool m_winOn = false;
    float m_winBox[9] = {1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f};
    float m_winHalf[3] = {0.0f, 0.0f, 0.0f};
    float m_winC[3] = {0.0f, 0.0f, 0.0f};
    float m_winUp[3] = {0.0f, 1.0f, 0.0f};
    float m_winSun[3] = {0.0f, 1.0f, 0.0f};
    float m_winEyeR = 6371000.0f;

    // The table, its UAV, and the kernel that fills it -- and, under --sky-probe only, the two
    // transmittance tables (float32, so the comparison is not the storage's).
    static constexpr uint32_t kTransW = 256, kTransH = 64;
    static constexpr uint32_t kMsW = 32, kMsH = 32;
    GpuTexture m_msTex, m_transTex, m_anaTex;
    uint32_t m_msUav = UINT32_MAX, m_transUav = UINT32_MAX, m_anaUav = UINT32_MAX;
    hal::RootSignatureRef m_lutRs;
    hal::Pso m_csMs, m_csTrans, m_csAna;
    bool m_lutStatic = false;   // the constant tables are built on the first frame
    bool m_probe = false;
    double m_planetR = 6371000.0;
};

}  // namespace ga
