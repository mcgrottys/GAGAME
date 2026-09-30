// ================================================================================================
//  GulfLayer - M3: the Gulf of Maine map view.
//
//  Uploads the GoMOFS regular-grid current field once, runs the velocity-gradient GA pass over
//  it once (VelGrad.hlsl -- divergence to grade 0, vorticity to grade 2, Okubo-Weiss beside),
//  then draws the map each frame: speed ramp + eddy blooms tinted by vorticity sign + the
//  model's own land mask + the 44029 buoy ring.
//
//  Numeric gate: the field sampled at buoy 44029 versus its live ADCP observation, computed at
//  Init and surfaced in the title bar.
// ================================================================================================
#pragma once

#include "hal/Views.h"
#include "scene/Layer.h"
#include "sim/CurrentModel.h"

#include <string>

namespace ga {

class GulfLayer : public Layer {
public:
    // NERACOOS A01 / NDBC 44029, Massachusetts Bay.
    static constexpr double kBuoyLon = -70.566, kBuoyLat = 42.523;

    void Configure(const std::wstring& shaderDir, const CurrentModel* currents) {
        m_shaderDir = shaderDir;
        m_currents = currents;
    }

    const char* Name() const override { return "gulf"; }
    void Init(Gpu& gpu, ShaderCompiler& sc, FieldSet& fields,
              hal::RootSignature rootSig) override;

    void ReloadShaders(Gpu& gpu, ShaderCompiler& sc) override;
    void Render(const FrameContext& ctx) override;

    std::string validation;   // "44029 obs 0.09 m/s @090 | model 0.07 @102" for the title

private:
    struct GulfCbData {
        float panel[4];
        uint32_t srvs[4];
        float params[4];
        float geo[4];
        float marker[4];
    };

    bool BuildDrawPso(Gpu& gpu, ShaderCompiler& sc);
    void RunVelGrad(Gpu& gpu, ShaderCompiler& sc);

    std::wstring m_shaderDir;
    const CurrentModel* m_currents = nullptr;
    hal::RootSignature m_rootSig = nullptr;
    hal::Pso m_drawPso;

    hal::RootSignatureRef m_csRootSig;
    hal::Pso m_csPso;
    GpuTexture m_uvTex;                    // u, v, mask, 0 (RGBA32F)
    hal::ResourceRef m_mvTex, m_owTex;  // outputs of the GA pass (RGBA16F)
    uint32_t m_uvSrv = UINT32_MAX, m_mvSrv = UINT32_MAX, m_owSrv = UINT32_MAX;
    hal::Table m_csTable;   // [u0 uv field, u1 mv2, u2 okubo-weiss]

    GulfCbData m_cb{};
    float m_aspectWoverH = 1.0f;
    bool m_haveField = false;
};

}  // namespace ga
