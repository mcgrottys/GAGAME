// The sky, as a product. Evidence that the Layer contract is cheap: a header, a ~40-line source
// file, and a shader. It registers no fields, owns one PSO, and issues one three-vertex draw.
#pragma once

#include "scene/Layer.h"

#include <string>

namespace ga {

class SkyLayer : public Layer {
public:
    void Configure(const std::wstring& shaderDir) { m_shaderDir = shaderDir; }

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

    // M13: THE GATE'S WINDOW HAS A SKY. The gate carried the destination's geometry and nothing
    // else, so above the destination's horizon the window showed the observer's own sky. A window
    // is a transform on the whole view: the same slab test the globe clips surfaces with runs in
    // the backdrop too, and the pixels inside it are answered in the destination's frame.
    //   boxRows/boxHalf/boxCentre  the box in the true camera frame (GlobeLayer::SetGate's own)
    //   skyRows                    this frame -> the DESTINATION's dome (its zenith on +y)
    //   sun                        the scene's one sun, said in that dome's frame
    // `on` false is the shipped pass, byte for byte.
    void SetGateWindow(const float boxRows[9], const float boxHalf[3], const float boxCentre[3],
                       const float skyRows[9], const float sun[3], bool on) {
        m_winOn = on;
        if (!on) return;
        for (int i = 0; i < 9; ++i) {
            m_winBox[i] = boxRows[i];
            m_winSky[i] = skyRows[i];
        }
        for (int i = 0; i < 3; ++i) {
            m_winHalf[i] = boxHalf[i];
            m_winC[i] = boxCentre[i];
            m_winSun[i] = sun[i];
        }
    }

private:
    bool BuildPso(Gpu& gpu, ShaderCompiler& sc);

    std::wstring m_shaderDir;
    hal::RootSignature m_rootSig = nullptr;
    hal::Pso m_pso;
    float m_rot[9] = {1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f};
    float m_sun[3] = {0.0f, 1.0f, 0.0f};
    bool m_winOn = false;
    float m_winBox[9] = {1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f};
    float m_winSky[9] = {1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f};
    float m_winHalf[3] = {0.0f, 0.0f, 0.0f};
    float m_winC[3] = {0.0f, 0.0f, 0.0f};
    float m_winSun[3] = {0.0f, 1.0f, 0.0f};
};

}  // namespace ga
