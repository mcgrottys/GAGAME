// ================================================================================================
//  TideLayer - M1's two views of the same analytic tide:
//
//    1. THE RIBBON: the lower Merrimack as a strip, x = along-channel distance (100 scene metres
//       per river km), height = the tide interpolated between the five harmonic stations,
//       vertically exaggerated. Scrub time fast and you SEE the tide wave propagate upriver --
//       the ~13-minute Newburyport lag and the hours to Riverside are real data, live.
//    2. THE CURVES: a screen-space overlay plotting each station's h(t) across the scrub window,
//       our analytic sum drawn solid, NOAA's official predictions drawn dashed underneath, plus
//       the 'now' cursor and the MLLW zero line. The dashed and solid curves lying on top of each
//       other IS the M1 gate, visible every frame.
//
//  Everything is evaluated in the vertex shader from constants pushed per frame; there is no
//  geometry, no state, and time is just a number. CPU keeps the doubles (absolute unix seconds
//  would drown fp32), shaders get window-relative phases.
// ================================================================================================
#pragma once

#include "scene/Layer.h"
#include "sim/TideModel.h"

#include <string>

namespace ga {

class TideLayer : public Layer {
public:
    static constexpr float kKmToSceneM = 100.0f;   // river km -> scene metres along +X
    static constexpr uint32_t kMaxStations = 8;
    static constexpr uint32_t kMaxCoeffs = 40;
    static constexpr uint32_t kOfficialSamples = 128;
    static constexpr uint32_t kQuadsX = 384, kQuadsZ = 24;
    static constexpr uint32_t kCurveVerts = 384;

    void Configure(const std::wstring& shaderDir, const TideModel* model, float exaggeration);

    const char* Name() const override { return "tide"; }
    void Init(Gpu& gpu, ShaderCompiler& sc, FieldSet& fields,
              hal::RootSignature rootSig) override;
    void ReloadShaders(Gpu& gpu, ShaderCompiler& sc) override;
    void Render(const FrameContext& ctx) override;

    // Call once per frame before RenderFrame. Fills both constant blocks from the model.
    void SetTime(double simUnix, double windowSec);

    double focusHeight = 0;    // m above MLLW at the focus station; main mirrors to waterLevel
    float halfWidthM = 110.0f;
    float contourStepM = 0.5f;

private:
    bool BuildPsos(Gpu& gpu, ShaderCompiler& sc);

    // Mirrored in shaders/TideRibbon.hlsl.
    struct RibbonCb {
        float rib[4];          // sceneLenM, halfWidthM, exagg, nRibbonStations
        float staX[2][4];
        float staH[2][4];
        float misc[4];         // mode (0 surface, 1 pylons), contourStep, focusX, unused
        float staColor[8][4];
    };
    // Mirrored in shaders/TideCurves.hlsl.
    struct CurvesCb {
        float win[4];          // windowSec, yMin, yMax, nStations
        float rect[4];         // ndc x0, y0(bottom), x1, y1(top)
        float misc[4];         // nCoeffMax, officialN, focusStation, unused
        float col[8][4];
        float mean[2][4];
        float ncoeff[2][4];
        float coeffs[kMaxStations * kMaxCoeffs][4];      // amp, omega, thetaAtWindowCentre, 0
        float official[kMaxStations * (kOfficialSamples / 4)][4];
    };

    const TideModel* m_model = nullptr;
    std::wstring m_shaderDir;
    float m_exagg = 60.0f;
    hal::RootSignature m_rootSig = nullptr;
    hal::Pso m_ribbonPso, m_curvesPso;

    RibbonCb m_ribbon{};
    CurvesCb m_curves{};
    uint32_t m_nRibbon = 0;    // stations with river_km >= 0, in river order
    uint32_t m_ribbonIdx[kMaxStations] = {};
    bool m_haveData = false;
    float m_yMin = -0.5f, m_yMax = 4.0f;
};

}  // namespace ga
