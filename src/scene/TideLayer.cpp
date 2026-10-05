#include "scene/TideLayer.h"

#include "hal/PixEvents.h"
#include "hal/Pipeline.h"
#include "scene/FieldSet.h"

#include <cmath>

namespace ga {

namespace {
// One colour per station by its order in the tide file (PHASE C3: no station is named here),
// bright enough to read after the ACES curve: cyan, orange, violet, green, pink, grey, ...
constexpr float kPalette[8][4] = {
    {0.30f, 1.80f, 1.90f, 1}, {2.30f, 1.10f, 0.15f, 1}, {1.30f, 0.55f, 2.20f, 1},
    {0.45f, 1.90f, 0.55f, 1}, {2.20f, 0.55f, 1.20f, 1}, {0.85f, 0.95f, 1.05f, 1},
    {1.50f, 1.50f, 0.30f, 1}, {1.00f, 1.00f, 1.00f, 1},
};
constexpr double kTwoPi = 6.283185307179586476925;
}  // namespace

void TideLayer::Configure(const std::wstring& shaderDir, const TideModel* model,
                          float exaggeration) {
    m_shaderDir = shaderDir;
    m_model = model;
    m_exagg = exaggeration;
}

void TideLayer::Init(Gpu& gpu, ShaderCompiler& sc, FieldSet& fields,
                     hal::RootSignature rootSig) {
    (void)fields;
    m_rootSig = rootSig;
    if (!m_model || m_model->Count() == 0) throw std::runtime_error("TideLayer needs a TideModel");
    if (!BuildPsos(gpu, sc)) throw std::runtime_error("tide PSOs could not be created");

    // Static setup: which stations lie on the river profile (already in river order in the JSON),
    // the palette, and the curve plot's vertical range from the coefficient sums.
    const size_t n = std::min<size_t>(m_model->Count(), kMaxStations);
    m_nRibbon = 0;
    double yMin = 0.0, yMax = 1.0;
    for (size_t i = 0; i < n; ++i) {
        const TideStation& s = m_model->S(i);
        double range = 0;
        for (const TideCoeff& c : s.coeffs) range += c.ampM;
        yMin = std::min(yMin, s.meanMllwM - range);
        yMax = std::max(yMax, s.meanMllwM + range);
        if (s.riverKm >= 0 && m_nRibbon < kMaxStations) {
            m_ribbonIdx[m_nRibbon++] = static_cast<uint32_t>(i);
        }
        for (int c = 0; c < 4; ++c) m_curves.col[i][c] = kPalette[i][c];
    }
    m_yMin = static_cast<float>(yMin - 0.3);
    m_yMax = static_cast<float>(yMax + 0.3);
    Log("[tide] layer: %u river stations on the ribbon, plot range [%.2f, %.2f] m MLLW",
        m_nRibbon, m_yMin, m_yMax);
}

bool TideLayer::BuildPsos(Gpu& gpu, ShaderCompiler& sc) {
    auto makePso = [&](const wchar_t* file, bool lines, bool depth, const char* tag) {
        const std::wstring path = m_shaderDir + L"/" + file;
        hal::GraphicsPipelineDesc d;
        d.rootSig = m_rootSig;
        d.vs = sc.Compile(path, L"VsMain", L"vs_6_0");
        d.ps = sc.Compile(path, L"PsMain", L"ps_6_0");
        // cull stays NONE: a displaced ribbon shows its back faces at grazing angles, same as
        // vqview's water. Reversed-Z everywhere: GREATER (the default), cleared to 0.
        d.depthClip = TRUE;
        d.depthTest = depth;
        d.depthWrite = depth;
        d.topology = lines ? D3D12_PRIMITIVE_TOPOLOGY_TYPE_LINE
                           : D3D12_PRIMITIVE_TOPOLOGY_TYPE_TRIANGLE;
        return hal::BuildGraphics(gpu, d, tag);
    };
    // The ribbon and the curves swap together or not at all.
    hal::ReloadSet set;
    if (!set.Add(m_ribbonPso, makePso(L"TideRibbon.hlsl", false, true, "tide.ribbon"))) {
        return false;
    }
    if (!set.Add(m_curvesPso, makePso(L"TideCurves.hlsl", true, false, "tide.curves"))) {
        return false;
    }
    set.Commit();
    return true;
}

void TideLayer::ReloadShaders(Gpu& gpu, ShaderCompiler& sc) {
    if (!BuildPsos(gpu, sc)) Log("[tide] reload failed; keeping the previous PSOs");
}

void TideLayer::SetTime(double simUnix, double windowSec) {
    if (!m_model) return;
    const size_t n = std::min<size_t>(m_model->Count(), kMaxStations);
    const double epoch = m_model->EpochUnix();

    // ---- ribbon: interpolated station heights, scene-space station positions
    for (uint32_t r = 0; r < m_nRibbon; ++r) {
        const TideStation& s = m_model->S(m_ribbonIdx[r]);
        const double h = m_model->Height(m_ribbonIdx[r], simUnix);
        m_ribbon.staX[r / 4][r % 4] = static_cast<float>(s.riverKm) * kKmToSceneM;
        m_ribbon.staH[r / 4][r % 4] = static_cast<float>(h);
        // Ribbon pylons take the station colour of the underlying model index.
        for (int c = 0; c < 4; ++c) m_ribbon.staColor[r][c] = kPalette[m_ribbonIdx[r]][c];
    }
    m_ribbon.rib[0] = static_cast<float>(m_model->TotalRiverKm()) * kKmToSceneM;
    m_ribbon.rib[1] = halfWidthM;
    m_ribbon.rib[2] = m_exagg;
    m_ribbon.rib[3] = static_cast<float>(m_nRibbon);
    m_ribbon.misc[0] = 0.0f;
    m_ribbon.misc[1] = contourStepM;
    m_ribbon.misc[2] = static_cast<float>(m_model->S(m_model->Focus()).riverKm) * kKmToSceneM;
    m_ribbon.misc[3] = 0.0f;

    // ---- curves: window-relative phases (the doubles stay on the CPU; shaders only ever see
    // omega * dt with |dt| <= windowSec/2, which fp32 carries comfortably)
    m_curves.win[0] = static_cast<float>(windowSec);
    m_curves.win[1] = m_yMin;
    m_curves.win[2] = m_yMax;
    m_curves.win[3] = static_cast<float>(n);
    m_curves.rect[0] = -0.93f;
    m_curves.rect[1] = -0.96f;
    m_curves.rect[2] = 0.93f;
    m_curves.rect[3] = -0.42f;
    m_curves.misc[0] = static_cast<float>(kMaxCoeffs);
    m_curves.misc[1] = static_cast<float>(kOfficialSamples);
    m_curves.misc[2] = static_cast<float>(m_model->Focus());
    m_curves.misc[3] = 0.0f;

    for (size_t s = 0; s < n; ++s) {
        const TideStation& st = m_model->S(s);
        const size_t nc = std::min<size_t>(st.coeffs.size(), kMaxCoeffs);
        m_curves.mean[s / 4][s % 4] = static_cast<float>(st.meanMllwM);
        m_curves.ncoeff[s / 4][s % 4] = static_cast<float>(nc);
        for (size_t c = 0; c < nc; ++c) {
            const TideCoeff& k = st.coeffs[c];
            const double theta = std::fmod(k.omegaRadS * (simUnix - epoch) + k.phaseRad, kTwoPi);
            float* dst = m_curves.coeffs[s * kMaxCoeffs + c];
            dst[0] = static_cast<float>(k.ampM);
            dst[1] = static_cast<float>(k.omegaRadS);
            dst[2] = static_cast<float>(theta);
            dst[3] = 0.0f;
        }
        for (uint32_t k = 0; k < kOfficialSamples; ++k) {
            const double u = static_cast<double>(k) / (kOfficialSamples - 1);
            const double t = simUnix + (u - 0.5) * windowSec;
            const double v = m_model->Official(s, t);
            m_curves.official[s * (kOfficialSamples / 4) + k / 4][k % 4] =
                std::isnan(v) ? -999.0f : static_cast<float>(v);
        }
    }

    focusHeight = m_model->Height(m_model->Focus(), simUnix);
    m_haveData = true;
}

void TideLayer::Render(const FrameContext& ctx) {
    if (!m_haveData || !m_ribbonPso || !m_curvesPso) return;

    // Ribbon surface, then pylons: same PSO, mode flag in b1.
    {
        PixScope scope(ctx.cmd->Native(), "tide.ribbon");
        ctx.cmd->Pipeline(m_ribbonPso.Get());
        ctx.cmd->Topology(D3D_PRIMITIVE_TOPOLOGY_TRIANGLELIST);
        m_ribbon.misc[0] = 0.0f;
        ctx.cmd->GraphicsConstants(1, m_ribbon);
        ctx.cmd->Draw(6 * kQuadsX * kQuadsZ, 1, 0, 0);

        PixMarker(ctx.cmd->Native(), "tide.pylons");
        m_ribbon.misc[0] = 1.0f;
        ctx.cmd->GraphicsConstants(1, m_ribbon);
        ctx.cmd->Draw(36, m_nRibbon, 0, 0);
    }
    {
        PixScope scope(ctx.cmd->Native(), "tide.curves (solid=analytic, dashed=NOAA official)");
        ctx.cmd->Pipeline(m_curvesPso.Get());
        ctx.cmd->Topology(D3D_PRIMITIVE_TOPOLOGY_LINESTRIP);
        ctx.cmd->GraphicsConstants(1, m_curves);
        ctx.cmd->Draw(kCurveVerts, 18, 0, 0);
    }
}

}  // namespace ga
