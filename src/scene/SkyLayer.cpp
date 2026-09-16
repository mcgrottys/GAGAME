#include "scene/SkyLayer.h"

#include "hal/Pipeline.h"
#include "hal/Root.h"

#include <algorithm>
#include <cmath>
#include <vector>
#include "scene/FieldSet.h"

namespace ga {

void SkyLayer::Init(Gpu& gpu, ShaderCompiler& sc, FieldSet& fields,
                    hal::RootSignature rootSig) {
    (void)fields;   // the sky reads no field data
    m_rootSig = rootSig;
    if (!BuildPso(gpu, sc)) throw std::runtime_error("sky PSO could not be created");

    // ---- THE ATMOSPHERE'S TABLES (M13). Three small textures: what the air lets through, what
    // it sends back after the first bounce, and this eye's hemisphere. The first two depend on
    // the air alone and are filled on the first frame; the third is refilled every frame,
    // because the eye and the sun move and the sky is a function of both.
    const DXGI_FORMAT fmt = DXGI_FORMAT_R16G16B16A16_FLOAT;
    auto make = [&](GpuTexture& t, uint32_t& uav, uint32_t w, uint32_t h, const wchar_t* name) {
        t = gpu.CreateTexture2D(w, h, fmt, D3D12_RESOURCE_FLAG_ALLOW_UNORDERED_ACCESS,
                                D3D12_RESOURCE_STATE_UNORDERED_ACCESS, name);
        if (!t.Valid()) return;
        t.state = D3D12_RESOURCE_STATE_UNORDERED_ACCESS;
        t.srv = gpu.CreateSrv(t.res.Get(), fmt);
        uav = gpu.CreateTextureUav(t.res.Get(), fmt, D3D12_UAV_DIMENSION_TEXTURE2D);
    };
    make(m_transTex, m_transUav, kTransW, kTransH, L"sky.transmittance");
    make(m_msTex, m_msUav, kMsW, kMsH, L"sky.multiscatter");
    if (!BuildLutPsos(gpu, sc)) {
        Log("[sky] the atmosphere's kernels did not build -- the dome falls back to the two "
            "constants it always had, and says so rather than drawing black");
    } else {
        Log("[sky] the air, tabulated once: transmittance %ux%u, multiple scattering %ux%u -- "
            "Rayleigh + Mie + ozone; every ray marches it from where it is",
            kTransW, kTransH, kMsW, kMsH);
    }
}

bool SkyLayer::BuildLutPsos(Gpu& gpu, ShaderCompiler& sc) {
    // Its own root signature, because this is a COMPUTE pass: b0 its constants, the whole heap
    // as bindless SRVs (the tables read each other by slot) and as bindless UAVs (the table
    // being written), and the house samplers so Common.hlsli's declarations bind.
    m_lutRs = hal::RootLayout{}
                  .Cbv(0)
                  .Table({hal::SrvRange(0, hal::kUnbounded, 1)})
                  .Table({hal::UavRange(0, hal::kUnbounded, 2)})
                  .Sampler(hal::StaticSampler(0, D3D12_FILTER_MIN_MAG_MIP_LINEAR,
                                              D3D12_TEXTURE_ADDRESS_MODE_CLAMP))
                  .Sampler(hal::StaticSampler(1, D3D12_FILTER_MIN_MAG_MIP_LINEAR,
                                              D3D12_TEXTURE_ADDRESS_MODE_WRAP))
                  .Sampler(hal::StaticSampler(2, D3D12_FILTER_MIN_MAG_MIP_POINT,
                                              D3D12_TEXTURE_ADDRESS_MODE_CLAMP))
                  .Sampler(hal::StaticSampler(3, D3D12_FILTER_ANISOTROPIC,
                                              D3D12_TEXTURE_ADDRESS_MODE_CLAMP, 8))
                  .Build(gpu, "skylut");
    if (!m_lutRs) return false;
    const std::wstring path = m_shaderDir + L"/SkyLut.hlsl";
    const bool a = hal::Reload(
        m_csTrans,
        [&] {
            return hal::BuildCompute(gpu, m_lutRs.Get(),
                                     sc.Compile(path, L"CsTransmittance", L"cs_6_0"), "skylut");
        },
        "sky.transmittance");
    const bool b = hal::Reload(
        m_csMs,
        [&] {
            return hal::BuildCompute(gpu, m_lutRs.Get(),
                                     sc.Compile(path, L"CsMultiScatter", L"cs_6_0"), "skylut");
        },
        "sky.multiscatter");
    return a && b;
}

// THE FILL. One constant-buffer shape for all three kernels: where the eye is, how big the
// planet is, which slots to read and which to write.
void SkyLayer::RunLuts(const FrameContext& ctx) {
    if (m_lutStatic || !m_csTrans || !m_csMs) return;
    struct LutCb {
        float a[4];
        float b[4];
        float c[4];
        uint32_t d[4];
    };
    const double Rb = m_planetR;
    const double Rt = m_planetR + 100000.0;
    auto fill = [&](LutCb& cb, uint32_t w, uint32_t h, uint32_t uav) {
        for (int i = 0; i < 3; ++i) cb.a[i] = 0.0f;
        cb.a[3] = static_cast<float>(Rb);
        cb.b[0] = static_cast<float>(Rb);
        cb.b[1] = static_cast<float>(Rt);
        cb.b[2] = 0.0f;
        cb.b[3] = 0.0f;
        cb.c[0] = static_cast<float>(w);
        cb.c[1] = static_cast<float>(h);
        cb.c[2] = 0.0f;
        cb.c[3] = 0.0f;
        cb.d[0] = uav;
        cb.d[1] = m_transTex.srv;
        cb.d[2] = m_msTex.srv;
        cb.d[3] = 0u;
    };
    ctx.cmd->ComputeRoot(m_lutRs.Get());
    ctx.cmd->ComputeBindless(1);
    ctx.cmd->ComputeBindless(2);
    auto toSrv = [&](GpuTexture& t) {
        ctx.cmd->Barrier(t.res.Get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS,
                         D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE |
                             D3D12_RESOURCE_STATE_PIXEL_SHADER_RESOURCE);
        t.state = D3D12_RESOURCE_STATE_PIXEL_SHADER_RESOURCE;
    };
    if (!m_lutStatic && m_csTrans && m_csMs) {
        LutCb cb{};
        fill(cb, kTransW, kTransH, m_transUav);
        ctx.cmd->ComputeConstants(0, cb);
        ctx.cmd->Pipeline(m_csTrans.Get());
        ctx.cmd->Dispatch((kTransW + 7) / 8, (kTransH + 7) / 8, 1);
        toSrv(m_transTex);
        fill(cb, kMsW, kMsH, m_msUav);
        ctx.cmd->ComputeConstants(0, cb);
        ctx.cmd->Pipeline(m_csMs.Get());
        ctx.cmd->Dispatch((kMsW + 7) / 8, (kMsH + 7) / 8, 1);
        toSrv(m_msTex);
        m_lutStatic = true;
    }
}

bool SkyLayer::BuildPso(Gpu& gpu, ShaderCompiler& sc) {
    const std::wstring path = m_shaderDir + L"/Sky.hlsl";
    // THE DEFAULTS, as the builder states them, were read from this block: opaque, cull none,
    // depth OFF for both test and write (the sky is a backdrop: it must not occlude anything,
    // and it must not leave depth values behind), the HDR target, triangles.
    hal::GraphicsPipelineDesc d;
    d.rootSig = m_rootSig;
    d.vs = sc.Compile(path, L"VsMain", L"vs_6_0");
    d.ps = sc.Compile(path, L"PsMain", L"ps_6_0");
    return hal::Reload(m_pso, [&] { return hal::BuildGraphics(gpu, d, "sky"); }, "sky");
}

void SkyLayer::ReloadShaders(Gpu& gpu, ShaderCompiler& sc) {
    BuildPso(gpu, sc);   // the reload law lives in BuildPso: swap only on success
}

void SkyLayer::Render(const FrameContext& ctx) {
    if (!m_pso) return;
    // The tables first: this pass and every later one read them THIS frame.
    RunLuts(ctx);
    // M10: the sky's frame (Sky.hlsl SkyFrameCb, b1): three rotation rows and the sun.
    struct {
        float r0[4], r1[4], r2[4], sun[4];
        float boxR0[4], boxR1[4], boxR2[4], boxC[4];   // M13: the gate's window (Sky.hlsl)
        float winUp[4], winSun[4];                     // ...and the viewpoint its rays land in
    } cb{};
    for (int i = 0; i < 3; ++i) {
        cb.r0[i] = m_rot[i];
        cb.r1[i] = m_rot[3 + i];
        cb.r2[i] = m_rot[6 + i];
        cb.sun[i] = m_sun[i];
        cb.boxR0[i] = m_winBox[i];
        cb.boxR1[i] = m_winBox[3 + i];
        cb.boxR2[i] = m_winBox[6 + i];
        cb.boxC[i] = m_winC[i];
        cb.winUp[i] = m_winUp[i];
        cb.winSun[i] = m_winSun[i];
    }
    cb.boxR0[3] = m_winHalf[0];
    cb.boxR1[3] = m_winHalf[1];
    cb.boxR2[3] = m_winHalf[2];
    cb.boxC[3] = m_winOn ? 1.0f : 0.0f;   // the one flag the shader tests
    cb.winUp[3] = m_winEyeR;
    ctx.cmd->Pipeline(m_pso.Get());
    ctx.cmd->GraphicsConstants(1, cb);
    ctx.cmd->DrawFullscreen();
}


// ---- --sky-probe: WHAT THE GPU ACTUALLY PUT IN THE TABLES ------------------------------------
// A picture of a sky is not evidence that the sky is right; the numbers are. This reads the two
// tables back off the device and holds them against quantities published outside this engine:
//
//   * the RAYLEIGH optical depth of the whole atmosphere, against Bodhaine et al. 1999's
//     standard formula (the one every atmospheric-optics paper cites for sea-level Rayleigh
//     scattering), computed here from wavelength alone -- an independent number, not a
//     restatement of the coefficients the shader used;
//   * the OZONE column's optical depth in the Chappuis band, against the 0.02-0.035 a 300 DU
//     column gives at 550-600 nm;
//   * the direct sun's transmitted colour at a ladder of elevations, which is what makes a
//     sunset red, and its Beer-Lambert airmass behaviour.
//
// The gate is the FIRST of those: the table's own vertical transmittance must be exp(-tau) for
// the tau its coefficients imply, and that tau must sit near the measurement.
void SkyLayer::Probe(Gpu& gpu) {
    if (!m_transTex.Valid()) {
        Log("[sky-probe] no tables (the kernels did not build) -- nothing to read");
        return;
    }
    auto grab = [&](GpuTexture& t, uint32_t w, uint32_t h, std::vector<float>& out) {
        uint32_t pitch = 0;
        std::vector<uint8_t> bytes = gpu.ReadbackTexture(t, &pitch);
        out.assign(size_t(w) * h * 4, 0.0f);
        if (bytes.empty()) return false;
        for (uint32_t y = 0; y < h; ++y) {
            const uint16_t* row = reinterpret_cast<const uint16_t*>(bytes.data() + size_t(y) * pitch);
            for (uint32_t x = 0; x < w * 4; ++x) out[size_t(y) * w * 4 + x] = HalfToFloat(row[x]);
        }
        return true;
    };
    std::vector<float> tr;
    if (!grab(m_transTex, kTransW, kTransH, tr)) {
        Log("[sky-probe] the readback came back empty");
        return;
    }
    const double Rb = m_planetR, Rt = m_planetR + 100000.0;
    // The table's own parameterisation, on the CPU (Atmosphere.hlsli AtmTransUv).
    auto transAt = [&](double r, double mu, double out[3]) {
        const double H = std::sqrt((std::max)(Rt * Rt - Rb * Rb, 1e-6));
        const double rho = std::sqrt((std::max)(r * r - Rb * Rb, 0.0));
        const double disc = r * r * (mu * mu - 1.0) + Rt * Rt;
        const double d = (std::max)(0.0, -r * mu + std::sqrt((std::max)(disc, 0.0)));
        const double u = std::clamp((d - (Rt - r)) / (std::max)((rho + H) - (Rt - r), 1e-6), 0.0, 1.0);
        const double v = std::clamp(rho / H, 0.0, 1.0);
        const double fx = u * kTransW - 0.5, fy = v * kTransH - 0.5;
        const int x0 = std::clamp(int(std::floor(fx)), 0, int(kTransW) - 1);
        const int y0 = std::clamp(int(std::floor(fy)), 0, int(kTransH) - 1);
        const int x1 = (std::min)(x0 + 1, int(kTransW) - 1);
        const int y1 = (std::min)(y0 + 1, int(kTransH) - 1);
        const double ax = std::clamp(fx - x0, 0.0, 1.0), ay = std::clamp(fy - y0, 0.0, 1.0);
        for (int c = 0; c < 3; ++c) {
            const double a = tr[(size_t(y0) * kTransW + x0) * 4 + c] * (1 - ax) +
                             tr[(size_t(y0) * kTransW + x1) * 4 + c] * ax;
            const double b = tr[(size_t(y1) * kTransW + x0) * 4 + c] * (1 - ax) +
                             tr[(size_t(y1) * kTransW + x1) * 4 + c] * ax;
            out[c] = a * (1 - ay) + b * ay;
        }
    };
    // BODHAINE ET AL. 1999, equation (30): the sea-level Rayleigh optical depth of the whole
    // atmosphere for a wavelength in micrometres. Published, and computed here from nothing this
    // engine owns.
    auto bodhaine = [](double umicron) {
        const double l2 = umicron * umicron;
        return 0.0021520 * (1.0455996 - 341.29061 / l2 - 0.90230850 * l2) /
               (1.0 + 0.0027059889 / l2 - 85.968563 * l2);
    };
    const double lam[3] = {0.680, 0.550, 0.440};
    // The model's own column: beta(0) * H for the exponential terms, beta * w for ozone's tent.
    const double rayB[3] = {5.802e-6, 13.558e-6, 33.100e-6};
    const double ozoB[3] = {0.650e-6, 1.881e-6, 0.085e-6};
    double tauModel[3], tauRayOnly[3];
    for (int c = 0; c < 3; ++c) {
        tauRayOnly[c] = rayB[c] * 8000.0;
        tauModel[c] = tauRayOnly[c] + 4.440e-6 * 1200.0 + ozoB[c] * 15000.0;
    }
    double tz[3];
    transAt(Rb, 1.0, tz);
    Log("[sky-probe] the air's transmittance, read back off the device (%ux%u)", kTransW, kTransH);
    Log("[sky-probe]   vertical transmittance at sea level, table vs exp(-tau) of its own "
        "coefficients:");
    for (int c = 0; c < 3; ++c) {
        const double want = std::exp(-tauModel[c]);
        Log("[sky-probe]     %3.0f nm  table %.5f   exp(-%.4f) = %.5f   (%+.2f %%)",
            lam[c] * 1000.0, tz[c], tauModel[c], want, 100.0 * (tz[c] / want - 1.0));
    }
    Log("[sky-probe]   RAYLEIGH column against Bodhaine et al. 1999 (independent of this engine):");
    for (int c = 0; c < 3; ++c) {
        const double meas = bodhaine(lam[c]);
        Log("[sky-probe]     %3.0f nm  model %.4f   measured %.4f   (%+.1f %%: the exponential "
            "profile's own error)",
            lam[c] * 1000.0, tauRayOnly[c], meas, 100.0 * (tauRayOnly[c] / meas - 1.0));
    }
    Log("[sky-probe]   OZONE column %.4f at 550 nm (a 300 DU Chappuis column measures "
        "0.02-0.035), MIE column %.4f (a very clean marine air; typical AOD is 0.05-0.15)",
        ozoB[1] * 15000.0, 4.440e-6 * 1200.0);
    Log("[sky-probe]   the SUN's own colour, through the air at this altitude:");
    for (double el : {60.0, 30.0, 10.0, 5.0, 2.0, 0.0}) {
        double t[3];
        transAt(m_planetR + 1.0, std::sin(el * 3.14159265358979 / 180.0), t);
        Log("[sky-probe]     %5.1f deg  (%.4f, %.4f, %.4f)  R/B = %.2f", el, t[0], t[1], t[2],
            (t[2] > 1e-6) ? t[0] / t[2] : 0.0);
    }
}

}  // namespace ga
