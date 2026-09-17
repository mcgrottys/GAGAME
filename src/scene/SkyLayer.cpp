#include "scene/SkyLayer.h"

#include "hal/Pipeline.h"
#include "hal/Root.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <vector>
#include "scene/FieldSet.h"

namespace ga {

void SkyLayer::Init(Gpu& gpu, ShaderCompiler& sc, FieldSet& fields,
                    hal::RootSignature rootSig) {
    (void)fields;   // the sky reads no field data
    m_rootSig = rootSig;
    if (!BuildPso(gpu, sc)) throw std::runtime_error("sky PSO could not be created");

    // ---- THE ATMOSPHERE'S TABLE (M13). One small texture, what the air sends back after the
    // first bounce, filled on the first frame: it depends on the air alone. What the sun
    // delivers is a closed form (Atmosphere.hlsli AtmSunT) and the view is marched per ray, so
    // nothing else is tabulated. --sky-probe adds the two transmittance tables it compares.
    auto make = [&](GpuTexture& t, uint32_t& uav, uint32_t w, uint32_t h, DXGI_FORMAT fmt,
                    const wchar_t* name) {
        t = gpu.CreateTexture2D(w, h, fmt, D3D12_RESOURCE_FLAG_ALLOW_UNORDERED_ACCESS,
                                D3D12_RESOURCE_STATE_UNORDERED_ACCESS, name);
        if (!t.Valid()) return;
        t.state = D3D12_RESOURCE_STATE_UNORDERED_ACCESS;
        t.srv = gpu.CreateSrv(t.res.Get(), fmt);
        uav = gpu.CreateTextureUav(t.res.Get(), fmt, D3D12_UAV_DIMENSION_TEXTURE2D);
    };
    make(m_msTex, m_msUav, kMsW, kMsH, DXGI_FORMAT_R16G16B16A16_FLOAT, L"sky.multiscatter");
    if (m_probe) {
        make(m_transTex, m_transUav, kTransW, kTransH, DXGI_FORMAT_R32G32B32A32_FLOAT,
             L"sky.probe.marched");
        make(m_anaTex, m_anaUav, kTransW, kTransH, DXGI_FORMAT_R32G32B32A32_FLOAT,
             L"sky.probe.closed");
    }
    if (!BuildLutPsos(gpu, sc)) {
        Log("[sky] the atmosphere's kernel did not build -- the dome falls back to the two "
            "constants it always had, and says so rather than drawing black");
    } else {
        Log("[sky] the air, tabulated once: multiple scattering %ux%u -- Rayleigh + Mie + ozone; "
            "the sun's light is a closed form, and every ray marches the air from where it is",
            kMsW, kMsH);
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
    auto kernel = [&](hal::Pso& pso, const wchar_t* entry, const char* name) {
        return hal::Reload(
            pso,
            [&] {
                return hal::BuildCompute(gpu, m_lutRs.Get(), sc.Compile(path, entry, L"cs_6_0"),
                                         "skylut");
            },
            name);
    };
    const bool ms = kernel(m_csMs, L"CsMultiScatter", "sky.multiscatter");
    if (m_probe && !(kernel(m_csTrans, L"CsTransmittance", "sky.probe.marched") &&
                     kernel(m_csAna, L"CsTransAnalytic", "sky.probe.closed"))) {
        Log("[sky-probe] the probe's kernels did not build -- the probe will say so");
    }
    return ms;
}

// THE FILL. One constant-buffer shape for every kernel: how big the planet is, how big the
// table is, and which slot to write.
void SkyLayer::RunLuts(const FrameContext& ctx) {
    if (m_lutStatic || !m_csMs) return;
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
        cb.d[1] = 0u;
        cb.d[2] = 0u;
        cb.d[3] = 0u;
    };
    ctx.cmd->ComputeRoot(m_lutRs.Get());
    ctx.cmd->ComputeBindless(1);
    ctx.cmd->ComputeBindless(2);
    const D3D12_RESOURCE_STATES readable = static_cast<D3D12_RESOURCE_STATES>(
        D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE | D3D12_RESOURCE_STATE_PIXEL_SHADER_RESOURCE);
    auto run = [&](hal::Pso& pso, uint32_t uav, uint32_t w, uint32_t h) {
        LutCb cb{};
        fill(cb, w, h, uav);
        ctx.cmd->ComputeConstants(0, cb);
        ctx.cmd->Pipeline(pso.Get());
        ctx.cmd->Dispatch((w + 7) / 8, (h + 7) / 8, 1);
    };
    run(m_csMs, m_msUav, kMsW, kMsH);
    ctx.cmd->Barrier(m_msTex.res.Get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS, readable);
    m_msTex.state = readable;
    // The probe's two stay where they were written: nothing draws with them, and the readback
    // takes them from the state they are in.
    if (m_probe && m_csTrans && m_csAna && m_transTex.Valid() && m_anaTex.Valid()) {
        run(m_csTrans, m_transUav, kTransW, kTransH);
        run(m_csAna, m_anaUav, kTransW, kTransH);
    }
    m_lutStatic = true;
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
        float winA[4];                              // M13: the view's windows (Sky.hlsl)
        float box[kMaxWindowChain * 16];
        float winUp[kMaxWindowChain * 4];           // ...and the viewpoint each depth lands in
        float winSun[kMaxWindowChain * 4];
    } cb{};
    for (int i = 0; i < 3; ++i) {
        cb.r0[i] = m_rot[i];
        cb.r1[i] = m_rot[3 + i];
        cb.r2[i] = m_rot[6 + i];
        cb.sun[i] = m_sun[i];
    }
    cb.winA[0] = static_cast<float>(m_winN);   // the one number the shader's walk reads
    for (int k = 0; k < m_winN; ++k) {
        m_winBoxes[k].Pack(cb.box + k * 16);
        for (int i = 0; i < 4; ++i) cb.winUp[k * 4 + i] = m_winUp[k * 4 + i];
        for (int i = 0; i < 3; ++i) cb.winSun[k * 4 + i] = m_winSun[k * 3 + i];
    }
    ctx.cmd->Pipeline(m_pso.Get());
    ctx.cmd->GraphicsConstants(1, cb);
    ctx.cmd->DrawFullscreen();
}


// ---- --sky-probe: WHAT THE DEVICE ACTUALLY COMPUTES --------------------------------------------
// A picture of a sky is not evidence that the sky is right; the numbers are. The sun's
// transmittance is a closed form (Atmosphere.hlsli AtmSunT), and this reads back what the device
// computes from it -- on the grid the old table used, every height from the ground to the top of
// the air and every angle down to the horizon -- and holds it against:
//
//   * the SAME AIR SUMMED BY BRUTE FORCE, here in doubles, 10000 steps a ray out to 300 km: the
//     closed form's own error. (The form counts the thin air above the 100 km the march stops
//     at; a reference that stopped there would charge the form with its own truncation, up to a
//     percent of the little optical depth left near the top.) The 40-step table the sky used to
//     read, which does stop at 100 km, is held against it too, so a change in the picture splits
//     into what the old table had wrong and what the form does;
//   * quantities published outside this engine: the RAYLEIGH optical depth of the whole
//     atmosphere against Bodhaine et al. 1999's standard formula, the OZONE column against the
//     0.02-0.035 a 300 DU column gives at 550-600 nm, and the direct sun's colour at a ladder of
//     elevations, which is what makes a sunset red.
void SkyLayer::Probe(Gpu& gpu) {
    if (!m_anaTex.Valid() || !m_transTex.Valid() || !m_csAna || !m_csTrans) {
        Log("[sky-probe] no tables (the kernels did not build) -- nothing to read");
        return;
    }
    auto grab = [&](GpuTexture& t, std::vector<float>& out) {
        uint32_t pitch = 0;
        std::vector<uint8_t> bytes = gpu.ReadbackTexture(t, &pitch);
        out.assign(size_t(kTransW) * kTransH * 4, 0.0f);
        if (bytes.empty()) return false;
        for (uint32_t y = 0; y < kTransH; ++y) {
            std::memcpy(out.data() + size_t(y) * kTransW * 4, bytes.data() + size_t(y) * pitch,
                        size_t(kTransW) * 4 * sizeof(float));
        }
        return true;
    };
    std::vector<float> ana, old;
    if (!grab(m_anaTex, ana) || !grab(m_transTex, old)) {
        Log("[sky-probe] the readback came back empty");
        return;
    }
    const double Rb = m_planetR, Rt = m_planetR + 100000.0;
    const double kPi = 3.14159265358979;
    // The air, as Atmosphere.hlsli states it.
    const double rayS[3] = {5.802e-6, 13.558e-6, 33.100e-6};
    const double ozoA[3] = {0.650e-6, 1.881e-6, 0.085e-6};
    const double mieE = 4.440e-6;
    // A texel's ray (Atmosphere.hlsli AtmTransParams, in doubles).
    auto rayOf = [&](double u, double v, double& r, double& mu) {
        const double H = std::sqrt(Rt * Rt - Rb * Rb);
        const double rho = H * v;
        r = std::sqrt(rho * rho + Rb * Rb);
        const double dMin = Rt - r, dMax = rho + H;
        const double d = dMin + u * (dMax - dMin);
        mu = (d == 0.0) ? 1.0 : std::clamp((H * H - rho * rho - d * d) / (2.0 * r * d), -1.0, 1.0);
    };
    // THE REFERENCE: the column summed in 10000 steps out to 300 km, past the last air a float
    // can see. False for a ray the ground stops -- the grid holds none, and the probe says so if
    // one turns up.
    auto reference = [&](double r, double mu, double T[3]) {
        if (mu < 0.0 && r * r * (mu * mu - 1.0) + Rb * Rb >= 0.0) return false;
        const double Rout = Rb + 300000.0;
        const double top = -r * mu + std::sqrt((std::max)(r * r * (mu * mu - 1.0) + Rout * Rout, 0.0));
        const int steps = 10000;
        const double ds = top / steps;
        double od[3] = {0.0, 0.0, 0.0};
        for (int i = 0; i < steps; ++i) {
            const double t = (i + 0.5) * ds;
            const double h = std::sqrt((std::max)(r * r + t * t + 2.0 * r * mu * t, 0.0)) - Rb;
            const double hp = (std::max)(h, 0.0);
            const double ray = std::exp(-hp / 8000.0), mie = std::exp(-hp / 1200.0);
            const double ozo = (std::max)(0.0, 1.0 - std::abs(h - 25000.0) / 15000.0);
            for (int c = 0; c < 3; ++c) od[c] += (rayS[c] * ray + mieE * mie + ozoA[c] * ozo) * ds;
        }
        for (int c = 0; c < 3; ++c) T[c] = std::exp(-od[c]);
        return true;
    };
    struct Worst {
        double err = 0.0, h = 0.0, el = 0.0, want = 0.0, got = 0.0;
        void Note(double e, double hh, double ee, double w, double g) {
            if (e > err) {
                err = e;
                h = hh;
                el = ee;
                want = w;
                got = g;
            }
        }
    };
    Worst absAna[3], absOld[3], tauAna[3], tauOld[3];
    double sumAna[3] = {0.0, 0.0, 0.0}, sumOld[3] = {0.0, 0.0, 0.0};
    uint32_t rays = 0, grounded = 0;
    for (uint32_t y = 0; y < kTransH; ++y) {
        for (uint32_t x = 0; x < kTransW; ++x) {
            double r = 0.0, mu = 0.0, T[3];
            rayOf((x + 0.5) / kTransW, (y + 0.5) / kTransH, r, mu);
            if (!reference(r, mu, T)) {
                ++grounded;
                continue;
            }
            ++rays;
            const double hKm = (r - Rb) / 1000.0, elDeg = std::asin(mu) * 180.0 / kPi;
            for (int c = 0; c < 3; ++c) {
                const size_t i = (size_t(y) * kTransW + x) * 4 + c;
                const double a = ana[i], o = old[i];
                absAna[c].Note(std::abs(a - T[c]), hKm, elDeg, T[c], a);
                absOld[c].Note(std::abs(o - T[c]), hKm, elDeg, T[c], o);
                sumAna[c] += std::abs(a - T[c]);
                sumOld[c] += std::abs(o - T[c]);
                // The optical depth's relative error, where there is light left to see it by and
                // enough air for a float to resolve it.
                if (T[c] > 1e-3 && T[c] < 0.999) {
                    const double tau = -std::log(T[c]);
                    tauAna[c].Note(std::abs(-std::log((std::max)(a, 1e-30)) / tau - 1.0), hKm,
                                   elDeg, T[c], a);
                    tauOld[c].Note(std::abs(-std::log((std::max)(o, 1e-30)) / tau - 1.0), hKm,
                                   elDeg, T[c], o);
                }
            }
        }
    }
    const int lamNm[3] = {680, 550, 440};
    Log("[sky-probe] the sun's transmittance, read back off the device on the %ux%u grid (%u rays, "
        "0-100 km, zenith to horizon%s) against the same air summed in doubles to 300 km, 10000 "
        "steps a ray (the old table stops at 100 km):",
        kTransW, kTransH, rays, grounded ? "; SOME TEXELS' RAYS HIT THE GROUND" : "");
    for (int c = 0; c < 3; ++c) {
        const double m = 1.0 / (std::max)(rays, 1u);
        Log("[sky-probe]   %d nm  closed form: |dT| max %.5f at %.1f km %+.2f deg (%.5f for %.5f), "
            "mean %.6f; tau within %.2f %% (worst at %.1f km %+.2f deg)",
            lamNm[c], absAna[c].err, absAna[c].h, absAna[c].el, absAna[c].got, absAna[c].want,
            sumAna[c] * m, 100.0 * tauAna[c].err, tauAna[c].h, tauAna[c].el);
        Log("[sky-probe]   %d nm  old table:   |dT| max %.5f at %.1f km %+.2f deg (%.5f for %.5f), "
            "mean %.6f; tau within %.2f %% (worst at %.1f km %+.2f deg)",
            lamNm[c], absOld[c].err, absOld[c].h, absOld[c].el, absOld[c].got, absOld[c].want,
            sumOld[c] * m, 100.0 * tauOld[c].err, tauOld[c].h, tauOld[c].el);
    }

    // The closed form's table, looked up the way the old one was (the inverse of
    // Atmosphere.hlsli AtmTransParams), for the published-number checks below.
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
            const double a = ana[(size_t(y0) * kTransW + x0) * 4 + c] * (1 - ax) +
                             ana[(size_t(y0) * kTransW + x1) * 4 + c] * ax;
            const double b = ana[(size_t(y1) * kTransW + x0) * 4 + c] * (1 - ax) +
                             ana[(size_t(y1) * kTransW + x1) * 4 + c] * ax;
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
    double tauModel[3], tauRayOnly[3];
    for (int c = 0; c < 3; ++c) {
        tauRayOnly[c] = rayS[c] * 8000.0;
        tauModel[c] = tauRayOnly[c] + mieE * 1200.0 + ozoA[c] * 15000.0;
    }
    // The grid's lowest, steepest texel is not quite straight up (a texel is read at its centre),
    // so it is shown beside the brute force on its own ray and beside exp(-tau) straight up.
    {
        double r0 = 0.0, mu0 = 0.0, T0[3];
        rayOf(0.5 / kTransW, 0.5 / kTransH, r0, mu0);
        reference(r0, mu0, T0);
        Log("[sky-probe]   the lowest, steepest texel (%.1f m, %.2f deg): closed form, the same ray "
            "summed, and exp(-tau) of the model's own vertical column:",
            r0 - Rb, std::asin(mu0) * 180.0 / kPi);
        for (int c = 0; c < 3; ++c) {
            const double got = ana[size_t(c)];
            const double want = std::exp(-tauModel[c]);
            Log("[sky-probe]     %3.0f nm  closed %.5f   summed %.5f (%+.3f %%)   straight up "
                "exp(-%.4f) = %.5f",
                lam[c] * 1000.0, got, T0[c], 100.0 * (got / T0[c] - 1.0), tauModel[c], want);
        }
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
        ozoA[1] * 15000.0, mieE * 1200.0);
    Log("[sky-probe]   the SUN's own colour, through the air at this altitude:");
    for (double el : {60.0, 30.0, 10.0, 5.0, 2.0, 0.0}) {
        double t[3];
        transAt(m_planetR + 1.0, std::sin(el * kPi / 180.0), t);
        Log("[sky-probe]     %5.1f deg  (%.4f, %.4f, %.4f)  R/B = %.2f", el, t[0], t[1], t[2],
            (t[2] > 1e-6) ? t[0] / t[2] : 0.0);
    }
}

}  // namespace ga
