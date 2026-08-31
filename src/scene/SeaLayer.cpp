#include "scene/SeaLayer.h"

#include "core/PixEvents.h"
#include "scene/FieldSet.h"

#include <algorithm>
#include <cmath>

namespace ga {

void SeaLayer::Init(Gpu& gpu, ShaderCompiler& sc, FieldSet& fields,
                    ID3D12RootSignature* rootSig) {
    (void)fields;
    m_rootSig = rootSig;
    if (!m_sea || !m_sea->Ready()) throw std::runtime_error("SeaLayer needs a SeaState");
    m_gpu = &gpu;
    if (!BuildPsos(gpu, sc)) throw std::runtime_error("sea PSOs could not be created");
    m_fft.Init(gpu, sc, m_shaderDir);
    if (gpu.TiledTier() >= D3D12_TILED_RESOURCES_TIER_2) {
        InitChurn(gpu, sc);
    } else {
        Log("[sea] tiled tier < 2: churn atlas disabled (dense fallback not built)");
    }

    if (const BuoyObs* b = m_sea->Buoy("44013")) hsBuoy = b->hs;
}

void SeaLayer::InitChurn(Gpu& gpu, ShaderCompiler& sc) {
    m_churn.Init(gpu, static_cast<uint32_t>(kChurnDomainM / kChurnTexelM),
                 static_cast<uint32_t>(kChurnDomainM / kChurnTexelM), DXGI_FORMAT_R16_FLOAT,
                 L"sea.churn (sparse stateful G0 bank)");
    m_lastActive.assign(static_cast<size_t>(m_churn.TilesX()) * m_churn.TilesY(), -1.0e18);

    // Root signature: b0 CBV, t0 root SRV (tile list from the frame arena), table
    // [t1 chop-deriv, t2 swe uv, t3 bathy, u0 churn], s0 wrap + s1 clamp samplers.
    D3D12_DESCRIPTOR_RANGE1 ranges[2]{};
    ranges[0].RangeType = D3D12_DESCRIPTOR_RANGE_TYPE_SRV;
    ranges[0].NumDescriptors = 3;
    ranges[0].BaseShaderRegister = 1;
    ranges[0].Flags = D3D12_DESCRIPTOR_RANGE_FLAG_DESCRIPTORS_VOLATILE;
    ranges[0].OffsetInDescriptorsFromTableStart = 0;
    ranges[1].RangeType = D3D12_DESCRIPTOR_RANGE_TYPE_UAV;
    ranges[1].NumDescriptors = 1;
    ranges[1].BaseShaderRegister = 0;
    ranges[1].Flags = D3D12_DESCRIPTOR_RANGE_FLAG_DESCRIPTORS_VOLATILE;
    ranges[1].OffsetInDescriptorsFromTableStart = 3;
    D3D12_ROOT_PARAMETER1 params[3]{};
    params[0].ParameterType = D3D12_ROOT_PARAMETER_TYPE_CBV;
    params[0].Descriptor.ShaderRegister = 0;
    params[1].ParameterType = D3D12_ROOT_PARAMETER_TYPE_SRV;
    params[1].Descriptor.ShaderRegister = 0;
    params[2].ParameterType = D3D12_ROOT_PARAMETER_TYPE_DESCRIPTOR_TABLE;
    params[2].DescriptorTable.NumDescriptorRanges = 2;
    params[2].DescriptorTable.pDescriptorRanges = ranges;
    D3D12_STATIC_SAMPLER_DESC samps[2]{};
    samps[0].Filter = D3D12_FILTER_MIN_MAG_MIP_LINEAR;
    samps[0].AddressU = samps[0].AddressV = samps[0].AddressW = D3D12_TEXTURE_ADDRESS_MODE_WRAP;
    samps[0].MaxLOD = D3D12_FLOAT32_MAX;
    samps[0].ShaderRegister = 0;
    samps[0].ComparisonFunc = D3D12_COMPARISON_FUNC_NEVER;
    samps[1] = samps[0];
    samps[1].AddressU = samps[1].AddressV = samps[1].AddressW = D3D12_TEXTURE_ADDRESS_MODE_CLAMP;
    samps[1].ShaderRegister = 1;
    D3D12_VERSIONED_ROOT_SIGNATURE_DESC vd{};
    vd.Version = D3D_ROOT_SIGNATURE_VERSION_1_1;
    vd.Desc_1_1.NumParameters = _countof(params);
    vd.Desc_1_1.pParameters = params;
    vd.Desc_1_1.NumStaticSamplers = 2;
    vd.Desc_1_1.pStaticSamplers = samps;
    Com<ID3DBlob> blob, err;
    GA_CHECK(D3D12SerializeVersionedRootSignature(&vd, &blob, &err));
    GA_CHECK(gpu.Device()->CreateRootSignature(0, blob->GetBufferPointer(), blob->GetBufferSize(),
                                              IID_PPV_ARGS(&m_churnRs)));
    m_churnRs->SetName(L"sea.churn root signature");

    auto makePso = [&](const wchar_t* entry, Com<ID3D12PipelineState>& out) {
        ShaderBlob cs = sc.Compile(m_shaderDir + L"/SeaChurn.hlsl", entry, L"cs_6_0");
        if (!cs.Valid()) throw std::runtime_error("SeaChurn kernel failed");
        D3D12_COMPUTE_PIPELINE_STATE_DESC d{};
        d.pRootSignature = m_churnRs.Get();
        d.CS = {cs.Data(), cs.Size()};
        GA_CHECK(gpu.Device()->CreateComputePipelineState(&d, IID_PPV_ARGS(&out)));
        out->SetName(entry);
    };
    makePso(L"CsChurnClear", m_churnClear);
    makePso(L"CsChurnUpdate", m_churnUpdate);

    m_churnTable = gpu.SrvHeap().Alloc(4);
    D3D12_SHADER_RESOURCE_VIEW_DESC sv{};
    sv.Shader4ComponentMapping = D3D12_DEFAULT_SHADER_4_COMPONENT_MAPPING;
    sv.Format = DXGI_FORMAT_R16G16B16A16_FLOAT;
    sv.ViewDimension = D3D12_SRV_DIMENSION_TEXTURE2D;
    sv.Texture2D.MipLevels = 1;
    gpu.Device()->CreateShaderResourceView(m_fft.DerivRes(2), &sv,
                                           gpu.SrvHeap().Cpu(m_churnTable + 0));
    // t2 (solved currents) and t3 (bathy) start as NULL views -- they read zeros, which the
    // kernel's gSweM.x gate never touches -- and are wired in RecordChurn once the solver
    // exists (main attaches it after this layer's Init).
    gpu.Device()->CreateShaderResourceView(nullptr, &sv, gpu.SrvHeap().Cpu(m_churnTable + 1));
    sv.Format = DXGI_FORMAT_R32_FLOAT;
    gpu.Device()->CreateShaderResourceView(nullptr, &sv, gpu.SrvHeap().Cpu(m_churnTable + 2));
    D3D12_UNORDERED_ACCESS_VIEW_DESC uv{};
    uv.Format = DXGI_FORMAT_R16_FLOAT;
    uv.ViewDimension = D3D12_UAV_DIMENSION_TEXTURE2D;
    gpu.Device()->CreateUnorderedAccessView(m_churn.Res(), nullptr, &uv,
                                            gpu.SrvHeap().Cpu(m_churnTable + 3));

    m_maskCpu.assign(static_cast<size_t>(m_churn.TilesX()) * m_churn.TilesY(), 0);
    m_maskTex = gpu.CreateTexture2D(m_churn.TilesX(), m_churn.TilesY(), DXGI_FORMAT_R8_UNORM,
                                    D3D12_RESOURCE_FLAG_NONE, D3D12_RESOURCE_STATE_COPY_DEST,
                                    L"sea.churn.residencyMask");
    gpu.UploadTexture(m_maskTex, m_maskCpu.data(), m_churn.TilesX());
    m_maskTex.srv = gpu.CreateSrv(m_maskTex.res.Get(), DXGI_FORMAT_R8_UNORM);
    m_churnReady = true;
}

bool SeaLayer::BuildPsos(Gpu& gpu, ShaderCompiler& sc) {
    auto makePso = [&](const wchar_t* file, bool lines, bool depth,
                       Com<ID3D12PipelineState>& out) -> bool {
        const std::wstring path = m_shaderDir + L"/" + file;
        ShaderBlob vs = sc.Compile(path, L"VsMain", L"vs_6_0");
        ShaderBlob ps = sc.Compile(path, L"PsMain", L"ps_6_0");
        if (!vs.Valid() || !ps.Valid()) return false;

        D3D12_GRAPHICS_PIPELINE_STATE_DESC d{};
        d.pRootSignature = m_rootSig;
        d.VS = {vs.Data(), vs.Size()};
        d.PS = {ps.Data(), ps.Size()};
        d.BlendState.RenderTarget[0].SrcBlend = D3D12_BLEND_ONE;
        d.BlendState.RenderTarget[0].DestBlend = D3D12_BLEND_ZERO;
        d.BlendState.RenderTarget[0].BlendOp = D3D12_BLEND_OP_ADD;
        d.BlendState.RenderTarget[0].SrcBlendAlpha = D3D12_BLEND_ONE;
        d.BlendState.RenderTarget[0].DestBlendAlpha = D3D12_BLEND_ZERO;
        d.BlendState.RenderTarget[0].BlendOpAlpha = D3D12_BLEND_OP_ADD;
        d.BlendState.RenderTarget[0].RenderTargetWriteMask = D3D12_COLOR_WRITE_ENABLE_ALL;
        d.SampleMask = UINT_MAX;
        d.RasterizerState.FillMode = D3D12_FILL_MODE_SOLID;
        d.RasterizerState.CullMode = D3D12_CULL_MODE_NONE;
        d.RasterizerState.DepthClipEnable = TRUE;
        d.DepthStencilState.DepthEnable = depth ? TRUE : FALSE;
        d.DepthStencilState.DepthWriteMask = depth ? D3D12_DEPTH_WRITE_MASK_ALL
                                                   : D3D12_DEPTH_WRITE_MASK_ZERO;
        d.DepthStencilState.DepthFunc = D3D12_COMPARISON_FUNC_GREATER;   // reversed-Z
        d.DSVFormat = DXGI_FORMAT_D32_FLOAT;
        d.PrimitiveTopologyType = lines ? D3D12_PRIMITIVE_TOPOLOGY_TYPE_LINE
                                        : D3D12_PRIMITIVE_TOPOLOGY_TYPE_TRIANGLE;
        d.NumRenderTargets = 1;
        d.RTVFormats[0] = DXGI_FORMAT_R16G16B16A16_FLOAT;
        d.SampleDesc.Count = 1;

        Com<ID3D12PipelineState> pso;
        if (FAILED(gpu.Device()->CreateGraphicsPipelineState(&d, IID_PPV_ARGS(&pso)))) {
            Log("[sea] PSO %S failed", file);
            return false;
        }
        out = pso;
        return true;
    };
    // The sea itself tessellates (M5b): VS emits control points, HS sets screen-space edge
    // factors, DS displaces -- vqview's chain, ported.
    auto makeSeaPso = [&](Com<ID3D12PipelineState>& out) -> bool {
        const std::wstring path = m_shaderDir + L"/Sea.hlsl";
        ShaderBlob vs = sc.Compile(path, L"VsMain", L"vs_6_0");
        ShaderBlob hs = sc.Compile(path, L"HsMain", L"hs_6_0");
        ShaderBlob ds = sc.Compile(path, L"DsMain", L"ds_6_0");
        ShaderBlob ps = sc.Compile(path, L"PsMain", L"ps_6_0");
        if (!vs.Valid() || !hs.Valid() || !ds.Valid() || !ps.Valid()) return false;

        D3D12_GRAPHICS_PIPELINE_STATE_DESC d{};
        d.pRootSignature = m_rootSig;
        d.VS = {vs.Data(), vs.Size()};
        d.HS = {hs.Data(), hs.Size()};
        d.DS = {ds.Data(), ds.Size()};
        d.PS = {ps.Data(), ps.Size()};
        d.BlendState.RenderTarget[0].SrcBlend = D3D12_BLEND_ONE;
        d.BlendState.RenderTarget[0].DestBlend = D3D12_BLEND_ZERO;
        d.BlendState.RenderTarget[0].BlendOp = D3D12_BLEND_OP_ADD;
        d.BlendState.RenderTarget[0].SrcBlendAlpha = D3D12_BLEND_ONE;
        d.BlendState.RenderTarget[0].DestBlendAlpha = D3D12_BLEND_ZERO;
        d.BlendState.RenderTarget[0].BlendOpAlpha = D3D12_BLEND_OP_ADD;
        d.BlendState.RenderTarget[0].RenderTargetWriteMask = D3D12_COLOR_WRITE_ENABLE_ALL;
        d.SampleMask = UINT_MAX;
        d.RasterizerState.FillMode = D3D12_FILL_MODE_SOLID;
        d.RasterizerState.CullMode = D3D12_CULL_MODE_NONE;
        d.RasterizerState.DepthClipEnable = TRUE;
        d.DepthStencilState.DepthEnable = TRUE;
        d.DepthStencilState.DepthWriteMask = D3D12_DEPTH_WRITE_MASK_ALL;
        d.DepthStencilState.DepthFunc = D3D12_COMPARISON_FUNC_GREATER;   // reversed-Z
        d.DSVFormat = DXGI_FORMAT_D32_FLOAT;
        d.PrimitiveTopologyType = D3D12_PRIMITIVE_TOPOLOGY_TYPE_PATCH;
        d.NumRenderTargets = 1;
        d.RTVFormats[0] = DXGI_FORMAT_R16G16B16A16_FLOAT;
        d.SampleDesc.Count = 1;

        Com<ID3D12PipelineState> pso;
        if (FAILED(gpu.Device()->CreateGraphicsPipelineState(&d, IID_PPV_ARGS(&pso)))) {
            Log("[sea] tessellated sea PSO failed");
            return false;
        }
        out = pso;
        return true;
    };

    Com<ID3D12PipelineState> sea, spec;
    if (!makeSeaPso(sea)) return false;
    if (!makePso(L"SpecPlot.hlsl", true, false, spec)) return false;
    m_seaPso = sea;
    m_specPso = spec;
    return true;
}

void SeaLayer::ReloadShaders(Gpu& gpu, ShaderCompiler& sc) {
    Com<ID3D12PipelineState> keepSea = m_seaPso, keepSpec = m_specPso;
    if (!BuildPsos(gpu, sc)) {
        m_seaPso = keepSea;
        m_specPso = keepSpec;
        Log("[sea] reload failed; keeping the previous PSOs");
    }
    if (!m_fft.ReloadShaders(gpu, sc)) Log("[sea] ocean compute reload failed; keeping previous");
}

void SeaLayer::SetStorm(float hs, float tp, float fromDeg) {
    m_stormHs = hs;
    m_stormTp = tp;
    m_stormDir = fromDeg;
    m_lastHour = -1;   // force a respectrum on the next SetTime
}

void SeaLayer::SetTime(double simUnix, double seaLevelM, double camX, double camZ) {
    if (!m_sea || !m_sea->Ready()) return;

    const int hour = m_sea->HourIndex(simUnix);
    if (hour != m_lastHour) {
        m_lastHour = hour;
        PartParam parts[4];
        if (m_stormHs > 0.01f) {
            // Sandbox storm: the named swell carries 85% of the energy, a fresher wind sea 25%
            // of the height on a slightly shorter period, 30 degrees off.
            parts[0] = SeaState::MakePartition(m_stormHs * 0.92, m_stormTp, m_stormDir, false);
            parts[1] = SeaState::MakePartition(m_stormHs * 0.40, m_stormTp * 0.55,
                                               m_stormDir + 30.0, true);
            activeParts = 2;
        } else {
            activeParts = m_sea->BuildParams(hour, parts);
        }
        const uint32_t seed = static_cast<uint32_t>(m_sea->CycleUnix() / 3600.0) * 2654435761u;
        m_fft.SetSeaState(parts, activeParts, seed);
        hsModel = SeaState::SignificantHeight(parts, activeParts);
        if (activeParts > 0) {
            // M7j: the hypervisor's second catch -- parts[0] is FILE order, not energy
            // order, and at calm hours the first entry can be the 0.05 m westerly wind
            // chop: the swell shadow then marched TOWARD the dunes from every ocean cell
            // and the whole sea read as sheltered. The peak is the most ENERGETIC
            // partition; cPeak and the shadow direction both ride it.
            int pk = 0;
            double best = -1.0;
            for (int i = 0; i < activeParts; ++i) {
                const double hsI = SeaState::SignificantHeight(&parts[i], 1);
                if (hsI > best) {
                    best = hsI;
                    pk = i;
                }
            }
            m_cPeak = static_cast<float>(9.81 / (2.0 * 3.14159265 * parts[pk].fp));
            m_peakDirX = parts[pk].dirToX;
            m_peakDirZ = parts[pk].dirToZ;
            m_peakDirValid = true;   // M7j: the shadow may only march a REAL direction
        }

        char note[32];
        if (m_stormHs > 0.01f) {
            snprintf(note, sizeof(note), "STORM %.1fm/%.0fs", m_stormHs, m_stormTp);
        } else {
            snprintf(note, sizeof(note), "f%03d", static_cast<int>(m_sea->Hour(hour).fh));
        }
        statusNote = note;

        // Whitecap COVERAGE follows wind speed steeply (Monahan's ~U^3.4 law); the Jacobian only
        // decides WHERE foam sits. Without this gate, the 2-second chop that a 4 m/s breeze
        // raises micro-breaks all over the render at full brightness -- geometrically true,
        // visually absurd (real coverage at 4 m/s is ~0.01%).
        double windMs = m_sea->Hour(hour).windMs;
        if (m_stormHs > 0.01f) {
            // Implied wind for the sandbox storm: the fully-developed speed for its wind sea.
            windMs = 9.81 * (m_stormTp * 0.55) / (2.0 * 3.14159265) * 0.85 * 2.0;
        }
        const double t = std::max(0.0, (windMs - 3.0) / 9.0);
        m_windGate = static_cast<float>(std::min(1.0, std::pow(t, 1.5)));

        // ---- M6t: GRADE SHEDDING WITH CONSERVATION. Each cascade's mean-square slope,
        // integrated from the SAME model spectrum the plot draws (deep-water k = (2 pi f)^2/g,
        // banded by the synthesis cuts). When a pixel's footprint can no longer resolve a
        // band's phase, the band's variance does not vanish -- it descends from geometry into
        // the BRDF's slope variance (the rotor sheds to grade 0; energy changes grade, never
        // disappears). The floor is calibrated so the fully-shed sum equals the globe's
        // Cox-Munk sigma^2(wind): at altitude the two water descriptions become THE SAME
        // pixel, and the old hand-tuned fade distances (which deleted the energy) retire.
        {
            const double kPiD = 3.14159265358979;
            const double kCut[4] = {2.0 * kPiD / 756.0, 2.0 * kPiD / 60.0, 2.0 * kPiD / 12.0,
                                    0.9 * kPiD * OceanFft::kN / 47.0};
            double mss[3] = {0, 0, 0};
            double m0b[3] = {0, 0, 0};   // M8 foamlaw: banded amplitude variance too
            const double df = 0.004;
            for (double f = df; f < 2.0; f += df) {
                const double k = (2.0 * kPiD * f) * (2.0 * kPiD * f) / 9.81;
                const double s = SeaState::SpectrumAt(parts, activeParts, f);
                for (int c = 0; c < 3; ++c) {
                    if (k >= kCut[c] && k < kCut[c + 1]) {
                        mss[c] += k * k * s * df;
                        m0b[c] += s * df;
                    }
                }
            }
            // M8 foamlaw: the unit-sea rms ENVELOPE per band, rms = sqrt(sum a^2) =
            // sqrt(2 m0), exaggerated like the geometry. The bank kernel scales it by
            // its per-texel band gains to get the local envelope the depth-excess
            // trigger and the crest gate normalize against (test the ENVELOPE, never
            // instantaneous |eta| -- the television-static lesson).
            for (int c = 0; c < 3; ++c) {
                m_bandRms[c] = static_cast<float>(std::sqrt(2.0 * m0b[c]) * heightScale);
            }
            const double ex2 = heightScale * heightScale;   // geometry is exaggerated; the
                                                            // shed variance must match it
            const double coxMunk = 0.003 + 0.00512 * windMs;
            double sum = 0.0;
            for (int c = 0; c < 3; ++c) {
                m_seaCb.bandSig[c] = static_cast<float>(mss[c] * ex2);
                sum += mss[c] * ex2;
            }
            m_seaCb.bandSig[3] = static_cast<float>(std::max(coxMunk - sum, 0.0015));
            Log("[sea] slope variance: bands %.4f/%.4f/%.4f + floor %.4f (Cox-Munk %.4f at "
                "%.1f m/s -- the far field is the same pixel the globe draws)",
                m_seaCb.bandSig[0], m_seaCb.bandSig[1], m_seaCb.bandSig[2], m_seaCb.bandSig[3],
                coxMunk, windMs);
        }

        // ---- spectrum plot: model + buoy on a shared axis
        const float fMax = 0.35f;
        const BuoyObs* buoy = m_sea->Buoy("44013");
        float sMax = 1e-3f;
        float model[kSpecSamples], meas[kSpecSamples];
        for (uint32_t i = 0; i < kSpecSamples; ++i) {
            const double f = fMax * i / (kSpecSamples - 1);
            model[i] = static_cast<float>(SeaState::SpectrumAt(parts, activeParts, f));
            sMax = std::max(sMax, model[i]);
            meas[i] = -1.0f;
            if (buoy && buoy->specFreqHz.size() > 1) {
                const auto& fq = buoy->specFreqHz;
                if (f >= fq.front() && f <= fq.back()) {
                    size_t k = 0;
                    while (k + 2 < fq.size() && fq[k + 1] < f) ++k;
                    const float t = static_cast<float>((f - fq[k]) / (fq[k + 1] - fq[k]));
                    meas[i] = buoy->specDens[k] + t * (buoy->specDens[k + 1] - buoy->specDens[k]);
                    sMax = std::max(sMax, meas[i]);
                }
            }
        }
        for (uint32_t i = 0; i < kSpecSamples; ++i) {
            m_specCb.model[i / 4][i % 4] = model[i];
            m_specCb.buoy[i / 4][i % 4] = meas[i];
        }
        m_specCb.rect[0] = 0.35f;
        m_specCb.rect[1] = -0.95f;
        m_specCb.rect[2] = 0.95f;
        m_specCb.rect[3] = -0.45f;
        m_specCb.axis[0] = fMax;
        m_specCb.axis[1] = sMax * 1.15f;
        m_specCb.axis[2] = static_cast<float>(kSpecSamples);
        m_specCb.axis[3] = (buoy && !buoy->specFreqHz.empty()) ? 1.0f : 0.0f;
        const float colM[4] = {2.2f, 1.1f, 0.2f, 1};
        const float colB[4] = {1.4f, 1.5f, 1.6f, 1};
        memcpy(m_specCb.colM, colM, sizeof(colM));
        memcpy(m_specCb.colB, colB, sizeof(colB));

        Log("[sea] %s %s: %d partitions, model Hs %.2f m (buoy 44013 %.2f m)",
            m_sea->CycleLabel().c_str(), statusNote.c_str(), activeParts, hsModel, hsBuoy);
    }

    m_tSec = static_cast<float>(simUnix - m_sea->CycleUnix());

    // Grid centre snapped to cascade-0 texels so vertices never swim against the texture.
    const double texel = m_fft.PatchL(0) / OceanFft::kN;
    m_seaCb.snap[0] = static_cast<float>(std::floor(camX / texel) * texel);
    m_seaCb.snap[1] = static_cast<float>(std::floor(camZ / texel) * texel);
    m_seaCb.sea[0] = static_cast<float>(seaLevelM);
    m_seaCb.sea[1] = 3600.0f;    // grid span, m
    m_seaCb.sea[2] = foamIntensity * m_windGate;
    m_seaCb.sea[3] = 0.80f;      // skirt start
    for (uint32_t c = 0; c < OceanFft::kCascades; ++c) {
        m_seaCb.dispSrv[c] = m_fft.DispSrv(c);
        m_seaCb.derivSrv[c] = m_fft.DerivSrv(c);
        m_seaCb.patchL[c] = m_fft.PatchL(c);
    }
    m_seaCb.patchL[3] = targetEdgePx;
    // M6t: the hand-tuned per-cascade fade DISTANCES are retired -- CascadeFade now folds each
    // band by the pixel's ground FOOTPRINT vs the band's wavelength (screen-resolution- and
    // zoom-aware), and the folded variance moves into the glint lobe instead of vanishing.
    // M6u: the row carries the model Hs instead (the far field's storm whitening -- the same
    // term the globe's ocean applies).
    m_seaCb.fadeD[0] = static_cast<float>(hsModel);
    m_seaCb.fadeD[1] = m_seaCb.fadeD[2] = 0.0f;

    // ---- M3: the entrance jet, live from the ACT0816 prediction clock
    double signedMs = 0.0;
    double floodDeg = 285.0, ebbDeg = 105.0;
    if (m_currents && m_ctSta >= 0) {
        signedMs = m_currents->SignedSpeed(static_cast<size_t>(m_ctSta), simUnix);
        const TidalCurrentStation& st = m_currents->S(static_cast<size_t>(m_ctSta));
        floodDeg = st.floodDeg;
        ebbDeg = st.ebbDeg;
        char cs[48];
        if (std::abs(signedMs) < 0.05) snprintf(cs, sizeof(cs), "slack");
        else snprintf(cs, sizeof(cs), "%s %.2f m/s", signedMs > 0 ? "flood" : "ebb",
                      std::abs(signedMs));
        currentStatus = cs;
    }
    const double d2r = 3.14159265358979 / 180.0;
    m_seaCb.jet[0] = static_cast<float>(signedMs);
    m_seaCb.jet[1] = 380.0f;
    m_seaCb.jet[2] = 1600.0f;
    m_seaCb.jet[3] = (m_currents && m_ctSta >= 0) ? 1.0f : 0.0f;
    m_seaCb.jetDir[0] = static_cast<float>(std::sin(floodDeg * d2r));
    m_seaCb.jetDir[1] = static_cast<float>(std::cos(floodDeg * d2r));
    m_seaCb.jetDir[2] = static_cast<float>(std::sin(ebbDeg * d2r));
    m_seaCb.jetDir[3] = static_cast<float>(std::cos(ebbDeg * d2r));
    // waveC.x carries the LOOK-side height exaggeration (band phase speeds now come from depth
    // + gBandK, so the old deep-water cPeak slot was free).
    m_seaCb.waveC[0] = heightScale;
    m_seaCb.waveC[1] = m_peakDirX;
    m_seaCb.waveC[2] = m_peakDirZ;
    m_seaCb.waveC[3] = static_cast<float>(std::fmod(m_tSec, 1024.0f));

    // Representative wavenumber per cascade band (geometric mid of the same 60 m / 12 m cuts
    // OceanFft uses). The SHADER turns these into phase speeds at the local depth -- in deep
    // water ~18/6.5/1.9 m/s, but an 11 s swell over the 4 m bar drops to ~6 m/s, which is why
    // the ebb stands the entrance up (M5b).
    const double kPiD = 3.14159265358979;
    const double kCut[4] = {2.0 * kPiD / 756.0, 2.0 * kPiD / 60.0, 2.0 * kPiD / 12.0,
                            0.9 * kPiD * OceanFft::kN / 47.0};
    for (int c = 0; c < 3; ++c) {
        m_seaCb.bandK[c] = static_cast<float>(std::sqrt(kCut[c] * kCut[c + 1]));
    }
    m_seaCb.bandK[3] = 0.0f;

    // ---- M4: churn atlas bindings + residency policy
    m_seaCb.churnU[0] = m_churnReady ? m_churn.Srv() : UINT32_MAX;
    m_seaCb.churnU[1] = m_churnReady ? m_maskTex.srv : UINT32_MAX;
    m_seaCb.churnU[2] = atlasVisualize ? 1u : 0u;
    m_seaCb.churnU[3] = 0;
    m_seaCb.churnF[0] = -0.5f * kChurnDomainM;
    m_seaCb.churnF[1] = -0.5f * kChurnDomainM;
    m_seaCb.churnF[2] = 1.0f / kChurnDomainM;
    m_seaCb.churnF[3] = 1.05f;   // churn -> foam gain
    if (m_churnReady) {
        m_seaCb.churnF2[0] = static_cast<float>(m_churn.TileW()) * kChurnTexelM;
        m_seaCb.churnF2[1] = static_cast<float>(m_churn.TileH()) * kChurnTexelM;
        m_seaCb.churnF2[2] = static_cast<float>(m_churn.TilesX());
        m_seaCb.churnF2[3] = static_cast<float>(m_churn.TilesY());
        UpdateChurnResidency(*m_gpu, simUnix, signedMs);
    }

    m_seaCb.bathyU[0] = m_bathySrv;
    memcpy(m_seaCb.bathyGeo, m_bathyGeo, sizeof(m_bathyGeo));

    // ---- M5c: swell-shadow mask -- rebuilt when the peak wave direction or the water level
    // moves enough to change what blocks the sea (a bar that shadows at low water drowns at
    // high water).
    // M7j: the hypervisor's first catch -- the mask could build from the DEFAULT peak
    // direction (toward east) before partitions loaded: marching "toward the source" then
    // walked WEST into the dunes from every ocean cell, and the whole sea read as deep
    // shadow. No real direction, no shadow (exposure 1 until the swell is known).
    if (m_bathyCpu && m_bathyCpu->Ready() && m_peakDirValid) {
        const float lvl = static_cast<float>(seaLevelM);
        const float dirDot = m_peakDirX * m_shadowDirX + m_peakDirZ * m_shadowDirZ;
        if (!m_shadowBuilt || dirDot < 0.98f || std::abs(lvl - m_shadowLevel) > 0.5f) {
            BuildShadowMask(*m_gpu, lvl);
        }
    }

    // ---- M5c: solver bindings. NULL-tile eta reads = "the tide plane is right here".
    const bool sweOn = m_swe && m_swe->Ready();
    m_seaCb.sweU[0] = sweOn ? m_swe->EtaSrv() : UINT32_MAX;
    m_seaCb.sweU[1] = sweOn ? m_swe->UvSrv() : UINT32_MAX;
    m_seaCb.sweU[2] = sweOn ? 1u : 0u;
    m_seaCb.sweU[3] = m_shadowBuilt ? m_shadowTex.srv : UINT32_MAX;
    if (sweOn) {
        m_seaCb.sweF[0] = static_cast<float>(m_swe->Nx());
        m_seaCb.sweF[1] = static_cast<float>(m_swe->Ny());
        m_seaCb.sweF[2] = 1.0f / m_swe->PadW();
        m_seaCb.sweF[3] = 1.0f / m_swe->PadH();
        if (!m_swe->stats.empty()) atlasStats += "  " + m_swe->stats;
    }
    m_seaCb.sweG[0] = sweCurrentGain;
    m_simUnix = simUnix;
    m_haveData = true;
}

// The march: from each water cell, walk TOWARD the peak-wave source; anything standing above
// the water line en route (jetty crest, Plum Island, an exposed bar) throws this cell into its
// geometric shadow. Two box blurs give the penumbra a wavelength-ish softness. This is
// line-of-sight, not diffraction -- honest about what it is, and it reads right: calm in the
// lee of the north jetty while the bar outside stays violent.
void SeaLayer::BuildShadowMask(Gpu& gpu, float waterNavd) {
    const BathyModel& bm = *m_bathyCpu;
    const int nx = bm.Nx(), ny = bm.Ny();
    const float cellX = bm.WorldSizeX() / nx, cellZ = bm.WorldSizeZ() / ny;
    const auto& elev = bm.Elev();

    // March in grid space (row 0 = north). Step ~2 cells; nearest-sample the bed directly.
    const float stepM = 13.0f;
    const float sx = -m_peakDirX * stepM / cellX;          // grid x per step
    const float sy = +m_peakDirZ * stepM / cellZ;          // grid y grows SOUTH; -(-dirZ)
    const int maxSteps = static_cast<int>(4000.0f / stepM);

    // Graded blocking: an awash bar (crest barely above water) BREAKS the swell and transmits a
    // reduced sea; only real walls -- jetty crests, dunes, the island -- throw a deep shadow.
    std::vector<float> mask(kShadowN * kShadowN, 1.0f);
    for (uint32_t my = 0; my < kShadowN; ++my) {
        for (uint32_t mx = 0; mx < kShadowN; ++mx) {
            const float gx0 = (mx + 0.5f) / kShadowN * nx;
            const float gy0 = (my + 0.5f) / kShadowN * ny;
            const float e0 = elev[static_cast<int>(gy0) * nx + static_cast<int>(gx0)];
            if (e0 > -9000.0f && e0 > waterNavd) continue;   // land cell; value never sampled
            float gx = gx0, gy = gy0;
            float excess = 0.0f;   // worst blocker height above the water line en route
            for (int s = 0; s < maxSteps; ++s) {
                gx += sx;
                gy += sy;
                const int ix = static_cast<int>(gx), iy = static_cast<int>(gy);
                if (ix < 0 || iy < 0 || ix >= nx || iy >= ny) break;   // open water: exposed
                const float e = elev[iy * nx + ix];
                if (e > -9000.0f) excess = std::max(excess, e - waterNavd);
                if (excess > 1.2f) break;   // already a full wall; no need to keep marching
            }
            if (excess > 0.15f) {
                // 0.15 m awash -> 0.5 transmission, ramping down to the 0.12 deep-shadow floor
                // by a 1.2 m wall.
                const float t = std::min((excess - 0.15f) / 1.05f, 1.0f);
                mask[my * kShadowN + mx] = 0.5f - 0.38f * t;
            }
        }
    }
    for (int pass = 0; pass < 2; ++pass) {
        std::vector<float> sm = mask;
        for (int y = 1; y < static_cast<int>(kShadowN) - 1; ++y) {
            for (int x = 1; x < static_cast<int>(kShadowN) - 1; ++x) {
                float a = 0;
                for (int dy = -1; dy <= 1; ++dy) {
                    for (int dx2 = -1; dx2 <= 1; ++dx2) {
                        a += mask[(y + dy) * kShadowN + (x + dx2)];
                    }
                }
                sm[y * kShadowN + x] = a / 9.0f;
            }
        }
        mask.swap(sm);
    }

    m_shadowCpu.resize(mask.size());
    for (size_t i = 0; i < mask.size(); ++i) {
        m_shadowCpu[i] = static_cast<uint8_t>(std::clamp(mask[i], 0.0f, 1.0f) * 255.0f + 0.5f);
    }
    if (!m_shadowTex.Valid()) {
        m_shadowTex = gpu.CreateTexture2D(kShadowN, kShadowN, DXGI_FORMAT_R8_UNORM,
                                          D3D12_RESOURCE_FLAG_NONE, D3D12_RESOURCE_STATE_COPY_DEST,
                                          L"sea.swellShadow");
    }
    gpu.UploadTexture(m_shadowTex, m_shadowCpu.data(), kShadowN);
    // The domain shader reads it too: PIXEL alone is not a legal state for that.
    ID3D12GraphicsCommandList* cl = gpu.BeginUpload();
    gpu.Transition(cl, m_shadowTex, D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE |
                                        D3D12_RESOURCE_STATE_PIXEL_SHADER_RESOURCE);
    gpu.EndUpload();
    if (m_shadowTex.srv == UINT32_MAX) {
        m_shadowTex.srv = gpu.CreateSrv(m_shadowTex.res.Get(), DXGI_FORMAT_R8_UNORM);
    }

    m_shadowDirX = m_peakDirX;
    m_shadowDirZ = m_peakDirZ;
    m_shadowLevel = waterNavd;
    m_shadowBuilt = true;
    Log("[sea] swell-shadow mask rebuilt (dir %.2f,%.2f water %.2f NAVD)", m_peakDirX,
        m_peakDirZ, waterNavd);
}

void SeaLayer::UpdateChurnResidency(Gpu& gpu, double simUnix, double signedMs) {
    if (!m_churnReady) return;
    const double dtSim = simUnix - m_prevChurnT;
    m_prevChurnT = simUnix;
    if (std::abs(dtSim) > 3600.0) {
        // A scrub jump: the memory this field carries belongs to a time that no longer exists.
        m_churn.RequestUnmapAll();
        std::fill(m_lastActive.begin(), m_lastActive.end(), -1.0e18);
        m_churnDt = 0.0f;
    } else {
        m_churnDt = static_cast<float>(std::clamp(dtSim, 0.0, 30.0));
    }

    // Map every tile whose worst-case opposing current can block the chop band; keep tiles warm
    // for several decay constants after the breaking stops so lingering wash stays visible.
    // Deep-water chop speed as the conservative onset (shallow water only blocks EARLIER).
    const float bandC2 = 1.86f;
    const double onset = 0.16 * bandC2;
    if (std::abs(signedMs) > onset) {
        const float fx = m_seaCb.jetDir[0], fz = m_seaCb.jetDir[1];
        const float ex = m_seaCb.jetDir[2], ez = m_seaCb.jetDir[3];
        for (uint32_t ty = 0; ty < m_churn.TilesY(); ++ty) {
            for (uint32_t tx = 0; tx < m_churn.TilesX(); ++tx) {
                const float wx = m_seaCb.churnF[0] +
                                 (tx + 0.5f) * m_seaCb.churnF2[0];
                const float wz = m_seaCb.churnF[1] +
                                 (ty + 0.5f) * m_seaCb.churnF2[1];
                const float along = wx * ex + wz * ez;
                const float px = wx - along * ex, pz = wz - along * ez;
                const float cross2 = px * px + pz * pz;
                float env = std::exp(-cross2 / (m_seaCb.jet[1] * m_seaCb.jet[1]));
                env *= (along > 0) ? std::exp(-along / m_seaCb.jet[2]) : 1.0f;
                (void)fx; (void)fz;
                // M5c: a WIDE superset -- the deposit is now geographic (solved currents +
                // depth), so an over-mapped tile just holds zeros, while an under-mapped one
                // punches a visible NULL rectangle into the middle of real breaking.
                if (env * std::abs(signedMs) > onset * 0.35) {
                    const uint32_t i = ty * m_churn.TilesX() + tx;
                    m_lastActive[i] = simUnix;
                    if (!m_churn.IsResident(tx, ty)) m_churn.RequestMap(tx, ty);
                }
            }
        }
    }
    for (uint32_t i = 0; i < m_lastActive.size(); ++i) {
        const uint32_t tx = i % m_churn.TilesX(), ty = i / m_churn.TilesX();
        if (m_churn.IsResident(tx, ty) && simUnix - m_lastActive[i] > 8.0 * kChurnTau) {
            m_churn.RequestUnmap(tx, ty);
        }
    }

    const uint32_t before = m_churn.ResidentCount();
    std::vector<uint32_t> fresh;
    m_churn.CommitMappings(gpu, &fresh);
    for (uint32_t f : fresh) m_pendingClear.push_back(f);
    if (m_churn.ResidentCount() != before || !fresh.empty()) m_maskDirty = true;

    if (m_maskDirty) {
        for (uint32_t i = 0; i < m_maskCpu.size(); ++i) {
            m_maskCpu[i] = m_churn.IsResident(i % m_churn.TilesX(), i / m_churn.TilesX()) ? 255
                                                                                          : 0;
        }
        gpu.UploadTexture(m_maskTex, m_maskCpu.data(), m_churn.TilesX());
        m_maskDirty = false;
    }

    char stats[96];
    snprintf(stats, sizeof(stats), "churn %u/%u t %.1f/%.0f MB", m_churn.ResidentCount(),
             m_churn.TilesX() * m_churn.TilesY(), m_churn.ResidentBytes() / 1048576.0,
             m_churn.VirtualBytes() / 1048576.0);
    atlasStats = stats;
}

void SeaLayer::RecordChurn(const FrameContext& ctx) {
    if (!m_churnReady) return;
    PixScope scope(ctx.cl, "sea.churn (sparse stateful atlas: clear fresh, decay+deposit)");

    // M5c late wiring: the solver is attached after Init, so its textures land in the table on
    // first use (overwriting the null views).
    if (!m_churnSweWired && m_swe && m_swe->Ready()) {
        D3D12_SHADER_RESOURCE_VIEW_DESC sv{};
        sv.Shader4ComponentMapping = D3D12_DEFAULT_SHADER_4_COMPONENT_MAPPING;
        sv.Format = DXGI_FORMAT_R16G16B16A16_FLOAT;
        sv.ViewDimension = D3D12_SRV_DIMENSION_TEXTURE2D;
        sv.Texture2D.MipLevels = 1;
        ctx.gpu->Device()->CreateShaderResourceView(m_swe->UvRes(), &sv,
                                                    ctx.gpu->SrvHeap().Cpu(m_churnTable + 1));
        sv.Format = DXGI_FORMAT_R32_FLOAT;
        ctx.gpu->Device()->CreateShaderResourceView(m_swe->BathyRes(), &sv,
                                                    ctx.gpu->SrvHeap().Cpu(m_churnTable + 2));
        m_churnSweWired = true;
    }

    auto barrierTo = [&](D3D12_RESOURCE_STATES to) {
        if (m_churnState == to) return;
        D3D12_RESOURCE_BARRIER b{};
        b.Type = D3D12_RESOURCE_BARRIER_TYPE_TRANSITION;
        b.Transition.pResource = m_churn.Res();
        b.Transition.Subresource = D3D12_RESOURCE_BARRIER_ALL_SUBRESOURCES;
        b.Transition.StateBefore = m_churnState;
        b.Transition.StateAfter = to;
        ctx.cl->ResourceBarrier(1, &b);
        m_churnState = to;
    };

    if (!m_churn.ResidentList().empty() || !m_pendingClear.empty()) {
        barrierTo(D3D12_RESOURCE_STATE_UNORDERED_ACCESS);

        m_churnCb.originX = m_seaCb.churnF[0];
        m_churnCb.originZ = m_seaCb.churnF[1];
        m_churnCb.texelM = kChurnTexelM;
        m_churnCb.domainM = kChurnDomainM;
        m_churnCb.tilesX = m_churn.TilesX();
        m_churnCb.tileW = m_churn.TileW();
        m_churnCb.tileH = m_churn.TileH();
        m_churnCb.dt = m_churnDt;
        m_churnCb.tau = static_cast<float>(kChurnTau);
        memcpy(m_churnCb.jetA, m_seaCb.jet, 16);
        memcpy(m_churnCb.jetB, m_seaCb.jetDir, 16);
        // M5c: the chop-band WAVENUMBER -- the kernel derives phase speed from the local depth,
        // exactly like Sea.hlsl (misc[0] was the deep-water speed when churn had no bathy).
        m_churnCb.misc[0] = m_seaCb.bandK[2];
        m_churnCb.misc[1] = m_fft.PatchL(2);
        m_churnCb.misc[2] = m_seaCb.waveC[3];
        m_churnCb.misc[3] = 0;
        memcpy(m_churnCb.bathyG, m_bathyGeo, sizeof(m_bathyGeo));
        m_churnCb.sweM[0] = m_churnSweWired ? 1.0f : 0.0f;
        m_churnCb.sweM[1] = sweCurrentGain;
        m_churnCb.sweM[2] = 500.0f;   // the same seaward handover ramp as Sea.hlsl's JetU
        m_churnCb.sweM[3] = 900.0f;
        m_churnCb.waveD[0] = m_seaCb.waveC[1];
        m_churnCb.waveD[1] = m_seaCb.waveC[2];
        m_churnCb.waveD[2] = 1.0f;
        m_churnCb.waveD[3] = 1.0f;

        ctx.cl->SetComputeRootSignature(m_churnRs.Get());

        auto dispatchList = [&](ID3D12PipelineState* pso, const std::vector<uint32_t>& list) {
            if (list.empty()) return;
            m_churnCb.listCount = static_cast<uint32_t>(list.size());
            ctx.cl->SetComputeRootConstantBufferView(
                0, ctx.gpu->PushConstants(&m_churnCb, sizeof(m_churnCb)));
            ctx.cl->SetComputeRootShaderResourceView(
                1, ctx.gpu->PushConstants(list.data(), list.size() * 4));
            ctx.cl->SetComputeRootDescriptorTable(2, ctx.gpu->SrvHeap().Gpu(m_churnTable));
            ctx.cl->SetPipelineState(pso);
            ctx.cl->Dispatch(m_churn.TileW() / 16, m_churn.TileH() / 16, m_churnCb.listCount);
        };

        if (!m_pendingClear.empty()) {
            PixMarker(ctx.cl, "churn.clearFresh (undefined pool memory -> zero)");
            dispatchList(m_churnClear.Get(), m_pendingClear);
            m_pendingClear.clear();
            D3D12_RESOURCE_BARRIER uav{};
            uav.Type = D3D12_RESOURCE_BARRIER_TYPE_UAV;
            uav.UAV.pResource = m_churn.Res();
            ctx.cl->ResourceBarrier(1, &uav);
        }
        dispatchList(m_churnUpdate.Get(), m_churn.ResidentList());
    }
    barrierTo(D3D12_RESOURCE_STATE_PIXEL_SHADER_RESOURCE);
}

void SeaLayer::Render(const FrameContext& ctx) {
    if (!m_haveData || !m_seaPso) return;

    // The compute chain records into the same command list; compute bindings do not disturb the
    // graphics root signature the Renderer already set.
    if (m_swe && m_swe->Ready()) {
        m_swe->Record(ctx.cl, *ctx.gpu, m_simUnix, m_seaCb.sea[0]);
    }
    m_fft.Record(ctx.cl, *ctx.gpu, m_tSec);
    RecordChurn(ctx);

    if (drawEnabled) {
        PixScope scope(ctx.cl, "sea.surface (tessellated: screen-space edge density)");
        ctx.cl->SetPipelineState(m_seaPso.Get());
        ctx.cl->IASetPrimitiveTopology(D3D_PRIMITIVE_TOPOLOGY_4_CONTROL_POINT_PATCHLIST);
        ctx.cl->SetGraphicsRootConstantBufferView(
            1, ctx.gpu->PushConstants(&m_seaCb, sizeof(m_seaCb)));
        ctx.cl->DrawInstanced(4 * kPatches * kPatches, 1, 0, 0);
    }
    if (drawEnabled) {
        PixScope scope(ctx.cl, "sea.spectrum (solid=model, dashed=buoy 44013)");
        ctx.cl->SetPipelineState(m_specPso.Get());
        ctx.cl->IASetPrimitiveTopology(D3D_PRIMITIVE_TOPOLOGY_LINESTRIP);
        ctx.cl->SetGraphicsRootConstantBufferView(
            1, ctx.gpu->PushConstants(&m_specCb, sizeof(m_specCb)));
        ctx.cl->DrawInstanced(kSpecSamples, 3, 0, 0);
    }
}

}  // namespace ga
