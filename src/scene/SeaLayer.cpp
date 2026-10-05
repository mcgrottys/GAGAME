#include "scene/SeaLayer.h"

#include "hal/GpuProfiler.h"

#include "hal/PixEvents.h"
#include "hal/Pipeline.h"
#include "hal/Root.h"
#include "hal/Views.h"
#include "scene/FieldSet.h"

#include <algorithm>
#include <cmath>

namespace ga {

void SeaLayer::Init(Gpu& gpu, ShaderCompiler& sc, FieldSet& fields,
                    hal::RootSignature rootSig) {
    (void)fields;
    (void)rootSig;
    if (!m_sea || !m_sea->Ready()) throw std::runtime_error("SeaLayer needs a SeaState");
    m_gpu = &gpu;
    m_fft.Init(gpu, sc, m_shaderDir);
    if (gpu.TiledTier() >= D3D12_TILED_RESOURCES_TIER_2) {
        InitChurn(gpu, sc);
    } else {
        Log("[sea] tiled tier < 2: churn atlas disabled (dense fallback not built)");
    }

    int nb = 0;
    hsBuoy = m_sea->BuoyHs(0.0, 1.0e300, &nb);   // PHASE C2: the mean of the file's own buoys
}

void SeaLayer::InitChurn(Gpu& gpu, ShaderCompiler& sc) {
    {
        // M9h: the bank as DATA. Aeration is one scalar, so grade 0 -- and saying so is what
        // lets a derived field ask the Cayley closure where a product with it can be non-zero.
        GradeBankDesc d;
        d.name = "sea.churn (sparse stateful G0 bank)";
        d.width = static_cast<uint32_t>(kChurnDomainM / kChurnTexelM);
        d.height = d.width;
        d.fmt = DXGI_FORMAT_R16_FLOAT;
        d.gradeSig = kG0;
        d.metersPerTexel = kChurnTexelM;
        d.vNorth = true;
        d.units = "0..1 aeration";
        d.range = "0..1";
        // policy::None(): this bank is driven by UpdateChurnResidency, not by GradeBank::Update.
        // Declaring an unused policy that lied about the residency would be worse than none.
        m_churn.Init(gpu, d, policy::None());
    }
    m_lastActive.assign(static_cast<size_t>(m_churn.TilesX()) * m_churn.TilesY(), -1.0e18);

    // Root signature: b0 CBV, t0 root SRV (tile list from the frame arena), table
    // [t1 chop-deriv, t2 swe uv, t3 bathy, t4 its residency map (M9ar), u0 churn], s0 wrap +
    // s1 clamp samplers.
    m_churnRs = hal::RootLayout{}
                    .Cbv(0)
                    .Srv(0)
                    .Table({hal::SrvRange(1, 4), hal::UavRange(0, 1)})
                    .Sampler(hal::StaticSampler(0, D3D12_FILTER_MIN_MAG_MIP_LINEAR,
                                                D3D12_TEXTURE_ADDRESS_MODE_WRAP))
                    .Sampler(hal::StaticSampler(1, D3D12_FILTER_MIN_MAG_MIP_LINEAR,
                                                D3D12_TEXTURE_ADDRESS_MODE_CLAMP))
                    .Build(gpu, "sea.churn");
    m_churnRs->SetName(L"sea.churn root signature");

    auto makePso = [&](const wchar_t* entry, hal::Pso& out) {
        out = hal::Require(
            hal::BuildCompute(gpu, m_churnRs.Get(),
                              sc.Compile(m_shaderDir + L"/SeaChurn.hlsl", entry, L"cs_6_0"),
                              "sea.churn"),
            "SeaChurn kernel");
        out->SetName(entry);
    };
    makePso(L"CsChurnClear", m_churnClear);
    makePso(L"CsChurnUpdate", m_churnUpdate);

    m_churnTable = hal::Table::Alloc(gpu, 5, "sea.churn");   // M9ar: + the residency map slot
    m_churnTable.Srv2D(0, m_fft.DerivRes(2), DXGI_FORMAT_R16G16B16A16_FLOAT);
    // t2 (solved currents) and t3 (bathy) start as NULL views -- they read zeros, which the
    // kernel's gSweM.x gate never touches -- and are wired in RecordChurn once the solver
    // exists (main attaches it after this layer's Init).
    m_churnTable.Srv2D(1, nullptr, DXGI_FORMAT_R16G16B16A16_FLOAT);
    // M9ar: t3/t4 are Texture2DArray in the kernel; null views must say so. (Until step 3d
    // t3 was first written as a Texture2D R32F null view and then, before any use, as this
    // one; the dead write is gone.)
    m_churnTable.SrvArray(2, nullptr, DXGI_FORMAT_R16_FLOAT, 0, 1);
    m_churnTable.SrvArray(3, nullptr, DXGI_FORMAT_R8_UNORM, 0, 1);
    m_churnTable.Uav2D(4, m_churn.Res(), DXGI_FORMAT_R16_FLOAT);

    m_maskCpu.assign(static_cast<size_t>(m_churn.TilesX()) * m_churn.TilesY(), 0);
    m_maskTex = gpu.CreateTexture2D(m_churn.TilesX(), m_churn.TilesY(), DXGI_FORMAT_R8_UNORM,
                                    D3D12_RESOURCE_FLAG_NONE, D3D12_RESOURCE_STATE_COPY_DEST,
                                    L"sea.churn.residencyMask");
    gpu.UploadTexture(m_maskTex, m_maskCpu.data(), m_churn.TilesX());
    m_maskTex.srv = gpu.CreateSrv(m_maskTex.res.Get(), DXGI_FORMAT_R8_UNORM);
    m_churnReady = true;
}

void SeaLayer::ReloadShaders(Gpu& gpu, ShaderCompiler& sc) {
    if (!m_fft.ReloadShaders(gpu, sc)) Log("[sea] ocean compute reload failed; keeping previous");
}

void SeaLayer::SetStorm(float hs, float tp, float fromDeg) {
    m_stormHs = hs;
    m_stormTp = tp;
    m_stormDir = fromDeg;
    m_lastHour = -1;   // force a respectrum on the next SetTime
}

int SeaLayer::PartsOf(const SeaState& src, double simUnix, bool storm, PartParam parts[4],
                      bool log) const {
    // PHASE C2: ONE LAW FOR EVERY SEA-STATE SOURCE -- the partitions a file's forecast hour gives, the
    // wind sea filled where it has none, and its own buoys' reading assimilated; or, for the source a
    // declared storm replaces (data.seastate's), the storm. The synthesis takes data.seastate's; the
    // field (WaveScale) takes every source's Hs from the same function, so where data.seastate's own
    // box holds a texel its scale is 1 by construction.
    int activeParts = 0;
    const int hour = src.HourIndex(simUnix);
    if (storm) {
            // Sandbox storm: the named swell carries 85% of the energy, a fresher wind sea 25%
            // of the height on a slightly shorter period, 30 degrees off.
            parts[0] = SeaState::MakePartition(m_stormHs * 0.92, m_stormTp, m_stormDir, false);
            parts[1] = SeaState::MakePartition(m_stormHs * 0.40, m_stormTp * 0.55,
                                               m_stormDir + 30.0, true);
            activeParts = 2;
    } else {
            activeParts = src.BuildParams(hour, parts);
            // M9a: THE MISSING WIND SEA. On light-wind hours GFS-Wave's partitioning hands
            // back swell trains ONLY -- and a swell partition is a Gaussian of
            // sigF = clamp(0.10/Tp^2, 0.004, 0.02) Hz: ~4 mHz wide, with no tail at all. So
            // every last joule lands in cascade 0 and cascades 1-2 realize NUMERICALLY ZERO
            // (measured at 12z f004: band Hs 0.357 / 1e-12 / 0.000 m, mss 3.0e-5 / 1.6e-25
            // / 0). The water then renders as poured glass -- no short faces to carry
            // normals, and the only sub-cascade roughness left is bandSig[3], which widens
            // the glint lobe and never moves a normal (priors 8: normals sell amplitude).
            // But the wind that raises those ripples is already sitting in the same file.
            // So synthesise the fully-developed sea for it and let it ride the partition
            // list like any other train -- the shapes, the spreading, the plot, the buoy
            // assimilation and the GPU all pick it up unchanged. Capped at the forecast's
            // own combined Hs: PM is the fetch-UNLIMITED answer and no coastal hour is that.
            bool hasWindSea = false;
            for (int i = 0; i < activeParts; ++i) hasWindSea |= parts[i].gamma > 0.0f;
            const SeaHour& hr = src.Hour(hour);
            double pmHs = 0.0, pmTp = 0.0;
            if (!hasWindSea && activeParts < 4 && windSeaFill > 0.0f &&
                SeaState::WindSeaPm(hr.windMs, &pmHs, &pmTp)) {
                pmHs *= windSeaFill;
                if (hr.combinedHs > 0.02) pmHs = std::min(pmHs, hr.combinedHs);
                if (pmHs > 0.02) {
                    parts[activeParts++] =
                        SeaState::MakePartition(pmHs, pmTp, hr.windFromDeg, true);
                    if (log) Log("[sea] f%03d has no wind-sea partition (wind %.1f m/s): filled with "
                        "Pierson-Moskowitz Hs %.2f m Tp %.1f s (lambda %.1f m) -- cascades "
                        "1-2 were exactly zero",
                        static_cast<int>(hr.fh), hr.windMs, pmHs, pmTp, 1.56 * pmTp * pmTp);
                }
            }
        }
    // M8 BUOY ASSIMILATION (the user's call: the single-point measurements should STEER the model,
    // not just grade it). One scalar on every partition's energy BEFORE synthesis: the measured Hs
    // over the forecast's, clamped [1/gmax, gmax]; never under a storm; only fresh observations.
    // PHASE C2: the measurement is the source's OWN stations -- the mean of the file's buoys that
    // are fresh (< buoyAssimAgeH) -- not one buoy named in code.
    if (!storm && activeParts > 0) {
        int nb = 0;
        const double hsObs = src.BuoyHs(simUnix, buoyAssimAgeH * 3600.0, &nb);
        const double hsFc = SeaState::SignificantHeight(parts, activeParts);
        if (nb > 0 && hsObs > 0.05 && hsFc > 0.05) {
            const double g = std::clamp(hsObs / hsFc, 1.0 / buoyAssimGainMax,
                                        static_cast<double>(buoyAssimGainMax));
            for (int i = 0; i < activeParts; ++i) {
                parts[i].specScale *= static_cast<float>(g * g);   // energy ~ Hs^2
            }
            if (log) {
                Log("[sea] %s: %d buoy(s) assimilated: Hs %.2f obs (their mean) vs %.2f forecast "
                    "-> gain %.2f on every partition",
                    src.Label().c_str(), nb, hsObs, hsFc, g);
            }
        }
    }
    return activeParts;
}

void SeaLayer::SetTime(double simUnix, double seaLevelM, double camX, double camZ) {
    if (!m_sea || !m_sea->Ready()) return;

    const int hour = m_sea->HourIndex(simUnix);
    if (hour != m_lastHour) {
        m_lastHour = hour;
        PartParam parts[4];
        activeParts = PartsOf(*m_sea, simUnix, StormOn(), parts, true);
        const uint32_t seed = static_cast<uint32_t>(m_sea->CycleUnix() / 3600.0) * 2654435761u;
        m_fft.SetSeaState(parts, activeParts, seed);
        // M9bq: THE CPU TWIN IS FED FROM THE SAME CALL SITE, with the same partitions and the
        // same seed, so the two processors cannot come to disagree about what sea this is. A
        // hull querying a separately-configured OceanCpu would ride a statistically identical
        // and physically DIFFERENT ocean -- right Hs, wrong crests -- and nothing would ever
        // look wrong enough to investigate.
        {
            const float pl[3] = {m_fft.PatchL(0), m_fft.PatchL(1), m_fft.PatchL(2)};
            const float lo[3] = {m_fft.BandLo(0), m_fft.BandLo(1), m_fft.BandLo(2)};
            const float hi[3] = {m_fft.BandHi(0), m_fft.BandHi(1), m_fft.BandHi(2)};
            m_oceanCpu.SetSeaState(parts, activeParts, seed, pl, lo, hi, m_fft.Lambda());
        }
        hsModel = SeaState::SignificantHeight(parts, activeParts);
        // PHASE C2: THE SEA STATE AS A FIELD (sim/WaveScale.h). The reference is the Hs the cascades
        // were synthesised for; every source with a place (its file's box, or sea.box for
        // data.seastate's) carries its own Hs by the same law (PartsOf), at its own clock.
        m_scale.hsRef = hsModel > 1e-3 ? hsModel : 1.0;
        m_scale.storm = StormOn();   // a declared storm is the planet's sea (WaveScale.h)
        m_scale.sources.clear();
        for (const SeaState* src : m_sources) {
            if (!src || !src->Ready() || !src->HasBox()) continue;
            PartParam sp[4];
            const int n = PartsOf(*src, simUnix, src == m_sea && StormOn(), sp, false);
            WaveScale::Source fs;
            src->Box(fs.box);
            fs.hs = SeaState::SignificantHeight(sp, n);
            m_scale.sources.push_back(fs);
            Log("[sea] field source %s: box %.3f..%.3f N x %.3f..%.3f E, Hs %.3f m (scale %.3f over "
                "the reference %.3f m)%s",
                src->Label().c_str(), fs.box[0], fs.box[2], fs.box[1], fs.box[3], fs.hs,
                fs.hs / m_scale.hsRef, m_scale.hsRef, src == m_sea && StormOn() ? " -- the storm" : "");
        }
        for (int i = 0; i < 4; ++i) m_parts[i] = (i < activeParts) ? parts[i] : PartParam{};
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
            // M9e: THE IMPLIED WIND for the sandbox storm. This drives the whitecap gate AND
            // -- through Cox-Munk sigma^2 = 0.003 + 0.00512 U -- the width of the glint lobe,
            // which is what a storm sea is mostly MADE of visually.
            //
            // The old closure was g(0.55 Tp)/2pi * 1.7, i.e. the wind-sea partition's deep
            // phase speed times 1.70. Two things were wrong with it. It ignored Hs entirely,
            // so --storm 0.5,10 and --storm 6.0,10 implied the SAME wind; and at Tp 10 it
            // returned 14.6 m/s where both physical anchors for a 3 m / 10 s sea say ~9-10
            // (PM inversion from Hs: 10.3; wave age 1 on the 5.5 s wind sea: 9.1). Cox-Munk
            // then ran 0.0777 instead of ~0.056 and the ebb ride's steep faces reflected the
            // pale horizon sky over their whole area -- the sea went white (measured: it is
            // NOT foam; ring-0 foam coverage is 3.6% over 0.5).
            //
            // Invert Pierson-Moskowitz on the height the caller actually asked for --
            // Hs = 0.0246 U19.5^2 -- and convert the 19.5 m anemometer height to U10 with the
            // same 1.075 log-profile ratio SeaState::WindSeaPm uses. One anchor, the one the
            // render is held to, and it scales with Hs the way a wind must.
            const double u195 = std::sqrt((std::max)(m_stormHs, 0.01f) / 0.0246);
            windMs = std::clamp(u195 / 1.075, 0.0, 40.0);
            Log("[sea] storm %.1f m / %.0f s -> implied wind %.1f m/s (PM inversion; the old "
                "period-only closure gave %.1f)",
                m_stormHs, m_stormTp, windMs,
                9.81 * (m_stormTp * 0.55) / (2.0 * 3.14159265) * 0.85 * 2.0);
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
            double lnk[3] = {0, 0, 0};   // M9c: energy-weighted sum of ln k, for the FOLD
            double lnk2[3] = {0, 0, 0};  // M9bt: and its SECOND moment -- the band's WIDTH
            const double df = 0.004;
            for (double f = df; f < 2.0; f += df) {
                const double k = (2.0 * kPiD * f) * (2.0 * kPiD * f) / 9.81;
                const double s = SeaState::SpectrumAt(parts, activeParts, f);
                for (int c = 0; c < 3; ++c) {
                    if (k >= kCut[c] && k < kCut[c + 1]) {
                        mss[c] += k * k * s * df;
                        m0b[c] += s * df;
                        lnk[c] += std::log(k) * s * df;
                        lnk2[c] += std::log(k) * std::log(k) * s * df;
                    }
                }
            }
            // ---- M9c: THE FOLD'S OWN WAVELENGTH. The fold asks "can this sampler resolve
            // the band's phase", and it has always answered with the geometric mean of the
            // band's CUTS -- a constant. But cascade 2 spans lambda 0.41..12 m, a 30x range
            // whose midpoint is 2.2 m, so a wind sea sitting at 4.6 m is discarded from
            // geometry as though it were 2.2 m. Measured: at U10 2.2 m/s the PM sea is
            // exactly there, and its band weight is 0.00 at the 2.4 m ring texel; folded on
            // its own wavelength it is 0.69.
            //
            // So weight by ENERGY, in log k (the generalization of the geometric mean the
            // cuts already gave, and the scale-free choice for a band this wide):
            //
            //     ln k_fold = integral(ln k . S df) / integral(S df)
            //
            // Clamped inside the band's own cuts, so no numerical excursion can move a band
            // outside itself. Empty bands keep the cut mean, which is what makes this a
            // no-op wherever there is no energy to weight by. This value feeds ONLY the fold
            // weight -- gBandK still carries the cut mean for phase speed, shoaling, and the
            // wave-current closure, whose gates and proofs pin the old number.
            for (int c = 0; c < 3; ++c) {
                const double kGeo = std::sqrt(kCut[c] * kCut[c + 1]);
                double kF = kGeo;
                if (m0b[c] > 1e-14) {
                    kF = std::clamp(std::exp(lnk[c] / m0b[c]), kCut[c], kCut[c + 1]);
                }
                // The closure is a LERP IN LOG K, so 0 restores the cut mean byte for byte
                // and the A/B is one number (priors 15: prove the wire, and a gain cannot
                // be silently swallowed the way a JSON bool was).
                const double g = std::clamp(static_cast<double>(bandFoldWeight), 0.0, 1.0);
                m_bandKFold[c] = static_cast<float>(
                    std::exp(std::log(kGeo) * (1.0 - g) + std::log(kF) * g));

                // ---- M9bt: THE BAND'S WIDTH, which the fold needs and never had. -----------
                // A cascade band is not a wavenumber, it is a DISTRIBUTION: band 2 spans
                // lambda 0.41..12 m, thirty to one. The fold has always judged the whole band
                // by its centre and admitted or shed it entire, which is a yes/no answer to a
                // question whose true answer is a fraction -- "how much of this band can a grid
                // of this spacing still carry". That fraction needs the second moment, so here
                // it is: the energy-weighted standard deviation of ln k, the natural partner to
                // the energy-weighted mean of ln k computed just above.
                //
                // An empty band falls back to the width its own cuts would have if the energy
                // were spread flat across them -- range over sqrt(12), the uniform
                // distribution's own standard deviation -- so this stays a no-op exactly where
                // the mean does.
                double var = 0.0;
                if (m0b[c] > 1e-14) {
                    const double mu = lnk[c] / m0b[c];
                    var = std::max(0.0, lnk2[c] / m0b[c] - mu * mu);
                }
                const double cutW = std::log(kCut[c + 1] / kCut[c]) / std::sqrt(12.0);
                m_bandKSpread[c] = static_cast<float>(
                    std::clamp((var > 1e-12) ? std::sqrt(var) : cutW, 0.05, 2.0));
            }
            Log("[sea] fold log-width per band: %.2f / %.2f / %.2f (energy-weighted sigma "
                "of ln k -- the fraction of each band Nyquist still admits)",
                m_bandKSpread[0], m_bandKSpread[1], m_bandKSpread[2]);
            Log("[sea] fold wavelength per band: %.1f / %.1f / %.2f m (cut means "
                "%.1f / %.1f / %.2f, weight %.2f)",
                6.283185307 / m_bandKFold[0], 6.283185307 / m_bandKFold[1],
                6.283185307 / m_bandKFold[2],
                6.283185307 / std::sqrt(kCut[0] * kCut[1]),
                6.283185307 / std::sqrt(kCut[1] * kCut[2]),
                6.283185307 / std::sqrt(kCut[2] * kCut[3]), bandFoldWeight);
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
            float bandSig[4];
            double sum = 0.0;
            for (int c = 0; c < 3; ++c) {
                bandSig[c] = static_cast<float>(mss[c] * ex2);
                sum += mss[c] * ex2;
            }
            bandSig[3] = static_cast<float>(std::max(coxMunk - sum, 0.0015));
            Log("[sea] slope variance: bands %.4f/%.4f/%.4f + floor %.4f (Cox-Munk %.4f at "
                "%.1f m/s -- the far field is the same pixel the globe draws)",
                bandSig[0], bandSig[1], bandSig[2], bandSig[3],
                coxMunk, windMs);
        }

        Log("[sea] %s %s: %d partitions, model Hs %.2f m (its buoys %.2f m)",
            m_sea->CycleLabel().c_str(), statusNote.c_str(), activeParts, hsModel, hsBuoy);
    }

    m_tSec = static_cast<float>(simUnix - m_sea->CycleUnix());

    m_seaLevel = static_cast<float>(seaLevelM);

    // Representative wavenumber per cascade band (geometric mid of the same 60 m / 12 m cuts
    // OceanFft uses). The SHADER turns these into phase speeds at the local depth -- in deep
    // water ~18/6.5/1.9 m/s, but an 11 s swell over the 4 m bar drops to ~6 m/s, which is why
    // the ebb stands the entrance up (M5b).
    const double kPiD = 3.14159265358979;
    const double kCut[4] = {2.0 * kPiD / 756.0, 2.0 * kPiD / 60.0, 2.0 * kPiD / 12.0,
                            0.9 * kPiD * OceanFft::kN / 47.0};
    m_chopK = static_cast<float>(std::sqrt(kCut[2] * kCut[3]));   // the chop band, the churn's

    // ---- M4: churn atlas bindings + residency policy
    // M9az: THE CHURN FOLLOWS THE CAMERA. The 16 km domain used to sit on the station: memory
    // could not exist +-8 km from Newburyport, Boston's window included. The atlas is now a
    // toroidal clipmap on a WORLD-anchored tile lattice -- the window's origin snaps to a tile
    // multiple like the wave bank's rings (ReanchorRing), a world tile lives at slot (tile mod
    // atlas tiles), so re-anchoring moves no bytes: content stays where the water is, the
    // slots whose world tile changed hands are cleared, and tiles that leave the window unmap.
    if (m_churnReady) {
        const float spanX = static_cast<float>(m_churn.TileW()) * kChurnTexelM;
        const float spanZ = static_cast<float>(m_churn.TileH()) * kChurnTexelM;
        const double half = 0.5 * kChurnDomainM;
        m_churnOrgX = static_cast<float>(std::floor((camX - half) / spanX) * spanX);
        m_churnOrgZ = static_cast<float>(std::floor((camZ - half) / spanZ) * spanZ);
        m_churnSpan[0] = spanX;
        m_churnSpan[1] = spanZ;
    }
    if (m_churnReady) UpdateChurnResidency(*m_gpu, simUnix);

    // M9ba: the swell exposure is a tree node driven from main (ExposureSource::Set on the
    // peak direction and the level); nothing is built here any more.

    if (m_swe && m_swe->Ready() && !m_swe->stats.empty()) atlasStats += "  " + m_swe->stats;
    m_simUnix = simUnix;
    m_haveData = true;
}

// The march: from each water cell, walk TOWARD the peak-wave source; anything standing above
// the water line en route (jetty crest, Plum Island, an exposed bar) throws this cell into its
// geometric shadow. Two box blurs give the penumbra a wavelength-ish softness. This is
// line-of-sight, not diffraction -- honest about what it is, and it reads right: calm in the
// lee of the north jetty while the bar outside stays violent.

void SeaLayer::UpdateChurnResidency(Gpu& gpu, double simUnix) {
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

    // PHASE C3b: map every tile the solver's current can reach -- a tile whose centre stands on the
    // solver's lattice over a bed that can be wet (+1.2 m NAVD, the residency law's own bound); keep
    // tiles warm for several decay constants after they leave so lingering wash stays visible. An
    // over-mapped tile just holds zeros (the deposit is the solved current + depth).
    const SweDomain* dom = (m_swe && m_swe->Domain().Ready() && m_surface) ? &m_swe->Domain() : nullptr;
    // M9az: walk the WINDOW's tiles (world lattice), not the atlas' slots. Each world tile maps
    // to one slot; a slot that held a different world tile last frame has changed hands.
    const uint32_t NX = m_churn.TilesX(), NY = m_churn.TilesY();
    const float spanX = m_churnSpan[0], spanZ = m_churnSpan[1];
    const int32_t T0x = static_cast<int32_t>(std::floor(m_churnOrgX / spanX));
    const int32_t T0y = static_cast<int32_t>(std::floor(m_churnOrgZ / spanZ));
    if (m_slotWorldX.size() != size_t(NX) * NY) {
        m_slotWorldX.assign(size_t(NX) * NY, INT32_MIN);
        m_slotWorldY.assign(size_t(NX) * NY, INT32_MIN);
    }
    auto slotOf = [&](int32_t Tx, int32_t Ty, uint32_t& sx, uint32_t& sy) {
        const int32_t mx = ((Tx % int32_t(NX)) + int32_t(NX)) % int32_t(NX);
        const int32_t my = ((Ty % int32_t(NY)) + int32_t(NY)) % int32_t(NY);
        sx = uint32_t(mx);
        sy = uint32_t(my);
    };
    for (uint32_t j = 0; j < NY; ++j) {
        for (uint32_t i = 0; i < NX; ++i) {
            const int32_t Tx = T0x + int32_t(i), Ty = T0y + int32_t(j);
            uint32_t sx, sy;
            slotOf(Tx, Ty, sx, sy);
            const uint32_t slot = sy * NX + sx;
            if (m_slotWorldX[slot] != Tx || m_slotWorldY[slot] != Ty) {
                // The slot changed hands: its bytes belong to water 16 km away.
                m_slotWorldX[slot] = Tx;
                m_slotWorldY[slot] = Ty;
                m_lastActive[slot] = -1.0e18;
                if (m_churn.IsResident(sx, sy)) m_pendingClear.push_back(slot);
            }
            if (!dom) continue;
            const double wx = (Tx + 0.5) * spanX, wz = (Ty + 0.5) * spanZ;
            const SurfaceFrame& sf = *m_surface;   // the tangent frame about the anchor's ground (ChurnPoint)
            const double drop = (wx * wx + wz * wz) / (2.0 * sf.planetR);
            double P[3], tx = -1.0, ty = -1.0;
            for (int c = 0; c < 3; ++c) P[c] = wx * sf.east[c] + (sf.planetR - drop) * sf.up[c] + wz * sf.north[c];
            if (!dom->CellOfDir(P, tx, ty) || tx < 0.0 || ty < 0.0 || tx >= dom->nx || ty >= dom->ny) continue;
            if (dom->elev[size_t(ty) * dom->nx + size_t(tx)] < 1.2f) {
                m_lastActive[slot] = simUnix;
                if (!m_churn.IsResident(sx, sy)) m_churn.RequestMap(sx, sy);
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
    PixScope scope(ctx.cmd->Native(),
                   "sea.churn (sparse stateful atlas: clear fresh, decay+deposit)");

    // M5c late wiring: the solver is attached after Init, so its textures land in the table on
    // first use (overwriting the null views).
    if (!m_churnSweWired && m_swe && m_swe->Ready()) {
        m_churnTable.Srv2D(1, m_swe->UvRes(), DXGI_FORMAT_R16G16B16A16_FLOAT);
        // M9ar: the bed is the height tenant (PHASE B3: its windows' slices), plus its residency map.
        if (m_hgtArr && m_hgtRes) {
            // M9ax: the whole tenant (cube faces + the page); the slice rides the CB.
            m_churnTable.SrvArray(2, m_hgtArr, DXGI_FORMAT_R16_FLOAT, 0, UINT32_MAX, m_hgtMips);
            m_churnTable.SrvArray(3, m_hgtRes, DXGI_FORMAT_R8_UNORM, 0, UINT32_MAX, 1);
        }
        m_churnSweWired = true;
    }

    auto barrierTo = [&](D3D12_RESOURCE_STATES to) {
        if (m_churnState == to) return;
        ctx.cmd->Barrier(m_churn.Res(), m_churnState, to);
        m_churnState = to;
    };

    // freezeChurn (harness, held stills only -- see SeaLayer.h): the advection/deposit pass is
    // suspended while a still is held at one instant, so the atlas is exactly what the last
    // unheld frame left and the hold's LENGTH stops changing the foam. The clear list is still
    // dispatched: a tile mapped during the hold would otherwise be read as undefined pool
    // memory. Outside a hold freezeChurn is false and this is the shipped path, dispatch for
    // dispatch.
    const bool doUpdate = !freezeChurn && !m_churn.ResidentList().empty();
    if (doUpdate || !m_pendingClear.empty()) {
        barrierTo(D3D12_RESOURCE_STATE_UNORDERED_ACCESS);

        m_churnCb.originX = m_churnOrgX;
        m_churnCb.originZ = m_churnOrgZ;
        m_churnCb.window[0] = std::floor(m_churnOrgX / m_churnSpan[0]);
        m_churnCb.window[1] = std::floor(m_churnOrgZ / m_churnSpan[1]);
        m_churnCb.window[2] = static_cast<float>(m_churn.TilesY());
        m_churnCb.window[3] = 0.0f;
        m_churnCb.texelM = kChurnTexelM;
        m_churnCb.domainM = kChurnDomainM;
        m_churnCb.tilesX = m_churn.TilesX();
        m_churnCb.tileW = m_churn.TileW();
        m_churnCb.tileH = m_churn.TileH();
        m_churnCb.dt = m_churnDt;
        m_churnCb.tau = static_cast<float>(kChurnTau);
        // M5c: the chop-band WAVENUMBER -- the kernel derives phase speed from the local depth,
        // (misc[0] was the deep-water speed when churn had no bathy).
        m_churnCb.miscC[0] = m_chopK;
        m_churnCb.miscC[1] = m_fft.PatchL(2);
        m_churnCb.miscC[2] = static_cast<float>(std::fmod(m_tSec, 1024.0f));
        m_churnCb.miscC[3] = 0;
        // PHASE C5: the world.flat chart's rows (the exact map); the height window's rows below.
        m_surface->TangentRows(m_churnCb.tanE, m_churnCb.tanU, m_churnCb.tanN);
        memcpy(m_churnCb.eyeT, m_churnEyeT, sizeof(m_churnEyeT));   // PHASE B2
        memcpy(m_churnCb.hwU, &m_churnHw, sizeof(m_churnHw));
        m_churnCb.sweM[0] = m_churnSweWired ? 1.0f : 0.0f;
        m_churnCb.sweM[1] = sweCurrentGain;
        m_churnCb.sweM[2] = 0.0f;   // PHASE C3b: no jet, no hand-over ramp
        m_churnCb.sweM[3] = 0.0f;
        m_churnCb.waveD[0] = m_peakDirX;
        m_churnCb.waveD[1] = m_peakDirZ;
        m_churnCb.waveD[2] = 1.0f;
        m_churnCb.waveD[3] = 1.0f;
        {   // M12 step 4b instrument: the churn kernel's constant buffer, fingerprinted after
            // its fill -- the gate for the lattice-row moves (winA, geoA from the surface) and
            // for the fills of 4e/4f. Logs when the hash changes, as the [surface] fills do.
            // listCount is dispatchList's, written per list below, not this fill's: the copy
            // hashed here carries it as zero so the fingerprint is the fill and nothing else.
            ChurnCbData fp = m_churnCb;
            fp.listCount = 0;
            const uint64_t h = Fnv1aBytes(&fp, sizeof(fp));
            if (h != m_churnCbFp) {
                m_churnCbFp = h;
                Log("[kernel] churn cb FNV-1a %016llx", static_cast<unsigned long long>(h));
            }
        }

        ctx.cmd->ComputeRoot(m_churnRs.Get());

        auto dispatchList = [&](hal::PsoPtr pso, const std::vector<uint32_t>& list) {
            if (list.empty()) return;
            m_churnCb.listCount = static_cast<uint32_t>(list.size());
            ctx.cmd->ComputeConstants(0, m_churnCb);
            ctx.cmd->ComputeSrvAt(1, ctx.gpu->PushConstants(list.data(), list.size() * 4));
            ctx.cmd->ComputeTable(2, m_churnTable.Base());
            ctx.cmd->Pipeline(pso);
            ctx.cmd->Dispatch(m_churn.TileW() / 16, m_churn.TileH() / 16, m_churnCb.listCount);
        };

        if (!m_pendingClear.empty()) {
            PixMarker(ctx.cmd->Native(), "churn.clearFresh (undefined pool memory -> zero)");
            dispatchList(m_churnClear.Get(), m_pendingClear);
            m_pendingClear.clear();
            ctx.cmd->UavBarrier(m_churn.Res());
        }
        if (doUpdate) dispatchList(m_churnUpdate.Get(), m_churn.ResidentList());
    }
    barrierTo(D3D12_RESOURCE_STATE_PIXEL_SHADER_RESOURCE);
}

void SeaLayer::Simulate(const FrameContext& ctx) {
    if (!m_haveData) return;
    if (!m_swe || !m_swe->Ready()) return;
    // THE SOLVER STEPS FOR ITS CONSUMERS, NOT FOR THE CAMERA (the water match, step 3). It stepped
    // inside Render, and Render runs only while the sea is `enabled` -- which the frame loop clears
    // whenever the camera is above 60 km. A hull floating in the solver's domain then read a frozen
    // surface (its region went unserved, its asOf aged), and a fly-in arrived to a solver that had
    // to catch up, re-anchor or reset by its clock policy. Now it steps when the sea is drawn, or
    // when anyone asked it for a region this frame (SweSolver::Demanded) -- interest-driven, sparse,
    // and blind to where the eye is. Once a frame, however many views draw the sea.
    if (!enabled && !m_swe->Demanded()) return;
    GpuScope gscope(ctx.prof, ctx.cmd->Native(), "sea.swe");
    m_swe->Record(*ctx.cmd, *ctx.gpu, m_simUnix, m_seaLevel);
}

void SeaLayer::Render(const FrameContext& ctx) {
    if (!m_haveData) return;

    // The compute chain records into the same command list; compute bindings do not disturb the
    // graphics root signature the Renderer already set. (The solver stepped in Simulate, before any
    // view: the churn below advects on this frame's current as it always did.)
    {
        GpuScope gscope(ctx.prof, ctx.cmd->Native(), "sea.fft");
        m_fft.Record(*ctx.cmd, *ctx.gpu, m_tSec);
    }
    {
        GpuScope gscope(ctx.prof, ctx.cmd->Native(), "sea.churn");
        RecordChurn(ctx);
    }

}

}  // namespace ga
