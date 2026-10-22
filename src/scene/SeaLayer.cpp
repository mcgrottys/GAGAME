#include "scene/SeaLayer.h"
#include "core/Planet.h"
#include "compose/HeightTree.h"
#include "core/Lattice.h"
#include "core/ThreadManager.h"

#include "hal/GpuProfiler.h"

#include "hal/PixEvents.h"
#include "hal/Pipeline.h"
#include "hal/Root.h"
#include "hal/Views.h"
#include "scene/FieldSet.h"

#include <algorithm>
#include <chrono>
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
    // A file's point carries no fetch here: its wind sea is the law's open-sea limit (B4 is the grid's).
    int activeParts = PartsOfHour(src.Hour(src.HourIndex(simUnix)), storm, parts, log, -1.0, -1.0);
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

int SeaLayer::PartsOfHour(const SeaHour& hr, bool storm, PartParam parts[4], bool log, double fetchM,
                          double depthM) const {
    int activeParts = 0;
    if (storm) {
            // Sandbox storm: the named swell carries 85% of the energy, a fresher wind sea 25%
            // of the height on a slightly shorter period, 30 degrees off.
            parts[0] = SeaState::MakePartition(m_stormHs * 0.92, m_stormTp, m_stormDir, false);
            parts[1] = SeaState::MakePartition(m_stormHs * 0.40, m_stormTp * 0.55,
                                               m_stormDir + 30.0, true);
            activeParts = 2;
    } else {
            activeParts = SeaState::BuildParamsOf(hr, parts);
            // M9a: THE MISSING WIND SEA. On light-wind hours GFS-Wave's partitioning hands
            // back swell trains ONLY -- and a swell partition is a Gaussian of
            // sigF = clamp(0.10/Tp^2, 0.004, 0.02) Hz: ~4 mHz wide, with no tail at all. So
            // every last joule lands in cascade 0 and cascades 1-2 realize NUMERICALLY ZERO
            // (measured at 12z f004: band Hs 0.357 / 1e-12 / 0.000 m, mss 3.0e-5 / 1.6e-25
            // / 0). The water then renders as poured glass -- no short faces to carry
            // normals, and the only sub-cascade roughness left is bandSig[3], which widens
            // the glint lobe and never moves a normal (priors 8: normals sell amplitude).
            // But the wind that raises those ripples is already sitting in the same file.
            // So synthesise its sea and let it ride the partition list like any other train --
            // the shapes, the spreading, the plot, the buoy assimilation and the GPU all pick it
            // up unchanged. Capped at the forecast's own combined Hs.
            // B4 (REVIEW 2026-10-21, Mark: "I see waves going south not into the inlet"): that sea
            // was Pierson-Moskowitz, the fetch-UNLIMITED answer. At the Merrimack, 10-22 16z, an
            // 8.5 m/s northerly raised 2.04 m at 6.7 s running south beside the grid's 2.10 m swell
            // from the east -- the model's Hs 2.10 drawn as 2.97, the larger share a sea the coast
            // to the north does not allow. Now it is grown over its fetch, in its depth (Young &
            // Verhagen 1996, SeaState::WindSeaGrown), whose long-fetch deep-water limit is PM.
            bool hasWindSea = false;
            for (int i = 0; i < activeParts; ++i) hasWindSea |= parts[i].gamma > 0.0f;
            double wsHs = 0.0, wsTp = 0.0;
            const double fetch = fetchM >= 0.0 ? fetchM : 1e12;   // no place to march from: the open sea
            if (!hasWindSea && activeParts < 4 && windSeaFill > 0.0f &&
                SeaState::WindSeaGrown(hr.windMs, fetch, depthM, &wsHs, &wsTp)) {
                wsHs *= windSeaFill;
                if (hr.combinedHs > 0.02) wsHs = std::min(wsHs, hr.combinedHs);
                if (wsHs > 0.02) {
                    parts[activeParts++] =
                        SeaState::MakePartition(wsHs, wsTp, hr.windFromDeg, true);
                    if (log) Log("[sea] f%03d has no wind-sea partition (wind %.1f m/s from %.0f deg): grown over "
                        "its fetch %s in %s of water (Young & Verhagen 1996): Hs %.2f m Tp %.1f s (lambda %.1f m)",
                        static_cast<int>(hr.fh), hr.windMs, hr.windFromDeg,
                        fetchM >= 0.0 ? (std::to_string(int(std::lround(fetchM))) + " m").c_str() : "unbounded",
                        depthM > 0.0 ? (std::to_string(int(std::lround(depthM))) + " m").c_str() : "deep",
                        wsHs, wsTp, 1.56 * wsTp * wsTp);
                }
            }
        }
    return activeParts;
}

namespace {
// A BUOY REACHES ONLY SO FAR -- the tide stations' law (compose/WaterAtlas.cpp StationWeight): 1 / d^2,
// tapered to nought by smoothstep from 90 km to 270 km; and the share it may claim at all, by the
// nearest one, the same taper. One open-ocean buoy speaks for a basin's sea, not a planet's.
double BuoyTaper(double dM) {
    const double t = std::clamp((dM - 90000.0) / 180000.0, 0.0, 1.0);
    return 1.0 - t * t * (3.0 - 2.0 * t);
}
double GreatCircleM(double la0, double lo0, double la1, double lo1) {
    constexpr double kD2R = 3.14159265358979323846 / 180.0;
    const double a = std::sin(0.5 * (la1 - la0) * kD2R), b = std::sin(0.5 * (lo1 - lo0) * kD2R);
    const double h = a * a + std::cos(la0 * kD2R) * std::cos(la1 * kD2R) * b * b;
    return 2.0 * 6371000.0 * std::asin(std::sqrt(std::clamp(h, 0.0, 1.0)));
}
}  // namespace

double SeaLayer::BuoyGainAt(double latDeg, double lonDeg, double simUnix, int hour, int* n) const {
    // THE BUOYS BY THEIR OWN RATIO (2026-10-21). They were the file's mean -- every buoy of a composite
    // that holds Florida's and Virginia's, against one place's forecast. Each buoy now says how far the
    // forecast WHERE IT STANDS is off (its measured Hs over the grid's there), and that ratio reaches
    // the place by the tide stations' law, in log (a ratio of 2 and one of 1/2 cancel).
    int k = 0;
    double sw = 0.0, slr = 0.0, dmin = 1e30;
    if (m_grid && m_sea) {
        for (const BuoyObs& b : m_sea->Buoys()) {
            if (!b.placed || !b.valid || !(b.hs > 0.05) || !(std::abs(simUnix - b.obsUnix) < buoyAssimAgeH * 3600.0)) continue;
            SeaHour at;
            if (!m_grid->HourAt(b.lat, b.lon, hour, at) || !(at.combinedHs > 0.05)) continue;
            const double d = GreatCircleM(latDeg, lonDeg, b.lat, b.lon);
            const double w = BuoyTaper(d) / (d * d + 1.0);
            if (w <= 0.0) continue;
            const double r = std::clamp(b.hs / at.combinedHs, 1.0 / buoyAssimGainMax, double(buoyAssimGainMax));
            sw += w;
            slr += w * std::log(r);
            dmin = (std::min)(dmin, d);
            ++k;
        }
    }
    if (n) *n = k;
    return sw > 0.0 ? std::exp(BuoyTaper(dmin) * slr / sw) : 1.0;
}

int SeaLayer::PartsAt(double latDeg, double lonDeg, double simUnix, PartParam parts[4], bool log,
                      SeaHour* hrOut, bool* fetchPending) const {
    if (fetchPending) *fetchPending = false;
    if (!m_grid) return -1;
    const int hour = m_grid->HourIndex(simUnix);
    SeaHour hr;
    double la = latDeg, lo = lonDeg;
    if (!m_grid->HourAt(la, lo, hour, hr) && !(m_grid->NearestSea(la, lo) && m_grid->HourAt(la, lo, hour, hr))) return -1;
    if (hrOut) *hrOut = hr;
    double depth = -1.0, fetch = -1.0;
    bool hasWindSea = false;
    for (const SeaPartition& p : hr.parts) hasWindSea |= p.windsea;
    if (!hasWindSea && hr.windMs > 0.9) fetch = FetchAt(la, lo, hr.windFromDeg, &depth);
    if (fetch == -2.0) {   // its walk is out: no wind sea raised before its fetch is known
        if (fetchPending) *fetchPending = true;
        fetch = 0.0;
    }
    int n = PartsOfHour(hr, StormOn(), parts, log, fetch, depth);
    if (!StormOn() && n > 0) {
        int nb = 0;
        const double g = BuoyGainAt(la, lo, simUnix, hour, &nb);
        for (int i = 0; i < n; ++i) parts[i].specScale *= static_cast<float>(g * g);   // energy ~ Hs^2
        if (log && nb > 0) Log("[sea] the grid's sea at %.3f, %.3f: %d buoy(s) within reach -> gain %.2f", la, lo, nb, g);
    }
    return n;
}

namespace {
// B4: THE WALK, on a loader thread. HeightTree::Fetch is the swell shadow's exact walk (every texel the
// great circle crosses; --tool fetch-probe holds it against a brute march) at one grain, so the ray goes
// in three legs, each ten times the last at ten times the grain: 20 km at 10 m (a barrier beach, a jetty),
// on to 200 km at 100 m, on to 2,000 km at 1 km, where the growth law is at its open-sea limit for any
// wind the grid carries. One great circle throughout: each leg starts where the last ended, in the
// direction the circle has there. GROUND UNDER THE START IS NOT A SHORE: an eye over a marsh or a beach
// reads the water upwind of it, from the first water the ray meets within 20 km. The depth is the water's
// mean along the fetch, the tree read at 24 points on it.
SeaLayer::FetchMemo WalkFetch(HeightTree& tree, double latDeg, double lonDeg, double windFromDeg, float level) {
    constexpr double kD2R = 3.14159265358979323846 / 180.0;
    const auto t0 = std::chrono::steady_clock::now();
    const double r = Lattice::kMercCirc / (2.0 * 3.14159265358979323846);   // the walk's own sphere
    const double la = latDeg * kD2R, lo = lonDeg * kD2R, th = windFromDeg * kD2R;
    const double cl = std::cos(la), sl = std::sin(la), co = std::cos(lo), so = std::sin(lo);
    double p[3] = {cl * co, sl, cl * so}, t[3];
    {
        const double e0[3] = {-so, 0.0, co}, n0[3] = {-sl * co, cl, -sl * so};
        for (int k = 0; k < 3; ++k) t[k] = std::sin(th) * e0[k] + std::cos(th) * n0[k];
    }
    const auto along = [&](double a, double q[3], double u[3]) {   // the circle's point and direction at arc a
        for (int k = 0; k < 3; ++k) {
            q[k] = p[k] * std::cos(a) + t[k] * std::sin(a);
            if (u) u[k] = -p[k] * std::sin(a) + t[k] * std::cos(a);
        }
    };
    const auto latLon = [](const double q[3], double& qla, double& qlo) {
        qla = std::asin((std::clamp)(q[1], -1.0, 1.0));
        qlo = std::atan2(q[2], q[0]);
    };
    SeaLayer::FetchMemo m;
    double skipped = 0.0;
    if (tree.Sample(la, lo, 10.0) > level) {
        bool wet = false;
        for (double d = 10.0; d < 2.0e4 && !wet; d *= 1.25) {
            double q[3], u[3], qla, qlo;
            along(d / r, q, u);
            latLon(q, qla, qlo);
            if (tree.Sample(qla, qlo, (std::max)(10.0, 0.25 * d)) <= level) {
                for (int k = 0; k < 3; ++k) { p[k] = q[k]; t[k] = u[k]; }
                skipped = d;
                wet = true;
            }
        }
        if (!wet) {
            Log("[sea] fetch from %.4f, %.4f toward the wind's %.0f deg: land for 20 km -- no fetch", latDeg, lonDeg,
                windFromDeg);
            return m;
        }
    }
    const double ps[3] = {p[0], p[1], p[2]}, ts[3] = {t[0], t[1], t[2]};   // where the water begins
    const double legs[3][2] = {{2.0e4, 10.0}, {1.8e5, 100.0}, {1.8e6, 1000.0}};
    double fetch = 0.0;
    for (const auto& leg : legs) {
        double pla, plo;
        latLon(p, pla, plo);
        const double pc = std::cos(pla), pn = std::sin(pla), oc = std::cos(plo), os = std::sin(plo);
        const double e[3] = {-os, 0.0, oc}, n[3] = {-pn * oc, pc, -pn * os};
        const double tE = t[0] * e[0] + t[1] * e[1] + t[2] * e[2], tN = t[0] * n[0] + t[1] * n[1] + t[2] * n[2];
        const double f = tree.Fetch(pla, plo, tE, tN, leg[0], leg[1], level);
        fetch += f;
        if (f < leg[0]) break;   // the shore
        double q[3], u[3];
        along(leg[0] / r, q, u);
        for (int k = 0; k < 3; ++k) { p[k] = q[k]; t[k] = u[k]; }
    }
    m.fetchM = fetch;
    if (fetch > 0.0) {
        double sumD = 0.0;
        int wet = 0;
        for (int i = 0; i < 24; ++i) {
            const double a = (i + 0.5) / 24.0 * fetch / r;
            double q[3], qla, qlo;
            for (int k = 0; k < 3; ++k) q[k] = ps[k] * std::cos(a) + ts[k] * std::sin(a);
            latLon(q, qla, qlo);
            const float h = tree.Sample(qla, qlo, (std::max)(10.0, fetch / 24.0));
            if (h < level) {
                sumD += double(level) - double(h);
                ++wet;
            }
        }
        m.depthM = wet > 0 ? sumD / wet : -1.0;
    }
    Log("[sea] fetch from %.4f, %.4f toward the wind's %.0f deg: %.2f km of water%s, mean depth %.1f m (%.0f ms)",
        latDeg, lonDeg, windFromDeg, fetch / 1000.0,
        skipped > 0.0 ? (" past " + std::to_string(int(std::lround(skipped))) + " m of ground").c_str() : "",
        m.depthM, std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count());
    return m;
}
}  // namespace

double SeaLayer::FetchAt(double latDeg, double lonDeg, double windFromDeg, double* depthM) const {
    if (depthM) *depthM = -1.0;
    HeightTree* tree = (m_comp && m_hgtCh >= 0) ? m_comp->HeightTreeOf(m_hgtCh) : nullptr;
    if (!tree) return -1.0;   // no height to walk over: the open sea's limit
    // The memo: the place to 0.005 deg (~500 m), the wind to 5 deg, the water to 0.25 m.
    const int64_t qa = std::llround(latDeg / 0.005), qo = std::llround(lonDeg / 0.005);
    const int64_t qd = ((std::llround(windFromDeg / 5.0) % 72) + 72) % 72, ql = std::llround(m_seaLevel / 0.25);
    const uint64_t key = (uint64_t(qa & 0xFFFFF) << 44) | (uint64_t(qo & 0x1FFFFF) << 23) | (uint64_t(qd) << 16) |
                         uint64_t(ql & 0xFFFF);
    {
        std::lock_guard<std::mutex> lk(m_fetch->mx);
        if (auto it = m_fetch->memo.find(key); it != m_fetch->memo.end()) {
            if (depthM) *depthM = it->second.depthM;
            return it->second.fetchM;
        }
        if (!m_fetch->asked.insert(key).second) return -2.0;   // out already
        if (m_fetch->memo.size() > 4096) m_fetch->memo.clear();   // a long flight's places: start again
    }
    const float level = m_seaLevel;
    const double la = double(qa) * 0.005, lo = double(qo) * 0.005, from = double(qd) * 5.0;   // the bucket's own
    Threads().Submit(Lane::Io, "sea.fetch", [shared = m_fetch, tree, la, lo, from, level, key] {
        const FetchMemo m = WalkFetch(*tree, la, lo, from, level);
        {
            std::lock_guard<std::mutex> lk(shared->mx);
            shared->memo[key] = m;
            shared->asked.erase(key);
        }
        shared->landed.fetch_add(1, std::memory_order_acq_rel);
    });
    return -2.0;
}

int64_t SeaLayer::EyeKey(double latDeg, double lonDeg, int hour) const {
    SeaHour hr;
    double la = latDeg, lo = lonDeg;
    if (!m_grid || (!m_grid->HourAt(la, lo, hour, hr) && !(m_grid->NearestSea(la, lo) && m_grid->HourAt(la, lo, hour, hr)))) {
        return int64_t(hour);
    }
    // B4: where the forecast has no wind sea, the eye's grown one moves with its fetch -- 25 % steps in log.
    int64_t fb = 0;
    bool hasWindSea = false;
    for (const SeaPartition& p : hr.parts) hasWindSea |= p.windsea;
    if (!hasWindSea && hr.windMs > 0.9) {
        const double f = FetchAt(la, lo, hr.windFromDeg, nullptr);
        fb = f == -2.0 ? 0xFF : f > 0.0 ? 1 + int64_t(std::floor(std::log((std::max)(f, 20.0) / 20.0) / std::log(1.25))) : 0;
    }
    // The peak: the most energetic partition (M7j's choice), the combined sea where there is none.
    double hs = -1.0, tp = hr.combinedTp, from = hr.combinedFromDeg;
    for (const SeaPartition& p : hr.parts) {
        if (p.hs > hs) {
            hs = p.hs;
            tp = p.tp;
            from = p.fromDeg;
        }
    }
    const int64_t tb = int64_t(std::floor(std::log((std::max)(tp, 0.5)) / std::log(1.05)));
    const int64_t db = int64_t(std::floor(from / 5.0)) % 72;
    return int64_t(hour) | (tb & 0xFFFF) << 16 | (db & 0xFF) << 32 | (fb & 0xFF) << 40;
}

void SeaLayer::SetTime(double simUnix, double seaLevelM, double camX, double camZ) {
    if (!m_sea || !m_sea->Ready()) return;
    m_seaLevel = static_cast<float>(seaLevelM);   // first: the fetch's walk (B4) ends at ground above it

    // THE SYNTHESIS'S SEA (2026-10-21): with the globe's grid, the sea at the eye's own ground point,
    // made again when the forecast hour moves or the eye's sea leaves its bucket (EyeKey: the peak's
    // period and direction -- the scale field carries its Hs); without, data.seastate's point by the hour.
    int hour = 0;
    bool newHour = false, respectrum = false;
    double eLat = 0.0, eLon = 0.0;
    if (m_grid) {
        hour = m_grid->HourIndex(simUnix);
        if (m_surface) m_surface->flat.LatLonOf(camX, camZ, eLat, eLon);
        const int64_t key = EyeKey(eLat, eLon, hour);
        newHour = hour != m_lastHour;
        // B4: a fetch's walk landed while the field's sea waited on one -- read the field's sea again.
        const uint32_t landed = m_fetch->landed.load(std::memory_order_acquire);
        const bool fieldAgain = m_fieldPending && landed != m_fetchLanded;
        m_fetchLanded = landed;
        respectrum = newHour || key != m_eyeKey || fieldAgain;
        m_eyeKey = key;
    } else {
        hour = m_sea->HourIndex(simUnix);
        newHour = respectrum = hour != m_lastHour;
    }
    if (respectrum) {
        m_lastHour = hour;
        if (!newHour) ++m_eyeSpectra;
        // The hour's change says everything; the eye's, the first and then one in 32 (they come as fast
        // as the eye crosses buckets).
        const bool logIt = newHour || m_eyeSpectra == 1u || (m_eyeSpectra & 31u) == 0u;
        PartParam parts[4];
        SeaHour eyeHr;
        const int nGrid = m_grid ? PartsAt(eLat, eLon, simUnix, parts, logIt, &eyeHr) : -1;
        m_fromGrid = nGrid >= 0;
        activeParts = m_fromGrid ? nGrid : PartsOf(*m_sea, simUnix, StormOn(), parts, logIt);
        if (m_grid && newHour) m_gridHourNew = hour;
        if (m_grid && m_fieldPlaced && (newHour || m_fieldPending)) {
            m_fieldActive = (std::max)(0, PartsAt(m_fieldLat, m_fieldLon, simUnix, m_fieldParts, logIt, nullptr,
                                                  &m_fieldPending));
        }
        const double cycle = m_fromGrid ? m_grid->CycleUnix() : m_sea->CycleUnix();
        const uint32_t seed = static_cast<uint32_t>(cycle / 3600.0) * 2654435761u;
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
        // With the grid the field is the grid's Hs (GlobeModel's waves, moved to this hour by the frame
        // loop) and the reference is the grid's own Hs at the eye, read as the field reads it: the
        // scale is 1 at the eye and the grid's relative sea everywhere else.
        const double gridRef = m_fromGrid ? m_grid->HsBilinear(eLat, eLon, hour) : -1.0;
        m_scale.hsRef = gridRef > 1e-3 ? gridRef : (hsModel > 1e-3 ? hsModel : 1.0);
        m_scale.storm = StormOn();   // a declared storm is the planet's sea (WaveScale.h)
        m_scale.sources.clear();
        for (const SeaState* src : m_sources) {
            if (!src || !src->Ready() || !src->HasBox()) continue;
            if (m_fromGrid && src == m_sea) continue;   // the grid IS data.seastate's measurement now
            PartParam sp[4];
            const int n = PartsOf(*src, simUnix, src == m_sea && StormOn(), sp, false);
            WaveScale::Source fs;
            src->Box(fs.box);
            fs.hs = SeaState::SignificantHeight(sp, n);
            m_scale.sources.push_back(fs);
            if (logIt) Log("[sea] field source %s: box %.3f..%.3f N x %.3f..%.3f E, Hs %.3f m (scale %.3f over "
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
            m_cPeak = static_cast<float>(PlanetGWave() / (2.0 * 3.14159265 * parts[pk].fp));
            m_peakDirX = parts[pk].dirToX;
            m_peakDirZ = parts[pk].dirToZ;
            m_peakDirValid = true;   // M7j: the shadow may only march a REAL direction
        }

        char note[32];
        if (m_stormHs > 0.01f) {
            snprintf(note, sizeof(note), "STORM %.1fm/%.0fs", m_stormHs, m_stormTp);
        } else {
            snprintf(note, sizeof(note), "f%03d", static_cast<int>(m_fromGrid ? m_grid->HourFh(hour) : m_sea->Hour(hour).fh));
        }
        statusNote = note;

        // Whitecap COVERAGE follows wind speed steeply (Monahan's ~U^3.4 law); the Jacobian only
        // decides WHERE foam sits. Without this gate, the 2-second chop that a 4 m/s breeze
        // raises micro-breaks all over the render at full brightness -- geometrically true,
        // visually absurd (real coverage at 4 m/s is ~0.01%).
        double windMs = m_fromGrid ? eyeHr.windMs : m_sea->Hour(hour).windMs;
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
                PlanetGWave() * (m_stormTp * 0.55) / (2.0 * 3.14159265) * 0.85 * 2.0);
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
                const double k = (2.0 * kPiD * f) * (2.0 * kPiD * f) / PlanetGWave();
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
            if (logIt) Log("[sea] fold log-width per band: %.2f / %.2f / %.2f (energy-weighted sigma "
                "of ln k -- the fraction of each band Nyquist still admits)",
                m_bandKSpread[0], m_bandKSpread[1], m_bandKSpread[2]);
            if (logIt) Log("[sea] fold wavelength per band: %.1f / %.1f / %.2f m (cut means "
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
            if (logIt) Log("[sea] slope variance: bands %.4f/%.4f/%.4f + floor %.4f (Cox-Munk %.4f at "
                "%.1f m/s -- the far field is the same pixel the globe draws)",
                bandSig[0], bandSig[1], bandSig[2], bandSig[3],
                coxMunk, windMs);
        }

        if (logIt && m_fromGrid) {
            Log("[sea] %s %s: the sea at the eye (%.3f, %.3f): %d partitions, model Hs %.2f m, the grid's %.2f m "
                "(the scale's reference); %u re-made for the eye's sea so far",
                m_grid->CycleLabel().c_str(), statusNote.c_str(), eLat, eLon, activeParts, hsModel, m_scale.hsRef,
                m_eyeSpectra);
        } else if (logIt) {
            Log("[sea] %s %s: %d partitions, model Hs %.2f m (its buoys %.2f m)",
                m_sea->CycleLabel().c_str(), statusNote.c_str(), activeParts, hsModel, hsBuoy);
        }
    }

    m_tSec = static_cast<float>(simUnix - (m_fromGrid ? m_grid->CycleUnix() : m_sea->CycleUnix()));

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
            m_churnTable.SrvArray(2, m_hgtArr, m_hgtArr->GetDesc().Format, 0, UINT32_MAX, m_hgtMips);
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
        m_churnCb.miscC[3] = static_cast<float>(PlanetGWave());   // the planet's gravity (SeaChurn: Jet.hlsli)
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
            const uint64_t h = CbTrace() ? Fnv1aBytes(&fp, sizeof(fp)) : m_churnCbFp;   // F9: --cb-trace
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
    if (onSimulate) onSimulate(ctx);   // the owned windows with consumers (WeatherManager::Simulate)
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
