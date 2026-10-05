// TwinSurface - --twin-surface.
// Moved verbatim from main.cpp (M12 step 1b); declared in app/Tools.h.
#include "app/Tools.h"

#include "core/Common.h"
#include "hal/Gpu.h"
#include "core/SceneConfig.h"
#include "render/Camera.h"
#include "scene/SeaLayer.h"
#include "scene/WaterBankLayer.h"
#include "sim/BathyModel.h"
#include "sim/Medium.h"
#include "sim/SeaState.h"
#include "sim/WaterSurfaceTree.h"
#include "sim/WaveField.h"
#include "sim/WeatherManager.h"

#include <algorithm>
#include <cmath>
#include <memory>
#include <vector>

namespace ga::app::tools {

// ==================================================================
//  M9bq --twin-surface: THE KEYSTONE GATE. Does the CPU water a hull
//  reads agree with the GPU water the player sees?
//
//  Everything in the vessel work rests on TreeWater being the same sea as
//  CsBankFill. Nothing else can check that: the rails are headless stills,
//  the selftest has no GPU bank, and a boat that floats wrong looks exactly
//  like a boat whose water is wrong. So this reads the bank ONCE and asks
//  the CPU the same question at the same points.
//
//  THE SITES ARE RANGES FROM THE CAMERA, NOT PLACES. The bank is a ladder of
//  camera-anchored rings ~20 km across at the coarsest, so a point in the
//  open Atlantic is not "offshore", it is OUTSIDE EVERY RING and the gate
//  would be comparing against nothing. Range is also the axis that actually
//  matters here: it sweeps ring texel size and it sweeps wWin, which is the
//  handover between the solved field and the cascades and therefore the
//  single most likely explanation for a disagreement.
//
//  Blue water is checked separately below, WITHOUT a GPU comparison, because
//  out there the question is not "do the two agree" but "does the CPU know
//  where the bottom is at all".
// ==================================================================
// THE THREE LEVELS THAT MUST AGREE at the camera (the startup transient, measured 2026-09-10):
// the classifier's -- the scalar the surface classifier holds the bed against, the tide plane --
// the bank's -- the level plane the GPU draws, which inside a solver's domain carries the
// solver's deviation -- and the CPU's -- what a hull reads, WeatherManager::Query, the solver
// being truth. A basin the solver holds below the tide splits the first from the other two.
void LogLevelsAtCamera(Gpu& gpu, WaterBankLayer* waterBank, const Camera& cam, double simUnix,
                       WeatherManager& weather, double classifierNavd) {
    weather.RefreshMirrorsTo(gpu, simUnix);
    const double xz[2] = {cam.px, cam.pz};
    WaterBankLayer::BankPoint bp{};
    if (waterBank) waterBank->ReadBankPoints(gpu, xz, 1, &bp);
    const double lat = BathyModel::kOrgLat + cam.pz / BathyModel::kMPerLat;
    const double lon = BathyModel::kOrgLon + cam.px / BathyModel::kMPerLon;
    const WeatherSample q = weather.Query(lat, lon, simUnix, 1.0);
    const double bank = bp.valid ? double(bp.level) : classifierNavd;
    const double spread = (std::max)({classifierNavd, bank, q.levelNavd}) -
                          (std::min)({classifierNavd, bank, q.levelNavd});
    Log("[twin] water level at the camera (%.0f, %.0f): classifier %+.3f | bank %+.3f%s | CPU "
        "%+.3f (%s) m NAVD88 -- spread %.3f m",
        cam.px, cam.pz, classifierNavd, bank, bp.valid ? "" : " (NO BANK POINT)", q.levelNavd,
        q.levelSrc, spread);
}

void RunTwinSurface(const Options&, Gpu& gpu, const SeaState& seaState, SeaLayer* sea,
                    const WaterSceneConfig& waterScene, WaterBankLayer* waterBank,
                    const Camera& cam, double simUnix, WeatherManager& weather,
                    const std::unique_ptr<WaveField>& waveField, double classifierNavd) {
    LogLevelsAtCamera(gpu, waterBank, cam, simUnix, weather, classifierNavd);
    weather.RefreshMirrorsTo(gpu, simUnix);
    TreeWater tw;
    tw.Configure(&weather, waveField.get(), &sea->Ocean(), &seaState,
                 sea->heightScale, waterScene.wfExag, waterScene.wfChop);
    tw.SetCascadeSea(sea->PeakDirX(), sea->PeakDirZ(), sea->PeakDirValid(), sea->Scale());
    Log("[twin] %s", tw.Describe(cam.px, cam.pz, simUnix).c_str());

    constexpr int kRings = 4, kPer = 64;
    const double kRangeM[kRings] = {0.0, 500.0, 2000.0, 8000.0};
    std::vector<double> pts;
    pts.reserve(kRings * kPer * 2);
    for (int r = 0; r < kRings; ++r) {
        for (int i = 0; i < kPer; ++i) {
            // A deterministic spiral, so two runs sample the same water and a
            // regression is a change in the ANSWER, not in the question.
            const double th = i * 2.39996322972865332;   // golden angle
            const double rr = kRangeM[r] + (r == 0 ? 0.0 : 0.0);
            const double jit = (r == 0) ? (i % 8) * 1.5 : 0.0;
            pts.push_back(cam.px + (rr + jit) * std::cos(th));
            pts.push_back(cam.pz + (rr + jit) * std::sin(th));
        }
    }
    std::vector<WaterBankLayer::BankPoint> bp(pts.size() / 2);
    waterBank->ReadBankPoints(gpu, pts.data(), int(bp.size()), bp.data());

    Log("[twin] ---- CPU TreeWater vs GPU bank, %d points, exag %.3f ----",
        int(bp.size()), double(sea->heightScale));
    int totalCmp = 0;
    double worstAll = 0.0;
    for (int r = 0; r < kRings; ++r) {
        int nCmp = 0, nNoBank = 0, nNoCpu = 0;
        double maxD = 0.0, sumSq = 0.0, wSum = 0.0;
        // THE ERROR MUST BE DECOMPOSED OR IT CANNOT BE DIAGNOSED. The surface
        // is mean level + wave displacement, and those come from completely
        // different machinery: the level from the tide atlas and the solver
        // mirror, the displacement from the solved field and the cascades. A
        // single total error number cannot tell a tide offset from a wave phase
        // error, and they have opposite fixes.
        double maxL = 0.0, sumSqL = 0.0, maxW = 0.0, sumSqW = 0.0;
        // The two fields' own rms, and their correlation. Together these say
        // WHICH kind of wrong: equal rms with zero correlation is a phase
        // error, unequal rms is an amplitude or gain error, and they have
        // nothing to do with each other.
        double sqC = 0.0, sqG = 0.0, cross = 0.0;
        int ringUsed = -1;
        for (int i = 0; i < kPer; ++i) {
            const int idx = r * kPer + i;
            const double wx = pts[idx * 2], wz = pts[idx * 2 + 1];
            if (!bp[idx].valid) { ++nNoBank; continue; }
            const SurfaceSample cs = tw.AtLabel(wx, wz, simUnix);   // the bank texel is a label
            if (!cs.valid) { ++nNoCpu; continue; }
            // The bank writes the mean level and the wave displacement into
            // different planes; the surface the renderer draws is their sum.
            const double gpuH = double(bp[idx].level) + double(bp[idx].dispY);
            const double d = cs.heightNavd - gpuH;
            maxD = (std::max)(maxD, std::abs(d));
            sumSq += d * d;
            // The same split on the CPU side: the mean surface straight from the
            // one point evaluator, and the waves as the remainder.
            double qlat = 0.0, qlon = 0.0;
            qlon = BathyModel::kOrgLon + wx / BathyModel::kMPerLon;
            qlat = BathyModel::kOrgLat + wz / BathyModel::kMPerLat;
            const double cpuLevel =
                weather.Query(qlat, qlon, simUnix, 1.0).levelNavd;
            const double dL = cpuLevel - double(bp[idx].level);
            const double dW = (cs.heightNavd - cpuLevel) - double(bp[idx].dispY);
            maxL = (std::max)(maxL, std::abs(dL));
            sumSqL += dL * dL;
            maxW = (std::max)(maxW, std::abs(dW));
            sumSqW += dW * dW;
            const double cw = cs.heightNavd - cpuLevel, gw = double(bp[idx].dispY);
            sqC += cw * cw;
            sqG += gw * gw;
            cross += cw * gw;
            wSum += tw.WindowWeight(wx, wz);
            ringUsed = bp[idx].ring;
            ++nCmp;
        }
        totalCmp += nCmp;
        worstAll = (std::max)(worstAll, maxD);
        if (nCmp > 0) {
            Log("[twin]   %5.0f m: n=%2d  TOTAL max %+.4f rms %.4f | "
                "LEVEL max %+.4f rms %.4f | WAVES max %+.4f rms %.4f | "
                "wWin %.3f ring %d (no bank %d)",
                kRangeM[r], nCmp, maxD, std::sqrt(sumSq / nCmp),
                maxL, std::sqrt(sumSqL / nCmp),
                maxW, std::sqrt(sumSqW / nCmp),
                wSum / nCmp, ringUsed, nNoBank);
            const double rc = std::sqrt(sqC / nCmp), rg = std::sqrt(sqG / nCmp);
            Log("[twin]            wave rms: CPU %.4f  GPU %.4f  ratio %.3f  "
                "correlation %+.3f",
                rc, rg, (rg > 1e-9) ? rc / rg : 0.0,
                (rc > 1e-9 && rg > 1e-9) ? (cross / nCmp) / (rc * rg) : 0.0);
        } else {
            Log("[twin]   %5.0f m: NO COMPARISON (no bank %d, no cpu %d)",
                kRangeM[r], nNoBank, nNoCpu);
        }
    }
    Log("[twin] compared %d points, worst |dh| %.4f m", totalCmp, worstAll);

    // ---- BLUE WATER. No GPU comparison is possible out here (no resident
    // ring), and that is the point: the question offshore is whether the CPU
    // knows the bottom at all. A bed of exactly 0.0 with no provenance is the
    // failure this checks for -- it would read as aground in mid-ocean.
    const double kOff[3][2] = {{40000.0, 0.0}, {120000.0, 40000.0},
                               {400000.0, 100000.0}};
    for (int i = 0; i < 3; ++i) {
        const double wx = kOff[i][0], wz = kOff[i][1];
        const SurfaceSample bs = tw.At(wx, wz, simUnix);
        Log("[twin] blue %6.0f km E: %s  level %+.3f bed %+.1f depth %+.1f m",
            wx / 1000.0, bs.valid ? "COVERED" : "NO COVERAGE",
            bs.heightNavd, bs.bedNavd, bs.depthM);
        if (bs.valid && std::abs(bs.bedNavd) < 1e-6) {
            Log("[twin]   WARNING bed is exactly 0.0 with coverage claimed -- "
                "that is the signature of an absent height source reading as "
                "datum, not of a real seabed at NAVD zero");
        }
    }
}

}  // namespace ga::app::tools
