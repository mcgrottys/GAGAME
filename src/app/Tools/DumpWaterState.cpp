// DumpWaterState - --dump-water-state.
// Moved verbatim from main.cpp (M12 step 1b); declared in app/Tools.h.
#include "app/Tools.h"

#include "core/Common.h"
#include "hal/Gpu.h"
#include "scene/SeaLayer.h"
#include "sim/WaterManager.h"

#include <cstdio>
#include <vector>

namespace ga::app::tools {

// M7p: export the inlet box's REAL fields (bed, level, current, shadow)
// so proofs/inlet_storm.py -- the user's own vqview wave model -- can run
// the independent 2D storm figure on the exact data this engine uses.
void RunDumpWaterState(const Options&, Gpu& gpu, SeaLayer* sea, double simUnix,
                       WaterManager& weather, const Space::Anchor& chart) {
    // The export reads the solver mirrors through Query: bring them to this
    // instant first (the one readback of the run; the loop never did one).
    weather.RefreshMirrorsTo(gpu, simUnix);
    const double bx0 = -1200.0, bz0 = -1600.0, cellW = 10.0;
    const int nxW = 420, nyW = 300;
    std::vector<float> bedW(nxW * nyW), lvlW2(nxW * nyW), uW(nxW * nyW),
        vW(nxW * nyW), shW(nxW * nyW);
    for (int j = 0; j < nyW; ++j) {          // row 0 = SOUTH (+v = north)
        for (int i2 = 0; i2 < nxW; ++i2) {
            const double wxD = bx0 + (i2 + 0.5) * cellW;
            const double wzD = bz0 + (j + 0.5) * cellW;
            double latD = 0.0, lonD = 0.0;   // PHASE C4: the box about the scene's anchor
            chart.LatLonOf(wxD, wzD, latD, lonD);
            const WeatherSample q =
                weather.Query(latD, lonD, simUnix, cellW);
            const size_t at = static_cast<size_t>(j) * nxW + i2;
            bedW[at] = q.bedNavd;
            lvlW2[at] = static_cast<float>(q.levelNavd);
            uW[at] = q.u;
            vW[at] = q.v;
            shW[at] = sea->ShadowAtWorld(static_cast<float>(wxD),
                                         static_cast<float>(wzD));
        }
    }
    auto wr = [&](const char* pth, std::vector<float>& g) {
        if (FILE* f2 = fopen(pth, "wb")) {
            fwrite(g.data(), sizeof(float), g.size(), f2);
            fclose(f2);
        }
    };
    wr("ws_bed.f32", bedW);
    wr("ws_level.f32", lvlW2);
    wr("ws_u.f32", uW);
    wr("ws_v.f32", vW);
    wr("ws_shadow.f32", shW);
    {   // the mirror itself and each dump cell's lattice cell (stage 2's instrument; the solver's chart in doubles)
        uint32_t mw = 0, mh = 0;
        const std::vector<float>* uv4 = nullptr;
        const SweSolver* solver = nullptr;
        if (const std::vector<float>* eta = weather.MirrorEta(mw, mh, &uv4, &solver); eta && solver && uv4) {
            std::vector<float> e(*eta), hcol(size_t(mw) * mh);
            for (size_t i = 0; i < hcol.size(); ++i) hcol[i] = (*uv4)[i * 4 + 2];
            wr("ws_mirror_eta.f32", e);
            wr("ws_mirror_h.f32", hcol);
            std::vector<float> txy(size_t(nxW) * nyW * 2);
            for (int j = 0; j < nyW; ++j) {
                for (int i2 = 0; i2 < nxW; ++i2) {
                    double la = 0.0, lo = 0.0, tx = 0.0, ty = 0.0;
                    chart.LatLonOf(bx0 + (i2 + 0.5) * cellW, bz0 + (j + 0.5) * cellW, la, lo);
                    solver->Domain().CellOf(la, lo, tx, ty);
                    txy[(size_t(j) * nxW + i2) * 2] = float(tx);
                    txy[(size_t(j) * nxW + i2) * 2 + 1] = float(ty);
                }
            }
            wr("ws_cells.f32", txy);
            Log("[waterstate] the mirror exported: %u x %u eta and h, and each dump cell's lattice cell", mw, mh);
        }
    }
    double qcLat = 0.0, qcLon = 0.0;   // the box's centre
    chart.LatLonOf(bx0 + 0.5 * nxW * cellW, bz0 + 0.5 * nyW * cellW, qcLat, qcLon);
    const WeatherSample qc = weather.Query(qcLat, qcLon, simUnix, 500.0);
    if (FILE* fj2 = fopen("ws_meta.json", "wb")) {
        fprintf(fj2,
                "{ \"x0\": %.1f, \"z0\": %.1f, \"cell\": %.1f, "
                "\"nx\": %d, \"ny\": %d, \"rows\": \"south-to-north\", "
                "\"hs\": %.2f, \"tp\": %.2f, \"dirFrom\": %.1f, "
                "\"peakDirX\": %.3f, \"peakDirZ\": %.3f }",
                bx0, bz0, cellW, nxW, nyW,
                sea->hsModel > 0.01 ? sea->hsModel : qc.hs, qc.tp,
                qc.dirDeg, sea->PeakDirX(), sea->PeakDirZ());
        fclose(fj2);
    }
    Log("[waterstate] ws_*.f32 + ws_meta.json exported (%dx%d at %.0f m)",
        nxW, nyW, cellW);
}

}  // namespace ga::app::tools
