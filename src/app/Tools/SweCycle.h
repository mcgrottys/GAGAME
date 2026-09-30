// ================================================================================================
//  SweCycle - --swe-cycle N: the headless tidal-cycle validation run, moved verbatim from
//  main.cpp (M12 step 1b). See app/Tools.h.
//
//  Header-only on purpose. The solver's boundary clocks (oceanAt / southAt / westAt / westQAt)
//  are lambdas that stay in main(), and SweSolver::Spinup / AdvanceTo take them as template
//  parameters; so RunSweCycle (the CSV integrator, unchanged) and RunSweCycleMode (the block
//  that ran under `if (opt.sweCycleH > 0)`) are templates too, and the lambdas bind exactly as
//  they did -- no std::function in between.
// ================================================================================================
#pragma once

#include "app/Options.h"
#include "core/Common.h"
#include "hal/Gpu.h"
#include "hal/Residency.h"
#include "sim/BathyModel.h"
#include "sim/CurrentModel.h"
#include "sim/SweSolver.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <functional>
#include <queue>
#include <string>
#include <utility>
#include <vector>

namespace ga::app::tools {

// THE CHANNEL'S PATH (the investigation of the reach's lag): the least-cost walk through the
// kernel's bed from (c0, r0) to (c1, r1), a step costing its length over the square of the depth
// below +1 m NAVD (dry above it; column 0, the pinned exterior, is never entered), so the walk
// keeps to the deepest water. Cells in walk order, s = metres along the walk.
struct ThalwegCell {
    int col, row;
    float s;
};
// The lowest crest between two cells: the walk (4 neighbours, column 0 never entered) whose
// highest bed is least -- the level water must stand above for the two to be one channel.
// Returns that bed (3e38 when none) and the crest's cell.
inline float LowestCrest(const std::vector<float>& bed, int nx, int ny, int start, int goal,
                         int& crestCell) {
    const size_t n = size_t(nx) * ny;
    std::vector<float> best(n, 3.0e38f);
    std::vector<int> prev(n, -1);
    using Item = std::pair<float, int>;
    std::priority_queue<Item, std::vector<Item>, std::greater<Item>> open;
    best[size_t(start)] = bed[size_t(start)];
    open.push({best[size_t(start)], start});
    while (!open.empty()) {
        const Item top = open.top();
        open.pop();
        if (top.first > best[size_t(top.second)]) continue;
        if (top.second == goal) break;
        const int c = top.second % nx, r = top.second / nx;
        for (int dr = -1; dr <= 1; ++dr) {
            for (int dc = -1; dc <= 1; ++dc) {
                const int cc = c + dc, rr = r + dr;
                // 4 neighbours, like the kernel's faces: water does not pass a corner
                if ((dc == 0) == (dr == 0) || cc < 1 || rr < 0 || cc >= nx || rr >= ny) continue;
                const int j = rr * nx + cc;
                if (bed[size_t(j)] < -9000.0f) continue;
                const float m = (std::max)(top.first, bed[size_t(j)]);
                if (m < best[size_t(j)]) {
                    best[size_t(j)] = m;
                    prev[size_t(j)] = top.second;
                    open.push({m, j});
                }
            }
        }
    }
    crestCell = -1;
    if (best[size_t(goal)] > 1.0e38f) return best[size_t(goal)];
    float hi = -1.0e9f;
    for (int i = goal; i >= 0; i = prev[size_t(i)]) {
        if (bed[size_t(i)] > hi) {
            hi = bed[size_t(i)];
            crestCell = i;
        }
    }
    return best[size_t(goal)];
}

inline std::vector<ThalwegCell> FindThalweg(const std::vector<float>& bed, int nx, int ny, float dx,
                                            float dy, int c0, int r0, int c1, int r1,
                                            float kLevel) {
    const size_t n = size_t(nx) * ny;
    std::vector<float> dist(n, 3.0e38f);
    std::vector<int> prev(n, -1);
    using Item = std::pair<float, int>;
    std::priority_queue<Item, std::vector<Item>, std::greater<Item>> open;
    const int start = r0 * nx + c0, goal = r1 * nx + c1;
    dist[size_t(start)] = 0.0f;
    open.push({0.0f, start});
    while (!open.empty()) {
        const Item top = open.top();
        open.pop();
        if (top.first > dist[size_t(top.second)]) continue;
        if (top.second == goal) break;
        const int c = top.second % nx, r = top.second / nx;
        for (int dr = -1; dr <= 1; ++dr) {
            for (int dc = -1; dc <= 1; ++dc) {
                const int cc = c + dc, rr = r + dr;
                if ((dc == 0 && dr == 0) || cc < 1 || rr < 0 || cc >= nx || rr >= ny) continue;
                const int j = rr * nx + cc;
                const float depth = kLevel - bed[size_t(j)];
                if (!(depth > 0.05f) || bed[size_t(j)] < -9000.0f) continue;
                const float len = std::sqrt(float(dc * dc) * dx * dx + float(dr * dr) * dy * dy);
                const float nd = top.first + len / (depth * depth);
                if (nd < dist[size_t(j)]) {
                    dist[size_t(j)] = nd;
                    prev[size_t(j)] = top.second;
                    open.push({nd, j});
                }
            }
        }
    }
    std::vector<ThalwegCell> path;
    if (goal != start && prev[size_t(goal)] < 0) return path;
    for (int i = goal; i >= 0; i = prev[size_t(i)]) path.push_back({i % nx, i / nx, 0.0f});
    std::reverse(path.begin(), path.end());
    for (size_t k = 1; k < path.size(); ++k) {
        const float ex = float(path[k].col - path[k - 1].col) * dx;
        const float ey = float(path[k].row - path[k - 1].row) * dy;
        path[k].s = path[k - 1].s + std::sqrt(ex * ex + ey * ey);
    }
    return path;
}

// THE WEST BOUNDARY AT ONE INSTANT (the investigation of the Flather face): what goes in -- the
// tide, the exterior's target deviation and the transport the boundary is told to carry, with the
// stations they are made of (`inputs`, the session's) -- the face's own terms for every pinned row
// (SweSolver::TraceWestFace), and the profile along the deepest pinned row from the boundary into
// the channel: the kernel's bed (the bed trace), eta less the tide, and u. An instrument: it reads
// back and waits.
inline void LogWestBoundary(Gpu& gpu, SweSolver& swe, double hours, double tide, double westDEta,
                            double westQ, const std::string& inputs, std::vector<float>& bedGpu) {
    Log("[west] +%.0f h: tide %+.3f m NAVD, target dEta %+.3f (the exterior at %+.3f), told %+.1f "
        "m^3/s | %s",
        hours, tide, westDEta, tide + westDEta, westQ, inputs.c_str());
    std::vector<SweSolver::WestFaceRow> rows;
    if (!swe.TraceWestFace(gpu, rows)) {
        Log("[west] the face trace did not run");
        return;
    }
    Log("[west]   row    etaX    etaI    sill     hf   u_ext     rad      ub     qRaw        q  "
        "clamp[lo, hi]      bit  dEta0    hI   bed0   bed1");
    double qSum = 0.0;
    int pinned = 0, deepest = -1;
    float deepBed = 1e9f;
    for (size_t y = 0; y < rows.size(); ++y) {
        const SweSolver::WestFaceRow& r = rows[y];
        if (r.pinned < 0.5f) continue;
        ++pinned;
        qSum += r.q;
        if (r.bed0 < deepBed) {
            deepBed = r.bed0;
            deepest = static_cast<int>(y);
        }
        Log("[west]   %4zu %+7.3f %+7.3f %+7.3f %6.3f %+7.3f %+7.3f %+7.3f %+8.2f %+8.2f "
            "[%+7.1f, %+6.1f] %s %+6.3f %5.2f %+6.2f %+6.2f",
            y, r.etaX, r.etaI, r.sill, r.hf, r.uext, r.rad, r.ub, r.qRaw, r.q, r.qLo, r.qHi,
            (r.q != r.qRaw) ? "BIT" : " - ", r.dEta0, r.hI, r.bed0, r.bed1);
    }
    Log("[west]   %d pinned rows; q summed %+.1f m^3/s against the told %+.1f; u_ext was divided by "
        "a live section of %.1f m^2",
        pinned, qSum, westQ, swe.WestArea());
    if (deepest < 0) return;
    std::vector<float> eta, uv4;
    uint32_t ew = 0, eh = 0, uw = 0, uh = 0;
    swe.ReadFields(gpu, eta, ew, eh, uv4, uw, uh);
    if (bedGpu.empty()) {
        SweSolver::BedTrace bt;
        if (swe.TraceBed(gpu, bt)) bedGpu = bt.bed;
    }
    Log("[west]   profile along row %d (the deepest pinned row): col, bed (the kernel's), eta less "
        "the tide, u east, valid",
        deepest);
    for (uint32_t x = 0; x <= 60 && x < ew; ++x) {
        const size_t i = size_t(deepest) * ew + x;
        Log("[west]   col %2u  bed %+7.3f  dEta %+7.3f  u %+6.3f  v%d", x,
            i < bedGpu.size() ? bedGpu[i] : 0.0f, i < eta.size() ? eta[i] : 0.0f,
            i * 4 < uv4.size() ? uv4[i * 4] : 0.0f, (i * 4 + 3 < uv4.size() && uv4[i * 4 + 3] > 0.5f) ? 1 : 0);
    }
}

// M5c validation harness: integrate a full tidal cycle headless and log probes to CSV --
// ocean sponge (must track the analytic tide), the entrance throat AT the ACT0816 station
// (solved current vs the CO-OPS prediction, the real gate), the Joppa Flats basin (lag +
// attenuation must EMERGE), and the river's standing slope upstream.
template <typename F, typename G, typename H, typename Q>
void RunSweCycle(Gpu& gpu, SweSolver& swe, F oceanAt, G westAt, H southAt, Q westQAt,
                 const CurrentModel* currents, int ctSta, const BathyModel& bathy,
                 double startUnix, double hours,
                 const std::function<std::string(double)>& describeWest = {}) {
    std::vector<float> bedGpu;   // the kernel's bed, read once for the west profiles
    auto tideAt = oceanAt;

    // River probe: the deepest channel cell near x = -5000 m.
    const int nx = bathy.Nx(), ny = bathy.Ny();
    const int ix = static_cast<int>((-5000.0f - bathy.WorldX0()) / bathy.WorldSizeX() * nx);
    int iyBest = ny / 2;
    float eBest = 1e9f;
    for (int iy = 0; iy < ny; ++iy) {
        const float e = bathy.Elev()[iy * nx + ix];
        if (e > -9000.0f && e < eBest) {
            eBest = e;
            iyBest = iy;
        }
    }
    const float riverZ = bathy.WorldZ0() + bathy.WorldSizeZ() * (1.0f - (iyBest + 0.5f) / ny);
    Log("[swe-cycle] river probe (-5000, %.0f), bed %.1f m", riverZ, eBest);

    // Throat probe: the deepest channel cell within 150 m of the ACT0816 station -- the jet
    // core, not whatever shoal the exact origin lands on.
    float throatX = 0, throatZ = 0, tBest = 1e9f;
    for (float wz = -150; wz <= 150; wz += 10) {
        for (float wx = -150; wx <= 150; wx += 10) {
            const float e = bathy.SampleWorld(wx, wz);
            if (e > -9000.0f && e < tBest) {
                tBest = e;
                throatX = wx;
                throatZ = wz;
            }
        }
    }
    Log("[swe-cycle] throat probe (%.0f, %.0f), bed %.1f m", throatX, throatZ, tBest);

    // West probe: the deepest cell of the column kWestIn cells in from the west boundary -- the
    // water the river brings has to pass it to reach the basin (HIERARCHY 4.17: the window
    // drawn in to where the survey paints at full weight is what lets it).
    constexpr int kWestIn = 30;
    const int ixW = (std::min)(kWestIn, nx - 1);
    int iyW = ny / 2;
    float eW = 1e9f;
    for (int iy = 0; iy < ny; ++iy) {
        const float e = bathy.Elev()[iy * nx + ixW];
        if (e > -9000.0f && e < eW) {
            eW = e;
            iyW = iy;
        }
    }
    const float westX = bathy.WorldX0() + bathy.WorldSizeX() * (ixW + 0.5f) / nx;
    const float westZ = bathy.WorldZ0() + bathy.WorldSizeZ() * (1.0f - (iyW + 0.5f) / ny);
    Log("[swe-cycle] west probe (%.0f, %.0f), bed %.1f m, %d cells in from the west boundary",
        westX, westZ, eW, kWestIn);

    // THE CHANNEL'S PATH: the kernel's bed once (the west log reuses it), the walk from the west
    // boundary's first interior column (its deepest cell among the pinned rows) to the jetty gap
    // (the deepest cell of the x = 350 transect), and every cell of it sampled every row with the
    // probes: out/swe_thalweg_<pid>.txt (the grid and the cells), .bed (the kernel's bed, f32),
    // .bin (a row: hours, tide, then dEta, u, v, valid a cell, f32).
    {
        SweSolver::BedTrace bt;
        if (swe.TraceBed(gpu, bt)) bedGpu = bt.bed;
    }
    std::vector<ThalwegCell> thalweg;
    FILE* tf = nullptr;
    if (bedGpu.size() == size_t(nx) * ny && nx > 2) {
        int r0 = -1, r1 = -1;
        float b0 = 1e9f, b1 = 1e9f;
        for (int iy = 0; iy < ny; ++iy) {
            const float e = bedGpu[size_t(iy) * nx + 1];
            if (bedGpu[size_t(iy) * nx] < 2.0f && e < b0) {
                b0 = e;
                r0 = iy;
            }
        }
        // No pinned row (the west face dry, as in the survey's faded band): the walk starts at the
        // deepest cell of the first column from the west whose bed goes below -2 m.
        int c0 = 1;
        for (int ix = 1; r0 < 0 && ix < nx / 3; ++ix) {
            for (int iy = 0; iy < ny; ++iy) {
                const float e = bedGpu[size_t(iy) * nx + ix];
                if (e < -2.0f && e < b0) {
                    b0 = e;
                    r0 = iy;
                    c0 = ix;
                }
            }
        }
        const int c1 = static_cast<int>((350.0f - bathy.WorldX0()) / bathy.WorldSizeX() * nx);
        for (float wz = -350.0f; wz <= 150.0f; wz += 5.0f) {
            const int iy = static_cast<int>((bathy.WorldZ0() + bathy.WorldSizeZ() - wz) /
                                            bathy.WorldSizeZ() * ny);
            if (iy < 0 || iy >= ny || c1 < 1 || c1 >= nx) continue;
            if (bedGpu[size_t(iy) * nx + c1] < b1) {
                b1 = bedGpu[size_t(iy) * nx + c1];
                r1 = iy;
            }
        }
        float level = 1.0f;
        if (r0 >= 0 && r1 >= 0) {
            // The walk stands its water 0.5 m above the lowest crest between the two ends when
            // that crest is higher than +0.5 m NAVD (a channel the tide cannot cross is a finding).
            int crestCell = -1;
            const float crest = LowestCrest(bedGpu, nx, ny, r0 * nx + c0, r1 * nx + c1, crestCell);
            if (crestCell >= 0) {
                const int cc = crestCell % nx, cr = crestCell / nx;
                Log("[thalweg] the lowest crest between the edge and the gap: bed %+.2f m NAVD at "
                    "(%d, %d), world (%.0f, %.0f)",
                    crest, cc, cr, bathy.WorldX0() + bathy.WorldSizeX() * (cc + 0.5f) / nx,
                    bathy.WorldZ0() + bathy.WorldSizeZ() * (1.0f - (cr + 0.5f) / ny));
                level = (std::max)(1.0f, crest + 0.5f);
            }
            thalweg = FindThalweg(bedGpu, nx, ny, bathy.WorldSizeX() / nx, bathy.WorldSizeZ() / ny,
                                  c0, r0, c1, r1, level);
        }
        Log("[thalweg] %zu cells from (%d, %d) bed %.2f to the gap (%d, %d) bed %.2f, %.0f m along "
            "(the walk's water at %+.2f m)",
            thalweg.size(), c0, r0, b0, c1, r1, b1, thalweg.empty() ? 0.0f : thalweg.back().s,
            level);
        const unsigned long pid = static_cast<unsigned long>(GetCurrentProcessId());
        char tp[96];
        snprintf(tp, sizeof(tp), "out/swe_thalweg_%lu.bed", pid);
        FILE* bf = nullptr;
        fopen_s(&bf, tp, "wb");
        if (bf) {
            fwrite(bedGpu.data(), sizeof(float), bedGpu.size(), bf);
            fclose(bf);
        }
        snprintf(tp, sizeof(tp), "out/swe_thalweg_%lu.txt", pid);
        FILE* hf = nullptr;
        fopen_s(&hf, tp, "w");
        if (hf) {
            fprintf(hf, "nx %d ny %d x0 %.3f z0 %.3f sizeX %.3f sizeZ %.3f cells %zu\n", nx, ny,
                    bathy.WorldX0(), bathy.WorldZ0(), bathy.WorldSizeX(), bathy.WorldSizeZ(),
                    thalweg.size());
            for (const ThalwegCell& c : thalweg) {
                fprintf(hf, "%d %d %.2f %.3f\n", c.col, c.row, c.s, bedGpu[size_t(c.row) * nx + c.col]);
            }
            fclose(hf);
        }
        if (!thalweg.empty()) {
            snprintf(tp, sizeof(tp), "out/swe_thalweg_%lu.bin", pid);
            fopen_s(&tf, tp, "wb");
        }
    }

    // Under out/, never data/ (the shared caches), and one file a process, so a batch of cycles
    // keeps every run's rows.
    char path[96];
    snprintf(path, sizeof(path), "out/swe_cycle_%lu.csv",
             static_cast<unsigned long>(GetCurrentProcessId()));
    FILE* f = nullptr;
    fopen_s(&f, path, "w");
    if (!f) {
        Log("[swe-cycle] cannot write %s", path);
        return;
    }
    fprintf(f,
            "unix,tide_navd,act_pred_ms,ocean_deta,throat_deta,throat_u,throat_v,basin_deta,"
            "river_deta,west_target,gap_u,river_u,river_valid,west_deta,west_u,west_valid,west_q,"
            "west_q_target\n");
    const double endUnix = startUnix + hours * 3600.0;
    int rows = 0;
    for (double t = startUnix; t <= endUnix; t += 120.0) {
        swe.AdvanceTo(gpu, t, tideAt, westAt, southAt, westQAt);
        // 4 named probes + a 6-point transect across the jetty gap (x = 350) whose valid-point
        // mean is the fair section current to hold against ACT0816.
        // The ocean probe at 2400: inside the full sponge of the drawn-in grid too.
        const float fixedPts[22] = {2400.0f, 0.0f,   throatX, throatZ, -2500.0f, -900.0f,
                                    -5000.0f, riverZ, 350.0f, -350.0f, 350.0f,  -250.0f,
                                    350.0f,  -150.0f, 350.0f, -50.0f,  350.0f,  50.0f,
                                    350.0f,  150.0f, westX,  westZ};
        // The 11 probes, then the channel's path (one readback serves both).
        std::vector<float> pts(fixedPts, fixedPts + 22);
        for (const ThalwegCell& c : thalweg) {
            pts.push_back(bathy.WorldX0() + bathy.WorldSizeX() * (c.col + 0.5f) / nx);
            pts.push_back(bathy.WorldZ0() + bathy.WorldSizeZ() * (1.0f - (c.row + 0.5f) / ny));
        }
        std::vector<SweSolver::Probe> pr(11 + thalweg.size());
        swe.ReadProbes(gpu, pts.data(), static_cast<int>(pr.size()), pr.data());
        if (tf) {
            const float head[2] = {static_cast<float>((t - startUnix) / 3600.0),
                                   static_cast<float>(tideAt(t))};
            fwrite(head, sizeof(float), 2, tf);
            for (size_t k = 0; k < thalweg.size(); ++k) {
                const SweSolver::Probe& p = pr[11 + k];
                const float rec[4] = {p.dEta, p.u, p.v, p.valid ? 1.0f : 0.0f};
                fwrite(rec, sizeof(float), 4, tf);
            }
        }
        const SweSolver::Probe& west = pr[10];
        const double westQ = swe.WestTransport(gpu);
        const SweSolver::Probe &ocean = pr[0], &throat = pr[1], &basin = pr[2], &river = pr[3];
        float gapU = 0;
        int gapN = 0;
        for (int gi = 4; gi < 10; ++gi) {
            if (pr[gi].valid) {
                gapU += pr[gi].u;
                ++gapN;
            }
        }
        gapU = gapN ? gapU / gapN : 0.0f;
        const double act =
            (currents && ctSta >= 0) ? currents->SignedSpeed(static_cast<size_t>(ctSta), t) : 0.0;
        fprintf(f, "%.0f,%.4f,%.3f,%.4f,%.4f,%.3f,%.3f,%.4f,%.4f,%.4f,%.3f,%.3f,%d,%.4f,%.3f,%d,"
                   "%.2f,%.2f\n",
                t, tideAt(t), act, ocean.dEta, throat.dEta, throat.u, throat.v, basin.dEta,
                river.dEta, westAt(t), gapU, river.u, river.valid ? 1 : 0, west.dEta, west.u,
                west.valid ? 1 : 0, westQ, westQAt(t));
        // The west boundary at hours 0, 3, 6 and 9 (a row every 120 s).
        if (rows % 90 == 0 && rows <= 270) {
            LogWestBoundary(gpu, swe, (t - startUnix) / 3600.0, tideAt(t), westAt(t), westQAt(t),
                            describeWest ? describeWest(t) : std::string(), bedGpu);
        }
        if (++rows % 30 == 0) {
            Log("[swe-cycle] +%.1f h  tide %+.2f  gap u %+.2f (ACT %+.2f)  basin dEta %+.3f",
                (t - startUnix) / 3600.0, tideAt(t), gapU, act, basin.dEta);
        }
    }
    fclose(f);
    if (tf) fclose(tf);
    Log("[swe-cycle] wrote %s (%d rows)", path, rows);
}

// The mode: spin the solver up two hours of history, integrate opt.sweCycleH hours to
// out/swe_cycle_<pid>.csv, tear down, exit 0.
template <typename Ocean, typename South, typename West, typename WestQ>
int RunSweCycleMode(const Options& opt, Gpu& gpu, const CurrentModel& currents, bool haveCurrents,
                    const BathyModel& bathy, SweSolver& swe, ResidencyManager& resMgr,
                    double simUnix, Ocean oceanAt, South southAt, West westAt, WestQ westQAt,
                    const std::function<std::string(double)>& describeWest = {}) {
    // --swe-cycle-stage M (an instrument, the net-flow measurement): the told exterior's stage
    // raised by a constant M metres; its rate, and so the told transport, is unchanged.
    const double stageM = opt.sweCycleStageM;
    const auto westAtStage = [&westAt, stageM](double t) { return westAt(t) + stageM; };
    if (stageM != 0.0) {
        Log("[swe-cycle] the told west stage raised by %+.3f m (--swe-cycle-stage)", stageM);
    }
    swe.Spinup(gpu, simUnix, 2.0, oceanAt, westAtStage, southAt, westQAt);
    const int ctSta = haveCurrents ? currents.StationIndex("ACT0816") : -1;
    RunSweCycle(gpu, swe, oceanAt, westAtStage, southAt, westQAt,
                haveCurrents ? &currents : nullptr, ctSta, bathy, simUnix,
                opt.sweCycleH, describeWest);
    gpu.WaitIdle();
    resMgr.Shutdown();
    gpu.Shutdown();
    return 0;
}

}  // namespace ga::app::tools
