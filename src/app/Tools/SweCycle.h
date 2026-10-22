// ================================================================================================
//  SweCycle - --swe-cycle N: the headless tidal-cycle validation run, moved verbatim from
//  main.cpp (M12 step 1b). See app/Tools.h.
//
//  Header-only on purpose. The solver's boundary clocks (oceanAt / faceAt / faceQAt)
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
#include "sim/CurrentModel.h"
#include "sim/Stations.h"
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
// below +1 m NAVD (dry above it; the domain's edge cells, the boundaries' exteriors, are never entered), so the walk
// keeps to the deepest water. Cells in walk order, s = metres along the walk.
struct ThalwegCell {
    int col, row;
    float s;
};
// The lowest crest between two cells: the walk (4 neighbours, the edge cells never entered) whose
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
                if ((dc == 0) == (dr == 0) || cc < 1 || rr < 1 || cc >= nx - 1 || rr >= ny - 1) continue;
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
                if ((dc == 0 && dr == 0) || cc < 1 || rr < 1 || cc >= nx - 1 || rr >= ny - 1) continue;
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

// THE FIRST RIVER FACE AT ONE INSTANT (the investigation of the Flather face): what goes in -- the
// tide, the exterior's target deviation and the transport the boundary is told to carry, with the
// stations they are made of (`inputs`, the session's) -- the face's own terms for every pinned row
// (SweSolver::TraceFace), and the profile along the deepest pinned row from the boundary into
// the channel: the kernel's bed (the bed trace), eta less the tide, and u. An instrument: it reads
// back and waits.
inline void LogWestBoundary(Gpu& gpu, SweSolver& swe, double hours, double tide, double westDEta,
                            double westQ, const std::string& inputs, std::vector<float>& bedGpu) {
    Log("[west] +%.0f h: tide %+.3f m NAVD, target dEta %+.3f (the exterior at %+.3f), told %+.1f "
        "m^3/s | %s",
        hours, tide, westDEta, tide + westDEta, westQ, inputs.c_str());
    std::vector<SweSolver::FaceRow> rows;
    if (!swe.TraceFace(gpu, rows)) {
        Log("[west] the face trace did not run");
        return;
    }
    Log("[west]   row    etaX    etaI    sill     hf   u_ext     rad      ub     qRaw        q  "
        "clamp[lo, hi]      bit  dEta0    hI   bed0   bed1");
    double qSum = 0.0;
    int pinned = 0, deepest = -1;
    float deepBed = 1e9f;
    for (size_t y = 0; y < rows.size(); ++y) {
        const SweSolver::FaceRow& r = rows[y];
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
        pinned, qSum, westQ, swe.FaceArea(0));
    if (deepest < 0) return;
    std::vector<float> eta, uv4;
    uint32_t ew = 0, eh = 0, uw = 0, uh = 0;
    swe.ReadFields(gpu, eta, ew, eh, uv4, uw, uh);
    if (bedGpu.empty()) {
        SweSolver::BedTrace bt;
        if (swe.TraceBed(gpu, bt)) bedGpu = bt.bed;
    }
    // Along the face's normal from its deepest pinned cell, inward: (side W: +x, E: -x, N: +y, S: -y).
    const SweDomain& dm = swe.Domain();
    const int side = dm.faces.empty() ? 2 : dm.faces[0].side;
    const int nx = int(dm.nx), ny = int(dm.ny);
    Log("[west]   profile from cell %d along the %c face's normal (the deepest pinned): step, bed (the kernel's), "
        "eta less the tide, u east, valid", deepest, "NSWE"[side]);
    for (int k = 0; k <= 60; ++k) {
        const int x = side == 2 ? k : side == 3 ? nx - 1 - k : deepest;
        const int y = side == 0 ? k : side == 1 ? ny - 1 - k : deepest;
        if (x < 0 || y < 0 || x >= nx || y >= ny) break;
        const size_t i = size_t(y) * ew + size_t(x);   // the fields' rows are ew wide (padded)
        const size_t ib = size_t(y) * size_t(nx) + size_t(x);
        Log("[west]   step %2d  bed %+7.3f  dEta %+7.3f  u %+6.3f  v%d", k,
            ib < bedGpu.size() ? bedGpu[ib] : 0.0f, i < eta.size() ? eta[i] : 0.0f,
            i * 4 < uv4.size() ? uv4[i * 4] : 0.0f, (i * 4 + 3 < uv4.size() && uv4[i * 4 + 3] > 0.5f) ? 1 : 0);
    }
}

// M5c validation harness: integrate a full tidal cycle headless and log probes to CSV. EVERY PROBE IS
// THE DATA'S (2026-10-21, Mark: "make sure you wrap up making everything part of a global model"; until
// this date they were the Merrimack's metres about its anchor): the ocean in an open side's sponge (must
// track the analytic tide); the throat at the tidal current station the domain holds (the solved current
// against its prediction, the real gate) and a section across it, square to the station's flood; the
// place's anchor; the river where it enters (the first face) and a third of the way from it to the throat.
template <typename F, typename G, typename Q>
void RunSweCycle(Gpu& gpu, SweSolver& swe, F oceanAt, G faceAt, Q faceQAt,
                 const CurrentModel* currents, int ctSta, const SweToolGrid& grid,
                 double startUnix, double hours,
                 const std::function<std::string(double)>& describeWest = {},
                 const SweFocusProbe* focus = nullptr) {
    std::vector<float> bedGpu;   // the kernel's bed, read once for the west profiles
    auto tideAt = oceanAt;
    const SweDomain& dom = *grid.dom;
    const int nx = int(dom.nx), ny = int(dom.ny);
    const auto bedAt = [&](int x, int y) { return dom.elev[size_t(y) * nx + size_t(x)]; };
    const auto flatOf = [&](double tx, double ty, float& x, float& z) {
        double xd = 0.0, zd = 0.0;
        grid.FlatOfCell(tx, ty, xd, zd);
        x = float(xd);
        z = float(zd);
    };

    // THE STATION: the tidal current station the domain holds nearest the place (RunSweCycleMode), its
    // place in the tools' flat chart; none, the anchor.
    double sx = 0.0, sz = 0.0, floodDeg = NAN;
    if (currents && ctSta >= 0) {
        const TidalCurrentStation& cs = currents->S(size_t(ctSta));
        grid.flat.FlatOf(cs.lat, cs.lon, sx, sz);
        floodDeg = cs.floodDeg;
    }
    // Throat probe: the deepest channel cell within 150 m of the station -- the jet core, not whatever
    // shoal the station's exact point lands on.
    float throatX = float(sx), throatZ = float(sz), tBest = 1e9f;
    for (float wz = -150; wz <= 150; wz += 10) {
        for (float wx = -150; wx <= 150; wx += 10) {
            const float e = grid.BedAtFlat(sx + wx, sz + wz);
            if (e > -9000.0f && e < tBest) {
                tBest = e;
                throatX = float(sx + wx);
                throatZ = float(sz + wz);
            }
        }
    }
    Log("[swe-cycle] throat probe (%.0f, %.0f), bed %.1f m, by %s", throatX, throatZ, tBest,
        (currents && ctSta >= 0) ? currents->S(size_t(ctSta)).id.c_str() : "the anchor (no current station)");
    // The section: six points across the throat, square to the flood (east of north, compass).
    float gap[12];
    {
        const double fr = std::isfinite(floodDeg) ? floodDeg * 3.14159265358979323846 / 180.0 : 0.0;
        const double ax = std::cos(fr), az = -std::sin(fr);   // the flood's compass turned a quarter: across it
        for (int k = 0; k < 6; ++k) {
            const double o = -250.0 + 100.0 * k;
            gap[2 * k] = float(throatX + ax * o);
            gap[2 * k + 1] = float(throatZ + az * o);
        }
    }
    // Ocean probe: the middle of an open side, inside its sponge.
    float oceanX = 0.0f, oceanZ = 0.0f;
    {
        const uint32_t o = dom.openSides;
        const double tx = (o & 4u) ? 2.5 : (o & 8u) ? nx - 2.5 : nx * 0.5;
        const double ty = (o & 1u) ? 2.5 : (o & 2u) ? ny - 2.5 : ny * 0.5;
        flatOf(tx, ty, oceanX, oceanZ);
        Log("[swe-cycle] ocean probe (%.0f, %.0f) %s", oceanX, oceanZ, o ? "in an open side's sponge" : "-- no open side: the middle");
    }
    // The river where it enters (the first face): the deepest cell of the line kIn cells in from its
    // side, and of the line a third of the way from the face to the throat.
    float riverX = throatX, riverZ = throatZ, westX = throatX, westZ = throatZ;
    constexpr int kWestIn = 30;
    if (!dom.faces.empty()) {
        const SweDomain::FaceCells& F = dom.faces[0];
        double ttx = 0.0, tty = 0.0;
        grid.CellOfFlat(throatX, throatZ, ttx, tty);
        const int toThroat = F.side == 2 ? int(ttx) : F.side == 3 ? nx - 1 - int(ttx) : F.side == 0 ? int(tty) : ny - 1 - int(tty);
        const auto deepestAt = [&](int in, float& x, float& z) {   // the line `in` cells in from the side
            int best = -1;
            float e = 1e9f;
            const int len = F.side < 2 ? nx : ny;
            for (int a = 0; a < len; ++a) {
                const int cx = F.side == 2 ? in : F.side == 3 ? nx - 1 - in : a;
                const int cy = F.side == 0 ? in : F.side == 1 ? ny - 1 - in : a;
                if (cx < 0 || cy < 0 || cx >= nx || cy >= ny) return;
                const float b = bedAt(cx, cy);
                if (b > -9000.0f && b < e) {
                    e = b;
                    best = a;
                }
            }
            if (best < 0) return;
            const int cx = F.side == 2 ? in : F.side == 3 ? nx - 1 - in : best;
            const int cy = F.side == 0 ? in : F.side == 1 ? ny - 1 - in : best;
            flatOf(cx + 0.5, cy + 0.5, x, z);
        };
        deepestAt((std::max)(1, toThroat / 3), riverX, riverZ);
        deepestAt((std::min)(kWestIn, (std::max)(1, toThroat - 1)), westX, westZ);
        Log("[swe-cycle] river probe (%.0f, %.0f), a third of the way from the %s face to the throat; face probe "
            "(%.0f, %.0f), %d cells in", riverX, riverZ, F.name.c_str(), westX, westZ, kWestIn);
    } else {
        Log("[swe-cycle] no river face: the river and face probes stand at the throat");
    }

    // THE CHANNEL'S PATH: the kernel's bed once (the face log reuses it), the walk from the first
    // river face's deepest pinned cell, one in, to the throat's cell, and every cell of it sampled every row with the
    // probes: out/swe_thalweg_<pid>.txt (the grid and the cells), .bed (the kernel's bed, f32),
    // .bin (a row: hours, tide, then dEta, u, v, valid a cell, f32).
    {
        SweSolver::BedTrace bt;
        if (swe.TraceBed(gpu, bt)) bedGpu = bt.bed;
    }
    std::vector<ThalwegCell> thalweg;
    FILE* tf = nullptr;
    if (bedGpu.size() == size_t(nx) * ny && nx > 2) {
        // From the first face's deepest pinned cell, one in, to the throat's cell.
        int c0 = -1, r0 = -1;
        float b0 = 1e9f, b1 = 1e9f;
        if (!dom.faces.empty()) {
            const SweDomain::FaceCells& F = dom.faces[0];
            for (int a = F.c0; a <= F.c1; ++a) {
                const int cx = F.side == 2 ? 1 : F.side == 3 ? nx - 2 : a;
                const int cy = F.side == 0 ? 1 : F.side == 1 ? ny - 2 : a;
                const float e = bedGpu[size_t(cy) * nx + cx];
                if (e < b0) {
                    b0 = e;
                    c0 = cx;
                    r0 = cy;
                }
            }
        }
        double gx = 0.0, gy = 0.0;
        grid.CellOfFlat(throatX, throatZ, gx, gy);
        const int c1 = std::clamp(int(gx), 1, nx - 2), r1 = std::clamp(int(gy), 1, ny - 2);
        b1 = bedGpu[size_t(r1) * nx + c1];
        float level = 1.0f;
        if (r0 >= 0 && c0 >= 0) {
            // The walk stands its water 0.5 m above the lowest crest between the two ends when
            // that crest is higher than +0.5 m NAVD (a channel the tide cannot cross is a finding).
            int crestCell = -1;
            const float crest = LowestCrest(bedGpu, nx, ny, r0 * nx + c0, r1 * nx + c1, crestCell);
            if (crestCell >= 0) {
                const int cc = crestCell % nx, cr = crestCell / nx;
                double crX = 0.0, crZ = 0.0;
                grid.FlatOfCell(cc + 0.5, cr + 0.5, crX, crZ);
                Log("[thalweg] the lowest crest between the face and the throat: bed %+.2f m NAVD at "
                    "(%d, %d), world (%.0f, %.0f)",
                    crest, cc, cr, crX, crZ);
                level = (std::max)(1.0f, crest + 0.5f);
            }
            thalweg = FindThalweg(bedGpu, nx, ny, float(dom.dx), float(dom.dy),
                                  c0, r0, c1, r1, level);
        }
        Log("[thalweg] %zu cells from (%d, %d) bed %.2f to the throat (%d, %d) bed %.2f, %.0f m along "
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
            fprintf(hf, "nx %d ny %d anchor %.7f %.7f dx %.4f dy %.4f cells %zu\n", nx, ny,
                    dom.latC, dom.lonC, dom.dx, dom.dy, thalweg.size());
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
            "unix,tide_navd,act_pred_ms,ocean_deta,throat_deta,throat_u,throat_v,anchor_deta,"
            "river_deta,west_target,gap_u,river_u,river_valid,west_deta,west_u,west_valid,west_q,"
            "west_q_target,focus_level,focus_pred\n");
    double ftx = -1.0, fty = -1.0;
    const bool haveFocus = focus && focus->pred && dom.CellOf(focus->lat, focus->lon, ftx, fty) &&
                           ftx >= 0.0 && fty >= 0.0 && ftx < nx && fty < ny;
    if (focus) {
        Log("[swe-cycle] focus probe: station %s at %.5f N %.5f E -> cell (%.2f, %.2f) of %dx%d%s, "
            "bed %+.2f m", focus->id.c_str(), focus->lat, focus->lon, ftx, fty, nx, ny,
            haveFocus ? "" : " -- OUTSIDE the domain",
            haveFocus ? double(dom.elev[size_t(fty) * nx + size_t(ftx)]) : 0.0);
    }
    double fSum = 0.0, fSq = 0.0, fMax = 0.0, fLo = 1e9, fHi = -1e9, pLo = 1e9, pHi = -1e9;
    double fLoT = 0.0, fHiT = 0.0, pLoT = 0.0, pHiT = 0.0;
    int fN = 0;
    const double endUnix = startUnix + hours * 3600.0;
    int rows = 0;
    for (double t = startUnix; t <= endUnix; t += 120.0) {
        swe.AdvanceTo(gpu, t, tideAt, faceAt, faceQAt);
        // 4 named probes + the 6-point section across the throat, whose valid-point mean is the fair
        // section current to hold against the station; then the face probe.
        float fixedPts[22] = {oceanX, oceanZ, throatX, throatZ, 0.0f, 0.0f, riverX, riverZ,
                              gap[0], gap[1], gap[2], gap[3], gap[4], gap[5], gap[6], gap[7], gap[8], gap[9],
                              gap[10], gap[11], westX, westZ};
        // The 11 probes, then the channel's path (one readback serves both).
        grid.CellsOfFlat(fixedPts, 11);   // the tools' points (world.flat) to the solver's cells
        std::vector<float> pts(fixedPts, fixedPts + 22);
        for (const ThalwegCell& c : thalweg) {
            pts.push_back(c.col + 0.5f);
            pts.push_back(c.row + 0.5f);
        }
        pts.push_back(float(ftx));   // the focus station, last
        pts.push_back(float(fty));
        std::vector<SweSolver::Probe> pr(12 + thalweg.size());
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
        const double westQ = swe.FaceTransport(gpu, 0);
        const SweSolver::Probe &ocean = pr[0], &throat = pr[1], &anchor = pr[2], &river = pr[3];
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
        const double fLevel = haveFocus ? tideAt(t) + double(pr.back().dEta) : 0.0;
        const double fPred = haveFocus ? focus->pred(t) : 0.0;
        if (haveFocus) {
            const double dd = fLevel - fPred;
            fSum += dd;
            fSq += dd * dd;
            fMax = (std::max)(fMax, std::abs(dd));
            ++fN;
            if (fLevel < fLo) { fLo = fLevel; fLoT = t; }
            if (fLevel > fHi) { fHi = fLevel; fHiT = t; }
            if (fPred < pLo) { pLo = fPred; pLoT = t; }
            if (fPred > pHi) { pHi = fPred; pHiT = t; }
        }
        fprintf(f, "%.0f,%.4f,%.3f,%.4f,%.4f,%.3f,%.3f,%.4f,%.4f,%.4f,%.3f,%.3f,%d,%.4f,%.3f,%d,"
                   "%.2f,%.2f,%.4f,%.4f\n",
                t, tideAt(t), act, ocean.dEta, throat.dEta, throat.u, throat.v, anchor.dEta,
                river.dEta, faceAt(0, t), gapU, river.u, river.valid ? 1 : 0, west.dEta, west.u,
                west.valid ? 1 : 0, westQ, faceQAt(0, t), fLevel, fPred);
        // The west boundary at hours 0, 3, 6 and 9 (a row every 120 s).
        if (rows % 90 == 0 && rows <= 270) {
            LogWestBoundary(gpu, swe, (t - startUnix) / 3600.0, tideAt(t), faceAt(0, t), faceQAt(0, t),
                            describeWest ? describeWest(t) : std::string(), bedGpu);
        }
        if (++rows % 30 == 0) {
            Log("[swe-cycle] +%.1f h  tide %+.2f  section u %+.2f (station %+.2f)  anchor dEta %+.3f",
                (t - startUnix) / 3600.0, tideAt(t), gapU, act, anchor.dEta);
        }
    }
    fclose(f);
    if (tf) fclose(tf);
    Log("[swe-cycle] wrote %s (%d rows)", path, rows);
    if (fN > 0) {
        Log("[swe-cycle] focus %s over %.2f h, %d rows: solver - prediction mean %+.4f m, rms %.4f m, "
            "max |d| %.4f m; solver low %+.3f at +%.2f h, high %+.3f at +%.2f h (range %.3f m); "
            "prediction low %+.3f at +%.2f h, high %+.3f at +%.2f h (range %.3f m)",
            focus->id.c_str(), hours, fN, fSum / fN, std::sqrt(fSq / fN), fMax, fLo,
            (fLoT - startUnix) / 3600.0, fHi, (fHiT - startUnix) / 3600.0, fHi - fLo, pLo,
            (pLoT - startUnix) / 3600.0, pHi, (pHiT - startUnix) / 3600.0, pHi - pLo);
    }
}

// The mode: spin the solver up two hours of history, integrate opt.sweCycleH hours to
// out/swe_cycle_<pid>.csv, tear down, exit 0.
template <typename Ocean, typename Face, typename FaceQ>
int RunSweCycleMode(const Options& opt, Gpu& gpu, const CurrentModel& currents, bool haveCurrents,
                    const SweToolGrid& grid, SweSolver& swe, ResidencyManager& resMgr,
                    double simUnix, Ocean oceanAt, Face faceAt, FaceQ faceQAt,
                    const std::function<std::string(double)>& describeWest = {},
                    const SweFocusProbe* focus = nullptr) {
    // --swe-cycle-stage M (an instrument, the net-flow measurement): the told exterior's stage
    // raised by a constant M metres; its rate, and so the told transport, is unchanged.
    const double stageM = opt.sweCycleStageM;
    const auto faceAtStage = [&faceAt, stageM](int f, double t) { return faceAt(f, t) + stageM; };
    if (stageM != 0.0) {
        Log("[swe-cycle] the told river faces' stage raised by %+.3f m (--swe-cycle-stage)", stageM);
    }
    swe.Spinup(gpu, simUnix, 2.0, oceanAt, faceAtStage, faceQAt);
    // PHASE C3: the tidal current station nearest the place that the domain holds.
    int ctSta = -1;
    if (haveCurrents) {
        double la = 0.0, lo = 0.0;
        grid.flat.LatLonOf(0.0, 0.0, la, lo);
        ctSta = NearestStation(currents, la, lo,
                               [&](double a, double b) { return swe.Domain().Holds(a, b); });
    }
    RunSweCycle(gpu, swe, oceanAt, faceAtStage, faceQAt,
                haveCurrents ? &currents : nullptr, ctSta, grid, simUnix,
                opt.sweCycleH, describeWest, focus);
    gpu.WaitIdle();
    resMgr.Shutdown();
    gpu.Shutdown();
    return 0;
}

}  // namespace ga::app::tools
