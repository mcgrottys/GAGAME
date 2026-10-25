// ================================================================================================
//  WaveField.cpp - M8: the solved wave field, ported NUMBER FOR NUMBER from proofs/wave_field.py
//  (the authoritative prototype, itself twin-tested to half an 8-bit LSB against the
//  vqview-inlet reference bake). ALGEBRA.md `wavefield` carries the derivation; every closure
//  constant below is cited from there or from the proof. The hypervisor's step 9c holds this
//  file to the proof through proofs/wave_field_check.py, which re-solves a cached .bin's own
//  input planes in python and diffs texel-for-texel.
//
//  FRAME LEDGER (declared, or the field is ambiguous -- the gauge lesson):
//    grid      row-major, ROW 0 = SOUTH (+v = +z = north; the patch.wrap family, no flips).
//              world x = orgX + (i+0.5)*cellM, z = orgZ + (j+0.5)*cellM (texel centers).
//    gauge     phi = 0 anchored at the SOUTHWEST corner texel; x-cumsum west->east
//              LEFT-INCLUSIVE; y-cumsum SOUTH->NORTH of the ROW-MEAN of k*d_n. (The vqview
//              reference integrated on a row-0-north array; same math, declared traversal.)
//    d^        unit propagation (east, north) -- a VALUE, never flipped, constant per
//              component; the spatial bending lives in the phase field.
//    time      the stored spinor is (cos phi, sin phi) at t = 0 (unix); advance is the rotor
//              (c,s) -> (c*ct + s*st, s*ct - c*st) = cos/sin(phi - sigma*t), t = simUnix.
//              (The GPU kernel should rebase t before casting to float; ProbeAt is double.)
//
//  ENGINE DECISIONS layered over the proof (each one mirrored by the checker):
//    * dry cells (h <= 0): skip the solve -- k held at the blocked value, a = 0. (The proof
//      floors h at 0.15 and solves anyway; solving land buys nothing and costs the hot loop.)
//    * current (W2, 2026-10-24): the water manager's, per cell -- the regional current field
//      (CurrentModel::SampleAt) everywhere, the solver's solved current over it by the solver's
//      domain weight (ComposeFlow). The Merrimack proxy that stood here (an ebb jet aimed toward
//      compass 105, x3.0 from ACT0816, decaying past x = 800 m of one window) is gone.
//    * the amplitude (W1, 2026-10-24): the wave action carried along the eikonal's rays,
//      div(N (c_g k^ + U)) = 0 (ActionTransport) -- refraction, shoaling and the current's action
//      exchange in one law, with no bar normal. The proof's Snell against one fixed frame (and
//      proofs/wave_field_check.py with it) no longer describes this solve.
//    * upload gate: a component ships only while the grid resolves its phase --
//      lambda_deep = 2*pi*g/sigma^2 >= minSamplesPerLambda * cellM; the FFT cascades keep the
//      rest (the fold doctrine). Gated slices stay zero, aMax = 0, table keeps sigma/dir.
//
//  CACHE (cache/wave/<fnv-1a-64>.bin), the compositor's identity-is-content law one
//  directory over. Binary layout (little-endian, no padding -- the python checker's contract):
//      CacheHead  (144 B: magic 'WAVF', version, key, dims, spectrum + bucket provenance)
//      GpuTable   (368 B, exactly the GPU contract struct)
//      atlas      atlasW * atlasH * 4 bytes RGBA8
//      [inputs]   when hasInputs: bed, u, v planes as float32[nx*ny] each, row 0 = south
//  The input planes exist so the checker can re-solve THE SAME problem offline -- the solve
//  is a pure function of (cfg, level, spectrum scalars, bed, u, v) by construction.
// ================================================================================================
// CurrentModel.h must precede WaveField.h: the header names CurrentModel* without declaring
// it (none of its own includes do) -- fine once the engine wiring includes both, but load-
// bearing here.
#include "core/GradeField.h"
#include "core/Image.h"
#include "sim/CurrentModel.h"

#include "sim/WaveField.h"
#include "core/Planet.h"
#include "core/Pga.h"
#include "sim/WaterTerms.h"   // F7: BreakFactor, the clipped Rayleigh sea
#include "core/ThreadManager.h"

#include "hal/Gpu.h"
#include "sim/SweSolver.h"

#include <algorithm>
#include <chrono>
#include <climits>
#include <cmath>
#include <thread>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <functional>
#include <mutex>

namespace ga {

namespace {

constexpr double kPiW = 3.14159265358979323846;
constexpr int kBracketN = 96;        // log-spaced dispersion candidates (proof: nsamp=96)
constexpr int kBisectN = 48;         // bisections -> bracket width ~4e-16 * k
constexpr double kDepthFloor = 0.15; // m; the proof's np.maximum(h, 0.15)
constexpr double kCgFloor = 0.15;    // m/s floor under the action's speed (cg + U_along; ActionTransport)
// Mase's diffraction parameter (H. Mase 2001, "Multi-directional random wave transformation model based on
// energy balance equation", Coastal Engineering Journal 43(4)): kappa = 2.5, his fit of the energy balance's
// transverse diffusion to the diffraction diagrams. The one closure the transport carries.
constexpr double kMaseKappa = 2.5;
constexpr double kGolden = 0.6180339887498949;   // golden-sequence directions
// M9bv: DIRECTIONS PER FREQUENCY. A directional spectrum is S(f, theta) -- two dimensional --
// and the fan used to sample it along a diagonal, one theta per f. Four per frequency turns
// 32 long-crested trains into 8 short-crested groups, which is what a sea actually is.
constexpr int kDirFan = 4;
// A SECOND irrational, for the frequency axis. Reusing kGolden for both would place the
// frequency and the direction of every component at the same fraction of their strata --
// a diagonal through the (f, theta) grid, which is the very correlation the fan is meant
// to break. sqrt(2) - 1 is independent of the golden ratio and equidistributes as well.
constexpr double kGolden2 = 0.41421356237309515;
constexpr double kFreqLo = 0.62, kFreqHi = 2.30; // geomspace band, x fp
constexpr double kExcessMax = 2.5;   // env G channel scale (excess/2.5), fixed
constexpr float kFeatherM = 120.0f;  // edge blend into the cascades, metres
constexpr uint32_t kCacheMagic = 0x46564157u;    // 'WAVF' on disk (little-endian)
constexpr uint32_t kCacheVersion = 2;            // v2 = input planes ride after the atlas
constexpr double kPartHsFloor = 0.05;            // below this no partition drives a solve

// The bracket grid, np.logspace(-4.0, 0.7, 96) verbatim: linspace exponents i*step + start
// with the ENDPOINT PINNED exactly (numpy sets y[-1] = stop), then 10^x. The hold for
// blocked/dry cells is kk[95]*0.25 ~ 1.2530 rad/m -- a GRID-TUNED closure (k*cell must stay
// under pi; re-derive it for a different cell size, ALGEBRA.md `wavefield`).
const double* BracketGrid() {
    static double kk[kBracketN];
    static const bool init = [] {
        const double step = 4.7 / 95.0;
        for (int i = 0; i < kBracketN; ++i) kk[i] = std::pow(10.0, double(i) * step - 4.0);
        kk[kBracketN - 1] = std::pow(10.0, 0.7);
        return true;
    }();
    (void)init;
    return kk;
}

// (sigma + k*Uopp)^2 = g k tanh(k h): first +to- sign change on the log grid selects the
// physical branch (cg_r > Uopp), then bisection. Newton is structurally unsafe here -- the
// root pair MERGES at blocking (f' -> 0; the reference measured a 628 km wavelength from
// it). No sign change = BLOCKED: arrested, standing, breaking -- flagged, never papered over.
double SolveDispersion(double sig, double hf, double uopp, bool& blocked) {
    const double* kk = BracketGrid();
    auto f = [&](double k) {
        const double s = sig + k * uopp;
        return s * s - (PlanetGWave() * k) * std::tanh(k * hf);
    };
    double fPrev = f(kk[0]);
    double lo = 0.0, hi = 0.0;
    bool found = false;
    for (int i = 1; i < kBracketN; ++i) {
        const double fCur = f(kk[i]);
        if (fPrev > 0.0 && fCur <= 0.0) {
            lo = kk[i - 1];
            hi = kk[i];
            found = true;
            break;
        }
        fPrev = fCur;
    }
    if (!found) {
        blocked = true;
        return kk[kBracketN - 1] * 0.25;
    }
    blocked = false;
    for (int it = 0; it < kBisectN; ++it) {
        const double mid = 0.5 * (lo + hi);
        if (f(mid) <= 0.0) hi = mid;
        else lo = mid;
    }
    return 0.5 * (lo + hi);
}

// Dequant scale rule: 1.05 * field max, rounded UP to 3 decimals, never under 1e-3 (a zero
// scale is the "slice unused" marker and must stay deliberate).
double Ceil3(double v) {
    const double r = std::ceil(v * 1000.0) / 1000.0;
    return (r > 1e-3) ? r : 1e-3;
}

// The reference exporter's TRUNCATING 8-bit quantizer: byte = floor(clip(x/max)*255).
// Decode center is (byte + 0.5)/255 -- the checker and ProbeAt both use it, which is why a
// faithful port lands at exactly <= 0.5 LSB.
uint8_t Quant8(double x, double maxv) {
    double t = x / maxv;
    if (!(t > 0.0)) t = 0.0;   // also swallows NaN, which must never reach the atlas
    if (t > 1.0) t = 1.0;
    const double q = std::floor(t * 255.0);
    return static_cast<uint8_t>((q < 255.0) ? q : 255.0);
}

double Wrap360(double deg) {
    double w = std::fmod(deg, 360.0);
    if (w < 0.0) w += 360.0;
    return w;   // matches python's floored %: [0, 360)
}

// The solver's row split. It used to SPAWN AND JOIN max(4, hw-2) threads on every call, at three
// call sites, nested inside the solve worker -- 14 threads per call on this machine, created and
// destroyed each time. It is the pool's ParallelFor now.
//
// THE CHUNKING IS UNCHANGED ON PURPOSE: the same nThreads expression gives the same chunk size,
// so every call gets the identical row ranges it always did. The rows write disjoint output and
// the only shared accumulator is an atomic counter, so the split could not change the answer --
// but keeping it identical means the solve is not a variable in this commit's gate.
void ParallelRows(int ny, const std::function<void(int, int)>& fn) {
    if (ny <= 0) return;
    const unsigned hw = std::thread::hardware_concurrency();
    const int nThreads = (std::max)(4, (hw > 2u) ? int(hw - 2u) : 4);
    const int chunk = (ny + nThreads - 1) / nThreads;
    Threads().ParallelFor(Lane::Long, "wave.rows", ny, chunk, fn);   // F13: seconds of rows
}

void CachePathFor(uint64_t key, char* buf, size_t n) {
    snprintf(buf, n, "cache/wave/%016llx.bin", static_cast<unsigned long long>(key));
}

// M8h: LRU prune. The cache is content-addressed and an entry is ~112 MB with its input
// planes -- a day of tide buckets mints gigabytes and nothing ever asked it to stop
// (seen: the disk, quietly). Newest-first by write time under a byte budget; the LIVE
// key is always spared (losing it only costs a re-solve, but the renderer would hitch).
// Errors are ignored file-by-file: pruning is a courtesy, never a failure mode.
void PruneWaveCache(uint64_t liveKey) {
    namespace fs = std::filesystem;
    constexpr uint64_t kBudgetBytes = 4ull << 30;   // 4 GiB ~ 36 buckets
    std::error_code ec;
    struct Entry {
        fs::file_time_type t;
        uint64_t bytes;
        fs::path p;
    };
    std::vector<Entry> files;
    for (fs::directory_iterator it("cache/wave", ec), end; !ec && it != end;
         it.increment(ec)) {
        if (!it->is_regular_file(ec) || it->path().extension() != ".bin") continue;
        std::error_code e2;
        files.push_back({it->last_write_time(e2), it->file_size(e2), it->path()});
    }
    std::sort(files.begin(), files.end(),
              [](const Entry& a, const Entry& b) { return a.t > b.t; });
    char livePath[128];
    CachePathFor(liveKey, livePath, sizeof(livePath));
    const fs::path liveName = fs::path(livePath).filename();
    uint64_t acc = 0, freed = 0;
    int pruned = 0;
    for (const Entry& f : files) {
        acc += f.bytes;
        if (acc <= kBudgetBytes || f.p.filename() == liveName) continue;
        std::error_code e3;
        if (fs::remove(f.p, e3)) {
            ++pruned;
            freed += f.bytes;
        }
    }
    if (pruned) {
        Log("[wave] cache pruned: %d entries / %.1f GB beyond the 4 GiB budget", pruned,
            double(freed) / (1024.0 * 1024.0 * 1024.0));
    }
}
}  // namespace

// ------------------------------------------------------------------------------------------------
//  wavecore -- the pure solve, EXTERNAL linkage on purpose: the offline twin harness drives
//  these directly (no engine, no GPU, no data directory) to produce a cache .bin for
//  proofs/wave_field_check.py before the renderer wiring exists. Not part of the class
//  contract; the engine reaches them only through WaveField.
// ------------------------------------------------------------------------------------------------
namespace wavecore {

struct Inputs {
    WaveFieldConfig cfg;
    double level = 0.0;                 // bucketed water level, NAVD m -- the solve's datum
    double currentMs = 0.0;             // bucketed signed ACT speed (+flood/-ebb), provenance
    double hs = 0.0, tp = 0.0, mwdFromDeg = 0.0;   // spectrum scalars; hs <= 0 = zero field
    std::vector<float> bed, u, v;       // nx*ny planes, row 0 = south
};

#pragma pack(push, 1)
struct CacheHead {
    uint32_t magic = 0, version = 0;
    uint64_t key = 0;
    uint32_t tableBytes = 0, atlasW = 0, atlasH = 0, nx = 0, ny = 0, nComp = 0,
             hasInputs = 0, pad0 = 0;
    double hs = 0, tp = 0, mwdDeg = 0;
    double cellM = 0, spreadDeg = 0, barNormalDeg = 0, gammaHs = 0, minSamplesPerLambda = 0;
    double orgX = 0, orgZ = 0, level = 0, currentMs = 0;
};
#pragma pack(pop)
static_assert(sizeof(CacheHead) == 144, "cache head layout is the python checker's contract");
// M9bl: 368 -> 688 with kMaxComp 16 -> 32 (48 fixed bytes + 5 float[kMaxComp] arrays).
// Deliberately a literal: the number is the contract, so growing the table has to be an
// edit someone reads. Old cache entries reject themselves on the tableBytes check in
// LoadCached, and kSolverVersion bumps so the bucket key changes too.
static_assert(sizeof(WaveField::GpuTable) == 688, "GpuTable layout is the GPU/cache contract");

// The solve: proofs/wave_field.py build_spectrum() + component() + dispersion_k(), one pass
// per component (rows parallel; the gauge cumsum sequential -- cheap), then the TOTAL-Hs
// limiter (ONE uniform factor on the rms envelope; per-line caps measured 6.8x over in the
// shallows, ALGEBRA.md) and the truncating quantizer into the 2-wide slice atlas.
// THE EIKONAL SWEEP, as a function: the phase of one component over the window, |grad phi| = feik,
// by eight Godunov sweeps (two rounds of the four orders), the inflow edges held at the incident
// deep-water plane k_inf d^ between sweeps. Out of Solve so the selftest can run it on a field of
// its own (RunWaveFieldSelfTest) -- the gauge that used to run INSIDE every solve (two more
// integrations and a residual over 1.75 million cells, printing the same answer each time) runs
// there now, once, where a failure fails the build and not a frame.
// F28: THE SWEEP RUNS UNTIL IT IS STILL. Two rounds of the four orders were a count written into
// the law, and the count was wrong where it mattered: measured on the helm window (the converge
// tool, 2026-10-06), a third round still moved up to 970k cells of a component by up to 35 rad,
// the field was still only after 3-8 more rounds, and 4.6-5.4 % of the drawn phase cells -- the
// whole inlet between the jetties, the basin, the river -- were a two-round iterate, not the
// eikonal solution. A fixed point has no number in it: a round of the four orders is swept until
// a round moves no cell (the update only lowers, bounded below by the seed, so it ends). Returns
// the rounds it took. `passes` > 0 is the instrument's probe (the converge tool): exactly that
// many passes from the phase it is given, no fixed point.
// `wet` + `mode` (W1, 2026-10-24): 0 sweeps every cell (the law before); 1 sweeps the WATER alone -- a dry cell
// is neither seeded nor updated and stays unreached, a wall the phase goes around; 2 fills the LAND from the
// water's edge with the water frozen (EikonalPhase below says why).
int EikonalSweep(int nx, int ny, double dxm, double kInfC, double d0, double d1,
                        const std::vector<double>& feik, std::vector<double>& phibuf, int passes = 0,
                        const std::vector<double>* wet = nullptr, int mode = 0) {
    auto skip = [&](size_t idx) {
        if (!wet || mode == 0) return false;
        const bool isWet = (*wet)[idx] > 0.0;
        return (mode == 1) ? !isWet : isWet;
    };
    const double kInf = 1e30;
    const int ib = (d0 >= 0.0) ? 0 : (nx - 1);      // the inflow corner
    const int jb = (d1 >= 0.0) ? 0 : (ny - 1);
    auto seed = [&]() {
        for (int i = 0; i < nx; ++i) {
            const size_t idx = size_t(jb) * size_t(nx) + size_t(i);
            if (!skip(idx)) phibuf[idx] = kInfC * d0 * double(i - ib) * dxm;
        }
        for (int j = 0; j < ny; ++j) {
            const size_t idx = size_t(j) * size_t(nx) + size_t(ib);
            if (!skip(idx)) phibuf[idx] = kInfC * d1 * double(j - jb) * dxm;
        }
    };
    seed();
    constexpr int kRoundCap = 256;   // a guard on the loop, not a law: never reached (3-8 measured)
    int rounds = 0;
    bool moved = true;
    for (int pass = 0; passes > 0 ? (pass < passes) : (moved || (pass & 3) != 0); ++pass) {
        if ((pass & 3) == 0) {   // a round begins: nothing has moved in it yet
            if (passes <= 0 && rounds >= kRoundCap) {
                Log("[wave] eikonal: %d rounds and still moving -- the guard stopped it", rounds);
                break;
            }
            moved = false;
            ++rounds;
        }
        seed();
        const bool xr = (pass & 1) != 0, yr = (pass & 2) != 0;
        for (int jj = 0; jj < ny; ++jj) {
            const int j = yr ? (ny - 1 - jj) : jj;
            const size_t row = size_t(j) * size_t(nx);
            for (int ii = 0; ii < nx; ++ii) {
                const int i = xr ? (nx - 1 - ii) : ii;
                const size_t idx = row + size_t(i);
                if (skip(idx)) continue;
                const double ax = (std::min)((i > 0) ? phibuf[idx - 1] : kInf,
                                             (i + 1 < nx) ? phibuf[idx + 1] : kInf);
                const double ay =
                    (std::min)((j > 0) ? phibuf[idx - size_t(nx)] : kInf,
                               (j + 1 < ny) ? phibuf[idx + size_t(nx)] : kInf);
                const double lo = (ax < ay) ? ax : ay;
                if (lo >= kInf) continue;
                const double f = feik[idx] * dxm;
                const double dif = ax - ay;
                const double cand = (std::abs(dif) >= f)
                                        ? (lo + f)
                                        : 0.5 * (ax + ay +
                                                 std::sqrt(2.0 * f * f - dif * dif));
                if (cand < phibuf[idx]) {
                    phibuf[idx] = cand;
                    moved = true;
                }
            }
        }
    }
    seed();   // the last sweep must not be allowed to undercut it either
    return rounds;
}

// THE PHASE GOES AROUND LAND (W1, 2026-10-24). A dry cell carries the wet field's shoreline k (kShore) so a
// footprint straddling the beach interpolates a continuing wave -- and swept with the water, that made land
// passable: across a jetty ten cells wide the path THROUGH it was the cheaper first arrival, so the Merrimack's
// entrance channel took its crests straight through the north jetty, and the action (which land does not
// send) left the channel dark. The water is solved first with land a wall (the rays go round the tips and
// diffract into the lee); then the land's phase is filled from the water's edge with the water frozen, the
// water's own phase continued for the shore's interpolation and nothing more. Returns the rounds of both.
int EikonalPhase(int nx, int ny, double dxm, double kInfC, double d0, double d1, const std::vector<double>& h,
                 const std::vector<double>& feik, std::vector<double>& phibuf) {
    const int water = EikonalSweep(nx, ny, dxm, kInfC, d0, d1, feik, phibuf, 0, &h, 1);
    return water + EikonalSweep(nx, ny, dxm, kInfC, d0, d1, feik, phibuf, 0, &h, 2);
}

// W1 (2026-10-24, Mark: "refraction by action transport along the eikonal, no bar normal"). THE
// AMPLITUDE IS THE WAVE ACTION, CARRIED. A component's action density N = A^2 / sigma_r rides the
// absolute group velocity F = c_g k^ + U, and in a steady sea it is conserved:
//
//     div (N F) = 0,      k^ = grad phi / |grad phi|   (the eikonal's own direction field)
//
// which is refraction, shoaling and the current's action exchange in one statement. Its limit on
// straight parallel contours with no current is Snell and energy-flux shoaling exactly,
// A^2 = a0^2 (cg0 / cg)(cos th0 / cos th) -- the law that stood here, against ONE declared bar
// normal for the whole window and spreading only (Kr <= 1). Carried along the eikonal's rays it
// needs no normal, and a shoal that bends rays together raises the sea behind it (focusing).
//
// Discretized on the cells along the EIKONAL'S OWN RAYS: each cell's direction is the upwind gradient
// the phase was solved with (so its velocity is the winning ray family's), it drains by that velocity
// through every face it leaves by, and along each axis it is fed from the one side its velocity comes
// from, by that neighbour's own flow toward it. So a cell's balance is
//
//     N_i (|F_i,x| + |F_i,z|)  =  sum over the axes of (F_src . n_in)+ N_src
//                                  + the incident swell where the source is the window's outside,
//
// marched in phase order (below) until a pass moves no cell by more than kStill of the deep action (the
// F28 law: a fixed point has no count in it). Every wet cell drains by its own velocity, whose speed is
// at least kCgFloor, so no cell is a sink; the side is the action's velocity's, not the phase's, so a
// current turning a component of F against k^ moves the action rather than losing it.
// WHERE TWO RAY FAMILIES MEET -- a crease of the first-arrival phase: off a jetty's tip, behind a
// shoal -- the losing family's flux ENDS: a crease cell's velocity is the winner's, and it is fed from
// the winner's side. Past the crease it would be a second sea crossing the first, and a single-valued
// phase holds one. Two versions taught it (2026-10-24, the helm window): faces with averaged velocities
// poured the losing family into the crease (lines of 3x the sea, a 27.8 m raw peak where the bar
// normal's field had 8.3 m); a centred direction fed a crease from both sides, being their bisector.
//
// DIFFRACTION, ACROSS THE RAY (Mase 2001, the energy balance equation with diffraction, EBED): geometric
// optics alone leaves a jetty's lee dark -- energy reaches it only by the fan of rays from the tip cell --
// and draws every convergence of rays as a line one cell wide (2026-10-24, the layer sheet: the inlet black,
// a streak off the north jetty). Mase's term spreads action along the crest,
//
//     kappa / (2 sigma) * [ d_n (C C_g d_n N) - 1/2 C C_g d_nn N ],     n along the crest, kappa = 2.5,
//
// written there on a grid axis with a cos^2 weight; here n is the local crest itself, from the eikonal's ray.
// On a field uniform along its crests it is zero (deep water stays exact); on the line a jetty's tip casts it
// leaves about a third of the energy (0.37 in [wavetest]; the random-wave diffraction diagrams give about half).
//
// THE BOUNDARY IS THE INCIDENT SWELL, stated as the phase's is. The eikonal holds the deep plane on the
// inflow edges, which fixes the tangential wavenumber on the assumption that the contours run along the
// edge; under that same assumption the action crossing the edge per metre of it is conserved from deep
// water, Phi0 (d^ . n_in)+ with Phi0 = a0^2 cg0 / sigma. (The first version used the refracted ray's own
// cosine there, which on a shallow edge the swell grazes -- the Merrimack's south edge, along Plum Island --
// overfed it 3.5x. A review caught it; the straight-contour test had been normalised to the same mistake.)
// Returns the rounds; `aOut` gets A = sqrt(sigma_r N) (0 on land), `flux` and `N` are the caller's.
// `stats` (the instrument): the carried amplitude over the swell shoaled onto each cell with no
// refraction, a0 sqrt(sigma_r cg0 / (sigma |F|)) -- what refraction and the creases did, by cell.
struct CarryStats {
    double p50 = 0.0, p99 = 0.0, max = 0.0;
    double over2 = 0.0;   // the share of wet cells carried above twice the shoaled swell
    int iMax = -1, jMax = -1;
};
int ActionTransport(int nx, int ny, double dxm, double sig, double d0, double d1, double a0,
                    const std::vector<double>& h, const double* kbuf, const std::vector<double>& phi,
                    const std::vector<float>& u, const std::vector<float>& v, std::vector<float>& flux,
                    std::vector<double>& N, std::vector<uint32_t>& order, std::vector<float>& ray, float* aOut,
                    const uint8_t* blockedBits = nullptr, CarryStats* stats = nullptr, float* gainOut = nullptr) {
    const size_t cells = size_t(nx) * size_t(ny);
    const bool haveFlow = u.size() == cells && v.size() == cells;
    const double g = PlanetGWave();
    auto sigmaR = [&](double k, double hc) {   // the intrinsic frequency the dispersion solved for
        const double hf = (hc > kDepthFloor) ? hc : kDepthFloor;
        return std::sqrt(g * k * std::tanh((std::min)(k * hf, 30.0)));
    };
    // THE RAY AT A CELL IS THE EIKONAL'S OWN. The direction is the upwind gradient the Godunov update
    // solved with -- along each axis the neighbour of lower phase, the steeper of the two where both are
    // lower (a crease: the winning family's) -- the one-sided difference inward at the window's edge.
    // The returned code (1 the lower neighbour, 2 the higher, 3 the edge, 0 none) is the side.
    auto P = [&](int i, int j) { return phi[size_t(j) * size_t(nx) + size_t(i)]; };
    auto upwind = [&](int i, int j, int n, bool alongX, double& gOut) -> uint8_t {
        const int c = alongX ? i : j;
        auto at = [&](int k) { return alongX ? P(k, j) : P(i, k); };
        gOut = 0.0;
        if (n < 2) return 0;
        if (c == 0) {
            gOut = (at(1) - at(0)) / dxm;
            return (gOut > 0.0) ? 3 : (gOut < 0.0) ? 2 : 0;
        }
        if (c == n - 1) {
            gOut = (at(c) - at(c - 1)) / dxm;
            return (gOut > 0.0) ? 1 : (gOut < 0.0) ? 3 : 0;
        }
        const double dm = (at(c) - at(c - 1)) / dxm, dp = (at(c + 1) - at(c)) / dxm;
        if (dm > 0.0 && dm >= -dp) {
            gOut = dm;
            return 1;
        }
        if (-dp > 0.0) {
            gOut = dp;
            return 2;
        }
        return 0;
    };
    for (int j = 0; j < ny; ++j) {
        for (int i = 0; i < nx; ++i) {
            const size_t idx = size_t(j) * size_t(nx) + size_t(i);
            float* F = &flux[2 * idx];
            float* R = &ray[3 * idx];
            if (h[idx] <= 0.0) {
                F[0] = F[1] = 0.0f;
                R[0] = R[1] = R[2] = 0.0f;
                continue;
            }
            double gx = 0.0, gz = 0.0;
            upwind(i, j, nx, true, gx);
            upwind(i, j, ny, false, gz);
            const double gl = std::sqrt(gx * gx + gz * gz);
            const double ke = (gl > 0.0) ? gx / gl : d0, kn = (gl > 0.0) ? gz / gl : d1;
            const double k = kbuf[idx];
            double kh = k * ((h[idx] > kDepthFloor) ? h[idx] : kDepthFloor);
            if (kh < 1e-4) kh = 1e-4;
            else if (kh > 30.0) kh = 30.0;
            const double cg = 0.5 * (1.0 + (2.0 * kh) / std::sinh(2.0 * kh)) * sigmaR(k, h[idx]) / k;
            R[0] = float(ke);   // the ray, and C C_g -- the diffraction's coefficient (below)
            R[1] = float(kn);
            R[2] = float(sigmaR(k, h[idx]) / k * cg);
            const double ue = haveFlow ? double(u[idx]) : 0.0, un = haveFlow ? double(v[idx]) : 0.0;
            const double along = ue * ke + un * kn;
            const double cgEff = (std::max)(cg + along, kCgFloor);
            F[0] = float(ke * cgEff + (ue - along * ke));
            F[1] = float(kn * cgEff + (un - along * kn));
        }
    }
    const double cg0 = 0.5 * g / sig;
    const double phi0 = a0 * a0 * cg0 / sig;   // the deep swell's action flux per crest
    const double nDeep = a0 * a0 / sig;        // and its density, the measure of "still"
    constexpr double kStill = 1e-7;
    std::fill(N.begin(), N.end(), 0.0);
    // THE DIFFRACTION IS A MARCH ALONG THE RAY. Mase's term makes the steady balance parabolic: along the
    // ray the action is carried, across it it spreads. Solved as written -- the crest coupling implicit --
    // the sweep was elliptic and crawled (2026-10-24: hundreds of rounds, cell by cell or line by line).
    // So it is integrated the way it runs: the action ARRIVING from an upwind neighbour has spread along
    // that neighbour's crest over the step it travelled, ds = dx |k^ . axis|,
    //
    //     S(N)_nb = (1 - 2r) N(q) + r N(q + dn n^) + r N(q - dn n^),   r = D ds / (|F| dn^2),
    //     D = kappa C Cg / (4 sigma)  (Mase's coefficient on N_nn),   dn = max(dx, sqrt(2 D ds / |F|)),
    //
    // so r <= 1/2 and every weight is positive; Mase's drift, kappa/(2 sigma) (C Cg)_n N_n, is the shift
    // of q along the crest by kappa (C Cg)_n ds / (2 sigma |F|) toward larger C Cg. The samples are
    // bilinear; land and the window's outside give back the neighbour's own value (no energy crosses a
    // wall). Every dependency is upstream, so it converges as the geometric transport does. On a field
    // uniform along its crests S is the identity: deep water and a plane wave stay exact.
    // bilinear N at cell coordinates: walls and the outside mirror `self`; a corner at or past `phiLim` (the
    // receiving cell's phase) is left out and the rest renormalised, so every sample is upstream of the cell
    // it feeds and one pass in phase order is the march (the corners just downstream held it to dozens)
    auto Nat = [&](double x, double z, double self, double phiLim) {
        const int i0 = int(std::floor(x)), j0 = int(std::floor(z));
        const double fx = x - double(i0), fz = z - double(j0);
        double v = 0.0, wt = 0.0;
        for (int c = 0; c < 4; ++c) {
            const int ci = i0 + (c & 1), cj = j0 + (c >> 1);
            const double w = ((c & 1) ? fx : 1.0 - fx) * ((c >> 1) ? fz : 1.0 - fz);
            if (!(w > 0.0)) continue;
            const size_t cn = size_t(cj) * size_t(nx) + size_t(ci);
            if (ci < 0 || cj < 0 || ci >= nx || cj >= ny || h[cn] <= 0.0) {
                v += w * self;
                wt += w;
                continue;
            }
            if (!(phi[cn] < phiLim)) continue;
            v += w * N[cn];
            wt += w;
        }
        return (wt > 0.0) ? v / wt : self;
    };
    auto Aat = [&](double x, double z, double self) {   // bilinear C Cg, walls mirror `self`
        const int i0 = int(std::floor(x)), j0 = int(std::floor(z));
        const double fx = x - double(i0), fz = z - double(j0);
        double v = 0.0;
        for (int c = 0; c < 4; ++c) {
            const int ci = i0 + (c & 1), cj = j0 + (c >> 1);
            const double w = ((c & 1) ? fx : 1.0 - fx) * ((c >> 1) ? fz : 1.0 - fz);
            if (!(w > 0.0)) continue;
            const size_t cn = size_t(cj) * size_t(nx) + size_t(ci);
            v += w * ((ci < 0 || cj < 0 || ci >= nx || cj >= ny || h[cn] <= 0.0) ? self : double(ray[3 * cn + 2]));
        }
        return v;
    };
    // the action arriving from cell (ni, nj) after `ds` metres of travel along its ray
    auto arriving = [&](int ni, int nj, double ds, double phiLim) {
        const size_t nb = size_t(nj) * size_t(nx) + size_t(ni);
        const double self = N[nb];
        const float* R = &ray[3 * nb];
        const float* G = &flux[2 * nb];
        const double fm = (std::max)(std::sqrt(double(G[0]) * G[0] + double(G[1]) * G[1]), kCgFloor);
        const double D = kMaseKappa * double(R[2]) / (4.0 * sig);
        if (!(D > 0.0) || !(ds > 0.0)) return self;
        const double cx = -double(R[1]), cz = double(R[0]);   // along the crest, in cells per cell
        const double dn = (std::max)(dxm, std::sqrt(2.0 * D * ds / fm));
        const double r = D * ds / (fm * dn * dn);
        const double an = (Aat(ni + cx, nj + cz, double(R[2])) - Aat(ni - cx, nj - cz, double(R[2]))) / (2.0 * dxm);
        // cells along the crest, at most one: the drift's step can outrun the march (a bar's face, a jetty's
        // toe: (C Cg)_n of 15 m/s^2 shifted 5-10 cells), and past a cell the sample is no longer this term's
        const double shift = std::clamp((kMaseKappa * an * ds / (2.0 * sig * fm)) / dxm, -1.0, 1.0);
        const double qx = double(ni) + shift * cx, qz = double(nj) + shift * cz, dc = dn / dxm;
        return (1.0 - 2.0 * r) * Nat(qx, qz, self, phiLim) + r * Nat(qx + dc * cx, qz + dc * cz, self, phiLim) +
               r * Nat(qx - dc * cx, qz - dc * cz, self, phiLim);
    };
    // THE MARCH RUNS IN PHASE ORDER. Every ray runs from lower phase to higher, so the wet cells sorted by
    // their phase put almost every cell after all it depends on -- its upwind neighbours, and the crest it
    // spreads from -- and a pass settles what four sweep orders took a hundred rounds to (2026-10-24: the
    // crest samples' corners sit just downstream of a cell as often as not). A pass is repeated until still,
    // for the few links that run against the phase (a current turning the action's velocity, a corner).
    // Two stages, sequenced: the geometric transport, then the diffraction from it.
    size_t nWet = 0;
    for (size_t idx = 0; idx < cells; ++idx) {
        if (h[idx] > 0.0) order[nWet++] = uint32_t(idx);
    }
    std::sort(order.begin(), order.begin() + ptrdiff_t(nWet), [&](uint32_t x, uint32_t y) { return phi[x] < phi[y]; });
    constexpr int kPassCap = 64;   // a guard on each stage's loop, not a law
    auto sweep = [&](bool diffract) {
        int passes = 0;
        bool moved = true;
        while (moved) {
            if (passes >= kPassCap) {
                Log("[wave] action: %d passes and still moving%s -- the guard stopped it", passes,
                    diffract ? " (the diffraction)" : "");
                break;
            }
            moved = false;
            ++passes;
            for (size_t o = 0; o < nWet; ++o) {
                const size_t idx = order[o];
                const int i = int(idx % size_t(nx)), j = int(idx / size_t(nx));
                const float* F = &flux[2 * idx];
                // out: every face this cell's own flow leaves by, whatever is beyond it
                const double out = std::abs(double(F[0])) + std::abs(double(F[1]));
                double in = 0.0;
                // in: along each axis, from the side the action's own velocity comes from -- the neighbour's
                // flow toward this cell, its action spread along its crest over the step (the diffraction) --
                // or, where that side is the window's outside, the deep swell's flux through the edge
                for (int ax = 0; ax < 2; ++ax) {
                    const double fc = double(F[ax]);
                    if (fc == 0.0) continue;
                    const int sg = (fc > 0.0) ? -1 : 1;   // the source's offset: against the flow
                    const int ni = (ax == 0) ? i + sg : i, nj = (ax == 1) ? j + sg : j;
                    if (ni < 0 || nj < 0 || ni >= nx || nj >= ny) {
                        // the incident swell, Phi0 (d^ . n_in): the edge's flux under the phase seed's own
                        // assumption (contours along the edge), not the refracted ray's cosine
                        const double dIn = (ax == 0) ? -double(sg) * d0 : -double(sg) * d1;
                        if (dIn > 0.0) in += phi0 * dIn;
                        continue;
                    }
                    const size_t nb = size_t(nj) * size_t(nx) + size_t(ni);
                    if (h[nb] <= 0.0) continue;   // land sends nothing
                    const double gn = -double(sg) * double(flux[2 * nb + size_t(ax)]);   // its flow toward this cell
                    if (gn > 0.0) {
                        in += gn * (diffract ? arriving(ni, nj, dxm * std::abs(double(ray[3 * nb + size_t(ax)])), phi[idx])
                                             : N[nb]);
                    }
                }
                const double nNew = in / (std::max)(out, kCgFloor);
                if (std::abs(nNew - N[idx]) > kStill * nDeep) moved = true;
                N[idx] = nNew;
            }
        }
        return passes;
    };
    const int roundsGeo = sweep(false);   // first: the operands of + are unsequenced
    const int rounds = roundsGeo + sweep(true);
    std::vector<float> gains;
    if (stats) gains.reserve(cells / 4);
    for (size_t idx = 0; idx < cells; ++idx) {
        if (!(h[idx] > 0.0)) {
            aOut[idx] = 0.0f;
            if (gainOut) gainOut[idx] = 0.0f;
            continue;
        }
        // a blocked cell holds a stand-in k (kHold), whose intrinsic frequency means nothing: the arrested
        // wave's action is read at the component's own sigma there
        const bool blocked = blockedBits && ((blockedBits[idx >> 3] >> (idx & 7)) & 1u);
        const double sr = blocked ? sig : (std::max)(sigmaR(kbuf[idx], h[idx]), 0.05 * sig);
        aOut[idx] = float(std::sqrt(sr * N[idx]));
        if (!stats && !gainOut) continue;
        const double fl = std::sqrt(double(flux[2 * idx]) * flux[2 * idx] + double(flux[2 * idx + 1]) * flux[2 * idx + 1]);
        const double shoaled = a0 * std::sqrt(sr * cg0 / (sig * (std::max)(fl, kCgFloor)));
        const double gain = (shoaled > 0.0) ? double(aOut[idx]) / shoaled : 0.0;
        if (gainOut) gainOut[idx] = float(gain);
        if (!stats) continue;
        gains.push_back(float(gain));
        if (gain > stats->max) {
            stats->max = gain;
            stats->iMax = int(idx % size_t(nx));
            stats->jMax = int(idx / size_t(nx));
        }
    }
    if (stats && !gains.empty()) {
        size_t n2 = 0;
        for (const float x : gains) n2 += (x > 2.0f) ? 1 : 0;
        stats->over2 = double(n2) / double(gains.size());
        std::nth_element(gains.begin(), gains.begin() + gains.size() / 2, gains.end());
        stats->p50 = gains[gains.size() / 2];
        const size_t k99 = (std::min)(gains.size() - 1, size_t(0.99 * double(gains.size())));
        std::nth_element(gains.begin(), gains.begin() + k99, gains.end());
        stats->p99 = gains[k99];
    }
    return rounds;
}

// THE GAUGE: |grad phi| / k - 1 over the wet interior, as a distribution. An eikonal solution is
// allowed kinks: where two ray families meet (behind a jetty, over a focusing shoal) the viscosity
// solution takes first arrival and creases, and a centred difference across a crease reads a
// gradient that is not the local one. Those cells are the physics (the crease is a caustic), so
// the percentiles say how much of the field is smooth and the max how sharp the sharpest crease
// is, reported apart. Sampled only where all four neighbours are wet: across the waterline the
// speed field steps from the wet k to the shoreline limit on purpose.
struct EikonalGauge {
    double p50 = 0.0, p90 = 0.0, p99 = 0.0, max = 0.0;
    size_t cells = 0;
};
EikonalGauge GaugeOf(int nx, int ny, double cellM, const std::vector<double>& h,
                            const std::vector<double>& kbuf, const std::vector<double>& phibuf) {
    std::vector<double> rs;
    rs.reserve(size_t(nx) * size_t(ny) / 4);
    for (int j = 1; j < ny - 1; ++j) {
        const size_t row = size_t(j) * size_t(nx);
        for (int i = 1; i < nx - 1; ++i) {
            const size_t idx = row + size_t(i);
            if (h[idx] <= 0.0 || h[idx - 1] <= 0.0 || h[idx + 1] <= 0.0 ||
                h[idx - size_t(nx)] <= 0.0 || h[idx + size_t(nx)] <= 0.0) continue;
            const double gx = (phibuf[idx + 1] - phibuf[idx - 1]) / (2.0 * cellM);
            const double gz = (phibuf[idx + size_t(nx)] - phibuf[idx - size_t(nx)]) / (2.0 * cellM);
            rs.push_back(std::abs(std::sqrt(gx * gx + gz * gz) - kbuf[idx]) / kbuf[idx]);
        }
    }
    std::sort(rs.begin(), rs.end());
    EikonalGauge g;
    g.cells = rs.size();
    if (rs.empty()) return g;
    auto pct = [&](double q) { return rs[(std::min)(rs.size() - 1, size_t(q * double(rs.size())))]; };
    g.p50 = pct(0.50);
    g.p90 = pct(0.90);
    g.p99 = pct(0.99);
    g.max = rs.back();
    return g;
}

// F25: the last solve's wavenumbers, by cell and component, with the planes they answer
// (WaveField.h m_memo). A cell whose depth and flow are bit for bit what they were, in a
// component whose frequency and direction are what they were, asked the same question of the
// same function; its k is the memo's. Everything else is solved. The blocked bit rides along
// (it is a statistic of the solve, not a plane the GPU reads).
struct Memo {
    int nx = 0, ny = 0, nc = 0;
    std::vector<double> h;                 // the depth plane the k planes were solved on
    std::vector<float> u, v;               // the flow planes
    double sig[WaveField::kMaxComp] = {}, d0[WaveField::kMaxComp] = {}, d1[WaveField::kMaxComp] = {};
    std::vector<double> k;                 // nc planes of nx*ny
    std::vector<uint8_t> blocked;          // nc planes of nx*ny bits (nx is a multiple of 8)
};
static bool g_solveAudit = false;

// --wave-map's LAYERS (Mark, 2026-10-24: "show every layer (kinda like how graphics programmers debug
// layers) from the inputs and different steps in the pipeline to produce the output"). One PNG per stage of
// the solve, on its own cells, north up, land dark, beside the map: <map>_<n>_<stage>.png. The inputs, then
// each step of a component (the most energetic one, and the one the action carried hottest), then the sea's
// sums and the limiter, then the map itself (eta). An instrument: nothing here touches the answer.
struct LayerSink {
    std::wstring base;   // the map's path less its extension; empty = off
    int nx = 0, ny = 0;
    const std::vector<double>* h = nullptr;
    static void Seq(double t, uint8_t* px) {   // dark violet -> teal -> yellow
        t = (std::min)((std::max)(t, 0.0), 1.0);
        const double a[3][3] = {{68, 1, 84}, {33, 145, 140}, {253, 231, 37}};
        const int k = (t < 0.5) ? 0 : 1;
        const double f = (t < 0.5) ? t * 2.0 : (t - 0.5) * 2.0;
        for (int c = 0; c < 3; ++c) px[c] = uint8_t(a[k][c] + f * (a[k + 1][c] - a[k][c]));
    }
    static void Div(double t, uint8_t* px) {   // blue (-1) -> white (0) -> red (+1)
        t = (std::min)((std::max)(t, -1.0), 1.0);
        const double lo[3] = {59, 76, 192}, mid[3] = {245, 245, 245}, hi[3] = {180, 4, 38};
        const double* e = (t < 0.0) ? lo : hi;
        const double f = std::abs(t);
        for (int c = 0; c < 3; ++c) px[c] = uint8_t(mid[c] + f * (e[c] - mid[c]));
    }
    static void Hue(double ang, double val, uint8_t* px) {   // direction as hue, strength as value
        const double hh = std::fmod(ang / (2.0 * 3.14159265358979) + 1.0, 1.0) * 6.0;
        const int si = int(hh) % 6;
        const double f = hh - std::floor(hh);
        const double v = (std::min)((std::max)(val, 0.0), 1.0) * 255.0;
        const double q = v * (1.0 - f), t = v * f;
        const double r[6] = {v, q, 0, 0, t, v}, g[6] = {t, v, v, q, 0, 0}, bl[6] = {0, 0, t, v, v, q};
        px[0] = uint8_t(r[si]);
        px[1] = uint8_t(g[si]);
        px[2] = uint8_t(bl[si]);
    }
    template <class Fn>
    void Write(const std::wstring& name, Fn rgbAt) const {
        if (base.empty()) return;
        std::vector<uint8_t> img(size_t(nx) * size_t(ny) * 4);
        for (int j = 0; j < ny; ++j) {
            const size_t src = size_t(ny - 1 - j) * size_t(nx);   // row 0 is SOUTH; a PNG's is the top
            for (int i = 0; i < nx; ++i) {
                uint8_t* px = &img[(size_t(j) * size_t(nx) + size_t(i)) * 4];
                px[3] = 255;
                if ((*h)[src + size_t(i)] <= 0.0) {
                    px[0] = px[1] = px[2] = 40;
                    continue;
                }
                rgbAt(src + size_t(i), px);
            }
        }
        SavePng(base + L"_" + name + L".png", img.data(), uint32_t(nx), uint32_t(ny), uint32_t(nx) * 4,
                img.size());
    }
};

// `phiOut`, when given: each component's EIKONAL phase, the real number before it is packed as a spinor
// (nComp planes of nx*ny, component-major) -- the field the blend between tide buckets interpolates.
void Solve(const Inputs& in, std::vector<uint8_t>& atlas, WaveField::GpuTable& table,
           Memo* memo, std::vector<float>* phiOut = nullptr) {
    const auto t0 = std::chrono::steady_clock::now();
    const WaveFieldConfig& cfg = in.cfg;
    const int nx = cfg.nx, ny = cfg.ny;
    const int nc = (std::min)(cfg.nComp, WaveField::kMaxComp);
    const size_t cells = size_t(nx) * size_t(ny);
    const int rows = (nc + 2) / 2;                    // nc comp slices + the env slice, 2 wide
    const uint32_t aw = uint32_t(2 * nx), ah = uint32_t(rows * ny);
    atlas.assign(size_t(aw) * size_t(ah) * 4, 0);
    table = {};
    table.orgX = float(cfg.orgX);
    table.orgZ = float(cfg.orgZ);
    table.invCell = float(1.0 / cfg.cellM);
    table.feather = cfg.featherM;   // scene-config driven (data/wave_scene.json)
    table.nx = uint32_t(nx);
    table.ny = uint32_t(ny);
    table.nUsed = uint32_t(nc);       // fixed layout: every comp slice allocated regardless
    table.envSlice = uint32_t(nc);
    table.excMax = float(kExcessMax);
    table.level = float(in.level);

    if (in.hs <= 0.0) {
        // No partition worth a solve: an all-zero field is still a VALID, cacheable answer
        // (aMax all 0 -- the kernel reads flat calm, the provenance stays honest).
        table.envMax = table.sumMax = 1e-3f;
        Log("[wave] solve: flat sea (no partition above %.2f m) -- zero field", kPartHsFloor);
        return;
    }

    // -- THE DIRECTIONAL SPECTRUM IS TWO-DIMENSIONAL, and sampling it along a DIAGONAL is
    //    what made the corduroy. -----------------------------------------------------------
    //
    // The fan paired one direction with one frequency, dirs[i] against fr[i], 1:1. That is a
    // sum of nc LONG-CRESTED trains: each component is a single infinite straight ridge, and
    // thirty-two of them read as thirty-two straight ridges plus their beat envelopes -- the
    // parallel ribbons, visible in the GEOMETRY, worst where the solved field is strongest.
    // Jittering the directions (the golden sequence below, kept) stops them forming a regular
    // lattice; it cannot stop each one being long-crested. M9bl's 16 -> 32 halved the angular
    // step and "pushed the superposition's repeat out of frame", which is a mitigation and
    // says so.
    //
    // A real sea is short-crested because EVERY frequency carries a spread of directions. So
    // the same 32 components are now 8 frequencies x 4 directions. Same count, same atlas
    // layout, same cbuffer packing -- only which point of S(f, theta) each component samples.
    //
    // The four directions of a frequency are STRATIFIED (one per quarter of the fan) and the
    // offset inside each quarter comes from the golden sequence on the GLOBAL index, so no
    // two frequencies share a direction set and nothing lines up across the ladder. The
    // directional weight is cos^2 in ENERGY over the declared half-width -- zero at the fan's
    // edge, where the uniform version was still at full strength -- so it is cos in AMPLITUDE,
    // which is what wts carries. The final renormalisation to sum w^2 = 1 is unchanged, so Hs
    // is preserved by construction and this moves energy in angle without creating any.
    const double fp = 1.0 / in.tp;
    double fr[WaveField::kMaxComp];
    double wts[WaveField::kMaxComp], dirs[WaveField::kMaxComp];
    const int nd = (nc >= kDirFan * 2) ? kDirFan : 1;   // directions per frequency
    const int nf = nc / nd;                             // frequencies in the ladder
    {
        // NO TWO COMPONENTS MAY SHARE A FREQUENCY. This is the whole lesson of the first
        // attempt: giving a frequency stratum four DIRECTIONS at ONE frequency replaced the
        // corduroy with something worse -- a brick lattice of dashes on the solver grid.
        // Trains of equal |k| at different angles interfere into a STANDING pattern; its
        // nodes and antinodes do not move, because there is no frequency difference for them
        // to beat at. Different frequencies drift past each other and smear; identical ones
        // lock. Bisected to it directly: ONE component alone renders perfectly clean, all
        // thirty-two together render the lattice.
        //
        // So the stratification is two-dimensional and every cell of it is distinct. Stratum
        // i owns a log-f band and stratum-column j owns a quarter of the direction fan; the
        // component at (i, j) takes its frequency from INSIDE stratum i and its direction
        // from inside quarter j, both placed by the golden sequence on the GLOBAL index. The
        // result is 32 distinct frequencies AND 4 directions per frequency band -- the
        // frequency resolution the old 32x1 ladder had, and the directional spread it never
        // had, in the same 32 slots.
        const double la = std::log10(kFreqLo * fp), lb = std::log10(kFreqHi * fp);
        const double step = (lb - la) / double(nf);
        // THE FAN'S HALF-WIDTH IS NOT spreadDeg, it is whatever makes cos^2 have the SAME
        // SECOND MOMENT spreadDeg has always meant. The old fan weighted +-spreadDeg
        // UNIFORMLY, whose directional standard deviation is spreadDeg/sqrt(3). Dropping a
        // cos^2 taper on the same half-width silently narrowed that to spreadDeg*0.3615 --
        // a 38% cut in effective spread, which is a 38% INCREASE in crest length, and the
        // sea came out visibly smeared into long horizontal streaks. The taper is right (a
        // real directional spectrum goes to zero at its edge; the uniform one did not), so
        // widen the fan until the moment matches: sqrt((1/3) / (1/3 - 2/pi^2)) = 1.5971.
        // spreadDeg then keeps meaning exactly what it meant before this file changed.
        constexpr double kCos2Widen = 1.5970512;
        const double halfW = (std::max)(cfg.spreadDeg, 1e-6) * kCos2Widen;
        for (int i = 0; i < nf; ++i) {
            // The stratum's own linear width in f, shared by its nd components.
            const double fLo = std::pow(10.0, la + double(i) * step);
            const double fHi = std::pow(10.0, la + double(i + 1) * step);
            const double dfc = (fHi - fLo) / double(nd);
            for (int j = 0; j < nd; ++j) {
                const int ic = i * nd + j;
                const double g = std::fmod(double(ic + 1) * kGolden, 1.0);
                const double h = std::fmod(double(ic + 1) * kGolden2, 1.0);
                // frequency: inside stratum i, at its own place -- never shared
                const double f = std::pow(10.0, la + (double(i) + (double(j) + h) /
                                                                      double(nd)) * step);
                // direction: inside quarter j of the fan, at its own place
                const double u = (double(j) + g) / double(nd);
                const double off = halfW * (2.0 * u - 1.0);
                const double S = std::pow(f, -5.0) * std::exp(-1.25 * std::pow(fp / f, 4.0));
                const double cw = std::cos(0.5 * kPiW * off / halfW);
                fr[ic] = f;
                dirs[ic] = in.mwdFromDeg + off;
                wts[ic] = std::sqrt((std::max)(S * dfc, 0.0)) * ((cw > 0.0) ? cw : 0.0);
            }
        }
        for (int i = nf * nd; i < nc; ++i) {   // nc not a multiple of nd: the tail is silent
            fr[i] = kFreqHi * fp;
            dirs[i] = in.mwdFromDeg;
            wts[i] = 0.0;
        }
        double norm = 0.0;
        for (int i = 0; i < nc; ++i) norm += wts[i] * wts[i];
        norm = std::sqrt((std::max)(norm, 1e-30));
        for (int i = 0; i < nc; ++i) wts[i] /= norm;
    }
    Log("[wave] fan: %d frequencies x %d directions = %d comps over +-%.1f deg "
        "(cos^2 in energy, stratified + golden jitter per frequency)",
        nf, nd, nc, cfg.spreadDeg);
    const double aTot = in.hs / 4.0 * std::sqrt(2.0);   // Hs = 4 sigma_eta = 2 sqrt2 rms

    // -- depth from the bucketed level (dry when the water never reaches the bed) ----------
    std::vector<double> h(cells);
    for (size_t idx = 0; idx < cells; ++idx) {
        h[idx] = (std::max)(in.level - double(in.bed[idx]), 0.0);
    }
    const double kHold = BracketGrid()[kBracketN - 1] * 0.25;
    LayerSink layers;
    if (!cfg.mapPath.empty()) {
        layers.base = cfg.mapPath;
        const size_t dot = layers.base.find_last_of(L'.');
        if (dot != std::wstring::npos) layers.base.resize(dot);
        layers.nx = nx;
        layers.ny = ny;
        layers.h = &h;
        layers.Write(L"0_depth", [&](size_t idx, uint8_t* px) {   // 0 .. 40 m, log
            LayerSink::Seq(std::log1p(h[idx]) / std::log1p(40.0), px);
        });
        double sMax = 0.05;
        for (size_t idx = 0; idx < cells && in.u.size() == cells; ++idx) {
            sMax = (std::max)(sMax, std::hypot(double(in.u[idx]), double(in.v[idx])));
        }
        layers.Write(L"1_current", [&](size_t idx, uint8_t* px) {   // hue = toward, value = speed / its max
            const double ue = (in.u.size() == cells) ? double(in.u[idx]) : 0.0;
            const double vn = (in.v.size() == cells) ? double(in.v[idx]) : 0.0;
            LayerSink::Hue(std::atan2(vn, ue), std::hypot(ue, vn) / sMax, px);
        });
        Log("[wave] layers: depth 0..40 m (log); current: hue the direction it flows toward, value its speed "
            "over the window's fastest, %.2f m/s", sMax);
    }
    // The layers' components: the most energetic, and (kept as it is found) the one carried hottest.
    int peakIc = 0;
    for (int i = 1; i < nc; ++i) {
        if (wts[i] > wts[peakIc]) peakIc = i;
    }
    int hotIc = -1, lastIc = 0;
    for (int i = 0; i < nc; ++i) {
        if (wts[i] > 0.0) lastIc = i;
    }
    std::vector<float> hotPhi, hotFlux, hotGain;

    // -- per-component solve ---------------------------------------------------------------
    // F25: the k planes are the memo's (or a solve-local one when no memo was given). The memo
    // stands for a component when its grid, frequency and direction are what they were; a cell
    // in a standing component keeps its k when its depth and flow are what they were.
    Memo local;
    Memo& M = memo ? *memo : local;
    const bool memoGrid = (M.nx == nx && M.ny == ny && M.nc == nc && M.k.size() == size_t(nc) * cells &&
                           M.h.size() == cells && M.u.size() == cells && M.v.size() == cells &&
                           in.u.size() == cells && in.v.size() == cells);
    if (!memoGrid) {
        M.nx = nx; M.ny = ny; M.nc = nc;
        M.k.assign(size_t(nc) * cells, 0.0);
        M.blocked.assign(size_t(nc) * ((cells + 7) / 8), 0);
        for (int c = 0; c < WaveField::kMaxComp; ++c) M.sig[c] = M.d0[c] = M.d1[c] = 0.0;
    }
    std::atomic<size_t> cellsSolved{0}, cellsKept{0};
    std::vector<float> aRaw(size_t(nc) * cells);
    std::vector<double> sumsq(cells, 0.0), sumA(cells, 0.0);
    std::vector<double> sumsqk(cells, 0.0);   // F7b: sum a^2 k, for the energy-weighted wavenumber
    std::vector<double> eta(cells, 0.0);   // --wave-map only: sum a cos(phi) at t = 0
    std::atomic<long long> blockedCells{0};
    bool uploaded[WaveField::kMaxComp] = {};
    int nUp = 0;

    for (int ic = 0; ic < nc; ++ic) {
        const double T = in.tp * (fp / fr[ic]);
        const double sig = 2.0 * kPiW / T;
        const double prop = Wrap360(dirs[ic] + 180.0) * (kPiW / 180.0);   // travel TOWARD
        const double d0 = std::sin(prop), d1 = std::cos(prop);            // (east, north)
        // W1 (2026-10-24): this pass is the dispersion alone. The amplitude is the action carried
        // along the eikonal's own rays (ActionTransport), so it is made after the phase, below;
        // the Snell factor against one bar normal that stood here is gone with the normal.
        float* aPlane = aRaw.data() + size_t(ic) * cells;
        double* kbuf = M.k.data() + size_t(ic) * cells;
        uint8_t* kblk = M.blocked.data() + size_t(ic) * ((cells + 7) / 8);
        // F25: the component stands in the memo when it asks what it asked last time
        const bool compStands = memoGrid && M.sig[ic] == sig && M.d0[ic] == d0 && M.d1[ic] == d1;

        ParallelRows(ny, [&](int j0, int j1) {
            long long nb = 0;
            size_t ns = 0, nk = 0;
            for (int j = j0; j < j1; ++j) {
                const size_t row = size_t(j) * size_t(nx);
                for (int i = 0; i < nx; ++i) {
                    const size_t idx = row + size_t(i);
                    const double hc = h[idx];
                    if (hc <= 0.0) {   // dry: engine rule -- hold k, zero a, skip the solve
                        kbuf[idx] = kHold;
                        aPlane[idx] = 0.0f;
                        continue;
                    }
                    const double hf = (hc > kDepthFloor) ? hc : kDepthFloor;
                    const double along = double(in.u[idx]) * d0 + double(in.v[idx]) * d1;
                    const double uopp = (along < 0.0) ? -along : 0.0;   // only opposing shortens
                    bool blocked = false;
                    double k;
                    if (compStands && h[idx] == M.h[idx] && in.u[idx] == M.u[idx] &&
                        in.v[idx] == M.v[idx]) {   // F25: the same question, the memo's answer
                        k = kbuf[idx];
                        blocked = (kblk[idx >> 3] >> (idx & 7)) & 1u;
                        ++nk;
                    } else {
                        k = SolveDispersion(sig, hf, uopp, blocked);
                        ++ns;
                        // rows are whole bytes (nx a multiple of 8): no two threads share one
                        if (blocked) kblk[idx >> 3] |= uint8_t(1u << (idx & 7));
                        else kblk[idx >> 3] &= uint8_t(~(1u << (idx & 7)));
                    }
                    if (blocked) ++nb;
                    kbuf[idx] = k;
                }
            }
            blockedCells += nb;
            cellsSolved += ns;
            cellsKept += nk;
        });
        M.sig[ic] = sig;
        M.d0[ic] = d0;
        M.d1[ic] = d1;

        table.sigma[ic] = float(sig);
        table.dirX[ic] = float(d0);
        table.dirZ[ic] = float(d1);

        const double lambdaDeep = 2.0 * kPiW * PlanetGWave() / (sig * sig);
        if (lambdaDeep < cfg.minSamplesPerLambda * cfg.cellM) continue;   // the fold gate
        uploaded[ic] = true;
        ++nUp;
    }
    // the memo's planes are the ones just answered (compared against the old ones above)
    M.h = h;
    M.u = in.u;
    M.v = in.v;
    const double secsDisp = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();

    // F25: THE PHASE, EVERY COMPONENT ON ITS OWN THREAD. The eikonal is one component's and reads
    // nothing of another's (its k plane, its sweep buffers, its atlas slice, its kMax); the grid
    // was walked 32 times in a row on one thread (4.5 s). Under --wave-map the components run in
    // order on one thread so eta's sum is the sum it always was, to the bit.
    const int compGrain = cfg.mapPath.empty() ? 1 : nc;
    std::atomic<int> roundsMin{INT_MAX}, roundsMax{0};   // F28: until still, per component
    // W1: EVERY component with energy is phased and carried, uploaded or not -- the limiter and the
    // envelope sum the whole sea, and its amplitude is now made from its phase. Only an uploaded
    // component's k and phase are written to the atlas (a gated slice stays zero bytes).
    std::atomic<int> carryMin{INT_MAX}, carryMax{0};   // the action's rounds until still
    wavecore::CarryStats carry[WaveField::kMaxComp] = {};   // the instrument, per component
    Threads().ParallelFor(Lane::Long, "wave.comps", nc, compGrain, [&](int c0, int c1) {
    std::vector<double> phibuf(cells), feik(cells);
    std::vector<float> flux(2 * cells);   // the action's velocity, (east, north) per cell
    std::vector<uint32_t> order(cells);   // the march's order: the wet cells by phase
    std::vector<float> ray(3 * cells);    // the ray and C Cg (the diffraction)
    std::vector<float> gainBuf;           // --wave-map: the action's gain over the shoaled swell
    for (int ic = c0; ic < c1; ++ic) {
        if (!(wts[ic] > 0.0)) continue;
        const double T = in.tp * (fp / fr[ic]);
        const double sig = 2.0 * kPiW / T;
        const double prop = Wrap360(dirs[ic] + 180.0) * (kPiW / 180.0);   // travel TOWARD
        const double d0 = std::sin(prop), d1 = std::cos(prop);            // (east, north)
        const double* kbuf = M.k.data() + size_t(ic) * cells;
        float* aPlane = aRaw.data() + size_t(ic) * cells;

        // ---- M9bw: THE PHASE IS A FIELD, NOT A PATH. -----------------------------------
        //
        // phi used to be accumulated by WALKING the grid and adding k*(d^ . dl) as it went,
        // and every version of that walk printed the walk itself into the sea. The row-mean
        // gauge laid its inconsistency down along ROWS -- east-west banding whatever the wave
        // direction, which is why storm 90 and storm 190 rendered the same streaks. Replacing
        // it with an honest west-edge-then-eastward path integral moved the bias without
        // removing it, and that is the tell: THERE IS NO PATH THAT WORKS. The field k(x,y) d^,
        // with d^ held at the deep-water direction, has curl
        //
        //     d/dx (k d1) - d/dy (k d0)  =  d1 dk/dx - d0 dk/dy
        //
        // which vanishes only where grad k is parallel to d^. Over a bar, up a channel, along
        // a jetty it is not, so the line integral genuinely depends on the route and phi is
        // not a function of position at all. The disagreement between two routes is logged
        // below in radians; it is whole wavelengths.
        //
        // The curl is not an error to be averaged away. It is SNELL'S LAW asking to be obeyed.
        // A wave whose |k| changes with depth must also TURN, and those are the same
        // statement: phi is single-valued, so grad phi is curl-free BY CONSTRUCTION, and
        // |grad phi| = k is the eikonal equation. Holding the direction fixed while letting
        // the magnitude shoal is exactly the inconsistency the walk kept trying to smear out.
        // The amplitude already knew: Kr above is built from the REFRACTED angle, so the
        // shoaling gain has been assuming a bend the phase refused to make.
        //
        // So phi is SOLVED, not integrated: Godunov upwind fast sweeping for |grad phi| = k,
        // seeded on the two INFLOW edges (a 1-D path has no path-dependence, so the boundary
        // datum is exact) and swept in the four diagonal orders until a round moves no cell
        // (F28; it was "twice", and twice left the inlet a two-round iterate). The update's own
        // quadratic is (phi-a)^2 + (phi-b)^2 = (k dx)^2, which for a plane wave reads
        // (k d0 dx)^2 + (k d1 dx)^2 = (k dx)^2 -- an identity -- so the scheme is EXACT for a
        // plane wave on a uniform k at ANY angle. Deep water is therefore untouched to the
        // last bit, no direction of the grid is special, and every departure from the plane
        // wave is refraction the bathymetry actually asked for: rays bending toward the
        // shallows and crests swinging parallel to the contours inside the inlet.
        //
        // Dry cells carry the k the wet field reaches at its OWN depth floor rather than the
        // quantisation hold, so the speed field is continuous across the waterline and the
        // shoreline is not a phase barrier. Their amplitude is zero either way; this is only
        // so that a footprint straddling the beach interpolates a continuing wave.
        {
            bool blShore = false;
            const double kShore = SolveDispersion(sig, kDepthFloor, 0.0, blShore);
            const double dxm = cfg.cellM;
            const double kInf = 1e30;
            for (size_t idx = 0; idx < cells; ++idx) {
                feik[idx] = (h[idx] > 0.0) ? kbuf[idx] : kShore;
                phibuf[idx] = kInf;
            }
            // ---- THE BOUNDARY IS THE INCIDENT SWELL, AND IT ARRIVES FROM DEEP WATER. ----
            //
            // The first version of this seeded each inflow edge by INTEGRATING the local k
            // along it, and that put the path-dependence straight back -- into the boundary
            // instead of the interior. The two edges accumulate different amounts of shoaling
            // (the east edge is open ocean, the south edge runs the length of Plum Island's
            // shallows), the fronts they launch disagree by tens of radians, and first arrival
            // resolves the disagreement as a SHOCK: a dead straight diagonal seam across the
            // ebb shoal, wave structure smooth on one side of it and unrelated on the other.
            // It was plainly visible in the very first --wave-map, and it is the bias that
            // survived the switch from the row-mean gauge.
            //
            // A boundary condition is not something to integrate; it is something to STATE.
            // The incident wave is a swell, and a swell is defined in DEEP water: a plane wave
            // of wavenumber k_inf = sigma^2/g travelling along d^. So that is what the two
            // inflow edges carry, both from the same expression, which makes them consistent
            // by construction -- one plane, not two integrals that have to be reconciled.
            //
            // This is not an approximation traded for tidiness, it is Snell's law stated at
            // the right place. Prescribing k_inf*d^ on the boundary fixes the TANGENTIAL
            // wavenumber at k_inf*sin(theta0) -- the invariant -- and the eikonal then takes
            // the normal component from the LOCAL k, which is precisely refraction from deep
            // water onto the window's edge. The window's east edge sits in ~18 m, where k is
            // about a third above deep; the interior k is unaffected (it is solved, not
            // seeded), and the incident direction it implies differs from the deep bearing by
            // roughly a degree -- which is the bend the wave really made getting here.
            //
            // Re-applied after every sweep so it stays a boundary condition rather than an
            // initial guess the sweep is free to undercut.
            const int rounds = EikonalPhase(nx, ny, dxm, (sig * sig) / PlanetGWave(), d0, d1, h, feik, phibuf);
            for (int cur = roundsMin.load(); rounds < cur && !roundsMin.compare_exchange_weak(cur, rounds);) {}
            for (int cur = roundsMax.load(); rounds > cur && !roundsMax.compare_exchange_weak(cur, rounds);) {}
        }

        // ---- W1: THE AMPLITUDE IS THE ACTION, CARRIED ALONG THE PHASE (ActionTransport). ----
        // feik is spent with the sweep; it holds the action density from here.
        {
            if (layers.base.size() && gainBuf.size() != cells) gainBuf.assign(cells, 0.0f);
            const int rounds = ActionTransport(nx, ny, cfg.cellM, sig, d0, d1, aTot * wts[ic], h, kbuf,
                                               phibuf, in.u, in.v, flux, feik, order, ray, aPlane,
                                               M.blocked.data() + size_t(ic) * ((cells + 7) / 8), &carry[ic],
                                               layers.base.size() ? gainBuf.data() : nullptr);
            for (int cur = carryMin.load(); rounds < cur && !carryMin.compare_exchange_weak(cur, rounds);) {}
            for (int cur = carryMax.load(); rounds > cur && !carryMax.compare_exchange_weak(cur, rounds);) {}
        }
        if (!cfg.mapPath.empty()) {   // --wave-map: components in order, so eta's sum is one thread's
            for (size_t idx = 0; idx < cells; ++idx) eta[idx] += double(aPlane[idx]) * std::cos(phibuf[idx]);
            const double a0c = aTot * wts[ic], kInfC = (sig * sig) / PlanetGWave();
            auto writeComp = [&](int c, const float* ph, const float* fx, const float* gn, const float* am) {
                const std::wstring p = L"c" + std::to_wstring(c) + L"_";
                const double* kc = M.k.data() + size_t(c) * cells;
                const double kI = (std::pow(2.0 * kPiW / (in.tp * (fp / fr[c])), 2.0)) / PlanetGWave();
                layers.Write(p + L"2_k", [&](size_t i, uint8_t* px) {   // k / k_deep: 1 .. 4
                    LayerSink::Seq((kc[i] / kI - 1.0) / 3.0, px);
                });
                layers.Write(p + L"3_phase", [&](size_t i, uint8_t* px) { LayerSink::Div(std::cos(double(ph[i])), px); });
                layers.Write(p + L"4_ray", [&](size_t i, uint8_t* px) {   // the action's direction (hue)
                    LayerSink::Hue(std::atan2(double(fx[2 * i + 1]), double(fx[2 * i])), 1.0, px);
                });
                layers.Write(p + L"5_crease", [&](size_t i, uint8_t* px) {   // both neighbours lower: a crease
                    const int ci = int(i % size_t(nx)), cj = int(i / size_t(nx));
                    auto ridge = [&](int di, int dj) {
                        const int ai = ci - di, aj = cj - dj, bi = ci + di, bj = cj + dj;
                        if (ai < 0 || aj < 0 || bi >= nx || bj >= ny) return false;
                        return ph[size_t(aj) * nx + ai] < ph[i] && ph[size_t(bj) * nx + bi] < ph[i];
                    };
                    const bool r = ridge(1, 0) || ridge(0, 1);
                    px[0] = r ? 200 : 235;
                    px[1] = r ? 30 : 235;
                    px[2] = r ? 30 : 235;
                });
                layers.Write(p + L"6_gain", [&](size_t i, uint8_t* px) {   // log2(A / shoaled swell): +-3 = 8x
                    LayerSink::Div((gn[i] > 0.0f) ? std::log2(double(gn[i])) / 3.0 : -1.0, px);
                });
                layers.Write(p + L"7_amp", [&](size_t i, uint8_t* px) {   // A / (2 a0)
                    LayerSink::Seq(double(am[i]) / (2.0 * aTot * wts[c]), px);
                });
                Log("[wave] layers: component %d (T %.1f s, from %.0f deg, a0 %.3f m): k/k_deep 1..4, cos phase, the "
                    "ray's direction, creases, log2 of the action's gain over the shoaled swell (+-3 = 8x), A/(2 a0); "
                    "its gain p50 %.2f p99 %.2f max %.2f at (%d, %d)",
                    c, in.tp * (fp / fr[c]), Wrap360(dirs[c]), aTot * wts[c], carry[c].p50, carry[c].p99, carry[c].max,
                    carry[c].iMax, carry[c].jMax);
            };
            (void)a0c;
            (void)kInfC;
            std::vector<float> phf(cells);
            for (size_t i = 0; i < cells; ++i) phf[i] = float(phibuf[i]);
            if (ic == peakIc) writeComp(ic, phf.data(), flux.data(), gainBuf.data(), aPlane);
            if (ic != peakIc && (hotIc < 0 || carry[ic].max > carry[hotIc].max)) {
                hotIc = ic;
                hotPhi = phf;
                hotFlux = flux;
                hotGain = gainBuf;
            }
            if (ic == lastIc && hotIc >= 0) {
                writeComp(hotIc, hotPhi.data(), hotFlux.data(), hotGain.data(), aRaw.data() + size_t(hotIc) * cells);
            }
        }
        if (!uploaded[ic]) continue;   // the fold gate: its slice stays zero bytes

        double kFieldMax = 0.0;
        for (size_t idx = 0; idx < cells; ++idx) kFieldMax = (std::max)(kFieldMax, kbuf[idx]);
        const double kMax = Ceil3(1.05 * kFieldMax);
        table.kMax[ic] = float(kMax);

        const int sx = (ic & 1) * nx, sy = (ic >> 1) * ny;
        {
            for (int j = 0; j < ny; ++j) {
                const size_t arow = (size_t(sy + j) * aw + size_t(sx)) * 4;
                const size_t row = size_t(j) * size_t(nx);
                for (int i = 0; i < nx; ++i) {
                    uint8_t* px = &atlas[arow + size_t(i) * 4];
                    px[1] = Quant8(kbuf[row + i], kMax);
                    const double ph = phibuf[row + i];
                    if (phiOut) (*phiOut)[size_t(ic) * cells + row + i] = float(ph);
                    px[2] = Quant8(std::cos(ph) * 0.5 + 0.5, 1.0);
                    px[3] = Quant8(std::sin(ph) * 0.5 + 0.5, 1.0);
                }
            }
        }
    }
    });
    // The sea's sums over every component, once the amplitudes exist (they were summed in the
    // dispersion pass while the amplitude was made there): rows in parallel, components in order.
    ParallelRows(ny, [&](int j0, int j1) {
        for (size_t idx = size_t(j0) * size_t(nx); idx < size_t(j1) * size_t(nx); ++idx) {
            double s2 = 0.0, s2k = 0.0, s1 = 0.0;
            for (int ic = 0; ic < nc; ++ic) {
                const double a = double(aRaw[size_t(ic) * cells + idx]);
                s2 += a * a;
                s2k += a * a * M.k[size_t(ic) * cells + idx];
                s1 += a;
            }
            sumsq[idx] = s2;
            sumsqk[idx] = s2k;
            sumA[idx] = s1;
        }
    });
    const double secsEik = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count() - secsDisp;
    {   // W1's instrument: what the transport did against the swell shoaled with no refraction
        int worst = -1;
        double p50 = 0.0, over2 = 0.0;
        int n = 0;
        for (int ic = 0; ic < nc; ++ic) {
            if (!(wts[ic] > 0.0)) continue;
            p50 += carry[ic].p50;
            over2 = (std::max)(over2, carry[ic].over2);
            ++n;
            if (worst < 0 || carry[ic].max > carry[worst].max) worst = ic;
        }
        if (worst >= 0) {
            Log("[wave] action vs the swell shoaled without refraction: mean p50 %.3f; worst component %d "
                "(T %.1f s, from %.0f deg): p99 %.2f max %.2f at cell (%d, %d); cells over 2x at most %.2f%%",
                n ? p50 / n : 0.0, worst, in.tp * (fp / fr[worst]), Wrap360(dirs[worst]), carry[worst].p99,
                carry[worst].max, carry[worst].iMax, carry[worst].jMax, 100.0 * over2);
        }
    }

    // -- the TOTAL-Hs limiter: Hs = 2 sqrt2 rms against gammaHs * h, ONE uniform factor per cell
    //    on every a_i (spectral shape and directions survive); excess > 1 is the breaking
    //    indicator -- "does the sea here want to be taller than the water allows". F7: the
    //    factor is the clipped Rayleigh sea's (WaterTerms.h BreakFactor), not min(1, 1/excess):
    //    the rms tends to the limit from below and the law has no corner at excess 1.
    std::vector<double> lim(cells);   // (and eta below, which carries the same factor)
    double rmsFieldMax = 0.0, sumFieldMax = 0.0;
    for (size_t idx = 0; idx < cells; ++idx) {
        const double rmsRaw = std::sqrt(sumsq[idx]);
        // F7b: the height the sea can stand to at this cell, by its energy-weighted wavenumber.
        const double kEff = sumsqk[idx] / ((sumsq[idx] > 1e-12) ? sumsq[idx] : 1e-12);
        const double rmsLim = BreakHmax(kEff, h[idx]) / (2.0 * std::sqrt(2.0));
        const double exc = rmsRaw / ((rmsLim > 1e-6) ? rmsLim : 1e-6);
        const double L = BreakFactor(exc);
        lim[idx] = L;
        rmsFieldMax = (std::max)(rmsFieldMax, rmsRaw * L);
        sumFieldMax = (std::max)(sumFieldMax, sumA[idx] * L);
    }
    const double envMax = Ceil3(1.05 * rmsFieldMax);
    if (layers.base.size()) {
        const double rmsDeep = aTot / std::sqrt(2.0);   // the incident sea's rms
        layers.Write(L"8_rms_raw", [&](size_t i, uint8_t* px) { LayerSink::Seq(std::sqrt(sumsq[i]) / (2.0 * rmsDeep), px); });
        layers.Write(L"9_excess", [&](size_t i, uint8_t* px) {
            const double kEff = sumsqk[i] / ((sumsq[i] > 1e-12) ? sumsq[i] : 1e-12);
            const double rmsLim = BreakHmax(kEff, h[i]) / (2.0 * std::sqrt(2.0));
            LayerSink::Seq(std::sqrt(sumsq[i]) / ((rmsLim > 1e-6) ? rmsLim : 1e-6) / 2.5, px);
        });
        layers.Write(L"10_limiter", [&](size_t i, uint8_t* px) { LayerSink::Seq(lim[i], px); });
        layers.Write(L"11_hs", [&](size_t i, uint8_t* px) {
            LayerSink::Seq(2.0 * std::sqrt(2.0) * std::sqrt(sumsq[i]) * lim[i] / (2.0 * in.hs), px);
        });
        Log("[wave] layers: the sea -- rms before the limiter / (2 x the incident rms %.3f m), excess / 2.5, the "
            "limiter's factor 0..1, Hs after it / (2 x the incident Hs %.2f m); the map (eta) is the last layer",
            rmsDeep, in.hs);
    }
    const double sumMax = Ceil3(1.05 * sumFieldMax);
    table.envMax = float(envMax);
    table.sumMax = float(sumMax);

    {   // env slice: (rms/envMax, excess/2.5, sum/sumMax, sqrt(tan beta))
        // The bed's steepest slope at the cell (the surf-similarity number's tan beta, Jet.hlsli
        // PlungeShare), by central differences of the BED over +-4 cells -- the slope of the face a
        // breaking crest runs up (land included: the beach), not of a single rock -- stored as its square
        // root so beach slopes of 0.01 to 0.1 keep their resolution in a byte (1 at 45 degrees).
        const int sGrad = 4;
        const int es = nc;
        const int sx = (es & 1) * nx, sy = (es >> 1) * ny;
        for (int j = 0; j < ny; ++j) {
            const size_t arow = (size_t(sy + j) * aw + size_t(sx)) * 4;
            const size_t row = size_t(j) * size_t(nx);
            for (int i = 0; i < nx; ++i) {
                const size_t idx = row + size_t(i);
                const double rmsRaw = std::sqrt(sumsq[idx]);
                const double kEff = sumsqk[idx] / ((sumsq[idx] > 1e-12) ? sumsq[idx] : 1e-12);
                const double rmsLim = BreakHmax(kEff, h[idx]) / (2.0 * std::sqrt(2.0));
                const double exc = rmsRaw / ((rmsLim > 1e-6) ? rmsLim : 1e-6);
                uint8_t* px = &atlas[arow + size_t(i) * 4];
                px[0] = Quant8(rmsRaw * lim[idx], envMax);
                px[1] = Quant8(exc, kExcessMax);
                px[2] = Quant8(sumA[idx] * lim[idx], sumMax);
                const int il = (std::max)(i - sGrad, 0), ir = (std::min)(i + sGrad, nx - 1);
                const int jl = (std::max)(j - sGrad, 0), jr = (std::min)(j + sGrad, ny - 1);
                const double gx = (double(in.bed[row + size_t(ir)]) - double(in.bed[row + size_t(il)])) /
                                  (double((std::max)(ir - il, 1)) * cfg.cellM);
                const double gz = (double(in.bed[size_t(jr) * size_t(nx) + size_t(i)]) -
                                   double(in.bed[size_t(jl) * size_t(nx) + size_t(i)])) /
                                  (double((std::max)(jr - jl, 1)) * cfg.cellM);
                px[3] = Quant8(std::sqrt((std::min)(std::sqrt(gx * gx + gz * gz), 1.0)), 1.0);
            }
        }
    }

    for (int ic = 0; ic < nc; ++ic) {
        if (!uploaded[ic]) continue;
        const float* aPlane = aRaw.data() + size_t(ic) * cells;
        double aFieldMax = 0.0;
        for (size_t idx = 0; idx < cells; ++idx) {
            aFieldMax = (std::max)(aFieldMax, double(aPlane[idx]) * lim[idx]);
        }
        const double aMax = Ceil3(1.05 * aFieldMax);
        table.aMax[ic] = float(aMax);
        const int sx = (ic & 1) * nx, sy = (ic >> 1) * ny;
        for (int j = 0; j < ny; ++j) {
            const size_t arow = (size_t(sy + j) * aw + size_t(sx)) * 4;
            const size_t row = size_t(j) * size_t(nx);
            for (int i = 0; i < nx; ++i) {
                atlas[arow + size_t(i) * 4] =
                    Quant8(double(aPlane[row + i]) * lim[row + i], aMax);
            }
        }
    }

    // ---- --wave-map: the field with nothing between it and the eye. -------------------
    // Every earlier argument about banding was conducted through a renderer, and a renderer
    // can invent structure (shading rate, mip choice, mesh spacing) and can hide it (a fold
    // that sheds a band into slope variance draws a mirror). This writes what the solver
    // actually produced: eta = sum a cos(phi) at t = 0, on the solve's own cells, one texel
    // per cell, with land drawn dark so the coastline gives the picture its orientation.
    // Blue is a trough, red a crest; a wave train from the east must show crests running
    // north-south offshore, and must swing parallel to the contours as it shoals.
    if (!cfg.mapPath.empty()) {
        double eMax = 1e-6;
        for (size_t idx = 0; idx < cells; ++idx) eMax = (std::max)(eMax, std::abs(eta[idx]));
        std::vector<uint8_t> img(cells * 4);
        for (int j = 0; j < ny; ++j) {
            // row 0 of the solve is SOUTH; a PNG's row 0 is the TOP, so north goes up.
            const size_t src = size_t(ny - 1 - j) * size_t(nx);
            const size_t dst = size_t(j) * size_t(nx);
            for (int i = 0; i < nx; ++i) {
                uint8_t* px = &img[(dst + size_t(i)) * 4];
                px[3] = 255;
                if (h[src + size_t(i)] <= 0.0) { px[0] = px[1] = px[2] = 40; continue; }
                const double u = eta[src + size_t(i)] / eMax;   // -1 trough .. +1 crest
                const double p = (u > 0.0) ? u : 0.0, n = (u < 0.0) ? -u : 0.0;
                px[0] = uint8_t(255.0 * (1.0 - 0.90 * n));
                px[1] = uint8_t(255.0 * (1.0 - 0.85 * (p + n)));
                px[2] = uint8_t(255.0 * (1.0 - 0.90 * p));
            }
        }
        if (SavePng(cfg.mapPath, img.data(), uint32_t(nx), uint32_t(ny), uint32_t(nx) * 4,
                    img.size())) {
            Log("[wave] wave map: %dx%d cells @ %.2f m, eta range +-%.2f m (display exag not "
                "applied -- this is the cache's raw physics)",
                nx, ny, cfg.cellM, eMax);
        } else {
            Log("[wave] wave map FAILED to write");
        }
    }

    const double secs =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
    Log("[wave] solved %dx%dx%d in %.1f s: %d/%d comps uploaded (gate %.0f m lambda), "
        "blocked %.3f%%, lvl %+.2f",
        nx, ny, nc, secs, nUp, nc, cfg.minSamplesPerLambda * cfg.cellM,
        100.0 * double(blockedCells.load()) / (double(nc) * double(cells)), in.level);
    Log("[wave]   the dispersion %.1f s: %zu cells solved, %zu kept from the memo (F25); the phase and the "
        "action %.1f s, components in parallel, %d-%d and %d-%d rounds until still (F28, W1); the rest %.1f s",
        secsDisp, cellsSolved.load(), cellsKept.load(), secsEik, roundsMax.load() ? roundsMin.load() : 0,
        roundsMax.load(), carryMax.load() ? carryMin.load() : 0, carryMax.load(), secs - secsDisp - secsEik);
}

// `phi`: the eikonal phase planes, after the inputs, flagged in the head's pad0 (bit 0) -- a reader
// that stops at the inputs (the python checker) never sees them.
bool WriteCache(const char* path, uint64_t key, const Inputs& in,
                const std::vector<uint8_t>& atlas, const WaveField::GpuTable& table,
                bool withInputs, const std::vector<float>* phi = nullptr) {
    CreateDirectoryA("cache", nullptr);
    CreateDirectoryA("cache\\wave", nullptr);
    FILE* f = fopen(path, "wb");
    if (!f) {
        Log("[wave] cache write failed to open %s", path);
        return false;
    }
    CacheHead hd;
    hd.magic = kCacheMagic;
    hd.version = kCacheVersion;
    hd.key = key;
    hd.tableBytes = uint32_t(sizeof(WaveField::GpuTable));
    hd.atlasW = table.nx * 2;
    hd.atlasH = uint32_t(((std::min)(in.cfg.nComp, WaveField::kMaxComp) + 2) / 2) * table.ny;
    hd.nx = table.nx;
    hd.ny = table.ny;
    hd.nComp = uint32_t((std::min)(in.cfg.nComp, WaveField::kMaxComp));
    hd.hasInputs = withInputs ? 1u : 0u;
    hd.pad0 = (phi && !phi->empty()) ? 1u : 0u;
    hd.hs = in.hs;
    hd.tp = in.tp;
    hd.mwdDeg = in.mwdFromDeg;
    hd.cellM = in.cfg.cellM;
    hd.spreadDeg = in.cfg.spreadDeg;
    hd.barNormalDeg = 0.0;   // W1: no bar normal; the slot keeps the head's layout
    hd.gammaHs = in.cfg.gammaHs;
    hd.minSamplesPerLambda = in.cfg.minSamplesPerLambda;
    hd.orgX = in.cfg.orgX;
    hd.orgZ = in.cfg.orgZ;
    hd.level = in.level;
    hd.currentMs = in.currentMs;
    bool ok = fwrite(&hd, sizeof(hd), 1, f) == 1;
    ok = ok && fwrite(&table, sizeof(table), 1, f) == 1;
    ok = ok && fwrite(atlas.data(), 1, atlas.size(), f) == atlas.size();
    if (withInputs) {
        const size_t cells = size_t(hd.nx) * size_t(hd.ny);
        ok = ok && in.bed.size() == cells && in.u.size() == cells && in.v.size() == cells;
        ok = ok && fwrite(in.bed.data(), sizeof(float), cells, f) == cells;
        ok = ok && fwrite(in.u.data(), sizeof(float), cells, f) == cells;
        ok = ok && fwrite(in.v.data(), sizeof(float), cells, f) == cells;
    }
    if (phi && !phi->empty()) ok = ok && fwrite(phi->data(), sizeof(float), phi->size(), f) == phi->size();
    fclose(f);
    if (ok) {
        Log("[wave] cached %s (%zu KB%s)", path, (sizeof(hd) + sizeof(table) + atlas.size()) >> 10,
            withInputs ? " + input planes" : "");
    } else {
        Log("[wave] cache write FAILED: %s", path);
        remove(path);
    }
    return ok;
}

bool ReadCache(const char* path, uint64_t key, const WaveFieldConfig& cfg,
               std::vector<uint8_t>& atlas, WaveField::GpuTable& table, std::vector<float>* phi = nullptr) {
    FILE* f = fopen(path, "rb");
    if (!f) return false;
    CacheHead hd;
    if (fread(&hd, sizeof(hd), 1, f) != 1) {
        fclose(f);
        return false;
    }
    const int nc = (std::min)(cfg.nComp, WaveField::kMaxComp);
    const uint32_t wantW = uint32_t(2 * cfg.nx);
    const uint32_t wantH = uint32_t(((nc + 2) / 2) * cfg.ny);
    // Every size is a CONTRACT: a stale or truncated file must lose quietly to a re-solve.
    if (hd.magic != kCacheMagic || hd.version != kCacheVersion || hd.key != key ||
        hd.tableBytes != sizeof(WaveField::GpuTable) || hd.nx != uint32_t(cfg.nx) ||
        hd.ny != uint32_t(cfg.ny) || hd.nComp != uint32_t(nc) || hd.atlasW != wantW ||
        hd.atlasH != wantH) {
        fclose(f);
        return false;
    }
    atlas.resize(size_t(wantW) * size_t(wantH) * 4);
    bool ok = fread(&table, sizeof(table), 1, f) == 1;
    ok = ok && fread(atlas.data(), 1, atlas.size(), f) == atlas.size();
    if (phi) {
        phi->clear();
        if (ok && (hd.pad0 & 1u)) {
            const size_t cells = size_t(hd.nx) * size_t(hd.ny);
            if (hd.hasInputs) ok = _fseeki64(f, int64_t(3 * cells * sizeof(float)), SEEK_CUR) == 0;
            phi->resize(size_t(nc) * cells);
            ok = ok && fread(phi->data(), sizeof(float), phi->size(), f) == phi->size();
            if (!ok) phi->clear();
        }
    }
    fclose(f);
    ok = ok && table.nx == hd.nx && table.ny == hd.ny;
    if (!ok) Log("[wave] cache %s failed validation -- re-solving", path);
    return ok;
}

}  // namespace wavecore

// ------------------------------------------------------------------------------------------------
//  WaveField -- the engine face: input gathering, buckets, cache, upload, probe.
// ------------------------------------------------------------------------------------------------

void WaveField::Configure(const WaveFieldConfig& cfg, const Compositor* comp, int hgtChannel,
                          const WaterAtlas* atlas, const TideModel* tides, int entranceStation,
                          const CurrentModel* currents, int actStation) {
    WaitForSolve();   // reconfigure never races a live solve
    m_inFlight.store(false);
    m_resultReady.store(false);
    m_cfg = cfg;
    m_comp = comp;
    m_hgtCh = hgtChannel;
    m_atlas = atlas;
    m_tides = tides;
    m_entranceSta = entranceStation;
    m_currents = currents;
    m_actSta = actStation;
    m_liveKey = 0;   // force the first Update to solve or hit cache
    // W2: THE FIELD'S CURRENT ON THE SOLVE'S CELLS, at each cell's place, in the quanta the solve eats.
    {
        auto f = std::make_shared<FieldFlow>();
        const size_t cells = size_t((std::max)(cfg.nx, 0)) * size_t((std::max)(cfg.ny, 0));
        f->u.assign(cells, 0.0f);
        f->v.assign(cells, 0.0f);
        if (currents && currents->FieldCount() && cells) {
            ParallelRows(cfg.ny, [&](int j0, int j1) {
                for (int j = j0; j < j1; ++j) {
                    for (int i = 0; i < cfg.nx; ++i) {
                        double la = 0.0, lo = 0.0;
                        cfg.PlaceOfCell(double(i) + 0.5, double(j) + 0.5, la, lo);
                        float u = 0.0f, v = 0.0f;
                        if (!currents->SampleAt(lo, la, u, v)) continue;
                        const size_t idx = size_t(j) * size_t(cfg.nx) + size_t(i);
                        f->u[idx] = std::round(u / 0.05f) * 0.05f;
                        f->v[idx] = std::round(v / 0.05f) * 0.05f;
                    }
                }
            });
        }
        uint64_t h = 14695981039346656037ull;
        auto mixB = [&h](const void* p, size_t n) {
            const uint8_t* bb = static_cast<const uint8_t*>(p);
            for (size_t i = 0; i < n; ++i) h = (h ^ bb[i]) * 1099511628211ull;
        };
        mixB("field flow", 10);
        mixB(f->u.data(), f->u.size() * sizeof(float));
        mixB(f->v.data(), f->v.size() * sizeof(float));
        f->sig = h;
        size_t moving = 0;
        float fastest = 0.0f;
        for (size_t idx = 0; idx < cells; ++idx) {
            const float sp = std::hypot(f->u[idx], f->v[idx]);
            if (sp > 0.0f) ++moving;
            fastest = (std::max)(fastest, sp);
        }
        Log("[wave] the field's current on the window: %zu of %zu cells moving, the fastest %.2f m/s "
            "(the solver's flow is composed over it where one stands)", moving, cells, fastest);
        m_field = f;
    }
    Log("[wave] configured: %dx%d @ %.1f m (%.0f x %.0f m window), %d comps, buckets "
        "tide %.2f m / current %.2f m/s",
        cfg.nx, cfg.ny, cfg.cellM, cfg.nx * cfg.cellM, cfg.ny * cfg.cellM, cfg.nComp,
        cfg.tideBucketM, cfg.currentBucketMs);
}

double WaveField::LevelRaw(double simUnix) const {
    double latC = 0.0, lonC = 0.0;
    m_cfg.PlaceOfCell(0.5 * m_cfg.nx, 0.5 * m_cfg.ny, latC, lonC);
    return m_atlas ? m_atlas->MslNavd(latC, lonC) + m_atlas->Level(latC, lonC, simUnix) : 0.0;
}

uint64_t WaveField::BucketKey(double simUnix, const PartParam* parts, int nParts, double levelAt) const {
    // FNV-1a over everything that touches the answer -- the compositor's identity-is-content
    // law: solver version, window geometry, the two buckets, the raw spectrum bytes, and the
    // height stack's signature (if the bed data changes shape, the field must change with it).
    uint64_t hsh = 14695981039346656037ull;
    auto mixBytes = [&hsh](const void* p, size_t n) {
        const uint8_t* b = static_cast<const uint8_t*>(p);
        for (size_t i = 0; i < n; ++i) hsh = (hsh ^ b[i]) * 1099511628211ull;
    };
    auto mixD = [&](double v) { mixBytes(&v, sizeof(v)); };
    auto mixU = [&](uint32_t v) { mixBytes(&v, sizeof(v)); };
    mixU(kSolverVersion);
    mixD(PlanetGWave());   // the planet's gravity: a field solved under one never answers for another
    mixD(m_cfg.orgX);
    mixD(m_cfg.orgZ);
    mixU(uint32_t(m_cfg.nx));
    mixU(uint32_t(m_cfg.ny));
    mixD(m_cfg.cellM);
    mixU(uint32_t(m_cfg.nComp));
    mixD(m_cfg.spreadDeg);
    mixD(m_cfg.gammaHs);
    mixD(m_cfg.minSamplesPerLambda);

    mixD(std::isnan(levelAt) ? std::round(LevelRaw(simUnix) / m_cfg.tideBucketM) * m_cfg.tideBucketM : levelAt);
    // W2: the flow the solve eats, by its content -- the solver's over the field, or the field's alone.
    // (The ACT station's speed bucket was in the key while it drove the proxy; it drives nothing now.)
    const uint64_t flowSig = (!m_curU.empty() && m_curSig != 0) ? m_curSig : (m_field ? m_field->sig : 0);
    mixBytes(&flowSig, sizeof(flowSig));

    if (parts && nParts > 0) mixBytes(parts, sizeof(PartParam) * size_t(nParts));
    if (m_comp && m_hgtCh >= 0) {
        for (const HeightSource* hs : m_comp->ChannelAt(m_hgtCh).height) {
            mixBytes(hs->Info().name.data(), hs->Info().name.size());
            mixBytes("|", 1);
            mixBytes(hs->Info().structure.data(), hs->Info().structure.size());
        }
        // the bed read through the height TREE when one is bound (compose/HeightTree.h): another field
        if (m_comp->HeightTreeOf(m_hgtCh)) mixBytes("height tree v2", 14);
    }
    return hsh;
}

// M8 FLOWS INTO WAVES: read the SWE's solved current back (main thread -- GPU readbacks
// never ride the worker), resample onto the solve grid with the row-0-north flip, scale
// by the prism-truncation gain, QUANTIZE to 0.05 m/s, and hash the quantized bytes into
// the bucket key. The solve then eats exactly the bytes the key describes -- content
// identity, so the same flow always finds its cache and a changed flow always re-solves.
void WaveField::ResampleFlow(CurJob& j, const SweSolver& swe, float gain, const WaveFieldConfig& cfg, Gpu& gpu) {
    const SweDomain& dom = swe.Domain();
    const size_t cells = size_t(cfg.nx) * size_t(cfg.ny);
    std::vector<float> eta;
    uint32_t ew = 0, eh = 0;
    SweSolver::UnpackRead(gpu, j.read, eta, ew, eh, j.uv4, j.uw, j.uh);
    if (j.uw < 2 || j.uh < 2) {   // nothing read: the last flow stands (and so does its signature)
        j.U = j.oldU;
        j.V = j.oldV;
        j.done.store(true, std::memory_order_release);
        return;
    }
    // W2: THE MANAGER'S TWO CURRENTS, PER CELL. The regional field everywhere (made at Configure), the
    // solver's solved current over it by the solver's domain weight, continuous across the domain's edge.
    // (WaterManager::SolverRefine still REPLACES the field with the solver's inside that one-cell band:
    // the hull's current is twinned with the bank's, which holds no field current. docs/INSERTS.md.)
    const bool haveField = j.field && j.field->u.size() == cells && j.field->v.size() == cells;
    j.U.assign(cells, 0.0f);
    j.V.assign(cells, 0.0f);
    // PHASE C4: each cell's place is its page texel's (WaveFieldConfig::PlaceOfCell), then
    // the solver's cell under it by the solver's own planes.
    for (int jj = 0; jj < cfg.ny; ++jj) {
        const size_t row = size_t(jj) * size_t(cfg.nx);
        for (int i = 0; i < cfg.nx; ++i) {
            const float fu = haveField ? j.field->u[row + i] : 0.0f;
            const float fv = haveField ? j.field->v[row + i] : 0.0f;
            j.U[row + i] = fu;
            j.V[row + i] = fv;
            double la = 0.0, lo = 0.0, tx = 0.0, ty = 0.0;
            cfg.PlaceOfCell(double(i) + 0.5, double(jj) + 0.5, la, lo);
            if (!dom.CellOf(la, lo, tx, ty) ||
                tx < 0.0 || ty < 0.0 || tx >= double(j.uw) || ty >= double(j.uh)) {
                continue;
            }
            const int ix = int(tx), iy = int(ty);   // row 0 = NORTH
            const float* sp = &j.uv4[(size_t(iy) * j.uw + size_t(ix)) * 4];
            if (sp[3] < 0.5f) continue;   // solver-invalid: sponge or dry -- the field's
            const float w = float(swe.DomainWeight(la, lo));
            // quantize to 0.05 m/s -- the sig hashes EXACTLY what the solve eats
            j.U[row + i] = std::round((fu + w * (sp[0] * gain - fu)) / 0.05f) * 0.05f;
            j.V[row + i] = std::round((fv + w * (sp[1] * gain - fv)) / 0.05f) * 0.05f;
        }
    }
    uint64_t h = 14695981039346656037ull;
    auto mixB = [&h](const void* p, size_t n) {
        const uint8_t* bb = static_cast<const uint8_t*>(p);
        for (size_t i = 0; i < n; ++i) h = (h ^ bb[i]) * 1099511628211ull;
    };
    mixB(j.U.data(), j.U.size() * sizeof(float));
    mixB(j.V.data(), j.V.size() * sizeof(float));
    j.h = h;
    // WHAT MOVED: the cells whose quantized flow differs from the last refresh's, and the
    // largest step -- the roll's cause, in the units the solve eats.
    if (j.oldU.size() == cells) {
        for (size_t i = 0; i < cells; ++i) {
            const float du = std::fabs(j.U[i] - j.oldU[i]);
            const float dv = std::fabs(j.V[i] - j.oldV[i]);
            if (du > 0.0f || dv > 0.0f) ++j.moved;
            j.worst = (std::max)(j.worst, (std::max)(du, dv));
        }
    }
    j.oldU.clear();
    j.oldV.clear();
    j.uv4.clear();
    j.done.store(true, std::memory_order_release);
}

void WaveField::RefreshSweCurrent(Gpu& gpu, double simUnix, bool block) {
    if (!m_swe || !m_swe->Ready()) return;
    // F13: ASKED ON ONE FRAME, RESAMPLED ON A WORKER, ADOPTED ON A LATER FRAME. The synchronous
    // read drained the queue, and the resample of 2.2 million cells after it took a quarter of a
    // second on the main thread: the one hitch the frame felt of the roll, every 90 s.
    if (m_curJob) {
        if (!m_curJob->done.load(std::memory_order_acquire)) return;
        if (m_inFlight.load()) return;   // never swap planes under a running solve
        CurJob& j = *m_curJob;
        m_curU.swap(j.U);
        m_curV.swap(j.V);
        m_flowKnown = true;   // F17: the first flow has landed; the key may form (Update)
        // F17: the LIVE planes are tested, not the job's. After the swap the job holds the planes
        // the last refresh ate, which are empty the first time; tested there, the first flow was
        // adopted but never signed, so the first solve ate the proxy jet and the key could not
        // roll until the second refresh, 90 s on -- the chaotic water at every start since F13.
        if (j.h != m_curSig && !m_curU.empty()) {
            m_curSig = j.h;
            Log("[wave] swe current refreshed: sig %016llx (gain %.1f, 0.05 m/s buckets): %zu of %zu "
                "cells moved since the last, the largest by %.2f m/s%s",
                static_cast<unsigned long long>(j.h), m_sweGain, j.moved, m_curU.size(), j.worst,
                j.first ? " (the first)" : "");
        }
        m_curJob.reset();
        return;
    }
    if (!m_swe->ReadFieldsPending()) {
        if (simUnix - m_curReadT < 90.0 && !m_curU.empty()) return;   // solve-cadence refresh
        if (m_inFlight.load()) return;
        if (!m_swe->ReadFieldsBegin(gpu)) return;
        if (!block) return;
        // F17: the blocking path eats the read here -- the copy was executed on the queue as it
        // was recorded, so its fence completes without the frame.
        while (!m_swe->ReadFieldsReady(gpu)) std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    if (!m_swe->ReadFieldsReady(gpu)) return;
    m_curReadT = simUnix;
    auto job = std::make_shared<CurJob>();
    job->read = m_swe->ReadFieldsDetach();
    job->oldU = m_curU;   // what the last refresh ate: the instrument's reference
    job->oldV = m_curV;
    job->first = m_curU.empty();
    job->h = m_curSig;    // nothing read keeps the last signature
    job->field = m_field;
    m_curJob = job;
    const SweSolver* swe = m_swe;
    const float gain = m_sweGain;
    const WaveFieldConfig cfg = m_cfg;
    Gpu* gp = &gpu;
    if (block) {   // F17: inline, then adopted at once -- the frame that asked sees the flow
        ResampleFlow(*job, *swe, gain, cfg, gpu);
        RefreshSweCurrent(gpu, simUnix, block);
        return;
    }
    Threads().Submit(Lane::Long, "wave.current", [job, swe, gain, cfg, gp]() {
        ResampleFlow(*job, *swe, gain, cfg, *gp);
    });
}

// The question a solve answers at an instant: the level and current buckets, the spectrum, the bed, the
// flow -- SolveNow's own inputs, made once (the blend's midpoint test asks them at three levels).
void WaveField::BuildInputs(double simUnix, const std::vector<PartParam>& parts, wavecore::Inputs& in,
                            double levelAt) const {
    in.cfg = m_cfg;
    const int nx = m_cfg.nx, ny = m_cfg.ny;
    const size_t cells = size_t(nx) * size_t(ny);

    // level at the WINDOW CENTER, bucketed -- the solve happens AT the bucket, so two frames
    // in the same bucket ask the identical question and the cache answers the second one.
    double latC = 0.0, lonC = 0.0;
    m_cfg.PlaceOfCell(0.5 * nx, 0.5 * ny, latC, lonC);
    // THE DATUM LINE (kept deliberately, once per solve): the still-water level this
    // field was solved at, decomposed into the two rungs that make it. Every depth in
    // the solve is bed-to-this, so a quiet bias here IS a wrong wave field -- and twice
    // it was: the boot-clock transient that solved at +1.50 before --start settled, and
    // the M8f MslNavd bias that charged the entrance +0.46 m of Riverside river slope.
    // Both were invisible in the render and obvious on this line.
    const double mslNavd = m_atlas ? m_atlas->MslNavd(latC, lonC) : 0.0;
    const double tideNavd = m_atlas ? m_atlas->Level(latC, lonC, simUnix) : 0.0;
    const double levelRaw = mslNavd + tideNavd;
    Log("[wave] solve datum at %.4f,%.4f t %.0f: MslNavd %+.3f + tide %+.3f = %+.3f m",
        latC, lonC, simUnix, mslNavd, tideNavd, levelRaw);
    in.level = std::isnan(levelAt) ? std::round(levelRaw / m_cfg.tideBucketM) * m_cfg.tideBucketM : levelAt;

    const double sRaw = (m_currents && m_actSta >= 0 &&
                         size_t(m_actSta) < m_currents->Count())
                            ? m_currents->SignedSpeed(size_t(m_actSta), simUnix)
                            : 0.0;
    in.currentMs = std::round(sRaw / m_cfg.currentBucketMs) * m_cfg.currentBucketMs;

    // spectrum scalars: combined Hs; Tp + direction FROM of the most energetic partition.
    int best = -1;
    double bestHs = 0.0;
    for (size_t i = 0; i < parts.size(); ++i) {
        const double hi = SeaState::SignificantHeight(&parts[i], 1);
        if (hi > bestHs) {
            bestHs = hi;
            best = int(i);
        }
    }
    if (best >= 0 && bestHs > kPartHsFloor) {
        in.hs = SeaState::SignificantHeight(parts.data(), int(parts.size()));
        in.tp = 1.0 / double(parts[size_t(best)].fp);
        in.mwdFromDeg = Wrap360(std::atan2(double(parts[size_t(best)].dirToX),
                                           double(parts[size_t(best)].dirToZ)) *
                                    (180.0 / kPiW) +
                                180.0);
    }

    // bed: the ONE composed height (the tree when bound, compose/HeightTree.h), at texel centers at the cell's own
    // resolution -- the same per-texel math the renderer's tiles run (sources are stateless;
    // the residency workers already paint through this stack concurrently).
    in.bed.resize(cells);
    if (m_comp && m_hgtCh >= 0) {
        ParallelRows(ny, [&](int j0, int j1) {
            for (int j = j0; j < j1; ++j) {
                const size_t row = size_t(j) * size_t(nx);
                for (int i = 0; i < nx; ++i) {
                    double la = 0.0, lo = 0.0;   // PHASE C4: the cell's page texel's place
                    m_cfg.PlaceOfCell(double(i) + 0.5, double(j) + 0.5, la, lo);
                    const double latR = la * (kPiW / 180.0), lonR = lo * (kPiW / 180.0);
                    in.bed[row + i] =
                        m_comp->SampleHeight(m_hgtCh, latR, lonR, m_cfg.cellM);
                }
            }
        });
    }
    if (!m_curU.empty() && m_curSig != 0) {
        // M8 flows-into-waves: the SWE's solved jet over the field (quantized planes; their hash
        // is in this solve's key). The dispersion sees the bent channel jet and the shear off the tips.
        in.u = m_curU;
        in.v = m_curV;
    } else if (m_field && m_field->u.size() == cells) {
        in.u = m_field->u;   // W2: no solver's flow (yet, or at all): the field's
        in.v = m_field->v;
    } else {
        in.u.assign(cells, 0.0f);
        in.v.assign(cells, 0.0f);
    }

}

WaveField::Solved WaveField::SolveNow(uint64_t key, double simUnix,
                                      const std::vector<PartParam>& parts, double levelAt) const {
    wavecore::Inputs in;
    BuildInputs(simUnix, parts, in, levelAt);
    const int nx = m_cfg.nx, ny = m_cfg.ny;
    (void)nx;
    (void)ny;
    Solved out;
    out.key = key;
    if (!m_memo) m_memo = std::make_shared<wavecore::Memo>();
    out.phi.assign(size_t((std::min)(m_cfg.nComp, kMaxComp)) * size_t(m_cfg.nx) * size_t(m_cfg.ny), 0.0f);
    wavecore::Solve(in, out.atlas, out.table, m_memo.get(), &out.phi);
    if (wavecore::g_solveAudit) {   // F25: the same question cold, and the cache's answer
        Solved cold;
        wavecore::Solve(in, cold.atlas, cold.table, nullptr);
        size_t dA = 0;
        for (size_t i = 0; i < out.atlas.size() && i < cold.atlas.size(); ++i) dA += out.atlas[i] != cold.atlas[i];
        dA += (out.atlas.size() > cold.atlas.size() ? out.atlas.size() : cold.atlas.size()) -
              (out.atlas.size() < cold.atlas.size() ? out.atlas.size() : cold.atlas.size());
        const bool dT = std::memcmp(&out.table, &cold.table, sizeof(GpuTable)) != 0;
        Log("[wave-audit] the memo's solve against a cold one (key %016llx): %zu of %zu atlas bytes differ, "
            "the table %s", static_cast<unsigned long long>(key), dA, out.atlas.size(),
            dT ? "DIFFERS" : "equal");
        Solved disk;
        if (LoadCache(key, disk)) {
            size_t dD = 0;
            for (size_t i = 0; i < out.atlas.size() && i < disk.atlas.size(); ++i) dD += out.atlas[i] != disk.atlas[i];
            Log("[wave-audit] the solve against the cache's entry for its key: %zu of %zu atlas bytes differ, "
                "the table %s", dD, disk.atlas.size(),
                std::memcmp(&out.table, &disk.table, sizeof(GpuTable)) != 0 ? "DIFFERS" : "equal");
        }
    }

    // v2 cache: the answer AND the question (input planes), so the offline checker can hold
    // this solve to the python proof without the engine. StoreCache below stays the
    // input-less writer for callers that only have a Solved.
    char path[128];
    CachePathFor(key, path, sizeof(path));
    wavecore::WriteCache(path, key, in, out.atlas, out.table, true, &out.phi);
    return out;
}

void WaveField::SetSolveAudit(bool on) { wavecore::g_solveAudit = on; }

// F25/F27: a v2 cache entry read back as the question it answered (its input planes) and the
// answer (atlas, table), for the two offline instruments below.
static bool ReadEntry(const std::string& path, wavecore::Inputs& in, std::vector<uint8_t>& atlas,
                      WaveField::GpuTable& table, uint64_t& key, const char* tag) {
    using namespace wavecore;
    FILE* f = fopen(path.c_str(), "rb");
    if (!f) { Log("[%s] cannot open %s", tag, path.c_str()); return false; }
    CacheHead hd;
    bool ok = fread(&hd, sizeof(hd), 1, f) == 1 && hd.magic == kCacheMagic &&
              hd.version == kCacheVersion && hd.hasInputs == 1u &&
              hd.tableBytes == sizeof(WaveField::GpuTable);
    if (ok) {
        atlas.resize(size_t(hd.atlasW) * size_t(hd.atlasH) * 4);
        const size_t cells = size_t(hd.nx) * size_t(hd.ny);
        in.bed.resize(cells); in.u.resize(cells); in.v.resize(cells);
        ok = fread(&table, sizeof(table), 1, f) == 1 &&
             fread(atlas.data(), 1, atlas.size(), f) == atlas.size() &&
             fread(in.bed.data(), sizeof(float), cells, f) == cells &&
             fread(in.u.data(), sizeof(float), cells, f) == cells &&
             fread(in.v.data(), sizeof(float), cells, f) == cells;
    }
    fclose(f);
    if (!ok) { Log("[%s] %s is not a v2 entry with input planes", tag, path.c_str()); return false; }
    in.cfg.nx = int(hd.nx);
    in.cfg.ny = int(hd.ny);
    in.cfg.nComp = int(hd.nComp);
    in.cfg.cellM = hd.cellM;
    in.cfg.spreadDeg = hd.spreadDeg;
    in.cfg.gammaHs = hd.gammaHs;
    in.cfg.minSamplesPerLambda = hd.minSamplesPerLambda;
    in.cfg.orgX = hd.orgX;
    in.cfg.orgZ = hd.orgZ;
    in.cfg.featherM = table.feather;
    in.level = hd.level;
    in.currentMs = hd.currentMs;
    in.hs = hd.hs;
    in.tp = hd.tp;
    in.mwdFromDeg = hd.mwdDeg;
    key = hd.key;
    return true;
}

bool WaveField::RecheckCache(const std::string& path) {
    using namespace wavecore;
    Inputs in;
    GpuTable table{};
    std::vector<uint8_t> atlas;
    uint64_t key = 0;
    if (!ReadEntry(path, in, atlas, table, key, "wave-recheck")) return false;
    std::vector<uint8_t> atlas2;
    GpuTable table2{};
    Solve(in, atlas2, table2, nullptr);
    size_t d = 0;
    const size_t n = (std::min)(atlas.size(), atlas2.size());
    for (size_t i = 0; i < n; ++i) d += atlas[i] != atlas2[i];
    d += (std::max)(atlas.size(), atlas2.size()) - n;
    const bool dT = std::memcmp(&table, &table2, sizeof(GpuTable)) != 0;
    Log("[wave-recheck] %s (key %016llx): this binary's solve of the entry's own planes: %zu of %zu atlas "
        "bytes differ, the table %s", path.c_str(), static_cast<unsigned long long>(key), d, atlas.size(),
        dT ? "DIFFERS" : "equal");
    return d == 0 && !dT;
}

// F27 (the instrument): HOW MANY ROUNDS THE EIKONAL NEEDS. This solves an entry's own planes with
// the law's sweep, then keeps sweeping each component a round at a time until no cell moves, and
// says per component how many cells each further round moved and by how much, and how many of
// the DRAWN phase bytes (cos, sin at 8 bits) the law's phase differs in from the still one. It
// found the two-round law wrong on 5 % of the drawn cells (the inlet); under F28 (until still) it
// must report no round moving a cell and no cell drawn differently -- that is its gate.
bool WaveField::ConvergeCache(const std::string& arg) {
    using namespace wavecore;
    // `<entry>[,<png>]`: the picture is where the two rounds' phase draws differently, north up,
    // land dark, water white, red by how many components are off at the cell.
    std::string path = arg, png;
    if (const size_t c = arg.find(','); c != std::string::npos) { path = arg.substr(0, c); png = arg.substr(c + 1); }
    Inputs in;
    GpuTable table{};
    std::vector<uint8_t> atlas;
    uint64_t key = 0;
    if (!ReadEntry(path, in, atlas, table, key, "wave-converge")) return false;
    Memo memo;   // the k planes and each component's (sigma, d0, d1), as the solve found them
    std::vector<uint8_t> atlas2;
    GpuTable t2{};
    Solve(in, atlas2, t2, &memo);
    const int nx = memo.nx, ny = memo.ny, nc = memo.nc;
    const size_t cells = size_t(nx) * size_t(ny);
    if (!cells || !nc) { Log("[wave-converge] nothing solved"); return false; }
    std::vector<double> h(cells);
    for (size_t idx = 0; idx < cells; ++idx) h[idx] = (std::max)(in.level - double(in.bed[idx]), 0.0);
    constexpr int kMaxRounds = 24;
    struct Row { int rounds = 0; size_t moved[kMaxRounds] = {}; double step[kMaxRounds] = {}; size_t cellsOff = 0; bool on = false; };
    std::vector<Row> perComp(static_cast<size_t>(nc));
    std::vector<uint8_t> offAll(cells, 0);   // how many components draw the cell differently
    std::mutex offMx;
    Threads().ParallelFor(Lane::Long, "wave.converge", nc, 1, [&](int c0, int c1) {
        std::vector<double> feik(cells), phi(cells), prev(cells);
        std::vector<uint8_t> offMine(cells, 0);
        for (int ic = c0; ic < c1; ++ic) {
            if (!(t2.kMax[ic] > 0.0f)) continue;   // not uploaded (the fold gate)
            Row& r = perComp[size_t(ic)];
            r.on = true;
            const double sig = memo.sig[ic], d0 = memo.d0[ic], d1 = memo.d1[ic];
            bool bl = false;
            const double kShore = SolveDispersion(sig, kDepthFloor, 0.0, bl);
            const double* kp = memo.k.data() + size_t(ic) * cells;
            for (size_t idx = 0; idx < cells; ++idx) {
                feik[idx] = (h[idx] > 0.0) ? kp[idx] : kShore;
                phi[idx] = 1e30;
            }
            EikonalPhase(nx, ny, in.cfg.cellM, (sig * sig) / PlanetGWave(), d0, d1, h, feik, phi);   // the solve's (F28: until still)
            std::vector<double> phi8 = phi;
            for (int round = 0; round < kMaxRounds; ++round) {
                prev = phi;
                EikonalSweep(nx, ny, in.cfg.cellM, (sig * sig) / PlanetGWave(), d0, d1, feik, phi, 4, &h, 1);   // the water's
                size_t moved = 0; double mx = 0.0;
                for (size_t idx = 0; idx < cells; ++idx) {
                    if (phi[idx] != prev[idx]) { ++moved; mx = (std::max)(mx, std::abs(prev[idx] - phi[idx])); }
                }
                r.moved[round] = moved; r.step[round] = mx; r.rounds = round + 1;
                if (!moved) break;
            }
            for (size_t idx = 0; idx < cells; ++idx) {
                if (h[idx] <= 0.0) continue;   // dry cells draw nothing
                const uint8_t c8 = Quant8(std::cos(phi8[idx]) * 0.5 + 0.5, 1.0), s8 = Quant8(std::sin(phi8[idx]) * 0.5 + 0.5, 1.0);
                const uint8_t c = Quant8(std::cos(phi[idx]) * 0.5 + 0.5, 1.0), s = Quant8(std::sin(phi[idx]) * 0.5 + 0.5, 1.0);
                if (c8 != c || s8 != s) { ++r.cellsOff; ++offMine[idx]; }
            }
        }
        std::lock_guard<std::mutex> lk(offMx);
        for (size_t idx = 0; idx < cells; ++idx) offAll[idx] = uint8_t(offAll[idx] + offMine[idx]);
    });
    size_t wet = 0;
    for (size_t idx = 0; idx < cells; ++idx) wet += h[idx] > 0.0;
    int worstRounds = 0; size_t totalOff = 0;
    for (int ic = 0; ic < nc; ++ic) {
        const Row& r = perComp[size_t(ic)];
        if (!r.on) continue;
        std::string per;
        char b[64];
        for (int k = 0; k < r.rounds; ++k) {
            snprintf(b, sizeof(b), "%s%zu (%.2g rad)", k ? ", " : "", r.moved[k], r.step[k]);
            per += b;
        }
        const int more = r.rounds - (r.moved[r.rounds - 1] ? 0 : 1);
        const double T = 2.0 * kPiW / memo.sig[ic];
        Log("[wave-converge] comp %2d (T %.1f s): still after %d more round(s); cells moved by each (max step): %s; "
            "the 2-round phase draws %zu of %zu wet cells differently (%.2f %%)",
            ic, T, more, per.c_str(), r.cellsOff, wet, 100.0 * double(r.cellsOff) / double((std::max)(wet, size_t(1))));
        worstRounds = (std::max)(worstRounds, more);
        totalOff += r.cellsOff;
    }
    Log("[wave-converge] %s (key %016llx): %dx%d, %zu wet cells, %d components; the slowest component is still "
        "after %d more round(s) beyond the solve's own; drawn-phase cells off, all components: %zu of %zu (%.2f %%)",
        path.c_str(), static_cast<unsigned long long>(key), nx, ny, wet, nc, worstRounds, totalOff,
        wet * size_t(nc), 100.0 * double(totalOff) / double((std::max)(wet * size_t(nc), size_t(1))));
    if (!png.empty()) {
        std::vector<uint8_t> img(cells * 4);
        for (int j = 0; j < ny; ++j) {
            const size_t src = size_t(ny - 1 - j) * size_t(nx);   // row 0 of the solve is SOUTH
            const size_t dst = size_t(j) * size_t(nx);
            for (int i = 0; i < nx; ++i) {
                uint8_t* px = &img[(dst + size_t(i)) * 4];
                px[3] = 255;
                if (h[src + size_t(i)] <= 0.0) { px[0] = px[1] = px[2] = 40; continue; }
                const double f = double(offAll[src + size_t(i)]) / double(nc);   // 0 = all agree
                px[0] = 255;
                px[1] = px[2] = uint8_t(255.0 * (1.0 - (std::min)(1.0, 3.0 * f)));
            }
        }
        const std::wstring wp(png.begin(), png.end());
        if (SavePng(wp, img.data(), uint32_t(nx), uint32_t(ny), uint32_t(nx) * 4, img.size())) {
            Log("[wave-converge] wrote %s: where the solve's two rounds draw the phase differently (red by how many "
                "components, saturating at a third of them)", png.c_str());
        } else {
            Log("[wave-converge] FAILED to write %s", png.c_str());
        }
    }
    return true;
}

// What join() used to do. Main thread only -- Configure and Update -- so it cannot be the
// thread the solve is waiting for.
void WaveField::WaitForSolve() {
    while (m_solveRunning.load(std::memory_order_acquire)) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}

// The display closure: exaggeration multiplies the DEQUANT tables, never the solve. The
// packed bytes stay raw physics (cache and bucket key untouched); the GPU decodes
// a/aMax * (exag*aMaxRaw) and ProbeAt applies this SAME function to its own snapshot, so the
// 9b/9c twin holds. It lives as a function rather than only inside AdoptTable because there
// are now two readers of a solve and a closure copied twice is a closure that can drift.
// (m_cfg is written once, by Configure, before any painting thread exists -- so a const call
// from a worker is safe; if a future Configure ever runs live this read becomes a race.)
WaveField::GpuTable WaveField::DisplayTable(const GpuTable& raw) const {
    GpuTable t = raw;
    const float e = m_cfg.displayExag;
    if (e > 0.0f && e != 1.0f) {
        for (int c = 0; c < kMaxComp; ++c) t.aMax[c] *= e;
        t.envMax *= e;    // the envelope shades what the geometry shows
        // excMax is not scaled: the excess is a RATIO (rms_raw / rms_limit), the same at any
        // exaggeration, and the GPU decodes it at the raw 2.5 (WaterBank.hlsl's excRaw, excW). Scaling
        // it here made ProbeAt's crest law and foam read the excess e times the drawn one.
    }
    return t;
}

void WaveField::AdoptTable(const GpuTable& t) { m_table = DisplayTable(t); }

void WaveField::Offer(Solved&& s, const char* how) { OfferShared(std::make_shared<const Solved>(std::move(s)), how); }

void WaveField::OfferShared(std::shared_ptr<const Solved> s, const char* how) {
    const uint64_t key = s->key;
    {
        std::lock_guard<std::mutex> lk(m_offeredMx);
        m_offered.push_back(s);
        while (m_offered.size() > 3) m_offered.erase(m_offered.begin());
    }
    m_nextKey = key;
    std::atomic_store(&m_next, std::move(s));
    char buf[160];
    snprintf(buf, sizeof(buf), "wave %ux%u key %016llx (%s, pages pending)", m_cfg.nx, m_cfg.ny,
             static_cast<unsigned long long>(key), how);
    stats = buf;
}

void WaveField::PublishKey(uint64_t key) {
    std::shared_ptr<const Solved> n = Next();
    if (n && n->key == key) {
        Publish();
        return;
    }
    const std::shared_ptr<const Solved> s = SolvedFor(key);
    if (!s || (Live() && Live()->key == key)) return;
    // an older offer: it goes live, and the newer next stays next
    std::shared_ptr<const Solved> keep = Next();
    const uint64_t keepKey = m_nextKey;
    std::atomic_store(&m_next, s);
    m_nextKey = key;
    Publish();
    if (keep) {
        std::atomic_store(&m_next, keep);
        m_nextKey = keepKey;
    }
}

void WaveField::Publish() {
    std::shared_ptr<const Solved> n = Next();
    if (!n) return;
    const GpuTable was = m_table;
    m_prevTable = was;
    AdoptTable(n->table);
    if (was.nUsed > 0) {
        // [wave-table] (the roll's table mismatch): what a page painted under the table before decodes
        // as under this one, per component -- the amplitude and wavenumber scales' ratios, the
        // frequencies' and directions' change, and the envelope's and level's.
        double aLo = 1e30, aHi = 0.0, kLo = 1e30, kHi = 0.0, dSig = 0.0, dDir = 0.0;
        int nC = 0;
        for (uint32_t c = 0; c < uint32_t(kMaxComp); ++c) {
            if (!(was.aMax[c] > 0.0f) || !(m_table.aMax[c] > 0.0f)) continue;
            const double ra = double(m_table.aMax[c]) / double(was.aMax[c]);
            const double rk = double(m_table.kMax[c]) / (std::max)(double(was.kMax[c]), 1e-12);
            aLo = (std::min)(aLo, ra); aHi = (std::max)(aHi, ra);
            kLo = (std::min)(kLo, rk); kHi = (std::max)(kHi, rk);
            dSig = (std::max)(dSig, std::abs(double(m_table.sigma[c]) - double(was.sigma[c])));
            dDir = (std::max)(dDir, std::hypot(double(m_table.dirX[c]) - double(was.dirX[c]),
                                               double(m_table.dirZ[c]) - double(was.dirZ[c])));
            ++nC;
        }
        Log("[wave-table] publish %016llx: %d comps | aMax new/old %.3f..%.3f | kMax new/old %.3f..%.3f | "
            "|d sigma| %.2e rad/s | |d dir| %.2e | envMax %.3f -> %.3f, sumMax %.3f -> %.3f, level %+.3f -> %+.3f m",
            static_cast<unsigned long long>(n->key), nC, aLo, aHi, kLo, kHi, dSig, dDir, double(was.envMax),
            double(m_table.envMax), double(was.sumMax), double(m_table.sumMax), double(was.level),
            double(m_table.level));
    }
    m_liveKey = n->key;
    std::atomic_store(&m_live, n);
    std::atomic_store(&m_next, std::shared_ptr<const Solved>());
    char buf[160];
    snprintf(buf, sizeof(buf), "wave %ux%u lvl %+.2f key %016llx (live)", m_table.nx, m_table.ny,
             double(m_table.level), static_cast<unsigned long long>(m_liveKey));
    stats = buf;
}

bool WaveField::LoadCache(uint64_t key, Solved& out) const {
    // --wave-map (the map and its layers) is drawn by a solve: a cache hit would answer the field and draw
    // nothing, so the instrument always solves. Its answer is the cache's to the byte (the key is the
    // question), and it is written back as any solve is.
    if (!m_cfg.mapPath.empty()) return false;
    char path[128];
    CachePathFor(key, path, sizeof(path));
    if (!wavecore::ReadCache(path, key, m_cfg, out.atlas, out.table, &out.phi)) return false;
    out.key = key;
    return true;
}

void WaveField::StoreCache(const Solved& s) const {
    // Input-less fallback writer (hasInputs = 0). The main path -- SolveNow -- writes the v2
    // file with the bed/current planes appended, which is what the twin checker consumes.
    wavecore::Inputs in;
    in.cfg = m_cfg;
    in.level = double(s.table.level);
    char path[128];
    CachePathFor(s.key, path, sizeof(path));
    wavecore::WriteCache(path, s.key, in, s.atlas, s.table, false);
}

bool WaveField::Update(Gpu& gpu, double simUnix, const PartParam* parts, int nParts,
                       bool block, bool reader) {
    if (!m_comp || m_hgtCh < 0 || !m_atlas) return false;
    RefreshSweCurrent(gpu, simUnix, block);   // flows into waves (throttled + inFlight-guarded)

    // hand over a finished background solve first: the old field stays live until the new
    // one is WHOLE (double-buffer by construction -- the renderer never sees a half solve).
    if (m_resultReady.load(std::memory_order_acquire)) {
        WaitForSolve();   // the job stores m_resultReady before it clears m_solveRunning
        m_resultReady.store(false);
        m_inFlight.store(false);
        std::shared_ptr<const Solved> r = std::move(m_result);
        if (r) {
            Log("[wave] a worker's field %016llx (%s, level %+.3f m): %s%s", static_cast<unsigned long long>(r->key),
                r->transient ? "a blend" : "a bucket's solve", double(r->table.level), m_resultOffer ? "offered" : "",
                m_resultHold ? (m_resultOffer ? " and held" : "held") : "");
        }
        if (r && m_resultHold) Hold(r);
        if (r && m_resultOffer) OfferShared(r, r->transient ? "blended" : "solved");
    }

    // F17: THE KEY IS CONTENT, AND THE FLOW IS PART OF IT. A key formed before the solver's first
    // flow has landed names the proxy jet; that field was solved, cached and drawn for the first
    // 90 s of every run since the read went asynchronous (F13). The field waits as it does for a
    // reader. Scenes with no solver have no flow to wait for and solve as before.
    if (m_swe && m_swe->Ready() && !m_flowKnown) {
        if (!m_flowWaitSaid) {
            m_flowWaitSaid = true;
            Log("[wave] the field waits for the solver's first flow: the key is content, and a "
                "flow not yet read is not a key");
        }
        return false;
    }
    // THE TIDE BETWEEN TWO BUCKETS: the floor and ceiling buckets of the live level, its place between
    // them in kBlendSteps; at a step of 0 or kBlendSteps the field IS a bucket's solve.
    const double dB = m_cfg.tideBucketM;
    const double lvl = LevelRaw(simUnix);
    const double L0 = std::floor(lvl / dB) * dB;
    const int step = int(std::lround((lvl - L0) / dB * kBlendSteps));
    const uint64_t k0 = BucketKey(simUnix, parts, nParts, L0);
    const uint64_t k1 = BucketKey(simUnix, parts, nParts, L0 + dB);
    const uint64_t key = step <= 0 ? k0 : step >= kBlendSteps ? k1 : BlendKey(k0, k1, step);
    if (key != NextKey() && !m_inFlight.load()) {
        // A FIELD IS SOLVED FOR A READER. The key is content and rolls when the content does;
        // the solve is work and is done when someone will read the answer -- the window at a
        // level under the pyramid's top, or a hull probing the window (the frame decides, by the
        // same rule every tenant's want uses). Without one the roll is noted once and the field
        // stays what it was; the next reader's frame finds the key still moved and solves then.
        // Measured 2026-10-04: standing at Haulover, the Merrimack field solved twice in 13 s,
        // 11.8 s each on the compute lane, for no reader (out/pc/CHANGES.md F8).
        if (!reader) {
            if (m_waitingKey != key) {
                m_waitingKey = key;
                Log("[wave] bucket rolled to %016llx with no reader: the field waits",
                    static_cast<unsigned long long>(key));
            }
            return false;
        }
        // F13: THE CACHE IS READ WHERE THE SOLVE IS MADE. A hit was read here, on the frame:
        // 294 MB in 65-130 ms at the helm, the roll's hitch whenever a bucket came back. The
        // asynchronous path looks the key up on its worker first and solves only on a miss; the
        // blocking path (the dump frames) reads here as before.
        if (s_blendTest) {   // --tool wave-blend-test: the first solve's question, at three levels
            std::vector<PartParam> pcopy;
            if (parts && nParts > 0) pcopy.assign(parts, parts + nParts);
            std::exit(BlendMidpointTest(simUnix, pcopy) ? 0 : 1);
        }
        std::vector<PartParam> pcopy;
        if (parts && nParts > 0) pcopy.assign(parts, parts + nParts);
        const bool real = step <= 0 || step >= kBlendSteps;
        const bool needPhi = !real;
        // what this decision still lacks: the real solve(s), nearest first, then the blend
        const double w = double(step) / double(kBlendSteps);
        const uint64_t want[2] = {w < 0.5 ? k0 : k1, w < 0.5 ? k1 : k0};
        const double wantL[2] = {w < 0.5 ? L0 : L0 + dB, w < 0.5 ? L0 + dB : L0};
        std::shared_ptr<const Solved> h0 = Held(k0, needPhi), h1 = Held(k1, needPhi);
        if (real) {
            const std::shared_ptr<const Solved> h = step <= 0 ? h0 : h1;
            if (h) {
                OfferShared(h, "held");
            } else if (block) {
                if (auto f = Fetch(key, simUnix, pcopy, step <= 0 ? L0 : L0 + dB, false)) {
                    Hold(f);
                    OfferShared(f, "fetched now");
                }
            } else {
                WaitForSolve();
                m_inFlight.store(true);
                m_resultOffer = true;
                m_resultHold = true;
                m_solveRunning.store(true, std::memory_order_release);
                const double L = step <= 0 ? L0 : L0 + dB;
                Threads().Submit(Lane::Long, "wave.solve", [this, key, simUnix, L, p = std::move(pcopy)]() {
                    m_result = Fetch(key, simUnix, p, L, false);
                    m_resultReady.store(true, std::memory_order_release);
                    m_solveRunning.store(false, std::memory_order_release);
                });
            }
        } else if (h0 && h1) {
            // both neighbours held with their phase: the blend
            if (block) {
                Solved b;
                if (BlendSolves(*h0, *h1, w, key, m_cfg.nComp, b)) {
                    Log("[wave] blended %016llx and %016llx at %.3f (the level %+.3f m; memory only)",
                        static_cast<unsigned long long>(k0), static_cast<unsigned long long>(k1), w, lvl);
                    OfferShared(std::make_shared<const Solved>(std::move(b)), "blended now");
                }
                else OfferShared(w < 0.5 ? h0 : h1, "held (fans differ: no blend)");
            } else {
                WaitForSolve();
                m_inFlight.store(true);
                m_resultOffer = true;
                m_resultHold = false;
                m_solveRunning.store(true, std::memory_order_release);
                const int nComp = m_cfg.nComp;
                Threads().Submit(Lane::Long, "wave.blend", [this, h0, h1, w, key, nComp]() {
                    Solved b;
                    const auto t0 = std::chrono::steady_clock::now();
                    if (BlendSolves(*h0, *h1, w, key, nComp, b)) {
                        Log("[wave] blended %016llx and %016llx at %.3f in %.2f s on a worker (memory only)",
                            static_cast<unsigned long long>(h0->key), static_cast<unsigned long long>(h1->key), w,
                            std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count());
                        m_result = std::make_shared<const Solved>(std::move(b));
                    } else {
                        m_result = w < 0.5 ? h0 : h1;
                    }
                    m_resultReady.store(true, std::memory_order_release);
                    m_solveRunning.store(false, std::memory_order_release);
                });
            }
        } else {
            // a neighbour is missing: fetch the nearer first (shown at once when nothing is), then the other
            for (int n = 0; n < 2; ++n) {
                if (Held(want[n], true)) continue;
                if (block) {
                    if (auto f = Fetch(want[n], simUnix, pcopy, wantL[n], true)) Hold(f);
                    continue;
                }
                WaitForSolve();
                m_inFlight.store(true);
                // the nearer is drawn the moment it lands (a flow's change shows as fast as it always did);
                // the farther is held for the blend that replaces it
                m_resultOffer = n == 0;
                m_resultHold = true;
                m_solveRunning.store(true, std::memory_order_release);
                const uint64_t kk = want[n];
                const double L = wantL[n];
                Threads().Submit(Lane::Long, "wave.solve", [this, kk, simUnix, L, p = pcopy]() {
                    m_result = Fetch(kk, simUnix, p, L, true);
                    m_resultReady.store(true, std::memory_order_release);
                    m_solveRunning.store(false, std::memory_order_release);
                });
                break;
            }
            if (block) {
                h0 = Held(k0, true);
                h1 = Held(k1, true);
                Solved b;
                if (h0 && h1 && BlendSolves(*h0, *h1, w, key, m_cfg.nComp, b)) {
                    Log("[wave] blended %016llx and %016llx at %.3f (the level %+.3f m; memory only)",
                        static_cast<unsigned long long>(k0), static_cast<unsigned long long>(k1), w, lvl);
                    OfferShared(std::make_shared<const Solved>(std::move(b)), "blended now");
                } else if (h0 || h1) {
                    OfferShared(h0 ? h0 : h1, "held (no blend)");
                }
            }
        }
    }
    return m_inFlight.load();
}

// ---- THE TIDE BETWEEN TWO BUCKETS --------------------------------------------------------------
std::shared_ptr<const WaveField::Solved> WaveField::Held(uint64_t key, bool needPhi) const {
    std::lock_guard<std::mutex> lk(m_heldMx);
    for (const auto& s : m_held) {
        if (s->key == key && (!needPhi || !s->phi.empty())) return s;
    }
    return nullptr;
}

void WaveField::Hold(const std::shared_ptr<const Solved>& s) {
    if (!s || s->transient) return;
    std::lock_guard<std::mutex> lk(m_heldMx);
    std::erase_if(m_held, [&](const std::shared_ptr<const Solved>& h) { return h->key == s->key; });
    m_held.push_back(s);
    auto heldBytes = [&] {
        size_t b = 0;
        for (const auto& h : m_held) b += SolvedBytes(*h);
        return b;
    };
    // the oldest goes, while there are too many or they weigh past the budget (the newest stays)
    while (m_held.size() > 1 && (m_held.size() > kHeldMax || heldBytes() > kHeldBytes)) m_held.erase(m_held.begin());
}

std::shared_ptr<const WaveField::Solved> WaveField::Fetch(uint64_t key, double simUnix,
                                                          const std::vector<PartParam>& parts, double levelAt,
                                                          bool needPhi) const {
    Solved s;
    if (!wavecore::g_solveAudit && LoadCache(key, s) && (!needPhi || !s.phi.empty())) {
        s.key = key;
        Log("[wave] cache hit %016llx (%+.3f m%s)", static_cast<unsigned long long>(key), levelAt,
            s.phi.empty() ? "" : ", with its phase");
        return std::make_shared<const Solved>(std::move(s));
    }
    if (needPhi && !s.atlas.empty()) {
        Log("[wave] cache entry %016llx has no phase (written before the blend): solved again to blend it",
            static_cast<unsigned long long>(key));
    }
    Solved n = SolveNow(key, simUnix, parts, levelAt);
    n.key = key;
    PruneWaveCache(key);
    return std::make_shared<const Solved>(std::move(n));
}

uint64_t WaveField::BlendKey(uint64_t k0, uint64_t k1, int step) {
    uint64_t h = 14695981039346656037ull;
    const auto mix = [&h](uint64_t v) {
        for (int b = 0; b < 8; ++b) h = (h ^ ((v >> (8 * b)) & 0xFFu)) * 1099511628211ull;
    };
    mix(0x626c656e64ull);   // "blend"
    mix(k0);
    mix(k1);
    mix(uint64_t(step));
    return h;
}

// THE BLEND (the midpoint test's EIKONAL law): per cell and component, a and k linear, the REAL
// phase linear and the spinor made from it, the envelope linear; scales the larger of the two, so
// nothing clips. A component one neighbour gated off fades in from zero with the other's phase.
bool WaveField::BlendSolves(const Solved& a, const Solved& b, double w, uint64_t key, int nComp, Solved& out) {
    const GpuTable& ta = a.table;
    const GpuTable& tb = b.table;
    if (ta.nx != tb.nx || ta.ny != tb.ny || ta.nUsed != tb.nUsed || a.atlas.size() != b.atlas.size()) return false;
    const uint32_t nx = ta.nx, ny = ta.ny, nc = ta.nUsed;
    const size_t cells = size_t(nx) * size_t(ny), aw = size_t(nx) * 2;
    if (a.phi.size() < size_t(nc) * cells || b.phi.size() < size_t(nc) * cells) return false;
    for (uint32_t c = 0; c < nc; ++c) {
        if (ta.sigma[c] != tb.sigma[c] || ta.dirX[c] != tb.dirX[c] || ta.dirZ[c] != tb.dirZ[c]) return false;
    }
    (void)nComp;
    out = Solved{};
    out.key = key;
    out.transient = true;
    out.atlas.assign(a.atlas.size(), 0);
    GpuTable& t = out.table;
    t = ta;
    t.level = float((1.0 - w) * double(ta.level) + w * double(tb.level));
    const auto dq = [](uint8_t v, double mx) { return (double(v) + 0.5) / 255.0 * mx; };
    const auto q8 = [](double x, double mx) -> uint8_t {
        double v = mx > 0.0 ? x / mx : 0.0;
        v = v > 0.0 ? (v < 1.0 ? v : 1.0) : 0.0;
        const double f = std::floor(v * 255.0);
        return uint8_t(f < 255.0 ? f : 255.0);
    };
    Threads().ParallelFor(Lane::Long, "wave.blend.comps", int(nc) + 1, 1, [&](int c0, int c1) {
        for (int ic = c0; ic < c1; ++ic) {
            const uint32_t c = uint32_t(ic);
            const size_t sx = size_t(c & 1u) * nx, sy = size_t(c >> 1u) * ny;
            if (c == nc) {   // the envelope: rms, excess, sum
                for (uint32_t j = 0; j < ny; ++j) {
                    for (uint32_t i = 0; i < nx; ++i) {
                        const size_t o = ((sy + j) * aw + sx + i) * 4;
                        const uint8_t* pa = &a.atlas[o];
                        const uint8_t* pb = &b.atlas[o];
                        uint8_t* d = &out.atlas[o];
                        d[0] = q8((1 - w) * dq(pa[0], ta.envMax) + w * dq(pb[0], tb.envMax), (std::max)(ta.envMax, tb.envMax));
                        d[1] = q8((1 - w) * dq(pa[1], ta.excMax) + w * dq(pb[1], tb.excMax), (std::max)(ta.excMax, tb.excMax));
                        d[2] = q8((1 - w) * dq(pa[2], ta.sumMax) + w * dq(pb[2], tb.sumMax), (std::max)(ta.sumMax, tb.sumMax));
                        d[3] = q8((1 - w) * dq(pa[3], 1.0f) + w * dq(pb[3], 1.0f), 1.0f);   // sqrt(tan beta): the bed's
                    }
                }
                continue;
            }
            const bool ina = ta.aMax[c] > 0.0f, inb = tb.aMax[c] > 0.0f;
            if (!ina && !inb) continue;
            const double aMax = (std::max)(ta.aMax[c], tb.aMax[c]), kMax = (std::max)(ta.kMax[c], tb.kMax[c]);
            const float* fa = a.phi.data() + size_t(c) * cells;
            const float* fb = b.phi.data() + size_t(c) * cells;
            for (uint32_t j = 0; j < ny; ++j) {
                for (uint32_t i = 0; i < nx; ++i) {
                    const size_t o = ((sy + j) * aw + sx + i) * 4, idx = size_t(j) * nx + i;
                    const uint8_t* pa = &a.atlas[o];
                    const uint8_t* pb = &b.atlas[o];
                    uint8_t* d = &out.atlas[o];
                    const double A = (1 - w) * (ina ? dq(pa[0], ta.aMax[c]) : 0.0) + w * (inb ? dq(pb[0], tb.aMax[c]) : 0.0);
                    const double K = (1 - w) * (ina ? dq(pa[1], ta.kMax[c]) : dq(pb[1], tb.kMax[c])) +
                                     w * (inb ? dq(pb[1], tb.kMax[c]) : dq(pa[1], ta.kMax[c]));
                    const double ph = ina && inb ? (1 - w) * double(fa[idx]) + w * double(fb[idx])
                                                 : double(ina ? fa[idx] : fb[idx]);
                    d[0] = q8(A, aMax);
                    d[1] = q8(K, kMax);
                    d[2] = q8(std::cos(ph) * 0.5 + 0.5, 1.0);
                    d[3] = q8(std::sin(ph) * 0.5 + 0.5, 1.0);
                }
            }
            t.aMax[c] = float(aMax);
            t.kMax[c] = float(kMax);
        }
    });
    t.envMax = (std::max)(ta.envMax, tb.envMax);
    t.excMax = (std::max)(ta.excMax, tb.excMax);
    t.sumMax = (std::max)(ta.sumMax, tb.sumMax);
    return true;
}

// ================================================================================================
//  THE BLEND'S MIDPOINT TEST (--tool wave-blend-test). Whether a solve between two tide buckets can be
//  the BLEND of its neighbours instead of a solve of its own: per cell and component the field is one
//  even element of Cl(2), z = a R with R = e^{B phi} the phase rotor in the e1^e2 plane (`wavefield`,
//  `cl2`). Three blends are held against the TRUE solve at the midpoint level, the bed, flow and
//  spectrum held fixed (only the level moves -- the tide's own effect: depth, so k, refraction and
//  shoaling):
//    SNAP     -- today: the nearest bucket's field (either neighbour);
//    CHORD    -- the cl2 law, componentwise: z = (z0 + z1) / 2 (right for neighbouring CELLS, whose
//                phases differ by little; the chord of an arc is shorter than it by cos(dphi/2));
//    GEODESIC -- the rotor's own interpolation, R0 (~R0 R1)^(1/2), with the amplitude halfway:
//                follows the arc, exact while the phase moves less than pi between the neighbours.
//  The measure is the time-averaged elevation error: sqrt(sum |z - z_true|^2 / sum |z_true|^2) over
//  wet cells and components (the components' frequencies differ, so the cross terms average out), and
//  k's relative error. And the phase step between the neighbours, |arg(~R0 R1)|, whose size decides
//  whether the geodesic is safe. At two spacings: 0.25 m (today's bucket) and 0.5 m.
// ================================================================================================
bool WaveField::BlendMidpointTest(double simUnix, const std::vector<PartParam>& parts) const {
    wavecore::Inputs base;
    BuildInputs(simUnix, parts, base);
    if (!(base.hs > 0.0)) {
        Log("[wave-blend] no sea to solve (Hs %.2f): nothing to test", base.hs);
        return false;
    }
    const int nx = m_cfg.nx, ny = m_cfg.ny;
    const size_t aw = size_t(nx) * 2;
    struct Field {
        std::vector<uint8_t> atlas;
        GpuTable t{};
        std::vector<float> phi;   // the eikonal phase, component-major
    };
    const auto solveAt = [&](double level, Field& f) {
        wavecore::Inputs in = base;
        in.level = level;
        const auto t0 = std::chrono::steady_clock::now();
        f.phi.assign(size_t(m_cfg.nComp) * size_t(m_cfg.nx) * size_t(m_cfg.ny), 0.0f);
        wavecore::Solve(in, f.atlas, f.t, nullptr, &f.phi);
        Log("[wave-blend] solved at %+.3f m NAVD in %.1f s", level,
            std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count());
    };
    // the decoded component at a cell: amplitude, wavenumber, spinor (cos, sin)
    const auto at = [&](const Field& f, uint32_t c, int i, int j, double& a, double& k, double& cs, double& sn) {
        const size_t sx = size_t(c & 1u) * size_t(nx) + size_t(i), sy = size_t(c >> 1u) * size_t(ny) + size_t(j);
        const uint8_t* b = &f.atlas[(sy * aw + sx) * 4];
        a = (double(b[0]) + 0.5) / 255.0 * double(f.t.aMax[c]);
        k = (double(b[1]) + 0.5) / 255.0 * double(f.t.kMax[c]);
        cs = (double(b[2]) + 0.5) / 255.0 * 2.0 - 1.0;
        sn = (double(b[3]) + 0.5) / 255.0 * 2.0 - 1.0;
        const double n = std::sqrt(cs * cs + sn * sn);
        if (n > 1e-9) { cs /= n; sn /= n; }
    };
    bool ok = true;
    const double L = base.level;
    for (const double spacing : {0.25, 0.5}) {
        Field lo, mid, hi;
        solveAt(L - 0.5 * spacing, lo);
        solveAt(L, mid);
        solveAt(L + 0.5 * spacing, hi);
        // the fan must be one fan: the same components, frequencies and directions
        bool sameFan = lo.t.nUsed == mid.t.nUsed && hi.t.nUsed == mid.t.nUsed;
        for (uint32_t c = 0; sameFan && c < mid.t.nUsed; ++c) {
            sameFan = lo.t.sigma[c] == mid.t.sigma[c] && hi.t.sigma[c] == mid.t.sigma[c] &&
                      lo.t.dirX[c] == mid.t.dirX[c] && hi.t.dirX[c] == mid.t.dirX[c];
        }
        if (!sameFan) {
            Log("[wave-blend] spacing %.2f m: the three solves' fans differ -- a blend has no meaning here", spacing);
            ok = false;
            continue;
        }
        double eTrue = 0.0, eSnapLo = 0.0, eSnapHi = 0.0, eChord = 0.0, eGeo = 0.0, ePlant = 0.0;
        double eTrueE = 0.0, eEik = 0.0, eEikPlant = 0.0, eSnapE = 0.0;   // on the eikonal phase
        std::vector<double> dPhiE;
        const size_t cells = size_t(nx) * size_t(ny);
        std::vector<double> kErr, dPhi;
        size_t wet = 0;
        for (int j = 0; j < ny; ++j) {
            for (int i = 0; i < nx; ++i) {
                for (uint32_t c = 0; c < mid.t.nUsed; ++c) {
                    if (!(mid.t.aMax[c] > 0.0f) || !(lo.t.aMax[c] > 0.0f) || !(hi.t.aMax[c] > 0.0f)) continue;
                    double a0, k0, c0, s0, am, km, cm, sm, a1, k1, c1, s1;
                    at(lo, c, i, j, a0, k0, c0, s0);
                    at(mid, c, i, j, am, km, cm, sm);
                    at(hi, c, i, j, a1, k1, c1, s1);
                    if (am < 1e-4 && a0 < 1e-4 && a1 < 1e-4) continue;   // dry in all three
                    ++wet;
                    const double zmR = am * cm, zmI = am * sm;
                    const double z0R = a0 * c0, z0I = a0 * s0, z1R = a1 * c1, z1I = a1 * s1;
                    eTrue += zmR * zmR + zmI * zmI;
                    eSnapLo += (z0R - zmR) * (z0R - zmR) + (z0I - zmI) * (z0I - zmI);
                    eSnapHi += (z1R - zmR) * (z1R - zmR) + (z1I - zmI) * (z1I - zmI);
                    const double chR = 0.5 * (z0R + z1R), chI = 0.5 * (z0I + z1I);
                    eChord += (chR - zmR) * (chR - zmR) + (chI - zmI) * (chI - zmI);
                    // ~R0 R1: the rotor carrying R0 onto R1; its angle is the phase step
                    const double rR = c0 * c1 + s0 * s1, rI = c0 * s1 - s0 * c1;
                    const double step = std::atan2(rI, rR);
                    if (a0 > 1e-4 && a1 > 1e-4) dPhi.push_back(std::abs(step));
                    const double h = 0.5 * step, ag = 0.5 * (a0 + a1);
                    const double gc = c0 * std::cos(h) - s0 * std::sin(h), gs = s0 * std::cos(h) + c0 * std::sin(h);
                    const double gR = ag * gc, gI = ag * gs;
                    eGeo += (gR - zmR) * (gR - zmR) + (gI - zmI) * (gI - zmI);
                    // THE PLANT: the geodesic turned the wrong way, R0 e^{-B step/2}
                    const double pc = c0 * std::cos(h) + s0 * std::sin(h), ps = s0 * std::cos(h) - c0 * std::sin(h);
                    ePlant += (ag * pc - zmR) * (ag * pc - zmR) + (ag * ps - zmI) * (ag * ps - zmI);
                    if (km > 1e-6) kErr.push_back(std::abs(0.5 * (k0 + k1) - km) / km);
                    // EIKONAL: the real phases, halfway, and the amplitude halfway -- then the rotor
                    const size_t pi_ = size_t(c) * cells + size_t(j) * size_t(nx) + size_t(i);
                    const double p0 = lo.phi[pi_], pm = mid.phi[pi_], p1 = hi.phi[pi_];
                    const double tR = am * std::cos(pm), tI = am * std::sin(pm);
                    eTrueE += tR * tR + tI * tI;
                    const double pb = 0.5 * (p0 + p1);
                    eEik += (ag * std::cos(pb) - tR) * (ag * std::cos(pb) - tR) + (ag * std::sin(pb) - tI) * (ag * std::sin(pb) - tI);
                    eSnapE += 0.5 * ((a0 * std::cos(p0) - tR) * (a0 * std::cos(p0) - tR) + (a0 * std::sin(p0) - tI) * (a0 * std::sin(p0) - tI) +
                                     (a1 * std::cos(p1) - tR) * (a1 * std::cos(p1) - tR) + (a1 * std::sin(p1) - tI) * (a1 * std::sin(p1) - tI));
                    // THE PLANT: the eikonal phase blended with the neighbours swapped in weight (a third and two thirds)
                    const double pp = (2.0 * p0 + p1) / 3.0;
                    eEikPlant += (ag * std::cos(pp) - tR) * (ag * std::cos(pp) - tR) + (ag * std::sin(pp) - tI) * (ag * std::sin(pp) - tI);
                    if (a0 > 1e-4 && a1 > 1e-4) dPhiE.push_back(std::abs(p1 - p0));
                }
            }
        }
        const auto rel = [&](double e) { return eTrue > 0.0 ? std::sqrt(e / eTrue) : 0.0; };
        const auto q = [](std::vector<double>& v, double f) {
            if (v.empty()) return 0.0;
            std::nth_element(v.begin(), v.begin() + ptrdiff_t(f * double(v.size() - 1)), v.end());
            return v[size_t(f * double(v.size() - 1))];
        };
        size_t overHalfPi = 0;
        for (double d : dPhi) overHalfPi += d > 1.5707963 ? 1 : 0;
        const double snap = 0.5 * (rel(eSnapLo) + rel(eSnapHi));
        Log("[wave-blend] spacing %.2f m (the midpoint %+.3f m between %+.3f and %+.3f), %zu wet component-cells: "
            "the elevation error relative to the true solve -- SNAP %.4f (low %.4f, high %.4f), CHORD %.4f, "
            "GEODESIC %.4f; k's error median %.4f, p95 %.4f",
            spacing, L, L - 0.5 * spacing, L + 0.5 * spacing, wet, snap, rel(eSnapLo), rel(eSnapHi), rel(eChord),
            rel(eGeo), q(kErr, 0.5), q(kErr, 0.95));
        Log("[wave-blend] spacing %.2f m: the phase step between the neighbours |arg(~R0 R1)| median %.4f rad, "
            "p95 %.4f, p99 %.4f, max %.4f; over pi/2 at %.3f %% of component-cells",
            spacing, q(dPhi, 0.5), q(dPhi, 0.95), q(dPhi, 0.99), q(dPhi, 1.0),
            dPhi.empty() ? 0.0 : 100.0 * double(overHalfPi) / double(dPhi.size()));
        const auto relE = [&](double e) { return eTrueE > 0.0 ? std::sqrt(e / eTrueE) : 0.0; };
        Log("[wave-blend] spacing %.2f m, ON THE EIKONAL PHASE (the real phi, unwrapped by the solve): SNAP %.4f, "
            "EIKONAL BLEND %.4f; the real phase step |phi1 - phi0| median %.3f rad, p95 %.3f, max %.3f -- "
            "PLANT (weights a third / two thirds) %.4f: %s",
            spacing, relE(eSnapE), relE(eEik), q(dPhiE, 0.5), q(dPhiE, 0.95), q(dPhiE, 1.0), relE(eEikPlant),
            relE(eEikPlant) > relE(eEik) ? "CAUGHT" : "NOT CAUGHT");
        ok &= relE(eEikPlant) > relE(eEik);
        Log("[wave-blend] spacing %.2f m: PLANT the geodesic turned the wrong way: %.4f -- %s", spacing, rel(ePlant),
            rel(ePlant) > rel(eGeo) ? "CAUGHT" : "NOT CAUGHT: this instrument cannot see a blend gone wrong");
        ok &= rel(ePlant) > rel(eGeo);
    }
    return ok;
}

WaveField::Probe WaveField::ProbeAt(double wx, double wz, double simUnix, double depthM, double chopM) const {
    Probe p;
    const std::shared_ptr<const Solved> live = Live();
    if (!live || live->atlas.empty()) return p;
    const std::vector<uint8_t>& m_cpuAtlas = live->atlas;
    // THREAD SAFETY (this used to be half-done): the atomic_load above takes a snapshot of the
    // BYTES, and the table must come from the same one. Reading m_table instead was two bugs
    // in one line -- a plain data race, because AdoptTable rewrites that member field by field
    // on the main thread while painting threads probe, and a torn PAIRING, because even a
    // clean read could hand this loop the new solve's dequant scales over the old solve's
    // bytes, decoding every component whose aMax moved at the wrong amplitude until the next
    // frame. live->table is immutable the instant it is published, so it is read here and the
    // display closure applied locally. The snapshot is now atlas-and-table or neither.
    const GpuTable t = DisplayTable(live->table);
    const int nx = int(t.nx), ny = int(t.ny);
    if (nx < 2 || ny < 2) return p;
    // texel-center bilinear, exactly the GPU kernel's addressing (clamp at the window edge).
    const double gx = (wx - double(t.orgX)) * double(t.invCell) - 0.5;
    const double gz = (wz - double(t.orgZ)) * double(t.invCell) - 0.5;
    if (gx < -0.5 || gz < -0.5 || gx > double(nx) - 0.5 || gz > double(ny) - 0.5) return p;
    m_probeReadT.store(simUnix, std::memory_order_relaxed);   // a reader (the hull's twin)
    const double fx = (std::min)((std::max)(gx, 0.0), double(nx - 1));
    const double fz = (std::min)((std::max)(gz, 0.0), double(ny - 1));
    const int i0 = (std::min)(int(fx), nx - 2);
    const int j0 = (std::min)(int(fz), ny - 2);
    const double wxf = fx - double(i0), wzf = fz - double(j0);
    const size_t aw = size_t(t.nx) * 2;
    auto bil = [&](uint32_t slice, int ch) {
        const size_t sx = size_t(slice & 1u) * t.nx + size_t(i0);
        const size_t sy = size_t(slice >> 1u) * t.ny + size_t(j0);
        auto at = [&](size_t dx, size_t dy) {
            // (byte + 0.5)/255: the truncating quantizer's unbiased decode center
            return (double(m_cpuAtlas[((sy + dy) * aw + sx + dx) * 4 + size_t(ch)]) + 0.5) /
                   255.0;
        };
        return (at(0, 0) * (1.0 - wxf) + at(1, 0) * wxf) * (1.0 - wzf) +
               (at(0, 1) * (1.0 - wxf) + at(1, 1) * wxf) * wzf;
    };
    // Eight running sums, all in DOUBLE. The engine's law is doubles on the CPU, and there is
    // a concrete reason here: kMaxComp is 32 now, and accumulating 32 terms in float is 32
    // roundings of the quantity a whole hull responds to. (eta moved to double with the rest:
    // the shift is ~1e-7 m, five orders under the 8-bit atlas's own half-LSB floor, so the
    // 9b/9c twin sees nothing -- it is only ever more accurate.)
    double eta = 0.0, dxS = 0.0, dzS = 0.0, sxS = 0.0, szS = 0.0;
    // THE CREST LAW's ratio, read before the components: the solver's raw-over-limited excess
    // at this point (the envelope slice; 1 where nothing breaks). With depthM <= 0 (the
    // instruments' deep-water read) the orbit is Airy's and the crest is not thrown.
    // (2026-10-23: the ratio UNCLAMPED, and the law run wherever the kernel runs it -- the kernel never
    // gated on the sea's excess, and this twin did, so a set crest the kernel threw in deep water the
    // hull never felt. WaterBank.hlsl's solved loop, THE SEA'S EXCESS UNCLAMPED, says why.)
    const double excRaw = (depthM > 0.0) ? bil(t.envSlice, 1) * double(t.excMax) : 0.0;
    // THE SWASH (WaterTerms.h SwashShare; the kernel's solved loop): the limiter scaled every component by
    // 1 / max(excess, 1), so the raw sea is the delivered one times that; each component runs up by its own
    const double rawW = (std::max)(excRaw, 1.0);
    const double rmsDel = bil(t.envSlice, 0) * double(t.envMax);
    double sumA = 0.0, sumAK = 0.0, sumACoth = 0.0, sumACothR = 0.0, sumADx = 0.0, sumADz = 0.0;
    double liftS = 0.0;   // -E[eta div D], chop 1 (Probe::lift)
    double vxS = 0.0, vyS = 0.0, vzS = 0.0;
    double jxxS = 0.0, jxzS = 0.0, jzzS = 0.0;
    for (uint32_t c = 0; c < t.nUsed && c < uint32_t(kMaxComp); ++c) {
        if (!(t.aMax[c] > 0.0f)) continue;   // gated to the cascades (or flat sea)
        const double a = bil(c, 0) * double(t.aMax[c]);
        const double k = bil(c, 1) * double(t.kMax[c]);
        const double cs = bil(c, 2) * 2.0 - 1.0;
        const double sn = bil(c, 3) * 2.0 - 1.0;
        // the rotor: (c,s) -> cos/sin(phi - sigma t). The spinor stays unit (bilinear of unit
        // spinors shrinks inside the circle, never rotates -- the cl2 law), and the LENGTH the lerp
        // left is the component's phase coherence over the footprint read: it multiplies the
        // amplitude, as the kernel's M9bu law does (WaterBank.hlsl `coh`), rather than being thrown
        // away -- a full-amplitude wave at the circular mean of a phase that turns over inside the
        // cell is not the field (the water match, step 2: the hull's sum and the bank's agree).
        const double wt = double(t.sigma[c]) * simUnix;
        const double ct = std::cos(wt), st = std::sin(wt);
        double ca = cs * ct + sn * st;
        double sa = sn * ct - cs * st;
        const double nrm = std::sqrt(ca * ca + sa * sa);
        if (nrm > 1e-6) {
            ca /= nrm;
            sa /= nrm;
        }
        const double aC0 = a * (std::min)(nrm, 1.0);   // the amplitude the footprint carries
        p.a[c] = float(a);
        p.k[c] = float(k);
        p.phase[c] = float(std::atan2(sa, ca));

        // ---- the six new sums. Derivation and sign ledger: WaveField.h, above `struct
        // Probe`. In one line: ca IS cos(theta) with theta = phi - sigma*t, so d(theta)/dt
        // = -sigma and grad(theta) = k*d^, and everything below is the chain rule on the
        // eta line that follows. a*ca and a*sa are the wave's in-phase and quadrature parts;
        // all six quantities are those two times a scalar, which is why this costs no
        // second sincos and no second bilinear tap.
        const double dX = double(t.dirX[c]), dZ = double(t.dirZ[c]);
        const double sig = double(t.sigma[c]);
        const double aC = aC0 * ca;   // in phase with the crest
        const double aS = aC0 * sa;   // 90 deg ahead of it
        // THE ORBIT IS THE DEPTH'S (the crest law, 2026-10-19; WaterBank.hlsl's solved loop): the
        // horizontal excursion is coth(kh) times the vertical -- Airy's ellipse at the surface, 1
        // in deep water. depthM <= 0 is the deep-water orbit (the instruments that read the field
        // alone); the hull's twin hands the depth it stands in.
        const double coth = (depthM > 0.0) ? 1.0 / std::tanh(k * (std::max)(depthM, 0.3)) : 1.0;
        // ...and in the swash toward the raw one, on this component's run-up (L0 = 2 pi coth / k)
        const double rawF = (rawW > 1.0)
            ? 1.0 + SwashShare(2.0 * a * rawW, 6.283185307179586 * coth / (std::max)(k, 1e-4), depthM) * (rawW - 1.0)
            : 1.0;
        const double cothL = coth * rawF;            // the lateral's factor
        eta += aC;                                   // eta = a cos(theta)   <- the reference
        dxS -= aS * dX * cothL;                      // D_h = -a sin(theta) coth(kh) d^   (chop = 1;
        dzS -= aS * dZ * cothL;                      //        WaterBank scales this by wfChop)
        // the sea's sums for the crest law after the loop (the kernel's)
        sumA += aC0;
        sumAK += aC0 * k;
        sumACoth += aC0 * coth;
        sumACothR += aC0 * cothL;
        sumADx += aC0 * dX;
        sumADz += aC0 * dZ;
        liftS += 0.5 * k * aC0 * aC0 * cothL;        // -E[eta div D] = k a a coth / 2 (the mean lift)
        sxS -= aS * k * dX;                          // grad eta = -a k sin(theta) d^
        szS -= aS * k * dZ;                          //   (the k is the term: a slope is 1/L)
        vxS += sig * aC * dX;                        // u = +sigma a cos(theta) d^  (crest
        vzS += sig * aC * dZ;                        //   water runs WITH the wave)
        vyS += sig * aS;                             // w = +sigma a sin(theta) == d(eta)/dt
        const double aKC = aC * k * cothL;           // grad D_h = -a k cos(theta) coth(kh) d^ (x) d^
        jxxS -= aKC * dX * dX;
        jxzS -= aKC * dX * dZ;
        jzzS -= aKC * dZ * dZ;
    }
    // THE CREST LAW (WaterTerms.h CrestLaw, the one law every description of the sea runs; the kernel's
    // twin is WaterBank.hlsl CrestLaw): on the sum, the crest thrown toward the landing by its own excess
    // where the bed plunges, the lip's fall bounded by the trough, the slope and the offset's Jacobian
    // carried through. Here the lateral is at chop 1 (the caller's lq carries the chop): post = chopM.
    if (depthM > 0.0 && sumA > 1e-9) {
        const double aEnv = (std::max)(1.4142135623730951 * rmsDel, 1e-3);
        const double kBar = sumAK / sumA;
        const double cothBar = sumACoth / sumA;
        // the sea's drawn lateral amplitude (chop applied), the swash's raise in it, HELD TO THE LANDING
        // (the kernel's bound): the lateral, its Jacobian and the lift take the one factor
        double aH = aEnv * (sumACothR / sumA) * chopM;
        const double fSw = (std::min)(1.0, LipReach(kBar, cothBar, 2.0 * aEnv, (std::max)(depthM, 0.3)) /
                                               (std::max)(aH, 1e-4));
        aH *= fSw;
        dxS *= fSw;
        dzS *= fSw;
        jxxS *= fSw;
        jxzS *= fSw;
        jzzS *= fSw;
        liftS *= fSw;
        const double dLen = std::sqrt(sumADx * sumADx + sumADz * sumADz);
        const double dBx = dLen > 1e-9 ? sumADx / dLen : 0.0, dBz = dLen > 1e-9 ? sumADz / dLen : 0.0;
        const double sqB = bil(t.envSlice, 3);   // sqrt(tan beta), the solve's bed slope
        CrestState st;
        st.eta = eta; st.dx = dxS; st.dz = dzS; st.sx = sxS; st.sz = szS;
        st.jxx = jxxS; st.jxz = jxzS; st.jzz = jzzS;
        CrestLaw(st, dBx, dBz, aEnv, excRaw, kBar, cothBar, aH, chopM, sqB * sqB, depthM);
        eta = st.eta; dxS = st.dx; dzS = st.dz; sxS = st.sx; szS = st.sz;
        jxxS = st.jxx; jxzS = st.jxz; jzzS = st.jzz;
    }
    p.eta = float(eta);
    p.dx = float(dxS);
    p.dz = float(dzS);
    p.sx = float(sxS);
    p.sz = float(szS);
    p.vx = float(vxS);
    p.vy = float(vyS);
    p.vz = float(vzS);
    p.jxx = float(jxxS);
    p.jxz = float(jxzS);
    p.jzz = float(jzzS);
    p.lift = float(liftS);
    p.rms = float(bil(t.envSlice, 0) * double(t.envMax));
    p.excess = float(bil(t.envSlice, 1) * double(t.excMax));
    p.valid = true;
    return p;
}

// THE GAUGE AS A SELFTEST. A smooth shoaling k over a wet window; the sweep's phase must have
// |grad phi| = k to the percentiles below, the inflow edges must carry the incident plane
// exactly, and the instrument must SEE a wrong phase: the same field scaled by 1.1 reads a
// residual of 0.1 at its median. The thresholds are the solver's own measured residual on the
// Merrimack field (p50 0.08 %, p90 0.68 %) with room; a smooth field has no caustic creases, so
// its p99 is held too.
bool RunWaveFieldSelfTest() {
    const int nx = 160, ny = 120;
    const double cellM = 10.0, T = 8.0, sig = 2.0 * 3.14159265358979 / T;
    const double kInfC = sig * sig / PlanetGWave();
    const double prop = 0.9;   // travel direction, radians from north through east
    const double d0 = std::sin(prop), d1 = std::cos(prop);
    const size_t cells = size_t(nx) * size_t(ny);
    std::vector<double> h(cells, 20.0), k(cells), phi(cells, 1e30);   // phi starts far (the sweep lowers it)
    for (int j = 0; j < ny; ++j)
        for (int i = 0; i < nx; ++i) {
            const double u = double(i) / nx, v = double(j) / ny;
            k[size_t(j) * nx + i] = kInfC * (1.0 + 0.5 * u + 0.3 * v * v);   // a shoaling k
        }
    wavecore::EikonalSweep(nx, ny, cellM, kInfC, d0, d1, k, phi);
    bool ok = true;
    // the inflow edges: the incident plane, exactly
    const int ib = (d0 >= 0.0) ? 0 : (nx - 1), jb = (d1 >= 0.0) ? 0 : (ny - 1);
    for (int i = 0; i < nx; ++i) {
        const double want = kInfC * d0 * double(i - ib) * cellM;
        if (std::abs(phi[size_t(jb) * nx + i] - want) > 1e-9) {
            Log("[wavetest] FAIL: inflow row phase %.6f, the plane says %.6f", phi[size_t(jb) * nx + i], want);
            ok = false;
            break;
        }
    }
    const wavecore::EikonalGauge g = wavecore::GaugeOf(nx, ny, cellM, h, k, phi);
    Log("[wavetest] eikonal residual |grad phi|/k over %zu cells: p50 %.3f%% p90 %.3f%% p99 %.3f%% max %.2f%%",
        g.cells, g.p50 * 100.0, g.p90 * 100.0, g.p99 * 100.0, g.max * 100.0);
    if (!(g.cells > cells / 2 && g.p50 < 0.002 && g.p90 < 0.01 && g.p99 < 0.03)) {
        Log("[wavetest] FAIL: the sweep's phase is not eikonal to the pinned percentiles");
        ok = false;
    }
    // the instrument sees a wrong phase
    std::vector<double> wrong(phi);
    for (double& x : wrong) x *= 1.1;
    const wavecore::EikonalGauge w = wavecore::GaugeOf(nx, ny, cellM, h, k, wrong);
    if (!(std::abs(w.p50 - 0.1) < 0.005)) {
        Log("[wavetest] FAIL: the gauge read p50 %.4f on a phase scaled by 1.1 (0.1 expected)", w.p50);
        ok = false;
    }
    // W1: THE ACTION'S TWO LIMITS. (a) Deep water, oblique: the transport hands the incident amplitude
    // back. (b) Straight parallel contours (depth a function of x alone), no current: Snell and energy-
    // flux shoaling, A = a0 sqrt((cg0 / cg)(cos th_e / cos th)), th by Snell from the east edge's th_e
    // -- the law the bar normal stated for one window, which the transport must reproduce wherever its
    // assumption is true. Measured over the cells whose rays enter by the east edge (the north edge's
    // datum is the swell shoaled without refraction: the edge's one assumption).
    auto carry = [&](int cx, int cy, double dx, const std::vector<double>& hh, double t0, double t1,
                     std::vector<float>& aOut) {
        const size_t n = size_t(cx) * size_t(cy);
        std::vector<double> kk(n), ph(n, 1e30), nn(n);
        std::vector<float> fl(2 * n), none;
        std::vector<uint32_t> up(n);
        std::vector<float> ry(3 * n);
        for (size_t idx = 0; idx < n; ++idx) {
            bool bl = false;
            kk[idx] = SolveDispersion(sig, hh[idx], 0.0, bl);
        }
        wavecore::EikonalPhase(cx, cy, dx, kInfC, t0, t1, hh, kk, ph);
        aOut.assign(n, 0.0f);
        return wavecore::ActionTransport(cx, cy, dx, sig, t0, t1, 1.0, hh, kk.data(), ph, none, none, fl, nn, up, ry,
                                         aOut.data());
    };
    {
        const int ax = 96, ay = 96;
        const double th = 0.6;   // toward the north-east, oblique to the grid
        std::vector<double> hd(size_t(ax) * ay, 400.0);
        std::vector<float> a;
        const int rounds = carry(ax, ay, 4.0, hd, std::sin(th), std::cos(th), a);
        double worst = 0.0;
        for (const float x : a) worst = (std::max)(worst, std::abs(double(x) - 1.0));
        Log("[wavetest] action, deep and oblique: |A/a0 - 1| max %.2e over %d cells (%d rounds)", worst, ax * ay,
            rounds);
        if (!(worst < 1e-3)) {
            Log("[wavetest] FAIL: the deep sea's action does not hand back the incident amplitude");
            ok = false;
        }
    }
    {
        const int sx = 400, sy = 1200;
        const double dx = 2.0, th0 = 30.0 * 3.14159265358979 / 180.0;
        const double t0 = -std::cos(th0), t1 = -std::sin(th0);   // toward the west-south-west, from deep water
        std::vector<double> hs(size_t(sx) * sy);
        for (int j = 0; j < sy; ++j)
            for (int i = 0; i < sx; ++i) hs[size_t(j) * sx + i] = 2.0 + 58.0 * double(i) / double(sx - 1);
        std::vector<float> a;
        const int rounds = carry(sx, sy, dx, hs, t0, t1, a);
        const double gW = PlanetGWave(), cg0 = 0.5 * gW / sig, c0 = gW / sig;
        auto at = [&](double hc, double& cg, double& c) {
            bool bl = false;
            const double kk = SolveDispersion(sig, hc, 0.0, bl);
            const double kh = kk * hc;
            c = sig / kk;
            cg = 0.5 * (1.0 + 2.0 * kh / std::sinh(2.0 * kh)) * c;
        };
        // Snell from DEEP water, cos th0 -- not the east edge's own refracted angle, which the first version of
        // this test used and so agreed with an edge datum that overfed a shallow edge (the review, 2026-10-24)
        const double cos0 = std::cos(th0);
        std::vector<double> err;
        const double reach = double(sx) * dx * std::tan(th0);   // a ray's fall over the window's width
        for (int j = 4; j < sy - 4; ++j) {
            if (double(j) * dx > double(sy) * dx - reach - 200.0) continue;   // fed by the north edge
            for (int i = 4; i < sx - 4; ++i) {
                const double hc = hs[size_t(j) * sx + i];
                if (hc < 3.0) continue;
                double cg = 0.0, c = 0.0;
                at(hc, cg, c);
                const double sn = std::sin(th0) * c / c0, cs = std::sqrt(1.0 - sn * sn);
                const double want = std::sqrt((cg0 / cg) * (cos0 / cs));
                err.push_back(std::abs(double(a[size_t(j) * sx + i]) / want - 1.0));
            }
        }
        std::sort(err.begin(), err.end());
        const double p50 = err.empty() ? 1.0 : err[err.size() / 2];
        const double p99 = err.empty() ? 1.0 : err[(std::min)(err.size() - 1, size_t(0.99 * double(err.size())))];
        const double mx = err.empty() ? 1.0 : err.back();
        Log("[wavetest] action, straight contours 60 -> 2 m at 30 deg: |A / Snell-shoaled - 1| p50 %.3f%% p99 %.3f%% "
            "max %.3f%% over %zu cells (%d rounds)", p50 * 100.0, p99 * 100.0, mx * 100.0, err.size(), rounds);
        if (!(p99 < 0.02 && mx < 0.05)) {
            Log("[wavetest] FAIL: the action on straight contours is not Snell and shoaling");
            ok = false;
        }
    }
    // THE JETTY'S SHADOW (Mase's diffraction): a semi-infinite breakwater across half of a deep window, the
    // swell square on to it. Geometric optics leaves the lee black; the energy balance with diffraction
    // leaves a third to a half of the energy on the line the tip casts (the random-wave diagrams give about
    // half), the lit side whole far from it, and the lee dimming with distance into it. Read 3 to 7
    // wavelengths behind.
    {
        const int bx = 400, by = 400;
        const double dx = 4.0;   // 1.6 km square; T 8 s in 50 m: lambda ~100 m
        std::vector<double> hb(size_t(bx) * by, 50.0);
        for (int j = 0; j < by / 2; ++j)
            for (int i = 20; i < 23; ++i) hb[size_t(j) * bx + i] = 0.0;   // the breakwater: rows j < by/2
        std::vector<float> a;
        const int rounds = carry(bx, by, dx, hb, 1.0, 0.0, a);
        auto meanAt = [&](int j0, int j1) {   // the mean amplitude over rows [j0, j1), columns 3..7 lambda behind
            double sum = 0.0;
            int n = 0;
            for (int j = j0; j < j1; ++j)
                for (int i = 23 + 75; i < 23 + 175; ++i) {
                    sum += double(a[size_t(j) * bx + i]);
                    ++n;
                }
            return n ? sum / n : 0.0;
        };
        const double line = meanAt(by / 2 - 1, by / 2 + 1);    // the two rows either side of the tip's line
        const double lit = meanAt(by / 2 + 75, by / 2 + 85);   // 300 m into the light
        const double lee = meanAt(by / 2 - 85, by / 2 - 75);   // 300 m into the lee
        Log("[wavetest] action, the breakwater's shadow (T 8 s, 50 m, kappa %.1f): amplitude on the tip's line %.3f "
            "(energy %.3f), 300 m into the light %.3f, 300 m into the lee %.3f (%d rounds)",
            kMaseKappa, line, line * line, lit, lee, rounds);
        if (!(line * line > 0.35 && line * line < 0.65 && lit > 0.9 && lit < 1.1 && lee < 0.35 && lee > 0.0)) {
            Log("[wavetest] FAIL: the breakwater's shadow is not the diffracted one (half the energy on the line, "
                "the light whole, the lee dim)");
            ok = false;
        }
    }
    Log("[wavetest] %s", ok ? "PASS" : "FAIL");
    return ok;
}

}  // namespace ga
