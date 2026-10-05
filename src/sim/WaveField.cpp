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
//    * current: a deterministic climatological proxy, NOT the live SWE (which stays a
//      per-frame amplitude modifier elsewhere): the bucketed signed ACT speed spread over the
//      vqview ebb-jet conveyance envelope w = clip((h-1.2)/(7.0-1.2),0,1)^0.75, a seaward
//      decay exp(-(x-800)/420) past the jetty tips, and the x3.0 throat-vs-station
//      calibration CLOSURE (ACT station max 0.91 m/s -> throat ~1.3, like the reference).
//      Ebb (signed speed < 0, CurrentModel's +flood/-ebb convention) flows TOWARD compass
//      105; flood toward 285.
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
#include "core/ThreadManager.h"

#include "hal/Gpu.h"
#include "sim/SweSolver.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <functional>

namespace ga {

namespace {

constexpr double kPiW = 3.14159265358979323846;
constexpr double kGrav = 9.81;
constexpr int kBracketN = 96;        // log-spaced dispersion candidates (proof: nsamp=96)
constexpr int kBisectN = 48;         // bisections -> bracket width ~4e-16 * k
constexpr double kDepthFloor = 0.15; // m; the proof's np.maximum(h, 0.15)
constexpr double kCgFloor = 0.15;    // m/s floor under cg + U_along (shoaling denominator)
constexpr double kSnellClip = 0.999; // sine clip at grazing
constexpr double kKrCosFloor = 1e-3; // Kr cosine floors
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
        return s * s - (kGrav * k) * std::tanh(k * hf);
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
    Threads().ParallelFor(Lane::Compute, "wave.rows", ny, chunk, fn);
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

// The climatological current proxy (engine decision 2 in the file header). Depth-gated
// conveyance + seaward decay + the x3.0 throat closure; direction by the sign convention
// pinned in CurrentModel.h (TidalEvent: "+ flood, - ebb"), ebb TOWARD compass 105.
void BuildCurrentPlanes(const WaveFieldConfig& cfg, double level, double signedMs,
                        const std::vector<float>& bed, std::vector<float>& u,
                        std::vector<float>& v) {
    const size_t cells = size_t(cfg.nx) * size_t(cfg.ny);
    u.assign(cells, 0.0f);
    v.assign(cells, 0.0f);
    if (signedMs == 0.0) return;   // slack: the sea rides currentless
    const double dirDeg = (signedMs < 0.0) ? 105.0 : 285.0;
    const double dirRad = dirDeg * (kPiW / 180.0);
    const double sinD = std::sin(dirRad), cosD = std::cos(dirRad);
    const double mag = std::fabs(signedMs) * 3.0;   // CLOSURE: ACT max 0.91 -> throat ~1.3
    for (int j = 0; j < cfg.ny; ++j) {
        const size_t row = size_t(j) * size_t(cfg.nx);
        for (int i = 0; i < cfg.nx; ++i) {
            const double h = (std::max)(level - double(bed[row + i]), 0.0);
            double w = (h - 1.2) / (7.0 - 1.2);   // the vqview ebb-jet conveyance proxy
            if (w < 0.0) w = 0.0;
            else if (w > 1.0) w = 1.0;
            w = std::pow(w, 0.75);
            const double x = cfg.orgX + (double(i) + 0.5) * cfg.cellM;
            const double decay = (x <= 800.0) ? 1.0 : std::exp(-(x - 800.0) / 420.0);
            const double uc = mag * w * decay;
            u[row + i] = float(uc * sinD);
            v[row + i] = float(uc * cosD);
        }
    }
}

// The solve: proofs/wave_field.py build_spectrum() + component() + dispersion_k(), one pass
// per component (rows parallel; the gauge cumsum sequential -- cheap), then the TOTAL-Hs
// limiter (ONE uniform factor on the rms envelope; per-line caps measured 6.8x over in the
// shallows, ALGEBRA.md) and the truncating quantizer into the 2-wide slice atlas.
void Solve(const Inputs& in, std::vector<uint8_t>& atlas, WaveField::GpuTable& table) {
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

    // -- per-component solve ---------------------------------------------------------------
    std::vector<double> kbuf(cells), phibuf(cells), feik(cells);
    std::vector<float> aRaw(size_t(nc) * cells);
    std::vector<double> sumsq(cells, 0.0), sumA(cells, 0.0);
    std::vector<double> eta(cells, 0.0);   // --wave-map only: sum a cos(phi) at t = 0
    std::atomic<long long> blockedCells{0};
    const int icDiag = nc / 2;   // one component carries the phase-gauge instrument
    bool uploaded[WaveField::kMaxComp] = {};
    int nUp = 0;

    for (int ic = 0; ic < nc; ++ic) {
        const double T = in.tp * (fp / fr[ic]);
        const double sig = 2.0 * kPiW / T;
        const double prop = Wrap360(dirs[ic] + 180.0) * (kPiW / 180.0);   // travel TOWARD
        const double d0 = std::sin(prop), d1 = std::cos(prop);            // (east, north)
        const double c0 = kGrav * T / (2.0 * kPiW);
        const double cg0 = 0.5 * c0;
        // Snell vs the fixed bar-normal frame, PER-COMPONENT incidence -- the mean direction
        // gets 15 of 16 components wrong (ALGEBRA.md `wavefield`, fidelity-verified).
        const double th0 =
            (Wrap360((dirs[ic] + 180.0) - cfg.barNormalDeg + 180.0) - 180.0) * (kPiW / 180.0);
        const double cosTh0 = std::cos(th0), sinTh0 = std::sin(th0);
        const double a0 = aTot * wts[ic];
        float* aPlane = aRaw.data() + size_t(ic) * cells;

        ParallelRows(ny, [&](int j0, int j1) {
            long long nb = 0;
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
                    const double k = SolveDispersion(sig, hf, uopp, blocked);
                    if (blocked) ++nb;
                    const double c = (2.0 * kPiW / k) / T;
                    double kh = k * hf;
                    if (kh < 1e-4) kh = 1e-4;
                    else if (kh > 30.0) kh = 30.0;
                    const double nfac = 0.5 * (1.0 + (2.0 * kh) / std::sinh(2.0 * kh));
                    const double cg = nfac * c;
                    double cgEff = cg + along;
                    if (cgEff < kCgFloor) cgEff = kCgFloor;
                    const double Ks = std::sqrt(cg0 / cgEff);   // energy-flux shoaling
                    double sinT = (sinTh0 * c) / c0;
                    if (sinT > kSnellClip) sinT = kSnellClip;
                    else if (sinT < -kSnellClip) sinT = -kSnellClip;
                    const double cosT = std::cos(std::asin(sinT));
                    const double Kr =
                        std::sqrt(((cosTh0 > kKrCosFloor) ? cosTh0 : kKrCosFloor) /
                                  ((cosT > kKrCosFloor) ? cosT : kKrCosFloor));
                    const double ar = (a0 * Ks) * Kr;
                    kbuf[idx] = k;
                    aPlane[idx] = float(ar);
                    sumsq[idx] += ar * ar;   // rows are disjoint across threads
                    sumA[idx] += ar;
                }
            }
            blockedCells += nb;
        });

        table.sigma[ic] = float(sig);
        table.dirX[ic] = float(d0);
        table.dirZ[ic] = float(d1);

        const double lambdaDeep = 2.0 * kPiW * kGrav / (sig * sig);
        if (lambdaDeep < cfg.minSamplesPerLambda * cfg.cellM) continue;   // the fold gate
        uploaded[ic] = true;
        ++nUp;

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
        // datum is exact) and swept in the four diagonal orders twice. The update's own
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
            const int ib = (d0 >= 0.0) ? 0 : (nx - 1);      // the inflow corner
            const int jb = (d1 >= 0.0) ? 0 : (ny - 1);
            const double kInfC = (sig * sig) / kGrav;       // deep-water k of this component
            auto seed = [&]() {
                for (int i = 0; i < nx; ++i) {
                    phibuf[size_t(jb) * size_t(nx) + size_t(i)] =
                        kInfC * d0 * double(i - ib) * dxm;
                }
                for (int j = 0; j < ny; ++j) {
                    phibuf[size_t(j) * size_t(nx) + size_t(ib)] =
                        kInfC * d1 * double(j - jb) * dxm;
                }
            };
            seed();
            for (int pass = 0; pass < 8; ++pass) {          // two rounds of the four orders
                seed();
                const bool xr = (pass & 1) != 0, yr = (pass & 2) != 0;
                for (int jj = 0; jj < ny; ++jj) {
                    const int j = yr ? (ny - 1 - jj) : jj;
                    const size_t row = size_t(j) * size_t(nx);
                    for (int ii = 0; ii < nx; ++ii) {
                        const int i = xr ? (nx - 1 - ii) : ii;
                        const size_t idx = row + size_t(i);
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
                        if (cand < phibuf[idx]) phibuf[idx] = cand;
                    }
                }
            }
            seed();   // the last sweep must not be allowed to undercut it either
        }
        if (ic == icDiag) {
            // The instrument that says the walk was never salvageable: the SAME k field,
            // integrated by two legal routes to every cell, and the largest disagreement
            // between them. A phase is a phase; if these differ by more than a fraction of a
            // radian the quantity was not one.
            std::vector<double> pa(cells), pb(cells);
            for (int j = 0; j < ny; ++j) {   // north up the west edge, then east along rows
                const size_t row = size_t(j) * size_t(nx);
                pa[row] = (j > 0) ? (pa[row - size_t(nx)] +
                                     kbuf[row - size_t(nx)] * d1 * cfg.cellM)
                                  : 0.0;
                for (int i = 1; i < nx; ++i)
                    pa[row + i] = pa[row + i - 1] + kbuf[row + i - 1] * d0 * cfg.cellM;
            }
            for (int i = 0; i < nx; ++i) {   // east along the south edge, then north up cols
                pb[i] = (i > 0) ? (pb[i - 1] + kbuf[i - 1] * d0 * cfg.cellM) : 0.0;
                for (int j = 1; j < ny; ++j) {
                    const size_t idx = size_t(j) * size_t(nx) + size_t(i);
                    pb[idx] = pb[idx - size_t(nx)] + kbuf[idx - size_t(nx)] * d1 * cfg.cellM;
                }
            }
            // The residual is a DISTRIBUTION, and a max is the wrong number to quote: an
            // eikonal solution is allowed kinks. Where two ray families meet -- behind a
            // jetty, over a focusing shoal -- the viscosity solution takes first arrival and
            // creases, and a centred difference across a crease reads a gradient that is not
            // the local one. Those cells are the physics (that crease is a caustic), so the
            // percentiles say how much of the field is smooth and the max says how sharp the
            // sharpest crease is, and they are reported separately rather than blended into
            // one alarming number. Sampled only where all four neighbours are wet: across the
            // waterline the speed field steps from the wet k to the shoreline limit on
            // purpose, so a difference taken over that step measures the beach, not the wave.
            double gap = 0.0;
            std::vector<double> rs;
            rs.reserve(cells / 4);
            for (int j = 1; j < ny - 1; ++j) {
                const size_t row = size_t(j) * size_t(nx);
                for (int i = 1; i < nx - 1; ++i) {
                    const size_t idx = row + size_t(i);
                    if (h[idx] <= 0.0) continue;
                    gap = (std::max)(gap, std::abs(pa[idx] - pb[idx]));
                    if (h[idx - 1] <= 0.0 || h[idx + 1] <= 0.0 ||
                        h[idx - size_t(nx)] <= 0.0 || h[idx + size_t(nx)] <= 0.0) continue;
                    const double gx = (phibuf[idx + 1] - phibuf[idx - 1]) / (2.0 * cfg.cellM);
                    const double gz = (phibuf[idx + size_t(nx)] - phibuf[idx - size_t(nx)]) /
                                      (2.0 * cfg.cellM);
                    rs.push_back(std::abs(std::sqrt(gx * gx + gz * gz) - kbuf[idx]) /
                                 kbuf[idx]);
                }
            }
            std::sort(rs.begin(), rs.end());
            auto pct = [&](double q) {
                return rs.empty() ? 0.0 : rs[(std::min)(rs.size() - 1,
                                                        size_t(q * double(rs.size())))];
            };
            Log("[wave] phase gauge (comp %d, T %.1f s): the old walk disagreed with itself "
                "by %.0f rad (%.1f wavelengths) between two routes -- the phase it wrote was "
                "not a function of position. Eikonal |grad phi|/k residual over %zu open-water "
                "cells: p50 %.2f%%, p90 %.2f%%, p99 %.1f%%, max %.0f%% (the tail is the "
                "caustic creases, where first arrival is meant to fold)",
                ic, T, gap, gap / (2.0 * kPiW), rs.size(), pct(0.50) * 100.0,
                pct(0.90) * 100.0, pct(0.99) * 100.0, (rs.empty() ? 0.0 : rs.back()) * 100.0);
        }

        double kFieldMax = 0.0;
        for (size_t idx = 0; idx < cells; ++idx) kFieldMax = (std::max)(kFieldMax, kbuf[idx]);
        const double kMax = Ceil3(1.05 * kFieldMax);
        table.kMax[ic] = float(kMax);

        const int sx = (ic & 1) * nx, sy = (ic >> 1) * ny;
        ParallelRows(ny, [&](int j0, int j1) {
            for (int j = j0; j < j1; ++j) {
                const size_t arow = (size_t(sy + j) * aw + size_t(sx)) * 4;
                const size_t row = size_t(j) * size_t(nx);
                for (int i = 0; i < nx; ++i) {
                    uint8_t* px = &atlas[arow + size_t(i) * 4];
                    px[1] = Quant8(kbuf[row + i], kMax);
                    const double ph = phibuf[row + i];
                    px[2] = Quant8(std::cos(ph) * 0.5 + 0.5, 1.0);
                    px[3] = Quant8(std::sin(ph) * 0.5 + 0.5, 1.0);
                    eta[row + i] += double(aPlane[row + i]) * std::cos(ph);
                }
            }
        });
    }

    // -- the TOTAL-Hs limiter: Hs = 2 sqrt2 rms <= gammaHs * h, ONE uniform factor per cell
    //    on every a_i (spectral shape and directions survive); excess > 1 is the breaking
    //    indicator -- "does the sea here want to be taller than the water allows".
    std::vector<double> lim(cells);   // (and eta below, which carries the same factor)
    double rmsFieldMax = 0.0, sumFieldMax = 0.0;
    for (size_t idx = 0; idx < cells; ++idx) {
        const double rmsRaw = std::sqrt(sumsq[idx]);
        const double hLim = (h[idx] > 0.05) ? h[idx] : 0.05;
        const double rmsLim = (cfg.gammaHs * hLim) / (2.0 * std::sqrt(2.0));
        const double exc = rmsRaw / ((rmsLim > 1e-6) ? rmsLim : 1e-6);
        const double L = (std::min)(1.0, 1.0 / ((exc > 1e-6) ? exc : 1e-6));
        lim[idx] = L;
        rmsFieldMax = (std::max)(rmsFieldMax, rmsRaw * L);
        sumFieldMax = (std::max)(sumFieldMax, sumA[idx] * L);
    }
    const double envMax = Ceil3(1.05 * rmsFieldMax);
    const double sumMax = Ceil3(1.05 * sumFieldMax);
    table.envMax = float(envMax);
    table.sumMax = float(sumMax);

    {   // env slice: (rms/envMax, excess/2.5, sum/sumMax, spare)
        const int es = nc;
        const int sx = (es & 1) * nx, sy = (es >> 1) * ny;
        for (int j = 0; j < ny; ++j) {
            const size_t arow = (size_t(sy + j) * aw + size_t(sx)) * 4;
            const size_t row = size_t(j) * size_t(nx);
            for (int i = 0; i < nx; ++i) {
                const size_t idx = row + size_t(i);
                const double rmsRaw = std::sqrt(sumsq[idx]);
                const double hLim = (h[idx] > 0.05) ? h[idx] : 0.05;
                const double rmsLim = (cfg.gammaHs * hLim) / (2.0 * std::sqrt(2.0));
                const double exc = rmsRaw / ((rmsLim > 1e-6) ? rmsLim : 1e-6);
                uint8_t* px = &atlas[arow + size_t(i) * 4];
                px[0] = Quant8(rmsRaw * lim[idx], envMax);
                px[1] = Quant8(exc, kExcessMax);
                px[2] = Quant8(sumA[idx] * lim[idx], sumMax);
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
        "blocked %.3f%%, lvl %+.2f cur %+.2f",
        nx, ny, nc, secs, nUp, nc, cfg.minSamplesPerLambda * cfg.cellM,
        100.0 * double(blockedCells.load()) / (double(nc) * double(cells)), in.level,
        in.currentMs);
}

bool WriteCache(const char* path, uint64_t key, const Inputs& in,
                const std::vector<uint8_t>& atlas, const WaveField::GpuTable& table,
                bool withInputs) {
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
    hd.hs = in.hs;
    hd.tp = in.tp;
    hd.mwdDeg = in.mwdFromDeg;
    hd.cellM = in.cfg.cellM;
    hd.spreadDeg = in.cfg.spreadDeg;
    hd.barNormalDeg = in.cfg.barNormalDeg;
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
               std::vector<uint8_t>& atlas, WaveField::GpuTable& table) {
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
    Log("[wave] configured: %dx%d @ %.1f m (%.0f x %.0f m window), %d comps, buckets "
        "tide %.2f m / current %.2f m/s",
        cfg.nx, cfg.ny, cfg.cellM, cfg.nx * cfg.cellM, cfg.ny * cfg.cellM, cfg.nComp,
        cfg.tideBucketM, cfg.currentBucketMs);
}

uint64_t WaveField::BucketKey(double simUnix, const PartParam* parts, int nParts) const {
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
    mixD(m_cfg.orgX);
    mixD(m_cfg.orgZ);
    mixU(uint32_t(m_cfg.nx));
    mixU(uint32_t(m_cfg.ny));
    mixD(m_cfg.cellM);
    mixU(uint32_t(m_cfg.nComp));
    mixD(m_cfg.spreadDeg);
    mixD(m_cfg.barNormalDeg);
    mixD(m_cfg.gammaHs);
    mixD(m_cfg.minSamplesPerLambda);

    double latC = 0.0, lonC = 0.0;
    m_cfg.PlaceOfCell(0.5 * m_cfg.nx, 0.5 * m_cfg.ny, latC, lonC);
    const double levelRaw =
        m_atlas ? m_atlas->MslNavd(latC, lonC) + m_atlas->Level(latC, lonC, simUnix) : 0.0;
    mixD(std::round(levelRaw / m_cfg.tideBucketM) * m_cfg.tideBucketM);
    const double sRaw = (m_currents && m_actSta >= 0 &&
                         size_t(m_actSta) < m_currents->Count())
                            ? m_currents->SignedSpeed(size_t(m_actSta), simUnix)
                            : 0.0;
    mixD(std::round(sRaw / m_cfg.currentBucketMs) * m_cfg.currentBucketMs);
    mixBytes(&m_curSig, sizeof(m_curSig));   // the solved flow's content signature

    if (parts && nParts > 0) mixBytes(parts, sizeof(PartParam) * size_t(nParts));
    if (m_comp && m_hgtCh >= 0) {
        for (const HeightSource* hs : m_comp->ChannelAt(m_hgtCh).height) {
            mixBytes(hs->Info().name.data(), hs->Info().name.size());
            mixBytes("|", 1);
            mixBytes(hs->Info().structure.data(), hs->Info().structure.size());
        }
    }
    return hsh;
}

// M8 FLOWS INTO WAVES: read the SWE's solved current back (main thread -- GPU readbacks
// never ride the worker), resample onto the solve grid with the row-0-north flip, scale
// by the prism-truncation gain, QUANTIZE to 0.05 m/s, and hash the quantized bytes into
// the bucket key. The solve then eats exactly the bytes the key describes -- content
// identity, so the same flow always finds its cache and a changed flow always re-solves.
void WaveField::RefreshSweCurrent(Gpu& gpu, double simUnix) {
    if (!m_swe || !m_swe->Ready()) return;
    if (simUnix - m_curReadT < 90.0 && !m_curU.empty()) return;   // solve-cadence refresh
    if (m_inFlight.load()) return;   // never swap planes under a running solve
    m_curReadT = simUnix;
    std::vector<float> eta, uv4;
    uint32_t ew = 0, eh = 0, uw = 0, uh = 0;
    m_swe->ReadFields(gpu, eta, ew, eh, uv4, uw, uh);
    if (uw < 2 || uh < 2) return;
    const size_t cells = size_t(m_cfg.nx) * size_t(m_cfg.ny);
    m_curU.assign(cells, 0.0f);
    m_curV.assign(cells, 0.0f);
    // PHASE C4: each cell's place is its page texel's (WaveFieldConfig::PlaceOfCell), then the
    // solver's cell under it by the solver's own planes.
    const SweDomain& dom = m_swe->Domain();
    for (int j = 0; j < m_cfg.ny; ++j) {
        const size_t row = size_t(j) * size_t(m_cfg.nx);
        for (int i = 0; i < m_cfg.nx; ++i) {
            double la = 0.0, lo = 0.0, tx = 0.0, ty = 0.0;
            m_cfg.PlaceOfCell(double(i) + 0.5, double(j) + 0.5, la, lo);
            if (!dom.CellOf(la, lo, tx, ty) ||
                tx < 0.0 || ty < 0.0 || tx >= double(uw) || ty >= double(uh)) {
                continue;
            }
            const int ix = int(tx), iy = int(ty);   // row 0 = NORTH
            const float* s = &uv4[(size_t(iy) * uw + size_t(ix)) * 4];
            if (s[3] < 0.5f) continue;   // solver-invalid: sponge or dry
            // quantize to 0.05 m/s -- the sig hashes EXACTLY what the solve eats
            m_curU[row + i] =
                std::round(s[0] * m_sweGain / 0.05f) * 0.05f;
            m_curV[row + i] =
                std::round(s[1] * m_sweGain / 0.05f) * 0.05f;
        }
    }
    uint64_t h = 14695981039346656037ull;
    auto mixB = [&h](const void* p, size_t n) {
        const uint8_t* b = static_cast<const uint8_t*>(p);
        for (size_t i = 0; i < n; ++i) h = (h ^ b[i]) * 1099511628211ull;
    };
    mixB(m_curU.data(), m_curU.size() * sizeof(float));
    mixB(m_curV.data(), m_curV.size() * sizeof(float));
    if (h != m_curSig) {
        m_curSig = h;
        Log("[wave] swe current refreshed: sig %016llx (gain %.1f, 0.05 m/s buckets)",
            static_cast<unsigned long long>(h), m_sweGain);
    }
}

WaveField::Solved WaveField::SolveNow(uint64_t key, double simUnix,
                                      const std::vector<PartParam>& parts) const {
    wavecore::Inputs in;
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
    in.level = std::round(levelRaw / m_cfg.tideBucketM) * m_cfg.tideBucketM;

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

    // bed: the ONE composed height stack, sampled at texel centers at the cell's own
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
                        m_comp->SampleHeightStack(m_hgtCh, latR, lonR, m_cfg.cellM);
                }
            }
        });
    }
    if (!m_curU.empty() && m_curSig != 0) {
        // M8 flows-into-waves: the SWE's solved jet (quantized planes; their hash is
        // in this solve's key). The dispersion now sees the bent channel jet and the
        // shear off the tips -- the proxy could only point one way.
        in.u = m_curU;
        in.v = m_curV;
    } else {
        wavecore::BuildCurrentPlanes(m_cfg, in.level, in.currentMs, in.bed, in.u, in.v);
    }

    Solved out;
    out.key = key;
    wavecore::Solve(in, out.atlas, out.table);

    // v2 cache: the answer AND the question (input planes), so the offline checker can hold
    // this solve to the python proof without the engine. StoreCache below stays the
    // input-less writer for callers that only have a Solved.
    char path[128];
    CachePathFor(key, path, sizeof(path));
    wavecore::WriteCache(path, key, in, out.atlas, out.table, true);
    return out;
}

void WaveField::SolveAsync(uint64_t key, double simUnix, std::vector<PartParam> parts) {
    WaitForSolve();   // a finished, already-consumed solve
    m_inFlight.store(true);
    m_result = Solved{};
    m_solveRunning.store(true, std::memory_order_release);
    Log("[wave] bucket rolled -> background solve (key %016llx)",
        static_cast<unsigned long long>(key));
    // Lane::Compute, not Io: this runs for seconds and would otherwise hold one of the twelve
    // loader slots for the whole solve. Its own ParallelRows is a nested ParallelFor on the same
    // lane, which is safe because that caller does its own share of the chunks.
    Threads().Submit(Lane::Compute, "wave.solve", [this, key, simUnix, p = std::move(parts)]() {
        Solved s = SolveNow(key, simUnix, p);
        m_result = std::move(s);
        m_resultReady.store(true, std::memory_order_release);
        m_solveRunning.store(false, std::memory_order_release);
    });
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
        t.excMax *= e;
    }
    return t;
}

void WaveField::AdoptTable(const GpuTable& t) { m_table = DisplayTable(t); }

void WaveField::Offer(Solved&& s, const char* how) {
    const uint64_t key = s.key;
    m_nextKey = key;
    std::atomic_store(&m_next, std::shared_ptr<const Solved>(std::make_shared<Solved>(std::move(s))));
    char buf[160];
    snprintf(buf, sizeof(buf), "wave %ux%u key %016llx (%s, pages pending)", m_cfg.nx, m_cfg.ny,
             static_cast<unsigned long long>(key), how);
    stats = buf;
}

void WaveField::Publish() {
    std::shared_ptr<const Solved> n = Next();
    if (!n) return;
    AdoptTable(n->table);
    m_liveKey = n->key;
    std::atomic_store(&m_live, n);
    std::atomic_store(&m_next, std::shared_ptr<const Solved>());
    char buf[160];
    snprintf(buf, sizeof(buf), "wave %ux%u lvl %+.2f key %016llx (live)", m_table.nx, m_table.ny,
             double(m_table.level), static_cast<unsigned long long>(m_liveKey));
    stats = buf;
}

bool WaveField::LoadCache(uint64_t key, Solved& out) const {
    char path[128];
    CachePathFor(key, path, sizeof(path));
    if (!wavecore::ReadCache(path, key, m_cfg, out.atlas, out.table)) return false;
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
                       bool block) {
    if (!m_comp || m_hgtCh < 0 || !m_atlas) return false;
    RefreshSweCurrent(gpu, simUnix);   // flows into waves (throttled + inFlight-guarded)

    // hand over a finished background solve first: the old field stays live until the new
    // one is WHOLE (double-buffer by construction -- the renderer never sees a half solve).
    if (m_resultReady.load(std::memory_order_acquire)) {
        WaitForSolve();   // the job stores m_resultReady before it clears m_solveRunning
        m_resultReady.store(false);
        m_inFlight.store(false);
        const uint64_t solvedKey = m_result.key;
        Offer(std::move(m_result), "solved");
        PruneWaveCache(solvedKey);   // a solve just wrote ~112 MB; keep the budget
    }

    const uint64_t key = BucketKey(simUnix, parts, nParts);
    if (key != NextKey() && !m_inFlight.load()) {
        Solved s;
        if (LoadCache(key, s)) {
            // a cache hit is offered like a solve: its pages swap in, then it is live.
            s.key = key;
            Offer(std::move(s), "cache");
            Log("[wave] cache hit %016llx -- offered without a solve",
                static_cast<unsigned long long>(key));
        } else if (block) {
            // headless determinism: the dump frames must see the field, so eat the
            // ~3 s here, once per bucket. Interactive runs keep the async path.
            std::vector<PartParam> pcopy;
            if (parts && nParts > 0) pcopy.assign(parts, parts + nParts);
            Solved s2 = SolveNow(key, simUnix, pcopy);
            s2.key = key;
            Offer(std::move(s2), "solved now");
            PruneWaveCache(key);   // the blocking solve wrote a cache entry too
        } else {
            std::vector<PartParam> pcopy;
            if (parts && nParts > 0) pcopy.assign(parts, parts + nParts);
            SolveAsync(key, simUnix, std::move(pcopy));
        }
    }
    return m_inFlight.load();
}

WaveField::Probe WaveField::ProbeAt(double wx, double wz, double simUnix) const {
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
        eta += aC;                                   // eta = a cos(theta)   <- the reference
        dxS -= aS * dX;                              // D_h = -a sin(theta) d^   (chop = 1;
        dzS -= aS * dZ;                              //        WaterBank scales this by wfChop)
        sxS -= aS * k * dX;                          // grad eta = -a k sin(theta) d^
        szS -= aS * k * dZ;                          //   (the k is the term: a slope is 1/L)
        vxS += sig * aC * dX;                        // u = +sigma a cos(theta) d^  (crest
        vzS += sig * aC * dZ;                        //   water runs WITH the wave)
        vyS += sig * aS;                             // w = +sigma a sin(theta) == d(eta)/dt
        const double aKC = aC * k;                   // grad D_h = -a k cos(theta) d^ (x) d^
        jxxS -= aKC * dX * dX;
        jxzS -= aKC * dX * dZ;
        jzzS -= aKC * dZ * dZ;
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
    p.rms = float(bil(t.envSlice, 0) * double(t.envMax));
    p.excess = float(bil(t.envSlice, 1) * double(t.excMax));
    p.valid = true;
    return p;
}

}  // namespace ga
