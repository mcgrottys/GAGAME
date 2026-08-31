// ================================================================================================
//  WaveField - M8: THE SOLVED WAVE FIELD. The stationary wave boundary-value problem, solved
//  per cell over the inlet window and CACHED: per spectral component, a per-cell amplitude
//  (shoaling * refraction * the total-Hs limiter), wavenumber (current-Doppler finite-depth
//  dispersion, bracket+bisect with the BLOCKED mask), and integrated spatial phase carried as
//  a unit spinor (cos phi, sin phi). Time enters only as the rotor e^{-i sigma t} -- the same
//  statelessness the tide constituents have: the solve carries the state, the GPU carries the
//  phase. ALGEBRA.md `wavefield` is the derivation; proofs/wave_field.py is the authoritative
//  prototype (twin-tested to half an 8-bit LSB against the vqview-inlet reference bake); this
//  class must reproduce it number for number -- the hypervisor's step 9c holds it there.
//
//  Lifecycle: Update() watches the inputs' quantization buckets (forecast hour, tide level
//  0.25 m, ACT current 0.1 m/s) and kicks a background re-solve when a bucket rolls; the
//  packed result double-buffers, so the renderer keeps the old field until the new one is
//  whole. Solves cache to cache/wave/<fnv>.bin keyed on everything that touches the answer
//  (solver version, window, spectrum bytes, buckets, height-stack signature) -- the compositor's
//  identity-is-content law, one directory over.
//
//  Packing (the GPU contract): ONE R8G8B8A8_UNORM atlas, slices in a 2-wide grid --
//  slice s at texel origin ((s&1)*nx, (s/2)*ny). Slices 0..nUsed-1 = per-component
//  (a/aMax, k/kMax, cos*0.5+0.5, sin*0.5+0.5); slice kEnvSlice = (rms/envMax,
//  excess/2.5, sum/sumMax, spare). Components whose wavelength the grid undersamples
//  (lambda < kMinSamplesPerLambda * cell) are NOT uploaded -- the FFT cascades keep that
//  band, exactly the fold doctrine: a rung carries a band only while it resolves its phase.
// ================================================================================================
#pragma once

#include <atomic>
#include <cstdint>
#include <string>
#include <thread>
#include <vector>

#include "compose/Compositor.h"
#include "compose/WaterAtlas.h"
#include "sim/SeaState.h"
#include "sim/TideModel.h"

namespace ga {

class Gpu;

struct WaveFieldConfig {
    // The solve window, world metres (BathyModel frame: x east, z north, ACT0816 anchor).
    // Covers the throat, both jetties, the bar, and the ebb shoal; row 0 = SOUTH
    // (+v = +z = north, the patch.wrap family -- no flips into the bank kernel).
    double orgX = -1400.0, orgZ = -800.0;
    int nx = 1600, ny = 1000;
    double cellM = 2.0;
    int nComp = 16;                    // the reference construction (JONSWAP gamma=1)
    double spreadDeg = 26.0;
    double barNormalDeg = 285.0;       // the entrance bar's normal, compass
    double gammaHs = 0.60;             // Hs <= gammaHs * h (the total limiter)
    double minSamplesPerLambda = 8.0;  // upload gate: lambda >= this * cell
    double tideBucketM = 0.25;         // re-solve quantization
    double currentBucketMs = 0.10;
    float featherM = 120.0f;           // window edge blend into the cascades (scene cfg)
};

class WaveField {
public:
    static constexpr int kMaxComp = 16;
    static constexpr uint32_t kSolverVersion = 1;

    void Configure(const WaveFieldConfig& cfg, const Compositor* comp, int hgtChannel,
                   const WaterAtlas* atlas, const TideModel* tides, int entranceStation,
                   const CurrentModel* currents, int actStation);

    // Per frame, main thread. parts/n = the live GFS-Wave partition set (SeaLayer's).
    // Kicks a background solve when a bucket rolls; uploads + swaps when one finishes.
    // Returns true while a solve is in flight (consumers may show provenance).
    bool Update(Gpu& gpu, double simUnix, const PartParam* parts, int nParts);

    bool Ready() const { return m_srv != 0xFFFFFFFFu; }
    uint32_t Srv() const { return m_srv; }

    // The GPU contract, valid when Ready(): window georef + packing + per-component rows.
    struct GpuTable {
        float orgX, orgZ, invCell, feather;    // feather m (edge blend into cascades)
        uint32_t nx, ny, nUsed, envSlice;
        float sigma[kMaxComp];                  // rad/s
        float dirX[kMaxComp], dirZ[kMaxComp];   // unit propagation (east, north)
        float aMax[kMaxComp], kMax[kMaxComp];   // dequant scales (aMax 0 = slice unused)
        float envMax, sumMax, excMax, level;    // env scales + the level it solved at
    };
    const GpuTable& Table() const { return m_table; }

    // The hypervisor's step 9c: evaluate the SOLVED field at a world point on the CPU
    // (bilinear on the packed planes, spinor advanced by the same rotor the kernel
    // applies) -- eta plus per-component (a, k, phase) for the printout.
    struct Probe {
        bool valid = false;
        float eta = 0.0f, rms = 0.0f, excess = 0.0f;
        float a[kMaxComp] = {}, k[kMaxComp] = {}, phase[kMaxComp] = {};
    };
    Probe ProbeAt(double wx, double wz, double simUnix) const;

    std::string stats;   // provenance line for the title bar / report

private:
    struct Solved {                       // one finished solve (CPU side)
        std::vector<uint8_t> atlas;       // packed RGBA8, (2*nx) x (rows*ny)
        GpuTable table{};
        uint64_t key = 0;
    };

    uint64_t BucketKey(double simUnix, const PartParam* parts, int nParts) const;
    void SolveAsync(uint64_t key, double simUnix, std::vector<PartParam> parts);
    Solved SolveNow(uint64_t key, double simUnix, const std::vector<PartParam>& parts) const;
    bool LoadCache(uint64_t key, Solved& out) const;
    void StoreCache(const Solved& s) const;

    WaveFieldConfig m_cfg;
    const Compositor* m_comp = nullptr;
    int m_hgtCh = -1;
    const WaterAtlas* m_atlas = nullptr;
    const TideModel* m_tides = nullptr;
    int m_entranceSta = -1;
    const CurrentModel* m_currents = nullptr;
    int m_actSta = -1;

    GpuTable m_table{};
    uint32_t m_srv = 0xFFFFFFFFu;
    uint64_t m_liveKey = 0;

    // background solve plumbing: one worker at a time, result handed over by flag
    std::thread m_worker;
    std::atomic<bool> m_inFlight{false};
    std::atomic<bool> m_resultReady{false};
    Solved m_result;                     // written by worker, read by main after the flag
    // CPU copies for ProbeAt (the live field's planes)
    std::vector<uint8_t> m_cpuAtlas;

    void* m_tex = nullptr;               // GpuTexture*, owned (avoids Gpu.h include here)
};

}  // namespace ga
