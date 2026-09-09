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
#include <memory>
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
// Used by reference in Configure() and held as a pointer below. It was never declared here and
// the header only ever compiled because every existing includer happened to pull CurrentModel.h
// in first; including WaveField.h from a new file exposed that immediately.
class CurrentModel;

struct WaveFieldConfig {
    // The solve window, world metres (BathyModel frame: x east, z north, ACT0816 anchor).
    // Covers the throat, both jetties, the bar, and the ebb shoal; row 0 = SOUTH
    // (+v = +z = north, the patch.wrap family -- no flips into the bank kernel).
    double orgX = -1400.0, orgZ = -800.0;
    int nx = 1600, ny = 1000;
    double cellM = 2.0;
    int nComp = 32;                    // M9bp: see SceneConfig wfComps (16 was the comb)
    double spreadDeg = 26.0;
    double barNormalDeg = 285.0;       // the entrance bar's normal, compass
    double gammaHs = 0.60;             // Hs <= gammaHs * h (the total limiter)
    double minSamplesPerLambda = 8.0;  // upload gate: lambda >= this * cell
    double tideBucketM = 0.25;         // re-solve quantization
    double currentBucketMs = 0.10;
    float featherM = 120.0f;           // window edge blend into the cascades (scene cfg)
    float displayExag = 1.15f;         // M8g: display exaggeration, applied to the dequant
                                       // tables at ADOPTION (upload) -- the cache and the
                                       // bucket key stay raw physics; ProbeAt reads the
                                       // same scaled table, so 9b/9c agree with the GPU
    // --wave-map: dump the solved field as an image, straight off the solver's own grid.
    // NOT part of the answer, so NOT in BucketKey -- turning the instrument on must not
    // invalidate a cache. The camera, the mesh, the fold, the streaming and the lighting all
    // sit between the solve and the screen, and every one of them can hide or fake a
    // structure in the field; this is the one picture with none of them in it.
    std::wstring mapPath;
};

class WaveField {
public:
    // M9bl: 32, was 16. Sixteen components over a 26 deg spread are 1.6 deg apart, and a
    // sum of sixteen long-crested trains that close together is a FIXED interference
    // lattice -- the regular comb of straight ridges down a storm face. A real sea is
    // irregular because its directional spectrum is continuous. Doubling the components
    // halves the angular step and pushes the superposition's repeat out of frame; the
    // cbuffer rows for 16..31 append at the END of BankCb (see WaterBank.hlsl gWaveSig2).
    static constexpr int kMaxComp = 32;
    // M9bv: 3, was 2. The fan is 8 frequencies x 4 directions rather than 32 x 1 -- same
    // component count and the same atlas layout, but a different point of S(f, theta) per
    // component, so every cached solve on disk is a different field and must be re-solved.
    // M9bw: 8, was 6 (7 was a discarded single-path test). The stored phase is the
    // EIKONAL solution of |grad phi| = k rather than a line integral of k*d^ along some
    // chosen path -- that integral was path-dependent (the
    // field has curl wherever grad k is not parallel to d^), so it printed the integration
    // route into the sea as banding. Every cached solve carries the old, walked phase.
    static constexpr uint32_t kSolverVersion = 9;

    void Configure(const WaveFieldConfig& cfg, const Compositor* comp, int hgtChannel,
                   const WaterAtlas* atlas, const TideModel* tides, int entranceStation,
                   const CurrentModel* currents, int actStation);

    // M8 FLOWS INTO WAVES (the user's priority, verbatim: "flows affect waves"): when
    // the SWE solver is resident its SOLVED current -- the bent jet, the shear off the
    // tips -- drives the dispersion instead of the climatological proxy. The planes are
    // read back on the main thread at a quantized cadence, resampled onto the solve
    // grid (row-0-north flip declared), scaled by the prism-truncation gain, and their
    // quantized bytes join the bucket key: content identity, not a clock.
    void SetSweCurrent(class SweSolver* swe, const class BathyModel* bathy, float gain) {
        m_swe = swe;
        m_sweBathy = bathy;
        m_sweGain = gain;
    }

    // Per frame, main thread. parts/n = the live GFS-Wave partition set (SeaLayer's).
    // Kicks a background solve when a bucket rolls; uploads + swaps when one finishes.
    // block = true (headless renders): solve SYNCHRONOUSLY on a key roll instead --
    // a deterministic dump must never race a background solve (the flat-helm catch:
    // short runs sampled the bank before the field landed).
    bool Update(Gpu& gpu, double simUnix, const PartParam* parts, int nParts,
                bool block = false);


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

    // M9bc: the field is a TREE NODE now (WaveFieldSource) -- the solver keeps the solve on the
    // CPU as an immutable snapshot the painting threads read; there is no GPU atlas here.
    struct Solved {                       // one finished solve (CPU side)
        std::vector<uint8_t> atlas;       // packed RGBA8, (2*nx) x (rows*ny)
        GpuTable table{};
        uint64_t key = 0;
    };
    std::shared_ptr<const Solved> Live() const { return std::atomic_load(&m_live); }
    uint64_t LiveKey() const { return m_liveKey; }
    bool Ready() const { return Live() != nullptr; }

    // The hypervisor's step 9c AND the water a hull floats in: evaluate the SOLVED field at
    // a world point on the CPU (bilinear on the packed planes, spinor advanced by the same
    // rotor the kernel applies).
    //
    // WHY IT RETURNS MORE THAN eta.  A floating body asks the surface four questions and eta
    // answers only one.  It needs the NORMAL, because buoyancy acts along the local surface
    // normal -- the free surface is an equipotential of the effective gravity, so that IS the
    // first-order law and world-up is the approximation.  It needs the WATER VELOCITY,
    // because every drag element works on velocity RELATIVE to the fluid, and a hull sitting
    // in a swell is being dragged by orbital motion it would otherwise be blind to.  And it
    // needs the horizontal GERSTNER OFFSET, because the map from query point to surface point
    // is (x, z) -> (x + dx, eta, z + dz) and is NOT the identity: the station that wants the
    // water under a hull point must iterate, and cannot iterate on a map it cannot see.
    //
    // None of the three is new physics.  All three are built from what the loop already holds
    // -- Table().sigma[c], dirX/dirZ[c], and the per-component a, k and phase it computes to
    // get eta at all -- for a multiply each and not one extra transcendental.
    //
    // THE SIGN LEDGER, every entry derived from the ONE line that was already in the loop,
    // `p.eta += a * ca`, and cross-checked against shaders/WaterBank.hlsl's solved-field
    // block (search THE SOLVED WAVE FIELD) which is the GPU doing the same sum:
    //
    //   theta.  ca = cos(phi)cos(sigma t) + sin(phi)sin(sigma t) = cos(phi - sigma t), so the
    //           accumulated eta = sum a*cos(theta) with theta = phi - sigma*t.  That is the
    //           FRAME LEDGER's rotor (WaveField.cpp, `time`) and WaterBank's cT verbatim.
    //           EVERY sign below is a consequence of that one convention: had the engine
    //           written theta = sigma*t - phi, eta would be identical (cos is even) and the
    //           velocity would flip.  This is why the derivation starts here and not from a
    //           textbook.
    //   d/dt.   d(theta)/dt = -sigma.  (The minus is the whole sign question for v.)
    //   grad.   grad(theta) = grad(phi) = k*d^.  This is the WKB reading -- a, k and d^ vary
    //           on the bathymetry's scale, not the wavelength's -- and it is the same
    //           statement ALGEBRA.md `wavefield` makes when it says each component WANTS
    //           grad(phi) = k*d^.  M9bw: the stored phase now realises it as far as it CAN
    //           be realised -- phi is the eikonal solution of |grad phi| = k, so |grad phi|
    //           is right everywhere and grad phi is curl-free by construction.  What it is
    //           NOT is k times the deep-water d^: the direction of grad phi refracts, which
    //           is the whole point (see the phase-gauge block in the .cpp).  So sz below,
    //           built from the fixed d^, is the LOCAL plane wave's slope in the UNREFRACTED
    //           direction -- the one a normal wants and the one the caustics derivation
    //           uses -- and it departs from the finite difference of the stored phase by
    //           exactly the refraction angle.  proofs/wavefield_probe.py measures that gap
    //           on a real cached solve.
    //
    //   eta = a cos(theta)                                   (the line above, unchanged)
    //   D_h = -a sin(theta) d^        <- ALGEBRA.md `caustics`: the Gerstner displacement is
    //         a rotor, R(a*yhat)R~ with R = exp(-B*theta/2), B = d^^yhat, and expanding gives
    //         the (a cos theta)yhat - (a sin theta)d^ pair.  WaterBank ships exactly this:
    //         `dW.xz -= gWaveB.z * wF * aW * sT * dir2` is -chop*a*sin(theta)*d^.  Confirmed
    //         a second, independent way below: it is the time-integral of the velocity.
    //   grad eta = -a k sin(theta) d^  <- chain rule on eta through grad(theta) = k d^.  The
    //         k factor is the entire content of the term (a slope is 1/length); dropping it
    //         leaves a quantity that is not dimensionless and is wrong by a factor of k,
    //         which for this window's bands is 0.02..1.3.
    //   u   = +sigma a cos(theta) d^,  w = +sigma a sin(theta)   <- d(D)/dt at fixed LABEL,
    //         using d(theta)/dt = -sigma: d/dt(-a sin theta) = +sigma a cos theta, and
    //         d/dt(a cos theta) = +sigma a sin theta.  Two independent confirmations: (1) w
    //         is then exactly d(eta)/dt, which is the linearised kinematic free-surface
    //         condition, and (2) deep-water linear theory has u = A sigma e^{kz} cos(kx-wt),
    //         w = A sigma e^{kz} sin(kx-wt), which at z = 0 is this with theta = kx - wt --
    //         the same theta the rotor defines.  Physically: crest water (theta = 0) moves
    //         WITH the wave, trough water against it.  proofs/wavefield_probe.py measures all
    //         three relations, each against a deliberately broken variant.
    //
    // TWO DELIBERATE DIFFERENCES FROM WHAT THE GPU DRAWS, stated so nobody has to find them:
    //
    //   * NO CHOP.  dx/dz are the physical Gerstner offset, chop = 1.  WaterBank multiplies
    //     its dW.xz by gWaveB.z, which is SceneConfig::wfChop (default 1.1) -- a DISPLAY
    //     constant that lives in the scene config and never enters WaveField, so this class
    //     cannot honestly apply it.  A caller that must ride exactly what is drawn scales dx,
    //     dz (and, if it builds the Jacobian, that too) by the same wfChop.  eta, the slope
    //     and the velocity are untouched by it: WaterBank scales only dW.xz, exactly as
    //     OceanCpu's ledger records for the cascades' lambda.
    //   * NO DISPLAY SHAPING.  The swell-shadow `expo`, the per-ring band-limit `wF` and the
    //     window blend `wWin` are the bank kernel's, applied per texel at shading resolution.
    //     ProbeAt answers the one question the kernel cannot: what IS the solved field here.
    //     (This was already true of eta, rms and excess; it is now true of six more fields.)
    //
    // AND THREE CAVEATS ON THE SLOPE, all three MEASURED on a real cached solve rather than
    // asserted (proofs/wavefield_probe.py PART C -- 1920x1152 window, Hs 3.0 m, 64 comps):
    //
    //   1. LABEL SPACE, not the displaced surface.  sx/sz are d(eta)/dx as the struct says.
    //      The surface a boat touches is the DISPLACED one, whose geometric slope is grad(eta)
    //      run through (I + J)^-1 with J = -chop * sum a k cos(theta) (d^ (x) d^) -- ALGEBRA.md
    //      `caustics`, the symmetric displacement Jacobian.  |J| ~ a*k is the steepness, so the
    //      correction is percent-level in swell and tens of percent in a steep bar sea.  Left
    //      to the caller because the caller owns chop, and because every term J needs is public
    //      here already: a[], k[], and Table().dirX/dirZ.
    //   2. WKB: grad(a) is dropped.  Against a finite difference of the field the atlas encodes,
    //      the x slope lands 10.3% off, and adding the dropped (da/dx)cos(theta) term back
    //      removes 76% of that -- so the WKB reading costs ~8% of |grad eta| and the remaining
    //      2.5% is the 8-bit atlas floor.  The term is not recoverable here: a's spatial
    //      derivative is not in the packed planes, and differencing the quantized bytes for it
    //      would cost more noise than it removes.
    //   3. THE Z LEG IS NOT THE ATLAS'S Z GRADIENT, and this one is a real disagreement rather
    //      than an error bar.  phi is a PER-ROW cumsum along x, so d(phi)/dz collects the
    //      accumulated row-to-row difference in k and grows with distance from the west anchor:
    //      measured |d(phi)/dz| runs 0.034 -> 0.44 rad/m from 266 m to 1841 m east, against a
    //      |k*d^_z| of 0.010..0.025 rad/m.  sz is the LOCAL PLANE WAVE slope -- the physics, and
    //      what a normal wants -- while the atlas's z gradient is the documented gauge choice
    //      ("a single reference column would print its depth profile as horizontal bands").
    //      They are different objects.  Which one a HULL should ride cannot be settled by a
    //      proof; it needs a render, and it has not had one.
    struct Probe {
        bool valid = false;
        float eta = 0.0f, rms = 0.0f, excess = 0.0f;
        float dx = 0.0f, dz = 0.0f;              // horizontal (Gerstner) displacement, m
        float sx = 0.0f, sz = 0.0f;              // surface slope d(eta)/dx, d(eta)/dz (-)
        float vx = 0.0f, vy = 0.0f, vz = 0.0f;   // water particle velocity at the surface, m/s
        float a[kMaxComp] = {}, k[kMaxComp] = {}, phase[kMaxComp] = {};
    };
    Probe ProbeAt(double wx, double wz, double simUnix) const;

    std::string stats;   // provenance line for the title bar / report

private:

    uint64_t BucketKey(double simUnix, const PartParam* parts, int nParts) const;
    void AdoptTable(const GpuTable& t);   // m_table = t with the display closure applied
    // The closure ITSELF, so the two readers of a solve cannot drift: AdoptTable bakes it
    // into m_table for the upload path, ProbeAt applies it to its own snapshot's table.
    GpuTable DisplayTable(const GpuTable& raw) const;
    void RefreshSweCurrent(Gpu& gpu, double simUnix);
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
    class SweSolver* m_swe = nullptr;             // M8: the real flow (optional)
    const class BathyModel* m_sweBathy = nullptr;
    float m_sweGain = 3.2f;                       // prism-truncation magnitude restore
    std::vector<float> m_curU, m_curV;            // solve-grid planes, main-thread owned
    uint64_t m_curSig = 0;                        // quantized content hash -> bucket key
    double m_curReadT = -1e18;                    // last refresh (sim s)

    GpuTable m_table{};
    std::shared_ptr<const Solved> m_live;   // M9bc: the adopted solve, swapped atomically
    uint64_t m_liveKey = 0;

    // background solve plumbing: one worker at a time, result handed over by flag
    std::atomic<bool> m_solveRunning{false};   // the pool job is live (replaces joining m_worker)
    void WaitForSolve();   // what m_worker.join() used to do; main thread only
public:
    // A solve in flight holds `this`. There was no destructor at all before, and ~std::thread
    // on a joinable thread calls std::terminate -- so a teardown mid-solve was a crash.
    ~WaveField() { WaitForSolve(); }
private:
    std::atomic<bool> m_inFlight{false};
    std::atomic<bool> m_resultReady{false};
    Solved m_result;                     // written by worker, read by main after the flag
    // CPU copies for ProbeAt (the live field's planes)


};

}  // namespace ga
