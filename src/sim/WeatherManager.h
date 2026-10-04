// ================================================================================================
//  WeatherManager - M6x: THE GLOBAL WEATHER/PHYSICS MANAGER -- the Vatican client for water
//  and near-surface weather. ONE query surface over every product the monasteries publish:
//
//      Query(lat, lon, t, groundResM) -> { level, current, waves, wind, bed, depth }
//
//  each component answered by the FINEST RESIDENT RUNG that covers the point -- the texture
//  doctrine applied to physics. Stateless global fields carry the world (the tide atlas'
//  constituent rotors, GFS wind vectors, GFS-Wave Hs, GoMOFS surface currents); STATEFUL
//  solver windows refine exactly where they are resident (the Merrimack SWE; Boston Harbor
//  spins up when the camera arrives -- detail rises on zoom because RESIDENCY rises on zoom,
//  the same statement the tiled atlas makes about texels). Every component reports its
//  PROVENANCE: which source, which rung -- heterogeneity is queryable, not hidden.
//
//  "NOAA carries the state, the GPU carries the phase": the manager is overwhelmingly a
//  stateless evaluator; the solver windows are the sparse exception, and their absence is
//  not an error -- it is the tide plane being exactly right.
// ================================================================================================
#pragma once

#include "compose/Compositor.h"
#include "compose/WaterAtlas.h"
#include "core/Lattice.h"
#include "hal/Tenant.h"
#include "sim/BathyModel.h"
#include "sim/CurrentModel.h"
#include "sim/GlobeModel.h"
#include "sim/SeaState.h"
#include "sim/SweSolver.h"
#include "sim/TideModel.h"

#include <cmath>
#include <functional>
#include <memory>
#include <string>
#include <vector>

namespace ga {

class ResidencyManager;

struct WeatherSample {
    // water
    double levelNavd = 0;             // surface elevation (tide + solver deviation)
    float u = 0, v = 0;               // surface current, m/s east/north
    float hs = 0, tp = 0, dirDeg = 0; // waves: significant height, peak period, FROM deg
    float bedNavd = 0;                // the one bed (the composed height stack)
    float depthM = 0;                 // level - bed; <= 0 means dry here, now
    // air
    float windU = 0, windV = 0;       // 10 m wind, m/s
    // provenance: the rung that answered each component (static strings)
    const char* levelSrc = "-";
    const char* currentSrc = "-";
    const char* waveSrc = "-";
    const char* windSrc = "-";
    const char* bedSrc = "-";
    // The current is a solver's (SolverRefine), not the Gulf field's: the one current the bank
    // kernel has, so the one its wave-current gain may be fed on both processors.
    bool currentSolved = false;
};

class WeatherManager {
public:
    void Init(Compositor* comp, int heightChannel, const WaterAtlas* atlas,
              const TideModel* tides, const GlobeModel* globe, const SeaState* sea,
              const CurrentModel* currents);

    // A stateful window. External windows (the Merrimack: driven by the render loop) are
    // mirrored only; OWNED windows (Boston) are advanced by Update() and spun up lazily
    // when the camera enters their footprint below the activation altitude.
    void AddExternalWindow(const char* name, SweSolver* solver, const BathyModel* bathy,
                           std::function<double(double)> oceanAt);
    void AddDormantWindow(const char* name, BathyModel* bathy, const SweConfig& cfg,
                          std::function<double(double)> oceanAt, double spinupHours);
    // M9ar: the bed every OWNED solver binds at activation -- the height page tenant. There is
    // no per-window bed texture any more.
    // PHASE B2 (D4): the bed is the STANDING WINDOW (SurfaceFrame::StandAbout) at its slice; every
    // owned solver binds the same window (it reads it where the window holds its cells, the cube
    // elsewhere), and the pin and the wait ask for its tiles in its own uv.
    void SetHeightPage(hal::Resource heightArr, hal::Resource resMapArr, uint32_t mips,
                       const hal::BlockBinding& standing, const SweSolver::BedWindow& bed) {
        m_hgtArr = heightArr;
        m_hgtRes = resMapArr;
        m_hgtSlice = bed.slice;
        m_hgtMips = mips;
        m_stand = standing;
        m_bed = bed;
    }

    // Per frame (or before a physics batch): lazy activation and owned-solver advancement.
    // Nothing here reads the GPU back: the CPU mirrors refresh on demand (RefreshMirrorsTo).
    void Update(Gpu& gpu, ShaderCompiler& sc, const std::wstring& shaderDir, double simUnix,
                double camLatDeg, double camLonDeg, double camAltM);
    // M9ay: THE SOLVER DOMAINS STAY RESIDENT AT MIP 0 -- every ACTIVE window's lattice, on
    // the page it reads, every frame. The pin used to live in main for the Merrimack alone
    // (AUDIT_WATER item 6): an active Boston solver read slice 6 at whatever mip the camera
    // had left there, and with no globe layer nothing warmed the page for anyone.
    void PinDomains(ResidencyManager& res, int hgtTenant);
    // THE BED BEFORE THE HOUR (review finding 48). A spin-up integrates on the bed its kernel
    // reads, and the kernel reads the height tenant through the residency map; before the first
    // frame nothing of any domain has been asked for. This asks for the active windows' lattices
    // at mip 0 -- the pin's own rectangles, or the one window named -- and runs residency turns
    // on upload lists until the bed the kernel reads IS that bed. "Landed" is what the solver's
    // own bed trace reads (SweSolver::TraceBed), never the manager's claim alone: a
    // DirectStorage tile is Mapped before its bytes land (review finding 24), and the kernel
    // reads the residency map a turn uploads, not the one the manager holds. Bounded: a bed that
    // is not whole after kBedWaitMaxS is reported and the spin-up goes on over what there is.
    struct BedWait {
        uint32_t windows = 0;          // windows waited for (inside the page)
        uint32_t turns = 0;            // residency turns taken
        uint32_t tiles = 0;            // tiles mapped over the wait, every tenant
        uint64_t cells = 0, whole = 0; // lattice cells traced, and those reading the page at mip 0
        double seconds = 0.0;
        bool done = false;             // every window's bed read whole
    };
    BedWait WaitForBeds(Gpu& gpu, ResidencyManager& res, int hgtTenant,
                        const char* only = nullptr);
    static constexpr double kBedWaitMaxS = 60.0;
    // What the MANAGER claims over the active domains (or the one named): its own residency map,
    // the CPU copy, histogrammed by mip (15 = nothing). The kernel reads the GPU copy the turns
    // upload, so where the two disagree the bed trace (SweSolver::TraceBed) is the one to believe.
    // Returns the samples taken.
    uint64_t ClaimedMips(const ResidencyManager& res, int hgtTenant, uint64_t hist[16],
                         const char* only = nullptr) const;
    // Force a dormant window up (the probe harness; interactive uses the camera rule).
    bool Activate(Gpu& gpu, ShaderCompiler& sc, const std::wstring& shaderDir,
                  const char* name, double simUnix);

    // THE MIRROR REFRESHES ON DEMAND, NEVER ON A CLOCK (step 2 of docs/PERF_EXPERIMENT.md).
    // Query reads a CPU mirror of each active window's eta/uv banks, and its only callers
    // are the --ocean-probe harness (before the loop) and the --dump-water-state / --trace
    // exports (after it): a reader refreshes the mirrors ONCE to its instant, then queries.
    // A refresh is two full-field ReadbackTexture calls (a committed READBACK resource and
    // a WaitIdle drain each) plus 8.7 M half->float conversions. MEASURED on the storm rail
    // (out/step1/bench, --bench --gpu-time) while Update() still fired it every kMirrorDt =
    // 2 sim-s: an isolated (loop - render) stall of 17.9-27.2 ms (mean 23.2, n = 19) on
    // every frame 61k -- the bench's whole p99 (28.7 ms) and 19 of its 21 frames over
    // 16.7 ms -- feeding nothing the loop renders (no frame consumer of Query exists).
    // The contract is unchanged: a mirror is at most kMirrorDt behind the asking clock.
    // Every refresh logs its cost, so a per-frame reader added later announces itself.
    void RefreshMirrorsTo(Gpu& gpu, double simUnix);

    // ---- THE SOLVER IS TRUTH (the water match, step 1 -- Mark, 2026-09-15). Inside a solver's
    // domain the water's surface IS the solver's: the tide plane it was forced by plus its
    // deviation, over the solver's domain weight (SweSolver::DomainWeight) -- the same expression
    // the bank kernel draws (WaterBank.hlsl), from the same texels. The hull used to read the
    // atlas alone there, because the only CPU copy of the solver was a whole-field mirror that
    // costs 18-27 ms to read and nothing in play read it (measured by --water-probe: the drawn sea
    // stood +0.39 m above the hull's at the Merrimack helm). Now a consumer of the water at a place
    // ASKS for the solver's region there, every frame it wants the answer fresh (RequestRegion:
    // copied through the frame ring, delivered two frames later, never by a flush), and Query reads
    // the delivered texels. A place the solver owns and has not answered reports its level ABSENT
    // (levelSrc "-"), not the atlas: absence is not a substitute. The tools' whole-field mirror
    // (RefreshMirrorsTo) answers through the same reconstruction.
    void RequestRegion(double latDeg, double lonDeg, double radiusM);
    // The solver's refinement of one point, applied to a sample built without it: TreeWater
    // memoises the slow atlas sample per cell and refines each point it asks at, so the solver's
    // surface is continuous under a hull rather than quantised to the memo's cells.
    void SolverRefine(double latDeg, double lonDeg, double unixT, WeatherSample& s) const;
    // The instant the solver's answers are coherent at: the oldest of the active windows' newest
    // delivered answers (region or mirror); kNeverRead when no window has answered anything.
    double SolverAsOf() const;
    static constexpr double kNeverRead = -1.0e18;

    WeatherSample Query(double latDeg, double lonDeg, double unixT, double groundResM = 500.0,
                        bool refineBySolver = true) const;

    int ActiveWindows() const;
    // The planet's grids (the global wave grid WaveScale reads), or null.
    const GlobeModel* Globe() const { return m_globe; }
    std::string stats;   // "wx 2 windows (merrimack, boston)" for the title bar

private:
    hal::Resource m_hgtArr = nullptr;   // M9ar: borrowed from the residency manager
    hal::Resource m_hgtRes = nullptr;
    uint32_t m_hgtSlice = 6, m_hgtMips = 7;
    hal::BlockBinding m_stand{};      // PHASE B2: the standing window
    SweSolver::BedWindow m_bed{};     // ...and its rows, for every owned solver
    struct Window {
        std::string name;
        SweSolver* solver = nullptr;              // external, or owned.get()
        std::unique_ptr<SweSolver> owned;
        const BathyModel* bathy = nullptr;
        BathyModel* bathyMut = nullptr;       // dormant windows: realized from the channel
                                              // on activation (the one bed, lazily)
        std::function<double(double)> oceanAt;    // NAVD level clock for forcing + queries
        SweConfig cfg;
        double spinupHours = 0.5;
        bool active = false;
        // CPU mirrors
        std::vector<float> eta, uv4;
        uint32_t etaW = 0, etaH = 0, uvW = 0, uvH = 0;
        double mirrorT = -1.0e18;             // sim instant of the mirror; never = unread
        char levelTag[48] = {0};
        char currentTag[48] = {0};
    };
    static constexpr double kMirrorDt = 2.0;      // a mirror may lag the asking clock this much
    void ReadMirrors(Gpu& gpu, double simUnix, double maxAge);
    // A window's lattice as a rectangle of the height page's uv (the pin's, and the bed wait's);
    // false where it does not reach inside the page.
    // PHASE B2: the domain's rectangles in the standing window's uv (modulo 16384: one, two or four).
    int DomainRects(const BathyModel& b, float out[4][4]) const;
    static constexpr double kActivateAltM = 30000.0;
    bool m_pinLogged = false;

    const Window* WindowAt(double latDeg, double lonDeg) const;

    Compositor* m_comp = nullptr;
    int m_hgtCh = -1;
    const WaterAtlas* m_atlas = nullptr;
    const TideModel* m_tides = nullptr;
    const GlobeModel* m_globe = nullptr;
    const SeaState* m_sea = nullptr;
    const CurrentModel* m_currents = nullptr;
    std::vector<Window> m_windows;
};

}  // namespace ga
