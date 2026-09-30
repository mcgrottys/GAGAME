// ================================================================================================
//  SweSolver - M5c: the sparse shallow-water solver. The estuary stops being a picture.
//
//  A virtual-pipe SWE (Mei-style: per-cell outflow fluxes down the water-surface gradient,
//  volume-limited, damped) runs over the CUDEM bathymetry grid, its state held in TWO
//  TileAtlas2D grade banks -- eta (R32F) and the flux quadruple (RGBA32F) -- resident only over
//  tiles that can ever be wet. The eta bank stores the DEVIATION from the analytic tide plane,
//  so a NULL tile means "the stateless model is exactly right here": the offshore sponge pins
//  the deviation to zero (which IS the tidal forcing -- the basin fills and drains against a
//  boundary riding the real CO-OPS clock), the river discharge pushes it up from the west, and
//  everything in between -- the throat jet, the basin's lag, the ebb/flood asymmetry -- emerges.
//
//  Coupling out: a dense RGBA16F surface-current texture (u east, v north, |U|, valid) derived
//  from the fluxes each frame. The sea shader samples it INSTEAD of the analytic Gaussian jet
//  wherever it is valid, which hands the wave-current blocking physics a current field with the
//  real channel's shape; and the eta bank rides under the FFT waves as the local mean surface.
//
//  Time: the solver advances its own clock toward the scene clock, at most kMaxSubsteps per
//  frame. If the scene clock runs away (fast time scales, scrubs), the state is reset to the
//  analytic plane -- the same "memory of a time that no longer exists" policy the churn uses.
// ================================================================================================
#pragma once

#include "core/GradeField.h"
#include "core/Lattice.h"
#include "hal/Context.h"
#include "hal/Gpu.h"
#include "hal/Readback.h"
#include "hal/Shader.h"
#include "hal/TileAtlas.h"
#include "hal/Views.h"
#include "sim/BathyModel.h"

#include <algorithm>
#include <cmath>
#include <functional>
#include <string>
#include <vector>

namespace ga {

// M6x: the solver is a WINDOW TYPE now, not the Merrimack. Per-window knobs that used to be
// hardcoded Merrimack-isms; defaults preserve the validated estuary behavior.
struct SweConfig {
    const char* name = "merrimack";
    float spongeX0 = 1400.0f;   // world-x where the offshore sponge ramps in (open sea east)
    bool westBoundary = true;   // the Flather river boundary (Boston's rivers are dammed:
                                // off, and the west edge is a wall like any land)
};

// A SOLVER'S WINDOW HOLDS ITS WATER (an instrument, said once where a window is made): the grid
// flooded 4-connected (the kernel's faces) through cells whose bed is under `level`, from its open
// boundaries -- the sponge's cells (world x past spongeX0) and the west face's wet-capable cells
// (column 0, bed under 2 m, when the window has a west boundary). Water the sponge's flood does not
// reach is listed, largest first, with where it meets the window's edge. `plantCrossEdge` lets the
// flood run along the outside of an edge it reaches: the selftest's plant, a cut channel made whole.
struct WaterPiece {
    struct EdgeRun {
        char edge;      // 'N', 'S', 'W', 'E'
        int from, to;   // cells along the edge: columns for N and S, rows for W and E
        float deepest;
    };
    int id = -1;                          // its label in the flood's label map
    int cells = 0;
    int c0 = 0, r0 = 0, c1 = 0, r1 = 0;   // its box, cells
    bool face = false;                    // the west face reaches it (so it is held by the face alone)
    std::vector<EdgeRun> runs;            // where it meets the window's edge
};
struct WaterHold {
    int joined = 0;      // cells the sponge's flood reaches
    int faceCells = 0;   // the west face's seeds
    std::vector<WaterPiece> pieces;
    std::vector<WaterPiece::EdgeRun> crossings;   // the sponge's water at the edge, not a boundary
    std::vector<uint8_t> sea;                     // by label: 1 = the sponge's flood reaches it
};
WaterHold FloodWindow(const float* bed, int nx, int ny, float level, int spongeCol0, bool westFace,
                      bool plantCrossEdge = false, std::vector<int>* labels = nullptr);
// The two beds' floods side by side: the cells the grid's joins that the kernel's does not (and
// back), the largest piece of the kernel's water the grid's joins, and where it parts from the
// sea -- the cell of the lowest crest on its way there over the kernel's bed, and that cell's bed
// in each.
struct FloodParting {
    int gridJoined = 0, kernelJoined = 0;   // the cells each bed's flood joins to the sponge
    int gridOnly = 0, kernelOnly = 0;
    WaterPiece piece;       // the kernel's piece (cells = those of it the grid joins); id -1 = none
    int crestCell = -1;
    float crestKernel = 0.0f, crestGrid = 0.0f;
};
FloodParting CompareFloods(const float* grid, const float* kernel, int nx, int ny, float level,
                           int spongeCol0, bool westFace);
void LogWindowHoldsWater(const BathyModel& bathy, const SweConfig& cfg, const char* when,
                         const std::vector<float>* kernelBed = nullptr);
bool RunWaterHoldSelfTest();

class SweSolver {
public:
    static constexpr uint32_t kMaxSubsteps = 16;

    // M9ar: bind the bed -- slice `slice` of the height PAGE tenant's array, with its residency
    // map, so the solver reads the same megatexture the water shading and the globe read, at
    // whatever mip is resident. Must be called before the first Step; there is no bed otherwise.
    // M12 step 4b: `window` is the z14 lattice the page sits on (the surface's winH); its
    // Rows() are the kernel's winA row.
    void SetHeightPage(Gpu& gpu, hal::Resource heightArr, hal::Resource resMapArr,
                       uint32_t slice, uint32_t mips, const Lattice& window);
    bool BedBound() const { return m_bedBound; }

    void Init(Gpu& gpu, ShaderCompiler& sc, const std::wstring& shaderDir,
              const BathyModel& bathy,
              const SweConfig& cfg = {});
    bool Ready() const { return m_ready; }

    // Boundary targets, as DEVIATIONS from the ocean tide plane (NAVD m). West = the
    // station-interpolated river tide (the truncated upriver prism arrives through the M1
    // fits); south = the sound's tide where the window cuts its real Ipswich entrance (M6d).
    // M6r: westQm3s is the TRANSPORT the Flather boundary must carry (+east; river discharge
    // minus the upriver prism demand) -- radiation alone cannot supply a prescribed prism,
    // it would throttle behind the standing dEta it needs to sustain the flow. Update per
    // frame.
    void SetBoundaries(float westDEtaM, float southDEtaM, float westQm3s = 0.0f) {
        m_westDEta = westDEtaM;
        m_southDEta = southDEtaM;
        m_westQ = westQm3s;
    }

    // Advance toward simUnix (records compute through cmd) and leave eta + uv sampleable.
    // tideNavd = the analytic water level, NAVD88 m. Returns substeps executed this frame.
    int Record(hal::CommandContext& cmd, Gpu& gpu, double simUnix, float tideNavd);

    // Solver-only advancement (own submits; no rendering): integrate up to targetUnix in
    // batches of 64 substeps per command list. tideAt(unix) supplies the ocean boundary level,
    // westAt/southAt the boundary deviations, westQAt the west transport (m^3/s, +east).
    template <typename F, typename G, typename H, typename Q>
    void AdvanceTo(Gpu& gpu, double targetUnix, F tideAt, G westAt, H southAt, Q westQAt) {
        while (m_simTime < targetUnix - m_dt) {
            const double target = (std::min)(m_simTime + 64.0 * m_dt, targetUnix);
            m_westDEta = static_cast<float>(westAt(m_simTime));
            m_southDEta = static_cast<float>(southAt(m_simTime));
            m_westQ = static_cast<float>(westQAt(m_simTime));
            hal::CommandContext up(gpu, gpu.BeginUpload(), hal::Owner::Upload);
            Record(up, gpu, target, static_cast<float>(tideAt(m_simTime)), 9999);
            gpu.EndUpload();
            gpu.ResetConstantArenaAfterIdle();   // thousands of batches; EndUpload waited
        }
    }

    // Spin-up: reset to the analytic plane `hours` before startUnix and integrate forward, so
    // the first rendered frame carries real basin history instead of a flat start.
    template <typename F, typename G, typename H, typename Q>
    void Spinup(Gpu& gpu, double startUnix, double hours, F tideAt, G westAt, H southAt,
                Q westQAt) {
        m_simTime = startUnix - hours * 3600.0;
        m_pendingReset = true;
        AdvanceTo(gpu, startUnix, tideAt, westAt, southAt, westQAt);
    }

    uint32_t EtaSrv() const { return m_eta.Srv(); }    // dEta from the tide plane, R32F
    uint32_t UvSrv() const { return m_uvSrv; }         // currents, RGBA16F (slice-0 view)
    uint32_t VelGradSrv() const { return m_velGrad.Srv(); }   // M9h: (div, curl), RG16F
    uint32_t VelGradResMapSrv() const { return m_velGrad.ResidencyMapSrv(); }
    uint32_t VelGradMips() const { return m_velGrad.MipCount(); }
    // M9i: the bank itself, so a COMPOSED page can be written into its coarse levels from
    // outside. The solver deliberately does not know where such a page came from -- that is
    // the compositor'''s business, and the last attempt failed by making this class know.
    GradeBank& VelGradBank() { return m_velGrad; }
    // The residency map is one texel per mip-0 TILE, so the lens indexes it in tile space.
    uint32_t VelGradResMapW() const { return m_velGrad.TilesX(); }
    uint32_t VelGradResMapH() const { return m_velGrad.TilesY(); }
    hal::Resource VelGradRes() const { return m_velGrad.Res(); }
    hal::Resource UvRes() const { return m_uvBank.Res(); }

    uint32_t Nx() const { return m_cb.nx; }
    float CellM() const { return m_cb.dx; }   // level-0 ground size, for a page ladder
    uint32_t Ny() const { return m_cb.ny; }
    uint32_t ResidentTiles() const { return m_eta.ResidentCount() + m_flux.ResidentCount(); }
    uint64_t ResidentBytes() const { return m_eta.ResidentBytes() + m_flux.ResidentBytes(); }
    float PadW() const { return static_cast<float>(m_eta.TilesX() * m_eta.TileW()); }
    float PadH() const { return static_cast<float>(m_eta.TilesY() * m_eta.TileH()); }

    // Synchronous full-texture readbacks for validation probes (world metres). Headless use.
    // The batch form shares ONE eta + ONE uv readback across all points.
    struct Probe {
        float dEta, u, v;
        bool valid;
    };
    void ReadProbes(Gpu& gpu, const float* xzPairs, int count, Probe* out);
    // M6x: the full-field CPU MIRROR for the weather manager -- eta (PADDED atlas dims) and
    // the derived currents (exact bathy dims, xyzw = u, v, speed, valid), one readback each.
    void ReadFields(Gpu& gpu, std::vector<float>& etaOut, uint32_t& etaW, uint32_t& etaH,
                    std::vector<float>& uv4Out, uint32_t& uvW, uint32_t& uvH);
    // Debug: raw readback of the flux bank (padded dims, RGBA32F). outW/outH = padded texels.
    std::vector<uint8_t> ReadFluxRaw(Gpu& gpu, uint32_t* outW, uint32_t* outH, uint32_t* outPitch);
    // The transport through the west boundary, m^3/s (+east): the Flather faces' flux summed down
    // the exterior column -- what the river actually brings in, beside what the boundary was told
    // to carry (SetBoundaries' westQ). A full readback of the flux bank: tools only.
    double WestTransport(Gpu& gpu);
    // THE WEST FACE, read back (an instrument): for each row of the exterior column, the Flather
    // face's terms as the flux kernel makes them for its next substep on this instant's state
    // (CsSweFlux compiled with SWE_WEST_TRACE: its own expressions, and no flux written). One row
    // per lattice row; `pinned` is the kernel's own test (the column is the exterior and its bed
    // below +2 m).
    struct WestFaceRow {
        float etaX, etaI, sill, hf;      // the exterior level, the first interior surface, the
                                         // face's sill and its depth (m NAVD, m)
        float uext, rad, ub, qRaw;       // u_ext, the radiation term, ub (m/s), q before the clamp
        float q, qLo, qHi, pinned;       // q after it (m^3/s, +east), the clamp's bounds, 1 = pinned
        float dEta0, hI, bed0, bed1;     // the exterior cell's eta, the interior depth, both beds
    };
    bool TraceWestFace(Gpu& gpu, std::vector<WestFaceRow>& rows);
    float WestArea() const { return m_westArea; }   // the live section u_ext was last divided by
    Probe ReadProbe(Gpu& gpu, float wx, float wz) {
        const float p[2] = {wx, wz};
        Probe out{};
        ReadProbes(gpu, p, 1, &out);
        return out;
    }

    // ---- THE BED THE KERNEL READS (the bed trace: an instrument, never on the frame path). The
    // solver's own BedAt over its whole lattice, through the rule and the residency map the flux
    // and height kernels read at this instant (Swe.hlsl CsSweBedTrace), read back with the mip
    // the rule read at and the slice it read. The CPU bed (BathyModel) is a different reader of
    // the same stack and cannot see what residency did to this one. `floorMip` raises the rule's
    // residency floor for this read alone (the planted failure: the coarsest mip). It records on
    // its own upload list and waits, so it is never called inside a frame's recording.
    struct BedTrace {
        uint32_t nx = 0, ny = 0;
        std::vector<float> bed;       // m NAVD88, row 0 north, nx * ny
        std::vector<uint8_t> mip;     // the mip the rule read at
        std::vector<uint8_t> slice;   // the slice it read: a cube face 0..5, or the page's
    };
    bool TraceBed(Gpu& gpu, BedTrace& out, float floorMip = 0.0f);
    // A SOLVER'S WINDOW HOLDS ITS WATER, said once: `traced` = the whole-bed wait finished, so the
    // trace reads the kernel's bed back and the line floods that (and the CPU grid's flood beside
    // it where the two differ); otherwise the CPU grid, and the line says the kernel's may differ.
    void LogHoldsWater(Gpu& gpu, bool traced);
    uint32_t PageSlice() const { return static_cast<uint32_t>(m_cb.pageB[0]); }

    std::string stats;         // "swe 476t 31/54MB 16ss" for the title bar
    double SimTime() const { return m_simTime; }
    float Dt() const { return m_dt; }

    // ---- THE SOLVER, READ WITHOUT A STALL (the water match, step 1 -- Mark, 2026-09-15: the
    // solver is truth). A hull reads the surface the solver holds, not the tide plane it was forced
    // by, and it may not stop the renderer to do it (Gpu::WaitIdle is 18-27 ms: WeatherManager.h).
    // So a consumer ASKS for a region -- the eta and current texels within radiusM of a point of
    // the flat frame, plus the one-texel apron the bilinear reconstruction reads -- and Record, on
    // the frame ring only, copies it through hal::RegionReadback; the answer arrives
    // Gpu::kFrameCount frames later, always, and never by a flush. A request lives one frame: a
    // consumer that wants its answer kept fresh asks every frame (the answers stay until newer
    // ones replace them). The upload-list paths (Spinup, AdvanceTo) serve nothing.
    void RequestRegion(double worldX, double worldZ, double radiusM);
    // Someone asked for a region this frame: the solver is wanted whether or not any view draws it
    // (SeaLayer::Simulate steps it on this as well as on the drawn sea).
    bool Demanded() const { return !m_requests.empty(); }
    struct Deviation {
        float dEta = 0.0f;           // m, from the tide plane the solver was forced by
        float u = 0.0f, v = 0.0f;    // the solved surface current, m/s east / north
        bool currentValid = false;   // the solver has a current there (its valid channel > 0.5)
        double asOf = 0.0;           // the solver's clock when the texels were copied
    };
    // THE KERNEL'S RECONSTRUCTION, at a point of the flat frame (shaders/WaterBank.hlsl: texel =
    // (u Nx, (1 - v) Ny), centres at -0.5, bilinear, clamped to the grid), from the newest
    // delivered region holding all four texels. False where no delivered region holds them.
    bool DeviationAt(double worldX, double worldZ, Deviation& out) const;
    // The same reconstruction over whole-field arrays (the tools' mirror: eta at rows of etaRowW,
    // the current's four channels at rows of uvRowW), so the two transports cannot disagree about
    // the law. False when the arrays are empty.
    bool DeviationFromFields(double worldX, double worldZ, const std::vector<float>& eta,
                             uint32_t etaRowW, const std::vector<float>& uv4, uint32_t uvRowW,
                             double asOf, Deviation& out) const;
    // THE DOMAIN'S WEIGHT: the solver's surface owns a point in proportion to how far inside the
    // grid it stands, rising from 0 at the grid's edge to 1 one cell in -- the field is defined at
    // cell centres and has no support of its own nearer the edge than that. The kernel's
    // expression, in the kernel's texel units.
    double DomainWeight(double worldX, double worldZ) const;
    // The newest delivered region's clock; a large negative number when none has been delivered.
    double DeliveredAsOf() const { return m_regions.empty() ? -1.0e18 : m_regions.front().asOf; }

private:
    int Record(hal::CommandContext& cmd, Gpu& gpu, double simUnix, float tideNavd,
               int maxSub);
    void RecordReset(hal::CommandContext& cmd, Gpu& gpu);
    // M12 step 4b instrument: the [kernel] swe cb fingerprint, logged at every upload when it
    // changes (the gate for the lattice-row moves and the fills of 4e/4f).
    void LogCbFingerprint();

    // Mirrors SweCb in shaders/Swe.hlsl exactly.
    struct SweCbData {
        uint32_t nx, ny, etaTilesX, fluxTilesX;
        uint32_t etaTileW, etaTileH, fluxTileW, fluxTileH;
        uint32_t listCount;
        float worldX0;
        float dy;   // M6r: north-south texel size -- the CUDEM grid is EQUIANGULAR, so
                    // dy = dlat*mPerLat (13.65 m) != dx = dlon*mPerLon (10.08 m)
        float westUext;   // M6r: Flather's u_ext (m/s, +east) = westQ / live west section area
        float dx, dt, damp, tideNavd;
        float spongeX0, spongeRate, riverDEta, gravity;
        float riverBox[4];
        // M6r: d(tideNavd)/dt. The eta bank stores deviation from a MOVING plane; the plane's
        // rise is booked as debt in every wet-capable interior cell so the prism must actually
        // ARRIVE through the boundaries (without it the basin filled by construction and the
        // gap carried only the deviation dynamics).
        float tideRate;
        float padA, padB, padC;
        // M9ar: THE BED IS THE HEIGHT MEGATEXTURE. Lattice texel -> lat/lon (this grid is
        // equiangular: lon0, lat1 (north edge), dlon, -dlat in degrees per texel), then the
        // Mercator page frame (org px x, org px y, 1/16384, world px at z14). Appended LAST.
        float geoLL[4];
        float winA[4];
        float pageB[4];   // M9ax: x = the z14 page's slice. APPENDED LAST (both sides).
        // M9h: the GoMOFS ingest rows. APPENDED AT THE END on both sides -- a same-size
        // insertion in the middle passes the byte-parity gate and silently offsets every later
        // row (the lesson WaterBank.hlsl:44 records, nearly repeated here).
    };

    bool m_ready = false;
    const BathyModel* m_bathy = nullptr;

    TileAtlas2D m_eta, m_flux;
    // M9h: grad(flow) -- residency derived from the Cayley closure, not a physics policy.
    GradeBank m_velGrad;
    ShaderCompiler* m_sc = nullptr;   // M9h: BuildChain compiles the reducer on first use
    std::wstring m_shaderDir;
    // M9ax: the derived currents as a Volatile GradeBank over the wet tiles -- the same
    // residency as eta -- with a TEXTURE2D view over slice 0 for the consumers that read a
    // plain texture (Sea.hlsl, SeaChurn.hlsl, WaterBank.hlsl). The dense committed RGBA16F
    // this replaces was the last flat texture on the per-frame water path (AUDIT_WATER item 4).
    GradeBank m_uvBank;
    uint32_t m_uvSrv = UINT32_MAX;
    hal::RootSignatureRef m_rs;
    hal::Pso m_clearEta, m_clearFlux, m_uvClear, m_fluxK, m_heightK, m_deriveK;
    hal::Pso m_velGradK;   // M9h: grad(flow) -> div + curl
    hal::Table m_table;   // [t1 height page, t2 its residency map, u0 eta, u1 flux, u2 uv, u3 mv]
    D3D12_RESOURCE_STATES m_etaState = D3D12_RESOURCE_STATE_UNORDERED_ACCESS;
    D3D12_RESOURCE_STATES m_uvState = D3D12_RESOURCE_STATE_UNORDERED_ACCESS;

    SweCbData m_cb{};
    double m_simTime = 0;
    float m_westDEta = 0;
    float m_southDEta = 0;
    float m_westQ = 0;                 // west transport target, m^3/s (+east)
    std::vector<float> m_westBed;      // exterior-column bed depths: the live section area
    SweConfig m_cfg;                   // as Init was given it (the holds-water line)
    bool m_bedBound = false;           // M9ar: SetHeightPage has run
    // The bed trace's own binding (TraceBed): the height tenant as SetHeightPage bound it, and the
    // kernel, table and target built on its first call -- nothing of it exists in a run without.
    hal::Resource m_bedArr = nullptr, m_bedRes = nullptr;
    uint32_t m_bedMips = 0;
    hal::RootSignatureRef m_traceRs;
    hal::Pso m_traceK;
    hal::Table m_traceTable;   // [t1 height page, t2 its residency map, u4 the trace]
    GpuTexture m_traceTex;     // RG32F: BedAt, mip + 16 * slice
    // The west trace's (TraceWestFace): the flux kernel under SWE_WEST_TRACE, its table and target.
    hal::RootSignatureRef m_westRs;
    hal::Pso m_westK;
    hal::Table m_westTable;    // [t1, t2, u0 eta, u1 flux, u2 uv, u3 mv, u5 the trace]
    GpuTexture m_westTex;      // RGBA32F, 4 x ny
    float m_westArea = 0.0f;   // Record's live west section, m^2
    uint64_t m_cbFp = 0;               // M12 step 4b: the [kernel] swe cb fingerprint's last value
    float m_lastTideNavd = 0;          // for the tide-plane rate (prism source term)
    double m_lastTideTime = 0;
    float m_dt = 0.25f;
    bool m_pendingReset = true;

    // ---- the region readback (RequestRegion / DeviationAt) --------------------------------------
    // A point of the flat frame in the kernel's continuous texel coordinates (u Nx, (1 - v) Ny).
    bool TexelOf(double worldX, double worldZ, double& tx, double& ty) const;
    // Read what the ring delivered for this frame's slot, then copy this frame's requests into it.
    void CollectRegions(Gpu& gpu);
    void ServeRegions(hal::CommandContext& cmd, Gpu& gpu,
                      const std::function<void(D3D12_RESOURCE_STATES)>& etaTo,
                      const std::function<void(D3D12_RESOURCE_STATES)>& uvTo);
    struct RegionRect {
        uint32_t x0 = 0, y0 = 0, w = 0, h = 0;
    };
    struct DeliveredRegion {
        RegionRect rect;
        std::vector<float> eta;   // w * h
        std::vector<float> uv;    // w * h * 4: u, v, speed, valid
        double asOf = 0.0;
    };
    // Bounds, and why these: a hull asks for its footprint plus its motion over the ring's latency
    // (a few metres to tens), so a region of kMaxRegionTexels a side holds any hull at any speed a
    // hull reaches in two frames; kMaxRegions requests a frame holds eight subjects. The readback
    // slot is sized from the two, and a request past either is refused and logged, never clipped.
    static constexpr uint32_t kMaxRegions = 8;
    static constexpr uint32_t kMaxRegionTexels = 64;
    std::vector<RegionRect> m_requests;           // this frame's
    std::vector<DeliveredRegion> m_regions;       // newest first
    double m_slotAsOf[Gpu::kFrameCount] = {};      // the clock each slot's copies were taken at
    hal::RegionReadback m_readback;
    bool m_refusedLogged = false;
};

}  // namespace ga
