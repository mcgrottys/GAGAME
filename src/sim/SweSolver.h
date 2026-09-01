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
#include "core/Gpu.h"
#include "core/Shader.h"
#include "core/TileAtlas.h"
#include "sim/BathyModel.h"

#include <algorithm>
#include <cmath>
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

class SweSolver {
public:
    static constexpr uint32_t kMaxSubsteps = 16;

    void Init(Gpu& gpu, ShaderCompiler& sc, const std::wstring& shaderDir,
              const BathyModel& bathy, ID3D12Resource* bathyRes,
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

    // Advance toward simUnix (records compute onto cl) and leave eta + uv sampleable.
    // tideNavd = the analytic water level, NAVD88 m. Returns substeps executed this frame.
    int Record(ID3D12GraphicsCommandList* cl, Gpu& gpu, double simUnix, float tideNavd);

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
            ID3D12GraphicsCommandList* cl = gpu.BeginUpload();
            Record(cl, gpu, target, static_cast<float>(tideAt(m_simTime)), 9999);
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
    uint32_t UvSrv() const { return m_uv.srv; }        // dense currents, RGBA16F
    uint32_t VelGradSrv() const { return m_velGrad.Srv(); }   // M9h: (div, curl), RG16F
    uint32_t VelGradResMapSrv() const { return m_velGrad.ResidencyMapSrv(); }
    uint32_t VelGradMips() const { return m_velGrad.MipCount(); }
    // The residency map is one texel per mip-0 TILE, so the lens indexes it in tile space.
    uint32_t VelGradResMapW() const { return m_velGrad.TilesX(); }
    uint32_t VelGradResMapH() const { return m_velGrad.TilesY(); }
    ID3D12Resource* VelGradRes() const { return m_velGrad.Res(); }
    ID3D12Resource* UvRes() const { return m_uv.res.Get(); }
    ID3D12Resource* BathyRes() const { return m_bathyRes; }   // the churn kernel reads both
    uint32_t Nx() const { return m_cb.nx; }
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
    Probe ReadProbe(Gpu& gpu, float wx, float wz) {
        const float p[2] = {wx, wz};
        Probe out{};
        ReadProbes(gpu, p, 1, &out);
        return out;
    }

    std::string stats;         // "swe 476t 31/54MB 16ss" for the title bar
    double SimTime() const { return m_simTime; }
    float Dt() const { return m_dt; }

private:
    int Record(ID3D12GraphicsCommandList* cl, Gpu& gpu, double simUnix, float tideNavd,
               int maxSub);
    void RecordReset(ID3D12GraphicsCommandList* cl, Gpu& gpu);

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
        // M9h: the GoMOFS ingest rows. APPENDED AT THE END on both sides -- a same-size
        // insertion in the middle passes the byte-parity gate and silently offsets every later
        // row (the lesson WaterBank.hlsl:44 records, nearly repeated here).
    };

    bool m_ready = false;
    const BathyModel* m_bathy = nullptr;
    ID3D12Resource* m_bathyRes = nullptr;   // borrowed from TerrainLayer; outlives the solver
    TileAtlas2D m_eta, m_flux;
    // M9h: grad(flow) -- residency derived from the Cayley closure, not a physics policy.
    GradeBank m_velGrad;
    ShaderCompiler* m_sc = nullptr;   // M9h: BuildChain compiles the reducer on first use
    std::wstring m_shaderDir;
    GpuTexture m_uv;
    uint32_t m_uvUav = UINT32_MAX;
    Com<ID3D12RootSignature> m_rs;
    Com<ID3D12PipelineState> m_clearEta, m_clearFlux, m_uvClear, m_fluxK, m_heightK, m_deriveK;
    Com<ID3D12PipelineState> m_velGradK;   // M9h: grad(flow) -> div + curl
    uint32_t m_table = UINT32_MAX;   // [t1 bathy SRV, u0 eta, u1 flux, u2 uv]
    D3D12_RESOURCE_STATES m_etaState = D3D12_RESOURCE_STATE_UNORDERED_ACCESS;
    D3D12_RESOURCE_STATES m_uvState = D3D12_RESOURCE_STATE_UNORDERED_ACCESS;

    SweCbData m_cb{};
    double m_simTime = 0;
    float m_westDEta = 0;
    float m_southDEta = 0;
    float m_westQ = 0;                 // west transport target, m^3/s (+east)
    std::vector<float> m_westBed;      // exterior-column bed depths: the live section area
    float m_lastTideNavd = 0;          // for the tide-plane rate (prism source term)
    double m_lastTideTime = 0;
    float m_dt = 0.25f;
    bool m_pendingReset = true;
};

}  // namespace ga
