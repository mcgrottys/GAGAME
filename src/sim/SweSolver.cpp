#include "sim/SweSolver.h"

#include "hal/PixEvents.h"
#include "hal/Pipeline.h"
#include "hal/Root.h"
#include "hal/Views.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <queue>
#include <string>
#include <utility>

namespace ga {

// ---- PHASE C1: THE SOLVER'S CHART (SweSolver.h) ------------------------------------------------

namespace {
constexpr double kSweD2R = 3.14159265358979323846 / 180.0;
double Dot3(const double a[3], const double b[3]) { return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]; }
void DirOf(double latDeg, double lonDeg, double d[3]) {   // GlobeModel::LatLonDir's convention
    const double la = latDeg * kSweD2R, lo = lonDeg * kSweD2R;
    d[0] = std::cos(la) * std::cos(lo);
    d[1] = std::sin(la);
    d[2] = std::cos(la) * std::sin(lo);
}
}  // namespace

void SweDomain::Place(double boxLat0, double boxLon0, double boxLat1, double boxLon1,
                      uint32_t cellsX, uint32_t cellsY, double planetR) {
    lat0 = boxLat0;
    lon0 = boxLon0;
    lat1 = boxLat1;
    lon1 = boxLon1;
    nx = cellsX;
    ny = cellsY;
    R = planetR;
    latC = 0.5 * (lat0 + lat1);
    lonC = 0.5 * (lon0 + lon1);
    DirOf(latC, lonC, A);
    const double lo = lonC * kSweD2R;
    E[0] = -std::sin(lo);
    E[1] = 0.0;
    E[2] = std::cos(lo);
    // north = east x up in this planet frame (FrameLoop's M6i note: an odd permutation of ECEF)
    N[0] = E[1] * A[2] - E[2] * A[1];
    N[1] = E[2] * A[0] - E[0] * A[2];
    N[2] = E[0] * A[1] - E[1] * A[0];
    dx = R * std::cos(latC * kSweD2R) * (lon1 - lon0) * kSweD2R / double(nx);
    dy = R * (lat1 - lat0) * kSweD2R / double(ny);
    // tx = (R (P.E) / (P.A) + nx dx / 2) / dx,  ty = (ny dy / 2 - R (P.N) / (P.A)) / dy
    for (int i = 0; i < 3; ++i) {
        U[i] = R / dx * E[i] + 0.5 * double(nx) * A[i];
        V[i] = 0.5 * double(ny) * A[i] - R / dy * N[i];
    }
    elev.clear();
    sponge.clear();
    openSides = 0;
}

bool SweDomain::CellOfDir(const double P[3], double& tx, double& ty) const {
    const double pa = Dot3(P, A);
    if (!Ready() || !(pa > 0.0)) return false;
    tx = Dot3(P, U) / pa;
    ty = Dot3(P, V) / pa;
    return true;
}

bool SweDomain::CellOf(double latDeg, double lonDeg, double& tx, double& ty) const {
    double d[3];
    DirOf(latDeg, lonDeg, d);
    return CellOfDir(d, tx, ty);
}

void SweDomain::LatLonOf(double tx, double ty, double& latDeg, double& lonDeg) const {
    const double s = (tx - 0.5 * double(nx)) * dx / R, t = (0.5 * double(ny) - ty) * dy / R;
    double p[3];
    for (int i = 0; i < 3; ++i) p[i] = A[i] + s * E[i] + t * N[i];
    const double l = std::sqrt(Dot3(p, p));
    latDeg = std::asin(p[1] / l) / kSweD2R;
    lonDeg = std::atan2(p[2], p[0]) / kSweD2R;
}

bool SweDomain::Holds(double latDeg, double lonDeg) const {
    double tx = 0.0, ty = 0.0;
    return CellOf(latDeg, lonDeg, tx, ty) && tx >= 0.0 && ty >= 0.0 && tx < double(nx) &&
           ty < double(ny);
}

void SweDomain::FillBed(const std::function<float(double, double, double)>& bedAt) {
    elev.assign(size_t(nx) * ny, -9999.0f);
    const double grain = (std::max)(dx, dy);   // RealizeFromChannel's: the cell's larger side
    for (uint32_t j = 0; j < ny; ++j) {
        for (uint32_t i = 0; i < nx; ++i) {
            double la = 0.0, lo = 0.0;
            LatLonOf(i + 0.5, j + 0.5, la, lo);
            elev[size_t(j) * nx + i] = bedAt(la, lo, grain);
        }
    }
}

double SweDomain::SpongeAt(int i, int j, double spongeM) const {
    double d = 1.0e300;
    if (openSides & 1u) d = (std::min)(d, (j + 0.5) * dy);
    if (openSides & 2u) d = (std::min)(d, (double(ny) - j - 0.5) * dy);
    if (openSides & 4u) d = (std::min)(d, (i + 0.5) * dx);
    if (openSides & 8u) d = (std::min)(d, (double(nx) - i - 0.5) * dx);
    const double t = std::clamp((d - (spongeM - 700.0)) / 700.0, 0.0, 1.0);
    return 1.0 - t * t * (3.0 - 2.0 * t);   // the kernel's 1 - smoothstep(d0 - 700, d0, d)
}

void SweDomain::Bound(const SweConfig& cfg) {
    openSides = 0;
    if (!Ready() || elev.size() != size_t(nx) * ny) return;
    const auto wet = [&](size_t i) { return elev[i] > -9000.0f && elev[i] < 1.2f; };
    for (int side = 0; side < 4; ++side) {
        if (side == 2 && cfg.westBoundary) continue;   // the river's side is the Flather face
        const int len = side < 2 ? int(nx) : int(ny);
        bool open = true;
        for (int k = 0; k < len && open; ++k) {
            const size_t i = side == 0   ? size_t(k)
                             : side == 1 ? size_t(ny - 1) * nx + k
                             : side == 2 ? size_t(k) * nx
                                         : size_t(k) * nx + (nx - 1);
            open = wet(i);
        }
        if (open) openSides |= 1u << side;
    }
    sponge.assign(size_t(nx) * ny, 0);
    for (uint32_t j = 0; j < ny; ++j) {
        for (uint32_t i = 0; i < nx; ++i) {
            sponge[size_t(j) * nx + i] = SpongeAt(int(i), int(j), cfg.spongeM) > 0.0 ? 1 : 0;
        }
    }
}

void SweDomain::KernelRows(const double east[3], const double up[3], const double north[3],
                           const double origin[3], float out[16]) const {
    for (int k = 0; k < 16; ++k) out[k] = 0.0f;
    if (!Ready()) return;
    const double* ax[3] = {east, up, north};
    const double* pl[3] = {U, V, A};
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) out[4 * r + c] = static_cast<float>(Dot3(pl[r], ax[c]) / R);
        out[4 * r + 3] = static_cast<float>(Dot3(pl[r], origin) / R);
    }
    out[12] = static_cast<float>(nx);
    out[13] = static_cast<float>(ny);
    out[14] = 1.0f;
}

void SweSolver::LogCbFingerprint() {
    // M12 step 4b instrument: the solver's constant buffer, fingerprinted at every upload --
    // the gate for the lattice-row moves (winA from the surface's lattice) and for the fills
    // of 4e/4f. Logs when the hash changes, as the [surface] fills do; the rows every Record
    // varies (the tide plane, its rate, u_ext) change it per batch, so the log is the solver's
    // whole CPU-side trajectory. A member, not a static: an owned Boston window is a second
    // instance.
    const uint64_t h = CbTrace() ? Fnv1aBytes(&m_cb, sizeof(m_cb)) : m_cbFp;   // F9: --cb-trace
    if (h != m_cbFp) {
        m_cbFp = h;
        Log("[kernel] swe cb FNV-1a %016llx", static_cast<unsigned long long>(h));
    }
}

void SweSolver::SetBed(Gpu& gpu, hal::Resource heightArr, hal::Resource resMapArr, uint32_t mips,
                       const BedWindow& w) {
    (void)gpu;
    m_table.SrvArray(0, heightArr, DXGI_FORMAT_R16_FLOAT, 0, UINT32_MAX, mips);
    m_table.SrvArray(1, resMapArr, DXGI_FORMAT_R8_UNORM, 0, UINT32_MAX, 1);
    const SweDomain& d = m_dom;
    m_cb.cellX[0] = static_cast<float>((0.5 - 0.5 * double(d.nx)) * d.dx / d.R);
    m_cb.cellX[1] = static_cast<float>(d.dx / d.R);
    m_cb.cellX[2] = static_cast<float>((0.5 * double(d.ny) - 0.5) * d.dy / d.R);
    m_cb.cellX[3] = static_cast<float>(-d.dy / d.R);
    for (int i = 0; i < 3; ++i) {
        m_cb.anc[i] = static_cast<float>(d.A[i]);
        m_cb.east[i] = static_cast<float>(d.E[i]);
    }
    m_cb.anc[3] = static_cast<float>(d.R);
    m_cb.east[3] = m_cb.dx;   // the cell's grain: its east-west side
    static_assert(sizeof(w.rows) == sizeof(float) * 80, "the kernels' rows");
    memcpy(m_cb.hwU, w.rows, sizeof(w.rows));
    m_bedSlice = w.slice;   // the standing window's slice (PageSlice)
    m_bedArr = heightArr;
    m_bedRes = resMapArr;
    m_bedMips = mips;
    m_bedBound = true;
    Log("[swe] bed bound to the standing window: slice %u, about %.5f N %.5f E, %u mips, the "
        "cell's grain %.2f m -- the solver reads the pyramid through its own window",
        w.slice, m_dom.latC, m_dom.lonC, mips, double(m_cb.dx));
}

bool SweSolver::TraceBed(Gpu& gpu, BedTrace& out, float floorMip) {
    if (!m_ready || !m_bedBound || !m_sc) return false;
    if (!m_traceK) {
        // Built on the first call: b0 the solver's own constants (BedAt reads its lattice rows
        // from them), b1 the floor, and a table of the solver's two bed views and the target.
        m_traceRs = hal::RootLayout{}
                        .Cbv(0)
                        .Cbv(1)
                        .Table({hal::SrvRange(1, 2), hal::UavRange(4, 1)})
                        .Build(gpu, "swe.bedtrace");
        m_traceRs->SetName(L"swe bed trace root signature");
        m_traceK = hal::BuildCompute(gpu, m_traceRs.Get(),
                                     m_sc->Compile(m_shaderDir + L"/Swe.hlsl", L"CsSweBedTrace",
                                                   L"cs_6_0", {L"HP_TRACE=1"}),
                                     "swe.bedtrace");
        if (!m_traceK) return false;
        m_traceTex = gpu.CreateTexture2D(m_cb.nx, m_cb.ny, DXGI_FORMAT_R32G32_FLOAT,
                                         D3D12_RESOURCE_FLAG_ALLOW_UNORDERED_ACCESS,
                                         D3D12_RESOURCE_STATE_UNORDERED_ACCESS,
                                         L"swe.bedtrace (BedAt, mip + 16 slice)");
        m_traceTable = hal::Table::Alloc(gpu, 3, "swe.bedtrace");
        m_traceTable.SrvArray(0, m_bedArr, DXGI_FORMAT_R16_FLOAT, 0, UINT32_MAX, m_bedMips);
        m_traceTable.SrvArray(1, m_bedRes, DXGI_FORMAT_R8_UNORM, 0, UINT32_MAX, 1);
        m_traceTable.Uav2D(2, m_traceTex.res.Get(), DXGI_FORMAT_R32G32_FLOAT);
    }
    struct {
        float floorMip, pad[3];
    } tc{floorMip, {0.0f, 0.0f, 0.0f}};
    {
        hal::CommandContext up(gpu, gpu.BeginUpload(), hal::Owner::Upload);
        up.BindHeaps();
        up.ComputeRoot(m_traceRs.Get());
        up.ComputeConstants(0, m_cb);
        up.ComputeConstants(1, tc);
        up.ComputeTable(2, m_traceTable.Base());
        up.Pipeline(m_traceK.Get());
        up.Dispatch((m_cb.nx + 15) / 16, (m_cb.ny + 15) / 16, 1);
        gpu.EndUpload();
    }
    uint32_t pitch = 0;
    const std::vector<uint8_t> raw = gpu.ReadbackTexture(m_traceTex, &pitch);
    out.nx = m_cb.nx;
    out.ny = m_cb.ny;
    const size_t n = size_t(out.nx) * out.ny;
    out.bed.resize(n);
    out.mip.resize(n);
    out.slice.resize(n);
    for (uint32_t y = 0; y < out.ny; ++y) {
        const float* row = reinterpret_cast<const float*>(&raw[size_t(y) * pitch]);
        for (uint32_t x = 0; x < out.nx; ++x) {
            const size_t i = size_t(y) * out.nx + x;
            // A cell the rule never reached keeps the trace's -1: it reads as mip 15, slice 15.
            const float c = row[x * 2 + 1];
            const uint32_t code = (c < 0.0f) ? 255u : static_cast<uint32_t>(c + 0.5f);
            out.bed[i] = row[x * 2];
            out.mip[i] = static_cast<uint8_t>(code & 15u);
            out.slice[i] = static_cast<uint8_t>(code >> 4);
        }
    }
    return true;
}

void SweSolver::Init(Gpu& gpu, ShaderCompiler& sc, const std::wstring& shaderDir,
                     const SweDomain& dom, const SweConfig& cfg) {
    m_dom = dom;
    m_dom.Bound(cfg);
    m_sc = &sc;
    m_shaderDir = shaderDir;
    const uint32_t nx = m_dom.nx, ny = m_dom.ny;
    const float dx = static_cast<float>(m_dom.dx);
    Log("[swe] %s domain: box %.5f..%.5f N x %.5f..%.5f E about its anchor %.6f N %.6f E, %ux%u "
        "cells of %.3f x %.3f m (true, at the anchor) -- open sides%s%s%s%s%s, the sponge %.0f m in "
        "from them; river face: %s",
        cfg.name, m_dom.lat0, m_dom.lat1, m_dom.lon0, m_dom.lon1, m_dom.latC, m_dom.lonC, nx, ny,
        m_dom.dx, m_dom.dy, m_dom.openSides ? "" : " none", (m_dom.openSides & 1u) ? " N" : "",
        (m_dom.openSides & 2u) ? " S" : "", (m_dom.openSides & 4u) ? " W" : "",
        (m_dom.openSides & 8u) ? " E" : "", double(cfg.spongeM), cfg.westBoundary ? "west" : "none");

    // Two grade banks over the bathy grid, padded up to tile multiples internally by the atlas.
    m_eta.Init(gpu, nx, ny, DXGI_FORMAT_R32_FLOAT, L"swe.eta (dEta from the tide plane)");
    m_flux.Init(gpu, nx, ny, DXGI_FORMAT_R32G32_FLOAT, L"swe.flux (signed face fluxes)");
    {
        // M9h: grad(flow). The first field here whose residency comes from the ALGEBRA rather
        // than from a physics policy. grad is a grade-1 operator and the flow is grade 1, so
        // the product signature is Cl2ProductSignature(kG1, kG1) = kG0 | kG2 -- divergence and
        // vorticity, and nothing else can appear. R16G16F is exactly those two grades, and at
        // 4 B a texel it tiles 128x128 like eta, so the derived demand maps 1:1 onto eta's
        // tile grid with no resampling.
        GradeBankDesc d;
        d.name = "swe.velgrad (grad flow: div + curl)";
        d.width = nx;
        d.height = ny;
        d.fmt = DXGI_FORMAT_R16G16B16A16_FLOAT;   // div, curl, COVERAGE, spare
        d.gradeSig = kG0 | kG2;
        d.metersPerTexel = dx;
        d.units = "1/s";
        d.range = "+-0.5";
        // M9h: THE CHAIN. Six levels over a 1863x1174 window takes the field from ~10 m to
        // ~320 m per texel, which is the range a zoom actually traverses. Without it the lens
        // sampled mip 0 at every altitude and tile edges read as hard rectangles; with it a
        // pulled-back camera reads a level whose texels match its footprint, and the residency
        // map keeps the sample from ever landing on a NULL.
        d.mipLevels = 6;
        // M9j: TWO PAGES. Slice 0 is this window -- the solve, at the solve's resolution.
        // Slice 1 is the REGION, composed from whatever covers the wider ground. They are
        // pages of one address space in one resource, so the consumer reads both from a single
        // view and composites them on coverage: no second SRV, and no branch deciding which
        // source owns a pixel.
        d.arraySlices = 2;
        // Channel 2 is coverage (the compose loop writes it there). Declaring it makes the mip
        // chain weight by it: without this, a coarse texel over a half-covered region averages
        // real divergence with the zeros of absence and reports the result as fact -- and the
        // error is WORST at the pinned floor, which is exactly what a distant sample reads.
        m_velGrad.SetCoverageChannel(2);
        m_velGrad.Init(gpu, d, policy::None());
        // A sliced bank activates nothing by itself -- an inactive slice honestly reports
        // kNothingResident. Slice 0 is ours and must be live before any residency is asked.
        m_velGrad.ActivateSlice(gpu, 0);
    }

    // Static residency: everything that can ever be wet -- bed below max tide + surge + wave
    // margin. Land and dune tiles stay NULL forever; their reads are the hardware zero.
    const auto& elev = m_dom.elev;
    auto mapWet = [&](auto& bank) {
        for (uint32_t ty = 0; ty < bank.TilesY(); ++ty) {
            for (uint32_t tx = 0; tx < bank.TilesX(); ++tx) {
                bool wet = false;
                const uint32_t x1 = (std::min)((tx + 1) * bank.TileW(), nx);
                const uint32_t y1 = (std::min)((ty + 1) * bank.TileH(), ny);
                for (uint32_t y = ty * bank.TileH(); y < y1 && !wet; ++y) {
                    for (uint32_t x = tx * bank.TileW(); x < x1; ++x) {
                        const float e = elev[y * nx + x];
                        // < 1.2 NAVD: subtidal + the flats that actually wet at normal ranges.
                        // The marsh PLATEAU above stays NULL: its creeks are subgrid at 13.7 m,
                        // so sheet-flooding those cells fills real storage on a fake clock
                        // (+80 min aggregate lag in the M6d validation). Rejoins the domain
                        // when subgrid channel conveyance exists.
                        if (e > -9000.0f && e < 1.2f) {
                            wet = true;
                            break;
                        }
                    }
                }
                if (wet) bank.RequestMap(tx, ty);
            }
        }
        std::vector<uint32_t> fresh;
        bank.CommitMappings(gpu, &fresh);
    };
    mapWet(m_eta);
    mapWet(m_flux);

    // ---- M9h: THE CAYLEY CLOSURE DECIDES THE DERIVED FIELD'S RESIDENCY --------------------
    // The flow is grade 1 wherever the solver simulates it, and NOWHERE else; nabla is grade 1
    // by construction. So the tiles where grad(flow) can be non-zero are exactly the tiles
    // where Cl2ProductSignature(kG1, kG1) is non-zero -- decided without reading one texel of
    // current. This is the load-bearing claim of the whole atlas, exercised for real rather
    // than assumed.
    //
    // The one thing algebra does NOT know is the stencil: a central difference reads its
    // neighbours, so the demand is DILATED by one tile. Skipping that would not crash -- null
    // tile writes are silently discarded (hazard 1) -- it would quietly punch holes along
    // every tile seam, which is the failure this project keeps having to learn to see.
    {
        const uint32_t tX = m_eta.TilesX(), tY = m_eta.TilesY();
        std::vector<uint8_t> flowSig(size_t(tX) * tY, 0), nablaSig(size_t(tX) * tY, kG1);
        for (uint32_t ty = 0; ty < tY; ++ty) {
            for (uint32_t tx = 0; tx < tX; ++tx) {
                if (m_eta.IsResident(tx, ty)) flowSig[size_t(ty) * tX + tx] = kG1;
            }
        }
        const TilePolicy derived = policy::Derived(flowSig, nablaSig, tX);
        uint32_t core = 0;
        std::vector<uint8_t> want(size_t(tX) * tY, 0);
        for (uint32_t ty = 0; ty < tY; ++ty) {
            for (uint32_t tx = 0; tx < tX; ++tx) {
                if (!derived(tx, ty)) continue;
                ++core;
                for (int dy = -1; dy <= 1; ++dy) {   // the stencil apron
                    for (int dx = -1; dx <= 1; ++dx) {
                        const int ax = int(tx) + dx, ay = int(ty) + dy;
                        if (ax < 0 || ay < 0 || ax >= int(tX) || ay >= int(tY)) continue;
                        want[size_t(ay) * tX + ax] = 1;
                    }
                }
            }
        }
        m_velGrad.SetPolicy(policy::Mask(want, tX));
        m_velGrad.Update(gpu);
        // Coarse levels get mapped here, outside any recording: the reduction that fills them
        // runs on the frame list, but the MAPPING touches the queue and the residency map.
        m_velGrad.EnsureCoarseMapped(gpu);
        Log("[swe] grad(flow) residency BY ALGEBRA: Cl2(g1,g1) = g0|g2 -> %u core tiles, "
            "%u with the stencil apron, of %u (%.0f%%); bank %.1f of %.1f MB",
            core, m_velGrad.ResidentCount(), tX * tY,
            100.0 * m_velGrad.ResidentCount() / double(tX * tY),
            m_velGrad.ResidentBytes() / 1048576.0, m_velGrad.VirtualBytes() / 1048576.0);
    }
    for (uint32_t ty = 0; ty < m_eta.TilesY(); ++ty) {
        for (uint32_t tx = 0; tx < m_eta.TilesX(); ++tx) {
            if (!m_eta.IsResident(tx, ty)) Log("[swe] eta NULL tile (%u,%u)", tx, ty);
        }
    }

    // Timestep from the deepest resident water (pipe scheme + damping tolerate ~0.6 dx / c).
    // M6r: the CUDEM grid is equiangular, so texels are ANISOTROPIC in metres (dlon*mPerLon =
    // 10.08 east vs dlat*mPerLat = 13.65 north); the CFL rides the smaller axis.
    const float dyM = static_cast<float>(m_dom.dy);
    float deepest = 0.0f;
    for (float e : elev) {
        if (e > -9000.0f) deepest = (std::min)(deepest, e);
    }
    const float hmax = -deepest + 3.0f;
    m_dt = 0.45f * (std::min)(dx, dyM) / std::sqrt(9.81f * (std::max)(hmax, 5.0f));

    m_cb.nx = nx;
    m_cb.ny = ny;
    m_cb.etaTilesX = m_eta.TilesX();
    m_cb.fluxTilesX = m_flux.TilesX();
    m_cb.etaTileW = m_eta.TileW();
    m_cb.etaTileH = m_eta.TileH();
    m_cb.fluxTileW = m_flux.TileW();
    m_cb.fluxTileH = m_flux.TileH();
    m_cb.spongeSides = static_cast<float>(m_dom.openSides);
    m_cb.dx = dx;
    m_cb.dy = dyM;
    m_cb.dt = m_dt;
    m_cb.damp = 0.99995f;   // background linear part only; the real friction is quadratic drag
    m_cb.spongeD0 = cfg.spongeM;          // ramp start from an open side: past the bar, before the open sea
    m_cb.spongeRate = m_dt / 10.0f;       // full-strength deviations die in ~10 s
    m_cb.gravity = 9.81f;
    // M6r: the west boundary is FLATHER now -- one exterior column pinned to the river-tide
    // data, its east face radiating at the gravity-wave speed (Swe.hlsl). The 24-texel
    // Dirichlet strip is retired: it was a soft wall (transients reflected off the pinned
    // eta), and its relax rate was a tuning knob the radiation condition does not need.
    // M6x: windows without a truncated river (Boston: dammed) turn it off -- the west edge
    // is then a wall like any land.
    m_cb.riverBox[0] = cfg.westBoundary ? 1.0f : 0.0f;   // exterior column width, texels
    m_cb.riverBox[1] = m_dt / 2.0f;       // relax rate: SOUTH strip only (retired, below)
    if (cfg.westBoundary) {
        // The exterior column's bed profile: Record turns the west TRANSPORT into Flather's
        // u_ext by dividing by the live wet section area (level moves every substep batch).
        for (uint32_t y = 0; y < ny; ++y) {
            const float e = elev[static_cast<size_t>(y) * nx];
            if (e > -9000.0f && e < 2.0f) m_westBed.push_back(e);
        }
        Log("[swe] %s west boundary: FLATHER, %zu wet-capable section cells (dy %.2f m)",
            cfg.name, m_westBed.size(), dyM);
    } else {
        Log("[swe] %s west boundary: wall (no truncated river)", cfg.name);
    }
    // M6d: the south strip exists only when the window actually reaches Plum Island Sound
    // (the sound's real entrance is south of the window; its tide must enter as data).
    // RETIRED pending a throttled treatment: pinning the sound's south edge to the ocean clock
    // turned it into an infinite reservoir 2 km from the harbor basin -- the basin then filled
    // through the BACK DOOR (Plum Island River) instead of the inlet, the throat current
    // collapsed to the west-boundary residual, and the validation lag blew out to +80 min.
    // The --swe-west-off diagnostic pinned it: gap flow went to ZERO with west off. The sound
    // now rides passively (fills via its narrow river; imperfect but bounded and honest).
    const bool hasSound = false;
    m_cb.riverBox[2] = hasSound ? static_cast<float>(ny - 8) : 1.0e9f;
    if (hasSound) {
        Log("[swe] south boundary strip active (Plum Island Sound rows %u..%u)", ny - 8, ny - 1);
    }
    m_cfg = cfg;

    // M9ax: the derived currents as a bank over the wet tiles. Grade 1 (a vector field);
    // RGBA16F keeps the (u, v, |U|, valid) fiber the consumers already read. Same residency as
    // eta: land tiles stay NULL and read the hardware zero, which the sea shader already treats
    // as "no solve here" through the valid channel.
    {
        GradeBankDesc d;
        d.name = "swe.uv (derived currents)";
        d.width = nx;
        d.height = ny;
        d.fmt = DXGI_FORMAT_R16G16B16A16_FLOAT;
        d.gradeSig = kG1;
        d.metersPerTexel = dx;
        d.units = "m/s";
        d.range = "+-2.5";
        d.mipLevels = 1;
        d.arraySlices = 1;
        m_uvBank.Init(gpu, d, policy::None());
        m_uvBank.ActivateSlice(gpu, 0);
        mapWet(m_uvBank);
        m_uvSrv = gpu.CreateSrv(m_uvBank.Res(), DXGI_FORMAT_R16G16B16A16_FLOAT);
        m_uvState = D3D12_RESOURCE_STATE_UNORDERED_ACCESS;
    }

    // Root signature: b0 CBV, t0 root SRV (tile list), table [t1 height page, t2 its residency
    // map (M9ar), u0 eta, u1 flux, u2 uv, u3 mv (M9h: the derived grad(flow) bank)].
    m_rs = hal::RootLayout{}
               .Cbv(0)
               .Srv(0)
               .Table({hal::SrvRange(1, 2), hal::UavRange(0, 4)})
               .Build(gpu, "swe");
    m_rs->SetName(L"swe root signature");

    auto makePso = [&](const wchar_t* entry, hal::Pso& out) {
        out = hal::Require(
            hal::BuildCompute(gpu, m_rs.Get(),
                              sc.Compile(shaderDir + L"/Swe.hlsl", entry, L"cs_6_0"), "swe"),
            "Swe kernel");
        out->SetName(entry);
    };
    makePso(L"CsSweClearEta", m_clearEta);
    makePso(L"CsSweClearFlux", m_clearFlux);
    makePso(L"CsSweUvClear", m_uvClear);
    makePso(L"CsSweFlux", m_fluxK);
    makePso(L"CsSweHeight", m_heightK);
    makePso(L"CsSweDerive", m_deriveK);
    makePso(L"CsSweVelGrad", m_velGradK);

    m_table = hal::Table::Alloc(gpu, 6, "swe");   // t1, t2 filled by SetHeightPage; u0..u3 here
    m_table.Uav2D(2, m_eta.Res(), DXGI_FORMAT_R32_FLOAT);
    m_table.Uav2D(3, m_flux.Res(), DXGI_FORMAT_R32G32_FLOAT);
    // The bank is an ARRAY now, so its UAV must be an array view pinned to slice 0. A plain
    // Texture2D view of an array resource is invalid, and the solve would write nowhere.
    m_table.UavArray(4, m_uvBank.Res(), DXGI_FORMAT_R16G16B16A16_FLOAT, 0, 1);   // M9ax: uv bank
    m_table.UavArray(5, m_velGrad.Res(), DXGI_FORMAT_R16G16B16A16_FLOAT, 0, 1);

    Log("[swe] grid %ux%u dx %.2f x dy %.2f m  dt %.3f s  eta %u/%u t  flux %u/%u t  "
        "resident %.1f MB",
        nx, ny, dx, dyM, m_dt, m_eta.ResidentCount(), m_eta.TilesX() * m_eta.TilesY(),
        m_flux.ResidentCount(), m_flux.TilesX() * m_flux.TilesY(),
        (m_eta.ResidentBytes() + m_flux.ResidentBytes()) / 1048576.0);
    // The region readback's slot: every request a frame may carry at its largest, eta and current
    // (12 bytes a texel), with room for the copy footprints' row alignment.
    m_readback.Init(gpu, uint64_t(kMaxRegions) * kMaxRegionTexels * kMaxRegionTexels * 16u,
                    L"swe region readback");
    m_ready = true;
    m_pendingReset = true;
}

// ---- THE REGION READBACK (the water match, step 1: the solver is truth) ---------------------------

bool SweSolver::TexelOf(double latDeg, double lonDeg, double& tx, double& ty) const {
    // The readers' own map (WaterBank.hlsl's solver rows): the domain's planes, in doubles.
    return m_ready && m_dom.CellOf(latDeg, lonDeg, tx, ty);
}

double SweSolver::DomainWeight(double latDeg, double lonDeg) const {
    double tx = 0.0, ty = 0.0;
    if (!TexelOf(latDeg, lonDeg, tx, ty)) return 0.0;
    const double e = (std::min)((std::min)(tx, double(m_cb.nx) - tx),
                                (std::min)(ty, double(m_cb.ny) - ty));
    const double t = std::clamp(e, 0.0, 1.0);
    return t * t * (3.0 - 2.0 * t);   // HLSL smoothstep(0, 1, e)
}

void SweSolver::RequestRegion(double latDeg, double lonDeg, double radiusM) {
    double tx = 0.0, ty = 0.0;
    if (!TexelOf(latDeg, lonDeg, tx, ty) || !(radiusM >= 0.0)) return;
    // The texels a bilinear read of any point within the radius touches: floor(t - 0.5) and the
    // one past it, at both ends.
    const double rx = radiusM / double(m_cb.dx), ry = radiusM / double(m_cb.dy);
    const double lx0 = std::floor(tx - 0.5 - rx), lx1 = std::floor(tx - 0.5 + rx) + 1.0;
    const double ly0 = std::floor(ty - 0.5 - ry), ly1 = std::floor(ty - 0.5 + ry) + 1.0;
    const double nx = double(m_cb.nx), ny = double(m_cb.ny);
    if (lx1 < 0.0 || ly1 < 0.0 || lx0 > nx - 1.0 || ly0 > ny - 1.0) return;   // nowhere on the grid
    RegionRect r;
    r.x0 = uint32_t(std::clamp(lx0, 0.0, nx - 1.0));
    r.y0 = uint32_t(std::clamp(ly0, 0.0, ny - 1.0));
    r.w = uint32_t(std::clamp(lx1, 0.0, nx - 1.0)) - r.x0 + 1u;
    r.h = uint32_t(std::clamp(ly1, 0.0, ny - 1.0)) - r.y0 + 1u;
    if (r.w > kMaxRegionTexels || r.h > kMaxRegionTexels) {
        if (!m_refusedLogged) {
            Log("[swe] region of %ux%u texels REFUSED (at most %u a side: radius %.1f m) -- the "
                "consumer's answer stays the last one delivered", r.w, r.h, kMaxRegionTexels, radiusM);
            m_refusedLogged = true;
        }
        return;
    }
    auto holds = [](const RegionRect& a, const RegionRect& b) {
        return b.x0 >= a.x0 && b.y0 >= a.y0 && b.x0 + b.w <= a.x0 + a.w && b.y0 + b.h <= a.y0 + a.h;
    };
    for (const RegionRect& q : m_requests) {
        if (holds(q, r)) return;   // already asked for this frame
    }
    m_requests.erase(std::remove_if(m_requests.begin(), m_requests.end(),
                                    [&](const RegionRect& q) { return holds(r, q); }),
                     m_requests.end());
    if (m_requests.size() >= kMaxRegions) {
        if (!m_refusedLogged) {
            Log("[swe] region request REFUSED: %u regions already asked this frame -- the "
                "consumer's answer stays the last one delivered", kMaxRegions);
            m_refusedLogged = true;
        }
        return;
    }
    m_requests.push_back(r);
}

namespace {

// THE ONE RECONSTRUCTION (WaterBank.hlsl LoadBilinearClamp at texel (u Nx, (1 - v) Ny)), over any
// store: `at(ix, iy, eta, uv4)` answers one grid texel or says it does not hold it.
template <class At>
bool Reconstruct(double tx, double ty, uint32_t nx, uint32_t ny, At at, SweSolver::Deviation& out) {
    const double fx = tx - 0.5, fy = ty - 0.5;
    const double bx = std::floor(fx), by = std::floor(fy);
    const double frx = fx - bx, fry = fy - by;
    double eta = 0.0, uv[4] = {0.0, 0.0, 0.0, 0.0};
    for (int k = 0; k < 4; ++k) {
        const int ix = std::clamp(int(bx) + (k & 1), 0, int(nx) - 1);
        const int iy = std::clamp(int(by) + (k >> 1), 0, int(ny) - 1);
        float e = 0.0f, c[4] = {0.0f, 0.0f, 0.0f, 0.0f};
        if (!at(ix, iy, e, c)) return false;
        const double w = ((k & 1) ? frx : 1.0 - frx) * ((k >> 1) ? fry : 1.0 - fry);
        eta += w * double(e);
        for (int j = 0; j < 4; ++j) uv[j] += w * double(c[j]);
    }
    out.dEta = float(eta);
    out.u = float(uv[0]);
    out.v = float(uv[1]);
    out.currentValid = uv[3] > 0.5;   // the kernel's `s.w > 0.5`
    return true;
}

}  // namespace

bool SweSolver::DeviationAt(double latDeg, double lonDeg, Deviation& out) const {
    double tx = 0.0, ty = 0.0;
    if (m_regions.empty() || !TexelOf(latDeg, lonDeg, tx, ty)) return false;
    for (const DeliveredRegion& r : m_regions) {
        const auto at = [&r](int ix, int iy, float& e, float c[4]) {
            if (ix < int(r.rect.x0) || iy < int(r.rect.y0) || ix >= int(r.rect.x0 + r.rect.w) ||
                iy >= int(r.rect.y0 + r.rect.h)) {
                return false;
            }
            const size_t i = size_t(iy - int(r.rect.y0)) * r.rect.w + size_t(ix - int(r.rect.x0));
            e = r.eta[i];
            for (int j = 0; j < 4; ++j) c[j] = r.uv[i * 4 + j];
            return true;
        };
        if (Reconstruct(tx, ty, m_cb.nx, m_cb.ny, at, out)) {
            out.asOf = r.asOf;
            return true;
        }
    }
    return false;
}

bool SweSolver::DeviationFromFields(double latDeg, double lonDeg, const std::vector<float>& eta,
                                    uint32_t etaRowW, const std::vector<float>& uv4,
                                    uint32_t uvRowW, double asOf, Deviation& out) const {
    double tx = 0.0, ty = 0.0;
    if (eta.empty() || uv4.empty() || !TexelOf(latDeg, lonDeg, tx, ty)) return false;
    const auto at = [&](int ix, int iy, float& e, float c[4]) {
        const size_t ie = size_t(iy) * etaRowW + size_t(ix);
        const size_t iu = (size_t(iy) * uvRowW + size_t(ix)) * 4;
        if (ie >= eta.size() || iu + 3 >= uv4.size()) return false;
        e = eta[ie];
        for (int j = 0; j < 4; ++j) c[j] = uv4[iu + j];
        return true;
    };
    if (!Reconstruct(tx, ty, m_cb.nx, m_cb.ny, at, out)) return false;
    out.asOf = asOf;
    return true;
}

void SweSolver::CollectRegions(Gpu& gpu) {
    if (!m_readback.Ready()) return;
    const uint32_t slot = gpu.FrameIndex();
    if (m_readback.BeginSlot(slot) == 0) return;
    // The copies arrive in pairs: tag 2k is request k's eta, 2k + 1 its current.
    std::vector<DeliveredRegion> fresh;
    for (const hal::RegionReadback::Region& g : m_readback.Delivered()) {
        const size_t k = g.tag / 2u;
        if (fresh.size() <= k) fresh.resize(k + 1);
        DeliveredRegion& d = fresh[k];
        d.rect = {g.x0, g.y0, g.w, g.h};
        d.asOf = m_slotAsOf[slot];
        const size_t n = size_t(g.w) * g.h;
        if ((g.tag & 1u) == 0u && g.bytes.size() == n * 4) {
            d.eta.resize(n);
            memcpy(d.eta.data(), g.bytes.data(), n * 4);
        } else if ((g.tag & 1u) == 1u && g.bytes.size() == n * 8) {
            d.uv.resize(n * 4);
            const uint8_t* b = g.bytes.data();
            for (size_t i = 0; i < n * 4; ++i) {
                uint16_t h = 0;
                memcpy(&h, b + i * 2, 2);
                d.uv[i] = HalfToFloat(h);
            }
        }
    }
    // Newest first: this slot's answers ahead of every older one; a pair that did not come back
    // whole is not an answer.
    for (auto it = fresh.rbegin(); it != fresh.rend(); ++it) {
        if (it->eta.empty() || it->uv.size() != it->eta.size() * 4) continue;
        m_regions.insert(m_regions.begin(), std::move(*it));
    }
    if (m_regions.size() > size_t(kMaxRegions) * Gpu::kFrameCount) {
        m_regions.resize(size_t(kMaxRegions) * Gpu::kFrameCount);
    }
}

void SweSolver::ServeRegions(hal::CommandContext& cmd, Gpu& gpu,
                             const std::function<void(D3D12_RESOURCE_STATES)>& etaTo,
                             const std::function<void(D3D12_RESOURCE_STATES)>& uvTo) {
    if (!m_readback.Ready() || m_requests.empty()) return;
    etaTo(D3D12_RESOURCE_STATE_COPY_SOURCE);
    uvTo(D3D12_RESOURCE_STATE_COPY_SOURCE);
    uint32_t k = 0;
    for (const RegionRect& r : m_requests) {
        const bool e = m_readback.CopyRegion(cmd, m_eta.Res(), 0, DXGI_FORMAT_R32_FLOAT, r.x0, r.y0,
                                             r.w, r.h, 2u * k);
        const bool u = e && m_readback.CopyRegion(cmd, m_uvBank.Res(), 0,
                                                  DXGI_FORMAT_R16G16B16A16_FLOAT, r.x0, r.y0, r.w,
                                                  r.h, 2u * k + 1u);
        if (!u) {
            if (!m_refusedLogged) {
                Log("[swe] region readback slot FULL at request %u -- the rest keep their last answers",
                    k);
                m_refusedLogged = true;
            }
            break;
        }
        ++k;
    }
    m_slotAsOf[gpu.FrameIndex()] = m_simTime;
    m_requests.clear();
}

void SweSolver::RecordReset(hal::CommandContext& cmd, Gpu& gpu) {
    PixScope scope(cmd.Native(), "swe.reset (state -> the analytic tide plane)");
    cmd.ComputeRoot(m_rs.Get());
    LogCbFingerprint();
    cmd.ComputeConstants(0, m_cb);
    cmd.ComputeTable(2, m_table.Base());

    const auto& el = m_eta.ResidentList();
    cmd.ComputeSrvAt(1, gpu.PushConstants(el.data(), el.size() * 4));
    cmd.Pipeline(m_clearEta.Get());
    cmd.Dispatch(m_eta.TileW() / 16, m_eta.TileH() / 16, static_cast<UINT>(el.size()));

    const auto& fl = m_flux.ResidentList();
    cmd.ComputeSrvAt(1, gpu.PushConstants(fl.data(), fl.size() * 4));
    cmd.Pipeline(m_clearFlux.Get());
    cmd.Dispatch(m_flux.TileW() / 16, m_flux.TileH() / 16, static_cast<UINT>(fl.size()));

    // Full-surface uv clear (fresh texture memory is undefined; NULL-region texels must read
    // invalid so the sea falls back to the analytic jet there).
    cmd.Pipeline(m_uvClear.Get());
    cmd.Dispatch((m_cb.nx + 15) / 16, (m_cb.ny + 15) / 16, 1);

    // Three UAV barriers in ONE call, in this order (M12 step 3f: the context's multi-barrier
    // form issues the same three in one ResourceBarrier, as this site wrote them).
    cmd.UavBarriers({m_eta.Res(), m_flux.Res(), m_uvBank.Res()});
}

int SweSolver::Record(hal::CommandContext& cmd, Gpu& gpu, double simUnix, float tideNavd) {
    return Record(cmd, gpu, simUnix, tideNavd, kMaxSubsteps);
}

int SweSolver::Record(hal::CommandContext& cmd, Gpu& gpu, double simUnix, float tideNavd,
                      int maxSub) {
    if (!m_ready) return 0;
    PixScope scope(cmd.Native(), "swe (sparse shallow-water: flux/height substeps + derive)");

    // Spin-up runs on the raw upload list, which has no descriptor heap bound; the frame list
    // already has this exact heap, so re-setting it is harmless there.
    cmd.BindHeaps();

    auto etaTo = [&](D3D12_RESOURCE_STATES to) {
        if (m_etaState == to) return;
        cmd.Barrier(m_eta.Res(), m_etaState, to);
        m_etaState = to;
    };
    auto uvTo = [&](D3D12_RESOURCE_STATES to) {
        if (m_uvState == to) return;
        cmd.Barrier(m_uvBank.Res(), m_uvState, to);
        m_uvState = to;
    };
    // The region readback: what this slot's previous frame copied is an answer now (the frame
    // ring waited on that frame's fence before this one began). Frame ring only.
    if (cmd.Who() == hal::Owner::Frame) CollectRegions(gpu);
    etaTo(D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
    uvTo(D3D12_RESOURCE_STATE_UNORDERED_ACCESS);

    // Clock policy. A true scrub (arrow keys, --start) means the state belongs to a time that
    // no longer exists: snap to the analytic plane. Mere STARVATION (a fast time scale asking
    // for more substeps than the frame budget allows) keeps the state -- the field stays
    // continuous, boundary-forced by the current tide -- and just re-anchors the clock, i.e.
    // time-dilated hydrodynamics instead of a visible reset flicker.
    const double drift = std::abs(simUnix - m_simTime);
    if (m_pendingReset || drift > 3600.0) {
        RecordReset(cmd, gpu);
        m_simTime = simUnix;
        m_pendingReset = false;
        m_lastTideTime = 0;   // no rate across a reset
    } else if (drift > 900.0) {
        m_simTime = simUnix - maxSub * static_cast<double>(m_dt);
        m_lastTideTime = 0;
    }

    // M6r, the prism source: the tide plane's rate of rise, finite-differenced across batch
    // starts on the SIM clock. Booked as per-substep debt in the height kernel so the basin's
    // filling volume must arrive through the gap instead of appearing by construction.
    // Clamp: M2 at this range peaks near 2.2e-4 m/s; anything past 6e-4 is a scrub artifact.
    float tideRate = 0.0f;
    if (m_lastTideTime > 0 && m_simTime > m_lastTideTime + 1.0e-6) {
        tideRate = std::clamp(
            static_cast<float>((tideNavd - m_lastTideNavd) / (m_simTime - m_lastTideTime)),
            -6.0e-4f, 6.0e-4f);
    }
    m_lastTideNavd = tideNavd;
    m_lastTideTime = m_simTime;
    m_cb.tideRate = tideRate;

    int n = static_cast<int>((simUnix - m_simTime) / m_dt);
    n = (std::max)(0, (std::min)(n, maxSub));

    m_cb.tideNavd = tideNavd;
    m_cb.riverDEta = m_westDEta;   // west-boundary target deviation (river tide - ocean tide)
    m_cb.riverBox[3] = m_southDEta;
    // M6r: Flather's u_ext = the prescribed transport over the LIVE wet section. The clamp is
    // a sanity rail (real river-tide currents at this reach peak near 1 m/s).
    float area = 0.0f;
    const float lvl = tideNavd + m_westDEta;
    for (const float bed : m_westBed) area += (std::max)(lvl - bed, 0.0f) * m_cb.dy;
    m_cb.westUext =
        (area > 1.0f) ? std::clamp(m_westQ / area, -1.5f, 1.5f) : 0.0f;
    m_westArea = area;

    cmd.ComputeRoot(m_rs.Get());
    LogCbFingerprint();
    cmd.ComputeConstants(0, m_cb);
    cmd.ComputeTable(2, m_table.Base());
    const auto& el = m_eta.ResidentList();
    const auto& fl = m_flux.ResidentList();
    const D3D12_GPU_VIRTUAL_ADDRESS elVa = gpu.PushConstants(el.data(), el.size() * 4);
    const D3D12_GPU_VIRTUAL_ADDRESS flVa = gpu.PushConstants(fl.data(), fl.size() * 4);

    for (int i = 0; i < n; ++i) {
        cmd.ComputeSrvAt(1, flVa);
        cmd.Pipeline(m_fluxK.Get());
        cmd.Dispatch(m_flux.TileW() / 16, m_flux.TileH() / 16, static_cast<UINT>(fl.size()));
        cmd.UavBarrier(m_flux.Res());

        cmd.ComputeSrvAt(1, elVa);
        cmd.Pipeline(m_heightK.Get());
        cmd.Dispatch(m_eta.TileW() / 16, m_eta.TileH() / 16, static_cast<UINT>(el.size()));
        cmd.UavBarrier(m_eta.Res());
    }
    m_simTime += n * m_dt;

    cmd.ComputeSrvAt(1, elVa);
    cmd.Pipeline(m_deriveK.Get());
    cmd.Dispatch(m_eta.TileW() / 16, m_eta.TileH() / 16, static_cast<UINT>(el.size()));

    // M9h: grad(flow), dispatched over the DERIVED bank's own resident list -- the eta list
    // dilated by the stencil apron. uv must be readable by this kernel first, and the write
    // target is a different bank, so the UAV barrier on uv is the whole synchronisation.
    {
        cmd.UavBarrier(m_uvBank.Res());
        const auto& gl = m_velGrad.ResidentList();
        if (!gl.empty()) {
            cmd.ComputeSrvAt(1, gpu.PushConstants(gl.data(), gl.size() * 4));
            cmd.Pipeline(m_velGradK.Get());
            cmd.Dispatch(m_eta.TileW() / 16, m_eta.TileH() / 16, static_cast<UINT>(gl.size()));
            // mip 0 just changed, so the coarse levels are stale. Refill them: an unfilled
            // coarse level is real memory reading zero, which looks exactly like "the flow is
            // irrotational here" -- a lie the lens would render in perfectly good faith.
            cmd.UavBarrier(m_velGrad.Res());
            if (m_sc) m_velGrad.BuildChain(gpu, *m_sc, m_shaderDir, cmd);
        }
    }

    // This frame's region requests, copied from the state just stepped (frame ring only).
    if (cmd.Who() == hal::Owner::Frame) ServeRegions(cmd, gpu, etaTo, uvTo);

    // Leave eta + uv sampleable by the domain and pixel shaders.
    etaTo(D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE |
          D3D12_RESOURCE_STATE_PIXEL_SHADER_RESOURCE);
    uvTo(D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE |
         D3D12_RESOURCE_STATE_PIXEL_SHADER_RESOURCE);

    char s[96];
    snprintf(s, sizeof(s), "swe %ut %.0f/%.0fMB %dss", m_eta.ResidentCount() + m_flux.ResidentCount(),
             (m_eta.ResidentBytes() + m_flux.ResidentBytes()) / 1048576.0,
             (m_eta.VirtualBytes() + m_flux.VirtualBytes()) / 1048576.0, n);
    stats = s;
    return n;
}

std::vector<uint8_t> SweSolver::ReadFluxRaw(Gpu& gpu, uint32_t* outW, uint32_t* outH,
                                            uint32_t* outPitch) {
    GpuTexture wrap;
    wrap.res = m_flux.Res();
    wrap.format = DXGI_FORMAT_R32G32_FLOAT;
    wrap.width = m_flux.TilesX() * m_flux.TileW();
    wrap.height = m_flux.TilesY() * m_flux.TileH();
    wrap.state = D3D12_RESOURCE_STATE_UNORDERED_ACCESS;   // flux stays UA between frames
    *outW = wrap.width;
    *outH = wrap.height;
    return gpu.ReadbackTexture(wrap, outPitch);
}

bool SweSolver::TraceWestFace(Gpu& gpu, std::vector<WestFaceRow>& rows) {
    if (!m_ready || !m_bedBound || !m_sc) return false;
    if (!m_westK) {
        // The solver's own layout with the trace's target beside it: b0, t0 the tile list, and a
        // table of the solver's six views and u5.
        m_westRs = hal::RootLayout{}
                       .Cbv(0)
                       .Srv(0)
                       .Table({hal::SrvRange(1, 2), hal::UavRange(0, 4), hal::UavRange(5, 1)})
                       .Build(gpu, "swe.westtrace");
        m_westRs->SetName(L"swe west trace root signature");
        m_westK = hal::BuildCompute(gpu, m_westRs.Get(),
                                    m_sc->Compile(m_shaderDir + L"/Swe.hlsl", L"CsSweFlux", L"cs_6_0",
                                                  {L"SWE_WEST_TRACE=1"}),
                                    "swe.westtrace");
        if (!m_westK) return false;
        m_westTex = gpu.CreateTexture2D(4, m_cb.ny, DXGI_FORMAT_R32G32B32A32_FLOAT,
                                        D3D12_RESOURCE_FLAG_ALLOW_UNORDERED_ACCESS,
                                        D3D12_RESOURCE_STATE_UNORDERED_ACCESS,
                                        L"swe.westtrace (the Flather face's terms)");
        m_westTable = hal::Table::Alloc(gpu, 7, "swe.westtrace");
        m_westTable.SrvArray(0, m_bedArr, DXGI_FORMAT_R16_FLOAT, 0, UINT32_MAX, m_bedMips);
        m_westTable.SrvArray(1, m_bedRes, DXGI_FORMAT_R8_UNORM, 0, UINT32_MAX, 1);
        m_westTable.Uav2D(2, m_eta.Res(), DXGI_FORMAT_R32_FLOAT);
        m_westTable.Uav2D(3, m_flux.Res(), DXGI_FORMAT_R32G32_FLOAT);
        m_westTable.UavArray(4, m_uvBank.Res(), DXGI_FORMAT_R16G16B16A16_FLOAT, 0, 1);
        m_westTable.UavArray(5, m_velGrad.Res(), DXGI_FORMAT_R16G16B16A16_FLOAT, 0, 1);
        m_westTable.Uav2D(6, m_westTex.res.Get(), DXGI_FORMAT_R32G32B32A32_FLOAT);
    }
    {
        hal::CommandContext up(gpu, gpu.BeginUpload(), hal::Owner::Upload);
        up.BindHeaps();
        if (m_etaState != D3D12_RESOURCE_STATE_UNORDERED_ACCESS) {
            up.Barrier(m_eta.Res(), m_etaState, D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
            m_etaState = D3D12_RESOURCE_STATE_UNORDERED_ACCESS;
        }
        if (m_uvState != D3D12_RESOURCE_STATE_UNORDERED_ACCESS) {
            up.Barrier(m_uvBank.Res(), m_uvState, D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
            m_uvState = D3D12_RESOURCE_STATE_UNORDERED_ACCESS;
        }
        up.ComputeRoot(m_westRs.Get());
        up.ComputeConstants(0, m_cb);
        const auto& fl = m_flux.ResidentList();
        up.ComputeSrvAt(1, gpu.PushConstants(fl.data(), fl.size() * 4));
        up.ComputeTable(2, m_westTable.Base());
        up.Pipeline(m_westK.Get());
        up.Dispatch(m_flux.TileW() / 16, m_flux.TileH() / 16, static_cast<UINT>(fl.size()));
        gpu.EndUpload();
    }
    uint32_t pitch = 0;
    const std::vector<uint8_t> raw = gpu.ReadbackTexture(m_westTex, &pitch);
    rows.assign(m_cb.ny, WestFaceRow{});
    for (uint32_t y = 0; y < m_cb.ny && size_t(y) * pitch + sizeof(WestFaceRow) <= raw.size(); ++y) {
        memcpy(&rows[y], &raw[size_t(y) * pitch], sizeof(WestFaceRow));   // 4 texels = 16 floats
    }
    return true;
}

double SweSolver::WestTransport(Gpu& gpu) {
    if (!m_ready) return 0.0;
    uint32_t w = 0, h = 0, pitch = 0;
    const std::vector<uint8_t> raw = ReadFluxRaw(gpu, &w, &h, &pitch);
    double q = 0.0;
    // Column 0's east face is the Flather face (Swe.hlsl CsSweFlux, t.x < riverBox.x); a NULL
    // tile reads zero, which is no transport.
    for (uint32_t y = 0; y < m_cb.ny && size_t(y) * pitch + 4 <= raw.size(); ++y) {
        float qx = 0.0f;
        memcpy(&qx, &raw[size_t(y) * pitch], 4);
        q += qx;
    }
    return q;
}

void SweSolver::ReadFields(Gpu& gpu, std::vector<float>& etaOut, uint32_t& etaW,
                           uint32_t& etaH, std::vector<float>& uv4Out, uint32_t& uvW,
                           uint32_t& uvH) {
    etaW = etaH = uvW = uvH = 0;
    if (!m_ready) return;
    uint32_t etaPitch = 0, uvPitch = 0;
    GpuTexture wrap;
    wrap.res = m_eta.Res();
    wrap.format = DXGI_FORMAT_R32_FLOAT;
    wrap.width = m_eta.TilesX() * m_eta.TileW();
    wrap.height = m_eta.TilesY() * m_eta.TileH();
    wrap.state = m_etaState;
    const std::vector<uint8_t> etaData = gpu.ReadbackTexture(wrap, &etaPitch);
    m_etaState = wrap.state;
    GpuTexture uvWrap;
    uvWrap.res = m_uvBank.Res();
    uvWrap.format = DXGI_FORMAT_R16G16B16A16_FLOAT;
    uvWrap.width = m_uvBank.TilesX() * m_uvBank.TileW();
    uvWrap.height = m_uvBank.TilesY() * m_uvBank.TileH();
    uvWrap.state = m_uvState;
    const std::vector<uint8_t> uvData = gpu.ReadbackTexture(uvWrap, &uvPitch);
    m_uvState = uvWrap.state;
    UnpackFields(m_cb.nx, m_cb.ny, etaData, etaPitch, uvData, uvPitch, etaOut, etaW, etaH, uv4Out, uvW, uvH);
}

SweSolver::FieldsRead SweSolver::ReadFieldsDetach() {
    FieldsRead r;
    r.eta = m_rbEta;
    r.uv = m_rbUv;
    r.nx = m_cb.nx;
    r.ny = m_cb.ny;
    m_rbEta = Gpu::TextureReadback{};
    m_rbUv = Gpu::TextureReadback{};
    return r;
}

void SweSolver::UnpackRead(Gpu& gpu, FieldsRead& r, std::vector<float>& etaOut, uint32_t& etaW,
                           uint32_t& etaH, std::vector<float>& uv4Out, uint32_t& uvW, uint32_t& uvH) {
    etaW = etaH = uvW = uvH = 0;
    if (!r.eta.Pending() || !r.uv.Pending()) return;
    const uint32_t etaPitch = r.eta.rowPitch, uvPitch = r.uv.rowPitch;
    const std::vector<uint8_t> etaData = gpu.ReadbackTake(r.eta);
    const std::vector<uint8_t> uvData = gpu.ReadbackTake(r.uv);
    UnpackFields(r.nx, r.ny, etaData, etaPitch, uvData, uvPitch, etaOut, etaW, etaH, uv4Out, uvW, uvH);
}

bool SweSolver::ReadFieldsBegin(Gpu& gpu) {
    if (!m_ready || m_rbEta.Pending()) return false;
    GpuTexture wrap;
    wrap.res = m_eta.Res();
    wrap.format = DXGI_FORMAT_R32_FLOAT;
    wrap.width = m_eta.TilesX() * m_eta.TileW();
    wrap.height = m_eta.TilesY() * m_eta.TileH();
    wrap.state = m_etaState;
    m_rbEta = gpu.ReadbackTextureBegin(wrap);
    GpuTexture uvWrap;
    uvWrap.res = m_uvBank.Res();
    uvWrap.format = DXGI_FORMAT_R16G16B16A16_FLOAT;
    uvWrap.width = m_uvBank.TilesX() * m_uvBank.TileW();
    uvWrap.height = m_uvBank.TilesY() * m_uvBank.TileH();
    uvWrap.state = m_uvState;
    m_rbUv = gpu.ReadbackTextureBegin(uvWrap);
    return true;
}

bool SweSolver::ReadFieldsReady(const Gpu& gpu) const {
    return m_rbEta.Pending() && m_rbUv.Pending() && gpu.ReadbackReady(m_rbUv);
}

void SweSolver::ReadFieldsTake(Gpu& gpu, std::vector<float>& etaOut, uint32_t& etaW,
                               uint32_t& etaH, std::vector<float>& uv4Out, uint32_t& uvW,
                               uint32_t& uvH) {
    etaW = etaH = uvW = uvH = 0;
    if (!ReadFieldsReady(gpu)) return;
    const uint32_t etaPitch = m_rbEta.rowPitch, uvPitch = m_rbUv.rowPitch;
    const std::vector<uint8_t> etaData = gpu.ReadbackTake(m_rbEta);
    const std::vector<uint8_t> uvData = gpu.ReadbackTake(m_rbUv);
    UnpackFields(m_cb.nx, m_cb.ny, etaData, etaPitch, uvData, uvPitch, etaOut, etaW, etaH, uv4Out, uvW, uvH);
}

void SweSolver::UnpackFields(uint32_t nx, uint32_t ny, const std::vector<uint8_t>& etaData,
                             uint32_t etaPitch, const std::vector<uint8_t>& uvData, uint32_t uvPitch,
                             std::vector<float>& etaOut, uint32_t& etaW, uint32_t& etaH,
                             std::vector<float>& uv4Out, uint32_t& uvW, uint32_t& uvH) {
    // The eta RESOURCE desc is the LOGICAL grid (nx x ny) -- the tile-multiple padding lives
    // in the atlas' addressing, not the texture dims. (ReadProbes' padded wrap dims were
    // cosmetic; a row loop that trusts them runs off the readback buffer.)
    etaW = nx;
    etaH = ny;
    etaOut.resize(static_cast<size_t>(etaW) * etaH);
    for (uint32_t y = 0; y < etaH; ++y) {
        memcpy(&etaOut[static_cast<size_t>(y) * etaW], &etaData[y * etaPitch], etaW * 4);
    }
    uvW = nx;
    uvH = ny;
    uv4Out.resize(static_cast<size_t>(uvW) * uvH * 4);
    for (uint32_t y = 0; y < uvH; ++y) {
        const uint16_t* px = reinterpret_cast<const uint16_t*>(&uvData[y * uvPitch]);
        for (uint32_t x = 0; x < uvW; ++x) {
            for (int c = 0; c < 4; ++c) {
                uv4Out[(static_cast<size_t>(y) * uvW + x) * 4 + c] =
                    HalfToFloat(px[x * 4 + c]);
            }
        }
    }
}

void SweSolver::ReadProbes(Gpu& gpu, const float* cellPairs, int count, Probe* out) {
    for (int i = 0; i < count; ++i) out[i] = Probe{0, 0, 0, false};
    if (!m_ready || count <= 0) return;

    uint32_t etaPitch = 0, uvPitch = 0;
    GpuTexture wrap;
    wrap.res = m_eta.Res();
    wrap.format = DXGI_FORMAT_R32_FLOAT;
    wrap.width = m_eta.TilesX() * m_eta.TileW();
    wrap.height = m_eta.TilesY() * m_eta.TileH();
    wrap.state = m_etaState;
    const std::vector<uint8_t> etaData = gpu.ReadbackTexture(wrap, &etaPitch);
    m_etaState = wrap.state;
    GpuTexture uvWrap;
    uvWrap.res = m_uvBank.Res();
    uvWrap.format = DXGI_FORMAT_R16G16B16A16_FLOAT;
    uvWrap.width = m_uvBank.TilesX() * m_uvBank.TileW();
    uvWrap.height = m_uvBank.TilesY() * m_uvBank.TileH();
    uvWrap.state = m_uvState;
    const std::vector<uint8_t> uvData = gpu.ReadbackTexture(uvWrap, &uvPitch);
    m_uvState = uvWrap.state;

    for (int i = 0; i < count; ++i) {
        // Row 0 is the NORTHERN edge; cell i spans [i, i + 1).
        const int tx = static_cast<int>(std::floor(cellPairs[i * 2]));
        const int ty = static_cast<int>(std::floor(cellPairs[i * 2 + 1]));
        if (tx < 0 || ty < 0 || tx >= static_cast<int>(m_cb.nx) ||
            ty >= static_cast<int>(m_cb.ny)) {
            continue;
        }
        out[i].dEta = *reinterpret_cast<const float*>(&etaData[ty * etaPitch + tx * 4]);
        const uint16_t* px = reinterpret_cast<const uint16_t*>(&uvData[ty * uvPitch + tx * 8]);
        out[i].u = HalfToFloat(px[0]);
        out[i].v = HalfToFloat(px[1]);
        out[i].valid = HalfToFloat(px[3]) > 0.5f;
    }
}

WaterHold FloodWindow(const float* bed, int nx, int ny, float level, const uint8_t* sponge, bool westFace,
                      bool plantCrossEdge, std::vector<int>* labels) {
    WaterHold h;
    const size_t n = size_t(nx) * size_t(ny);
    const auto face = [&](size_t i) {
        return westFace && i % size_t(nx) == 0 && bed[i] > -9000.0f && bed[i] < 2.0f;
    };
    const auto wet = [&](size_t i) { return (bed[i] > -9000.0f && bed[i] < level) || face(i); };
    const auto edgeCell = [&](int e, int t) {   // e: 0 N, 1 S, 2 W, 3 E; t along that edge
        return (e == 0)   ? size_t(t)
               : (e == 1) ? size_t(ny - 1) * nx + t
               : (e == 2) ? size_t(t) * nx
                          : size_t(t) * nx + size_t(nx - 1);
    };
    for (int r = 0; r < ny; ++r) h.faceCells += face(size_t(r) * nx) ? 1 : 0;
    std::vector<int> label(n, -1);
    std::vector<size_t> q;
    int id = 0;
    for (size_t s = 0; s < n; ++s) {
        if (label[s] >= 0 || !wet(s)) continue;
        WaterPiece p;
        p.id = id;
        p.c0 = nx;
        p.r0 = ny;
        p.c1 = p.r1 = -1;
        bool sea = false, edge = false, crossed[4] = {false, false, false, false};
        q.assign(1, s);
        label[s] = id;
        for (size_t k = 0; k < q.size(); ++k) {
            const size_t i = q[k];
            const int c = int(i % size_t(nx)), r = int(i / size_t(nx));
            ++p.cells;
            p.c0 = (std::min)(p.c0, c);
            p.c1 = (std::max)(p.c1, c);
            p.r0 = (std::min)(p.r0, r);
            p.r1 = (std::max)(p.r1, r);
            sea = sea || sponge[i] != 0;
            p.face = p.face || face(i);
            const auto visit = [&](size_t j) {
                if (label[j] < 0 && wet(j)) {
                    label[j] = id;
                    q.push_back(j);
                }
            };
            if (c > 0) visit(i - 1);
            if (c + 1 < nx) visit(i + 1);
            if (r > 0) visit(i - size_t(nx));
            if (r + 1 < ny) visit(i + size_t(nx));
            const int e = (r == 0) ? 0 : (r == ny - 1) ? 1 : (c == 0) ? 2 : (c == nx - 1) ? 3 : -1;
            edge = edge || e >= 0;
            if (plantCrossEdge && e >= 0 && !crossed[e]) {   // THE PLANT: the flood runs along the
                crossed[e] = true;                            // outside of an edge it reaches
                for (int t = 0; t < ((e < 2) ? nx : ny); ++t) visit(edgeCell(e, t));
            }
        }
        // Where the piece meets the window's edge; for the sponge's water only where the edge is
        // no boundary (not the sponge's columns, not the west face): the crossings a law must open.
        std::vector<WaterPiece::EdgeRun>& runs = sea ? h.crossings : p.runs;
        for (int e = 0; edge && e < 4; ++e) {
            const int len = (e < 2) ? nx : ny;
            WaterPiece::EdgeRun run{"NSWE"[e], -1, -1, 1.0e9f};
            for (int t = 0; t <= len; ++t) {
                const size_t j = (t < len) ? edgeCell(e, t) : 0;
                const bool open = sea && (sponge[j] != 0 || face(j));
                if (t < len && label[j] == id && !open) {
                    if (run.from < 0) run.from = t;
                    run.to = t;
                    run.deepest = (std::min)(run.deepest, bed[j]);
                } else if (run.from >= 0) {
                    runs.push_back(run);
                    run = WaterPiece::EdgeRun{"NSWE"[e], -1, -1, 1.0e9f};
                }
            }
        }
        if (sea) {
            h.joined += p.cells;
        } else {
            h.pieces.push_back(std::move(p));
        }
        h.sea.push_back(sea ? 1 : 0);
        ++id;
    }
    std::stable_sort(h.pieces.begin(), h.pieces.end(),
                     [](const WaterPiece& a, const WaterPiece& b) { return a.cells > b.cells; });
    if (labels) *labels = std::move(label);
    return h;
}

FloodParting CompareFloods(const float* grid, const float* kernel, int nx, int ny, float level,
                           const uint8_t* sponge, bool westFace) {
    FloodParting f;
    std::vector<int> gl, kl;
    const WaterHold g = FloodWindow(grid, nx, ny, level, sponge, westFace, false, &gl);
    const WaterHold k = FloodWindow(kernel, nx, ny, level, sponge, westFace, false, &kl);
    f.gridJoined = g.joined;
    f.kernelJoined = k.joined;
    const size_t n = size_t(nx) * size_t(ny);
    std::vector<int> share(k.sea.size(), 0);   // by the kernel's label: its cells the grid joins
    for (size_t i = 0; i < n; ++i) {
        const bool gj = gl[i] >= 0 && g.sea[size_t(gl[i])];
        const bool kj = kl[i] >= 0 && k.sea[size_t(kl[i])];
        f.gridOnly += (gj && !kj) ? 1 : 0;
        f.kernelOnly += (kj && !gj) ? 1 : 0;
        if (gj && !kj && kl[i] >= 0) ++share[size_t(kl[i])];
    }
    int best = -1;
    for (size_t id = 0; id < share.size(); ++id) {
        if (share[id] > 0 && (best < 0 || share[id] > share[size_t(best)])) best = int(id);
    }
    if (best < 0) return f;
    for (const WaterPiece& p : k.pieces) {
        if (p.id == best) f.piece = p;
    }
    f.piece.cells = share[size_t(best)];
    // Where it parts from the sea: the lowest crest on its way there over the kernel's bed, a
    // minimax walk 4-connected like the kernel's faces; the crest is the walk's highest cell (the
    // one nearest the piece where the walk's top is flat).
    std::vector<float> top(n, 3.0e38f);
    std::vector<int> prev(n, -1);
    using Item = std::pair<float, int>;
    std::priority_queue<Item, std::vector<Item>, std::greater<Item>> open;
    for (size_t i = 0; i < n; ++i) {
        if (kl[i] == best) {
            top[i] = kernel[i];
            open.push({kernel[i], int(i)});
        }
    }
    int goal = -1;
    while (!open.empty()) {
        const Item it = open.top();
        open.pop();
        const size_t i = size_t(it.second);
        if (it.first > top[i]) continue;
        if (kl[i] >= 0 && k.sea[size_t(kl[i])]) {
            goal = it.second;
            break;
        }
        const int c = int(i % size_t(nx)), r = int(i / size_t(nx));
        const int nb[4] = {c > 0 ? it.second - 1 : -1, c + 1 < nx ? it.second + 1 : -1,
                           r > 0 ? it.second - nx : -1, r + 1 < ny ? it.second + nx : -1};
        for (const int j : nb) {
            if (j < 0 || kernel[size_t(j)] < -9000.0f) continue;
            const float m = (std::max)(it.first, kernel[size_t(j)]);
            if (m < top[size_t(j)]) {
                top[size_t(j)] = m;
                prev[size_t(j)] = it.second;
                open.push({m, j});
            }
        }
    }
    if (goal < 0) return f;
    float hi = -3.0e38f;
    for (int i = goal; i >= 0; i = prev[size_t(i)]) {
        if (kernel[size_t(i)] >= hi) {
            hi = kernel[size_t(i)];
            f.crestCell = i;
        }
    }
    f.crestKernel = kernel[size_t(f.crestCell)];
    f.crestGrid = grid[size_t(f.crestCell)];
    return f;
}

void SweSolver::LogHoldsWater(Gpu& gpu, bool traced) {
    if (!m_ready) return;
    BedTrace bt;
    if (traced && TraceBed(gpu, bt) && bt.bed.size() == size_t(m_cb.nx) * m_cb.ny) {
        LogWindowHoldsWater(m_dom, m_cfg, "on the kernel's bed, traced after the whole-bed wait",
                            &bt.bed);
    } else {
        LogWindowHoldsWater(m_dom, m_cfg,
                            "on the CPU bed, the stack at the cells: the whole-bed wait did not run or "
                            "finish, and the kernel's bed may differ");
    }
}

void LogWindowHoldsWater(const SweDomain& dom, const SweConfig& cfg, const char* when,
                         const std::vector<float>* kernelBed) {
    constexpr float kLevel = 1.0f;   // m NAVD88: the level the flood stands at
    const int nx = int(dom.nx), ny = int(dom.ny);
    if (nx < 2 || ny < 2 || dom.elev.size() != size_t(nx) * ny || dom.sponge.size() != dom.elev.size()) return;
    const float dx = float(dom.dx), dz = float(dom.dy);
    const double km2 = double(dx) * dz / 1.0e6;
    const uint8_t* spongeCells = dom.sponge.data();
    const float* grid = dom.elev.data();
    const bool kernel = kernelBed && kernelBed->size() == size_t(nx) * size_t(ny);
    const float* bed = kernel ? kernelBed->data() : grid;
    const WaterHold h = FloodWindow(bed, nx, ny, kLevel, spongeCells, cfg.westBoundary);
    // Places in the domain's chart: metres east of its west side, north of its south side.
    const auto X = [&](int c) { return (c + 0.5f) * dx; };
    const auto Z = [&](int r) { return (ny - r - 0.5f) * dz; };
    int all = 0, atEdge = 0, edgeCells = 0;
    for (const WaterPiece& p : h.pieces) {
        all += p.cells;
        if (!p.runs.empty()) {
            ++atEdge;
            edgeCells += p.cells;
        }
    }
    char b[640];
    const auto runText = [&](const WaterPiece::EdgeRun& r) {   // z ascending on the W and E edges
        char t[96];
        const bool ns = r.edge == 'N' || r.edge == 'S';
        snprintf(t, sizeof(t), " %c %s %.0f..%.0f (deepest %+.2f)", r.edge, ns ? "x" : "z",
                 ns ? X(r.from) : Z(r.to), ns ? X(r.to) : Z(r.from), r.deepest);
        return std::string(t);
    };
    snprintf(b, sizeof(b),
             "[swe] %s window (%s): does it hold its water? flooded at %+.2f m NAVD, 4-connected, "
             "from the sponge (%.0f m in from an open side) and the west face (%d wet-capable cells): %.2f km^2 "
             "joined to the sponge; not joined to it: %zu pieces, %.2f km^2, %d at the window's "
             "edge (%.2f km^2)",
             cfg.name ? cfg.name : "?", when, kLevel, double(cfg.spongeM), h.faceCells, h.joined * km2,
             h.pieces.size(), all * km2, atEdge, edgeCells * km2);
    std::string s = b;
    int shown = 0;
    for (const WaterPiece& p : h.pieces) {
        if (p.runs.empty()) continue;
        if (++shown > 4) {
            s += " | ...";
            break;
        }
        snprintf(b, sizeof(b), " | %.3f km^2 %s, x %.0f..%.0f z %.0f..%.0f m, meets", p.cells * km2,
                 p.face ? "held by the west face alone" : "joined to none", X(p.c0), X(p.c1),
                 Z(p.r1), Z(p.r0));
        s += b;
        for (size_t k = 0; k < p.runs.size() && k < 4; ++k) s += (k ? "," : "") + runText(p.runs[k]);
        if (p.runs.size() > 4) s += " +" + std::to_string(p.runs.size() - 4) + " more";
    }
    std::vector<WaterPiece::EdgeRun> cross = h.crossings;   // the deepest first
    std::stable_sort(cross.begin(), cross.end(),
                     [](const WaterPiece::EdgeRun& u, const WaterPiece::EdgeRun& w) { return u.deepest < w.deepest; });
    if (!cross.empty()) {
        s += " | the sponge's water meets the edge where it is no boundary: " + std::to_string(cross.size()) +
             " runs, the deepest";
        for (size_t k = 0; k < cross.size() && k < 4; ++k) s += (k ? "," : "") + runText(cross[k]);
    }
    for (const WaterPiece& p : h.pieces) {
        if (!p.runs.empty()) continue;
        snprintf(b, sizeof(b), " | the largest away from the edge: %.3f km^2 %s, x %.0f..%.0f z %.0f..%.0f m",
                 p.cells * km2, p.face ? "held by the west face alone" : "joined to none", X(p.c0),
                 X(p.c1), Z(p.r1), Z(p.r0));
        s += b;
        break;
    }
    Log("%s", s.c_str());
    if (!kernel) return;
    // Beside it, the survey's grid flooded by the same law: where the two beds' water parts.
    const FloodParting f = CompareFloods(grid, bed, nx, ny, kLevel, spongeCells, cfg.westBoundary);
    const char* name = cfg.name ? cfg.name : "?";
    if (f.gridOnly == 0 && f.kernelOnly == 0) {
        Log("[swe] %s window: the survey's grid, flooded by the same law, joins the same water to "
            "the sponge as the kernel's bed (%.2f km^2)",
            name, f.gridJoined * km2);
        return;
    }
    snprintf(b, sizeof(b),
             "[swe] %s window: the survey's grid joins %.2f km^2 that the kernel's bed does not (its "
             "flood joins %.2f km^2 to the sponge, the kernel's %.2f), and the kernel's bed joins "
             "%.2f km^2 that the grid does not",
             name, f.gridOnly * km2, f.gridJoined * km2, f.kernelJoined * km2, f.kernelOnly * km2);
    std::string t = b;
    if (f.piece.id >= 0) {
        snprintf(b, sizeof(b), "; the largest of it, %.3f km^2 of the kernel's water %s (x %.0f..%.0f z %.0f..%.0f m),",
                 f.piece.cells * km2, f.piece.face ? "held by the west face alone" : "joined to none",
                 X(f.piece.c0), X(f.piece.c1), Z(f.piece.r1), Z(f.piece.r0));
        t += b;
        if (f.crestCell >= 0) {
            const int cc = f.crestCell % nx, cr = f.crestCell / nx;
            snprintf(b, sizeof(b),
                     " parts from the sea at cell (%d, %d), world (%.0f, %.0f): the kernel's bed "
                     "%+.2f m, the grid's %+.2f m -- the lowest crest on the kernel's way to the sea",
                     cc, cr, X(cc), Z(cr), f.crestKernel, f.crestGrid);
            t += b;
        } else {
            t += " and the kernel's bed has no way from it to the sea";
        }
    }
    Log("%s", t.c_str());
}

// The instrument's gate, on a made-up window 64 x 40: land at +5 m, the open sea in the last 8
// columns (the sponge from column 56), and a channel at -3 m from the west edge (row 20) that runs
// east to column 20, turns north out through the north edge, comes back in at column 30 and runs on
// to the sea -- the Merrimack's bend in miniature. WHOLE closes the bend inside the grid (row 2).
bool RunWaterHoldSelfTest() {
    constexpr int nx = 64, ny = 40, kSponge = 56;
    const auto make = [&](bool whole) {
        std::vector<float> b(size_t(nx) * ny, 5.0f);
        for (int r = 0; r < ny; ++r) {
            for (int c = kSponge; c < nx; ++c) b[size_t(r) * nx + c] = -10.0f;
        }
        for (int t = 0; t <= 20; ++t) {
            b[size_t(20) * nx + t] = -3.0f;   // the channel from the west edge
            b[size_t(t) * nx + 20] = -3.0f;   // north to the edge
            b[size_t(t) * nx + 30] = -3.0f;   // and back from it
        }
        for (int c = 30; c < kSponge; ++c) b[size_t(20) * nx + c] = -3.0f;
        for (int c = 20; whole && c <= 30; ++c) b[size_t(2) * nx + c] = -3.0f;
        return b;
    };
    std::vector<uint8_t> spongeMask(size_t(nx) * ny, 0);   // the sponge's columns
    for (int r = 0; r < ny; ++r) {
        for (int c = kSponge; c < nx; ++c) spongeMask[size_t(r) * nx + c] = 1;
    }
    const uint8_t* sp = spongeMask.data();
    int checks = 0;
    bool ok = true;
    const auto expect = [&](bool cond, const char* what) {
        ++checks;
        if (!cond) {
            ok = false;
            Log("[waterhold]   FAIL %s", what);
        }
    };
    const std::vector<float> cut = make(false), whole = make(true);
    const WaterHold a = FloodWindow(cut.data(), nx, ny, 1.0f, sp, false);
    expect(a.pieces.size() == 1 && a.pieces[0].cells == 41 && !a.pieces[0].face,
           "a channel cut by the north edge is reported: one piece of 41 cells, joined to none");
    bool north = false, west = false;
    for (size_t k = 0; !a.pieces.empty() && k < a.pieces[0].runs.size(); ++k) {
        const WaterPiece::EdgeRun& r = a.pieces[0].runs[k];
        north = north || (r.edge == 'N' && r.from == 20 && r.to == 20 && r.deepest == -3.0f);
        west = west || (r.edge == 'W' && r.from == 20 && r.to == 20);
    }
    expect(north && west, "it meets the north edge at column 20 (deepest -3) and the west edge at row 20");
    expect(a.crossings.size() == 1 && a.crossings[0].edge == 'N' && a.crossings[0].from == 30 &&
               a.crossings[0].to == 30,
           "the sea's water meets the edge where it is no boundary once: the north edge at column 30");
    const WaterHold f = FloodWindow(cut.data(), nx, ny, 1.0f, sp, true);
    expect(f.faceCells == 1 && f.pieces.size() == 1 && f.pieces[0].face,
           "with a west face, the cut channel is held by the face alone");
    const WaterHold w = FloodWindow(whole.data(), nx, ny, 1.0f, sp, false);
    expect(w.pieces.empty() && w.joined > 0, "a whole channel is not reported");
    const WaterHold p = FloodWindow(cut.data(), nx, ny, 1.0f, sp, false, true);
    expect(p.pieces.empty(),
           "PLANT (the flood crosses the edge): the cut channel joins the sea, so the first check "
           "would fail on it -- caught");
    // THE TWO BEDS: the grid (the bend whole) against a kernel's bed with a band raised to +2 m
    // along the north edge, as the fade into the 15-arcsecond relief raises one (finding 68).
    std::vector<float> raised = whole;
    for (int r = 0; r < 4; ++r) {
        for (int c = 0; c < kSponge; ++c) {
            raised[size_t(r) * nx + c] = (std::max)(raised[size_t(r) * nx + c], 2.0f);
        }
    }
    const WaterHold rk = FloodWindow(raised.data(), nx, ny, 1.0f, sp, false);
    expect(w.pieces.empty() && rk.pieces.size() == 1 && rk.pieces[0].cells == 37,
           "two beds: the reach is joined on the grid and cut on the raised bed (one piece, 37 cells)");
    const FloodParting fp = CompareFloods(whole.data(), raised.data(), nx, ny, 1.0f, sp, false);
    expect(fp.gridOnly == 54 && fp.kernelOnly == 0 && fp.piece.cells == 37 && fp.crestCell >= 0 &&
               fp.crestCell / nx < 4 && fp.crestKernel == 2.0f && fp.crestGrid == -3.0f,
           "the grid joins 54 cells the raised bed does not, and the reach parts from the sea at a "
           "band cell: +2 on the kernel's bed, -3 on the grid's");
    if (ok) {
        Log("[waterhold] ---- PASS (%d checks): a channel the window's edge cuts is reported with "
            "where it meets the edge, held by the west face alone when it has one, a whole one is "
            "not, and the plant (the flood across the edge) is caught",
            checks);
    } else {
        Log("[waterhold] ---- FAIL");
    }
    return ok;
}

}  // namespace ga
