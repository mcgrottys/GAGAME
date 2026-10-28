#include "core/Planet.h"
#include "sim/SweSolver.h"

#include "hal/PixEvents.h"
#include "hal/Pipeline.h"
#include "hal/Root.h"
#include "hal/Views.h"

#include <algorithm>
#include <chrono>
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
}

bool SweDomain::CellOfDir(const double P[3], double& tx, double& ty) const {
    const double pa = Dot3(P, A);
    if (!Ready() || !(pa > 0.0)) return false;
    tx = Dot3(P, U) / pa - ox;
    ty = Dot3(P, V) / pa - oy;
    return true;
}

bool SweDomain::CellOf(double latDeg, double lonDeg, double& tx, double& ty) const {
    double d[3];
    DirOf(latDeg, lonDeg, d);
    return CellOfDir(d, tx, ty);
}

void SweDomain::LatLonOf(double tx, double ty, double& latDeg, double& lonDeg) const {
    const double s = (tx + ox - 0.5 * double(nx)) * dx / R, t = (0.5 * double(ny) - (ty + oy)) * dy / R;
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

bool SweDomain::SeaEdgeCell(int side, int a) const {
    const float e = elev[EdgeCell(side, a)];
    if (e < -9000.0f) return false;
    for (const FaceCells& f : faces) {
        if (f.side == side && a >= f.c0 && a <= f.c1 && e < 2.0f) return false;   // the river's
    }
    return true;
}

std::vector<uint8_t> SweDomain::FaceMask() const {
    std::vector<uint8_t> m(size_t(nx) * ny, 0);
    for (const FaceCells& f : faces) {
        for (int a = f.c0; a <= f.c1; ++a) {
            const size_t i = EdgeCell(f.side, a);
            if (elev[i] > -9000.0f && elev[i] < 2.0f) m[i] = 1;
        }
    }
    return m;
}

void SweDomain::Bound(const SweConfig& cfg) {
    faces.clear();
    if (!Ready() || elev.size() != size_t(nx) * ny) return;
    // THE RIVER FACES' CELLS: the edge cell nearest the river's crossing that can be wet (bed under
    // +2 m, the kernel's face test), and the run of such cells along the side about it -- the
    // river's section where the domain cuts it.
    for (const SweConfig::RiverFace& rf : cfg.faces) {
        if (rf.side < 0 || rf.side > 3 || int(faces.size()) >= kMaxRiverFaces) continue;
        double tx = 0.0, ty = 0.0;
        if (!CellOf(rf.lat, rf.lon, tx, ty)) continue;
        const int len = rf.side < 2 ? int(nx) : int(ny);
        const int a0 = std::clamp(int(std::floor(rf.side < 2 ? tx : ty)), 0, len - 1);
        const auto capable = [&](int a) {
            const float e = elev[EdgeCell(rf.side, a)];
            return e > -9000.0f && e < 2.0f;
        };
        int at = -1;
        for (int d = 0; d < len && at < 0; ++d) {
            if (a0 - d >= 0 && capable(a0 - d)) at = a0 - d;
            else if (a0 + d < len && capable(a0 + d)) at = a0 + d;
        }
        if (at < 0) {
            Log("[swe] %s: %s crosses the %c side, which has no cell that can be wet -- a wall", cfg.name,
                rf.name.c_str(), "NSWE"[rf.side]);
            continue;
        }
        FaceCells f{rf.side, at, at, rf.name};
        while (f.c0 > 0 && capable(f.c0 - 1)) --f.c0;
        while (f.c1 + 1 < len && capable(f.c1 + 1)) ++f.c1;
        faces.push_back(f);
        Log("[swe] %s: %s's face on the %c side, cells %d..%d along it (the crossing at %d)", cfg.name,
            rf.name.c_str(), "NSWE"[rf.side], f.c0, f.c1, a0);
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
    std::vector<hal::Table*> tables = {&m_table, &m_tableB, &m_waveTable[0], &m_waveTable[1], &m_table1, &m_tableB1, &m_waveTable1[0], &m_waveTable1[1],
                                       &m_tableFold[0][0], &m_tableFold[0][1], &m_tableFold[1][0], &m_tableFold[1][1]};   // stage 1a/1b: every table reads the same page
    m_heightArr = heightArr;   // stage 2c: a level made later (an event's patch) takes the same views
    m_resMapArr = resMapArr;
    m_heightMips = mips;
    for (auto& L : m_lv) {   // stage 2b/2c: every level's bed fill reads the same stack (the tail's first run read nothing: a dry tail)
        for (hal::Table* t : {&L->table[0], &L->table[1], &L->fold[0][0], &L->fold[0][1], &L->fold[1][0], &L->fold[1][1]}) {
            if (t->Valid()) tables.push_back(t);
        }
    }
    for (hal::Table* t : tables) {
        t->SrvArray(0, heightArr, heightArr ? heightArr->GetDesc().Format : DXGI_FORMAT_R16_FLOAT, 0, UINT32_MAX, mips);
        t->SrvArray(1, resMapArr, DXGI_FORMAT_R8_UNORM, 0, UINT32_MAX, 1);
    }
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
        m_traceTable.SrvArray(0, m_bedArr, m_bedArr->GetDesc().Format, 0, UINT32_MAX, m_bedMips);
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
        "cells of %.3f x %.3f m (true, at the anchor) -- every edge cell the sea's, the Level over its bed; "
        "river faces: %s",
        cfg.name, m_dom.lat0, m_dom.lat1, m_dom.lon0, m_dom.lon1, m_dom.latC, m_dom.lonC, nx, ny,
        m_dom.dx, m_dom.dy, [&] {
            std::string t;
            for (const SweDomain::FaceCells& f : m_dom.faces) t += (t.empty() ? "" : ", ") + f.name + " " + "NSWE"[f.side];
            return t.empty() ? std::string("none") : t;
        }().c_str());

    // The banks over the bathy grid, padded up to tile multiples internally by the atlas. C1: the state
    // is the water each column holds and the fluxes (two banks, taking turns); the bed is cached a
    // frame; the published surface is what the readers read.
    {
        // THE COLUMN FIELD (stage 0, WATER_V2.md 14): a GradeBank of grade 0 with the chain's mips in the
        // one reserved resource -- as many as take the extent down to one tile; stage 0 fills mip 0 alone.
        GradeBankDesc d;
        d.name = "swe.h (the water each column holds)";
        d.width = nx;
        d.height = ny;
        d.fmt = DXGI_FORMAT_R32_FLOAT;
        d.gradeSig = kG0;
        d.mipLevels = 1;
        while (((std::max)(nx, ny) >> d.mipLevels) >= 128u) ++d.mipLevels;
        m_h.Init(gpu, std::move(d), policy::All());
    }
    m_bed.Init(gpu, nx, ny, DXGI_FORMAT_R32_FLOAT, L"swe.bed (the column's bed, a frame)", 64, 2);   // stage 1a: the chain's mip 1
    m_soil.Init(gpu, nx, ny, DXGI_FORMAT_R32_FLOAT, L"swe.soil (the water the column's soil holds)", 64, 2);   // stage 1a: the chain's mip 1
    m_eta.Init(gpu, nx, ny, DXGI_FORMAT_R32_FLOAT, L"swe.eta (the column's surface less the tide plane)", 64, 2);   // stage 1a: the chain's mip 1
    m_flux.Init(gpu, nx, ny, DXGI_FORMAT_R32G32_FLOAT, L"swe.flux (signed face fluxes)", 64, 2);   // stage 1a: the chain's mip 1
    m_velA.Init(gpu, nx, ny, DXGI_FORMAT_R32G32_FLOAT, L"swe.vel (the faces' velocities, one turn)", 64, 2);   // stage 1a: the chain's mip 1
    m_velB.Init(gpu, nx, ny, DXGI_FORMAT_R32G32_FLOAT, L"swe.vel (the faces' velocities, the other turn)", 64, 2);   // stage 1a: the chain's mip 1
    m_waveA.Init(gpu, nx, ny, DXGI_FORMAT_R16G16B16A16_FLOAT, L"swe.wave (the sea in the columns, one sweep)", 64, 2);   // stage 1a: the chain's mip 1
    m_waveB.Init(gpu, nx, ny, DXGI_FORMAT_R16G16B16A16_FLOAT, L"swe.wave (the sea in the columns, the other)", 64, 2);   // stage 1a: the chain's mip 1
    m_lim.Init(gpu, nx, ny, DXGI_FORMAT_R32_FLOAT, L"swe.lim (a column's share of its outgoing fluxes)", 64, 2);   // stage 1a: the chain's mip 1
    // STAGE 1a: THE COARSE TWIN'S RESIDENCY -- every tile of mip 1 of every bank, the whole box, mapped once
    // (a quarter of mip 0's count; stage 2 makes it demand's). Its lists and live words follow.
    {
        const auto mapAll = [&](TileAtlas2D& b) {
            for (uint32_t ty = 0; ty < b.TilesY(1); ++ty) {
                for (uint32_t tx = 0; tx < b.TilesX(1); ++tx) b.RequestMap(1, tx, ty);
            }
            b.CommitMappings(gpu, nullptr);
        };
        mapAll(m_h.Atlas());
        for (TileAtlas2D* b : {&m_bed, &m_soil, &m_eta, &m_flux, &m_velA, &m_velB, &m_waveA, &m_waveB, &m_lim}) mapAll(*b);
        m_list1.clear();
        for (uint32_t ty = 0; ty < m_h.Atlas().TilesY(1); ++ty) {
            for (uint32_t tx = 0; tx < m_h.Atlas().TilesX(1); ++tx) m_list1.push_back(ty * m_h.Atlas().TilesX(1) + tx);
        }
        m_fluxList1.clear();
        for (uint32_t ty = 0; ty < m_flux.TilesY(1); ++ty) {
            for (uint32_t tx = 0; tx < m_flux.TilesX(1); ++tx) m_fluxList1.push_back(ty * m_flux.TilesX(1) + tx);
        }
        m_live1.assign(m_list1.size(), 1u);
        m_coarseFresh = true;
        uint32_t resH = 0, resB = 0;
        for (uint32_t ty = 0; ty < m_bed.TilesY(1); ++ty) {
            for (uint32_t tx = 0; tx < m_bed.TilesX(1); ++tx) {
                resH += m_h.Atlas().IsResident(1u, tx, ty) ? 1u : 0u;
                resB += m_bed.IsResident(1u, tx, ty) ? 1u : 0u;
            }
        }
        Log("[swe] stage 1a: after the mapping, mip 1 resident: the water %u, the bed %u of %u tiles (refused %u / %u)",
            resH, resB, m_bed.TilesX(1) * m_bed.TilesY(1), m_h.Atlas().RefusedMaps(), m_bed.RefusedMaps());
        Log("[swe] stage 1a: the coarse twin at mip 1 -- %u x %u tiles of the water (%u standard mips of %u), %u of the fluxes (%u standard mips), the whole box",
            m_h.Atlas().TilesX(1), m_h.Atlas().TilesY(1), m_h.Atlas().StandardMips(), m_h.Atlas().MipCount(), static_cast<unsigned>(m_fluxList1.size()), m_flux.StandardMips());
    }
    if (cfg.tailCells > 0 && cfg.bedAt && m_testH.empty()) InitTelescope(gpu, cfg);   // stage 2b/2c (the levels' tables below, once every bank stands)
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

    const auto& elev = m_dom.elev;
    // THE SPILL LEVELS (C1): for each cell, the lowest level at which water from the boundary reaches
    // it -- the least, over the paths from an open side or a river face, of the highest bed along the
    // path (4-connected, the kernel's faces; a priority flood). The domain's wet area at a level L is
    // the cells whose spill level is under L: the hypsometry the open sea's prism is reckoned from
    // (OceanBoundary). A hollow behind a dune is not part of it until the water tops the dune.
    std::vector<float> spill(size_t(nx) * ny, 3.0e38f);
    {
        using Item = std::pair<float, uint32_t>;
        std::priority_queue<Item, std::vector<Item>, std::greater<Item>> open;
        const auto seed = [&](size_t i) {
            if (elev[i] > -9000.0f && elev[i] < spill[i]) {
                spill[i] = elev[i];
                open.push({elev[i], uint32_t(i)});
            }
        };
        int seaCells[4] = {0, 0, 0, 0}, underMsl[4] = {0, 0, 0, 0};   // a census at mean sea (0 m NAVD88): an instrument
        for (int side = 0; side < 4; ++side) {
            const int len = side < 2 ? int(nx) : int(ny);
            for (int a = 0; a < len; ++a) {
                if (!m_dom.SeaEdgeCell(side, a)) continue;
                seed(m_dom.EdgeCell(side, a));   // at its own bed: the sea reaches it once the Level tops it
                ++seaCells[side];
                if (elev[m_dom.EdgeCell(side, a)] < 0.0f) ++underMsl[side];
            }
        }
        Log("[swe] %s the edge is the sea's: N %d, S %d, W %d, E %d cells (%d, %d, %d, %d of them under mean sea), "
            "each a Flather face on the Level over its bed",
            cfg.name, seaCells[0], seaCells[1], seaCells[2], seaCells[3], underMsl[0], underMsl[1], underMsl[2],
            underMsl[3]);
        for (const SweDomain::FaceCells& F : m_dom.faces) {
            for (int a = F.c0; a <= F.c1; ++a) seed(m_dom.EdgeCell(F.side, a));
        }
        while (!open.empty()) {
            const Item top = open.top();
            open.pop();
            if (top.first > spill[top.second]) continue;
            const int c = int(top.second % nx), r = int(top.second / nx);
            const int nb[4][2] = {{c + 1, r}, {c - 1, r}, {c, r + 1}, {c, r - 1}};
            for (const auto& q : nb) {
                if (q[0] < 0 || q[1] < 0 || q[0] >= int(nx) || q[1] >= int(ny)) continue;
                const size_t j = size_t(q[1]) * nx + size_t(q[0]);
                if (elev[j] < -9000.0f) continue;
                const float m = (std::max)(top.first, elev[j]);
                if (m < spill[j]) {
                    spill[j] = m;
                    open.push({m, uint32_t(j)});
                }
            }
        }
        m_spill.clear();
        for (const float s : spill) {
            if (s < 1.0e38f) m_spill.push_back(s);
        }
        std::sort(m_spill.begin(), m_spill.end());
    }
    // C2: THE FIRST RESIDENCY, before the water says where it is: the tiles holding a cell the boundary's
    // water reaches at +1.2 m NAVD. From the first batch on the water's own rule decides (UpdateResidency):
    // a tile stays while a column holds water or water touches its edge, and goes when dry past the
    // hysteresis -- the marsh platforms, the dunes and the streets join when the water comes.
    auto mapWet = [&](auto& bank) {
        for (uint32_t ty = 0; ty < bank.TilesY(); ++ty) {
            for (uint32_t tx = 0; tx < bank.TilesX(); ++tx) {
                bool wet = false;
                const uint32_t x1 = (std::min)((tx + 1) * bank.TileW(), nx);
                const uint32_t y1 = (std::min)((ty + 1) * bank.TileH(), ny);
                for (uint32_t y = ty * bank.TileH(); y < y1 && !wet; ++y) {
                    for (uint32_t x = tx * bank.TileW(); x < x1; ++x) {
                        if (spill[size_t(y) * nx + x] < 1.2f) {
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
    // Every other bank covers the state's tiles (their grids differ by format: a 4-byte texel tiles
    // 128 x 128, an 8-byte one 128 x 64).
    auto mapCover = [&](auto& bank, uint32_t apron = 0) {
        for (uint32_t ty = 0; ty < bank.TilesY(); ++ty) {
            for (uint32_t tx = 0; tx < bank.TilesX(); ++tx) {
                const uint32_t x0 = tx * bank.TileW(), y0 = ty * bank.TileH();
                const uint32_t x1 = x0 + bank.TileW() - 1 + apron, y1 = y0 + bank.TileH() - 1 + apron;
                bool live = false;
                for (uint32_t hy = y0 / m_h.TileH(); hy <= y1 / m_h.TileH() && hy < m_h.TilesY() && !live; ++hy) {
                    for (uint32_t hx = x0 / m_h.TileW(); hx <= x1 / m_h.TileW() && hx < m_h.TilesX(); ++hx) {
                        if (m_h.IsResident(hx, hy)) {
                            live = true;
                            break;
                        }
                    }
                }
                if (live) bank.RequestMap(tx, ty);
            }
        }
        std::vector<uint32_t> fresh;
        bank.CommitMappings(gpu, &fresh);
    };
    mapWet(m_h);
    mapCover(m_bed);
    mapCover(m_soil);
    mapCover(m_eta);
    mapCover(m_flux, 1);   // stage 1c: the apron (CoverState's note)
    mapCover(m_velA, 1);
    mapCover(m_velB, 1);
    mapCover(m_waveA);
    mapCover(m_waveB);
    mapCover(m_lim);

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
    m_dt = 0.45f * (std::min)(dx, dyM) / std::sqrt(static_cast<float>(PlanetGWave()) * (std::max)(hmax, 5.0f));

    {
        Log("[swe] %s spill levels: %zu cells reachable from the boundary; wet at 0 m NAVD %.2f km^2, at +1 m "
            "%.2f km^2, at +2 m %.2f km^2",
            cfg.name, m_spill.size(), WetAreaAt(0.0) / 1.0e6, WetAreaAt(1.0) / 1.0e6, WetAreaAt(2.0) / 1.0e6);
    }

    m_cb.nx = nx;
    m_cb.ny = ny;
    m_cb.etaTilesX = m_eta.TilesX();
    m_cb.fluxTilesX = m_flux.TilesX();
    m_cb.etaTileW = m_eta.TileW();
    m_cb.etaTileH = m_eta.TileH();
    m_cb.fluxTileW = m_flux.TileW();
    m_cb.fluxTileH = m_flux.TileH();
    m_cb.unused0 = 0.0f;
    m_cb.dx = dx;
    m_cb.dy = dyM;
    m_cb.dt = m_dt;
    m_cb.damp = 0.99995f;   // background linear part only; the real friction is quadratic drag
    m_cb.wetEps = 0.01f;    // wet is h > 1 cm: the solver's one tolerance (GeoClaw's dry cells)
    m_cb.soilRate = 0.0f;
    m_cb.soil[0] = cfg.soilCapM;      // C2: the land soaks (Swe.hlsl CsSweHeight)
    m_cb.soil[1] = cfg.soilKs;
    m_cb.soil[2] = cfg.soilSuctionM;
    // GAGAME_SWE_NOSOIL (an instrument: the soil's A/B)
    m_cb.soil[3] = (cfg.soilCapM > 0.0f && !std::getenv("GAGAME_SWE_NOSOIL")) ? 1.0f : 0.0f;
    m_cb.cd = 0.0015f;      // the bed's drag (Swe.hlsl FaceUpdate)
    m_cb.gravity = static_cast<float>(PlanetGWave());   // the planet's (core/Planet.h)
    // M6r: A RIVER FACE IS FLATHER -- its exterior cells pinned to the river-tide data, the face
    // between each and its interior neighbour radiating at the gravity-wave speed (Swe.hlsl). The
    // 24-texel Dirichlet strip it replaced was a soft wall (transients reflected off the pinned eta),
    // and its relax rate was a tuning knob the radiation condition does not need. A domain no river
    // enters has no face: its edges are the sea's, each cell on the Level over its bed. (The south strip that held Plum Island
    // Sound's tide, retired in M6d -- it filled the basin through the back door -- is gone.)
    static_assert(kMaxRiverFaces == 4, "SweCbData's faceA / faceB and Swe.hlsl's gFaceA / gFaceB hold four");
    m_cb.faceCount = static_cast<float>(m_dom.faces.size());
    for (size_t f = 0; f < m_dom.faces.size(); ++f) {
        const SweDomain::FaceCells& F = m_dom.faces[f];
        m_cb.faceA[4 * f + 0] = static_cast<float>(F.side);
        m_cb.faceA[4 * f + 1] = static_cast<float>(F.c0);
        m_cb.faceA[4 * f + 2] = static_cast<float>(F.c1);
        m_cb.faceA[4 * f + 3] = 0.0f;
        // The exterior cells' beds: Record turns the face's TRANSPORT into Flather's u_ext by
        // dividing by the live wet section (the level moves every substep batch).
        m_faceL[f] = F.side < 2 ? dx : dyM;
        m_faceBed[f].clear();
        for (int a = F.c0; a <= F.c1; ++a) {
            const float e = elev[m_dom.EdgeCell(F.side, a)];
            if (e > -9000.0f && e < 2.0f) m_faceBed[f].push_back(e);
        }
        Log("[swe] %s %s face (%c): FLATHER, %zu wet-capable section cells of %.2f m", cfg.name, F.name.c_str(),
            "NSWE"[F.side], m_faceBed[f].size(), double(m_faceL[f]));
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
        mapCover(m_uvBank);
        m_uvSrv = gpu.CreateSrv(m_uvBank.Res(), DXGI_FORMAT_R16G16B16A16_FLOAT);
        m_uvState = D3D12_RESOURCE_STATE_UNORDERED_ACCESS;
    }

    // Root signature: b0 CBV, t0 root SRV (tile list), table [t1 height page, t2 its residency
    // map (M9ar), u0 eta, u1 flux, u2 uv, u3 mv (M9h: the derived grad(flow) bank), u6 h, u7 the
    // fluxes before, u8 bed, u9 soil], t3 root SRV (the live tiles).
    m_rs = hal::RootLayout{}
               .Cbv(0)
               .Srv(0)
               .Table({hal::SrvRange(1, 2), hal::UavRange(0, 4), hal::UavRange(6, 10)})
               .Srv(3)
               .Table({hal::UavRange(16, 12)})  // stage 1c: the chains (u16..u27: h, bed, the read velocities, the sea; two mips and the tail)
               .Srv(4)                          // stage 1c: the live words of the mip above
               .Srv(5)                          // stage 3: an impact's table (CsSweImpose)
               .Srv(6)                          // stage 2b: the tail's live words
               .Build(gpu, "swe");
    m_rs->SetName(L"swe root signature");

    auto makePso = [&](const wchar_t* entry, hal::Pso& out) {
        out = hal::Require(
            hal::BuildCompute(gpu, m_rs.Get(),
                              sc.Compile(shaderDir + L"/Swe.hlsl", entry, L"cs_6_0"), "swe"),
            "Swe kernel");
        out->SetName(entry);
    };
    makePso(L"CsSweReset", m_resetK);
    makePso(L"CsSweBedFill", m_bedFillK);
    makePso(L"CsSweTileStats", m_statsK);
    makePso(L"CsSweLedger", m_ledgerK);
    makePso(L"CsSweClearTile", m_clearTileK);
    makePso(L"CsSweWaves", m_wavesK);
    makePso(L"CsSweLimit", m_limitK);
    makePso(L"CsSweClearFlux", m_clearFlux);
    makePso(L"CsSweUvClear", m_uvClear);
    makePso(L"CsSweFlux", m_fluxK);
    makePso(L"CsSweHeight", m_heightK);
    makePso(L"CsSweDerive", m_deriveK);
    makePso(L"CsSweVelGrad", m_velGradK);
    makePso(L"CsSweFold", m_foldK);   // stage 1b
    makePso(L"CsSweImpose", m_imposeK);   // stage 3
    makePso(L"CsSweFoldTail", m_foldTailK);   // stage 2b

    // t1, t2 filled by SetBed; the UAVs here. m_table reads m_velA (u7) and writes m_velB (u10);
    // m_tableB the other way (the substeps take turns: Record).
    for (int k = 0; k < 2; ++k) {
        hal::Table& t = k ? m_tableB : m_table;
        t = hal::Table::Alloc(gpu, 16, "swe");
        t.Uav2D(2, m_eta.Res(), DXGI_FORMAT_R32_FLOAT);
        t.Uav2D(3, m_flux.Res(), DXGI_FORMAT_R32G32_FLOAT);
        // The bank is an ARRAY now, so its UAV must be an array view pinned to slice 0. A plain
        // Texture2D view of an array resource is invalid, and the solve would write nowhere.
        t.UavArray(4, m_uvBank.Res(), DXGI_FORMAT_R16G16B16A16_FLOAT, 0, 1);   // M9ax: uv bank
        t.UavArray(5, m_velGrad.Res(), DXGI_FORMAT_R16G16B16A16_FLOAT, 0, 1);
        t.Uav2D(6, m_h.Atlas().Res(), DXGI_FORMAT_R32_FLOAT);
        t.Uav2D(7, (k ? m_velB : m_velA).Res(), DXGI_FORMAT_R32G32_FLOAT);
        t.Uav2D(8, m_bed.Res(), DXGI_FORMAT_R32_FLOAT);
        t.Uav2D(9, m_soil.Res(), DXGI_FORMAT_R32_FLOAT);
        t.Uav2D(10, (k ? m_velA : m_velB).Res(), DXGI_FORMAT_R32G32_FLOAT);
    }
    // C2: the tile statistics and the ledger.
    m_statsTex = gpu.CreateTexture2D(m_h.TilesX(), m_h.TilesY(), DXGI_FORMAT_R32G32B32A32_UINT,
                                     D3D12_RESOURCE_FLAG_ALLOW_UNORDERED_ACCESS, D3D12_RESOURCE_STATE_UNORDERED_ACCESS,
                                     L"swe.tilestats (the water, tile by tile)");
    m_ledgerTex = gpu.CreateTexture2D(1, 1, DXGI_FORMAT_R32G32B32A32_FLOAT, D3D12_RESOURCE_FLAG_ALLOW_UNORDERED_ACCESS,
                                      D3D12_RESOURCE_STATE_UNORDERED_ACCESS, L"swe.ledger (in through the open faces)");
    // STAGE 1a: the coarse twin's tables -- the same slots, every per-cell bank's view at MIP 1 (the uv and
    // gradient banks stay mip 0's: the derive does not run at mip 1; stats and ledger likewise unused there)
    for (int k = 0; k < 2; ++k) {
        hal::Table& t = k ? m_tableB1 : m_table1;
        t = hal::Table::Alloc(gpu, 16, "swe.mip1");
        t.Uav2D(2, m_eta.Res(), DXGI_FORMAT_R32_FLOAT, 1);
        t.Uav2D(3, m_flux.Res(), DXGI_FORMAT_R32G32_FLOAT, 1);
        t.UavArray(4, m_uvBank.Res(), DXGI_FORMAT_R16G16B16A16_FLOAT, 0, 1);
        t.UavArray(5, m_velGrad.Res(), DXGI_FORMAT_R16G16B16A16_FLOAT, 0, 1);
        t.Uav2D(6, m_h.Atlas().Res(), DXGI_FORMAT_R32_FLOAT, 1);
        t.Uav2D(7, (k ? m_velB : m_velA).Res(), DXGI_FORMAT_R32G32_FLOAT, 1);
        t.Uav2D(8, m_bed.Res(), DXGI_FORMAT_R32_FLOAT, 1);
        t.Uav2D(9, m_soil.Res(), DXGI_FORMAT_R32_FLOAT, 1);
        t.Uav2D(10, (k ? m_velA : m_velB).Res(), DXGI_FORMAT_R32G32_FLOAT, 1);
        t.Uav2D(11, m_statsTex.res.Get(), DXGI_FORMAT_R32G32B32A32_UINT);
        t.Uav2D(12, m_ledgerTex.res.Get(), DXGI_FORMAT_R32G32B32A32_FLOAT);
        t.Uav2D(13, m_waveB.Res(), DXGI_FORMAT_R16G16B16A16_FLOAT, 1);
        t.Uav2D(14, m_waveA.Res(), DXGI_FORMAT_R16G16B16A16_FLOAT, 1);
        t.Uav2D(15, m_lim.Res(), DXGI_FORMAT_R32_FLOAT, 1);
    }
    // STAGE 1c: THE CHAINS' TABLE -- h and the bed at mips 0 and 1, and the velocity bank each mip reads (the
    // fine's parity fp, the twin's cp), and the sea bank each mip's solve reads: u16 h[0], u17 h[1], u18 bed[0],
    // u19 bed[1], u20 vel[0], u21 vel[1], u22 wave[0], u23 wave[1]
    // STAGE 2b/2c: the window's parent level's views at index 2 of each chain (its h, its bed, the bank its last step
    // wrote, by its parity; a sea bank unread); the window's own at mip 0 where no level holds it (never read: gTail.x < 0).
    const Level* WP = (m_winParent >= 0) ? m_lv[size_t(m_winParent)].get() : nullptr;
    for (int fp = 0; fp < 2; ++fp) {
        for (int cp = 0; cp < 2; ++cp) {
            for (int tp = 0; tp < 2; ++tp) {
                hal::Table& t = m_tableChain[fp][cp][tp];
                t = hal::Table::Alloc(gpu, 12, "swe.chain");
                t.Uav2D(0, m_h.Atlas().Res(), DXGI_FORMAT_R32_FLOAT, 0);
                t.Uav2D(1, m_h.Atlas().Res(), DXGI_FORMAT_R32_FLOAT, 1);
                t.Uav2D(2, (WP ? WP->h : m_h.Atlas()).Res(), DXGI_FORMAT_R32_FLOAT, 0);
                t.Uav2D(3, m_bed.Res(), DXGI_FORMAT_R32_FLOAT, 0);
                t.Uav2D(4, m_bed.Res(), DXGI_FORMAT_R32_FLOAT, 1);
                t.Uav2D(5, (WP ? WP->bed : m_bed).Res(), DXGI_FORMAT_R32_FLOAT, 0);
                t.Uav2D(6, (fp ? m_velB : m_velA).Res(), DXGI_FORMAT_R32G32_FLOAT, 0);
                t.Uav2D(7, (cp ? m_velB : m_velA).Res(), DXGI_FORMAT_R32G32_FLOAT, 1);
                t.Uav2D(8, (WP ? (tp ? WP->velB : WP->velA) : m_velA).Res(), DXGI_FORMAT_R32G32_FLOAT, 0);
                t.Uav2D(9, (fp ? m_waveB : m_waveA).Res(), DXGI_FORMAT_R16G16B16A16_FLOAT, 0);   // m_table / m_tableB's gWave
                t.Uav2D(10, m_waveB.Res(), DXGI_FORMAT_R16G16B16A16_FLOAT, 1);                    // m_table1's gWave
                t.Uav2D(11, m_waveA.Res(), DXGI_FORMAT_R16G16B16A16_FLOAT, 0);                    // unread
            }
        }
    }
    for (auto& L : m_lv) LevelTables(gpu, *L);   // stage 2c: every level's tables, once every bank stands
    // STAGE 1c: the twin's sweeps' tables -- m_table1's views with the two sea banks in turn at mip 1
    for (int k = 0; k < 2; ++k) {
        hal::Table& t = m_waveTable1[k];
        t = hal::Table::Alloc(gpu, 16, "swe.waves1");
        t.Uav2D(2, m_eta.Res(), DXGI_FORMAT_R32_FLOAT, 1);
        t.Uav2D(3, m_flux.Res(), DXGI_FORMAT_R32G32_FLOAT, 1);
        t.UavArray(4, m_uvBank.Res(), DXGI_FORMAT_R16G16B16A16_FLOAT, 0, 1);
        t.UavArray(5, m_velGrad.Res(), DXGI_FORMAT_R16G16B16A16_FLOAT, 0, 1);
        t.Uav2D(6, m_h.Atlas().Res(), DXGI_FORMAT_R32_FLOAT, 1);
        t.Uav2D(7, m_velA.Res(), DXGI_FORMAT_R32G32_FLOAT, 1);
        t.Uav2D(8, m_bed.Res(), DXGI_FORMAT_R32_FLOAT, 1);
        t.Uav2D(9, m_soil.Res(), DXGI_FORMAT_R32_FLOAT, 1);
        t.Uav2D(10, m_velB.Res(), DXGI_FORMAT_R32G32_FLOAT, 1);
        t.Uav2D(11, m_statsTex.res.Get(), DXGI_FORMAT_R32G32B32A32_UINT);
        t.Uav2D(12, m_ledgerTex.res.Get(), DXGI_FORMAT_R32G32B32A32_FLOAT);
        t.Uav2D(13, (k ? m_waveB : m_waveA).Res(), DXGI_FORMAT_R16G16B16A16_FLOAT, 1);
        t.Uav2D(14, (k ? m_waveA : m_waveB).Res(), DXGI_FORMAT_R16G16B16A16_FLOAT, 1);
        t.Uav2D(15, m_lim.Res(), DXGI_FORMAT_R32_FLOAT, 1);
    }
    // STAGE 1b: the fold's tables -- the fine banks at mip 0 in the read slots (gH, gBedC, gVelPrev = the
    // fine's newest velocities), the twin's at mip 1 in the written ones (gEta = the water, gSoilW = the bed,
    // gFlux = the twin's read velocity bank), one per parity pair [fine newest][twin's read]
    for (int fp = 0; fp < 2; ++fp) {
        for (int cp = 0; cp < 2; ++cp) {
            hal::Table& t = m_tableFold[fp][cp];
            t = hal::Table::Alloc(gpu, 16, "swe.fold");
            t.Uav2D(2, m_h.Atlas().Res(), DXGI_FORMAT_R32_FLOAT, 1);
            t.Uav2D(3, (cp ? m_velB : m_velA).Res(), DXGI_FORMAT_R32G32_FLOAT, 1);
            t.UavArray(4, m_uvBank.Res(), DXGI_FORMAT_R16G16B16A16_FLOAT, 0, 1);
            t.UavArray(5, m_velGrad.Res(), DXGI_FORMAT_R16G16B16A16_FLOAT, 0, 1);
            t.Uav2D(6, m_h.Atlas().Res(), DXGI_FORMAT_R32_FLOAT, 0);
            t.Uav2D(7, (fp ? m_velB : m_velA).Res(), DXGI_FORMAT_R32G32_FLOAT, 0);
            t.Uav2D(8, m_bed.Res(), DXGI_FORMAT_R32_FLOAT, 0);
            t.Uav2D(9, m_bed.Res(), DXGI_FORMAT_R32_FLOAT, 1);
            t.Uav2D(10, m_velA.Res(), DXGI_FORMAT_R32G32_FLOAT, 1);
            t.Uav2D(11, m_statsTex.res.Get(), DXGI_FORMAT_R32G32B32A32_UINT);
            t.Uav2D(12, m_ledgerTex.res.Get(), DXGI_FORMAT_R32G32B32A32_FLOAT);
            t.Uav2D(13, m_waveB.Res(), DXGI_FORMAT_R16G16B16A16_FLOAT, 1);
            t.Uav2D(14, m_waveA.Res(), DXGI_FORMAT_R16G16B16A16_FLOAT, 1);
            t.Uav2D(15, m_lim.Res(), DXGI_FORMAT_R32_FLOAT, 1);
        }
    }
    for (hal::Table* t : {&m_table, &m_tableB}) {
        t->Uav2D(11, m_statsTex.res.Get(), DXGI_FORMAT_R32G32B32A32_UINT);
        t->Uav2D(12, m_ledgerTex.res.Get(), DXGI_FORMAT_R32G32B32A32_FLOAT);
        t->Uav2D(13, m_waveB.Res(), DXGI_FORMAT_R16G16B16A16_FLOAT);   // the solve reads B
        t->Uav2D(14, m_waveA.Res(), DXGI_FORMAT_R16G16B16A16_FLOAT);
        t->Uav2D(15, m_lim.Res(), DXGI_FORMAT_R32_FLOAT);
    }
    // C4: the sweeps' tables -- m_table's views with the two sea banks in turn.
    for (int k = 0; k < 2; ++k) {
        hal::Table& t = m_waveTable[k];
        t = hal::Table::Alloc(gpu, 16, "swe.waves");
        t.Uav2D(2, m_eta.Res(), DXGI_FORMAT_R32_FLOAT);
        t.Uav2D(3, m_flux.Res(), DXGI_FORMAT_R32G32_FLOAT);
        t.UavArray(4, m_uvBank.Res(), DXGI_FORMAT_R16G16B16A16_FLOAT, 0, 1);
        t.UavArray(5, m_velGrad.Res(), DXGI_FORMAT_R16G16B16A16_FLOAT, 0, 1);
        t.Uav2D(6, m_h.Atlas().Res(), DXGI_FORMAT_R32_FLOAT);
        t.Uav2D(7, m_velA.Res(), DXGI_FORMAT_R32G32_FLOAT);
        t.Uav2D(8, m_bed.Res(), DXGI_FORMAT_R32_FLOAT);
        t.Uav2D(9, m_soil.Res(), DXGI_FORMAT_R32_FLOAT);
        t.Uav2D(10, m_velB.Res(), DXGI_FORMAT_R32G32_FLOAT);
        t.Uav2D(11, m_statsTex.res.Get(), DXGI_FORMAT_R32G32B32A32_UINT);
        t.Uav2D(12, m_ledgerTex.res.Get(), DXGI_FORMAT_R32G32B32A32_FLOAT);
        t.Uav2D(13, (k ? m_waveB : m_waveA).Res(), DXGI_FORMAT_R16G16B16A16_FLOAT);
        t.Uav2D(14, (k ? m_waveA : m_waveB).Res(), DXGI_FORMAT_R16G16B16A16_FLOAT);
        t.Uav2D(15, m_lim.Res(), DXGI_FORMAT_R32_FLOAT);
    }
    m_dryFor.assign(size_t(m_h.TilesX()) * m_h.TilesY(), 0.0);
    // The boundary's faces (the ledger's): every edge cell once. (The tiles the boundary's water stands
    // on are the Level's, each residency turn: BoundaryTiles.)
    m_ledgerFaces.assign(1, 0u);
    for (int side = 0; side < 4; ++side) {
        const int len = side < 2 ? int(nx) : int(ny);
        for (int a = 0; a < len; ++a) {
            const int x = side == 2 ? 0 : side == 3 ? int(nx) - 1 : a;
            const int y = side == 0 ? 0 : side == 1 ? int(ny) - 1 : a;
            // each edge cell once (a corner belongs to its row, N or S)
            if (side < 2 || (y > 0 && y < int(ny) - 1)) m_ledgerFaces.push_back(uint32_t(x) | (uint32_t(y) << 12));
        }
    }
    m_ledgerFaces[0] = uint32_t(m_ledgerFaces.size() - 1);
    m_live.assign(size_t(m_h.TilesX()) * m_h.TilesY(), 0u);
    for (uint32_t ty = 0; ty < m_h.TilesY(); ++ty) {
        for (uint32_t tx = 0; tx < m_h.TilesX(); ++tx) m_live[size_t(ty) * m_h.TilesX() + tx] = m_h.IsResident(tx, ty) ? 1u : 0u;
    }

    Log("[swe] grid %ux%u dx %.2f x dy %.2f m  dt %.3f s  h %u/%u t  flux %u/%u t  "
        "resident %.1f MB (h, bed, soil, eta, two fluxes)",
        nx, ny, dx, dyM, m_dt, m_h.ResidentCount(), m_h.TilesX() * m_h.TilesY(),
        m_flux.ResidentCount(), m_flux.TilesX() * m_flux.TilesY(),
        (ResidentBytes() + m_soil.ResidentBytes()) / 1048576.0);
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

double SweSolver::WetAreaAt(double levelNavd) const {
    const size_t k = size_t(std::lower_bound(m_spill.begin(), m_spill.end(), float(levelNavd)) - m_spill.begin());
    return double(k) * m_dom.dx * m_dom.dy;
}

double SweSolver::EdgeLevel(int side, int a) const {
    for (int f = 0; f < FaceCount(); ++f) {
        const SweDomain::FaceCells& F = m_dom.faces[f];
        if (F.side == side && a >= F.c0 && a <= F.c1 && m_dom.elev[m_dom.EdgeCell(side, a)] < 2.0f) {
            return double(m_cb.tideNavd) + double(m_cb.faceB[4 * f + 1]);
        }
    }
    const double len = side < 2 ? double(m_cb.nx) : double(m_cb.ny);
    const double t = std::clamp((a + 0.5) / len, 0.0, 1.0) * 3.0;   // 0..3 (Swe.hlsl OceanEtaAt)
    const double l0 = -(t - 1.0) * (t - 2.0) * (t - 3.0) / 6.0, l1 = t * (t - 2.0) * (t - 3.0) / 2.0;
    const double l2 = -t * (t - 1.0) * (t - 3.0) / 2.0, l3 = t * (t - 1.0) * (t - 2.0) / 6.0;
    const float* y = &m_cb.oceanEta[4 * side];
    return double(m_cb.tideNavd) + l0 * y[0] + l1 * y[1] + l2 * y[2] + l3 * y[3];
}

std::vector<uint8_t> SweSolver::BoundaryTiles() const {
    const uint32_t tX = m_h.TilesX(), tY = m_h.TilesY();
    std::vector<uint8_t> keep(size_t(tX) * tY, 0);
    for (int side = 0; side < 4; ++side) {
        const int len = side < 2 ? int(m_cb.nx) : int(m_cb.ny);
        for (int a = 0; a < len; ++a) {
            const size_t i = m_dom.EdgeCell(side, a);
            const float e = m_dom.elev[i];
            if (e < -9000.0f || !(EdgeLevel(side, a) > double(e))) continue;   // the Level under the bed: no water stands here
            const uint32_t x = uint32_t(i % m_cb.nx), y = uint32_t(i / m_cb.nx);
            keep[size_t(y / m_h.TileH()) * tX + x / m_h.TileW()] = 1;
        }
    }
    return keep;
}

void SweSolver::RequestRegion(double latDeg, double lonDeg, double radiusM) {
    double tx = 0.0, ty = 0.0;
    if (!TexelOf(latDeg, lonDeg, tx, ty) || !(radiusM >= 0.0)) return;
    // The texels the reconstruction of any point within the radius touches: floor(t - 0.5) and the
    // one past it, and the ring about those (the water's level over a dry crest), at both ends.
    const double rx = radiusM / double(m_cb.dx), ry = radiusM / double(m_cb.dy);
    const double lx0 = std::floor(tx - 0.5 - rx) - 1.0, lx1 = std::floor(tx - 0.5 + rx) + 2.0;
    const double ly0 = std::floor(ty - 0.5 - ry) - 1.0, ly1 = std::floor(ty - 0.5 + ry) + 2.0;
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

// THE ONE RECONSTRUCTION (WaterBank.hlsl SolverSurfaceDev and LoadBilinearClamp at texel (u Nx, (1 - v) Ny)),
// over any store: `at(ix, iy, eta, uv4)` answers one grid texel or says it does not hold it.
// THE WATER DECIDES (2026-10-23, Mark: "you are using rhetorical methods to decide how it treats the water,
// instead of letting the water decide what it does when it hits or is over a bar"). The drawn level is the
// surface of the water the solver's columns hold -- the wet ones' (the fibre's w: h > 1 cm), by their
// bilinear weights -- and nothing else. A dry column has no surface to offer: where the four corners
// about the point are dry, the water's level is that of the columns holding water one cell further out
// (the ring of twelve about them, by a tent two cells wide), so a dry crest in the water is ground the
// water's own level is held against: it shows where its bed stands above that level, at the reader's
// grain, as the water would leave it. The rule it replaces dove the level under each dry column's bed,
// and between a dry crest and its wet neighbours the drawn water fell into a pit -- the higher the bed,
// the lower the water, the inverse of the height map (the bar's dimples). Only where no column within two
// cells holds water is there no level to read: the level stands under the solver's own published bed by
// the bank's dry band and never above the frame's waterline (a dune is land under the waterline as
// surely as under its crest), so the dry weight reads zero and the classifier reads land. The hull and the
// drawn sea compose one level from the same texels.
// A COLUMN SPEAKS BY THE WATER IT HOLDS (2026-10-23; the kernel says why): its say is h / (h + h_wet), the
// corners speak their share, the rest is the ring's, the ring's the dry floor's as its water goes.
// Here: dEta is that level less the tide plane, and `wet` the corners' say.
template <class At>
bool Reconstruct(double tx, double ty, uint32_t nx, uint32_t ny, double hWet, At at, SweSolver::Deviation& out) {
    const double fx = tx - 0.5, fy = ty - 0.5;
    const double bx = std::floor(fx), by = std::floor(fy);
    const double frx = fx - bx, fry = fy - by;
    double eta = 0.0, sw = 0.0, se = 0.0, rw = 0.0, re = 0.0, uv[4] = {0.0, 0.0, 0.0, 0.0};
    for (int j = -1; j <= 2; ++j) {
        for (int i = -1; i <= 2; ++i) {
            const int ix = std::clamp(int(bx) + i, 0, int(nx) - 1);
            const int iy = std::clamp(int(by) + j, 0, int(ny) - 1);
            float e = 0.0f, c[4] = {0.0f, 0.0f, 0.0f, 0.0f};
            if (!at(ix, iy, e, c)) return false;
            const double h = (std::max)(double(c[2]), 0.0);   // the water the column holds
            const double wet = h / (h + (std::max)(hWet, 1e-4));   // its say in its own surface (the kernel's omega)
            if (i >= 0 && i <= 1 && j >= 0 && j <= 1) {
                const double w = (i ? frx : 1.0 - frx) * (j ? fry : 1.0 - fry);
                eta += w * double(e);
                sw += w * wet;
                se += w * wet * double(e);
                for (int k = 0; k < 4; ++k) uv[k] += w * double(c[k]);
            } else {
                const double w = (std::max)(2.0 - std::abs(double(i) - frx), 0.0) *
                                 (std::max)(2.0 - std::abs(double(j) - fry), 0.0);
                rw += w * wet;
                re += w * wet * double(e);
            }
        }
    }
    const double dryL = (std::min)(eta - 0.65, 0.0);
    const double ringL = (re + 0.5 * dryL) / (rw + 0.5);   // the dry floor speaks as one ring cell at the wet depth
    out.dEta = float(se + (1.0 - sw) * ringL);
    out.wet = float(sw);
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
        if (Reconstruct(tx, ty, m_cb.nx, m_cb.ny, m_cb.wetEps, at, out)) {
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
    if (!Reconstruct(tx, ty, m_cb.nx, m_cb.ny, m_cb.wetEps, at, out)) return false;
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

void SweSolver::SeaBoundary() {
    if (!m_testH.empty()) return;   // a scheme test states its own sea
    for (float& v : m_cb.sea) v = 0.0f;
    // GAGAME_SWE_NOSEA (an instrument: the A/B of the waves' push, and the calm day's gate)
    static const bool noSea = std::getenv("GAGAME_SWE_NOSEA") != nullptr;
    if (!m_seaAt || noSea) return;
    // the open water: the edge's water, its cells' centroid weighted by their depth under the Level (the
    // deep sea pulls it; the global wave grid is far coarser than the window); the domain's centre where
    // the Level stands over no edge cell
    double tx = 0.5 * double(m_cb.nx), ty = 0.5 * double(m_cb.ny);
    {
        double sx = 0.0, sy = 0.0, sw = 0.0;
        for (int side = 0; side < 4; ++side) {
            const int len = side < 2 ? int(m_cb.nx) : int(m_cb.ny);
            for (int a = 0; a < len; ++a) {
                if (!m_dom.SeaEdgeCell(side, a)) continue;
                const size_t i = m_dom.EdgeCell(side, a);
                const double d = EdgeLevel(side, a) - double(m_dom.elev[i]);
                if (!(d > 0.0)) continue;
                sx += d * (double(i % m_cb.nx) + 0.5);
                sy += d * (double(i / m_cb.nx) + 0.5);
                sw += d;
            }
        }
        if (sw > 0.0) {
            tx = sx / sw;
            ty = sy / sw;
        }
    }
    double la = 0.0, lo = 0.0;
    m_dom.LatLonOf(tx, ty, la, lo);
    double hs = 0.0, tp = 0.0, fromDeg = 0.0;
    if (!m_seaAt(la, lo, m_simTime, hs, tp, fromDeg) || !(hs > 0.01) || !(tp > 1.0)) return;
    const double toR = (fromDeg + 180.0) * 3.14159265358979323846 / 180.0;
    m_cb.sea[0] = static_cast<float>(hs);
    m_cb.sea[1] = static_cast<float>(2.0 * 3.14159265358979323846 / tp);
    m_cb.sea[2] = static_cast<float>(hs * hs / 16.0);
    m_cb.sea[3] = 1.0f;
    m_cb.sea2[0] = static_cast<float>(std::sin(toR));    // east
    m_cb.sea2[1] = static_cast<float>(-std::cos(toR));   // south (texture +y)
    static double saidHs = -1.0, saidTp = -1.0, saidFrom = -1.0;
    if (std::abs(hs - saidHs) > 0.01 * hs || std::abs(tp - saidTp) > 0.1 || std::abs(fromDeg - saidFrom) > 2.0) {
        saidHs = hs;
        saidTp = tp;
        saidFrom = fromDeg;
        Log("[swe] the sea at the open water (%.4f N %.4f E): Hs %.2f m, Tp %.1f s, from %.0f deg -- its waves push "
            "the solved water", la, lo, hs, tp, fromDeg);
    }
}

void SweSolver::AfterBatch(Gpu& gpu) {
    if (!m_ready || !m_testH.empty()) return;
    ProbeCoarse(gpu);
    GpuTexture wrap = m_statsTex;
    wrap.state = D3D12_RESOURCE_STATE_UNORDERED_ACCESS;
    uint32_t pitch = 0;
    const std::vector<uint8_t> raw = gpu.ReadbackTexture(wrap, &pitch);
    UpdateResidency(gpu, raw, pitch);
}

void SweSolver::UpdateResidency(Gpu& gpu, const std::vector<uint8_t>& raw, uint32_t pitch) {
    const uint32_t tX = m_h.TilesX(), tY = m_h.TilesY();
    const double dtS = (m_statsAt > 0.0) ? (std::max)(m_simTime - m_statsAt, 0.0) : 0.0;
    m_statsAt = m_simTime;
    std::vector<uint8_t> want(size_t(tX) * tY, 0);
    std::vector<float> vol(size_t(tX) * tY, 0.0f);
    float hMax = 0.0f, uMax = 0.0f;
    uint32_t wet = 0;
    for (uint32_t ty = 0; ty < tY; ++ty) {
        for (uint32_t tx = 0; tx < tX; ++tx) {
            if (!m_h.IsResident(tx, ty) || size_t(ty) * pitch + size_t(tx + 1) * 16 > raw.size()) continue;
            uint32_t v[4];
            memcpy(v, &raw[size_t(ty) * pitch + size_t(tx) * 16], 16);
            float hm = 0.0f, um = 0.0f;
            memcpy(&hm, &v[1], 4);
            memcpy(&um, &v[2], 4);
            memcpy(&vol[size_t(ty) * tX + tx], &v[3], 4);
            if (!(v[0] & 1u)) continue;
            ++wet;
            hMax = (std::max)(hMax, hm);
            uMax = (std::max)(uMax, um);
            want[size_t(ty) * tX + tx] = 1;
            // the ring: water on an edge wants the tile across it
            if ((v[0] & 2u) && ty > 0) want[size_t(ty - 1) * tX + tx] = 1;
            if ((v[0] & 4u) && ty + 1 < tY) want[size_t(ty + 1) * tX + tx] = 1;
            if ((v[0] & 8u) && tx > 0) want[size_t(ty) * tX + tx - 1] = 1;
            if ((v[0] & 16u) && tx + 1 < tX) want[size_t(ty) * tX + tx + 1] = 1;
        }
    }
    // STAGE 1d OF ONE WATER ON THE TREE (2026-10-26, docs/WATER_V2.md 14): THE EYE'S DEMAND. Mip 0 is wanted
    // where the water is AND within the eye's reach; beyond the reach the water is the twin's own (the fold
    // carried it to mip 1 each batch, so a released tile's water is not lost: it stands there). The fine set
    // is aligned to the coarse tiles (a 2 x 2 wanted whole), so every coarse tile over it is a fold and none
    // is half its own. The boundary's tiles (BoundaryTiles: where the Level stands over an edge cell now)
    // are kept only within the reach: beyond it the twin's edge rows are the box's boundary. A tile
    // beyond the reach goes at once (demand, not dryness).
    std::vector<uint8_t> keep = BoundaryTiles();
    std::vector<uint8_t> beyond(size_t(tX) * tY, 0);
    if (m_coarse && m_eyeOn && m_reachM > 0.0) {
        double ex = 0.0, ey = 0.0;
        if (TexelOf(m_eyeLat, m_eyeLon, ex, ey)) {
            for (uint32_t ty = 0; ty < tY; ++ty) {
                for (uint32_t tx = 0; tx < tX; ++tx) {
                    const double x0 = double(tx) * m_h.TileW(), x1 = x0 + m_h.TileW();
                    const double y0 = double(ty) * m_h.TileH(), y1 = y0 + m_h.TileH();
                    const double dxM = (std::max)({x0 - ex, 0.0, ex - x1}) * m_dom.dx;   // the tile's nearest cell to the eye's
                    const double dyM = (std::max)({y0 - ey, 0.0, ey - y1}) * m_dom.dy;
                    const size_t i = size_t(ty) * tX + tx;
                    if (std::hypot(dxM, dyM) > m_reachM) {
                        beyond[i] = 1;
                        want[i] = 0;
                        keep[i] = 0;
                    }
                }
            }
            for (uint32_t ty = 0; ty < tY; ty += 2) {   // the alignment: a coarse tile's four children together
                for (uint32_t tx = 0; tx < tX; tx += 2) {
                    uint8_t w = 0, k = 0;
                    for (uint32_t j = 0; j < 2; ++j) {
                        for (uint32_t i = 0; i < 2; ++i) {
                            if (tx + i >= tX || ty + j >= tY) continue;
                            w |= want[size_t(ty + j) * tX + tx + i];
                            k |= keep[size_t(ty + j) * tX + tx + i];
                        }
                    }
                    for (uint32_t j = 0; j < 2; ++j) {
                        for (uint32_t i = 0; i < 2; ++i) {
                            if (tx + i >= tX || ty + j >= tY) continue;
                            const size_t c = size_t(ty + j) * tX + tx + i;
                            want[c] = w;
                            keep[c] = k;
                            if (w || k) beyond[c] = 0;
                        }
                    }
                }
            }
        }
    }
    // THE STEP, from the live water: the fastest gravity wave plus the fastest current, at the Courant
    // number the solver has always run at (0.45).
    if (wet > 0) {
        const float c = std::sqrt(m_cb.gravity * (std::max)(hMax, 0.5f)) + uMax;
        m_dt = 0.45f * (std::min)(m_cb.dx, m_cb.dy) / c;
        m_cb.dt = m_dt;
    }
    uint32_t mapped = 0, released = 0;
    double releasedM3 = 0.0;
    for (uint32_t ty = 0; ty < tY; ++ty) {
        for (uint32_t tx = 0; tx < tX; ++tx) {
            const size_t i = size_t(ty) * tX + tx;
            if (want[i] || keep[i]) {
                m_dryFor[i] = 0.0;
                if (!m_h.IsResident(tx, ty)) {
                    m_h.RequestMap(tx, ty);
                    ++mapped;
                }
            } else if (m_h.IsResident(tx, ty)) {
                m_dryFor[i] += dtS;
                if (beyond[i] || m_dryFor[i] >= m_releaseS) {
                    m_h.RequestUnmap(tx, ty);
                    if (!beyond[i]) releasedM3 += double(vol[i]) * m_dom.dx * m_dom.dy;   // beyond the reach: the twin holds it
                    m_dryFor[i] = 0.0;
                    ++released;
                }
            }
        }
    }
    if (mapped == 0 && released == 0) return;
    std::vector<uint32_t> fresh;
    m_h.CommitMappings(gpu, &fresh);
    m_freshH.insert(m_freshH.end(), fresh.begin(), fresh.end());
    m_released += releasedM3;
    CoverState(gpu);
    for (uint32_t ty = 0; ty < tY; ++ty) {
        for (uint32_t tx = 0; tx < tX; ++tx) m_live[size_t(ty) * tX + tx] = m_h.IsResident(tx, ty) ? 1u : 0u;
    }
    DeriveGradResidency(gpu);
    if (m_coarse && m_eyeOn && m_reachM > 0.0) {   // stage 1d: the fine set, a row a line (# resident, . not, E the eye's tile)
        double ex = 0.0, ey = 0.0;
        const bool eye = TexelOf(m_eyeLat, m_eyeLon, ex, ey);
        for (uint32_t ty = 0; ty < tY; ++ty) {
            std::string row;
            for (uint32_t tx = 0; tx < tX; ++tx) {
                const bool e = eye && uint32_t(ex) / m_h.TileW() == tx && uint32_t(ey) / m_h.TileH() == ty;
                row += e ? 'E' : m_h.IsResident(tx, ty) ? '#' : '.';
            }
            Log("[swe]   mip 0 row %2u %s", ty, row.c_str());
        }
    }
    Log("[swe] the water's residency: +%u / -%u state tiles (%u hold water; %u resident), %.2f m^3 given up "
        "with the released; the step %.3f s (deepest %.1f m, fastest %.2f m/s)",
        mapped, released, wet, m_h.ResidentCount(), releasedM3, double(m_dt), double(hMax), double(uMax));
}

void SweSolver::CoverState(Gpu& gpu) {
    // STAGE 1c: the flux and velocity banks cover one cell past each state tile too (apron 1): a face on the
    // fine set's west or north edge is owned by the cell beyond it, whose tile must hold its flux.
    const auto cover = [&](auto& bank, std::vector<uint32_t>* freshOut, uint32_t apron = 0) {
        for (uint32_t ty = 0; ty < bank.TilesY(); ++ty) {
            for (uint32_t tx = 0; tx < bank.TilesX(); ++tx) {
                const uint32_t x0 = tx * bank.TileW(), y0 = ty * bank.TileH();
                const uint32_t x1 = x0 + bank.TileW() - 1 + apron, y1 = y0 + bank.TileH() - 1 + apron;
                bool live = false;
                for (uint32_t hy = y0 / m_h.TileH(); hy <= y1 / m_h.TileH() && hy < m_h.TilesY() && !live; ++hy) {
                    for (uint32_t hx = x0 / m_h.TileW(); hx <= x1 / m_h.TileW() && hx < m_h.TilesX(); ++hx) {
                        if (m_h.IsResident(hx, hy)) {
                            live = true;
                            break;
                        }
                    }
                }
                if (live && !bank.IsResident(tx, ty)) bank.RequestMap(tx, ty);
                if (!live && bank.IsResident(tx, ty)) bank.RequestUnmap(tx, ty);
            }
        }
        std::vector<uint32_t> fresh;
        bank.CommitMappings(gpu, &fresh);
        if (freshOut) freshOut->insert(freshOut->end(), fresh.begin(), fresh.end());
    };
    cover(m_bed, nullptr);
    cover(m_soil, nullptr);
    cover(m_eta, nullptr);
    cover(m_flux, &m_freshF, 1);
    cover(m_velA, nullptr, 1);
    cover(m_velB, nullptr, 1);
    cover(m_waveA, nullptr);
    cover(m_waveB, nullptr);
    cover(m_lim, nullptr);
    cover(m_uvBank, nullptr);
}

void SweSolver::DeriveGradResidency(Gpu& gpu) {
    // M9h: THE CAYLEY CLOSURE DECIDES THE DERIVED FIELD'S RESIDENCY -- the flow's tiles, dilated by the
    // stencil's apron (Init's note).
    const uint32_t tX = m_eta.TilesX(), tY = m_eta.TilesY();
    std::vector<uint8_t> flowSig(size_t(tX) * tY, 0), nablaSig(size_t(tX) * tY, kG1);
    for (uint32_t ty = 0; ty < tY; ++ty) {
        for (uint32_t tx = 0; tx < tX; ++tx) {
            if (m_eta.IsResident(tx, ty)) flowSig[size_t(ty) * tX + tx] = kG1;
        }
    }
    const TilePolicy derived = policy::Derived(flowSig, nablaSig, tX);
    std::vector<uint8_t> want(size_t(tX) * tY, 0);
    for (uint32_t ty = 0; ty < tY; ++ty) {
        for (uint32_t tx = 0; tx < tX; ++tx) {
            if (!derived(tx, ty)) continue;
            for (int dy = -1; dy <= 1; ++dy) {
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
    m_velGrad.EnsureCoarseMapped(gpu);
}

uint32_t SweSolver::DryWaiting() const {
    uint32_t n = 0;
    for (uint32_t ty = 0; ty < m_h.TilesY(); ++ty) {
        for (uint32_t tx = 0; tx < m_h.TilesX(); ++tx) {
            if (m_h.IsResident(tx, ty) && m_dryFor[size_t(ty) * m_h.TilesX() + tx] > 0.0) ++n;
        }
    }
    return n;
}

void SweSolver::LedgerOn(Gpu& gpu) {
    m_ledgerOn = true;
    m_ledgerBase = 0.0;
    m_ledgerBase = LedgerIn(gpu);
    m_released = 0.0;
}

double SweSolver::LedgerIn(Gpu& gpu) {
    GpuTexture wrap = m_ledgerTex;
    wrap.state = D3D12_RESOURCE_STATE_UNORDERED_ACCESS;
    uint32_t pitch = 0;
    const std::vector<uint8_t> raw = gpu.ReadbackTexture(wrap, &pitch);
    float v[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    if (raw.size() >= 16) memcpy(v, raw.data(), 16);
    return double(v[0]) - double(v[1]) - m_ledgerBase;
}

void SweSolver::Volumes(Gpu& gpu, double& columns, double& soil, std::vector<float>* hOut) {
    columns = soil = 0.0;
    const auto readR32 = [&](TileAtlas2D& bank, std::vector<float>& out) {
        GpuTexture wrap;
        wrap.res = bank.Res();
        wrap.format = DXGI_FORMAT_R32_FLOAT;
        wrap.width = bank.TilesX() * bank.TileW();
        wrap.height = bank.TilesY() * bank.TileH();
        wrap.state = D3D12_RESOURCE_STATE_UNORDERED_ACCESS;
        uint32_t pitch = 0;
        const std::vector<uint8_t> raw = gpu.ReadbackTexture(wrap, &pitch);
        out.assign(size_t(m_cb.nx) * m_cb.ny, 0.0f);
        for (uint32_t y = 0; y < m_cb.ny && size_t(y) * pitch + size_t(m_cb.nx) * 4 <= raw.size(); ++y) {
            memcpy(&out[size_t(y) * m_cb.nx], &raw[size_t(y) * pitch], size_t(m_cb.nx) * 4);
        }
    };
    std::vector<float> h, w, bed;
    readR32(m_h.Atlas(), h);
    readR32(m_soil, w);
    readR32(m_bed, bed);
    // the exterior cells are the boundary's (pinned), not the domain's: the kernel's test (Swe.hlsl
    // ExteriorLevel) on the bed it cached
    std::vector<uint8_t> ext(size_t(m_cb.nx) * m_cb.ny, 0);
    for (int side = 0; side < 4; ++side) {
        const int len = side < 2 ? int(m_cb.nx) : int(m_cb.ny);
        for (int a = 0; a < len; ++a) {
            const size_t i = m_dom.EdgeCell(side, a);
            bool river = false;
            for (const SweDomain::FaceCells& F : m_dom.faces) {
                river = river || (F.side == side && a >= F.c0 && a <= F.c1 && bed[i] < 2.0f);
            }
            if (river || bed[i] > -9000.0f) ext[i] = 1;   // every edge cell with a bed is the boundary's
        }
    }
    for (size_t i = 0; i < h.size(); ++i) {
        if (ext[i]) continue;
        columns += double(h[i]);
        soil += double(w[i]);
    }
    columns *= m_dom.dx * m_dom.dy;
    soil *= m_dom.dx * m_dom.dy;
    if (hOut) *hOut = std::move(h);
}

// THE STATELESS COLUMN (a reset: the spin-up's first instant, a scrub): each column refills to the
// water the tide plane stands over it and is still. Recorded inside Record, after the bed is cached
// and with its root and table bound.
// STAGE 1a: the coarse twin's constants from mip 0's -- the lattice halved, the cell doubled, the faces'
// runs halved, the cell steps about the same anchor (the coarse cell's centre is its two fine cells'
// midpoint), its step the interval over its own count (n1 = n / 2 steps of twice the size: the CFL
// scales with the cell). The waves' push is off at mip 1 (its banks are not swept there).
void SweSolver::CoarseCb(int n, int n1) {
    m_cb1 = m_cb;
    m_cb1.nx = (m_cb.nx + 1u) / 2u;
    m_cb1.ny = (m_cb.ny + 1u) / 2u;
    m_cb1.etaTilesX = m_h.Atlas().TilesX(1);
    m_cb1.fluxTilesX = m_flux.TilesX(1);
    m_cb1.listCount = static_cast<uint32_t>(m_list1.size());
    m_cb1.dx = 2.0f * m_cb.dx;
    m_cb1.dy = 2.0f * m_cb.dy;
    m_cb1.dt = (n1 > 0) ? (float(n) * m_dt) / float(n1) : m_dt;
    for (int f = 0; f < kMaxRiverFaces; ++f) {
        m_cb1.faceA[4 * f + 1] = std::floor(m_cb.faceA[4 * f + 1] * 0.5f);
        m_cb1.faceA[4 * f + 2] = std::floor(m_cb.faceA[4 * f + 2] * 0.5f);
    }
    m_cb1.cellX[1] = 2.0f * m_cb.cellX[1];
    m_cb1.cellX[3] = 2.0f * m_cb.cellX[3];
    m_cb1.cellX[0] = static_cast<float>((0.5 - 0.5 * double(m_cb1.nx)) * double(m_cb1.cellX[1]));
    m_cb1.cellX[2] = static_cast<float>((0.5 * double(m_cb1.ny) - 0.5) * double(-m_cb1.cellX[3]));
    m_cb1.mip = 1.0f;   // (the sea stays on: stage 1c sweeps it at mip 1 too)   // stage 1c: the twin's dispatches run at mip 1 (nothing above it)
    ParentFields(m_cb1, m_winParent, 1u, 0, 0);   // stage 2b/2c: the window's parent level under the twin's cells
}

void SweSolver::RecordCoarseBegin(hal::CommandContext& cmd, Gpu& gpu, int n, int n1, bool reset) {
    PixScope scope(cmd.Native(), "swe.mip1 (the coarse twin: the batch's prelude)");
    CoarseCb(n, n1);
    memcpy(m_cb1.impactA, m_cb.impactA, sizeof(m_cb.impactA));   // stage 3: the same event, the twin's own chart
    memcpy(m_cb1.impactB, m_cb.impactB, sizeof(m_cb.impactB));
    if (m_coarseFresh) {
        Log("[swe] stage 1a: the twin's first batch -- %d fine steps of %.3f s, %d coarse of %.3f s; lattice %u x %u, tiles %u across, cell %.2f x %.2f m",
            n, m_dt, n1, m_cb1.dt, m_cb1.nx, m_cb1.ny, m_cb1.etaTilesX, m_cb1.dx, m_cb1.dy);
    }
    m_live1Va = gpu.PushConstants(m_live1.data(), m_live1.size() * 4);
    cmd.ComputeConstants(0, m_cb1);
    cmd.ComputeTable(2, m_table1.Base());
    cmd.ComputeSrvAt(3, m_live1Va);
    m_hl1Va = gpu.PushConstants(m_list1.data(), m_list1.size() * 4);
    m_fl1Va = gpu.PushConstants(m_fluxList1.data(), m_fluxList1.size() * 4);
    m_hn1 = static_cast<UINT>(m_list1.size());
    m_fn1 = static_cast<UINT>(m_fluxList1.size());
    if (m_coarseFresh) {   // undefined memory under the new tiles: cleared, then the stateless column
        cmd.ComputeSrvAt(1, m_hl1Va);
        cmd.Pipeline(m_clearTileK.Get());
        cmd.Dispatch(m_h.TileW() / 16, m_h.TileH() / 16, m_hn1);
        cmd.ComputeSrvAt(1, m_fl1Va);
        cmd.Pipeline(m_clearFlux.Get());
        cmd.Dispatch(m_flux.TileW() / 16, m_flux.TileH() / 16, m_fn1);
        cmd.UavBarriers({m_h.Atlas().Res(), m_soil.Res(), m_eta.Res(), m_flux.Res(), m_velA.Res(), m_velB.Res()});
        cmd.UavBarriers({m_waveA.Res(), m_waveB.Res(), m_lim.Res()});
    }
    cmd.ComputeSrvAt(1, m_hl1Va);
    cmd.Pipeline(m_bedFillK.Get());
    cmd.Dispatch(m_h.TileW() / 16, m_h.TileH() / 16, m_hn1);
    cmd.UavBarrier(m_bed.Res());
    if (reset || m_coarseFresh) {
        cmd.ComputeSrvAt(1, m_hl1Va);
        cmd.Pipeline(m_resetK.Get());
        cmd.Dispatch(m_h.TileW() / 16, m_h.TileH() / 16, m_hn1);
        cmd.ComputeSrvAt(1, m_fl1Va);
        cmd.Pipeline(m_clearFlux.Get());
        cmd.Dispatch(m_flux.TileW() / 16, m_flux.TileH() / 16, m_fn1);
        cmd.UavBarriers({m_h.Atlas().Res(), m_soil.Res(), m_flux.Res(), m_velA.Res(), m_velB.Res()});
        cmd.UavBarriers({m_waveA.Res(), m_waveB.Res(), m_lim.Res()});
    }
    m_coarseFresh = false;
    RecordImpose(cmd, m_hl1Va, m_hn1, m_velNew1 ? m_tableB1 : m_table1);   // stage 3: the twin's cells
    // STAGE 1b: THE FOLD'S LIST -- the coarse tiles whose four fine children are resident take the fine's fold
    // each coarse step (RecordCoarseStep); the twin's height kernel leaves them (their state is the fold's)
    {
        m_foldList.clear();
        const uint32_t tx1 = m_h.Atlas().TilesX(1), ty1 = m_h.Atlas().TilesY(1);
        for (uint32_t ty = 0; ty < ty1; ++ty) {
            for (uint32_t tx = 0; tx < tx1; ++tx) {
                bool all = true;
                for (uint32_t j = 0; j < 2 && all; ++j) {
                    for (uint32_t i = 0; i < 2 && all; ++i) {
                        const uint32_t fx = 2 * tx + i, fy = 2 * ty + j;
                        if (fx >= m_h.TilesX() || fy >= m_h.TilesY() || !m_h.IsResident(fx, fy)) all = false;
                    }
                }
                if (all) m_foldList.push_back(ty * tx1 + tx);
            }
        }
        m_stepList1.clear();
        {
            std::vector<uint8_t> folded(m_list1.size(), 0);
            for (uint32_t t : m_foldList) if (t < folded.size()) folded[t] = 1;
            for (uint32_t t : m_list1) if (!folded[t]) m_stepList1.push_back(t);
        }
        m_sl1Va = m_stepList1.empty() ? m_hl1Va : gpu.PushConstants(m_stepList1.data(), m_stepList1.size() * 4);
        m_sn1 = static_cast<UINT>(m_stepList1.size());
    }
    // STAGE 1c: THE SEA IN THE TWIN'S COLUMNS -- the same sweeps at mip 1, from the twin's own open sides; the
    // fine set's edge reads the coarse sea through the chain (its push, and its sweeps' up-ray source)
    if (m_cb1.sea[3] > 0.0f) {
        cmd.ComputeSrvAt(1, m_hl1Va);
        cmd.Pipeline(m_wavesK.Get());
        for (int s = 0; s < kWaveSweeps; ++s) {
            cmd.ComputeTable(2, m_waveTable1[s & 1].Base());
            cmd.Dispatch(m_h.TileW() / 16, m_h.TileH() / 16, m_hn1);
            cmd.UavBarriers({m_waveA.Res(), m_waveB.Res()});
        }
    }
}

void SweSolver::RecordCoarseStep(hal::CommandContext& cmd, Gpu& gpu) {
    PixScope scope(cmd.Native(), "swe.mip1 (the fold, one coarse step)");
    cmd.ComputeConstants(0, m_cb1);
    cmd.ComputeSrvAt(3, m_live1Va);
    // THE FOLD (stage 1b): the fine's state now, at the parities now
    if (!m_foldList.empty()) {
        const uint32_t fineNewest = m_velNew ? 1u : 0u;   // the bank the fine's last step wrote (velB when m_velNew)
        const uint32_t twinRead = m_velNew1 ? 1u : 0u;    // the bank the twin's next step reads (velB when m_velNew1)
        cmd.ComputeTable(2, m_tableFold[fineNewest][twinRead].Base());
        cmd.ComputeSrvAt(1, gpu.PushConstants(m_foldList.data(), m_foldList.size() * 4));
        cmd.Pipeline(m_foldK.Get());
        cmd.Dispatch(m_h.TileW() / 16, m_h.TileH() / 16, static_cast<UINT>(m_foldList.size()));
        cmd.UavBarriers({m_h.Atlas().Res(), m_bed.Res(), m_velA.Res(), m_velB.Res()});
    }
    // ONE COARSE STEP: the faces over the whole lattice, the share every column pays (the folded ones' faces are
    // the twin's flux kernel's, so their share is the law's), the step over the twin's own cells
    cmd.ComputeTable(2, (m_velNew1 ? m_tableB1 : m_table1).Base());
    cmd.ComputeTable(4, m_tableChain[m_velNew][m_velNew1][WinParentParity()].Base());   // stage 1c (nothing above mip 1: unread)
    cmd.ComputeSrvAt(1, m_fl1Va);
    cmd.Pipeline(m_fluxK.Get());
    cmd.Dispatch(m_flux.TileW() / 16, m_flux.TileH() / 16, m_fn1);
    cmd.UavBarriers({m_flux.Res(), m_velA.Res(), m_velB.Res()});
    m_velNew1 ^= 1;
    cmd.ComputeSrvAt(1, m_hl1Va);
    cmd.Pipeline(m_limitK.Get());
    cmd.Dispatch(m_h.TileW() / 16, m_h.TileH() / 16, m_hn1);
    cmd.UavBarrier(m_lim.Res());
    if (!m_stepList1.empty()) {
        cmd.ComputeSrvAt(1, m_sl1Va);
        cmd.Pipeline(m_heightK.Get());
        cmd.Dispatch(m_h.TileW() / 16, m_h.TileH() / 16, m_sn1);
        cmd.UavBarrier(m_h.Atlas().Res());
    }
}

// STAGE 3: AN IMPACT DUE THIS BATCH. The first event of the record not yet imposed whose zero the batch's start
// has passed: its place in the anchor's chart (its great-circle distance along its bearing from the anchor), the
// seconds since its zero, its basin depth and its table go to the constants (CsSweImpose reads them); the
// instrument says where it fell and what lies under it.
bool SweSolver::BeginImpose(Gpu& gpu, bool reset) {
    m_cb.impactB[0] = m_cb.impactB[1] = m_cb.impactB[2] = m_cb.impactB[3] = 0.0f;
    m_imposeEvent = -1;
    if (!m_impacts || !m_testH.empty() || !m_ready) return false;
    const std::vector<ImpactField::Event>& ev = m_impacts->Events();
    if (m_imposed.size() < ev.size()) m_imposed.resize(ev.size(), 0);
    if (reset) std::fill(m_imposed.begin(), m_imposed.end(), 0);   // a scrub: every event's state then, again
    for (size_t i = 0; i < ev.size(); ++i) {
        if (m_imposed[i] || ev[i].zeroUnix < 0.0 || ev[i].zeroUnix > m_simTime || !ev[i].wave.On()) continue;
        const ImpactWave& w = ev[i].wave;
        const double D = w.DistanceM(m_dom.latC, m_dom.lonC);
        const double la0 = m_dom.latC * kSweD2R, lo0 = m_dom.lonC * kSweD2R;
        const double la1 = w.P().lat * kSweD2R, lo1 = w.P().lon * kSweD2R;
        const double be = std::sin(lo1 - lo0) * std::cos(la1);   // the bearing from the anchor: east, north
        const double bn = std::cos(la0) * std::sin(la1) - std::sin(la0) * std::cos(la1) * std::cos(lo1 - lo0);
        const double bl = std::hypot(be, bn);
        m_cb.impactA[0] = static_cast<float>(bl > 0.0 ? D * be / bl : 0.0);
        m_cb.impactA[1] = static_cast<float>(bl > 0.0 ? D * bn / bl : 0.0);
        m_cb.impactA[2] = static_cast<float>(m_simTime - ev[i].zeroUnix);
        m_cb.impactA[3] = static_cast<float>(w.P().basinDepthM);
        // THE LEVELS THAT HOLD THE CAVITY take the law's state; the rest of the field receives it through the solve
        // (the table's sum is the law near its cavity and the table's truncation far from it: +-12 m before the front
        // 2600 km out for a 10 km stone, which planted on the window would be a wave from nowhere)
        double txW = 0.0, tyW = 0.0;
        const bool inW = m_dom.CellOf(w.P().lat, w.P().lon, txW, tyW) && txW >= 0.0 && tyW >= 0.0 && txW < double(m_cb.nx) && tyW < double(m_cb.ny);
        m_cb.impactB[0] = inW ? static_cast<float>(w.Terms()) : 0.0f;
        // STAGE 2c: the levels that hold the cavity take it; where the finest of them holds its lip over fewer than
        // eight cells, the event gets a PATCH of its own at the rank its radius needs (R_D over eight window cells,
        // as a power of two), about its cavity, with that level as its parent -- made now, imposed this batch.
        bool inT = false;
        int finest = -1;
        for (int k = 0; k < int(m_lv.size()); ++k) {
            Level& L = *m_lv[size_t(k)];
            double tx = 0.0, ty = 0.0;
            const bool in = L.dom.CellOf(w.P().lat, w.P().lon, tx, ty) && tx >= 0.0 && ty >= 0.0 && tx < double(L.nx) && ty < double(L.ny);
            L.imposeN = in ? static_cast<float>(w.Terms()) : 0.0f;
            if (in) { inT = true; if (finest < 0 || L.rank < m_lv[size_t(finest)]->rank) finest = k; }
        }
        if (finest >= 0 && m_bedAt) {
            const Level& F = *m_lv[size_t(finest)];
            const double cell0 = (std::min)(m_dom.dx, m_dom.dy);
            const double need = w.C().lipRadiusM / 8.0;   // the cell the lip wants
            if (need < (std::min)(F.dom.dx, F.dom.dy) * 0.999) {
                const uint32_t rank = uint32_t(std::clamp(std::floor(std::log2((std::max)(need / cell0, 1.0))), 2.0, double(F.rank) - 1.0));
                double cx = 0.0, cy = 0.0;
                m_dom.CellOf(w.P().lat, w.P().lon, cx, cy);   // the impact in rank-0 cells (the chart's inverse holds beyond the window)
                char name[48];
                snprintf(name, sizeof(name), "patch 2^%u (event %zu)", rank, i);
                const int at = AddLevel(gpu, rank, 512u, cx, cy, name, true, true);
                if (at >= 0) {
                    m_lv[size_t(at)]->imposeN = static_cast<float>(w.Terms());
                    Log("[impact] stage 2c: the cavity's lip (R_D %.1f km) stood over %.1f cells of %s: the event's own level, %s, %u cells of %.0f m, its parent %s",
                        w.C().lipRadiusM / 1000.0, w.C().lipRadiusM / (std::min)(F.dom.dx, F.dom.dy), F.name.c_str(), m_lv[size_t(at)]->name.c_str(),
                        m_lv[size_t(at)]->nx, (std::min)(m_lv[size_t(at)]->dom.dx, m_lv[size_t(at)]->dom.dy),
                        m_lv[size_t(at)]->parent >= 0 ? m_lv[size_t(m_lv[size_t(at)]->parent)]->name.c_str() : "none");
                }
            }
        }
        m_cb.impactC[0] = static_cast<float>(w.C().radiusM);
        m_cb.impactC[1] = static_cast<float>(w.C().depthM);
        m_cb.impactC[2] = m_cb.impactC[3] = 0.0f;
        m_cb.impactB[1] = static_cast<float>(w.Dir()[0]);   // the impact's direction: the chord gives the distance
        m_cb.impactB[2] = static_cast<float>(w.Dir()[1]);
        m_cb.impactB[3] = static_cast<float>(w.Dir()[2]);
        m_imposeVa = gpu.PushConstants(w.Table().data(), size_t(w.Terms()) * 16);
        m_imposeEvent = static_cast<int>(i);
        m_imposed[i] = 1;
        m_imposedAt = m_simTime;
        double tx = 0.0, ty = 0.0;
        const bool in = m_dom.CellOf(w.P().lat, w.P().lon, tx, ty) && tx >= 0.0 && ty >= 0.0 && tx < double(m_cb.nx) && ty < double(m_cb.ny);
        const float bedAt = in ? m_dom.elev[size_t(ty) * m_cb.nx + size_t(tx)] : 0.0f;
        Log("[impact] stage 3: event %zu imposed on the solver's columns at T=%.0f s after its zero (%s the window: %.1f km "
            "from the anchor, bearing %.0f deg%s%.1f m): cavity %.1f km across, %.0f m deep over a basin of %.0f m, %d terms; "
            "the law carries it from here, the twin's cells and the fine set's alike",
            i, m_simTime - ev[i].zeroUnix, in ? "inside" : "outside", D / 1000.0, std::atan2(be, bn) / kSweD2R,
            in ? "; the bed under it " : "; ", in ? bedAt : 0.0f, w.C().diameterM / 1000.0, w.C().depthM,
            w.P().basinDepthM, w.Terms());
        if (!m_lv.empty()) {   // stage 2b's gate: the ray's own arrival over the tail's bed, against the one-depth law's
            double shallow = 0.0;
            const double ray = TailRayArrivalS(w.P().lat, w.P().lon, w.P().basinDepthM, shallow);
            const double fastest = TailFastestArrivalS(w.P().lat, w.P().lon);
            Log("[impact] stage 2b: the event %s the tail (%s)%s; from it to the anchor over the tail's bed at sqrt(g h): the great-circle ray "
                "%.2f h (the shallowest water on it %.0f m), THE FASTEST PATH %.2f h (Fermat: the front's own way round the shoals); the one-depth law's %.2f h at sqrt(g %.0f)",
                inT ? "lies inside" : "lies outside", inT ? "imposed there" : "its train enters at the tail's edge",
                inW ? ", and inside the window (imposed there too)" : "; outside the window, which receives it through the tail",
                ray / 3600.0, shallow, fastest / 3600.0, w.ArrivalS(m_dom.latC, m_dom.lonC) / 3600.0, w.P().basinDepthM);
        }
        return true;
    }
    return false;
}

// ================================================================================================
//  STAGE 2c OF ONE WATER ON THE TREE (2026-10-27, Mark: "the telescope"): THE LEVELS. The tail of stage 2b is
//  the coarsest of a LIST of levels, each a rectangle of the one anchor chart at its own cell size (2^rank window
//  cells), its corner on its parent's lattice so its cells nest in the parent's exactly, stepped by the same
//  kernels on its own banks, lists and clock, coarsest first each batch. A level's edge is the sea's exterior
//  (the one edge law) at ITS PARENT's departure and velocity in the ghost cell beyond the side (Swe.hlsl
//  TailDepAt / TailUAt with gTail the parent's); the top level's edge reads the impacts' law. A level of mean
//  sea's gauge folds its wet cells' mean into its parent before each parent step (CsSweFoldTail; the window's
//  twin, of the tide's gauge, does not). The ANCHOR LEVELS telescope from the tail down to the window
//  (water.swe.telescopeLevels of telescopeCells, telescopeStep ranks apart), the window's edge reading the finest
//  of them; an EVENT whose cavity is unresolved where it falls (R_D under eight cells of the finest level holding
//  it) gets a PATCH of its own at the rank its radius needs, about its cavity, the batch it is imposed. All
//  levels carry departures over mean sea; the tide is the atlas's at the window alone.
// ================================================================================================

// THE PARENT'S FIELDS in a level's (or the window's) constants: the shift from its cells to the parent's, the
// parent cell its corner lies in, the parent's cells across and down, its tiles across (Swe.hlsl gTail/gTail2).
void SweSolver::ParentFields(SweCbData& cb, int parent, uint32_t rank, int64_t c0x, int64_t c0y) const {
    if (parent < 0 || parent >= int(m_lv.size())) {
        cb.tail[0] = -1.0f;
        cb.tail[1] = cb.tail[2] = cb.tail[3] = 0.0f;
        cb.tail2[0] = cb.tail2[1] = cb.tail2[2] = cb.tail2[3] = 0.0f;
        return;
    }
    const Level& P = *m_lv[size_t(parent)];
    cb.tail[0] = float(P.rank - rank);
    cb.tail[1] = float((c0x - P.c0x) >> P.rank);
    cb.tail[2] = float((c0y - P.c0y) >> P.rank);
    cb.tail[3] = float(P.nx);
    cb.tail2[0] = float(P.ny);
    cb.tail2[1] = float(P.h.TilesX());
    cb.tail2[2] = cb.tail2[3] = 0.0f;
}

uint32_t SweSolver::WinParentParity() const {
    return (m_winParent >= 0 && m_winParent < int(m_lv.size())) ? m_lv[size_t(m_winParent)]->velNew : 0u;
}

// A level's constants: the window's with its own lattice, chart, step and clock; no river, no sea's push, no
// soil; its tide plane mean sea (0 m); nothing above it in the magnify's sense; its parent's fields for its edge.
void SweSolver::LevelCb(Level& L) {
    SweCbData& cb = L.cb;
    cb = m_cb;
    cb.nx = L.nx;
    cb.ny = L.ny;
    cb.etaTilesX = L.h.TilesX();
    cb.fluxTilesX = L.flux.TilesX();
    cb.listCount = static_cast<uint32_t>(L.list.size());
    cb.dx = static_cast<float>(L.dom.dx);
    cb.dy = static_cast<float>(L.dom.dy);
    cb.dt = L.dt;
    cb.tideNavd = 0.0f;
    cb.faceCount = 0.0f;
    cb.soil[3] = 0.0f;
    cb.sea[3] = 0.0f;
    cb.oceanU = 0.0f;
    cb.mip = 0.0f;
    cb.mips = 1.0f;
    cb.etaTilesXUp = 0.0f;
    const double ds0 = m_dom.dx / m_dom.R, dt0 = m_dom.dy / m_dom.R, per = double(int64_t(1) << L.rank);
    cb.cellX[0] = static_cast<float>((double(L.c0x) + 0.5 * per - 0.5 * double(m_dom.nx)) * ds0);
    cb.cellX[1] = static_cast<float>(per * ds0);
    cb.cellX[2] = static_cast<float>((0.5 * double(m_dom.ny) - double(L.c0y) - 0.5 * per) * dt0);
    cb.cellX[3] = static_cast<float>(-per * dt0);
    cb.east[3] = static_cast<float>((std::min)(L.dom.dx, L.dom.dy));   // the cell's grain: the bed's level
    cb.impactB[0] = L.imposeN;   // the event's cavity inside this level, else nothing to impose here
    ParentFields(cb, L.parent, L.rank, L.c0x, L.c0y);
}

// A level's own exterior, four samples a side of its edge: the top level's the impacts' law (events outside the
// field); a nested level's nought -- its parent's departure stands in the ghost cell (the kernel adds it).
void SweSolver::LevelBoundary(Level& L) {
    for (float& v : L.cb.oceanEta) v = 0.0f;
    if (L.parent >= 0 || !m_tailLevelAt) return;
    const double nx = double(L.nx), ny = double(L.ny);
    for (int s = 0; s < 4; ++s) {
        const double len = s < 2 ? nx : ny;
        for (int k = 0; k < 4; ++k) {
            const double a = len * k / 3.0;
            const double tx = s == 2 ? 0.5 : s == 3 ? nx - 0.5 : a;
            const double ty = s == 0 ? 0.5 : s == 1 ? ny - 0.5 : a;
            double la = 0.0, lo = 0.0;
            L.dom.LatLonOf(tx, ty, la, lo);
            const double v = m_tailLevelAt(la, lo, L.simTime);
            L.cb.oceanEta[4 * s + k] = static_cast<float>(std::isfinite(v) ? v : 0.0);
        }
    }
}

// A level's tables: its step tables (the same slots over its own banks; uv, gradient, stats and ledger the
// window's, unwritten there), its chain (the parent's h, bed and read velocity bank at the chain's slot 2, by the
// parent's parity) and its fold into the parent (its banks in the read slots, the parent's water and read
// velocities in the written ones, the parent's bed at gSoilW).
void SweSolver::LevelTables(Gpu& gpu, Level& L) {
    for (int k = 0; k < 2; ++k) {
        hal::Table& t = L.table[k];
        t = hal::Table::Alloc(gpu, 16, "swe.level");
        t.Uav2D(2, L.eta.Res(), DXGI_FORMAT_R32_FLOAT);
        t.Uav2D(3, L.flux.Res(), DXGI_FORMAT_R32G32_FLOAT);
        t.UavArray(4, m_uvBank.Res(), DXGI_FORMAT_R16G16B16A16_FLOAT, 0, 1);
        t.UavArray(5, m_velGrad.Res(), DXGI_FORMAT_R16G16B16A16_FLOAT, 0, 1);
        t.Uav2D(6, L.h.Res(), DXGI_FORMAT_R32_FLOAT);
        t.Uav2D(7, (k ? L.velB : L.velA).Res(), DXGI_FORMAT_R32G32_FLOAT);
        t.Uav2D(8, L.bed.Res(), DXGI_FORMAT_R32_FLOAT);
        t.Uav2D(9, L.soil.Res(), DXGI_FORMAT_R32_FLOAT);
        t.Uav2D(10, (k ? L.velA : L.velB).Res(), DXGI_FORMAT_R32G32_FLOAT);
        t.Uav2D(11, m_statsTex.res.Get(), DXGI_FORMAT_R32G32B32A32_UINT);
        t.Uav2D(12, m_ledgerTex.res.Get(), DXGI_FORMAT_R32G32B32A32_FLOAT);
        t.Uav2D(13, m_waveB.Res(), DXGI_FORMAT_R16G16B16A16_FLOAT, 0);
        t.Uav2D(14, m_waveA.Res(), DXGI_FORMAT_R16G16B16A16_FLOAT, 0);
        t.Uav2D(15, L.lim.Res(), DXGI_FORMAT_R32_FLOAT);
    }
    const Level* P = (L.parent >= 0) ? m_lv[size_t(L.parent)].get() : nullptr;
    for (int pp = 0; pp < 2; ++pp) {
        hal::Table& t = L.chain[pp];
        t = hal::Table::Alloc(gpu, 12, "swe.level.chain");
        t.Uav2D(0, L.h.Res(), DXGI_FORMAT_R32_FLOAT);
        t.Uav2D(1, L.h.Res(), DXGI_FORMAT_R32_FLOAT);
        t.Uav2D(2, (P ? P->h : L.h).Res(), DXGI_FORMAT_R32_FLOAT);
        t.Uav2D(3, L.bed.Res(), DXGI_FORMAT_R32_FLOAT);
        t.Uav2D(4, L.bed.Res(), DXGI_FORMAT_R32_FLOAT);
        t.Uav2D(5, (P ? P->bed : L.bed).Res(), DXGI_FORMAT_R32_FLOAT);
        t.Uav2D(6, L.velA.Res(), DXGI_FORMAT_R32G32_FLOAT);
        t.Uav2D(7, L.velA.Res(), DXGI_FORMAT_R32G32_FLOAT);
        t.Uav2D(8, (P ? (pp ? P->velB : P->velA) : L.velA).Res(), DXGI_FORMAT_R32G32_FLOAT);
        t.Uav2D(9, m_waveA.Res(), DXGI_FORMAT_R16G16B16A16_FLOAT, 0);
        t.Uav2D(10, m_waveA.Res(), DXGI_FORMAT_R16G16B16A16_FLOAT, 0);
        t.Uav2D(11, m_waveA.Res(), DXGI_FORMAT_R16G16B16A16_FLOAT, 0);
    }
    if (P) {
        for (int cn = 0; cn < 2; ++cn) {
            for (int pr = 0; pr < 2; ++pr) {
                hal::Table& t = L.fold[cn][pr];
                t = hal::Table::Alloc(gpu, 16, "swe.level.fold");
                t.Uav2D(2, P->h.Res(), DXGI_FORMAT_R32_FLOAT);
                t.Uav2D(3, (pr ? P->velB : P->velA).Res(), DXGI_FORMAT_R32G32_FLOAT);
                t.UavArray(4, m_uvBank.Res(), DXGI_FORMAT_R16G16B16A16_FLOAT, 0, 1);
                t.UavArray(5, m_velGrad.Res(), DXGI_FORMAT_R16G16B16A16_FLOAT, 0, 1);
                t.Uav2D(6, L.h.Res(), DXGI_FORMAT_R32_FLOAT);
                t.Uav2D(7, (cn ? L.velB : L.velA).Res(), DXGI_FORMAT_R32G32_FLOAT);
                t.Uav2D(8, L.bed.Res(), DXGI_FORMAT_R32_FLOAT);
                t.Uav2D(9, P->bed.Res(), DXGI_FORMAT_R32_FLOAT);
                t.Uav2D(10, L.velA.Res(), DXGI_FORMAT_R32G32_FLOAT);
                t.Uav2D(11, m_statsTex.res.Get(), DXGI_FORMAT_R32G32B32A32_UINT);
                t.Uav2D(12, m_ledgerTex.res.Get(), DXGI_FORMAT_R32G32B32A32_FLOAT);
                t.Uav2D(13, m_waveB.Res(), DXGI_FORMAT_R16G16B16A16_FLOAT, 0);
                t.Uav2D(14, m_waveA.Res(), DXGI_FORMAT_R16G16B16A16_FLOAT, 0);
                t.Uav2D(15, L.lim.Res(), DXGI_FORMAT_R32_FLOAT);
            }
        }
    }
    if (m_heightArr) {   // the stack's page views (SetBed fills them for every table; a level made later takes them here)
        for (hal::Table* t : {&L.table[0], &L.table[1], &L.fold[0][0], &L.fold[0][1], &L.fold[1][0], &L.fold[1][1]}) {
            if (!t->Valid()) continue;
            t->SrvArray(0, m_heightArr, m_heightArr->GetDesc().Format, 0, UINT32_MAX, m_heightMips);
            t->SrvArray(1, m_resMapArr, DXGI_FORMAT_R8_UNORM, 0, UINT32_MAX, 1);
        }
    }
}

// A LEVEL: cells of 2^rank window cells, `cells` a side, about the rank-0 cell (cx, cy) of the window's lattice
// (the window's corner is (0, 0)); its corner on its parent's lattice; its CPU bed the stack at its grain; the
// tiles the sea reaches from its edge at mean sea mapped once; its step from its deepest water and its far
// corners' metric. Returns the level's index in the list (coarsest first), or -1 where no level can stand.
int SweSolver::AddLevel(Gpu& gpu, uint32_t rank, uint32_t cells, double cx, double cy, const char* name, bool patch, bool tables) {
    if (!m_bedAt || cells < 16 || rank == 0) return -1;
    const int64_t per = int64_t(1) << rank;
    // the parent: the finest level coarser than this one whose rectangle holds this one and a ghost cell beyond
    int parent = -1;
    int64_t c0x = int64_t(std::floor(cx - 0.5 * double(cells) * double(per))), c0y = int64_t(std::floor(cy - 0.5 * double(cells) * double(per)));
    for (int k = int(m_lv.size()) - 1; k >= 0; --k) {
        const Level& P = *m_lv[size_t(k)];
        if (P.rank <= rank) continue;
        const int64_t pp = int64_t(1) << P.rank;
        const int64_t ax = (c0x - P.c0x) / pp * pp + P.c0x, ay = (c0y - P.c0y) / pp * pp + P.c0y;   // aligned to P's lattice
        const int64_t bx = ax + int64_t(cells) * per, by = ay + int64_t(cells) * per;
        if (ax - pp >= P.c0x && ay - pp >= P.c0y && bx + pp <= P.c0x + int64_t(P.nx) * pp && by + pp <= P.c0y + int64_t(P.ny) * pp) {
            parent = k;
            c0x = ax;
            c0y = ay;
            break;
        }
    }
    if (parent < 0 && !m_lv.empty()) {
        Log("[swe] stage 2c: no level '%s' (2^%u, %u cells about (%.0f, %.0f)): no coarser level holds it", name, rank, cells, cx, cy);
        return -1;
    }
    auto Lp = std::make_unique<Level>();
    Level& L = *Lp;
    L.name = name;
    L.rank = rank;
    L.nx = L.ny = cells;
    L.c0x = c0x;
    L.c0y = c0y;
    L.parent = parent;
    L.patch = patch;
    L.foldUp = parent >= 0;
    L.dom = m_dom;
    L.dom.nx = L.dom.ny = cells;
    L.dom.dx = m_dom.dx * double(per);
    L.dom.dy = m_dom.dy * double(per);
    for (int i = 0; i < 3; ++i) {
        L.dom.U[i] = L.dom.R / L.dom.dx * L.dom.E[i] + 0.5 * double(cells) * L.dom.A[i];
        L.dom.V[i] = 0.5 * double(cells) * L.dom.A[i] - L.dom.R / L.dom.dy * L.dom.N[i];
    }
    // the rectangle's place on the chart: the centred lattice's cell less this one's (SweDomain::ox, oy)
    L.dom.ox = (double(c0x) - 0.5 * double(m_dom.nx)) / double(per) + 0.5 * double(cells);
    L.dom.oy = (double(c0y) - 0.5 * double(m_dom.ny)) / double(per) + 0.5 * double(cells);
    {
        double la0 = 90.0, la1 = -90.0, lo0 = 180.0, lo1 = -180.0;
        const double n = double(cells);
        const double pts[8][2] = {{0, 0}, {n, 0}, {0, n}, {n, n}, {0.5 * n, 0}, {0.5 * n, n}, {0, 0.5 * n}, {n, 0.5 * n}};
        for (const auto& q : pts) {
            double la = 0.0, lo = 0.0;
            L.dom.LatLonOf(q[0], q[1], la, lo);
            la0 = (std::min)(la0, la); la1 = (std::max)(la1, la);
            lo0 = (std::min)(lo0, lo); lo1 = (std::max)(lo1, lo);
        }
        L.dom.lat0 = la0; L.dom.lat1 = la1; L.dom.lon0 = lo0; L.dom.lon1 = lo1;
    }
    const auto t0 = std::chrono::steady_clock::now();
    L.dom.FillBed(m_bedAt);
    L.dom.faces.clear();
    {
        const double texel0 = 0.5 * 3.141592653589793 * L.dom.R / 16384.0;
        L.bedMip = uint32_t((std::max)(std::floor(std::log2((std::min)(L.dom.dx, L.dom.dy) / texel0)), 0.0));
    }
    const uint32_t n = cells;
    std::vector<uint8_t> sea(size_t(n) * n, 0);
    {
        std::vector<uint32_t> q;
        const auto push = [&](uint32_t i) {
            if (!sea[i] && L.dom.elev[i] > -9000.0f && L.dom.elev[i] < 0.0f) { sea[i] = 1; q.push_back(i); }
        };
        for (uint32_t a = 0; a < n; ++a) { push(a); push((n - 1) * n + a); push(a * n); push(a * n + n - 1); }
        for (size_t k = 0; k < q.size(); ++k) {
            const uint32_t i = q[k], c = i % n, r = i / n;
            if (c > 0) push(i - 1);
            if (c + 1 < n) push(i + 1);
            if (r > 0) push(i - n);
            if (r + 1 < n) push(i + n);
        }
    }
    const std::wstring wn(L.name.begin(), L.name.end());
    L.h.Init(gpu, n, n, DXGI_FORMAT_R32_FLOAT, (wn + L" h (the columns over mean sea)").c_str(), 64, 1);
    L.bed.Init(gpu, n, n, DXGI_FORMAT_R32_FLOAT, (wn + L" bed").c_str(), 64, 1);
    L.soil.Init(gpu, n, n, DXGI_FORMAT_R32_FLOAT, (wn + L" soil (unused)").c_str(), 64, 1);
    L.eta.Init(gpu, n, n, DXGI_FORMAT_R32_FLOAT, (wn + L" eta (unpublished)").c_str(), 64, 1);
    L.flux.Init(gpu, n, n, DXGI_FORMAT_R32G32_FLOAT, (wn + L" flux").c_str(), 64, 1);
    L.velA.Init(gpu, n, n, DXGI_FORMAT_R32G32_FLOAT, (wn + L" vel (one turn)").c_str(), 64, 1);
    L.velB.Init(gpu, n, n, DXGI_FORMAT_R32G32_FLOAT, (wn + L" vel (the other)").c_str(), 64, 1);
    L.lim.Init(gpu, n, n, DXGI_FORMAT_R32_FLOAT, (wn + L" lim").c_str(), 64, 1);
    uint32_t seaCells = 0;
    for (uint32_t i = 0; i < n * n; ++i) seaCells += sea[i];
    {
        const uint32_t tw = L.h.TileW(), th = L.h.TileH();
        L.live.assign(size_t(L.h.TilesX()) * L.h.TilesY(), 0u);
        for (uint32_t ty = 0; ty < L.h.TilesY(); ++ty) {
            for (uint32_t tx = 0; tx < L.h.TilesX(); ++tx) {
                bool any = false;
                for (uint32_t y = ty * th; y < (std::min)((ty + 1) * th, n) && !any; ++y) {
                    for (uint32_t x = tx * tw; x < (std::min)((tx + 1) * tw, n); ++x) {
                        if (sea[size_t(y) * n + x]) { any = true; break; }
                    }
                }
                if (!any) continue;
                L.list.push_back(ty * L.h.TilesX() + tx);
                L.live[size_t(ty) * L.h.TilesX() + tx] = 1u;
                for (TileAtlas2D* b : {&L.h, &L.bed, &L.soil, &L.eta, &L.lim}) b->RequestMap(tx, ty);
            }
        }
        for (uint32_t ty = 0; ty < L.flux.TilesY(); ++ty) {
            for (uint32_t tx = 0; tx < L.flux.TilesX(); ++tx) {
                const uint32_t hx = (tx * L.flux.TileW()) / tw, hy = (ty * L.flux.TileH()) / th;
                if (!L.live[size_t(hy) * L.h.TilesX() + hx]) continue;
                L.fluxList.push_back(ty * L.flux.TilesX() + tx);
                for (TileAtlas2D* b : {&L.flux, &L.velA, &L.velB}) b->RequestMap(tx, ty);
            }
        }
        for (TileAtlas2D* b : {&L.h, &L.bed, &L.soil, &L.eta, &L.lim, &L.flux, &L.velA, &L.velB}) b->CommitMappings(gpu, nullptr);
    }
    {
        float deepest = 0.0f;
        for (const float e : L.dom.elev) if (e > -9000.0f) deepest = (std::min)(deepest, e);
        const double ds0 = m_dom.dx / m_dom.R, dt0 = m_dom.dy / m_dom.R;
        double mMin = 1.0;   // the metric at the rectangle's corners: the smallest true cell
        for (int k = 0; k < 4; ++k) {
            const double s = (double(c0x) + ((k & 1) ? double(cells) * double(per) : 0.0) - 0.5 * double(m_dom.nx)) * ds0;
            const double t = (0.5 * double(m_dom.ny) - double(c0y) - ((k & 2) ? double(cells) * double(per) : 0.0)) * dt0;
            const double q = 1.0 + s * s + t * t;
            mMin = (std::min)(mMin, (std::min)(std::sqrt(1.0 + t * t), std::sqrt(1.0 + s * s)) / q);
        }
        L.dt = static_cast<float>(0.45 * (std::min)(L.dom.dx, L.dom.dy) * mMin / (2.0 * std::sqrt(PlanetGWave() * (std::max)(-double(deepest) + 10.0, 50.0))));
        Log("[swe] stage 2c: LEVEL '%s' -- %u x %u cells of 2^%u of the window's (%.2f x %.2f km at the anchor; %.0f x %.0f km across on the plane; "
            "the corners' cells %.2f of their size on the sphere), corner (%lld, %lld) in window cells, parent %s; the sea from its edge reaches "
            "%u cells (%.0f%%) in %zu tiles; the deepest water %.0f m; its step %.2f s; its bed the cube's mip %u; CPU bed in %.1f s%s",
            L.name.c_str(), n, n, rank, L.dom.dx / 1000.0, L.dom.dy / 1000.0, n * L.dom.dx / 1000.0, n * L.dom.dy / 1000.0, mMin,
            static_cast<long long>(c0x), static_cast<long long>(c0y), parent >= 0 ? m_lv[size_t(parent)]->name.c_str() : "none (the top: its edge reads the law)",
            seaCells, 100.0 * seaCells / double(n) / double(n), L.list.size(), -deepest, L.dt, L.bedMip,
            std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count(), patch ? " -- an event's own patch" : "");
    }
    // the list stays coarsest first: this level goes after the last level of a rank above its own
    size_t at = m_lv.size();
    while (at > 0 && m_lv[at - 1]->rank < rank) --at;
    m_lv.insert(m_lv.begin() + static_cast<std::ptrdiff_t>(at), std::move(Lp));
    for (size_t k = 0; k < m_lv.size(); ++k) {   // the indices the parents are named by move with the insertion
        if (m_lv[k]->parent >= int(at)) ++m_lv[k]->parent;
    }
    if (m_winParent >= int(at)) ++m_winParent;
    if (tables) LevelTables(gpu, *m_lv[at]);
    return int(at);
}

// THE TELESCOPE AT INIT: the tail (the top), then the anchor levels telescoping to the window; the window's parent
// is the finest level that holds it with a ghost cell beyond.
void SweSolver::InitTelescope(Gpu& gpu, const SweConfig& cfg) {
    m_bedAt = cfg.bedAt;
    if (!m_bedAt || cfg.tailCells == 0) return;
    const double cx = 0.5 * double(m_dom.nx), cy = 0.5 * double(m_dom.ny);   // the window's centre, rank-0 cells
    if (AddLevel(gpu, cfg.tailRank, cfg.tailCells, cx, cy, "tail", false, false) < 0) return;
    for (uint32_t k = 1; k <= cfg.telescopeLevels; ++k) {
        const int64_t r = int64_t(cfg.tailRank) - int64_t(k) * int64_t(cfg.telescopeStep);
        if (r < 2) break;
        char name[32];
        snprintf(name, sizeof(name), "level 2^%lld", static_cast<long long>(r));
        if (AddLevel(gpu, uint32_t(r), cfg.telescopeCells, cx, cy, name, false, false) < 0) break;
    }
    // the window's parent: the finest level holding the window and one of its own cells beyond each side
    m_winParent = -1;
    for (int k = int(m_lv.size()) - 1; k >= 0; --k) {
        const Level& P = *m_lv[size_t(k)];
        const int64_t pp = int64_t(1) << P.rank;
        if (P.c0x + pp <= 0 && P.c0y + pp <= 0 && P.c0x + int64_t(P.nx) * pp >= int64_t(m_dom.nx) + pp && P.c0y + int64_t(P.ny) * pp >= int64_t(m_dom.ny) + pp) {
            m_winParent = k;
            break;
        }
    }
    Log("[swe] stage 2c: %zu level(s); the window's edge reads %s", m_lv.size(),
        m_winParent >= 0 ? m_lv[size_t(m_winParent)]->name.c_str() : "the atlas alone (no level holds the window)");
}

void SweSolver::LevelTo(hal::CommandContext& cmd, Level& L, D3D12_RESOURCE_STATES to) {
    if (L.state == to) return;
    cmd.Barrier(L.h.Res(), L.state, to);
    cmd.Barrier(L.bed.Res(), L.state, to);
    L.state = to;
}

// A LEVEL'S BATCH -- its first instant (clear, bed, rest at mean sea), an impact due, then its steps up to the
// batch's end, each: its children's folds into it, the faces, the share, the columns. The levels run coarsest
// first, so a level's edge reads the departure its parent has carried to it this batch.
void SweSolver::RecordLevel(hal::CommandContext& cmd, Gpu& gpu, Level& L, double batchEnd, bool reset) {
    PixScope scope(cmd.Native(), L.name.c_str());
    LevelTo(cmd, L, D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
    LevelCb(L);
    if (L.fresh) L.simTime = m_simTime;
    LevelBoundary(L);
    L.liveVa = gpu.PushConstants(L.live.data(), L.live.size() * 4);
    L.hlVa = gpu.PushConstants(L.list.data(), L.list.size() * 4);
    L.flVa = gpu.PushConstants(L.fluxList.data(), L.fluxList.size() * 4);
    const UINT hn = static_cast<UINT>(L.list.size()), fn = static_cast<UINT>(L.fluxList.size());
    const Level* P = (L.parent >= 0) ? m_lv[size_t(L.parent)].get() : nullptr;
    const D3D12_GPU_VIRTUAL_ADDRESS pLiveVa = P ? gpu.PushConstants(P->live.data(), P->live.size() * 4) : L.liveVa;
    cmd.ComputeConstants(0, L.cb);
    cmd.ComputeSrvAt(3, L.liveVa);
    cmd.ComputeSrvAt(5, L.liveVa);   // nothing above in the magnify's sense (gMips = 1): unread
    cmd.ComputeSrvAt(7, pLiveVa);    // the parent's live words: the ghost
    cmd.ComputeTable(4, L.chain[P ? P->velNew : 0].Base());
    cmd.ComputeTable(2, L.table[L.velNew].Base());
    if (L.fresh) {
        cmd.ComputeSrvAt(1, L.hlVa);
        cmd.Pipeline(m_clearTileK.Get());
        cmd.Dispatch(L.h.TileW() / 16, L.h.TileH() / 16, hn);
        cmd.ComputeSrvAt(1, L.flVa);
        cmd.Pipeline(m_clearFlux.Get());
        cmd.Dispatch(L.flux.TileW() / 16, L.flux.TileH() / 16, fn);
        cmd.UavBarriers({L.h.Res(), L.soil.Res(), L.eta.Res(), L.flux.Res(), L.velA.Res(), L.velB.Res(), L.lim.Res()});
        cmd.ComputeSrvAt(1, L.hlVa);
        cmd.Pipeline(m_bedFillK.Get());
        cmd.Dispatch(L.h.TileW() / 16, L.h.TileH() / 16, hn);
        cmd.UavBarrier(L.bed.Res());
    }
    if (L.fresh || reset) {
        cmd.ComputeSrvAt(1, L.hlVa);
        cmd.Pipeline(m_resetK.Get());
        cmd.Dispatch(L.h.TileW() / 16, L.h.TileH() / 16, hn);
        cmd.ComputeSrvAt(1, L.flVa);
        cmd.Pipeline(m_clearFlux.Get());
        cmd.Dispatch(L.flux.TileW() / 16, L.flux.TileH() / 16, fn);
        cmd.UavBarriers({L.h.Res(), L.soil.Res(), L.flux.Res(), L.velA.Res(), L.velB.Res(), L.lim.Res()});
        if (reset) L.simTime = m_simTime;
    }
    L.fresh = false;
    RecordImpose(cmd, L.hlVa, hn, L.table[L.velNew]);   // stage 3: the event on this level where its cavity lies
    int steps = 0;
    while (L.simTime + double(L.dt) <= batchEnd + 1.0e-6 && steps < 256) {
        // the children's folds: each level of mean sea's gauge whose parent this is, its wet cells' mean into mine
        for (size_t k = 0; k < m_lv.size(); ++k) {
            Level& C = *m_lv[k];
            if (!C.foldUp || C.parent < 0 || m_lv[size_t(C.parent)].get() != &L || C.fresh) continue;
            LevelCb(C);
            cmd.ComputeConstants(0, C.cb);
            cmd.ComputeSrvAt(3, gpu.PushConstants(C.live.data(), C.live.size() * 4));
            cmd.ComputeTable(2, C.fold[C.velNew][L.velNew].Base());
            cmd.Pipeline(m_foldTailK.Get());
            const uint32_t across = (std::max)(C.nx >> (L.rank - C.rank), 1u);
            cmd.Dispatch(across * across, 1, 1);
            cmd.UavBarriers({L.h.Res(), L.velA.Res(), L.velB.Res()});
        }
        cmd.ComputeConstants(0, L.cb);
        cmd.ComputeSrvAt(3, L.liveVa);
        cmd.ComputeTable(2, L.table[L.velNew].Base());
        cmd.ComputeSrvAt(1, L.flVa);
        cmd.Pipeline(m_fluxK.Get());
        cmd.Dispatch(L.flux.TileW() / 16, L.flux.TileH() / 16, fn);
        cmd.UavBarriers({L.flux.Res(), L.velA.Res(), L.velB.Res()});
        L.velNew ^= 1;
        cmd.ComputeSrvAt(1, L.hlVa);
        cmd.Pipeline(m_limitK.Get());
        cmd.Dispatch(L.h.TileW() / 16, L.h.TileH() / 16, hn);
        cmd.UavBarrier(L.lim.Res());
        cmd.Pipeline(m_heightK.Get());
        cmd.Dispatch(L.h.TileW() / 16, L.h.TileH() / 16, hn);
        cmd.UavBarrier(L.h.Res());
        L.simTime += double(L.dt);
        ++steps;
    }
}

// THE LEVELS to the readers of the far field (sim/ImpactGpu.h LevelsGpu; shaders/TailRead.hlsli): coarsest first.
LevelsGpu SweSolver::LevelsGpuDesc(float exag) const {
    LevelsGpu g;
    if (m_lv.empty()) return g;
    for (int i = 0; i < 3; ++i) {
        g.up[i] = static_cast<float>(m_dom.A[i]);
        g.east[i] = static_cast<float>(m_dom.E[i]);
        g.north[i] = static_cast<float>(m_dom.N[i]);
    }
    g.R = static_cast<float>(m_dom.R);
    g.exag = exag;
    const double ds0 = m_dom.dx / m_dom.R, dt0 = m_dom.dy / m_dom.R;
    for (size_t k = 0; k < m_lv.size() && g.count < LevelsGpu::kMax; ++k) {
        const Level& L = *m_lv[k];
        if (L.fresh) continue;   // not yet on the GPU
        LevelsGpu::Level& d = g.level[g.count++];
        d.hSrv = L.h.Srv();
        d.bedSrv = L.bed.Srv();
        d.nx = L.nx;
        d.ny = L.ny;
        const double per = double(int64_t(1) << L.rank);
        d.s0 = static_cast<float>((double(L.c0x) + 0.5 * per - 0.5 * double(m_dom.nx)) * ds0);
        d.t0 = static_cast<float>((0.5 * double(m_dom.ny) - double(L.c0y) - 0.5 * per) * dt0);
        d.ds = static_cast<float>(per * ds0);
        d.dt = static_cast<float>(-per * dt0);
    }
    return g;
}

std::vector<SweSolver::LevelBox> SweSolver::LevelBoxes() const {
    std::vector<LevelBox> out;
    for (const auto& L : m_lv) {
        LevelBox b;
        b.box[0] = L->dom.lat0; b.box[1] = L->dom.lon0; b.box[2] = L->dom.lat1; b.box[3] = L->dom.lon1;
        b.mip = L->bedMip;
        out.push_back(b);
    }
    return out;
}

// STAGE 2b: THE RAY'S ARRIVAL -- from the impact to the anchor along the great circle, the long wave's time
// sum ds / sqrt(g h) cell by cell over the tail's CPU bed (h0 where the tail holds no cell; 5 m the least), and
// the shallowest water met. An instrument: the gate's reference beside the one-depth law.
double SweSolver::TailRayArrivalS(double latDeg, double lonDeg, double h0, double& shallowest) const {
    shallowest = 1.0e9;
    if (m_lv.empty()) return 0.0;
    const SweDomain& T = m_lv[0]->dom;
    const double la0 = latDeg * kSweD2R, lo0 = lonDeg * kSweD2R, la1 = m_dom.latC * kSweD2R, lo1 = m_dom.lonC * kSweD2R;
    const double p0[3] = {std::cos(la0) * std::cos(lo0), std::sin(la0), std::cos(la0) * std::sin(lo0)};
    const double p1[3] = {std::cos(la1) * std::cos(lo1), std::sin(la1), std::cos(la1) * std::sin(lo1)};
    const double ang = std::acos(std::clamp(p0[0] * p1[0] + p0[1] * p1[1] + p0[2] * p1[2], -1.0, 1.0));
    const double R = m_dom.R, D = R * ang;
    const int n = (std::max)(int(D / 2000.0), 1);
    const double ds = D / n;
    const double g = double(m_cb.gravity);
    double tSum = 0.0;
    for (int k = 0; k < n; ++k) {
        const double f = (k + 0.5) / n;
        const double a = std::sin((1.0 - f) * ang) / std::sin(ang), b = std::sin(f * ang) / std::sin(ang);
        const double q[3] = {a * p0[0] + b * p1[0], a * p0[1] + b * p1[1], a * p0[2] + b * p1[2]};
        const double la = std::asin(q[1]) / kSweD2R, lo = std::atan2(q[2], q[0]) / kSweD2R;
        double tx = 0.0, ty = 0.0;
        double h = h0;
        if (T.CellOf(la, lo, tx, ty) && tx >= 0.0 && ty >= 0.0 && tx < double(T.nx) && ty < double(T.ny)) {
            const float e = T.elev[size_t(ty) * T.nx + size_t(tx)];
            if (e > -9000.0f) h = (std::max)(-double(e), 5.0);
        }
        shallowest = (std::min)(shallowest, h);
        tSum += ds / std::sqrt(g * h);
    }
    return tSum;
}

// STAGE 2b: THE FASTEST ARRIVAL -- the least time from the impact's cell to the anchor's over the tail's sea cells
// at the long wave's speed sqrt(g h) (Dijkstra, 8-connected, each step's length the chart's true metric, its speed
// the two cells' mean): Fermat's path. The gate's reference. 0 where the impact or the anchor is no sea cell.
double SweSolver::TailFastestArrivalS(double latDeg, double lonDeg) const {
    if (m_lv.empty()) return 0.0;
    const SweDomain& T = m_lv[0]->dom;
    const uint32_t n = T.nx;
    double tx = 0.0, ty = 0.0, ax = 0.0, ay = 0.0;
    if (!T.CellOf(latDeg, lonDeg, tx, ty) || !T.CellOf(m_dom.latC, m_dom.lonC, ax, ay)) return 0.0;
    if (tx < 0.0 || ty < 0.0 || tx >= n || ty >= n || ax < 0.0 || ay < 0.0 || ax >= n || ay >= n) return 0.0;
    const uint32_t start = uint32_t(ty) * n + uint32_t(tx), goal = uint32_t(ay) * n + uint32_t(ax);
    const double g = double(m_cb.gravity), R = T.R, ds0 = T.dx / R, dt0 = T.dy / R;
    const auto speed = [&](uint32_t i) {
        const float e = T.elev[i];
        return (e > -9000.0f && e < 0.0f) ? std::sqrt(g * (std::max)(-double(e), 5.0)) : 0.0;
    };
    const auto metric = [&](uint32_t i, double& mx, double& my) {
        const double s = (double(i % n) + 0.5 - 0.5 * n) * ds0, t = (0.5 * n - double(i / n) - 0.5) * dt0;
        const double q = 1.0 + s * s + t * t;
        mx = T.dx * std::sqrt(1.0 + t * t) / q;
        my = T.dy * std::sqrt(1.0 + s * s) / q;
    };
    if (speed(start) <= 0.0 || speed(goal) <= 0.0) return 0.0;
    std::vector<float> best(size_t(n) * n, 3.0e38f);
    using Item = std::pair<float, uint32_t>;
    std::priority_queue<Item, std::vector<Item>, std::greater<Item>> open;
    best[start] = 0.0f;
    open.push({0.0f, start});
    while (!open.empty()) {
        const Item it = open.top();
        open.pop();
        const uint32_t i = it.second;
        if (it.first > best[i]) continue;
        if (i == goal) return double(it.first);
        const int c = int(i % n), r = int(i / n);
        const double ci = speed(i);
        double mx = 0.0, my = 0.0;
        metric(i, mx, my);
        for (int dr = -1; dr <= 1; ++dr) {
            for (int dc = -1; dc <= 1; ++dc) {
                if (!dr && !dc) continue;
                const int cc = c + dc, rr = r + dr;
                if (cc < 0 || rr < 0 || cc >= int(n) || rr >= int(n)) continue;
                const uint32_t j = uint32_t(rr) * n + uint32_t(cc);
                const double cj = speed(j);
                if (cj <= 0.0) continue;
                const double len = std::hypot(dc * mx, dr * my);
                const float t = it.first + static_cast<float>(len / (0.5 * (ci + cj)));
                if (t < best[j]) {
                    best[j] = t;
                    open.push({t, j});
                }
            }
        }
    }
    return 0.0;
}

void SweSolver::RecordImpose(hal::CommandContext& cmd, D3D12_GPU_VIRTUAL_ADDRESS listVa, UINT count, const hal::Table& table) {
    if (m_imposeEvent < 0 || count == 0) return;
    PixScope scope(cmd.Native(), "swe.impose (an impact's state onto the columns)");
    cmd.ComputeTable(2, table.Base());   // u7 = the velocities the next step reads
    cmd.ComputeSrvAt(6, m_imposeVa);
    cmd.ComputeSrvAt(1, listVa);
    cmd.Pipeline(m_imposeK.Get());
    cmd.Dispatch(m_h.TileW() / 16, m_h.TileH() / 16, count);
    cmd.UavBarriers({m_h.Atlas().Res(), m_velA.Res(), m_velB.Res()});
}

// THE GATE'S PROBE: the level at the place, mip 0 against mip 1 -- the water and the bed read back at the
// cell each holds the place in (a drain each, once a sim-minute: an instrument, loud by design). Stage 3: for
// the hour after an impact is imposed, every batch, with the law's own level (the Bessel sum shoaled to the
// water there, unbounded) beside the solver's at the anchor and the two sea points.
void SweSolver::ProbeCoarse(Gpu& gpu) {
    if (!m_probeOn || !m_coarse || !m_ready || m_coarseFresh) return;   // the twin has stepped
    const bool live = m_impacts && m_imposedAt > -1.0e17 && m_simTime - m_imposedAt < (!m_lv.empty() ? 8.0 * 3600.0 : 3600.0);
    if (m_simTime - m_probeAt < (live ? (!m_lv.empty() ? 60.0 : 0.0) : 30.0)) return;
    m_probeAt = m_simTime;
    // the points: the anchor, then a line out to sea (inside the fine set, then the twin's own) and one up the
    // river (the twin's own at a 1.5 km reach) -- where a disturbance stands first says where it is from
    struct Pt { const char* name; double dLat, dLon; };
    static const Pt kPts[] = {{"anchor", 0.0, 0.0}, {"sea+1km", 0.0, 0.013}, {"sea+3km", 0.0, 0.035}, {"river-3km", -0.0023, -0.035},
                              {"n2.2", 0.020, 0.013}, {"n1.1", 0.010, 0.013}, {"s1.1", -0.010, 0.013}, {"s2.2", -0.020, 0.013},
                              {"s3.3", -0.030, 0.013}, {"s4.4", -0.040, 0.013}, {"s6.7", -0.060, 0.013}};
    struct Plane { std::vector<uint8_t> raw; uint32_t pitch = 0; };
    const auto readAll = [&](TileAtlas2D& bank, uint32_t mip, Plane& out) {
        GpuTexture wrap;
        wrap.res = bank.Res();
        wrap.format = DXGI_FORMAT_R32_FLOAT;
        wrap.width = bank.TilesX(mip) * bank.TileW();
        wrap.height = bank.TilesY(mip) * bank.TileH();
        wrap.state = D3D12_RESOURCE_STATE_UNORDERED_ACCESS;
        out.raw = gpu.ReadbackTexture(wrap, &out.pitch, mip);
    };
    Plane h0, b0, h1, b1;
    readAll(m_h.Atlas(), 0, h0);
    readAll(m_bed, 0, b0);
    readAll(m_h.Atlas(), 1, h1);
    readAll(m_bed, 1, b1);
    const auto at = [](const Plane& p, uint32_t cx, uint32_t cy, float& out) {
        const size_t i = size_t(cy) * p.pitch + size_t(cx) * 4;
        if (i + 4 > p.raw.size()) return false;
        memcpy(&out, &p.raw[i], 4);
        return true;
    };
    std::string line;
    char buf[200];
    for (const Pt& pt : kPts) {
        double tx = 0.0, ty = 0.0;
        if (!TexelOf(m_probeLat + pt.dLat, m_probeLon + pt.dLon, tx, ty)) continue;
        const uint32_t cx0 = uint32_t(std::clamp(tx, 0.0, double(m_cb.nx) - 1.0)), cy0 = uint32_t(std::clamp(ty, 0.0, double(m_cb.ny) - 1.0));
        float hh0 = 0.0f, bb0 = 0.0f, hh1 = 0.0f, bb1 = 0.0f;
        const bool fine = m_h.IsResident(cx0 / m_h.TileW(), cy0 / m_h.TileH()) && at(h0, cx0, cy0, hh0) && at(b0, cx0, cy0, bb0);
        if (!at(h1, cx0 / 2u, cy0 / 2u, hh1) || !at(b1, cx0 / 2u, cy0 / 2u, bb1)) continue;
        if (fine) snprintf(buf, sizeof buf, "  %s: %+.3f|%+.3f", pt.name, bb0 + hh0, bb1 + hh1);   // mip 0 | mip 1
        else snprintf(buf, sizeof buf, "  %s: --|%+.3f (bed %.1f)", pt.name, bb1 + hh1, bb1);
        line += buf;
        if (live && (&pt - kPts) < 3) {   // the law beside it: the tide plane plus the sum, shoaled to the water there
            const double law = double(m_cb.tideNavd) + m_impacts->EtaAt(m_probeLat + pt.dLat, m_probeLon + pt.dLon, m_simTime, double(hh1), false);
            snprintf(buf, sizeof buf, " (law %+.3f)", law);
            line += buf;
        }
    }
    if (!m_lv.empty() && live) {   // stage 2b/2c: what the tail carries at the anchor's cell and east of the window, the law beside it; each patch at its centre
        const auto readLevel = [&](TileAtlas2D& bank, D3D12_RESOURCE_STATES state, Plane& out) {
            GpuTexture wrap;
            wrap.res = bank.Res();
            wrap.format = DXGI_FORMAT_R32_FLOAT;
            wrap.width = bank.TilesX() * bank.TileW();
            wrap.height = bank.TilesY() * bank.TileH();
            wrap.state = state;   // the readers' state after a batch, the kernels' within one
            out.raw = gpu.ReadbackTexture(wrap, &out.pitch, 0);
        };
        {
            Level& T = *m_lv[0];
            Plane hT, bT;
            readLevel(T.h, T.state, hT);
            readLevel(T.bed, T.state, bT);
            double cxd = 0.0, cyd = 0.0, exd = 0.0, eyd = 0.0;
            T.dom.CellOf(m_dom.latC, m_dom.lonC, cxd, cyd);
            const uint32_t cx = uint32_t(std::clamp(cxd, 0.0, double(T.nx) - 1.0)), cy = uint32_t(std::clamp(cyd, 0.0, double(T.ny) - 1.0));
            const uint32_t ex = uint32_t(std::clamp(double((int64_t(m_dom.nx) - T.c0x) >> T.rank), 0.0, double(T.nx) - 1.0));   // the first tail cell past the window's east edge
            float h0 = 0.0f, b0 = 0.0f, h1 = 0.0f, b1 = 0.0f;
            if (at(hT, cx, cy, h0) && at(bT, cx, cy, b0) && at(hT, ex, cy, h1) && at(bT, ex, cy, b1)) {
                T.dom.LatLonOf(ex + 0.5, cy + 0.5, exd, eyd);
                const double law = (b1 < 0.0f) ? m_impacts->EtaAt(exd, eyd, m_simTime, -double(b1), false) : 0.0;
                snprintf(buf, sizeof buf, "  tail: at the anchor %+.3f (bed %.0f), east of the window %+.3f (bed %.0f; the law there %+.3f)",
                         b0 < 0.0f ? h0 + b0 : 0.0f, b0, b1 < 0.0f ? h1 + b1 : 0.0f, b1, law);
                line += buf;
            }
        }
        for (auto& Lp : m_lv) {
            Level& L = *Lp;
            if (!L.patch || L.fresh) continue;
            Plane hP, bP;
            readLevel(L.h, L.state, hP);
            readLevel(L.bed, L.state, bP);
            const uint32_t cx = L.nx / 2u, cy = L.ny / 2u;
            float hh = 0.0f, bb = 0.0f;
            if (at(hP, cx, cy, hh) && at(bP, cx, cy, bb)) {
                snprintf(buf, sizeof buf, "  %s at its centre %+.1f (bed %.0f)", L.name.c_str(), bb < 0.0f ? hh + bb : 0.0f, bb);
                line += buf;
            }
        }
    }
    Log("[swe] the probe t=%.0f (%u fine tiles, the tide plane %+.3f):%s", m_simTime, m_h.ResidentCount(), m_cb.tideNavd, line.c_str());
}

void SweSolver::SetEye(double latDeg, double lonDeg, double reachM) {
    m_eyeOn = true;
    m_eyeLat = latDeg;
    m_eyeLon = lonDeg;
    if (reachM != m_reachM) {
        m_reachM = reachM;
        Log("[swe] stage 1d: mip 0 by the eye's demand, within %.0f m of its ground point (0 = the whole window); the twin's own cells beyond", reachM);
    }
}

void SweSolver::RecordReset(hal::CommandContext& cmd, D3D12_GPU_VIRTUAL_ADDRESS hList, UINT hCount,
                            D3D12_GPU_VIRTUAL_ADDRESS fList, UINT fCount) {
    PixScope scope(cmd.Native(), "swe.reset (each column -> the stateless column)");
    cmd.ComputeSrvAt(1, hList);
    cmd.Pipeline(m_resetK.Get());
    cmd.Dispatch(m_h.TileW() / 16, m_h.TileH() / 16, hCount);

    cmd.ComputeSrvAt(1, fList);
    cmd.Pipeline(m_clearFlux.Get());
    cmd.Dispatch(m_flux.TileW() / 16, m_flux.TileH() / 16, fCount);

    // Full-surface uv clear (fresh texture memory is undefined; NULL-region texels must read
    // invalid so the sea falls back to the analytic jet there).
    cmd.Pipeline(m_uvClear.Get());
    cmd.Dispatch((m_cb.nx + 15) / 16, (m_cb.ny + 15) / 16, 1);

    cmd.UavBarriers({m_h.Atlas().Res(), m_soil.Res(), m_flux.Res(), m_velA.Res(), m_velB.Res()});
    cmd.UavBarriers({m_waveA.Res(), m_waveB.Res(), m_lim.Res(), m_uvBank.Res()});
}

void SweSolver::OceanBoundary(float tideNavd, float tideRate) {
    for (float& v : m_cb.oceanEta) v = 0.0f;
    m_cb.oceanU = 0.0f;
    const double nx = double(m_cb.nx), ny = double(m_cb.ny);
    // the scripted surge (SetSurge), and its rate
    const double ts = (m_simTime - m_surgePeak) / m_surgeTau;
    const double surge = m_surgeA * std::exp(-ts * ts);
    const double surgeRate = -2.0 * ts / m_surgeTau * surge;
    double section = 0.0, levelSum = 0.0;
    int levelN = 0;
    for (int s = 0; s < 4; ++s) {
        const double len = s < 2 ? nx : ny;
        // The manager's Level at four points along the side, on its exterior cells' line.
        double y[4];
        for (int k = 0; k < 4; ++k) {
            const double a = len * k / 3.0;
            const double tx = s == 2 ? 0.5 : s == 3 ? nx - 0.5 : a;
            const double ty = s == 0 ? 0.5 : s == 1 ? ny - 0.5 : a;
            double la = 0.0, lo = 0.0;
            m_dom.LatLonOf(tx, ty, la, lo);
            const double v = m_levelAt ? m_levelAt(la, lo, m_simTime) : double(tideNavd);
            y[k] = (std::isfinite(v) ? v - double(tideNavd) : 0.0) + surge;
            m_cb.oceanEta[4 * s + k] = static_cast<float>(y[k]);
        }
        // THE OPEN SIDE'S LEVEL, SAID (2026-10-25, the asteroid's arrival drew nothing: an instrument on the
        // path the faces ride): once a sim-minute while the departure stands, which side, the manager's
        // four departures, the surge, and the instant it was asked at.
        if (m_simTime - m_openSaidT[s] >= 60.0 && (std::fabs(y[0]) > 0.5 || std::fabs(y[3]) > 0.5)) {
            m_openSaidT[s] = m_simTime;   // per side: one side's line must not silence another's (2026-10-26)
            Log("[swe] side %d at t=%.0f: the level's departure from the tide plane %+.2f %+.2f %+.2f %+.2f m (surge %+.2f; source %s)",
                s, m_simTime, y[0], y[1], y[2], y[3], surge, m_levelAt ? "the manager's Level" : "the tide plane");
        }
        // The side's live section under that Level (the kernel's cubic, Swe.hlsl OceanEtaAt).
        const double cellL = s < 2 ? double(m_cb.dx) : double(m_cb.dy);
        for (int i = 0; i < int(len); ++i) {
            const double a = (i + 0.5) / len * 3.0;
            const double l0 = -(a - 1.0) * (a - 2.0) * (a - 3.0) / 6.0, l1 = a * (a - 2.0) * (a - 3.0) / 2.0;
            const double l2 = -a * (a - 1.0) * (a - 3.0) / 2.0, l3 = a * (a - 1.0) * (a - 2.0) / 6.0;
            const double lvl = double(tideNavd) + l0 * y[0] + l1 * y[1] + l2 * y[2] + l3 * y[3];
            const float e = m_dom.elev[m_dom.EdgeCell(s, i)];
            if (m_dom.SeaEdgeCell(s, i) && lvl > double(e)) {   // the sea's cell, and the Level stands over it
                section += (lvl - double(e)) * cellL;
                levelSum += lvl;
                ++levelN;
            }
        }
    }
    m_openSea = levelN ? levelSum / levelN : double(tideNavd) + surge;
    // THE PRISM BY CONTINUITY: the open sea brings what the stateless tide says the domain must hold
    // -- its wet area at the plane times the plane's rate -- less what the rivers bring; Flather's
    // radiation carries every departure from it. (The river faces' law: radiation alone cannot carry
    // a prism.) The clamp is a sanity rail, as theirs.
    double q = WetAreaAt(tideNavd + surge) * (double(tideRate) + surgeRate);
    for (int f = 0; f < FaceCount(); ++f) q -= double(m_faceQ[f]);
    m_cb.oceanU = section > 1.0 ? static_cast<float>(std::clamp(q / section, -1.5, 1.5)) : 0.0f;
}

int SweSolver::Record(hal::CommandContext& cmd, Gpu& gpu, double simUnix, float tideNavd) {
    return Record(cmd, gpu, simUnix, tideNavd, m_stepBudget);
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
    // C2: the frame ring's water statistics, when their readback has come back.
    if (cmd.Who() == hal::Owner::Frame && m_rbStats.Pending() && gpu.ReadbackReady(m_rbStats) && m_testH.empty()) {
        const uint32_t pitch = m_rbStats.rowPitch;
        const std::vector<uint8_t> raw = gpu.ReadbackTake(m_rbStats);
        UpdateResidency(gpu, raw, pitch);
    }
    etaTo(D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
    uvTo(D3D12_RESOURCE_STATE_UNORDERED_ACCESS);

    // Clock policy. A true scrub (arrow keys, --start) means the state belongs to a time that
    // no longer exists: snap to the analytic plane. Mere STARVATION (a fast time scale asking
    // for more substeps than the frame budget allows) keeps the state -- the field stays
    // continuous, boundary-forced by the current tide -- and just re-anchors the clock, i.e.
    // time-dilated hydrodynamics instead of a visible reset flicker.
    const double drift = std::abs(simUnix - m_simTime);
    bool reset = false;
    if (m_pendingReset || drift > 3600.0) {
        reset = true;
        m_simTime = simUnix;
        m_pendingReset = false;
        m_lastTideTime = 0;   // no rate across a reset
    } else if (drift > 900.0) {
        m_simTime = simUnix - maxSub * static_cast<double>(m_dt);
        m_lastTideTime = 0;
    }

    // The tide plane's rate of rise, finite-differenced across batch starts on the SIM clock: the
    // open sea's prism (OceanBoundary). Clamp: M2 at this range peaks near 2.2e-4 m/s; anything past
    // 6e-4 is a scrub artifact.
    float tideRate = 0.0f;
    if (m_lastTideTime > 0 && m_simTime > m_lastTideTime + 1.0e-6) {
        tideRate = std::clamp(
            static_cast<float>((tideNavd - m_lastTideNavd) / (m_simTime - m_lastTideTime)),
            -6.0e-4f, 6.0e-4f);
    }
    m_lastTideNavd = tideNavd;
    m_lastTideTime = m_simTime;

    int n = static_cast<int>((simUnix - m_simTime) / m_dt);
    n = (std::max)(0, (std::min)(n, maxSub));

    m_cb.tideNavd = tideNavd;
    // M6r: each face's Flather u_ext = the prescribed transport over its LIVE wet section, and its
    // target deviation (river tide - ocean tide). The clamp is a sanity rail (river-tide currents
    // at a face peak near 1 m/s).
    for (int f = 0; f < FaceCount(); ++f) {
        float area = 0.0f;
        const float lvl = tideNavd + m_faceDEta[f];
        for (const float bed : m_faceBed[f]) area += (std::max)(lvl - bed, 0.0f) * m_faceL[f];
        m_cb.faceB[4 * f + 0] = (area > 1.0f) ? std::clamp(m_faceQ[f] / area, -1.5f, 1.5f) : 0.0f;
        m_cb.faceB[4 * f + 1] = m_faceDEta[f];
        m_cb.faceB[4 * f + 2] = m_cb.faceB[4 * f + 3] = 0.0f;
        m_faceArea[f] = area;
    }
    OceanBoundary(tideNavd, tideRate);   // C1: the open sea's Level and its transport
    SeaBoundary();                       // C4: the sea there, whose waves push
    BeginImpose(gpu, reset);             // stage 3: an impact due this batch, its state onto the columns below
    ParentFields(m_cb, m_winParent, 0u, 0, 0);   // stage 2b/2c: the window's parent level under the fine lattice

    // STAGE 1c: this dispatch's mip, the chain's mips and the mip above's stride (Swe.hlsl UpLiveAt); the live
    // words of the mip above on root SRV 4, the chains' table on root param 4 (rebound a step: the parities)
    m_cb.mip = 0.0f;
    m_cb.mips = (m_coarse && m_testH.empty()) ? 2.0f : 1.0f;
    m_cb.etaTilesXUp = m_coarse ? static_cast<float>(m_h.Atlas().TilesX(1)) : 0.0f;
    cmd.ComputeRoot(m_rs.Get());
    LogCbFingerprint();
    cmd.ComputeConstants(0, m_cb);
    cmd.ComputeTable(2, m_table.Base());
    const D3D12_GPU_VIRTUAL_ADDRESS liveVa = gpu.PushConstants(m_live.data(), m_live.size() * 4);
    cmd.ComputeSrvAt(3, liveVa);
    cmd.ComputeSrvAt(5, gpu.PushConstants(m_live1.empty() ? m_live.data() : m_live1.data(), (m_live1.empty() ? m_live.size() : m_live1.size()) * 4));
    const D3D12_GPU_VIRTUAL_ADDRESS winParentLiveVa = gpu.PushConstants(m_winParent >= 0 ? m_lv[size_t(m_winParent)]->live.data() : m_live.data(),
                                                                       (m_winParent >= 0 ? m_lv[size_t(m_winParent)]->live.size() : m_live.size()) * 4);
    cmd.ComputeSrvAt(7, winParentLiveVa);   // stage 2b/2c: the window's parent level's live words (the ghost)
    cmd.ComputeTable(4, m_tableChain[m_velNew][m_velNew1][WinParentParity()].Base());
    const auto& hl = m_testH.empty() ? m_h.Atlas().ResidentList() : m_testH;
    const auto& fl = m_testF.empty() ? m_flux.ResidentList() : m_testF;
    const D3D12_GPU_VIRTUAL_ADDRESS hlVa = gpu.PushConstants(hl.data(), hl.size() * 4);
    const D3D12_GPU_VIRTUAL_ADDRESS flVa = gpu.PushConstants(fl.data(), fl.size() * 4);

    // C2: the tiles the water's residency mapped since the last Record hold nothing yet.
    if (!m_freshH.empty() || !m_freshF.empty()) {
        if (!m_freshH.empty()) {
            cmd.ComputeSrvAt(1, gpu.PushConstants(m_freshH.data(), m_freshH.size() * 4));
            cmd.Pipeline(m_clearTileK.Get());
            cmd.Dispatch(m_h.TileW() / 16, m_h.TileH() / 16, static_cast<UINT>(m_freshH.size()));
        }
        if (!m_freshF.empty()) {
            cmd.ComputeSrvAt(1, gpu.PushConstants(m_freshF.data(), m_freshF.size() * 4));
            cmd.Pipeline(m_clearFlux.Get());
            cmd.Dispatch(m_flux.TileW() / 16, m_flux.TileH() / 16, static_cast<UINT>(m_freshF.size()));
        }
        cmd.UavBarriers({m_h.Atlas().Res(), m_soil.Res(), m_eta.Res(), m_flux.Res(), m_velA.Res(), m_velB.Res()});
        cmd.UavBarriers({m_waveA.Res(), m_waveB.Res(), m_lim.Res()});
        m_freshH.clear();
        m_freshF.clear();
    }
    const D3D12_GPU_VIRTUAL_ADDRESS ledVa =
        m_ledgerOn ? gpu.PushConstants(m_ledgerFaces.data(), m_ledgerFaces.size() * 4) : 0;

    // THE COLUMN'S BED, a frame: the stack read once a cell, where it was read eight times a cell a
    // substep.
    cmd.ComputeSrvAt(1, hlVa);
    cmd.Pipeline(m_bedFillK.Get());
    cmd.Dispatch(m_h.TileW() / 16, m_h.TileH() / 16, static_cast<UINT>(hl.size()));
    cmd.UavBarrier(m_bed.Res());
    if (reset) RecordReset(cmd, hlVa, static_cast<UINT>(hl.size()), flVa, static_cast<UINT>(fl.size()));
    RecordImpose(cmd, hlVa, static_cast<UINT>(hl.size()), m_velNew ? m_tableB : m_table);   // stage 3: the fine set
    // STAGE 2b/2c: THE LEVELS AHEAD OF THE BATCH, coarsest first -- each its steps up to the batch's end on its own
    // clock, its edge reading the departure its parent has carried to it; then the fine's bindings
    if (!m_lv.empty()) {
        for (auto& L : m_lv) RecordLevel(cmd, gpu, *L, m_simTime + double(n) * double(m_dt), reset);
        cmd.ComputeConstants(0, m_cb);
        cmd.ComputeTable(2, m_table.Base());
        cmd.ComputeSrvAt(3, liveVa);
        cmd.ComputeSrvAt(5, gpu.PushConstants(m_live1.empty() ? m_live.data() : m_live1.data(), (m_live1.empty() ? m_live.size() : m_live1.size()) * 4));
        cmd.ComputeSrvAt(7, winParentLiveVa);
        cmd.ComputeTable(4, m_tableChain[m_velNew][m_velNew1][WinParentParity()].Base());
    }
    // C4: THE SEA IN THE COLUMNS, a few sweeps a batch (an even number: the solve reads the bank they end on).
    if (m_cb.sea[3] > 0.0f) {
        cmd.ComputeSrvAt(1, hlVa);
        cmd.Pipeline(m_wavesK.Get());
        for (int s = 0; s < kWaveSweeps; ++s) {
            cmd.ComputeTable(2, m_waveTable[s & 1].Base());
            cmd.Dispatch(m_h.TileW() / 16, m_h.TileH() / 16, static_cast<UINT>(hl.size()));
            cmd.UavBarriers({m_waveA.Res(), m_waveB.Res()});
        }
        cmd.ComputeTable(2, m_table.Base());
    }

    // STAGE 1c: THE TWIN INTERLEAVED (Berger and Oliger): each coarse step is the fold of the fine's state, one
    // step of the twin, then the fine's steps up to the same time (two, the ratio of the scales), reading the
    // coarse neighbour the twin just advanced. Coupled every two fine steps, not every batch.
    const bool twin = m_coarse && m_testH.empty() && n > 0;
    const int n1 = twin ? (std::max)(1, n / 2) : 0;
    if (twin) RecordCoarseBegin(cmd, gpu, n, n1, reset);
    int i = 0;
    for (int k = 0; k < (twin ? n1 : 1); ++k) {
    if (twin) {
        RecordCoarseStep(cmd, gpu);
        cmd.ComputeConstants(0, m_cb);   // the fine's constants and live words return
        cmd.ComputeSrvAt(3, liveVa);
    }
    const int fineTo = twin ? ((k + 1) * n) / n1 : n;
    for (; i < fineTo; ++i) {
        // The substeps take turns: read the newest velocities (u7), write the other bank (u10).
        cmd.ComputeTable(2, (m_velNew ? m_tableB : m_table).Base());
        cmd.ComputeTable(4, m_tableChain[m_velNew][m_velNew1][WinParentParity()].Base());   // stage 1c: the chains at this parity
        cmd.ComputeSrvAt(1, flVa);
        cmd.Pipeline(m_fluxK.Get());
        cmd.Dispatch(m_flux.TileW() / 16, m_flux.TileH() / 16, static_cast<UINT>(fl.size()));
        cmd.UavBarriers({m_flux.Res(), m_velA.Res(), m_velB.Res()});
        m_velNew ^= 1;
        cmd.ComputeSrvAt(1, hlVa);
        cmd.Pipeline(m_limitK.Get());
        cmd.Dispatch(m_h.TileW() / 16, m_h.TileH() / 16, static_cast<UINT>(hl.size()));
        cmd.UavBarrier(m_lim.Res());
        if (m_ledgerOn) {
            cmd.ComputeSrvAt(1, ledVa);
            cmd.Pipeline(m_ledgerK.Get());
            cmd.Dispatch(1, 1, 1);
            cmd.UavBarrier(m_ledgerTex.res.Get());
        }

        cmd.ComputeSrvAt(1, hlVa);
        cmd.Pipeline(m_heightK.Get());
        cmd.Dispatch(m_h.TileW() / 16, m_h.TileH() / 16, static_cast<UINT>(hl.size()));
        cmd.UavBarrier(m_h.Atlas().Res());
    }
    }
    m_simTime += n * m_dt;

    cmd.ComputeTable(2, m_table.Base());
    cmd.ComputeSrvAt(1, hlVa);
    cmd.Pipeline(m_deriveK.Get());
    cmd.Dispatch(m_h.TileW() / 16, m_h.TileH() / 16, static_cast<UINT>(hl.size()));
    // C2: the water, tile by tile (the residency's and the step's), once the derive's currents stand.
    cmd.UavBarrier(m_uvBank.Res());
    cmd.Pipeline(m_statsK.Get());
    cmd.Dispatch(1, 1, static_cast<UINT>(hl.size()));

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
    // C2: on the frame ring, the statistics go back on their own list every few seconds of the sim (the
    // copy runs ahead of this frame's list, so it reads the frame before's: the ring is a tile wide).
    if (cmd.Who() == hal::Owner::Frame && !m_rbStats.Pending() && m_simTime - m_statsAt >= 8.0) {
        GpuTexture wrap = m_statsTex;
        wrap.state = D3D12_RESOURCE_STATE_UNORDERED_ACCESS;
        m_rbStats = gpu.ReadbackTextureBegin(wrap);
    }

    // Leave eta + uv sampleable by the domain and pixel shaders.
    etaTo(D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE |
          D3D12_RESOURCE_STATE_PIXEL_SHADER_RESOURCE);
    uvTo(D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE |
         D3D12_RESOURCE_STATE_PIXEL_SHADER_RESOURCE);
    // stage 2b/2c: every level's h and bed likewise, for the far field's readers (TailRead.hlsli)
    for (auto& L : m_lv) LevelTo(cmd, *L, D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE | D3D12_RESOURCE_STATE_PIXEL_SHADER_RESOURCE);

    char s[96];
    snprintf(s, sizeof(s), "swe %ut %.0f/%.0fMB %dss", ResidentTiles(), ResidentBytes() / 1048576.0,
             (m_h.Atlas().VirtualBytes() + m_bed.VirtualBytes() + m_eta.VirtualBytes() + m_flux.VirtualBytes() +
              m_velA.VirtualBytes() + m_velB.VirtualBytes()) / 1048576.0, n);
    stats = s;
    return n;
}

std::vector<uint8_t> SweSolver::ReadFluxRaw(Gpu& gpu, uint32_t* outW, uint32_t* outH,
                                            uint32_t* outPitch) {
    GpuTexture wrap;
    wrap.res = FluxNew().Res();   // the newest substep's
    wrap.format = DXGI_FORMAT_R32G32_FLOAT;
    wrap.width = m_flux.TilesX() * m_flux.TileW();
    wrap.height = m_flux.TilesY() * m_flux.TileH();
    wrap.state = D3D12_RESOURCE_STATE_UNORDERED_ACCESS;   // flux stays UA between frames
    *outW = wrap.width;
    *outH = wrap.height;
    return gpu.ReadbackTexture(wrap, outPitch);
}

bool SweSolver::TraceFace(Gpu& gpu, std::vector<FaceRow>& rows) {
    if (!m_ready || !m_bedBound || !m_sc || m_dom.faces.empty()) return false;
    const uint32_t len = m_dom.faces[0].side < 2 ? m_cb.nx : m_cb.ny;
    if (!m_faceK) {
        // The solver's own layout with the trace's target beside it: b0, t0 the tile list, and a
        // table of the solver's six views and u5.
        m_faceRs = hal::RootLayout{}
                       .Cbv(0)
                       .Srv(0)
                       .Table({hal::SrvRange(1, 2), hal::UavRange(0, 4), hal::UavRange(5, 11)})
                       .Srv(3)
                       .Table({hal::UavRange(16, 12)})   // stage 1c: the chains (stage 2b: and the tail)
                       .Srv(4)
                       .Srv(5)
                       .Srv(6)
                       .Build(gpu, "swe.facetrace");
        m_faceRs->SetName(L"swe face trace root signature");
        m_faceK = hal::BuildCompute(gpu, m_faceRs.Get(),
                                    m_sc->Compile(m_shaderDir + L"/Swe.hlsl", L"CsSweFlux", L"cs_6_0",
                                                  {L"SWE_FACE_TRACE=1"}),
                                    "swe.facetrace");
        if (!m_faceK) return false;
        m_faceTex = gpu.CreateTexture2D(4, (std::max)(m_cb.nx, m_cb.ny), DXGI_FORMAT_R32G32B32A32_FLOAT,
                                        D3D12_RESOURCE_FLAG_ALLOW_UNORDERED_ACCESS,
                                        D3D12_RESOURCE_STATE_UNORDERED_ACCESS,
                                        L"swe.facetrace (the Flather face's terms)");
        m_faceTable = hal::Table::Alloc(gpu, 17, "swe.facetrace");
        m_faceTable.Uav2D(16, m_lim.Res(), DXGI_FORMAT_R32_FLOAT);
        m_faceTable.Uav2D(12, m_statsTex.res.Get(), DXGI_FORMAT_R32G32B32A32_UINT);
        m_faceTable.Uav2D(13, m_ledgerTex.res.Get(), DXGI_FORMAT_R32G32B32A32_FLOAT);
        m_faceTable.Uav2D(14, m_waveB.Res(), DXGI_FORMAT_R16G16B16A16_FLOAT);
        m_faceTable.Uav2D(15, m_waveA.Res(), DXGI_FORMAT_R16G16B16A16_FLOAT);
        m_faceTable.SrvArray(0, m_bedArr, m_bedArr->GetDesc().Format, 0, UINT32_MAX, m_bedMips);
        m_faceTable.SrvArray(1, m_bedRes, DXGI_FORMAT_R8_UNORM, 0, UINT32_MAX, 1);
        m_faceTable.Uav2D(2, m_eta.Res(), DXGI_FORMAT_R32_FLOAT);
        m_faceTable.UavArray(4, m_uvBank.Res(), DXGI_FORMAT_R16G16B16A16_FLOAT, 0, 1);
        m_faceTable.UavArray(5, m_velGrad.Res(), DXGI_FORMAT_R16G16B16A16_FLOAT, 0, 1);
        m_faceTable.Uav2D(6, m_faceTex.res.Get(), DXGI_FORMAT_R32G32B32A32_FLOAT);
        m_faceTable.Uav2D(7, m_h.Atlas().Res(), DXGI_FORMAT_R32_FLOAT);
        m_faceTable.Uav2D(9, m_bed.Res(), DXGI_FORMAT_R32_FLOAT);
        m_faceTable.Uav2D(10, m_soil.Res(), DXGI_FORMAT_R32_FLOAT);
    }
    // The trace reads the newest velocities as the substep before (u7) and writes nothing of the
    // state (u1 and u10 are bound, never written).
    m_faceTable.Uav2D(3, m_flux.Res(), DXGI_FORMAT_R32G32_FLOAT);
    m_faceTable.Uav2D(8, (m_velNew ? m_velB : m_velA).Res(), DXGI_FORMAT_R32G32_FLOAT);
    m_faceTable.Uav2D(11, (m_velNew ? m_velA : m_velB).Res(), DXGI_FORMAT_R32G32_FLOAT);
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
        up.ComputeRoot(m_faceRs.Get());
        up.ComputeConstants(0, m_cb);
        const auto& fl = m_flux.ResidentList();
        up.ComputeSrvAt(1, gpu.PushConstants(fl.data(), fl.size() * 4));
        up.ComputeTable(2, m_faceTable.Base());
        up.ComputeSrvAt(3, gpu.PushConstants(m_live.data(), m_live.size() * 4));
        up.ComputeTable(4, m_tableChain[m_velNew][m_velNew1][WinParentParity()].Base());   // stage 1c
        up.ComputeSrvAt(5, gpu.PushConstants(m_live1.empty() ? m_live.data() : m_live1.data(), (m_live1.empty() ? m_live.size() : m_live1.size()) * 4));
        up.ComputeSrvAt(7, gpu.PushConstants(m_winParent >= 0 ? m_lv[size_t(m_winParent)]->live.data() : m_live.data(), (m_winParent >= 0 ? m_lv[size_t(m_winParent)]->live.size() : m_live.size()) * 4));   // stage 2b
        up.Pipeline(m_faceK.Get());
        up.Dispatch(m_flux.TileW() / 16, m_flux.TileH() / 16, static_cast<UINT>(fl.size()));
        gpu.EndUpload();
    }
    uint32_t pitch = 0;
    const std::vector<uint8_t> raw = gpu.ReadbackTexture(m_faceTex, &pitch);
    rows.assign(len, FaceRow{});
    for (uint32_t y = 0; y < len && size_t(y) * pitch + sizeof(FaceRow) <= raw.size(); ++y) {
        memcpy(&rows[y], &raw[size_t(y) * pitch], sizeof(FaceRow));   // 4 texels = 16 floats
    }
    return true;
}

double SweSolver::FaceTransport(Gpu& gpu, int f) {
    if (!m_ready || f < 0 || f >= FaceCount()) return 0.0;
    uint32_t w = 0, h = 0, pitch = 0;
    const std::vector<uint8_t> raw = ReadFluxRaw(gpu, &w, &h, &pitch);
    const SweDomain::FaceCells& F = m_dom.faces[size_t(f)];
    // The face's flux as Swe.hlsl CsSweFlux stores it: the east face (x) or the south face (y) of the
    // lower cell of the pair, + east / + south -- inward on the W and N sides, outward on E and S. A
    // NULL tile reads zero, which is no transport.
    double q = 0.0;
    for (int a = F.c0; a <= F.c1; ++a) {
        const int i = F.side == 2 ? 0 : F.side == 3 ? int(m_cb.nx) - 2 : a;
        const int j = F.side == 0 ? 0 : F.side == 1 ? int(m_cb.ny) - 2 : a;
        const size_t at = size_t(j) * pitch + size_t(i) * 8 + (F.side < 2 ? 4 : 0);   // RG32F: x east, y south
        if (at + 4 > raw.size()) continue;
        float v = 0.0f;
        memcpy(&v, &raw[at], 4);
        q += (F.side == 0 || F.side == 2) ? v : -v;
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
    // THE MIRROR'S CENSUS (stage 2 of one water on the tree, 2026-10-26): what the mirror holds, said with
    // every read -- the cells whose surface or water is not zero, their bounding box in the lattice, and the
    // tiles the eta and uv banks hold. A reader that finds the dry floor where it expected water reads this
    // line first: the mirror is mip 0's, the fine set about the eye, and beyond its reach the twin holds the
    // water at mip 1, which no reader reads yet.
    {
        uint32_t nz = 0, x0 = UINT32_MAX, y0 = UINT32_MAX, x1 = 0, y1 = 0, nzUv = 0;
        for (uint32_t y = 0; y < etaH; ++y) {
            for (uint32_t x = 0; x < etaW; ++x) {
                const bool e = etaOut[size_t(y) * etaW + x] != 0.0f;
                const bool h = uv4Out[(size_t(y) * uvW + x) * 4 + 2] > 0.0f;
                nzUv += h ? 1u : 0u;
                if (!e && !h) continue;
                ++nz;
                x0 = (std::min)(x0, x); y0 = (std::min)(y0, y); x1 = (std::max)(x1, x); y1 = (std::max)(y1, y);
            }
        }
        Log("[swe] the mirror: %u of %u cells carry a surface or water (%u hold water), x %u..%u y %u..%u of the "
            "lattice; eta tiles resident %u, uv tiles %u, h tiles %u",
            nz, etaW * etaH, nzUv, nz ? x0 : 0u, nz ? x1 : 0u, nz ? y0 : 0u, nz ? y1 : 0u, m_eta.ResidentCount(),
            m_uvBank.ResidentCount(), m_h.ResidentCount());
    }
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

namespace {

// Ritter's dam break onto a dry flat bed: the depth at x (m from the dam, + downstream) at time t.
double RitterH(double x, double t, double hL, double g) {
    const double c0 = std::sqrt(g * hL);
    const double xi = x / t;
    if (xi <= -c0) return hL;
    if (xi >= 2.0 * c0) return 0.0;
    const double a = 2.0 * c0 - xi;
    return a * a / (9.0 * g);
}

// Stoker's dam break onto a wet flat bed (hR > 0): the middle state (h_m, u_m) and the shock's speed.
void StokerState(double hL, double hR, double g, double& hm, double& um, double& s) {
    const double cL = std::sqrt(g * hL);
    const auto f = [&](double h) {
        const double uRare = 2.0 * (cL - std::sqrt(g * h));
        const double uShock = (h - hR) * std::sqrt(g * (h + hR) / (2.0 * h * hR));
        return uRare - uShock;
    };
    double lo = hR, hi = hL;
    for (int k = 0; k < 200; ++k) {
        const double mid = 0.5 * (lo + hi);
        if (f(mid) > 0.0) lo = mid;
        else hi = mid;
    }
    hm = 0.5 * (lo + hi);
    um = 2.0 * (cL - std::sqrt(g * hm));
    s = hm * um / (hm - hR);
}

double StokerH(double x, double t, double hL, double hR, double g, double hm, double um, double s) {
    const double cL = std::sqrt(g * hL), cm = std::sqrt(g * hm);
    const double xi = x / t;
    if (xi <= -cL) return hL;
    if (xi <= um - cm) {
        const double a = 2.0 * cL - xi;
        return a * a / (9.0 * g);
    }
    if (xi <= s) return hm;
    return hR;
}

}  // namespace

bool SweSolver::RunSchemeTests(Gpu& gpu) {
    if (!m_ready || !m_sc || !m_bedBound) return false;
    // The square: the largest k x k block (k <= 4) of resident state tiles wholly inside the lattice.
    const uint32_t tw = m_h.TileW(), th = m_h.TileH();
    uint32_t bx = 0, by = 0, bk = 0;
    for (uint32_t k = 4; k >= 1 && !bk; --k) {
        for (uint32_t ty = 0; ty + k <= m_h.TilesY() && !bk; ++ty) {
            for (uint32_t tx = 0; tx + k <= m_h.TilesX() && !bk; ++tx) {
                if ((tx + k) * tw > m_cb.nx || (ty + k) * th > m_cb.ny) continue;
                bool all = true;
                for (uint32_t j = ty; j < ty + k && all; ++j) {
                    for (uint32_t i = tx; i < tx + k && all; ++i) all = m_h.IsResident(i, j);
                }
                if (all) {
                    bx = tx;
                    by = ty;
                    bk = k;
                }
            }
        }
    }
    if (!bk || tw != th) {
        Log("[swe-test] no square of resident tiles to test in");
        return false;
    }
    const int x0 = int(bx * tw), y0 = int(by * th), side = int(bk * tw);
    Log("[swe-test] the square: cells (%d, %d) .. (%d, %d), %d a side (%.0f x %.0f m); dt %.3f s",
        x0, y0, x0 + side - 1, y0 + side - 1, side, side * double(m_cb.dx), side * double(m_cb.dy), double(m_dt));

    hal::Pso bedT = hal::BuildCompute(gpu, m_rs.Get(),
                                      m_sc->Compile(m_shaderDir + L"/Swe.hlsl", L"CsSweBedFill", L"cs_6_0", {L"SWE_TEST=1"}),
                                      "swe.test.bed");
    hal::Pso resetT = hal::BuildCompute(gpu, m_rs.Get(),
                                        m_sc->Compile(m_shaderDir + L"/Swe.hlsl", L"CsSweReset", L"cs_6_0", {L"SWE_TEST=1"}),
                                        "swe.test.reset");
    hal::Pso wavesT = hal::BuildCompute(gpu, m_rs.Get(),
                                        m_sc->Compile(m_shaderDir + L"/Swe.hlsl", L"CsSweWaves", L"cs_6_0", {L"SWE_TEST=1"}),
                                        "swe.test.waves");
    hal::Pso clearT = hal::BuildCompute(gpu, m_rs.Get(),
                                        m_sc->Compile(m_shaderDir + L"/Swe.hlsl", L"CsSweClearFlux", L"cs_6_0", {L"SWE_TEST=1"}),
                                        "swe.test.clear");
    if (!bedT || !resetT || !wavesT || !clearT) return false;
    std::swap(m_bedFillK, bedT);
    std::swap(m_resetK, resetT);
    std::swap(m_wavesK, wavesT);
    std::swap(m_clearFlux, clearT);
    const SweCbData saved = m_cb;
    const double savedT = m_simTime;
    // Only the square steps: its tiles are the lists and the only live tiles (outside it, walls).
    const std::vector<uint32_t> savedLive = m_live;
    std::fill(m_live.begin(), m_live.end(), 0u);
    for (uint32_t j = by; j < by + bk; ++j) {
        for (uint32_t i = bx; i < bx + bk; ++i) {
            m_testH.push_back(j * m_h.TilesX() + i);
            m_live[size_t(j) * m_h.TilesX() + i] = 1u;
        }
    }
    for (uint32_t j = 0; j < m_flux.TilesY(); ++j) {
        for (uint32_t i = 0; i < m_flux.TilesX(); ++i) {
            const uint32_t cx = i * m_flux.TileW(), cy = j * m_flux.TileH();
            if (cx >= uint32_t(x0) && cx < uint32_t(x0 + side) && cy >= uint32_t(y0) && cy < uint32_t(y0 + side)) {
                m_testF.push_back(j * m_flux.TilesX() + i);
            }
        }
    }
    // GAGAME_SWETEST: the cases to run (lake, ledger, ritter, stoker; all when unset).
    const char* only = std::getenv("GAGAME_SWETEST");
    const auto wants = [only](const char* name) { return !only || std::strstr(only, name) != nullptr; };

    const auto readR32 = [&](TileAtlas2D& bank, std::vector<float>& out) {
        GpuTexture wrap;
        wrap.res = bank.Res();
        wrap.format = DXGI_FORMAT_R32_FLOAT;
        wrap.width = bank.TilesX() * bank.TileW();
        wrap.height = bank.TilesY() * bank.TileH();
        wrap.state = D3D12_RESOURCE_STATE_UNORDERED_ACCESS;
        uint32_t pitch = 0;
        const std::vector<uint8_t> raw = gpu.ReadbackTexture(wrap, &pitch);
        out.assign(size_t(m_cb.nx) * m_cb.ny, 0.0f);
        for (uint32_t y = 0; y < m_cb.ny && size_t(y) * pitch + size_t(m_cb.nx) * 4 <= raw.size(); ++y) {
            memcpy(&out[size_t(y) * m_cb.nx], &raw[size_t(y) * pitch], size_t(m_cb.nx) * 4);
        }
    };
    const double g = double(m_cb.gravity), dx = double(m_cb.dx), dy = double(m_cb.dy);
    std::vector<float> h0, h1, bed;
    double elapsed = 0.0;
    // One case: the first instant (a Record of no substeps: the bed, the reset, the derive), then
    // `seconds` of substeps on the closed square.
    float seaHs = 0.0f;
    const auto run = [&](int kind, float hL, float hR, float level, float cd, float damp, double seconds) {
        m_cb = saved;
        m_cb.faceCount = 0.0f;
        m_cb.cd = cd;
        m_cb.damp = damp;
        m_cb.soil[3] = 0.0f;   // the scheme alone: no soil takes the water
        for (float& v : m_cb.sea) v = 0.0f;   // and no sea, unless the case states one (seaHs)
        if (seaHs > 0.0f) {
            m_cb.sea[0] = seaHs;
            m_cb.sea[1] = 2.0f * 3.14159265f / 8.0f;   // an 8 s sea
            m_cb.sea[2] = seaHs * seaHs / 16.0f;
            m_cb.sea[3] = 1.0f;
            m_cb.sea2[0] = 1.0f;   // travelling east
            m_cb.sea2[1] = 0.0f;
        }
        const float a[4] = {float(kind), float(x0), float(y0), float(side)};
        const float b[4] = {hL, hR, level, 0.0f};
        memcpy(m_cb.testA, a, sizeof(a));
        memcpy(m_cb.testB, b, sizeof(b));
        m_simTime = 1.0e9;
        m_pendingReset = true;
        {
            hal::CommandContext up(gpu, gpu.BeginUpload(), hal::Owner::Upload);
            Record(up, gpu, m_simTime, level, 0);
            gpu.EndUpload();
        }
        readR32(m_h.Atlas(), h0);
        readR32(m_bed, bed);
        const double t0 = m_simTime;
        const auto zero = [](int, double) { return 0.0; };
        AdvanceTo(gpu, t0 + seconds, [level](double) { return double(level); }, zero, zero);
        readR32(m_h.Atlas(), h1);
        elapsed = m_simTime - t0;
    };
    const auto at = [&](int i, int j) { return size_t(j) * m_cb.nx + size_t(i); };
    bool pass = true;

    // 1. A LAKE AT REST: the surface flat to 1 mm after an hour, over a rough bed with an island out of it.
    if (wants("lake")) {
        run(0, 0.0f, 0.0f, 0.0f, saved.cd, saved.damp, 3600.0);
        double dev = 0.0;
        int wet = 0, flipped = 0;
        for (int j = y0; j < y0 + side; ++j) {
            for (int i = x0; i < x0 + side; ++i) {
                const float hh = h1[at(i, j)];
                if ((h0[at(i, j)] > m_cb.wetEps) != (hh > m_cb.wetEps)) ++flipped;
                if (hh <= m_cb.wetEps) continue;
                ++wet;
                dev = (std::max)(dev, std::abs(double(bed[at(i, j)]) + double(hh)));
            }
        }
        const bool ok = dev < 1.0e-3 && flipped == 0;
        pass = pass && ok;
        Log("[swe-test] lake at rest, %.0f s: %d wet columns, max |surface - level| %.2e m, %d changed wetness -- %s "
            "(gate 1 mm)", elapsed, wet, dev, flipped, ok ? "PASS" : "FAIL");
    }
    // 2. THE LEDGER: a hump on the same lake, an hour sloshing in a closed basin: the water conserved.
    if (wants("ledger")) {
        run(3, 0.0f, 0.0f, 0.0f, saved.cd, saved.damp, 3600.0);
        double v0 = 0.0, v1 = 0.0;
        for (int j = y0; j < y0 + side; ++j) {
            for (int i = x0; i < x0 + side; ++i) {
                v0 += double(h0[at(i, j)]);
                v1 += double(h1[at(i, j)]);
            }
        }
        v0 *= dx * dy;
        v1 *= dx * dy;
        const double rel = std::abs(v1 - v0) / (std::max)(v0, 1.0);
        const bool ok = rel < 1.0e-5;
        pass = pass && ok;
        Log("[swe-test] ledger, %.0f s: volume %.6e -> %.6e m^3, relative change %.2e -- %s (gate 1e-5)", elapsed, v0,
            v1, rel, ok ? "PASS" : "FAIL");
    }
    // 3 and 4. THE DAM BREAKS along x on a flat bed, frictionless: the middle row against the solutions.
    const int jm = y0 + side / 2;
    const double xDam = double(x0) + 0.5 * side;
    const auto profile = [&](const char* name, int kind, double hL, double hR, double seconds) {
        run(kind, float(hL), float(hR), 0.0f, 0.0f, 1.0f, seconds);
        const double T = elapsed;
        double hm = 0.0, um = 0.0, s = 0.0;
        if (hR > 0.0) StokerState(hL, hR, g, hm, um, s);
        const auto exact = [&](double x) {
            return hR > 0.0 ? StokerH(x, T, hL, hR, g, hm, um, s) : RitterH(x, T, hL, g);
        };
        const double c0 = std::sqrt(g * hL);
        const double xa = -1.5 * c0 * T, xb = (hR > 0.0 ? 1.5 * s : 3.0 * c0) * T;
        double l1 = 0.0, l1n = 0.0, front = -1e9, atDam = 0.0;
        const double edge = hR > 0.0 ? 0.5 * (hm + hR) : 0.01;
        for (int i = x0; i < x0 + side; ++i) {
            const double x = (i + 0.5 - xDam) * dx;
            const double hh = double(h1[at(i, jm)]);
            if (x >= xa && x <= xb) {
                l1 += std::abs(hh - exact(x));
                l1n += exact(x);
            }
            if (hh > edge) front = (std::max)(front, x + 0.5 * dx);
            if (std::abs(x) < 0.6 * dx) atDam += 0.5 * hh;
        }
        const double frontExact = hR > 0.0 ? s * T : 2.0 * c0 * T;
        const double rel = l1 / (std::max)(l1n, 1e-9);
        const double frontErr = (front - frontExact) / dx;
        if (hR > 0.0) {
            const double xPl = 0.5 * ((um - std::sqrt(g * hm)) + s) * T;   // the middle of the middle state
            const int ip = int(std::floor(xPl / dx + xDam));
            const double hPl = (ip >= x0 && ip < x0 + side) ? double(h1[at(ip, jm)]) : -1.0;
            const bool ok = rel < 0.05 && std::abs(frontErr) <= 3.0 && std::abs(hPl - hm) < 0.03 * hm;
            pass = pass && ok;
            Log("[swe-test] %s (hL %.2f, hR %.2f), %.1f s: L1 %.2f %%; the shock at %+.1f m against %+.1f (%+.1f "
                "cells); the plateau %.4f m against %.4f; at the dam %.4f -- %s (gates 5 %%, 3 cells, 3 %%)",
                name, hL, hR, T, 100.0 * rel, front, frontExact, frontErr, hPl, hm, atDam, ok ? "PASS" : "FAIL");
        } else {
            const bool ok = rel < 0.05 && std::abs(atDam - 4.0 / 9.0 * hL) < 0.03 * hL;
            pass = pass && ok;
            Log("[swe-test] %s (hL %.2f, dry), %.1f s: L1 %.2f %%; the front (1 cm) at %+.1f m against %+.1f "
                "(%+.1f cells); at the dam %.4f m against %.4f -- %s (gates 5 %%, 3 %% of hL at the dam; the front "
                "is reported)",
                name, hL, T, 100.0 * rel, front, frontExact, frontErr, atDam, 4.0 / 9.0 * hL, ok ? "PASS" : "FAIL");
        }
        // the row, for plots: x, the solver's h, the exact h
        char p[96];
        snprintf(p, sizeof(p), "out/swe_test_%s.csv", name);
        FILE* f = nullptr;
        fopen_s(&f, p, "w");
        if (f) {
            fprintf(f, "x,h,exact\n");
            for (int i = x0; i < x0 + side; ++i) {
                const double x = (i + 0.5 - xDam) * dx;
                fprintf(f, "%.2f,%.5f,%.5f\n", x, double(h1[at(i, jm)]), exact(x));
            }
            fclose(f);
        }
    };
    if (wants("ritter")) profile("ritter", 1, 2.0, 0.0, 120.0);
    if (wants("stoker")) profile("stoker", 2, 2.0, 0.5, 120.0);

    // 5. C4, THE SETUP ON A PLANAR BEACH: a 2 m, 8 s sea entering from the west over 8 m of water, the bed
    // rising 1 in 437 to the shore 3.5 km in; an hour of the waves' push. The rise of the mean surface from
    // offshore to the shoreline against (5 gamma / 16) H_b (gamma = 0.60, the breaking law's, on Hs; H_b
    // the highest Hs along the row) and the field's 0.16 Hs offshore.
    if (wants("setup")) {
        seaHs = 2.0f;
        run(4, 0.0f, 0.0f, 0.0f, saved.cd, saved.damp, 3600.0);
        seaHs = 0.0f;
        std::vector<uint8_t> waveRaw;
        uint32_t wpitch = 0;
        {
            GpuTexture wrap;
            wrap.res = m_waveB.Res();
            wrap.format = DXGI_FORMAT_R16G16B16A16_FLOAT;
            wrap.width = m_waveB.TilesX() * m_waveB.TileW();
            wrap.height = m_waveB.TilesY() * m_waveB.TileH();
            wrap.state = D3D12_RESOURCE_STATE_UNORDERED_ACCESS;
            waveRaw = gpu.ReadbackTexture(wrap, &wpitch);
        }
        const auto hsAt = [&](int i, int j) {
            const size_t at8 = size_t(j) * wpitch + size_t(i) * 8;
            if (at8 + 8 > waveRaw.size()) return 0.0;
            uint16_t e = 0;
            memcpy(&e, &waveRaw[at8], 2);
            return 4.0 * std::sqrt((std::max)(double(HalfToFloat(e)), 0.0));
        };
        double off = 0.0, hb = 0.0, xb = 0.0;
        int nOff = 0, last = -1;
        for (int i = x0; i < x0 + side; ++i) {
            const size_t k = at(i, jm);
            if (h1[k] > m_cb.wetEps) last = i;
            const double x = (i - x0 + 0.5) * dx;
            if (x > 300.0 && x < 1200.0 && h1[k] > m_cb.wetEps) {
                off += double(bed[k]) + double(h1[k]);
                ++nOff;
            }
            const double hs = hsAt(i, jm);
            if (hs > hb) {
                hb = hs;
                xb = x;
            }
        }
        off /= (std::max)(nOff, 1);
        double shore = 0.0;
        int nS = 0;
        for (int i = (std::max)(last - 2, x0); i <= last && last >= 0; ++i) {
            shore += double(bed[at(i, jm)]) + double(h1[at(i, jm)]);
            ++nS;
        }
        shore /= (std::max)(nS, 1);
        // (5 gamma / 16) H_b is one wave's law (H = gamma h through the surf); the breaking law limits the
        // sea's Hs (gamma 0.60 on Hs), so read in one measure -- the rms wave carrying the sea's energy,
        // gamma_rms = 0.60 / sqrt 2, H_b,rms = H_b / sqrt 2 -- it is (5 x 0.60 / 16) H_b / 2. Both stated.
        const double setup = shore - off, theory = 5.0 * 0.60 / 16.0 * hb / 2.0, theoryHs = 5.0 * 0.60 / 16.0 * hb;
        const bool ok = std::abs(setup - theory) <= 0.15 * theory;
        pass = pass && ok;
        Log("[swe-test] setup, %.0f s: the sea's highest Hs %.3f m at %.0f m (H_b); the mean surface %+.4f m offshore, "
            "%+.4f m at the shoreline (%.0f m): setup %.3f m against (5 gamma / 16) H_b in the rms wave's measure "
            "%.3f m (%+.0f %%); read on Hs %.3f m; the field's 0.16 Hs offshore %.3f -- %s (gate 15 %%, rms)",
            elapsed, hb, xb, off, shore, (last - x0 + 0.5) * dx, setup, theory, 100.0 * (setup - theory) / theory,
            theoryHs, 0.16 * 2.0, ok ? "PASS" : "FAIL");
        char p[96];
        snprintf(p, sizeof(p), "out/swe_test_setup.csv");
        FILE* f = nullptr;
        fopen_s(&f, p, "w");
        if (f) {
            fprintf(f, "x,bed,h,surface,hs\n");
            for (int i = x0; i < x0 + side; ++i) {
                const size_t k = at(i, jm);
                fprintf(f, "%.2f,%.4f,%.5f,%.5f,%.4f\n", (i - x0 + 0.5) * dx, double(bed[k]), double(h1[k]),
                        double(bed[k]) + double(h1[k]), hsAt(i, jm));
            }
            fclose(f);
        }
    }

    // 6. C5, RUN-UP: Synolakis's solitary wave onto a plane beach (1987; NTHMP's BP1, the run-up law his
    // Carrier--Greenspan solution gives, R / d = 2.831 sqrt(cot beta) (H / d)^(5/4), non-breaking). The beach
    // 1 in 19.85 from the square's middle, H / d = 0.019, frictionless; at d = 50 m and at d = 100 m (the
    // run-up's tongue 9 and 18 cells long: the grid's own convergence), the step held to 0.15 s.
    for (const double d : {50.0, 100.0}) {
        if (!wants("runup")) break;
        const double cot = 19.85, H = 0.019 * d;
        m_cb.testB[0] = 0.0f;
        run(5, float(d), float(cot), 0.0f, 0.0f, 1.0f, 0.0);
        // the case's own rows: d, cot and H ride testB (run() set hL, hR, level); state them and start again
        m_cb.testB[0] = float(d);
        m_cb.testB[1] = float(cot);
        m_cb.testB[3] = float(H);
        m_cb.dt = 0.15f;
        m_simTime = 1.0e9;
        m_pendingReset = true;
        {
            hal::CommandContext up(gpu, gpu.BeginUpload(), hal::Owner::Upload);
            Record(up, gpu, m_simTime, 0.0f, 0);
            gpu.EndUpload();
        }
        readR32(m_bed, bed);
        // the run-up: the highest bed any column has wetted (1 cm), stepped to the wave's arrival and back
        double runup = -1e9;
        const double t0 = m_simTime;
        const auto zero = [](int, double) { return 0.0; };
        const float dtSaved = m_dt;
        m_dt = 0.15f;
        for (int k = 0; k < 300; ++k) {
            AdvanceTo(gpu, t0 + 2.0 * (k + 1), [](double) { return 0.0; }, zero, zero);
            readR32(m_h.Atlas(), h1);
            for (int i = x0; i < x0 + side; ++i) {
                const size_t c = at(i, jm);
                if (h1[c] > m_cb.wetEps) runup = (std::max)(runup, double(bed[c]) + double(h1[c]));
            }
        }
        m_dt = dtSaved;
        const double law = 2.831 * std::sqrt(cot) * std::pow(H / d, 1.25) * d;
        const bool ok = std::abs(runup - law) <= 0.10 * law;
        pass = pass && ok;
        Log("[swe-test] run-up (a solitary wave, H/d %.3f, onto 1 in %.2f; d %.0f m), %.0f s: the highest wetted "
            "surface %+.3f m against Synolakis's %.3f m (%+.0f %%) -- %s (gate 10 %%)",
            H / d, cot, d, m_simTime - t0, runup, law, 100.0 * (runup - law) / law, ok ? "PASS" : "FAIL");
    }

    std::swap(m_bedFillK, bedT);
    std::swap(m_resetK, resetT);
    std::swap(m_wavesK, wavesT);
    std::swap(m_clearFlux, clearT);
    m_live = savedLive;
    m_testH.clear();
    m_testF.clear();
    m_cb = saved;
    m_simTime = savedT;
    m_pendingReset = true;
    Log("[swe-test] %s", pass ? "ALL PASS" : "SOME FAIL");
    return pass;
}

WaterHold FloodWindow(const float* bed, int nx, int ny, float level, const uint8_t* faceMask,
                      std::vector<int>* labels) {
    WaterHold h;
    const size_t n = size_t(nx) * size_t(ny);
    const auto face = [&](size_t i) { return faceMask && faceMask[i] && bed[i] > -9000.0f && bed[i] < 2.0f; };
    const auto wet = [&](size_t i) { return (bed[i] > -9000.0f && bed[i] < level) || face(i); };
    const auto edgeOf = [&](size_t i) {   // the side an edge cell answers (0 N, 1 S, 2 W, 3 E; -1 inside)
        const int c = int(i % size_t(nx)), r = int(i / size_t(nx));
        return (r == 0) ? 0 : (r == ny - 1) ? 1 : (c == 0) ? 2 : (c == nx - 1) ? 3 : -1;
    };
    const auto edgeCell = [&](int e, int t) {   // e: 0 N, 1 S, 2 W, 3 E; t along that edge
        return (e == 0)   ? size_t(t)
               : (e == 1) ? size_t(ny - 1) * nx + t
               : (e == 2) ? size_t(t) * nx
                          : size_t(t) * nx + size_t(nx - 1);
    };
    // THE SEA'S SEEDS: every edge cell under the level that no river face holds -- the kernel's one edge
    // law (Swe.hlsl OceanSideOf: the Level over the cell's bed).
    const auto seaSeed = [&](size_t i) { return edgeOf(i) >= 0 && !face(i) && bed[i] > -9000.0f && bed[i] < level; };
    for (size_t i = 0; i < n; ++i) h.faceCells += face(i) ? 1 : 0;
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
        bool sea = false, edge = false;
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
            sea = sea || seaSeed(i);
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
            edge = edge || edgeOf(i) >= 0;
        }
        // Where a piece the sea does not reach meets the window's edge: a river face's cells (every
        // other wet edge cell is the sea's seed, so such a piece is joined).
        for (int e = 0; edge && !sea && e < 4; ++e) {
            const int len = (e < 2) ? nx : ny;
            WaterPiece::EdgeRun run{"NSWE"[e], -1, -1, 1.0e9f};
            for (int t = 0; t <= len; ++t) {
                const size_t j = (t < len) ? edgeCell(e, t) : 0;
                if (t < len && label[j] == id) {
                    if (run.from < 0) run.from = t;
                    run.to = t;
                    run.deepest = (std::min)(run.deepest, bed[j]);
                } else if (run.from >= 0) {
                    p.runs.push_back(run);
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
                           const uint8_t* face) {
    FloodParting f;
    std::vector<int> gl, kl;
    const WaterHold g = FloodWindow(grid, nx, ny, level, face, &gl);
    const WaterHold k = FloodWindow(kernel, nx, ny, level, face, &kl);
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
    if (nx < 2 || ny < 2 || dom.elev.size() != size_t(nx) * ny) return;
    const float dx = float(dom.dx), dz = float(dom.dy);
    const double km2 = double(dx) * dz / 1.0e6;
    const float* grid = dom.elev.data();
    const bool kernel = kernelBed && kernelBed->size() == size_t(nx) * size_t(ny);
    const float* bed = kernel ? kernelBed->data() : grid;
    const std::vector<uint8_t> faceMask = dom.FaceMask();
    const WaterHold h = FloodWindow(bed, nx, ny, kLevel, faceMask.data());
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
             "from the sea (every edge cell under it, the kernel's one edge law) and the river faces "
             "(%d wet-capable cells): %.2f km^2 joined to the sea; not joined to it: %zu pieces, %.2f km^2, %d at the window's "
             "edge (%.2f km^2)",
             cfg.name ? cfg.name : "?", when, kLevel, h.faceCells, h.joined * km2,
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
                 p.face ? "held by a river face alone" : "joined to none", X(p.c0), X(p.c1),
                 Z(p.r1), Z(p.r0));
        s += b;
        for (size_t k = 0; k < p.runs.size() && k < 4; ++k) s += (k ? "," : "") + runText(p.runs[k]);
        if (p.runs.size() > 4) s += " +" + std::to_string(p.runs.size() - 4) + " more";
    }
    for (const WaterPiece& p : h.pieces) {
        if (!p.runs.empty()) continue;
        snprintf(b, sizeof(b), " | the largest away from the edge: %.3f km^2 %s, x %.0f..%.0f z %.0f..%.0f m",
                 p.cells * km2, p.face ? "held by a river face alone" : "joined to none", X(p.c0),
                 X(p.c1), Z(p.r1), Z(p.r0));
        s += b;
        break;
    }
    Log("%s", s.c_str());
    if (!kernel) return;
    // Beside it, the survey's grid flooded by the same law: where the two beds' water parts.
    const FloodParting f = CompareFloods(grid, bed, nx, ny, kLevel, faceMask.data());
    const char* name = cfg.name ? cfg.name : "?";
    if (f.gridOnly == 0 && f.kernelOnly == 0) {
        Log("[swe] %s window: the survey's grid, flooded by the same law, joins the same water to "
            "the sea as the kernel's bed (%.2f km^2)",
            name, f.gridJoined * km2);
        return;
    }
    snprintf(b, sizeof(b),
             "[swe] %s window: the survey's grid joins %.2f km^2 that the kernel's bed does not (its "
             "flood joins %.2f km^2 to the sea, the kernel's %.2f), and the kernel's bed joins "
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
// columns at -10 m, a channel at -3 m along row 20 from the west edge to the sea (the sea reaches it
// from both ends: a wet edge cell is the sea's), a reach at -3 m along row 10 from the west edge to
// column 10 (the sea reaches it through the west edge alone), and a pool at -1 m, rows 30..32 x
// columns 10..14, that touches no edge. A second bed lifts the reach's column 5 to +2 m.
bool RunWaterHoldSelfTest() {
    constexpr int nx = 64, ny = 40, kSea = 56;
    const auto at = [&](int r, int c) { return size_t(r) * nx + c; };
    std::vector<float> b(size_t(nx) * ny, 5.0f);
    for (int r = 0; r < ny; ++r) {
        for (int c = kSea; c < nx; ++c) b[at(r, c)] = -10.0f;
    }
    for (int c = 0; c < kSea; ++c) b[at(20, c)] = -3.0f;
    for (int c = 0; c <= 10; ++c) b[at(10, c)] = -3.0f;
    for (int r = 30; r <= 32; ++r) {
        for (int c = 10; c <= 14; ++c) b[at(r, c)] = -1.0f;
    }
    std::vector<float> raised = b;
    raised[at(10, 5)] = 2.0f;
    std::vector<uint8_t> westMask(size_t(nx) * ny, 0);   // a river face along the west column
    for (int r = 0; r < ny; ++r) westMask[at(r, 0)] = 1;
    int checks = 0;
    bool ok = true;
    const auto expect = [&](bool cond, const char* what) {
        ++checks;
        if (!cond) {
            ok = false;
            Log("[waterhold]   FAIL %s", what);
        }
    };
    const WaterHold a = FloodWindow(b.data(), nx, ny, 1.0f, nullptr);
    expect(a.joined == 8 * ny + kSea + 11 && a.pieces.size() == 1 && a.pieces[0].cells == 15 && !a.pieces[0].face &&
               a.pieces[0].runs.empty(),
           "the sea joins its columns, the channel and the reach (a wet edge cell is the sea's); the pool "
           "alone is not joined: one piece of 15 cells, meeting no edge");
    const WaterHold f = FloodWindow(b.data(), nx, ny, 1.0f, westMask.data());
    expect(f.faceCells == 2 && f.joined == 8 * ny + kSea && f.pieces.size() == 2,
           "with a river face on the west: two face cells (rows 10 and 20), the channel still the sea's, the "
           "reach held by the face alone, the pool by none");
    bool reach = false, pool = false;
    for (const WaterPiece& p : f.pieces) {
        reach = reach || (p.cells == 11 && p.face && p.runs.size() == 1 && p.runs[0].edge == 'W' &&
                          p.runs[0].from == 10 && p.runs[0].to == 10 && p.runs[0].deepest == -3.0f);
        pool = pool || (p.cells == 15 && !p.face && p.runs.empty());
    }
    expect(reach && pool, "the reach meets the west edge at row 10 (deepest -3), at the face; the pool meets no edge");
    const WaterHold rk = FloodWindow(raised.data(), nx, ny, 1.0f, nullptr);
    expect(rk.pieces.size() == 2 && rk.joined == 8 * ny + kSea + 5,
           "on the raised bed the reach parts from the sea at column 5: the sea keeps 5 of it, two pieces (the "
           "5 behind the crest, the pool)");
    const FloodParting fp = CompareFloods(b.data(), raised.data(), nx, ny, 1.0f, nullptr);
    expect(fp.gridOnly == 6 && fp.kernelOnly == 0 && fp.piece.cells == 5 && fp.crestCell == int(at(10, 5)) &&
               fp.crestKernel == 2.0f && fp.crestGrid == -3.0f,
           "the grid joins 6 cells the raised bed does not (the crest and the 5 behind it); the kernel's piece "
           "of 5 parts from the sea at the crest: +2 on the kernel's bed, -3 on the grid's");
    if (ok) {
        Log("[waterhold] ---- PASS (%d checks): a wet edge cell is the sea's, a pool the edge does not touch is "
            "reported, a reach a river face alone holds is said so, and the crest where a raised bed parts a "
            "reach from the sea is found",
            checks);
    } else {
        Log("[waterhold] ---- FAIL");
    }
    return ok;
}

}  // namespace ga
