// BedTrace - --bed-trace N: the bed the solver reads, read back through its own kernel.
// Declared in app/Tools.h. An instrument: every reading records on its own upload list and waits.
#include "app/Tools.h"

#include "core/Common.h"
#include "hal/Gpu.h"
#include "sim/BathyModel.h"
#include "sim/SweSolver.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <map>
#include <string>
#include <vector>

namespace ga::app::tools {

namespace {

void WriteRaw(const std::string& path, const void* data, size_t bytes) {
    if (FILE* f = nullptr; fopen_s(&f, path.c_str(), "wb") == 0 && f) {
        fwrite(data, 1, bytes, f);
        fclose(f);
    }
}

}  // namespace

void BedTracer::Configure(const SweToolGrid& grid, const std::string& dir, ClaimFn claim) {
    if (!grid.dom || !grid.dom->Ready()) return;
    m_claim = std::move(claim);
    m_nx = grid.dom->nx;
    m_ny = grid.dom->ny;
    m_dxM = grid.dom->dx;
    m_dyM = grid.dom->dy;
    // PHASE C1: positions in the log are metres in the domain's chart (east of its west side, north
    // of its south side); the throat's point is the tools' (world.flat), taken to its cell.
    m_worldX0 = 0.0;
    m_worldZ1 = m_ny * m_dyM;
    m_dir = dir;
    std::error_code ec;
    std::filesystem::create_directories(m_dir, ec);

    // THE THROAT, on the CPU bed. The solver's probe names a point in the inlet (ReadProbes' texel
    // law); the cross-section is the run of wet-capable cells through that point's row, column by
    // column within kThroatSearchM of it, and the narrowest run is the throat -- jetty to jetty,
    // because the jetties are walls in the realized bed. The CPU bed chooses WHERE; what the
    // readings count there is the kernel's own bed.
    const std::vector<float>& e = grid.dom->elev;
    double thx = 0.0, thy = 0.0;
    grid.CellOfFlat(kThroatX, kThroatZ, thx, thy);
    const int col0 = static_cast<int>(std::floor(thx));
    const int row = std::clamp(static_cast<int>(std::floor(thy)), 0, int(m_ny) - 1);
    const int reach = static_cast<int>(std::lround(kThroatSearchM / m_dxM));
    m_throatCol = uint32_t((std::max)(col0, 0));
    m_throatRow0 = 1;   // an empty run until one is found
    m_throatRow1 = 0;
    auto wet = [&](int c, int r) {
        const float v = e[size_t(r) * m_nx + size_t(c)];
        return v > -9000.0f && v < kWetCapableNavd;
    };
    int bestW = 0;
    for (int c = (std::max)(0, col0 - reach); c <= (std::min)(int(m_nx) - 1, col0 + reach); ++c) {
        if (!wet(c, row)) continue;
        int r0 = row, r1 = row;
        while (r0 > 0 && wet(c, r0 - 1)) --r0;
        while (r1 + 1 < int(m_ny) && wet(c, r1 + 1)) ++r1;
        const int w = r1 - r0 + 1;
        const bool nearer = std::abs(c - col0) < std::abs(int(m_throatCol) - col0);
        if (bestW == 0 || w < bestW || (w == bestW && nearer)) {
            bestW = w;
            m_throatCol = uint32_t(c);
            m_throatRow0 = uint32_t(r0);
            m_throatRow1 = uint32_t(r1);
        }
    }
    if (bestW == 0) {
        Log("[bedtrace] throat: NO wet-capable cell on the probe's row within %.0f m of (%.0f, "
            "%.0f) -- the readings count no throat",
            kThroatSearchM, kThroatX, kThroatZ);
    } else {
        float lo = 1e9f, hi = -1e9f;
        for (uint32_t r = m_throatRow0; r <= m_throatRow1; ++r) {
            lo = (std::min)(lo, e[size_t(r) * m_nx + m_throatCol]);
            hi = (std::max)(hi, e[size_t(r) * m_nx + m_throatCol]);
        }
        Log("[bedtrace] throat: the narrowest run of wet-capable cells (CPU bed < %+.2f m "
            "NAVD88) through the probe's row within %.0f m of (%.0f, %.0f): column %u (x %.0f), "
            "rows %u..%u (z %.0f..%.0f), %d cells = %.0f m across; CPU bed along it %+.2f..%+.2f m",
            kWetCapableNavd, kThroatSearchM, kThroatX, kThroatZ, m_throatCol,
            m_worldX0 + (m_throatCol + 0.5) * m_dxM, m_throatRow0, m_throatRow1,
            m_worldZ1 - (m_throatRow1 + 0.5) * m_dyM, m_worldZ1 - (m_throatRow0 + 0.5) * m_dyM,
            bestW, bestW * m_dyM, lo, hi);
    }
    WriteRaw(m_dir + "/cpu_bed.f32", e.data(), e.size() * sizeof(float));
    Log("[bedtrace] readings and the CPU bed (%ux%u f32, row 0 north) go to %s", m_nx, m_ny,
        m_dir.c_str());
}

bool BedTracer::Read(Gpu& gpu, SweSolver& swe, const std::string& label, double tideNavd,
                     float floorMip) {
    if (!Configured()) return false;
    SweSolver::BedTrace t;
    if (!swe.TraceBed(gpu, t, floorMip) || t.nx != m_nx || t.ny != m_ny) {
        Log("[bedtrace] %s: the trace did not run", label.c_str());
        return false;
    }
    const uint32_t page = swe.PageSlice();
    const size_t n = t.bed.size();
    // The rule's choices over the lattice: (slice, mip) -> cells.
    std::map<uint32_t, uint64_t> hist;
    uint64_t zero = 0, below = 0;
    double sum = 0.0;
    float lo = 1e9f, hi = -1e9f;
    for (size_t i = 0; i < n; ++i) {
        ++hist[uint32_t(t.slice[i]) * 16u + t.mip[i]];
        if (t.bed[i] == 0.0f) ++zero;
        if (t.bed[i] < tideNavd) ++below;
        sum += t.bed[i];
        lo = (std::min)(lo, t.bed[i]);
        hi = (std::max)(hi, t.bed[i]);
    }
    std::string mips;
    for (const auto& [code, cells] : hist) {
        const uint32_t slice = code / 16u, mip = code % 16u;
        char b[64];
        if (slice == page) {
            snprintf(b, sizeof(b), " page m%u %.1f%%", mip, 100.0 * double(cells) / double(n));
        } else {
            snprintf(b, sizeof(b), " cube f%u m%u %.1f%%", slice, mip,
                     100.0 * double(cells) / double(n));
        }
        mips += b;
    }
    // The throat: its cells whose bed is below the tide plane -- the inlet is open where any is.
    uint32_t open = 0, cells = 0;
    float deepest = 1e9f;
    std::string throatMips;
    std::map<uint32_t, uint32_t> tm;
    for (uint32_t r = m_throatRow0; r <= m_throatRow1; ++r) {
        const size_t i = size_t(r) * m_nx + m_throatCol;
        ++cells;
        if (t.bed[i] < tideNavd) ++open;
        deepest = (std::min)(deepest, t.bed[i]);
        ++tm[t.mip[i]];
    }
    for (const auto& [mip, c] : tm) {
        char b[24];
        snprintf(b, sizeof(b), " m%u:%u", mip, c);
        throatMips += b;
    }
    // Beside it, what the manager CLAIMS over the domain -- its own copy of the map. The kernel
    // reads the copy the residency turns upload; a difference between the two is a finding.
    std::string claim;
    if (m_claim) {
        uint64_t h[16];
        const uint64_t samples = m_claim(h);
        for (int m = 0; m < 16 && samples; ++m) {
            if (!h[m]) continue;
            char b[32];
            snprintf(b, sizeof(b), " m%d %.1f%%", m, 100.0 * double(h[m]) / double(samples));
            claim += b;
        }
        claim = " | the manager's own map claims" + (claim.empty() ? std::string(" nothing") : claim);
    }
    Log("[bedtrace] %s%s: the rule read%s%s | bed %+.2f..%+.2f mean %+.2f m, %llu cells exactly "
        "0.0 (%.1f%%), %.1f%% below the tide %+.3f | THROAT %u of %u cells below the tide (%s), "
        "deepest %+.2f m, mips%s",
        label.c_str(), floorMip > 0.0f ? " [PLANTED: residency floor forced to the coarsest mip]" : "",
        mips.c_str(), claim.c_str(), lo, hi, sum / double(n), static_cast<unsigned long long>(zero),
        100.0 * double(zero) / double(n), 100.0 * double(below) / double(n), tideNavd, open, cells,
        cells == 0 ? "no throat" : open ? "OPEN" : "CLOSED", deepest, throatMips.c_str());
    WriteRaw(m_dir + "/" + label + "_bed.f32", t.bed.data(), n * sizeof(float));
    WriteRaw(m_dir + "/" + label + "_mip.u8", t.mip.data(), n);
    m_kept.push_back({label, std::move(t.bed)});
    return true;
}

void BedTracer::CompareWithWhole(const std::string& wholeLabel) const {
    const Reading* whole = nullptr;
    for (const Reading& r : m_kept) {
        if (r.label == wholeLabel) whole = &r;
    }
    if (!whole) {
        Log("[bedtrace] no reading named %s to compare against", wholeLabel.c_str());
        return;
    }
    for (const Reading& r : m_kept) {
        if (&r == whole) continue;
        double sq = 0.0, worst = 0.0, sqT = 0.0, worstT = 0.0;
        size_t at = 0;
        for (size_t i = 0; i < r.bed.size(); ++i) {
            const double d = double(r.bed[i]) - double(whole->bed[i]);
            sq += d * d;
            if (std::abs(d) > worst) {
                worst = std::abs(d);
                at = i;
            }
        }
        for (uint32_t row = m_throatRow0; row <= m_throatRow1; ++row) {
            const size_t i = size_t(row) * m_nx + m_throatCol;
            const double d = double(r.bed[i]) - double(whole->bed[i]);
            sqT += d * d;
            worstT = (std::max)(worstT, std::abs(d));
        }
        const uint32_t ax = uint32_t(at % m_nx), ay = uint32_t(at / m_nx);
        const uint32_t throatCells =
            (m_throatRow1 >= m_throatRow0) ? m_throatRow1 - m_throatRow0 + 1 : 0u;
        Log("[bedtrace] %s against %s: RMS %.3f m, largest %.3f m at (%.0f, %.0f) | throat RMS "
            "%.3f m, largest %.3f m",
            r.label.c_str(), wholeLabel.c_str(), std::sqrt(sq / double(r.bed.size())), worst,
            m_worldX0 + (ax + 0.5) * m_dxM, m_worldZ1 - (ay + 0.5) * m_dyM,
            throatCells ? std::sqrt(sqT / double(throatCells)) : 0.0, worstT);
    }
}

}  // namespace ga::app::tools
