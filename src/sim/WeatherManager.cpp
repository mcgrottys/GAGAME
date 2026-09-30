#include "sim/WeatherManager.h"

#include "core/Common.h"
#include "hal/Residency.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <thread>

namespace ga {

namespace {
constexpr double kD2R = 3.14159265358979 / 180.0;

// lon/lat (deg) -> the flat world frame (the ACT0816 tangent). The same linear constants
// every window's georef was built with, so the round trip is exact by construction.
void WorldOf(double latDeg, double lonDeg, double& x, double& z) {
    x = (lonDeg - BathyModel::kOrgLon) * BathyModel::kMPerLon;
    z = (latDeg - BathyModel::kOrgLat) * BathyModel::kMPerLat;
}
}  // namespace

void WeatherManager::Init(Compositor* comp, int heightChannel, const WaterAtlas* atlas,
                          const TideModel* tides, const GlobeModel* globe,
                          const SeaState* sea, const CurrentModel* currents) {
    m_comp = comp;
    m_hgtCh = heightChannel;
    m_atlas = atlas;
    m_tides = tides;
    m_globe = globe;
    m_sea = sea;
    m_currents = currents;
}

void WeatherManager::AddExternalWindow(const char* name, SweSolver* solver,
                                       const BathyModel* bathy,
                                       std::function<double(double)> oceanAt) {
    Window w;
    w.name = name;
    w.solver = solver;
    w.bathy = bathy;
    w.oceanAt = std::move(oceanAt);
    w.active = solver && solver->Ready();
    snprintf(w.levelTag, sizeof(w.levelTag), "swe.%s + stations", name);
    snprintf(w.currentTag, sizeof(w.currentTag), "swe.%s (solved)", name);
    m_windows.push_back(std::move(w));
}

void WeatherManager::AddDormantWindow(const char* name, BathyModel* bathy,
                                      const SweConfig& cfg,
                                      std::function<double(double)> oceanAt,
                                      double spinupHours) {
    // Said once here too: a dormant window's solver only Inits on activation.
    if (bathy && bathy->Ready()) {
        SweConfig named = cfg;
        named.name = name;
        LogWindowHoldsWater(*bathy, named,
                            "dormant: on the CPU bed, its survey before an activation realizes the "
                            "bed, and the kernel's bed may differ");
    }
    Window w;
    w.name = name;
    w.bathy = bathy;
    w.bathyMut = bathy;
    w.cfg = cfg;
    w.cfg.name = nullptr;   // points at the caller's literal; rebound on activation
    w.oceanAt = std::move(oceanAt);
    w.spinupHours = spinupHours;
    snprintf(w.levelTag, sizeof(w.levelTag), "swe.%s + stations", name);
    snprintf(w.currentTag, sizeof(w.currentTag), "swe.%s (solved)", name);
    m_windows.push_back(std::move(w));
}

bool WeatherManager::Activate(Gpu& gpu, ShaderCompiler& sc, const std::wstring& shaderDir,
                              const char* name, double simUnix) {
    for (Window& w : m_windows) {
        if (w.name != name) continue;
        if (w.active) return true;
        if (!w.bathy || !w.bathy->Ready()) return false;
        Log("[weather] spinning up the %s window (%dx%d cells, %.1f h of history -- a "
            "one-time cost as the region becomes RESIDENT)",
            w.name.c_str(), w.bathy->Nx(), w.bathy->Ny(), w.spinupHours);
        // The window's bed realizes from the ONE height channel first (M6w): CUDEM inset,
        // shelf beyond, hand edits on top -- lazily, as part of the activation cost.
        if (w.bathyMut && m_comp && m_hgtCh >= 0) {
            w.bathyMut->RealizeFromChannel(*m_comp, m_hgtCh);
        }
        // M9ar: NO per-window bed texture. The owned solver reads the height megatexture --
        // the same page slice the Merrimack solver, the sea shader and the water bank read.
        if (!m_hgtArr || !m_hgtRes) {
            Log("[weather] %s: cannot activate -- no height page bound and no fallback bed by "
                "design",
                w.name.c_str());
            return false;
        }
        w.owned = std::make_unique<SweSolver>();
        SweConfig cfg = w.cfg;
        cfg.name = w.name.c_str();
        w.owned->Init(gpu, sc, shaderDir, *w.bathy, cfg);
        w.owned->SetHeightPage(gpu, m_hgtArr, m_hgtRes, m_hgtSlice, m_hgtMips, m_hgtWin);
        w.solver = w.owned.get();
        auto zero = [](double) { return 0.0; };
        w.solver->Spinup(gpu, simUnix, w.spinupHours, w.oceanAt, zero, zero, zero);
        w.active = true;
        // No mirror read here: the window's mirror fills on the first RefreshMirrorsTo (the
        // probe harness calls it right after Update); the camera-rule activation in the
        // loop has no reader to fill it for.
        return true;
    }
    return false;
}

void WeatherManager::RefreshMirrorsTo(Gpu& gpu, double simUnix) {
    ReadMirrors(gpu, simUnix, kMirrorDt);
}

void WeatherManager::RequestRegion(double latDeg, double lonDeg, double radiusM) {
    double x = 0.0, z = 0.0;
    WorldOf(latDeg, lonDeg, x, z);
    for (Window& w : m_windows) {
        if (!w.active || !w.solver || !w.solver->Ready() || !w.bathy) continue;
        // A region may reach a window whose box its centre is outside of.
        if (x + radiusM < w.bathy->WorldX0() || x - radiusM > w.bathy->WorldX0() + w.bathy->WorldSizeX() ||
            z + radiusM < w.bathy->WorldZ0() || z - radiusM > w.bathy->WorldZ0() + w.bathy->WorldSizeZ()) {
            continue;
        }
        w.solver->RequestRegion(x, z, radiusM);
    }
}

double WeatherManager::SolverAsOf() const {
    double asOf = kNeverRead;
    bool any = false;
    for (const Window& w : m_windows) {
        if (!w.active || !w.solver) continue;
        const double t = (std::max)(w.solver->DeliveredAsOf(), (w.etaW > 0) ? w.mirrorT : kNeverRead);
        if (t <= kNeverRead) continue;
        asOf = any ? (std::min)(asOf, t) : t;
        any = true;
    }
    return asOf;
}

void WeatherManager::SolverRefine(double latDeg, double lonDeg, double unixT,
                                  WeatherSample& s) const {
    const Window* w = WindowAt(latDeg, lonDeg);
    if (!w || !w->solver || !w->solver->Ready()) return;
    double x = 0.0, z = 0.0;
    WorldOf(latDeg, lonDeg, x, z);
    const double wDom = w->solver->DomainWeight(x, z);
    if (!(wDom > 0.0)) return;
    SweSolver::Deviation d;
    const bool have =
        w->solver->DeviationAt(x, z, d) ||
        w->solver->DeviationFromFields(x, z, w->eta, w->etaW, w->uv4, w->uvW, w->mirrorT, d);
    if (!have) {
        // THE SOLVER OWNS THIS SURFACE AND HAS NOT ANSWERED HERE. The atlas is not its answer.
        s.levelSrc = "-";
        s.depthM = static_cast<float>(s.levelNavd - s.bedNavd);
        return;
    }
    const double atlas = s.levelNavd;
    s.levelNavd = atlas + wDom * (w->oceanAt(unixT) + double(d.dEta) - atlas);
    s.levelSrc = w->levelTag;
    if (d.currentValid) {
        s.u = d.u;
        s.v = d.v;
        s.currentSrc = w->currentTag;
        s.currentSolved = true;
    }
    s.depthM = static_cast<float>(s.levelNavd - s.bedNavd);
}

void WeatherManager::ReadMirrors(Gpu& gpu, double simUnix, double maxAge) {
    for (Window& w : m_windows) {
        if (!w.active || !w.solver || !w.solver->Ready()) continue;
        if (simUnix - w.mirrorT <= maxAge) continue;   // inside the contract: no drain
        const auto t0 = std::chrono::steady_clock::now();
        w.solver->ReadFields(gpu, w.eta, w.etaW, w.etaH, w.uv4, w.uvW, w.uvH);
        w.mirrorT = simUnix;
        const double ms =
            std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count() *
            1000.0;
        // Loud by design: two full GPU drains. One line per probe/dump/trace run is the
        // expectation; a line per frame means a reader crept into the loop (step 2) -- play reads
        // the solver through RequestRegion, never through this.
        Log("[weather] %s mirror read back to t=%.0f: %ux%u eta + uv in %.1f ms (on demand: "
            "a frame-loop reader would print this every %.0f sim-s)",
            w.name.c_str(), simUnix, w.etaW, w.etaH, ms, maxAge);
    }
}

bool WeatherManager::DomainUv(const BathyModel& b, float& u0, float& v0, float& u1,
                              float& v1) const {
    const double piP = 3.14159265358979, n14 = 16384.0 * 256.0;
    // M12 step 4b: the origin is the window lattice's (an integer below 2^24: the double the
    // two members held, exactly).
    auto mercU = [&](double lonDeg) {
        return ((lonDeg + 180.0) / 360.0 * n14 - static_cast<double>(m_hgtWin.orgPxX)) /
               16384.0;
    };
    auto mercV = [&](double latDeg) {
        const double l = latDeg * piP / 180.0;
        return ((0.5 - std::log(std::tan(piP * 0.25 + l * 0.5)) / (2.0 * piP)) * n14 -
                static_cast<double>(m_hgtWin.orgPxY)) /
               16384.0;
    };
    const double lon1 = b.Lon0() + b.Nx() * b.Dlon();
    const double lat0 = b.Lat1() - b.Ny() * b.Dlat();
    u0 = float((std::max)(0.0, mercU(b.Lon0())));
    u1 = float((std::min)(1.0, mercU(lon1)));
    v0 = float((std::max)(0.0, mercV(b.Lat1())));
    v1 = float((std::min)(1.0, mercV(lat0)));
    return u1 > u0 && v1 > v0;
}

void WeatherManager::PinDomains(ResidencyManager& res, int hgtTenant) {
    if (hgtTenant < 0 || !m_hgtArr) return;
    std::string pinned;
    for (const Window& w : m_windows) {
        if (!w.active || !w.bathy || !w.bathy->Ready()) continue;
        float u0 = 0.0f, v0 = 0.0f, u1 = 0.0f, v1 = 0.0f;
        const bool inside = DomainUv(*w.bathy, u0, v0, u1, v1);
        // M13: the solver's domain is its own reader of the shared cache -- it pins the bed
        // under its lattice whether or not any view is looking there.
        if (inside) res.Want(res.Sampler("solver"), hgtTenant, m_hgtSlice, 0u, u0, v0, u1, v1);
        if (!m_pinLogged) {
            char line[160];
            snprintf(line, sizeof(line), " %s uv %.4f..%.4f x %.4f..%.4f (%s)", w.name.c_str(),
                     u0, u1, v0, v1, inside ? "inside the page" : "OUTSIDE -- unpinned");
            pinned += line;
        }
    }
    if (!m_pinLogged && !pinned.empty()) {
        m_pinLogged = true;
        Log("[weather] solver domains pinned on height page slice %u mip 0:%s", m_hgtSlice,
            pinned.c_str());
    }
}

uint64_t WeatherManager::ClaimedMips(const ResidencyManager& res, int hgtTenant,
                                     uint64_t hist[16], const char* only) const {
    for (int m = 0; m < 16; ++m) hist[m] = 0;
    if (hgtTenant < 0 || !m_hgtArr) return 0;
    uint64_t n = 0;
    for (const Window& w : m_windows) {
        if (!w.active || !w.bathy || !w.bathy->Ready()) continue;
        if (only && w.name != only) continue;
        float u0 = 0.0f, v0 = 0.0f, u1 = 0.0f, v1 = 0.0f;
        if (!DomainUv(*w.bathy, u0, v0, u1, v1)) continue;
        // A quarter of a byte's span apart, both edges included: the map is at tile grain, so
        // this is a few thousand reads of the manager's own bytes.
        const int nu = static_cast<int>(std::ceil((u1 - u0) * 512.0f));
        const int nv = static_cast<int>(std::ceil((v1 - v0) * 512.0f));
        for (int j = 0; j <= nv; ++j) {
            const float v = v0 + (v1 - v0) * float(j) / float((std::max)(nv, 1));
            for (int i = 0; i <= nu; ++i) {
                const float u = u0 + (u1 - u0) * float(i) / float((std::max)(nu, 1));
                ++hist[(std::min)(res.ResidentMipAt(hgtTenant, m_hgtSlice, u, v), 15u)];
                ++n;
            }
        }
    }
    return n;
}

WeatherManager::BedWait WeatherManager::WaitForBeds(Gpu& gpu, ResidencyManager& res,
                                                    int hgtTenant, const char* only) {
    BedWait bw;
    if (hgtTenant < 0 || !m_hgtArr) return bw;
    struct Domain {
        SweSolver* solver;
        float u0, v0, u1, v1;
    };
    std::vector<Domain> domains;
    for (Window& w : m_windows) {
        if (!w.active || !w.solver || !w.solver->Ready() || !w.bathy || !w.bathy->Ready()) continue;
        if (only && w.name != only) continue;
        Domain d{w.solver, 0.0f, 0.0f, 0.0f, 0.0f};
        if (DomainUv(*w.bathy, d.u0, d.v0, d.u1, d.v1)) domains.push_back(d);
    }
    bw.windows = static_cast<uint32_t>(domains.size());
    if (domains.empty()) return bw;
    const auto t0 = std::chrono::steady_clock::now();
    const int sampler = res.Sampler("solver");
    const uint32_t mapped0 = res.MappedCount();
    // The turn fills the frame's upload-ring slot, which a frame still in flight may be copying
    // from (an activation inside the loop); boot has nothing in flight and this costs nothing.
    gpu.WaitIdle();
    // The manager's claim is read before the kernel's (it is cheap); the trace is the answer.
    auto claimed = [&] {
        uint64_t hist[16];
        const uint64_t n = ClaimedMips(res, hgtTenant, hist, only);
        return n > 0 && hist[0] == n;
    };
    SweSolver::BedTrace trace;
    for (;;) {
        // Asked again every turn, as the pin asks every frame: the ring gate admits a level only
        // under a mapped parent, so one ask brings one ring.
        for (const Domain& d : domains) {
            res.Want(sampler, hgtTenant, m_hgtSlice, 0u, d.u0, d.v0, d.u1, d.v1);
        }
        {
            hal::CommandContext up(gpu, gpu.BeginUpload(), hal::Owner::Upload);
            res.ProcessQueues(gpu, up.Native());
            gpu.EndUpload();
        }
        ++bw.turns;
        bw.seconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
        if (claimed()) {
            bw.cells = bw.whole = 0;
            for (const Domain& d : domains) {
                if (!d.solver->TraceBed(gpu, trace)) continue;
                const uint32_t page = d.solver->PageSlice();
                for (size_t i = 0; i < trace.bed.size(); ++i) {
                    ++bw.cells;
                    if (trace.mip[i] == 0 && trace.slice[i] == page) ++bw.whole;
                }
            }
            if (bw.cells > 0 && bw.whole == bw.cells) {
                bw.done = true;
                break;
            }
        }
        if (bw.seconds > kBedWaitMaxS) break;
        // The loads run on the Io lane; a turn that finds nothing landed has nothing to map.
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    bw.tiles = res.MappedCount() - mapped0;
    return bw;
}

void WeatherManager::Update(Gpu& gpu, ShaderCompiler& sc, const std::wstring& shaderDir,
                            double simUnix, double camLatDeg, double camLonDeg,
                            double camAltM) {
    int active = 0;
    for (Window& w : m_windows) {
        if (!w.active && w.bathy && w.bathy->Ready() && camAltM < kActivateAltM) {
            // Residency rises on zoom: the camera entering a dormant window's footprint IS
            // the demand signal, exactly as a CDLOD node entering the frustum demands tiles.
            double x, z;
            WorldOf(camLatDeg, camLonDeg, x, z);
            if (x > w.bathy->WorldX0() && x < w.bathy->WorldX0() + w.bathy->WorldSizeX() &&
                z > w.bathy->WorldZ0() && z < w.bathy->WorldZ0() + w.bathy->WorldSizeZ()) {
                Activate(gpu, sc, shaderDir, w.name.c_str(), simUnix);
            }
        }
        if (w.active && w.owned) {
            // Owned windows advance on their own submits; at real time this is one 64-substep
            // batch every ~13 sim-seconds -- background noise.
            auto zero = [](double) { return 0.0; };
            w.solver->AdvanceTo(gpu, simUnix, w.oceanAt, zero, zero, zero);
        }
        // The mirror used to refresh here every kMirrorDt (two readbacks, two WaitIdle
        // drains, 8.7 M half->float: 17.9-27.2 ms on every 61st frame of the storm rail)
        // for a Query nobody in the loop calls. RefreshMirrorsTo, on demand, replaced it.
        if (w.active) ++active;
    }
    char s[96];
    snprintf(s, sizeof(s), "wx %d/%d windows", active, static_cast<int>(m_windows.size()));
    stats = s;
}

int WeatherManager::ActiveWindows() const {
    int n = 0;
    for (const Window& w : m_windows) n += w.active ? 1 : 0;
    return n;
}

const WeatherManager::Window* WeatherManager::WindowAt(double latDeg, double lonDeg) const {
    double x, z;
    WorldOf(latDeg, lonDeg, x, z);
    for (const Window& w : m_windows) {
        if (!w.active || !w.bathy) continue;
        if (x > w.bathy->WorldX0() && x < w.bathy->WorldX0() + w.bathy->WorldSizeX() &&
            z > w.bathy->WorldZ0() && z < w.bathy->WorldZ0() + w.bathy->WorldSizeZ()) {
            return &w;
        }
    }
    return nullptr;
}

WeatherSample WeatherManager::Query(double latDeg, double lonDeg, double unixT,
                                    double groundResM, bool refineBySolver) const {
    WeatherSample s;
    const double latRad = latDeg * kD2R, lonRad = lonDeg * kD2R;

    // ---- the bed: the one composed stack, provenance = the topmost covering source.
    if (m_comp && m_hgtCh >= 0) {
        s.bedNavd = m_comp->SampleHeightStack(m_hgtCh, latRad, lonRad, groundResM);
        s.bedSrc = "earth.height base";
        const auto& ch = m_comp->ChannelAt(m_hgtCh);
        for (const auto* src : ch.height) {
            const SourceInfo& si = src->Info();
            if (lonDeg >= si.lon0 && lonDeg <= si.lon1 && latDeg >= si.lat0 &&
                latDeg <= si.lat1 && si.lon1 - si.lon0 < 359.0) {
                s.bedSrc = si.name.c_str();   // topmost regional source wins the label
            }
        }
    }

    // ---- the level: the tide atlas everywhere; inside a solver's domain the solver's own surface
    // (SolverRefine, at the end: the solver is truth).
    if (m_atlas && m_atlas->Ready()) {
        s.levelNavd = m_atlas->MslNavd(latDeg, lonDeg) +
                      m_atlas->Level(latDeg, lonDeg, unixT, groundResM);
        s.levelSrc = "water.tide atlas";
    }

    // ---- currents: the GoMOFS Gulf field; the solver's solved current replaces it where the
    // solver has one (SolverRefine).
    if (m_currents && m_currents->Field().Valid()) {
        float u = 0, v = 0;
        if (m_currents->Field().Sample(lonDeg, latDeg, u, v)) {
            s.u = u;
            s.v = v;
            s.currentSrc = "gomofs.surface (700 m)";
        }
    }

    // ---- waves: the global Hs grid carries the world; the Gulf point forecast adds
    // period + direction inside its box (a POINT product -- declared, not hidden).
    if (m_globe && m_globe->WavesNx() > 0) {
        const auto& hs = m_globe->Hs();
        const int nx = m_globe->WavesNx(), ny = m_globe->WavesNy();
        double lonS = lonDeg;
        if (m_globe->WavesLon1() > 180.0 && lonS < 0.0) lonS += 360.0;
        const double fx =
            (lonS - (m_globe->WavesLon1() - nx * m_globe->WavesDLon())) / m_globe->WavesDLon();
        const double fy = (m_globe->WavesLat1() - latDeg) / m_globe->WavesDLat();
        const int ix = static_cast<int>(fx), iy = static_cast<int>(fy);
        if (ix >= 0 && iy >= 0 && ix < nx && iy < ny) {
            const float v = hs[static_cast<size_t>(iy) * nx + ix];
            if (v >= 0.0f) {
                s.hs = v;
                s.waveSrc = "gfswave.global 0p25 (Hs)";
            }
        }
    }
    if (m_sea && m_sea->Ready() && latDeg > 41.5 && latDeg < 44.5 && lonDeg > -71.2 &&
        lonDeg < -68.5) {
        const int hi = m_sea->HourIndex(unixT);
        const SeaHour& h = m_sea->Hour(hi);
        if (!h.parts.empty()) {
            s.tp = static_cast<float>(h.combinedTp);
            s.dirDeg = static_cast<float>(h.combinedFromDeg);
            if (s.hs <= 0.0f) s.hs = static_cast<float>(h.combinedHs);
            s.waveSrc = "gfswave gulf point (Hs/Tp/dir)";
        }
    }

    // ---- wind: the global GFS 10 m vector grid.
    if (m_globe && m_globe->WindNx() > 0) {
        const int nx = m_globe->WindNx(), ny = m_globe->WindNy();
        double lonS = lonDeg;
        if (m_globe->WindLon1() > 180.0 && lonS < 0.0) lonS += 360.0;
        const double fx =
            (lonS - (m_globe->WindLon1() - nx * m_globe->WindDLon())) / m_globe->WindDLon();
        const double fy = (m_globe->WindLat1() - latDeg) / m_globe->WindDLat();
        const int ix = static_cast<int>(fx), iy = static_cast<int>(fy);
        if (ix >= 0 && iy >= 0 && ix < nx && iy < ny) {
            s.windU = m_globe->WindU()[static_cast<size_t>(iy) * nx + ix];
            s.windV = m_globe->WindV()[static_cast<size_t>(iy) * nx + ix];
            s.windSrc = "gfs.wind 0p5 (10 m)";
        }
    }

    s.depthM = static_cast<float>(s.levelNavd - s.bedNavd);
    if (refineBySolver) SolverRefine(latDeg, lonDeg, unixT, s);
    return s;
}

}  // namespace ga
