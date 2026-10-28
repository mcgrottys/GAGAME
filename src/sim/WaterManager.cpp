#include "sim/WaterManager.h"

#include "compose/SurfaceFrame.h"

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
}  // namespace

void WaterManager::Init(Compositor* comp, int heightChannel, const WaterAtlas* atlas,
                        const TideModel* tides, const GlobeModel* globe,
                        const SeaState* sea, const CurrentModel* currents) {
    m_comp = comp;
    m_hgtCh = heightChannel;
    m_atlas = atlas;
    m_tides = tides;
    m_globe = globe;
    m_sea = sea;
    m_currents = currents;
    Register();
}

void WaterManager::AddExternalWindow(const char* name, SweSolver* solver,
                                     std::function<double(double)> oceanAt) {
    Window w;
    w.name = name;
    w.solver = solver;
    if (solver && solver->Ready()) {
        const SweDomain& d = solver->Domain();
        w.box[0] = d.lat0;
        w.box[1] = d.lon0;
        w.box[2] = d.lat1;
        w.box[3] = d.lon1;
        w.levels = solver->LevelBoxes();   // stage 2b/2c: the levels' beds are wanted with the window's
    }
    w.oceanAt = std::move(oceanAt);
    w.active = solver && solver->Ready();
    snprintf(w.levelTag, sizeof(w.levelTag), "swe.%s + stations", name);
    snprintf(w.currentTag, sizeof(w.currentTag), "swe.%s (solved)", name);
    m_windows.push_back(std::move(w));
    Register();
}

void WaterManager::RefreshMirrorsTo(Gpu& gpu, double simUnix) {
    ReadMirrors(gpu, simUnix, kMirrorDt);
}

void WaterManager::RequestRegion(double latDeg, double lonDeg, double radiusM) {
    for (Window& w : m_windows) {
        if (!w.active || !w.solver || !w.solver->Ready()) continue;
        // A region may reach a window whose box its centre is outside of: the solver's own chart
        // says which of its cells the region touches, and none is nothing.
        w.solver->RequestRegion(latDeg, lonDeg, radiusM);
    }
}

double WaterManager::SolverAsOf() const {
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

void WaterManager::SolverRefine(double latDeg, double lonDeg, double unixT,
                                WeatherSample& s) const {
    const Window* w = WindowAt(latDeg, lonDeg);
    if (!w || !w->solver || !w->solver->Ready()) return;
    const double wDom = w->solver->DomainWeight(latDeg, lonDeg);
    if (!(wDom > 0.0)) return;
    SweSolver::Deviation d;
    const bool have =
        w->solver->DeviationAt(latDeg, lonDeg, d) ||
        w->solver->DeviationFromFields(latDeg, lonDeg, w->eta, w->etaW, w->uv4, w->uvW, w->mirrorT, d);
    if (!have) {
        // THE SOLVER OWNS THIS SURFACE AND HAS NOT ANSWERED HERE. The atlas is not its answer.
        // Said once a run, with the point and the mirror's shape.
        static bool said = false;
        if (!said) {
            said = true;
            double tx = 0.0, ty = 0.0;
            w->solver->Domain().CellOf(latDeg, lonDeg, tx, ty);
            Log("[weather] the solver has not answered at %.5f N %.5f E (cell %.1f, %.1f of %u x %u): regions %s, "
                "the mirror %zu eta / %zu uv floats, rows %u / %u, read at t=%.0f",
                latDeg, lonDeg, tx, ty, w->solver->Nx(), w->solver->Ny(),
                w->solver->DeviationAt(latDeg, lonDeg, d) ? "answered" : "silent", w->eta.size(), w->uv4.size(),
                w->etaW, w->uvW, w->mirrorT);
        }
        s.levelSrc = "-";
        s.depthM = static_cast<float>(s.levelNavd - s.bedNavd);
        return;
    }
    const double atlas = s.levelNavd;
    // THE DRY FLOOR'S SIGNATURE, said once (stage 2 of one water on the tree, 2026-10-26): an answer of exactly
    // the dry floor (-0.65: a zero surface and no water in the whole stencil) -- which path answered, the cell,
    // and the mirror's own values at its corner. A reader beyond the fine set's reach gets this where the twin
    // holds the water at mip 1: the mirror is mip 0's (the readers do not read mip 1 yet). The stage-2 dump
    // read it over most of its grid, which stands about the origin lattice's node, 2 km east and 3 km north
    // of the eye the fine set is about.
    static bool saidFloor = false;
    if (!saidFloor && std::fabs(double(d.dEta) + 0.65) < 0.02 && d.wet <= 0.0f) {
        saidFloor = true;
        double tx = 0.0, ty = 0.0;
        w->solver->Domain().CellOf(latDeg, lonDeg, tx, ty);
        SweSolver::Deviation r;
        const bool byRegion = w->solver->DeviationAt(latDeg, lonDeg, r);
        const int ix = int(std::floor(tx - 0.5)), iy = int(std::floor(ty - 0.5));
        float e = 0.0f, h = 0.0f;
        if (ix >= 0 && iy >= 0 && uint32_t(ix) < w->etaW && uint32_t(iy) < w->etaH) {
            e = w->eta[size_t(iy) * w->etaW + size_t(ix)];
            h = w->uv4[(size_t(iy) * w->uvW + size_t(ix)) * 4 + 2];
        }
        Log("[weather] the dry floor answered at %.5f N %.5f E (cell %.1f, %.1f): by %s (regions held %zu, as of %.0f); "
            "the mirror at (%d, %d): eta %+.3f, h %.3f; dEta %+.3f wet %.2f, the plane %+.3f, the atlas %+.3f",
            latDeg, lonDeg, tx, ty, byRegion ? "a REGION" : "the mirror", w->solver->RegionCount(),
            w->solver->DeliveredAsOf(), ix, iy, e, h, d.dEta, d.wet, w->oceanAt(unixT), atlas);
    }
    // the wet columns' surface; where none holds water, under the solver's bed (SweSolver Reconstruct)
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

void WaterManager::ReadMirrors(Gpu& gpu, double simUnix, double maxAge) {
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

int WaterManager::DomainRects(const Window& w, float out[4][4]) const {
    return SurfaceFrame::SliceRectsLL(m_stand, w.box[0], w.box[1], w.box[2], w.box[3], out);
}

int WaterManager::WantDomain(ResidencyManager& res, int sampler, int hgtTenant, const Window& w,
                             float win[4][4], float cube[6][4], uint32_t& faces) const {
    // PHASE B2: the standing window, in its own uv (modulo the page: one, two or four rectangles)...
    const int n = DomainRects(w, win);
    for (int k = 0; k < n; ++k) {
        res.Want(sampler, hgtTenant, m_hgtSlice, 0u, win[k][0], win[k][1], win[k][2], win[k][3]);
    }
    // ...and the cube's faces under the domain, where the rule reads when the window holds nothing:
    // the height tenant's slices 0..5 are the cube's faces (Assembly: "paint cube faces").
    faces = SurfaceFrame::CubeRectsLL(w.box[0], w.box[1], w.box[2], w.box[3], cube);
    for (uint32_t f = 0; f < 6u; ++f) {
        if (!((faces >> f) & 1u)) continue;
        res.Want(sampler, hgtTenant, f, 0u, cube[f][0], cube[f][1], cube[f][2], cube[f][3]);
    }
    // STAGE 2b: ...and the cube under the tail at the tail's own mip (its cells' grain): the tail's kernel reads the
    // cube where the window holds nothing, which is everywhere but the window.
    for (const SweSolver::LevelBox& lb : w.levels) {
        if (!(lb.box[2] > lb.box[0])) continue;
        float ct[6][4];
        const uint32_t ft = SurfaceFrame::CubeRectsLL(lb.box[0], lb.box[1], lb.box[2], lb.box[3], ct);
        for (uint32_t f = 0; f < 6u; ++f) {
            if (!((ft >> f) & 1u)) continue;
            res.Want(sampler, hgtTenant, f, lb.mip, ct[f][0], ct[f][1], ct[f][2], ct[f][3]);
        }
    }
    return n;
}

void WaterManager::PinDomains(ResidencyManager& res, int hgtTenant) {
    if (hgtTenant < 0 || !m_hgtArr) return;
    std::string pinned;
    for (const Window& w : m_windows) {
        if (!w.active || !(w.box[2] > w.box[0])) continue;
        float r[4][4], c[6][4];
        uint32_t faces = 0;
        // M13: the solver's domain is its own reader of the shared cache -- it pins the bed
        // under its lattice whether or not any view is looking there.
        // Step 5: a standing reader, the order's first class (HIERARCHY 4.19).
        const int n = WantDomain(res, res.Sampler("solver", true), hgtTenant, w, r, c, faces);
        if (!m_pinLogged) {
            char line[200];
            snprintf(line, sizeof(line), " %s in %d rectangle(s)%s", w.name.c_str(), n,
                     n ? "" : " -- OUTSIDE the standing window, unpinned");
            pinned += line;
            for (int k = 0; k < n; ++k) {
                snprintf(line, sizeof(line), " uv %.4f..%.4f x %.4f..%.4f", r[k][0], r[k][2], r[k][1], r[k][3]);
                pinned += line;
            }
            for (uint32_t f = 0; f < 6u; ++f) {
                if (!((faces >> f) & 1u)) continue;
                snprintf(line, sizeof(line), "; the cube's face %u uv %.4f..%.4f x %.4f..%.4f", f, c[f][0], c[f][2],
                         c[f][1], c[f][3]);
                pinned += line;
            }
        }
    }
    if (!m_pinLogged && !pinned.empty()) {
        m_pinLogged = true;
        Log("[weather] solver domains pinned on the standing window, slice %u mip 0, and on the cube under them:%s",
            m_hgtSlice, pinned.c_str());
    }
}

uint64_t WaterManager::ClaimedMips(const ResidencyManager& res, int hgtTenant,
                                   uint64_t hist[16], const char* only) const {
    for (int m = 0; m < 16; ++m) hist[m] = 0;
    if (hgtTenant < 0 || !m_hgtArr) return 0;
    uint64_t n = 0;
    constexpr double kD2R = 3.14159265358979323846 / 180.0;
    const FaceWindow fw{m_stand.face, m_stand.rung, 0, 0};
    const double dim = Lattice::kFaceDim;
    for (const Window& w : m_windows) {
        if (!w.active || !(w.box[2] > w.box[0])) continue;
        if (only && w.name != only) continue;
        // A sample every few cells over the domain, its slice uv modulo the page (PHASE B2): the
        // map is at tile grain, so this is a few thousand reads of the manager's own bytes.
        const int nu = 256, nv = 256;
        for (int j = 0; j <= nv; ++j) {
            const double lat = (w.box[2] - (w.box[2] - w.box[0]) * double(j) / nv) * kD2R;
            for (int i = 0; i <= nu; ++i) {
                const double lon = (w.box[1] + (w.box[3] - w.box[1]) * double(i) / nu) * kD2R;
                const double d[3] = {std::cos(lat) * std::cos(lon), std::sin(lat), std::cos(lat) * std::sin(lon)};
                double X = 0.0, Y = 0.0;
                fw.TexelOf(d, X, Y);
                if (X < double(m_stand.OrgX()) || Y < double(m_stand.OrgY()) ||
                    X >= double(m_stand.OrgX()) + dim || Y >= double(m_stand.OrgY()) + dim) {
                    ++hist[15];
                    ++n;
                    continue;
                }
                const float u = float(std::fmod(X, dim) / dim), v = float(std::fmod(Y, dim) / dim);
                ++hist[(std::min)(res.ResidentMipAt(hgtTenant, m_hgtSlice, u, v), 15u)];
                ++n;
            }
        }
    }
    return n;
}

WaterManager::BedWait WaterManager::WaitForBeds(Gpu& gpu, ResidencyManager& res,
                                                  int hgtTenant, const char* only) {
    BedWait bw;
    if (hgtTenant < 0 || !m_hgtArr) return bw;
    std::vector<const Window*> domains;
    for (const Window& w : m_windows) {
        if (!w.active || !w.solver || !w.solver->Ready()) continue;
        if (only && w.name != only) continue;
        float r[4][4];
        if (DomainRects(w, r)) domains.push_back(&w);
    }
    bw.windows = static_cast<uint32_t>(domains.size());
    if (domains.empty()) return bw;
    const auto t0 = std::chrono::steady_clock::now();
    const int sampler = res.Sampler("solver");
    const uint32_t mapped0 = res.MappedCount();
    // The turn fills the frame's upload-ring slot, which a frame still in flight may be copying
    // from (an activation inside the loop); boot has nothing in flight and this costs nothing.
    gpu.WaitIdle();
    for (;;) {
        // Asked again every turn, as the pin asks every frame: the ring gate admits a level only
        // under a mapped parent, so one ask brings one ring.
        for (const Window* w : domains) {
            float r[4][4], c[6][4];
            uint32_t faces = 0;
            WantDomain(res, sampler, hgtTenant, *w, r, c, faces);
        }
        {
            hal::CommandContext up(gpu, gpu.BeginUpload(), hal::Owner::Upload);
            res.ProcessQueues(gpu, up.Native());
            gpu.EndUpload();
        }
        ++bw.turns;
        bw.seconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
        // THE END: the order has answered every tile the ask implies (the first turn states the ask;
        // the pass that orders it runs in that turn, so from the second the count is the order's).
        bw.pending = res.PendingOf(sampler, hgtTenant);
        if (bw.turns >= 2 && bw.pending == 0) {
            bw.done = true;
            break;
        }
        if (bw.seconds > kBedWaitMaxS) break;
        // The loads run on the Io lane; a turn that finds nothing landed has nothing to map.
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    bw.tiles = res.MappedCount() - mapped0;
    {   // What the kernel read, by slice kind and mip: the page (the standing window) or the cube.
        SweSolver::BedTrace trace;
        uint64_t page[16] = {}, cube[16] = {};
        bw.cells = bw.whole = 0;
        for (const Window* w : domains) {
            if (!w->solver->TraceBed(gpu, trace)) continue;
            const uint32_t slice = w->solver->PageSlice();
            for (size_t i = 0; i < trace.bed.size(); ++i) {
                ++bw.cells;
                const uint32_t m = (std::min)(static_cast<uint32_t>(trace.mip[i]), 15u);
                if (trace.slice[i] == slice) {
                    ++page[m];
                    if (m == 0u) ++bw.whole;
                } else {
                    ++cube[m];
                }
            }
        }
        char line[64];
        for (int m = 0; m < 16; ++m) {
            if (!page[m]) continue;
            snprintf(line, sizeof(line), " page m%d:%llu", m, static_cast<unsigned long long>(page[m]));
            bw.read += line;
        }
        for (int m = 0; m < 16; ++m) {
            if (!cube[m]) continue;
            snprintf(line, sizeof(line), " cube m%d:%llu", m, static_cast<unsigned long long>(cube[m]));
            bw.read += line;
        }
        if (bw.read.empty()) bw.read = " nothing traced";
    }
    {   // what the manager's map claims over the domain at the wait's end, by mip (15 = outside the page)
        uint64_t hist[16];
        const uint64_t n = ClaimedMips(res, hgtTenant, hist, only);
        std::string h;
        for (int m = 0; m < 16; ++m) {
            if (!hist[m]) continue;
            h += " m" + std::to_string(m) + ":" + std::to_string(hist[m]);
        }
        Log("[weather] the bed wait's claim over %llu samples:%s (in flight at the end: the manager's own count)",
            static_cast<unsigned long long>(n), h.empty() ? " none" : h.c_str());
    }
    return bw;
}

void WaterManager::Update(double simUnix, double camLatDeg, double camLonDeg, double camAltM) {
    (void)simUnix;
    int active = 0;
    for (Window& w : m_windows) {
        if (!w.active) continue;
        ++active;
        // THE EYE LEFT THE WINDOW (stage 2): beyond it the tide plane carries the eye, as it did beyond the
        // box; said once. Stage 2b re-stands the window about the eye here.
        if (!w.beyondSaid && w.solver && w.solver->Ready() && camAltM < 30000.0 &&
            !w.solver->Domain().Holds(camLatDeg, camLonDeg)) {
            w.beyondSaid = true;
            Log("[weather] the eye at %.5f N %.5f E (%.0f m up) is beyond the %s window: the tide plane carries "
                "it there (stage 2b of one water on the tree re-stands the window about the eye)",
                camLatDeg, camLonDeg, camAltM, w.name.c_str());
        }
    }
    char s[96];
    snprintf(s, sizeof(s), "wx %d/%d windows", active, static_cast<int>(m_windows.size()));
    stats = s;
}

int WaterManager::ActiveWindows() const {
    int n = 0;
    for (const Window& w : m_windows) n += w.active ? 1 : 0;
    return n;
}

const WaterManager::Window* WaterManager::WindowAt(double latDeg, double lonDeg) const {
    for (const Window& w : m_windows) {
        if (!w.active) continue;
        if (w.solver && w.solver->Ready() ? w.solver->Domain().Holds(latDeg, lonDeg)
                                          : w.InBox(latDeg, lonDeg)) {
            return &w;
        }
    }
    return nullptr;
}

// ---- THE REGISTRY -------------------------------------------------------------------------------
namespace {
// The nearest node of a regular lat/lon grid (rows from lat1 down, columns ending at lon1); false
// outside it. A grid whose longitudes run 0..360 takes a western longitude modulo 360.
bool NearestNode(double latDeg, double lonDeg, int nx, int ny, double lon1, double dLon, double lat1,
                 double dLat, size_t& at) {
    if (lon1 > 180.0 && lonDeg < 0.0) lonDeg += 360.0;
    const int ix = static_cast<int>((lonDeg - (lon1 - nx * dLon)) / dLon);
    const int iy = static_cast<int>((lat1 - latDeg) / dLat);
    if (ix < 0 || iy < 0 || ix >= nx || iy >= ny) return false;
    at = static_cast<size_t>(iy) * nx + ix;
    return true;
}
}  // namespace

namespace water {
const char* Name(Quantity q) {
    static const char* k[] = {"level", "current", "bed", "sea", "wind", "memory"};
    return k[static_cast<int>(q)];
}
const char* Name(Support s) {
    static const char* k[] = {"point", "grid", "tree", "window", "analytic"};
    return k[static_cast<int>(s)];
}
const char* Name(TimeForm f) {
    static const char* k[] = {"static", "harmonic", "packet", "series", "stochastic", "transient"};
    return k[static_cast<int>(f)];
}
}  // namespace water

// The producers this manager answers for, in the order their law composes them (OVER: the later on
// top), then the solver windows (SolverRefine REPLACES), then the declared.
void WaterManager::Register() {
    using Q = water::Quantity;
    using Su = water::Support;
    using T = water::TimeForm;
    m_producers.clear();
    const auto add = [&](const char* name, Q q, Su su, T f, double grainM,
                         const char* reader, decltype(water::Producer::at) at) {
        water::Producer p;
        p.name = name;
        p.q = q;
        p.support = su;
        p.form = f;
        p.grainM = grainM;
        p.reader = reader;
        p.at = std::move(at);
        m_producers.push_back(std::move(p));
    };
    // The bed: the one composed stack at the finest level held; provenance the topmost regional source.
    if (m_comp && m_hgtCh >= 0) {
        add("water.bed (the height stack)", Q::Bed, Su::Tree, T::Static, 3.43,
            "Query; the bank, the solvers, the hull (TreeWater::BedAt)",
            [this](double la, double lo, double, double g, water::Sample& x) {
                // RESIDENT: a hull asks this on the frame thread every step (TreeWater::SlowAt, at 1 m); the
                // tree's blocking read painted each cold tile there -- 2.8 s of the frame's waits over four
                // travels (2026-10-15). The finest level held answers now; the step after a landing reads
                // the finer one.
                x.v[0] = m_comp->SampleHeightResident(m_hgtCh, la * kD2R, lo * kD2R, g);
                x.has = 1;
                x.src = "earth.height base";
                for (const auto* src : m_comp->ChannelAt(m_hgtCh).height) {
                    const SourceInfo& si = src->Info();
                    if (lo >= si.lon0 && lo <= si.lon1 && la >= si.lat0 && la <= si.lat1 &&
                        si.lon1 - si.lon0 < 359.0) {
                        x.src = si.name.c_str();   // the topmost regional source wins the label
                    }
                }
                return true;
            });
    }
    // The level: the tide atlas everywhere (inside a solver's domain the solver's own surface).
    if (m_atlas) {
        add("water.tide atlas", Q::Level, Su::Grid, T::Harmonic, 0.0,
            "Query; the bank's corners, the solve's level",
            [this](double la, double lo, double t, double g, water::Sample& x) {
                if (!m_atlas->Ready()) return false;
                x.v[0] = m_atlas->MslNavd(la, lo) + m_atlas->Level(la, lo, t, g);
                x.has = 1;
                x.src = "water.tide atlas";
                return true;
            });
    }
    // The current: the scene's regional model fields (the solver's solved current replaces it).
    if (m_currents) {
        add("current.field (surface)", Q::Current, Su::Grid, T::Static, 0.0,
            "Query; the Gulf map",
            [this](double la, double lo, double, double, water::Sample& x) {
                float u = 0, v = 0;
                if (!m_currents->FieldCount() || !m_currents->SampleAt(lo, la, u, v)) return false;
                x.v[0] = u;
                x.v[1] = v;
                x.has = 3;
                x.src = "current.field (surface)";
                return true;
            });
    }
    // The sea's summary: the global Hs grid carries the world; the sea-state sources (each file at its
    // own place and clock, the later listed on top: WaveScale's order) add period and direction inside
    // their boxes, and Hs where the grid has none -- point products, declared, not hidden.
    if (m_globe) {
        add("gfswave.global 0p25 (Hs)", Q::Sea, Su::Grid, T::Series, 27750.0,
            "Query (tools); WaveScale; the far sea (Globe.hlsl FarSeaScale)",
            [this](double la, double lo, double, double, water::Sample& x) {
                size_t i = 0;
                if (m_globe->WavesNx() <= 0 ||
                    !NearestNode(la, lo, m_globe->WavesNx(), m_globe->WavesNy(), m_globe->WavesLon1(),
                                 m_globe->WavesDLon(), m_globe->WavesLat1(), m_globe->WavesDLat(), i)) {
                    return false;
                }
                const float v = m_globe->Hs()[i];
                if (!(v >= 0.0f)) return false;
                x.v[0] = v;
                x.has = 1;
                x.src = "gfswave.global 0p25 (Hs)";
                return true;
            });
    }
    for (const SeaState* src : m_seaSources) {
        if (!src) continue;
        add(src->Label().c_str(), Q::Sea, Su::Point, T::Series, 0.0,
            "Query (tools); the eye's spectrum (SeaLayer::PartsAt); WaveScale",
            [src](double la, double lo, double t, double, water::Sample& x) {
                if (!src->Ready() || !src->HasBox()) return false;
                double b[4];
                src->Box(b);
                if (la < b[0] || la > b[2] || lo < b[1] || lo > b[3]) return false;
                const SeaHour& h = src->Hour(src->HourIndex(t));
                if (h.parts.empty()) return false;
                x.v[0] = h.combinedHs;
                x.v[1] = h.combinedTp;
                x.v[2] = h.combinedFromDeg;
                x.has = 7;
                x.fills = 1;
                x.src = src->Label().c_str();
                return true;
            });
    }
    // The wind: the global GFS 10 m vector grid.
    if (m_globe) {
        add("gfs.wind 0p5 (10 m)", Q::Wind, Su::Grid, T::Static, 55500.0,
            "Query; the pixel's Cox-Munk slopes",
            [this](double la, double lo, double, double, water::Sample& x) {
                size_t i = 0;
                if (m_globe->WindNx() <= 0 ||
                    !NearestNode(la, lo, m_globe->WindNx(), m_globe->WindNy(), m_globe->WindLon1(),
                                 m_globe->WindDLon(), m_globe->WindLat1(), m_globe->WindDLat(), i)) {
                    return false;
                }
                x.v[0] = m_globe->WindU()[i];
                x.v[1] = m_globe->WindV()[i];
                x.has = 3;
                x.src = "gfs.wind 0p5 (10 m)";
                return true;
            });
    }
    // The solver windows: transient state, refining the level and the current where they are resident.
    for (const Window& w : m_windows) {
        add(w.levelTag, Q::Level, Su::Window, T::Transient, 0.0,
            "SolverRefine; the bank's level", nullptr);
        add(w.currentTag, Q::Current, Su::Window, T::Transient, 0.0,
            "SolverRefine; the bank, the wave solve", nullptr);
    }
    for (const water::Producer& p : m_declared) m_producers.push_back(p);
}

void WaterManager::LogProducers() const {
    Log("[water] %zu producers (Water V2, sim/WaterProducer.h): quantity, support, time, grain, name -- who "
        "reads it",
        m_producers.size());
    for (const water::Producer& p : m_producers) {
        char g[24] = "-";
        if (p.grainM > 0.0) snprintf(g, sizeof(g), "%.4g m", p.grainM);
        Log("[water]   %-8s %-9s %-10s %-9s %s -- %s%s", water::Name(p.q), water::Name(p.support),
            water::Name(p.form), g, p.name.c_str(), p.reader.c_str(), p.at ? "" : " (declared)");
    }
}

// THE QUERY, ONE LOOP: each producer that answers here, composed by its quantity's law (SUM: the
// first says, the rest add; OVER and the sea's summary: the later on top, a `fills` component only
// where nothing positive is composed yet); a quantity's provenance is the last producer that said
// any of it. Then the depth, and the solver windows refine (SolverRefine: the solver is truth).
WeatherSample WaterManager::Query(double latDeg, double lonDeg, double unixT,
                                  double groundResM, bool refineBySolver) const {
    struct Acc {
        double v[3] = {0.0, 0.0, 0.0};
        uint8_t has = 0;
        const char* src = "-";
    } acc[water::kQuantities];
    for (const water::Producer& p : m_producers) {
        water::Sample x;
        if (!p.at || !p.at(latDeg, lonDeg, unixT, groundResM, x)) continue;
        Acc& a = acc[static_cast<int>(p.q)];
        const bool sum = water::CombineOf(p.q) == water::Combine::Sum;
        bool said = false;
        for (int i = 0; i < 3; ++i) {
            const uint8_t bit = static_cast<uint8_t>(1u << i);
            if (!(x.has & bit)) continue;
            if ((x.fills & bit) && (a.has & bit) && a.v[i] > 0.0) continue;
            a.v[i] = (sum && (a.has & bit)) ? a.v[i] + x.v[i] : x.v[i];
            a.has |= bit;
            said = true;
        }
        if (said) a.src = x.src;
    }
    using Q = water::Quantity;
    const Acc& lv = acc[int(Q::Level)];
    const Acc& cu = acc[int(Q::Current)];
    const Acc& bd = acc[int(Q::Bed)];
    const Acc& se = acc[int(Q::Sea)];
    const Acc& wi = acc[int(Q::Wind)];
    WeatherSample s;
    if (bd.has) {
        s.bedNavd = static_cast<float>(bd.v[0]);
        s.bedSrc = bd.src;
    }
    if (lv.has) {
        s.levelNavd = lv.v[0];
        s.levelSrc = lv.src;
    }
    if (cu.has) {
        s.u = static_cast<float>(cu.v[0]);
        s.v = static_cast<float>(cu.v[1]);
        s.currentSrc = cu.src;
    }
    if (se.has) {
        s.hs = static_cast<float>(se.v[0]);
        s.tp = static_cast<float>(se.v[1]);
        s.dirDeg = static_cast<float>(se.v[2]);
        s.waveSrc = se.src;
    }
    if (wi.has) {
        s.windU = static_cast<float>(wi.v[0]);
        s.windV = static_cast<float>(wi.v[1]);
        s.windSrc = wi.src;
    }
    s.depthM = static_cast<float>(s.levelNavd - s.bedNavd);
    if (refineBySolver) SolverRefine(latDeg, lonDeg, unixT, s);
    return s;
}

}  // namespace ga
