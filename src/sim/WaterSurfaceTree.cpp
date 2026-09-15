// ================================================================================================
//  TreeWater.cpp - the assembly. See WaterSurfaceTree.h for why each term comes from where.
// ================================================================================================
#include "sim/WaterSurfaceTree.h"

#include "core/Common.h"
#include "sim/BathyModel.h"
#include "sim/WaterTerms.h"
#include "sim/WaveScale.h"

#include <algorithm>
#include <cmath>

namespace ga {

namespace {

// The flat world frame is the ACT0816 tangent plane, and WeatherManager::WorldOf is its forward
// map. This is its exact inverse -- same four constants, so the round trip is exact by
// construction rather than by agreement.
void LatLonOf(double wx, double wz, double& latDeg, double& lonDeg) {
    lonDeg = BathyModel::kOrgLon + wx / BathyModel::kMPerLon;
    latDeg = BathyModel::kOrgLat + wz / BathyModel::kMPerLat;
}

// HLSL's smoothstep, so the window feather is the same curve on both processors.
double SmoothStep(double e0, double e1, double x) {
    if (e1 <= e0) return (x < e0) ? 0.0 : 1.0;
    const double t = (std::min)((std::max)((x - e0) / (e1 - e0), 0.0), 1.0);
    return t * t * (3.0 - 2.0 * t);
}

}  // namespace

void TreeWater::Configure(const WeatherManager* wx, const WaveField* wave, const OceanCpu* ocean,
                          const SeaState* sea, double heightScale, double waveExag,
                          double waveChop) {
    m_wx = wx;
    m_wave = wave;
    m_ocean = ocean;
    m_sea = sea;
    m_heightScale = (heightScale > 0.0) ? heightScale : 1.0;
    m_waveExag = (waveExag > 0.0) ? waveExag : 1.0;
    m_waveChop = (waveChop > 0.0) ? waveChop : 1.0;
}

void TreeWater::SetChart(const Space::Anchor* chart) {
    m_hasChart = chart != nullptr;
    if (chart) m_chart = *chart;
    m_memoX = 1e30;   // the memo belongs to the chart it was asked in
    m_memoZ = 1e30;
    m_memoT = -1e30;
}

void TreeWater::PlaceOf(double wx, double wz, double& latDeg, double& lonDeg) const {
    if (m_hasChart) {
        m_chart.LatLonOf(wx, wz, latDeg, lonDeg);
    } else {
        LatLonOf(wx, wz, latDeg, lonDeg);
    }
}

void TreeWater::RootOf(double wx, double wz, double& rx, double& rz) const {
    if (!m_hasChart) {
        rx = wx;
        rz = wz;
        return;
    }
    double la = 0.0, lo = 0.0;
    m_chart.LatLonOf(wx, wz, la, lo);
    rx = (lo - BathyModel::kOrgLon) * BathyModel::kMPerLon;
    rz = (la - BathyModel::kOrgLat) * BathyModel::kMPerLat;
}

void TreeWater::SetBoats(const WakeBoat* boats, int count) {
    m_boatCount = (std::min)((std::max)(count, 0), 8);
    for (int i = 0; i < m_boatCount; ++i) m_boats[i] = boats[i];
}

double TreeWater::CascadeTime(double simUnix) const {
    if (!m_sea) return 0.0;
    // The float cast is deliberate and is explained in the header: mirror the GPU's clock, do
    // not improve on it.
    return static_cast<double>(static_cast<float>(simUnix - m_sea->CycleUnix()));
}

double TreeWater::WindowWeight(double wx, double wz) const {
    if (!m_wave || !m_wave->Ready()) return 0.0;
    const WaveField::GpuTable& t = m_wave->Table();
    if (t.nx < 2 || t.ny < 2 || !(t.invCell > 0.0f)) return 0.0;
    // CsBankFill's own expression (WaterBank.hlsl:425): distance to the nearest window edge in
    // CELLS, converted to metres, feathered over featherM. Same curve, same constants -- if this
    // drifts from the kernel the twin will show a seam exactly at the window edge and nowhere
    // else, which is the signature to look for.
    const double cx = (wx - double(t.orgX)) * double(t.invCell);
    const double cz = (wz - double(t.orgZ)) * double(t.invCell);
    const double eCells = (std::min)((std::min)(cx, double(t.nx) - cx),
                                     (std::min)(cz, double(t.ny) - cz));
    const double eM = eCells / double(t.invCell);
    return SmoothStep(0.0, (std::max)(double(t.feather), 1.0), eM);
}

const WeatherSample& TreeWater::SlowAt(double wx, double wz, double simUnix) const {
    const double cx = std::floor(wx / kSlowCellM);
    const double cz = std::floor(wz / kSlowCellM);
    if (cx != m_memoX || cz != m_memoZ || simUnix != m_memoT) {
        double latDeg = 0.0, lonDeg = 0.0;
        // Ask at the CELL CENTRE, not at the caller's point: then every station in the cell gets
        // the same answer regardless of which one asked first, so the hull's forces do not
        // depend on the order its elements happen to be evaluated in.
        PlaceOf((cx + 0.5) * kSlowCellM, (cz + 0.5) * kSlowCellM, latDeg, lonDeg);
        // The SLOW field only: the solver's surface varies across a cell (the throat's jet, the
        // basin's gradient) and is cheap to read, so every point refines it (At, MeanLevelAt).
        m_memo = m_wx->Query(latDeg, lonDeg, simUnix, 1.0, false);
        m_memoX = cx;
        m_memoZ = cz;
        m_memoT = simUnix;
    }
    return m_memo;
}

WeatherSample TreeWater::MeanStateAt(double wx, double wz, double simUnix) const {
    WeatherSample q = SlowAt(wx, wz, simUnix);
    double latDeg = 0.0, lonDeg = 0.0;
    PlaceOf(wx, wz, latDeg, lonDeg);
    m_wx->SolverRefine(latDeg, lonDeg, simUnix, q);   // THE SOLVER IS TRUTH, at this point
    return q;
}

double TreeWater::MeanLevelAt(double wx, double wz, double simUnix) const {
    if (!m_wx) return std::nan("");
    const WeatherSample q = MeanStateAt(wx, wz, simUnix);
    return (q.levelSrc && q.levelSrc[0] != '-') ? q.levelNavd : std::nan("");
}

SurfaceSample TreeWater::At(double wx, double wz, double simUnix) const {
    SurfaceSample s;
    if (!m_wx) return s;   // valid stays false: no tree attached is not flat water

    const WeatherSample q = MeanStateAt(wx, wz, simUnix);

    // ---- THE COVERAGE GATE. Test the PROVENANCE, never the value: a bed of 0.0 is what both a
    // point at datum and a point nothing covers return, and only one of those is a measurement.
    const bool haveBed = q.bedSrc && q.bedSrc[0] != '-';
    const bool haveLevel = q.levelSrc && q.levelSrc[0] != '-';
    if (!haveBed || !haveLevel) return s;

    s.bedNavd = double(q.bedNavd);
    s.heightNavd = q.levelNavd;             // mean surface: tide atlas + any solver refinement
    s.vx = double(q.u);                     // surface current, east
    s.vz = double(q.v);                     // surface current, north
    s.vy = 0.0;

    // ---- THE WATER'S OWN CONTEXT, in CsBankFill's order (the water match, step 2: one wave rule).
    // The depth under the mean surface; the dry weight every displacement rides (the kernel's `dry`);
    // the local sea-state scale (WaveScale, the one law the bank's tile corners carry). The swell
    // shadow is the kernel's page read and has no CPU reader yet: the hull's sea stands exposed (1),
    // which Describe() states and --water-probe measures against the page at the hull.
    const double depth = q.levelNavd - double(q.bedNavd);
    const double dry = wt::Smoothstep(0.05, 0.65, depth);
    const double expo = 1.0;
    double latDeg = 0.0, lonDeg = 0.0;
    PlaceOf(wx, wz, latDeg, lonDeg);
    const double hsScale =
        WaveScale::For(m_wx->Globe(), m_sea, m_storm, simUnix).At(latDeg, lonDeg);

    // ---- THE WAVES, in CsBankFill's order and with its weights. The solved field owns the
    // window and the cascades own everywhere else; wWin is the one blend and it is the same
    // expression on both processors.
    // THE BLEND MUST SUM TO ONE, and the only honest way to guarantee that is to let the
    // window weight be WHAT THE SOLVED FIELD ACTUALLY SUPPLIED rather than what the geometry
    // says it should have.
    //
    // ABSENCE IS NOT ZERO -- the ingest rule, and this is where breaking it capsizes a boat.
    // The solved field's pages STREAM. Drive into an area whose tiles have not landed and
    // ProbeAt answers "not valid", so the solved term was skipped -- but the cascades below
    // were still handed only (1 - wWin), so wWin of the sea VANISHED. Not a smoother sea: a
    // HOLE, with the straight edge of the tile that had not arrived, sitting next to full
    // amplitude on the tile that had. The hull reads that step as a wall and goes over it,
    // which is exactly "weird waves at the edge of tiles that flip the boat, when I move the
    // boat to a new area".
    //
    // A point the solved field does not cover is not a point with less sea. It is a point the
    // cascades own entire, exactly as they do everywhere outside the window -- so wWin starts
    // at zero and is raised only by a probe that answered. Streaming then changes WHICH
    // description carries the sea, never HOW MUCH sea there is.
    double rwx = 0.0, rwz = 0.0;   // this point in the root's flat frame (the window, the wakes)
    RootOf(wx, wz, rwx, rwz);
    const double wGeom = WindowWeight(rwx, rwz);
    double wWin = 0.0;

    double dispX = 0.0, dispY = 0.0, dispZ = 0.0;   // wave displacement, physical metres
    double slopeX = 0.0, slopeZ = 0.0;
    double orbX = 0.0, orbY = 0.0, orbZ = 0.0;

    if (wGeom > 0.001 && m_wave) {
        const WaveField::Probe p = m_wave->ProbeAt(rwx, rwz, simUnix);
        if (p.valid) {
            wWin = wGeom;
            // The swell shadow shelters the solved components as it does the cascades (the kernel's
            // aW = a * aMax * expo).
            const double wS = wWin * expo;
            dispY += wS * double(p.eta);
            dispX += wS * m_waveChop * double(p.dx);   // gWaveB.z, see the header
            dispZ += wS * m_waveChop * double(p.dz);
            slopeX += wS * double(p.sx);
            slopeZ += wS * double(p.sz);
            orbX += wS * double(p.vx);
            orbY += wS * double(p.vy);
            orbZ += wS * double(p.vz);
        }
    }
    if (m_ocean && m_ocean->Ready()) {
        // THE PER-BAND LAW, the kernel's (CsBankFill's cascade loop), one gain per cascade: the
        // local sea-state scale and the swell shadow; in water, the band's shoaling (Green's law at
        // its representative wavenumber) and its wave-current gain (the solver's current projected
        // on the peak's travel, against the band's phase speed at this depth -- the kernel has no
        // other current, so neither does its twin); and inside the solved window the STRUCTURE
        // bands (0 and 1) yield to the solved field by its weight while the chop band stays on.
        // (The twin used to stand all three down by (1 - wWin) and apply none of the rest.)
        const double curX = q.currentSolved ? double(q.u) : 0.0;
        const double curZ = q.currentSolved ? double(q.v) : 0.0;
        double gain[OceanCpu::kCascades];
        for (int c = 0; c < OceanCpu::kCascades; ++c) {
            double amp = hsScale * expo;
            if (depth > 0.05) {
                const double kB = m_ocean->BandK(c);
                const double cB = BandPhaseSpeed(kB, wt::Max(depth, 0.3));
                const WaveCurrentGain ab = m_peakValid
                                               ? WaveCurrentAmp(curX, curZ, m_peakDirX, m_peakDirZ, cB)
                                               : WaveCurrentGain{1.0, 0.0};
                amp *= ab.amp * ShoalFactor(kB, depth);
            }
            if (c != 2) amp *= 1.0 - wWin;
            gain[c] = amp;
        }
        OceanSample o;
        // Band-limited to what the hull can actually feel. A wave shorter than the panel
        // spacing puts as much up-force on one half of a panel as down on the other, so it
        // integrates to nothing -- summing it is a sincos per bin for a force of zero, and the
        // short cascade is ~85% of the retained bins.
        m_ocean->SampleForHull(wx, wz, CascadeTime(simUnix), 2.0 * m_sampleM, gain, o);
        dispY += o.h;
        dispX += o.dx;
        dispZ += o.dz;
        slopeX += o.sx;
        slopeZ += o.sz;
        orbX += o.vx;
        orbY += o.vy;
        orbZ += o.vz;
    }

    // ---- THE WAKES. Closed form and stateless, so the CPU runs the same law the bank does
    // rather than a cheaper stand-in -- WaterTerms.h carries the signed stationary phase and the
    // gate that the folded reference form fails. Superposition: N boats cost a loop and there is
    // no interaction to resolve.
    //
    // The band-limit is the one place a hull differs from the kernel. CsBankFill limits against
    // ITS tile's texel, because there the kernel IS the mesh; a hull has no texel, so it declares
    // the scale it wants resolved (SetSampleScale, default the panel spacing). Below that a wake
    // ripple cannot push the hull anyway -- it would integrate to nothing across a panel -- so
    // resolving it would be cost without force.
    if (m_boatCount > 0) {
        WakeSample w;
        for (int i = 0; i < m_boatCount; ++i) {
            const WakeBoat& b = m_boats[i];
            if (!b.enabled) continue;
            WakeVessel v;
            v.x = b.x; v.z = b.z; v.heading = b.headingRad; v.speed = b.speedMs;
            v.wakeAmp = b.ampM; v.hullHalfLen = b.halfLenM; v.enabled = true;
            WakeOne(v, rwx, rwz, m_sampleM, w);
        }
        // The bank adds the wake to d.y BEFORE the exaggeration multiplies the total, so the
        // wake is exaggerated with everything else. Matching that is what keeps a hull riding
        // its neighbour's wake at the height the neighbour's wake is drawn.
        dispY += w.eta;
        slopeX += w.slopeX;
        slopeZ += w.slopeZ;
    }

    // ---- THE EXAGGERATION AND THE DRY WEIGHT. The kernel's `d *= dry * gPatch.w`: gPatch.w
    // multiplies the bank's whole displacement, so it multiplies the slope with it (the slope IS a
    // derivative of the thing being scaled) and the orbital velocity too (the same surface moving
    // through the same period); the dry weight takes every wave off water too thin to carry one. The
    // hull rides the sea the renderer draws; the header says why, and --height-scale 1.0 is the
    // physics-truth run.
    double k = dry * m_heightScale;
    // ---- DEPTH-LIMITED BREAKING, the kernel's cap: no displacement taller than 0.55 of the depth.
    // An AMPLITUDE cap, so the whole displacement -- and the slope and the orbital velocity that are
    // linear in the same amplitude -- scales by the one ratio: continuous at the cap, where the
    // kernel's horizontal step to 0.85 was not.
    const double hmax = 0.55 * wt::Max(depth, 0.05);
    const double hY = std::abs(k * dispY);
    if (hY > hmax) k *= hmax / hY;
    s.heightNavd += k * dispY;
    s.dx = k * dispX;
    s.dz = k * dispZ;
    s.vx += k * orbX;
    s.vy += k * orbY;
    s.vz += k * orbZ;

    // ---- THE NORMAL, from the slope of the surface the hull is standing on. n proportional to
    // (-dh/dx, 1, -dh/dz), normalised. This is the Eulerian normal and it ignores the horizontal
    // Gerstner offset's contribution to the surface tangent -- the same approximation the bank
    // makes, kept deliberately so the two agree.
    const double sx = k * slopeX, sz = k * slopeZ;
    const double inv = 1.0 / std::sqrt(sx * sx + 1.0 + sz * sz);
    s.nx = -sx * inv;
    s.ny = inv;
    s.nz = -sz * inv;

    s.depthM = s.heightNavd - s.bedNavd;
    s.valid = true;
    return s;
}

void TreeWater::WindAt(double wx, double wz, double simUnix, double out[3]) const {
    out[0] = out[1] = out[2] = 0.0;
    if (!m_wx) return;
    double latDeg = 0.0, lonDeg = 0.0;
    LatLonOf(wx, wz, latDeg, lonDeg);
    const WeatherSample q = m_wx->Query(latDeg, lonDeg, simUnix, 1.0);
    if (!q.windSrc || q.windSrc[0] == '-') return;   // no wind data is calm, and says so
    out[0] = double(q.windU);   // east
    out[1] = 0.0;               // the 10 m field is horizontal; vertical gust is not modelled
    out[2] = double(q.windV);   // north
}

std::string TreeWater::Describe(double wx, double wz, double simUnix) const {
    if (!m_wx) return "water.tree: NO TREE ATTACHED (answers valid=false everywhere)";
    double latDeg = 0.0, lonDeg = 0.0;
    LatLonOf(wx, wz, latDeg, lonDeg);
    const WeatherSample q = m_wx->Query(latDeg, lonDeg, simUnix, 1.0);
    // THE AGE IT READS. Inside a solver's domain the level and the current are the solver's, as of
    // the region it last delivered (WeatherManager::RequestRegion); this says that instant, or that
    // no solver has answered yet -- in which case a point the solver owns reports no level at all.
    const double asOf = m_wx->SolverAsOf();
    char age[96];
    if (asOf <= WeatherManager::kNeverRead) {
        snprintf(age, sizeof(age), "no solver has answered (its domain reports no level yet)");
    } else {
        snprintf(age, sizeof(age), "solver answers as of t=%.0f, %.1f s old", asOf, simUnix - asOf);
    }
    // THE BAND LAWS' INPUTS (one wave rule): the sea-state scale, the peak the wave-current gain
    // projects on, and the one term the hull does not read yet -- the swell shadow (exposed).
    const WaveScale scale = WaveScale::For(m_wx->Globe(), m_sea, m_storm, simUnix);
    char laws[200];
    snprintf(laws, sizeof(laws),
             "sea state x%.2f (%s, ref Hs %.2f m) | peak %s | swell shadow NOT READ (exposed)",
             scale.At(latDeg, lonDeg), m_storm ? "a declared storm is the reference" : "grid",
             scale.hsRef, m_peakValid ? "valid" : "none (wave-current gain 1)");
    char buf[900];
    snprintf(buf, sizeof(buf),
             "water.tree @ (%.1f, %.1f) = %.5f/%.5f deg | bed %s | level %s | current %s | "
             "wind %s | wWin %.3f | cascades %s | solved %s | exag %.3f | %s | %s",
             wx, wz, latDeg, lonDeg, q.bedSrc, q.levelSrc, q.currentSrc, q.windSrc,
             WindowWeight(wx, wz),
             (m_ocean && m_ocean->Ready()) ? "ready" : "ABSENT",
             (m_wave && m_wave->Ready()) ? "ready" : "ABSENT", m_heightScale, laws, age);
    return std::string(buf);
}

}  // namespace ga
