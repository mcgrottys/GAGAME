// ================================================================================================
//  TreeWater - M9bq: the WaterSurface a hull actually sails on, read out of the sparse GA tree.
//
//  WaterSurface.h declares the interface and three ANALYTIC seas (still, tilted, Airy) whose
//  answers are closed form, because a gate needs water it already knows the answer for. This is
//  the other kind: the real one, assembled from the tree, for the boat the player drives.
//
//  IT IS THE CPU TWIN OF CsBankFill (shaders/WaterBank.hlsl), and it is assembled in that
//  kernel's order so the two can be diffed term by term rather than argued about:
//
//      mean state   WeatherManager::Query   tide atlas + SWE window refinement, the one bed,
//                                           the surface current, the 10 m wind
//      solved sea   WaveField::ProbeAt      inside the solve window, weighted by wWin (x the shadow)
//      cascade sea  OceanCpu::SampleForHull everywhere, one gain per cascade -- the kernel's band
//                                           law (sea-state scale, shadow, shoaling, wave-current);
//                                           the structure bands yield by (1 - wWin), the chop stays
//      wake         WaterTerms WakeOne      the fleet's Kelvin wakes, closed form
//      over all     the dry weight, the display exaggeration, the depth-limited amplitude cap
//
//  WHY THE MEAN STATE IS NOT RE-DERIVED HERE. WeatherManager::Query is already the tree's point
//  evaluator: it composes the height stack, refines the level inside a resident solver window,
//  falls back from the solver's current to GoMOFS, and carries the 10 m wind. Rewriting that
//  here would be a second description of the same physics that nothing ever checks against the
//  first -- which is the exact failure the compositor's one-sampler rule exists to prevent. One
//  evaluator, read by both.
//
//  ABSENCE IS NOT ZERO, AND THIS IS THE TRAP WORTH NAMING. Compositor::SampleHeightStack starts
//  at h = 0 and skips layers of zero weight, so a point NO source covers returns NAVD 0 -- which
//  reads as `the seabed is exactly at the surface`, which reads as aground. Offshore the ETOPO
//  layer covers the globe at weight 1 and saves us; but if it ever fails to load, a boat in
//  mid-Atlantic would quietly believe it had run onto the beach, and every other number would
//  look fine. So this class does NOT test the value -- it tests WeatherSample's provenance
//  strings, which say which rung answered, and reports `valid = false` when nothing did. A hull
//  that does not know where the bottom is must say so, not guess zero.
//
//  THE EXAGGERATION, DECIDED AND STATED. The engine draws the sea taller than physics:
//  SeaLayer::heightScale (1.15) scales the whole bank displacement, and WaveFieldConfig::
//  displayExag (1.15) is folded into the solved field's dequant tables -- so inside the window
//  the drawn sea is ~1.32x physics and outside it ~1.15x. The hull rides the DISPLAYED surface,
//  because a boat floating visibly above its own wave is the one error a player cannot unsee,
//  and because `the scene runs a 1.15x sea` is a single stated number rather than a fudge in two
//  places. `--height-scale 1.0 --exagg 1.0` is then the physics-truth run, and that is the one to
//  use against a real IMU log.
//
//  NO READBACKS, EVER. Not SweSolver::ReadProbes, not WaterBankLayer::TraceProbe. Both pull whole
//  multi-megabyte textures and end in Gpu::WaitIdle(); at 240 Hz that is not a slow path, it is a
//  stopped renderer. Everything here is analytic or reads a CPU-resident mirror.
// ================================================================================================
#pragma once

#include "sim/OceanCpu.h"
#include "sim/PlaceField.h"
#include "sim/SeaState.h"
#include "sim/WaterSurface.h"
#include "sim/WaveField.h"
#include "sim/WeatherManager.h"
#include "core/Space.h"

#include <string>

namespace ga {

// One vessel's contribution to the wake field, in the same layout WaterBankLayer::SetBoats uses
// (x, z, heading rad, speed m/s) and (wake amp m, hull half length m, enabled, spare) so the CPU
// and the bank kernel are fed from one table and cannot drift.
struct WakeBoat {
    double x = 0, z = 0, headingRad = 0, speedMs = 0;
    double ampM = 0, halfLenM = 0;
    bool enabled = false;
};

class TreeWater : public WaterSurface {
public:
    // All four may be null; each absence removes a term and is REPORTED by Describe() rather
    // than silently substituting zero. A TreeWater with no weather manager answers `valid =
    // false` everywhere, which is the honest reading of `I have no tree attached`.
    void Configure(const WeatherManager* wx, const WaveField* wave, const OceanCpu* ocean,
                   const SeaState* sea, double heightScale, double waveExag, double waveChop);

    // The fleet, for the wake term. Copied, not referenced: this is read from the physics tick
    // while the frame thread rewrites the table.
    void SetBoats(const WakeBoat* boats, int count);

    // The sample size the wake band-limits against, metres. The bank uses ITS tile's texel so the
    // kernel is the mesh; a hull has no texel, so it declares the scale it wants resolved -- the
    // panel spacing. Below this the wake is a force the hull cannot feel anyway.
    void SetSampleScale(double m) { m_sampleM = (m > 0.01) ? m : 0.01; }

    // THE CASCADE SEA'S CONTEXT (the water match, step 2), as the bank kernel is handed it: the
    // direction its most energetic partition travels (gPeakDir -- the wave-current gain projects the
    // current on it), and whether its partitions are a declared storm (whose height IS the reference,
    // WaveScale). The sea layer owns both; whoever steps a hull hands them in, every step. Until then
    // there is no peak (the gain is 1, as the kernel's is without one) and no storm.
    void SetCascadeSea(double peakDirX, double peakDirZ, bool peakValid, bool storm) {
        m_peakDirX = peakDirX;
        m_peakDirZ = peakDirZ;
        m_peakValid = peakValid;
        m_storm = storm;
    }
    // THE SWELL SHADOW, as the kernel reads it (the water match, step 3): the exposure page's texels
    // at a place (compose/ExposurePage -- the node asked at the texel centres the painter asks it at,
    // at the kernel's floor mip, quantized as the page stores them), floored at kSwellShadowFloor; no
    // opinion (no swell direction, outside the page) is exposed, the kernel's own absence law. Null:
    // no shadow reader, and the sea stands exposed.
    void SetSwellShadow(const PlaceField* shadow) { m_shadow = shadow; }
    // THE BED THE WATER KERNELS READ (the water match, step 3): the height page's finest texels at a
    // place (compose/HeightPage), for every depth law this water applies and the bed it reports. No
    // opinion, or null: the slow field's bed (the stack at 1 m, per 8 m memo cell).
    void SetBed(const PlaceField* bed) { m_bed = bed; }

    // THE CHART (the cuboid gate, 2026-09-14): the flat frame this water is read in. None (the
    // default) is the root's -- the ACT0816 constants, byte for byte. A hull carried through a
    // gate reads the SAME trees through its destination space's chart: a point's place comes from
    // that chart; the solved window and the fleet's wakes are asked at the point's place in the
    // root's chart (away from the Merrimack they answer nothing); and the cascades, a stationary
    // sea, are evaluated in the chart's own metres.
    void SetChart(const Space::Anchor* chart);
    // The place (lat, lon) of a point of this water's flat frame.
    void PlaceOf(double wx, double wz, double& latDeg, double& lonDeg) const;

    const char* Name() const override { return "water.tree"; }
    // THE SURFACE THE MESH DRAWS, at a point of the world (the water match, step 3). The mesh places
    // each particle at its label plus its lateral offset, so the water standing over a point came
    // from a label behind it: the height, normal and offset here are that particle's, found by one
    // Newton step on label + offset = point with the offset's own Jacobian (first order in the
    // steepness beyond the step, second order in what is left).
    SurfaceSample At(double wx, double wz, double simUnix) const override;
    // The same water at a LABEL: the particle whose rest position is the point, before its lateral
    // offset -- what the bank's texel at that point holds. For instruments that compare the two
    // processors texel for texel (--twin-surface, --water-probe's kernel column); a hull wants At.
    SurfaceSample AtLabel(double wx, double wz, double simUnix) const;
    void WindAt(double wx, double wz, double simUnix, double out[3]) const override;

    // What answered, and what did not -- for the boot log and the twin report. M12 step 5e:
    // and the AGE of the solver mirror the level and current are read from, or that it was
    // never read (the freshness contract, scene/Entity.h).
    std::string Describe(double wx, double wz, double simUnix) const;

    // The solved-window blend weight at a point, 0 outside. Public because the twin gate reports
    // it per site: the term split is the thing most likely to explain a disagreement.
    double WindowWeight(double wx, double wz) const;
    // The MEAN surface alone at a point of this water's frame (the slow field At() adds its waves
    // to), NAVD metres; NaN where no level rung answered. For the water probe's level/wave split.
    double MeanLevelAt(double wx, double wz, double simUnix) const;
    // The per-band amplitude law at a point -- the FULL closure (sea-state scale x shadow x shoaling
    // x wave-current, before the solved window's stand-down) and the dry weight: what the bank
    // kernel writes to its detail plane, for --water-probe to hold the two against. False where no
    // level or bed answered.
    bool BandGains(double wx, double wz, double simUnix, double gains[OceanCpu::kCascades],
                   double& dry) const;

private:
    // The cascade clock. SeaLayer feeds the FFT `simUnix - CycleUnix()` cast to FLOAT
    // (SeaLayer.cpp:526), and at a forecast hour that cast quantises time to ~8 ms. The twin
    // mirrors the cast rather than keeping the double, because the point is to sample the sea
    // the GPU is drawing, not a better one: 8 ms of a 10 s wave is 0.3 degrees of phase, and
    // matching it costs nothing while NOT matching it would show up in the twin as a drift that
    // grows with the forecast hour and look like a bug in this file.
    double CascadeTime(double simUnix) const;
    // At and AtLabel: one assembly; `displaced` stands the answer on the drawn surface.
    SurfaceSample Evaluate(double wx, double wz, double simUnix, bool displaced) const;
    // The kernel's cascade loop's gains, once for At and BandGains.
    void BandLaw(const WeatherSample& q, double depth, double hsScale, double expo,
                 double gain[OceanCpu::kCascades]) const;

    const WeatherManager* m_wx = nullptr;
    const WaveField* m_wave = nullptr;
    const OceanCpu* m_ocean = nullptr;
    const SeaState* m_sea = nullptr;
    double m_heightScale = 1.0;    // SeaLayer::heightScale -- the bank's display exaggeration
    double m_waveExag = 1.0;       // reported only; WaveField folds its own into the dequant
    // WaveField::ProbeAt returns the solved field's Gerstner offset UNSCALED, because the scale
    // is a scene knob and not a property of the solve. CsBankFill applies it as gWaveB.z
    // (WaterBankLayer.cpp:613 <- WaterSceneConfig::wfChop, default 1.1), so the twin must too --
    // otherwise every horizontal displacement inside the window is 10% short and the error hides
    // as a small phase-looking offset rather than announcing itself as a missing factor.
    double m_waveChop = 1.0;
    double m_sampleM = 0.5;
    double m_peakDirX = 0.0, m_peakDirZ = 0.0;
    bool m_peakValid = false;
    bool m_storm = false;
    const PlaceField* m_shadow = nullptr;
    const PlaceField* m_bed = nullptr;
    // The bed at a place: the kernels' page (SetBed) or the slow field's.
    double BedAt(const WeatherSample& q, double latDeg, double lonDeg) const;
    // The swell shadow at a place, by the kernel's law (SetSwellShadow).
    double ExposureAt(double latDeg, double lonDeg) const;
    WakeBoat m_boats[8];
    int m_boatCount = 0;

    // ---- THE SLOW FIELD, MEMOISED. --------------------------------------------------------
    // A hull asks this evaluator ~120 times per physics step, once per station and per tube
    // slice, and WeatherManager::Query is not cheap: six height layers, eighteen tide
    // constituents each a compositor sample, the current, the wave grid and the wind. Measured
    // at 27 us a call it was 28 ms a frame, and the boat cost more than the entire renderer.
    //
    // But that query answers the SLOW field. The tide gradient is ~1e-6 m/m, the surge and the
    // current vary over hundreds of metres, and the wind over kilometres -- none of them change
    // meaningfully across a 5.5 m hull. What DOES vary at hull scale is the waves, and those are
    // evaluated per station regardless. So the mean state is fetched once per cell and reused --
    // all but the SOLVER's surface (the water match, step 1): the throat's jet and the basin's
    // gradient vary within a cell, and reading a delivered region is cheap, so each point is
    // refined by the solver on its own (WeatherManager::SolverRefine) and the hull's level is as
    // continuous as the kernel's.
    //
    // The cell is 8 m and the memo is one entry, which is all a single hull needs (its stations
    // are inside one cell). Quantising POSITION means a hull straddling a boundary flips between
    // two answers that differ by ~1e-5 m of tide -- far under the wave amplitude and under the
    // atlas's own half-LSB. The BED is the term that genuinely varies over 8 m, and it is used
    // for depth and grounding, not for a force, so a hull-length smoothing of it is honest.
    //
    // Single-threaded by contract: this is read from the physics tick only. If a second vessel
    // ever steps concurrently this needs a per-vessel memo, not a lock.
    static constexpr double kSlowCellM = 8.0;
    bool m_hasChart = false;
    Space::Anchor m_chart;
    // A point of this water's frame in the ROOT's flat frame (the solved window's and the wakes').
    void RootOf(double wx, double wz, double& rx, double& rz) const;
    mutable double m_memoX = 1e30, m_memoZ = 1e30, m_memoT = -1e30;
    mutable WeatherSample m_memo;
    const WeatherSample& SlowAt(double wx, double wz, double simUnix) const;
    // The mean state at a point: the memoised slow field, refined by the solver at the point.
    WeatherSample MeanStateAt(double wx, double wz, double simUnix) const;
};

}  // namespace ga
