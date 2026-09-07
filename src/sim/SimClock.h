// SimClock -- the scene clock advances in WHOLE QUANTA, never by however long the last frame
// happened to take.
//
// WHAT WAS WRONG. The playable path advanced the scene clock as `simUnix += dt * timeScale` with
// dt straight off the wall clock (main.cpp), so the amount of world time a frame covered was a
// function of how fast that frame rendered. Headless already had a fixed step -- the clock there
// is `startUnix + recFrame / 30` -- which is exactly why every rail and settled still in this
// project is reproducible and nothing in the interactive path is.
//
// Everything downstream inherits that. SweSolver already integrates at its own fixed 0.25 s
// substep and chases the scene clock (SweSolver.h), capped at kMaxSubsteps per frame: feed it a
// clock that lurches and it falls behind by an amount that depends on the frame rate, which is
// docs/PERF_EXPERIMENT.md step 26 ("the solver's state after N sim-seconds must not depend on
// wall time") at its root. The churn atlas is stateful for the same reason. And a boat integrated
// against a wall-clock dt handles differently at 60 fps than at 140 -- the classic vehicle-sim
// failure, and painful to retrofit once a hull is riding on it.
//
// THE PROPERTY THIS BUYS, and the one the self-test pins: the number of quanta taken over an
// interval depends on the LENGTH of the interval, not on how it was chopped into frames, and the
// clock lands on the same VALUE to the bit. It holds down to the catch-up floor,
// kHz / kMaxStepsPerFrame = 3.75 fps; below that the world slows rather than spiralling, which is
// the deliberate choice and is pinned by the test rather than left to be discovered.
//
// Consumers that INTEGRATE step once per quantum -- `for (int i = 0; i < steps; ++i) Step(kDt)`,
// which is where boat physics attaches. Consumers that are pure functions of time (the tide, the
// wave field, the sun) just read Now(), which is now always an exact multiple of the quantum
// away from where the clock was last set.
#pragma once

#include <cmath>
#include <cstdint>

namespace ga {

class SimClock {
public:
    // 240 Hz: fine enough for rigid-body work at the rate the boat wants, and the quantum costs
    // nothing on its own -- Advance is arithmetic, not a loop over steps.
    static constexpr double kHz = 240.0;
    static constexpr double kDt = 1.0 / kHz;
    // A hitch must not become a debt the sim spends the next second repaying (the spiral of
    // death). 64 quanta is 0.27 s of catch-up; past that the missed time is SHED and counted,
    // because a visible skip is honest and an unbounded catch-up loop is not.
    static constexpr int kMaxStepsPerFrame = 64;

    void Reset(double unixSeconds) {
        m_base = unixSeconds;
        m_accum = 0.0;
        m_steps = 0;
        m_total = 0;
        m_shed = 0;
    }
    // A deliberate jump: the time-nudge keys, HOME, a scene reload. The clock re-bases and the
    // partial quantum is dropped rather than smeared across the discontinuity.
    void SetTo(double unixSeconds) {
        m_base = unixSeconds;
        m_steps = 0;
        m_accum = 0.0;
    }

    // Advance by a wall-clock interval. Returns the number of WHOLE quanta taken, which is what
    // an integrating consumer steps. `timeScale` is sim seconds per wall second.
    int Advance(double wallDt, double timeScale) {
        if (!(wallDt > 0.0)) return 0;
        const double owed = m_accum + wallDt * timeScale;
        if (owed < kDt) {
            m_accum = owed;
            return 0;
        }
        double whole = std::floor(owed / kDt);
        int steps;
        if (whole >= static_cast<double>(kMaxStepsPerFrame)) {
            steps = kMaxStepsPerFrame;
            m_shed += static_cast<uint64_t>(whole) - static_cast<uint64_t>(kMaxStepsPerFrame);
            m_accum = 0.0;   // shed the rest: no debt survives the hitch
        } else {
            steps = static_cast<int>(whole);
            m_accum = owed - whole * kDt;
        }
        m_steps += static_cast<uint64_t>(steps);
        m_total += static_cast<uint64_t>(steps);
        return steps;
    }

    // COMPUTED FROM THE COUNT, not accumulated into. `m_unix += steps * kDt` would make the
    // clock depend on how the quanta were grouped into frames: 600 frames of 4 steps and 40 of
    // 60 sum the same 2400 quanta to values an epsilon apart, and the whole point of this class
    // is that those two sessions are the same world. One multiply, and they are equal to the bit.
    double Now() const { return m_base + static_cast<double>(m_steps) * kDt; }
    // The fraction of a quantum already accumulated: what a renderer interpolates by when it
    // draws between two sim states. Nothing reads it yet; the boat will.
    double Alpha() const { return m_accum / kDt; }
    uint64_t Steps() const { return m_total; }
    uint64_t Shed() const { return m_shed; }

private:
    double m_base = 0.0;      // where the clock was last SET (Reset/SetTo)
    double m_accum = 0.0;     // sim seconds owed, always < kDt
    uint64_t m_steps = 0;     // quanta since m_base: Now() is exactly m_base + m_steps * kDt
    uint64_t m_total = 0;     // quanta over the clock's life, for reporting
    uint64_t m_shed = 0;
};

// Defined in SimClockTest.cpp; runs under --selftest.
bool RunSimClockSelfTest();

}  // namespace ga
