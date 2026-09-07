// RunSimClockSelfTest -- the gate on the fixed-timestep scene clock.
//
// The property that matters is not "it advances"; it is that the world advances by the same
// amount over the same INTERVAL however that interval was chopped into frames. That is the whole
// reason for the change, so it is the assertion with the most weight here: the same total wall
// time delivered as 600 tiny frames, 120 ordinary ones and 40 slow ones must produce the identical
// number of quanta and the identical clock, to the bit. That holds BELOW the catch-up cap and not
// above it -- inherent to every fixed-timestep scheme -- so the cap boundary is pinned separately.
//
// None of this is reachable from a rail or a settled still: both pin the clock to the frame index
// (main.cpp), which is exactly why the interactive path was the one that drifted. A self-test is
// the only gate this code has.
#include <cstdio>
#include <vector>

#include "core/Common.h"
#include "sim/SimClock.h"

namespace ga {

namespace {

bool Near(double a, double b, double tol) { return (a > b ? a - b : b - a) <= tol; }

bool Expect(const char* what, double got, double want, double tol = 0.0) {
    if (Near(got, want, tol)) return true;
    Log("[simclock]   FAIL %s: %.9f, expected %.9f", what, got, want);
    return false;
}

// Run a sequence of wall-clock frame times through a fresh clock.
struct Run {
    uint64_t steps = 0;
    double now = 0.0;
    uint64_t shed = 0;
};
Run Drive(const std::vector<double>& frames, double timeScale = 1.0, double start = 1000.0) {
    SimClock c;
    c.Reset(start);
    for (const double d : frames) c.Advance(d, timeScale);
    return {c.Steps(), c.Now(), c.Shed()};
}

}  // namespace

bool RunSimClockSelfTest() {
    bool ok = true;
    const double kDt = SimClock::kDt;

    // (1) The clock only ever lands on whole quanta from where it was set.
    {
        SimClock c;
        c.Reset(1000.0);
        c.Advance(0.0131, 1.0);   // not a multiple of anything
        const double off = c.Now() - 1000.0;
        const double q = off / kDt;
        ok &= Expect("advance is a whole number of quanta", q - std::floor(q), 0.0, 1e-9);
        ok &= Expect("quanta counted match the clock", static_cast<double>(c.Steps()), q, 1e-9);
    }

    // (2) THE ONE THAT MATTERS. The same total wall time, chopped different ways, must advance
    //     the world identically -- that is what "frame-rate independent" means, and it is the
    //     property the old `simUnix += dt * timeScale` did not have once anything downstream
    //     integrated per frame rather than per quantum.
    //
    //     IT HOLDS BELOW THE CAP AND NOT ABOVE IT, which is inherent to every fixed-timestep
    //     scheme and is asserted here rather than assumed: the cap is what stops a hitch turning
    //     into a debt, and a frame that owes more than kMaxStepsPerFrame sheds the rest. So the
    //     framings below run from 60 fps down to 4 fps -- 240/4 = 60 quanta a frame, just inside
    //     the 64 cap -- and case (5) pins what happens past it. The implied floor is
    //     kHz / kMaxStepsPerFrame = 3.75 fps: below that the world slows down instead of
    //     spiralling, which is the deliberate choice.
    {
        const double total = 10.0;
        std::vector<double> fast(600, total / 600.0);   // 60 fps
        std::vector<double> mid(120, total / 120.0);    // 12 fps
        std::vector<double> slow(40, total / 40.0);     // 4 fps: 60 quanta a frame
        const Run a = Drive(fast), b = Drive(mid), c = Drive(slow);
        if (a.steps != b.steps || b.steps != c.steps) {
            Log("[simclock]   FAIL same interval, different framing: %llu / %llu / %llu quanta",
                static_cast<unsigned long long>(a.steps), static_cast<unsigned long long>(b.steps),
                static_cast<unsigned long long>(c.steps));
            ok = false;
        }
        ok &= Expect("60 fps and 4 fps clocks agree bit for bit", a.now - c.now, 0.0, 0.0);
        ok &= Expect("10 s advanced 2400 quanta at 240 Hz", static_cast<double>(a.steps), 2400.0,
                     0.0);
        ok &= Expect("nothing was shed above the floor",
                     static_cast<double>(a.shed + b.shed + c.shed), 0.0, 0.0);
    }

    // (2b) The floor is where the doc says it is. One frame at exactly the cap keeps everything;
    //      one frame past it shears. Pinned so the number cannot drift silently.
    {
        SimClock c;
        c.Reset(1000.0);
        ok &= Expect("a frame at the cap sheds nothing",
                     static_cast<double>(c.Advance(SimClock::kMaxStepsPerFrame * SimClock::kDt,
                                                   1.0)),
                     static_cast<double>(SimClock::kMaxStepsPerFrame), 0.0);
        ok &= Expect("... and owes nothing", static_cast<double>(c.Shed()), 0.0, 0.0);
        SimClock d;
        d.Reset(1000.0);
        d.Advance((SimClock::kMaxStepsPerFrame + 10) * SimClock::kDt, 1.0);
        ok &= Expect("a frame past the cap sheds the excess", static_cast<double>(d.Shed()), 10.0,
                     0.0);
    }

    // (3) Ragged frame times -- what a real session delivers -- lose nothing. The clock tracks
    //     the wall interval to within the one quantum still in the accumulator.
    {
        std::vector<double> ragged;
        double total = 0.0;
        uint32_t rng = 12345u;
        for (int i = 0; i < 2000; ++i) {
            rng = rng * 1664525u + 1013904223u;
            const double d = 0.002 + (rng >> 8) % 40000 * 1e-6;   // 2-42 ms
            ragged.push_back(d);
            total += d;
        }
        const Run r = Drive(ragged);
        const double advanced = r.now - 1000.0;
        if (advanced > total || advanced < total - kDt) {
            Log("[simclock]   FAIL ragged frames: advanced %.6f s against %.6f s of wall time",
                advanced, total);
            ok = false;
        }
        ok &= Expect("ragged run shed nothing", static_cast<double>(r.shed), 0.0, 0.0);
    }

    // (4) timeScale scales the RATE, not the quantum: 2x speed over the same wall time is twice
    //     the quanta, each still exactly kDt of world.
    {
        const std::vector<double> f(100, 1.0 / 60.0);
        const Run one = Drive(f, 1.0), two = Drive(f, 2.0);
        ok &= Expect("2x timeScale is 2x the quanta", static_cast<double>(two.steps),
                     static_cast<double>(one.steps) * 2.0, 1.0);
    }

    // (5) A hitch SHEDS rather than spiralling. A ten-second stall must not buy the next frames
    //     a debt of 2400 steps to repay.
    {
        SimClock c;
        c.Reset(1000.0);
        const int steps = c.Advance(10.0, 1.0);
        ok &= Expect("a 10 s hitch is capped", static_cast<double>(steps),
                     static_cast<double>(SimClock::kMaxStepsPerFrame), 0.0);
        ok &= Expect("the rest is shed, not owed", static_cast<double>(c.Shed()),
                     2400.0 - SimClock::kMaxStepsPerFrame, 0.0);
        const int next = c.Advance(1.0 / 60.0, 1.0);
        ok &= Expect("the frame after a hitch is ordinary", static_cast<double>(next), 4.0, 0.0);
    }

    // (6) A deliberate jump re-bases without smearing the partial quantum across it.
    {
        SimClock c;
        c.Reset(1000.0);
        c.Advance(0.001, 1.0);   // sub-quantum: accumulates, does not step
        c.SetTo(50000.0);
        ok &= Expect("SetTo lands exactly", c.Now(), 50000.0, 0.0);
        ok &= Expect("SetTo drops the partial quantum", c.Alpha(), 0.0, 0.0);
    }

    Log("[simclock] ---- %s: the clock advances in whole quanta, the same interval advances the "
        "world identically however it is framed, a hitch sheds instead of spiralling ----",
        ok ? "PASS" : "FAIL");
    return ok;
}

}  // namespace ga
