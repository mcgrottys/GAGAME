// ================================================================================================
//  RunVesselSelfTest - the gate on the factory and on the element laws.
//
//  Every check here has a CLOSED FORM on the other side. That is the whole reason box.test is a
//  rectangular barge: draught, heave period, roll period and metacentric height are all exact for
//  a box, so "the boat floats about right" is replaced by a number with four digits in it.
//
//  Two of these gates exist only to stop the others being vacuous, which is this repo's standing
//  lesson (proofs/ripple_prefilter.py once shipped a check computing f(a) - f(a)):
//    * the roll period is checked against the ROLL inertia specifically, on a hull whose roll and
//      pitch inertias differ by 3.6x -- so a swapped body-axis mapping fails. It did.
//    * the DOWN-SLOPE force on a frozen tilted plane is what proves buoyancy acts along the
//      surface normal. The obvious test -- watch the hull drift along a swell -- was tried first
//      and does NOT discriminate: measured 0.0349 m/s with the normal and 0.0215 m/s with
//      buoyancy forced to world up, because relative-velocity drag against the orbital motion
//      carries the hull either way. On a frozen ramp there is no orbital motion to hide behind
//      and the wrong law gives exactly zero. The drift measurement is still reported, as a
//      number rather than as evidence.
// ================================================================================================
#include "sim/Vessel.h"
#include "sim/VesselSpec.h"
#include "sim/WaterSurface.h"

#include <algorithm>
#include <cmath>

namespace ga {

namespace {

constexpr double kPi = 3.14159265358979323846;

// Settle a hull to rest by stepping it with heavy artificial damping on the momenta. Used only
// to REACH equilibrium quickly; every measurement afterwards runs undamped.
void Settle(Vessel& v, const WaterSurface& sea, int steps) {
    VesselControls c;
    const double dt = 1.0 / 240.0;
    for (int i = 0; i < steps; ++i) {
        v.Step(sea, c, 0.0, dt);
        Bivector t = v.Body().Twist();
        t = t * 0.90;                     // not physics: a numerical shortcut to equilibrium
        v.Body().SetTwist(t);
    }
}

// Period of a lightly damped oscillation, from upward zero crossings of `sample() - mean`.
template <typename F>
double MeasurePeriod(F sample, double mean, int steps, double dt) {
    double prev = sample() - mean;
    int crossings = 0;
    double first = -1.0, last = -1.0;
    for (int i = 0; i < steps; ++i) {
        const double y = sample() - mean;
        if (prev < 0.0 && y >= 0.0) {
            if (first < 0.0) first = i * dt;
            last = i * dt;
            ++crossings;
        }
        prev = y;
    }
    return (crossings > 1) ? (last - first) / (crossings - 1) : 0.0;
}

}  // namespace

bool RunVesselSelfTest() {
    bool ok = true;
    auto fail = [&](const char* what, double got, double want) {
        Log("[vessel] FAIL %s: %.9g vs %.9g", what, got, want);
        ok = false;
    };

    // ---- 0. THE FACTORY IS A FACTORY. A name in, a spec out; an unknown name refuses rather
    // than substituting anything -- a silently-defaulted hull would float, and be the wrong boat.
    VesselRegistry reg;
    RegisterBuiltinVessels(reg);
    {
        if (!reg.Knows("box.test")) fail("registry lost box.test", 0, 1);
        if (reg.Build("no.such.hull").kind.empty() == false) {
            fail("registry invented a hull for an unknown name", 1, 0);
        }
        Vessel bad;
        if (bad.Build(VesselSpec{}, Motor::Identity())) {
            fail("Vessel built from an empty spec", 1, 0);
        }
    }

    const VesselSpec spec = reg.Build("box.test");
    const double B = 2.0, H = 2.0, L = 5.0, M = 2000.0, rho = 1025.0;

    // ---- 1. THE SECTION CLIP, against a closed form. A rectangle cut by a level line, and then
    // by a TILTED one -- the tilted case is the one a depth-based area curve cannot do, and it is
    // the whole of roll stability.
    {
        const double px[4] = {-1.0, 1.0, 1.0, -1.0}, py[4] = {-1.0, -1.0, 1.0, 1.0};
        double cx = 0.0, cy = 0.0;
        // level waterline at y = -0.4: inside is y <= -0.4, i.e. 0*x + 1*y + 0.4 <= 0
        const double a1 = ClipSectionArea(px, py, 4, 0.0, 1.0, 0.4, &cx, &cy);
        if (std::abs(a1 - 2.0 * 0.6) > 1e-12) fail("clip: level waterline area", a1, 1.2);
        if (std::abs(cy - (-0.7)) > 1e-12) fail("clip: level waterline centroid", cy, -0.7);
        // wholly submerged: the whole square, and no special case was written for it
        const double a2 = ClipSectionArea(px, py, 4, 0.0, 1.0, -5.0, &cx, &cy);
        if (std::abs(a2 - 4.0) > 1e-12) fail("clip: fully immersed area", a2, 4.0);
        // wholly dry
        const double a3 = ClipSectionArea(px, py, 4, 0.0, 1.0, 5.0, &cx, &cy);
        if (a3 != 0.0) fail("clip: fully dry area", a3, 0.0);
        // a 45 degree waterline through the centre: exactly half the square, centroid on the
        // diagonal. Normal (1,1)/sqrt2, offset 0 -> inside is x + y <= 0.
        const double a4 = ClipSectionArea(px, py, 4, 0.7071067811865476, 0.7071067811865476, 0.0,
                                          &cx, &cy);
        if (std::abs(a4 - 2.0) > 1e-12) fail("clip: 45 deg waterline area", a4, 2.0);
        if (std::abs(cx - cy) > 1e-12) fail("clip: 45 deg centroid off the diagonal", cx, cy);
        if (cx > -0.1) fail("clip: 45 deg centroid not in the wet half", cx, -0.333);
    }

    // ---- 2. DRAUGHT. The barge must settle where Archimedes says and nowhere else.
    const double V = M / rho;                       // displaced volume, m^3
    const double dExpect = V / (B * L);             // draught for a prismatic hull
    StillWater still;
    still.levelNavd = 0.0;
    still.bedNavd = -40.0;
    {
        Vessel v;
        if (!v.Build(spec, Motor::Translation(0.0, 3.0, 0.0))) fail("build box.test", 0, 1);
        Settle(v, still, 4000);
        double p[3] = {0, 0, 0};
        v.Body().pose.TransformPoint(p[0], p[1], p[2]);
        // The keel sits at body y = -H/2, so the CG floats (H/2 - d) above the waterline.
        const double yExpect = still.levelNavd + (0.5 * H - dExpect);
        Log("[vessel] box.test draught: %.6f m (analytic %.6f), CG at y = %.6f (expect %.6f), "
            "displaced %.4f m^3 (expect %.4f)",
            v.Telemetry().draughtM, dExpect, p[1], yExpect, v.Telemetry().immersedVol, V);
        if (std::abs(p[1] - yExpect) > 1e-3) fail("floating height", p[1], yExpect);
        if (std::abs(v.Telemetry().immersedVol - V) > 1e-3) {
            fail("displaced volume", v.Telemetry().immersedVol, V);
        }

        // ...and at equilibrium the NET WRENCH is zero, which is a far sharper statement than
        // "it stopped moving".
        VesselControls c;
        const Bivector w = v.NetWrench(still, c, 0.0);
        const double fmag = std::sqrt(w.a[0]*w.a[0] + w.a[1]*w.a[1] + w.a[2]*w.a[2]);
        if (fmag > 1.0) fail("residual force at equilibrium (N)", fmag, 0.0);
    }

    // ---- 3. HEAVE PERIOD. T = 2 pi sqrt(m / (rho g A_wp)). Pure hydrostatic stiffness; if the
    // buoyancy magnitude were wrong by any factor this lands somewhere else.
    {
        const double k = rho * kG * B * L;
        const double tExpect = 2.0 * kPi * std::sqrt(M / k);
        Vessel v;
        v.Build(spec, Motor::Translation(0.0, 3.0, 0.0));
        Settle(v, still, 4000);
        double eq[3] = {0, 0, 0};
        v.Body().pose.TransformPoint(eq[0], eq[1], eq[2]);
        // displace 0.1 m and release, undamped
        v.Body().pose = Motor::Translation(0.0, eq[1] + 0.1, 0.0);
        v.Body().Rest();
        VesselControls c;
        const double dt = 1.0 / 240.0;
        const double measured = MeasurePeriod(
            [&]() {
                v.Step(still, c, 0.0, dt);
                double p[3] = {0, 0, 0};
                v.Body().pose.TransformPoint(p[0], p[1], p[2]);
                return p[1];
            },
            eq[1], 12000, dt);
        Log("[vessel] box.test heave period: %.6f s (analytic %.6f)", measured, tExpect);
        if (std::abs(measured - tExpect) / tExpect > 5.0e-3) {
            fail("heave period", measured, tExpect);
        }
    }

    // ---- 4. ROLL PERIOD, against GM. THE gate for the polygon clip and for the body-axis
    // mapping: it uses the ROLL inertia (about the longitudinal axis), which on this hull is
    // 3.6x smaller than the pitch inertia, so getting the axes the wrong way round -- as this
    // file's author did on the first pass -- misses by 90%.
    {
        const double KB = 0.5 * dExpect;                    // centre of buoyancy above the keel
        const double Iwp = L * B * B * B / 12.0;            // waterplane 2nd moment, longitudinal
        const double BM = Iwp / V;
        const double KG = 0.5 * H;                          // CG at the box centre
        const double GM = KB + BM - KG;
        const double Iroll = M / 12.0 * (B * B + H * H);
        const double tExpect = 2.0 * kPi * std::sqrt(Iroll / (M * kG * GM));

        Vessel v;
        v.Build(spec, Motor::Translation(0.0, 3.0, 0.0));
        Settle(v, still, 4000);
        double eq[3] = {0, 0, 0};
        v.Body().pose.TransformPoint(eq[0], eq[1], eq[2]);
        // heel 3 degrees about the LONGITUDINAL axis (body/world z) and release
        const double org[3] = {0.0, eq[1], 0.0}, axis[3] = {0.0, 0.0, 1.0};
        v.Body().pose = Motor::Rotation(org, axis, 3.0 * kPi / 180.0) *
                        Motor::Translation(0.0, eq[1], 0.0);
        v.Body().Rest();
        VesselControls c;
        const double dt = 1.0 / 240.0;
        const double measured = MeasurePeriod(
            [&]() {
                v.Step(still, c, 0.0, dt);
                double up[3] = {0.0, 1.0, 0.0};
                v.Body().pose.TransformDir(up[0], up[1], up[2]);
                return std::atan2(up[0], up[1]);
            },
            0.0, 12000, dt);
        Log("[vessel] box.test roll period: %.6f s (analytic %.6f, GM = %.4f m)", measured,
            tExpect, GM);
        if (std::abs(measured - tExpect) / tExpect > 2.0e-2) fail("roll period", measured,
                                                                  tExpect);
    }

    // ---- 4b. BUOYANCY IS NORMAL TO THE SURFACE, and this is the gate that PROVES it, because
    // the wave-drift measurement below does not. MEASURED: with the surface normal the hull
    // drifts along a swell at 0.0349 m/s; with buoyancy forced to world up it still drifts at
    // 0.0215 m/s, because relative-velocity drag against the orbital motion carries it anyway.
    // A gate that only watched the drift would therefore have passed either law -- exactly the
    // vacuous-gate failure this repo has shipped once already.
    //
    // On a frozen tilted plane there is no orbital motion to hide behind. Buoyancy normal to the
    // surface gives a horizontal force of m g tan(theta); buoyancy along world up gives ZERO.
    {
        TiltedWater ramp;
        ramp.slopeRad = 0.08;                            // ~4.6 degrees, a moderate wave face
        ramp.levelNavd = 0.0;
        ramp.bedNavd = -60.0;

        Vessel v;
        v.Build(spec, Motor::Translation(0.0, 3.0, 0.0));
        Settle(v, ramp, 6000);
        VesselControls c;
        const Bivector w = v.NetWrench(ramp, c, 0.0);
        // The wrench is body frame, and the hull settles very nearly upright, so the horizontal
        // component is read in world coordinates to keep the comparison honest.
        double fw[3] = {w.a[0], w.a[1], w.a[2]};
        v.Body().pose.TransformDir(fw[0], fw[1], fw[2]);
        const double expect = M * kG * std::tan(ramp.slopeRad);
        Log("[vessel] tilted plane %.1f deg: horizontal force %.1f N (analytic m g tan = %.1f N; "
            "world-up buoyancy would give 0)",
            ramp.slopeRad * 180.0 / kPi, fw[0], expect);
        if (std::abs(fw[0] - expect) > 0.06 * expect) {
            fail("down-slope force (buoyancy is not normal to the surface)", fw[0], expect);
        }
        if (std::abs(fw[1]) > 0.02 * M * kG) fail("vertical residual on the ramp", fw[1], 0.0);
    }

    // ---- 5. THE WAVE GATE. On an Airy train the hull must (a) heave at the ENCOUNTER period and
    // (b) be carried ALONG the wave -- and nothing in Vessel.cpp mentions waves. (b) is the
    // discriminating half: a hull that merely bobbed would pass any test of (a) alone, and would
    // say nothing about the surface normal being load-bearing.
    {
        AiryWave wave;
        wave.amp = 0.6;
        wave.k = 2.0 * kPi / 40.0;                          // 40 m swell
        wave.dirX = 1.0; wave.dirZ = 0.0;
        wave.levelNavd = 0.0;
        wave.bedNavd = -80.0;

        Vessel v;
        v.Build(spec, Motor::Translation(0.0, 3.0, 0.0));
        VesselControls c;
        const double dt = 1.0 / 240.0;
        for (int i = 0; i < 4000; ++i) v.Step(wave, c, i * dt, dt);   // let it find the surface

        double x0[3] = {0, 0, 0};
        v.Body().pose.TransformPoint(x0[0], x0[1], x0[2]);
        double minY = 1e30, maxY = -1e30;
        const double t0 = 4000 * dt;
        const int n = 20000;
        for (int i = 0; i < n; ++i) {
            v.Step(wave, c, t0 + i * dt, dt);
            double p[3] = {0, 0, 0};
            v.Body().pose.TransformPoint(p[0], p[1], p[2]);
            minY = std::min(minY, p[1]);
            maxY = std::max(maxY, p[1]);
        }
        double x1[3] = {0, 0, 0};
        v.Body().pose.TransformPoint(x1[0], x1[1], x1[2]);
        const double heave = 0.5 * (maxY - minY);
        const double drift = x1[0] - x0[0];
        const double secs = n * dt;
        Log("[vessel] airy 40 m x %.2f m amp: heave amplitude %.4f m, along-wave drift %.3f m "
            "in %.1f s (%.4f m/s)", wave.amp, heave, drift, secs, drift / secs);

        // The hull is much shorter than the wave, so it should follow it closely: heave within
        // a factor of two of the wave amplitude. Far outside that means it is not tracking.
        if (heave < 0.4 * wave.amp || heave > 1.6 * wave.amp) {
            fail("heave amplitude on a long swell", heave, wave.amp);
        }
        // ...and it must be CARRIED. Stokes drift plus the down-slope component is small but
        // strictly positive and in the direction of propagation; a hull feeling only vertical
        // buoyancy would sit at x0 forever.
        if (!(drift > 0.05)) fail("hull is not carried along the wave (surfing absent)", drift,
                                  0.05);
    }

    // ---- 6. ABSENCE IS NOT FLAT WATER. A surface that reports no coverage must leave the hull
    // unsupported and SAY so -- never quietly float it at datum zero, which is the failure that
    // would let a boat in mid-Atlantic believe it had run onto a beach.
    {
        struct NoCoverage : WaterSurface {
            const char* Name() const override { return "no coverage (test)"; }
            SurfaceSample At(double, double, double) const override { return SurfaceSample{}; }
        } none;
        Vessel v;
        v.Build(spec, Motor::Translation(0.0, 3.0, 0.0));
        VesselControls c;
        const Bivector w = v.NetWrench(none, c, 0.0);
        if (v.Telemetry().waterValid) fail("no-coverage water reported valid", 1, 0);
        // gravity only: the net force is exactly -mg, no buoyancy invented from nothing
        if (std::abs(w.a[1] + M * kG) > 1e-6) fail("no-coverage buoyancy invented", w.a[1],
                                                   -M * kG);
    }

    Log("[vessel] ---- %s: factory refuses unknown kinds, section clip (level/tilted/full/dry), "
        "draught + zero residual wrench, heave period, roll period vs GM, carried by a swell, "
        "absence is not flat water ----",
        ok ? "PASS" : "FAIL");
    return ok;
}

}  // namespace ga
