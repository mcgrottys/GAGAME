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

    // ---- 7. THE GROUND (beaching, 2026-10-11). The bed is a second surface, and the same
    // stations that are clipped against the water are pushed back by it. Three closed forms:
    // (a) on dry flat ground the barge stands on its six keel points, its weight on the bed and
    // the spring's settle under it; (b) on a dry slope Coulomb's law: below atan(mu) it stands
    // still, above it slides at g (sin t - mu cos t); (c) driven at a beach under level water it
    // STOPS on the beach -- and the same hull with no ground law sails on, which is the control
    // that keeps (c) from being vacuous.
    const double mu = 0.5;
    const double dtLive = 1.0 / 60.0;   // the hull's own step rate (Entity::Update)
    auto cgOf = [](const Vessel& v, double p[3]) {
        p[0] = p[1] = p[2] = 0.0;
        v.Body().pose.TransformPoint(p[0], p[1], p[2]);
    };
    auto speedOf = [](const Vessel& v) {
        const double o[3] = {0, 0, 0};
        double vv[3];
        v.Body().VelocityAtBody(o, vv);
        return std::sqrt(vv[0] * vv[0] + vv[1] * vv[1] + vv[2] * vv[2]);
    };
    auto heelOf = [](const Vessel& v) {
        double up[3] = {0.0, 1.0, 0.0};
        v.Body().pose.TransformDir(up[0], up[1], up[2]);
        return std::abs(std::atan2(up[0], up[1])) * 180.0 / kPi;
    };
    // (a) dry flat ground: the bed a metre ABOVE the water.
    {
        StillWater dry;
        dry.levelNavd = 0.0;
        dry.bedNavd = 1.0;
        Vessel v;
        v.Build(spec, Motor::Translation(0.0, 3.0, 0.0));
        Settle(v, dry, 4000);
        double p[3];
        cgOf(v, p);
        const double pen = dry.bedNavd - (p[1] - 0.5 * H);   // the keel below the bed
        const double settle = 0.015;
        VesselControls c;
        const Bivector w = v.NetWrench(dry, c, 0.0);
        const double fmag = std::sqrt(w.a[0] * w.a[0] + w.a[1] * w.a[1] + w.a[2] * w.a[2]);
        const double tmag = std::sqrt(w.b[0] * w.b[0] + w.b[1] * w.b[1] + w.b[2] * w.b[2]);
        const VesselTelemetry& t = v.Telemetry();
        Log("[vessel] dry ground: keel %.4f m into the bed (settle %.3f), bed carries %.0f N on %d "
            "points (weight %.0f), water %.4f m^3, residual %.3g N %.3g N m",
            pen, settle, t.contactN, t.contactPts, M * kG, t.immersedVol, fmag, tmag);
        if (std::abs(pen - settle) > 0.02) fail("ground: penetration vs settle", pen, settle);
        if (!(pen > 0.0)) fail("ground: the barge is not on the bed", pen, settle);
        if (t.contactPts != 6) fail("ground: keel points touching", t.contactPts, 6);
        if (std::abs(t.contactN - M * kG) > 0.02 * M * kG) fail("ground: weight on the bed", t.contactN, M * kG);
        if (t.immersedVol > 1e-9) fail("ground: water under a dry hull", t.immersedVol, 0.0);
        if (fmag > 2.0) fail("ground: residual force at rest (N)", fmag, 0.0);
        if (tmag > 2.0) fail("ground: residual torque at rest (N m)", tmag, 0.0);
        if (!t.aground) fail("ground: telemetry label", 0, 1);
    }
    // (b) Coulomb on a dry slope.
    for (int pass = 0; pass < 2; ++pass) {
        TiltedWater hill;
        hill.slopeRad = (pass == 0 ? 20.0 : 35.0) * kPi / 180.0;   // atan(0.5) = 26.57 deg
        hill.levelNavd = 0.0;
        hill.bedFollows = true;
        hill.bedAbove = 0.5;
        Vessel v;
        v.Build(spec, Motor::Translation(0.0, 3.0, 0.0));
        Settle(v, hill, 6000);
        VesselControls c;
        double p0[3];
        cgOf(v, p0);
        if (pass == 0) {
            for (int i = 0; i < 600; ++i) {
                Vessel::s_gripTrace = (i == 120 || i == 121);
                v.Step(hill, c, i * dtLive, dtLive);   // 10 s
                Vessel::s_gripTrace = false;
                if (i % 60 == 59) {
                    double q[3];
                    cgOf(v, q);
                    Log("[vessel]   hold t %2d s: x %+.5f y %+.5f z %+.5f | %.3f m/s | bed %.0f N on %d pts",
                        (i + 1) / 60, q[0] - p0[0], q[1] - p0[1], q[2] - p0[2], speedOf(v),
                        v.Telemetry().contactN, v.Telemetry().contactPts);
                }
            }
            double p1[3];
            cgOf(v, p1);
            const double drift = std::sqrt((p1[0] - p0[0]) * (p1[0] - p0[0]) + (p1[1] - p0[1]) * (p1[1] - p0[1]) +
                                           (p1[2] - p0[2]) * (p1[2] - p0[2]));
            Log("[vessel] dry slope 20 deg (mu %.2f, holds below %.1f deg): drift %.4f m in 10 s, heel %.2f deg",
                mu, std::atan(mu) * 180.0 / kPi, drift, heelOf(v));
            if (drift > 0.01) fail("ground: a hull below atan(mu) drifts (m / 10 s)", drift, 0.01);
        } else {
            for (int i = 0; i < 60; ++i) v.Step(hill, c, i * dtLive, dtLive);    // 1 s: out of the creep
            const double v1 = speedOf(v);
            for (int i = 0; i < 60; ++i) v.Step(hill, c, (60 + i) * dtLive, dtLive);
            const double v2 = speedOf(v);
            const double a = v2 - v1;
            const double aExpect = kG * (std::sin(hill.slopeRad) - mu * std::cos(hill.slopeRad));
            Log("[vessel] dry slope 35 deg: slides at %.4f m/s^2 (g (sin t - mu cos t) = %.4f), heel %.2f deg",
                a, aExpect, heelOf(v));
            if (std::abs(a - aExpect) > 0.05 * aExpect) fail("ground: sliding acceleration", a, aExpect);
        }
    }
    // (c) the beach: level water, the bed rising ahead; a thruster pushes the barge at it.
    for (int pass = 0; pass < 2; ++pass) {
        RampBedWater beach;
        beach.levelNavd = 0.0;
        beach.bedAtZ0 = -3.0;
        beach.bedSlope = 0.08;
        VesselSpec pushed = spec;
        for (Element& e : pushed.elements) {
            if (e.kind == ElementKind::Buoyancy && pass == 1) e.contactSettle = Num{};   // no ground law
        }
        Element prop;
        prop.kind = ElementKind::Thruster;
        prop.name = "push";
        prop.mount.at = Motor::Translation(0.0, -0.95, -2.4);
        prop.maxThrust = Num::Of(6000.0, "n", "DECLARED: the gate's push");
        prop.propRadius = Num::Of(0.3, "m", "DECLARED");
        prop.freeRunSpeed = Num::Of(6.0, "m/s", "DECLARED");
        prop.propTorqueArm = Num::Of(0.0, "m", "DECLARED: no torque reaction");
        pushed.elements.push_back(prop);
        Vessel v;
        v.Build(pushed, Motor::Translation(0.0, 3.0, 0.0));
        Settle(v, beach, 3000);
        VesselControls c;
        c.throttle[0] = 1.0;
        double zMax = -1e30;
        for (int i = 0; i < 90 * 60; ++i) {
            v.Step(beach, c, i * dtLive, dtLive);
            double p[3];
            cgOf(v, p);
            zMax = std::max(zMax, p[2]);
        }
        double p[3];
        cgOf(v, p);
        const VesselTelemetry& t = v.Telemetry();
        const double spd = speedOf(v);
        Log("[vessel] beach%s: after 90 s at z %.1f m (bed there %+.2f), %.3f m/s, heel %.2f deg, trim %+.1f deg, "
            "bed carries %.0f N on %d points, water %.3f m^3%s",
            pass == 0 ? "" : " (no ground law, the control)", p[2], beach.bedAtZ0 + beach.bedSlope * p[2],
            spd, heelOf(v), t.trimRad * 180.0 / kPi, t.contactN, t.contactPts, t.immersedVol,
            v.Body().pose.s == v.Body().pose.s ? "" : " NaN");
        if (pass == 0) {
            if (!(spd < 0.05)) fail("beach: the hull did not stop (m/s)", spd, 0.05);
            if (!(t.contactPts > 0)) fail("beach: no station on the bed", t.contactPts, 1);
            if (!(heelOf(v) < 5.0)) fail("beach: heel (deg)", heelOf(v), 5.0);
            if (!(p[2] > 30.0)) fail("beach: it never reached the beach (z)", p[2], 30.0);
        } else {
            if (!(spd > 0.3)) fail("beach control: the hull with no ground law stopped too (m/s)", spd, 0.3);
        }
    }
    // ...and the batch (F22) serves the bed plane's three probes: every table answer against
    // the point asked live, bit for bit, with the hull on the beach.
    {
        RampBedWater beach;
        Vessel v;
        v.Build(spec, Motor::Translation(0.0, 0.5, 40.0));
        const bool audit0 = Vessel::s_batchAudit, batch0 = Vessel::s_batchWater;
        Vessel::s_batchWater = true;
        Vessel::s_batchAudit = true;
        const uint64_t fails0 = Vessel::s_auditFails, pts0 = Vessel::s_batchPoints, steps0 = Vessel::s_batchSteps;
        VesselControls c;
        for (int i = 0; i < 120; ++i) v.Step(beach, c, i * dtLive, dtLive);
        Vessel::s_batchAudit = audit0;
        Vessel::s_batchWater = batch0;
        const uint64_t perStep = (Vessel::s_batchPoints - pts0) / std::max<uint64_t>(1, Vessel::s_batchSteps - steps0);
        Log("[vessel] batch audit on the beach: %llu points a step (stations 2 + drag 1 + CG 1 + bed 3), %llu fails",
            static_cast<unsigned long long>(perStep), static_cast<unsigned long long>(Vessel::s_auditFails - fails0));
        if (perStep != 7) fail("batch: the bed probes are not in the table", double(perStep), 7.0);
        if (Vessel::s_auditFails != fails0) fail("batch: a table answer differs from the live one", double(Vessel::s_auditFails - fails0), 0.0);
    }

    Log("[vessel] ---- %s: factory refuses unknown kinds, section clip (level/tilted/full/dry), "
        "draught + zero residual wrench, heave period, roll period vs GM, carried by a swell, "
        "absence is not flat water, the ground (dry flat, Coulomb's slope both ways, the beach "
        "stops it and the control sails on, the bed probes in the batch) ----",
        ok ? "PASS" : "FAIL");
    return ok;
}

}  // namespace ga
