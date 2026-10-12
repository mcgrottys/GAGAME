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

    // ---- 8. THE DRONE (Hover). A velocity servo and an attitude servo with one time constant:
    // a commanded 5 m/s is reached within 3 tau, a 60 s hover with no command drifts less than a
    // centimetre, and a 30 deg tilt is level again inside 2 s. It asks the water nothing, so the
    // surface under it is whatever is to hand.
    {
        const VesselSpec drone = reg.Build("drone.recovery");
        if (drone.kind.empty() || !drone.recovery) fail("drone: not registered as the recovery body", 0, 1);
        const double tau = 0.35;
        // (a) 5 m/s forward commanded: both levers at 5/12 of the 12 m/s horizontal speed.
        {
            Vessel v;
            v.Build(drone, Motor::Translation(0.0, 10.0, 0.0));
            VesselControls c;
            c.throttle[0] = c.throttle[1] = 5.0 / 12.0;
            double reached = -1.0;
            for (int i = 0; i < 600; ++i) {
                v.Step(still, c, i * dtLive, dtLive);
                if (reached < 0.0 && speedOf(v) >= 0.95 * 5.0) reached = (i + 1) * dtLive;
            }
            double p[3];
            cgOf(v, p);
            Log("[vessel] drone: 5 m/s commanded, 95%% reached at %.2f s (3 tau = %.2f), after 10 s %.3f m/s "
                "at y %+.3f (started +10.000), heel %.2f deg", reached, 3.0 * tau, speedOf(v), p[1], heelOf(v));
            if (!(reached > 0.0 && reached <= 3.0 * tau)) fail("drone: 5 m/s not reached within 3 tau (s)", reached, 3.0 * tau);
            if (std::abs(p[1] - 10.0) > 0.05) fail("drone: it did not hold its height under way (m)", p[1], 10.0);
        }
        // (b) 60 s hover, no command.
        {
            Vessel v;
            v.Build(drone, Motor::Translation(3.0, 10.0, -7.0));
            VesselControls c;
            for (int i = 0; i < 3600; ++i) v.Step(still, c, i * dtLive, dtLive);
            double p[3];
            cgOf(v, p);
            const double drift = std::sqrt((p[0] - 3.0) * (p[0] - 3.0) + (p[1] - 10.0) * (p[1] - 10.0) + (p[2] + 7.0) * (p[2] + 7.0));
            Log("[vessel] drone: 60 s hover drifts %.5f m", drift);
            if (drift > 0.01) fail("drone: hover drift in 60 s (m)", drift, 0.01);
        }
        // (c) level from 30 deg of roll.
        {
            Vessel v;
            const double o[3] = {0.0, 10.0, 0.0}, ax[3] = {0.0, 0.0, 1.0};
            v.Build(drone, Motor::Rotation(o, ax, 30.0 * kPi / 180.0) * Motor::Translation(0.0, 10.0, 0.0));
            VesselControls c;
            double t1 = -1.0;
            for (int i = 0; i < 180; ++i) {
                v.Step(still, c, i * dtLive, dtLive);
                if (t1 < 0.0 && heelOf(v) < 1.0) t1 = (i + 1) * dtLive;
            }
            Log("[vessel] drone: 30 deg roll is under 1 deg at %.2f s, %.3f deg at 3 s", t1, heelOf(v));
            if (!(t1 > 0.0 && t1 < 2.0)) fail("drone: level recovery (s)", t1, 2.0);
        }
    }

    // ---- 9. THE CAR (Wheel, car.baja). Four closed forms: (a) at rest on flat dry ground every
    // wheel carries its static share of the weight (the CG's split between the axles) at the
    // compression that share puts into the progressive spring, and the body stands still; (b) a
    // 20 deg hill, nose down, brake on: it holds; (c) full throttle from rest on the flat for 5 s
    // reaches the drive law's own closed form, v = vMax (1 - exp(-F0 t / (m vMax))), less the
    // rolling and air losses, with every wheel on the ground and no yaw; (d) a step steer while
    // coasting: the yaw rate over the speed is Ackermann's tan(delta) / L, which a car whose
    // cornering stiffness is its load's (neutral steer) must meet.
    {
        const VesselSpec car = reg.Build("car.baja");
        if (car.kind.empty()) fail("car: not registered", 0, 1);
        const double W = (car.massLoaded.v > 0.0 ? car.massLoaded.v : car.massDry.v) * kG;
        auto wheelRoot = [](const Element& e, double load) {   // k x (1 + (x/T)^2) = load, Newton
            const double T = e.suspTravel.v, k = e.suspK.v;
            double x = load / k;
            for (int i = 0; i < 40; ++i) {
                const double f = k * x * (1.0 + x * x / (T * T)) - load;
                x -= f / (k * (1.0 + 3.0 * x * x / (T * T)));
            }
            return x;
        };
        auto seat = [&](Vessel& v, const WaterSurface& ground, const Motor& pose) {
            v.Build(car, pose);
            Settle(v, ground, 4800);
            VesselControls c;
            for (int i = 0; i < 180; ++i) v.Step(ground, c, i * dtLive, dtLive);   // 3 s undamped
        };
        StillWater dry;
        dry.levelNavd = 0.0;
        dry.bedNavd = 1.0;
        // (a) at rest on the flat.
        {
            Vessel v;
            seat(v, dry, Motor::Translation(0.0, 1.0 + 1.2, 0.0));
            double zF = 0.0, zR = 0.0;
            for (const Element& e : v.Spec().elements) {
                if (e.kind != ElementKind::Wheel) continue;
                double p[3];
                e.mount.Origin(0.0, p);
                (e.mount.jointed ? zF : zR) = p[2];
            }
            const double shareF = (0.0 - zR) / (zF - zR);   // the CG is the body origin
            size_t wi = 0;
            for (const Element& e : v.Spec().elements) {
                if (e.kind != ElementKind::Wheel) continue;
                const Vessel::WheelState& ws = v.Wheels()[wi++];
                const double want = (e.mount.jointed ? shareF : 1.0 - shareF) * 0.5 * W;
                const double xWant = wheelRoot(e, want);
                Log("[vessel] car at rest: %s load %.0f N (static share %.0f), compression %.4f m (the spring's %.4f)",
                    e.name.c_str(), ws.load, want, ws.comp, xWant);
                if (std::abs(ws.load - want) > 0.02 * want) fail("car: a wheel's static load (N)", ws.load, want);
                if (std::abs(ws.comp - xWant) > 0.02 * xWant) fail("car: a wheel's static compression (m)", ws.comp, xWant);
            }
            const VesselTelemetry& t = v.Telemetry();
            Log("[vessel] car at rest: the ground carries %.0f N of %.0f on %d wheels, %.4f m/s, heel %.3f deg, trim %.3f deg",
                t.contactN, W, t.contactPts, speedOf(v), heelOf(v), t.trimRad * 180.0 / kPi);
            if (std::abs(t.contactN - W) > 0.01 * W) fail("car: the ground carries the weight (N)", t.contactN, W);
            if (t.contactPts != 4) fail("car: wheels on the ground", t.contactPts, 4);
            if (speedOf(v) > 2e-3) fail("car: at rest (m/s)", speedOf(v), 0.0);
        }
        // (b) the hill, nose down (+x is downhill on TiltedWater), brake on.
        {
            TiltedWater hill;
            hill.slopeRad = 20.0 * kPi / 180.0;
            hill.levelNavd = 0.0;
            hill.bedFollows = true;
            hill.bedAbove = 0.5;
            const double o[3] = {0.0, 0.0, 0.0}, upY[3] = {0.0, 1.0, 0.0};
            Vessel v;
            v.Build(car, Motor::Translation(0.0, 2.2, 0.0) * Motor::Rotation(o, upY, 0.5 * kPi));
            VesselControls c;
            c.tilt = 1.0;   // the pedal
            double pStart[3];
            cgOf(v, pStart);
            for (int i = 0; i < 1200; ++i) {   // 20 s: landed and settled
                v.Step(hill, c, i * dtLive, dtLive);
                if (i % 240 == 239) {
                    double q[3];
                    cgOf(v, q);
                    Log("[vessel]   car hill t %2d s: moved %+.4f along x, %+.4f up | %.4f m/s | %d wheels %.0f N",
                        (i + 1) / 60, q[0] - pStart[0], q[1] - pStart[1], speedOf(v), v.Telemetry().contactPts,
                        v.Telemetry().contactN);
                }
            }
            double p0[3];
            cgOf(v, p0);
            for (int i = 0; i < 600; ++i) v.Step(hill, c, (1200 + i) * dtLive, dtLive);   // 10 s held
            double p1[3];
            cgOf(v, p1);
            const double drift = std::sqrt((p1[0] - p0[0]) * (p1[0] - p0[0]) + (p1[1] - p0[1]) * (p1[1] - p0[1]) +
                                           (p1[2] - p0[2]) * (p1[2] - p0[2]));
            Log("[vessel] car on a 20 deg hill, brake on: drift %.4f m in 10 s, %.4f m/s, %d wheels carrying %.0f N",
                drift, speedOf(v), v.Telemetry().contactPts, v.Telemetry().contactN);
            if (drift > 0.01) fail("car: a braked car drifts down a hill (m / 10 s)", drift, 0.01);
            if (v.Telemetry().contactPts != 4) fail("car: wheels on the hill", v.Telemetry().contactPts, 4);
        }
        // (c) full throttle for 5 s on the flat, and (d) the step steer, from the same seat.
        {
            Vessel v;
            seat(v, dry, Motor::Translation(0.0, 1.0 + 1.2, 0.0));
            double F0 = 0.0, vMax = 0.0, L = 0.0, zF = 0.0, zR = 0.0;
            for (const Element& e : v.Spec().elements) {
                if (e.kind != ElementKind::Wheel) continue;
                F0 += e.driveForce.v * e.driven.v;
                vMax = e.driveSpeed.v;
                double p[3];
                e.mount.Origin(0.0, p);
                (e.mount.jointed ? zF : zR) = p[2];
            }
            L = zF - zR;
            const double m = v.Body().mp.mass;
            VesselControls c;
            c.throttle[0] = 1.0;
            for (int i = 0; i < 300; ++i) v.Step(dry, c, i * dtLive, dtLive);
            const double vGot = speedOf(v);
            const double vForm = vMax * (1.0 - std::exp(-F0 * 5.0 / (m * vMax)));
            const double yaw = std::abs(v.Telemetry().headingRad) * 180.0 / kPi;
            Log("[vessel] car, full throttle 5 s: %.2f m/s (the drive law alone %.2f, losses below it), yaw %.3f deg, %d wheels down",
                vGot, vForm, yaw, v.Telemetry().contactPts);
            if (!(vGot > 0.88 * vForm && vGot < vForm)) fail("car: 5 s of full throttle (m/s)", vGot, vForm);
            if (yaw > 1.0) fail("car: it pulled under power (deg)", yaw, 0.0);
            if (v.Telemetry().contactPts != 4) fail("car: a wheel left the ground under power", v.Telemetry().contactPts, 4);
            // (d) throttle off, a step steer of 0.04 rad (the helm's sign: D is negative), 3 s to
            // settle into the turn, then one second's mean of yaw rate over speed. 0.04 and not
            // more: at the 16 m/s the throttle run ends at, 0.08 rad asks 7.6 m/s^2 sideways, and
            // this lifted car's rollover threshold (g x half-track / CG height) is 8.7 -- it went
            // over, which is the physics answering, not the gate.
            c.throttle[0] = 0.0;
            c.steer = -0.04;
            const double delta = 0.04;
            for (int i = 0; i < 180; ++i) v.Step(dry, c, (300 + i) * dtLive, dtLive);
            double ratio = 0.0;
            for (int i = 0; i < 60; ++i) {
                v.Step(dry, c, (480 + i) * dtLive, dtLive);
                ratio += v.Body().Twist().a[1] / (std::max)(speedOf(v), 1e-6);
            }
            ratio /= 60.0;
            const double ackermann = std::tan(delta) / L;
            Log("[vessel] car, step steer %.2f rad at %.1f m/s: yaw rate / speed %.4f 1/m (Ackermann tan(delta)/L = %.4f), heel %.2f deg",
                delta, speedOf(v), ratio, ackermann, heelOf(v));
            if (std::abs(ratio - ackermann) > 0.10 * ackermann) fail("car: Ackermann yaw rate / speed (1/m)", ratio, ackermann);
        }
    }

    // ---- 9b. THE SHELL. A car on its roof and a car on its side on dry ground rest on their
    // shell -- they do not fall through the beach (the live failure: a rolled car had no ground law
    // but its tyres), their struts grazing the ground carry nothing enormous, and the body stays
    // sane. The roof is at a declared height: the CG stands above the ground by it.
    {
        const VesselSpec car = reg.Build("car.baja");
        StillWater dry;
        dry.levelNavd = 0.0;
        dry.bedNavd = 1.0;
        const double o[3] = {0.0, 0.0, 0.0}, zAx[3] = {0.0, 0.0, 1.0};
        for (int pass = 0; pass < 2; ++pass) {
            const double roll = (pass == 0) ? kPi : 0.5 * kPi;
            Vessel v;
            v.Build(car, Motor::Translation(0.0, 1.0 + 2.5, 0.0) * Motor::Rotation(o, zAx, roll));
            VesselControls c;
            double yMin = 1e30, yRebound = -1e30;
            bool landed = false, sane = true;
            for (int i = 0; i < 600; ++i) {
                v.Step(dry, c, i * dtLive, dtLive);
                double q[3];
                cgOf(v, q);
                yMin = (std::min)(yMin, q[1]);
                if (v.Telemetry().contactPts > 0) landed = true;
                if (landed && v.Telemetry().contactPts == 0) yRebound = (std::max)(yRebound, q[1]);
                sane = sane && v.Body().Sane();
            }
            Log("[vessel] car %s: after landing it left the ground to %.3f m over the ground (never: -inf)",
                pass == 0 ? "on its roof" : "on its side", yRebound - 1.0);
            if (yRebound - 1.0 > 1.5) fail("shell: the body bounced back up after landing (CG over ground, m)", yRebound - 1.0, 1.5);
            double q[3];
            cgOf(v, q);
            Log("[vessel] car %s: after 10 s the CG is %.3f m over the ground (lowest %.3f), %d points carrying %.0f N, heel %.1f deg, %.4f m/s",
                pass == 0 ? "on its roof" : "on its side", q[1] - 1.0, yMin - 1.0, v.Telemetry().contactPts,
                v.Telemetry().contactN, heelOf(v), speedOf(v));
            if (!(q[1] - 1.0 > 0.3)) fail("shell: the body fell into the ground (CG over ground, m)", q[1] - 1.0, 0.3);
            // The drop's rebound: with Hunt-Crossley damping the body does not leave the ground
            // again by more than a hand after it first lands (it took 135 kN and flew 7 m before).
            if (!(yMin > 0.0)) fail("shell: the landing's lowest point", yMin, 1.0);
            if (v.Telemetry().contactPts < 2) fail("shell: the body rests on the ground (points)", v.Telemetry().contactPts, 2);
            if (speedOf(v) > 0.05) fail("shell: at rest (m/s)", speedOf(v), 0.0);
            if (!sane) fail("shell: the body left the domain", 0, 1);
        }
    }

    // ---- 9c. THE SOLIDS. On dry flat ground, two prisms in the car's path: a wall 3 m tall and
    // a step 0.4 m tall. Driven at the wall at full throttle the car STOPS at it (the shell's
    // points pushed out through the wall's face; the CG never past the face less the nose); driven
    // at the step it CLIMBS it (the struts' rays take the step's top: the CG ends 0.4 m higher,
    // four wheels down). Same law, two surfaces; nothing in the physics knows a building.
    {
        const VesselSpec car = reg.Build("car.baja");
        StillWater dry;
        dry.levelNavd = 0.0;
        dry.bedNavd = 1.0;
        auto seatCar = [&](Vessel& v) {
            v.Build(car, Motor::Translation(0.0, 1.0 + 1.2, 0.0));
            Settle(v, dry, 4800);
            VesselControls c;
            for (int i = 0; i < 120; ++i) v.Step(dry, c, i * dtLive, dtLive);
        };
        // (a) the wall: 20 m wide, 10 m deep, its near face at z = +12 (the car's nose at +2.5).
        {
            Vessel v;
            seatCar(v);
            std::vector<SolidPrism> solids(1);
            solids[0].xz = {-10.0, 12.0, 10.0, 12.0, 10.0, 22.0, -10.0, 22.0};
            solids[0].yBottom = 1.0;
            solids[0].yTop = 4.0;
            solids[0].cx = 0.0; solids[0].cz = 17.0; solids[0].radius = 12.0;
            v.SetSolids(&solids);
            VesselControls c;
            c.throttle[0] = 1.0;
            double zMax = -1e30, vPeak = 0.0;
            bool sane = true;
            for (int i = 0; i < 480; ++i) {   // 8 s: at the wall by 1.5 s, held against it after
                v.Step(dry, c, i * dtLive, dtLive);
                double q[3];
                cgOf(v, q);
                zMax = (std::max)(zMax, q[2]);
                vPeak = (std::max)(vPeak, speedOf(v));
                sane = sane && v.Body().Sane();
            }
            double q[3];
            cgOf(v, q);
            Log("[vessel] car at a wall: peak %.1f m/s, the CG reached z %.2f (the face at 12.00, the nose 2.16 ahead of the CG), ends at z %.2f, %d points carrying %.0f N, heel %.1f deg",
                vPeak, zMax, q[2], v.Telemetry().contactPts, v.Telemetry().contactN, heelOf(v));
            // The nose (the shell's front station, 2.43 m from the datum) is 2.16 m ahead of the CG;
            // the shell gives 0.5 m at 8.5 m/s (a spring sized to sink 2 cm under the weight).
            if (!(zMax < 12.0 - 2.16 + 0.6)) fail("solids: the car went through the wall (CG z, m)", zMax, 12.0 - 2.16);
            if (!(q[2] < 12.0 - 2.16 + 0.1)) fail("solids: the car rests inside the wall (CG z, m)", q[2], 12.0 - 2.16);
            if (!(zMax > 8.5)) fail("solids: the car never reached the wall (CG z, m)", zMax, 9.5);
            if (!sane) fail("solids: the body left the domain at the wall", 0, 1);
        }
        // (b) the step: 0.4 m tall from z = +8 on, 20 m wide and 40 m deep.
        {
            Vessel v;
            seatCar(v);
            double q0[3];
            cgOf(v, q0);
            std::vector<SolidPrism> solids(1);
            solids[0].xz = {-10.0, 8.0, 10.0, 8.0, 10.0, 48.0, -10.0, 48.0};
            solids[0].yBottom = 1.0;
            solids[0].yTop = 1.4;
            solids[0].cx = 0.0; solids[0].cz = 28.0; solids[0].radius = 25.0;
            v.SetSolids(&solids);
            VesselControls c;
            c.throttle[0] = 0.5;
            for (int i = 0; i < 240; ++i) v.Step(dry, c, i * dtLive, dtLive);   // 4 s: up and on it
            c.throttle[0] = 0.0;
            c.tilt = 1.0;
            for (int i = 0; i < 240; ++i) v.Step(dry, c, (240 + i) * dtLive, dtLive);   // braked to rest on it
            double q[3];
            cgOf(v, q);
            Log("[vessel] car at a step: the CG rose %.3f m (the step 0.400) at z %.1f, %d wheels carrying %.0f N, %.3f m/s, heel %.1f deg",
                q[1] - q0[1], q[2], v.Telemetry().contactPts, v.Telemetry().contactN, speedOf(v), heelOf(v));
            if (std::abs((q[1] - q0[1]) - 0.4) > 0.1) fail("solids: the car climbed the step (m)", q[1] - q0[1], 0.4);
            if (v.Telemetry().contactPts != 4) fail("solids: four wheels on the step", v.Telemetry().contactPts, 4);
        }
    }

    // ---- 9d. A RAMP: a deck segment's sloped top (sim/Solids.h gradients), 10 m wide, rising 2 m
    // over 20 m from z = 8. Driven up it the car's CG rises with the grade and all four wheels
    // stay down; at the top it stands level 2 m higher.
    {
        const VesselSpec car = reg.Build("car.baja");
        StillWater dry;
        dry.levelNavd = 0.0;
        dry.bedNavd = 1.0;
        Vessel v;
        v.Build(car, Motor::Translation(0.0, 1.0 + 1.2, 0.0));
        Settle(v, dry, 4800);
        VesselControls c;
        for (int i = 0; i < 120; ++i) v.Step(dry, c, i * dtLive, dtLive);
        double q0[3];
        cgOf(v, q0);
        std::vector<SolidPrism> solids(2);
        solids[0].xz = {-5.0, 8.0, 5.0, 8.0, 5.0, 28.0, -5.0, 28.0};   // the ramp: 0 at z 8, 2 m at z 28
        solids[0].cx = 0.0; solids[0].cz = 18.0; solids[0].yTop = 1.0 + 1.0; solids[0].yBottom = 1.0 + 1.0 - 1.0;
        solids[0].gx = 0.0; solids[0].gz = 0.1; solids[0].radius = 13.0;
        solids[1].xz = {-5.0, 28.0, 5.0, 28.0, 5.0, 60.0, -5.0, 60.0};   // the deck beyond: level at 2 m
        solids[1].cx = 0.0; solids[1].cz = 44.0; solids[1].yTop = 3.0; solids[1].yBottom = 2.0; solids[1].radius = 20.0;
        v.SetSolids(&solids);
        c.throttle[0] = 0.5;
        double worst = 0.0;
        int wheelsOff = 0;
        for (int i = 0; i < 360; ++i) {   // 6 s: up the ramp and onto the deck
            v.Step(dry, c, i * dtLive, dtLive);
            double q[3];
            cgOf(v, q);
            if (q[2] > 11.0 && q[2] < 25.0) {   // every wheel on the slope
                const double want = 0.1 * (q[2] - 8.0);
                worst = (std::max)(worst, std::abs((q[1] - q0[1]) - want));
                if (v.Telemetry().contactPts != 4) ++wheelsOff;
            }
        }
        c.throttle[0] = 0.0;
        c.tilt = 1.0;
        for (int i = 0; i < 240; ++i) v.Step(dry, c, (360 + i) * dtLive, dtLive);
        double q[3];
        cgOf(v, q);
        Log("[vessel] car up a ramp: on the slope the CG followed the grade to within %.3f m with %d steps short of four wheels; at rest on the deck %.3f m up (the deck 2.000) at z %.1f, %d wheels, trim %.2f deg",
            worst, wheelsOff, q[1] - q0[1], q[2], v.Telemetry().contactPts, v.Telemetry().trimRad * 180.0 / kPi);
        if (worst > 0.25) fail("solids: the CG left the ramp's grade (m)", worst, 0.25);
        if (wheelsOff > 6) fail("solids: wheels off the ramp (steps)", wheelsOff, 0);
        if (std::abs((q[1] - q0[1]) - 2.0) > 0.1) fail("solids: the car stands on the deck (m)", q[1] - q0[1], 2.0);
    }

    // ---- 10. PARTS PLUS FORMS. One body built from the barge and the car: in its first form it
    // is the barge (the same weight on the same six keel points on dry ground as the barge alone,
    // its draught afloat the barge's to the micrometre), and after SetForm it is the car (the same
    // four wheel loads as the car alone), the origin having moved to the car's CG. A form is the
    // parts' own laws, unchanged by sharing a body; the parts the form does not engage weigh
    // nothing, ask nothing and draw nothing.
    {
        const VesselSpec both = CombineForms({{"barge", spec}, {"car", reg.Build("car.baja")}});
        if (both.forms.size() != 2) fail("forms: CombineForms made", double(both.forms.size()), 2.0);
        // Afloat, as the barge.
        {
            Vessel one, two;
            one.Build(spec, Motor::Translation(0.0, 0.3, 0.0));
            two.Build(both, Motor::Translation(0.0, 0.3, 0.0));
            Settle(one, still, 4800);
            Settle(two, still, 4800);
            const double d1 = one.Telemetry().draughtM, d2 = two.Telemetry().draughtM;
            Log("[vessel] forms: afloat, the barge alone draws %.6f m and the barge form of the body %.6f m", d1, d2);
            if (std::abs(d1 - d2) > 1e-6) fail("forms: the barge form's draught is the barge's (m)", d2, d1);
        }
        // On dry ground: the barge form, then the car form.
        {
            StillWater dry;
            dry.levelNavd = 0.0;
            dry.bedNavd = 1.0;
            const double Wb = (spec.massLoaded.v > 0.0 ? spec.massLoaded.v : spec.massDry.v) * kG;
            const double Wc = (both.forms[1].massLoaded.v > 0.0 ? both.forms[1].massLoaded.v : both.forms[1].massDry.v) * kG;
            Vessel v;
            v.Build(both, Motor::Translation(0.0, 1.0 + 1.5, 0.0));
            Settle(v, dry, 4800);
            VesselControls c;
            for (int i = 0; i < 180; ++i) v.Step(dry, c, i * dtLive, dtLive);
            Log("[vessel] forms: the barge form stands on %d points carrying %.0f N of %.0f, %zu wheels drawn as %zu slots",
                v.Telemetry().contactPts, v.Telemetry().contactN, Wb, size_t(0), v.Wheels().size());
            if (v.Telemetry().contactPts != 6) fail("forms: the barge form's keel points", v.Telemetry().contactPts, 6);
            if (std::abs(v.Telemetry().contactN - Wb) > 0.01 * Wb) fail("forms: the barge form's weight on the ground (N)", v.Telemetry().contactN, Wb);
            double before[3];
            cgOf(v, before);
            if (!v.SetForm(1)) fail("forms: SetForm(1)", 0, 1);
            double after[3];
            cgOf(v, after);
            v.Body().SetPose(Motor::Translation(after[0], 1.0 + 1.5, after[2]));   // the car dropped onto the ground
            Settle(v, dry, 4800);
            for (int i = 0; i < 180; ++i) v.Step(dry, c, i * dtLive, dtLive);
            Log("[vessel] forms: SetForm moved the origin by (%+.3f, %+.3f, %+.3f) m; the car form stands on %d wheels carrying %.0f N of %.0f",
                after[0] - before[0], after[1] - before[1], after[2] - before[2], v.Telemetry().contactPts,
                v.Telemetry().contactN, Wc);
            if (v.Telemetry().contactPts != 4) fail("forms: the car form's wheels", v.Telemetry().contactPts, 4);
            if (std::abs(v.Telemetry().contactN - Wc) > 0.01 * Wc) fail("forms: the car form's weight on the ground (N)", v.Telemetry().contactN, Wc);
            if (v.Spec().kind != "car.baja") fail("forms: Spec() reads as the form", 0, 1);
        }
    }

    Log("[vessel] ---- %s: factory refuses unknown kinds, section clip (level/tilted/full/dry), "
        "draught + zero residual wrench, heave period, roll period vs GM, carried by a swell, "
        "absence is not flat water, the ground (dry flat, Coulomb's slope both ways, the beach "
        "stops it and the control sails on, the bed probes in the batch), the drone (5 m/s in "
        "3 tau, a 60 s hover, level from 30 deg), the car (static loads and compressions, the "
        "braked hill, the drive law, Ackermann), parts plus forms (the barge form is the barge, "
        "the car form is the car, the origin moves to the form's CG) ----",
        ok ? "PASS" : "FAIL");
    return ok;
}

}  // namespace ga
