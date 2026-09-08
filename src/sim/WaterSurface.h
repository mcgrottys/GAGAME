// ================================================================================================
//  WaterSurface - M9bq: THE ONE EVALUATOR A HULL READS THE WORLD THROUGH.
//
//  The architectural line for the vessel work is that a vessel is not a tree -- a rigid body is
//  13 doubles with no spatial extent, and making it a tenant would cost a DomainSource plus
//  forty lines of wiring to carry a single point. But everything the vessel READS comes through
//  the sparse GA tree, via exactly one evaluator, with no side channels and no readbacks. This
//  is that evaluator's interface.
//
//  WHY IT IS AN INTERFACE AND NOT JUST THE REAL THING. A gate needs water whose answer is known
//  in closed form, and the tempting way to get it -- construct the real evaluator with no tree
//  attached and let it fall back to flat -- is exactly the failure this engine's ingest rule
//  exists to forbid: absence would read as a measurement, and every "the boat floats correctly"
//  result would also be what a totally disconnected boat looks like. So the analytic surfaces
//  below are DECLARED implementations that say what they are, and the real one reports absence
//  as absence.
//
//  NEVER call SweSolver::ReadProbes or WaterBankLayer::TraceProbe from here. Both read back
//  whole multi-megabyte textures and end in Gpu::WaitIdle(), which at 240 Hz is not a slow path,
//  it is a stopped renderer.
// ================================================================================================
#pragma once

#include "core/Common.h"
#include "sim/Medium.h"

#include <cmath>

namespace ga {

class WaterSurface {
public:
    virtual ~WaterSurface() = default;
    virtual const char* Name() const = 0;

    // The free surface at a world point and instant. `valid` false means no coverage -- the
    // caller must treat that as "I do not know", never as flat water at datum zero.
    virtual SurfaceSample At(double wx, double wz, double simUnix) const = 0;

    // The wind at a point, world frame, m/s. Defaults to still so an implementation that has no
    // weather says so by being calm rather than by being wrong.
    virtual void WindAt(double wx, double wz, double simUnix, double out[3]) const {
        (void)wx; (void)wz; (void)simUnix;
        out[0] = out[1] = out[2] = 0.0;
    }

    // ---- the two media, assembled from the above. Non-virtual: every implementation must build
    // them the same way, because an element's whole contract is that it cannot tell them apart.
    Medium WaterAt(double wx, double wz, double simUnix) const {
        return WaterFrom(At(wx, wz, simUnix), wx, wz);
    }
    // The same medium from a sample the caller ALREADY HAS. Buoyancy asked for the surface and
    // then asked for the water at the same point, which evaluated the entire sea twice per
    // station -- and the sea is not cheap: a hull query sums the retained spectrum. Every
    // element that needs both now pays once.
    Medium WaterFrom(const SurfaceSample& s, double wx, double wz) const {
        Medium m = Medium::Seawater();
        FillSurface(m, s, wx, wz);
        m.vx = s.vx; m.vy = s.vy; m.vz = s.vz;
        return m;
    }
    Medium AirAt(double wx, double wz, double simUnix) const {
        const SurfaceSample s = At(wx, wz, simUnix);
        Medium m = Medium::Air();
        FillSurface(m, s, wx, wz);
        double w[3];
        WindAt(wx, wz, simUnix, w);
        m.vx = w[0]; m.vy = w[1]; m.vz = w[2];
        return m;
    }

private:
    static void FillSurface(Medium& m, const SurfaceSample& s, double wx, double wz) {
        m.sp[0] = wx; m.sp[1] = s.heightNavd; m.sp[2] = wz;
        m.nx = s.nx; m.ny = s.ny; m.nz = s.nz;
    }
};

// ================================================================================================
//  StillWater - a declared flat sea at a stated level. For gates whose answer is a closed form.
// ================================================================================================
class StillWater : public WaterSurface {
public:
    double levelNavd = 0.0;
    double bedNavd = -30.0;

    const char* Name() const override { return "still (declared analytic)"; }

    SurfaceSample At(double, double, double) const override {
        SurfaceSample s;
        s.heightNavd = levelNavd;
        s.bedNavd = bedNavd;
        s.depthM = levelNavd - bedNavd;
        s.ny = 1.0;
        s.valid = true;
        return s;
    }
};

// ================================================================================================
//  TiltedWater - a flat sea frozen at a fixed slope. Physically it is one instant of a wave face,
//  with all the dynamics removed, and it exists to ISOLATE the buoyancy-direction law.
//
//  On this surface the closed form is exact and has no free parameters: buoyancy is normal to the
//  surface, weight is vertical, so at vertical equilibrium the horizontal force is
//
//      F_h = m g tan(theta)
//
//  and the hull accelerates down-slope at g tan(theta), exactly like a ball on a ramp. Buoyancy
//  taken along WORLD UP instead gives F_h = 0 identically -- so unlike the wave-drift measurement
//  (where relative-velocity drag supplies most of the motion and both laws look alike), this test
//  separates the two by everything-versus-nothing.
// ================================================================================================
class TiltedWater : public WaterSurface {
public:
    double slopeRad = 0.05;           // surface tilts down toward +x
    double levelNavd = 0.0;           // elevation at x = 0
    double bedNavd = -60.0;

    const char* Name() const override { return "tilted plane (declared analytic)"; }

    SurfaceSample At(double wx, double, double) const override {
        const double t = std::tan(slopeRad);
        SurfaceSample s;
        s.heightNavd = levelNavd - t * wx;            // downhill toward +x
        const double inv = 1.0 / std::sqrt(1.0 + t * t);
        s.nx = t * inv;                               // normal leans downhill
        s.ny = inv;
        s.nz = 0.0;
        s.bedNavd = bedNavd;
        s.depthM = s.heightNavd - bedNavd;
        s.valid = true;
        return s;
    }
};

// ================================================================================================
//  AiryWave - a declared linear deep-water wave train. The gate for everything the plan claims
//  comes free from SurfaceSample carrying a plane and a velocity rather than a height:
//  a hull on this surface must heave, pitch, AND be carried along the wave, with nothing in the
//  vessel code written for waves.
//
//      eta   = a cos(theta),                theta = k (d . x) - omega t,  omega = sqrt(g k)
//      slope = -a k d sin(theta)            -> the normal is its dual, normalised
//      u     = a omega d cos(theta)         (horizontal, along d)
//      w     = a omega   sin(theta)         (vertical)
//
//  At a crest (theta = 0) the water moves FORWARD at a*omega and the surface is level; on the
//  face it moves up. That is the whole mechanism of surfing and it is why the velocities are
//  carried in the sample rather than reconstructed by a consumer.
// ================================================================================================
class AiryWave : public WaterSurface {
public:
    double amp = 0.5;                 // metres
    double k = 2.0 * 3.14159265358979323846 / 40.0;   // rad/m (40 m wave)
    double dirX = 1.0, dirZ = 0.0;    // unit propagation direction
    double levelNavd = 0.0;
    double bedNavd = -60.0;           // deep, so the deep-water form is the right one

    const char* Name() const override { return "airy (declared analytic)"; }

    double Omega() const { return std::sqrt(kG * k); }

    SurfaceSample At(double wx, double wz, double simUnix) const override {
        const double w = Omega();
        const double th = k * (dirX * wx + dirZ * wz) - w * simUnix;
        const double c = std::cos(th), sn = std::sin(th);

        SurfaceSample s;
        s.heightNavd = levelNavd + amp * c;
        // Gerstner horizontal displacement, the same sign the bank kernel uses: a particle at a
        // crest has been carried forward, in the trough back.
        s.dx = -amp * dirX * sn;
        s.dz = -amp * dirZ * sn;

        // The normal is the dual of the surface tangent 2-blade; for a height field that is
        // (-deta/dx, 1, -deta/dz) normalised.
        const double sl = -amp * k * sn;               // d(eta)/d(along d)
        double nx = -sl * dirX, ny = 1.0, nz = -sl * dirZ;
        const double inv = 1.0 / std::sqrt(nx * nx + ny * ny + nz * nz);
        s.nx = nx * inv; s.ny = ny * inv; s.nz = nz * inv;

        s.vx = amp * w * dirX * c;
        s.vy = amp * w * sn;
        s.vz = amp * w * dirZ * c;

        s.bedNavd = bedNavd;
        s.depthM = s.heightNavd - bedNavd;
        s.valid = true;
        return s;
    }
};

}  // namespace ga
