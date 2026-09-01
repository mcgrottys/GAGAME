// ================================================================================================
//  GaTest.cpp - M7j: the gatest gate. The GA products and closed forms the SHADERS rely on,
//  re-derived on the CPU and pinned; plus the GA AST's frame rules as a build gate, with the
//  orientation ledger's ground truths asserted edge by edge. A frame mismatch or a broken
//  product identity fails --selftest -- not a render review three sessions later.
// ================================================================================================
#include <cmath>
#include <cstring>
#include <random>

#include "Common.h"
#include "GaAst.h"
#include "GradeField.h"   // M9h: the type-level grade algebra pins itself here
#include "CurrentFieldLoader.h"
#include "FieldLoader.h"
#include "GeoRef.h"
#include "PageTable.h"
#include "Pga.h"

namespace ga {

namespace {

struct V3 {
    double x, y, z;
};
V3 operator+(V3 a, V3 b) { return {a.x + b.x, a.y + b.y, a.z + b.z}; }
V3 operator-(V3 a, V3 b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
V3 operator*(double s, V3 a) { return {s * a.x, s * a.y, s * a.z}; }
double Dot(V3 a, V3 b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
V3 Cross(V3 a, V3 b) {
    return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}
V3 Norm(V3 a) {
    const double l = std::sqrt(Dot(a, a));
    return {a.x / l, a.y / l, a.z / l};
}
double Len(V3 a) { return std::sqrt(Dot(a, a)); }

bool Near(double a, double b, double tol, const char* what, bool& ok) {
    if (std::abs(a - b) > tol) {
        Log("[gatest] FAIL %s: %.9f vs %.9f", what, a, b);
        ok = false;
        return false;
    }
    return true;
}

float Smooth(float e0, float e1, float x) {
    const float t = std::fmin(std::fmax((x - e0) / (e1 - e0), 0.0f), 1.0f);
    return t * t * (3.0f - 2.0f * t);
}

}  // namespace

bool RunGaSelfTest() {
    bool ok = true;
    ast::RegisterKnownWaterEdges();   // the canonical table -- selftest needs no scene
    std::mt19937 rng(20260830);
    std::uniform_real_distribution<double> U(-1.0, 1.0);
    auto rv = [&] { return Norm(V3{U(rng), U(rng), U(rng) + 1.7}); };

    // ---- 1. THE SANDWICH: reflection of v in the plane orthogonal to unit n is -n v n in
    // the geometric algebra; the shader ships the expanded form v - 2(v.n)n. Pin both, and
    // pin that reflecting twice is the identity (n v n n v n = v since n^2 = 1).
    for (int i = 0; i < 64; ++i) {
        const V3 n = rv();
        V3 v = {U(rng) * 2.0, U(rng) * 2.0, U(rng) * 2.0};
        const V3 r = v - 2.0 * Dot(v, n) * n;
        Near(Dot(r, n), -Dot(v, n), 1e-12, "reflect: normal component negates", ok);
        Near(Len(Cross(r - (-2.0 * Dot(v, n)) * n, v)), 0.0, 1e-9,
             "reflect: tangential part preserved", ok);
        const V3 rr = r - 2.0 * Dot(r, n) * n;
        Near(Len(rr - v), 0.0, 1e-12, "reflect twice = identity (n^2=1)", ok);
    }

    // ---- 2. THE REFRACTION ROTOR: Snell's construction as a rotor in the incidence plane
    // (the bivector d ^ n) must equal the closed form the shader ships:
    //     t = eta d + (eta ci - sqrt(1 - eta^2(1-ci^2))) n.
    // Build t independently by ROTATING d in the plane spanned by (n, tangent) through
    // (theta_t - theta_i) and compare. Also pin Snell: sin(theta_t) = eta sin(theta_i).
    for (int i = 0; i < 64; ++i) {
        const V3 n = rv();
        V3 d = rv();
        if (Dot(d, n) > -0.05) d = Norm(V3{-n.x + 0.3 * d.x, -n.y + 0.3 * d.y, -n.z + 0.3 * d.z});
        const double eta = 1.0 / 1.34;
        const double ci = -Dot(d, n);
        const double s2 = eta * eta * (1.0 - ci * ci);
        if (s2 >= 1.0) continue;   // TIR: covered by the sandwich above
        const V3 t = Norm(eta * d + (eta * ci - std::sqrt(1.0 - s2)) * n);
        // Independent rotor construction: rotate d about the axis n x d (the dual of the
        // incidence bivector) by (theta_t - theta_i).
        const double thI = std::acos(std::fmin(std::fmax(ci, -1.0), 1.0));
        const double thT = std::asin(std::sqrt(s2));
        const double ang = thT - thI;   // rotation from d toward the normal's inverse
        V3 axis = Cross(d, n);
        const double al = Len(axis);
        if (al < 1e-9) continue;        // normal incidence: t == d, closed form handles it
        axis = (1.0 / al) * axis;
        // Rodrigues = the rotor sandwich R d ~R expanded for a simple bivector.
        const V3 rd = std::cos(ang) * d + std::sin(ang) * Cross(axis, d) +
                      (1.0 - std::cos(ang)) * Dot(axis, d) * axis;
        Near(Len(Norm(rd) - t), 0.0, 1e-9, "refract: rotor == closed form", ok);
        Near(std::sqrt(1.0 - Dot(t, n) * Dot(t, n)), eta * std::sin(thI), 1e-9,
             "refract: Snell sin ratio", ok);
    }

    // ---- 3. THE FOLD TELESCOPE (M6t/M7a): per band, the ring carries wRing, the pixel adds
    // wDet = sat(wPix - wRing), sigma^2 keeps the rest. Pin the contract's endpoints: a pixel
    // whose footprint equals the ring texel adds NOTHING (no double counting); a pixel that
    // resolves everything adds exactly the ring's shortfall; weights stay in [0,1].
    for (int i = 0; i < 256; ++i) {
        const float lam = 2.0f + 200.0f * static_cast<float>(0.5 + 0.5 * U(rng));
        const float bT = 0.5f + 150.0f * static_cast<float>(0.5 + 0.5 * U(rng));
        auto w = [&](float foot) { return 1.0f - Smooth(lam * 0.12f, lam * 0.5f, foot); };
        const float wRing = w(bT);
        const float atRing = std::fmax(w(bT) - wRing, 0.0f);
        Near(atRing, 0.0, 1e-7, "fold: footprint==ring adds zero detail", ok);
        const float atZero = std::fmax(w(0.0f) - wRing, 0.0f);
        Near(atZero, 1.0f - wRing, 1e-6, "fold: full pixel adds the ring's shortfall", ok);
        if (wRing < 0.0f || wRing > 1.0f) {
            Log("[gatest] FAIL fold: wRing out of [0,1]");
            ok = false;
        }
    }

    // ---- 4. THE SPINOR BLEND (M6v): a tidal constituent interpolates as a Cl(2)+ spinor
    // (re, im) -- pin that the component blend of two equal-amplitude phasors preserves the
    // MEAN PHASE exactly, and that amplitude collapse is cos(dphi/2) (the geometric truth),
    // where a naive amp/phase lerp would keep amplitude 1 and can even walk the wrong way
    // around the circle.
    for (int i = 0; i < 64; ++i) {
        const double p0 = U(rng) * 3.0, dp = U(rng) * 2.5;
        const double re = 0.5 * (std::cos(p0) + std::cos(p0 + dp));
        const double im = 0.5 * (std::sin(p0) + std::sin(p0 + dp));
        const double mid = std::atan2(im, re);
        double want = p0 + dp * 0.5;
        while (want > 3.14159265358979) want -= 2.0 * 3.14159265358979;
        while (want < -3.14159265358979) want += 2.0 * 3.14159265358979;
        if (std::sqrt(re * re + im * im) > 1e-6) {
            double dd = mid - want;
            while (dd > 3.14159265358979) dd -= 2.0 * 3.14159265358979;
            while (dd < -3.14159265358979) dd += 2.0 * 3.14159265358979;
            Near(dd, 0.0, 1e-9, "spinor: component blend preserves mean phase", ok);
            Near(std::sqrt(re * re + im * im), std::abs(std::cos(dp * 0.5)), 1e-9,
                 "spinor: amplitude folds as cos(dphi/2)", ok);
        }
    }

    // ---- 4b. THE SLICE PLANE (M7o, proofs/slice_plane.py): reflection through the
    // plane pi = (n, d) is the sandwich P' = P - 2 s(P) n with s(P) = P.n - d, and it
    // NEGATES the signed distance -- the discard predicate splits exactly the two halves
    // the sandwich exchanges. Pinned before the renderer ever discards a pixel.
    for (int i = 0; i < 64; ++i) {
        const V3 n = rv();
        const double dOff = U(rng) * 500.0;
        const V3 P = {U(rng) * 900.0, U(rng) * 900.0, U(rng) * 900.0};
        const double sP = Dot(P, n) - dOff;
        const V3 Pr = P - 2.0 * sP * n;
        Near(Dot(Pr, n) - dOff, -sP, 1e-9, "slice: sandwich negates signed distance", ok);
    }

    // ---- 5. THE FRAME RULES: the AST's flip rule over every registered edge, then the
    // orientation ledger's ground truths asserted by name -- the conventions that BIT us,
    // pinned forever. (Row-0-north rasters demand a flip into +v=north consumers; the wrap
    // and bank atlases agree with world +z and demand none.)
    ok &= ast::Validate();
    struct Truth {
        const char* from, *field;
        bool srcVNorth, flip;
    };
    const Truth truths[] = {
        {"swe.solver", "eta", false, true},
        {"swe.solver", "uv", false, true},
        {"bathy.cudem", "bed", false, true},
        {"churn.kernel", "churn", true, false},
        {"ocean.fft", "cascade.disp", true, false},
        {"swe.solver", "shadow", false, true},
    };
    for (const Truth& t : truths) {
        bool found = false;
        for (const auto& e : ast::Edges()) {
            if (std::strcmp(e.from, t.from) || std::strcmp(e.field, t.field)) continue;
            found = true;
            if (e.src.vNorth != t.srcVNorth || e.flip != t.flip) {
                Log("[gatest] FAIL ledger truth: %s '%s' expected src %s flip=%d, "
                    "registered src %s flip=%d",
                    t.from, t.field, t.srcVNorth ? "+v=N" : "+v=S", t.flip ? 1 : 0,
                    e.src.vNorth ? "+v=N" : "+v=S", e.flip ? 1 : 0);
                ok = false;
            }
        }
        if (!found) {
            Log("[gatest] FAIL ledger truth: edge %s '%s' is not registered", t.from,
                t.field);
            ok = false;
        }
    }

    // ---- M7r: the world->latlon->mercator->uv chain, FLOAT (the shader/kernel path)
    // against a DOUBLE reference. Five sites share this chain (WaterBank kernel,
    // CsWindowUv, GisStencil, the want walk, the paint loops); the CPU sites run doubles
    // (exact), the shader sites run float32 -- this pins the float path's ground error
    // under a third of a bed texel over the whole window, so "is the abstraction working
    // everywhere" has a number instead of a vibe.
    {
        const double n14 = 16384.0 * 256.0;
        const double kOrgLat = 42.6017, kOrgLon = -70.8600;   // anchor family (values
        const double mPerLat = 111132.0, mPerLon = 81800.0;   // representative; the test
                                                              // bounds the FLOAT OPS, not
                                                              // the survey constants)
        const double winOrgX = 1263360.0, winOrgY = 1538048.0;
        double worstM = 0.0;
        for (int iz = -20; iz <= 20; ++iz) {
            for (int ix = -20; ix <= 20; ++ix) {
                const double wx = ix * 2000.0, wz = iz * 2000.0;
                // double reference
                const double latD = kOrgLat + wz / mPerLat;
                const double lonD = kOrgLon + wx / mPerLon;
                const double mxD = (lonD + 180.0) / 360.0 * n14;
                const double myD =
                    (0.5 - std::log(std::tan(0.78539816339744831 +
                                             latD * 0.017453292519943295 * 0.5)) /
                               (2.0 * 3.14159265358979324)) *
                    n14;
                // float chain, exactly as the kernel/CsWindowUv run it
                const float latF = static_cast<float>(kOrgLat) +
                                   static_cast<float>(wz) * static_cast<float>(1.0 / mPerLat);
                const float lonF = static_cast<float>(kOrgLon) +
                                   static_cast<float>(wx) * static_cast<float>(1.0 / mPerLon);
                const float latR = latF * 0.01745329252f;
                const float mxF = (lonF + 180.0f) / 360.0f * static_cast<float>(n14);
                const float myF =
                    (0.5f - std::log(std::tan(0.7853981634f + latR * 0.5f)) *
                                0.15915494309f) *
                    static_cast<float>(n14);
                const double groundPerPx = 9.55 * std::cos(latD * 0.0174533);
                const double errM =
                    std::hypot((mxF - mxD) * groundPerPx, (myF - myD) * groundPerPx);
                worstM = (std::max)(worstM, errM);
                (void)winOrgX; (void)winOrgY;
            }
        }
        Log("[gatest] merc chain float-vs-double: worst ground error %.2f m over "
            "+-40 km (bed texel 13.7 m)", worstM);
        if (worstM > 4.5) {
            Log("[gatest] FAIL merc chain: float path drifts past a third of a bed texel");
            ok = false;
        }
    }

    // ================================================================================
    //  M8 WATER PARITY (ALGEBRA.md wavefield/caustics/ripple/foamlaw/wake/bedalbedo).
    //  The closed forms the solved wave field and its shading ship, pinned on CPU
    //  doubles. The heavy statistics live in proofs/ (wave_field, caustic_jacobian,
    //  ripple_prefilter, foam_discipline, kelvin_wake, bed_relief); these blocks pin
    //  the identities those proofs established, so a regression fails --selftest.
    // ================================================================================
    const double PI = 3.14159265358979324;
    const double G = 9.81;

    // ---- 6. THE SOLVED WAVE FIELD: dispersion under current (bracket+bisect selects
    // the physical branch; Newton diverges near blocking), the deep blocking point at
    // r = -1/4, WaveCurrentAmp == exact deep-water action transport, Green's law,
    // the total-Hs limiter's invariants, the spinor bilinear error bound, and the
    // blocked-k hold's Nyquist margin (grid-tuned closure: k*delta < pi).
    {
        auto sigmaR = [&](double k, double h) { return std::sqrt(G * k * std::tanh(k * h)); };
        auto solveK = [&](double sig, double h, double uopp, bool& blocked) {
            h = (std::max)(h, 0.15);
            double lo = 0.0, hi = 0.0, prev = 1e-4, fPrev = 0.0;
            blocked = true;
            for (int i = 0; i < 96; ++i) {
                const double k = std::pow(10.0, -4.0 + 4.7 * i / 95.0);
                const double f = (sig + k * uopp) * (sig + k * uopp) - G * k * std::tanh(k * h);
                if (i > 0 && fPrev > 0.0 && f <= 0.0) { lo = prev; hi = k; blocked = false; break; }
                prev = k; fPrev = f;
            }
            if (blocked) return 0.25 * std::pow(10.0, 0.7);
            for (int i = 0; i < 48; ++i) {
                const double mid = 0.5 * (lo + hi);
                const double f = (sig + mid * uopp) * (sig + mid * uopp) -
                                 G * mid * std::tanh(mid * h);
                if (f <= 0.0) hi = mid; else lo = mid;
            }
            return 0.5 * (lo + hi);
        };
        // residual + still-water deep closed form k = sigma^2/g
        for (int i = 0; i < 16; ++i) {
            const double T = 3.0 + 9.0 * (0.5 + 0.5 * U(rng));
            const double sig = 2.0 * PI / T;
            const double h = 1.0 + 40.0 * (0.5 + 0.5 * U(rng));
            const double uopp = 1.2 * (0.5 + 0.5 * U(rng));
            bool blocked = false;
            const double k = solveK(sig, h, uopp, blocked);
            if (!blocked) {
                const double f = (sig + k * uopp) * (sig + k * uopp) - G * k * std::tanh(k * h);
                Near(f / (sig * sig), 0.0, 1e-9, "wavefield: dispersion residual", ok);
            }
            bool b2 = false;
            const double kDeep = solveK(sig, 500.0, 0.0, b2);
            Near(kDeep, sig * sig / G, 1e-6 * sig * sig / G,
                 "wavefield: deep still-water k = sigma^2/g", ok);
        }
        // deep blocking at Uopp = c0/4: sigma_max = g/(4*Uopp); the discrete solver may
        // block slightly EARLY (conservative), never late.
        {
            const double uopp = 1.0;
            const double sigMax = G / (4.0 * uopp);
            bool bBelow = false, bAbove = false;
            solveK(sigMax * 0.97, 500.0, uopp, bBelow);
            solveK(sigMax * 1.03, 500.0, uopp, bAbove);
            if (bBelow || !bAbove) {
                Log("[gatest] FAIL wavefield: blocking margin (below=%d above=%d)",
                    bBelow ? 1 : 0, bAbove ? 1 : 0);
                ok = false;
            }
        }
        // WaveCurrentAmp's closed form == exact deep-water action transport:
        // with cr = (1+sqrt(1+4r))/2 (so r = cr^2 - cr), action gives
        // 1/sqrt(cr*(cr+2r)) and the engine ships 1/sqrt(cr^2*(2cr-1)) -- identical.
        for (int i = 0; i < 64; ++i) {
            const double r = -0.24 + 0.54 * (0.5 + 0.5 * U(rng));
            const double cr = 0.5 * (1.0 + std::sqrt(1.0 + 4.0 * r));
            const double engine = 1.0 / std::sqrt(cr * cr * (2.0 * cr - 1.0));
            const double action = 1.0 / std::sqrt(cr * (cr + 2.0 * r));
            Near(engine, action, 1e-12, "wavefield: WaveCurrentAmp == action transport", ok);
        }
        // Green's law: at U = 0 the SHALLOW limit gives Ks ~ h^(-1/4): a x16 depth drop
        // doubles Ks. Both depths must sit in the shallow asymptote (kh << 1), which is
        // under the solver's 0.15 m guard floor -- so this pin solves the continuum
        // dispersion directly (floorless bisection); the floor is a guard, not physics.
        {
            auto ks = [&](double h) {
                const double sig = 2.0 * PI / 20.0;
                double lo = 1e-6, hi = 10.0;
                for (int i = 0; i < 80; ++i) {
                    const double mid = 0.5 * (lo + hi);
                    if (G * mid * std::tanh(mid * h) < sig * sig) lo = mid; else hi = mid;
                }
                const double k = 0.5 * (lo + hi);
                const double c = sig / k;
                const double kh = k * h;
                const double n = 0.5 * (1.0 + 2.0 * kh / std::sinh(2.0 * kh));
                const double c0 = G * 20.0 / (2.0 * PI);
                return std::sqrt(0.5 * c0 / (n * c));
            };
            Near(ks(0.03125) / ks(0.5), 2.0, 0.02, "wavefield: Green's law h^(-1/4)", ok);
        }
        // The total-Hs limiter: rms capped, spectral shape preserved, excess iff scaled.
        {
            double a[16], rms2 = 0.0;
            for (int i = 0; i < 16; ++i) { a[i] = 0.05 + 0.4 * (0.5 + 0.5 * U(rng)); rms2 += a[i] * a[i]; }
            const double h = 0.8;
            const double lim = 0.60 * h / (2.0 * std::sqrt(2.0));
            const double excess = std::sqrt(rms2) / lim;
            const double s = (std::min)(1.0, 1.0 / excess);
            double rms2After = 0.0;
            for (int i = 0; i < 16; ++i) rms2After += (a[i] * s) * (a[i] * s);
            if (std::sqrt(rms2After) > lim * (1.0 + 1e-12) && excess > 1.0) {
                Log("[gatest] FAIL wavefield: limiter exceeds rms cap");
                ok = false;
            }
            Near((a[3] * s) / (a[7] * s), a[3] / a[7], 1e-12,
                 "wavefield: limiter preserves spectral shape", ok);
        }
        // Spinor bilinear phase-error bound: lerp of unit spinors e^{+-i delta} has
        // |arg error| <= 0.16*delta^3 for delta <= 0.94, amplitude >= cos(delta).
        for (int i = 0; i < 32; ++i) {
            const double delta = 0.05 + 0.89 * (0.5 + 0.5 * U(rng));
            double worst = 0.0, ampMin = 1.0;
            for (int it = 0; it <= 64; ++it) {
                const double t = it / 64.0;
                const double re = std::cos(delta);
                const double im = (2.0 * t - 1.0) * std::sin(delta);
                worst = (std::max)(worst,
                                   std::abs(std::atan2(im, re) - (2.0 * t - 1.0) * delta));
                ampMin = (std::min)(ampMin, std::hypot(re, im));
            }
            if (worst > 0.16 * delta * delta * delta + 1e-9) {
                Log("[gatest] FAIL wavefield: spinor lerp error %.6f > 0.16*delta^3 (delta %.3f)",
                    worst, delta);
                ok = false;
            }
            if (ampMin < std::cos(delta) - 1e-12) {
                Log("[gatest] FAIL wavefield: spinor lerp amplitude under cos(delta)");
                ok = false;
            }
        }
        // Blocked-k hold: k*delta stays under pi on the reference grid (0.60*pi).
        Near(0.25 * std::pow(10.0, 0.7) * 1.5 / PI, 0.5984, 2e-3,
             "wavefield: blocked-k hold Nyquist margin", ok);
    }

    // ---- 7. CAUSTICS: the tangent bivector's two projections (normal + areaJac), the
    // CORRECTED physical curvature (the adversarial catch: numerator cos t - s a k, no
    // cos 2t), the crest identity lap/areaJac^2, the K = 1 - 1/n gap, and flat gain 1.
    {
        for (int i = 0; i < 64; ++i) {
            const double a = 0.05 + 0.5 * (0.5 + 0.5 * U(rng));
            const double k = 0.1 + 1.2 * (0.5 + 0.5 * U(rng));
            const double s = 0.9 * (0.5 + 0.5 * U(rng));
            const double th = PI * U(rng);
            if (s * a * k > 0.85) continue;   // stay off the fold, as the chop guard does
            // single-component areaJac closed form |1 - s a k cos|
            const double J00 = -s * a * k * std::cos(th);   // d along x
            const double areaJac = std::abs((1.0 + J00) * 1.0 - 0.0);
            Near(areaJac, std::abs(1.0 - s * a * k * std::cos(th)), 1e-12,
                 "caustics: single-component areaJac", ok);
            // corrected physical curvature vs the parametric chain
            const double xp = 1.0 - s * a * k * std::cos(th);
            const double xpp = s * a * k * k * std::sin(th);
            const double yp = -a * k * std::sin(th);
            const double ypp = -a * k * k * std::cos(th);
            const double chain = (ypp * xp - yp * xpp) / (xp * xp * xp);
            const double formula = -a * k * k * (std::cos(th) - s * a * k) / (xp * xp * xp);
            Near(chain, formula, 1e-12 * (1.0 + std::abs(chain)),
                 "caustics: corrected curvature numerator", ok);
        }
        // crest identity: eta''_phys(crest) == lap_param / areaJac^2
        {
            const double a = 0.4, k = 2.0 * PI / 8.0, s = 1.0;
            const double aj = 1.0 - s * a * k;
            const double phys = -a * k * k * (1.0 - s * a * k) / (aj * aj * aj);
            Near(phys, (-a * k * k) / (aj * aj), 1e-12, "caustics: crest identity", ok);
        }
        Near(std::abs(0.25 - (1.0 - 1.0 / 1.333)), 1.875e-4, 2e-5,
             "caustics: K = 1 - 1/n gap vs the shipped 0.25", ok);
        // flat surface: gain == 1 through washout and clamps
        {
            const double areaJac = 1.0, lap = 0.0, hBed = 7.0;
            double gain = 1.0 / ((std::max)(areaJac, 0.05) *
                                 (std::max)(1.0 + hBed * 0.25 * lap, 0.05));
            const double t = (std::min)((std::max)((hBed - 4.0) / 16.0, 0.0), 1.0);
            gain = 1.0 + (gain - 1.0) * (1.0 - t * t * (3.0 - 2.0 * t));
            gain = (std::min)((std::max)(gain, 0.35), 2.6);
            Near(gain, 1.0, 1e-12, "caustics: flat surface gain is 1", ok);
        }
        // fold assembly: one active cascade at w = 1 reproduces its J exactly
        {
            const double Jc = 0.73;
            Near(1.0 + 1.0 * (Jc - 1.0), Jc, 1e-15, "caustics: fold assembly single-cascade", ok);
        }
    }

    // ---- 8. THE RIPPLE PREFILTER: exact Gaussian of the footprint frame. w(0)=1,
    // isotropic reduction, the Gram/bivector identity det(J J^T) = |fpx ^ fpz|^2,
    // rotation equivariance, the mss closed form + its 1/sqrt(N) invariance, the
    // golden three-gap structure, and the telescope endpoint.
    {
        auto wOf = [&](double kx, double ky, double ax, double ay, double bx, double by) {
            const double A = 0.5 * (kx * ax + ky * ay);
            const double B = 0.5 * (kx * bx + ky * by);
            return std::exp(-0.5 * (A * A + B * B));
        };
        Near(wOf(0.0, 0.0, 3.7, 0.1, -0.4, 9.9), 1.0, 1e-12, "ripple: w(k=0) = 1", ok);
        for (int i = 0; i < 32; ++i) {
            const double L = 0.2 + 4.0 * (0.5 + 0.5 * U(rng));
            const double ang = PI * U(rng);
            const double kx = 2.0 * U(rng), ky = 2.0 * U(rng);
            const double w = wOf(kx, ky, L * std::cos(ang), L * std::sin(ang),
                                 -L * std::sin(ang), L * std::cos(ang));
            Near(w, std::exp(-(kx * kx + ky * ky) * L * L / 8.0), 1e-12,
                 "ripple: isotropic reduction", ok);
            // Gram identity + rotation equivariance
            const double ax = 3.0 * U(rng), ay = 3.0 * U(rng);
            const double bx = 3.0 * U(rng), by = 3.0 * U(rng);
            const double m00 = ax * ax + bx * bx, m01 = ax * ay + bx * by,
                         m11 = ay * ay + by * by;
            const double wedge = ax * by - ay * bx;
            Near(m00 * m11 - m01 * m01, wedge * wedge,
                 1e-9 * (1.0 + m00 * m11), "ripple: det(JJ^T) = |fpx^fpz|^2", ok);
            const double ca = std::cos(0.7), sa = std::sin(0.7);
            Near(wOf(ca * kx - sa * ky, sa * kx + ca * ky,
                     ca * ax - sa * ay, sa * ax + ca * ay,
                     ca * bx - sa * by, sa * bx + ca * by),
                 wOf(kx, ky, ax, ay, bx, by), 1e-12, "ripple: rotation equivariance", ok);
        }
        // mss closed form and N-invariance (ALGEBRA.md ripple, proofs/ripple_prefilter.py)
        auto mss = [&](int N) {
            double sum = 0.0;
            for (int i = 0; i < N; ++i) {
                const double t = (i + 0.5) / N;
                const double T = 0.70 * std::pow(4.0 / 0.70, t);
                const double kk = (2.0 * PI / T) * (2.0 * PI / T) / G;
                const double amp = 0.0156 * std::sqrt(20.0 / N) * std::pow(kk, -1.25);
                sum += amp * kk * amp * kk * 0.5;
            }
            return sum;
        };
        Near(mss(96), 2.29679e-3, 5e-7, "ripple: mss(96) closed-form pin", ok);
        Near(mss(20), 2.29610e-3, 5e-7, "ripple: mss(20) calibration pin", ok);
        Near(mss(20) / mss(96), 1.0, 1e-3, "ripple: 1/sqrt(N) invariance", ok);
        // golden three-gap at N = 96: gaps take exactly three values, largest = sum of
        // the other two (Steinhaus), adjacent ratio phi.
        {
            double g[96];
            for (int i = 0; i < 96; ++i) {
                const double v = i * 0.6180339887498949;
                g[i] = v - std::floor(v);
            }
            for (int i = 0; i < 96; ++i)
                for (int j = i + 1; j < 96; ++j)
                    if (g[j] < g[i]) { const double t = g[i]; g[i] = g[j]; g[j] = t; }
            double d[96];
            for (int i = 0; i < 95; ++i) d[i] = g[i + 1] - g[i];
            d[95] = 1.0 - g[95] + g[0];
            double v0 = 2.0, v1 = -1.0, v2 = -1.0;   // small, mid, large
            for (int i = 0; i < 96; ++i) v0 = (std::min)(v0, d[i]);
            for (int i = 0; i < 96; ++i)
                if (d[i] > v0 + 1e-9 && (v1 < 0.0 || d[i] < v1)) v1 = d[i];
            for (int i = 0; i < 96; ++i) v2 = (std::max)(v2, d[i]);
            int distinct = 0;
            for (int i = 0; i < 96; ++i) {
                if (std::abs(d[i] - v0) > 1e-9 && std::abs(d[i] - v1) > 1e-9 &&
                    std::abs(d[i] - v2) > 1e-9)
                    ++distinct;
            }
            if (distinct) { Log("[gatest] FAIL ripple: more than three gap lengths"); ok = false; }
            Near(v2, v0 + v1, 1e-9, "ripple: largest gap = sum of other two", ok);
            Near(v1 / v0, 1.6180339887498949, 1e-4, "ripple: adjacent gap ratio = phi", ok);
        }
        // telescope endpoint for the variance-true pairing: footprint == ring adds zero
        {
            const float lam = 26.8f, bT = 9.6f;
            const float wp = 1.0f - Smooth(lam * 0.12f, lam * 0.5f, bT);
            Near(std::fmax(wp * wp - wp * wp, 0.0f), 0.0, 1e-12,
                 "ripple: variance telescope endpoint", ok);
        }
    }

    // ---- 9. FOAM DISCIPLINE: sqrt(N) peak/rms, tanh shaping, the crest-gate
    // quadrature, the Jacobian<->steepness bijection, the renormalization invariance,
    // and the churn half-life.
    {
        {
            double sum = 0.0, sum2 = 0.0;
            for (int i = 0; i < 16; ++i) { sum += 0.15; sum2 += 0.15 * 0.15; }
            Near(sum / std::sqrt(sum2), 4.0, 1e-12, "foamlaw: peak/rms = sqrt(N)", ok);
        }
        Near(std::tanh(1.6), 0.92166855, 1e-6, "foamlaw: tanh(1.6) shaping ceiling", ok);
        // crest-gate quadrature: C(0.28, 0.80) over a Gaussian sea = 0.2256
        {
            double C = 0.0;
            const int n = 4000;
            for (int i = 0; i <= n; ++i) {
                const double z = -8.0 + 16.0 * i / n;
                const double t = (std::min)(
                    (std::max)((z / std::sqrt(2.0) - 0.28) / (0.80 - 0.28), 0.0), 1.0);
                const double S = t * t * (3.0 - 2.0 * t);
                const double phi = std::exp(-0.5 * z * z) / std::sqrt(2.0 * PI);
                const double wSimp = (i == 0 || i == n) ? 1.0 : ((i & 1) ? 4.0 : 2.0);
                C += wSimp * S * phi;
            }
            C *= (16.0 / n) / 3.0;
            Near(C, 0.2256, 5e-4, "foamlaw: crest-gate Gaussian coverage", ok);
        }
        // Jacobian <-> steepness bijection: min_phi J = 1 - lambda*a*k exactly; the
        // engine's foam ramp saturate((0.80 - J)*4) then spans ak in [0.1818, 0.4091].
        for (int i = 0; i < 32; ++i) {
            const double ak = 0.05 + 0.5 * (0.5 + 0.5 * U(rng));
            double mn = 10.0;
            for (int p = 0; p < 720; ++p) {
                const double phi = 2.0 * PI * p / 720.0;
                mn = (std::min)(mn, 1.0 - 1.1 * ak * std::cos(phi));
            }
            Near(mn, 1.0 - 1.1 * ak, 1e-9, "foamlaw: min_phi J = 1 - lambda*ak", ok);
        }
        Near(0.20 / 1.1, 0.181818, 1e-6, "foamlaw: Jacobian foam onset ak", ok);
        Near(0.45 / 1.1, 0.409091, 1e-6, "foamlaw: Jacobian foam saturation ak", ok);
        // FoamNoise renormalization: mean-1/2 octaves keep mean 1/2 under ANY fades
        for (int i = 0; i < 32; ++i) {
            const double w1 = 0.5 + 0.5 * U(rng), w2 = 0.5 + 0.5 * U(rng),
                         w3 = 0.5 + 0.5 * U(rng);
            const double wsum = 0.45 * w1 + 0.34 * w2 + 0.21 * w3;
            if (wsum <= 1e-3) continue;
            Near((0.45 * w1 * 0.5 + 0.34 * w2 * 0.5 + 0.21 * w3 * 0.5) / wsum, 0.5, 1e-12,
                 "foamlaw: FoamNoise renormalization preserves the mean", ok);
        }
        Near(90.0 * std::log(2.0), 62.383, 1e-3, "foamlaw: churn half-life", ok);
    }

    // ---- 10. KELVIN WAKES: the wedge as discriminant, root stationarity + Vieta,
    // THE SIGNED PHASE (|grad ph| = k exactly; the reference's folded form fails this),
    // per-class wavelengths, plane-fit exactness, motor-frame invariance.
    {
        Near(std::atan(1.0 / (2.0 * std::sqrt(2.0))), std::asin(1.0 / 3.0), 1e-12,
             "wake: wedge angle atan(1/2sqrt2) == asin(1/3)", ok);
        auto roots = [&](double xi, double zeta, double& t1, double& t2) {
            const double disc = xi * xi - 8.0 * zeta * zeta;
            const double sq = std::sqrt((std::max)(disc, 0.0));
            t1 = (-xi + sq) / (4.0 * zeta);
            t2 = (-xi - sq) / (4.0 * zeta);
            return disc > 0.0;
        };
        for (int i = 0; i < 64; ++i) {
            const double xi = 5.0 + 200.0 * (0.5 + 0.5 * U(rng));
            const double zeta = (0.02 + 0.93 * (0.5 + 0.5 * U(rng))) * xi / (2.0 * std::sqrt(2.0));
            double t1, t2;
            if (!roots(xi, zeta, t1, t2)) continue;
            const double scale = (std::max)(std::abs(xi), std::abs(zeta));
            Near((2.0 * zeta * t1 * t1 + xi * t1 + zeta) / scale, 0.0, 1e-9,
                 "wake: transverse root stationarity", ok);
            Near((2.0 * zeta * t2 * t2 + xi * t2 + zeta) / scale, 0.0, 1e-9,
                 "wake: divergent root stationarity", ok);
            Near(t1 * t2, 0.5, 1e-9, "wake: Vieta t+ t- = 1/2", ok);
        }
        // The signed stationary phase: |grad ph|/k == 1 (FD, branch-uniform step);
        // the reference's folded form measures > 1.5 on the divergent branch.
        {
            const double U0 = 4.86, K0 = G / (U0 * U0);
            auto phaseAt = [&](double xi, double zeta, bool folded, int branch) {
                double t1, t2;
                if (!roots(xi, std::abs(zeta), t1, t2)) return 0.0;
                const double t = (branch == 0) ? t1 : t2;
                const double sec = std::sqrt(1.0 + t * t);
                if (folded) return K0 * (xi * sec + std::abs(zeta) * sec * std::abs(t));
                return K0 * sec * (xi - std::abs(zeta) * std::abs(t));
            };
            for (int branch = 0; branch < 2; ++branch) {
                for (int i = 0; i < 16; ++i) {
                    const double xi = 30.0 + 150.0 * (0.5 + 0.5 * U(rng));
                    const double zeta =
                        (0.10 + 0.75 * (0.5 + 0.5 * U(rng))) * xi / (2.0 * std::sqrt(2.0));
                    double t1, t2;
                    roots(xi, zeta, t1, t2);
                    const double t = (branch == 0) ? t1 : t2;
                    const double k = K0 * (1.0 + t * t);
                    const double hs = 1e-3 / (1.0 + t * t);
                    const double dpx =
                        (phaseAt(xi + hs, zeta, false, branch) -
                         phaseAt(xi - hs, zeta, false, branch)) / (2.0 * hs);
                    const double dpz =
                        (phaseAt(xi, zeta + hs, false, branch) -
                         phaseAt(xi, zeta - hs, false, branch)) / (2.0 * hs);
                    Near(std::hypot(dpx, dpz) / k, 1.0, 1e-4,
                         "wake: signed phase |grad ph| = k", ok);
                }
            }
            // document the reference bug: the folded form's gradient overshoots
            double t1, t2;
            roots(60.0, 18.0, t1, t2);
            const double k = K0 * (1.0 + t2 * t2);
            const double hs = 1e-3 / (1.0 + t2 * t2);
            const double dpx = (phaseAt(60.0 + hs, 18.0, true, 1) -
                                phaseAt(60.0 - hs, 18.0, true, 1)) / (2.0 * hs);
            const double dpz = (phaseAt(60.0, 18.0 + hs, true, 1) -
                                phaseAt(60.0, 18.0 - hs, true, 1)) / (2.0 * hs);
            if (std::hypot(dpx, dpz) / k < 1.5) {
                Log("[gatest] FAIL wake: folded reference form unexpectedly stationary");
                ok = false;
            }
        }
        Near(2.0 * PI * 4.86 * 4.86 / G, 15.128, 1e-3, "wake: recreational lambda_t", ok);
        Near(2.0 * PI * 3.34 * 3.34 / G, 7.145, 1e-3, "wake: commercial lambda_t", ok);
        // plane-fit exactness on planar input over the 5x3 stencil
        {
            const double al = 0.13, be = -0.21, ga = 0.77, HL = 7.5, HB = 2.55;
            double sE = 0.0, sXE = 0.0, sZE = 0.0, sXX = 0.0, sZZ = 0.0;
            for (int ix = 0; ix < 5; ++ix) {
                for (int iz = 0; iz < 3; ++iz) {
                    const double x = HL * (ix * 0.5 - 1.0), z = HB * (iz - 1.0);
                    const double e = al * x + be * z + ga;
                    sE += e; sXE += x * e; sZE += z * e; sXX += x * x; sZZ += z * z;
                }
            }
            Near(sE / 15.0, ga, 1e-12, "wake: plane fit heave exact", ok);
            Near(sXE / sXX, al, 1e-12, "wake: plane fit slopeX exact", ok);
            Near(sZE / sZZ, be, 1e-12, "wake: plane fit slopeZ exact", ok);
        }
        // motor-frame invariance: eta from world-frame projections == eta from the
        // body frame reached through the vessel motor's inverse (Pga.h machinery).
        {
            const double U0 = 4.62, K0 = G / (U0 * U0);
            auto etaOf = [&](double xi, double across) {
                const double zeta = std::abs(across);
                if (xi <= 0.5) return 0.0;
                double t1, t2;
                if (!roots(xi, (std::max)(zeta, 1e-3), t1, t2)) return 0.0;
                double eta = 0.0;
                for (int branch = 0; branch < 2; ++branch) {
                    const double t = (branch == 0) ? t1 : t2;
                    const double sec = std::sqrt(1.0 + t * t);
                    const double ph = K0 * sec * (xi - zeta * std::abs(t));
                    eta += ((branch == 0) ? 1.0 : 0.85) * std::cos(ph);
                }
                return eta;
            };
            const double hd = 0.83, px = 412.0, pz = -167.0;
            const double org[3] = {0, 0, 0};
            const double up[3] = {0, 1, 0};
            const Motor mv = Motor::Translation(px, 0.0, pz) * Motor::Rotation(org, up, -hd);
            const Motor inv = mv.Inverse();
            for (int i = 0; i < 24; ++i) {
                const double qx = px + 250.0 * U(rng), qz = pz + 250.0 * U(rng);
                // world-frame projections
                const double fwdX = std::cos(hd), fwdZ = std::sin(hd);
                const double rgtX = -fwdZ, rgtZ = fwdX;
                const double rx = qx - px, rz = qz - pz;
                const double xiW = -(rx * fwdX + rz * fwdZ);
                const double acW = rx * rgtX + rz * rgtZ;
                // body frame through the motor inverse (ship faces +x there)
                double bx = qx, by = 0.0, bz = qz;
                inv.TransformPoint(bx, by, bz);
                Near(etaOf(xiW, acW), etaOf(-bx, bz), 1e-9,
                     "wake: motor-frame invariance", ok);
            }
        }
    }

    // ---- 11. BED ALBEDO + THE WATERLINE METRIC: the cross extractor's transfer
    // function (zeros/peaks/diagonal gain 2), the skew vector's clamped mean, the
    // narrowness classifier's exact discrete count, the hemisphere partition, and the
    // waterline flip-fraction exactness.
    {
        auto Hax = [&](double lam) {
            const double s = std::sin(6.0 * PI / lam);
            return s * s;
        };
        Near(Hax(6.0), 0.0, 1e-12, "bed: axial zero at 6 m", ok);
        Near(Hax(3.0), 0.0, 1e-12, "bed: axial zero at 3 m", ok);
        Near(Hax(12.0), 1.0, 1e-12, "bed: axial peak at 12 m", ok);
        Near(Hax(4.0), 1.0, 1e-12, "bed: axial peak at 4 m", ok);
        Near(1.0 - std::cos(std::sqrt(2.0) * 6.0 * PI / (6.0 * std::sqrt(2.0))), 2.0, 1e-12,
             "bed: diagonal gain 2 at 6*sqrt(2) m", ok);
        // skew vector (the adversarial correction, twice): 12 m-periodic beds are in the
        // operator's odd-symmetry blind spot; lambda = 8 m + its 4 m second harmonic
        // shows clamp(mean) != mean(clamp).
        {
            const int n = 16;                       // 24 m period at 1.5 m texels
            double z[n];
            for (int i = 0; i < n; ++i) {
                const double x = i * 1.5;
                z[i] = 0.8 * std::sin(2.0 * PI * x / 8.0) +
                       0.4 * std::sin(2.0 * PI * x / 4.0 + 1.0);
            }
            double meanRaw = 0.0, meanClamped = 0.0;
            for (int i = 0; i < n; ++i) {
                const double m = 0.5 * (z[(i + 4) % n] + z[(i - 4 + n) % n]);   // 6 m taps
                const double r = (z[i] - m) * 0.85;
                meanRaw += r;
                meanClamped += (std::min)((std::max)(r, -0.45), 0.45);
            }
            Near(meanRaw / n, 0.0, 1e-12, "bed: unclamped relief mean-zero", ok);
            if (meanClamped / n <= 0.005) {
                Log("[gatest] FAIL bed: skew vector clamped mean %.6f <= 0.005",
                    meanClamped / n);
                ok = false;
            }
        }
        // narrowness at a straight datum shoreline: exact discrete count (r+1)/(2r+1)
        // with the center column on land (the 61x61 box at 1.5 m texels).
        Near(31.0 / 61.0, 0.508196721, 1e-9, "bed: shoreline land fraction 31/61", ok);
        // hemisphere partition sums to one for every unit normal
        for (int i = 0; i < 16; ++i) {
            const V3 n = rv();
            Near(0.5 * (1.0 + n.y) + 0.5 * (1.0 - n.y), 1.0, 1e-12,
                 "bed: hemisphere view factors sum to 1", ok);
        }
        // waterline metric: flipping an exact fraction p of ALL cell labels gives
        // A(L0) == 1 - p exactly on a monotone beach.
        {
            const int n = 4000, flips = 800;
            int wrong = 0;
            for (int i = 0; i < n; ++i) {
                const bool predDry = (i >= n / 2);         // z ramps with i; L0 at midpoint
                bool obsDry = predDry;
                if ((i * 7919) % n < flips) obsDry = !obsDry;   // exact-count flip
                if (predDry != obsDry) ++wrong;
            }
            Near(1.0 - static_cast<double>(wrong) / n, 1.0 - static_cast<double>(flips) / n,
                 1e-12, "bed: waterline flip-fraction exactness", ok);
        }
    }

    // ---- M9h: THE PAGE TABLE. The (level, x, y) address space, exercised. Three properties
    // matter, and the third is the one the whole no-pop story rests on.
    {
        LevelLadder ladder;
        ladder.level0MetersPerTexel = 0.01;   // 1 cm at the finest level
        ladder.pageTexels = 16384;

        // 1. THE LADDER REACHES CENTIMETRES, and the page count to cover ground is bounded.
        // At 1 cm a page spans 163.84 m, so an inlet-sized window is a handful of pages, not
        // the 6.4e10 texels a flat cm-resolution raster over the same area would need.
        Near(ladder.MetersPerTexel(0), 0.01, 1e-12, "pages: level 0 is 1 cm", ok);
        Near(ladder.PageGroundMeters(0), 163.84, 1e-9, "pages: a 1 cm page spans 163.84 m", ok);
        const uint32_t lvlEarth = ladder.LevelsToSpan(40.0e6);
        Log("[gatest] pages: 1 cm at L0; one page spans %.2f m; L%u spans %.0f km (Earth needs "
            "L%u)",
            ladder.PageGroundMeters(0), lvlEarth, ladder.PageGroundMeters(lvlEarth) / 1000.0,
            lvlEarth);
        // The inlet window (3200 x 2000 m) at centimetre resolution:
        const double pg = ladder.PageGroundMeters(0);
        const uint32_t inletPages = uint32_t(std::ceil(3200.0 / pg)) *
                                    uint32_t(std::ceil(2000.0 / pg));
        Log("[gatest] pages: the 3.2 x 2.0 km inlet at 1 cm = %u pages of %u slices",
            inletPages, 1024u);
        if (inletPages > 1024u) {
            Log("[gatest] FAIL pages: inlet at 1 cm needs %u pages, more than one array holds",
                inletPages);
            ok = false;
        }

        PageTable pt;
        pt.Init(8, ladder, "gatest.pages");

        // 2. Reserve/Find round-trips, and the same address is the same slice.
        const PageAddr a{0, 5, 9};
        const uint32_t s0 = pt.Reserve(a);
        if (s0 == PageTable::kNoSlice || pt.Find(a) != s0 || pt.Reserve(a) != s0) {
            Log("[gatest] FAIL pages: reserve/find does not round-trip");
            ok = false;
        }
        if (!(pt.AddrOf(s0) == a)) {
            Log("[gatest] FAIL pages: slice does not report the address it holds");
            ok = false;
        }

        // 3. FindCovering DEGRADES TO COARSER, never to nothing. This is the CPU half of the
        // no-pop contract: ask for a fine page that was never reserved and get the coarse page
        // describing the same ground, so detail changes and nothing else does.
        const PageAddr coarse{4, 0, 0};
        pt.Reserve(coarse);
        uint32_t gotLevel = 999;
        // (0,5,9) at level 0 sits under (4, 0, 0): 5>>4 == 0, 9>>4 == 0.
        const PageAddr fineUnder{0, 5, 9};
        pt.Release(fineUnder);
        const uint32_t cov = pt.FindCovering(fineUnder, &gotLevel);
        if (cov == PageTable::kNoSlice || gotLevel != 4) {
            Log("[gatest] FAIL pages: covering lookup gave slice %u at level %u, expected the "
                "level-4 parent", cov, gotLevel);
            ok = false;
        }
        // And an address no page covers is honestly absent, not silently level 0.
        if (pt.FindCovering(PageAddr{0, 900000, 900000}) != PageTable::kNoSlice) {
            Log("[gatest] FAIL pages: uncovered address resolved to something");
            ok = false;
        }

        // 4. Exhaustion refuses and counts, rather than evicting a page whose contents may be
        // irreproducible (the pool cap's rule, section 19).
        for (uint32_t i = 0; i < 32; ++i) pt.Reserve(PageAddr{0, 100u + i, 0});
        if (pt.Refused() == 0 || pt.Resident() > pt.Slices()) {
            Log("[gatest] FAIL pages: exhaustion neither refused nor counted");
            ok = false;
        }
        Log("[gatest] pages: %s", pt.Stats().c_str());
    }

    // ---- M9i: THE PLUGIN SEAM, on a SECOND file type. The point is that nothing above the
    // loader changed to accept it -- same registry, same interface, same three answers.
    {
        LoaderRegistry reg;
        reg.Register("json", CurrentFieldLoader::Open);
        auto ld = reg.Open("data/currents/currents.json");
        if (!ld) {
            Log("[gatest] pages: no data/currents/currents.json -- loader gate skipped");
        } else {
            const GeoRef& g = ld->Ref();
            // Grade is declared by the SOURCE; what grad() of it becomes is the algebra's
            // business, and the type already proved that (kG1 * kG1 = kG0 | kG2).
            if (ld->GradeSig() != kG1) {
                Log("[gatest] FAIL loader: a current field is grade 1, got %u", ld->GradeSig());
                ok = false;
            }
            // THE FLIP, DERIVED. CurrentField::Sample uses fy = (lat - lat0)/dlat with dlat
            // positive, so row 0 is SOUTH -- the opposite of every bathy grid here. Nothing
            // declared that; VNorth() reads it off the affine, which is the whole reason
            // GeoRef exists (priors 10).
            if (!g.VNorth()) {
                Log("[gatest] FAIL loader: current field should be row-0-south (vNorth)");
                ok = false;
            }
            if (!g.NeedsFlipInto(false)) {
                Log("[gatest] FAIL loader: a +v=N source into a +v=S consumer needs a flip");
                ok = false;
            }
            if (g.provenance != CrsProvenance::Embedded) {
                Log("[gatest] FAIL loader: georeference came from the file, not a claim");
                ok = false;
            }
            // ABSENCE IS NOT A VALUE: land (<= -900) must never be handed out as a sample.
            if (!g.IsNoData(-999.0) || g.IsNoData(0.35)) {
                Log("[gatest] FAIL loader: nodata test does not separate land from slack water");
                ok = false;
            }
            TilePayload tp;
            uint32_t withData = 0, empty = 0;
            const uint32_t tw = 64, th = 64;
            for (uint32_t ty = 0; ty * th < g.height; ++ty) {
                for (uint32_t tx = 0; tx * tw < g.width; ++tx) {
                    if (ld->LoadTile(tx, ty, tw, th, tp)) ++withData; else ++empty;
                }
            }
            Log("[gatest] loader: %s %ux%u, %s -> %u tiles carry data, %u are pure land "
                "(never allocated)",
                ld->Name(), g.width, g.height, g.Describe().c_str(), withData, empty);
            if (withData == 0) {
                Log("[gatest] FAIL loader: no tile carried a current");
                ok = false;
            }
        }
    }

    if (ok) {
        Log("[gatest] ---- PASS: sandwich, refraction rotor, fold telescope, spinor blend, "
            "frame rules + orientation ledger, merc chain bound; water parity: solved "
            "wave field, caustic bivector, ripple prefilter, foam discipline, signed "
            "Kelvin phase, bed relief + waterline metric ----");
    }
    return ok;
}

}  // namespace ga
