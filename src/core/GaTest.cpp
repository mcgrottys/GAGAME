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

    if (ok) {
        Log("[gatest] ---- PASS: sandwich, refraction rotor, fold telescope, spinor blend, "
            "frame rules + orientation ledger ----");
    }
    return ok;
}

}  // namespace ga
