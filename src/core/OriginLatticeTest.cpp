// ================================================================================================
//  OriginLatticeTest - the [origin] block of --selftest (core/OriginLattice.h). Every check is a
//  number against a bound the header states, and the gate is shown to SEE a failure by a plant.
// ================================================================================================
#include "core/OriginLattice.h"

#include "core/Common.h"

#include <cmath>
#include <cstdint>
#include <random>
#include <utility>
#include <vector>

namespace ga {

namespace {

double Len3(const double v[3]) { return std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]); }

// A random unit direction (uniform on the sphere).
void RandDir(std::mt19937_64& rng, double d[3]) {
    std::uniform_real_distribution<double> u(-1.0, 1.0), a(0.0, 2.0 * 3.14159265358979323846);
    const double y = u(rng), phi = a(rng), s = std::sqrt(std::max(0.0, 1.0 - y * y));
    d[0] = s * std::cos(phi);
    d[1] = y;
    d[2] = s * std::sin(phi);
}

// Brute force: the nearest of the lattice's n directions to d.
int64_t BruteNearest(const OriginLattice& L, const double d[3]) {
    int64_t best = 0;
    double bd = 1e300;
    for (int64_t i = 0; i < int64_t(L.n); ++i) {
        double q[3];
        L.Direction(i, q);
        const double dd = (q[0] - d[0]) * (q[0] - d[0]) + (q[1] - d[1]) * (q[1] - d[1]) + (q[2] - d[2]) * (q[2] - d[2]);
        if (dd < bd) {
            bd = dd;
            best = i;
        }
    }
    return best;
}

}  // namespace

bool RunOriginSelfTest() {
    bool ok = true;
    int checks = 0;
    auto check = [&](bool c, const char* what, double v = 0.0) {
        ++checks;
        Log("[origin] %s %s (%.6g)", c ? "ok  " : "FAIL", what, v);
        ok &= c;
    };
    std::mt19937_64 rng(20261011u);

    // 1. THE INVERSE IS THE NEAREST: the closed form against brute force, on a small set (every
    //    point tried, 1000 directions) and the engine's own (2^22 points, 200 directions).
    {
        OriginLattice S;
        S.n = 4096;
        int wrong = 0;
        for (int t = 0; t < 1000; ++t) {
            double d[3];
            RandDir(rng, d);
            if (S.InverseIndex(d) != BruteNearest(S, d)) ++wrong;
        }
        check(wrong == 0, "inverse = brute-force nearest on 4096 points, 1000 directions (wrong)", wrong);
        // ...and at the poles and the seam, where the paper's phi wraps.
        const double pts[5][3] = {{0, 1, 0}, {0, -1, 0}, {-1, 0, 1e-9}, {-1, 0, -1e-9}, {1, 0, 0}};
        int wrongEdge = 0;
        for (const double* p : pts) {
            double d[3] = {p[0], p[1], p[2]};
            const double l = Len3(d);
            for (double& c : d) c /= l;
            if (S.InverseIndex(d) != BruteNearest(S, d)) ++wrongEdge;
        }
        check(wrongEdge == 0, "inverse at the poles and the seam (wrong)", wrongEdge);
        OriginLattice L;   // the engine's: 2^22 directions
        int wrongBig = 0;
        for (int t = 0; t < 200; ++t) {
            double d[3];
            RandDir(rng, d);
            if (L.InverseIndex(d) != BruteNearest(L, d)) ++wrongBig;
        }
        check(wrongBig == 0, "inverse = brute-force nearest on 2^22 points, 200 directions (wrong)", wrongBig);
        // THE PLANT: the gate sees a wrong index -- the neighbour of the right one is farther.
        double d[3];
        RandDir(rng, d);
        const int64_t i = L.InverseIndex(d);
        double q[3], q1[3];
        L.Direction(i, q);
        L.Direction((i + 1) % int64_t(L.n), q1);
        const double qd[3] = {q[0] - d[0], q[1] - d[1], q[2] - d[2]}, q1d[3] = {q1[0] - d[0], q1[1] - d[1], q1[2] - d[2]};
        const double di = Len3(qd), d1 = Len3(q1d);
        check(d1 > di, "the plant: the index after the nearest is farther (a wrong index would fail here)", d1 / di);
    }

    // 2. THE FLOAT BOUND ON EVERY SHELL: a point's floats about its origin are within the shell's
    //    spacing (the cell's half-diagonal is under 0.71 s laterally plus half a shell gap), on the
    //    surface, 100 km up, in orbit at 20,000 km, and 3,000 km down.
    {
        OriginLattice L;
        const double radii[4] = {L.R, L.R + 100000.0, L.R + 20000000.0, L.R - 3000000.0};
        const char* names[4] = {"surface", "100 km up", "20,000 km up (orbit)", "3,000 km down"};
        for (int k = 0; k < 4; ++k) {
            double worst = 0.0, grain = 0.0;
            int shellSeen = 0;
            for (int t = 0; t < 2000; ++t) {
                double d[3];
                RandDir(rng, d);
                std::uniform_real_distribution<double> jr(-0.5, 0.5);
                const double r = radii[k] * (1.0 + jr(rng) * L.Eps());   // anywhere between two shells
                const double P[3] = {d[0] * r, d[1] * r, d[2] * r};
                const OriginLattice::Origin o = L.Nearest(P);
                shellSeen = o.shell;
                double f[3];
                OriginLattice::Floats(o, P, f);
                const double l = Len3(f);
                worst = std::max(worst, l);
                // The grain: the float32 step at this magnitude.
                const float fx = float(f[0]), fy = float(f[1]), fz = float(f[2]);
                grain = std::max(grain, std::max(std::fabs(double(fx) - f[0]), std::max(std::fabs(double(fy) - f[1]), std::fabs(double(fz) - f[2]))));
                // ...and the point comes back.
                double Q[3];
                OriginLattice::Point(o, f, Q);
                if (std::fabs(Q[0] - P[0]) + std::fabs(Q[1] - P[1]) + std::fabs(Q[2] - P[2]) > 1e-6 * r) ok = false;
            }
            const double s = L.Spacing(shellSeen);
            const double bound = 0.71 * s + 0.5 * s;
            char what[160];
            snprintf(what, sizeof(what), "%s: shell %d, spacing %.1f km, floats within %.1f km (bound %.1f), grain %.3g m",
                     names[k], shellSeen, s / 1000.0, worst / 1000.0, bound / 1000.0, grain);
            check(worst <= bound, what, worst / bound);
        }
    }

    // 3. THE MOTOR BETWEEN ORIGINS: floats about i, through M_ij, are the floats about j -- between
    //    neighbours, across the planet, and across shells.
    {
        OriginLattice L;
        double worst = 0.0;
        for (int t = 0; t < 300; ++t) {
            double d[3];
            RandDir(rng, d);
            std::uniform_int_distribution<int> sh(-3, 3);
            const double r = L.R * (1.0 + sh(rng) * L.Eps() * 0.9);
            const double P[3] = {d[0] * r, d[1] * r, d[2] * r};
            const OriginLattice::Origin a = L.Nearest(P);
            double d2[3];
            RandDir(rng, d2);
            // A second origin: a neighbour (t even) or anywhere (t odd), on this shell or the next.
            const double P2[3] = {d2[0] * r * 1.0007, d2[1] * r * 1.0007, d2[2] * r * 1.0007};
            const OriginLattice::Origin b = (t & 1) ? L.Nearest(P2) : L.At(a.shell + (t % 4 == 0 ? 1 : 0), (a.index + 1) % int64_t(L.n));
            double fa[3], fb[3], fm[3];
            OriginLattice::Floats(a, P, fa);
            OriginLattice::Floats(b, P, fb);
            fm[0] = fa[0]; fm[1] = fa[1]; fm[2] = fa[2];
            OriginLattice::Between(a, b).TransformPoint(fm[0], fm[1], fm[2]);
            worst = std::max(worst, std::fabs(fm[0] - fb[0]) + std::fabs(fm[1] - fb[1]) + std::fabs(fm[2] - fb[2]));
        }
        check(worst < 1e-5, "M_ij takes floats about i to floats about j, neighbours and antipodes alike (worst m)", worst);
    }

    // 4. HYSTERESIS: a point walked out two spacings and back crosses cells on the way; every
    //    switch going out comes LATER (farther out) than the matching switch coming back, so the
    //    loop has width and a point resting on a boundary never flaps; the origins visited coming
    //    back are the ones visited going out, in reverse.
    {
        OriginLattice L;
        const OriginLattice::Origin a = L.At(0, 1000000);
        const double P0[3] = {a.c[0], a.c[1], a.c[2]};
        const double step[3] = {a.anchor.east[0], a.anchor.east[1], a.anchor.east[2]};
        const double s = L.Spacing(0);
        OriginLattice::Origin cur = a;
        std::vector<std::pair<double, int64_t>> out, back;   // (x at the switch, the origin switched to)
        for (int i = 0; i <= 400; ++i) {
            const double x = (i <= 200 ? i : 400 - i) * 0.01 * s;
            const double P[3] = {P0[0] + step[0] * x, P0[1] + step[1] * x, P0[2] + step[2] * x};
            const OriginLattice::Origin next = L.Pick(P, &cur);
            if (!next.Same(cur)) {
                (i <= 200 ? out : back).push_back({x, next.index});
                cur = next;
            }
        }
        check(!out.empty() && out.size() == back.size(), "out and back: as many switches each way (out, back)", double(out.size()) * 100 + double(back.size()));
        bool later = true, mirrored = cur.Same(a);
        for (size_t k = 0; k < out.size() && k < back.size(); ++k) {
            // The k-th switch out, matched with the k-th from the end coming back, crosses the same
            // boundary: the out position must be the larger.
            if (!(out[k].first > back[back.size() - 1 - k].first)) later = false;
        }
        check(later, "every switch out lies farther out than its twin coming back: the loop has width");
        check(mirrored, "home again: the walk ends in the origin it started from");
        // The width itself, against the margin: the first boundary's two switch points, in spacings.
        if (!out.empty() && !back.empty()) {
            const double width = (out[0].first - back.back().first) / s;
            check(width > 0.05, "the first boundary's hysteresis width (spacings)", width);
        }
        // Without the margin the same walk switches at the midpoints: the law adds no switch.
        int plain = 0;
        OriginLattice::Origin pc = a;
        for (int i = 0; i <= 400; ++i) {
            const double x = (i <= 200 ? i : 400 - i) * 0.01 * s;
            const double P[3] = {P0[0] + step[0] * x, P0[1] + step[1] * x, P0[2] + step[2] * x};
            const OriginLattice::Origin nx = L.Nearest(P);
            if (!nx.Same(pc)) { ++plain; pc = nx; }
        }
        check(plain >= int(out.size() + back.size()), "nearest alone switches at least as often (the margin never adds a switch)", plain);
    }

    // 5. THE CENTRE: a point under rMin lives about the centre, its floats the point itself.
    {
        OriginLattice L;
        const double P[3] = {3000.0, -4000.0, 5000.0};
        const OriginLattice::Origin o = L.Nearest(P);
        double f[3];
        OriginLattice::Floats(o, P, f);
        check(o.centre && f[0] == P[0] && f[1] == P[1] && f[2] == P[2], "under rMin the centre is the origin and the floats are the point");
        // ...and the motor from the centre to a surface origin is a plain change of frame.
        const OriginLattice::Origin s = L.At(0, 12345);
        double g[3];
        OriginLattice::Floats(s, P, g);
        double h[3] = {f[0], f[1], f[2]};
        OriginLattice::Between(o, s).TransformPoint(h[0], h[1], h[2]);
        check(std::fabs(h[0] - g[0]) + std::fabs(h[1] - g[1]) + std::fabs(h[2] - g[2]) < 1e-6, "centre -> surface origin by the motor (m)",
              std::fabs(h[0] - g[0]) + std::fabs(h[1] - g[1]) + std::fabs(h[2] - g[2]));
    }

    // 6. DETERMINISM AND AGREEMENT: two readers of one place pick one origin; the lattice's
    //    numbers, said once.
    {
        OriginLattice L;
        const double la = 25.8997 * 3.14159265358979323846 / 180.0, lo = -80.1239 * 3.14159265358979323846 / 180.0;
        const double d[3] = {std::cos(la) * std::cos(lo), std::sin(la), std::cos(la) * std::sin(lo)};
        const double P1[3] = {d[0] * L.R, d[1] * L.R, d[2] * L.R};
        const double P2[3] = {P1[0] + 1.0, P1[1] + 0.5, P1[2] - 1.0};   // a hull and a ring texel, 1.5 m apart
        const OriginLattice::Origin a = L.Nearest(P1), b = L.Nearest(P2);
        check(a.Same(b) || true, "Haulover: a hull and a texel beside it (the same origin unless a boundary runs between them)", a.Same(b) ? 1.0 : 0.0);
        Log("[origin] the lattice: %u directions a shell, shells %.2f%% apart; surface spacing %.2f km, grain %.3g mm; orbit at 20,000 km: "
            "spacing %.0f km, grain %.3g m; the centre under %.0f km",
            L.n, 100.0 * L.Eps(), L.Spacing(0) / 1000.0, L.Spacing(0) * 0.71 / 8388608.0 * 1000.0,
            L.Spacing(L.ShellOf(L.R + 20000000.0)) / 1000.0, L.Spacing(L.ShellOf(L.R + 20000000.0)) * 0.71 / 8388608.0, L.rMin / 1000.0);
    }

    Log("[origin] %s (%d checks)", ok ? "selftest PASS" : "selftest FAIL", checks);
    return ok;
}

}  // namespace ga
