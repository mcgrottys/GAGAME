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

// Brute force: the nearest of this level's points to d, by every ordinal.
int64_t BruteNearest(const OriginLattice& L, const double d[3]) {
    int64_t best = -1;
    double bd = -2.0;
    for (int64_t o = 0; o < L.Ordinals(); ++o) {
        const int64_t idx = L.IndexOfOrdinal(o);
        if (idx < 0) continue;
        double q[3];
        OriginLattice::Direction(idx, q);
        const double dd = q[0] * d[0] + q[1] * d[1] + q[2] * d[2];
        if (dd > bd) {
            bd = dd;
            best = idx;
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

    // 0. THE SOLID: twenty faces outward, every point of level 6 seen once through its owner.
    {
        OriginLattice S;
        S.level = 6;
        int64_t seen = 0;
        for (int64_t o = 0; o < S.Ordinals(); ++o) seen += (S.IndexOfOrdinal(o) >= 0);
        check(seen == S.Points(), "level 6: the ordinals name every point once (10 x 4^6 + 2)", double(seen - S.Points()));
        // Every face's three corners and its centre: the centre outward of the corners' plane.
        int inward = 0;
        for (int f = 0; f < 20; ++f) {
            double A[3], B[3], C[3];
            OriginLattice::Direction(OriginLattice::Encode(0, f, 0, 0), A);
            OriginLattice::Direction(OriginLattice::Encode(0, f, 1, 0), B);
            OriginLattice::Direction(OriginLattice::Encode(0, f, 0, 1), C);
            const double e1[3] = {B[0] - A[0], B[1] - A[1], B[2] - A[2]}, e2[3] = {C[0] - A[0], C[1] - A[1], C[2] - A[2]};
            const double n[3] = {e1[1] * e2[2] - e1[2] * e2[1], e1[2] * e2[0] - e1[0] * e2[2], e1[0] * e2[1] - e1[1] * e2[0]};
            if (n[0] * A[0] + n[1] * A[1] + n[2] * A[2] <= 0.0) ++inward;
        }
        check(inward == 0, "every face winds counter-clockwise seen from outside (inward faces)", inward);
    }

    // 1. THE INVERSE IS THE NEAREST: the closed form against brute force over every point of
    //    level 6 (40,962 points, 1000 directions) and level 8 (655,362 points, 100 directions).
    {
        for (int lv : {6, 8}) {
            OriginLattice S;
            S.level = lv;
            const int trials = lv == 6 ? 1000 : 100;
            int wrong = 0;
            for (int t = 0; t < trials; ++t) {
                double d[3];
                RandDir(rng, d);
                if (S.InverseIndex(d) != BruteNearest(S, d)) ++wrong;
            }
            char what[120];
            snprintf(what, sizeof(what), "level %d: inverse = brute-force nearest over %lld points, %d directions (wrong)", lv,
                     static_cast<long long>(S.Points()), trials);
            check(wrong == 0, what, wrong);
        }
        // ...and at the twelve vertices, the edges' midpoints and the poles, where ownership decides.
        OriginLattice S;
        S.level = 6;
        int wrongEdge = 0;
        for (int f = 0; f < 20; ++f) {
            for (const auto [i, j] : {std::pair<int64_t, int64_t>{0, 0}, {64, 0}, {0, 64}, {32, 0}, {0, 32}, {32, 32}}) {
                double d[3];
                OriginLattice::Direction(OriginLattice::Encode(6, f, i, j), d);
                if (S.InverseIndex(d) != BruteNearest(S, d)) ++wrongEdge;
            }
        }
        const double poles[2][3] = {{0, 1, 0}, {0, -1, 0}};
        for (const double* p : poles) {
            if (S.InverseIndex(p) != BruteNearest(S, p)) ++wrongEdge;
        }
        check(wrongEdge == 0, "inverse at the corners, the edges' midpoints and the poles (wrong)", wrongEdge);
        // THE PLANT: the gate sees a wrong index -- a neighbour of the right one is farther.
        double d[3];
        RandDir(rng, d);
        const int64_t i = S.InverseIndex(d);
        int face;
        int64_t ii, jj;
        OriginLattice::Decode(i, face, ii, jj);
        const int64_t step = int64_t(1) << (OriginLattice::kLmax - 6);
        const int64_t j = OriginLattice::Encode(6, face, ii / step + (ii / step < 60 ? 1 : -1), jj / step);
        double q[3], q1[3];
        OriginLattice::Direction(i, q);
        OriginLattice::Direction(j, q1);
        const double qd[3] = {q[0] - d[0], q[1] - d[1], q[2] - d[2]}, q1d[3] = {q1[0] - d[0], q1[1] - d[1], q1[2] - d[2]};
        check(Len3(q1d) > Len3(qd), "the plant: a neighbour of the nearest is farther (a wrong index would fail here)", Len3(q1d) / Len3(qd));
    }

    // 2. NESTING: every point of level 6 is a point of levels 7 and 10 -- the same index, the same
    //    direction to the bit -- and a coarse origin kept by a child needs no motor.
    {
        OriginLattice c6, c7, c10;
        c6.level = 6; c7.level = 7; c10.level = 10;
        int notNested = 0, moved = 0;
        for (int64_t o = 0; o < c6.Ordinals(); ++o) {
            const int64_t idx = c6.IndexOfOrdinal(o);
            if (idx < 0) continue;
            double d[3];
            OriginLattice::Direction(idx, d);
            if (c7.InverseIndex(d) != idx || c10.InverseIndex(d) != idx) ++notNested;
            // The origin at level 10 of this point IS the level-6 origin: identical frames.
            const OriginLattice::Origin a = c6.At(0, idx), b = c10.At(0, idx);
            if (!a.Same(b) || a.c[0] != b.c[0] || a.c[1] != b.c[1] || a.c[2] != b.c[2]) ++moved;
        }
        check(notNested == 0, "every level-6 point is a level-7 and a level-10 point, the same index (not nested)", notNested);
        check(moved == 0, "...and the same origin to the bit, so a child keeps its parent's with no motor (moved)", moved);
        // The shells nest the same way: every shell of level 6 is a shell of level 7 (its units a
        // multiple of the finer step), and the same radius to the bit through either lattice.
        const int64_t s6 = c6.ShellOf(c6.R * 1.01), step7 = int64_t(1) << (OriginLattice::kLmax - 7);
        check(s6 % step7 == 0 && c6.RadiusOf(s6) == c7.RadiusOf(s6), "a coarse shell is a fine shell, the same radius to the bit (coarse shell mod fine step)", double(s6 % step7));
    }

    // 3. THE FLOAT BOUND ON EVERY SHELL: a point's floats about its origin are within the level's
    //    spacing (the cell's half-diagonal is under 0.71 s laterally plus half a shell gap), on the
    //    surface, 100 km up, in orbit at 20,000 km, and 3,000 km down.
    {
        OriginLattice L;
        const double radii[4] = {L.R, L.R + 100000.0, L.R + 20000000.0, L.R - 3000000.0};
        const char* names[4] = {"surface", "100 km up", "20,000 km up (orbit)", "3,000 km down"};
        for (int k = 0; k < 4; ++k) {
            double worst = 0.0, grain = 0.0;
            int64_t shellSeen = 0;
            for (int t = 0; t < 2000; ++t) {
                double d[3];
                RandDir(rng, d);
                std::uniform_real_distribution<double> jr(-0.5, 0.5);
                const double r = radii[k] * std::exp(jr(rng) * L.Eps());   // anywhere between two shells
                const double P[3] = {d[0] * r, d[1] * r, d[2] * r};
                const OriginLattice::Origin o = L.Nearest(P);
                shellSeen = o.shell;
                double f[3];
                OriginLattice::Floats(o, P, f);
                worst = std::max(worst, Len3(f));
                const float fx = float(f[0]), fy = float(f[1]), fz = float(f[2]);
                grain = std::max(grain, std::max(std::fabs(double(fx) - f[0]), std::max(std::fabs(double(fy) - f[1]), std::fabs(double(fz) - f[2]))));
                double Q[3];
                OriginLattice::Point(o, f, Q);
                if (std::fabs(Q[0] - P[0]) + std::fabs(Q[1] - P[1]) + std::fabs(Q[2] - P[2]) > 1e-6 * r) ok = false;
            }
            const double s = L.Spacing(shellSeen);
            const double bound = 0.71 * s + 0.5 * s;
            char what[180];
            snprintf(what, sizeof(what), "%s: shell %lld, spacing %.1f km, floats within %.1f km (bound %.1f), grain %.3g m",
                     names[k], static_cast<long long>(shellSeen), s / 1000.0, worst / 1000.0, bound / 1000.0, grain);
            check(worst <= bound, what, worst / bound);
        }
    }

    // 4. THE MOTOR BETWEEN ORIGINS: floats about i, through M_ij, are the floats about j -- between
    //    neighbours, across the planet, and across shells.
    {
        OriginLattice L;
        double worst = 0.0;
        for (int t = 0; t < 300; ++t) {
            double d[3];
            RandDir(rng, d);
            std::uniform_int_distribution<int> sh(-3, 3);
            const double r = L.R * std::exp(sh(rng) * L.Eps() * 0.9);
            const double P[3] = {d[0] * r, d[1] * r, d[2] * r};
            const OriginLattice::Origin a = L.Nearest(P);
            double d2[3];
            RandDir(rng, d2);
            const double P2[3] = {d2[0] * r * 1.0007, d2[1] * r * 1.0007, d2[2] * r * 1.0007};
            const OriginLattice::Origin b = (t & 1) ? L.Nearest(P2) : L.At(a.shell + (t % 4 == 0 ? (int64_t(1) << (OriginLattice::kLmax - L.level)) : 0), a.index);
            double fa[3], fb[3], fm[3];
            OriginLattice::Floats(a, P, fa);
            OriginLattice::Floats(b, P, fb);
            fm[0] = fa[0]; fm[1] = fa[1]; fm[2] = fa[2];
            OriginLattice::Between(a, b).TransformPoint(fm[0], fm[1], fm[2]);
            worst = std::max(worst, std::fabs(fm[0] - fb[0]) + std::fabs(fm[1] - fb[1]) + std::fabs(fm[2] - fb[2]));
        }
        check(worst < 1e-5, "M_ij takes floats about i to floats about j, neighbours and antipodes alike (worst m)", worst);
    }

    // 5. HYSTERESIS: a point walked out two spacings and back crosses cells on the way; every
    //    switch going out comes LATER (farther out) than the matching switch coming back, so the
    //    loop has width and a point resting on a boundary never flaps.
    {
        OriginLattice L;
        double d0[3];
        RandDir(rng, d0);
        const OriginLattice::Origin a = L.Surface(d0);
        const double P0[3] = {a.c[0], a.c[1], a.c[2]};
        const double step[3] = {a.anchor.east[0], a.anchor.east[1], a.anchor.east[2]};
        const double s = L.Spacing(0);
        OriginLattice::Origin cur = a;
        std::vector<std::pair<double, int64_t>> out, back;
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
        bool later = true;
        for (size_t k = 0; k < out.size() && k < back.size(); ++k) {
            if (!(out[k].first > back[back.size() - 1 - k].first)) later = false;
        }
        check(later, "every switch out lies farther out than its twin coming back: the loop has width");
        check(cur.Same(a), "home again: the walk ends in the origin it started from");
        if (!out.empty() && !back.empty()) {
            check((out[0].first - back.back().first) / s > 0.05, "the first boundary's hysteresis width (spacings)", (out[0].first - back.back().first) / s);
        }
    }

    // 6. THE CENTRE: a point under rMin lives about the centre, its floats the point itself.
    {
        OriginLattice L;
        const double P[3] = {3000.0, -4000.0, 5000.0};
        const OriginLattice::Origin o = L.Nearest(P);
        double f[3];
        OriginLattice::Floats(o, P, f);
        check(o.centre && f[0] == P[0] && f[1] == P[1] && f[2] == P[2], "under rMin the centre is the origin and the floats are the point");
        const OriginLattice::Origin s = L.SurfaceAt(42.8, -70.8);
        double g[3];
        OriginLattice::Floats(s, P, g);
        double h[3] = {f[0], f[1], f[2]};
        OriginLattice::Between(o, s).TransformPoint(h[0], h[1], h[2]);
        check(std::fabs(h[0] - g[0]) + std::fabs(h[1] - g[1]) + std::fabs(h[2] - g[2]) < 1e-6, "centre -> surface origin by the motor (m)",
              std::fabs(h[0] - g[0]) + std::fabs(h[1] - g[1]) + std::fabs(h[2] - g[2]));
    }

    // 7. THE NUMBERS, said once.
    {
        OriginLattice L;
        Log("[origin] the lattice: level %d (%lld points a shell), shells %.3f%% apart; surface spacing %.2f km, grain %.3g mm; orbit at "
            "20,000 km: spacing %.0f km, grain %.3g m; the centre under %.0f km; the identity at level %d",
            L.level, static_cast<long long>(L.Points()), 100.0 * L.Eps(), L.Spacing(0) / 1000.0, L.Spacing(0) * 0.71 / 8388608.0 * 1000.0,
            L.Spacing(L.ShellOf(L.R + 20000000.0)) / 1000.0, L.Spacing(L.ShellOf(L.R + 20000000.0)) * 0.71 / 8388608.0, L.rMin / 1000.0,
            OriginLattice::kLmax);
    }

    Log("[origin] %s (%d checks)", ok ? "selftest PASS" : "selftest FAIL", checks);
    return ok;
}

}  // namespace ga
