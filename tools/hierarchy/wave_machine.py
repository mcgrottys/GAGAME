#!/usr/bin/env python3
"""HIERARCHY 4.16: what a piston bound to a band of music can do to the water.

Three questions, answered by arithmetic and by one brute-force integral. Nothing here touches
the engine.

  1. A piston that follows a beat of f hertz makes waves of what length, moving how fast, and
     which rank of the pyramid has texels small enough to carry them?
  2. How much wave does a stroke make? Wavemaker theory's ratio for a piston (Havelock 1929;
     Dean and Dalrymple 1991), with its two limits checked.
  3. Is the water made by a source whose history is known a FUNCTION of the place and the
     instant? In one dimension, deep water, linear theory: the surface far from a source driven
     in a narrow band, by the full integral over every wavenumber and the whole history,
     against the closed form  eta = s(k0) / (2 cg) * E(t - x / cg) * cos(k0 x - w0 t),
     which reads the band's envelope E at the retarded instant and keeps no state.

Pure Python on purpose: this machine's Python has no numpy.
"""
import math

G = 9.80665
SIGMA_RHO = 0.0728 / 1025.0          # surface tension over density, sea water, m^3 / s^2
FACE_TEXEL_M = 611.496               # the cube lattice's rung 0 at the face centre
RANKS = [(rank, 3 * rank, FACE_TEXEL_M / 2 ** (3 * rank)) for rank in range(6)]


def wavenumber(f_hz, depth_m=None, capillary=True):
    """k of w^2 = (g k + (sigma/rho) k^3) tanh(k h), by Newton from the deep-water guess."""
    w2 = (2.0 * math.pi * f_hz) ** 2
    k = w2 / G
    for _ in range(60):
        t = 1.0 if depth_m is None else math.tanh(k * depth_m)
        dt = 0.0 if depth_m is None else depth_m * (1.0 - t * t)
        s = SIGMA_RHO if capillary else 0.0
        a = G * k + s * k ** 3
        da = G + 3.0 * s * k * k
        f = a * t - w2
        df = da * t + a * dt
        step = f / df
        k -= step
        if abs(step) < 1e-14 * k:
            break
    return k


def group_speed(f_hz, depth_m=None):
    """dw/dk by a centred difference on the relation itself."""
    k = wavenumber(f_hz, depth_m)
    def w(kk):
        t = 1.0 if depth_m is None else math.tanh(kk * depth_m)
        return math.sqrt((G * kk + SIGMA_RHO * kk ** 3) * t)
    h = 1e-5 * k
    return (w(k + h) - w(k - h)) / (2.0 * h)


def piston_ratio(kh):
    """Wave height over stroke, piston type (Dean and Dalrymple 1991, eq. 6.25)."""
    return 2.0 * (math.cosh(2.0 * kh) - 1.0) / (math.sinh(2.0 * kh) + 2.0 * kh)


def part1():
    print("1. A beat, the wave it makes, and the rank that carries it (deep water)")
    print("   ranks: " + ", ".join(f"{r} = rung {rung} = {t:.4g} m" for r, rung, t in RANKS))
    print(f"   {'beat Hz':>8} {'bpm':>5} {'length m':>9} {'phase m/s':>10} {'group m/s':>10} "
          f"{'capillary':>10} {'breaks at m':>12} {'rank':>5} {'texels/wave':>12}")
    for f in (0.25, 0.5, 1.0, 2.0, 3.0, 4.0, 8.0):
        k = wavenumber(f)
        k_grav = wavenumber(f, capillary=False)
        lam = 2.0 * math.pi / k
        cp = 2.0 * math.pi * f / k
        cg = group_speed(f)
        # The coarsest rank with at least eight texels to a wavelength.
        rank = next((r for r, _, t in RANKS if lam / t >= 8.0), None)
        per = lam / RANKS[rank][2] if rank is not None else float("nan")
        print(f"   {f:8.2f} {60 * f:5.0f} {lam:9.3f} {cp:10.3f} {cg:10.3f} "
              f"{100 * (k_grav / k - 1):9.1f}% {lam / 7.0:12.3f} "
              f"{'none' if rank is None else rank:>5} {per:12.1f}")
    print("   'capillary' is how much shorter gravity alone would make the wave; 'breaks at' is a")
    print("   seventh of the length, the steepest wave that stands.")


def part2():
    print("\n2. Wave height over stroke, a piston in water of depth h")
    print(f"   {'kh':>6} {'H / S':>8} {'shallow limit kh':>17}")
    for kh in (0.05, 0.1, 0.3, 1.0, 2.0, 3.0, 6.0):
        print(f"   {kh:6.2f} {piston_ratio(kh):8.4f} {kh:17.4f}")
    lo, hi = piston_ratio(1e-4) / 1e-4, piston_ratio(20.0)
    print(f"   limits: H / (S kh) -> {lo:.6f} as kh -> 0 (want 1);  H / S -> {hi:.6f} as kh -> "
          f"infinity (want 2)")
    print("   a 2 Hz beat in 3 m of water is kh = "
          f"{wavenumber(2.0, 3.0) * 3.0:.1f}: the deep limit; a 0.25 Hz one is kh = "
          f"{wavenumber(0.25, 3.0) * 3.0:.2f}")


def part3():
    print("\n3. A known source makes a known sea: the full integral against the closed form")
    f0, sigma, t_env, t_now = 1.0, 0.15, 20.0, 30.0
    w0 = 2.0 * math.pi * f0
    k0 = w0 * w0 / G                      # gravity alone here: the closed form's own relation
    cg = G / (2.0 * w0)
    def env(tau):                         # one slow swell of the band's level, 0 outside
        return 0.5 - 0.5 * math.cos(2.0 * math.pi * tau / t_env) if 0.0 <= tau <= t_env else 0.0
    def shat(k):                          # the source's shape, a Gaussian sigma wide
        return sigma * math.sqrt(2.0 * math.pi) * math.exp(-0.5 * (k * sigma) ** 2)

    # The history, once for every wavenumber: I(k, t) = cos(w t) C(k) + sin(w t) S(k), with
    # C and S the cosine and sine transforms of the forcing q = E cos(w0 tau) over [0, t_env].
    nk, kmax, ntau = 2400, 3.0 * k0, 4000
    dk, dtau = kmax / nk, t_env / ntau
    taus = [(j + 0.5) * dtau for j in range(ntau)]
    q = [env(t) * math.cos(w0 * t) for t in taus]
    hist = []
    for i in range(nk):
        k = (i + 0.5) * dk
        w = math.sqrt(G * k)
        c = s = 0.0
        cw, sw = math.cos(w * dtau), math.sin(w * dtau)
        ca, sa = math.cos(w * taus[0]), math.sin(w * taus[0])
        for j in range(ntau):             # a rotation a step: no trig in the inner loop
            c += q[j] * ca
            s += q[j] * sa
            ca, sa = ca * cw - sa * sw, sa * cw + ca * sw
        hist.append((k, shat(k) * dtau * (math.cos(w * t_now) * c + math.sin(w * t_now) * s)))

    def full(x):
        return sum(h * math.cos(k * x) for k, h in hist) * dk / math.pi
    def closed(x):
        return shat(k0) / (2.0 * cg) * env(t_now - x / cg) * math.cos(k0 * x - w0 * t_now)

    peak = shat(k0) / (2.0 * cg)
    print(f"   band {f0} Hz: length {2 * math.pi / k0:.3f} m, group speed {cg:.3f} m/s; the band's "
          f"level swells once over {t_env:.0f} s; read at t = {t_now:.0f} s")
    print(f"   {'x m':>7} {'retarded s':>11} {'full':>11} {'closed':>11} {'difference':>11}")
    worst = 0.0
    n = 0
    x = 6.0
    while x <= 25.0:
        a, b = full(x), closed(x)
        worst = max(worst, abs(a - b))
        if n % 16 == 0:
            print(f"   {x:7.2f} {t_now - x / cg:11.2f} {a:11.5f} {b:11.5f} {a - b:11.5f}")
        n += 1
        x += 0.0975                       # a sixteenth of a wavelength: every phase is met
    print(f"   {n} places from 6 to 25 m: the largest difference is {worst:.5f}, "
          f"{100 * worst / peak:.1f} % of the closed form's peak {peak:.5f}")
    print("   The closed form keeps no state: it reads the envelope at t - x / cg. What it leaves")
    print("   out is the band's own spreading, which grows with the distance travelled.")


if __name__ == "__main__":
    part1()
    part2()
    part3()
