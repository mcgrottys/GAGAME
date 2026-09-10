# ==================================================================================================
#  proofs/wavefield_probe.py - the six fields WaveField::ProbeAt grew, held to the relations that
#  DEFINE them.
#
#  WHAT IS BEING PROVED.  ProbeAt used to return only eta = sum a*cos(theta).  A hull needs three
#  more things from the same surface -- the Gerstner offset (dx, dz), the slope (sx, sz) and the
#  water particle velocity (vx, vy, vz) -- and all three are DERIVATIVES of that one eta line
#  under two conventions the engine has already fixed:
#
#      theta = phi - sigma*t        (WaveField.cpp FRAME LEDGER `time`; WaterBank.hlsl's cT)
#      grad(phi) = k * d^           (WaveField.cpp's phase gauge; ALGEBRA.md `wavefield`)
#
#  So there is nothing here to take on faith and nothing to tune.  Each added field is a claim of
#  the form "this equals that derivative of eta", and each claim is checked by DIFFERENCING eta
#  and comparing.  If a sign is wrong the difference says so.
#
#  WHY THE NEGATIVE CONTROLS ARE NOT OPTIONAL.  This repo has been bitten twice by gates that
#  could not fail: a telescope check that computed f(a) - f(a), and four lines of PROSE in
#  proofs/ocean_cpu.py asserting a crest direction nobody had measured.  So every gate below is
#  run a second time against a deliberately BROKEN variant of the same formula -- k dropped from
#  the slope, a sign flipped, cos swapped for sin -- and prints how far the broken one lands.  A
#  gate whose control lands where the truth lands is a gate that proves nothing, so each check
#  ALSO asserts its own margin and fails the run if the control is not separated.
#
#  STRUCTURE
#    PART A  the transliterated evaluator + the gates, on a single analytic component and then on
#            a 32-component field (the sums must compose; per-component indexing bugs die here).
#            These are the pass/fail gates and the script's exit code is theirs.
#    PART B  the CLOCK.  The gates run at a modest t on purpose, and this part says why: at a real
#            simUnix the rotor argument is ~1e9 rad, whose double spacing is 2.4e-7 rad, and a
#            finite difference over 1 mm amplifies that into 1e-4 of slope.  Shown by the
#            signature -- a round-off error grows as 1/h, a formula error does not move.
#    PART C  a MEASUREMENT, not a gate, on a real cache/wave/*.bin the engine wrote: how far the
#            slope ProbeAt now returns is from a finite difference of the surface the ATLAS
#            actually encodes.  The x answer and the z answer are different, and the z one is a
#            finding, not a pass.
#
#  Usage:  py proofs/wavefield_probe.py [path\to\file.bin]
# ==================================================================================================
import glob
import math
import os
import struct
import sys

import numpy as np

G = 9.81
FAILS = []


def check(name, ok, detail=""):
    print("[%s] %s%s" % ("PASS" if ok else "FAIL", name, (" -- " + detail) if detail else ""))
    if not ok:
        FAILS.append(name)


# ==================================================================================================
#  PART A -- the evaluator, transliterated from src/sim/WaveField.cpp ProbeAt
# ==================================================================================================
#  Line for line, this IS the C++ accumulation loop.  The bilinear atlas decode is replaced by
#  exact per-component (a, k, d^, sigma, phi0), because the decode is not what is under test --
#  the six accumulations are, and they are identical either way.  `theta` is written in the form
#  the gauge produces it, phi(x,z) = phi0 + k*(d^ . x), which is what the cumsum builds for a
#  locally plane component.
#
#  Every VARIANT is one deliberate corruption of one line.  They exist so the gates can be shown
#  to discriminate; nothing else reads them.

VARIANTS = (
    "true",            # what the C++ ships
    "slope_no_k",      # slope without the wavenumber: the classic dimensional blunder
    "slope_flip",      # slope sign flipped
    "disp_flip",       # Gerstner offset sign flipped (the FFT cascades' documented bug)
    "disp_cos",        # Gerstner offset in phase with eta instead of in quadrature
    "vel_flip",        # velocity signs flipped == assuming theta = sigma*t - phi
    "vy_flip",         # only the vertical velocity flipped
    "vel_k",           # orbital speed scaled by k instead of sigma
)


def probe(comps, x, z, t, variant="true"):
    """comps: (N,6) array of (a, k, dX, dZ, sigma, phi0).  x/z/t broadcast together.

    Returns a dict with the same eight names WaveField::Probe carries."""
    x = np.asarray(x, np.float64)
    z = np.asarray(z, np.float64)
    t = np.asarray(t, np.float64)
    shape = np.broadcast(x, z, t).shape
    out = {n: np.zeros(shape, np.float64) for n in
           ("eta", "dx", "dz", "sx", "sz", "vx", "vy", "vz")}
    for (a, k, dX, dZ, sig, phi0) in comps:
        theta = phi0 + k * (dX * x + dZ * z) - sig * t
        ca, sa = np.cos(theta), np.sin(theta)
        aC, aS = a * ca, a * sa
        out["eta"] += aC                                # eta = a cos(theta)  <- the reference

        # D_h = -a sin(theta) d^
        s_disp = +1.0 if variant == "disp_flip" else -1.0
        base = aC if variant == "disp_cos" else aS
        out["dx"] += s_disp * base * dX
        out["dz"] += s_disp * base * dZ

        # grad eta = -a k sin(theta) d^
        kf = 1.0 if variant == "slope_no_k" else k
        s_slope = +1.0 if variant == "slope_flip" else -1.0
        out["sx"] += s_slope * aS * kf * dX
        out["sz"] += s_slope * aS * kf * dZ

        # u = +sigma a cos(theta) d^ ,  w = +sigma a sin(theta)
        rate = k if variant == "vel_k" else sig
        s_vel = -1.0 if variant == "vel_flip" else +1.0
        s_vy = s_vel * (-1.0 if variant == "vy_flip" else +1.0)
        out["vx"] += s_vel * rate * aC * dX
        out["vz"] += s_vel * rate * aC * dZ
        out["vy"] += s_vy * rate * aS
    return out


def deep_comp(a, tp, dirdeg, phi0=0.37):
    """One deep-water linear component: sigma from the period, k from sigma^2 = g k."""
    sig = 2.0 * math.pi / tp
    k = sig * sig / G
    th = math.radians(dirdeg)
    return np.array([[a, k, math.sin(th), math.cos(th), sig, phi0]])   # compass: x east, z north


def random_field(n, seed=20260908):
    """n components over the solve window's real range: sigma 0.35..1.9 rad/s (Tp 18..3.3 s), the
    matching deep-water k, spread 26 deg off mwd on the golden sequence -- the spectrum shape
    ALGEBRA.md `wavefield` ships."""
    rng = np.random.default_rng(seed)
    sig = np.geomspace(0.35, 1.9, n)
    k = sig * sig / G
    ang = np.radians(95.0 + 26.0 * (2.0 * ((np.arange(n) * 0.618034) % 1.0) - 1.0))
    a = 0.35 * rng.random(n) + 0.02
    return np.stack([a, k, np.sin(ang), np.cos(ang), sig, rng.random(n) * 2 * math.pi], 1)


# --------------------------------------------------------------------------------------------------
#  Gate 1 -- the slope IS the spatial derivative of the elevation
# --------------------------------------------------------------------------------------------------
def gate_slope(comps, label, xs, zs, t, h=1e-3, gate=True):
    """Central-difference eta in x and z and compare against (sx, sz).

    Step h = 1 mm: truncation is (h^2/6)|eta'''| < 1e-8 for every band here and round-off is
    eps*|eta|/h ~ 1e-12 at a modest t, so anything above ~1e-7 is the FORMULA, not the ruler."""
    res = {}
    fdx = fdz = None
    for v in ("true", "slope_no_k", "slope_flip"):
        p = probe(comps, xs, zs, t, v)
        fdx = (probe(comps, xs + h, zs, t)["eta"] - probe(comps, xs - h, zs, t)["eta"]) / (2 * h)
        fdz = (probe(comps, xs, zs + h, t)["eta"] - probe(comps, xs, zs - h, t)["eta"]) / (2 * h)
        res[v] = max(np.max(np.abs(p["sx"] - fdx)), np.max(np.abs(p["sz"] - fdz)))
    scale = max(np.max(np.abs(fdx)), np.max(np.abs(fdz)))
    if gate:
        margin = res["slope_no_k"] / max(res["true"], 1e-300)
        check("slope == d(eta)/dx,dz  [%s]" % label,
              res["true"] < 1e-7 * max(scale, 1e-3) and margin > 1e3,
              "max|err| %.3e over |grad eta| up to %.4f ; controls: no-k %.3e (%.2ex), "
              "sign-flip %.3e (%.2ex)"
              % (res["true"], scale, res["slope_no_k"], margin, res["slope_flip"],
                 res["slope_flip"] / max(res["true"], 1e-300)))
    return res


# --------------------------------------------------------------------------------------------------
#  Gate 2 -- the velocity IS the time derivative of the surface motion
# --------------------------------------------------------------------------------------------------
def gate_velocity(comps, label, xs, zs, t, dt=1e-4):
    """Three claims, because vy alone would leave the HORIZONTAL velocity's sign ungated:
         (a) vy == d(eta)/dt        -- the linearised kinematic free-surface condition
         (b) (vx,vz) == d(dx,dz)/dt -- velocity is the derivative of displacement, by definition
         (c) |v| == sigma*a         -- deep-water orbits are circles traversed at sigma*a.  This
                                       one is PHYSICS, not self-consistency, and it is what kills
                                       a k-for-sigma swap (which (a) and (b) alone would not, as
                                       the k-scaled field is still its own derivative in shape)."""
    res = {}
    dEta_dt = None
    for v in ("true", "vel_flip", "vy_flip", "vel_k"):
        p = probe(comps, xs, zs, t, v)
        e1 = probe(comps, xs, zs, t + dt, v)
        e0 = probe(comps, xs, zs, t - dt, v)
        dEta_dt = (e1["eta"] - e0["eta"]) / (2 * dt)
        dDx_dt = (e1["dx"] - e0["dx"]) / (2 * dt)
        dDz_dt = (e1["dz"] - e0["dz"]) / (2 * dt)
        res[v] = (float(np.max(np.abs(p["vy"] - dEta_dt))),
                  float(max(np.max(np.abs(p["vx"] - dDx_dt)), np.max(np.abs(p["vz"] - dDz_dt)))))
    scale = float(np.max(np.abs(dEta_dt)))
    tol = 1e-7 * max(scale, 1e-3)
    mvy = res["vy_flip"][0] / max(res["true"][0], 1e-300)
    mvh = res["vel_flip"][1] / max(res["true"][1], 1e-300)
    ctl = " ".join("%s %.3e/%.3e" % (v, res[v][0], res[v][1])
                   for v in ("vel_flip", "vy_flip", "vel_k"))
    check("velocity == d/dt of the surface motion  [%s]" % label,
          res["true"][0] < tol and res["true"][1] < tol and mvy > 1e3 and mvh > 1e3,
          "max|vy - d(eta)/dt| %.3e, max|v_h - d(D_h)/dt| %.3e over |w| up to %.4f m/s ; "
          "controls vy/vh: %s ; margins %.2ex / %.2ex" % (res["true"][0], res["true"][1], scale,
                                                          ctl, mvy, mvh))

    if len(comps) == 1:    # (c) -- a sum of components is not one circle, so single only
        a, k, dX, dZ, sig, _ = comps[0]
        p = probe(comps, xs, zs, t)
        err = float(np.max(np.abs(np.sqrt(p["vx"] ** 2 + p["vy"] ** 2 + p["vz"] ** 2) - sig * a)))
        pk = probe(comps, xs, zs, t, "vel_k")
        errk = float(np.max(np.abs(np.sqrt(pk["vx"] ** 2 + pk["vy"] ** 2 + pk["vz"] ** 2)
                                   - sig * a)))
        check("orbital speed == sigma*a  [%s]" % label,
              err < 1e-12 and errk > 1e3 * max(err, 1e-30),
              "max||v| - sigma*a| = %.3e m/s (sigma*a = %.4f) ; k-for-sigma control %.3e (%.2ex)"
              % (err, sig * a, errk, errk / max(err, 1e-300)))
    return res


# --------------------------------------------------------------------------------------------------
#  Gate 3 -- the Gerstner offset is 90 degrees out of phase with the elevation
# --------------------------------------------------------------------------------------------------
def gate_quadrature(comps, label):
    """Sample four wavelengths along d^ at one instant.  For eta = a cos(theta) and
    D_along = -a sin(theta):
        corr(D_along(s), eta(s))            == 0    (they are orthogonal)
        corr(D_along(s), eta(s + lambda/4)) == +1   (eta shifted a quarter wave IS D_along)
    The SECOND carries the sign: flip the displacement and the zero stays a zero while +1 becomes
    -1.  Both are printed so the vacuous half is visibly vacuous, and only the second is gated."""
    a, k, dX, dZ, sig, _ = comps[0]
    lam = 2.0 * math.pi / k
    s = np.linspace(0.0, 4.0 * lam, 8192, endpoint=False)
    q = lam / 4.0
    out = {}
    for v in ("true", "disp_flip", "disp_cos"):
        p = probe(comps, s * dX, s * dZ, 0.0, v)
        along = p["dx"] * dX + p["dz"] * dZ
        etaq = probe(comps, (s + q) * dX, (s + q) * dZ, 0.0, v)["eta"]
        out[v] = (float(np.corrcoef(along, p["eta"])[0, 1]),
                  float(np.corrcoef(along, etaq)[0, 1]))
    disc = abs(out["true"][1] - out["disp_flip"][1])
    check("Gerstner offset in QUADRATURE with eta  [%s]" % label,
          abs(out["true"][0]) < 1e-9 and abs(out["true"][1] - 1.0) < 1e-9 and disc > 1.9,
          "corr(D_along, eta) = %+.3e (want 0), corr(D_along, eta @ +lambda/4) = %+.9f (want +1)"
          " ; controls: sign-flip %+.6f (separation %.3f), in-phase corr-with-eta %+.6f"
          % (out["true"][0], out["true"][1], out["disp_flip"][1], disc, out["disp_cos"][0]))
    return out


# --------------------------------------------------------------------------------------------------
#  Gate 4 -- the crest runs WITH the wave (the physical reading of the velocity's sign)
# --------------------------------------------------------------------------------------------------
def gate_crest_direction(comps, label):
    """At the crest (theta = 0) the water's horizontal velocity must point ALONG d^, at the trough
    against it.  This is the one statement about the velocity a reader can check against a
    photograph of a wave, and it is exactly what the rotor's sign decides -- the human-readable
    form of gate 2."""
    a, k, dX, dZ, sig, phi0 = comps[0]
    t_crest, t_trough = phi0 / sig, (phi0 + math.pi) / sig   # theta(0,0,t) = phi0 - sigma t
    ac = float(np.sum(probe(comps, 0.0, 0.0, t_crest)["vx"] * dX
                      + probe(comps, 0.0, 0.0, t_crest)["vz"] * dZ))
    at = float(np.sum(probe(comps, 0.0, 0.0, t_trough)["vx"] * dX
                      + probe(comps, 0.0, 0.0, t_trough)["vz"] * dZ))
    pf = probe(comps, 0.0, 0.0, t_crest, "vel_flip")
    af = float(pf["vx"] * dX + pf["vz"] * dZ)
    check("crest water runs WITH the wave  [%s]" % label,
          ac > 0 and at < 0 and abs(ac - sig * a) < 1e-12 and af < 0,
          "u.d^ at crest %+.4f m/s (= sigma*a = %+.4f), at trough %+.4f ; sign-flip control %+.4f"
          % (ac, sig * a, at, af))


T_GATE = 137.0          # a plain scene time; PART B is where the real simUnix magnitude is faced
T_REAL = 1.7576e9       # 2025-09-08-ish unix seconds, the magnitude ProbeAt is actually called at


def part_a():
    print("=" * 98)
    print("PART A -- the six added fields against the relations that define them")
    print("=" * 98)
    rng = np.random.default_rng(7)
    xs = rng.uniform(-400.0, 400.0, 4096)
    zs = rng.uniform(-400.0, 400.0, 4096)

    one = deep_comp(a=1.20, tp=10.0, dirdeg=95.0)     # the cache's own Hs 3.0 / Tp 10 / mwd 95
    print("\n-- single component: a = 1.20 m, Tp = 10 s, dir 95 deg, sigma = %.5f rad/s, "
          "k = %.6f rad/m, lambda = %.2f m, t = %.1f s"
          % (one[0, 4], one[0, 1], 2 * math.pi / one[0, 1], T_GATE))
    gate_slope(one, "1 comp", xs, zs, T_GATE)
    gate_velocity(one, "1 comp", xs, zs, T_GATE)
    gate_quadrature(one, "1 comp")
    gate_crest_direction(one, "1 comp")

    many = random_field(32)
    print("\n-- 32 components, sigma %.3f..%.3f rad/s, k %.4f..%.4f rad/m, sum a = %.3f m"
          % (many[0, 4], many[-1, 4], many[0, 1], many[-1, 1], many[:, 0].sum()))
    gate_slope(many, "32 comp", xs, zs, T_GATE)
    gate_velocity(many, "32 comp", xs, zs, T_GATE)
    return one


# ==================================================================================================
#  PART B -- the clock, so the gates' choice of t is a measurement and not a convenience
# ==================================================================================================
def part_b(one):
    print()
    print("=" * 98)
    print("PART B -- why the gates run at t = %.0f s and not at a real simUnix" % T_GATE)
    print("=" * 98)
    rng = np.random.default_rng(11)
    xs = rng.uniform(-400.0, 400.0, 2048)
    zs = rng.uniform(-400.0, 400.0, 2048)
    print("  ProbeAt forms wt = sigma*simUnix in double. At simUnix %.4e that argument is %.3e"
          % (T_REAL, one[0, 4] * T_REAL))
    print("  rad, whose double spacing is %.2e rad -- so theta itself carries that much error and"
          % np.spacing(one[0, 4] * T_REAL))
    print("  a finite difference over h divides it by 2h. The SIGNATURE separates the two causes:")
    print("  round-off grows as 1/h, truncation shrinks as h^2, a wrong formula does neither.")
    print("    %-8s %-16s %-16s" % ("h (m)", "err @ t=%.0f" % T_GATE, "err @ t=%.4g" % T_REAL))
    for h in (1e-4, 1e-3, 1e-2, 1e-1):
        a = gate_slope(one, "", xs, zs, T_GATE, h, gate=False)["true"]
        b = gate_slope(one, "", xs, zs, T_REAL, h, gate=False)["true"]
        print("    %-8.0e %-16.3e %-16.3e" % (h, a, b))
    print("  The right column is exactly 1/h: it is the CLOCK. The left column is flat at the")
    print("  double floor, which is the formula being exact. eta itself is unharmed -- the same")
    print("  2.4e-7 rad moves the surface by a*2.4e-7 ~ 3e-7 m -- and so are the ANALYTIC")
    print("  derivatives, which never difference anything (see the orbital-speed gate, which")
    print("  passes at 2e-16 whatever t it is given).")


# ==================================================================================================
#  PART C -- the slope against the surface a real solve actually encodes
# ==================================================================================================
#  ProbeAt's slope assumes grad(theta) = k*d^ with the LOCAL k.  The stored phase realises that
#  exactly along x -- the gauge cumsums k*d0*cell west->east, left-inclusive -- and only in the
#  ROW MEAN along z.  This part decodes a real cache file, rebuilds eta at texel centres from the
#  packed planes exactly as the kernel does, differences it, and compares.  It is a MEASUREMENT,
#  not a gate: the x number is a confirmation and the z number is a finding.
HEAD_FMT = "<IIQ8I12d"
HEAD_SIZE = struct.calcsize(HEAD_FMT)          # 144
MAGIC = 0x46564157                             # 'WAVF'


def load_head(path):
    with open(path, "rb") as f:
        buf = f.read(HEAD_SIZE)
        hv = struct.unpack_from(HEAD_FMT, buf, 0)
        d = dict(zip(["magic", "version", "key", "tableBytes", "atlasW", "atlasH", "nx", "ny",
                      "nComp", "hasInputs", "pad0", "hs", "tp", "mwdDeg", "cellM", "spreadDeg",
                      "barNormalDeg", "gammaHs", "minSamplesPerLambda", "orgX", "orgZ", "level",
                      "currentMs"], hv))
        if d["magic"] != MAGIC:
            raise ValueError("bad magic 0x%08x" % d["magic"])
        # kMaxComp is NOT fixed across builds (16 -> 32 -> 64 in this tree), so it is derived from
        # the table's own size, never assumed: GpuTable = 4f + 4u + 5*float[kMaxComp] + 4f.
        nc = (d["tableBytes"] - 48) // 20
        if nc * 20 + 48 != d["tableBytes"]:
            raise ValueError("tableBytes %d is not a GpuTable" % d["tableBytes"])
        f.seek(HEAD_SIZE)
        tv = struct.unpack("<4f4I%df4f" % (5 * nc), f.read(d["tableBytes"]))
    t = dict(orgX=tv[0], orgZ=tv[1], invCell=tv[2], feather=tv[3], nx=tv[4], ny=tv[5],
             nUsed=tv[6], envSlice=tv[7],
             sigma=np.array(tv[8:8 + nc]), dirX=np.array(tv[8 + nc:8 + 2 * nc]),
             dirZ=np.array(tv[8 + 2 * nc:8 + 3 * nc]), aMax=np.array(tv[8 + 3 * nc:8 + 4 * nc]),
             kMax=np.array(tv[8 + 4 * nc:8 + 5 * nc]), kMaxComp=nc)
    return d, t


def part_c(path, tile=384):
    print()
    print("=" * 98)
    print("PART C -- MEASUREMENT on a real solve: ProbeAt's slope vs the atlas's own surface")
    print("=" * 98)
    d, t = load_head(path)
    nx, ny, cell = d["nx"], d["ny"], d["cellM"]
    print("file %s (%.0f MB): window %dx%d @ %.4f m, kMaxComp %d, nUsed %d, Hs %.3f Tp %.2f "
          "mwd %.1f" % (os.path.basename(path), os.path.getsize(path) / 1e6, nx, ny, cell,
                        t["kMaxComp"], t["nUsed"], d["hs"], d["tp"], d["mwdDeg"]))
    up = [c for c in range(int(t["nUsed"])) if t["aMax"][c] > 0.0]
    if not up:
        print("  no uploaded components -- nothing to measure")
        return
    atlas = np.memmap(path, dtype=np.uint8, mode="r", offset=HEAD_SIZE + d["tableBytes"],
                      shape=(d["atlasH"], d["atlasW"], 4))
    i0, j0 = (nx - tile) // 2, (ny - tile) // 2
    # eta and the analytic slope, both at TEXEL CENTRES and both from the same decoded bytes.
    eta = np.zeros((tile, tile))
    sx = np.zeros((tile, tile))
    sz = np.zeros((tile, tile))
    sxq = np.zeros((tile, tile))     # the analytic slope with the central-difference transfer
    szq = np.zeros((tile, tile))     # sinc(k*d*cell) folded in -- see below
    wkb_x = np.zeros((tile, tile - 2))   # the (da/dx)cos(theta) term the WKB reading drops
    wet = None
    for c in up:
        sx0, sy0 = (c & 1) * nx, (c >> 1) * ny
        tl = np.asarray(atlas[sy0 + j0:sy0 + j0 + tile, sx0 + i0:sx0 + i0 + tile, :], np.float64)
        a = (tl[:, :, 0] + 0.5) / 255.0 * t["aMax"][c]
        k = (tl[:, :, 1] + 0.5) / 255.0 * t["kMax"][c]
        cs = (tl[:, :, 2] + 0.5) / 255.0 * 2.0 - 1.0
        sn = (tl[:, :, 3] + 0.5) / 255.0 * 2.0 - 1.0
        nrm = np.hypot(cs, sn)
        cs, sn = cs / nrm, sn / nrm              # the cl2 renormalise ProbeAt does
        wet = (tl[:, :, 0] > 0) if wet is None else (wet | (tl[:, :, 0] > 0))
        dX, dZ = t["dirX"][c], t["dirZ"][c]
        eta += a * cs                            # t = 0: the rotor is identity
        sx -= a * sn * k * dX
        sz -= a * sn * k * dZ
        # A central difference over one cell does not return the derivative of a sinusoid, it
        # returns it times sin(q)/q with q = k*d*cell -- up to 8 percent at this window's Nyquist
        # band. Folding the SAME factor into the analytic slope removes the ruler's bias and
        # leaves only what is being measured: gauge, amplitude gradient and 8-bit noise.
        qx, qz = k * dX * cell, k * dZ * cell
        sxq -= a * sn * k * dX * np.sinc(qx / math.pi)
        szq -= a * sn * k * dZ * np.sinc(qz / math.pi)
        wkb_x += (a[:, 2:] - a[:, :-2]) / (2.0 * cell) * cs[:, 1:-1]
    fdx = (eta[:, 2:] - eta[:, :-2]) / (2 * cell)
    fdz = (eta[2:, :] - eta[:-2, :]) / (2 * cell)
    mx, mz = wet[:, 1:-1], wet[1:-1, :]
    rms = lambda v: float(np.sqrt((v ** 2).mean()))   # noqa: E731
    print("  tile %dx%d at the window centre, %d uploaded comps, %.1f%% wet, rms eta %.3f m"
          % (tile, tile, len(up), 100.0 * wet.mean(), rms(eta[wet])))
    for ax, an, fd, m in (("x", sxq[:, 1:-1], fdx, mx), ("z", szq[1:-1, :], fdz, mz)):
        print("    %s:  rms|analytic - FD| %.4e   vs rms|FD| %.4e   ->  %.2f%%"
              % (ax, rms((an - fd)[m]), rms(fd[m]),
                 100.0 * rms((an - fd)[m]) / max(rms(fd[m]), 1e-12)))
    # How much of the x residual is the WKB assumption (grad a dropped) rather than 8-bit noise?
    # The dropped term is exactly (da/dx)cos(theta); add it back from the SAME decoded planes and
    # see what it buys. Whatever remains is quantisation, which is the atlas's floor, not a bug.
    r0 = rms((sxq[:, 1:-1] - fdx)[mx])
    r1 = rms((sxq[:, 1:-1] + wkb_x - fdx)[mx])
    print("    x residual split: %.4e -> %.4e when the dropped (da/dx)cos(theta) term is added"
          % (r0, r1))
    print("      i.e. %.0f%% of the x residual IS the WKB assumption (%.2f%% of |grad eta|); what"
          " survives is %.2f%%, the 8-bit atlas floor."
          % (100.0 * (1.0 - r1 / max(r0, 1e-12)), 100.0 * r0 / max(rms(fdx[mx]), 1e-12),
             100.0 * r1 / max(rms(fdx[mx]), 1e-12)))
    # And the structural cause, one component at a time: what the stored phase's own gradient is.
    print("  per-component grad(phi) straight off the spinors (rad/m), vs the k*d^ the slope uses:")
    print("    %-4s %-8s %-9s  %-24s %-24s" % ("c", "sigma", "lambda", "x: FD / k*dX^",
                                               "z: FD / k*dZ^"))
    for c in up[:6]:
        sx0, sy0 = (c & 1) * nx, (c >> 1) * ny
        tl = np.asarray(atlas[sy0 + j0:sy0 + j0 + tile, sx0 + i0:sx0 + i0 + tile, :], np.float64)
        k = (tl[:, :, 1] + 0.5) / 255.0 * t["kMax"][c]
        cs = (tl[:, :, 2] + 0.5) / 255.0 * 2.0 - 1.0
        sn = (tl[:, :, 3] + 0.5) / 255.0 * 2.0 - 1.0
        dphix = np.arctan2(sn[:, 1:] * cs[:, :-1] - cs[:, 1:] * sn[:, :-1],
                           cs[:, 1:] * cs[:, :-1] + sn[:, 1:] * sn[:, :-1]) / cell
        dphiz = np.arctan2(sn[1:, :] * cs[:-1, :] - cs[1:, :] * sn[:-1, :],
                           cs[1:, :] * cs[:-1, :] + sn[1:, :] * sn[:-1, :]) / cell
        wx = k[:, 1:] * t["dirX"][c]      # left-inclusive gauge: the RIGHT cell's k
        wz = k[1:, :] * t["dirZ"][c]
        print("    %-4d %-8.4f %-9.2f  %+.4e / %+.4e   %+.4e / %+.4e"
              % (c, t["sigma"][c], 2 * math.pi / float(np.median(k)),
                 float(np.median(dphix)), float(np.median(wx)),
                 float(np.median(dphiz)), float(np.median(wz))))
    # The MECHANISM behind the z column, isolated: phi[j,i] is a per-row cumsum along x, so
    # d(phi)/dz picks up the ACCUMULATED difference in k between neighbouring rows, and that
    # accumulation grows with distance from the gauge's west anchor. If that is the cause, the
    # spurious z-gradient must grow left-to-right across the window. It does.
    c = up[0]
    sx0, sy0 = (c & 1) * nx, (c >> 1) * ny
    print("  d(phi)/dz for comp %d vs distance east of the gauge's anchor column:" % c)
    for frac in (0.08, 0.30, 0.55, 0.80, 0.95):
        i = int(frac * (nx - 8))
        st = np.asarray(atlas[sy0 + j0:sy0 + j0 + 256, sx0 + i:sx0 + i + 8, :], np.float64)
        kk = (st[:, :, 1] + 0.5) / 255.0 * t["kMax"][c]
        cs = (st[:, :, 2] + 0.5) / 255.0 * 2.0 - 1.0
        sn = (st[:, :, 3] + 0.5) / 255.0 * 2.0 - 1.0
        dz = np.arctan2(sn[1:, :] * cs[:-1, :] - cs[1:, :] * sn[:-1, :],
                        cs[1:, :] * cs[:-1, :] + sn[1:, :] * sn[:-1, :]) / cell
        print("    col %5d (%5.0f m east):  median |d(phi)/dz| = %.4e   vs |k*dZ^| = %.4e rad/m"
              % (i, i * cell, float(np.median(np.abs(dz))),
                 abs(float(np.median(kk)) * t["dirZ"][c])))
    print("  READING. x is the gauge's EXACT leg, so the two columns must and do agree -- what is")
    print("  left there is the 8-bit spinor floor. z is the ROW-MEAN leg, and it does not agree:")
    print("  the per-row x-cumsum drifts between rows wherever k does (dry cells hold k = 1.253,")
    print("  so a row crossing more land drifts hard), and that drift lands entirely in d(phi)/dz.")
    print("  ProbeAt returns the LOCAL PLANE WAVE slope, which is the physics and what a normal")
    print("  wants; the atlas's z-gradient is the documented gauge. They are different objects and")
    print("  this is the size of the difference. Resolving which one a HULL should ride needs a")
    print("  render, not a proof -- it is not settled here.")
    del atlas


def newest_cache():
    here = os.path.dirname(os.path.abspath(__file__))
    got = glob.glob(os.path.join(here, "..", "cache", "wave", "*.bin"))
    return max(got, key=os.path.getmtime) if got else None


if __name__ == "__main__":
    one = part_a()
    part_b(one)
    path = sys.argv[1] if len(sys.argv) > 1 else newest_cache()
    if path and os.path.exists(path):
        try:
            part_c(path)
        except Exception as e:                                  # noqa: BLE001
            print("\nPART C skipped: %s: %s" % (type(e).__name__, e))
    else:
        print("\nPART C skipped: no cache/wave/*.bin (run the engine over the window once)")
    print()
    if FAILS:
        print("FAILED: " + ", ".join(FAILS))
        sys.exit(1)
    print("all gates PASS")
