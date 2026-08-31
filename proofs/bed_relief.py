# ==================================================================================================
#  bed_relief.py - proof for the data-driven bed albedo feature (bedform relief extractor et al).
#
#  ALGEBRA FIRST: this file is the authoritative small-scale rendering of the math in
#  scratchpad/math/bed.derivation.md (post-verdict corrected form); the engine port must
#  reproduce these numbers.  Deterministic (seed 0), stdlib + numpy + PIL only, prints PASS/FAIL
#  per assertion, exits nonzero on failure, renders proofs/bed_relief.png.
#
#  Covered (the corrected gatest set):
#    1. 6 m cross high-pass transfer function: axial zeros lambda = 6/n m, unit axial peaks at
#       12 and 4 m, diagonal peak 2 at 6*sqrt(2) m; the FILLED-BOX contrast (peaks ~1.2266,
#       the 9-point Dirichlet sidelobe -- never 2; verdict correction of "never exceeds 1");
#       operator == symbol (analytic 5-tap and circulant grid).
#    2. DC zero on constants; H(0) == 0.
#    3. Fold law: lambda=12 sine (clamp inactive) clamped mean == 0; the CORRECTED skew vector
#       z = 0.8 sin(2 pi x/8) + 0.4 sin(2 pi x/4 + 1)  (fundamental 8 m + its SECOND harmonic
#       4 m: any 12 m-periodic bed survives the operator with odd harmonics only, since even
#       harmonics of 12 m sit on the zero set lambda = 6/n, and the clamp of a half-period-
#       antisymmetric field has mean EXACTLY zero -- the twice-corrected verdict catch).
#       Clamped mean ~ +0.0225 > 0.005 while unclamped mean == 0 and coarse-input == 0.
#    4. Sediment renormalization invariants (contrast/modulation ranges, base endpoints).
#    5. Hemisphere-ambient partition of unity + flat-calibration redistribution constraint
#       (and the 1/0.55 = 1.818x wash-out counterexample).
#    6. Narrowness classifier exact discrete values: straight shoreline lf = (r+1)/(2r+1)
#       = 31/61 (center-on-land convention; 0.5 is unattainable on the odd box -- verdict).
#    7. Waterline metric self-consistency with the corrected flip wording: flip an exact
#       INTEGER count of ALL cell labels; A(L0) == 1 - flips/N exactly.
#    8. REAL vqview bathy (vqview_ref.load_bathy): periodic np.roll circulant extractor gives
#       |DC(relief_raw)|/|DC(z)| < 1e-12 (exact circulant identity; the edge-padded form is
#       non-circulant, DC ~ 7.6e-6 relative, kept only for the visual -- verdict); coarse-input
#       leakage with the PRECISE evaluation (60 m block means -> bilinear back to fine texel
#       centers -> SAME fine-grid integer-tap periodic operator) gated at
#       RMS(coarse)/RMS(fine) < 0.15 (measured 0.105, recorded then frozen).
# ==================================================================================================
import os
import sys

import numpy as np
from PIL import Image

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from vqview_ref import load_bathy, CELL

HERE = os.path.dirname(os.path.abspath(__file__))
PNG = os.path.join(HERE, "bed_relief.png")

E = 6.0            # cross tap radius, m               (Water.hlsl:132)
GAIN = 0.85        # relief gain                       (Water.hlsl:136)
CLAMP = 0.45       # relief clamp, m                   (Water.hlsl:136)
SAND = np.array([0.360, 0.318, 0.258])   # (Water.hlsl:103)
SILT = np.array([0.105, 0.100, 0.092])   # (Water.hlsl:104)
SKY_IRR = np.array([0.420, 0.520, 0.680])  # (Water.hlsl:102)
K_AMB = 0.55       # flat-ground ambient calibration   (Water.hlsl:809)

FAILS = []


def check(name, ok, detail=""):
    print("%s  %s%s" % ("PASS" if ok else "FAIL", name, ("  [%s]" % detail) if detail else ""))
    if not ok:
        FAILS.append(name)


def smoothstep(a, b, x):
    t = np.clip((x - a) / (b - a), 0.0, 1.0)
    return t * t * (3.0 - 2.0 * t)


def H_sym(k1, k2):
    """Fourier symbol of the 6 m cross high-pass (metres, rad/m)."""
    return 1.0 - 0.5 * (np.cos(E * k1) + np.cos(E * k2))


def relief_raw_periodic(z, taps=4):
    """Circulant (np.roll) form of c - m; exact DC-zero identity holds here."""
    m = 0.25 * (np.roll(z, taps, 1) + np.roll(z, -taps, 1)
                + np.roll(z, taps, 0) + np.roll(z, -taps, 0))
    return z - m


def relief_raw_padded(z, taps=4):
    """Edge-padded (non-circulant) form -- for visuals only (DC ~1e-5 relative on real data)."""
    zp = np.pad(z, taps, mode="edge")
    c = zp[taps:-taps, taps:-taps]
    m = 0.25 * (zp[taps:-taps, 2 * taps:] + zp[taps:-taps, :-2 * taps]
                + zp[2 * taps:, taps:-taps] + zp[:-2 * taps, taps:-taps])
    return c - m


def clampr(rr):
    return np.clip(GAIN * rr, -CLAMP, CLAMP)


# ------------------------------------------------------------------------------------------------
# 1. transfer function
# ------------------------------------------------------------------------------------------------
def part_transfer(rng):
    print("-- 1. transfer function of the 6 m cross high-pass --")
    # axial zeros lambda = 6/n  (k = 2 pi n / 6)
    for n in (1, 2, 3):
        lam = 6.0 / n
        h = H_sym(2.0 * np.pi / lam, 0.0)
        check("axial zero at lambda = %g m" % lam, abs(h) < 1e-9, "H = %.3e" % h)
    # axial unit peaks lambda = 12, 4
    for lam in (12.0, 4.0):
        h = H_sym(2.0 * np.pi / lam, 0.0)
        check("axial unit peak at lambda = %g m" % lam, abs(h - 1.0) < 1e-9, "H = %.12f" % h)
    # diagonal peak: k1 = k2 = pi/6 -> lambda = 2 pi / |k| = 6 sqrt(2)
    h = H_sym(np.pi / 6.0, np.pi / 6.0)
    check("diagonal peak H(pi/6, pi/6) == 2 (lambda = %.2f m)" % (6.0 * np.sqrt(2.0)),
          abs(h - 2.0) < 1e-9, "H = %.12f" % h)

    # filled-box contrast (verdict minor: box high-pass peaks ~1.2266, NOT <= 1, and never 2)
    N = 9  # 13.5 m box at 1.5 m texels
    k = np.linspace(1e-9, np.pi / CELL, 400001)
    D = np.sin(N * k * CELL / 2.0) / (N * np.sin(k * CELL / 2.0))
    box_hp_peak = 1.0 - D.min()   # 2-D product box: other axis at D=1, so the 1-D min governs
    check("box high-pass peak ~1.2266 (9-tap Dirichlet sidelobe), in (1.15, 1.30) and < 2",
          1.15 < box_hp_peak < 1.30, "peak = %.6f" % box_hp_peak)

    # operator == symbol, analytic 5-tap at x = 0 on z = cos(k.x), 32 random continuous k
    ok = True
    worst = 0.0
    for _ in range(32):
        k1, k2 = rng.uniform(-np.pi / CELL, np.pi / CELL, 2)
        # c = cos(0) = 1;  m = 0.25*(cos(6 k1) + cos(-6 k1) + cos(6 k2) + cos(-6 k2))
        op = 1.0 - 0.25 * (np.cos(E * k1) + np.cos(-E * k1) + np.cos(E * k2) + np.cos(-E * k2))
        worst = max(worst, abs(op - H_sym(k1, k2)))
        ok &= abs(op - H_sym(k1, k2)) < 1e-9
    check("analytic 5-tap == symbol on 32 random k", ok, "max err %.2e" % worst)

    # operator == symbol on a periodic grid (circulant): 32 random integer mode pairs
    Ng = 128
    x = np.arange(Ng) * CELL
    X, Y = np.meshgrid(x, x)
    ok = True
    worst = 0.0
    for _ in range(32):
        mi, ni = rng.integers(0, Ng, 2)
        k1 = 2.0 * np.pi * mi / (Ng * CELL)
        k2 = 2.0 * np.pi * ni / (Ng * CELL)
        z = np.cos(k1 * X + k2 * Y)
        err = np.abs(relief_raw_periodic(z) - H_sym(k1, k2) * z).max()
        worst = max(worst, err)
        ok &= err < 1e-6
    check("circulant grid operator == symbol on 32 random modes", ok, "max err %.2e" % worst)
    return box_hp_peak


# ------------------------------------------------------------------------------------------------
# 2. DC zero on constants
# ------------------------------------------------------------------------------------------------
def part_dc(rng):
    print("-- 2. DC zero --")
    ok = True
    for c in rng.uniform(-40.0, 40.0, 8):
        z = np.full((16, 16), c)
        r = clampr(relief_raw_periodic(z))
        ok &= np.all(r == 0.0)
    check("relief of constant fields == 0 exactly", ok)
    check("H(0) == 0 exactly", H_sym(0.0, 0.0) == 0.0)


# ------------------------------------------------------------------------------------------------
# 3. fold law / skew (the twice-corrected test vector)
# ------------------------------------------------------------------------------------------------
def relief_1d(zf, x):
    """1-D-in-x field (constant in y): the y taps equal c, so c - m = 0.5 c - 0.25 (z+ + z-)."""
    return zf(x) - 0.25 * (zf(x + E) + zf(x - E) + 2.0 * zf(x))


def part_fold():
    print("-- 3. fold law: clamp(mean) != mean(clamp) --")
    # (a) lambda = 12 pure sine, a = 0.40: gain 1, 0.85*0.40 = 0.34 < 0.45 -> clamp inactive
    a = 0.40
    zf = lambda x: a * np.sin(2.0 * np.pi * x / 12.0)
    x = np.arange(0.0, 12.0, CELL)          # exactly one period, 8 samples
    rr = relief_1d(zf, x)
    check("lambda=12 sine: clamp inactive", GAIN * np.abs(rr).max() < CLAMP,
          "max 0.85|rr| = %.4f" % (GAIN * np.abs(rr).max()))
    check("lambda=12 sine: clamped mean == 0", abs(clampr(rr).mean()) < 1e-6,
          "mean = %.2e" % clampr(rr).mean())
    zbar = zf(x).mean()                     # coarse INPUT: period box-mean is a constant
    check("lambda=12 sine: coarse-input relief == 0",
          abs(clampr(np.array([zbar - zbar]))[0]) < 1e-12)

    # (b) CORRECTED skew vector: fundamental 8 m (gain 1/2) + SECOND harmonic 4 m (gain 1).
    zf = lambda x: 0.8 * np.sin(2.0 * np.pi * x / 8.0) + 0.4 * np.sin(2.0 * np.pi * x / 4.0 + 1.0)
    x = np.arange(0.0, 24.0, CELL)          # lcm(8 m, 1.5 m) = 24 m -> 16 samples, exact period
    rr = relief_1d(zf, x)
    r = clampr(rr)
    check("skew vector: clamp active", GAIN * np.abs(rr).max() > CLAMP,
          "max 0.85|rr| = %.4f" % (GAIN * np.abs(rr).max()))
    check("skew vector: UNclamped mean == 0 (H kills DC)", abs(rr.mean()) < 1e-12,
          "mean = %.2e" % rr.mean())
    check("skew vector: |clamped mean| > 0.005 (the fold inequality)", abs(r.mean()) > 0.005,
          "mean(clamp r) = %+.6f m" % r.mean())
    zbar = zf(x).mean()
    check("skew vector: coarse-input relief == 0",
          abs(clampr(np.array([zbar - zbar]))[0]) < 1e-12)
    print("      clamp(mean) = 0 but mean(clamp) = %+.4f m -- answers must fold." % r.mean())


# ------------------------------------------------------------------------------------------------
# 4. sediment renormalization invariants
# ------------------------------------------------------------------------------------------------
def base_albedo(d):
    s = smoothstep(2.5, 11.0, d)
    return SAND[None, :] * (1.0 - s[:, None]) + SILT[None, :] * s[:, None]


def part_renorm():
    print("-- 4. sediment renormalization invariants --")
    d = np.linspace(0.0, 100.0, 5001)
    contrast = 1.0 - 0.55 * smoothstep(3.0, 14.0, d)
    check("contrast(d) in [0.45, 1] on d in [0,100]",
          contrast.min() >= 0.45 - 1e-7 and contrast.max() <= 1.0 + 1e-7,
          "range [%.4f, %.4f]" % (contrast.min(), contrast.max()))
    r = np.linspace(-CLAMP, CLAMP, 1001)
    mod = 1.0 + r[:, None] * contrast[None, :]
    check("modulation (1 + r*contrast) in [0.55, 1.45]",
          mod.min() >= 0.55 - 1e-7 and mod.max() <= 1.45 + 1e-7,
          "range [%.4f, %.4f]" % (mod.min(), mod.max()))
    b = base_albedo(d)
    check("base(0) == SAND", np.abs(b[0] - SAND).max() < 1e-7)
    check("base(d >= 11) == SILT", np.abs(b[d >= 11.0] - SILT[None, :]).max() < 1e-7)
    check("base monotone non-increasing per channel", np.all(np.diff(b, axis=0) <= 1e-12))
    check("max channel never clips: 0.360*1.45 = %.4f < 1" % (0.36 * 1.45), 0.36 * 1.45 < 1.0)


# ------------------------------------------------------------------------------------------------
# 5. hemisphere ambient: partition of unity + redistribution constraint
# ------------------------------------------------------------------------------------------------
def part_hemisphere(rng):
    print("-- 5. split-hemisphere ambient --")
    v = rng.normal(size=(256, 3))
    n = v / np.linalg.norm(v, axis=1, keepdims=True)
    ny = n[:, 1]
    w_sky = (1.0 + ny) / 2.0
    w_gnd = (1.0 - ny) / 2.0
    check("w_sky + w_gnd == 1 for 256 random unit normals",
          np.abs(w_sky + w_gnd - 1.0).max() < 1e-7)
    check("both weights in [0,1]",
          w_sky.min() >= -1e-12 and w_sky.max() <= 1.0 + 1e-12
          and w_gnd.min() >= -1e-12 and w_gnd.max() <= 1.0 + 1e-12)
    # flat calibration: n = +y, skyVis = 1; ground hemisphere term vanishes; any flat ambient
    sky_vis = 1.0
    flat_amb = SKY_IRR * 1.2345  # arbitrary stand-in for flatLit's ambient; coefficient (1-skyVis)=0
    e_amb = SKY_IRR * K_AMB * sky_vis + flat_amb * K_AMB * (1.0 - sky_vis)
    check("flat calibration reproduced exactly at n = +y (redistribution constraint)",
          np.abs(e_amb - SKY_IRR * K_AMB).max() == 0.0)
    # counterexample: dropping kAmb from the sky term at n = y washes flat ground by 1/0.55
    washed = SKY_IRR * sky_vis
    ratio = washed / (SKY_IRR * K_AMB)
    check("dropping kAmb redistribution gives the documented 1/0.55 = 1.818x wash-out",
          np.abs(ratio - 1.0 / K_AMB).max() < 1e-7, "ratio = %.6f" % ratio[0])


# ------------------------------------------------------------------------------------------------
# 6. narrowness classifier exact discrete geometry
# ------------------------------------------------------------------------------------------------
def rock_of_lf(lf):
    u = np.clip((lf - 0.28) / (0.62 - 0.28), 0.0, 1.0)
    return 1.0 - u * u * (3.0 - 2.0 * u)


def part_narrowness():
    print("-- 6. narrowness classifier (r = 30 texels, 61x61 box) --")
    r = 30
    n = 2 * r + 1  # 61
    # all land
    check("lf(all land) == 1", np.mean(np.ones((n, n))) == 1.0)
    # infinite straight axis-aligned shoreline, CENTER COLUMN LAND (convention stated):
    # land columns are the center and everything on one side -> r+1 of the 2r+1 columns.
    win = np.zeros((n, n))
    win[:, r:] = 1.0                       # columns r..2r land -> r+1 columns
    lf = win.mean()
    exact = (r + 1) / (2 * r + 1)          # 31/61 = 0.508197 -- 0.5 is unattainable (verdict)
    check("lf(straight shoreline, center on land) == (r+1)/(2r+1) = 31/61 exactly",
          abs(lf - exact) < 1e-9, "lf = %.9f, 31/61 = %.9f" % (lf, exact))
    win2 = np.zeros((n, n))
    win2[:, r + 1:] = 1.0                  # center on the WATER side -> r land columns
    check("lf(straight shoreline, center on water) == r/(2r+1) = 30/61 exactly",
          abs(win2.mean() - r / (2 * r + 1)) < 1e-9)
    # centered odd strips
    ok = True
    for w in (1, 13, 27, 61):
        strip = np.zeros((n, n))
        strip[:, r - (w - 1) // 2: r + (w - 1) // 2 + 1] = 1.0
        ok &= abs(strip.mean() - w / (2 * r + 1)) < 1e-9
    check("lf(centered odd strip w) == w/(2r+1) exactly for w in {1,13,27,61}", ok)
    # rock endpoints + monotonicity
    check("rock(lf <= 0.28) == 1", rock_of_lf(np.array([0.0, 0.28]))[1] == 1.0
          and rock_of_lf(np.array([0.1]))[0] == 1.0)
    check("rock(lf >= 0.62) == 0", rock_of_lf(np.array([0.62]))[0] == 0.0
          and rock_of_lf(np.array([0.9]))[0] == 0.0)
    check("rock(lf = 0.45) == 0.5", abs(rock_of_lf(np.array([0.45]))[0] - 0.5) < 1e-6)
    lfg = np.linspace(0.0, 1.0, 2001)
    check("rock monotone non-increasing in lf", np.all(np.diff(rock_of_lf(lfg)) <= 1e-12))
    # geometry calibration echoes (informational): jetty w ~ 20 m -> lf ~ w/2R
    print("      geometry: w/2R for jetty 20 m -> %.2f (measured 0.22), spit 40 m -> %.2f "
          "(measured 0.45)" % (20.0 / 90.0, 40.0 / 90.0))


# ------------------------------------------------------------------------------------------------
# 7. waterline metric self-consistency (corrected flip wording)
# ------------------------------------------------------------------------------------------------
def agreement(z, wet, valid, L):
    pred_dry = z > L
    return np.mean((pred_dry == ~wet)[valid])


def part_waterline(rng):
    print("-- 7. waterline agreement metric --")
    Ls = np.arange(-3.0, 3.01, 0.10)
    L0 = 0.70                                        # on the sweep grid
    z = rng.uniform(-3.0, 3.0, size=(200, 200))
    valid = np.ones_like(z, dtype=bool)
    wet = z <= L0
    a0 = agreement(z, wet, valid, L0)
    check("A(L0) == 1 with wet == (z <= L0)", a0 == 1.0, "A = %.12f" % a0)
    # corrected wording (verdict): flip an exact INTEGER count of ALL cell labels, p < 0.5,
    # chosen uniformly at random over all cells; A(L0) == 1 - flips/N EXACTLY.
    N = z.size
    p = 0.2
    flips = int(round(p * N))
    idx = rng.permutation(N)[:flips]
    wet_f = wet.copy().ravel()
    wet_f[idx] = ~wet_f[idx]
    wet_f = wet_f.reshape(z.shape)
    a_flip = agreement(z, wet_f, valid, L0)
    check("A(L0) after flipping %d/%d of ALL labels == 1 - %g exactly" % (flips, N, p),
          a_flip == 1.0 - flips / N, "A = %.12f" % a_flip)
    sweep = np.array([agreement(z, wet_f, valid, L) for L in Ls])
    check("L0 remains the argmax after random flips (p < 1/2)",
          np.all(a_flip >= sweep), "max other = %.6f" % sweep[np.abs(Ls - L0) > 1e-9].max())
    # monotone beach profile -> A(L) unimodal about L0
    zb = np.linspace(-3.0, 3.0, 6001)
    wetb = zb <= L0
    vb = np.ones_like(zb, dtype=bool)
    ab = np.array([agreement(zb, wetb, vb, L) for L in Ls])
    i0 = int(np.argmin(np.abs(Ls - L0)))
    left, right = ab[:i0 + 1], ab[i0:]
    check("A(L) non-increasing away from L0 on a strictly monotone beach",
          np.all(np.diff(left) >= -1e-12) and np.all(np.diff(right) <= 1e-12))


# ------------------------------------------------------------------------------------------------
# 8. the REAL bathy: circulant DC identity, saturation, coarse-input leakage gate
# ------------------------------------------------------------------------------------------------
def bilinear_upsample_blocks(zb, B, ny, nx):
    """Block means back to fine texel centers by separable bilinear interpolation
    (block centers at (i + 0.5)*B - 0.5 in fine index coords; edges clamped by np.interp)."""
    ny2, nx2 = zb.shape
    yc = (np.arange(ny2) + 0.5) * B - 0.5
    xc = (np.arange(nx2) + 0.5) * B - 0.5
    yf = np.arange(ny)
    xf = np.arange(nx)
    tmp = np.empty((ny2, nx))
    for i in range(ny2):
        tmp[i] = np.interp(xf, xc, zb[i])
    out = np.empty((ny, nx))
    for j in range(nx):
        out[:, j] = np.interp(yf, yc, tmp[:, j])
    return out


def part_real():
    print("-- 8. real vqview bathy (wide crop, 1.5 m texels, 6 m = exactly 4 taps) --")
    z = load_bathy()                                   # (690, 1720) m MLLW, row 0 north
    ny, nx = z.shape
    print("      bathy %dx%d, z in [%.2f, %.2f] m MLLW" % (ny, nx, z.min(), z.max()))

    rr_per = relief_raw_periodic(z)
    dc_rel = abs(rr_per.mean()) / abs(z.mean())
    check("circulant DC identity: |DC(relief_raw)|/|DC(z)| < 1e-12", dc_rel < 1e-12,
          "%.2e (mean z = %.3f m)" % (dc_rel, z.mean()))

    # the edge-padded (visual) form is non-circulant: report its DC, do not assert tight
    rr_pad = relief_raw_padded(z)
    dc_pad = abs(rr_pad.mean()) / abs(z.mean())
    check("edge-padded DC only loose (< 1e-4 relative, boundary strip)", dc_pad < 1e-4,
          "%.2e -- visuals only" % dc_pad)

    interior = (slice(4, -4), slice(4, -4))
    relief_fine = clampr(rr_per)
    sat = np.mean(np.abs(GAIN * rr_per[interior]) >= CLAMP)
    rms_fine = np.sqrt(np.mean(relief_fine[interior] ** 2))
    print("      saturated-clamp fraction (interior): %.2f%%   RMS(relief_fine) = %.4f m"
          % (100.0 * sat, rms_fine))
    check("saturated fraction sane (1%..15%)", 0.01 < sat < 0.15, "%.4f" % sat)

    # coarse-input leakage, PRECISELY specified (verdict): 60 m = 40-texel block means,
    # bilinearly interpolated back to the FINE grid texel centers, run through the SAME
    # fine-grid integer-tap periodic operator; interior 4-texel border discarded; relative RMS.
    B = 40
    ny2, nx2 = ny // B, nx // B
    zt = z[:ny2 * B, :nx2 * B]
    zb = zt.reshape(ny2, B, nx2, B).mean(axis=(1, 3))
    zc = bilinear_upsample_blocks(zb, B, ny2 * B, nx2 * B)
    ic = (slice(4, ny2 * B - 4), slice(4, nx2 * B - 4))
    relief_coarse = clampr(relief_raw_periodic(zc))
    relief_fine_t = clampr(relief_raw_periodic(zt))
    rms_c = np.sqrt(np.mean(relief_coarse[ic] ** 2))
    rms_f = np.sqrt(np.mean(relief_fine_t[ic] ** 2))
    ratio = rms_c / rms_f
    check("coarse-input leakage RMS ratio < 0.15 (frozen; measured 0.105 at freeze)",
          ratio < 0.15, "RMS coarse %.4f / fine %.4f = %.4f" % (rms_c, rms_f, ratio))
    return z, rr_pad, sat


# ------------------------------------------------------------------------------------------------
# figure: relief map (edge-padded form, visuals) + transfer curves
# ------------------------------------------------------------------------------------------------
def draw_line(img, xs, ys, color):
    """Poly-line into an HxWx3 uint8 array (dense sampling; xs/ys already pixel coords)."""
    h, w, _ = img.shape
    for i in range(len(xs) - 1):
        x0, y0, x1, y1 = xs[i], ys[i], xs[i + 1], ys[i + 1]
        n = int(max(abs(x1 - x0), abs(y1 - y0))) + 1
        for t in np.linspace(0.0, 1.0, n):
            x = int(round(x0 + t * (x1 - x0)))
            y = int(round(y0 + t * (y1 - y0)))
            if 0 <= x < w and 0 <= y < h:
                img[y, x] = color
                if y + 1 < h:
                    img[y + 1, x] = color


def render(z, rr_pad, sat):
    relief = clampr(rr_pad)
    t = (relief / CLAMP)                       # [-1, 1]
    # diverging map: troughs blue, crests orange, zero mid-gray; land tinted green-gray
    r = np.where(t >= 0, 0.42 + 0.55 * t, 0.42 * (1 + t) + 0.10 * (-t))
    g = np.where(t >= 0, 0.42 + 0.33 * t, 0.42 * (1 + t) + 0.35 * (-t))
    b = np.where(t >= 0, 0.42 - 0.30 * t, 0.42 * (1 + t) + 0.85 * (-t))
    rgb = np.stack([r, g, b], axis=-1)
    land = z >= 0.0
    rgb[land] = rgb[land] * 0.35 + np.array([0.16, 0.22, 0.14]) * 0.65
    top = (np.clip(rgb, 0, 1) * 255).astype(np.uint8)

    # transfer-curve panel: H_ax and H_diag vs lambda in [2, 30] m
    ph, pw = 300, top.shape[1]
    panel = np.full((ph, pw, 3), 24, dtype=np.uint8)
    lam = np.linspace(2.0, 30.0, pw)
    hax = np.sin(6.0 * np.pi / lam) ** 2
    hdg = 1.0 - np.cos(np.sqrt(2.0) * 6.0 * np.pi / lam)
    def to_px(lamv, hv):
        x = (lamv - 2.0) / 28.0 * (pw - 1)
        y = (ph - 20) - hv / 2.1 * (ph - 40)
        return x, y
    # gridlines at H = 0, 1, 2
    for hv, c in ((0.0, 60), (1.0, 60), (2.0, 60)):
        _, y = to_px(2.0, hv)
        panel[int(y), :, :] = c
    xs, ys = to_px(lam, hax)
    draw_line(panel, xs, ys, np.array([255, 190, 80], dtype=np.uint8))    # axial: orange
    xs, ys = to_px(lam, hdg)
    draw_line(panel, xs, ys, np.array([110, 170, 255], dtype=np.uint8))   # diagonal: blue
    # markers: axial zeros 6,3,2 (down ticks), peaks 12,4 axial and 8.49 diagonal (up ticks)
    for lv, hv, col in ((6.0, 0.0, (255, 80, 80)), (3.0, 0.0, (255, 80, 80)),
                        (2.0, 0.0, (255, 80, 80)), (12.0, 1.0, (255, 255, 255)),
                        (4.0, 1.0, (255, 255, 255)), (6.0 * np.sqrt(2.0), 2.0, (255, 255, 255))):
        x, y = to_px(lv, hv)
        x, y = int(x), int(y)
        panel[max(y - 6, 0):min(y + 7, ph), max(x - 1, 0):x + 2] = col

    strip = np.full((26, pw, 3), 24, dtype=np.uint8)
    img = np.concatenate([top, strip, panel], axis=0)
    Image.fromarray(img).save(PNG)
    print("      wrote %s  (relief map %dx%d + transfer curves; sat frac %.2f%%)"
          % (PNG, top.shape[1], top.shape[0], 100.0 * sat))


def main():
    rng = np.random.default_rng(0)
    part_transfer(rng)
    part_dc(rng)
    part_fold()
    part_renorm()
    part_hemisphere(rng)
    part_narrowness()
    part_waterline(rng)
    z, rr_pad, sat = part_real()
    render(z, rr_pad, sat)
    print()
    if FAILS:
        print("FAILED (%d): %s" % (len(FAILS), "; ".join(FAILS)))
        sys.exit(1)
    print("ALL PASS (bed_relief)")


if __name__ == "__main__":
    main()
