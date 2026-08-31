# proofs/ripple_prefilter.py
# ALGEBRA FIRST proof for the ripple feature: the anisotropic Gaussian footprint
# prefilter w = exp(-0.5*|kf|^2), kf = 0.5*(k.fpx, k.fpz)  (Water.hlsl:666-667)
# vs brute-force supersampled ground truth at grazing incidence, plus every
# closed-form pin from the corrected derivation/package.
#
# Deterministic: fixed seeds, stdlib + numpy only. Renders proofs/ripple_prefilter.png
# via PIL (no matplotlib). Exits nonzero on any FAIL.
#
# Corrected pins applied per ripple.verdicts.md:
#   mss(20) = 2.29610e-3 (not 2.29627e-3); torus-discrepancy grid convention pinned
#   to corners at j/32, j=0..32; lambda_c band 1 = 26.8 m (not used numerically here).

import math
import os
import sys

import numpy as np
from PIL import Image, ImageDraw

HERE = os.path.dirname(os.path.abspath(__file__))

# ---------------------------------------------------------------- constants
# All verbatim from Water.hlsl (lines cited in the package).
N = 96                       # :616
T_MIN = 0.70                 # :617
T_MAX = 4.00                 # :618
G = 9.81                     # :654
AMP_C = 0.0156 * math.sqrt(20.0 / N)   # :619-627  (1/sqrt(N) variance law)
SPREAD = 62.0                # :628 half-width, degrees
PHI_HAT = 0.6180339887498949 # :657  1/phi
BETA = 0.7548776662466927    # :675  inverse plastic constant
P_EXP = 1.25                 # :656  a = AMP_C * k^-1.25
SIGMA_PIX = 0.5              # :666  screen-space Gaussian sigma, pixels
KILL = 1e-3                  # :668  early-out threshold on w
MWD = 30.0                   # arbitrary mean wave direction for the test field

failures = []


def check(name, cond, detail=""):
    tag = "PASS" if cond else "FAIL"
    print(f"[{tag}] {name}" + (f"  ({detail})" if detail else ""))
    if not cond:
        failures.append(name)


# ---------------------------------------------------------------- component set
def component_set(n):
    i = np.arange(n, dtype=np.float64)
    t = (i + 0.5) / n                              # :651 midpoint sampling
    T = T_MIN * (T_MAX / T_MIN) ** t               # :652
    sg = 2.0 * np.pi / T                           # :653
    k = sg * sg / G                                # :654 deep-water dispersion
    a_c = 0.0156 * math.sqrt(20.0 / n)
    a = a_c * k ** (-P_EXP)                        # :656
    gi = np.mod(i * PHI_HAT, 1.0)                  # :657
    pr = np.radians(np.mod(MWD + SPREAD * (2.0 * gi - 1.0) + 180.0, 360.0))  # :658
    d = np.stack([np.sin(pr), np.cos(pr)], axis=-1)  # (n,2) heading convention
    ph0 = 2.0 * np.pi * np.mod(i * BETA, 1.0)      # :675
    return T, sg, k, a, d, ph0


T96, sg96, k96, a96, d96, ph096 = component_set(N)

print("=" * 78)
print("ripple_prefilter.py -- anisotropic footprint prefilter proof")
print("=" * 78)

# ---------------------------------------------------------------- 1. mss closed form
ak2 = (a96 * k96) ** 2 / 2.0
mss_discrete = float(np.sum(ak2))
q = (T_MAX / T_MIN) ** (1.0 / N)
k_top = 4.0 * np.pi ** 2 / (G * T_MIN ** 2)
mss_closed = (AMP_C ** 2 / 2.0) * k_top ** (-0.5) * math.sqrt(q) * (q ** N - 1.0) / (q - 1.0)

check("ripple_mss_closed_form (discrete == geometric closed form, rel 1e-9)",
      abs(mss_discrete / mss_closed - 1.0) <= 1e-9,
      f"discrete={mss_discrete:.6e} closed={mss_closed:.6e}")
check("ripple_mss_closed_form (absolute pin |mss - 2.29679e-3| <= 5e-7)",
      abs(mss_discrete - 2.29679e-3) <= 5e-7,
      f"mss={mss_discrete:.6e}")

# 1/sqrt(N) invariance: N=20 with the same A_C(N) formula. Corrected pin 2.29610e-3.
_, _, k20, a20, _, _ = component_set(20)
mss20 = float(np.sum((a20 * k20) ** 2 / 2.0))
check("ripple_amp_sqrtN_invariance (|mss(20)/mss(96) - 1| <= 1e-3)",
      abs(mss20 / mss_discrete - 1.0) <= 1e-3,
      f"mss(20)={mss20:.6e} ratio_dev={abs(mss20/mss_discrete-1.0):.3e}")
check("ripple_amp_sqrtN_invariance (corrected pin mss(20) = 2.29610e-3, +-5e-7)",
      abs(mss20 - 2.29610e-3) <= 5e-7,
      f"mss(20)={mss20:.6e}")

# ---------------------------------------------------------------- 2. golden three-gap
pts = np.sort(np.mod(np.arange(N) * PHI_HAT, 1.0))
gaps = np.diff(np.concatenate([pts, [pts[0] + 1.0]]))  # circular
gq = np.round(gaps / 1e-6) * 1e-6                       # 1e-6 quantization grouping
vals, counts = np.unique(gq, return_counts=True)
# collapse quantization neighbors (values within 2e-6)
merged = []
for v, c in zip(vals, counts):
    if merged and abs(v - merged[-1][0]) <= 2e-6:
        merged[-1][1] += c
    else:
        merged.append([v, c])
vals = np.array([m[0] for m in merged])
counts = np.array([m[1] for m in merged])
expected_gaps = [(0.0050250, 7), (0.0081306, 41), (0.0131556, 48)]
ok3 = len(vals) == 3
if ok3:
    for (ev, ec), v, c in zip(expected_gaps, vals, counts):
        ok3 = ok3 and abs(v - ev) <= 1e-5 and c == ec
maxgapN = float(np.max(gaps) * N)
check("ripple_golden_three_gap (3 distinct gaps, pinned values/counts)",
      ok3, f"gaps={[(round(float(v),7), int(c)) for v, c in zip(vals, counts)]}")
check("ripple_golden_three_gap (largest = sum of other two, 1e-9)",
      ok3 and abs(vals[2] - (vals[0] + vals[1])) <= 1e-9)
check("ripple_golden_three_gap (adjacent ratio = phi, 1e-4)",
      ok3 and abs(vals[1] / vals[0] - (1.0 + math.sqrt(5.0)) / 2.0) <= 1e-4,
      f"ratio={float(vals[1]/vals[0]):.6f}")
check("ripple_golden_three_gap (N * maxgap = 1.2629 <= 1.27)",
      abs(maxgapN - 1.2629) <= 1e-3 and maxgapN <= 1.27, f"N*maxgap={maxgapN:.4f}")

# ---------------------------------------------------------------- 3. plastic + torus
plastic_res = abs((1.0 / BETA) ** 3 - (1.0 / BETA) - 1.0)
check("ripple_phase_plastic_torus (plastic identity <= 1e-12)",
      plastic_res <= 1e-12, f"residual={plastic_res:.2e}")

# Star-discrepancy estimate, pinned grid convention: corners at j/32, j=0..32.
px = np.mod(np.arange(N) * PHI_HAT, 1.0)
py = np.mod(np.arange(N) * BETA, 1.0)
grid = np.arange(33, dtype=np.float64) / 32.0
inx = px[:, None] < grid[None, :]          # (N,33) membership in [0, x)
iny = py[:, None] < grid[None, :]
countbox = (inx.astype(np.float64).T @ iny.astype(np.float64)) / N  # (33,33)
disc = float(np.max(np.abs(countbox - grid[:, None] * grid[None, :])))
check("ripple_phase_plastic_torus (star discrepancy <= 0.045, grid j/32)",
      disc <= 0.045, f"D*={disc:.5f} (pinned measurement 0.0391)")

# ---------------------------------------------------------------- 4. dispersion
def newton_fd(k_deep, h):
    k = k_deep
    for _ in range(60):
        th = math.tanh(k * h)
        f = k * th - k_deep
        df = th + k * h * (1.0 - th * th)
        k -= f / df
    return k


ratios = {}
for Tt in (4.0, 3.0):
    sgt = 2.0 * math.pi / Tt
    kd = sgt * sgt / G
    kfd = newton_fd(kd, 6.0)
    ratios[Tt] = kfd / kd
check("ripple_dispersion_deep_validity (T=4 s, h=6 m: k_fd/k_deep = 1.0799 +-1e-3)",
      abs(ratios[4.0] - 1.0799) <= 1e-3, f"ratio={ratios[4.0]:.4f}")
check("ripple_dispersion_deep_validity (T=3 s, h=6 m: ratio = 1.0090 +-1e-3)",
      abs(ratios[3.0] - 1.0090) <= 1e-3, f"ratio={ratios[3.0]:.4f}")

kfd96 = np.array([newton_fd(kk, 6.0) for kk in k96])
werr = float(np.sum(ak2 * (kfd96 / k96 - 1.0)) / np.sum(ak2))
check("ripple_dispersion_deep_validity (slope-weighted mean wavenumber error 0.0121 +-1e-3)",
      abs(werr - 0.0121) <= 1e-3, f"weighted mean={werr:.5f}")

# ---------------------------------------------------------------- 5. filter identities
def w_aniso(kv, fpx, fpz, early_out=False):
    """w = exp(-0.5*|kf|^2), kf = 0.5*(kv.fpx, kv.fpz). kv (...,2), fpx/fpz (...,2)."""
    A = 0.5 * (kv[..., 0] * fpx[..., 0] + kv[..., 1] * fpx[..., 1])
    B = 0.5 * (kv[..., 0] * fpz[..., 0] + kv[..., 1] * fpz[..., 1])
    w = np.exp(-0.5 * (A * A + B * B))
    if early_out:
        w = np.where(w <= KILL, 0.0, w)
    return w


rng = np.random.default_rng(20260831)

# (a) DC: w(k=0) = 1 exactly, arbitrary frames.
fpx_r = rng.normal(size=(50, 2))
fpz_r = rng.normal(size=(50, 2))
wdc = w_aniso(np.zeros((50, 2)), fpx_r, fpz_r)
check("ripple_prefilter_dc (w(k=0) = 1, |w-1| <= 1e-7)",
      float(np.max(np.abs(wdc - 1.0))) <= 1e-7)

# (b) isotropic reduction: fpx=R(L,0), fpz=R(0,L) -> w = exp(-|k|^2 L^2/8).
ok_iso = True
for _ in range(120):
    th = rng.uniform(0, 2 * np.pi)
    R = np.array([[math.cos(th), -math.sin(th)], [math.sin(th), math.cos(th)]])
    L = rng.uniform(0.01, 5.0)
    kv = rng.normal(size=2) * rng.uniform(0.1, 3.0)
    w = w_aniso(kv, R @ np.array([L, 0.0]), R @ np.array([0.0, L]))
    w_ref = math.exp(-float(kv @ kv) * L * L / 8.0)
    if abs(w - w_ref) > 1e-12 * max(w_ref, 1e-300):
        ok_iso = False
check("ripple_prefilter_isotropic_reduction (120 random R,L,k; rel 1e-12)", ok_iso)

# (c) Gram/bivector: det(J J^T) = |fpx ^ fpz|^2.
ok_gram = True
for _ in range(120):
    fx = rng.normal(size=2)
    fz = rng.normal(size=2)
    J = np.column_stack([fx, fz])
    detM = float(np.linalg.det(J @ J.T))
    biv2 = (fx[0] * fz[1] - fx[1] * fz[0]) ** 2
    scale = float((fx @ fx) * (fz @ fz))   # Hadamard bound: det <= |fpx|^2 |fpz|^2
    if abs(detM - biv2) > 1e-9 * scale:
        ok_gram = False
check("ripple_prefilter_gram_bivector (det(JJ^T) = |fpx^fpz|^2, rel 1e-9 of Hadamard scale)",
      ok_gram)

# (d) equivariance: w(Rk; [R fpx | R fpz]) = w(k; [fpx | fpz]).
ok_eq = True
for _ in range(120):
    th = rng.uniform(0, 2 * np.pi)
    R = np.array([[math.cos(th), -math.sin(th)], [math.sin(th), math.cos(th)]])
    fx = rng.normal(size=2)
    fz = rng.normal(size=2)
    kv = rng.normal(size=2) * rng.uniform(0.1, 3.0)
    w1 = w_aniso(kv, fx, fz)
    w2 = w_aniso(R @ kv, R @ fx, R @ fz)
    if abs(w1 - w2) > 1e-12 * max(w1, 1e-300):
        ok_eq = False
check("ripple_prefilter_equivariance (co-rotates with frame, rel 1e-12)", ok_eq)

# ---------------------------------------------------------------- 6. ledger w vs w^2
# Variance of a Gaussian-footprint-filtered sinusoid = w^2 * raw variance.
gh_n, gh_w = np.polynomial.hermite.hermgauss(25)


def filtered_value(amp, kv, fpx, fpz, psi):
    """2D Gauss-Hermite quadrature of a*sin(kv.p + psi) over the footprint Gaussian
    (sigma = 0.5*fpx, 0.5*fpz), separable along the fpx/fpz axes."""
    au = float(kv @ fpx) * math.sqrt(2.0) * 0.5   # phase per unit GH node along fpx
    av = float(kv @ fpz) * math.sqrt(2.0) * 0.5
    Fx = float(np.sum(gh_w * np.cos(au * gh_n))) / math.sqrt(math.pi)
    Fz = float(np.sum(gh_w * np.cos(av * gh_n))) / math.sqrt(math.pi)
    return amp * math.sin(psi) * Fx * Fz


# Pinned single case: kL = 2.4, sigma = L/2 -> w = exp(-(kL)^2/8) = 0.4868, w^2 = 0.2369.
L = 1.0
kk = 2.4 / L
w_pin = math.exp(-(kk * L) ** 2 / 8.0)
psis = np.linspace(0, 2 * np.pi, 720, endpoint=False)
vals = np.array([filtered_value(1.0, np.array([kk, 0.0]),
                                np.array([L, 0.0]), np.array([0.0, L]), p) for p in psis])
ratio_pin = float(np.mean(vals ** 2) / 0.5)
check("ripple_ledger_w_squared (kL=2.4 quadrature ratio = w^2 = 0.2369, rel 1e-2)",
      abs(ratio_pin / w_pin ** 2 - 1.0) <= 1e-2,
      f"ratio={ratio_pin:.4f} w^2={w_pin**2:.4f} w={w_pin:.4f}")

# 200 random (J, k): ratio == w^2 and NOT w whenever w in [0.2, 0.8].
ok_lw, ok_lnw, n_mid = True, True, 0
for _ in range(200):
    fx = rng.normal(size=2) * rng.uniform(0.2, 2.0)
    fz = rng.normal(size=2) * rng.uniform(0.2, 2.0)
    kv = rng.normal(size=2)
    kv *= rng.uniform(0.2, 3.0) / max(np.linalg.norm(kv), 1e-12)
    w = float(w_aniso(kv, fx, fz))
    if w < 0.05:
        continue
    vals = np.array([filtered_value(1.0, kv, fx, fz, p) for p in psis[::6]])
    ratio = float(np.mean(vals ** 2) / 0.5)
    if abs(ratio - w * w) > 1e-2:
        ok_lw = False
    if 0.2 <= w <= 0.8:
        n_mid += 1
        if abs(ratio - w) <= 0.1:
            ok_lnw = False
check("ripple_ledger_w_squared (200 random J,k: |ratio - w^2| < 1e-2)", ok_lw)
check("ripple_ledger_w_squared (ratio != w: |ratio - w| > 0.1 for w in [0.2,0.8])",
      ok_lnw and n_mid >= 20, f"mid-band cases={n_mid}")

# ---------------------------------------------------------------- 7. fold curves
def smoothstep(a, b, x):
    tt = np.clip((x - a) / (b - a), 0.0, 1.0)
    return tt * tt * (3.0 - 2.0 * tt)


x = np.linspace(0.0, 1.0, 20001)
diff = np.abs((1.0 - smoothstep(0.12, 0.5, x)) - np.exp(-np.pi ** 2 * x ** 2 / 2.0))
imax = int(np.argmax(diff))
check("fold_smoothstep_vs_gaussian_gap (max diff 0.321 +-0.01)",
      abs(diff[imax] - 0.321) <= 0.01, f"max={diff[imax]:.4f}")
check("fold_smoothstep_vs_gaussian_gap (argmax x = 0.456 +-0.01)",
      abs(x[imax] - 0.456) <= 0.01, f"argmax={x[imax]:.4f}")
h_ss = x[int(np.argmin(np.abs((1.0 - smoothstep(0.12, 0.5, x)) - 0.5)))]
h_g = x[int(np.argmin(np.abs(np.exp(-np.pi ** 2 * x ** 2 / 2.0) - 0.5)))]
check("fold_smoothstep_vs_gaussian_gap (half-points 0.310 vs 0.375, +-0.005)",
      abs(h_ss - 0.310) <= 0.005 and abs(h_g - 0.375) <= 0.005,
      f"smoothstep half={h_ss:.4f} gaussian half={h_g:.4f}")

# telescope: footPx == bT -> wDet = saturate(wPix - wRing) = 0 identically,
# for the smoothstep pair and for the Gaussian pair alike.
lam = np.geomspace(0.1, 500.0, 400)
bT = np.geomspace(0.01, 100.0, 300)
TT, LL = np.meshgrid(bT, lam)
w_ss = 1.0 - smoothstep(0.12 * LL, 0.5 * LL, TT)
w_g = np.exp(-np.pi ** 2 * (TT / LL) ** 2 / 2.0)
tel_ss = float(np.max(np.abs(np.clip(w_ss - w_ss, 0.0, 1.0))))
tel_g = float(np.max(np.abs(np.clip(w_g - w_g, 0.0, 1.0))))
check("fold_telescope_zero_detail (wPix==wRing -> wDet == 0, smoothstep & Gaussian)",
      tel_ss == 0.0 and tel_g == 0.0)

# ---------------------------------------------------------------- 8. headline example
# Derivation 3.3: h=2 m, r=100 m, delta=1e-3 -> L_perp=0.1 m, L_par=5 m; lambda=2.2 m.
k_ex = 2.0 * np.pi / 2.2
Lp, Ll = 0.1, 5.0
w_across = float(w_aniso(np.array([k_ex, 0.0]), np.array([Lp, 0.0]), np.array([0.0, Ll])))
w_along = float(w_aniso(np.array([0.0, k_ex]), np.array([Lp, 0.0]), np.array([0.0, Ll])))
check("grazing example (across-view w = 0.990 +-0.001)",
      abs(w_across - 0.990) <= 1e-3, f"w_across={w_across:.4f}")
check("grazing example (along-view w <= 1e-10)",
      w_along <= 1e-10, f"w_along={w_along:.2e}")

# ================================================================ grazing field test
# Geometry: helm camera h=2 m, pixel angle delta=0.00097 rad (1080p, 60 deg vfov).
# fpx = (r*delta, 0) across-view; fpz = (0, r*delta/sin(theta_g)), sin = h/sqrt(r^2+h^2).
H_CAM = 2.0
DELTA = 0.00097
N_R, N_X = 64, 128
r_grid = np.geomspace(5.0, 500.0, N_R)
sin_tg = H_CAM / np.sqrt(r_grid ** 2 + H_CAM ** 2)
Lx = r_grid * DELTA                    # across-view footprint extent
Lz = r_grid * DELTA / sin_tg           # along-view footprint extent

akw = a96 * k96                        # slope amplitude per component

# Ground-truth stencil. SPEC DEVIATION (documented): the spec's 15x15 Gauss-Hermite
# stencil aliases WITHIN the stencil at grazing -- at r=500 m the along-view footprint
# sigma is ~60 m against 0.78 m wavelengths (~690 rad of phase across the stencil),
# far beyond GH-15's ~degree-29 exactness, so the "ground truth" itself leaked O(0.1)
# spurious amplitude (measured A/GT rms error 0.75 in the 100-500 m bucket). Replaced
# with a dense separable Gaussian-weighted supersample: uniform nodes over +-5 sigma,
# >= 8 samples per cycle of the shortest component per axis (adaptive count). Because
# the footprint axes are orthogonal here and the stencil is a tensor product, the
# separable evaluation is bit-for-bit the same sum as the full 2D supersample,
# reordered -- it IS brute-force supersampling, just converged.
GT_HALF_SIGMAS = 5.0
LAMBDA_MIN = 2.0 * np.pi / float(np.max(k96))


def gt_axis_factors(Lax):
    """Per-component stencil factor F_c = sum_j W_j cos(k_ax,c * u_j) for a Gaussian
    (sigma = 0.5*Lax) supersample along one footprint axis. Returns callable input:
    the per-component axis wavenumbers (array), -> factors (array)."""
    sigma = 0.5 * Lax
    span = 2.0 * GT_HALF_SIGMAS * sigma
    n_nodes = max(801, int(span / (LAMBDA_MIN / 8.0)) | 1)   # odd, >= 8 per cycle
    u = np.linspace(-GT_HALF_SIGMAS * sigma, GT_HALF_SIGMAS * sigma, n_nodes)
    wgt = np.exp(-u * u / (2.0 * sigma * sigma))
    wgt /= np.sum(wgt)
    def factors(k_ax):
        return np.cos(np.outer(k_ax, u)) @ wgt
    return factors


def field_rows(gtime):
    """Return dict of slope fields (N_R, N_X, 2) for GT (brute-force supersampled)
    and variants A (aniso closed form), B (no filter), C (iso-max), D (iso-min)."""
    out = {v: np.zeros((N_R, N_X, 2)) for v in ("GT", "A", "B", "C", "D")}
    for m in range(N_R):
        r = r_grid[m]
        lx, lz = Lx[m], Lz[m]
        fpx = np.array([lx, 0.0])
        fpz = np.array([0.0, lz])
        xs = (np.arange(N_X) - (N_X - 1) / 2.0) * lx    # one pixel per sample
        # component phases at the pixel centers: theta[c, p]
        theta = (k96[:, None] * (d96[:, 0:1] * xs[None, :] + d96[:, 1:2] * r)
                 - sg96[:, None] * gtime + ph096[:, None])
        s_ctr = np.sin(theta)                            # (96, N_X)

        # --- ground truth: brute-force supersample of the UNFILTERED field over
        # the footprint Gaussian (sigma = 0.5*fpx, 0.5*fpz), separable evaluation
        # of the dense tensor-product stencil (see deviation note above). By node
        # symmetry the odd (sin) part of each axis sum vanishes, so the stencil
        # sum collapses to Fx_c * Fz_c * sin(theta_c) exactly.
        Fx = gt_axis_factors(lx)(k96 * d96[:, 0])        # (96,)
        Fz = gt_axis_factors(lz)(k96 * d96[:, 1])        # (96,)
        s_gt = (Fx * Fz)[:, None] * s_ctr                # (96, N_X)

        # --- analytic variant weights per component
        kd = k96[:, None] * d96                          # wavevectors (96,2)
        wA = w_aniso(kd, fpx, fpz, early_out=True)       # exact aniso, shader early-out
        wB = np.ones(N)                                  # no filter
        lmax, lmin = max(lx, lz), min(lx, lz)
        wC = np.exp(-(k96 * lmax) ** 2 / 8.0)            # isotropic on the long axis
        wC = np.where(wC <= KILL, 0.0, wC)
        wD = np.exp(-(k96 * lmin) ** 2 / 8.0)            # isotropic on footPx (cross)
        wD = np.where(wD <= KILL, 0.0, wD)

        for name, s, w in (("GT", s_gt, None), ("A", s_ctr, wA), ("B", s_ctr, wB),
                           ("C", s_ctr, wC), ("D", s_ctr, wD)):
            amp = akw if w is None else akw * w
            out[name][m] = -np.einsum("c,cp,cx->px", amp, s, d96)
    return out


print("-" * 78)
print("grazing field test: 64 log ranges 5..500 m x 128 across-view pixels, h=2 m")
F0 = field_rows(0.0)

buckets = [("5-20 m", (r_grid >= 5.0) & (r_grid < 20.0)),
           ("20-100 m", (r_grid >= 20.0) & (r_grid < 100.0)),
           ("100-500 m", (r_grid >= 100.0) & (r_grid <= 500.0))]


def rms(field, mask):
    return float(np.sqrt(np.mean(np.sum(field[mask] ** 2, axis=-1))))


print(f"{'bucket':<10} {'rms(GT)':>10} {'A err':>10} {'B err':>10} {'C err':>10} "
      f"{'D err':>10} {'A/GT':>7} {'C/A':>7} {'B/A':>7}")
stats = {}
for bname, bmask in buckets:
    g = rms(F0["GT"], bmask)
    e = {v: rms(F0[v] - F0["GT"], bmask) for v in ("A", "B", "C", "D")}
    stats[bname] = (g, e)
    print(f"{bname:<10} {g:>10.4e} {e['A']:>10.4e} {e['B']:>10.4e} {e['C']:>10.4e} "
          f"{e['D']:>10.4e} {e['A']/g:>7.3f} {e['C']/max(e['A'],1e-300):>7.1f} "
          f"{e['B']/max(e['A'],1e-300):>7.1f}")

for bname, _ in buckets:
    g, e = stats[bname]
    check(f"grazing target 1: rms(A-GT) <= 0.10*rms(GT)  [{bname}]",
          e["A"] <= 0.10 * g, f"A/GT={e['A']/g:.4f}")
for bname in ("20-100 m", "100-500 m"):   # r/h >= 10 buckets
    g, e = stats[bname]
    check(f"grazing target 2: rms(C-GT) >= 3*rms(A-GT)  [{bname}] (isotropic-max over-blur)",
          e["C"] >= 3.0 * e["A"], f"C/A={e['C']/e['A']:.1f}")
    check(f"grazing target 3: rms(B-GT) >= 3*rms(A-GT)  [{bname}] (unfiltered aliasing)",
          e["B"] >= 3.0 * e["A"], f"B/A={e['B']/e['A']:.1f}")

# --- temporal aliasing witness: error autocorrelation across 30 Hz frames.
F1 = field_rows(1.0 / 30.0)
F2 = field_rows(2.0 / 30.0)


def err_corr(Fa, Fb, variant):
    ea = (Fa[variant] - Fa["GT"]).ravel()
    eb = (Fb[variant] - Fb["GT"]).ravel()
    ea = ea - ea.mean()
    eb = eb - eb.mean()
    den = math.sqrt(float(ea @ ea) * float(eb @ eb))
    return float(ea @ eb) / den if den > 0 else 1.0


print("temporal witness (frame-to-frame error autocorrelation, 30 Hz):")
print("  (B's error is leaked full-amplitude short-wave content: it stays a")
print("   coherent field but CRAWLS -- each component's error advances sg/30 rad")
print("   per frame at a false spatial frequency; A's residual is the early-out")
print("   truncation, orders of magnitude smaller. Reported, not asserted.)")
for v in ("A", "B"):
    c01 = err_corr(F0, F1, v)
    c12 = err_corr(F1, F2, v)
    e_abs = float(np.sqrt(np.mean(np.sum((F0[v] - F0["GT"]) ** 2, axis=-1))))
    print(f"  variant {v}: corr(t0,t1)={c01:+.4f}  corr(t1,t2)={c12:+.4f}  "
          f"rms err={e_abs:.3e}")

# ================================================================ figure (PIL)
def colormap_diverging(v, scale):
    """signed field -> blue-white-red, v/scale clipped to [-1,1]."""
    t = np.clip(v / scale, -1.0, 1.0)
    r = np.where(t >= 0, 1.0, 1.0 + t)
    gc = 1.0 - np.abs(t) * 0.55
    b = np.where(t <= 0, 1.0, 1.0 - t)
    return np.stack([r, gc, b], axis=-1)


def colormap_hot(v, scale):
    t = np.clip(v / scale, 0.0, 1.0)
    r = np.clip(3.0 * t, 0, 1)
    gc = np.clip(3.0 * t - 1.0, 0, 1)
    b = np.clip(3.0 * t - 2.0, 0, 1)
    return np.stack([r, gc, b], axis=-1)


def to_panel(rgb01, up=5):
    img = (np.clip(rgb01, 0, 1) * 255).astype(np.uint8)
    img = np.kron(img, np.ones((up, up, 1), dtype=np.uint8))
    return img


sx_scale = 3.0 * float(np.sqrt(np.mean(F0["GT"][..., 0] ** 2)))
err_scale = float(np.sqrt(np.mean(np.sum((F0["B"] - F0["GT"]) ** 2, axis=-1))))
panels_top = [("ground truth (supersampled)", colormap_diverging(F0["GT"][..., 0], sx_scale)),
              ("anisotropic prefilter (A)", colormap_diverging(F0["A"][..., 0], sx_scale)),
              ("isotropic-max (C)", colormap_diverging(F0["C"][..., 0], sx_scale))]
panels_bot = [("|A - GT| error", colormap_hot(np.linalg.norm(F0["A"] - F0["GT"], axis=-1), err_scale)),
              ("|C - GT| error (over-blur)", colormap_hot(np.linalg.norm(F0["C"] - F0["GT"], axis=-1), err_scale)),
              ("|B - GT| error (aliasing)", colormap_hot(np.linalg.norm(F0["B"] - F0["GT"], axis=-1), err_scale))]

UP = 5
pw, ph = N_X * UP, N_R * UP
margin, header, label_h = 12, 46, 20
W_img = margin + 3 * (pw + margin)
H_img = header + 2 * (ph + label_h + margin) + margin
canvas = Image.new("RGB", (W_img, H_img), (18, 18, 24))
draw = ImageDraw.Draw(canvas)
draw.text((margin, 8),
          "ripple footprint prefilter -- grazing slope field (x-slope), h=2 m, r=5..500 m "
          "(top->bottom log range), 128 px across", fill=(230, 230, 230))
draw.text((margin, 24),
          f"mss={mss_discrete:.4e}  bucket A/GT errs: "
          + "  ".join(f"{b}:{stats[b][1]['A']/stats[b][0]:.3f}" for b, _ in buckets),
          fill=(180, 200, 230))
for row, panels in enumerate((panels_top, panels_bot)):
    y0 = header + row * (ph + label_h + margin)
    for col, (title, rgb) in enumerate(panels):
        x0 = margin + col * (pw + margin)
        canvas.paste(Image.fromarray(to_panel(rgb, UP)), (x0, y0 + label_h))
        draw.text((x0, y0 + 3), title, fill=(220, 220, 220))
png_path = os.path.join(HERE, "ripple_prefilter.png")
canvas.save(png_path)
print(f"figure written: {png_path}")

# ================================================================ verdict
print("-" * 78)
if failures:
    print(f"{len(failures)} FAILURE(S): " + ", ".join(failures))
    sys.exit(1)
print("ALL ASSERTS PASSED")
