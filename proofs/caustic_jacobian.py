# proofs/caustic_jacobian.py -- CAUSTICS: the tangent bivector, the ray-map Jacobian,
# and the corrected physical curvature (M7w).  Algebra first.
#
# The derivation (scratchpad math/caustics.derivation.md, post-verdict) says three things
# this prototype must pin with the SAME numbers:
#
#   1. One Gerstner component is a rotor field: R(a yhat)R~ with R = exp(-B theta/2),
#      B = dhat ^ yhat, collapses to  a cos(theta) yhat - a sin(theta) dhat.  From it the
#      tangent bivector T = t_z ^ t_x yields BOTH the unnormalized normal (its dual)
#        N = (hs a k sin(t) dx,  1 - s a k cos(t),  hs a k sin(t) dz)
#      and the area Jacobian (its horizontal part)
#        areaJac = |det(I+J)| = |1 - s a k cos(t)|   (single component, exact),
#      plus the exact analytic Laplacian  lap = -a k^2 cos(phi - sigma t)  with the phase
#      advanced by the cos/sin spinor product, never atan2.
#
#   2. The TRUE physical curvature of the displaced surface (the fatal verdict fix):
#        eta''_phys = -a k^2 (cos t - s a k) / (1 - s a k cos t)^3
#      (the erroneous draft had a cos 2t in the numerator; it is wrong everywhere but
#      theta in {0, pi} -- documented below by direct numeric differentiation).
#      Crest identity: eta''_phys(crest) = lap_param / areaJac^2.
#
#   3. The bed-irradiance gain.  Ground truth is brute force: uniform sun flux per unit
#      physical horizontal area, full vector Snell at the exact surface normal, march to
#      the bed, histogram.  Against it, sampled at the entry parameter (+ sun-run offset):
#        (a) G_phys   = 1 / |1 + h K eta''_phys|          (physical bookkeeping: uniform
#            per-physical-area capture cancels det(I+J); K = 1 - 1/1.333)
#        (b) G_vqview = 1 / [max(areaJac,.05) max(1 - h*0.25*lap_param,.05)]   (shipped)
#        (c) G_laponly= 1 / max(1 - h*0.25*lap_param,.05)
#      Corrected correlation gate: corr(a) >= 0.99 (the cos-2t formula only reached ~0.93).
#      At s = 0 the signs face each other nakedly: ground truth is BRIGHT under crests;
#      the shipped (1 - hK lap) form anticorrelates.
#      The sun-entry offset (derivation section 9): for oblique sun the bed pattern sits
#      displaced by sunRun = h tan(asin(sin(zen)/1.333)) from the entry-sampled field --
#      cross-correlation peak must land there to within one bin.
#
# Figure: proofs/caustic_jacobian.png -- caustic-gain over a sloped sand bed under one
# Gerstner component (Beer-Lambert water, washout + clamp verbatim from Water.hlsl
# :933-934) and the defocus/focus diverging map of the same gain field.
#
# Deterministic (fixed seed), stdlib + numpy + PIL only.  Exit nonzero on any FAIL.
import math
import os
import sys

import numpy as np
from PIL import Image, ImageDraw

RNG = np.random.default_rng(20260831)
HERE = os.path.dirname(os.path.abspath(__file__))

# ---- constants, verbatim from the package (Water.hlsl / OceanFft.h citations) --------
N_WATER = 1.333
NW = 0.750188            # Water.hlsl:108
K_CODE = 0.25            # Water.hlsl:927, :780
K_EXACT = 1.0 - 1.0 / N_WATER
MU = 1.0 / N_WATER       # exact refraction ratio for the ground truth
FRESNEL_IN = 0.94        # Water.hlsl:110
SUN_IRR = np.array([1.350, 1.283, 1.161])   # Water.hlsl:101
KD = np.array([0.36, 0.105, 0.06])          # Beer-Lambert, radiometry topic
FLOOR_AJ = 0.05
FLOOR_CONV = 0.05
CLAMP_LO, CLAMP_HI = 0.35, 2.6              # Water.hlsl:934
WASH_A, WASH_B = 4.0, 20.0                  # Water.hlsl:933

FAILS = []


def gate(name, ok, detail):
    tag = "PASS" if ok else "FAIL"
    print(f"{tag}  {name}  {detail}")
    if not ok:
        FAILS.append(name)


def mixed_ok(err_abs, expected, abs_tol=1e-7, rel_tol=1e-5):
    """verdict fix: abs error < abs_tol OR rel error < rel_tol, denom max(|expected|,eps)."""
    rel = err_abs / np.maximum(np.abs(expected), 1e-300)
    return bool(np.all((err_abs < abs_tol) | (rel < rel_tol)))


def pearson(u, v):
    u = u - u.mean()
    v = v - v.mean()
    return float(np.dot(u, v) / math.sqrt(np.dot(u, u) * np.dot(v, v)))


def smoothstep(a, b, x):
    t = np.clip((x - a) / (b - a), 0.0, 1.0)
    return t * t * (3.0 - 2.0 * t)


# =====================================================================================
# PART A -- closed-form pins (double precision)
# =====================================================================================
print("== PART A: single-Gerstner closed forms ==")

# A1 rotor sandwich: R (a yhat) R~, R = exp(-B theta/2), B = dhat ^ yhat.  The dual axis
# of B is u = dhat x yhat; the sandwich is the quaternion rotation by theta about u.
def quat_mul(p, q):
    pw, px, py, pz = p
    qw, qx, qy, qz = q
    return (pw*qw - px*qx - py*qy - pz*qz,
            pw*qx + px*qw + py*qz - pz*qy,
            pw*qy - px*qz + py*qw + pz*qx,
            pw*qz + px*qy - py*qx + pz*qw)


err_max = 0.0
for _ in range(200):
    psi = RNG.uniform(0, 2*math.pi)
    dh = (math.cos(psi), 0.0, math.sin(psi))          # horizontal unit dhat
    a = RNG.uniform(0.01, 3.0)
    th = RNG.uniform(-math.pi, math.pi)
    u = (dh[1]*0 - 0*dh[2], dh[2]*0 - dh[0]*0, dh[0]*0 - dh[1]*0)  # placeholder
    # u = dhat x yhat, yhat = (0,1,0):
    u = (dh[1]*0 - dh[2]*1, dh[2]*0 - dh[0]*0, dh[0]*1 - dh[1]*0)  # = (-dz, 0, dx)
    q = (math.cos(th/2), math.sin(th/2)*u[0], math.sin(th/2)*u[1], math.sin(th/2)*u[2])
    qc = (q[0], -q[1], -q[2], -q[3])
    v = (0.0, 0.0, a, 0.0)                            # a * yhat as pure quaternion
    r = quat_mul(quat_mul(q, v), qc)
    got = np.array(r[1:])
    want = np.array([-a*math.sin(th)*dh[0], a*math.cos(th), -a*math.sin(th)*dh[2]])
    err_max = max(err_max, float(np.max(np.abs(got - want))))
gate("A1_rotor_sandwich_expansion", err_max < 1e-12,
     f"max|R(a yhat)R~ - (a cos th yhat - a sin th dhat)| = {err_max:.2e} (tol 1e-12)")


# multi-component EvalSurface equivalent (2D reference plane q = (x, z))
def eval_surface(qx, qz, comps, s, t=0.0):
    eta = np.zeros_like(qx, float)
    ex = np.zeros_like(qx, float)
    ez = np.zeros_like(qx, float)
    dx_ = np.zeros_like(qx, float)
    dz_ = np.zeros_like(qx, float)
    j00 = np.zeros_like(qx, float)
    j01 = np.zeros_like(qx, float)
    j11 = np.zeros_like(qx, float)
    lap = np.zeros_like(qx, float)
    for (a, k, ddx, ddz, sig, ph0) in comps:
        phi = k * (ddx*qx + ddz*qz) + ph0
        ct, st = math.cos(sig*t), math.sin(sig*t)
        c = np.cos(phi)*ct + np.sin(phi)*st      # spinor advance (Surface.hlsli:53-56)
        sn = np.sin(phi)*ct - np.cos(phi)*st
        eta += a*c
        ex += -a*k*sn*ddx
        ez += -a*k*sn*ddz
        dx_ += -s*a*sn*ddx
        dz_ += -s*a*sn*ddz
        akc = s*a*k*c
        j00 -= akc*ddx*ddx
        j01 -= akc*ddx*ddz
        j11 -= akc*ddz*ddz
        lap += -a*k*k*c
    area = np.abs((1.0 + j00)*(1.0 + j11) - j01*j01)   # abs() as Water.hlsl:244
    return eta, ex, ez, dx_, dz_, j00, j01, j11, area, lap


# A2 jacobian symmetry: J01 == J10 identically (one rotor plane per component).  The
# build stores a single off-diagonal channel, so we verify the theorem it encodes: for
# random states the analytic J01 equals an independent finite-difference dDx/dz & dDz/dx.
def rand_state(nc):
    comps = []
    for _ in range(nc):
        psi = RNG.uniform(0, 2*math.pi)
        comps.append((RNG.uniform(0.02, 0.35), RNG.uniform(0.15, 1.6),
                      math.cos(psi), math.sin(psi), RNG.uniform(0.3, 2.0),
                      RNG.uniform(0, 2*math.pi)))
    return comps


comps = rand_state(16)
s_chop = 1.07
tt = 1.7
pts = RNG.uniform(-40, 40, size=(2, 24))
h_fd = 1e-5
_, _, _, Dx0, Dz0, J00, J01, J11, _, _ = eval_surface(pts[0], pts[1], comps, s_chop, tt)
_, _, _, Dxp, _, _, _, _, _, _ = eval_surface(pts[0], pts[1]+h_fd, comps, s_chop, tt)
_, _, _, Dxm, _, _, _, _, _, _ = eval_surface(pts[0], pts[1]-h_fd, comps, s_chop, tt)
_, _, _, _, Dzp, _, _, _, _, _ = eval_surface(pts[0]+h_fd, pts[1], comps, s_chop, tt)
_, _, _, _, Dzm, _, _, _, _, _ = eval_surface(pts[0]-h_fd, pts[1], comps, s_chop, tt)
dDx_dz = (Dxp - Dxm) / (2*h_fd)
dDz_dx = (Dzp - Dzm) / (2*h_fd)
e_sym = max(float(np.max(np.abs(dDx_dz - J01))), float(np.max(np.abs(dDz_dx - J01))))
gate("A2_jac_symmetry_rotor_plane", e_sym < 1e-7,
     f"16-component state: max|FD dDx/dz - J01|,|FD dDz/dx - J01| = {e_sym:.2e} (tol 1e-7)"
     " -- both off-diagonal slots are the SAME expression")

# A3 finite-difference anchor for eta gradient (the analytic path is not self-checking)
eta0, EX, EZ, _, _, _, _, _, _, LAP = eval_surface(pts[0], pts[1], comps, s_chop, tt)
ep, _, _, _, _, _, _, _, _, _ = eval_surface(pts[0]+h_fd, pts[1], comps, s_chop, tt)
em, _, _, _, _, _, _, _, _, _ = eval_surface(pts[0]-h_fd, pts[1], comps, s_chop, tt)
e_gx = float(np.max(np.abs((ep - em)/(2*h_fd) - EX)))
ep, _, _, _, _, _, _, _, _, _ = eval_surface(pts[0], pts[1]+h_fd, comps, s_chop, tt)
em, _, _, _, _, _, _, _, _, _ = eval_surface(pts[0], pts[1]-h_fd, comps, s_chop, tt)
e_gz = float(np.max(np.abs((ep - em)/(2*h_fd) - EZ)))
gate("A3_fd_anchor_gradient", max(e_gx, e_gz) < 1e-7,
     f"max|FD grad eta - analytic| = {max(e_gx, e_gz):.2e} (tol 1e-7)")

# A4/A5/A7 single-component closed forms over random phases (mixed tolerances, verdict)
n_r = 4000
a1, k1 = 0.4, 2*math.pi/8.0
psi = RNG.uniform(0, 2*math.pi)
d1 = (math.cos(psi), math.sin(psi))
s1, hs = 1.0, 1.4
sig1, ph0 = 0.9, RNG.uniform(0, 2*math.pi)
qx = RNG.uniform(-60, 60, n_r)
qz = RNG.uniform(-60, 60, n_r)
one = [(a1, k1, d1[0], d1[1], sig1, ph0)]
eta1, ex1, ez1, _, _, j00, j01, j11, area1, lap1 = eval_surface(qx, qz, one, s1, tt)
theta = k1*(d1[0]*qx + d1[1]*qz) + ph0 - sig1*tt

# normal from the tangent bivector: N = cross(t_z, t_x) with heightScale on grad only
t_xv = np.stack([1.0 + j00, hs*ex1, j01], axis=-1)
t_zv = np.stack([j01, hs*ez1, 1.0 + j11], axis=-1)
Nvec = np.cross(t_zv, t_xv)
Nwant = np.stack([hs*a1*k1*np.sin(theta)*d1[0],
                  1.0 - s1*a1*k1*np.cos(theta),
                  hs*a1*k1*np.sin(theta)*d1[1]], axis=-1)
e_n = np.abs(Nvec - Nwant)
gate("A4_normal_closed_form", mixed_ok(e_n, Nwant),
     f"max abs err = {float(e_n.max()):.2e} (mixed: abs<1e-7 or rel<1e-5)")

e_aj = np.abs(area1 - np.abs(1.0 - s1*a1*k1*np.cos(theta)))
gate("A5_areajac_closed_form", mixed_ok(e_aj, np.abs(1.0 - s1*a1*k1*np.cos(theta))),
     f"max abs err |det(I+J)| vs |1 - s a k cos| = {float(e_aj.max()):.2e}")

e_lp = np.abs(lap1 - (-a1*k1*k1*np.cos(theta)))
gate("A7_laplacian_closed_form_spinor", mixed_ok(e_lp, a1*k1*k1*np.cos(theta)),
     f"max abs err vs -a k^2 cos(phi - sigma t) = {float(e_lp.max()):.2e}"
     " (phase advanced by the cos/sin spinor product)")

# A6 zero chop: areaJac == 1 for ANY superposition
_, _, _, _, _, _, _, _, area0, _ = eval_surface(pts[0], pts[1], comps, 0.0, tt)
e_z = float(np.max(np.abs(area0 - 1.0)))
gate("A6_zero_chop_areajac_unity", e_z < 1e-7, f"max|areaJac - 1| = {e_z:.2e} (tol 1e-7)")

# A8-A10 the corrected physical curvature (THE fatal fix).
# x(q) = q - s a sin(kq), y(q) = a cos(kq):
#   d2y/dx2 = (y'' x' - y' x'')/x'^3 = -a k^2 (cos t - s a k) / (1 - s a k cos t)^3
def etapp_phys(thq, a, k, s):
    eps = s*a*k
    return -a*k*k*(np.cos(thq) - eps) / (1.0 - eps*np.cos(thq))**3


def etapp_old_wrong(thq, a, k, s):   # the pre-verdict draft, kept only as documentation
    eps = s*a*k
    return -a*k*k*(np.cos(thq) - eps*np.cos(2*thq)) / (1.0 - eps*np.cos(thq))**3


qf = np.linspace(0, 2*math.pi/k1, 20001)
dq = qf[1] - qf[0]
xs = qf - s1*a1*np.sin(k1*qf)
ys = a1*np.cos(k1*qf)
slope = np.gradient(ys, dq) / np.gradient(xs, dq)
curv_num = np.gradient(slope, dq) / np.gradient(xs, dq)
inner = slice(2, -2)
e_c = np.abs(etapp_phys(k1*qf, a1, k1, s1)[inner] - curv_num[inner])
gate("A8_curvature_phys_corrected", mixed_ok(e_c, np.abs(curv_num[inner]),
                                             abs_tol=1e-5, rel_tol=1e-4),
     f"closed form vs numeric d2y/dx2: max abs err = {float(e_c.max()):.2e}"
     " (mixed abs<1e-5 / rel<1e-4; FD-limited)")

eps1 = s1*a1*k1
crest_id = etapp_phys(0.0, a1, k1, s1) - (-a1*k1*k1)/(1.0 - eps1)**2
gate("A9_crest_identity", abs(crest_id) < 1e-12,
     f"eta''(crest) - lap_param/areaJac^2 = {crest_id:.2e} (tol 1e-12)")

v90_num = float(np.interp(math.pi/2/k1, qf, curv_num))
v90_new = float(etapp_phys(math.pi/2, a1, k1, s1))
v90_old = float(etapp_old_wrong(math.pi/2, a1, k1, s1))
e_old = float(np.max(np.abs(etapp_old_wrong(k1*qf, a1, k1, s1)[inner] - curv_num[inner])))
ok10 = (abs(v90_new - 0.07752) < 5e-4 and abs(v90_num - v90_new) < 1e-4
        and v90_old < 0.0 and 0.15 < e_old < 0.25)
gate("A10_old_formula_fatal_documented", ok10,
     f"theta=90deg: numeric {v90_num:+.5f}, corrected {v90_new:+.5f}, "
     f"old cos2t formula {v90_old:+.5f} (WRONG SIGN); old max abs err {e_old:.3f} "
     "(verdict: 0.195)")

# A11 refraction constant closures
d_nw = abs(NW - 1.0/1.333)
d_k = abs(K_CODE - K_EXACT)
gate("A11_refraction_constant_closures", d_nw < 1e-6 and d_k < 2e-4,
     f"|NW - 1/1.333| = {d_nw:.2e} (tol 1e-6); |0.25 - (1-1/1.333)| = {d_k:.3e}"
     " (tol 2e-4; absolute gap 1.9e-4 = 0.075% relative)")

# =====================================================================================
# PART B -- ray-cast ground truth vs the three analytic gains
# =====================================================================================
print("== PART B: full-Snell ray density vs the factorized gains ==")

A_B, LAM_B = 0.4, 8.0
K_B = 2*math.pi/LAM_B
NLAM = 8
DOM = NLAM*LAM_B                 # 64 m, periodic
NBIN = 4096
BINW = DOM/NBIN
NRAY = 1 << 22                   # 4194304 rays, uniform quadrature (>= 4e6 per spec)
XB = (np.arange(NBIN) + 0.5)*BINW


def invert_xs(x, a, k, s, iters=14):
    q = np.array(x, float).copy()
    for _ in range(iters):
        q -= (q - s*a*np.sin(k*q) - x) / (1.0 - s*a*k*np.cos(k*q))
    return q


def surf_normal(q, a, k, s):
    th = k*q
    nx = a*k*np.sin(th)
    ny = 1.0 - s*a*k*np.cos(th)
    nn = np.sqrt(nx*nx + ny*ny)
    return nx/nn, ny/nn


def refract2(Ix, Iy, nx, ny, mu):
    ci = -(Ix*nx + Iy*ny)
    ct = np.sqrt(1.0 - mu*mu*(1.0 - ci*ci))
    return mu*Ix + (mu*ci - ct)*nx, mu*Iy + (mu*ci - ct)*ny


def gt_vertical_flat(a, k, s, hbed):
    """uniform physical-x rays, vertical sun, full Snell, flat bed; periodic wrap."""
    x = (np.arange(NRAY) + 0.5) * (DOM/NRAY)
    q = invert_xs(x, a, k, s)
    res = float(np.max(np.abs(q - s*a*np.sin(k*q) - x)))
    assert res < 1e-9, f"Newton inversion residual {res:.1e}"
    ys = a*np.cos(k*q)
    nx, ny = surf_normal(q, a, k, s)
    tx, ty = refract2(np.zeros_like(x), -np.ones_like(x), nx, ny, MU)
    b = np.mod(x + (ys + hbed)*(tx/(-ty)), DOM)
    hist = np.bincount((b/BINW).astype(np.int64), minlength=NBIN).astype(float)
    return hist / hist.mean()


def invert_raymap(xb, a, k, s, hbed, heta=True, iters=40):
    """entry parameter of the ray that reaches bed point xb (derivation s.9: the
    Jacobian is a function of WHERE THE RAY CROSSED THE SURFACE): invert the
    factorized map b(q) = x_s(q) + h_eff K eta_x_phys(q) by damped fixed point --
    monotone while det = 1 + h K eta'' > 0 (true everywhere here)."""
    q = invert_xs(xb, a, k, s)
    for _ in range(iters):
        th = k*q
        slope = (-a*k*np.sin(th))/(1.0 - s*a*k*np.cos(th))
        he = hbed + (a*np.cos(th) if heta else 0.0)
        q = invert_xs(xb - he*K_EXACT*slope, a, k, s)
    return q


def comparators(xb, a, k, s, hbed, sun_run=0.0):
    """G_phys sampled at the entry parameter of the ray reaching each bed bin (the
    factorized map inverted); G_vqview/G_laponly sampled shader-style at
    e = xb - sunRun (Water.hlsl:911-917 does exactly that and no more)."""
    qm = invert_raymap(xb - sun_run, a, k, s, hbed, heta=True)
    th = k*qm
    heff = hbed + a*np.cos(th)
    g_phys_h = 1.0/np.abs(1.0 + heff*K_EXACT*etapp_phys(th, a, k, s))
    qf = invert_raymap(xb - sun_run, a, k, s, hbed, heta=False)
    g_phys_f = 1.0/np.abs(1.0 + hbed*K_EXACT*etapp_phys(k*qf, a, k, s))
    e = np.mod(xb - sun_run, DOM)
    qe = invert_xs(e, a, k, s)
    the = k*qe
    g_phys_entry = 1.0/np.abs(1.0 + (hbed + a*np.cos(the))*K_EXACT
                              * etapp_phys(the, a, k, s))
    aj = np.abs(1.0 - s*a*k*np.cos(the))
    lap = -a*k*k*np.cos(the)
    g_vq = 1.0/(np.maximum(aj, FLOOR_AJ)*np.maximum(1.0 - hbed*K_CODE*lap, FLOOR_CONV))
    g_lap = 1.0/np.maximum(1.0 - hbed*K_CODE*lap, FLOOR_CONV)
    return g_phys_h, g_phys_f, g_vq, g_lap, g_phys_entry


def crest_stat(field):
    """mean over the 8 crest neighbourhoods (+-3 bins) of the local max."""
    vals = []
    for i in range(NLAM):
        c = int(round((i*LAM_B)/BINW)) % NBIN
        idx = (np.arange(c-3, c+4)) % NBIN
        vals.append(field[idx].max())
    return float(np.mean(vals))


H_B = 3.0

# --- B1/B2/B3/B4: s = 1, vertical sun, flat bed 3 m ---------------------------------
gt1 = gt_vertical_flat(A_B, K_B, 1.0, H_B)
gph, gpf, gvq, glp, gpe = comparators(XB, A_B, K_B, 1.0, H_B)
c_ph = pearson(gt1, gph)
c_pf = pearson(gt1, gpf)
c_vq = pearson(gt1, gvq)
c_lp = pearson(gt1, glp)
c_pe = pearson(gt1, gpe)
print(f"      s=1 corr: phys(h+eta, map inverted) {c_ph:+.4f}  phys(fixed h) "
      f"{c_pf:+.4f}  phys(entry-sampled) {c_pe:+.4f}  vqview {c_vq:+.4f}  "
      f"laponly {c_lp:+.4f}")
print(f"      peak gains: GT {gt1.max():.3f}  phys(h+eta) {gph.max():.3f}  "
      f"vqview {gvq.max():.3f}  laponly {glp.max():.3f}"
      f"   ratios GT/pred: {gt1.max()/gph.max():.3f}, {gt1.max()/gvq.max():.3f}")
gate("B1_corr_phys_ge_0.99", c_ph >= 0.99,
     f"corr(GT, G_phys h+eta) = {c_ph:.4f} (corrected gate >= 0.99; fixed-h {c_pf:.4f})")
gate("B4_bookkeeping_ordering", c_ph > c_vq,
     f"corr physical {c_ph:.4f} > corr shipped {c_vq:.4f} "
     f"(laponly {c_lp:+.4f}: wrong-signed curvature alone anticorrelates)")

# analytic pins of the section-7 worked numbers (K = 0.25 as quoted there)
eps_b = A_B*K_B
lap_c = A_B*K_B*K_B
pin = {
    "phys crest": (1.0/(1.0 - H_B*K_CODE*lap_c/(1.0 - eps_b)**2), 1.6486),
    "vq crest": (1.0/((1.0 - eps_b)*(1.0 + H_B*K_CODE*lap_c)), 1.2304),
    "phys trough": (1.0/(1.0 + H_B*K_CODE*lap_c/(1.0 + eps_b)**2), 0.9032),
    "vq trough": (1.0/((1.0 + eps_b)*(1.0 - H_B*K_CODE*lap_c)), 0.9337),
}
ok_pin = all(abs(v - w) < 1.5e-3 for v, w in pin.values())
gate("B2_worked_number_pins", ok_pin,
     "; ".join(f"{n} {v:.4f} (quote {w})" for n, (v, w) in pin.items()))

gt_crest = crest_stat(gt1)
pred_crest = 1.0/abs(1.0 + (H_B + A_B)*K_EXACT*etapp_phys(0.0, A_B, K_B, 1.0))
r = gt_crest/pred_crest
gate("B3_gt_crest_vs_physical", abs(r - 1.0) < 0.10,
     f"GT crest gain {gt_crest:.3f} vs physical h+eta prediction {pred_crest:.3f} "
     f"(ratio {r:.3f}, tol 10%); shipped formula predicts only {pin['vq crest'][0]:.3f}")

# --- B5: s = 0, the naked sign test --------------------------------------------------
gt0 = gt_vertical_flat(A_B, K_B, 0.0, H_B)
gph0, _, gvq0, glp0, _ = comparators(XB, A_B, K_B, 0.0, H_B)
c_ph0 = pearson(gt0, gph0)
c_vq0 = pearson(gt0, gvq0)
gt0_crest = crest_stat(gt0)
pred0 = 1.0/abs(1.0 + (H_B + A_B)*K_EXACT*(-A_B*K_B*K_B))
gate("B5_zero_chop_sign_test", gt0_crest > 1.1 and c_ph0 >= 0.99 and c_vq0 < 0.0,
     f"s=0: GT crest {gt0_crest:.3f} BRIGHT (physical predicts {pred0:.3f}, "
     f"shipped sign predicts {1.0/(1.0 + H_B*K_CODE*A_B*K_B*K_B):.3f} dark); "
     f"corr phys {c_ph0:+.4f} (>=0.99), corr shipped {c_vq0:+.4f} (<0)")

# --- B6: oblique sun 40 deg -- the sun-entry offset demonstration --------------------
# Small amplitude (a = 0.025 m) so the offset geometry is first-order clean; the
# comparator is the exact differential of the full-Snell entry->bed map, evaluated at
# the ENTRY parameter (no offset), including the oblique capture factor dx0/dq.  The
# cross-correlation peak against the bed histogram must sit at
# sunRun = h tan(asin(sin 40 * NW)) to within one bin; omitting the offset costs
# exactly that displacement.
ZEN = math.radians(40.0)
A_S = 0.025
Y0 = 1.0
sun_t = math.asin(math.sin(ZEN)*NW)
SUNRUN = H_B*math.tan(sun_t)


def gt_oblique_flat(a, k, s, hbed, zen):
    x0 = (np.arange(NRAY) + 0.5) * (DOM/NRAY)
    cot = 1.0/math.tan(zen)
    q = x0 + Y0*math.tan(zen)
    for _ in range(30):
        g = a*np.cos(k*q) - Y0 + (q - s*a*np.sin(k*q) - x0)*cot
        gp = -a*k*np.sin(k*q) + cot*(1.0 - s*a*k*np.cos(k*q))
        q -= g/gp
    res = float(np.max(np.abs(a*np.cos(k*q) - Y0
                              + (q - s*a*np.sin(k*q) - x0)*cot)))
    assert res < 1e-9, f"oblique intersection residual {res:.1e}"
    xsq = q - s*a*np.sin(k*q)
    ysq = a*np.cos(k*q)
    nx, ny = surf_normal(q, a, k, s)
    Ix, Iy = math.sin(zen), -math.cos(zen)
    tx, ty = refract2(np.full_like(q, Ix), np.full_like(q, Iy), nx, ny, MU)
    b = np.mod(xsq + (ysq + hbed)*(tx/(-ty)), DOM)
    hist = np.bincount((b/BINW).astype(np.int64), minlength=NBIN).astype(float)
    return hist/hist.mean()


gt_ob = gt_oblique_flat(A_S, K_B, 1.0, H_B, ZEN)

qg = np.linspace(0.0, DOM, 1 << 17, endpoint=False)
xsg = qg - A_S*np.sin(K_B*qg)
ysg = A_S*np.cos(K_B*qg)
nxg, nyg = surf_normal(qg, A_S, K_B, 1.0)
txg, tyg = refract2(np.full_like(qg, math.sin(ZEN)), np.full_like(qg, -math.cos(ZEN)),
                    nxg, nyg, MU)
x0g = xsg - (Y0 - ysg)*math.tan(ZEN)
bg = xsg + (ysg + H_B)*(txg/(-tyg))
dqg = qg[1] - qg[0]
dens_q = np.gradient(x0g, dqg)/np.gradient(bg, dqg)   # exact entry-field density
qb = invert_xs(XB, A_S, K_B, 1.0)                     # entry parameter of each bed bin
F0 = np.interp(np.mod(qb, DOM), qg, dens_q)           # NO offset applied

shifts = np.arange(-40, 200)
cc = np.array([pearson(gt_ob, np.roll(F0, sh)) for sh in shifts])
sh_pk = int(shifts[np.argmax(cc)])
err_run = abs(sh_pk*BINW - SUNRUN)
c_at_peak = float(cc.max())
c_at_zero = float(cc[shifts.tolist().index(0)])
gate("B6_sun_entry_offset", err_run <= BINW and c_at_peak >= 0.99,
     f"cross-corr peak at shift {sh_pk} bins = {sh_pk*BINW:.4f} m vs "
     f"sunRun = h tan(asin(sin40*NW)) = {SUNRUN:.4f} m (err {err_run:.4f} <= "
     f"{BINW:.4f}); corr at peak {c_at_peak:.4f}, corr with offset OMITTED "
     f"{c_at_zero:+.4f}")

# --- B7: sloped bed, vertical sun, s = 1 --------------------------------------------
def bed_depth(x):
    return 0.5 + 5.5*np.minimum(np.asarray(x, float)/60.0, 1.0)


x0 = np.linspace(-2.0, 66.0, NRAY)
q = invert_xs(x0, A_B, K_B, 1.0)
ys = A_B*np.cos(K_B*q)
nx, ny = surf_normal(q, A_B, K_B, 1.0)
tx, ty = refract2(np.zeros_like(q), -np.ones_like(q), nx, ny, MU)
tau = (ys + bed_depth(x0))/(-ty)
for _ in range(12):
    tau = (ys + bed_depth(x0 + tx*tau))/(-ty)
b = x0 + tx*tau
sel = (b >= 0.0) & (b < DOM)
hist = np.bincount((b[sel]/BINW).astype(np.int64), minlength=NBIN).astype(float)
per_bin = NRAY/(68.0/BINW)
gt_sl = hist/per_bin

qb = invert_raymap(XB, A_B, K_B, 1.0, bed_depth(XB), heta=True)
thb = K_B*qb
heff = bed_depth(XB) + A_B*np.cos(thb)
g_sl = 1.0/np.abs(1.0 + heff*K_EXACT*etapp_phys(thb, A_B, K_B, 1.0))
qe = invert_xs(XB, A_B, K_B, 1.0)
aj = np.abs(1.0 - A_B*K_B*np.cos(K_B*qe))
lapb = -A_B*K_B*K_B*np.cos(K_B*qe)
g_sl_vq = 1.0/(np.maximum(aj, FLOOR_AJ)
               * np.maximum(1.0 - bed_depth(XB)*K_CODE*lapb, FLOOR_CONV))
c_sl = pearson(gt_sl, g_sl)
c_sl_vq = pearson(gt_sl, g_sl_vq)
gate("B7_sloped_bed_corr", c_sl >= 0.95,
     f"0.5->6 m bed: corr(GT, G_phys h+eta) = {c_sl:.4f} (gate 0.95); "
     f"shipped form {c_sl_vq:.4f}; deep-end GT peak {gt_sl[XB > 55].max():.2f} vs "
     f"predicted {g_sl[XB > 55].max():.2f}")

# =====================================================================================
# PART C -- identities of the shipped pipeline + the figure
# =====================================================================================
print("== PART C: pipeline identities and the figure ==")


def caustic_factor(area, lap, h, k_lit=K_CODE, sign=+1.0):
    """gain -> washout lerp -> clamp, verbatim Water.hlsl:927,933,934 (sign of the
    convergence term selectable: -1 shipped, +1 physical)."""
    det = np.maximum(area, FLOOR_AJ)*np.maximum(1.0 + sign*h*k_lit*lap, FLOOR_CONV)
    g = 1.0/det
    g = 1.0 + (g - 1.0)*(1.0 - smoothstep(WASH_A, WASH_B, h))
    return np.clip(g, CLAMP_LO, CLAMP_HI)


hgrid = np.linspace(0.0, 30.0, 601)
flat = caustic_factor(np.ones_like(hgrid), np.zeros_like(hgrid), hgrid)
e_flat = float(np.max(np.abs(flat - 1.0)))
gate("C1_flat_surface_gain_unity", e_flat < 1e-12,
     f"a=0: max|gain - 1| over hBed in [0,30] = {e_flat:.2e} (areaJac=1, lap=0, "
     "floors, washout and clamp all pass through)")

aj_t = np.array([0.3, 0.7, 1.0, 1.4])
lp_t = np.array([-0.30, -0.05, 0.10, 0.25])
deep = caustic_factor(aj_t, lp_t, np.full(4, 25.0))
shal = caustic_factor(aj_t, lp_t, np.full(4, 3.0), sign=-1.0)
want_shal = np.clip(1.0/(np.maximum(aj_t, .05)*np.maximum(1.0 - 3.0*K_CODE*lp_t, .05)),
                    CLAMP_LO, CLAMP_HI)
e_deep = float(np.max(np.abs(deep - 1.0)))
e_shal = float(np.max(np.abs(shal - want_shal)))
gate("C2_washout_endpoints", e_deep < 1e-9 and e_shal < 1e-9,
     f"hBed=25: |gain-1| = {e_deep:.2e}; hBed=3: |gain - clamp(1/detJ)| = {e_shal:.2e}")

zen_t = np.radians(np.arange(0.0, 76.0, 5.0))
sw_x = NW*np.sin(zen_t)
ci = np.cos(zen_t)
ctt = np.sqrt(1.0 - NW*NW*(1.0 - ci*ci))
sw_y = NW*(-ci) + (NW*ci - ctt)*1.0        # = -ct
sdown = np.maximum(np.abs(sw_y), 0.15)
run = np.abs(sw_x)/sdown*H_B
want = H_B*np.tan(np.arcsin(np.sin(zen_t)*NW))
e_run = np.abs(run - want)
ok_run = mixed_ok(e_run, np.maximum(want, 1e-30), abs_tol=1e-12, rel_tol=1e-9) \
    and run[0] < 1e-12
gate("C3_sun_run_geometry", ok_run,
     f"zenith 0..75 deg: max|sunRun - h tan(asin(sin z * NW))| = {float(e_run.max()):.2e}"
     " (floor 0.15 inactive over the range; vertical sun -> 0 exactly)")

# ---- the figure: one Gerstner component over a sloped sand bed ----------------------
NPX = 512
L2D = 40.0
a2, lam2 = 0.35, 9.0
k2 = 2*math.pi/lam2
psi2 = math.radians(20.0)
d2 = (math.cos(psi2), math.sin(psi2))
s2 = 1.1
zen2 = math.radians(25.0)

xx = (np.arange(NPX) + 0.5)*(L2D/NPX)
X, Z = np.meshgrid(xx, xx)
Hb = 0.5 + 5.5*(X/L2D)

ci2 = math.cos(zen2)
ct2 = math.sqrt(1.0 - NW*NW*(1.0 - ci2*ci2))
swx = NW*math.sin(zen2)
swy = -ct2
srun_x = swx/abs(swy)
Ex = X - srun_x*Hb
th2 = k2*(d2[0]*Ex + d2[1]*Z)
area2 = np.abs(1.0 - s2*a2*k2*np.cos(th2))
lap2 = -a2*k2*k2*np.cos(th2)
gain_phys = caustic_factor(area2, lap2, Hb, sign=+1.0)   # physical: bright under crests
gain_ship = caustic_factor(area2, lap2, Hb, sign=-1.0)   # shipped sign, for reference

albedo = np.array([0.78, 0.68, 0.47])
E = (SUN_IRR[None, None, :]*FRESNEL_IN
     * np.exp(-KD[None, None, :]*(Hb/abs(swy))[:, :, None])
     * gain_phys[:, :, None])
rgb = albedo[None, None, :]*E
rgb = np.clip(rgb/np.percentile(rgb, 99.5), 0.0, 1.0)**(1.0/2.2)
panel_sand = (rgb*255).astype(np.uint8)

v = np.clip((gain_phys - 1.0)/1.6, -1.0, 1.0)
c_neg = np.array([70, 110, 220], float)      # defocus (gain < 1)
c_mid = np.array([235, 235, 235], float)
c_pos = np.array([236, 150, 42], float)      # focus (gain > 1)
w = np.abs(v)[:, :, None]
col = np.where(v[:, :, None] >= 0,
               c_mid*(1 - w) + c_pos*w,
               c_mid*(1 - w) + c_neg*w)
panel_map = col.astype(np.uint8)

PAD, HDR, FTR = 16, 58, 46
Wim = PAD + NPX + PAD + NPX + PAD
Him = HDR + NPX + FTR
im = Image.new("RGB", (Wim, Him), (13, 15, 20))
im.paste(Image.fromarray(panel_sand), (PAD, HDR))
im.paste(Image.fromarray(panel_map), (PAD + NPX + PAD, HDR))
dr = ImageDraw.Draw(im)
dr.text((PAD, 10), "CAUSTIC JACOBIAN  gain = 1/[areaJac * (1 + hK lap)], "
                   "K = 1-1/n, sampled at entry = bed - sunRun", fill=(232, 232, 240))
dr.text((PAD, 26), "one Gerstner a=0.35m lam=9m chop=1.1, sand bed 0.5->6m (left->"
                   "right), sun 25deg; washout smoothstep(4,20), clamp [0.35,2.6]",
        fill=(160, 165, 185))
dr.text((PAD, 42), f"eta''_phys = -ak^2(cos t - sak)/(1 - sak cos t)^3   [corrected: "
                   f"ray-cast corr {c_ph:.3f} phys vs {c_vq:.3f} shipped; crest gain "
                   f"GT {gt_crest:.2f} = phys {pred_crest:.2f}, shipped 1.23]",
        fill=(208, 176, 96))
dr.text((PAD, HDR + NPX + 6), "sunlit sand bed x Beer-Lambert(0.36,0.105,0.06)/m "
                              "x caustic gain (physical sign: BRIGHT under crests)",
        fill=(200, 200, 210))
dr.text((PAD + NPX + PAD, HDR + NPX + 6),
        "defocus/focus map of the same gain: blue = defocus (<1), amber = focus (>1)",
        fill=(200, 200, 210))
dr.text((PAD, HDR + NPX + 22),
        f"gain range after clamp: [{gain_phys.min():.2f}, {gain_phys.max():.2f}]  "
        f"(shipped-sign field would span [{gain_ship.min():.2f}, "
        f"{gain_ship.max():.2f}] with the pattern contrast understated)",
        fill=(140, 145, 165))
png_path = os.path.join(HERE, "caustic_jacobian.png")
im.save(png_path)
print(f"      wrote {png_path}")

gate("C4_figure_written", os.path.exists(png_path) and gain_phys.max() > 1.2
     and gain_phys.min() < 0.9,
     f"gain field spans [{gain_phys.min():.3f}, {gain_phys.max():.3f}] "
     "(visible focus AND defocus)")

# =====================================================================================
print()
if FAILS:
    print(f"RESULT: FAIL ({len(FAILS)} gate(s)): " + ", ".join(FAILS))
    sys.exit(1)
print("RESULT: ALL GATES PASS")
