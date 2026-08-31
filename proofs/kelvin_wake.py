# ==================================================================================================
#  proofs/kelvin_wake.py - the Kelvin wake, CORRECTED: signed stationary phase, not the fold.
#
#  ALGEBRA FIRST: authoritative small-scale rendering of scratchpad math/wake.derivation.md
#  (post-verdict). The engine port must mirror wake_body() 1:1.
#
#  THE CAMPAIGN'S MOST IMPORTANT FIX (wake.verdicts.md, fatal): the reference shader
#  (vqview Water.hlsl:322 with abs(t) at 418-419) evaluates the folded phase
#      ph_folded = K0*(xi*sec + zeta*sec*|t|)
#  which is the phase at a NON-stationary angle (+|t| where the stationary root is -|t|).
#  Measured here by finite differences, |grad ph_folded|/k runs 1.03-5.18 across the wedge.
#  The CORRECT field evaluates the SIGNED stationary phase
#      ph = K0*sec*(xi - zeta*|t|)          (mirror-symmetric in across already)
#  with slope direction d = grad(ph)/k = -(cos th * fwd + sin th * side * rgt); ph and d flip
#  TOGETHER relative to the reference so eta, slope, laplacian stay one field's derivatives.
#
#  Gates (each printed PASS/FAIL, exit nonzero on any failure):
#    1. quadratic roots satisfy stationarity of phi(theta)          (1e-6)
#    2. wedge discriminant angle == atan(1/(2 sqrt 2)) == arcsin(1/3)  (1e-12; achieves 1e-14)
#    3. NEW - THE GATE THE REFERENCE FAILS: FD |grad ph|/k == 1 within 1e-4 on both branches
#       across the wedge interior (and the folded reference form measurably fails it)
#    4. cusp wavelength (2/3)*2pi U^2/g at the wedge edge, 5%, measured on the corrected field
#    5. transverse wavelength 2pi U^2/g on-axis
#    6. vessel-attitude LSQ plane fit exact on planar input          (1e-12)
#    7. motor-frame invariance: world-frame vs body-frame (PGA motor sandwich) eta   (1e-12)
#
#  Deterministic (no randomness), stdlib + numpy + PIL only. Saves proofs/kelvin_wake.png:
#  corrected wake for U=4.86 m/s, eta grayscale, 19.47-deg wedge lines overlaid, and the
#  bright-arm half-angle fitted numerically.
# ==================================================================================================
import math
import os
import sys

import numpy as np
from PIL import Image, ImageDraw

HERE = os.path.dirname(os.path.abspath(__file__))

G = 9.81
U_REC = 4.86            # recreational class mean SOG (ais_route_wide.json / dx12 main.cpp:443)
U_COM = 3.34            # commercial class
HALFLEN = 7.5
BX = 0.55               # base wake amplitude (main.cpp:58,472)
SAMPLEM = 0.5           # fine mesh: band limit passes for lam > 2.5 m, physics sets the picture
ALPHA = math.atan(1.0 / (2.0 * math.sqrt(2.0)))   # Kelvin half-angle
INV2R2 = 1.0 / (2.0 * math.sqrt(2.0))

FAILS = []


def check(name, ok, detail=""):
    print("[%s] %s%s" % ("PASS" if ok else "FAIL", name, (" -- " + detail) if detail else ""))
    if not ok:
        FAILS.append(name)


def smoothstep(e0, e1, x):
    t = np.clip((x - e0) / (e1 - e0), 0.0, 1.0)
    return t * t * (3.0 - 2.0 * t)


# --------------------------------------------------------------------------------------------------
#  Portable core: the CORRECTED wake field in body coordinates (xi astern, across signed).
#  Every amplitude constant verbatim from Water.hlsl:300-420; the PHASE is the corrected
#  signed-stationary form (derivation 1.5). The engine port mirrors this function.
# --------------------------------------------------------------------------------------------------
def wake_body(xi, across, U, halfLen, Bx, sampleM, branches="both", folded=False):
    """Return (eta, s_fwd, s_rgt, lap, aenv): elevation, slope components along fwd/rgt,
    laplacian, and the amplitude envelope sum_branches(a) (the arm's brightness).

    folded=True reproduces the reference shader's buggy phase (for the refutation print only).
    """
    xi = np.asarray(xi, dtype=np.float64)
    across = np.asarray(across, dtype=np.float64)
    K0 = G / (U * U)
    zeta = np.abs(across)
    side = np.where(across >= 0.0, 1.0, -1.0)

    disc = xi * xi - 8.0 * zeta * zeta
    live = (xi > 0.5) & (disc > 0.0)                    # bow gate + wedge discriminant
    sq = np.sqrt(np.maximum(disc, 0.0))

    dist = np.hypot(xi, across)
    dRel = np.maximum(dist, 1.0) / max(halfLen, 1.0)
    cuspFar = 1.0 + 1.8 * np.clip(1.0 - sq / np.maximum(xi, 1e-3), 0.0, 1.0) ** 3
    cusp = 1.0 + (cuspFar - 1.0) * smoothstep(1.0, 3.5, dRel)
    nearFade = smoothstep(0.5, 2.5, dRel)
    spread = 1.0 / np.sqrt(0.6 + dRel)
    ampCap = 0.22 * halfLen * 0.34
    grow = smoothstep(0.0, 2.0 * halfLen, xi)
    amp = np.minimum(Bx * grow * cusp * spread, ampCap) * nearFade
    live &= amp >= 1e-4                                  # early-out

    degen = zeta < 1e-3                                  # near-track: single branch t = 0
    z4 = np.where(degen, 1.0, 4.0 * zeta)
    t1 = np.where(degen, 0.0, (-xi + sq) / z4)           # transverse (smaller |t|)
    t2 = np.where(degen, 0.0, (-xi - sq) / z4)           # divergent

    eta = np.zeros_like(xi)
    sf = np.zeros_like(xi)
    sr = np.zeros_like(xi)
    lap = np.zeros_like(xi)
    aenv = np.zeros_like(xi)
    todo = []
    if branches in ("both", "transverse"):
        todo.append((t1, 1.0, live))
    if branches in ("both", "divergent"):
        todo.append((t2, 0.85, live & ~degen))
    for t, fac, mask in todo:
        tt = np.abs(t)
        sec2 = 1.0 + t * t
        sec = np.sqrt(sec2)
        k = K0 * sec2
        lam = 2.0 * np.pi / k
        w = smoothstep(2.0, 5.0, lam / max(sampleM, 1e-3))
        aMax = 0.30 / np.maximum(k, 1e-3)
        damp = np.exp(-((k / K0) / 5.0) ** 2)
        a = np.where(mask, np.minimum(amp * fac * w * damp, aMax), 0.0)
        if folded:
            ph = K0 * sec * (xi + zeta * tt)             # the reference's bug (Water.hlsl:322)
            dfw, drg = np.cos(np.arctan(tt)), side * np.sin(np.arctan(tt))
        else:
            ph = K0 * sec * (xi - zeta * tt)             # signed stationary phase (CORRECTED)
            dfw = -1.0 / sec                             # d = grad(ph)/k
            drg = -side * tt / sec
        eta += a * np.cos(ph)
        coef = -a * k * np.sin(ph)
        sf += coef * dfw
        sr += coef * drg
        lap += -a * k * k * np.cos(ph)
        aenv += a
    return eta, sf, sr, lap, aenv


def wake_world(x, z, px, pz, hd, U, halfLen, Bx, sampleM):
    """World-frame evaluation via the Cl(2) fwd/rgt projections (derivation 3.1)."""
    fwdx, fwdz = math.cos(hd), math.sin(hd)
    rgtx, rgtz = -math.sin(hd), math.cos(hd)             # fwd rotated +90 deg in plane
    rx, rz = x - px, z - pz
    xi = -(rx * fwdx + rz * fwdz)
    across = rx * rgtx + rz * rgtz
    return wake_body(xi, across, U, halfLen, Bx, sampleM)


def phase_point(xi, across, U, branch, folded=False):
    """Phase only, root recomputed at the point (for finite-difference gradient tests)."""
    K0 = G / (U * U)
    zeta = abs(across)
    disc = xi * xi - 8.0 * zeta * zeta
    sq = math.sqrt(disc)
    t = (-xi + sq) / (4.0 * zeta) if branch == 1 else (-xi - sq) / (4.0 * zeta)
    tt = abs(t)
    sec = math.sqrt(1.0 + t * t)
    return K0 * sec * (xi + zeta * tt) if folded else K0 * sec * (xi - zeta * tt)


def zero_crossings(s, v):
    """Linear-interpolated zero crossings of v(s)."""
    sgn = np.sign(v)
    idx = np.nonzero(sgn[:-1] * sgn[1:] < 0)[0]
    return s[idx] - v[idx] * (s[idx + 1] - s[idx]) / (v[idx + 1] - v[idx])


# --------------------------------------------------------------------------------------------------
#  PGA motor, ported 1:1 from src/core/Pga.h (dual-quaternion coordinates, same conventions).
# --------------------------------------------------------------------------------------------------
def qmul(a, b):
    return (a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3],
            a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2],
            a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1],
            a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0])


def qrot(r, v):
    rc = (r[0], -r[1], -r[2], -r[3])
    o = qmul(qmul(r, (0.0, v[0], v[1], v[2])), rc)
    return (o[1], o[2], o[3])


class Motor:
    def __init__(self, re=(1.0, 0.0, 0.0, 0.0), du=(0.0, 0.0, 0.0, 0.0)):
        self.re, self.du = tuple(re), tuple(du)

    @staticmethod
    def translation(tx, ty, tz):
        return Motor((1.0, 0.0, 0.0, 0.0), (0.0, 0.5 * tx, 0.5 * ty, 0.5 * tz))

    @staticmethod
    def rotation(p, d, angle):
        h = 0.5 * angle
        sh = math.sin(h)
        re = (math.cos(h), sh * d[0], sh * d[1], sh * d[2])
        rp = qrot(re, p)
        tv = (0.0, 0.5 * (p[0] - rp[0]), 0.5 * (p[1] - rp[1]), 0.5 * (p[2] - rp[2]))
        return Motor(re, qmul(tv, re))

    def __mul__(self, o):
        d1 = qmul(self.re, o.du)
        d2 = qmul(self.du, o.re)
        return Motor(qmul(self.re, o.re), tuple(d1[i] + d2[i] for i in range(4)))

    def inverse(self):                                   # reverse ~M (unit motor); q keeps sign
        return Motor((self.re[0], -self.re[1], -self.re[2], -self.re[3]),
                     (self.du[0], -self.du[1], -self.du[2], -self.du[3]))

    def transform_point(self, v):
        x, y, z = qrot(self.re, v)
        rc = (self.re[0], -self.re[1], -self.re[2], -self.re[3])
        t = qmul(self.du, rc)
        return (x + 2.0 * t[1], y + 2.0 * t[2], z + 2.0 * t[3])


# ==================================================================================================
#  GATE 2 first (it pins the constant everything else uses): the wedge angle.
# ==================================================================================================
print("== wedge discriminant angle ==")
a_atan = math.atan(1.0 / (2.0 * math.sqrt(2.0)))
a_asin = math.asin(1.0 / 3.0)
check("wedge atan(1/(2 sqrt 2)) == arcsin(1/3) (1e-12)", abs(a_atan - a_asin) <= 1e-12,
      "atan=%.17g asin=%.17g diff=%.3g (also within 1e-14: %s)"
      % (a_atan, a_asin, abs(a_atan - a_asin), abs(a_atan - a_asin) <= 1e-14))
check("wedge angle value", abs(a_atan - 0.3398369094541219) <= 1e-14,
      "%.16f rad = %.8f deg" % (a_atan, math.degrees(a_atan)))
zb = INV2R2
check("discriminant boundary xi=1, zeta=1/(2 sqrt 2)", abs(1.0 - 8.0 * zb * zb) <= 1e-15,
      "xi^2-8 zeta^2 = %.3g" % (1.0 - 8.0 * zb * zb))

# ==================================================================================================
#  GATE 1: quadratic roots satisfy stationarity of phi(theta) = K0(xi sec + zeta sec tan).
# ==================================================================================================
print("== stationarity of the quadratic roots (SIGNED) ==")
K0 = G / (U_REC * U_REC)
max_alg = 0.0
max_num = 0.0
for xi in (5.0, 20.0, 50.0, 100.0, 250.0, 400.0):
    for f in (0.1, 0.5, 0.9, 0.999):
        zeta = f * xi * INV2R2
        sq = math.sqrt(xi * xi - 8.0 * zeta * zeta)
        for t in ((-xi + sq) / (4.0 * zeta), (-xi - sq) / (4.0 * zeta)):
            alg = abs(2.0 * zeta * t * t + xi * t + zeta) / max(xi, 1.0)
            max_alg = max(max_alg, alg)
            th = math.atan(t)                             # signed root -> signed angle
            h = 1e-6

            def phi(a):
                return K0 * (xi / math.cos(a) + zeta * math.tan(a) / math.cos(a))

            dnum = abs(phi(th + h) - phi(th - h)) / (2.0 * h) / (K0 * math.hypot(xi, zeta))
            max_num = max(max_num, dnum)
check("algebraic residual |2 zeta t^2 + xi t + zeta| <= 1e-9 * max(xi,1)", max_alg <= 1e-9,
      "max %.3g" % max_alg)
check("numeric d(phi)/d(theta) at signed roots <= 1e-6 * K0 * |r|", max_num <= 1e-6,
      "max %.3g (relative)" % max_num)

print("== Vieta and the cusp ==")
max_v = 0.0
for xi in (5.0, 20.0, 50.0, 100.0, 250.0, 400.0):
    for f in (0.1, 0.5, 0.9, 0.999):
        zeta = f * xi * INV2R2
        sq = math.sqrt(xi * xi - 8.0 * zeta * zeta)
        tp = (-xi + sq) / (4.0 * zeta)
        tm = (-xi - sq) / (4.0 * zeta)
        max_v = max(max_v, abs(tp * tm - 0.5))
check("Vieta t+ * t- == 1/2 (1e-12)", max_v <= 1e-12, "max dev %.3g" % max_v)
t_cusp = -1.0 / math.sqrt(2.0)
xi_c, zeta_c = 100.0, 100.0 * INV2R2                      # on the boundary: double root
t_dbl = -xi_c / (4.0 * zeta_c)
check("boundary double root t = -1/sqrt(2), k/K0 = 3/2 (1e-12)",
      abs(t_dbl - t_cusp) <= 1e-12 and abs((1.0 + t_dbl * t_dbl) - 1.5) <= 1e-12,
      "t=%.15f theta=%.5f deg k/K0=%.15f" % (t_dbl, math.degrees(math.atan(-t_dbl)),
                                             1.0 + t_dbl * t_dbl))

# ==================================================================================================
#  GATE 3 - THE GATE THE REFERENCE FAILS: finite-difference |grad ph| / k == 1 (both branches).
#  h is scaled by 1/(1+t^2) so k*h is branch-uniform (divergent k grows as sec^2).
# ==================================================================================================
print("== phase-gradient consistency (the corrected field passes, the reference fails) ==")
worst_ok = 0.0
worst_folded = 1.0
for xi in (20.0, 50.0, 100.0, 200.0, 350.0):
    for f in (0.05, 0.2, 0.4, 0.6, 0.8, 0.95):
        for sgn in (+1.0, -1.0):
            across = sgn * f * xi * INV2R2
            zeta = abs(across)
            sq = math.sqrt(xi * xi - 8.0 * zeta * zeta)
            for branch in (1, 2):
                t = (-xi + sq) / (4.0 * zeta) if branch == 1 else (-xi - sq) / (4.0 * zeta)
                k = K0 * (1.0 + t * t)
                h = 1e-3 / (1.0 + t * t)
                for fold in (False, True):
                    px = (phase_point(xi + h, across, U_REC, branch, fold)
                          - phase_point(xi - h, across, U_REC, branch, fold)) / (2.0 * h)
                    pz = (phase_point(xi, across + h, U_REC, branch, fold)
                          - phase_point(xi, across - h, U_REC, branch, fold)) / (2.0 * h)
                    ratio = math.hypot(px, pz) / k
                    if fold:
                        worst_folded = max(worst_folded, ratio)
                    else:
                        worst_ok = max(worst_ok, abs(ratio - 1.0))
check("CORRECTED phase: FD |grad ph|/k == 1 within 1e-4, both branches, wedge interior",
      worst_ok <= 1e-4, "max |ratio-1| = %.3g" % worst_ok)
check("REFERENCE folded phase FAILS the same gate (max ratio > 1.5)", worst_folded > 1.5,
      "max |grad ph_folded|/k = %.3f (verdict measured up to 5.18; non-stationary phase)"
      % worst_folded)

# ==================================================================================================
#  Per-class transverse wavelengths (transcription insurance).
# ==================================================================================================
print("== class wavelengths ==")
lams = [2.0 * math.pi * u * u / G for u in (4.86, 4.62, 3.83, 3.34)]
ref = [15.1281, 13.6708, 9.3953, 7.1450]
quoted = [15.1, 13.7, 9.4, 7.1]
check("lambda_t = 2 pi U^2/g per class (1e-3 m)",
      max(abs(a - b) for a, b in zip(lams, ref)) <= 1e-3,
      " ".join("%.4f" % v for v in lams))
check("matches shipped comment values (0.05 m)",
      max(abs(a - b) for a, b in zip(lams, quoted)) <= 0.05, str(quoted))
max_id = 0.0
for xi in (30.0, 90.0, 210.0, 330.0):
    for f in (0.15, 0.45, 0.75, 0.97):
        zeta = f * xi * INV2R2
        sq = math.sqrt(xi * xi - 8.0 * zeta * zeta)
        for t in ((-xi + sq) / (4.0 * zeta), (-xi - sq) / (4.0 * zeta)):
            th = math.atan(abs(t))
            lam_k = 2.0 * math.pi / (K0 * (1.0 + t * t))
            lam_th = (2.0 * math.pi * U_REC * U_REC / G) * math.cos(th) ** 2
            max_id = max(max_id, abs(lam_k - lam_th) / lam_th)
check("branch identity lambda = 2 pi U^2 cos^2(theta)/g (1e-12)", max_id <= 1e-12,
      "max rel dev %.3g" % max_id)

# ==================================================================================================
#  GATE 5: transverse wavelength on-axis, measured on the corrected field by zero crossings.
# ==================================================================================================
print("== transverse wavelength on-axis ==")
for U in (U_REC, U_COM):
    lam_t = 2.0 * math.pi * U * U / G
    s = np.arange(80.0, 440.0, 0.01)
    eta_ax, _, _, _, _ = wake_body(s, np.zeros_like(s), U, HALFLEN, BX, SAMPLEM)
    zc = zero_crossings(s, eta_ax)
    lam_meas = 2.0 * float(np.mean(np.diff(zc)))
    check("on-axis lambda U=%.2f == %.3f m (1%%)" % (U, lam_t),
          abs(lam_meas - lam_t) / lam_t <= 0.01, "measured %.4f m" % lam_meas)

# ==================================================================================================
#  GATE 4: cusp wavelength at the wedge edge, measured on the corrected field as the local
#  wavelength 2 pi / |grad ph|_FD at f = 0.9999 (root recomputed at every FD sample - nothing
#  assumed). Any finite MARCH off the edge under-samples k = 1.5 K0 (the root approaches
#  -1/sqrt(2) with a sqrt singularity in f), so the FD gradient is the correct instrument.
#  On the folded reference field the same measurement collapses to ~0.9 m (printed below).
# ==================================================================================================
print("== cusp wavelength at the wedge edge ==")
lam_cusp = (2.0 / 3.0) * 2.0 * math.pi * U_REC * U_REC / G
vals = []
vals_folded = []
h4 = 2e-4
for xi0 in (150.0, 200.0, 250.0, 300.0, 350.0):
    across0 = 0.9999 * xi0 * INV2R2
    for branch in (1, 2):
        for fold, sink in ((False, vals), (True, vals_folded)):
            px = (phase_point(xi0 + h4, across0, U_REC, branch, fold)
                  - phase_point(xi0 - h4, across0, U_REC, branch, fold)) / (2.0 * h4)
            pz = (phase_point(xi0, across0 + h4, U_REC, branch, fold)
                  - phase_point(xi0, across0 - h4, U_REC, branch, fold)) / (2.0 * h4)
            sink.append(2.0 * math.pi / math.hypot(px, pz))
lam_c_meas = float(np.median(vals))
check("cusp lambda at wedge edge == (2/3)*2 pi U^2/g = %.3f m (5%%)" % lam_cusp,
      abs(lam_c_meas - lam_cusp) / lam_cusp <= 0.05,
      "measured %.3f m (branch spread %.3f..%.3f)" % (lam_c_meas, min(vals), max(vals)))
print("       (folded reference field at the same points: local lambda %.3f m vs the ~0.9 m"
      " the verdict measured slightly further inside -- the fold's gradient diverges at the"
      " edge; either way, nowhere near 10.09 m)" % float(np.median(vals_folded)))

#  Divergent-branch crest spacing along a 15-deg ray (roots are constant along rays from the
#  origin - the quadratic is homogeneous in zeta/xi - so the prediction is a single number).
psi = math.radians(15.0)
tanp = math.tan(psi)
t2r = (-1.0 - math.sqrt(1.0 - 8.0 * tanp * tanp)) / (4.0 * tanp)
sec2r = math.sqrt(1.0 + t2r * t2r)
spacing_pred = math.pi / (K0 * sec2r * abs(math.cos(psi) - abs(t2r) * math.sin(psi)))
s = np.arange(150.0, 420.0, 0.005)
eta_d, _, _, _, _ = wake_body(s * math.cos(psi), s * math.sin(psi), U_REC, HALFLEN, BX,
                              SAMPLEM, branches="divergent")
zc = zero_crossings(s, eta_d)
spacing_meas = float(np.mean(np.diff(zc)))
check("divergent crest spacing along 15-deg ray (2%%)",
      abs(spacing_meas - spacing_pred) / spacing_pred <= 0.02,
      "measured %.4f m, predicted %.4f m" % (spacing_meas, spacing_pred))

# ==================================================================================================
#  GATE 6: the vessel-attitude LSQ plane fit is exact on planar input (Vessel.hlsl 5x3 stencil).
# ==================================================================================================
print("== plane-fit exactness ==")
halfBeam = 0.34 * HALFLEN
xs = np.array([HALFLEN * (i * 0.5 - 1.0) for i in range(5) for _ in range(3)])
zs = np.array([halfBeam * (j - 1.0) for _ in range(5) for j in range(3)])
worst_fit = 0.0
for (al, be, ga_) in [(0.02, -0.05, 0.3), (0.0, 0.0, 0.0), (-0.11, 0.07, -1.2),
                      (1e-3, 2e-3, 0.0), (0.09, 0.0, 2.5)]:
    e = al * xs + be * zs + ga_
    heave = float(np.sum(e)) / 15.0
    slopeX = float(np.sum(xs * e)) / float(np.sum(xs * xs))
    slopeZ = float(np.sum(zs * e)) / float(np.sum(zs * zs))
    worst_fit = max(worst_fit, abs(heave - ga_), abs(slopeX - al), abs(slopeZ - be))
check("5x3 stencil LSQ recovers (alpha, beta, gamma) exactly (1e-12)", worst_fit <= 1e-12,
      "max abs dev %.3g (decoupling: sum x = sum z = sum xz = 0 by symmetry)" % worst_fit)

# ==================================================================================================
#  GATE 7: motor-frame invariance. World-frame Cl(2) projections vs body frame reached through
#  the PGA motor sandwich M^{-1} X ~M^{-1}, M = T(p) * R(origin, +y, -hd) per Pga.h.
# ==================================================================================================
print("== motor-frame invariance ==")
px_, py_, pz_ = qrot((math.cos(math.pi / 4), 0.0, math.sin(math.pi / 4), 0.0), (1.0, 0.0, 0.0))
check("convention pin: right-handed quarter turn about +y maps +x -> -z",
      abs(px_) <= 1e-9 and abs(py_) <= 1e-9 and abs(pz_ + 1.0) <= 1e-9,
      "(%.2g, %.2g, %.2g)" % (px_, py_, pz_))
body_pts = [(30.0, 5.0), (100.0, 30.0), (100.0, -30.0), (250.0, 80.0), (250.0, -80.0),
            (400.0, 120.0), (150.0, 0.0005), (60.0, -15.0), (10.0, 2.0), (-20.0, 4.0),
            (100.0, 80.0)]                                # ahead of bow + outside wedge included
p = (137.5, 0.0, -42.0)
worst_eta = worst_slope = worst_lap = 0.0
for hd in (0.0, 0.7, 2.4, -1.9):
    fwd = (math.cos(hd), math.sin(hd))
    rgt = (-math.sin(hd), math.cos(hd))
    M = Motor.translation(p[0], 0.0, p[2]) * Motor.rotation((0.0, 0.0, 0.0), (0.0, 1.0, 0.0), -hd)
    Minv = M.inverse()
    for (xi_b, ac_b) in body_pts:
        wx = p[0] - xi_b * fwd[0] + ac_b * rgt[0]
        wz = p[2] - xi_b * fwd[1] + ac_b * rgt[1]
        eW, sfW, srW, lW, _ = wake_world(np.float64(wx), np.float64(wz), p[0], p[2], hd,
                                         U_REC, HALFLEN, BX, SAMPLEM)
        bx, _, bz = Minv.transform_point((wx, 0.0, wz))
        eB, sfB, srB, lB, _ = wake_body(np.float64(-bx), np.float64(bz),
                                        U_REC, HALFLEN, BX, SAMPLEM)
        worst_eta = max(worst_eta, abs(float(eW) - float(eB)))
        worst_slope = max(worst_slope, abs(math.hypot(float(sfW), float(srW))
                                           - math.hypot(float(sfB), float(srB))))
        worst_lap = max(worst_lap, abs(float(lW) - float(lB)))
check("eta world == eta body-via-motor (1e-12)", worst_eta <= 1e-12, "max %.3g" % worst_eta)
check("|slope| world == body (1e-12)", worst_slope <= 1e-12, "max %.3g" % worst_slope)
check("laplacian world == body (1e-12)", worst_lap <= 1e-12, "max %.3g" % worst_lap)

# ==================================================================================================
#  The picture: corrected wake for U=4.86 m/s, wedge lines overlaid, bright-arm angle fitted.
# ==================================================================================================
print("== render + bright-arm fit ==")
CELL = 0.5
xv = np.arange(-30.0, 450.0 + CELL / 2, CELL)             # astern axis (xi = x)
yv = np.arange(-170.0, 170.0 + CELL / 2, CELL)            # across
XX, YY = np.meshgrid(xv, yv)
eta, _, _, _, aenv = wake_body(XX, YY, U_REC, HALFLEN, BX, SAMPLEM)


def fit_arm(aenv_grid, xv, yv):
    """Bright-arm half-angle: per astern distance, the across of maximum amplitude ENVELOPE
    (the arm's brightness - |eta| merely oscillates inside it), LSQ line through the origin."""
    up = yv >= 0.0
    yup = yv[up]
    sxy = sxx = 0.0
    for xi_i in np.arange(100.0, 400.0, 2.0):
        col = int(round((xi_i - xv[0]) / (xv[1] - xv[0])))
        zmax = yup[int(np.argmax(aenv_grid[up, col]))]
        sxy += xi_i * zmax
        sxx += xi_i * xi_i
    return math.atan(sxy / sxx)


alpha_fit = fit_arm(aenv, xv, yv)
check("bright-arm half-angle fit == 19.4712 deg (1.0 deg)",
      abs(math.degrees(alpha_fit) - math.degrees(ALPHA)) <= 1.0,
      "fit %.3f deg vs %.4f deg" % (math.degrees(alpha_fit), math.degrees(ALPHA)))

#  1/U^2 character: same wedge, half the wavelength at the commercial speed.
eta_c, _, _, _, aenv_c = wake_body(XX, YY, U_COM, HALFLEN, BX, SAMPLEM)
alpha_fit_c = fit_arm(aenv_c, xv, yv)
check("bright-arm half-angle fit, U=3.34 (speed-independent wedge, 1.0 deg)",
      abs(math.degrees(alpha_fit_c) - math.degrees(ALPHA)) <= 1.0,
      "fit %.3f deg" % math.degrees(alpha_fit_c))

enorm = float(np.percentile(np.abs(eta), 99.5))
gray = np.clip(0.5 + 0.5 * eta / enorm, 0.0, 1.0)
img = Image.fromarray((gray[::-1, :] * 255.0).astype(np.uint8), mode="L").convert("RGB")
draw = ImageDraw.Draw(img)
ny = len(yv)
col0 = int(round((0.0 - xv[0]) / CELL))
row0 = (ny - 1) - int(round((0.0 - yv[0]) / CELL))
x_end = xv[-1]
for sgn in (+1.0, -1.0):
    y_end = sgn * x_end * math.tan(ALPHA)
    row1 = (ny - 1) - int(round((y_end - yv[0]) / CELL))
    draw.line([(col0, row0), (len(xv) - 1, row1)], fill=(255, 96, 32), width=1)
draw.text((6, 4), "Kelvin wake U=4.86 m/s (corrected signed phase)", fill=(255, 255, 96))
draw.text((6, 16), "wedge 19.4712 deg overlaid; bright-arm fit %.3f deg"
          % math.degrees(alpha_fit), fill=(255, 255, 96))
draw.text((6, 28), "lambda_t=%.2f m  lambda_cusp=%.2f m (measured %.2f m)"
          % (2 * math.pi * U_REC ** 2 / G, lam_cusp, lam_c_meas), fill=(255, 255, 96))
png_path = os.path.join(HERE, "kelvin_wake.png")
img.save(png_path)
print("saved %s (%dx%d, eta norm %.3f m)" % (png_path, img.width, img.height, enorm))

# ==================================================================================================
print()
if FAILS:
    print("RESULT: FAIL (%d): %s" % (len(FAILS), "; ".join(FAILS)))
    sys.exit(1)
print("RESULT: ALL PASS")
sys.exit(0)
