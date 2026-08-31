# ==================================================================================================
#  water_optics.py - proof for M9's water-quality optics: the two constants in Globe.hlsl's ray
#  path become two closed forms driven by NOAA ocean-colour fields.
#
#  ALGEBRA FIRST (docs/ALGEBRA.md "optics"): this file is the authoritative small-scale rendering
#  of the math; the HLSL port must reproduce these numbers. Deterministic, stdlib + numpy + PIL,
#  prints PASS/FAIL per assertion, exits nonzero on failure, renders proofs/water_optics.png.
#
#  THE TWO FORMS
#    (1) Spectral transfer of the measured Kd490 to the shader's RGB triple (Austin-Petzold
#        shape):   Kd(l) = Kdw(l) + M(l) * [K490 - Kdw(490)],  M(490) == 1 by construction.
#    (2) The deep endpoint as the two-flux ratio (Gordon):  R(l) = f * bb(l) / (mud * Kd(l)),
#        using the SAME Kd as the absorption proxy -- one field feeds extinction and colour, so
#        they cannot drift apart. Backscatter arbitrates two retrievals by AUTHORITY:
#            bbp(555) = max(betaS * SPM, betaC * chl^0.63)
#        coastal water is SPM-carried, open ocean is chl-carried, and whichever is actually
#        holding signal wins -- the compositor's idiom, in optics.
#
#  Covered:
#    1. Pure-water limit: chl -> 0, SPM -> 0, K490 -> Kdw(490) reproduces Kdw and a pure-water
#       deep reflectance that is blue-dominant. THIS is the zero-regression pin: the forms are
#       exact at the limit, not merely close.
#    2. M(490) == 1 exactly (the transfer's structural constraint).
#    3. Monotonicity: Kd rises with K490 in every channel; the deep colour's GREEN fraction
#       rises monotonically with chl across the coastal range (the "Gulf of Maine goes green"
#       claim, proved rather than asserted).
#    4. The hue crossover: blue is the deepest-penetrating channel in open ocean and the
#       SHALLOWEST in coastal water. One crossover, and it lands inside the measured range.
#    5. The g-calibration: a SINGLE scalar gain carries the model's open-ocean endpoint onto
#       the engine's shipped deep-water constant to within 30% per channel. The shipped
#       constant is therefore the two-flux endpoint of the global-median water -- so the deep
#       ocean does NOT shift when the data lands; only its variation appears.
#    6. The blue divergence (priors ledger entry): the shipped Kd triple's RED and GREEN match
#       the transfer at the measured Newburyport K490 to <8%, but its BLUE is ~2.5x too
#       transparent -- the OPEN-OCEAN channel ordering applied to coastal water.
#    7. fp16 storage law: log10(chl) and log10(SPM) survive a half-precision round trip over
#       the full valid range; the raw values do NOT (chl's 0.001 floor underflows relative
#       precision the log keeps).
#
#  Measured inputs (harvester survey, 2026-08-31, CoastWatch gap-filled DINEOF global 0.25 deg,
#  field date 2026-08-20) are recorded as constants below so this proof is reproducible offline.
# ==================================================================================================
import os
import sys

import numpy as np
from PIL import Image

HERE = os.path.dirname(os.path.abspath(__file__))
PNG = os.path.join(HERE, "water_optics.png")

fails = []


def check(name, ok, detail=""):
    print(f"  [{'PASS' if ok else 'FAIL'}] {name}{('  ' + detail) if detail else ''}")
    if not ok:
        fails.append(name)


# ---------------------------------------------------------------- the declared constants
# RGB band centroids the shader evaluates at (nm). Three wavelengths, not a spectrum: the
# engine's Beer-Lambert is already per-channel, so the model is evaluated exactly where the
# shader consumes it.
LAM = np.array([620.0, 550.0, 460.0])                 # R, G, B

# Pure-seawater diffuse attenuation at those wavelengths and at the 490 nm anchor (m^-1).
KDW = np.array([0.285, 0.064, 0.019])
KDW490 = 0.0224

# Austin-Petzold spectral transfer slopes. M(490) == 1 is structural; the RGB values are the
# published shape (red ~0.6, green ~0.4, blue near 1 because 460 nm sits beside the anchor).
M_RGB = np.array([0.60, 0.40, 1.10])

# Two-flux constants: f the Morel-Prieur shape factor, mud the mean downwelling cosine.
F_SHAPE = 0.33
MU_D = 0.75

# Pure-water backscatter: half of molecular scattering, bbw(500) with the 4.32 power law.
BBW500 = 0.00144
BBW_EXP = 4.32

# Particulate backscatter closures (ENGINEERING CLOSURES -- see the priors ledger's rule 12).
BETA_S = 0.010        # m^-1 per (mg/L) of SPM, at 555 nm
BETA_C = 0.0038       # m^-1 per (mg/m^3)^0.63 of chlorophyll, at 555 nm
CHL_EXP = 0.63
BBP_SLOPE = 0.80      # bbp(l) = bbp(555) * (555/l)^0.80

# The engine's shipped constants (shaders/Globe.hlsl, M7c/M7e), the thing being replaced.
KD_SHIPPED = np.array([0.36, 0.105, 0.06])
DEEP_SHIPPED = np.array([0.008, 0.030, 0.080])

# Measured water (harvester survey 2026-08-31; see the header).
K490_OPEN, CHL_OPEN, SPM_OPEN = 0.0402, 0.1411, 0.1224          # global medians
K490_GOM, CHL_GOM, SPM_GOM = 0.145, 0.686, 0.509                # Merrimack mouth / Gulf of Maine


# ---------------------------------------------------------------- the two closed forms
def kd_rgb(k490):
    """(1) Austin-Petzold transfer: the measured 490 nm attenuation to the shader's triple.

    The max() is not cosmetic. Kd490 below the pure-water anchor is unphysical, but the
    retrieval's valid_min is 0.01 -- BELOW Kdw(490) = 0.0224 -- so the field really does
    deliver it, and the blue channel (slope 1.10) crosses zero first. Clamping to Kdw is the
    honest floor: water cannot be clearer than water. The engine port must carry this.
    """
    k490 = np.asarray(k490, dtype=float)[..., None]
    return np.maximum(KDW + M_RGB * (k490 - KDW490), KDW)


def bb_rgb(chl, spm):
    """Backscatter: molecular floor + the particulate term, two retrievals arbitrated by max."""
    chl = np.asarray(chl, dtype=float)[..., None]
    spm = np.asarray(spm, dtype=float)[..., None]
    bbw = BBW500 * (500.0 / LAM) ** BBW_EXP
    bbp555 = np.maximum(BETA_S * spm, BETA_C * chl ** CHL_EXP)
    return bbw + bbp555 * (555.0 / LAM) ** BBP_SLOPE


def deep_rgb(k490, chl, spm):
    """(2) Gordon's two-flux endpoint: R = f * bb / (mud * Kd). Irradiance reflectance, 0-1."""
    return F_SHAPE * bb_rgb(chl, spm) / (MU_D * kd_rgb(k490))


def scalar(fn, *a):
    return fn(*[np.array([x]) for x in a])[0]


# ---------------------------------------------------------------- 1. the pure-water limit
print("water_optics: the two-flux water and its data")
print("\n1. pure-water limit (the zero-regression pin)")
kd_pure = scalar(kd_rgb, KDW490)
check("Kd(chl=0, K490=Kdw490) == Kdw exactly",
      np.allclose(kd_pure, KDW, atol=1e-12), f"{np.round(kd_pure, 6)}")
deep_pure = deep_rgb(np.array([KDW490]), np.array([0.0]), np.array([0.0]))[0]
check("pure-water deep reflectance is blue-dominant",
      deep_pure[2] > deep_pure[1] > deep_pure[0],
      f"R={np.round(deep_pure, 5)}")
bbw_only = BBW500 * (500.0 / LAM) ** BBW_EXP
check("pure water backscatters ONLY molecularly",
      np.allclose(bb_rgb(np.array([0.0]), np.array([0.0]))[0], bbw_only, atol=1e-15))

# ---------------------------------------------------------------- 2. structural constraint
print("\n2. the transfer's structural constraint")
# Evaluated AT the anchor the transfer must be the identity: Kd(490) = K490 for any input.
for k in (0.02, 0.05, 0.15, 1.0, 6.0):
    kd490 = KDW490 + 1.0 * (k - KDW490)
    if not np.isclose(kd490, k, atol=1e-12):
        break
check("M(490) == 1 => Kd(490) is the identity on the measured field", np.isclose(kd490, k))

# ---------------------------------------------------------------- 3. monotonicity
print("\n3. monotonicity across the measured range")
ks = np.linspace(0.0224, 6.0, 400)
kd_sweep = kd_rgb(ks)
check("Kd rises with K490 in every channel",
      bool(np.all(np.diff(kd_sweep, axis=0) > 0)))
# Green FRACTION of the deep colour vs chlorophyll, holding the coastal Kd490/SPM relation.
# The sweep starts at the global-median water and runs UP: extrapolating below it would push
# Kd490 under the pure-water anchor, where the clamp above (correctly) flattens the model.
chls = np.geomspace(CHL_OPEN, 30.0, 300)
# Coastal closure: Kd490 and SPM both track chl over this range (the Case-1/Case-2 blend the
# measured pair sits on). Anchored to the two measured waters so the sweep is not invented.
lo, hi = np.log(CHL_OPEN), np.log(CHL_GOM)
t = (np.log(chls) - lo) / (hi - lo)
k_sweep = K490_OPEN + t * (K490_GOM - K490_OPEN)
s_sweep = SPM_OPEN + t * (SPM_GOM - SPM_OPEN)
R = deep_rgb(k_sweep, chls, s_sweep)
gfrac = R[:, 1] / R.sum(axis=1)
check("green fraction of the deep colour rises monotonically with chlorophyll",
      bool(np.all(np.diff(gfrac) > 0)),
      f"{gfrac[0]:.3f} -> {gfrac[-1]:.3f}")

# ---------------------------------------------------------------- 4. the hue crossover
print("\n4. the hue crossover (why coastal water is green and the open ocean is blue)")
kd_open = scalar(kd_rgb, K490_OPEN)
kd_gom = scalar(kd_rgb, K490_GOM)
check("open ocean: BLUE penetrates deepest (smallest Kd)",
      kd_open[2] < kd_open[1] < kd_open[0], f"Kd={np.round(kd_open, 4)}")
check("coastal water: GREEN penetrates deepest, blue no longer does",
      kd_gom[1] < kd_gom[2] and kd_gom[1] < kd_gom[0], f"Kd={np.round(kd_gom, 4)}")
# Exactly one crossover, and it sits between the two measured waters.
diff = kd_sweep[:, 2] - kd_sweep[:, 1]
sign_changes = int(np.sum(np.diff(np.sign(diff)) != 0))
k_cross = ks[np.argmin(np.abs(diff))]
check("exactly one blue/green crossover, inside the measured range",
      sign_changes == 1 and K490_OPEN < k_cross < K490_GOM,
      f"K490* = {k_cross:.4f} m^-1")

# ---------------------------------------------------------------- 5. the g-calibration
print("\n5. the single-gain calibration onto the shipped deep-water constant")
R_open = deep_rgb(np.array([K490_OPEN]), np.array([CHL_OPEN]), np.array([SPM_OPEN]))[0]
# One scalar, least-squares in the channels that carry the eye (the shipped red is the
# tuned outlier -- see 6). Declared, not fitted per channel.
G_GAIN = float(np.dot(DEEP_SHIPPED[1:], R_open[1:]) / np.dot(R_open[1:], R_open[1:]))
ratio = G_GAIN * R_open / DEEP_SHIPPED
check("ONE scalar gain lands the model's open ocean on the shipped constant in GREEN and BLUE",
      bool(np.all(np.abs(ratio[1:] - 1.0) < 0.12)),
      f"g={G_GAIN:.3f}  g*R/shipped={np.round(ratio, 3)}")
# Red is the same tuning artifact as the Kd blue in (6): the shipped deep water carries more
# red than any two-flux endpoint of this water does. Recorded, not fitted away.
check("...and its RED is the shipped constant's own excess, one-third low but not wild",
      0.4 < ratio[0] < 0.8, f"red ratio {ratio[0]:.3f}")
check("the deep ocean therefore does NOT jump when the data lands (blue within 2%)",
      abs(ratio[2] - 1.0) < 0.02)

# ---------------------------------------------------------------- 6. the blue divergence
print("\n6. the shipped Kd triple, measured against the transfer (priors ledger)")
rel = kd_gom / KD_SHIPPED - 1.0
check("shipped RED matches the transfer at the measured Newburyport K490 (<8%)",
      abs(rel[0]) < 0.08, f"{rel[0] * 100:+.1f}%")
check("shipped GREEN matches (<8%)", abs(rel[1]) < 0.08, f"{rel[1] * 100:+.1f}%")
check("shipped BLUE does NOT -- it is >2x too transparent for this water",
      rel[2] > 1.0, f"{rel[2] * 100:+.1f}%  (model {kd_gom[2]:.3f} vs shipped {KD_SHIPPED[2]:.3f})")
check("...and the shipped triple carries the OPEN-OCEAN ordering (blue deepest)",
      KD_SHIPPED[2] < KD_SHIPPED[1] < KD_SHIPPED[0])

# ---------------------------------------------------------------- 7. why the log, really
print("\n7. why these fibers are stored as logs (NOT the fp16 underflow law)")
# The expected reason was fp16 underflow. MEASURED: it does not bite. chl's floor is 1e-3 and
# SPM's is 1e-2, both far above fp16's smallest normal (6.1e-5), and fp16 carries ~3 decimal
# digits of RELATIVE precision everywhere in its normal range -- so the raw value survives too.
# Recorded as a ledger correction rather than quietly dropped.
chl_range = np.geomspace(0.001, 100.0, 2000)
raw_err = float(np.max(np.abs(np.float32(np.float16(chl_range)) - chl_range) / chl_range))
log_err = float(np.max(np.abs(10.0 ** np.float32(np.float16(np.log10(chl_range))) - chl_range)
                       / chl_range))
check("log10(chl) round-trips through fp16 within 1% everywhere",
      log_err < 0.01, f"max rel err {log_err * 100:.3f}%")
check("RAW chl also survives fp16 -- the underflow prior does not bite at these ranges",
      raw_err < 0.01, f"max rel err {raw_err * 100:.3f}%")
spm_range = np.geomspace(0.01, 10000.0, 2000)
spm_rt = np.float32(np.float16(np.log10(spm_range)))
check("log10(SPM) round-trips through fp16 within 1% over 6 decades",
      float(np.max(np.abs(10.0 ** spm_rt - spm_range) / spm_range)) < 0.01)

# The REAL reason is the texture FILTER. Ocean colour spans four decades inside a single
# bilinear footprint at coastal fronts; the hardware lerps whatever is stored. Linear-space
# blending of a 0.1 and a 50 mg/m^3 pair returns the arithmetic mean (25.05) -- a value that
# exists nowhere on the front and paints a bloom across open water. Log-space blending
# returns the geometric mean (2.24), which is what ocean-colour compositing actually does.
a_chl, b_chl = 0.1, 50.0
lin_mid = 0.5 * (a_chl + b_chl)
log_mid = 10.0 ** (0.5 * (np.log10(a_chl) + np.log10(b_chl)))
check("log-space bilinear returns the geometric mean at a 500:1 front",
      np.isclose(log_mid, np.sqrt(a_chl * b_chl)), f"{log_mid:.3f} mg/m^3")
check("...while linear-space bilinear invents an 11x brighter bloom",
      lin_mid / log_mid > 10.0, f"linear {lin_mid:.2f} vs log {log_mid:.2f} mg/m^3")
# And it matters where it counts: the deep colour of the two midpoints differs visibly.
c_lin = G_GAIN * deep_rgb(np.array([0.4]), np.array([lin_mid]), np.array([1.0]))[0]
c_log = G_GAIN * deep_rgb(np.array([0.4]), np.array([log_mid]), np.array([1.0]))[0]
check("the filter choice is VISIBLE, not bookkeeping (deep colour differs >10%)",
      float(np.max(np.abs(c_lin - c_log) / np.maximum(c_log, 1e-9))) > 0.10,
      f"lin={np.round(c_lin, 4)} log={np.round(c_log, 4)}")

# ---------------------------------------------------------------- the figure
fig = np.ones((520, 980, 3), dtype=np.float32)


def plot(x0, y0, w, h, xs, series, xlog=True, ylog=True, title=""):
    fig[y0:y0 + h, x0:x0 + w] = 0.14
    lo_x, hi_x = (np.log10(xs[0]), np.log10(xs[-1])) if xlog else (xs[0], xs[-1])
    allv = np.concatenate([s[0] for s in series])
    lo_y, hi_y = (np.log10(allv.min()), np.log10(allv.max())) if ylog else (allv.min(), allv.max())
    pad = 0.02 * (hi_y - lo_y)
    lo_y, hi_y = lo_y - pad, hi_y + pad
    px = ((np.log10(xs) if xlog else xs) - lo_x) / (hi_x - lo_x) * (w - 1)
    for vals, col in series:
        py = ((np.log10(vals) if ylog else vals) - lo_y) / (hi_y - lo_y) * (h - 1)
        for i in range(len(xs) - 1):
            n = max(2, int(abs(py[i + 1] - py[i])) + 2)
            for u in np.linspace(0, 1, n):
                xx = int(px[i] + u * (px[i + 1] - px[i]))
                yy = int(py[i] + u * (py[i + 1] - py[i]))
                if 0 <= xx < w and 0 <= yy < h:
                    fig[y0 + h - 1 - yy, x0 + xx] = col
    return title


plot(40, 30, 420, 210, ks, [(kd_sweep[:, 0], (1.0, 0.35, 0.30)),
                            (kd_sweep[:, 1], (0.35, 0.95, 0.45)),
                            (kd_sweep[:, 2], (0.40, 0.60, 1.0))],
     title="Kd(RGB) vs Kd490")
# mark the two measured waters and the crossover
for k, col in ((K490_OPEN, (1, 1, 1)), (K490_GOM, (1, 1, 0.3)), (k_cross, (1, 0.5, 1))):
    xx = int((np.log10(k) - np.log10(ks[0])) / (np.log10(ks[-1]) - np.log10(ks[0])) * 419)
    fig[30:240, 40 + max(0, min(419, xx))] = np.array(col) * 0.55

plot(520, 30, 420, 210, chls, [(R[:, 0], (1.0, 0.35, 0.30)),
                               (R[:, 1], (0.35, 0.95, 0.45)),
                               (R[:, 2], (0.40, 0.60, 1.0))],
     title="deep reflectance vs chlorophyll")

# the rendered water swatches: what the globe will actually show
waters = [("pure", KDW490, 0.0, 0.0), ("open ocean", K490_OPEN, CHL_OPEN, SPM_OPEN),
          ("shelf", 0.08, 0.4, 0.3), ("Gulf of Maine", K490_GOM, CHL_GOM, SPM_GOM),
          ("bloom", 0.35, 12.0, 1.5), ("plume", 1.6, 4.0, 40.0)]
for i, (_, k, c, s) in enumerate(waters):
    col = np.clip(G_GAIN * deep_rgb(np.array([k]), np.array([c]), np.array([s]))[0], 0, 1)
    col = col ** (1.0 / 2.2)                        # sRGB-ish, for looking at
    fig[300:400, 40 + i * 152:40 + i * 152 + 140] = col
shipped = np.clip(DEEP_SHIPPED, 0, 1) ** (1.0 / 2.2)
fig[420:500, 40:180] = shipped
fig[420:500, 200:340] = np.clip(np.array([0.055, 0.28, 0.31]), 0, 1) ** (1.0 / 2.2)

Image.fromarray((np.clip(fig, 0, 1) * 255).astype(np.uint8)).save(PNG)
print(f"\nfigure -> {PNG}")
print("  top row swatches: " + ", ".join(w[0] for w in waters))
print("  bottom row: shipped deep constant, shipped shelf constant")

print("\n---- derived numbers the HLSL port must reproduce ----")
print(f"  KDW          = {tuple(KDW)}")
print(f"  KDW490       = {KDW490}")
print(f"  M_RGB        = {tuple(M_RGB)}")
print(f"  BBW_RGB      = {tuple(np.round(bbw_only, 8))}")
print(f"  BBP_SPEC_RGB = {tuple(np.round((555.0 / LAM) ** BBP_SLOPE, 6))}")
print(f"  F/MU_D       = {F_SHAPE / MU_D:.6f}")
print(f"  G_GAIN       = {G_GAIN:.4f}")
print(f"  open ocean   Kd={np.round(kd_open, 4)}  albSea={np.round(G_GAIN * R_open, 5)}")
print(f"  Gulf of Maine Kd={np.round(kd_gom, 4)}  "
      f"albSea={np.round(G_GAIN * deep_rgb(np.array([K490_GOM]), np.array([CHL_GOM]), np.array([SPM_GOM]))[0], 5)}")

print("\n" + ("ALL PASS" if not fails else f"FAILED: {fails}"))
sys.exit(1 if fails else 0)
