# proofs/inlet_storm.py -- THE INLET IN A STORM, 2D, from the engine's own fields (M7p).
#
# The physics core is ported from the user's vqview-inlet wave_model.py (dispersion under an
# opposing current with a BLOCKED mask, current-in-flux shoaling, refraction, depth-limited
# breaking) and runs here on the fields the ENGINE exports (--dump-water-state): bed, level,
# solved current, swell shadow. So this figure is the independent 2D answer to "what should
# the inlet look like right now", and the match section holds the bank kernel's fast
# approximation (ShoalFactor * WaveCurrentAmp, Jet.hlsli) to it, shape-normalized.
import json
import math
import struct

import numpy as np
from PIL import Image, ImageDraw

G = 9.81

meta = json.load(open("ws_meta.json"))
nx, ny, cell = meta["nx"], meta["ny"], meta["cell"]

def grid(path):
    a = np.fromfile(path, dtype=np.float32)
    return a.reshape(ny, nx)  # row 0 = SOUTH

bed = grid("ws_bed.f32")
level = grid("ws_level.f32")
u = grid("ws_u.f32")
v = grid("ws_v.f32")
shadow = grid("ws_shadow.f32")
h = np.maximum(level - bed, 0.0)
wet = h > 0.08

# ---- the vqview core (ported; see vqview-inlet/renders/geotiff/water/wave_model.py) ----
def dispersion_k(T, hh, Uopp, nsamp=96):
    sig = 2.0 * np.pi / T
    hh = np.maximum(hh, 0.15)
    kk = np.logspace(-4, 0.7, nsamp)[:, None, None]
    f = (sig + kk * Uopp) ** 2 - G * kk * np.tanh(kk * hh)
    sign_change = (f[:-1] > 0) & (f[1:] <= 0)
    any_root = sign_change.any(axis=0)
    idx = np.argmax(sign_change, axis=0)
    lo = kk[:, 0, 0][idx]
    hi = kk[:, 0, 0][np.minimum(idx + 1, nsamp - 1)]
    for _ in range(48):
        mid = 0.5 * (lo + hi)
        fm = (sig + mid * Uopp) ** 2 - G * mid * np.tanh(mid * hh)
        hi = np.where(fm <= 0, mid, hi)
        lo = np.where(fm > 0, mid, lo)
    return 0.5 * (lo + hi), ~any_root

def component(T, a0, dir_from_deg):
    prop = math.radians((dir_from_deg + 180.0) % 360.0)
    d = (math.sin(prop), math.cos(prop))
    along = u * d[0] + v * d[1]
    Uopp = np.maximum(-along, 0.0)
    c0 = G * T / (2 * np.pi)
    cg0 = 0.5 * c0
    k, blocked = dispersion_k(T, h, Uopp)
    kh = np.clip(k * np.maximum(h, 0.15), 1e-4, 30.0)
    c = (2 * np.pi / k) / T
    n = 0.5 * (1.0 + 2.0 * kh / np.sinh(2.0 * kh))
    cg = n * c
    cg_eff = np.maximum(cg + along, 0.15)
    Ks = np.sqrt(cg0 / cg_eff)
    a_raw = a0 * Ks * shadow
    a_break = 0.39 * np.maximum(h, 0.05)
    broke = a_raw > a_break
    a = np.minimum(a_raw, a_break)
    return a, a_raw, broke, blocked, Ks

Hs, Tp, dirF = meta["hs"], meta["tp"], meta["dirFrom"]
aP, aP_raw, brokeP, blockedP, KsP = component(Tp, Hs / 2.0, dirF)     # the storm swell
T1 = math.sqrt(2 * math.pi * 26.9 / G)                                # cascade band 1
a1, a1_raw, broke1, blocked1, Ks1 = component(T1, 0.25, dirF)

H = 2.0 * aP  # local significant-ish height of the peak component

# ---- the figure ----
SC = 3
im = Image.new("RGB", (nx * SC, ny * SC), (10, 12, 16))
px = im.load()
Hmax = max(np.percentile(H[wet], 99.0), 0.5)
for j in range(ny):
    for i in range(nx):
        y = (ny - 1 - j)  # screen row: north up
        if not wet[j, i]:
            c = (198, 180, 140) if bed[j, i] < level[j, i] + 2.5 else (150, 138, 118)
            for dy in range(SC):
                for dx in range(SC):
                    px[i * SC + dx, y * SC + dy] = c
            continue
        t = min(H[j, i] / Hmax, 1.0)
        # dark navy -> teal -> yellow ramp; breaking paints white
        if t < 0.5:
            f = t / 0.5
            c = (int(8 + 30 * f), int(30 + 120 * f), int(70 + 80 * f))
        else:
            f = (t - 0.5) / 0.5
            c = (int(38 + 200 * f), int(150 + 90 * f), int(150 - 100 * f))
        if brokeP[j, i] or blockedP[j, i]:
            c = (245, 246, 248)
        for dy in range(SC):
            for dx in range(SC):
                px[i * SC + dx, y * SC + dy] = c

dr = ImageDraw.Draw(im)
for j in range(6, ny, 14):        # current arrows
    for i in range(6, nx, 14):
        if not wet[j, i]:
            continue
        sp = math.hypot(u[j, i], v[j, i])
        if sp < 0.15:
            continue
        L = min(10 + 14 * sp, 34)
        x0, y0 = i * SC + 1, (ny - 1 - j) * SC + 1
        x1 = x0 + u[j, i] / sp * L
        y1 = y0 - v[j, i] / sp * L
        dr.line((x0, y0, x1, y1), fill=(255, 90, 60), width=2)
dr.text((10, 8),
        "MERRIMACK INLET IN A STORM  Hs %.1f m Tp %.0f s from %.0f  (engine fields + "
        "vqview physics)" % (Hs, Tp, dirF), fill=(240, 240, 245))
dr.text((10, 26), "color = local wave height (99p %.1f m)  white = breaking/blocked  "
        "arrows = solved current" % Hmax, fill=(180, 185, 195))
im.save("proofs/inlet_storm.png")
print("[2d] proofs/inlet_storm.png  H p50 %.2f p99 %.2f m  breaking %.1f%% of wet" %
      (np.percentile(H[wet], 50), np.percentile(H[wet], 99),
       100.0 * (brokeP & wet).mean() / max(wet.mean(), 1e-9)))

# ---- the match: bank detail plane (band-1 gain) vs this model's band-1 gain ----
fm = json.load(open("fiber_meta.json"))
det = np.fromfile("fiber_detail.f32", dtype=np.float32).reshape(512, 3072)
gain_py = Ks1 * shadow  # shape field; the engine's adds hsScale + its analytic current form
x0b, z0b = meta["x0"], meta["z0"]
rows = []
for ring in range(3):
    org = fm["rings"][ring]["org"]
    tex = fm["rings"][ring]["texelM"]
    e_vals, p_vals = [], []
    for ty in range(0, 512, 2):
        for tx in range(0, 512, 2):
            wx = org[0] + (tx + 0.5) * tex
            wz = org[1] + (ty + 0.5) * tex
            gi = (wx - x0b) / cell - 0.5
            gj = (wz - z0b) / cell - 0.5
            if gi < 1 or gj < 1 or gi >= nx - 2 or gj >= ny - 2:
                continue
            j0, i0 = int(gj), int(gi)
            if not wet[j0, i0]:
                continue
            e = det[ty, ring * 512 + tx]
            if e <= 0.02:
                continue
            e_vals.append(e)
            p_vals.append(gain_py[j0, i0])
    if len(e_vals) < 200:
        continue
    e = np.array(e_vals)
    p = np.array(p_vals)
    e /= np.median(e)          # shape-normalize both (the engine folds in hsScale and its
    p /= np.median(p)          # analytic current form; the SHAPE is the contract)
    r = np.abs(np.log(np.maximum(e, 1e-3) / np.maximum(p, 1e-3)))
    corr = np.corrcoef(e, p)[0, 1]
    rows.append((ring, len(e), np.mean(r), np.percentile(r, 95), corr))
    print("[match] ring %d  n %5d  mean|log ratio| %.3f  p95 %.3f  corr %.3f" % rows[-1])

ok = all(r[4] > 0.5 and r[2] < 0.35 for r in rows) and rows
print("[match] %s -- engine band-1 gain vs vqview-physics gain, shape-normalized" %
      ("AGREES" if ok else "DIVERGES (inspect)"))
