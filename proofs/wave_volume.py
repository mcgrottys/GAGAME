# proofs/wave_volume.py -- THE WATER AS A VOLUME ON THE RINGS (the rolling-wave work, 2026-10-18).
#
# The bank's disp plane is a sheet of LABELS: texel l holds the particle P(l) = l + D(l). Over a
# ground point the sheet is single-valued only while det(1 + J) > 0; where a crest overturns the
# vertical through the ground crosses the surface three times. WaterBank.hlsl's CsBankScatter folds
# every label's particle into the EULERIAN column it lands in -- three crossings a column (face,
# lip underside, lip top): the water's run-length voxel along the vertical.
#
# This script, on the planes the engine dumped (--dump-fibers: fiber_disp.f32, fiber_param.f32,
# fiber_column.f32, fiber_meta.json), does four things with numpy and no engine code:
#   1. the TWIN: rasterizes the label cells itself (the kernel's law) and holds the kernel's column
#      plane to it texel for texel, the lip's coverage count included;
#   2. the FOLD: where a crest overturns, how many labels fold (det < 0), how many Eulerian columns
#      hold a tube, the lip's thickness, and the HOLES left (none once the cells are rasterized:
#      a stretched cell's image still tiles the plane);
#   3. the VOLUME on the sparse structure: the occupancy chi of the water in a 3D lattice built
#      from ring 0's columns, folded 2x2x2 (the coverage mean), against the same lattice built
#      from ring 1's columns (the coarser ring's own description) -- the fold identity that lets a
#      coarse voxel be the mean of fine ones; and what a full 3D tile volume would cost beside
#      the three numbers a column;
#   4. the FIGURE: the ring-0 row with the most tubes -- the Lagrangian sheet as a curve, the
#      column's three crossings over it (proofs/wave_volume.png).
import json
import math
import sys

import numpy as np

try:
    from PIL import Image, ImageDraw
except ImportError:  # the figure is optional
    Image = None

RING_W = 512
N_RINGS = 6
meta = json.load(open("fiber_meta.json"))
rings = meta["rings"]


def plane4(path):
    a = np.fromfile(path, dtype=np.float32)
    return a.reshape(RING_W, N_RINGS * RING_W, 4)


disp = plane4("fiber_disp.f32")      # (dx, dy, dz, foam), rings left to right
param = plane4("fiber_param.f32")    # (level - waterline, sigma2, u, v)
col = np.fromfile("fiber_column.f32", dtype=np.float32).reshape(4, RING_W, N_RINGS * RING_W)
# the column plane's bands: 0 face, 1 lip top (inverted), 2 lip underside (upright max), 3 the lip's coverage count; NaN = absent


def ring(a, m):
    return a[..., m * RING_W:(m + 1) * RING_W] if a.ndim == 2 else a[:, m * RING_W:(m + 1) * RING_W]


def scatter(dx, dy, dz, lvl, texel, n=None):
    """The kernel's law (CsBankScatter), in numpy: THE MESH'S TRIANGLES. Each cell is the four
    labels at a texel and its +x, +z, +x+z neighbours at their own particles and heights, cut into
    the two triangles GlobeMesh.hlsl draws; every Eulerian texel whose centre lies in a triangle
    takes its barycentric height; a triangle whose orientation flipped is the fold (the inverted
    branch, counted on a 4x4 sub-grid for the coverage). Returns face (upright MIN), lipBot
    (upright MAX, the tube's ceiling), lipTop (inverted MAX, the crest), NaN absent, the coverage
    count, and the folded-label mask (a label is folded when its cell's first triangle is)."""
    H, W = dy.shape
    Px = (np.arange(W)[None, :] + 0.5) * texel + dx
    Pz = (np.arange(H)[:, None] + 0.5) * texel + dz
    Hh = lvl + dy
    face = np.full((H, W), np.inf, np.float32)
    lipBot = np.full((H, W), -np.inf, np.float32)
    lipTop = np.full((H, W), -np.inf, np.float32)
    count = np.zeros((H, W), np.int64)
    inverted = np.zeros((H, W), bool)
    # cells with all four corners inside the ring
    c00 = (slice(0, H - 1), slice(0, W - 1)); c10 = (slice(0, H - 1), slice(1, W))
    c01 = (slice(1, H), slice(0, W - 1)); c11 = (slice(1, H), slice(1, W))
    corners = {0: c00, 1: c10, 2: c01, 3: c11}
    for ti, (ia, ib, ic) in enumerate(((0, 1, 2), (1, 3, 2))):
        Ax, Az, Ah = Px[corners[ia]], Pz[corners[ia]], Hh[corners[ia]]
        Bx, Bz, Bh = Px[corners[ib]], Pz[corners[ib]], Hh[corners[ib]]
        Cx, Cz, Ch = Px[corners[ic]], Pz[corners[ic]], Hh[corners[ic]]
        area = (Bx - Ax) * (Cz - Az) - (Cx - Ax) * (Bz - Az)
        lox, hix = np.minimum(Ax, np.minimum(Bx, Cx)), np.maximum(Ax, np.maximum(Bx, Cx))
        loz, hiz = np.minimum(Az, np.minimum(Bz, Cz)), np.maximum(Az, np.maximum(Bz, Cz))
        ok = (hix - lox <= 8 * texel) & (hiz - loz <= 8 * texel) & (np.abs(area) >= 1e-6 * texel * texel)
        inv = area < 0
        if ti == 0:
            inverted[c00] = inv & ok
        q0x = np.floor(lox / texel).astype(np.int64); q1x = np.floor(hix / texel).astype(np.int64)
        q0z = np.floor(loz / texel).astype(np.int64); q1z = np.floor(hiz / texel).astype(np.int64)
        for S, mask in ((1, ok & ~inv), (4, ok & inv)):
            src = np.nonzero(mask)
            if src[0].size == 0:
                continue
            iar = 1.0 / area[src]
            A_x, A_z, A_h = Ax[src], Az[src], Ah[src]
            B_x, B_z, B_h = Bx[src], Bz[src], Bh[src]
            C_x, C_z, C_h = Cx[src], Cz[src], Ch[src]
            for oz_t in range(9):
                for ox_t in range(9):
                    qx = q0x[src] + ox_t
                    qz = q0z[src] + oz_t
                    inb = (qx <= q1x[src]) & (qz <= q1z[src]) & (qx >= 0) & (qx < W) & (qz >= 0) & (qz < H)
                    if not inb.any():
                        continue
                    sel = np.nonzero(inb)[0]
                    qxs, qzs = qx[sel], qz[sel]
                    hits = np.zeros(sel.shape, np.int64)
                    vmax = np.full(sel.shape, -np.inf)
                    vmin = np.full(sel.shape, np.inf)
                    for sj in range(S):
                        for si in range(S):
                            px = (qxs + (si + 0.5) / S) * texel
                            pz = (qzs + (sj + 0.5) / S) * texel
                            wA = ((B_x[sel] - px) * (C_z[sel] - pz) - (C_x[sel] - px) * (B_z[sel] - pz)) * iar[sel]
                            wB = ((C_x[sel] - px) * (A_z[sel] - pz) - (A_x[sel] - px) * (C_z[sel] - pz)) * iar[sel]
                            wC = 1.0 - wA - wB
                            inside = (wA >= -1e-4) & (wB >= -1e-4) & (wC >= -1e-4)
                            h = wA * A_h[sel] + wB * B_h[sel] + wC * C_h[sel]
                            hits += inside
                            vmax = np.where(inside, np.maximum(vmax, h), vmax)
                            vmin = np.where(inside, np.minimum(vmin, h), vmin)
                    got = hits > 0
                    lin = (qzs * W + qxs)[got]
                    if S > 1:
                        np.maximum.at(lipTop.reshape(-1), lin, vmax[got])
                        np.add.at(count.reshape(-1), lin, hits[got])
                    else:
                        np.minimum.at(face.reshape(-1), lin, vmin[got])
                        np.maximum.at(lipBot.reshape(-1), lin, vmax[got])
    face[~np.isfinite(face)] = np.nan
    lipBot[~np.isfinite(lipBot)] = np.nan
    lipTop[~np.isfinite(lipTop)] = np.nan
    areaJac = np.ones((H, W))
    areaJac[inverted] = -1.0
    return face, lipBot, lipTop, inverted, areaJac, count


print("== THE COLUMN PLANE vs ITS NUMPY TWIN, per ring ==")
print("ring texel   wet     fold%  tubes%  lipTh p50/p95   holes   twin: face|d|<=1cm  lipTop  lipBot  coverage=")
best = (None, -1, None)
per_ring = []
for m in range(N_RINGS):
    texel = rings[m]["texelM"]
    d = ring(disp, m)
    p = ring(param, m)
    dx, dy, dz = d[..., 0], d[..., 1], d[..., 2]
    lvl = p[..., 0]
    wet = (np.abs(dy) > 1e-6) | (np.abs(dx) > 1e-6)    # a NULL tile reads zero in every channel
    nwet = int(wet.sum())
    if nwet == 0:
        print(f"{m:4d} {texel:5.1f}   (no wet texel)")
        continue
    face, lipBot, lipTop, inverted, areaJac, count = scatter(dx, dy, dz, lvl, texel)
    kc = col[3, :, m * RING_W:(m + 1) * RING_W]
    kf, kt, kb = col[0, :, m * RING_W:(m + 1) * RING_W], col[1, :, m * RING_W:(m + 1) * RING_W], col[2, :, m * RING_W:(m + 1) * RING_W]
    tube = np.isfinite(lipTop) & np.isfinite(face) & (lipTop > lipBot + 1e-3) & (lipBot > face + 1e-3)
    th = (lipTop - lipBot)[tube]
    holes = int((wet & ~np.isfinite(face)).sum())
    cov_eq = float((np.abs(kc - count) <= 0).mean())
    # the twin: where both present, |d| <= 1 mm (the fixed point's step); presence must agree
    def twin(a, b):
        both = np.isfinite(a) & np.isfinite(b)
        agree = (np.abs(a - b) <= 1.1e-2) & both   # the word's centimetre
        pres = (np.isfinite(a) == np.isfinite(b))
        return agree.sum() / max(both.sum(), 1), pres.mean()
    tf, tb, tt = twin(face, kf), twin(lipBot, kb), twin(lipTop, kt)
    fold_pct = 100.0 * (inverted & wet).sum() / nwet
    tube_pct = 100.0 * tube.sum() / nwet
    p50 = float(np.percentile(th, 50)) if th.size else 0.0
    p95 = float(np.percentile(th, 95)) if th.size else 0.0
    print(f"{m:4d} {texel:5.1f} {nwet:7d} {fold_pct:6.2f} {tube_pct:6.2f}   {p50:4.2f}/{p95:4.2f} m   "
          f"{holes:6d}   {100 * tf[0]:5.1f}% ({100 * tf[1]:5.1f}% pres) "
          f"{100 * tt[0]:5.1f}% {100 * tb[0]:5.1f}% ({100 * tb[1]:5.1f}% pres) {100 * cov_eq:5.1f}%")
    per_ring.append((m, texel, nwet, fold_pct, tube_pct, p50, p95, holes, tf, tb, tt, cov_eq))
    if tube.sum() > best[1]:
        rows = tube.sum(axis=1)
        best = (m, int(tube.sum()), int(np.argmax(rows)))

# ---- 3. THE VOLUME ON THE SPARSE STRUCTURE: occupancy chi and its fold ------------------------
print("\n== THE VOLUME: chi from ring 0's columns, folded 2x2x2, against ring 1's own ==")
m0, m1 = 0, 1
t0 = rings[m0]["texelM"]
VZ = t0 / 4.0            # the vertical voxel: a quarter of the texel (chi is a coverage, so any grain folds)
ZLO, ZHI = -6.0, 6.0     # the band about the waterline the storm's crests reach
nz = int(round((ZHI - ZLO) / VZ))


def chi_of(face, lipBot, lipTop, vz, nz, zlo):
    """Coverage of each voxel of a column: water below the face, air, then the lip's interval."""
    H, W = face.shape
    zc = zlo + (np.arange(nz) + 0.5) * vz
    f = np.where(np.isfinite(face), face, -1e9)[..., None]
    b = np.where(np.isfinite(lipBot), lipBot, 1e9)[..., None]
    t = np.where(np.isfinite(lipTop), lipTop, -1e9)[..., None]
    z = zc[None, None, :]
    # coverage of [z - vz/2, z + vz/2] by (-inf, f] union [b, t], exact for intervals
    lo, hi = z - vz / 2, z + vz / 2
    cov1 = np.clip((f - lo) / vz, 0, 1)
    cov2 = np.clip((np.minimum(t, hi) - np.maximum(b, lo)) / vz, 0, 1)
    return np.clip(cov1 + cov2, 0, 1).astype(np.float32)


f0, b0, tt0, _, _, _ = scatter(ring(disp, m0)[..., 0], ring(disp, m0)[..., 1], ring(disp, m0)[..., 2], ring(param, m0)[..., 0], t0)
f1, b1, tt1, _, _, _ = scatter(ring(disp, m1)[..., 0], ring(disp, m1)[..., 1], ring(disp, m1)[..., 2], ring(param, m1)[..., 0], rings[m1]["texelM"])
chi0 = chi_of(f0, b0, tt0, VZ, nz, ZLO)                       # (512, 512, nz) at ring 0's grain
# fold 2x2x2: the coverage MEAN (alpha is fiber; the fold answers)
chi0f = chi0.reshape(256, 2, 256, 2, nz // 2, 2).mean(axis=(1, 3, 5))
# ring 1's own columns at the SAME lattice: ring 0 covers the central quarter of ring 1
ox = int(round((rings[m0]["org"][0] - rings[m1]["org"][0]) / rings[m1]["texelM"]))
oz = int(round((rings[m0]["org"][1] - rings[m1]["org"][1]) / rings[m1]["texelM"]))
chi1 = chi_of(f1[oz:oz + 256, ox:ox + 256], b1[oz:oz + 256, ox:ox + 256], tt1[oz:oz + 256, ox:ox + 256], 2 * VZ, nz // 2, ZLO)
wet0 = np.isfinite(f0).reshape(256, 2, 256, 2).all(axis=(1, 3))
wet1 = np.isfinite(f1[oz:oz + 256, ox:ox + 256])
both = wet0 & wet1
dchi = (chi0f - chi1)[both]
colsum0 = (chi0f.sum(axis=-1) * 2 * VZ + ZLO)[both]     # the column's water height from the fold
colsum1 = (chi1.sum(axis=-1) * 2 * VZ + ZLO)[both]
print(f"lattice: {t0:.1f} m texels, {VZ:.2f} m voxels, band [{ZLO:+.0f}, {ZHI:+.0f}] m: {nz} voxels a column; "
      f"folded to {2 * t0:.1f} x {2 * VZ:.2f} m ({nz // 2} a column)")
print(f"columns compared {both.sum()} | chi fold vs ring 1's own: mean |d| {np.abs(dchi).mean():.4f}, "
      f"p95 {np.percentile(np.abs(dchi), 95):.4f}, max {np.abs(dchi).max():.3f} "
      f"| column water height (the heightfield as the fold's integral): rms {np.sqrt(((colsum0 - colsum1) ** 2).mean()):.3f} m, "
      f"p95 {np.percentile(np.abs(colsum0 - colsum1), 95):.3f} m")
# the volume's cost, as 3D tiles, against the column plane
occ = (chi0 > 0) & (chi0 < 1)      # voxels the SURFACE passes through (a 3D tile must hold these)
tiles3d = 0
TX, TY, TZ = 32, 32, 16            # an R32 3D tile (64 KB)
for tz in range(0, nz, TZ):
    for ty in range(0, 512, TY):
        for tx in range(0, 512, TX):
            if occ[ty:ty + TY, tx:tx + TX, tz:tz + TZ].any():
                tiles3d += 1
print(f"a 3D tile volume of ring 0 at {VZ:.2f} m voxels would hold {tiles3d} tiles of 64 KB ({tiles3d * 64 / 1024:.1f} MB) "
      f"for the surface band; the column plane holds ring 0 in {3 * 16} tiles ({3 * 16 * 64 / 1024:.1f} MB) "
      f"-- {tiles3d / (3 * 16):.1f}x")

# ---- 4. THE FIGURE: the row with the most tubes --------------------------------------------
if Image is not None and best[0] is not None and best[1] > 0:
    m, _, row = best
    texel = rings[m]["texelM"]
    d = ring(disp, m)[row]
    lvl = ring(param, m)[row, :, 0]
    face, lipBot, lipTop, inverted, areaJac, _ = scatter(ring(disp, m)[..., 0], ring(disp, m)[..., 1], ring(disp, m)[..., 2], ring(param, m)[..., 0], texel)
    tube = np.isfinite(lipBot[row]) & np.isfinite(lipTop[row])
    cols = np.where(tube)[0]
    c0, c1 = max(cols.min() - 40, 0), min(cols.max() + 40, RING_W - 1)
    W, H = 1600, 500
    img = Image.new("RGB", (W, H), (18, 20, 28))
    dr = ImageDraw.Draw(img)
    xs = (np.arange(c0, c1 + 1) + 0.5) * texel
    zlo, zhi = -4.0, 5.0
    def sx(x): return int((x - xs[0]) / (xs[-1] - xs[0]) * (W - 40) + 20)
    def sy(z): return int(H - 20 - (z - zlo) / (zhi - zlo) * (H - 40))
    dr.line([(20, sy(0)), (W - 20, sy(0))], fill=(70, 70, 90))
    # the Lagrangian sheet: label + lateral, height = level + dy
    pts = [(sx((i + 0.5) * texel + d[i, 0]), sy(lvl[i] + d[i, 1])) for i in range(c0, c1 + 1)]
    dr.line(pts, fill=(120, 200, 255), width=2)
    # the column's crossings
    for i in range(c0, c1 + 1):
        x = sx((i + 0.5) * texel)
        if np.isfinite(face[row, i]): dr.point((x, sy(face[row, i])), fill=(255, 230, 90))
        if np.isfinite(lipBot[row, i]): dr.point((x, sy(lipBot[row, i])), fill=(255, 90, 90))
        if np.isfinite(lipTop[row, i]): dr.point((x, sy(lipTop[row, i])), fill=(120, 255, 140))
    dr.text((24, 6), f"ring {m} ({texel:.1f} m texels) row {row}: the Lagrangian sheet (blue) and the column plane's "
                     f"crossings -- face (yellow), lip underside (red), lip top (green); {int(tube.sum())} tube columns in the row",
            fill=(230, 230, 230))
    out = sys.argv[1] if len(sys.argv) > 1 else "wave_volume.png"
    img.save(out)
    print(f"\nfigure: {out} (ring {m}, row {row}, columns {c0}..{c1})")
else:
    print("\nno tube in any ring: no figure (the sheet never folds in this sea)")
