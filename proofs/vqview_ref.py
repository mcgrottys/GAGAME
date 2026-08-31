# ==================================================================================================
#  vqview_ref.py - loader for the vqview-inlet reference solution (the twin-test ground truth).
#
#  vqview-inlet (C:\vqview-inlet\vqview-inlet) baked a per-cell SOLVED wave field for the
#  Merrimack inlet: 16 spectral components, each with per-cell amplitude a (shoaling, refraction,
#  blocking, total-Hs limiter applied), wavenumber k (current-Doppler finite-depth dispersion),
#  and integrated spatial phase phi (grad phi = k*dhat), quantized to RGBA8 with per-set-of-4
#  normalizers.  This module decodes that bake back to float arrays so GAGAME's own solver
#  (proofs/wave_field.py and later src/sim/WaveField.cpp) can be held to it numerically.
#
#  Everything is returned in the vqview crop frame: row 0 = SOUTH after the flips below
#  (their PNGs are row-0-north; we flip to match wave_model.py's array convention, which
#  operates on the cropped bathy directly - bathy_comp2.npy row 0 is NORTH, and the exporter
#  never flips, so PNG row r == array row r == northmost first.  We therefore keep row 0 =
#  NORTH throughout and document it, rather than flipping: the reference model and the
#  reference textures share the same row order and the comparison is row-for-row.)
#
#  Frames/units ledger (for the GA AST when this crosses into the engine):
#    grid: (ny, nx) row-major, row 0 = NORTH, col 0 = WEST; cell 1.5 m
#    bathy z: metres relative to MLLW (their composite; +1.00 m empirical offset was fitted
#             by their ortho_waterline audit - carry as-is, the twin test is internally
#             consistent because model and textures share the same z)
#    a: metres; k: rad/m; phase: unit spinor (cos phi, sin phi); sigma: rad/s
#    dir: (east, north) unit propagation vectors (CONSTANT per component; the spatial
#         bending lives in the phase field)
# ==================================================================================================
import json
import os

import numpy as np
from PIL import Image

VQVIEW = os.environ.get("VQVIEW", r"C:\vqview-inlet\vqview-inlet")
GEO = os.path.join(VQVIEW, "renders", "geotiff")
WATER = os.path.join(GEO, "water")
TEXTURES = os.path.join(WATER, "textures_wide")

# The wide crop that produced textures_wide (README regen commands): full-array (x0,x1,y0,y1).
CROP = (120, 1840, 70, 760)
CELL = 1.5

# bathy_comp2.npy georef (ortho_waterline.py:37): full array (1106, 1851), row 0 = north.
FULL_LON0, FULL_LAT1 = -70.8320, 42.8250
FULL_LON1, FULL_LAT0 = -70.7980, 42.8100


def load_bathy(crop=CROP):
    """Cropped bathy, metres MLLW, shape (ny, nx) = (690, 1720) for the wide crop, row 0 north."""
    z = np.load(os.path.join(GEO, "bathy_comp2.npy")).astype(np.float64)
    x0, x1, y0, y1 = crop
    return z[y0:y1, x0:x1]


def load_current(sign=1.0):
    """Solved ebb current (u east, v north, m/s) on the wide crop grid; solved at tide +0.30."""
    c = np.load(os.path.join(WATER, "current_wide_low.npz"))
    return (c["u"].astype(np.float64) * sign, c["v"].astype(np.float64) * sign,
            float(c["tide"]))


def load_uniforms(tag="chop_big"):
    return json.load(open(os.path.join(TEXTURES, "uniforms16_%s.json" % tag)))


def _png(name):
    return np.asarray(Image.open(os.path.join(TEXTURES, name)), dtype=np.float64)


def load_solution(tag="chop_big"):
    """Decode the 19-texture bake to float arrays.

    Returns dict:
      a     (16, ny, nx) metres          k     (16, ny, nx) rad/m
      cos   (16, ny, nx)                 sin   (16, ny, nx)   (unit spinor, |.| ~ 1 mod 8-bit)
      env_rms, env_exc, env_sum (ny, nx)
      elev  (ny, nx) metres MLLW (from elev_cm_u16)
      rock  (ny, nx) 0..1
      sigma (16,) rad/s;  dir (16, 2) (east, north);  meta = uniforms dict
    Quantization floor: 8-bit over per-set ranges - expect ~kMax[s]/255 and aMax[s]/255 noise.
    """
    u = load_uniforms(tag)
    n = int(u["n_comp"])
    nsets = (n + 3) // 4
    kmax = [float(v) for v in u["kMax"]]
    amax = [float(v) for v in u["aMax"]]

    a = []
    k = []
    for s in range(nsets):
        K = _png("waveK%d_%s_rgba8.png" % (s, tag)) / 255.0
        A = _png("waveA%d_%s_rgba8.png" % (s, tag)) / 255.0
        for ci in range(4):
            if s * 4 + ci >= n:
                break
            k.append(K[:, :, ci] * kmax[s])
            a.append(A[:, :, ci] * amax[s])

    cos = []
    sin = []
    for p in range(n // 2):
        cs = _png("wavePhaseCS%d_%s_rgba8.png" % (p, tag)) / 255.0 * 2.0 - 1.0
        cos.append(cs[:, :, 0]); sin.append(cs[:, :, 1])
        cos.append(cs[:, :, 2]); sin.append(cs[:, :, 3])

    env = _png("waveEnv16_%s_rgb8.png" % tag) / 255.0
    elev16 = np.asarray(Image.open(os.path.join(TEXTURES, "elev_cm_u16.png")),
                        dtype=np.float64)
    rock = _png("rock_mask_u8.png") / 255.0

    return dict(
        a=np.stack(a), k=np.stack(k), cos=np.stack(cos), sin=np.stack(sin),
        env_rms=env[:, :, 0] * float(u["envMax"]),
        env_exc=env[:, :, 1] * float(u["excMax"]),
        env_sum=env[:, :, 2] * float(u["sumMax"]),
        elev=elev16 / 100.0 - float(u["elev_bias_m"]),
        rock=rock,
        sigma=np.array(u["sigma"], dtype=np.float64),
        dir=np.array(u["dir"], dtype=np.float64),
        meta=u,
    )


if __name__ == "__main__":
    z = load_bathy()
    sol = load_solution()
    u, v, ctide = load_current()
    print("bathy crop", z.shape, "range %.2f..%.2f m MLLW" % (z.min(), z.max()))
    print("current tide %+.2f  p99 speed %.2f m/s" % (ctide,
          np.percentile(np.hypot(u, v), 99)))
    print("solution: %d comps  a max %.3f  k max %.3f  spinor |.| mean %.4f" % (
        sol["a"].shape[0], sol["a"].max(), sol["k"].max(),
        float(np.mean(np.hypot(sol["cos"][0], sol["sin"][0])))))
    print("env rms/exc/sum max: %.3f %.3f %.3f" % (
        sol["env_rms"].max(), sol["env_exc"].max(), sol["env_sum"].max()))
    print("elev range %.2f..%.2f  rock mean %.3f" % (
        sol["elev"].min(), sol["elev"].max(), sol["rock"].mean()))
