"""imgdiff -- the fidelity gate for renderer optimizations.

Compares two renders (PNG) and reports what an optimization may have changed:

    identical %      pixels whose RGB bytes are equal
    max |d|          largest per-channel byte difference
    n(|d|>1)         pixels with any channel differing by more than one LSB
    mean |d|         mean absolute byte difference over all channels
    SSIM             structural similarity on luma (gaussian 11x11, sigma 1.5)

and writes an amplified difference image (|d| x 16, clamped) so the WHERE is visible.

The gate the optimization work uses (docs/NEXT_SESSION.md section 4, priors 9/15):
    'none'    bit-identical: identical 100 %, max |d| 0
    'sub-lsb' max |d| <= 1 and n(|d|>1) == 0 and identical >= 99.9 %
    'noise'   max |d| <= 3, n(|d|>1) <= 64, identical >= 99.99 % -- the measured run-to-run
              floor of the engine itself (two runs of one build differ by 11 horizon pixels)
    anything else is a visible change and must be argued for on its own.

Usage:
    py -3 tools/imgdiff.py a.png b.png [--out diff.png]
    py -3 tools/imgdiff.py --pairs dirA dirB [--out-dir diffs]   # same-named PNGs

Only numpy and Pillow are needed (scikit-image is not installed on this machine).
"""
import argparse
import os
import sys

import numpy as np
from PIL import Image


def _gaussian_kernel(size=11, sigma=1.5):
    r = np.arange(size) - (size - 1) / 2.0
    k = np.exp(-(r * r) / (2.0 * sigma * sigma))
    return k / k.sum()


def _blur(img, k):
    # separable, 'same' size, edge-replicated -- enough for a gate, exact enough to be stable
    pad = len(k) // 2
    p = np.pad(img, ((pad, pad), (pad, pad)), mode="edge")
    tmp = np.empty_like(p)
    for i in range(p.shape[0]):
        tmp[i] = np.convolve(p[i], k, mode="same")
    out = np.empty_like(tmp)
    for j in range(tmp.shape[1]):
        out[:, j] = np.convolve(tmp[:, j], k, mode="same")
    return out[pad:-pad, pad:-pad]


def ssim_luma(a, b):
    """SSIM on luma in [0,1]; the Wang et al. constants, gaussian window."""
    k = _gaussian_kernel()
    c1 = (0.01 * 1.0) ** 2
    c2 = (0.03 * 1.0) ** 2
    mu_a = _blur(a, k)
    mu_b = _blur(b, k)
    s_aa = _blur(a * a, k) - mu_a * mu_a
    s_bb = _blur(b * b, k) - mu_b * mu_b
    s_ab = _blur(a * b, k) - mu_a * mu_b
    num = (2 * mu_a * mu_b + c1) * (2 * s_ab + c2)
    den = (mu_a * mu_a + mu_b * mu_b + c1) * (s_aa + s_bb + c2)
    return float(np.mean(num / den))


def compare(path_a, path_b, out_path=None, with_ssim=True):
    a = np.asarray(Image.open(path_a).convert("RGB"), dtype=np.int16)
    b = np.asarray(Image.open(path_b).convert("RGB"), dtype=np.int16)
    if a.shape != b.shape:
        return {"error": f"shape mismatch {a.shape} vs {b.shape}"}
    d = np.abs(a - b)
    per_pixel_max = d.max(axis=2)
    n_px = per_pixel_max.size
    identical = int((per_pixel_max == 0).sum())
    over1 = int((per_pixel_max > 1).sum())
    res = {
        "identical_pct": 100.0 * identical / n_px,
        "max_abs": int(d.max()),
        "n_over1": over1,
        "n_over1_pct": 100.0 * over1 / n_px,
        "mean_abs": float(d.mean()),
        "pixels": n_px,
    }
    if with_ssim:
        la = (0.299 * a[..., 0] + 0.587 * a[..., 1] + 0.114 * a[..., 2]) / 255.0
        lb = (0.299 * b[..., 0] + 0.587 * b[..., 1] + 0.114 * b[..., 2]) / 255.0
        res["ssim"] = ssim_luma(la.astype(np.float64), lb.astype(np.float64))
    # MEASURED NOISE FLOOR (2026-09-05, helm still, 240 frames, two runs of the same build and
    # flags): 11 of 1.44 M pixels differ, all on the horizon line, max |d| = 3. The engine is not
    # bit-deterministic run to run (residency timing at the far water), so a change inside that
    # band is indistinguishable from re-running the baseline. Anything past it is a real change.
    if res["max_abs"] == 0:
        res["verdict"] = "none (bit-identical)"
    elif res["max_abs"] <= 1 and res["identical_pct"] >= 99.9:
        res["verdict"] = "sub-lsb"
    elif res["max_abs"] <= 3 and over1 <= 64 and res["identical_pct"] >= 99.99:
        res["verdict"] = "noise (within the measured run-to-run floor)"
    else:
        res["verdict"] = "VISIBLE"
    if out_path:
        amp = np.clip(d * 16, 0, 255).astype(np.uint8)
        Image.fromarray(amp, "RGB").save(out_path)
        res["diff_image"] = out_path
    return res


def fmt(name, r):
    if "error" in r:
        return f"{name:24s} ERROR {r['error']}"
    s = (f"{name:24s} identical {r['identical_pct']:7.3f}%  max|d| {r['max_abs']:3d}  "
         f"n(|d|>1) {r['n_over1']:8d} ({r['n_over1_pct']:.3f}%)  mean|d| {r['mean_abs']:.4f}")
    if "ssim" in r:
        s += f"  SSIM {r['ssim']:.5f}"
    return s + f"  -> {r['verdict']}"


def main(argv):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("a", nargs="?")
    ap.add_argument("b", nargs="?")
    ap.add_argument("--out", help="amplified difference image (single pair)")
    ap.add_argument("--pairs", nargs=2, metavar=("DIR_A", "DIR_B"), help="compare same-named PNGs in two directories")
    ap.add_argument("--out-dir", help="where to write difference images in --pairs mode")
    ap.add_argument("--no-ssim", action="store_true", help="skip SSIM (faster)")
    args = ap.parse_args(argv)

    worst = "none"
    if args.pairs:
        da, db = args.pairs
        names = sorted(n for n in os.listdir(da) if n.lower().endswith(".png") and os.path.exists(os.path.join(db, n)))
        if not names:
            print("no common PNGs")
            return 2
        if args.out_dir:
            os.makedirs(args.out_dir, exist_ok=True)
        for n in names:
            out = os.path.join(args.out_dir, n.replace(".png", "_diff.png")) if args.out_dir else None
            r = compare(os.path.join(da, n), os.path.join(db, n), out, not args.no_ssim)
            print(fmt(n, r))
            v = r.get("verdict", "VISIBLE")
            if v.startswith("VISIBLE") or (v == "sub-lsb" and worst == "none"):
                worst = "VISIBLE" if v.startswith("VISIBLE") else "sub-lsb"
    else:
        if not (args.a and args.b):
            ap.print_help()
            return 2
        r = compare(args.a, args.b, args.out, not args.no_ssim)
        print(fmt(os.path.basename(args.b), r))
        worst = "VISIBLE" if r.get("verdict", "VISIBLE").startswith("VISIBLE") else r.get("verdict")
    return 1 if worst == "VISIBLE" else 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
