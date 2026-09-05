"""imgdiff -- the fidelity gate for renderer optimizations.

Compares two renders (PNG) and reports what an optimization may have changed:

    identical %      pixels whose RGB bytes are equal
    max |d|          largest per-channel byte difference
    n(|d|>1)         pixels with any channel differing by more than one LSB
    mean |d|         mean absolute byte difference over all channels
    SSIM             structural similarity on luma (gaussian 11x11, sigma 1.5)
    hist             differing pixels by their per-pixel max |d|: 1 / 2 / 3 / >3
    argmax           where the worst pixel is (x, y) and the baseline luma there
    by luma          differing pixels binned by the baseline's luma (0-8, 8-32, 32-128, 128-255)

and writes an amplified difference image (|d| x 16, clamped) so the WHERE is visible.

The gate the optimization work uses (docs/NEXT_SESSION.md section 4, priors 9/15):
    'none'    bit-identical: identical 100 %, max |d| 0
    'sub-lsb' max |d| <= 1 and n(|d|>1) == 0 and identical >= 99.9 %
    'noise'   max |d| <= 6, n(|d|>1) <= 64, identical >= 99.99 % -- the measured run-to-run
              floor of the engine itself (runs of one build differ by 3-11 horizon pixels, |d| <= 4)
    anything else is a visible change and must be argued for on its own.

v2 (perf plan step 1) adds what a sub-LSB claim actually needs:

    --hdr A.rgba16f B.rgba16f   two --dump-hdr dumps (RGBA16F rows + .json sidecar) compared IN
              RADIANCE through the tonemap Tonemap.hlsl applies (shoulder at the knee, then the
              1/2.2 gamma, times 255). The number reported per pixel is the exact continuous
              8-bit-scale difference |v(E*b) - v(E*a)| -- the 'lsb_equiv' of the plan; the
              derivative form 255*|dL|*E/2.2*(E*L)^(1/2.2-1) is its small-delta limit and
              diverges at L -> 0, so the exact form is used. A 1-LSB flip in the PNG whose
              lsb_equiv is < 0.5 is a rounding-boundary crossing, not a shifted value.
    --floor A1.png A2.png      an A/A pair of the same binary printed beside the A/B, so the
              verdict is read against the floor the engine has today.
    --json PATH                every number, machine-readable.

Usage:
    py -3 tools/imgdiff.py a.png b.png [--out diff.png] [--floor a1.png a2.png] [--json r.json] [--ignore-rows 425:437]
    py -3 tools/imgdiff.py --pairs dirA dirB [--out-dir diffs] [--json r.json]   # same-named PNGs
    py -3 tools/imgdiff.py --hdr a.rgba16f b.rgba16f [--out diff.png] [--json r.json]

Only numpy and Pillow are needed (scikit-image is not installed on this machine).
"""
import argparse
import json
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


def _luma8(a):
    return 0.299 * a[..., 0] + 0.587 * a[..., 1] + 0.114 * a[..., 2]


def compare(path_a, path_b, out_path=None, with_ssim=True, ignore_rows=None):
    a = np.asarray(Image.open(path_a).convert("RGB"), dtype=np.int16)
    b = np.asarray(Image.open(path_b).convert("RGB"), dtype=np.int16)
    if a.shape != b.shape:
        return {"error": f"shape mismatch {a.shape} vs {b.shape}"}
    d = np.abs(a - b)
    d_full = d
    strip = None
    if ignore_rows:
        # The helm's horizon strip (rows 430-432 at 1600x900) is the one residual outside the
        # residency: step 25 measured an --settle-exact A/A of 65 px there with hash-equal
        # resident sets (probe P16 stays open). The verdict is taken on every other row; the
        # strip's own numbers are reported beside it, never folded into the verdict.
        y0, y1 = ignore_rows
        pm_strip = d[y0:y1].max(axis=2) if y1 > y0 else np.zeros((0, d.shape[1]), dtype=d.dtype)
        strip = {"rows": f"{y0}:{y1}", "n_diff": int((pm_strip > 0).sum()),
                 "n_over1": int((pm_strip > 1).sum()), "max_abs": int(pm_strip.max()) if pm_strip.size else 0}
        d = d.copy()
        d[y0:y1] = 0
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
        "hist": {
            "1": int((per_pixel_max == 1).sum()),
            "2": int((per_pixel_max == 2).sum()),
            "3": int((per_pixel_max == 3).sum()),
            ">3": int((per_pixel_max > 3).sum()),
        },
    }
    la8 = _luma8(a)
    if res["max_abs"] > 0:
        y, x = np.unravel_index(int(np.argmax(per_pixel_max)), per_pixel_max.shape)
        res["argmax"] = {"x": int(x), "y": int(y), "d": int(per_pixel_max[y, x]),
                         "luma": float(la8[y, x])}
        diff_mask = per_pixel_max > 0
        edges = [0, 8, 32, 128, 256]
        by_luma = {}
        for lo, hi in zip(edges[:-1], edges[1:]):
            m = diff_mask & (la8 >= lo) & (la8 < hi)
            by_luma[f"{lo}-{min(hi, 255)}"] = int(m.sum())
        res["by_luma"] = by_luma
    if with_ssim:
        la = la8 / 255.0
        lb = _luma8(b) / 255.0
        res["ssim"] = ssim_luma(la.astype(np.float64), lb.astype(np.float64))
    # MEASURED NOISE FLOOR (2026-09-05, helm still, 240 frames): two runs of the same build and
    # flags differ in 11 of 1.44 M pixels, all on the horizon line, max |d| = 3; the same still
    # rendered by a build that changed no shader (the --gpu-time instrument, flag off) differed
    # in 3 horizon pixels, max |d| = 4, while bird and globe were bit-identical. The engine is not
    # bit-deterministic run to run (residency timing at the far water), so a change inside that
    # band is indistinguishable from re-running the baseline. Anything past it is a real change.
    # THE LEDGER AFTER STEP 1 OF THE PERF PLAN (2026-09-05, ~30 runs, two binaries): the helm at
    # 14:00 stays 5-18 px run to run, settled residency (--settle-sync) or not, so 'noise' is a
    # verdict there; bird and globe are 0 px gates when nothing else disturbs the machine (a
    # first run after a build landed 302 instead of 174 pending globe tiles and differed in 3 %
    # of its pixels); key7km differs by 115-18.7k px between two runs of ONE binary (a tile grid
    # of mip levels); the same helm pose at 19:30 (ebb) differs by 6-9 % of its pixels between
    # two runs, settled residency included, in crest-shaped foam blobs -- the churn atlas keeps
    # the history of when the wave pages landed. So key7km and the ebb helm cannot gate at
    # 'none' or 'noise' until those producers are fixed; the numbers above say which class a
    # difference belongs to, they do not make it acceptable.
    if res["max_abs"] == 0:
        res["verdict"] = "none (bit-identical)"
    elif res["max_abs"] <= 1 and res["identical_pct"] >= 99.9:
        res["verdict"] = "sub-lsb"
    elif res["max_abs"] <= 6 and over1 <= 64 and res["identical_pct"] >= 99.99:
        res["verdict"] = "noise (within the measured run-to-run floor)"
    else:
        res["verdict"] = "VISIBLE"
    if strip is not None:
        res["ignored_strip"] = strip
    if out_path:
        amp = np.clip(d_full * 16, 0, 255).astype(np.uint8)
        Image.fromarray(amp, "RGB").save(out_path)
        res["diff_image"] = out_path
    return res


# ---------------------------------------------------------------------------- HDR (radiance) mode

def load_hdr(path):
    """A --dump-hdr dump: RGBA16F rows without pitch, plus PATH.json {width,height,exposure,knee,gamma}."""
    with open(path + ".json", "r", encoding="utf-8") as f:
        meta = json.load(f)
    w, h = int(meta["width"]), int(meta["height"])
    raw = np.fromfile(path, dtype=np.float16)
    if raw.size != w * h * 4:
        raise ValueError(f"{path}: {raw.size} halves, expected {w * h * 4} for {w}x{h} RGBA16F")
    return raw.astype(np.float32).reshape(h, w, 4)[..., :3], meta


def tonemap_255(x, knee=0.85, gamma=2.2):
    """Tonemap.hlsl on exposed radiance x = E*L, per channel, on the continuous 0..255 scale."""
    ex = np.maximum(x - knee, 0.0)
    y = np.minimum(x, knee) + (1.0 - knee) * (1.0 - np.exp(-ex / (1.0 - knee)))
    return 255.0 * np.power(np.maximum(y, 0.0), 1.0 / gamma)


def compare_hdr(path_a, path_b, out_path=None):
    a, ma = load_hdr(path_a)
    b, mb = load_hdr(path_b)
    if a.shape != b.shape:
        return {"error": f"shape mismatch {a.shape} vs {b.shape}"}
    if abs(float(ma.get("exposure", 1.0)) - float(mb.get("exposure", 1.0))) > 1e-9:
        return {"error": f"exposure differs: {ma.get('exposure')} vs {mb.get('exposure')}"}
    E = float(ma.get("exposure", 1.0))
    knee = float(ma.get("knee", 0.85))
    gamma = float(ma.get("gamma", 2.2))
    # the 8-bit-scale value each dump WOULD produce, before quantization
    va = tonemap_255(np.maximum(a * E, 0.0), knee, gamma)
    vb = tonemap_255(np.maximum(b * E, 0.0), knee, gamma)
    lsb = np.abs(va - vb)                    # per channel
    lsb_px = lsb.max(axis=2)                 # per pixel, worst channel
    dL = np.abs(a - b)
    n_px = lsb_px.size
    res = {
        "mode": "hdr",
        "pixels": n_px,
        "exposure": E,
        "identical_radiance_pct": 100.0 * float((dL.max(axis=2) == 0).sum()) / n_px,
        "max_dL": float(dL.max()),
        "mean_dL": float(dL.mean()),
        "lsb_equiv": {
            "max": float(lsb_px.max()),
            "p99.9": float(np.percentile(lsb_px, 99.9)),
            "n_ge_0.5": int((lsb_px >= 0.5).sum()),
            "n_ge_1": int((lsb_px >= 1.0).sum()),
            "per_channel_max": [float(v) for v in lsb.reshape(-1, 3).max(axis=0)],
        },
        "nonfinite_a": int((~np.isfinite(a)).sum()),
        "nonfinite_b": int((~np.isfinite(b)).sum()),
    }
    if res["max_dL"] > 0:
        y, x = np.unravel_index(int(np.argmax(lsb_px)), lsb_px.shape)
        res["argmax"] = {"x": int(x), "y": int(y), "lsb_equiv": float(lsb_px[y, x]),
                         "dL": float(dL[y, x].max()), "L_a": [float(v) for v in a[y, x]]}
    # The radiance verdict: 'none' when the radiance is identical; 'sub-lsb' when every pixel's
    # continuous 8-bit-scale difference is under half an LSB (so any PNG flip is a rounding
    # crossing); otherwise the difference is a shifted value and needs its own argument.
    if res["max_dL"] == 0:
        res["verdict"] = "none (radiance bit-identical)"
    elif res["lsb_equiv"]["max"] < 0.5:
        res["verdict"] = "sub-lsb (every pixel < 0.5 LSB-equivalent in radiance)"
    else:
        res["verdict"] = "VISIBLE (some pixel moved by >= 0.5 LSB-equivalent in radiance)"
    if out_path:
        amp = np.clip(lsb * 32.0, 0, 255).astype(np.uint8)   # 8 LSB-equiv saturates
        Image.fromarray(amp, "RGB").save(out_path)
        res["diff_image"] = out_path
    return res


# ------------------------------------------------------------------------------------ reporting

def fmt(name, r):
    if "error" in r:
        return f"{name:24s} ERROR {r['error']}"
    if r.get("mode") == "hdr":
        le = r["lsb_equiv"]
        s = (f"{name:24s} radiance identical {r['identical_radiance_pct']:7.3f}%  max|dL| {r['max_dL']:.3g}  "
             f"lsb_equiv max {le['max']:.3f} p99.9 {le['p99.9']:.3f}  n(>=0.5) {le['n_ge_0.5']}  "
             f"n(>=1) {le['n_ge_1']}")
        if "argmax" in r:
            s += f"  worst at ({r['argmax']['x']},{r['argmax']['y']})"
        return s + f"  -> {r['verdict']}"
    s = (f"{name:24s} identical {r['identical_pct']:7.3f}%  max|d| {r['max_abs']:3d}  "
         f"n(|d|>1) {r['n_over1']:8d} ({r['n_over1_pct']:.3f}%)  mean|d| {r['mean_abs']:.4f}")
    if "ssim" in r:
        s += f"  SSIM {r['ssim']:.5f}"
    if "ignored_strip" in r:
        st = r["ignored_strip"]
        s += f"  [rows {st['rows']} ignored: {st['n_diff']} px differ, n(|d|>1) {st['n_over1']}, max|d| {st['max_abs']}]"
    s += f"  -> {r['verdict']}"
    if r["max_abs"] > 0:
        h = r["hist"]
        s += (f"\n{'':24s} hist |d|=1:{h['1']} 2:{h['2']} 3:{h['3']} >3:{h['>3']}  "
              f"worst at ({r['argmax']['x']},{r['argmax']['y']}) |d| {r['argmax']['d']} "
              f"luma {r['argmax']['luma']:.0f}  by luma " +
              " ".join(f"{k}:{v}" for k, v in r["by_luma"].items()))
    return s


def main(argv):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("a", nargs="?")
    ap.add_argument("b", nargs="?")
    ap.add_argument("--out", help="amplified difference image (single pair)")
    ap.add_argument("--pairs", nargs=2, metavar=("DIR_A", "DIR_B"), help="compare same-named PNGs in two directories")
    ap.add_argument("--out-dir", help="where to write difference images in --pairs mode")
    ap.add_argument("--no-ssim", action="store_true", help="skip SSIM (faster)")
    ap.add_argument("--ignore-rows", metavar="Y0:Y1", help="rows [Y0, Y1) left out of the verdict and reported separately (the helm's horizon strip: 425:437)")
    ap.add_argument("--hdr", nargs=2, metavar=("A_RAW", "B_RAW"), help="compare two --dump-hdr dumps in radiance")
    ap.add_argument("--floor", nargs=2, metavar=("A1", "A2"), help="an A/A pair of one binary, printed beside the A/B")
    ap.add_argument("--json", help="write every number to this JSON file")
    args = ap.parse_args(argv)

    ign = None
    if args.ignore_rows:
        y0, y1 = (int(v) for v in args.ignore_rows.split(":"))
        ign = (y0, y1)
    worst = "none"
    results = {}
    if args.hdr:
        r = compare_hdr(args.hdr[0], args.hdr[1], args.out)
        name = os.path.basename(args.hdr[1])
        results[name] = r
        print(fmt(name, r))
        worst = "VISIBLE" if r.get("verdict", "VISIBLE").startswith("VISIBLE") else r.get("verdict")
    elif args.pairs:
        da, db = args.pairs
        names = sorted(n for n in os.listdir(da) if n.lower().endswith(".png") and os.path.exists(os.path.join(db, n)))
        if not names:
            print("no common PNGs")
            return 2
        if args.out_dir:
            os.makedirs(args.out_dir, exist_ok=True)
        for n in names:
            out = os.path.join(args.out_dir, n.replace(".png", "_diff.png")) if args.out_dir else None
            r = compare(os.path.join(da, n), os.path.join(db, n), out, not args.no_ssim, ign)
            results[n] = r
            print(fmt(n, r))
            v = r.get("verdict", "VISIBLE")
            if v.startswith("VISIBLE") or (v == "sub-lsb" and worst == "none"):
                worst = "VISIBLE" if v.startswith("VISIBLE") else "sub-lsb"
    else:
        if not (args.a and args.b):
            ap.print_help()
            return 2
        r = compare(args.a, args.b, args.out, not args.no_ssim, ign)
        results[os.path.basename(args.b)] = r
        print(fmt(os.path.basename(args.b), r))
        worst = "VISIBLE" if r.get("verdict", "VISIBLE").startswith("VISIBLE") else r.get("verdict")
    if args.floor:
        rf = compare(args.floor[0], args.floor[1], None, not args.no_ssim, ign)
        results["floor(A/A)"] = rf
        print(fmt("floor (A/A)", rf))
    if args.json:
        with open(args.json, "w", encoding="utf-8") as f:
            json.dump(results, f, indent=1)
    return 1 if worst == "VISIBLE" else 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
