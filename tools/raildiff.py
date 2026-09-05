"""raildiff -- the fade instrument: a rail recording against a baseline recording, per frame.

Three series through ffmpeg (the essentials build on this machine; ffprobe beside it):

    ssim / psnr        per-frame SSIM (luma, chroma, all) and PSNR of NEW against BASE
    yavg               per-frame mean luma of each recording (signalstats YAVG), compared per
                       second the way the Sep 2 sanity check did (matched within 0.4/255)
    tblend             per-frame mean of |frame_f - frame_{f-1}| over the WATER ROWS of each
                       recording (tblend=all_mode=difference, then signalstats YAVG): the
                       frame-to-frame change series. A landing or a bucket roll that pops is a
                       spike in it; a fade is not. A frame is flagged when NEW's delta exceeds
                       3x its own rolling median (the pop-in track's gate: no frame > 3x the
                       settled median across a landing) and BASE has no spike of its own there.

Video is lossy (H.264): this is a smoke test for continuity and for WHEN something moved --
never the pixel gate (tools/imgdiff.py on --dump stills is).

Usage:
    py -3 tools/raildiff.py BASE.mp4 NEW.mp4 [--out-dir DIR] [--rows 0.45:1.0] [--stills N]
                            [--ssim-floor 0.95] [--spike 3.0]

    --rows y0:y1   the water rows as fractions of the height (default 0.45:1.0, below the
                   horizon at the helm); the tblend series is cropped to them
    --stills N     write amplified |BASE - NEW| stills for the N worst-SSIM frames into DIR
Writes DIR/series.csv (frame, t, ssim_all, ssim_y, psnr, yavg_base, yavg_new, tb_base, tb_new)
and prints the summary. Exit 1 when a spike is flagged or the SSIM floor is broken.
"""
import argparse
import csv
import os
import re
import shutil
import subprocess
import sys


def run(cmd):
    p = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True,
                       encoding="utf-8", errors="replace")
    if p.returncode != 0:
        sys.stderr.write(" ".join(cmd) + "\n" + p.stderr[-2000:] + "\n")
        raise SystemExit(f"ffmpeg failed ({p.returncode})")
    return p


def probe(path):
    out = run(["ffprobe", "-v", "error", "-select_streams", "v:0", "-show_entries",
               "stream=width,height,nb_frames,r_frame_rate", "-of", "csv=p=0", path]).stdout
    parts = out.strip().split(",")
    w, h = int(parts[0]), int(parts[1])
    fr = parts[2]
    num, den = (fr.split("/") + ["1"])[:2]
    fps = float(num) / float(den)
    nb = int(parts[3]) if len(parts) > 3 and parts[3].isdigit() else 0
    return w, h, fps, nb


def ssim_psnr(base, new, out_dir):
    ssim_log = os.path.join(out_dir, "ssim.log")
    psnr_log = os.path.join(out_dir, "psnr.log")
    run(["ffmpeg", "-v", "error", "-y", "-i", base, "-i", new, "-lavfi",
         f"[0:v][1:v]ssim=stats_file={_ff(ssim_log)};[0:v][1:v]psnr=stats_file={_ff(psnr_log)}",
         "-f", "null", "-"])
    ssim_all, ssim_y, psnr = {}, {}, {}
    rx = re.compile(r"n:(\d+)\s+Y:([\d.]+)\s+U:([\d.]+)\s+V:([\d.]+)\s+All:([\d.]+)")
    with open(ssim_log, "r", encoding="utf-8", errors="replace") as f:
        for line in f:
            m = rx.search(line)
            if m:
                n = int(m.group(1)) - 1
                ssim_all[n] = float(m.group(5))
                ssim_y[n] = float(m.group(2))
    rp = re.compile(r"n:(\d+)\s+mse_avg:([\d.a-z]+)\s+.*?psnr_avg:([\d.a-z]+)")
    with open(psnr_log, "r", encoding="utf-8", errors="replace") as f:
        for line in f:
            m = rp.search(line)
            if m:
                v = m.group(3)
                psnr[int(m.group(1)) - 1] = float("inf") if v == "inf" else float(v)
    return ssim_all, ssim_y, psnr


def _ff(path):
    # ffmpeg filter-graph option values: forward slashes, and the drive colon escaped
    return path.replace("\\", "/").replace(":", "\\:")


def yavg_series(path, out_dir, tag, vf_prefix=""):
    """signalstats YAVG per frame through metadata=print; vf_prefix runs before it (crop, tblend)."""
    meta = os.path.join(out_dir, f"yavg_{tag}.txt")
    vf = f"{vf_prefix}signalstats,metadata=print:key=lavfi.signalstats.YAVG:file={_ff(meta)}"
    run(["ffmpeg", "-v", "error", "-y", "-i", path, "-vf", vf, "-f", "null", "-"])
    series = {}
    frame = None
    rf = re.compile(r"^frame:(\d+)")
    rv = re.compile(r"lavfi\.signalstats\.YAVG=([\d.]+)")
    with open(meta, "r", encoding="utf-8", errors="replace") as f:
        for line in f:
            m = rf.match(line)
            if m:
                frame = int(m.group(1))
                continue
            m = rv.search(line)
            if m and frame is not None:
                series[frame] = float(m.group(1))
    return series


def rolling_median(vals, half=15):
    import statistics
    out = []
    n = len(vals)
    for i in range(n):
        lo, hi = max(0, i - half), min(n, i + half + 1)
        out.append(statistics.median(vals[lo:hi]))
    return out


def main(argv):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("base")
    ap.add_argument("new")
    ap.add_argument("--out-dir", default=None)
    ap.add_argument("--rows", default="0.45:1.0", help="water rows as height fractions y0:y1")
    ap.add_argument("--stills", type=int, default=0, help="amplified difference stills for the N worst frames")
    ap.add_argument("--ssim-floor", type=float, default=0.95)
    ap.add_argument("--spike", type=float, default=3.0, help="flag NEW frames whose tblend delta exceeds this x rolling median")
    args = ap.parse_args(argv)

    if shutil.which("ffmpeg") is None or shutil.which("ffprobe") is None:
        raise SystemExit("ffmpeg/ffprobe not on PATH")
    out_dir = args.out_dir or (os.path.splitext(args.new)[0] + "_raildiff")
    os.makedirs(out_dir, exist_ok=True)

    wb, hb, fps_b, nb_b = probe(args.base)
    wn, hn, fps_n, nb_n = probe(args.new)
    print(f"base {args.base}: {wb}x{hb} {fps_b:g} fps {nb_b} frames")
    print(f"new  {args.new}: {wn}x{hn} {fps_n:g} fps {nb_n} frames")
    if (wb, hb) != (wn, hn):
        raise SystemExit("frame sizes differ; scale one first (the comparison would be meaningless)")
    fps = fps_n or 30.0

    y0, y1 = (float(v) for v in args.rows.split(":"))
    crop_h = max(2, int(round(hn * (y1 - y0)))) // 2 * 2
    crop_y = int(round(hn * y0)) // 2 * 2
    crop = f"crop={wn}:{crop_h}:0:{crop_y},"

    ssim_all, ssim_y, psnr = ssim_psnr(args.base, args.new, out_dir)
    yb = yavg_series(args.base, out_dir, "base")
    yn = yavg_series(args.new, out_dir, "new")
    tb = yavg_series(args.base, out_dir, "tb_base", crop + "tblend=all_mode=difference,")
    tn = yavg_series(args.new, out_dir, "tb_new", crop + "tblend=all_mode=difference,")

    n = min(len(ssim_all), len(yb), len(yn))
    frames = list(range(n))
    with open(os.path.join(out_dir, "series.csv"), "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["frame", "t", "ssim_all", "ssim_y", "psnr", "yavg_base", "yavg_new", "tb_base", "tb_new"])
        for i in frames:
            w.writerow([i, f"{i / fps:.3f}", f"{ssim_all.get(i, float('nan')):.5f}",
                        f"{ssim_y.get(i, float('nan')):.5f}", f"{psnr.get(i, float('nan')):.3f}",
                        f"{yb.get(i, float('nan')):.3f}", f"{yn.get(i, float('nan')):.3f}",
                        f"{tb.get(i, float('nan')):.4f}", f"{tn.get(i, float('nan')):.4f}"])

    # ---- SSIM
    worst = sorted(frames, key=lambda i: ssim_all.get(i, 1.0))[:5]
    below = [i for i in frames if ssim_all.get(i, 1.0) < args.ssim_floor]
    print(f"\nSSIM (All) over {n} frames: mean {sum(ssim_all[i] for i in frames) / n:.4f}, "
          f"min {ssim_all[worst[0]]:.4f} at frame {worst[0]} (t={worst[0] / fps:.2f} s); "
          f"{len(below)} frames below {args.ssim_floor}")
    print("  worst five: " + ", ".join(f"f{i} {ssim_all[i]:.4f} (t={i / fps:.1f}s)" for i in worst))
    sec = int(n / fps)
    print("  per second (All): " + " ".join(
        f"{s}s:{sum(ssim_all.get(int(s * fps) + k, 1.0) for k in range(int(fps))) / int(fps):.3f}"
        for s in range(0, sec, 2)))

    # ---- YAVG per second, both recordings
    dy = []
    for s in range(sec):
        ids = [int(s * fps) + k for k in range(int(fps))]
        mb = sum(yb.get(i, 0.0) for i in ids) / len(ids)
        mn = sum(yn.get(i, 0.0) for i in ids) / len(ids)
        dy.append((abs(mn - mb), s, mb, mn))
    dy_max = max(dy) if dy else (0.0, 0, 0.0, 0.0)
    print(f"YAVG per second: max |new - base| {dy_max[0]:.3f}/255 at {dy_max[1]} s "
          f"(base {dy_max[2]:.2f}, new {dy_max[3]:.2f}); mean |diff| {sum(d[0] for d in dy) / max(1, len(dy)):.3f}/255")

    # ---- the pop instrument: frame-to-frame change over the water rows
    tb_ids = sorted(k for k in tn if k in tb)
    tn_v = [tn[k] for k in tb_ids]
    tb_v = [tb[k] for k in tb_ids]
    med_n = rolling_median(tn_v)
    med_b = rolling_median(tb_v)
    flagged, base_spikes = [], []
    for j, k in enumerate(tb_ids):
        new_spike = tn_v[j] > args.spike * max(med_n[j], 0.05)
        base_spike = tb_v[j] > args.spike * max(med_b[j], 0.05)
        if base_spike:
            base_spikes.append(k)
        if new_spike and not base_spike:
            flagged.append((k, tn_v[j], med_n[j], tb_v[j]))
    print(f"tblend |f - (f-1)| over rows {y0:.2f}..{y1:.2f} of the height: new median "
          f"{sorted(tn_v)[len(tn_v) // 2] if tn_v else 0:.3f}, base median "
          f"{sorted(tb_v)[len(tb_v) // 2] if tb_v else 0:.3f} (/255); base has {len(base_spikes)} "
          f"spikes > {args.spike}x its rolling median" +
          (f" at frames {base_spikes[:12]}{'...' if len(base_spikes) > 12 else ''}" if base_spikes else ""))
    if flagged:
        print(f"  FLAGGED {len(flagged)} new-only spikes (frame, delta, rolling median, base delta):")
        for k, v, m, b in flagged[:20]:
            print(f"    f{k} t={k / fps:.2f}s  new {v:.3f} vs median {m:.3f}  base {b:.3f}")
    else:
        print("  no new-only spikes: every frame-to-frame change in NEW is within "
              f"{args.spike}x its rolling median, or BASE spikes there too")

    # ---- stills for the eye, worst SSIM frames
    if args.stills > 0:
        for i in worst[:args.stills]:
            t = i / fps
            out = os.path.join(out_dir, f"diff_f{i:04d}.png")
            run(["ffmpeg", "-v", "error", "-y", "-ss", f"{t:.3f}", "-i", args.base, "-ss", f"{t:.3f}",
                 "-i", args.new, "-lavfi", "[0:v][1:v]blend=all_mode=difference,curves=all='0/0 0.0625/1'",
                 "-frames:v", "1", out])
            print(f"  wrote {out} (|base - new| x16, frame {i})")

    print(f"series -> {os.path.join(out_dir, 'series.csv')}")
    return 1 if (flagged or below) else 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
