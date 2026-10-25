# tools/wave_layers.py -- the solved wave field's layers on one sheet (Mark, 2026-10-24: "show every layer
# ... from the inputs and different steps in the pipeline to produce the output").
#
# `gagame ... --wave-map out/x/map.png` writes, beside the map, one PNG per stage of the solve
# (src/sim/WaveField.cpp LayerSink): map_0_depth.png, map_1_current.png, map_c<n>_2_k.png .. _7_amp.png for two
# components, map_8_rms_raw.png .. map_11_hs.png. This lays them out in pipeline order, labelled, with the map
# (eta) last, and writes map_sheet.png.
#
#   py -3 tools/wave_layers.py out/x/map.png [--width 480] [--crop x0,y0,x1,y1]
#
# --crop takes a box in the layers' own pixels (the solve's cells, north up) to look closer at one place.
import argparse
import glob
import os
import re

from PIL import Image, ImageDraw, ImageFont

STAGES = {
    "0_depth": "depth 0..40 m (log)",
    "1_current": "current: hue = toward, value = speed",
    "2_k": "k / k_deep (1..4)",
    "3_phase": "cos(phase)",
    "4_ray": "the ray's direction (hue)",
    "5_crease": "creases of the phase (red)",
    "6_gain": "action / shoaled swell, log2 (+-3 = 8x)",
    "7_amp": "amplitude / (2 a0)",
    "8_rms_raw": "rms before the limiter",
    "9_excess": "excess / 2.5",
    "10_limiter": "the limiter's factor",
    "11_hs": "Hs after the limiter",
}


def order_key(path, base):
    tail = os.path.basename(path)[len(os.path.basename(base)) + 1:-4]
    m = re.match(r"c(\d+)_(\d+)_", tail)
    if m:
        return (2, int(m.group(1)), int(m.group(2)))
    m = re.match(r"(\d+)_", tail)
    return (1 if int(m.group(1)) < 2 else 3, 0, int(m.group(1))) if m else (9, 0, 0)


def label_of(path, base):
    tail = os.path.basename(path)[len(os.path.basename(base)) + 1:-4]
    m = re.match(r"c(\d+)_(.*)", tail)
    if m:
        return "component %s: %s" % (m.group(1), STAGES.get(m.group(2), m.group(2)))
    return STAGES.get(tail, tail)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("map")
    ap.add_argument("--width", type=int, default=480)
    ap.add_argument("--crop", default="")
    ap.add_argument("--cols", type=int, default=4)
    a = ap.parse_args()
    base = a.map[:-4] if a.map.lower().endswith(".png") else a.map
    files = [f for f in glob.glob(base + "_*.png") if not f.endswith("_sheet.png")]
    files.sort(key=lambda f: order_key(f, base))
    files.append(a.map)
    crop = tuple(int(v) for v in a.crop.split(",")) if a.crop else None
    tiles = []
    for f in files:
        im = Image.open(f).convert("RGB")
        if crop:
            im = im.crop(crop)
        h = int(im.height * a.width / im.width)
        tiles.append((im.resize((a.width, h), Image.LANCZOS), "eta (the map)" if f == a.map else label_of(f, base)))
    th = max(t.height for t, _ in tiles) + 22
    rows = (len(tiles) + a.cols - 1) // a.cols
    sheet = Image.new("RGB", (a.cols * a.width + (a.cols + 1) * 6, rows * th + (rows + 1) * 6), (24, 24, 24))
    d = ImageDraw.Draw(sheet)
    font = ImageFont.load_default()
    for n, (t, lab) in enumerate(tiles):
        x = 6 + (n % a.cols) * (a.width + 6)
        y = 6 + (n // a.cols) * (th + 6)
        d.text((x + 2, y + 4), lab, fill=(230, 230, 230), font=font)
        sheet.paste(t, (x, y + 20))
    out = base + ("_sheet_crop.png" if crop else "_sheet.png")
    sheet.save(out)
    print("wrote %s: %d layers" % (out, len(tiles)))


if __name__ == "__main__":
    main()
