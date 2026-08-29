#!/usr/bin/env python3
"""GAGAME Harvester, M5 groundwork: NOAA CUDEM 1/9-arc-second topobathy for the Merrimack mouth.

Lists the NCEI ninth-arc tile directory, fetches the tile(s) covering the requested window
(one-time, cached forever -- these are static products), decodes them with the stdlib GeoTIFF
reader (WINDOWED: only the compressed blocks under the crop are ever decompressed), and emits:

    data/bathy/merrimack.json + merrimack.f32   -- engine grid, metres NAVD88-ish, row 0 = north
    data/bathy/preview.png                      -- hypsometric preview (also stdlib-only)

CUDEM merges topography and bathymetry, so the jetties, the beach, and the channel are all in
the same surface -- which is exactly what M5 wants to render.
"""

import json
import math
import os
import re
import struct
import sys
import zlib
from array import array
from datetime import datetime, timezone

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import geotiff  # noqa: E402
from harvest_currents import fetch, log  # noqa: E402  (same polite fetch + cache)

BASE = "https://coast.noaa.gov/htdata/raster2/elevation/NCEI_ninth_Topobathy_2014_8483/MA_NH_ME/"

# M6d: the WIDE window -- the river to Rocks Village (~km 15), the mouth, and Plum Island
# Sound, so the SWE solver holds most of the real tidal prism instead of borrowing it through
# a gain. Two tiles stitch along 42.75.
WIN = {"lon0": -71.000, "lon1": -70.770, "lat0": 42.700, "lat1": 42.845}
STEP = 4   # every 4th sample: ~13.7 m -- 2.2x the old area at the old cell count; the jetties
           # stay 2+ texels wide and the tessellated waves carry the close-up detail anyway


def list_tiles(cache_dir):
    """Returns [(name, url)] -- the index links straight into the NODD S3 bucket (mirrors-first,
    exactly per the playbook)."""
    html, _ = fetch(BASE, os.path.join(cache_dir, "index.html"), max_age_s=7 * 86400)
    tiles = []
    for url in sorted(set(re.findall(r'href="([^"]*ncei19[^"]+\.tif)"', html))):
        name = url.rsplit("/", 1)[-1]
        tiles.append((name, url if url.startswith("http") else BASE + url))
    log(f"directory lists {len(tiles)} tiles")
    return tiles


def tile_bounds(name):
    """ncei19_n43x00_w070x75_* -> the name gives the tile's NORTHWEST corner; cells are 0.25 deg."""
    m = re.search(r"n(\d+)x(\d+)_w(\d+)x(\d+)", name)
    if not m:
        return None
    lat_n = int(m.group(1)) + int(m.group(2)) / 100.0
    lon_w = -(int(m.group(3)) + int(m.group(4)) / 100.0)
    return {"lat0": lat_n - 0.25, "lat1": lat_n, "lon0": lon_w, "lon1": lon_w + 0.25}


def write_png_gray(path, pix, w, h):
    """8-bit grayscale PNG, stdlib only."""
    raw = bytearray()
    for y in range(h):
        raw.append(0)
        raw += bytes(pix[y * w:(y + 1) * w])

    def chunk(tag, payload):
        c = struct.pack(">I", len(payload)) + tag + payload
        return c + struct.pack(">I", zlib.crc32(tag + payload) & 0xFFFFFFFF)

    with open(path, "wb") as f:
        f.write(b"\x89PNG\r\n\x1a\n")
        f.write(chunk(b"IHDR", struct.pack(">IIBBBBB", w, h, 8, 0, 0, 0, 0)))
        f.write(chunk(b"IDAT", zlib.compress(bytes(raw), 6)))
        f.write(chunk(b"IEND", b""))


def main():
    out_dir = os.path.join("data", "bathy")
    cache_dir = os.path.join("cache", "bathy")
    os.makedirs(out_dir, exist_ok=True)

    tiles = list_tiles(cache_dir)
    needed = []
    for name, url in tiles:
        b = tile_bounds(name)
        if not b:
            continue
        if (b["lon0"] < WIN["lon1"] and b["lon1"] > WIN["lon0"] and
                b["lat0"] < WIN["lat1"] and b["lat1"] > WIN["lat0"]):
            needed.append((name, url, b))
    log(f"window needs {len(needed)} tile(s): {[n for n, _, _ in needed]}")
    if not needed:
        raise RuntimeError("no tile covers the window; check the name convention")

    # Output grid (regular lat/lon at STEP * 1/9 arcsec).
    src_step = 1.0 / 9.0 / 3600.0
    dlat = src_step * STEP
    dlon = src_step * STEP
    nx = int((WIN["lon1"] - WIN["lon0"]) / dlon)
    ny = int((WIN["lat1"] - WIN["lat0"]) / dlat)
    grid = array("f", [float("nan")]) * (nx * ny)
    log(f"output grid {nx} x {ny} at ~{dlat * 110574:.1f} m")

    for name, url, b in needed:
        cache = os.path.join(cache_dir, name)
        data, cached = fetch(url, cache, binary=True, timeout=1200)
        log(f"    {name}: {len(data) / 1e6:.1f} MB {'(cached)' if cached else '(fetched)'}")
        tif = geotiff.GeoTiff(data)
        log(f"    {tif.width}x{tif.height}, comp {tif.compression}, predictor {tif.predictor}, "
            f"{'tiled' if tif.tiled else 'striped'} {tif.tw}x{tif.th}, "
            f"origin ({tif.lat0:.4f}, {tif.lon0:.4f}), nodata {tif.nodata}")

        # Window inside this tile (clamped), then copy every STEPth sample into the output grid.
        x0 = max(tif.px_of_lon(max(WIN["lon0"], b["lon0"])), 0)
        x1 = min(tif.px_of_lon(min(WIN["lon1"], b["lon1"])), tif.width)
        y0 = max(tif.py_of_lat(min(WIN["lat1"], b["lat1"])), 0)
        y1 = min(tif.py_of_lat(max(WIN["lat0"], b["lat0"])), tif.height)
        w, h = x1 - x0, y1 - y0
        if w <= 0 or h <= 0:
            continue
        log(f"    decoding window {w} x {h} px")
        vals = tif.read_window(x0, y0, w, h)
        # THALWEG-PRESERVING box resampling (M6d). Plain point-sampling (the first cut) aliases
        # a 100 m channel at 13.7 m cells; plain box MEANS smear the deep thread shallow, which
        # slows the tidal wave (sqrt(gh)) and throttled the solved currents (+80 min phase lag
        # in the validation cycle). Hydraulic grids keep CONVEYANCE: wet-majority boxes take
        # half mean-of-wet, half deepest-sample; bank/land boxes take the plain mean.
        cnt = array("H", [0]) * (nx * ny)
        ssum = array("d", [0.0]) * (nx * ny)
        wcnt = array("H", [0]) * (nx * ny)
        wsum = array("d", [0.0]) * (nx * ny)
        vmin = array("f", [1e9]) * (nx * ny)
        colmap = array("i", [0]) * w
        for xx in range(w):
            lon = tif.lon0 + (x0 + xx) * tif.sx
            gx = int((lon - WIN["lon0"]) / dlon)
            colmap[xx] = gx if 0 <= gx < nx else -1
        for yy in range(h):
            lat = tif.lat0 - (y0 + yy) * tif.sy
            gy = int((WIN["lat1"] - lat) / dlat)          # row 0 = north in the OUTPUT too
            if not (0 <= gy < ny):
                continue
            base = gy * nx
            row = vals[yy * w:(yy + 1) * w]
            for xx in range(w):
                v = row[xx]
                if math.isnan(v):
                    continue
                gx = colmap[xx]
                if gx < 0:
                    continue
                i = base + gx
                cnt[i] += 1
                ssum[i] += v
                if v < -1.0:                # genuinely SUBTIDAL: the channel thread. Flats and
                    wcnt[i] += 1            # marsh (v ~ -1..+1) keep plain means -- deep-biasing
                    wsum[i] += v            # them inflates the basin's storage and ADDS lag,
                    if v < vmin[i]:         # which is exactly what the first cut of this rule
                        vmin[i] = v         # did to the validation cycle.
        copied = 0
        for i in range(nx * ny):
            if not cnt[i]:
                continue
            if wcnt[i] * 2 >= cnt[i]:
                grid[i] = 0.5 * (wsum[i] / wcnt[i]) + 0.5 * vmin[i]
            else:
                grid[i] = ssum[i] / cnt[i]
            copied += 1
        log(f"    filled {copied} cells from {sum(cnt)} samples (thalweg-preserving)")

    valid = [v for v in grid if not math.isnan(v)]
    if not valid:
        raise RuntimeError("window decoded to all-nodata")
    lo, hi = min(valid), max(valid)
    below = sum(1 for v in valid if v < 0.0)
    log(f"elevation range [{lo:.1f}, {hi:.1f}] m, {100.0 * below / len(valid):.0f}% below datum, "
        f"{100.0 * len(valid) / (nx * ny):.0f}% coverage")

    with open(os.path.join(out_dir, "merrimack.f32"), "wb") as f:
        enc = array("f", (v if not math.isnan(v) else -9999.0 for v in grid))
        f.write(struct.pack(f"<{len(enc)}f", *enc))
    meta = {
        "generated_utc": datetime.now(timezone.utc).isoformat(timespec="seconds"),
        "source": "NOAA NCEI CUDEM ninth-arc topobathy (2014_8483 MA_NH_ME)",
        "file": "merrimack.f32", "nx": nx, "ny": ny,
        "lon0": WIN["lon0"], "lat1": WIN["lat1"], "dlon": dlon, "dlat": dlat,
        "row0": "north", "nodata": -9999.0,
        "min_m": lo, "max_m": hi,
    }
    with open(os.path.join(out_dir, "merrimack.json"), "w", encoding="utf-8") as f:
        json.dump(meta, f, indent=1)

    # Hypsometric preview: dark deep -> light shallow -> mid gray at 0 -> bright land.
    pix = bytearray(nx * ny)
    for i, v in enumerate(grid):
        if math.isnan(v):
            pix[i] = 8
        elif v < 0:
            pix[i] = max(16, min(120, int(120 + v * 8)))      # -13 m .. 0 -> 16..120
        else:
            pix[i] = max(140, min(255, int(140 + v * 10)))    # land ramp
    write_png_gray(os.path.join(out_dir, "preview.png"), pix, nx, ny)
    log(f"wrote data/bathy/merrimack.json + .f32 + preview.png ({nx}x{ny})")


if __name__ == "__main__":
    main()
