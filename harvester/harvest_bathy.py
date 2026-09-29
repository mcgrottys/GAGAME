#!/usr/bin/env python3
"""GAGAME Harvester, M5 groundwork: NOAA CUDEM 1/9-arc-second topobathy for a window or a place.

Finds its tiles through the dataset's own TILE INDEX (tileindex_NCEI_ninth_Topobathy_2014.zip on
the NODD S3 bucket: one polygon per tile, carrying the tile's region folder, its S3 URL and its
declared CRS), fetches the tiles covering the requested box (one-time, cached forever -- these
are static products), decodes them with the stdlib GeoTIFF reader (WINDOWED: only the compressed
blocks under the crop are ever decompressed), and emits:

    data/bathy/<name>.json + <name>.f32   -- engine grid, metres NAVD88, row 0 = north
    data/bathy/preview_<name>.png         -- hypsometric preview (also stdlib-only)

CUDEM merges topography and bathymetry, so the jetties, the beach, and the channel are all in
the same surface -- which is exactly what M5 wants to render.

Two ways in:
    --window merrimack|capeann|boston     the named New England windows, byte-for-byte what
                                          they always produced (same tiles, same order, same grid)
    --place NAME --box lon0,lat0,lon1,lat1 [--near lon,lat] [--step N] [--plan]
            [--max-bytes N]               any place the index covers: region folder and tiles
                                          come from the index, downloads are PLANNED (sizes from
                                          the S3 listing), nearest-first from --near, capped by
                                          the budget, and go through harvester/polite.py (never
                                          overwrites; the grid is written once)
    --offline                             either way: never touch the network; tiles must be cached
    --out DIR                             write the grid elsewhere than data/bathy (the proofs)
"""

import io
import json
import math
import os
import re
import struct
import sys
import zipfile
import zlib
from array import array
from datetime import datetime, timezone

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import geotiff  # noqa: E402
from harvest_currents import fetch, log  # noqa: E402  (same polite fetch + cache)

# The index lives beside the tiles on the NODD mirror (coast.noaa.gov serves the same file).
S3_BUCKET = "noaa-nos-coastal-lidar-pds"
S3_DIR = "dem/NCEI_ninth_Topobathy_2014_8483/"
INDEX_NAME = "tileindex_NCEI_ninth_Topobathy_2014.zip"
INDEX_URL = f"https://{S3_BUCKET}.s3.amazonaws.com/{S3_DIR}{INDEX_NAME}"

# M6d: the WIDE window -- the river to Rocks Village (~km 15), the mouth, and Plum Island
# Sound, so the SWE solver holds most of the real tidal prism instead of borrowing it through
# a gain. Two tiles stitch along 42.75.
# M6w: the window is a PARAMETER -- harvest any focus region into its own CUDEM plane, each
# registered as a source in the earth.height stack (HQ static insets, per the user's rule).
WINDOWS = {
    "merrimack": {"lon0": -71.000, "lon1": -70.770, "lat0": 42.700, "lat1": 42.845},
    "capeann":   {"lon0": -70.950, "lon1": -70.560, "lat0": 42.480, "lat1": 42.700},
    "boston":    {"lon0": -71.100, "lon1": -70.780, "lat0": 42.250, "lat1": 42.480},
}
WIN = WINDOWS["merrimack"]
STEP = 4   # every 4th sample: ~13.7 m -- 2.2x the old area at the old cell count; the jetties
           # stay 2+ texels wide and the tessellated waves carry the close-up detail anyway

# Raw NCEI tiles are 100-400 MB each: cache them on the user-granted big-data drive when it
# exists (offline forever), fall back to the repo cache. Place tiles land in a region subfolder
# (<cache>/<region>/<tile>); the flat layout and cache/bathy are where the first New England
# tiles already live, so they are LOOKED IN before anything is fetched.
BIGCACHE = r"D:\DataCache\GAGAME\cudem"
LEGACY_CACHE = os.path.join("cache", "bathy")


# ------------------------------------------------------------------------------------ tiles

def read_tile_index(zip_bytes):
    """The index shapefile, stdlib only: DBF columns (location = <region>/<tile>.tif, srs,
    URL) + each polygon's bbox. -> [{'name', 'region', 'url', 'srs', 'bbox'}]."""
    z = zipfile.ZipFile(io.BytesIO(zip_bytes))
    names = z.namelist()
    shp = z.read(next(n for n in names if n.lower().endswith(".shp")))
    dbf = z.read(next(n for n in names if n.lower().endswith(".dbf")))
    nrec = struct.unpack("<I", dbf[4:8])[0]
    hsz, rsz = struct.unpack("<2H", dbf[8:12])
    fields = []
    p = 32
    while p < hsz - 1 and dbf[p] != 0x0D:
        fields.append((dbf[p:p + 11].split(b"\0")[0].decode("ascii"), dbf[p + 16]))
        p += 32
    rows = []
    for r in range(nrec):
        row = dbf[hsz + r * rsz:hsz + (r + 1) * rsz]
        q = 1
        vals = {}
        for fname, flen in fields:
            vals[fname.lower()] = row[q:q + flen].decode("latin1").strip()
            q += flen
        rows.append(vals)
    boxes = []
    off = 100
    while off + 8 <= len(shp):
        clen = struct.unpack(">i", shp[off + 4:off + 8])[0] * 2
        rec = shp[off + 8:off + 8 + clen]
        off += 8 + clen
        st = struct.unpack("<i", rec[:4])[0] if len(rec) >= 4 else 0
        boxes.append(struct.unpack("<4d", rec[4:36]) if st in (5, 15, 25) else None)
    tiles = []
    for vals, bb in zip(rows, boxes):
        loc = vals.get("location", "")
        region, _, name = loc.rpartition("/")
        tiles.append({"name": name, "region": region, "url": vals.get("url", ""),
                      "srs": vals.get("srs", ""), "bbox": bb})
    return tiles


def tile_bounds(name):
    """ncei19_n43x00_w070x75_* -> the name gives the tile's NORTHWEST corner; cells are 0.25 deg.
    (Florida's folder spells the separator 'X': ncei19_n26X00_w080X25_2018v1.tif.)"""
    m = re.search(r"([ns])(\d+)x(\d+)_([ew])(\d+)x(\d+)", name, re.I)
    if not m:
        return None
    lat_n = int(m.group(2)) + int(m.group(3)) / 100.0
    lon_w = int(m.group(5)) + int(m.group(6)) / 100.0
    if m.group(1).lower() == "s":
        lat_n = -lat_n
    if m.group(4).lower() == "w":
        lon_w = -lon_w
    return {"lat0": lat_n - 0.25, "lat1": lat_n, "lon0": lon_w, "lon1": lon_w + 0.25}


def tiles_for(win, index):
    """Index tiles whose 0.25-degree cell meets the window, in URL order (the order the old
    directory-listing path processed them, which decides who writes the seam rows)."""
    needed = []
    for t in sorted(index, key=lambda t: t["url"]):
        b = tile_bounds(t["name"])
        if not b:
            continue
        if (b["lon0"] < win["lon1"] and b["lon1"] > win["lon0"] and
                b["lat0"] < win["lat1"] and b["lat1"] > win["lat0"]):
            needed.append((t["name"], t["url"], b, t))
    return needed


def cached_tile(t, cache_dir):
    """Where a tile already is, if anywhere: region subfolder, the flat big-cache layout, the
    M5 repo cache. None if it has to be fetched."""
    for p in (os.path.join(cache_dir, t["region"], t["name"]),
              os.path.join(cache_dir, t["name"]),
              os.path.join(LEGACY_CACHE, t["name"])):
        if os.path.isfile(p):
            return p
    return None


def km_to_cell(b, lon, lat):
    """Distance (km) from a point to a tile's cell (0 inside) -- the nearest-first order."""
    clon = min(max(lon, b["lon0"]), b["lon1"])
    clat = min(max(lat, b["lat0"]), b["lat1"])
    return math.hypot((lon - clon) * 111.32 * math.cos(math.radians(lat)), (lat - clat) * 110.57)


# ------------------------------------------------------------------------------------ output

def write_png_gray(path, pix, w, h, mode="wb"):
    """8-bit grayscale PNG, stdlib only."""
    raw = bytearray()
    for y in range(h):
        raw.append(0)
        raw += bytes(pix[y * w:(y + 1) * w])

    def chunk(tag, payload):
        c = struct.pack(">I", len(payload)) + tag + payload
        return c + struct.pack(">I", zlib.crc32(tag + payload) & 0xFFFFFFFF)

    with open(path, mode) as f:
        f.write(b"\x89PNG\r\n\x1a\n")
        f.write(chunk(b"IHDR", struct.pack(">IIBBBBB", w, h, 8, 0, 0, 0, 0)))
        f.write(chunk(b"IDAT", zlib.compress(bytes(raw), 6)))
        f.write(chunk(b"IEND", b""))


def build_grid(win, needed, step, load_tile):
    """Decode the tiles under `win` into a regular lat/lon grid at `step` * 1/9 arcsec, row 0 =
    north, thalweg-preserving. load_tile(name, url) -> bytes. Returns (grid, nx, ny, dlon, dlat,
    declared) where `declared` holds each tile's GeoTIFF georeferencing keys."""
    src_step = 1.0 / 9.0 / 3600.0
    dlat = src_step * step
    dlon = src_step * step
    nx = int((win["lon1"] - win["lon0"]) / dlon)
    ny = int((win["lat1"] - win["lat0"]) / dlat)
    grid = array("f", [float("nan")]) * (nx * ny)
    log(f"output grid {nx} x {ny} at ~{dlat * 110574:.1f} m")
    declared = {}

    for name, url, b in needed:
        data = load_tile(name, url)
        tif = geotiff.GeoTiff(data)
        declared[name] = tif.geokeys()
        log(f"    {tif.width}x{tif.height}, comp {tif.compression}, predictor {tif.predictor}, "
            f"{'tiled' if tif.tiled else 'striped'} {tif.tw}x{tif.th}, "
            f"origin ({tif.lat0:.4f}, {tif.lon0:.4f}), nodata {tif.nodata}")

        # Window inside this tile (clamped), then copy every STEPth sample into the output grid.
        x0 = max(tif.px_of_lon(max(win["lon0"], b["lon0"])), 0)
        x1 = min(tif.px_of_lon(min(win["lon1"], b["lon1"])), tif.width)
        y0 = max(tif.py_of_lat(min(win["lat1"], b["lat1"])), 0)
        y1 = min(tif.py_of_lat(max(win["lat0"], b["lat0"])), tif.height)
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
            gx = int((lon - win["lon0"]) / dlon)
            colmap[xx] = gx if 0 <= gx < nx else -1
        for yy in range(h):
            lat = tif.lat0 - (y0 + yy) * tif.sy
            gy = int((win["lat1"] - lat) / dlat)          # row 0 = north in the OUTPUT too
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
    return grid, nx, ny, dlon, dlat, declared


def write_outputs(out_dir, name, win, grid, nx, ny, dlon, dlat, source, extra=None,
                  exclusive=False):
    """<name>.f32 + <name>.json + preview_<name>.png. exclusive=True refuses to replace any
    existing file (place harvests only ever ADD). Returns [(path, bytes)] written."""
    valid = [v for v in grid if not math.isnan(v)]
    if not valid:
        raise RuntimeError("window decoded to all-nodata")
    lo, hi = min(valid), max(valid)
    below = sum(1 for v in valid if v < 0.0)
    log(f"elevation range [{lo:.1f}, {hi:.1f}] m, {100.0 * below / len(valid):.0f}% below datum, "
        f"{100.0 * len(valid) / (nx * ny):.0f}% coverage")
    paths = [os.path.join(out_dir, f"{name}.f32"), os.path.join(out_dir, f"{name}.json"),
             os.path.join(out_dir, f"preview_{name}.png")]
    mode = "xb" if exclusive else "wb"
    if exclusive:
        clash = [p for p in paths if os.path.exists(p)]
        if clash:
            raise SystemExit(f"refusing to overwrite {clash} (place harvests only add)")

    with open(paths[0], mode) as f:
        enc = array("f", (v if not math.isnan(v) else -9999.0 for v in grid))
        f.write(struct.pack(f"<{len(enc)}f", *enc))
    meta = {
        "generated_utc": datetime.now(timezone.utc).isoformat(timespec="seconds"),
        "source": source,
        "file": f"{name}.f32", "nx": nx, "ny": ny,
        "lon0": win["lon0"], "lat1": win["lat1"], "dlon": dlon, "dlat": dlat,
        "row0": "north", "nodata": -9999.0,
        "min_m": lo, "max_m": hi,
    }
    meta.update(extra or {})
    with open(paths[1], mode.replace("b", ""), encoding="utf-8") as f:
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
    write_png_gray(paths[2], pix, nx, ny, mode)
    log(f"wrote {out_dir}/{name}.json + .f32 + preview ({nx}x{ny})")
    return [(p, os.path.getsize(p)) for p in paths]


def source_label(needed):
    regions = sorted({t["region"] for _, _, _, t in needed})
    return f"NOAA NCEI CUDEM ninth-arc topobathy (2014_8483 {' + '.join(regions)})"


# ------------------------------------------------------------------------------------ place

def region_sizes(regions, polite):
    """Tile sizes from the NODD bucket listing (one cached request per region folder)."""
    sizes = {}
    for region in regions:
        objs, _ = polite.s3_list(S3_BUCKET, f"{S3_DIR}{region}/",
                                 os.path.join("cache", "places", "_s3", f"cudem_{region}.xml"))
        for o in objs:
            sizes[o["key"].rsplit("/", 1)[-1]] = o["size"]
    return sizes


def plan_place(name, box, near, cache_dir, index, polite, offline=False):
    """The tile plan for a place: every index tile meeting the box, nearest-first from `near`,
    each with its size and where it is (or will be) cached. -> (needed, plan)."""
    needed = tiles_for(box, index)
    sizes = {} if offline else region_sizes(sorted({t["region"] for _, _, _, t in needed}),
                                            polite)
    plan = polite.Plan(f"CUDEM tiles for {name}")
    for tname, url, b, t in sorted(needed, key=lambda n: km_to_cell(n[2], *near)):
        have = cached_tile(t, cache_dir)
        plan.add(tname, url, sizes.get(tname) or (os.path.getsize(have) if have else None),
                 have or os.path.join(cache_dir, t["region"], tname),
                 km=round(km_to_cell(b, *near), 1), region=t["region"], srs=t["srs"])
    return needed, plan


def harvest_place(name, box, near=None, step=STEP, out_dir=None, plan_only=False,
                  offline=False, reserve=0, max_bytes=None, grid_box=None, out_name=None):
    """Plan, fetch (nearest-first, inside the budget) and grid a place. Returns a record for the
    place manifest: the plan (fetched / cached / left for later) and the files written.
    grid_box/out_name grid only part of the place (e.g. the part whose tiles are fetched)."""
    import polite
    out_dir = out_dir or os.path.join("data", "bathy")
    cache_dir = BIGCACHE if os.path.isdir(os.path.dirname(BIGCACHE)) else LEGACY_CACHE
    near = near or ((box["lon0"] + box["lon1"]) / 2, (box["lat0"] + box["lat1"]) / 2)
    index_path = os.path.join(cache_dir, INDEX_NAME)
    if offline and not os.path.exists(index_path):
        raise SystemExit(f"--offline: {index_path} is not cached")
    index = read_tile_index(polite.get(INDEX_URL, index_path)[0])
    needed, plan = plan_place(name, box, near, cache_dir, index, polite, offline)
    log(f"place '{name}' box {box}: the index names {len(needed)} tile(s) in "
        f"{sorted({t['region'] for _, _, _, t in needed})}")
    keep, cut = plan.fit(reserve=reserve)
    if max_bytes is not None:
        run, kept = 0, []
        for it in keep:
            if not it.get("cached"):
                if cut or run + it["size"] > max_bytes:
                    cut.append(it)
                    continue
                run += it["size"]
            kept.append(it)
        keep = kept
    plan.print(keep, cut)
    record = {"plan": [{k: v for k, v in it.items()} for it in plan.items],
              "later": [it["key"] for it in cut], "files": []}
    if plan_only:
        return record
    for it in keep:
        if not it.get("cached"):
            if offline:
                raise SystemExit(f"--offline: {it['key']} is not cached")
            polite.download(it["url"], it["dest"], it["size"], note=f"cudem {name}")
        it["cached"] = True
    have = {it["key"]: it["dest"] for it in plan.items if os.path.exists(it["dest"])}

    grid_out = os.path.join(out_dir, f"{out_name or name}.f32")
    if os.path.exists(grid_out):
        log(f"{grid_out} exists: tiles fetched, grid left as it is (a place only adds)")
        return record
    gbox = grid_box or box
    use = [(n, u, b) for n, u, b, t in tiles_for(gbox, index) if n in have]
    missing = [n for n, u, b, t in tiles_for(gbox, index) if n not in have]
    if missing:
        log(f"grid box {gbox}: {len(missing)} tile(s) not fetched yet: {missing}")
    if not use:
        log("no tiles on disk under the grid box; no grid written")
        return record

    def load_tile(tname, url):
        with open(have[tname], "rb") as f:
            data = f.read()
        log(f"    {tname}: {len(data) / 1e6:.1f} MB (cached)")
        return data

    grid, nx, ny, dlon, dlat, declared = build_grid(gbox, use, step, load_tile)
    used = [x for x in tiles_for(gbox, index) if x[0] in have]
    extra = {"place": name, "box": gbox, "step": step,
             "tiles": [{"name": n, "region": t["region"], "url": u, "srs_index": t["srs"],
                        "geokeys": declared.get(n)} for n, u, b, t in used],
             "tiles_missing": missing}
    record["files"] = write_outputs(out_dir, out_name or name, gbox, grid, nx, ny, dlon, dlat,
                                    source_label(used), extra, exclusive=True)
    record["declared"] = declared
    return record


# ------------------------------------------------------------------------------------ main

def _pair(s):
    return [float(v) for v in s.split(",")]


def main():
    global WIN
    args = sys.argv[1:]

    def opt(flag, default=None):
        return args[args.index(flag) + 1] if flag in args else default

    offline = "--offline" in args
    if "--place" in args:
        name = opt("--place")
        if "--box" not in args:
            raise SystemExit("--place needs --box lon0,lat0,lon1,lat1")
        lon0, lat0, lon1, lat1 = _pair(opt("--box"))
        box = {"lon0": lon0, "lat0": lat0, "lon1": lon1, "lat1": lat1}
        near = _pair(opt("--near")) if "--near" in args else None
        harvest_place(name, box, near=near, step=int(opt("--step", STEP)),
                      out_dir=opt("--out"), plan_only="--plan" in args, offline=offline,
                      max_bytes=float(opt("--max-bytes")) if "--max-bytes" in args else None)
        return

    win_name = opt("--window", "merrimack")
    if win_name not in WINDOWS:
        raise SystemExit(f"unknown window '{win_name}' (have: {list(WINDOWS)})")
    WIN = WINDOWS[win_name]

    out_dir = opt("--out", os.path.join("data", "bathy"))
    cache_dir = cache_dir = (BIGCACHE if os.path.isdir(os.path.dirname(BIGCACHE))
                             else os.path.join("cache", "bathy"))
    os.makedirs(cache_dir, exist_ok=True)
    os.makedirs(out_dir, exist_ok=True)
    log(f"window '{win_name}' {WIN} (tile cache: {cache_dir})")

    index_path = os.path.join(cache_dir, INDEX_NAME)
    if offline:
        if not os.path.exists(index_path):
            raise SystemExit(f"--offline: {index_path} is not cached")
        with open(index_path, "rb") as f:
            index = read_tile_index(f.read())
    else:
        index = read_tile_index(fetch(INDEX_URL, index_path, binary=True)[0])
    log(f"tile index lists {len(index)} tiles")
    needed = tiles_for(WIN, index)
    log(f"window needs {len(needed)} tile(s): {[n for n, _, _, _ in needed]}")
    if not needed:
        raise RuntimeError("no tile covers the window; check the name convention")

    def load_tile(name, url):
        t = next(t for n, _, _, t in needed if n == name)
        have = cached_tile(t, cache_dir)
        if have:
            with open(have, "rb") as f:
                data = f.read()
            cached = True
        elif offline:
            raise SystemExit(f"--offline: {name} is not cached")
        else:
            data, cached = fetch(url, os.path.join(cache_dir, name), binary=True, timeout=1200)
        log(f"    {name}: {len(data) / 1e6:.1f} MB {'(cached)' if cached else '(fetched)'}")
        return data

    grid, nx, ny, dlon, dlat, _ = build_grid(WIN, [(n, u, b) for n, u, b, _ in needed], STEP,
                                             load_tile)
    write_outputs(out_dir, win_name, WIN, grid, nx, ny, dlon, dlat, source_label(needed))


if __name__ == "__main__":
    main()
