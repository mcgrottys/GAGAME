# ==================================================================================================
#  harvest_gis.py - M6i: GIS ground truth for the stencil overlay.
#
#  Downloads the GSHHG native-binary distribution ONCE (NOAA/NCEI bulk product, ~110 MB zip,
#  cached under cache/gshhg/ forever), parses the full-resolution shorelines (gshhs_f.b) and
#  WDBII rivers (wdb_rivers_f.b) for the New England window plus a low-resolution global
#  coastline (gshhs_l.b), and writes simple float32 polyline runs the engine rasterizes into
#  stencil textures. The stencil is SURVEYED truth: imagery, composed heights and physics all
#  get compared against it, not against each other.
#
#  Output (data/gis/):
#    gis.json          {"coast_ne": ..., "rivers_ne": ..., "coast_global": ...}
#    *.bin             uint32 numPolylines, then per line: uint32 count, count x (f32 lon, f32 lat)
# ==================================================================================================
import io
import json
import os
import ssl
import struct
import sys
import urllib.request
import zipfile

CACHE = os.path.join("cache", "gshhg")
OUT = os.path.join("data", "gis")
# The dataset hops hosts over the years; try the canonical author mirror first, then NOAA's.
ZIP_URLS = [
    "https://www.soest.hawaii.edu/pwessel/gshhg/gshhg-bin-2.3.7.zip",
    "https://www.ngdc.noaa.gov/mgg/shorelines/data/gshhg/oldversions/version2.3.7/gshhg-bin-2.3.7.zip",
    "https://www.ncei.noaa.gov/pub/web/mgg/shorelines/data/gshhg/latest/gshhg-bin-2.3.7.zip",
]
ZIP_PATH = os.path.join(CACHE, "gshhg-bin-2.3.7.zip")

# The NE window (generous around the harvested data footprint) and the global decimation.
NE = {"lon0": -75.0, "lon1": -66.0, "lat0": 40.0, "lat1": 45.5}


def fetch_zip():
    os.makedirs(CACHE, exist_ok=True)
    if os.path.exists(ZIP_PATH) and os.path.getsize(ZIP_PATH) > 50_000_000:
        print(f"[gis] cache hit: {ZIP_PATH} ({os.path.getsize(ZIP_PATH)//1048576} MB)")
        return
    for url in ZIP_URLS:
        print(f"[gis] downloading {url} (one-time; lands in the forever-cache)")
        req = urllib.request.Request(url,
                                     headers={"User-Agent": "GAGAME/0.1 (hobby ocean sim)"})
        try:
            with urllib.request.urlopen(req, timeout=600) as r, open(ZIP_PATH, "wb") as f:
                while True:
                    chunk = r.read(1 << 20)
                    if not chunk:
                        break
                    f.write(chunk)
                    sys.stdout.write(f"\r[gis] {os.path.getsize(ZIP_PATH)//1048576} MB")
                    sys.stdout.flush()
            print(f"\n[gis] downloaded {os.path.getsize(ZIP_PATH)//1048576} MB")
            return
        except (urllib.error.HTTPError, urllib.error.URLError) as e:
            print(f"\n[gis] {e}; trying the next mirror")
        except ssl.SSLError:
            # The MOLA lesson: python's cert store may lack the CA; PowerShell uses Windows'.
            print("\n[gis] SSL failure -- fetch via PowerShell into the cache and re-run:")
            print(f"  Invoke-WebRequest {url} -OutFile {ZIP_PATH}")
            sys.exit(1)
    print("[gis] every mirror failed")
    sys.exit(1)


def parse_gshhg(blob, west, east, south, north, min_points=2, decimate=1, want_levels=None):
    """GSHHG native binary: per polygon a 44-byte big-endian header (id, n, flag, west, east,
    south, north in micro-degrees, area, area_full, container, ancestor) then n points of
    (lon, lat) micro-degrees. Longitudes may run 0..360; wrap to -180..180. flag & 255 is the
    hierarchy level: 1 land, 2 lake, 3 island-in-lake, 4 pond. Returns (level, points)."""
    lines = []
    off = 0
    size = len(blob)
    while off + 44 <= size:
        (_pid, n, flag, w, e, s, nn, _a, _af, _c, _anc) = struct.unpack_from(">11i", blob, off)
        off += 44
        if n <= 0 or off + 8 * n > size:
            break
        level = flag & 255
        w *= 1e-6
        e *= 1e-6
        s *= 1e-6
        nn *= 1e-6
        if w > 180.0:
            w -= 360.0
        if e > 180.0:
            e -= 360.0
        keep = not (e < west or w > east or nn < south or s > north)
        # A polygon spanning the dateline confuses the box test; global sets keep everything.
        if west <= -180.0 and east >= 180.0:
            keep = True
        if want_levels is not None and level not in want_levels:
            keep = False
        if not keep:
            off += 8 * n
            continue
        pts = struct.unpack_from(f">{2 * n}i", blob, off)
        off += 8 * n
        line = []
        for i in range(0, 2 * n, 2 * decimate):
            lon = pts[i] * 1e-6
            lat = pts[i + 1] * 1e-6
            if lon > 180.0:
                lon -= 360.0
            line.append((lon, lat))
        if len(line) >= min_points:
            lines.append((level, line))
    return lines


import math


# The Merrimack z14 Mercator window: MUST match main.cpp's winOrg/size (one shared frame).
WIN_ORG_X = 4935 * 256
WIN_ORG_Y = 6008 * 256
WIN_SIZE = 16384
N14 = 16384 * 256


def bake_mask(levelled, rows, cols, row_to_lat, lon_to_col, lon_min, lon_max):
    """Filled land mask by even-odd scanline parity, per GSHHG hierarchy level (odd levels are
    land, even are water); polygons are NOT clipped, so parity stays true for coastlines that
    wander outside the raster. Vector stays the authority -- this is one RASTER REALIZATION."""
    mask = bytearray(rows * cols)
    row_lats = [row_to_lat(r) for r in range(rows)]
    # Rows are monotonically decreasing in latitude; map a lat to the row range it falls in.
    lat_hi, lat_lo = row_lats[0], row_lats[-1]

    def row_of(lat):   # fractional row index for a latitude (rows decrease northward)
        # binary search over row_lats (descending)
        lo, hi = 0, rows - 1
        while lo < hi:
            mid = (lo + hi) // 2
            if row_lats[mid] > lat:
                lo = mid + 1
            else:
                hi = mid
        return lo

    crossings = [[] for _ in range(rows)]   # per row: (lon, level)
    for level, line in levelled:
        for i in range(len(line)):
            lon0, lat0 = line[i]
            lon1, lat1 = line[(i + 1) % len(line)]
            if lat0 == lat1:
                continue
            if abs(lon1 - lon0) > 180.0:
                continue   # a dateline wrap, not a real chord; parity slack far from here
            if max(lat0, lat1) < lat_lo or min(lat0, lat1) > lat_hi:
                continue
            r0 = row_of(max(lat0, lat1))
            r1 = row_of(min(lat0, lat1))
            for r in range(max(r0, 0), min(r1 + 1, rows)):
                lat = row_lats[r]
                lo, hi = (lat0, lat1) if lat0 < lat1 else (lat1, lat0)
                # Half-open [lo, hi): shared vertices count once.
                if not (lo <= lat < hi):
                    continue
                t = (lat - lat0) / (lat1 - lat0)
                crossings[r].append((lon0 + (lon1 - lon0) * t, level))
    for r in range(rows):
        if not crossings[r]:
            continue
        evs = sorted(crossings[r])
        parity = [0, 0, 0, 0, 0]
        spans = []
        prev_lon = -1e9
        state = False
        for lon, level in evs:
            new_state = (parity[1] ^ parity[2] ^ parity[3] ^ parity[4]) == 1
            if new_state:
                spans.append((prev_lon, lon))
            parity[min(level, 4)] ^= 1
            prev_lon = lon
        for a, b in spans:
            c0 = max(0, int(math.ceil(lon_to_col(max(a, lon_min)) - 0.5)))
            c1 = min(cols - 1, int(math.floor(lon_to_col(min(b, lon_max)) - 0.5)))
            base = r * cols
            for c in range(c0, c1 + 1):
                mask[base + c] = 255
    return mask


def clip_to_box(lines, box):
    """Split polylines into runs whose points fall inside the box (segment-level clip is
    overkill for a stencil; a half-pixel of slack at the box edge is invisible)."""
    out = []
    for line in lines:
        run = []
        for lon, lat in line:
            inside = box["lon0"] <= lon <= box["lon1"] and box["lat0"] <= lat <= box["lat1"]
            if inside:
                run.append((lon, lat))
            elif run:
                if len(run) >= 2:
                    out.append(run)
                run = []
        if len(run) >= 2:
            out.append(run)
    return out


def write_bin(path, lines):
    with open(path, "wb") as f:
        f.write(struct.pack("<I", len(lines)))
        for line in lines:
            f.write(struct.pack("<I", len(line)))
            for lon, lat in line:
                f.write(struct.pack("<ff", lon, lat))
    total = sum(len(l) for l in lines)
    print(f"[gis] {path}: {len(lines)} polylines, {total} points")


def main():
    fetch_zip()
    os.makedirs(OUT, exist_ok=True)
    zf = zipfile.ZipFile(ZIP_PATH)
    names = {os.path.basename(n): n for n in zf.namelist()}

    def member(base):
        if base not in names:
            print(f"[gis] FATAL: {base} not in the zip ({sorted(names)[:8]}...)")
            sys.exit(1)
        return zf.read(names[base])

    # ---- VECTORS (the authority; the engine renders these as line geometry) ----
    coast_f = parse_gshhg(member("gshhs_f.b"), NE["lon0"], NE["lon1"], NE["lat0"], NE["lat1"])
    coast_ne = clip_to_box([l for lv, l in coast_f if lv in (1, 2)], NE)
    write_bin(os.path.join(OUT, "coast_ne.bin"), coast_ne)

    rivers_f = parse_gshhg(member("wdb_rivers_f.b"), NE["lon0"], NE["lon1"], NE["lat0"],
                           NE["lat1"])
    rivers_ne = clip_to_box([l for _lv, l in rivers_f], NE)
    write_bin(os.path.join(OUT, "rivers_ne.bin"), rivers_ne)

    coast_l = parse_gshhg(member("gshhs_l.b"), -180.0, 180.0, -90.0, 90.0, decimate=2,
                          want_levels=(1,))
    write_bin(os.path.join(OUT, "coast_global.bin"), [l for _lv, l in coast_l])

    # ---- RASTER REALIZATIONS (filled land masks; the default classifier) ----
    print("[gis] baking the window land mask (4096^2, shared Mercator z14 frame)...")

    def win_row_lat(r):
        y = (WIN_ORG_Y + (r + 0.5) * WIN_SIZE / 4096.0) / N14
        return math.degrees(math.atan(math.sinh(math.pi * (1.0 - 2.0 * y))))

    def win_lon_col(lon):
        return ((lon + 180.0) / 360.0 * N14 - WIN_ORG_X) * 4096.0 / WIN_SIZE

    win_lon0 = WIN_ORG_X / N14 * 360.0 - 180.0
    win_lon1 = (WIN_ORG_X + WIN_SIZE) / N14 * 360.0 - 180.0
    mask_ne = bake_mask(coast_f, 4096, 4096, win_row_lat, win_lon_col, win_lon0, win_lon1)
    with open(os.path.join(OUT, "landmask_ne.raw"), "wb") as f:
        f.write(mask_ne)
    land_pct = 100.0 * sum(1 for b in mask_ne if b) / len(mask_ne)
    print(f"[gis] landmask_ne.raw: {land_pct:.1f}% land")

    print("[gis] baking the global land mask (4096x2048 equirect)...")
    coast_l_all = parse_gshhg(member("gshhs_l.b"), -180.0, 180.0, -90.0, 90.0)
    gw, gh = 4096, 2048
    mask_g = bake_mask(coast_l_all, gh, gw, lambda r: 90.0 - (r + 0.5) / gh * 180.0,
                       lambda lon: (lon + 180.0) / 360.0 * gw, -180.0, 180.0)
    with open(os.path.join(OUT, "landmask_global.raw"), "wb") as f:
        f.write(mask_g)
    land_pct = 100.0 * sum(1 for b in mask_g if b) / len(mask_g)
    print(f"[gis] landmask_global.raw: {land_pct:.1f}% land (Earth is ~29%)")

    with open(os.path.join(OUT, "gis.json"), "w") as f:
        json.dump(
            {
                "coast_ne": "coast_ne.bin",
                "rivers_ne": "rivers_ne.bin",
                "coast_global": "coast_global.bin",
                "landmask_ne": "landmask_ne.raw",
                "landmask_ne_dim": 4096,
                "landmask_global": "landmask_global.raw",
                "landmask_global_w": gw,
                "landmask_global_h": gh,
                "win_org_px": [WIN_ORG_X, WIN_ORG_Y],
                "win_size_px": WIN_SIZE,
                "source": "GSHHG 2.3.7 full-res shorelines + WDBII rivers (NOAA/NCEI)",
                "crs": "EPSG:4326",
                "ne_window": NE,
            },
            f,
            indent=1,
        )
    print("[gis] done -> data/gis/gis.json")


if __name__ == "__main__":
    main()
