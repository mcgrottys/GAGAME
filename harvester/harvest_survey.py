# ==================================================================================================
#  harvest_survey.py - M7y: THE SURVEY UPGRADE. CUSP + NHD into the survey order.
#
#  Two authorities join the stencil (GAMEPLAN 13.5's named upgrade path):
#    * NOAA CUSP (Continually Updated Shoreline Product, NGS): the COASTLINE, meter-class,
#      referenced to Mean High Water, continually updated (the North Atlantic region file was
#      refreshed four days before this harvester was written). Distributed as regional zips
#      from https://geodesy.noaa.gov/dist_shoreline/ (found on the NSDE page itself).
#      CUSP is POLYLINES (open segments) -- an authority for WHERE the coast runs, not a
#      parity-fillable region source.
#    * USGS NHD HR (High Resolution, per-HU4 GeoPackages from The National Map's staged
#      products on S3): rivers (NHDFlowline), and -- the mask's missing half -- CLOSED
#      open-water polygons (NHDWaterbody lakes/ponds/reservoirs/estuaries, NHDArea wide
#      rivers/sea). HU4s 0106 (Saco + NH seacoast), 0107 (Merrimack), 0109 (MA coastal).
#
#  The mask recipe (landmask_ne.raw, same 4096^2 z14 window frame the engine already reads):
#    base  = GSHHG full-res land parity (harvest_gis's bake, reused verbatim), then
#    carve = each NHD open-water feature scanline-filled even-odd over its own rings
#            (outer ring minus islands, per feature -- overlapping features OR harmlessly)
#            and cleared to water. GSHHG says "land here"; NHD says "except this pond,
#            this river run, this estuary". CUSP stays vector -- the shoreline's authority
#            rides the vpack for the stencil and the coming photo-vs-bed gate.
#
#  Raw archives live forever in D:\DataCache\GAGAME\{cusp,nhd} (the 300 GB grant).
#  Engine-ready outputs (data/gis/): cusp_ne.bin, nhd_rivers_ne.bin, nhd_water_ne.bin
#  (+ .attrs.json fcode sidecars), the rebaked landmask_ne.raw, survey.json provenance.
#  harvest_vectors.py packs the bins into vectors.vpack (lossless, wedge importance).
#
#  CRS note, documented not corrected (the M6l aerial precedent): CUSP and NHD are NAD83
#  geographic (EPSG:4269 / 4617-adjacent realizations), ~1 m from WGS84 here.
# ==================================================================================================
import io
import json
import math
import os
import sqlite3
import struct
import sys
import urllib.request
import zipfile

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import harvest_gis   # the GSHHG base bake + window frame constants (one shared frame)

DCACHE = r"D:\DataCache\GAGAME"
CUSP_DIR = os.path.join(DCACHE, "cusp")
NHD_DIR = os.path.join(DCACHE, "nhd")
OUT = os.path.join("data", "gis")
PROOF = "proofs"

UA = "GAGAME/0.1 (hobby ocean simulator; marksmcgrotty@gmail.com)"

CUSP_URL = "https://geodesy.noaa.gov/dist_shoreline/North_Atlantic.zip"
NHD_HU4 = ["0106", "0107", "0109"]
NHD_URL = ("https://prd-tnm.s3.amazonaws.com/StagedProducts/Hydrography/NHD/HU4/GPKG/"
           "NHD_H_{hu}_HU4_GPKG.zip")

# The focus box: the z14 window plus a generous margin (Cape Ann to the NH seacoast, inland
# past the fall line). Everything vpack-bound and every mask carve clips here; the raw
# regional archives keep the rest for the day the window grows.
FOCUS = {"lon0": -71.85, "lon1": -69.85, "lat0": 42.05, "lat1": 43.55}

# NHD FCode families that are OPEN WATER (the mask carve) -- wetlands/marsh deliberately
# EXCLUDED: a marsh is land to a texture gate (Google photographs vegetation), water only
# when the tide says so (the engine's live-tide classifier owns that call).
def fcode_is_open_water(fc):
    fam = fc // 100
    return (fam == 390 or            # LakePond
            fam == 436 or            # Reservoir
            fc == 49300 or           # Estuary
            fam == 460 or            # StreamRiver (NHDArea: the wide river surface)
            fc == 44500 or           # SeaOcean
            fam == 336 or            # CanalDitch
            fc == 43100)             # Rapids

def fcode_is_river_line(fc):
    fam = fc // 100
    return (fam == 460 or            # StreamRiver
            fc == 55800 or           # ArtificialPath (thalweg through waterbodies)
            fam == 336)              # CanalDitch
    # NOT 56600 Coastline (CUSP owns the coast), NOT 428xx pipelines.


# ---------------------------------------------------------------- fetch (cache-first, polite)

def fetch(url, path, min_bytes):
    if os.path.exists(path) and os.path.getsize(path) >= min_bytes:
        print(f"[survey] cache hit: {path} ({os.path.getsize(path)//1048576} MB)")
        return
    os.makedirs(os.path.dirname(path), exist_ok=True)
    print(f"[survey] downloading {url}")
    req = urllib.request.Request(url, headers={"User-Agent": UA})
    with urllib.request.urlopen(req, timeout=1200) as r, open(path, "wb") as f:
        while True:
            chunk = r.read(1 << 20)
            if not chunk:
                break
            f.write(chunk)
            sys.stdout.write(f"\r[survey] {os.path.getsize(path)//1048576} MB")
            sys.stdout.flush()
    print(f"\n[survey] done: {path}")


# ---------------------------------------------------------------- WKB (GeoPackage blobs)

def _wkb_geoms(blob):
    """GeoPackage geometry blob -> list of (kind, parts) where kind is 'line'|'poly' and
    parts is a list of point runs [(lon, lat), ...]; polygons keep every ring, outer first
    (WKB order). Handles XY/XYZ/XYM/XYZM, both byte orders, Multi* and GeometryCollection."""
    if len(blob) < 8 or blob[0:2] != b"GP":
        return []
    flags = blob[3]
    env = (flags >> 1) & 7
    env_len = {0: 0, 1: 32, 2: 48, 3: 48, 4: 64}.get(env, 0)
    off = 8 + env_len
    out = []

    def parse(off):
        bo = "<" if blob[off] == 1 else ">"
        (t,) = struct.unpack_from(bo + "I", blob, off + 1)
        off += 5
        base = t % 1000
        dim = t // 1000            # 0 XY, 1 Z, 2 M, 3 ZM
        stride = 2 + (1 if dim in (1, 2) else 2 if dim == 3 else 0)

        def points(off, n):
            vals = struct.unpack_from(bo + f"{n * stride}d", blob, off)
            return [(vals[i * stride], vals[i * stride + 1]) for i in range(n)], \
                   off + 8 * n * stride

        if base == 1:              # Point: ignored (no mask/stencil use)
            return None, off + 8 * stride
        if base == 2:              # LineString
            (n,) = struct.unpack_from(bo + "I", blob, off)
            pts, off = points(off + 4, n)
            return ("line", [pts]), off
        if base == 3:              # Polygon
            (nr,) = struct.unpack_from(bo + "I", blob, off)
            off += 4
            rings = []
            for _ in range(nr):
                (n,) = struct.unpack_from(bo + "I", blob, off)
                pts, off = points(off + 4, n)
                rings.append(pts)
            return ("poly", rings), off
        if base in (4, 5, 6, 7):   # Multi* / collection: recurse
            (n,) = struct.unpack_from(bo + "I", blob, off)
            off += 4
            for _ in range(n):
                g, off = parse(off)
                if g:
                    out.append(g)
            return None, off
        return None, off           # unknown: caller's slice ends here

    g, _ = parse(off)
    if g:
        out.append(g)
    return out


def _gpkg_env(blob):
    """The GP header's envelope (minx, maxx, miny, maxy) if present, else None -- the cheap
    clip that skips full WKB decode for features far from the focus box."""
    if len(blob) < 8 or blob[0:2] != b"GP":
        return None
    env = (blob[3] >> 1) & 7
    if env == 0:
        return None
    return struct.unpack_from("<4d", blob, 8)


# ---------------------------------------------------------------- readers

def read_nhd_gpkg(gpkg_path, box):
    """-> (rivers [polyline], water [(fcode, rings)]) clipped to box by feature envelope."""
    db = sqlite3.connect(gpkg_path)
    cur = db.cursor()
    tables = {r[0].lower(): r[0] for r in
              cur.execute("SELECT table_name FROM gpkg_contents WHERE data_type='features'")}
    geocol = {r[0].lower(): r[1] for r in
              cur.execute("SELECT table_name, column_name FROM gpkg_geometry_columns")}

    def fcode_col(table):
        for r in cur.execute(f'PRAGMA table_info("{table}")'):
            if r[1].lower() == "fcode":
                return r[1]
        return None

    def rows(tname):
        table = tables.get(tname.lower())
        if not table:
            return
        gc = geocol.get(table.lower(), "geom")
        fc = fcode_col(table)
        sel = f'SELECT "{gc}"' + (f', "{fc}"' if fc else ", NULL") + f' FROM "{table}"'
        for blob, fcode in cur.execute(sel):
            if blob is None:
                continue
            e = _gpkg_env(blob)
            if e and (e[1] < box["lon0"] or e[0] > box["lon1"] or
                      e[3] < box["lat0"] or e[2] > box["lat1"]):
                continue
            yield blob, (int(fcode) if fcode is not None else 0)

    rivers, water = [], []
    for blob, fc in rows("NHDFlowline"):
        if not fcode_is_river_line(fc):
            continue
        for kind, parts in _wkb_geoms(blob):
            for pts in parts:
                if len(pts) >= 2:
                    rivers.append(pts)
    for tname in ("NHDWaterbody", "NHDArea"):
        for blob, fc in rows(tname):
            if not fcode_is_open_water(fc):
                continue
            for kind, parts in _wkb_geoms(blob):
                if kind == "poly" and parts:
                    water.append((fc, parts))
    db.close()
    return rivers, water


def read_cusp_zip(zip_path, box):
    """Every shapefile in the regional zip, polylines clipped (point-run) to box."""
    zf = zipfile.ZipFile(zip_path)
    shps = [n for n in zf.namelist() if n.lower().endswith(".shp")]
    print(f"[survey] CUSP archive: {len(shps)} shapefiles")
    lines = []
    for name in shps:
        data = zf.read(name)
        if len(data) < 100 or struct.unpack(">i", data[:4])[0] != 9994:
            continue
        off = 100
        while off + 8 <= len(data):
            clen = struct.unpack(">i", data[off + 4:off + 8])[0] * 2
            rec = data[off + 8:off + 8 + clen]
            off += 8 + clen
            if len(rec) < 44:
                continue
            st = struct.unpack("<i", rec[:4])[0]
            if st not in (3, 5, 13, 15, 23, 25):
                continue
            bx = struct.unpack("<4d", rec[4:36])
            if (bx[2] < box["lon0"] or bx[0] > box["lon1"] or
                    bx[3] < box["lat0"] or bx[1] > box["lat1"]):
                continue
            nparts, npts = struct.unpack("<2i", rec[36:44])
            parts = struct.unpack(f"<{nparts}i", rec[44:44 + 4 * nparts])
            base = 44 + 4 * nparts
            pts = [struct.unpack_from("<2d", rec, base + 16 * i) for i in range(npts)]
            for pi in range(nparts):
                s = parts[pi]
                e = parts[pi + 1] if pi + 1 < nparts else npts
                if e - s >= 2:
                    lines.append(pts[s:e])
    return lines


def clip_runs(lines, box):
    return harvest_gis.clip_to_box(lines, box)


# ---------------------------------------------------------------- the water carve

def carve_water(mask, rows, cols, features, row_to_lat, lon_to_col, lon_min, lon_max):
    """Clear mask texels to water inside each feature's rings (even-odd per FEATURE, so a
    pond's island ring correctly stays land; distinct features union harmlessly)."""
    row_lats = [row_to_lat(r) for r in range(rows)]
    lat_hi, lat_lo = row_lats[0], row_lats[-1]

    def row_of(lat):
        lo, hi = 0, rows - 1
        while lo < hi:
            mid = (lo + hi) // 2
            if row_lats[mid] > lat:
                lo = mid + 1
            else:
                hi = mid
        return lo

    carved = 0
    for _fc, rings in features:
        rmin, rmax = rows, -1
        crossings = {}
        for ring in rings:
            n = len(ring)
            for i in range(n):
                lon0, lat0 = ring[i]
                lon1, lat1 = ring[(i + 1) % n]
                if lat0 == lat1:
                    continue
                if max(lat0, lat1) < lat_lo or min(lat0, lat1) > lat_hi:
                    continue
                r0 = row_of(max(lat0, lat1))
                r1 = row_of(min(lat0, lat1))
                for r in range(max(r0, 0), min(r1 + 1, rows)):
                    lat = row_lats[r]
                    lo, hi = (lat0, lat1) if lat0 < lat1 else (lat1, lat0)
                    if not (lo <= lat < hi):
                        continue
                    t = (lat - lat0) / (lat1 - lat0)
                    crossings.setdefault(r, []).append(lon0 + (lon1 - lon0) * t)
                    rmin, rmax = min(rmin, r), max(rmax, r)
        for r, xs in crossings.items():
            xs.sort()
            for i in range(0, len(xs) - 1, 2):
                a, b = xs[i], xs[i + 1]
                if b < lon_min or a > lon_max:
                    continue
                c0 = max(0, int(math.ceil(lon_to_col(max(a, lon_min)) - 0.5)))
                c1 = min(cols - 1, int(math.floor(lon_to_col(min(b, lon_max)) - 0.5)))
                base = r * cols
                for c in range(c0, c1 + 1):
                    if mask[base + c]:
                        mask[base + c] = 0
                        carved += 1
    return carved


# ---------------------------------------------------------------- main

def main():
    os.makedirs(OUT, exist_ok=True)
    os.makedirs(PROOF, exist_ok=True)

    # ---- fetch the archives (forever-cache) ----
    cusp_zip = os.path.join(CUSP_DIR, "North_Atlantic.zip")
    fetch(CUSP_URL, cusp_zip, 150_000_000)
    gpkgs = []
    for hu in NHD_HU4:
        z = os.path.join(NHD_DIR, f"NHD_H_{hu}_HU4_GPKG.zip")
        fetch(NHD_URL.format(hu=hu), z, 30_000_000)
        # extract the .gpkg beside the zip (sqlite needs a real file); keep both forever
        gname = None
        with zipfile.ZipFile(z) as zf:
            for n in zf.namelist():
                if n.lower().endswith(".gpkg"):
                    gname = os.path.basename(n)
                    dst = os.path.join(NHD_DIR, gname)
                    if not os.path.exists(dst) or os.path.getsize(dst) < zf.getinfo(n).file_size:
                        print(f"[survey] extracting {gname}")
                        with zf.open(n) as src, open(dst, "wb") as f:
                            while True:
                                chunk = src.read(1 << 22)
                                if not chunk:
                                    break
                                f.write(chunk)
                    gpkgs.append(dst)
        if gname is None:
            print(f"[survey] FATAL: no .gpkg inside {z}")
            sys.exit(1)

    # ---- CUSP -> polylines ----
    cusp = clip_runs(read_cusp_zip(cusp_zip, FOCUS), FOCUS)
    harvest_gis.write_bin(os.path.join(OUT, "cusp_ne.bin"), cusp)

    # ---- NHD -> rivers + water polygons ----
    rivers, water = [], []
    for g in gpkgs:
        r, w = read_nhd_gpkg(g, FOCUS)
        print(f"[survey] {os.path.basename(g)}: {len(r)} river lines, {len(w)} water polys")
        rivers += r
        water += w
    harvest_gis.write_bin(os.path.join(OUT, "nhd_rivers_ne.bin"), clip_runs(rivers, FOCUS))
    rings_flat = []
    attrs = []
    for fc, rings in water:
        for ring in rings:
            if len(ring) >= 3:
                rings_flat.append(ring)
                attrs.append({"fcode": fc})
    harvest_gis.write_bin(os.path.join(OUT, "nhd_water_ne.bin"), rings_flat)
    json.dump(attrs, open(os.path.join(OUT, "nhd_water_ne.attrs.json"), "w"), indent=0)

    # ---- the mask: GSHHG land base + NHD open-water carve, in the shared z14 frame ----
    print("[survey] baking the survey land mask (GSHHG base + NHD carve, 4096^2 z14 frame)")
    harvest_gis.fetch_zip()
    zf = zipfile.ZipFile(harvest_gis.ZIP_PATH)
    names = {os.path.basename(n): n for n in zf.namelist()}
    ne = harvest_gis.NE
    coast_f = harvest_gis.parse_gshhg(zf.read(names["gshhs_f.b"]), ne["lon0"], ne["lon1"],
                                      ne["lat0"], ne["lat1"])

    def win_row_lat(r):
        y = (harvest_gis.WIN_ORG_Y + (r + 0.5) * harvest_gis.WIN_SIZE / 4096.0) / harvest_gis.N14
        return math.degrees(math.atan(math.sinh(math.pi * (1.0 - 2.0 * y))))

    def win_lon_col(lon):
        return (((lon + 180.0) / 360.0 * harvest_gis.N14 - harvest_gis.WIN_ORG_X)
                * 4096.0 / harvest_gis.WIN_SIZE)

    lon0 = harvest_gis.WIN_ORG_X / harvest_gis.N14 * 360.0 - 180.0
    lon1 = (harvest_gis.WIN_ORG_X + harvest_gis.WIN_SIZE) / harvest_gis.N14 * 360.0 - 180.0
    mask = bytearray(harvest_gis.bake_mask(coast_f, 4096, 4096, win_row_lat, win_lon_col,
                                           lon0, lon1))
    before = sum(1 for b in mask if b)
    carved = carve_water(mask, 4096, 4096, water, win_row_lat, win_lon_col, lon0, lon1)
    after = sum(1 for b in mask if b)
    print(f"[survey] carve: {before} -> {after} land texels ({carved} cleared to water)")
    with open(os.path.join(OUT, "landmask_ne.raw"), "wb") as f:
        f.write(mask)

    # ---- proof + provenance ----
    try:
        from PIL import Image
        img = Image.frombytes("L", (4096, 4096), bytes(mask)).resize((1024, 1024))
        img.save(os.path.join(PROOF, "survey_mask.png"))
        print(f"[survey] proof -> {PROOF}/survey_mask.png")
    except ImportError:
        pass
    json.dump({
        "cusp": {"url": CUSP_URL, "cache": cusp_zip, "crs": "NAD83 (~1 m vs WGS84, "
                 "documented not corrected)", "datum": "Mean High Water",
                 "polylines_ne": len(cusp)},
        "nhd": {"hu4": NHD_HU4, "url_pattern": NHD_URL, "cache_dir": NHD_DIR,
                "crs": "NAD83 EPSG:4269", "rivers_ne": len(rivers),
                "water_polys_ne": len(water)},
        "focus_box": FOCUS,
        "landmask": {"base": "GSHHG full-res parity (harvest_gis)",
                     "carve": "NHD open-water even-odd per feature",
                     "land_texels_before": before, "land_texels_after": after},
        "open_water_fcodes": "390xx 436xx 49300 460xx 44500 336xx 43100 "
                             "(marsh/wetland EXCLUDED: the live tide owns that call)",
    }, open(os.path.join(OUT, "survey.json"), "w"), indent=1)
    print("[survey] done -> data/gis/survey.json")


if __name__ == "__main__":
    main()
