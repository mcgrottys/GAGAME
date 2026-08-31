# ==================================================================================================
#  harvest_refbathy.py - ingest the vqview-inlet 1.5 m bathymetry composite as a GAGAME height rung.
#
#  Source: C:\vqview-inlet\vqview-inlet\renders\geotiff\bathy_comp2.npy - the user's own
#  USACE eHydro + NOAA BlueTopo + CUDEM composite for the Merrimack inlet mouth
#  (1851 x 1106 @ 1.5 m, vertical datum MLLW, row 0 = north; georef per ortho_waterline.py:
#  lon -70.8320..-70.7980, lat 42.8250(N)..42.8100(S)).  Its cells are ~9x finer than
#  noaa.cudem.merrimack's 13.7 m plane, and it carries the surveyed channel/jetty/bar
#  structure (dredge lines, scour, sand waves) that the wave-field solve and the bed
#  albedo's bedform relief actually need.  A fresh eHydro/BlueTopo harvester is the named
#  upgrade; this is the proven first rung (the composite was waterline-audited at 93.2%
#  agreement against MassGIS orthos in its home project).
#
#  DATUM (the whole job of this script): the composite is MLLW; the engine's height channel
#  speaks NAVD88 (M6w, one bed).  Convert AT HARVEST: elev_navd = elev_mllw + (MLLW - NAVD88).
#  The offset is derived from data/tides/stations.json exactly like main.cpp ResolveDatum:
#  the entrance station (8440452) publishes no NAVD link, so transfer via NAVD ~= MSL + delta
#  with delta calibrated at the nearest station carrying both (Boston: delta = +0.092 m):
#      MLLW - NAVD88  =  -(mean_mllw + delta)  ~=  -(1.308 + 0.092)  =  -1.400 m
#  vqview's own ortho_waterline audit found a +1.00 m correction had been fitted into the
#  composite's history - so the engine-side acceptance gate is the SAME waterline test run
#  against our MassGIS orthos + live tide (the M7 water campaign's bed leg).  --offset lets
#  that gate feed a residual correction back in; the applied value is documented in the json.
#
#  Identity: the sidecar carries an FNV-1a hash of the raw npy bytes; the engine source bakes
#  it into its structure string (the M6w stale-cache law - content change = repaint).
#
#  Output: data/bathy/inlet15.f32 + inlet15.json (BathyModel contract: little-endian float32,
#  row-major, row 0 = north, nodata -9999.0, metres NAVD88) + inlet15_preview.png.
#  Raw source copied once to D:\DataCache\GAGAME\refbathy\ for provenance.
# ==================================================================================================
import json
import os
import shutil
import struct
import sys
import time

VQVIEW_NPY = os.environ.get(
    "REFBATHY", r"C:\vqview-inlet\vqview-inlet\renders\geotiff\bathy_comp2.npy")
DCACHE = r"D:\DataCache\GAGAME"
OUT_DIR = os.path.join("data", "bathy")
NAME = "inlet15"

LON0, LAT1 = -70.8320, 42.8250     # west edge, NORTH edge (row 0)
LON1, LAT0 = -70.7980, 42.8100
NODATA = -9999.0


def fnv1a(data):
    h = 0xcbf29ce484222325
    for b in data:
        h ^= b
        h = (h * 0x100000001b3) & 0xFFFFFFFFFFFFFFFF
    return h


def load_npy(path):
    """Minimal .npy reader (v1.0/2.0, little-endian float32/float64, C order)."""
    with open(path, "rb") as f:
        magic = f.read(6)
        if magic != b"\x93NUMPY":
            raise ValueError("not a .npy file")
        major, minor = f.read(2)
        if major == 1:
            hlen = struct.unpack("<H", f.read(2))[0]
        else:
            hlen = struct.unpack("<I", f.read(4))[0]
        hdr = eval(f.read(hlen).decode("latin1"),
                   {"__builtins__": {}}, {"False": False, "True": True})
        if hdr["fortran_order"]:
            raise ValueError("fortran order unsupported")
        dt = hdr["descr"]
        ny, nx = hdr["shape"]
        raw = f.read()
    import array
    if dt in ("<f4", "|f4", "f4"):
        a = array.array("f")
        a.frombytes(raw[: ny * nx * 4])
    elif dt in ("<f8", "|f8", "f8"):
        d = array.array("d")
        d.frombytes(raw[: ny * nx * 8])
        a = array.array("f", d)
    else:
        raise ValueError("dtype %s unsupported" % dt)
    return a, nx, ny


def resolve_mllw_minus_navd():
    """The main.cpp ResolveDatum recipe: focus station's own link, else MSL transfer."""
    p = os.path.join("data", "tides", "stations.json")
    try:
        d = json.load(open(p))
    except OSError:
        print("WARNING: %s missing; using -1.400 m (Boston-transfer value)" % p)
        return -1.400, "fallback constant (stations.json absent)"
    sta = {s["id"]: s for s in d["stations"]}
    focus = sta.get("8440452") or sta.get("8440466")
    if focus and focus.get("mllw_minus_navd_m") is not None:
        return float(focus["mllw_minus_navd_m"]), "station %s published link" % focus["id"]
    best = None
    for s in d["stations"]:
        mn = s.get("mllw_minus_navd_m")
        mm = s.get("mean_mllw_m")
        if mn is None or mm is None:
            continue
        delta = -mn - mm            # NAVD - MSL at this station
        if best is None or abs(delta) < abs(best[0]):
            best = (delta, s["id"])
    if best is None:
        print("WARNING: no station carries a NAVD link; using -1.400 m")
        return -1.400, "fallback constant (no NAVD links)"
    delta, via = best
    mm = float(focus["mean_mllw_m"])
    off = -(mm + delta)
    return off, "MSL transfer via %s (delta %+0.3f, focus mean_mllw %.3f)" % (via, delta, mm)


def main():
    offset_extra = 0.0
    for i, a in enumerate(sys.argv):
        if a == "--offset" and i + 1 < len(sys.argv):
            offset_extra = float(sys.argv[i + 1])

    if not os.path.isfile(VQVIEW_NPY):
        print("source not found: %s" % VQVIEW_NPY)
        return 1
    raw_bytes = open(VQVIEW_NPY, "rb").read()
    content = fnv1a(raw_bytes)

    if os.path.isdir(DCACHE):
        keep = os.path.join(DCACHE, "refbathy")
        os.makedirs(keep, exist_ok=True)
        dst = os.path.join(keep, "bathy_comp2.npy")
        if not os.path.exists(dst):
            shutil.copyfile(VQVIEW_NPY, dst)
            print("provenance copy -> %s" % dst)

    grid, nx, ny = load_npy(VQVIEW_NPY)
    mllw_minus_navd, how = resolve_mllw_minus_navd()
    off = mllw_minus_navd + offset_extra
    print("grid %dx%d  MLLW->NAVD88 offset %+.3f m (%s)%s" % (
        nx, ny, off, how,
        "  + residual %+.3f" % offset_extra if offset_extra else ""))

    vmin, vmax = 1e9, -1e9
    n_nodata = 0
    for i, v in enumerate(grid):
        if v != v or v < -9000.0:          # NaN or already-sentinel
            grid[i] = NODATA
            n_nodata += 1
        else:
            v = v + off
            grid[i] = v
            vmin = min(vmin, v)
            vmax = max(vmax, v)

    os.makedirs(OUT_DIR, exist_ok=True)
    f32 = os.path.join(OUT_DIR, NAME + ".f32")
    if sys.byteorder != "little":
        grid.byteswap()
    open(f32, "wb").write(grid.tobytes())

    meta = {
        "file": NAME + ".f32",
        "nx": nx, "ny": ny,
        "lon0": LON0, "lat1": LAT1,
        "dlon": (LON1 - LON0) / nx, "dlat": (LAT1 - LAT0) / ny,
        "row0": "north", "nodata": NODATA,
        "min_m": round(vmin, 3), "max_m": round(vmax, 3),
        "source": ("vqview-inlet bathy_comp2.npy (USACE eHydro + NOAA BlueTopo + CUDEM "
                   "composite, 1.5 m, MLLW) shifted %+.3f m to NAVD88 (%s); "
                   "content fnv1a %016x" % (off, how, content)),
        "content_fnv64": "%016x" % content,
        "datum_offset_applied_m": round(off, 4),
        "nodata_cells": n_nodata,
        "generated_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
    }
    json.dump(meta, open(os.path.join(OUT_DIR, NAME + ".json"), "w"), indent=1)
    print("wrote %s (%d x %d, %.2f..%.2f m NAVD88, %d nodata)" % (
        f32, nx, ny, vmin, vmax, n_nodata))

    # stdlib grayscale preview (harvest_bathy tradition)
    try:
        import zlib
        w, h = nx, ny
        span = max(vmax - vmin, 1e-6)
        rows = bytearray()
        for r in range(h):
            rows.append(0)
            base = r * w
            for c in range(w):
                v = grid[base + c]
                rows.append(0 if v == NODATA else
                            max(0, min(255, int((v - vmin) / span * 255))))
        def chunk(tag, data):
            out = struct.pack(">I", len(data)) + tag + data
            return out + struct.pack(">I", zlib.crc32(tag + data) & 0xFFFFFFFF)
        png = (b"\x89PNG\r\n\x1a\n"
               + chunk(b"IHDR", struct.pack(">IIBBBBB", w, h, 8, 0, 0, 0, 0))
               + chunk(b"IDAT", zlib.compress(bytes(rows), 6))
               + chunk(b"IEND", b""))
        open(os.path.join(OUT_DIR, NAME + "_preview.png"), "wb").write(png)
        print("preview -> %s" % os.path.join(OUT_DIR, NAME + "_preview.png"))
    except Exception as e:
        print("preview skipped: %s" % e)
    return 0


if __name__ == "__main__":
    sys.exit(main())
