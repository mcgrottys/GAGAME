"""GAGAME Harvester, M6: the planet.

Two fetches, both polite:

1. ETOPO 2022 global relief (NOAA NCEI): the 60 arc-second ice-surface GeoTIFF, ~450 MB,
   ONE download cached forever, decoded by our stdlib reader and box-decimated to an
   8192 x 4096 equirectangular int16 grid (~5 km/texel at the equator) -- the "low to
   medium-low global view" the project charter asks for. GEBCO 15" tiles slot in later
   when a milestone actually needs sub-km global relief.

2. The LIVE global sea: GFS-Wave significant height + 10 m wind from the latest cycle's
   f000 global 0.25-degree grid (~3 MB via the NOMADS filter, cached per cycle), so the
   globe renders TODAY'S ocean, not a texture.

Usage:  py -3 harvester\\harvest_globe.py [--skip-etopo] [--skip-waves]
Output: data/globe/globe.json, etopo_8192.i16, hs.f32, wind.f32
"""
import json
import os
import struct
import sys
import time
import urllib.request
from array import array
from datetime import datetime, timedelta, timezone

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from geotiff import GeoTiff           # noqa: E402
from grib2 import read_messages       # noqa: E402

UA = "GAGAME/0.1 (hobby ocean simulator; contact: marksmcgrotty@gmail.com)"
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
CACHE = os.path.join(ROOT, "cache", "globe")
OUT = os.path.join(ROOT, "data", "globe")

ETOPO_URLS = [
    # NCEI's directory really is spelled "gtif" (verified 2026-08-28).
    "https://www.ngdc.noaa.gov/mgg/global/relief/ETOPO2022/data/60s/"
    "60s_surface_elev_gtif/ETOPO_2022_v1_60s_N90W180_surface.tif",
    "https://www.ngdc.noaa.gov/thredds/fileServer/global/ETOPO2022/60s/"
    "60s_surface_elev_gtif/ETOPO_2022_v1_60s_N90W180_surface.tif",
]

ONX, ONY = 8192, 4096   # output equirect dims


def log(msg):
    print(msg, flush=True)


def fetch_big(urls, path):
    """Streamed download with progress; any cached file wins."""
    if os.path.exists(path) and os.path.getsize(path) > 1 << 20:
        log(f"  cached: {path} ({os.path.getsize(path) >> 20} MB)")
        return
    os.makedirs(os.path.dirname(path), exist_ok=True)
    last_err = None
    for url in urls:
        try:
            log(f"  GET {url}")
            req = urllib.request.Request(url, headers={"User-Agent": UA})
            with urllib.request.urlopen(req, timeout=120) as r, open(path + ".part", "wb") as f:
                total = int(r.headers.get("Content-Length", "0"))
                done = 0
                t0 = time.time()
                while True:
                    chunk = r.read(1 << 20)
                    if not chunk:
                        break
                    f.write(chunk)
                    done += len(chunk)
                    if done % (64 << 20) < (1 << 20):
                        mb = done >> 20
                        log(f"    {mb} / {total >> 20} MB  ({mb / max(time.time() - t0, 1):.0f} MB/s)")
            os.replace(path + ".part", path)
            log(f"  done: {os.path.getsize(path) >> 20} MB")
            return
        except Exception as e:  # noqa: BLE001
            last_err = e
            log(f"    failed ({e}); trying next mirror")
    raise RuntimeError(f"all ETOPO mirrors failed: {last_err}")


def do_etopo():
    tif_path = os.path.join(CACHE, "ETOPO_2022_v1_60s_surface.tif")
    fetch_big(ETOPO_URLS, tif_path)

    log("  decoding + decimating (one-time; a few minutes of stdlib number crunching)...")
    with open(tif_path, "rb") as f:
        g = GeoTiff(f.read())
    log(f"  source {g.width}x{g.height}, compression {g.compression}, predictor {g.predictor}")

    # ETOPO is gap-free, so decimation is plain box sums -- done with C-speed slice sums per
    # OUTPUT column (2-3 source texels each) instead of a 233M-iteration python loop.
    col_edges = [(ox * g.width) // ONX for ox in range(ONX + 1)]
    acc = array("d", [0.0]) * (ONX * ONY)
    cnt = array("I", [0]) * (ONX * ONY)
    band = 512 if g.tiled else max(1, g.th)
    t0 = time.time()
    for sy0 in range(0, g.height, band):
        rows = min(band, g.height - sy0)
        block = g.read_window(0, sy0, g.width, rows)
        for ry in range(rows):
            oy = ((sy0 + ry) * ONY) // g.height
            base_out = oy * ONX
            base_src = ry * g.width
            for ox in range(ONX):
                c0, c1 = col_edges[ox], col_edges[ox + 1]
                acc[base_out + ox] += sum(block[base_src + c0:base_src + c1])
                cnt[base_out + ox] += c1 - c0
        if (sy0 // band) % 4 == 0:
            pct = 100.0 * (sy0 + rows) / g.height
            log(f"    {pct:5.1f}%  ({time.time() - t0:.0f} s)")

    out = array("h", [0]) * (ONX * ONY)
    for i in range(ONX * ONY):
        if cnt[i]:
            v = acc[i] / cnt[i]
            out[i] = max(-32000, min(32000, int(round(v))))
    os.makedirs(OUT, exist_ok=True)
    with open(os.path.join(OUT, "etopo_8192.i16"), "wb") as f:
        out.tofile(f)
    log(f"  wrote data/globe/etopo_8192.i16 ({ONX}x{ONY} int16, "
        f"{os.path.getsize(os.path.join(OUT, 'etopo_8192.i16')) >> 20} MB)")
    return {"relief_file": "etopo_8192.i16", "nx": ONX, "ny": ONY,
            "lon0": -180.0, "lat0": 90.0, "dlon": 360.0 / ONX, "dlat": -180.0 / ONY,
            "source": "ETOPO 2022 v1 60s ice-surface (NOAA NCEI), box-decimated"}


def do_waves():
    """Latest available GFS-Wave cycle, f000, global 0.25-degree HTSGW + WIND."""
    now = datetime.now(timezone.utc)
    for lag_h in range(5, 30, 6):
        cyc_t = now - timedelta(hours=lag_h)
        ymd = cyc_t.strftime("%Y%m%d")
        cyc = f"{(cyc_t.hour // 6) * 6:02d}"
        # The raw global files are packed with GRIB2 template 5.40 (JPEG2000), which our stdlib
        # reader does not speak -- but the filter RE-ENCODES any subregion request, so asking
        # for the whole world AS a subregion hands back templates it does (verified: 5.0/5.2/5.3).
        url = ("https://nomads.ncep.noaa.gov/cgi-bin/filter_gfswave.pl?"
               f"dir=%2Fgfs.{ymd}%2F{cyc}%2Fwave%2Fgridded&"
               f"file=gfswave.t{cyc}z.global.0p25.f000.grib2&"
               "var_HTSGW=on&var_WIND=on&all_lev=on&"
               "subregion=&toplat=90&bottomlat=-90&leftlon=0&rightlon=360")
        cache = os.path.join(CACHE, f"gfswave_{ymd}_{cyc}_f000_hs_wind.grib2")
        try:
            if not (os.path.exists(cache) and os.path.getsize(cache) > 10000):
                log(f"  GET gfswave {ymd} {cyc}z f000 (global Hs + wind)")
                req = urllib.request.Request(url, headers={"User-Agent": UA})
                with urllib.request.urlopen(req, timeout=120) as r:
                    data = r.read()
                if len(data) < 10000:
                    raise RuntimeError(f"short response ({len(data)} B)")
                os.makedirs(CACHE, exist_ok=True)
                with open(cache, "wb") as f:
                    f.write(data)
            with open(cache, "rb") as f:
                msgs = read_messages(f.read())
            hs = next(m for m in msgs if m.discipline == 10)
            wind = next((m for m in msgs if m.discipline == 0), None)
            log(f"  {ymd} {cyc}z: Hs grid {hs.ni}x{hs.nj}, lat1 {hs.lat1}, lon1 {hs.lon1}, "
                f"scan {hs.scan:#x}; wind {'yes' if wind else 'NO'}")

            os.makedirs(OUT, exist_ok=True)
            for name, msg in (("hs", hs), ("wind", wind)):
                if msg is None:
                    continue
                vals = array("f", [-1.0]) * (msg.ni * msg.nj)
                for i, v in enumerate(msg.values):
                    if v is not None:
                        vals[i] = float(v)
                if msg.scan & 0x40:   # rows stored south->north: normalise to row 0 = NORTH
                    flipped = array("f", [-1.0]) * (msg.ni * msg.nj)
                    for r in range(msg.nj):
                        s = (msg.nj - 1 - r) * msg.ni
                        flipped[r * msg.ni:(r + 1) * msg.ni] = vals[s:s + msg.ni]
                    vals = flipped
                with open(os.path.join(OUT, f"{name}.f32"), "wb") as f:
                    vals.tofile(f)
            lat_top = hs.lat1 + (hs.nj - 1) * hs.dlat if (hs.scan & 0x40) else hs.lat1
            return {"waves_cycle": f"{ymd}t{cyc}z", "waves_nx": hs.ni, "waves_ny": hs.nj,
                    "waves_lat1": lat_top, "waves_lon1": hs.lon1,
                    "waves_dlat": hs.dlat, "waves_dlon": hs.dlon, "waves_scan": 0,
                    "hs_file": "hs.f32", "wind_file": "wind.f32" if wind else None}
        except Exception as e:  # noqa: BLE001
            log(f"    cycle {ymd} {cyc}z unavailable ({e}); stepping back")
    log("  NO wave cycle reachable -- the globe will render relief without a live sea")
    return {}


# The 3D sky: TCDC (cloud fraction, %) on these isobaric levels -- a TRUE volumetric field,
# not a synthetic profile. Altitudes are ISA standard heights for each pressure.
CLOUD_LEVELS = [(1000, 110), (925, 760), (850, 1460), (700, 3010), (600, 4200),
                (500, 5570), (400, 7180), (300, 9160), (250, 10360), (200, 11780)]


def do_clouds():
    """Latest GFS cycle: isobaric cloud fraction, 0.5 deg global, 10 levels (~2.5 MB).
    f003 fallback -- some diagnosed cloud fields only exist past the analysis."""
    now = datetime.now(timezone.utc)
    lev_params = "".join(f"&lev_{mb}_mb=on" for mb, _ in CLOUD_LEVELS)
    for lag_h in range(5, 30, 6):
        cyc_t = now - timedelta(hours=lag_h)
        ymd = cyc_t.strftime("%Y%m%d")
        cyc = f"{(cyc_t.hour // 6) * 6:02d}"
        for fhr in ("f000", "f003"):
            url = ("https://nomads.ncep.noaa.gov/cgi-bin/filter_gfs_0p50.pl?"
                   f"dir=%2Fgfs.{ymd}%2F{cyc}%2Fatmos&"
                   f"file=gfs.t{cyc}z.pgrb2full.0p50.{fhr}&"
                   f"var_TCDC=on{lev_params}&"
                   "subregion=&toplat=90&bottomlat=-90&leftlon=0&rightlon=360")
            cache = os.path.join(CACHE, f"gfs_{ymd}_{cyc}_{fhr}_tcdc_iso.grib2")
            try:
                if not (os.path.exists(cache) and os.path.getsize(cache) > 10000):
                    log(f"  GET gfs {ymd} {cyc}z {fhr} (isobaric cloud fraction)")
                    req = urllib.request.Request(url, headers={"User-Agent": UA})
                    with urllib.request.urlopen(req, timeout=120) as r:
                        data = r.read()
                    if len(data) < 10000:
                        raise RuntimeError(f"short response ({len(data)} B)")
                    os.makedirs(CACHE, exist_ok=True)
                    with open(cache, "wb") as f:
                        f.write(data)
                with open(cache, "rb") as f:
                    msgs = read_messages(f.read())
                # TCDC = (0, 6, 1) on level type 100 (isobaric, value in Pa).
                byLev = {}
                for m in msgs:
                    if m.discipline == 0 and m.param_category == 6 and m.level_type == 100:
                        byLev[int(m.level_value / 100)] = m
                missing = [mb for mb, _ in CLOUD_LEVELS if mb not in byLev]
                if missing:
                    raise RuntimeError(f"levels missing: {missing}")
                g0 = byLev[CLOUD_LEVELS[0][0]]
                log(f"  gfs {ymd} {cyc}z {fhr}: {len(CLOUD_LEVELS)} isobaric cloud levels, "
                    f"grid {g0.ni}x{g0.nj}, lat1 {g0.lat1}, scan {g0.scan:#x}")
                os.makedirs(OUT, exist_ok=True)
                # One packed file: level-major, row 0 = north, values 0..100 (%).
                out = array("f")
                for mb, _ in CLOUD_LEVELS:
                    msg = byLev[mb]
                    vals = array("f", [0.0]) * (msg.ni * msg.nj)
                    for i, v in enumerate(msg.values):
                        if v is not None:
                            vals[i] = float(v)
                    if msg.scan & 0x40:
                        flipped = array("f", [0.0]) * (msg.ni * msg.nj)
                        for r in range(msg.nj):
                            s = (msg.nj - 1 - r) * msg.ni
                            flipped[r * msg.ni:(r + 1) * msg.ni] = vals[s:s + msg.ni]
                        vals = flipped
                    out.extend(vals)
                with open(os.path.join(OUT, "cloud_iso.f32"), "wb") as f:
                    out.tofile(f)
                lat_top = (g0.lat1 + (g0.nj - 1) * g0.dlat) if (g0.scan & 0x40) else g0.lat1
                return {"clouds_cycle": f"{ymd}t{cyc}z{fhr}", "clouds_nx": g0.ni,
                        "clouds_ny": g0.nj, "clouds_nz": len(CLOUD_LEVELS),
                        "clouds_lat1": lat_top, "clouds_lon1": g0.lon1,
                        "clouds_dlat": g0.dlat, "clouds_dlon": g0.dlon,
                        "clouds_alt_m": [alt for _, alt in CLOUD_LEVELS],
                        "clouds_file": "cloud_iso.f32"}
            except Exception as e:  # noqa: BLE001
                log(f"    {ymd} {cyc}z {fhr} unavailable ({e})")
    log("  NO cloud cycle reachable -- the sky stays clear")
    return {}


NE15_URLS = [
    "https://www.ngdc.noaa.gov/mgg/global/relief/ETOPO2022/data/15s/15s_surface_elev_gtif/"
    "ETOPO_2022_v1_15s_N45W075_surface.tif",
]
NE_WIN = {"lon0": -72.0, "lon1": -66.0, "lat0": 40.0, "lat1": 45.0}


def do_ne15():
    """M6d: the New England 15-arc-second window (~460 m) -- the LOD ring between the global
    5 km relief and the estuary's 14 m CUDEM. One ETOPO tile, windowed with the stdlib reader."""
    tif_path = os.path.join(CACHE, "ETOPO_2022_v1_15s_N45W075_surface.tif")
    fetch_big(NE15_URLS, tif_path)
    with open(tif_path, "rb") as f:
        g = GeoTiff(f.read())
    log(f"  15s tile {g.width}x{g.height}, origin ({g.lat0:.2f}, {g.lon0:.2f})")
    x0 = max(g.px_of_lon(NE_WIN["lon0"]), 0)
    x1 = min(g.px_of_lon(NE_WIN["lon1"]), g.width)
    y0 = max(g.py_of_lat(NE_WIN["lat1"]), 0)
    y1 = min(g.py_of_lat(NE_WIN["lat0"]), g.height)
    w, h = x1 - x0, y1 - y0
    vals = g.read_window(x0, y0, w, h)
    out = array("h", [0]) * (w * h)
    for i, v in enumerate(vals):
        out[i] = max(-32000, min(32000, int(round(v)))) if v == v else 0
    os.makedirs(OUT, exist_ok=True)
    with open(os.path.join(OUT, "ne_15s.i16"), "wb") as f:
        out.tofile(f)
    log(f"  wrote data/globe/ne_15s.i16 ({w}x{h} int16, ~460 m)")
    return {"ne_file": "ne_15s.i16", "ne_nx": w, "ne_ny": h,
            "ne_lon0": NE_WIN["lon0"], "ne_lat1": NE_WIN["lat1"],
            "ne_dlon": 15.0 / 3600.0, "ne_dlat": -15.0 / 3600.0}


def do_wind_vec():
    """M6d: the global 10 m wind VECTOR (u, v) -- the source for the sparse Mv2 wind bank
    (divergence scalar + wind vector + curl bivector; resident only where the atmosphere is
    doing something). Same cycle discovery as the clouds."""
    now = datetime.now(timezone.utc)
    for lag_h in range(5, 30, 6):
        cyc_t = now - timedelta(hours=lag_h)
        ymd = cyc_t.strftime("%Y%m%d")
        cyc = f"{(cyc_t.hour // 6) * 6:02d}"
        url = ("https://nomads.ncep.noaa.gov/cgi-bin/filter_gfs_0p50.pl?"
               f"dir=%2Fgfs.{ymd}%2F{cyc}%2Fatmos&"
               f"file=gfs.t{cyc}z.pgrb2full.0p50.f000&"
               "var_UGRD=on&var_VGRD=on&lev_10_m_above_ground=on&"
               "subregion=&toplat=90&bottomlat=-90&leftlon=0&rightlon=360")
        cache = os.path.join(CACHE, f"gfs_{ymd}_{cyc}_f000_uv10.grib2")
        try:
            if not (os.path.exists(cache) and os.path.getsize(cache) > 10000):
                log(f"  GET gfs {ymd} {cyc}z f000 (10 m wind vector)")
                req = urllib.request.Request(url, headers={"User-Agent": UA})
                with urllib.request.urlopen(req, timeout=120) as r:
                    data = r.read()
                if len(data) < 10000:
                    raise RuntimeError(f"short response ({len(data)} B)")
                os.makedirs(CACHE, exist_ok=True)
                with open(cache, "wb") as f:
                    f.write(data)
            with open(cache, "rb") as f:
                msgs = read_messages(f.read())
            u = next(m for m in msgs if m.param_category == 2 and m.param_number == 2)
            v = next(m for m in msgs if m.param_category == 2 and m.param_number == 3)
            log(f"  gfs {ymd} {cyc}z: u/v grid {u.ni}x{u.nj}, scan {u.scan:#x}")
            os.makedirs(OUT, exist_ok=True)
            for name, msg in (("wind_u", u), ("wind_v", v)):
                vals = array("f", [0.0]) * (msg.ni * msg.nj)
                for i, val in enumerate(msg.values):
                    if val is not None:
                        vals[i] = float(val)
                if msg.scan & 0x40:
                    flipped = array("f", [0.0]) * (msg.ni * msg.nj)
                    for r in range(msg.nj):
                        s = (msg.nj - 1 - r) * msg.ni
                        flipped[r * msg.ni:(r + 1) * msg.ni] = vals[s:s + msg.ni]
                    vals = flipped
                with open(os.path.join(OUT, f"{name}.f32"), "wb") as f:
                    vals.tofile(f)
            lat_top = (u.lat1 + (u.nj - 1) * u.dlat) if (u.scan & 0x40) else u.lat1
            return {"windvec_cycle": f"{ymd}t{cyc}z", "windvec_nx": u.ni, "windvec_ny": u.nj,
                    "windvec_lat1": lat_top, "windvec_lon1": u.lon1,
                    "windvec_dlat": u.dlat, "windvec_dlon": u.dlon}
        except Exception as e:  # noqa: BLE001
            log(f"    {ymd} {cyc}z unavailable ({e})")
    return {}


def main():
    meta = {}
    if "--skip-etopo" not in sys.argv:
        log("[etopo] global relief")
        meta.update(do_etopo())
    if "--skip-ne" not in sys.argv:
        log("[ne15] New England 15-arc-second window")
        meta.update(do_ne15())
    if "--skip-windvec" not in sys.argv:
        log("[windvec] global 10 m wind vector")
        meta.update(do_wind_vec())
    if "--skip-waves" not in sys.argv:
        log("[waves] live global sea state")
        meta.update(do_waves())
    if "--skip-clouds" not in sys.argv:
        log("[clouds] live cloud layers")
        meta.update(do_clouds())
    meta["generated_utc"] = datetime.now(timezone.utc).isoformat(timespec="seconds")
    path = os.path.join(OUT, "globe.json")
    old = {}
    if os.path.exists(path):
        with open(path, "r", encoding="utf-8") as f:
            old = json.load(f)
    old.update(meta)
    os.makedirs(OUT, exist_ok=True)
    with open(path, "w", encoding="utf-8") as f:
        json.dump(old, f, indent=1)
    log(f"wrote {path}")


if __name__ == "__main__":
    main()
