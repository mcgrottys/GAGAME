#!/usr/bin/env python3
"""GAGAME Harvester, M6v: THE WATER ATLAS registry -- heterogeneous water data, one survey.

Builds data/water/registry.json: every water-parameter source near the focus region
(Merrimack / Gloucester / Boston), each declaring WHAT it measures, WHERE it is, in WHICH
datum, and at WHAT quality -- the same schema discipline the texture registry enforces.
Heterogeneity is the point: harmonic tide stations (rich), current-prediction stations,
buoys (spot obs + spectra), river gauges (discharge), and the global EOT20 constituent
atlas (medium-quality base the New England stations enhance).

  1. CO-OPS station DISCOVERY: the full tide + current station lists (two cached fetches),
     filtered to the region box. Verified IDs stay pinned; discovery adds what we missed
     (Gloucester-area subordinates, Boston Harbor currents).
  2. NDBC buoy metadata from station_table.txt (one cached fetch): 44013 / 44029 / 44098.
  3. USGS river gauges: 01100000 Lowell, 01100500 Lawrence (live-verified 2026-08-28).
  4. Points -> cache/vectors/water_points.geojson so harvest_vectors folds every station
     into vectors.vpack (kind=points + attrs sidecar): single-point measurements enter the
     SURVEY ORDER like any other vector data.
  5. --eot20: one-time 2 GB download (SEANOE 85762.zip, CC-BY 4.0 -- attribution
     Hart-Davis et al. 2021) into D:/DataCache/GAGAME, then M2/S2/N2/K1/O1 ocean-tide
     grids converted to flat float32 (re, im) rasters in data/water/ for the engine.
     1/8 degree, 66S..66N -- the global LOW-MEDIUM base of the water.tide channels.

POLITENESS: everything cached forever; discovery lists are two requests; the EOT20 zip is
one request ever, stored on the user-granted big-data drive.
"""

import argparse
import json
import math
import os
import sys
import time
import urllib.parse
import urllib.request

UA = "GAGAME/0.1 (hobby ocean simulator; contact: marksmcgrotty@gmail.com)"
MDAPI = "https://api.tidesandcurrents.noaa.gov/mdapi/prod/webapi"
EOT20_URL = "https://www.seanoe.org/data/00683/79489/data/85762.zip"

BIGCACHE = r"D:\DataCache\GAGAME"
CACHE = os.path.join("cache", "coops")
OUT = os.path.join("data", "water")

# The focus box: Merrimack mouth to Boston Harbor, Cape Ann included.
BOX = {"lat0": 42.20, "lat1": 43.05, "lon0": -71.15, "lon1": -70.35}

# Live-verified anchors (2026-08-28). Discovery ADDS; it never removes these.
TIDE_PINNED = ["8440452", "8440466", "8440273", "8440369", "8440889", "8443970"]
CURRENT_PINNED = ["ACT0816", "ACT0821", "ACT0826", "ACT0831"]
BUOYS = ["44013", "44029", "44098"]
RIVER_GAUGES = [
    {"id": "01100000", "name": "Merrimack R at Lowell", "lat": 42.6459, "lon": -71.2981,
     "parameter": "discharge_cms", "api": "waterservices.usgs.gov (migrate to "
     "api.waterdata.usgs.gov by Q1 2027)"},
    {"id": "01100500", "name": "Merrimack R at Lawrence", "lat": 42.7028, "lon": -71.1651,
     "parameter": "discharge_cms", "api": "same"},
]

_last = [0.0]


def log(m):
    print(m, flush=True)


def fetch(url, cache_path, binary=False):
    if os.path.exists(cache_path):
        with open(cache_path, "rb" if binary else "r",
                  encoding=None if binary else "utf-8") as f:
            return f.read(), True
    os.makedirs(os.path.dirname(cache_path), exist_ok=True)
    wait = 0.5 - (time.monotonic() - _last[0])
    if wait > 0:
        time.sleep(wait)
    req = urllib.request.Request(url, headers={"User-Agent": UA})
    with urllib.request.urlopen(req, timeout=60) as r:
        data = r.read()
    _last[0] = time.monotonic()
    with open(cache_path, "wb") as f:
        f.write(data)
    if not binary:
        data = data.decode("utf-8")
    return data, False


def in_box(lat, lon):
    return BOX["lat0"] <= lat <= BOX["lat1"] and BOX["lon0"] <= lon <= BOX["lon1"]


def discover_tide_stations():
    """The full CO-OPS tide-prediction station list (one cached fetch), region-filtered."""
    raw, cached = fetch(f"{MDAPI}/stations.json?type=tidepredictions",
                        os.path.join(CACHE, "_lists", "tidepredictions.json"))
    data = json.loads(raw)
    out = []
    for st in data.get("stations", []):
        lat, lon = float(st.get("lat", 0)), float(st.get("lng", 0))
        if not in_box(lat, lon):
            continue
        out.append({"id": st["id"], "name": st.get("name", "?"), "lat": lat, "lon": lon,
                    "type": st.get("type", "?")})   # R = reference (harmonic), S = subordinate
    log(f"[water] tide stations in box: {len(out)} ({'cache' if cached else 'fetched'})")
    return out


def discover_current_stations():
    raw, cached = fetch(f"{MDAPI}/stations.json?type=currentpredictions",
                        os.path.join(CACHE, "_lists", "currentpredictions.json"))
    data = json.loads(raw)
    out = []
    for st in data.get("stations", []):
        lat, lon = float(st.get("lat", 0)), float(st.get("lng", 0))
        if not in_box(lat, lon):
            continue
        cid = st.get("currbin") and f"{st['id']}_{st['currbin']}" or st["id"]
        out.append({"id": st["id"], "bin": st.get("currbin"), "name": st.get("name", "?"),
                    "lat": lat, "lon": lon, "type": st.get("type", "?")})
    log(f"[water] current stations in box: {len(out)} ({'cache' if cached else 'fetched'})")
    return out


def buoy_positions():
    """NDBC master station table (one cached fetch): lon/lat + owner for the pinned buoys."""
    raw, cached = fetch("https://www.ndbc.noaa.gov/data/stations/station_table.txt",
                        os.path.join(CACHE, "_lists", "ndbc_station_table.txt"))
    out = []
    for line in raw.splitlines():
        if line.startswith("#") or "|" not in line:
            continue
        f = [c.strip() for c in line.split("|")]
        if f[0].lower() not in [b.lower() for b in BUOYS]:
            continue
        # location column like '42.346 N 70.651 W ...'
        loc = f[6].split()
        try:
            lat = float(loc[0]) * (1 if loc[1] == "N" else -1)
            lon = float(loc[2]) * (1 if loc[3] == "E" else -1)
        except (IndexError, ValueError):
            continue
        out.append({"id": f[0], "name": f[4] or f[0], "lat": lat, "lon": lon,
                    "payload": f[1]})
    log(f"[water] buoys located: {[b['id'] for b in out]} ({'cache' if cached else 'fetched'})")
    return out


def convert_eot20(constituents):
    """Download (once, to the big-data drive) + convert EOT20 ocean-tide grids to flat
    float32 (re, im) rasters the engine memory-maps. CC-BY 4.0: Hart-Davis et al. (2021),
    doi:10.17882/79489."""
    os.makedirs(BIGCACHE, exist_ok=True)
    zpath = os.path.join(BIGCACHE, "eot20_85762.zip")
    if not os.path.exists(zpath):
        # curl: Windows cert store + resume (-C -) beat urllib for a 2 GB pull (python's
        # bundled certs rejected seanoe's chain).
        import subprocess
        log(f"[eot20] downloading {EOT20_URL} (~2 GB, once) -> {zpath}")
        tmp = zpath + ".part"
        r = subprocess.run(["curl", "-L", "--fail", "--retry", "3", "-C", "-",
                            "-A", UA, "-o", tmp, EOT20_URL])
        if r.returncode != 0 or not os.path.exists(tmp):
            raise RuntimeError(f"curl failed ({r.returncode})")
        os.replace(tmp, zpath)
        log(f"[eot20] downloaded {os.path.getsize(zpath) / 1e9:.2f} GB")
    else:
        log(f"[eot20] zip cached ({os.path.getsize(zpath) / 1e9:.2f} GB)")

    import io
    import zipfile
    import numpy as np
    import netCDF4

    os.makedirs(OUT, exist_ok=True)
    outer = zipfile.ZipFile(zpath)
    # The SEANOE deposit nests ocean_tides.zip / load_tides.zip inside the download.
    if "ocean_tides.zip" in outer.namelist():
        inner = os.path.join(BIGCACHE, "ocean_tides.zip")
        if not os.path.exists(inner):
            log("[eot20] extracting nested ocean_tides.zip")
            with open(inner, "wb") as f:
                f.write(outer.read("ocean_tides.zip"))
        z = zipfile.ZipFile(inner)
    else:
        z = outer
    if True:
        names = z.namelist()
        for con in constituents:
            outp = os.path.join(OUT, f"eot20_{con}.rg32")
            if os.path.exists(outp):
                log(f"[eot20] {con}: already converted")
                continue
            cand = [n for n in names
                    if n.lower().endswith(f"/{con.lower()}_ocean_eot20.nc")
                    or n.lower().endswith(f"ocean_tides/{con.lower()}.nc")]
            if not cand:
                cand = [n for n in names if "ocean" in n.lower()
                        and n.lower().split("/")[-1].startswith(con.lower())
                        and n.endswith(".nc")]
            if not cand:
                log(f"[eot20] {con}: no matching nc in zip (names like: "
                    f"{[n for n in names if n.endswith('.nc')][:3]})")
                continue
            member = cand[0]
            log(f"[eot20] {con}: extracting {member}")
            ncbytes = z.read(member)
            ds = netCDF4.Dataset("inmem", memory=ncbytes)
            lat = ds.variables["lat"][:]
            lon = ds.variables["lon"][:]
            units = str(getattr(ds.variables["real"], "units", "")).strip().lower()
            re = np.ma.filled(ds.variables["real"][:], np.nan).astype(np.float32)
            im = np.ma.filled(ds.variables["imag"][:], np.nan).astype(np.float32)
            ds.close()
            # Normalize: row 0 = NORTH, lon ascending from lon[0].
            if lat[0] < lat[-1]:
                re, im = re[::-1], im[::-1]
                lat = lat[::-1]
            # M8i UNITS BY DECLARATION, never by guess: the old >30 heuristic read
            # "centimetres" off the big constituents by luck and left the small ones
            # (T2 max 27 cm) unscaled -- a silent x100. Every EOT20 file carries
            # units="cm"; the fallback heuristic survives only for a file without the
            # attribute.
            if units.startswith("cm") or units.startswith("centimet"):
                scale = 0.01
            elif units in ("m", "meter", "meters", "metre", "metres"):
                scale = 1.0
            else:
                scale = 0.01 if np.nanmax(np.hypot(re, im)) > 30.0 else 1.0
                log(f"[eot20] {con}: WARNING no units attribute, heuristic scale {scale}")
            re = re * scale
            im = im * scale
            # M8i OUTLIER CLAMP: EOT20 carries a handful of blown-up near-coast texels
            # (T2's global max is 26.9 m against a 99.9th percentile of 5 cm). Clamp
            # amplitude to 4x the 99.9th percentile, PHASE-PRESERVING, and say so --
            # genuine resonance survives (M2 Fundy 4.97 m vs cap ~8.9 m).
            amp = np.hypot(re, im)
            p999 = float(np.nanpercentile(amp, 99.9))
            cap = max(4.0 * p999, 0.02)
            hot = amp > cap
            nHot = int(np.count_nonzero(hot & np.isfinite(amp)))
            if nHot:
                shrink = np.where(hot, cap / np.maximum(amp, 1e-12), 1.0)
                re = re * shrink
                im = im * shrink
                log(f"[eot20] {con}: clamped {nHot} outlier texels to {cap:.3f} m "
                    f"(p99.9 {p999:.3f} m)")
            rg = np.empty((re.shape[0], re.shape[1], 2), np.float32)
            rg[:, :, 0] = np.nan_to_num(re, nan=0.0)
            rg[:, :, 1] = np.nan_to_num(im, nan=0.0)
            rg.tofile(outp)
            meta = {"file": os.path.basename(outp), "rows": int(re.shape[0]),
                    "cols": int(re.shape[1]), "lat_north": float(lat[0]),
                    "lat_south": float(lat[-1]), "lon_first": float(lon[0]),
                    "lon_last": float(lon[-1]), "units": "m (re, im interleaved)",
                    "row0": "north", "nodata": "0 (land/out-of-domain)",
                    "source": "EOT20 (Hart-Davis et al. 2021, doi:10.17882/79489, CC-BY 4.0)",
                    "phase_convention": "Greenwich lag: h = re*cos(w*t_astro) + im*sin; "
                    "the ENGINE re-references to the fit epoch via the station consensus "
                    "(the epoch ladder -- see WaterAtlas)"}
            with open(os.path.join(OUT, f"eot20_{con}.json"), "w") as f:
                json.dump(meta, f, indent=1)
            log(f"[eot20] {con}: {re.shape[1]}x{re.shape[0]} -> {outp} "
                f"(max amp {np.nanmax(np.hypot(rg[:, :, 0], rg[:, :, 1])):.2f} m)")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--eot20", action="store_true",
                    help="download + convert the EOT20 global constituent atlas (2 GB, once)")
    args = ap.parse_args()

    os.makedirs(OUT, exist_ok=True)
    tide = discover_tide_stations()
    curr = discover_current_stations()
    buoys = buoy_positions()

    for t in tide:
        t["pinned"] = t["id"] in TIDE_PINNED
        t["harmonic"] = t.get("type") == "R"
    reg = {
        "_readme": [
            "THE WATER SURVEY: every water-parameter source near the focus region, each",
            "declaring position (WGS84), datum, and what it measures. Heterogeneity is",
            "explicit: harmonic stations carry 37-constituent fits (data/tides), currents",
            "carry prediction clocks, buoys carry spot obs + spectra, gauges carry",
            "discharge, EOT20 carries the global low-medium constituent base.",
        ],
        "box": BOX,
        "tide_stations": tide,
        "current_stations": curr,
        "buoys": buoys,
        "river_gauges": RIVER_GAUGES,
        "global_base": {
            "name": "eot20", "crs": "EPSG:4326 equirect 1/8 deg", "coverage": "66S..66N",
            "constituents_converted": sorted(
                c[6:-5] for c in os.listdir(OUT)
                if c.startswith("eot20_") and c.endswith(".rg32")) if os.path.isdir(OUT) else [],
            "license": "CC-BY 4.0 (Hart-Davis et al. 2021, doi:10.17882/79489)",
        },
    }
    with open(os.path.join(OUT, "registry.json"), "w") as f:
        json.dump(reg, f, indent=1)
    log(f"[water] registry: {len(tide)} tide, {len(curr)} current, {len(buoys)} buoys, "
        f"{len(RIVER_GAUGES)} gauges -> data/water/registry.json")

    # Points into the survey order: one GeoJSON, ingested by harvest_vectors as a vpack
    # points layer with attrs (kind, id, name -- queryable like any survey layer).
    feats = []
    for t in tide:
        feats.append({"type": "Feature", "properties": {
            "kind": "tide", "id": t["id"], "name": t["name"],
            "harmonic": t["harmonic"]},
            "geometry": {"type": "Point", "coordinates": [t["lon"], t["lat"]]}})
    for c in curr:
        feats.append({"type": "Feature", "properties": {
            "kind": "current", "id": c["id"], "name": c["name"]},
            "geometry": {"type": "Point", "coordinates": [c["lon"], c["lat"]]}})
    for b in buoys:
        feats.append({"type": "Feature", "properties": {
            "kind": "buoy", "id": b["id"], "name": b["name"]},
            "geometry": {"type": "Point", "coordinates": [b["lon"], b["lat"]]}})
    for g in RIVER_GAUGES:
        feats.append({"type": "Feature", "properties": {
            "kind": "river", "id": g["id"], "name": g["name"]},
            "geometry": {"type": "Point", "coordinates": [g["lon"], g["lat"]]}})
    os.makedirs(os.path.join("cache", "vectors"), exist_ok=True)
    with open(os.path.join("cache", "vectors", "water_points.geojson"), "w") as f:
        json.dump({"type": "FeatureCollection", "features": feats}, f, indent=0)
    log(f"[water] {len(feats)} survey points -> cache/vectors/water_points.geojson "
        "(rerun harvest_vectors to fold into the vpack)")

    # Discovered harmonic stations not yet fitted: tell harvest_tides.
    known = set(TIDE_PINNED)
    extra = [t for t in tide if t["harmonic"] and t["id"] not in known]
    if extra:
        with open(os.path.join(OUT, "extra_tide_stations.json"), "w") as f:
            json.dump([{"id": t["id"], "name": t["name"]} for t in extra], f, indent=1)
        log(f"[water] {len(extra)} harmonic stations need M1 fits: "
            f"{[t['id'] + ' ' + t['name'] for t in extra]}")
        log("[water]   -> harvest_tides.py picks up data/water/extra_tide_stations.json")

    if args.eot20:
        # M8i: the widened set -- every EOT20 constituent the atlas consumes. (EOT20's 17
        # minus MF/MM/S1, which are either sub-mm here or radiational-contaminated; L2,
        # NU2, MU2, M6 have no EOT20 grid and ride the station rung only.)
        convert_eot20(["M2", "S2", "N2", "K1", "O1",
                       "K2", "P1", "Q1", "2N2", "T2", "J1", "M4", "SA", "SSA"])
    else:
        log("[water] run with --eot20 for the 2 GB global constituent base (once)")


if __name__ == "__main__":
    main()
