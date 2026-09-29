#!/usr/bin/env python3
"""GAGAME Harvester, M3: currents.

Three sources, in descending order of certainty:

  1. CO-OPS tidal current PREDICTIONS for the Merrimack entrance stations (ACT0816 Merrimack
     entrance, ACT0821 Newburyport, ACT0826 Plum Island Sound): max/slack event tables the
     engine turns into analytic sinusoids -- the ebb/flood clock that Doppler-steepens the sea.
  2. NDBC buoy 44029 (NERACOOS A01, Mass Bay) ADCP surface current -- point ground truth.
  3. GoMOFS surface currents (ROMS, ~700 m) for the Gulf-of-Maine eddy field, via the CO-OPS
     THREDDS server: catalog crawl -> NetcdfSubset (classic netCDF-3, read by netcdf3.py) with
     an OPeNDAP-ascii fallback. Resampled to a regular lat/lon grid for the engine. This source
     is PROBED, not assumed; if it is unreachable the output simply omits the field and the
     engine's gulf view stays disabled.

Politeness as ever: cached forever where immutable, spaced requests, contact UA.
Output: data/currents/currents.json (+ gomofs_uv.f32 when the field probe succeeds).

A PLACE (harvest_place(), or --place NAME --box lon0,lat0,lon1,lat1 [--radius-km R]
[--max-stations N] [--adcp BUOY]): the current-prediction stations nearest the place, found in
CO-OPS's own station list (weak-and-variable stations skipped, one depth bin per station -- the
one nearest the surface), the given NDBC ADCP buoy, and optionally a model field handed in by
the caller, written to data/currents/<NAME>/currents.json in the same schema. The GoMOFS crawl
above is Gulf-of-Maine code (its catalog path, its GRID) and is NOT run for a place; a place's
field comes from harvest_place.py (NODD S3). Place requests go through harvester/polite.py.
"""

import json
import math
import os
import re
import struct
import sys
import time
import urllib.error
import urllib.parse
import urllib.request
from array import array
from datetime import datetime, timedelta, timezone

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import netcdf3  # noqa: E402

UA = "GAGAME/0.1 (hobby ocean simulator; contact: marksmcgrotty@gmail.com)"
COOPS = "https://api.tidesandcurrents.noaa.gov/api/prod/datagetter"
NDBC = "https://www.ndbc.noaa.gov/data/realtime2"
THREDDS = "https://opendap.co-ops.nos.noaa.gov/thredds"

STATIONS = [("ACT0816", "Merrimack River entrance"),
            ("ACT0821", "Newburyport"),
            ("ACT0826", "Plum Island Sound entrance")]

# Regular output grid for the Gulf window (covers the GoMOFS heart, Merrimack to Georges edge).
GRID = {"lon0": -71.1, "lat0": 41.9, "lon1": -69.3, "lat1": 43.3, "nx": 280, "ny": 200}
KNOT = 0.514444

_last_fetch = [0.0]
_polite = None      # a place harvest routes every request through harvester/polite.py


def log(msg):
    print(msg, flush=True)


def fetch(url, cache_path, binary=False, max_age_s=None, timeout=90):
    if _polite is not None:
        return _polite.fetch_compat(url, cache_path, binary=binary, timeout=timeout)
    if cache_path and os.path.exists(cache_path):
        age = time.time() - os.path.getmtime(cache_path)
        if max_age_s is None or age < max_age_s:
            with open(cache_path, "rb") as f:
                raw = f.read()
            return raw if binary else raw.decode("utf-8", errors="replace"), True
    if cache_path:
        os.makedirs(os.path.dirname(cache_path), exist_ok=True)
    for attempt in range(3):
        wait = 0.6 - (time.monotonic() - _last_fetch[0])
        if wait > 0:
            time.sleep(wait)
        try:
            req = urllib.request.Request(url, headers={"User-Agent": UA})
            with urllib.request.urlopen(req, timeout=timeout) as r:
                raw = r.read()
            _last_fetch[0] = time.monotonic()
            if cache_path:
                with open(cache_path, "wb") as f:
                    f.write(raw)
            return raw if binary else raw.decode("utf-8", errors="replace"), False
        except (urllib.error.URLError, urllib.error.HTTPError, TimeoutError) as e:
            _last_fetch[0] = time.monotonic()
            if attempt == 2:
                raise
            time.sleep(2.0 * (2 ** attempt))


# ================================================================= 1. tidal current predictions

def fetch_tidal_station(sid, name, cache_dir, bin_=None):
    now = datetime.now(timezone.utc)
    begin = now - timedelta(days=10)
    events = []
    flood_deg = ebb_deg = None
    for chunk in range(2):
        b = begin + timedelta(days=30 * chunk)
        e = min(b + timedelta(days=30), begin + timedelta(days=45))
        params = {
            "product": "currents_predictions", "application": "GAGAME", "station": sid,
            "begin_date": b.strftime("%Y%m%d"), "end_date": e.strftime("%Y%m%d"),
            "interval": "MAX_SLACK", "units": "english", "time_zone": "gmt", "format": "json",
        }
        tag = sid
        if bin_ is not None:                 # multi-bin (ADCP-derived) stations: name the depth
            params["bin"] = str(bin_)
            tag = f"{sid}_b{bin_}"
        url = COOPS + "?" + urllib.parse.urlencode(params)
        cache = os.path.join(cache_dir, f"{tag}_{params['begin_date']}_{params['end_date']}.json")
        text, cached = fetch(url, cache)
        data = json.loads(text)
        cp = (data.get("current_predictions") or {}).get("cp") or []
        if not cp and "error" in data:
            raise RuntimeError(f"{sid}: {data['error']}")
        for row in cp:
            t = datetime.strptime(row["Time"], "%Y-%m-%d %H:%M").replace(tzinfo=timezone.utc)
            typ = str(row.get("Type", "")).lower()
            v = float(row.get("Velocity_Major", 0.0)) * KNOT
            if "flood" in typ:
                signed = abs(v)
            elif "ebb" in typ:
                signed = -abs(v)
            else:
                signed = 0.0
                typ = "slack"
            if flood_deg is None and row.get("meanFloodDir") is not None:
                flood_deg = float(row["meanFloodDir"])
                ebb_deg = float(row.get("meanEbbDir", (flood_deg + 180.0) % 360.0))
            events.append({"t": t.timestamp(), "type": typ, "ms": round(signed, 4)})
        if not cached:
            log(f"    fetched {sid} {params['begin_date']}..{params['end_date']}: {len(cp)} events")
    events.sort(key=lambda x: x["t"])
    # De-duplicate the chunk boundary.
    dedup = []
    for ev in events:
        if not dedup or abs(ev["t"] - dedup[-1]["t"]) > 60:
            dedup.append(ev)
    log(f"[{sid}] {name}: {len(dedup)} events, flood {flood_deg} degT / ebb {ebb_deg} degT")
    out = {"id": sid, "name": name, "flood_deg": flood_deg, "ebb_deg": ebb_deg,
           "events": dedup}
    if bin_ is not None:
        out["bin"] = bin_
    return out


# ================================================================= 2. NDBC ADCP ground truth

def fetch_adcp(bid, cache_dir):
    stamp = datetime.now(timezone.utc).strftime("%Y%m%d%H")
    try:
        text, _ = fetch(f"{NDBC}/{bid}.adcp", os.path.join(cache_dir, f"{bid}_{stamp}.adcp"),
                        max_age_s=1800)
    except Exception as e:  # noqa: BLE001
        log(f"buoy {bid}: no .adcp ({e})")
        return None
    for ln in text.splitlines():
        if ln.startswith("#") or not ln.strip():
            continue
        t = ln.split()
        if len(t) < 8:
            continue
        try:
            obs = datetime(int(t[0]), int(t[1]), int(t[2]), int(t[3]), int(t[4]),
                           tzinfo=timezone.utc)
            dep = float(t[5])
            direc = float(t[6])       # degT, direction the current flows TOWARD
            spd = float(t[7]) / 100.0  # cm/s -> m/s
        except ValueError:
            continue
        log(f"buoy {bid}: {spd:.2f} m/s toward {direc:.0f} degT at {dep:.1f} m depth")
        return {"id": bid, "obs_unix": obs.timestamp(), "depth_m": dep,
                "toward_deg": direc, "ms": spd}
    return None


# ================================================================= 3. GoMOFS field via THREDDS

def crawl_gomofs_datasets(cache_dir):
    """Walk the THREDDS catalog toward GOMOFS 2ds files. Bounded, logged, cached briefly."""
    seen, found = set(), []
    queue = ["/thredds/catalog.xml",
             "/thredds/catalog/NOAA/GOMOFS/MODELS/catalog.xml"]
    hops = 0
    while queue and hops < 8:
        path = queue.pop(0)
        if path in seen:
            continue
        seen.add(path)
        hops += 1
        url = "https://opendap.co-ops.nos.noaa.gov" + path
        cache = os.path.join(cache_dir, "catalog",
                             re.sub(r"[^A-Za-z0-9]+", "_", path) + ".xml")
        try:
            xml, _ = fetch(url, cache, max_age_s=6 * 3600)
        except Exception as e:  # noqa: BLE001
            log(f"    catalog {path}: {e}")
            continue
        for href in re.findall(r'href="([^"]+\.xml)"', xml):
            full = href if href.startswith("/") else \
                os.path.dirname(path) + "/" + href
            if ("GOMOFS" in full or "gomofs" in full.lower()) and full not in seen:
                queue.append(full)
        for up in re.findall(r'urlPath="([^"]+)"', xml):
            if "gomofs" in up.lower():
                found.append(up)
        # Prefer walking date-sorted subdirectories newest-first.
        queue.sort(reverse=True)
    return found


def ncss_dataset_vars(ds_path, cache_dir):
    for stem in ("/thredds/ncss/grid/", "/thredds/ncss/"):
        url = f"https://opendap.co-ops.nos.noaa.gov{stem}{ds_path}/dataset.xml"
        try:
            xml, _ = fetch(url, None, timeout=60)
        except Exception:  # noqa: BLE001
            continue
        names = set(re.findall(r'<grid name="([^"]+)"', xml)) | \
            set(re.findall(r'<variable name="([^"]+)"', xml))
        if names:
            return stem, names
    return None, set()


def pick(cands, names):
    for c in cands:
        if c in names:
            return c
    return None


def parse_dap_ascii(text, var):
    """Values for `var` from a THREDDS DAP2 .ascii response: rows prefixed with [j][i] indices,
    comma-separated values, block ends at a blank line."""
    vals = []
    in_block = False
    for ln in text.splitlines():
        s = ln.strip()
        if not in_block:
            if s.startswith(var) and "[" in s:
                in_block = True
            continue
        if not s:
            break
        body = s
        if s.startswith("["):
            comma = s.find(",")
            if comma < 0:
                continue
            body = s[comma + 1:]
        for tok in body.split(","):
            tok = tok.strip()
            if not tok:
                continue
            try:
                vals.append(float(tok))
            except ValueError:
                pass
    return vals


def dap_dds_shapes(ds, cache_dir):
    url = f"{THREDDS}/dodsC/{ds}.dds"
    cache = os.path.join(cache_dir, "dds", re.sub(r"[^A-Za-z0-9]+", "_", ds) + ".dds")
    text, _ = fetch(url, cache)
    shapes = {}
    for m in re.finditer(r"Float\d+\s+(\w+)((?:\[[^\]]+\])+);", text):
        name = m.group(1)
        dims = [int(d) for d in re.findall(r"=\s*(\d+)\]", m.group(2))]
        shapes[name] = dims
    return shapes


def dap_fetch_var(ds, var, index_expr, cache_dir, tag):
    # Tomcat rejects RAW brackets in the query string with HTTP 400 (observed live); the DAP
    # constraint must be percent-encoded.
    expr = index_expr.replace("[", "%5B").replace("]", "%5D")
    url = f"{THREDDS}/dodsC/{ds}.ascii?{var}{expr}"
    cache = os.path.join(cache_dir, "field",
                         re.sub(r"[^A-Za-z0-9]+", "_", ds) + f"_{tag}.txt")
    text, cached = fetch(url, cache, timeout=240)
    vals = parse_dap_ascii(text, var)
    if not cached:
        log(f"    DAP {var}{index_expr}: {len(vals)} values")
    return vals


def fetch_gomofs_field(cache_dir):
    datasets = crawl_gomofs_datasets(cache_dir)
    log(f"GoMOFS: catalog crawl found {len(datasets)} dataset paths")
    twods = sorted([d for d in datasets if "2ds" in d], reverse=True)

    for ds in twods[:4]:
        # NCSS refuses bbox queries on the ROMS curvilinear grid, so use OPeNDAP index
        # hyperslabs, stride 2 (~1.4 km sampling). ROMS staggers u and v onto their own grids
        # with their own coordinate arrays; each is scattered onto the regular output grid
        # separately, then rotated by `angle` (resampled from the rho grid) into east/north.
        try:
            shapes = dap_dds_shapes(ds, cache_dir)
        except Exception as e:  # noqa: BLE001
            log(f"    {ds}: DDS failed ({e})")
            continue
        needed = ["u_sur", "v_sur", "lon_u", "lat_u", "lon_v", "lat_v", "angle",
                  "mask_u", "mask_v"]
        if any(n not in shapes for n in needed):
            log(f"    {ds}: missing some of {needed}")
            continue

        stride = 2

        def grab(var, tag, lead_time):
            dims = shapes[var]
            sl = f"[0:{stride}:{dims[-2] - 1}][0:{stride}:{dims[-1] - 1}]"
            lead = "[0]" * (len(dims) - 2) if lead_time else ""
            vals = dap_fetch_var(ds, var, lead + sl, cache_dir, tag)
            sny = (dims[-2] - 1) // stride + 1
            snx = (dims[-1] - 1) // stride + 1
            if len(vals) < sny * snx:
                raise RuntimeError(f"{var}: {len(vals)} of {sny * snx}")
            return vals, sny, snx

        try:
            u, uny, unx = grab("u_sur", "u", True)
            lon_u, _, _ = grab("lon_u", "lonu", False)
            lat_u, _, _ = grab("lat_u", "latu", False)
            mask_u, _, _ = grab("mask_u", "masku", False)
            v, vny, vnx = grab("v_sur", "v", True)
            lon_v, _, _ = grab("lon_v", "lonv", False)
            lat_v, _, _ = grab("lat_v", "latv", False)
            mask_v, _, _ = grab("mask_v", "maskv", False)
            ang, any_, anx = grab("angle", "ang", False)
            lon_r, _, _ = grab("lon_rho", "lonr", False)
            lat_r, _, _ = grab("lat_rho", "latr", False)
        except Exception as e:  # noqa: BLE001
            log(f"    {ds}: DAP subset failed ({e})")
            continue

        gu = scatter(lon_u, lat_u, u, mask_u, uny, unx)
        gv = scatter(lon_v, lat_v, v, mask_v, vny, vnx)
        ga_ = scatter(lon_r, lat_r, ang, None, any_, anx)
        for gr in (gu, gv, ga_):
            gap_fill(gr, sweeps=3)

        g = GRID
        out_u = array("f", [-999.0]) * (g["nx"] * g["ny"])
        out_v = array("f", [-999.0]) * (g["nx"] * g["ny"])
        filled = 0
        for k in range(g["nx"] * g["ny"]):
            uu, vv, aa = gu[k], gv[k], ga_[k]
            if uu > -900 and vv > -900:
                a = aa if aa > -900 else 0.0
                out_u[k] = uu * math.cos(a) - vv * math.sin(a)
                out_v[k] = uu * math.sin(a) + vv * math.cos(a)
                filled += 1
        log(f"    resampled {ds}: {filled} of {g['nx'] * g['ny']} cells water")
        if filled < 1000:
            continue
        return {"u": out_u, "v": out_v, "source": ds}
    return None


def scatter(lon, lat, vals, mask, sny, snx):
    """Nearest-cell scatter of a (possibly masked) staggered-grid variable onto GRID."""
    g = GRID
    out = array("f", [-999.0]) * (g["nx"] * g["ny"])
    dlon = (g["lon1"] - g["lon0"]) / g["nx"]
    dlat = (g["lat1"] - g["lat0"]) / g["ny"]
    for j in range(sny):
        for i in range(snx):
            k = j * snx + i
            if mask is not None and mask[k] < 0.5:
                continue
            x = vals[k]
            if math.isnan(x) or abs(x) > 100:
                continue
            # Splat into the 2x2 neighbourhood: stride-2 native sampling (~1.4 km) is sparser
            # than the output cells (~0.6 km), and single-cell seeding left a diagonal lattice
            # of holes in the first render.
            gx = int((lon[k] - g["lon0"]) / dlon)
            gy = int((lat[k] - g["lat0"]) / dlat)
            for oy in (0, 1):
                for ox in (0, 1):
                    xx, yy = gx + ox, gy + oy
                    if 0 <= xx < g["nx"] and 0 <= yy < g["ny"]:
                        out[yy * g["nx"] + xx] = x
    return out


def gap_fill(grid, sweeps):
    g = GRID
    for sweep in range(sweeps):
        need = 2 if sweep == 0 else 1   # strict first, then let coasts creep closed
        src = array("f", grid)
        for gy in range(g["ny"]):
            for gx in range(g["nx"]):
                k = gy * g["nx"] + gx
                if src[k] > -900:
                    continue
                s = 0.0
                n = 0
                for oy, ox in ((0, 1), (0, -1), (1, 0), (-1, 0)):
                    yy, xx = gy + oy, gx + ox
                    if 0 <= yy < g["ny"] and 0 <= xx < g["nx"]:
                        kk = yy * g["nx"] + xx
                        if src[kk] > -900:
                            s += src[kk]
                            n += 1
                if n >= need:
                    grid[k] = s / n


def resample_arrays(lon, lat, curvilinear, ny, nx, u, v, ang, source):
    g = GRID
    out_u = array("f", [-999.0]) * (g["nx"] * g["ny"])
    out_v = array("f", [-999.0]) * (g["nx"] * g["ny"])
    dlon = (g["lon1"] - g["lon0"]) / g["nx"]
    dlat = (g["lat1"] - g["lat0"]) / g["ny"]

    filled = 0
    for j in range(ny):
        for i in range(nx):
            k = j * nx + i
            lo = lon[k] if curvilinear else lon[i]
            la = lat[k] if curvilinear else lat[j]
            uu, vv = u[k], v[k]
            if math.isnan(uu) or math.isnan(vv) or abs(uu) > 30 or abs(vv) > 30:
                continue
            if ang is not None:
                a = ang[k]
                uu, vv = uu * math.cos(a) - vv * math.sin(a), \
                    uu * math.sin(a) + vv * math.cos(a)
            gx = int((lo - g["lon0"]) / dlon)
            gy = int((la - g["lat0"]) / dlat)
            if 0 <= gx < g["nx"] and 0 <= gy < g["ny"]:
                out_u[gy * g["nx"] + gx] = uu
                out_v[gy * g["nx"] + gx] = vv
                filled += 1
    log(f"    resampled {source}: {ny}x{nx} native -> {g['nx']}x{g['ny']} regular, "
        f"{filled} samples, {'rotated' if ang is not None else 'east/north native'}")

    # Fill small gaps by 4-neighbour averaging (two sweeps) so the regular grid is contiguous.
    for _ in range(2):
        src_u, src_v = array("f", out_u), array("f", out_v)
        for gy in range(g["ny"]):
            for gx in range(g["nx"]):
                k = gy * g["nx"] + gx
                if src_u[k] > -900:
                    continue
                su = sv = 0.0
                n = 0
                for oy, ox in ((0, 1), (0, -1), (1, 0), (-1, 0)):
                    yy, xx = gy + oy, gx + ox
                    if 0 <= yy < g["ny"] and 0 <= xx < g["nx"]:
                        kk = yy * g["nx"] + xx
                        if src_u[kk] > -900:
                            su += src_u[kk]
                            sv += src_v[kk]
                            n += 1
                if n >= 2:
                    out_u[k] = su / n
                    out_v[k] = sv / n
    return {"u": out_u, "v": out_v, "source": source}


# ================================================================= a place

def _km(lat0, lon0, lat1, lon1):
    return math.hypot((lon1 - lon0) * 111.32 * math.cos(math.radians((lat0 + lat1) / 2)),
                      (lat1 - lat0) * 110.57)


def discover_place(box, radius_km, anchor, max_stations, coops_cache):
    """Current-prediction stations within radius_km of the box, nearest the anchor first, from
    CO-OPS's own list (mdapi stations.json?type=currentpredictions, cached forever). Type W
    (weak and variable: no predictions exist) is skipped; a multi-bin station contributes its
    bin nearest the surface. -> [(id, name, bin or None, km)]"""
    raw, _ = fetch("https://api.tidesandcurrents.noaa.gov/mdapi/prod/webapi/stations.json?"
                   "type=currentpredictions",
                   os.path.join(coops_cache, "_lists", "currentpredictions.json"))
    listed = json.loads(raw).get("stations", [])
    best = {}
    for st in listed:
        if st.get("type") == "W":
            continue
        lat, lon = float(st.get("lat", 0)), float(st.get("lng", 0))
        clat = min(max(lat, box["lat0"]), box["lat1"])
        clon = min(max(lon, box["lon0"]), box["lon1"])
        if _km(lat, lon, clat, clon) > radius_km:
            continue
        depth = st.get("depth")
        key = (depth if depth is not None else 0.0, st.get("currbin") or 0)
        cur = best.get(st["id"])
        if cur is None or key < cur[0]:
            best[st["id"]] = (key, st, _km(lat, lon, anchor[1], anchor[0]))
    many_bins = {}
    for st in listed:
        many_bins[st["id"]] = many_bins.get(st["id"], 0) + 1
    picks = sorted(best.values(), key=lambda v: v[2])[:max_stations]
    return [(st["id"], st.get("name", st["id"]),
             st.get("currbin") if many_bins[st["id"]] > 1 else None, round(d, 1))
            for _, st, d in picks]


def harvest_place(place, box, radius_km, anchor=None, max_stations=8, adcp=None, field=None,
                  out_root=None):
    """data/currents/<place>/currents.json (+ the field's .f32 when `field` is given: a dict
    with 'u', 'v' float arrays and the grid keys of the schema). Never overwrites.
    Returns {'files': [(path, bytes)], 'stations': [...], 'adcp': ...}."""
    global _polite
    import polite
    _polite = polite
    anchor = anchor or ((box["lon0"] + box["lon1"]) / 2, (box["lat0"] + box["lat1"]) / 2)
    out_dir = os.path.join(out_root or os.path.join("data", "currents"), place)
    path = os.path.join(out_dir, "currents.json")
    if os.path.exists(path):
        raise SystemExit(f"refusing to overwrite {path} (places only add)")
    cache_dir = os.path.join("cache", "currents")
    picks = discover_place(box, radius_km, anchor, max_stations, os.path.join("cache", "coops"))
    log(f"[{place}] {len(picks)} current station(s): "
        f"{[(s, b, km) for s, _, b, km in picks]}")
    stations, failed = [], []
    for sid, name, bin_, km in picks:
        try:
            st = fetch_tidal_station(sid, name, cache_dir, bin_)
            st["km_from_anchor"] = km
            stations.append(st)
        except polite.Stop:
            raise
        except Exception as e:  # noqa: BLE001
            log(f"[{sid}] FAILED: {e}")
            failed.append({"id": sid, "bin": bin_, "why": str(e)})
    buoy = fetch_adcp(adcp, cache_dir) if adcp else None
    out = {"generated_utc": datetime.now(timezone.utc).isoformat(timespec="seconds"),
           "tidal_stations": stations, "buoy_adcp": buoy, "field": None}
    os.makedirs(out_dir, exist_ok=True)
    files = []
    if field:
        fpath = os.path.join(out_dir, field["file"])
        with open(fpath, "xb") as f:
            f.write(struct.pack(f"<{len(field['u'])}f", *field["u"]))
            f.write(struct.pack(f"<{len(field['v'])}f", *field["v"]))
        files.append((fpath, os.path.getsize(fpath)))
        out["field"] = {k: v for k, v in field.items() if k not in ("u", "v")}
    with open(path, "x", encoding="utf-8") as f:
        json.dump(out, f, indent=1)
    files.append((path, os.path.getsize(path)))
    log(f"wrote {path} ({len(stations)} tidal stations, adcp {'yes' if buoy else 'no'}, "
        f"field {'yes' if field else 'no'})")
    return {"files": files, "stations": [(s["id"], s.get("bin")) for s in stations],
            "failed": failed, "adcp": buoy}


# ================================================================= main

def main():
    args = sys.argv[1:]
    if "--place" in args:
        def opt(flag, default=None):
            return args[args.index(flag) + 1] if flag in args else default
        lon0, lat0, lon1, lat1 = (float(v) for v in opt("--box").split(","))
        harvest_place(opt("--place"), {"lon0": lon0, "lat0": lat0, "lon1": lon1, "lat1": lat1},
                      float(opt("--radius-km", "20")),
                      anchor=tuple(float(v) for v in opt("--anchor").split(","))
                      if "--anchor" in args else None,
                      max_stations=int(opt("--max-stations", "8")), adcp=opt("--adcp"))
        return
    out_dir = os.path.join("data", "currents")
    cache_dir = os.path.join("cache", "currents")
    os.makedirs(out_dir, exist_ok=True)

    stations = []
    for sid, name in STATIONS:
        try:
            stations.append(fetch_tidal_station(sid, name, cache_dir))
        except Exception as e:  # noqa: BLE001
            log(f"[{sid}] FAILED: {e}")

    adcp = fetch_adcp("44029", cache_dir)

    field = None
    try:
        field = fetch_gomofs_field(cache_dir)
    except Exception as e:  # noqa: BLE001
        log(f"GoMOFS field: {e}")

    out = {
        "generated_utc": datetime.now(timezone.utc).isoformat(timespec="seconds"),
        "tidal_stations": stations,
        "buoy_adcp": adcp,
        "field": None,
    }
    if field:
        g = GRID
        with open(os.path.join(out_dir, "gomofs_uv.f32"), "wb") as f:
            f.write(struct.pack(f"<{len(field['u'])}f", *field["u"]))
            f.write(struct.pack(f"<{len(field['v'])}f", *field["v"]))
        out["field"] = {
            "file": "gomofs_uv.f32", "nx": g["nx"], "ny": g["ny"],
            "lon0": g["lon0"], "lat0": g["lat0"],
            "dlon": (g["lon1"] - g["lon0"]) / g["nx"],
            "dlat": (g["lat1"] - g["lat0"]) / g["ny"],
            "source": field["source"],
        }
    path = os.path.join(out_dir, "currents.json")
    with open(path, "w", encoding="utf-8") as f:
        json.dump(out, f, indent=1)
    log(f"wrote {path} ({len(stations)} tidal stations, field: "
        f"{'yes, ' + field['source'] if field else 'NO'})")


if __name__ == "__main__":
    main()
