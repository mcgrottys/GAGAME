#!/usr/bin/env python3
"""GAGAME Harvester, M2: GFS-Wave sea state for the Gulf of Maine + NDBC buoy ground truth.

NOMADS retired OPeNDAP in 2025 (Service Change Notice 25-81), so the sanctioned small-subset
path is the GRIB FILTER: server-side variable + region subsetting, returning tiny GRIB2 files
that harvester/grib2.py decodes (stdlib only). Politeness per GAMEPLAN.md 6.3:

  * one probe per candidate cycle until the newest published one is found;
  * an 11-request CALIBRATION, once per lifetime (cached): fetch each variable alone at f000 and
    learn its (category, number, level) keys, so no GRIB parameter table is ever trusted blindly;
  * then ONE request per forecast hour (all variables, ~1 x 1 degree box), f000..f048 step 4;
  * every response cached forever (cycle files are immutable); requests spaced >= 0.6 s.

Ground truth: NDBC realtime2 for buoys 44013 (Boston, directional) + 44098 (Jeffrey's Ledge),
including 44013's measured spectral density S(f). Output: data/sea/seastate.json.
"""

import json
import math
import os
import sys
import time
import urllib.error
import urllib.parse
import urllib.request
from datetime import datetime, timedelta, timezone

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import grib2  # noqa: E402

UA = "GAGAME/0.1 (hobby ocean simulator; contact: marksmcgrotty@gmail.com)"
FILTERS = [
    "https://nomads.ncep.noaa.gov/cgi-bin/filter_gfswave.pl",
    "https://nomads.ncep.noaa.gov/gribfilter.php",
]
NDBC = "https://www.ndbc.noaa.gov/data/realtime2"

TARGET_LAT, TARGET_LON_E = 42.35, 289.35          # near buoy 44013 (42.346N, 70.651W)
BOX = {"leftlon": 289.0, "rightlon": 290.0, "toplat": 42.75, "bottomlat": 42.0}
VARS = ["HTSGW", "PERPW", "DIRPW", "WVHGT", "WVPER", "WVDIR",
        "SWELL", "SWPER", "SWDIR", "WIND", "WDIR"]
FHS = list(range(0, 49, 4))

_last_fetch = [0.0]


def log(msg):
    print(msg, flush=True)


def fetch_bytes(url, cache_path, max_age_s=None):
    if os.path.exists(cache_path):
        age = time.time() - os.path.getmtime(cache_path)
        if max_age_s is None or age < max_age_s:
            with open(cache_path, "rb") as f:
                return f.read(), True
    os.makedirs(os.path.dirname(cache_path), exist_ok=True)
    for attempt in range(4):
        wait = 0.6 - (time.monotonic() - _last_fetch[0])
        if wait > 0:
            time.sleep(wait)
        try:
            req = urllib.request.Request(url, headers={"User-Agent": UA})
            with urllib.request.urlopen(req, timeout=60) as r:
                data = r.read()
            _last_fetch[0] = time.monotonic()
            with open(cache_path, "wb") as f:
                f.write(data)
            return data, False
        except (urllib.error.URLError, urllib.error.HTTPError, TimeoutError) as e:
            _last_fetch[0] = time.monotonic()
            if attempt == 3:
                raise
            backoff = 2.0 * (2 ** attempt)
            log(f"    retry in {backoff:.0f}s ({e})")
            time.sleep(backoff)


def filter_url(base, ymd, cyc, fh, var_flags):
    params = {
        "file": f"gfswave.t{cyc:02d}z.global.0p25.f{fh:03d}.grib2",
        "all_lev": "on",
        "subregion": "",
        "leftlon": BOX["leftlon"], "rightlon": BOX["rightlon"],
        "toplat": BOX["toplat"], "bottomlat": BOX["bottomlat"],
        "dir": f"/gfs.{ymd}/{cyc:02d}/wave/gridded",
    }
    for v in var_flags:
        params[f"var_{v}"] = "on"
    return base + "?" + urllib.parse.urlencode(params)


def fetch_grib(ymd, cyc, fh, var_flags, cache_dir, tag):
    cache = os.path.join(cache_dir, ymd, f"{cyc:02d}z_f{fh:03d}_{tag}.grib2")
    last_err = None
    for base in FILTERS:
        try:
            data, cached = fetch_bytes(filter_url(base, ymd, cyc, fh, var_flags), cache)
            if data[:4] == b"GRIB" or b"GRIB" in data[:128]:
                if not cached:
                    log(f"    fetched f{fh:03d} {tag}: {len(data)} B")
                return data
            last_err = f"non-GRIB response ({len(data)} B) from {base.split('/')[-1]}"
            os.remove(cache)
        except Exception as e:      # noqa: BLE001 - try the next endpoint
            last_err = str(e)
    raise RuntimeError(f"f{fh:03d} {tag}: {last_err}")


def find_cycle(cache_dir):
    now = datetime.now(timezone.utc)
    for day_back in (0, 1):
        day = now - timedelta(days=day_back)
        for cyc in (18, 12, 6, 0):
            cyc_dt = day.replace(hour=cyc, minute=0, second=0, microsecond=0)
            if cyc_dt + timedelta(hours=4, minutes=30) > now:
                continue
            ymd = cyc_dt.strftime("%Y%m%d")
            try:
                fetch_grib(ymd, cyc, FHS[-1], ["HTSGW"], cache_dir, "probe")
                return cyc_dt, ymd, cyc
            except Exception as e:  # noqa: BLE001
                log(f"  cycle {ymd} {cyc:02d}z not ready ({e})")
    raise RuntimeError("no published gfswave cycle found via the grib filter")


def calibrate(ymd, cyc, cache_dir):
    """Learn each variable's (discipline, category, number, level) keys from single-variable
    fetches -- no GRIB parameter table to trust, no way to misattribute a message."""
    map_path = os.path.join(cache_dir, "varmap.json")
    if os.path.exists(map_path):
        with open(map_path, "r", encoding="utf-8") as f:
            return {k: [tuple(t) for t in v] for k, v in json.load(f).items()}
    varmap = {}
    for v in VARS:
        data = fetch_grib(ymd, cyc, 0, [v], cache_dir, f"cal_{v}")
        keys = sorted({m.key() for m in grib2.read_messages(data)})
        varmap[v] = keys
        log(f"    {v}: {keys}")
    with open(map_path, "w", encoding="utf-8") as f:
        json.dump(varmap, f)
    return varmap


def parse_ndbc_met(text):
    for ln in text.splitlines():
        if ln.startswith("#") or not ln.strip():
            continue
        t = ln.split()
        if len(t) < 12:
            continue

        def num(idx):
            try:
                return float(t[idx])
            except (ValueError, IndexError):
                return None

        obs = datetime(int(t[0]), int(t[1]), int(t[2]), int(t[3]), int(t[4]),
                       tzinfo=timezone.utc)
        return {"obs_unix": obs.timestamp(), "wdir": num(5), "wspd": num(6), "hs": num(8),
                "dpd": num(9), "apd": num(10), "mwd": num(11)}
    return None


def parse_ndbc_spec(text):
    for ln in text.splitlines():
        if ln.startswith("#") or not ln.strip():
            continue
        t = ln.split()
        freqs, dens = [], []
        i = 6
        while i + 1 < len(t):
            try:
                d = float(t[i])
                f = float(t[i + 1].strip("()"))
            except ValueError:
                break
            dens.append(d)
            freqs.append(f)
            i += 2
        if freqs:
            return {"freq_hz": freqs, "dens_m2_hz": dens}
    return None


def main():
    out_dir = os.path.join("data", "sea")
    cache_dir = os.path.join("cache", "gfswave")
    ndbc_cache = os.path.join("cache", "ndbc")
    os.makedirs(out_dir, exist_ok=True)

    cyc_dt, ymd, cyc = find_cycle(cache_dir)
    log(f"cycle gfswave {ymd} {cyc:02d}z")
    varmap = calibrate(ymd, cyc, cache_dir)

    key_to_var = {}
    for v, keys in varmap.items():
        for k in keys:
            key_to_var[tuple(k)] = v

    hours = []
    for fh in FHS:
        data = fetch_grib(ymd, cyc, fh, VARS, cache_dir, "all")
        point = {}
        for m in grib2.read_messages(data):
            v = key_to_var.get(m.key())
            if v is None:
                continue
            name = f"{v}_{m.level_value}" if v in ("SWELL", "SWPER", "SWDIR") else v
            point[name] = m.at_latlon(TARGET_LAT, TARGET_LON_E)

        entry = {
            "fh": fh,
            "wind_ms": point.get("WIND"), "wind_from_deg": point.get("WDIR"),
            "combined": {"hs": point.get("HTSGW"), "tp": point.get("PERPW"),
                         "from_deg": point.get("DIRPW")},
            "partitions": [],
        }
        ws = {"hs": point.get("WVHGT"), "tp": point.get("WVPER"),
              "from_deg": point.get("WVDIR"), "kind": "windsea"}
        if ws["hs"] and ws["tp"] and ws["tp"] > 0.5:
            entry["partitions"].append(ws)
        for i in (1, 2, 3):
            sw = {"hs": point.get(f"SWELL_{i}"), "tp": point.get(f"SWPER_{i}"),
                  "from_deg": point.get(f"SWDIR_{i}"), "kind": "swell"}
            if sw["hs"] and sw["hs"] > 0.02 and sw["tp"] and sw["tp"] > 0.5:
                entry["partitions"].append(sw)
        hours.append(entry)
        if fh == 0:
            log(f"    f000 @ ({TARGET_LAT}, {TARGET_LON_E - 360.0}): "
                f"Hs {entry['combined']['hs']} m, Tp {entry['combined']['tp']} s, "
                f"wind {entry['wind_ms']} m/s, {len(entry['partitions'])} partitions")

    buoys = {}
    for bid in ("44013", "44098"):
        stamp = datetime.now(timezone.utc).strftime("%Y%m%d%H")
        met_raw, _ = fetch_bytes(f"{NDBC}/{bid}.txt",
                                 os.path.join(ndbc_cache, f"{bid}_{stamp}.txt"), max_age_s=1800)
        b = parse_ndbc_met(met_raw.decode("utf-8", errors="replace")) or {}
        try:
            spec_raw, _ = fetch_bytes(f"{NDBC}/{bid}.data_spec",
                                      os.path.join(ndbc_cache, f"{bid}_{stamp}.data_spec"),
                                      max_age_s=1800)
            spec = parse_ndbc_spec(spec_raw.decode("utf-8", errors="replace"))
            if spec:
                b["spectrum"] = spec
        except Exception as e:      # noqa: BLE001
            log(f"    {bid}.data_spec unavailable ({e})")
        buoys[bid] = b
        log(f"buoy {bid}: Hs {b.get('hs')} m, DPD {b.get('dpd')} s, MWD {b.get('mwd')} degT"
            + (f", spectrum {len(b['spectrum']['freq_hz'])} bins" if "spectrum" in b else ""))

    out = {
        "generated_utc": datetime.now(timezone.utc).isoformat(timespec="seconds"),
        "source": "GFS-Wave via NOMADS grib filter (OPeNDAP retired, SCN 25-81) + NDBC realtime2",
        "cycle_unix": cyc_dt.timestamp(),
        "cycle_label": f"gfswave {ymd} {cyc:02d}z",
        "point_lat": TARGET_LAT, "point_lon": TARGET_LON_E - 360.0,
        "hours": hours,
        "buoys": buoys,
    }
    path = os.path.join(out_dir, "seastate.json")
    with open(path, "w", encoding="utf-8") as f:
        json.dump(out, f, indent=1)

    h0 = hours[0]
    b13 = buoys.get("44013", {})
    log(f"wrote {path}")
    if h0["combined"]["hs"] and b13.get("hs"):
        log(f"sanity: model f000 Hs {h0['combined']['hs']:.2f} m vs buoy 44013 Hs "
            f"{b13['hs']:.2f} m (same neighbourhood expected)")


if __name__ == "__main__":
    main()
