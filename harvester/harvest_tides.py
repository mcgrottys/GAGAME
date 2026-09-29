#!/usr/bin/env python3
"""GAGAME Harvester, M1: NOAA CO-OPS tides for the lower Merrimack, fetched ONCE, politely.

Per station:
  1. metadata (name, lat, lon)      mdapi /stations/{id}.json                 [cached forever]
  2. harmonic constituents          mdapi /stations/{id}/harcon.json          [cached forever]
  3. official hourly predictions for one whole year, 12 monthly requests
     (product=predictions, datum=MLLW, units=metric, time_zone=gmt)          [cached forever]
  4. least-squares fit of amplitude+phase AT THE EXACT HARCON FREQUENCIES to that year
  5. emit  data/tides/stations.json  +  official_{id}_{year}.f32 sidecars

WHY FIT rather than compute nodal factors: the fit reproduces NOAA's official predictions to
millimetres BY CONSTRUCTION -- which is M1's acceptance gate -- while the engine stays a pure
stateless sum of cosines, and no Schureman astronomy gets transcribed (a classic source of silent
sign/epoch bugs). The cost: the fitted phases absorb the fit year's nodal state, so accuracy
degrades by a few cm/year outside that year. Re-running with --year refits from cache for free.

POLITENESS (GAMEPLAN.md section 6.3): every request carries application=GAGAME and a contact
User-Agent; every response is cached forever under cache/coops/ and never re-fetched; network
hits are spaced >= 0.4 s with capped exponential backoff on failure. Total for the default six
stations: ~90 small requests, once ever. Offline afterwards.

A PLACE (--place NAME --box lon0,lat0,lon1,lat1 [--radius-km R]): the harmonic (type R)
stations inside the box or within R km of it, found in CO-OPS's own station list (the cached
mdapi stations.json?type=tidepredictions that harvest_water.py keeps), fitted exactly as above
and written to data/tides/<NAME>/ -- stations.json + official_<id>_<year>.f32, the same format,
so TideModel reads it by path. Place requests go through harvester/polite.py (budgeted, >= 1 s
per host, no contact address) and ask for the fit year in ONE request per station when CO-OPS
allows it (twelve monthly ones otherwise); a place never overwrites a file.
"""

import json
import math
import os
import struct
import sys
import time
import urllib.error
import urllib.parse
import urllib.request
from datetime import datetime, timezone

try:
    import numpy as np
except ImportError:
    np = None

UA = "GAGAME/0.1 (hobby ocean simulator; contact: marksmcgrotty@gmail.com)"
API = "https://api.tidesandcurrents.noaa.gov/api/prod/datagetter"
MDAPI = "https://api.tidesandcurrents.noaa.gov/mdapi/prod/webapi"

# Mouth -> upriver, then the Boston reference (river=False keeps it off the ribbon profile).
# IDs verified live against the CO-OPS metadata API on 2026-08-28.
STATIONS = [
    ("8440452", True,  "Merrimack Entrance"),   # Plum Island, Merrimack River Entrance
    ("8440466", True,  "Newburyport"),
    ("8440273", True,  "Salisbury Point"),
    ("8440369", True,  "Merrimacport"),
    ("8440889", True,  "Riverside"),
    ("8443970", False, "Boston"),
]

# M6v: the water atlas widens the survey -- harvest_water.py discovers harmonic stations in
# the focus region (Gloucester, Rockport, Boston Light, Salem, ...) and lists them here.
# Same fit machinery, same politeness; river=False keeps them off the ribbon profile.
_extra = os.path.join("data", "water", "extra_tide_stations.json")
if os.path.exists(_extra):
    with open(_extra, encoding="utf-8") as _f:
        for _st in json.load(_f):
            if not any(s[0] == _st["id"] for s in STATIONS):
                STATIONS.append((_st["id"], False, _st["name"]))

MIN_AMP_M = 0.001      # constituents below 1 mm are dropped from the fit
_last_fetch = [0.0]
_polite = None         # a place harvest routes every request through harvester/polite.py
_year_in_one = [None]  # does CO-OPS hand out a whole year of hourly predictions per request?


def log(msg):
    print(msg, flush=True)


def fetch_json(url, cache_path):
    """Cache-forever GET. Only genuinely new URLs ever touch the network."""
    if os.path.exists(cache_path):
        with open(cache_path, "r", encoding="utf-8") as f:
            return json.load(f), True
    if _polite is not None:
        raw, cached = _polite.get(url, cache_path, timeout=60)
        return json.loads(raw.decode("utf-8")), cached
    os.makedirs(os.path.dirname(cache_path), exist_ok=True)

    for attempt in range(4):
        wait = 0.4 - (time.monotonic() - _last_fetch[0])
        if wait > 0:
            time.sleep(wait)
        try:
            req = urllib.request.Request(url, headers={"User-Agent": UA})
            with urllib.request.urlopen(req, timeout=30) as r:
                data = json.loads(r.read().decode("utf-8"))
            _last_fetch[0] = time.monotonic()
            with open(cache_path, "w", encoding="utf-8") as f:
                json.dump(data, f)
            return data, False
        except (urllib.error.URLError, urllib.error.HTTPError, TimeoutError) as e:
            _last_fetch[0] = time.monotonic()
            if attempt == 3:
                raise
            backoff = 2.0 * (2 ** attempt)
            log(f"    retry in {backoff:.0f}s ({e})")
            time.sleep(backoff)


def month_days(year, month):
    if month == 12:
        return 31
    return (datetime(year, month + 1, 1) - datetime(year, month, 1)).days


def get_station_meta(sid, cache_dir):
    data, cached = fetch_json(f"{MDAPI}/stations/{sid}.json",
                              os.path.join(cache_dir, sid, "meta.json"))
    st = data["stations"][0] if "stations" in data else data
    return {"name": st.get("name", sid), "lat": float(st.get("lat", 0)),
            "lon": float(st.get("lng", st.get("lon", 0)))}, cached


def get_datums(sid, cache_dir):
    """Station datum elevations (metric). Returns MLLW - NAVD88 in metres (the offset that turns
    a tide height in MLLW into NAVD88, the CUDEM terrain's datum), or None if the station has no
    NAVD link."""
    try:
        data, _ = fetch_json(f"{MDAPI}/stations/{sid}/datums.json?units=metric",
                             os.path.join(cache_dir, sid, "datums_metric.json"))
    except Exception as e:  # noqa: BLE001
        log(f"    datums unavailable for {sid} ({e})")
        return None
    vals = {d.get("name"): d.get("value") for d in data.get("datums", [])}
    if "MLLW" in vals and "NAVD88" in vals and vals["NAVD88"] is not None:
        return float(vals["MLLW"]) - float(vals["NAVD88"])
    return None


def get_harcon(sid, cache_dir):
    data, cached = fetch_json(f"{MDAPI}/stations/{sid}/harcon.json?units=metric",
                              os.path.join(cache_dir, sid, "harcon_metric.json"))
    cons = data.get("HarmonicConstituents", [])
    out = []
    for c in cons:
        amp = float(c.get("amplitude", 0.0))
        out.append({"name": c.get("name", "?"), "speed_dph": float(c["speed"]),
                    "amp_m": amp, "phase_gmt_deg": float(c.get("phase_GMT", 0.0))})
    # Unit sanity: M2 at these stations is ~1.2 m (Merrimack) / ~1.4 m (Boston). If it looks like
    # feet (~4), the units param was ignored -- convert rather than trust.
    m2 = next((c for c in out if c["name"] == "M2"), None)
    if m2 and m2["amp_m"] > 2.5:
        log(f"    harcon for {sid} looks like feet (M2={m2['amp_m']:.2f}); converting to metres")
        for c in out:
            c["amp_m"] *= 0.3048
    return out, cached


def _predictions_whole_year(sid, year, cache_dir):
    """Place harvests only: the fit year in ONE request, when CO-OPS allows the span (the answer
    is learned once per run). Returns (times, values) or None to fall back to monthly."""
    if _polite is None or _year_in_one[0] is False:
        return None
    if all(os.path.exists(os.path.join(cache_dir, sid, f"pred_{year}_{m:02d}.json"))
           for m in range(1, 13)):
        return None                       # the monthly cache is complete: it is the answer
    params = {
        "product": "predictions", "application": "GAGAME", "station": sid,
        "begin_date": f"{year}0101", "end_date": f"{year}1231",
        "datum": "MLLW", "units": "metric", "time_zone": "gmt",
        "interval": "h", "format": "json",
    }
    cache = os.path.join(cache_dir, sid, f"pred_{year}_all.json")
    fresh = not os.path.exists(cache)
    try:
        data, cached = fetch_json(API + "?" + urllib.parse.urlencode(params), cache)
    except urllib.error.HTTPError as e:
        log(f"    one-request year refused for {sid} (HTTP {e.code}); monthly from now on")
        _year_in_one[0] = False
        return None
    preds = data.get("predictions", [])
    if "error" in data or len(preds) < 8700:
        log(f"    one-request year refused for {sid} "
            f"({data.get('error', {}).get('message', len(preds))}); monthly from now on")
        _year_in_one[0] = False
        return None
    if fresh:
        _year_in_one[0] = True
        log(f"    fetched {year} in one request: {len(preds)} h")
    times, values = [], []
    for p in preds:
        t = datetime.strptime(p["t"], "%Y-%m-%d %H:%M").replace(tzinfo=timezone.utc)
        times.append(t.timestamp())
        values.append(float(p["v"]))
    return times, values


def get_predictions_year(sid, year, cache_dir):
    """One year of official hourly predictions, datum MLLW, metric, GMT. 12 monthly requests."""
    whole = _predictions_whole_year(sid, year, cache_dir)
    if whole is not None:
        return whole
    times, values = [], []
    for month in range(1, 13):
        nd = month_days(year, month)
        params = {
            "product": "predictions", "application": "GAGAME", "station": sid,
            "begin_date": f"{year}{month:02d}01", "end_date": f"{year}{month:02d}{nd:02d}",
            "datum": "MLLW", "units": "metric", "time_zone": "gmt",
            "interval": "h", "format": "json",
        }
        url = API + "?" + urllib.parse.urlencode(params)
        cache = os.path.join(cache_dir, sid, f"pred_{year}_{month:02d}.json")
        data, cached = fetch_json(url, cache)
        if "error" in data:
            raise RuntimeError(f"{sid} {year}-{month:02d}: CO-OPS error: "
                               f"{data['error'].get('message', data['error'])}")
        preds = data.get("predictions", [])
        if not preds:
            raise RuntimeError(f"{sid} {year}-{month:02d}: empty predictions")
        for p in preds:
            t = datetime.strptime(p["t"], "%Y-%m-%d %H:%M").replace(tzinfo=timezone.utc)
            times.append(t.timestamp())
            values.append(float(p["v"]))
        if not cached:
            log(f"    fetched {year}-{month:02d}: {len(preds)} h")
    return times, values


def cholesky_solve(g, r):
    """Solve G x = r for symmetric positive-definite G, pure Python. ~75x75: instant."""
    n = len(r)
    l = [[0.0] * n for _ in range(n)]
    for i in range(n):
        for j in range(i + 1):
            s = g[i][j] - sum(l[i][k] * l[j][k] for k in range(j))
            if i == j:
                if s <= 0:
                    raise RuntimeError(f"normal matrix not SPD at {i} (s={s:g}); "
                                       "duplicate constituent frequencies?")
                l[i][i] = math.sqrt(s)
            else:
                l[i][j] = s / l[j][j]
    y = [0.0] * n
    for i in range(n):
        y[i] = (r[i] - sum(l[i][k] * y[k] for k in range(i))) / l[i][i]
    x = [0.0] * n
    for i in range(n - 1, -1, -1):
        x[i] = (y[i] - sum(l[k][i] * x[k] for k in range(i + 1, n))) / l[i][i]
    return x


def fit_station(times, values, omegas, epoch):
    """Least squares of v ~ mean + sum_i a_i cos(w_i tau) + b_i sin(w_i tau), tau = t - epoch.
    Returns (mean, [(a_i, b_i)...], rms, maxerr)."""
    n_c = len(omegas)
    m = 1 + 2 * n_c
    n_s = len(times)

    if np is not None:
        tau = np.asarray(times) - epoch
        a = np.empty((n_s, m))
        a[:, 0] = 1.0
        for i, w in enumerate(omegas):
            a[:, 1 + 2 * i] = np.cos(w * tau)
            a[:, 2 + 2 * i] = np.sin(w * tau)
        v = np.asarray(values)
        x, *_ = np.linalg.lstsq(a, v, rcond=None)
        resid = v - a @ x
        rms = float(np.sqrt(np.mean(resid ** 2)))
        mx = float(np.max(np.abs(resid)))
        x = x.tolist()
    else:
        g = [[0.0] * m for _ in range(m)]
        r = [0.0] * m
        row = [0.0] * m
        row[0] = 1.0
        t0 = time.monotonic()
        for s in range(n_s):
            tau = times[s] - epoch
            for i, w in enumerate(omegas):
                wt = w * tau
                row[1 + 2 * i] = math.cos(wt)
                row[2 + 2 * i] = math.sin(wt)
            v = values[s]
            for i in range(m):
                ri = row[i]
                r[i] += ri * v
                gi = g[i]
                for j in range(i, m):
                    gi[j] += ri * row[j]
            if s % 2000 == 1999:
                log(f"    accumulating {s + 1}/{n_s} ({time.monotonic() - t0:.0f}s)")
        for i in range(m):
            for j in range(i):
                g[i][j] = g[j][i]
        x = cholesky_solve(g, r)
        sq = 0.0
        mx = 0.0
        for s in range(n_s):
            tau = times[s] - epoch
            h = x[0]
            for i, w in enumerate(omegas):
                h += x[1 + 2 * i] * math.cos(w * tau) + x[2 + 2 * i] * math.sin(w * tau)
            e = h - values[s]
            sq += e * e
            mx = max(mx, abs(e))
        rms = math.sqrt(sq / n_s)

    mean = x[0]
    ab = [(x[1 + 2 * i], x[2 + 2 * i]) for i in range(n_c)]
    return mean, ab, rms, mx


def haversine_km(lat0, lon0, lat1, lon1):
    rl0, rl1 = math.radians(lat0), math.radians(lat1)
    dlat = rl1 - rl0
    dlon = math.radians(lon1 - lon0)
    a = math.sin(dlat / 2) ** 2 + math.cos(rl0) * math.cos(rl1) * math.sin(dlon / 2) ** 2
    return 2 * 6371.0 * math.asin(math.sqrt(a))


def box_km(lat, lon, box):
    """Distance (km) from a point to a lon/lat box; 0 inside."""
    clat = min(max(lat, box["lat0"]), box["lat1"])
    clon = min(max(lon, box["lon0"]), box["lon1"])
    return haversine_km(lat, lon, clat, clon)


def discover_place(box, radius_km, cache_dir, anchor=None):
    """Harmonic (type R) stations inside the box or within radius_km of it, nearest first
    (ties inside the box broken by distance to `anchor`, so index 0 -- TideModel's focus
    fallback -- is the station AT the place). The station list is CO-OPS's own metadata
    (mdapi stations.json?type=tidepredictions), cached forever like every other response.
    -> ([(id, on_river=False, name)], [the subordinate stations in range, not fitted: they
    publish offsets to a reference station, not constituents])."""
    data, _ = fetch_json(f"{MDAPI}/stations.json?type=tidepredictions",
                         os.path.join(cache_dir, "_lists", "tidepredictions.json"))
    anchor = anchor or ((box["lon0"] + box["lon1"]) / 2, (box["lat0"] + box["lat1"]) / 2)
    found, subordinate = [], []
    for st in data.get("stations", []):
        lat, lon = float(st.get("lat", 0)), float(st.get("lng", 0))
        d = box_km(lat, lon, box)
        if d > radius_km:
            continue
        if st.get("type") != "R":
            subordinate.append({"id": st["id"], "name": st.get("name", "?"),
                                "reference_id": st.get("reference_id"), "km": round(d, 1)})
            continue
        found.append((d, haversine_km(lat, lon, anchor[1], anchor[0]), st["id"],
                      st.get("name", st["id"])))
    found.sort()
    return [(sid, False, name) for _, _, sid, name in found], subordinate


def main():
    year = datetime.now(timezone.utc).year
    out_dir = os.path.join("data", "tides")
    cache_dir = os.path.join("cache", "coops")
    place = box = anchor = None
    radius_km = 0.0
    args = sys.argv[1:]
    while args:
        a = args.pop(0)
        if a == "--year":
            year = int(args.pop(0))
        elif a == "--out":
            out_dir = args.pop(0)
        elif a == "--cache":
            cache_dir = args.pop(0)
        elif a == "--place":
            place = args.pop(0)
        elif a == "--box":
            lon0, lat0, lon1, lat1 = (float(v) for v in args.pop(0).split(","))
            box = {"lon0": lon0, "lat0": lat0, "lon1": lon1, "lat1": lat1}
        elif a == "--radius-km":
            radius_km = float(args.pop(0))
        elif a == "--anchor":
            anchor = tuple(float(v) for v in args.pop(0).split(","))
        else:
            log(f"ignoring '{a}'")

    if place is None:
        harvest(STATIONS, year, out_dir, cache_dir)
        return
    if box is None:
        raise SystemExit("--place needs --box lon0,lat0,lon1,lat1")
    harvest_place(place, box, radius_km, year=year, cache_dir=cache_dir, anchor=anchor)


def harvest_place(place, box, radius_km, year=None, cache_dir=None, anchor=None, out_root=None):
    """The place mode: discover, fit, write data/tides/<place>/ (never overwriting)."""
    global _polite
    import polite
    _polite = polite
    year = year or datetime.now(timezone.utc).year
    cache_dir = cache_dir or os.path.join("cache", "coops")
    out_dir = os.path.join(out_root or os.path.join("data", "tides"), place)
    stations, subordinate = discover_place(box, radius_km, cache_dir, anchor)
    log(f"[{place}] {len(stations)} harmonic station(s) within {radius_km:g} km of {box}: "
        f"{[s[0] for s in stations]}; {len(subordinate)} subordinate station(s) have no "
        f"constituents of their own (not fitted)")
    rec = harvest(stations, year, out_dir, cache_dir, exclusive=True, tolerant=True)
    rec["subordinate"] = subordinate
    return rec


def harvest(stations, year, out_dir, cache_dir, exclusive=False, tolerant=False):
    """Fit every station and write stations.json + official sidecars into out_dir.
    exclusive: refuse to replace any file; tolerant: a station that fails is logged and
    skipped instead of ending the run. Returns {'files': [(path, bytes)], 'failed': [...]}."""
    epoch = datetime(year, 1, 1, tzinfo=timezone.utc).timestamp()
    os.makedirs(out_dir, exist_ok=True)
    wmode = "xb" if exclusive else "wb"
    if exclusive and os.path.exists(os.path.join(out_dir, "stations.json")):
        raise SystemExit(f"refusing to overwrite {out_dir}/stations.json (places only add)")
    log(f"harvest_tides: year {year}, epoch {epoch:.0f}, "
        f"{'numpy' if np is not None else 'pure-python'} solver")

    stations_out = []
    files = []
    failed = []
    prev = None
    river_km = 0.0

    for sid, on_river, short in stations:
        log(f"[{sid}] {short}")
        try:
            meta, _ = get_station_meta(sid, cache_dir)
            harcon, _ = get_harcon(sid, cache_dir)
            mllw_minus_navd = get_datums(sid, cache_dir)
            if mllw_minus_navd is not None:
                log(f"    MLLW - NAVD88 = {mllw_minus_navd:+.3f} m")
            if not harcon and tolerant:
                raise RuntimeError("no harmonic constituents published")
            times, values = get_predictions_year(sid, year, cache_dir)
        except Exception as e:  # noqa: BLE001
            if not tolerant or type(e).__name__ == "Stop":
                raise
            log(f"    SKIPPED {sid}: {e}")
            failed.append({"id": sid, "name": short, "why": str(e)})
            continue
        log(f"    {meta['name']} ({meta['lat']:.4f}, {meta['lon']:.4f}): "
            f"{len(harcon)} constituents, {len(times)} official hours")

        # Fit at the exact harcon frequencies (dropping sub-millimetre and duplicate speeds).
        use = []
        seen = set()
        for c in harcon:
            if c["amp_m"] < MIN_AMP_M or c["speed_dph"] in seen:
                continue
            seen.add(c["speed_dph"])
            use.append(c)
        omegas = [c["speed_dph"] * math.pi / 180.0 / 3600.0 for c in use]
        mean, ab, rms, mx = fit_station(times, values, omegas, epoch)
        log(f"    fit: {len(use)} constituents, mean {mean:.3f} m MLLW, "
            f"rms {rms * 1000:.1f} mm, max {mx * 1000:.1f} mm")

        coeffs = []
        warn = []
        for c, (a, b) in zip(use, ab):
            amp = math.hypot(a, b)
            phase = math.atan2(-b, a)   # h = amp * cos(w tau + phase)
            coeffs.append({"name": c["name"], "speed_dph": c["speed_dph"],
                           "amp_m": round(amp, 6), "phase_rad": round(phase, 6)})
            # Fitted amplitude vs published harcon amplitude: the ratio is the nodal factor for
            # the fit year and must sit in its physical range. Outside it, something is wrong.
            if c["amp_m"] > 0.05:
                ratio = amp / c["amp_m"]
                if not (0.70 <= ratio <= 1.30):
                    warn.append(f"{c['name']} fitted/harcon = {ratio:.2f}")
        if warn:
            log("    WARNING amplitude ratios out of nodal range: " + ", ".join(warn))
        top = sorted(zip(use, ab), key=lambda p: -math.hypot(*p[1]))[:4]
        log("    top: " + "  ".join(
            f"{c['name']}={math.hypot(a, b):.3f}m(x{math.hypot(a, b) / c['amp_m']:.3f})"
            for c, (a, b) in top))

        if on_river:
            if prev is not None:
                river_km += haversine_km(prev[0], prev[1], meta["lat"], meta["lon"])
            prev = (meta["lat"], meta["lon"])

        off_file = f"official_{sid}_{year}.f32"
        with open(os.path.join(out_dir, off_file), wmode) as f:
            f.write(struct.pack(f"<{len(values)}f", *values))
        files.append((os.path.join(out_dir, off_file), 4 * len(values)))

        stations_out.append({
            "id": sid, "name": short, "full_name": meta["name"],
            "lat": meta["lat"], "lon": meta["lon"],
            "mllw_minus_navd_m": mllw_minus_navd,
            "river_km": round(river_km, 3) if on_river else None,
            "mean_mllw_m": round(mean, 5),
            "fit_rms_m": round(rms, 5), "fit_max_m": round(mx, 5),
            "official_file": off_file,
            "official_start_unix": times[0], "official_dt_s": 3600, "official_n": len(values),
            "coeffs": coeffs,
        })

    out = {
        "generated_utc": datetime.now(timezone.utc).isoformat(timespec="seconds"),
        "source": "NOAA CO-OPS harcon frequencies + least-squares fit to official hourly "
                  "predictions (see this script's docstring)",
        "year": year, "epoch_unix": epoch,
        "stations": stations_out,
    }
    path = os.path.join(out_dir, "stations.json")
    with open(path, wmode.replace("b", ""), encoding="utf-8") as f:
        json.dump(out, f, indent=1)
    log(f"wrote {path} ({len(stations_out)} stations) -- the engine is now network-free")
    files.append((path, os.path.getsize(path)))
    return {"files": files, "failed": failed, "stations": stations_out, "year": year,
            "epoch_unix": epoch}


if __name__ == "__main__":
    main()
