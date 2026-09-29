#!/usr/bin/env python3
"""GAGAME Harvester: a PLACE, whole -- one name, one box, every source the Merrimack has.

    python harvester/harvest_place.py --place haulover                  (a named place)
    python harvester/harvest_place.py --place NAME --box lon0,lat0,lon1,lat1 --anchor lon,lat
                                      --sea lon,lat                     (any place)
    options: --steps a,b,...   plan tides currents waves ofs ofsfield ofscheck bathy survey
                               terms manifest (default: all, in the fetch order; a place with a
                               forecast system runs ofs + ofsfield in place of currents, because
                               currents.json is written once and carries the model field)
             --final           with 'manifest': write data/places/<NAME>.json (once, never
                               overwritten); without it the manifest is a draft in out/places/

WHY AN ORCHESTRATOR and not only --place flags: harvest_bathy and harvest_tides take a place
directly (their structure allowed it); harvest_currents and harvest_waves take one through
their harvest_place() functions. What does NOT fit any of them lives here:
  * the budget across sources (harvester/polite.py's ledger) and the plan that orders it;
  * NDBC discovery (which buoys near the place publish waves / ADCP right now);
  * the NOS forecast system on the NODD bucket (noaa-nos-ofs-pds) -- harvest_currents' field
    code is a GoMOFS THREDDS crawler with a Gulf-of-Maine grid; adapting it would be a rewrite;
  * the survey shoreline check (CUSP regional zips, read through their central directory);
  * what every source declares about itself (terms, citation) and the place MANIFEST.

A place only ADDS: every output carries the place's name, every write is exclusive-create, and
each step skips itself when its output already exists. Progress lives in
out/places/<NAME>.record.json (scratch) until --final writes the manifest.
"""

import json
import math
import os
import re
import struct
import sys
from datetime import datetime, timedelta, timezone

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import polite  # noqa: E402
import harvest_bathy  # noqa: E402
import harvest_currents  # noqa: E402
import harvest_tides  # noqa: E402
import harvest_waves  # noqa: E402

log = polite.log
DCACHE = r"D:\DataCache\GAGAME"
NDBC = "https://www.ndbc.noaa.gov"
OFS_BUCKET = "noaa-nos-ofs-pds"
SAFETY_BYTES = 30_000_000     # kept free of tiles for the run's own small requests

# Named places. A box is the area the place's grids cover; the anchor is the point the place is
# ABOUT (nearest-first order, the focus station); the sea point is the box's seaward side (the
# wave model's node and the buoys are chosen nearest it).
PLACES = {
    "haulover": {
        "box": {"lon0": -80.22, "lat0": 25.82, "lon1": -80.03, "lat1": 26.02},
        "box_why": "the coordinator's box kept as given: Bakers Haulover Inlet at its centre "
                   "latitude, the north of Biscayne Bay to the west, the shelf edge (~-80.03, "
                   "where the reef tract drops away) to the east; two CUDEM tiles (FL "
                   "n26X00/n26X25 w080X25) cover it",
        "anchor": (-80.1225, 25.9030),       # the inlet: CO-OPS 8723080 Haulover Pier, ACT8141
        "sea": (-80.03, 25.90),              # the box's east edge at the inlet's latitude
        "tide_radius_km": 40.0, "current_radius_km": 40.0, "max_current_stations": 8,
        "adcp_radius_km": 60.0, "ofs": None,
    },
    "chesapeake": {
        "box": {"lon0": -77.40, "lat0": 36.80, "lon1": -75.60, "lat1": 39.60},
        "box_why": "the coordinator's box kept: it holds the whole tidal bay from the Virginia "
                   "capes to the Susquehanna flats and the C&D canal, the tidal Potomac to "
                   "Washington and the James to Richmond's fall line (-77.43, 0.03 deg outside "
                   "the west edge); its east edge (-75.60) also takes in Delmarva's Atlantic "
                   "barrier islands, which the tile order (nearest Cape Henry first) reaches "
                   "only after the bay's mouth",
        "anchor": (-76.01, 36.93),           # Cape Henry, the bay's mouth
        "sea": (-75.75, 36.95),              # outside the mouth, where CDIP buoy 44099 sits
        "tide_radius_km": 0.0, "current_radius_km": 25.0, "max_current_stations": 8,
        "adcp_radius_km": 60.0, "ofs": "cbofs",
        # The whole bay at the usual step would be ~264 M cells (BathyModel refuses > 2^26), and
        # the budget fetches the tiles nearest the mouth first: grid the MOUTH -- Hampton Roads,
        # the capes and the Bridge-Tunnel, out to buoy 44099 -- from the six tiles that cover it.
        "grid_box": {"lon0": -76.50, "lat0": 36.80, "lon1": -75.75, "lat1": 37.25},
        "grid_name": "chesapeake_mouth",
    },
}

# What each source says about itself (part of every manifest). Pages are fetched once (cached),
# the quote is the first sentence carrying one of the patterns.
TERMS = {
    "noaa": {"url": "https://www.noaa.gov/disclaimer",
             "patterns": [r"public domain", r"copyright"]},
    "coops": {"url": "https://tidesandcurrents.noaa.gov/disclaimers.html",
              "patterns": [r"public domain", r"copyright", r"may be used", r"freely"]},
    "ndbc": {"url": "https://www.ndbc.noaa.gov/disclaimer.shtml",
             "patterns": [r"public domain", r"copyright", r"may be used"]},
    "nws": {"url": "https://www.weather.gov/disclaimer",
            "patterns": [r"public domain"]},
    "nodd_gfs": {"url": "https://registry.opendata.aws/noaa-gfs-bdp-pds/",
                 "patterns": [r"open to the public", r"can be used as desired", r"attribution"]},
    "nodd_ofs": {"url": "https://registry.opendata.aws/noaa-ofs/",
                 "patterns": [r"open to the public", r"can be used as desired", r"attribution"]},
    "nodd_lidar": {"url": "https://registry.opendata.aws/noaa-coastal-lidar/",
                   "patterns": [r"open to the public", r"can be used as desired",
                                r"attribution"]},
}


def now_utc():
    return datetime.now(timezone.utc)


def rel(path):
    """Repository-relative with forward slashes when under the repository; absolute otherwise
    (the big-data drive)."""
    a = os.path.abspath(path)
    if os.path.normcase(a).startswith(os.path.normcase(os.getcwd()) + os.sep):
        return os.path.relpath(a).replace("\\", "/")
    return a


# ------------------------------------------------------------------------------------ record

def record_path(place):
    return os.path.join("out", "places", f"{place}.record.json")


def load_record(place, p):
    path = record_path(place)
    if os.path.exists(path):
        with open(path, encoding="utf-8") as f:
            return json.load(f)
    return {"place": place, "box": p["box"], "anchor": p["anchor"], "sea": p["sea"],
            "steps": {}, "skipped": []}


def save_record(rec):
    path = record_path(rec["place"])
    os.makedirs(os.path.dirname(path), exist_ok=True)
    tmp = path + ".tmp"
    with open(tmp, "w", encoding="utf-8") as f:
        json.dump(rec, f, indent=1, default=str)
    os.replace(tmp, path)          # out/ is scratch: the record may be rewritten


def skip(rec, source, why):
    rec["skipped"] = [s for s in rec["skipped"] if s["source"] != source]
    rec["skipped"].append({"source": source, "why": why})
    log(f"SKIPPED {source}: {why}")


# ------------------------------------------------------------------------------------ NDBC

def ndbc_catalog():
    """(positions, files): the NDBC station table (cached list, shared with harvest_water) and
    the realtime2 directory listing -- which stations publish which files NOW. A listing
    younger than six hours is reused rather than fetched again."""
    tab, _ = polite.get_text(f"{NDBC}/data/stations/station_table.txt",
                             os.path.join("cache", "coops", "_lists", "ndbc_station_table.txt"))
    pos = {}
    for line in tab.splitlines():
        if line.startswith("#") or "|" not in line:
            continue
        f = [c.strip() for c in line.split("|")]
        m = re.match(r"([\d.]+)\s+([NS])\s+([\d.]+)\s+([EW])", f[6])
        if m:
            pos[f[0].upper()] = (float(m.group(1)) * (1 if m.group(2) == "N" else -1),
                                 float(m.group(3)) * (1 if m.group(4) == "E" else -1), f[4])
    ndir = os.path.join("cache", "ndbc")
    fresh = [n for n in sorted(os.listdir(ndir)) if n.startswith("_realtime2_listing_")]
    listing = None
    if fresh:
        stamp = fresh[-1][len("_realtime2_listing_"):-len(".html")]
        age = now_utc() - datetime.strptime(stamp, "%Y%m%d%H").replace(tzinfo=timezone.utc)
        if age < timedelta(hours=6):
            listing = os.path.join(ndir, fresh[-1])
    if listing is None:
        listing = os.path.join(ndir, f"_realtime2_listing_{now_utc():%Y%m%d%H}.html")
    html, _ = polite.get_text(f"{NDBC}/data/realtime2/", listing)
    files = {}
    for href in re.findall(r'href="([^"?/][^"]*)"', html):
        sid, _, ext = href.partition(".")
        files.setdefault(sid.upper(), set()).add(ext)
    return pos, files, listing


def km(lon0, lat0, lon1, lat1):
    return math.hypot((lon1 - lon0) * 111.32 * math.cos(math.radians((lat0 + lat1) / 2)),
                      (lat1 - lat0) * 110.57)


def nearest_ndbc(pos, files, point, want, n, radius_km):
    got = []
    for sid, (lat, lon, name) in pos.items():
        if want & files.get(sid, set()):
            d = km(point[0], point[1], lon, lat)
            if d <= radius_km:
                got.append((d, sid, name))
    return sorted(got)[:n]


# ------------------------------------------------------------------------------------ steps

def step_tides(place, p, rec):
    out = os.path.join("data", "tides", place, "stations.json")
    if os.path.exists(out):
        log(f"[tides] {out} exists; step already done")
        return
    r = harvest_tides.harvest_place(place, p["box"], p["tide_radius_km"], anchor=p["anchor"])
    rec["steps"]["tides"] = {
        "files": [(rel(a), b) for a, b in r["files"]], "failed": r["failed"],
        "subordinate": r["subordinate"], "year": r["year"], "epoch_unix": r["epoch_unix"],
        "stations": [{"id": s["id"], "name": s["full_name"], "lat": s["lat"], "lon": s["lon"],
                      "n_coeffs": len(s["coeffs"]), "fit_rms_m": s["fit_rms_m"],
                      "mllw_minus_navd_m": s["mllw_minus_navd_m"]} for s in r["stations"]]}


def step_currents(place, p, rec, field=None):
    out = os.path.join("data", "currents", place, "currents.json")
    if os.path.exists(out):
        log(f"[currents] {out} exists; step already done")
        return
    pos, files, listing = ndbc_catalog()
    adcp = nearest_ndbc(pos, files, p["anchor"], {"adcp"}, 1, p["adcp_radius_km"])
    if not adcp:
        skip(rec, "ndbc adcp", f"no NDBC station within {p['adcp_radius_km']:g} km of the "
                               f"anchor publishes realtime2 .adcp now")
    r = harvest_currents.harvest_place(place, p["box"], p["current_radius_km"],
                                       anchor=p["anchor"],
                                       max_stations=p["max_current_stations"],
                                       adcp=adcp[0][1] if adcp else None, field=field)
    rec["steps"]["currents"] = {"files": [(rel(a), b) for a, b in r["files"]],
                                "stations": r["stations"], "failed": r["failed"],
                                "adcp": ({"id": adcp[0][1], "km": round(adcp[0][0], 1),
                                          "obs_unix": (r["adcp"] or {}).get("obs_unix")}
                                         if adcp else None),
                                "field": bool(field)}


def step_waves(place, p, rec):
    out = os.path.join("data", "sea", place, "seastate.json")
    if os.path.exists(out):
        log(f"[waves] {out} exists; step already done")
        return
    pos, files, listing = ndbc_catalog()
    buoys = nearest_ndbc(pos, files, p["sea"], {"data_spec", "spec"}, 2, 250.0)
    log(f"[waves] nearest NDBC wave stations to {p['sea']}: "
        f"{[(b, round(d, 1)) for d, b, _ in buoys]}")
    r = harvest_waves.harvest_place(place, p["sea"], [b for _, b, _ in buoys])
    rec["steps"]["waves"] = {"files": [(rel(a), b) for a, b in r["files"]],
                             "node": r["node"], "cycle": r["cycle"],
                             "cycle_unix": r["cycle_unix"],
                             "buoys": [{"id": b, "km_from_sea_point": round(d, 1), "name": n,
                                        "obs_unix": r["buoys"].get(b)} for d, b, n in buoys]}


def ofs_vdatums(ofs):
    """The OFS's vertical-datum grid in OFS_Grid_Datum/ (a static folder: any cached listing
    of it is reused). -> {'key', 'size'} or None."""
    d = os.path.join("cache", "places", "_s3")
    old = sorted(n for n in os.listdir(d) if n.startswith("ofs_griddatum"))
    cache = os.path.join(d, old[-1] if old else "ofs_griddatum.xml")
    objs, _ = polite.s3_list(OFS_BUCKET, "OFS_Grid_Datum/", cache)
    return next((o for o in objs if o["key"].endswith(f"/{ofs}_vdatums.nc")), None)


def ofs_latest_cycle(ofs):
    """The newest cycle whose six nowcast field files and station nowcast are all published.
    -> (ymd, cyc, {name: size}, the day's S3 prefix)"""
    for back in range(0, 3):
        day = now_utc() - timedelta(days=back)
        prefix = f"{ofs}/netcdf/{day:%Y/%m/%d}/"
        cache = os.path.join("cache", "places", "_s3",
                             f"ofs_{ofs}_{day:%Y%m%d}_{now_utc():%Y%m%d%H}.xml")
        older = sorted(n for n in os.listdir(os.path.dirname(cache))
                       if n.startswith(f"ofs_{ofs}_{day:%Y%m%d}_"))
        if older and back > 0:
            cache = os.path.join(os.path.dirname(cache), older[-1])   # a past day is complete
        objs, _ = polite.s3_list(OFS_BUCKET, prefix, cache)
        sizes = {o["key"].rsplit("/", 1)[-1]: o["size"] for o in objs}
        for cyc in (18, 12, 6, 0):
            names = [f"{ofs}.t{cyc:02d}z.{day:%Y%m%d}.fields.n{i:03d}.nc" for i in range(1, 7)]
            names.append(f"{ofs}.t{cyc:02d}z.{day:%Y%m%d}.stations.nowcast.nc")
            if all(n in sizes for n in names):
                return f"{day:%Y%m%d}", cyc, {n: sizes[n] for n in names}, prefix
    raise RuntimeError(f"no complete {ofs} nowcast cycle in the last three days")


def step_ofs(place, p, rec, plan_only=False):
    ofs = p["ofs"]
    if not ofs:
        skip(rec, "nos ofs", "no NOS operational forecast system covers this place on the NODD "
                             "bucket (noaa-nos-ofs-pds lists cbofs ciofs creofs dbofs glofs "
                             "gomofs leofs lmhofs loofs lsofs necofs negofs ngofs ngofs2 nwgofs "
                             "nyofs sfbofs sjrofs sscofs tbofs wcofs; OFS_Grid_Datum/ holds a "
                             "secofs_vdatums.nc dated 2026-07-08, but there is no secofs output "
                             "folder)")
        return None
    ymd, cyc, files, prefix = ofs_latest_cycle(ofs)
    dest_dir = os.path.join(DCACHE, "ofs", ofs, ymd)
    plan = polite.Plan(f"{ofs.upper()} nowcast t{cyc:02d}z {ymd} (one cycle)")
    for name, size in files.items():
        plan.add(name, f"https://{OFS_BUCKET}.s3.amazonaws.com/{prefix}{name}", size,
                 os.path.join(dest_dir, name))
    vd = ofs_vdatums(ofs)
    if vd:
        plan.add(vd["key"].rsplit("/", 1)[-1],
                 f"https://{OFS_BUCKET}.s3.amazonaws.com/{vd['key']}", vd["size"],
                 os.path.join(DCACHE, "ofs", ofs, vd["key"].rsplit("/", 1)[-1]))
    total = plan.total(plan.items)
    if total > 1_000_000_000:
        log(f"[ofs] one cycle is {total / 1e9:.2f} GB > 1 GB: the station product only")
        plan.items = [i for i in plan.items if "stations" in i["key"] or "vdatums" in i["key"]]
    keep, cut = plan.fit()
    plan.print(keep, cut)
    rec["steps"].setdefault("ofs", {})
    rec["steps"]["ofs"].update({"cycle": f"{ymd} t{cyc:02d}z", "plan": plan.items,
                                "later": [i["key"] for i in cut]})
    if plan_only:
        return None
    raw = []
    for it in keep:
        polite.download(it["url"], it["dest"], it["size"], note=f"{ofs} {place}")
        raw.append((it["dest"], os.path.getsize(it["dest"]), it["url"]))
    rec["steps"]["ofs"]["raw"] = raw
    return {"ymd": ymd, "cyc": cyc, "dir": dest_dir, "raw": raw}


def _scatter(grid, lon, lat, vals, mask):
    """Nearest-cell scatter with the 2x2 splat of harvest_currents.scatter, onto `grid`."""
    from array import array
    out = array("f", [-999.0]) * (grid["nx"] * grid["ny"])
    for k in range(len(vals)):
        if mask is not None and not (mask[k] > 0.5):
            continue
        x = vals[k]
        if x != x or abs(x) > 100:
            continue
        gx = int((lon[k] - grid["lon0"]) / grid["dlon"])
        gy = int((lat[k] - grid["lat0"]) / grid["dlat"])
        for oy in (0, 1):
            for ox in (0, 1):
                xx, yy = gx + ox, gy + oy
                if 0 <= xx < grid["nx"] and 0 <= yy < grid["ny"]:
                    out[yy * grid["nx"] + xx] = x
    return out


def _gap_fill(grid, g, sweeps):
    from array import array
    for sweep in range(sweeps):
        need = 2 if sweep == 0 else 1
        src = array("f", g)
        for gy in range(grid["ny"]):
            for gx in range(grid["nx"]):
                k = gy * grid["nx"] + gx
                if src[k] > -900:
                    continue
                s, n = 0.0, 0
                for oy, ox in ((0, 1), (0, -1), (1, 0), (-1, 0)):
                    yy, xx = gy + oy, gx + ox
                    if 0 <= yy < grid["ny"] and 0 <= xx < grid["nx"]:
                        kk = yy * grid["nx"] + xx
                        if src[kk] > -900:
                            s += src[kk]
                            n += 1
                if n >= need:
                    g[k] = s / n


def convert_ofs(ofs, ofs_dir, cycle_files, box, place):
    """The newest nowcast field of a ROMS-based NOS OFS -> the engine's currents field (the
    GoMOFS recipe of harvest_currents: surface u and v scattered from their own staggered grids,
    `angle` from the rho grid, rotated to east/north on a regular lat/lon grid; -999 = no
    water). Also returns what the file declares (units, datum, time). netCDF-4 -> hdf5.py."""
    import hdf5
    from array import array
    fields = sorted(n for n in cycle_files if ".fields.n" in n)
    path = os.path.join(ofs_dir, fields[-1])
    f, var, gatts = hdf5.open_netcdf4(path)
    need = ["u", "v", "lon_u", "lat_u", "lon_v", "lat_v", "mask_u", "mask_v", "angle",
            "lon_rho", "lat_rho", "ocean_time"]
    missing = [n for n in need if n not in var]
    if missing:
        raise RuntimeError(f"{fields[-1]}: missing {missing} (has {sorted(var)[:40]})")

    def surface(name):
        vals, shape = var[name].read()
        ny, nx = shape[-2], shape[-1]
        return vals[len(vals) - ny * nx:], ny, nx        # last time, last (top) s-level

    u, uny, unx = surface("u")
    v, vny, vnx = surface("v")
    rd = {n: var[n].read()[0] for n in ("lon_u", "lat_u", "lon_v", "lat_v", "mask_u", "mask_v",
                                        "angle", "lon_rho", "lat_rho")}
    t, _ = var["ocean_time"].read()
    lon_r, lat_r = rd["lon_rho"], rd["lat_rho"]
    # grid: the model's water extent inside the box, at about the model's own spacing
    wet = [(lon_r[k], lat_r[k]) for k in range(len(lon_r))
           if box["lon0"] <= lon_r[k] <= box["lon1"] and box["lat0"] <= lat_r[k] <= box["lat1"]]
    lon0 = max(box["lon0"], min(p[0] for p in wet))
    lon1 = min(box["lon1"], max(p[0] for p in wet))
    lat0 = max(box["lat0"], min(p[1] for p in wet))
    lat1 = min(box["lat1"], max(p[1] for p in wet))
    step = 0.005
    grid = {"lon0": lon0, "lat0": lat0, "dlon": step, "dlat": step,
            "nx": int((lon1 - lon0) / step) + 1, "ny": int((lat1 - lat0) / step) + 1}
    gu = _scatter(grid, rd["lon_u"], rd["lat_u"], u, rd["mask_u"])
    gv = _scatter(grid, rd["lon_v"], rd["lat_v"], v, rd["mask_v"])
    ga = _scatter(grid, lon_r, lat_r, rd["angle"], None)
    for g in (gu, gv, ga):
        _gap_fill(grid, g, 3)
    n = grid["nx"] * grid["ny"]
    out_u = array("f", [-999.0]) * n
    out_v = array("f", [-999.0]) * n
    filled = 0
    for k in range(n):
        uu, vv, aa = gu[k], gv[k], ga[k]
        if uu > -900 and vv > -900:
            a = aa if aa > -900 else 0.0
            out_u[k] = uu * math.cos(a) - vv * math.sin(a)
            out_v[k] = uu * math.sin(a) + vv * math.cos(a)
            filled += 1
    tunits = var["ocean_time"].attrs.get("units", "")
    valid = None
    m = re.match(r"seconds since (\d{4}-\d{2}-\d{2})[ T](\d{2}:\d{2}:\d{2})", tunits or "")
    if m:
        t0 = datetime.strptime(m.group(1) + " " + m.group(2), "%Y-%m-%d %H:%M:%S")
        valid = (t0.replace(tzinfo=timezone.utc) + timedelta(seconds=t[-1])).isoformat()
    declared = {
        "file": fields[-1], "ocean_time_units": tunits, "valid_utc": valid,
        "u_units": var["u"].attrs.get("units"), "u_long_name": var["u"].attrs.get("long_name"),
        "v_units": var["v"].attrs.get("units"),
        "zeta_units": var["zeta"].attrs.get("units") if "zeta" in var else None,
        "zeta_long_name": var["zeta"].attrs.get("long_name") if "zeta" in var else None,
        "angle_units": var["angle"].attrs.get("units"),
        "global": {k: gatts.get(k) for k in ("title", "type", "Conventions", "history",
                                              "grd_file", "source", "institution")
                   if gatts.get(k) is not None},
        "model_grid": {"u": [uny, unx], "v": [vny, vnx], "rho": list(var["lon_rho"].shape)},
    }
    log(f"[ofs] {fields[-1]}: surface u/v rotated to east/north, {filled} of {n} cells water "
        f"on a {grid['nx']}x{grid['ny']} grid at {step} deg; valid {valid}")
    field = {"file": f"{ofs}_uv.f32", "nx": grid["nx"], "ny": grid["ny"],
             "lon0": grid["lon0"], "lat0": grid["lat0"], "dlon": step, "dlat": step,
             "source": f"{ofs.upper()} {fields[-1]} (NODD noaa-nos-ofs-pds), surface level",
             "valid_utc": valid, "u": out_u, "v": out_v}
    return field, declared


def vdatums_declared(path):
    """What an OFS_Grid_Datum/*_vdatums.nc (classic netCDF-3) says about the model's vertical
    reference: its global attributes and each datum-difference variable's name and unit."""
    import netcdf3
    with open(path, "rb") as f:
        nc = netcdf3.NetCDF3(f.read())
    strip = lambda s: s.strip() if isinstance(s, str) else s  # noqa: E731
    return {"global": {k: strip(v) for k, v in nc.gatts.items()},
            "variables": {n: {k: strip(v.atts.get(k)) for k in ("long_name", "unit")}
                          for n, v in nc.vars.items()}}


def step_ofsfield(place, p, rec):
    """Convert the fetched nowcast into the engine's currents field and write the place's
    currents.json WITH it (currents.json is written once, so this step replaces 'currents'
    for a place that has a forecast system)."""
    if os.path.exists(os.path.join("data", "currents", place, "currents.json")):
        log(f"[ofsfield] data/currents/{place}/currents.json exists; step already done")
        return
    ofs = rec["steps"].get("ofs", {})
    raw = ofs.get("raw") or []
    if not raw:
        raise RuntimeError("run the ofs step first")
    names = [os.path.basename(r[0]) for r in raw]
    field_dir = os.path.dirname(next(r[0] for r in raw if ".fields." in r[0]))
    field, declared = convert_ofs(p["ofs"], field_dir, names, p["box"], place)
    vd = next((r[0] for r in raw if r[0].endswith("_vdatums.nc")), None)
    if vd:
        declared["vdatums"] = vdatums_declared(vd)
    rec["steps"]["ofs"]["declared"] = declared
    step_currents(place, p, rec, field=field)


def step_ofscheck(place, p, rec, n_stations=5):
    """An independent check of the netCDF-4 decode: the model's zeta at the place's harmonic
    tide stations nearest the anchor, hour by hour through the nowcast, beside CO-OPS's own
    astronomical prediction moved from MLLW to MSL with the station's published datums. Same
    phase and a steady offset (the non-tidal water level) says the bytes were read right."""
    import hdf5
    raw = [r[0] for r in rec["steps"]["ofs"]["raw"] if ".fields.n" in r[0]]
    stations = [s for s in rec["steps"]["tides"]["stations"]][:n_stations]
    rows = []
    for path in sorted(raw):
        f, var, _ = hdf5.open_netcdf4(path)
        t, _ = var["ocean_time"].read()
        m = re.match(r"seconds since (\d{4})-(\d{2})-(\d{2})", var["ocean_time"].attrs["units"])
        when = datetime(int(m.group(1)), int(m.group(2)), int(m.group(3)),
                        tzinfo=timezone.utc) + timedelta(seconds=t[-1])
        z, _ = var["zeta"].read()
        lon, _ = var["lon_rho"].read()
        lat, _ = var["lat_rho"].read()
        mask, _ = var["mask_rho"].read()
        row = {"valid_utc": when.isoformat(), "stations": {}}
        for s in stations:
            k = min((i for i in range(len(lon)) if mask[i] > 0.5),
                    key=lambda i: (lon[i] - s["lon"]) ** 2 + (lat[i] - s["lat"]) ** 2)
            dat = _json(os.path.join("cache", "coops", s["id"], "datums_metric.json")) or {}
            dv = {d["name"]: d["value"] for d in dat.get("datums", [])}
            pred = _json(os.path.join("cache", "coops", s["id"],
                                      f"pred_{when.year}_all.json")) or {}
            key = when.strftime("%Y-%m-%d %H:%M")
            pv = next((float(x["v"]) for x in pred.get("predictions", []) if x["t"] == key),
                      None)
            rel_msl = (round(pv - (dv["MSL"] - dv["MLLW"]), 3)
                       if pv is not None and "MSL" in dv and "MLLW" in dv else None)
            row["stations"][s["id"]] = {"model_zeta_m": round(z[k], 3),
                                        "coops_prediction_vs_msl_m": rel_msl}
        rows.append(row)
    diffs = [v["model_zeta_m"] - v["coops_prediction_vs_msl_m"] for r in rows
             for v in r["stations"].values() if v["coops_prediction_vs_msl_m"] is not None]
    rec["steps"]["ofs"]["check"] = {
        "what": "model zeta at the nearest wet rho point vs CO-OPS prediction (MLLW -> MSL via "
                "the station's datums.json) at each nowcast hour",
        "rows": rows,
        "model_minus_prediction_m": {"min": round(min(diffs), 3), "max": round(max(diffs), 3),
                                     "mean": round(sum(diffs) / len(diffs), 3)}}
    log(f"[ofscheck] model - prediction over {len(diffs)} station-hours: "
        f"{rec['steps']['ofs']['check']['model_minus_prediction_m']}")


def step_bathy(place, p, rec, plan_only=False, reserve=SAFETY_BYTES):
    """Tiles nearest the anchor first, inside the budget (a later run with a fresh ledger takes
    the rest: tiles on disk are never requested again), then the grid -- once."""
    name = p.get("grid_name", place)
    r = harvest_bathy.harvest_place(place, p["box"], near=p["anchor"], plan_only=plan_only,
                                    reserve=reserve, grid_box=p.get("grid_box"), out_name=name)
    prev = rec["steps"].get("bathy", {})
    rec["steps"]["bathy"] = {"plan": r["plan"], "later": r["later"],
                             "files": [(rel(a), b) for a, b in r["files"]] or prev.get("files",
                                                                                   []),
                             "declared": r.get("declared") or prev.get("declared", {})}


def _zip_directory(tail, total):
    """Central directory entries from a zip's tail bytes: [(name, csize, usize, offset)]."""
    eocd = tail.rfind(b"PK\x05\x06")
    if eocd < 0:
        raise RuntimeError("no end-of-central-directory record in the tail")
    n, cd_size, cd_off = struct.unpack("<HII", tail[eocd + 10:eocd + 20])
    if cd_off == 0xFFFFFFFF:
        raise RuntimeError("zip64 archive (central directory past 4 GB)")
    base = total - len(tail)
    if cd_off < base:
        raise RuntimeError(f"central directory ({cd_size} B) does not fit the tail read")
    p = cd_off - base
    out = []
    for _ in range(n):
        if tail[p:p + 4] != b"PK\x01\x02":
            break
        csize, usize = struct.unpack("<II", tail[p + 20:p + 28])
        ln, le, lc = struct.unpack("<HHH", tail[p + 28:p + 34])
        off = struct.unpack("<I", tail[p + 42:p + 46])[0]
        out.append((tail[p + 46:p + 46 + ln].decode("cp437"), csize, usize, off))
        p += 46 + ln + le + lc
    return out


def step_survey(place, p, rec):
    """harvest_survey.py's shoreline source (NGS CUSP regional zips) for the place, if cheap:
    the zip that covers the place is inspected through its central directory (one suffix
    Range request); a member is fetched only if the covering data is a modest part of it."""
    rec["skipped"] = [s for s in rec["skipped"] if s["source"] != "cusp shoreline"]
    if place == "chesapeake":
        skip(rec, "cusp shoreline", "not requested for the Chesapeake; the covering archive in "
                                    "the listing is DE_MD_DC_VA_Inclusive_20250930.zip (100M)")
        return
    if place != "haulover":
        skip(rec, "cusp shoreline", "not requested for this place")
        return
    lst = sorted(n for n in os.listdir(os.path.join("cache", "places", "_cusp"))
                 if n.startswith("dist_shoreline_"))
    html = open(os.path.join("cache", "places", "_cusp", lst[-1]), encoding="utf-8").read()
    zips = {}
    for m in re.finditer(r'href="([^"/]+\.zip)".*?<td[^>]*>\s*([\d.]+[KMG])\s*</td>', html):
        zips[m.group(1)] = m.group(2)
    log(f"[survey] CUSP regional zips: {zips}")
    cands = [z for z in zips if z.lower().startswith(("southeast", "fif"))]
    report = {"listing": rel(os.path.join("cache", "places", "_cusp", lst[-1])), "zips": zips,
              "inspected": {}}
    for zname in cands:
        url = f"https://geodesy.noaa.gov/dist_shoreline/{zname}"
        tail, total, _ = polite.get_tail(url, 262144, os.path.join(
            "cache", "places", "_cusp", f"{zname}.tail"))
        try:
            entries = _zip_directory(tail, total)
        except RuntimeError as e:
            report["inspected"][zname] = {"bytes": total, "error": str(e)}
            continue
        report["inspected"][zname] = {"bytes": total, "members": [
            {"name": n, "compressed": c, "size": u} for n, c, u, _ in entries]}
        log(f"[survey] {zname}: {total / 1e6:.1f} MB, {len(entries)} members: "
            f"{[(n, round(c / 1e6, 1)) for n, c, u, _ in entries][:12]}")
    rec["steps"]["survey"] = report
    # Florida's Atlantic shoreline is in Southeast_Caribbean.zip (FIF is inland, per state, and
    # has no Florida member). Cheap means a covering shapefile member under the planning
    # threshold; a deflate member cannot be read in part.
    se = report["inspected"].get("Southeast_Caribbean.zip", {})
    shp = [m for m in se.get("members", []) if m["name"].lower().endswith(".shp")]
    if shp and max(m["compressed"] for m in shp) > polite.BIG_BYTES:
        skip(rec, "cusp shoreline",
             f"not cheap: Florida's Atlantic shoreline is one {shp[0]['compressed'] / 1e6:.1f} MB "
             f"deflate member (Southeast_Caribbean.shp) of the {se['bytes'] / 1e6:.1f} MB "
             f"regional zip, and a deflate member cannot be read in part; FIF_20260908.zip is "
             f"per-state inland (22 states, no Florida). Read through the zips' central "
             f"directories only (two suffix-range requests)")
    return report


def step_terms(rec):
    """Each source's own statement of terms: the page is fetched once (cached) and the first
    sentence matching the source's patterns is kept with its URL."""
    out = {}
    before = dict(rec.get("terms", {}))
    rdir = os.path.join("out", "places")
    for fn in sorted(os.listdir(rdir)):              # a page that refused ANY place's run
        if fn.endswith(".record.json"):              # is not asked again for another
            other = _json(os.path.join(rdir, fn)) or {}
            for k, v in (other.get("terms") or {}).items():
                if "error" in v and k not in before:
                    before[k] = v
    for key, t in TERMS.items():
        cache = os.path.join("cache", "places", "_terms", re.sub(r"[^A-Za-z0-9]+", "_",
                                                               t["url"]) + ".html")
        if "error" in before.get(key, {}) and not os.path.exists(cache):
            out[key] = before[key]           # it refused once: do not ask again
            log(f"[terms] {key}: refused earlier ({before[key]['error']}); not asked again")
            continue
        try:
            html, _ = polite.get_text(t["url"], cache)
        except polite.Stop:
            raise
        except Exception as e:  # noqa: BLE001
            out[key] = {"url": t["url"], "error": str(e)[:200]}
            continue
        html = re.sub(r"<(script|style)[^>]*>.*?</\1>", " ", html, flags=re.S | re.I)
        quote = None
        # Search block by block (paragraphs, list items, cells) so a heading or a menu never
        # glues itself onto the sentence; keep the sentence that carries the pattern.
        blocks = re.split(r"<(?:p|li|td|th|div|h[1-6]|dd|dt|br|section|article)\b[^>]*>",
                          html, flags=re.I)
        for pat in t["patterns"]:
            for blk in blocks:
                text = re.sub(r"<[^>]+>", " ", blk)
                text = re.sub(r"&nbsp;|&#160;", " ", text).replace("&amp;", "&")
                text = re.sub(r"\s+", " ", text).strip()
                if not re.search(pat, text, re.I):
                    continue
                for sent in re.split(r"(?<=[.!?])\s+(?=[A-Z])", text):
                    if re.search(pat, sent, re.I):
                        quote = sent.strip()
                        break
                if quote:
                    break
            if quote:
                break
        out[key] = {"url": t["url"], "quote": quote}
        log(f"[terms] {key}: {quote!r}")
    rec["terms"] = out


def step_plan(place, p, rec):
    """Everything the place needs, with sizes and the total, BEFORE anything large is fetched:
    small products as request counts with measured per-request sizes, large ones (the forecast
    system's files, the CUDEM tiles) with their exact sizes from the NODD listings, in fetch
    order, and where the byte budget cuts the list."""
    small = []
    stations, sub = harvest_tides.discover_place(p["box"], p["tide_radius_km"],
                                                 os.path.join("cache", "coops"), p["anchor"])
    small.append({"what": f"CO-OPS tide stations: meta + harcon + datums + one year of hourly "
                          f"predictions each ({len(stations)} harmonic; {len(sub)} subordinate "
                          f"listed only)", "requests": 4 * len(stations),
                  "bytes": len(stations) * 375_000})
    picks = harvest_currents.discover_place(p["box"], p["current_radius_km"], p["anchor"],
                                            p["max_current_stations"],
                                            os.path.join("cache", "coops"))
    small.append({"what": f"CO-OPS current predictions, 45 days MAX_SLACK in two requests "
                          f"({len(picks)} stations nearest the anchor)",
                  "requests": 2 * len(picks), "bytes": len(picks) * 2 * 25_000})
    small.append({"what": "GFS-Wave point via the NOMADS grib filter: one cycle probe + 13 "
                          "forecast hours (f000..f048 by 4)", "requests": 14, "bytes": 14 * 12_000})
    small.append({"what": "NDBC realtime2: listing (reused <6 h) + two wave buoys (.txt, "
                          ".data_spec) + an ADCP buoy", "requests": 6, "bytes": 600_000 + 5 * 60_000})
    small.append({"what": "each source's terms page (7 pages)", "requests": 7, "bytes": 7 * 80_000})
    log(f"PLAN {place}: small products")
    for s in small:
        log(f"  {s['requests']:5d} req  {s['bytes'] / 1e6:8.2f} MB  {s['what']}")
    small_req = sum(s["requests"] for s in small)
    small_bytes = sum(s["bytes"] for s in small)
    ofs_plan = None
    if p["ofs"]:
        ymd, cyc, files, prefix = ofs_latest_cycle(p["ofs"])
        vd = ofs_vdatums(p["ofs"])
        if vd:
            files = dict(files)
            files[vd["key"].rsplit("/", 1)[-1]] = vd["size"]
        ofs_plan = {"cycle": f"{ymd} t{cyc:02d}z", "files": files,
                    "bytes": sum(files.values())}
        log(f"PLAN {place}: {p['ofs'].upper()} nowcast t{cyc:02d}z {ymd} + its vertical-datum "
            f"grid: {len(files)} files, {ofs_plan['bytes'] / 1e6:.1f} MB")
        for n, s in files.items():
            log(f"        {n:48s} {s / 1e6:8.1f} MB")
    cache_dir = harvest_bathy.BIGCACHE
    index = harvest_bathy.read_tile_index(polite.get(
        harvest_bathy.INDEX_URL, os.path.join(cache_dir, harvest_bathy.INDEX_NAME))[0])
    needed, tplan = harvest_bathy.plan_place(place, p["box"], p["anchor"], cache_dir, index,
                                             polite)
    reserve = small_bytes + (ofs_plan["bytes"] if ofs_plan else 0) + SAFETY_BYTES
    keep, cut = tplan.fit(reserve=reserve)
    tplan.print(keep, cut)
    total = reserve + tplan.total(tplan.items)
    left = polite.remaining()
    log(f"PLAN {place} TOTAL: {total / 1e9:.3f} GB and ~{small_req + len(tplan.items) + 10} "
        f"requests for everything; the budget has {left[0] / 1e9:.3f} GB and {left[1]} "
        f"requests left, so {len(cut)} tile(s) ({tplan.total(cut) / 1e9:.3f} GB) wait for a "
        f"later run")
    rec["plan"] = {"made_utc": now_utc().isoformat(timespec="seconds"), "small": small,
                   "ofs": ofs_plan, "tiles": tplan.items, "tiles_later": [i["key"] for i in cut],
                   "total_bytes": total, "budget_left_bytes": left[0],
                   "budget_left_requests": left[1]}


# ------------------------------------------------------------------------------------ manifest

CUDEM_META = os.path.join(DCACHE, "cudem", "ncei_nintharcsec_dem_m8483_met.xml")
CUDEM_META_URL = ("https://noaa-nos-coastal-lidar-pds.s3.amazonaws.com/dem/"
                  "NCEI_ninth_Topobathy_2014_8483/ncei_nintharcsec_dem_m8483_met.xml")
# GRIB2 discipline 10 (oceanographic) category 0 (waves) and discipline 0 category 2
# (momentum) as the WMO code table 4.2 declares them -- the keys harvest_waves learned.
GRIB_UNITS = {"HTSGW": "m", "PERPW": "s", "DIRPW": "degree true (from)", "WVHGT": "m",
              "WVPER": "s", "WVDIR": "degree true (from)", "SWELL": "m", "SWPER": "s",
              "SWDIR": "degree true (from)", "WIND": "m s-1", "WDIR": "degree true (from)"}


def _ledger_entries():
    out = []
    if os.path.exists(polite.LEDGER):
        with open(polite.LEDGER, encoding="utf-8") as f:
            out = [json.loads(ln) for ln in f if ln.strip()]
    return out


def _written_files(entries):
    """Every file a ledger request wrote: its `dest` when the ledger has it, otherwise the file
    whose size equals the bytes received and whose mtime is within seconds of the request's
    end (each response is written the moment it arrives)."""
    t0 = min((e["t_end"] for e in entries), default=0) - 120
    roots = [os.path.join("cache", d) for d in ("coops", "currents", "ndbc", "gfswave",
                                                  "places")]
    roots += [os.path.join(DCACHE, "cudem"), os.path.join(DCACHE, "ofs")]
    disk = []
    for r in roots:
        for dp, _, fns in os.walk(r):
            for fn in fns:
                pth = os.path.join(dp, fn)
                st = os.stat(pth)
                if st.st_mtime >= t0:
                    disk.append((pth, st.st_size, st.st_mtime))
    used = set()
    out = []
    for e in entries:
        if not e.get("bytes") or e.get("error") or e.get("method") == "HEAD":
            continue
        dest = e.get("dest")
        if not dest:
            best = None
            for pth, size, mt in disk:
                if pth in used or size != e["bytes"]:
                    continue
                dt = abs(mt - e["t_end"])
                if dt < 10 and (best is None or dt < best[1]):
                    best = (pth, dt)
            dest = best[0] if best else None
        if dest:
            used.add(dest)
        out.append({"path": rel(dest) if dest else None, "bytes": e["bytes"], "url": e["url"],
                    "fetched_utc": e["t"], "range": e.get("range"), "place": e.get("place")})
    return out


def _json(path):
    try:
        with open(path, encoding="utf-8") as f:
            return json.load(f)
    except (OSError, ValueError):
        return None


def _tide_declared(sid):
    d = os.path.join("cache", "coops", sid)
    dat = _json(os.path.join(d, "datums_metric.json")) or {}
    har = _json(os.path.join(d, "harcon_metric.json")) or {}
    vals = {x.get("name"): x.get("value") for x in dat.get("datums", [])}
    return {"harcon_units": har.get("units"), "datums_units": dat.get("units"),
            "datum_epoch": dat.get("epoch"), "orthometric_datum": dat.get("OrthometricDatum"),
            "datums_relative_to": "STND (station datum) as published",
            "MLLW": vals.get("MLLW"), "MSL": vals.get("MSL"), "NAVD88": vals.get("NAVD88")}


def _s3_modified(key_tail):
    """LastModified of a key from any cached NODD listing (the listing is what declared it)."""
    d = os.path.join("cache", "places", "_s3")
    for fn in sorted(os.listdir(d)):
        with open(os.path.join(d, fn), encoding="utf-8", errors="replace") as f:
            xml = f.read()
        m = re.search(r"<Key>[^<]*" + re.escape(key_tail) +
                      r"</Key><LastModified>([^<]+)</LastModified>", xml)
        if m:
            return m.group(1)
    return None


def _raw_declared(path, steps):
    """Datum/units/valid-instant of a raw download as its source declares it."""
    name = os.path.basename(path)
    if name.endswith(".tif"):
        return {"datum": "vertical NAVD88 (dataset metadata), horizontal NAD83 (GeoKeys)",
                "units": "m", "valid": f"static: CUDEM tile version {name.rsplit('_', 1)[-1][:-4]}"
                                       f", bucket LastModified {_s3_modified(name)}"}
    if ".fields.n" in name:
        rows = (steps.get("ofs") or {}).get("check", {}).get("rows", [])
        fields = sorted(r[0] for r in (steps.get("ofs") or {}).get("raw", [])
                        if ".fields.n" in r[0])
        valid = next((rows[i]["valid_utc"] for i, f in enumerate(fields)
                      if os.path.basename(f) == name and i < len(rows)), None)
        return {"datum": "zeta 'free-surface' (no datum attribute; the model's datum grid "
                         "relates NAVD88/MLLW/... to mean sea level)",
                "units": "zeta 'meter', u/v 'meter second-1', angle 'radians'",
                "valid": f"{valid} (ocean_time, seconds since 2016-01-01 00:00:00)"}
    if ".stations.nowcast" in name:
        try:
            import hdf5
            f, var, _ = hdf5.open_netcdf4(path)
            t, _ = var["ocean_time"].read()
            t0 = datetime(2016, 1, 1, tzinfo=timezone.utc)
            valid = (f"{(t0 + timedelta(seconds=t[0])).isoformat()}.."
                     f"{(t0 + timedelta(seconds=t[-1])).isoformat()}, {len(t)} times "
                     f"({var['ocean_time'].attrs.get('units')})")
        except Exception as e:  # noqa: BLE001
            valid = f"not read ({e})"
        return {"datum": "as the fields files", "units": "as the fields files", "valid": valid}
    if name.endswith("_vdatums.nc"):
        return {"datum": "differences to mean sea level (navd88tomsl, mhhwtomsl, mhwtomsl, "
                         "mlwtomsl, mllwtomsl), NAD83 positions", "units": "meters",
                "valid": "static: 'File created: 09-Aug-2010' (its creation_date attribute); "
                         f"bucket LastModified {_s3_modified(name)}"}
    if name.endswith(".zip") or name.endswith(".xml"):
        return {"valid": f"bucket LastModified {_s3_modified(name)}"}
    return {}


def build_manifest(place, p, rec):
    entries = _ledger_entries()
    mine = [e for e in entries if e.get("place") == place]
    shared = [e for e in entries if e.get("place") == "discovery"]
    written = _written_files(mine + shared)
    by_url = {}
    for w in written:
        by_url.setdefault(w["url"], w)
    outputs = []
    steps = rec.get("steps", {})

    b = steps.get("bathy")
    if b and b.get("files"):
        meta = _json(os.path.join("data", "bathy", f"{p.get('grid_name', place)}.json")) or {}
        tiles = meta.get("tiles", [])
        for pth, size in b["files"]:
            outputs.append({
                "path": pth, "bytes": size, "kind": "CUDEM topobathy grid (BathyModel format)",
                "source_url": [t["url"] for t in tiles] + [harvest_bathy.INDEX_URL],
                "datum": "vertical NAVD88 height, metres -- declared by the dataset metadata "
                         "(ncei_nintharcsec_dem_m8483_met.xml: 'The DEMs are referenced "
                         "vertically to the North American Vertical Datum of 1988. The vertical "
                         "units of the DEMs are meters.'); horizontal NAD83 geographic "
                         "(GeographicType 4269 in each tile's GeoKeys; index srs "
                         f"{sorted({t['srs_index'] for t in tiles})}); the tiles carry no "
                         "vertical GeoKey", "units": "m (GDAL UNITTYPE 'metre' in each tile)",
                "valid": f"static: CUDEM tile versions "
                         f"{sorted({t['name'].rsplit('_', 1)[-1][:-4] for t in tiles})}",
                "grid": {k: meta.get(k) for k in ("nx", "ny", "lon0", "lat1", "dlon", "dlat",
                                                  "step", "row0", "nodata")}})

    t = steps.get("tides")
    if t:
        decl = {s["id"]: _tide_declared(s["id"]) for s in t["stations"]}
        for pth, size in t["files"]:
            sid = re.search(r"official_(\w+)_\d{4}\.f32", pth)
            ids = [sid.group(1)] if sid else [s["id"] for s in t["stations"]]
            outputs.append({
                "path": pth, "bytes": size,
                "kind": ("official hourly predictions, float32 (TideModel sidecar)" if sid else
                         "tide constituent fits (TideModel format: stations.json)"),
                "source_url": sorted(u for u in by_url if any(
                    f"/stations/{i}" in u or f"station={i}&" in u for i in ids)),
                "datum": "heights in metres above each station's MLLW (requests datum=MLLW, "
                         "units=metric); mllw_minus_navd_m from each station's datums.json "
                         "where it publishes NAVD88",
                "declared": {i: decl[i] for i in ids},
                "units": "m",
                "valid": f"fit year {t['year']} (epoch_unix {t['epoch_unix']:.0f} = "
                         f"{t['year']}-01-01T00:00Z); hourly predictions "
                         f"{t['year']}-01-01..{t['year']}-12-31 UTC"})

    c = steps.get("currents")
    if c:
        cj = _json(os.path.join("data", "currents", place, "currents.json")) or {}
        units = set()
        for w in written:
            if "currents_predictions" in w["url"] and w["path"]:
                units.add((_json(w["path"]) or {}).get("current_predictions", {}).get("units"))
        ev = [e["t"] for s in cj.get("tidal_stations", []) for e in s.get("events", [])]
        for pth, size in c["files"]:
            is_field = pth.endswith(".f32")
            outputs.append({
                "path": pth, "bytes": size,
                "kind": ("model surface current field, float32 u then v (CurrentModel field)"
                         if is_field else "tidal current clocks + ADCP (CurrentModel format)"),
                "source_url": ([rr[2] for rr in steps.get("ofs", {}).get("raw", [])
                                if ".fields." in rr[2]][-1:] if is_field else
                               sorted(u for u in by_url if "currents_predictions" in u or
                                      u.endswith(".adcp"))),
                "datum": ("depth: the model's top sigma level (surface)" if is_field else
                          "CO-OPS declares units " + ", ".join(sorted(x for x in units if x))
                          + "; written as m/s signed (flood +), directions degT"),
                "units": "m s-1 (east, north)" if is_field else "m s-1, degT",
                "valid": ((cj.get("field") or {}).get("valid_utc") if is_field else
                          (f"MAX_SLACK events {datetime.fromtimestamp(min(ev), timezone.utc):%Y-%m-%dT%H:%MZ}"
                           f"..{datetime.fromtimestamp(max(ev), timezone.utc):%Y-%m-%dT%H:%MZ}"
                           if ev else None)),
                "adcp": c.get("adcp")})

    w_ = steps.get("waves")
    if w_:
        for pth, size in w_["files"]:
            outputs.append({
                "path": pth, "bytes": size,
                "kind": "GFS-Wave point forecast + NDBC buoys (SeaState format)",
                "source_url": sorted(u for u in by_url if "nomads" in u or
                                     "realtime2/" in u and (u.endswith(".txt") or
                                                            u.endswith(".data_spec"))),
                "datum": "sea state (no vertical datum); GFS-Wave node "
                         f"{w_['node']} nearest the seaward point {p['sea']}",
                "units": GRIB_UNITS | {"ndbc": "per realtime2 header row: WSPD m/s, WVHT m, "
                                               "DPD/APD sec, MWD degT"},
                "valid": f"{w_['cycle']} f000..f048 every 4 h; buoys observed at "
                         + ", ".join(f"{bb['id']} "
                                     f"{datetime.fromtimestamp(bb['obs_unix'], timezone.utc):%Y-%m-%dT%H:%MZ}"
                                     for bb in w_["buoys"] if bb.get("obs_unix"))})

    raw = []
    tiles_meta = {}
    for it in (steps.get("bathy") or {}).get("plan", []):
        if os.path.exists(it["dest"]):
            tiles_meta[os.path.normcase(os.path.abspath(it["dest"]))] = it
    for wf in written:
        if wf["path"] and re.match(r"^[A-Za-z]:", wf["path"]):
            it = tiles_meta.get(os.path.normcase(os.path.abspath(wf["path"])))
            entry = dict(wf)
            if it:
                entry["declared"] = (steps.get("bathy") or {}).get("declared", {}).get(it["key"])
                entry["km_from_anchor"] = it.get("km")
            entry.update(_raw_declared(wf["path"], steps))
            raw.append(entry)
    ofs = steps.get("ofs")
    cache = [w for w in written if w["path"] and not re.match(r"^[A-Za-z]:", w["path"])]
    later = []
    if b:
        later = [i for i in b.get("plan", []) if i["key"] in set(b.get("later", []))]
    elif rec.get("plan"):
        later = [i for i in rec["plan"]["tiles"] if i["key"] in set(rec["plan"]["tiles_later"])]
    reqs = {}
    for e in mine:
        h = reqs.setdefault(e["host"], {"requests": 0, "bytes": 0})
        h["requests"] += 1
        h["bytes"] += e.get("bytes") or 0
    manifest = {
        "place": place,
        "generated_utc": now_utc().isoformat(timespec="seconds"),
        "about": "Every file the place harvest wrote (outputs under data/, raw downloads on the "
                 "big-data drive, raw responses under cache/), with source URL, size, the datum "
                 "and units the source declares, and the instant it is valid for; what each "
                 "source says about itself; what was skipped and why. Written once by "
                 "harvester/harvest_place.py --final.",
        "box": p["box"], "box_why": p.get("box_why"), "anchor": p["anchor"],
        "sea_point": p["sea"],
        "outputs": outputs,
        "raw_downloads": raw,
        "cache_files": cache,
        "ofs": ({k: v for k, v in ofs.items() if k != "plan"} if ofs else None),
        "sources": sources_block(rec, place),
        "skipped": rec.get("skipped", []),
        "surprises": SURPRISES,
        "requests_this_place": reqs,
        "requests_shared_discovery": {"requests": len(shared),
                                      "bytes": sum(e.get("bytes") or 0 for e in shared)},
        "ledger": rel(polite.LEDGER),
    }
    if later or rec.get("plan"):
        pl = rec.get("plan") or {}
        manifest["plan"] = {
            "made_utc": pl.get("made_utc"), "total_bytes_everything": pl.get("total_bytes"),
            "small_products": pl.get("small"), "ofs": pl.get("ofs"),
            "tiles_fetched": [{"name": i["key"], "bytes": i["size"], "km": i.get("km"),
                               "path": rel(i["dest"])}
                              for i in (b or {}).get("plan", []) if os.path.exists(i["dest"])],
            "tiles_remaining": [{"name": i["key"], "bytes": i["size"], "km": i.get("km"),
                                 "url": i["url"], "dest": i["dest"]} for i in later],
            "tiles_remaining_bytes": sum(i["size"] or 0 for i in later),
            "continue_with": f"python harvester/harvest_place.py --place {place} --steps "
                             f"bathy,manifest --final, with GAGAME_LEDGER naming a fresh ledger "
                             f"(a fresh ledger is a fresh budget): tiles on disk are never "
                             f"requested again, the remaining ones are taken nearest-first from "
                             f"the anchor in the order listed here, and the run ADDS "
                             f"data/places/{place}.<UTC stamp>.json beside this manifest",
        }
    return manifest


SURPRISES = [
    "CUDEM Florida tiles spell the name separator 'X' (ncei19_n26X00_w080X25_2018v1.tif); the "
    "New England ones use 'x'. harvest_bathy's name parser is now case-insensitive.",
    "CUDEM tiles declare only their horizontal CRS (GeoKey GeographicType 4269, NAD83); the "
    "vertical datum (NAVD88) is declared by the dataset metadata XML, not by the tile. The index "
    "gives srs EPSG:4269 for FL/chesapeake_bay tiles but EPSG:5498 (NAD83 + NAVD88 height) for "
    "the northeast_sandy ones.",
    "The CUDEM tile index (tileindex_NCEI_ninth_Topobathy_2014.zip) points every tile at the NODD "
    "bucket noaa-nos-coastal-lidar-pds, not at coast.noaa.gov; the old per-region directory "
    "listing did the same.",
    "GFS-Wave on the NODD bucket is GRIB2 template 5.40 (JPEG 2000) for every gridded product "
    "(checked: atlocn.0p16, global.0p25), which the stdlib grib2.py cannot decode; the text "
    "bulletins there (bulls.tHHz/gfswave.<id>.bull/.cbull) are per output point, carry no wind, "
    "and their two flavours disagree on direction by 180 degrees for the same swell (44099, "
    "18z: .bull 254, .cbull 074; the buoy itself reported MWD 75 'from' at 23z, so the .cbull "
    "is 'from' and the .bull 'toward'). Buoy 41122 (Hollywood, 13 km from the Haulover sea "
    "point) is not a GFS-Wave output point.",
    "NOS OFS output on the NODD bucket is netCDF-4 (HDF5: superblock 0, the root group's links "
    "and attributes in DENSE storage -- fractal heap + v2 B-tree -- variables chunked but "
    "unfiltered); only OFS_Grid_Datum/*_vdatums.nc is classic netCDF-3. CBOFS zeta declares "
    "units 'meter' and long_name 'free-surface' but no datum; the datum grid gives NAVD88, MHHW, "
    "MHW, MLW, MLLW each 'minus mean sea level' in metres, is dated 2010 and names its model "
    "'DRBM_3D' under the title 'Chesapeake Bay and River Model 2' (a model name that is not "
    "CBOFS's). The bucket also holds a datum grid for 'secofs' (2026-07-08) but no secofs "
    "output folder.",
    "CO-OPS current stations list one entry per depth BIN (types H, S and W per bin; W = weak "
    "and variable, no predictions), and multi-bin stations need &bin= on the request; the "
    "Haulover-area subordinate TIDE stations all reference 8723178 (Government Cut), not the "
    "harmonic stations beside them.",
    "NDBC 41122 (Hollywood Beach, a CDIP Waverider) publishes .adcp; the NDBC table lists "
    "44010, 44071 and 44096 at the Chesapeake mouth but none of them publishes to realtime2 now.",
    "CUSP's Southeast_Caribbean.zip holds Florida's Atlantic shoreline as ONE 246.8 MB deflate "
    "member; FIF_20260908.zip is per-state inland water (22 states, no Florida).",
    "CO-OPS answered a whole year of hourly predictions in ONE request (8760 values), where "
    "harvest_tides had always asked month by month; a place asks once per station.",
    "www.noaa.gov/disclaimer and www.ndbc.noaa.gov/disclaimer.shtml answered HTTP 403 to this "
    "client (not retried); NDBC's realtime2 data files answered normally.",
]


def sources_block(rec, place):
    terms = rec.get("terms") or {}
    steps = rec.get("steps", {})

    def q(key):
        t = terms.get(key, {})
        quote = t.get("quote")
        if quote and " so long as " in quote:       # one line: the grant, not its conditions
            quote = quote.split(" so long as ")[0] + " [...]"
        return {"quote": quote, "url": t.get("url"), "error": t.get("error")}

    meta = open(CUDEM_META, encoding="utf-8", errors="replace").read() \
        if os.path.exists(CUDEM_META) else ""
    cite = re.search(r"Cite as:\s*([^<]+)", meta)
    out = {
        "public_domain_where_stated": [
            {"url": "https://tidesandcurrents.noaa.gov/disclaimers.html",
             "quote": q("coops")["quote"]},
            {"url": "https://www.weather.gov/disclaimer", "quote": q("nws")["quote"]},
            {"url": CUDEM_META_URL,
             "quote": "Produced by the NOAA National Centers for Environmental Information. Not "
                      "subject to copyright protection within the United States."},
            {"url": "https://www.noaa.gov/disclaimer", "note": q("noaa")["error"]}],
        "cudem": {
            "what": "NOAA NCEI CUDEM 1/9 arc-second topobathy tiles (dataset 8483)",
            "hosts": ["noaa-nos-coastal-lidar-pds.s3.amazonaws.com"],
            "terms": [{"url": CUDEM_META_URL,
                       "quote": "Not subject to copyright protection within the United States."},
                      q("nodd_lidar")],
            "may_keep": "yes: public domain (no copyright in the US) and NODD 'open to the "
                        "public and can be used as desired'; 'Not to be used for navigation.'",
            "cite": (cite.group(1).strip() if cite else None),
            "cite_nodd": "NOAA Coastal Lidar Data was accessed on DATE from "
                         "https://registry.opendata.aws/noaa-coastal-lidar."},
        "coops": {
            "what": "NOAA CO-OPS station metadata, harmonic constituents, datums, predictions",
            "hosts": ["api.tidesandcurrents.noaa.gov"],
            "terms": [q("coops")],
            "may_keep": "yes: public domain per the page quoted",
            "cite": "NOS requests that attribution be given whenever NOS material is reproduced "
                    "and re-disseminated (same page): 'NOAA Tides and Currents (CO-OPS), station "
                    "<id>'."},
        "ndbc": {
            "what": "NOAA NDBC realtime2 observations (met/wave summary, spectra, ADCP) and the "
                    "station table",
            "hosts": ["www.ndbc.noaa.gov"],
            "terms": [q("ndbc"), q("nws")],
            "may_keep": "yes: NDBC is part of NOAA's National Weather Service, whose disclaimer "
                        "(quoted) puts its information in the public domain; NDBC's own "
                        "disclaimer page answered 403",
            "cite": "NOAA National Data Buoy Center, station <id> (the NWS page asks that the "
                    "information not be claimed as your own, nor used to imply endorsement)"},
        "gfswave": {
            "what": "NCEP GFS-Wave (WAVEWATCH III) gridded forecast, subset by the NOMADS grib "
                    "filter; the S3 mirror (noaa-gfs-bdp-pds) was used only to list the cycle, "
                    "read two .idx files, two 512-byte message heads and one bulletin pair",
            "hosts": ["nomads.ncep.noaa.gov", "noaa-gfs-bdp-pds.s3.amazonaws.com"],
            "terms": [q("nws"), q("nodd_gfs")],
            "may_keep": "yes: NWS public domain; NODD 'open to the public and can be used as "
                        "desired'",
            "cite": "NOAA Global Forecast System (GFS) was accessed on DATE from "
                    "https://registry.opendata.aws/noaa-gfs-bdp-pds (the registry's form)"},
    }
    if steps.get("ofs"):
        out["cbofs"] = {
            "what": "NOAA NOS Chesapeake Bay Operational Forecast System (ROMS) nowcast fields, "
                    "station nowcast and the vertical-datum grid",
            "hosts": ["noaa-nos-ofs-pds.s3.amazonaws.com"],
            "terms": [q("nodd_ofs")],
            "may_keep": "yes: NODD 'open to the public and can be used as desired'",
            "cite": "NOAA Operational Forecast System (OFS) was accessed on DATE from "
                    "https://registry.opendata.aws/noaa-ofs (the registry's form)"}
    if place == "haulover":
        out["cusp"] = {"what": "NGS Continually Updated Shoreline Product (not used: skipped)",
                       "hosts": ["geodesy.noaa.gov"],
                       "terms": "NGS is part of NOAA: public domain per the pages quoted above; "
                                "the directory listing carries no terms of its own",
                       "may_keep": "yes", "cite": None}
    return out


def step_manifest(place, p, rec, final=False):
    m = build_manifest(place, p, rec)
    draft = os.path.join("out", "places", f"{place}.manifest.draft.json")
    with open(draft, "w", encoding="utf-8") as f:
        json.dump(m, f, indent=1, default=str)
    log(f"[manifest] draft {draft}: {len(m['outputs'])} outputs, {len(m['raw_downloads'])} raw "
        f"downloads, {len(m['cache_files'])} cache files")
    if final:
        # The first manifest is data/places/<place>.json; a later run that continues the place
        # (more tiles on a fresh budget) ADDS data/places/<place>.<UTC stamp>.json beside it.
        path = os.path.join("data", "places", f"{place}.json")
        if os.path.exists(path):
            path = os.path.join("data", "places", f"{place}.{now_utc():%Y%m%dT%H%MZ}.json")
        os.makedirs(os.path.dirname(path), exist_ok=True)
        with open(path, "x", encoding="utf-8") as f:
            json.dump(m, f, indent=1, default=str)
        log(f"[manifest] wrote {path}")


# ------------------------------------------------------------------------------------ main

def main():
    args = sys.argv[1:]

    def opt(flag, default=None):
        return args[args.index(flag) + 1] if flag in args else default

    place = opt("--place")
    if not place:
        raise SystemExit(__doc__)
    p = dict(PLACES.get(place, {}))
    if "--box" in args:
        lon0, lat0, lon1, lat1 = (float(v) for v in opt("--box").split(","))
        p["box"] = {"lon0": lon0, "lat0": lat0, "lon1": lon1, "lat1": lat1}
        p.setdefault("box_why", "given on the command line")
    for k in ("anchor", "sea"):
        if f"--{k}" in args:
            p[k] = tuple(float(v) for v in opt(f"--{k}").split(","))
    if "box" not in p:
        raise SystemExit(f"unknown place '{place}' (have {list(PLACES)}); give --box")
    b = p["box"]
    p.setdefault("anchor", ((b["lon0"] + b["lon1"]) / 2, (b["lat0"] + b["lat1"]) / 2))
    p.setdefault("sea", p["anchor"])
    for k, v in (("tide_radius_km", 20.0), ("current_radius_km", 20.0),
                 ("max_current_stations", 8), ("adcp_radius_km", 60.0), ("ofs", None)):
        p.setdefault(k, v)
    default = ("plan,tides,waves,ofs,ofsfield,ofscheck,bathy,survey,terms,manifest" if p["ofs"]
               else "tides,currents,waves,bathy,survey,terms,manifest")
    steps = opt("--steps", default).split(",")
    polite.CONTEXT["place"] = place
    rec = load_record(place, p)
    rec["box_why"] = p.get("box_why")
    log(f"=== place {place}: box {p['box']}, anchor {p['anchor']}, sea {p['sea']}; "
        f"steps {steps}; budget left {polite.remaining()[0] / 1e9:.3f} GB / "
        f"{polite.remaining()[1]} requests")
    try:
        for s in steps:
            if s == "plan":
                step_plan(place, p, rec)
            elif s == "tides":
                step_tides(place, p, rec)
            elif s == "currents":
                step_currents(place, p, rec)
            elif s == "waves":
                step_waves(place, p, rec)
            elif s == "ofs":
                step_ofs(place, p, rec)
            elif s == "ofsfield":
                step_ofsfield(place, p, rec)
            elif s == "ofscheck":
                step_ofscheck(place, p, rec)
            elif s == "bathy":
                step_bathy(place, p, rec)
            elif s == "survey":
                step_survey(place, p, rec)
            elif s == "terms":
                step_terms(rec)
            elif s == "manifest":
                step_manifest(place, p, rec, final="--final" in args)
            else:
                raise SystemExit(f"unknown step '{s}'")
            save_record(rec)
    except polite.Stop as e:
        rec["stopped"] = str(e)
        log(f"STOPPED: {e}")
    finally:
        rec["ledger"] = polite.summary()
        save_record(rec)
    log(f"=== {place}: ledger {polite.summary()['requests']} requests, "
        f"{polite.summary()['bytes'] / 1e9:.3f} GB; record {record_path(place)}")


if __name__ == "__main__":
    main()
