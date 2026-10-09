# ==================================================================================================
#  harvest_overture.py - Overture Maps buildings that carry a HEIGHT, as building solids, one
#  source per place the height came from.
#
#  WHY. OSM tags a height on almost nothing here (0 of 5,857 at Newburyport). Overture's buildings
#  theme conflates OSM with Microsoft's ML footprints and attaches heights with their provenance:
#  in the Newburyport box (release 2026-09-23.1) 5,154 of 6,010 carry one -- 2,529 from USGS LIDAR
#  (measured) and 2,512 from MICROSOFT ML BUILDINGS (estimated). Those are two different claims, so
#  they are two SOURCES, and the scene's stack says which wins: no code decides it.
#
#  STAGES (GA Load -> Normalize):
#    LOAD       the release's STAC catalog (stac.overturemaps.org) names each GeoParquet file's box;
#               only the files meeting --bbox are opened, by GDAL over HTTP (QGIS's ogr2ogr, found
#               or named by --ogr2ogr), and GDAL reads only the row groups meeting the box (2 of 256
#               for the Newburyport box). Only features with `height` or `num_floors` are taken.
#    NORMALIZE  Overture's height and min_height are metres above the ground (its schema), the
#               harvest's datum. Coordinates are rounded to 1e-7 degree (1.1 cm), the format's unit.
#               The id is the OSM way the feature came from (sources[].record_id "w123@4"; a
#               relation's negated), 0 for an ML-only footprint: so a solid here replaces its OSM
#               twin by id, and an ML footprint OSM lacks is simply added.
#    SPLIT      one GABLDG01 file per dataset the HEIGHT is attributed to (sources[].property ==
#               "/properties/height"): usgs_lidar, microsoft_ml_buildings, ... and "unattributed".
#
#  GDAL 3.12.2 TRAP: selecting the boolean `has_parts` column crashes ogr2ogr (access violation,
#  measured 2026-10-09); it is not selected.
#
#  LICENCE: Overture buildings are ODbL 1.0 (OSM-derived); credit "(c) OpenStreetMap contributors,
#  Overture Maps Foundation". Each source's own terms ride its manifest.
#
#  Usage:  py -3 harvester/harvest_overture.py --bbox -73.51 41.23 -69.92 42.89 --stem massachusetts
#          [--release latest] [--out D:\DataCache\Overture\buildings]
# ==================================================================================================
import argparse
import json
import math
import os
import re
import subprocess
import sys
import time
import urllib.error
import urllib.request

import numpy as np

from harvest_buildings import cell_of, write_gabldg

STAC = "https://stac.overturemaps.org"
UA = {"User-Agent": "GAGAME/0.1 (hobby ocean sim; Overture buildings by bbox)"}
COLUMNS = "id,height,min_height,num_floors,min_floor,sources,roof_shape,roof_height"
QGIS_ENV = r"C:\Program Files\QGIS 4.0.0\bin\o4w_env.bat"
NAN = float("nan")


def get_json(url, tries=5):
    """One JSON document; a dropped connection is asked again after a growing pause (the STAC
    host closed one of 512 sequential item reads on 2026-10-09)."""
    for k in range(tries):
        try:
            with urllib.request.urlopen(urllib.request.Request(url, headers=UA), timeout=120) as r:
                return json.load(r)
        except (urllib.error.URLError, ConnectionError, TimeoutError):
            if k == tries - 1:
                raise
            time.sleep(2.0 * 2 ** k)


def items_for(release, bbox, cache_dir):
    """The building files whose STAC box meets bbox, from a per-release cache of every item's box."""
    path = os.path.join(cache_dir, f"stac-{release}-building.json")
    if os.path.exists(path):
        items = json.load(open(path))
    else:
        coll = get_json(f"{STAC}/{release}/buildings/building/collection.json")
        items = []
        for link in coll["links"]:
            if link["rel"] != "item":
                continue
            it = get_json(link["href"])
            href = [a["href"] for a in it["assets"].values() if a["href"].endswith(".parquet")][0]
            items.append({"id": it["id"], "bbox": it["bbox"], "href": href})
        os.makedirs(cache_dir, exist_ok=True)
        json.dump(items, open(path, "w"))
    w, s, e, n = bbox
    return [it for it in items if it["bbox"][0] <= e and it["bbox"][2] >= w and it["bbox"][1] <= n and it["bbox"][3] >= s]


def fetch(href, bbox, out_path, env_bat):
    """ogr2ogr the file's features in bbox carrying a height or floors into GeoJSONSeq. Run from a
    batch file beside the output: QGIS's GDAL needs its own environment (o4w_env), and cmd's
    quoting does not survive a second layer of quotes from here."""
    w, s, e, n = bbox
    bat = out_path + ".bat"
    with open(bat, "w") as f:
        f.write("@echo off\n")
        f.write(f'call "{env_bat}"\n')
        f.write(f'set "GDAL_HTTP_USERAGENT={UA["User-Agent"]}"\n')
        f.write(f'ogr2ogr -f GeoJSONSeq "{out_path}" "/vsicurl/{href}" -spat {w} {s} {e} {n} '
                f'-select "{COLUMNS}" -where "height IS NOT NULL OR num_floors IS NOT NULL"\n')
        f.write("exit /b %ERRORLEVEL%\n")
    r = subprocess.run(["cmd.exe", "/c", bat], capture_output=True, text=True)
    os.remove(bat)
    if r.returncode != 0:
        raise RuntimeError(f"ogr2ogr exit {r.returncode}: {r.stderr.strip()[-400:]}")


def osm_id(sources):
    for src in sources or []:
        if src.get("dataset") == "OpenStreetMap" and src.get("record_id"):
            m = re.match(r"([wr])(\d+)", src["record_id"])
            if m:
                return int(m.group(2)) * (1 if m.group(1) == "w" else -1)
    return 0


def height_dataset(sources):
    for src in sources or []:
        if src.get("property") == "/properties/height":
            return re.sub(r"[^a-z0-9]+", "_", (src.get("dataset") or "unattributed").lower()).strip("_")
    return "unattributed"


def num(v):
    return NAN if v is None else float(v)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--bbox", type=float, nargs=4, required=True, metavar=("LON0", "LAT0", "LON1", "LAT1"))
    ap.add_argument("--stem", required=True)
    ap.add_argument("--release", default="latest")
    ap.add_argument("--out", default=r"D:\DataCache\Overture\buildings")
    ap.add_argument("--qgis-env", default=QGIS_ENV)
    a = ap.parse_args()
    t0 = time.time()
    release = get_json(f"{STAC}/catalog.json")["latest"] if a.release == "latest" else a.release
    raw_dir = os.path.join(os.path.dirname(a.out.rstrip("\\/")), "raw", release)
    os.makedirs(raw_dir, exist_ok=True)
    items = items_for(release, a.bbox, os.path.dirname(raw_dir))
    print(f"[overture] release {release}: {len(items)} building files meet the box")

    roof_shapes, by_ds = [], {}
    refused = {"geometry": 0}
    for it in items:
        part = os.path.join(raw_dir, f"{a.stem}-{it['id']}.geojsons")
        if not os.path.exists(part):   # the forever-cache: a box is read from Overture once
            fetch(it["href"], a.bbox, part + ".tmp", a.qgis_env)
            os.replace(part + ".tmp", part)
        print(f"[overture] {it['id']}: {os.path.getsize(part) / 1048576:.1f} MB of features ({time.time() - t0:.0f} s)")
        for line in open(part, encoding="utf-8"):
            line = line.strip("\x1e \r\n")
            if not line:
                continue
            f = json.loads(line)
            p, g = f["properties"], f.get("geometry") or {}
            src = p.get("sources")
            src = json.loads(src) if isinstance(src, str) else src
            polys = [g["coordinates"]] if g.get("type") == "Polygon" else g.get("coordinates", []) if g.get("type") == "MultiPolygon" else []
            geo = []
            for poly in polys:
                for k, ring in enumerate(poly):
                    xy = np.rint(np.asarray(ring, np.float64)[:, :2] * 1e7).astype(np.int32)
                    if len(xy) > 1 and (xy[0] == xy[-1]).all():
                        xy = xy[:-1]   # implicitly closed
                    if len(xy) >= 3:
                        geo.append((1 if k == 0 else 0, xy))
            if not geo:
                refused["geometry"] += 1
                continue
            roof = 255
            if p.get("roof_shape"):
                if p["roof_shape"] not in roof_shapes:
                    roof_shapes.append(p["roof_shape"])
                roof = roof_shapes.index(p["roof_shape"])
            attrs = (roof, num(p.get("height")), num(p.get("min_height")), num(p.get("num_floors")),
                     num(p.get("min_floor")), num(p.get("roof_height")))
            x0, y0 = geo[0][1][0]
            ds = height_dataset(src)
            by_ds.setdefault(ds, {})[p["id"]] = (cell_of(int(x0), int(y0)), osm_id(src), 0, attrs, geo)

    for ds, recs in sorted(by_ds.items()):
        stem = f"{a.stem}-{ds}"
        bin_path, tagged = write_gabldg(
            a.out, stem, list(recs.values()),
            {"release": release, "theme": "buildings/building", "files": [it["href"] for it in items],
             "heightDataset": ds},
            "ODbL 1.0", "(c) OpenStreetMap contributors, Overture Maps Foundation", a.bbox, refused, {},
            roof_shapes, coordinates="int32, 1e-7 degree, WGS84 lon/lat (Overture's doubles, rounded)")
        print(f"[overture] {stem}: {len(recs):,} solids -> {bin_path} ({os.path.getsize(bin_path) / 1048576:.0f} MB); tagged {tagged}")
    print(f"[overture] done in {time.time() - t0:.0f} s; refused {refused}")


if __name__ == "__main__":
    sys.exit(main())
