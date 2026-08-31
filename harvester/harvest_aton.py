# ==================================================================================================
#  harvest_aton.py - surveyed aids to navigation from NOAA ENC Direct to GIS.
#
#  vqview-inlet's marker layer placed indicative buoys at fixed offsets from the AIS lane and its
#  README asked for exactly this upgrade: "those come from the USCG Light List / ATON dataset."
#  ENC Direct serves the charted S-57 aids from the live NOAA ENC cells as plain ArcGIS REST JSON
#  (no key, public domain, weekly cadence synced to Notice to Mariners).  Endpoints verified
#  2026-08-31: the Merrimack box returns 43 aids from harbour cells US5MA1VG/US5MA1VH, including
#  "Merrimack River Buoy 24" (red nun, CATLAM 2) and the jetty light Fl G 2.5s.
#
#  Raw per-layer responses cached forever in D:\DataCache\GAGAME\aton\ (a weekly-updating product,
#  but a charted snapshot stays valid for the sim; delete the cache to refresh).  Engine-ready
#  output: data/gis/aton_ne.json - flat feature list with decoded S-57 attributes, WGS84 lon/lat.
#
#  Positions are charted/assigned: fixed aids (lights, daymarks, beacons) are effectively exact;
#  floating aids swing on their moorings and are seasonally serviced (SORDAT/SORIND carry the
#  source vintage).  "Not for navigation" applies - which is fine, we are a renderer.
# ==================================================================================================
import json
import os
import sys
import time
import urllib.parse
import urllib.request

UA = "GAGAME/0.1 (hobby ocean simulator; contact: marksmcgrotty@gmail.com)"
BASE = "https://encdirect.noaa.gov/arcgis/rest/services/encdirect/enc_harbour/MapServer"
DCACHE = r"D:\DataCache\GAGAME\aton"
OUT = os.path.join("data", "gis", "aton_ne.json")

# The NE focus box (same as harvest_water's water survey box).
BOX = {"lon0": -71.15, "lat0": 42.20, "lon1": -70.35, "lat1": 43.05}

# enc_harbour ATON point layers (id -> kind). Verified by live queries; field sets differ per
# layer so we always request outFields=* (a named missing field 400s the whole query).
LAYERS = {
    1:  "beacon_lateral",
    2:  "beacon_safe_water",
    3:  "beacon_special",
    4:  "buoy_cardinal",
    5:  "buoy_isolated_danger",
    6:  "buoy_lateral",
    7:  "buoy_safe_water",
    8:  "buoy_special",
    9:  "daymark",
    11: "light",
    12: "light_float",
}

# S-57 COLOUR codes.
S57_COLOUR = {1: "white", 2: "black", 3: "red", 4: "green", 5: "blue", 6: "yellow",
              7: "grey", 8: "brown", 9: "amber", 10: "violet", 11: "orange",
              12: "magenta", 13: "pink"}
# S-57 LITCHR codes (the ones that occur around here).
S57_LITCHR = {1: "F", 2: "Fl", 3: "LFl", 4: "Q", 5: "VQ", 6: "UQ", 7: "Iso", 8: "Oc",
              9: "IQ", 10: "IVQ", 11: "IUQ", 12: "Mo", 13: "FFl", 14: "FlLFl",
              15: "OcFl", 16: "FLFl", 17: "OcAlt", 18: "LFlAlt", 19: "FlAlt",
              25: "Q+LFl", 26: "VQ+LFl", 27: "Al", 28: "Al.Oc", 29: "Al.Fl"}


def fetch(url, cache_path, pause=0.6):
    if os.path.exists(cache_path) and os.path.getsize(cache_path) > 2:
        return open(cache_path, "rb").read(), True
    last = None
    for attempt in range(3):
        try:
            req = urllib.request.Request(url, headers={"User-Agent": UA})
            with urllib.request.urlopen(req, timeout=60) as r:
                data = r.read()
            os.makedirs(os.path.dirname(cache_path), exist_ok=True)
            open(cache_path, "wb").write(data)
            time.sleep(pause)
            return data, False
        except Exception as e:                                     # noqa: BLE001
            last = e
            time.sleep(2.0 * (2 ** attempt))
    raise RuntimeError("fetch failed %s: %s" % (url, last))


def query_layer(layer_id):
    """All features of one layer in BOX, paginated (maxRecordCount is 1000)."""
    feats = []
    offset = 0
    while True:
        params = {
            "where": "1=1",
            "geometry": "%f,%f,%f,%f" % (BOX["lon0"], BOX["lat0"], BOX["lon1"], BOX["lat1"]),
            "geometryType": "esriGeometryEnvelope",
            "inSR": "4326",
            "spatialRel": "esriSpatialRelIntersects",
            "outFields": "*",
            "returnGeometry": "true",
            "resultOffset": str(offset),
            "f": "json",
        }
        url = "%s/%d/query?%s" % (BASE, layer_id, urllib.parse.urlencode(params))
        cache = os.path.join(DCACHE, "enc_harbour_L%d_o%d.json" % (layer_id, offset))
        data, cached = fetch(url, cache)
        j = json.loads(data)
        if "error" in j:
            raise RuntimeError("layer %d error: %s" % (layer_id, j["error"]))
        page = j.get("features", [])
        feats.extend(page)
        if not j.get("exceededTransferLimit") or not page:
            return feats, cached
        offset += len(page)


def light_characteristic(at):
    """Compose 'Fl G 2.5s'-style from S-57 attrs where present."""
    ch = S57_LITCHR.get(at.get("LITCHR") or 0, "")
    if not ch:
        return ""
    grp = (at.get("SIGGRP") or "").strip()
    if grp and grp not in ("(1)",):
        ch += grp
    col = S57_COLOUR.get(int(at.get("COLOUR") or 0) if str(at.get("COLOUR", "")).strip().isdigit()
                         else 0, "")
    if col:
        ch += " " + {"white": "W", "red": "R", "green": "G", "yellow": "Y"}.get(col, col)
    per = at.get("SIGPER")
    if per:
        ch += " %gs" % per
    return ch


def main():
    all_feats = []
    for lid, kind in sorted(LAYERS.items()):
        try:
            feats, cached = query_layer(lid)
        except RuntimeError as e:
            print("  layer %d (%s): FAILED %s" % (lid, kind, e))
            continue
        print("  layer %2d %-22s %3d features%s" % (lid, kind, len(feats),
                                                    "  (cache)" if cached else ""))
        for f in feats:
            at = f.get("attributes", {})
            g = f.get("geometry", {})
            if "x" not in g:
                continue
            colour_raw = str(at.get("COLOUR", "")).strip()
            # COLOUR can be "3" or "3,1" (striped); keep the list, decode the first.
            codes = [int(c) for c in colour_raw.split(",") if c.strip().isdigit()]
            entry = {
                "lon": round(g["x"], 7), "lat": round(g["y"], 7),
                "kind": kind,
                "name": (at.get("OBJNAM") or "").strip(),
                "colour": [S57_COLOUR.get(c, str(c)) for c in codes],
                "shape": (str(at.get("BOYSHP") or at.get("BCNSHP") or "")).strip(),
                "catlam": at.get("CATLAM"),
                "light": light_characteristic(at),
                "height_m": at.get("HEIGHT"),
                "range_nm": at.get("VALNMR"),
                "cell": (at.get("DSNM") or "").strip(),
                "source": (at.get("SORIND") or "").strip(),
                "source_date": (at.get("SORDAT") or "").strip(),
            }
            all_feats.append(entry)

    # de-dup: the same aid can appear in overlapping ENC cells; keep first per (pos, kind)
    seen = set()
    uniq = []
    for e in all_feats:
        key = (round(e["lon"], 6), round(e["lat"], 6), e["kind"])
        if key in seen:
            continue
        seen.add(key)
        uniq.append(e)

    out = {
        "_readme": ["Charted aids to navigation from NOAA ENC Direct (enc_harbour band),",
                    "S-57 attributes decoded; positions WGS84 charted/assigned - fixed aids",
                    "exact, floating aids swing on moorings. Public domain; not for navigation.",
                    "Source service: " + BASE],
        "box": BOX,
        "generated_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
        "count": len(uniq),
        "aids": sorted(uniq, key=lambda e: (e["kind"], e["name"], e["lat"])),
    }
    os.makedirs(os.path.dirname(OUT), exist_ok=True)
    json.dump(out, open(OUT, "w"), indent=1)
    print("wrote %s: %d aids (%d raw, %d duplicate cell overlaps)" % (
        OUT, len(uniq), len(all_feats), len(all_feats) - len(uniq)))

    inlet = [e for e in uniq
             if -70.88 <= e["lon"] <= -70.75 and 42.76 <= e["lat"] <= 42.86]
    print("Merrimack box: %d aids" % len(inlet))
    for e in sorted(inlet, key=lambda e: e["kind"]):
        print("   %-20s %-28s %s %s" % (e["kind"], e["name"] or "-",
                                        "/".join(e["colour"]) or "-", e["light"] or ""))
    return 0


if __name__ == "__main__":
    sys.exit(main())
