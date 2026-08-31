# ==================================================================================================
#  harvest_route.py - georeference the vqview-inlet AIS traffic lane into engine coordinates.
#
#  vqview's ais_route.json is a traffic-weighted 18-waypoint lane distilled from 9 days / 7087 AIS
#  reports in the Merrimack box, with per-vessel-class mean speed over ground.  Its coordinates are
#  CROP-LOCAL PATCH METRES (x east, z north from the crop's SW corner, 1.5 m cells) - the wide
#  variant (ais_route_wide.json) lives in the wide crop frame, CROP = (120, 1840, 70, 760) of
#  bathy_comp2.npy (georef: lon -70.8320..-70.7980, lat 42.8250(N)..42.8100(S), 1851 x 1106).
#
#  This converts to WGS84 lat/lon AND the engine world frame (x = (lon+70.81)*81660,
#  z = (lat-42.81833)*110574 - BathyModel constants), preserving arc-length parameterization
#  and the AIS per-class speeds.  Output: data/gis/route_merrimack.json.
#
#  Live AIS (aisstream.io) is the named upgrade; this static climatology is the traffic PRIOR -
#  where boats actually run, at the speeds they actually run.
# ==================================================================================================
import json
import math
import os
import sys

VQ = os.environ.get("VQVIEW", r"C:\vqview-inlet\vqview-inlet")
ROUTE = os.path.join(VQ, "renders", "geotiff", "water", "ais_route_wide.json")
OUT = os.path.join("data", "gis", "route_merrimack.json")

FULL_LON0, FULL_LAT1 = -70.8320, 42.8250
FULL_LON1, FULL_LAT0 = -70.7980, 42.8100
FULL_NX, FULL_NY = 1851, 1106
CROP = (120, 1840, 70, 760)          # the wide crop (x0, x1, y0, y1)
CELL = 1.5

K_ORG_LON, K_ORG_LAT = -70.81, 42.81833      # BathyModel.h anchors
K_M_PER_LON, K_M_PER_LAT = 81660.0, 110574.0


def main():
    r = json.load(open(ROUTE))
    dlon = (FULL_LON1 - FULL_LON0) / FULL_NX
    dlat = (FULL_LAT1 - FULL_LAT0) / FULL_NY
    x0, x1, y0, y1 = CROP
    crop_w_lon = FULL_LON0 + x0 * dlon
    crop_s_lat = FULL_LAT1 - y1 * dlat

    pts = []
    for px, pz in r["route"]:
        lon = crop_w_lon + (px / CELL) * dlon
        lat = crop_s_lat + (pz / CELL) * dlat
        wx = (lon - K_ORG_LON) * K_M_PER_LON
        wz = (lat - K_ORG_LAT) * K_M_PER_LAT
        pts.append({"lon": round(lon, 7), "lat": round(lat, 7),
                    "x": round(wx, 1), "z": round(wz, 1)})

    # arc length in ENGINE metres (the lane must be distance-parameterized: AIS waypoints
    # are even in longitude, not distance - vqview Route.h's own lesson)
    length = 0.0
    for a, b in zip(pts, pts[1:]):
        length += math.hypot(b["x"] - a["x"], b["z"] - a["z"])

    out = {
        "_readme": ["Traffic-weighted AIS lane for the Merrimack inlet, from vqview-inlet's",
                    "9-day / 7087-report climatology; georeferenced from the wide-crop patch",
                    "frame via bathy_comp2 bounds. speeds_ms are per-AIS-class mean SOG.",
                    "Arc-length parameterize when following (waypoints are even in lon)."],
        "source": "vqview-inlet ais_route_wide.json (9 days AIS, 7087 reports)",
        "length_m": round(length, 1),
        "speeds_ms": r.get("speeds_ms", {}),
        "waypoints": pts,
    }
    os.makedirs(os.path.dirname(OUT), exist_ok=True)
    json.dump(out, open(OUT, "w"), indent=1)
    print("wrote %s: %d waypoints, %.0f m lane" % (OUT, len(pts), length))
    print("  seaward end: %.5f, %.5f (world %.0f, %.0f)" % (
        pts[-1]["lat"], pts[-1]["lon"], pts[-1]["x"], pts[-1]["z"]))
    print("  upriver end: %.5f, %.5f (world %.0f, %.0f)" % (
        pts[0]["lat"], pts[0]["lon"], pts[0]["x"], pts[0]["z"]))
    print("  speeds:", ", ".join("%s %.2f" % kv for kv in
                                 sorted(out["speeds_ms"].items())))
    return 0


if __name__ == "__main__":
    sys.exit(main())
