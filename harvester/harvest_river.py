#!/usr/bin/env python3
"""GAGAME Harvester, M5: Merrimack River discharge and stage from USGS.

Uses the legacy instantaneous-values service (no key, decommissioned Q1 2027 -- the migration to
api.waterdata.usgs.gov, which needs the free key, is budgeted for when the user grabs one).
Two mainstem gauges: 01100000 Lowell, 01100500 Lawrence. Gentle: one request, gzip accepted,
cached for 15 minutes (the sensor cadence).

Output: data/river/river.json. M5 shows it; M5b's shallow-water solver will use Lawrence
discharge (lagged ~half a day for the ~35 km to Newburyport) as the upstream boundary.
"""

import gzip
import io
import json
import os
import sys
import time
import urllib.request
from datetime import datetime, timezone

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from harvest_currents import log  # noqa: E402

UA = "GAGAME/0.1 (hobby ocean simulator; contact: marksmcgrotty@gmail.com)"
SITES = "01100000,01100500"
URL = (f"https://waterservices.usgs.gov/nwis/iv/?format=json&sites={SITES}"
       "&parameterCd=00060,00065&siteStatus=all")
CFS_TO_CMS = 0.0283168


def main():
    out_dir = os.path.join("data", "river")
    cache = os.path.join("cache", "river", "iv.json")
    os.makedirs(out_dir, exist_ok=True)
    os.makedirs(os.path.dirname(cache), exist_ok=True)

    if os.path.exists(cache) and time.time() - os.path.getmtime(cache) < 900:
        with open(cache, "r", encoding="utf-8") as f:
            data = json.load(f)
        log("using cached USGS response (< 15 min old)")
    else:
        try:
            req = urllib.request.Request(URL, headers={"User-Agent": UA,
                                                       "Accept-Encoding": "gzip"})
            with urllib.request.urlopen(req, timeout=60) as r:
                raw = r.read()
                if r.headers.get("Content-Encoding") == "gzip":
                    raw = gzip.GzipFile(fileobj=io.BytesIO(raw)).read()
            data = json.loads(raw.decode("utf-8"))
            with open(cache, "w", encoding="utf-8") as f:
                json.dump(data, f)
            log("fetched USGS instantaneous values")
        except Exception as e:  # noqa: BLE001
            # The legacy service is in its decommission brownout period (late 2026, exactly as
            # the service notice warned). The stale cache, if any, is better than nothing; the
            # real fix is the keyed api.waterdata.usgs.gov migration (M5b).
            log(f"USGS legacy service unavailable ({e}) -- likely a decommission brownout; "
                "get the free api.waterdata.usgs.gov key for the migration")
            if os.path.exists(cache):
                with open(cache, "r", encoding="utf-8") as f:
                    data = json.load(f)
                log("falling back to the stale cache")
            else:
                data = {}

    sites = {}
    for ts in data.get("value", {}).get("timeSeries", []):
        info = ts.get("sourceInfo", {})
        code = info.get("siteCode", [{}])[0].get("value", "?")
        name = info.get("siteName", code)
        param = ts.get("variable", {}).get("variableCode", [{}])[0].get("value", "")
        vals = ts.get("values", [{}])[0].get("value", [])
        if not vals:
            continue
        last = vals[-1]
        v = float(last.get("value", "nan"))
        t = last.get("dateTime", "")
        entry = sites.setdefault(code, {"id": code, "name": name})
        if param == "00060":
            entry["discharge_cfs"] = v
            entry["discharge_cms"] = round(v * CFS_TO_CMS, 2)
        elif param == "00065":
            entry["gage_ft"] = v
        entry["obs_iso"] = t

    out = {
        "generated_utc": datetime.now(timezone.utc).isoformat(timespec="seconds"),
        "source": "USGS waterservices IV (legacy; migrate to api.waterdata.usgs.gov by 2027Q1)",
        "sites": list(sites.values()),
    }
    path = os.path.join(out_dir, "river.json")
    with open(path, "w", encoding="utf-8") as f:
        json.dump(out, f, indent=1)
    for s in out["sites"]:
        log(f"  {s['id']} {s['name']}: {s.get('discharge_cms', '?')} m3/s, "
            f"stage {s.get('gage_ft', '?')} ft @ {s.get('obs_iso', '?')}")
    log(f"wrote {path}")


if __name__ == "__main__":
    main()
