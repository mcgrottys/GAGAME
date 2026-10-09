# ==================================================================================================
#  harvest_buildings.py - OSM buildings from a .osm.pbf, as SOLIDS: every footprint's rings with
#  the heights OSM tagged, normalized to metres, and nothing guessed.
#
#  WHY SOLIDS. A building is a prism (its rings, from min_height to height above the ground), not
#  a bump in the height map: the picture can paint it into the height tree, and the water's sparse
#  voxels can later collide with the same record. One store, many readers.
#
#  STAGES (GA Load -> Normalize; compose and physics are the engine's):
#    LOAD       osmpbf.py reports the file: ways and multipolygon relations tagged building=* or
#               building:part=*, their node coordinates in the file's own 1e-7 degree integers.
#    NORMALIZE  height / min_height / roof:height parsed to METRES ("12", "12 m", "40 ft", "40'",
#               "12'6\""); a value that does not parse is REFUSED (NaN) and counted by tag, never
#               rounded into a guess. building:levels and building:min_level stay counts: the
#               metres a level stands for is the scene's declared assumption, not the harvest's.
#               OSM measures height from the ground at the footprint, so the datum is "ground":
#               the compose stands each solid on the composed height, not on a number made here.
#    UNTOUCHED  an untagged height stays NaN. At Newburyport (2026-10-09) 0 of 5,857 buildings
#               carry `height`: the default belongs to the scene, where it is visible as one.
#
#  OUTPUT, beside the input unless --out:   <stem>.buildings.bin  +  <stem>.buildings.json
#    bin    "GABLDG01", then records sorted by cell (the manifest's index):
#             int64 id (way id; a relation's id negated)
#             uint8 kind (0 building, 1 building:part), uint8 roof (index into roofShapes, 255 none)
#             uint16 rings
#             float32 height, min_height, levels, min_level, roof_height   (NaN = not tagged)
#             per ring: uint32 n, uint8 outer (1) / inner (0), 3 pad, n x (int32 lon, int32 lat) 1e-7 deg
#           rings are implicitly closed (the repeated last node is dropped).
#    json   the source's identity, units, licence and attribution, counts and refusals, and
#           the cell index: [ix, iy, byte offset, count] for cells of `cellDeg` by the first node.
#
#  LICENCE: OSM data is ODbL 1.0; anything drawn from it credits "(c) OpenStreetMap contributors".
#
#  Usage:  py -3 harvester/harvest_buildings.py D:\DataCache\OSM\massachusetts-261008.osm.pbf
#          [--out DIR] [--bbox lon0 lat0 lon1 lat1]
# ==================================================================================================
import argparse
import hashlib
import json
import math
import os
import re
import struct
import sys
import time

import numpy as np

import osmpbf

CELL_DEG = 0.05
NAN = float("nan")
FT = 0.3048
IN = 0.0254
_NUM = r"[-+]?\d+(?:[.,]\d+)?"


def metres(text):
    """A length tag in metres, or None when it does not parse (refused, not guessed)."""
    t = text.strip().lower().replace("~", "")
    m = re.fullmatch(rf"({_NUM})\s*(m|meters?|metres?)?", t)
    if m:
        return float(m.group(1).replace(",", "."))
    m = re.fullmatch(rf"({_NUM})\s*(ft|feet|foot|')", t)
    if m:
        return float(m.group(1).replace(",", ".")) * FT
    m = re.fullmatch(rf"({_NUM})\s*'\s*({_NUM})\s*(\"|in)?", t)
    if m:
        return float(m.group(1)) * FT + float(m.group(2)) * IN
    return None


def count(text):
    t = text.strip().replace(",", ".")
    try:
        v = float(t)
    except ValueError:
        return None
    return v if v >= 0 else None


KEEP = ("building", "building:part", "height", "min_height", "building:levels",
        "building:min_level", "roof:shape", "roof:height", "type")


def tags_of(block, keys, vals):
    s = block.strings
    return {s[k]: s[v] for k, v in zip(keys, vals) if s[k] in KEEP}


def is_building(t):
    b = t.get("building")
    if b is not None and b != "no":
        return 0
    p = t.get("building:part")
    if p is not None and p != "no":
        return 1
    return None


def stitch(ways):
    """Join member ways (node id arrays) into closed rings by shared ends. Unclosable pieces drop."""
    rings, open_ = [], []
    for w in ways:
        (rings if w[0] == w[-1] and len(w) >= 4 else open_).append(list(w))
    while open_:
        cur = open_.pop()
        grown = True
        while cur[0] != cur[-1] and grown:
            grown = False
            for i, w in enumerate(open_):
                if w[0] == cur[-1]:
                    cur += w[1:]
                elif w[-1] == cur[-1]:
                    cur += w[-2::-1]
                elif w[-1] == cur[0]:
                    cur = w[:-1] + cur
                elif w[0] == cur[0]:
                    cur = w[:0:-1] + cur
                else:
                    continue
                open_.pop(i)
                grown = True
                break
        if cur[0] == cur[-1] and len(cur) >= 4:
            rings.append(cur)
    return rings


def cell_of(lon_e7, lat_e7):
    return (int(math.floor(lat_e7 * 1e-7 / CELL_DEG)), int(math.floor(lon_e7 * 1e-7 / CELL_DEG)))


def write_gabldg(out_dir, stem, recs, source, licence, attribution, bbox, refused, skipped, roof_shapes,
                 coordinates="int32, 1e-7 degree, WGS84 lon/lat (OSM's own precision)"):
    """recs: (cell, id, kind, (roof, h, mh, lv, mlv, rh), [(outer, int32 xy array)]) -> .bin + .json."""
    os.makedirs(out_dir, exist_ok=True)
    recs.sort(key=lambda r: (r[0], r[1]))

    bin_path = os.path.join(out_dir, stem + ".buildings.bin")
    index = []
    tagged = {"height": 0, "min_height": 0, "building:levels": 0, "roof:shape": 0}
    with open(bin_path, "wb") as f:
        f.write(b"GABLDG01")
        last = None
        for cell, sid, kind, (roof, h, mh, lv, mlv, rh), geo in recs:
            if cell != last:
                index.append([cell[1], cell[0], f.tell(), 0])
                last = cell
            index[-1][3] += 1
            tagged["height"] += not math.isnan(h)
            tagged["min_height"] += not math.isnan(mh)
            tagged["building:levels"] += not math.isnan(lv)
            tagged["roof:shape"] += roof != 255
            f.write(struct.pack("<qBBH5f", sid, kind, roof, len(geo), h, mh, lv, mlv, rh))
            for outer, xy in geo:
                f.write(struct.pack("<IB3x", len(xy), outer))
                f.write(np.ascontiguousarray(xy, "<i4").tobytes())

    kinds = [r[2] for r in recs]
    manifest = {
        "format": "GABLDG01",
        "source": source,
        "licence": licence, "attribution": attribution,
        "units": {"coordinates": coordinates,
                  "height": "metres above the ground at the footprint",
                  "levels": "count"},
        "datum": "ground",
        "bbox": bbox,
        "counts": {"records": len(recs), "buildings": kinds.count(0), "parts": kinds.count(1),
                   "tagged": tagged},
        "refused": refused,
        "skipped": skipped,
        "roofShapes": roof_shapes,
        "cellDeg": CELL_DEG,
        "cells": index,
    }
    with open(os.path.join(out_dir, stem + ".buildings.json"), "w") as f:
        json.dump(manifest, f, indent=1)
    return bin_path, tagged


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("pbf")
    ap.add_argument("--out", default=None)
    ap.add_argument("--bbox", type=float, nargs=4, default=None, metavar=("LON0", "LAT0", "LON1", "LAT1"))
    a = ap.parse_args()
    t0 = time.time()
    out_dir = a.out or os.path.join(os.path.dirname(os.path.abspath(a.pbf)), "buildings")
    os.makedirs(out_dir, exist_ok=True)
    stem = os.path.basename(a.pbf).split(".")[0]

    refused = {"height": 0, "min_height": 0, "roof:height": 0, "building:levels": 0,
               "building:min_level": 0}
    skipped = {"open_way": 0, "missing_node": 0, "relation_no_ring": 0, "outside_bbox": 0}
    roof_shapes = []

    def normalize(t):
        def length(k):
            if k not in t:
                return NAN
            v = metres(t[k])
            if v is None:
                refused[k] += 1
                return NAN
            return v

        def levels(k):
            if k not in t:
                return NAN
            v = count(t[k])
            if v is None:
                refused[k] += 1
                return NAN
            return v
        roof = 255
        if "roof:shape" in t:
            if t["roof:shape"] not in roof_shapes:
                roof_shapes.append(t["roof:shape"])
            roof = roof_shapes.index(t["roof:shape"])
        return (roof, length("height"), length("min_height"), levels("building:levels"),
                levels("building:min_level"), length("roof:height"))

    # ---- pass 1: building ways, building multipolygons, and where the node blocks are.
    solids = []          # [id, kind, attrs, [(outer, node id array)]]
    member_roles = {}    # way id -> list of (solid index, outer)
    way_blocks, dense_blocks = [], []
    hdr = osmpbf.header(a.pbf)
    for off, btype, raw in osmpbf.blobs(a.pbf):
        if btype != "OSMData":
            continue
        b = osmpbf.Block(raw)
        kinds = set(b.kinds())
        if "dense" in kinds or "nodes" in kinds:
            dense_blocks.append(off)
        if "ways" in kinds:
            way_blocks.append(off)
            bk = {b.index.get("building", -1), b.index.get("building:part", -1)}
            for wid, keys, vals, refs in b.ways():
                if bk.isdisjoint(keys):
                    continue
                t = tags_of(b, keys, vals)
                kind = is_building(t)
                if kind is None:
                    continue
                r = osmpbf.packed_delta(refs)
                if len(r) < 4 or r[0] != r[-1]:
                    skipped["open_way"] += 1
                    continue
                solids.append([wid, kind, normalize(t), [(1, r)]])
        if "relations" in kinds:
            for rid, keys, vals, mem, typ, roles in b.relations():
                t = tags_of(b, keys, vals)
                kind = is_building(t)
                if kind is None or t.get("type") != "multipolygon":
                    continue
                si = len(solids)
                solids.append([-rid, kind, normalize(t), []])
                for m, ty, ro in zip(mem, typ, roles):
                    if ty == 1:
                        member_roles.setdefault(int(m), []).append((si, b.strings[ro] != "inner"))
    print(f"[buildings] pass 1: {len(solids):,} building records "
          f"({len(member_roles):,} relation member ways) in {time.time() - t0:.0f} s")

    # ---- pass 2: the relations' member ways, stitched into rings per role.
    if member_roles:
        pieces = {}
        for off in way_blocks:
            b = osmpbf.Block(osmpbf.blob_at(a.pbf, off))
            for wid, keys, vals, refs in b.ways():
                if wid in member_roles:
                    for si, outer in member_roles[wid]:
                        pieces.setdefault((si, outer), []).append(osmpbf.packed_delta(refs))
        for (si, outer), ws in pieces.items():
            for ring in stitch(ws):
                solids[si][3].append((1 if outer else 0, np.asarray(ring, np.int64)))
        before = len(solids)
        solids = [s for s in solids if any(o for o, _ in s[3])]
        skipped["relation_no_ring"] = before - len(solids)
        print(f"[buildings] pass 2: member ways stitched in {time.time() - t0:.0f} s")

    # ---- pass 3: the coordinates of every node a ring names.
    need = np.unique(np.concatenate([r for s in solids for _, r in s[3]]))
    lat = np.full(need.size, np.iinfo(np.int64).min, np.int64)
    lon = np.zeros(need.size, np.int64)
    for off in dense_blocks:
        b = osmpbf.Block(osmpbf.blob_at(a.pbf, off))
        for ids, la, lo in b.dense():
            pos = np.searchsorted(need, ids)
            hit = pos < need.size
            hit[hit] = need[pos[hit]] == ids[hit]
            lat[pos[hit]] = la[hit]
            lon[pos[hit]] = lo[hit]
        for nid, la, lo in b.nodes():
            p = np.searchsorted(need, nid)
            if p < need.size and need[p] == nid:
                lat[p], lon[p] = la, lo
    print(f"[buildings] pass 3: {need.size:,} nodes placed in {time.time() - t0:.0f} s")

    # ---- write: records sorted by cell of their first node.
    have = lat != np.iinfo(np.int64).min
    recs = []
    for sid, kind, attrs, rings in solids:
        geo = []
        ok = True
        for outer, r in rings:
            p = np.searchsorted(need, r)
            if not have[p].all():
                ok = False
                break
            xy = np.stack([lon[p], lat[p]], 1)[:-1].astype(np.int32)   # drop the closing node
            geo.append((outer, xy))
        if not ok:
            skipped["missing_node"] += 1
            continue
        x0, y0 = geo[0][1][0]
        if a.bbox and not (a.bbox[0] <= x0 * 1e-7 <= a.bbox[2] and a.bbox[1] <= y0 * 1e-7 <= a.bbox[3]):
            skipped["outside_bbox"] += 1
            continue
        cell = cell_of(x0, y0)
        recs.append((cell, sid, kind, attrs, geo))
    md5 = hashlib.md5()
    with open(a.pbf, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 24), b""):
            md5.update(chunk)
    source = {"file": os.path.basename(a.pbf), "bytes": os.path.getsize(a.pbf), "md5": md5.hexdigest(),
              "header": hdr}
    bin_path, tagged = write_gabldg(out_dir, stem, recs, source, "ODbL 1.0", "(c) OpenStreetMap contributors",
                                    a.bbox, refused, skipped, roof_shapes)
    print(f"[buildings] {len(recs):,} solids -> {bin_path} "
          f"({os.path.getsize(bin_path) / 1048576:.0f} MB) in {time.time() - t0:.0f} s")
    print(f"[buildings] tagged {tagged}; refused {refused}; skipped {skipped}")


if __name__ == "__main__":
    sys.exit(main())
