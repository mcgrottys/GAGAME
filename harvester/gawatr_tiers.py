"""GAWATR01 size tiers (2026-10-21).

The planet's inland water (harvest_osm_water.py) holds 26 million rings, most of them ponds. A GIS-mask
tile reads every ring of every 1-degree cell its box touches, so a coarse tile -- a texel kilometres
across -- read whole continents of ponds it cannot resolve (measured: 8,536 cells, 10 GB, 186 s in one
600-frame run). A ring smaller than a texel cannot be told by that texel, so a tile whose texel is at
least T metres reads only the rings at least T across. This writes, beside the source file, one file per
threshold T holding exactly those rings, in the same GAWATR01 layout:

    "GAWATR01", u32 cells, u32 rings, u64 refs
    cells x {i16 cx, i16 cy, u32 nrefs, u64 first}   sorted (cy, cx); first = index into refs
    refs  x u64                                      absolute file offset of a ring record
    ring: f32 lon0,lat0,lon1,lat1; u8 class; u8 outer; u16 0; f32 level_m; i64 osm; u32 n; n x (f32 lon, f32 lat)

A ring's size is the larger side of its box in metres (the east side at the box's latitude nearest the
equator). Its cells are every 1-degree cell its box touches, as in the source.

    py -3 harvester/gawatr_tiers.py D:/DataCache/OSM/water/planet-261005-water.gawatr 1000 30000

writes planet-261005-water.t1000.gawatr and planet-261005-water.t30000.gawatr.
"""
import math
import mmap
import os
import struct
import sys
from collections import defaultdict

HEAD = struct.Struct("<8sIIQ")
CELL = struct.Struct("<hhIQ")
RING = struct.Struct("<4fBBHfqI")   # 36 bytes
M_PER_DEG = 111194.93


def ring_size_m(lon0, lat0, lon1, lat1):
    lat_eq = 0.0 if lat0 <= 0.0 <= lat1 else min(abs(lat0), abs(lat1))
    ew = (lon1 - lon0) * M_PER_DEG * math.cos(math.radians(lat_eq))
    ns = (lat1 - lat0) * M_PER_DEG
    return max(ew, ns)


def main():
    src = sys.argv[1]
    tiers = sorted(int(t) for t in sys.argv[2:])
    with open(src, "rb") as f:
        mm = mmap.mmap(f.fileno(), 0, access=mmap.ACCESS_READ)
    magic, ncell, nring, nref = HEAD.unpack_from(mm, 0)
    assert magic == b"GAWATR01", magic
    rings_at = HEAD.size + ncell * CELL.size + nref * 8
    size = len(mm)
    kept = {t: [] for t in tiers}          # per tier: (offset, length, box)
    off = rings_at
    seen = 0
    while off < size:
        lon0, lat0, lon1, lat1, cls, outer, _pad, level, osm, n = RING.unpack_from(mm, off)
        length = RING.size + 8 * n
        s = ring_size_m(lon0, lat0, lon1, lat1)
        for t in tiers:
            if s >= t:
                kept[t].append((off, length, lon0, lat0, lon1, lat1))
        off += length
        seen += 1
        if seen % 2_000_000 == 0:
            print(f"[tiers] {seen:,} of {nring:,} rings read", flush=True)
    print(f"[tiers] {seen:,} rings read; kept " + ", ".join(f"{len(kept[t]):,} at >= {t} m" for t in tiers), flush=True)
    base, ext = os.path.splitext(src)
    for t in tiers:
        rings = kept[t]
        cells = defaultdict(list)
        for i, (_o, _l, lon0, lat0, lon1, lat1) in enumerate(rings):
            for cy in range(max(-90, math.floor(lat0)), min(89, math.floor(lat1)) + 1):
                for cx in range(math.floor(lon0), math.floor(lon1) + 1):
                    wx = ((cx + 180) % 360 + 360) % 360 - 180
                    cells[(cy, wx)].append(i)
        keys = sorted(cells)
        nrefs = sum(len(cells[k]) for k in keys)
        ring_base = HEAD.size + len(keys) * CELL.size + nrefs * 8
        new_off = []
        at = ring_base
        for (_o, length, *_b) in rings:
            new_off.append(at)
            at += length
        out = f"{base}.t{t}{ext}"
        tmp = out + ".part"
        with open(tmp, "wb") as g:
            g.write(HEAD.pack(b"GAWATR01", len(keys), len(rings), nrefs))
            first = 0
            for (cy, cx) in keys:
                g.write(CELL.pack(cx, cy, len(cells[(cy, cx)]), first))
                first += len(cells[(cy, cx)])
            for k in keys:
                g.write(struct.pack(f"<{len(cells[k])}Q", *(new_off[i] for i in cells[k])))
            for (o, length, *_b) in rings:
                g.write(mm[o:o + length])
        os.replace(tmp, out)
        print(f"[tiers] wrote {out}: {len(rings):,} rings in {len(keys):,} cells, {os.path.getsize(out) / 1e6:.0f} MB", flush=True)


if __name__ == "__main__":
    main()
