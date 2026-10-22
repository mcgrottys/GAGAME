# ==================================================================================================
#  harvest_osm_water.py - the world's inland and coastal water bodies from OpenStreetMap into
#  GAWATR01, the inland twin of GALAND01 (harvest_land_polygons.py): the carve's water rings for
#  any place, each with its class and, where OSM gives one, its own level. Datasets by measurement,
#  never by place (Mark, 2026-10-21): this replaces nhd_water_ne (New England only) everywhere.
#
#  THE CHAIN (osmium for the node locations, as harvest_planet_buildings.py; this file's reader is
#  osmpbf.py):
#    O=/d/Tools/osmium/Library/bin/osmium.exe
#    $O tags-filter IN.osm.pbf w/natural=water w/waterway=riverbank w/landuse=reservoir \
#         r/natural=water r/waterway=riverbank r/landuse=reservoir -o W.osm.pbf
#    $O add-locations-to-ways W.osm.pbf -i dense_file_array,W.nodes.idx -o W-loc.osm.pbf  (planet)
#    $O add-locations-to-ways W.osm.pbf -o W-loc.osm.pbf                                   (a state)
#    py -3 harvester/harvest_osm_water.py build W-loc.osm.pbf OUT.gawatr [--workers 14]
#    py -3 harvester/harvest_osm_water.py compare OUT.gawatr data/gis/nhd_water_ne.bin --box -71.1,42.7,-70.75,42.9
#
#  WHAT IS WATER. natural=water areas, waterway=riverbank (the old river-area tag) and
#  landuse=reservoir|basin; closed ways and type=multipolygon relations (outer and inner rings
#  stitched by shared node ids). An open way so tagged is not an area and is counted, not kept.
#
#  THE CLASS (the carve's three, as GisMask reads NHD's FCodes):
#    0 sea        tidal=yes (any water), or water = lagoon | bay | sea | ocean | estuary | harbour |
#                 cove | fjord | strait | sound          -- the sea's level
#    1 impounded  water = lake | pond | reservoir | basin | oxbow | lake;pond | fishpond |
#                 reflecting_pool | wastewater | salt_pond | salt_pool | natural | moat | dam,
#                 landuse = reservoir | basin, and natural=water with no
#                 water=, or a water= not listed here    -- a level of its own
#    2 reach      water = river | canal | stream | ditch | drain | rapids | lock | stream_pool |
#                 creek | riverbank, and waterway=riverbank -- tidal where the solver stands
#  tidal=yes outranks every water=: an estuary drawn as water=river + tidal=yes is the sea's.
#  The LEVEL is ele= in metres (a trailing "m"; "ft" or ' converted at 0.3048), NaN when absent or
#  not a number; it is OSM's, never guessed.
#
#  GAWATR01 (little-endian; a ring is listed in EVERY 1-degree cell its box meets, so a reader takes
#  only the cells a tile covers, however large the lake):
#    magic "GAWATR01", u32 cells, u32 rings, u64 refs
#    cells x {i16 cx, i16 cy, u32 nrefs, u64 first}   sorted by (cy, cx); cx = floor(lon),
#                                                    cy = floor(lat); first = index into refs
#    refs  x u64                                      the file offset of a ring record
#    per ring: f32 lon0, lat0, lon1, lat1 (its box); u8 class (0 sea, 1 impounded, 2 reach);
#              u8 outer (1 outer, 0 inner); u16 0; f32 level_m (NaN: none); i64 osm (+way id,
#              -relation id: the rings of one polygon share it); u32 n; n x (f32 lon, f32 lat),
#              the ring open (its first point not repeated)
#  A point is in a polygon when it is inside an ODD number of that polygon's rings (outer and
#  inner both toggle); polygons overlap in OSM (a riverbank over a river area), so parity is per
#  polygon (`osm`), never across polygons. A sidecar OUT.json holds the counts and the mapping.
#
#  LICENCE: OSM data is ODbL 1.0 -- credit "(c) OpenStreetMap contributors"; a derived database
#  shared publicly is shared under ODbL.
# ==================================================================================================
import argparse
import array
import collections
import glob
import json
import math
import multiprocessing as mp
import os
import pickle
import shutil
import struct
import sys
import time

import numpy as np

import osmpbf

KEEP = {"natural", "water", "waterway", "landuse", "tidal", "ele", "type", "name"}
SEA = {"lagoon", "bay", "sea", "ocean", "estuary", "harbour", "cove", "fjord", "strait", "sound"}
REACH = {"river", "canal", "stream", "ditch", "drain", "rapids", "lock", "stream_pool", "creek", "riverbank"}
IMPOUNDED = {"lake", "pond", "reservoir", "basin", "oxbow", "lake;pond", "fishpond", "reflecting_pool",
             "wastewater", "salt_pond", "salt_pool", "natural", "moat", "dam"}
CLASS_NAMES = ["sea", "impounded", "reach"]
RING = struct.Struct("<4fBBHfqI")
NB = 64                      # relation buckets: a bucket's member ways are held at once
FLUSH = 32 << 20
NAN = float("nan")

_pbf = _tmp = _mw = _mb = None


def tags_of(b, keys, vals):
    s = b.strings
    return {s[k]: s[v] for k, v in zip(keys, vals) if s[k] in KEEP}


def is_water(t):
    return t.get("natural") == "water" or t.get("waterway") == "riverbank" or t.get("landuse") in ("reservoir", "basin")


def classify(t):
    """(class, unlisted water= value or None)."""
    if t.get("tidal") == "yes":
        return 0, None
    if t.get("waterway") == "riverbank":
        return 2, None
    w = t.get("water")
    if w in SEA:
        return 0, None
    if w in REACH:
        return 2, None
    if w in IMPOUNDED or w is None or t.get("landuse") in ("reservoir", "basin"):
        return 1, None
    return 1, w


def level_of(t):
    e = t.get("ele")
    if e is None:
        return NAN
    s = e.strip().lower()
    k = 1.0
    if s.endswith("ft") or s.endswith("'"):
        k, s = 0.3048, s.rstrip("ft'").strip()
    elif s.endswith("m"):
        s = s[:-1].strip()
    try:
        v = float(s.replace(",", "."))
    except ValueError:
        return NAN
    return v * k if math.isfinite(v) else NAN


def ring_record(cls, outer, level, osm, lon_e7, lat_e7):
    """lon/lat e7 int arrays of an OPEN ring -> the record's bytes."""
    lon = (np.asarray(lon_e7, np.float64) * 1e-7).astype("<f4")
    lat = (np.asarray(lat_e7, np.float64) * 1e-7).astype("<f4")
    head = RING.pack(float(lon.min()), float(lat.min()), float(lon.max()), float(lat.max()),
                     cls, outer, 0, level, osm, len(lon))
    return head + np.stack([lon, lat], 1).tobytes()


def blocks(path):
    """(offset, type) of every blob, by seeking past the data."""
    out = []
    with open(path, "rb") as f:
        while True:
            off = f.tell()
            hl = f.read(4)
            if len(hl) < 4:
                return out
            (n,) = struct.unpack(">I", hl)
            btype, size = "", 0
            for fn, v in osmpbf.fields(f.read(n)):
                if fn == 1:
                    btype = bytes(v).decode()
                elif fn == 3:
                    size = v
            f.seek(size, 1)
            out.append((off, btype))


def _init(pbf, tmp, mw=None, mb=None):
    global _pbf, _tmp, _mw, _mb
    _pbf, _tmp = pbf, tmp
    _mw = np.load(mw, mmap_mode="r") if mw else None
    _mb = np.load(mb, mmap_mode="r") if mb else None


class Spill:
    """A worker's ring records, spilled to a temp file as made (a planet block's rings are too many to
    hold or pickle back); the parent gets the file, each record's size and its box."""

    def __init__(self, name):
        self.path = os.path.join(_tmp, name)
        self.f = open(self.path, "wb")
        self.size, self.box = array.array("q"), array.array("f")

    def append(self, rec):
        self.f.write(rec)
        self.size.append(len(rec))
        self.box.frombytes(rec[:16])

    def done(self):
        self.f.close()
        return self.path, self.size.tobytes(), self.box.tobytes()


def pass1(offsets):
    """Water ways that close -> ring records (spilled); water multipolygons -> returned whole."""
    recs, rels, way_blocks, n = Spill(f"p1_{offsets[0]:016d}.bin"), [], [], collections.Counter()
    unlisted = collections.Counter()
    for off in offsets:
        b = osmpbf.Block(osmpbf.blob_at(_pbf, off))
        kinds = set(b.kinds())
        wk = {b.index.get("natural", -1), b.index.get("waterway", -1), b.index.get("landuse", -1)}
        if "ways" in kinds:
            way_blocks.append(off)
            for wid, keys, vals, refs, lats, lons in b.ways(locations=True):
                if wk.isdisjoint(keys):
                    continue
                t = tags_of(b, keys, vals)
                if not is_water(t):
                    continue
                r = osmpbf.packed_delta(refs)
                if len(r) < 4 or r[0] != r[-1]:
                    n["open_way"] += 1
                    continue
                if not lats:
                    n["no_locations"] += 1
                    continue
                x, y = b.way_e7(lats, lons)
                cls, u = classify(t)
                if u:
                    unlisted[u] += 1
                recs.append(ring_record(cls, 1, level_of(t), int(wid), x[:-1], y[:-1]))
                n[f"way_{CLASS_NAMES[cls]}"] += 1
        if "relations" in kinds:
            for rid, keys, vals, mem, typ, roles in b.relations():
                t = tags_of(b, keys, vals)
                if t.get("type") != "multipolygon" or not is_water(t):
                    continue
                cls, u = classify(t)
                if u:
                    unlisted[u] += 1
                members = [(int(m), b.strings[ro] != "inner") for m, ty, ro in zip(mem, typ, roles) if ty == 1]
                rels.append((int(rid), cls, level_of(t), members))
    return recs.done(), rels, way_blocks, n, unlisted


def pass2(offsets):
    """Every member way named in the shared arrays -> its bucket files (id, n, refs, lon, lat)."""
    buf, size, tag = collections.defaultdict(list), 0, f"{os.getpid()}"

    def flush():
        for bk, items in buf.items():
            with open(os.path.join(_tmp, f"m{bk:03d}.{tag}.part"), "ab") as f:
                f.write(b"".join(items))
        buf.clear()

    for off in offsets:
        b = osmpbf.Block(osmpbf.blob_at(_pbf, off))
        for wid, keys, vals, refs, lats, lons in b.ways(locations=True):
            p = int(np.searchsorted(_mw, wid))
            if p >= len(_mw) or _mw[p] != wid or not lats:
                continue
            r = osmpbf.packed_delta(refs).astype("<i8")
            x, y = b.way_e7(lats, lons)
            item = struct.pack("<qI", wid, len(r)) + r.tobytes() + x.astype("<i4").tobytes() + y.astype("<i4").tobytes()
            while p < len(_mw) and _mw[p] == wid:
                buf[int(_mb[p])].append(item)
                size += len(item)
                p += 1
            if size > FLUSH:
                flush()
                size = 0
    flush()
    return True


def stitch(ways):
    """Node-id lists -> closed rings by shared ends, O(n) by an end index. Unclosable pieces drop."""
    rings, open_ = [], []
    for w in ways:
        (rings if len(w) >= 4 and w[0] == w[-1] else open_).append(w)
    ends = collections.defaultdict(list)
    for i, w in enumerate(open_):
        ends[w[0]].append(i)
        ends[w[-1]].append(i)
    used = [False] * len(open_)
    for i in range(len(open_)):
        if used[i]:
            continue
        used[i] = True
        cur = list(open_[i])
        while cur[0] != cur[-1]:
            j = next((j for j in ends[cur[-1]] if not used[j]), None)
            if j is None:
                break
            used[j] = True
            w = open_[j]
            cur += w[1:] if w[0] == cur[-1] else w[-2::-1]
        if len(cur) >= 4 and cur[0] == cur[-1]:
            rings.append(cur)
    return rings


def pass3(bk):
    """One relation bucket: its member ways -> stitched ring records."""
    ways, pos = {}, {}
    for part in glob.glob(os.path.join(_tmp, f"m{bk:03d}.*.part")):
        data = open(part, "rb").read()
        p = 0
        while p < len(data):
            wid, n = struct.unpack_from("<qI", data, p)
            p += 12
            r = np.frombuffer(data, "<i8", n, p); p += 8 * n
            x = np.frombuffer(data, "<i4", n, p); p += 4 * n
            y = np.frombuffer(data, "<i4", n, p); p += 4 * n
            ways[wid] = r.tolist()
            pos.update(zip(ways[wid], zip(x.tolist(), y.tolist())))
        os.remove(part)
    rels = pickle.load(open(os.path.join(_tmp, f"r{bk:03d}.pkl"), "rb"))
    recs, n = Spill(f"p3_{bk:03d}.bin"), collections.Counter()
    for rid, cls, level, members in rels:
        got = 0
        for outer in (True, False):
            pieces = [ways[w] for w, o in members if o == outer and w in ways]
            for ring in stitch(pieces):
                xy = [pos[k] for k in ring[:-1]]
                recs.append(ring_record(cls, 1 if outer else 0, level, -rid, [a for a, _ in xy], [b for _, b in xy]))
                got += outer
                n[f"rel_{'outer' if outer else 'inner'}_{CLASS_NAMES[cls]}"] += 1
        if not got:
            n["rel_no_ring"] += 1
    return recs.done(), n


def chunks(seq, k):
    return [seq[i:i + k] for i in range(0, len(seq), k)]


class Writer:
    """Ring records appended to a temp file; each one's box and offset kept for the index."""

    def __init__(self, path):
        self.f = open(path, "wb")
        self.off, self.box = array.array("q"), array.array("f")   # compact: tens of millions at planet scale

    def add(self, spilled):
        path, size, box = spilled
        sz = np.frombuffer(size, np.int64)
        start = self.f.tell()
        self.off.frombytes((start + np.r_[0, np.cumsum(sz)[:-1]]).astype(np.int64).tobytes() if len(sz) else b"")
        self.box.frombytes(box)
        with open(path, "rb") as f:
            shutil.copyfileobj(f, self.f, 16 << 20)
        os.remove(path)


def write_index(out, w, tmp_rings):
    w.f.close()
    box = np.asarray(w.box, np.float64).reshape(-1, 4)
    off = np.asarray(w.off, np.int64)
    cx0 = np.clip(np.floor(box[:, 0]), -180, 179).astype(np.int64)
    cy0 = np.clip(np.floor(box[:, 1]), -90, 89).astype(np.int64)
    cx1 = np.clip(np.floor(box[:, 2]), -180, 179).astype(np.int64)
    cy1 = np.clip(np.floor(box[:, 3]), -90, 89).astype(np.int64)
    nx, ny = cx1 - cx0 + 1, cy1 - cy0 + 1
    rep = nx * ny
    ring = np.repeat(np.arange(len(off)), rep)
    k = np.arange(len(ring)) - np.repeat(np.cumsum(rep) - rep, rep)
    cx = cx0[ring] + k % nx[ring]
    cy = cy0[ring] + k // nx[ring]
    order = np.lexsort((ring, cx, cy))
    cx, cy, ring = cx[order], cy[order], ring[order]
    key = (cy + 90) * 360 + (cx + 180)
    starts = np.flatnonzero(np.r_[True, key[1:] != key[:-1]])
    counts = np.diff(np.r_[starts, len(key)])
    ncell, nref = len(starts), len(ring)
    head = 8 + 4 + 4 + 8
    rings_at = head + 16 * ncell + 8 * nref
    with open(out, "wb") as o:
        o.write(b"GAWATR01" + struct.pack("<IIQ", ncell, len(off), nref))
        cells = np.zeros(ncell, dtype=[("cx", "<i2"), ("cy", "<i2"), ("n", "<u4"), ("first", "<u8")])
        cells["cx"], cells["cy"], cells["n"], cells["first"] = cx[starts], cy[starts], counts, starts
        o.write(cells.tobytes())
        o.write((off[ring] + rings_at).astype("<u8").tobytes())
        with open(tmp_rings, "rb") as f:
            shutil.copyfileobj(f, o, 64 << 20)
    os.remove(tmp_rings)
    return ncell, nref


def build(a):
    t0 = time.time()
    pbf, out = a.pbf, a.out
    hdr = osmpbf.header(pbf)
    if "LocationsOnWays" not in hdr.get("optional", []):
        sys.exit("[water] no LocationsOnWays in the header: run osmium add-locations-to-ways first")
    tmp = out + ".tmp"
    os.makedirs(tmp, exist_ok=True)
    for f in glob.glob(os.path.join(tmp, "*.part")):
        os.remove(f)
    offs = [o for o, t in blocks(pbf) if t == "OSMData"]
    if not offs:
        sys.exit(f"[water] {pbf} holds no data blocks: nothing written")
    print(f"[water] {len(offs):,} data blocks; {a.workers} workers ({time.time() - t0:.0f} s)", flush=True)
    w = Writer(out + ".rings.tmp")
    counts, unlisted, rels, way_blocks = collections.Counter(), collections.Counter(), [], []
    with mp.Pool(a.workers, _init, (pbf, tmp)) as pool:
        for i, (recs, rr, wb, n, u) in enumerate(pool.imap(pass1, chunks(offs, 4))):
            w.add(recs)
            rels += rr
            way_blocks += wb
            counts.update(n)
            unlisted.update(u)
            if i % 100 == 0:
                print(f"[water] pass 1: {len(w.off):,} way rings, {len(rels):,} relations ({time.time() - t0:.0f} s)", flush=True)
    print(f"[water] pass 1 done: {len(w.off):,} way rings, {len(rels):,} multipolygons ({time.time() - t0:.0f} s)", flush=True)

    # the relations into NB buckets by id; their member ways' (id, bucket) as sorted shared arrays
    byb = collections.defaultdict(list)
    pairs = array.array("q")
    for rel in rels:
        bk = rel[0] % NB
        byb[bk].append(rel)
        for m, _ in rel[3]:
            pairs.extend((m, bk))
    for bk in range(NB):
        pickle.dump(byb.get(bk, []), open(os.path.join(tmp, f"r{bk:03d}.pkl"), "wb"))
    pairs = np.unique(np.asarray(pairs, np.int64).reshape(-1, 2), axis=0)
    mw, mb = os.path.join(tmp, "mw.npy"), os.path.join(tmp, "mb.npy")
    np.save(mw, pairs[:, 0].copy())
    np.save(mb, pairs[:, 1].copy())
    print(f"[water] {len(pairs):,} member way entries in {NB} buckets", flush=True)
    with mp.Pool(a.workers, _init, (pbf, tmp, mw, mb)) as pool:
        for _ in pool.imap_unordered(pass2, chunks(way_blocks, 4)):
            pass
    print(f"[water] pass 2 done ({time.time() - t0:.0f} s)", flush=True)
    with mp.Pool(min(a.workers, 6), _init, (pbf, tmp)) as pool:   # a bucket's ways in memory each
        for recs, n in pool.imap(pass3, range(NB)):
            w.add(recs)
            counts.update(n)
    print(f"[water] pass 3 done: {len(w.off):,} rings ({time.time() - t0:.0f} s)", flush=True)
    ncell, nref = write_index(out, w, out + ".rings.tmp")
    shutil.rmtree(tmp, ignore_errors=True)
    man = {"format": "GAWATR01", "source": os.path.basename(pbf), "osm_header": {k: v for k, v in hdr.items() if k != "bbox_nanodeg"},
           "classes": {"0": "sea", "1": "impounded", "2": "reach"},
           "mapping": {"sea": sorted(SEA) + ["tidal=yes (any)"], "impounded": sorted(IMPOUNDED) + ["landuse=reservoir|basin", "natural=water without water=", "any unlisted water="],
                       "reach": sorted(REACH) + ["waterway=riverbank"]},
           "rings": len(w.off), "cells": ncell, "refs": nref, "counts": dict(counts), "unlisted_water_values": dict(unlisted.most_common(40)),
           "licence": "ODbL 1.0, (c) OpenStreetMap contributors", "seconds": round(time.time() - t0, 1)}
    json.dump(man, open(os.path.splitext(out)[0] + ".json", "w"), indent=1)
    print(f"[water] wrote {out}: {len(w.off):,} rings in {ncell:,} cells ({nref:,} refs); {dict(counts)} ({time.time() - t0:.0f} s)", flush=True)


# ------------------------------------------------------------------ compare against nhd_water_ne

def clip_area_km2(xy, box):
    """Signed area (km2) of a ring clipped to a lon/lat box (Sutherland-Hodgman), local metres."""
    lon0, lat0, lon1, lat1 = box
    pts = [tuple(p) for p in xy]
    for edge in range(4):
        if not pts:
            return 0.0
        def inside(p):
            return (p[0] >= lon0, p[1] >= lat0, p[0] <= lon1, p[1] <= lat1)[edge]
        def cut(p, q):
            if edge in (0, 2):
                x = (lon0, None, lon1)[edge]
                t = (x - p[0]) / (q[0] - p[0])
            else:
                y = (None, lat0, None, lat1)[edge]
                t = (y - p[1]) / (q[1] - p[1])
            return (p[0] + t * (q[0] - p[0]), p[1] + t * (q[1] - p[1]))
        out, prev = [], pts[-1]
        for cur in pts:
            if inside(cur):
                if not inside(prev):
                    out.append(cut(prev, cur))
                out.append(cur)
            elif inside(prev):
                out.append(cut(prev, cur))
            prev = cur
        pts = out
    if len(pts) < 3:
        return 0.0
    a = np.asarray(pts)
    kx = 111.32 * math.cos(math.radians(0.5 * (lat0 + lat1)))
    x, y = a[:, 0] * kx, a[:, 1] * 110.574
    return 0.5 * float(np.dot(x, np.roll(y, -1)) - np.dot(y, np.roll(x, -1)))


def read_gawatr(path, box):
    with open(path, "rb") as f:
        assert f.read(8) == b"GAWATR01"
        ncell, nring, nref = struct.unpack("<IIQ", f.read(16))
        cells = np.frombuffer(f.read(16 * ncell), dtype=[("cx", "<i2"), ("cy", "<i2"), ("n", "<u4"), ("first", "<u8")])
        refs = np.frombuffer(f.read(8 * nref), "<u8")
        want = set()
        for c in cells:
            if c["cx"] + 1 > box[0] and c["cx"] < box[2] and c["cy"] + 1 > box[1] and c["cy"] < box[3]:
                want.update(refs[int(c["first"]):int(c["first"]) + int(c["n"])].tolist())
        for off in sorted(want):
            f.seek(off)
            lo0, la0, lo1, la1, cls, outer, _, level, osm, n = RING.unpack(f.read(RING.size))
            if lo1 < box[0] or lo0 > box[2] or la1 < box[1] or la0 > box[3]:
                continue
            yield cls, outer, level, osm, np.frombuffer(f.read(8 * n), "<f4").reshape(-1, 2).astype(np.float64)


def read_nhd(path, attrs, box):
    fam_cls = {445: 0, 493: 0, 390: 1, 436: 1}
    codes = [a.get("fcode", 0) for a in json.load(open(attrs))]
    with open(path, "rb") as f:
        (n,) = struct.unpack("<I", f.read(4))
        for i in range(n):
            (c,) = struct.unpack("<I", f.read(4))
            xy = np.frombuffer(f.read(8 * c), "<f4").reshape(-1, 2).astype(np.float64)
            if xy[:, 0].max() < box[0] or xy[:, 0].min() > box[2] or xy[:, 1].max() < box[1] or xy[:, 1].min() > box[3]:
                continue
            yield fam_cls.get(int(codes[i]) // 100, 2), xy


def compare(a):
    box = [float(v) for v in a.box.split(",")]
    osm = collections.defaultdict(lambda: [0, 0, 0.0])   # rings, inner rings, km2 (outer - inner)
    for cls, outer, level, oid, xy in read_gawatr(a.gawatr, box):
        s = osm[CLASS_NAMES[cls]]
        s[0] += 1
        s[1] += 0 if outer else 1
        s[2] += (1 if outer else -1) * abs(clip_area_km2(xy, box))
    nhd = collections.defaultdict(lambda: [0, 0.0])
    attrs = a.attrs or os.path.splitext(a.nhd)[0] + ".attrs.json"
    for cls, xy in read_nhd(a.nhd, attrs, box):
        s = nhd[CLASS_NAMES[cls]]
        s[0] += 1
        s[1] += abs(clip_area_km2(xy, box))
    print(f"box {box}")
    print(f"{'class':10} {'osm rings':>10} {'(inner)':>8} {'osm km2':>9} {'nhd rings':>10} {'nhd km2':>9}")
    for c in CLASS_NAMES:
        o, h = osm.get(c, [0, 0, 0.0]), nhd.get(c, [0, 0.0])
        print(f"{c:10} {o[0]:10d} {o[1]:8d} {o[2]:9.3f} {h[0]:10d} {h[1]:9.3f}")


def main():
    ap = argparse.ArgumentParser()
    sub = ap.add_subparsers(dest="cmd", required=True)
    b = sub.add_parser("build")
    b.add_argument("pbf")
    b.add_argument("out")
    b.add_argument("--workers", type=int, default=max(1, (os.cpu_count() or 2) - 2))
    c = sub.add_parser("compare")
    c.add_argument("gawatr")
    c.add_argument("nhd")
    c.add_argument("--attrs")
    c.add_argument("--box", default="-71.1,42.7,-70.75,42.9")
    a = ap.parse_args()
    build(a) if a.cmd == "build" else compare(a)


if __name__ == "__main__":
    main()
