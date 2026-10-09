# ==================================================================================================
#  harvest_planet_buildings.py - harvest_buildings.py's solids at the scale of the planet.
#
#  WHY A SECOND HARVESTER. harvest_buildings.py holds every building and a node table in memory: a
#  state (2.7 M buildings) in 3 minutes, the planet (~600 M) not at all. The node table is the job
#  osmium-tool does at planet scale (the de facto standard), so it is osmium's, and this reads what
#  osmium leaves: every building way already carrying its own coordinates.
#
#    osmium tags-filter planet.osm.pbf w/building w/building:part r/building r/building:part -o b.pbf
#    osmium add-locations-to-ways b.pbf -i dense_file_array,<node index> -o b-loc.pbf
#    py -3 harvester/harvest_planet_buildings.py b-loc.pbf [--workers 14] [--out DIR]
#
#  THE SAME LAWS AS harvest_buildings.py (its helpers, imported): heights normalized to metres,
#  refused not guessed; floors stay counts; the datum is the ground; coordinates the file's own
#  1e-7 degree; multipolygons stitched by shared ends. One difference: roof shapes are a FIXED
#  vocabulary (OSM's documented values), because several processes write records at once; any other
#  value is 254, "other".
#
#  PASSES (a pool of processes over the file's blocks, found by a seek-only walk of its headers):
#    1  building ways -> records, bucketed by 2.5 degree tile into per-process temp files; building
#       multipolygon relations returned whole (they are few).
#    2  the relations' member ways (ids in a shared sorted array) -> their coordinates; the parent
#       stitches rings and writes the relations' records into its own buckets.
#    3  each bucket sorted by cell (0.05 deg) and id into a chunk; the chunks concatenated in order.
#  The output is GABLDG01 records exactly as harvest_buildings.py writes them; the cell index, too
#  large for a JSON manifest at this scale, is a binary sidecar `<stem>.buildings.idx`: int32 ix,
#  int32 iy, int64 offset, int64 count, sorted by (iy, ix), named by the manifest's "cellIndex".
#
#  LICENCE: OSM data is ODbL 1.0 -- offline copies are allowed; credit "(c) OpenStreetMap
#  contributors", and a derived database shared publicly is shared under ODbL.
# ==================================================================================================
import argparse
import collections
import glob
import json
import math
import multiprocessing as mp
import os
import struct
import sys
import time

import numpy as np

import osmpbf
from harvest_buildings import CELL_DEG, NAN, attrs_of, cell_of, is_building, stitch, tags_of

BUCKET = int(round(2.5 / CELL_DEG))   # cells a bucket spans on each axis (2.5 degrees: a dense one
                                      # is a few GB to sort, and several sort at once)
ROOFS = ["flat", "skillion", "gabled", "half-hipped", "hipped", "pyramidal", "gambrel", "mansard",
         "dome", "onion", "round", "saltbox", "quadruple_saltbox", "sawtooth", "cone", "crosspitched",
         "side_hipped", "side_half-hipped", "double_saltbox", "gabled_height_moved", "butterfly", "many"]
ENTRY = struct.Struct("<iiqI")   # a temp entry's head: iy, ix, id, the record's byte length
FLUSH = 32 << 20                 # bytes a process buffers before appending to its bucket files

_pbf = _tmp = None
_members = None


def blocks(path):
    """(offset, type) of every blob, by seeking past the data: no decompression."""
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


def record(sid, kind, attrs, geo):
    roof, h, mh, lv, mlv, rh = attrs
    parts = [struct.pack("<qBBH5f", sid, kind, roof, len(geo), h, mh, lv, mlv, rh)]
    for outer, xy in geo:
        parts.append(struct.pack("<IB3x", len(xy), outer))
        parts.append(np.ascontiguousarray(xy, "<i4").tobytes())
    return b"".join(parts)


def bucket_of(cell):
    return f"{cell[0] // BUCKET:+04d}_{cell[1] // BUCKET:+04d}"


class Buckets:
    """Per-process buffers of temp entries, appended to <tmp>/<bucket>.<pid>.part."""

    def __init__(self, tmp, tag):
        self.tmp, self.tag, self.buf, self.size = tmp, tag, collections.defaultdict(list), 0

    def add(self, cell, sid, rec):
        self.buf[bucket_of(cell)].append(ENTRY.pack(cell[0], cell[1], sid, len(rec)) + rec)
        self.size += len(rec) + ENTRY.size
        if self.size > FLUSH:
            self.flush()

    def flush(self):
        for b, items in self.buf.items():
            with open(os.path.join(self.tmp, f"{b}.{self.tag}.part"), "ab") as f:
                f.write(b"".join(items))
        self.buf.clear()
        self.size = 0


def _init(pbf, tmp, members_path=None):
    global _pbf, _tmp, _members
    _pbf, _tmp = pbf, tmp
    _members = np.load(members_path, mmap_mode="r") if members_path else None


def _refused():
    return {"height": 0, "min_height": 0, "roof:height": 0, "building:levels": 0, "building:min_level": 0}


def pass1(offsets):
    """Building ways -> bucketed records; building multipolygons -> returned."""
    out = Buckets(_tmp, f"w{os.getpid()}")
    refused, n, skipped, rels, way_blocks = _refused(), 0, collections.Counter(), [], []
    for off in offsets:
        b = osmpbf.Block(osmpbf.blob_at(_pbf, off))
        kinds = set(b.kinds())
        if "ways" in kinds:
            way_blocks.append(off)
            bk = {b.index.get("building", -1), b.index.get("building:part", -1)}
            for wid, keys, vals, refs, lats, lons in b.ways(locations=True):
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
                if not lats:
                    skipped["no_locations"] += 1
                    continue
                x, y = b.way_e7(lats, lons)
                xy = np.stack([x, y], 1)[:-1].astype(np.int32)   # drop the closing node
                cell = cell_of(int(xy[0, 0]), int(xy[0, 1]))
                out.add(cell, wid, record(wid, kind, attrs_of(t, refused, ROOFS, grow=False), [(1, xy)]))
                n += 1
        if "relations" in kinds:
            for rid, keys, vals, mem, typ, roles in b.relations():
                t = tags_of(b, keys, vals)
                kind = is_building(t)
                if kind is None or t.get("type") != "multipolygon":
                    continue
                members = [(int(m), b.strings[ro] != "inner") for m, ty, ro in zip(mem, typ, roles) if ty == 1]
                rels.append((rid, kind, attrs_of(t, refused, ROOFS, grow=False), members))
    out.flush()
    return n, refused, dict(skipped), rels, way_blocks


def pass2(offsets):
    """The coordinates of every member way named in the shared array: {id: [(node, lon, lat)]}."""
    got = {}
    for off in offsets:
        b = osmpbf.Block(osmpbf.blob_at(_pbf, off))
        for wid, keys, vals, refs, lats, lons in b.ways(locations=True):
            p = np.searchsorted(_members, wid)
            if p >= len(_members) or _members[p] != wid or not lats:
                continue
            x, y = b.way_e7(lats, lons)
            got[wid] = list(zip(osmpbf.packed_delta(refs).tolist(), x.tolist(), y.tolist()))
    return got


def pass3(args):
    """One bucket's parts -> a chunk sorted by (iy, ix, id), and its cells (relative offsets)."""
    bucket, parts, chunk = args
    data = b"".join(open(p, "rb").read() for p in parts)
    heads, p = [], 0
    while p < len(data):
        iy, ix, sid, n = ENTRY.unpack_from(data, p)
        heads.append((iy, ix, sid, p + ENTRY.size, n))
        p += ENTRY.size + n
    heads.sort()
    cells, off = [], 0
    with open(chunk, "wb") as f:
        for iy, ix, sid, start, n in heads:
            if not cells or cells[-1][0] != iy or cells[-1][1] != ix:
                cells.append([iy, ix, off, 0])
            cells[-1][3] += 1
            f.write(data[start:start + n])
            off += n
    for q in parts:
        os.remove(q)
    return bucket, chunk, cells, len(heads)


def chunks(seq, n):
    return [seq[i:i + n] for i in range(0, len(seq), n)]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("pbf", help="a buildings .osm.pbf with LocationsOnWays (osmium add-locations-to-ways)")
    ap.add_argument("--out", default=None)
    ap.add_argument("--workers", type=int, default=max(1, os.cpu_count() - 2))
    a = ap.parse_args()
    t0 = time.time()
    hdr = osmpbf.header(a.pbf)
    if "LocationsOnWays" not in hdr.get("optional", []):
        sys.exit("[planet] no LocationsOnWays in the header: run osmium add-locations-to-ways first")
    out_dir = a.out or os.path.join(os.path.dirname(os.path.abspath(a.pbf)), "buildings")
    stem = os.path.basename(a.pbf).split(".")[0]
    tmp = os.path.join(out_dir, stem + ".tmp")
    os.makedirs(tmp, exist_ok=True)
    for q in glob.glob(os.path.join(tmp, "*")):
        os.remove(q)

    data_blocks = [off for off, t in blocks(a.pbf) if t == "OSMData"]
    print(f"[planet] {len(data_blocks):,} data blocks; {a.workers} workers ({time.time() - t0:.0f} s)", flush=True)
    refused, skipped, n_ways, rels, way_blocks = _refused(), collections.Counter(), 0, [], []
    with mp.Pool(a.workers, _init, (a.pbf, tmp)) as pool:
        for i, (n, rf, sk, rl, wb) in enumerate(pool.imap_unordered(pass1, chunks(data_blocks, 64))):
            n_ways += n
            for k, v in rf.items():
                refused[k] += v
            skipped.update(sk)
            rels += rl
            way_blocks += wb
            if i % 200 == 0:
                print(f"[planet] pass 1: {n_ways:,} building ways, {len(rels):,} relations ({time.time() - t0:.0f} s)", flush=True)
    print(f"[planet] pass 1 done: {n_ways:,} ways, {len(rels):,} multipolygons ({time.time() - t0:.0f} s)", flush=True)

    members_path = os.path.join(tmp, "members.npy")
    np.save(members_path, np.unique(np.asarray([m for r in rels for m, _ in r[3]], np.int64)))
    pieces = {}
    with mp.Pool(a.workers, _init, (a.pbf, tmp, members_path)) as pool:
        for got in pool.imap_unordered(pass2, chunks(sorted(way_blocks), 64)):
            pieces.update(got)
    own = Buckets(tmp, "rel")
    for rid, kind, attrs, members in rels:
        geo = []
        for outer in (True, False):
            for ring in stitch([pieces[m] for m, o in members if o == outer and m in pieces]):
                geo.append((1 if outer else 0, np.asarray([(x, y) for _, x, y in ring[:-1]], np.int32)))
        geo.sort(key=lambda g: -g[0])   # outer rings first
        if not geo or not geo[0][0]:
            skipped["relation_no_ring"] += 1
            continue
        own.add(cell_of(int(geo[0][1][0, 0]), int(geo[0][1][0, 1])), -rid, record(-rid, kind, attrs, geo))
    own.flush()
    del pieces
    print(f"[planet] pass 2 done: relations stitched ({time.time() - t0:.0f} s)", flush=True)

    parts = collections.defaultdict(list)
    for q in glob.glob(os.path.join(tmp, "*.part")):
        parts[os.path.basename(q).split(".")[0]].append(q)
    jobs = [(b, parts[b], os.path.join(tmp, b + ".chunk")) for b in sorted(parts)]
    bin_path = os.path.join(out_dir, stem + ".buildings.bin")
    cells, total = [], 0
    with mp.Pool(min(a.workers, 6)) as pool, open(bin_path, "wb") as f:
        f.write(b"GABLDG01")
        done = {}
        order = [j[0] for j in jobs]
        nxt = 0
        for bucket, chunk, bc, n in pool.imap_unordered(pass3, jobs):
            done[bucket] = (chunk, bc, n)
            while nxt < len(order) and order[nxt] in done:   # concatenated in bucket order
                ch, bc2, n2 = done.pop(order[nxt])
                base = f.tell()
                with open(ch, "rb") as g:
                    while True:
                        buf = g.read(64 << 20)
                        if not buf:
                            break
                        f.write(buf)
                os.remove(ch)
                cells += [(iy, ix, base + o, c) for iy, ix, o, c in bc2]
                total += n2
                nxt += 1
    idx = np.array(sorted(cells), dtype=np.int64)   # (iy, ix, offset, count) by (iy, ix)
    rec = np.zeros(len(idx), dtype=[("ix", "<i4"), ("iy", "<i4"), ("off", "<i8"), ("n", "<i8")])
    rec["iy"], rec["ix"], rec["off"], rec["n"] = idx[:, 0], idx[:, 1], idx[:, 2], idx[:, 3]
    rec.tofile(os.path.join(out_dir, stem + ".buildings.idx"))
    manifest = {
        "format": "GABLDG01",
        "source": {"file": os.path.basename(a.pbf), "bytes": os.path.getsize(a.pbf), "header": hdr,
                   "tool": "osmium tags-filter + add-locations-to-ways"},
        "licence": "ODbL 1.0", "attribution": "(c) OpenStreetMap contributors",
        "units": {"coordinates": "int32, 1e-7 degree, WGS84 lon/lat (OSM's own precision)",
                  "height": "metres above the ground at the footprint", "levels": "count"},
        "datum": "ground",
        "counts": {"records": total, "ways": n_ways, "multipolygons": len(rels)},
        "refused": refused, "skipped": dict(skipped),
        "roofShapes": ROOFS + ["(other: 254)"],
        "cellDeg": CELL_DEG,
        "cellIndex": stem + ".buildings.idx",
        "cellCount": len(rec),
    }
    with open(os.path.join(out_dir, stem + ".buildings.json"), "w") as f:
        json.dump(manifest, f, indent=1)
    os.remove(members_path)
    if not os.listdir(tmp):
        os.rmdir(tmp)
    print(f"[planet] {total:,} solids in {len(rec):,} cells -> {bin_path} "
          f"({os.path.getsize(bin_path) / 2**30:.1f} GB) in {(time.time() - t0) / 60:.0f} min", flush=True)
    print(f"[planet] refused {refused}; skipped {dict(skipped)}", flush=True)


if __name__ == "__main__":
    sys.exit(main())
