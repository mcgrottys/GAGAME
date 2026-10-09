# ==================================================================================================
#  osmpbf.py - an OpenStreetMap PBF reader with no dependencies beyond zlib and numpy, in the
#  family of geotiff.py / grib2.py / hdf5.py: it reports the file faithfully and decides nothing.
#
#  THE FORMAT (wiki.openstreetmap.org/wiki/PBF_Format). A file is a run of
#      [uint32 BE length][BlobHeader][Blob]
#  BlobHeader {1 type: "OSMHeader" | "OSMData", 3 datasize}; Blob {1 raw | 2 raw_size, 3 zlib_data}.
#  An OSMData blob is a PrimitiveBlock {1 stringtable, 2 groups, 17 granularity (nanodegrees, 100),
#  19 lat_offset, 20 lon_offset}; a group holds ONE kind: 2 dense nodes, 3 ways or 4 relations
#  (1 plain nodes, rare). Ids, coordinates and way refs are packed, zigzagged and delta coded.
#
#  COORDINATES are kept as the file's own integers: OSM's precision is 1e-7 degree (1.1 cm), and
#  a block's coordinate is offset + granularity x value nanodegrees. With granularity 100 and the
#  offsets multiples of 100 (every planet and Geofabrik file) that is an exact int in 1e-7 degree;
#  any other block is refused by name rather than rounded.
#
#  Dense nodes decode in numpy (a block holds 8,000 of them); ways and relations are parsed per
#  message in Python, which is the pass's cost, so a caller filters on tags before decoding refs.
# ==================================================================================================
import struct
import zlib

import numpy as np


def _varint(b, p):
    r = 0
    s = 0
    while True:
        c = b[p]
        p += 1
        r |= (c & 0x7F) << s
        if c < 0x80:
            return r, p
        s += 7


def fields(b, p=0, end=None):
    """Yield (field number, value) over one message: an int for a varint, a slice for bytes."""
    end = len(b) if end is None else end
    while p < end:
        key, p = _varint(b, p)
        f, wt = key >> 3, key & 7
        if wt == 0:
            v, p = _varint(b, p)
        elif wt == 2:
            n, p = _varint(b, p)
            v = b[p:p + n]
            p += n
        elif wt == 1:
            v = b[p:p + 8]
            p += 8
        elif wt == 5:
            v = b[p:p + 4]
            p += 4
        else:
            raise ValueError(f"wire type {wt} not in the PBF format")
        yield f, v


def packed_u(b):
    """A packed varint field as uint64, every value, in numpy."""
    a = np.frombuffer(bytes(b), np.uint8)
    if a.size == 0:
        return np.zeros(0, np.uint64)
    ends = np.flatnonzero(a < 0x80)
    starts = np.empty_like(ends)
    starts[0] = 0
    starts[1:] = ends[:-1] + 1
    lens = ends - starts + 1
    out = np.zeros(ends.size, np.uint64)
    for k in range(int(lens.max())):
        m = lens > k
        out[m] |= (a[starts[m] + k] & 0x7F).astype(np.uint64) << np.uint64(7 * k)
    return out


def packed_s(b):
    """A packed sint (zigzag) field as int64."""
    u = packed_u(b)
    return (u >> np.uint64(1)).astype(np.int64) ^ -(u & np.uint64(1)).astype(np.int64)


def packed_delta(b):
    """A packed, zigzagged, delta-coded field (ids, lats, lons, refs) as absolute int64."""
    return np.cumsum(packed_s(b), dtype=np.int64)


def _small_u(b):
    """A short packed varint field (a way's keys) as a Python list: cheaper than numpy here."""
    out = []
    p = 0
    n = len(b)
    while p < n:
        v, p = _varint(b, p)
        out.append(v)
    return out


class Block:
    """One PrimitiveBlock: its strings, its coordinate frame and its groups, undecoded."""

    def __init__(self, raw):
        self.strings = []
        self.gran, self.lat_off, self.lon_off = 100, 0, 0
        self.groups = []
        for f, v in fields(raw):
            if f == 1:
                self.strings = [bytes(s).decode("utf-8", "replace") for g, s in fields(v) if g == 1]
            elif f == 2:
                self.groups.append(v)
            elif f == 17:
                self.gran = v
            elif f == 19:
                self.lat_off = v - (1 << 64) if v >= 1 << 63 else v
            elif f == 20:
                self.lon_off = v - (1 << 64) if v >= 1 << 63 else v
        if self.gran % 100 or self.lat_off % 100 or self.lon_off % 100:
            raise ValueError(f"block granularity {self.gran} / offsets {self.lat_off},{self.lon_off} "
                             "are not whole 1e-7 degree: refused rather than rounded")
        self.index = {s: i for i, s in enumerate(self.strings)}

    def kinds(self):
        """The kind of each group: 'dense', 'nodes', 'ways', 'relations' or 'changesets'."""
        names = {1: "nodes", 2: "dense", 3: "ways", 4: "relations", 5: "changesets"}
        out = []
        for g in self.groups:
            for f, _ in fields(g):
                out.append(names.get(f, "?"))
                break
            else:
                out.append("empty")
        return out

    def _e7(self, v, off):
        return (off + self.gran * v) // 100

    def dense(self):
        """Every dense node of the block: (ids, lat e7, lon e7) as int64 arrays."""
        for g in self.groups:
            for f, v in fields(g):
                if f != 2:
                    continue
                ids = lat = lon = None
                for h, w in fields(v):
                    if h == 1:
                        ids = packed_delta(w)
                    elif h == 8:
                        lat = packed_delta(w)
                    elif h == 9:
                        lon = packed_delta(w)
                if ids is not None:
                    yield ids, self._e7(lat, self.lat_off), self._e7(lon, self.lon_off)

    def nodes(self):
        """Plain (non-dense) nodes, one at a time: (id, lat e7, lon e7)."""
        for g in self.groups:
            for f, v in fields(g):
                if f != 1:
                    continue
                nid = la = lo = 0
                for h, w in fields(v):
                    if h == 1:
                        nid = (w >> 1) ^ -(w & 1)
                    elif h == 8:
                        la = (w >> 1) ^ -(w & 1)
                    elif h == 9:
                        lo = (w >> 1) ^ -(w & 1)
                yield nid, self._e7(la, self.lat_off), self._e7(lo, self.lon_off)

    def ways(self):
        """Every way, lazily: (id, key ids, val ids, refs slice). Decode refs with packed_delta."""
        for g in self.groups:
            for f, v in fields(g):
                if f != 3:
                    continue
                wid, keys, vals, refs = 0, (), (), b""
                for h, w in fields(v):
                    if h == 1:
                        wid = w
                    elif h == 2:
                        keys = _small_u(w)
                    elif h == 3:
                        vals = _small_u(w)
                    elif h == 8:
                        refs = w
                yield wid, keys, vals, refs

    def relations(self):
        """Every relation: (id, key ids, val ids, member ids, member types, role string ids)."""
        for g in self.groups:
            for f, v in fields(g):
                if f != 4:
                    continue
                rid, keys, vals, mem, typ, roles = 0, (), (), b"", b"", b""
                for h, w in fields(v):
                    if h == 1:
                        rid = w
                    elif h == 2:
                        keys = _small_u(w)
                    elif h == 3:
                        vals = _small_u(w)
                    elif h == 8:
                        roles = w
                    elif h == 9:
                        mem = w
                    elif h == 10:
                        typ = w
                yield (rid, keys, vals, packed_delta(mem), packed_u(typ).astype(np.int64),
                       packed_u(roles).astype(np.int64))


def blobs(path):
    """Yield (offset, type, raw bytes) for every blob of the file, decompressed."""
    with open(path, "rb") as f:
        while True:
            off = f.tell()
            hl = f.read(4)
            if len(hl) < 4:
                return
            (n,) = struct.unpack(">I", hl)
            hdr = f.read(n)
            btype, size = "", 0
            for fn, v in fields(hdr):
                if fn == 1:
                    btype = bytes(v).decode()
                elif fn == 3:
                    size = v
            yield off, btype, _decompress(f.read(size))


def blob_at(path, off):
    """The one blob at a byte offset blobs() reported, decompressed."""
    with open(path, "rb") as f:
        f.seek(off)
        (n,) = struct.unpack(">I", f.read(4))
        size = 0
        for fn, v in fields(f.read(n)):
            if fn == 3:
                size = v
        return _decompress(f.read(size))


def _decompress(blob):
    raw = zbuf = None
    for f, v in fields(blob):
        if f == 1:
            raw = v
        elif f == 3:
            zbuf = v
        elif f in (4, 5, 6, 7):
            raise ValueError(f"blob compression field {f} (lzma/bzip2/lz4/zstd) is not read here")
    return bytes(raw) if raw is not None else zlib.decompress(zbuf)


def header(path):
    """The OSMHeader's facts a manifest should carry: bbox, required features, replication time."""
    for _, btype, raw in blobs(path):
        if btype != "OSMHeader":
            continue
        out = {"required": [], "optional": []}
        for f, v in fields(raw):
            if f == 1:
                box = {}
                for h, w in fields(v):
                    box[h] = (w >> 1) ^ -(w & 1)
                out["bbox_nanodeg"] = [box.get(1), box.get(4), box.get(2), box.get(3)]  # W S E N
            elif f == 4:
                out["required"].append(bytes(v).decode())
            elif f == 5:
                out["optional"].append(bytes(v).decode())
            elif f == 16:
                out["writingprogram"] = bytes(v).decode()
            elif f == 32:
                out["replication_timestamp"] = v
        return out
    return {}
