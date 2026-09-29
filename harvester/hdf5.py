"""Minimal HDF5 reader, stdlib only (struct + zlib), for netCDF-4 files -- the NOS operational
forecast systems (CBOFS & co.) on the NODD bucket are netCDF-4, i.e. HDF5, which netcdf3.py
refuses on purpose.

Scope is exactly what those files use, and anything else raises loudly rather than guessing:
  * superblock v0/v1 and v2/v3; object headers v1 and v2 (with continuation blocks);
  * groups old-style (symbol table: v1 B-tree + local heap) and new-style (link messages,
    compact or DENSE: fractal heap + v2 B-tree, depth <= 1);
  * attributes compact or dense; fixed-point, floating-point and fixed-length string types;
  * datasets compact, contiguous, or chunked with a v1 B-tree index (layout message v3), with
    the deflate, shuffle and fletcher32 filters.
Reads are whole-variable (the files are tens of MB); values come back as array('f'/'d'/...)
in row-major order with the variable's shape.

Written against the HDF5 File Format Specification version 3.0 (The HDF Group).
"""

import struct
import zlib
from array import array

UNDEF = None


class H5Error(ValueError):
    pass


def _u(b):
    return int.from_bytes(b, "little")


class _Datatype:
    def __init__(self, raw):
        self.cls = raw[0] & 0x0F
        self.version = raw[0] >> 4
        bits = raw[1:4]
        self.size = _u(raw[4:8])
        self.big = bool(bits[0] & 1)
        self.signed = bool(bits[0] & 8)
        self.code = None
        if self.cls == 0:                                   # fixed-point
            self.code = {1: "b", 2: "h", 4: "i", 8: "q"}.get(self.size)
            if self.code and not self.signed:
                self.code = self.code.upper()
        elif self.cls == 1:                                 # floating-point
            self.code = {4: "f", 8: "d"}.get(self.size)
        elif self.cls == 3:                                 # fixed-length string
            self.code = "s"

    def decode(self, raw, count):
        if self.code == "s":
            out = [raw[i * self.size:(i + 1) * self.size].split(b"\0")[0].decode(
                "utf-8", errors="replace").rstrip() for i in range(count)]
            return out[0] if count == 1 else out
        if self.code is None:
            return None                                     # vlen, references, compounds
        vals = struct.unpack(("<" if not self.big else ">") + f"{count}{self.code}",
                             raw[:count * self.size])
        return vals[0] if count == 1 else list(vals)


def _dataspace(raw, L):
    ver = raw[0]
    rank = raw[1]
    if ver == 1:
        off = 8
    elif ver == 2:
        off = 4
        if raw[3] == 0:          # scalar
            return []
        if raw[3] == 2:          # null
            return None
    else:
        raise H5Error(f"dataspace version {ver}")
    return [_u(raw[off + i * L:off + (i + 1) * L]) for i in range(rank)]


class H5File:
    def __init__(self, source):
        if isinstance(source, (bytes, bytearray, memoryview)):
            self.d = bytes(source)
        else:
            with open(source, "rb") as f:
                self.d = f.read()
        d = self.d
        base = 0
        while d[base:base + 8] != b"\x89HDF\r\n\x1a\n":
            base = 512 if base == 0 else base * 2
            if base > len(d):
                raise H5Error("no HDF5 signature")
        ver = d[base + 8]
        self.sb_version = ver
        if ver in (0, 1):
            self.O = d[base + 13]
            self.L = d[base + 14]
            p = base + 24 + (4 if ver == 1 else 0)
            p += 4 * self.O                  # base, free-space, EOF, driver addresses
            # root group symbol table entry: link name offset, object header address, ...
            self.root_addr = _u(d[p + self.O:p + 2 * self.O])
            cache_type = _u(d[p + 2 * self.O:p + 2 * self.O + 4])
            self._root_scratch = (cache_type, d[p + 2 * self.O + 8:p + 2 * self.O + 24])
        elif ver in (2, 3):
            self.O = d[base + 9]
            self.L = d[base + 10]
            p = base + 12 + 3 * self.O       # base, superblock extension, EOF
            self.root_addr = _u(d[p:p + self.O])
        else:
            raise H5Error(f"superblock version {ver}")
        self.undef = (1 << (8 * self.O)) - 1

    # -------------------------------------------------------------------- primitives
    def addr(self, p):
        return _u(self.d[p:p + self.O])

    def length(self, p):
        return _u(self.d[p:p + self.L])

    # -------------------------------------------------------------------- object headers
    def messages(self, addr):
        """[(type, raw_bytes)] of an object header, continuation blocks followed."""
        d = self.d
        out = []
        if d[addr:addr + 4] == b"OHDR":
            flags = d[addr + 5]
            p = addr + 6
            if flags & 0x20:
                p += 16
            if flags & 0x10:
                p += 4
            nsz = 1 << (flags & 3)
            size0 = _u(d[p:p + nsz])
            p += nsz
            blocks = [(p, p + size0)]
            creation = bool(flags & 0x04)
            while blocks:
                s, e = blocks.pop(0)
                q = s
                hdr = 4 + (2 if creation else 0)
                while q + hdr <= e:
                    mtype = d[q]
                    msize = _u(d[q + 1:q + 3])
                    q += hdr
                    body = d[q:q + msize]
                    q += msize
                    if mtype == 0x10:
                        caddr, clen = self.addr_len(body)
                        if d[caddr:caddr + 4] != b"OCHK":
                            raise H5Error("continuation block without OCHK")
                        blocks.append((caddr + 4, caddr + clen - 4))
                    elif mtype != 0:
                        out.append((mtype, body))
            return out
        if d[addr] != 1:
            raise H5Error(f"object header version {d[addr]} at {addr}")
        nmsgs = _u(d[addr + 2:addr + 4])
        size0 = _u(d[addr + 8:addr + 12])
        blocks = [(addr + 16, addr + 16 + size0)]
        seen = 0
        while blocks and seen < nmsgs:
            s, e = blocks.pop(0)
            q = s
            while q + 8 <= e and seen < nmsgs:
                mtype = _u(d[q:q + 2])
                msize = _u(d[q + 2:q + 4])
                body = d[q + 8:q + 8 + msize]
                q += 8 + msize
                seen += 1
                if mtype == 0x10:
                    caddr, clen = self.addr_len(body)
                    blocks.append((caddr, caddr + clen))
                elif mtype != 0:
                    out.append((mtype, body))
        return out

    def addr_len(self, body):
        return _u(body[:self.O]), _u(body[self.O:self.O + self.L])

    # -------------------------------------------------------------------- groups
    def links(self, addr):
        """{name: object header address} of the group at addr."""
        msgs = self.messages(addr)
        out = {}
        for mtype, body in msgs:
            if mtype == 0x11:                          # old-style: symbol table
                btree, heap = self.addr_len(body[:2 * self.O])[0], _u(body[self.O:2 * self.O])
                out.update(self._symbol_table(btree, heap))
            elif mtype == 0x06:                        # new-style, compact
                name, target = self._link(body)
                if target is not None:
                    out[name] = target
            elif mtype == 0x02:                        # new-style, dense
                flags = body[1]
                p = 2 + (8 if flags & 1 else 0)
                fheap = _u(body[p:p + self.O])
                bt = _u(body[p + self.O:p + 2 * self.O])
                if fheap != self.undef and bt != self.undef:
                    heap = _FractalHeap(self, fheap)
                    for rec in self._btree2_records(bt):
                        name, target = self._link(heap.get(rec[4:]))
                        if target is not None:
                            out[name] = target
        return out

    def _link(self, body):
        flags = body[1]
        p = 2
        ltype = 0
        if flags & 0x08:
            ltype = body[p]
            p += 1
        if flags & 0x04:
            p += 8
        if flags & 0x10:
            p += 1
        nsz = 1 << (flags & 3)
        n = _u(body[p:p + nsz])
        p += nsz
        name = body[p:p + n].decode("utf-8", errors="replace")
        p += n
        if ltype != 0:
            return name, None                         # soft/external links: not followed
        return name, _u(body[p:p + self.O])

    def _symbol_table(self, btree, heap):
        d = self.d
        if d[heap:heap + 4] != b"HEAP":
            raise H5Error("local heap signature")
        data_addr = self.addr(heap + 8 + 2 * self.L)
        out = {}

        def name_at(off):
            e = d.index(b"\0", data_addr + off)
            return d[data_addr + off:e].decode("utf-8", errors="replace")

        def walk(node):
            if d[node:node + 4] != b"TREE":
                raise H5Error("group B-tree signature")
            level = d[node + 5]
            used = _u(d[node + 6:node + 8])
            p = node + 8 + 2 * self.O
            children = []
            for i in range(used):
                p += self.L                                   # key i (heap offset)
                children.append(self.addr(p))
                p += self.O
            for c in children:
                if level > 0:
                    walk(c)
                else:
                    if d[c:c + 4] != b"SNOD":
                        raise H5Error("symbol node signature")
                    n = _u(d[c + 6:c + 8])
                    q = c + 8
                    for _ in range(n):
                        off = self.length(q) if self.L == self.O else self.addr(q)
                        obj = self.addr(q + self.O)
                        out[name_at(off)] = obj
                        q += 2 * self.O + 24
        walk(btree)
        return out

    def _btree2_records(self, addr):
        d = self.d
        if d[addr:addr + 4] != b"BTHD":
            raise H5Error("v2 B-tree header signature")
        node_size = _u(d[addr + 6:addr + 10])
        rec_size = _u(d[addr + 10:addr + 12])
        depth = _u(d[addr + 12:addr + 14])
        root = self.addr(addr + 16)
        nroot = _u(d[addr + 16 + self.O:addr + 18 + self.O])

        def leaf(node, n):
            if d[node:node + 4] != b"BTLF":
                raise H5Error("v2 B-tree leaf signature")
            p = node + 6
            return [d[p + i * rec_size:p + (i + 1) * rec_size] for i in range(n)]

        if depth == 0:
            return leaf(root, nroot)
        if depth != 1:
            raise H5Error(f"v2 B-tree depth {depth} not supported")
        if d[root:root + 4] != b"BTIN":
            raise H5Error("v2 B-tree internal signature")
        max_leaf = (node_size - 10) // rec_size
        m = (max_leaf.bit_length() + 7) // 8
        p = root + 6
        recs = [d[p + i * rec_size:p + (i + 1) * rec_size] for i in range(nroot)]
        p += nroot * rec_size
        out = []
        for i in range(nroot + 1):
            child = self.addr(p)
            n = _u(d[p + self.O:p + self.O + m])
            p += self.O + m
            out += leaf(child, n)
            if i < nroot:
                out.append(recs[i])
        return out

    # -------------------------------------------------------------------- attributes
    def attributes(self, addr, msgs=None):
        msgs = self.messages(addr) if msgs is None else msgs
        out = {}
        for mtype, body in msgs:
            if mtype == 0x0C:
                k, v = self._attribute(body)
                out[k] = v
            elif mtype == 0x15:
                flags = body[1]
                p = 2 + (2 if flags & 1 else 0)
                fheap = _u(body[p:p + self.O])
                bt = _u(body[p + self.O:p + 2 * self.O])
                if fheap != self.undef and bt != self.undef:
                    heap = _FractalHeap(self, fheap)
                    for rec in self._btree2_records(bt):
                        k, v = self._attribute(heap.get(rec[:8]))
                        out[k] = v
        return out

    def _attribute(self, body):
        ver = body[0]
        nsz, tsz, ssz = _u(body[2:4]), _u(body[4:6]), _u(body[6:8])
        if ver == 1:
            pad = lambda n: (n + 7) // 8 * 8  # noqa: E731
            p = 8
            name = body[p:p + nsz]
            p += pad(nsz)
            dt = body[p:p + tsz]
            p += pad(tsz)
            ds = body[p:p + ssz]
            p += pad(ssz)
        elif ver in (2, 3):
            p = 9 if ver == 3 else 8
            name = body[p:p + nsz]
            p += nsz
            dt = body[p:p + tsz]
            p += tsz
            ds = body[p:p + ssz]
            p += ssz
        else:
            raise H5Error(f"attribute version {ver}")
        key = name.split(b"\0")[0].decode("utf-8", errors="replace")
        t = _Datatype(dt)
        dims = _dataspace(ds, self.L)
        count = 1
        for x in dims or []:
            count *= x
        if dims is None:
            return key, None
        return key, t.decode(body[p:], count)

    # -------------------------------------------------------------------- datasets
    def dataset(self, addr):
        return Dataset(self, addr)


class _FractalHeap:
    def __init__(self, f, addr):
        d = f.d
        if d[addr:addr + 4] != b"FRHP":
            raise H5Error("fractal heap signature")
        self.f = f
        O, L = f.O, f.L
        p = addr + 5
        self.id_len = _u(d[p:p + 2])
        filt_len = _u(d[p + 2:p + 4])
        self.flags = d[p + 4]
        self.max_obj = _u(d[p + 5:p + 9])
        p += 9
        p += L + O + L + O + L + L + L + L + L + L + L + L     # through "number of tiny objs"
        self.width = _u(d[p:p + 2])
        self.start = _u(d[p + 2:p + 2 + L])
        p += 2 + L
        self.max_direct = _u(d[p:p + L])
        p += L
        self.max_heap_bits = _u(d[p:p + 2])
        p += 4                                     # max heap size + starting # rows
        self.root = _u(d[p:p + O])
        self.root_rows = _u(d[p + O:p + O + 2])
        if filt_len:
            raise H5Error("filtered fractal heaps are not supported")
        self.off_size = (self.max_heap_bits + 7) // 8
        self.len_size = (min(self.max_direct, self.max_obj).bit_length() + 7) // 8
        self.blocks = []                           # (heap offset, size, address)
        self.max_drows = (self.max_direct.bit_length() - 1) - (self.start.bit_length() - 1) + 2
        if self.root_rows == 0:
            self.blocks.append((0, self.start, self.root))
        else:
            self._indirect(self.root, self.root_rows, 0)

    def _row_size(self, r):
        return self.start if r == 0 else self.start << (r - 1)

    def _indirect(self, addr, nrows, heap_off):
        d = self.f.d
        if d[addr:addr + 4] != b"FHIB":
            raise H5Error("fractal heap indirect block signature")
        p = addr + 5 + self.f.O + self.off_size
        off = heap_off
        for r in range(nrows):
            size = self._row_size(r)
            for _ in range(self.width):
                child = _u(d[p:p + self.f.O])
                p += self.f.O
                if r < self.max_drows:
                    if child != self.f.undef:
                        self.blocks.append((off, size, child))
                else:
                    if child != self.f.undef:
                        crows = (size.bit_length() - 1) - \
                                ((self.start * self.width).bit_length() - 1) + 1
                        self._indirect(child, crows, off)
                off += size

    def get(self, heap_id):
        kind = (heap_id[0] >> 4) & 3
        if kind == 2:                               # tiny: the object IS the id
            n = (heap_id[0] & 0x0F) + 1
            return heap_id[1:1 + n]
        if kind != 0:
            raise H5Error("huge fractal-heap objects are not supported")
        off = _u(heap_id[1:1 + self.off_size])
        n = _u(heap_id[1 + self.off_size:1 + self.off_size + self.len_size])
        for boff, bsize, baddr in self.blocks:
            if boff <= off < boff + bsize:
                p = baddr + (off - boff)
                return self.f.d[p:p + n]
        raise H5Error(f"fractal heap offset {off} not in any direct block")


class Dataset:
    def __init__(self, f, addr):
        self.f = f
        self.addr = addr
        self.msgs = f.messages(addr)
        self.shape = None
        self.dtype = None
        self.layout = None
        self.filters = []
        for mtype, body in self.msgs:
            if mtype == 0x01:
                self.shape = _dataspace(body, f.L)
            elif mtype == 0x03:
                self.dtype = _Datatype(body)
            elif mtype == 0x08:
                self.layout = body
            elif mtype == 0x0B:
                self.filters = self._pipeline(body)
        self._attrs = None

    @property
    def attrs(self):
        if self._attrs is None:
            self._attrs = self.f.attributes(self.addr, self.msgs)
        return self._attrs

    @staticmethod
    def _pipeline(body):
        ver = body[0]
        n = body[1]
        p = 8 if ver == 1 else 2
        out = []
        for _ in range(n):
            fid = _u(body[p:p + 2])
            if ver == 1 or fid >= 256:
                nlen = _u(body[p + 2:p + 4])
                p += 4
            else:
                nlen = 0
                p += 2
            flags = _u(body[p:p + 2])
            nvals = _u(body[p + 2:p + 4])
            p += 4
            if nlen:
                p += (nlen + 7) // 8 * 8 if ver == 1 else nlen
            vals = [_u(body[p + 4 * i:p + 4 * i + 4]) for i in range(nvals)]
            p += 4 * nvals
            if ver == 1 and nvals % 2:
                p += 4
            out.append((fid, flags, vals))
        return out

    def _unfilter(self, raw, mask):
        for i in range(len(self.filters) - 1, -1, -1):
            if mask & (1 << i):
                continue
            fid, _, vals = self.filters[i]
            if fid == 1:
                raw = zlib.decompress(raw)
            elif fid == 2:
                es = vals[0] if vals else self.dtype.size
                n = len(raw) // es
                out = bytearray(len(raw))
                for b in range(es):
                    out[b::es] = raw[b * n:(b + 1) * n]
                raw = bytes(out)
            elif fid == 3:
                raw = raw[:-4]
            else:
                raise H5Error(f"filter {fid} not supported (deflate/shuffle/fletcher32 only)")
        return raw

    def read(self):
        """-> (array, shape): the whole variable, row-major."""
        if self.dtype is None or self.dtype.code in (None, "s"):
            raise H5Error("only numeric datasets can be read")
        shape = self.shape or []
        total = 1
        for s in shape:
            total *= s
        es = self.dtype.size
        body = self.layout
        ver = body[0]
        if ver != 3:
            raise H5Error(f"data layout message version {ver} not supported")
        cls = body[1]
        f = self.f
        if cls == 0:                                    # compact
            n = _u(body[2:4])
            raw = body[4:4 + n]
        elif cls == 1:                                  # contiguous
            a = _u(body[2:2 + f.O])
            if a == f.undef:
                raw = b"\0" * (total * es)
            else:
                raw = f.d[a:a + total * es]
        elif cls == 2:                                  # chunked, v1 B-tree
            nd = body[2]
            bt = _u(body[3:3 + f.O])
            p = 3 + f.O
            cdims = [_u(body[p + 4 * i:p + 4 * i + 4]) for i in range(nd)]
            raw = self._chunked(bt, cdims[:-1], shape, total, es)
        else:
            raise H5Error(f"layout class {cls}")
        out = array(self.dtype.code)
        out.frombytes(bytes(raw[:total * es]))
        if self.dtype.big:
            out.byteswap()
        return out, shape

    def _chunked(self, bt, cdims, shape, total, es):
        f = self.f
        d = f.d
        rank = len(shape)
        out = bytearray(total * es)
        if bt == f.undef:
            return out
        chunk_elems = 1
        for c in cdims:
            chunk_elems *= c
        strides = [1] * rank
        for i in range(rank - 2, -1, -1):
            strides[i] = strides[i + 1] * shape[i + 1]
        cstrides = [1] * rank
        for i in range(rank - 2, -1, -1):
            cstrides[i] = cstrides[i + 1] * cdims[i + 1]
        key_size = 8 + 8 * (rank + 1)

        def walk(node):
            if d[node:node + 4] != b"TREE" or d[node + 4] != 1:
                raise H5Error("chunk B-tree node")
            level = d[node + 5]
            used = _u(d[node + 6:node + 8])
            p = node + 8 + 2 * f.O
            for _ in range(used):
                csize = _u(d[p:p + 4])
                mask = _u(d[p + 4:p + 8])
                offs = [_u(d[p + 8 + 8 * i:p + 16 + 8 * i]) for i in range(rank)]
                child = f.addr(p + key_size)
                p += key_size + f.O
                if level > 0:
                    walk(child)
                else:
                    data = self._unfilter(d[child:child + csize], mask)
                    place(offs, data)

        def place(offs, data):
            # copy the chunk's rows (runs along the last dimension) into the output
            ext = [min(cdims[i], shape[i] - offs[i]) for i in range(rank)]
            run = ext[-1] * es
            idx = [0] * (rank - 1)
            while True:
                src = sum(idx[i] * cstrides[i] for i in range(rank - 1)) * es
                dst = (sum((offs[i] + idx[i]) * strides[i] for i in range(rank - 1))
                       + offs[-1]) * es
                out[dst:dst + run] = data[src:src + run]
                k = rank - 2
                while k >= 0:
                    idx[k] += 1
                    if idx[k] < ext[k]:
                        break
                    idx[k] = 0
                    k -= 1
                if k < 0:
                    break

        walk(bt)
        return out


def open_netcdf4(path):
    """-> (H5File, {name: Dataset}, global attributes) for a netCDF-4 file's root group."""
    f = H5File(path)
    links = f.links(f.root_addr)
    return f, {k: Dataset(f, a) for k, a in links.items()}, f.attributes(f.root_addr)
