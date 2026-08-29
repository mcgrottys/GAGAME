"""Minimal classic NetCDF (v1/v2) reader, stdlib only.

Written for GAGAME's current harvester: THREDDS NetcdfSubset returns netCDF-3 classic when asked
for accept=netcdf, and that format is a straightforward big-endian binary layout -- unlike
netCDF-4/HDF5, which is out of scope on purpose. Supports dimensions, attributes (enough for
scale_factor / add_offset / _FillValue), fixed and record variables. Raises loudly on anything
else.
"""

import struct
from array import array

NC_BYTE, NC_CHAR, NC_SHORT, NC_INT, NC_FLOAT, NC_DOUBLE = 1, 2, 3, 4, 5, 6
_SIZES = {1: 1, 2: 1, 3: 2, 4: 4, 5: 4, 6: 8}
_FMTS = {1: "b", 3: "h", 4: "i", 5: "f", 6: "d"}


class Var:
    def __init__(self, name, dims, shape, nc_type, vsize, begin, atts, is_record):
        self.name = name
        self.dims = dims          # dim names
        self.shape = shape        # per-record shape has record dim first with its length
        self.nc_type = nc_type
        self.vsize = vsize
        self.begin = begin
        self.atts = atts
        self.is_record = is_record


class NetCDF3:
    def __init__(self, data: bytes):
        self.data = data
        if data[:3] != b"CDF" or data[3] not in (1, 2):
            raise ValueError("not a classic NetCDF file (netCDF-4/HDF5 is not supported here)")
        self.version = data[3]
        self.pos = 4
        self.numrecs = self._u32()
        self.dim_names, self.dim_sizes = [], []
        self._read_dims()
        self.gatts = self._read_atts()
        self.vars = {}
        self._read_vars()

    # ---- primitive readers
    def _u32(self):
        v = struct.unpack_from(">I", self.data, self.pos)[0]
        self.pos += 4
        return v

    def _u64(self):
        v = struct.unpack_from(">Q", self.data, self.pos)[0]
        self.pos += 8
        return v

    def _name(self):
        n = self._u32()
        s = self.data[self.pos:self.pos + n].decode("utf-8", errors="replace")
        self.pos += (n + 3) // 4 * 4
        return s

    # ---- header sections
    def _read_dims(self):
        tag = self._u32()
        count = self._u32()
        if tag == 0 and count == 0:
            return
        if tag != 0x0A:
            raise ValueError(f"bad dim tag {tag:#x}")
        for _ in range(count):
            self.dim_names.append(self._name())
            self.dim_sizes.append(self._u32())

    def _read_atts(self):
        tag = self._u32()
        count = self._u32()
        if tag == 0 and count == 0:
            return {}
        if tag != 0x0C:
            raise ValueError(f"bad att tag {tag:#x}")
        atts = {}
        for _ in range(count):
            name = self._name()
            t = self._u32()
            n = self._u32()
            size = _SIZES[t] * n
            raw = self.data[self.pos:self.pos + size]
            self.pos += (size + 3) // 4 * 4
            if t == NC_CHAR:
                atts[name] = raw.decode("utf-8", errors="replace").rstrip("\x00")
            else:
                vals = struct.unpack(f">{n}{_FMTS[t]}", raw)
                atts[name] = vals[0] if n == 1 else list(vals)
        return atts

    def _read_vars(self):
        tag = self._u32()
        count = self._u32()
        if tag == 0 and count == 0:
            return
        if tag != 0x0B:
            raise ValueError(f"bad var tag {tag:#x}")
        for _ in range(count):
            name = self._name()
            nd = self._u32()
            dimids = [self._u32() for _ in range(nd)]
            atts = self._read_atts()
            t = self._u32()
            vsize = self._u32()
            begin = self._u64() if self.version == 2 else self._u32()
            dims = [self.dim_names[d] for d in dimids]
            shape = [self.dim_sizes[d] for d in dimids]
            is_record = bool(shape) and shape[0] == 0
            if is_record:
                shape = [self.numrecs] + shape[1:]
            self.vars[name] = Var(name, dims, shape, t, vsize, begin, atts, is_record)
        # Record stride: sum of vsize over record variables (vsize already 4-byte padded).
        self.record_stride = sum(v.vsize for v in self.vars.values() if v.is_record)

    # ---- data access
    def read(self, name):
        """Returns (flat array('f' or 'd' ...), shape) with scale/offset/fill applied to floats."""
        v = self.vars[name]
        t = v.nc_type
        if t == NC_CHAR:
            raise ValueError("char variables not supported")
        esize = _SIZES[t]
        fmt = _FMTS[t]
        per_record = 1
        for s in v.shape[1:] if v.is_record else v.shape:
            per_record *= s
        out = array("d")
        if v.is_record:
            for r in range(v.shape[0]):
                off = v.begin + r * self.record_stride
                out.extend(struct.unpack_from(f">{per_record}{fmt}",
                                              self.data, off))
        else:
            n = per_record
            out.extend(struct.unpack_from(f">{n}{fmt}", self.data, v.begin))

        scale = v.atts.get("scale_factor", 1.0)
        offset = v.atts.get("add_offset", 0.0)
        fill = v.atts.get("_FillValue", v.atts.get("missing_value"))
        if scale != 1.0 or offset != 0.0 or fill is not None:
            res = array("d")
            for x in out:
                if fill is not None and x == fill:
                    res.append(float("nan"))
                else:
                    res.append(x * scale + offset)
            out = res
        return out, v.shape
