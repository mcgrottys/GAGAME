"""Minimal GRIB2 reader, stdlib only. Scope: exactly what NCEP's grib-filter subsets need --
edition 2, grid template 3.0 (regular lat/lon), data representation templates 5.0 (simple),
5.2 (complex) and 5.3 (complex + spatial differencing), section-6 bitmaps. Anything else raises
loudly rather than guessing.

Written for GAGAME after NOMADS retired OPeNDAP (SCN 25-81): the grib filter is now the one
sanctioned small-subset path, and it speaks GRIB2 only. This decoder is deliberately tiny and
validated end-to-end against NDBC buoy observations by the caller.
"""

import struct


def _u(b):
    return int.from_bytes(b, "big")


def _sm(b):
    """GRIB signed-magnitude integer: high bit is the sign, NOT two's complement."""
    v = int.from_bytes(b, "big")
    sign = v >> (len(b) * 8 - 1)
    mag = v & ((1 << (len(b) * 8 - 1)) - 1)
    return -mag if sign else mag


class _Bits:
    def __init__(self, data, byte_off=0):
        self.data = data
        self.pos = byte_off * 8

    def read(self, nbits):
        if nbits == 0:
            return 0
        end = self.pos + nbits
        byte0, byte1 = self.pos // 8, (end + 7) // 8
        chunk = _u(self.data[byte0:byte1])
        shift = byte1 * 8 - end
        self.pos = end
        return (chunk >> shift) & ((1 << nbits) - 1)

    def align(self):
        self.pos = (self.pos + 7) // 8 * 8


class Grib2Message:
    def __init__(self):
        self.discipline = 0
        self.param_category = 0
        self.param_number = 0
        self.level_type = 0
        self.level_value = 0
        self.forecast_hour = 0
        self.ni = 0
        self.nj = 0
        self.lat1 = 0.0     # first grid point (degrees)
        self.lon1 = 0.0
        self.dlat = 0.0     # positive step magnitudes
        self.dlon = 0.0
        self.scan = 0
        self.values = []    # row-major as stored; None where the bitmap masks

    def key(self):
        return (self.discipline, self.param_category, self.param_number, self.level_type,
                self.level_value)

    def at_latlon(self, lat, lon_e):
        """Nearest-node value. lon_e in [0, 360)."""
        lat_rows = [self.lat1 + r * (self.dlat if (self.scan & 0x40) else -self.dlat)
                    for r in range(self.nj)]
        lon_cols = [(self.lon1 + c * (-self.dlon if (self.scan & 0x80) else self.dlon)) % 360.0
                    for c in range(self.ni)]
        r = min(range(self.nj), key=lambda i: abs(lat_rows[i] - lat))
        c = min(range(self.ni), key=lambda i: min(abs(lon_cols[i] - lon_e),
                                                  360.0 - abs(lon_cols[i] - lon_e)))
        return self.values[r * self.ni + c]


def _decode_data(sec5, sec6, sec7, n_points):
    tmpl = _u(sec5[9:11])
    n_values = _u(sec5[5:9])
    ref = struct.unpack(">f", sec5[11:15])[0]
    bin_scale = _sm(sec5[15:17])
    dec_scale = _sm(sec5[17:19])
    nbits = sec5[19]
    two_e = 2.0 ** bin_scale
    ten_d = 10.0 ** (-dec_scale)

    if tmpl == 0:
        bits = _Bits(sec7, 5)
        raw = [bits.read(nbits) for _ in range(n_values)]
        vals = [(ref + x * two_e) * ten_d for x in raw]
    elif tmpl in (2, 3):
        if sec5[22] != 0:
            raise ValueError(f"missing-value management {sec5[22]} not supported")
        ng = _u(sec5[31:35])
        ref_gw = sec5[35]
        nb_gw = sec5[36]
        ref_gl = _u(sec5[37:41])
        len_inc = sec5[41]
        last_len = _u(sec5[42:46])
        nb_sgl = sec5[46]

        bits = _Bits(sec7, 5)
        order = 0
        extra = []
        gmin = 0
        if tmpl == 3:
            order = sec5[47]
            nbytes_extra = sec5[48]
            if order not in (1, 2):
                raise ValueError(f"spatial differencing order {order} not supported")
            for _ in range(order):
                extra.append(_sm(sec7[bits.pos // 8: bits.pos // 8 + nbytes_extra]))
                bits.pos += nbytes_extra * 8
            gmin = _sm(sec7[bits.pos // 8: bits.pos // 8 + nbytes_extra])
            bits.pos += nbytes_extra * 8

        grefs = [bits.read(nbits) for _ in range(ng)]
        bits.align()
        gwidths = [ref_gw + bits.read(nb_gw) for _ in range(ng)]
        bits.align()
        glens = [ref_gl + len_inc * bits.read(nb_sgl) for _ in range(ng)]
        bits.align()
        if ng:
            glens[-1] = last_len

        raw = []
        for g in range(ng):
            w = gwidths[g]
            r0 = grefs[g]
            for _ in range(glens[g]):
                raw.append(r0 + (bits.read(w) if w else 0))
        raw = raw[:n_values]

        if tmpl == 3:
            # Undo the spatial differencing: the first `order` ORIGINAL values arrived in the
            # extra descriptors; every later entry is a difference biased by gmin.
            x = list(raw)
            for i in range(order, len(x)):
                x[i] += gmin
            for i in range(order):
                if i < len(x):
                    x[i] = extra[i]
            if order == 1:
                for i in range(1, len(x)):
                    x[i] += x[i - 1]
            else:
                for i in range(2, len(x)):
                    x[i] += 2 * x[i - 1] - x[i - 2]
            raw = x
        vals = [(ref + x * two_e) * ten_d for x in raw]
    else:
        raise ValueError(f"data representation template 5.{tmpl} not supported "
                         "(only 5.0 / 5.2 / 5.3)")

    # Bitmap: indicator 0 = bitmap in this message, 255 = none, 254 = reuse (unsupported).
    ind = sec6[5] if sec6 is not None else 255
    if ind == 255:
        if len(vals) < n_points:
            vals += [None] * (n_points - len(vals))
        return vals[:n_points]
    if ind != 0:
        raise ValueError(f"bitmap indicator {ind} not supported")
    bits = _Bits(sec6, 6)
    out = []
    it = iter(vals)
    for _ in range(n_points):
        out.append(next(it, None) if bits.read(1) else None)
    return out


def read_messages(data):
    """Parse every GRIB2 message in a byte string."""
    msgs = []
    pos = 0
    while pos + 16 <= len(data):
        idx = data.find(b"GRIB", pos)
        if idx < 0:
            break
        if data[idx + 7] != 2:
            raise ValueError(f"GRIB edition {data[idx + 7]} not supported")
        total = _u(data[idx + 8:idx + 16])
        body = data[idx:idx + total]
        if body[-4:] != b"7777":
            raise ValueError("message does not end in 7777")

        m = Grib2Message()
        m.discipline = body[6]
        p = 16
        secs = {}
        while p < total - 4:
            slen = _u(body[p:p + 4])
            snum = body[p + 4]
            secs[snum] = body[p:p + slen]
            if snum == 7:
                # Sections 4..7 repeat per field; grib-filter output is one field per message,
                # so decode as soon as the data section closes.
                s3, s4, s5 = secs[3], secs[4], secs[5]
                if _u(s3[12:14]) != 0:
                    raise ValueError(f"grid template 3.{_u(s3[12:14])} not supported")
                m.ni = _u(s3[30:34])
                m.nj = _u(s3[34:38])
                m.lat1 = _sm(s3[46:50]) / 1e6
                m.lon1 = _sm(s3[50:54]) / 1e6
                m.lat2 = _sm(s3[55:59]) / 1e6
                m.dlon = _u(s3[63:67]) / 1e6
                m.dlat = _u(s3[67:71]) / 1e6
                m.scan = s3[71]
                # 4.0/4.1 = instantaneous; 4.8 = statistically processed (time-averaged --
                # GFS layer clouds, fluxes). 4.8 extends 4.0, so the fields we read sit at
                # the same offsets; the trailing time-interval block is irrelevant here.
                if _u(s4[7:9]) not in (0, 1, 8):
                    raise ValueError(f"product template 4.{_u(s4[7:9])} not supported")
                m.param_category = s4[9]
                m.param_number = s4[10]
                m.forecast_hour = _u(s4[18:22])
                m.level_type = s4[22]
                sv = _u(s4[24:28])
                m.level_value = sv // (10 ** s4[23]) if s4[23] else sv
                m.values = _decode_data(s5, secs.get(6), secs[7], m.ni * m.nj)
                msgs.append(m)
                m = Grib2Message()
                m.discipline = body[6]
            p += slen
        pos = idx + total
    return msgs
