"""Minimal GeoTIFF reader, stdlib only, for NOAA CUDEM topobathy tiles.

Supports what those files actually use: classic TIFF (little/big endian), strip or tile layout,
float32 samples, compression none/LZW/Deflate, predictor none/horizontal/floating-point, plus the
GeoTIFF georeferencing tags (ModelPixelScale + ModelTiepoint) and GDAL's nodata tag. WINDOWED
reads decode only the strips/tiles intersecting the requested pixel window -- a full 8100^2 tile
never needs to be decompressed to crop out the river mouth.
"""

import struct
import zlib
from array import array

TAG_WIDTH, TAG_HEIGHT = 256, 257
TAG_BPS, TAG_COMPRESSION = 258, 259
TAG_STRIP_OFFSETS, TAG_ROWS_PER_STRIP, TAG_STRIP_COUNTS = 273, 278, 279
TAG_PLANAR = 284
TAG_PREDICTOR = 317
TAG_TILE_W, TAG_TILE_H, TAG_TILE_OFFSETS, TAG_TILE_COUNTS = 322, 323, 324, 325
TAG_SAMPLE_FORMAT = 339
TAG_PIXEL_SCALE, TAG_TIEPOINT = 33550, 33922
TAG_GDAL_NODATA = 42113

_TYPE_SIZES = {1: 1, 2: 1, 3: 2, 4: 4, 5: 8, 6: 1, 7: 1, 8: 2, 9: 4, 10: 8, 11: 4, 12: 8}


def _lzw_decode(data):
    """TIFF-variant LZW: MSB-first bit packing, codes 9..12 bits, early change."""
    CLEAR, EOI = 256, 257
    out = bytearray()
    table = [bytes([i]) for i in range(256)] + [b"", b""]
    bitbuf = 0
    bitcnt = 0
    pos = 0
    width = 9
    prev = None
    n = len(data)
    while pos < n or bitcnt >= width:
        while bitcnt < width and pos < n:
            bitbuf = (bitbuf << 8) | data[pos]
            pos += 1
            bitcnt += 8
        if bitcnt < width:
            break
        code = (bitbuf >> (bitcnt - width)) & ((1 << width) - 1)
        bitcnt -= width
        if code == CLEAR:
            table = table[:258]
            width = 9
            prev = None
            continue
        if code == EOI:
            break
        if prev is None:
            entry = table[code]
        elif code < len(table):
            entry = table[code]
            table.append(prev + entry[:1])
        else:
            entry = prev + prev[:1]
            table.append(entry)
        out += entry
        prev = entry
        # "Early change": width bumps one code earlier than the table strictly requires.
        if len(table) + 1 >= (1 << width) and width < 12:
            width += 1
    return bytes(out)


def _undo_predictor(raw, w, h, bps, sample_format, predictor):
    if predictor in (0, 1):
        return raw
    bpp = bps // 8
    row_bytes = w * bpp
    out = bytearray(raw)
    if predictor == 2:
        # Horizontal differencing on integer samples.
        for r in range(h):
            base = r * row_bytes
            for x in range(1, w):
                for b in range(bpp):
                    out[base + x * bpp + b] = (out[base + x * bpp + b] +
                                               out[base + (x - 1) * bpp + b]) & 0xFF
        return bytes(out)
    if predictor == 3:
        # Floating-point predictor (TIFF TechNote 3): each row stores byte-plane-separated,
        # horizontally differenced bytes. Undo the byte deltas, then re-interleave planes,
        # then reverse the plane order (it is stored big-endian-ish, MSB plane first).
        res = bytearray(len(raw))
        for r in range(h):
            base = r * row_bytes
            row = bytearray(raw[base:base + row_bytes])
            for i in range(1, row_bytes):
                row[i] = (row[i] + row[i - 1]) & 0xFF
            for x in range(w):
                for b in range(bpp):
                    res[base + x * bpp + (bpp - 1 - b)] = row[b * w + x]
        return bytes(res)
    raise ValueError(f"predictor {predictor} not supported")


class GeoTiff:
    def __init__(self, data: bytes):
        self.data = data
        bo = data[:2]
        if bo == b"II":
            self.e = "<"
        elif bo == b"MM":
            self.e = ">"
        else:
            raise ValueError("not a TIFF")
        magic, ifd_off = struct.unpack_from(self.e + "HI", data, 2)
        if magic != 42:
            raise ValueError("BigTIFF not supported")
        self.tags = {}
        self._read_ifd(ifd_off)

        self.width = self.tag1(TAG_WIDTH)
        self.height = self.tag1(TAG_HEIGHT)
        self.bps = self.tag1(TAG_BPS, 32)
        self.compression = self.tag1(TAG_COMPRESSION, 1)
        self.predictor = self.tag1(TAG_PREDICTOR, 1)
        self.sample_format = self.tag1(TAG_SAMPLE_FORMAT, 3)
        self.tiled = TAG_TILE_OFFSETS in self.tags
        if self.tiled:
            self.tw = self.tag1(TAG_TILE_W)
            self.th = self.tag1(TAG_TILE_H)
            self.offsets = self.tag(TAG_TILE_OFFSETS)
            self.counts = self.tag(TAG_TILE_COUNTS)
            self.tiles_x = (self.width + self.tw - 1) // self.tw
        else:
            self.tw = self.width
            self.th = self.tag1(TAG_ROWS_PER_STRIP, self.height)
            self.offsets = self.tag(TAG_STRIP_OFFSETS)
            self.counts = self.tag(TAG_STRIP_COUNTS)
            self.tiles_x = 1

        scale = self.tag(TAG_PIXEL_SCALE)
        tie = self.tag(TAG_TIEPOINT)
        self.sx = scale[0] if scale else 1.0
        self.sy = scale[1] if scale else 1.0
        # Tiepoint: raster (i, j, k) -> model (x, y, z); CUDEM anchors pixel (0,0) at the NW corner.
        self.lon0 = tie[3] if tie else 0.0
        self.lat0 = tie[4] if tie else 0.0
        nod = self.tag(TAG_GDAL_NODATA)
        try:
            self.nodata = float(nod.strip("\x00 ").strip()) if isinstance(nod, str) else None
        except (ValueError, AttributeError):
            self.nodata = None

    # ---- tag plumbing
    def _read_ifd(self, off):
        e = self.e
        (count,) = struct.unpack_from(e + "H", self.data, off)
        p = off + 2
        for _ in range(count):
            tag, typ, n, val_off = struct.unpack_from(e + "HHII", self.data, p)
            size = _TYPE_SIZES.get(typ, 1) * n
            if size <= 4:
                raw = self.data[p + 8:p + 8 + size]
            else:
                raw = self.data[val_off:val_off + size]
            self.tags[tag] = (typ, n, raw)
            p += 12

    def tag(self, tag, default=None):
        if tag not in self.tags:
            return default
        typ, n, raw = self.tags[tag]
        e = self.e
        if typ == 2:
            return raw.decode("ascii", errors="replace")
        fmt = {1: "B", 3: "H", 4: "I", 8: "h", 9: "i", 11: "f", 12: "d"}.get(typ)
        if fmt is None:
            if typ == 5:   # rational
                vals = struct.unpack_from(e + f"{2 * n}I", raw)
                return [vals[i] / max(vals[i + 1], 1) for i in range(0, 2 * n, 2)]
            return raw
        vals = list(struct.unpack_from(e + f"{n}{fmt}", raw))
        return vals

    def tag1(self, tag, default=None):
        v = self.tag(tag)
        return v[0] if isinstance(v, list) and v else (default if v is None else v)

    # ---- pixel access
    def _decode_block(self, idx, w, h):
        raw = self.data[self.offsets[idx]:self.offsets[idx] + self.counts[idx]]
        if self.compression == 1:
            dec = raw
        elif self.compression == 5:
            dec = _lzw_decode(raw)
        elif self.compression in (8, 32946):
            dec = zlib.decompress(raw)
        else:
            raise ValueError(f"compression {self.compression} not supported")
        dec = _undo_predictor(dec, w, h, self.bps, self.sample_format, self.predictor)
        vals = array("f")
        vals.frombytes(dec[:w * h * 4])
        if self.e == ">":
            vals.byteswap()
        return vals

    def read_window(self, x0, y0, w, h):
        """float32 window, row-major, NaN where nodata. Decodes only intersecting blocks."""
        out = array("f", [float("nan")]) * (w * h)
        bx0, bx1 = x0 // self.tw, (x0 + w - 1) // self.tw
        by0, by1 = y0 // self.th, (y0 + h - 1) // self.th
        for by in range(by0, by1 + 1):
            for bx in range(bx0, bx1 + 1):
                if self.tiled:
                    idx = by * self.tiles_x + bx
                    bw, bh = self.tw, self.th
                else:
                    idx = by
                    bw = self.width
                    bh = min(self.th, self.height - by * self.th)
                if idx >= len(self.offsets):
                    continue
                block = self._decode_block(idx, bw, bh)
                ox = bx * self.tw
                oy = by * self.th
                fx0, fx1 = max(x0, ox), min(x0 + w, ox + bw)
                fy0, fy1 = max(y0, oy), min(y0 + h, oy + bh)
                for yy in range(fy0, fy1):
                    src = (yy - oy) * bw + (fx0 - ox)
                    dst = (yy - y0) * w + (fx0 - x0)
                    out[dst:dst + (fx1 - fx0)] = block[src:src + (fx1 - fx0)]
        if self.nodata is not None:
            for i, v in enumerate(out):
                if v == self.nodata or v < -99999.0 or v > 99999.0:
                    out[i] = float("nan")
        return out

    # ---- georef
    def px_of_lon(self, lon):
        return int((lon - self.lon0) / self.sx)

    def py_of_lat(self, lat):
        return int((self.lat0 - lat) / self.sy)   # row 0 = northern edge
