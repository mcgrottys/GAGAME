"""OSM land polygons (osmdata.openstreetmap.de land-polygons-split-4326, made from OSM's
natural=coastline by osmcoastline) into GALAND01: the global land/sea mask's rings, indexed by
1-degree cell so the engine pages them in by the tiles it sweeps (compose/GisMask.h).

    py -3 -I harvester/harvest_land_polygons.py <land_polygons.shp> <out.galand>

The split file's polygons are single rings (no holes), each within about 1.1 degrees, so the
mask's parity is taken per ring. A ring is filed in the cell of its box's centre; its own box rides
beside it, so a reader takes the cells around a tile and keeps the rings whose boxes meet it.

GALAND01 (little-endian):
    magic "GALAND01", u32 cells
    cells x {i16 cx, i16 cy, u32 rings, u64 offset}      sorted by (cy, cx); offset from file start
    per ring: f32 lon0, lat0, lon1, lat1; u32 n; n x (f32 lon, f32 lat)
"""
import math
import struct
import sys
from collections import defaultdict

import numpy as np


def main(shp, out):
    cells = defaultdict(list)   # (cy, cx) -> [(file offset of the record's points, n, box)]
    with open(shp, 'rb') as f:
        f.seek(100)
        n = 0
        while True:
            h = f.read(8)
            if len(h) < 8:
                break
            _, clen = struct.unpack('>ii', h)
            start = f.tell()
            head = f.read(44)
            st = struct.unpack('<i', head[:4])[0]
            if st == 5:
                x0, y0, x1, y1 = struct.unpack('<4d', head[4:36])
                nparts, npts = struct.unpack('<ii', head[36:44])
                pts_at = start + 44 + 4 * nparts
                cx = math.floor(0.5 * (x0 + x1))
                cy = math.floor(0.5 * (y0 + y1))
                cells[(cy, cx)].append((pts_at, npts, (x0, y0, x1, y1)))
                n += 1
            f.seek(start + clen * 2)
    keys = sorted(cells)
    print(f'{n} rings in {len(keys)} cells', flush=True)
    with open(shp, 'rb') as f, open(out, 'wb') as o:
        o.write(b'GALAND01')
        o.write(struct.pack('<I', len(keys)))
        index_at = o.tell()
        o.write(b'\0' * (16 * len(keys)))
        index = []
        pts_total = 0
        for k, (cy, cx) in enumerate(keys):
            index.append((cx, cy, len(cells[(cy, cx)]), o.tell()))
            for pts_at, npts, box in cells[(cy, cx)]:
                f.seek(pts_at)
                xy = np.frombuffer(f.read(16 * npts), dtype='<f8').astype('<f4')
                o.write(struct.pack('<4fI', *box, npts))
                o.write(xy.tobytes())
                pts_total += npts
            if k % 5000 == 0:
                print(f'  {k} of {len(keys)} cells, {pts_total} points', flush=True)
        o.seek(index_at)
        for cx, cy, rings, off in index:
            o.write(struct.pack('<hhIQ', cx, cy, rings, off))
    print(f'wrote {out}: {n} rings, {pts_total} points', flush=True)


if __name__ == '__main__':
    main(sys.argv[1], sys.argv[2])
