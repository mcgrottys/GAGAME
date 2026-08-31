# ==================================================================================================
#  harvest_vectors.py - M6p: THE SURVEY ORDER. Vector files (SHP, GeoJSON, the GSHHG bins) as
#  first-class monastery manuscripts: LOSSLESS storage, queryable by LOD.
#
#  The trick that answers "lossless + LOD + decimation": every vertex is kept forever, but
#  each carries the Visvalingam-Whyatt EFFECTIVE AREA at which decimation would remove it --
#  and that area is the WEDGE PRODUCT, |(B-A) ^ (C-A)| / 2, the bivector magnitude of the
#  vertex's triangle (the algebra IS the decimation metric). Querying at tolerance t is then
#  a filter: keep vertices whose importance >= t. One structure, every LOD, no duplication.
#  Importance is stored as sqrt(area) in METERS so filters read as "meters of detail".
#
#  Inputs:
#    data/gis/*.bin            the GSHHG coast/river polylines already harvested (converted)
#    cache/vectors/*.shp(+dbf) any ESRI shapefile (polyline/polygon/point, Z/M tolerated)
#    cache/vectors/*.geojson   any GeoJSON (Feature/FeatureCollection)
#    data/gis/*.kml, cache/vectors/*.kml   any KML (Placemark LineString/Polygon/Point)
#
#  Output: data/vectors/vectors.vpack + vectors.json (attribute sidecars per layer).
#  Format VPK1 (little-endian):
#    u32 'VPK1', u32 layerCount
#    per layer: u16 nameLen, name utf8, u8 kind (0 polyline, 1 polygon, 2 points),
#               u8 flags (1 = has z, 2 = has t), u32 polyCount
#      per poly: u32 vertCount, verts: f32 lon, f32 lat, f32 importance_m [, f32 z][, f32 t]
#
#  Also authors data/gis/edits.geojson (if absent): the HAND-EDITABLE mask override layer.
#  The default content marks the Merrimack jetties as LAND -- the survey shoreline predates
#  them and the height classifier smears their thin ridges, so they dropped into the ocean.
#  Edit the polygons in any text editor or GIS tool; the engine re-rasterizes on next run.
# ==================================================================================================
import glob
import heapq
import json
import math
import os
import struct

GIS = os.path.join("data", "gis")
CACHE = os.path.join("cache", "vectors")
OUT = os.path.join("data", "vectors")


# ---------------------------------------------------------------- importance (the wedge)

def vw_importance(pts):
    """Per-vertex Visvalingam-Whyatt effective area, monotone (a vertex's importance is at
    least that of every vertex removed before it), in local meters. Endpoints: +inf."""
    n = len(pts)
    imp = [float("inf")] * n
    if n < 3:
        return imp
    lat0 = sum(p[1] for p in pts) / n
    kx = 111320.0 * max(math.cos(math.radians(lat0)), 0.05)
    ky = 110574.0

    def area(i, j, k):
        ax, ay = (pts[j][0] - pts[i][0]) * kx, (pts[j][1] - pts[i][1]) * ky
        bx, by = (pts[k][0] - pts[i][0]) * kx, (pts[k][1] - pts[i][1]) * ky
        return abs(ax * by - ay * bx) * 0.5   # |wedge| / 2 -- the bivector magnitude

    prev = list(range(-1, n - 1))
    nxt = list(range(1, n + 1))
    ver = [0] * n
    heap = [(area(i - 1, i, i + 1), i, 0) for i in range(1, n - 1)]
    heapq.heapify(heap)
    floor = 0.0
    while heap:
        a, i, v = heapq.heappop(heap)
        if v != ver[i]:
            continue   # stale heap entry; a neighbour's removal re-queued this vertex
        floor = max(floor, a)          # monotonicity: never less important than the removed
        imp[i] = floor
        p, q = prev[i], nxt[i]
        prev[q], nxt[p] = p, q
        for j in (p, q):
            if 0 < j < n - 1:
                ver[j] += 1
                heapq.heappush(heap, (area(prev[j], j, nxt[j]), j, ver[j]))
    return imp


def to_meters(imp):
    return [1.0e9 if math.isinf(a) else math.sqrt(a) for a in imp]


# ---------------------------------------------------------------- readers

def read_gshhg_bin(path):
    out = []
    with open(path, "rb") as f:
        (n,) = struct.unpack("<I", f.read(4))
        for _ in range(n):
            (c,) = struct.unpack("<I", f.read(4))
            raw = struct.unpack(f"<{2 * c}f", f.read(8 * c))
            out.append([(raw[2 * i], raw[2 * i + 1]) for i in range(c)])
    return out


def read_shp(path):
    """Types 1/8 (points), 3/13/23 (polyline), 5/15/25 (polygon); Z/M ignored beyond parts."""
    lines, points, kind = [], [], 0
    with open(path, "rb") as f:
        data = f.read()
    if struct.unpack(">i", data[:4])[0] != 9994:
        return None
    shape_type = struct.unpack("<i", data[32:36])[0]
    off = 100
    while off + 8 <= len(data):
        clen = struct.unpack(">i", data[off + 4:off + 8])[0] * 2
        rec = data[off + 8:off + 8 + clen]
        off += 8 + clen
        if len(rec) < 4:
            break
        st = struct.unpack("<i", rec[:4])[0]
        if st in (1, 11, 21):
            x, y = struct.unpack("<2d", rec[4:20])
            points.append((x, y))
        elif st in (8, 18, 28):
            (npt,) = struct.unpack("<i", rec[36:40])
            for i in range(npt):
                x, y = struct.unpack("<2d", rec[40 + 16 * i:56 + 16 * i])
                points.append((x, y))
        elif st in (3, 5, 13, 15, 23, 25):
            nparts, npts = struct.unpack("<2i", rec[36:44])
            parts = struct.unpack(f"<{nparts}i", rec[44:44 + 4 * nparts])
            base = 44 + 4 * nparts
            pts = [struct.unpack("<2d", rec[base + 16 * i:base + 16 * i + 16])
                   for i in range(npts)]
            for pi in range(nparts):
                s = parts[pi]
                e = parts[pi + 1] if pi + 1 < nparts else npts
                if e - s >= 2:
                    lines.append(pts[s:e])
            kind = 1 if st in (5, 15, 25) else 0
    if points and not lines:
        return 2, [[p] for p in points]
    return kind, lines


def read_dbf(path):
    if not os.path.exists(path):
        return []
    with open(path, "rb") as f:
        data = f.read()
    nrec = struct.unpack("<I", data[4:8])[0]
    hsz, rsz = struct.unpack("<2H", data[8:12])
    fields = []
    p = 32
    while p < hsz - 1 and data[p] != 0x0D:
        name = data[p:p + 11].split(b"\0")[0].decode("ascii", "replace")
        fields.append((name, data[p + 16]))
        p += 32
    recs = []
    for r in range(min(nrec, 100000)):
        row = data[hsz + r * rsz:hsz + (r + 1) * rsz]
        if not row or row[0:1] == b"*":
            continue
        vals, q = {}, 1
        for name, flen in fields:
            vals[name] = row[q:q + flen].decode("ascii", "replace").strip()
            q += flen
        recs.append(vals)
    return recs


def read_geojson(path):
    gj = json.load(open(path, encoding="utf8"))
    feats = gj["features"] if gj.get("type") == "FeatureCollection" else [gj]
    lines, points, kind, attrs = [], [], 0, []
    for ft in feats:
        g = ft.get("geometry", ft)
        t, c = g.get("type"), g.get("coordinates")
        if t == "Point":
            points.append(tuple(c[:2]))
        elif t == "MultiPoint":
            points += [tuple(p[:2]) for p in c]
        elif t == "LineString":
            lines.append([tuple(p[:2]) for p in c])
        elif t == "MultiLineString":
            lines += [[tuple(p[:2]) for p in part] for part in c]
        elif t == "Polygon":
            kind = 1
            lines += [[tuple(p[:2]) for p in ring] for ring in c]
        elif t == "MultiPolygon":
            kind = 1
            for poly in c:
                lines += [[tuple(p[:2]) for p in ring] for ring in poly]
        attrs.append(ft.get("properties", {}))
    if points and not lines:
        return 2, [[p] for p in points], attrs
    return kind, lines, attrs


def read_kml(path):
    """Placemark LineString/LinearRing/Polygon/Point; name + description become attrs.
    Namespaces vary across exporters, so match tags by local name."""
    import xml.etree.ElementTree as ET
    root = ET.parse(path).getroot()

    def local(el):
        return el.tag.rsplit("}", 1)[-1]

    def parse_coords(el):
        pts = []
        for tok in (el.text or "").split():
            parts = tok.split(",")
            if len(parts) >= 2:
                pts.append((float(parts[0]), float(parts[1])))
        return pts

    lines, points, kind, attrs = [], [], 0, []
    for pm in root.iter():
        if local(pm) != "Placemark":
            continue
        a = {}
        for ch in pm:
            if local(ch) in ("name", "description") and ch.text:
                a[local(ch)] = ch.text.strip()
        for el in pm.iter():
            t = local(el)
            if t == "Point":
                for c in el.iter():
                    if local(c) == "coordinates":
                        points += parse_coords(c)
            elif t == "LineString":
                for c in el.iter():
                    if local(c) == "coordinates":
                        pts = parse_coords(c)
                        if len(pts) >= 2:
                            lines.append(pts)
            elif t == "Polygon":
                kind = 1
                for ring in el.iter():
                    if local(ring) == "LinearRing":
                        for c in ring.iter():
                            if local(c) == "coordinates":
                                pts = parse_coords(c)
                                if len(pts) >= 2:
                                    lines.append(pts)
        attrs.append(a)
    if points and not lines:
        return 2, [[p] for p in points], attrs
    return kind, lines, attrs


# ---------------------------------------------------------------- writer

def write_vpack(layers, path):
    with open(path, "wb") as f:
        f.write(struct.pack("<4sI", b"VPK1", len(layers)))
        for name, kind, polys in layers:
            nb = name.encode("utf8")
            f.write(struct.pack("<H", len(nb)))
            f.write(nb)
            f.write(struct.pack("<BBI", kind, 0, len(polys)))
            for pts in polys:
                imp = to_meters(vw_importance(pts)) if kind != 2 else [1.0e9] * len(pts)
                f.write(struct.pack("<I", len(pts)))
                for (lon, lat), m in zip(pts, imp):
                    f.write(struct.pack("<3f", lon, lat, min(m, 1.0e9)))


DEFAULT_EDITS = {
    "type": "FeatureCollection",
    "_readme": [
        "HAND-EDITABLE mask overrides. Each polygon forces the land/water classifier:",
        "properties.mask = 'land' | 'water'. Coordinates are WGS84 lon/lat. The engine",
        "rasterizes these OVER the survey mask AND over the in-window live-tide classifier",
        "at load -- edit, save, rerun. DON'T hand-digitize charted structures: seed them",
        "from surveyed vectors (the repo copy carries the Merrimack jetty footprints from",
        "OSM ways 640430599/640430601/256125510/559887262 -- see data/gis/structures.kml).",
    ],
    "features": [],
}


def main():
    os.makedirs(OUT, exist_ok=True)
    os.makedirs(CACHE, exist_ok=True)
    layers, registry = [], []

    # M7y: the survey upgrade's layers ride the same bin format -- CUSP shoreline (polyline
    # authority for the coast), NHD flowlines, NHD open-water rings (polygons).
    for name, kind, src in (("coast_ne", 0, "gshhg"), ("rivers_ne", 0, "gshhg"),
                            ("coast_global", 0, "gshhg"), ("cusp_ne", 0, "cusp"),
                            ("nhd_rivers_ne", 0, "nhd"), ("nhd_water_ne", 1, "nhd")):
        p = os.path.join(GIS, f"{name}.bin")
        if os.path.exists(p):
            polys = read_gshhg_bin(p)
            layers.append((name, kind, polys))
            registry.append({"layer": name, "kind": ["polyline", "polygon"][kind],
                             "source": src, "polys": len(polys),
                             "verts": sum(len(x) for x in polys)})

    for p in sorted(glob.glob(os.path.join(CACHE, "*.shp"))):
        name = os.path.splitext(os.path.basename(p))[0]
        r = read_shp(p)
        if not r:
            print(f"[vectors] {name}.shp: not a shapefile?")
            continue
        kind, polys = r
        attrs = read_dbf(p[:-4] + ".dbf")
        if attrs:
            json.dump(attrs, open(os.path.join(OUT, f"{name}.attrs.json"), "w"), indent=0)
        layers.append((name, kind, polys))
        registry.append({"layer": name, "kind": ["polyline", "polygon", "points"][kind],
                         "source": "shp", "polys": len(polys),
                         "verts": sum(len(x) for x in polys), "attrs": len(attrs)})

    for p in sorted(glob.glob(os.path.join(CACHE, "*.geojson"))):
        name = os.path.splitext(os.path.basename(p))[0]
        kind, polys, attrs = read_geojson(p)
        if attrs:
            json.dump(attrs, open(os.path.join(OUT, f"{name}.attrs.json"), "w"), indent=0)
        layers.append((name, kind, polys))
        registry.append({"layer": name, "kind": ["polyline", "polygon", "points"][kind],
                         "source": "geojson", "polys": len(polys),
                         "verts": sum(len(x) for x in polys)})

    for p in sorted(glob.glob(os.path.join(GIS, "*.kml")) +
                    glob.glob(os.path.join(CACHE, "*.kml"))):
        name = os.path.splitext(os.path.basename(p))[0]
        kind, polys, attrs = read_kml(p)
        if not polys:
            print(f"[vectors] {name}.kml: no geometry")
            continue
        if attrs:
            json.dump(attrs, open(os.path.join(OUT, f"{name}.attrs.json"), "w"), indent=0)
        layers.append((name, kind, polys))
        registry.append({"layer": name, "kind": ["polyline", "polygon", "points"][kind],
                         "source": "kml", "polys": len(polys),
                         "verts": sum(len(x) for x in polys)})

    write_vpack(layers, os.path.join(OUT, "vectors.vpack"))
    json.dump({"format": "VPK1",
               "importance": "sqrt(Visvalingam effective area) in meters = |wedge|/2 "
                             "(query LOD by filtering importance >= tolerance)",
               "crs": "EPSG:4326", "layers": registry},
              open(os.path.join(OUT, "vectors.json"), "w"), indent=1)
    total = sum(r["verts"] for r in registry)
    print(f"[vectors] {len(layers)} layers, {total} vertices (lossless) -> vectors.vpack")

    edits = os.path.join(GIS, "edits.geojson")
    if not os.path.exists(edits):
        json.dump(DEFAULT_EDITS, open(edits, "w"), indent=1)
        print(f"[vectors] authored {edits} (jetty land overrides -- edit by hand)")
    else:
        print(f"[vectors] {edits} exists; leaving your edits alone")


if __name__ == "__main__":
    main()
