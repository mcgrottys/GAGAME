# ==================================================================================================
#  harvest_aerial.py - M6l: MassGIS 2023 15 cm aerial orthos (plane-flown, leaf-off) for the
#  Merrimack inlet -- the layer compositor's first INDEPENDENT high-res source, used to verify
#  that painting aligns projections and meters-per-pixel correctly.
#
#  Input: cache/aerial/19T*.zip (MassGIS S3, USNG-named 1500 m tiles; GeoJP2 declares
#  EPSG:6348 = NAD83(2011) / UTM zone 19N, 0.15 m/px, lossless JP2).
#  Output per tile (data/aerial/):
#    <tile>.rgb   8-bit RGB mip chain (box-filtered levels 0.15 -> ~19 m/px, concatenated)
#    aerial.json  georeferencing: UTM origin, m/px, mip table, and the WGS84 bbox
#  Requires Pillow (bundles OpenJPEG for the JP2 decode).
# ==================================================================================================
import glob
import json
import math
import os
import re
import zipfile

from PIL import Image

Image.MAX_IMAGE_PIXELS = 300_000_000
CACHE = os.path.join("cache", "aerial")
OUT = os.path.join("data", "aerial")


def utm19_inverse(e, n):
    """UTM 19N -> lat/lon degrees (GRS80). Snyder; for the json's coverage bbox only."""
    a = 6378137.0
    f = 1 / 298.257222101
    k0 = 0.9996
    e2 = f * (2 - f)
    ep2 = e2 / (1 - e2)
    e1 = (1 - math.sqrt(1 - e2)) / (1 + math.sqrt(1 - e2))
    m = (n - 0.0) / k0
    mu = m / (a * (1 - e2 / 4 - 3 * e2 * e2 / 64 - 5 * e2**3 / 256))
    p1 = (mu + (3 * e1 / 2 - 27 * e1**3 / 32) * math.sin(2 * mu)
          + (21 * e1 * e1 / 16 - 55 * e1**4 / 32) * math.sin(4 * mu)
          + (151 * e1**3 / 96) * math.sin(6 * mu))
    sin1, cos1, tan1 = math.sin(p1), math.cos(p1), math.tan(p1)
    c1 = ep2 * cos1 * cos1
    t1 = tan1 * tan1
    n1 = a / math.sqrt(1 - e2 * sin1 * sin1)
    r1 = a * (1 - e2) / (1 - e2 * sin1 * sin1) ** 1.5
    d = (e - 500000.0) / (n1 * k0)
    lat = p1 - (n1 * tan1 / r1) * (d * d / 2 - (5 + 3 * t1 + 10 * c1 - 4 * c1 * c1 - 9 * ep2)
                                   * d**4 / 24
                                   + (61 + 90 * t1 + 298 * c1 + 45 * t1 * t1 - 252 * ep2
                                      - 3 * c1 * c1) * d**6 / 720)
    lon = math.radians(-69.0) + (d - (1 + 2 * t1 + c1) * d**3 / 6
                                 + (5 - 2 * c1 + 28 * t1 - 3 * c1 * c1 + 8 * ep2 + 24 * t1 * t1)
                                 * d**5 / 120) / cos1
    return math.degrees(lat), math.degrees(lon)


def main():
    os.makedirs(OUT, exist_ok=True)
    tiles = []
    for zp in sorted(glob.glob(os.path.join(CACHE, "19T*.zip"))):
        name = os.path.splitext(os.path.basename(zp))[0]
        raw_path = os.path.join(OUT, name + ".rgb")
        z = zipfile.ZipFile(zp)
        aux = z.read(name + ".jp2.aux.xml").decode("utf8", errors="replace")
        mlo = re.search(r"<gml:lowerCorner>(\d+) (\d+)</gml:lowerCorner>", aux)
        mhi = re.search(r"<gml:upperCorner>(\d+) (\d+)</gml:upperCorner>", aux)
        e0, n0 = int(mlo.group(1)), int(mlo.group(2))
        e1, n1 = int(mhi.group(1)), int(mhi.group(2))
        mips = []
        if not os.path.exists(raw_path):
            jp2 = os.path.join(CACHE, name + ".jp2")
            if not os.path.exists(jp2):
                z.extract(name + ".jp2", CACHE)
            print(f"[aerial] decoding {name}.jp2 (10000^2 lossless; a minute or two)")
            im = Image.open(jp2).convert("RGB")
            with open(raw_path, "wb") as f:
                level = im
                px = level.size[0]
                while px >= 40:
                    mips.append({"px": px, "offset": f.tell()})
                    f.write(level.tobytes())
                    px = px // 2
                    if px >= 40:
                        level = level.resize((px, px), Image.BOX)
            print(f"[aerial] {name}: {len(mips)} mip levels, "
                  f"{os.path.getsize(raw_path)//1048576} MB raw")
        else:
            # Rebuild the mip table from the known layout (px halves from 10000).
            off = 0
            px = 10000
            while px >= 40:
                mips.append({"px": px, "offset": off})
                off += px * px * 3
                px //= 2
            print(f"[aerial] cache hit: {raw_path}")
        lat_sw, lon_sw = utm19_inverse(e0, n0)
        lat_ne, lon_ne = utm19_inverse(e1, n1)
        tiles.append({
            "file": name + ".rgb", "utm_e0": e0, "utm_n0": n0, "utm_e1": e1, "utm_n1": n1,
            "m_per_px": 0.15, "px": 10000, "mips": mips,
            "wgs84_bbox": [lon_sw, lat_sw, lon_ne, lat_ne],
        })
    with open(os.path.join(OUT, "aerial.json"), "w") as f:
        json.dump({
            "source": "MassGIS 2023 aerial orthos (15 cm, leaf-off, plane-flown)",
            "crs": "EPSG:6348 NAD83(2011) / UTM zone 19N",
            "tiles": tiles,
        }, f, indent=1)
    print(f"[aerial] done -> data/aerial/aerial.json ({len(tiles)} tiles)")


if __name__ == "__main__":
    main()
