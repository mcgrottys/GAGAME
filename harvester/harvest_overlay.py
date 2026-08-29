# ==================================================================================================
#  harvest_overlay.py - M6o: user GeoTIFF overlays as compositor planes, ALPHA AS FIBER.
#
#  The structural point (user call): quad trees (Google tiles) and FLAT IMAGES (GeoTIFFs)
#  combine in the paint loop's per-pixel lerp -- so a mostly-transparent GeoTIFF of
#  "highlights" must bleed through the final composed quadtree pixel by pixel, its alpha
#  multiplying the paint weight. This script feeds that path:
#
#    * Drop any GeoTIFF into cache/overlay/*.tif -- georeferencing is read from the GeoTIFF
#      tags (ModelPixelScale + ModelTiepoint; ProjectedCSTypeGeoKey checked for UTM 19N,
#      which is all v1 projects -- others are logged and skipped). RGB or RGBA; RGBA keeps
#      its alpha through an alpha-weighted mip soak.
#    * With no .tif present, generates a DEMO highlights overlay over the Merrimack inlet
#      (translucent ring at the jetty mouth + soft channel band) so the path is provable
#      without hunting for data.
#
#  Output: data/overlay/<name>.rgba mip chains + overlay.json (AerialOrthoSource schema,
#  channels=4, name "user.highlights").
# ==================================================================================================
import glob
import json
import math
import os

import numpy as np

CACHE = os.path.join("cache", "overlay")
OUT = os.path.join("data", "overlay")
UTM19N_EPSG = {6348, 26918, 26919, 32619}   # accepted ProjectedCSTypeGeoKey values


def utm19_inverse(e, n):
    a, f, k0 = 6378137.0, 1 / 298.257222101, 0.9996
    e2 = f * (2 - f)
    ep2 = e2 / (1 - e2)
    e1 = (1 - math.sqrt(1 - e2)) / (1 + math.sqrt(1 - e2))
    mu = (n / k0) / (a * (1 - e2 / 4 - 3 * e2 * e2 / 64 - 5 * e2**3 / 256))
    p1 = (mu + (3 * e1 / 2 - 27 * e1**3 / 32) * math.sin(2 * mu)
          + (21 * e1 * e1 / 16 - 55 * e1**4 / 32) * math.sin(4 * mu)
          + (151 * e1**3 / 96) * math.sin(6 * mu))
    s1, c1, t1 = math.sin(p1), math.cos(p1), math.tan(p1)
    cc, tt = ep2 * c1 * c1, t1 * t1
    n1 = a / math.sqrt(1 - e2 * s1 * s1)
    r1 = a * (1 - e2) / (1 - e2 * s1 * s1) ** 1.5
    d = (e - 500000.0) / (n1 * k0)
    lat = p1 - (n1 * t1 / r1) * (d * d / 2 - (5 + 3 * tt + 10 * cc - 4 * cc * cc - 9 * ep2)
                                 * d**4 / 24)
    lon = math.radians(-69.0) + (d - (1 + 2 * tt + cc) * d**3 / 6) / c1
    return math.degrees(lat), math.degrees(lon)


def reduce_rgba(level):
    """Alpha-weighted 2x2 box: color averages weight by alpha (a transparent pixel must not
    darken its neighbours), alpha averages plainly -- the soak, with the fiber respected."""
    h, w, _ = level.shape
    level = level[: h // 2 * 2, : w // 2 * 2].astype(np.float32)
    q = level.reshape(h // 2, 2, w // 2, 2, 4)
    a = q[..., 3:4]
    wsum = a.sum(axis=(1, 3))
    rgb = (q[..., :3] * a).sum(axis=(1, 3)) / np.maximum(wsum, 1e-3)
    out = np.zeros((h // 2, w // 2, 4), np.uint8)
    out[..., :3] = np.clip(rgb, 0, 255).astype(np.uint8)
    out[..., 3] = np.clip(a.mean(axis=(1, 3))[..., 0], 0, 255).astype(np.uint8)
    return out


def write_chain(img, path):
    mips = []
    with open(path, "wb") as f:
        level = img
        while True:
            mips.append({"px": int(level.shape[1]), "offset": f.tell()})
            f.write(level.tobytes())
            if level.shape[1] // 2 < 40 or level.shape[1] % 2:
                break
            level = reduce_rgba(level)
    return mips


def ingest_geotiffs():
    import tifffile
    entries = []
    for path in sorted(glob.glob(os.path.join(CACHE, "*.tif")) +
                       glob.glob(os.path.join(CACHE, "*.tiff"))):
        name = os.path.splitext(os.path.basename(path))[0]
        with tifffile.TiffFile(path) as tf:
            page = tf.pages[0]
            tags = {t.name: t.value for t in page.tags.values()}
            scale = tags.get("ModelPixelScaleTag")
            tie = tags.get("ModelTiepointTag")
            if scale is None or tie is None:
                print(f"[overlay] {name}: no GeoTIFF georeferencing tags; skipped")
                continue
            keys = tags.get("GeoKeyDirectoryTag", ())
            epsg = 0
            for k in range(4, len(keys), 4):
                if keys[k] == 3072:
                    epsg = keys[k + 3]
            if epsg and epsg not in UTM19N_EPSG:
                print(f"[overlay] {name}: EPSG {epsg} not UTM 19N; v1 skips (extend "
                      f"Projections.h to accept more)")
                continue
            img = page.asarray()
        if img.ndim == 2:
            img = np.stack([img] * 3, axis=-1)
        if img.shape[2] == 3:
            img = np.concatenate([img, np.full(img.shape[:2] + (1,), 255, img.dtype)], -1)
        img = img[:, :, :4]
        if img.dtype != np.uint8:
            img = np.clip(img.astype(np.float32) / img.max() * 255, 0, 255).astype(np.uint8)
        sx, sy = float(scale[0]), float(scale[1])
        i, j, _, X, Y, _ = [float(x) for x in tie[:6]]
        e0 = X - i * sx
        n1 = Y + j * sy
        e1 = e0 + img.shape[1] * sx
        n0 = n1 - img.shape[0] * sy
        entries.append((name, img, e0, n0, e1, n1, sx))
        print(f"[overlay] {name}: {img.shape[1]}x{img.shape[0]} rgba, "
              f"UTM E {e0:.0f}..{e1:.0f} N {n0:.0f}..{n1:.0f}, {sx} m/px")
    return entries


def demo_highlights():
    """No user GeoTIFF present: a translucent demo -- ring at the jetty mouth, soft band
    over the bar -- covering the aerial tiles' union at 0.5 m/px."""
    e0, n0, e1, n1 = 351000.0, 4740000.0, 352500.0, 4743000.0
    m = 0.5
    w, h = int((e1 - e0) / m), int((n1 - n0) / m)
    yy, xx = np.mgrid[0:h, 0:w]
    E = e0 + (xx + 0.5) * m
    N = n1 - (yy + 0.5) * m
    img = np.zeros((h, w, 4), np.uint8)
    # amber ring, r 260 m, 26 m wide, centred near the jetty mouth
    r = np.hypot(E - 352260.0, N - 4741900.0)
    ring = np.clip(1.0 - np.abs(r - 260.0) / 13.0, 0, 1)
    # soft highlight disc over the bar
    r2 = np.hypot(E - 351650.0, N - 4741350.0)
    disc = np.clip(1.0 - r2 / 320.0, 0, 1) ** 2
    a = np.clip(ring * 0.62 + disc * 0.38, 0, 1)
    img[..., 0] = 255
    img[..., 1] = np.where(ring > disc, 176, 64)
    img[..., 2] = np.where(ring > disc, 32, 200)
    img[..., 3] = (a * 255).astype(np.uint8)
    print(f"[overlay] demo highlights generated ({w}x{h} @ {m} m/px)")
    return [("demo_highlights", img, e0, n0, e1, n1, m)]


def squarify(entries):
    """AerialOrthoSource addresses SQUARE tiles (one px count serves both axes and the row
    stride). Slice any taller-than-wide plane into square row-chunks; a remainder that is
    not a full square is dropped with a log (pad upstream if it matters)."""
    out = []
    for name, img, e0, n0, e1, n1, m in entries:
        h, w = img.shape[:2]
        if h == w:
            out.append((name, img, e0, n0, e1, n1, m))
            continue
        if h % w:
            print(f"[overlay] {name}: dropping {h % w} remainder rows (not a full square)")
        for k in range(h // w):
            top = k * w
            n_hi = n1 - top * m
            out.append((f"{name}_{k}", img[top:top + w], e0, n_hi - w * m, e1, n_hi, m))
    return out


def main():
    os.makedirs(CACHE, exist_ok=True)
    os.makedirs(OUT, exist_ok=True)
    entries = squarify(ingest_geotiffs() or demo_highlights())
    tiles = []
    for name, img, e0, n0, e1, n1, m in entries:
        raw = os.path.join(OUT, f"{name}.rgba")
        mips = write_chain(img, raw)
        lat_sw, lon_sw = utm19_inverse(e0, n0)
        lat_ne, lon_ne = utm19_inverse(e1, n1)
        tiles.append({
            "file": f"{name}.rgba", "channels": 4,
            "utm_e0": e0, "utm_n0": n0, "utm_e1": e1, "utm_n1": n1,
            "m_per_px": m, "mips": mips,
            "wgs84_bbox": [lon_sw, lat_sw, lon_ne, lat_ne],
        })
        print(f"[overlay] {raw}: {len(mips)} mips, {os.path.getsize(raw)//1048576} MB")
    with open(os.path.join(OUT, "overlay.json"), "w") as f:
        json.dump({
            "name": "user.highlights",
            "structure": "geotiff overlay rgba (per-pixel alpha = paint weight)",
            "crs": "EPSG:6348 NAD83(2011) / UTM zone 19N",
            "cm_per_px": entries[0][6] * 100.0,
            "source": "user GeoTIFFs in cache/overlay/ (or the generated demo)",
            "tiles": tiles,
        }, f, indent=1)
    print(f"[overlay] done -> data/overlay/overlay.json ({len(tiles)} planes)")


if __name__ == "__main__":
    main()
