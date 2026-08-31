# ==================================================================================================
#  harvest_hqbathy.py - M7z: THE HQ BED. BlueTopo + eHydro, refbathy's named successor.
#
#  harvest_refbathy.py ingested the vqview composite as "the proven first rung" and named this
#  harvester as the upgrade: build the Merrimack-inlet high-res seafloor plane from the PRIMARY
#  sources, reproducible on any machine from D:\DataCache alone.
#
#  Sources (rung rule: freshest-and-densest wins per cell):
#    * NOAA BlueTopo (the base, covers the whole box): anonymous S3, 6 known 4 m UTM19 tiles
#      (BH4ZW5GK/GL/GM + BH4ZX5GK/GL/GM), vertical NAVD88 (EPSG:5103) NATIVE - no conversion.
#      The current tile names come from the published tile-scheme GeoPackage (sqlite3 reads it;
#      queried by tile NAME, not rtree). Tiles are classic COGs: 512^2 deflate blocks, THREE
#      interleaved float32 bands (Elevation, Uncertainty, Contributor), NoData=NaN. The repo's
#      geotiff.py assumes one band per pixel, so a minimal chunky-3-band reader lives here
#      (numpy-decoded; predictor 1 and the TechNote-3 float predictor both handled).
#    * USACE eHydro (the channel truth, freshest): CENAE_MA_01_NEB surveys
#      20250908_CS_045 (full condition survey) then 20251007_XC_045 (fresher cross sections,
#      overlaid second so it wins where both hit). *_FULL.XYZ point clouds, NAD83 State Plane
#      MA Mainland US SURVEY FEET (EPSG:2249), Z = elevation usft relative to MLLW.
#
#  Projections, exact (Snyder, GRS80 -- the M6l alignment contract):
#    * lat/lon -> UTM19N forward TM ported from src/compose/Projections.h (the aerial
#      harvester's hand TM INVERSE is reproduced too, as the round-trip pin).
#    * State Plane MA Mainland inverse LCC: EPSG:2249's false origins converted usft->m first,
#      then the EPSG:26986 meter parameters (same cone Projections.h::MassMainland builds);
#      pinned against the M6l ACT0816 anchor before any point is trusted.
#    * NAD83 vs WGS84 ~1 m: documented, not corrected (M6l precedent).
#
#  Datum (the refbathy recipe, same station ladder): z_navd = z_mllw_usft * (1200/3937) - 1.400.
#  -1.400 m = MLLW minus NAVD88 at the entrance (Boston link -1.678 + MSL transfer +0.092,
#  focus mean_mllw 1.308). The survey's own XML quotes VDatum 4.43..5.29 ft (1.35..1.61 m) for
#  the project area -- our constant sits inside it. Sign: MLLW sits 1.400 m BELOW NAVD88 zero,
#  so elevations-above-MLLW convert by SUBTRACTING 1.400.
#
#  Compositing (thalweg doctrine lives upstream of the engine here: eHydro is used RAW, mean
#  of points per 1.5 m cell -- no box smearing at this resolution):
#    (a) all cells nodata; (b) BlueTopo bilinear per target cell centre (NaN-aware weights,
#    accepted where valid weight >= 0.5); (c) eHydro points binned per cell, mean, REPLACING
#    BlueTopo (fresher and denser in-channel); (d) remaining holes stay nodata (-9999) -- the
#    engine's height stack has CUDEM beneath this rung.
#
#  Validation before anything is written: LCC anchor pin, TM round-trip pin, then the datum
#  cross-check against the CUDEM plane (data/bathy/merrimack.*) at random wet cells, split
#  inside/outside the federal channel. Outside-channel |median| > 0.5 m aborts the write.
#
#  Output (BathyModel contract, < 2^26 cells): data/bathy/inlethq.f32 + inlethq.json
#  (+ inlethq_preview.png, inlethq_src.png). Raw products cached forever under
#  D:\DataCache\GAGAME\{bluetopo,ehydro}; polite fetches (UA, 0.6 s spacing, .part+rename).
# ==================================================================================================
import io
import json
import math
import os
import re
import sqlite3
import struct
import sys
import time
import urllib.error
import urllib.parse
import urllib.request
import zipfile
import zlib
from datetime import datetime, timezone

import numpy as np

DCACHE = r"D:\DataCache\GAGAME"
BT_DIR = os.path.join(DCACHE, "bluetopo")
EH_DIR = os.path.join(DCACHE, "ehydro")
OUT_DIR = os.path.join("data", "bathy")
NAME = "inlethq"

UA = "GAGAME/0.1 (hobby ocean simulator; contact: marksmcgrotty@gmail.com)"

S3 = "https://noaa-ocs-nationalbathymetry-pds.s3.amazonaws.com/"
SCHEME_PREFIX = "BlueTopo/_BlueTopo_Tile_Scheme/"
BT_TILES = ["BH4ZW5GK", "BH4ZW5GL", "BH4ZW5GM", "BH4ZX5GK", "BH4ZX5GL", "BH4ZX5GM"]

EH_INDEX = ("https://services7.arcgis.com/n1YM8pTrFmm7L4hs/arcgis/rest/services/"
            "eHydro_Survey_Data/FeatureServer/0/query")
EH_CHANNEL = "CENAE_MA_01_NEB"
EH_SURVEYS = ["20250908_CS_045", "20251007_XC_045"]   # overlay order: CS base, XC wins

# ---- the target grid: ~1.5 m cells, regular lat/lon, row 0 = north --------------------------
LON0, LON1 = -70.8700, -70.7400
LAT0, LAT1 = 42.7300, 42.8500
DLON = 1.5 / 81660.0        # deg per 1.5 m of longitude at 42.79 N
DLAT = 1.5 / 110574.0       # deg per 1.5 m of latitude
NX = math.ceil((LON1 - LON0) / DLON)
NY = math.ceil((LAT1 - LAT0) / DLAT)
NODATA = -9999.0

MLLW_MINUS_NAVD = -1.400            # m; the refbathy station ladder (see header)
USFT = 1200.0 / 3937.0              # US survey foot, exact

# Federal channel box for the validation split (dredging differences are EXPECTED inside).
CHAN = {"lon0": -70.828, "lon1": -70.790, "lat0": 42.808, "lat1": 42.824}

# M6l anchor pin: MA-LCC meters -> lon/lat (ACT0816), +-0.001 deg.
LCC_PIN = {"x": 256429.4, "y": 952196.8, "lon": -70.8110, "lat": 42.8183}

_last_fetch = [0.0]


def log(msg):
    print(f"[hqbathy] {msg}", flush=True)


# ---------------------------------------------------------------- fetch (cache-first, polite)

def fetch(url, path, min_bytes=1, max_age_s=None, timeout=900):
    """Forever-cache unless max_age_s; .part+rename; 0.6 s spacing; 3 tries."""
    if path and os.path.exists(path) and os.path.getsize(path) >= min_bytes:
        if max_age_s is None or time.time() - os.path.getmtime(path) < max_age_s:
            return open(path, "rb").read(), True
    os.makedirs(os.path.dirname(path), exist_ok=True)
    part = path + ".part"
    for attempt in range(3):
        wait = 0.6 - (time.monotonic() - _last_fetch[0])
        if wait > 0:
            time.sleep(wait)
        try:
            req = urllib.request.Request(url, headers={"User-Agent": UA})
            with urllib.request.urlopen(req, timeout=timeout) as r, open(part, "wb") as f:
                while True:
                    chunk = r.read(1 << 20)
                    if not chunk:
                        break
                    f.write(chunk)
            _last_fetch[0] = time.monotonic()
            sz = os.path.getsize(part)
            if sz < min_bytes:
                raise RuntimeError(f"only {sz} bytes (wanted >= {min_bytes})")
            os.replace(part, path)
            what = url.rsplit("/", 1)[-1].split("?")[0] or url.split("?")[-1][:60]
            log(f"    fetched {what}: {sz / 1e6:.1f} MB")
            return open(path, "rb").read(), False
        except urllib.error.HTTPError:
            _last_fetch[0] = time.monotonic()
            raise                      # 404 etc: caller re-lists the index, no hammering
        except Exception as e:  # noqa: BLE001
            _last_fetch[0] = time.monotonic()
            if attempt == 2:
                raise
            log(f"    retry {attempt + 1}: {e}")
            time.sleep(2.0 * (2 ** attempt))


def s3_list(prefix, cache_name, max_age_s=7 * 86400):
    """S3 list-type=2 XML GET -> keys under the prefix."""
    url = S3 + "?" + urllib.parse.urlencode({"list-type": "2", "prefix": prefix})
    xml, _ = fetch(url, os.path.join(BT_DIR, cache_name), max_age_s=max_age_s)
    return re.findall(r"<Key>([^<]+)</Key>", xml.decode("utf-8", errors="replace"))


# ---------------------------------------------------------------- projections (Snyder, GRS80)

def tm19_forward(lat_deg, lon_deg):
    """lat/lon degrees -> UTM zone 19N easting/northing. Projections.h::TransverseMercator,
    vectorized. GRS80, k0 = 0.9996, central meridian -69."""
    a, f, k0 = 6378137.0, 1.0 / 298.257222101, 0.9996
    e2 = f * (2.0 - f)
    ep2 = e2 / (1.0 - e2)
    lat = np.radians(np.asarray(lat_deg, np.float64))
    lon = np.radians(np.asarray(lon_deg, np.float64))
    s, c, t = np.sin(lat), np.cos(lat), np.tan(lat)
    n_ = a / np.sqrt(1.0 - e2 * s * s)
    big_t = t * t
    big_c = ep2 * c * c
    big_a = c * (lon - math.radians(-69.0))
    m = a * ((1 - e2 / 4 - 3 * e2 * e2 / 64 - 5 * e2**3 / 256) * lat
             - (3 * e2 / 8 + 3 * e2 * e2 / 32 + 45 * e2**3 / 1024) * np.sin(2 * lat)
             + (15 * e2 * e2 / 256 + 45 * e2**3 / 1024) * np.sin(4 * lat)
             - (35 * e2**3 / 3072) * np.sin(6 * lat))
    a2 = big_a * big_a
    a3, a4 = a2 * big_a, a2 * a2
    a5, a6 = a4 * big_a, a4 * a2
    e_ = 500000.0 + k0 * n_ * (big_a + (1 - big_t + big_c) * a3 / 6.0
                               + (5 - 18 * big_t + big_t * big_t + 72 * big_c - 58 * ep2)
                               * a5 / 120.0)
    n_out = k0 * (m + n_ * t * (a2 / 2.0 + (5 - big_t + 9 * big_c + 4 * big_c * big_c)
                                * a4 / 24.0
                                + (61 - 58 * big_t + big_t * big_t + 600 * big_c - 330 * ep2)
                                * a6 / 720.0))
    return e_, n_out


def utm19_inverse(e, n):
    """UTM 19N -> lat/lon degrees (GRS80). harvest_aerial.py's hand TM inverse, verbatim."""
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


def lcc_mass_inverse(x_m, y_m):
    """State Plane MA Mainland METERS -> lat/lon degrees. The inverse of the cone
    Projections.h::MassMainland builds (LCC 2SP, GRS80): sp 41d43' / 42d41', lat0 41,
    lon0 -71.5, FE 200000 m, FN 750000 m. EPSG:2249 callers convert usft->m FIRST."""
    a, f = 6378137.0, 1.0 / 298.257222101
    e = math.sqrt(f * (2.0 - f))
    sp1, sp2 = math.radians(41.71666666666667), math.radians(42.68333333333333)
    lat0, lon0 = math.radians(41.0), math.radians(-71.5)

    def m_(p):
        return math.cos(p) / math.sqrt(1.0 - e * e * math.sin(p) ** 2)

    def t_(p):
        return (math.tan(math.pi / 4.0 - p / 2.0)
                / ((1.0 - e * math.sin(p)) / (1.0 + e * math.sin(p))) ** (e / 2.0))

    n = (math.log(m_(sp1)) - math.log(m_(sp2))) / (math.log(t_(sp1)) - math.log(t_(sp2)))
    a_f = a * m_(sp1) / (n * t_(sp1) ** n)
    rho0 = a_f * t_(lat0) ** n

    xp = np.asarray(x_m, np.float64) - 200000.0
    yp = np.asarray(y_m, np.float64) - 750000.0
    rho = math.copysign(1.0, n) * np.sqrt(xp * xp + (rho0 - yp) ** 2)
    theta = np.arctan2(xp, rho0 - yp)
    tt = (rho / a_f) ** (1.0 / n)
    lat = np.pi / 2.0 - 2.0 * np.arctan(tt)
    for _ in range(8):
        es = e * np.sin(lat)
        lat = np.pi / 2.0 - 2.0 * np.arctan(tt * ((1.0 - es) / (1.0 + es)) ** (e / 2.0))
    return np.degrees(lat), np.degrees(theta / n + lon0)


def projection_pins():
    """Both projections must hit their pins before any data is trusted."""
    lat, lon = lcc_mass_inverse(LCC_PIN["x"], LCC_PIN["y"])
    dlat, dlon = abs(float(lat) - LCC_PIN["lat"]), abs(float(lon) - LCC_PIN["lon"])
    log(f"LCC anchor: ({LCC_PIN['x']}, {LCC_PIN['y']}) m -> ({lat:.5f}, {lon:.5f}); "
        f"pin ({LCC_PIN['lat']}, {LCC_PIN['lon']}) delta ({dlat:.5f}, {dlon:.5f}) deg")
    if dlat > 0.001 or dlon > 0.001:
        raise SystemExit("LCC anchor pin FAILED -- refusing to convert eHydro coordinates")
    la, lo = utm19_inverse(350000.0, 4738000.0)
    e2, n2 = tm19_forward(la, lo)
    err = math.hypot(float(e2) - 350000.0, float(n2) - 4738000.0)
    log(f"TM round-trip: (350000, 4738000) -> ({la:.6f}, {lo:.6f}) -> back, {err * 100:.2f} cm")
    if err > 0.05:
        raise SystemExit("TM round-trip pin FAILED -- refusing to sample BlueTopo")


# ---------------------------------------------------------------- BlueTopo COG reader

class BlueTopoTiff:
    """Just enough TIFF for BlueTopo: classic (magic 42), first IFD, tiled, N interleaved
    float32 samples, deflate or raw, predictor none or TechNote-3 floating-point. Band 0 is
    Elevation. Anything else in the wild -> a clear error, never fabricated data."""

    _TYPE = {1: "B", 3: "H", 4: "I", 8: "h", 9: "i", 11: "f", 12: "d", 16: "Q"}
    _SIZE = {1: 1, 2: 1, 3: 2, 4: 4, 5: 8, 7: 1, 8: 2, 9: 4, 11: 4, 12: 8, 16: 8}

    def __init__(self, data, name):
        self.data = data
        self.name = name
        if data[:2] == b"II":
            self.e = "<"
        elif data[:2] == b"MM":
            self.e = ">"
        else:
            raise SystemExit(f"{name}: not a TIFF")
        magic = struct.unpack_from(self.e + "H", data, 2)[0]
        if magic == 43:
            raise SystemExit(f"{name}: BigTIFF -- this reader handles the classic COGs "
                             "BlueTopo currently publishes; STOPPING rather than guessing")
        if magic != 42:
            raise SystemExit(f"{name}: bad TIFF magic {magic}")
        tags = {}
        (off,) = struct.unpack_from(self.e + "I", data, 4)
        (cnt,) = struct.unpack_from(self.e + "H", data, off)
        for i in range(cnt):
            tag, typ, n = struct.unpack_from(self.e + "HHI", data, off + 2 + i * 12)
            val = data[off + 2 + i * 12 + 8:off + 2 + i * 12 + 12]
            size = self._SIZE.get(typ, 1) * n
            raw = val[:size] if size <= 4 else data[
                struct.unpack_from(self.e + "I", val)[0]:
                struct.unpack_from(self.e + "I", val)[0] + size]
            fmt = self._TYPE.get(typ)
            tags[tag] = (struct.unpack_from(self.e + fmt * n, raw) if fmt else raw)

        def t1(tag, default=None):
            v = tags.get(tag)
            return v[0] if v else default

        self.w, self.h = t1(256), t1(257)
        self.spp = t1(277, 1)
        self.comp = t1(259, 1)
        self.planar = t1(284, 1)
        self.pred = t1(317, 1)
        self.tw, self.th = t1(322), t1(323)
        self.offsets = tags.get(324, ())
        self.counts = tags.get(325, ())
        bits = tags.get(258, (32,))
        fmts = tags.get(339, (3,))
        if (self.tw is None or self.planar != 1 or set(bits) != {32}
                or set(fmts) != {3}):
            raise SystemExit(f"{name}: layout {bits}/{fmts} planar {self.planar} tiled "
                             f"{self.tw} -- not the interleaved float32 COG this reader knows")
        if self.comp not in (1, 8, 32946):
            raise SystemExit(f"{name}: compression {self.comp} (JPEG/LERC/ZSTD would land "
                             "here) -- STOPPING rather than fabricating bathymetry")
        scale = tags.get(33550, (1.0, 1.0, 0.0))
        tie = tags.get(33922, (0.0,) * 6)
        self.sx, self.sy = scale[0], scale[1]
        self.e0 = tie[3] - tie[0] * self.sx      # model x of raster (0,0): NW pixel corner
        self.n1 = tie[4] + tie[1] * self.sy      # model y of raster (0,0)

    def _undo_fp_predictor(self, dec):
        """TIFF TechNote 3: per row, byte-delta then MSB-first byte planes across the row."""
        w = self.tw * self.spp                   # samples per tile row
        a = np.frombuffer(dec, np.uint8).reshape(self.th, 4 * w)
        a = np.cumsum(a.astype(np.int64), axis=1) & 0xFF
        planes = a.astype(np.uint8).reshape(self.th, 4, w)
        le = planes[:, ::-1, :].transpose(0, 2, 1)          # -> little-endian byte order
        return np.ascontiguousarray(le).tobytes()

    def band0(self):
        out = np.full((self.h, self.w), np.nan, np.float32)
        tiles_x = (self.w + self.tw - 1) // self.tw
        expect = self.tw * self.th * self.spp * 4
        for idx, (off, cnt) in enumerate(zip(self.offsets, self.counts)):
            if cnt == 0:
                continue                          # sparse COG block: stays NaN
            raw = self.data[off:off + cnt]
            dec = raw if self.comp == 1 else zlib.decompress(raw)
            if self.pred == 3:
                dec = self._undo_fp_predictor(dec)
            elif self.pred not in (0, 1):
                raise SystemExit(f"{self.name}: predictor {self.pred} unsupported")
            if len(dec) < expect:
                dec = dec + b"\x00" * (expect - len(dec))
            block = np.frombuffer(dec[:expect], self.e + "f4").reshape(
                self.th, self.tw, self.spp)
            by, bx = divmod(idx, tiles_x)
            y0, x0 = by * self.th, bx * self.tw
            hh = min(self.th, self.h - y0)
            ww = min(self.tw, self.w - x0)
            band = block[:hh, :ww, 0]
            if self.e == ">":
                band = band.astype("<f4")
            out[y0:y0 + hh, x0:x0 + ww] = band
        out[np.abs(out) > 99999.0] = np.nan
        return out


def fetch_bluetopo_tiles():
    """Tile-scheme gpkg (current name from the S3 listing) -> the 6 tiles' GeoTIFF links ->
    cached tiffs. Returns [(name, delivered, BlueTopoTiff)]."""
    keys = s3_list(SCHEME_PREFIX, "_tile_scheme_listing.xml")
    gpkgs = sorted(k for k in keys if k.lower().endswith(".gpkg"))
    if not gpkgs:
        raise SystemExit("no tile-scheme gpkg in the S3 listing")
    scheme_key = gpkgs[-1]
    scheme_path = os.path.join(BT_DIR, scheme_key.rsplit("/", 1)[-1])
    _, cached = fetch(S3 + scheme_key, scheme_path, min_bytes=1_000_000)
    log(f"tile scheme: {os.path.basename(scheme_path)} "
        f"({os.path.getsize(scheme_path) / 1e6:.1f} MB{', cached' if cached else ''})")

    db = sqlite3.connect(scheme_path)
    cur = db.cursor()
    (table,) = [r[0] for r in cur.execute(
        "SELECT table_name FROM gpkg_contents WHERE data_type='features'")]
    rows = {r[0]: r for r in cur.execute(
        f'SELECT tile, GeoTIFF_Link, Delivered_Date, Resolution, UTM FROM "{table}" '
        f"WHERE tile IN ({','.join('?' * len(BT_TILES))})", BT_TILES)}
    db.close()
    missing = [t for t in BT_TILES if t not in rows]
    if missing:
        raise SystemExit(f"tile scheme no longer lists {missing} -- the scheme moved under us")

    tiles = []
    for tname in BT_TILES:
        _, link, delivered, res, utm = rows[tname]
        if utm not in ("19", 19):
            raise SystemExit(f"{tname}: UTM zone {utm} (expected 19)")
        path = os.path.join(BT_DIR, link.rsplit("/", 1)[-1])
        try:
            data, cached = fetch(link, path, min_bytes=500_000)
        except urllib.error.HTTPError as ex:
            if ex.code != 404:
                raise
            # The tile was re-delivered since the scheme snapshot: list its prefix once.
            log(f"    {tname}: {ex.code} on {link.rsplit('/', 1)[-1]}; re-listing prefix")
            keys = s3_list(f"BlueTopo/{tname}/", f"_listing_{tname}.xml", max_age_s=3600)
            tiffs = sorted(k for k in keys if k.lower().endswith(".tiff"))
            if not tiffs:
                raise SystemExit(f"{tname}: no tiff under BlueTopo/{tname}/")
            path = os.path.join(BT_DIR, tiffs[-1].rsplit("/", 1)[-1])
            data, cached = fetch(S3 + tiffs[-1], path, min_bytes=500_000)
        tif = BlueTopoTiff(data, os.path.basename(path))
        if abs(tif.sx - 4.0) > 1e-6 or abs(tif.sy - 4.0) > 1e-6:
            raise SystemExit(f"{tname}: pixel {tif.sx}x{tif.sy} m (expected 4 m); the "
                             "resolution ladder changed -- re-plan before compositing")
        log(f"    {tname}: {tif.w}x{tif.h} px, comp {tif.comp}, pred {tif.pred}, "
            f"{len(data) / 1e6:.1f} MB {'(cached)' if cached else '(fetched)'}, "
            f"delivered {delivered}")
        tiles.append((os.path.basename(path), delivered, tif))
    return tiles


def build_mosaic(tiles):
    """Paste the 6 UTM19 tiles (one shared 4 m lattice) into one elevation array."""
    e_min = min(t.e0 for _, _, t in tiles)
    e_max = max(t.e0 + t.w * 4.0 for _, _, t in tiles)
    n_max = max(t.n1 for _, _, t in tiles)
    n_min = min(t.n1 - t.h * 4.0 for _, _, t in tiles)
    mw = int(round((e_max - e_min) / 4.0))
    mh = int(round((n_max - n_min) / 4.0))
    mosaic = np.full((mh, mw), np.nan, np.float32)
    for name, _, t in tiles:
        fx = (t.e0 - e_min) / 4.0
        fy = (n_max - t.n1) / 4.0
        ox, oy = int(round(fx)), int(round(fy))
        if abs(fx - ox) > 1e-3 or abs(fy - oy) > 1e-3:
            raise SystemExit(f"{name}: tile off the shared 4 m lattice by ({fx - ox}, "
                             f"{fy - oy}) px -- mosaic assumption broken")
        band = t.band0()
        dst = mosaic[oy:oy + t.h, ox:ox + t.w]
        np.copyto(dst, band, where=np.isfinite(band))
        log(f"    {name}: E {t.e0:.0f}..{t.e0 + t.w * 4:.0f}, N {t.n1 - t.h * 4:.0f}.."
            f"{t.n1:.0f}, {100.0 * np.isfinite(band).mean():.0f}% valid")
    log(f"mosaic {mw}x{mh} px (4 m), {100.0 * np.isfinite(mosaic).mean():.0f}% valid")
    return mosaic, e_min, n_max


def splat_bluetopo(grid, src, mosaic, m_e0, m_n1):
    """(b) of the recipe: bilinear sample of the mosaic at every target cell centre.
    NaN-aware: weights renormalized over valid neighbours, accepted where they carry >= 0.5."""
    mh, mw = mosaic.shape
    lon_c = LON0 + (np.arange(NX, dtype=np.float64) + 0.5) * DLON
    filled = 0
    chunk = 128
    for r0 in range(0, NY, chunk):
        r1 = min(r0 + chunk, NY)
        lat_c = LAT1 - (np.arange(r0, r1, dtype=np.float64) + 0.5) * DLAT
        lat2 = np.repeat(lat_c[:, None], NX, axis=1)
        lon2 = np.broadcast_to(lon_c, lat2.shape)
        e, n = tm19_forward(lat2, lon2)
        px = (e - m_e0) / 4.0 - 0.5
        py = (m_n1 - n) / 4.0 - 0.5
        x0 = np.floor(px).astype(np.int64)
        y0 = np.floor(py).astype(np.int64)
        fx = (px - x0).astype(np.float32)
        fy = (py - y0).astype(np.float32)
        inb = (x0 >= 0) & (x0 < mw - 1) & (y0 >= 0) & (y0 < mh - 1)
        x0c = np.clip(x0, 0, mw - 2)
        y0c = np.clip(y0, 0, mh - 2)
        wsum = np.zeros(fx.shape, np.float32)
        vsum = np.zeros(fx.shape, np.float32)
        for oy, ox, w in ((0, 0, (1 - fx) * (1 - fy)), (0, 1, fx * (1 - fy)),
                          (1, 0, (1 - fx) * fy), (1, 1, fx * fy)):
            v = mosaic[y0c + oy, x0c + ox]
            ok = np.isfinite(v)
            wsum += np.where(ok, w, 0.0)
            vsum += np.where(ok, np.where(ok, v, 0.0) * w, 0.0)
        good = inb & (wsum >= 0.5)
        vals = vsum / np.maximum(wsum, np.float32(1e-9))
        grid[r0:r1][good] = vals[good]
        src[r0:r1][good] = 1
        filled += int(good.sum())
    log(f"BlueTopo splat: {filled} of {NX * NY} cells "
        f"({100.0 * filled / (NX * NY):.1f}%)")
    return filled


# ---------------------------------------------------------------- eHydro

def ehydro_index():
    params = {"where": f"channelareaidfk='{EH_CHANNEL}'", "outFields": "*", "f": "json"}
    url = EH_INDEX + "?" + urllib.parse.urlencode(params)
    cache = os.path.join(EH_DIR, f"index_{EH_CHANNEL}.json")
    raw, cached = fetch(url, cache, min_bytes=200, max_age_s=30 * 86400)
    feats = json.loads(raw).get("features", [])
    log(f"eHydro index: {len(feats)} surveys for {EH_CHANNEL} "
        f"{'(cached)' if cached else '(fetched)'}")
    return feats


def parse_xyz(text):
    """Space-delimited X Y Z rows after the Key==value header paragraphs: skip lines until
    one parses as three floats, then bulk-load (line loop fallback for stray text)."""
    lines = text.split("\n")
    start = None
    for i, ln in enumerate(lines):
        p = ln.split()
        if len(p) == 3:
            try:
                float(p[0]), float(p[1]), float(p[2])
                start = i
                break
            except ValueError:
                continue
    if start is None:
        raise RuntimeError("no 'X Y Z' data rows found")
    try:
        arr = np.loadtxt(io.StringIO("\n".join(lines[start:])), dtype=np.float64, ndmin=2)
        if arr.shape[1] != 3:
            raise ValueError(f"{arr.shape[1]} columns")
    except ValueError:
        rows = []
        for ln in lines[start:]:
            p = ln.split()
            if len(p) != 3:
                continue
            try:
                rows.append((float(p[0]), float(p[1]), float(p[2])))
            except ValueError:
                continue
        arr = np.array(rows, np.float64)
    return arr


def overlay_ehydro(grid, src, feats):
    """(c) of the recipe: per survey (oldest first, fresher wins), bin the FULL point cloud
    into target cells; the per-cell mean REPLACES BlueTopo. Returns provenance list."""
    chosen = []
    for want in EH_SURVEYS:
        hits = [f["attributes"] for f in feats
                if want in str(f["attributes"].get("surveyjobidpk", ""))]
        if not hits:
            log(f"    survey *{want}*: NOT in the index (skipping; plan called it optional "
                "for the XC)")
            continue
        chosen.append(hits[0])

    used = []
    for att in chosen:
        sid = att["surveyjobidpk"]
        url = att["sourcedatalocation"]
        path = os.path.join(EH_DIR, url.rsplit("/", 1)[-1])
        data, cached = fetch(url, path, min_bytes=1_000_000)
        log(f"    {sid}: {len(data) / 1e6:.1f} MB {'(cached)' if cached else '(fetched)'}")
        zf = zipfile.ZipFile(io.BytesIO(data))
        xyzs = [n for n in zf.namelist() if n.upper().endswith(".XYZ")]
        member = (next((n for n in xyzs if n.upper().endswith("_FULL.XYZ")), None)
                  or next((n for n in xyzs if n.upper().endswith("_A.XYZ")), None)
                  or max(xyzs, key=lambda n: zf.getinfo(n).file_size))
        pts = parse_xyz(zf.read(member).decode("ascii", errors="replace"))
        # EPSG:2249 usft -> meters FIRST, then the meter-parameter LCC inverse.
        lat, lon = lcc_mass_inverse(pts[:, 0] * USFT, pts[:, 1] * USFT)
        z_navd = pts[:, 2] * USFT + MLLW_MINUS_NAVD
        ix = np.floor((lon - LON0) / DLON).astype(np.int64)
        iy = np.floor((LAT1 - lat) / DLAT).astype(np.int64)
        ok = (ix >= 0) & (ix < NX) & (iy >= 0) & (iy < NY)
        flat = (iy[ok] * NX + ix[ok])
        uniq, inv = np.unique(flat, return_inverse=True)
        means = (np.bincount(inv, weights=z_navd[ok]) / np.bincount(inv)).astype(np.float32)
        grid.reshape(-1)[uniq] = means
        src.reshape(-1)[uniq] = 2
        log(f"    {member.rsplit('/', 1)[-1]}: {len(pts)} pts, {int(ok.sum())} in-grid "
            f"-> {len(uniq)} cells; lon {lon.min():.4f}..{lon.max():.4f} lat "
            f"{lat.min():.4f}..{lat.max():.4f}, z {z_navd.min():.1f}..{z_navd.max():.1f} m NAVD")
        used.append({"survey": sid, "zip": url, "xyz": member.rsplit("/", 1)[-1],
                     "points": int(len(pts)), "cells": int(len(uniq))})
    if not used:
        raise SystemExit("no eHydro survey found at all -- the channel truth is the point "
                         "of this harvester; refusing to ship BlueTopo alone")
    return used


# ---------------------------------------------------------------- validation

def cudem_bilinear(meta, cud, lat, lon):
    """Bilinear sample of the CUDEM plane; invalid where out of grid or any corner nodata."""
    px = (lon - meta["lon0"]) / meta["dlon"] - 0.5
    py = (meta["lat1"] - lat) / meta["dlat"] - 0.5
    x0 = np.floor(px).astype(np.int64)
    y0 = np.floor(py).astype(np.int64)
    inb = (x0 >= 0) & (x0 < meta["nx"] - 1) & (y0 >= 0) & (y0 < meta["ny"] - 1)
    x0c = np.clip(x0, 0, meta["nx"] - 2)
    y0c = np.clip(y0, 0, meta["ny"] - 2)
    fx = px - x0
    fy = py - y0
    v00 = cud[y0c, x0c]
    v01 = cud[y0c, x0c + 1]
    v10 = cud[y0c + 1, x0c]
    v11 = cud[y0c + 1, x0c + 1]
    ok = inb & (v00 > -9000) & (v01 > -9000) & (v10 > -9000) & (v11 > -9000)
    val = (v00 * (1 - fx) * (1 - fy) + v01 * fx * (1 - fy)
           + v10 * (1 - fx) * fy + v11 * fx * fy)
    return val, ok


def stats_line(tag, d):
    return (f"    {tag}: n={d.size} mean={d.mean():+.3f} median={np.median(d):+.3f} "
            f"p5={np.percentile(d, 5):+.3f} p95={np.percentile(d, 95):+.3f} m")


def validate_vs_cudem(grid):
    """The datum cross-check: inlethq minus CUDEM-bilinear at random wet cells, split
    inside/outside the federal channel. Returns (report dict, outside-median)."""
    meta = json.load(open(os.path.join(OUT_DIR, "merrimack.json")))
    cud = np.fromfile(os.path.join(OUT_DIR, "merrimack.f32"),
                      np.float32).reshape(meta["ny"], meta["nx"])
    rng = np.random.default_rng(20260831)
    iy = rng.integers(0, NY, 1_500_000)
    ix = rng.integers(0, NX, 1_500_000)
    v = grid[iy, ix]
    lat = LAT1 - (iy + 0.5) * DLAT
    lon = LON0 + (ix + 0.5) * DLON
    wet = (v > -9000) & (v < -1.0)
    cv, cok = cudem_bilinear(meta, cud, lat, lon)
    keep = wet & cok
    diff = (v - cv)[keep]
    in_ch = ((lon >= CHAN["lon0"]) & (lon <= CHAN["lon1"])
             & (lat >= CHAN["lat0"]) & (lat <= CHAN["lat1"]))[keep]
    d_in = diff[in_ch][:20000]
    d_out = diff[~in_ch][:20000]
    log("datum cross-check vs CUDEM (inlethq - cudem, wet cells < -1 m):")
    if d_in.size:
        log(stats_line("inside federal channel (dredge/shoal EXPECTED)", d_in))
    if d_out.size:
        log(stats_line("outside channel (datum agreement)", d_out))
    med_out = float(np.median(d_out)) if d_out.size else 0.0
    rep = {"inside_channel": {"n": int(d_in.size), "mean_m": round(float(d_in.mean()), 3),
                              "median_m": round(float(np.median(d_in)), 3)} if d_in.size else None,
           "outside_channel": {"n": int(d_out.size), "mean_m": round(float(d_out.mean()), 3),
                               "median_m": round(med_out, 3)} if d_out.size else None}
    return rep, med_out


# ---------------------------------------------------------------- output

def fnv1a(buf):
    h = 0xcbf29ce484222325
    for b in memoryview(buf):
        h = ((h ^ b) * 0x100000001b3) & 0xFFFFFFFFFFFFFFFF
    return h


def write_png_gray(path, arr):
    """8-bit grayscale PNG, stdlib only (the harvest_bathy/refbathy pattern, numpy rows)."""
    h, w = arr.shape
    raw = np.zeros((h, w + 1), np.uint8)
    raw[:, 1:] = arr

    def chunk(tag, payload):
        c = struct.pack(">I", len(payload)) + tag + payload
        return c + struct.pack(">I", zlib.crc32(tag + payload) & 0xFFFFFFFF)

    with open(path, "wb") as f:
        f.write(b"\x89PNG\r\n\x1a\n")
        f.write(chunk(b"IHDR", struct.pack(">IIBBBBB", w, h, 8, 0, 0, 0, 0)))
        f.write(chunk(b"IDAT", zlib.compress(raw.tobytes(), 6)))
        f.write(chunk(b"IEND", b""))


def write_previews(grid, src):
    g = grid[::4, ::4]                                   # 1/4-res proofs
    pix = np.full(g.shape, 8, np.uint8)
    wet = (g > -9000) & (g < 0)
    dry = g >= 0
    pix[wet] = np.clip(120.0 + g[wet] * 8.0, 16, 120).astype(np.uint8)
    pix[dry] = np.clip(140.0 + g[dry] * 10.0, 140, 255).astype(np.uint8)
    write_png_gray(os.path.join(OUT_DIR, f"{NAME}_preview.png"), pix)
    write_png_gray(os.path.join(OUT_DIR, f"{NAME}_src.png"), src[::4, ::4] * 127)
    log(f"previews -> {NAME}_preview.png, {NAME}_src.png ({pix.shape[1]}x{pix.shape[0]})")


# ---------------------------------------------------------------- main

def main():
    if NX * NY > (1 << 26):
        raise SystemExit(f"{NX}x{NY} = {NX * NY} cells exceeds BathyModel's 2^26 cap")
    log(f"target grid {NX} x {NY} = {NX * NY} cells at ~1.5 m "
        f"(cap 2^26 = {1 << 26}; dlon {DLON:.3e}, dlat {DLAT:.3e})")
    os.makedirs(BT_DIR, exist_ok=True)
    os.makedirs(EH_DIR, exist_ok=True)
    os.makedirs(OUT_DIR, exist_ok=True)

    projection_pins()

    # ---- (a) start all-nodata; (b) BlueTopo base ----
    tiles = fetch_bluetopo_tiles()
    mosaic, m_e0, m_n1 = build_mosaic(tiles)
    grid = np.full((NY, NX), NODATA, np.float32)
    src = np.zeros((NY, NX), np.uint8)
    splat_bluetopo(grid, src, mosaic, m_e0, m_n1)
    del mosaic

    # ---- (c) eHydro overlay (fresher, denser in-channel: it REPLACES) ----
    surveys = overlay_ehydro(grid, src, ehydro_index())

    # ---- validation gates, BEFORE anything is written ----
    valid = grid > -9000
    cover = float(valid.mean())
    vmin = float(grid[valid].min())
    vmax = float(grid[valid].max())
    n_bt = int((src == 1).sum())
    n_eh = int((src == 2).sum())
    log(f"grid: {100 * cover:.1f}% covered (bluetopo {n_bt}, ehydro {n_eh} cells), "
        f"range [{vmin:.1f}, {vmax:.1f}] m NAVD88")
    if vmin < -60.0 or vmax > 40.0:
        log("WARNING: range outside the expected -25..+12 envelope; inspect the preview")
    rep, med_out = validate_vs_cudem(grid)
    if abs(med_out) > 0.5:
        raise SystemExit(
            f"outside-channel median {med_out:+.3f} m vs CUDEM exceeds 0.5 m -- a datum or "
            "projection sign is wrong (check usft factor, the -1.400 sign, the UTM zone). "
            "NOT writing output.")
    if abs(med_out) > 0.35:
        log(f"WARNING: outside-channel median {med_out:+.3f} m > 0.35 m -- datums only "
            "loosely aligned; shipping, but the waterline gate should re-audit")

    # ---- write: f32, identity hash, sidecar, previews ----
    f32_path = os.path.join(OUT_DIR, f"{NAME}.f32")
    blob = grid.astype("<f4").tobytes()
    with open(f32_path, "wb") as f:
        f.write(blob)
    log(f"wrote {f32_path} ({len(blob) / 1e6:.1f} MB); hashing (FNV-1a, ~30 s)")
    content = fnv1a(blob)

    meta = {
        "generated_utc": datetime.now(timezone.utc).isoformat(timespec="seconds"),
        "source": ("NOAA BlueTopo 4 m UTM19 COGs (%s) + USACE eHydro %s (%s), "
                   "rung-rule composite at ~1.5 m: eHydro cell means replace BlueTopo "
                   "bilinear; holes stay nodata (CUDEM sits beneath this rung)"
                   % (", ".join(n for n, _, _ in tiles), EH_CHANNEL,
                      ", ".join(s["survey"] for s in surveys))),
        "file": f"{NAME}.f32", "nx": NX, "ny": NY,
        "lon0": LON0, "lat1": LAT1, "dlon": DLON, "dlat": DLAT,
        "row0": "north", "nodata": NODATA,
        "min_m": round(vmin, 3), "max_m": round(vmax, 3),
        "content_fnv64": "%016x" % content,
        "datum": ("NAVD88 (BlueTopo native EPSG:5103; eHydro converted MLLW->NAVD88 "
                  "-1.400 m via CO-OPS station ladder)"),
        "crs_note": "NAD83 vs WGS84 ~1 m: documented, not corrected (M6l precedent)",
        "coverage": round(cover, 4),
        "cells_bluetopo": n_bt, "cells_ehydro": n_eh,
        "surveys": surveys,
        "validation_vs_cudem": rep,
    }
    with open(os.path.join(OUT_DIR, f"{NAME}.json"), "w", encoding="utf-8") as f:
        json.dump(meta, f, indent=1)
    log(f"sidecar -> {NAME}.json (content fnv64 {meta['content_fnv64']})")

    write_previews(grid, src)
    log("done")
    return 0


if __name__ == "__main__":
    sys.exit(main())
