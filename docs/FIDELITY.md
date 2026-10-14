# Fidelity, sources and composites: the data a region wants, where it comes from, and the one product

Started 2026-10-13. Three files, three readers. Nothing in them names a place: a region is a cell
of the engine's lattice or a box, and what is there is what the data says.

| file | type | written by | read by |
|---|---|---|---|
| `*.gafid` | the fidelity map: the finest grain each region wants | the painter (demo launcher) | every harvester; later the engine's residency |
| `data/sources.limits.json` | what each data source allows: rates, sizes, days | by hand, from each source's documentation | `Polite` (every harvester's fetch) |
| `*.composite.json` | one product of one data layer, made of many sources | a composite harvester | the engine (a scene's `sources` entry `composite`) |

## 1. The fidelity map, `.gafid` (GAFID01)

**What it says.** For each region of the planet, the finest grain, in metres on the ground,
that the region WANTS: 0.001 (a millimetre) to 100,000 (a hundred kilometres). A harvester asks
a region for no finer data than this, and takes the finest it can get down to it. A region
nobody painted wants its layer's default, and the default `+Infinity` means "the coarsest
source is enough": the global base.

**The lattice is the engine's.** A cell is `(face, level, x, y)` on the quad-sphere of
`src/core/Lattice.cpp` (`ComposeCubeDir`, `CubeFaceOfDir`; planet frame x = cos lat cos lon,
y = sin lat, z = cos lat sin lon): level L cuts a face into 2^L x 2^L cells, the cell holds
u in [x, x+1) / 2^L and v likewise. A cell's edge at its face's centre is (pi/2) R / 2^L: level 0
is 10,008 km, level 10 is 9.8 km, level 20 is 9.5 m, level 31 is 4.7 mm. The cell says WHERE;
its value says HOW FINE. The two are independent: a 100 km cell may want 1 m data.

**Reading.** The grain at a point is the value of the DEEPEST painted cell holding it, else the
layer's default. The grain a box wants is the least over the cells it touches (and the default
where it reaches past them). `harvest/Gagame.Harvest/Core/Fidelity.cs` is the reference reader
and writer; the engine's reader, when it comes, must answer equal at every cell centre.

**Painting.** Painting a cell sets the region whole: its descendants go, its ancestors stay
(the parts of them outside it keep what they had). Erasing a cell removes it and its
descendants, and the region falls back to what holds it above.

**Layers.** A file holds one or more layers, each a name and its own cells. `*` is every layer
the file does not name; for now the painter writes `*` only. A later layer named `water.bed`,
`colour`, `tides` ... overrides `*` for its readers alone.

**Bytes** (little-endian):

```
char[8]  "GAFID01\0"
f64      planet radius, metres (6371000 for the Earth: the cell edges above are measured on it)
u32      layer count
per layer:
  u16    name length, then the name in UTF-8
  f32    default grain, metres (+Inf = the coarsest source is enough)
  u32    cell count
  per cell, sorted by (face, level, y, x), 16 bytes:
    u8 face (0..5), u8 level (0..31), u16 0, u32 x, u32 y, f32 grain metres
```

A cell off the lattice (face > 5, level > 31, x or y >= 2^level) refuses the file.

## 2. The sources' limits, `data/sources.limits.json`

One entry per data source a harvester fetches from, with the limits its own documentation
states and the page that states them. Where a source publishes none, the entry says so and
takes the conservative default, never a guess presented as policy. `Polite` reads it; the
constants that were in `Polite.cs` become the file's defaults.

```json
{
  "_readme": "...",
  "default": {"minIntervalS": 1.1, "maxConcurrent": 1, "dayRequests": 2000, "dayBytes": 8e9,
              "retryAfterS": 30, "maxRetries": 1, "bigBytes": 5e7},
  "sources": {
    "<id>": {
      "title": "NOAA CO-OPS Data API",
      "hosts": ["api.tidesandcurrents.noaa.gov"],
      "minIntervalS": 1.1, "maxConcurrent": 1, "dayRequests": ..., "dayBytes": ...,
      "maxSpan": "31 days per request for 6-minute data",
      "documented": "https://...", "quote": "under 15 words from the page, if it states a limit",
      "stated": true,
      "banned": false
    }
  }
}
```

`stated: false` means the source publishes no limit and the numbers are ours. `banned: true`
means no harvester may fetch from the host (Google's hosts: the engine's own day ledger,
`core/DayLedger.h`, governs the Google tile source, not the harvesters).

## 3. The composite, `*.composite.json` (GACOMP01)

**Many sources in, one product out.** A composite harvester takes every source of one data
layer, ranks them by grain and by quality, and writes ONE product: for each region the finest
source that has data there, down to the grain the fidelity map wants, its gaps filled from the
next source, every value converted to the one unit and datum the product declares. The engine
reads the product, never the sources, and does not know how many there were.

```json
{
  "format": "GACOMP01",
  "layer": "water.bed",
  "kind": "height | tides | currents | seastate | river",
  "generated_utc": "...",
  "fidelity": {"file": "data/fidelity/world.gafid", "fnv64": "..."},
  "unit": "m", "datum": "NAVD88",
  "tiers": [
    {"grain_m": 3.4, "folder": "bed/t3", "match": "*.tif", "over": 3, "inputs": ["noaa.cudem.ninth"]},
    {"grain_m": 460, "folder": "bed/t0", "match": "*.tif", "over": 0, "inputs": ["noaa.etopo2022.15s"]}
  ],
  "file": "tides/stations.json",
  "inputs": [{"id": "noaa.cudem.ninth", "title": "...", "grain_m": 3.4, "url": "...", "license": "...",
              "cite": "...", "datum": "NAVD88", "converted": "none | +offset m | model"}],
  "coverage": [{"box": [lon0, lat0, lon1, lat1], "grain_m": 3.4, "input": "noaa.cudem.ninth"}],
  "stats": {"seconds": 0, "bytes_in": 0, "bytes_out": 0, "requests": 0, "peak_mb": 0}
}
```

- A **raster** layer (`kind: height`) is `tiers`: each tier one grain, a folder of GeoTIFFs in
  EPSG:4326 (one float32 band, nodata declared), every tile of a tier the same grain. A tier is
  one raster set as the engine already reads a folder (`compose/RasterFileSource.h`: one CRS and
  one grain a set); the engine stacks the tiers by `over`, finer over coarser, and its tree folds
  and magnifies between them (HIERARCHY 4.20). Where a finer tier has nodata the coarser shows.
- A **point and field** layer (`tides`, `currents`, `seastate`, `river`) is one `file` in the
  engine's existing schema for that kind, holding every region at once: all the stations, all
  the fields, all the forecast points, each carrying its own position and box. The engine picks
  by position, never by a place's name.
- `coverage` is what came from where, for the owner and for the license page; `stats` is what
  it cost, for the scalability measurements (section 4).

## 4. Scalability against performance

Measured by the composite harvesters, written into each product's `stats` and summarized in
section 5 when measured: seconds, bytes fetched, bytes written, peak memory, against the painted
area and grain. No number appears here until it is measured.

## 5. Measurements

Measured 2026-10-13/14 on the owner's machine (16 logical CPUs, the caches on D:), C#
(`harvest compose-bed`), one process a run. The maps are `harvest/testdata/*.gafid`; area is the
sum of the painted cells' lat/lon boxes. "Cold" is the first run (what it fetched), "warm" a rerun
over the same map (everything cached: the first copy is read, nothing fetched). MB are 10^6 bytes;
MB in is source bytes read from disk, MB out the tiers written (Deflate, level Optimal); peak is
the process's peak working set.

| map | area | grain | run | seconds | MB in | MB out | peak MB | requests (MB fetched) |
|---|---|---|---|---|---|---|---|---|
| box5km_3m: one ~5 x 5 km box (6 cells, level 11) | 101 km2 | 3 m | cold | 3.1 | 658 | 618 * | 1,771 | 1 (0.015) |
| | | | warm | 4.4 | 658 | 539 | 2,178 | 0 |
| coast3_3to10m: three coastal boxes (13 cells, level 9) | 4,259 km2 | 3, 3, 10 m | cold | 79.9 | 3,740 | 1,501 * | 2,908 | 12 (1,107) |
| | | | warm | 25.7 | 3,740 | 1,333 | 2,476 | 0 |
| natlantic_463m: the North Atlantic, 80 W..0, 0..60 N (695 cells, level 5) | 69.6 M km2 | 463 m | cold | 267.0 | 1,222 | 1,094 | 1,920 | 66 (678) |
| | | | warm | 10.7 | 1,222 | 1,094 | 2,374 | 0 |
| globe_base: nothing painted, the globe at the base grain | 510 M km2 | +Inf (60") | warm | 3.4 | 466 | 468 | 2,354 | 0 |

\* the cold runs of the first two maps wrote with Deflate level Fastest (618 and 1,501 MB); the
level was then set to Optimal: 13 % smaller (539 vs 618 MB on the same map) for about a second.
Every map writes the global 60" base (72 files, 466 MB in, 468 MB out, 3.2 s): it is most of
the bytes of the small maps, so the per-tier lines in each product's `tiers[].stats` are the
honest cost of a painted region.

**Where the time goes** (`phases_s` in each product, thread-seconds summed over the workers):
decoding the sources (Deflate + the float predictor, the repo's GeoTiff reader) runs 67-84 MB/s a
thread; encoding (predictor + Deflate Optimal) costs about as much again; the block mean of a
decimated tier is under 1 s; the file loads (one at a time) 0.3-2.6 s. Warm, the run is
CPU-bound: the globe base is 233 Mpx (933 MB of floats) in 3.4 s of wall clock against 45
thread-seconds, 13x on 16 threads, so it is parallelized where it is CPU-bound (the output tiles
of a source file in parallel, the 256 x 256 blocks of an output file in parallel). Cold, it is
fetch-bound: the 1,107 MB of CUDEM came at 20-30 MB/s one file at a time (Polite: one request in
flight, >= 1.1 s between requests). The decimated tier is the costly one per output byte: the
Chesapeake region at 10 m reads 1,678 MB of 1/9" source for 96 MB of 13.7 m tier (15 s). Peak
memory is set by the largest source file held whole (the 466 MB 60" file, the 240 MB CUDEM
tiles) plus the windows in flight, 1.8-3.0 GB; it does not grow with the painted area.

**C# or C++.** C# stayed: the hot loop is Deflate (System.IO.Compression is zlib-ng underneath)
and a byte shuffle, at 67-84 MB/s a thread, and the warm runs are seconds; the cold runs are
bounded by the polite fetch, which no language changes. `harvester/cpp` (roads and bridges) has no
raster path to reuse.

The other layers, over coast3_3to10m (cold first, warm after): tides 7.1 s / 4 requests, then
2.5 s / 0 (21 fitted stations, 17 EOT20 model stations); currents 73 stations, 8
fields, 1 ADCP buoy: 108 CO-OPS requests and 22 to the OFS bucket (one 171 MB GoMOFS surface
file, 18 datum grids, 339 MB, fetched once to learn each model's domain, 3 listings), then
18 s / 0; seastate 50 NOMADS requests at 2.5 s apart (265 s),
then 0.5 s / 0; river 14 requests (16 s; NWIS answers 404 for a box with no gauge, kept as an empty first
copy), then 0.08 s / 0. The whole session fetched 2.31 GB in 314 requests.

## 6. The water composites (`harvest compose-*`)

`harvest compose-<layer> --fidelity FILE.gafid --out data/composite/<name>/` (or `compose-water`
for all five, `compose-check --out ...` for the gates). Each reads the map's layer by name
(`water.bed`, `tides`, `currents`, `seastate`, `river`), else `*`. Every path in a product is
relative to the folder of its `.composite.json`.

```
data/composite/<name>/
  water.bed.composite.json   tiers[]: grain_m, step_deg, folder, match "*.tif", over, inputs, decimation,
                             datum, vertical_epsg, unit, nodata, offset_m, offset_measured, source_files, stats;
                             datum_separation, phases_s, later
  bed/<source>_<grain>m/*.tif  float32 GeoTIFF tiles (section 3), named by their NW corner (n42x8125_w70x8125.tif)
  tides.composite.json       file "tides/stations.json"
  tides/stations.json        the TideModel schema: year, epoch_unix, stations[] (+ kind "station"|"model", source)
  tides/official_<id>_<year>.f32
  currents.composite.json    file "currents/currents.json"
  currents/currents.json     tidal_stations[] (+ lat, lon, source), buoy_adcp (+ lat, lon, name), buoys_adcp[],
                             field (the finest of fields), fields[]: file, nx, ny, lon0, lat0, dlon, dlat, source,
                             valid_utc, model ("noaa.cbofs"), grain_m, wanted_grain_m, box, row0 "south", layout
  currents/<ofs>_r<region>_uv.f32   u plane then v plane, float32, -999 land
  seastate.composite.json    file "sea/seastate.json"
  sea/seastate.json          the SeaState schema at points[0] (cycle_unix, cycle_label, point_lat, point_lon, box,
                             hours) + points[]: id, lat, lon, grain_m, wanted_grain_m, cycle_unix, cycle_label, box,
                             hours + buoys{id: obs_unix, wdir, wspd, hs, dpd, apd, mwd, spectrum, lat, lon, name}
  river.composite.json       file "river/river.json"
  river/river.json           sites[]: id, name, discharge_cfs, discharge_cms, gage_ft, obs_iso, lat, lon, source
  check.json                 compose-check's gates
```

**The bed's law.** A source of native step a offers a 2^k (k = 0 its pixels copied, k > 0 the
mean of 2^k x 2^k of them where at least half are there); a region wanting g takes the least
offered grain >= g (a tie to the coarser native), and under it every coarser-native source at its
own least grain >= g; unpainted ground takes the coarsest source (ETOPO 60"). Sources: CUDEM 1/9"
(NODD, NAVD88), CUDEM 1/3" (NODD, NAVD88; it covers where the 1/9" does not, so in the three
coastal boxes it had no tile), ETOPO 2022 15" and 60" (NCEI, EGM2008). ETOPO 2022 over GEBCO 2024:
the same 15" grain, ETOPO already carries GEBCO in the open ocean and CUDEM on the US coast, it is
public domain and tiled per 15 degrees by HTTP (GEBCO is one 4 GB global netCDF), and its 60"
and one 15" tile were already cached. Skipped: NCEI BAG and USACE eHydro (point clouds and UTM
surveys per channel, a projection and a datum each; BlueTopo is 4 m UTM, no finer than CUDEM's
3.4 m in EPSG:4326 and a reprojection would resample it).

**The datum, measured.** The EGM2008 tiers carry `offset_m`, the median over 15" cells of ETOPO
minus the CUDEM area mean, cells wholly covered and flat within 1 m. Over coast3 (7,933 cells):
in the Chesapeake 2018/2019 tiles and the Massachusetts 2021 tiles ETOPO equals CUDEM to
0.000 +- 0.002 m (ETOPO 2022 carries CUDEM's numbers there as they are), in the Florida 2018
tiles ETOPO is +0.360 m (MAD 0.008). So the declared offset is 0.000 (median) with p05 -0.003
and p95 +0.336: the error of one offset is up to 0.36 m where ETOPO meets CUDEM, and unknown
away from them (NAVD88 is not defined outside North America; EGM2008 is a geoid, not local mean
sea level, which differs from it by up to a metre or two). A map with no CUDEM ground declares
0 and `offset_measured: false`.

**Tides.** CO-OPS harmonic stations inside the regions, fitted (`Tides.Fit`) to the earliest
year the cache holds; EOT20 model stations on the 1/8 deg lattice at stride ceil(g / 13.9 km)
where no fitted station stands within half a spacing. The model's phase is V0 + u - G (Doodson
arguments, Pugh's nodal terms): `tides.composite.json` `model_check` holds the instrument, V0 + u
against (fit phase + harcon lag) at every fitted station: M2/S2/N2 within 0.03 deg, K1 +0.5,
O1 -0.95 deg (it caught the diurnal quarter-turn signs inverted: 180 deg). The model's hourly
series against the official one: 0.07-0.20 m rms (the model's 14 constituents and its 14 km
node against a harbour gauge).

**The first copy** (Mark, 2026-10-13): a time-varying source (a forecast cycle, a buoy's line, a
prediction window, an OFS cycle, a gauge response) is read from the earliest copy the cache
holds and fetched only when there is none; each input records `valid_utc` and `fetched_utc`.
