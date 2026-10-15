# Fidelity, sources and composites: the data a region wants, where it comes from, and the one product

Started 2026-10-13. Four kinds of file, four readers. Nothing in them names a place: a region is a cell
of the engine's lattice or a box, and what is there is what the data says.

| file | type | written by | read by |
|---|---|---|---|
| `*.gafid` | the fidelity map: the finest grain each region wants | the painter (demo launcher) | every harvester; later the engine's residency |
| `harvest/sources.limits.json` | what each data source allows: rates, sizes, days | by hand, from each source's documentation | `Polite` (every harvester's fetch) |
| `*.composite.json` | one product of one data layer, made of many sources | a composite harvester | the engine (a scene's `sources` entry `composite`) |
| `harvest/sources/<layer>/*.json` | one source each: its reader, server, variable map, units, datum (section 7) | by hand | the composite harvesters, through `Core/SourceFactory.cs` |

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

**Painting tools** (`harvest/Gagame.Harvest/Core/FidelityRegions.cs`, shared by the painter and
`harvest fidelity`). A box or a brush circle names cells of one level: an eighth of the region's
extent (a box's shorter side, a brush's diameter), at most 4096 cells a region, a coarser level
past that (the outline coarsens, the grain asked for does not). A box takes every cell its lat/lon
box meets, so the box is covered whole; a brush takes the cells whose centres lie within its
radius. A brush's ERASE and `erase-box` are `Clear`: the region returns to the layer's default, a
coarser painted cell reaching over it is cut into the children outside it (each keeping its
grain). `FinestIn` walks the lattice down from the six faces carrying the grain the deepest
painted cell above hands down, so a coarse cell painted inside a fine one answers coarse.

## 2. The sources' limits, `harvest/sources.limits.json`

One entry per data source a harvester fetches from, with the limits its own documentation
states and the page that states them. Where a source publishes none, the entry says so and
takes the conservative default, never a guess presented as policy. `Polite` reads it (the file
`GAGAME_SOURCE_LIMITS` names, else `harvest/sources.limits.json` in the checkout: tracked in git,
which `data/` is not); the constants that were in `Polite.cs` became the file's defaults, and
`harvest limits --check` holds the shipped file to them over every host a harvester names.

```json
{
  "_readme": "...",
  "default": {"minIntervalS": 1.1, "budgetRequests": 2000, "budgetBytes": 8e9,
              "backoffS": 30, "bigBytes": 5e7},
  "sources": {
    "<id>": {
      "title": "NOAA CO-OPS Data API",
      "hosts": ["api.tidesandcurrents.noaa.gov"],
      "minIntervalS": 1.1, "perSecond": 0, "perMinute": 0, "perHour": 0, "perDay": 0, "dayBytes": 0,
      "maxSpan": "6-minute 1 month; hourly 1 year; ...",
      "documented": "https://...", "quote": "under 15 words from the page, if it states a limit",
      "stated": true, "note": "...",
      "banned": false
    }
  }
}
```

- `default`: the pace for a host no entry names; the request and byte budget of one ledger
  (`GAGAME_LEDGER`, all hosts together, as before); the back-off before the one retry when a host
  names no Retry-After; the largest unplanned GET.
- A host takes the most specific entry naming it: equal or a subdomain (`s3.amazonaws.com` holds
  every bucket), or a pattern ending in `.` found anywhere in host + `.` (the Google fragments).
  A ban holds whatever more specific entry also names the host.
- A source's pace is ONE law: the least gap is the slowest of its `minIntervalS` (ours, default
  the default's) and every stated rate turned into a gap (`perMinute` 120 is 0.5 s, `perDay` 10000
  is 8.64 s). All the hosts of a source share the pace (NCEP's 120 hits a minute is summed over its
  hosts). `dayBytes` is checked against the ledger's last 24 hours of that source.
- One request in flight and one retry are laws of `Polite`, not numbers of a source; the earlier
  draft's `maxConcurrent`, `maxRetries` and per-day budget names are gone (the budget was always
  a ledger's, not a day's).
- `stated: false` means the source publishes no limit and the numbers are ours. `banned: true`
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
  A tier is written in **parts**, one folder (`bed/<tier>/p<k>`) for each group of tiles that touch,
  each a `tiers` entry of its own with its `box`: the engine takes a set's footprint as ONE box, and
  one tier read whole from Florida to Maine claimed every tile of the coast between (2026-10-13:
  the Merrimack never settled). The engine reads a height's box past two texels a side by the
  windows' summed-area tables (`RasterFileSource::WindowSat`), so a coarse ask of a fine part costs
  the windows it touches.
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
  water.bed.composite.json   tiers[] (one a part): grain_m, step_deg, folder, box, match "*.tif", over, inputs, decimation,
                             datum, vertical_epsg, unit, nodata, offset_m, offset_measured, source_files, stats;
                             datum_separation, phases_s, later
  bed/<source>_<grain>m/p<k>/*.tif  float32 GeoTIFF tiles (section 3), named by their NW corner (n42x8125_w70x8125.tif)
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

## 7. What is stored: the storage index and view

`harvest storage` (and the painter's Storage tab) keeps an index of the bytes ON DISK, per
source, parameter, store and cell of this lattice: `%LOCALAPPDATA%\GAGAME\storage-index.bin`
(GASTOR04), `harvest/Gagame.Harvest/Core/StorageIndex.cs`. It reads names, sizes and
directories, never a tile's payload, and a rescan reads only the folders whose own write time
moved. `harvest storage check` is its gate (a planted tree, archive and composite of known
sizes, cell for cell).

**Read** (as the disk has them): each loose tile's size and name `f<face>_m<mip>_x<x>_y<y>[_<key>]`
under `cache/trees/<name>.<id>/<tag>/` (TileTree.h) and `cache/composed/<channel>/<tag>/`; each
archive's directory (`<tag>.gaa`, TileArchive.h: the 16-byte header and the 24-byte records,
key = face << 61 | mip << 56 | y << 28 | x, and each record's size; `.gaa.stale` is the store
`archive.stale`); the composites' tier tiles' sizes and corner names, their point files' sizes
and points, and `coverage[]` as written; `.void` markers are counted, with no bytes. A source is
a tree folder `<name>.<id>` (the id is the hash of the node's identity, name and structure; it is
not invertible, so the folder name IS the identity), a composed channel, or a composite input;
the parameter is the tree's node name, the channel, or the composite's layer.

**Derived** (and how): a cube tile's cell is exact when its width is known -- the realization
`cube<K>k` has a face of K x 1024 texels and a tile 128 high, so tile (f, m, x, y) IS the cell
(f, log2(K x 1024) - 7 - m, x, y) -- but the width (128 for RGBA8 and Raw4, 256 for Half and FloatW)
is the tenant's and is not written in the folder: a 256 KB tile is FloatW; a tile at an x or a
mip a 256-wide tile cannot reach is 128; at the mip where one 256-wide tile spans a face, y = 1
without x = 1 is 256; otherwise 128 is ASSUMED (a 256-wide tile then lands in a cell half its
width). A 256-wide tile's bytes are split evenly over its two cells. A window tile
(`window_z<z>_<orgX>_<orgY>`, Web-Mercator) goes to the cell holding its centre, at the level of
its own ground size. A composite tier tile's size is the least spacing of its folder's corner
names (one tile: the tier box). A point file's bytes are split evenly over its points (a JSON
file has no size per point). Cells deeper than level 12 (~2.4 km) add to their level-12
ancestor: on the owner's cache level 14 kept 1.47 M aggregates in 55 MB, level 12 keeps
890 k in 33 MB.

**Who answers here.** The only weights the stores record are presence: a tree tile exists where
that source painted, and a composite's `coverage[]` says which input answers a box at what grain
(and whether as a gap fill). The view shows those as they are -- the bytes per cell and the
coverage boxes -- and computes no weight the engine does not store.

**Measured** (2026-10-14, the owner's machine, cache on D:): 1,682 folders and archives, 13.2 M
directory entries (7.2 M tiles, 3.6 M voids, 2.3 M other markers), 561.7 GB in cells and 9.5 MB
(archive directories) outside them. The first scan took 14.5 s; a rebuild with the file system's
cache warm 3.6 s; a rescan with nothing changed 0.1-0.2 s plus 0.3-0.5 s to load the 33 MB index.

### The cache lens on the minimap

The minimap's third button (left of Travel, a 3 x 3 grid of squares, bright while on and faint while off) paints that
eye -- and only that eye; the first eye's pipeline and rows are untouched -- as a heat map of this
index: where tiles are cached offline and how fine. The engine reads `storage-index.bin` itself
(`src/compose/CacheIndex.{h,cpp}`, a reader of the layout `StorageIndex.Save` writes) the first time
the toggle turns on in a session, and never again in that session: run `harvest storage` to refresh
the index, then restart. Every entry, of every source, store and parameter (derived ones too), is
reduced by one law to the lattice at level 10 (1024 cells a face side, ~9.8 km): an entry of level
L covers the level-10 cells under it, or the one above it when L > 10; each cell keeps the finest
level over it and the bytes over it (an entry's bytes spread evenly over the cells it covers). The
six faces go to the GPU stacked in one 1024 x 6144 R8G8 texture, read in `shaders/CacheLens.hlsl` by
the pixel's planet direction through the lattice's own face and uv (HpCubeFace), one Load a pixel.

**The colours** are the finest level: dark navy where nothing is stored, blue for the coarsest
levels, through cyan (6) and yellow (9) to red (11) and white (12, ~2.4 km cells -- anything finer,
centimetre imagery included, is counted in its level-12 cell, so white means "level 12 or finer").
The bytes brighten a cell by up to 30 % (log2 of the bytes, over 0..2^40). The day/night lambert
of the meshlet tint is kept, so the terminator stays readable. The key, bottom left of the map,
above the altitude, reads left to right: nothing, then levels 0 to 12. No index on disk: the lens
paints grey and the log says so once.

**The instrument.** `GAGAME_MINIMAP_CACHE=1` starts a run with the toggle on (read once, in
`FrameLoop::MinimapStep`), so a headless still can show it; the log line `[cachelens]` gives the
file, folders, entries, cells lit and the milliseconds the read took.

## 8. Sources by configuration (`harvest/sources/<layer>/<id>.json`)

Started 2026-10-14 (Mark: "a factory design pattern taking buoy data from different sources ... ERDDAP
for a semi standard to follow, with configs allowing us to adjust to the schema different researchers
use. same with the relief"). A source is ONE file; adding a buoy network or a DEM a reader already
reads is writing a config, not C#. `Core/SourceFactory.cs` finds the reader a config's `kind` names
(the assembly's `[SourceKind]` classes) and makes it; `compose-seastate` and `compose-bed` take every
config of their layer, or the ids `--sources a,b` lists, ranked.

### 8.1 ERDDAP, the semi-standard

ERDDAP (NOAA NMFS SWFSC ERD) puts one request grammar and one metadata model in front of whatever
files a data centre holds. Read 2026-10-14 from the servers' own documentation pages
(https://coastwatch.pfeg.noaa.gov/erddap/tabledap/documentation.html,
https://coastwatch.pfeg.noaa.gov/erddap/griddap/documentation.html):

- **tabledap** (tables: stations, profiles, trajectories): `<server>/tabledap/<datasetID>.<type>?<vars>&<constraints>`;
  the variables comma-separated, each constraint `&var<op>value` with `= != =~ < <= > >=`, strings
  in double quotes, times ISO 8601 (`time>=2026-10-13T00:00:00Z`), seconds since 1970, or relative
  (`now-1day`, `max(time)-10minutes`); server-side functions `orderByMax("station,time")` (the latest
  row of each station), `distinct()`, `orderBy...`. The query must be percent-encoded: Tomcat answers
  400 to a bare `"` or `>` (measured).
- **griddap** (grids): `<server>/griddap/<datasetID>.<type>?<var>[(start):stride:(stop)][...]`, one
  bracket per axis in the variable's axis order; `(value)` takes the nearest axis value, a bare number
  is an index; since v2.17 a high/low pair in the "wrong" order is swapped. A very large request
  "will usually fail": ask in pieces.
- **Types**: `.csv` (row 1 the names, row 2 the units, `NaN` where missing, ISO times), `.csvp`,
  `.json` (`table.columnNames/columnUnits/rows`, `null` where missing), `.nc` (classic netCDF-3:
  `Readers/NetCdf3.cs` reads it), `.ncCF` (CF Discrete Sampling Geometries), `.geotif` (griddap,
  lon/lat only, all longitudes on one side of 180).
- **An empty answer is HTTP 404** with "Your query produced no matching results"; an unknown dataset is
  also 404 ("Currently unknown datasetID", seen while CoastWatch restarted). `Polite` now keeps the
  start of an error's body (`HttpError.Body`) so the two are told apart: only the first is kept as an
  empty first copy, and only for a box's latest rows (a moment the server does not hold YET is not).
- **Metadata**: `<server>/info/<datasetID>/index.csv` (or `.json`): one row per variable (`variable`)
  or grid axis (`dimension`), one per attribute. CF attributes per variable (`standard_name`, `units`,
  `_FillValue`, `positive`, `actual_range`), ACDD/CF globals (`cdm_data_type` Grid / TimeSeries /
  Point / Profile / Trajectory, `featureType`, `cdm_timeseries_variables` naming the station
  variables, `geospatial_lat/lon_min/max`, `time_coverage_start/end`, `license`, `infoUrl`,
  `institution`).
- **Discovery**: `<server>/tabledap/allDatasets.csv?datasetID,cdm_data_type,...&<constraints>` (every
  dataset of the server as a table, measured on upwell), `<server>/search/index.csv?searchFor=...`
  (free text), `<server>/search/advanced.csv?standard_name=...&minLon=...` (by CF name and box).
- **Limits**: no ERDDAP page states a number; each documentation page asks to "run just one script or
  curl command at a time" (one request in flight is already `Polite`'s law). Administrators throttle
  per IP in their own setup, unpublished. Entries `erddap.coastwatch` and `erddap.marine-ie` in
  `harvest/sources.limits.json` (`stated: false`); `harvest limits --check` now holds every host a
  source config names to an entry of its own.

**Servers** (each answered 2026-10-14 unless said):

| server | holds (for the engine) | note |
|---|---|---|
| https://coastwatch.pfeg.noaa.gov/erddap (CoastWatch West, ERD) | `cwwcNDBCMet` NDBC standard met 1970-present (TimeSeries); `GEBCO_2020` 15"; `srtm15plus`, `srtm30plus*`, `etopo180` / `etopo1_*` (ETOPO1 60"), `etopo5` | down (503, then "unknown datasetID") for part of the morning; its mirror https://upwell.pfeg.noaa.gov/erddap answers info pages but REDIRECTS data requests (302) back to coastwatch.pfeg |
| https://erddap.marine.ie/erddap (Marine Institute, Ireland) | `IWBNetwork` Irish Weather Buoy Network (TimeSeries, CC BY 4.0) | wind in knots, Hs labelled `sea_surface_swell_wave_significant_height` |
| https://erddap.secoora.org/erddap (SECOORA) | NDBC and regional buoys, ONE dataset a station (`gov-ndbc-41013`, `41159-lej3wave-...`) | |
| https://erddap.sensors.ioos.us/erddap (IOOS sensor map) | every IOOS RA's stations, one dataset a station | |
| https://data.neracoos.org/erddap (NERACOOS; www.neracoos.org/erddap redirects there) | Gulf of Maine buoys (A01 = NDBC 44029 ...) | not queried |
| https://erddap.cencoos.org/erddap, https://pae-paha.pacioos.hawaii.edu/erddap | CeNCOOS, PacIOOS | pae-paha did not answer the first probe |
| https://erddap.dataexplorer.oceanobservatories.org/erddap (OOI) | OOI arrays | not queried |
| https://coastwatch.noaa.gov/erddap, https://polarwatch.noaa.gov/erddap, https://www.ncei.noaa.gov/erddap | CoastWatch central, PolarWatch, NCEI | no wave search hit on NCEI |
| https://erddap.emodnet.eu/erddap (EMODnet Physics) | European in-situ | no wave search hit |

**What the metadata gets wrong** (each seen on the servers above; why a config DECLARES what it must):

- CoastWatch `cwwcNDBCMet` `MWD`: `standard_name` `sea_surface_wave_to_direction`, while NDBC defines
  MWD as the direction the waves COME FROM. Trusting the name turns every wave 180 degrees; the config
  declares `"direction": "from"` and the reader logs the contradiction. The agreement gate (7.3) holds
  the declaration right (41122: 59 deg in both).
- `DPD` and `APD` share `standard_name` `sea_surface_swell_wave_period`: a period is NAMED in a config,
  never matched (the resolver refuses a standard_name two variables hold).
- `GEBCO_2020` `elevation`: `standard_name` `height_above_reference_ellipsoid`; GEBCO states heights
  relative to mean sea level. The config declares `"datum": "MSL"` (EPSG:5714).
- IWBN `WaveHeight`: `sea_surface_swell_wave_significant_height` for the total Hs; `WindSpeed` in
  `knots`; `WindDirection` in `degrees true` (a space) where NDBC writes `degrees_true`. Its
  `WavePeriod` is taken as the average period: an assumption, not checked against the network's docs.

### 8.2 The config

Common keys (both layers): `id` (unique), `title`, `label` (the phrase a product's text names it by),
`kind` (the reader), `rank` (lower answers first: for the same station, or at equal native step),
`server`, `dataset`, `url` (the input's url), `license`, `cite`, `coverage` [lon0, lat0, lon1, lat1]
(a box outside it is never asked), `variables` {quantity: "name" | {`name` or `standard_name`, `units`,
`direction` "from"|"toward", `north` "true", `file`}}. A quantity matched by `standard_name` is the ONE
variable holding it; units the config leaves out are the server's; a direction's way is the config's,
else its standard_name's (`*_from_direction` / `*_to_direction`); a magnetic north is refused (no
declination model). `Units` (`Core/SourceConfig.cs`) turns a unit into the engine's by one factor (m,
ft, cm; s; m/s, knots, km/h, mph; degrees true; m2/Hz) and refuses one it does not know; a factor of 1
leaves the value untouched, so a ported source is bit-identical.

Buoys (`harvest/sources/buoys`): the quantities `time station lat lon name wdir wspd hs dpd apd mwd
spectrum` (sim/SeaState.cpp's buoy keys), plus per kind:
- `ndbc-realtime2`: `cache` (the first-copy folder); `time` is the five columns year month day hour
  minute; each quantity names its column in the file's header and its `file` (`txt`, `data_spec`).
- `erddap-tabledap`: `cache`, `recent` (the `time>=` of a box's latest-rows request, `now-1day`).

```json
{ "id": "erddap.coastwatch.ndbcmet", "kind": "erddap-tabledap", "rank": 1,
  "server": "https://coastwatch.pfeg.noaa.gov/erddap", "dataset": "cwwcNDBCMet",
  "cache": "cache/erddap/cwwcNDBCMet", "recent": "now-1day",
  "variables": { "station": "station", "lat": {"standard_name": "latitude"}, "lon": {"standard_name": "longitude"},
    "time": {"standard_name": "time"}, "wdir": {"standard_name": "wind_from_direction"},
    "wspd": {"standard_name": "wind_speed"}, "hs": {"standard_name": "sea_surface_wave_significant_height"},
    "dpd": "DPD", "apd": "APD", "mwd": {"name": "MWD", "direction": "from"} } }
```

Relief (`harvest/sources/relief`): also `short` (the tier folder's stem), `datum` and `vertical_epsg`
(the GeoKey its tiers carry), `step_arcsec` (15, or "1/9" for the division the step is),
`variables.elevation` (its units; a kind that copies pixels refuses anything but metres), plus per kind:
- `cudem-index`: `bucket`, `prefix`, `index`, `index_cache`, `cache` (path templates of `{datacache}`,
  `{region}`, `{name}`: the first existing is read, the first is where a fetch lands), `listing_cache`.
- `tiled-geotiff`: `tile_deg` [w, h], `name` (`{ns}{lat1:00}{ew}{lon0:000}` of the NW corner), `fetch`
  (URL template), `cache`.
- `erddap-griddap`: `chunk_deg` (one request a chunk of the tile lattice, a whole number of steps),
  `name`, `cache`, `metadata_cache`; `variables.lat/lon` name the axes. A chunk is the pixel centres
  inside it, its axes checked against the lattice of `step_arcsec` to a thousandth of a step (a grid
  off it is refused, never resampled), written beside its `.nc` as the composites' float32 GeoTIFF.

```json
{ "id": "erddap.coastwatch.gebco2020", "kind": "erddap-griddap", "rank": 1, "short": "gebco15",
  "server": "https://coastwatch.pfeg.noaa.gov/erddap", "dataset": "GEBCO_2020",
  "datum": "MSL", "vertical_epsg": 5714, "step_arcsec": 15, "chunk_deg": 1,
  "variables": { "elevation": {"name": "elevation", "units": "m"},
    "lat": {"standard_name": "latitude"}, "lon": {"standard_name": "longitude"} },
  "name": "GEBCO_2020_{ns}{lat1:00}{ew}{lon0:000}.tif", "cache": ["{datacache}/erddap/GEBCO_2020/{name}"],
  "metadata_cache": "{datacache}/erddap/GEBCO_2020" }
```

**The interfaces** (`Core/SourceFactory.cs`): `IObservationSource { Config; Stations(Box);
Observe(Station, double? at = null); FetchedUtc }` and `IReliefSource { Config; Meets(Box) ->
List<ReliefFile> }`, a `ReliefFile` a GeoTIFF on disk once made (`Dest`, `Cell`, `Url`, `Size`,
`SizeOf`, `Make`: a subset service's fetch-and-write, null for a planned download).

**The laws the factory adds.** Buoys: sources by rank; a station id a better-ranked source answered is
not asked of the next; the product names (`source`, `inputs`) only the sources that answered. Relief:
of the sources of one native step holding files in a region the lowest rank answers; at equal grain the
better rank lies over; the PRODUCT's datum is the finest source's; each other datum's offset is
measured against that datum's finest source and its tiers carry it; `datum_separation` is that one
object when the product has one other datum, an object of them by datum when it has several.

### 8.3 Measured (2026-10-14, coast3_3to10m)

- **Ported, byte for byte.** `compose-bed` through the factory (all five configs, and the four ported
  ones by `--sources`) against the pre-factory binary on the same map: 162 of 162 tier GeoTIFFs equal
  by SHA-256; `water.bed.composite.json` equal in every value but `generated_utc`, `stats`, `phases_s`
  and one text, `datum_separation.what` (now spelled from the configs' labels: "CUDEM 1/9" or CUDEM
  1/3" ... covered by them" for "CUDEM 1/9" or 1/3" ... covered by CUDEM"). `compose-seastate`
  (`noaa.ndbc` alone, and all three buoy configs) equal in every value but `generated_utc` and
  `stats`. `compose-check` on the ported product: 6 of 6 gates (with `--against-sea` haulover and
  chesapeake).
- **ERDDAP tabledap against NDBC realtime2** (`harvest sources agree-buoys --a noaa.ndbc --b
  erddap.coastwatch.ndbcmet`, coast3 + a Mass Bay box): 13 stations, 18 of 18 quantities held by both
  equal (worst difference 0 in every quantity; tolerance half NDBC's last printed digit), 14 held by
  one only: CoastWatch has no row for 44013 and 44098 at their first copies' 2026-08-28 lines (older
  than the realtime window), for 44087 at :26, nor for 44029's line 2 hours old (its
  time_coverage_end lagged). Only one wave buoy (41122) compared its waves. The 13 latest-row requests
  of a cold `compose-seastate` with the ERDDAP source took 491 s (about 35 s each, the server's own
  time); warm, 0.2 s.
- **ERDDAP griddap against ETOPO 2022 15"** (`agree-relief --a erddap.coastwatch.gebco2020 --b
  noaa.etopo2022.15s`, coast3 + the slope box -74.5,36.5,-73.5,37.5; 12 chunks): 82,751 pixels,
  centres equal (0 px); GEBCO minus ETOPO: deep (<= -200 m) median +2.35 m, |d| median 2.70 m, p90
  9.5 m, max 166 m; shelf median +1.32 m, |d| p90 6.0 m; land median +4.93 m. The gate, stated before
  the run (one lattice, deep median |d| <= 5 m): pass.
- **A bed with the griddap tier in place of ETOPO 15"** (`--sources noaa.cudem.ninth,noaa.cudem.third,
  erddap.coastwatch.gebco2020,noaa.etopo2022.60s`): tiers cudem19 3.43 / 13.73 m, gebco15 463 m (8
  files), etopo60; `compose-check` 4 of 4 (geotiff, bed, holes, gap). NAVD88 - MSL measured against
  GEBCO: -0.967 m (GEBCO minus CUDEM median +0.967, MAD 0.673, p05 -0.84, p95 +4.65 over 7,933 cells):
  GEBCO 2020 near these coasts is not CUDEM, where ETOPO 2022 is (0.000).
- The session's harvester fetches: 41 requests, 2.1 MB (40 to coastwatch.pfeg, one NDBC file).

**Not yet**: the IOOS regional servers publish one dataset a station, and a config is one dataset, so
a network there is a config a station, or a kind that lists its stations by `allDatasets` or
`search/advanced` by standard_name (not built). No spectra from ERDDAP (cwwcNDBCMet has none). The
per-tile keys of `datum_separation` keep their old names (`cudem_tile`, `median_etopo_minus_cudem_m`)
so the product's keys are unchanged, though the reference may now be another source.
