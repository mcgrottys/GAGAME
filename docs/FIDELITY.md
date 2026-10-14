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

(none yet)
