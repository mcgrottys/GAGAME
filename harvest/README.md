# harvest

The harvesters in C#: a class library (`Gagame.Harvest`) and one console (`harvest`), the port
of `harvester/*.py` with the same outputs, cache layout, politeness rules and log lines, so the
data on a machine harvested by the Python is the data this writes.

```bat
dotnet run --project harvest\harvest -- --list
dotnet run --project harvest\harvest -- tides
dotnet run --project harvest\harvest -- bathy --window capeann
dotnet run --project harvest\harvest -- place --place haulover --final
```

`harvest <name> [args]` runs from the checkout it sits inside (or `--root DIR`); the big-data
drive is `GAGAME_DATACACHE` or `--datacache DIR` (default `D:\DataCache\GAGAME`). Every
harvester's own flags are the Python's (`--place --box --anchor --sea --radius-km`,
`--window`, `--skip-*`, `--eot20`, `--offline`, `--out`, ...).

## The factory

`IHarvester` is one product: its `Name` (the command), `Title`, `What`, the `Outputs` whose
presence says it is there, its `Sources`, the `Insert` it takes (a place and box, a named window,
files dropped in a folder, a file argument, flags) and `Run(ctx, args)`. `HarvesterFactory`
registers every implementation in the assembly and makes one by name; the demo's data list and
`harvest --list --json` both read it, so there is one catalog.

| name | writes | the Python |
|---|---|---|
| tides | data/tides/stations.json + official sidecars | harvest_tides.py |
| waves | data/sea/seastate.json | harvest_waves.py |
| currents | data/currents/currents.json (+ gomofs_uv.f32) | harvest_currents.py |
| river | data/river/river.json | harvest_river.py |
| bathy | data/bathy/<window>.json/.f32/preview | harvest_bathy.py |
| hqbathy | data/bathy/inlethq.* | harvest_hqbathy.py |
| refbathy | data/bathy/inlet15.* | harvest_refbathy.py |
| globe | data/globe/globe.json + the grids | harvest_globe.py |
| gis | data/gis/gis.json, the GSHHG bins, the land masks | harvest_gis.py |
| survey | data/gis/survey.json, cusp/nhd bins, the carved mask | harvest_survey.py |
| vectors | data/vectors/vectors.vpack | harvest_vectors.py |
| aerial | data/aerial/aerial.json + .rgb mip chains | harvest_aerial.py |
| overlay | data/overlay/overlay.json + .rgba | harvest_overlay.py |
| water | data/water/registry.json (+ eot20_*.rg32) | harvest_water.py |
| aton | data/gis/aton_ne.json | harvest_aton.py |
| route | data/gis/route_merrimack.json | harvest_route.py |
| buildings | <stem>.buildings.bin/.json | harvest_buildings.py |
| planet-buildings | <stem>.buildings.bin/.json/.idx | harvest_planet_buildings.py |
| overture | <stem>-<dataset>.buildings.* | harvest_overture.py |
| land-polygons | <out>.galand | harvest_land_polygons.py |
| place | data/{tides,currents,sea,bathy}/<place>/, data/places/<place>.json | harvest_place.py |
| compose-bed | <out>/water.bed.composite.json + bed/<tier>/*.tif | (new) |
| compose-tides | <out>/tides.composite.json + tides/stations.json | (new) |
| compose-currents | <out>/currents.composite.json + currents/currents.json + fields | (new) |
| compose-seastate | <out>/seastate.composite.json + sea/seastate.json | (new) |
| compose-river | <out>/river.composite.json + river/river.json | (new) |
| compose-water | all five above | (new) |
| compose-check | <out>/check.json (the composites' gates) | (new) |

## The composites: no places

`harvest compose-<layer> --fidelity FILE.gafid --out data/composite/<name>/` takes every source of
one water layer that has data in the regions a fidelity map paints, ranks them by grain, and
writes ONE product (GACOMP01, docs/FIDELITY.md sections 3 and 6): the bed as tiers of float32
GeoTIFFs (CUDEM 1/9" and 1/3" over ETOPO 2022 15" over 60"), tides as one stations.json (CO-OPS
fitted stations + EOT20 model stations), currents as one currents.json (CO-OPS stations, NDBC
ADCP buoys, a surface field per region and ROMS OFS), sea state as one seastate.json (GFS-Wave
points + NDBC buoys), rivers as one river.json (USGS gauges). No place name anywhere: a region is
a cell of the map. Every time-varying source is read from the FIRST copy the cache holds and is
never refetched for a newer one; each input records its valid and fetched time.

```bat
harvest compose-water --fidelity harvest\testdata\coast3_3to10m.gafid --out data\composite\coast3
harvest compose-check --out data\composite\coast3 --against-tides data\tides\haulover\stations.json
```

`harvest/testdata/` holds the four maps the measurements were made on (docs/FIDELITY.md 5):
`box5km_3m` (one ~5 km box at 3 m), `coast3_3to10m` (three ~20 km coastal boxes at 3, 3 and
10 m), `natlantic_463m` (the North Atlantic at 463 m), `globe_base` (nothing painted: the base).
`compose-check` is the gate: every tier file read back (GeoKeys, nodata, step), the stacked bed
against its own sources at a grid of points (equal to the bit where pixels are copied), no
unanswered point in a painted region, a gap planted in a copy of a finest tile filled by the next
tier with its own source's value, and stations, buoys and forecast points against older per-place
files, entry for entry.

## Packages and ports

Packages stand where the Python used a third-party library: **PureHDF** for the netCDF-4 /
HDF5 files (the NOS OFS nowcasts, the EOT20 grids; in place of hdf5.py and the netCDF4 module),
**Magick.NET** for the JP2 orthos (in place of Pillow + OpenJPEG), **Microsoft.Data.Sqlite** for
the GeoPackages (in place of sqlite3). The small stdlib decoders stay faithful ports under
`Readers/`: GeoTIFF float windows with the TechNote-3 floating-point predictor (LibTiff.NET is
libtiff 3.9 and lacks it), GRIB2 complex packing with the bitmap as NaN (NGrib would change the
decoded values), classic netCDF, OSM PBF with LocationsOnWays (OsmSharp drops those fields),
shapefile and DBF. `Core/Polite.cs` is the budgeted, ledgered fetcher of a place harvest;
`Core/Fetch.cs` the simple cache-first one every other harvester carried.

Two harvesters still shell out, as the Python did: `overture` to QGIS's `ogr2ogr`, and
`planet-buildings` expects osmium's output. `harvester/cpp` (the roads and bridges) is unchanged.

## The gate (2026-10-12)

Run on the owner's cache against the Python's own fresh output, byte for byte: the tide fits
(every coefficient and sidecar, 20 stations), the GSHHG bins and both land masks, the vpack,
the Merrimack CUDEM grid, the ETOPO 8192 and New England 15" grids, the water registry.
Two traps made the difference between "close" and "identical", and both are now in the code:

- `math.radians(x)` is ONE multiply by a precomputed constant; `x * Math.PI / 180` rounds twice
  and lands an ulp away, which re-orders the decimation of near-tie vertices (`Geo.DegToRad`).
- CPython 3.12+'s `sum()` of floats is Neumaier-compensated (`Vectors.NeumaierSum`).

Not yet gated here: the live-cycle products (waves, currents, river, aton, the globe's sea), the
big-drive products (survey, hqbathy, aerial, EOT20) and the building chain; their code is the
same port, but a run of each beside the Python is the proof.
