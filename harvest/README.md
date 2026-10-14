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
