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

## The fidelity map and the sources' limits (docs/FIDELITY.md)

```bat
harvest fidelity new data\fidelity\world.gafid --default inf
harvest fidelity paint-box data\fidelity\world.gafid -72,40,-69,43 30m [--layer NAME] [--level L]
harvest fidelity erase-box data\fidelity\world.gafid 5,45,10,48
harvest fidelity at|finest|dump|stats FILE ...
harvest fidelity check
harvest limits --check
harvest storage [--rebuild] [--index FILE]
harvest storage check
```

`harvest storage` brings the storage index (`Core/StorageIndex.cs`, docs/FIDELITY.md section 7)
up to date and prints what is stored per parameter and store: every tile tree, archive
directory, composed cache and composite read by name and size into the fidelity lattice's cells,
never a payload. The index is `%LOCALAPPDATA%\GAGAME\storage-index.bin`; a rescan reads only
the folders whose own write time moved. `storage check` plants a tree, an archive and a
composite of known sizes and holds the index to them cell for cell.

`Core/Fidelity.cs` is the GAFID01 reader and writer; `Core/FidelityRegions.cs` turns boxes and
brush circles into lattice cells (cells an eighth of the region's extent, at most 4096 a region,
coarser past that), clears a region back to the default, measures each cell's ground exactly
(the gnomonic cube's solid angle) and estimates a height layer's bytes. Grains take units (`1mm`,
`5cm`, `30m`, `2km`, `inf`). `fidelity check` is the contract's self-test: the lattice round trip
(20000 directions), every cell box holding its cell's points, save/load equal cell for cell and
GrainAt for GrainAt, a planted off-lattice cell refusing the file, PaintMany and Clear against
their one-cell definitions, FinestIn against dense sampling.

`harvest/sources.limits.json` (tracked; `GAGAME_SOURCE_LIMITS` names another) is what each source
allows, with the page that says so; `Polite` reads its pace, budgets and bans from it. `limits
--check` holds the shipped file to the constants Polite had before (1.1 s, 2.5 s on NOMADS, 2000
requests, 8 GB, 30 s, 50 MB, the Google fragments) over every host a harvester names.

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
| compose-water | all five above (order: bed, river, tides, currents, seastate) | (new) |
| compose-check | <out>/check.json (the composites' gates) | (new) |
| sources | lists the source configs; <out>/sources_agree_*.json (two sources against each other) | (new) |

## The composites: no places

`harvest compose-<layer> --fidelity FILE.gafid --out data/composite/<name>/` takes every source of
one water layer that has data in the regions a fidelity map paints, ranks them by grain, and
writes ONE product (GACOMP01, docs/FIDELITY.md sections 3 and 6): the bed as tiers of float32
GeoTIFFs (the relief configs: CUDEM 1/9" and 1/3" over ETOPO 2022 15" over 60"), tides as one
stations.json (CO-OPS fitted stations + EOT20 model stations), currents as one currents.json (CO-OPS
stations, NDBC ADCP buoys, a surface field per region and ROMS OFS), sea state as one seastate.json
(GFS-Wave points + the buoy configs' stations), rivers as one river.json (USGS gauges). No place name anywhere: a region is
a cell of the map. Every time-varying source is read from the FIRST copy the cache holds and is
never refetched for a newer one; each input records its valid and fetched time.

POINT LAYERS ARE THE PROVIDER'S WHOLE SET (2026-10-21: datasets by measurement, never by place).
Rasters follow the map's grain; stations are cheap, so compose-tides fits every CO-OPS harmonic
(reference) station and compose-currents takes every CO-OPS current-prediction station, wherever
they stand; the coverage still lists which stand in which region. Before deleting its old output
each counts the requests its stations' missing first copies need and stops, fetching nothing, when
the ledger's budget (sources.limits.json `budgetRequests`, per `GAGAME_LEDGER`) cannot hold them.

THE RIVER REACHES (compose-river, Harvesters/RiverReaches.cs). `river/river.json` keeps `sites`
(every field as before, each site appending `drainage_km2` from NWIS drain_area_va) and adds
`reaches`: every river that meets the sea in a region with an active USGS stream gauge on its main
stem, found from the data alone, no distance constant:
1. the mouths: NHDPlus V2 network flowlines with terminalfl = 1 whose outlet lies in a region
   (USGS wmadata WFS on api.water.usgs.gov, one request a region);
2. the gauges: NLDI upstream-main navigation from each mouth
   (`nldi/linked-data/comid/{T}/navigation/UM/nwissite`), kept where NWIS lists an active stream
   gauge measuring discharge (00060) or stage (00065) (`nwis/site/?siteOutput=expanded`, 100 ids a
   request); tributary gauges are not on the line and are not kept;
3. the line: the terminal's NHDPlus level path (`levelpathi`), mouth first, cut after the
   farthest flowline holding a gauge or tide station; each vertex is [lon, lat, km from the mouth
   (chained great-circle length of the NHD geometry), the flowline's totdasqkm, tidal 0|1 (NHDPlus
   `tidal`)]; `tidal_km` is the end of the tidal run from the mouth (the head of tide NHDPlus draws);
4. the tides: each CO-OPS harmonic station in the stem's box asks NLDI for its own flowline
   (`comid/position`); on the stem, its km is the projection onto that flowline's part of the line.
Gauges carry id, km, drainage_km2, discharge (whether it reports one), off_m, comid, name; a
main-stem gauge reporting discharge joins `sites` even inland of the regions (its own IV request,
so the region gauges' first copies stay theirs). compose-tides then reads each station's `river_km`
(and appends `river_reach`) from these reaches. Every response is cached under cache/river/ forever.

```bat
harvest compose-water --fidelity harvest\testdata\coast3_3to10m.gafid --out data\composite\coast3
harvest compose-check --out data\composite\coast3 --against-tides data\tides\haulover\stations.json
```

## The sources: one config file each

`harvest/sources/buoys/*.json` and `harvest/sources/relief/*.json` (tracked) are the composites'
sources, one file a source (docs/FIDELITY.md 8): its reader (`kind`), server and dataset, the map from
the engine's quantities to the source's variables (a name, or a CF `standard_name`), the units and
direction conventions it writes, a relief's datum and native step, its rank and coverage.
`Core/SourceFactory.cs` makes each with the reader its kind names (`[SourceKind]` classes under
`Sources/`): `ndbc-realtime2`, `erddap-tabledap` (buoys), `cudem-index`, `tiled-geotiff`,
`erddap-griddap` (relief). `compose-seastate` and `compose-bed` take every config of their layer, or
`--sources id,id`.

```bat
harvest sources list
harvest sources agree-buoys --a noaa.ndbc --b erddap.coastwatch.ndbcmet --fidelity harvest\testdata\coast3_3to10m.gafid --out data\composite\coast3
harvest sources agree-relief --a erddap.coastwatch.gebco2020 --b noaa.etopo2022.15s --fidelity harvest\testdata\coast3_3to10m.gafid --box -74.5,36.5,-73.5,37.5 --out data\composite\coast3
harvest compose-bed --fidelity harvest\testdata\coast3_3to10m.gafid --out data\composite\gebco --sources noaa.cudem.ninth,noaa.cudem.third,erddap.coastwatch.gebco2020,noaa.etopo2022.60s
```

`list` makes every config (its reader checks it) without a fetch; `agree-buoys` holds two sources of
one instrument to equality at the same station and moment (half the last digit NDBC prints);
`agree-relief` holds two grids to one lattice and measures their difference by depth band.

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
