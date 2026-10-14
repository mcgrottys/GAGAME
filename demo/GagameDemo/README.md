# GagameDemo

The demo launcher: one .NET 10 WinForms exe that stands beside a GAGAME checkout and drives it
the way `docs/LAUNCH.md` says to. It carries no data, shaders or scenes of its own; it points at
the checkout (found by walking up from its own exe, else asked for once) and runs
`build\bin\gagame.exe <scene> --set streaming.*=...` from the repo root.

```bat
dotnet run --project demo\GagameDemo
dotnet publish demo\GagameDemo -c Release -r win-x64 --self-contained false
```

`GAGAME.sln` at the repo root holds this project and `tools\Scriptorium`; the engine itself stays
a CMake + Ninja build (`build.bat`), and the demo expects that to have run.

## What opens

The scene bar, always. A banner under it names what a fresh machine still lacks (the engine not
built, datasets missing, no Google key) with `Build engine` and `Setup...` beside it; `Dismiss`
hides it until the list changes. `Setup` is a dropdown: cache and limits, the key, the data
files (each opens the install form scrolled to that section), build the engine (`build.bat`,
logged in the window), choose another checkout, open the checkout folder.

## Install form (`Setup` dropdown)

- **Checkout.** The repo and whether `build\bin\gagame.exe` exists; `Build engine` runs `build.bat`.
- **Cache.** The engine reads `cache\` relative to the checkout. `Link` makes that a directory
  junction (`mklink /J`, no admin) to the folder you pick on a big disk. A `cache\` that is already
  a folder with files in it is refused and said so: moving data is the owner's job. `Measure` sizes
  it; `List stale trees` runs the engine's `--tree-prune` listing (changes nothing).
- **Limits.** The Google tile caps are the scene's `streaming.tileBudget`, `dayTiles`, `dayBytes`
  and `googleZoom`; the demo passes them as `--set` on every launch, so they override whatever the
  scene file says.
- **Key.** The engine reads `GAGAME_GOOGLE_MAPS_KEY` from the environment, else from
  `HKCU\Environment`. `Save key` writes that user variable (what `setx` does) and nothing in the
  checkout; the status shows length and prefix only. With no key the form says how to get one.
- **Data.** Every product the engine reads, the harvester that writes it, where it comes from and
  whether its output is present. `Download` runs the harvester with its defaults (`py -3 -I
  harvester\<script>` from the checkout); `Source` opens the provider; `Insert...` runs it for a
  custom place and box, a named window, files dropped into its cache folder, a file argument, or
  skip flags, whichever that harvester supports (`Datasets.cs` is the catalog).

Settings live in `%LOCALAPPDATA%\GAGAME\demo.json`; the key never does.

## Scene bar

`Scene` lists `scenes\*.json`, `scenes\demos\*.json` and `scenes\debug\*.json` (whatever stands there at run time), titled by each file's `_readme`. Picking
one stops the running engine and boots the next (the restart waits for the first exit, so the GPU
and the cache's files are released first). `Relaunch`, `Stop`, `Open scene file` (edit it: the
engine hot-reloads what is hot) and `Controls` (`docs/LAUNCH.md`) are beside it; the engine's
log is the window.

## Debug views (second toolbar row)

The engine's debug views (scene section `debug`; `G` in the window steps through them): a view
dropdown (`none flat edges normals height depth meshlets albedo residency residency.height
residency.landsea`), wire width in pixels, a wire colour (right click returns it to the engine's),
`wireBy` (`colour edge depth`) and contour metres. Each is kept in `demo.json` and passed at the
next launch as `--set debug.view=height --set debug.wirePx=1.5 --set debug.wireColor=[1,0.85,0.2,1]
...`, and only where it differs from the default (`(scene)`, 0, no colour picked), so a debug
scene's own section stands otherwise.

## Fidelity map (`Setup` > `Fidelity map...`, or `GagameDemo --fidelity [FILE]`)

The painter for `.gafid` files (`docs/FIDELITY.md` section 1): the finest grain, in metres on the
ground, each region of the planet wants, which every harvester reads. `--fidelity` opens it alone,
with no scene bar and no engine started.

- **The globe.** Orthographic, software rendered (no GPU). The orientation is one rotor of Cl(3)
  (`Rotor.cs`, planet frame to view; the engine's planet frame is mirrored against east-north-out,
  so the view reflects z once and the rotor does the rest). Drag rotates (left in Look, right
  always), the wheel zooms about the cursor from the whole globe to a view about 2 km across. The
  base is the checkout's own data: the relief raster `data\globe\globe.json` names and every
  `coast*` polyline file `data\gis\gis.json` names, under the graticule (its step follows the zoom)
  and the lattice's cells at the level a view-sized brush would paint. Coarse while moving, full
  resolution at rest.
- **The brush.** A radius in ground metres (slider, log, 1 m to 2000 km; or typed), a grain on a
  log slider from 1 mm to 100 km, and Paint / Erase. A dab writes the lattice cells whose centres
  lie within the radius, at the level whose cells are a quarter of the radius (about 50 cells a
  dab, never more than 4096: a bigger set is cut at a coarser level). Erase returns the ground to
  the layer's default (`FidelityRegions.Clear`). `Paint a box` takes lon0, lat0, lon1, lat1 and a
  grain (`= view` fills it from the view). Undo keeps the last 32 strokes (Ctrl+Z).
- **The overlay.** Painted cells in inferno over the log of the grain (fine is bright), deepest
  last as `GrainAt` reads them, with a legend. The status bar reads the point under the cursor:
  lat/lon, the grain wanted there, the painted cell that answers, the brush's cell there.
- **Layer and default.** The layers are `*`, the file's own, and every data layer a composite
  product (`data\composite\*\<layer>.composite.json`) or a source config (`harvest\sources\**\*.json`,
  a `layer` key) names, with the sources that feed each listed under it. Painting a layer writes
  that layer's cells; until it is painted its readers take `*` (docs/FIDELITY.md section 1). The
  default takes units (`30 m`, `2 km`) or `coarsest` (+Inf).
- **Storage tab.** The same globe, coloured by the bytes STORED per lattice cell (log, 1 kB to
  100 GB), from the storage index (docs/FIDELITY.md section 7: tile trees, archives, the composed
  cache, the composites, read by name and size only). Filters: source (a tree's `<name>.<id>`, a
  composed channel, a composite input), parameter (the tree's name or the composite's layer),
  store (loose, archive, archive.stale, composite:<product>) and cell level. The composites'
  `coverage[]` boxes are outlined: who answers where. A totals table per parameter and store
  (double-click a row to filter by it), and under the cursor the bytes per source and the
  coverage boxes there. `Rescan` reads only what moved; `Rebuild` reads everything.
- **The summary.** Cells, ground per decade of grain (km², each cell less the painted cells below
  it), and an ESTIMATE of a height layer at that grain: area / grain² x 4 bytes, plus a third for
  the pyramid. Arithmetic, not what any source holds.
- **Files.** New, Open, Save, Save as; the last file, the view and the brush are kept in
  `demo.json`. Dialogs start in `data\fidelity`.
