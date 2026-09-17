# LAUNCH — how to build, run, and interrogate GAGAME

Served by the Scriptorium's `note` tool, section by section (`## id — Title` headers, same
grammar as ALGEBRA.md). Update THIS file when the CLI grows; the index follows on reindex.

## launch-build — Building (Windows, VS2022 + Ninja)

cmake is not on PATH in a bare shell; go through vcvars64. The session scratchpad keeps a
`build.bat` with exactly this:

```bat
call "C:\Program Files\Microsoft Visual Studio\2022\Community\VC\Auxiliary\Build\vcvars64.bat"
"C:\Program Files\Microsoft Visual Studio\2022\Community\Common7\IDE\CommonExtensions\Microsoft\CMake\CMake\bin\cmake.exe" -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=RelWithDebInfo
"C:\Program Files\Microsoft Visual Studio\2022\Community\Common7\IDE\CommonExtensions\Microsoft\CMake\CMake\bin\cmake.exe" --build build
```

Binary: `build/bin/gagame.exe`. Run from the REPO ROOT (data/, shaders/, docs/ paths are
relative). Shaders compile at RUNTIME — editing `shaders/*.hlsl` needs no rebuild, just
rerun. The Scriptorium MCP server rebuilds with
`dotnet build -c Release tools/Scriptorium` and reindexes with
`dotnet run --project tools/Scriptorium -- --index`.

## launch-run — Running the sim

Windowed (interactive): `gagame.exe --sea --one-water` — the one-water globe with the live
Merrimack window. Every scene-shaped flag below is also a scene property (`launch-scenes`). `--globe-cam lat,lon,altkm` starts at an orbit pose. Mars: `--mars`.

Headless render: `--headless --frames N --dump out.png` (dump implies headless; ~200+
frames before judging visuals — residency and composed caches need warm-up; 80-frame
stills at the bird's eye are COLD and lie: the classifier misreads and storm/calm render
identical).

Camera: `--campos x,z` (world metres, ACT0816 anchor, +x east +z north) and
`--cam alt,heading,pitch` (metres, compass degrees, degrees). Time:
`--start 2026-08-28T14:00:00` (UTC; that timestamp is max flood at the entrance, +1.06 m/s
— the standard reference frame). Sea state override: `--storm Hs,Tp,dirDeg` (e.g.
`--storm 3.0,10,95`); omit for live NOAA conditions. `--tile-budget N` raises the Google
fetch budget for long rides (default polite; 3000 for rail renders).

## launch-videos — The rail rides

`--rail-jetty DIR` — ground to space: helm at the gap, tip-to-tip, bird's eye, 800 km
orbit (40 s). `--rail-flood DIR` — the zoom-in: orbit → inlet → helm at the entrance
(40 s). `--rail-zoom DIR`, `--rail DIR` — older orbit rides. Rails render 150 unrecorded
SETTLE frames first (residency warm-up) then 1200 numbered PNGs. Encode:

```bat
ffmpeg -y -framerate 30 -i DIR/rail_%04d.png -c:v libx264 -pix_fmt yuv420p -crf 18 -preset slow out.mp4
```

Showcase conditions: storm ascent `--storm 3.0,10,95 --start 2026-08-28T19:30:00` (max
ebb — the whitewater river over the shoal); calm evening low `--start 2026-08-28T22:15:00`.

**M10 — the Droste rails (the globe within the globe; `math('droste')`).** `--droste` hangs
the root under a leaf of its own quadtree (default: the level-16 leaf at the entrance mouth, a
153 m inner Earth resting on the bed, a quarter twist per level about north). On the storm
preset:

```bat
build\bin\gagame.exe --sea --one-water --pixel-water --storm 3.0,10,95 --start 2026-08-28T14:00:00 --tile-budget 3000 --rail-droste out\droste\dive --mp4 out\droste\dive.mp4 --droste-light appealing --droste-levels 2 --droste-level-sec 20
```

`--rail-droste DIR` flies the storm rail to the helm (its last key re-aimed at the fixed point),
then the similarity's own logarithmic spiral pose(u) = S^u(helm), one level per
`--droste-level-sec`, `--droste-levels` deep. `--rail-droste-out DIR` is the out-and-back: in
two levels, turn at the bottom, back out along the same spiral facing outward, then the climb to
orbit. `--droste-light realistic|appealing` is the lighting A/B (realistic: one real sun, so the
twisted inner Merrimacks face away from it and are at night; appealing: every level lit as the
root). `--droste-at lat,lon,level`, `--droste-fill f` and `--droste-twist deg` move and reshape
the link (under ~75° of twist the dive runs from 45° above the mouth instead of through the
helm). The log names every re-root (`[droste] frame N: the nearest ground is now level k`).

## launch-scenes — Scenes, recipes and tools (M12)

A run is a SCENE. The flags above still work — they are one way to write a scene — but the
engine resolves every launch into one document and boots from that (`docs/ARCHITECTURE.md` §4):

```bat
build\bin\gagame.exe [scene.json] [--set a.b.c=value ...] [--tool name[:args]] [instrument flags]
```

- **No file, no mode flag** → `scenes/chart.json` (the M1 chart). **A mode flag and no file**
  (`--sea`, `--globe`, `--gulf`) → `scenes/merrimack.json`. **`--planet mars`** → `scenes/mars.json`.
- **The fold:** struct defaults < the `base` file < each `include` overlay in order < every `--set`
  in command-line order. `--set views.sea.fovY=40` edits one key; `--set views.sea.at={...}` replaces
  the eye whole; `--set portals.droste.twistDeg=45` edits one portal by name.
- **`--print-scene`** prints the resolved document and exits before the device. It is the round
  trip: `gagame <flags> --print-scene > my.json`, then `gagame my.json` runs the same scene.
- **Recipes** (`scenes/recipes/*.json`) are the recorded flag lines as complete documents, each line
  kept in `_recipe`: `storm_rail`, `helm`, `helm_ebb`, `bird`, `key7km`, `globe`, `droste`,
  `helm_boat`, `selftest`. The storm rail from its file, full length, with the in-process encode:

```bat
build\bin\gagame.exe scenes\recipes\storm_rail.json --set capture.frames=1200 --set capture.mp4=out\rail.mp4
```

  (The recipe keeps the gate's 300 frames; the legacy rail line resolves to 1200 recorded frames, so
  set it back for a video.)
- **Rails are files:** `scenes/rails/{classic,zoom,flood,jetty,mars,droste,droste-out}.json`,
  selected by `rails.active` (the `--rail-*` flags set it). A rail is segments — `keys` (screw slerp
  between poses), `spiral` (Sᵘ through the portal), `hold`, `turn` — and editing a copy is how a new
  flight is authored.
- **Entities, portals, effects** are named lists: `entities[]` (`vessel`, `at` — its `az` is the
  bow's heading, absent = north — `controller` helm|fixed, `throttle`, `steer`; a hull reads the
  solver's surface around itself every frame, no setting), `portals[droste]` (`lat`,
  `lon`, `level`, `fill`, `twistDeg`, `lighting`, and the optional destination `toLat`/`toLon`: the
  place the inner globe presents where the root shows the leaf), `effects[slice.plane]`.
- **Interests** (`interests[]`): a subject (`target`, an entity's name — the interest follows it) or a
  fixed place (`at` {x, alt, z}) whose water stays resident within `radius` metres at the grain its
  kernels read: the solved field's pages at the solver's cells, the swell shadow at the bank's floor,
  the bed at the fine rings' grain. A view holds only the interests it names
  (`views[].interests: [{"name": ...}]`), so an eye flying in arrives to landed data, and a camera
  that names none holds none. The Haulover demo's view names its boat.
- **Gates** (`gates[]`): a cuboid in the root's flat frame (`at` {x, alt, z, az}, `size` [across, up,
  along]) whose far side is a place on the same planet (`toLat`, `toLon`, and the heading `toAz`). A
  hull whose centre of gravity enters the box is carried by one motor into the destination's own
  tangent space, still moving, its height above its own water kept; nothing else moves, and the
  destination has no box, so anyone there sees the boat appear. From the source side the box is a
  window: whatever is seen through it is the destination, the same planet walked once more from the
  carried eye (its tiles load while the box is in view), outlined by a faint rim.
- **Play: the Haulover gate.** `build\bin\gagame.exe scenes\demos\haulover_portal.json` opens a window at the helm of the
  RHIB in the Merrimack entrance with a gate across the channel ahead. Helm: W/S throttles, A/D steer,
  Q/E split the levers, Shift/Ctrl trim; T leaves the helm for the free camera and returns. Through
  the box you see Haulover's sea; drive into it and you come out of Baker's Haulover Inlet heading
  out to sea. Haulover renders only
  as well as the engine's data there (its global relief floods the barrier island).
- **Hot reload:** every file the fold read, plus the active rail, is watched. Save one and the log
  prints `[scene] reload: <files> <n> fields changed (<k> hot, <r> restart) FNV-1a <state>`; a key
  marked `restart` in `docs/scene_schema.json` is reported and not applied, a removed key returns to
  its default, and an unknown key refuses the whole reload with its path.
- **Tools** are one-shot modes by name: `--tool selftest`, `--tool pack-tiles`, `--tool ocean-probe`,
  ... (`docs/registries.json` lists all twenty); the legacy flags still reach the same tool.
- **Instruments stay flags** and are never scene data: `--pix`, `--gpu-time`, `--bench`,
  `--no-vsync`, `--res-trace`, `--lens`, `--stencil`, `--albedo`, `--viz`, the `--dump-*` probes.
- **For tools that write scenes:** `docs/scene_schema.json` (every key's type, quantity, unit,
  default, doc and hot|restart) and `docs/registries.json` (every nameable thing and its properties)
  are rewritten at each boot from the engine's own tables.

## launch-verify — The verification loop (run before believing anything)

- `--selftest` (or `--tool selftest`) — the fifteen gates, in run order: pga, dxtest (CB parity,
  the sampler law, the mesh stage, the view list's scene constants), cga, droste, gatest (GA
  products + fold + frames + AST validation), space (the frame calculus), composetest,
  watertest, tiletest, atlastest, threadtest, simclock, rigid, vessel, scene (properties, the
  fold, views, water, rails, entities, portals, effects, reload). All must PASS.
- `--trace lat,lon` — THE HYPERVISOR: one sample walked through the whole one-water chain
  on the CPU, every transformation printed with its AST edge and frame. First tool when
  the water surprises you; validate against NOAA with the printed station numbers.
- `--dump-fibers` — the bank's planes as PNGs + `fiber_detail.f32` + `fiber_meta.json`
  (range-checked against the AST's declared ranges; violations print).
- `--settle-sync` / `--settle-hold N` — a still's dump frame is HELD at its instant (the last
  frame `--frames N` renders) for extra frames, so *when* the far tiles happened to land stops
  deciding the image. `--settle-sync` holds until the residency is quiet (pending 0, no
  in-flight read, nothing ring-held, for `kEvictAgeFrames` frames; give-up cap 3000, and it
  says so). `--settle-hold N` holds exactly N frames whatever residency is doing; together
  they drain first and then hold to at least N. The exit line names the rule that held it.
  A/B TWO BINARIES AT THE SAME N: a drain-judged hold runs as long as the residency makes it
  (211 vs 225 frames on two runs of one binary at the bird), and only the counted form is a
  flag both sides share. The churn atlas is frozen for exactly the held frames — its kernel
  only climbs at a frozen dt, so an unfrozen hold made the foam a function of the hold's
  length (`[gpu] sea.churn 0.000 ms` on held frames is the proof of wire).
- `--settle-exact` — the same hold, exited only when the RESIDENT SET IS THE WALK'S WANT SET:
  every held turn the residency manager reads the walk's per-tile frame stamps, counts the
  deficit (wanted, not mapped at that mip), drops every tracked tile the walk did not want
  this frame (the invalidation's own retire path; the coarsest mip is the floor and stays; a
  tile is dropped once unwanted for `kEvictAgeFrames` turns and never while a DirectStorage
  batch still carries it), and reports the turn EXACT when deficit, stale, pending, in-flight
  and retiring are all zero; the dump waits for `kEvictAgeFrames + 4` exact turns in a row. A
  wanted tile that can never land (its load failed, or an ancestor's did) is excluded by its
  state, never by a timeout. `--settle-sync`'s quiet test fires over two different resident
  sets (two quiet bird holds differed in whole tiles' mips); this is THE GATE'S definition of
  settled from perf step 25 on, on both binaries of an A/B: the still is a function of the
  pose and the data. The exit prints a `[settle-exact]` ledger per tenant -- wanted, mapped,
  deficit, unreachable, dropped over the hold, and an FNV-1a of the mapped set -- so two runs
  that differ can be told apart as "different resident set" or "same set, different bytes".
  Composes with `--settle-hold N` (exact first, then to at least N). `--settle-sync` stays as
  it is for the pop-in track's landing series, which needs the schedule-dependent behaviour.
  `--settle-clear-churn` zeroes the churn atlas at the first held frame (the clear kernel over
  every resident tile, then the freeze keeps it): the A/B of a held still with and without it
  separates a residual the residency owns from one the churn's pre-hold history owns (the foam
  deposited during the real frames lands when the bed and the wave pages happened to).
- `py -3 proofs/water_optics.py` — M9's water-quality forms (the Kd490→RGB transfer and the
  two-flux deep colour) against the measured NOAA fields; renders `proofs/water_optics.png`
  with the rendered water swatches. Toggle the feature itself with `closures.waterOptics`
  in `data/wave_scene.json` (false restores the M7c constants byte for byte). NOTE the flag
  is under `closures`, and it is a JSON **bool** — see priors 15 before trusting an A/B.
- `--dump-water-state` then `py -3 proofs/inlet_storm.py` — the independent 2D physics
  figure (`proofs/inlet_storm.png`) + the MATCH REPORT holding the kernel to the same
  pure functions on the same fields (healthy: corr ≥ 0.9 per ring, mean |log ratio| < 5%).
- `--lens waterdata|bed|level|current|ring|cascade` — data-as-color debug views (flat,
  unlit; compare waterdata 1:1 with the proof figure). `--inject 1|2` — synthetic pattern
  cards for alignment forensics.
- `--water-map f.png` / `--bathy-map f.png` — georeferenced charts with coastline.
- `--ocean-probe lat,lon` — WeatherManager rungs + provenance at a point.
- `--albedo` — raw composed color lens; `--stencil` — GIS alignment overlay.
- `--pix N` — programmatic PIX GPU capture of N frames (needs PIX installed; passes are
  marked with AST node names).
- `--gpu-time` — timestamp queries around every pass (each layer, tonemap, present copy, and
  the compute-vs-draw sub-passes inside sea/waterbank/globe), read frames-in-flight deep so
  nothing stalls. Prints `[gpu] <pass> mean p50 p95 max@frame` at exit next to the `[rail]`
  lines and writes `<raildir>/gpu_ms.csv` (one row per frame, one column per pass). Off = no
  queries issued.
- `--no-vsync` — windowed only: ALLOW_TEARING swapchain + `Present(0, ALLOW_TEARING)` when
  DXGI supports it, so the `[perf]` line measures the engine and not the display's refresh.
  Default stays `Present(1, 0)`.
- Boot report (`[gpu] boot:` lines, always printed) — the adapter chosen, whether it is the
  high-performance pick, which adapter owns each DXGI output, and whether the present is
  SAME-ADAPTER or CROSS-ADAPTER (hybrid laptop: RTX renders, the iGPU that owns the panel
  flips). Read it before believing any windowed frame rate.
- `py -3 tools/raildiff.py BASE.mp4 NEW.mp4 --out-dir DIR --stills 3` — two recordings of the
  same rail, per frame. THE RULE: the verdict is CONTINUITY + LUMA — exit 1 only on a
  new-only `tblend` spike (a pop that BASE does not also have) or a per-second mean luma
  drifting past `--yavg-max` (0.5/255). SSIM is printed and never gates: two rails of one
  flight from two binaries measured SSIM min 0.93 with YAVG inside 0.15/255 and no spike,
  because a residency-landing shift decorrelates the helm's crests while the sea's brightness
  and its frame-to-frame continuity are unchanged. Record both sides like for like
  (`--rail-flood DIR --mp4 OUT.mp4 --tile-budget 3000`, same `--storm` and `--start`).
- Boot always prints the AST (`[gaast]`) and regenerates `docs/GA_AST.md` +
  `docs/ga_ast.json`; `py -3 tools/astdiagram.py` redraws `docs/diagrams/*.svg`.

## launch-data — Data prerequisites & caches

Harvesters (python, repo root; all cache-first and polite): `harvest_globe.py`
(NE15/ETOPO relief, GFS-Wave Hs/wind, cloud volume, and — M9 — the gap-filled ocean-colour
trio `oc_chl/oc_kd490/oc_spm.f32` plus sea ice `ice.f32`; per-section `--skip-*` flags, so
`--skip-etopo --skip-mars --skip-ne --skip-windvec --skip-clouds` refreshes just the water's
quality in ~15 s), `harvest_bathy.py --window merrimack|capeann|boston` (CUDEM),
`harvest_tides.py` (constituent fits incl. the 20-station set), `harvest_water.py`
(+`--eot20` for the 2.3 GB EOT20 grids), `harvest_waves.py` / `harvest_currents.py` /
`harvest_gis.py` / `harvest_vectors.py` / `harvest_aerial.py` (MassGIS orthos) /
`harvest_survey.py` (M7y: CUSP shoreline + NHD HR hydrography; rebakes landmask_ne.raw
with the NHD open-water carve — run harvest_vectors.py after it to refresh the vpack).
Large static datasets live in `D:\DataCache\GAGAME\` (300 GB granted 2026-08-31, for
texture / GIS / topology / bathymetry / weather). `data/` holds the
repo-sized realizations; the composed cache lives beside it and repaints exactly what a
program edit touches (the soak rule — deleting it only costs a repaint).

## launch-secrets — Keys and attribution

`GAGAME_GOOGLE_MAPS_KEY` — user env var (set once with `setx`, read at boot). NEVER
commit or print the key (repo is public; log length/prefix only). `secrets/` is
gitignored. Attribution: "Imagery (c) Google" on rendered imagery; EOT20 CC-BY 4.0
(Hart-Davis et al. 2021, doi:10.17882/79489); OSM ODbL for survey edits; MassGIS public
orthos; NOAA data courtesy CO-OPS/NCEI/NDBC.
