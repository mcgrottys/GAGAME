# Next session: the megatexture, on trees that now exist

Paste this whole file as the opening prompt. It is written to be read cold.

---

## 0. Hard rules, learned the hard way

**CHECK THE SCRIPTORIUM FIRST.** `math('priors')`, then `math('compose')` (or the topic you are
touching), then `symbols(<name>)` before adding any class. The user called out a whole session of
compositor work done without opening it; priors §15 and §9 already described both traps that
session fell into, and §18 records the correction. If the MCP tools are not loaded, the server
was not attached at session start -- `tools/Scriptorium/bin/Release/net10.0/Scriptorium.dll` must
exist (`dotnet build -c Release tools/Scriptorium`) and the user enables it in `/mcp`. Until then
read `docs/ALGEBRA.md` directly; it is the same content.

**BUILD ONLY THROUGH THE PowerShell TOOL.** Not Bash.

```
Get-Process -Name gagame -ErrorAction SilentlyContinue | Stop-Process -Force; cmd.exe /c "C:\Users\lordc\source\repos\GAGAME\build.bat" 2>&1 | Select-String -Pattern "error C|error LNK|built:" | Select-Object -First 10
```

- `cmd.exe /c build.bat | grep ...` from the **Bash** tool hangs past the 10 minute timeout, repeatedly, every session. It is not flaky — it does not work. Do not retry it.
- The `Stop-Process` prefix is not optional. Windows locks a running `gagame.exe` and the **linker fails** on it; if a previous run is still alive the build dies at LNK with a confusing message.
- Bash is fine for everything else (running the exe, `git`, `ffmpeg`, `py` scripts, greps).

**Python edits**: write the patch script to a file and run it with `py <file>`. Inline `py -c "..."` mangles quotes through the shell nesting and has silently corrupted source.

**A SOURCE'S FOOTPRINT IS ITS ADMISSION TICKET.** `SourceTouches` rejects any source whose
declared box does not overlap the tile, so a source constructed BEFORE its data is loaded
declares an empty box and is silently never asked anything -- built, wired, logged, and inert.
Two A/B renders came back byte-identical and looked like a result. What caught it was a tree
directory with zero files in it. **After loading anything that sets bounds, re-declare them**
(`GisMaskSource::Refresh`), and check that a new source's tree folder is non-empty before
believing any A/B.

**BACKSLASHES IN PATCH SCRIPTS BIT ME TWICE THIS SESSION.** A Windows path literal inside a
non-raw Python string gets un-escaped once on the way in: `"cache\\\\composed"` in the heredoc
becomes `"cache\\composed"` in Python and lands in the C++ file as `"cache\composed"`, where MSVC
silently eats the unknown escape. Use a raw string, or build the separator from `chr(92)`, and
**grep the emitted line afterwards** — the compiler will not tell you.

**Line endings**: the repo is mixed. A patch helper must detect CRLF per file and convert the
search/replace text to match, or `s.count(old)` returns 0 and the assert fires. Every patch script
in this repo does this — copy it.

**A FAILED FILE WRITE MUST BE LOUD.** `std::ofstream` on a path whose folder does not exist fails
silently. The tree audit ran a full pass, painted 1742 tiles, reported perfectly good numbers, and
stored **nothing** — because it composed without ever calling `EnsureFrame`. `tree_detail::WriteTile`
now checks and logs once. Do that for every new writer.

**The CB layout law** (documented in `WaterBank.hlsl`): new constant-buffer rows append at the
**END on BOTH sides**. A same-size permutation passes the byte-parity gate and silently offsets
every later row.

---

## 1. Where the architecture stands

```
1. INGEST         GeoTIFFs; Google tiles      -> raw files in a local cache        DONE (was)
2. NORMALIZE      to the body's scale/units/projection
                  -> cached as its OWN sparse GA tree on disk                      DONE (colour)
3. COMPOSE        the trees -> ONE composite sparse GA tree
                  (references into the inputs; composited tiles only where
                   they OVERLAP)                                                   DONE (colour)
4. the same again for SEAFLOOR textures                                            DONE (earth.seafloor)
5. the same again for a sparse GIS tree                                            DONE (vector)
6. MEGATEXTURE    land/sea mask <- the GIS tree; final tree = land texture OVER
                  sea texture, sea mask making the earth layer transparent over
                  water, per pixel. That final tree is THE one mega earth
                  land/floor texture and it is what goes into the Tiled 2D
                  Texture Array.                                                   DONE (earth.color root)
```

All six stages exist as ONE graph -- the water's graph -- with `TileTree` caching every node's
output on the NVMe (§31 of `docs/SPARSE_GA.md`). `--color-trees` puts the root in front of the
three colour tenants; it is still OFF by default, for the reason in item 1 below.

Stages 2 and 3 landed this session for `earth.color`, measured (§29 of `docs/SPARSE_GA.md`):
8484 tiles audited, **worst |direct − tree| = 1/255 over 139 M texels**, 52.7% of tiles are pure
references, globe still pixel-identical through the renderer.

### What landed, and what is next

**Landed:** the megatexture as the water's graph (§31): `earth.land` = [google, aerial],
`earth.seafloor` = [bed], `gis.landsea` (vector, meridian sweep), `seafloor<gis` = GateSource,
`earth.color` = LayeredOver(land, gated seafloor). `TileTree` caches every node; references are
STORED as zero-byte `.ref-<childId>` entries; a node's identity is its inputs'. 21,508 addresses
warm: root 74% references, `earth.land` 97%, `earth.seafloor` 100%. Renders pixel-identical to
the incumbent (globe 0 px, helm 2 px by 1/255); bench 4.65–4.82 ms vs incumbent 4.75. The §29
`SourceTree`/`ColorTreeStack` path is DELETED -- do not resurrect it; the graph is the path.

**Next, in order:**

1. **Pack the trees, resolve refs to places: DONE (§33).** `--pack-trees` writes one archive per node per
   frame (28,584 tiles, 1.79 GB, 10.7 s); `TileTree::Tile` answers a `TileLoc` when the caller can take
   one, and a stored reference resolves into the child's archive. Reads 1.8 -> 1.2 ms/tile through one
   handle; `--direct-storage` is pixel-identical to the upload ring on the helm still. DirectStorage is now the
   default; re-pack after any warm.
2. **Ring loads: DONE, measured, THE DEFAULT (§32).** `--res-trace` is the instrument
   (deficit by mip, queue depths, slots as reads vs paints by provider wall time); `--ring-loads`
   admits a request only if its parent is mapped. Frame 600 of the rail is the picture: baseline
   patchy, ring uniformly sharp at the same instant; 540 identical, 720 converged. Timing a wash.
   The instrument's other finding is the next lever: with the trees warm, loads are 1.8 ms READS
   (per-file open), not paints -- which is item 1. The user made it the default after the frame-600 pair and the rail video;
   `--no-ring-loads` is the A/B.
3. **Then delete the rungs.** See §2. Still the largest structural item, and now the megatexture
   root is the one provider the ladder needs.

### Rules the user has stated and re-stated

- **No fallback textures, only the megatextures.** `TerrainLayer::m_tex` is gone; a bed bank
  that fails to build throws at boot. Remaining non-atlas DATA textures are conversions, not
  fallbacks: `GulfLayer::m_uvTex`, `SweSolver::m_uv` (Volatile bank), `WeatherManager`'s bed
  mirror, `FieldSet` PNGs, and `GisStencil`'s three raster masks (the shader classifier still
  samples them; the compositor no longer does).
- **No "regional layers".** Do not add another `AddTexture2D` tenant. The 16384 cap is a
  **per-page** limit; `PageTable` + `LevelLadder` (`src/core/PageTable.h`) already make the address
  space unbounded — pages tile it and resolve to array slices. The existing three colour tenants
  are a *workaround that predates that* and are what should be deleted.
- **No traditional LODs.** Quality comes from tile residency and the mip chain, nothing else. A
  discrete level decides WHICH NODE, never how sharp that node is allowed to be.
- **Disk is virtual memory.** The CPU compositor's job is to prep/organize/transcode tiles onto the
  NVMe and keep only an **index** in memory. Residency uses that index to decide what to load, by
  mip level. DirectStorage does the read.
- The user is aware storing colour in a sparse GA tree looks like overkill. They want it anyway.
  Do not relitigate it.

---

## 2. One bed (section 36): the solver, the sea, the churn and the water bank all read
##    slice 6 of the height page tenant. The bed bank and the per-window mirror are gone.

The solver domain is pinned at mip 0 every frame; the trace probe (`--trace 42.8125,-70.8175`,
200 frames) is the check -- page texel vs CPU stack, MATCH. A pixel diff is NOT a check here:
any bed change moves the solve and the foam everywhere. The height page is fed by a TileTree
over BuildHeightStack under --color-trees (section 37: fidelity-sorted inputs, float leaves,
R16F root, worst 0.25 m / 7 mm against the composed cache). USER DECISIONS (2026-09-01): the water/weather planes stay in RAM -- fine as is; a disk tree
for them waits until the loader identity carries the forecast cycle (a tree keyed on name|unit
would serve a stale GFS forever). Confirmed: seabed HEIGHTS are already in the height
megatexture (the same layers carry land and seafloor elevation); seabed COLOUR is in the colour
megatexture via earth.seafloor. What the acceptance image still wants is an INGESTED seafloor
appearance source (shaded bathymetry / sediment imagery) replacing the classifier's colour --
a colour-tree source, not a height change. Next: that, then --color-trees as the default.

## 2b. The rungs are DELETED for colour (section 34) AND height (section 35).

Colour is ONE tenant (pages 0..5 cube, 6 z14, 7 z17); height is ONE tenant (pages 0..5 cube,
6 z14). Both select by containment and residency, no fades. The water bank reads the bed from
slice 6 of the height page tenant through its own root signature's new Texture2DArray space.
What keeps "only three things on the GPU" partial is the stray list in docs/AUDIT.md row 13.

DirectStorage is ON by default and PROVEN: helm 400 and globe 200 frames are 0 pixels against
the upload ring, after the four fixes in section 34. Every earlier "not pixel-equal" was the
slot race, not streaming timing.

THE ACCEPTANCE IMAGE the user supplied: New England from altitude, uniform resolution across
land and sea, the SEAFLOOR VISIBLE THROUGH THE WATER with its own relief and colour, no haze.
`synth.bed` is a classifier; the seafloor needs to be an INGESTED texture tree (bathymetry-
derived shading + sediment colour) composed under the land tree. That, and the height pages, are
the two builds that move the picture toward the image.

### (history) The three rungs, and why deleting them was the main event

`colorCubeT` (cube 16k), `winTenant` (Mercator z14 window), `detTenant` (z17 detail window) are
three hand-picked pages of a ladder nobody wrote down as a ladder, with a **64× resolution cliff**
between the cube's finest (611 m/texel) and the window's coarsest, and hand-off smoothsteps in
`Compose.hlsli` that each rung must know about its neighbour to compute.

A previous session nearly made this worse by proposing a fourth `AddTexture2D` at z11. The user
stopped it. The correction is **uniform rungs selected by containment**, resolved CPU-side through
`PageTable`, which needs no hand-off gates at all.

`GlobeLayer::BuildColorBank` exists, compiles, and is **not called** — it composes pages eagerly at
startup (1.4 billion per-texel queries for four 16384² pages) and had to be killed. That is a wrong
*stage*, not a slow constant: the right shape is the lazy `TileProviderFn` the residency manager
already drives, which `ColorTreeStack::Realization` now returns. The remaining work is the page
ladder that decides *which* frame a node asks, not the painting.

---

## 3. The blur at mid zoom — what is now known

**Reproduce:**

```bash
./build/bin/gagame.exe --sea --one-water --headless --rail-flood out/mipchk --frames 360 --dump /path/mip12s.png --start 2026-08-28T14:00:00
```

`--frames` overrides the rail's own count, so `360` lands the camera at t=12 s. Add `--albedo` for
the unlit version.

- **`--albedo` is the decisive instrument.** It renders unlit, so it separates *texture* from
  *shading*. Use it first on any "is this blurry" question.
- The **colour** seam at that camera is FIXED (`e20c7c2`): a leaf asked for the mip its node's
  *centre* needed; it now uses `distNear = dist - arc*0.5`.
- **The user's hypothesis — "the composition step may be blurring the Google tiles" — was tested
  and is FALSE.** `Compositor::PaintColorTile` blends with `acc += (rgba - acc) * w`, which at
  `w == 1` is an exact copy, and `GoogleColorSource::Pixel` is a **nearest** fetch with no
  filtering at all. Composition does no blur; the user's stated expectation was right.
  What the read *did* turn up: `ZoomFor` is **clamped at z14** (`955 cm/px`, the politeness cap).
  The z17 detail window asks for 1.2 m/texel and google can only answer 9.5 m, so that window is
  an 8× nearest magnification — **blocky, not blurry**, and a different artefact from the reported
  one. Worth showing the user a crop before acting on it.
- **The lit view is still soft where the albedo is sharp**, so what remains is in the
  **height/shading** path. Prime suspect: `ComposedHeightGrad` sizes its finite-difference eps from
  `ComposedHeightLod`, still a face-on `dist*pixAng` estimate with **no foreshortening term**. A
  grazing-angle LOD change was tried and reverted once (M9aa) for lack of measurement — redo it
  *with* the measurement.
- **Check the index before blaming the scheduler.** A full cube face at mip 0 is 128×128×6 =
  98,304 tiles; **1,099 exist**. One percent of the finest level has ever been painted. The far
  field is coarse because the tiles *do not exist*.

---

## 4. Flags, tracks and standard captures

| Flag | Meaning |
|---|---|
| `--rail-flood DIR` | the 40 s track: globe → 800 km → 80 km → 7 km → overview → helm-in → helm-gap. 1200 frames + 150 unrecorded settle frames |
| `--frames N` | **overrides** the rail's count — dump the rail at a chosen moment (N/30 = seconds) |
| `--start 2026-08-28T14:00:00` | the standard instant used for every comparison in this repo |
| `--storm Hs,Tp,dir` | e.g. `3.0,10,95`. Sandbox sea state override |
| `--one-water` | the unified wave-bank path — **use this** |
| `--mp4 PATH` | encode straight from the framebuffer, no PNGs |
| `--bench` | fly the rail, capture nothing, time honestly |
| `--albedo` | unlit — separates texture from shading |
| `--wireframe` / `--dump-both` | geometry check |
| `--flat-bed N` | constant bed at N m NAVD — A/B what bathymetry does to the MESH |
| `--pack-tiles` | pack the composed cache into `.gaa` archives, then exit |
| `--no-direct-storage` | the upload ring for every tile. NVMe→GPU reads are the **default**, and proven: helm 400 / globe 200 frames are 0 px against the ring (§34) |
| `--ds-serial` | diagnostic: one DirectStorage batch in flight at a time |
| `--no-color-trees` | the incumbent providers behind the page tenants, for the A/B. The trees (colour AND height) are the **default** |
| `--tree-audit N` | the megatexture tree vs the incumbent, N tiles per realization, exit. Skips tiles the incumbent has not repainted from today's stack |
| `--warm-trees` | compose every address of the tree regardless, no comparison. **This is the warm-up**; ~15 min from cold for 21.5k addresses |
| `--pack-trees` | one `.gaa` per node per frame under `cache/trees/`; refs resolve into the child's archive. Re-run after a warm; loose files are kept |
| `--no-gis-gate` | drop the vector land/sea gate (on by default) — the A/B for what the survey changed |
| `--no-seafloor` | drop the global seafloor relief source (`synth.seafloor.relief`, on by default) — the A/B for the seabed |
| `--no-exposure` | no swell-exposure page tenant (everything exposed) — the A/B for the shadow node. With `--res-trace`, every 150 frames logs the page's resident mip at the camera, the node's own value, and the manager's view of the tiles under it |
| `--gis-dump PATH` | write the survey gate as a 1024² PGM over its box and exit (255 water / 0 land). `py out/pgm2png.py` converts it. **Look at the gate**, don't infer it |
| `--res-trace` | every 30 frames: residency deficit by mip per tenant, queue depths, slots spent on reads vs paints |
| `--no-ring-loads` | the old queue: request the whole column at once. Ring loads (parent must be mapped; the view refines one ring at a time) are the **default** |
| `--selftest` | must stay green |

**Standard stills:**

```bash
# helm, in the throat
--campos 120,-10 --cam 7,92.5,-1.5 --frames 6

# globe, settled (30 frames lets residency settle; fewer is not comparable)
--campos 0,0 --cam 200000,0,0 --frames 30
```

**Standard recording** (~53 s each including encode; full-res 1600×900 lands ~30 MiB, right at the
delivery limit — make a 1280-wide copy to send):

```bash
./build/bin/gagame.exe --sea --one-water --headless --rail-flood out/bench --mp4 out/rail_flood.mp4 --start 2026-08-28T14:00:00
ffmpeg -y -i out/rail_storm.mp4 -vf "scale=1280:-2" -c:v libx264 -pix_fmt yuv420p -crf 26 out/rail_storm_720.mp4
```

**Comparing images**: same build, same flags, two runs is bit-identical (verified), so any nonzero
diff is a real change. **A COLD CACHE IS NOT A PIXEL DIFFERENCE.** Comparing `--color-trees`
against the incumbent on a cold tree cache reported *worst 131/255* and 19% of pixels — all of it
residency, none of it paint. After `--tree-audit 100000` warmed the trees the same comparison was
0 pixels and 2 pixels. Warm both sides or you are measuring the cache.

---

## 5. State you are inheriting

**Green:** selftest passes; default render path bit-deterministic across runs.

**THE INHERITED "1.59 ms mean (627 fps)" DOES NOT REPRODUCE, and it is not this session's doing.**
Measured today, all on the same machine, same rail, same instant, all converged over repeat
passes (`--bench`, 1200 frames, RENDER only):

    per-source trees + the vector gate      4.70 ms mean, p99 11.5    (3 passes: 4.70/4.69/4.71)
    incumbent paint, gate OFF               4.75 ms mean, p99 11.3    <- the inherited config
    incumbent paint, gate ON                8.11 ms mean, p99 27.7    <- repainting the region

So the trees cost nothing against the path they replace (4.70 vs 4.75 is noise), and the gate
costs nothing at render time by construction -- it changes tile CONTENT on worker threads and no
shader reads it. The third row is a cache still filling, not a cost: the incumbent has to repaint
every tile the new source's footprint touches, which is the whole argument for the trees.

Do not chase the 1.59 ms. Whatever it was measured on, this machine does not do it today with
the pre-session configuration either. Re-baseline before treating any number here as a
regression.

**Built recently, newest first:**

- `SourceTree.h` — `SourceTree` (one source's tiles, identity = the source, `.void` markers for
  declared-but-unreached tiles) and `ColorTreeStack` (the composite tree; computed references;
  `Compose` is where the megatexture's blend rule goes). `AuditColorTrees` is the gate.
- `ColorFrame` (in `Compositor.h/.cpp`) — the realization's geometry, said once.
  `Compositor::ColorRealization(channel, frame)` is both old colour providers.
- `TileArchive.h` — one `.gaa` per realization, `(offset, size)` directory, subset-aware lookup.
  `--pack-tiles`. 1.14 GB → 681 MB after dropping ~40% superseded tiles.
- `TileStream.h` — DirectStorage. **Off by default.**
- `TileIndex.h` — the in-memory index of what the NVMe holds. **Not yet wired to the scheduler**,
  and it does not know about trees.
- `GaUnits.h`, `ComposeTree.h`, `HeightStackSource.h` — normalization, composition as a converging
  tree, the six-layer height stack.

**Deliberately unfinished, with reasons:**

1. **`--direct-storage` is the DEFAULT.** Through the packed trees it is pixel-identical to the
   upload ring (§33); the 9.14% figure was the composed-cache path. `--no-direct-storage` is the
   A/B. Re-run `--pack-trees` after any warm, or new tiles take the ring until packed.
2. **`--color-trees` is off.** Proven equal to 1 LSB, but a reference still copies bytes instead of
   resolving to a `TileLoc`; do item 1 of §1 before switching the default.
3. **`TileIndex` is not wired to residency.** The scheduler still cannot prefer ready work, and
   readiness *ordering* measured worse on a cold cache (preferring what is cached starves the work
   that fills the cache). It should pay on a warm one.

**Non-sparse texture audit — still owed the tree:** `FieldSet.cpp` (PNG fields, needs a PNG
`FieldLoader`), `GulfLayer.cpp` (`m_uvTex`, straightforward `BuildPlaneBank`), `WeatherManager.cpp`
(`ownedBathyTex`), `SweSolver.cpp` (`m_uv`, owed as a **Volatile** bank), `TerrainLayer.cpp`
(`m_tex` — the GA path is live and equivalent at 0.0000 m, so this is a *fallback* now and is safe
to delete). Legitimately committed and NOT owed the tree: `Renderer.cpp` ×3 (render targets),
`TileAtlas.cpp` (residency map), `GisStencil.cpp` ×3, `SeaLayer.cpp` ×2 (derived per frame).

**Docs:** `docs/SPARSE_GA.md` §22–29 carry the architecture and the measurements, including §25's
parked non-viable path (composing pages into RAM — wrong *stage*, not just slow) and §29's three
trees.
