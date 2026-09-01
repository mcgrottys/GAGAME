# Next session: the megatexture, on trees that now exist

Paste this whole file as the opening prompt. It is written to be read cold.

---

## 0. Hard rules, learned the hard way

**BUILD ONLY THROUGH THE PowerShell TOOL.** Not Bash.

```
Get-Process -Name gagame -ErrorAction SilentlyContinue | Stop-Process -Force; cmd.exe /c "C:\Users\lordc\source\repos\GAGAME\build.bat" 2>&1 | Select-String -Pattern "error C|error LNK|built:" | Select-Object -First 10
```

- `cmd.exe /c build.bat | grep ...` from the **Bash** tool hangs past the 10 minute timeout, repeatedly, every session. It is not flaky — it does not work. Do not retry it.
- The `Stop-Process` prefix is not optional. Windows locks a running `gagame.exe` and the **linker fails** on it; if a previous run is still alive the build dies at LNK with a confusing message.
- Bash is fine for everything else (running the exe, `git`, `ffmpeg`, `py` scripts, greps).

**Python edits**: write the patch script to a file and run it with `py <file>`. Inline `py -c "..."` mangles quotes through the shell nesting and has silently corrupted source.

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
4. the same again for SEAFLOOR textures                                            NOT STARTED
5. the same again for a sparse GIS tree                                            NOT STARTED
6. MEGATEXTURE    land/sea mask <- the GIS tree; final tree = land texture OVER
                  sea texture, sea mask making the earth layer transparent over
                  water, per pixel. That final tree is THE one mega earth
                  land/floor texture and it is what goes into the Tiled 2D
                  Texture Array.                                                   NOT STARTED
```

Stages 2 and 3 landed this session for `earth.color`, measured (§29 of `docs/SPARSE_GA.md`):
8484 tiles audited, **worst |direct − tree| = 1/255 over 139 M texels**, 52.7% of tiles are pure
references, globe still pixel-identical through the renderer.

### The next build, in order

1. **Pack the trees, so a reference costs zero bytes.** `TileLoc` already lets a provider return a
   *place* — path, offset, size — and DirectStorage reads it NVMe→GPU. A reference IS a `TileLoc`
   into the input tree's archive. What blocks it: `TileArchive::Pack` is hardwired to
   `cache/composed/<channel>/<realization>` and to filenames carrying a subset hash; a source tree
   has neither. Parameterize the root and make the subset optional. This is the single highest-value
   next step and it is small.
2. **The seafloor tree and the GIS tree**, through the same `SourceTree`/`ColorTreeStack`.
   `GisStencil.cpp` already realizes coast/river masks per frame — that mask becomes a tree.
3. **The megatexture**, which is then one more `ColorTreeStack` whose blend is not "over by weight"
   but "land over sea, alpha from the GIS mask". `ColorTreeStack::Compose` is where that rule goes;
   it is ~20 lines and reads three 64 KB tiles.
4. **Then delete the rungs.** See §2.

### Rules the user has stated and re-stated

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

## 2. The three rungs, and why deleting them is now the main event

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
| `--direct-storage` | opt in to NVMe→GPU tile reads (**off by default**) |
| `--color-trees` | colour through the per-source trees (**off by default**) |
| `--tree-audit N` | compose N tiles per realization both ways, print the disagreement, exit. Also **warms the trees**, which is what a fair render comparison needs |
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

**Green:** selftest passes; default render path bit-deterministic across runs; engine renders
1.59 ms mean (627 fps) on the rail, 96 fps at helm under `--bench`.

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

1. **`--direct-storage` is off.** The streamed path renders correct colour but is not pixel-equal to
   the upload ring (9.14% differs) — and equality is not the right test, since streaming timing
   legitimately differs. It needs a test that can judge it.
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
