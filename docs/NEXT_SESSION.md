# Next session: the three-tree megatexture

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

**Python edits**: write the patch script to a file and run it with `py <file>`. Inline `py -c "..."` mangles quotes through the shell nesting and has silently corrupted source (a `'` became `'"'"'` inside a comment and broke a later anchor match). Heredocs into `py - <<'EOF'` are usually fine; inline `-c` is not.

**Line endings**: the repo is mixed. A patch helper must detect CRLF per file and convert the search/replace text to match, or `s.count(old)` returns 0 and the assert fires. Every patch script in this repo does this — copy it.

**The CB layout law** (documented in `WaterBank.hlsl` and violated once per session if you are not careful): new constant-buffer rows append at the **END on BOTH sides**. A same-size permutation passes the byte-parity gate and silently offsets every later row.

---

## 1. What the user actually wants built

This is the architecture. It replaces the current cube + z14 window + z17 detail tenant ladder.

### The pipeline, per source type

```
1. INGEST         GeoTIFFs; Google tiles      -> raw files in a local cache
2. NORMALIZE      to the target body's scale, units, projection
                  -> cached as their OWN sparse GA tree on disk
3. COMPOSE        the two trees -> ONE composite sparse GA tree
                  (a third tree on disk: tile REFERENCES into the input trees,
                   and genuinely composited tiles cached only where they OVERLAP)
```

Then the same process again for the **seafloor textures**, and again for a new
**sparse GIS tree**.

### Then the megatexture

```
land/sea mask   <- from the GIS sparse tree
final tree      <- land texture OVER sea texture, sea mask making the earth
                   layer transparent over water, per pixel
```

That final GA tree is **the one and only mega earth land/floor texture**, and it is what
goes into the Tiled 2D Texture Array.

### Rules the user has stated and re-stated

- **No "regional layers".** Do not add another `AddTexture2D` tenant. The 16384 cap is a
  **per-page** limit; `PageTable` + `LevelLadder` (`src/core/PageTable.h`) already make the
  address space unbounded — pages tile it and resolve to array slices. The existing three
  colour tenants are a *workaround that predates that* and are what should be deleted.
- **No traditional LODs.** Quality comes from tile residency and the mip chain, nothing else.
  A discrete level decides WHICH NODE, never how sharp that node is allowed to be.
- **Disk is virtual memory.** The CPU compositor's job is to prep/organize/transcode tiles
  onto the NVMe and keep only an **index** in memory. Residency uses that index to decide
  what to load, by mip level. DirectStorage does the read.
- The user is aware storing colour in a sparse GA tree looks like overkill. They want it
  anyway. Do not relitigate it.

---

## 2. The open bug: blur at mid zoom

**Reproduce exactly:**

```bash
./build/bin/gagame.exe --sea --one-water --headless --rail-flood out/mipchk --frames 360 --dump /path/mip12s.png --start 2026-08-28T14:00:00
```

`--frames` now overrides the rail's own count, so `360` lands the camera at t=12 s — the
moment in the video where the user sees it. Add `--albedo` for the unlit version.

**What is already known:**

- **`--albedo` is the decisive instrument.** It renders unlit, so it separates *texture* from
  *shading*. Use it first on any "is this blurry" question.
- The **colour** seam at that camera is FIXED (commit `e20c7c2`). Cause was the leaf requesting
  its mip from the distance to the node's **centre**; a node that stopped descending is large,
  its near edge needs several levels more, and the boundary where descent stops rendered that
  request gap as a hard line. Now uses `distNear = dist - arc*0.5`.
- **The lit view is still soft in the same region while the albedo is sharp.** So what remains
  is in the **height/shading** path, not colour. Prime suspect: `ComposedHeightGrad` sizes its
  finite-difference eps from `ComposedHeightLod`, which is still a face-on `dist*pixAng`
  estimate with **no foreshortening term**. A grazing-angle LOD change was tried and reverted
  once (M9aa) for lack of measurement — redo it *with* the measurement.
- **The user's own hypothesis, untested:** the GeoTIFF and Google-tile sources have different
  pixel formats and the composition step may be blurring the Google tiles. Their stated
  expectation: composition does **no blur**, only per-pixel transparency. Worth verifying
  directly — read `Compositor::CubeColor`'s paint loop and check the resample of each source.

---

## 3. Non-sparse texture audit (asked for; done)

14 `CreateTexture*` sites remain outside the tree. Classified honestly:

**Legitimately committed — NOT owed the tree:**

| Site | What |
|---|---|
| `Renderer.cpp` ×3 | scene colour, depth, LDR target — render targets |
| `TileAtlas.cpp` | R8_UINT residency map — the sparse machinery's own metadata |
| `GisStencil.cpp` ×3 | R8G8/R8 stencil masks, derived per frame |
| `SeaLayer.cpp` ×2 | churn mask (tiles-sized) + shadow mask — derived |

**Still owed the tree:**

| Site | What | Note |
|---|---|---|
| `FieldSet.cpp` | loads PNGs as fields | cheapest conversion — needs a PNG `FieldLoader`; the loader interface already exists |
| `GulfLayer.cpp` | `m_uvTex` currents, RGBA32F | straightforward `BuildPlaneBank` |
| `WeatherManager.cpp` | `ownedBathyTex` per-window bed mirror | follows the bed |
| `SweSolver.cpp` | `m_uv` solver state | owed, but as a **Volatile** bank — computed, not loaded |
| `TerrainLayer.cpp` | `m_tex` the bed | **GA path is live and equivalent (0.0000 m)**; consumers read `BedBankReady() ? BedSrv() : HeightSrv()`, so this is now a *fallback*, not the source. Safe to delete once you are happy. |

All five `GlobeLayer` planes (hs, wind, ocean, ice, windSrc) are **already converted** and each
measured `worst |GA - array| = 0`.

---

## 4. Camera tracks and scene configs

**Flags** (`--cam alt,az,pitch`, `--campos x,z`):

| Flag | Meaning |
|---|---|
| `--rail-flood DIR` | the 40 s track: globe → 800 km → 80 km → 7 km → overview → helm-in → helm-gap. 1200 frames + 150 unrecorded settle frames |
| `--frames N` | **overrides** the rail's count — use it to dump the rail at a chosen moment (N/30 = seconds) |
| `--start 2026-08-28T14:00:00` | the standard instant used for every comparison in this repo |
| `--storm Hs,Tp,dir` | e.g. `3.0,10,95`. Sandbox sea state override |
| `--one-water` | the unified wave-bank path — **use this**; it is the path being developed |
| `--mp4 PATH` | encode straight from the framebuffer, no PNGs |
| `--bench` | fly the rail, capture nothing, time honestly |
| `--albedo` | unlit — separates texture from shading |
| `--wireframe` / `--dump-both` | geometry check; `--dump-both` renders solid + wireframe at the same instant |
| `--flat-bed N` | constant bed at N m NAVD — A/B what bathymetry does to the MESH |
| `--pack-tiles` | pack the composed cache into `.gaa` archives, then exit |
| `--direct-storage` | opt in to NVMe→GPU tile reads (**off by default**, see §5) |
| `--selftest` | must stay green |

**Standard stills:**

```bash
# helm, in the throat
--campos 120,-10 --cam 7,92.5,-1.5 --frames 6

# globe, settled (30 frames lets residency settle; fewer is not comparable)
--campos 0,0 --cam 200000,0,0 --frames 30
```

**Standard recording:**

```bash
./build/bin/gagame.exe --sea --one-water --headless --rail-flood out/bench --mp4 out/rail_storm.mp4 --storm 3.0,10,95 --start 2026-08-28T14:00:00
./build/bin/gagame.exe --sea --one-water --headless --rail-flood out/bench --mp4 out/rail_flood.mp4 --start 2026-08-28T14:00:00
```

~53 s each including encode. Full-res 1600×900 lands ~30 MiB, which is right at the delivery
limit — make a 1280-wide copy to send:

```bash
ffmpeg -y -i out/rail_storm.mp4 -vf "scale=1280:-2" -c:v libx264 -pix_fmt yuv420p -crf 26 out/rail_storm_720.mp4
```

**Comparing images**: there is a `pngdiff.py` pattern used throughout — same build, same flags,
two runs is bit-identical (verified), so any nonzero diff is a real change. Beware comparing
against a reference captured before an intervening commit.

---

## 5. State you are inheriting

**Green:** selftest passes; default render path bit-deterministic across runs; engine renders
1.59 ms mean (627 fps) on the rail, 96 fps at helm under `--bench`.

**Built this session, in order:**

- `GaUnits.h` — the normalization stage. `UnitSpec::AcceptsFrom` converts across a scale,
  refuses across a quantity or a vertical datum.
- `ComposeTree.h` — composition as a converging tree; `CompositeSource` makes a compositor a
  source; `BinaryFieldSource` combines two quantities into a third; `DeclareDatumLink` makes
  someone assert an offset with provenance.
- `HeightStackSource.h` — the six-layer height stack as `DomainSource`s, `Blend::LayeredOver`.
- `TileIndex.h` — the in-memory index of what the NVMe holds. `FinestReady()` is the question
  residency wants. **Not yet wired to the scheduler.**
- `TileArchive.h` — one `.gaa` per realization, `(offset, size)` directory, subset-aware
  lookup. `--pack-tiles`. 1.14 GB → 681 MB after dropping ~40% superseded tiles.
- `TileStream.h` — DirectStorage. **Off by default.**

**Two things deliberately left unfinished, with reasons:**

1. **`--direct-storage` is off.** The streamed path renders correct colour but is not pixel-equal
   to the upload ring (9.14% differs) — and equality is not even the right test, since streaming
   timing legitimately differs. It needs a test that can judge it. Two bugs were already found
   and fixed here: `DESTINATION_TILES` writes bytes verbatim while `CopyTiles` does a
   **linear→swizzled** conversion (so the cache is linear and the read must land in a buffer,
   not a tile); and offering a provider a `TileLoc*` when the reader is disabled left `data`
   empty and memcpy'd **zero bytes** into a mapped tile.
2. **`TileIndex` is not wired to residency.** The scheduler still cannot prefer ready work.

**A finding worth keeping:** a full cube face at mip 0 is 128×128×6 = **98,304 tiles; 1,099
exist**. One percent of the finest level has ever been painted. The far field is coarse because
the tiles *do not exist*, not because residency chose badly. Any "why is this blurry" question
should check the index before blaming the scheduler.

**Docs:** `docs/SPARSE_GA.md` §22–26 carry the architecture and the measurements, including
§25's parked non-viable path (composing pages into RAM — wrong *stage*, not just slow).
