# Audit: the water's shading and geometry against the Sparse GA design

Written 2026-09-02 on `main` at a738134 (the seafloor relief and the camera views merged),
after the Scriptorium (`math`: radiometry, optics, physics, wavefield, swe, caustics, fold,
frames, compose, priors; `channels`; the GA AST) and three file sweeps: every resource the
water shaders sample, every stage of the water geometry and where its bed comes from, and
every GPU resource created in `src/`. File:line references are to that commit.

## The verdict, in three lines

- **Shading rides the sparse data.** On the Earth path the bed, the bed albedo, the heights
  and the imagery all come from the two page tenants (`earth.height`, `earth.color`).
  `Residency::AddTexture2D` has **zero** call sites. The legacy window branches in
  `Compose.hlsli` and `WaterBank.hlsl` are compiled-in dead code forced to `UINT32_MAX`.
- **Geometry rides it too — but through one keyhole.** The sea mesh, the globe mesh and the
  refracted-ray bed cast read cube + window (`ComposedHeight`, anywhere on the planet). The
  three *kernels* — SWE, churn, wave bank — read **only slice 6**, the z14 page at Mercator
  origin (1263360, 1538048), hard-coded in six places; outside it the SWE sees a wall, the
  churn sees −30 m, the bank falls back to a CPU corner lerp.
- **What is still flat is not bathymetry.** Five weather/optics planes (each its own bank),
  three GIS rasters that are the *default* land/sea classifier every frame, one dense SWE
  current texture, three Gulf panel textures, and four pieces of dead code.

## 1. What the shaders sample (the shading inventory)

| Input | Where it comes from | Class |
|---|---|---|
| Bed (sea PS, globe PS/VS/mesh, refracted cast, churn, SWE, bank) | `earth.height` pages: cube 0–5 + z14 slice 6 (`main.cpp:2172`) | **page tenant** |
| Bed albedo through the ray, land imagery, seafloor relief | `earth.color` pages: cube 0–5 + z14 (6) + z17 (7) (`main.cpp:2363`) | **page tenant** |
| Waves: 3 FFT cascades disp/deriv | `OceanFft.cpp:173-185`, 256² wrapping patches, per frame | sim output |
| Wave bank disp/param/detail rings | `WaterBankLayer.cpp:39-43`, 6 rings × 512², camera-anchored | sim output (sparse) |
| Solved wave field (a, k, phase spinor) | `WaveField.cpp:262`, NE window, re-solved per bucket | sim output (sparse) |
| SWE eta / flux | `SweSolver.cpp:50-51`, sparse atlas over the CUDEM lattice | sim output (sparse) |
| SWE currents | `SweSolver.cpp:237` — **dense committed** RGBA16F 1863×1174 | **stray** |
| Churn (foam memory) | `SeaLayer.cpp:44`, 8192² sparse, fixed 16 km box at the origin | sim output (sparse) |
| Swell shadow (exposure) | `SeaLayer.cpp:713`, 160² R8, CPU march over `BathyModel::Elev` | LUT (CPU bed, not the page) |
| Ocean colour (chl/Kd490/SPM), Hs, wind, ice, GFS wind u/v | `GlobeLayer.cpp:302-361, 621` — five `PlaneBank`s, one epoch, built at boot | planes (RAM → GPU, no disk tree) |
| Land/sea classifier (`ComposedLandness`, the sea's `discard`) | `GisStencil.cpp:184-214` — three committed rasters read from `landmask_ne.raw` / `landmask_global.raw` | **stray — and it contradicts the vector-GIS rule** |
| Cloud volume shadow | `GlobeLayer.cpp:428` 3D tiled bank + `m_cloudSrc` staging kept alive | 3D (on design); staging is a stray |

Fallback constants the shaders still carry, all on "channel off" branches: `-30 m` bed
(`Sea.hlsl:78, 409`, `SeaChurn.hlsl:53`), `+100 m` wall off the page (`Swe.hlsl:71, 80`), bed
albedo `(0.42,0.38,0.28)` / `(0.44,0.40,0.31)` (`Sea.hlsl:377`, `Globe.hlsl:663`), the M7c K_d
triple and constant scatter pair (`Globe.hlsl:356-357, 487-489`), **6 m/s wind** when the
GFS grid is absent (`Globe.hlsl:489`), mid-grey colour (`Compose.hlsli:107`). The design says
no fallbacks; the terrain layer already refuses to draw without its data
(`TerrainLayer.cpp:75-80`). These are the same decision not yet made for the water: either the
channel cannot be off (delete the branches) or its absence refuses (log and skip), never paints.

## 2. Where the geometry gets its bed (the geometry inventory)

| Stage | Extent / placement | Bed | Verdict |
|---|---|---|---|
| Sea mesh (`Sea.hlsl:178-234`) | 3600 m camera-centred tessellated grid, skirt to ±110 km | `ComposedHeight` cube + page, lod −8 inside the CUDEM box | on design |
| Globe mesh (`GlobeMesh.hlsl:80-84`, `Globe.hlsl:278`) | CDLOD to depth 18 (1.19 m verts) | `ComposedHeight` | on design |
| Refracted-ray bed cast (`Globe.hlsl:640-655`) | anywhere | `ComposedHeight`, 2 secant steps | on design |
| SWE (`Swe.hlsl:70-95`) | fixed 1863×1174 CUDEM lattice, 18.8×16 km; Boston 2591×1862 | **slice 6 only**; off-page = wall | keyhole |
| Churn (`SeaChurn.hlsl:46-58`) | fixed 16 km box at the world origin | **slice 6 only**; off-page = −30 m | keyhole |
| Wave bank (`WaterBank.hlsl:261-293`) | 6 camera-following rings to 19.7 km | **slice 6 only**; off-page = CPU corner lerp of the stack at 100 m | keyhole |
| Terrain (`Terrain.hlsl:25-28`) | the CUDEM box | its own `GradeBank` built by `BuildBedBank` — **which has no call site**, so the layer renders nothing | dead |

The cube slices are already bound to the same array view every kernel holds; none of the three
kernels asks for them. `ComposedHeightPages` (`Compose.hlsli:254-270`) is the resolution rule —
containment picks the page, residency picks the mip — and it is a pixel-stage function today.

## 3. Loose ends, ranked

1. **The GA graph is stale where §36 rewired things.** `GaAst.cpp:181, 217, 219` still
   register `height.window → water.bank`, `bathy.cudem → churn.kernel`, `bathy.cudem → sea.ps`
   (citing `Sea.hlsl:71`, which now reads the page); `GaAst.cpp:133-142` still describe
   `window.z14/z17` as tenants. `GaTest.cpp:176` pins `bathy.cudem/bed` as a ledger truth. The
   boot validator cannot see this: the edges are self-consistent descriptions of resources that
   no longer exist. Re-register as `height.pages → {sea.ps, churn.kernel, swe.solver,
   water.bank}` and `color.pages → globe.ps`, and re-pin the truth to the page path (the flip
   ledger changes: page reads are Mercator-uv, no flip, like `CsWindowUv`).
2. **The three GIS rasters are the default classifier**, not a `--stencil` extra:
   `GisStencil.cpp:184-214` → `Compositor.cpp:730-731, 758` → `ComposedLandness`
   (`Compose.hlsli:298-337`) and the sea's `discard` (`Sea.hlsl:321`). They read the `.raw`
   parity fills `GisMask.h:6-13` says are "a realization, not the survey". `gis.landsea` already
   has its own tree on the shared addresses; a third page tenant (R8, 256² tiles, same addresses)
   fed by `TileTree::Provider` over that node retires all three rasters and the two `.raw` files.
3. **Dead code to delete**: `TerrainLayer::BuildBedBank` + `m_bedBank`
   (`TerrainLayer.cpp:123-213`, six levels pinned resident for a layer that never draws);
   `GlobeLayer::BuildColorBank` + `m_colorB` (`GlobeLayer.cpp:44-134`, the pre-M9ap ladder);
   `Residency::AddTexture2D` (`Residency.h:109`); `FieldSet::Add`/`LoadPng`
   (`FieldSet.cpp:20-30`) — the audit's "FieldSet PNGs" stray was already dead, not live.
4. **`SweSolver::m_uv`** (`SweSolver.cpp:237`): the one dense committed texture on the
   per-frame water path. Its sibling `m_velGrad` beside it is already a Volatile `GradeBank`
   with the same residency (`mapWet`); the change is the descriptor and UAV binding.
5. **The kernels' keyhole** (§2): lift `ComposedHeightPages`' containment-then-residency rule
   into a compute-stage `PageBedAt` (manual bilinear `Load`, priors 1) that resolves cube-or-
   window per texel. Then the z14 origin stops being repeated in `main.cpp:2084, 2179-2186,
   3223-3227, 3968-3970, 4011-4017` and the SWE, churn and bank have a bed everywhere.
6. **Boston's SWE is never mip-pinned.** The mip-0 pin (`main.cpp:4008-4034`) covers only the
   Merrimack `bathy` and lives inside `mode == 1 && globe` (`main.cpp:3988`); an active Boston
   solver reads slice 6 at whatever mip the camera left, and with no globe layer nothing warms
   the page at all. The pin belongs to `WeatherManager` per active window.
7. **Local-frame constants** that stop the same machinery working at another estuary: the world
   frame itself (`BathyModel.h:25-28`, one tangent plane at Newburyport, used by
   `SeaChurn`/`WaterBank` `gGeoA`, `WeatherManager::WorldOf`, camera lat/lon); the jet axis
   through the origin (`Jet.hlsli:11-14`); the x-ramps 500/900 and 1400/2100 (`Sea.hlsl:100,
   128`, `SeaLayer.cpp:884`); the churn box at the origin (`SeaLayer.h:250`);
   `sweCurrentGain = 3.2` (`SeaLayer.h:146`); the 14 m foundation sink (`Globe.hlsl:284-293`);
   the hard-coded 128² residency map and 16384 page dim in three kernels (`Swe.hlsl:81`,
   `SeaChurn.hlsl:54`, `WaterBank.hlsl:271`). These are the `Host` object's missing fields
   (AUDIT row 1) seen from the water's side.
8. **Weather/optics planes**: five banks each with its own georeference and a single epoch
   (`GlobeLayer.cpp:302-361`), no disk tree, no forecast-cycle identity; the water's colour is
   whatever epoch was on disk at boot. Deferred by the user until identity carries the cycle;
   recorded here so it is not forgotten.
9. **Gulf panel** (`GulfLayer.cpp:46-74`): three committed textures allocated on every launch,
   drawn only under `--gulf`; `m_mvSrv` still seeds `swe.velgrad` slice 1. The GoMOFS field
   already has a `RasterSource` path (`main.cpp:2503-2509`); the panel should read that plane,
   and Okubo–Weiss is a function of the div/curl slice that already exists.
10. **Small**: `SeaLayer::m_maskTex` (debug residency visualizer; the bank's own residency map
    is the same information); `GlobeLayer::m_cloudSrc` (720×361×10 R32F staging kept for the
    process lifetime after a one-shot build); the swell shadow marches a CPU copy of CUDEM
    rather than the height tree.

## 4. What is now unblocked

The point of the sparse substrate was never the megatexture itself; it was that every field
is a tree on shared addresses, resolvable anywhere at any LOD, cacheable on disk, and
composable. Now that the height and colour are that, these become ordinary work rather than
new architecture:

- **A solver domain anywhere.** The SWE needs a bed and a coast; with item 5 done it reads
  cube-or-window and `WeatherManager` already owns N windows keyed by a `BathyModel` box.
  A window per estuary (Boston exists; Portsmouth, Plum Island Sound, the Cape) is data, not
  code — and `AddTexturePages` takes N z14 slices, so each window can have its fine page.
- **Shoaling, refraction and depth-limited breaking on every coast.** The bank's per-texel bed
  today is CPU corner lerps outside the page; with the cube it has ≥611 m bed everywhere, so
  Green's law, `WaveCurrentAmp` and the `|η| ≤ 0.55 h` breaking act on every shelf and every
  surf zone the camera visits, not only inside 115 km of Newburyport.
- **The solved wave field as a tree node.** `WaveField` (`wavefield`) solves a stationary BVP
  per 1.5 m cell over the NE window and caches it in RAM per bucket. Its inputs are the height
  tree and a current; its identity is those tiles' keys plus the bucket. That is exactly
  `CompositeSource` + `TileTree` ("GAs chain, so new composite trees for CPU physics are
  easy", AUDIT row 14): solve per tile on demand, cache on the NVMe on the shared addresses,
  and the twin test already pins the solver.
- **Exposure (the swell shadow) as a tree node**, marched over the height tree at the tile's
  own resolution instead of a 160² CPU LUT over one CUDEM copy — and therefore available to
  the bank at every ring, everywhere.
- **Churn that follows the camera.** The churn bank is already sparse; re-anchor it like the
  wave bank rings (`ReanchorRing`) and breaking memory exists wherever the camera is, not in a
  16 km box at the station.
- **Derivative products of the height tree** — slope, aspect, curvature, the bedform relief
  cross of `bedalbedo` when eHydro lands, sediment classes, the hillshade already shipped as
  `synth.seafloor.relief` — each a `ColorSource`/`DomainSource` over the stack, each its own
  tree, each composable under the gate. The relief source is the worked example (§39).
- **The classifier from the vector survey** (item 2): once `gis.landsea` has a page tenant,
  `ComposedLandness`, the sea's discard, the bank's `TileWet` and `synth.bed`'s gate all read
  one answer from one tree, and the `.raw` parity fills go.
- **Volumetrics on the same law**: `TileTree` gained `FloatW`/`Half` for scalar fields; a
  volume format over `TileAtlas3D` addresses is the same code path (cloud, then the water
  column itself — K_d as a field in depth is what `optics` already computes at the surface).

## Progress

- Items 1 and 3: done (PR #9) -- the graph names the page tenants; the dead code is gone.
- Items 2 and 6: done (M9ay, §41) -- the survey's tree is a third page tenant and the
  classifier reads it (`GisStencil`'s three rasters and the `.raw` fills are gone; the gate
  became a value gate so the mask can be read as well as gate); the solver mip pin moved
  into `WeatherManager::PinDomains`, every active window, every frame.
- Items 4 and 5: done (M9ax) -- `SweSolver::m_uv` is a Volatile `GradeBank` over the wet
  tiles; `shaders/HeightPages.hlsli` resolves cube-or-window per texel for the SWE, the churn
  and the bank, so the three kernels have a bed everywhere the tenant does. Found on the way:
  `ChurnCbData` had been rotated against `ChurnCb` since M9ar (same size, rows shifted; the
  churn read its current gain from the longitude) -- fixed, and `dxtest` now holds row
  layout by name and offset, not only size (ALGEBRA priors 22).

## 5. Recommended order

1. AST re-registration + ledger truths (item 1) — an afternoon, and the validator then guards
   the page path.
2. Delete the dead code (item 3) and convert `SweSolver::m_uv` (item 4).
3. `PageBedAt` resolving cube-or-window in compute (item 5), then pin per active window from
   `WeatherManager` (item 6).
4. The `gis.landsea` page tenant (item 2); delete `GisStencil`'s rasters and the two `.raw`s.
5. Churn re-anchoring and the exposure node (§4).
6. The wave field as a tree node (§4) — the largest win and the one that most needs the
   Scriptorium's `wavefield` read first.
7. The `Host` object (item 7 / AUDIT row 1) when the second estuary makes the constants bite.
