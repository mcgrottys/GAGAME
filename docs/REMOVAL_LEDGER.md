# Removal ledger: code that no shipped scene runs

A read-only survey of `main` at `71e3a33`, as checked out in the worktree
`earth-texture-hierarchy-485aec` (its branch adds documents and one Python tool on top of main;
`git diff --stat 71e3a33 HEAD -- src shaders scenes tools CMakeLists.txt` shows only
`tools/hierarchy/archive_stale.py`). Nothing was built, run, changed or deleted.

It is a menu, not a proposal. Each row says what would go, what would be lost, and what would
have to be true first.

**Checked again by grep, 2026-09-29, by the one who asked for the survey:** `Retire(` has no
caller; `TreeWater::SetBoats` has no caller; `SeafloorReliefMod` has no caller;
`TerrainLayer::Render` returns before any draw; `m_bathyCpu` is written and never read;
`scene/LayerComponent.h` is included by nothing. The rest is as the survey read it and is
not checked a second time. Rows 9, 10, 13, 14 and 16 lose nothing that runs and are being
removed in a scratch tree behind gates (every stage's bytecode, six stills pixel for pixel,
the selftest); the other rows each lose something and wait for the owner's word.

**How to read it.**

- **READ**: I read the lines, or a grep printed them.
- **INFERRED**: from structure, from a grep's absence of hits, or from partial reading.
- Line counts are `wc -l` for whole files. For ranges, `a-b` counts both ends. Sums are mine.
- "Shipped scenes" are `scenes/merrimack.json`, `scenes/mars.json`, `scenes/chart.json`,
  `scenes/recipes/*.json` and `scenes/demos/*.json` (through their `base`).
- The engine is 82,883 lines of `src` and `shaders` (`.cpp .h .hlsl .hlsli`). READ (wc).

---

## 6. The table (placed first)

Sorted by lines, largest first. "Tests" means selftest code that exists only for that side.
Sections 1 to 5 and 7 follow it.

| # | candidate | C++ | shaders | tests | total | what is lost | confidence |
|---|---|---:|---:|---:|---:|---|---|
| 1 | Residency manager, being replaced by another agent (`src/hal/Residency*`) | 3,533 | 0 | in the files | 3,533 | replaced, not removed: net unknown | READ (wc only; not analysed) |
| 2 | AIS fleet and its Kelvin wakes (`water.fleet.enabled`, default false) | 563 | 105 | 128 | 796 | the lane of floats, the GPU wake term, the CPU wake twin (dead today), the wake gatest block | READ ranges; WaterComponent part INFERRED |
| 3 | The sea's own tessellated sheet (`--no-one-water`) | 154 | 623 | 2 | 779 | the M5b sheet, the spectrum plot, the `sea.ps` AST node, one layout check | READ |
| 4 | Gulf of Maine map (`scene.mode` gulf, `--gulf`) | 297 | 141 | 0 | 438 | the GoMOFS map view and its 44029 validation line | READ files; the ~37 lines of sites INFERRED |
| 5 | Slice-plane effect (`effects` slice.plane, `--slice`) | ~265 | 8 | n/m | ~273 | the M7o cutaway, the only effect type | INFERRED |
| 6 | GIS vector layer (drawn only under `--stencil`) | 220 | 43 | 0 | 263 | the coast and graticule alignment overlay | READ gate; files wc |
| 7 | Typed-node seam (`Node`, `LayerComponent.h`, the `nodes` key) | 246 | 0 | n/m | 246 | the plugin seam M12 built; nothing in the engine uses it | READ files; key INFERRED unread |
| 8 | The tile tree's old fold walk (`FoldWalkForTest`) | 91 | 0 | ~150 | ~241 | the byte reference the new walk is held to | READ (TileTree.h); tests INFERRED |
| 9 | `TerrainLayer`, which draws nothing | 129 | 105 | 0 | 234 | nothing (it draws nothing); saves a PSO compiled at every boot | READ |
| 10 | `PageTable` class (no caller outside GaTest) | 109 | 0 | 75 | 184 | the M9h page-table model and its gatest block | READ |
| 11 | Globe vertex fallback (`--no-ms`, or no mesh shaders) | ~85 | 70 | 0 | ~155 | the engine would need mesh shaders | READ ranges; sum mine |
| 12 | Vertex-shaded water (`--no-pixel-water`) | ~12 | 107 | 0 | ~119 | the M9bg look as an A/B; saves a per-vertex shading the default path pays for | READ |
| 13 | `hal::Retire` (no caller of `Retire()`) | 88 | 0 | 0 | 88 | a deferred-release queue that is always empty | READ |
| 14 | Shader functions nothing references | 0 | 37 | 0 | 37 | nothing | READ (grep) |
| 15 | The `--no-color-trees` switch itself | ~20 | 0 | 0 | ~20 | the A/B render with the flat compositor | READ sites; count INFERRED |
| 16 | `SeaLayer::SetBathyCpu` (written, never read) | 3 | 0 | 0 | 3 | nothing | READ |
| 17 | Eleven small switches (section 2.14) | n/m | n/m | n/m | n/m | one A/B each | sites READ; lines not measured |
| | **Total without row 1** | **2,282** | **1,239** | **355** | **3,876** | | |
| | **Total with row 1** | **5,815** | **1,239** | **355** | **7,409** | | |

n/m = not measured. Rows 2 to 16 are 4.7 % of the engine's lines.

The three largest outside the residency manager:

- **The AIS fleet and its wakes (796).** No shipped scene enables the fleet, so the bank's wake
  loop runs over a table of zeros, and the CPU twin of the wake law has no caller at all.
- **The sea's own sheet (779).** Every shipped scene draws the sea with the globe's meshlets;
  `Sea.hlsl` and `SpecPlot.hlsl` are compiled at every boot and never drawn.
- **The Gulf of Maine map (438).** No scene starts in it; only `--gulf` or the TAB key reach it.

---

## 1. The switches

Every flag in `src/app/Options.cpp` and key in `src/scene/SceneSchema.cpp` that picks one of
two implementations, or turns a whole subsystem on. Defaults are from `src/scene/SceneSchema.h`
(scene keys) or `src/app/Options.h` (raw flags). READ unless marked.

### 1.1 Scene keys (each also has a flag, through `Options::ToSets`, Options.cpp:537-798)

| key (SceneSchema.cpp) | flag (Options.cpp) | default | shipped scenes on the other side | selftest or tool |
|---|---|---|---|---|
| `scene.mode` (:49-50) | `--sea`/`--globe`/`--gulf` (:96-98) | world (SceneSchema.h:80) | chart: `chart.json:8`, `recipes/selftest.json:9`. Gulf: none | a bare `gagame` boots `chart.json` (Options.cpp:563-566) |
| `scene.planet` (:51) | `--planet` (:195) | earth | mars: `mars.json:14` | none seen |
| `sun.source` (:112-114) | `--sun` (:419-423) = pinned | ephemeris (SceneSchema.h:111) | earth: all four demos (`haulover_portal.json:30`, `haulover_dusk.json:17`, `merrimack_dusk.json:21`, `skyfall.json:43`). Pinned: none | none seen |
| `sea.datum.fromStation` (:138) | `--datum` (:397-400) | true | none | none seen |
| `water.oneWater` (:253) | `--no-one-water` (:416) | true (SceneSchema.h:170) | none: every file says true (e.g. `merrimack.json:31`, `mars.json:33`, `storm_rail.json:42`, `selftest.json:37`) | tools pass `--one-water`, the default (`tools/stills.sh:14`, `gate_stills.sh:15`) |
| `water.pixelWater` (:254) | `--no-pixel-water` (:418) | true | none (`merrimack.json:32` and the rest) | none |
| `water.swe.enabled` (:159) | `--swe-off` (:401) | true | none | none seen |
| `water.swe.westBoundary` (:160) | `--swe-west-off` (:402) | true | none | none seen |
| `water.bank.flatBed` (:172) | `--flat-bed` (:319) | false | none | none seen |
| `water.wavefield.enabled` (:182) | none | true | none | none seen |
| `water.closures.waterOptics` (:221) | none | true | none | none seen |
| `water.fleet.enabled` (:243) | none | **false** (SceneSchema.h:167; SceneConfig.h:112) | none: all false (`merrimack.json:75`, `mars.json:77`, recipes' line 91) | none |
| `streaming.directStorage` (:274) | `--no-direct-storage` (:253) | true | none | also the only path when the redist is absent (CMakeLists.txt:175-177) |
| `streaming.colorTrees` (:275) | `--no-color-trees` (:259) | true | none (`merrimack.json:81`, `storm_rail.json:105`, `selftest.json:100`...) | the tree tools build the trees whatever it says (Assembly.cpp:675, :715, :956) |
| `streaming.gisGate` (:276) | `--no-gis-gate` (:261) | true | none | none seen |
| `streaming.seafloor` (:277) | `--no-seafloor` (:262) | true | none | none seen |
| `streaming.exposure` (:278) | `--no-exposure` (:263) | true | none | none seen |
| `streaming.ringLoads` (:279) | `--no-ring-loads` (:267) | true | none | inside the residency manager |
| `views[].reversedZ` (:368) | none | true | none (`bird.json:129` and the rest say true) | none seen |
| `rails.active` (:405-407) | `--rail*` (:106-123) | none | flood: `storm_rail.json:213` | classic, zoom, jetty, droste, droste-out: flags only |
| `portals[].enabled` (:419) | `--droste` (:124) | true when declared | `recipes/droste.json:220` | |
| `portals[].lighting` (:432) | `--droste-light` (:136-139) | realistic | appealing: none (grep of `scenes` for "appealing": no hit) | none seen |
| `gates[]` (:438-463) | none | none | `haulover_portal.json:44-47`, `skyfall.json:57-60` | |
| `entities[]` (:479-496) | `--boat` (:177) | none | `helm_boat.json`, the demos | |
| `effects[]` slice.plane (:498-515, :640) | `--slice` (:186-189) | none | none | SceneTest uses it (4 mentions) |
| `layers[].enabled` (:521) | none | true | none | read only for the order check (Scene.cpp:169, :441-464); nothing sets a layer's `enabled` from it. INFERRED from grep |
| `nodes[]` (:539-551, :615) | none | none | none | only SceneTest registers a component type (SceneTest.cpp:547) |

### 1.2 Raw flags that pick an implementation (not scene keys)

| flag (Options.cpp) | default (Options.h) | what it selects | set by |
|---|---|---|---|
| `--no-ms` (:327) | mesh shaders on (:143) | the globe's vertex fallback (section 2.9) | nothing shipped; also taken when the device or runtime lacks mesh shaders (GlobeLayer.cpp:206-217, :2350-2353) |
| `--predict-inline` (:316) | off (:68) | the prefetch walk on the main thread (FrameLoop.cpp:2952, :3166, :3804) | an A/B instrument |
| `--jobs-inline` (:278) | off (:83) | every job on the calling thread (main.cpp:53) | the selftest (ThreadTest.cpp:321) and `tools/gate_compare.sh:9` |
| `--ds-serial` (:254) | off (:72) | one DirectStorage batch in flight | residency; not analysed |
| `FoldWalkForTest` (TileTree.h:1338) | the new walk (:1323) | the tile tree's old fold walk | the selftest only (TileTreeTest.cpp:527, :702, :868, :973) |

### 1.3 Flags that turn on an instrument (listed, not analysed)

All off by default. None is set by a shipped scene; the recipes carry them only as `capture.*`
keys (settle) or not at all. READ (Options.cpp lines named).

`--pix` (:152), `--dump-fibers` and `--sky-probe` (:156), `--lens` (:163), `--probe-cull-far`
(:173), `--dump-meshlets` (:174), `--inject` (:190, bank test cards; WaterBank.hlsl:394,
:866), `--debug` (:84), `--res-trace` (:268), `--water-tiles` (:270), `--thread-audit` (:274),
`--res-trace-frames` (:282), `--water-probe`/`--pages-trace`/`--res-audit` (:292), `--bench`
and `--bench-overlap` (:210-213), `--gpu-time` (:216), `--no-vsync` (:217), `--viz` (:204),
`--wireframe`/`--meshlets`/`--wireflat` (:205, :322, :324), `--mesh-stats` (:325),
`--dump-both` (:320), `--stencil` (:326, see row 6), `--albedo` (:328), the four `--settle-*`
(:224-246), `--dump-hdr` (:247), `--mp4` (:207). The tools (`--tool`, `--selftest`,
`--tree-audit` and the rest) are dispatched; see section 3.6.

---

## 2. What runs only on the non-default side

### 2.1 The AIS fleet and its Kelvin wakes (`water.fleet.enabled`, default false)

What it is. Five floats shuttle an AIS lane; the bank kernel adds each one's Kelvin wake.
READ: FrameLoop.cpp:2765-2797 fills the boat table only `if (waterScene.fleetEnabled &&
route.Ready())`, and hands it to the bank at :2795-2796 either way, zeros when off.

Only on the fleet's side:

- **C++, engine** (563):
  - FrameLoop.cpp:1191-1194 (the lane's load), :2765-2797 (the traffic and `SetBoats`),
    about 12 lines of `ApplyWater` at :1544-1563 (the boats list), FrameLoop.h:78, :324. 51. READ.
  - `src/scene/Route.h`, 91, whole file. Its only user is FrameLoop (grep). READ.
  - SceneConfig.h:103-121 (the boats), :156-166 (their default JSON), :240-257 (their parse). 48. READ.
  - WaterComponent.h:150-162, :178 and WaterComponent.cpp:27, :110-131, :229, :247, :291,
    :363-366 and the list handling in `ReadJson` (:416-). About 45. INFERRED.
  - SceneSchema.cpp:227-248, :262; SceneSchema.h:162-168, :176. 31. READ.
  - Assembly.cpp:573-582, about 4 lines of it. READ (the lines), INFERRED (the count).
  - WaterBankLayer.h:136-140, :190-191, :278; WaterBankLayer.cpp:706-707. 10. READ.
  - WaterSurfaceTree.h:64-71 (`WakeBoat`), :81-88 (`SetBoats`, `SetSampleScale`), :192-193;
    WaterSurfaceTree.cpp:142-145, :388-412 (the wake term). 47. READ.
  - WaterTerms.h:262-497: `WakeVessel`, `WakeSample`, `WakeBranch`, `WakeOne`. 236. READ (structure by grep; not every line read).
- **Shaders** (105): WaterBank.hlsl:56-63 (the `gBoatA`/`gBoatB` rows), :308-389 (`WakeBranch`,
  `WakeOne`), :783-797 (the eight-boat loop in the fill). READ.
- **Tests** (128): GaTest.cpp:652-779, block 10, "KELVIN WAKES". READ (block bounds).
- Data: the `fleet` block in every scene file. Not counted.

Two facts beside the switch:

- **The CPU wake twin is dead today, fleet or not.** `TreeWater::SetBoats`
  (WaterSurfaceTree.cpp:142) has no caller: grep for `SetBoats(` finds only
  `waterBank->SetBoats` and `waterBankB->SetBoats` (FrameLoop.cpp:2795-2796). So
  `m_boatCount` stays 0 and :398-412 never runs; `WakeOne` in WaterTerms.h is called only
  there (:406). READ. This is REVIEW finding 13.
- **On the default path the kernel loops over eight zeroed boats per texel** (WaterBank.hlsl:792-794).
  Whether `WakeOne` returns early on a disabled slot was not read. INFERRED cost, not measured.

### 2.2 The sea's own sheet (`water.oneWater = false`, `--no-one-water`)

What it is. The M5b tessellated grid (VS, HS, DS, PS) and the spectrum plot beside it.
READ: Assembly.cpp:626 sets `sea->drawEnabled = !S.water.oneWater`; SeaLayer.cpp:824-838
draws the sheet and the plot only `if (drawEnabled)`.

But the layer is not the sheet. `SeaLayer` also owns the sea state, the cascades (`m_fft`),
the churn, the currents and the CPU ocean twin, and those run on the default path
(SeaLayer.cpp:16-31, :152-461, :794-822). READ.

Only on the sheet's side:

- **Shaders** (623): `shaders/Sea.hlsl` (565) and `shaders/SpecPlot.hlsl` (58), whole files.
  Sea.hlsl is compiled only at SeaLayer.cpp:117-133 and named in the DxTest layout table;
  no shader includes it. SpecPlot.hlsl is compiled only at SeaLayer.cpp:140. READ (grep).
- **C++** (154):
  - SeaLayer.cpp:100-145 (`BuildPsos`: the sheet, its wireframe, the plot), :421-457 (the
    plot's constants), :824-838 (the draw). 98. READ.
  - SeaLayer.h:33-34 (`kPatches`, `kSpecSamples`), :153-157 (`pixelWater`, `targetEdgePx`),
    :175-178 (`drawEnabled`), :264-271 (`SpecCbData`), part of :278 (three PSOs), :283. About 21. READ.
  - Assembly.cpp:322-323, :326-327 (`pixelWater`, `targetEdgePx`, `atlasVisualize`,
    `wireframe`, which feed only the sheet's constants), :626-630, :1285. 10. READ.
  - GlobeLayer.h:114-128 (the `oneWater` argument), :745; GlobeLayer.cpp:1460, :2035; FrameLoop.cpp:1540, :2836. About 5. READ.
  - Options.cpp:415-416, :605; Options.h:179-180; SceneSchema.cpp:253. 6. READ.
  - GaAst.cpp:230-231, :235-238, :340-341, :406-408, :443-445: the edges into `sea.ps`. 14. READ.
- **Tests** (2): DxTest.cpp:144-145, the `SeaCb` layout row. READ.

Not counted, and why. `SeaCbData` (SeaLayer.h:215-237) and its fill (SeaLayer.cpp:463-581)
stay unless split: the churn's constants copy rows of `m_seaCb` (SeaLayer.cpp:613-754) and the
solver takes `m_seaCb.sea[0]` (:806). Which rows only Sea.hlsl reads was not traced. INFERRED.

Also READ: `SeaLayer::Simulate` and `Render` return early without `m_seaPso`
(SeaLayer.cpp:795, :810), so today the sheet's PSO gates the FFT, the churn and the solver.
Every boot where the sea state loads (Assembly.cpp:316) compiles the four sheet stages and the
plot (SeaLayer.cpp:22).

### 2.3 The Gulf of Maine map (`scene.mode` gulf, `--gulf`)

What it is. M3's map: the GoMOFS current field, a velocity-gradient pass over it, the 44029
buoy ring (GulfLayer.h:1-11). READ.

Reach. No scene file sets mode gulf. The layer is built whenever the currents carry a field
(Assembly.cpp:549-555), and TAB cycles into it in any windowed run (FrameLoop.cpp:2074-2078).
So it is reachable by a key press, not only by the flag. READ.

Only on its side:

- **C++** (297): GulfLayer.cpp (185) and GulfLayer.h (75), whole. Assembly.cpp:36, :207,
  :549-555; Assembly.h:67, :155. FrameLoop.cpp:42, :424, :506, :512, :1746, :1896, parts of
  :2074-2078, :3530, and the gulf branch of `FormatTitle` (:129-136). Options.cpp:97, parts
  of :555, :565, :572; Options.h:58; Scene.h:94 and SceneSchema.cpp:49-50, :641 (a word each).
  Files READ (wc); the ~37 lines of sites INFERRED.
- **Shaders** (141): Gulf.hlsl (76) and VelGrad.hlsl (65). Each is compiled only by
  GulfLayer (grep). READ.
- `GulfLayer::MvSrv()` (GulfLayer.h:35-39) has no caller (grep for `MvSrv(`). READ.
- Not in it: the `swe.velgrad` bank behind `--lens velgrad` is built from GoMOFS through the
  compositor (Assembly.cpp:1093-1255), not through this layer. READ.

### 2.4 The slice-plane effect (`effects` slice.plane, `--slice`)

What it is. M7o's cutaway plane, the "algebra-first demo node" (Globe.hlsl:1110-1117). It is
the only effect type registered (SceneSchema.cpp:640). No shipped scene declares it. READ.

- **C++** (about 265): SlicePlane.cpp (73), SlicePlane.h (57), Effect.h (49), whole.
  Assembly.cpp:266-267, :1257-1265; FrameLoop.cpp:1503-1520; Scene.cpp:340-352;
  SceneSchema.cpp:498-515, :592, :613, :640, and the props in SceneSchema.h;
  Options.cpp:186-189, :721-724; Options.h:46-47; the globe's constant rows it writes.
  Files READ (wc); the rest INFERRED.
- **Shaders** (8): Globe.hlsl:1110-1117. READ.
- Tests: SceneTest.cpp mentions `SlicePlane` 4 times; GaTest pins the plane's algebra
  (Globe.hlsl:1111 says "gatest-pinned"). Not measured.
- Note: the effect is declared and fanned out at every boot, switched off
  (Assembly.cpp:1260-1265). READ.

### 2.5 The GIS vector layer (drawn only under `--stencil`)

READ: Assembly.cpp:1325-1336 builds `GisLayer` whenever `data/gis/gis.json` loads and sets
`gisLayer->enabled = opt.stencil`. INFERRED: the scenes' `{"name": "gis", "enabled": true}`
does not turn it on, because grep finds `layers[].enabled` read only for the order check
(Scene.cpp:169).

- **C++** (220): GisLayer.cpp (136), GisLayer.h (76), and Assembly.cpp:1328-1335.
- **Shaders** (43): GisVec.hlsl, compiled only by GisLayer (grep).
- Not counted: `GisStencil`, `VectorPack` and `Exchange`, which the tools and other layers
  also use (grep); the surface's stencil row (SurfaceFrame.cpp:143, :195).
- It is an instrument, not an old path: M6i's coast and graticule alignment overlay
  (Options.h:142).

### 2.6 The typed-node seam (`Node`, `LayerComponent`, the `nodes` key)

- `src/scene/LayerComponent.h` (84) is included by no file. Its two mentions are comments
  (Component.h:27, ViewContext.h:31). READ (grep for `#include "scene/LayerComponent.h"`: none).
- `src/scene/Node.h` (78) is included only by Node.cpp and SceneTest.cpp:89; Node.cpp (60) is
  built (CMakeLists.txt:76). READ (grep).
- The `nodes` key (SceneSchema.cpp:539-551, :615) resolves against `ComponentSchemas()`, which
  only SceneTest fills (SceneTest.cpp:547). No engine file reads a scene's nodes. INFERRED (grep
  for readers found only the schema, SchemaDoc and SceneTest).
- Lines: 222 in the three files, about 24 in the schema. SceneTest's node tests not measured.
- `Component.h` stays: Entity, Portal, View, WaterComponent and Effect use it. READ (grep).

### 2.7 The tile tree's old fold walk (`FoldWalkForTest(kWalkNested)`)

READ: TileTree.h:1253-1258 says the walk is kept "for the tiletree selftest alone and selected
by nothing else". HIERARCHY section 6 says it goes when the new walk is accepted.

- **C++** (91): TileTree.h:1095-1098 (the dispatch), :1167-1171 (a comment about it), :1184
  (the child-only plant), :1253-1321 (`FoldUpNested`, `FoldIntoNested`), :1322-1326
  (`FoldWalkFlag`, `FoldWalk`), :1333-1338 (the switch), :415 (a comment). READ.
- **Tests** (about 150): TileTreeTest.cpp `TodayGate` (:675-769, 95 lines; its purpose is old
  against new), the plant in `DepthGate` (:527-532), the old half of `StagedRaceGate`
  (:842-924) and of `PrefillGates` (:969-979). Ranges READ; the share that would go INFERRED.
- `FoldStepHook` stays: the new walk's race staging uses it too (TileTree.h:1106). READ.

### 2.8 `TerrainLayer`, which draws nothing

READ: `TerrainLayer::Render` (TerrainLayer.cpp:43-58) returns before any draw, always: "draws
nothing: no bed of its own". `Init` still compiles `Terrain.hlsl` into a PSO at every boot
where the bathymetry loads (TerrainLayer.cpp:16, :27-37). `waterNavd` is written each frame
(FrameLoop.cpp:3409) and read by nothing (TerrainLayer.h:28 is its only other hit).

- **C++** (129): TerrainLayer.cpp (60), TerrainLayer.h (54); Assembly.cpp:39, :204, :437-441,
  :654; Assembly.h:70, :152; FrameLoop.cpp:52, :421, :511, :1744, :3409.
- **Shaders** (105): Terrain.hlsl, compiled only by TerrainLayer (grep).
- One line must change, not go: the solver starts `if (terrain && sea && ...)`
  (Assembly.cpp:470), so `terrain` is used as "the bathymetry loaded".
- It is not tied to the vertex fallback: on that path too it draws nothing.

### 2.9 The globe's vertex fallback (`--no-ms`, or a device without mesh shaders)

READ: GlobeMesh.hlsl:19 defines `GA_MESH_PATH` and the mesh path compiles `MsMain`
(GlobeLayer.cpp:722); compiling Globe.hlsl directly gives the classic `VsMain`. The fallback is
taken by `--no-ms`, by a failed mesh PSO (GlobeLayer.cpp:206-217), or by a runtime without
List6 (:2350-2353). Its PSOs are built at every boot (`BuildPso`, :202, :605-697).

- **Shaders** (70): Globe.hlsl:490-500 (`GlobeNode`) and :650-708 (`VsMain`). READ.
- **C++** (about 85): GlobeLayer.cpp:607 (the `VsMain` compile), :664-697 (the surface PSO and
  its wireframe, meshlet and lens twins), :215-217, :1457 (part), :1672-1682 (node records),
  :1784, :2020 (part), :2306-2307, :2319 (part), :2350-2353, :2392-2403 (the draw);
  GlobeLayer.h:464-471 (`NodeData`), :747, :261-262; Options.cpp:327, :785; Options.h:143;
  Assembly.cpp:646. Ranges READ; the sum mine.
- `PsMain` is compiled twice today, at ps_6_0 for the fallback (:608) and at ps_6_5 for the
  mesh path (:723). READ.

### 2.10 Vertex-shaded water (`water.pixelWater = false`, `--no-pixel-water`)

READ: the mesh stage shades every vertex with `WaterVertexColor` (GlobeMesh.hlsl:187). The pixel
stage then replaces it with `WaterPixelColor` wherever pixel water is on, the planet is not
Mars and `landness < 0.999` (Globe.hlsl:1455-1459). On Mars the vertex colour is never used
(:1460, `gStreamF.z` is "planet is Mars", Globe.hlsl:58). So on the default path the vertex
colour reaches only shore pixels with landness in [0.999, 1), at a weight of at most 0.001.

- **Shaders** (107): Globe.hlsl:527-622 (`WaterVertexColor` and its banner), :634-636 (the
  `wcol` interpolant); GlobeMesh.hlsl:180-187. READ. About 3 lines of the mix at
  Globe.hlsl:1455-1460 would simplify. INFERRED.
- **C++** (about 12): GlobeLayer.cpp:2073, GlobeLayer.h:434, FrameLoop.cpp:1168,
  Assembly.cpp:1289 (the condition), Options.cpp:417-418, :606, Options.h:181-183,
  SceneSchema.cpp:254. READ.
- The sheet's own per-pixel choice (SeaLayer.h:153-156) goes with section 2.2.
- REVIEW finding 52 (the vertex water reads the bank at the float32 point) goes with it.
- The per-vertex cost on the default path was not measured. INFERRED.

### 2.11 `--no-color-trees` and the flat compositor

READ: with the switch off, the height binds `compositor.CubeHeight` and `WindowHeight`
(Assembly.cpp:756-765), the colour binds `compositor.ColorRealization` (:960-963), and the
survey-mask tenant is not declared (:1007). The switch itself is about 20 lines: those
ternaries, Options.cpp:258-259, :623, Options.h:73-74, SceneSchema.cpp:275. INFERRED count.

The flat compositor does **not** go with the switch. Its realizations have other readers:

- the tree audit compares against it (TreeAudit.cpp:49, :61; `AuditTileTree`, TileTree.h:1729-1744);
- the `export` tool (Export.cpp:47-65);
- ComposeTest (ComposeTest.cpp:104-352, nine calls);
- Mars's height (Assembly.cpp:702), whatever the switch says;
- `HeightPage`, the CPU twin (Assembly.cpp:770), and the solver's bed
  (`RealizeFromChannel`, Assembly.cpp:433).

READ (grep for the calls). If the audit, the tool and the tests moved to the trees, about 83
more lines would have no reader: `ColorRealization` (Compositor.cpp:342-364), `CubeColor`
(:366-368), `WindowColor` (:370-374), `WindowHeight` (:436-487). `CubeHeight` (:376-434) stays
while Mars uses it. Helpers not traced. INFERRED.

### 2.12 The residency manager (being replaced; not analysed)

`src/hal/Residency.cpp` 1,842, `Residency.h` 681, `ResidencyAudit.cpp` 864,
`ResidencyAudit.h` 146: 3,533 lines. READ (wc). The `ringLoads = false` old queue
(Assembly.cpp:679), `--ds-serial` (:680) and the upload-ring side of `directStorage` live
here. The upload ring is also the only path on a machine without the DirectStorage
redistributable (CMakeLists.txt:175-177), so that side is not an A/B only.

### 2.13 Mars and the chart

Not candidates by the rule: `mars.json` and `chart.json` are shipped, and a bare `gagame`
boots the chart (Options.cpp:563-566). Named so the menu is whole: HIERARCHY 4.12 counts the
three-tenant paths of `Compose.hlsli` as reachable only for Mars.

### 2.14 The small switches

Each is a branch of a few lines. Sites READ (grep over `src`, excluding Options, the schema
and the tests); lines not measured.

| switch | sites on the non-default side |
|---|---|
| `water.swe.enabled` false | Assembly.cpp:470 (the solver is not built) |
| `water.swe.westBoundary` false | FrameLoop.cpp:992 |
| `water.bank.flatBed` true | Assembly.cpp:595-601, :620-621; WaterBankLayer.h:43 and its uses |
| `sea.datum.fromStation` false | Assembly.cpp:343 |
| `streaming.gisGate` false | Assembly.cpp:882 |
| `streaming.seafloor` false | Assembly.cpp:858 |
| `streaming.exposure` false | Assembly.cpp:781 |
| `water.wavefield.enabled` false | SceneConfig.h:34, :194; WaterComponent.cpp:326-329; FrameLoop.cpp:1121, :2635 |
| `water.closures.waterOptics` false | 8 sites (grep count) |
| `sun.source` pinned | Assembly.cpp:1291; FrameLoop.cpp:1162, :2205 |
| `portals[].lighting` appealing | FrameLoop.cpp:2305-2476, :2874, :2890 (branches on `lighting == 0` or `== 1`) |
| `--predict-inline` | FrameLoop.cpp:2952, :3166, :3804 |
| `--jobs-inline` | main.cpp:53; ThreadManager.h:71 |

---

## 3. Dead or nearly dead code without a switch

How each was checked is said beside it.

1. **`SeaLayer::SetBathyCpu` and `m_bathyCpu`**: written, never read. Grep for `SetBathyCpu(`
   and `m_bathyCpu` over `src`: the setter (SeaLayer.h:73), the member (:314), one call
   (Assembly.cpp:547). 3 lines. READ.
2. **`TerrainLayer::waterNavd`**: written (FrameLoop.cpp:3409), never read; grep finds only the
   declaration (TerrainLayer.h:28). Counted in 2.8. READ.
3. **`GulfLayer::MvSrv()`**: no caller (grep for `MvSrv(`). Counted in 2.3. READ.
4. **`TreeWater::SetBoats`** and the CPU wake term: no caller (section 2.1). READ.
5. **`hal::Retire`**: `src/hal/Retire.h` (86). Grep for `Retire(` finds its definition
   (Retire.h:56) and no call; the other hits are comments (Pipeline.h:25, Context.h:21) and the
   tree-prune tool's own "retire", which is another thing. `DrainRetired` is called once
   (Gpu.cpp:320-321) and drains a queue nothing fills. 88 lines. READ. REVIEW section 5.1.
6. **The `PageTable` class**: PageTable.h:87-195 (109). Used only by GaTest.cpp:845-919 (the
   "M9h: THE PAGE TABLE" block, 75). `PageAddr` (:37-52) and `LevelLadder` (:60-77) in the same
   file ARE used (Assembly, GlobeLayer, Lattice, DomainSource, ColorStackSource). Checked by grep
   of each name. READ. REVIEW section 5.1.
7. **`scene/LayerComponent.h`**: included by nothing (section 2.6). READ.
8. **Shader functions nothing calls.** Checked by listing the 241 functions defined at column 0
   in `shaders/` and grepping each name across `shaders/` and `src/`. The entry points came up
   too (they are named only by C++ compile calls) and are left out. What is left, with no
   reference anywhere (READ):
   - `FieldUv` (Common.hlsli:116-119), `SampleFieldPoint` (:129-135), `FieldOverrun` (:139-147);
   - `SunThroughAir()` (Common.hlsli:244-247; `SunThroughAirAt` is used);
   - `SeafloorReliefMod` (Compose.hlsli:198-208; its only C++ mention is a comment,
     Sources.h:194);
   - `ScalarPart`, `BivectorPart` (GA.hlsli:52-53).
   37 lines. The regex saw only functions returning built-in types, so functions returning a
   struct were not checked. INFERRED that no more exist.
9. **Files not in `CMakeLists.txt`**: none. Every `.cpp` under `src` is listed (a loop over
   `find src -name '*.cpp'` against the file). READ.
10. **Tools nothing dispatches**: none. Every `Run*` in Tools.h has one call site outside
    `src/app/Tools/`, and every tool name `ToSets` writes (Options.cpp:731-751) is dispatched
    (`Tool("...")` greps; `wave-map` through FrameLoop.cpp:1094, `load-field` through
    main.cpp:114). Finding 37 is mended: the tree tools dispatch (Assembly.cpp:675). READ.
11. **Scene keys nothing reads.**
    - `views[].viewport`: parsed (Scene.cpp:207-211) and read by nothing else (grep for
      `.viewport`). The schema's own comment says so (SceneSchema.cpp:37-38). READ.
    - `layers[].enabled`: read only to build the declared order (Scene.cpp:169). It does not
      enable or disable a layer. INFERRED (grep).
    - `nodes`: see 2.6. INFERRED.
12. **A stale comment**: WaterBank.hlsl:391-393 says "Globe.hlsl carries the SAME function"
    (`CardPattern`); grep finds it only in WaterBank.hlsl (:394, :866). READ.

---

## 4. Duplicates: one law written more than once

1. **The Mercator closed form.** The review found three copies beside the lattice's; there are
   more (READ, grep for `log(tan`, `atan(sinh` and the sine form):
   - latitude to y: `Lattice.h:235` (the lattice's own), FrameLoop.cpp:3263, WeatherManager.cpp:197
     (in :188-200), GlobeLayer.cpp:1165-1189 (the same law through the sine), Sources.cpp:70,
     SurfaceFrame.cpp:43, WaterAtlas.cpp:393, :513, Trace.cpp:101, GaTest.cpp:291, :303;
     on the GPU PageSample.hlsli:81 and :89 (the "two spellings of the page uv", HIERARCHY 4.12).
   - y to latitude: Lattice.cpp:72-73, :97; Compositor.cpp:451-452, :468, :503-504, :520;
     WaterAtlas.cpp:395, :532; Trace.cpp:140; ComposeTest.cpp:153.
2. **The cascade band cuts** `{2 pi/756, 2 pi/60, 2 pi/12, 0.9 pi N/47}`: SeaLayer.cpp:316,
   :527; WaterBankLayer.cpp:627; FrameLoop.cpp:2808 (`kCutB`). The patch sizes
   `{756, 186, 47}`: OceanFft.h:91 and GlobeLayer.h:733. READ.
3. **The planet's radius.** Named twice, `GlobeModel::kR` (GlobeModel.h:20) and
   `kEarthRadiusM` (Ephemeris.h:62), and written as `6371000` at Sources.cpp:603, :620-621,
   :656-657, :775, :797-798; Droste.h:87, :352; Renderer.h:81, :172; SkyLayer.h:88;
   WaveChart.h:62; Globe.hlsl:34; Sea.hlsl:423, :526; Terrain.hlsl:44. The WGS84 `6378137` of the
   projections (DomainSource.h:251, Projections.h:32, :72, GeoRef.h:146) is a different
   constant, rightly. READ.
4. **Cube face to direction, in two orientations.** `ComposeCubeDir` (Lattice.cpp:18) has v
   pointing down (`p.y = -t`); `CubeDirD` (GlobeLayer.cpp:150) and `CubeDir` (Globe.hlsl:506)
   have v pointing up (`p.y = cy`). One cube, two conventions, three bodies. READ. Whether the
   step-3 `CubeFaceAxes` is a fourth was not read.
5. **The Kelvin wake**: WaterBank.hlsl:308-389 and WaterTerms.h:262-497, a deliberate CPU twin
   (WaterTerms.h:2-35), whose only caller never runs (section 2.1). READ. HIERARCHY 4.12's
   "water written twice by hand" is the larger case of the same thing (the bank kernel and
   `TreeWater`); not re-counted here.
6. **String narrowing and widening**: Options.cpp:20, :489; TreePrune.cpp:844, :851;
   SceneConfig.h:384, :391; ResidencyAudit.cpp:314; Tenant.cpp:42. Five narrowings, three
   widenings. READ (definitions only).
7. **The sphere's drop `d^2 / 2R`**: Sea.hlsl:423, :526 and Terrain.hlsl:44, both in files that
   go with rows 3 and 9. READ.
8. **The globe's pixel stage compiled twice** (GlobeLayer.cpp:608 and :723): a duplicate
   compile, not a duplicate law. Goes with row 11. READ.

---

## 5. What each removal would cost

One line each: what is lost, then what must be true first.

1. **Residency manager**: being replaced; the new manager must pass HIERARCHY 4.19's gates first.
2. **Fleet and wakes**: loses the AIS floats and the only wake the renderer can draw; first,
   decide that the hull's own wake (HIERARCHY step 12) will not reuse this closed form, or keep
   `WaterTerms.h`'s wake for it.
3. **The sea's sheet**: loses the M5b sheet, the spectrum plot (model against buoy 44013) and
   the escape hatch M9bp kept; first, `SeaLayer::Simulate` and `Render` must stop gating on
   `m_seaPso` (SeaLayer.cpp:795, :810), and the churn's and the solver's rows of `SeaCbData`
   must be split from the draw's.
4. **Gulf map**: loses the Gulf of Maine view, its GoMOFS eddy picture and the 44029 check in
   its title; first, the owner says the map is not wanted, and TAB cycles two views.
5. **Slice plane**: loses the M7o cutaway and the effects list's only type; first, decide the
   effect seam waits for a real effect.
6. **GIS vector layer**: loses the `--stencil` alignment overlay, an instrument for the coast;
   first, decide it is not needed as a gate.
7. **Typed-node seam**: loses the plugin seam's first half and its SceneTest; first, decide
   whether the component future (HIERARCHY 4.9) builds on it.
8. **Old fold walk**: loses the byte reference and the planted failure the stripe count trips
   on; first, the owner accepts the new walk (HIERARCHY section 6 says so).
9. **TerrainLayer**: loses nothing drawn; first, Assembly.cpp:470 tests the bathymetry itself,
   and the eleven scene files may drop `terrain` from `layers` (the order check skips a layer
   that is not built, Scene.cpp:443-446).
10. **PageTable class**: loses the M9h address-space model and its gatest block; first,
    ARCHITECTURE section 3 stops describing it as the tenant's page table (REVIEW 5.1).
11. **Vertex fallback**: the engine then needs a mesh-shader device and the List6 runtime;
    first, the owner says no such machine is a target.
12. **Vertex-shaded water**: loses the M9bg look as an A/B; first, accept that shore pixels with
    landness at or above 0.999 take the pixel colour (weight at most 0.001; Mars untouched).
13. **`hal::Retire`**: loses nothing that runs; first, the deferred-release contract the
    comments describe (Context.h:21, Pipeline.h:25) is restated or built another way.
14. **Dead shader helpers**: loses nothing; first, nothing.
15. **`--no-color-trees`**: loses the A/B render against the flat compositor; the audit,
    `export`, ComposeTest and Mars still use the compositor, so it stays (2.11).
16. **`SetBathyCpu`**: loses nothing; first, nothing.
17. **Small switches**: each loses one A/B or diagnostic; first, the owner names which ones his
    gates still use.

Cross-reference. HIERARCHY 4.12 lists what the refactor itself will delete (the Mercator window
as a lattice kind, the page rows, the anchor chart's consumers, ring sets A and B as two things,
the second Mercator path in Compositor.cpp). Those are not switch-gated and are not re-counted
here; the Mercator copies of section 4 are the same family.

---

## 7. What I did not read

- `src/hal/Residency*`: by instruction; line counts only.
- `FrameLoop.cpp` beyond the ranges named (about 300 of 4,119 lines); `Globe.hlsl` beyond
  270-285, 470-720, 1105-1135 and 1425-1485; `GlobeLayer.cpp` beyond 146-222, 600-700,
  1160-1192, 1665-1685 and 2340-2405.
- The bodies of `Sea.hlsl`, `SpecPlot.hlsl`, `Terrain.hlsl`, `Gulf.hlsl`, `VelGrad.hlsl`,
  `GisVec.hlsl`, `GulfLayer.cpp`, `GisLayer.cpp`, `SlicePlane.*`, `Node.*`,
  `LayerComponent.h`. They are counted whole and attributed by their only compile or include
  sites.
- `SeaLayer.cpp` 463-781: so which rows of `SeaCbData` only the sheet reads is not known.
- `WaterComponent.cpp`'s fleet list handling, and the WaterBank.hlsl `WakeOne` body: whether a
  disabled slot returns early is not known.
- `SceneTest.cpp`, `ComposeTest.cpp`, `ThreadTest.cpp`, `DxTest.cpp` beyond greps and the
  layout table; so the tests that would go with rows 5 and 7 are not measured.
- The small switches' branches (2.14): sites only.
- Whether any tool or script outside `tools/` passes a `--no-*` flag. Inside `tools/`, none
  does (grep).
- The rails' own code (`Rail.cpp`): whether the classic, zoom, jetty and droste rails need
  code beyond their data files.
- `docs/`, `harvester/` and `proofs/` Python: not counted, by instruction.
