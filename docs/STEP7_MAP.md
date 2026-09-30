# Where the height, the exposure and the solver's bed are read: the work list for step 7

A read-only map made on 2026-09-29 by an agent, for `docs/HIERARCHY.md` section 6, step 7.
It proposes and does not decide. **The lines are not `main`'s:** the tree that was read is
`main` at `71e3a33` with the migration of the colour and the mask applied and not committed
(4.17's commits 1 to 4), so a line cited here may stand some lines off in `main`.

*A read-only mapping, 2026-09-29, for the engineer who briefs step 7's agents. TREE is
`out\integration\tree_m`: `main` at 71e3a33 plus an uncommitted diff of 32 files, +4,945 and
-108 (READ: `git diff --stat HEAD`). That diff holds the colour-and-mask migration's commits 1 to 4
and, by its file names, step 2's probe (`ResidencyFloor.hlsl`, `ResidencyTest.cpp`; INFERRED).
Line numbers are TREE's unless a path says otherwise. Each statement ends with READ (I read the
lines cited) or INFERRED (reasoned from what I read, not seen in code and not run). Nothing was
built or run. "The page" is the z14 Mercator window at slice 6 of a tenant. "Blocks" are slices
6 + i under the scene key `streaming.faceWindows` (shader define `GA_BLOCK_RANKS`). "Finding N"
is `docs/REVIEW_2026-09-28.md`; "4.x" is `docs/HIERARCHY.md`.*

## 0. What matters most for the design

1. **The exposure is read through the height's rows.** Both GPU readers of the exposure take the
   HEIGHT's slice and the height's window row: `Sea.hlsl:114-116` (`CsWindowUv` over `gCsMerc`,
   slice `gCsU6.z`) and `WaterBank.hlsl:486-491` (`gWinA`, slice `gSlotsD.z`). No exposure slice
   is handed to either (`WaterBankLayer.cpp:652-656`; `SeaLayer.cpp:538`, `:573`). The bank also
   reads no exposure at all unless the height's slice is set (`WaterBank.hlsl:486`). This is the
   code behind 4.17's "the height and the exposure move together". READ.
2. **The three compute kernels cannot call the colour's block code.** `Swe.hlsl`,
   `SeaChurn.hlsl` and `WaterBank.hlsl` do not include `Compose.hlsli`: they have no surface rows
   (b2), no directory and no walk. They address by latitude and longitude on their own lattices,
   read by Loads, and take a page row and a slice through their own constant buffers and bindings
   (`Swe.hlsl:40-53`; `SeaChurn.hlsl:30-45`; `WaterBank.hlsl:34-37`, `:174-176`). Swe and SeaChurn
   bind the height and its residency map at fixed registers (`Swe.hlsl:52-53`,
   `SeaChurn.hlsl:44-45`; finding 76). READ. One function is owed: a block form of
   `HpHeightAt`, with rows that each kernel's C++ fills. INFERRED.
3. **The mesh and domain stages read the height with bindless `SampleLevel`, which "priors 1"
   says returns zero there.** `GlobeMesh.hlsl:102`, `:116` and `Sea.hlsl:490` reach
   `PageSampleLevelCube` / `PageSampleLevel` through `ComposedHeight` (`Compose.hlsli:392`, `:400`).
   `docs/ALGEBRA.md:1042-1047`, `Globe.hlsl:538-541` and `WaterBank.hlsl:169-173` say a bindless
   `SampleLevel` returns zero in the compute and mesh stages. READ (both sides). Which holds on
   today's binary: NOT determined. It decides whether the vertex-stage height readers may reuse
   `PageSampleLevel` on blocks or must read by Loads.
4. **The solver's bed is the whole height array, addressed by Mercator from latitude and
   longitude, read at whatever is resident.** `SweSolver::SetHeightPage` binds every slice and
   every mip (`SweSolver.cpp:34-35`). `BedAt` maps a lattice cell to lat/lon (`Swe.hlsl:80-81`)
   and calls `HpHeightAt` with `pageMipMin` 0 (`:82`), whose mip is one residency byte
   (`HeightPages.hlsli:53`, `:58`): physics reads residency (finding 48). `PinDomains` asks slice
   6 at mip 0 over the domain's box in the page's uv, and only when Boston's block ran
   (`WeatherManager.cpp:186-227`; `FrameLoop.cpp:1053-1058`; findings 17, 70). READ. The
   instrument that sees the bed (the bed trace) and the wait for it live in the `solver-bed`
   worktree, not in TREE. Its `HeightPages.hlsli` hunk is written against a file in which
   `HpCubeFace` still stands; TREE moved `HpCubeFace` to `PageSample.hlsli`
   (`PageSample.hlsli:170-190`), so the hunk will not apply as it is. READ (both files); INFERRED
   (the conflict).
5. **An exposure tenant cannot be declared with blocks alone, and the level it is read at is a
   rank-1 block's.** `Tenant.cpp:248` refuses a tenant with no lattice binding, and
   `Tenant::Bind(tree)` gives the tree the first binding's lattice (`Tenant.cpp:415`). Today the
   exposure's only binding is the page on `winH` (`Assembly.cpp:822`), and its cube slices are
   unbound and answer 1.0 (`Assembly.cpp:815-819`). READ. The exposure is read at mip 3 and
   coarser of the page (`WaterBank.hlsl:491`, `Sea.hlsl:120`, `WaterTerms.h:85`), which is rung 3:
   the mip 0 of a rank-1 block (4.3's ladder). READ (the reads); INFERRED (the equivalence).

Two more that will bite step 7. The per-node wants carry at most four blocks
(`GlobeLayer.cpp:1518`; `GlobeLayer.h:189-191`) while the key takes eight (`SurfaceFrame.h:117`).
And the close-up land material is gated on the height's Mercator window (`Globe.hlsl:1485-1487`),
so moving the height moves where that material is drawn unless the gate is restated. READ.

## 1. Every reader of the height tenant

### 1.1 The tenant

- Declared at `Assembly.cpp:757-779`: R16_FLOAT in 256x128 tiles (`:760`), Texture, Streamable,
  Unloaded (`:761-763`), 7 slices (`:764`): 0..5 on `surface.cubeH`, 6 on `surface.winH`, the z14
  page (`:767-778`). The provider is the height tree on each binding's lattice, or with
  `--no-color-trees` the compositor's `CubeHeight` and `WindowHeight` (`:768-777`). READ.
- Seven mips: the rows say max lod 6 for the cube and the page (`SurfaceFrame.cpp:359-363`), and
  `BlockBinding::Mips` gives 7 for 256x128 tiles (`Tenant.h:197-200`). READ.
- `heightTenant.Bind(*heightTree)` (`Assembly.cpp:785`) gives the tree the cube's lattice
  (`Tenant.cpp:415`). A fold on the page's lattice reaches slice 6 by its tag
  (`Tenant.cpp:158-175`, `:216-229`). READ.
- Its tile index scans `cube16k` only (`Assembly.cpp:1145`, `:1155`). READ.
- Mars declares a height cube with no page (`Assembly.cpp:710-719`); every reader below that tests
  for a page must keep working with none. READ.

### 1.2 The binders: the C++ that hands the tenant to a shader

| # | binder | call site | what it hands | where the shader sees it | mark |
|---|---|---|---|---|---|
| B1 | `SurfaceFrame::Fill` | every frame, `FrameLoop.cpp:3207` | `u6` = array SRV, residency-array SRV, the page's slice, ~0 (`SurfaceFrame.cpp:334-338`); `u2` = cube SRV and cube residency SRV, and ~0 for the old window (`:339-344`); `merc` = the COLOUR's `win.Rows` (`:358`), which has the height page's origin (`:196`, `:198`); `g` = 6, texel arc, 6, stencil (`:361-364`); `ground` (`:392-395`) | b2 `SurfaceCb` (`Common.hlsli:428-431`), rows `gCsU2`, `gCsU6`, `gCsMerc`, `gCsG`, `gCsGround` (`:385-401`); textures by index into `gTexArr[]` space5 and `gTexCubeArr[]` space6 (`:101`, `:103`) | READ |
| B2 | `SweSolver::SetHeightPage` | `Assembly.cpp:788-792` (slice `6u`, `surface.winH`) | table slot 0 = the array over every slice and mip, slot 1 = the residency array (`SweSolver.cpp:34-35`; table of 6 at `:294`); rows `geoLL` = the survey lattice as lat/lon (`:37-40`), `winA` = `winH.Rows` (`:43`), `pageB.x` = the slice (`:44`) | `t1 gBathy`, `t2 gBathyRes`; `SweCb` `gGeoLL`, `gWinA`, `gPageB` (`Swe.hlsl:40-42`, `:52-53`) | READ |
| B3 | `WeatherManager::SetHeightPage` | `FrameLoop.cpp:1053-1058`, inside the Boston block | the same, for the owned solvers at activation (`WeatherManager.cpp:93`) and for `PinDomains` (`m_hgtWin`, `m_hgtSlice`; `WeatherManager.h:78-84`, `:148`) | as B2 | READ |
| B4 | `SeaLayer::SetHeightPage` | `Assembly.cpp:839-843` (`6u`, `winH`) | churn table slots 2 and 3 = the array and the residency array, every slice (`SeaLayer.cpp:697-700`); churn rows `winA`, `pageB.x` (`:747-748`), `geoA` = the flat chart (`:746`) | `t3 gBathy`, `t4 gBathyRes`; `ChurnCb` `gGeoA`, `gWinA`, `gPageB` (`SeaChurn.hlsl:30-32`, `:44-45`) | READ |
| B5 | `WaterBankLayer::SetHeightWindow` | `FrameLoop.cpp:2818-2831`, bank sets A and B (`6u` when the window is the height tenant) | `slotsD` = array SRV, residency SRV, slice, ~0 (`WaterBankLayer.cpp:666-669`); `winA` (`:674`); `geoA` (`:673`) | `BankCb` `gSlotsD`, `gWinA` (`WaterBank.hlsl:34-37`); textures by index into `gTA[]` space5 (`:175`) | READ |
| B6 | `GlobeLayer::SetSurface` | `GlobeLayer.cpp:1446-1464` | the tenant id and the page's slice, for wants only (`:1454-1456`) | none | READ |

### 1.3 The GPU readers

Stages: VS vertex, MS mesh, DS domain, PS pixel, CS compute. "Gather" = `GatherRed` of four
residency bytes and their largest (`PageHave`, `PageHaveCube`: `PageSample.hlsli:107-114`).
"Byte" = one `Load` of the byte under the uv (`PageHaveLoad`: `:117-120`). "4 Loads" = manual
bilinear at one mip, taps clamped to the slice and not to what is resident (`PageLoad4`:
`:123-139`; finding 9).

| # | where | stage | slices | address | read | residency | what the value does | mark |
|---|---|---|---|---|---|---|---|---|
| H1 | `ComposedHeightPages`, `Compose.hlsli:390-404` (the pages path of `ComposedHeight`, `:406-425`) | any graphics stage that calls it (H3 to H16) | cube 0..5 by the cube view `gCsU2.x`; the page by the array view `gCsU6.x`, slice `gCsU6.z` | cube: the direction; page: `CsWindowUv(dir)` = `PageUv(dir, gCsMerc)` (`:50`; `PageSample.hlsli:77-83`), Mercator from `asin`/`atan2` of the float32 direction, strict containment | `SampleLevel`, `sLinearClamp`: cube at `max(lod, haveC)`; page at `max(clamp(lod + 6, 0, gCsG.z), haveW)` (`:392`, `:396-400`) | gather on `gCsU2.y` and on `gCsU6.y` at the page's slice (`:391`, `:397`) | returns the height; the page wins where its resident ground is at least as fine as the cube's (`:399`) | READ |
| H2 | `ComposedHeight`'s old path, `Compose.hlsli:409-423` | as H1 | a cube texture and a separate window texture (`gCsU2.z`, `.w`) | `CsWindowUv`, 6 % feather | `SampleLevel` | gather (`CsHaveCube`, `CsHave2D`) | Mars's cube only today (`:409-410`); the window part is unreachable on Earth's pages path (`SurfaceFrame.cpp:343-344` writes ~0) | READ; INFERRED (unreachable) |
| H3 | `ComposedHeightGrad`, `Compose.hlsli:585-596` | as caller | as H1 | four directions rotated by eps = texel arc x 2^lod | four `ComposedHeight` | as H1 | east and north slope | READ |
| H4 | `VsMain`, `Globe.hlsl:674-675` (fallback pipeline, `#ifndef GA_MESH_PATH`, `:650-708`) | VS | as H1 | the vertex's direction; no block chain in this function | H1 at `max(ComposedHeightLod, 3)` | as H1 | displacement (`:700`) and `WaterVertexColor(h)` (`:704`) | READ; whether this pipeline runs by default NOT READ |
| H5 | `SurfaceVertex`, `GlobeMesh.hlsl:98-102` | MS | as H1 | the vertex's direction | two H1 reads (lod -8 near, `vlod` far), morphed by `k` | as H1 | relief displacement (`dispLand`, `:125`) | READ |
| H6 | `GlobeMesh.hlsl:115-116` | MS | as H1 | as H5; the block chain is walked at `:113`, AFTER H5 | H1 at `max(vl, -4)` | as H1 | input to `ComposedLandness`: the vertex's land and water mix | READ |
| H7 | `GlobeMesh.hlsl:193` into `WaterVertexColor`, `Globe.hlsl:545` | MS | none new | the H5 value | none | none | the shelf tint (`Globe.hlsl:543-544`) | READ |
| H8 | `PsMain` `hp`, `Globe.hlsl:1158` | PS | as H1 | `up` = the interpolated direction (`:1105`) | H1 at `ComposedHeightLod` | as H1 | the land's albedo (`:1202`), the planet's shadow `PlanetShadow(upT, sun, hp)` (`:1162-1163`), the pixel water's depth `lvlW - hp` (`:981`, passed at `:1471`) | READ |
| H9 | `PsMain` `hpC`, `Globe.hlsl:1177-1180` | PS | as H1 | as H8 | H1 at `max(lod, -5)` | as H1 | input to `ComposedLandness` | READ |
| H10 | `Globe.hlsl:1198` | PS | as H1 | as H8 | H3 at `lod` | as H1 | the land's normal (`:1200-1201`) | READ |
| H11 | `Globe.hlsl:1485-1490` | PS | as H1 | gated on `CsHeightWindowOn()` and `CsWindowUv(up)` inside the page | H3 at lod -8 | as H1 | the close-up material: wet sand, flats, dune grass, riprap by slope | READ |
| H12 | the refracted cast, `Globe.hlsl:1014-1019` (`--pixel-water`, inside a branch) | PS | as H1 | a direction built from the eye-free cast (`:1016`) | two H1 reads at `lod` | as H1 | where the bed is landed on; then the bed's colour walks its own chain (`:1025-1026`) | READ |
| H13 | lenses 2 and 3, `Globe.hlsl:1384-1393` | PS | the page | `CsWindowUv(up)` | none | lens 3: gather at the page (`CsHaveHeightWin`, `Compose.hlsli:382-385`) | debug colour | READ |
| H14 | the residency lens, tenant 1, `ResidencyLens.hlsl:137-151` | PS | cube and page | `CsWindowUv` (`:77`) | none | gather, cube (`:142`) and page (`:144`) | which page answers and its floor; no block branch for the height | READ |
| H15 | the stencil, `Compose.hlsli:551-568` (`--stencil` only) | PS (`Globe.hlsl:1261`, `:1627`; `Terrain.hlsl:106`) | as H1 | as caller; draws the page's frame from `CsWindowUv` (`:557-562`) | H1 at lod -8 (`:553`) | as H1 | the red coastline overlay | READ |
| H16 | the sea sheet's `BedAt`, `Sea.hlsl:68-80` | DS (`DsMain`, `:490`) and PS (`SeaPixelColor`, `:427-430`) | as H1 | `SeaPlanetDir(xz)`: flat frame to a direction by the curvature drop (`:60-63`) | H1 at -8 inside the survey box, -2 outside (`:78`) | as H1 | DS: depth for dispersion, shoaling, breaking, `dryGuard` (`:494-521`); PS: the refracted ray's landing | READ. The sheet draws only when `water.oneWater` is false (`Assembly.cpp:642`; `SeaLayer.cpp:824-831`) |
| H17 | the sea sheet's land cut, `Sea.hlsl:556-557` | PS | none new | the DS depth | `ComposedIsLand(dir, gSea.x - i.sh.x, ...)` | none new | discard over land; `ComposedLandness` reads the old window row `gCsU2.z` (`Compose.hlsli:516`) | READ |
| H18 | the solver's `BedAt`, `Swe.hlsl:76-83` | CS | the whole array (`t1`), its residency (`t2`) | cell centre to lat/lon by `gGeoLL` (`:80-81`), then H21 with `gWinA`, slice `gPageB.x`, floor 0 | H21 | H21 | the bed of every substep: `SurfaceAt` (`:92-97`) in `FaceUpdate`'s depths and sill (`:184-186`, `CsSweFlux`), the west boundary (`:223-227`), `CsSweHeight` (`:273`, `:280`), `CsSweDerive` (`:296`). Outside the lattice a 100 m wall (`:77`) | READ |
| H19 | the churn's `PageBedAt`, `SeaChurn.hlsl:53-57` | CS (`CsChurnUpdate`, `:102-106`) | `t3`, `t4`, every slice | world metres to lat/lon by the flat chart `gGeoA` (`:54-55`), then H21 | H21, floor 0 | H21 | depth for the chop's phase speed and its blocking; only inside the survey box and when the solved field is on (`:102-104`), else 30 m (`:101`) | READ |
| H20 | the bank's bed, `WaterBank.hlsl:429-446` | CS (`CsBankFill`, `:401-416`) | `gTA[gSlotsD.x]`, `gTA[gSlotsD.y]`, every slice | lat/lon from the tile's place rows (`TilePlace`, `:132-139`, `:416`), then H21 | H21, floor = the ring's grain: `floor(log2(texelM / (kHpPageTexelM cos lat)))` (`:443-444`) | H21 | depth (`:474`): dry weight, shoaling, breaking. Falls back to a CPU corner lerp (`:425`); `--flat-bed` replaces it (`:473`) | READ |
| H21 | `HpHeightAt`, `HeightPages.hlsli:45-64` (the rule H18 to H20 share) | CS | cube face from `HpCubeFace` (`PageSample.hlsli:174-190`); the page by strict containment | `PageUvLatLon(lat, lon, winA)` (`PageSample.hlsli:86-91`) | 4 Loads at the chosen mip (`:60`, `:63`) | byte, cube (`:53`, clamped to 0..6) and page (`:58`, clamped to floor..6) | page where `PageWins` on resident grounds (`:59`), else the cube | READ |

Not a reader: the terrain layer samples its own survey texture (`Terrain.hlsl:25-28`, `gTSrv.x`),
not the tenant. READ. Only the functions above reach `gCsU2`, `gCsU6` or `HeightPages.hlsli`
(grep over every shader). READ.

**The priors-1 question.** `PageHave` gathers and `PageSampleLevel` samples in the mesh and
domain stages through H1 (H5, H6, H16). Priors 1 (`docs/ALGEBRA.md:1042-1047`) says a bindless
`SampleLevel` returns zero in the compute and mesh stages; the kernels obey it with Loads
(`WaterBank.hlsl:169-173`), and the exposure's domain-stage read says it obeys it too
(`Sea.hlsl:109-111`). The mask's block reads in the mesh stage (`GlobeMesh.hlsl:116` into
`CsMaskSample`, `Compose.hlsli:448`) are also `SampleLevel`, and commit 2 was gated by pictures.
READ. What makes the one work and the other fail: NOT determined. A readback of H5's value
against H8's at the same ground would decide it. INFERRED.

### 1.4 The CPU readers of the same lattice (twins; none reads the tenant)

- `HeightPage` (`HeightPage.h:41-116`), made on `winH` and `cubeH` (`Assembly.cpp:782`), handed
  to hulls as `eo.bed` (`FrameLoop.cpp:1387`), read by `TreeWater::BedAt`
  (`WaterSurfaceTree.cpp:239-240`, `:271`). It is H21's rule at the finest level: the page by
  strict containment (`HeightPage.h:53-63`), else the cube face (`:64-70`), each texel the CPU
  stack at the texel's centre with `GroundRes(0)`, quantized as half (`:97-109`). It reads no
  residency. READ.
- `ExposureSource` marches the CPU height stack, not the tenant (`ExposureSource.h:122`, `:153`).
  READ.
- The wave solve takes its bed from the CPU stack at its cell centres through the anchor chart
  (`WaveField.cpp:1088-1106`). READ.
- The solver's own CPU grid (`BathyModel::Elev`, the survey file) decides its static residency
  (`SweSolver.cpp:104-132`), its time step (`:189-195`) and its west section (`:221-229`,
  `:607`), while its kernels integrate on the tenant (H18). READ. Two beds in one solver: the
  class of finding 69. INFERRED.
- `--trace` step 11 reads one GPU texel of the height page back and holds it against the CPU
  stack (`Trace.cpp:96-153`). READ.
- The bank's corner bed (`WaterBank.hlsl:107`, `:425`) is filled on the CPU; the fill in
  `WaterBankLayer.cpp` was NOT READ.

### 1.5 Who asks for the height's tiles (the wants)

- The globe's node walk: the cube at the node's mip (`GlobeLayer.cpp:1142`); the page by the
  node's box in z14 pixels (`:1144-1208`, the height at `:1206`); the blocks for the colour and
  the mask only (`:1230-1268`). The height has no block wants yet. READ.
- The floors: the page at mips 4 to 7 every frame (`GlobeLayer.cpp:2175-2178`); the height
  carries mips 0 to 6, so mip 7 is past its chain (what `Want` does with it NOT READ). The blocks'
  floors are the colour's and the mask's only (`:2185-2190`). READ.
- `PinDomains` (`WeatherManager.cpp:186-227`, called every frame at `FrameLoop.cpp:2602`), section
  4.3. READ.
- The interests: the bed at mip 0 over each interest's box in the page's uv
  (`FrameLoop.cpp:3364-3385`, the bed at `:3381-3383`). READ.
- `--warm-inlet` (`WarmInlet.cpp:46-52`). READ.
- `solver-bed`'s `WaitForBeds` (its `WeatherManager.cpp` diff): the pin's rectangles at mip 0
  every turn until the trace reads mip 0 of the page in every cell. READ (the diff).

## 2. Every reader of the exposure tenant

### 2.1 The tenant and its binders

- Declared at `Assembly.cpp:793-838`, only when there is a sea, a height channel and
  `S.streaming.exposure` (`:793`). The node is `ExposureSource` over the height channel, in a
  `DomainCompositor` (LayeredOver), as the composite `swell.exposure`, painted by a Half
  `TileTree` held in a holder (`:794-800`). The tenant: R16_FLOAT 256x128, Texture,
  Recomputable, OutOfDomain (`:810-814`); unbound slices answer 1.0 (`0x3C00` halves,
  `:815-819`); 7 slices (`:820`); one binding, slice 6 on `winH`, painted by the tree in the
  holder (`:821-823`). `Bind(**exposureTree)` (`:826`). READ.
- Binders: `sea->SetExposurePage(array SRV, residency SRV, node)` (`Assembly.cpp:827-828`;
  `SeaLayer.h:89-95`) gives the sea's rows `sweU[3]` = array, `churnU[3]` = residency
  (`SeaLayer.cpp:573`, `:538`) and the bank's `slotsC[1]`, `slotsC[2]`
  (`WaterBankLayer.cpp:654-655`). No binder hands a slice. READ.

### 2.2 The GPU readers

| # | where | stage | slice | address | read | residency | use | mark |
|---|---|---|---|---|---|---|---|---|
| E1 | `WaterBank.hlsl:483-494` | CS (`CsBankFill`) | `gSlotsD.z`, the HEIGHT's | `PageUvLatLon(place, gWinA)`, the height's row, strict containment | 4 Loads at `max(have, 3)`, floored at 0.18 (`:491`) | byte (`:489`); above 6 means none: exposed | the ocean bands fold by exposure (`:477-482`) | READ |
| E2 | `SweShadow`, `Sea.hlsl:112-132` | DS (`:495`) and PS (`:390`) of the sea sheet | `gCsU6.z`, the HEIGHT's (`:116`) | `CsWindowUv(SeaPlanetDir(xz))`, `gCsMerc` (`:114`) | 4 Loads written out, mip `max(round(have), 3)` (`:120-131`) | byte (`:117-118`); above 7.5 means exposed | the bands' amplitude (`BandScale`) | READ. Draws only when `water.oneWater` is false (`Assembly.cpp:642`) |

No lens reads the exposure: the residency lens has the colour, the height and the survey only
(`ResidencyLens.hlsl:89`, `:137`, `:152`). READ.

### 2.3 How it is composed on the CPU

- `ExposureSource::SampleAt` (`ExposureSource.h:111-167`): where the CPU stack's height at the
  texel centre stands above the bucket's level, exposed (`:122-125`). Else five rays at 0, +-13
  and +-26 degrees toward the source, a step of `max(13 m, 0.35 groundM)` to 4 km (`:127-128`,
  `:139-156`), the worst blocker above the level mapped to a transmission, and the best weighted
  ray kept (`:157-165`). READ.
- Its identity: the program, the direction bucket (5 degrees), the level bucket (0.25 m) and the
  height stack's names and structures (`:78-90`). A tile painted across a bucket change is
  refused (`:104-109`). READ.
- The CPU twin `ExposurePage` (`ExposurePage.h:42-113`), made on `winH`, slice 6, mip 3
  (`Assembly.cpp:832-833`): the node asked at the page's mip-3 texel centres, bilinear, memoized
  per bucket (`ExposurePage.h:58-62`, `:84-105`). Hulls read it as `eo.swellShadow`
  (`FrameLoop.cpp:1386`; `WaterSurfaceTree.cpp:245`). `SeaLayer::ShadowAtWorld` asks the node at
  76 m for the trace (`SeaLayer.h:124-128`). READ.

### 2.4 What invalidates it, and which tiles

- **The bucket roll** (`FrameLoop.cpp:3253-3261`). Every frame `Set(peak direction, waterNavd,
  valid)` (`:3254-3255`; `ExposureSource.h:49-60`). When the direction's 5-degree bucket or the
  level's 0.25 m bucket changes: a new tree over the same root, `Bind` (`:3257`), an atomic swap
  into the holder (`:3258`) and `resMgr.Drop(exposureT)`: every tile of the tenant (`:3259`).
  READ.
- **A tree's change** (`TileTree.h:1467-1470` `InvalidateAbove` calls `onChanged(tag, r)`, hung
  by `Tenant::Bind`, `Tenant.cpp:415-417`). `Changed` turns it into
  `{SliceOf(tag) + r.face, mip, x, y}` = slice 6 and calls `ResidencyManager::Invalidate`
  (`Tenant.cpp:216-233`). The tile is the coarser one a fold rewrote (`FrameLoop.cpp:4083-4086`).
  READ. Finding 3's 210 invalidations on the storm rail, 65 over a mapped descendant, were these;
  `DropOne` then raises the bytes over the parent's whole footprint (the review's
  `Residency.cpp:728-745`, `:804-811`; NOT READ here). Why the exposure and not the height: its
  identity rolls with the swell and the tide, so its tree is painted and folded during the
  flight. INFERRED.

### 2.5 Who asks for the exposure's tiles

- Mip 3 over +-20 km about the camera, and about the Droste outer eye, by a Mercator closed form
  on `winH`'s origin (`FrameLoop.cpp:1761-1762`, `:3262-3291`). READ.
- The interests, at `kSwellShadowMipFloor` (`FrameLoop.cpp:3377-3379`). READ.
- `--res-trace` logs the page's resident mip under the camera (`FrameLoop.cpp:3292-3307`). READ.

## 3. Where the z14 or z17 window is named for the height or the exposure

| what | where | kind | mark |
|---|---|---|---|
| `kHpWorldPxZ14`, `kHpPageTexelM`, `kHpMaxMip` | `HeightPages.hlsli:34`, `:36`, `:37` | constants | READ |
| the page branch of `HpHeightAt` and its row comment | `HeightPages.hlsli:41-44`, `:56-61` | function | READ |
| `PageUvLatLon` (the kernels' spelling) and its banner | `PageSample.hlsli:14-26`, `:86-91` | function | READ |
| `PageUv`, `CsWindowUv` (shared with the colour's old path) | `PageSample.hlsli:77-83`; `Compose.hlsli:50` | functions | READ |
| `gCsMerc`, filled from the colour's `win` | `Common.hlsli:389`; `SurfaceFrame.cpp:358` | row | READ |
| `gCsU6.z` (the page's slice); `gCsU2.z`, `.w` (the old window) | `Common.hlsli:385`, `:401`; `SurfaceFrame.cpp:337`, `:343-344` | rows | READ |
| `gCsG.z` = 6, "height window max lod" | `Common.hlsli:390`; `SurfaceFrame.cpp:363` | row | READ |
| `lod + 6.0f`: the page six mips finer than the cube | `Compose.hlsli:396`, `:417` | literal | READ |
| `CsHeightWindowOn`, `CsHaveHeightWin` | `Compose.hlsli:380-385` | functions | READ |
| the live-tide band gated on `gCsU2.z` (finding 1) | `Compose.hlsli:516-521` | row | READ |
| the stencil's page frame | `Compose.hlsli:557-562` | debug | READ |
| lenses 2 and 3; the residency lens's tenant 1 | `Globe.hlsl:1384-1393`; `ResidencyLens.hlsl:137-151` | debug | READ |
| the close-up material's gate | `Globe.hlsl:1485-1487` | gate | READ |
| `gWinA`, `gPageB` | `Swe.hlsl:41-42`; `SeaChurn.hlsl:31-32` | rows | READ |
| `gSlotsD`, `gWinA` (and the exposure through them) | `WaterBank.hlsl:34-37`, `:486-491` | rows | READ |
| `SweShadow`: `CsWindowUv`, `gCsU6.z`, the literals 16384, 128, 15.9375 and mip 3 | `Sea.hlsl:112-132` | reader | READ |
| `SurfaceFrame::win`, `winH`, `det`: `Lattice::Window(1263360, 1538048, 14)` in two tile shapes, and the z17 arithmetic | `SurfaceFrame.h:103-106`; `SurfaceFrame.cpp:196-215` | lattices | READ |
| `hgtWinSlice` from `SliceOf(winH.Tag())` | `SurfaceFrame.h:111`; `SurfaceFrame.cpp:228`, `:301`, `:334` | slice | READ |
| `WindowLattice` | not found by grep in TREE's `src` | none | READ (absence) |
| slice 6 as a number (the height's and the exposure's cases of findings 71 and 72) | `Assembly.cpp:790`, `:821`, `:841`; `FrameLoop.cpp:1056`, `:2823`, `:2828`, `:3280`, `:3290`, `:3299`, `:3303`; `WarmInlet.cpp:49`; `Trace.cpp:107` | literals | READ |
| `m_hgtWinT`, `m_hgtWinFace` and their wants | `GlobeLayer.h:720`, `:723`; `GlobeLayer.cpp:1454-1456`, `:1206`, `:2177-2178` | wants | READ |
| a node's box in z14 pixels (`n14`), shared by the colour's pages, the mask's and the height's page | `GlobeLayer.cpp:1144-1208` | wants | READ |
| `PinDomains`' `mercU`, `mercV`; `m_hgtWin` | `WeatherManager.cpp:188-200`; `WeatherManager.h:148` | wants | READ |
| the exposure's wants (`n14`, `winOrgX/Y`) | `FrameLoop.cpp:1761-1762`, `:3263-3291` | wants | READ |
| the interests on `page = surface.winH` | `FrameLoop.cpp:3319-3385` | wants | READ |
| `HeightPage` on `winH` (memo face `kPageFace` 7) | `Assembly.cpp:782`; `HeightPage.h:46-63`, `:77` | CPU twin | READ |
| `ExposurePage` on `winH`, slice 6, mip 3 | `Assembly.cpp:832-833`; `ExposurePage.h:48-57` | CPU twin | READ |
| the exposure's binding: `exposureSlice = 6u` on `winH` | `Assembly.cpp:821-822` | declaration | READ |
| a log that spells the exposure's ground as `9.55 * 8.0` | `Assembly.cpp:837` | literal | READ |
| `SeaLayer::m_hgtWin`, `WaterBankLayer::m_hgtWin`, `SweSolver`'s `winA`, `pageB` | `SeaLayer.h:334`; `WaterBankLayer.h:267`; `SweSolver.h:227-228` | members | READ |
| tools | `Trace.cpp:26-27`, `:70`, `:96-153`; `WarmInlet.cpp:24`, `:46-52`; `Export.cpp:54-55`; `PackTiles.cpp:21-29`; `TreeAudit.cpp:28` | tools | READ |
| `kSwellShadowMipFloor = 3`: the page's mip 3, 76 m | `WaterTerms.h:85` | constant | READ |
| the wave tenant's z16 frame (section 5) | `WaveFieldSource.h:44-92` | lattice | READ |

## 4. The solver's bed

### 4.1 The binding

`SweSolver::SetHeightPage(heightArr, resMapArr, slice, mips, window)`
(`SweSolver.cpp:29-49`) puts the whole array, every slice and every mip, at table slot 0 (`t1`)
and the residency array at slot 1 (`t2`) (`:34-35`). It fills `geoLL` from the survey lattice
(lon0, lat1, dlon, -dlat; `:37-40`), `winA` from `winH.Rows()` (`:43`) and `pageB.x` = 6 (`:44`).
It is called once at boot for the Merrimack (`Assembly.cpp:788-792`), and for owned windows at
activation (`WeatherManager.cpp:93`) from what `FrameLoop.cpp:1053-1058` handed the weather
manager inside the Boston block. The lattice is the survey's equiangular grid, 1701 x 890 cells
of 10.08 m by 13.65 m (`Swe.hlsl:20`, `:25-26` comments). READ.

### 4.2 `BedAt`: the rows, the mip, the residency

- Rows read: `gGeoLL` (`Swe.hlsl:80-81`), `gWinA` and `gPageB.x` (`:82`), and inside `HpHeightAt`
  the constants of `HeightPages.hlsli:33-37`. READ.
- The mip: `pageMipMin` 0 (`Swe.hlsl:82`), so the page is read at its finest RESIDENT mip,
  `clamp(round(byte), 0, 6)` (`HeightPages.hlsli:58`), and the cube likewise (`:53`). The reason
  written is "the solver 0" (`:43-44`). READ. That the solver wants the finest bed because its
  cells are finer than the page's mip 1: INFERRED.
- The residency: one byte decides both the mip and the page-or-cube choice (`:53-59`). Physics
  therefore reads residency (finding 48). A byte of 255, nothing here, is clamped to 6 and read
  (finding 62). The GPU map is born zeroed, which says mip 0, so the spin-up read unmapped zeros
  (finding 66). A bilinear tap may cross into an unmapped tile (finding 9;
  `PageSample.hlsli:129-134`). READ (the lines); the findings as the review states them.

### 4.3 What `PinDomains` asks for

Every frame (`FrameLoop.cpp:2602`), for every active window with a ready grid:
`Want(Sampler("solver"), hgtTenant, m_hgtSlice = 6, mip 0, u0, v0, u1, v1)`, the domain's
lat/lon box turned into the page's uv by its own Mercator closed form and clamped to the page
(`WeatherManager.cpp:188-214`). Nothing is asked of the cube. It returns at once when no height
page was bound (`:187`), which happens where Boston's data is absent (findings 17, 70). It logs
once (`:215-226`). Finding 67 (a pinned domain short of whole for a whole run) is about this
want. READ.

### 4.4 Every other reader of the bed

| reader | where | mark |
|---|---|---|
| the churn | H19, `SeaChurn.hlsl:53-57`, `:102-106` | READ |
| the water bank (the drawn sea, and through it the hull's comparison) | H20, `WaterBank.hlsl:429-446` | READ |
| the wave field's current | not the tenant: `WaveField::RefreshSweCurrent` reads the solver's fields back every 90 sim-seconds (`WaveField.cpp:986-1019`), which the solver integrated on H18's bed; the wave solve's own bed is the CPU stack (`:1088-1106`) | READ |
| the sea sheet's depth | H16, `Sea.hlsl:68-80`, `:490` (oneWater off only) | READ |
| the globe's pixel water: its depth and its cast | H8 (`Globe.hlsl:981`), H12 (`:1014-1019`) | READ |
| the vertex's land and water mix | H6, `GlobeMesh.hlsl:115-116` | READ |
| the hull | `TreeWater`: the level and current through `SolverRefine` (`WaterSurfaceTree.cpp:193`; `WeatherManager.cpp:137-165`), the bed through `HeightPage` (`WaterSurfaceTree.cpp:239-240`) | READ |
| the solver's CPU grid | static residency, time step, west section (section 1.4) | READ |

Findings 12, 48, 66, 67, 69 and 76 are about these readers; finding 68 (the survey's feather at
the domain's edge) lives in the SOURCE (`Sources.h:126` per the review; NOT READ), so a move to
blocks does not mend it. INFERRED.

### 4.5 What the `solver-bed` worktree adds (its diff read, against 35a9eb7)

- The bed trace: `CsSweBedTrace` writes `BedAt`, and with `HP_TRACE` the mip read and the slice
  read, into an RG32F target (`Swe.hlsl` +26); `SweSolver::TraceBed` builds its own root, table
  and kernel and reads it back (`SweSolver.cpp` +80, `SweSolver.h` +28). The slice is stored in
  four bits (`mip + 16 slice`), so slices up to 15 fit. READ (the diff).
- `HP_HAVE` and `HP_CHOSE` hooks in `HpHeightAt`, empty without `HP_TRACE`
  (`HeightPages.hlsli`, 24 lines changed). READ (the diff).
- `WeatherManager::DomainUv` (the pin's Mercator box, factored out), `ClaimedMips` and
  `WaitForBeds`: residency turns until every cell's trace reads mip 0 of `PageSlice()`, bounded
  by `kBedWaitMaxS` = 60 s (`WeatherManager.cpp`, 118 lines changed; `.h` +29). READ (the diff).
- `BathyModel::DrawFrom` and the solver's own grid `bathySwe`: the domain drawn in to where the
  survey paints at full weight when `water.swe.window == 0` (`Assembly.cpp`, 36 lines changed;
  `BathyModel.cpp` +44). Its default: NOT READ (the scene schema's diff was outside the scope).
- On blocks, three of its assumptions change: the whole test is "mip 0 and slice ==
  `PageSlice()`", one slice; `DomainUv` is the z14 page's uv; `TraceBed` binds the array as
  `SetHeightPage` did. A domain that lies in two blocks has two slices. INFERRED.

### 4.6 What the bed needs on blocks

- A block form of H21: lat/lon to a direction, the face, then each candidate block's texel from
  its rows, `PageWins` against the cube, 4 Loads. From a float32 direction the address is 0.06 of
  a texel at rung 6 and 0.43 at rung 9 (4.4's table, emulated), against 0.27 at rung 6 for
  today's Mercator spelling. READ (the table); INFERRED (that this is the kernels' form).
- The blocks the domain lies in: whether the Merrimack's domain lies inside one rank-2 block was
  NOT computed; 4.17 measured only the mouth's rung-9 block (face 5, block (166, 4)).
- The pin, the wait and the trace in the blocks' uv and slices (4.5). INFERRED.
- The 4.17 law "a solver integrates on a window that stands whole" is `water.swe.bedWait`, default
  off (the owner's decision, in the memory index). The move does not change that decision.
  INFERRED.

## 5. The wave pages

- **The tenant** (`FrameLoop.cpp:1130-1157`): RGBA8_UNORM in 128x128 tiles (`:1133-1134`), Field,
  Volatile, Zero, a zero tile for unbound slices (`:1135-1138`); `6 + kMaxComp + 1` = 39 slices
  (`:1139`; `kMaxComp` = 32 at `WaveField.h:83`); one binding, slices 6 to 38, on
  `waveFrame.color` (`:1140`), painted by the tree in a holder (`:1142`). READ.
- **Its lattice**: a z16 Mercator window whose origin stands four tiles north-west of the solve's
  window (`WaveFieldSource.h:44-48`, `:88-90`). `Align` makes the solve's cell the z16 texel's
  ground at the window's centre latitude, puts its origin on a z16 texel's corner, and rounds the
  cells to whole tiles (`:60-92`). READ.
- **Who paints it**: `WaveFieldSource::PaintTile` (`:155-195`) from the live solve: plane
  `r.face`, mip m the mean of 2^m by 2^m cells, rows flipped to the north (`:171-191`). The whole
  pyramid is prefilled on a worker per bucket, then swapped in and the tenant dropped
  (`FrameLoop.cpp:2644-2682`). READ.
- **Who reads it**: `WaterBank.hlsl` only (grep over every shader). `WavePageUv` turns world metres
  into cells from the window's corner, flips them, adds the window's texel in the page frame and
  divides by 16384 (`:260-263`); `WavePageHave` is a byte at slice 6 + plane (`:266-268`);
  `WavePageSample` is 4 Loads at the resident mip plus half a quantum (`:277-288`). Rows
  `gWaveU`, `gWaveA`, `gWaveP`, `gWaveD` (`:47-49`, `:75-77`), filled at
  `WaterBankLayer.cpp:709-726` from `SetWavePages` (`FrameLoop.cpp:1145-1153`). The hull reads the
  solve itself, in world metres, through `WaveField::ProbeAt` (`WaterSurfaceTree.cpp:318`;
  `WaveField.cpp:1256`). READ.
- **Its wants**: every plane over the window's uv box at a mip by screen size less two levels, at
  most 6 (`FrameLoop.cpp:2683-2763`); the interests at mip 0 (`:3388-3408`). READ.
- **The frame the solved field lives in**: world metres of the flat anchor chart (`BathyModel`'s
  anchor and its frozen metres a degree: `WaveFieldSource.h:63-66`, `:79-82`;
  `WaveField.cpp:1092-1101`), square cells, and its page texel IS its cell
  (`WaterBank.hlsl:252-259`). READ.
- **What a face plane's block would change** (INFERRED): the cell would have to be the block's
  texel, which on the ground is neither square nor orthogonal (523 m by 395 m at 103 degrees at
  rung 0 at the mouth, 4.2), so the wave solve would run on a skewed, anisotropic grid, or the
  tenant would stop holding the solve's exact bytes, which `WaveFieldSource.h:12-14` forbids.
  Beside it: `Align`, `PaintTile`'s tile arithmetic (the block binding asks the pyramid's global
  tile), `WavePageUv` and its rows, `ProbeAt`'s cell law, the wants, the prefill's tile box and
  the identity string (`WaveFieldSource.h:141-147`). The planes are 33 slices a window, so the
  wave tenant cannot use the surface tenants' "block i is slice 6 + i" (4.7).

## 6. What the colour's migration built, and whether the height can use it

| piece | where | the height can use it unchanged? | what differs | mark |
|---|---|---|---|---|
| declaring blocks: the key, the rank law, the directory | `SurfaceFrame.cpp:17-142` | yes: blocks are ground, not a tenant's (4.1) | the height must declare the SAME blocks at the same slices, or the shared rows and directory lie (`Common.hlsli:413`: "its slice of the colour and the mask") | READ; INFERRED |
| `BlockBinding`, `BlockSlice`, the gates, block dispatch, the change law | `Tenant.h:182-226`; `Tenant.cpp:204-229`, `:276-294` | yes: 256x128 tiles are handled (`Tenant.h:197-200`) | the height tree must be asked on `BlockBinding::Pyramid(256, 128)`, another tag than the colour's; with `--no-color-trees` the height has no frame-generic provider (`Compositor.h:150-154`: `CubeHeight`, `WindowHeight` only) | READ |
| the slice count | `Assembly.cpp:996`, `:1091-1092` | the same expression (the height's `:764` and the exposure's `:820` say 7 today) | the exposure needs a lattice binding too (fact 5) | READ |
| the address rows `gCsBlk*` about the eye | `Common.hlsli:406-421`; `SurfaceFrame.cpp:164-173`, `:396-429` | yes, in stages that have b2 (globe, mesh, sea sheet, lenses): block i's uv is the same for every tenant | the kernels have no b2; they need their own copy of the rows | READ; INFERRED |
| the walk and the directory | `Walk.hlsli:77-90`; `Compose.hlsli:111-114`; `SurfaceFrame.cpp:144-162` | yes in graphics stages: `CsWalk` takes no derivative; the CPU twins can call `SurfaceFrame::Walk` | the kernels have no directory binding (Swe and SeaChurn tables, finding 76; `WaterBank.hlsl:174-175` declares no `Texture2D<uint>` array) | READ |
| the chain once a stage (`CS_WC`) | `Compose.hlsli:140-148`; `Globe.hlsl:1106-1115`; `GlobeMesh.hlsl:109-114`; `Sea.hlsl:547-551` | yes, where a chain exists | the mesh stage walks AFTER the height read (`GlobeMesh.hlsl:102` before `:113`); `VsMain` and `DsMain` walk nothing; `ComposedHeightGrad`'s four directions and the cast's landing points are not the pixel's point | READ |
| the point from a direction | `Compose.hlsli:103-105` | yes where only a direction exists (the sea sheet, the cast, the gradient) | its grain (0.06 texel at rung 6, 0.43 at rung 9) | READ |
| a ray walks for itself (`CsWalkAt`, `CS_WALK_AT`) | `Compose.hlsli:132-144` | no: it takes screen derivatives, pixel stage only | the height needs the walk without the gradients | READ |
| one sample a read, the gradients before any branch | `Compose.hlsli:123-131`, `:151-190` | the one-sample rule yes: decide by the residency's levels, then read the winner once | no gradient is needed: every height read is `SampleLevel` at an explicit level (`Compose.hlsli:392`, `:400`) or Loads, so the horizon fault of commit 4 cannot arise | READ; INFERRED |
| the texel's grain over the chain | `Compose.hlsli:342-355` | the pattern | the height's ground and seven mips | READ |
| the lens's block rows | `ResidencyLens.hlsl:117-135` | the pattern for tenant 1 | 256x128 tiles (`:141`) | READ |
| the block wants and floors | `GlobeLayer.cpp:1230-1268`, `:2185-2190` | yes: one more `emit` for the height | the cap of four blocks (`:1518`); the height's floor is mips 4 to 6 | READ |
| the define and its `#if` walls | `Assembly.cpp:302-313`; `Compose.hlsli:84-149` | yes | the define reaches every later compile of `renderer.Shaders()` (`Shader.h:31-33`), so the kernels compile with it too; they do not include `Compose.hlsli`, so their bytes should not move | READ; INFERRED |

What differs for the height as a whole: it is one R16F channel in 256x128 tiles with seven mips
(`Assembly.cpp:760`, `:764`); it is read in the vertex, mesh, domain and compute stages and on the
CPU, where the colour is read in the pixel stage; its want lies `+6` mips from the cube on the
page (`Compose.hlsli:396`), which on a block is `+rung`; the kernels read it with Loads on their
own lattices; and the gradient reads it four more times at other points (`Compose.hlsli:585-596`).
READ; INFERRED (the `+rung`).

## 7. Forced pairs

1. **A row and the struct that fills it**, each pair one commit: `ComposedSurfaceCb`
   (`Compositor.h:276-314`) and `Common.hlsli:383-421`, held by the parity gate
   (`DxTest.cpp:276-278`); `SweSolver`'s constant block (page rows at `SweSolver.h:227-228`) and
   `SweCb` (`Swe.hlsl:19-43`); the churn's (page rows at `SeaLayer.h:257-258`) and `ChurnCb`
   (`SeaChurn.hlsl:19-34`); the bank's (rows at `WaterBankLayer.h:174-201`) and `BankCb`
   (`WaterBank.hlsl:22` onward), rows appended at the end on both sides (`WaterBank.hlsl:57-61`).
   READ (the rows cited; the structs' full extents not read).
2. **The height and the exposure**: `Sea.hlsl:114-116` and `WaterBank.hlsl:486-491` read the
   exposure through the height's slice and row; the interests ask both over one Mercator box
   (`FrameLoop.cpp:3364-3385`); both tenants sit on `winH` (`Assembly.cpp:772`, `:822`). READ. A
   separate exposure row in the two readers would break the pair; the design says they move
   together (4.17). INFERRED.
3. **One bed**: the solver (H18), the churn (H19), the bank (H20), the globe's pixel water (H8,
   H12), the vertex's mix (H6) and the hull's twin (`HeightPage`) read one bed today
   (`SweSolver.cpp:46-48`; `HeightPage.h:5-13`). Moving one splits the bed that findings 12 and
   69 already measure apart. READ; INFERRED.
4. **The bed and its want and its instruments**: `PinDomains`' box, `solver-bed`'s whole test and
   the trace's slice code must name the slices `BedAt` reads. INFERRED.
5. **The height's declaration and everything that names its page**: `SurfaceFrame::Declare` asks
   `SliceOf(winH.Tag())` (`SurfaceFrame.cpp:228`); with blocks that tag is undeclared and
   `SliceOf` routes it to slice 0 (finding 71; `Tenant.cpp:158-175`), so `Declare` must not ask,
   as the colour's commit did (`SurfaceFrame.cpp:222-225`); then `Fill`'s `u6`, the globe's
   `m_hgtWinT`, and the `6u` literals of section 3 follow. READ; INFERRED.
6. **The height's blocks and the colour's**: one set of rows and one directory serve both, so the
   two declare the same blocks. INFERRED.
7. **The lens and the reader it mirrors**: tenant 1 of the lens must make `ComposedHeightPages`'
   choice, or it paints a choice the picture does not make (`ResidencyLens.hlsl:20-21`). READ.
8. **The twins and their kernels**: `HeightPage` with H21's rule; `ExposurePage` with E1's floor
   and bilinear law. READ.
9. **The exposure's tree and its first binding** (`Tenant.cpp:415`). READ.
10. **The colour's commit 5 and step 7 share the Mercator row**: `gCsMerc` is filled from the
    colour's `win` (`SurfaceFrame.cpp:358`), so deleting `win` before the height leaves the page
    takes the height's row away. READ; INFERRED.
11. **`solver-bed` and TREE's `HeightPages.hlsli`** (fact 4). INFERRED.

## 8. A proposed order of commits (a proposal; nothing here is decided)

The design (section 6, row 7) moves the height, the exposure and the bed as one. The proposal
keeps that one move and puts in front of it what changes no picture, so the move itself is small.

| commit | what it changes | what it must leave unchanged | the instrument that can see it fail | what that instrument is blind to |
|---|---|---|---|---|
| 7.0 instruments | land `solver-bed` on TREE (the bed trace, the wait default off, `DrawFrom` if its default is today's); mend finding 73 (`--water-probe` under a recipe) and finding 75 (the bank's fingerprint hashing descriptor indices); give the lens's tenant 1 the blocks behind the define; a probe of priors 1 in the mesh stage (H5's value read back against H8's) | key empty: every stage's DXIL and the six stills' `[settle-exact]` hashes TREE's; key set: TREE's key-set pictures | the hashes; DXIL compared byte for byte; the trace's own planted floor (`HP_TRACE`) | a picture a still does not show; a transition |
| 7.1 functions, not called | the block form of `HpHeightAt`; the block form of `ComposedHeightPages` over the chain; the walk without gradients for a ray; `HeightPage` and `ExposurePage` on `SurfaceFrame::Walk`; one helper for "a lat/lon box in a block's uv" to replace the three Mercator closures; a selftest holding the CPU twin against a GPU readback at random ground (commit 4's pattern) | everything: new code only inside `#if GA_BLOCK_RANKS` and in tests (`ResidencyLens.hlsl:7-13` measured that code a file holds can move another entry's pixels) | the selftest, with a planted wrong row; the hashes | the picture, which nothing draws yet |
| 7.2 the move | under the key: the height and the exposure declare the blocks in place of the page (with the exposure's lattice binding); `Declare`, `Fill`, the wants, the floors, the interests, the pin and the wait in the blocks' uv; every GPU reader of sections 1.3 and 2.2 and both twins switched; the kernels given block rows by their binders | key empty: every hash TREE's. Key set: bit identity is gone by construction; what must hold is the drawn level against the hull's inside today's floor, and the bed within a stated bound of today's (the lattices hold the same ground: 4.17's texel-against-texel check, run on the height tree) | `--water-probe` at the helm and the bird; the bed trace (every cell at mip 0 of its block after the wait; its values against today's); the residency audit on the new slices; the lens; stills at four poses by SSIM and by eye; `--sea-verify` | `--water-probe`: only near hulls, the displaced point (finding 53); the trace: the solver's bed only, not the bank's or the churn's; the audit: a wrong tile under a right byte; `--sea-verify`: reads the cascades' displacement, not the bed or the exposure (`Tools.h:118`); `[kernel]` fingerprints change by design here, so they cannot gate this commit |
| 7.3 the bed's law | `water.swe.bedWait` on by default, if the owner says so | the level at frames 600 and 5400 within the solver-bed experiment's numbers (4.17) | the bed trace; `--water-probe` | whether the wait finishes on another disk |
| 7.4 the old path of these tenants deleted | the page and its rows, literals, closures and tools of section 3, for the height and the exposure | the key-on bytes reproduced | the hashes, key set, against 7.2's | nothing new: a deletion |
| 7.5 the wave pages | the solve re-aligned to a face plane's block (section 5) | the solved field against `ProbeAt` at the hull (the water match's step 2 gate) | `--water-probe`; the wave's fidelity map (`Tools/FidelityMap.cpp`, NOT READ) | the look of a skewed grid, the owner's to judge |
| 7.6 the water surface as a tenant | 4.11 | beyond this map | | |

An alternative for 7.2 (a proposal only): keep the z14 page as the LAST slice (6 + n) while the
key stands, so the readers can move one commit at a time, each gated alone, and delete it at
7.4. Its price: a second copy of the height resident near the Merrimack, and a separate exposure
slice row in `Sea.hlsl` and `WaterBank.hlsl`, which breaks forced pair 2 on purpose. INFERRED.

Decisions this needs from the owner, named and not taken: the rank each tenant stands at (the
exposure could stand on rank 1 alone, fact 5); whether `ComposedLandness`'s dead live-tide band
(finding 1) is deleted or restated on blocks, which would change the picture; whether the
close-up material's gate becomes "inside a rank-2 block"; whether the sea sheet's readers (drawn
only with `oneWater` off) move or go.

## 9. Line counts

Lines today are `ReadAllLines().Count` of TREE's files (READ). Removed and added are my estimates
for commits 7.1 to 7.4 (INFERRED; plus or minus half).

| file | lines | removed | added | what |
|---|---|---|---|---|
| `shaders/HeightPages.hlsli` | 66 | 15 | 30 | the page branch and the z14 constants out; the block form in |
| `shaders/PageSample.hlsli` | 192 | 19 | 6 | `PageUvLatLon` and its banner out once no kernel uses it |
| `shaders/Compose.hlsli` | 598 | 42 | 35 | `ComposedHeightPages`, the old window branch, `CsHeightWindowOn`, the dead band out; the chain form in |
| `shaders/Common.hlsli` | 439 | 3 | 3 | row comments |
| `shaders/Globe.hlsl` | 1830 | 14 | 14 | the chain to each height read; lenses; the material's gate |
| `shaders/GlobeMesh.hlsl` | 441 | 4 | 4 | the walk moved above the height read |
| `shaders/Sea.hlsl` | 570 | 22 | 25 | `SweShadow` and `BedAt` onto blocks |
| `shaders/SeaChurn.hlsl` | 137 | 4 | 6 | rows |
| `shaders/Swe.hlsl` | 312 | 3 | 6 | rows (plus 26 if the bed trace lands) |
| `shaders/WaterBank.hlsl` | 896 | 12 | 14 | the bed's and the exposure's rows and address |
| `shaders/ResidencyLens.hlsl` | 240 | 15 | 20 | tenant 1 onto blocks |
| `src/compose/SurfaceFrame.h` | 207 | 3 | 2 | `winH`, `hgtWinSlice` |
| `src/compose/SurfaceFrame.cpp` | 432 | 8 | 4 | `Declare`, `Fill` |
| `src/compose/Compositor.h` | 330 | 2 | 2 | comments |
| `src/compose/HeightPage.h` | 117 | 12 | 15 | containment onto blocks |
| `src/compose/ExposurePage.h` | 115 | 10 | 12 | the same |
| `src/app/Assembly.cpp` | 1514 | 30 | 30 | the two declarations; the binders' arguments |
| `src/app/FrameLoop.cpp` | 4129 | 70 | 40 | three Mercator boxes into one helper; the bank's binder |
| `src/sim/WeatherManager.cpp` / `.h` | 369 / 184 | 20 | 15 | the pin's closed form (and `solver-bed`'s `DomainUv`) |
| `src/sim/SweSolver.cpp` / `.h` | 772 / 298 | 8 | 15 | `SetHeightPage`'s rows |
| `src/scene/SeaLayer.cpp` / `.h` | 841 / 354 | 8 | 12 | the churn's and the sea's rows |
| `src/scene/WaterBankLayer.cpp` / `.h` | 799 / 295 | 10 | 15 | the bank's rows |
| `src/scene/GlobeLayer.cpp` / `.h` | 2460 / 828 | 8 | 6 | the page's wants out, the blocks' in |
| `src/app/Tools/Trace.cpp` | 161 | 58 | 40 | step 11 onto blocks, or deleted |
| `src/app/Tools/WarmInlet.cpp` | 79 | 7 | 7 | its wants |
| `Export.cpp`, `PackTiles.cpp`, `TreeAudit.cpp` | 188, 39, 87 | 4 | 3 | a frame or a tag each |
| **total** | | **about 410** | **about 380** | |

Plainly: step 7 by itself about breaks even, because every Mercator reader gets a block reader.
It deletes the page's code for these two tenants. The larger deletion comes when the colour's
commit 5 and step 7 have BOTH landed: then `PageUv`, `CsWindowUv`, `gCsMerc`, `gCsDet`, `gCsU4`,
`SurfaceFrame::win`, `det` and `winH` with the z17 arithmetic (`SurfaceFrame.cpp:196-215`), the
node's Mercator box (`GlobeLayer.cpp:1144-1229`, 86 lines) and the three Mercator closures
(`WeatherManager.cpp:188-200`, `FrameLoop.cpp:3263-3275`, `GlobeLayer.cpp:1165-1189`) have no
reader; about 250 lines more. INFERRED. Landing `solver-bed` adds +373 and -23 in the files
diffed, plus `BedTrace.cpp` (untracked, not read). READ (`git diff --stat`). The wave pages'
re-alignment is a change of the solve, not a deletion; not estimated.

## 10. What I did not read, and what looked wrong

Not read:

- `Residency.cpp` beyond a grep for slice numbers (it names none), and the residency manager's
  `Want` and `Invalidate` bodies.
- `WaterBankLayer.cpp`'s fill of the corner bed; `WaterProbe.cpp` beyond its banner;
  `SeaVerify.cpp` (its purpose from `Tools.h:118` only); `FidelityMap.cpp`; `BedTrace.cpp`.
- `solver-bed`'s `Sources`, `SceneSchema`, `Options`, `FrameLoop` and tools diffs (outside the
  scope given): so the default of `water.swe.window` and where `WaitForBeds` is called are not
  known here.
- Whether the fallback vertex pipeline (H4) runs by default, and whether any shipped scene sets
  `water.oneWater` false (which is what draws H16, H17 and E2).
- `WaveField`'s solver stencil (whether it assumes square cells), `Lattice.cpp`,
  `TileTree.h`'s fold, `GisLayer`, `TerrainLayer.cpp`.
- The priors-1 question (fact 3), and whether the solver's domain lies in one rank-2 block.

Looked wrong, and not step 7's (one line each):

- `GlobeLayer.cpp:1518` caps the per-node block wants at four (`GlobeLayer.h:189-191`) while the
  key takes eight (`SurfaceFrame.h:117`); blocks past the fourth are asked only at their floor
  (`GlobeLayer.cpp:2186-2190`). INFERRED consequence.
- `GlobeLayer.cpp:2175-2178` asks the height page for mip 7; the height has mips 0 to 6. READ.
- `Globe.hlsl:538-541` and `docs/ALGEBRA.md:1042-1047` against `GlobeMesh.hlsl:102`, `:116`: the
  comment and the code disagree about `SampleLevel` in the mesh stage. READ.
- `Globe.hlsl:1485-1487`: the close-up material exists only inside the Merrimack's z14 window, so
  a second place never gets it (a place's name in code, for `docs/PLACE_KEYS.md`). INFERRED.
- `WaterBank.hlsl:486`: the bank reads no exposure unless the height's page slice is set. READ.
- `ExposureSource.h:146-147` marches in the anchor chart's frozen metres a degree at every place
  on the planet. READ; its effect far from the anchor INFERRED.
- Three copies of one Mercator closed form: `WeatherManager.cpp:188-200`,
  `FrameLoop.cpp:3263-3275`, `GlobeLayer.cpp:1165-1189`, beside `Lattice::PxOf`
  (`Lattice.h:231`). READ.
- `SweSolver.cpp:104-132`, `:189-195`, `:221-229`: the solver's residency, time step and west
  section come from the CPU survey while its kernels read the tenant: two beds in one solver
  (finding 69's class). READ; INFERRED (the class).
- `Assembly.cpp:837` spells the exposure's ground as `9.55 * 8.0` in a log. READ.
- `SurfaceFrame.cpp:358` fills the height's Mercator row from the colour's lattice `win`; the
  same origin today (`:196`, `:198`), but the height's row dies with the colour's lattice. READ.
