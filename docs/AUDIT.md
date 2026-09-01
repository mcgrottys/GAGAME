# Audit: the project against the Sparse GA design

Written 2026-09-01 against the design as the user restated it that day, grounded in the
Scriptorium registry (`channels`, `symbols`), `docs/ALGEBRA.md` (`compose`, `priors`), and a live
grep of every texture creation site. Each row says what the code does today, not what the docs
say it should. **Aligned** means the path exists and is measured; **partial** means the shape is
there and something named is missing; **missing** means nothing on the path exists yet.

## The design, one line each, and where it stands

| # | The rule | Status | Where it stands today |
|---|---|---|---|
| 1 | Multiple hosts/domains in a scene, each with its own GA schema (projections, units, scales) | **partial** | Earth and Mars exist as *modes* (`marsMode` in `main.cpp`), not as host objects. `LevelLadder` is the per-body scale (`PageTable.h`), `UnitSpec`/`GaUnits.h` the units, `GeoRef` the projection — but they are not gathered into one `Host` that a source is normalized *to*. The earth ladder is declared inline. |
| 2 | Every ingested file — volumes, rasters, numerics — written to its own Sparse GA model **on disk** for the target host | **partial** | Colour: yes, every leaf of the megatexture graph has its own on-disk tree (`TileTree`, §31). Height: the bed composes into a GPU `GradeBank` at boot from `GeoGridLoader` + `RasterSource` (§25–28) — normalized and composed on the GA path, but **not cached as a tree on disk**. Water/weather planes (`GlobeLayer::BuildPlaneBank` ×5, `TerrainLayer::BuildBedBank`, `GulfLayer`, `WeatherManager`): same — GA path, no disk tree. Volumes (cloud banks): `TileAtlas` 3D reserved, no disk tree. |
| 3 | A GA adapter normalizes each model to the domain (projection, units, scale) before use | **aligned for units/scale, partial for projection** | `NormalizedSource` + `Normalize()`/`NormalizeToSi()` refuse an unnormalized or incommensurable source (§26). Projection is handled *per source* inside `Sample()` (each source resolves to the WGS84 exchange frame); there is no adapter that re-projects a raster to the host's frame as a stage. Mercator vs cube vs lat/lon pages are frames of the *realization*, not of the model. |
| 4 | PNG / GeoTIFF / etc. into a lossless engine bitmap | **missing** | `FieldSet.cpp:26` loads PNGs straight into committed textures. GeoTIFF-derived data arrives as `.rgb`/`.f32`/`.bin` blobs produced by harvester scripts (see `products` in the registry) — those *are* lossless, but ad hoc per script; there is no engine-side lossless tile format other than the 64 KB tree tile itself, and no PNG → tile ingest. |
| 5 | GeoTIFFs are not tiled: the loader must build the lower-LOD mips for the tree | **partial** | `TileTree` paints each mip **independently from the source** at that mip's ground resolution (the compositor's rule since M6i: "each LOD averages the source over its own footprint"). That yields correct coarse levels, but it re-samples the source per level rather than building a mip chain from the finest — for a 15 cm ortho every coarse tile re-reads the JP2. A fold-down (§ALGEBRA `fold`: average the *answers*) from the finest tree level would be cheaper and is not written. |
| 6 | Painting order with transparency: Google base, GeoTIFF over, partial/overlay paint lets the base fill through | **aligned** | `DomainCompositor::LayeredOver` with straight alpha (`OverStep`/`OverFinish`); alpha is fiber, per texel (§compose, `composetest` "alpha-ramp" gate). Measured tile-for-tile against the incumbent at worst 1/255 (§31). |
| 7 | Compose two trees into a third with **references** into the inputs, composited tiles only where they overlap | **aligned** | `TileTree`: zero-byte `.ref-<childId>` entries, composited bytes only where >1 input covers. 21,508 addresses warm: root 74% references, `earth.land` 97%, `earth.seafloor` 100% (§31). |
| 8 | The same process for seafloor textures | **partial** | The *tree* exists (`earth.seafloor`), but its only source is `synth.bed` — a classifier over the height stack, not ingested seafloor imagery. The acceptance image (seafloor relief visible through water) needs a bathymetry-derived shaded/sediment texture as an **ingested** source. |
| 9 | A Sparse GIS tree, same process | **aligned (vector)** | `gis.landsea`: rings are the authority, no `.raw` opened, meridian sweep with the exact great-arc meet, its own tree on the shared addresses (§30). |
| 10 | Land/sea mask from the GIS tree; land over sea with the sea mask making land transparent per pixel | **aligned** | `GateSource` (seafloor weight × survey water coverage) composed under the land tree in `earth.color`; "the GIS mask gates, the height band refines" (§30–31). |
| 11 | That final tree is the one and only mega land/floor texture in the tiled 2D texture array | **aligned as of §34** | One tenant: `AddTexturePages`, a reserved `Texture2DArray` of 8 pages (6 cube faces + 2 Mercator pages), one SRV, one residency map, one budget. The insets are deleted. |
| 12 | Same for water, heights, future volumetrics | **partial** | Height is **one** page tenant as of §35. Water/weather are `GradeBank` arrays (aligned in storage), composed on the GA path, but from RAM not from disk trees. |
| 13 | Everything staged to the GPU follows the Sparse GA design: 3D → 3D tiled resource, 2D → tiled 2D texture array, 1D → one big buffer | **partial** | 2D: colour ✔ (§34), height ✔ (§35), water banks ✔ (`TileAtlas` reserved arrays). 3D: cloud banks ✔ (`TileAtlas` volumes). 1D: the fields buffer (`gFields`, `StructuredBuffer<FieldDesc>`) exists for descriptors; scalar *data* still lives in per-field textures. **Not** on the design: `GulfLayer::m_uvTex`, `SweSolver::m_uv`, `WeatherManager::ownedBathyTex`, `FieldSet` PNGs, `GisStencil`'s three raster masks (shader-side). `TerrainLayer::m_tex` was deleted (§32). |
| 14 | GAs chain, so new composite trees (e.g. for CPU physics) are easy | **aligned for the mechanism, unproven for physics** | `CompositeSource`, `GateSource`, `BinaryFieldSource` chain; `TileTree` caches any node. `water.depth = tide − bed` runs through `BinaryFieldSource` and matches the engine to 0.0000 m (§26). No physics consumer *reads a tree from disk* yet — the SWE lattice reads the bed bank. |
| 15 | Only three things go to the GPU: 1 global colour GA, 1 global height GA, N water/weather GAs | **partial** | Colour: 1 ✔. Height: 1 ✔ (§35). Water/weather: N banks ✔. What keeps this partial is the strays in row 13. |

## What is genuinely on the design, measured

- Colour, end to end: ingest → normalize (`NormalizeToSi`) → per-source tree → composite tree with
  stored references → megatexture tree → **one** page tenant → NVMe→GPU by DirectStorage.
  Worst |Δ| 1/255 against the incumbent tile-for-tile; 0 pixels DS vs ring at 200/400 frames.
- The GIS tree is vector-backed, exact on the sphere, and the gate's absent/void semantics are
  carried in the tile key.
- The blend is the water's blend: one kernel (`OverStep`) for the point path and the tile path.

## What is not, in the order it should be fixed

1. ~~Height → one page tenant~~ — done, §35.
2. **Height and water as disk trees** (row 2). `BuildBedBank`/`BuildPlaneBank` compose pages into
   RAM at boot; put a `TileTree` under each node so the bed and the planes are cached on the NVMe
   like colour, and boot becomes a read. This is the point where "chain for CPU physics" (row 14)
   becomes real: the SWE lattice can then ask the bed tree for its tiles.
3. **Ingested seafloor texture** (row 8) — the acceptance image.
4. **A `Host` object** (row 1): ladder + unit frame + projection + radius, one per body, that every
   `Normalize()` targets and every `TileTree` keys on. Today the earth's is implicit.
5. **The strays** (row 13): `GulfLayer::m_uvTex`, `SweSolver::m_uv` (a Volatile bank),
   `WeatherManager::ownedBathyTex`, `FieldSet` PNGs, and the shader's three GIS rasters, which the
   compositor no longer needs and the classifier should read from the mask tree's pages instead.
6. **Mip folding for GeoTIFF leaves** (row 5): fold the finest level down (§ALGEBRA `fold`) rather
   than re-sampling the source per level.
7. **Lossless ingest format** (row 4): the 64 KB tree tile *is* the engine's lossless bitmap; a
   PNG/GeoTIFF loader that writes straight into a leaf tree closes the gap.

## Two ledger entries this audit adds nothing to but everything depends on

- Priors §9/§15: a cold cache is not a pixel difference, and an A/B that agrees exactly is a
  severed wire. Both cost this session hours before being re-read.
- Priors §19: recording a copy is not executing it. Every DirectStorage difference since M9ai.
