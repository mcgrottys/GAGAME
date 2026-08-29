# ATLAS — planes, monasteries, and the lingua franca

*The texture-handling writeup, at the fork before the water work. 2026-08-29.*

This document does three things: records what the pipeline actually is today (§1–2), names
the doctrine it has been converging toward — rasters as **planes**, channel families as
**monasteries**, the shared algebra as the **bible** they all read (§3–6) — and commits to
the decisions the water, air, and Maxwell monasteries will inherit (§7–9). §10 covers the
Scriptorium, the .NET/MCP tooling service that keeps the growing project navigable.

---

## 1. What exists, as built

The inventory, honest, as of commit `bdc32e3`:

**Sources** (`src/compose/Sources.*`, the schema registry). Each declares name, structure,
**native CRS**, cm/px, and WGS84 coverage; each answers point queries in the exchange frame
(§4) with a weight (0 = no coverage, feathered edges, −1 = transient failure, never cached):

| source | structure | native CRS | finest |
|---|---|---|---|
| google.satellite | mercator tile tree (jpeg 256², sessioned, cache-first) | EPSG:3857 | ~10 m/px (z14 cap) |
| massgis.coq2023 | 1500 m USNG tiles, mip chains, memory-mapped | EPSG:6348 UTM 19N | 0.15 m/px |
| noaa.etopo2022 | equirect int16 8192×4096 | EPSG:4326 | ~4.9 km/px |
| etopo.ne15s | window grid int16 | EPSG:4326 | ~461 m/px |
| noaa.cudem.merrimack | GeoTIFF-derived window, thalweg-preserving | local tangent @ ACT0816 | 13.7 m/px |
| usgs.mola (Mars) | equirect int16 | areographic equirect | ~7.4 km/px |
| gshhg coast/rivers | **vector polylines** + parity-filled masks | EPSG:4326 | survey (~100 m) |

**Channels** — a channel is meaning, not storage: `earth.color`, `earth.height`,
`mars.height`. A channel is an *ordered stack* of sources; upper layers paint over lower
with per-pixel weights.

**Realizations** — a channel made samplable: the 16k cube pyramid (whole planet), the
Mercator z14 window (156 km around the estuary, color *and* height in one shared frame),
the z19 export-only realization (~22 cm ground). Tiles are painted once on worker threads,
cached to disk (`cache/composed/`, the DirectStorage-ready folder), and streamed by the
residency manager. Paint cost is per *tile actually demanded*, never per source ingested.

**The soak rule** (`Compositor::ColorSubset`/`HeightSubset`) — every tile's cache identity
hashes only the sources that meaningfully touch it (footprint intersects, and spans ≥ ~2
texels at that LOD). A new source repaints exactly the tiles it can change at every LOD
above and below its native resolution; where it shrinks below a texel it drops out of the
subset and the "repaint" short-circuits into a byte-identical cache hit. Pinned in
`--selftest`.

**One render path** (`shaders/Compose.hlsli`) — the renderer knows channels only. Globe,
sea, terrain-materials, and the mesh-shader surface all call the same `ComposedColor` /
`ComposedHeight` / `ComposedLandness` on the same constants, filled by one C++ function.
Land/water classification is analog through a ~40 cm live-tide shore band; its input is
residency-stable.

**The trust surface** — four `--selftest` gates (PGA motors, compositor contracts +
projections, tiled-resource semantics, atlas kernels); `--export <channel>[:mip]` pulls any
channel out through the identical provider path (PNG / raw / OBJ); `--albedo` strips all
presentation; `--stencil` overlays surveyed truth. The compositor runs renderer-free — the
exporter proves the decoupling seam already exists.

**The Exchange** (`src/compose/Exchange.h`) — named, versioned GPU buffer channels with
declared multivector layouts (stride + Cl grade-signature byte + semantic). GIS polylines
and the tide-station **motor buffer** (CPU PGA → GPU sandwich, formulas line-for-line) ride
it today.

## 2. The flow, end to end

```
raw source caches            composed caches                GPU
(jpg/jp2/i16/bin,  ──paint──▶ 64 KB tiles, per-tile ──map──▶ reserved-resource
 forever, polite)             subset identity              pyramids + residency maps
        ▲                            ▲                            ▲
   harvesters (py)             worker threads               residency manager
   one-time, cached            box-filter soak              (wants from the CDLOD walk,
                                                            classic invariants, screw
                                                            prefetch)
```

Nothing in a shader names a source. Nothing in a source knows a shader exists.

---

## 3. The reframe: a raster is a plane

Stop reading these as textures. Each dataset is a **plane** in the geometric sense: a flat
(or locally-flattened) 2-manifold carrying

- a **frame** — its native CRS: which projection pinned it flat, from which datum;
- a **metric** — meters per pixel, and *where that statement is true* (Mercator's m/px is a
  function of latitude; UTM's is nearly constant; the tangent frame's is exact at the
  anchor);
- a **lattice** — the sample grid, with an origin and an orientation;
- a **value fiber** — what each cell holds: sRGB radiance, meters of elevation, a
  multivector of flow.

A "texture fetch" is really: *project a point from the exchange frame onto this plane,
express it in the plane's lattice, and integrate the fiber over the query's footprint.*
Every bug we fixed this month was a violation of that sentence — the M6g reflection (a
frame that wasn't a rotation), the sRGB curve hack (a fiber read with the wrong transfer
function), the classification speckle (a footprint mismatch between question and answer).

The algebra makes the taxonomy of transforms honest:

- **Frame changes** between rigid frames are PGA **motors** — versor sandwiches, composed
  by multiplication, pinned by selftest. The tangent↔planet rotation, station placement,
  camera rails: all motors already.
- **Projections** (sphere → Mercator / TM / LCC) are **conformal maps**, not motors — angle-
  preserving, scale-varying. That conformality is *why* they compose cleanly with rotors
  (a conformal map's Jacobian is a rotor times a scalar), and why CGA is the natural home
  when we outgrow hand-written Snyder forms. Today: exact double-precision forwards in
  `Projections.h`, selftest-pinned at the anchor.
- **Resampling** is **integration against the dual cell** — box means for conservative
  quantities, filtered means for radiance. Never nearest for physics.

## 4. The lingua franca — three tiers, not one grid

The most important ingest decision is what everything transcodes onto. The answer we
committed to (and the one this document makes law) is that the lingua franca is **not a
grid**. Forcing every source onto one master lattice at ingest destroys information at the
altar of uniformity — the 7 cm source blurred to 10, or five copies of the 10 cm source
pretending to be 5. Instead, three tiers:

**Tier 1 — the exchange frame.** WGS84 geographic coordinates on the sphere (lat/lon,
double precision). Every `Sample()` speaks it; every plane resolves it into its own frame
with an *exact* projection, no linear approximations. Two planes can only disagree by being
wrong, never by speaking different coordinates. (Mars swaps in its areographic equivalent;
the tier is "the planet's angular frame," not Earth specifically.)

**Tier 2 — the value algebra: the bible.** Fibers are declared as multivector layouts with
grade signatures — the same Cl bits the atlas and the Exchange already speak. A scalar
height, a G2 flow multivector (div + vector + curl in one texel), a motor, an
electromagnetic bivector: each channel declares what its values *are*, and the Cayley
closure (`Cl2ProductSignature`) reasons about products of channels without reading data.
This tier is what lets monasteries communicate (§6): they never share grids, they share
*meaning*.

**Tier 3 — the ladder.** Realizations sample on canonical halving rungs. Each realization
declares an anchor and walks it by 2×: the cube walks ~611 m/px down its mips, the window
walks 9.55 m Mercator, the aerial chain walks 0.15 m. The rungs are per-realization, but
the *rule* is shared and is the answer to the granularity question:

> **The rung rule.** A realization serving a consumer picks the first rung *strictly finer
> than the finest source that touches it*. A 10 cm and a 7 cm source realize onto 5 cm —
> never 7, never 10 — so no source is ever undersampled at the point of use; sampling error
> lives only in the *coarsening* direction, where the soak's box means make it a
> measurement, not an artifact. Physics that differentiates (SWE gradients, Maxwell curls)
> gets one rung finer still if its stencil demands it.

Dozens of fidelities are then a non-event. The soak rule already makes each tile's paint
ask only the sources that touch it, each source answers at its own best (box-filtered down,
bilinear up), and cache identity tracks the subset. Fidelity diversity costs exactly the
tiles it changes.

**Alpha is fiber.** Quad trees and flat images (GeoTIFFs) never meet in a special case —
they meet in the paint loop's per-pixel lerp, and a plane's own alpha channel multiplies
its paint weight texel by texel. A mostly-transparent GeoTIFF of highlights bleeds through
the final composed quadtree exactly as much as each pixel's alpha says (weight = footprint
feather × per-pixel alpha; RGBA mip chains reduce alpha-weighted so transparent pixels
never darken neighbours; the soak then carries the highlight up-LOD in proportion to its
alpha × area — a thin stroke honestly fades from orbit). Drop any UTM-19N GeoTIFF into
`cache/overlay/` and `harvest_overlay.py` ingests it from its own GeoTIFF tags; pinned in
`--selftest` (the alpha-ramp contract).

One distinction the ladder must keep sacred: **conservative vs. perceptual resampling**.
Heights, depths, fluxes, charge — quantities physics integrates — reduce by *plain box
means* (mass preserved). Radiance reduces through the correct transfer function (the sRGB
lesson: hardware decode, then filter linear). The value algebra (tier 2) records which rule
a fiber follows.

## 5. The granularity worked example

The question as asked: *"if we have a 10 cm source and a 7 cm source, what will the
standard be? 10, 7, or 5-and-less for more accurate physics sampling?"*

Committed answer: **5 and less — but only at realization, never at ingest.**

- At **ingest**, both planes keep their native lattices untouched, plus a baked box-mean
  mip chain each (the aerial pattern). Nothing is ever resampled *onto another source*.
- At **realization**, any consumer that needs their union picks the rung finer than 7 cm —
  on the decimal-halving ladder that is **5 cm** (on a binary ladder anchored at 1 m,
  6.25 cm; the anchor is the realization's choice, declared in the registry). The 7 cm
  plane magnifies 1→~2 (bilinear); the 10 cm plane magnifies 1→2; neither is ever averaged
  *away* below its information content.
- Everything coarser than 5 cm comes from the soak: each rung a fresh box mean over its own
  footprint, recursively, short-circuiting where a source stops mattering.

This is the pattern the water data inherits directly: a 30 m NOAA current grid over a
13.7 m bed over 10 m wave spectra realize onto the SWE lattice at the rung the *solver's
stencil* demands, and each keeps its native truth for any finer future consumer.

## 6. Monasteries, the bible, the Vatican

Adopting the metaphor as architecture vocabulary, because it names the dependency rules
better than "module" ever did:

**A monastery** is a self-contained channel family: its own sources, its own canonical
frame and ladder anchors, its own realizations, caches, physics, and harvesters. Today:
*earth-color*, *earth-height/bathymetry*, *mars*, the *survey* (vector coastlines/rivers —
an order of scribes, not a raster house), and half-built: *water* (SWE, tides, waves),
*air* (cloud volume, wind Mv2). Planned: *maxwell*. **Monasteries do not import each
other's internals.** The water monastery never reads Google tiles; the color monastery
never reads eta.

**The bible** is what they all read instead: the exchange frame, the value algebra
(multivector layouts + grade signatures + the Cayley closure), motors for every rigid
frame, the projection canon (`Projections.h`, selftest-pinned), the ladder and rung rule,
the soak rule, and the conservative/perceptual resampling law. In code: `src/compose/`,
`core/Pga.h`, `shaders/GA.hlsli`, the signature registry. A monastery is *defined* by
conforming to the bible, and by nothing else.

**The Vatican** is any client where monasteries convene under those shared standards.
Today the renderer is the biggest one — and the doctrine there stays: *it knows channels
only.* Earth water is earth water; earth color is earth color. The exporter is a second,
smaller Vatican (same channels, files instead of frames). The background sim service (§8)
and the MCP surface (§10) are the next two. Each monastery may bring a **relic** — something
uniquely its own that others don't take home but may use in the meeting: the sea brings
live Cox–Munk glint physics no mosaic can know; the survey brings vector coastlines that
stay crisp at every zoom; the stations bring motors. Relics are allowed; *dependencies* are
not.

**The cardinals** are the kernels and shaders that paint new pictures from the convened
data — `Compose.hlsli`, the SWE compute passes, the wind-curl kernel, the mesh-shader
surface. A cardinal reads channels and relics, speaks the bible's algebra on the GPU (the
motor sandwich in `GA.hlsli` is the same formula the CPU pinned), and owes loyalty to no
monastery.

## 7. Water through the same pattern

The water monastery, mapped onto the triad before we build it:

- **Sources**: tide-station harmonic series (an *analytic* source — like the survey's
  polylines, authority without a lattice), the CUDEM bed (shared datum with earth-height,
  NAVD88), GFS wind and wave spectra, river discharge, and later CO-OPS currents. Each with
  declared CRS, units, datum, and cadence (water sources have a *time* axis; the schema
  grows a `valid_at`).
- **Channels**: `water.eta` (surface deviation), `water.flux` (a G2 multivector fiber —
  the staggered face fluxes as vector part, divergence and curl as scalar/bivector),
  `water.depth` (derived: eta − bed, by the Cayley closure a product channel whose demand
  is computable without reading either).
- **Realizations**: the **solver lattice** (the SWE's staggered grid — the first
  *read-write* realization, double-buffered, fence-published per step), the renderer's
  read-only peek (exactly how the sea layer consumes eta today), gauges/exports.
- **Contours as vector authority**: the wet/dry line is the water monastery's coastline —
  extracted per step like GSHHG's, rendered by the same vector pattern, and compared against
  the survey's static shoreline as a *validation* (the stencil habit, made permanent).
- **The relic** it brings to the Vatican: breaking energy (`i.brk`) and churn — live
  physics the imagery monastery can never know, already gating foam correctly as of M6n.

Air follows identically (the cloud volume and wind bank are already channel-shaped), and
Maxwell is where the value algebra stops being decoration: **F = E + IB is one bivector
fiber in one channel**, staggered realizations are the same idea as the SWE's face fluxes,
and the field product F·F̃ giving the invariants is a Cayley-closure demand derivation the
registry already knows how to reason about.

## 8. Decoupling: the sim as a background service

Staged, honestly:

1. **Now** — the seam exists. The compositor, physics, caches, and Exchange run without a
   swap chain; `--export` and the selftests are renderer-free clients. `TileProviderFn` is
   process-agnostic by signature.
2. **AtlasD** — a headless process hosting the monasteries: paints, solves, publishes.
   Tiles and Exchange buffers move over a shared-memory ring (Windows named sections — the
   64 KB tile is already the natural message), control over a named pipe: `Want(channel,
   tile)`, `Query(buffer)`, `Subscribe(channel)`. The renderer becomes a *client* whose
   residency provider fetches from AtlasD instead of painting locally; the compositor code
   does not change, it just runs in the other process. The sim keeps stepping when nobody
   is looking; the renderer *peeks in* — the user's phrase, and the correct contract.
3. **Many Vaticans** — exporter, notebooks, the MCP surface (§10), a Maxwell workbench:
   all AtlasD clients with the same three verbs.

What this buys physics: solver cadence decoupled from vsync; snapshot semantics (a client
always sees a consistent published step, never a half-written one — the double-buffer +
version pattern the Exchange already implements in-process).

## 9. The design pattern, stated once

Every future dataset — earth, water, air, electromagnetic — enters the same way:

1. Declare the **plane**: frame (CRS + datum), metric, lattice, fiber (value layout +
   grade signature + conservative/perceptual law), coverage, cadence, feather.
2. Give it a **Sample()** in the exchange frame with an exact projection. No linear
   shortcuts; the selftest pins the projection.
3. Assign it to a **channel** in its monastery, at a position in the stack.
4. Let **realizations** pick rungs by the rung rule; let the **soak** own propagation and
   redundancy; let cache identity be per-tile subsets.
5. Clients consume **channels** (rasters) and **Exchange buffers** (vectors/motors/
   multivector products) — never sources.

If a step can't be followed, the dataset isn't understood yet — which is precisely the
signal a schema registry is for.

## 10. The Scriptorium — the .NET/MCP keeper

The project crossed the size (≈13 k lines C++, ≈3 k HLSL, 11 harvesters, dozens of data
products) where "what writes this file?" and "where is that symbol?" deserve a service, not
a grep. Answer to the ask: yes — built as **`tools/Scriptorium/`**, a .NET 10 MCP server
(stdio) backed by **SQLite**.

Why SQLite and not MySQL: single-file, zero administration, transactional, right-sized for
a single-machine scriptorium; the schema is vanilla SQL and ports to MySQL unchanged the
day it needs to be multi-user. (If you want MySQL anyway, the connection string is one line
in `appsettings`.)

What it stores (v1, honest): a **symbol graph**, not yet a full AST — every type, function,
cbuffer, channel, and selftest across C++/HLSL/Python, with file/line and doc-comment,
reindexed on demand; the **harvester registry** — each script's purpose, inputs, outputs,
and the data products it owns; **provenance** — which script produced which `data/` file,
when, from which cached source; and the **monastery registry** — channels, sources, CRS,
rungs, mirrored from the code. Full libclang ASTs are the named upgrade path if the symbol
graph ever proves too shallow.

MCP tools exposed: `symbols(query)`, `who_writes(path)`, `script_for(product)`,
`channels()`, `reindex()`. Registered in `.mcp.json`, so any future session (or any other
MCP client) can ask the repo about itself instead of rediscovering it.

---

*The monasteries keep their own hours. The bible keeps them honest. The Vatican is where
the picture gets painted — and the picture, lately, looks like the Merrimack.*
