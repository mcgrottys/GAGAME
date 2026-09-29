# HIERARCHY: the earth's textures as windows onto one pyramid

*A proposal, 2026-09-28, written on `claude/earth-texture-hierarchy-53a3b3` at `e6acf22`. Of
sections 4 to 6 only step 0's probes are built (section 6, "Standing"); nothing else is. The
review that stands beside it is `docs/REVIEW_2026-09-28.md`; "finding N" below is that document's
numbering.*

How to read the numbers: **measured** means a harness named beside the number produced it on this
machine; **emulated** means float32 arithmetic reproduced operation by operation on the CPU, not
read back from the GPU; **documented** means a Microsoft or vendor page says so and this engine has
not yet checked it; everything else is design. The harnesses are in `tools/hierarchy/`.

## 0. The proposal in one paragraph

A Direct3D texture cannot be wider than 16384 texels, so no single texture reaches from the globe
to a centimetre. The proposal keeps ONE sparse pyramid per tenant on the CPU and on disk, addressed
by `(face, rung, x, y)` on the cube's own lattice with no notion of a regional or a micro texture,
and gives the GPU a small set of **windows** onto it: each window is one slice of the tenant's
reserved array, 16384 texels of its finest rung, addressed modulo its size so it can stand anywhere
and slide without moving a byte. Windows come in **ranks** three rungs apart (the ladder the engine
already walks: z8, z11, z14, z17, z20, z23). A point on the ground finds its window through a
**directory** that depends on the ground and on nothing else, so a camera, a second camera, a
portal's carried eye, a hull and a refracted ray all read the same texel of the same tile. Its
address inside the window is a **ratio of two plane evaluations** on the eye-relative position,
exact in float32 at every rung. Three laws govern what happens where windows meet: the **porch**
(two windows of one rung), the **phase** (two adjacent rungs) and the **floor** (what is resident).
The water is a tenant like the rest, written once and read by the hull and the renderer alike.

## 1. What is built today

Read from the code by the review (its section 6 has the tables and the counts):

- Five page tenants, each one reserved `Texture2DArray` of 16384 squared slices: the cube's faces
  in slices 0 to 5, and beyond them Mercator windows fixed at the Merrimack (z14 for all, z17 as
  well for the colour and the mask) or, for the wave tenant, the 33 planes of one z16 window.
- A page is found by **containment in one window** whose rows ride the surface constant buffer.
  No world has a list of pages. Away from the Merrimack every place, the far side of a gate
  included, is the cube's 611 m.
- A slice is **fixed at creation**. Nothing activates, moves or releases one; nothing maps one
  heap tile at two places; the adapter's address bits are not asked.
- `PageTable` and `LevelLadder`, the `(level, x, y)` space of SPARSE_GA section 21, have no caller
  outside the tests.
- The water is addressed **five ways** (ring metres, the anchor chart, the Mercator page, the wave
  chart's cells, latitude and longitude), and the CPU's bed mirrors the GPU's page-or-cube choice.

Four of the review's findings are this proposal's subject seen from the other side: a page never
hands back to its parent (finding 8, the missing phase), a page has no margin at its edge
(the missing porch), a kernel reads across a residency frontier (finding 9, the floor), and the
z17 page is addressed to two texels (finding 5, the address).

## 2. What the hardware allows

| fact | value | standing |
|---|---|---|
| widest texture, reserved or not | 16384 | measured (`--selftest`, SPARSE_GA section 15) |
| array slices | 2048 by the API; the address space runs out first | documented; measured |
| virtual address space of one reserved array | 512 GB created, 1 TB removes the device | measured (SPARSE_GA section 16), R16F with one mip |
| address bits per resource and per process | 40 for one resource (1 TB), 44 for the process (16 TB) on this adapter; at least 40 each on any feature level 12_2 device | measured (step 0: asked at boot, the `[gpu] virtual address bits` line); the minimum is documented. The array that removed the device asked for 2^40 bytes, which is the per-resource limit itself: consistent, not re-measured |
| NULL tiles read zero, swallow writes | tier 2 | measured (`TileSelfTest`) |
| packed mips on an arrayed reserved texture | tier 4 only | documented |
| one heap tile mapped at several places of one resource | holds: mip 0 of two slices and mip 1 of a third read the same 65,536 bytes, filled through one place; the two left keep reading it when the first is unmapped | measured (step 0, `[tiletest] share`), read back by `CopyTiles`, a range per place. NOT measured: read through the sampler, the API's single-tile range flag, under load |
| one heap tile shared by two resources | needs an aliasing barrier, and data is inherited only in the common state | documented. This proposal never does it |
| filling a tile that is mapped twice | undefined unless the bytes are identical: fill through one mapping | documented |
| hardware filtering across array slices | none | by construction: a slice is a separate surface |
| wrap addressing inside a slice | holds: a tap past u = 1 filters with the texel at u = 0 of the same slice, at mip 0 and under a residency clamp onto mip 1 | measured (step 0, `[tiletest] wrap`), pixel stage, through `PageSample` itself. NOT measured: the engine's anisotropic sampler, a fractional clamp, a tap into an unmapped tile |
| sampler feedback from any stage | `WriteSamplerFeedbackLevel` and `Grad` | documented; unused by this engine |

Two consequences shape everything below. **The budget is address space, and it is shared.** A
16384 slice with its chain costs 1.33 GiB of address space in RGBA8, 0.67 in R16F, 2.67 in RGBA16F
(`tools/hierarchy/stride.py`). One array must stay under the per-resource figure: by arithmetic some 760 slices of RGBA8 or 380
of RGBA16F, and the largest array this engine has created is 512 GB. All arrays together must
stay under the per-process figure, 16 TB here and 1 TB on the documented minimum. **A window cannot filter into its neighbour**, so wherever a filter
footprint could cross a window's edge, something must be arranged in advance. That arrangement is
the porch.

## 3. What others have done

Every ingredient below has prior art. What this proposal adds is the combination and the algebra it
is stated in. I searched for the combination within one session, a dozen queries and not a
literature review, and did not find it described in one place. That is weak evidence of novelty
and none of merit.

- **Clipmaps** (Tanner, Migdal and Jones, 1998): a stack of same-sized levels centred on the
  viewer, each covering twice the ground of the one below, updated toroidally. A window here is a
  clip level whose paging is done by the hardware's own tile mappings.
- **Sparse virtual textures** (Barrett 2008; id Tech 5; van Waveren 2012): one huge virtual
  texture, a software page table, physical pages with borders. Reserved resources replace the
  page table INSIDE a window; the directory replaces it BETWEEN windows.
- **Adaptive virtual textures** (Chen, GDC 2015, Far Cry 4): 10 km by 10 km at ten texels a
  centimetre would need a 512K virtual texture, so each 64 m sector is given its own virtual image
  whose size follows its distance. Trilinear is two fetches blended by hand; eight-tap anisotropy
  is bought with a four-texel border on every page. The same problem, the same two remedies.
- **Virtual shadow maps** (Unreal Engine 5): a 16k by 16k virtual map in 128-texel pages, a stack
  of clipmaps for the sun, a cube of 16k maps for a point light. The closest published shape to
  this one: the same page size, the same extent, the same two arrangements.
- **Proland** (Bruneton and Neyret, 2008): a planet as six quadtrees, tiles made by producers and
  cached in texture arrays, each quad handed the slot and the uv transform of its tile by the CPU.
  That is the binding of section 4.5.
- **FV3** (GFDL), the dynamical core of NOAA's own GFS since 2019, solves on a gnomonic cubed
  sphere. Its authors name the two costs this proposal meets: the local coordinates are not
  orthogonal, and they kink at the cube's edges, in the halo. A solver on the cube's lattice has a
  precedent that size, and the porch has a name there.
- **Ptex** (Burley and Lacewell, 2008): a texture per face with an adjacency table, so a filter can
  cross a face boundary. The porch at a cube edge is the same problem.
- **Shell maps** (Porumbescu and others, 2005) and their successors: the space between a surface
  and its offset is filled with prisms or tetrahedra, and a ray is carried into texture space cell
  by cell. Tetrahedra with affine maps buckle; a 2025 preprint (Hoetzlein) uses prisms and a
  projective map for that reason.
- **Tetrahedral cages** (Gruen and others, HPG 2026): section 4.10.
- **Projective textures** (Segal and others, 1992): a texture coordinate that is a ratio of two
  linear forms, divided at the pixel. The address of section 4.4 is that arithmetic: a central
  projection onto a face's plane is a perspective projection from the body's centre.
- **Rendering relative to the eye** (Cozzi and Ring, *3D Engine Design for Virtual Globes*,
  2011): positions are given to the GPU as small numbers about the eye, and the one large
  subtraction is taken in doubles on the CPU. Section 4.4 does to a texture's address what that
  book does to a vertex.

## 4. The structure

### 4.1 One pyramid, many windows

On the CPU and on disk there is one tree per tenant. A tile is `(face, rung, x, y)`: rung 0 is the
cube's mip 0, 611.496 m of nominal ground a texel; rung r is that divided by 2^r; the nominal grain
of rung r is a Web-Mercator zoom of 8 + r. Rungs below zero are the cube's own mips. The tree has
parents and children because a parent is the fold of its children (SPARSE_GA section 43), and it
has nothing else: no regional texture, no micro texture, no window. **That is the sense in which
the CPU tree has no LODs.** A physics query reads what the sources can paint at a place, at the
rung it asks for, and learns neither what the cache holds nor what the GPU has mapped: what is
on disk and what is mapped both depend on where somebody has flown. The CPU's own readers
already keep to that (`HeightPage` evaluates the sources at the page's texel centres, and
chooses its page by containment). One path does not: the solver's bed is read on the GPU by
residency, and the hull reads the solver (finding 48).

On the GPU a page tenant stays what it is today, one reserved `Texture2DArray`. Slices 0 to 5 stay
the cube's faces with their full chains. Every further slice is a **window**: 16384 texels a side
at its finest rung, carrying four mips (its own three rungs and one shared with the rank above),
addressed modulo 16384. Its fourth mip is its FLOOR, and the parent of a tile at the floor is
the same ground in the window of the rank above, not a coarser mip of its own slice; the
array's further mips exist, because one array has one mip count, and a window leaves them
unmapped. A global texel `X` at the window's rung lives at `X mod 16384`, and at mip
m at `(X >> m) mod (16384 >> m)`; those agree for every window origin (measured: 200,000 random
pairs, no disagreement, `tools/hierarchy/porch_floor.py`), so a WRAP sampler and the hardware's own
chain read the global lattice correctly at every mip. Finding 4, a page whose coarse mips sit 100 m
from where they are addressed because its origin is not a multiple of 2^m, cannot be written in
this form. Moving a window maps the tiles that enter and unmaps the tiles that leave. No byte is
copied, and a tile's content is where the ground is.

A window is declared by three things: the face plane it lies in, its finest rung, and the tile its
centre stands on. Whether it **follows** something (a view, a carried eye, an interest) or
**stands** over a place (a solver's domain, a harbour with a survey) is the allocator's policy and
not a second mechanism.

The surface tenants share the same windows: slice i is the same ground in the colour, the height,
the mask and the exposure. One directory walk then serves every tenant a pixel reads, which is the
GPU-resident law's "one lattice" carried to the slices.

### 4.2 The lattice is the cube's

Today the regional pages are Web-Mercator windows and the globe is a gnomonic cube: two lattices,
met by resampling wherever a window hands over to the cube. The proposal puts every window on the
cube's own lattice, in the plane of a cube face. (An earlier version of this section had a
window extend past the face's square where it needs to. The tree's address is unsigned, so a
tile past a face's edge has no name today. Section 4.18 gives it one: an apron is a face of
its own.) The reasons, in order of weight:

1. **One lattice.** A window's texels ARE the pyramid's texels. Where two windows answer for one
   ground they hold the same bytes, and where a window hands over to its parent it does so at a
   rung both carry. Nothing is blended because nothing differs.
2. **The address is exact.** A central projection is a ratio of two linear forms, so the window
   coordinate of a point is one division in float32 (section 4.4). Mercator needs a logarithm of a
   tangent, which float32 cannot hold at these magnitudes and which has no closed form relative to
   the eye.
3. **A straight ray stays straight.** The central projection of a line is a line, so a ray crosses
   the texel grid of a face plane along a straight path: a two-dimensional walk, exact, with the
   residency pyramid there to skip what is empty. Under Mercator the same ray is a curve.
4. **The poles exist.** Mercator stops at 85 degrees.

What it costs, stated plainly:

- **The texels are not square on the ground.** Measured (`stride.py`), rung 0: 778 m by 778 m at a
  face's centre, 550 by 389 at the middle of an edge, 367 by 367 with the axes 120 degrees apart
  at a corner, and **523 m by 395 m with the axes at 103 degrees at the Merrimack's mouth**, which
  lies 155 texels from the edge of face 5. A Mercator texel of the same zoom there is 449 m square.
- **Google's imagery is resampled.** Today a z14 window texel IS a Google pixel. On the cube's
  lattice it is not, and the rung rule (ATLAS section 4) then asks for the first rung strictly
  finer than the source: one rung finer, four times the texels, for imagery only.
- **The window caches are repainted** from the source caches. Counted: the colour tree written
  to today holds 4,167 z14 and 7,775 z17 tiles, and the fullest of its older identities 4,270
  and 16,565. No source is fetched again if the paint asks the zooms already cached; that is
  checked before the first paint, not after.
- **The cube's edge runs through the home waters.** On the Merrimack's meridian faces 5 and 2 meet
  at 43.364 N, **60.9 km north of the mouth** (measured). It is the one place a porch must blend,
  and it is near enough to drive to. The mesh's level seams across cube faces are open today and
  would show on the same drive.

### 4.3 Ranks, and why three rungs

Two levels of 16384 do reach 3.7 cm: 2^14 times 2^14. The address space is enough. What is not
enough is the **reach** of a window. A rung is wanted on the ground out to the distance where one
of its texels fills one pixel, and in that rung's own texels the distance is always the same: one
over the pixel's angle, about a thousand texels at 1080p. A window carrying s rungs must reach
2^(s-1) thousand of its own finest texels, and it is only 16384 wide.

Windows one viewer needs at each rank (arithmetic, `stride.py`):

| display | pixel, mrad | 2 rungs | 3 rungs | 4 rungs | 7 rungs |
|---|---|---|---|---|---|
| 1600 x 900 | 1.164 | 1 | 1 | 1 | 49 |
| 1920 x 1080 | 0.970 | 1 | 1 | 4 | 81 |
| 2560 x 1440 | 0.727 | 1 | 1 | 4 | 121 |
| 3840 x 2160 | 0.485 | 1 | 4 | 9 | 289 |

The trees on disk say the same (measured: a census of `cache/trees` by file name,
`tools/hierarchy/tree_census.py`). The cache holds a dozen identities of the colour tree, one
for every change of its sources or its code, and each holds what was flown while it was
current. Two of them, the fullest (`earth.color.819596d8`, written on 2026-09-01) and the one
written to today (`earth.color.c343ccc5`):

| realization | identity | mip 0 | mip 1 | mip 2 | mip 3 and coarser |
|---|---|---|---|---|---|
| the cube (98,304 tiles at mip 0) | the fullest | 1.5 % | 4.5 % | 11 % | 35 %, 84 %, then all |
| | today's | 1.4 % | 4.4 % | 20 % | 56 %, then all |
| the z14 window | the fullest | 9.1 % | 39 % | 83 % | all |
| | today's | 7.8 % | 37 % | 99.5 % | all |
| the z17 window | the fullest | 68 % | all | all | all |
| | today's | 25 % | 59 % | 98 % | all |

A window's fine mips are asked for near where an eye has been and nowhere else, and its mip 3 is
asked for everywhere in it. That is a window three rungs deep whose fourth belongs to its parent.
An earlier version of this section gave the fullest tree's numbers as "after every flight flown
so far"; they are one identity's, and the step 4 mapping caught it.

So "global, regional, micro" at seven rungs apart is three ranks and some eighty windows a rank;
three rungs apart is five ranks and one window a rank, and it is the ladder the engine already has:

| rank | finest rung | nominal texel | zoom | wanted within (1600 x 900) | today |
|---|---|---|---|---|---|
| 0 | 0 | 611.5 m | z8 | 526 km | the cube |
| 1 | 3 | 76.4 m | z11 | 66 km | missing: the z14 page's coarse mips stand in, inside its 156 km |
| 2 | 6 | 9.55 m | z14 | 8.2 km | the Merrimack window |
| 3 | 9 | 1.19 m | z17 | 1.0 km | the detail window |
| 4 | 12 | 14.9 cm | z20 | 128 m | the MassGIS orthos, exporter only |
| 5 | 15 | 1.87 cm | z23 | 16 m | none |

The stride is one constant. The structure holds for any stride up to seven; three is the
recommendation, and at 4K it wants either two or four windows at the finest ranks.

### 4.4 The address is a ratio of two planes

For a face whose plane has the unit normal n and the in-plane axes a and b, the face coordinate of
a point P of the body is `s = (P . a) / (P . n)`. A window's origin edge is the plane through the
body's centre with `s = s0`, that is, the plane whose normal is `a - s0 n`. With the eye E held in
doubles and the point given relative to it, P = E + p:

    texel = k ( A + p . (a - s0 n) ) / ( E.n + p.n ),      A = E . (a - s0 n)

`A` is the one large cancellation, taken in double on the CPU: it is the eye's own distance from
the window's edge plane, a few hundred metres at the finest rank. Everything the shader adds to it
is eye-relative. In the algebra this is the regressive product of a point with a plane, twice: the
coordinate is `(P v edge) / (P v base)`, a ratio of two oriented distances, and a window is a cell
bounded by four such planes through the body's centre. Nothing in it is a matrix, and the planes
move into any other space by `Placement::PullPlane`, which spacetest already holds. The
arithmetic is a projective texture's, taken relative to the eye (section 3): three planes and a
division. It is not new; it is what the lattice of 4.2 makes available.

Worst error in texels, the Merrimack helm, 4,000 ground points within reach of the eye (emulated,
`tools/hierarchy/uv_precision.py`):

| rung | texel | from the float32 direction | today's Mercator spelling | ratio of planes |
|---|---|---|---|---|
| 6 (z14) | 9.55 m | 0.06 | 0.27 | 0.003 |
| 9 (z17) | 1.19 m | 0.43 | **2.0** | 0.002 |
| 12 (z20) | 14.9 cm | 3.7 | 18 | 0.003 |
| 15 (z23) | 1.87 cm | 29 | 138 | 0.003 |

The Mercator column is kind to today's shader: it starts from exactly rounded latitude and
longitude, where the shader starts from `asin` and `atan2` of a float32 direction. From 10 km up
the ratio's error at rung 15 rises to 0.045 texel, because the point is then 10 km from the eye;
nothing at that height wants rung 15.

### 4.5 Finding the window: the directory and the binding

**The directory** is a 16 by 16 grid beside every slice, two 16-bit slice numbers a cell. A rank's
window covers an eighth of its parent's extent, so a sixteenth is half a window, and a cell names
the window of the next rank that cell lies deepest in, and a second window where a porch must
blend. The walk starts at the face (the largest component of the direction), reads a cell, steps
into the window it names, and stops where no window is named, where the named window has nothing
resident there, or where the wanted rung is reached. It is five dependent loads at most for the
whole ladder, of textures a few kilobytes in size, once per pixel for every tenant. Today a shore
pixel resolves its page nineteen times.

**The binding** is the same answer computed on the CPU. The walk of the planet already visits
every node it draws and already asks for that node's tiles; it can name the node's window in
the record it hands the mesh stage, which has 29 spare bits beside its level. The window's
planes are not the record's: they are the same for every record of one world, so they ride one
table a world, and a record of 96 bytes stays 96. A rasterized pixel then pays no directory
load at all. A ray that leaves the surface (the pixel
water's two rays, a march, a compute kernel) has no node and walks the directory.

The two must be one function. It is written once with a C++ body and an HLSL body, and a gate
compares them on random ground points through a GPU readback: the instrument that can see them
disagree.

What the directory must never depend on is a camera. Which windows EXIST follows the viewers, as
residency does. Which window ANSWERS for a piece of ground, given the windows that exist, is a
function of the ground.

### 4.6 Three laws where windows meet

**The porch: two windows, one rung.** A window answers for a cell only where the cell lies at
least one tile of its coarsest carried mip inside its edge: 1,024 of its finest texels, 128 at
that mip, more than any filter's footprint. That margin is the porch. Where two windows of one
rank both cover a cell, the directory names the one the cell lies deeper in. On one face plane the
two windows hold the same bytes, so the value is the same whichever is read and nothing is
blended. Across an edge of the cube the two lie in different planes and hold two resamplings of
the same sources; there both are read and mixed by a smooth weight of the signed distance from the
edge plane, itself one more ratio of planes, over a band a few hundred texels wide. It is one law,
`lerp(A, B, w)`; on one plane A equals B and the second read is skipped because it cannot change
the answer.

**The phase: two rungs, two windows.** A window carries one rung in common with the rank above
it, so every pair of adjacent rungs lies inside one window and the hardware's own trilinear does
the mixing. Which window serves a pixel is decided by the rung the pixel WANTS, not by what is
resident: the hand-over happens at the rung both hold, where they hold the same texels. That is
what finding 8 lacks. It costs the shared rung twice, about 200 tiles a window in each RGBA8
tenant, unless the tile is mapped in both places. Step 0 measured that mapping on this adapter:
one heap tile reads the same bytes through mip 0 of one slice and mip 1 of another. On an adapter
where it does not hold, the window carries three mips and the one pair that straddles two windows
is fetched twice and mixed by hand: the same law, executed by the shader.

**The floor: what is resident.** Today the clamp is the largest of the four nearest residency
bytes: a staircase, so the picture's sharpness steps along tile lines. The first proposal was to
give each byte the largest of its 3 by 3 neighbourhood and read the map with plain bilinear
filtering. On random maps it is never finer than today's clamp, the largest step between
samples a sixteenth of a cell apart falls from 7 mips to 0.44, and it costs 0.15 of a mip
(`porch_floor.py`). **Measured on the GPU, it is not sound, and it was not staged** (step 2,
2026-09-29).

The instrument is the M6h case made a probe: a reserved array with some tiles mapped and
filled and their neighbours NULL, read through `PageSample` at 473,088 points a sampler, under
footprints of ratio 1, 4 and 8, and the question asked of each point is whether any of a NULL
tile's zero came into the sample.

| the law | trilinear | anisotropic 8x, the colour's sampler |
|---|---|---|
| today's: gather and max, the true map | 0 | 1,606 |
| the 3 by 3 floor, read bilinear | 0 | 4,221 |
| a 5 by 5 floor, read bilinear | 0 | 5,471 |
| the true map read bilinear (M6h, the plant) | 18,463 | 27,575 |
| gather and max over the 3 by 3 floor | 0 | 77 |
| gather and max over a 5 by 5 floor | 0 | 0 |

Why: where the clamp decides, this GPU keeps the footprint's ratio and scales the whole
footprint to the clamped level. At a clamp L a footprint of ratio N reaches N / 2 texels OF
MIP L along its long axis, and one more for the bilinear tap: measured, 2.00 cells at a clamp
of 6 for a ratio of 8, which is four texels of mip 6. So the reach is a number of texels of
the level read, and a number of cells is the wrong unit. A cell is 128 texels of mip 0 and two
of mip 6: a 3 by 3 of cells is sixty times the margin mip 0 needs and half of what mip 6
needs, and a ramp that raises the clamp ahead of a frontier sends the wide footprint of a
coarse level across it. Today's law has the same fault where the hardware's own level decides,
under footprints 256 to 512 texels long (finding 61).

**In its place: a margin per level.** A level may be read at a place only where the place
stands at least M of THAT level's texels inside what is resident at that level, and the same
holds at every coarser level. It is one law at every level, and its unit is the texel that is
read. The byte is computed where the map is written. Measured in the same probe, first at
footprints whose length is a power of two and then at lengths between them, 2^(k + 0.5) and
2^(k + 0.9): the points that touch a NULL tile under the anisotropic sampler, of 473,088 and
of 811,008. Under the trilinear sampler every row is zero.

| M, in texels of the level read | gather and max over the floor | bilinear over the floor's 3 by 3 |
|---|---|---|
| 2 | 77, not run | 5,471, not run |
| 4 | 0, then 34 | 497, then 750 |
| 6 | 0, 0 | 0, then 60 |
| 8 | 0, 0 | 0, 0 |
| 10 | 0, 0 | 0, 0 |
| 12 | 0, 0 | 0, 0 |

The lengths between the powers are what decide it, and the first run had none. A sample
whose level is L + f reads levels L and L + 1 with its taps at the same places, so in the
texels of L, the finer of the two, its long axis is 2^f times as long: the reach is
(N / 2) 2^f texels, and one for the bilinear tap. Measured for a ratio of 8 at levels 6.00,
6.25, 6.50, 6.75 and 6.90, in texels of mip 6: 4.0 to 4.4, 4.6 to 5.1, 5.4 to 5.9, 6.4 to 6.8
and 7.0 to 7.4, each under its bound of 5.0, 5.8, 6.7, 7.7 and 8.5. As f goes to one the
bound goes to N + 1. So **M is the sampler's anisotropy and two**: nine is the bound for the
engine's sampler, and one texel is kept in hand. M = 8 reads zero at every fraction the probe
has, and the probe has none above 0.9. The same bound holds where the hardware's own level
decides, which is where today's law fails: at the lengths between the powers it lets a NULL
tile into 3,635 samples.

**The read is the bilinear one:** the floor's 3 by 3, read with plain filtering, one sample
where today's clamp gathers four bytes and takes the largest. Its largest step between samples
a sixteenth of a cell apart is 0.44 of a mip where the gather's is 7, which is what the floor
was wanted for. Its cost in sharpness is known on random maps only: 0.43 of a mip in the mean
beside today's clamp, and 0.26 where no face is dead. A gather over a 5 by 5 floor, the other
law that read zero at the powers of two, costs 0.20 and 0.19 there and keeps the step of 7.
On a flight the cost is not measured, and the rail decides. **The want is owed the same
margin:** what a view reads at a level, grown by M of that level's texels, or the outer band
of every level reads the level above it. That is at most one more ring of tiles at a
level's frontier.

**What others have done.** The fault is known. Intel's sample of sampler-feedback streaming,
which is this engine's arrangement (tiled resources, a map of the finest level held a
region, a clamp in the shader), says that the hardware sampler reaches across tile
boundaries under anisotropic sampling and meets tiles that are not mapped, and proposes to
dilate or erode the map until neighbouring regions differ by one level at most; it leaves
that undone. A slope of one level a region is not the margin measured here: at mip 6 the
reach is two and a half regions, and at mip 0 a twenty-fifth of one. I did not find a margin
stated in texels of the level read; that is two searches, not a survey.

**Staged in the engine, 2026-09-29, and not kept in that form.** Through the manager's own
map the law touches no NULL tile under either sampler, at footprints of every length the
probe has. The audit is clean at every audited turn of a still and of the rail. Every
tenant's hashes are the unstaged binary's at the six poses, and the kernels' fingerprints
with them. Two things fail it.

It is softer than the random maps said. The clamp it sets, less today's, as a mean over the
frame's ground, measured with step 1's lens, which resolves whole mips:

| pose | coarser by, mips | ground pixels a level coarser | pixels that differ, of 1,440,000 |
|---|---|---|---|
| helm | 0.002 | 0.2 % | 994 |
| 7 km | 0.004 | 0.4 % | 3,306 |
| bird | 0.054 | 5.4 % | 181,570 |
| Droste | 0.078 | 6.3 % | 31,663 |
| globe | 0.40 | 29.6 % | 94,367 |

It is never finer than today's. At the globe the loss is plain in the picture. To give the
sharpness back the want would have to grow by what the read reaches, two cells and the
margin, and that doubles what a view wants: 13,939 tiles more on 14,935 at the helm. And
the map's refresh costs 1.13 ms in the mean where it cost 0.13, 4.7 ms at the 95th turn in
a hundred, 50 ms at the worst, though it recomputes only what a turn touched.

Both have one cause: the margin is small and the map's cell is large. Ten texels of the
level read are 0.078 of a tile at EVERY level, because a tile is 128 of its own level's
texels. A cell is a tile of mip 0, and its byte can say only that the whole cell may read
a level or may not. So at the finest level a band of ten texels costs a cell of 128, the
bilinear read spreads that over two cells more, and at the coarse levels the margin is a
window ten cells wide to search.

**Proposed in its place: the same law, evaluated where it is exact.** At level L a point's
place in its own tile is the fraction of `uv * tiles(L)`, and the level may be read unless
the point stands within 0.078 of an edge or a corner across which the tile is absent. So the
map carries, beside the byte it has, one byte a TILE at every level: which of the tile's
eight neighbours are absent. That is a texture of 128 texels a side with a mip chain. The
shader walks up from the byte's level, a load a level, and the clamp is

    clamp(p) = max( byte, max over L of ( (L + 1) (1 - s(d_L)) ) )
    s(d) = smoothstep(M, 2 M, d)

with `d_L` the point's distance, in texels of level L, to the nearest edge or corner whose
tile is absent. It is continuous. It is never finer than the law, because where `d_L` is
under M the level's term is L + 1; and a level whose edge is 2 M away or more asks nothing.
It is coarser than the tiles only within 0.16 of a tile of where a level ends. As this
section first wrote it the term was L + 1 - s, which asks L of every level however far
its edge stands, so the coarsest level always won. The probe caught it before any picture
was taken: the margins planted too small touched nothing, and the suite failed. A tile that is
mapped or released writes eight bytes. And the want grows by the margin alone, a neighbour being
wanted where a read stands within 0.078 of a tile's edge: some tiles in a hundred, not as many
again. Nothing of it is built. The probe and the lens that judged the first form judge this one.

**Built by the tile and gated, 2026-09-29, and not kept in that form either.** It is sound:
through the engine's own shader function no sample holds any of a NULL tile's zero, under
either sampler, and the margins planted too small are caught. The flags equal the tiles at
every audited turn. Every hash is the unstaged binary's. And the manager's cost is today's:
the map's refresh 0.122 ms in the mean where today's is 0.124, because a tile that flips
writes its neighbours' bytes and nothing is searched.

| pose | mean level of the clamp: today, by the cell, by the tile | ground a level coarser than today: by the cell, by the tile | pixels that differ from today: by the cell, by the tile |
|---|---|---|---|
| helm | 0.009, 0.011, 0.009 | 0.2 %, 0.0 % | 994, 23 |
| 7 km | 0.019, 0.023, 0.016 | 0.4 %, 0.0 % | 3,306, 928 |
| bird | 1.020, 1.074, 1.010 | 5.4 %, 0.0 % | 181,570, 10,266 |
| Droste | 1.839, 1.917, 1.825 | 6.3 %, 0.1 % | 31,663, 1,882 |
| globe | 0.594, 0.995, 1.792 | 29.6 %, 35.6 % | 94,367, 142,498 |

Near the ground it is what was wanted: nothing a level coarser, and where it differs from
today's clamp it is the finer, because today's gather coarsens half a cell at a frontier
and this coarsens some texels. Two things fail it. At the globe it is coarser than either,
by more than a level in the mean, and the picture has lost its texture there. And the GPU
pays for the walk: the globe's pass takes 28.0 ms at the helm where it took 13.5, and
12.8 ms at the bird where it took 6.4, every page read of every pixel loading a level's
flags at a time.

Why the globe: a level's band is counted in that level's texels, so where the levels read
are coarse and what is resident of each is a few tiles across, the bands are a large part
of the ground; and the ramp this form was given runs from M to twice M, twenty texels, a
sixth of a tile a side. By the tile a wanted region three tiles across keeps its level
over half its area. Today's clamp gives a coarse level a margin of one or two of its
texels, which is why today's picture is sharp there and why today's law is the one that is
not sound there (finding 61).

**What soundness costs, stated once.** A sample's footprint reaches M texels of the level it
reads. Either the tiles within that reach are resident, which asks of a view one ring of
tiles beyond what it looks at, at every level (counted as an upper bound: 6,293 more on
14,935 at the helm, 983 on 1,676 at the globe); or the sample gives way where they are not.
It can give way in two manners. It can read a coarser level, which is what both forms built
so far do, and costs a level over the whole reach. Or it can shorten its footprint to what
is resident around it, and read a coarser level only where the shortest footprint there
is, a texel and its bilinear tap, would still reach an absent tile.

**The form that follows, proposed: a footprint reads no tile that is absent.** Within M of an
absent tile the footprint's long axis is cut to the distance, by the gradients handed to the
sampler; within three texels, the trilinear read's own reach, the level is raised, over a
ramp of two texels more. The flags and their pyramid stay as built. A cell's byte carries
one bit more, set where any level over the cell has an absent neighbour, so that a pixel
far from every frontier loads one byte and walks nothing. A slice read with a clamping
sampler has no neighbour past its own edge to touch, so its edge sets no flag. The band is
then five texels where it was twenty, and what is lost inside the reach beyond it is the
length of a footprint and not a level. Nothing of it is built. Its gate is the same probe,
the same lens, and the globe's pass no slower than today's.

What the probe cannot tell: another adapter or driver, or an anisotropy other than 8;
fractions above 0.9; a frontier other than a cliff, where every level below the coarsest ends
at once; footprints that cross it at angles other than 0, 45 and 90 degrees; the picture.
The probe stays in the selftest, so an adapter whose taps reach farther fails the suite and
says where.

What stands of the first proposal: the construction is gated on the CPU with its plants caught
(the floor without its neighbours across a face's edge falls short at 534,276 points, all
within half a cell of an edge), and a kernel's single byte covers its four taps under it
(finding 9: 0 of 14 million taps in a coarser cell, where the true map has 11,505).

### 4.7 Residency, sharing and the budget

A tile belongs to the pyramid, not to a window. The residency manager's tracked unit becomes the
global tile, and a window holds mappings of it. With following windows a tile is mapped once in
the common case. It is mapped twice in two cases: the rung a window shares with its parent, and
two windows of one rank that overlap (two viewers a few kilometres apart, a standing window under
a following one). It is never shared across a cube edge, where the lattices differ, and never
between two tenants.

The budget: sixty-four windows in the four surface tenants are 256 GiB of address space. The wave
tenant's slices are planes, 33 to a window, so it is given its windows separately: 44 GiB each.
The pool is what it is today; the windows change where tiles are addressed, not how many a view
wants.

Three things in the manager must change before a window MOVES, because a moving window leans
on all three: the tail a batch loses at the pool's cap (finding 2), the residency bytes an
invalidation erases (finding 3), and the residency map's upload, which today re-sends a whole
tenant for one dirty byte. A window that stands on an aligned block adds nothing to them
(4.17), which is why the shader's contract is made first, on standing blocks, and the manager
after it. They are today's faults all the same. Step 1's audit saw the first two on today's
binary, over today's windows, and two more beside them (findings 63 and 64).

**The map is computed, not kept.** The residency byte of a cell is a function of the tiles:
the finest level L at which the tile over the cell is mapped and its bytes have landed, at L
and at every coarser level. Today's manager keeps the byte by increments, lowered when a tile
is mapped and raised when one is unmapped, and the increments equal the function only while
tiles are mapped from coarse to fine and unmapped from fine to coarse. Four orders of events
are not that. A parent is invalidated over a mapped child and mapped again, and the byte is
left coarser than the tiles (finding 3, seen). A parent is evicted while its child waits in
the same batch (63, seen). A tile is admitted under a chain that is broken above its parent
(64, seen). A claim is made while the parent's bytes are still in flight (24, read and not
seen). The last three leave the byte FINER than the tiles, and that is the direction in which
a sampler reads a NULL tile's zeros.

Each could be mended where it stands. Proposed, one law that mends them together: the byte is
written from the function, over the footprint of every tile whose state changed in the turn.
The audit's function is that function, so the audit becomes the gate of the code that
replaces what it judged. The margin of 4.6 is computed from the same residency, a level at a
time, so what the GPU reads is one function of the tiles: what is resident at each level,
and then the margin. A tile mapped under a broken chain is then a tile wasted until its
parent returns, and never a tile read.

The lost tail is a fault of the queue and not of the byte. On the storm rail the pool is full
on half of the turns, and from the first of them to the last the manager loads 7,917 tiles,
gathers them into batches, finds no slot, and leaves them in no queue: as many as the pool
holds. They are never mapped, each keeps its 64 KB, and what waits beneath them waits for
good. Returning the tail to its queue is three lines; what it needs with it is a bound, so
that nothing is loaded that no slot can take, and that bound is the new manager's (4.19).

### 4.8 Portals, and several eyes

A gate's world is rasterized, not traced: the far world is walked from the carried eye, drawn in
this frame by one motor, and a per-pixel slab test keeps each pixel for the world whose depth its
ray reaches. So every world has its own meshlet records, and the binding of section 4.5 gives each
record its window, and each world its table of planes relative to THAT world's eye. A carried eye is an interest
like any other, and windows follow it as they follow the camera; the gate's residency sampler
already exists.

What makes two eyes agree is that nothing about a texel depends on who looks. The lattice is the
ground's, the tile is the pyramid's, the directory is keyed by ground. Two worlds that see one
piece of ground read one tile through one window. The gate for it is direct: one ground point
read back through two worlds, equal.

**Several gates at once is a requirement** (the owner, 2026-09-28): at an inlet, a portal onto an
aerial view of that same inlet. For the textures it is the easy case and the one that shows the
point of the structure: the aerial eye wants coarse rungs of the very pyramid the helm reads fine,
so the two worlds share tiles outright and the far one costs only what the near one does not
already hold. For the gates it is work: today the chain takes one box a depth (finding 15), and
several gates on screen need a tree of windows.

What the hierarchy does not touch: that tree, a gate past 90 degrees of arc reading the wrong water
until the rings retire (finding 31), and the globe's state being built once a frame for one view.
Those are the gate's and the renderer's, not the textures'.

### 4.9 Bodies and entities

An entity is a space with a similarity placement; the boat already is, with a floating origin it
re-centres within a kilometre and an interest that keeps its surroundings resident. That is the
pattern to keep. What generalizes it is one word: a **body** is anything with root charts. The
earth's roots are six gnomonic faces. Mars has six of its own. An entity's roots are the charts
of its own mesh, in its own unit length. Below the roots everything is the same: windows, the
directory, the three laws.

Scale enters in one place. A placement carries a scale s, and a footprint of f metres in the world
is f / s metres in the entity's own space. So as an entity grows, the rung its OWN texture is
wanted at rises by log2 s (its texels grow with it, and the same pixel asks for finer ones), and
the rung of the EARTH it wants under it falls by log2 s (its hull's features are s times as
large). An entity that grows from a boat to a continent changes one number in its placement
continuously (the power of a similarity, which `Placement::PowApply` already applies about its
fixed point), its own coordinates never change, and its windows change rank as any viewer's do
when it climbs. The Droste tower's ground weight, r^2 / (h^2 (r^2 + h^2)), already says which
ground owns an eye among bodies of different sizes; an entity large enough to stand on is one
more ground in that sum.

Two things to hold to, so that nothing built now blocks this: a sampling law takes its footprint
as an input and never assumes a metre is a metre of the earth; and an address is given in a space,
with the planes pulled into it, never in "the world".

### 4.10 Cells, and the tetrahedral cage

**The technique** (from AMD's two articles; the paper itself I could not open). A coarse
tetrahedral cage surrounds a dense mesh. The mesh is cut once into pieces, one to a tetrahedron,
each with a small static acceleration structure in the REST pose. Only the cage is animated. A ray
that enters a deformed tetrahedron is clipped to it and carried by that tetrahedron's affine map
into rest space, where it meets the static piece. 585 million animated triangles at 60 frames a
second; 1.7 GB of structure where a rebuild would hold 80.

**Are we already doing our own flavour?** In two halves that do not yet meet:

| | the engine has | a cage needs |
|---|---|---|
| a cell with a test | a gate's box: three slabs, per pixel (`GateSlabFrom`) | four planes |
| a map on entering | one rigid motor for a whole world; the WORLD is moved, not the ray | an affine map for each cell |
| a per-cell affine chart | the fine meshlet's anchor and Jacobian: forward only, rebuilt each frame, no inverse, no containment test | the inverse, into a rest space |
| cells that know their neighbours | none: gates are scanned in a list, one chain | adjacency through faces |
| an index over cells | none for volumes; the quadtree indexes the surface by address | one |
| a payload in a rest space | a tile has an address; nothing names a rest-pose piece | the static piece |

The engine has no ray-tracing structure at all: no site in `src` or `shaders` names one.

**What the hierarchy supplies.** A window is a cell bounded by planes through the body's centre.
Its coordinate is a ratio of plane evaluations. A tetrahedron's barycentric coordinate is a ratio
of plane evaluations with a constant denominator: `(P v face) / (V v face)`. They are the same
record: a few planes, a coordinate that is a ratio of two of them, a payload. A gate's box is that
record with a rigid map. So one **cell contract** serves the window now, the gate when it is
restated, and the cage when it is wanted: the planes (for the test and the interval along a ray),
the coordinate forms, the map to the payload's space, the payload, and the neighbours. For windows
the neighbours are arithmetic on an address and the index is the directory; both exist the day
windows do.

**Which algebra.** The test and the coordinates are incidence: joins, meets and the regressive
product, which is projective geometric algebra's own. A placement is a versor and stays one. A
cage's map is neither: it is linear on points, an outermorphism, and it is not conformal. A 3 by 4
array appears only where a ray-tracing instance asks for one, derived, at the boundary, as the 4
by 4 does at the rasterizer.

### 4.11 The water, and the boat on it

The rule, in Mark's words: the renderer should not know which part of the water is a wake; it is
fed the same water as the boat. Today it is the other way round. The renderer's kernel adds wakes
from a table and the hull's twin is never given one (finding 13). The two read the solver at
different texels (12), blend the solved field by a law that loses variance (14), fall back to
different levels where a tile is absent (10), and classify the shore by the Merrimack's tide
everywhere (11).

The structure for the rule: **the water surface is a tenant.** Its producers write it: the tide's
level, the solver's deviation, the waves, the wakes of every hull, the player's included. Its
readers read it: the mesh, and the hull. On the shared windows, by the one address, under one
statement of what absence means (the coarser ancestor, never zero). A wake is then something a
hull deposits and nobody reads by name.

What that settles by construction, and what it does not:

| cause of a boat and a sea that disagree | findings | settled by the structure? |
|---|---|---|
| two addresses for one place | 12; the water report's A6, A11, A13 | yes: one address law on both processors |
| two answers where a tile is absent | 10, 21 | yes: absence is declared once, with the tenant |
| one level for every world | 11, 20 | yes: the level is the tenant's, at the place |
| the drawn sea is coarser far from the eye | the water report's A12 | bounded: a coarse rung is the fold of the rung the hull reads |
| two expressions of one law | 13, 14; A9, A14 | no: only one producer settles it |
| time | 18; a bucket roll | no |

The last two rows are settled by a decision, not by the structure.

**One description, three executions** (the owner, 2026-09-28: two players' machines must compute
the same boat, bit for bit, and a service may compute the water once for both).

- **The record** is the water the hull feels. It is computed by an execution that gives the same
  bytes on every machine: doubles, a fixed order of operations, its own elementary functions.
- **The picture** is the GPU's float32 execution of the same description. It draws. A standing
  probe at every hull holds its difference from the record to a bound that is stated.
- **The stream** is the record computed by a service and sent as tiles to every client, the
  64 KB tile being the message ATLAS section 8 already names. A client that reads the stream
  feeds the same tiles to its hull and to its renderer, which is the rule of this section in its
  plainest form.

All three come from ONE description of the water, so that a term cannot be added to one and
forgotten in another, which is how finding 13 happened. Which execution a producer uses is part
of how the scene binds it, beside its clock and its gain.

What makes it possible is the project's first thesis: wherever it can be, the water is a pure
function of the place, the instant and the bound data, and carries no state. Where it cannot,
the solver's deviation and a wake, the state advances on the fixed-step clock from declared
beginnings, and is as repeatable as the execution that advances it.

What "bit for bit" costs, named before it bites:

- A GPU's float32 is not the record. Two vendors round a sine differently.
- The C runtime's own `sin`, `exp` and `pow` may take a different path on a processor that has
  fused multiply-add, so two CPUs can differ in the last place. The record needs elementary
  functions of its own, or the runtime's dispatch pinned.
- A compiler may fuse a multiply and an add where it likes (the Droste gate met this, ALGEBRA
  section droste). The record is compiled with contraction off.
- Physics must never read residency. What is mapped depends on the wall clock and the disk. This
  is the other reason the CPU tree has no LODs.
- The stream avoids all four: one machine computes and every client reads the same bytes.

**Global data is the floor** (the owner, 2026-09-28). The level that draws the shoreline and the
level the water stands at are one number, read from one structure. Every tenant has a root that
covers the planet from a global source (ETOPO for the bed, EOT20 for the tide, GFS-Wave for the
sea state), and a finer source refines it on the same lattice where it exists: a station, a buoy,
a survey, a satellite pass. That is the compose stack the colour and the height already are; the
water's sources join it rather than keeping charts of their own. Charted and uncharted water then
differ in how deep the pyramid goes at a place, and in nothing else.

**Time is part of an address, and whose time is a binding** (the owner, 2026-09-28). A tile of a
quantity that changes is keyed by the instant it is valid for, as the exposure and the wave trees
already re-key when their bucket rolls. But there is no one clock. The scene binds each producer
to a clock of its own: the tide to now, the sea to a hurricane of months ago, the sky to a sun the
scene has placed where it likes. The sun is already free of the tide's clock; this makes that the
rule instead of the exception. A binding is a placement on the time axis, `t_source = t0 + rate
(t_scene - t_ref)`: an origin and a rate, the one-dimensional case of the similarity every space
already sits in its parent by. A tile's key holds the SOURCE's instant, so a storm replayed twice
is painted once, and a cache never learns what the scene's clock said. What the structure owes
the binding is only that nothing reads "the time": every producer is handed its own.

**A value is bound as a clock is** (the owner, 2026-09-28). A gain, an offset, a unit and a datum
are part of how a producer enters the scene, stated once, in the scene, where the data comes in.
Both readers then receive the same number because both read the product. The solver's current
gain of 3.2 is the case in hand: today it is applied inside one consumer, the wave solve, and
neither the hull's drift nor the kernel's own gain sees it. What a view adds for the eye alone,
as the relief's exaggeration is added today, belongs to the view's lens (4.13) and changes no
physics.

**The renderer does not know what the data is** (the owner, 2026-09-28): this is a heterogeneous
data visualizer and analysis tool whose math is fast enough to be a game, and data is put
together like a collage of magazine snippings. So every snipping declares what it is, its frame,
its datum, its unit and its instant, as ATLAS section 9 already asks of a plane, and the collage
is composed from those declarations. A geoid separation is one more snipping, used where a bed
referenced to one datum meets a level referenced to another. No reader converts anything.

### 4.12 What goes away

Candidates, counted by the review. Each is deleted by the step that replaces it, and each step
reports lines added and lines removed beside its gate.

- the Mercator window as a kind of lattice, and the 12 sites that branch on the kind;
- two spellings of the page uv, and the 15 shader sites that compute a window's uv;
- the page rows of four constant buffers, and some twenty literal slice numbers;
- the 25 consumers of the anchor chart;
- the three-tenant paths of `Compose.hlsli`, reachable today only for Mars;
- the page-or-cube switch in the CPU's `HeightPage` and `ExposurePage`;
- ring set A and ring set B as two things;
- the second Mercator path in `Compositor.cpp` that goes around `Lattice`;
- the classifier's shore band and its three comments, now that the shoreline is the water's depth;
- and the water written twice by hand: the kernel and the hull's twin become two executions of
  one description (4.11), so the second is generated and not maintained.

### 4.13 Maps, lenses and the GIS bank

A map in the game is a **view with a lens**, and it reads the tiles the game reads (the owner,
2026-09-28). The globe within the globe already draws the planet twice from one set of tiles; a
mini map is the same thing with a different eye and a different style.

- **The view** is an eye, a viewport and a target, as `views[]` already are in a scene. A map's
  eye is high, so it wants coarse rungs: the cube's own mips and rank 1, which the main view
  mostly holds already.
- **The lens** is what the view draws from the tenants and how: the colour as it is, or the
  weather, or the players, or county lines, or **the residency of the scene's earth**. That last
  one is the first lens to build, because it is the instrument the hierarchy's own steps are
  judged by: rank, window and resident rung, painted per pixel.
- **The GIS is a bank.** The survey is already a page tenant (`gis.landsea`) and the vectors
  already keep every vertex with its level of detail (ATLAS 9a). Boundaries, names and tracks
  join them as layers a lens may ask for. The survey keeps its work in the compose, where it
  gates what is painted; it stops being the renderer's answer to "is this land".

What the structure owes a lens is what it owes every reader: any tenant is reached by the one
walk, and a view says which ranks it wants. What a map needs that the hierarchy does not give is
a second view in one frame. The renderer builds the globe's state once a frame for one eye and
runs the residency turn inside that draw (finding 19); several gates at once need the same work.

### 4.14 The cache, and when it is wrong

Tiles are painted once, kept on disk, packed, and pulled by DirectStorage. A tile on disk is wrong
for one of three reasons (the owner, 2026-09-28): the scene binds the data differently, new data
has been pulled, or the code that paints has changed. One mechanism answers all three: **a
tile's name covers all three**, the source's content, the binding the scene gave it, and the
version of the painting code. A name that no longer matches is never read, so nothing is
invalidated by hand and nothing stale is served.

Less of that is in the names today than this section first said (corrected by the step 4
mapping, 2026-09-28):

- **The code.** `kTileTreeVersion` and the tile's format are in a tree's identity, and
  `kBlendVersion` in a composite's key. `kComposeVersion` is the flat compositor's and is in no
  tree's name.
- **The sources' content.** A colour leaf's identity carries its structure. Google's and the
  orthos' carry no hash of what they hold, and a height leaf's identity is its name and its
  unit: the hash of its edits never reaches it (`HeightStackSource.h:79-112`,
  `DomainSource.h:149`).
- **The bindings.** Not yet, because bindings do not exist yet.
- **What each input holds.** A composite's key records whether each input is present or void
  and not what it holds; a leaf is looked up in an archive with no key at all; and the archive
  is read before the loose file, while a fold reads and writes loose files only (finding 25).
  Measured, byte against byte (`tools/hierarchy/archive_stale.py`): 4,577 of the 194,527
  archived tiles hold bytes their loose file no longer holds, and 1,793 more have lost their
  loose file to a drop and are still in the archive. All of them are in 22 archives of the
  identities in use, which were set aside on 2026-09-28 by renaming them in place
  (`<tag>.gaa.stale`), on the owner's word, until step 4b; the engine reads those trees' loose
  files meanwhile. Pictures taken before 22:48:48Z that day show the cache as it was.

With the names complete an archive and a loose file cannot disagree, which is git's answer to
the same question. Step 4b completes them: a leaf's name carries a token of its bytes, a marker's
token is the hash of its four children's, a composite's key folds its inputs' tokens, and a leaf
is looked up in an archive by its token. It costs one directory search a lookup, which is what
three probes cost today, and no tile's bytes are read to find a name.

Warming a place and packing it are then tools a scene names, run when a place is added.
`--warm-trees` and `--pack-trees` do nothing today (finding 37): their bodies exist and nothing
dispatches them. Step 4a restores them.

**A third tool retires what nothing uses** (the owner, 2026-09-28: build it). The cache keeps
every identity a tree has ever had, a dozen of the colour tree alone, and step 4b strands
today's as well. What makes pruning hard is that some twenty working trees on different
branches share the one cache, so a tree this build cannot name may be one another branch reads
daily, and that the file system keeps no usable time of last reading, so a tree that is fully
painted looks old by its files. The design:

- **A tree says when it was last used.** Every run stamps the frame folders it uses with the
  time, the engine's revision and the scene. The unit is the lattice's folder and not the
  node's, because a lattice that is dropped goes stale inside a node that is still live.
- **The tool lists, and by default does nothing else.** Stamped within the age, kept; stamped
  and older, stale; never stamped, judged by its newest write and given twice the age.
- **Retiring is a rename.** A stale folder and its archive move into a dated folder beside the
  trees, with a manifest and the commands that put them back. Nothing is deleted.
- **Purging deletes retired folders older than a second age,** and is the only path that does.
- **It never follows a link,** acts only on names of a tree's form under a root named `trees`,
  refuses while an engine is running, and asks for the root's own path as its confirmation.

Only the trees are derived: a tree that is gone repaints from the source caches with no fetch.
No source cache is in the tool's reach. Retiring and purging are run by the owner.

### 4.15 Sources, and what each may be asked for

The pyramid accepts a snipping from any source and may have any source replaced (the owner,
2026-09-28). What a source may be asked for is part of its declaration, beside its frame and its
datum: how much a day, and whether what it sends may be kept.

| source | covers | grain | may it be kept? | asked how |
|---|---|---|---|---|
| Google Map Tiles (satellite) | the planet | to z20 and finer | **Not as the engine keeps it.** The policy forbids pre-fetching, storing or caching beyond what the response's cache headers allow, and forbids offline use | 15,000 tiles a day by default quota, 100,000 a month without charge, billed beyond that; billing must be enabled on the project |
| NAIP (USDA) | the contiguous United States | 0.3 to 1 m | yes: public domain | cloud-optimized GeoTIFF. The AWS buckets are requester-pays: whoever asks pays for every request. Microsoft's Planetary Computer holds the same collection, 2010 to now, in Azure and lists it through a STAC catalogue; read from search results, not tried |
| Sentinel-2 cloudless (EOX) | the planet | 10 m | the 2016 mosaic yes, with attribution (CC BY 4.0); later years are non-commercial share-alike | tiles on S3, zoom 7 to 13 |
| MassGIS orthos | Massachusetts | 15 cm | yes | already harvested |
| ETOPO 2022, CUDEM (NOAA) | the planet; the US coast | 1.8 km; 3 m | yes | already harvested for New England |
| NOAA tide stations, EOT20, GFS-Wave, the operational forecast systems | stations; the planet; the planet; one estuary each | | yes | the NODD mirrors on S3 first, NOMADS under its limit |

Two things follow. **The free allowance is counted in tiles, not in bytes.** A satellite tile is
some 20 KB, so 100,000 a month is about 2 GB, and a budget of 5 to 10 GB a day is far more than
Google gives without charge. The engine's cap for Google is therefore a count, 3,000 a day, which
keeps a month inside the allowance whatever the day's quota permits; the 5 to 10 GB a day governs
the open sources. The owner has since set the cap himself (section 5): 100,000 tiles a day and
5 GB a day. In `main` neither is code: the budget is counted per run,
1,000 by default (`Options.h:62`, the scene's `streaming.tileBudget`), and no run knows what an
earlier run of the same day fetched.

**The day's count is built** (2026-09-29, in a scratch tree on `71e3a33`, not committed; no
request has been made with it). One file a UTC day beside the source's cache,
`cache/google/day_<date>.json`, holds what was asked, the tiles that came and their bytes.

- **It counts what is asked.** A request that fails is counted with one that lands, because
  the cap is on what is asked of a paid source. A request may go only while the asked are
  under `streaming.dayTiles` and the bytes under `streaming.dayBytes`. As first built it
  counted the tiles that came, and a source that answered every request with an error would
  have been asked on and on (finding 78).
- **Every engine on the machine counts into the one file,** under one named lock that is
  never held across the network. The count is read again under the lock before it is
  written, so two engines lose nothing of each other's.
- **It fails closed.** A ledger that cannot be read whole is the cap met and not a count of
  zero. A lock that cannot be had refuses the request. A cap of zero is no request, not no
  limit.
- **A run stops asking after eight requests in a row land nothing.** The run's own budget is
  asked before the ledger, so a run that may not fetch takes neither the lock nor the file:
  every gate render is such a run.
- **A body is whole or the request failed,** so a tile cut short is not kept (finding 79).
- **What it does not promise:** a day can close over its cap by the requests in flight when
  the cap is met, one a fetching thread at most, and the bytes by those tiles' sizes.

Its test is in the selftest and opens no connection: the engine's own rule with a stand-in
for the request and one for the clock. Three plants are caught: the count not read again
under the lock loses a fetch; the first rule lets a sixth request go to a source that failed
five; the old log line holds a made-up key but for one character (finding 77). Not run by
anything: a real request; two engine processes at the lock; a body cut short, which needs a
connection. The engine's lines for it: 590 added and 44 removed, beside 614 of test.

**The finest zoom is the scene's.** The Google source held its zoom to 14 by a literal, and
its own comment says why: a deeper zoom is a budget decision that a realization makes inside
a window it owns, and the cube can never demand one. A block that a scene declares is such a
window. At Haulover zoom 14 is 8.6 m a pixel, so a block of rank 3, 1.19 m a texel, would have
been painted from zoom 14 enlarged. The literal becomes `streaming.googleZoom`, 14 unless a
scene says otherwise and 19 at most, and the day's count is what bounds the tiles. A source's
identity is its name and its structure. At 14 both are the strings they were, so no tree
moves; at any other zoom the structure names it, so a scene that raises the zoom paints a
tree of its own and writes nothing finer into a tree that says 14. Tested on the source
alone, with the clamp left at 14 planted and caught.

**The first requests, 2026-09-29, at Haulover** (a scratch tree: `main`, the migration's
form H and the day's count; two scenes that cap themselves at 4,000 requests a day, zoom 17,
1,500 a run). Before any request the same poses were run with the run's budget at zero,
which asks nothing and says what would have been asked: 237, 429, 558 and 302 tiles at four
poses.

- **The first run that fetched asked for every tile twice** (finding 91): the day's count
  read 250 asked and 250 tiles of 4,779,668 bytes, and the cache gained 127 files of
  2,527,268. Loader threads that paint neighbouring tiles each fetched the tile they share.
  Now one thread fetches a tile and the others wait and take what it brought. The next run
  asked 150 and the cache gained 150.
- **A run fetches some 250 tiles at most,** the throttle's 80 ms against the settle's hold
  of 3,000 frames, so a pose is filled over several runs, the coarse zooms first.
- **What the engine draws at Haulover is its own coast.** The survey's mask does not reach
  Florida and the global relief is ETOPO's, 4.9 km a sample. The inlet, the barrier island
  and the bay are open sea to it; its land begins some 3 km west of the true ocean shore,
  and north of a straight line 2.7 km south of the inlet nothing is land at all. South of
  that line the cube's height answers at its mip 1, and north of it at its mip 6, 39 km a
  texel (the residency lens). The line lies at 65/256 of the face, which is also the edge
  of the inlet's rank 3 block. Why nothing finer is asked of the height north of it is not
  read. The imagery shows on the engine's land and lies unseen under its sea. It is
  today's picture of the place and not the blocks': a run with no block declared draws
  the same coast. The height on blocks, with the bed that was harvested there, is what
  mends it (step 7).
- **On the engine's land the blocks draw the imagery:** the mainland from 7 km, the streets
  and the bay's islands, the want set resident and exact, no request refused. From 1.5 km
  the finest block answers at its mip 1, 2.4 m a texel, from zoom 16; from 300 m at its
  mip 0, 1.2 m, from zoom 17, over all of the picture but the tiles' outlines.
- **A picture is taken from the cache, after the run that fetched.** The run that fetched
  from 300 m settled exact and drew one tile sharp and the rest from texels of 9 m, with
  every tile of the finest level mapped. The same pose drawn afterwards from the cache
  alone is sharp throughout. That is finding 3 seen in a picture: while tiles are painted
  and folded the map's bytes stand coarser than the tiles, and the settle judges the
  tiles. The manager of 4.19 computes the map from what is held, and has no such state.
- **The gate's scene with blocks at both places,** seven of the eight rows, draws from the
  cache as the gate did before: the want set of 18,189 tiles resident and exact, the
  window and its corridor in it. Nothing was fetched for it: its window looks at open sea.
  It would have asked 589 tiles, most of them at the Merrimack, where a scene that says
  zoom 17 asks Google for what the cache holds to zoom 14.
- **The day's count at the end of the test:** 798 requests, 798 tiles, 19,368,775 bytes,
  of the 4,000 requests the test's scenes allow themselves and the 100,000 the owner
  allows. 123 of them were the first run's doubled requests.

**The colour is a composite of its sources** (the owner, 2026-09-28), and for
now Google fills its base. NAIP and Sentinel-2 between them cover every place named so far at
rank 2 and, in the United States, at rank 3 and beyond; they enter the same stack as inserts
above the base, or as the base where a scene says so. No source is the floor by construction.

I read Google's terms through a summary of two of its pages, once. They are the owner's account
and the owner's to read.

**The places.** Haulover first: NOAA's station 8723080 stands at Haulover Pier, and NAIP covers
Florida. For the body of water with the most data I recommend **Chesapeake Bay**: one source
counts 38 stations in its PORTS, the largest count I found; it has a three-dimensional
operational forecast (CBOFS: level, current, temperature, salinity); CUDEM and NAIP cover it;
and it is 300 km long, so it exercises ranks 1 to 3 at once. San Francisco Bay is the
alternative, with a forecast system and five current profilers of its own. All three places, the
Merrimack included, lie on face 5 of the cube. The edge that exercises the porch is the one
61 km north of the Merrimack.

**What was harvested, 2026-09-28** (branch `claude/harvest-places`; Python only, nothing
fetched from Google; 460 requests and 7.75 GB of a budget of 2,000 and 8 GB; no existing file
changed, and the three New England windows come out of the new code byte for byte as before):

| | Haulover | Chesapeake |
|---|---|---|
| the bed | the whole box at 13.7 m, 1538 by 1619, -274 to +17 m NAVD88 | the mouth only, 6075 by 3645: 33 of the bay's 78 tiles, the 45 left listed in the manifest with 9.75 GB to go. The whole bay at this grain is 264 million cells, past the loader's 2^26 |
| tides | 14 harmonic stations within 40 km, 8723080 first; 11 tied to NAVD88 | 62 harmonic stations |
| currents | 8 stations, first the one in Bakers Haulover Cut itself, and one profiler on a buoy | 8 stations at the mouth, and the forecast system's surface current as a field |
| the forecast system | none covers Miami | one whole cycle of CBOFS, 384 MB: level, current, temperature, salinity |
| waves | the GFS-Wave point off the inlet, two buoys | the GFS-Wave point off the capes, two buoys |
| not taken | the survey shoreline: Florida's is one file of 247 MB, part of it unreadable | |

Each place has a manifest (`data/places/<place>.json`) naming every file, its source, its
datum and unit as the source declares them, the instant it is valid for, and each source's own
words on whether it may be kept.

**What "a place is data and a scene entry" still costs,** counted by the harvest against the
engine's code. The files are in the formats the engine reads, and four of the scene's `data`
keys can already point at them. The rest names the Merrimack in code:

- **the anchor of the world**, one longitude and latitude with fixed metres a degree
  (`BathyModel.h:25-28`), used by the frame loop, the water's tree, the sea, the sources, the
  wave field and the solver;
- **the two windows**, built by one function for one place (`SurfaceFrame.cpp:31-49`);
- **some twenty literal paths and names**: the bed's file reopened whatever the scene says
  (`Assembly.cpp:485`), the two insets, the currents' file, the water's, the survey's, the
  river's; one tide station, one buoy, one current station and Boston's, each by its number;
- **the solver's compass**: the sea to the east and the river to the west (`Swe.hlsl:102`,
  `:223`);
- **a projection's zone**: 19, where Haulover is in 17 and the Chesapeake in 18 (finding 36);
- **a box the forecast is believed in**: 41.5 to 44.5 N (`WeatherManager.cpp:336-337`).

Step 8 is that list turned into keys of the scene, and its gate stays what it was: the source
tree does not change between the scene without the place and the scene with it.

**The list was a count. The map is `docs/PLACE_KEYS.md`** (2026-09-29, read-only): 69 rows,
each with the file and the line, who reads it, the key that carries it today or the key
proposed, and today's literal as the default. By class: 23 paths and names of data, 7
identifiers of stations, buoys and gauges, 13 coordinates, zones and boxes, 11 assumptions
about geography written as code, 5 windows, 10 defaults. What it changes in the plan:

- **The anchor is the physics' chart, and it is the largest item.** The flat frame every
  solver, kernel and hull works in is degrees times metres a degree frozen at 42.8 N. A bed
  at Haulover placed by it is solved on cells 0.82 of their true width east to west, and
  at the Chesapeake 0.92. So a place declares its chart, an anchor with its own metres a
  degree, and every reader takes the chart from the place. The gates already build a chart
  at another place by the same law (`Gateway.cpp:39-40`). The Merrimack's default carries
  the literal 81,660 and not the cosine, which is 81,654.6: a point 10 km from the anchor
  would move 0.7 m.
- **Two keys exist and are passed by.** The depth tree reopens the Merrimack's bed whatever
  `data.bathy` says, and the velocity-gradient bank reopens the default currents' file
  whatever `data.currents` says.
- **A name is part of an identity.** The layer built from `data.bathy` is named
  `noaa.cudem.merrimack` whatever bed it holds, and the station field `tide.stations.ne`;
  the names reach the trees' folders. A place's data needs the place's name.
- **What is an assumption and not a literal stays an assumption at step 8, and is said.**
  The solver's open sea is the half-plane east of a threshold and its river the first
  column; the boundary clocks are chosen by river kilometre; the entrance jet's axis passes
  through the origin; every solver's bed is the one z14 page. At Haulover and at the
  Chesapeake's mouth the sea is to the east, neither tide file has a river kilometre, so
  the clocks fall to the focus station, and with no station of its name the jet is off.
  The second place runs on those terms. A coast that faces west does not, and the law that
  replaces the compass is a boundary the place declares as a plane of its chart.
- **The zone of a raster falls to 19** for any code the loader does not know, and the codes
  of zones 17 and 18 in NAD83(2011) are among them. The zone is the file's to declare.

### 4.16 Music as a source: the buoy, and the wave machine

The owner, 2026-09-28, as the measure of how abstract the algebra must be: a music file is added
to a scene, objects like pistons and pumps are placed in the water, and the water answers to the
music's bands as an equalizer's bars do. His form of it: **the file is a source of the water's
physics as a buoy is**, and the renderer shows the object in a practical way, moved either by
the water or by the music. Adding the file and the objects is an edit to the scene. It is not
scheduled. It is written here because the structure must not have to change when it comes.

**The music is a buoy.** A buoy reports the sea's energy band by band: NOAA's 44013 sends 46
bands from 0.033 to 0.485 Hz, in square metres per hertz, and 44098 sends 98 from 0.025 to
0.96 Hz (counted in `cache/ndbc`). An equalizer's bars are the same thing, energy band by band.
So the file is declared as a point source at a place, its bands bound to the sea's bands, and
the sea near it takes that energy with an authority that falls with distance, as it would from
a buoy. The waves are the sea's own, 2 m to 2 km long and seen from the air. Nothing is added to
the physics. The engine is most of the way there, and the two things it lacks it lacks for real
buoys too:

- The sea takes one number from a buoy. 44013's height over the forecast's is one gain on every
  partition (`SeaLayer.cpp:217-228`); its measured bands are loaded and drawn in the spectrum
  plot and shape nothing (`SeaLayer.cpp:425-453`). The sea must accept a spectrum given as
  bands, beside the two shapes it fits today.
- The sea's state is one point for the whole scene (`SeaState.h`). As a tenant on the pyramid
  it has a global floor and a point's authority near the point (4.11), which is what makes a
  music buoy local. The point kind exists and was written for this: `PointSource` in
  `DomainSource.h`, inverse distance within a radius, "so a buoy can join a raster in one
  product".

**What the scene binds**, each a declaration and none of them code:

| binding | what it says | already in the design as |
|---|---|---|
| the clock | where the file stands on the scene's time axis, and at what rate | 4.11, time |
| the bands | which band of the file feeds which band of the sea | new: a table of pairs |
| the gain | a level has no unit and the sea's band is m^2/Hz; the property table refuses units that do not close | 4.11, value |
| the smoothing | the attack and release of each band | the filter's own parameters |
| the direction | a file has none, so the scene gives one: a bearing, or outward from the place | a buoy's direction, declared |

What turns samples into bands is a filter bank, declared in the scene, and what it makes is data
like any other: kept on disk under a name made of the file's content, the filter's parameters
and the code's version (4.14).

**The object is display.** It is an entity (4.9) with its own mesh. Its pose is bound to the
water at its place, and it floats as a hull does; or to a band, and it strokes as a bar of the
equalizer. Bound to a band it changes no physics: it is a lens's business (4.13), like the
relief's exaggeration. The spectrum plot the engine already draws beside the buoy's bars is the
same display in two dimensions.

**The second form: the object drives the water.** Where rings are wanted that leave the object
and cross the water, the object is a source and not a display, and it acts through one term, the
volume it displaces. Write `beta` for the depth of body beneath the surface in the water column
at a place. The water's continuity carries its rate:

```
d(eta)/dt + div(flux) = - d(beta)/dt
```

A hull under way carries `beta` along its track, and the term is its wake. A piston's face
carries `beta` into the columns it enters. A pump is the same term given as a flux. A pier is a
`beta` that does not change and drives nothing. The renderer is told none of this: it is fed the
water (4.11). Today's wake cannot express a piston: it is a closed form for a hull on a straight
track at a constant speed, eight floats a boat (ALGEBRA `wake`), and it is this term's steady
limit.

In the algebra the term is the rate of a join. A body is a set of cells (4.10). A cell of four
points has the volume `(P0 v P1 v P2 v P3) / 6`, and the part of it beneath the water's plane is
cells again. A rigid body moves by one twist `B`, a bivector; a face plane `p` then moves at the
rate `p x B`, the commutator, which is a plane; and how fast that face closes on a point `X` of
the water is that plane evaluated at the point, `(p x B) v X`. It is the address of 4.4 once
more, a plane evaluated at a point, with the plane's rate in the plane's place. A cage that
deforms has a velocity at each point and the rate is the join's derivative by the product rule.
Nothing in the term asks what the body is. The hull's hydrostatics already clip its sections at
the water's plane to find the volume it displaces (`Vessel.cpp`, `ClipSectionArea`); the rate of
that volume is what is not yet given to the water. This algebra is design: it is pinned in
`gatest` when it is built, its signs included.

For a source that stays where it is, linear theory makes each wavenumber `k` of the surface an
oscillator at the water's own frequency `w(k)`, `w^2 = g k tanh(k h)`, driven by the source:

```
eta_k(t) = - s_k * integral from 0 to t of  d'(tau) cos( w(k) (t - tau) )  dtau
```

with `s_k` the source's shape and `d(t)` its stroke. The integral is the running Fourier
transform of the stroke's rate, taken at the frequency the water gives that wavelength: the pond
is a spectrum analyser, and dispersion sorts the bands in space. Far from the source a narrow
band's waves are the band's envelope read at the retarded instant, `E(t - x / cg)`, on a
carrier. Emulated in one dimension (`tools/hierarchy/wave_machine.py`): against the full
integral over every wavenumber and the whole history, that closed form is within 8.3 % of its
peak from 6 to 25 m, for a band whose level swells once in 20 s; what it leaves out is the
band's own spreading. Two dimensions add a spreading by the square root of the distance and are
not measured. Whether one stationary-phase form covers a source that also moves along a curve
is not worked out here; the route that needs no closed form is the term given to the solved
field, at the rank whose texels resolve the wave.

**The two forms, side by side.**

| | the file as a buoy | the object as a source |
|---|---|---|
| what the water does | the sea near the place takes the music's energy, band by band | rings leave the object and travel at the group speed |
| the waves | the sea's own bands: 2 m to 2 km, seen from the air | what the stroke's rate makes: see below |
| new physics | none | the displaced-volume term |
| new in the sea | a spectrum given as bands; the state as a tenant | the same term serves every hull's wake |
| the ranks it needs | those the sea already has | the two finest, for waves in time with a beat |

One scene may hold both, bound to the same file.

**Either way the water has no state of its own from the music.** A file is known from its first
sample to its last, so the sea it makes is a function of the scene and the instant. Two machines
agree on it and so does one that joins late, which is the standing the swell and the steady wake
already have. A hull that a player steers has no known future; its wake is state, advanced on
the fixed-step clock (4.11).

**What the physics allows.** In the buoy's form a band of the sea cannot change faster than its
own period: a swell of 10 s that doubled with every beat would not be a swell of 10 s. So there
are three honest choices and the scene makes one. The band follows the music's slow contour,
smoothed over several of its own periods: physical. The band follows the beat: the hull and the
picture still read one water, so they agree, but energy appears everywhere at once and no sea
does that. Or the file's clock is slowed until its beat is a swell's period, while the speakers
play it at full speed.

In the second form the stroke's rate sets the wave (`wave_machine.py`, deep water):

| the beat | the wave's length | its group speed | it breaks at a height of | the rank with 8 texels to a wave |
|---|---|---|---|---|
| 0.25 Hz | 25.0 m | 3.12 m/s | 3.6 m | 3 (1.19 m) |
| 0.5 Hz | 6.24 m | 1.56 m/s | 0.89 m | 4 (15 cm) |
| 1 Hz, 60 to the minute | 1.56 m | 0.78 m/s | 0.22 m | 4 |
| 2 Hz, 120 to the minute | 0.39 m | 0.39 m/s | 5.6 cm | 5 (1.9 cm) |
| 4 Hz | 0.10 m | 0.21 m/s | 1.4 cm | none: rank 5 has 5 |

Waves in time with a beat are chop: decimetres long, centimetres high, crossing ten metres in
half a minute, seen from the helm. They are the micro scale's first real use.

**Hazards, named before they bite.**

- An MP3 does not decode to the same samples in every decoder: the standard bounds the
  difference and does not forbid it (documented). The record (4.11) therefore reads the kept
  table of bands and never the file, and the table is what two machines share. A lossless file
  decodes to the same samples everywhere.
- A sound card has a clock of its own. What is heard and what is simulated are two readers of
  one producer. How the heard sound is held to the scene's clock is not designed here.
- A wall reflects. Reflection is a boundary of the water and belongs to the solver's cells, not
  to a source.
- The displaced-volume term is a source on the surface. It gives a piston in shallow water its
  right wave, `H / S = k h`, and cannot give the deep limit, `H / S = 2`, which comes from the
  motion's shape in depth. Where that matters the body declares its ratio, as a source declares
  its datum.

**The gates, when it is built.** For the buoy's form: the sea synthesised from a buoy's own
bands, measured back at the buoy's place, gives the buoy's bands; then the same with a file in
the buoy's place. For the second form: a piston in water of depth `h` driven at one frequency,
the wave's height far from it over the stroke, against wavemaker theory's ratio,
`H / S = 2 (cosh 2kh - 1) / (sinh 2kh + 2kh)`, whose two limits the harness checks. For both,
step 8's gate again: between the scene without the music and the scene with it,
`git diff --stat src shaders` is empty.

**Prior art.** Nothing in this subsection is a new technique. Taking a measured spectrum into a
synthesised sea is what every spectral ocean does with the spectrum it is given. Wavemaker
theory is Havelock's, 1929. In graphics, sources and obstacles on a dispersive height field are
iWave (Tessendorf 2004); packets of waves that travel at the group speed are Jeschke and Wojtan
2017; a closed form for the ripples of a point source, and sources fitted to boundaries, are
Schreck, Hafner and Wojtan 2019; dispersion inside a shallow-water solver is Jeschke and Wojtan
2023. Water driven by music is a fountain show in the world and a common visualiser on a
screen. What the engine would have is the combination: the source is bound in a scene with its
units checked, and the water it makes is the water the hull reads.

### 4.17 What the code said of 4.1 to 4.8

A second read-only mapping (2026-09-28) walked the residency manager, the wants, and every
stage's way to a page, and put the design beside them. Where it corrected a section, the
section now says so. What it changed in the plan:

**A window that stands on an aligned block needs no new manager.** A window whose origin is a
multiple of 16384 texels of its rung is one block of the cube's lattice at that rung: all of its
mips line up, its uv stays inside [0, 1), and today's clamp samplers, today's residency map and
today's per-slice manager are right for it. It is the Mercator page's shape on the other
lattice. So the order of section 6 is interleaved: the shader's contract first, on standing
blocks, where a picture tests the pyramid, the address and the directory; then the manager;
then windows that move. Its price is that a place near a block's edge needs two blocks of a
rank where a following window would need one. Measured by the first commit's selftest: the
Merrimack's mouth lies in block (166, 4) of rung 9 on face 5, 1.40 km from its west edge and
1.76 km from its south edge, so a viewer there needs four blocks of that rung within 8 km.
This section had said 2.3 km, by an arithmetic nobody had run.

**What a moving window adds, and the code does not have:**

| what | where it bites |
|---|---|
| slots addressed modulo the window | a tile's identity is its slot today (`StampIndex`), and its key holds 21 bits of x and y |
| a floor that is not the slice's coarsest mip | the parent rule, the boot map and the hold all say `mips - 1` |
| a want across the modulo seam | `Want` takes one rectangle in a slice's uv, clamped to [0, 0.9999] |
| a landing checked by address | a DirectStorage copy lands on the slot captured when it was queued, and ownership is checked by slot |
| the turn at the head of the frame | it runs inside the globe's draw, after the sea and the banks have recorded their reads |
| a map upload that cannot be deferred | a map that finds no room stays dirty a turn, and after an eviction in that turn it points at zeros |
| an anisotropic sampler that wraps | the engine has none; step 0 measured the wrap with a trilinear one |
| origins that are a function of the pose | or the settled stills stop being exact; their hash must cover global tiles and origins |
| the capacity declared | slices are fixed when the array is made, and a residency sampler exists per gate and per subject, of sixteen |

**What the code forces a decision on:**

- **The solver's bed** (finding 48). The solver reads its bed on the GPU, choosing page or cube
  and the mip by what is resident, and the hull's level and current are the solver's. Physics
  reads residency there today. Either the solver stands on a pinned window and is held until
  that window is whole, or it keeps a bed of its own. The first is the design's: a solver's
  domain is a standing window (4.1).

  **Measured, 2026-09-29** (branch `claude/solver-bed-resident`; three arms, fifteen runs of
  5,400 frames, the kernel's own reading of the bed read back over the domain). Today the
  hour of spin-up integrates on a bed of exactly 0.0 m in every one of the domain's
  2,187,162 cells. The residency map's copy on the GPU is born zeroed, zero says that mip 0
  is here, and the residency turn that first writes it comes after the spin-up (finding 66);
  so the kernel reads mip-0 tiles that nothing has mapped, and they read zero. The tide an
  hour before the start stands at -0.367 m, under that plain: the basin is dry for the hour,
  and at frame 600 its level stands 0.552 m under the tide, which is the tide's rise over
  the hour to 5 mm. The minutes after it are the refill through the real inlet, and that is
  the beach band.

  | the bed the hour ran on | the level less the tide, frame 600 | frame 5400 |
  |---|---|---|
  | today's: 0.0 m in every cell | -0.552 m at the beach, -0.552 at the helm | -0.116, -0.251 |
  | the whole bed, waited for | -0.028, -0.055 | -0.006, -0.032 |
  | the coarsest mip, the map written and nothing asked (beach only) | -0.002 | -0.008 |

  Three runs an arm at each pose, and no spread within an arm at frame 600. The wait is 334
  tiles, 16 residency turns and a tenth of a second, where the time to a first frame varies
  by 7 to 14 s from run to run. Started on the coarsest bed the level is right at once, and
  the water sloshes by 0.14 m a minute later, when the fine tiles land under it. So the
  conjecture's mechanism, a coarse bed that dams the inlet, is refuted: the inlet is open on
  the coarsest bed. Its class is confirmed: the solver integrated on what residency had not
  delivered. And in four of today's six runs the pinned domain never became whole in 5,400
  frames (finding 67), so in play too the bed is what was mapped.

  The first form is taken, and stated as a law: **a solver integrates on a window that
  stands whole, from its first step.** The window is asked for before the solver starts; the
  solver starts when it is whole; its tiles are held while the solver runs; and a tile of it
  that is absent is an error that is said, not a coarser answer that is read. What the
  experiment built is the first half of that in today's engine, behind a scene key,
  `water.swe.bedWait`, whose default is today's behaviour.

  **And it stands where its sources paint at full weight.** The experiment's bed, read
  back, shows a second fault of the same kind, in space where the first was in time
  (finding 68). The solver's grid is the survey's window, cell for cell, and the survey's
  layer fades into the relief beneath it over the outer 4 % of that window. At the
  solver's west column the bed is therefore the 15-arcsecond relief, +2.03 m at its
  lowest, where the survey has the river at -4.21 m; the channel comes back to its
  surveyed depth 74 columns in. No cell of the west boundary can be wet, so the
  Merrimack's discharge and its tide have never entered since the bed became the composed
  height. A fade at a layer's edge is the picture's device. A solver's edge is a
  boundary condition, and it needs the data: its window is declared inside the band, or
  the source that carries its boundary is given no band on that side.

  **Both were built and gated, 2026-09-29, and one of them is not ready** (the keys
  `water.swe.bedWait` and `water.swe.window`; 64 runs; not committed). The owner had said
  yes to both together. What the gate measured:

  | | today's settings | whole bed, today's window | whole bed, the window drawn in |
  |---|---|---|---|
  | cells of the west boundary that can be wet | 0 | 0 | 18 |
  | the level less the tide at frame 600, beach and helm | -0.552, -0.552 m | | -0.030, -0.058 |
  | at frame 5400 | -0.116, -0.251 | | -0.005, -0.032 |
  | through the west boundary over one tide, m3/s | | 0.0 | +16.1, where it is told +17.8 |
  | the same, hour by hour | | | against what it is told: correlation -0.64 |
  | the gap's current against the station's, gain | | 0.68 | 0.51 |

  - **The whole bed is a plain gain.** The low basin of the startup is gone, the wait is
    252 tiles and a tenth of a second, and the new arm's two runs are one picture at all
    six poses, where today's differ from each other at four.
  - **The window drawn in lets the river enter and is not right yet.** Over one tide the
    boundary carries what it is told to within 1.7 m3/s. Hour by hour it does not: the flow
    it carries goes with the tide's level (correlation +0.95) and the flow it is told goes
    with the tide's rate, as a prism's does. At the flood it brings water into the domain
    from the west where it is told to take it upriver. And the gap's current fell against
    the station's, where opening the boundary to the prism upriver should have raised it.
    That boundary had no wet cell since the bed became the composed height, so its law had
    never carried water.
  - **The boundary's line is right, and the reach behind it is late.** Its terms were
    traced cell by cell at four instants: the pinned column holds the target exactly, the
    signs and the datums are right, the clamp never bites, and the flux it carries is the
    sum of its two terms to 0.4 m3/s. But the water next to it is not the water the law
    supposes. The reach from the first interior column to the 25th stands flat, 0.69 m
    under the exterior at the flood and 0.6 m over it at the ebb: the model's river
    delivers the tide there 2.70 h after the ocean's, where the stations say 1.07 h at
    that kilometre and the model itself is 0.17 h late 10 km downriver. So the radiation
    term, which is there to let reflections out, feeds and drains the reach, and the face
    is a second mouth.
  - **The river loses no time along a channel: the window cuts its bend.** The suspect I
    named, the damping a step on every face's flux, is cleared: its rate is 1.9e-4 a
    second, of the order of the tide's frequency and not far above it, and four cycles
    from the shipped damping to none move the lag at the edge by 0.006 h. The cause is the
    grid. The river leaves the window through its north edge at 9.5 km west of the anchor
    and comes back through it 0.8 km further east, and the edge is a wall. Inside the
    grid the lowest way from the upper reach to the sea crosses land at +7.27 m. So the
    reach is a pond of 2.64 km2 whose only mouth, once the window is drawn in, is the
    west face; with today's window it has none and stands still, its range 5 mm. Below
    the bend the model is right: 0.18 h late 5 km up, where Newburyport is 0.25 h.
  - **The law it asks for: wherever water meets the window's edge the edge is an open
    boundary, and carries the data of what lies beyond it.** The north edge's two
    crossings are boundaries as the west face is. That replaces the solver's compass,
    the sea to the east and the river in the first column, which 4.15 already names as
    an assumption a second place cannot keep. It is step 8's. And an instrument, now:
    at its start a solver floods its grid from its open boundaries and says what water
    is joined to none of them.
  - **So the first law goes forward as the default and the second does not yet.** The
    window's key stays, its default today's window, until the flow the boundary carries
    goes with the flow it is told and the gap's current does not fall.
  - **Closed since:** the new binary with both keys at today's values draws today's
    pictures. At the ebb `main` itself gives three pictures up to 13,666 pixels apart, and
    runs of the new binary are byte for byte runs of `main`. The frames of 52 to 71 ms on
    the storm rail are not the wait: two are a residency turn (34 ms, 29 of it releasing
    and mapping; 40 ms, 36 of it opening files for DirectStorage) and two have no phase
    that accounts for them.
- **The height and the exposure move together.** The bank's kernel and the sea read both through
  one slice and one row.
- **The colour and the mask move together.** They share lattices, a tree, rows and wants.
- **Rank 1 cannot be skipped.** The directory names the next rank, and a cell of a face's
  directory is four times a rank-2 window: without a window of rank 1 the walk cannot reach
  rank 2.
- **The apron at a cube's edge.** Blocks of one rank that tile a face do not overlap, so a filter
  at their seam has no neighbour to read, as a Mercator page has none at its edge today; and at
  the face's own edge the neighbour lies in another plane. A porch needs a margin that both
  windows hold. On one face a following window brings it. Across an edge it means tiles past
  the face's square, which the tree cannot name today. Section 4.18 names them.
- **The address needs the ground, not the wave.** The point a pixel is addressed by must be the
  undisplaced one; the interpolated position carries the wave's sideways displacement. Step 1a
  adds that interpolant for the water's own sample point, and the address reuses it.

**The migration, for the colour and the mask at the Merrimack,** with the old path and the new
chosen by one scene key until the last commit, so that every commit is compared by picture at
one pose:

| commit | adds | gate |
|---|---|---|
| 1 | a slice's binding to one aligned block of a finer cube lattice: slot to global tile and back, and a tree's changes routed to the slot. Nothing reachable | the round trip at every mip; the default run's hashes unchanged |
| 2 | the key; slices 6 and beyond as blocks of ranks 2 and 3; appended rows; the colour and the mask read by `PageTexelUv` with today's choice between page and cube; the wants in the rung's grid | stills at four poses by SSIM and by eye, the albedo lens; every other tenant's hashes equal across the two arms; no fetch |
| 3 | the eye-relative point: the planes about the tangent origin, the undisplaced point | step 3's gate at the pixel stage |
| 4 | the directory and rank 1, in C++ and in HLSL | the two equal on random ground through a readback |
| 5 | the old path deleted, and the key | the key-on bytes reproduced; more lines removed than added |

**Commits 1 and 2 are made and gated, 2026-09-29**, not committed, in a scratch tree that holds
the six pieces of section 6 beneath them.

- **Commit 1.** A block's binding: +1,394 and -31, of which the engine's are +331. Every
  tenant's hashes are the unmodified binary's at the six poses; the helm and the globe are
  the same picture bit for bit, and the others differ by no more than two unmodified runs
  differ from each other, the bird within that spread and not under it.
- **Commit 2.** One key, `streaming.faceWindows`, a list of points each with a rung: the
  engine takes the block of that rung that holds the point and says which. +306 and -17.
  With the key empty the hashes are the old binary's at the six poses. With it set, the
  height's, the exposure's and the wave's hashes are equal across the two arms, and the
  picture against the Mercator windows:

  | pose | SSIM | pixels that differ | floor, key off and key on |
  |---|---|---|---|
  | helm | 0.9957 | 6,784 | 0 and 0 |
  | helm at the ebb | 0.9968 | 4,424 | 411 and 255 |
  | bird | 0.9693 | 259,623 | 0 and 0 |
  | 7 km | 0.9890 | 28,446 | 0 and 0 |

- **The two lattices hold the same ground.** Read from the trees' own files, each texel of a
  pyramid tile against the Mercator pixel that holds its centre, the colour's means agree to
  0.01 of a step of 255 at rung 9 over 466 tiles, to 0.15 at rungs 8 and 7, to 0.3 at rung 6,
  with the contrast inside a tile equal to a part in a hundred; the mask's agree to 0.02. So
  the pyramid is painted right, and the pictures' difference is not in what the tiles hold.
- **The colour is equal, and the finished picture is not.** In the albedo lens, which shows
  the colour as it is read and nothing else, the two arms have one tone over the land of the
  bird's frame, the mean colour equal to a part in ten thousand, and one detail, the rms of
  luminance less its local mean 0.0336 with the key set and 0.0333 without. In the finished
  picture the same land is lighter with the key set, its mean luminance 0.476 against 0.464
  and more than 0.02 apart over 28 % of it, and it has 0.80 to 0.85 of the detail. The water
  is the same in both. So the difference is made after the colour is read, in what the
  land is mixed with: the mask's reads, or the mesh stage's.
- **Found, mended and proven, the same day.** Two lenses were made that paint what the pixel
  stage mixes by. Landness, the edit mask and the close-up material's weight are equal in
  the two arms. The imagery's share inside the close-up material is not: 0.337 with the key
  empty and 0.001 with it set. The function that says how fine the colour's texel is at a
  pixel knew the Mercator window and not the blocks, so with the key set it answered the
  cube's 611 m, and within 2.7 km of the eye the close-up material painted its own sand in
  the photo's place. It was a gap of commit 2's and not a property of the lattice. With the
  blocks in that one function, over the land of the bird's frame:

  | | tone less the Mercator arm's | tone more than 0.02 apart | detail, rms less the mean over 3, 5, 9 pixels | neighbours' mean difference |
  |---|---|---|---|---|
  | before | +0.0118 | 27.8 % | 0.74, 0.83, 0.87 | 0.85 |
  | after | -0.0009 | 1.9 % | 0.88, 1.00, 1.03 | 1.02 |

  At five pixels and more the two pictures have one detail, and by the mean difference of
  neighbouring pixels they have one at every scale. At the single pixel the rms is 0.88 of
  the Mercator arm's, and part of that is the Mercator arm's own: a quarter of its
  vertical neighbours are the same to the step where a fifth of the pyramid's are, the
  staircase of an address that is two texels wrong at that zoom. How much of it is that
  is not measured. With the key empty every hash is the old binary's, at the six poses.
- **Commit 3 is made and gated:** the address is the undisplaced ground point about the eye,
  the planes taken every frame in doubles, eight rows, the fourth fine block named. Read
  back from the pixel stage the address is within 0.0016 of a texel at rung 9 and 0.0034
  at rung 6, and the direction in float32, planted, is 0.35 and 0.045 off and caught. Key
  off, every hash is the old binary's. Several stages have a direction only and keep its
  grain: the pixel water's bed, the sea and terrain layers, the lens, and any level but the
  camera's own.
- **Two things this section said were wrong, and are withdrawn.** That the land was not
  lighter with the key set: the tone had been measured in the albedo lens and written of the
  picture. The agent's eye and mine had seen the picture rightly. And that the address was
  the suspected cause of the lost detail: commit 3 made the address exact and the picture's
  detail did not move, 0.803 of the Mercator arm's at the bird where it had been 0.800.
- **Commit 4 is made and gated, and two things of it are owed.** Rank 1 is in the key, a grid
  of sixteen by sixteen cells stands beside every slice, and the walk is one function with a
  body in C++ and a body in HLSL: on 15,000 points the two return the same chain, slice for
  slice, the address within 0.0033 of a texel, and a directory with one cell planted wrong
  is caught at every point of that cell. Read back from the GPU the directory is as built in
  all 3,072 cells, 24 of them naming a block. A key whose block has no parent declared is
  refused whole and said, and the Mercator windows stand.

  **The default path is today's to the byte.** The blocks' code is compiled only when the key
  is not empty. With the key empty all 68 stages compile to the program they compiled to
  before the migration began, the pictures are the old binary's to the pixel where two runs
  of one binary are, and the globe's pass costs what it cost. The commit's first form had
  not held that: every reader walked for itself into an array, and the default path paid
  4.9 ms at the helm for code it did not run.

  | the globe's pass, ms | today | key empty | the ladder of commit 3 | the directory |
  |---|---|---|---|---|
  | helm | 12.90 | 12.92 | 13.97 | 13.95 |
  | bird | 6.13 | 6.13 to 6.31 | 6.27 | 7.26 |

  With the key set the directory draws the ladder's picture, SSIM 0.9999 and better at the
  four poses, but for sixteen pixels that are the same in both runs. Owed: the key set costs
  a millisecond more than today's path, because the colour is sampled at every rank that
  wins and only the last sample is used; the rank is to be decided from the residency's
  levels and the winner sampled once. And the sixteen pixels are the pixel water's bed,
  read with its pixel's chain where the bed point has left that block; a ray that leaves
  the surface walks for itself (4.5). Rank 1 answered nowhere in the four frames.
- **Commit 4's second form: one sample a read, and the bed walks for itself.** The winner
  is decided from the residency's levels and sampled once; the pixel water's bed walks the
  directory from where it lands. With the key empty the 68 stages are still the old
  binary's. With it set, three poses are within their floors and the helm's two bed pixels
  are the ladder's again. Two things are owed still. At the 7 km pose some 25 pixels of the
  horizon changed by up to 52 levels: the winner is sampled inside a branch, and where the
  pixels of one group of four choose different ranks the sampler's footprint is taken from
  neighbours that did not run the sample, which the language leaves undefined; the old
  picture leaned on the same thing. The footprint is to be taken before the branch. And the
  key set still costs a millisecond at the helm:

  | the globe's pass, ms | today | the ladder | the directory | one sample a read |
  |---|---|---|---|---|
  | helm | 12.88 | 13.97 | 13.93 | 13.88 |
  | bird | 6.15 | 6.29 | 7.27 | 6.95 |

  One sample in the place of three bought back 0.05 ms at the helm, so the millisecond is
  not the colour's samples. It is inside the globe's mesh draw, and the ladder pays it too:
  it belongs to reading blocks at all, and where in that is being measured by taking things
  away. Counted from the code, a land pixel with the key set makes five residency gathers,
  three directory loads and one colour sample, where today's makes three gathers and three
  samples. The binding of 4.5, a window's index carried in the mesh's record, is what takes
  the directory's loads away from a rasterized pixel, and it is not built.
- **Two gates of this migration were lost to one fault of the harness,** and one the night
  before in step 2: PowerShell's names ignore case, so a key held in `$K` was overwritten by
  a loop's `$k`, and thirty runs meant to have the key set ran without it. The rule since: a
  batch's first run has its own argument line read before the rest are let go.
- **What the key-on runs painted:** 22,814 files and 1.09 GiB in nine trees, from the source
  caches, the fetch budget at zero. They refused 35 source tiles a run at the helm and 16 at
  the bird. A tile painted while a source was refused is not kept: the paint answers that
  it is not complete and the tree stores nothing (`TileTree.h`, read).
- **Owed by commit 2:** its rows hold four blocks, and the mouth stands by a corner of its
  block, so one fine block of the four a viewer there needs is not declared.

### 4.18 The apron: a window past its face's edge

Design, for step 9. It answers the point 4.17 left open.

**What is needed.** Near an edge of the cube a filter's footprint crosses the edge, and the
porch's blend runs across it. Both want the window on each side to hold texels on BOTH sides of
the edge, in its own plane. A face's plane does not stop at the face's square: the central
projection carries on, and `ComposeCubeDir` already takes a coordinate past 1. The shader needs
nothing new either: `PageTexel` is the address in the plane, and a texel past the edge is one
more texel of the window, found by the same modulo. What has no name is the TILE, because a
tile's address is unsigned and the face's tiles start at zero.

**The answer: an apron is a face of its own.** Beside the six faces the pyramid has their
aprons: for each face, four strips past its edges and four squares past its corners, 48 in all.
An apron's tiles count from the edge outward, so their addresses are unsigned like every other,
and an apron has parents and children of its own by the same halving. Its texel centres are the
face's own coordinates carried past the edge: for the strip past the edge `s = 1` of a face
with `N` texels a side, texel `x` stands at `s = 1 + (x + 1/2)(2 / N)`. A window that reaches
past the edge maps the face's tiles where `X < N` and the apron's where it is not; which it is
is decided where the manager names a tile to the tree, and nowhere else. The shader cannot tell
an apron's tile from a face's.

**How wide.** A window answers for a cell only where the cell lies a porch inside the window's
edge (4.6), and the blend's band is a few hundred texels. So an apron is some sixteen tiles
deep at a window's finest rung, two at its floor, and exists only where a window has stood by
an edge. Rank 0 has none: its faces are read through the cube's own views, and the hardware
filters across them.

**The weight is one more ratio of planes.** The plane through the body's centre that holds the
edge between two faces has the normal `m = (nA - nB) / sqrt 2`, and the two faces' bisector is
`nA + nB`. The weight is a smooth step of `(P . m) / (P . (nA + nB))`, which is zero on the
edge, has the same form seen from either face, and counts the distance from the edge evenly in
the texels of both. The engine blends across cube edges this way already for the cascade sea
(`WaveChart`: a partition of unity over a band, with its continuity gated), so this is that
law given to textures, not a new one.

**What it costs.** Ground within an apron's depth of an edge is painted twice where it was
painted once, in its own face and in the apron of the face across the edge; by a corner, where
three faces meet, up to five times. Sixteen tiles are 2.4 km at rank 3's finest rung and 19 km
at rank 2's. The tree's keys must hold 54 faces where they hold 8: the archive's key gives the
face three bits today. Step 4b changes the keys anyway, and gives the face six.

**What was considered and not taken.**

- *A signed address.* It breaks the halving that finds a parent, which is a shift, at every
  site that computes one.
- *No apron: near an edge, fall to rank 0,* which the hardware filters across faces. Every
  place on an edge of the cube would then be drawn at 611 m, and an edge runs 61 km north of
  the Merrimack.
- *Charts of their own over the edges and the corners,* twenty more planes, blended inside the
  faces. No apron, and the address would not change, since any plane through the centre gives
  a ratio of planes. But it is twenty more lattices, each with a pyramid of its own, where the
  design has one.

**Its gate** is step 9's, as written: a rail across 43.364 N, and a lens that paints the weight.
And one the apron adds: a texel of an apron and the texel of the face across the edge that
covers the same ground are painted from the same sources, so they agree within the containment
of step 4d.

### 4.19 The manager that replaces today's

Proposed, 2026-09-29, from what steps 1 and 2 and the solver-bed experiment measured. Nothing of
it is built. It is written as laws because every fault seen in today's manager is an order of
events that a ledger kept by increments did not foresee, and a law has no order of events.

**Two sets, and one order.** What is WANTED is said by the readers every turn: a view, a gate's
carried eye, a kernel's domain, a prediction. What is HELD is the tiles that have a slot and
whose bytes have landed. The manager has one order over tiles and nothing else to decide with:

1. a tile of a standing window (a pin) before any other;
2. then by how lately it was wanted: this turn, within the glance (60 turns today), longer ago;
3. then the coarser ground before the finer, by rung, across slices and tenants;
4. then by the reader's own weight, the nearer to what it looks at the sooner.

**The held set is the first P of that order,** P being the pool. The loader reads the first
tiles of the order that are not held. The evictor releases the last tiles held that are not
among the first P. They are the two ends of one comparison, and the settled state is a function
of the wants alone.

What follows from it, each against what was seen:

| the law | what it replaces | the finding it answers |
|---|---|---|
| **The map is a function of what is held** (4.7): the finest level held with every coarser one, and then the margin (4.6), written for the cells under the tiles that changed | the byte kept by increments | 3, 63, 64: no order of maps and unmaps can leave it wrong |
| **A want is closed upward,** as it is today, and with the order that makes the held set closed upward: a parent was wanted at least as lately as its child and is the coarser, so it comes before its child into the pool and leaves after it | the ring gate, the gather's test of the parent, the evictor's test of the children | 63, 64: each tested one level, at one moment |
| **A tile that lands out of turn waits in the map, not in a queue.** A child whose bytes arrive before its parent's is held, and the map names the coarser level until the parent lands | the gate that holds a request behind its parent | 24 |
| **Nothing is read from disk that no slot will take.** The loader takes only tiles among the first P; a tile that has left the first P by the time it lands is let go in that turn | the batch, its lost tail, the tiles Loaded and in no queue | 2: 7,917 tiles on the storm rail; 67 |
| **When the want is larger than the pool, what is lost is the want's own tail:** the finest rung's tiles farthest from what their reader looks at | the landing order | priors 30: at the bird pose 9,176 tiles are wanted of 8,192, and which 984 lose differs run to run |
| **A tile that is repainted keeps its slot.** The new bytes are read and copied over the old in place; the tile is held throughout and the map does not change | the drop, the raised bytes, the second map in a new slot | 3: and the soft flash over every tile that a fold rewrites |
| **Release is two steps.** The map stops naming a tile in the turn that decides it; its slot is unmapped and free once no frame in flight can read it | the NULL mapping made in the turn of the eviction | 34 |
| **The turn is at the head of the frame,** before any reader records a read | a turn inside the globe's draw, after the sea and the banks have recorded theirs | 34; 4.17's table |
| **A map is born saying that nothing is here,** on the GPU as on the CPU | a map born zeroed, which says that mip 0 is here | 66: the solver's hour on a bed of 0.0 m |
| **A pin is whole before its reader starts,** and the reader asks: `Whole(pin)` | a pin made inside the frame loop, after the solver has spun up | 48, 67 |

**What others have done.** Coarse before fine on the way in and fine before coarse on the
way out is the common practice: Intel's sample queues a region's tiles from the coarsest
level up and releases them from the finest down, by reference counts, never evicts the
packed mips, and delays an eviction by the frames in flight, rescuing a tile that is wanted
again before the delay ends. Virtual texturing at large keeps a cache by least recent use
with the coarsest pages pinned. What this section adds to the practice is small and I do
not claim it is new: that the two ends are one comparison, so that a want larger than the
pool loses a tail that the want itself names, and that the map is computed from what is
held. I did not find either stated; two searches, not a survey.

**What it costs.** One selection a turn, of the first P among the tiles wanted within the
glance, some twenty thousand: a pass over buckets of (class, lateness, rung) and a partial
order by weight inside the one bucket that straddles P. A weight for every wanted tile, which
the want walk has in hand: it knows the reader's centre and the tile. A reserve of slots equal
to a turn's maps times the frames in flight, 384 of 8,192, because a released slot is not free
at once. And a repaint in place needs the copy of a tile's bytes onto a mapped tile to be
ordered with the draws, which the landing of a DirectStorage tile already is.

**What it is not.** It is not a second policy beside the first: the headroom pass of pull
request 33, which releases what nobody has wanted for a while, is the order's second key and
is not kept as a pass. It has no notion of a window that moves: that is the binding's (4.5),
which turns a global tile into a slot; this manager's unit is the global tile, as 4.7 says.
And it does not choose the pool's size, which stays a budget that is stated.

**Its gate** is step 1's audit, which computes the map's function from the tiles: clean over
the flight that paints and over the storm rail, with no tile Loaded and in no queue. With it:
the pages ledger's count of tiles read and never mapped, which is zero; the settled stills'
hashes equal from run to run at the bird pose, where today they are not; the solver's pinned
domain whole in six runs of six; and the lines of `src/hal/Residency.*`, which are 2,670
today, fewer.

**Built as far as the order's turn, 2026-09-29** (a scratch tree, not committed, beside
today's manager behind the key `streaming.manager`, whose default is today's). Three parts
of five are built: the key and the split; the map as a function of what is held, born saying
nothing; the order's turn, its loader, its gather and its release in two steps.

- **With the key at its default nothing moved:** every settle and prediction hash equal at
  the six poses, pixels within the floors two runs of one binary make.
- **The map is right at every audited turn.** On the flight that paints and on the storm
  rail, 1,350 turns each: no byte finer than the tiles, none coarser, none split, no orphan,
  and nothing gathered and not filled. The flight painted 100 tiles and invalidated 77 of
  them over a mapped descendant, which is finding 3's case, and left no coarser cell. On the
  same rail today's manager showed finding 63 on 20 turns and finding 2's 7,917 orphans.
- **It livelocked, and the cause is a word of law 2.** The predicted walk speaks every third
  turn. "Wanted this turn" counted the manager's turns, so the want grew and shrank by some
  1,600 tiles every third turn, the cut fell inside it, and the same 48 tiles were loaded,
  let go, mapped and released in a cycle, with 3,900 tiles of the first P never begun. **A
  want stands until its reader speaks again:** lateness is counted in the reader's own
  statements. Being built.
- **With no weight the lost tail is chosen by address,** which is no law. At the helm the
  want is 14,935 tiles against a cut of 7,808, and of the 7,157 lost the wave field's were
  5,657. The weight is law 4 and is not built yet: inside the rung that straddles the cut
  the tile nearest what its reader looks at comes first, whatever its tenant.
- **The hold lifts the cap, as today's does.** A still is an instrument that makes the held
  set the want set. With the cap kept in the hold the stills differed from today's in half
  their pixels, all of it the water, which says what was absent and not whether the picture
  is right. What the cap loses is judged on the rail and by eye.
- **The turn is too slow as built:** 4.84 ms in the mean on the storm rail where today's is
  1.24, because the order is made again on turns when no reader spoke and nothing landed.
  It is a gate of the next part.
- **The lines so far:** 458 in the new file and 191 added to the engine's others, 28 removed.
  They are beside today's manager; what the step removes is removed when the key's default
  changes.

**The fourth part, the same day: the stills are one picture, and two gates were written too
simply.** Lateness in the reader's statements, the weight, the hold that lifts the cap, the
retire loop's unmaps in one call.

- **In the hold the two managers draw one picture.** Six poses, two runs an arm, all four
  runs pixel for pixel, every mapped set's hash equal but one run's wave field by six tiles.
  Two things that are not the map had to be taken out of both arms for it: the solver's
  spin-up, which under the new manager runs on the real bed and not on 0.0 m (law 9 at
  work), and the churn, which builds on the tiles that are there.
- **At a standing camera the held set settles and then changes by no tile,** and nothing is
  read twice. On the storm rail 3,215 tiles were read again within the glance, where the
  third part read 18,660 again. They are the prediction's: a tile it asked for, that its
  next statement no longer holds and the view does not hold yet, is wanted by nobody for
  some turns, and at the cap what nobody wants now is past the cut. **A prediction is a
  statement about an interval:** what it asks for stands until the instant it predicted has
  passed. And a quarter of the tiles released were wanted back while their slot was still
  retiring, so a tile is taken back from the retire list, as Intel's sample does.
- **The weight was upside down for the water.** At the cap, in the helm phase, the nearest
  band of the sea lost its finest wave tiles and the far water kept them, with a straight
  edge across the picture. The walk's leaves were weighed by their distance from the eye
  and the wave's wants by a focus point. A want made on a view's behalf takes its weight
  from that view's eye, whatever the tenant.
- **What the cap loses is the wave field's finest level,** with or without the weight: at
  the helm 14,947 tiles are wanted and 7,808 are held, and of the 7,139 lost 5,280 are the
  wave field's at 2 m. The weight chooses which, not how many. The want is 934 MB and the
  pool 512 MB: the pool's size is the owner's.
- **The turn costs 3.1 to 3.4 ms where today's costs 1.2,** and the sort is not the cost:
  1.55 ms of a pass is the walk over 12,315 tiles, three reads apiece in scattered memory.
  The turn's work is to go with what changed and with the one rung that straddles the cut.
- **The order of a frame, measured:** the solver reads its bed before any want of the frame
  is said, and the terrain and the water bank read before the turn. A turn at the head of
  the frame, on the wants of the frame before, precedes every read (law 8).
- **The lines:** `Residency.*` is 3,595 where it was 2,670, both managers standing.

**One law more, from Haulover: a tile is held whole or it is not held** (finding 83). 4.11
says that absence is the coarser ancestor and never zero. Today a composite whose source
was refused is delivered without that source and mapped, so the map names a tile that
holds nothing of the ground, the land is black, and the coarser imagery, which is real,
is not read. A tile that a source refused is not delivered. It is unreachable for the
run, as a tile that failed four tries is, so the hold can still be exact without it, no
loader asks for it again and again, and the level above it answers. It is a change in
the tile tree's composite and in what the loader does with its answer; not built.

## 5. Decisions for Mark

Settled already, 2026-09-28, and built into the sections above:

- Several gates are visible at once (4.8).
- **A place is data and a scene entry.** Once its tiles are pulled from Google and NOAA, putting a
  boat there is an edit to a scene file and to nothing else. Step 8's gate is that the source tree
  does not change.
- One level for the shoreline and the water, from the one structure, global data beneath and finer
  data above (4.11).
- Pixel water is the look in use: every shipped scene sets it, the Droste recipe and the Haulover
  demo included, so findings 6 and 7 are on screen today.
- Time is bindable: each producer has its own clock, set in the scene (4.11). Real weather now,
  or a clear sky over a sea driven by a hurricane of last season.
- Entities bring their own meshes and textures and must be able to change scale (4.9).
- The renderer is fed the water the boat floats on and does not know a wake from a wave (4.11).
- A value is bound as a clock is: a gain is stated where the data enters and both readers receive
  it (4.11).
- The renderer does not know what the data is. Every snipping of the collage declares itself, and
  a datum's separation is one more snipping (4.11).
- The GIS is a bank, and a map in the game is a view with a lens over the game's own tiles (4.13).
- The cache is on disk and DirectStorage pulls it; it is wrong for three reasons and one
  mechanism answers them (4.14).
- Findings 2 and 3 are replaced by the new residency manager, not repaired in the old one. The
  audit that can see them comes first. It has come, and it saw both; the manager's laws are
  proposed in 4.19.
- The measure of the abstraction is music on the water: a music file is a source of the water's
  physics as a buoy is, and the objects placed with it are shown moved by the water or by the
  music, all by an edit to the scene (4.16). Wanted at some point, not now.

Settled by the eight answers of the same day:

| asked | answered | where |
|---|---|---|
| The lattice of the windows | The cube's. The structure accepts inserts from, and may be fed entirely by, sources that are not Google. | 4.2, 4.15 |
| The stride | Three rungs. | 4.3 |
| Must two machines compute the same boat, bit for bit? | Yes, and the water should be stateless enough for it. A service that computes the water once and streams the same grids to both clients is wanted as well. | 4.11 |
| Is the shoreline where the water meets the bed? | Yes. The classifier's band (finding 1) is superseded; the survey works in the compose and in maps. | 4.13 |
| The pixel water's two defects (findings 6 and 7) | Fix them now. | step 1a |
| The work found only in a working tree | Commit it; pull it in if recommended. It is `claude/gate-pose-and-pool-headroom`, and it is recommended: see below. | |
| Which places, and how much a day | Haulover, and the body of water with the most data. 5 to 10 GB a day; Google on its free tier. | 4.15 |
| Should the Scriptorium serve this and the review? | Yes: `plan`, beside `math`. | |

**The kept work is recommended in, whole.** Its box posed twice is what a portal onto an aerial
view needs: a gate whose far end leans looks down. Its pages ledger is the per-lattice residency
instrument step 1 asks for. Its headroom pass is measured (ALGEBRA priors 47 on that branch: 2.5
times the tiles mapped at a place arrived at) and its rule, that a cache decides every frame what
it holds, carries into the manager that replaces this one. It was based on `e6acf22`, gated
against it (the review, section 4), and merged the same day on the owner's word: `main` is
`d7c209d`.

And by three more:

| asked | answered |
|---|---|
| Is the kept branch pushed and opened as a pull request? | Yes: mcgrottys/GAGAME pull request 33. |
| Chesapeake Bay as the second place? | Chesapeake. |
| Does the colour's base move to sources that may be kept? | The colour is a composite of its sources. For now Google fills the base. |

**Asked on 2026-09-29, and answered the same day:**

| asked | answered | where |
|---|---|---|
| Are the pieces that are made committed, each a pull request of its own, in the order the dry run applied them? | Yes. | section 6, Standing |
| Is the solver held until its bed is whole, and its window drawn in from the survey's faded edge so that the river enters? | Yes to both, together. Every baseline is taken again once, after both. | 4.17, findings 48 and 68 |
| The looks: the pixel water's fixes, and the colour read from the cube's lattice | The pictures look good. | step 1a; 4.17 |
| Imagery for the second place | Google's tiles for Haulover or the Chesapeake, kept in the cache as the others are, under a cap: 5 GB a day, and 100,000 tiles a day, of which the test should need few. | 4.15 |
| Is the wave tree given a cap? It is 58 GiB of the cache's 95.6 | No. The cache may hold 200 GB where it might hold 100. | 4.14 |

What is still the owner's to say, each when it comes: the merging of a pull request; and
the prune tool's two modes that move and delete.

Not asked, and done unless refused: the floor law is tried first, by itself (step 2), and
Google is asked for no more than 3,000 tiles a day (4.15).

## 6. The order of work, and the gate for each step

Each step names the instrument that can see it fail, and what that instrument cannot see.

| step | what | gate | blind to |
|---|---|---|---|
| 0 | Probes in `--selftest`, no behaviour changed: the address bits the adapter reports; one heap tile mapped at two slices and at two mips, filled through one, read through both; WRAP sampling of a reserved slice under a residency clamp | bytes equal, per probe, and seen to fail on a planted wrong mapping | a driver that shares correctly only under load |
| 1 | Instruments before changes: the residency audit (bytes against the mapped set); a line per shutdown phase, flushed; the residency lens; the pages ledger of the kept branch. Done: it reports finding 3, and finding 2, and two more | the audit run on today's binary over a flight that paints: it reports finding 3 or clears it | the audit sees a wrong byte, not a wrong picture |
| 1a | The pixel water's two defects (findings 6 and 7), in the shader as it stands | a probe of the cast's landing point against doubles; stills and a rail, before and after, for the owner's eye | the look is his to judge, not a threshold's |
| 2 | The floor law. The first, a 3 by 3 of cells read bilinear, measured unsound and was not staged. A margin per level, M = 10 texels of the level read, reads zero in the probe and in the engine (4.6). Staged by the cell and read bilinear it is sound, softer than it need be and slow to refresh. Evaluated by the tile it is sound, as cheap to keep as today's and sharp near the ground, and it is coarser at the globe and doubles the globe's pass on the GPU. Neither is kept. Next: a footprint shortened to what is resident, the level raised only within the trilinear read's reach | the GPU probe of 4.6: no sample holds any of a NULL tile's zero, under the engine's own samplers; then the storm rail A/B by eye and by SSIM | one adapter and one driver; a settled still cannot see a transition |
| 3 | `Lattice` gains the face-plane window: ground metric, box, texel, tag, the plane rows. The address function in C++ and HLSL | `uv_precision.py` as a selftest; GPU readback of the address at random points | nothing downstream reads it yet |
| 4 | The tree keyed `(face, rung, x, y)`, in four parts (below the table): 4a the tree made fit for depth, 4b its names completed, 4c the pyramid painted at the Merrimack, 4d the audit across lattices | below the table | picture quality: by eye, in the albedo lens |
| 5 | **After step 6's standing blocks** (4.17). The residency manager that tracks the pyramid's tiles, with windows that activate, move and release. Its laws are 4.19's: one order, the held set its first P, the map a function of what is held. It replaces the code of findings 2, 3, 24, 34, 63, 64 and 66 | 4.19's: step 1's audit, clean, over the flight that paints and over the storm rail; no tile read and never mapped; slot audit; `[settle-exact]` hashes over global tiles and origins; the storm rail | whether the picture is right |
| 6 | **Before step 5, on standing aligned blocks** (4.17's five commits, the colour and the mask first): the address, the directory, rank 1. Then, with step 5 behind it, the porch and the phase on windows that move | stills and rail against the Mercator baseline by SSIM and by eye, floors stated; a lens that paints rank and window | bit identity is gone by construction: the lattice changed |
| 7 | The height with the exposure, as one move, and with them the solver's bed: a solver integrates on a window that stands whole (4.17, measured); the wave pages, as a window on the place's own plane, the solve not moved (below); then the water surface as a tenant | `--water-probe` (drawn level against the level the hull reads), standing, per hull; `[kernel]` fingerprints; `--sea-verify` | |
| 8 | A second place, then one in each face of the cube: harvested (Haulover and the Chesapeake's mouth are, 4.15), declared in a scene file, the boat put in. What names the Merrimack in code becomes keys of the scene (`docs/PLACE_KEYS.md`, 69 rows), the place's chart first. The politeness budget governs every fetch | `git diff --stat src shaders` is empty between the scene without the place and the scene with it; one ground point read back through two worlds: equal | data quality at the far place |
| 9 | The porch at the cube's edge, and the aprons it reads (4.18) | a rail across 43.364 N; a lens that paints the weight; an apron's texel against the texel across the edge, by containment | |
| 10 | Rank 4 from the 15 cm orthos at the jetty | a texel checkerboard at the helm | |
| 11 | Gates restated as cells; cages when they are wanted | | |
| 12 | Not scheduled. The sea accepts a spectrum given as bands from a point source: a buoy's own first, then a music file in a buoy's place (4.16). After it, and only if rings are wanted: a body's displaced volume drives the water, the hull's wake first | the sea synthesised from a buoy's bands, measured back at its place, gives the bands; step 8's gate, with the music in the place of the place. For the second form, the piston's wave height over its stroke against wavemaker theory's ratio | the look, which is the owner's; two dimensions of the closed form, until they are measured |

**Step 4, in four parts.** A read-only mapping of the tree's code (2026-09-28) tested one
candidate: the pyramid is `Lattice::Cube(16384 << 17)`, a face 2^31 texels across, so that
`mip = 17 - rung` is a fixed numbering and a tile is the `TileRequest` it always was. The
addressing survives as it is: every lattice expression is in doubles, and every key packs the
mip in 5 bits and x and y in 28. Rung 17, 4.7 mm, is the finest a 32-bit face dimension holds.
A paint at mip 17 + k is the arithmetic of today's cube at mip k. What does not survive is
depth, and the mapping found the names short in more places than the review had.

| part | what | gate | blind to |
|---|---|---|---|
| 4a | The tree made fit for depth, changing no name, no byte and no picture. The fold holds one stripe where it nests sixteen today (finding 28) and walks a cold chain once, not twice; a tile-native chain stops where its ancestors are already marked; `Prefill` takes a finest mip and whole tiles; the archive assigns an offset to a payload it has written (finding 29); a refused fetch is counted and remembered, so a run says what it WOULD have fetched; the tools that nothing dispatches are dispatched (findings 37, 39) | a counter of stripes held, seen to trip on today's path; every ancestor of a painted leaf equal to the fold of its children, on a 25-level scratch tree; today's depth, old path against new, byte for byte; the six stills' hashes | a driver of contention that twelve threads do not raise |
| 4b | The names completed (4.14): a leaf's name carries a token of its bytes, a composite's key folds its inputs' tokens and not their presence, a leaf is looked up in an archive by its token, a height leaf's identity carries its content as a colour leaf's does, and the archive's key gives the face six bits where it gives three, for the aprons of 4.18. The tree's version is raised, so every tree repaints from the source caches on its next flight; nothing is fetched and nothing is deleted | `tools/hierarchy/archive_stale.py`: no archive holds bytes its loose file does not; a fold after a pack is read back as folded; the stills, where the pixels that move are the tiles that were served stale | what a token costs on a disk that is not an NVMe |
| 4c | The pyramid painted at the Merrimack: ranks 2 and 3 over the ground of today's two windows, from the cached sources, the fetch budget at zero. The real-data test of sparseness: tiles and bytes a rank, beside the Mercator trees | fetches that would have been made: zero; the census | whether the picture is good: nothing draws it until step 6 |
| 4d | The audit across lattices. Today's audit compares a tree with the flat compositor at the same address and enforces nothing | containment: a pyramid texel lies within the least and the greatest of the page texels around its centre, by one step of the byte. No constant bound is honest: across an edge in the imagery two lattices of nearly equal grain differ by as much as the edge does | a source the two lattices both sample wrongly |

Two things the mapping measured about the cost of depth. A leaf painted in a region nobody has
flown costs up to 152 tile writes today, because each absent ancestor is painted, folded
upward, and then folded again by its caller; walked once it is 16. And ten thousand leaves of
one region make some 3,400 ancestors, so a rank's tiles are about a third more than its leaves.

**Step 7, in parts.** Proposed 2026-09-29 from a read-only map of every reader,
`docs/STEP7_MAP.md`; nothing of it is built. What the map found that the plan did not know:

- **The exposure is read through the height's slice and the height's row** in both of its
  readers (finding 87). That is the code behind "the height and the exposure move together".
- **The three kernels cannot call the colour's code.** The solver, the churn and the bank
  have no surface rows, no directory and no walk; they address by latitude and longitude on
  lattices of their own and read by loads. One function is owed, the block form of the rule
  they share, with rows that each kernel's own constants carry.
- **The vertex's height is read in a way that priors 1 says returns zero** (finding 85). Which
  holds is measured first, because it decides whether those readers sample or load.
- **The exposure is read at 76 m and coarser,** which is a rank 1 block's finest level: it
  can stand on rank 1 alone.
- **By itself step 7 about breaks even in lines,** some 410 out and 380 in, because every
  Mercator reader gets a block reader. The deletion is what follows when the colour's last
  commit and step 7 have both landed: some 250 lines more that then have no reader.

| part | what | gate | blind to |
|---|---|---|---|
| 7.0 | Instruments, no picture changed: the solver's bed trace and its wait; `--water-probe` under a recipe (finding 73); the bank's fingerprint over what the kernel reads (finding 75); the lens's height on blocks; a readback of the mesh stage's height against the pixel stage's at one ground (finding 85) | the key empty: every stage's DXIL and the six stills' hashes; the trace's own planted floor | a transition |
| 7.1 | The functions, called by nothing: the kernels' block form of the bed; the composed height over the chain; a ray's walk without gradients; the two CPU twins on the walk; one helper for a box of latitude and longitude in a block's uv, where three copies of a Mercator form stand (finding 89) | the selftest, the CPU twin against a GPU readback at random ground, a wrong row planted; the hashes | the picture: nothing draws it yet |
| 7.2 | The move, as one: the height and the exposure declare the blocks in the page's place; the wants, the floors, the interests, the pin and the wait in the blocks' uv; every reader and both twins switched; the kernels given their rows | the key empty: every hash. The key set: the drawn level against the level the hull reads, inside today's floor; the bed within a stated bound of today's, texel against texel; the bed trace, every cell at the finest level of its block; the audit on the new slices; stills by SSIM and by eye | `--water-probe` sees near hulls; the trace sees the solver's bed and not the bank's; the kernels' fingerprints change by design and cannot gate this part |
| 7.3 | The old path of the two tenants deleted: the page, its rows, its literals, its closures, its tools | the key-set bytes of 7.2 | nothing new: a deletion |
| 7.4 | The wave pages as a window on the place's own plane. The solve is not moved onto the cube's lattice: a block's texel on the ground is neither square nor at right angles (523 m by 395 m at 103 degrees at the mouth, at rung 0), and the solve's closure for a blocked cell, its phase gauge and its refraction are written for square cells on east and north axes. Its grid is declared as a window on the tangent plane at the place, and its address is the law every window has: the map from the direction to a plane's grid is a ratio of planes whatever the plane, a cube's face being six cases of it. The solve's bytes stay exact and the hull's probe and the pages stay one field. What goes is the pages' z16 Mercator frame and its arithmetic | the address against doubles at random ground, as step 3 measured a face window's; the solved field against the hull's probe | a window whose plane is far from its ground: the tangent plane's texel is square at its centre and grows by one part in five million at 3 km |
| 7.5 | The water surface as a tenant (4.11) | | |

**Step 8, in five parts.** Proposed 2026-09-29 from `docs/PLACE_KEYS.md`; nothing of it is
built. The first three change no picture and need nothing of steps 5 to 7, so they can be made
beside them. The last two need step 7: a solver at a second place reads its bed from a window
that stands there, and today the one bed on the GPU is the Merrimack's.

| part | what | gate | blind to |
|---|---|---|---|
| 8a | The chart is the place's. The root place owns a chart, the type the gates already build at a far place (`Space::Anchor`: an anchor, metres a degree, the tangent frame's rows). Every reader of the four constants of `BathyModel.h:25-28` takes the chart from the place: 95 uses in 20 files. The default is today's literals, 81,660 and not the cosine's 81,654.6 | the six stills' hashes and pixels; every stage's DXIL; the selftest; the four names found nowhere outside the default | a reader that holds the anchor as a number of its own |
| 8b | The keys that exist are obeyed, and a name is the place's: the depth tree takes `data.bathy`, the velocity-gradient bank takes `data.currents`, a layer's name is built from the place's name. The Merrimack's names come out as they are, so no tree's folder moves | the trees' folders listed before and after: equal; the stills' hashes | a name that reaches a folder by a path not read |
| 8c | The scene declares the place: its name, its anchor, its chart (the cosine at its latitude unless it says otherwise), its stations and its buoy by number, its boxes, its windows. The literals of `docs/PLACE_KEYS.md` become the defaults of keys | `--print-scene` of every shipped scene, before and after: equal but for the new keys at their defaults; the stills' hashes | what is an assumption and not a literal (8d says them) |
| 8d | The second place, a scene file and nothing else: Haulover's bed, tides, currents and sea state, its chart, a window of each rank over it, the boat put in. It runs on the terms 4.15 names: the sea to the east, no river, the clocks at the focus station, the jet off | `git diff --stat src shaders` is empty between the scene without the place and the scene with it; the solver's level at the focus station against the station's own prediction; pictures, for the owner's eye | data quality at the far place; a coast that faces west |
| 8e | One ground point read through two worlds: the same texel of the pyramid reached from the Merrimack's scene through a gate and from Haulover's scene standing there | the two reads equal, colour and height, byte for byte | |

**Standing, 2026-09-29.** `main` is `71e3a33`. Steps 0 and 3 came into it with pull request
34 on 2026-09-28. **The harvest, the prune tool, step 4a, step 1a and step 1 came into it on
2026-09-29 as pull requests 35 to 39,** committed and merged on the owner's word, one commit
each, from the bottom of the stack up.

- Each stage of that stack was built alone from the `main` before it with the stages
  beneath it: a clean build, the selftest's exit 0 (244 lines in the old `main`, then 244,
  307, 336, 336 and 349), `hal_lint` passing, and the documents the engine writes at its
  boot taken into the commit that changes them. The merged `main` has the tree of the last
  stage, byte for byte, so what was built and tested is what is in `main`.
- Where a note below says that a piece of those five is not committed, it was written
  before that day's word. What is still uncommitted: step 2's probe and its forms of the
  law, the migration's commits, the solver's bed and window, and the new manager.

- **The pieces that are made apply together.** A dry run applied six of them to `main` in one
  order: the harvest, the prune tool, step 4a, step 1a, step 1 and step 2's probe. Together
  they build, the selftest exits 0 and `hal_lint` passes. Three files had lines added by two
  pieces at one place. One conflict no merge shows, and only the selftest found it: the prune
  tool stamps every folder a run uses, and step 4a's test listed the stamp as a stray file.
  The engine's lines, counted from the dry run's trees: +1,185 and -156 over the six, beside
  10,431 of tests, instruments and tools. These are the steps that add instruments; the steps
  that remove follow them.

- **Step 0 is done**, in `src/hal/Gpu.{h,cpp}`, `src/hal/TileAtlas.{h,cpp}` and
  `shaders/TileWrap.hlsl`. `--selftest` exits 0 with the probes in it, 26 new lines and no old
  line changed. The three facts are in section 2's table. Each instrument was seen to fail: a
  place moved to another heap tile was pinned to that place, the CLAMP sampler's rows were told
  from WRAP's, and a build with both plants removed failed the suite with exit 1.
- What step 0 did not see, beyond its row above: the shared tile was read by `CopyTiles`, never by
  the sampler; the shared pair was mips 0 and 1, where the design shares a finer window's mip 3
  with a coarser window's mip 0; the WRAP rows used a trilinear sampler and crossed one edge.
  Step 6 reads a shared tile through the sampler for the first time, and its lens is where a
  failure would show.
- Run under the debug layer, the selftest dies before the probes are reached, in the atlas test
  (findings 43 and 44). The probes themselves drew no message from the layer.
- **Step 3 is done**, in `src/core/Lattice.{h,cpp}` (`CubeFaceAxes`, `FaceWindow`),
  `shaders/PageSample.hlsli` (`PageTexel`, `PageTexelUv`), `src/core/SpaceTest.cpp` and the tile
  selftest. Nothing reads it yet, so it changes no picture. Measured, on the harness's own
  points at the helm: the CPU's float32 twin is within 0.0033 texel of the doubles at rungs 6,
  9, 12 and 15, in the planet's axes, in the eye's tangent frame, and in that frame scaled by
  0.001 and by 1000; the GPU's own result is within 0.0020 texel at rung 15 and 0.0014 at rung
  9. The plant, the cancellation taken in float32, is 28 texels off at rung 15 and is caught on
  both processors. The GPU agrees with the CPU twin bit for bit at 70 % of coordinates and
  within 0.0024 texel at the rest: a driver may fuse inside a dot, so the GPU is gated against
  the doubles only.
- What step 3 did not see: one place, the Merrimack mouth on face 5; ground on the sphere,
  with no relief; points within reach of the eye only; the pixel stage only. Nothing is sampled
  at the address. The anchor is chosen for the eye and not for the window, so ground far from
  the eye is addressed from the eye's own anchor and the error grows with the distance in
  texels, by arithmetic about one part in eight million of it; that growth is not measured
  beyond the 10 km row.
- **Step 2 measured its law and did not stage it** (`claude/residency-floor-law`, on
  `35a9eb7`, uncommitted). The 3 by 3 floor read bilinear lets a NULL tile into 4,221 of
  473,088 samples under the anisotropic sampler, where today's law lets it into 1,606; 4.6
  has the table, the reason, and the law proposed in its place. What the branch holds is the
  probe, the construction with its gates, and the GPU still reading the true map. Its six
  stills' hashes equal `main`'s. The law proposed in its place, a margin per level, reads
  zero in the same probe from M = 8 up, and M = 6 fails it at footprints whose length lies
  between two powers of two; the reach that decides it is measured (4.6). Not staged yet.
- **Pull request 33's rail check, taken with step 2's baselines** (all four runs after the
  archives were set aside). Per frame, `e6acf22` against the merged code: in the flight the
  mean SSIM is 0.998 where two runs of one binary give 0.9993 to 0.9996; in the helm phase,
  where the pool is at its cap, 0.88 where two runs of one binary give 0.94. So the merged
  code draws a different arrangement there, past the floor. The detail is the same within a
  half of one percent (mean PNG bytes of the helm phase, 1,148,106 and 1,149,951 against
  1,142,368 and 1,147,231), and the residency's turn is quicker, 1.16 ms where it was 1.35 to
  1.42. That is what 2026-09-17 measured on the same code before it was committed.
- **Step 1 is done**, uncommitted on `claude/residency-instruments`, on `35a9eb7`: 17 files,
  +198 and -11, and four new ones, which are the audit, the shutdown trail and the lens's
  shader. The pages ledger came with pull request 33. The audit is a pure function of the
  tiles, compared with the bytes at the end of a turn. Its selftest plants seven faults
  through the manager's own `UpdateResidencyByte` and catches each where it was planted.
  With every instrument off, the six stills' settle and predict hashes equal `main`'s.
- **What the audit saw on today's binary** (2026-09-29, the fetch budget at zero, every run
  after the archives were set aside). Finding 3, on a flight that paints, which is the storm
  rail under a swell direction never painted: 210 invalidations, 65 of them over a mapped
  descendant, all in the exposure's z14 page; bytes coarser than the tiles on 782 of 1,350
  turns, and 3,072 cells still two mips coarser at the last turn. Finding 2, on the storm
  rail: from the first turn the pool is full, tiles Loaded and in no queue, 7,917 of them by
  the end. The landing ledger, counted by itself, gives the same 7,917 in 179 turns; with
  the audit off it gives 7,954 in 178, so the audit's own cost is not the cause. Finding 24:
  no hole under bytes in flight, in any run. And two faults the review had not read,
  findings 63 and 64, both leaving a byte finer than the tiles (4.7).
- **The solver's bed was measured** (4.17), uncommitted on `claude/solver-bed-resident`: 17
  files, +474 and -21, and the bed's trace, 215 lines. The eleven kernels that read the
  bed compile to the bytes they compiled to, so the default run is today's. It found the
  cause of the startup's low basin, which had stood unproven since it was first seen.
- **Thirty boots** of the helm recipe, one at a time: every one exits 0, with the same 21
  lines of the shutdown trail. The exit-255 race (finding 33) was not met. The trail names
  the phase on the day it is.
- **The lens** paints the page that answers by hue, the level the sampler is clamped to by
  brightness, and the answering tile's outline. It reads what the pixel shader reads, which
  is three of the five tenants. Its pictures are in the branch's `out\p4c`.
- What step 1 did not see: the GPU's copy of the map; the order of work inside a frame
  (finding 34); a fault made and mended within one turn; a wrong tile under a right byte.
  Auditing every turn costs 30 ms a turn and changes what lands when: with it on, the
  settled stills orphan about 3,000 tiles, and with it off 0 to 113.
- **Step 4a is done**, uncommitted on `claude/pyramid-tree`: 17 files, +570 and -99, and a
  selftest of its own. Measured on a scratch tree of 25 levels: a leaf painted where nothing
  has been painted costs 16 writes where it cost 136, and the walk holds one stripe where it
  held 14. On today's depth the new walk leaves the files today's leaves, byte for byte. The
  walk this design had asked for, an absent parent painted and not folded, does NOT: it
  changes six tiles of 33, because today each level's 4/255 test is decided against the
  parent's own paint already risen. So what rises from a level is the versions it took, in
  order, and each level is read once and written once. The walk it replaced loses a fold when
  two threads paint siblings (finding 57); the new one reads the child again under the
  parent's stripe and does not. The archive's packer, the refusals' count, the tools'
  dispatch, the stripe's hash and `Prefill` are as section 6 says, each with its plant caught.
- What step 4a did not see: composite drops compared old against new; a parent served from an
  archive; one interleaving of the race, not all; `--warm-trees` unbounded on the real cache.
  And the walk it replaced is still in `TileTree.h`, behind a switch only the selftest sets,
  as the reference its files are held equal to. It goes when the new walk is accepted.
- **The prune tool is done**, uncommitted on `claude/tree-prune` (4.14). The cache it listed
  holds 95.6 GiB in 474 folders, 58 GiB of it the wave tree's 226 identities (finding 51), and
  nothing in it is stale yet, because nothing was stamped before today.
- **Step 1a is done**, uncommitted on `claude/pixel-water-float-wall`, moved onto `35a9eb7`,
  where it draws what it drew: no pixel differs between the two bases, and two runs of it
  are one picture. The grain of the near water was the sample point (finding 42).
- Renders are taken one engine at a time. Two engines on one GPU were measured to move the
  pictures of both: the unmodified binary differed from itself by 16,669 pixels at one pose.
- The Scriptorium serves this document and the review through a `plan` tool.

## 7. Sources

- Gruen and others, *Ray Tracing Massive Amounts of Animated Geometry*, Proc. ACM CGIT 9(4), 2026:
  <https://dl.acm.org/doi/10.1145/3820014>; AMD's summary:
  <https://gpuopen.com/learn/ray-tracing-massive-amounts-animated-geometry/> and
  <https://gpuopen.com/learn/how-tetrahedral-cages-significantly-reduce-bvh-memory-usage/>.
  The paper itself is behind a wall I could not pass; what is said of it here comes from AMD's two
  articles and the abstract as search engines quote it.
- Luton and Tricard, 2026: <https://hal.science/hal-05646767/document> (not readable from here).
- Segal, Korobkin, van Widenfelt, Foran and Haeberli, *Fast Shadows and Lighting Effects Using
  Texture Mapping*, 1992: <https://dl.acm.org/doi/10.1145/142920.134071>. Cozzi and Ring, *3D
  Engine Design for Virtual Globes*, 2011:
  <https://www.routledge.com/3D-Engine-Design-for-Virtual-Globes/Cozzi-Ring/p/book/9781568817118>;
  its treatment of vertex precision is cited from memory of the book.
- For 4.16. Tessendorf, *Interactive Water Surfaces*, 2004:
  <https://people.computing.clemson.edu/~jtessen/reports/papers_files/Interactive_Water_Surfaces.pdf>;
  Jeschke and Wojtan, *Water Wave Packets*, 2017:
  <https://visualcomputing.ist.ac.at/publications/2017/WWP/>; Jeschke and others, *Water Surface
  Wavelets*, 2018: <https://visualcomputing.ist.ac.at/publications/2018/WSW/>; Schreck, Hafner and
  Wojtan, *Fundamental Solutions for Water Wave Animation*, 2019:
  <https://visualcomputing.ist.ac.at/publications/2019/FundamentalSolutionForWaterWave/>; Jeschke
  and Wojtan, *Generalizing Shallow Water Simulations with Dispersive Surface Waves*, 2023:
  <https://dl.acm.org/doi/10.1145/3592098>. Read as their abstracts, not as the papers. The
  piston's ratio is Dean and Dalrymple, *Water Wave Mechanics for Engineers and Scientists*, 1991,
  from memory of the book; the search found the theory named and the formula nowhere in full, so
  the harness checks its two limits. MPEG audio's tolerance:
  <https://www.underbit.com/resources/mpeg/audio/compliance>.
- For 4.6 and 4.19. Intel, *Sampler Feedback Streaming* (the sample's README):
  <https://github.com/GameTechDev/SamplerFeedbackStreaming/blob/main/README.md>, read through
  a summary of the page. *How Virtual Textures Really Work*:
  <https://www.shlom.dev/articles/how-virtual-textures-really-work/>. Van Waveren, *Software
  Virtual Textures*, 2012: <https://mrelusive.com/publications/papers/Software-Virtual-Textures.pdf>,
  which I could not read from here and do not cite for any statement.
- For 4.15's NAIP row: <https://registry.opendata.aws/naip/> (the buckets are requester-pays);
  <https://planetarycomputer.microsoft.com/dataset/naip>. Both as search engines quote them.
- Microsoft: tiled resource tiers
  <https://learn.microsoft.com/windows/win32/api/d3d12/ne-d3d12-d3d12_tiled_resources_tier>;
  tier 4 <https://microsoft.github.io/DirectX-Specs/d3d/D3D12TiledResourceTier4.html>;
  duplicate mappings
  <https://learn.microsoft.com/windows/win32/direct3d11/tile-access-limitations-with-duplicate-mappings->;
  tile APIs <https://learn.microsoft.com/windows/win32/direct3d11/tiled-resource-apis>;
  `UpdateTileMappings`
  <https://learn.microsoft.com/windows/win32/api/d3d12/nf-d3d12-id3d12commandqueue-updatetilemappings>;
  `CopyTiles`
  <https://learn.microsoft.com/windows/win32/api/d3d12/nf-d3d12-id3d12graphicscommandlist-copytiles>;
  aliasing <https://learn.microsoft.com/windows/win32/direct3d12/memory-aliasing-and-data-inheritance>;
  address bits
  <https://learn.microsoft.com/windows/win32/api/d3d12/ns-d3d12-d3d12_feature_data_gpu_virtual_address_support>
  and <https://microsoft.github.io/DirectX-Specs/d3d/D3D12_FeatureLevel12_2.html>;
  sampler feedback <https://microsoft.github.io/DirectX-Specs/d3d/SamplerFeedback.html>.
- Chen, *Adaptive Virtual Texture Rendering in Far Cry 4*, GDC 2015:
  <https://media.gdcvault.com/gdc2015/presentations/Chen_Ka_AdaptiveVirtualTexture.pdf>.
- Epic, virtual shadow maps:
  <https://dev.epicgames.com/documentation/en-us/unreal-engine/virtual-shadow-maps-in-unreal-engine>.
- Proland: <http://proland.inrialpes.fr/doc/proland-4.0/core/html/index.html>.
- Burley and Lacewell, Ptex: <https://ptex.us/ptexpaper.html>.
- Porumbescu and others, *Shell Maps*, 2005: <https://dl.acm.org/doi/10.1145/1073204.1073239>.
- Hoetzlein, *Projective Displacement Mapping for Ray Traced Editable Surfaces*, 2025:
  <https://arxiv.org/abs/2502.02011>.
- GFDL, FV3: <https://www.gfdl.noaa.gov/fv3/> and
  <https://www.gfdl.noaa.gov/wp-content/uploads/2020/02/FV3-Technical-Description.pdf>.
- Tanner, Migdal and Jones, *The Clipmap: A Virtual Mipmap*, 1998; Barrett, *Sparse Virtual
  Textures*, 2008; van Waveren, *Software Virtual Textures*, 2012: cited from memory, not re-read
  for this document.
