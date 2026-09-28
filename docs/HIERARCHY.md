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
the CPU tree has no LODs.** A physics query reads the finest rung the tree holds at a place and
never learns what the GPU has mapped. Today it does learn it: `HeightPage` answers from the z14
page at the Merrimack and from the cube elsewhere.

On the GPU a page tenant stays what it is today, one reserved `Texture2DArray`. Slices 0 to 5 stay
the cube's faces with their full chains. Every further slice is a **window**: 16384 texels a side
at its finest rung, carrying four mips (its own three rungs and one shared with the rank above),
addressed modulo 16384. A global texel `X` at the window's rung lives at `X mod 16384`, and at mip
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
cube's own lattice, in the plane of a cube face, extended past the face's square where a window
needs it. The reasons, in order of weight:

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
- **The window caches are repainted** from the source caches. Counted today: a colour tree holds
  2,620 z14 and 4,215 to 16,565 z17 tiles. No source is fetched again if the paint asks the zooms
  already cached; that is checked before the first paint, not after.
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
`tools/hierarchy/tree_census.py`). In the fullest colour tree, after every flight flown so far:

| realization | mip 0 | mip 1 | mip 2 | mip 3 and coarser |
|---|---|---|---|---|
| the cube (98,304 tiles at mip 0) | 1.5 % | 4.5 % | 11 % | 35 %, 84 %, then all |
| the z14 window | 9.1 % | 39 % | 83 % | all |
| the z17 window | 68 % | all | all | all |

A window's fine mips are asked for near where an eye has been and nowhere else, and its mip 3 is
asked for everywhere in it. That is a window three rungs deep whose fourth belongs to its parent.

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
every node it draws and already asks for that node's tiles; it can name the node's window and
hand the mesh stage the window's two planes, as it hands it an anchor and a Jacobian today. A
rasterized pixel then pays no directory load at all. A ray that leaves the surface (the pixel
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
bytes: sound, and a staircase, so the picture's sharpness steps along tile lines. Proposed: when
the map is written each byte takes the largest of its 3 by 3 neighbourhood, and the shader reads
it with plain bilinear filtering. Measured on random maps (`porch_floor.py`, 76,800 samples): never
finer than today's clamp, so at least as sound; the largest step between samples a sixteenth of a
cell apart falls from 7 mips to 0.44; the price is 0.15 of a mip of sharpness on average. The same
widening makes a kernel's single byte cover its four taps (finding 9). This one is independent of
everything else here and can be tried first.

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

Three things in the manager must change first, because windows lean on all three: the tail a
batch loses at the pool's cap (finding 2), the residency bytes an invalidation erases (finding 3),
and the residency map's upload, which today re-sends a whole tenant for one dirty byte and is
capped at 256 slices a turn.

### 4.8 Portals, and several eyes

A gate's world is rasterized, not traced: the far world is walked from the carried eye, drawn in
this frame by one motor, and a per-pixel slab test keeps each pixel for the world whose depth its
ray reaches. So every world has its own meshlet records, and the binding of section 4.5 gives each
record its window and its two planes relative to THAT world's eye. A carried eye is an interest
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

Two of the three are in the names today (`kComposeVersion`, `kTileTreeVersion`, `kBlendVersion`
for the code; the sources' content hashes). The third is not, because bindings do not exist yet.
And the names stop short in the two places finding 25 found: a composite's key records whether
each input is present or void and not what it holds, and a leaf is looked up in an archive with
no key. With the names complete an archive and a loose file cannot disagree, which is git's
answer to the same question.

Warming a place and packing it are then tools a scene names, run when a place is added, and
`--warm-trees` and `--pack-trees`, which do nothing today (finding 37), are restored as those.

### 4.15 Sources, and what each may be asked for

The pyramid accepts a snipping from any source and may have any source replaced (the owner,
2026-09-28). What a source may be asked for is part of its declaration, beside its frame and its
datum: how much a day, and whether what it sends may be kept.

| source | covers | grain | may it be kept? | asked how |
|---|---|---|---|---|
| Google Map Tiles (satellite) | the planet | to z20 and finer | **Not as the engine keeps it.** The policy forbids pre-fetching, storing or caching beyond what the response's cache headers allow, and forbids offline use | 15,000 tiles a day by default quota, 100,000 a month without charge, billed beyond that; billing must be enabled on the project |
| NAIP (USDA) | the contiguous United States | 0.3 to 1 m | yes: public domain | cloud-optimized GeoTIFF on AWS open data |
| Sentinel-2 cloudless (EOX) | the planet | 10 m | the 2016 mosaic yes, with attribution (CC BY 4.0); later years are non-commercial share-alike | tiles on S3, zoom 7 to 13 |
| MassGIS orthos | Massachusetts | 15 cm | yes | already harvested |
| ETOPO 2022, CUDEM (NOAA) | the planet; the US coast | 1.8 km; 3 m | yes | already harvested for New England |
| NOAA tide stations, EOT20, GFS-Wave, the operational forecast systems | stations; the planet; the planet; one estuary each | | yes | the NODD mirrors on S3 first, NOMADS under its limit |

Two things follow. **The free allowance is counted in tiles, not in bytes.** A satellite tile is
some 20 KB, so 100,000 a month is about 2 GB, and a budget of 5 to 10 GB a day is far more than
Google gives without charge. The engine's cap for Google is therefore a count, 3,000 a day, which
keeps a month inside the allowance whatever the day's quota permits; the 5 to 10 GB a day governs
the open sources. **The colour is a composite of its sources** (the owner, 2026-09-28), and for
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
  audit that can see them comes first.
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
it holds, carries into the manager that replaces this one. It is based on `e6acf22`, which is
still `main`. It was not gated again when it was committed; it is gated before it is merged.

And by three more:

| asked | answered |
|---|---|
| Is the kept branch pushed and opened as a pull request? | Yes: mcgrottys/GAGAME pull request 33. |
| Chesapeake Bay as the second place? | Chesapeake. |
| Does the colour's base move to sources that may be kept? | The colour is a composite of its sources. For now Google fills the base. |

Nothing is open. Not asked, and done unless refused: the floor law is tried first, by itself
(step 2), and Google is asked for no more than 3,000 tiles a day (4.15).

## 6. The order of work, and the gate for each step

Each step names the instrument that can see it fail, and what that instrument cannot see.

| step | what | gate | blind to |
|---|---|---|---|
| 0 | Probes in `--selftest`, no behaviour changed: the address bits the adapter reports; one heap tile mapped at two slices and at two mips, filled through one, read through both; WRAP sampling of a reserved slice under a residency clamp | bytes equal, per probe, and seen to fail on a planted wrong mapping | a driver that shares correctly only under load |
| 1 | Instruments before changes: the residency audit (bytes against the mapped set); a line per shutdown phase, flushed; the residency lens; the pages ledger of the kept branch | the audit run on today's binary over a flight that paints: it reports finding 3 or clears it | the audit sees a wrong byte, not a wrong picture |
| 1a | The pixel water's two defects (findings 6 and 7), in the shader as it stands | a probe of the cast's landing point against doubles; stills and a rail, before and after, for the owner's eye | the look is his to judge, not a threshold's |
| 2 | The floor law | storm rail A/B by eye and by SSIM; the settled stills for soundness | a settled still cannot see a transition |
| 3 | `Lattice` gains the face-plane window: ground metric, box, texel, tag, the plane rows. The address function in C++ and HLSL | `uv_precision.py` as a selftest; GPU readback of the address at random points | nothing downstream reads it yet |
| 4 | The tree keyed `(face, rung, x, y)`, its names complete (4.14); warming and packing as tools a scene names; paint rank 2 and rank 3 at the Merrimack from the cached sources. The real-data test of sparseness: tiles and bytes a rank, beside the Mercator trees | `--tree-audit` against the Mercator pages at the same ground, within the resampling bound; fetch count zero; a fold after a pack is read back as folded | picture quality: by eye, in the albedo lens |
| 5 | The residency manager that tracks the pyramid's tiles, with windows that activate, move and release; the directory and the plane rows uploaded. It replaces the code of findings 2 and 3 | step 1's audit, clean; slot audit; `[settle-exact]` hashes; the storm rail | whether the picture is right |
| 6 | The shader contract: the walk, the address, the porch and the phase | stills and rail against the Mercator baseline by SSIM and by eye, floors stated; a lens that paints rank and window | bit identity is gone by construction: the lattice changed |
| 7 | The kernels' bed, the exposure and the wave pages through the same contract; then the water surface as a tenant | `--water-probe` (drawn level against the level the hull reads), standing, per hull; `[kernel]` fingerprints; `--sea-verify` | |
| 8 | A second place, then one in each face of the cube: harvested, declared in a scene file, the boat put in. The politeness budget governs every fetch | `git diff --stat src shaders` is empty between the scene without the place and the scene with it; one ground point read back through two worlds: equal | data quality at the far place |
| 9 | The porch at the cube's edge | a rail across 43.364 N; a lens that paints the weight | |
| 10 | Rank 4 from the 15 cm orthos at the jetty | a texel checkerboard at the helm | |
| 11 | Gates restated as cells; cages when they are wanted | | |
| 12 | Not scheduled. The sea accepts a spectrum given as bands from a point source: a buoy's own first, then a music file in a buoy's place (4.16). After it, and only if rings are wanted: a body's displaced volume drives the water, the hull's wake first | the sea synthesised from a buoy's bands, measured back at its place, gives the bands; step 8's gate, with the music in the place of the place. For the second form, the piston's wave height over its stroke against wavemaker theory's ratio | the look, which is the owner's; two dimensions of the closed form, until they are measured |

**Standing, 2026-09-28.** Steps 0 and 3 are committed on `claude/earth-texture-hierarchy-53a3b3`,
each as its own commit. The other branches named below are not committed.

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
- **Step 1a is being made** on its own branch, `claude/pixel-water-float-wall`.
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
- **Step 2 is being made** on its own branch, `claude/residency-floor-law`, from `e6acf22`. Its
  GPU probe measures the law's soundness under the engine's own samplers, against today's law.
- Step 1 waits for pull request 33 to be merged or refused: its pages ledger is that branch's.
  The branch passed its gate in everything a settled still can see (the review, section 4).
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
