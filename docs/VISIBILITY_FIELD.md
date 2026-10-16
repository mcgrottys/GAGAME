# The visibility field: one walk for the air, the residency and the occlusion

2026-10-15. The owner, after the building shadows landed: "speed up the building loads and the
GPU passes ... a GA oct tree ... for both atmospherics and residency, culling the buildings that
are too low or small and obscured by the atmosphere". This note measures where the cost is, states
the law that was missing (the air is a term of what is seen), builds its first step, and proposes
the structure that carries the rest. Numbers from Tokyo city (9 km range, 1600 x 900, the clock
pinned at 16:30 JST) unless said.

## 1. Where the cost is (measured, PR 94's binary)

| what | cost | how it is driven |
|---|---|---|
| the buildings' walk (CPU, pool thread) | 128 ms a walk: 274,224 nodes visited (visit 106.6 ms), 39,914 boxes + 14,642 shapes out | `lodPixels` = 1 px: every quad whose largest building could be a pixel is opened, to the horizon |
| the stands (CPU, pool) | 0.1 ms each, 178,740 in 400 frames | one per node the walk reaches, at two pixels of ground |
| the pages (Io lane) | 865 pages, 244 MB resident; a load waits 1.3 ms, reads 0.2 ms | what the walk wants |
| buildings on the GPU | 2.0 ms (boxes 0.56, shapes 0.48) + the terrain march on the faces 2.0 + the sun map 1.0 | the instances the walk emitted; 12 height samples per building pixel; one more draw |
| the globe on the GPU | 13.8 ms, of which the terrain march 4.0 | 12 height samples per land pixel |
| the frame | 22 ms (45 fps) settled | |

The far field is the cost. Nodes grow as the square of the range and the law that opens them
(a pixel of size) does not know that at 100 km the air has taken most of what a pixel would show.
The near field's cost is per pixel (the two marches), and the plan for that is the height fold's
max lane and the horizon tenant (the side-session note `cranky-spence-horizon-plan.md`).

## 2. The law: what is seen is contrast, and the air takes it

A building is drawn today when its diameter covers `lodPixels` of the picture. But the picture
holds not the building, only its CONTRAST against what is behind it, and the air between the eye
and it leaves T(r) of that contrast (Common.hlsli `AerialPerspective`: col T + L; the L is the
same for the building and its ground and cancels in the difference). So the size a building must
have to be worth a pixel is lodPixels / T(r): one pixel at the quay, 2.5 at 50 km, 6 at 100 km in
clear air, and under a haze whatever the haze says. T is bounded from above -- the extinction at
the path's least dense point, the eye's altitude over a city (the Rayleigh and aerosol rows of
scene/Air.h thinned by their scale heights) -- so the law can only keep what the air leaves and
never cut what it does not. It has no knob: lodPixels is the knob it already had, and the air is
the scene's.

This is Luebke and Hallen's perceptual criterion (contrast against the visual system's
threshold) with the atmosphere as the thing that lowers the contrast, and the flight simulators'
"fog culling" made exact: not a visibility range chosen by hand, but the transmittance the sky's
own integral uses, at the size law's own place. Prior art: Luebke and Hallen 2001, "Perceptually
driven simplification for interactive rendering" (contrast and spatial frequency decide what is
imperceptible); Outerra's object threshold "at that distance aerial fog plays a significant role";
FlightGear's static-object visibility range; Cesium's screen-space error, which has no air term.

**Built (this branch):** `BuildingLayer::SetAir` carries the planet's rows to the walk;
`Walk` computes the extinction at the eye's altitude once and multiplies the threshold by
exp(extinction x distance) at every node, building, fade and finest-children test (`tau`); the
walk's line says how many nodes the air cut. Measured against the binary without it: section 5.

## 3. The structure: the lattice's cells as the field

The engine has an octree already. `OriginLattice` (core/OriginLattice.h, docs/ORIGINS.md) is a
dyadic icosahedral lattice of directions on log-radius shells: a cell at level L is a triangle of
the sphere between two shells, and its children are its four triangles on its two half-shells,
eight of them -- an octree whose levels are versors (a cell's frame is its origin's tangent frame)
and whose cells are the floors of the eye's address, exactly the object the owner asked the
designs to be stated as. Nothing is stored; a cell is an equation of its index.

**The field.** Each frame, about the eye, a walk over these cells (coarse to fine, stopping where
a cell is small enough on screen or out of sight) assigns every cell ONE number per channel of
visibility, from which every reader takes its answer:

- the solid angle the cell subtends (its size on screen), from its shell and its triangle;
- the air's transmittance T to the cell's centre -- the same closed form the sky uses
  (Atmosphere.hlsli `AtmSunT`'s Chapman column), on the CPU, once per cell instead of per node;
- what the frame drew in front of it -- the occluder grid (Renderer::Occluder) on the cell's sphere,
  once per cell instead of per node;
- and later, what stands in the cell -- the height fold's max lane and the building field's
  roof-height moment, folded up the same cells, so a cell whose tallest content cannot reach above
  the ground in front of it is dark to every reader at once.

Their product is the cell's worth. A cell under the threshold is closed, with everything in it.

**The readers.** Today four walks each test the horizon, the cone, the occluder and the pixel
size on their own trees: the buildings' (`BuildingLayer::Walk`, 274k nodes), the globe's
(`GlobeLayer::SetView` + `PredictWants`), the exposure's and the masks' through the globe's wants,
the clouds' column. With the field, a reader's walk begins at a cell that is open and asks its
own tree only inside it: the buildings' quadtree over a 1.4 km cell is a few dozen nodes, and a
cell the air or the ground has closed is never entered. The stands are asked for by cell worth,
the pages by cell worth, in one order for every tenant: the residency manager's "what is
wanted" becomes a reading of the field (HIERARCHY 4.19's manager already states wants per
reader; the field is the common prior every reader multiplies its own by).

**The atmosphere in the same cells.** The clouds are a column texture today (M6c) and the air's
density a formula of height. A cell can carry a density (a cloud bank's sparse voxels, the fog
over a valley), folded up the lattice as any lane: then T to a cell is the product of the cells on
the way -- Crassin's voxel cone tracing on the lattice, the cone the eye's pixel -- and "the city
under the overcast is not loaded at a metre" falls out of the same number. One structure answers
"how much of this reaches the eye" for the air, the clouds, the ground and the towers, and the
residency reads the answer rather than each tenant guessing its own.

**What it reduces.** Four cone/horizon/occluder walks become one; the per-node air and occluder
tests of the buildings' walk move to the cells (the walk's 106 ms of visits are mostly nodes that
one cell test would have closed); the pixel-size law, the air law and the max lane become three
lanes of one fold. Lines: the buildings' walk loses its horizon, cone and occluder blocks (~90
lines) and gains a cell iterator; the globe's wants likewise. The field itself is ~250 lines on
OriginLattice.

### 3a. What the field cannot do alone (found building step 2, 2026-10-15)

The buildings' walk stops a subtree at the first node that fails the horizon, the cone or the
occluder, so a field carrying those three verdicts closes exactly the nodes the walk already
closes and cannot reduce its visits; the 200k nodes it visits at Tokyo city are open by every one
of those tests. The field earns its place by two things the walk does not have: (1) verdicts
SHARED by every tenant (the globe's, the masks', the exposure's wants are each their own walk
today), and (2) CONTENT lanes folded on its cells -- the tallest thing in a cell, the roofs'
height, a cloud's density -- which no tenant's tree carries and which close cells no geometric
test can. So the order below puts the content laws first, where they can be stated on the
buildings' own tree now, and the field where several tenants read it.

The first content law tried was the owner's "too low", **the lift** (section 6): a drawn solid
adds to the picture only what it lifts the imagery by, its height across the view, h cos e. It
cut nine tenths of the boxes; the owner liked the suburbs showing and set it aside for the three
FORMS (section 7): the same buildings, each as the cheapest form its size on screen allows.

## 4. Order and gates

1. **The air term in the walk** (built): section 5.
2. **The three forms** (built; the lift of section 6 tried and set aside): section 7 -- a point
   under boxPixels, the box to shapePixels (8), the mesh above; opaque at every size.
   The walk's per-building work moved to the GPU and its stand asks made a list (built): section 8
   -- the walk 84 -> 16 ms at the city, O(nodes visited); the fold stand-ins gone (the owner: one
   hexahedron per building, never a node's mass as a box).
3. **A height lane in the building tree** (the planet tree rebuilt with each node's tallest
   building; hours, every core): then the lift closes SUBTREES, not just emits, and the walk's
   visits fall with the boxes. The gate: nodes visited and the walk's ms at the city and 30 km,
   the same instances out.
4. **The field over the lattice, read by the globe's wants and the buildings' walk**: one
   horizon/cone/occluder/air verdict per cell, every tenant reading it; the gate is the residency
   turn's ms and the loader's issued reads at the storm rail and Tokyo city, and the lines removed
   from each walk.
5. **The max lane / roof lane folded on the cells**: the occlusion by content; then the horizon
   tenant retires the per-pixel marches (the other note).
6. **Density lanes for the clouds and fog**: the atmosphere as a sparse tree on the same cells.

## 5. Measured: the air term

Before = PR 94's binary with the world-held sun map; after = the same plus the air law. Stills
`--set capture.headless=true --set capture.frames=300 --set hud.fps=false --scale 0
--settle-exact`, Tokyo's clock pinned at 2026-08-28T07:30Z; the walk's last line at the end of
each run (settled), the 50th walk's line for the visit count.

| | Tokyo city, 9 km | | Tokyo from 30 km (tilt 75) | |
|---|---|---|---|---|
| | before | after | before | after |
| pages resident at the end | 1,634 | **678** | 1,943 | **1,326** |
| boxes + shapes drawn (settled) | 280,960 + 28,453 | 249,019 + 27,918 | 23,552 + 3,947 | 18,180 + 3,739 |
| nodes visited, 50th walk | 219,982 (144.5 ms) | 203,076, 31,457 cut by the air (114.9 ms) | | |
| stands made in the run | 146,208 | 138,414 | 166,082 | 161,832 |
| mean frame (unheld) | 20.93 ms | 20.70 ms | 15.37 ms | 15.68 ms |
| the still | | 7.2 % of pixels differ | | 0.58 % |

What it bought: the pages resident fall by 59 % at the city and 32 % from 30 km -- the loads are
what the owner asked to speed up, and the far field's pages were most of them; the walk visits
15 % fewer nodes and runs 20 % faster; the boxes drawn fall 11 % and 23 %. The frame's mean is
within its own noise (the GPU's buildings rows of the two `--gpu-time` runs are not comparable:
they were taken at different stages of the stream, 26k against 180k boxes drawn).

What it costs: the still changes on 7 % of its pixels at the city. The law is lodPixels / T, so
at 9 km (T = 0.9) a building between 1.0 and 1.1 px that was drawn is not, and every building's
fade-in band moves with it -- scattered single pixels over the whole field, not a region. The
30 km view, where the law bites hardest, changes 0.6 %. It is a scaling of the owner's own knob
by the air, and his eye decides whether 1 / T is the right scaling or lodPixels should be read as
"a pixel of contrast" (then the law is lodPixels / (T C) with C the wall's albedo contrast, and
the threshold at the quay doubles too). The nodes cut are counted on the walk's line.

## 6. Measured: the lift

Before = the air-law binary, after = the same plus the lift (`BuildingLayer::Walk`, the `lift`
test both emits ask; the walk's line says how many it cut, "too low"). Stills as section 5; the
50th walk's line for the counts, the last walk for the settled set.

| | Tokyo city | | Tokyo harbour (1.5 km) | | Tokyo from 30 km | |
|---|---|---|---|---|---|---|
| | before | after | before | after | before | after |
| boxes + shapes, settled | 241,624 + 27,265 | **24,004 + 9,138** | 90,497 + 24,080 | **12,750 + 12,321** | 17,363 + 3,680 | **732 + 594** |
| cut as too low, 50th walk | | 175,638 | | 83,095 | | 14,300 |
| nodes visited, 50th walk | 202,295 | 202,324 | 125,349 | 125,378 | 335,189 | 334,929 |
| the walk | 98.7 ms (buffers 7.4) | 87.7 (buffers 0.7) | 49.4 | 40.9 | 132.7 | 136.5 |
| mean frame (unheld) | 21.93 ms | **19.37** | 24.66 | 24.31 | 15.66 | 16.16 |
| the still | | 29 % px, SSIM 0.67 | | 4.1 % | | 2.3 % |

What it bought: nine tenths of the boxes at the city and the harbour, 96 % from 30 km; the frame
at the city 2.6 ms (the boxes' draw and the buffers the walk filled); the walk's buffers from
7.4 to 0.7 ms. The nodes visited do not move: the lift is tested at the emit, and the walk still
opens every node whose LARGEST POSSIBLE building could be a pixel wide -- the height lane
(section 4, step 3) is what lets it stop above.

What it changed: the city's still on 29 % of its pixels -- the carpet of one- and two-pixel grey
boxes that stood for the suburbs is gone and the imagery's own roofs show through; the towers,
the mid-rise blocks and everything whose walls reach a pixel stand as before. The harbour (4 %)
and the 30 km view (2 %) lose the same carpet where it was thin. The owner's eye decides whether
the imagery is the better suburb; the law has no knob of its own (lodPixels is the pixel), and
if a fraction of a pixel of lift should still draw, that is lodPixels for the height alone.

## 7. Measured: the three forms (the lift set aside)

The owner, on the lift's stills: "I kinda liked those suburbs showing ... I don't think the
transparent blocks are good. It should be a point at the lowest LOD, a hexahedron, then the full
mesh ... if a building fits more than 8x8 pixels we do the proper mesh"; and "a simple point,
maybe a GA product for orientation so that it can be lit and coloured accurately"; and "make sure
that improved scalability does not hurt frame times".

**The law.** One record (the 48-byte instance: centre, rotor, extents) and one width on screen
through the air decide the form: under `boxPixels` (2) a POINT -- one vertex through the vertex
stage (Buildings.hlsl VsPoint), lit by its rotor: the eye's elevation against the rotor's up is
the share of roof the eye sees, the rest the wall that faces it, so its colour and normal are that
mix of the palette and the point is lit as the building would be (the building field's law for the
walls it does not draw, Globe.hlsl PsSurface); from boxPixels the moment box (a hexahedron); from
`shapePixels` (now 8, was 4) the building's own form. Every form is opaque: the page fade-in and
the size fade are gone (the boxes' blend state with them; the shapes keep blending for a ribbon
thinner than a pixel, whose width is its alpha), so no block is ever transparent. Folds take the
same law by their box's width. The points are not drawn into the sun map (under two pixels, they
cast nothing a texel could hold).

**Measured.** Before = the air-law binary (the suburbs as boxes, fades on); after = the three
forms. Stills as section 5; `--gpu-time` over 600 frames at Tokyo city, unheld.

| | Tokyo city | | harbour | | 30 km | |
|---|---|---|---|---|---|---|
| | before | after | before | after | before | after |
| boxes + shapes + points, 50th walk | 168,245 + 21,173 | 51,408 + 3,003 + 153,296 | 85,263 + 22,521 | 20,555 + 2,720 + 84,080 | 11,155 + 3,108 | 5,287 + 280 + 8,856 |
| buildings on the GPU (mean / p95 ms) | 6.08 / 7.51 | **3.65 / 4.39** | | | | |
| of which boxes / shapes / points / sun draw | 2.36 / 1.39 / - / 0.65 | 1.09 / 0.35 / 0.56 / 0.14 | | | | |
| mean frame (unheld) | 21.48 ms | **19.75** | 24.66 | 23.80 | 15.66 | 15.62 |
| the still | | 17 % px | | 1.9 % | | 0.8 % |

What it bought: 2.4 ms of the buildings' 6.1 at the city (a point is one vertex where a box was
eight and twelve triangles; a building between 4 and 8 px is a box where it was a mesh; the sun
map draws a third of what it drew), 1.7 ms off the frame, every building that showed still
showing, the walk unchanged. What it changed: the still on 17 % of the city's pixels -- the
suburbs stand as single lit points where they stood as one- and two-pixel boxes, and nothing
fades: a building is there or not. The walk's own cost (85-125 ms a walk on the pool) is
untouched by the form; that is the height lane's and the field's (section 4).

## 8. The walk O(nodes): the per-building work moved to the GPU (2026-10-15)

The owner, after the three forms: does "simple" fit the structure, or is it O(N) where it should
be O(log n)? It was O(N): the walk's `visit` looped over every building of every stand it reached,
decided each one's form in doubles, copied a fresh 48-byte record per building per walk, sorted
the lot far to near and filled an upload buffer -- 85-125 ms a walk at Tokyo city on the pool --
while the GPU did the same N in under a millisecond. The structural form is CPU work bounded by
what the walk VISITS (nodes) and per-instance work on the GPU. This section builds it, and finds
on the way that the walk's time was somewhere else.

**The objects.** A stand's buildings are written ONCE, when the stand lands, as 64-byte records
into a persistent pool on the GPU (`BuildingLayer::StoodRec`; `layers.buildings.standMB`, 256 MB
= 4.2 M records; free runs taken best-fit and given back coalesced -- `Ranges` -- so a stand of
three records sits beside one of three thousand with no slot size to waste). A record is the Box
the draw stages read plus what the per-building law needs: its radius rho, the centroid's height
over the ground zc, its shape's pool address, its heading and its detail cell. It is stated about
the STAND'S OWN CENTRE -- a translator per node -- so it is small floats wherever the node stands,
and the walk's origin can move without touching it; the walk's node entry carries the stand's
centre about the walk's origin, the difference taken in doubles and cast once, and the pass
composes the two. A box that stands nothing claims no size (rho = 0), so the pass draws it as
nothing: one law, no special case. The shape's ground
frame is the box's rotor with the heading taken back out (a rotor about up by -heading, applied
first: `QMul(q, (0, sin h/2, 0, cos h/2))`); the stand checks that product against the ground
frame on the first 32 shaped buildings of a run and logs a disagreement over 1e-3 (none seen).

**The walk** (`Walk`): per visited node that has something to draw it emits ONE entry of 32 bytes
(`NodeEntry`): the stand's centre about the origin, its first record in the pool, how many of its
buildings lie within the node's size bound -- the page ranks a
node's buildings largest first, so that bound is one `partition_point`, the walk's O(log) share
of the per-building law -- and the air's factor 1/T at the node. No per-building loop, no copy,
no sort: every form is opaque and depth tested, so the order the pass writes is as good as any.

**The pass** (`shaders/BuildingsClassify.hlsl`, `BuildingLayer::Classify`), recorded once when a
walk lands, never per frame: one thread a record, its node found by a binary search over the
list's running counts, the law the walk applied before moved unchanged -- drawn when its diameter
through the air covers lodPixels; its own shape from shapePixels, else its box, a point under
boxPixels; not drawn when its detail cell's prisms are -- and the 48-byte instance appended to
that form's list. Four dispatches on the walk's one buffer: clear the header, count each form,
write the lists' bases and the draws' indirect arguments, write the records (the two classifying
passes are one pure function of the same inputs, so they agree record for record). The draw
stages (`MsBox`, `AsShape`/`MsShape`, `VsPoint`, `VsBox`) read their first row and their count
from the buffer's own header (`FormBase`, `FormCount`); `DrawSolids` binds the buffer and calls
`ExecuteIndirect` through two command signatures (a mesh dispatch, a draw) and never knows how
many. The buffer's first row is copied to a readback ring and read three frames on, so the walk's
log line still says what the GPU drew of what it sent, and the end-of-run line the settled set.

**No folds (the owner, the same evening).** The first build kept the two stand-ins the walk had:
a node's own fold as one box until its buildings stood, and its descendants' fold while their
page was on its way. On a fast zoom-out from the city thousands of pages are missing at once and
those stand-ins are the folds of 10 km nodes: opaque slabs over the land (since fbccee7 nothing
fades) that shrank level by level as the pages landed -- the owner saw them, and the 16 ms walk
showed every intermediate state where the 140 ms one had lagged the flight. He wanted one
hexahedron per BUILDING as the middle state, never a node's mass as a box. So the folds are gone
from the records, the walk and the pass: a stand's records are its buildings and nothing else, a
node is named only for the buildings within its size bound, and a page that has not landed shows
nothing until it does (`RtNode` keeps its folds for the size bound and the sphere alone). Measured
against the build with the stand-ins: the orbit view's 50th walk sends 0 records where it drew
521 fold boxes; the settled city still differs on 358 pixels (0.025 %: folds still standing at the
far edge), its settled set 62,838 boxes + 3,062 shapes + 215,686 points; the market bit-identical.
Lines: -82 / +46 more.

**The stream's order, once per walk (the loose end, the same evening).** The stands' stream merged every
eye's asks into a map and sorted them EVERY FRAME, then sent whatever was not pending -- so once a
batch had landed, the same asks (still the eyes' lists until the next walk replaced them) were sent
again, and again: 195,774 stands made in a 300-frame run at the city, three times the 62,549 the
walks had asked for, each one a duplicate of a stand already made. The instrument this adds (the
stream's ms on the frame thread, on the end-of-run line) measured it at 1.015 ms a frame. Now the
merged order is made once when a walk lands and consumed from a cursor: 0.252 ms a frame, 62,549
stands made, the settled set the same (63,156 boxes + 3,062 shapes + 217,190 points against 62,819
+ 3,062 + 215,636), the still within the stream's noise (0.081 % of pixels against the run before, the after binary's own A/A being 1.29 %).

**The black specks (found the same night; a shader bug older than this branch).** Looking toward
the sun from the city the far suburbs were a carpet of black points against pale haze, and the
owner expected the air to blend or cull them. The cull already reads the air; the points were not
hazed: Buildings.hlsl's PsMain lights a face with the planet's zenith at the point (`fromCentre / r`)
but handed the air's integral `GA_SKY_UP`, the flat frame's y, which is the zenith only at the
frame's anchor -- at Tokyo, 7,000 km from it, the sky's integral ran its ray through the wrong
air and gave a building 50 km off almost no in-scatter. The globe passes the true zenith, so the
ground under the same building hazed and the building did not. One line: the air's up is the
point's. Measured on the sunward view (heading 290, tilt 82, 9 km; 600 frames, settled), the
luma percentiles by band: the far band's p2 / p10 / p50 went 33 / 38 / 93 to 86 / 90 / 127 and
the share of pixels 40 under their band's median (the specks) 0.32 to 0.03; the mid band 0.063 to
0.000; the Newburyport market sub-lsb (the frame's y IS the zenith there); the city still differs
on 30 % of its pixels, every building now in the same haze as its ground. Points, boxes and the
terrain march had measured the same before the fix (three arms), which is what pointed away from
the forms and at the air.

**The stand's lifetime.** A stand is published (its node's atomic pointer set) by Simulate when
its records are recorded for the GPU, not by the job that made it, so a walk that reads a stand
names records the pass will find. The stand it replaces, and every stand of a dropped page, gives
its range back once nothing holds it (a walk holds the stands it named until its pass is
recorded) and kRetireFrames on. `Stood` no longer carries its buildings on the CPU at all.

**Where the walk's time was** (found with the instrument this adds to the walk's line: the visit
split into the coarse levels, the subtrees' drain on its threads, and the merge). With the
per-building work gone the walk was NOT faster: 83.9 -> 88.7 ms at the city. The visit's 79 ms
were 1 ms of roots, 18 ms of draining 205k nodes on 8 threads, and 60 ms of MERGE -- the stand
asks, keyed per node in a `std::map` and merged map into map, ~100k entries a walk while the
stands' stream is saturated (it makes 512 a frame; the walk asks for every visited node without a
stand). A node is visited once per walk, so the map was needless: the asks are a list, appended.
That was the cost; the per-building loop was never where the time went. Both changes stand.

**Lines.** +877 / -369 outside docs (the kernel 148 of them; the HAL's additions -- a UAV
buffer, a mapped readback buffer, a command signature, ExecuteIndirect, a root UAV -- and the
profiler's `LastMs`); the walk's emit lambdas, the sort, the upload copy, the CPU copy of the
instances for a lost device's report and `Stood::blds` are gone.

**Measured.** Before = fbccee7 (the sibling worktree's binary and its shaders); after = this
branch. Stills `--set capture.headless=true --set capture.frames=300 --set hud.fps=false
--scale 0 --settle-exact`, Tokyo's clock pinned at 2026-08-28T07:30Z; `--gpu-time` over 600
frames at the city, unheld. The walk's line is the settled one (202k nodes visited at the city:
before's 50th walk at frame 185, after's 100th at frame 227 -- the walks come faster now); the
harbour's is the 50th walk of each (125k / 128k nodes).

| | Tokyo city | | harbour | | Newburyport market | |
|---|---|---|---|---|---|---|
| | before | after | before | after | before | after |
| the walk, settled | 83.9 ms (visit 71.0, sort 0.9, buffers 0.9) | **15.8** (visit 14.7: roots 1.5, drain 8.0, merge 5.2; buffers 0.1) | 45.4 | **12.7** | | |
| the walk, instrumented before the ask list | | 98.8 (visit 79.1: roots 1.2, drain 18.3, merge 59.6) | | | | |
| nodes named / records sent, settled | | 20,311 / 270,068 | | 18,416 / 117,505 | | 117 / 440 |
| drawn, settled (the GPU's count) | 64,300 boxes + 3,062 shapes (points not counted) | 64,917 + 3,062 + 229,024 points (62,838 + 3,062 + 215,686 without the folds) | 21,200 + 2,720 | 21,189 + 2,720 + 90,963 | 0 + 0 | 0 + 0 + 0 |
| buildings.classify (GPU, on landing frames) | | 0.11 ms mean, 0.31 max | | | | |
| buildings on the GPU (mean) | 3.69 ms (boxes 1.12, shapes 0.35, points 0.55) | 4.26 (1.20, 0.35, 0.77) | | | | |
| mean frame (unheld, 590 frames) | 20.28 ms (GPU p95 26.3) | 20.54 (GPU p95 19.9) | 24.66 (section 7) | 23.87 | | 12.78 |
| the still vs before | | 0.82 % px (d over 1); A/A of the after binary 1.29 % | | 0.008 % px | | bit-identical |

What it bought: the walk at the city 84 -> 16 ms and at the harbour 45 -> 13, with the CPU's
per-building work gone and the stand asks a list; the walk's buffers 0.9 -> 0.1 ms; the sort
gone. The pass costs 0.1 ms on the frames a walk lands. The settled set is the same: the shapes
equal (3,062 and 2,720), the boxes within 1 % (the stream lands its pages in a different order
each run, and the forms' counts follow). The frame's mean is within the stream's noise: the
buildings' GPU row at the city is 0.6 ms more because the walks come three times as often now
(111 against 64 in 300 frames: a landing re-walks the eye every 15 frames, and a 16 ms walk is
always done by then) and the drawn set reaches its settled size sooner -- 229k points drawn at the
end against a run that counted none; the points row is where the 0.2 ms are.

What it changed: the city's still on 0.8 % of its pixels, scattered single pixels in the far
field where a building sits at a form's threshold (the pass decides in floats where the walk
decided in doubles) -- and the same binary differs from itself by 1.3 % there, the stream's own
order (the sibling note measured 0.3-6 % at Tokyo); the harbour on 115 pixels; the Newburyport
market bit-identical.

Open: the drain is 8 ms for 202k nodes visited -- the visit itself is now the walk, and that is the
height lane's and the field's (section 4, steps 3-4), unchanged by this. The records a walk sends
(270k at the city for 268k drawn) are bounded by the node's nearest distance, looser than each
building's own; the buffer is cap x 48 bytes a walk. A lost device's report (`DumpChunk`) now
says what each walker sent, not which instance the GPU stopped in. The stands' stream is
saturated for the whole of a 300-frame run at the city (512 a frame, 196k made): its order and
its rate are the next cost on the pool.
