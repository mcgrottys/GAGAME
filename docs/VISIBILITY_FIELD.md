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

## 4. Order and gates

1. **The air term in the walk** (built here): the walk's nodes, boxes and ms at Tokyo city and at
   30 km; the GPU buildings row; the still's difference confined to what the air had already taken
   (sub-LSB away from the fade band). Section 5.
2. **The field over the lattice, read by the buildings' walk alone**: the same stills and numbers,
   plus the walk's visit count; the gate is "the same instances out, fewer nodes visited".
3. **The globe's wants read the field**: the residency turn's ms and the loader's issued reads at
   the storm rail and Tokyo city, against today's.
4. **The max lane / roof lane folded on the cells**: the occlusion by content; then the horizon
   tenant retires the per-pixel marches (the other note).
5. **Density lanes for the clouds and fog**: the atmosphere as a sparse tree on the same cells.

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
