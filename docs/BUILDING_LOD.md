# Building LOD — mass as a conformal vector, size as a level

The owner, 2026-10-09: large buildings should be seen from very far away, from near-space
altitudes, the small ones hidden; check the PGA/CGA literature and the engine's sparse structure
first, and look for a law that may serve the water's sparse voxels later.

Today (PR 74) buildings stream as 0.05° cells within `radius` of the eye, every solid at full
detail. Tokyo at 9 km is 26.6 M vertices; at 27 km it is not drawable. Raising the radius cannot be
the answer: the far field must hold *fewer, larger* things.

## 1. What the literature gives, and what I claim

| piece | source | used for |
|---|---|---|
| Sum of squared plane distances is one 10-number quadric; quadrics **add** | Garland & Heckbert, *Surface Simplification Using Quadric Error Metrics* (1997) | the aggregate's shape is a sum, so it folds |
| Spheres/planes fitted to points linearly, as conformal vectors | Hildenbrand, *Foundations of Geometric Algebra Computing* §5.3; Hildenbrand & Hitzer, point clouds in CGA (2008); Sveier et al. (2017) | the mass of a set is a conformal vector |
| Object level from object SIZE, cell from its centre | Ulrich, loose octrees (1999; *Game Programming Gems*) | where a building lives in the tree |
| Selection by geometric error over distance, replacement refinement | OGC 3D Tiles (HLOD) | when a level is drawn |
| Footprint generalized to an equivalent rectangle | Kwinta & Bac-Bronowicz, *Simplification of 2D shapes with equivalent rectangles* (2022); CityGML LoD1 generalization (Biljecki et al.) | the far shape of one building |
| Geometry sheds to statistics past Nyquist, energy conserved | this engine, `math fold` | what a hidden building leaves behind |

Each piece is published. I have not found them put together as below, but I searched only
briefly, so **I claim the combination as untested for novelty, not as new.**

## 2. The object: a cell's mass is one conformal vector

For a solid of unit density, its moments about an origin are `m = ∫dV`, `s = ∫x dV`,
`S = ∫x xᵀ dV` (1 + 3 + 6 = 10 numbers: the same ten as a Garland–Heckbert quadric). The conformal
embedding is linear in `(1, x, x²)`:

    X(x) = n0 + x + ½ x² n∞

so integrating it over the solid is

    M = ∫ X dV = m n0 + s + ½ tr(S) n∞            (a grade-1 vector of Cl(4,1))

and three facts follow, each pinned in the self-test:

1. **Folding is addition.** A union of solids has `M = Σ Mᵢ`. A parent cell's mass is the sum of its
   children's, exactly. This is the fold law of `math fold` in its linear case: nothing is
   thresholded before it is summed.
2. **Moving a frame is the versor sandwich.** A child stored about its own origin reaches the
   parent's origin by `T M T̃` with the same `cga::Translator` the sun uses, and a change of unit is
   the `cga::Dilator`. The parallel-axis theorem is this sandwich acting on the n∞ part.
3. **The vector is a sphere.** `M = m (X(c) + ½ σ² n∞)`: the centroid `c = s/m`, and the spread
   `σ² = tr(S)/m − |c|²` read back as `σ² = −M²/(M·n∞)²`, an imaginary dual sphere of radius σ.
   So the bounding/attention sphere of any aggregate comes out of the sum with no pass over its
   parts.

The CGA vector holds the isotropic part of `S`. The orientation needs the traceless part, 5 more
numbers, folded by the same parallel-axis law. Both are kept: the vector for the sphere and the
frame changes, the full `S` for the box.

## 3. The far shape: the moment box

From `S` about the centroid, the horizontal 2×2 block gives the principal heading
`θ = ½ atan2(2Cxy, Cxx − Cyy)` and variances `λ₁, λ₂`. The vertical gives `λz`. A uniform box of
half-width `a` has variance `a²/3`, so the box is

    half-extents  aᵢ = k √(3λᵢ),   hz = √(3λz),   k² = A / (4 a₁ a₂)   with A = m / (2 hz)

One law for one building and for a crowd: a single prism returns its own height exactly
(`λz = L²/12`), a rectangle returns itself, and every box conserves **volume, centroid and
heading**. Only the aspect is the moments' choice. A tower stays a tower and a block of row houses
becomes one long low box. 12 triangles replace a footprint of any size.

## 4. The correction (2026-10-10): fold, don't drop

The first build (steps 2–3, commit 23e8621) drew a building only while it covered a pixel and
kept the rest out of view. From Tokyo at 9 km the owner saw what that does: a dense square of
detail cells, a straight edge where it ends, and a thin scatter of large boxes beyond. **The
city must stay full to the horizon, the important things loading first, with no tile or ring
anywhere.** The algebra was already the answer: what is hidden keeps its mass, folded.
The pyramid of size levels was replaced by the folded tree below.

## 5. The folded tree

**The structure.** A quadtree on the harvest's own degrees: quads of `0.05° × 2^L` for
`L = −6 … +4` (87 m to 89 km). Each building's moment box lives in ONE node: the smallest quad
at least four of its radii across (Ulrich's loose tree), picked by its centroid. A house is in
the 87 m quads, a 500 m tower in the 2.8 km ones. Every node also holds two folds: **own**, the
moments of its own buildings summed, and **desc**, those of all its descendants, each child's sum
moved into the node's frame (`Moments::About`, the parallel-axis theorem, which §2 shows is the
translator's sandwich) and added. A fold is a moment box: total volume, centroid, height and
heading kept.

**The walk** (each time the eye moves 0.2% of its height or turns 0.3°, a page lands, or a detail
cell comes or goes; on the pool, never on the frame):

- A node whose quad is under `kQuadPixels` (4 px) across draws its whole fold (own + desc) as one
  box, and the walk stops there.
- Otherwise its own buildings are drawn one by one where each covers `lodPixels` (1 px), else as
  their fold. Then its children are walked; where a child's page is not loaded yet, the descendants'
  fold stands in and the page is asked for.
- A building whose 0.05° detail cell is drawn as prisms is not drawn again.
- Pages are asked for in order of their nodes' size on screen, the largest first, so the roots
  and the large structures come before the fill.

Every decision is per node, by its own distance, so no ring and no tile edge exists to be seen.
What is drawn is bounded by the screen's pixels (a node smaller than 4 px is one box) and not by
how many buildings exist. The boxes are one buffer and one draw.

**What a fold looks like.** A suburb at 30 km is one box per 87 m patch, standing at the patch's
centroid with its total footprint and its height. It reads as built-up land in the street pattern.
Up close it is faintly dot-like, and the dots resolve into houses as the eye comes nearer.

**On disk** (`--tool building-lod`, `GALOD02`): per level, nodes (80 B), their own buildings
(20 B, half floats) and an index of pages (32 quads wide, never crossing a 0.8° build band).
Massachusetts: 4.75 M buildings, 2.87 M nodes, 325 MB, 15 s. The planet extrapolates to about
49 GB, mostly nodes; a 56-byte node is the obvious next saving.

## 6. On the sparse structure

The tree is on the harvest's 0.05° grid doubled and halved, not on the cube lattice the other
tenants share (HIERARCHY §0). This keeps every building's box in the same cell as its detail
prisms, which is how a building is never drawn twice. Moving both to the cube lattice is one
change, for later. The tree's folds are also what step 4 and the water's voxels want: a node's
fold is that patch's built volume, height and spread.

## 7. Order of work

1. **The moment algebra**: done (`compose/BuildingMoments`, `--selftest [lod]`, now 14 checks,
   including a box's own moments giving the box back and two folded boxes keeping their volume).
2. **The folded tree's tool**: done (`compose/BuildingLod`).
3. **The walk and the draw**: done (`BuildingLayer::TreeFrame`, `VsBox`). Buildings and boxes
   are seen through `AerialPerspective`, the ground's own air.
4. **The residual as a ground tenant**: open; the folds now carry most of it.

### Measured (Massachusetts tree, Boston, 1600×900, frozen clock; pages still landing at capture)

| range | eye | boxes drawn (buildings + folds) | nodes walked | walk | buildings GPU |
|---|---|---|---|---|---|
| 30 km | 9.3 km up | 88,945 (22,564 + 66,381) | 100,442 | 53 ms | 0.61 ms |
| 100 km | 51 km up | 69,545 (2,399 + 67,146) | 94,403 | 42 ms | 0.34 ms |

### Superseded: the first build's numbers (size levels, buildings under a pixel dropped)

Boston from 30 km drew 14,803 boxes; Tokyo from 27 km drew 47,043; the air cost 2.7 ms over
Tokyo at 9 km (4.4 → 7.1 ms) and 0.77 ms downtown, and taken per vertex it cost more (1.87 ms).

## 8. Open for the owner

- Two numbers decide the look: `lodPixels` (1 px, a scene key), when a building is drawn on its own,
  and `kQuadPixels` (4 px, in code), how small a fold's patch gets. Halving the second roughly
  quadruples the folds and makes the suburbs finer.
- The box replaces a building's shape past its detail radius. An outline that matters at distance
  (a stadium's ring, the Pentagon) would need a second far shape. Level-2 rings simplified by the
  existing Visvalingam–Whyatt importance could serve; I left that out.

## 9. The GPU packing (2026-10-10, measured)

The owner asked that the data be packed for the mesh and amplification stages. What changed, each
measured on frozen Tokyo stills (`--scale 0 --settle-exact`, 400 frames, the city view at 9 km):

- **An instance is 48 bytes, three float4 rows**, where it was 64: the centre and its fade-in; the
  attitude as ONE ROTOR, the four components of the rotor taking the flat frame's x, y, z to east,
  up, north (a right-handed triple: built from east, north, up it is a mirror, which a rotor cannot
  reach, and the footprints came out inside-out); and a box's three half-extents with its landing,
  or a shape's record address with its landing. The GPU rotates the axes back out (`Rotate`).
- **No task list.** An amplification group takes 64 instances, reads each record's counts from the
  pool, prefix-sums them in groupshared, and launches a mesh group for every 64 edges and every 64
  roof triangles of the group's instances laid end to end -- full groups whatever the buildings'
  sizes, where a four-edged house took a 32-edge group of its own. A mesh thread finds its instance
  by six halvings over the payload's running counts. The walk's buffer phase and the 2^22-group
  chunking are gone with it.
- **Far boxes through a mesh shader**, 8 vertices and 12 triangles a box, 16 boxes a group; the
  faces share corners, so the pixel shader takes the face's normal from the surface's derivatives
  and reads the top as a roof by its tilt. The vertex-stage path (36 vertices a box) stays for a
  device without mesh shaders, on the same 48-byte rows (unmeasured here).
- **Culling** (same day): the outside is the front everywhere, counter-clockwise, and back faces
  are dropped. **`shapePixels`** (default 4): a building is its own form from that many pixels
  across and its moment box below, the same volume, centroid and heading.

| still | pass | before | after |
|---|---|---|---|
| Tokyo, shapes from 1 px (430,606 shapes) | shapes | 16.1 ms | 6.5 ms |
| Tokyo, shapes from 4 px (26k shapes, 404k boxes) | boxes + shapes | 1.71 + 1.18 ms | 1.42 + 0.70 ms |
| Newburyport market | boxes + shapes | 0.59 + 0.36 ms | 0.38 + 0.34 ms |

Newburyport is pixel for pixel the old still (13 of 1.44 M over 8 levels); Tokyo differs in 27k
pixels over 8 levels, scattered at box edges where the derivative normal meets another face than
the old per-vertex one did. The CPU walk at 1 px fell from 90 ms to 53 ms (its sort and buffer phases 27 + 14 ms to 8 + 7 ms).
The pages are kept by their worth on screen (`layer.buildings.pageMB`; the law is in
BuildingLayer::Simulate): a page is worth its width in pixels were the eye to look at it, the
least worth goes first and never to make room for one worth less, so the coarse levels are the floor.

## 10. A second eye (2026-10-11)

The minimap is a view of its own (ViewContext index 1), and a walk culled to the first eye's
frustum shows a second eye looking down almost nothing: 7 boxes over the Merrimack's mouth. So the
walk is keyed by eye (`BuildingLayer::Eye`, `kEyes` = 2): each eye keeps its own cloud origin,
last inputs, drawn buffer, held pages and wants; `SimulateEye` launches the second eye's walk
from its own camera and rectangle (FrameLoop::MinimapFrame), its result lands in the next
Simulate beside the first's, and Render draws the eye `ctx.viewIndex` names. The pages, the
shape pool and the budget are the layer's, shared: the two streams send the union of the eyes'
wants (each page once, at the most it is worth to either), and a page any eye's drawn walk reads
is kept under it.

What an eye draws is the scene's to say: `hud.minimap.layers` lists the layer names
(`"globe vessels buildings sky"`), empty meaning that list; the sky only where the eye's own law
hands it one. A gate would take the same key once what a window shows is decided per layer.

Measured on `scenes/demos/haulover_city.json` (the gate scene with the planet's buildings and
roads): over the harbour at 1.5 km the second eye's last walk drew 844 boxes + 2,164 shapes from
19 pages beside the first eye's 3,252 + 2,734 from 70; the frame 24.7 ms headless with both.

## 11. After a carry, the buildings are slow to come: measured, not yet fixed (2026-10-11)

Through the Haulover gate the first eye's walk asks for its pages at once, but they land one every
hundred frames. The instrument (`[buildings] page L.. : read, then standing N buildings on the
ground; meanwhile M raster windows decoded`) says where the time is: a page job stands EVERY
building of its page on the composed ground when the page loads, and a page is 32 quads wide --
88 km at level -1 -- so the job pays for thousands of buildings the view never asks for. At the
Merrimack that is hidden: its height source is one small raster that lives in memory (2,244
buildings stood in 9 ms). Haulover's is two 200 MB tiled GeoTIFFs decoded by the window: the page
over Bal Harbour at level -1 stood 3,466 buildings in 4.7 s, decoding 906 windows (more than the
256-window cache holds, while the globe paints the same files). The stream sends the coarse level
one page at a time, so the chain crawls.

Tried and set aside: the ground memoised at the level's grain (`BuildingLayer::Ground` is now a
field per job, kept: the right grain, but the windows dominate); the ground from the height tree's
own tiles (a cold tile paints on the spot: seconds a page).

The law that fits the owner's rule -- what is loaded and stood is by proximity and view, nowhere
by place -- is THE STAND, and it is built: a page lands with nothing on the ground (its nodes at sea
level, its buildings as the records the tree filed), and the walk, on reaching a node, asks for its
stand -- the node's own and descendants' folds on the composed ground, and its buildings once the
largest is a pixel -- at the grain of two pixels of ground at its distance (a metre at least). A
pool job stands a batch of 64 nodes with one ground field at the batch's finest grain and
publishes each node's stand whole under an atomic pointer (`Page::stood`), so a walk reads a stand
or none, never half of one, and a finer stand replaces a coarser without a race; a ribbon's
profile goes back to the pool's slot as bytes. A node without a stand is tested at the worst of
elevations (its sphere grown 9 km) and draws nothing until its stand lands -- a few frames, in
the order of worth on screen. A walk asks for its 4,096 most worth; a landing re-walks the eye at
most every 15 frames.

Measured: the Haulover carry draws the first eye's buildings within 80 frames (67 boxes + 109
shapes at frame 209; before, 22 + 8 at frame 345), the stands 1.2 ms each over the cold rasters;
Newburyport's market still is bit-identical to the one before; Tokyo under a pinned clock makes
178,740 stands in 400 frames at 0.1 ms each, the frame 16.5 ms (15.3 before), its still within
4.7 % of pixels of the merged commit, all of them the far field still standing. The instrument
for a page that is slow to parse stays, and `[buildings] at the end: N stands made` says the rest.

## 12. The worlds a view reaches (2026-10-11)

Through a gate's window the far world showed no buildings: the layer walked only from the eye's
own position and cut away every pixel a window shows. The owner's law from the portal work -- the
portal owns the residency of what it shows -- gives the form: an eye's walk is one of several, one
per world the view reaches. Each window world is walked from the eye carried through that window
(`WindowLink::carry` on the camera: its position and forward in that world's own frame), by the
same walker as the eye's own (`BuildingLayer::Eye`, now `kEyes x kWorlds`, the frame loop saying
each eye's worlds every frame in `PublishBuildingWorlds`), and drawn pulled back through the
window's motor: the walk's origin through `pull` in doubles, the instances' centres and rotors
composed with the pull's rotor (`gBdWorld`) on the GPU, and the pixels kept only where the ray
reaches that world's depth (the hulls' rule, `WindowChainDepth`), lit by that window's sun and
zenith. Two windows deep (`kWorlds` = 3); beyond that a window is too small to hold a building.

Measured from the Merrimack's mouth looking through the Haulover gate (`out/roads/city_mouth.json`,
a frozen still): the own world 227 boxes + 65 shapes, the Haulover world through the window 67 +
171, the Merrimack again through two windows 312 + 99; the still unchanged from frame 400 to 1200.

And lit as that world is lit: the owner saw the far side's buildings carrying the near side's
sunset. Buildings.hlsl now defines the sun, the zenith and the eye's radius the sky and haze
helpers read (`GA_SUN_DIR`, `GA_SKY_UP`, `GA_SKY_EYE_R`) as statics loaded at the top of PsMain
from the view's world table at the pixel's depth -- Globe.hlsl's LoadLevel, for the buildings --
so a window world's walls take its own sun through its own air and its own sky's irradiance.
Through the Haulover gate at the Merrimack's sunset, the far towers stand in Florida's noon.
