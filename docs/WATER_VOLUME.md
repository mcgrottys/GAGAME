# The water as a volume: rolling waves on the sparse rings

Written 2026-10-18, branch `claude/jovial-lehmann-0d5c8a`, on main e7bc52f. Mark's ask: "get the
water data into a 3d voxel mesh so we can get rolling waves ... making sure the math works on our
sparse structure." What follows is the law, what was built, what it cost, and what it measured at
the Newburyport bar (`scenes/demos/newburyport_surf.json`: the buildings scene, Hs 3 m / Tp 10 s
from 95 deg at the ebb, two eyes seaward of the north jetty tip).

## 1. What the water is today, and why it cannot roll

The wave bank (`WaterBank.hlsl` CsBankFill) is a sheet of LABELS on the camera's rings: texel `l`
holds the particle `P(l) = l + D(l)`, `D = (dx, dy, dz)` -- a Gerstner (Lagrangian) displacement,
the cascades' and the solved field's orbital terms summed. The mesh (`GlobeMesh.hlsl` SurfaceVertex)
draws `P(l)` at every label, so the drawn water is already a parametric surface in three dimensions
and not a height field; what forbids the roll is a guard, not the representation:

- `kLatFoldFloor` (M9bm) fades the lateral term to zero as `areaJac = det(1 + J)` -- the tangent
  bivector's horizontal magnitude -- falls through 0.35 to 0, i.e. exactly where the crest would
  turn over. Without it the folded triangles are back-face culled (`GlobeLayer::BuildMeshPso`,
  CULL_BACK) and the fold renders as "a near-vertical wall with the water visible behind it".
- Everything that READS the water is Eulerian and single-valued: the pixel water's two rays start
  at `lvlW + dispW` over the pixel's ground point; the hull (`TreeWater::At`) inverts the sheet by
  one Newton step, which has one answer; `--water-probe` compares heights over ground points.

So the overturn is present in the data (the kernel computes `D`; the foam law pays its excess out)
and removed from the geometry, and every reader assumes the removal.

## 2. The law: the Lagrangian sheet read back as Eulerian columns

A volume stores, at a point of space, whether it is water: the characteristic function `chi`.
Its fold is the coverage mean, which is this engine's one fold law (`math('fold')`, priors 4, 16:
fold ANSWERS, never inputs): a coarse voxel holds the fraction of its fine voxels that are water,
and the heightfield is the integral of `chi` along the vertical. A multi-valued surface is no
special case of `chi`; it is simply a column with more than one crossing.

On the rings the water's vertical run-length is what matters, and it is small: through an
overturning crest the vertical over a ground point crosses the surface THREE times -- the face
below, the lip's underside, the lip's top -- and for one wave train never more (a prolate trochoid
has one loop; Longuet-Higgins 1982, New 1983, Greenhow 1983 model exactly this loop). So the
voxel column of the water, run-length encoded, is three numbers:

    water(x, z) = (-inf, face] U [lipBot, lipTop]

with the heightfield the degenerate case `face = lipTop`, no `lipBot`. That is **the column
plane**: a fourth bank on the SAME rings (`water.column`, R32_UINT, three bands of 512 rows),
mapped per logical tile exactly as disp/param/detail are (the land test, NULL tiles), filled every
frame after the fill by **CsBankScatter**:

1. every label (2 sub-labels an axis, its displacement to first order from the texel's
   central-difference Jacobian) is carried to the Eulerian texel its particle lands in;
2. its branch is the sign of `areaJac` at the label -- upright or inverted (the fold);
3. the column folds its crossings with three atomics: the upright branches' MIN is the face,
   their MAX the lip's top, the inverted branch's MIN the lip's underside.

The fixed point stores `(h + 64 m) * 1000` clamped to `[1, 2^32 - 2]`, so a NULL tile (reads 0)
and a cleared Min (reads `~0`) are both ABSENT -- the null-tile law of 2026-10-18 (a NULL tile must
mean "no departure", never a value). The fill kernel clears its own texel's three bands; a UAV
barrier stands between the two dispatches.

Readers (`Globe.hlsl` ColumnFetch): the finest ring holding the point, bilinear, with the plane's
own absence law -- a lip band's absent texel is the face's value there, so the lip TAPERS onto the
face instead of stepping; an absent face is the label's own height. Two sheets are drawn from it by
**MsLip** (`GlobeMesh.hlsl`): the same 9x9 grid of every meshlet twice, Eulerian (each vertex at
its own ground point at the column's height, no lateral term), two-sided (the tube is seen from
inside), a cell none of whose corners holds a lip emitting a degenerate triangle. The pixel stage
shades a lip fragment (mid bit 30; bit 29 the underside) as water with the sheet's own normal --
the underside's pointing down, into the tube's air -- and its own height for the rays.

**Why this and not a 3D tile volume.** `TileAtlas3D` exists (the cloud volume; 64 KB tiles of
32x32x16 R32) and a surface voxelised at ring 0's grain would need (section 5) many times the
column plane's 48 tiles for the surface band alone, and a marching-cubes mesh stage on top; the
column IS the sparse voxel of the water along the vertical, three numbers where the voxel column
would hold tens, on the structure the bank already has (rings, tiles, NULL law, residency). The
general `chi` is still the right object when the water's topology changes -- a lip that detaches,
a splash, the cage of HIERARCHY 4.10 -- and the column folds into it exactly (section 5 measures
the fold), so nothing here is lost when a 3D tenant is wanted.

## 3. What one law buys, and what it does not (yet)

Settled by the column:
- the overturn is geometry again where the kernel says the crest turns over (areaJac < 0), and
  nowhere else; the face sheet is drawn as it was (bit for bit away from folds);
- absence is declared once (NULL = no lip), on the bank's own tiles;
- the Eulerian question "is this point water" has an answer with more than one crossing, which
  the pixel water's rays and the hull's inversion can read (not wired: section 6).

Not settled, said plainly:
- the FACE sheet is still the guarded Lagrangian one; where the lip exists the guarded face stands
  a little behind the Lagrangian face, and the lip's underside is drawn over the guarded face --
  the seam is hidden inside the tube, but it is two descriptions meeting. The next step is the one
  that removes the guard: draw the face from the column too (band 0), Eulerian, and delete
  `kLatFoldFloor` on both processors (`WaterTerms.h` has the twin);
- the lip's thickness is the label cell's (one texel of labels at c = 0); the jet's real thickness
  is a flow quantity the sheet does not carry;
- the lip sheets are shaded as water seen from above (the body's depth is the column's depth to the
  bed; the underside's refraction ray marches into the body) -- a thin sheet's radiometry is a
  follow-up;
- the hull reads the Lagrangian sheet with one Newton step (`TreeWater::At`); it does not feel
  the lip. The column is a pure function of the bank's planes, so its CPU twin is a scatter over
  the hull's label patch -- `proofs/wave_volume.py` is that twin in numpy and holds the kernel to
  it;
- set B (the Droste/gate rings) and the minimap's rings carry no column (`column = false`).
- **the lip's edge is a texel staircase** (Mark's crop, 2026-10-18 evening): ColumnFetch's absence
  law is per TAP -- a tap is present or falls back to the face -- so the ribbon's boundary steps at
  the ring's 1.2 m texel, and at the boundary the bilinear's slope between a present tap and a
  fallen-back one is spiky, which is the row of white glints along the ribbon's top edge. The fix
  is a continuous edge weight, not a per-tap switch: the tube's coverage of the texel (the scatter
  counting inverted sub-labels into a fourth band, divided like the mask's opinion channel), so the
  lip's height AND its slope taper over the texel instead of stepping at it.

## 4. Prior art (searched 2026-10-18; nothing here is claimed new)

- Fournier & Reeves 1986 (SIGGRAPH): Gerstner/Rankine orbits as a parametric ocean surface; notes
  the model places breakers. The engine's sheet is this family.
- Longuet-Higgins 1981/1982, New 1983, Greenhow 1983 (JFM): the overturning crest as a parametric
  LOOP of elementary functions; New's ellipse; "loop + jet" models. The prolate trochoid's single
  loop is why one column needs three crossings and not more.
- Thuerey et al. 2007 (Eurographics; patent US 8,204,725): breaking waves on a shallow-water
  heightfield by detecting high curvature and adding a separate SHEET for the overturning water.
  The column is the Eulerian dual of that sheet, computed from the sheet the engine already has.
- Malan, SIGGRAPH 2022 Advances (Horizon Forbidden West): breaking waves as a baked cross-section
  deformation texture applied along artist curves; the inner loop is kept "because you can look
  into the tube"; position, partial derivatives and UV offsets baked; tessellation raised near the
  front. Confirms the two-sided tube and the derivative sheets; their shape is authored, ours is
  the kernel's.
- Lopatin, SIGGRAPH 2024 Talks, "Simulation and Representation of Topology-changing Rolling Waves
  for Massive Open Ocean Games": physical sim captured as approximation curves handling overhangs
  and changing topology under a strict budget (abstract only; the ACM page is paywalled).
- Jeschke & Wojtan, SIGGRAPH 2023, "Generalizing Shallow Water Simulations with Dispersive Surface
  Waves": bulk flow + surface waves on a heightfield; no overhang. Chentanez, Mueller & Kim 2014
  (SCA): 3D grid + heightfield + particles coupled -- the architectural precedent for a volume
  where the heightfield fails. Huang et al. 2021 (SIGGRAPH Asia): FLIP + BEM for splashes near
  hulls.
- No 2023-2025 paper was found that puts a wave surface with overhangs on a sparse voxel
  hierarchy; the closest is Ando & Batty 2020's octree liquid (not a heightfield), and commercial
  sparse GPU fluid (Axiom 3.3, 2024) with heightfields as collision sources.

## 5. Measured

All at `scenes/demos/newburyport_surf.json`, frozen clock (`--scale 0 --settle-exact`, time.start pinned),
600 frames, one take each (`out/surf/`). The twin: `--dump-fibers` then `py -3 proofs/wave_volume.py`.

**The sea does not fold (ALGEBRA priors 52).** At the shipped chop (1.1), over every ring, ZERO labels
have `areaJac < 0`; the inverted band is empty on 1,572,864 texels. The kernel's displacement is a sum
of linear orbits whose steepness the limiter, the Miche cap and the chop keep under the loop's
threshold, and the fold guard was removing nothing. The storm's crests at the bar do not roll in the
DATA; a crest law (the front face steepening, `u_crest > c`) is what is missing, not a representation.

**The machinery, gated on a sea made to fold** (`--set water.wavefield.chop=4`, a stand-in for that law):

| ring | texel | wet texels | labels folded | columns with a tube | lip thickness p50 / p95 | holes at 1 / 2 / 4 sub-labels |
|---|---|---|---|---|---|---|
| 0 | 1.2 m | 248,015 | 4.41 % | 1.82 % | 0.60 / 3.09 m | 60,994 / 3,228 / 1,930 |
| 1 | 2.4 m | 241,035 | 1.98 % | 1.05 % | 0.41 / 2.13 m | 49,371 / 1,000 / 460 |
| 2 | 4.8 m | 194,264 | 0.62 % | 0.55 % | 0.39 / 1.76 m | 24,531 / 263 / 68 |
| 3 | 9.6 m | 168,415 | 0.02 % | 0.02 % | 0.85 / 2.50 m | 5,915 / 29 / 0 |
| 4-5 | 19-38 m | 242,512 | 0 | 0 | - | 92 / 0 / 0 |

- **Kernel = twin.** On every band of every ring the GPU's column agrees with the numpy scatter on
  100.0 % of present texels to within the fixed point's millimetre, and presence agrees on 100 %
  (99.9 % on ring 2's lip top: one texel in a thousand, the float Jacobian's sign at a crossing).
- **Holes** (a wet column no upright label lands in, the stretch where J > 1): 25 % of ring 0 at
  one label a texel, 1.3 % at 2x2 sub-labels, 0.8 % at 4x4 -- and the remainder is the ring's
  edge and the dry neighbours. 2x2 is what ships; the taper hides what is left.
- **The fold on the structure.** `chi` (coverage) from ring 0's columns on a 1.2 x 0.30 m lattice,
  folded 2x2x2 (the coverage mean), against `chi` from ring 1's own columns on the folded lattice:
  mean |d| 0.0055, p95 0.021 (coverage units), max 1.0 at a tube's edge; the column's water height
  (the fold's vertical integral) differs by 0.10 m rms / 0.18 m p95 -- the two rings are two
  REALIZATIONS (ring 1 sheds more of the chop), so this is the ring handover's own difference,
  not the fold's. At chop 1.1: 0.0053 / 0.019 / 0.087 m rms.
- **What a 3D tile volume would cost.** The surface band of ring 0 alone, at 0.30 m voxels, needs
  472 tiles of 64 KB (29.5 MB); the column plane holds ring 0 in 48 tiles (3.0 MB): 9.8x, before a
  marching-cubes stage.

**The frame (GPU, `--gpu-time`, means over the run):**

| pass | shipped | column on, no tube (chop 1.1) | column on, tubes in view (chop 4, `tube`) |
|---|---|---|---|
| waterbank.scatter | - | 0.10 ms | 0.12 ms |
| globe.lip | - | 0.84 ms (the 81-probe early-out) | 3.0-3.4 ms |
| globe.mesh (the face) | 12.5-12.7 ms | 12.5 ms | 11.0 ms |
| still vs shipped | - | **bit-identical** (`imgdiff` none) | the lips, by design |

The first cut drew the lip-top sheet wherever the upright MAX was present (everywhere) and cost
18 ms of facets; "have" is now "a tube stands here" (the inverted branch present).

**The stills** (`out/surf/c4f/`, chop 4): `A_tube.png` the shipped renderer, `B_tube.png` and
`B_tube_high.png` the column's lips -- dark undersides along the crest lines, the tops catching the
sky. Crude: flat-shaded ribbons with the body's radiometry (section 3), on a sea distorted 4x to
make them exist. The tube clusters stand 40-200 m WEST of the jetty tip, in the throat where the
ebb stands the crests up, not on the bar the `surf` eye looks at.

**The first cut's wrong order** (measured by the twin, lip thickness p50 NEGATIVE): up the vertical
through a prolate loop the crossings are lower upright (the face), upper upright (the loop's floor,
the tube's ceiling), inverted (the crest, the lip's top); the inverted branch is the TOP, not the
underside. Bands are now 0 face, 1 lip top (inverted MAX), 2 lip underside (upright MAX).

## 5b. The lip sheet, second pass (same evening, on Mark's word: "do this task here")

Three changes, all gated by the same twin and stills (`out/surf/c4g/`, chop 4, views tube / tube_high;
`out/surf/B4_surf.png` at chop 1.1):

1. **The scatter rasterizes the label cell.** CsBankScatter maps each texel-sized cell by its
   first-order affine map (1 + J), walks every Eulerian texel in the image's bounding box and writes
   those whose centre the inverse map puts inside the cell (the images of neighbouring cells tile,
   so a stretched cell leaves no hole); an inverted cell is tested on a 4x4 sub-grid and its hits
   COUNTED into a fourth band, the lip's coverage of the texel (0..16). A cell whose image spans more
   than 8 texels is a torn sheet and writes nothing; the fold line (det ~ 0) writes its centre.
2. **The reader's edge weight is continuous.** ColumnFetch reads a lip band as the face plus the
   lip's departure from the face at each tap weighted by that tap's coverage, bilinear; the lip
   tapers onto the face across the tube's edge in height and slope alike. `have` is coverage > 0;
   MsLip's early-out reads the coverage band.
3. **Thin-sheet radiometry.** A lip fragment's body is the sheet's: the refracted ray crosses
   lipThick / |t.n| of water (the column's top less its underside at the fragment) and what it sees
   beyond is the face under the tube (a ray going down) or the sky behind the lip (a ray going up),
   one weight continuous through the horizontal; what the sheet keeps scatters in its own colour,
   lit by the sky and the sun it transmits. With no thickness the sheet is transparent and the body
   stands, so the face path is the same formula at the limit.

Measured (chop 4, ring 0): kernel = twin on face 100 %, lip top 99.9 %, underside 100 %, the
coverage counts EQUAL on 100 % of texels, every ring. Holes 3,228 -> 2,531 of 248,015 wet texels,
and of those 1,508 are the ring's 2-texel border (never read), 1,290 lie within two texels of a dry
tile (the central differences read a NULL neighbour as zero), 905 (0.36 %) are interior tears of the
4x-distorted sea where adjacent cells' first-order images neither overlap nor touch; none from
stretching (no cell past the 8-texel rule). Scatter 0.12 -> 0.15 ms; the lip pass 2.7-3.1 ms with
tubes in view, 0.83 ms without. The chop-1.1 still against the shipped renderer: 5,298 pixels (0.37 %)
differ by ONE LSB, none by more -- the new sheet block changed the pixel stage's fp16 codegen (the
same 1-LSB drift measured for fp16 in REVIEW finding 32), not its logic: no tube exists there and the
lip pass emits nothing. Not bit-identical; sub-LSB.

Pictures: `out/surf/c4g/lip_before_after.png` (the crop Mark sent, before over after): the white
staircase along the ribbon's top edge is gone and the ribbon's edge tapers; the black slit under the
lip REMAINS -- it is the guarded face not meeting the lip (section 3), the other card's work. The
lips still read as ribbons at the mesh's 1.2 m grid; a denser lip grid was not built (piece 3 of the
brief: measure first), since the slit dominates what the eye sees until the face is drawn from the
column.

## 6. Next

1. The face from the column (band 0), the guard deleted on both processors; `--water-probe`
   against the Lagrangian inversion as the gate.
2. The column's CPU twin on the hull's label patch (the scatter in `TreeWater`), so the boat
   feels the lip.
3. The pixel water's rays and the mirror's land through the column's intervals (an Eulerian
   "is water" test with three crossings).
4. The general `chi` tenant (TileAtlas3D) only when the topology changes -- fed by the column's
   fold, never beside it.
