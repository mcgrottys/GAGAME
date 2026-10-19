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

## 5c. The face from the column, the guard gone (same night; PR 104)

Mark's second crop: a dark sheet hanging under a crest "like a reflection". Measured on ring 0's
4,596 tube columns: the three crossings keep their order (0 floor-below-face; 134 roof-below-floor =
columns with two folds, which three numbers truncate), but the column's face stands 2.0 m p50 BELOW
the guarded Lagrangian face the mesh drew at the same texel, the floor 1.3 m below it, the roof level
with it: both lip sheets hung under the drawn surface, and the tube's floor -- the back face rolled
over the crest, a mirror of it by construction -- showed under the crest's forward edge.

**Done:** `SurfaceVertex` draws the face from the column's band 0 at the vertex's own ground point
(Eulerian, no lateral term) wherever a ring holds the point; where none does (another level's rings,
a planet without a bank) the label's own particle, label plus offset, UNCUT. `kLatFoldFloor` is
deleted on both processors: the mesh's `smoothstep(0, 0.35, areaJac)` and the hull's `g` in
`TreeWater`'s one Newton step (`WaterTerms.h` keeps the note). The lateral inversion is now the
column's; the pixel stage still reads the water at a LABEL, so the vertex carries the label standing
at its point -- one Newton step on the offset's own Jacobian, `(1 + J) e = D`, label = point - e,
TreeWater's step to the letter -- in `geo`. Without that step every pixel shaded its neighbour's
particle and the chop-1.1 still differed from the shipped one on 33 % of its pixels (SSIM 0.965);
with it 2.8 % differ by more than one LSB, mean |d| 0.17, SSIM 0.9965: the same sea reconstructed
from texel centres (bilinear) instead of from labels (Catmull), plus the first-order step.

Gate: self-test passes (every FAIL a planted fault, caught); lip pass 0.83 ms without a tube; the
chop-4 tube views (`out/surf/c4h/`) no longer show the slit or the hanging floor -- what remains is
the 4x sea's own plateaus and the 1.2 m lip grid.

**What this leaves:** the two-fold columns (3 % of tubes at chop 4) truncate to one tube; the hull
reads the label's own particle in a fold (its column twin is still to build); the lips are ribbons at
the mesh's grid; and the crest law, so a real sea folds at all.

## 7. The crest law (2026-10-19, PR 104's third commit)

The sea never folded at the shipped chop (priors 52). What makes a real breaker fold, found in six
measured steps, each a dump and a twin, each wrong in a way the next one named:

1. **The lateral keeps the raw amplitude.** The limiter and the breaking factor bound the sea's
   HEIGHT; the horizontal excursion is not height. Cascades: lateral x 1/brk; the solved field:
   x max(excess, 1), the solver's own limiter inverted. Measured: the lateral grew 20 %, the
   tightest crests went from det(1 + J) 0.43 to 0.33. Not enough: 13 inverted texels.
2. **The orbit is the depth's.** Airy's ellipse: horizontal = coth(kh) x vertical. Put on the deep-
   water k the cascades are synthesized at, the 213 m band reached coth 113 in the shallows: 52 m
   of slosh, a torn sheet. The k has to be the band's IN THIS DEPTH -- Guo's explicit dispersion
   (Coastal Engineering 45, 2002; checked here at 0.79 % against the exact), `ShoaledKh` /
   `OrbitCoth` in Jet.hlsli and WaterTerms.h. The solved field's k is already finite-depth.
3. **Applied over the whole orbit, the raw ratio shears every crest sideways** by ten times its
   height (lateral / vertical 9.8 on 9 % of ring 0): a sea mid-plunge at every phase, which no sea
   is. A breaker throws its CREST; the trough stays.
4. **Weighting each of 32 components by its own crest phase drifts the whole sheet** (the weights
   average one half): the same 59 m throws. The crest is the SEA's: where its elevation stands
   against its envelope, w = 1/2 + eta / (2 sqrt2 rms), 1 at the crest, 0 in the trough.
5. **The law** (WaterBank.hlsl's solved loop, after its components; WaveField::ProbeAt the same
   on the sum, with the Jacobian by the product rule): on the summed orbit D of the solved field,
   D' = D (1 + q w) + q w A_h d, with q = excess - 1 (0 where nothing breaks), A_h = sqrt2 rms x
   coth-bar the sea's horizontal amplitude, d the amplitude-weighted travel; bounded by the
   wavelength, k-bar A_h (1 + q) <= pi/2: a crest thrown past a quarter wavelength is no longer the
   same wave (the loop wider than the crest), beyond it the water is a bore and the foam's. One
   min; no constant placed. Measured: lateral / vertical 2.5 (the orbit's), the worst throw 30 m
   (in the depth floor's shallows, dry-weighted), 2,239 inverted texels, lips 0.14 / 0.70 m p50 /
   p95 on ring 0.
6. **Two things the law exposed in the column**, both fixed: the scatter's first-order affine
   cell image (central differences over 2.4 m, noisy under the 2 m chop band) made neighbouring
   images overlap or gap -- concentric rings in the near water; the cell is now THE MESH'S two
   triangles between the labels' own particles, rasterized exactly (`CsBankScatter`), and holes
   fell 2,531 -> 1,832 on ring 0. And the per-vertex Newton step to the label (`(1 + J) e = D`)
   blew up where det was small -- rings in the shading, one hull-probe record 0.45 m off; the
   column now carries the LABEL beside each crossing's height, packed height-major in one word
   (`ColEncode`: 14 bits of centimetres, 9 + 9 bits of the label's offset), so the atomics keep
   their order and the mesh reads the exact, fold-aware inversion. Every probe record is then
   within 3-7 cm rms at correlation 1.00.

Gate (`out/surf/crest7/`): twin 100 % on face, underside and coverage, 99.9 % on the lip top;
scatter 0.24 ms; lip pass 2.0 ms at the throat; self-test passes; the surf still against the
face-from-column one differs on 25 % of pixels (SSIM 0.980) -- the orbit of a shallow sea is now
2.5 times wider than its height, which is the Airy ellipse and not a reconstruction difference.
What the eye sees at the throat: crests pitched forward with steep faces, dark crest lines where
the lips stand, no rings.

## 8. The finer column level where tubes stand (2026-10-19, PR 104's fourth commit)

The structure's own answer to the plates and terraces (Mark's crops): ring 0's columns again at a
quarter of its texel (0.3 m), four bands, in a second reserved atlas (`water.columnF`, 2048 a band,
16x16 tiles of 128^2) resident only where a tube stood. **Who decides residency:** the scatter
itself -- an inverted triangle flags the 38 m fine tiles its box touches in a 16x16 texture
(`InterlockedOr`); the flags go back to the CPU through the frame ring (`hal::RegionReadback`, a
kilobyte a frame, kFrameCount frames of latency); the CPU maps the flagged tiles' four bands and
unmaps the rest; the kernel is handed the mapped set (256 bits in the constant buffer) and
rasterizes and clears only there (a write to an unmapped tile is dropped by the hardware anyway;
the set spares the work). **Who reads it:** `ColumnFetch` on ring 0, for the lip bands only --
the lip's departure from the FINE face weighted by the FINE coverage, so the taper and the
terraces are a quarter texel's -- and falls back to the ring's columns wherever a fine tap is a
NULL tile. The drawn FACE stays the ring's everywhere: one reconstruction, no seam at a fine
tile's edge.

Measured (`out/surf/fine2/`, the throat at the shipped sea, `tube_high`):

| | ring 0, 1.2 m | fine, 0.3 m |
|---|---|---|
| lip-top step between neighbouring tube texels, p50 / p90 | 0.060 / 0.258 m | 0.010 / 0.020 m |
| tube area found (coarse-texel equivalents) | 922 | 9,397 |
| tiles resident | 48 (3 MB) | 41 of 256 (10 MB) |
| scatter / fill | 0.24 / 3.0 ms | 0.65 / 3.46 ms |

- The terraces fall six-fold in absolute height, to a centimetre.
- ~~**The surprise:** the fine level finds TEN TIMES the tube area.~~ **RETRACTED 2026-10-19.** The
  "ten times" was the DUMP's, not the sea's: `fiber_columnF.f32` is read back with a texture copy,
  and a copy from a tile the fine atlas does not map returns whatever the pool page last held -- on
  this GPU the face band's clear value, 0xFFFFFFFF, which the coverage column counted as a tube
  (35,413 of the dump's 36,449 coverage texels in section 10's still were that word). Tested tile by
  tile against the ring's face, 14 fine tiles were current and held 1,036 coverage texels: 65
  coarse-texel equivalents, where the ring's own band held 520. The fine level finds LESS tube
  area than the coarse 4x4 sub-grid, not more; what it adds is the lips' grain. A shader's read of
  an unmapped tile returns zero (the renderer was never fooled); a copy's does not: the dump must
  mask by the mapped set, or be read through a shader. The early-out's blindness in Mark's crops
  was real, but its cause is section 10's, not this.
- The cost is about 0.9 ms with 41 tiles resident; it scales with tube area, not with the sea.
- The still differs from the coarse-only one on 1.6 % of pixels (the lips' rims). Two runs of the
  fine level are not bit-identical: the flags' latency makes the mapped set depend on which frame
  the still settled on. A gate on the fine level must compare the column dumps, not the pixels.

What this leaves: the lip GRID is still the meshlet's 9x9 at the helm's 1.2 m, so the sheets' shape
between vertices is linear whatever the column's grain; a denser lip grid near tubes (an
amplification stage in front of MsLip: the 81-probe early-out, then 16 groups only where a tube
stands) is the next step, and `hal::MeshPipelineDesc` already takes an `as` blob. And the early-out
should read the fine coverage where it is resident, since the coarse band misses most tubes.

## 9. The spray: the throw the sheet cannot carry (2026-10-19, PR 104's fifth commit)

Mark's idea: the energy the fold law has nowhere to put, and the droplets a breaker shoots out,
become particles. The quantity is already in the crest law (section 7): the throw bound removes
`(lipQ - 1 - q1) w A_h` metres of the excursion the crest wanted past a quarter wavelength. That
water leaves the sheet as a parcel of spray.

**Where it lives.** A ring of 65,536 particles on the GPU (`water.spray`, two float4 each: chart
position + age, velocity + size), no CPU twin -- spray is not water a hull feels. CsBankFill spawns
at a crest texel of ring 0 (w > 3/4, wet) with probability gain x excess x dt, a parcel anywhere in
the texel. **Its launch is the breaking criterion itself:** a wave breaks when the crest's water
overtakes the crest, u = c, so a parcel leaves at the sea's phase speed at this depth,
c = sqrt(g tanh(k h) / k) on the summed k, along the travel, yawed up to 20 degrees, at 0.7-1.3 c,
rising at a quarter to a half of c (Peregrine 1983: the plunging jet's tip at 1.0-1.5 c). The
orbital velocity was the first cut -- a metre a second, a tenth-of-a-second flight. CsSprayStep is
gravity and the drag of a drop in air written by its terminal speed (a = -g |v| v / vT^2, Gunn &
Kinzer 1949: 4-6.5 m/s for 1-2 mm drops; vT = 5), death below the column's face at the parcel's
ground point (band 0, this frame's), outside ring 0, or past six seconds. GlobeLayer draws the
ring after the lip sheets as camera-facing discs lit as foam, radius 0.25 m growing a quarter metre
a second, alpha falling as 1 / (1 + age). The counters (spawned, alive) ride the frame ring with the
fine tiles' flags; `[spray]` logs them every 120 frames.

Measured at the throat (`out/surf/spray/C_*`): 27-46 parcels a frame, 320-630 alive (a flight of
about half a second), `waterbank.spray` 0.010 ms, `globe.spray` 0.003 ms.

What the still showed: the parcels stand on the crest where the lip sheets stand -- the chart
inverse in VsSpray is right -- and the first cut drew them as a string of beads on one parabola
(one texel spawning each frame); the jitter is the mend. **And a wrong site:** in `tube_high` the
parcels and the white lip facets stand along the north jetty, not on a wave (Mark). The sheet's
lateral term carries water into the rocks because nothing stops it at the shore (the particle's
landing point is never asked whether it is dry); the fold law then fires against the jetty and the
spray inherits it, its dry gate being the texel's, not the landing point's. The shore stop on the
lateral term, both processors, comes before any more spray work.

## 10. The lip fixes: the shore stop, the lateral bound, the roofs, the dense grid (2026-10-19, PR 104's seventh commit)

Mark's read of the spray still: the white lip facets and the parcels stood along the north JETTY,
not on a wave; and the lips were "very pixelated". Four laws, one picture.

**The shore stop** (CsBankFill after `d *= dry * gPatch.w`; TreeWater::Finish). The lateral term --
the orbit and the crest's throw -- carried wet labels onto the riprap, because nothing asked
whether the water the particle moves through is water. Now the whole lateral term rides the MEAN
DRYNESS ALONG THE THROW: the detail plane's dry channel sampled at eight points from half a throw
to four throws out (the bed is quasi-static; a NULL tile reads zero = land), the twin reading the
bed at the same points. In deep water it is 1; toward a wall the term fades over four throws, the
way a standing wave's excursion fades toward the wall it reflects from. The first cut read the
LANDING point alone and that was worse: the far particle stopped dead a texel before the near one,
the sheet compressed into the last wet texel and folded there, a row of false tubes along the
jetty (the chips in `B_tube_high`). The mean's steepest slope is an eighth of a texel's, so the
stopped sheet can fold only for a throw past 9.6 m, which the bound below forbids.

**The lateral bound** (after the cascade loop; the twin's lateral gains). Airy's orbit is the
depth's, coth(kh) times the vertical, and in half a metre of water that is six times a half-metre
wave: the particles crossed on every crest at the rocks and the column read a tube at every one
-- the chips that survived the shore stop (`D_tube_high`). Linear theory has no such water. The
crest law's own bound (section 7, a particle thrown past a quarter wavelength is a bore) now holds
the cascades' summed lateral term too, on the sea's energy wavenumber IN THIS DEPTH and its raw
lateral amplitude: `d.xz *= min(1, (pi/2) / (k_sh A_h))`. One min; in deep water it is 1.

**The absence law at the shore** (CsBankScatter). A label on a NULL tile or on land has no water
(its dry weight reads exactly zero) and bounds no water column; a cell with such a corner was
rasterized with that corner standing undisplaced at the waterline while its wet neighbours were
thrown toward it, and the triangle inverted. The column leaves those triangles out, as the sheet's
own mesh fades those corners by landness.

**The roofs** (CsBankScatter). The fine level's inverted hits raise the COARSE coverage of their
parent texel to at least one, so MsLip's early-out and ColumnFetch's taper know the tube is there
and the fine level supplies its shape. (Measured: every fine-coverage parent in a current fine
tile is tagged, 204 of 204.) And ColumnFetch's fine slope is no longer thrown away: the ring's
slope was assigned AFTER the fine block and overwrote it.

**The dense lip grid** (GlobeMesh.hlsl AsLip / MsLip; GlobeLayer builds the lip pipeline with the
amplification stage). The lips were ribbons on the meshlet's 9x9 grid at 1.2 m whatever the
column's grain. Now AsLip, one group a record, runs the 81-probe early-out and hands MsLip the
record and the 2x2-cell BLOCKS (sixteen a record) any of whose nine corner probes found a tube;
MsLip draws one block a group as a 9x9 grid at a QUARTER of the cell -- 0.3 m at the helm, the
fine level's grain -- the same 162 vertices and 256 triangles the first cut drew for the whole
record. A record with no tube costs its probes and nothing more.

Measured (`out/surf/lips/`, the frozen clock, `A_*` before and `E_*` after):

| | before | after |
|---|---|---|
| globe.lip, tube_high | 2.33 ms | 1.26 ms |
| waterbank.fill / scatter | 3.24 / 0.67 ms | 3.17 / 0.61 ms |
| pixels changed, tube / tube_high | | 14,120 / 33,694 of 1,440,000 |
| hull probe (helm_boat, 30 records) | 3-7 cm rms, corr 1.00 | 3-10 cm rms, corr 1.00 on 12 records; record 60 see below |

The lip pass halves (the dense grid draws only the blocks a tube stands in, and the amplification
stage's probes replace the mesh groups' own); the picture changes on 1-2 % of the pixels: the jetty's
lip facets are gone, the throat's lips are at the fine grain. Still open: the pale chips along the
jetty's wet toe in `tube_high` survive every one of the four laws -- the still with the column plane
off (`N_tube_high`) says whether they are lips at all or the coast's own stair steps.

**Record 60 of the probe** (the river mouth, 11 m deep, the hull's water 1.3 m rms where the bank
holds 0.19) is not these laws': the twin's eta is the plain sum of the solved components, untouched
by any of them, and the run's log shows the solved field's identity moving ("a worker's field
offered ... 52 held tiles stay drawn, stale") a few records before -- the GPU's pages and the CPU's
atlas were not the same field at that record. In the first run of the day the swap had not happened
by record 60 (the twin's window weight was 0 there) and every record agreed. An ownership question
for the wave table, filed here, not fixed.

**Also measured and retracted:** section 8's "ten times the tube area" (see the note there).

## 6. Next

1. The face from the column (band 0), the guard deleted on both processors; `--water-probe`
   against the Lagrangian inversion as the gate.
2. The column's CPU twin on the hull's label patch (the scatter in `TreeWater`), so the boat
   feels the lip.
3. The pixel water's rays and the mirror's land through the column's intervals (an Eulerian
   "is water" test with three crossings).
4. The general `chi` tenant (TileAtlas3D) only when the topology changes -- fed by the column's
   fold, never beside it.
