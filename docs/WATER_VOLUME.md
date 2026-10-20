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

## 11. The lips are the fine level's; a fold is witnessed on ring 0 (2026-10-19, PR 104's ninth commit)

Mark's crop of a small roll at Mavericks: a crest crease a metre wide ran down to a lip drawn as
2 to 3 m white slabs. Three causes, three laws.

- **Footprint.** The lip bands were read bilinearly from the ring's 1.2 m taps with the coverage
  taper, so a roll a few decimetres wide smeared into a 2.4 m blob, and the dense grid (section 10)
  drew the blob faithfully. ColumnFetch now reads the lip bands from the finer level alone; where
  its tile is not yet resident (the flags' two or three frames) there is no lip -- a wait, not a
  wrong picture. The early-out still reads the coarse coverage (the roof flag), so the blocks are
  dispatched the frame the tube appears and the vertices wait for the fine data.
- **Normal.** The slabs faced the sky and mirrored it white. The lip's normal now returns to the
  face's as its coverage falls, so a thin roll's rim is water; the thin-sheet path is the
  thickness by the coverage.
- **The witness's grain.** Mavericks' dump held lips on rings 1-3 (1,198 / 345 / 82 texels at 2.4
  to 9.6 m): a 17 s swell's lateral excursion is tens of metres and a coarse cell straddling a
  crest inverts by aliasing, not by overturning. Ring 0 alone writes the inverted branch; the
  coarse rings keep the face.

Measured: the slabs at the roll gone (the crease stays, as it should), the jetty's chips at the
Newburyport throat nearly gone, globe.lip 1.26 -> 0.45 ms at the throat.

**Not the lips:** the cones on Mavericks' western horizon (Mark: "likely related"). They stand with
the water bank off, with tide/gulf/sea off, with gis/markers/vessels off and with the terrain layer
off; the 60-second ETOPO has no land within 600 km on those bearings and the CUDEM's western tile
has no nodata and no spike. They lie between bearings 250 and 292 from the point, far and tall, and
they are the globe layer's own. TRACED THIS FAR (the renders T_*, X_*, E_*, L_* in out/mavericks):
they are WATER VERTICES LIFTED BY THE LAND TERM. Cutting `disp = lerp(dispWater, dispLand, landness)`
to `disp = dispWater` removes them; cutting the lateral term or the bank height leaves them whole;
the bank's every ring is within +-8 m at that eye; the ring size, the cross-fade, the column plane,
the solved field, the chop, the period and the amplitude (Hs 1 and 5 alike) change nothing; the
storm's DIRECTION does (from 110 they leave the west), and the day's forecast sea has none. The
residency and mip lenses paint the spikes in the NEAR water's rank, standing inside the far band's
rank: far vertices whose height read claims a fine, resident window and gets a land height from it
(+100 m and more; `landness` follows through `hLand`). So the fault is in the address of a far
vertex's height read -- the chain's containment or the page texel at a window's edge (the eye's
windows move with what the frame asks for, which is how the storm's direction enters) -- not in the
water. Mark's two guesses, index handling and residency, were both right. Next: read CsChain and
CsPointOfDir for a far coarse vertex against the windows' extents.

## 12. The window's edge: a filtered read stays inside its box (2026-10-19, PR 104's eleventh commit)

The cones on Mavericks' western horizon (section 11), found and fixed. Mark: "they are obviously on
the edge of different mip levels."

**The seam.** The eye's windows (Window.hlsli) are boxes of 16384 texels of rung 3(k+1) about the
eye, one a rank: rank 3's is 19.5 km wide, its edge 9.8 km from the eye -- the horizon from 60 m. A
box is addressed modulo its own size: its texture is a torus, and the box's eastern edge and its
western edge are neighbouring texels. The chain tests containment at the POINT (b in [0, 1)) and the
readers then SAMPLE with a WRAP sampler at the mip their footprint asks for. A far vertex a few
texels inside the western edge, read at a coarse mip, had a trilinear footprint that reached across
the edge and blended the eastern edge's texels: the coast and its hills, 19 km east, averaged into
the sea 10 km west. The height came back a hundred metres and more, the classifier called it land
(`hLand`), and the vertex rose to `dispLand`. Why the storm's direction moved them: the windows
slide with what the frame asks for, and the storm's swell march asks westward.

**The law.** A window answers only where the footprint of the read lies within the box. The chain
now carries each rank's distance to its box's nearest edge (`WalkChain.e*`, Window.hlsli), and
every FILTERED reader -- the height (trilinear, 2 texels of reach at the mip plus one), the colour
and the building field (anisotropic, 8), the survey mask (bilinear at its floor, 2) -- skips a rank
whose footprint crosses the edge (`WalkEdgeOk`). The rank above holds the same ground at its mip 0
with the point deep inside, so the ladder's answer is the coarser rank's, which is what the far
vertex should have read all along. The kernels' chain read (HpChainRead) takes POINT loads and a
mip texel never straddles the texture's seam, so it was never wrong and is unchanged; the C++ twin
(SurfaceFrame::Chain) is unchanged, since the chain's ranks, slices and addresses are.

Measured: the cones gone from every eye (`out/mavericks/F_high270.png`); the Newburyport throat's
still and the probe unchanged (the throat's vertices stand deep inside their boxes).

## 13. The crest climbs to the cusp (2026-10-19 late, PR 104's twelfth commit)

Mark: "continue what remains so we can start seeing barrels at the beach ... you might need to make
some adjustments to how we handle the shallow/beach waves."

**Measured first** (Mavericks' bowl, the point's eye, the current fine tiles): the lips were tongues --
5 cm thick at the median, 0.4 m at the 90th; 0.3 m over the face at the median; 2.7 m wide at the
90th. No barrel. The crest law's gain was the limiter's own ratio, lipQ - 1, and at a 7 m reef under a
5 m swell that ratio is 0.2: a crest thrown a metre. Linear theory cannot do better by any ratio of
heights: in shallow water the orbit's steepness is k A_h = H / 2h <= gamma / 2 = 0.3 whatever the
wave does, and the cusp (k A_h = 1, the crest's water at the phase speed) and the loop (pi / 2) are
out of its reach. A breaking wave is not linear: its face steepens because the depth cannot carry
the wave's energy, and that energy carries the crest forward.

**The law.** The share of the raw variance above the limit, 1 - 1/lipQ^2 (the clipped Rayleigh sea's
own measure, the same family as BreakFactor), takes the crest's excursion from the sea's own, A_h,
toward the loop's limit, pi / 2k:

    A' = A_h + (pi / 2k - A_h) (1 - 1 / lipQ^2),    q1 = A' / A_h - 1

on the crest's weight w as before (the orbit's amplitude grows by q1 w, the crest is carried
forward by q1 w A_h). At lipQ = 1 nothing is added; at 1.5 the crest stands at the cusp (k A' = 1);
past 2 it overturns; the bound is never crossed and there is no corner. On both processors
(WaterBank.hlsl, WaveField::ProbeAt with the same Jacobian by the product rule). The spray's share is
now the crest's excursion past the cusp, max(A' - 1/k, 0) w: a loop's water, the plunging lip's.

**The reef's own breaking index.** The solver's limiter held Hs <= 0.6 h everywhere -- a beach's
random-sea saturation, the Merrimack's. On a steep reef single waves stand to 0.8-1.0 of the depth
before they plunge; scenes/demos/mavericks.json says `gammaHs 0.8`. A scene key, not a code change.

**The beach.** Half Moon Bay's beach lies in Pillar Point's lee from the WNW swell that fires the
reef -- the swell march says so and it is true of the place -- so `scenes/demos/mavericks_beach.json`
brings the swell from 245 into the bay (Hs 3 m, Tp 14 s) over a solved field that spans the beach:
a sand bar in 2-5 m of water, a beach break. (The first beach eye stood at the first field box's
edge, where its feather fades the waves; the box was moved so the feather lies on the sand.)

Measured (`out/mavericks/U_*`, `out/surf/lips/I_*`):

**The set wave, the same night.** The cusp law alone still left tongues (1,401 fold texels on ring 0,
lips 0.2 m over the face): the crest weight w saturates at the significant amplitude, so every crest
above it was thrown alike and the set waves -- 1.5 to 2 times the significant crest, the ones that
tube -- got an average crest's throw. The excess that folds THIS crest is the sea's times this
crest's height over the significant amplitude, `eCr = lipQ * eta / aEnv`, and the share is
`1 - 1/eCr^2` (1 below the significant crest's own limit, nothing in the trough). Both processors;
the twin's Jacobian treats eCr as constant at the point.

**The shore stop's twin moved into Prepare.** The hull probe crashed in `HeightPage::Texel` under
`AtMany`'s ParallelFor (and hung an hour the time before): the stop's eight bed reads stood in
Finish, the pure part that runs across the pool, and the page memo's map is not a thing eight
threads may insert into. Prepare, the memoised part on one thread, now reads the dry weight at
eight stations each way along the sea's travel (spaced to reach four throws at the crest law's
largest excursion here), and Finish reads its mean dryness along the actual throw from that
profile -- the same law, the twin's quadrature along the travel.

| | cusp law | + the set wave |
|---|---|---|
| fold texels on ring 0 (the lineup) | 1,401 | 4,498 |
| lip height over the face, p50 / p90 / max | 0.19 / 0.40 / 0.56 m | 0.51 / 1.21 / 1.65 m |
| lip width, p50 / p90 / max | 0.9 / 1.5 / 1.8 m | 1.5 / 2.1 / 2.4 m |
| fine column tiles at the Newburyport throat | 40 | 84 |
| the throat's still, pixels changed | | 239,000 of 1,440,000 |

Not yet a barrel: the lip's top still stands under the crest's back at the median (a thrown tongue,
not an arch), and from the shoulder the set wave reads as a long dark wall with a slab of spray on
its crest -- the spray's spawn now runs on the excursion past the cusp and packs the ring; its
density and its drawing want a law of their own. The beach: Half Moon Bay's beach lies in Pillar
Point's lee from the WNW (the swell march says so and it is true of the place);
`scenes/demos/mavericks_beach.json` brings Hs 3 m from 245 into the bay over a field that spans
the beach, and the bay still looked calm at frame 900 -- untested further tonight.

**What the minute of video showed (Mark's three crops).** Poles: thin vertical slivers from the
water to the top of the frame, a few frames at a time. The fill cleared a fine tile only under the
coarse texels it visited, and a fine tile mapped under a coarse tile the fill never visits (a NULL
tile at the shore whose box an inverted triangle touched) kept whatever its pool page last held; the
lip sheet then ran from the face to the stale word's height, a hundred metres on this encoding. Now
`CsFineClear` sets every mapped fine tile's four bands to absent each frame before the scatter.
Square ends: the lips stopped on the 38 m tile grid, because ColumnFetch's fine taps at a tile's
edge read the neighbour and an unmapped neighbour reads absent; a flagged tile now maps its eight
neighbours with it. The carpet: a parcel of spray per crest texel per frame once the spawn ran on
the excursion past the cusp, drawn as quarter-metre discs; the spawn gain is a volume budget again
(0.25 a frame per metre of excursion per second) and the discs are 0.12 m growing 0.12 m a second at
alpha 0.35. The staircase behind the carpet is the lip sheet showing the 1.2 m label grid edge-on:
a throw of twenty metres across sixteen labels is piecewise planar, and that is the grain of ring 0,
not a fault -- the finer label grid is the next step for the lips' shape.

**The tube's end cap and the spray's streaks (2026-10-20, Mark's next two crops).** The spike at
the end of every roll was the lip sheet's end: a lip band's vertex with no lip stood on the column's
FACE, which under a roof is the trough, so the sheet drew a wall from the trough to the crest where
the tube ended. ColumnFetch now reads the lip's height as the coverage-weighted mean of the lip's own
heights and stands the vertex between the sheet's own surface (the uncut particle, the unbroken
crest) and that height by the coverage: the roof merges into the crest. The dotted arcs were one
texel's parcels, one a frame, on one parabola; a parcel's sprite is now stretched along its velocity
in the sprite's plane by the distance it flies in a twelfth of a second, a jet instead of beads. The
blocky zipper seen from above is the lip sheet's cells on the 1.2 m label grid: each cell's roof is
planar because the scatter rasterizes the cell's two triangles between the four labels' particles
with barycentric heights. The next step for the lips' shape is a cubic scatter -- the sheet's height
inside a cell from the Catmull-Rom of the label grid, as the mesh already reads the bank -- on the
DONE the same night: the fine pass cuts each cell into 4 x 4 sub-cells whose corners stand on the
Catmull-Rom of the 4 x 4 label patch, 32 small planar triangles tiling the curved cell, written only
where a fine tile is mapped (residency is what pays for it, as Mark asked). Measured at the lineup:
the lip top's second difference along x fell 0.030 -> 0.010 m at the median; the scatter 0.30 -> 0.55
ms; the lip's height over the face rose 0.5 -> 2.6 m at the median, which is more tiles holding set
waves (4 -> 16 current) and possibly the cubic's overshoot at a fold -- not yet separated.
fine level alone.

**Recording.** The recorder is fed only on the rail path, so a plain headless run writes an empty
mp4. A scene can now name its own rail file (`rails.file`); `scenes/rails/mavericks_lineup.json`
holds the lineup eye for sixty seconds, and `capture.railDir` with `capture.mp4` records it.

Gate: the hull probe (helm_boat, 30 records) at 9-11 cm rms, correlation 1.00 on every record that
prints one, no crash under the watchdog; the self-test's 37 pass. The twin's bed lap grew 0.3 -> 3.8 ms
a frame for the profile's sixteen reads (memoised after the first pass).

## 14. The column must be a ladder, not two named levels (2026-10-20, Mark's design bar)

Mark: "nothing should be defined as coarse or fine in code I hope. I assume you are being implicit
that you are referring to the mip levels and the nested layers of mega textures." It is not implicit
today, and that is a fault against the bar he set the same night (the sparse voxel water is for global
coverage: tsunamis, an impact's ripples from orbit). The bank's rings ARE a mip ladder; the column's
"finer level" is a second, named atlas (`water.columnF`, `kFine*`, `gColF`, `FineMapped`, `CsFineClear`,
`lvl == 0 / 1` in the scatter, the fine-only branch in ColumnFetch) with its own residency and its own
kernel paths: 65 / 44 / 61 mentions in WaterBank.hlsl, Globe.hlsl and WaterBankLayer.cpp. (The
meshlets' `fine` in GlobeMesh.hlsl is an older, different word: a record within 650 m, the one whose
points come from doubles.)

**The shape it should have.** The column is a ring ladder like the bank's, one atlas, every ring a
mip of the one below it, with the rings' residency nested: a ring is resident where the ring above it
witnessed a fold (the flags, which today are the fine level's alone, become the law between EVERY
pair of rings), and the ladder may go down as far as the sea asks -- 0.3 m at a reef, 2 cm at a
model's edge, never named. The scatter writes every ring whose texel is at or under the cell's grain
where that ring is mapped; the cubic sheet (section 13's addendum) is cut to the grain of the finest
ring the cell reaches, not to "the fine level"; the readers walk the ladder finest-first and take
the first ring that answers; the clear runs over the mapped set of every ring. The lip pass draws
the records of any level, not level 0. Nothing in the code says fine or coarse; a ring knows its
texel and its parent.

DONE the same night (PR 104's next commit). `water.columnF`, `kFine*`, `gColF`, `FineMapped`, `CsFineClear`
and the scatter's `lvl == 0 / 1` are gone. The column is ring 0's atlas (six rings, the bank's) and a
ladder of RINGS UNDER it: ring -(k+1) at ring 0's texel over 2^(k+1), 512 x 2^(k+1) texels a side over
ring 0's span, four bands, its own reserved atlas of 128^2 tiles and its own flags; how deep the ladder
goes is the scene's (`water.bank.columnTexel`, the finest texel it reaches, 0.3 m by default = two
rings under; 0.15 m = three, the API's 16384-texel ceiling). The laws are relative: a fold witnessed by
a triangle of ring r flags the tiles of ring r - 1 its box touches; the CPU maps those tiles and
their neighbours (nested residency between every pair of rings); `CsColumnClear` clears the mapped
tiles of every ring under; the scatter writes a cell's own ring, and for every ring under that is
mapped beneath the cell, the cell's cubic subdivision into n x n sub-cells, n that ring's texels to
ring 0's; `ColumnFetch` walks the rings under finest-first for the lip bands and takes the first that
holds the point; the dump writes `fiber_column_r-k.f32` per ring, masked by its mapped set. Nothing is
named by its grain; a ring knows its texel and its parent. (MsLip's block grid is still a quarter
cell -- the mesh's own grid, not the column's; a third ring under is finer than it.)


## 15. The Lagrangian sheet, drawn whole (2026-10-20, Mark: "switch to drawing the Lagrangian sheet")

Mark's crops of the minute on the ladder, from behind the set wave: gaps along the lip's back, a comb
of hanging slivers along its front, and from the shoulder a dark palisade. Then his question: "I'm
wondering if you might be over complicating your mesh/ribbons." He was right, and the measurements
said where.

**What was measured first.** The lips moved onto the column ladder's own texels (an AS group per tile
of the finest ring, an MS group per 8x8-texel block, each vertex the exact word at a texel centre,
stood by the lattice origin's arithmetic) -- and the palisade stayed. So it was in the DATA: the ring
-2 dump at the peak view holds the top band smooth (1-3 cm a texel, max 0.64 m) and 261 underside
texels 2-8 m under their neighbours, every one at the lip's edge with partial coverage, every one
with the underside equal to the face to the centimetre. At a fold line the texel's centre stands
under no upright arm of the loop, and "the upright branches' max" there is the face. A law closing the
underside onto the top at the fold line took the slats away -- and left a flat shelf. The lip
triangles' curtains to the label's height (the end cap's fallback) were the slivers. Each of these
was the same fault: a folded surface REBUILT from Eulerian height bands does not know which crossing
joins which across a texel, and every artefact of the week (curtains, end caps, square ends, slats,
the shelf) was the rebuild.

**The change.** The water vertex is its label at its particle (label + lateral displacement, level +
vertical): the Lagrangian sheet, which is the folded surface already -- the crest's labels run
backwards and their triangles turn over. The pipeline culls nothing; `MsMain` culls per primitive
(`SV_CullPrimitive`) a triangle that shows the eye its back AND has no water corner, so land culls as
it always did and the sea's sheet is two-sided: the turned-over triangles are the tube, drawn. "Back"
is said in the triangle's own terms -- the displaced triangle turns to the eye the side its undisplaced
self turns away. The pixel water turns the bank's normal to the side of the sheet the eye is on
(the drawn triangle's own normal from rel's screen derivatives), so the tube's ceiling seen from
below is lit as a ceiling. The mesh is already at the label grain where the waves are (`SplitAt`'s
wave-grain rule: level 18, 1.19 m cells within the inner rings), so no new amplification was needed
to draw it.

Gone: `MsLip`/`AsLip` and the lip PSO and pass (the spray keeps its draw after the surface),
`ColumnFetch` and every pixel reader of the column, the lip fragments' thin-sheet radiometry, the
face from the column in `SurfaceVertex`. The column plane and its ladder still run (the spray's death
reads ring 0's face; the dumps); nothing draws from them now.

**The frame (Mark: "the same methods as 3d buildings ... the orientations adjust to the part of the
world they are on").** `ChartPointRel` stands a point of the rings' chart -- the lattice origin under
the eye, the buildings' floating origin -- at its height, eye-relative, from small numbers only: the
horizontal scale (R + alt)/R, the sag s(R + alt)/2R^2 under the chart's plane, and the eye's own
height over that plane carried from doubles (`WaterBankLayer::ChartRows`, row 3 w). The spray stands
on it (it took the flat frame's y as an altitude before, and the sag as zero). Wherever the origin
stands, up is the chart's up.

Measured at Mavericks' peak and lineup: one connected sheet, the overturned part a thin band along
the crest; no slats, no curtains, no gaps along the back. The lip is small: the loop is a few 1.19 m
cells across. Self-test 37.

**Next.** The loop's size on screen is the label grid's: refine the walk below level 18 where the
sea folds (the ladder's residency already says where) and read the bank's displacement on the
Catmull-Rom of the labels (`SheetAt`'s law) so sub-label vertices stand on a smooth sheet. Then
decide what the column ladder is for, now that nothing draws from it (the thickness the shading
could see through; the hull's lip).

## 6. Next

1. The face from the column (band 0), the guard deleted on both processors; `--water-probe`
   against the Lagrangian inversion as the gate.
2. The column's CPU twin on the hull's label patch (the scatter in `TreeWater`), so the boat
   feels the lip.
3. The pixel water's rays and the mirror's land through the column's intervals (an Eulerian
   "is water" test with three crossings).
4. The general `chi` tenant (TileAtlas3D) only when the topology changes -- fed by the column's
   fold, never beside it.
