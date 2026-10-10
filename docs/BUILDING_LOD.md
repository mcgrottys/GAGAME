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

## 4. Where a building lives: size picks the level

Each solid's size is its box's circumscribed radius `ρ = √(a₁² + a₂² + hz²)`. It lives at level

    k = ⌊log₂(ρ / ρ₀)⌋,   ρ₀ = 4 m

with its cell at that level chosen by its centroid (Ulrich). A level-`k` thing covers
`ρ / (d · pixAng)` pixels at distance `d`, so level `k` is wanted out to

    d_k = ρ₀ 2^k / (τ · pixAng)

which is the globe's own `LeafWants` measure (span over distance times pixel angle), not a new
one. With `pixAng ≈ 1e-3` and `τ = 1 px`: level 0 to 4 km, level 6 (ρ ≥ 256 m) to 256 km,
level 8 (ρ ≥ 1 km) to 1000 km, which is near space. Cells at level `k` are `2^k` times the size of
level 0's, so each level holds about the same number of cells in view. The tree is as deep as the
largest structure, and there is no altitude switch: one inequality per level.

## 5. What a hidden building leaves: geometry sheds to statistics

Below a level's threshold a building's box is not drawn, but its `M` stays summed in its cell.
The cell's residual mass (the sum of what is hidden) is the built-up **volume fraction, mean
height and spread**, the same three numbers a sparse water voxel needs for porosity and blockage.
Drawn, it darkens and roughens the ground instead of vanishing. This is the water's fold law
(geometry → σ², energy kept) applied to buildings, and it is why a city does not pop out as you
climb. It is step 4 below, not the first PR.

## 6. On the sparse structure

The level-`k` cells are the cube lattice's tiles (`face, rung, x, y`), the lattice every tenant
shares (HIERARCHY §0, the GPU-resident law), not a second grid of degrees. A tile at a coarser
rung holds the buildings of the coarser size class and the folded `M` of all beneath it, so the
tree is one more tenant. Its parent/child fold is `Σ T M T̃`, its name is a path (HIERARCHY 4.20
law 1), and it has no depth bound but the largest thing on Earth. The 0.05° harvest stays the
level-0 source of full rings near the eye.

## 7. Order of work

1. **The moment algebra** (`compose/BuildingMoments`): prism moments, fold, frame change, CGA
   vector, moment box, level law. Gate: `--selftest [lod]` pins §2's three facts and §3's limits.
   *(this PR, first commit)*
2. **The pyramid tool**: one pass over the planet harvest writes the size-stratified box records
   (≈28 B each) by cube tile, with each tile's folded `M`. Offline, cached, rebuilt when the
   harvest's identity changes.
3. **The far layer**: the globe walk's leaves ask for the levels their `d_k` admits; boxes drawn
   instanced, one draw per tile; a box whose 0.05° detail cell is resident is skipped (no double
   draw). Gate: Tokyo from 9, 27, 100 and 400 km (vertices, ms, stills).
4. **The residual**: the hidden mass as a ground tenant (coverage, height, roughness).

## 8. Open for the owner

- `ρ₀ = 4 m` and `τ = 1 px` are the two numbers the law needs. They are scene keys
  (`layers.buildings.lodRho0`, `lodPixels`), not constants in code.
- The box replaces a building's shape past its detail radius. An outline that matters at distance
  (a stadium's ring, the Pentagon) would need a second far shape. Level-2 rings simplified by the
  existing Visvalingam–Whyatt importance could serve; I left that out.
