# ORIGINS: every floating origin is a point of a parent body's cloud

*2026-10-11, on `claude/osm-roads-feasibility-297c45` (PR 76). The owner's proposal, the night the
sea through a portal came out flat because two readers of one place had charts born by two rules:
"a point cloud of potential floating origins coming from the centre of the earth", a 3D Fibonacci
set, so that any leaf needing an origin takes one by a formula of its position; then "every leaf
should follow the appropriate floating origin point of its nearest parent's cloud"; "the cloud
isn't literal: an equation, indexed and lazily made"; "an equation that allows points to overlap,
the preferred pass-through points going from coarse to finer groups and back"; and "everything in
the engine should inherit this doctrine through the compiled C++ types". This is the record of
what that became.*

## 1. Why

The GPU works in float32. A point 2,000 km from the frame's origin carries a grain of 0.25 m; at
10,000 km (Tokyo from the Merrimack's frame) 1 m. The engine's answer so far was per layer: every
layer subtracted the eye in doubles and cast the difference ("address about anchor"), and the
charts that stand things on the ground -- the root's flat frame, the wave bank's rings, a gate's
destination, a hull's own space -- were each born by a rule of their own: the scene's place, where
the eye stood when it left a 16 km reach, the gate's destination, the hull's position 60 km out.
Two readers of one place could disagree, and at Haulover they did. The CPU keeps doubles end to
end; the cloud exists for what the GPU is handed, and the CPU may use a coarser spacing or none.

## 2. The law (core/OriginLattice.h)

- **A cloud belongs to a parent body**, and it is a dyadic icosahedral lattice: the 20 faces of an
  icosahedron, each subdivided 2^L times along its edges at level L, a point the normalised
  barycentric combination of its face's corners; shells of radius R·exp(k·ε_L) about the body's
  centre, ε_L = (the icosahedron's edge, 1.1072 rad) / 2^L, so a shell's neighbours stand as far
  apart radially as laterally (level 10 at the Earth's surface: 6.9 km, a float grain of 0.58 mm).
  Under rMin the centre is the one origin.
- **It nests.** Every point of level L is a point of level L+1 (its coordinates doubled) and every
  other shell of L+1 is a shell of L. A coarse origin IS a fine origin: a child standing on a
  shared point keeps its parent's origin with no motor, and a group folding up keeps its origin
  where it is shared. These are the pass-through points the owner asked for. (The spherical
  Fibonacci set of the first form does not nest and was replaced the same night.)
- **The identity is level-free.** An origin's `index` is its (face, i, j) at the finest level
  kLmax = 24, canonical on edges and corners (the lowest face owning the point), its `shell` in the
  finest level's units; `Same()` across levels is the geometric identity. Nothing is stored:
  `Direction(index)` is the equation, `InverseIndex(d)` its closed-form inverse (the face by the
  largest dot with its centre, the gnomonic barycentrics rounded to the grid, the nearest of the
  cell's corners), an origin made when asked for, 100 bytes, held by whoever holds it.
- **An origin is a versor**: a tangent frame (east, up, north) on its shell. A point's floats are
  (east, up, north)·(P − c), the flat frame's own convention. Moving floats from origin i to j is
  one motor, M_ij = A_j ~A_i (`Between`), in doubles.
- **The pick has hysteresis**: a point keeps its origin until it has left the cell by a margin
  (`Pick`, `PickSurface` along the ground for charts), so a boundary never flaps and the switch,
  when it comes, is the same motor as any other.
- **A leaf follows its nearest parent's cloud**, the origin a property of a node of the space tree,
  inherited down it. The Earth has a cloud, Mars has one, an inner Droste globe has one at its
  scale, an MRI on an island on a nested Earth would have one of its own at its radius.
- **The doctrine is in the compiled types.** `Space` carries a `CloudStand` (shell, index); the one
  factory `SpaceAt(parent, origin, name)` sets it; `Space::Declare` refuses a child of a body
  without a stand. There is no other way to make a space under a planet, so the bank, the gates,
  the hulls and the root cannot be born by rules of their own again.
- **The same math on both sides**: the CPU computes the origin and the floats in doubles (and may
  use a coarser level or none); the GPU receives the cast. There is no second lattice to learn.

Measured (`--selftest [origin]`, 20 checks): the inverse equals brute force on every point of
level 4 and on 200 directions of level 11, poles and seams included; nesting holds (every level-L
point is a level-L+1 point, a coarse shell a multiple of the fine step); floats stay within the
bound on the surface shell, 100 km up, in orbit at 20,000 km (29 km spacing) and 3,000 km down;
M_ij reproduces a point's floats to 1e-8 m between neighbours, across the planet and across
shells; walking out and back, every switch out lies farther than its twin coming back and the
walk ends where it began; a space without a stand is refused.

Prior art: floating origin (Thorne 2005), origin rebasing (Cesium for Unreal, Outerra), the
icosahedral DGGS family (ISEA, H3's base). A nested lattice of versor origins on log-radius shells
with a level-free identity, a constant-time pick and inheritance down a space tree enforced by the
type system, we have not seen named; it is the combination, and we claim it untested for novelty.

## 3. What stands on the cloud now

| leaf | before | now |
|---|---|---|
| the root world (`SurfaceFrame::About`, the tangent space) | the scene's `place.anchor`, a frame of its own and the one `legacy` space | the cloud's origin nearest the place; the place is a pose in it (`anchorFloats`, `anchorToRoot`), and every hand-written number of the scene -- views, spawns, interests, rails, the built-in sea pose, the root's gate places, the solver window -- comes in through that one motor (`Camera::Carry`, `Entity::CarrySpawn`, `PoseFrame::toRoot`) |
| the wave bank's chart (sets A, B, the second eye) | the root's until the eye left a 16 km reach, then where the eye stood | the surface origin under the eye, by `PickSurface`; two banks at one place share it |
| a gate's destination place | a tangent frame at the place | the cloud's origin nearest the place; the place stands at its floats there, the gate's boxes posed from their places |
| a hull's own space (Entity::Recentre) | a frame at the hull's position, 60 km out | the surface origin under the hull, kept while it stands in its cell; the 60 m threshold stays as the moment a hull leaves the root's frame |
| the buildings' walk | every instance about the walk's eye, a new walk every 0.2 % of height | every instance about the eye's cloud origin; the eye's offset is one float3 in the constants; a walk every 2 % of height |

A lat/lon spelling needs nothing from the move: `FlatOf` is the root's. At Newburyport the scene's
place stands at (−2005.5, −1.03, −3022.4) m in origin 40846856973041664 (42.84551 N, −70.78541 E),
the tangent frame turned 0.03° between the two.

Gates: `--selftest` exits 0 (the scene test's 920 checks restated to the new law at four places);
the Haulover portal and mouth stills overlap the previous commit's to the pixel above the horizon
(the sea below is the solver's window re-gridded about the new origin: same place, new cell
phase); Newburyport's market differs in 0.87 % of pixels, all edge shimmer from re-addressing;
the continent view in 0.09 %; Tokyo under a pinned clock in 0.35 %, the counter's digits.

## 4. What remains

Every layer still subtracts the eye itself before casting. The census of those sites: the frame
loop 45, the globe 11, the minimap 5, the twin-surface tool 5, the renderer 3 (`gEyeRel`, the
camera's root position as a float), the rail 2, the view, the vessel layer, the water and hull
probes 1 each. The move is the one the buildings made: the eye's cloud origin becomes the frame's
per-frame origin, every layer states its floats about it and the constants carry the eye's offset
within the cell. The bank's chart could hold a `Space` of its own rather than an `Origin`. Each
step is gated by its still, layer by layer, with the owner.
