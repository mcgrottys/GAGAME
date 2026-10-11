# ORIGINS: every floating origin is a point of a parent body's cloud

*2026-10-11, on `claude/osm-roads-feasibility-297c45` (PR 76). The owner's proposal, the night the
sea through a portal came out flat because two readers of one place had charts born by two rules:
"a point cloud of potential floating origins coming from the centre of the earth", a 3D Fibonacci
set, so that any leaf needing an origin takes one by a formula of its position; then "every leaf
should follow the appropriate floating origin point of its nearest parent's cloud"; and "the cloud
isn't literal: an equation, indexed and lazily made". This is the record of what that became.*

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

- **A cloud belongs to a parent body**: shells of radius R(1+ε)^k about its centre, k any integer
  with r_k ≥ rMin (under that, the centre is the one origin), each shell carrying n spherical
  Fibonacci directions, ε = sqrt(4π/n) so a shell's neighbours stand as far apart radially as
  laterally. Nothing is stored: `Direction(i)` is the equation, `InverseIndex(d)` its closed-form
  inverse (Keinert, Innmann, Sänger, Stamminger, *Spherical Fibonacci Mapping*, SIGGRAPH Asia 2015),
  and an origin is made when asked for, 100 bytes, held by whoever holds it.
- **An origin is a versor**: a tangent frame (east, up, north) on its shell. A point's floats are
  (east, up, north)·(P − c), the flat frame's own convention. Moving floats from origin i to j is
  one motor, M_ij = A_j ~A_i (`Between`), in doubles.
- **The pick has hysteresis**: a point keeps its origin until it has left the cell by a margin
  (`Pick`, `PickSurface` along the ground for charts), so a boundary never flaps and the switch,
  when it comes, is the same motor as any other.
- **A leaf follows its nearest parent's cloud**, the origin a property of a node of the space tree,
  inherited down it; a node takes a point of its own only when its content outruns the parent
  origin's reach, and the motor between the two is its placement in the parent, which the tree
  carries already. The Earth has a cloud, Mars has one, an inner Droste globe has one at its scale,
  an MRI on an island on a nested Earth would have one of its own at its radius.
- **The same math on both sides**: the CPU computes the origin and the floats in doubles; the GPU
  receives the cast. There is no second lattice for the GPU to learn.

Measured (`--selftest [origin]`, 17 checks): the inverse equals brute force on every point of a
4,096-set and on 200 directions of the 2^22-set, poles and seam included; floats stay within 0.67
of the bound on the surface shell (11.0 km spacing, 0.93 mm grain), 100 km up, in orbit at 20,000
km (46 km, 3.9 mm) and 3,000 km down (5.8 km, 0.5 mm); M_ij reproduces a point's floats to 7e-9 m
between neighbours, across the planet and across shells; walking out two spacings and back, every
switch out lies farther than its twin coming back (the loop 0.19 spacings wide) and the walk ends
where it began.

Prior art: floating origin (Thorne 2005), origin rebasing (Cesium for Unreal, Outerra), the
spherical Fibonacci inverse (Keinert 2015). A lattice of versor origins on shells with a constant-
time pick and inheritance down a space tree, we have not seen named; it is the combination, and we
claim it untested for novelty.

## 3. What stands on the cloud now

| leaf | before | now |
|---|---|---|
| the wave bank's chart (sets A, B, the second eye) | the root's until the eye left a 16 km reach, then where the eye stood | the surface origin under the eye, by `PickSurface`; two banks at one place share it |
| a gate's destination place | a tangent frame at the place | the cloud's origin nearest the place; the place stands at its floats there, the gate's boxes posed from their places |
| a hull's own space (Entity::Recentre) | a frame at the hull's position, 60 km out | the surface origin under the hull, kept while it stands in its cell; the 60 m threshold stays as the moment a hull leaves a frame that is not yet a cloud point |
| the buildings' walk | every instance about the walk's eye, a new walk every 0.2 % of height | every instance about the eye's cloud origin (its shell at the eye's altitude); the eye's offset is one float3 in the constants; a walk every 2 % of height |

Gates: `--selftest` exits 0 (the scene test's 920 checks restated to the new law at four places);
the Haulover mouth still has the far world's sea whole on the lattice chart; Newburyport's market
within 79 pixels of the previous still; the continent view within 2,285; Tokyo under a pinned
clock within 182 pixels of the previous commit, which are the frame-rate counter's digits.

## 4. What remains: the root world

The root flat frame (`SurfaceFrame::About` at the scene's `place.anchor`) is not a cloud point yet,
and every layer still subtracts the eye itself before casting. The census of those sites: the frame
loop 45, the globe 11, the minimap 5, the buildings 5 (now the origin's), the twin-surface tool 5,
the renderer 3 (`gEyeRel`, the camera's root position as a float), the rail 2, the view, the vessel
layer, the water and hull probes 1 each. The move is the one the buildings made: the eye's cloud
origin becomes the frame's per-frame origin, every layer states its floats about it and the
constants carry the eye's offset within the cell; the root's own tangent space becomes a node of
the planet whose origin is the cloud's point nearest the scene's anchor, the scene's declared
coordinates its floats there, as a gate's place already is. The scene's hand-placed views, gates,
spawns and the minimap's rings read through that offset. It is a day's work with the owner, layer
by layer, each gated by its still.
