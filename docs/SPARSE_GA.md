# The Sparse GA Object — design

*Started 2026-08-31. The living design for making sparsity the product and the D3D12 renderer
the demonstration of it. Companion to `ALGEBRA.md` (the mathematics) and `GA_AST.md` (the frame
contract).*

---

## 1. The shape of the thing

The target, in the user's own sketch:

```csharp
SparseGATypeA dataA = new SparseGATypeA(plugin.LoadFileTypeX("filepath1"));
SparseGATypeB dataB = new SparseGATypeB(plugin.LoadFileTypeY("filepath2"));
var dataC = dataA * dataB;
```

Adding a dataset should cost **a loader and a grade declaration** — not a bespoke pipeline, not
a script. The sparse structure is the product; the renderer is how its value is shown.

Two of the three pieces this needs already existed before the work started:

| piece | where | status |
|---|---|---|
| the Cayley closure over grade signatures | `Cl2ProductSignature`, self-test pinned | existed, now `constexpr` |
| per-tile residency from that closure | `ResidencyManager::DeriveDemand` | existed, unused by new fields |
| the frame contract | `GaAst.h` — frame/units/range/flip, validated at boot | existed |
| the typed field + expression algebra | `GradeField.h` | **new** |
| the georeference as evidence | `GeoRef.h` | **new** |
| the loader plugin seam | `FieldLoader.h` | **new** |

## 2. The type IS the algebra

`Cl2ProductSignature` made `constexpr` means the result grade of a product is computed at
compile time. `Field<A> * Field<B>` is a `Field<Cl2ProductSignature(A,B)>` — no declared result
type, no runtime grade check, and a grade that cannot arise is a build error rather than a
silent zero.

```cpp
static_assert(GeometricProductExpr<kG1, kG1>::kSig == (kG0 | kG2));  // v*v = dot + wedge
static_assert(GradExpr<kG1>::kSig == (kG0 | kG2));                   // grad v = div + curl
```

C# would need generic gymnastics for this. C++ gets it free, and the asserts cost nothing at
runtime.

**Expressions are lazy on purpose.** `a * b` records what the product *is*; it does not
dispatch. Eager evaluation would allocate a full atlas per intermediate, which defeats the
entire point — most tiles should not exist. Materialisation (allocate with `policy::Derived`,
run one kernel over the resident list) stays an explicit, fusable step.

**Composition, not inheritance.** What varies between banks is data — dims, format, grade,
policy — not behaviour, and virtual dispatch inside a per-tile loop buys nothing. The one
genuinely polymorphic seam (how a tile is filled, how a file is decoded) is one virtual call
per *tile*, never per texel.

## 3. The ingest rule: nodata is absence

> **Nodata is not a value. It is absence, and absence is a NULL tile.**

This is what reconciles *lossless* with *sparse*, and they are otherwise in tension:

- A dense raster is dense. Storing bathymetry sparsely saves nothing and loses nothing.
- A **survey** (eHydro, a single-beam track, a cloud-free composite) is mostly nodata. A tile
  that is entirely absent must never be allocated — then Tier-2's read-zero *means* "no survey
  here", the consumer adds it unconditionally, and no sentinel (`-9999`, NaN) ever reaches a
  shader disguised as terrain. Lossless, because no real sample was touched.
- A genuinely dense source becomes legitimately sparse as a **deviation from a base**, so zero
  means "the base is exactly right here". The SWE eta plane already reads this way; it is the
  shape the CUDEM bed needs before it can be sparse at all.

## 4. Georeference as evidence, not documentation

`SourceInfo::crs` was free text — it documented the contract but could not be checked, and that
is exactly where this project has been bitten. `GeoRef` narrows it as far as the source allows
and records **how much to trust it**:

| provenance | source | example |
|---|---|---|
| `Embedded` | the file carries it | GeoTIFF ModelPixelScale + ModelTiepoint, NetCDF CF, GRIB2 GDS |
| `Convention` | the protocol fixes it | WMTS/Google tiles *are* EPSG:3857; nothing inside a cached JPEG says so |
| `Declared` | the operator claims it | a bare image plus explicit projection and units |

The two questions that have caused real bugs are now **derived, never typed**:

- `VNorth()` comes from the sign of `scaleY`, so a source cannot disagree with its own affine
  (priors 10 — the flip inversion that was already inside the mercator formula).
- `centers` is a field, because corner-vs-centre is a half-lattice translation no flip rule can
  see (priors 7), and logical dims are stored, never padded ones (priors 2).

A loader **never reprojects**. It reports the CRS it found; the compositor resolves to the
exchange frame with the exact formulas in `Projections.h`. A loader that guessed would be the
one place a silent misalignment could enter.

## 5. Residence class — the third axis

`FIELD` vs `TEXTURE` (null means zero vs null means absent) is not enough once disk is virtual
memory. A bank must also declare what an **eviction costs**:

| class | eviction costs | examples |
|---|---|---|
| `Streamable` | a disk read | imagery, bathymetry, forecast grids |
| `Recomputable` | a solve, from a cache key | the solved wave field |
| `Volatile` | **state loss** | churn/foam memory, anything simulated in place |

This is the axis that makes water genuinely different from terrain, and it decides whether the
residency manager may evict a tile freely or must write it back first.

## 6. The three tiers

Disk is the master; RAM and GPU are shards of it.

```
DISK   the master sparse GA tree, all fields, all levels        (streamable + recomputable)
  |    one tile = one 64 KB DirectStorage request
CPU    a shard: physics, residency decisions, ProbeAt/trace readback
  |    UpdateTileMappings + CopyTiles
GPU    a shard of the shard: 3D volumes + 2D banks + 1D scalar arrays, for rendering
```

**One tile identity across all three** — `(bankId, level, x, y, z)` — or the tiers cannot talk.
The happy alignment worth designing *to*: DirectStorage's natural request is 64 KB, which is
exactly the D3D12 tile size, so one tile is one request end to end.

## 7. Many GA objects per body

A render object owns a **frame**; each GA object owns a **domain** and its grade banks. Earth
carries several at once:

| object | domain | banks |
|---|---|---|
| Sky | atmosphere shell | 3D volume (cloud density, already sparse) + 2D (wind, curl) + scalars |
| Water | surface + column | 2D (level, current, foam, solved wave field) + 3D (chlorophyll/SPM with depth) |
| Earth | crust → core | 3D (composition) + 2D (bed, imagery) + scalars |

## 8. Colour, and where GA is actually load-bearing for light

**RGB is not a multivector.** There is no geometric product of two colours that means anything
physical, and forcing grades onto it would be cargo-cult GA that costs clarity for nothing.
Colour *does* want the identical sparse machinery — streaming, residency, LOD — so it belongs
in a bank as three grade-0 channels, with no pretence.

Where GA genuinely becomes load-bearing for light is **polarization**: the Stokes/coherency
structure is naturally bivector-shaped, and this engine already has a Fresnel split and a
two-ray water model. Polarized sky and sea-surface reflection is both reachable and GA-native —
that is the elegant thing, one layer over from colour.

3D for colour: **no** for satellite surface albedo; **yes** for the water column (a chlorophyll
/SPM profile producing depth-varying colour) and the atmosphere — and both are already separate
GA objects above.

## 9. The paved road (Wolfram-Alpha shaped)

The standard to hold this to: *testing GA algebra here should be as easy as Wolfram Alpha, or
easier — rapid prototyping of maths and physics visuals without dealing with size or sparsity
by hand. No shortcuts; a really well paved road.*

**One algebra, two front-ends** — and the second must use the production machinery, not a demo
path, or it is a shortcut:

| front-end | grades resolved | for |
|---|---|---|
| C++ `Field<Sig>` | compile time, `static_assert` | shipped physics, zero overhead |
| parsed expression | runtime, same Cayley table | prototyping |

Both derive residency through `policy::Derived` and dispatch the same kernels.

**Entry reuses two mechanisms that already work here**: data files hot-reload on mtime
(`wave_scene.json`), and shaders compile at runtime (editing HLSL needs no rebuild). So an
expression in a hot-reloading file becomes a recompiled kernel visible on the next frame,
with no new infrastructure category.

**Grade picks the visualization** — the "you don't choose the plot type" behaviour, and the
generalisation of the existing `--lens` family:

| grade | default view |
|---|---|
| 0 | scalar ramp |
| 1 | LIC / arrows / magnitude+direction |
| 2 | signed diverging ramp (sign = handedness) |
| mixed | split, or a named component |

**Two honest caveats.**

1. A runtime evaluator means generating and compiling HLSL per expression. Runtime DXC is
   already here, so it is work rather than risk — but it is the real work item.
2. Not every operation is pointwise. `grad` needs neighbours, integrals need reductions, and
   **cross-tile stencils need a one-tile apron** in the occupancy list (GAMEPLAN §4.2 hazard 3).
   The expression language must distinguish pointwise / stencil / reduction, because they
   demand different residency. Pretending everything is a map is the shortcut to avoid.

## 10. Milestones

| # | step | status |
|---|---|---|
| 1a | `constexpr` Cayley closure + `GradeField.h` (typed fields, expressions, policies, `GradeBank`) | **done** — gates 7/7 |
| 1b | `GeoRef.h` — provenance, derived flip, the ingest rule | **done** |
| 1c | `FieldLoader.h` — the plugin seam + registry + residence class | **done** |
| 1d | port one existing bank to `GradeBank`, byte-identical render | **done** — churn, byte-identical |
| 2 | `WaveField` → sparse `GradeBank` (the architecture violation that started this) | **done** — byte-identical, 1671/1775 tiles (94%) |
| 3 | TerrainLayer bed → deviation field (null = "the stack is right") | |
| 4 | GlobeLayer planes → banks (low value; consistency once the substrate exists) | |
| 5 | expression materialisation + fusion; apron handling for stencils | |
| 6 | the runtime expression front-end + grade-driven `--lens` | |
| 7 | polarization as a bivector field (where GA earns its keep for light) | |

## 11. Open questions

- The CPU shard's identity: does `ProbeAt` (the trace's 9b/9c twin, the match report) read the
  same tile store as the GPU, or keep a dense mirror? The twin test depends on it.
- Whether the compositor's existing `SourceInfo` should be *replaced* by `GeoRef` or wrap it.
- Grade-shedding LOD (GAMEPLAN application 7) interacts with residence class: a distant tile
  dropping to grade 0 is not an eviction, it is a *different bank* going resident.

## 12. The GPU must not know what is "default"

A correction from the user, and it invalidates how the two-LOD lens is currently written:

> the GPU shouldn't know what is default or not, it just sees a globe scale sparse structure
> that can get to cm levels if we have that data in the sparse structure.

`--lens velgrad` today holds **two SRVs and an if/else** — inlet field where it covers, global
wind elsewhere. That works, and it demonstrated the idea, but it puts source-selection logic in
the shader. The GPU learns there are two sources, and every new source would add a branch.

The right structure is **one bank, mip-chained, with a shader-visible residency map**: the
shader samples the finest resident level and never learns provenance. Fine levels are filled by
surveys, coarse levels by globals, and "default" stops being a category the renderer can see —
it is just the level that happens to be resident there.

This already exists in this codebase, for the *other* tenant class. `Residency.h`: texture
tenants carry "a shader-visible RESIDENCY MAP (R8 cube, byte = finest-resident mip * 16) and
samplers CLAMP their LOD to what is resident: misses degrade to blur, never to garbage." Field
banks are flat and have no mip chain, which is why the lens needed the branch.

So the structural item is: **give grade banks the mip chain and residency map that texture
tenants already have.** The branch in the lens then deletes itself, and cm-scale data is
simply a finer resident level rather than a new code path.

## 13. The closure is a bound, not a policy

Measured when `DeriveDemand` was wired to actually drive the wind bank:

    wind Mv2 residency DRIVEN: algebra 36, physics 36, resident 36 of 36 tiles

Both agree, and both say *everything*. At 6 tiles across a planet one tile spans 60 degrees, and
wind is non-zero somewhere in every one of them.

The lesson generalises: **the Cayley closure gives a CONSERVATIVE bound** — "cannot be non-zero
outside here" — not a tight one. It cannot know that a 3 m/s breeze is beneath notice, because
that is a physics judgement, not an algebraic one. So the two compose as
`And(algebra, physics)`: algebra proves the outside is empty, physics decides which of the
inside is worth carrying. Either alone is wrong.

And sparsity at a coarse lattice is illusory. The wind bank is 2 MB virtual over 6x6 tiles; it
was never going to be sparse, and the "calm air stays NULL" comment had been aspirational since
M6d. Sparsity needs a signature lattice fine enough to express it.

## 14. The chain, and binding banks to a body

**Terminology, corrected.** In D3D12 the object is a **Reserved Resource**
(`CreateReservedResource`); "tiled resources" survives only as the feature-tier name
(`D3D12_TILED_RESOURCES_TIER`). The resource is reserved; the tiles are tiles.

`TileAtlas2D` now takes `mipLevels`. One reserved resource, one SRV, one shader path at every
altitude — **zoom changes which tiles are resident, never which code runs.** No LOD in the
traditional sense, because there is nothing to pop: no shader variant, no second buffer, no
branch.

**The packed tail is pinned.** D3D12 packs every mip small enough to share tiles into a single
tail. That tail is a handful of tiles and it is the coarsest description of the whole field, so
it is mapped once at Init and never evicted. This is the floor that makes a miss impossible: a
sample finding nothing finer still lands on real data. Proved cold in `--selftest`, with
nothing requested at all —

    chain: cold floor -- every region resolves, coarsest 5 of 5 (the pinned tail)
    chain: mapping one mip-0 tile moved its own region 5 -> 0, far region unchanged at 5

**The residency map** is R8, one texel per mip-0 tile, value = finest resident mip there.
Samplers clamp LOD to it, so a region streaming in simply gets sharper. It is the same scheme
`ResidencyManager` already runs for texture tenants; field banks just never had it.

### Binding to a body

A render object (sphere/cube planet) owns a **frame**; the banks bound to it share one
parameterisation so lookups line up without per-bank math:

| bank | addressed by | carries |
|---|---|---|
| 3D | `(face, u, v, radial)` | fills the volume — atmosphere shell, crust to core |
| 2D | `(face, u, v)` | the surface — colour, bed, water |
| 1D | `(radial)` or per-face scalar | profiles, scale, per-shell constants |

The cube-sphere gives the shared `(face, u, v)`; the 3D bank simply adds a radial axis, so a
volume lookup and a surface lookup at the same `(face, u, v)` refer to the same column. That
alignment is the requirement — the object binds a *set* of banks and every lookup shares
coordinates.

Multiple GA objects per body (Sky / Water / Earth, §7) then compose without interfering, and
multiple bodies — each with its own frame and bank set — is the same structure one level up.
Nested simulations follow from that, not from anything new.

## 15. How big can one bank be, measured

Asserted first, measured second, and the assertion was half right in a way that mattered.
The probe lives in `--selftest` permanently (it costs nothing -- no tile is mapped):

    16384 x 16384    created   (Earth: 2441.41 m/texel)
    32768 x 32768    REFUSED
    largest accepted: 16384

**A reserved resource buys a huge virtual MEMORY space, not a huge virtual EXTENT.** The
sparsity is in what is committed; `D3D12_RESOURCE_DESC` dimension limits still apply and
`CreateReservedResource` refuses past 16384 exactly as a committed texture would.

**But that is not a wall, and treating it as one was the error.** The cap bounds one
resource at one LEVEL. A tree is levels, each level is backed by however many resources it
needs, and fine levels are sparse by premise -- so they need very few. Alignment is what
makes this safe: level L in tree A and level L in tree B describe the same ground, and level
L+1 nests exactly inside level L, so nothing drifts.

With level 0 = Earth in one 16384 texture (2441 m/texel), 1 cm is about 18 levels down, and
one resource at level 18 covers ~152 m of ground. The whole planet at that level is absurd;
only regions with data are ever instantiated.

The bounds, ranked by whether they actually bind:

| bound | value | binding? |
|---|---|---|
| resource extent | 16384^2 per level | no -- levels are separate resources |
| descriptors | 4096 self-imposed; ~1e6 on Resource Binding Tier 3 | no -- one constant |
| **data volume** | 128 GB for 1 cm over the 3.2 x 2 km inlet | **yes, the only real one** |

Centimetre data exists in PATCHES -- a survey strip, a dock -- and a patch fits in one
resource. Bindless (`gTex[]`) is already here, so per-(level, region) resources need no new
binding machinery. A physical tile pool with a page table is therefore an OPTIMISATION
(fewer resources, tighter packing, less descriptor churn), not a prerequisite.

### What all trees share

Not a texture, and not an extent: the **`(level, x, y)` address space**. Level N means the
same ground resolution on every tree and every body. Each tree owns its own resources and
residency; they align by construction, globe to camera.

## 16. Texture arrays are the default; VA is the budget

Corrected against the MSDN docs and a clean re-probe, after a first probe that took the
device down.

**What the docs say.** `D3D11_REQ_TEXTURE2D_ARRAY_AXIS_DIMENSION` is 2048, and
Texture2DArray tiling is explicit -- *"for a texture array, each mip level at a given array
slice is a subresource"*. They also warn, in the same paragraph as the dimension limits, that
*"exhaustion of GPU virtual address space, memory residency budget, and or system memory may
easily occur first"*.

**What actually happened.** The first array probe reported a 512-slice cap. That was wrong:
16384^2 x 2048 x 2B is a TERABYTE of virtual address space, and asking for it returned
`DXGI_ERROR_DEVICE_REMOVED` -- the driver did not refuse cleanly, it fell over, and every
result after that point was contaminated. The sweep now stops at the first refusal.

**Measured, clean:**

    array tiling: 4096^2 x8 slices x5 mips RGBA16F -> 21824 tiles, 40 subresources, tile 128x64
      per slice: 5 standard mips, 0 packed, 0 tiles for the tail
      slice0 mip0 32x64 tiles, slice1 mip0 32x64 tiles: per-slice tiling CONFIRMED

      16 GB virtual (32 slices)    created
     256 GB virtual (512 slices)   created
     512 GB virtual (1024 slices)  created
    1024 GB virtual (2048 slices)  REFUSED
    -> virtual address space is the budget, not the slice count: 512 GB in one reserved array

So **arrays are the right default**: one resource, one descriptor, 1024 slices of 16384^2,
each slice independently tiled and independently resident. The slice axis is the scale, and
the thing to budget is total virtual BYTES (width x height x slices x bpp), not slices.

### The bug this exposed

That probe line -- `0 packed, 0 tiles for the tail` -- is a shape with **no packed tail at
all**, because its coarsest mip is still larger than one tile. `FinestResident` had been
returning `mipCount - 1` unconditionally as the floor, on the assumption that a pinned tail
always covers. On such a shape it would name a level that is NULL, and the whole
"degrade to blur, never garbage" guarantee would be a lie for exactly the shapes that look
most reasonable.

Fixed: when `NumPackedMips == 0`, the coarsest STANDARD mip is pinned instead, and
`m_pinnedFloor` records which level the floor actually is. Same guarantee, same argument,
now true for every shape rather than the ones that happened to be tested.
