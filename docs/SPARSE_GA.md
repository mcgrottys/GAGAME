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

## 17. The array dimension, plumbed

`TileAtlas2D::Init` takes `arraySlices`. A slice is a **page** of the shared
`(level, x, y)` address space: one resource, one descriptor, up to 1024 slices of 16384^2,
each slice independently mip-chained and independently resident.

That is what reaches centimetres. Coarse levels of a page cover the globe; a page whose mip 0
sits at a survey's resolution covers a patch. Nothing about the SHADER changes between them --
same SRV, same sampler, LOD clamped by the residency map.

**Activation is on demand.** A slice costs its pinned floor the moment it activates, and 1024
floors committed for pages carrying nothing would defeat the purpose. An un-activated slice
has no floor, and `FinestResident` returns `kNothingResident` for it -- an honest "no data
here" rather than naming a level that is silently NULL. Single-slice banks activate slice 0 at
Init, because every existing caller expects one live page.

**A bank with no chain makes no floor promise.** `ActivateSlice` returns immediately when
`MipCount() == 1`: there is no coarser level to fall back to, so there is nothing to pin.
Pinning "the coarsest standard mip" there would map mip 0 in its entirety and quietly make
every flat bank fully resident -- the opposite of the point.

### What the self-test pins

Slices are pages, so the property that matters is **isolation**. Get the subresource index
wrong -- it is `mip + slice * mipCount` -- and nothing errors: tiles still map, dispatches
still run, and the pages simply alias each other. So it is checked directly:

    array: 16 slices, 5 mips (5 standard); every slice starts INACTIVE (slice 0 finest = nothing)
    array: slice3 floor 4 -> 0 after its own map; slice5 unmoved at 4
    array: pages are isolated; slices do not alias

That shape is also the **0-packed-mips** case (5 standard + 0 packed), so the same test covers
the floor bug from section 16: the floor correctly lands on mip 4, the coarsest standard.

Existing banks render **byte-identical** at matched frame counts.

## 18. What the floor costs, and the cap that is actually missing

Measured, because every active page pins a floor and that multiplies:

| page shape | floor | 1024 pages |
|---|---|---|
| 16384^2 R16F, full chain | **1 tile / 64 KB** | 64 MB |
| 16384^2 RGBA16F, full chain | **1 tile / 64 KB** | 64 MB |
| 4096^2 RGBA16F, full chain | **1 tile / 64 KB** | 64 MB |
| 4096^2 RGBA16F, **5 mips (truncated)** | 8 tiles / 512 KB | 512 MB |

**With a full mip chain the floor is one tile, always** -- size and format do not matter,
because the tail packs down to a single tile. Truncating a page's chain is the only thing that
makes the floor expensive, and it is 8x worse. So the rule is simply: **never truncate a
page's chain.** The floor guarantee is otherwise a rounding error.

### The probe that removed the device on every run

The first floor measurement returned five spurious REFUSEDs. The VA sweep immediately before
it ended by requesting 1 TB -- which does not refuse cleanly, it returns
`DXGI_ERROR_DEVICE_REMOVED` -- so every probe after it failed. Re-discovering a known-bad
value on every `--selftest` run took the device down every run and would have masked any real
failure that followed. The ceiling is now RECORDED (512 GB good, 1 TB fatal) instead of
re-measured.

### The real gap: no pool cap

The D3D11.2 sample this engine's ResidencyManager descends from sets a hard budget:

    PoolSizeInTiles = 256          // 16 MB, and it evicts to fit
    MaxTilesLoadedPerFrame = 100

`TileAtlas2D::CommitMappings` has **no such cap**: when `m_freeTiles` runs dry it simply
creates another heap, in both the 2D and 3D atlas. Growth is unbounded and residency policy is
the only thing holding it back. That -- not the floor -- is what needs the sample's
discipline: a pool ceiling, LRU eviction against it, and a per-frame mapping budget (the
manager already has the last one for texture tenants).

Against a 1 GB pool, 1024 pinned floors would be 64 MB -- 6%, leaving the rest for detail.
The floor was never the thing to worry about.

## 19. The pool cap

`TileAtlas2D` and `TileAtlas3D` now carry a pool budget. Until now `CommitMappings` created
another heap whenever it ran dry -- unbounded growth in both, with residency policy the only
thing holding it back.

**Default 2 GB per atlas.** The sample's 256 tiles (16 MB) was a 2013 budget; this GPU has
7.9 GB dedicated. A cap exists to make exhaustion LOUD, not to be stingy. `SetPoolCapBytes`
per bank.

### It refuses; it does not evict

Deliberate, and the important part. A grade bank may be **Volatile** -- churn and foam memory
exist only on the GPU, so evicting one does not cost a reload, it **destroys simulated state**.
Blind LRU would silently delete physics. So the budget is enforced by refusing new mappings and
saying so, the same discipline the meshlet budget already uses: *a dropped leaf is a hole,
reported, never silent.*

Banks whose contents are `Streamable` or `Recomputable` (section 5) can layer eviction on top
later -- for them a refusal really is just a reload deferred. Refusing first is the version that
cannot corrupt anything, and the residence class is what says which banks may go further.

### Pinned

    pool cap: asked 128 tiles, cap 16, resident 16, refused 112
              -- cap held and every refusal counted

The second half of that check matters as much as the first: a cap that silently drops mappings
would look identical to a cap that works, right up until a hole appears in a field and nothing
explains it.

Real renders are byte-identical at the 2 GB default -- no bank is near it.

## 20. Why budgets stay per bank

The question was whether a SHARED pool would disturb how GA products align, and whether
co-locating banks in memory would help. Neither, and the reasoning is worth recording because
it decides the allocator design.

**Alignment is an address-space property, not a memory one.** Bank A's tile at
`(level, x, y)` covers the same ground as bank B's because they share the ADDRESSING scheme
(section 15). A pool decides where bytes physically live; it never decides what coordinate they
represent. Products align identically under any allocator.

**Physical co-location buys nothing.** Tiles are read through the texture unit with swizzled
layout and page-table translation, so adjacency removes no memory transaction, and two
resources are separate cache streams regardless of proximity. It can actively hurt: the memory
controller interleaves across channels, and deliberately concentrating operands narrows that.

So budgets stay **per bank** -- simpler, independent, and no worse.

### What a shared pool would have caught, kept anyway

The sum. Ten banks at 2 GB each is a 20 GB ceiling on a 7.9 GB card, and nothing enforces it.
Rather than couple the banks, the sum is accounted and reported: once tile pools exceed 70% of
dedicated VRAM, the atlas says so once, in as many words -- *budgets are per bank, nothing
enforces the SUM, the next bank to grow may be the one that fails.*

### The coupling that IS real, and is not allocation

If `C = A * B`, then C's tiles are only useful where **both** operands are resident. A refusal
in A wastes whatever B spent on the matching tiles. That is residency CORRELATION, and it
belongs to `DeriveDemand` (section 13) -- the Cayley closure already answers where a product
can be non-zero. An allocator cannot fix it and a shared pool would not have helped.

## 21. The page table: the address space becomes a structure

`src/core/PageTable.h`. The `(level, x, y)` space stops being a convention in these notes and
becomes the thing that answers: **which slice holds level L at page (x, y)?**

**The ladder is the alignment guarantee, in executable form.** `LevelLadder` holds one level-0
resolution per body and halves every level. Every GA tree on that body uses it, so level L
means the same ground resolution everywhere and page (L, x, y) means the same ground. Two trees
asked the same address answer about the same place -- which is what lets sky, water and crust
compose without knowing about each other, and what makes a product of two trees well-defined
tile by tile. A tree computing its own per-level resolution could not be composed, and nothing
would catch the drift until two fields disagreed about where the coast was.

### Globe to centimetre, measured

    pages: 1 cm at L0; one page spans 163.84 m; L18 spans 42950 km (Earth needs L18)
    pages: the 3.2 x 2.0 km inlet at 1 cm = 260 pages of 1024 slices

**19 levels cover the whole range.** A 16384-texel page at 1 cm spans 163.84 m; at level 18 it
spans the planet. And the entire Merrimack window at centimetre resolution is 260 pages -- a
quarter of one reserved array -- against the 6.4e10 texels (128 GB) a flat cm raster over the
same ground would need. That gap is the whole argument for the structure.

### FindCovering degrades, it does not fail

The lookup the renderer wants walks UP the ladder: ask for a fine page that was never reserved
and get the coarser page describing the same ground. That is the CPU half of the no-pop
contract, the counterpart to the pinned floor inside a slice (section 14) -- detail changes,
nothing else does. An address no page covers at any level returns `kNoSlice`, honestly absent
rather than silently level 0.

### Exhaustion refuses

Same rule as the pool cap (section 19), for the same reason: a page may hold state nothing can
rebuild, so the table will not guess which pages are safe to destroy. It refuses, counts, and
says so once. A caller that knows its pages are Streamable or Recomputable can `Release()` a
victim and retry -- the residence class is what decides, and only the caller knows it.

### Not a GPU indirection texture

Resolution happens CPU-side, where residency decisions already live, and the slice index is
handed to the shader like any other bindless index. A GPU-side page table earns its place when
one draw must resolve many pages per pixel; before that it is machinery without a caller.

## 22. The current-field loader, and the rule for 2D sources

`src/core/CurrentFieldLoader.h` -- the second concrete `FieldLoader`, and it exists to show
the seam takes a second file type with nothing above it changing: same registry, same
interface, same three answers.

    loader: noaa.gomofs.current 280x200, EPSG:4326 (embedded, row0 south, centres)
            -> 19 tiles carry data, 1 are pure land (never allocated)

**What the first attempt did wrong.** Getting GoMOFS into the tree the first time did not go
through a loader at all: it reached into GulfLayer's private texture, added a kernel to
`Swe.hlsl`, and wired special-case descriptors into `SweSolver` so one subsystem could feed
one bank. Backed out. A dataset costs a loader and a grade declaration, or the design is not
doing its job.

**The flip is derived, not declared.** `CurrentField::Sample` uses `fy = (lat - lat0)/dlat`
with dlat positive, so row 0 is the SOUTH edge -- the opposite of every bathy grid here.
Nothing types that: `scaleY` is positive and `VNorth()` reads it off the affine, so the source
cannot disagree with its own georeference. That is the whole reason `GeoRef` exists (priors 10,
where a hand-declared flip WAS the bug), and the gate pins it.

**Grade is declared by the source; the algebra decides the rest.** A current is grade 1. What
`grad()` of it becomes -- kG0 | kG2, divergence and vorticity -- is computed in the type by the
Cayley closure. The loader never has to know, and neither does any call site.

### The rule for 2D sources

**Every 2D raster source goes into the global sparse GA tree, and so into the one reserved
array.** Not a dense per-source texture, not a bespoke wire to whoever consumes it. The path is
always the same three steps:

    loader -> GA object (GeoRef + grade + coverage)
           -> page(s) in the shared (level, x, y) tree
           -> composed on coverage weights, like every other source

That is what makes the next dataset cost a loader instead of another pipeline, and it is what
lets two trees be multiplied tile by tile at all -- they align because they are pages of the
same address space, not because anybody matched them up.

Still dense, and therefore still owed this treatment: GulfLayer's uv/Mv2/Okubo-Weiss trio,
GlobeLayer's wave/wind/ice/ocean-colour planes, the SWE uv, TerrainLayer's bed.

## 23. FieldSource: composition past imagery

`src/compose/FieldSource.h`. `Compositor.h` composes sources into a raster and its model is
right -- point query, value, WEIGHT, feather at the edges. What it cannot express is anything
that is not a colour on a 2D mercator tile. So the same model, domain-neutral:

| domain | examples |
|---|---|
| `Point` | buoys, tide/current stations, ADCP -- scattered, no grid |
| `Profile` | a depth cast: one column, many depths |
| `Raster` | GoMOFS currents, the CUDEM bed, a GeoTIFF survey |
| `Volume` | cloud density, water column, crust composition |

A source answers *"your value and your weight at this position"*. **The domain decides only how
it answers, never what the compositor does with the answer** -- which is what lets all four
contribute to one product. `FieldQuery` carries depth and time even for 2D sources, because a
compositor that had to know which fields a source reads could not mix domains at all.

Proven in `--selftest` on a raster plus a scattered point set:

    compose: 2 sources (raster + 1-point), 2631/4096 texels covered

### The rules it had to be held to

**Projection.** The exchange frame is WGS84 lat/lon and a source resolves it into its OWN
projection exactly (`RasterSource::ToSourceCrs`, via `Projections.h`). The first draft applied
the inverse affine straight to lat/lon -- correct only for EPSG:4326, and it would have put a
UTM survey or a 3857 tile somewhere else entirely, silently, because a wrong answer there is
still a plausible number. A CRS the engine cannot evaluate is refused, never guessed.

**Scale.** A page states its own lat/lon extent. The first draft used the anchor-linear form
(`kOrgLat` with a frozen `mPerLon`), which the AST declares valid only NEAR ITS ANCHOR: a
metres-per-degree frozen at 42.8N is 40% wrong at the equator and unbounded at the pole. Fine
for the local water consumers that share it; wrong for a planet-scale tree.

**Units and grade.** A product carries ONE grade and ONE unit. The first source sets the
contract, the rest are held to it, and a mismatch is refused loudly -- averaging two different
fields makes a number nothing downstream can detect:

    scalar.intruder REFUSED: grade 1/2 channels 1/2

**Absence.** A source with no coverage returns weight 0, texels nothing covers keep zero
coverage, and a page with no covered texel is never allocated. The ingest rule survives from
the file all the way to the page.

### Priority vs weight

Highest priority with any coverage wins; equal priorities blend by weight. That is how a 1.5 m
survey beats a 700 m model without either knowing about the other, and why a point set feathers
to its radius instead of stamping a disc.

### Still to build

A `VolumeSource` and a `ProfileSource` (the enum is honest about them existing; only `Point`
and `Raster` are implemented), sparse volume files as a loader type, and the step that takes a
composed page and puts it in the reserved array.

## 24. The loop closed

    [compose] grad(GoMOFS) -> swe.velgrad levels 3..5: 3 composed
              (loader -> source -> page -> bank; the solver never saw it)

The path, end to end, with nothing reaching around anything:

    CurrentFieldLoader   the file, its GeoRef, its grade, its absence
      -> RasterSource    answers value+weight in the WGS84 exchange frame
      -> ComposePage     one page of the shared ladder, with coverage
      -> grad()          on the COMPOSITE, giving grade 0 + grade 2
      -> UploadLevel     texels into the reserved array

`grad` is taken on the composed page, not per source: divergence and vorticity are properties
of the composite, and taking them per-source then blending would average two different
derivatives. `GradeBank::UploadLevel` is deliberately ignorant -- it takes a level and some
texels and does not know whether they came from GoMOFS, a GeoTIFF, a buoy set, or all three.

That ignorance is the whole difference from the backed-out version, which made `SweSolver`
import `GulfLayer`.

### Naming

`Compositor.h` already has a `FieldSource` -- the imagery/tide compositor's two-component
spinor source, same contract shape, narrower scope. The generalization is therefore
`DomainSource` / `DomainCompositor`, named apart rather than shadowing a working class.

### What this does NOT yet do

**The wind fallback is still there, and cannot go yet.** The velgrad bank is ONE page over the
SWE window (18.8 x 16 km). Outside it there is no bank at all, so deleting the fallback would
make everything beyond the window black rather than regional. GoMOFS now fills levels 3-5
INSIDE that window; covering the ground outside needs a second page, which needs the
multi-slice array and the page table driving residency -- both built (sections 17, 21), neither
yet wired to this bank.

So: the pipeline is proven end to end on one page. Making the fallback unnecessary is a matter
of pages, not of mechanism.

## 25. The bed through the GA path, and what the disagreement revealed

`TerrainLayer::BuildBedBank` builds the CUDEM bed the way every 2D source is supposed to
arrive -- GA Load (`GeoGridLoader`), GA Compose (`RasterSource` through `DomainCompositor`),
DirectX sparse structure (a paged `GradeBank`: reserved array, mip chain, pinned floor,
residency map) -- **alongside** the committed texture, and reports the worst disagreement:

    [bed] GA path: 1863x1174, 6 levels, 2187162/2187162 texels covered at L0,
          worst |GA - committed| = 28.3231 m (DIVERGENT -- do not switch consumers)

Nothing switched. The bed feeds the solver, the sea shader, the water bank and the globe, so a
silent half-texel slide there moves a coastline everywhere at once -- which is exactly why the
new path was built beside the old one with a number attached rather than swapped in.

### The 28 m is not a bug in the GA path

    [bathy] realized from the height channel: 395205 moved (152351 by >0.5 m,
            worst 29.1 m: feathers + hand-edit structures)

The GA path reproduces the FILE exactly. The engine's bed is that file **realized against the
composed height channel** -- survey edits and feathering laid over CUDEM -- and 28.3 sits
inside that 29.1. The two paths disagree by precisely the step the GA path does not perform.

**And that step is a composition.** Survey edits over a base grid, resolved by authority, is
what `DomainCompositor` exists to do: the edits are a second source with higher priority, and
the feather is the weight ramp it already implements. Today it runs as a post-process inside
`BathyModel`, which is why a compositor that has never heard of it cannot reproduce the result.

So the finish is not to make the GA path imitate the realization -- it is to move the
realization INTO the compositor as a source, at which point the disagreement should collapse
and the committed texture becomes redundant rather than authoritative.

That is the shape of every remaining conversion in section 22: not "read the same bytes
sparsely", but "make the thing that edits the bytes a source".
