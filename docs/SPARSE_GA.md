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

## 26. The normalization stage, and composition as a reverse tree

The pipeline was always stated as five arrows:

    GA Load -> GA Unit/Scale/Projection normalize -> GA Compose -> GA physics -> sparse GPU

The second arrow did not exist. `GeoRef::valueUnit` was written by every loader, logged once at
boot, and read by nothing; `DomainCompositor::Add` said *"a product carries one grade and one
unit"* while checking only grade and channel count. A bed in `m NAVD88` and a survey in `ft MLLW`
would have composed to full coverage and a number that is neither.

`src/core/GaUnits.h` is that arrow. `Quantity` says what a number IS; `UnitSpec` parses a
`valueUnit`, carries a factor to SI and a vertical datum, and `AcceptsFrom` converts across a
scale but refuses across a quantity or a datum.

**Why it is its own stage.** A loader must report its file faithfully, units it cannot convert
included — converting there erases the provenance that makes a refusal explainable, and "this
file is feet" is the useful half of the error. A compositor sees floats, by which point the unit
is already gone. So it sits between them, and `Add()` now enforces both halves it promised —
including refusing a *convertible but unnormalized* source, because a stage that can be skipped
is not a stage.

### The reverse tree

A `DomainCompositor` was a leaf-eater: sources in, page out, nothing downstream could consume a
composed product except the GPU. `CompositeSource` makes a compositor a source. Three things
follow: composites **nest**; the unit check **composes with them**, so a wrong datum four levels
down is caught at the edge that introduced it; and every node declares a **cadence**, which is
the gantt column — the bed composes once, the tide is a function of `t`.

`PrintTree` walks the live graph, so a mis-wired node shows up there before it shows up in pixels.

### The tide step

Depth is not a dataset anyone ships. It is water level minus bed — two quantities, two vertical
datums, two domains, two clocks. A compositor resolves *disagreement about one quantity*;
`BinaryFieldSource` combines *two quantities into a third*, and its coverage multiplies rather
than averages, because a depth is meaningful only where both operands are known.

Subtracting two levelled heights yields a **thickness with no datum**, and the unit algebra
records that — so the result refuses to compose with anything still carrying one.

### The datum rule caught its own author first

`TideSource` originally emitted NAVD88 directly, applying each station's published CO-OPS link and
dropping any station without one. That looked more rigorous and measured **1.7346 m worse**:

    [tide-src] 2 stations usable in NAVD88 (18 dropped: no published NAVD link)

Only 2 of 20 stations carry a link, and **Newburyport — the focus, whose harmonics actually
describe this estuary — is not one of them.** The water level was being interpolated from
Riverside and Boston. The curve came from the wrong place in order to avoid guessing a constant.

So a source reports what its data says, in the frame its data is in, and a missing link is a
reason to make somebody **declare** it rather than to discard a good tide curve.
`DeclareDatumLink` takes the offset and a provenance string and prints both every run. It is
handed main's own `ResolveDatum` result — an estimate that was always there, hand-carried in a
`BathyModel` comment. It is now an edge with a type on both ends.

    [datum] no NAVD link at Newburyport; MLLW - NAVD88 = -1.396 m via NAVD=MSL+0.092 (from Boston)
    [datum-link] tide.mllw: MLLW -> NAVD88, -1.3960 m -- declared, not derived
    [tree] node                       kind       domain  unit                       cadence
    [tree] water.depth                subtract   point   m [length]                 per-instant
    [tree]   +- tide.mllw             normalize  point   length NAVD88              per-instant
    [tree]     +- tide.mllw           load       point   length MLLW                per-instant
    [tree]   +- merrimack             load       raster  m NAVD88                   static
    [tree] MLLW tide - NAVD88 bed: refused, as it must be
    [tree] 24 hourly probes at Newburyport: worst |tree - engine| level 0.0000 m,
           depth 0.0000 m (equivalent)

Equal to the engine's own water level to the printed digit — through a graph that states its
units, its datums, and which of its numbers is an assumption.

## 27. The realization moved into the compositor, and the bed converged

Section 25 left the GA bed **DIVERGENT at 28.3231 m** and named the cause: the engine's bed is
not a file. `BathyModel::RealizeFromChannel` *overwrites every cell* with
`Compositor::SampleHeightStack`, so the authoritative bed is a six-layer composite and the GA
path had exactly one of those layers.

    L0 noaa.etopo2022        4.9 km   global floor
    L1 noaa.etopo15s.ne      461 m    regional
    L2 noaa.cudem.capeann    13.7 m
    L3 noaa.cudem.boston     13.7 m
    L4 noaa.cudem.merrimack  13.7 m   <- the one layer the GA path had
    L5 survey.edits          5 m      hand-edit polygons -- "THE LAW"

`HeightStackSource.h` adapts each existing `HeightSource` into a `DomainSource`. Nothing is
reimplemented — every layer still answers through the same `Sample()` the renderer calls — so
there is no second copy of the resample, the feather, or the edit polygons to drift out of step.
The fix was never "read the same bytes sparsely"; it was "make the thing that edits the bytes a
source".

### The blend rule had to move too

This was the subtler half. `SampleHeightStack` is a sequential **over** in layer order:

    h += (m - h) * w          for each layer, bottom to top

while `DomainCompositor`'s native rule is priority-bucketed weighted **average**. Those agree
only where every weight is 0 or 1 — and disagree *precisely across a feather*, which is where a
survey edit meets the grid beneath it, and where the 28 m lived. Averaging would have pulled the
edit toward the CUDEM under it instead of laying it over. Hence `Blend::LayeredOver`.

### The datum: a real number, not a label

The unit stage refused the stack outright, and it was right to. CUDEM and the edits are NAVD88;
the two ETOPO layers are MSL/geoid referenced. The engine has always blended them as one height.

Labelling that with a zero shift would have been documentation, not a fix. The real number turned
out to be **already on disk** — CO-OPS publishes a station's datums on one staff, and Boston's
cached `datums_metric.json` carries both:

    MSL 2.660    NAVD88 2.752   ->   NAVD88 zero sits 0.092 m ABOVE local MSL

so an MSL height becomes NAVD88 by subtracting 0.092 m. `main::ResolveDatum` already derived this
same quantity from the same data; `NavdAboveMsl` now factors it out, so there is one derivation
and two consumers — the tide's MLLW link and the bed's MSL link.

Its limit is stated because it is real: local MSL is not the global geoid, and the separation
varies across the domain. This is the best published number available short of GEOID18 or VDatum.

### Fidelity and correctness, measured separately

Two questions hide in one number, and answering them together would let each excuse the other.
So the stack is composed **twice**:

    [bed] realization fidelity: worst |GA(link=0) - committed| = 0.0000 m
          (EQUIVALENT -- the six-layer stack is reproduced)
    [bed] GA path: 1863x1174, 6 levels, 6 layers, 2187162/2187162 covered at L0 --
          SAFE TO SWITCH: stack reproduced exactly, and the shipped bed adds the datum link
    [bed] shipped GA bed vs committed: 0.0920 m, all of it the published MSL -> NAVD88 link
          the committed bed omits (it blends MSL-referenced ETOPO as if it were NAVD88)
    [bed] datum sensitivity: 1.000 m of MSL error moves the bed by at most 1.0000 m
          (mean 0.0781 m)

**28.3231 m → 0.0000 m.** With the link at zero the GA path reproduces the engine's bed exactly,
which is the proof that the realization moved. The shipped bed then differs by 0.0920 m, and that
difference is a **finding about the committed bed**, not an error in this path: the engine's bed
carries a 9.2 cm datum error wherever ETOPO shows through.

The sensitivity probe is what makes that claim quantitative rather than rhetorical. Displacing
only the MSL-referenced layers by a whole metre moves the finished bed by at most 1.0 m and by
0.078 m on average — so ETOPO holds full authority somewhere (outside CUDEM's footprint) and is
painted over almost everywhere else. The datum question is settled *where it matters* by
measurement instead of by argument.

Consumers are **not** switched. That is a separate change, with the solver, the sea shader, the
water bank and the globe all reading the bed at once.

## 28. The datum fixed at the source, the switch, and nodata through the chain

### The correction went where every consumer sees it

Section 27 found the engine's bed carried a 9.2 cm error wherever ETOPO showed through.
Correcting it only in the GA adapter would have fixed one bank and left the renderer, the cube,
the window and every other reader of the height stack still wrong — which is *worse* than being
consistently wrong, because then the two disagree.

So `EquirectHeightSource` and `WindowHeightSource` take a `datumShiftM` and apply it in
`Sample()`. `main` derives the link once (`NavdAboveMsl`, from CO-OPS station datums already
cached on disk), logs its provenance, and hands it to the two ETOPO layers:

    [datum] height stack: MSL -> NAVD88 = -0.092 m, from CO-OPS published datums at Boston
            (regional, not GEOID18) -- applied to the ETOPO layers at the source
    L0 noaa.etopo2022    EPSG:4326 equirect, vdatum -0.092 m -> NAVD88
    L1 noaa.etopo15s.ne  EPSG:4326 window (row 0 N), vdatum -0.092 m -> NAVD88

`kComposeVersion` moved 3 → 4: a datum shift is a paint-math change, so every composed height
tile repaints once. The GA adapter then *stopped* applying its own link — with the sources
honest it would have corrected twice — and the bed comparison became direct again at 0.0000 m.

### Nodata, at the texel

Two real bugs, both introduced by the compositing work itself:

**Absence was being written as zero.** `ComposePage` filled uncovered texels with `0.0` — and
zero is a *legal value* for nearly every field it composes. A bed at 0 m is sea level; a current
at 0 m/s is slack water. Absence became indistinguishable from data, with no way to recover the
difference downstream. It now takes a `nodata` parameter, and the bed passes `-9999`,
`BathyModel`'s own sentinel, so both paths agree on what absence looks like.

**Partial coverage was dragging values toward zero.** `LayeredOver` accumulated from zero, which
is compositing over *transparent black*: a texel 30% covered came out as `0.3 × value`. It now
un-premultiplies by accumulated coverage. This does not move the bed — ETOPO covers the globe at
`w=1`, so the divisor is 1 — it matters exactly where the old form was silently wrong.

### And through the chain

`MipReduce.hlsl` gained a coverage-weighted variant. The plain box average is correct only when a
null texel *means* zero. The moment a bank carries coverage, averaging four texels of which two
are absent halves the value and reports it as fact — and the error **compounds upward**, so the
coarsest mips (the pinned floor, which is what a distant sample actually reads) end up the most
corrupted. That is backwards from every other error in the system.

Value channels are now weighted by coverage; coverage itself averages plain, because it is the
fraction of the coarse texel that had data. The velgrad bank declares channel 2 and uses it.

### The switch, done without touching a shader

A paged bank is a `Texture2DArray` and every bed consumer reads a `Texture2D` — but in D3D12
those are the same resource type, so a `TEXTURE2D` SRV over the array views slice 0 and every
consumer works untouched.

That is worth more than tidiness. Switching the bed is the highest-blast-radius change in the
engine — solver, sea shader, churn kernel, water bank, globe — and doing it *without editing a
shader* means that if anything moves on screen, the bank's **content** is the only possible
cause. And its content was already proved equal at 0.0000 m across all 2,187,162 texels.

    [bed] consumers bound to the GA sparse bank (slice 0)

### Recordings report their cost

A rail is the only run long enough and varied enough — orbit to helm, every residency regime in
one take — for frame time to mean anything, and it is exactly the run nobody measures because
they are watching the pictures.

Percentiles, not a mean: a rail crosses regimes deliberately, so an average describes no moment
that ever happened. The worst frame's **index** is reported because that is the half that makes
it actionable — a spike at frame 600 is the 80 km → 7 km descent paging in; a spike at 1100 is
helm-height water detail, and they have different fixes.

The first measurement immediately caught a comment of mine lying. `dt` spans the whole loop and
therefore carries the *previous* frame's PNG encode, which at 1600×900 is tens of milliseconds of
the **recorder's** cost — reporting that as frame time would have slandered the engine. Both
clocks are kept now: `renderMs` brackets `RenderFrame` alone and judges the engine, `dt` judges
the capture, and their difference is printed. The per-frame series lands in `metrics.csv` beside
the frames, because a summary answers *was it smooth* and only the series answers *where did it
stop being smooth*.

## 29. The three trees, and the reference that costs nothing

The pipeline has been stated the same way for months:

    1. INGEST      GeoTIFFs; Google tiles  ->  raw files in a local cache
    2. NORMALIZE   to the body's scale, units, projection
                   ->  cached as its OWN sparse GA tree on disk
    3. COMPOSE     the trees  ->  ONE composite sparse GA tree
                   (tile REFERENCES into the input trees, and genuinely composited tiles
                    cached only where they OVERLAP)

**Stage 2 had no output.** `GaUnits`/`NormalizeToSi` converted at *sample* time and nothing kept
the result; `cache/composed/<channel>/<realization>/` held only the finished blend. Everything
odd about that cache follows from the missing middle.

### What the missing middle costs, counted

A composed tile's filename carries the source-SUBSET hash it was painted from, and
`SubsetSeed` folds in `kComposeVersion`. So a change to the **blend** re-identifies every tile in
the channel. Reconstructing the hashes offline against the cache on this machine:

    cube16k    4340 tiles   271 MB
               2429 = google-only at compose v4
               1792 = google-only at compose v3   <- superseded by a version bump alone
                119 = everything else

97.3% of the global cube has exactly one source in it and cannot be changed by a blend edit, and
41% of it is a repaint that was owed to nothing.

### A source tree is identified by the source

`SourceTree` (`src/compose/SourceTree.h`) stores one source's tiles on the shared addresses. Its
identity is `name | structure | kSourceTreeVersion` and **nothing about the stack above it** —
`kSourceTreeVersion` is deliberately *not* `kComposeVersion`, because coupling them would
reintroduce exactly the orphaning the split exists to stop.

RGB is the source's own bytes. **Alpha is the paint weight**, because the stack's feather lives
in the weight and a tree that dropped it could not be composed afterwards.

A source that declares a tile in its footprint and then does not reach it writes a **zero-byte
`.void` entry** instead of 64 KB of zeros. 1893 of those exist so far: 118 MB of nothing not
written down. Storing zeros to say "nothing here" is the opposite of a sparse tree.

### The reference is computed, not stored

Where exactly one source covers a tile **and claims every texel outright**, the composite *is*
that source's tile: the lerp from black by weight 1 is the identity, and the composite's alpha
(255 wherever anything covers) is the weight the tree already holds. So the composite returns
those bytes and writes nothing.

A marker file per referenced tile would carry no information `ColorSubset` does not already
produce, from declared footprints, in microseconds, with no filesystem contact. So there is one
directory entry saved per reference and, more to the point, nothing on disk that can go stale.

`fullCover` is the test, and it is **computed from the tile**, never assumed from the footprint.
It has to be: the composite starts from black, so a *lone* source at weight 0.5 composites to
half its own colour. A reference over a feather would be a quietly brightened coastline.

### The number

`--tree-audit N` walks the composed cache, keeps every tile whose filename still carries the
subset the compositor would use **today** (a stale one would only measure an old stack), and asks
the tree path for the same address. It strides across the whole folder rather than taking the
first N, because directory order is dominated by the coarse mips — which are exactly the case
that is trivially exact.

    earth.color/cube16k       2454 tiles, 2441 byte-identical, 0.0041% of texels differ,
                              2441 referenced,   13 composed
    earth.color/window z14    2290 tiles, 1393 byte-identical, 1.4353% differ,
                              1304 referenced,  986 composed
    earth.color/window z17    3740 tiles, 1996 byte-identical, 2.4189% differ,
                               729 referenced, 3011 composed
    TOTAL                     8484 tiles, 5830 byte-identical (68.7%), 52.7% referenced,
                              worst |direct - tree| = 1/255 over 139,001,856 texels,
                              0 alpha mismatches

**Not one texel moves by more than a single LSB**, and it is the alpha quantization and nothing
else. The bound is analytic: the composite lerps by `w`, so half an alpha step moves a channel by
at most `|rgba - acc| / 510` — under one LSB, and only where `0 < w < 1`. Every source that
answers 0 or 1 round-trips exactly, which is why the cube (google alone, weight 1) is 99.5%
byte-identical and the windows — feathered ortho, the bed's intertidal alpha ramp — are not.

Through the renderer, with the trees warm:

    globe still (--campos 0,0 --cam 200000,0,0 --frames 30):     0 of 1440000 pixels differ
    helm still  (--campos 120,-10 --cam 7,92.5,-1.5 --frames 6): 2 of 1440000 differ, by 1/255

### What it costs, honestly

1057 MB of trees for the 8484 addresses the incumbent stores in 530 MB of current tiles. **Two
times the bytes where sources overlap** — which is what "composited tiles cached only where they
OVERLAP" *means*, since the overlap regions now hold each input as well as the product. Against
the incumbent's actual on-disk state (1001 MB, 471 MB of it superseded) it is a wash today, and
the asymmetry is in what happens next: a blend edit re-composes; it does not repaint.

### The frame had to be said once first

`CubeColor` and `WindowColor` each carried their own copy of the tile's lat/lon box, the
per-texel lat/lon, and the ground resolution — and the paint loop under them was duplicated
verbatim. Two copies of one geometry is how the globe and the terrain came to disagree about
where the coast was (§ the M6h glitch), and a per-source tree is a *third* caller of exactly that
math. `ColorFrame` is (kind, tile extent, and either a cube face dimension or a Mercator origin
plus zoom base); `Compositor::ColorRealization` is both old functions with a frame filled in.
Both standard stills came out bit-identical across that refactor.

### What this buys

The megatexture is the point. A land/sea mask arriving as a GIS tree and a seafloor arriving as
its own tree compose with these by reading 64 KB and writing 64 KB — no reprojection, no HTTP, no
resample, and no source sampled twice. Composition stops being a thing that repaints the planet
and becomes a thing that reads it.

### Not yet

`--color-trees` is **off by default**. Two things are unfinished and neither is a defect in the
above:

1. **A reference still copies bytes.** `TileLoc` can hand DirectStorage a path, an offset and a
   size, which is exactly what a reference *is* — but `TileArchive::Pack` is hardwired to
   `cache/composed/<channel>/<realization>` and to a filename carrying a subset hash, and a
   source tree has neither. Packing the trees turns every reference into a zero-copy NVMe read.
2. **`TileIndex` does not know about trees.** It scans one realization folder; the composite
   tree's readiness is the union of its inputs' readiness, which is a different question and
   wants a different answer.

## 30. The land/sea gate: the survey gates, the height band refines

`synth.bed` is the colour stack's **water authority** — its height-band alpha reclaims everything
below the intertidal ramp and hands land back to the photos above +1.2 m NAVD. That is a
threshold on elevation, and a threshold on elevation cannot tell an inland hollow below +1.2 m
from the sea. The survey can. The survey, at GSHHG's 1:250k, cannot place a waterline to the
metre. The bed can.

So neither replaces the other, and the user's rule is a **product**:

    bed weight  =  height-band alpha  x  survey water coverage

which is a different composition from the stack's own paint-over. The stack resolves *who is on
top*; a gate resolves *whether a layer is allowed here at all*. `Channel::gateOf` / `gateOnly`
plus `SetColorGate` are that, applied identically in `PaintColorTile` and in
`ColorTreeStack::Compose`, entirely CPU-side at paint time. No shader moved.

### The GIS stays vector

The first cut of this read `data/gis/landmask_ne.raw` — 16 MB of even-odd parity fill, plus 8 MB
global. The user stopped it, and they were right: those files are a **realization** of the
survey, at one resolution, in one frame. `GisStencil.h` has said so since it was written — the
polylines "stay resident as the authority" — and it left a promissory note that GA earns its
keep here because *spherical polygons are chains of great arcs; incidence and clipping are meets
and joins*. `GisMask.cpp` cashes it. Neither `.raw` is opened.

    [gismask] VECTOR land/sea: 1298 coast rings, 26788 NHD open-water rings, 4 hand-edit rings,
              1261898 points -- no .raw parity fill opened

The composition rule was already written down, in `data/gis/survey.json`, as the recipe the
harvester used: GSHHG coast rings are **land**, NHD open-water polygons **carve** water back out
of them, and `edits.geojson` polygons are **law** over both — with marsh and wetland deliberately
excluded because "the live tide owns that call". The same division of labour, one level down.

### Why columns, and where the algebra is real

A point-in-polygon test on a sphere is a crossing count along an arc from the query point. Fill
the grid by **rows** and that arc is a *parallel* — not a great circle — so every crossing is an
approximation in the lon/lat chart, worse the longer the edge and the higher the latitude. Fill
it by **columns** and the arc is a *meridian*, which is a great circle, and the test becomes
exact incidence:

    edge A->B spans the great circle   n = A ^ B          (the join of two points)
    the meridian at longitude L is     m = (-sin L, cos L, 0)
    it crosses iff                     sign(m . A) != sign(m . B)
    and the crossing point is          d = n x m          (the MEET of the two circles)

Four dot products, one cross product, one `asin` per crossing. No chart, no dateline special
case, no `cos(lat)` fudge.

**The index is by longitude and only by longitude.** Parity along a meridian depends on every
edge that crosses it, however far south of the tile — so latitude cannot prune, and must not:

    [gismask] meridian index: 1799 longitude buckets, 91230 coast edges, 1169694 water edges
              (14.7 MB) -- a column touches ~737 edges, not 1260924

### The grids line up, which is the whole compute saving

The mask is a **layer**, so it gets its own sparse tree on the *same* `(face, mip, x, y)`
addresses as the imagery and the bed. The meridian sweep therefore happens **once per address,
ever**. After that the gate is a 64 KB read, and combining the bed with the land textures is a
per-texel byte multiply — no reprojection, no resample, no second frame to reconcile.

That is also why the mask oversamples its sweep only 2×: `Sample` reads only the cells under
texel centres, so anything past the tile's own resolution is discarded work. Nearest, never
interpolated — a gate that blurs is not a gate.

### The composite is keyed by its inputs

`kComposeVersion` re-identified a whole channel when the **blend** changed, and re-identified
nothing when a **source** did. Both halves are wrong, and the second was a live bug: a
re-harvested source tree would have been ignored by an already-cached composite.

A composite tile is a pure function of the input tree tiles it consumed plus the rule that
combined them, so its key is now exactly that — each contributing tree's identity, what that tree
holds at the address, the gate wiring, and a `kBlendVersion` for the rule itself. **It updates
when new tiles appear in a source and at no other time.** `SourceTree::Peek` answers from two
directory lookups, so a warm composite hit costs no tile I/O at all.

### Three bugs, found by looking

1. **The gate silently never entered a single subset.** `GisMaskSource` was constructed *before*
   `Load()`, so it declared the empty box `GisVectorMask` starts with (lon 180..−180),
   `SourceTouches` rejected it against every tile, and the gate was built, wired, logged, and
   never once asked a question. Two A/B renders came back byte-identical and looked like a
   result. What caught it was a tree directory with **zero files in it**.
2. **Absent and empty are opposite answers for a gate.** An all-land mask tile stores as a
   zero-byte `.void`, and `Compose` read absence as *no opinion* — which would have let the bed
   paint over dry land, the one thing the gate exists to stop.
3. **After a source is added there is nothing to compare against**, so `--tree-audit` skipped
   every gated address as stale and warmed none of them. `--warm-trees` composes regardless.

### Measured

Residency-free, tile for tile, the two gated paint loops on the same addresses:

    TOTAL  2602 tiles, 2415 byte-identical (92.8%), worst |direct - tree| = 1/255 over
           42,631,168 texels, 0 alpha mismatches, 92.7% referenced

The remaining tiles are *stale skipped* — the incumbent has not repainted the region yet, and
that asymmetry **is** the result. Adding one source cost the tree path one new tree plus a
recompose of the tiles it touches; it costs the incumbent path a full repaint of every tile the
new source's footprint can reach.

A rendered A/B was misleading twice before it was useful, both times for the same reason: a cold
cache is not a pixel difference. `--color-trees` against the incumbent read as *worst 131/255,
19% of pixels* on empty trees and *0 pixels* once warm; the gated direct path read as *40% of
pixels* while it was still repainting a region the tree path had already built. **Warm both
sides, or use the audit, which compares tiles and cannot be fooled by residency at all.**

## 31. The megatexture as the water's graph

The user, twice in one afternoon: *"we should have done most of this work already for the
water's Sparse GA tree."* Correct. `ComposeTree.h` had already made a compositor a source
(`CompositeSource`); `DomainSource` already carried the tree, the unit stage and the blend; and
`SampleBlended` had already been fixed (§27) to un-premultiply by coverage — the exact property
that lets one composite feed another. §29 built a second graph beside all of it. §31 deletes that
graph and puts the imagery where the water is.

### The one new piece

The water composes one page over the estuary into RAM. Imagery cannot: a page is 16384², and
four of them eight mips deep is a billion queries (§25). So the only thing the water's path lacked
is the **output of every node, on the NVMe, one tile at a time, on the addresses every tree
shares**. `TileTree` (`src/compose/TileTree.h`) is that and nothing else — it has no blend rule
of its own. The kernel is `DomainCompositor::OverStep`/`OverFinish`, factored out so the tile-wise
compose and the point-wise one cannot drift.

### The pipeline, as stated, and where each arrow lands

    INGEST      raw files in a local cache                    the loaders, unchanged
    NORMALIZE   -> each source's OWN sparse tree on disk       TileTree over a leaf
    COMPOSE     -> a third tree of REFERENCES into the inputs  TileTree over a compose node
                   composited tiles only where inputs OVERLAP
    ...the same again for the seafloor, and for the GIS mask
    MEGATEXTURE the three trees -> ONE tree for the atlas       TileTree over the root

    [tree] earth.color                compose    raster  sRGB byte [colour]
    [tree]   +- earth.land            compose
    [tree]     +- google.satellite    normalize / load
    [tree]     +- massgis.coq2023     normalize / load
    [tree]   +- seafloor<gis          gate
    [tree]     +- earth.seafloor      compose
    [tree]       +- synth.bed         normalize / load
    [tree]     +- gis.landsea         normalize / load

Land is the base; the seafloor paints over it with weight = its own height-band alpha × the
survey's water coverage. Over surveyed water the land goes transparent per pixel and the
seafloor shows; inland the seafloor is gated out and the megatexture tile is a **reference** to
the land tree's. The flat incumbent channel stays as the definition the audit compares against.

### The reference is stored

A compose tile that is one input's tile untouched is a zero-byte entry whose *name* carries the
input tree's identity: `f0_m3_x12_y7_<key>.ref-<childId>`. The composite tree on disk therefore
describes itself — an index can be built from its folder alone, and a reference resolves to a
place in the input tree without running the compositor. That is what lets DirectStorage read it
later: a reference *is* `(path, offset, size)` in someone else's archive.

**Straight alpha makes the reference exact with no full-coverage condition.** `OverFinish`
divides by the coverage it just multiplied by, so a single-input compose at partial weight *is*
its input, byte for byte. The lerp-from-black form §29 inherited from the incumbent could not do
this (a lone layer at w=0.5 composites to half its colour) — priors §18.

### Identity is the inputs

A node's identity folds its children's, recursively (`DomainSource::Identity`). A tile's key
folds what each child *holds* at that address — content or void. So a composite updates when a
source's tiles change and at no other time, and `Peek` answers from directory lookups alone.

`GateSource` is a third kind of edge beside compose and subtract: the value is the layer's, only
the weight is the product. *Absent* (outside the gate's footprint: no opinion) and *void* (inside
it, covering nothing: blocks) stay different answers, carried in the key.

### Measured

Tile for tile against the incumbent, before warming:

    cube16k     165 tiles, 165 byte-identical, worst 0/255
    window z14   92 tiles, worst 1/255, 4.37% of texels by one LSB
    window z17  106 tiles, worst 1/255, 3.80%
    TOTAL       363 tiles, worst |direct - tree| = 1/255, 0 cover mismatches

The one-LSB texels are the weight quantized at three nodes (bed, gis, gate) instead of one. The
audit caught two real disagreements first: **27/255 on 567 texels** of one cube tile, because
`MayCover` admitted a sub-texel ortho the incumbent's soak rule drops (priors §16, fixed with
`DomainSource::Footprint` + `Compositor::Touches`); and 6–15% of window texels off by one from
rounding where the reference truncates.

The full warm, 21,508 addresses, is the structure on disk:

    earth.color      11007 ref   3782 composed      74% references
      earth.land     14108 ref    453 composed      97%
      earth.seafloor  3977 ref      0 composed     100% -- a compose over one source is free
      seafloor<gis    2562 void  3781 composed      the gate, void wherever the survey says land

Through the renderer, warm: **globe 0 of 1,440,000 pixels differ; helm 2, by 1/255.** Bench,
three passes: 4.65 / 4.69 / 4.82 ms mean against the incumbent's 4.75 — no render-time cost,
because nothing here touches a shader.

### What went wrong, in order

- The graph and its `TileTree` were block-locals; the audit returned from inside the block, the
  render path left it, and every worker thread dereferenced a freed tree at once. Hoisted.
- The Scriptorium's Release binary — the one `.mcp.json` launches — had never been built, so the
  server could not start in any session. Built, reindexed, verified over stdio. Priors §18 is the
  entry that should have been read first; the memory now says so.

## 32. Ring loads: the instrument first, then the gate, then the picture

The user's frames at t=12 s showed low-resolution tiles popping in around the inlet on the way
down, and a proposal: do not ask for the finest mip a node needs in one go; do one more pass so
that neighbouring tiles never differ by more than a mip, and the view refines as an even spread.
The rule for this repo is instrument before change, so this section is in that order.

### The instrument

`--res-trace` prints, every 30 frames, per texture tenant: the residency **deficit** — tiles in
the queues (wanted and not mapped) by mip — the queue depths, how many entries are speculative
or stale, the ring gate's holds, and **where the load slots went**: a provider that returns in
under 4 ms read a file, one that took longer painted. The picture is made of the deficit; the
slots are why it is what it is.

Two things it said before the gate existed:

- With the trees warm, **paints are rare** — 14–30 of ~2,000 loads on the rail — at ~4.5 ms
  each. The popping is not painting.
- **Reads cost 1.8 ms per 64 KB tile.** That is the per-file open, the cost `TileArchive`'s
  header names as the one that survives every other optimisation, and it is the lever the
  user's rule (painted tiles live on disk; DirectStorage loads them) points at.

The instrument had a bug of its own worth recording: it filtered queue entries by a frame stamp
whose meaning depended on call order, and printed `deficit 0` beside `seen 834`. The queues *are*
the wanted-and-unmapped set by construction; it now counts them and reports staleness separately.

### The gate

`ResidencyManager::Want()` walks each node's column coarsest-first. With `--ring-loads` a **new**
request below the coarsest level is admitted only if its parent is already mapped; otherwise it
is left unstamped and the column stops there, so next frame's walk asks again one ring finer. It
changes the *queue*, not the invariant — coarse-before-fine mapping was always enforced;
requesting was not. On a fast descent the near field wanted mip 0 immediately, the whole column
went into the queue, and the 48 slots filled with finest-mip tiles while the next ring of the
ground the camera was actually over waited behind them.

Nothing is lost: the node walk repeats every frame, and no slot is ever spent on a tile more than
one level from being displayable. A half-written version with a `goto` was reverted rather than
left unmeasured; this one is a flag and a counter.

### The picture, at the same instants

Both arms: warm megatexture trees, the standard rail, one frame dumped per run, and the only
difference the flag.

    frame 540  (t=18 s, start of the 80 km -> 7 km descent)    0 pixels differ
    frame 600  (t=20 s, mid-descent)                       45.6% differ, worst 152/255
    frame 720  (t=24 s, arriving at 7 km)                   0.24% differ, worst 30/255

Frame 600 is the argument. Baseline: the near field — Newburyport, the lower-left land, the
marsh — sits on a coarse ring while a few patches carry their finest mips. Ring: the whole view
resolved one ring at a time and is uniformly sharp at the same instant. By 720 both arms have
converged, so what the gate changes is exactly the transient, which is what was reported.

Over the rail the gate held **11,852** requests; bench mean 4.61 vs 4.64 ms, p99 10.85 vs 11.00
— timing is a wash, as it should be for a change that moves work, not adds it.

### No fallback textures

Also this section, because the user's rule arrived while the instrument was running: *there
should be no fallback textures, only the megatextures.* `TerrainLayer::m_tex` — the committed
CUDEM copy the bed bank was proved equal to at 0.0000 m (§28) — is deleted. The layer's own draw
and the SWE solver read the bank; a bank that fails to build throws at boot instead of degrading,
and `HeightSrv()` returns an invalid index rather than a second copy of the bed. The helm still
is pixel-identical before and after.

What remains off the atlases, from the live grep rather than the old audit: `GulfLayer::m_uvTex`
(currents), `SweSolver::m_uv` (solver state, owed as a Volatile bank), `WeatherManager`'s
per-window bed mirror, `FieldSet`'s PNG fields, and `GisStencil`'s three raster masks — which the
*compositor* no longer needs (the gate is vector) but the shader classifier still samples. Those
are conversions, not fallbacks. `WaveField`'s atlas is already a `GradeBank`.

### The trace, both arms, same rail

    baseline   mean 4.76 ms, p99 11.10, max 13.19   queue peaks: f300 547, f510 700, f540 746, f720 590
    ring       mean 4.72 ms, p99 10.91, max 23.40   queue peaks: f300 495, f510 470, f720  86; held 11918 total

At every sampled instant the queued entries are flagged **predicted**: during the descent the
backlog is the prefetch walk's, the real view's tiles are already resident, and the gate is
throttling speculation as much as demand. That is why the per-mip deficit column read empty
(it excludes speculative entries) -- an instrument finding worth its own follow-up, not chased
here.

**Ring loads are the default** as of this commit — the user's call on the frame-600 pair and the
rail video. `--no-ring-loads` restores the whole-column queue for an A/B.

## 33. The trees packed, and a reference becomes a place

§32's instrument said the rail's loads were 64 KB reads at **1.8 ms each** — the per-file open,
the cost `TileArchive`'s header names as the one that survives every other optimisation. The
user's rule is that painted tiles live on disk and DirectStorage loads them. This section is that
rule for the trees.

### Pack any folder

`TileArchive::PackDir` packs a tree node's frame folder — `f_m_x_y.bin` for a leaf,
`f_m_x_y_<key>.bin` for a compose node; the key lands in the record's `subset`, a leaf's is 0, so
`Find(key, subset)` is one lookup for both. The zero-byte `.void` and `.ref-*` entries are **not**
packed: they cost a directory lookup and never a read, and a reference resolves through the
*child's* archive, which is the point.

    --pack-trees: 28,584 tiles into 21 archives, 1.79 GB, 10.7 s. Loose files kept.

### A tile is a place

`TileTree::Tile` takes an optional `TileLoc`. A tile that lives in an archive is answered as
(path, offset, size) with `out` empty, and the bytes never enter the process. A **stored
reference resolves through the child it names**, so a reference in the megatexture's folder
becomes a `TileLoc` into `earth.land`'s archive, or `google.satellite`'s — the chain ends at
whichever tree actually holds bytes.

A caller that must have the bytes — a parent gathering children, or the upload ring when
DirectStorage is off — reads them through the archive's **one open handle** with a positioned
`ReadFile`, which is thread-safe on a synchronous handle. That alone is the win the instrument
asked for:

    reads on the rail   1.8 ms/tile (loose)  ->  1.2-1.3 ms/tile (archive handle)

### Measured

    globe still, archives vs incumbent          0 pixels
    helm, 6 frames, loose vs archive            0 pixels
    helm, 200 frames, loose vs archive          0 pixels
    helm, --direct-storage vs upload ring       0 pixels   <- the test --direct-storage never had

The 30-frame helm read 43% different between loose and archive on the same tree, and 52%
different between frame 30 and frame 200 on the *same* path: a still that is not settled is a
picture of residency in flight, not of the data (priors §9). Two hundred frames settles it.

`--direct-storage` was left off because the composed-cache path was "not pixel-equal to the
upload ring (9.14% differs) — and equality is not even the right test". Through the trees it *is*
equal, on the same still, so the test exists now; whether to make it the default is a rail
review, as ring loads were.

### One caution recorded

`<windows.h>` in a header that `Compositor.h` includes reached `SeaLayer` and broke `std::min`
through the `min`/`max` macros — guarded with `NOMINMAX` and `#undef` in both tree headers.

**DirectStorage is the default** as of the next commit — the user's call on the pixel-identical
helm still. `--no-direct-storage` restores the upload ring for an A/B. A tile not yet packed
takes the ring until `--pack-trees` runs again; nothing is wrong, it is just slower.

## 34. No insets: one colour page tenant, and the DirectStorage race that hid behind them

*"I swear you are using a separate inset texture in the renderer."* Correct. Three colour
textures — the 16k cube, the z14 window, the z17 detail inset — with hand-off fades in
`Compose.hlsli`. The megatexture tree fed all three, but they were three GPU textures, three
budgets and shader seams: the rung ladder §2 of the handoff had named for deletion.

### One tenant

`AddTexturePages` creates one reserved `Texture2DArray` whose slices are pages of one ladder —
0..5 the cube faces, 6 the z14 page, 7 the z17 page. One SRV, one residency map, one budget, one
provider dispatching on the slice. Slices 0..5 are also viewed as a `TextureCubeArray` (bindless
`space6`) so the globe keeps seamless cube filtering.

`ComposedColorPages` selects by **containment and residency**: every page is the same
megatexture at a different ground resolution, so the page whose resident mip gives the finest
texel at the pixel is the answer, and where two pages are resident at the same resolution they
hold the same pixels. There is nothing to fade between. `hand` and `finer` are gone.

    [residency] earth.color (megatexture pages): 16384x16384 x8 8 mips, 174760 tiles virtual

### The race

The globe then came up green and magenta — only with DirectStorage. Parking one archive at a
time named the bytes: height tiles in colour, then GIS mask tiles, then seafloor tiles, then
*oceans next to mountains* — right bytes, wrong coordinates. Four things, found in that order:

1. `ReleaseCompleted` closed **every** open file when **any** fence completed; reads queued
   behind a later fence read from closed files. Files now travel with their batch's fence.
2. The landing buffer was reset every frame, reusing slot 0 while last frame's slot 0 was still
   waiting on its fence. Slots are a free list.
3. **A drained slot was freed when its `CopyTiles` was recorded, not when it executed.** The
   same frame's `MapAndFill` gave the slot to a new read, DirectStorage wrote it at once on its
   own queue, and the later-executing copy put the new tile's bytes into the old coordinate.
   Slots retire on a 4-frame delay — the overlap discipline the upload ring and eviction already
   keep. This was the one. Every DirectStorage comparison since M9ai had shown 1.8–9% "not
   pixel-equal, and equality is not the right test": it was this bug, not streaming timing.
4. Nothing had ever asked DirectStorage whether a read failed. `RetrieveErrorRecord` is called per
   completed batch. Zero failures here — which is what let the race be found instead of blamed on
   the disk.

Archive payloads are 64 KB-aligned as well (kept; DirectStorage prefers it; it was not the bug).

### Measured

    helm,  400 frames, DirectStorage vs upload ring    0 of 1,440,000 pixels
    globe, 200 frames, DirectStorage vs upload ring    0 of 1,440,000 pixels

### The acceptance image

The user supplied the target: New England from altitude, one uniform resolution across land and
sea, **the seafloor visible through the water with its own relief and colour**, no haze. That is
the composite doing its job — land tree over a *real* seafloor tree, gated by the survey — and it
says plainly that `synth.bed` (a classifier over the height stack) is a placeholder for an
ingested seafloor texture tree. That is the next tree.

## 35. Height pages: one height tenant

Same shape as §34, for the second of the spec's three lines — *1 Sparse Global Height GA*.
`earth.height` was a cube tenant plus a z14 window tenant with a feathered lerp between them;
it is one `AddTexturePages` tenant now, pages 0..5 the cube faces and 6 the z14 page, one
provider dispatching on the slice, `ComposedHeightPages` selecting by containment and residency.

    [residency] earth.height (megatexture pages): 16384x16384 x7 7 mips, 76454 tiles virtual

### What the two-tenant design was hiding

Three readers of the height *window* had bound the second texture directly, and each had to
learn that the window is a slice:

- the globe's data lens and its near-field material gate (`Globe.hlsl`);
- the trace probe's readback, whose subresource is now `slice × mips + mip`;
- **the water bank**, which reads the bed per texel by integer `Load` from the window. Its own
  root signature had no `Texture2DArray` space at all. It has one now, the slice travels in
  `gSlotsD.z` (previously `~0` and unused, so the CB layout is unchanged), and the loads branch
  on it.

### Measured

    globe 200 frames, height pages vs two tenants     0 of 1,440,000 pixels
    helm  400 frames                                  2,148 pixels (0.15%), deterministic,
                                                      all in the horizon band y 428..485

The band is the far shore, where the old path feathered the window into the cube over 6% of its
span and the pages path takes whichever page is resident at the finer texel. Same class of change
as the colour fades' removal in §34, and the same argument: the pages are one height field, so
where both are resident at a resolution they agree, and where they are not, the sharper one is
the truth.

### What is on the GPU now, against the spec's last three lines

    1 global colour GA    earth.color  (megatexture pages, 8)        yes
    1 global height GA    earth.height (megatexture pages, 7)        yes
    N water/weather GAs   the GradeBank arrays                       yes
    strays                GulfLayer::m_uvTex, SweSolver::m_uv, WeatherManager's bed mirror,
                          FieldSet PNGs, GisStencil's three raster masks (shader-side)

## 36. The water simulation reads the height megatexture

The user's requirement, verbatim: *make sure the water simulation is using the height mega
texture for bathymetry for the water shading.* Before this section three different GPU copies of
the bed existed: the height page tenant (read by the globe and the water bank), the bed
`GradeBank` (read by the SWE solver, the sea shader's near field and the churn kernel), and a
per-window bed mirror in `WeatherManager` for owned solvers. All three were the same six-layer
height stack — §28 proved the bank equal to the composed height at 0.0000 m — which is exactly
why two of them could go.

### One bed

The bed bank is not built. The per-window mirror is deleted. Every reader binds **slice 6 of the
height page tenant** — the z14 Mercator page, 9.55 m/px, which is the survey's own resolution
(the bank was 1863×1174 at ~10 m) — with its residency map, and reads at the resident mip:

| reader | before | now |
|---|---|---|
| SWE solver (`Swe.hlsl` `BedAt`) | `gBathy` Texture2D from the bank, lattice-indexed | lattice → lat/lon → page uv (`gGeoLL`, `gWinA`), residency-clamped bilinear from `Texture2DArray` slice 6 |
| sea shader (`Sea.hlsl` `BedAt`) | bank sample inside the survey, `ComposedHeight` outside | `ComposedHeight` everywhere; *surveyed* is the survey's footprint, sampled at lod −8 |
| churn kernel (`SeaChurn.hlsl`) | `SweSolver::BathyRes()` | `PageBedAt(world)` — the water bank's mapping, residency-clamped |
| water bank | already slice 6 | unchanged |
| owned weather solvers (Boston) | `ownedBathyTex` per window | `WeatherManager::SetHeightPage` → the same slice; Boston lies inside the page |

The residency manager now leaves tenants and maps in `PIXEL | NON_PIXEL` shader-resource state:
compute has been reading them all along under a pixel-only state the driver forgave.

### What the bank guaranteed and the page must earn

The bank was pinned wholesale; the page is demand-driven. The solver's lattice is therefore
`Want`ed every frame at mip 0 on slice 6 (~110 tiles of 256×128 over 18.8 × 16 km) so recency
keeps it mapped. The trace probe is the check, because a pixel diff cannot be — any change in the
bed, even the R16F quantization against the old R32F bank, moves the solve and the foam
everywhere:

    frame  60   height.window mip 1  texel (4354,3064): GPU -1.18 m vs CPU stack -1.18 m  MATCH
    frame 200   height.window mip 0  texel (8708,6128): GPU -1.18 m vs CPU stack -1.18 m  MATCH

Under ring loads the domain reaches mip 0 between those two; the value is the stack's at both.
Helm 400 frames run-to-run: 0 pixels. Bench 4.90 ms. Selftest clean.

### Compositing order, as the user stated it

Highest-fidelity tree paints last. `BuildHeightStack` already returns the layers bottom-to-top by
fidelity (ETOPO → NE 15 s → CUDEM → survey edits) and `LayeredOver` paints in that order; when
the height `TileTree` is built (next), its inputs are ordered by ladder depth — the deeper nest
paints over everything it intersects — rather than by hand.

### Still owed

The height page's provider is the compositor's `SampleHeightStack` through `cache/composed`: a
disk cache, but not per-source trees with references. The water/weather planes compose into RAM
at boot. Both are the "disk trees" half of this step and are next; what this section did is
make sure there is only one bed for them to become.

## 37. The height disk tree

The second half of "height and water disk trees". §36 made one bed on the GPU; this makes it
come from per-source trees on the NVMe, the way colour does (§31).

### Fidelity is the sort key

    [height-tree] compose order, coarsest first (the deepest tree paints last):
                  noaa.etopo2022(489200cm) noaa.etopo15s.ne(46100cm) noaa.cudem.capeann(1370cm)
                  noaa.cudem.boston(1370cm) noaa.cudem.merrimack(1370cm) survey.edits(500cm)

The user's rule — the highest-precision tree paints last, over everything it intersects — is
implemented as a sort by declared cm/px, coarsest first, printed every boot. It matches the hand
order `BuildHeightStack` had, and now it cannot drift from it.

### Two scalar tile formats, and why the leaves are float

`TileTree` gains `FloatW` — 256×128 texels of (float value, float weight), 256 KB — for every
leaf and intermediate, and `Half` — 256×128 half values, 64 KB, R16F exactly — for the root,
which always materializes because its tile is what DirectStorage reads.

The first cut stored leaves as half. The audit caught it: 84 of 89 cube tiles exact and a 4 m
worst, all on the NE 15 s ring. The worst-texel dump put both blends within a millimetre in float
(ETOPO −3130 m w 1.000, NE 15 s −3160 m w 0.999 → incumbent −3160.00, tree −3158.00): one
**half-ULP at 3 km depth**, from composing quantized leaves. A leaf tree is the lossless source
(the user's rule 2 and 4); it stores float. With float leaves:

    earth.height/cube16k      89 tiles, 87 exact, 3 of 2,916,352 texels differ, worst 0.250 m
    earth.height/window z14   72 tiles, 56 exact, 52 of 2,359,296 texels differ, worst 0.007 m

That is equivalence to the R16F format.

### A latent difference, recorded

The incumbent height paint is `h += (m − h)·w` from **zero** — the over-transparent-black form
§27 found for the bed. The tree composes straight-alpha. They agree today because ETOPO is global
at w = 1; over a partial base they would not, and the tree is the correct one.

### Still owed: the water/weather planes as trees

`BuildPlaneBank` (Hs, wind, ocean colour, ice, windSrc) composes into RAM at boot. Same
machinery applies — a `TileTree(FloatW)` per plane over its `RasterSource` — with one honest
caveat: these are hourly forecasts, so a disk tree buys boot time, not the repaint economics
that justify it for imagery and bathymetry.

### Measured, warm and packed

    warm    earth.height root 2531 composed (materialized by design); leaves: etopo 2531 content,
            NE 15 s 1140 void, CUDEMs 2208-2480 void, edits 2525 void -- the sparse trees ARE sparse
    pack    earth.height.62f857aa/cube16k.gaa 1320 tiles, window_z14 1372 tiles;
            "archive open, 1320 tiles answer as places" -- DirectStorage reads the height tree
    probe   frame 200, height.window mip 0, GPU -1.18 m vs CPU stack -1.18 m, MATCH -- the
            solver's bed is the tree's, which is the stack's
    helm    400 frames, tree-fed vs composed-cache-fed height page: 860 px (0.06%) -- the
            0.25 m / 7 mm texel differences moving the solve slightly
    globe   200 frames: 12.4% differ, 3.05% by more than 1 LSB -- relief shading under a
            different resident height mip at frame 200 (load order), not content; the probe
            is the content check

## 38. The trees are the default, and the storm rail

`--color-trees` is the default (it feeds the height pages too); `--no-color-trees` restores the
incumbent providers behind the same page tenants for an A/B. Everything on: trees, one page
tenant per channel, DirectStorage, ring loads, the solver on the height megatexture.

    storm rail (--storm 3.0,10,95), default      5.08 / 5.11 ms mean, p99 11.54 / 11.94
    storm rail, --no-color-trees                 4.97 ms mean, p99 10.54
    flood rail, default                          4.91 ms mean, p99 11.84

The storm costs ~0.2 ms over the flood (the sea's own work), and the trees cost ~0.1 ms mean and
~1 ms at p99 against the incumbent providers -- load-side, from resolving stored references
hop by hop and the height root materializing; nothing on the shader side changed. Selftest green.

## 39. The seafloor's texture, and the two bugs it exposed

**The ask**: "do the ingested seafloor texture next", with the acceptance image (Google Earth over
New England: seabed relief visible through the water, uniform resolution). The inventory said
there is nothing to ingest -- `data/` holds bathymetry (ETOPO 2022, the NE 15 s grid, three
CUDEM windows), no photograph of the floor, because none exists. The ocean floor's texture in
every map is a PRODUCT of its bathymetry, and the bathymetry IS ingested: it is the height stack,
the same one the bed classifier already reads. So the source is derived, and says so.

**`synth.seafloor.relief`** (`src/compose/Sources.cpp`, program in `data/bed/seafloor_rules.json`,
authored on first run, never clobbered):

- relief = the gradient of the bed lit by one fixed cartographic sun (az 315, el 45, ambient
  0.45), normalized so a flat bed shades to exactly 1. Central differences, stepped at
  max(texel, the finest data grain under the point) -- `Compositor::HeightGrainM` -- and
  exaggerated x60 (the Gulf's slopes are ~1%; at x25 the painted swing was 8%, invisible lit).
- colour = a dry sediment ramp keyed on DATUM depth: sand 0 m, silt by 200 m, clay by 4 km
  (`SeafloorRamp`, mirrored in HLSL as `SeafloorRampLuma`), times the shade.
- weight = the classifier's waterline band above (full at +0.4 m, off at +1.2 m NAVD), folded
  as coverage at coarse texels (M7s), and NO deep cutoff: every ocean texel is painted.
- footprint global; identity = the rules text + the ramp/gradient version + the height stack's
  signature (as `synth.bed`).

It is a leaf of `earth.seafloor` UNDER `synth.bed` (the classifier keeps its authority inside its
own box by painting over it), and the survey gates it exactly as it gates the classifier:

    google ---+
              +-> earth.land ------------------+
    aerial ---+                                |
    relief --+                                 +-> earth.color (mega)
             +-> earth.seafloor --+            |
    bed -----+                    +-> gate ----+
    gis.landsea (vector) ---------+

Its own tree on disk, its tiles referenced by the composite where nothing else paints -- the
same three-tree process as the land (S29). `--no-seafloor` is the A/B.

**Through the water.** The megatexture's ocean texels are DRY albedo, like the classifier's, so
the water's optics stay the renderer's: Beer-Lambert over the measured K_d, the two-flux
endpoint from chlorophyll and SPM (`optics`) -- hue AND brightness. Where the physical bed term
has died (weight 1 - mean T_w) the floor's hillshade rides the endpoint as a brightness
modulation only (`Compose.hlsli` `SeafloorReliefMod`: divide the ramp's own luminance back out
of the texel, what remains is the shade, flat bed = 1). Two constants, `kSeafloorRelief` (0 =
the endpoint alone) and a contrast gain left at 1. Tried and REJECTED on the user's frame pair:
a brighter "map ocean" that redrew the floor through water of the measured hue at a declared
visibility -- the physical dark water with the relief riding it is the look. What made the
relief legible was the source, not the shader: exaggeration 25 -> 60 (an authored value, so the
tree's identity moved with it).

**Bug 1 -- the chord across the Gulf** (priors 20). The first albedo still had a solid wedge
from the Maine coast to Cape Ann where the relief was gated OUT and Google's blue showed.
`data/gis/coast_ne.bin` holds the coastline CLIPPED to the survey box: 1283 closed rings and 15
open pieces, the mainland one 32,727-point piece from New Jersey (south edge) to Maine (east
edge). The crossing test closed each piece with a chord back to its own start; that chord ran
across the Gulf of Maine and even-odd parity called everything inside it land. Invisible for a
week because nothing gated in deep water until the relief did. Fix (`GisVectorMask::ReadRings`,
`StitchAlongBox`): walk the box boundary counter-clockwise (interior on the left -- the
coastline convention) from each piece's end to the next piece's start, chaining until the chain
closes: 14 pieces -> 9 disjoint land polygons. `--gis-dump` writes the gate as a PGM; the picture
is the proof (water: the Gulf, the Sound, Champlain, the St Lawrence, the NHD lakes; land: the
rest, Cape Cod and the Nova Scotia sliver included).

**Bug 2 -- terraces down the slope** (priors 21). Beyond the NE grid, ETOPO's 4.9 km cells are
sampled bilinearly; a 1 km gradient step inside a cell measured the interpolant's facet, and
the continental slope rendered as a staircase. Derivatives are taken at the data's grain.

**Measured (warm, `--warm-trees` then `--pack-trees`)**: `synth.seafloor.relief` 9308 tiles
painted, 2855 void (land); `earth.seafloor` 4151 composed (where the classifier overlaps it),
2302 references; `earth.color` 5912 composed, 7226 references, 12208 cache hits; the stitched
`gis.landsea` 7079 painted. 34,999 tiles packed across the megatexture tree. Stills: frame 345
(~250 km) lit and albedo, frame 450 (80 km); `out/rail_seafloor_720.mp4` is the flood rail.
Bench not re-run this session (the shader change is one multiply per water pixel; the tree cost
was measured in S38).

## 40. The bed everywhere: one resolver for the kernels, and the row the size gate could not see

**The keyhole (AUDIT_WATER item 5).** The pixel stage has resolved the planet's height with
one rule since section 35 -- `ComposedHeightPages`: the z14 page by containment where its
resident texel is at least as fine as the cube's, else the cube face by direction, the
residency map clamping the mip. The three kernels that simulate on the bed each carried HALF
of it: the page only, with a +100 m wall (SWE), -30 m (churn) or a CPU corner lerp (bank)
outside it. The cube slices were bound to every one of them and never asked.

**One include, any stage.** `shaders/HeightPages.hlsli` is the whole rule with manual bilinear
Loads (ALGEBRA priors 1: a bindless SampleLevel outside the pixel stage returns zero), on the
tenant's full array view (slices 0..5 the cube faces, 6 the page) and its residency-map array:
`HpHeightAt(arr, res, lat, lon, winA, winSlice, pageMipMin)`. Two residency reads decide, one
bilinear is paid. The cube-face inverse of `ComposeCubeDir` is a hand port, so gatest
round-trips 20 000 random directions through it (worst 1-dot ~1e-16). The SWE, churn and bank
views are the WHOLE tenant now (`ArraySize -1`), the page's slice rides a CB row
(`gPageB.x`), and the bank's corner lerp survives only for a bank with no height tenant at
all. Trace probe at the mouth, mip 0: page -9.02 m vs CPU stack -9.02 m, MATCH.

**The SWE currents as a bank (item 4).** `SweSolver::m_uv` was the last dense committed
texture on the per-frame water path. It is a Volatile `GradeBank` now (grade 1, RGBA16F,
one mip, the same wet-tile residency as eta), with a TEXTURE2D view over slice 0 for the
three consumers that read a plain texture, an array UAV pinned to slice 0 for the solve, and
the readback wrapped like eta's. Land tiles are NULL and read the hardware zero, which the
sea already treats as "no solve here" through the valid channel.

**The row the size gate could not see (priors 22).** Wiring the churn found `ChurnCbData`
rotated against `ChurnCb` since M9ar: `geoA, winA` inserted before `sweM` on the C++ side and
appended after `gSweM` on the HLSL side. Same size; every row from `gSweM` on shifted. The
churn read its "solved field on" from the anchor latitude, its current gain from the
longitude (-70.8), and its page frame from the SWE handover ramp, so its bed was -30 m
everywhere and its churn physics ran on nonsense for a week without a visible tell. Fixed by
putting the C++ rows in the shader's order, and `dxtest` now reflects every cbuffer variable
and matches its offset, size and name against the header's rows (nested `ComposedSurfaceCb`
rows carry their member as a prefix, `cs + u == gCsU`); a deliberate same-size rotation fails
with `FAIL cb layout: at 128 B ChurnCb has gSweM but ChurnCbData has pageB`. Two bank rows
and one churn row were renamed on the C++ side to their shader names on the way.

**Measured.** Flood rail, 1200 frames RENDER: 5.05 / 5.11 ms mean (the session's earlier
default 4.86-4.91): the resolver's trig and two residency reads per bank texel per ring, ~0.15
ms. `--selftest` passes (dxtest x5 size + layout, gatest incl. the cube-face inverse); the boot
validates 59 graph edges; `out/rail_bed_720.mp4` is the flood rail on this path.

**What this opens.** A `WeatherManager` window anywhere the tenant has a bed -- which is
everywhere -- and the bank's shoaling, current amplification and depth-limited breaking on
every coast the rings reach. The z14 origin is still repeated in `main.cpp` (item 7's Host
object); Boston's window is still not mip-pinned (item 6).

## 41. The survey as pages, and the pin that belongs to the windows

**The survey as a page tenant (AUDIT_WATER item 2).** The land/sea classifier -- `ComposedLandness`,
the sea's `discard`, the hand-edit override -- read three committed rasters `GisStencil` built
at boot from `landmask_ne.raw` / `landmask_global.raw` / a 4096^2 edit box: the parity fills
the vector mask (section 30) had refused to open. Meanwhile `gis.landsea` already had its own
tree on the shared addresses, painted by the meridian sweep, gating the seafloor. It could gate
but could not be READ: the gate multiplied by the layer's WEIGHT, so land had to be painted as
absence, and a land texel and an unsurveyed texel were the same byte.

Three changes make the tree the classifier's pages:
- **The sweep writes two bytes a cell** -- value (255 water, 0 land) and flags (1 surveyed,
  2 edited). `GisMaskSource` returns weight 1 where surveyed and `(value, value, edited, 255)`,
  weight 0 (no opinion) elsewhere.
- **The gate is a VALUE gate** (`GateSource`): coverage says whether the gate has an opinion,
  the value channel is the factor. Void tiles still block; unsurveyed texels pass; land blocks
  by its value. Identity `gate2|`, so every composite that consumed the old form rehashes.
- **A third page tenant**, `gis.landsea (survey mask pages)` -- R8G8B8A8_UNORM, the colour
  ladder's 8 slices, fed by `TileTree::Find("gis.landsea")->Provider(frame)`, demanded by the
  same CDLOD walk as the colour and the height (cube by node, z14 and z17 by containment,
  floor mips 4..7). `FillComposedCb` puts it in `gCsU3` (array SRV + residency, cube SRV +
  residency). `CsMaskSample` takes the finest page with an opinion (a > 0): z17 where the ~1 m
  edit raster used to answer, z14, the cube face; none -> the height sign, as before where no
  survey existed. Addresses the survey has no opinion about have no tile; the loader marks them
  NULL after its retries and never maps a zero.

`GisStencil` keeps the vectors for the overlay and nothing else; `BuildMasks`, the three
`GpuTexture`s, the edit box and the `.raw` paths are gone. The GA graph names `compose.stack ->
mask.pages -> {globe.ps, sea.ps}`.

**The pin belongs to the windows (item 6).** The solver domain's mip-0 pin lived in `main`,
for the Merrimack alone, inside `mode == 1 && globe`. `WeatherManager::PinDomains` now asks
for every ACTIVE window's lattice on the page it reads, every frame, from the manager that
owns the windows -- Boston included when it wakes, and with or without a globe layer.

**Found by the storm video: the channel was land.** With the classifier reading the sweep for
the first time, the helm stood on drained bed: the z17 mask tiles two and three tiles west of
the mouth were land, surveyed. The harvester's rule for the water carve is "NHD open-water
even-odd PER FEATURE" and the sweep took parity over the UNION of the water rings; where two
water polygons overlap (SeaOcean over the estuary/river polygon at the mouth) the parities
cancel and the channel is land. Since M9ak that same union parity had gated the bed classifier
out of the river without anyone seeing it -- the raster classifier still said water, so the
sea drew water over a Google photo of water. The crossings now carry their ring id
(`CrossingsTagged`), each feature fills its own parity and the features OR (priors 23). The
coast set keeps union parity: GSHHG's hierarchy is nested and XOR is its semantics.

**Found on the way: the flag that ate the next flag.** `--storm --rail-flood DIR --mp4 F`
parsed as storm = "--rail-flood" and a stray DIR: no rail, no frame cap, the encoder pipe
waiting forever on stdin while the sim free-ran at render speed and re-solved the wave field
every 90 sim-seconds -- three "stuck recordings" in a row before the log line `[args] ignoring
'out/storm_rail'` was read. Every flag with a default went through the same `next()`; it no
longer consumes a token that begins with `--`.

**Measured.** Warm after the gate's identity change: `gis.landsea` 7082 tiles painted, 13 void;
`earth.color` 6439 composed, 5996 references; 52 185 tiles packed across the megatexture
tree (the z17 mask page alone 4215 tiles, 264 MB). Boot: `[gis] the survey is page tenant 2`,
`[weather] solver domains pinned ... merrimack (inside the page)`, 60 graph edges validate,
no orphan. `--selftest` passes (dxtest x5 size + layout, gatest). Stills at 80 km (lit and
albedo) and the helm frame render as before: the coastline classifier, the edit override on
the jetties and the sea's discard now come from the pages.

## 42. The churn follows the camera; the swell shadow is a tree node

Two items from the water audit (AUDIT_WATER §4), both about the same thing: a field that
existed only inside one box near Newburyport now exists wherever the camera is.

**The churn (M9az).** The foam-memory atlas was a fixed 16 km box centred on the station:
breaking could not be remembered eight kilometres from the ACT0816 buoy, Boston's window
included. The atlas is unchanged (8192², 2 m texels, sparse, list-driven kernels); its
ADDRESSING is now toroidal on a world-anchored tile lattice -- a world tile (Tx, Ty) lives at
slot (Tx mod NX, Ty mod NY), and the domain is the ±8 km window around the camera whose origin
snaps to a tile multiple exactly as the wave bank's rings do (`ReanchorRing`). Re-anchoring
moves no bytes: content stays where the water is; a slot whose world tile changed hands is
cleared (its bytes belonged to water 16 km away); tiles that leave the window unmap. Consumers
test the window on the origin and sample at `frac(world / domain)` with a WRAP sampler, since
the wrap line runs through the window. The kernel unwraps a slot to its world tile from the
window's origin tile (`gWindow`, appended to `ChurnCb` on both sides). Measured: the helm frame
differs from the fixed-domain build in 296 of 1.44 M pixels by at most 6/255 -- the wrap
sampler at tile bilinear edges, nothing else.

**The swell exposure (M9ba).** The line-of-sight shadow was a 160² CPU raster marched over one
copy of the Merrimack CUDEM and uploaded as a committed texture -- inside one 18 km box the
sea was sheltered, everywhere else the bank's rings whitecapped every harbour. It is a field
over the bed, so it is now a `DomainSource` (`ExposureSource`): the march runs over the height
STACK (whatever is finest under the ray), identity = the stack's signature + the peak direction
bucketed to 5° + the level bucketed to 0.25 m + the program version. A one-input compose root
over it gives a `TileTree` that materializes R16F; the tree is the provider of a fourth page
tenant, `swell.exposure (pages)`, demanded every frame at mip 3 (76 m texels) over ±20 km
around the camera, read by the sea shader and the bank kernel through the same lat/lon →
page frame as the bed (`SweShadow`, the bank's `expo`), residency-clamped, nothing resident =
exposed. When the bucket rolls the tree re-keys under an atomic shared pointer (loader threads
may be inside the old tree's provider) and the tenant DROPS its tiles: residency bytes go to
"nothing" this frame, pool slots are NULL-mapped after the frame-overlap window (priors 19),
loads in flight for the old identity are discarded when they land (`ResidencyManager::Drop`).
The CPU mirror for `--trace` is the node itself.

**What the tree showed that the raster hid.** Marched over the composed stack, the survey
edits make the jetties real +2.5 m walls, and a single line of sight from mid-channel toward an
080° swell crosses the north jetty: the whole channel went to the 0.12 floor, where the old
raster -- marching CUDEM's smeared crests and then box-smoothing a 117 m grid -- leaked. A
single ray cannot carry what actually puts swell in that channel, so the node marches a
cosine-weighted FAN of five rays across the sea's declared directional spread (±26°, the
wave-field spectrum's own) and averages their transmissions. Diffraction is not modelled and
is not claimed; the fan is the spread the spectrum already states. The graph: `sea.peakdir →
exposure.node → {water.bank, sea.ps}`, mercator-uv, no flip; the ledger truth re-pinned.

## 43. The pyramid, in the compositor

The user's rule, stated while the exposure page misbehaved: parent mips are filled by folding
the children DURING composition, not resampled from the source per level, and not as a
separate pass. Until now every mip of every tree was its own paint (M6i's "each LOD averages
the source over its own footprint"): correct where the source can answer at that footprint,
but two independent answers per address, and for a node whose answer is a *march* (the
exposure) two different marches. Now (`TileTree::FoldUp`, tree version 2):

- A leaf's freshly painted tile is folded into its parent's quadrant -- 2x2 box,
  coverage-weighted (straight colour x alpha / sum alpha, alpha = mean; value x weight / sum
  weight, weight = mean), the fold law -- and that parent into its own, up to the frame's
  coarsest mip. A parent that does not exist yet is painted from the source first (as every
  mip always was), then overwritten where children exist. So a coarse texel is the source's
  own resample where nothing finer was ever painted, and exactly the fold of the finer level
  where it was; the two agree by construction wherever both exist.
- Composites are not folded: they compose their children's folded parents through the same
  `OverStep`, so fold(compose) meets compose(fold) up to the over's own nonlinearity. Their
  CACHED tiles at a changed address are dropped (`DropCachedAddress`): a composite's key folds
  what its children hold, not what they say, so a fold below would otherwise never be seen.
- A changed tile is announced upward (`onChanged`); the residency manager queues a per-tile
  refetch (`ResidencyManager::Invalidate`, applied on the main thread with Drop's retire
  rules), so a page tenant holding the old parent takes the folded one on its next Want.
- Parents are read-modify-written by painting threads under a per-tree recursive mutex.
  Archives serve before loose files, so a pack after a warm is still the rule.

**§42 postscript -- what was actually wrong.** The exposure node was right on disk from its
second build, and the flat storm at the helm was never the node: every tenant staged its
residency map at the same tail of the upload ring, and the exposure page (7 slices, like the
height page) inherited the height tenant's map (priors 25). With one region per tenant the page
reads its own residency, samples mip 4 where mip 4 is what it has, and the helm frame differs
from `--no-exposure` by the shadow alone (96 k pixels, the footprint the constant probe
predicted). Everything the probes said along the way was true; the copy between the CPU's map
and the GPU's was the lie.
