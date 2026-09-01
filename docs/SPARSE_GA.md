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
