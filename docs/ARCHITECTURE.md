# ARCHITECTURE — the engine after M12

*The refactor's own document, written as the steps landed (M12, 2026-09-13 to 14). The plan it executes is the
approved M12 plan (spaces as versors, one lattice, scenes as files, a paved road under the
layers); the doctrine it serves is unchanged: ATLAS.md (planes, monasteries, the bible, the
Vatican), the five pipeline stages, the GPU-resident law, priors 32 (a space declares its unit
length), and "which algebra, not which matrix".*

## 0. The shape, in one paragraph

A **scene** is a graph of **spaces**. A space has a unit length and sits in its parent by one
**placement** — a similarity versor: rigid for the tangent frame and the boats, scaled for the
Droste link, composed by the geometric product, inverted by the reverse. Everything positional is
resolved by ONE fold along the parent chain with the identity at the root. A raster realization
sits on a **lattice** (the sample grid and the projection that pinned it flat), and a node
**inherits** its lattice from its parent unless it declares its own. The **app** is a shell of
four units in construction order — `Options`, the `Tools`, the `Assembly`, the `FrameLoop` — whose
members are the locals `main()` used to hold, in the order it declared them, because declaration
order is destruction order and that order is load-bearing. The GPU sees three kinds of tenant
(one colour, one height, N water parameters) on one lattice, through a HAL that is a paved road
over Direct3D 12 and nothing more. A demo is a scene file; an instrument is a tool.

## 1. The app shell (`src/app/`) — steps 1a–1d

| unit | file | what it owns |
|---|---|---|
| `Options` | `app/Options.{h,cpp}` | the command line as a struct (`ParseArgs` verbatim from `main.cpp`) and, since step 5, the SHIM: `ToSets` writes a legacy flag line into the scene document `--print-scene` prints. Only the pure instruments and the tools' own arguments are still read from it. |
| `Scene` | `app/Scene.{h,cpp}` | the resolved scene, typed (step 5d): what `main` resolves once and the boot reads instead of the flags. |
| `FramePipe` | `app/FramePipe.h` | the rail recorder: frames straight into ffmpeg over a pipe. |
| tools | `app/Tools.h`, `app/Tools/*.cpp` | the one-shot modes, one file each, bodies verbatim, parameters = the `main()` locals each touched, each call at its original point in the boot. Since step 5d they dispatch by name from the scene's `tools` list (`--tool name[:args]`, or the legacy flag). |
| `Assembly` | `app/Assembly.{h,cpp}` | the scene as one object: the tide model, window, device, renderer, data models, compositor and sources, water atlas, solver, layers, residency manager, trees, tenants, streaming index — 91 members in `main()`'s declaration order, built by `Assemble(opt, scene)` from the resolved scene. |
| `FrameLoop` | `app/FrameLoop.{h,cpp}` | the session (cameras, the one frame, clocks, weather, the wave field — and, since step 5e, the entity, portal, rail and effect nodes it drives), the instrumentation, the loop (`Frame()`), the post-loop tools and the shutdown sequence (`Finish()`). |

**The lifetime law.** `main()`'s locals were safe only because its frame never moved: five
`TileTree::onChanged` closures capture `&resMgr` and a tenant id by reference and run on loader
threads; the `RegisterField` adapters capture `&swe`, `sea`, `globe`; `megaKeep`/`megaTree` are
declared beside the tenants "so they cannot die first"; the wave-prefill job captures four
locals by reference. So each unit is heap-allocated once and never moved, its members are
declared in exactly `main()`'s order, and the explicit shutdown sequence (`gpu.WaitIdle`,
`resMgr.Shutdown`, `sceneWatch.Stop`, `renderer.Shutdown`, `gpu.Shutdown`, `window.Destroy`,
the thread report, `Threads().Shutdown()`) runs verbatim before anything destructs.

**The alias technique.** Each unit's body opens with one reference per member (`auto& model =
A->model;`) and then runs `main()`'s code unchanged; a lambda that captures `[&resMgr]` captures
the object the reference is bound to (C++14, CWG 2011), so the closures outlive the function that
made them. This is how three thousand lines moved with a diff-based proof that every removed line
reappears verbatim, and why each move was gated bit-identical (or at the measured run-to-run
floor) on six settled stills and the 300-frame storm rail.

**One instrument change** rode with 1d: the `--settle-exact` hold now also requires that no
wave prefill is in flight. Measured before the fix: two runs of one binary at the globe pose
dumped with different wave-field mapped sets, because the exact hold could end before the
background prefill swapped its tree; the bird pose's two-state churn had the same cause.

## 2. The frame calculus (`src/core/`) — step 2

### `TileAddress.h`
`TileRequest{face, mip, x, y}`, `TileLoc` (a tile as a PLACE for DirectStorage) and
`TileProviderFn`, moved out of the residency manager: an address is not a Direct3D concept.

### `Lattice.h` — the frame, said once (was `ColorFrame`)
A raster is a plane with a frame, a metric, a lattice and a fiber (ATLAS §3). `Lattice` is the
lattice and its projection: the quad-sphere cube of `faceDim` texels per face, or a Web-Mercator
window of tiles from an origin pixel at zoom `zBase`. `GroundRes`, `Box`, `Texel` and `Tag` are
the M9aj bodies verbatim, and `Tag()` — THE CACHE IDENTITY — reads no new member, ever. Added
beside them are the declarations a frame used to carry nowhere: `Crs()`, `VNorth()`, `MaxMip()`,
`Addr/Request` (the bridge to `PageAddr`), `Ladder()` (the once-dead `LevelLadder`, reachable
from the live lattice), `SameGround()`, `Rows()` (the constant-buffer row every kernel repeated by
hand) and `PxOf()`. `using ColorFrame = Lattice;` keeps every call site as it was.

### Inheritance — step 2d
`DomainSource::OwnLattice()` returns the lattice a node declares, or `nullptr` to inherit: the
identity element. `TileTree::Resolve(inherited)` is the fold's one step, applied at every public
entry point (`Peek`, `Tile`, `Provider`, `Prefill`); the resolved lattice is what the children
receive, so a grandchild inherits the resolved value and "first declaration walking up" falls out
of composition. The tenant binds the root's lattice (`SetLattice`; `Provider()` with no argument).
A child on a lattice of its own is refused where bytes are combined — the compose gather and the
gate node — with a once-per-node log naming both lattices: composing across lattices needs a
resample node ("resampling is integration against the dual cell"), and the tree will not invent
one. No shipped node declares its own lattice yet, so no byte moved; the Mercator pages of the
page tenants become the first declared overrides when the tenant facade lands (step 3e).

### `Space.h` — placements as versors, and the fold
```
Placement   x_parent = t + s R x_own       s (scale), r[4] (a ROTOR, Pga.h's real-part layout), t[3]
            Identity, Frame(rows), Rigid(motor), Similar(p, s, axis, twist)
            Then, Inverse, Pow(k), Apply, ApplyDir, PullPlane, Rows, ToMotor, FixedPoint
            Versor(L)   the conformal versor T(t/L) R D(s), built on demand
            ToMatrix    THE rasterizer boundary -- the only 4x4, derived, never stored or composed
Space       name, unitM, extentM, parent, link (own -> parent), anchor (the world.flat chart)
            Declare()   priors 32: refuses an extent past the conformal collapse (|x|/L >= 9.49e7)
            ToRoot()    THE fold: parent->ToRoot().Then(link), Identity at the root
            To(other), Embed(x), Cycle(name, within, S), Level(k) = link.Pow(k)
```
**The contract, exactly** (sharpened after an outside review): a `Placement` supports rigid
motion and positive uniform scale — a *similarity* — and nothing else; nonuniform scale, shear and
the projection morphs are other contracts with other types. Its one authoritative representation
is the rotor + scale + translation; the conformal versor and the 4×4 are derived execution forms.
Invariants: the rotor is unit (`Normalize()` re-unitizes a long product, the law of
`Motor::Normalize`), "inverse is reverse" holds only under it, `s > 0`, and `Pow(k)` takes the
principal branch (angle in (−π, π]). The rotation is stored in the motor's own layout and composed
with the motor's own `QMul`/`QRotate`, so the file adds no second convention and
`RunPgaSelfTest`'s orientation pins carry over. Scale enters only as the dilator inside a
similarity (the Droste link is `Placement::Similar`, the same numbers as `droste::Portal`); a
change of unit is NOT a placement — metres are metres in every space, the unit length only
conditions the conformal embedding, and no physical size, velocity or mass changes when a space
declares a different unit. `Pow(k)` is the one-parameter subgroup: the fixed-point form when
`(I − sR)` inverts (the logarithmic spirals of the dive), the screw through `Motor::Log/Exp` when
the map is rigid — the algebra's own case split. **Precision:** `To(other)` evaluates through the
nearest common ancestor (the fold from it down each branch, one inverted), never root-relative-
then-cancel, so two boats 40 m apart under a tangent frame a planet radius from the root see
exactly 40 m; `extentM / unitM` guards the embedding, the LCA rule guards the doubles.

**The portal shares the algebra, not plain traversal.** The ownership tree is acyclic. The
Droste tower is `Space::Cycle`: a space whose placement is the portal's `S` and whose CONTENT is
the root again — an explicit reference the walk expands under a traversal context (the
accumulated placement, the recursion level, the visibility and work budget), exactly as
`GlobeLayer::WalkLevel` does today. `Level(k)` is the k-th expansion, and the gauge identity
`Sᵏ(y) − C = sᵏQᵏ(y − S⁻ᵏ(C))` is what makes each expansion drawable; the screen decides when the
walk stops. "Infinite recursion" means a finitely described scene with unbounded expansion,
evaluated to a finite visible result.

**Parity.** The engine's planet frame is left-handed (priors 34): ECEF → planet is a
reflection, which a rotor cannot write. A placement carries parity as the sign of `s` — any
improper orthogonal map is (−I) times a rotation, `Frame()` factors a left-handed triple so,
directions flip with it, the versor carries the Euclidean pseudoscalar as an odd factor, two
reflections compose to a proper placement, and a fractional power of a reflection (no principal
branch) is refused. So the contract reads: rigid motion, uniform scale and parity.

**The gradient's contract** (`core/GradeField.h`, the expression layer that stays above the
HAL): `GradExpr::kSig` is `Cl2ProductSignature(kG1, A)` — the vector derivative is a vector
times the field — so a scalar's gradient is a vector, a vector's is scalar + bivector, a
bivector's is a vector; the old form got six of seven signatures wrong and nothing shipped had
evaluated it. A signature is a BOUND on the possible grades, not a promise of a non-zero.

**Gate.** `spacetest` (`RunSpaceSelfTest`, 1001 checks at step 2 and 1598 after step 4d-2, sixth in `--selftest`): the group and
the fold; `Rigid` acts exactly as the motor; `Similar` equals the portal's closed forms at every
power; `Versor(R)` equals `droste::SimilarityVersor` coefficient by coefficient; the gauge
identity; the plane transport in both forms (GlobeLayer's `n' = Qᵀn, d' = d/σ` and the general
preimage); the unit-length refusal; `Frame()` rows round-trip; the screw power equals
`Motor::Slerp`; `To()` through the nearest common ancestor; `Normalize`; the left-handed planet
frame as an improper placement. It was seen to FAIL on a planted dropped-scale defect before it
was trusted.

**A freshness finding for step 5** (the review, confirmed in the source): `TreeWater::SlowAt`
queries the weather manager every hull step, but the solver's CPU mirror is refreshed only by
`RefreshMirrorsTo`, whose callers are four tools — in ordinary play the hull reads the analytic
tide and the waves and never the solved level or current. The physics consumer needs a coherent
snapshot with a declared simulation time and a fallback policy; that contract lands with
`Entity::Update` and a declared mirror cadence, not with a readback per frame. It landed in step 5e,
default off and measured (§4).

### `SurfaceFrame` — step 4a (the one surface begins)

`src/compose/SurfaceFrame.h` is what the shipped surface IS, declared once: the planet's radius,
the tangent frame's rows the session derives from the anchor, the lattices every realization
sits on (the 16k quad-sphere, the Merrimack z14 window and its z17 detail window in the
colour's tiling, the same cube and window in the height's 256×128 tiling), the tenants realized
on them as ids and page slices read off the tenant declarations, and the stencil flag.
`Merrimack()` writes the z14 and z17 origins down and no other line outside the tests does;
`Fill()` is the old `FillComposedCb` body moved, the merc row from `Lattice::Rows()` bit for
bit; the frame loop and the globe both fill through it, and the fingerprint step 0 planted at
both fills is the gate that proved the collapse was motion. One thing it found: the Mars branch
handed the globe a height cube with no window page, and the shader's window path turns on from
the window id alone, so the frame says there is no window where no slice is declared.

4b took the kernels' hand-filled rows to the lattice: the water bank's, the churn's and the
solver's window row are `Lattice::Rows()` on the window they are handed (the old four casts bit
for bit), the bank's and the churn's world.flat row is `SurfaceFrame::flat` (a `Space::Anchor`
set from BathyModel's four constants) through `FlatRows()`, and the wave field aligns through
`Lattice::PxOf` after a standalone harness ran the old Mercator closed forms against the
lattice's on 44.6 million inputs, bitwise identical. The instrument it added stays: a
`[kernel] <name> cb FNV-1a` line per constant-buffer fill, so every later change to a kernel's
constants is a sequence compare, not a pixel guess.

4c registers the AST's compose edges from the declarations: `SurfaceFrame::RegisterEdges()`
walks the tenants' `astNode`/`astField` and their bindings' lattices, `Lattice::AstFrame()`
says which frame a lattice IS (a cube is the cube.face frame, a window the mercator.px frame
whose rows grow south), and the flip flag is `ast::NeedsFlip` on the two frames, the one rule
GeoRef states, which the validator uses too. The hand table is gone, the window origin it
spelled with it; the regenerated `docs/GA_AST.md` differs only in the range and code columns
of those four rows, and a gatest block pins the shipped lattices' frames against the hand
frames field by field.

4d declared the Droste tower in the calculus (a planet space, the tangent space placed at the
anchor by the frame rows, `Space::Cycle("droste.leaf", tangent, Placement::Similar(...))` from
the same portal `BuildPortal` builds) and then MEASURED before switching: every read the
renderer makes of the portal was evaluated from `Level(k)` beside it and compared bit for bit.
Only the scale agreed. The camera differed by 7e-16 relative, because the portal evaluates
about its fixed point while a placement's power is applied about the origin and cancels two
terms of 1e21 m; the rotation by 9e-16, a rotor sandwich against Rodrigues; the plane normals
by a few ulps. The space self-test had pinned these at 1e-9 to 1e-12, never bitwise, and the
instrument said so. So the reads stayed the portal's, the comparison stayed in the code as the
record, and the switch is its own commit: the placement gains a power applied about its fixed
point, the gauge form the plan already names, and the ulp-level rest is measured on the
stills. Found beside it: the near clip plane the walk transports is NaN on every level (the
reversed-Z infinite projection's row normalises by zero), so the near-plane cull has never
fired; flagged for its own change.

4d-2 gave the placement its power applied about the fixed point (`PowApply`: p + sᵏQᵏ(x − p),
the gauge form) and switched the renderer's Droste reads to it. The largest camera component is
now bit-identical to the portal's at every level; what remains is the rotor's rounding of the
rotation's near-zero entries times the eye's distance in the level's own units, 1e-15 relative
and far below the float constant buffer's resolution there, and the pictures agree (the droste
still inside its floor, the dive rail md5-identical). The space self-test now prints how close
the two closed forms are: within 33 ulps. Making it smaller would mean Rodrigues inside the
placement, the habit the calculus replaces; the record says what the rotor costs, and the
picture says it costs nothing.

4e gave every consumer of a page tenant one contract, `shaders/PageSample.hlsli`: the window
uv of a direction, the residency map's floor, the compute-safe load, the pixel-stage sample
with the floor as min-LOD, and the cube-versus-page rule; Compose, HeightPages and WaterBank
call into it. The kernels' lat/lon spelling of the uv was measured against the direction's
first, in float32 in the shaders' own operation order, and found 10 to 13 percent of texels
apart by up to three eighths of a texel, so it stays its own function with the numbers in the
banner. 4g made the surface constant buffer one object: filled once a frame by the frame loop,
pushed once and bound on the shared layout's `b2` by the renderer for every layer, the four
embedded copies and their setters gone, the globe's sky constants moved to `b3`; the bytes the
renderer pushes and the bytes each layer would have uploaded hashed identical before the copies
were deleted. Two findings: under one-water only the globe's copy was ever read by the GPU, and
the frame loop's fill had run once per session while the globe's ran per frame. 4f, last and
alone, replaces the three ground-resolution literals with the lattice's exact values through
that buffer, with the page winning at equal resolution; measured on the owner's decision.

## 3. The HAL (`src/hal/`) — step 3

DX12-first and DX12-only: a convenience layer that makes the layers' lives easier, not a
portability layer. Everything that names a Direct3D interface moves under `src/hal/`
(`tools/hal_lint.py` enforces it and was seen to fail on a planted line). Five facades sized from
the 1,273 raw call sites: `CommandContext` (tracked barriers, dispatch/draw/mesh, typed
constants), the pipeline builders (one line each instead of the nine copied PSO blocks), typed
views, upload/readback scopes, and `hal::Tenant` — **the sparse default**: a GPU-resident field
is a fiber (format, grade signature, unit, conservative/perceptual law) plus a lattice per slice
(inherited unless overridden), and residency, the page table, mapping batches, null-tile-reads-
zero, the packed-mip floor and the DirectStorage fill come with it. `hal::Committed(..., why)` is
the named exception. The tiled-resource contracts are stated in the facade's banner, not hidden.

**As built so far.** 3a moved the eight Direct3D files (`Gpu`, `Residency`, `TileAtlas`,
`TileStream`, `GpuProfiler`, `PixEvents`, `Shader`, `DxTest`) under `src/hal/` with `git mv` and
rewrote 55 include lines — nothing else moved. 3b put `hal::CommandContext` (`src/hal/Context.h`)
in `FrameContext` in place of the raw list: a thin, non-virtual wrapper that says which recording
it belongs to (`Owner::Frame` or `Owner::Upload`) and offers the operations the layers actually
repeat — `Barrier` on a tracked texture or an untracked resource, `UavBarrier`, `Pipeline`,
`GraphicsRoot/ComputeRoot`, `GraphicsConstants(slot, pod)` (push + root CBV in one),
`GraphicsBindless`, `Draw`, `DrawFullscreen`, `Dispatch`, `Targets`, `ClearColor/ClearDepth`,
`Viewport`, `Copy`. `Native()` is the raw list, named so every use is a countable call site: after
3b the layers, the renderer, the FFT, the solver and the grade-field fill hold 179 facade sites
and 48 `Native()` sites (the mesh pipeline's `QueryInterface`, `CopyTiles`, `CopyTextureRegion`,
`SetDescriptorHeaps` on upload lists, root 32-bit constants, one three-barrier `ResourceBarrier`,
the profiler and PIX scopes). The hal-internal APIs (`ProcessQueues`, `TileAtlas::BuildMips`,
`GpuProfiler`, PIX) keep a raw parameter until their own sub-step. `tools/hal_lint.py` counts
271 → 199 Direct3D lines outside `src/hal/` after 3b; the remainder is root signatures, pipeline
descs, DXGI formats and descriptor descs — 3c and 3d.

3c replaced all 34 hand-written pipeline blocks with `src/hal/Pipeline.h`: `GraphicsPipelineDesc`
carries the fields the thirteen graphics blocks actually varied (blend, cull, depth
clip/test/write/func, fill, topology, the tessellation stages, the target formats) with the
sky's block as the defaults; `BuildGraphics`, `BuildGraphicsRaw` (take `ToDesc()`, edit one
field, build — the globe's wire and meshlet variants) and `BuildCompute` create and log;
`Reload` is the swap-on-success rule every layer wrote by hand, `ReloadSet` the same rule for
pipelines that must agree, `Require` the boot policy. The gate came first: one probe run
compared every builder-produced D3D struct bitwise with the struct the layer used to spell out.
26 of 34 were equal; eight differed only in fields the hardware never reads (a depth comparison
under a disabled test, blend factors under disabled blending), because the old blocks disagreed
among themselves about dead fields — so no single law reproduces all of them bitwise, and the
pipeline the hardware sees is identical everywhere. That is the shape of a 3c finding: reported,
not absorbed into the defaults. hal_lint 199 → 147.

3d found that no layer makes a view that stands alone: every one of the twenty-five hand-built
SRV/UAV descs outside the HAL was a slot of one of seven kernel tables, so `src/hal/Views.h`
makes the TABLE the object (`hal::Table::Alloc` + one creator per view shape per slot, `Base()`
for the bind; `Rtv`/`Dsv` over the device's target heaps) with the desc law one function per
shape carrying every field an old site set. `src/hal/Root.h` makes a root signature a layout
said in binding order (`Cbv`, `Srv`, `Constants`, `Table` of SRV/UAV ranges, static samplers;
version 1.1; the running-sum offset law with unbounded ranges at 0 — the bindless law: one
heap, one index, every dimensionality), the renderer's shared graphics layout a named builder
built once. `src/hal/Resources.h` is `hal::Committed`, the named exception to sparse, logging a
`why` per creation. The gate: 421 EQUAL lines over two probe runs (view descs, serialized
root-signature blobs, committed descs), and the final binary's probe frame md5-identical to the
old code's. Findings, reported not absorbed: the ocean's mip-reduce root signature was the one
version-1.0 signature in the tree; two dead descriptor writes; the water bank's two static
samplers are unlike the other nine and dead (its shaders declare no sampler). hal_lint
147 → 110; what remains is headers and `Layer::Init`'s signature, the mesh stream, the solver's
three-UAV barrier.

3e made the sparse default a declaration. `hal::Tenant` (`src/hal/Tenant.h`) takes a
`TenantDesc`: the fiber (format, tile shape, what one texel holds in words), the semantics (a
NULL tile means zero, a field, or absent, a texture), the residence (streamed, recomputed,
volatile), the absence (zero, unloaded, out of domain, coarse, and the bytes an unbound slice
answers) and the bindings (a slice range, its lattice, its provider or the holder it reads).
`Sparse()` registers it and builds the one dispatcher; `Bind(tree)` hangs the one `onChanged`,
which maps a tree's lattice tag to its slice by the tag the lattice itself prints. The five
hand-written page dispatchers and five closures became five declarations, printed at boot as
`[tenant]` lines: the GPU-resident law read off the log. The gate was the per-tenant
`[settle-exact]` mapped set at the helm, equal tenant by tenant with matching hashes, before
and after. Two things the declaration had to know: the holder is bound before registration,
because the residency manager loads every slice's coarsest tile synchronously inside
`AddTexturePages`; and the wave tenant binds no tree, because it never had an `onChanged`.
`src/hal/Retire.h` states temporal ownership as one fence-keyed queue drained by
`Gpu::BeginFrame`; nothing is routed through it yet, because the manager's three deferred
releases are keyed by turn count and the exact settle counts on that number. The grade bank
moved to `src/hal/GradeBank.h` byte for byte; the expression layer stayed in
`src/core/FieldExpr.h`; `DeriveDemand` now refuses operands on different ground, a contract
the shipped operands already meet. hal_lint 110 → 107.

3f closed the rule: `tools/hal_lint.py` passes with an empty allowlist, and its self-check
plants a violation into the real walk and reports it caught. What a layer holds from Direct3D
keeps the D3D type under a hal spelling (`hal::RootSignature`, `Pso`, `Resource` and their
owning `Ref` forms: typedefs, not wrappers); the globe's mesh-shader pipeline stream lives in
`Pipeline.h` (`MeshPipelineDesc`, `BuildMesh`) and `CommandContext::DispatchMesh` stands on the
one `ID3D12GraphicsCommandList6` the device queries when it creates its one frame list; the
solver's three-UAV barrier is `UavBarriers`; a static sampler is `hal::SamplerFields`. The
stream gate found that the old subobject stream's padding bytes were whatever the stack held,
so the bytes `CreatePipelineState` received had never been deterministic; both streams are
built in zeroed storage now and compare byte-equal. The residue the rule still allows is the
plain enums (`DXGI_FORMAT`, `D3D12_GPU_VIRTUAL_ADDRESS`, `D3D12_RESOURCE_STATES`): DX12-first,
no portability tax.

Two contracts the review sharpened. **Absence has a meaning:** a hardware zero cannot say which
of four things it is, so the fiber does — `Zero` (a known physical zero: NULL tile MEANS zero,
sample unconditionally), `Unloaded` (not arrived: sample the residency floor), `OutOfDomain`
(the declared value outside the field's domain, the exposure page's "everything exposed"), or
`Coarse` (a valid coarse substitute, the packed-mip floor) — and `Conservative` names what is
conserved and under which restriction/prolongation (box mean for a level, area-weighted for a
flux or a coverage), `Perceptual` the transfer function it is filtered through. **Temporal
ownership:** CPU scope exit and GPU completion are different events; `hal::Retire(handle,
fence)` is the only way a descriptor, tile, upload allocation or replaced pipeline is released
(a deferred queue keyed by the frame fence), a tenant rebind drains through it, a hot-reload
keeps the old pipeline until the last frame that recorded it retires, and every submission
names its owner.

## 4. The scene (`src/scene/`, `src/app/Scene.*`, `scenes/`) — step 5

What the engine used to be told by some 170 flags is a document with a schema: resolved by one
fold, read by the boot, and hot-reloaded through the same `Apply` it booted with. A demo is a
scene file. The flags survive as one way to write that document, and every recorded recipe was
gated through both spellings before anything read the file.

### The document and its fold — 5a

- **`Props` / `Schema`** — one schema per component type (a Flyweight) binds each key to a live
  field once, with its type, its `Quantity` and unit (`core/GaUnits.h`: a velocity into a length
  refuses with `AcceptsFrom`'s own reason; a number is a bare value in the declared unit, `"12 kn"`,
  or `VesselSpec`'s `{v, unit, src}`), its doc and `Hot | Restart`. Reading, printing, unit refusal
  and reload diffing all come from the one table. An unknown key refuses, naming the node path; a
  `_`-prefixed key is a comment (the `data/wave_scene.json` convention).
- **`SceneBuilder`** — the fold with override: struct defaults < the `base` file < each `include`
  overlay in order < every `--set` in command-line order. Objects merge by key; a NAMED array
  (views, portals, entities, effects, layers, nodes, tools) merges in place by `name`, `"remove":
  true` deletes, a new name appends; an UNNAMED array (`include`, `water.fleet.boats`,
  `rails.keys`) is a value and an overlay replaces it whole; a scalar or a placement is replaced
  whole. `Resolved()` is the memento a recipe is judged by, and its fixed point — Resolved →
  WriteJson → Load → Resolve is the identity — is pinned (priors 42: appending made the fold
  non-idempotent).
- **`Registry<T>`** (`core/Registry.h`) — the house factory idiom written once: `Register`,
  `Knows`, `Make`, `Names`; a miss logs and returns the empty product, never a substitute.
  `VesselRegistry` and `LoaderRegistry` are untouched and proven to answer as the template does.
- **`Node`** (Composite: children and components in FILE order; `ResolveAlong` is the fold through
  the spaces, `Walk` the catamorphism every traversal is written as) and **`Component`** — the
  plugin unit, with no Direct3D type in a signature: `Name`, `Props`, `Configure(const Wiring&)`
  (nullable observers, absence reported), `Init`, `Apply(const PropSet&)` (idempotent: load and
  reload are the same call), `Update(const FrameInfo&)`, `Record(const ViewContext&)`,
  `ReloadShaders`. `LayerComponent` adapts a `Layer` unchanged.
- **`Pose.h`** — the session's pose maps as pure functions. The placement sugar's four spellings
  (`{x, alt, z, az, pitch}`, `{lat, lon, alt[, lookAt]}`, `{motor}`, `{similarity}`) resolve
  through them bit for bit.
- **The files.** `scenes/merrimack.json` is the default scene, every value the legacy default;
  `scenes/chart.json` is three keys over it by `base`; `scenes/views/builtin.json` holds the
  built-in views as data; `scenes/mars.json` is the Mars base (5f); `scenes/rails/*.json` are the
  rails (5e); and `scenes/recipes/*.json` are the nine recorded recipes, each the complete resolved
  document of one flag line (kept in `_recipe`), written FROM the shim and then re-read.
- **The shim.** `Options::ToSets` reproduces the implication laws behind every legacy flag.
  Numbers print as the shortest decimal that reads back to the value the code consumes, float or
  double, so a recipe reproduces its legacy value bit for bit. One refusal is stated: `--boat`
  without a position, which spawned at a 1e9 sentinel, exits 2 with the reason.

### Views — 5b

`View` is a component whose placement is a MOTOR in its space, with a gauge (the space the eye is
expressed in), optics `{fovY, nearZ, reversedZ}`, a viewport, a target, and an optional `Follow
{back, up, aimLift}`. `FromCamera` / `ToCamera` are the session's pose maps, and `ToCamera` is the
rasterizer boundary. `View::Level(pose, up)` re-levels a motor against the up FIELD — roll removed
by construction rather than dropped by extraction — pinned over 1018 poses against the
rasterizer's own basis; it is not yet on the render path. `ViewContext` and `ViewSet` (views in
file order) feed `Renderer::RenderFrame(const ViewSet&)`; the old overload is a one-line forwarder,
and a set of one is byte-identical (dxtest's `SceneConstants` memcmp, equal at the six recipe
poses, seen to fail on a planted `fovY`). `FrameContext.viewIndex` generalizes the set-B precedent.
A second simultaneous view is a named follow-on (§8).

### Water — 5c

`WaterComponent` is the water scene's fan-out said once. Its schema binds every `WaterSceneConfig`
field by the key `data/wave_scene.json` uses (`wavefield`, `closures`, `fleet`: 34 scalars and the
boats), held equal key for key, unit for unit and flag for flag to the scene's `water` section.
`Configure(Observers)` takes nullable pointers (the live config, the bank, the outer Droste bank,
the sea, the globe, the wave field, the water atlas, a `Rebuild` adapter for the solver's
re-configure, the file's watch) and reports what is absent once. `Apply` is the session's boot
block moved verbatim, called at load and on every reload. Two changes are stated: **a reload
applies the same set as boot** (before, it applied neither the sea's four closures nor the auto
edit floor), and **a key removed from the file reverts to its declared default**, not to the value
the run was carrying. `bankTexelM` is the one `Restart` key (the ring spans derive from it at
construction). The wave-field window is `Hot` — the reload re-configures the solver and its bucket
key rolls — but the wave field's page tenant stays bound to the boot window, and the component
logs that when it happens. The instrument is one line per apply, `[water] apply: <n> fields (<k>
changed): key=value, ... | set FNV-1a <hash>`; the reload probe's restored set equals the boot
fingerprint.

### The boot reads the scene — 5d

`main` resolves the scene ONCE, through the same shim `--print-scene` prints, so `gagame <flags>`
and `gagame <the file those flags print>` resolve to one document. `app/Scene.h` is that document
typed: a `SceneDocument` filled through the schema's own table (no second reader), plus the named
lists, each element's placement sugar kept as written (a placement needs the tangent frame the
assembly has only after the surface is built). `Assemble(opt, S)` and `FrameLoop(opt, S, A)` read
property paths: 94 of the 120 distinct `opt.` reads the two files made. Eighteen stay raw
instruments (`--pix`, `--gpu-time`, `--bench`, `--no-vsync`, `--res-trace`, `--lens`, `--stencil`,
`--albedo`, `--viz`, the dump probes, ...). The one-shot modes dispatch by name from the document's
`tools` list, their arguments filled from `--tool name:args` into the flag's own field. The
`layers` list is the registration — and draw — order as data: `Layer::declared` is set once from
it, and `CheckLayerOrder` refuses a boot whose registered order is not a subsequence of the
declared one. Neither the frame loop's phase order nor the assembly's lifetime order moved.

```
gagame [scene.json] [--set a.b.c=value ...] [--tool name[:args]] [instrument flags]
```

A bare launch is `scenes/chart.json` (the M1 chart stays the default picture); a mode flag without
a file is `scenes/merrimack.json`; legacy flags route through the shim into the same document.

### Entities, portals, rails and effects — 5e

- **`Rail`** — segments as data in `scenes/rails/<name>.json`, `rails.active` naming the one flown:
  `keys` (the screw slerp between key motors, `Motor::Slerp`, each leg eased), `spiral` (`pose(u) =
  Sᵘ(base)` through `Space::LevelApply`, the power about the portal's fixed point, with the dive and
  leg laws), `hold`, `turn`. `Rail::At(t)` is pure and returns the eye, the aim, the level and the
  up. The seven files were PRINTED from the session's hand tables; `[scene]` holds the printed text
  byte-equal and `At(t)` bitwise equal to the old lambdas over 36,000 instants. No 4×4.
- **`Entity`** — one `Vessel` built by kind, a spawn motor, a `Controller` (helm | fixed), and
  `Update(f)` = `stepBoat` moved verbatim, its state per entity and its own `TreeWater` (a
  single-threaded memo, never shared between hulls): two boats are two nodes. The chase camera is
  `View::Follow`, fed by the entity's `ChaseFrame`. A residue is stated: the vessel layer takes one
  list, so with two entities it draws the last to step.
- **The freshness contract.** A hull queries the weather manager every step, but the solver's CPU
  mirror was filled only by four tools, so in ordinary play the hull rode the analytic tide and the
  waves and never the solved level or current, with nothing saying so. Now `FrameInfo.asOf` names
  the simulation time the snapshot is coherent at; `entities[].mirrorCadence` (seconds; default 0 =
  never, which is the physics and the cost that shipped) drives `WeatherManager::RefreshOnCadence`,
  one readback per cadence and never one per hull step; `MirrorAsOf` and `TreeWater::Describe` say
  the age that was read. Measured on the helm boat at a 1 s cadence: the set-down surface moves from
  +0.174 to −0.362 m NAVD (the level becomes the solver's, whose basin still carries the recorded
  spin-up transient), and every hull line after it shifts by about half a metre. Turning it on is a
  physics decision, not a refactor.
- **`Portal`** — `Build` is the session's `droste::BuildPortal` block and its `Space::Cycle`,
  verbatim; the frame loop reads `Link()` and `Cycle()` from the node.
- **`Effect`** and `effects/SlicePlane` — the "paper visual" slot: `FieldPort{name, gradeSig, unit,
  space}` declarations in and out, and `RegisterEdges()`, which registers the identical `user.plane
  → globe.ps "slice"` row the assembly wrote by hand. `--slice` reaches it through
  `effects[slice.plane]`.

### Whole-scene reload and the data contracts — 5f

`SceneReload` generalizes the water's law from one file to the fold. One watcher per file the fold
read (the base scene, the scenes it inherits, every `include`) plus the active rail file, the
water's watcher adopted rather than duplicated. On a change the candidate is the WHOLE fold, run
again from the boot's own inputs, so an overlay edit composes with its base exactly as at boot. It
is validated whole (an unknown key, a wrong type, a unit refusal or an unknown node type refuses
the reload and the previous scene stands), and the accepted diff is applied per live TARGET — a
struct, its schema, and the push to the objects that read it. `Restart` keys are reported and held
back; a removed key reverts to its declared default; every change no target covers is walked and
named as needing a restart, including a new or removed node. `Hot` in the table says the engine
can take a change; a target says this run has somewhere to put it, and the log says which one was
missing. One line per reload, `[scene] reload: <files> <n> fields changed (<k> hot, <r> restart)
FNV-1a <state>`, with the hash over every target's accepted set; the probe's restore returns the
boot's hash. Two defects the probe exposed were fixed in the step: the view target had refused its
whole set for want of a frame (so `fovY` never landed), and the water fan-out would have emptied
the fleet on the first reload, because a property set carries no lists (a list under a target is
now a whole value). Seven keys that were `Hot` with nowhere honest to land became `Restart`:
`scene.view`, `views[].at`, `views[].gauge`, `views[].viewport`, `scene.name`, and the two the boot
derives AST registrations from, `sun.source` and `effects[].enabled`.

**The data contracts.** Every boot writes, beside `docs/ga_ast.json` and in its LF convention,
`docs/scene_schema.json` — the sections and their node types, the four placement spellings, and
every type's keys with type, field width, quantity, unit, default, doc, `hot|restart` and enum
names — and `docs/registries.json` — components, effects, layers, vessels (each hull's ledger with
its SI value, raw value, unit and source), loaders and tools, with each type's property schema. A
second boot rewrites both byte-identical. They are what the future WPF/WinUI tool consumes without
linking C++: an inspector with units and defaults, a refusal of the wrong quantity, the knowledge of
which edits apply live, the list of what can be placed; it writes scene files and `--set` lines,
launches `gagame scene.json`, and checks itself with `--print-scene`.

`scenes/mars.json` is the Mars base, a copy of the Merrimack scene with the planet and the orbit
altitude changed (a copy, because `base` is a printed key); `--planet mars` with no file maps to it.
Instantiating a node at reload is the named follow-on: it needs the assembly's construction order,
device objects, residency registration, the layer-order check, AST re-validation and the retire
queue.

### How it was gated

Every recipe through both spellings on one binary: `--print-scene` zero lines from the flag line
(and from the file, apart from the `_recipe` comment); the render from the file equal to the render
from the flags (bit-identical where the pose's own A/A is, the churn-history poses by resident sets
and fingerprints); the six settled stills and the storm rail against the previous sub-step's
binary, every `[settle-exact]`, `[predict]`, `[surface]` and `[kernel]` stream equal; the helm
boat's telemetry identical from both spellings. `[scene]` grew 303 → 316 → 527 → 536 → 755 →
791 checks. What the gates caught inside the step, and the step fixed: a resolved recipe
refused for its empty `base`; appended lists doubling a reloaded fleet; the shim forcing a named
scene file to the chart; a Mars launch inheriting Earth's orbit altitude; and 5d's `--rail-droste`
falling through to the classic keys, which the dive-rail gate caught in 5e (the Droste rail files
carry the flood keys M10 flew).

## 5. The harness (`tools/`) — step 0

`tools/gate_stills.sh EXE OUT` renders the five `stills.sh` poses plus the droste pose for one
binary (each twice, the warm-up discarded, `--settle-exact`); `tools/gate_compare.sh BASE NEW`
puts two such folders side by side: `imgdiff`'s verdict per pose, then the STRICT half — the
per-tenant `[settle-exact]` mapped-set hashes and the `[predict]` want stream — with the `[jobs]`
hash shown but not gated (it folds worker timing). Measured floors (one binary, two runs): bird,
globe, key7km bit-identical; helm 16 px on the horizon strip; helm_ebb ~0.3–3 % and droste
~90–180 px of churn history (resident-set gates, not pixel gates; the bird pose has two states of
the same kind); the 300-frame rail has TWO tile-landing windows, frames 35–70 and 149–180, plus rare
single-frame blips (83, 111, 178, 230 seen between runs of one binary). The second window was
learned the hard way: the Step 2 binary differed there deterministically (three runs, two
worktrees, identical per-frame counts), and it took a standalone harness proving the moved
lattice bodies bit-identical (93,684 checks), a layout variant of the same source (no window
with one translation unit left out of the link), and the BASELINE binary under CPU load
reproducing the window against its own quiet run, to classify it as timing the binary's
layout moves. Rail gate: identical outside the landing windows, `[predict]` equal, the settled
resident sets strict; a per-frame resident-set hash in the rail log is owed to the harness so
presence-vs-content becomes a one-line read. A `[surface]` fingerprint at both fills of the composed-surface rows proved they agree,
so collapsing them (step 4a) is pure motion.

**Learned after step 0, and now part of the recipe.**
- Under CPU or GPU load the rail's second landing window spreads from 149–180 to about 163–230:
  two runs of one binary differed on 74 frames there with the predicted-want hash equal (5d, 5e).
  The rail gate is the `[predict]` hash plus identity outside the windows, never a frame count.
- The churn-history poses (bird, ebb, droste) are judged by the resident sets and by the `[surface]`
  and `[kernel]` fingerprint SEQUENCES compared line for line (4b's instrument: one FNV-1a per
  constant-buffer fill). When a pair looks larger than the pose's recent pairs, an A/A of the new
  binary decides: at 5f the ebb read 1.12 % against its parent, and two more renders of the new
  binary landed 11 px and 1.5 % from it.
- State hashes are the reload's gate: `[water] apply ... set FNV-1a` (5c) and `[scene] reload ...
  FNV-1a <state>` (5f) return to the boot's value when the edited files are restored.
- The recipe gate runs BOTH spellings: `--print-scene` zero lines from the flag line, and the render
  from the scene file equal to the render from the flags on one binary (5d).
- The perf envelope beside the pixels: the helm pose, 600 frames, five interleaved pairs, each
  binary under its own commit's `shaders/`. Every M12 pair landed inside its spread.
- Shaders compile at run time from the worktree: a shader edit during a render or a perf run
  contaminates it, and checking out another commit's `shaders/` for an A/B reverts uncommitted
  shader work — commit first.
- A still that does not move is not a camera that did not move (priors 43): a change to any map
  that builds a rail key, a view or a spawn is gated on the rail as well.

## 6. The contracts, and what a plugin or a UI touches

**The plugin surface** is six kinds of thing, none with a Direct3D type or a template in its
signature: a **component** (`Component`, and `LayerComponent` for a draw), an **entity** (a hull
built by a `VesselSpec` factory — a hull is data, not a class), an **effect** (declared ports and
edges), a **tool** (a one-shot mode by name), and the two that predate M12, a **source**
(`DomainSource`, a node in a tile tree) and a **loader** (`FieldLoader`). The named kinds live in
registries: `Registry<T>` is the template the new kinds use, and the vessel and loader registries
keep their own code, proven to answer as the template does. Exchange channels stay the GPU-buffer
socket: a consumer names the channel, never the producer. In-tree registration is a function per module. A DLL entry point is its own later
step, not a small one: virtual objects across a module boundary still need ownership rules, a
version handshake and compiler agreement, and none of that is designed here.

**The data contracts** are three files a tool reads without linking the engine: `docs/scene_schema.json`
(the scene vocabulary), `docs/registries.json` (what can be named, and each type's properties), and
`docs/ga_ast.json` (the frames and edges every field travels, the node editor's save format). All
three are rewritten at boot from the tables the engine runs on, so they cannot describe a different
engine from the one that wrote them. A scene file plus `--set` lines is the whole input a tool
produces, and `--print-scene` is the round trip it checks itself against.

**The laws a contributor meets**, each enforced where it can be:

| law | where it is enforced |
|---|---|
| no Direct3D interface outside `src/hal/` | `tools/hal_lint.py`, empty allowlist |
| a placement is a similarity (rotor, scale with parity, translation); matrices only at `ToMatrix` | `spacetest` |
| lattice identity before any pointwise field operation | `SameGround` refusals at the tree's compose gather and gate node, and in the residency manager's demand derivation |
| a scene key has a type, a quantity, a unit, a doc and `Hot | Restart` | the schema tables; `docs/scene_schema.json` |
| load and reload are one `Apply`, idempotent | `scenetest` (`[water]`, `[reload]`) |
| a removed key reverts to its declared default | `scenetest`; the reload probes |
| every edge declares its frames; flips are derived, never written | `gatest`, boot validation |
| a change is applied, or named as needing a restart | `SceneReload`'s residue walk |

## 7. The patterns, and where objects stop and functions start

Named where they are, not as aspirations: **Composite** (`Node`) · **Builder** (`SceneBuilder`;
`Assemble` builds the monastery) · **Registry / Abstract Factory** (`Registry<T>` behind the named
kinds) · **Strategy, selected by data** (rail segment kinds; `Entity`'s controller; the sun's source;
Droste lighting)
· **Visitor** (`Walk`) · **Observer** (the scene watchers → reload; `TileTree::onChanged` →
`Invalidate`; Exchange) · **Command** (tools by name) · **Prototype** (`VesselSpec` factories;
`include` overlays; `scenes/views/builtin.json`) · **Flyweight** (one `Schema` per type; one tile set
for every Droste level; the shared root layout) · **Facade** (`hal::`; `WeatherManager::Query`) ·
**Template Method** (the component lifecycle; the frame loop's fixed phase order) · **Memento**
(`--print-scene`; the `[settle-exact]` mapped-set hash; the reload's state hash).

**Objects own identity, lifetime and GPU residency**: the assembly, the layers, the tenants, the
entity, portal and effect nodes, the watchers. **Pure functions own the math**: `Space::ToRoot` and
`ResolveAlong` (a fold over a path), `Placement::PowApply`, `Rail::At`, `View::Level` and
`View::Follow`, the `droste::` closed forms, `Motor::Slerp`, property resolution (a fold with
override), and the walks written as catamorphisms. The scene file is data; loading is a fold,
overlays are a merge, `--set` is an assoc, and a reload is the same fold again with its diff applied.
The rule between the two halves: an object re-reads a pure function's answer when its inputs move,
and never caches a derived form (a 4×4, a resolved pose) as if it were the source.

## 8. What M12 did not do — the named follow-ons

Each was found or scoped during the steps and left out on purpose; the step that named it is given.

- **A second simultaneous view** (5b): a second target chain, a camera-anchored bank and walk per
  view, the weather manager over a span of viewers, one scene-constants block per view. The seam
  records a list of views; the list has one.
- **Instantiating a node at reload** (5f): construction through the assembly's order, device
  objects, residency registration, the layer-order check, AST re-validation, destruction through the
  retire queue.
- **The vessel layer owning the entity list** (5e): with two entities it draws the last hull to step.
- **The wave field's page tenant following a run-time window move** (5c): the solver re-solves at the
  new window onto pages bound at boot; logged when it happens.
- **The float-yaw correction** (5b, measured in 5e): held as a patch, because correcting the camera
  map re-quantizes the storm rail's imagery beyond the few-pixel precedent; the owner's call.
- **The freshness contract's default** (5e): `mirrorCadence` ships at 0 = never, today's physics. A
  cadence makes the hull read the solved water; that is a physics decision, measured and not taken.
- **The near clip plane in the Droste walk** (4d): NaN on every level, so the near-plane cull has
  never fired.
- **The two window-uv spellings in the kernels** (4e): measured 10–13 % of texels apart by up to
  three eighths of a texel, kept separate with the numbers in the banner.
- **The settle-first hold** (a separate branch): holding a `--settle-exact` still at the first instant
  as well makes the bird and ebb poses bit-identical run to run, which would turn their churn-history
  bands into pixel gates. It was measured on its own branch and is ported after M12.
- **Named commands for the UI** (plan): F5, R, T and the time nudges as commands a tool can invoke.
- **A non-draining versioned `Exchange::Publish`** (plan): per-frame plugin data keeps using the
  constants ring until then.
- **The DLL loader** (plan): §6.
