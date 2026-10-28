# WATER V2 — the water as a collage of declared functions of time

*2026-10-22. Mark: "find all the abstractions and primitives shared between [the data and math we've been
throwing at the water] ... a water manager/orchestrator that knows the data and its reach ... dealing with
folds and different data types ... the reach of a single point measurement like a buoy ... the 'mostly'
static elevation map ... the variable water and coarseness of it all." And: "These are waves right? Why not
send wave functions of time instead of just a normal height? That way if we only have one segment of time
... it can repeat cleanly without too many CPU uploads." The star: an asteroid into the Atlantic off screen,
and the wave reaching Newburyport.*

This document states V2 as objects and products first (what each thing IS), then the manager that composes
them, then the ladder that carries a tsunami, then the stages that get there from today's code with a gate
each. Nothing here is a new rendering idea; every part is a published method or a law already in the engine
said once instead of five times.

---------------------------------------------------------------------------------------------------------

## 1. What the water already is, and the one sentence it is trying to say

Read in one pass, today's water is already two kinds of thing:

* **Functions of place and time that the GPU evaluates.** The tide is 18 constituent PHASOR fields
  (`compose/WaterAtlas.h`: h = msl + Σ Re[P_c(x) e^{iω_c(t−T0)}]); the solved wave field is, per spectral
  component, an amplitude, a wavenumber and a unit phase SPINOR advanced by the rotor e^{−iσt}
  (`sim/WaveField.h`); the FFT cascades are spectral modes advanced by the same rotor
  (`shaders/OceanCompute.hlsl`). "NOAA carries the state, the GPU carries the phase"
  (`sim/WeatherManager.h`).
* **State that only a solver can carry.** The shallow-water windows (`sim/SweSolver.h`) store the
  DEVIATION from the analytic tide plane, so a NULL tile means "the functions are exactly right here";
  the foam churn remembers.

And one evaluator already sits over all of it: `WeatherManager::Query(lat, lon, t, grain)` answers level,
current, waves, wind, bed, depth from "the finest resident rung that covers the point", with provenance
strings. `TreeWater` is the CPU twin of the GPU kernel `CsBankFill`, assembled in the kernel's order.

So the sentence V2 says is the one the code has been converging on:

> **The water is a sum of declared functions of time over one bed. A solver exists only to carry the part
> no function can, and hands it back as a function the moment it can.**

What is missing is that the sentence is said in five dialects. Each producer declares its place, its grain,
its time and its absence its own way; the bank kernel hard-codes a block per producer; the CPU twin
mirrors each block by hand; and a new input (a buoy, a storm surge, a tsunami) means a new block in both.

## 2. The primitives

Six objects. Everything in §5's inventory is one of them or a product of them.

### 2.1 Quantity (what is said)

| quantity | value | datum / unit | combine law (how two producers meet) |
|---|---|---|---|
| **Level** η | scalar | m, NAVD88 | SUM of departures: tide + surge + tsunami + a solver's deviation |
| **Current** u | vector (east, north) | m/s | SUM of departures; a solver window REPLACES what it resolves |
| **Bed** z_b | scalar | m, NAVD88 | OVER (finer over coarser) + the edit layer on top |
| **Sea** S(ω, θ) | spectrum / partitions / modes | m²/Hz/rad | ENERGY sum, then the FOLD (§2.3) |
| **Wind** U₁₀ | vector | m/s | finest rung |
| **Memory** (foam, churn) | scalar | 0..1 | a solver's own (history is state) |

The datum belongs to the snipping, never to a reader (the collage rule): a datum link is one more
source in the compose, not a branch.

### 2.2 Support and reach (where a producer has an opinion, and how it fades)

Every producer declares a **reach**: a weight w(x, g) ∈ [0, 1] at place x and grain g, with **absence =
w = 0, never value = 0** (the rule the height stack learned the hard way, `TreeWater`'s header). Five
kinds, one function each:

| support | reach law | today |
|---|---|---|
| **Point** (a buoy, a tide station) | the stations' kernel: 1/d² tapered by smoothstep 90 → 270 km, its share by the nearest's taper; a point speaks as a CORRECTION (a ratio or offset on a field at its place), not as a field | `SeaLayer::BuoyGainAt`, `WaterAtlas` StationWeight |
| **Grid** (GFS-Wave, GoMOFS, ETOPO) | its cells/nodes with NaN = absence; absence reads the nearest valid node within a declared reach (B2) | `SeaGrid::NearestWet`, `GlobeModel` |
| **Tree** (the bed, colour, exposure) | the tile tree's coverage per tile, its own level and the fold below it | `TileTree`, `HeightTree` |
| **Window** (a solver's domain) | its box, feathered to its parent over a declared band; open faces read the parent | `SweSolver`, `WaveField` window |
| **Analytic** (equilibrium tide, a storm declared in a scene) | 1 everywhere | `TideModel`, `--storm` |

### 2.3 Grain and the fold (coarseness, one law per quantity)

Every producer has a native grain g₀. Read at g ≥ g₀ it is FOLDED; read finer it is magnified (HIERARCHY
4.20). The fold is a property of the quantity, written once:

* Level, current, bed → the box mean (the bed also keeps its max lane for the casts).
* A phasor field → the Cl(2)+ box mean of the spinor (`WaveFieldSource`: the cl2 law), whose modulus falls
  where the phase winds inside the box — coherence lost to the grain is variance.
* A sea → geometry while the grain resolves its phase, statistics (σ²) past Nyquist: the logistic in
  ln(k_Nyquist / k_fold) (`WaterBank.hlsl` M9bt/M9bx). Energy is never lost or doubled between rings.

Today this law is written separately for the cascade bands, the solved components, the WaveFieldSource mips
and the bank rings. V2 has `Fold(quantity, value, g)` once and every producer goes through it.

### 2.4 Time form — Mark's "wave functions of time"

Every producer declares HOW it depends on time, and that declaration IS what crosses to the GPU. The CPU
uploads when the FUNCTION changes, never per frame:

| time form | f(x, t) | GPU form | examples |
|---|---|---|---|
| **Static** | f(x) | a page | the bed (versioned, §2.6) |
| **Harmonic** | f₀(x) + Σₙ Re[Pₙ(x) e^{iωₙt}] | phasor planes + an ω table; time = a rotor per term | tide constituents; the solved field's components; **a solver's periodic response fitted online (§4.2)** |
| **Packet** | Σₙ Aₙ(x) cos(ωₙ(t − τₙ(x)) + φₙ) | amplitude + travel-time planes per frequency | **a tsunami's far field (§6)**; a recorded transient at a nest boundary |
| **Series** | frames f_k(x) at t_k, a law in time between them | the two bracketing frames + a weight | GFS-Wave hours (today snapped to the nearest hour, which is the respectrum pop) |
| **Stochastic** | a spectrum S realised with a seeded phase per mode | the cascades' h₀ and its rotor | the sea between the solved components |
| **Transient** | state that steps | a solver's banks, own clock | the SWE; an event while it is still near its source |

A segment of time with a beginning and an end becomes Harmonic by its Fourier series over the segment
(periodic extension), which is exactly Mark's "repeat cleanly": a tidal cycle of solver output, fitted once,
plays forever with no stepping and no uploads.

GA statement: a Harmonic field is a field of even elements of Cl(2) (spinors) and a time advance is the
product with the rotor e^{Iωt}; a Packet is the same with a phase that is a travel time times ω. One
evaluator in the kernel: η(x, t) = Σ_terms ⟨P_term(x) R_term(t)⟩₀.

### 2.5 Clock (bindable per producer)

Each producer binds a clock in the scene (the collage rule): the real now, a scene time, or an EVENT's own
t = 0 ("a sunny sky over a hurricane of months ago"). A tsunami is a producer whose clock starts at impact.

*Built 2026-10-23* (Mark: "the times for data should either be scene constants or bound to another
variable like xaml data binding works"; `scene/SceneClocks.h`).

**The clocks.** One clock runs: the scene's (`time.start`, paced by `time.timeScale`, nudged by the
keys). Every other clock is stated in the scene file as one of:
* a **constant**, `{"start": "2026-10-23T18:59:47Z", "rate": 1}`;
* a **binding**, `{"bind": "sun.earth.clock", "offset": "-3h", "rate": 1}`. That is the bound clock's
  instant plus the offset, at rate times its pace: XAML's Binding with an affine converter;
* **neither**, which is the scene's own clock.

**What a scene can name:**
* `water.clock` is the data's: the tide, the solver and its faces, the currents, the sea state's hour,
  the wind and the cascades. Everything that reads the frame's `simUnix` reads it.
* `sun.clock` is the season.
* `sun.earth.clock` is the time of day.

**The guarantees:**
* A cycle is refused at load: it is logged, and the clock that closes it reads the scene's clock.
* `[clock]` in the log states every clock.
* The window's title shows each one resolved.
* The scene selftest's `[clock]` checks pin the semantics.
* An event's own t = 0 (the tsunami) is the next clock to add: a binding with an event as its anchor.

### 2.6 Identity (the "mostly static" bed and everything that depends on it)

Identity is content (the compositor's law). The bed's identity is per tile (the tile tree's). Every product
that read the bed — a wave-field solve, a fitted harmonic set, a fetch, an exposure tile, a solver spin-up —
keys on the identities of the tiles it read. Editing the bed (`survey.edits`, or later a brush) moves only
those identities, so exactly the dependents over the edit re-solve; a stepping solver reads the new bed on
its next step. Nothing global is invalidated, nothing stale survives.

## 3. The manager

`WaterManager` grows out of `WeatherManager` (the query surface, the rungs and the provenance are already
right). It holds a registry of producers, each a small object:

```
struct Producer {
    Quantity q;            // level | current | bed | sea | wind | memory
    Reach    reach;        // point(kernel) | grid(cells, reachNodes) | tree | window(box, feather, faces) | analytic
    Grain    g0;           // native grain (m)
    TimeForm form;         // static | harmonic | packet | series | stochastic | transient
    Clock    clock;        // scene-bound
    Identity id;           // content key (bed tiles read, inputs, version)
    // CPU: value + weight + provenance at a place, instant and grain
    Sample At(Place, Time, Grain) const;
    // GPU: the planes and the term table it contributes to a ring at a grain (no per-frame upload)
    Terms Export(Ring, Grain) const;
};
```

Two readers, one composition (Mark's rule: the renderer is fed the same water the boat floats on):

* `Query(place, t, grain)` composes `At` over the producers by the combine laws (§2.1), weighted by reach,
  folded by grain — what a hull, a probe, the HUD read.
* `Export(ring)` hands the bank kernel the TERM TABLE for that ring: each term a (quantity, time form,
  planes, weight plane, fold). The kernel loops over terms. It never learns a tide from a tsunami.

The manager also SCHEDULES: which solver windows run (demand = residency, as today's owned windows spin up
lazily), when a solver's output is fit back into a harmonic set (§4.2), and which nest feeds which (§6).

## 4. What changes in the existing pieces

### 4.1 The bank kernel and its twin

`CsBankFill` becomes a loop over the term table instead of a block per producer; `TreeWater::At` becomes the
same loop on the CPU over the same table. The closed-form laws (`Jet.hlsli`, `WaterTerms.h`) stay as they
are — they are the physics, and they already have their twins. The win is that a new producer is one
registry entry, not a block in two languages.

### 4.2 The solver hands its work back as functions (harmonic playback)

A tide-driven window repeats with the tide. While it steps through one tidal cycle, it accumulates, per
cell, the least-squares normal equations for the constituents and their shallow-water overtides (M2 S2 N2
K1 O1 + M4 MS4 M6 for η, u, v) — the online harmonic analysis ocean models already do (MOM6's
`MOM_harmonic_analysis`; FVCOM output through UTide). Then it STOPS stepping: the window plays back as a
Harmonic producer, no state, no uploads, a few phasor planes. It steps again only for what the fit cannot
carry — a storm surge, a tsunami, an edited bed — and then it carries only the departure from the fit, the
same trick the eta bank already plays against the tide plane. (Caveat from the analysis literature: a
clean separation of the main constituents wants a lunar month of record; for playback of the same forcing
one M2 cycle per spring–neap phase is enough, because the fit only has to reproduce the model, not
separate constituents physically.)

### 4.3 Series in time (the respectrum pop)

The grid's forecast hours are snapped to the nearest hour today (`SeaGrid::HourIndex`), so the synthesis
re-makes its spectrum at a step. As a Series producer it blends the two bracketing hours by energy, and the
cascades' h₀ is re-made as a lerp between two realisations; the solved field already blends two tide
buckets the same way.

### 4.4 The crest law (the "clip" Mark saw) — see §7.

## 5. Inventory of today

The full read-only inventory (21 inputs, ~45 laws, every reader, every writer and bound of the drawn
height, with file:line) is `out/review/water_inventory_2026-10-22.md`. What it says for V2:

**The primitives are already there, said five ways.**
* Phasor fields with **three sign conventions**: stations `cos(ωt+φ)`, the atlas `Re[P e^{+iωτ}]`, the
  cascades `h₀ e^{−iωt}`, the solved field's spinor × a CPU rotor. Only the cascades and the solved field
  send phasors to the GPU; the atlas tide is evaluated on the CPU at tile corners and lerped.
* The fold, five spellings: the cascades' logistic, the solved field's sinc² × coherence, the wakes'
  smoothstep, the pixel's `wRingPx`, the solve's upload gate + the cl2 box-mean mips.
* Reach kernels: the same 90 → 270 km taper written twice (stations, buoys), the station coverage feather
  written twice, window feathers of four different kinds.
* Absence-is-not-zero, identity-is-content, variance-preserving blends, buckets that trigger work, and
  CPU/GPU twins — each present, each per producer.

**Duplications that drift (not intentional twins).** NAVD-above-MSL twice; the gfswave Hs read four ways
(one nearest-node with a longitude quirk, tools only); two tide evaluators meeting over one cell at the
solver's edge; the churn's depth computed with no tide; and **four confirmed kernel ↔ hull drifts**: the
excess scaled by the exaggeration on the CPU only; the set-wave throw gated on the CPU only; the shelter
applied before the crest law on the GPU and after it on the CPU; the cascade lip fall mixing exaggerated
and raw metres on the GPU only.

**Dead or dormant.** `wavefield.gammaHs` (in the cache key, not the law); the column plane drives no
geometry since MsLip left (it feeds shading, spray death, the LOD); `WaterVertexColor` under pixel water;
the churn is never advected; and **both Python twin gates are dead**: `proofs/water_terms.py` stopped at
`LipFlightTau` (array out-parameter) and at `kGWave` (gravity became the planet's on 10-20), and
`proofs/wave_field_check.py` still expects the 368-byte table of 16 components (M9bl made it 688/32).

**Place constants inside global laws** (against "datasets by measurement"): `barNormalDeg` 285 for the
whole window, the Merrimack-shaped current proxy (x ≤ 800 m, 105/285°, ×3.0), the exposure fan ±26°.

**Exaggeration.** `wavefield.exag` is 1.6 in 38 of 39 scenes (the doc says 1.15): with `heightScale` 1.15
the solved sea is drawn **1.84× physics**, while the depth laws (dry weight, land floor, landness) are not
scaled. It makes every breaking artefact taller than its own depth allows. **Done (stage 1a, Mark: "do the
exaggeration"):** `wavefield.exag` is 1 in the code and every scene, so the whole sea is drawn at the one
stated `heightScale` 1.15; the CPU twin no longer scales the excess (a ratio) — drift 1 below is closed.
(Correction, stage 1: `data/wave_scene.json`, the untracked water config every scene includes, still said
1.6 and won over the scenes; stage 1a's renders were at 1.84×. It says 1.0 now.)

**What a tsunami meets today.** No basin state; the solver's open sides are a sponge (an arrival is
absorbed, never admitted); its deviation is clamped ±6 m; a scrub over 1 h resets it; and the waterline
is ONE scalar for the planet (`gWavesB.w`), which the land classifier and the past-the-rings sea compare
against — an inundation could not move the drawn shore. What can carry it: the bank's per-texel level
departure (param.x), the eikonal sweep (`wavecore::EikonalSweep`), Green's law, the page tenant.

## 6. The ladder that carries a tsunami to Newburyport

Prior art says how this is done operationally, and it is the same shape as the engine:

* NOAA's SIFT forecasts with MOST: a database of PRECOMPUTED linear deep-ocean propagations from unit
  sources, combined linearly, whose composite deep-water solution gives the BOUNDARY CONDITIONS for nested
  nonlinear inundation models (Stand-by Inundation Models) near shore.
* Asteroid tsunamis (Ward & Asphaug 2000): the impact is a parabolic transient CAVITY whose depth and
  diameter follow from the impactor's energy by crater scaling; the far field is LINEAR DISPERSIVE theory
  (frequency dispersion makes the long-wave 1/√r decay nearly 1/r for impactors under a few hundred metres),
  and linear shoaling at the peak frequency near land. Korycansky, Asphaug & Ward found linear theory a
  reasonable guide while the cavity is moderate against the ocean's depth.
* GPU tsunami codes exist at both ends: TRITON-G (spherical shallow water with tree refinement to 50 m, 10 h
  of the 2004 Indian Ocean event in ~10 min on three P100s) and Celeris (Boussinesq, Direct3D, faster than
  real time near shore).

The ladder, three rungs, one-way nested:

1. **Basin** (the North Atlantic, ~4 km cells on the cube lattice, wet tiles only). The event's far field
   as a **Packet**: for each of ~32–64 frequencies, the travel time τ(ω, x) is the eikonal of the phase speed
   c(ω, h) from the full dispersion relation (a fast-marching/Huygens walk on the sphere — the same kind of
   walk as the fetch and the exposure), and the amplitude is the source spectrum × geometric spreading on
   the sphere × Green's-law shoaling. That is exactly the solved wave field's algebra (amplitude, wavenumber,
   phase spinor per component) at basin scale, and it is Mark's "wave function of time": the tsunami at any
   place and instant is Σₙ Aₙ(x) cos(ωₙ(t − τₙ(x)) + φₙ), evaluated by the GPU with no stepping. The globe's
   far sea can draw it from orbit — a ring crossing the Atlantic. A time-domain linear shallow-water run of
   the same basin (the virtual-pipe kernel on the sphere) is the CROSS-CHECK for what rays miss (reflections,
   caustics), as SIFT's database runs are time-domain.
2. **Shelf** (the Gulf of Maine, ~250 m). Linear-to-weakly-nonlinear shallow water, whose open faces read the
   basin Packet as an incoming level and flux.
3. **Local** (the Merrimack window, 10 m, nonlinear, wetting and drying — the solver that exists). Its
   Flather faces already read "the tide plane + a deviation"; they read the manager's Level at the face
   instead, which now includes the shelf's tsunami.

The event is one more producer: `Impact{lat, lon, radius, density, speed, angle}` → the Ward & Asphaug
cavity as the basin's initial condition (and later `Rupture` with an Okada seafloor deformation). Its clock
starts at impact; the scene can run the event clock faster than the eye's.

Rough numbers for an impact 2,000–3,000 km out: deep-water speed √(gh) ≈ 200 m/s, so 3–4 h to the shelf
edge and 1–2 h across the Gulf of Maine at 30–45 m/s. A 1,500 × 1,000-cell basin at 4 km with a 20 s step
is ~1,100 steps for 6 h in the time-domain cross-check — seconds on the GPU; the Packet needs one eikonal
walk per frequency and is cached by (event, bed identity).

## 7. The clip: measured

Mark's crops (crests near the inlet bar drawn with flat tops and near-vertical walls). At a low pose over
the bar (`out/review/mer_bar.json`, 10-22 16:00Z):

| arm | walls |
|---|---|
| as shipped | yes |
| column plane off (`water.bank.column=false`) | yes |
| lip fall + flight removed (shader copy) | yes |
| solved field's lateral orbit + flight removed (shader copy) | **almost all gone** (faint double lines) |
| solved field off (`water.wavefield.enabled=false`) | **none** |

And the solve itself (`cache/wave/5133caf1e513ca1b.bin`, decoded): amplitude and phase are continuous in
open water; their jumps sit on the shoreline, where they belong. So the "clip" is not a cap on height — the
2026-10-04 removal of |η| ≤ 0.55 h holds. It is the CREST LAW FOLDING THE SHEET: near breaking the solved
field's crest water is thrown forward (the depth's orbit coth(kh), the excess ratio, the flight), bounded by
"the landing" (`Jet.hlsli` LipReach, 1/k + e*), and a height-field mesh folds past k·A_h = 1 — the crest lays
over the face as a flat terrace with a vertical edge.

The fold is right where real waves plunge (the Mavericks barrels it was built for) and wrong where they
spill. Which one a beach makes is the **surf-similarity (Iribarren) number** ξ = tan β / √(H/L₀) (Battjes
1974): spilling below ~0.5, plunging above. The Merrimack's ebb shoal (slope ~1:50, H 2 m, T 9 s) is
ξ ≈ 0.16 — spilling; Mavericks' reef (~1:15, H 6 m, T 16 s) is ξ ≈ 0.54 — plunging. V2's crest law weights
the THROW (everything past the fold onset k·A_h = 1) by the plunging share P(ξ), with tan β the bed's own
slope along the travel and H the local crest height; below the onset nothing changes. Spilling breakers keep
the steep face and the foam; plungers keep the barrel. One law, the bed decides.

**Built (stage 0, branch `claude/water-v2`).** `PlungeShare(tan β, H_c, k, h)` in `Jet.hlsli` and its twin in
`WaterTerms.h`: the plunging band between Battjes' breaking-height bounds 0.4 and 2.0, a product of two
logistics in ln ξ whose width (0.092: 10–90 % over a factor of 1.5) is a stated closure. tan β is the bed's
steepest slope by ±4-cell differences, carried by the solve in the envelope's fourth (spare) channel as
√tan β, so the kernel and `WaveField::ProbeAt` read the same number; the solver version is 12. Measured on
the new solves, over the cells where the sea breaks: the Merrimack (10-22 16z) ξ median 0.19, 20 % of the
cells with P > 0.5; Mavericks under a 6 m 16 s swell ξ median 0.27, 30 % (the reef's steep face). The bar
pose before/after: the folded terrace is gone, the crests stay steep (`out/review/bar_plunge_ab.png`).

Still on the list from the inventory, not changed yet: the solved lip's floor `max(y, min(y_was, −aEnv))`
and the cascades' lip fall (a pure vertical map sized by the un-yielded envelope, in mixed units); the
mesh's land floor at 0 m NAVD and its one-vertex water↔land lerp against the scalar waterline (walls where
the classifier calls a bar land at low tide).

## 8. Stages, each with its gate

| stage | what | gate |
|---|---|---|
| **0** | The crest law's throw weighted by the surf-similarity share (§7), GPU and both CPU twins (`TreeWater`, `WaveField::ProbeAt`); residency fixes from the audit (§9) | the bar pose before/after (walls gone at the Merrimack); the Mavericks lineup still barrels; drawn-vs-felt probe unchanged at the hull; selftest |
| **1** | The declarations: `Quantity`, `Reach`, `Grain`/`Fold`, `TimeForm`, `Clock`, `Identity`, `Producer`; today's producers wrapped behind them; `WaterManager` = `WeatherManager` + the registry. No behaviour change. | frozen-clock stills byte-identical on the gate scenes; line count of the water code DOWN (the roadmap ledger). **Built 2026-10-22** (`sim/WaterProducer.h`; `WaterManager`'s registry; `Query` one loop; the boot manifest, 18 producers at the Merrimack; the point's reach, the NAVD-MSL link and the window's smoothstep each written once; PM's wind sea and two solver reads removed). Measured: `--dump-water-state` (bed, level, u, v, shadow on 420×300 cells through `Query`) BYTE-IDENTICAL to stage 1a's binary; the datum and sea log lines identical; selftest PASS. The still could not gate it: stage 1a's binary against itself differs on 7.2 % of pixels (the settle hold never closes: 37 `swell.exposure` paints of the raw height stack hold the Io lane past the 600-turn watchdog, run to run). **Ledger: engine +533 / −303, net +230 — not down.** The removal that pays for the registry is stage 2's. |
| **1a** | The mip ramp (D1) and the NULL-read climb (D2), the no-bytes fill (F6), the time-based retry (F8); one stated exaggeration | a descent rail: max per-frame pixel difference down, no frame with a snap; `[restest]` aniso rows 0; `[stuck]` 0. **Done 2026-10-22.** `[restest]` "then the climb" 8x anisotropic: 0 of 473,088 (the clamp alone: 1,606). A frozen-clock hold at the inlet (`--scale 0`, one pose, 12 s from boot; only landings move), `streaming.fadeTurns` 0 vs 6 on one binary: 98,423 fade steps; over the trickle after the load burst the largest 32×32-block luma change a frame 7.2 → 3.5, p99 4.6 → 2.1, frames over 4× the median 12 → 3. What the fade does NOT reach, seen in the burst's largest jumps (≈ 22 in both runs): the town's buildings appearing at once (the building layer's LOD) and the marsh's flooded extent moving as finer heights land (physics reads the finest held mip by law). The descent clip with the sea moving cannot see the fade at all (its holds read alike): the frozen hold is the instrument. |
| **1b** | Music as the first new producer (§10): band table, scene bindings, Media Foundation + XAudio2 playback | the water and a light term follow the file's bands at a place; two runs agree frame for frame |
| **2** | One term table: the bank kernel and `TreeWater` loop over terms; tides evaluated on the GPU from the atlas phasors (no per-tile CPU corner params) | stills identical to stage 1 within the declared tolerance; frame time not worse; the hull probe. **2a built 2026-10-22:** the ring corners carry the tide's 18 phasors (asked once per lattice point when a ring moves), the kernel turns them by this frame's rotors (`WaterAtlas::Rotors`, doubles on the CPU: w t runs to 10^4 rad). `--twin-surface`'s LEVEL rows identical to 0.1 mm at 0/500/2000/8000 m; the bank's CPU tile list **15.7 → 0.43 ms a frame**; the fill +0.04 ms at p50. Engine +95 / −15. **The generic term loop waits** for its third level producer (the Packet, stage 5): the level has two terms today (the harmonic tide, SUMMED; the solver window, REPLACING over its weight), and the hull already composes it through stage 1's registry loop — a table with no third consumer would be an abstraction without a reader. |
| **2b** | **The water as columns (§12):** wet is the column holding water; the solver holds the column, not a departure from a plane; residency follows the water; the waves push (setup); the tide and a tsunami enter at Flather ocean faces | §12.5's C1–C5: the stations, a lake at rest, a scripted surge, the bar awash under 2 m, Monai valley. **Built 2026-10-23 (§12.10):** lake, ledger, Ritter, Stoker, planar setup pass; the bar awash; the surge floods the platforms with the ledger closed to 6 × 10⁻⁶; run-up −10 % at the finer grid; Salisbury Point and one current cell worse; Monai not fetched |
| **3** | Harmonic playback: the SWE window fits its cycle online and plays back; steps only for departures | levels at the stations and the inlet current vs the stepping solver over one tide (rms, lag); frame time with the solver idle |
| **4** | Series in time: the grid's hours blended (no respectrum pop) | a 4 h clip across an hour boundary, no visible step; the spectrum's moments continuous |
| **5** | The basin rung + `Impact` producer as a Packet; the time-domain cross-check | travel time vs √(gh) integrals; decay vs Ward & Asphaug's fits; the two methods agree on arrival and first-crest amplitude in deep water |
| **6** | Shelf rung + nesting into the Merrimack window; the asteroid demo scene | the wave reaches Newburyport at the time the ladder predicts; the local solver floods what the bed says it floods |

## 9. Residency (the mip-transition hiccups)

The read-only audit (`out/review/residency_audit_2026-10-22.md`) found the queue story sound — one DIRECT
queue, `UpdateTileMappings` on it during recording, DirectStorage landings fenced before `CopyTiles`, the
map monotonic and conservative — and these, ranked by what a player sees at a transition:

| # | finding | status |
|---|---|---|
| F2 | **The pop is by construction.** A landing tile's map byte drops a whole mip in one turn ((m+1)·16 → m·16) and every sampler's min-LOD clamp with it: each tile snaps into focus, a descent is a patchwork of snaps. The ×16 byte has room for a ramp (D1): write the fractional bytes over 4–8 turns; trilinear blends m and m+1, both held by law. | **fixed (stage 1a)**: a landing tile's level steps from its parent's to its own over `streaming.fadeTurns` (6) turns, short of its mip + 1, so every reader that floors the byte (kernels, CPU, audit) reads the finest held mip at once; a tile held under a gap keeps its fade until it is seen; a settled still is not exact while a fade moves |
| F1 | Finding 61 open: the colour's anisotropic sampler reads NULL tiles past the gather's clamp (measured 1,606 of 473,088 probe points, up to 36 % zero): dark streaks at tile frontiers at grazing angles. Fix: `CheckAccessFullyMapped` on the sample, re-sample one mip coarser in the quads that touched a NULL tile (proven on this GPU in `ResidencyFloor.hlsl`). | **fixed (stage 1a)**: `PageSampleClimb` for the colour and the buildings, a level above the level READ (clamp or the hardware's LOD), twice at most, a window never past its floor (a rank still touching NULL leaves the pixel to the rank above). `[restest]` "then the climb": **0** of 473,088 (a climb from the clamp alone left 486) |
| F3 | A tree's "not now" that was not a stop or a permit refusal was handed over as 64 KB of zeros, mapped and claimed: a flat or black tile standing until an Invalidate. | **fixed**: answered not whole, nothing mapped, asked again when the tree changes (no retry spent); counted (`N not now` in the tree's stats). **Its Again (2026-10-23)**: a not-whole tile is never held, so its subtree waits on it (`ParentGate`); the wave field names the loads it refused (`g_loadTenant`, as the Google fetches do) and the frame loop says `Again` once the solve it was asked for is there -- its own key's, or the first for a tile refused before any solve (the boot's floor tiles: a headless boot of merrimack_boat saw all 33 planes' tops refused, every plane's pyramid waiting for the first swap; the next boot, none) -- never a newer key's, which the tree drawn until the swap would paint under its own identity (`WaveFieldSource::Reopen`; a swap's `Reload` asks them all, `ForgetRefused`); a plane past the solve's answers void, not "not now"; the exposure's refusal is always its roll, whose `Drop` asks every tile anew; `Again` leaves a held tile that is current as it is (it made it stale: on the wave tenant, read by the table before) |
| F4 | Finding 9 open: kernel bilinear taps cross a tile frontier under a one-byte floor and read a NULL neighbour (a seam line in the water while the neighbour is coarser). | **fixed (stage 1a)**: `PageLoad4` reads each tap's status; a NULL tap re-reads the four a level up, twice at most, within the array's mips; no whole level, the first read stands |
| F5 | A refused refresh left a held tile stale AND pending forever: `PendingCount()` never 0, no settle closes. | **fixed**: stale (its version bit is the truth of its bytes) but not pending |
| F6 | A DirectStorage tile with no landing slot is mapped, filled with stale staging bytes and never landed: a leaked slot and a rectangle that never sharpens. The 512 slots are reachable at 96 tiles a turn with a 2-turn fence lag. | **fixed (stage 1a)**: the gather maps a disk tile only with a landing slot for it this turn (else it stays Loaded, `waitLanding`); a tile that still reaches the fill without bytes copies nothing and is let go with its unlanded aliases |
| F7 | `TileAtlas2D::FlushResidencyMap` has no caller: the velgrad lens reads an Init-time map. | debug lens only |
| F8 | Four retries in four turns, then a permanent hole for the run. | **fixed (stage 1a)**: a failure is asked again a glance (60 turns) later, 4 rounds at most |
| F9 | Four reader slots per record; a fifth standing reader evicts a pin. | count first |
| F10 | A 255 byte clamps to mip 7 and reads NULL in the kernels. | no change: the bed's "none" is also 0 m |

D3 (barriers and map uploads per turn only for the tenants that changed), D4 (the pool kept further ahead:
bigger heap chunks), D5 (a tile's siblings land in one turn), D7 (write the one-queue assumption down: an
async compute queue would need fences after the turn's mappings) are in the report with their DX12/NVIDIA
reasoning.

## 10. Music as a producer

Mark, 2026-09-28: "the mp3 could be like a buoy source in the water physics with the renderer displaying it
in a practical way that maybe responds to the water or just maps to the music"; 2026-10-22: "use music as a
source for coloring/lighting and even affecting the water." In V2 terms it is not a feature, it is one more
producer, and it is the yardstick for whether the primitives are right:

* **Quantity:** band energies over time — the same data type as a buoy's spectrum (NDBC 44013: 46 bands,
  m²/Hz) and an equaliser's bars.
* **Reach:** a Point at a declared place, with a kernel (the buoy's), or global (a scene-wide light).
* **Time form:** Series — the file is known start to end, so the band table is precomputed once and cached
  by the file's content identity; the sea and the light are functions of (scene, instant). Two machines and
  a late joiner agree; decoders that are not bit-exact never matter because readers read the table.
* **Clock:** the scene binds the file's clock (play from t₀, loop, scale).
* **Bindings (the scene's, never the reader's):** a band map (file band → sea band, or → a light/colour
  channel), a gain (unitless → m²/Hz, or → nits), smoothing, a direction (a file has none). For the sea the
  physics limit is honest and the scene chooses it: a sea band cannot change faster than its own period —
  follow the music's slow contour, follow the beat as chop (rings near the source), or slow the file's clock
  until the beat is a swell's period.
* **Output for light and colour:** the same producer feeds a Light quantity (the sky's/sun's tint, the
  water's emissive foam, the far sea's glint) through its band map — the renderer reads a light term, not
  "music".
* **Audio playback** is a reader, not a producer: Windows' Media Foundation decodes the mp3 and XAudio2
  plays it (both built in, no new dependency); the analysis (an FFT into the band table) runs once; the
  sound card's clock is slaved to the scene clock, not the other way round.

It lands after stage 1 (the producer registry), as its first new producer — "On a Pale Horse" as the test
file. Mark's d3 visualizer (link to come) is the reference for the band mapping and the look.

## 11. Questions for Mark

1. ~~**The exaggeration.**~~ Answered (2026-10-22): one stated number — `wavefield.exag` 1, the sea at
   `heightScale` 1.15 everywhere.
2. ~~**The mip ramp (F2/D1).**~~ Answered: yes — built in stage 1a, 6 turns (`streaming.fadeTurns`).
3. **The stages' order.** 0 (done) → 1 declarations → 2 one term table → 3 harmonic playback → 4 series in
   time → 5 basin + impact → 6 shelf + nesting, with music after 1. Anything you want earlier?
4. **The tsunami's first form.** The frequency-domain Packet (stateless, dispersive, what rays give) first,
   with the time-domain basin run as its cross-check — or the time-domain run first?
5. ~~**The waterline.**~~ Answered (2026-10-23, Mark: "wetness should be based off the solved water there
   not just the tide"): the shore reads the bank's level at every vertex and pixel (dbd25c0). §12 takes the
   next step: wet is the column holding water, with no level compared at all.
6. ~~**The water as columns (§12).**~~ Its four questions answered 2026-10-23 (§12.9).

## 12. The water as columns

*2026-10-23. Mark, looking at the Merrimack's entrance bar: "the water shouldn't drop a foot around and over
the bar", "its like bars have a piecewise function", "wetness should be based off the solved water there
not just the tide", and then: "I'm thinking the voxel model we have can actually make this more elegant
than we are thinking. Especially if we want tsunamis."*

What prompted it, measured that day: under a 2 m forecast swell the entrance bar draws a row of tiny
islands, tan shore rims around dark pits, and the waves funnel down into each one. The drawn mean level is
smooth: the ocean +0.09 m, the entrance +0.13 m and upriver +0.25 m NAVD at noon. The stations' own
predictions are the open coast −0.15 to −0.03, the entrance +0.07, Newburyport +0.12 and Salisbury Point
+0.19, so inside the estuary the solver is right. The drawn ocean stands about 0.12 m high because the
sponge pins the open sea to the entrance station's plane (+0.075), not the open coast's tide; that is one
more reason for §12.3's ocean faces. What is wrong at the bar is the frame. The bar's bumps stand above the mean water, so every reader that compares a level
with the bed calls them dry. The water that would cover them is the waves' setup, and nothing in the
engine makes it.

### 12.1 The object

The water at a place is a **column**: the bed `b` (the one stack), the water the column holds `h ≥ 0`
(volume per area), and how that water moves (its momentum, the face fluxes). The surface is `b + h`.
**Wet is `h > 0`.** No reference level appears anywhere: not the planet's waterline, not a station's
plane, not a threshold to classify against.

As GA: the column is the interval of the up line from `b` to `b + h`, a 1-blade along `e_up` of length
`h`; the water's volume element is `h` times the area bivector `e_east ∧ e_north`, a 3-blade. Volume
conservation, `∂h/∂t = −∇·(h u)`, is the divergence theorem the face fluxes already discretise.

The same object exists at three grains in the engine already:

* **depth-integrated:** the shallow-water solver's cell (its volume and its face fluxes);
* **resolved along the vertical:** the column plane (`docs/WATER_VOLUME.md`): the drawn sheet's crossings
  of one column (face, lip underside, lip top) where a crest turns over;
* **a full volume:** `TileAtlas3D` (the cloud's tenant), where topology needs it. This is Irving et al.'s
  tall cells, a column with a resolved layer on top (SIGGRAPH 2006), made real-time by Chentanez and
  Müller's restricted tall-cell grid (2011).

Kass and Miller (SIGGRAPH 1990) drew water this way, as columns of a height field moving over dry terrain
and receding from it. GeoClaw (LeVeque, George and Berger 2011) and Tsunami-HySEA (INGV's operational
warning code) inundate the same way: a dry cell is a cell holding no water, and cells change between wet
and dry as the flow comes and goes.

### 12.2 Why today's water cannot say it

1. **Wetness is three level comparisons, each a threshold.**
   * The shore classifier, `ComposedLandnessEdit` (`Compose.hlsli:420`): the bed against a level, with
     `over` a smoothstep over 0.4 m (`:435`). Since dbd25c0 that level is the bank's, not the planet's.
   * The bank's dry weight, `smoothstep(0.05, 0.65, lvl − bed)` (`WaterBank.hlsl:640`).
   * The solver's own wet/dry: `SurfaceAt` answers the water's surface if wet, else the bed
     (`Swe.hlsl:110`), and the derived current calls `h < 0.05` dry (`:375`).

   Three readers, three cutoffs, one bed. The bar's islands sit in the gaps between them.
2. **The solver stores a departure from a moving plane, not the column.** `gEta` holds `η − tide(t)`
   (`Swe.hlsl:1–8`, `:71`). Every resident cell books the plane's rise as debt (`THE PRISM TERM`, `:338`),
   dry cells included, so a dry cell's stored value sinks under its own bed. 1ee0e89 had to make the bank
   read around that. A NULL tile means "exactly the tide plane" (`:105`), not "no water".
3. **Where the solver may hold water is fixed by the bed at init.** A tile is resident only if one of its
   cells has a bed under +1.2 m NAVD (`SweSolver.cpp:382`). Ground above that can never be flooded: the
   marsh platforms (+1.2 to +1.7 m), the dunes, the streets. A surge or a tsunami could not inundate
   Newburyport by construction.
4. **Bounds that are not physics.**
   * The deviation is clamped to ±6 m (`Swe.hlsl:360`).
   * `BedAt` answers a 100 m wall outside the lattice.
   * The step is sized from the deepest water at init (`SweSolver.cpp:458`).
   * A scrub of more than an hour snaps back to the plane (`:835`).
5. **The open sea absorbs.** The sponge relaxes the deviation to zero in about 10 s (`SweSolver.cpp:474`,
   `Swe.hlsl:122`), so an arrival is eaten, never admitted. The river faces already radiate (Flather,
   `Swe.hlsl:270`). The ocean sides do not.
6. **The waves never push the water.** Breaking is computed twice: the solved field's limiter keeps a
   fraction `L` of each cell's rms (`WaveField.cpp:829–830`), and the bank's `BreakFactor` does the same
   per texel. The momentum the broken waves give up never reaches the flow, so there is no setup. A bar
   under a 2 m sea stays dry where the real one would be awash.

### 12.3 The law

**State per column:** the surface `η = b + h` (absolute, NAVD88) and the signed face fluxes. These are
today's virtual-pipe faces, kept: they are the momentum on the faces.

**The scheme is a hybrid (Mark: "why not both"), and the hybrid has a name.** Stelling and Duinmeijer
(2003) keep the staggered faces we already have and make them shock-capturing. In a flow expansion they
approximate the momentum advection to conserve momentum, which is what a Godunov scheme does at a bore,
a hydraulic jump or a front running onto dry land. In a contraction they conserve the energy head
(Bernoulli). The face depth is taken upwind. The two approximations converge to the same solution where
the flow is smooth, so it is one law.

Today's pipes have no advection term at all (a face accelerates down the surface slope and is damped).
Run-up and bores are exactly where that is missing, so the term is the hybrid's whole cost. GeoClaw's kind
stays in the plan as the instrument: an HLL face flux on the same faces, run side by side on the dam break
and Monai (§12.5). If the staggered hybrid misses there, the HLL flux joins it at the fronts, blended by a
continuous front weight, not switched.

* **Mass:** `∂h/∂t + ∇·q = 0`. Today's `CsSweHeight` without the prism debt.
* **Momentum,** per unit width: `∂(h u)/∂t + ∇·(h u ⊗ u) = −g h_f ∇η − C_d |u| u + F_w/ρ`, where
  `h_f` is the face depth and `F_w` the waves' force per area (below); a face's flux is this times its
  length. That is today's `FaceUpdate` plus two terms: the advection, discretised as above, and the
  waves' force.
* **Wet is `h > ε`,** one tolerance, the solver's own (1 cm; GeoClaw's "dry cells").
* **Positivity:** no face ships more than its donor holds. Today's quarter-volume clamp (`Swe.hlsl:227`)
  is the "limit the outgoing fluxes of draining cells" of Bollermann, Chen, Kurganov and Noelle (2013).
* **Well-balanced:** a lake at rest stays at rest. The face depth is the hydrostatic reconstruction,
  `h_f = max(η − max(b_L, b_R), 0)` (Audusse et al. 2004). That is today's `hface` (`Swe.hlsl:213`), kept.
* **The tide enters only through the open sea,** as the rivers already do: Flather faces on the ocean
  sides, `u_b = u_ext + √(g/h)(η − η_ext)`, with `η_ext` the manager's Level at the face. That Level is the
  tide's phasors (stage 2a), plus the Packet's tsunami (stage 5), plus the setup of a sea no solver holds.
  No moving plane and no debt; the prism arrives because the boundary rises. The open sea stops being a
  sponge: a sponge would eat a tsunami before it arrived.
* **The land soaks (Mark: "water gets sponged up by dry cells but still spreads out and goes down rivers;
  eventually soil saturation").** The sponge moves from the open sea to the ground, where it belongs. A
  land column holds a second store under its bed: its soil's water. Water standing on it infiltrates at a
  rate that falls as the soil fills, by the Green–Ampt law that GPU flood models pair with the shallow-water
  equations (Buttinger-Kreuzhuber et al. 2022; Hou et al. 2021). What the soil does not take keeps flowing,
  over the land and down the river channels the bed already carries, and out at the river faces. For now
  the soil store is one stated depth and rate. Later it is the soil's own measurements: saturated
  conductivity and water contents at 250 m from SoilGrids2 (HiHydroSoil v2, or CSIRO's six-layer product);
  one global dataset per measurement, never per place. Then saturation is the soil column's state: it
  drains, and a second surge runs off ground the first one wet.
* **The waves push.** `F_w = −∇·S`, with `S` the radiation stress of the sea in the column:
  `S_xx = E(n(1 + cos²θ) − ½)`, `S_yy = E(n(1 + sin²θ) − ½)`, `S_xy = E n sinθ cosθ`, where
  `E = ρ g H_rms²/8` and `n = c_g/c` (Longuet-Higgins and Stewart 1964). In the surf zone this is the
  breaking dissipation `D` over the phase speed along the travel, `F ≈ D k̂/c`; XBeach's longshore
  forcing, `F_y = D_r sinθ / c`, is that form.
* **The wave solve publishes the force (Mark asked which fits the design and the boat).** The deciding
  fact: the bank's rings follow the camera. A solver that read its force from them would put a setup under
  the boat that changed with where the eye is, and with two eyes (the minimap) it would have two answers.
  The physics must not know where anyone is looking. The wave solve's window is fixed and its products
  are cached by content identity, so it publishes `F_w` as two planes per bucket, beside its others: a
  STATIC producer (§2.4) that every reader takes alike.
  * C4 grows the wave solve with the flow: a second, nested window at the solver's grain over the
    solver's domain, as XBeach-type models pair a wave model with the flow it drives.
  * Where no wave solve covers, the solver computes the force itself from the global GFS-Wave grid's sea
    at the cell (eye-independent) through the bank's breaking law at the cell's own depth.
  * The boat reads the setup through the solver's level, which it already reads (`SolverRefine`): drawn
    and felt see one setup, wherever the eyes are.

Setup, a bar going awash, overwash and run-up then come out of the solver. No setup law of our own is
written. The magnitude to expect: at a planar shore the setup is about `(5γ/16) H_b`, and field data give
about `0.16 H_s`, so about 0.3 m under today's 2 m sea. That covers the bar's bumps.

**What the readers read.**

* **The bank publishes the column's departure from the stateless column,**
  `Δh = h − max(L(x, t) − b, 0)`, with `L` the manager's Level. A NULL tile reads 0, meaning "the
  stateless water is right here" (the "NULL tile reads zero" rule). A dry cell outside the reach of the
  stateless water has `h = 0` and a stateless `0`, so `Δh = 0` with no debt anywhere.
* **The shore classifier's height witness becomes "does the column hold water":** wet is `h > ε` under
  the point. The survey mask stays the witness where it is finer (unchanged), and the hand edits stay law.
* **The bank's dry weight becomes the same smoothstep of `h`,** not of `lvl − bed`.
* **The hull (`TreeWater`, `WaterManager::SolverRefine`) reads the column's surface `b + h`** and wet by
  `h`: one water, two readers.
* **The column plane composes on top unchanged.** The drawn sheet is the column's surface plus the
  waves' displacement, and the trough bound of 1ee0e89 is the column's own statement: water cannot
  stand below its bed.

**Where the solver holds water: residency follows the water.** A tile is resident while any of its
columns holds water, or while a neighbour's water can reach it within the next batch (a one-tile ring
where wet columns touch a tile's edge). It is released after it has been dry for a while (hysteresis).
The +1.2 m threshold goes. The literature does the same: GeoClaw refines where the surface departs from
sea level; GPU flood codes compute fluxes only for the wet cells and their wet/dry interfaces, and grow
that set as the inundation spreads (HydroMPM, *Water* 17(18); GeoSWE's compressed active cells; Brodtkorb et al.'s
early exit for dry blocks). The tiled banks (`RequestMap`, `CommitMappings`) already do the mapping. What
changes is the rule. The step is sized each batch from the live water, `max(√(g h) + |u|)` over the
resident columns. The ±6 m clamp goes: `h ≥ 0` is the only bound, and positivity enforces it.

### 12.4 What it touches

| file | change |
|---|---|
| `shaders/Swe.hlsl` | the state from a departure to the absolute surface; the prism term goes; the ocean sides get Flather faces (the river faces' law); the wave force joins `FaceUpdate`; the ±6 m clamp goes |
| `src/sim/SweSolver.cpp` | residency by the water (`mapWet` → the wet set plus a ring, per batch); the step per batch from the live water; the sponge → ocean faces; a scrub re-fills to the stateless column (`h = max(L − b, 0)`) and steps, instead of snapping to a plane |
| `shaders/WaterBank.hlsl` | the level read becomes the column (`Δh` above); the dry weight on `h`; `SolverSurfaceDev` (1ee0e89) retires, since there is no debt to read around |
| `shaders/Compose.hlsli`, `GlobeMesh.hlsl`, `Globe.hlsl` | the classifier's height witness becomes wet by `h` (`VsOut.wl` becomes the column's water) |
| `src/sim/WaterManager.cpp`, `WaterSurfaceTree.cpp` | the hull reads `b + h` and wet by `h`; `Reconstruct` becomes the column's surface |
| `src/sim/WaveField.cpp` | publishes `F_w = −∇·S` as two planes per bucket from its components (`a·L`, `k`, direction); C4's nested window at the solver's grain |
| `shaders/Swe.hlsl` (again) | the Stelling–Duinmeijer advection on the faces; the land's soil store and its Green–Ampt infiltration; the force read from the published planes, or from the GFS-Wave grid's sea at the cell where none covers |

### 12.5 Steps, each with its gate

| step | what | gate |
|---|---|---|
| **C1** | The solver holds columns: the absolute surface, no plane, no debt; the tide at Flather ocean faces from the manager's Level; wet is `h > 1 cm`; the Stelling–Duinmeijer advection | §12.6's judges, each for what it can see: the levels at Plum Island, Newburyport and Salisbury Point over a tide cycle, rms and lag no worse than today's; the currents at the entrance (ACT0816) and Newburyport (ACT0821), peak speeds and slack times; a lake at rest flat to 1 mm for an hour; volume conserved to 10⁻⁵ of the prism; the dam break against Ritter's solution |
| **C2** | Residency follows the water; the +1.2 m threshold goes; the land soaks (the soil store, Green–Ampt) | a scripted surge (+2 m at the faces) floods the marsh platforms and recedes, with no tile left resident and dry past its hysteresis; the flooded area against a flood fill of the bed under the same level (`FloodWindow`, `SweSolver.cpp:1164`), within one ring of cells; the volume ledger closes (in at the faces = in the columns + in the soil + out at the faces) |
| **C3** | The drawn water and the hull read columns: the bank's `Δh`, the classifier wet by `h`, `TreeWater` the same | `--twin-surface`'s LEVEL rows unchanged; frozen-clock stills at the entrance where only shore texels change |
| **C4** | The waves push: the wave solve publishes the force, a nested window covers the solver's domain, and the GFS-Wave grid stands in where none does | the setup on a planar test beach against `(5γ/16) H_b` within 15 %; the entrance bar awash under the 2 m sea (the islands gone); no change on a calm day (`H_s → 0`); the boat's level and the drawn level agree under two eyes (`--twin-surface` with the minimap on) |
| **C5** | Run-up: Carrier–Greenspan on a plane beach (analytic), then the Monai valley benchmark (NTHMP BP7, the 1:400 tank) as a scene, then the asteroid through stages 5–6 | the run-up against Carrier–Greenspan; Monai's maximum run-up in the valley within 10 % (NTHMP's standard for this test); the HLL instrument on the same faces as a second opinion |

C1 changes the solver's state, so today's spin-up and the stations are its gate. C3 is what you will
see at the bar. C4 is what makes the bar right.

### 12.6 The judges (Mark: "can't we get a few to trust? ... be careful with the word 'only'")

No one of them is the truth. Each sees some failures and is blind to others, so each is named with its
blind spot. A disagreement between two of them is a finding to read, not a verdict.

| judge | what it sees | what it cannot see |
|---|---|---|
| **The stations' harmonic predictions** (Plum Island 8440452 at the entrance, Newburyport 8440466, Salisbury Point 8440273; `data/composite/world/tides`) | the astronomical tide's level and phase, place by place | surge, a river in flood, anything not astronomical: a prediction is a model too |
| **NOAA's observed water levels** (CO-OPS 6-minute verified data where a station observes, Boston 8443970 among them, on NAVD) | the real water, surge and river included | places without a gauge; verified data lag about a month; fetching them is a new network source, asked of Mark first |
| **The current predictions** (ACT0816 Merrimack River entrance, ACT0821 Newburyport, ACT0826 Plum Island Sound entrance; already loaded) | the flow: peak flood and ebb speeds, slack times | the level; a single point of a section |
| **The manager's Level at the open faces** (the atlas: EOT20 and the stations) | that the boundary is fed what every other reader composes | anything inside: a consistency check, not truth |
| **Analytic solutions** (a lake at rest, exact volume accounting, Ritter's dam break, the planar-beach setup, Carrier–Greenspan run-up) | the scheme itself, exactly | real beds and real seas |
| **The lab** (NTHMP's Monai valley, BP7) | run-up over complex ground, measured | our place |

### 12.7 Cost (estimates, to be measured)

* **The window:** 1788 × 1174 cells (2.1 M), a step of about 0.25 s, up to 16 substeps a frame.
* **The force:** two loads from the published planes and a few multiply-adds a face. The planes are made
  once per bucket on the CPU, beside the wave solve's others: about 13 MB a bucket at today's 2 m window,
  and about 17 MB for the nested window over the solver's 2.1 M cells.
* **The advection (Stelling–Duinmeijer):** an upwind momentum difference a face, a handful of loads.
* **The soil:** one more value a land column (its water) and one rate a step.
* **Residency:** one reduction a batch, over the wet set's edge tiles.
* **The step:** a reduction, instead of a constant.
* **Expected net:** about today's cost. More resident tiles where water spreads, fewer where marsh dries.

### 12.8 What it does not change

* the tide as phasors (stage 2a);
* the term loop's deferral (stage 2), though C1 makes the Level the solver's boundary, which is that
  loop's first new reader;
* the crest law;
* the column plane's drawing;
* the residency of the tile tree.

### 12.9 Mark's answers (2026-10-23)

1. **The scheme:** "Why not both or a hybrid?" Yes: the staggered faces made shock-capturing by
   Stelling–Duinmeijer, with an HLL face flux as the instrument beside it; a blend at the fronts only if
   Monai asks for it (§12.3).
2. **The ocean side:** "for now we should have water get sponged up by dry cells but still spread out and
   go down rivers. Eventually I'll want soil saturation." Read as: the sponge belongs to the ground, not the
   sea. The land soaks (the soil store, Green–Ampt), the water spreads and drains down the rivers, and the
   soil's saturation becomes its state later, from the soil's own measurements. The open sea gets Flather
   faces, because a sponge there would eat the tsunami (§12.3). *If the sea's side should stay a sponge for
   now too, that is one line to change back.*
3. **The force's source:** "which do you think fits our design best? I would want the one that works best
   with my boat." The wave solve publishes it, so the physics never depends on where the eye is: one setup
   for the drawn sea, the solver and the boat (§12.3).
4. **The judges:** "Can't we get a few to trust? ... be careful with the word 'only'." Six, each with its
   blind spot (§12.6). None alone.

### 12.10 Built (2026-10-23, overnight: Mark, "complete all of section 12")

Every number below is from a run in this checkout; the commands are in §12.11. Where a gate failed or the
plan changed, it says so.

**C1, the solver holds columns** (`shaders/Swe.hlsl`, `src/sim/SweSolver.{h,cpp}`).

* **The state** is the water each column holds, `h` (its own R32F bank: a NULL tile holds none), over the
  bed cached once a frame per cell from the stack (`CsSweBedFill`; it was read eight times a cell a
  substep), and the faces' velocities in two banks that take turns (the advection reads the substep
  before). No plane, no debt, no ±6 m clamp, no 100 m wall: a NULL neighbour is a wall through the live
  mask until the residency maps it.
* **Published** for the readers: `m_eta` = the column's surface less the frame's tide plane (a dry column
  publishes its bed), and the fibre `(u, v, h, wet)` (wet = `h` > 1 cm). *Deviation from §12.3:* the plan
  said the bank publishes `Δh = h − max(L − b, 0)`. The surface plus the wet flag says the same to every
  reader and keeps "a NULL tile reads the stateless water" (a departure of 0) without a per-cell Level in
  the solver.
* **The scheme:** Stelling and Duinmeijer in their velocity form: the momentum-conservative advection
  where the flow decelerates, the energy head where it accelerates, the face depth upwind; the pressure
  the hydrostatic reconstruction at the sill. Two first tries failed their own tests and are recorded:
  the advection applied to stored fluxes with the current upwind depth (a 4.35 m spike ran ahead of
  Stoker's shock), and the discharge form with momentum everywhere (Stoker passed; Ritter's rarefaction
  rang with spurious jumps — S&D's own reason for the energy head).
* **Positivity:** Bollermann et al.'s limit, all of a column's outgoing faces scaled together to the
  water it holds (`CsSweLimit`), in place of the quarter-of-the-donor clamp a face.
* **The open sea:** Flather faces on the open sides, riding the tide atlas's Level at four points a side
  (`SetLevelSource`; a cubic along the side), their `u_ext` the prism by continuity: the domain's wet area
  at the plane (a priority flood's spill levels from the boundary, so a hollow behind a dune does not
  count) times the plane's rate, less the rivers' transport, over the sides' live section. *Grown from the
  plan:* every edge cell whose bed lies below mean sea is a sea face too (`OceanSideOf`,
  `SweDomain::SeaFaceCell`): the Merrimack window's south edge cuts Plum Island Sound off from its own
  inlet, and its tide had been coming from the sponge reaching 1.9 km in from the east side, across
  Plum Island. Faces: 379 N, 180 S, 1174 E. The sponge is gone.

| C1's analytic judges (`--tool swe-test`, a resident 512² square, walls round it) | result | gate |
|---|---|---|
| lake at rest, rough bed with an island out of it, 1 h | max \|surface − level\| **0.0**, no column changed wetness | 1 mm ✓ |
| volume, a hump sloshing in the closed basin, 1 h | **1.3 × 10⁻⁷** | 10⁻⁵ ✓ |
| Ritter (2 m onto dry, 120 s) | L1 **2.0 %**, 0.885 m at the dam against 0.889 | 5 % ✓ — the 1 cm tip lags 23 cells (reported) |
| Stoker (2 m onto 0.5 m) | L1 **0.8 %**, the shock within **0.5 cell**, the plateau 1.102 against 1.104 m | 5 %, 3 cells, 3 % ✓ |

| the stations (`--swe-cycle 13`, 2026-09-24 16Z; the current predictions end 2026-10-03) | before (pre-C1) | after (C1 + C2) |
|---|---|---|
| 8440452 Plum Island, entrance: rms / lag / range | 6.7 cm / +10 min / ×1.00 | **5.1 cm / +8 min / ×0.99** |
| 8440466 Newburyport | 6.5 cm / +10 / ×1.00 | **6.5 cm / +8 / ×0.98** |
| 8440273 Salisbury Point | 9.5 cm / +14 / ×0.98 | **15.9 cm / +22 / ×0.93** — worse; open (in the river, bed −8.6 m: the advection's losses in the bends against a drag coefficient tuned without them) |
| 8440369 Merrimacport (by the river face) | 12.5 cm / −2 / ×1.16 | 11.6 cm / +12 / ×1.11 |
| 8441241 Plum Island Sound (south) | 11.0 cm / +12 / ×0.91 (the sponge's) | 26.4 cm / +38 / ×0.88 (its own inlet; ×0.15 before the sea faces) |
| ACT0816 entrance, the deepest cell by it: peak flood / ebb | +0.83 / −0.75 (pred. +0.88 / −0.64) | +0.18 / −0.47 — the jet moved off this cell: the six-point section across the throat *gained* (flood −0.33 → −0.40 m/s); open |
| ACT0821 Newburyport | +0.66 / −0.66, lag +14 (pred. +0.62 / −0.62) | +0.49 / −0.54, lag **+6** |
| ACT0826 Plum Island Sound entrance | +0.09 / −0.08 (pred. +0.62 / −0.67) | **+0.37 / −0.41** |

**C2, residency follows the water; the step from it; the land soaks.**

* **The rule:** each batch a kernel sums every resident tile's water (`CsSweTileStats`: which columns hold
  water and which edges it touches, the deepest and fastest column, its volume). A tile stays while a
  column holds water or water touches its edge from a neighbour (the ring). It is released after 30 min
  dry, with the water it held booked in the ledger. The boundary's tiles always stay. The first residency
  is the spill level under +1.2 m, not the bed under it.
* **The step,** each batch: `0.45 min(dx, dy) / (√(g h_max) + |u|_max)`. Seen: 0.21–0.26 s.
* **The soil** (Green–Ampt, a stated soil for now: 0.10 m capacity, 10⁻⁵ m/s, 0.05 m suction × deficit;
  full under standing water at a reset).
* **The ledger** on the GPU: every face an exterior cell shares with the interior, each substep (Kahan).
* **Gate, a scripted surge** (`--swe-cycle-surge 2`: +2 m on the open sea, 3 h wide, peaking on the high
  water; the open sea +3.07 m NAVD):
  * The solver wets 758,550 cells, **226,471 above the old +1.2 m floor** — the marsh platforms and the
    streets.
  * Its water lies inside a static flood fill of the bed at that level but for 87 cells.
  * The fill's other 138,152 cells are water a 3 h surge had not carried inland by its peak. "Within one
    ring" holds only for an equilibrium the surge never reaches; the gate's wording was wrong, not the
    water.
  * Tiles 84 → 88 as the water came; none dry and waiting at the end.
  * **The ledger closes to 6.0 × 10⁻⁶ of the tidal prism** over 13 h before the waves pushed: 8.69 × 10⁷
    m³ in, the columns + 8.28 × 10⁷, the soil + 4.12 × 10⁶, 830 m³ unaccounted.
  * A first version counted only each exterior cell's face toward the inside and missed 6.7 × 10⁻⁴: a
    sea face's run also meets interior edge cells sideways.
  * On the final build:
    * The tide alone closes to **1.0 × 10⁻⁵**.
    * With the waves pushing and the surge, it gains **3.6–7.2 × 10⁻⁵** from run to run. Part of that
      was the `h ≥ 0` guard lifting rounding-negative columns the new limiter emptied exactly; it now
      leaves a hair, 10⁻⁶ of the water.
    * The rest is not found: open.

**C3, the readers read columns** (`shaders/WaterBank.hlsl` `SolverSurfaceDev`, its twin
`SweSolver.cpp` `Reconstruct`). The surface is the wet columns', by their bilinear weights. Where no
corner holds water the level stands under the solver's own published bed by the bank's dry band (0.65 m),
so the dry weight reads 0 and the classifier reads land. A first version stood it under each *reader's*
bed, and the hull's and the ring's beds differ in grain: the twin's LEVEL rows read 0.62 m at 500 m. The
solver's bed is one value for both. A second version let that level climb the dunes (+3 to +5 m beside
0.1 m water); interpolated between a ring's texels it made cliffs of water metres high (LEVEL 1.7 m at
8 km, the hull's waves 2.6 m rms on nonsense depths). The dry level is now capped at the frame's
waterline: a dune is land under the waterline as surely as under its crest.

| `--twin-surface`, the entrance camera, Oct 22 noon: LEVEL max / rms (m) | 0 m | 500 m | 2000 m | 8000 m |
|---|---|---|---|---|
| before (pre-C1) | 0.0001 / 0.0000 | 0.066 / 0.008 | 0.126 / 0.022 | 0.076 / 0.030 |
| after (C1–C4) | 0.0005 / 0.0001 | **0.399 / 0.050** | **0.382 / 0.055** | 0.076 / 0.030 |

The gate said "unchanged": it held at the camera and at 8 km, and not between them -- a dry column's level
stepped down at the water's edge and the two readers interpolated the step at different grains.

**Then the water decides** (Mark: "you are using rhetorical methods to decide how it treats the water,
instead of letting the water decide what it does when it hits or is over a bar"):
* The drawn level is the surface of the columns that hold water, and nothing else.
* Where the four corners about a point are dry, it is the surface of those holding water one cell further
  out (a tent two cells wide). A dry crest in the water is then ground that the water's own level is held
  against.
* The dive under the dry column's bed is gone. Only where no column within two cells holds water does a
  "no water" level remain.
* The twin, same instant:

| LEVEL max / rms (m) | 0 m | 500 m | 2000 m | 8000 m |
|---|---|---|---|---|
| the water decides | 0.0005 / 0.0001 | **0.0003 / 0.0001** | **0.085 / 0.012** | 0.18 / 0.039 |

The bar's ovals at 3 pm on Oct 22 (an hour before low water):
* They are not this law: both laws draw them alike.
* They are not the soil (an A/B, `GAGAME_SWE_NOSOIL`).
* They are not the land classifier: `--lens mix` reads them as water.
* They are not the drawn surface's height: `debug.view height` at 5 cm contours shows no dip.
* With the bed under the water held at plain sand they mostly go, so the dark centres are the aerial
  photo seen through shallow water. The faint rest is the bar's bumps as shallower water, the right way
  round. What the seabed's colour should be is the footprint question, still Mark's.

The solver's own surface over the shoals rises with the bed (noon Oct 22: +0.08 m over −2.5 m to +0.24 m
over 0 m with the waves pushing). *Not done:* the classifier still
reads the level, not `h` directly. It reads land wherever the solver says no water, which is the
statement, but the fibre's `h` is not yet in the globe's planes.

**C4, the waves push.**

* *Changed from the plan:* the wave solve (`WaveField`) does not publish the force yet. The solver
  carries its own sea: a steady balance of the wave energy flux marched shoreward one cell a sweep, four
  sweeps a batch (`CsSweWaves`), from the global wave grid's sea at the domain's open water
  (`SetSeaSource`: GFS-Wave's Hs, Tp and direction, the plan's "fallback" made the source). It does:
  * **shoaling** (the flux conserved along the ray);
  * **refraction** (Snell at the local contour);
  * **shelter** (dry ground passes nothing on);
  * **breaking as a dissipation per metre** (Baldock et al. 1998: Battjes and Janssen made explicit, with
    the bank's breaking height read for one wave as √2 of its Hs limit).
* The force on each face is `−∇·S`.
* It is eye-independent and at the solver's grain, so the design's reason holds: one setup for the drawn
  water, the solver and the hull. But the waves that push and the waves that are drawn are two models of
  one sea, broken by one height law. Publishing `WaveField`'s planes over a nested window is the open step.
* A first version applied the bank's clipped-Rayleigh factor cell by cell. It compounds (a sea loses energy
  every cell even in deep water) and gave half the setup.

| C4's judges | result |
|---|---|
| planar beach (8 m deep, 1 in 437, Hs 2 m, 8 s, 1 h) | setup **0.167 m**, against (5γ/16) H_b read in one wave's measure (γ_rms = 0.60/√2, H_b,rms) **0.184 m (−9 %) ✓ 15 %**. Read on Hs as written in §12.5: 0.367 m — that mixes the Hs limit into a one-wave law. The field's 0.16 Hs₀: 0.32 m (on steeper beaches) |
| a calm day (Hs → 0) | no force: the push is off below 1 cm of Hs, so the solve is the C1–C3 one (by construction) |
| the entrance bar under the 2.1 m sea, Oct 22 noon | **the islands are gone** — awash with the push, standing out without it (`out/review/c4/bar_three.png`) |
| the setup at the entrance camera | the drawn and the hull's level +0.050 m with the push off, **+0.159 m** with it (Hs 2.1 m, Tp 9 s, from 98°) |
| two eyes | not run with the minimap. The solver's sea has no camera input (the open water's grid value), so one eye's twin stands for both: at the camera the hull and the drawn level agree to 0.5 mm |

**The cost** (`--rail-zoom` over the inlet with `--gpu-time`, the wave cache warm):
* The solver's GPU time is 0.57 ms at p50 (0.24 before), 0.65 mean (0.35); the render p50 7.7 ms (6.7).
* The first run after the change stalled 32 s: the CPU wave field re-solved twice because the solved
  current it hashes had changed (the known cache miss), not the solver.

**C5, run-up.** Synolakis's solitary wave on a plane beach (NTHMP BP1: the run-up law of his
Carrier–Greenspan solution, `R/d = 2.831 √(cot β) (H/d)^{5/4}`), H/d 0.019 on 1 in 19.85, frictionless.
At d = 50 m (the run-up's tongue nine cells long) **−16 %**; at d = 100 m (eighteen cells) **−10 %**,
against a 10 % gate. It converges with the grid at first order. **Monai** (BP7) needs NTHMP's data
downloaded: not fetched, asked of Mark. The HLL face flux was not built; the hybrid has not needed it.

### 12.11 The judges' commands

* The scheme (C1, C4's setup, C5's run-up), on analytic beds in one resident square, walls round it:
  `gagame.exe out/review/fp/entrance_low.json --tool swe-test --headless --set streaming.offline=true`.
  `GAGAME_SWETEST=lake,ledger,ritter,stoker,setup,runup` picks cases; rows land in `out/swe_test_*.csv`.
* The stations, the currents, the ledger and the water from above (C1, C2):
  `gagame.exe out/review/fp/entrance_low.json --swe-cycle 13 --set time.start=2026-09-24T16:00:00Z --set
  streaming.offline=true`, plus `--swe-cycle-surge 2` for the scripted surge. It writes
  `out/swe_judges_<pid>.csv` and `out/swe_water_{highwater,surge}_<pid>.png`. The current predictions
  (`currents.json`) end 2026-10-03, so the currents are judged on 2026-09-24.
* The push's A/B: `GAGAME_SWE_NOSEA=1` turns the waves' force off (the calm day's gate is the same switch).
* Before tonight: `build/bin/gagame_c0.exe` with `--shaders out/review/c0/shaders`; `gagame_c0j.exe` is
  the same solver with the judges.

### 12.12 The shoal at the jetty tip: the drawn sea's normalization (2026-10-23)

Mark, at the ridge off the south jetty's tip: "the bar is sponging up water when it should already be
over saturated", "shouldn't the water be getting pushed up there?", and then "really review your physics
and normalization of water. Something is up with that shoal/bar at the tip of the jetty and it's
affecting the mesh."

**The instrument.** `GAGAME_TWIN_GRID=latC,lonC,halfM,stepM,file.csv` on `--twin-surface` reads the bank
on a square of labels (`ReadBankPoints`, the mesh's own reconstruction). Each label is written beside the
bed and level a hull reads, then summarised by depth:
* the labels' mean `dispY` (Lagrangian);
* the surface an eye sees, each label weighted by det(1 + ∇D) (Eulerian);
* the bank level against the hull level;
* folds.

The scratch scene is `out/review/shoal/shoal.json` (views `shoal_above`, `shoal_low`, `deep_in`, `deep_out`).

**What it found** (16:00Z Oct 22, Hs 2.2 m, Tp 9 s, from 99°; the ebb jet 0.9 m/s across the ridge):

| | before | after |
|---|---|---|
| the level (bank − hull) | 0.000 everywhere | 0.000 |
| the drawn mean, 1–2 m of water | **−0.29 m** (−0.24 at 14:00Z Oct 23) | +0.03 to +0.05 (one instant, a 400 m square) |
| the hollow's deepest | **−0.95 m**, where the jet crosses the ridge | gone |
| the drawn mean, 5–8 m inside the window (125k labels, two instants) | Euler **−0.058** | **+0.008** |
| the drawn mean, 8 m and deeper outside the window (cascades only) | Euler about +0.025 (inferred: the after run's Euler − Lagrangian, the chop's sign) | **−0.008** |
| the largest lateral offset in 17 m of water | **14.9 m**, folding the sheet | 3.0 m, no fold |
| the twin's lift at the camera (CPU / GPU) | — | **0.0760 / 0.0756 m** |

The solver was right all along: over the shallows it stands +0.085 m mean and up to +0.21 m (setup, the
water pushed up), and +0.04 to +0.18 m where the jet crosses the ridge. The drawn waves were pressing a
hollow into the sheet five to ten times deeper than that.

Two more A/Bs, each three kernels (HEAD, now, now with the lift off) on one binary and one wave cache:

| Euler mean | HEAD | now | lift off |
|---|---|---|---|
| shoal, 14:00Z Oct 23 (high tide +1.04 m), 1–2 m | **−0.255** | +0.026 | −0.010 |
| shoal, 14:00Z Oct 23, 5–8 m (104k labels) | −0.032 | **−0.002** | −0.036 |
| Salisbury Beach (cascades only), 1–2 m | **−0.368** | −0.134 | −0.084 |
| Salisbury, 0.5–1 m | −0.194 | −0.190 | −0.170 |
| Salisbury, 3–5 m (78k) | +0.074 | **+0.011** | +0.074 |
| Salisbury, 8 m+ (68k) | +0.025 | **−0.022** | +0.025 |

* The hollow was not the jetty's alone: the same law sank Salisbury's surf (−0.37 m), and outside the
  window the cascades' chop stood the deep sea 2–7 cm *above* the level (their sign).
* In every deep bin the lift cancels the measured E[η ∇·D] to 87–100 %. What remains is the patch's
  own wave mean at one instant (a 600 m square holds three of band 0's wavelengths).
* A run of the same instant whose solved pages had not landed (the cascades owned the inlet) read the
  bar top at +0.11 to +0.22 m. Residency at frame 300 is not deterministic (priors 9); the A/B above
  has the solved field in all three arms.

**The causes, each a normalization** (`WaterBank.hlsl` CsBankFill; `TreeWater::Finish` and
`WaveField::ProbeAt` the twins):
1. *The cascade crest law judged the sea it does not draw.* Inside the solved window the cascades draw
   only the chop (bands 0 and 1 yield by wCas). The block's envelope was the whole sea's, so the crest
   weight compared the chop's elevation against the swell's envelope and sat at ½ at every phase. The
   lip's fall then came off as a smooth offset wherever the swell's reach passed the cusp. Now the
   envelope and wavenumber are the drawn bands': the window's yield and each band's fold (on a coarse
   ring the chop leaves geometry for σ², and the same fault came back).
2. *The lateral amplitude was divided by the breaking factor twice.* The loop draws `amp coth / brk`,
   which is the raw gain times coth. The bound divided by brk again, overstating the reach exactly where
   the depth limits the sea. Now it is the raw orbit, once.
3. *Units.* `gRmsRef` carries the 1.15 exaggeration while `d` is physical until `d *= dry * gPatch.w`.
   The envelope is now taken back to physical. The CPU twin had done the law in drawn units, so the two
   twins disagreed by the exaggeration there; both are physical now.
4. *The solved field's set-crest excess was clamped before it multiplied.* `max(excess, 1) × crest /
   envelope` forgot how far under its limit a sea stands. Any crest over the significant one read as
   over the depth's limit, even in 17 m of water. Now it is `excess × crest / envelope`, unchanged
   wherever the sea breaks (excess ≥ 1). The CPU twin ran the law only where the sea broke, so the hull
   never felt those throws; it now runs wherever the kernel does.

**The lift** (Mark: "so the drawn sea's mean equals the solver's level"). The sheet is Lagrangian, so its
surface stands E[η ∇·D] below its labels' mean. That is −½ k a a_h a component for Gerstner's sign (the
solved field) and +½ k a a_h for the cascades' texture, whose chop points away from the crests (the
2026-09-08 sign finding, still open). Every label is lifted by Λ = −E[η ∇·D], at the weights it is drawn
with. The derivation and the measurement are in ALGEBRA.md, physics: "The drawn sea's mean level". The
cascades' moment is the texture's own, `OceanCpu::LiftMoment` (λ Σ 2k(|A|² + |B|²)), sent as `gBandLift`.

**Found on the way, not changed:**
* *The twin tool stood at (0°, 0°).* `--twin-surface` never gave its TreeWater a chart, so its CPU read
  the Gulf of Guinea's bed (−4935 m). The vertical comparison inside the window could not see it, because
  the solved field's height does not read the depth; the lift's coth(kh) could, and came out 1.6× apart.
  Fixed in the tool (`SetChart`, `SetRootChart`, as `Entity.cpp` does for a hull). With the CPU at the
  real place, the far rings compare honestly for the first time:
  * correlation 0.62 at 500 m and 0.41 at 2000 m;
  * the blue-water checks at 120 and 400 km read a bed of **exactly 0.0 with coverage claimed**, the
    absent-height signature the check exists to catch. Open.
* The dry band `smoothstep(0.05, 0.65, depth)` and the trough's tanh bound are still rules. Neither
  moved the mean measurably here.

**Open, for Mark:**
* *The lip's fall is a sink.* It lowers crests and raises nothing, so wherever it fires as the law means
  it to, the drawn mean drops. In Salisbury's inner surf (0.5–1 m) that is −0.17 to −0.19 m, under water
  the solver says should be set up. The lift overshoots there by 2–5 cm as well, because the fall and the
  trough bound reshape the crests the moment assumes. A conservative fall puts the lip's water into the
  trough ahead.
* *The surf's orbit.* With the double ÷brk gone, the cascades' lateral in cascade-owned surf is the raw
  orbit the crest law asks for, bounded by the landing: at Salisbury, 1–2 m of water, mean 1.2 → 3.8 m,
  max 3.7 → 11.8 m, no folds. From 450 m it does not show (`out/review/shoal/sal_before_after.png`);
  close in it will. The alternative is the orbit of the broken wave (the lateral on the delivered
  amplitude): broken energy is dissipated, not turned sideways.
* Stills at Mavericks (bowl) and the Newburyport surf, before and after: mean |ΔL| 0.57 and 0.75, under
  1.3 % of pixels past 8 levels. Neither scene's instant has breaking surf in view, so neither tests the
  rolling crests.

(The first two were answered the same day by §12.13.)

### 12.13 One crest law (2026-10-23)

Mark: "give the cascades the solved field's crest law (orbit on the delivered amplitude, the crest-local
throw by the crest's excess and the Iribarren gate from the bank's own bed slope), GPU and CPU twin."

**Why this one.** Both open items of §12.12 had one cause: the cascades ran an older crest law than the
solved field.
* That law put the raw amplitude in the orbit at every phase. ALGEBRA.md priors 53 (c) retired exactly
  that for the solved field ("a sea mid-plunge at every phase").
* It spent the energy the depth takes twice, because the solver already turns it into the push and the
  setup.

The solved field's law is the architecture's own:
* the orbit on what the depth lets stand;
* the throw at the crest alone, by that crest's excess, where the bed says it plunges;
* the fall as the flight's boost back, bounded by the trough.

**What it is now.** One function on each side, `CrestLaw` in `WaterBank.hlsl` and its twin in
`WaterTerms.h`. It is called by:
* the solved block;
* the cascades;
* `WaveField::ProbeAt`;
* `TreeWater::Finish`.

For the cascades:
* the orbit rides the delivered gain times coth(kh);
* the law is judged on the sea they draw (yield × fold), with that sea's own excess against the depth.
  Inside the window that is the chop's, so the cascades throw nothing there; the swell's crest is the
  solved field's.

The Iribarren gate reads the bank's bed one wave-radian (1/k, never under a texel) east and north. It
reads only where a crest is over its limit, so open water pays nothing. The CPU twin reads the hull's bed
(`SetBed`, now handed to the twin tool too).

The old block is gone: the raw orbit held to the landing, the lip's fall at every crest past the cusp,
the bound. The change is net-neutral in lines (+241 / −240).

**Gates** (16:00Z Oct 22, Hs 2.2 m, one binary; pushed = c34f7ee):

| Euler mean (lateral mean / max) | HEAD | pushed | one law |
|---|---|---|---|
| Salisbury, 0.5–1 m | −0.194 (0.4 / 1.9 m) | −0.190 (2.7 / 8.7 m) | **−0.011 / +0.013** (0.8 / 3.0 m) |
| Salisbury, 1–2 m | −0.368 (1.2 / 3.7 m) | −0.134 (3.8 / **11.8** m) | **−0.002 / −0.010** (1.1 / 3.0 m) |
| Salisbury, 2–3 m (a strip under one phase) | −0.274 | −0.086 | −0.092 / +0.071 at +4 s |
| Salisbury, 8 m+ | +0.025 | −0.022 | −0.022 / +0.019 |
| the jetty tip, every bin | (§12.12) | — | the pushed values to the millimetre |
| Mavericks bowl (5 m, 17 s), every bin | 3–5 m **−1.15** | — | the pushed values to the millimetre |

**What the gates say:**
* *Salisbury* (cascades only): the inner-surf sink and the 11.8 m orbits are gone. No bin folds, and
  every bin's two-instant mean is within ±0.02 m of the level.
* *The jetty tip and Mavericks* are solved-owned, so moving the solved law into the shared function
  changed nothing there. At Mavericks, §12.12's fix had already taken out a hollow of −1.15 m over the
  reef (3–5 m).

**At Mavericks the lift is approximate where the crest law throws hard.** The bowl's crests are thrown up
to 35 m and the sheet folds in 13 % of its 2–3 m water, the same as at HEAD. There the measured E[η ∇·D]
and the lift agree to 0.8–1.4× in the large bins, and 3.3× in the bowl itself. The drawn mean's departures
there (+0.16, −0.23, −0.05, +0.10 m) are mostly the patch's own phase: a 600 m square holds three
wavelengths of a 5 m, 17 s swell.

Selftest passes: cb parity, the solved field's water parity, oceancpu.

### 12.14 The boat rode another instant of the sea (2026-10-23)

Mark, in `merrimack_boat.json` at a beach: "the water is not matching what the boat hits."

**The measurement.** `--water-probe` with the boat in Salisbury's surf, where only the cascades draw:
* drawn minus the hull's water swung −0.43 to +0.42 m near the hull, flipping with the wave period;
* the hull's and the bank's waves were 0.8–0.99 correlated near the hull and unrelated 500 m out;
* the gains were identical (`kernel 0.981 0.711 0.826 | hull 0.981 0.711 0.826`), and the raster drew
  what the bank holds (drawn − bank ≤ 0.08 m).

`GAGAME_NO_CASCADE_CREST=1` turns the cascades' crest law off on both processors (`gDebugA.z`), and it
changed nothing. So the crest law was not the cause.

`GAGAME_TWIN_GRID_CPU=1` writes the hull's water (`TreeWater::AtLabel`) beside the bank's on the grid.
Over a 300 m square they had the same power at every wavelength but coherence 0.07–0.29 in every band:
the same sea, another realization.

**The cause.** The cascades' clock. The FFT is fed `simUnix − cycle` (`SeaLayer::SetTime`), and since the
global wave grid feeds the sea (PR 107) that cycle is the GRID's. The hull's `TreeWater::CascadeTime` kept
subtracting the `SeaState`'s, so the boat sampled the same cascades at another instant. The seed's own
comment had warned of exactly this: "right Hs, wrong crests". It only shows where the cascades carry the
swell, outside the solved window, which is why the inlet's twin never saw it.

**The fix.** One expression, `SeaLayer::CascadeCycleUnix()`. The FFT, the seed and every CPU reader take
it: `TreeWater` via `SetCascadeSea`, and the probe's orientation line.

| Salisbury, 16:00Z Oct 22 | before | after |
|---|---|---|
| grid, 300 m square: corr / rms(gpu − cpu) | 0.27 / 0.60 m | **0.990 / 0.078 m** |
| grid coherence (>60, 12–60, 2.4–12 m) | 0.29, 0.07, 0.11 | **0.98, 0.98, 0.97** |
| twin 0 m / 500 m / 2000 m correlation | 0.93 / −0.06 / ≈0 | **0.993 / 0.989 / 0.928** |
| hull probe, 0–10 m: drawn − hull mean | −0.43 … +0.42 | **−0.09 … +0.05** |

**Also in the twin tool:** it now hands its TreeWater the swell shadow and the bed the hull gets.

**Still apart:** the 8 km ring (ratio 1.78, correlation 0.63). The coarse ring folds the bands into σ²
while the hull sums them, and that is by design.

### 12.15 The one law: every breaker to the cusp, the flight by its plunge, the swash (2026-10-23)

Mark: "the waves don't seem to be rolling as they should, before there was water going up the beach and
flowing back down into the ocean and causing the waves flowing onto beach to increase in size and roll."
§12.13 had done it. It retired the cascades' raw orbit (the run-up and the backwash) and gated a crest's
whole going-forward on the Iribarren number, so the Merrimack's spilling beaches (median ξ 0.19) did not
roll. Mark chose the one-law fix (ALGEBRA.md priors 57):

* **`CrestLaw`, both processors.** The crest's forward share is share × [(to the cusp) + PlungeShare ×
  (from the cusp to the landing)]. Every breaking crest goes forward to the cusp (k A_h = 1), and the jet
  flies on only where the bed plunges. At PlungeShare = 1 it is the law of before.
* **The swash, every description, both processors.** The lateral orbit is the delivered one × (1 + S (raw /
  delivered − 1)), where S = exp(−h / R) and R = 0.043 √(Hs L0) (Stockdon et al. 2006, the dissipative
  beaches' R2%).
  * Cascades: per band, raw / delivered = 1 / brk, L0 = 2π / k_band.
  * Solved field: per component, max(excess, 1), L0 = 2π coth / k.
  * The raised orbit is held to the landing reach (the cascades' bound of 2026-10-19), and the lift rides
    the lateral it is taken on. Offshore the delivered orbit never reaches the bound.
* **The twin's bed slope moves into `Prepare`.** bbd1a98 read it in `TreeWater::Finish`, which `AtMany` runs
  across the pool, while `HeightPage`'s memo is single-threaded. That was the crash and the hour-long hang
  of 2026-10-19 come back. Both processors now take its step on the WHOLE sea's energy wavenumber, which is
  the sea that breaks, even inside the window where only the chop is drawn.

| Salisbury, 16:00Z Oct 22, cascades only | §12.12 (raw orbit) | §12.13 | **now** |
|---|---|---|---|
| lateral mean / max, 0.5–1 m water | 2.73 / 8.75 m | 0.78 / 3.02 m | **2.52 / 8.47 m** |
| lateral mean / max, 1–2 m water | 3.81 / 11.79 m | 1.11 / 3.03 m | **2.47 / 8.83 m** |
| drawn mean, 0.5–1 m / 1–2 m water | −0.19 / −0.17 m | −0.02 / −0.02 m | **−0.26 / −0.04 m** |
| offshore (5–99 m) | — | — | unchanged to the mm |
| twin correlation 0 / 500 / 2000 m | — | 0.993 / 0.989 / 0.928 | **0.993 / 0.989 / 0.928** |

**Open:** the inner surf's drawn mean stands below the solver's level again (−0.26 m at 0.5–1 m water). The
swash orbit is past the cusp at every crest there, and water past the cusp falls. The §12.12 look Mark
preferred carried the same (−0.19 m). The grid's fold count reads 0 % in all three, so it is not the
instrument for the roll: Mark's eye is.

**The same evening, Mark at Salisbury's waterline:** "looks better but not rolling as much as I would expect"
and "the sand on the beach adjusting height every few frames ... it looks like the beach breathes". Three
changes:

* **The spiller's roller** (Jet.hlsli / WaterTerms.h `SpillReach`; Mark: "now that you are in the code where
  water affects the land mesh, you should be able to deduce the right spiller roll method").
  * The shore's own law: water that cannot stand on a slope runs down it, a height dH carried dH / tan θ.
  * A spiller is that law on its OWN front face. The crest over its limit, Hc (1 − 1/e_cr), runs down a face
    of tan θ = k Hc / 2, reaching 2 (1 − 1/e_cr) / k past the cusp, and drops linearly along it.
  * A plunger's jet flies to the landing and falls on the parabola. `PlungeShare` mixes the two.
  * At PlungeShare = 1 it is the law of before.
  * Duncan (1981) measured the roller riding a ~12° face, and field roller slopes run 0.05–0.35. That is why
    the face here is the crest's own rather than a constant.
* **Each band runs up by its own run-up:** its own raw Hs and L0 (the solved field's components by their own
  2a). The chop is broken in the surf and runs up nothing.
* **The beach's breathing.** "Wet" was the solver's threshold (h > 1 cm), and the drawn level (`SolverSurfaceDev`
  and its twin) switched with it. A shoreline column crossing 1 cm moved the level by the bed's rise across its
  cell, and the shore's classifier (GlobeMesh.hlsl, "the shore at the water's own level", dbd25c0) moved the
  sand's vertices between land and water height with it. Now a column speaks by the water it holds:
  ω = h / (h + h_wet), with h_wet the solver's own wet depth (`gSweB.y`). The corners speak their share and
  the ring takes the rest, sliding to the dry floor as its water goes. The level is continuous in the water;
  wherever the water is real depth, it is unchanged.

| Salisbury, 16:00Z Oct 22 | 0.5–1 m water | 1–2 m water | 2–3 m water | 5–99 m |
|---|---|---|---|---|
| folds (the sheet overturned) | 0 → 0 % | 0 → **1.3 %** | 0 → 0.1 % | 0 |
| drawn mean | −0.26 → **−0.11 m** | −0.04 → −0.08 | −0.13 → −0.15 | unchanged |
| hull vs drawn correlation | 0.92 → **0.39** | 0.65 → **0.94** | 0.94 → **0.98** | 0.99 |

**The land still moved with the camera still** (Mark: "the land heights still seem to be adjusting when the
camera is still", with his run's log). The log made the case:
* the residency pool full (32,379 of 32,768 tiles), with a want set of 33–36 k cut at its tail each pass
  ("2^-2 m: kept to 279 m, lost from 272 m");
* the walk re-made on 7,397 of 7,657 frames, and "a reader said something else -- view" on 2,777 turns;
* 73,598 reads and 426,939 fade steps in 7,676 turns.

The path:
1. The bank's fold grid (`m_fold`) was cleared and re-marked every frame with the sheet's folds of the
   instant.
2. The walk refines the land's mesh below level 18 near those folds (`GlobeLayer` `SplitAt`, the digest's
   "the sea's folds"), and the vertex morphs on their distance (`CsFoldDist`).
3. So the refined region, the morph, the view's wants and the residency's cut all moved with every crest,
   and the land near the surf re-drew its heights each frame.
4. The column ladder's tile mapping followed the same flags (the log's `column ring -N: ... mapped under a
   fold` every few frames).

Now a fold is HELD for two of the sea's peak periods (`SeaLayer::PeakPeriod`):
* the grid keeps the tick a block last folded at (`InterlockedMax`; `gFoldClock`);
* `CsFoldDist` and the walk count the blocks that folded within the hold;
* the grid restarts only where ring 0 re-anchors or the clock is set back;
* the ladder holds its tiles alike.

The held set moves as the surf zone does, with the tide and the sea. A still run at the bar under a
declared swell: 405 walks made, 795 kept, the view's wants changed 16 times in 1,223 turns (that run was
still loading the scene).

`scenes/demos/merrimack_bar_swell.json`: the bar's clocks and views under a declared east-northeast
groundswell (`sea.storm`: Hs 2.5 m, Tp 11 s, from 70°). The bar is judged at a size that breaks over it, not
at the day's buoys (0.3 m on 2026-10-23).

**Open:** the swash's twin (0.5–1 m water, rms 0.11 m). ξ there sits on the spill/plunge boundary (~0.3–0.4),
and the two processors read the bed's slope from different caches (the hull's HeightPage, the bank's
resident chain). So the fall's shape, the jet's parabola or the roller's slope, differs between them. A boat
there is aground. The fix is one bed for both readers (the residency read of 2026-10-23, "two beds").

### 12.16 The storm bar and the one text (2026-10-25)

See docs/WATER_LAWS.md: the flat bar under the 3 m storm was the action transport dropping every ray
convergence as a crease (80.9 % of the swell), fixed by conserving action (kSolverVersion 15); the solved
field is no longer sheltered twice (the exposure page is the cascades' closure); and every closed-form law
of the water is one text under `shaders/laws/`, compiled by the kernels and by `WaterTerms.h`.

## 13. The asteroid (2026-10-25, Mark: "a massive impact in the middle of the Atlantic; two recordings")

Built on PR 109's branch the evening the laws became one text. Section 6's ladder said how a tsunami
reaches Newburyport; this is its first rung as ONE LAW with no stepping, and the two eyes Mark asked for.

### 13.1 The law (`shaders/laws/Impact.hlsli`, one text for the GPU and the CPU)

Ward & Asphaug (2000, 2003): the impact is a transient CAVITY, a parabola of depth D_c and radius R_c
with a lip out to R_D = √2 R_c holding its water (net volume nought); its evolution is linear dispersive
theory over a basin of one depth h0:

    η(r, t) = ∫₀^∞ F(k) J₀(k r) cos(ω(k) t) k dk,     ω² = g k tanh(k h0),
    F(k)    = D_c [ R_D J₁(k R_D) / k − 4 J₂(k R_D) / k² ]      (the lipped parabola's Hankel transform)

evaluated as a SUM over a table of wavenumbers the host builds once (`sim/ImpactWave.h`: n terms from
0 to 12 / R_D, A_n = F(k_n) k_n Δk). It holds at every r and t: the cavity at t = 0, the first ring,
the dispersive train behind it. The discrete sum's image returns at r = 2π/Δk, so the terms follow from
the REACH the scene asks (`reachM`, 6000 km by default): n = 12 reach / (π R_D), at most 4096 -- 880 for
the 10 km impactor, 1600 for a 1 km one, 4000 for 300 m. (512 terms held a 1 km impactor's sum to
3900 km only, and the image's wave stood at the Merrimack, 2625 km out, before the front: the numpy
check showed ±50 m at the faces an hour early. The terms are the law's cost, a vertex and a texel each.) Near land the wave shoals by Green's law from h0 to the depth
there, BOUNDED by the shore's saturation with one odd smooth law, a = b tanh(G η / b), b = γ h / 2
(`Breaking.hlsli` kGammaHs: Hs ≤ γ h): a linear reader's wave stands no taller than a breaking one,
and the solver's columns say the rest. The first bound was the depth itself; from orbit a trough at
−h drained the coarse globe's banks to their bed and drew them as sea floor (the slabs of the first
clip).

The cavity from the impactor: the diameter by the pi-scaling form (Schmidt & Holsapple; Collins, Melosh
& Marcus 2005 for rock) with its constant FITTED to W&A 2003's published case (1100 m, 2.2 g/cc,
17.8 km/s → 18.9 km across: C = 1.374); the depth by the energy argument (9.4 % of the impactor's
kinetic energy as the lipped parabola's potential energy π ρ g D_c² R_c² / 3), capped at the ocean's
depth. Checked in numpy against W&A's figures for 1950 DA: 14 m at 800 km after an hour (×4.7 by
Green's law to a 25 m beach: their "120 m from Cape Cod to Hatteras"), 3–5 m at 2800 km (their "about
9 m mid-ocean"). The Bessel functions are Abramowitz & Stegun's polynomial forms, 5e-8 absolute,
checked against the integral definition.

### 13.2 The record of events (Mark: "a player or some director can cause impacts at will")

An impact is an EVENT -- an impactor, where it lands, the instant it does -- and the water carries a
RECORD of them (`sim/ImpactField.h`), appended to at any instant: the scene's `impacts` list at boot
(each with its own `at`, or the water clock's first instant), the I key at the point under the cursor
(`sea.impactor`, at the clock's instant; the globe's sea-level sphere from altitude, the ground below
it), and anything else that holds the field. Each event keeps its own clock zero and its own table;
the CPU sums every event's law, and the GPU readers sum the same tables from ONE buffer the field
packs when the record changes (a header of eight words an event -- direction, zero on the record's
base, basin depth, exaggeration, its terms' place and count -- then the terms; `shaders/ImpactRead.hlsli`
reads it in the globe's vertex stage and the bank kernel, through the bindless uint buffers). A
retired buffer lives until the run ends: the frames in flight read it. The record is replayable: a
second player handed the same events sees the same sea. Hot-reloading the list is not here (a new list
element is "needs a restart" to the scene reload, by its own law); the key is the live path.

The exaggeration is applied to the DEEP wave before the shoaling's bound, so a wave over a shelf
stands no taller than the shelf's water whatever the exaggeration: the first orbit clip exaggerated
after the bound and drew the Gulf of Maine's banks as flat slabs ten times their depth.

### 13.3 Where the law is read

| reader | how |
|---|---|
| the hull, the twin | `WaterManager` Level producer `impacts` (SUM with the tide); `ImpactField::EtaAt` sums every event's wave with the depth from the height stack |
| the solver's open faces | `SweSolver::SetLevelSource`: tide + the impact's level shoaled to the face's depth by Green's law UNBOUNDED (the solver is the nonlinear reader; the saturation bound is the drawn far field's and the hull's -- bounded, the asteroid's surge at the Merrimack's faces was six metres and the town stayed dry); the solver carries it inland and over the land |
| the bank kernel (`CsBankFill`) | the same tables from the record's buffer (`ImpactRead.hlsli`); the texel's lat/lon (`TilePlace`) as a direction, the sum, the shoaling — added to the atlas level wherever the solver is silent |
| the globe's vertices (`GlobeMesh.hlsl`) | `ImpactAt(dir, depth)` past the rings: the level, and the slope east/north into `VsOut.imp`; inside the rings the bank's level plane carries it |
| the far sea's pixel (`WaterPixelColor`) | the level and the normal's tilt from `imp` past the rings (inside them the ring's slope) |

`exag` exaggerates the DRAWN wave alone (the globe and the bank); the hull and the solver read it raw.
The sum costs n Bessel evaluations a vertex and a texel per event (512 terms by default), nothing while
no event has begun.

### 13.4 The two scenes

* `scenes/demos/impact_orbit.json` + `scenes/rails/impact_orbit.json`: 10 km at 20 km/s into 35 N
  42 W at 15:00Z, the clock at 240×, the eye 3000 km over the cavity then west with the wave; the
  drawn wave 10×. The cavity is 110 km across and 4000 m deep (the energy says 31 km: the ocean caps
  it); 2.95e22 J ride the wave.
* `scenes/demos/impact_newburyport.json` + `scenes/rails/impact_shore.json`: the same event; the first
  wave reaches the Merrimack's mouth (2625 km) 3.68 h after it at √(g h0), 18:41Z; the scene starts at
  18:20Z and runs 30× from the Plum Island dune by the north jetty, then up to 400 and 900 m over the
  river. No exaggeration. By the law the faces see ±12 m before the front and hundreds of metres at it
  (Green's law ×4 from the deep train's ±200 m): a 10 km impactor drowns everything in this window.
* `scenes/demos/impact_newburyport_1km.json`: the same shore and instant under a 1 km impactor (1950
  DA's class, Ward & Asphaug 2003): tens of metres at the faces, the dunes and the low town under, the
  high ground standing -- the water washing over the land in view, which Mark asked to see.
* `scenes/demos/impact_descent.json` + `scenes/rails/impact_descent.json` (Mark: "like the orbit one but
  two minutes, the orbit going all the way down to a bird's eye view following the waves crashing onto
  land"): the 1 km event on ONE clock at 170×, so the front reaches the mouth at 78 s of 120 and the
  train's peak near 106 s, the eye 40 km over the inlet to meet it and 3 km over Plum Island through the
  peak. The solver's window is on; the drawn far wave 10×, the solver's water raw. It was the 10 km
  event at 130× first: the solver's step collapsed to nothing at that surge (a face velocity of 92 km/s
  in the log; the same event at 30× it survived at 350 m/s) and the record ended on a dry shore.

### 13.5 The rolling wave on the coast (Mark, 2026-10-26: "if the impact is big enough")

A surge's front over the marsh is a BORE: a hydraulic jump marching inland. Two things make it a wave to
the eye, its whitewater and its face. The whitewater is one law (`Breaking.hlsli` BoreFoam): Belanger's
jump from a column h1 to a column h2 loses the head (h2 − h1)³ / (4 h1 h2), and against the deeper
column's own head that loss is the share of the water's energy turned to turbulence — nought at a tide's
creep (Δh³ over metres of water), whole at a surge's wetting front. The bank kernel reads the solver's
columns a cell behind and a cell ahead along the flow and takes that fraction into the foam, ungated by
the drawn sea's crests (it is not one of them); the churn remembers it and the pixel stage textures it
as every foam. The face is the solver's own (its shock spans two or three cells), and it ROLLS by the
same crest law as every crest (Mark: "feed the bore's face into the column plane so the surge's front
overturns"): the bank kernel makes a crest of the front — its elevation across the front as the crest's
eta about the mid-height, its half-jump the significant amplitude, three cells the wavelength, the jump
over the depth's limit in the water ahead its excess, the bed's slope along the flow by the same height
chain the cascades read — and `CrestLaw` throws its labels: to the cusp at the top of the face, down the
face as the roller where the bed is flat (the marsh: spilling), into the flight where the bed is steep
(the dune: plunging). The throw is added to the sheet's own displacement; the column plane folds where
the labels overtake and MsLip draws the lip. No law of the land (Mark: "the voxels should get pushed and
pulled along the topology like an extension of the seabed"): the bed is one surface and the columns
ride it, and a creeping sheet throws centimetres. Not mirrored on the CPU twin yet: the hull rides the
solver's level, not the bore's throw.

What the clips of 2026-10-26 showed, stated: no rolling face yet, at any of four events. The mid-Atlantic
1 km event rises at the Merrimack over minutes and floods the marsh as a thin tongue (a wetting front over
dry ground has no face: a dam-break's tongue); the same impactor 800 km out swings the sea side −110 to
+130 m and swallows a 15 m eye; a 500 m one 800 km out (`impact_bore.json`, the eye 80 m over the jetty
looking to sea) raises the sea +5 to +23 m over ten minutes in swings of a few metres, each crest of the
train a kilometre long with a slope of a hundredth: nothing the crest law throws far. A bore steepens
over a LONG shallow run (a river's tidal bore, the Qiantang's); this window's shelf is five kilometres,
and the train's crests reach its faces as long gentle rises. The rolling face wants the shelf rung (the
Gulf of Maine at 250 m, section 6's rung 2) carrying the train in nonlinear shallow water until its
crests shock, and the window's faces fed by that. The mechanism at the face is ready for it.

### 13.6 The shelf rung (2026-10-26, Mark: "build the shelf rung")

Section 6's rung 2, as the engine already had the pieces: a second solver window over the Gulf of Maine
(`solvers` in the scene: 1600 × 1150 cells of ~260 m from 41.75 N to 44.6 N, 71 W to 66 W, the entrance
station's tide), asleep until the eye comes under 30 km inside its box, its bed the one height tree at its
own level (Mark: "coarse and fine heights should be part of the same mega mega-texture" — the standing
window where the page holds its cells, the tree's coarser levels elsewhere; no second bed, no coarse cache
of its own; the bed before the water, as finding 48 has it). Its open sea rides the atlas plus the
asteroid's wave by Green's law to the depth there (`WaterManager::SetLevelDeparture`), unbounded: at the
Gulf's mouth that is ×2 of the deep wave over 200 m of water, and from there the solver's own nonlinear
shallow water carries the train 120 km across the Gulf, shoaling and steepening it. The Merrimack window's
faces read the shelf's level (`WaterManager::OwnedLevelAt`, from a region about the window the frame loop
asks for every frame; a demanded owned window steps on the frame), the analytic wave only where no shelf
answers. One-way nesting, level alone (no transport yet). The owned solver takes the scene's step budget.

Its faces take the BOUNDED law (the shore's saturation): the box's landward sides cut coastal water, and
Green's law unbounded into a few metres of depth forced those faces by ten times the deep wave and blew
the solver up (820 m/s on 250 m cells; the Merrimack's faces, fed that noise, followed). At the Gulf's
mouth, 200 m deep, the bound stands far above the wave. Newburyport's station carries no NAVD88 link in
the tide data, so the window rides Boston's.

Stated: the shelf activates on the first frame, after the Merrimack's own spin-up, which therefore ran on
the analytic wave; the shelf spins up `water.swe.spinupH` before the start, so a scene starts after the
train has entered the Gulf (the bore scene: the 500 m impactor's front enters at 15:50Z, the scene starts
16:45Z with a 1.5 h spin-up). The bank kernel's far field outside the Merrimack window still draws the
bounded analytic wave, not the shelf's level: a seam at the window's edge in the drawn sea.

What the bounded take showed (05:30, 2026-10-26): the shelf solver stable (a 1.5 s step, 8 m/s at 450 m of
water), and the Merrimack's sea side fed ±30 to ±60 m by the train it carried, the four points along the
face reading a crest crossing obliquely; the Merrimack solver answering with 90 m of water in the inlet and
190 m/s at its front. The picture from 80 m over the jetty does not yet read as a rolling wave: the sea
rises and falls as a whole under a fixed eye with a five-minute period, the ordinary swell's striations
dominate the sheet at that height, and the bore's throw (a cusp's length, 1/k ≈ 13 m) is a line from
there; the whitewater shows as patches. The eye for the roll is a few metres over the bar in the seconds
the face arrives, with the column plane at ring 0 under it — a rail timed to the train's crests, which the
instrument now prints a minute apart. The physics of the rung is in place.

### 13.7 What is not in this rung, stated

* One depth for the propagation (the dispersion's h0): the shelf's slower speed and its refraction
  (rung 2, the eikonal per frequency) are not here; the arrival at Newburyport is the deep-water
  clock's, early by the Gulf of Maine's share. Green's law takes the depth's effect on the AMPLITUDE
  only.
* The cavity's cliff at R_D (the lip's edge) is under-resolved by the table (Gibbs ringing in the
  cavity's own profile at t = 0: ±8 % of D_c); the far field is unaffected.
* No reflection, no refraction, no breaking at sea: linear theory, as W&A's.
* The classifier on the globe past the solver reads the raised level as the waterline (the coarse
  globe's coast moves with the wave): the inundation picture from orbit is the mesh's, not a solver's.
  Inside the solver's reach the survey's shoreline used to stand under 18 m of water (the mesh drew
  teeth along the river: land vertices at their height beside water at the flood's). Mark, 2026-10-25:
  "think of the water layer as always being a global sparse voxel tree; we might not have water in all
  of its places, but it's there for water to flow into" -- land is a column with no water in it, and
  the survey names the shore's shape at its datum, not the water now. So the classifier takes THE
  COLUMN'S OWN ANSWER wherever a ring texel holds one: the bank's detail plane carries each column's
  water weight (`DryWeight` of the solver's surface over the bed where the solver speaks), the mesh
  hands it to `ComposedLandnessEdit` (`columnWet`; the pixel through `VsOut.colWet`), and a column
  holding water is water whatever the survey said, with no band (a first cut used a 3 m band; Mark:
  "why more than 3 m? we have everything designed to be sparse"). One direction only tonight: a
  column's emptiness does not make land of the survey's water, because past the solver's domain the
  bank's weight is the coarse bed's against the tide and the bank carries no domain weight to tell
  the two apart. Next: the solver's domain weight in the bank, so the column decides both ways where
  it is stepped, and the stateless column (tide over bed) past the rings.
  Mark, on the classifier itself: "why do we care about wet/dry other than the visual look of wet
  sand?" -- the answer is that today three things read it (which height a vertex takes, which shading
  runs, the swash band), and under the column law the first two are the column's depth (the drawn
  surface is max(bed, surface); the shading is whether the column holds water here) and only the third
  is visual, a memory on the column like the foam's. The classifier is to collapse to that.
* The solver STARVED at 30x: `SweSolver::kMaxSubsteps` (16 a frame since M5) is the budget past which
  its clock re-anchors and its dynamics run time-dilated -- the surge rose at its faces at the scene's
  rate and the water inside answered at a fraction of it, so the first beach clips stayed dry through a
  600 m surge (the probe with the spin-up's batch, which steps whole, drowned the eye). The budget is a
  scene value now (`water.swe.stepsPerFrame`): the beach records say 256, the descent 512.
* A face point on land (the side's cubic asks four points; a window's corner can be land) took Green's
  law at the depth floor, ×12.8: −1400 m at the north side's west end. A point with no water column
  takes the deep wave as it is.
* A small cavity's train peaks well after the √(g h0) front: the 1 km impactor's deep wave at the
  Merrimack is 1 m at 3.7 h, 3 m at 4.0 h, 9 m by 5.2 h (the dominant 60 km wavelength's group speed
  in 4000 m is 0.9 of the long-wave speed); a 512-term table (alias at 3900 km) had shown ±50 m at
  3.3 h, the image's wave. The 1 km beach scene starts at 19:45Z.
* THE SOLVER UNDER A SURGE FAR TALLER THAN ITS WATER: the 10 km event's faces rise hundreds of metres
  over a 15 m column; at 30× the scheme carried it (velocities to 350 m/s, the step 0.012 s), at 130×
  its step collapsed to nothing (92 km/s at a face) and the water never moved. The Flather face's flux is
  positivity-clamped; the blow-up is inside. The open item for a forcing of that class; the 1 km event
  (tens of metres at the faces) is carried at both clocks.
* The wave solve re-keys on the solver's current every `water.swe.currentRefreshS` (90 s since F13): a
  scene run at 30x re-solved 46 times in an hour of record, each a three-minute solve a headless
  capture waits for. The beach scene holds the first current for its length; the cadence is a scene
  value now.

## 14. One water on the tree (2026-10-26, Mark: "big refactor time ... we deviated from the Cayley structure that would allow better water simulations at fine and coarse scale without ad hoc solvers")

### 14.1 The deviation, named

SPARSE_GA.md's own tests: the type IS the algebra (a product's grades from the Cayley table, at compile
time); a product's residency comes from the Cayley closure, `And(algebra, physics)`; one reserved
resource with mips, so that ZOOM CHANGES WHICH TILES ARE RESIDENT, NEVER WHICH CODE RUNS; the packed
tail pinned as the floor a miss cannot fall through. The wave field passed into that structure (stage 2,
byte-identical). The water's solvers did not:

| piece today | how it deviates |
|---|---|
| `SweSolver` on a fixed lat/lon box (`water.swe.box`), its sponge, its Flather faces at the box's edge | a dense window where the tree has levels; the box is a policy, not residency; the open sea is a special case at a line the scene drew |
| the dormant windows (`solvers`), `AddDormantWindow`, the activation altitude | more boxes, awake by a camera rule, each its own bed binding and tide station |
| the shelf rung of 13.6: `SetLevelDeparture`, `OwnedLevelAt`, `RequestRegionOwned` | one box nested in another by a level lambda and a per-frame region; level alone; a seam the bank cannot see |
| the impact's Bessel sum + Green's law at every reader (13.1-13.3) | an analytic law stitched to the solvers at their faces, bounded here and unbounded there, with a second shoaling (Green) the solver would do itself |
| the bank kernel's level: `lerp(atlas + impact, solver, wDom)` | the stitching, drawn |

Every edge case of 13.4-13.6 (the starving budget, the bound at the coastal edges, the land point's
Green factor, the spin-up order, the station's datum link, the seam at the window's edge) is a seam
between two of these. The structure that removes them is the one the rest of the engine already stands
on: the column field on the tree, one law at every level, residency as the extent.

### 14.2 The object

**The column field.** One `GradeBank` on the planet's tree, mipped, the one reserved resource: per cell
the water the column holds, h (grade 0), and its discharge, q = h v (grade 1). Two grades in one bank
(SPARSE_GA.md 7: many GA objects per body; the current-field loader already carries a vector). The bed B
is the height tree at the same cell (the same mega-texture: "coarse and fine heights should be part of
the same mega mega-texture"); the level eta = B + h is an EXPRESSION, never stored (`Field<kG0> + Field<kG0>`);
wet is h > 0, the classifier of 13.4 become the datum itself. A NULL tile is absence: no column there
holds water the tree knows of, and the stateless sea answers (the tide atlas over the bed); the pinned
tail guarantees every point resolves.

**The law.** Shallow water in geometric calculus on the column field, written once:

    dh/dt = -div q
    dq/dt = -div(q q / h) - g h grad(B + h) + F

with grad and div the vector derivative on the tree (`GradExpr<kG0>` is a scalar's gradient; the
expression algebra already types it), F the forcing (wind stress, the waves' push -div S of 12.3, the
rivers' discharge as sources at their cells, the soil's intake as a sink). The KERNEL BODY is
`Swe.hlsl`'s as it is (Stelling & Duinmeijer's momentum advection, the hydrostatic face depth,
positivity by the donor's water, wetting and drying, the soil), run over the RESIDENT LIST of a mip, as
every `GradeBank` fill is (a fill dispatches from `ResidentList()`, never over the full domain). The mip
is the scale: a cell at mip m is 2^m of the finest; its step is its own CFL; a coarse mip takes one step
while the mip under it takes two (Berger & Oliger's subcycling, the standard of adaptive shallow water:
GeoClaw, TRITON-G's tree refinement). The same code at every altitude.

**The faces between levels are the tree's own laws** (HIERARCHY.md 4.20, the five relative laws): a
coarse face bordering fine cells takes the SUM of the fine faces' fluxes (the fold: conservative
restriction); a fine face bordering a coarse cell reads the coarse state magnified (the magnify law);
the state of a coarse cell under fine ones is their sum (the fold again, after each fine step). No
Flather face inside the resident set. The ONE boundary is the frontier of residency, wherever the
resident set ends, at whatever level, and there the neighbour is the stateless sea: the Flather law
once, written against the atlas's level and transport at that point. It is the same Flather the window
has today; what changes is where it stands (the frontier, not a box) and that it is the only one.

**Residency is the extent.** `DeriveDemand` as SPARSE_GA.md 13 has it, `And(algebra, physics)`: the
algebra bounds where h can be non-zero (a column whose bed stands over the stateless level and holds
nothing is NULL); the physics says which of the inside is carried at which mip: the eye's demand (fine
at the inlet, as the tiled atlas asks for texels) and THE WATER'S OWN (a step asks for the cells its
fluxes will reach next step: 12.3's "residency follows the water", now the law of extent, not a window's
growth), so a tsunami's front demands its residency ahead of itself at the mip its wavelength needs.
The pinned tail is the planet's water at the coarsest mips, always resident and always stepping: the
basin solve is free, and nothing arrives from nowhere.

**The sources are initial conditions and forcing, not solvers.** The impact of 13 becomes the cavity
IMPOSED on the column field at t0 at the mip its radius needs (R_D over 8 cells), and the law carries
it; the Bessel sum stays as the dispersive CHECK of the far field (the Hankel law is exact linear
theory; shallow water is not dispersive: a 10 km cavity's train loses little, a 300 m one's loses the
dispersion that spreads it; a Boussinesq term is a later grade-2 correction to F, stated, not built).
Green's law goes: the solver shoals. The tide is the stateless fill and the frontier's exterior. The
rivers are sources. The events' record (13.2) stays: an event writes its cavity into the field.

**The readers do not change their contract.** The bank kernel, the twin, the hull, the globe read the
column at the texel's level ("one water, two readers"): h and q where a tile is resident, the stateless
sea where none is. The `lerp(atlas, solver, wDom)` goes, and the seam with it.

### 14.3 What goes, what stays

Goes: `SweDomain`'s box and sponge, `water.swe.box`, `solvers`, `AddDormantWindow`, the activation
altitude, `SetLevelDeparture`, `OwnedLevelAt`, `RequestRegionOwned`, `SetLevelSource` as a box's
exterior (it becomes the frontier's), the impact's readers in the bank kernel and the globe
(`ImpactRead.hlsli`, `ImpactAt`), Green's law, the shoaling bound, `stepsPerFrame` (a mip's step is
its CFL; the frame carries the mips it can and the rest starve honestly at the coarsest, never the
finest), the shelf scene key. (Stage 2 took the box, `solvers`, `AddDormantWindow`, the activation
altitude and the shelf rung's three verbs; the sponge went with the one edge law on 2026-10-27, below;
`SetLevelSource` is the window's exterior still, the frontier's when the tail is the planet's.)
Stays: `Swe.hlsl`'s flux and update laws, wetting and drying, the soil, the rivers' faces (as sources),
the waves' push, the churn, the bank and its column plane, the crest law and the bore's face (13.5),
`ImpactWave` as the source's cavity and the far field's check, the events' record, `laws/`.

### 14.4 Stages, each with its gate (self-tests are a privilege: one run before, one after)

| stage | what | gate |
|---|---|---|
| 0 | the column field as a `GradeBank` with mips over today's Merrimack extent; the kernel over the resident list of mip 0 instead of the box; the frontier's Flather at the resident set's edge (which IS the box, today) | the storm rail's `--water-probe` twin match byte for byte against today's window; the bar still |
| 1 | mip 1 around mip 0 (the surround, the Gulf's near shelf at 2x), the fold and magnify faces between them, subcycling | the tide through the inlet: the anchor's level within a centimetre of today's; the prism by continuity closes |
| 2 | the extent by demand: the eye's and the water's; the tail pinned and stepping; no `solvers`, no box | Boston Harbor floods with the eye there and no scene key; the wave probe at the Merrimack unchanged |
| 3 | the impact as the cavity imposed at its mip; the train crosses in the tail's solve | arrival at the Merrimack within 10 % of sqrt(g h) from the Bessel law; amplitude at 2600 km against it within its dispersion |
| 4 | the bore at the mouth: the front demands its own residency; the eye low over the bar | the clip Mark asked for on 2026-10-26 |

Stage 0 is the one that proves the shape without changing a picture; it is where to start.

**Stage 0, built (2026-10-26).** 0a: `SweSolver::m_h`, the water each column holds, is a `GradeBank` of
grade 0 with as many mips as take the extent to one tile, in the one reserved resource; stage 0 fills
mip 0 alone and the kernels run over its resident list as they did. 0b: the kernel's exterior is
RESIDENCY'S EDGE, not the lattice's edge row -- `ExteriorLevel` answers for any cell no state tile holds
(the tide plane as its level, the megatexture's bed as its sill in `FlatherFlux`), while the edge rows
keep the side's cubic and the rivers' faces. The gate: `--dump-water-state` on the bar scene under the
storm, 60 frames, before and after each half -- level, u, v and the shadow IDENTICAL byte for byte
through both (the bed dump differs between any two runs: it reads the height stack at the dump's
instant, 56,538 of 126,000 texels by up to 4.3 m, the loads' race). On this scene no live cell borders a
NULL tile under its own surface, so the frontier never spoke; it is the law stage 2 extends the
resident set against. Not in stage 0: the discharge as grade 1 in the same bank (the staggered faces are
its home; stage 1 decides whether the vector lives on faces or at centres with a face reconstruction),
the chain's reduction for h (the fold is a sum over the fine cells' water: `BuildChain` averages, so the
coarse mips must be written by the solver's own fold, stage 1), and a frontier cell's departure beyond
the tide plane (the impact's, the shelf's: stage 3 makes them the field's own).

**Stage 1a, built (2026-10-26): the coarse twin.** Every per-cell bank carries the chain's mip 1 and the
same kernels step the same lattice at mip 1 -- cells twice the size, the whole box resident (35 tiles),
its own constants (the lattice halved, the faces' runs halved, the cell steps about the same anchor), its
own lists and views (the twin's tables read the same height page: without those two SRV slots the bed fill
wrote zeros, the first probe's lesson), half as many steps of twice the size over mip 0's interval (the
subcycling), uncoupled from mip 0. Gate: mip 0's level, u and v byte-identical to the baseline through
the twin's addition; the twin's level at the anchor against mip 0's, said once a sim-minute through the
hour of spin-up under the ebb: 2 to 5 cm under mip 0 (its bed from the page's coarse mip is 6 cm deeper;
its own faces, its own prism). That is the uncoupled coarse solve's discretization, what 1a can show; the
centimetre gate is the coupled system's (1c, 1d), where the fine solve sets the level inside and the
coarse only feeds its faces. The twin costs a quarter of mip 0's cells at half its steps.

**Stage 1b, built (2026-10-26): the fold.** A coarse cell under four resident fine cells IS their fold
(`Swe.hlsl` CsSweFold): the water their mean (the column's water per area), the bed their mean (so the
level's mean follows), a face's velocity the mean of the two fine faces on it (the flux-weighted fold by
the faces' water is 1c's). It runs each batch over the coarse tiles whose four children are resident,
after the twin's bed fill and reset and before its steps, through a fold table per parity pair (the fine's
newest velocity bank, the twin's read bank) that puts the fine banks at mip 0 in the kernel's read slots
and the twin's at mip 1 in its written ones. The folded tiles are the fine's: the twin's flux kernel
reads them (their faces to the outside are computed), its height and limit kernels leave them (stepped,
the twin drifted 17 cm from the fine at the anchor over one batch of the spin-up's first transient, and
2 cm after). Gate: mip 0 byte-identical; the twin at the anchor within 1 mm of mip 0 through the hour
(the mean of four cells against the one). On this scene the fine set is the whole wet box, so the twin
has no cells of its own yet; 1c gives the fine set's edge the coarse neighbour (the magnify) and 1d lets
mip 0 be demand's, which is where the centimetre gate means something.

**Stage 1c, built (2026-10-26): the magnify, through the chains.** A cell a mip holds no tile for is HELD
by the mip above where its tile there is resident (`Swe.hlsl` UpLiveAt, from the mip above's live words
on a root SRV), and its water, bed, face velocities and sea are the coarse cell's, magnified (nearest:
the coarse value stands over its 2 x 2; HIERARCHY 4.20's magnify law). The kernels read the mip above
through ONE ROOT TABLE OF THE CHAINS (`gHChain`, `gBedChain`, `gVelChain`, `gWaveChain`: every mip of
each, indexed by the mip; `SweSolver::m_tableChain`, one per parity pair), since the kernel's 16-slot
table is a mip's own views. So the fine set's edge borders the coarse water, not a frontier: the frontier
(the stateless sea) stands only where NO mip holds the cell. The flux and velocity banks cover one cell
past each state tile (the apron), because a face on the fine set's west or north edge is owned by the
cell beyond it. The twin sweeps the sea at mip 1 from its own open sides; the fine set's sweeps take
the coarse sea as their up-ray source (the fine set has no open side of its own) and its push continues
across the edge. THE ORDER IS BERGER AND OLIGER'S: each coarse step is the fold of the fine's state,
one step of the twin, then the fine's two steps reading the coarse neighbour the twin just advanced
(`RecordCoarseBegin` / `RecordCoarseStep`, interleaved in `Record`'s loop). The first build stepped the
fine's whole batch against a coarse state frozen for sixteen steps and then the twin against a fold
frozen for eight; the probe line (the anchor, a line out to sea and across the fine set's seaward edge,
every 30 s) showed a metre of ringing born at the last fine cells before the edge and growing to eight
metres in the open sea before friction took it: two Dirichlet couplings alternating a batch apart. With
the whole window fine under the same kernels the anchor matched stage 1b to the millimetre, so the
kernel changes were sound and the interface was the fault. Interleaved, the line is quiet.

**Stage 1d, built (2026-10-26): mip 0 by the eye's demand.** `water.swe.fineReachM` (0 = the whole
window, today's): mip 0 is wanted where the water is AND within the reach of the eye's ground point
(the spin-up's eye stands at the place's anchor; each frame's is the camera's, with `weather.Update`);
beyond it the water is the twin's own, its fold having carried it there each coarse step, so a released
tile's water stands at mip 1. The fine set is aligned to the coarse tiles (a 2 x 2 wanted whole, so
every coarse tile over it is a fold and none is half its own); the boundary's tiles are kept only within
the reach, beyond it the twin's edge rows are the box's boundary; a tile beyond the reach goes at once
(demand, not dryness). The residency log draws the fine set a row a line. GATE (`merrimack_bar --storm
3,10,95 --headless --frames 60 --dump-water-state --set water.swe.fineReachM=1500`, against stage 1b's
run): 55 fine tiles of 120 (a 4 x 4 of 128-cell tiles about the inlet, the outer sea, the Sound and the
river above Newburyport at mip 1); the anchor's level through the hour's spin-up within a millimetre of
stage 1b's at every probe; the dump at the frames' end identical at the anchor, within 2.3 cm over the
kilometre about it, 5.4 cm at most anywhere (beyond the reach, where the bank's readers still take the
tide plane: they do not read mip 1 yet, stage 2's). The twin's limit kernel now runs over its whole
lattice, the folded tiles included (measured a no-op: the dump byte-identical; the law is the honest
one).

Residuals, stated: the magnify is nearest (bilinear is a line); the fine's two steps read the coarse
state at the end of the coarse step, not interpolated in time (first order; Berger and Oliger
interpolate); the exchange across the edge is computed by both sides (the fine face against the
magnified cell, the coarse face against the fold) and not refluxed, so mass across the edge is
consistent to first order, not conserved to the bit (the fold of fluxes, 4.20's first law, is the
next piece); the bank, the globe and the hull read mip 0 or the tide plane, never mip 1.

**The sphere (Mark, 2026-10-26: "are you taking into account the spherical nature of things?").** Not
inside the kernel. The solver's lattice is the window's equiangular lat/lon chart with ONE metre pair
(dx, dy) for the whole box; the sphere enters at the bed (each cell's place is a direction from the
anchor through the window's chart) and at the readers (the bank and the hull read the solver in that
chart). Fine at 18 km; over the Gulf the east-west cell length drifts a few percent with latitude and
the law does not know it. What maps well: the tree's laws are tile-local (fold, magnify, the frontier,
demand) and only the kernel's metric is not; the cubed sphere (Ronchi, Iacono and Paolucci 1996; the
lattice of MITgcm and FV3) is six quadtrees of tiles with per-cell metric terms, which is the origin
lattice's and the cube-lattice blocks' shape already. The kernel stays one text: the metric becomes a
per-cell face length and span the tree supplies as it supplies the bed, and Coriolis a bivector
rotation of the face velocity. The first honest step is a per-row metric in the window; the tail of
stage 2 (the planet's water at the coarse mips) is where the chart has to become the cube-sphere tree.

**Stage 2, built (2026-10-26): the solver stands where the eye is.** No scene names a box or a solver now:
`water.swe.box`, the `solvers` list, `AddDormantWindow`, the activation altitude, the shelf rung's one-way
nesting (`SetLevelDeparture`, `OwnedLevelAt`, `RequestRegionOwned`) and the manager's owned windows are gone
(`SweSolver`'s domain, `WaterManager`, `FrameLoop`, the schema; 376 lines out, 110 in). The window is
`water.swe.windowTiles` tiles of 128 cells a side (16: 2048 cells, 28 km at the cell's 13.7 m) about the
eye's start -- the place's anchor, where every view stands -- with its south-west corner on the GLOBAL
LATTICE of the cell's angle, a tile's angle apart, so a place falls in the same cells whichever eye came to
it (`Assembly.cpp`). Its bed is the one height stack at every cell, its tide focus the nearest station in
it, its datum that station's link, its rivers whatever reaches of the world's river data cross its edge,
its open sea the tide atlas along every edge cell whose bed lies below mean sea, the twin at mip 1 the
whole window (the tail, pinned and stepping), mip 0 by the eye's demand within `fineReachM` (1d). The
window's size is a budget, the tail's extent at two mips; the chain's coarse mips carrying the planet is
what retires it, and the window following the eye in flight (re-stood about it: its bed, rivers, tide
focus, standing window and the bank's ladder anew) is stage 2b's, with those mips -- today an eye that
leaves the window finds the tide plane, as it did beyond the box, said once (`WaterManager::Update`).
THE MERRIMACK (`merrimack_bar --storm 3,10,95 --headless --frames 60 --dump-water-state --set
water.swe.fineReachM=1500`, against stage 1d's run): the window 42.683..42.936 N x -70.938..-70.686 E,
2048 x 2048 cells, the bed whole in 0.5 s (1221 tiles), 81 fine tiles; the anchor's level through the
hour's spin-up 10 to 14 cm UNDER stage 1d's at every probe (-1.108 against -1.0 at the hour's end, the
plane -1.007), and the dump, where both runs' grids lie inside the fine set, 10.4 cm lower (median; 11.2 cm
at most) -- not the kernels (1c/1d matched to the millimetre under the same kernels) but the boundary: the
window's edges 10 km further out, the Merrimack's face 4.6 km further up the river (km 13.3, its prism
5.36 km^2 against 3.40), the Ipswich entering on the south, the tide focus Plum Island instead of
Newburyport (2 cm of datum). Run to run the dump is identical to the bit. Two readings of the dump that
cost the evening: (1) 85 % of its grid read the DRY FLOOR (-0.65 under the plane) while the kernel held
water there, because the dump's grid stands about the origin lattice's node, 2 km east and 3 km north of the
eye, and the fine set is now the eye's 4 x 4 tiles -- beyond its reach the twin holds the water at mip 1 and
no reader reads mip 1 (the residual stated under 1d, now visible; `SweSolver::ReadFields` says what the
mirror holds with every read, and the dump exports the mirror and each cell's lattice cell); (2) the
world's river composite carries a reach with no name (nhdplus 5860733) that the window's west edge now
cuts, a nameless face with no discharge. BOSTON HARBOR (`scenes/demos/boston_harbor.json`: the anchor at
the station, the clocks pinned, a view; no box, no station, no solver): the solver stands with no key --
the window 42.225..42.478 N x -71.191..-70.938 E, Boston's station the focus and the datum (-1.678 m), no
river, Flather on the atlas's level at 1525 edge cells, the twin's 64 tiles, the pin's rectangles across
the page's seam (two, the harbour straddling u = 1 of the standing window's modulo-16384 addressing) --
BUT THE KERNEL'S BED DID NOT LAND: the whole-bed wait maps 3 to 42 of its 1221 tiles in 60 s over 6000
residency turns (the manager's claim m15 over the domain: nothing of slice 46 resident), while the same
boot at the Merrimack, whose height tree is warm on disk, maps 1221 in half a second; two root loads (the
colour's and the survey mask's, face 2) sit "in flight, never returned" from frame 1 under
`streaming.offline`. The kernel read the cube's coarse heights instead (+33 m at Long Wharf against the
stack's +0.5), nothing was wet, and the harbour did not flood. Moving the anchor 10 km east or north
(scenes/debug/boston_seam*.json) changed the seam's cut (four rectangles) and the claim's progress (mip 0
at 89 of 66049 samples, mip 3 at 5506) but not the outcome: the block is the order's at a cold place
offline, not the seam's alone and not a water law's. The height tree's identity is the place's by design
(the datum's offset is in it: `|norm x... -0.092`), so every new place paints cold. Stage 2's gate stood half met
there for a night.

**Stage 2, closed (2026-10-27): a cold place's bed is the cube's own level, and the solver wants what its
kernel reads.** The block was not the seam, not the two root loads, not the order's need law, and not Google:
it was the solver's want. A rank-2 standing window's pages hold mips 0..3 (10 m to 76 m of ground), and the
height stack holds Boston only at its global tiers (ETOPO's 15", the 463 m tier: nothing finer is harvested
there), so by the tree's own law (F14, `TileTree::Magnified`) EVERY page of the window is its parent magnified
-- "known without a load", never mapped, and 1835 of them were. The kernel's read rule
(`shaders/HeightPages.hlsli` `HpChainRead`) was already right: the standing window where it holds the cell's
ground, the cube face by direction where it does not. But the solver's pin and its whole-bed wait asked for the
window's slice alone, never the cube, and the wait ended only when the window's mip 0 was claimed over the
domain -- which a place with no survey can never do; it ran to its 60 s bound while the kernel read the cube's
root (m7, 78 km texels: +33 m at Long Wharf). Now `WaterManager::WantDomain` states the solver's want as the
rule reads: the domain's rectangles on the standing window at mip 0 AND on the cube's faces under it at mip 0
(`SurfaceFrame::CubeRectsLL`), the magnified law closing every tile below the own level without a load, so the
ask lands at the own level wherever that is -- the window's pages at the Merrimack, the cube's m0 at Boston --
and the wait ends when the order has answered every tile this reader's wants imply
(`ResidencyManager::PendingOf(sampler, tenant)`: not held and not closed as magnified or refused; the colour's
cold root, another reader's, no longer holds it). The bed wait's line now says what the kernel read, by slice
kind and mip. BOSTON: the bed whole in 0.11 s (10 tiles, 16 turns; the cube's m0 over all 4194304 cells),
17.5 % of the window below the tide, 120.87 km^2 of water joined to the sea through 1525 Flather face cells on
the east edge, 0 km^2 unjoined at the edge, the harbour's level following the falling plane to within 5 to 9 cm
through the hour (sea+1km -1.264 against the plane -1.320 at its end; the lag the Merrimack's sea shows too).
The anchor's own texel, Long Wharf, is LAND at 611 m: the inner harbour does not exist at that grain, and that
is what the tree holds, not a law -- a 1/9" tile harvested there would paint the window's pages and the same
want would land them. THE MERRIMACK is byte-identical before and after (the window's pages win over the cube
wherever they are resident; its bed whole in 0.48 s, 1229 tiles: the window's and the cube's column). The
scene (`scenes/demos/boston_harbor.json`) holds the eye's start, its clocks and two views and names nothing
(no `scene.name` either, Mark: "the only thing the scene should have about Boston is the camera's projection
and coordinate location"). One instrument mended on the way: the flood test (`SweSolver::LogHoldsWater`)
seeded only from the sponge of a side open along its WHOLE length and from river faces; Boston's east side is
open on 1525 of 2048 cells, so it has Flather faces and no open side, and the instrument said "joined to
none" of water the kernel had joined. Its seeds are now where the sea continues past the edge: the sponge's
columns of a whole-open side, the Flather face cells elsewhere (`SweDomain::Bound`). Still open, not the
water's: the colour's and the survey mask's face-2 root paints never return under `streaming.offline` (two Io
threads of twelve for the run); a side open on most but not all of its cells gets faces and no sponge (the
open-side law is whole-side -- made one law the same day, below). Stage 2's gate is met: the solver by demand
with no key, the cold place flooding on what the tree holds, the Merrimack unchanged.

**The edge is one law (2026-10-27, Mark: "make the open-side law one law for a side open on part of its
cells").** Until this date the window's boundary had three laws: a side was *open* when every cell of it lay
under +1.2 m NAVD88 and no river entered there (its every cell a Flather face on the Level, and an offshore
sponge standing on it); on any other side only a cell whose bed lay below mean sea was the sea's; the rest were
walls. Boston's harbour mouth, open on 1525 of 2048 cells of the east edge, had no open side. Now every edge
cell with a bed that no river face holds is the stateless sea's exterior, pinned to the Level along its side
(`Swe.hlsl OceanSideOf`, `SweDomain::SeaEdgeCell`): it holds the water that Level stands over its bed and none
where the Level is under it (`CsSweHeight`), and its faces to the interior are Flather. That is the law
residency's frontier already had (stage 0b: the tide plane over the megatexture's bed), standing at the
lattice's edge too: a channel the edge cuts fills and drains with the sea and a dune at the edge stands over
it, each instant, with no side open and none a wall. Gone with it: `openSides`, the sponge and
`water.swe.sponge`, the whole-side test, the kernel's bitmask (its cbuffer slot kept as `gUnused0` so `gFaceA`
stays on its register), the boundary's keep-forever tiles (`m_keep`; the tiles the boundary's water stands on
are now computed each residency turn from the Level, `SweSolver::BoundaryTiles`, the same rule as any water),
and the flood instrument's "crossings" (a wet edge cell is a seed, so there can be none). The spill flood seeds
from every sea edge cell at its own bed; the sea source's point is the edge water's depth-weighted centroid;
`OpenSeaLevel` is the mean Level over the edge cells it stands over; the cycle tool's ocean probe stands by the
deepest edge cell. Gate (stage 2's two runs, before and after): Boston unchanged to 1 mm (its three land sides
are dry exterior now as they were walls before); the Merrimack within 2 mm of level and 14 cm/s of current over
the dump's area (mean 0.06 mm, 0.6 mm/s), the north and south edges' cells under the Level now the sea's (892
and 568 of them under mean sea); the flood instrument joins the same 120.87 and 303.28 km^2 with nothing
unjoined at the edge; its self-test rewritten to the law (the sea joins a cut channel from both ends, a pool
the edge does not touch is reported, a reach a river face alone holds is said so, the crest a raised bed parts
a reach at is found). Stated: where ground at +1 m lies at a land edge and the atlas's Level tops it, the sea
enters there, as it does at residency's frontier; a dike is the bed's to say, not a side's.

**Stage 3, built where the field stands (2026-10-27): the impact imposed on the columns.** An event of the
record (`sim/ImpactField.h`) is IMPOSED on the column field at the first batch the field stands past its zero,
at every mip the field holds -- mip 0's fine set and the twin, each on its own list and constants
(`Swe.hlsl CsSweImpose`; `SweSolver::BeginImpose`, `RecordImpose`; the record reaches the solver through
`SetImpacts`). What is imposed is the law's state at that instant: the level, the sum over the event's table at
the cell's distance (`laws/Impact.hlsli ImpactTerm`: at T = 0 the cavity itself, the lipped parabola of net
volume nought; later the train linear theory has carried), and its radial velocity from the same sum by
continuity, u_r = sum A_n omega_n / (h0 k_n) J1(k_n r) sin(omega_n T), nought at T = 0 (the cavity is at rest) --
each shoaled from the basin depth to the column's own water by Green's law (eta by (h0/h)^(1/4), u by (h0/h)^(3/4),
the WKB limit of the same theory, unbounded: the columns are the nonlinear reader). The level goes into the column
(h >= 0: a cavity deeper than the water stands on the bed), the velocity onto the cell's east and south faces
radial from the impact, and the law carries it from there: the ring, the shoaling, the steepening, the run-up,
with no analytic reader inside the window. The distance is in the anchor's chart with the impact placed its
great-circle distance along its bearing (the chart's error against the sphere is second order in the window's
size over that distance: 2 m at 2600 km). The table rides a root SRV (t5); the event's place, instant and term
count are two float4 appended to the constants on both sides. One event a batch. The gate's probe
(`ProbeCoarse`) says the law's own level beside the solver's every batch for the hour after an imposition.

*The gate* (`scenes/demos/impact_bar.json`, out/review/stage3/impact_bar1): a 30 m stone at 20 km/s into the
sea 7.2 km east of the Merrimack's mouth, ten minutes into the spin-up hour on the bar scene's pinned clocks;
its cavity 1.3 km across and 25 m deep (the scene's basin depth; the bed under it is 41 m), 244 terms, imposed at
T = 13 s. At the 3 km sea point (4.2 km from the impact, the twin's own cells, 26 m of water) the solver's first
crest arrives at 168 s (+2.04 m) against the law's 197 s (+2.83 m) and the troughs coincide at 225 s (-1.86
against -2.86); at the 1 km point (6.0 km, the fine set) the first motion is 284 s against 313 s and the crest
299 s (+1.61) against 313 s (+3.22); between the two sea points the solver's crest travels 1.8 km in 131 s, 13.7
m/s = sqrt(g h) for the 19 m of water between them, while the law's travels it in 116 s, 15.5 m/s = sqrt(g h) of
its one depth. So the solver rides sqrt(g h) of the bed it has, and the 15 % by which it leads the law at the
first point is the 41 m under the impact against the law's 25: the law has one depth, the solver the bed. The
gate's first clause is met on sqrt(g h); against the law it is met at the 1 km point (9 %) and not at the 3 km
point (15 %) for the reason stated. At the anchor, in the inlet between the jetties, the solver shows +0.90 m at
419 s where the law says +2.6 to +2.8 (Green's law 1-D and unbounded into 5 m of water, which the jetties and
the bar make false): no gate there. The law's second crest at the 1 km point (410 s) the solver does not carry:
shallow water does not disperse a 1.3 km cavity's train, as 14.2 said. Amplitude: the solver's 0.65-0.72 of the
law's at 4 km, 0.42-0.51 at 6 km -- measured, not settled: the imposition is 0.88 of the law's cavity to begin
with (Green's law from 25 to 41 m), the twin's 27 m cells diffuse, and the linear law keeps what dispersion
spreads. The Merrimack without an event is byte-identical through the change (merrimack7 = merrimack8).
The exterior still reads the Bessel law at every edge cell (the one edge law), so the solver's own train
meets the law's at the edge: the east edge's Level stood +2.16 m at T = 256 s.

*Then not built:* the tail -- built the same day, below.

**Stage 2b, built (2026-10-27, Mark: "build the tail"): the train crosses the ocean in the tail's own solve.**
The tail is a third lattice of the same solver: the window's chart coarsened by 2^rank about the same anchor,
`water.swe.tailCells` a side (2048 of 2^9: 5.2 x 7.0 km at the Merrimack, 10,560 x 14,395 km across on the
tangent plane, the far corners' cells 0.44 of their size on the sphere), the window whole in its central cells
(1022..1026 of 2048: its corner lies on the tail's lattice by construction), its CPU bed the one stack at the
tail's grain (4 M cells in under 5 s) and its GPU bed the cube's mip 3 (wanted with the window's, `WantDomain`),
its residency the tiles the sea reaches from its edge at mean sea (63 % of the cells, 213 tiles, for the run),
its own constants, lists and step (1.77 s: the CFL at the deepest water, 8384 m, over the far corners' true cell,
and as much again for the water's own speed under a falling lip), stepped ahead of each batch on its own clock
(`RecordTail`). Its gauge is MEAN SEA: its columns rest at 0 m NAVD over their bed, its tide plane is 0, and what it
carries is the DEPARTURE -- the train, a surge -- not the tide (an ocean tide solve at 5 km with no Coriolis would
have fed the Merrimack a wrong tide; the tide is the atlas's, as before). Its own far edge reads the impacts' law
alone (`SetTailLevelSource`). THE WINDOW'S EDGE KEEPS ITS ONE LAW, and the Level there is the atlas's PLUS the
tail's departure and its velocity at the tail cell just beyond the side (`Swe.hlsl TailDepAt`, `TailUAt`: Berger
and Oliger's ghost, never the cell the window lies in); the analytic law is no longer read at the window's edge
where a tail stands. THE CHART'S METRIC (`CellM`): every kernel takes a cell's true size from its place in the
tangent plane, the gnomonic chart's diagonal metric (R ds sqrt(1 + t^2) / q along s, R dt sqrt(1 + s^2) / q
along t, q = 1 + s^2 + t^2; the shear s t / q left out, 0.2 at the tail's far corners) -- the window's cells
change by 5e-6, and the Merrimack with the tail standing is within 0.16 mm of level and 2 mm/s of current of
the run before. An impact is imposed on the levels that hold its cavity (the tail: yes; the window: not for a
far event), and AT ITS INSTANT AS THE CAVITY ITSELF, closed (`laws/Impact.hlsli ImpactCavityAt`, while the front
has moved less than a tenth of the lip): the table's sum, imposed at T = 6 s, had planted +-2 m at the window
2600 km out and +-12 m before the front at the window's edge -- the truncation's leakage, a wave from nowhere.
The fold of the twin into the tail is written (`CsSweFoldTail`) and OFF: the twin's departure from the window's
tide plane carries the estuary's own lag against that plane, which folded into a 5 km ocean cell radiated back
to the window's edge as a sea-level change before any event; the tail is one-way (the window's water does not
return to the ocean; its feedback on a 5 km cell is nil). The spin-up stops itself, and says so, when the step
collapses under 5 ms (a blow-up), instead of crawling for hours.

*The gate* (`scenes/demos/impact_tail.json`, out/review/stage3/impact_tail2; impact_tail1_ray is the same run
before the fastest-path instrument): the 10 km stone of 13.4 into 35 N 42 W at 15:00Z, 2625 km from the
Merrimack's mouth, the spin-up from 14:30Z for 8 h (139 s of wall clock). The cavity (109.9 km across, 4000 m
deep, 21 cells) imposed on the tail at T = 6 s; the tail SILENT at the window for five hours (0.000 m at the
anchor's cell and east of the window); then the train. Its first 0.1 m of motion reaches the tail cell east of
the window at 5.15 h and the anchor's cell at 5.31 h, the half-crest about 5.5 h, the crest at 5.70 h (east,
+9.4 m) and 5.85 h (the anchor's cell, +11.9 m), the trough -4.5 and -7.1 m at 6.7 h. The reference, two
instruments on the tail's own bed at sqrt(g h): the great-circle ray 5.84 h (16 m of water at its shallowest,
Nantucket's shoals) and the fastest path 5.90 h (Dijkstra over the sea cells at the chart's metric: the ray is
already the quickest way on this bed); the one-depth law said 3.68 h, 2.2 h early, the shelf's share. So the
crest rides sqrt(g h) over the bed to within 3 %, and the first motion runs 12 % ahead of it: the lip's step
front, smeared by the first-order scheme over about 150 km after 500 cells of travel. The gate's first clause is
met on the crest; its second (the amplitude at 2600 km within the law's dispersion) is not judged here: the law's
far field is its one-depth train and the tail's is the bed's, and the 10 km stone's train at the Merrimack --
+10.7 m at the twin's sea point, +10.0 m at the anchor, then a 6 m trough -- collapses the window's step at 7.06 h
(the guard stopped the spin-up with 0.43 h left): the window does not survive this stone, which is the window's
matter and not the tail's.

**The tail drawn (2026-10-27, Mark: "draw the tail").** The far field's readers -- the bank kernel's texel
(`WaterBank.hlsl`) and the globe's vertex (`Globe.hlsl ImpactAt`, the pixel through `imp`) -- read the tail's
column where the tail holds the point, in place of the impacts' analytic sum, which stays only for points the
tail does not hold (`shaders/TailRead.hlsli`; `sim/ImpactGpu.h TailGpu`, `SweSolver::TailGpuDesc`, the layers'
`SetTail` from the frame loop each frame beside `SetImpact`). The cell under a direction is the gnomonic chart's
(s = d.E / d.A, t = d.N / d.A over the tail's anchor frame); the departure is the column less mean sea's over the
bed, h + b, read bindlessly from the tail's two textures (a tile the tail does not hold reads 0: no sea,
nothing); a land cell takes its nearest sea neighbour's (the coast within a tail cell); the slope is the
neighbours' departures over the chart's true metric along the chart's east and north projected onto the tangent
plane; the display exaggeration applies to the drawn departure as it did to the drawn law; the departure and
its slope are shoaled from the water the tail stands in to the point's by the same bounded law. The tail's h and
bed stand in the readers' state between batches (`TailTo`), as eta and uv do; the probe reads them in whatever
state they stand. *The gate* (`scenes/demos/impact_tail_orbit.json`, out/review/stage3/stills): the 10 km stone
an hour after it, the eye 3000 km over the impact, the drawn departure x10 -- with the tail, ONE BROAD FRONT 700
km out with a few rings inside it, a little asymmetric where the bed is; with the tail off (no solver window:
`--set water.swe.windowTiles=0`), the law's dense concentric train over one depth, the Bessel sum's dispersion
and its ringing. The two stills differ only about the ring (mean 1.0 level of 255 inside 300 px of the impact,
0.15 beyond): the field the eye sees is the tail's where the tail stands. At 4.5 h from 3000 km over the shelf the
train is ten metres and shows in neither still even at x10 -- the globe's vertex carries it, the eye cannot see
it from there; a shore eye sees the window's own water, the solver's. Stated: the hull's reader (WaterManager's
`impacts` producer, the CPU sum) still reads the law beyond the window -- a boat far at sea on the tail's train
is the next reader to turn; the far field's readers take the tail with no residency of their own (the tail's
tiles are the run's).

**Stage 2c, built (2026-10-27, Mark: "the telescope"): the levels.** The tail is now the coarsest of a LIST of
levels (`SweSolver::Level`, `m_lv`, coarsest first), each a rectangle of the one anchor chart at its own cell size
(2^rank window cells) with its corner on its parent's lattice so its cells nest exactly, stepped by the same
kernels on its own banks, lists and clock, coarsest first each batch (`RecordLevel`). A level's edge is the
sea's exterior at its PARENT's departure and velocity in the ghost cell beyond the side (the kernel's `gTail`
is now "the parent's fields", `ParentFields`); the top's edge reads the impacts' law. A level of mean sea's
gauge folds its wet cells' mean into its parent before each parent step (`CsSweFoldTail`, the same kernel; the
window's twin, of the tide's gauge, still does not). THE ANCHOR LEVELS telescope from the tail to the window:
`water.swe.telescopeLevels` of `telescopeCells`, `telescopeStep` ranks apart under the tail's (3 of 512, 2
apart: ranks 7, 5, 3 -- 1.3 x 1.8 km, 320 x 440 m and 80 x 110 m cells, 900, 225 and 56 km across), the window's
edge reading the finest that holds it with a ghost beyond (level 2^3). AN EVENT whose cavity is unresolved
where it falls -- its lip over fewer than eight cells of the finest level holding it -- gets a PATCH of its own
the batch it is imposed (`AddLevel` at run time: the stack's bed at its grain, its banks, its tables with the
page views kept from `SetBed`): 512 cells at the rank its lip wants (R_D over eight window cells, a power of
two), about its cavity, nested in and folding into the finest level that holds it. The chart's inverse carries
a rectangle's offset (`SweDomain::ox, oy`) so every level's `CellOf` and `LatLonOf` stand. The readers take the
finest level holding the point (`TailRead.hlsli` over `gLvlA[8]`/`gLvlU[8]`, coarsest first in the cbuffers, the
search finest first); the manager wants each level's bed at the cube's mip its cells read (`LevelBoxes`).

*The gate* (out/review/stage2/merrimack10; `scenes/demos/impact_telescope.json`, out/review/stage3/impact_telescope1):
the Merrimack with four levels standing is BYTE-IDENTICAL to the run with the tail alone (the window's edge reads
level 2^3's ghost, at rest as the tail was). The 1 km stone (1950 DA's class) at 35 N 42 W: its lip, 12.9 km,
stood over 2.5 cells of the tail, so the event got `patch 2^7 (event 0)`, 512 cells of 1.3 x 1.8 km (660 x 900
km, all sea, 16 tiles), parent the tail; its cavity (18.2 km across, 4000 m deep, 1777 terms) imposed there at
T = 1 s -- the probe's cell 8 km off the centre read +2218 m, the lip, then -2 m a minute later, and the centre
sloshed +-3 m for hours. Then the honest result: THE TRAIN DOES NOT SURVIVE THE CROSSING. At the window's east
cell the tail's departure rose to 0.09 m (0.13 m at the anchor's cell) from 5.4 h, where the one-depth law says
+-20 m (Green's law unbounded to 68 m: +-9 m deep) and Ward & Asphaug 3 to 9 m deep; the window answered
+-0.5 m at the bar. The 10 km stone's 50 to 200 km waves crossed the tail at two thirds of the law's amplitude;
this stone's 13 to 50 km waves are three to ten of the tail's 5 km cells, and the first-order upwind flux's
diffusion over 460 cells of travel takes them (a 13 km wave to a few percent, a 30 km one to half, by the
scheme's own diffusion u dx / 2 over the crossing's 3.5 h) -- and the law's dispersive train, which shallow
water does not make, spreads what remains. So the telescope resolves the cavity WHERE IT FALLS and not ALONG
THE WAY: the patch is a level about the impact, not a level that follows the front at the mip its wavelength
needs (14.2's words), which is what this stone's train would need -- or a tail less diffusive than first
order, or a finer tail (2.5 km at 4096 cells: 16 M cells). Stated, not built, and the next piece.

**Stage 2d, built (2026-10-27, Mark: "follow the front"): the level that follows the train.** An event's
patch is now a SPARSE level: its lattice is its parent's whole extent at the rank its lip needs (the tail's
8192 x 8192 at 2^7, 1.3 x 1.8 km cells; a rank whose extent would pass 16384 cells is refused), its bed the
stack's on the GPU alone (no CPU bed, no flood), resident about the cavity to begin with (four lip radii each
way) and from there BY THE WINDOW'S OWN RESIDENCY LAW ON THE DEPARTURE: the tile statistics (`CsSweTileStats`,
counting a column live where |h + b| passes the level's tolerance, 5 cm, over sea alone) read back outside the
recording (after the batch in the spin-up, through the asynchronous ring on the frame), a tile across an edge
the departure reaches mapped ahead of the front, a tile quiet past the hysteresis released; a tile mapped since
the last batch is cleared, bedded and INITIALIZED FROM ITS PARENT (`CsSweMagnifyInit`: the parent cell's
departure and velocities over its own bed -- Berger and Oliger's interpolation; a tile rested instead erased,
through the fold, the train its parent carried under it), before any level folds or steps (every level's
prelude first). A level's non-resident cells are HELD BY ITS PARENT in the magnify's sense (`gUp`: the shift
and offset to the parent's cells; `UpLiveAt`, the chain reads at index 1 are the parent's), so a front leaving
the resident set meets the parent's water, not a frontier. The fold is one thread a parent cell under the
child's resident tiles (`CsSweFoldLevel`). The levels step in Berger and Oliger's order: the top steps, and
after each of its steps its children step up to its time, recursively (`RecordLevel` recurses); the fine twin
has run so since stage 1c, and running every level a whole batch apart had left the ring ringing.

*The gate* (out/review/stage3/impact_front4; impact_front1..3 the runs before the interleaving, the magnify
init and the land fix): the 1 km stone again. Its level grows from 2 tiles to 1427 by 7 h (the ring never
calms: every resident tile keeps a column over 5 cm), the spin-up of 8 h takes 16 minutes, and at the window's
east cell the departure reaches +0.26 m at 6.48 h and -0.34 m at 6.75 h (the window answers +0.5 / -0.9 m at
the anchor), against 0.09 m with the static patch and the one-depth law's +-27 m there (Green's law to 68 m;
+-9.6 m deep). The interleaving and the parent initialization changed the numbers by nothing to three
decimals. Two things stand, stated: a train crossing a level's residency edge is water the fine level loses
(its flux into a non-resident cell goes to no column; the parent receives the fine's state only by the fold),
so the ring must stay ahead of the front -- it does, at this stone's pace; and THE CAVITY IS A HOLE TO THE BED,
4000 m deep and 9 km across in 4000 m of water, whose collapse in the nonlinear solver is a dam break that
spends its energy in the rim's bores and the centre's sloshing (+-2 m at the impact for hours) -- which is what
the hydrocode studies found against Ward and Asphaug's linear estimates (Gisler and others, 2011: deep cavities
raise far smaller tsunamis than linear theory says). The 10 km stone's cavity, 55 km across for the same depth,
is gentle, and its train crossed the tail at two thirds of the law. So the 1 km stone's 0.3 m at 2600 km may be
nearer a nonlinear truth than the law's 9 m; the solver's own dissipation at the collapse (first order, on 1.3
km cells) is not separated from the physics here. The separation is a convergence run: the same event at a
finer patch (the lip over sixteen cells) and the departure at a fixed distance from the impact at both ranks.

**The convergence run (09db298, the same night).** `scenes/demos/impact_convergence.json`: the 1 km stone, two
hours of its time, the probe 300 km from the impact toward the anchor; `water.swe.patchCellsPerLip` 8 (the patch
at rank 7, 1.3 km cells) against 16 (rank 6, 0.64 km cells). At 300 km the two ranks agree on the front: the
first motion in the same probe (one minute's resolution), the leading crest +33.2 against +31.6 m, the trough
-41.1 against -45.5 m, at the same minutes; the second crest does not (+38 against +27 m), the train behind the
front being the scheme's. So the collapse is the physics (a converged 35 m front at 300 km from a cavity 4 km
deep, where the law's 1/sqrt(r) from its 27 m at 2600 km would put 80), and the crossing is the scheme's:
from 300 to 2600 km geometric spreading alone leaves 12 m and the rank-7 run arrived at 0.3. (I read that as the
first-order flux's diffusion and named a second-order flux as next; the next day's clean line, below, shows the
deep crossing holding 1/sqrt(r) and the loss standing between the shelf and the window's edge -- the second-order
flux is not the piece.) Two more things the night fixed:
the solver steps whenever levels exist (the sea layer skipped it above 60 km, and the first orbit clip drew the
levels at rest), and a level's constants and lists are pushed once a batch in its prelude and bound by address
in the steps (the Berger-Oliger recursion pushed them at every call: a few hundred a frame at 240x exhausted the
frame's 4 MB constant arena). The clips: `out/review/stage3/clips/orbit_levels.mp4` (240x from orbit, 104 ms a
frame with the window stepping too; stepping the levels alone when the sea is off is a follow-up) and
`shore_levels.mp4` (the dune at 30x as the train arrives through the tail).

**The columns over the marsh (2026-10-28).** The shore clip drew the marsh and the town as land-textured columns
kilometres tall from nine minutes of sea time on. Not the levels (undrawn: the same; none: the same), not the stone
(none: the same), not the levels' fields (every level's greatest departure stayed sane; the cavity's 5 km is the only
large number and sits at the impact), not the readers' order (an event's patch, appended last, was read as "finest"
at the shore -- fixed, the list is ordered by rank, 8c119ff), not the column plane, the wave table or the height
pages. The writer is THE WINDOW: with `water.swe.fineReachM` (stage 1d) bounding mip 0 to 1500 m about the eye, the
fine set moves from the anchor (the spin-up's eye) to the camera at the first frame, and at its edge across open
water -- the river mouth off the dune, tile (7, 8) of the 4 x 4 block -- the window blows up in 6-15 minutes of sea
time (columns 1e15..1e28 m, currents 92 km/s); the bank reads that surface (`SolverSurfaceDev`), its level plane
goes to 1e20, every vertex under that waterline is water and rises with it, and the pixels still draw land. The reach
at 300 m maps nothing (clean), at 6000 m changes nothing (clean), at 1500 m maps eight tiles under the twin's water
and stands its edge in the channel (columns); at 1x a minute of sea time is too short to see it. Initializing a fresh
mip-0 tile from the twin (the levels' CsSweMagnifyInit at mip 0) took the dam break away (1.8 m/s after the mapping
turn against 20.7) and the blow-up came SOONER: the instability is the fine/coarse interface itself -- the fine edge
reads the twin's magnified cells while the twin steps its own beyond, with no reflux -- not the fresh clear; reverted.
Every gate recipe carried `--set water.swe.fineReachM=1500` (a speed knob) with the eye at the anchor, so no gate
stood where the frames did. Clips and gates run at the default reach (0, the whole window) from here: the dune at
60x for thirty minutes of sea time never crosses 60 m or 25 m/s there; the frame costs 117 ms against 85. The tools
that found it: scenes/demos/impact_shore_quick*.json and shore_quick_*.json (the stone 115 km east, a quarter
hour's spin-up, five minutes a render), `GAGAME_LEVEL_STATS=1` (every level's greatest departure and where; the
window's deepest and fastest past 60 m / 25 m/s), `--bank-trace 150` (the bank's planes per ring), and a detector
scoring the sky in a frame's top third. One more reading from the bar scene's gates at the default reach: the
window's "fastest" current at its south edge tile climbs 26 -> 56 m/s over a minute with the tail standing and is
quiet without it, while the twin grid 4 km about the anchor is byte-identical either way and the clip is clean. My
reading: the derive's q / h in a thin edge cell the Flather inflow feeds on the ebb (the derive floors nothing above
the wet tolerance); but the fastest TILE holds 9.3 m at its deepest and the fastest cell's own depth is not printed,
so it is not shown -- OPEN: the window's edge with the parent level's ghost, at mip 0, on the frame ring.

**The causal impose, the damping as a time, and the beach (2026-10-28).** Three findings from one day, in the order
they fell. (1) `m_cb.damp = 0.99995f` a substep was a hidden linear damping: an e-folding of 1.7 h on the window
(0.3 s steps), 2.4 h on the event's patch, 17 h on the tail -- the 1 km stone's train spends 5.6 h from 300 km to the
beach, e^-2.3 from the damping alone. It is a stated time now, `water.swe.dampH` (hours; 0 = none, the default;
`SweSolver::DampOf(dt)` wherever a step is set). (2) With the damping gone the window's step moved (its fastest
current moved), the batch boundary moved, and the impose fell at T = 10 s instead of 4 -- past `CsSweImpose`'s
threshold (the closed cavity while T c < 0.1 R_D, 6.5 s for this stone in 4 km of water; the truncated Bessel sum
after) -- and the sum's truncation PLANTED metres over the whole ocean: every patch tile departed at the first
statistics, fresh tiles 166 km out held 1.5-12 m within seconds, the ring mapped the ocean (+6, +10, +14 tiles a
readback; 2600 tiles), and the line's probes read the planted field as the train (a 49.8 m "crest" at 600 km). The
symptom to know it by: the departure's residency growing quadratically right after the impose, and thousands of
metres on tiles far from the cavity. The law is one and continuous now: the cavity itself while T c < R_D (a
minute), the sum beyond with a causal cutoff, `w = 1 - smoothstep(R_D + c T, + R_D, r)` on eta and u_r. (3) THE
CLEAN LINE (`scenes/demos/impact_line.json`, probes at 300, 600, 1200, 1800, 2400 km on the chart, `kProbeKm`; the
probes every ~69 s of a ~4 min period, so a crest reads 0.6-1.0 of itself): undamped 39.0, 38.1, 17.1, 16.2, 9.1 m;
with the old damping stated (`dampH` 2.44) 35.4, 37.0, 16.3, 10.9, 4.5 m. The deep crossing holds 1/sqrt(r) within
the sampling (38 at 600 km -> 19 at 2400 by spreading; 9 read, on the shelf, where the probe's cadence catches less
of a shorter wave). The train reaches the window's east edge at 0.1-0.7 m (level 2^3's ghost cell, 49 m of water)
and the beach at +-0.5-0.9 m: the loss is between the patch's 9 m at 225 km (the Gulf of Maine) and the window,
in the telescope's intake from the tail (level 2^7's edge, 330 km out, reads the tail's ghost) or its shelf
crossing (2^7 -> 2^5 -> 2^3). OPEN; the instrument is in: every level now prints its departure along the stone's
line at 2, 5, 10 ... 1280 km from the anchor (` toward the stone:`), and the window's parent at the window's east
edge and 2, 5, 10 km beyond. THE NEAR STONE (`scenes/demos/impact_shore_near.json`: the 1 km stone 115 km east on
the shelf, 150 m of water, at 15:00Z; the cavity 18 km across to the bed) shows the telescope carrying a train it is
imposed on: +16 m at 80 km, 8-9 m at 20 km, 6.7 m two kilometres off the window's east edge at 15:39Z, +8.0 m at the
jetty at 15:48:40Z over a +0.9 tide, -7 m at 15:59Z, +2 m at 16:05Z; the window's land tiles 197 -> 205 (the
overwash); the levels agree with each other at every probe. Two traps in reading it: the coarse probes run in the
spin-up alone (`AfterBatch` on the frame path does not probe), so a scene whose spin-up ends before the arrival
reads the tide and nothing after -- the near scene's probes stopped at 15:40Z with the train 2 km out, and I
called the shore a wall on that (it is not); the arrival is read with `--set time.start=...17:00Z --set
water.swe.spinupH=2.0 --headless --frames 1`. And the window's "fastest" statistic is the derive's q / h in the
overwash's thin films (58-257 m/s in the hour the train is ashore; the step falls to 0.046 s and the 256-step
budget still covers a 2 s frame many times): not a blow-up, not the step starving. The first clip's rail
(`impact_shore.json`) stood in the marsh behind the dune, where the sea is a strip at the horizon and the arrival
reads as the marsh flooding at 0:12-0:18 -- the wave Mark could not find at 0:58 had passed forty minutes before;
`scenes/rails/impact_shore_face.json` stands over the beach face looking the bore in the eye.

*Stated, not built:* the telescope's intake of a train from the tail (the line's x13 between 225 km and the
window's edge: where, by the per-level probes); the window's fine/coarse interface in the levels' manner (the fine
fluxes refluxed onto the twin, the ghost at the twin's time) or the reach retired; a floor on the derived current
(and the level's step law with the current term, its factor 2 unexplained); the coarse probes on the frame path; the
hull's reader of the far field; the anchor
levels' residency is the run's; the metric's shear; the twin's fold into its parent, kept off (the gauge); a
sparse level is never retired after its event; the cost: the 1 km stone's level at 1400 tiles took the spin-up
from 3 to 16 minutes. A second-order flux is no longer on this list: the clean line crosses the deep ocean at
1/sqrt(r).

## Sources

- NOAA PMEL, [Tsunami propagation database (SIFT/MOST unit sources)](https://nctr.pmel.noaa.gov/propagation-database.html);
  Gica et al., [Tsunami hazard assessment using short-term inundation forecasting](https://nctr.pmel.noaa.gov/Pdf/Gica4-EASTEC2007.pdf).
- [A response function approach for rapid far-field tsunami forecasting](https://ar5iv.labs.arxiv.org/html/1812.01627) (pulse
  response functions, convolution of detector records).
- Ward & Asphaug, impact tsunami (Icarus 2000) via [SCEC summary](https://central.scec.org/node/3386) and
  [Korycansky, Asphaug & Ward (NASA NTRS)](https://ntrs.nasa.gov/citations/20030110765).
- [MOM6 harmonic analysis](https://mom6.readthedocs.io/en/dev-gfdl/api/generated/modules/harmonic_analysis.html);
  [FVCOM output harmonic analysis with UTide (pyfvcom2)](https://pyfvcom2.readthedocs.io/en/latest/cookbook/tidal_harmonic_analysis_fvcom.html).
- [TRITON-G, Arce Acuña & Aoki, NHESS 18, 2018](https://nhess.copernicus.org/articles/18/2561/2018/);
  [Celeris, Tavakkol & Lynett](https://arxiv.org/abs/1611.05984v1).
- Battjes (1974), surf similarity; Iribarren & Nogales (1949).
- §12, the water as columns:
  - Kass & Miller, [Rapid, stable fluid dynamics for computer graphics](https://history.siggraph.org/?p=122046),
    SIGGRAPH 1990: columns of a height field moving over dry terrain.
  - Irving, Guendelman, Losasso & Fedkiw, [Efficient simulation of large bodies of water by coupling two and
    three dimensional techniques](https://naml.us/paper/irving2006_rle.pdf), SIGGRAPH 2006 (tall cells);
    Chentanez & Müller, [Real-time Eulerian water simulation using a restricted tall cell grid](https://history.siggraph.org/?p=114669),
    SIGGRAPH 2011; Narita et al., [Quadtree tall cells](https://graphics.c.u-tokyo.ac.jp/hp/en/archives/3252).
  - LeVeque, George & Berger, [Tsunami modelling with adaptively refined finite volume methods](https://faculty.washington.edu/rjl/pubs/AN2011/index.html),
    Acta Numerica 2011; Berger, George, LeVeque & Mandli, [The GeoClaw software](https://ar5iv.arxiv.org/html/1008.0455)
    (dry cells, wet/dry by depth, flagging where the surface departs from sea level); George,
    [Augmented Riemann solvers for the shallow water equations over variable topography](https://bu.edu/pasi-tsunami/files/2012/11/George2008.pdf), 2008.
  - Audusse et al., [hydrostatic reconstruction](https://www.math.ntnu.no/conservation/2003/044.html), 2004;
    Bollermann, Chen, Kurganov & Noelle, [a well-balanced reconstruction of wet/dry fronts](https://ar5iv.labs.arxiv.org/html/1412.3580),
    2013 (positivity by limiting the outgoing fluxes of draining cells).
  - XBeach: [processes and model formulation](https://xbeach.readthedocs.io/en/latest/xbeach_manual.html)
    (the wave action balance and roller drive the shallow-water equations through radiation-stress gradients;
    `F_y = D_r sinθ / c`); McCall et al., [EGU 2009](https://meetingorganizer.copernicus.org/EGU2009/EGU2009-4934.pdf)
    (flooding and drying in one scheme); Roelvink et al. 2018, [the non-hydrostatic XBeach](https://www.vliz.be/imisdocs/publications/330001.pdf).
  - Radiation stress and setup: [SWAN, wave-induced set-up](https://swanmodel.sourceforge.io/online_doc/swantech/node30.html)
    (Dingemans et al. 1987: setup from the rotation-free part of the force);
    [Bosboom & Stive, wave-induced forces](https://geo.libretexts.org/Bookshelves/Oceanography/Coastal_Dynamics_(Bosboom_and_Stive)/05%253A_Coastal_hydrodynamics/5.05%253A_Wave-induced_set-up_and_currents/5.5.3%253A_Wave-induced_forces);
    [Coastal Wiki, wave set-up](https://coastalwiki.org/wiki/Wave_set-up) (`η ≈ (5γ/16) H_b`; field `≈ 0.16 H_s`; bars need a model).
  - Active-cell GPU flood codes: [HydroMPM (Water, MDPI)](https://www.mdpi.com/2073-4441/17/18/2662);
    [GeoSWE](https://pypi.org/project/geoswe/); [a GPU implementation for 2D shallow water modelling](https://ar5iv.arxiv.org/html/1309.1230)
    (Brodtkorb's early exit for dry blocks).
  - Tsunami-HySEA: [an operational GPU model for tsunami early warning](https://riuma.uma.es/entities/publication/0073a3e0-2ceb-4ef0-ab32-7b7be570e73f);
    NTHMP [benchmark problems](https://www.weather.gov/media/nthmp/MMS/Benchmarking/TsunamiInundationBenchmarkProblemsSummary.pdf)
    (BP7, Monai valley: run-up within 10 %; BP9, Okushiri).
  - Stelling & Duinmeijer, [A staggered conservative scheme for every Froude number in rapidly varied shallow
    water flows](https://cir.nii.ac.jp/crid/1362262945010575872), Int. J. Numer. Meth. Fluids 43, 2003
    (doi 10.1002/fld.537); Pudjaprasetya & Magdalena, [momentum conservative schemes for shallow water flows](https://global-sci.com/article/82799/momentum-conservative-schemes-for-shallow-water-flows), 2014.
  - Infiltration with shallow-water flood models on GPUs: Buttinger-Kreuzhuber et al., [Environmental Modelling &
    Software 2022](https://repositum.tuwien.at/handle/20.500.12708/101838?mode=full) (full SWE, Green–Ampt, ~1000x on the GPU);
    Hou et al. 2021, [Water Policy](https://iwa.silverchair.com/wp/article-pdf/23/4/1059/924697/023041059.pdf);
    the Sanders group, [surface flow with a groundwater store](https://lboro.ac.uk/departments/abce/unesco-chair/events/webinar-series/2026/11-march-sanders).
  - Soil hydraulic properties at 250 m: FutureWater, [HiHydroSoil v2.0](https://www.futurewater.eu/2020/10/futurewater-releases-new-hihydrosoil-dataset-global-maps-of-soil-hydraulic-properties-at-250m-resolution/);
    CSIRO, [Global 250m soil hydraulic properties](https://researchdata.edu.au/global-250m-soil-hydraulic-properties/3378855).
  - NOAA CO-OPS: [API for data retrieval](https://tidesandcurrents.noaa.gov/api/) (`water_level`, 6-minute,
    preliminary or verified); [verified 6-minute data](https://catalog.data.gov/dataset/nos-co-ops-water-level-data-verified-6-minute).
