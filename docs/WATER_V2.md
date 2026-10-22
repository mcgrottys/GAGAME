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
scaled. It makes every breaking artefact taller than its own depth allows.

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
| **1** | The declarations: `Quantity`, `Reach`, `Grain`/`Fold`, `TimeForm`, `Clock`, `Identity`, `Producer`; today's producers wrapped behind them; `WaterManager` = `WeatherManager` + the registry. No behaviour change. | frozen-clock stills byte-identical on the gate scenes; line count of the water code DOWN (the roadmap ledger) |
| **1a** | The mip ramp (D1) and the NULL-read climb (D2), the no-bytes fill (F6), the time-based retry (F8) | a descent rail: max per-frame pixel difference down, no frame with a snap; `[restest]` aniso rows 0; `[stuck]` 0 |
| **1b** | Music as the first new producer (§10): band table, scene bindings, Media Foundation + XAudio2 playback | the water and a light term follow the file's bands at a place; two runs agree frame for frame |
| **2** | One term table: the bank kernel and `TreeWater` loop over terms; tides evaluated on the GPU from the atlas phasors (no per-tile CPU corner params) | stills identical to stage 1 within the declared tolerance; frame time not worse; the hull probe |
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
| F2 | **The pop is by construction.** A landing tile's map byte drops a whole mip in one turn ((m+1)·16 → m·16) and every sampler's min-LOD clamp with it: each tile snaps into focus, a descent is a patchwork of snaps. The ×16 byte has room for a ramp (D1): write the fractional bytes over 4–8 turns; trilinear blends m and m+1, both held by law. | **next (D1)** — your call on a 4–8 frame fade-in |
| F1 | Finding 61 open: the colour's anisotropic sampler reads NULL tiles past the gather's clamp (measured 1,606 of 473,088 probe points, up to 36 % zero): dark streaks at tile frontiers at grazing angles. Fix: `CheckAccessFullyMapped` on the sample, re-sample one mip coarser in the quads that touched a NULL tile (proven on this GPU in `ResidencyFloor.hlsl`). | next (D2) |
| F3 | A tree's "not now" that was not a stop or a permit refusal was handed over as 64 KB of zeros, mapped and claimed: a flat or black tile standing until an Invalidate. | **fixed**: answered not whole, nothing mapped, asked again when the tree changes (no retry spent); counted (`N not now` in the tree's stats) |
| F4 | Finding 9 open: kernel bilinear taps cross a tile frontier under a one-byte floor and read a NULL neighbour (a seam line in the water while the neighbour is coarser). | next (D2's `Load` form) |
| F5 | A refused refresh left a held tile stale AND pending forever: `PendingCount()` never 0, no settle closes. | **fixed**: stale (its version bit is the truth of its bytes) but not pending |
| F6 | A DirectStorage tile with no landing slot is mapped, filled with stale staging bytes and never landed: a leaked slot and a rectangle that never sharpens. The 512 slots are reachable at 96 tiles a turn with a 2-turn fence lag. | next (decide the path before mapping) |
| F7 | `TileAtlas2D::FlushResidencyMap` has no caller: the velgrad lens reads an Init-time map. | debug lens only |
| F8 | Four retries in four turns, then a permanent hole for the run. | next (time-based retry) |
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

1. **The exaggeration.** The solved sea is drawn at 1.6 × 1.15 = 1.84× physics in nearly every scene. Keep
   it, bring `wavefield.exag` to 1.0 (the sea as measured), or one stated number for the whole water?
2. **The mip ramp (F2/D1).** Detail fading in over 4–8 frames (~100 ms) after a tile lands, in place of the
   snap. Yes?
3. **The stages' order.** 0 (done) → 1 declarations → 2 one term table → 3 harmonic playback → 4 series in
   time → 5 basin + impact → 6 shelf + nesting, with music after 1. Anything you want earlier?
4. **The tsunami's first form.** The frequency-domain Packet (stateless, dispersive, what rays give) first,
   with the time-domain basin run as its cross-check — or the time-domain run first?
5. **The waterline.** Today one scalar for the planet. V2 makes the land/water line read the local level
   (the bank's per-texel departure). It moves every shoreline a little (the tide's spatial variation along a
   coast). OK to take that re-baseline?

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
