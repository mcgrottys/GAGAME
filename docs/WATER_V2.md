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
| **2b** | **The water as columns (§12):** wet is the column holding water; the solver holds the column, not a departure from a plane; residency follows the water; the waves push (setup); the tide and a tsunami enter at Flather ocean faces | §12.5's C1–C5: the stations, a lake at rest, a scripted surge, the bar awash under 2 m, Monai valley |
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
| F3 | A tree's "not now" that was not a stop or a permit refusal was handed over as 64 KB of zeros, mapped and claimed: a flat or black tile standing until an Invalidate. | **fixed**: answered not whole, nothing mapped, asked again when the tree changes (no retry spent); counted (`N not now` in the tree's stats) |
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
6. **The water as columns (§12).** Its four questions are in §12.8.

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

* **Mass:** `∂h/∂t + ∇·q = 0`. Today's `CsSweHeight` without the prism debt.
* **Momentum,** per unit width: `∂(h u)/∂t = −g h_f ∇η − C_d |u| u + F_w/ρ`, where `h_f` is the face
  depth and `F_w` the waves' force per area (below); a face's flux is this times its length. That is
  today's `FaceUpdate` plus the one new term.
* **Wet is `h > ε`,** one tolerance, the solver's own (1 cm; GeoClaw's "dry cells").
* **Positivity:** no face ships more than its donor holds. Today's quarter-volume clamp (`Swe.hlsl:227`)
  is the "limit the outgoing fluxes of draining cells" of Bollermann, Chen, Kurganov and Noelle (2013).
* **Well-balanced:** a lake at rest stays at rest. The face depth is the hydrostatic reconstruction,
  `h_f = max(η − max(b_L, b_R), 0)` (Audusse et al. 2004). That is today's `hface` (`Swe.hlsl:213`), kept.
* **The tide enters only through the open sea,** as the rivers already do: Flather faces on the ocean
  sides, `u_b = u_ext + √(g/h)(η − η_ext)`, with `η_ext` the manager's Level at the face. That Level is the
  tide's phasors (stage 2a), plus the Packet's tsunami (stage 5), plus the setup of a sea no solver holds.
  No moving plane and no debt; the prism arrives because the boundary rises.
* **The waves push.** `F_w = −∇·S`, with `S` the radiation stress of the sea in the column:
  `S_xx = E(n(1 + cos²θ) − ½)`, `S_yy = E(n(1 + sin²θ) − ½)`, `S_xy = E n sinθ cosθ`, where
  `E = ρ g H_rms²/8` and `n = c_g/c` (Longuet-Higgins and Stewart 1964). In the surf zone this is the
  breaking dissipation `D` over the phase speed along the travel, `F ≈ D k̂/c`; XBeach's longshore
  forcing, `F_y = D_r sinθ / c`, is that form. The sea comes from the solved field where its window holds
  it (each component's `a·L`, `k` and direction), and from the eye's spectrum through the bank's own
  breaking law elsewhere. That is one law, read from the two sources the bank already composes.

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
| `src/sim/WaveField.cpp`, the bank | the sea the force reads: per-component `a·L`, `k` and direction in the window; the eye's spectrum through `BreakFactor` elsewhere |

### 12.5 Steps, each with its gate

| step | what | gate |
|---|---|---|
| **C1** | The solver holds columns: the absolute surface, no plane, no debt; the tide at Flather ocean faces from the manager's Level; wet is `h > 1 cm` | the levels at the three stations inside the window (Plum Island at the entrance, Newburyport, Salisbury Point: their official predictions) over a tide cycle, rms and lag no worse than today's; a lake at rest over the real bed stays flat to 1 mm for an hour; volume conserved to 10⁻⁵ of the prism |
| **C2** | Residency follows the water; the +1.2 m threshold goes | a scripted surge (+2 m at the faces) floods the marsh platforms and recedes, with no tile left resident and dry past its hysteresis; the flooded area against a flood fill of the bed under the same level (`FloodWindow`, `SweSolver.cpp:1164`), within one ring of cells |
| **C3** | The drawn water and the hull read columns: the bank's `Δh`, the classifier wet by `h`, `TreeWater` the same | `--twin-surface`'s LEVEL rows unchanged; frozen-clock stills at the entrance where only shore texels change |
| **C4** | The waves push: the radiation-stress force from the solved field and the eye's sea | the setup on a planar test beach against `(5γ/16) H_b` within 15 %; the entrance bar awash under the 2 m sea (the islands gone); no change on a calm day (`H_s → 0`) |
| **C5** | Run-up: the Monai valley benchmark (NTHMP BP7, the 1:400 tank) as a scene, then the asteroid through stages 5–6 | the maximum run-up in the valley within 10 % (NTHMP's standard for this test) |

C1 changes the solver's state, so today's spin-up and the stations are its gate. C3 is what you will
see at the bar. C4 is what makes the bar right.

### 12.6 Cost (estimates, to be measured)

* **The window:** 1788 × 1174 cells (2.1 M), a step of about 0.25 s, up to 16 substeps a frame.
* **The force:** a handful of loads and multiply-adds a face; its sea is sampled from what the bank
  already holds.
* **Residency:** one reduction a batch, over the wet set's edge tiles.
* **The step:** a reduction, instead of a constant.
* **Expected net:** about today's cost. More resident tiles where water spreads, fewer where marsh dries.

### 12.7 What it does not change

* the tide as phasors (stage 2a);
* the term loop's deferral (stage 2), though C1 makes the Level the solver's boundary, which is that
  loop's first new reader;
* the crest law;
* the column plane's drawing;
* the residency of the tile tree.

### 12.8 Questions for Mark

1. **The scheme.** Keep the virtual-pipe faces with the column state (the smallest step; its physics is
   measured), or move to a Godunov (`h`, `hu`, `hv`) scheme with an HLL/HLLC solver, as GeoClaw and
   Tsunami-HySEA use (sharper bores and run-up, a larger change)? Recommended: the pipes for C1–C4; revisit
   at C5 if Monai misses.
2. **The ocean side.** Flather faces (they admit an arrival and radiate the outgoing) or a sponge that
   relaxes to the stateless Level (simpler; it partly reflects)? Recommended: Flather, the rivers' law
   already in the solver.
3. **The wave force's source.** The solver reads the sea itself from the planes the bank holds
   (recommended: no new plane), or the wave solve publishes force planes per bucket (precomputed; about
   13 MB a bucket at the window's 2 m)?
4. **The re-baseline.** C1 moves the inlet's levels by whatever today's plane formulation gets wrong (the
   debt, the clamp). OK to take the stations as the only judge?

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
