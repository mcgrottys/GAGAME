# WATER LAWS — one text per law, read by both processors (2026-10-25)

*Mark, on the storm scene at the Merrimack bar: "the waves are unusually flat around this bar. Figure out
if the math should be making waves roll across that bar, if we are removing them or haven't added them.
And while you are looking at the code find rule breaking code habits and fix them ... the algebraic
functions in their own files/folders making grepping easier ... it takes a while for the seabed to affect
the water mesh/physics ... you might find a pattern violation." And: "I expect the water to change."*

This note is in three parts: what the code was found to do to a 3 m sea at the bar (read, then one run
to confirm the reading); the refactor that puts every closed-form water law in one text under
`shaders/laws/`, compiled by the kernels and by the CPU; and the laws that changed.

---------------------------------------------------------------------------------------------------------

## 1. The bar under the storm: what the math says, what the code delivers

The storm recipe declares Hs 3.0 m, Tp 10 s, from 95°, straight into the mouth. At the scene's instant
(`merrimack_bar.json`, 2026-10-23 18:59Z) the atlas tide is **−0.92 m** on MSL −0.09: the still level is
−1.01 m NAVD, an hour off low water. The ebb delta's crests stand about 0 m NAVD, so at this instant they
are a metre out of the still water.

**What a 3 m, 10 s sea must do there.** Depth-limited breaking (Battjes–Janssen, the engine's own
`BreakHmax`): Hs ≤ (0.88/k) tanh(0.6 k h / 0.88). In 3 m of water a 10 s wave has k ≈ 0.12 and
Hmax ≈ 1.75 m; in 1.5 m, Hmax ≈ 0.9 m. The sea's raw rms at the bar is ~1.1 m (3.0 m / 2√2, shoaled), the
limit's ~0.6 m, so the excess is ~1.9 and the clipped-Rayleigh factor ~0.5: a delivered Hs of ~1.7 m,
every crest over its limit (the crest law's share 0.7–0.9), spilling (the delta's ξ is ~0.2). The seaward
face of the delta is a line of spilling breakers with rollers; the crests themselves are awash under the
sea's own setup (the solver with the push already says so: "the islands are gone — awash with the push",
WATER_V2 §12.10); the lee carries the broken remnant and the swell refracted around the crests through
the swatches (~0.5–1 m), on top of the chop.

**What the code delivered** (read along the delivery path, kernel `CsBankFill` and its twin
`TreeWater::Finish`; confirmed by the layers of one `--wave-map` run, section 4):

1. *Two levels.* The wave solve (`WaveField::BuildInputs`) and the exposure march
   (`ExposureSource`) take the depth from the ATLAS tide, bucketed, at the window's centre. The
   drawn level and the hull's take the SOLVER's surface, which carries the setup. At this instant
   the atlas puts the delta's crests a metre out of the water; the solver, under the push, has them
   awash. So the eikonal sees walls (W1: "the water is solved first with land a wall") where the
   solver has water, and the lee of every crest is a shadow in the solve.
2. *The solved field sheltered twice.* Inside the window every solved component is multiplied by
   `expo`, the line-of-sight exposure page, "exactly as the cascades are" (the kernel's comment dates
   from before W1, when the solve had no walls and "solved comps sailed through the jetty's lee
   unsheltered"). Since W1 the solve goes round the walls itself and diffracts into the lee (Mase, ~⅓ of
   the energy on the tip's line). Multiplying that by the exposure's 0.12–0.5 shelters the same lee
   twice: the solved sea behind a crest, a jetty, a dune is ⅓ × ⅓.
3. *The exposure's floor on the whole sea.* The bank floors the exposure at 0.18 and applies it to
   every band including the chop. Behind a crest that stands 1 m out of the still water the march
   reads excess 1.0 m → transmission 0.19 (0.5 at 0.15 m awash, 0.12 at 1.2 m: fixed metres, not the
   wave's own units). A 3 m sea sheltered to 0.18 is a 0.54 m sea: the smooth, unbroken undulations of
   the screenshot. (The closure in the wave's units — d'Angremond & van der Meer 1996, Kt on the
   freeboard over Hi — gives ~0.5 for a crest awash by a third of the wave height. Not changed here:
   section 5.)
4. *The crease drop.* `ActionTransport` feeds a cell, along each axis, from the one side its own
   velocity comes from. Where two ray families meet — the lee of a focusing shoal, the channel between
   the jetties — the losing family's flux ends. The ledger says 56 % of the swell entering the Merrimack
   window is dropped there. Energy at a crossing is not lost in nature: two trains superpose, and the
   rms is the incoherent sum.
5. *The crest line itself.* The dry weight (`smoothstep(0.05, 0.65, depth)` on the atlas-level depth)
   takes the geometry off the crests, and the shore classifier draws them as sand.

So: **the math makes the waves, and the delivery removes them** — walls and shadows judged at a level
the water has already risen above, a shelter applied twice, a floor in fixed metres, and a
non-conservative crossing. Nothing was "not added" except the overtopping of a crest the setup floods,
which (1) gives for free once the solve takes the solver's level.

**Which of these changed in this branch, and which are named for Mark:** section 5.

## 2. The seabed reaching the water: three beds, three clocks

The bed reaches the water by three paths, at three grains, on three clocks:

| reader | the bed it reads | when it changes |
|---|---|---|
| the bank kernel (`CsBankFill`, `HpHeightChain` at the ring's texel) | the height pages, the finest RESIDENT level no finer than the ring's grain | as the pages' mips land: coarse first (`ParentGate`), the pool at capacity (32.4k of 32.8k, the wave planes holding the most) |
| the SWE (`CsSweBedFill`, the same chain at the cell's grain) | the same pages | the same |
| the wave solve (`BuildInputs`: `Compositor::SampleHeight`, the CPU stack through the tree) | the full stack, at the cell | at the next key roll (the tide bucket, 0.25 m; the solver's current signature, every 90 s), then the solve (10–60 s), the prefill, the swap |
| the hull (`TreeWater::BedAt`: `HeightPage`'s CPU memo) | the stack at the page's mip-0 texel centres | at once |

"It takes a while for the seabed to affect the water" is the first two rows while the fine mips climb,
and the third row's latency. The pattern violation is not in the climb (the climb is the residency's
honest absence law): it is that the wave solve's **inputs are a different water** from the solver's — the
atlas level, the stack's bed — so the solved sea stands on a bed and a level the drawn water does not,
and changes on a clock of its own. Section 5's first law change closes the level half; the bed half is
the open item "one bed for both readers" (WATER_V2 §12.15).

## 3. The refactor: `shaders/laws/`

### 3.1 The habit

Every closed-form law of the water had two bodies: HLSL in `shaders/Jet.hlsli` or inline in
`WaterBank.hlsl`, and C++ in `src/sim/WaterTerms.h` "transcribed with the physical argument", plus a
third copy of the breaking height inline in `Swe.hlsl` (`hb`), a second Guo dispersion (`WaveK`), the
dry band written four times, the window weight, the yield share, the per-component orbit and swash
factors and the whole band law (gains → envelope → kEff → excess → factor) each written twice. The
inventory of 2026-10-22 counted four kernel↔hull drifts; reading for this note found a fifth: the twin's
breaking excess was judged on the EXAGGERATED band rms (`m_bandRms × heightScale` with no `/ exag`),
the kernel's on the physical one — the hull broke its sea earlier than the bank by the display factor.
And `LipFlightTau` sat outside `Jet.hlsli`'s include guard.

A law written twice is checked by a gate that compares the two; a law written once needs no gate for
that. Mark's rule for this branch — do not rely on self-tests, the water will change — is the same
statement: the text is the contract.

### 3.2 One text, two hosts

The laws are HLSL files under `shaders/laws/`, written in the subset both compilers accept:

* scalar `real` arithmetic (the host owns the typedef: `float` on the GPU, `double` on the CPU);
* `inline` free functions, structs returned by value, `static const real` constants;
* the intrinsics `max min clamp saturate lerp smoothstep sqrt tanh sinh exp log pow cos sin abs floor
  rsqrt` (the C++ host defines them on `real`);
* `GA_PLANET_G`, the planet's surface gravity, from the host (a kernel's constant buffer;
  `PlanetGWave()` on the CPU).

No vector types, no `inout`, no texture reads: a law takes components and returns a struct. Both
compilers were held to the dialect before the first file was written (dxc 10.0.26100 at `-HV 2021`,
MSVC `/W4 /std:c++20`).

| file | laws |
|---|---|
| `laws/Dispersion.hlsli` | `BandPhaseSpeed`, `ShoaledKh`, `OrbitCoth`, `OrbitCothK`, `ShoalFactor`, `WaveCurrentAmp` → `WaveCurrentGain{amp, blocked}` |
| `laws/Breaking.hlsli` | `kGammaHs`, `BreakHmax`, `BreakFactor`, `SeaBreak` (rms_raw, kEff, depth → excess, factor), `DryWeight`, `TroughBound` |
| `laws/Bands.hlsli` | `BandLaw` (the three cascades' gains, the raw envelope, kEff, excess, factor), `YieldShare`, `WindowEdgeM`, `WindowWeight` |
| `laws/Crest.hlsli` | `LipFall`, `LipFallDe`, `LipReach`, `SpillReach`, `SwashShare`, `SwashFactor`, `PlungeShare`, `CrestExcessShare`, `CrestState`, `CrestLaw` |
| `laws/Wake.hlsli` | `WakeVessel`, `WakeSample`, `WakeBranch`, `WakeOne` |
| `laws/Laws.hlsli` | all of them, for a host that wants the set |

The hosts:

* `shaders/WaterBank.hlsl`, `SeaChurn.hlsl`, `Swe.hlsl`: `typedef float real;`, `GA_PLANET_G` from
  their constant buffers, then the include. `Jet.hlsli` is gone; `CrestLaw`, `WakeOne` and the band
  block left the bank kernel for the laws.
* `src/sim/WaterTerms.h` is the CPU host: `using real = double;`, the intrinsics, `GA_PLANET_G`, the
  include into `ga::law`, and using-declarations into `ga`. The 720 lines of transcription are gone;
  the file is the contract and a page of history. `proofs/water_terms.py` transpiled the old header and
  has been dead since 2026-10-20 (it looks up `kGWave`); it is not revived here.

`CMakeLists.txt` adds `shaders` to the include path. Shaders still compile at run time from
`--shader-dir`; the laws are found relative to the kernel that includes them.

### 3.3 What a caller sees

* `WaveCurrentAmp(curX, curZ, dirX, dirZ, c0)` returns a named pair; the kernels' `.x/.y` are
  `.amp/.blocked`.
* `CrestLaw(CrestState, ...)` returns the new state on both processors. The kernel packs its
  displacement and slope into the state and unpacks; the Jacobian terms it does not draw it ignores.
  The translator's sandwich on a point is written as the shift it is (GA.hlsli `MotorPoint` with the
  real part 1 and the dual part τ/2 gives p + τ; Pga.h `Motor::Translation` the same): the shared text
  has no `float4`.
* `WakeOne(WakeVessel, px, pz, sampleM, WakeSample)` accumulates and returns; the kernel builds the
  vessel from its two `float4` rows.
* `BandLaw(...)` returns the gains, the raw envelope, kEff, the excess and the factor; the kernel's
  `gRmsRef` is now the PHYSICAL unit-sea band rms (`SeaLayer::BandRmsPhysical`), as the twin's always
  was, and the six `/ gPatch.w` that undid the exaggeration inside the physics are gone. The
  exaggeration is applied once, where it always was: `d *= dry * gPatch.w`.

## 4. The run (one, before the change; one after)

`gagame.exe scenes/demos/merrimack_bar.json --storm 3.0,10,95 --headless --frames 6 --dump ... --wave-map
out/review/storm/wave_map.png --tool dump-water-state --set streaming.offline=true`: the solve with
nothing between it and the eye (every layer), and the inlet box's bed, level and exposure at 10 m.

**Before** (kSolverVersion 14, the law of 2026-10-24):

* The solve datum: MslNavd −0.093 + tide −0.917 = **−1.01 m**; the delta's crests are under water at this
  level (the depth layer: a dark arc of 1–3 m seaward of the tips, no dry cells on it). So at this instant
  the walls and the line-of-sight shadow are the jetties' alone; the bar's flatness is not (1) or (3).
* **The ledger: 80.9 % of the swell's action dropped at creases**, 12.9 % onto land, 7.4 % out by the edge.
  The "dropped" layer is dense streaks over the whole window — every convergence of the ray field, the
  delta's lee thick with them — and the Hs layer is dark west of the delta's arc and in the channel.
  The crease drop is the bar's flatness.
* The `--tool dump-water-state` export was blind: at frame 6 the slow field's bed read −24 m under the
  jetties (the stack cold, the cube answering), so its exposure read 1.0 everywhere. An instrument trap,
  noted: that tool reports the composed stack only once the tree is warm.

**After** (kSolverVersion 15, the same command on the new binary; `out/review/storm_after15`):

| the action's ledger, all 32 components | before | after |
|---|---|---|
| out through the window's edge again | 7.4 % | 11.6 % |
| onto land (the beaches, the jetties) | 12.9 % | 6.3 % |
| turned back (dropped at creases, before: 80.9 %; head-on now) | **80.9 %** | **6.5 %** |
| broken at the water's limit (the cap in the march) | — | 158 % |
| unaccounted (the diffraction gather's mixing) | −1.2 % | −82 % |
| the action over the swell shoaled without refraction, p50 | 0.62 | **0.89** |
| … p99 / max (a focus, bounded by the cap) | 1.57 / 2.98 | 2.65 / 15.4 |
| the march: rounds until still / the solve | 30–53 / 313 s | 32–80 / 409 s |
| the analytic gates (`--selftest` [wavetest]: deep plane wave, straight contours, the breakwater) | pass | pass (exact, 1.1 %, 0.37 / 0.20) |

The layers (`out/review/storm_after7`, the same law): the Hs layer carries the sea between the jetty tips
and up the channel (a bright streak into the mouth, where before it was dark), the delta's arc is a ring of
depth-limited sea, Salisbury's beach shoals the swell to its limit, and the turned-back layer is a handful of
short lines (a head-on meeting at the river's bend, two creases off the south jetty) where the dropped layer
had been dense streaks everywhere.

**What the ledger says about the diffraction, honestly.** 158 % broken against −82 % unaccounted is one
number read twice: W1's diffraction gathers the neighbours' ACTION averaged into the receiver, and beside a
focus (now a real one, no longer leaked away) that invents action, which the cap then breaks. The sea the
gather draws beside foci is at the water's limit where it should be somewhat under it. Three conservative
forms were built and measured on this branch and set aside (section 5.2); the one-pass conservative
diffraction is the open item.

**Also found on the way, in the march itself:** the stillness test (a move under 10⁻⁷ of the deep
action) was tighter than the atlas's own floor and is 10⁻⁵ now (5 × 10⁻⁶ of the deep amplitude); the guard
reports how far from still it stopped; the ledger's "crease crossings" figure is a sum over every crease a
flux crosses, not a share, and the log says so. The storm's solve at this instant is 409 s (313 before):
the two-sided feeding's links against the phase cost 10–30 more passes.

## 5. The laws that changed

1. **The solved field is not sheltered by the exposure page** inside its window. The solve carries
   its own walls, its own diffraction and its own breaking; the page is the cascades' closure, outside
   the window and for the chop. The cascades' yield compares the solved sea's DELIVERED energy with the
   bands' sheltered one, as before.
2. **A convergence is not a crease, and a cell holds no more than the water lets a wave stand to.**
   A cell is now fed by every wet neighbour whose flow enters it and goes the cell's way (the donor-cell
   form of div(N F) = 0); a ridge cell of the phase (both neighbours lower on an axis) carries its ray
   along the ridge; a head-on family is turned back and counted. Where the current closes a loop of F
   (the ebb jet's shallow pocket, near blocking: flows meeting from every side) the loop fills only to
   the water's limit: a cell's action is capped at one wave at Battjes–Janssen's Hmax (Miche's
   steepness in deep water), as waves meeting from every side break; the focus geometric optics makes
   is bounded the same way (a caustic cell reached 505× the deep amplitude once the creases stopped
   leaking). The ledger counts what breaks. `kSolverVersion` 15: every cached solve re-solves.
   *What was tried and set aside* (each measured on the storm, each in the run logs under
   `out/review/storm_after*`): feeding only from LOWER phase (acyclic, exact in one pass, but the
   current's part of F runs along the contours and was refused at every other cell: 83 % turned back);
   a conservative SCATTER of each sender's flux along its receiver's crest (the ledger closed to 0.0 %,
   but its deferred deposits relaxed a cell a pass whatever the order, phase or a topological sort of
   the flow graph: 190 passes, 27 minutes a solve); the adjoint GATHER of the senders' fluxes (exact on
   a plane wave to 10⁻⁷ and on straight contours to 0.6 %, four passes, but on the window 81 % of the
   swell went unaccounted, the cause not found). The diffraction stays W1's gather of the neighbours'
   action, which invents action beside a focus and conserves nothing there; the ledger says how much
   as "unaccounted", and the cap takes it back. That is the open item of this branch.
3. **The twin's breaking excess** is judged on the physical band rms, as the kernel's is.

Named, not changed (each a law of its own, for Mark's call):

* the wave solve and the exposure march at the SOLVER's level (tide + setup) per cell, not the atlas
  bucket at the window's centre — the solver's eta is already read back for the current and thrown
  away (`ResampleFlow`); it would join the key as the flow does;
* the exposure's transmission in the wave's units (freeboard over Hi, d'Angremond 1996) read at the
  bank from a page that stores the freeboard, instead of a transmission in fixed metres baked into the
  page;
* the dry band `smoothstep(0.05, 0.65, depth)` and the chop's 0.4 m floors: fixed metres;
* the SWE's own wave march (`CsSweWaves`) is a third model of the one sea pushing the water; W1's
  planes over the solver's domain would make it the solved field's.

## 6. Other habits found and fixed

* The cascade band cuts (756 / 60 / 12 m and the 0.9 π N/47 guard) were written in four places
  (`SeaLayer.cpp` twice, `FrameLoop.cpp`, `WaterBankLayer.cpp`) beside `OceanFft`'s own
  `m_bandLo/Hi`. One static, `OceanFft::BandCut`, now.
* `LipFlightTau` defined after `#endif` (every includer redefined it).
* Six divisions by the display exaggeration inside the physics (`/ gPatch.w`): gone with the physical
  `gRmsRef`.

## 7. The roller's curvature, the hulls, and DX12 Ultimate (2026-10-25, Mark's second look)

Mark, after running the bar: "much better ... the only issue I notice with the mesh now is that the tops
seem a bit flat. I wonder if that diffraction work might address it." And: "make sure we are not under
utilizing DX12 Ultimate shader features. I'd hope these physics could apply to the boats."

### 7.1 The flat tops are the roller's drop, not the diffraction

The diffraction decides how much action lands where along a crest; the crest's shape is the crest law's
(`laws/Crest.hlsli`). A spilling crest over its limit was carried forward to the cusp and the water past
the cusp lowered linearly along its own face, the roller, `ePast × face` — applied at crest weight alone,
so every label on the top dropped by the same amount: a shelf with a lip at its end, the plateau in the
close-up. The roller is water that cannot stand on the FRONT FACE sliding down it, so a label is now
lowered by the roller in the share of the face it sits on: the sheet's descent along the travel, −m (the
`m` the flight's translator already computes), over the roller's own face k Hc / 2, saturated. Nothing on
the level top and on the back face, the whole drop where the sheet is as steep as the roller's face or
steeper. The top stays rounded and leans forward; the face below it carries the roller. The jet is in
free flight and falls by its parabola wherever it is, unchanged. One continuous expression; a saturate on a
ratio, which is a domain clamp, not a case.

Two things that stack on it in the storm and were checked: the march's per-component cap (Hmax / 2 for
ONE wave) is far looser than the sea's total limiter (Hmax / 2√2 on the rms of all 32), so the two never
flatten the same crest twice: the cap bites only a focus, the limiter then scales every component by one
factor and the shapes survive. And the trough bound flattens troughs, not crests; the eye reads the crests
as flatter by contrast.

### 7.2 The hulls

The crest law is one text, so `TreeWater::Finish` runs the rounded roller the frame it is drawn with: a
hull rides the rounded top and the roller on the face, because there is no second body to update. (The
hull's state carries the sheet's slope, so its `m` is the same quantity.) Still open and older: the hull
does not feel the fleet's wakes (`TreeWater::SetBoats` has no caller; REVIEW_2026-10-17).

### 7.3 DX12 Ultimate, audited against the water path

The device of the storm runs: RTX 5060 (laptop), shader model 6.7, mesh shader tier 1, sampler feedback
tier 0.9, tiled resources tier 4, resource binding tier 3. What the water asks of it today:

| feature | used | by | fit for the water |
|---|---|---|---|
| mesh + amplification shaders (SM 6.5) | yes | the globe's mesh (`MsMain`), the column ladder's lip (`MsLip`), buildings (`AsShape`/`MsShape`) | the sheet IS a mesh-shader job; done |
| bindless over the heap (tier 3) | yes | every kernel (`gT[]`, `gTA[]`, `gU[]` by slot) | done; SM 6.6's `ResourceDescriptorHeap[]` would only drop the root-signature ranges |
| reserved resources, tiled residency (tier 4) | yes | every page tenant, the banks | done |
| **variable rate shading** (tier 2) | **no** | — | **the one clear gap.** REVIEW_2026-10-21 measured the frame's GPU budget dominated by WATER PIXELS (the two-ray water, the caustic tiers). The bank already knows per texel where the sea is smooth and where it breaks (the detail plane's gains, the foam, the fold grid): a shading-rate image written from it (2×2 over the smooth open sea, 1×1 within a wavelength of a crest's foam and the fold grid's held blocks, by distance beyond) is a direct win on the dominant cost with no change to the laws. A render change; it wants the A/B and Mark's eye |
| **sampler feedback** (tier 0.9) | **no** | — | the second real fit: the residency's wants are PREDICTED by the CPU walk (SetView 3.0 ms, PredictWants 1.0 ms a frame, the review of 2026-10-17); `FeedbackTexture2D<MinMip>` written by the globe's pixel shader as it samples the page arrays is "a reader wants what it reads" said by the hardware. A design change to the residency, not a kernel edit |
| wave intrinsics (SM 6.0+, available at `cs_6_0`) | no (water); yes (buildings) | — | small: `CsBankFill` reads 33 residency bytes per texel for the planes' share (`nHave`), the same bytes for a whole tile; one lane could read them and broadcast (`WaveReadLaneFirst`). A few percent of that kernel |
| 64-bit atomics (SM 6.6) | no | — | the column plane packs a 14-bit height with an 18-bit label into one 32-bit word for its three `InterlockedMin/Max`; a 64-bit word would carry a full float height. The 14 bits are a centimetre over ±82 m, which is under the drawn floor: not a need |
| 16-bit types (SM 6.2) | no in compute; the pixel stage has an fp16 variant | — | the bank sums 32 components; fp16 accumulation would lose the hull's twin to rounding. Not for the kernels |
| DXR / inline `RayQuery` (SM 6.5) | no | — | the sheet is a height field with columns; the engine marches it (`TerrainRay.hlsli`, the maximum-mipmap cast, the PCSS sun map). A BLAS of the sheet rebuilt every frame would cost more than the march and answer the same ray. Not a fit |

So: the water's kernels are not leaving a shader feature on the table that would change the physics; the
two features that would change the frame are VRS on the water pixels and sampler feedback for the
residency's wants. Both are proposed here, neither is in this branch.
