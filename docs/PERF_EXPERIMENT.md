# PERF_EXPERIMENT -- the renderer performance experiment (2026-09-05)

Branch `claude/storm-rail-renderer-check-4fb3cb`. Base f4a380f. Instrument commit 9e43985 (`--gpu-time`,
`--no-vsync`, boot adapter report, `tools/imgdiff.py`). The Understand phase's findings are in the
worktree's `out/perf/understand.json` (digest `out/perf/digest.txt`); the panel's five plans, three
judgements and this synthesis are in `out/perf/design.json`. Results are appended below the plan as
each step lands.

The owner's brief: performance with every non smoke-and-mirrors technique (no TAA, no DLSS, no upscaling,
no temporal reuse), no loss of fidelity, GA doing real work, everything fades (the wave/whitecap pop-in is
the known offender), 150+ fps then effects; references: the Microsoft Tiled Resources sample and the
SparseGeoAlgPlatform Catalog (not integrated this pass).

## Baseline (this build, storm rail `--rail-flood --storm 3.0,10,95 --start 2026-08-28T19:30:00`, 1600x900)

| what | whole flight | helm phase |
|---|---|---|
| `--bench` RENDER (CPU record + fence + GPU) | 6.60 ms (151 fps) | ~8.0 ms |
| loop (bench, serialized) | 9.62 ms | ~12-14 ms |
| outside RenderFrame (sim, residency, walk) | 3.02 ms | ~5.0-5.6 ms |
| GPU whole frame | 3.80 ms | 5.96 ms |
| GPU globe.mesh (mesh shader + Globe.hlsl PsMain) | 2.84 ms | 4.23 ms |
| GPU waterbank.fill | 0.70 ms | 1.20 ms |
| GPU sea (SWE + FFT + churn) | 0.50 ms | 0.49 ms |
| windowed 1600x900, helm pose | 114 fps vsync / 116 fps tearing | |
| windowed 1600x900, globe pose | 159 fps vsync / 266 fps tearing | |

The panel (2048x1280) is on the Radeon 780M; the RTX 5060 owns no output, so every windowed present is
cross-adapter. A dead 28 MB readback (WeatherManager::RefreshMirror) stalls the loop 16-27 ms every 61
frames. The engine is not bit-deterministic run to run: 3-11 horizon pixels at the helm, |d| <= 4; bird and
globe are bit-identical between runs.

# docs/PERF_EXPERIMENT.md -- the one execution plan (synthesized 2026-09-05)

Worktree: `C:/Users/lordc/source/repos/GAGAME/.claude/worktrees/storm-rail-renderer-check-4fb3cb` (branch `claude/storm-rail-renderer-check-4fb3cb`, instrument commits 9e43985 + 5fcda2a on top of f4a380f). Findings: `out/perf/understand.json`. Four plans (GA mathematician, GPU architect, streaming engineer, fade specialist, systems engineer) and three judges (skeptic, purist, engineer) merged; every number below was re-read against the code and the CSVs before it was kept.

## 1. Thesis

The helm frame is CPU-bound and one dead readback owns the p99. Fenced split at this build (out/instr_bench/metrics.csv minus gpu_ms.csv, helm = frames >= 900): CPU 4.66 ms outside RenderFrame (5.03 with the spike frames) + 1.92 ms record/fence = 6.58 ms against 5.96 ms of GPU; whole flight 2.74 + 2.61 = 5.35 ms CPU against 3.80 ms GPU. WeatherManager::RefreshMirror stalls the loop 22.2 ms (n=20) every 61 frames for a mirror nothing in the loop reads. The GPU is one draw: globe.mesh 4.23 ms at the helm (2.84 whole), waterbank.fill 1.18 (0.70), sea 0.49 (0.50), everything else < 0.05 ms.

The exact items every plan and every judge agree on remove ~3.5 ms of helm CPU with the identical want stream and record buffer, and ~0.8-1.2 ms of GPU with identical pixels. That puts the fenced helm loop near 5 ms; whether the WINDOWED number reaches 150 fps depends on a 2 ms gap nobody has measured (probe P1). GPU headroom for effects stays thin until the owner adjudicates the two 'visible' levers (the screen-projected CDLOD split, the shipped slope plane) or async compute proves its overlap (P13).

The pop-in is a consumer problem: four gates in the bank kernel step on the residency byte, Drop() empties a tenant on every bucket roll, and the wave pyramid's parents are `.fold` markers rebuilt by 4^m recursive mip-0 reads on every serve. The fix (steps 15-20) is the fade specialist's design with three corrections from the judges, bit-identical at settle and judged by a per-frame difference series through every landing.

## 2. The measured baseline (storm rail --bench --gpu-time, 1600x900, RTX 5060 Laptop)

| quantity | whole flight | helm phase |
|---|---|---|
| [rail] RENDER (CPU record + fence + serialized GPU) | 6.42 / 6.60 ms (two runs), p95 10.31, p99 12.25, max 18.0 @561 | ~8.0 |
| loop mean | 9.48 / 9.62 ms | ~12.0-12.5 (13.8-14.2 on frame%3==1) |
| outside RenderFrame | 3.06 ms | 4.66 (5.03 incl. spikes) |
| GPU whole-frame | 3.80 ms | 5.96 |
| globe.mesh / waterbank.fill / sea | 2.84 / 0.70 / 0.50 | 4.23 / 1.18 / 0.49 |
| sea.swe / churn / fft | 0.28 / 0.13 / 0.09 | 0.24 (bimodal 0.20 on 260 frames, 0.50 on 40) / 0.16 / 0.09 |
| helm sections | | SetView 3.03, PredictWants 0.66, weather.Update 0.42, stat 0.21, sea.SetTime 0.07, unbracketed residual 0.60 (the 17 wave-plane Wants) |
| 61-frame comb | +22.2 ms on frame%61==1; 20 of the 26 frames over 16.7 ms | |
| windowed 1600x900 | helm 8.77 ms vsync / 8.64 tearing (114/116 fps); globe 6.30 / 3.76 (159/266); present is CROSS-ADAPTER (RTX owns 0 outputs; panel 2048x1280 on the 780M) | |
| fidelity floor | tools/imgdiff.py: none / sub-lsb (max 1, identical >= 99.9 %) / noise (max <= 6, n(>1) <= 64, identical >= 99.99 %) / VISIBLE; helm A/A 3-11 horizon px, bird and globe 0 px | |

## 3. THE GATE (every step; the number that proves the saving is named in the step)

```
# build (PowerShell tool; junction cache/ and data/ first, see memory worktree-run-setup)
cmd.exe /c \"<worktree>\\build.bat\"
build\\bin\\gagame.exe --selftest            # [pga][dxtest][gatest][composetest][watertest][tiletest][atlastest] all PASS

# the five stills, 240 frames each (G = out/gate/stepNN); A/A on the new binary FIRST, then A/B vs the previous step, then vs out/gate/step01 (= out/baseline)
gagame.exe --sea --one-water --headless --campos 120,-10 --cam 7,92.5,-1.5 --frames 240 --storm 3.0,10,95 --start 2026-08-28T14:00:00 --dump G/helm.png
gagame.exe --sea --one-water --headless --campos 120,-10 --cam 7,92.5,-1.5 --frames 240 --storm 3.0,10,95 --start 2026-08-28T19:30:00 --dump G/helm_ebb.png
gagame.exe --sea --one-water --headless --campos 380,10 --cam 1500,272,-88   --frames 240 --storm 3.0,10,95 --start 2026-08-28T14:00:00 --dump G/bird.png
gagame.exe --sea --one-water --headless --globe-cam 42.74,-70.87,7          --frames 240 --storm 3.0,10,95 --start 2026-08-28T14:00:00 --dump G/key7km.png
gagame.exe --sea --one-water --headless --campos 0,0 --cam 200000,0,0        --frames 240 --storm 3.0,10,95 --start 2026-08-28T14:00:00 --dump G/globe.png
py -3 tools/imgdiff.py --pairs out/gate/stepNN-1 G --out-dir G/diff
#   required: bird, globe, key7km 'none' (0 px); helm + helm_ebb 'none' under --settle-sync (step 1), else within the recorded floor ('noise') until P16 lands
#   a step declared 'visible' (none in this plan's perf track) is judged by the owner with a lens, never by these thresholds

# the bench (the instrument's flags, comparable to out/instr_bench.log)
gagame.exe --sea --one-water --headless --rail-flood G/rail --bench --gpu-time --storm 3.0,10,95 --start 2026-08-28T19:30:00 > G/bench.log
#   [rail] RENDER/loop/p95/p99/max, the section table, [gpu] table, G/rail/metrics.csv + gpu_ms.csv; after step 1 also --bench-overlap
# the recording (only for steps that touch residency / LOD / fade), against the Sep 2 baseline
gagame.exe --sea --one-water --headless --rail-flood G/cap --mp4 G/rail.mp4 --storm 3.0,10,95 --start 2026-08-28T14:00:00 --tile-budget 3000
py -3 tools/raildiff.py C:/Users/lordc/source/repos/GAGAME/out/rail_storm.mp4 G/rail.mp4   # per-frame ssim + tblend difference series
# bank changes: --dump-fibers at helm, helm_ebb, key7km -> disp/param/detail |delta| = 0, zero signs ignored
```

The helm/helm_ebb --start pair is inferred from out/baseline logs (ebb's solve datum t = 1787945400 = 19:30Z; helm, bird and key7km share the 14:00 wave key 5d8508bd527c9591); step 1's argv echo pins every recipe in its log. The key7km still is a 240-frame `--globe-cam` still (the baseline was one); add `--rail-key 21` (the rail's own motor at the 7 km key) if that still is not pixel-identical to out/baseline/key7km.png.

## 4. Execution order (credible ms / effort, instruments first, probes before doubted steps)

| # | step | class | effort | expected | gate number |
|---|---|---|---|---|---|
| 1 | Instruments and harness: argv+rev echo, PROF slots (ProcessQueues by phase, wave-plane Wants, exposure Want, bank tile list, meshlet memcpy), CSV re-pairing, --bench-overlap, windowed --gpu-time with the GPU gap, GetFrameStatistics, tools/raildiff.py, --settle-sync + the A/A bisect, --dump-hdr + imgdiff v2; re-record the baselines from the unchanged binary | none | M | 0 ms | A/A five stills; bench reproduces 6.42-6.60 / 3.80 / 5.96 |
| 2 | Delete the dead weather mirror readback (lazy Query) | none | S | -0.33..0.42 ms mean + the 22 ms spike every 61 frames | no comb in metrics.csv; weather.Update helm <= 0.10; --dump-water-state byte-identical |
| 3 | Scene hot-reload stat -> directory watcher | none | S | -0.20 helm | slot 1 ~0.00 |
| 4 | Dense tracked-slot array; bottom-up column stamp scan; per-tenant Drop lists; indexed m_mapped | none | S | -0.7..1.1 helm | Want touches/hits identical; --res-trace stream identical; the 0.60 residual gone |
| 5 | PredictWants on a worker, ordered replay before ProcessQueues (keep the 72-frame lookahead quirk) | none | M | -0.66 amortized (-2.0 on frame%3==1) | FNV of the predicted rect list identical; frame%3 loop means within 0.3 ms |
| 6 | Polar-plane horizon cull at every altitude (today's margin above 10 km), camera face first, discard-free shipped PSO, MsMain dedupe | none | S | CPU -0.2..0.5; GPU 0..0.75 (P2/P4) | --mesh-stats far buckets; PS invocations; depth-buffer diff bit-identical |
| 7 | Camera-independent walk cache (dir, tangent pos, Mercator bound, anchors, Jacobians) | none | M | -0.5..1.0 helm | record buffer + Want stream byte-identical over 300 helm frames |
| 8 | PsMain/MsMain grade-0 gates, one mask fiber, identical-argument fetches, dead cube taps; (b),(h) after P3 | none | M | GPU -0.3..0.6 helm/bird | dxc -Fc counts; globe 0 px; bird + key7km bit-identical |
| 9 | CsBankFill w==0 / dry skips, shared band physics; CPU 25-corner dedupe, dead bed sample, phasor cache (Level()'s own loop) | none | S | GPU -0.1..0.2; CPU per P6 | --dump-fibers |delta| = 0 (zero signs ignored) |
| 10 | Residency turn: IDStorageFile handle cache, compact-key sorts (same permutation), notify outside the lock, per-slice map uploads, touched-tenant barriers, atomic Tracked::state | none | M | descent RENDER p95 10.3 -> ~7-8; ~0 at helm | P5 bracket; --res-trace stream identical; Failures() == 0 |
| 11 | Relief-true frustum bound (MAX pyramid + kDispAllow) with a CULL-side audit -- only if P4 shows the omnidirectional near-leaf share | none under audit | L | CPU -0.8..1.2; GPU -0.2..0.5 | cull-side audit 0 violations; depth diff vs the 9 km bound bit-identical |
| 12 | Cloud-march uniform floor skip -- only if P7 says slices 0-1 are zero in reach (CB row at the END) | none | S | GPU up to -0.3..0.6 on clear sessions | dxtest; stills |
| 13 | CDLOD walk as pure geometry jobs, Wants replayed in DFS order -- only if the helm CPU is still > ~5.5 ms after 2-11 | none | L | hides the residual geometry | FNV of records + Want sequence identical; pre-RenderFrame helm <= 2.5 |
| 14 | Hygiene: tonemap into the backbuffer, FFT cascade interleave, batched bank barriers, dead sceneColor clear, silent PIX scopes, BedAt dedupe in CsSweFlux | none | S | tens of microseconds | debug layer clean; fibers byte-identical |
| 15 | POP-IN 0: per-tree wave solve snapshot + key (H6 correctness) | none | S | 0 | --tree-audit on the old tree after a forced roll |
| 16 | POP-IN 1: materialize the served wave-pyramid parents (mips >= 2) | none | S | 0 / roll seconds | byte compare vs the recursive rebuild; --res-trace deficit drains |
| 17 | POP-IN 2: finest resident rung, coherent fraction r = |<sp>|, h-weighted shed, resolved-fraction handover, 2f(1-f) deficit, lockstep planes, envelope fallback | none at settle | M | 0 | fibers |delta| = 0; D_f bounded; energy within 1 %; gatest sinc |
| 18 | POP-IN 3: the fade byte -- slope-limited effective mip in a second map; fractional reads of bed/exposure/pages; answers lerped | none at settle | M | 0 | eff == have when idle; monotone + bounded series; tblend series |
| 19 | POP-IN 4: generations (second slot per bucketed tenant) with the cl2 cross-fade; DropFaces at weight zero | none at settle | L | 0 / removes the roll absence | forced roll passes the series; frame 240 vs a new-bucket still |
| 20 | POP-IN 5: cache fread + prefill on a worker (interactive; rails keep blocking) | none | M | interactive 0.34-4 s hitch | no frame > 33 ms across a roll; tree folder hash identical |

Steps 15 and 16 are S and may run any time after step 1; steps 17-19 need step 10(d) (per-slice uploads) first.

### Step details that carry the exactness argument

- **Step 2.** WeatherManager.cpp:169 fires on kMirrorDt = 2 sim-s with strict '>' -> 61 frames at 1/30 s; ReadFields (SweSolver.cpp:535-579) = two ReadbackTexture (Gpu.cpp:649-706: committed READBACK + EndUpload -> WaitIdle) + 8.7 M half->float; Query (WeatherManager.cpp:196-241) is the only reader, called at main.cpp:3391/3411/4611/4632/4658, never in the loop. No GPU command that produces a pixel changes. RefreshSweCurrent stays synchronous (its bytes are hashed into the bucket key: P11).
- **Step 4.** Residency.h:371 is std::map; Want finds at Residency.cpp:458 and :472; the 17 wave-plane Wants (main.cpp:4122-4129) run after PROF_END(2) and are the 0.60 ms helm residual (5.025 - 4.423). The slot answers the same question with the same Tracked objects; m_seen's append order and the unstable sort at :757-761 are untouched. Any tenant-group fan-out stays tenant-major.
- **Step 5.** The comb is measured (helm loop 13.8-14.2 ms on frame%3==1 vs 11.9-12.5). Replay in emission order before ++m_frame (Residency.cpp:595); PredictNextPose keeps its call site and cadence.
- **Step 6.** A point of radius <= R + Hmax whose central angle from the eye's nadir exceeds acos(R_occ/r) + acos(R_occ/(R + Hmax)) lies behind the R_occ sphere; the occluder along that sight line is rendered geometry inside the frustum (land >= -430 m NAVD, sea at the geoid, sag cell^2/8R), GREATER depth, no SV_Depth, no blend. Above 10 km today's threshold form is kept, so the node set there is bit-identical (the relief-exact margin would admit exaggerated limb peaks: 'visible', a bug fix on its own). Face 5 (-z) holds the Merrimack camera and is emitted last today.
- **Step 8.** Landness is exactly 0 on open water (Compose.hlsli:361-381; the analog band at :365 is gated on gCsU2.z = 0xFFFFFFFF on the pages path); DXC lowers lerp as x + s*(y - x) so x + (+-0) == x. Never gate the water side on landness == 1 as 'none'.
- **Step 9.** w = 1 - smoothstep(0.12 lam, 0.5 lam, texelM) saturates to exactly 0 for band 2 on rings 2-5 and band 1 on rings 4-5 (lam = 154.1/36.7/7.18 m); skipped consumers are x + (+-0) and max(x, +-0) with dv.w, blocked >= 0; only a zero's sign in an fp16 foam fiber can differ and saturate()/lerp erase it -- compare fiber VALUES. Never extend to the solved loop (its shed needs the fetched a, k).
- **Step 10.** Sorting compact keys with the identical comparator on the identical sequence yields the identical permutation; SKIPPING a sort does not (an unstable sort's tie order depends on the input sequence, and a skipped sort changes the next frame's input) -- the systems plan's 4b is rejected.
- **Step 11.** A fold/box mean is never a bound: hBound comes from the finest data per realization; the audit is on the CULLED side (an under-estimate is a hole that the emit-side audit cannot see).

## 5. Measure-first probes (before the step whose basis a judge doubted)

P1 windowed denominator (the unexplained ~2 ms) -- before any 150 fps claim. P2 PIX at cHelmGap/bird (MS/PS split, overwritten PS invocations, overshading, occupancy) -- before steps 6 GPU half, 8, 11 and the sort. P3 dxc -Fc (DCE, CSE, lerp lowering, coordinate placement) -- before step 8 (b),(h). P4 --mesh-stats with a bearing histogram -- before steps 6 and 11. P5 ProcessQueues by phase on the descent -- before step 10's number. P6 bank tile-list build at the helm -- before step 9's CPU half. P7 cloud floor (slices 0-1 max density in reach) -- before step 12. P8 walk slack histogram + pool sizing -- before step 13 and the deferred motor-slack walk. P9 bank Load- vs ALU-bound, in-window vs out -- before step 9's GPU claim and the deferred caches. P10 FoldFromChildren serve timing, wave deficit drain, the exposure roll frame + tblend -- before steps 16-19. P11 RefreshSweCurrent's hashed frame -- before any async readback variant. P12 PresentMon / GetFrameStatistics on the owner's session -- before any swapchain change. P13 FFT-only async prototype -- before the deferred async-compute item. P14 fractional min-LOD clamp probe -- before the pixel-stage fade extension. P15 --res-trace deficit by mip on the descent -- before the deferred ring-gate-at-Loaded. P16 the helm A/A bisect and --settle-sync -- before 'none' is claimed at the helm.

## 6. The pop-in track (go)

Mechanism, gate and the why-this-time argument are in section 4 rows 15-20 and the structured `popin_track`. In one paragraph: sample the finest RESIDENT rung of the solved pages with all planes in lockstep; keep the box-mean spinor's magnitude as the fold's coherent fraction (a r to geometry, a^2 (1 - r^2) k^2/2 to sigma^2 weighted by the solved share h, Miche-capped), hand the cascades over by the resolved energy fraction f with the 2f(1-f) deficit shed; ramp a CPU-owned effective mip at 1/8 mip per frame in a second map that EXTENDS across consecutive rungs (the age-nibble variants restart per claim and jump ~0.7-0.9 mip at every rung under the ring gate's 1-3-frame cadence: rejected), keep the finer tile mapped across a coarsening, lerp thresholded answers; swap generations instead of Drop (never per-tile Invalidate, never an in-place remap); materialize the served parents; prefill off-thread. Settled frames are today's instruction sequence (fibers |delta| = 0); transitions are judged by the bounded / monotone / conserved series and the raildiff tblend series, never by eye. Not covered: the ring re-anchor jump (visible at settle; owner's call), the pixel-stage LOD step (after P14), frames inside a roll fade (residency-timed). Feasibility 0.7.

## 7. Stop rules

Stop optimizing when --bench-overlap helm loop <= 6.67 ms AND helm GPU <= 6.0 AND helm CPU <= 6.0 AND windowed --no-vsync helm >= 150 fps (P1's denominator) AND presents/refresh >= 0.95 on the owner's session AND p99 <= 12 ms with no comb AND the five stills 'none' vs step 1 with raildiff flat except at the declared roll frames. A failed gate is reverted, the falsified prior goes into ALGEBRA priors with its evidence, the probe re-runs; thresholds are never widened; 'visible' never ships as perf; behaviour changes (lookahead, sort ties, the hashed SWE frame, predict cadence) never ride an exact change; probes under 0.1 ms drop their step; step 13 is skipped if steps 2-11 already clear ~5.5 ms.

## 8. Deferred (kept, not ranked) and rejected

Deferred: the shipped slope plane (visible; owner); the relief-exact horizon margin above 10 km (visible bug fix); the screen-projected CDLOD split (visible; after P2); bed/exposure ring planes cached under exact invalidation and the static tenant-read cache (GPU behind the CPU wall; stale-bed class; after P9); the SWE derive skip on n == 0 (not sub-lsb as argued -- build the exact variant that folds gTideNavd out of derive); ring gate at Loaded / whole-column admission (after P15; never helps wave.field until step 16); the 3-buffer waitable swapchain (after P12; touches every kFrameCount ring); async compute (after P13; priors 19/25 hazards); the motor-slack incremental walk (after P8); skipping the water side on land pixels (sub-lsb; after the --dump-hdr gate exists); the bank fill from orbit (an altitude gate is not exact -- a design decision); a kMaxSubsteps catch-up cap (interactive hitch); the pixel-stage fractional-clamp fade (after P14, driven by the effMap).

Rejected with the reason: half-rate outer rings (fp16 double rounding + a 15 Hz foam alternation on the far band: visible); async RefreshSweCurrent as 'none' (a different frame's bytes can be a different key = a different solve); the sort skip (changes tie order); the age-nibble / arrival-frame fades (restart per rung); Invalidate instead of Drop and in-place retire-as-replaced (absence / unfilled pool memory); kappa(have) (a tuned weight where the computed fraction exists); the footprint Gram cutoff and the wDet == 0 skip in the PS detail loop (not sub-lsb through the glint lobe; drops the group envelope); versor sandwiches for rotation (more ops); hardware bilinear / fp16 / sRGB target / VRS (not bit-identical); predict cadence /6 and the 72-frame lookahead fix inside a perf change (change the predicted stream); sampler feedback replacing the walk (visible, untested on bindless views; a Catalog-pass direction); stateful per-bin FFT rotors (the stateless doctrine); MSAA now; sky drawn last (0.036 ms).

## 9. What the Catalog direction buys later (not integrated in this pass; the renderer lines up with it)

The null-tile climb (CheckAccessFullyMapped, tiletest-proven on this GPU) replaces the residency-map Gathers/Loads and their per-frame uploads once the served parents are materialized on the pinned ladder (step 16 is the first rung of that) -- the pop-in's 'absent' becomes 'ancestor' by construction and the effMap becomes the mapping's own version. One shared coordinate space (authority frames; the wave grid on a rung of the height page's ladder) turns the bank's three Mercator chains into address shifts and the walk's per-leaf Mercator bound into integer tile addresses: step 7's cache IS the tile table. Doubles with integer addresses (ARCHITECTURE.md 6a) is the law the fine meshlets already obey and the PS sample position does not (the camera-relative varying in effects). Exact shader-written feedback can replace the CDLOD Want walk -- the largest CPU item -- provided the screw prefetch is kept as a second, predicted request map and the settled LOD change (one rung sharper where the analytic LOD is fractional) is adjudicated as visible. The residency byte contract is untouched by this plan (the fade lives in a second map), so nothing here needs re-declaring to the store; the second slot per bucketed tenant lives as extra slices of the existing water tenants, inside the resident law.

## 10. Effects when fast

Listed in the structured `effects_when_fast`: the spectral Laplacian channel and the Gram-form caustic fold; the shipped tangent bivector slope plane; the pixel-stage residency fade through the fractional clamp; the analytic ripple tail with the footprint-bivector prefilter; coverage-folded whitecaps; Kelvin wakes and vessel motors; a 7th ring at 0.6 m, level 19, 32 components; physical bucket interpolation and a second solver window as generation B; a Bruneton-class atmosphere and a denser sun-ward cloud march where P7 finds density; terrain/structure shadows and reflections through the existing secant cast; the camera-relative sample position; MSAA only after the split rule stops emitting slivers; determinism (--settle-sync, argv echo, the HDR gate) as the effect that makes every other one measurable.

## Artifact diagnosis (2026-09-05 11:50, the owner's screenshot from an interactive session)

The owner's oblique view up the estuary toward Newburyport (19:30 storm, tide +0.88 m, ebb 0.48 m/s)
showed three things that read as "clipping". Lens stills at the cached bird (1500 m) and 7 km poses,
`--settle-sync`, on the step-5 binary (`out/lens/*.png`: lit, `--albedo`, `--stencil`, `--lens
waterdata`, `--lens ring`, `--lens mip`) pin each one to its producer. None is a regression of this
branch: every step is gated bit-identical at settle against the previous binary.

- **Pale sheets and soft blobs over the water (offshore, and the flooded flats).** Shading, not data:
  the `--albedo` lens shows no trace of them (the composed colour is uniform over the water), the
  ring lens shows they sit in the coarser bank rings (the grey/yellow annuli around the camera's
  ground point), and the waterdata lens shows the breaking triggers firing as broad diagonal bands
  there. That is the crest gate applied to a box-averaged crest at a coarse ring (foamlaw + fold:
  the folding law's own counterexample), so whitecap streaks fuse into sheets at altitude. The fix is
  on the effects list (coverage-folded whitecaps: foam as the expected coverage of the fine answer);
  the pop-in track's consumer changes remove the timing half (which sheets appear depends on when
  the wave pages landed -- the same mechanism as the ebb helm's 6-9 % run-to-run floor).
- **Blocky pale rectangles on the marsh.** At +0.88 m the flats are flooded and the classifier says
  water there (the albedo lens shows the khaki water class over the flats); the rectangles are the
  same foam gate on the coarse ring, quantized by the bed classifier's coarse-mip decision (the
  documented "speckle over flats" residue). Steps 17-18 (consumers read the fold's answer at any
  resident rung; thresholded answers lerp as answers) are the structural fix.
- **Straight seams crossing the water.** Window edges, not mesh clips: the stencil lens draws them
  as thin green polygons -- the z17 detail window (1.2 m/texel; outside it the imagery is the z14
  rung at 9.5 m/texel, which is why the river corridor looks sharper than its surroundings) and the
  solved wave field's z16 window (3.4 x 2.0 km around the entrance, blended over 120 m; beyond it
  the FFT cascades carry the sea). The picture is one colour tenant and one height tenant sampled at
  whatever rung is resident; the water parameters are separate tenants and their borders are where
  a consumer reads absence. Step 19 turns the bucket-roll Drop into a swap with a cross-fade.
- **Found on the way.** The 7 km waterdata lens shows a rectangular hole (no data) inside the solved
  window at settled residency (`out/lens/key7km_waterdata.png`, lower left): a tile that never lands
  or a void marker served as absence. Steps 16-17 (materialized parents, any-rung sampling) would
  cover it with the coarse rung; it deserves its own look with `--res-trace` at that pose.

## Results so far (checkpoint 2026-09-05, PR)

Eight steps attempted, seven accepted, one reverted whole and one re-scoped after its premise was
falsified by its own measurement. Every accepted step is one commit that can be reverted alone; the
message body carries the numbers it was gated on. The run is still going (order below), so this
section is a checkpoint, not a conclusion.

### (a) The steps

| step | commit | status | what changed | measured (the row the step targeted, before -> after) |
|---|---|---|---|---|
| 1 | `34be398` | pass (0 px) | instruments before optimizations: `[boot]` rev+argv, CPU brackets outside and inside RenderFrame, `--bench-overlap`, `gpu.gap`, `--settle-sync`, `--dump-hdr`, `tools/raildiff.py` | the unbracketed helm residual 0.60 ms -> 0.007 ms (sum of the twelve 4.612 vs pre-RenderFrame 4.619); the pipelined loop measured for the first time: 5.02 ms |
| 2 | `1a8e74b` | pass | the dead weather-mirror readback deleted (lazy `Query`) -- two full-field readbacks with a `WaitIdle` drain each, every 2 sim-s, feeding no pixel | weather.Update 0.358/0.374 -> 0.044/0.057 ms (whole/helm); the 61-frame comb 22.4 ms x 19/19 isolated peaks -> 2.8 ms x 0/19; loop p99 27.5 -> 16.0, max 40.5 -> 21.2 |
| 3 | `6adf6fc` | pass | the scene hot-reload `_stat64` off the frame: a thread in `ReadDirectoryChangesW` on the resolved `data/` directory raises one atomic flag | scene hot-reload stat 0.133/0.192 -> 0.000/0.000 ms; pre-RenderFrame total 2.496/4.358 -> 2.369/4.119 |
| 4 | `f1cbe53` | pass | the residency tracked set leaves `std::map`: a per-tenant slot array on the stamp array's layout, a bottom-up column stamp scan, per-tenant Drop lists, one `erase_if` over `m_mapped` | globe.SetView 1.471/3.388 -> 1.074/2.292; PredictWants 0.395/0.699 -> 0.241/0.373; wave-plane Wants 0.371/0.530 -> 0.074/0.082; pre-RenderFrame 2.518/4.789 -> 1.631/2.856; loop 8.75 -> 7.99 |
| 5 | `cfc8e50` | pass | the screw-predicted six-face prefetch walk runs as pure geometry on a worker; the main thread replays its rects through `Want()` in DFS order, so the residency manager sees the identical stream | PredictWants 0.350/0.543 -> 0.167/0.247; pre-RenderFrame 2.177/3.663 -> 1.748/3.099; loop 8.96 -> 8.24; the frame%3 comb 0.915 -> 0.270 whole and 1.449 -> 0.882 helm; predicted stream FNV-1a `3e40fd73c3bf3da6` on the worker, under `--predict-inline` and from the in-place walk |
| 6 | none -- reverted | FAIL (fidelity) | the eye's polar plane as the horizon cull at every altitude, camera face first, discard-free shipped PSO, MsMain per-vertex dedupe | the largest GPU move of the run -- globe.mesh 2.716/4.006 -> 2.071/2.721 (-1.285 ms at the helm) -- but key7km settled is VISIBLE by seven single pixels: the shell is not watertight and those rays were landing on the antipodal surface the cull removes. Reverted whole; `out/regress_step6.mp4` recorded |
| 21 (attempt 1) | none -- reverted | FAIL (premise) | `--settle-hold N` + the churn atlas frozen during a hold, gated at 0 px between two hold lengths | the gate's premise is false: bird rendered TWICE at the SAME hold (drain + 300, both quiet at 56 frames, byte-identical logs) differs by 12.861 % of pixels. The churn freeze reached the GPU (`[gpu]` sea.churn p50 0.178 -> 0.000 ms on held frames) and is simply not the term |
| 21 (attempt 2) | `7879a2b` | pass | re-scoped to its landable parts: `--settle-hold N` counted, the churn frozen for exactly the held frames, raildiff's verdict = continuity + luma with SSIM reported and never gating | proof of wire: `[gpu]` sea.churn mean 0.184 / p50 0.178 ms over 240 unheld frames vs mean 0.101 / p50 0.000 over the 540 of a 240 + 300 render. No saving claimed (fenced RENDER 6.24 / loop 7.72; predicted stream hash unchanged) |
| 25 | `c92ed30` | pass | `--settle-exact`: hold until the resident set IS the walk's want set (stale tiles dropped through the retire path, the pool grown to the want set for the hold), with a per-tenant ledger and an FNV-1a of the mapped set; `--settle-clear-churn` | two `--settle-sync` birds of one binary differ by 0-12 % of the pixels (21.7 % across binaries); under `--settle-exact` the bird is EXACT in 202-216 held frames and two runs are bit-identical (0 px). Globe 0 px in 619-737 frames, key7km 0 px with the churn cleared, the helm EXACT in 243-252 frames where `--settle-sync` capped at 3000 |
| 22 | in flight | running | step 6's three exact sub-mechanisms alone -- face-first walk, discard-free shipped PSO (`--slice` behind a define), MsMain per-vertex `ComposedHeight` dedupe | -- |

Tooling commits alongside them: `df7c911` (`tools/perf/` -- the two workflow scripts and the resume
recipe), `c658a22` (the artifact diagnosis above), `2bf7f4a` (imgdiff `--ignore-rows`), `c5f8cf2`,
`d55d3e9`, `969d754`, `396f7fd`, `b8d0a82`, `a9e474d` (the gate's revisions and the model routing).

### (b) The headline numbers

Baseline = `out/instr_bench.log`, the instrumented but unchanged binary. Latest accepted bench =
step 25's (`out/step25/attempt1/bench.log` and `bench_overlap.log`, rev `7879a2b`-dirty). Storm rail,
`--rail-flood --storm 3.0,10,95 --start 2026-08-28T19:30:00`, 1600x900, 1200 frames.

| quantity | baseline | latest accepted |
|---|---|---|
| `[rail]` RENDER mean (fenced: CPU record + a serializing WaitIdle) | 6.42 ms (155.7 fps) | 6.80 ms (147.1 fps) |
| `[rail]` loop mean (fenced) | 9.48 ms | 8.45 ms |
| outside RenderFrame, whole / helm | 3.067 / 5.025 ms | 1.652 / 2.733 ms |
| GPU whole-frame, whole / helm | 3.804 / 5.964 ms | 3.629 / 5.692 ms |
| GPU globe.mesh, whole / helm | 2.837 / 4.226 ms | 2.703 / 4.015 ms |
| `--bench-overlap` loop mean (the shipped pipelined frame) | not measurable before step 1 | 4.52 ms (5.02 at step 1); `[perf]` 4.10 ms, 244 fps headless |

Every accepted step so far is CPU-side, and the whole saving is in the row that says so: **outside
RenderFrame fell 3.07 -> 1.65 ms on the whole flight and 5.03 -> 2.73 ms at the helm.** The fenced
`[rail]` RENDER mean did not move and was never supposed to -- it brackets record + fence + a
serialized GPU, and none of that is where the work was. The GPU rows moved by 3-5 %; no accepted
step touched a GPU pass, so that is the machine, not the code.

The owner's own windowed measurement (screen on, the steps 1-4 binary, helm pose, `--gpu-time`):

```
[perf] mean frame 7.15 ms over 590 frames (140 fps) [vsync]
[present] \\.\DISPLAY1 ... at 165 Hz: 593 presents shown over 698 refreshes
          = 0.850 presents/refresh (~140.2 fps perceived)
GPU whole-frame 3.93 ms at the helm pose, windowed
```

so the cross-adapter present path is not the cap it was feared to be (the earlier "30 fps" was the
screen-off power state plus the 22 ms mirror stall this run deleted).

**The caveat that governs every number above.** This machine drifts 8-10 % run to run on rows the
code never touched: step 1 measured every GPU-only row 8-10 % *faster* than the baseline log on a
command stream proven identical (globe.mesh helm 4.23 -> 4.00 with the globe still 0 px), and step
6's bench sat ~8 % *slow* across untouched rows (waterbank tile list helm 1.157 -> 1.545). Only a
same-session comparison against the previous accepted binary is like for like, and that is how every
step above was actually gated -- the baseline-to-latest column is orientation, not evidence.

### (c) Found on the way

Defects and facts this run established, none of them introduced by it:

- **The surface shell is not watertight.** Seven single-pixel cracks at the 7 km pose --
  (1572,50) (506,237) (571,246) (377,322) (1362,352) (1384,355) (1525,374) -- have no triangle
  covering them, and the ray through such a crack leaves the shell and lands on the ANTIPODAL
  surface (a -45 deg ray exits at ~90 deg of central angle, which no useful horizon margin retains).
  This is why step 6 failed: culling the far side removes those accidental backdrops. The six
  neighbours of each crack are byte-identical in both builds. Step 23 fixes them, declared visible
  by exactly those pixels; step 24 then re-gates the cull.
- **The settled still was never reproducible.** `--settle-sync`'s quiet test (pending 0, nothing in
  flight, nothing ring-held) is necessary and not sufficient: a request the ring gate holds and the
  walk then stops asking for is neither pending nor resident, so the test fires over a race-decided
  resident set. Two `--settle-sync` renders of ONE binary at the SAME hold length differ by ~13 % of
  the water pixels; a data-lens A/A shows whole tiles resident at different mips. `--settle-exact`
  (step 25) makes the still a function of the pose and the data instead.
- **The lost batch tail at the pool cap.** The bird pose wants 9176 tiles (573 MB) against
  `kPoolCapTiles` 8192 (512 MB). At the cap `MapAndFill`'s evictor finds no victim, breaks out of
  the batch, and the batch's remaining tiles -- already erased from `m_loading` by the gather -- are
  LOST in state Loaded, tracked and unqueued until something invalidates them: 988 `wave.field`
  mip-0 tiles in exactly that state on the first exact run's ledger. Permanent holes at that mip,
  and which ~984 tiles lose is landing order. Step 27 returns the unmappable tail to the loading
  queue and reports the cap; the owner decided the budget with it: `kPoolCapTiles` -> 16384 (1 GB).
  (2720 of the 9176 are the 17 RGBA8 wave planes at mip 0; packing them to 5 stays a later item.)
- **The SWE solver is not reproducible over real frames.** Two `--settle-exact` runs at the ebb helm
  (`--dump-water-state`): level differs on 65 % of cells within 1.5 cm, u/v on 38-47 % within
  3 cm/s; bed and shadow identical. Candidates are the substep-budget drift clamp in
  `SweSolver::Record` dropping steps on a slow frame, or a non-deterministic GPU reduction. Step 26,
  and a prerequisite for every fiber gate and for the pop-in track.
- **The windowed exit crash.** Every windowed run exits `0xC0000409` after printing `done` --
  pre-existing (main's Sep 2 binary does it too), unfixed, and out of this run's scope.
- **The helm's 149 never-draining prefetch requests** (recorded in steps 4 and 5 as "waiting on
  parents no walk asks for") are the predicted walk's FAR HEMISPHERE: step 6's cull drained them,
  and step 25's exact turn drops them as stale pending tiles.
- **The horizon strip, rows 430-432, is the one residual outside residency.** Under `--settle-exact`
  the helm's A/A floor is 65 px (max |d| 15) without the churn cleared and 106 px (max 23) with it,
  ALL on those three rows, with every tenant's mapped-set hash equal -- so no residency tile can be
  named for it. `tools/imgdiff.py --ignore-rows 425:437` takes the verdict off the strip and prints
  the strip's own numbers beside it.
- **`--lens bed` is not a parser token** -- it resolves to lens 1, `worldxz`. A real bed lens is a
  small follow-up; every "bed lens" result quoted in this run is therefore a `worldxz` lens.

### (d) The harness that now exists

| flag / tool | what it does |
|---|---|
| `--gpu-time` | D3D12 timestamp spans per pass and sub-pass, resolved a frame later so nothing stalls; adds the `gpu.gap` row (idle between one frame's closing stamp and the next's opening one) |
| `--no-vsync` | windowed: ALLOW_TEARING swapchain + `Present(0, ALLOW_TEARING)` |
| `--bench-overlap` | `--bench` without the per-frame `WaitIdle`: the shipped pipelined frame, loop = max(CPU, GPU). The `[rail]` header says which bench it is |
| `--settle-sync` | holds the dump instant until residency is quiet (pending 0, no in-flight read, nothing ring-held; cap 3000 held frames, logged loudly). Necessary, not sufficient -- see (c) |
| `--settle-hold N` | holds exactly N frames past the dump instant whatever residency is doing; composes with the others as "drain first, then at least N"; the exit line names the rule that held it |
| `--settle-exact` | holds until the resident set IS the walk's want set, then four more exact turns; prints a per-tenant ledger (wanted, mapped, deficit, unreachable, stale, dropped, re-queued) and an FNV-1a over the mapped set in key order |
| `--settle-clear-churn` | puts the whole churn atlas on the clear kernel's list at the first held frame (count logged) |
| `--dump-hdr` | dumps the pre-tonemap radiance buffer, so a fidelity claim can be made in radiance rather than in bytes |
| `--predict-inline` | runs the prefetch walk synchronously at the old call site -- the A/B for step 5, and the way its stream hash was proven identical |
| `--res-trace` | (existing) the residency trace, plus step 4's slot audit |
| `tools/imgdiff.py` | identical %, max abs d, n(abs d > 1), SSIM and an amplified difference image, with the measured verdict bands none / sub-lsb / noise / VISIBLE. `--pairs A B` over two directories, `--floor A1 A2` prints an A/A pair beside the A/B, `--hdr` compares two `--dump-hdr` dumps in radiance, `--ignore-rows Y0:Y1` takes the verdict off named rows and reports them separately |
| `tools/raildiff.py` | per-frame SSIM/PSNR, per-second YAVG, and the `tblend` difference series over the water rows with new-only spike flagging. Exit 1 = a new-only spike or per-second mean luma past `--yavg-max` (0.5/255); `--ssim-floor` is printed and informational |
| the `[boot]` line | `[boot] gagame rev <sha> \| argv: <flags>` as the first line of every log, so no measurement is orphaned from its build or its recipe |
| the `[present]` line | at exit, windowed: `DXGI_FRAME_STATISTICS` presents over refreshes with the panel's Hz, and a plain statement when DXGI counts nothing (which it does on this cross-adapter swapchain) |

### (e) What is running, and what is pending

The order is **22, 23, 24, 27, 26, then 7-20** ("after step 25" is a position, not a number):

- **22** -- step 6's three exact sub-mechanisms alone: camera face first in both walks, the
  discard-free shipped PSO, the MsMain per-vertex `ComposedHeight` dedupe. Some unknown share of
  step 6's -1.285 ms helm globe.mesh is theirs, and separating them is one A/B. *(running now)*
- **23** -- make the surface shell watertight; declared visible by exactly the crack pixels.
- **24** -- the polar-plane horizon cull at every altitude, re-gated once the shell is watertight.
- **27** -- return the unmappable batch tail to the loading queue at the pool cap, report the cap,
  and raise `kPoolCapTiles` to 16384 (1 GB); declared visible by exactly the holes it fills.
- **26** -- SWE determinism: the solver's state after N sim-seconds must not depend on wall time.
  Prerequisite for every fiber gate and for the pop-in track.
- **7-20** -- then the plan's main sequence: 7 the camera-independent walk cache; 8 the grade-0
  gates and dead taps in Globe's PsMain/MsMain; 9 the exact `CsBankFill` skips and the bank's CPU
  corner build; 10 the residency turn on the main thread (cached DirectStorage handles, compact-key
  sorts, per-slice map uploads); 11 the relief-true frustum bound; 12 the cloud-march floor skip;
  13 the CDLOD walk as geometry jobs; 14 the small exact hygiene; and 15-20, the pop-in track
  (per-tree solve snapshot, materialized wave-pyramid parents, solved pages at any resident rung,
  the fade byte in a second map, generations instead of Drop, the cache read off-thread).

**To resume:** `tools/perf/README.md`. `git log --oneline f4a380f..HEAD` says which steps landed
(one commit per step, "Step N of docs/PERF_EXPERIMENT.md" in the message) and `out/step<N>/` holds
each step's artifacts; the tree must be clean and `build/bin/gagame.exe` must be the last accepted
build. Then relaunch `tools/perf/implement.js` with `startStep` = the first step not yet accepted
and the compact step list from `out/perf/exec_compact.json`. A Workflow `resumeFromRunId` is
same-session only.

### (f) How to test this branch

Build from PowerShell (the worktree needs its `cache/` and `data/` junctions to the main checkout
first -- see the project's worktree run-setup note):

```
cmd.exe /c "<worktree>\build.bat"
build\bin\gagame.exe --selftest                    # 7/7 PASS
```

The storm rail bench, the run's main instrument:

```
build\bin\gagame.exe --sea --one-water --headless --rail-flood out/x --bench --gpu-time ^
    --storm 3.0,10,95 --start 2026-08-28T19:30:00
```

and the same with `--bench-overlap` for the shipped, pipelined loop (the fenced `--bench` number is
CPU record + a serializing WaitIdle and is not the frame you see). The windowed check:

```
build\bin\gagame.exe --sea --one-water --frames 600 --storm 3.0,10,95 ^
    --start 2026-08-28T19:30:00 --campos 120,-10 --cam 7,92.5,-1.5 --gpu-time
```

-- a window opens, closes after 600 frames, and prints `[perf]`, `[gpu]` and `[present]`. Drop
`--frames` for the interactive session at the same pose. (The windowed exit still crashes with
`0xC0000409` after `done`; pre-existing, see (c).)

Two cautions while the run is unattended: in this worktree `cache/` and `data/` are junctions to the
main checkout, and running `gagame` from ANY checkout while the workflow is working perturbs its
gates -- a local run at 10:57-11:00 folded a tree-cache rebuild into every measurement after it and
cost step 5 its bird A/A floor.
