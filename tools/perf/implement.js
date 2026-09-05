export const meta = {
  name: 'gagame-perf-implement',
  description: 'Implement the synthesized plan step by step: build, gate on stills + bench, commit or revert, verify each commit',
  phases: [
    { title: 'Implement', detail: 'one step at a time on one GPU; gate = selftest + 5 stills vs baseline + storm-rail bench with --gpu-time' },
    { title: 'Verify', detail: 'adversarial review of each passing commit from its diff and artifacts' },
    { title: 'Close', detail: 'final bench, storm-rail recording, results document' },
  ],
}

const WT = 'C:/Users/lordc/source/repos/GAGAME/.claude/worktrees/storm-rail-renderer-check-4fb3cb'
const WTW = 'C:\\Users\\lordc\\source\\repos\\GAGAME\\.claude\\worktrees\\storm-rail-renderer-check-4fb3cb'
const PLAN = `${WT}/out/perf/exec.json`
const FINDINGS = `${WT}/out/perf/understand.json`

const LAWS = `
STANDING LAWS OF THIS REPO (violating one fails the step):
- Fidelity vocabulary: 'none' = bit-identical 8-bit output; 'sub-lsb' = max |d| <= 1, n(|d|>1) == 0; 'noise' = within the measured run-to-run floor (identical >= 99.99 %, max |d| <= 6, n(|d|>1) <= 64: the engine itself differs by 3-11 horizon pixels at |d| <= 4 between two identical runs; bird and globe are bit-identical run to run, so a non-zero diff there is a real change); 'VISIBLE' = anything else. tools/imgdiff.py prints the verdict. A step declared 'none'/'sub-lsb'/'noise' must get that verdict (or better) on EVERY still; a step declared 'visible' must be a bug fix the plan argued for, and its stills must be looked at (Read the PNG) and described.
- The fade rule: every LOD / residency / ring / altitude transition fades continuously; a pop is a bug.
- No TAA, no DLSS, no upscaling, no temporal reuse of shading.
- CB layout law: constant-buffer rows append at the END on both the C++ and the HLSL side; the dxtest gate in --selftest compares reflected offsets (a same-size mid insert silently rotates every later row).
- Residency staging law: one writer per staging region per frame; the slab must have room for every writer.
- GPU resident law: one colour tenant, one height tenant, N water-param tenants, every one a sparse GA tree on one lattice; doubles by default on the CPU; never claim alignment unmeasured.
- The fold law: never threshold a box-averaged field; average the thresholded answers.
- Priors 1: SampleLevel on a bindless array returns ZERO outside the pixel stage on this GPU; compute/mesh stages use manual Loads.
- Priors 9/15: judge from >= 200-frame renders; an A/B that differs by 0-3 pixels when a change should show is a severed wire, not a subtle effect: prove the flag/path reached the GPU.
- Rails stay deterministic run to run at the same timing; the settled image must not change unless the step is a declared bug fix.
- The math reference is ${WT}/docs/ALGEBRA.md; cite section ids in code comments the way the codebase does (M-tags, measured numbers, failed attempts recorded).
- Code style: match the repo (C++20, the existing Log() conventions, comments that state the measurement that justified the change). No dead flags, no TODOs.`

const PROCEDURE = `
ENVIRONMENT AND PROCEDURE:
- Worktree: ${WT} (a git worktree; cache/ and data/ are junctions to the main checkout; never use git stash; never touch the main checkout; never push; never merge).
- Build ONLY through the PowerShell tool: cmd /c "\\"${WTW}\\build.bat\\" > \\"${WTW}\\build_log.txt\\" 2>&1" then read build_log.txt (Bash cannot run this build). Fix every warning-as-error.
- Run gagame from the worktree root. NEVER run two gagame processes at once and the GPU is shared with nobody else: before EVERY gagame run poll PowerShell 'Get-Process gagame -ErrorAction SilentlyContinue' and Start-Sleep 10 until none exists.
- Redirect every run's output with cmd /c "... > out\\\\stepN\\\\name.log 2>&1" (PowerShell 2>&1 on a native exe mangles stderr).
- Baseline stills live in ${WT}/out/baseline/ (helm.png, helm_b.png, bird.png, key7km.png, globe.png, helm_ebb.png) rendered with exactly these flags (S = --sea --one-water --headless --storm 3.0,10,95; T = --start 2026-08-28T14:00:00):
    helm:     S T --campos 120,-10 --cam 7,92.5,-1.5 --frames 240 --dump out/stepN/helm.png
    bird:     S T --campos 0,0 --cam 1500,300,-60 --frames 240 --dump out/stepN/bird.png
    key7km:   S T --campos 0,0 --cam 7000,0,-45 --frames 240 --dump out/stepN/key7km.png
    globe:    S T --campos 0,0 --cam 200000,0,0 --frames 240 --dump out/stepN/globe.png
    helm_ebb: S --start 2026-08-28T19:30:00 --campos 120,-10 --cam 7,92.5,-1.5 --frames 240 --dump out/stepN/helm_ebb.png
  Gate them with: py -3 tools/imgdiff.py --pairs out/baseline out/stepN --out-dir out/stepN/diff   (helm_b is compared too and is a free A/A reference).
- Bench: build\\bin\\gagame.exe --sea --one-water --headless --rail-flood out/stepN/bench --bench --gpu-time --storm 3.0,10,95 --start 2026-08-28T19:30:00  > out\\stepN\\bench.log. Report the [rail] RENDER mean/p50/p95/p99/max, the loop mean, the outside-RenderFrame table and the [gpu] per-pass lines, and compare with the PREVIOUS accepted step's bench (path in the plan state below). A step that claims a saving must show it here (mean or p95 of the pass it targets); a CPU-only step must show it in the outside-RenderFrame table.
- Selftest: build\\bin\\gagame.exe --selftest  > out\\stepN\\selftest.log ; all seven gates must PASS.
- On PASS: git add the files you changed (never data/, never out/, never build_log.txt, never docs/GA_AST.md or docs/ga_ast.json unless the step changed the AST on purpose) and commit on the current branch with a message in the repo's voice (one sentence of what and why, then the measured numbers), ending with the trailer line 'Co-Authored-By: Claude Fable 5.1 <noreply@anthropic.com>'. Report the sha.
- On FAIL: if a still came back VISIBLE and the step was not declared visible, FIRST record the regression: build\\bin\\gagame.exe --sea --one-water --headless --rail-flood out/stepN/regress --mp4 out/regress_stepN.mp4 --storm 3.0,10,95 --start 2026-08-28T19:30:00 > out\\stepN\\regress.log (about 70 s); then revert ONLY the files you touched (git checkout -- <files>; delete files you created), rebuild so the tree is back to the last accepted state, and report status 'fail' with the reason and the video path. If the build cannot be fixed within a reasonable effort, revert the same way.
- A 'measure-first' step runs its probe and returns 'skipped' with the measured answer when the probe says no gain; do not implement on a hunch. The plan's measure_first list (P1..P16) is in the plan file; a step that names a probe runs that probe FIRST and records its answer in notes.
- THE GATE SET IS FIXED: the five stills above with exactly those flags against out/baseline (plus helm_b as the A/A reference). The plan text suggests other poses (cBird, --globe-cam, --rail-key); those are NOT the gate. Step 1 may add instruments, but must not change which stills gate the later steps. If step 1 adds --settle-sync and it makes the helm pair reach 0 px, later steps may use it for the helm still ONLY by rendering a new helm baseline with the same binary that renders the candidate (an A/A pair), never by comparing across binaries.
- The implement-side order is the plan's order. If a step's prerequisites (its 'dependencies' or the plan's notes) were reverted or skipped earlier in this run, say so in notes and either adapt (if the step stands alone) or return 'skipped'.
- Budget your time: a step is one focused change. Do not widen scope; note follow-ups in 'notes'.`

const PROCEDURE_V2 = `
ENVIRONMENT AND PROCEDURE (gate v2, after step 1 measured the engine's run-to-run floors):
- Worktree: ${WT} (a git worktree; cache/ and data/ are junctions to the main checkout; never use git stash; never touch the main checkout; never push; never merge). Every log's first line names the rev and argv (step 1).
- Build ONLY through the PowerShell tool: cmd /c "\\"${WTW}\\build.bat\\" > \\"${WTW}\\build_log.txt\\" 2>&1" then read build_log.txt (Bash cannot run this build). Fix every warning-as-error.
- Run gagame from the worktree root. NEVER run two gagame processes at once: before EVERY gagame run poll PowerShell 'Get-Process gagame -ErrorAction SilentlyContinue' and Start-Sleep 10 until none exists. Redirect every run with cmd /c "... > out\\\\stepN\\\\name.log 2>&1".
- FIRST ACTION OF THE STEP, before any edit or build: copy build\\bin\\gagame.exe to out\\prev_exe\\gagame.exe and write 'git rev-parse --short HEAD' into out\\prev_exe\\rev.txt -- the binary on disk at step start is the last ACCEPTED build (a failed step rebuilds it after reverting). out/prev_exe/gagame.exe is 'the previous binary' below; run it exactly like build\\bin\\gagame.exe (same working directory, same flags).
- The five stills, exactly these flags (S = --sea --one-water --headless --storm 3.0,10,95; T = --start 2026-08-28T14:00:00), rendered into out/stepN/:
    helm:     S T --campos 120,-10 --cam 7,92.5,-1.5 --frames 240 --dump out/stepN/helm.png
    bird:     S T --campos 0,0 --cam 1500,300,-60 --frames 240 --dump out/stepN/bird.png
    key7km:   S T --campos 0,0 --cam 7000,0,-45 --frames 240 --dump out/stepN/key7km.png
    globe:    S T --campos 0,0 --cam 200000,0,0 --frames 240 --dump out/stepN/globe.png
    helm_ebb: S --start 2026-08-28T19:30:00 --campos 120,-10 --cam 7,92.5,-1.5 --frames 240 --dump out/stepN/helm_ebb.png
  Compare: py -3 tools/imgdiff.py --pairs out/baseline out/stepN --out-dir out/stepN/diff --json out/stepN/imgdiff.json
- THE MEASURED FLOORS (step 1, ~30 stills on two binaries): bird and globe are bit-identical run to run (a first run right after a build once landed a different tile count and differed by 3 %: an environment perturbation, re-render once). helm at 14:00 differs 5-18 px run to run, or by ONE far-water tile at (1348,444) (~870 px, max 147) that lands by frame 240 in about half the runs. key7km differs 115-18.7k px between two runs of ONE binary (a tile grid of mip levels). helm_ebb differs 6-9 % of its pixels between two runs (crest-shaped foam blobs: the churn atlas remembers when the wave pages landed). These floors are facts about the engine, not permissions: a step still may not change the settled picture.
- THE STILL GATE, MECHANICAL (no narrative adjudication):
    bird, globe: 'none' (0 px) vs out/baseline required. If not 'none', re-render that still once; if still not 'none', the step FAILS on fidelity.
    helm: 'noise' or better vs out/baseline passes. Otherwise the same-session 2x2 protocol: render helm a second time with the candidate (helm_2.png) and twice with the previous binary (helm_prev1.png, helm_prev2.png), same flags, all four in this session; the still passes if the BEST of the four cross pairs (candidate_i vs prev_j) is 'noise' or better. Report all four cross lines and the two A/A lines (candidate pair, prev pair).
    key7km, helm_ebb: run the 2x2 protocol whenever the verdict vs out/baseline is VISIBLE. The still passes only if BOTH hold: (a) the best cross pair's n(|d|>1) <= 1.5 x max(n(|d|>1) of the prev A/A pair, of the candidate A/A pair) + 64, and its max|d| <= max(A/A max|d|) + 8; (b) you Read the amplified diff PNG of the best cross pair AND of the prev A/A pair and they show the same KIND of pattern (a tile grid of mips; scattered foam blobs). A coherent change over the whole water, a moved coastline or jetty, a changed sky, a shifted horizon, a brightness change over an area is a REGRESSION whatever the counts say. Write what you saw in still_verdicts[].line.
    A step declared 'visible' in the plan (none are) would need its own argument; every step in this plan is 'none' at settle.
- For any change touching the bank kernel, the wave pages, exposure, the bed reads or a shader the stills exercise (steps 8, 9, 12, 14, 17, 18, 19 at least): ALSO --dump-fibers at the helm pose (add --settle-sync, from step 1) with the candidate and with the previous binary, and compare the fiber_detail/dispy .f32 numerically (numpy; |delta| = 0 required, zero signs ignored) unless the step's spec says which texels may differ and why.
- Bench: build\\bin\\gagame.exe --sea --one-water --headless --rail-flood out/stepN/bench --bench --gpu-time --storm 3.0,10,95 --start 2026-08-28T19:30:00 > out\\stepN\\bench.log, and the same with --bench-overlap into out/stepN/bench_overlap (the shipped, pipelined loop; added in step 1). Report [rail] RENDER mean/p50/p95/p99/max, loop mean, the outside-RenderFrame tables (both), the [gpu] per-pass lines, and compare with the previous accepted step's out/stepM/bench.log and bench_overlap.log. The machine drifts 8-10 % run to run on rows the code did not touch: a claimed saving must show on the specific row the step targets (a CPU step in its bracket; a GPU step in its pass) and, when the delta is within that drift, confirm it by running the previous binary's bench once in the same session and comparing row for row.
- Continuity (the fade rule): record the storm rail with the candidate: build\\bin\\gagame.exe --sea --one-water --headless --rail-flood out/stepN/rail --mp4 out/stepN/rail.mp4 --storm 3.0,10,95 --start 2026-08-28T19:30:00 > out\\stepN\\rail.log, then py -3 tools/raildiff.py <previous accepted step's rail.mp4> out/stepN/rail.mp4 --out-dir out/stepN/raildiff --stills 3 (exit 0 = no new-only spikes and the SSIM floor holds; a landing-timing shift is not a spike). The previous accepted rail is out/step1/rail.mp4 until a later step passes. For the pop-in track steps this series IS the primary gate (bounded, monotone, conserved through every landing) next to the settled fibers.
- Selftest: build\\bin\\gagame.exe --selftest > out\\stepN\\selftest.log ; all seven gates must PASS.
- On PASS: git add the files you changed (never data/, never out/, never build_log.txt, never docs/GA_AST.md or docs/ga_ast.json unless the step changed the AST on purpose) and commit on the current branch with a message in the repo's voice (one sentence of what and why, then the measured numbers; a 'Step N of docs/PERF_EXPERIMENT.md' line), ending with 'Co-Authored-By: Claude Fable 5.1 <noreply@anthropic.com>'. Report the sha.
- On FAIL: if a still or the raildiff failed as a regression, keep the recording as the evidence: copy out/stepN/rail.mp4 to out/regress_stepN.mp4 (record it first if it does not exist yet). Then revert ONLY the files you touched (git checkout -- <files>; delete files you created), rebuild so build\\bin\\gagame.exe is the last accepted build again, and report status 'fail' with the reason and the video path. If the build cannot be fixed within reasonable effort, revert the same way.
- A 'measure-first' step runs its probe first and returns 'skipped' with the measured answer when the probe says no gain; the plan's P1..P16 are in the plan file. If a step's prerequisites were reverted or skipped earlier in this run, say so and either adapt (if the step stands alone) or return 'skipped'.
- THE GATE SET IS FIXED: these five stills, these flags, out/baseline as the reference (plus the previous binary for the floors). The plan's other poses (cBird, --globe-cam, --rail-key) are not the gate.
- Scope and time: one step is one focused change; do not widen it; note follow-ups in 'notes'. Aim to finish a step, gate included, in about 90 minutes of wall time: if it is running long, cut to the core mechanism the plan names, gate that, and leave the rest as a follow-up. Never leave the tree dirty at the end.`

const PROCEDURE_V3 = `
ENVIRONMENT AND PROCEDURE (gate v3: v2 plus what steps 1-2 measured -- the strict stills are gated SETTLED):
- Worktree: ${WT} (a git worktree; cache/ and data/ are junctions to the main checkout; never use git stash; never touch the main checkout; never push; never merge). Every log's first line names the rev and argv.
- Build ONLY through the PowerShell tool, with this exact form (the escaped-quote form fails in PowerShell): $wt = "${WTW}"; cmd /c "\`"$wt\\build.bat\`" > \`"$wt\\build_log.txt\`" 2>&1"; Get-Content "$wt\\build_log.txt" -Tail 5   -- then read build_log.txt; it must end with 'built:'. Fix every warning-as-error.
- Run gagame from the worktree root. NEVER run two gagame processes at once: before EVERY gagame run poll PowerShell 'Get-Process gagame -ErrorAction SilentlyContinue' and Start-Sleep 10 until none exists. Redirect every run with cmd /c "... > out\\\\stepN\\\\name.log 2>&1".
- FIRST ACTION OF THE STEP, before any edit or build: copy build\\bin\\gagame.exe AND build\\bin\\*.dll (dstorage.dll, dstoragecore.dll, dxcompiler.dll, dxil.dll) into out\\prev_exe\\ and write 'git rev-parse --short HEAD' into out\\prev_exe\\rev.txt -- the binary on disk at step start is the last ACCEPTED build (a failed step rebuilds it after reverting). out/prev_exe/gagame.exe is 'the previous binary' below; run it exactly like build\\bin\\gagame.exe (same working directory, same flags).
- The five stills, exactly these flags (S = --sea --one-water --headless --storm 3.0,10,95; T = --start 2026-08-28T14:00:00), rendered into out/stepN/:
    helm:     S T --campos 120,-10 --cam 7,92.5,-1.5 --frames 240 --dump out/stepN/helm.png
    bird:     S T --campos 0,0 --cam 1500,300,-60 --frames 240 --settle-sync --dump out/stepN/bird.png
    key7km:   S T --campos 0,0 --cam 7000,0,-45 --frames 240 --settle-sync --dump out/stepN/key7km.png
    globe:    S T --campos 0,0 --cam 200000,0,0 --frames 240 --settle-sync --dump out/stepN/globe.png
    helm_ebb: S --start 2026-08-28T19:30:00 --campos 120,-10 --cam 7,92.5,-1.5 --frames 240 --dump out/stepN/helm_ebb.png
  --settle-sync (step 1) holds the dump instant until residency is quiet (pending 0, no in-flight read; cap 3000 held frames, it logs loudly if the cap hits). Step 2 measured why: at frame 240 the globe still has 180-380 tiles pending on any binary; the removed stalls had been accidental sync barriers for the loaders, so from now on every CPU saving shifts landing times. The SETTLED picture is the law (priors 9); settled cross-binary renders were bit-identical (0 px) in step 2.
  Compare against out/baseline for the record: py -3 tools/imgdiff.py --pairs out/baseline out/stepN --out-dir out/stepN/diff --json out/stepN/imgdiff.json (out/baseline is UNSETTLED frame-240 data for bird/globe/key7km, so a VISIBLE there is expected and is not the gate for those three).
- THE STILL GATE, MECHANICAL (no narrative adjudication):
    bird, globe, key7km (settled): render the same still with the previous binary in the same session (bird_prev.png etc., same flags including --settle-sync); candidate vs previous must be 'none' (0 px). If not, re-render both once; if still not 'none', the step FAILS on fidelity (a settled cross-binary difference is a real change). If --settle-sync hits its 3000-frame cap on a still (it logs it), fall back to the v2 2x2 protocol for that still and say so.
    helm: 'noise' or better vs out/baseline passes. Otherwise the same-session 2x2 protocol: render helm a second time with the candidate (helm_2.png) and twice with the previous binary (helm_prev1.png, helm_prev2.png); the still passes if the BEST of the four cross pairs is 'noise' or better. Report all four cross lines and the two A/A lines.
    helm_ebb: run the 2x2 protocol whenever the verdict vs out/baseline is VISIBLE. Passes only if BOTH hold: (a) the best cross pair's n(|d|>1) <= 1.5 x max(n(|d|>1) of the prev A/A pair, of the candidate A/A pair) + 64, and its max|d| <= max(A/A max|d|) + 8; (b) you Read the amplified diff PNG of the best cross pair AND of the prev A/A pair and they show the same KIND of pattern (scattered crest-shaped foam blobs, the horizon strips). A coherent change over the whole water, a moved coastline or jetty, a changed sky, a shifted horizon, a brightness change over an area is a REGRESSION whatever the counts say. Write what you saw in still_verdicts[].line.
    Every step in this plan is 'none' at settle; a step that needs another argument is a fail.
- For any change touching the bank kernel, the wave pages, exposure, the bed reads or a shader the stills exercise (steps 8, 9, 12, 14, 17, 18, 19 at least): ALSO --dump-fibers at the helm pose with --settle-sync, candidate and previous binary, and compare fiber_detail/fiber_dispy .f32 numerically (numpy; |delta| = 0 required, zero signs ignored) unless the step's spec says which texels may differ and why.
- Bench: build\\bin\\gagame.exe --sea --one-water --headless --rail-flood out/stepN/bench --bench --gpu-time --storm 3.0,10,95 --start 2026-08-28T19:30:00 > out\\stepN\\bench.log, and the same with --bench-overlap into out/stepN/bench_overlap (the shipped, pipelined loop). Report [rail] RENDER mean/p50/p95/p99/max, loop mean, the outside-RenderFrame tables (both), the inside-RenderFrame CPU table, the [gpu] per-pass lines, and compare with the previous accepted step's out/stepM/bench.log and bench_overlap.log. The machine drifts 8-10 % run to run on rows the code did not touch: a claimed saving must show on the specific row the step targets (a CPU step in its bracket; a GPU step in its pass) and, when the delta is within that drift, confirm it by running the previous binary's bench once in the same session (out/stepN/bench_prev.log) and comparing row for row.
- Continuity (the fade rule): record the storm rail with the candidate LIKE FOR LIKE with the reference recording: build\\bin\\gagame.exe --sea --one-water --headless --rail-flood out/stepN/rail --mp4 out/stepN/rail.mp4 --storm 3.0,10,95 --start 2026-08-28T14:00:00 --tile-budget 3000 > out\\stepN\\rail.log, then py -3 tools/raildiff.py <previous accepted step's rail.mp4> out/stepN/rail.mp4 --out-dir out/stepN/raildiff --stills 3 (exit 0 = no new-only spikes and the SSIM floor holds; a landing-timing shift is not a spike). The previous accepted rail is out/step2/rail.mp4 (step 1's is out/step1/rail.mp4), then the last passing step's. For the pop-in track steps this series IS the primary gate (bounded, monotone, conserved through every landing) next to the settled fibers; those steps also record the 19:30 ebb rail for the same diff.
- Selftest: build\\bin\\gagame.exe --selftest > out\\stepN\\selftest.log ; all seven gates must PASS.
- Known events, not regressions: frames ~723 and ~732 of the storm rail carry 15-20 ms pre-RenderFrame events on every binary (a bucket / cache event on the descent; step 20's territory). --dump-water-state overwrites the TRACKED ws_*.f32 + ws_meta.json at the repo root: if you run it, run it from a scratch cwd or restore them with git checkout -- afterwards; never commit them.
- On PASS: git add the files you changed (never data/, never out/, never build_log.txt, never the ws_* exports, never docs/GA_AST.md or docs/ga_ast.json unless the step changed the AST on purpose) and commit on the current branch with a message in the repo's voice (one sentence of what and why, then the measured numbers; a 'Step N of docs/PERF_EXPERIMENT.md' line), ending with 'Co-Authored-By: Claude Fable 5.1 <noreply@anthropic.com>'. Report the sha.
- On FAIL: if a still or the raildiff failed as a regression, keep the recording as the evidence: copy out/stepN/rail.mp4 to out/regress_stepN.mp4 (record it first if it does not exist yet). Then revert ONLY the files you touched (git checkout -- <files>; delete files you created), rebuild so build\\bin\\gagame.exe is the last accepted build again, and report status 'fail' with the reason and the video path. If the build cannot be fixed within reasonable effort, revert the same way.
- A 'measure-first' step runs its probe first and returns 'skipped' with the measured answer when the probe says no gain; the plan's P1..P16 are in the plan file. If a step's prerequisites were reverted or skipped earlier in this run, say so and either adapt (if the step stands alone) or return 'skipped'.
- THE GATE SET IS FIXED: these five stills, these flags, the previous binary in the same session as the reference for the settled ones and out/baseline for the helm. The plan's other poses (cBird, --globe-cam, --rail-key) are not the gate.
- Scope and time: one step is one focused change; do not widen it; note follow-ups in 'notes'. Aim to finish a step, gate included, in about 90 minutes of wall time: if it is running long, cut to the core mechanism the plan names, gate that, and leave the rest as a follow-up. Never leave the tree dirty at the end.`

const PROCEDURE_V4 = PROCEDURE_V3
  .replace(`    helm:     S T --campos 120,-10 --cam 7,92.5,-1.5 --frames 240 --dump out/stepN/helm.png`,
           `    helm:     S T --campos 120,-10 --cam 7,92.5,-1.5 --frames 240 --settle-sync --dump out/stepN/helm.png`)
  .replace(`    helm: 'noise' or better vs out/baseline passes. Otherwise the same-session 2x2 protocol: render helm a second time with the candidate (helm_2.png) and twice with the previous binary (helm_prev1.png, helm_prev2.png); the still passes if the BEST of the four cross pairs is 'noise' or better. Report all four cross lines and the two A/A lines.`,
           `    helm (settled, from step 5): render with --settle-sync on both binaries (helm.png vs helm_prev.png). At the helm the hold usually hits its 3000-frame cap because the predicted walk leaves ~149 requests that never drain (step 4 measured both binaries capping the same way and the settled pair agreeing to 8 px, max 4): that is acceptable when both logs show the cap. Candidate vs previous settled must be 'noise' or better (the helm keeps a 3-11 px horizon residue even settled). If VISIBLE, re-render both once; if still VISIBLE, the step FAILS on fidelity. Step 4 measured why the unsettled frame-240 helm can no longer gate: a loop 2 ms/frame faster lands the far field one ring later at frame 240 (11.7k px on the horizon strip) while the settled picture is unchanged -- so the frame-240 helm vs out/baseline is recorded for the record only, never as the verdict.`)
if (PROCEDURE_V4 === PROCEDURE_V3) throw new Error('PROCEDURE_V4 replacements did not apply')

const RESULT_SCHEMA = {
  type: 'object',
  properties: {
    step: { type: 'number' },
    name: { type: 'string' },
    status: { type: 'string', enum: ['pass', 'fail', 'skipped'] },
    commit: { type: 'string' },
    files: { type: 'array', items: { type: 'string' } },
    still_verdicts: { type: 'array', items: { type: 'object', properties: { still: { type: 'string' }, verdict: { type: 'string' }, line: { type: 'string' } }, required: ['still', 'verdict', 'line'] } },
    bench_before: { type: 'string' },
    bench_after: { type: 'string' },
    gpu_pass_deltas: { type: 'array', items: { type: 'string' } },
    outside_render_ms: { type: 'string' },
    selftest: { type: 'string' },
    regression_video: { type: 'string' },
    what_changed: { type: 'string' },
    notes: { type: 'string' },
  },
  required: ['step', 'name', 'status', 'commit', 'files', 'still_verdicts', 'bench_before', 'bench_after', 'gpu_pass_deltas', 'outside_render_ms', 'selftest', 'regression_video', 'what_changed', 'notes'],
}

const VERIFY_SCHEMA = {
  type: 'object',
  properties: {
    step: { type: 'number' },
    verdict: { type: 'string', enum: ['approve', 'revert', 'approve-with-followup'] },
    reasons: { type: 'array', items: { type: 'string' } },
    law_violations: { type: 'array', items: { type: 'string' } },
    followups: { type: 'array', items: { type: 'string' } },
  },
  required: ['step', 'verdict', 'reasons', 'law_violations', 'followups'],
}

const plan = args && args.plan ? args.plan : null
if (!plan || !plan.execution_order) return { error: 'pass the exec plan as args.plan' }

const state = { lastBench: `${WT}/out/instr_bench/bench.log`, results: [], verifications: [] }
if (args.lastBench) state.lastBench = args.lastBench

const startStep = args.startStep || 1
if (args.priorResults) state.results.push(...args.priorResults)

phase('Implement')
for (const step of plan.execution_order) {
  const n = step.step
  if (n < startStep) continue
  log(`step ${n}: ${step.name}`)
  const res = await agent(`You are implementing ONE step of a performance plan in the GAGAME engine (D3D12, C++20, HLSL). Read the plan file ${PLAN} (the whole plan, so you know what comes before and after), the findings file ${FINDINGS} for the evidence behind this step (search it for the file names the step cites), and the relevant docs/ALGEBRA.md sections, then the cited code.
${LAWS}
${n === 1 ? PROCEDURE : n === 2 ? PROCEDURE_V2 : n <= 4 ? PROCEDURE_V3 : PROCEDURE_V4}

THIS STEP: number ${n}, '${step.name}', declared fidelity class '${step.fidelity_class}', effort ${step.effort}. Its FULL specification (mechanism, files_lines, expected_ms_saved, gate, ga_angle, notes, dependencies) is the entry with "step": ${n} in execution_order inside ${PLAN} -- Read that file and follow that entry exactly; the plan's measure_first probes, popin_track, stop_rules are in the same file.
PREVIOUS ACCEPTED BENCH LOG: ${state.lastBench}
PREVIOUS RESULTS THIS RUN (for context): ${JSON.stringify(state.results.map(r => ({ step: r.step, name: r.name, status: r.status, commit: r.commit, bench_after: r.bench_after })))}
Output directory for this step: ${WT}/out/step${n}/ (create it).

Do the work: implement exactly this step's mechanism, build, selftest, the five stills + imgdiff, the bench with --gpu-time, decide pass/fail against the step's gate, commit or revert as the procedure says, and return the result through the schema. Read the diff images (Read the PNG) when a verdict is not 'none' and say in notes where the pixels moved.`,
    { label: `implement:${n}:${step.name.slice(0, 40)}`, phase: 'Implement', schema: RESULT_SCHEMA, effort: 'max' })
  if (!res) {
    // A null here is an agent that died on a terminal API error (a usage limit, most likely), not a
    // failed gate. Marking the step failed and marching on would burn every later step the same way,
    // so stop and leave the tree for a resume (same session: resumeFromRunId; new session: relaunch
    // with args.startStep = this step and args.priorResults = the results so far).
    log(`step ${n}: the agent returned nothing (usage limit or API error) -- stopping the run here; resume from step ${n}`)
    state.results.push({ step: n, name: step.name, status: 'fail', commit: '', notes: 'agent returned nothing; run stopped here for a resume' })
    state.stoppedAt = n
    break
  }
  state.results.push(res)
  if (res.status === 'pass' && res.commit) {
    state.lastBench = `${WT}/out/step${n}/bench.log`
    const v = await agent(`You are the adversarial verifier for one committed step of a performance plan in the GAGAME engine (worktree ${WT}; read-only: do NOT edit, build, or run gagame). Read 'git -C "${WT}" show ${res.commit}' in full, the step's artifacts in ${WT}/out/step${n}/ (imgdiff verdicts, bench.log, selftest.log, the diff PNGs via Read), the previous bench ${state.results.length > 1 ? state.results[state.results.length - 2].bench_after : 'see notes'}, and the ALGEBRA.md sections the change touches.
${LAWS}
STEP: number ${n}, '${step.name}', declared class '${step.fidelity_class}' -- its full specification is the execution_order entry with "step": ${n} in ${PLAN} (Read it).
IMPLEMENTER'S REPORT: ${JSON.stringify(res)}
Try to REFUTE the commit: is the change really exact where it claims to be (float reassociation, sample-order changes, dropped terms multiplied by a value that is only NEARLY zero)? Does a gate that passed actually ask the question (were the stills rendered with the baseline's exact flags; is the bench comparison like for like; could the saving be noise: compare against the p95 spread)? Any law violated? Any new pop or discontinuity introduced (read the code paths around residency/LOD transitions)? Does the code comment state the measurement? Return approve / revert / approve-with-followup with reasons; 'revert' only for a real defect or a broken gate, not style.${n === 1 ? '' : `
${n === 2 ? `GATE V2 (in force from step 2): bird and globe must be 'none' (0 px) vs out/baseline; helm passes at 'noise' or through the same-session 2x2 protocol against out/prev_exe/gagame.exe (best cross pair 'noise' or better); key7km and helm_ebb pass only through the 2x2 protocol with the count rule (best cross pair n(|d|>1) <= 1.5 x max A/A + 64, max|d| <= max A/A + 8) AND a described same-kind diff pattern; shader/bank steps also need --dump-fibers |delta| = 0 against the previous binary; the storm-rail raildiff against the previous accepted rail must be clean. Confirm from the artifacts in out/step${n}/ (imgdiff.json, the diff PNGs, the A/A lines, raildiff/series.csv summary) that the implementer applied these rules mechanically and did not pass a still by narrative. If the artifacts do not exist for a rule that applied, the gate was not asked: 'revert'.` : `GATE V3 (in force from step 3): bird, globe and key7km are rendered with --settle-sync and must be 'none' (0 px) against the previous binary's settled render in the same session (out/prev_exe/gagame.exe; the *_prev.png files in out/step${n}/); helm passes at 'noise' vs out/baseline or through the same-session 2x2 protocol (best cross pair 'noise' or better); helm_ebb passes only through the 2x2 protocol with the count rule (best cross pair n(|d|>1) <= 1.5 x max A/A + 64, max|d| <= max A/A + 8) AND a described same-kind diff pattern; shader/bank steps also need --dump-fibers |delta| = 0 against the previous binary under --settle-sync; the storm-rail raildiff (14:00, --tile-budget 3000, like for like) against the previous accepted rail must be clean; the bench must show the saving on the row the step targets. Confirm from the artifacts in out/step${n}/ (the *_prev renders, imgdiff.json, the diff PNGs, the A/A lines, raildiff/series.csv summary, bench logs) that the implementer applied these rules mechanically and did not pass a still by narrative. If the artifacts do not exist for a rule that applied, the gate was not asked: 'revert'.${n === 4 ? `
ORCHESTRATOR'S RULING FOR STEP 4: the helm rule is met by the SETTLED pair -- out/step4/helm_settled.png vs helm_settled_prev.png (both --settle-sync, both binaries capping the hold the same way) must be 'noise' or better; the implementer reports 8 px max 4. The unsettled frame-240 2x2 (11.7k px on the far horizon strip) is landing timing from a loop 2 ms/frame faster and is informational; do not revert on it. Verify the settled pair from the artifacts and the rest of the gate as written.` : n >= 5 ? `
GATE V4 (from step 5): the helm is also gated SETTLED (helm.png vs helm_prev.png, both --settle-sync, 'noise' or better; both logs may show the 3000-frame cap); the unsettled helm vs out/baseline is informational only.` : ''}`}`}`,
      { label: `verify:${n}`, phase: 'Verify', schema: VERIFY_SCHEMA, effort: 'high' })
    state.verifications.push(v || { step: n, verdict: 'approve-with-followup', reasons: ['verifier returned nothing'], law_violations: [], followups: [] })
    if (v && v.verdict === 'revert') {
      log(`step ${n} reverted by the verifier: ${v.reasons.join('; ')}`)
      const r = await agent(`Revert one commit in the GAGAME worktree ${WT}: run 'git -C "${WT}" revert --no-edit ${res.commit}', then rebuild with the PowerShell tool: cmd /c "\\"${WTW}\\build.bat\\" > \\"${WTW}\\build_log.txt\\" 2>&1" and confirm build_log.txt ends with 'built:'. Then run build\\bin\\gagame.exe --selftest (poll 'Get-Process gagame' first so no other instance runs) and confirm all gates PASS. Reason for the revert: ${v.reasons.join('; ')}. Return the new HEAD sha and the selftest line as plain text.`,
        { label: `revert:${n}`, phase: 'Verify', effort: 'low' })
      state.results[state.results.length - 1].status = 'fail'
      state.results[state.results.length - 1].notes += ' | REVERTED by verifier: ' + v.reasons.join('; ') + ' | ' + (r || '')
      state.lastBench = state.results.length > 1 ? (state.results.slice(0, -1).reverse().find(x => x.status === 'pass') || {}).bench_after || state.lastBench : state.lastBench
    }
  }
  log(`step ${n} ${res.status}${res.commit ? ' ' + res.commit.slice(0, 8) : ''}: ${res.bench_after}`)
}

if (state.stoppedAt) return { results: state.results, verifications: state.verifications, closing: null, stoppedAt: state.stoppedAt }

phase('Close')
const closing = await agent(`You are closing a performance experiment in the GAGAME engine worktree ${WT}. Read the plan ${PLAN} and these results: ${JSON.stringify(state.results)} and verifications: ${JSON.stringify(state.verifications)}.
${PROCEDURE}
Do, in order (one gagame process at a time, poll Get-Process gagame first):
1. Final measurements on the current HEAD: the storm-rail bench with --gpu-time into out/final/bench.log; the like-for-like storm rail recording: build\\bin\\gagame.exe --sea --one-water --headless --rail-flood out/final/storm_rail --mp4 out/final/rail_storm.mp4 --storm 3.0,10,95 --start 2026-08-28T14:00:00 --tile-budget 3000 > out\\final\\rec.log; then ffmpeg SSIM against the pre-experiment recording: ffmpeg -hide_banner -loglevel error -i out/final/rail_storm.mp4 -i out/rail_storm_1400.mp4 -lavfi "[0:v][1:v]ssim=stats_file=out/final/ssim.log" -f null - and summarize per-second SSIM (mean, min, where); the windowed runs the instrument used (600 frames at the helm and at the globe, with and without --no-vsync) into out/final/windowed_*.log and quote their [perf] lines.
2. ${WT}/docs/PERF_EXPERIMENT.md already holds the baseline table and the plan (committed before the run). APPEND a '## Results' section: one row per step (name, status, commit, what changed, fidelity verdicts, before/after numbers), the final table next to the baseline table, the windowed numbers, the pop-in track outcome (with the landing-series numbers), what was reverted and why (with regression video paths), what remains (the deferred list), and what the Catalog direction buys next. Keep the repo's documentary voice: measured numbers, failed attempts recorded.
3. If any step falsified a prior (a textbook expectation measured wrong), append a numbered entry to the priors ledger in docs/ALGEBRA.md in the existing format, and add any new flags to docs/LAUNCH.md launch-verify. Then run 'dotnet run --project tools/Scriptorium -- --index' from the worktree root (Bash is fine) so the Scriptorium serves the updates.
4. Commit the docs (same trailer). Return as plain text: the final [rail] and [gpu] lines, the windowed [perf] lines, the SSIM summary, the list of commits on this branch since ${args.baseSha || 'the start'} (git log --oneline), and the path of PERF_EXPERIMENT.md.`,
  { label: 'close', phase: 'Close', effort: 'high' })

return { results: state.results, verifications: state.verifications, closing }
