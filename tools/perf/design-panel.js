export const meta = {
  name: 'gagame-perf-design',
  description: 'Judge panel: four optimization plans from distinct lenses, three adversarial judges, one synthesized execution plan',
  phases: [
    { title: 'Propose', detail: 'four designers, distinct lenses, max effort' },
    { title: 'Judge', detail: 'three judges score every item, adversarially' },
    { title: 'Synthesize', detail: 'one ordered execution plan with gates' },
  ],
}

const WT = 'C:/Users/lordc/source/repos/GAGAME/.claude/worktrees/storm-rail-renderer-check-4fb3cb'
const FINDINGS = `${WT}/out/perf/understand.json`
const SAMPLE = 'D:/tiledresources (2)/tiledresources/sourceCode/tiledresources'
const SGA = 'C:/Users/lordc/source/repos/SparseGeoAlgPlatform'

const CONTEXT = `
PROJECT: GAGAME, a real-time global ocean simulator on a tiled geometric-algebra atlas (D3D12, C++20, HLSL), worktree ${WT}. The owner is a Clifford/CGA/PGA mathematician who took up graphics so his algebra gets applied; GA is load-bearing, not decoration. His brief, verbatim in spirit:
- Improve performance with every NON smoke-and-mirrors technique (no TAA, no DLSS, no upscaling, no temporal reconstruction). Target 150+ fps at 1600x900 on the RTX 5060 Laptop; then effects.
- Fidelity must not drop. Vocabulary: 'none' = bit-identical 8-bit output; 'sub-lsb' = max |d| <= 1; 'noise' = within the MEASURED run-to-run floor of the engine (runs of one build, and a build that changed no shader, differ in 3-11 horizon pixels at the helm, max |d| 4, bird and globe bit-identical: identical >= 99.99 %, n(|d|>1) <= 64, max |d| <= 6); 'visible' = anything else, forbidden unless it is a bug fix argued on its own.
- The fade rule: every LOD / residency / ring / altitude transition must fade continuously; anything that pops is a bug. The known offender: waves and whitecaps appear only near the inlet and pop with the mips. He has tried before without success; a proposal there must say why it would work THIS time or admit it would not.
- The two references he named as the standard to beat: the Microsoft Tiled Resources sample (${SAMPLE}) and his new data store SparseGeoAlgPlatform (${SGA}, docs/ARCHITECTURE.md). The Catalog is NOT to be integrated in this pass, but the renderer's direction must line up with it: Cayley-closure-structured sparse resources (3D tiled textures for volumes, 2D texture arrays for rasters, buffers for scalars), null-tile 'no data' hits as a first-class trick, double vertex coordinates on the CPU, one implicit shared coordinate space.
- Standing laws in this repo (violating one is disqualifying): the GPU resident law (one colour tenant, one height tenant, N water-param tenants, all Sparse GA trees on one lattice, doubles by default, never claim alignment unmeasured); the five-stage data pipeline (GA Load -> normalization -> Compose -> physics -> DirectX sparse GA structure); the CB layout law (rows append at the END on both sides, the dxtest gate compares reflected offsets); the fold law (never threshold a box average, average the answers); priors 9/15 (judge from >= 200-frame renders; an A/B that differs by 0-3 pixels is a severed wire, not a subtle effect); the residency staging law (one writer per staging region per frame).
- The math reference is ${WT}/docs/ALGEBRA.md (sections pga, cl3, cl2, fft, fold, physics, swe, radiometry, frames, compose, discrete, wavefield, caustics, ripple, foamlaw, wake, bedalbedo, optics, priors, verification). Cite section ids.
- Measured baseline at this build (storm rail --bench, 1600x900, GPU fenced per frame): RenderFrame mean 6.69 ms (p95 10.69, max 17.79 at frame 632), loop mean 9.95 ms, outside-RenderFrame 3.27 ms (helm phase 5.56 ms: globe.SetView 3.44, PredictWants 0.70, weather.Update 0.48, hot-reload stat 0.20; waveField.Update averages 3.23 ms because a few frames carry the whole re-solve). Windowed mode presents with vsync; the owner perceives ~30 fps there.
- THE FINDINGS FILE: ${FINDINGS} holds the Understand phase's output as JSON: 'anatomy' (the frame's pass list and sync points), 'timers' (per-pass GPU timestamps and the windowed measurements from the instrument agent, if it succeeded), and 'readers' keyed by subsystem (globe-shading, water-bank, residency, cpu-loop, shader-static, ga-lens, harness, tiledresources-sample, sga-platform), each with facts, cost_drivers and candidates. Read it first with the Read tool (it is large; read it fully), then read the code it cites before you commit to a number.
- HEADLINE FINDINGS from the Understand phase (all cited in the findings file; verify before relying):
  (1) A DEAD per-frame cost owns the p99: WeatherManager::RefreshMirror (src/sim/WeatherManager.cpp:169, kMirrorDt = 2 sim-s = every 61 frames) does two synchronous GPU readbacks (~28 MB, fresh committed READBACK resource each, EndUpload -> WaitIdle) plus 8.7 M half->float conversions; the mirror it refreshes is read only by Query(), which nothing in the frame loop calls. Measured: +16..27 ms loop spikes every 61 frames in every metrics.csv.
  (2) Windowed pacing: Present(1,0) on a 2-buffer FLIP_DISCARD chain, no waitable object, no tearing flag, no output enumeration (src/core/Gpu.cpp:144-158, 186-214). Any frame over 16.7 ms drops the flip to 30 fps; (1) guarantees such frames. Cross-adapter present on the hybrid laptop is unconfirmed until the instrument's adapter report.
  (3) THE POP-IN, pinned: the bank kernel accepts the solved wave pages ONLY at mip 0 (shaders/WaterBank.hlsl:130-133, WavePageResident), wWin snaps 0 -> smoothstep(0, 120 m) per 224 m tile (:353), cascades 0-1 are scaled by (1 - wWin) (:403) and the solved sum enters at :467; the 17 component planes and the envelope plane are separate faces that land separately (:135, :435, :464), so the sea flattens, refills component by component, and the foam gates (:502-514) flip when the envelope tile lands. Exposure: nothing resident reads as expo 1.0 (:320-334) and every 5 deg / 0.25 m bucket roll Drop()s the tenant (src/main.cpp:4284); the bed snaps to an integer mip (shaders/HeightPages.hlsli:88). Earlier work (M6t/M8, M9bc/M9bd) made the RINGS energy-conserving and cross-faded and the loads fast; the CONSUMERS still switch answers instantly on residency events. That is why the previous attempts did not remove the pop.
  (4) Bit-exact shader wins found by reading (ranked by the readers): skip the land side where landness == 0 (4 ComposedHeight + ComposedColor + mask on every water pixel, Globe.hlsl:466-788) and, sub-lsb, the water side where landness == 1; fetch the survey-mask fiber once instead of 3-4 times per pixel and per mesh vertex (Compose.hlsli CsMaskSample via ComposedLandMask/CsEditMask/ComposedEditLand); hpC = hp when lod >= -5; reuse ComposedColor(up) as the bed albedo when the secant cast did not run; FoamBreakup only where foamW > 0; GlobeMesh: one ComposedHeight per vertex when vl == vlP; CsBankFill: skip the cascade Loads where the fold weight w == 0 (a third of the cascade loads on rings 2-5: bands whose grade-1 content the fold already shed); skip SWE derive/velgrad/mip rebuild on zero-substep frames (7 of 8 frames); draw the sky/haze dome LAST with GREATER_EQUAL depth instead of first under everything; polar-plane horizon cull at every altitude (today disabled below 10 km, so far-side leaves are emitted, rasterized and shaded at the helm) and front-to-back meshlet emission for early-Z; de-duplicate BedAt in CsSweFlux; cache the per-node camera-independent walk data; replace std::map in Want() with a hash map; hoist horizon constants; scene hot-reload stat on a cadence or watcher (0.13-0.23 ms of syscall per frame through the data/ junction).
  (5) Bigger levers that are not bit-exact and need a measurement first: the CDLOD split rule dist < 3*arc is view-independent, so at the helm ~1.3 M triangles rasterize into a ~90-row sea band as sub-pixel slivers (quad overshading up to 4x on ~10 % of the screen); the cloud march (14 steps, 2 transcendentals + 1-2 Texture3D taps per step) runs on every surface pixel with 989/1024 volume tiles resident; the static/dynamic split of CsBankFill (only the cascade textures and the per-component rotor change between frames; bed, exposure, level, current, fold weights, gains, the solved a/k/spinor are stationary until residency, tide or an SWE substep changes them).
  (6) Honest GA verdicts from the GA reader: rotor advance of stored spinors is already exploited everywhere a phase is stored; the footprint Gram form does NOT give an exact early-out in Globe's detail loop (its isotropic wPix gate already reaches 0 first, and the Cox-Munk lobe makes the tail non-sub-LSB); versor sandwiches are not fewer ops than the expanded kernels. Where GA IS cheaper it changes what is STORED: the fold weight as control flow, the bank's tangent bivector shipped as a slope plane (one BankSample instead of three; changes output, so 'visible' unless proven otherwise), the analytic Laplacian in the reserved u0.w channel for the caustics (ALGEBRA caustics says this is the correct form; 12 FD taps today), the polar plane of the eye as the horizon, landness as the grade-0 gate of the pixel stage.
  (7) The engine is not bit-deterministic run to run (11 horizon pixels, max |d| 3 at the helm) and the [rail] RENDER number in captured runs is CPU record time only; --bench folds GPU execution in via a per-frame WaitIdle.
  (8) MEASURED WITH THE NEW INSTRUMENT (commit 9e43985 on this branch: --gpu-time timestamp spans, --no-vsync, boot adapter report; logs in ${WT}/out/instr_bench.log, out/instr_bench/gpu_ms.csv, out/instr_win_*.log; the structured report is 'timers' in the findings file). Storm rail --bench --gpu-time, 1600x900, GPU ms per pass, whole flight mean / helm-phase mean: whole frame 3.80 / 5.96; globe.mesh (the DispatchMesh surface: mesh shader + Globe.hlsl PsMain) 2.84 / 4.23; waterbank.fill 0.70 / 1.20; sea total 0.50 (sea.swe 0.27 mean with one 5.06 ms spin-up spike, sea.fft 0.09, sea.churn 0.13); globe.sky 0.025, sky 0.036, tonemap 0.022, globe.residency 0.04, markers/gis 0. [rail] RENDER mean 6.60 ms = ~2.6 ms CPU record + fence + 3.8 ms GPU; loop mean 9.62 ms; outside RenderFrame 3.02 ms whole / ~5.5 ms at the helm. Windowed 1600x900 client on the 2048x1280 panel: helm 114 fps with Present(1,0) vs 116 fps with tearing; globe 159 vs 266 fps. The present is confirmed CROSS-ADAPTER (the RTX 5060 owns 0 outputs; the panel is on the Radeon 780M) and Present(1,0) did NOT hold the loop to a refresh cadence on this session. CONCLUSION TO DESIGN AGAINST: at the helm the frame is CPU-bound (~2.3 ms record + ~5.5 ms sim/residency/walk against ~6.0 ms of GPU); 150 fps whole-loop needs the CPU side under ~6.6 ms AND the GPU under ~6.6 ms, and headroom for effects needs globe.mesh well under 3 ms at the helm. Away from the helm the GPU is 3-4 ms and the CPU ~5 ms: the walk and the record are the limit everywhere.
- Tools: Bash (grep/sed/cat) and Read for code; do NOT edit files, do NOT build, do NOT run gagame.exe (the GPU may be busy). Your final text IS the return value; return only through the schema.`

const ITEM = {
  type: 'object',
  properties: {
    name: { type: 'string' },
    mechanism: { type: 'string', description: 'what changes, precisely, in words a reviewer can check' },
    files_lines: { type: 'array', items: { type: 'string' } },
    expected_ms_saved: { type: 'string', description: 'per frame, at which pose; say which measurement it rests on' },
    basis: { type: 'string', description: 'the measured fact(s) from the findings that justify the estimate' },
    fidelity_class: { type: 'string', enum: ['none', 'sub-lsb', 'noise', 'visible'] },
    exactness_argument: { type: 'string' },
    fade_rule: { type: 'string', description: 'how transitions stay continuous, or n/a' },
    ga_angle: { type: 'string', description: 'the algebraic object doing the work, or "none, plain engineering"' },
    effort: { type: 'string', enum: ['S', 'M', 'L'] },
    verification: { type: 'string', description: 'the exact gate: which stills/bench, what result proves it' },
    dependencies: { type: 'array', items: { type: 'string' } },
    risk: { type: 'string' },
  },
  required: ['name', 'mechanism', 'files_lines', 'expected_ms_saved', 'basis', 'fidelity_class', 'exactness_argument', 'fade_rule', 'ga_angle', 'effort', 'verification', 'dependencies', 'risk'],
}

const PLAN_SCHEMA = {
  type: 'object',
  properties: {
    lens: { type: 'string' },
    thesis: { type: 'string', description: 'where the frame time really goes and the one idea that unlocks most of it' },
    items: { type: 'array', items: ITEM, description: 'ranked, best first, at most 12' },
    popin_proposal: { type: 'object', properties: {
      mechanism: { type: 'string' }, settled_frame_exactness: { type: 'string' }, why_previous_attempts_failed: { type: 'string' },
      why_this_time: { type: 'string' }, feasibility: { type: 'number' }, files_lines: { type: 'array', items: { type: 'string' } } },
      required: ['mechanism', 'settled_frame_exactness', 'why_previous_attempts_failed', 'why_this_time', 'feasibility', 'files_lines'] },
    measure_first: { type: 'array', items: { type: 'string' }, description: 'hypotheses that need a probe before any code' },
    rejected: { type: 'array', items: { type: 'object', properties: { idea: { type: 'string' }, why: { type: 'string' } }, required: ['idea', 'why'] } },
    effects_when_fast: { type: 'array', items: { type: 'string' }, description: 'what to spend the headroom on past 150 fps, in this engine' },
  },
  required: ['lens', 'thesis', 'items', 'popin_proposal', 'measure_first', 'rejected', 'effects_when_fast'],
}

const LENSES = [
  { key: 'ga-mathematician', brief: `LENS: the GA mathematician. Every item must be an algebraic object already in ALGEBRA.md doing real work: the footprint bivector fpx^fpz and its Gram form as an analytic wavenumber cutoff (spectral loops end where w < 2^-10, exact at 8 bits), the three-tier telescope (wDet == 0 skips), rotor advance of phase spinors instead of trig, the tangent bivector giving normal + Jacobian in one object, the eye's polar plane against the sphere (PGA/CGA) as horizon cull and LOD oracle, motor-relative rendering for precision, grade-banked evaluation so cheap grades gate expensive ones. Derive each in a few lines and bound its error in LSB. Be honest where GA is not cheaper. Also say which of these the Catalog direction (doubles, one shared coordinate space, null-tile hits) makes easier later.` },
  { key: 'gpu-architect', brief: `LENS: the GPU architect. Use the per-pass timestamp table in the findings (or, if the instrument failed, the static shader review) to attack the largest passes: overdraw and draw order (early-Z, sky last under a far-depth test, water only over the water mask), barrier and sync structure, occupancy and register pressure, bandwidth (formats, plane counts, sample counts per pixel), redundant per-pixel work hoisted to per-vertex or per-draw, compute dispatch merging, async compute overlap, the present path (vsync, flip model, cross-adapter on the hybrid laptop, frames in flight). Variable-rate shading is admissible ONLY with a proof that the shaded signal is band-limited below the coarse rate in that region (e.g. the footprint prefilter says every band has w ~ 0) -- otherwise it is a fidelity loss and you must reject it yourself. No temporal reuse of any kind.` },
  { key: 'streaming-engineer', brief: `LENS: the residency and streaming engineer, with the Microsoft Tiled Resources sample as the standard to beat (read ${SAMPLE}/ResidencyManager.cpp, SamplingRenderer.cpp, SamplingRenderer.ps.hlsl, TerrainRenderer.Tier2.ps.hlsl yourself). Attack the CPU demand walk (1099 nodes, 653 leaves, 22k Want() touches per frame, 1.5-3.4 ms), the residency map path, the per-frame tile budget and upload path, the number of tenants and whether one lattice can be shared, the wave-field re-solve stall, and how a GPU-derived demand (a low-resolution feedback pass or DX12 sampler feedback) would replace or shrink the walk. Residency TIMING may change if the settled image is identical; say how each item affects the pop-in (better, worse, neutral) and keep the staging law.` },
  { key: 'fade-specialist', brief: `LENS: the water physicist who owns the fade rule. Your plan is the pop-in track and the water compute graph, nothing else. Read shaders/WaterBank.hlsl in full (the kernel, WavePageResident at 130-135, wWin at 353, the cascade scaling at 403, the solved sum at 455-470, the foam gates at 500-515, the exposure read at 320-334), shaders/HeightPages.hlsli, src/scene/WaterBankLayer.cpp, src/sim/WaveField.cpp (the bucket key and the Drop/re-key at src/main.cpp:4270-4290), src/core/Residency.cpp (residency map bytes, what a tile landing changes and when), ALGEBRA.md fold / wavefield / foamlaw / cl2 / priors, and GAMEPLAN.md entries M6g, M6t, M7g, M8 plus docs/SPARSE_GA.md 1300-1340 and 1960-1990. Design the fade so that (a) the settled frame is unchanged (fidelity 'none' or 'noise' once every tile has landed), (b) every residency event at every consumer (wave pages at coarse mips, component planes in lockstep, the envelope derived from the components when its plane is absent, exposure swap-not-drop on bucket roll, the bed's integer-mip snap) is continuous in time and in mip, (c) energy is conserved through the fade by the fold law (what leaves geometry enters sigma^2), and (d) it costs less than it saves or nearly so. Give the exact gate: a per-frame difference series across a residency landing that must be bounded and monotone, and the settled-still gate. Say precisely why each previous attempt (M6g one-world handoff, M6t grade shedding, M7g residency refinement, M9bc/M9bd prefilled pyramid) left this pop in place and why your mechanism is different in kind. Then add the water-graph performance items (static/dynamic split of CsBankFill, w == 0 cascade skip, zero-substep SWE skip, half-rate outer rings with the rotor's Lie-algebra step) ranked and gated.` },
  { key: 'systems-engineer', brief: `LENS: the CPU/systems engineer. Attack everything outside RenderFrame and everything that stalls: the per-frame filesystem stat for hot-reload, weather.Update, SetView/PredictWants, the wave-field solver on the main thread (move to a worker with a double-buffered result and a rotor-exact handoff), sea.SetTime, the title formatting, allocation churn, locks, and the windowed present (vsync interval, ALLOW_TEARING, cross-adapter copy on the hybrid laptop, DXGI frame latency waitable object). Preserve rail determinism (rails must stay bit-identical run to run at the same timing, and the settled image must not change). Give the threading hazards you see in the code, with lines.` },
]

const JUDGE_SCHEMA = {
  type: 'object',
  properties: {
    judge: { type: 'string' },
    scores: { type: 'array', items: { type: 'object', properties: {
      plan: { type: 'string' }, item: { type: 'string' },
      gain_credibility: { type: 'number' }, fidelity_safety: { type: 'number' }, fade_compliance: { type: 'number' },
      architecture_consistency: { type: 'number' }, verification_strength: { type: 'number' },
      verdict: { type: 'string', enum: ['do', 'measure-first', 'defer', 'reject'] }, reason: { type: 'string' } },
      required: ['plan', 'item', 'gain_credibility', 'fidelity_safety', 'fade_compliance', 'architecture_consistency', 'verification_strength', 'verdict', 'reason'] } },
    contradictions: { type: 'array', items: { type: 'string' }, description: 'where two plans disagree on a fact; say which is right and why (cite code)' },
    popin_verdicts: { type: 'array', items: { type: 'object', properties: { plan: { type: 'string' }, verdict: { type: 'string' }, reason: { type: 'string' } }, required: ['plan', 'verdict', 'reason'] } },
    missing: { type: 'array', items: { type: 'string' }, description: 'obvious wins no plan proposed, with evidence' },
  },
  required: ['judge', 'scores', 'contradictions', 'popin_verdicts', 'missing'],
}

const JUDGES = [
  { key: 'skeptic', brief: `JUDGE LENS: the skeptic. For every item, try to REFUTE the expected gain: is the pass it targets actually on the critical path per the timestamp table? Is the estimate grounded in a measured number or in a feeling? Would the saving be hidden behind another stall? Would the change break rail determinism? Default to 'measure-first' when the basis is not a measurement. Score gain_credibility hard (0-5).` },
  { key: 'purist', brief: `JUDGE LENS: the fidelity and physics purist. Check every exactness argument against ALGEBRA.md: does the fold law survive (no thresholded box averages), does energy stay conserved across any new fade (statistics handback), is a claimed 'none' really bit-identical (float reassociation, exp2/log2 substitutions, different sample order are NOT), does any item quietly lose Fresnel/glint/foam detail the owner would see at the helm? Score fidelity_safety and fade_compliance hard (0-5). Reject anything TAA/DLSS/upscaling-shaped.` },
  { key: 'engineer', brief: `JUDGE LENS: the engineering-risk judge who knows this codebase's laws: the CB layout law (dxtest), the residency staging law (one writer per staging region per frame), the GPU resident law (one colour, one height, N water tenants on one lattice, doubles), the five-stage pipeline, the bindless-sampler trap (SampleLevel on bindless arrays returns zero outside the pixel stage), threading hazards in the residency manager, the DirectStorage landing-slot retirement rule. Score architecture_consistency and verification_strength (0-5) and call out any item whose verification could pass while the change is wrong (a gate that has not been asked the question).` },
]

const EXEC_SCHEMA = {
  type: 'object',
  properties: {
    thesis: { type: 'string' },
    execution_order: { type: 'array', items: { type: 'object', properties: {
      step: { type: 'number' }, name: { type: 'string' }, from_plans: { type: 'array', items: { type: 'string' } },
      mechanism: { type: 'string' }, files_lines: { type: 'array', items: { type: 'string' } },
      expected_ms_saved: { type: 'string' }, fidelity_class: { type: 'string' }, gate: { type: 'string' },
      ga_angle: { type: 'string' }, effort: { type: 'string' }, notes: { type: 'string' } },
      required: ['step', 'name', 'from_plans', 'mechanism', 'files_lines', 'expected_ms_saved', 'fidelity_class', 'gate', 'ga_angle', 'effort', 'notes'] } },
    measure_first: { type: 'array', items: { type: 'object', properties: { hypothesis: { type: 'string' }, probe: { type: 'string' }, decides: { type: 'string' } }, required: ['hypothesis', 'probe', 'decides'] } },
    popin_track: { type: 'object', properties: { go: { type: 'boolean' }, mechanism: { type: 'string' }, gate: { type: 'string' }, reason: { type: 'string' } }, required: ['go', 'mechanism', 'gate', 'reason'] },
    stop_rules: { type: 'array', items: { type: 'string' } },
    effects_when_fast: { type: 'array', items: { type: 'string' } },
    plan_markdown: { type: 'string', description: 'the whole plan as a markdown document for docs/PERF_EXPERIMENT.md: thesis, the measured baseline, the ordered steps with gates, the measure-first probes, the pop-in track, the stop rules, and what the Catalog direction buys later' },
  },
  required: ['thesis', 'execution_order', 'measure_first', 'popin_track', 'stop_rules', 'effects_when_fast', 'plan_markdown'],
}

phase('Propose')
log('Four designers read the findings and the code, each through one lens')
const plans = (await parallel(LENSES.map(l => () =>
  agent(`${CONTEXT}\n\n${l.brief}\n\nDELIVER a ranked plan (at most 12 items, best first) through the schema. Every expected_ms_saved must name the measured number it rests on; if there is none, put the item under measure_first instead. Include a popin_proposal (or feasibility 0 with the honest reason), rejected ideas with why, and effects_when_fast.`,
    { label: 'design:' + l.key, phase: 'Propose', schema: PLAN_SCHEMA, effort: 'max' })
    .then(p => p ? { key: l.key, plan: p } : null)
))).filter(Boolean)
log(`${plans.length} of ${LENSES.length} plans returned`)
if (!plans.length) return { error: 'no plans' }

phase('Judge')
const plansText = JSON.stringify(plans)
const judgements = (await parallel(JUDGES.map(j => () =>
  agent(`${CONTEXT}\n\n${j.brief}\n\nTHE FOUR PLANS (JSON):\n${plansText}\n\nScore EVERY item of EVERY plan (plan = the designer key, item = the item name verbatim). Read the code an item cites before scoring it. Then list contradictions between plans (decide who is right, cite code), give a verdict on each popin_proposal, and list obvious wins no plan proposed.`,
    { label: 'judge:' + j.key, phase: 'Judge', schema: JUDGE_SCHEMA, effort: 'max' })
    .then(r => r ? { key: j.key, judgement: r } : null)
))).filter(Boolean)
log(`${judgements.length} of ${JUDGES.length} judgements returned`)

phase('Synthesize')
const exec = await agent(`${CONTEXT}

You are the synthesizer. Below are four plans and the judges' scores. Merge them into ONE execution plan the implementation phase will follow step by step. Rules:
- Order by (credible ms saved / effort), but put measure-first probes before any step whose basis a judge doubted, and put instruments and harness fixes first.
- Drop any item a majority of judges rejected; keep 'defer' items in a deferred list inside notes.
- Each step gets a GATE: the exact commands (the standard stills at helm / bird / key7km / globe / helm_ebb with 240 frames, tools/imgdiff.py verdict required: none or sub-lsb or noise unless the step is a declared visible bug fix; the storm rail --bench before/after with --gpu-time; --selftest) and the number that proves the saving.
- The pop-in track: go/no-go with the mechanism the judges rated best and the gate that shows a fade (frame-by-frame difference across a residency event must be bounded and monotone; the settled frame unchanged).
- Stop rules: when to stop optimizing and start effects (150 fps whole loop at the helm in --bench? windowed with --no-vsync?), and what to do if a step fails its gate (revert, record in the priors ledger if a prior was falsified).
- effects_when_fast: concrete, in this engine, non-smoke-and-mirrors.
- plan_markdown: the full plan as a document.

PLANS:
${plansText}

JUDGEMENTS:
${JSON.stringify(judgements)}`,
  { label: 'synthesize', phase: 'Synthesize', schema: EXEC_SCHEMA, effort: 'max' })

return { plans, judgements, exec }
