# tools/perf -- the orchestration of docs/PERF_EXPERIMENT.md

Two Claude Code Workflow scripts (plain JavaScript run by the `Workflow` tool; each `agent()`
is a subagent) that produced and executed the plan in `docs/PERF_EXPERIMENT.md`. They are kept
here so a later session, on any model, can continue the run instead of re-deriving it.

| file | what |
|---|---|
| `design-panel.js` | five designers (GA mathematician, GPU architect, streaming engineer, fade specialist, systems engineer), three adversarial judges, one synthesizer; reads `out/perf/understand.json`, writes nothing (the session saved its return value as `out/perf/design.json` and `out/perf/exec.json`) |
| `implement.js` | one step at a time: implement, build, gate (selftest, five stills, `--dump-fibers` where a shader/bank changed, storm-rail bench with `--gpu-time`, raildiff), commit or revert, then an adversarial verifier per commit; a closing agent appends the results to `docs/PERF_EXPERIMENT.md` |

The findings (`out/perf/understand.json`, digest `out/perf/digest.txt`) and the plan
(`out/perf/exec.json`, its markdown in `docs/PERF_EXPERIMENT.md`) live in the worktree's
gitignored `out/`; the plan is what `implement.js` reads (`PLAN` at the top of the script).

## The gate, as it ended up (v4; the history is in the script)

Steps 1-4 measured the engine's run-to-run floors: bird / globe / 7 km key are bit-identical
run to run once residency is SETTLED (`--settle-sync`, step 1), the helm keeps a 3-11 px horizon
residue even settled, and the ebb helm differs by 6-9 % between two runs of one binary (the churn
atlas remembers when the wave pages landed). So a step is judged against the PREVIOUS ACCEPTED
BINARY in the same session (`out/prev_exe/`, copied at step start), settled stills 0 px, the helm
'noise', the ebb helm through a 2x2 A/A protocol with a count rule and a read diff image; never
against a frame-240 snapshot, which every CPU saving shifts.

## Resuming

Inside the session that launched it: `Workflow({scriptPath, resumeFromRunId})` replays every
unchanged `agent()` from cache and runs the rest live.

From a NEW session (the cache is session-local):

```
Workflow({
  scriptPath: '<repo>/tools/perf/implement.js',
  args: {
    baseSha: 'f4a380f',
    lastBench: '<worktree>/out/step<last accepted>/bench.log',
    startStep: <first step not yet accepted>,
    priorResults: [ { step, name, status, commit, bench_after } ... ]   // optional, for the doc
    plan: { execution_order: [ { step, name, fidelity_class, effort, from_plans } ... ] }  // from out/perf/exec_compact.json
  }
})
```

Before relaunching: `git log --oneline f4a380f..HEAD` says which steps landed (one commit per
step, 'Step N of docs/PERF_EXPERIMENT.md' in the message), `out/step<N>/` holds each step's
artifacts, the tree must be clean and `build/bin/gagame.exe` must be the last accepted build
(rebuild HEAD if in doubt). The script stops itself when an agent dies on a usage limit
(`stoppedAt` in its return value) instead of marking later steps failed.

The worktree needs the junctions from the project memory note (`cache/` and `data/` to the main
checkout) and a build via `build.bat` from PowerShell.
