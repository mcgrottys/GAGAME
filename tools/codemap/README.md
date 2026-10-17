# codemap

A map of the code that is regenerated after every build, laid out like the code, so a question
costs the few lines that answer it. Open `build/codemap/INDEX.md` first; ask with
`py -3 tools/codemap/q.py` (or the Scriptorium's `codemap` tool, same command line).

## What it is made of

| layer | from | what it gives |
|---|---|---|
| C++ symbols, calls, overrides | **clangd** 23.1 (`clangd-indexer --format=binary`, one per TU, read by `riff.py`) | every symbol; every reference with the function it sits in; base-of / overridden-by |
| what clangd leaves out | libclang (`tools/astindex`) | calls to functions seen only in their own .cpp (anonymous namespace, `static`: clangd keeps the symbol, drops refs to it) and the functions of local structs. Measured 2026-10-17: clangd 38,404 call sites, libclang adds 3,388 |
| conditions, values, scene keys | libclang (`ast_regions`, `ast_values`, `ast_litcalls`) | the condition every line runs under; every value a field or local may take; `Schema::Bind("key", p.field)` |
| shaders | dxc `-ast-dump` + the C++ `Compile(path, L"Entry")` calls | HLSL calls; each entry linked to the function that compiles it (85 of 85) |
| a scene | `gagame.exe <scene> --print-scene` | the folded document (base, includes, defaults) |

One node id for both C++ indexers: clangd's SymbolID is the first 8 bytes of SHA-1 over the USR,
the USR libclang reports (12,828 of 13,356 libclang declarations land on a clangd symbol; the
rest are members of function-local structs, which become nodes of their own).

## Scene -> code paths (the analysis)

Abstract interpretation, sound by construction (`graph.py`):

- A **value** is a set of constants or unknown. A field's set is the union of everything that may
  be written to it: its default, every assignment (through reference locals too:
  `auto& mode = m_mode; mode = 1;`), its scene key's value. A write that cannot be modelled
  (`x += e`, an out-parameter, `&x` taken, a non-const member called on it, a braced init of its
  record) makes it unknown.
- A **condition** is decided three ways. A call is cut only when a condition holding it is
  DEFINITELY false, so whatever runs for the scene is inside what is reached.
- **Reachability**, **live classes** (rapid type analysis: a virtual call reaches an override only
  in a class some reached code constructs or names; a live class's constructors and destructor
  run) and **which writes count** (only those in reached code, under conditions not decided
  false) are one fixpoint. Each round only cuts more, each cut justified by the round before.
- The scene's own lookups (`S.Tool("x")`, `S.LayerOn("x")`, `Portal`, `Entity`, `Effect`, ...)
  are decided from the folded file; `ParseArgs`'s writes count only with a flag (a scene runs
  without flags).

Not modelled (left unknown, never a wrong constant): values through pointers or references to a
field, parameters, return values (but the scene lookups and `std::string::empty/size`), a local
aliased by a pointer. HLSL branches are not evaluated (an entry is reached or not).

"Collapsing paths": cycles collapse to one node (Tarjan); `dominators.txt` is the dominator tree
(Cooper-Harvey-Kennedy) from `main` and from `FrameLoop::Frame` -- every path to a function passes
its parent there, so a subtree is what its root owns; `architecture.html` orders the directories
by a weighted feedback arc set (Eades-Lin-Smyth) so the fewest calls point upward.

## Files

`run.py` the post-build pipeline (astindex -> clangd -> map; one at a time, re-runs if a build
lands mid-run; skips the map when nothing it reads changed) · `clangd_index.py` + `riff.py`
clangd · `graph.py` the graph and the analysis · `build.py` writes `build/codemap/` · `q.py`
questions · `rag.py` full-text (SQLite FTS5, BM25; no model, no network) · `viz.py` the picture.

`riff.py` is checked against clangd's own YAML of one TU: `py -3 tools/codemap/riff.py x.idx x.yaml`
(32,588 symbols, 275,844 refs, 600 relations, 0 differences).

## Setup and numbers

clangd: the release zips `clangd-windows-23.1.0.zip` + `clangd_indexing_tools-windows-23.1.0.zip`
from github.com/clangd/clangd unpacked to `%USERPROFILE%\tools\clangd\23.1.0` (or set
`GA_CLANGD_DIR`). NOT under `%LOCALAPPDATA%`: a packaged app sees that folder virtualized and a
plain console would not find it. Without clangd the run logs one line and the map uses libclang's
calls alone.

Measured 2026-10-17 on this machine (131 TUs, 8 processes): libclang full 75 s, clangd full 107 s,
one .cpp touched: 4 s + 6 s; the map 17 s (graph 1 s, 35 scenes folded and evaluated 11 s, files
3 s). The build itself returns at once (6.8 s for a one-file change, the pipeline after it in 31 s).
Output: 361 files, 9 MB without the clangd shards (58 MB, `build/codemap/clangd`). Byte-identical
across runs and across Python hash seeds.
