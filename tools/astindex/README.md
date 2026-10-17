# astindex

The compiler's view of the code, for the Scriptorium. The Scriptorium's own indexer is a regex
scan: it finds where a name is declared. It cannot say who calls a function, who writes a field,
what offset a member sits at, or which shader reads `gWavesB.z`; those need every name resolved
the way the compiler resolves it. This indexer walks two compilers' ASTs and writes the `ast_*`
tables of [`schema.sql`](schema.sql) into `tools/Scriptorium/scriptorium.db`, where six MCP tools
read them.

- **C++**: libclang (the `libclang` pip wheel, clang 18.1.1, which bundles `libclang.dll`), fed
  each TU's flags from `build/compile_commands.json` with the MSVC spellings translated, the
  MSVC + Windows SDK include path, and `--target=x86_64-pc-windows-msvc -fms-compatibility`.
- **HLSL**: `dxc.exe -ast-dump`, one dump per `.hlsl` (its `.hlsli` come along), with the defines
  the C++ `Compile()` calls pass for that file (`TileBed.hlsl` needs `HP_TRACE=1`).

Setup, once: `py -3 -m pip install libclang` (81 MB in site-packages). After that the build keeps
the index current. Without the wheel the post-build run indexes the HLSL and writes one log line
saying the C++ was skipped.

## What is indexed

| table | rows (2026-10-15, main 2a96c5f) | what |
|---|---|---|
| `ast_decls` | 13,590 | every type, field, function, method, enumerator, typedef and namespace-scope variable in `src/` (local variables are left out; local structs are kept, since cbuffer mirrors are often local), with clang's USR, qualified name, extent, type, parent, definition flag, virtual / static / pure / access. HLSL functions, cbuffer fields and globals. |
| `ast_refs` | 187,612 | every reference that resolves to an engine declaration: `call`, `read`, `write`, `addr`, `type`, `base`, with the enclosing function. HLSL calls and global reads, with the component read (`xyz`, `[]`, `.m`) in `detail`. A call in a template with dependent arguments is recorded once per overload candidate, `detail = candidate`. |
| `ast_calls` | (view) | `ast_refs` where kind = call: caller -> callee. |
| `ast_layouts` / `ast_fields` | 803 / 6,757 | complete records: size, alignment, every field's byte offset and size. C++ as clang lays it out for the MSVC x64 ABI; HLSL cbuffers and StructuredBuffer structs COMPUTED from the packing rules. |
| `ast_includes` | 2,282 | each repo file's `#include` lines, resolved the way the compiler searches. |
| `ast_hlsl_entries` / `ast_hlsl_reach` | 81 / 990 | every function in a `.hlsl` that `src/` names by a string literal (`L"PsMain"`) is an entry point; `reach` is what each entry calls, transitively. |
| `ast_tus` | 125 | each TU's argument hash and every file it read, with (mtime, size): the incremental key. |

## The MCP tools

All answer from indexed lookups; nothing is parsed at call time. Measured through the server's
stdio (Git Bash printf piped into `dotnet .../Scriptorium.dll`): process start + initialize +
tools/list is 139 ms, and each of these calls adds 40-70 ms on the 104 MB database.

| tool | example | answers |
|---|---|---|
| `refs(symbol)` | `refs WaveField::Solved` | 29 references, each with kind, file:line:col and enclosing function; per declaration, so two members named `Update` are not merged |
| `callers(fn)` | `callers BuildGitRev` | `prune::WriteManifest`, `tree_detail::StampLive`, `main`, with call counts and the first site |
| `callees(fn)` | `callees ShaderCompiler::Compile` | the 7 engine functions it calls and where each is declared |
| `layout(type)` | `layout SceneConstants` | its fields beside `SceneCb`'s (Common.hlsli), matched by name (`gSunDir` = `sunDir`, a shared prefix like `gCs` dropped), with a verdict per field: ok / OFFSET DIFFERS / size differs / same bytes, renamed / same bytes, split |
| `cbuffer_reads(arg)` | `cbuffer_reads gMisc.z` | `PsSky` (Globe.hlsl:1840), `SkyAlong` (Common.hlsli:240) read SceneCb's; `CsBankFill` reads BankCb's. `cbuffer_reads PsSky` lists every field and component it and its 11 callees read, and the entries that reach it |
| `includes(file)` | `includes Renderer.h` | its 13 includes, the 10 files that include it, and the 9 TUs that read it at any depth |

## Numbers

Measured on this machine (16 logical CPUs), 125 TUs, warm file cache, 2026-10-15:

| run | wall |
|---|---|
| full index, 8 processes | 72-87 s (6 runs); per TU mean 4.2-4.9 s, of which parse 3.6-4.3 s; slowest TU 11-17 s (FrameLoop.cpp, main.cpp, SceneTest.cpp) |
| single TU, cold, 1 process | main.cpp 25.4 s (17.2 s parse); the same parse warm: 8.0 s |
| HLSL pass (32 dumps on 8 threads + parse) | 1.1-1.3 s |
| nothing changed | 0.50 s (stats every file each TU read) |
| one .cpp touched | 6.2-6.4 s |
| `src/core/Common.h` touched (123 of 125 TUs re-parse) | 83 s |
| launch.py, the post-build step itself | 0.45-0.71 s (one outlier of 2.4 s in 3) |
| database | 104 MB (WAL mode); `ast_refs` is most of it (USR strings and three indexes) |

Errors: 0 diagnostics across all 125 TUs, system headers included, once the three builtin
shims below are in. Before them: 10 errors in main.cpp, one fatal.

**Deterministic**: the digest of every table, sorted, was identical after a full run and after an
incremental run that re-parsed one .cpp, and identical again after one that re-parsed 123 TUs
for a touched header. Two things made it so, both found by that comparison: `= default` members
are a definition only in the TUs that use them (asked of the syntax now), and libclang's
inclusion list omits a `#pragma once` header's includes in every TU but the first to reach it
(edges come from the `#include` lines now).

**HLSL offsets are gated against the compiler**: `py -3 tools/astindex/verify_layouts.py`
compiles all 81 entry points with `dxc -Fc` and compares the offsets DXC prints in each
listing's Buffer Definitions with `ast_fields`: 34 of 34 cbuffers, 327 fields, 0 mismatches,
11.5 s. StructuredBuffer struct packing is computed the same way and is NOT gated.

## The post-build hook

`CMakeLists.txt` sets `CMAKE_EXPORT_COMPILE_COMMANDS ON` and adds a POST_BUILD step on `gagame`:
`py -3 tools/astindex/launch.py build`. The launcher starts `astindex.py` with a hidden console,
below-normal priority, at most 8 processes, stdout to `build/astindex.log`, and returns; Ninja is
not left holding a pipe, so the build does not wait (measured: the build printed `built:` at
01:52:11.9 with the indexer still running; it finished at 01:52:12). One run at a time: a build
that ends during a run leaves a mark and the running one goes round again. `-DGA_ASTINDEX=OFF`
turns the step off; without `py` on PATH it is not added.

The step runs only when `gagame` relinks, which is when there is anything new to index. By hand:
`py -3 tools/astindex/astindex.py` (incremental), `--full`, `--hlsl-only`, `--only <path part>`.

## Why these choices

- **libclang here, clangd beside it.** clangd's index (`clangd-indexer`, read by
  tools/codemap/riff.py) now carries the call graph of tools/codemap: every reference with the
  function it sits in, and the override relations. It has no layouts, no read-vs-write, no
  conditions and no HLSL, and it drops references to functions seen only in their own .cpp; those
  stay this indexer's, which also writes the three tables tools/codemap reads (`ast_regions`,
  `ast_values`, `ast_litcalls`: the condition each line runs under, the values each declaration
  may take, the calls that name something by a string).
- **libclang through the pip wheel, not a full LLVM install.** The wheel's clang 18 parses the
  MSVC 14.51 STL with three shims (`_ALLOW_COMPILER_AND_STL_VERSION_MISMATCH`, and
  `__builtin_verbose_trap`, `__is_layout_compatible`, `__is_pointer_interconvertible_base_of`
  defined away -- they are clang 19 builtins the STL uses in traits the engine never asks) and an
  empty `shim/cpuid.h` (the wheel has no clang resource headers, and DirectXMath includes it under
  clang; without it every TU reaching DirectXMath stopped at a fatal error). No full LLVM needed.
- **No `-fdelayed-template-parsing`**, though MSVC behaves that way: delayed bodies are never in the
  cursor tree, so references inside templates would be missing. Parse time did not rise (main.cpp
  7.5 s without, 8.0 s with).
- **MSVC's own outputs.** `/sourceDependencies` gives a per-TU include list (what `ast_tus` and
  `ast_includes` hold already). The PDB (76 MB here, /Zi) holds MSVC's actual record layouts and
  is the ground truth for C++ layout, but only for types that reached the binary, needs the DIA
  SDK through COM to read, and has no references, calls or HLSL; clang's MSVC-ABI layout answers
  the layout question for every complete type in one pass with the rest. Not cross-checked
  against the PDB here.
- **WAL journal**: with the default rollback journal the Scriptorium got "database is locked" for
  the length of a run; in WAL it keeps answering from the last committed index.

## What it does not see, and what the regex index still does better

- Calls through a member of a dependent type inside a template (`t.Foo()` with `t` a template
  parameter) resolve only at instantiation, which the cursor tree does not hold. Unqualified
  dependent calls are kept as `candidate`s.
- Code under a disabled `#if`, in either language: the index is of one configuration (the
  build's, and for HLSL the defines the C++ passes).
- An HLSL read through a `#define` (`gTime` is `(gParams0.x)`) is attributed to the right function
  and component, but its line is the macro's.
- HLSL entry points are names `src/` spells in a wide string literal; an entry reached only through
  a computed name would be missed.
- The regex index (`symbols`) is untouched and still the one for: Python harvesters, data-product
  provenance, channels / sources / Exchange buffers, the doc comment above a declaration, and
  substring search across names. It also works on a tree that does not compile.
