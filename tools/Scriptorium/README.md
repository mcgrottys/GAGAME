# Scriptorium

The monastery where the manuscripts are kept: a .NET 10 MCP server (stdio) over SQLite that
keeps a growing GAGAME navigable for Claude and any other MCP client.

**What it stores** (v1 — a symbol graph, honestly not yet a full AST; libclang is the named
upgrade path):

- every type, method, HLSL function, cbuffer, and Python def across `src/`, `shaders/`,
  `harvester/`, with file:line and the doc comment above it;
- the harvester registry — each script's purpose (from its header) and the `data/` outputs
  it references;
- data-product provenance — which script owns which `data/` file, size, mtime;
- the monastery registry — channels, sources (structure + CRS), Exchange buffers, scraped
  from the code itself.

**Use:**

```bash
dotnet run --project tools/Scriptorium -- --index    # rebuild scriptorium.db
```

Registered in the repo's `.mcp.json`; tools: `symbols(query)`, `who_writes(path)`,
`script_for(name)`, `channels()`, `reindex()`, `graph()` (docs/ga_ast.json — the
state-diagram contract a future node editor loads), `note(name?)` (operational
notes from `docs/LAUNCH.md`: build, run, rail videos, the verification loop, data
prerequisites, secrets policy), and `math(topic?)` — the algebra
whitepaper (`docs/ALGEBRA.md`) served per topic: the GA products, wave physics,
radiometry, frame calculus, and the PRIORS LEDGER (where measured reality diverged
from textbook/training expectations). No arg lists topics; read `priors` first when
the engine surprises you.

`plan(name?)` serves what is PROPOSED or FOUND, NOT what is built: the texture-hierarchy
proposal (`docs/HIERARCHY.md`) and the code reviews beside it (every
`docs/REVIEW_*.md`), per section. `docs/ALGEBRA.md`, served by `math`, stays the record
of what is built; these get a tool of their own so a proposal is never read as that
record. No arg lists the documents and their sections; `plan 4.6` (or `hierarchy 4.6`,
`review 1`) prints a section, `plan finding 7` prints that row of a review's tables
under its section and the table's header row, and any other text matches section
titles, then lines of the text. The documents are chosen by `PlanDocs` in `Program.cs`.

SQLite over MySQL on purpose: one file, zero administration, transactional; the schema is
plain SQL and ports to MySQL unchanged the day this becomes multi-user.
