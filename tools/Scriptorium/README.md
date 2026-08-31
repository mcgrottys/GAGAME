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
`script_for(name)`, `channels()`, `reindex()`, and `math(topic?)` — the algebra
whitepaper (`docs/ALGEBRA.md`) served per topic: the GA products, wave physics,
radiometry, frame calculus, and the PRIORS LEDGER (where measured reality diverged
from textbook/training expectations). No arg lists topics; read `priors` first when
the engine surprises you.

SQLite over MySQL on purpose: one file, zero administration, transactional; the schema is
plain SQL and ports to MySQL unchanged the day this becomes multi-user.
