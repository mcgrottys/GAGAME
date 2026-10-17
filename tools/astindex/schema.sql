-- tools/astindex: the AST tables written into tools/Scriptorium/scriptorium.db beside the
-- Scriptorium's own (symbols, math, notes, ...). Every table is prefixed ast_ and created IF NOT
-- EXISTS: the Scriptorium's --index drops and recreates ONLY its own tables, never the file, so the
-- two indexers share one database without either erasing the other.
--
-- Ownership: every row belongs to the FILE it is located in (`file`, repo-relative, '/'). An
-- incremental run deletes the rows of each file a re-parsed TU can see and writes them again, so
-- the answer for a file never depends on which TUs happened to be re-parsed (a header's rows are
-- the same from every TU that includes it: same flags, and template patterns, not instantiations).

CREATE TABLE IF NOT EXISTS ast_meta(key TEXT PRIMARY KEY, value TEXT);

-- One row per C++ TU: its translated arguments' hash and every file it read with (mtime_ns, size),
-- system headers included, so a changed SDK re-parses too.
CREATE TABLE IF NOT EXISTS ast_tus(tu TEXT PRIMARY KEY, args_hash TEXT, deps TEXT, seconds REAL,
                                   errors INTEGER, diags INTEGER);

-- Declarations. usr is clang's Unified Symbol Resolution for C++ (stable across TUs) and
-- 'hlsl:<name> <type>' for HLSL functions, 'hlsl:cb:<cbuffer>.<field>' for cbuffer fields.
CREATE TABLE IF NOT EXISTS ast_decls(usr TEXT, kind TEXT, name TEXT, qual TEXT, file TEXT,
                                     line INTEGER, col INTEGER, end_line INTEGER, type TEXT,
                                     parent_usr TEXT, is_def INTEGER, flags TEXT, lang TEXT);
CREATE UNIQUE INDEX IF NOT EXISTS ast_decls_u ON ast_decls(usr, file, line, col, kind);
CREATE INDEX IF NOT EXISTS ast_decls_name ON ast_decls(name);
CREATE INDEX IF NOT EXISTS ast_decls_qual ON ast_decls(qual);
CREATE INDEX IF NOT EXISTS ast_decls_file ON ast_decls(file);

-- References: every expression or type use that resolves to a declaration in the repo.
-- kind: call | read | write | addr | type | member-init | base.  container = usr of the enclosing
-- function ('' at namespace or class scope). detail: for HLSL the component read (xyz, [i], .m).
CREATE TABLE IF NOT EXISTS ast_refs(usr TEXT, name TEXT, file TEXT, line INTEGER, col INTEGER,
                                    kind TEXT, container TEXT, detail TEXT, lang TEXT);
-- No unique index here (it was 40% of the file): the indexer drops duplicates itself, which only
-- arise within one run, since a file's rows are deleted before any TU writes them again.
CREATE INDEX IF NOT EXISTS ast_refs_usr ON ast_refs(usr);
CREATE INDEX IF NOT EXISTS ast_refs_container ON ast_refs(container);
CREATE INDEX IF NOT EXISTS ast_refs_file ON ast_refs(file);

CREATE VIEW IF NOT EXISTS ast_calls AS
  SELECT container AS caller, usr AS callee, file, line, col, lang FROM ast_refs WHERE kind = 'call';

-- Record layouts. lang: cpp (clang's own record layout, MSVC ABI) | hlsl-cbuffer (HLSL constant
-- buffer packing: 16-byte rows, no straddling) | hlsl-struct (StructuredBuffer packing).
CREATE TABLE IF NOT EXISTS ast_layouts(type_usr TEXT, name TEXT, qual TEXT, file TEXT, line INTEGER,
                                       size INTEGER, align INTEGER, lang TEXT);
CREATE UNIQUE INDEX IF NOT EXISTS ast_layouts_u ON ast_layouts(type_usr, file, line, lang);
CREATE TABLE IF NOT EXISTS ast_fields(type_usr TEXT, ord INTEGER, name TEXT, type TEXT,
                                      offset INTEGER, size INTEGER, file TEXT, line INTEGER, lang TEXT);
CREATE UNIQUE INDEX IF NOT EXISTS ast_fields_u ON ast_fields(type_usr, ord, file, lang);

-- The include graph, as the preprocessor resolved it (repo includers only; the included file is
-- repo-relative when it is in the repo, else absolute).
CREATE TABLE IF NOT EXISTS ast_includes(file TEXT, included TEXT, lang TEXT);
CREATE UNIQUE INDEX IF NOT EXISTS ast_includes_u ON ast_includes(file, included);

-- tools/codemap's three: the conditions code runs under, the values fields and variables take, and
-- the calls that name something by a string (a scene key, a registry name, a shader entry). An
-- expression `expr` / `cond` / `args` is JSON: a number, true/false, null, a string literal, or a
-- list -- ["f"|"g"|"l"|"p"|"fn", usr] (field, global, local, parameter, function), [op, a, b] for
-- == != < <= > >= && || + - * / ! neg, ["call", callee usr, object or null, args...],
-- ["?:", c, a, b], ["[]", a, i], ["{}", items...] (a braced list), ["?"] for what is not modelled.
--
-- A REGION is a span of source that runs only when `cond` is true (pol 1) or false (pol -1): an if's
-- branches, a ?: arm, the right side of && and ||, a while body, a switch case's statements, and
-- the rest of a block after `if (c) return;`. pol 0 marks a loop body (cond "loop"): not a guard,
-- a multiplicity. Regions nest, so a site's condition is every region that contains it.
CREATE TABLE IF NOT EXISTS ast_regions(file TEXT, l0 INTEGER, c0 INTEGER, l1 INTEGER, c1 INTEGER,
                                       cond TEXT, pol INTEGER, container TEXT, lang TEXT);
CREATE INDEX IF NOT EXISTS ast_regions_file ON ast_regions(file);
-- A VALUE a declaration may take: kind default (a field's in-class initializer), init (a global's
-- initializer), local (a local's initializer; detail 'const' when it is const, 'ref' when it is a
-- reference: the name it is initialized from is what its writes write), assign (x = e,
-- x op= e, ++x: expr ["?"] unless plain '='), byref (passed to a non-const reference parameter,
-- detail = the callee's name; or its address taken, detail '&'; or a non-const member called on it,
-- detail = the member: ptr.reset(), name.clear()), aggregate (usr = a record initialized from a braced list: every field
-- of it may be written). detail for an assign through a subscript: the index, e.g. '[0]'.
CREATE TABLE IF NOT EXISTS ast_values(usr TEXT, file TEXT, line INTEGER, kind TEXT, expr TEXT,
                                      container TEXT, detail TEXT, lang TEXT);
CREATE INDEX IF NOT EXISTS ast_values_usr ON ast_values(usr);
CREATE INDEX IF NOT EXISTS ast_values_file ON ast_values(file);
-- A call with at least one string literal among its arguments: Schema::Bind("shadowSteps", p.x),
-- Register("car.baja", MakeCarBaja), S.Tool("ingest"), Compile(L"Globe.hlsl", L"PsSurface", ...).
CREATE TABLE IF NOT EXISTS ast_litcalls(callee TEXT, name TEXT, file TEXT, line INTEGER, col INTEGER,
                                        container TEXT, args TEXT, lang TEXT);
CREATE INDEX IF NOT EXISTS ast_litcalls_callee ON ast_litcalls(callee);
CREATE INDEX IF NOT EXISTS ast_litcalls_file ON ast_litcalls(file);

-- HLSL: entry points (a function in a .hlsl named by a string literal in src/), and which
-- functions each entry reaches through the call graph of its own file's AST.
CREATE TABLE IF NOT EXISTS ast_hlsl_entries(file TEXT, entry TEXT, profile TEXT, usr TEXT,
                                            defines TEXT, cpp_site TEXT);
CREATE TABLE IF NOT EXISTS ast_hlsl_reach(entry_file TEXT, entry TEXT, usr TEXT);
CREATE INDEX IF NOT EXISTS ast_hlsl_reach_usr ON ast_hlsl_reach(usr);
