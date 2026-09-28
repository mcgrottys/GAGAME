// ================================================================================================
//  Scriptorium - the monastery where the manuscripts are kept.
//
//  A .NET MCP server (stdio, newline-delimited JSON-RPC) backed by SQLite, so a growing
//  GAGAME stays navigable: every symbol across C++/HLSL/Python, every harvester and the data
//  products it owns, and the monastery registry (channels + sources + CRS) mirrored from the
//  code. v1 is a SYMBOL GRAPH, not a full AST -- honest and fast; libclang is the named
//  upgrade path if this ever proves too shallow.
//
//  Modes:
//    dotnet run -- --index          scan the repo, (re)build scriptorium.db, print a summary
//    dotnet run                     serve MCP over stdio (tools: symbols, who_writes,
//                                   script_for, channels, reindex, graph, note, math, plan)
//
//  'math' is the record of what IS built (docs/ALGEBRA.md). 'plan' serves what is PROPOSED
//  or FOUND -- the texture-hierarchy proposal (docs/HIERARCHY.md) and the code reviews beside
//  it (docs/REVIEW_*.md), per section -- and never stands in for that record.
//
//  SQLite over MySQL: single file, zero administration, transactional; the schema is plain
//  SQL and ports the day it needs to be multi-user.
// ================================================================================================
using System.Text;
using System.Text.Json;
using System.Text.Json.Nodes;
using System.Text.RegularExpressions;
using Microsoft.Data.Sqlite;

var repo = FindRepoRoot();
var dbPath = Path.Combine(repo, "tools", "Scriptorium", "scriptorium.db");

if (args.Contains("--index"))
{
    var counts = Indexer.Run(repo, dbPath);
    // The counts print here and not inside Run: 'reindex' and the first-run index call Run
    // while stdout is the JSON-RPC channel, where a bare line is a frame no client can parse.
    Console.WriteLine($"[scriptorium] {counts.math} math topics from docs/ALGEBRA.md");
    Console.WriteLine($"[scriptorium] {counts.notes} notes from docs/LAUNCH.md");
    Console.WriteLine($"[scriptorium] {counts.plan} plan sections from {counts.planDocs} documents");
    Console.WriteLine($"[scriptorium] {counts.symbols} symbols, {counts.scripts} scripts, " +
                      $"{counts.products} products, {counts.channels} channel/source rows -> {dbPath}");
    return;
}

if (!File.Exists(dbPath)) Indexer.Run(repo, dbPath);
McpServer.Serve(repo, dbPath);
return;

static string FindRepoRoot()
{
    var d = new DirectoryInfo(AppContext.BaseDirectory);
    for (var p = d; p != null; p = p.Parent)
        if (File.Exists(Path.Combine(p.FullName, "GAMEPLAN.md"))) return p.FullName;
    for (var p = new DirectoryInfo(Directory.GetCurrentDirectory()); p != null; p = p.Parent)
        if (File.Exists(Path.Combine(p.FullName, "GAMEPLAN.md"))) return p.FullName;
    return Directory.GetCurrentDirectory();
}

// ------------------------------------------------------------------------------- indexing

static class Indexer
{
    public static (int symbols, int scripts, int products, int channels, int math, int notes, int plan, int planDocs)
        Run(string repo, string dbPath)
    {
        Directory.CreateDirectory(Path.GetDirectoryName(dbPath)!);
        using var db = new SqliteConnection($"Data Source={dbPath}");
        db.Open();
        Exec(db, """
            DROP TABLE IF EXISTS symbols;
            DROP TABLE IF EXISTS scripts;
            DROP TABLE IF EXISTS products;
            DROP TABLE IF EXISTS channels;
            DROP TABLE IF EXISTS math;
            DROP TABLE IF EXISTS notes;
            DROP TABLE IF EXISTS plan_sections;
            CREATE TABLE symbols(name TEXT, kind TEXT, file TEXT, line INTEGER, doc TEXT);
            CREATE TABLE math(topic TEXT PRIMARY KEY, title TEXT, body TEXT);
            CREATE TABLE notes(name TEXT PRIMARY KEY, title TEXT, body TEXT);
            CREATE TABLE plan_sections(doc TEXT, number TEXT, title TEXT, level INTEGER, ord INTEGER, body TEXT);
            CREATE TABLE scripts(name TEXT PRIMARY KEY, path TEXT, purpose TEXT, outputs TEXT);
            CREATE TABLE products(path TEXT PRIMARY KEY, script TEXT, bytes INTEGER, mtime TEXT);
            CREATE TABLE channels(name TEXT, kind TEXT, detail TEXT, file TEXT, line INTEGER);
            CREATE INDEX idx_sym ON symbols(name);
            """);

        int symbols = 0, channels = 0;
        using var tx = db.BeginTransaction();

        foreach (var file in Enumerate(repo, "src", "*.h").Concat(Enumerate(repo, "src", "*.cpp")))
            symbols += IndexCpp(db, repo, file, ref channels);
        foreach (var file in Enumerate(repo, "shaders", "*.hlsl").Concat(Enumerate(repo, "shaders", "*.hlsli")))
            symbols += IndexHlsl(db, repo, file);
        var scripts = 0;
        foreach (var file in Enumerate(repo, "harvester", "*.py"))
        {
            IndexPython(db, repo, file, ref symbols);
            scripts++;
        }
        var products = IndexProducts(db, repo);
        var math = IndexMath(db, repo);
        var notes = IndexNotes(db, repo);
        var (plan, planDocs) = IndexPlan(db, repo);

        tx.Commit();
        return (symbols, scripts, products, channels, math, notes, plan, planDocs);
    }

    static IEnumerable<string> Enumerate(string repo, string sub, string pattern)
    {
        var dir = Path.Combine(repo, sub);
        return Directory.Exists(dir)
            ? Directory.EnumerateFiles(dir, pattern, SearchOption.AllDirectories)
            : [];
    }

    static readonly Regex CppType = new(@"^\s*(class|struct|enum class)\s+([A-Za-z_]\w*)\s*[:{;]", RegexOptions.Compiled);
    static readonly Regex CppMethod = new(@"^[\w:<>*&~\[\] ]*?\b([A-Za-z_]\w*)::([A-Za-z_~]\w*)\s*\(", RegexOptions.Compiled);
    static readonly Regex CppChannel = new(@"Add(Color|Height)Channel\(\s*""([^""]+)""", RegexOptions.Compiled);
    static readonly Regex CppSource = new(@"m_info\s*=\s*\{\s*""([^""]+)""\s*,\s*""([^""]+)""\s*,\s*""([^""]+)""", RegexOptions.Compiled);
    static readonly Regex CppExchange = new(@"Register\(\s*\n?\s*""([^""]+)""", RegexOptions.Compiled);

    static int IndexCpp(SqliteConnection db, string repo, string file, ref int channels)
    {
        var rel = Path.GetRelativePath(repo, file).Replace('\\', '/');
        var lines = File.ReadAllLines(file);
        var n = 0;
        for (var i = 0; i < lines.Length; i++)
        {
            var m = CppType.Match(lines[i]);
            if (m.Success)
            {
                Insert(db, m.Groups[2].Value, m.Groups[1].Value, rel, i + 1, DocAbove(lines, i));
                n++;
            }
            var mm = CppMethod.Match(lines[i]);
            if (mm.Success && !lines[i].TrimStart().StartsWith("//"))
            {
                Insert(db, $"{mm.Groups[1].Value}::{mm.Groups[2].Value}", "method", rel, i + 1, DocAbove(lines, i));
                n++;
            }
            var ch = CppChannel.Match(lines[i]);
            if (ch.Success)
            {
                Exec(db, "INSERT INTO channels VALUES($n,$k,$d,$f,$l)",
                     ("$n", ch.Groups[2].Value), ("$k", ch.Groups[1].Value.ToLowerInvariant() + "-channel"),
                     ("$d", ""), ("$f", rel), ("$l", i + 1));
                channels++;
            }
            var src = CppSource.Match(lines[i]);
            if (src.Success)
            {
                Exec(db, "INSERT INTO channels VALUES($n,$k,$d,$f,$l)",
                     ("$n", src.Groups[1].Value), ("$k", "source"),
                     ("$d", $"{src.Groups[2].Value} | {src.Groups[3].Value}"), ("$f", rel), ("$l", i + 1));
                channels++;
            }
            var ex = CppExchange.Match(lines[i]);
            if (ex.Success && rel.Contains("main.cpp"))
            {
                Exec(db, "INSERT INTO channels VALUES($n,$k,$d,$f,$l)",
                     ("$n", ex.Groups[1].Value), ("$k", "exchange-buffer"), ("$d", ""), ("$f", rel), ("$l", i + 1));
                channels++;
            }
        }
        return n;
    }

    static readonly Regex HlslFn = new(@"^\s*(?:float[234x]*|void|bool|uint[234]?|int[234]?|half)\s+([A-Za-z_]\w*)\s*\(", RegexOptions.Compiled);
    static readonly Regex HlslCb = new(@"^\s*cbuffer\s+([A-Za-z_]\w*)", RegexOptions.Compiled);

    static int IndexHlsl(SqliteConnection db, string repo, string file)
    {
        var rel = Path.GetRelativePath(repo, file).Replace('\\', '/');
        var lines = File.ReadAllLines(file);
        var n = 0;
        for (var i = 0; i < lines.Length; i++)
        {
            var f = HlslFn.Match(lines[i]);
            if (f.Success) { Insert(db, f.Groups[1].Value, "hlsl-fn", rel, i + 1, DocAbove(lines, i)); n++; }
            var c = HlslCb.Match(lines[i]);
            if (c.Success) { Insert(db, c.Groups[1].Value, "cbuffer", rel, i + 1, DocAbove(lines, i)); n++; }
        }
        return n;
    }

    static readonly Regex PyDef = new(@"^def\s+([A-Za-z_]\w*)", RegexOptions.Compiled);

    static void IndexPython(SqliteConnection db, string repo, string file, ref int symbols)
    {
        var rel = Path.GetRelativePath(repo, file).Replace('\\', '/');
        var lines = File.ReadAllLines(file);
        var purpose = new StringBuilder();
        foreach (var l in lines.Take(20))
        {
            var t = l.TrimStart('#', ' ', '=');
            if (l.StartsWith('#') && t.Length > 3) purpose.AppendLine(t);
        }
        var outputs = string.Join(";",
            Regex.Matches(string.Join('\n', lines), @"""(data[/\\][^""]+)""|'(data[/\\][^']+)'")
                 .Select(m => (m.Groups[1].Success ? m.Groups[1].Value : m.Groups[2].Value).Replace('\\', '/'))
                 .Distinct().Take(8));
        Exec(db, "INSERT OR REPLACE INTO scripts VALUES($n,$p,$u,$o)",
             ("$n", Path.GetFileName(file)), ("$p", rel),
             ("$u", purpose.ToString().Trim()), ("$o", outputs));
        for (var i = 0; i < lines.Length; i++)
        {
            var d = PyDef.Match(lines[i]);
            if (d.Success) { Insert(db, d.Groups[1].Value, "py-def", rel, i + 1, ""); symbols++; }
        }
    }

    static int IndexProducts(SqliteConnection db, string repo)
    {
        var dataDir = Path.Combine(repo, "data");
        if (!Directory.Exists(dataDir)) return 0;
        var n = 0;
        foreach (var f in Directory.EnumerateFiles(dataDir, "*", SearchOption.AllDirectories))
        {
            var rel = Path.GetRelativePath(repo, f).Replace('\\', '/');
            var family = rel.Split('/').Skip(1).FirstOrDefault() ?? "";
            var script = $"harvest_{family}.py";
            if (!File.Exists(Path.Combine(repo, "harvester", script))) script = "";
            var fi = new FileInfo(f);
            Exec(db, "INSERT OR REPLACE INTO products VALUES($p,$s,$b,$m)",
                 ("$p", rel), ("$s", script), ("$b", fi.Length),
                 ("$m", fi.LastWriteTimeUtc.ToString("u")));
            n++;
        }
        return n;
    }

    static string DocAbove(string[] lines, int i)
    {
        var doc = new List<string>();
        for (var j = i - 1; j >= 0 && j > i - 6; j--)
        {
            var t = lines[j].TrimStart();
            if (t.StartsWith("//")) doc.Insert(0, t.TrimStart('/', ' '));
            else break;
        }
        return string.Join(' ', doc);
    }

    static void Insert(SqliteConnection db, string name, string kind, string file, int line, string doc)
        => Exec(db, "INSERT INTO symbols VALUES($n,$k,$f,$l,$d)",
                ("$n", name), ("$k", kind), ("$f", file), ("$l", line), ("$d", doc));

    // M7t: THE ALGEBRA WHITEPAPER (docs/ALGEBRA.md) ingested per topic -- sections are
    // "## topic-id -- Title"; the MCP 'math' tool serves them so a human (or an outside
    // expert) can read the engine's actual mathematics, including the priors ledger:
    // the places where measured reality diverged from textbook/training expectations.
    // The operational notes (docs/LAUNCH.md today): how to build, run, render the rail
    // videos, and interrogate the engine -- same section grammar as the whitepaper, so a
    // fresh session (or a new pair of hands) can ask the monastery instead of the
    // scrollback. Served by the 'note' tool.
    static int IndexNotes(SqliteConnection db, string repo)
    {
        var path = Path.Combine(repo, "docs", "LAUNCH.md");
        if (!File.Exists(path)) return 0;
        var lines = File.ReadAllLines(path);
        var n = 0;
        string? name = null, title = null;
        var body = new StringBuilder();
        void Flush()
        {
            if (name == null) return;
            Exec(db, "INSERT OR REPLACE INTO notes(name, title, body) VALUES($n, $t, $b)",
                 ("$n", name), ("$t", title ?? name), ("$b", body.ToString().Trim()));
            n++;
            body.Clear();
        }
        foreach (var line in lines)
        {
            var m = Regex.Match(line, @"^## ([a-z0-9-]+) — (.+)$");
            if (!m.Success) m = Regex.Match(line, @"^## ([a-z0-9-]+) -- (.+)$");
            if (m.Success)
            {
                Flush();
                name = m.Groups[1].Value;
                title = m.Groups[2].Value;
            }
            else if (name != null)
            {
                body.AppendLine(line);
            }
        }
        Flush();
        return n;
    }

    static int IndexMath(SqliteConnection db, string repo)
    {
        var path = Path.Combine(repo, "docs", "ALGEBRA.md");
        if (!File.Exists(path)) return 0;
        var lines = File.ReadAllLines(path);
        var n = 0;
        string? topic = null, title = null;
        var body = new StringBuilder();
        void Flush()
        {
            if (topic == null) return;
            Exec(db, "INSERT OR REPLACE INTO math(topic, title, body) VALUES($t, $ti, $b)",
                 ("$t", topic), ("$ti", title ?? topic), ("$b", body.ToString().Trim()));
            n++;
            body.Clear();
        }
        foreach (var line in lines)
        {
            var m = Regex.Match(line, @"^## ([a-z0-9-]+) — (.+)$");
            if (!m.Success) m = Regex.Match(line, @"^## ([a-z0-9-]+) -- (.+)$");
            if (m.Success)
            {
                Flush();
                topic = m.Groups[1].Value;
                title = m.Groups[2].Value.Trim();
                continue;
            }
            if (topic != null) body.AppendLine(line);
        }
        Flush();
        return n;
    }

    // The plan documents: what is PROPOSED (the texture hierarchy) and what a review FOUND.
    // They are not folded into ALGEBRA.md, which must stay the record of what IS built, so
    // 'plan' serves them apart. The proposal is named; the reviews are every docs/REVIEW_*.md
    // in name order, so the next review is served the day it is written. A missing file is
    // skipped, not an error.
    static IEnumerable<string> PlanDocs(string repo)
    {
        var docs = Path.Combine(repo, "docs");
        if (!Directory.Exists(docs)) yield break;
        var proposal = Path.Combine(docs, "HIERARCHY.md");
        if (File.Exists(proposal)) yield return proposal;
        foreach (var review in Directory.GetFiles(docs, "REVIEW_*.md").OrderBy(p => p, StringComparer.Ordinal))
            yield return review;
    }

    // Levels 1-3. The '# ' title is a heading too, so the preamble under it (HIERARCHY's
    // status line and its legend for measured / emulated / documented) is kept and searched.
    static readonly Regex PlanHeading = new(@"^(#{1,3})\s+(.+?)\s*$", RegexOptions.Compiled);
    static readonly Regex PlanNumber = new(@"^(\d+(?:\.\d+)*)\.?\s+(.+)$", RegexOptions.Compiled);

    // One rule for every level: a section's body runs to the next heading of ANY level, so a
    // '## ' with subsections holds only its preamble and 'plan 4.6' never runs on into 4.7.
    // A '## ' inside a code fence is an example, not a heading.
    static (int sections, int docs) IndexPlan(SqliteConnection db, string repo)
    {
        int n = 0, docs = 0;
        foreach (var path in PlanDocs(repo))
        {
            docs++;
            var doc = Path.GetFileName(path);
            string? number = null, title = null, fence = null;
            int level = 0, ord = 0;
            var body = new StringBuilder();
            void Flush()
            {
                if (title == null) return;
                Exec(db, "INSERT INTO plan_sections VALUES($d, $n, $t, $l, $o, $b)",
                     ("$d", doc), ("$n", number ?? ""), ("$t", title), ("$l", level), ("$o", ord),
                     ("$b", body.ToString().Trim('\n')));
                n++;
                ord++;
                body.Clear();
            }
            foreach (var line in File.ReadAllLines(path))
            {
                var t = line.TrimStart();
                if (fence == null && (t.StartsWith("```") || t.StartsWith("~~~"))) fence = t[..3];
                else if (fence != null && t.StartsWith(fence)) fence = null;
                else if (fence == null && PlanHeading.Match(line) is { Success: true } h)
                {
                    Flush();
                    level = h.Groups[1].Length;
                    var num = PlanNumber.Match(h.Groups[2].Value);
                    number = num.Success ? num.Groups[1].Value : "";
                    title = num.Success ? num.Groups[2].Value : h.Groups[2].Value;
                    continue;
                }
                // '\n', not AppendLine: the server splits bodies into lines to find a table's
                // rows, and AppendLine would leave a '\r' on each one under Windows.
                if (title != null) body.Append(line).Append('\n');
            }
            Flush();
        }
        return (n, docs);
    }

    static void Exec(SqliteConnection db, string sql, params (string, object)[] args)
    {
        using var cmd = db.CreateCommand();
        cmd.CommandText = sql;
        foreach (var (k, v) in args) cmd.Parameters.AddWithValue(k, v);
        cmd.ExecuteNonQuery();
    }
}

// ------------------------------------------------------------------------------- MCP

static class McpServer
{
    public static void Serve(string repo, string dbPath)
    {
        using var db = new SqliteConnection($"Data Source={dbPath}");
        db.Open();
        // Stdin is read as UTF-8 whatever the console's code page. MCP frames are UTF-8 and a
        // JS client sends non-ASCII raw; Console.ReadLine decoded it with the OEM page (437
        // here), so an 'é' reached every tool as box-drawing characters. Replies were never
        // at risk: the serializer escapes all non-ASCII as \uXXXX.
        using var stdin = new StreamReader(Console.OpenStandardInput(), new UTF8Encoding(false));
        string? line;
        while ((line = stdin.ReadLine()) != null)
        {
            // A BOM on the very first line (PowerShell pipes, some launchers) made Parse
            // throw and the silent catch DROPPED the handshake -- the whole server looked
            // dead to any client whose first message was initialize. Strip it.
            line = line.TrimStart('\uFEFF', ' ', '\t');
            if (string.IsNullOrWhiteSpace(line)) continue;
            JsonNode? msg;
            try { msg = JsonNode.Parse(line); }
            catch (Exception e) { Console.Error.WriteLine($"[scriptorium] bad json: {e.Message}"); continue; }
            var method = msg?["method"]?.GetValue<string>();
            var id = msg?["id"];
            if (method == null) continue;
            if (id == null) continue;   // notifications need no reply

            JsonNode result = method switch
            {
                "initialize" => JsonNode.Parse("""
                    {"protocolVersion":"2024-11-05",
                     "capabilities":{"tools":{}},
                     "serverInfo":{"name":"scriptorium","version":"1.0"}}
                    """)!,
                "tools/list" => ToolList(),
                "tools/call" => ToolCall(db, repo, dbPath, msg!),
                _ => JsonNode.Parse("{}")!
            };
            var reply = new JsonObject
            {
                ["jsonrpc"] = "2.0",
                ["id"] = id.DeepClone(),
                ["result"] = result
            };
            Console.WriteLine(reply.ToJsonString(new JsonSerializerOptions { WriteIndented = false }));
        }
    }

    static JsonNode ToolList() => JsonNode.Parse("""
        {"tools":[
          {"name":"symbols","description":"Search the symbol graph (types, methods, HLSL fns, cbuffers, python defs) by substring.","inputSchema":{"type":"object","properties":{"query":{"type":"string"}},"required":["query"]}},
          {"name":"who_writes","description":"Which harvester script owns a data product path (substring match).","inputSchema":{"type":"object","properties":{"path":{"type":"string"}},"required":["path"]}},
          {"name":"script_for","description":"Describe a harvester script: purpose and outputs.","inputSchema":{"type":"object","properties":{"name":{"type":"string"}},"required":["name"]}},
          {"name":"channels","description":"The monastery registry: channels, sources (with structure/CRS), Exchange buffers.","inputSchema":{"type":"object","properties":{}}},
          {"name":"reindex","description":"Rescan the repo and rebuild the database.","inputSchema":{"type":"object","properties":{}}},
          {"name":"graph","description":"The GA state diagram as machine-readable JSON (docs/ga_ast.json, emitted by the engine every boot): nodes + edges with frames, units, ranges, gains, flips, code anchors. The contract a future Blueprint-style node editor loads/saves; tools/astdiagram.py renders it to docs/diagrams/*.svg.","inputSchema":{"type":"object","properties":{}}},
          {"name":"note","description":"Operational notes (docs/LAUNCH.md): how to BUILD (vcvars64 + VS cmake + Ninja), RUN (windowed / headless renders, camera + time + storm flags), render the RAIL VIDEOS (+ the ffmpeg line), the VERIFICATION LOOP (seven gates, the --trace hypervisor, fiber dumps, the 2D proof figure + match report, data lenses), DATA prerequisites (harvesters, D:\\DataCache), and SECRETS policy. No arg: list sections. With name (substring, e.g. 'build', 'videos', 'verify'): print that section.","inputSchema":{"type":"object","properties":{"name":{"type":"string"}}}},
          {"name":"math","description":"The algebra whitepaper (docs/ALGEBRA.md), served per topic. No arg: list topics. With topic (substring): print that section's mathematics for a human or an outside expert -- GA products, wave physics, radiometry, frames, the compositor's algebra, and the PRIORS LEDGER (where measured reality diverged from textbook/training expectations; read it first when the engine surprises you).","inputSchema":{"type":"object","properties":{"topic":{"type":"string"}}}},
          {"name":"plan","description":"What is PROPOSED or FOUND, NOT what is built: the texture-hierarchy proposal (docs/HIERARCHY.md) and the code reviews beside it (docs/REVIEW_*.md), served per section. The record of what IS built is 'math' (docs/ALGEBRA.md); never cite these as the engine's behaviour. No arg: list the documents and their numbered sections. With name: a section number, optionally after a document word (e.g. '4.6', 'hierarchy 5', 'review 1'), prints that section (a parent prints its preamble and the list of its subsections); 'finding N' (e.g. 'finding 7') prints that row of the review's tables under its section heading and the table's header row; anything else matches section titles (substring), and failing that, lines of the text.","inputSchema":{"type":"object","properties":{"name":{"type":"string"}}}}
        ]}
        """)!;

    static JsonNode ToolCall(SqliteConnection db, string repo, string dbPath, JsonNode msg)
    {
        var name = msg["params"]?["name"]?.GetValue<string>() ?? "";
        var args = msg["params"]?["arguments"];
        var text = name switch
        {
            "symbols" => Query(db,
                "SELECT kind, name, file, line, doc FROM symbols WHERE name LIKE $q ORDER BY file, line LIMIT 60",
                ("$q", $"%{args?["query"]?.GetValue<string>() ?? ""}%"),
                r => $"{r.GetString(0),-10} {r.GetString(1),-44} {r.GetString(2)}:{r.GetInt32(3)}" +
                     (r.GetString(4).Length > 0 ? $"\n             {Truncate(r.GetString(4), 100)}" : "")),
            "who_writes" => Query(db,
                "SELECT path, script, bytes, mtime FROM products WHERE path LIKE $q LIMIT 40",
                ("$q", $"%{args?["path"]?.GetValue<string>() ?? ""}%"),
                r => $"{r.GetString(0)}  <- {(r.GetString(1).Length > 0 ? r.GetString(1) : "(unknown)")}  {r.GetInt64(2)} B  {r.GetString(3)}"),
            "script_for" => Query(db,
                "SELECT name, path, purpose, outputs FROM scripts WHERE name LIKE $q",
                ("$q", $"%{args?["name"]?.GetValue<string>() ?? ""}%"),
                r => $"{r.GetString(0)} ({r.GetString(1)})\n{r.GetString(2)}\noutputs: {r.GetString(3)}\n"),
            "channels" => Query(db,
                "SELECT kind, name, detail, file, line FROM channels ORDER BY kind, name",
                null,
                r => $"{r.GetString(0),-16} {r.GetString(1),-26} {Truncate(r.GetString(2), 60),-60} {r.GetString(3)}:{r.GetInt32(4)}"),
            "reindex" => Reindex(repo, dbPath),
            "graph" => File.Exists(Path.Combine(repo, "docs", "ga_ast.json"))
                ? File.ReadAllText(Path.Combine(repo, "docs", "ga_ast.json"))
                : "(docs/ga_ast.json not found -- run the engine once to emit it)",
            "note" => (args?["name"]?.GetValue<string>() is { Length: > 0 } nn)
                ? Query(db,
                    "SELECT name, title, body FROM notes WHERE name LIKE $q OR title LIKE $q",
                    ("$q", $"%{nn}%"),
                    r => $"## {r.GetString(0)} — {r.GetString(1)}\n\n{r.GetString(2)}\n")
                : Query(db,
                    "SELECT name, title FROM notes ORDER BY rowid",
                    null,
                    r => $"{r.GetString(0),-16} {r.GetString(1)}"),
            "math" => (args?["topic"]?.GetValue<string>() is { Length: > 0 } t)
                ? Query(db,
                    "SELECT topic, title, body FROM math WHERE topic LIKE $q OR title LIKE $q",
                    ("$q", $"%{t}%"),
                    r => $"## {r.GetString(0)} — {r.GetString(1)}\n\n{r.GetString(2)}\n")
                : Query(db,
                    "SELECT topic, title FROM math ORDER BY rowid",
                    null,
                    r => $"{r.GetString(0),-14} {r.GetString(1)}"),
            // ToString, not GetValue<string>: a section number is the argument most likely to
            // arrive as a JSON number (4.6), and GetValue<string> would throw and end the server.
            "plan" => Plan(db, args?["name"]?.ToString()),
            _ => $"unknown tool '{name}'"
        };
        return new JsonObject
        {
            ["content"] = new JsonArray(new JsonObject { ["type"] = "text", ["text"] = text })
        };
    }

    static string Reindex(string repo, string dbPath)
    {
        var c = Indexer.Run(repo, dbPath);
        return $"reindexed: {c.symbols} symbols, {c.scripts} scripts, {c.products} products, {c.channels} channel rows, " +
               $"{c.math} math topics, {c.notes} notes, {c.plan} plan sections from {c.planDocs} documents";
    }

    // 'plan': the proposal and the review, per section. Everything it prints is PROPOSED or
    // FOUND; the listing says so on its first line because an agent that lists and skims would
    // otherwise take a proposed structure for the engine's. Every section prints one way,
    // whichever route found it: heading, body, and, for a parent, the list of what is under it.
    static string Plan(SqliteConnection db, string? name)
    {
        var s = new List<(string doc, string number, string title, int level, string body)>();
        try
        {
            using var cmd = db.CreateCommand();
            cmd.CommandText = "SELECT doc, number, title, level, body FROM plan_sections ORDER BY rowid";
            using var r = cmd.ExecuteReader();
            while (r.Read()) s.Add((r.GetString(0), r.GetString(1), r.GetString(2), r.GetInt32(3), r.GetString(4)));
        }
        // A database indexed by an older build has no plan table, and an exception here would
        // end the server and every other tool with it.
        catch (SqliteException) { return "(no plan sections: this database predates the plan tool -- call reindex)"; }
        if (s.Count == 0) return "(no plan sections indexed: are the plan documents in docs/? call reindex)";

        var q = (name ?? "").Trim();
        var nothing = $"(nothing in the plan documents answers '{q}'; call plan with no argument to list every section)";
        var o = new List<string>();
        static bool Names(string word, string doc) => doc.Contains(word, StringComparison.OrdinalIgnoreCase);
        string Head(int i) =>
            $"{s[i].doc}  {new string('#', s[i].level)} {(s[i].number.Length > 0 ? s[i].number + " " : "")}{s[i].title}";
        string Entry(int i, int depth) =>
            new string(' ', 2 * depth) + (s[i].number.Length > 0 ? $"{s[i].number,-5} " : "") + s[i].title;
        IEnumerable<int> Under(int i)
        {
            for (var j = i + 1; j < s.Count && s[j].doc == s[i].doc && s[j].level > s[i].level; j++) yield return j;
        }
        void Full(int i)
        {
            if (o.Count > 0) o.Add("");
            o.Add(Head(i));
            if (s[i].body.Length > 0) { o.Add(""); o.Add(s[i].body); }
            var under = Under(i).ToList();
            if (under.Count == 0) return;
            o.Add("");
            o.Add("subsections:");
            foreach (var k in under) o.Add(Entry(k, s[k].level - s[i].level));
        }

        if (q.Length == 0)
        {
            o.Add("These are PROPOSALS and FINDINGS, NOT what is built: `math` (docs/ALGEBRA.md) is the record of what is built.");
            for (var i = 0; i < s.Count; i++)
            {
                if (i == 0 || s[i].doc != s[i - 1].doc) { o.Add(""); o.Add(s[i].doc); }
                o.Add(Entry(i, s[i].level));
            }
            o.Add("");
            o.Add("plan '4.6' or 'hierarchy 4.6' prints a section, 'finding 7' a row of the review; other text searches the titles, then the text.");
            return string.Join('\n', o);
        }

        // A finding is a row of one of the review's tables whose first cell is its number. It
        // prints under its section's heading with the table's header row above it, so the
        // columns can be read; 'review' is the document word, as in 'review 1'.
        var f = Regex.Match(q, @"^findings?\s*(\d+)$", RegexOptions.IgnoreCase);
        if (f.Success)
        {
            for (var i = 0; i < s.Count; i++)
            {
                if (!Names("review", s[i].doc)) continue;
                var lines = s[i].body.Split('\n');
                for (var j = 0; j < lines.Length; j++)
                {
                    var cells = lines[j].Trim().Split('|');
                    if (cells.Length < 3 || cells[0].Length > 0 || cells[1].Trim() != f.Groups[1].Value) continue;
                    var top = j;
                    while (top > 0 && lines[top - 1].TrimStart().StartsWith('|')) top--;
                    if (o.Count > 0) o.Add("");
                    o.Add(Head(i));
                    o.Add("");
                    for (var h = top; h < Math.Min(top + 2, j); h++) o.Add(lines[h]);   // header, separator
                    o.Add(lines[j]);
                }
            }
            return o.Count > 0 ? string.Join('\n', o) : nothing;
        }

        // A number, alone or after a document word. A first word that names no document is not
        // a document word, and the whole name falls through to the title search below.
        var m = Regex.Match(q, @"^(?:(\S+)\s+)?(\d+(?:\.\d+)*)\.?$");
        var word = m.Groups[1].Success ? m.Groups[1].Value : null;
        if (m.Success && (word == null || s.Any(x => Names(word, x.doc))))
        {
            for (var i = 0; i < s.Count; i++)
                if (s[i].number == m.Groups[2].Value && (word == null || Names(word, s[i].doc))) Full(i);
            return o.Count > 0 ? string.Join('\n', o) : nothing;
        }

        for (var i = 0; i < s.Count; i++)
            if (s[i].title.Contains(q, StringComparison.OrdinalIgnoreCase)) Full(i);
        if (o.Count > 0) return string.Join('\n', o);

        o.Add($"No section title contains '{q}'; the lines that do, under their sections:");
        var hits = 0;
        for (var i = 0; i < s.Count; i++)
        {
            var headed = false;
            foreach (var line in s[i].body.Split('\n'))
            {
                if (!line.Contains(q, StringComparison.OrdinalIgnoreCase) || ++hits > 40) continue;
                if (!headed) { o.Add(Head(i)); headed = true; }
                o.Add("    " + line.Trim());
            }
        }
        if (hits == 0) return nothing;
        if (hits > 40) o.Add($"... and {hits - 40} more lines");
        return string.Join('\n', o);
    }

    static string Query(SqliteConnection db, string sql, (string, object)? arg, Func<SqliteDataReader, string> fmt)
    {
        using var cmd = db.CreateCommand();
        cmd.CommandText = sql;
        if (arg is { } a) cmd.Parameters.AddWithValue(a.Item1, a.Item2);
        using var r = cmd.ExecuteReader();
        var sb = new StringBuilder();
        var n = 0;
        while (r.Read()) { sb.AppendLine(fmt(r)); n++; }
        return n == 0 ? "(no rows)" : sb.ToString();
    }

    static string Truncate(string s, int n) => s.Length <= n ? s : s[..n] + "...";
}
