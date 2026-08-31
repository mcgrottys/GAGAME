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
//                                   script_for, channels, reindex)
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
    public static (int symbols, int scripts, int products, int channels) Run(string repo, string dbPath)
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
            CREATE TABLE symbols(name TEXT, kind TEXT, file TEXT, line INTEGER, doc TEXT);
            CREATE TABLE math(topic TEXT PRIMARY KEY, title TEXT, body TEXT);
            CREATE TABLE notes(name TEXT PRIMARY KEY, title TEXT, body TEXT);
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

        tx.Commit();
        Console.WriteLine($"[scriptorium] {math} math topics from docs/ALGEBRA.md");
        Console.WriteLine($"[scriptorium] {notes} notes from docs/LAUNCH.md");
        return (symbols, scripts, products, channels);
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
        string? line;
        while ((line = Console.ReadLine()) != null)
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
          {"name":"math","description":"The algebra whitepaper (docs/ALGEBRA.md), served per topic. No arg: list topics. With topic (substring): print that section's mathematics for a human or an outside expert -- GA products, wave physics, radiometry, frames, the compositor's algebra, and the PRIORS LEDGER (where measured reality diverged from textbook/training expectations; read it first when the engine surprises you).","inputSchema":{"type":"object","properties":{"topic":{"type":"string"}}}}
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
        return $"reindexed: {c.symbols} symbols, {c.scripts} scripts, {c.products} products, {c.channels} channel rows";
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
