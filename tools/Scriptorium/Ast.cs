// ================================================================================================
//  Ast -- the Scriptorium tools over the AST index (tools/astindex writes the ast_* tables into
//  this same scriptorium.db; see tools/astindex/README.md).
//
//  The regex index answers WHERE A NAME IS DECLARED. These answer what only the compiler's
//  resolution of every name can: who calls, who writes, what offset, which shader reads which
//  component. Every query is a lookup on an indexed column; none parses anything at call time.
//
//  A name is resolved to declarations first (exact qualified name, else exact name, else a
//  '::suffix' of a qualified name), and the answer is given per declaration, so two members
//  called Update in two classes are never merged the way a grep merges them.
// ================================================================================================
using System.Text;
using System.Text.RegularExpressions;
using Microsoft.Data.Sqlite;

static class Ast
{
    const string Missing =
        "(no AST index in this database: run `py -3 tools/astindex/astindex.py` once -- the build's " +
        "post-build step keeps it current after that; see tools/astindex/README.md)";

    // A database the AST indexer never touched has no ast_* tables, and the exception would end
    // the server and every other tool with it.
    public static string Guard(Func<string> f)
    {
        try { return f(); }
        catch (SqliteException e) when (e.Message.Contains("no such table")) { return Missing; }
    }

    static List<object[]> Rows(SqliteConnection db, string sql, params (string, object)[] args)
    {
        using var cmd = db.CreateCommand();
        cmd.CommandText = sql;
        foreach (var (k, v) in args) cmd.Parameters.AddWithValue(k, v);
        using var r = cmd.ExecuteReader();
        var o = new List<object[]>();
        while (r.Read())
        {
            var row = new object[r.FieldCount];
            r.GetValues(row);
            o.Add(row);
        }
        return o;
    }

    record Decl(string Usr, string Kind, string Name, string Qual, string File, long Line, long EndLine,
                string Type, string Lang);

    // Declarations a user's word names: the qualified name exactly, else the name exactly, else a
    // '::suffix'. All redeclarations of one usr collapse to its definition (or first declaration).
    static List<Decl> Resolve(SqliteConnection db, string q, string? kindLike = null)
    {
        q = q.Trim();
        var kind = kindLike == null ? "" : " AND kind LIKE $k";
        var args = new List<(string, object)> { ("$q", q), ("$s", "%::" + q) };
        if (kindLike != null) args.Add(("$k", kindLike));
        List<object[]> rows = [];
        foreach (var where in new[] { "qual = $q", "name = $q", "qual LIKE $s" })
        {
            rows = Rows(db, $"SELECT usr, kind, name, qual, file, line, end_line, type, lang, is_def FROM ast_decls " +
                            $"WHERE {where}{kind} ORDER BY is_def DESC, file, line", args.ToArray());
            if (rows.Count > 0) break;
        }
        var seen = new HashSet<string>();
        var o = new List<Decl>();
        foreach (var r in rows)
            if (seen.Add((string)r[0]))
                o.Add(new Decl((string)r[0], (string)r[1], (string)r[2], (string)r[3], (string)r[4],
                               (long)r[5], (long)r[6], (string)r[7], (string)r[8]));
        return o;
    }

    static string QualOf(SqliteConnection db, string usr, Dictionary<string, string> cache)
    {
        if (usr.Length == 0) return "(file scope)";
        if (cache.TryGetValue(usr, out var q)) return q;
        var r = Rows(db, "SELECT qual FROM ast_decls WHERE usr = $u LIMIT 1", ("$u", usr));
        return cache[usr] = r.Count > 0 ? (string)r[0][0] : usr;
    }

    static string NotFound(string what, string q) =>
        $"(no {what} named '{q}' in the AST index; `symbols` searches the regex index by substring)";

    // ------------------------------------------------------------------ refs

    public static string Refs(SqliteConnection db, string q)
    {
        var decls = Resolve(db, q);
        if (decls.Count == 0) return NotFound("declaration", q);
        var o = new StringBuilder();
        var names = new Dictionary<string, string>();
        foreach (var d in decls.Take(12))
        {
            var rows = Rows(db, "SELECT kind, file, line, col, container, detail FROM ast_refs WHERE usr = $u " +
                                "ORDER BY file, line, col", ("$u", d.Usr));
            var byKind = rows.GroupBy(r => (string)r[0]).OrderBy(g => g.Key)
                             .Select(g => $"{g.Key} {g.Count()}");
            o.AppendLine($"{d.Kind} {d.Qual}  {d.File}:{d.Line}  [{d.Type}]");
            o.AppendLine($"  {rows.Count} references: {string.Join(", ", byKind)}");
            foreach (var r in rows.Take(150))
            {
                var detail = ((string)r[5]).Length > 0 ? $".{r[5]}" : "";
                o.AppendLine($"  {r[0] + detail,-10} {r[1]}:{r[2]}:{r[3]}  in {QualOf(db, (string)r[4], names)}");
            }
            if (rows.Count > 150) o.AppendLine($"  ... and {rows.Count - 150} more");
        }
        if (decls.Count > 12) o.AppendLine($"... and {decls.Count - 12} more declarations; qualify the name");
        return o.ToString();
    }

    // ------------------------------------------------------------------ callers / callees

    static List<Decl> Functions(SqliteConnection db, string q) =>
        Resolve(db, q).Where(d => d.Kind is "function_decl" or "cxx_method" or "constructor" or "destructor"
                                     or "conversion_function" or "function_template" or "hlsl-fn").ToList();

    public static string Callers(SqliteConnection db, string q)
    {
        var fns = Functions(db, q);
        if (fns.Count == 0) return NotFound("function", q);
        var o = new StringBuilder();
        var names = new Dictionary<string, string>();
        foreach (var d in fns.Take(12))
        {
            var rows = Rows(db, "SELECT container, COUNT(*), MIN(file || ':' || printf('%06d', line)) FROM ast_refs " +
                                "WHERE usr = $u AND kind IN ('call', 'addr') GROUP BY container", ("$u", d.Usr));
            o.AppendLine($"{d.Qual} {d.Type}  {d.File}:{d.Line}  -- {rows.Count} callers");
            foreach (var r in rows.Select(r => (who: QualOf(db, (string)r[0], names), n: (long)r[1], at: (string)r[2]))
                                  .OrderBy(x => x.who, StringComparer.Ordinal))
                o.AppendLine($"  {r.who,-60} x{r.n}  {Site(r.at)}");
        }
        return o.ToString();
    }

    public static string Callees(SqliteConnection db, string q)
    {
        var fns = Functions(db, q);
        if (fns.Count == 0) return NotFound("function", q);
        var o = new StringBuilder();
        foreach (var d in fns.Take(12))
        {
            var rows = Rows(db,
                "SELECT r.usr, COUNT(*), MIN(r.file || ':' || printf('%06d', r.line)), " +
                "COALESCE((SELECT qual || '  ' || file || ':' || line FROM ast_decls x WHERE x.usr = r.usr ORDER BY is_def DESC LIMIT 1), " +
                "MIN(r.name) || '  (implicit or compiler-declared)') " +
                "FROM ast_refs r WHERE r.container = $u AND r.kind = 'call' GROUP BY r.usr", ("$u", d.Usr));
            o.AppendLine($"{d.Qual} {d.Type}  {d.File}:{d.Line}  -- calls {rows.Count} engine functions");
            foreach (var r in rows.OrderBy(r => r[3] as string ?? (string)r[0], StringComparer.Ordinal))
                o.AppendLine($"  {r[3] as string ?? (string)r[0],-90} x{r[1]}  first at {Site((string)r[2])}");
        }
        return o.ToString();
    }

    static string Site(string fileLine)
    {
        var i = fileLine.LastIndexOf(':');
        return i < 0 ? fileLine : $"{fileLine[..i]}:{long.Parse(fileLine[(i + 1)..])}";
    }

    // ------------------------------------------------------------------ layout

    public record Field(string Name, string Type, long Offset, long Size) { public string Key { get; set; } = ""; }
    public record Rec(string Usr, string Name, string Qual, string File, long Line, long Size, long Align, string Lang,
                  List<Field> Fields);

    static List<Rec> Layouts(SqliteConnection db, string where, params (string, object)[] args)
    {
        var o = new List<Rec>();
        foreach (var r in Rows(db, $"SELECT type_usr, name, qual, file, line, size, align, lang FROM ast_layouts WHERE {where} " +
                                   "ORDER BY lang, file, line", args))
        {
            var fields = Rows(db, "SELECT name, type, offset, size FROM ast_fields WHERE type_usr = $u AND lang = $l ORDER BY ord",
                              ("$u", r[0]), ("$l", r[7]))
                .Select(f => new Field((string)f[0], (string)f[1], (long)f[2], (long)f[3])).ToList();
            Keys(fields);
            o.Add(new Rec((string)r[0], (string)r[1], (string)r[2], (string)r[3], (long)r[4], (long)r[5], (long)r[6],
                             (string)r[7], fields));
        }
        return o;
    }

    // gSunDir, m_sunDir, sunDir and SunDir are one name across the C++ / HLSL line.
    public static string Norm(string n)
    {
        if (n.StartsWith("m_")) n = n[2..];
        else if (n.Length > 1 && n[0] == 'g' && char.IsUpper(n[1])) n = n[1..];
        return n.ToLowerInvariant();
    }

    // The name a field is matched by: Norm, less the prefix every field of its record shares
    // (SurfaceCb's gCsU, gCsF, ... are ComposedSurfaceCb's u, f, ...).
    static void Keys(List<Field> fields)
    {
        var n = fields.Select(f => Norm(f.Name)).ToList();
        var p = n.Count < 3 ? "" : n.Aggregate((x, y) =>
        {
            var i = 0;
            while (i < x.Length && i < y.Length && x[i] == y[i]) i++;
            return x[..i];
        });
        if (p.Length < 2 || n.Any(k => k.Length <= p.Length)) p = "";
        for (var i = 0; i < fields.Count; i++) fields[i].Key = n[i][p.Length..];
    }

    static double Overlap(Rec a, Rec b)
    {
        var x = a.Fields.Select(f => f.Key).ToHashSet();
        var y = b.Fields.Select(f => f.Key).ToHashSet();
        if (x.Count == 0 || y.Count == 0) return 0;
        // Jaccard, not the smaller side's share: a 9-field cbuffer must not pick a 45-field struct
        // that happens to hold 5 of its names (measured: BankRingCb took BankCbData that way).
        return (double)x.Intersect(y).Count() / x.Union(y).Count();
    }

    // Names first, then the bytes: equal size (in 16-byte rows) outweighs a few names, which is
    // what tells BedRows (336 bytes) from ChainCb (16) as the mirror of a 336-byte struct.
    static double Score(Rec a, Rec b) => Overlap(a, b) + ((a.Size + 15) / 16 == (b.Size + 15) / 16 ? 0.5 : 0);

    // The mirror of a layout on the other side of the C++ / HLSL line: the same name, else the
    // layout sharing the most field names (Jaccard 0.4 or more, and at least three: with two,
    // every pair of 'b'/'s' records in the engine matched).
    public static (Rec? mirror, string how) Mirror(List<Rec> others, Rec l)
    {
        // The same name counts only with a field in common or the same size: two Bands, one CPU
        // ocean band and one mesh band, share a name and nothing else.
        var same = others.FirstOrDefault(o => o.Name == l.Name && (o.Size == l.Size || Overlap(l, o) > 0));
        if (same != null) return (same, "same name");
        var best = others.Select(o => (o, s: Score(l, o)))
                         .Where(t => Overlap(l, t.o) >= 0.4 && t.o.Fields.Select(f => f.Key).Intersect(l.Fields.Select(f => f.Key)).Count() >= 3)
                         .OrderByDescending(t => t.s).ThenBy(t => t.o.Name, StringComparer.Ordinal).FirstOrDefault();
        return best.o != null ? (best.o, $"{Overlap(l, best.o):P0} of field names shared") : (null, "");
    }

    public static string Layout(SqliteConnection db, string q)
    {
        q = q.Trim();
        var mine = Layouts(db, "name = $q OR qual = $q", ("$q", q));
        if (mine.Count == 0) mine = Layouts(db, "qual LIKE $s", ("$s", "%::" + q));
        if (mine.Count == 0) return NotFound("laid-out record (a complete, non-template struct, class, cbuffer)", q);
        var o = new StringBuilder();
        foreach (var l in mine.Take(6))
        {
            var otherSide = l.Lang == "cpp"
                ? Layouts(db, "lang LIKE 'hlsl%'")
                : Layouts(db, "lang = 'cpp' AND file NOT LIKE 'build/%'");
            if (l.Lang != "cpp" && otherSide.Any(c => c.Name == l.Name) == false)
                otherSide = otherSide.Where(c => c.Fields.Count > 0).ToList();
            var (m, how) = Mirror(otherSide, l);
            // Mutual best only: one C++ struct mirrors one HLSL record. ProbeBedChain::Cb is
            // BedRows's mirror (40 % of names), and ChainCb, sharing 3 of its names, took it too.
            if (m != null)
            {
                var sameSide = l.Lang == "cpp" ? Layouts(db, "lang = 'cpp'") : Layouts(db, "lang LIKE 'hlsl%'");
                var (back, _) = Mirror(sameSide, m);
                if (back != null && back.Usr != l.Usr && Score(back, m) > Score(l, m))
                {
                    how = "";
                    o.AppendLine($"(its nearest match, {m.Qual}, mirrors {back.Qual} more closely)");
                    m = null;
                }
            }
            o.AppendLine(Describe(l));
            if (m == null)
            {
                foreach (var f in l.Fields) o.AppendLine($"  {f.Offset,6} {f.Size,5}  {f.Name,-28} {f.Type}");
                o.AppendLine("  (no mirror on the other side of the C++ / HLSL line)");
                o.AppendLine();
                continue;
            }
            o.AppendLine($"mirror ({how}): {Describe(m)}");
            o.Append(Compare(l, m));
            o.AppendLine();
        }
        return o.ToString();
    }

    static string Describe(Rec l) =>
        $"{l.Lang} {l.Qual}  {l.File}:{l.Line}  size {l.Size} align {l.Align}  ({l.Fields.Count} fields)";

    // Side by side by normalized name, in the first layout's order; then the mirror's leftovers.
    public static string Compare(Rec a, Rec b)
    {
        var o = new StringBuilder();
        o.AppendLine($"  {"field",-28} {a.Lang + " off/size",-16} {b.Lang + " off/size",-20} verdict");
        var bm = b.Fields.GroupBy(f => f.Key).ToDictionary(g => g.Key, g => g.First());
        var used = new HashSet<string>();
        int bad = 0;
        var unnamed = b.Fields.Where(g => !a.Fields.Any(f => f.Key == g.Key)).ToList();
        foreach (var f in a.Fields)
        {
            var k = f.Key;
            if (bm.TryGetValue(k, out var g))
            {
                used.Add(k);
                var v = g.Offset != f.Offset ? "OFFSET DIFFERS" : g.Size != f.Size ? "size differs" : "ok";
                if (v != "ok") bad++;
                o.AppendLine($"  {f.Name,-28} {f.Offset + "/" + f.Size,-16} {g.Offset + "/" + g.Size + " " + g.Name,-20} {v}");
                continue;
            }
            // No name in common. One field of the other side holding this one's bytes (lvl00..lvl11
            // here, one float4 lvl there) ...
            var holder = unnamed.FirstOrDefault(h => h.Offset <= f.Offset && f.Offset + f.Size <= h.Offset + h.Size &&
                                                      h.Size > f.Size);
            if (holder != null)
            {
                used.Add(holder.Key);
                o.AppendLine($"  {f.Name,-28} {f.Offset + "/" + f.Size,-16} {"in " + holder.Name + " " + holder.Offset + "/" + holder.Size,-20} same bytes, split");
                continue;
            }
            // ... or the other side's unmatched fields lying inside this one's bytes (a renamed
            // slot, or a nested struct on one side spread into rows on the other).
            var inside = unnamed.Where(h => h.Offset >= f.Offset && h.Offset < f.Offset + f.Size).ToList();
            if (inside.Count == 0)
            {
                o.AppendLine($"  {f.Name,-28} {f.Offset + "/" + f.Size,-16} {"-",-20} only in {a.Lang}");
                continue;
            }
            foreach (var h in inside) used.Add(h.Key);
            var end = inside.Max(h => h.Offset + h.Size);
            var tiles = inside[0].Offset == f.Offset && end == f.Offset + f.Size;
            if (!tiles) bad++;
            var label = inside.Count == 1 ? inside[0].Offset + "/" + inside[0].Size + " " + inside[0].Name
                                          : $"{inside.Count} fields {inside[0].Name}..{inside[^1].Name}";
            o.AppendLine($"  {f.Name,-28} {f.Offset + "/" + f.Size,-16} {label,-20} " +
                         (tiles ? (inside.Count == 1 ? "same bytes, renamed" : "same bytes, split") : "BYTES DIFFER"));
        }
        foreach (var g in b.Fields.Where(g => !used.Contains(g.Key)))
            o.AppendLine($"  {g.Name,-28} {"-",-16} {g.Offset + "/" + g.Size,-20} only in {b.Lang}");
        // A cbuffer occupies whole 16-byte rows: 60 bytes of root constants fill a 64-byte FloorCb.
        static long Rows16(long n) => (n + 15) / 16 * 16;
        var cb = a.Lang == "hlsl-cbuffer" || b.Lang == "hlsl-cbuffer";
        var sizeNote = a.Size == b.Size ? "sizes equal"
                     : cb && Rows16(a.Size) == Rows16(b.Size) ? $"sizes equal in 16-byte rows ({a.Size} vs {b.Size})"
                     : $"SIZES DIFFER ({a.Size} vs {b.Size})";
        o.AppendLine($"  => {bad} matched fields disagree; {sizeNote}");
        return o.ToString();
    }

    // ------------------------------------------------------------------ cbuffer_reads

    public static string CbufferReads(SqliteConnection db, string q)
    {
        q = q.Trim();
        var o = new StringBuilder();
        var fnNames = new Dictionary<string, string>();
        string FnName(string usr) => usr.StartsWith("hlsl:") ? usr[5..] : QualOf(db, usr, fnNames);

        // a field, optionally with a component: gWavesB.z / SceneCb.gMisc / gMisc
        var m = Regex.Match(q, @"^(?:(\w+)\.)?(\w+)(?:\.([xyzwrgba]{1,4}|\w+))?$");
        if (m.Success)
        {
            var a = m.Groups[1].Value; var b = m.Groups[2].Value; var c = m.Groups[3].Value;
            // 'SceneCb' alone, or 'SceneCb.gMisc'
            var cbRows = Rows(db, "SELECT usr, qual FROM ast_decls WHERE kind='hlsl-cbfield' AND parent_usr = $p ORDER BY line",
                              ("$p", "hlsl:cbuffer:" + (a.Length > 0 ? a : b)));
            List<object[]> fields;
            string comp = c;
            if (cbRows.Count > 0 && a.Length == 0)
                fields = cbRows;                                             // a whole cbuffer
            else if (cbRows.Count > 0)
                fields = cbRows.Where(r => ((string)r[1]).EndsWith("." + b)).ToList();
            else
            {
                // 'gWavesB.z': a = field, b = component
                var field = a.Length > 0 ? a : b;
                comp = a.Length > 0 ? b + (c.Length > 0 ? "." + c : "") : c;
                fields = Rows(db, "SELECT usr, qual FROM ast_decls WHERE kind IN ('hlsl-cbfield','hlsl-global') AND name = $n",
                              ("$n", field));
            }
            if (fields.Count > 0)
            {
                foreach (var f in fields)
                {
                    var rows = Rows(db, "SELECT detail, container, file, line FROM ast_refs WHERE usr = $u AND kind='read' " +
                                        "ORDER BY detail, container, file, line", ("$u", f[0]));
                    if (comp.Length > 0)
                        rows = rows.Where(r => ComponentHits((string)r[0], comp)).ToList();
                    o.AppendLine($"{f[1]}{(comp.Length > 0 ? " component " + comp : "")}: {rows.Count} reads in " +
                                 $"{rows.Select(r => r[1]).Distinct().Count()} functions");
                    foreach (var g in rows.GroupBy(r => (string)r[0]).OrderBy(g => g.Key, StringComparer.Ordinal))
                    {
                        var label = g.Key.Length == 0 ? "(whole)" : g.Key;
                        o.AppendLine($"  .{label,-6} " + string.Join(", ",
                            g.GroupBy(r => (string)r[1]).Select(h => $"{FnName(h.Key).Split(' ')[0]} ({h.First()[2]}:{h.First()[3]})")));
                    }
                    if (rows.Count == 0) o.AppendLine("  (read by no function: dead in every shader)");
                }
                return o.ToString();
            }
        }

        // a function: what it and everything it calls reads, and which entries reach it
        var fns = Rows(db, "SELECT usr FROM ast_decls WHERE kind='hlsl-fn' AND name = $n ORDER BY file, line", ("$n", q));
        if (fns.Count == 0) return $"(no cbuffer field, cbuffer or HLSL function named '{q}')";
        foreach (var fr in fns.Take(8))
        {
            var root = (string)fr[0];
            var seen = new HashSet<string> { root };
            var todo = new Stack<string>(seen);
            while (todo.Count > 0)
                foreach (var r in Rows(db, "SELECT DISTINCT usr FROM ast_refs WHERE container = $u AND kind='call' AND lang='hlsl'",
                                       ("$u", todo.Pop())))
                    if (seen.Add((string)r[0])) todo.Push((string)r[0]);
            var entries = Rows(db, "SELECT DISTINCT entry_file || ' ' || entry FROM ast_hlsl_reach WHERE usr = $u ORDER BY 1", ("$u", root))
                .Select(r => (string)r[0]).ToList();
            o.AppendLine($"{root[5..]}  -- {seen.Count - 1} functions under it; reached by {entries.Count} entries: " +
                         (entries.Count > 0 ? string.Join(", ", entries.Take(12)) : "(none: dead code)"));
            var reads = new SortedDictionary<string, SortedSet<string>>(StringComparer.Ordinal);
            foreach (var u in seen)
                foreach (var r in Rows(db, "SELECT r.name, r.detail, d.qual FROM ast_refs r JOIN ast_decls d ON d.usr = r.usr " +
                                           "WHERE r.container = $u AND r.kind='read' AND r.lang='hlsl' AND d.kind='hlsl-cbfield'", ("$u", u)))
                {
                    var key = (string)r[2];
                    if (!reads.TryGetValue(key, out var set)) reads[key] = set = new SortedSet<string>(StringComparer.Ordinal);
                    set.Add(((string)r[1]).Length == 0 ? "(whole)" : (string)r[1]);
                }
            foreach (var (k, v) in reads) o.AppendLine($"  {k,-34} {string.Join(" ", v)}");
            if (reads.Count == 0) o.AppendLine("  (reads no cbuffer field)");
        }
        return o.ToString();
    }

    // Does a recorded read component cover the asked one? 'xyz' covers 'z'; '' (whole) covers all.
    static bool ComponentHits(string read, string asked)
    {
        if (read.Length == 0) return true;
        if (asked.StartsWith('.') || read.StartsWith('.') || read.StartsWith('[')) return read.Contains(asked);
        var sw = read.TrimStart('[', ']');
        string Canon(string s) => s.Replace('r', 'x').Replace('g', 'y').Replace('b', 'z').Replace('a', 'w');
        return Canon(asked).All(ch => Canon(sw).Contains(ch));
    }

    // ------------------------------------------------------------------ includes

    public static string Includes(SqliteConnection db, string q)
    {
        q = q.Trim().Replace('\\', '/');
        var files = Rows(db, "SELECT DISTINCT file FROM ast_includes WHERE file LIKE $q UNION " +
                             "SELECT DISTINCT included FROM ast_includes WHERE included LIKE $q ORDER BY 1", ("$q", "%" + q + "%"))
            .Select(r => (string)r[0]).Where(f => !f.Contains(':')).ToList();   // repo files only
        if (files.Count == 0) return $"(no repo file matching '{q}' in the include graph)";
        var o = new StringBuilder();
        foreach (var f in files.Take(6))
        {
            var inc = Rows(db, "SELECT included FROM ast_includes WHERE file = $f ORDER BY included LIKE '%:%', included", ("$f", f))
                .Select(r => (string)r[0]).ToList();
            var by = Rows(db, "SELECT file FROM ast_includes WHERE included = $f ORDER BY file", ("$f", f))
                .Select(r => (string)r[0]).ToList();
            var tus = Rows(db, "SELECT COUNT(*) FROM ast_tus WHERE deps LIKE $p", ("$p", "%" + f.Replace('/', '\\').ToLowerInvariant().Replace("\\", "\\\\") + "%"));
            o.AppendLine($"{f}");
            o.AppendLine($"  includes ({inc.Count}): {string.Join(", ", inc.Select(Short))}");
            o.AppendLine($"  included by ({by.Count}): {string.Join(", ", by)}");
            if (!f.StartsWith("shaders/")) o.AppendLine($"  TUs that read it, directly or not: {tus[0][0]}");
        }
        if (files.Count > 6) o.AppendLine($"... and {files.Count - 6} more files match");
        return o.ToString();
    }

    static string Short(string f) => f.Contains(':') ? "<" + Path.GetFileName(f) + ">" : f;
}
