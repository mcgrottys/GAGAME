using System.Globalization;
using System.Text;

namespace Gagame.Harvest.Sources;

/// What every ERDDAP server answers alike (docs/FIDELITY.md 8): `info/<dataset>/index.csv` (one
/// row per variable and per attribute: units, standard_name, _FillValue ...), the `.csv` answers
/// of tabledap (row 1 the names, row 2 the units, ISO 8601 times, NaN where missing), and the
/// request grammar, percent-encoded (Tomcat refuses a bare `"`, `>`, `[`).
public static class Erddap
{
    /// The dataset's variables and their attributes, from the first copy of its info page (the
    /// metadata is the schema the config is checked against; a newer copy is never fetched).
    public static Dictionary<string, Dictionary<string, string>> Info(SourceEnv env, string server, string dataset, string cacheDir)
    {
        var (text, _) = env.Polite.GetTextP($"{server}/info/{dataset}/index.csv", Path.Combine(cacheDir, "info.csv"), note: "erddap info");
        var vars = new Dictionary<string, Dictionary<string, string>>(StringComparer.Ordinal);
        foreach (var r in Csv(text).Skip(1))
        {
            if (r.Length < 5) continue;
            if (r[0] is "variable" or "dimension") vars[r[1]] = new() { ["_type"] = r[3] };   // a grid's axes are its dimensions
            else if (r[0] == "attribute" && r[1] != "NC_GLOBAL" && vars.TryGetValue(r[1], out var a)) a[r[2]] = r[4];
        }
        if (vars.Count == 0) throw new InvalidDataException($"{server}/info/{dataset}: no variables in the answer");
        return vars;
    }

    /// RFC 4180 rows: commas, quotes with "" inside them, newlines inside quotes.
    public static List<string[]> Csv(string text)
    {
        var rows = new List<string[]>();
        var row = new List<string>();
        var f = new StringBuilder();
        var q = false;
        for (var i = 0; i < text.Length; i++)
        {
            var ch = text[i];
            if (q)
            {
                if (ch == '"' && i + 1 < text.Length && text[i + 1] == '"') { f.Append('"'); i++; }
                else if (ch == '"') q = false;
                else f.Append(ch);
            }
            else if (ch == '"') q = true;
            else if (ch == ',') { row.Add(f.ToString()); f.Clear(); }
            else if (ch == '\n')
            {
                row.Add(f.ToString().TrimEnd('\r')); f.Clear();
                rows.Add(row.ToArray()); row.Clear();
            }
            else f.Append(ch);
        }
        if (f.Length > 0 || row.Count > 0) { row.Add(f.ToString()); rows.Add(row.ToArray()); }
        return rows;
    }

    /// One constraint or projection, percent-encoded except the operator's `=`.
    public static string Q(string s) => Uri.EscapeDataString(s).Replace("%3D", "=");

    public static string F(double v) => v.ToString("R", CultureInfo.InvariantCulture);

    public static double? Num(string s) => double.TryParse(s, NumberStyles.Float, CultureInfo.InvariantCulture, out var v) && !double.IsNaN(v) ? v : null;

    /// An answer with no rows is HTTP 404 ("Your query produced no matching results"): kept as a
    /// first copy holding only the names, so a rerun does not ask again. Any other 404 (a dataset
    /// the server does not hold now) is an error, never cached.
    public static string GetOrEmpty(SourceEnv env, string url, string cachePath, string header, string note)
    {
        if (File.Exists(cachePath)) return File.ReadAllText(cachePath, Encoding.UTF8);
        try { return env.Polite.GetTextP(url, cachePath, note: note).text; }
        catch (HttpError e) when (e.Code == 404 && e.Body.Contains("no matching results", StringComparison.OrdinalIgnoreCase))
        {
            Directory.CreateDirectory(Path.GetDirectoryName(cachePath)!);
            var empty = header + "\n\n";
            File.WriteAllText(cachePath, empty, Encoding.UTF8);
            env.Log($"    {Polite.HostOf(url)}: no rows for {url} (kept as an empty first copy)");
            return empty;
        }
    }
}
