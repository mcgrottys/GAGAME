using System.Globalization;
using System.Text;
using System.Text.Encodings.Web;
using System.Text.Json;
using System.Text.Json.Nodes;
using System.Runtime.CompilerServices;

namespace Gagame.Harvest;

/// The harvesters' JSON: documents built as ordered dictionaries (the Python dicts), written
/// with one-space indent as json.dump(indent=1) did, NaN as the Python `NaN` token, and read
/// back as JsonNode. Values are plain CLR: string, bool, int/long/double, null, lists, dicts.
public static class JsonUtil
{
    static readonly JsonSerializerOptions Read = new() { AllowTrailingCommas = true, ReadCommentHandling = JsonCommentHandling.Skip };

    public static JsonNode? Parse(string text) => JsonNode.Parse(text, null, new JsonDocumentOptions { AllowTrailingCommas = true, CommentHandling = JsonCommentHandling.Skip });

    public static JsonNode? Load(string path) => Parse(File.ReadAllText(path, Encoding.UTF8));

    public static JsonNode? TryLoad(string path)
    {
        try { return File.Exists(path) ? Load(path) : null; }
        catch (Exception) { return null; }
    }

    /// Writes `doc` with the Python indent=1 layout. `exclusive` refuses to replace a file.
    public static void Save(string path, object? doc, bool exclusive = false, int indent = 1)
    {
        var text = Dump(doc, indent);
        Directory.CreateDirectory(Path.GetDirectoryName(Path.GetFullPath(path))!);
        using var f = new FileStream(path, exclusive ? FileMode.CreateNew : FileMode.Create, FileAccess.Write);
        var b = new UTF8Encoding(false).GetBytes(text);
        f.Write(b);
    }

    public static string Dump(object? doc, int indent = 1)
    {
        var sb = new StringBuilder();
        Write(sb, doc, indent, 0);
        return sb.ToString();
    }

    static void Write(StringBuilder sb, object? v, int indent, int depth)
    {
        switch (v)
        {
            case null: sb.Append("null"); break;
            case bool b: sb.Append(b ? "true" : "false"); break;
            case string s: WriteString(sb, s); break;
            case int i: sb.Append(i.ToString(CultureInfo.InvariantCulture)); break;
            case long l: sb.Append(l.ToString(CultureInfo.InvariantCulture)); break;
            case uint ui: sb.Append(ui.ToString(CultureInfo.InvariantCulture)); break;
            case ulong ul: sb.Append(ul.ToString(CultureInfo.InvariantCulture)); break;
            case short sh: sb.Append(sh.ToString(CultureInfo.InvariantCulture)); break;
            case byte by: sb.Append(by.ToString(CultureInfo.InvariantCulture)); break;
            case float f: WriteDouble(sb, f); break;
            case double d: WriteDouble(sb, d); break;
            case decimal m: sb.Append(m.ToString(CultureInfo.InvariantCulture)); break;
            case JsonNode n: sb.Append(n.ToJsonString(new JsonSerializerOptions { WriteIndented = false, Encoder = JavaScriptEncoder.UnsafeRelaxedJsonEscaping })); break;
            case Box bx: Write(sb, bx.ToJson(), indent, depth); break;
            case System.Collections.IDictionary dict:
                {
                    if (dict.Count == 0) { sb.Append("{}"); break; }
                    sb.Append('{');
                    var first = true;
                    foreach (System.Collections.DictionaryEntry kv in dict)
                    {
                        if (!first) sb.Append(',');
                        first = false;
                        NewLine(sb, indent, depth + 1);
                        WriteString(sb, kv.Key?.ToString() ?? "null");
                        sb.Append(": ");
                        Write(sb, kv.Value, indent, depth + 1);
                    }
                    NewLine(sb, indent, depth);
                    sb.Append('}');
                    break;
                }
            case System.Collections.IEnumerable list:
                {
                    var items = list.Cast<object?>().ToList();
                    if (items.Count == 0) { sb.Append("[]"); break; }
                    sb.Append('[');
                    var first = true;
                    foreach (var it in items)
                    {
                        if (!first) sb.Append(',');
                        first = false;
                        NewLine(sb, indent, depth + 1);
                        Write(sb, it, indent, depth + 1);
                    }
                    NewLine(sb, indent, depth);
                    sb.Append(']');
                    break;
                }
            default:
                if (v is ITuple t) { Write(sb, Enumerable.Range(0, t.Length).Select(i => t[i]).ToList(), indent, depth); break; }
                WriteString(sb, v.ToString() ?? "");
                break;
        }
    }

    static void NewLine(StringBuilder sb, int indent, int depth)
    {
        if (indent <= 0) { if (depth > 0) sb.Append(' '); return; }
        sb.Append('\n');
        sb.Append(' ', indent * depth);
    }

    /// Python's float repr: the shortest round-trip, `1.0` for integral values, `NaN`/`Infinity`.
    public static void WriteDouble(StringBuilder sb, double d)
    {
        if (double.IsNaN(d)) { sb.Append("NaN"); return; }
        if (double.IsPositiveInfinity(d)) { sb.Append("Infinity"); return; }
        if (double.IsNegativeInfinity(d)) { sb.Append("-Infinity"); return; }
        var s = d.ToString("R", CultureInfo.InvariantCulture);
        if (s.Contains('E'))
        {
            // Python prints 1e-05 and 1e+16 for the extremes; keep the digits, Python's exponent form.
            var parts = s.Split('E');
            var exp = int.Parse(parts[1], CultureInfo.InvariantCulture);
            s = parts[0] + "e" + (exp < 0 ? "-" : "+") + Math.Abs(exp).ToString("00", CultureInfo.InvariantCulture);
        }
        else if (!s.Contains('.')) s += ".0";
        sb.Append(s);
    }

    static void WriteString(StringBuilder sb, string s)
    {
        sb.Append('"');
        foreach (var c in s)
        {
            switch (c)
            {
                case '"': sb.Append("\\\""); break;
                case '\\': sb.Append("\\\\"); break;
                case '\n': sb.Append("\\n"); break;
                case '\r': sb.Append("\\r"); break;
                case '\t': sb.Append("\\t"); break;
                default:
                    if (c < 0x20 || c > 0x7e) sb.Append("\\u").Append(((int)c).ToString("x4"));
                    else sb.Append(c);
                    break;
            }
        }
        sb.Append('"');
    }

    /// Python's round(x, n): half to even on the decimal representation.
    public static double Round(double x, int digits) => Math.Round(x, digits, MidpointRounding.ToEven);

    /// datetime.now(timezone.utc).isoformat(timespec="seconds")
    public static string NowIso() => DateTime.UtcNow.ToString("yyyy-MM-dd'T'HH:mm:ss'+00:00'", CultureInfo.InvariantCulture);
    public static string IsoZ(DateTime t) => t.ToUniversalTime().ToString("yyyy-MM-dd'T'HH:mm:ss'+00:00'", CultureInfo.InvariantCulture);
    public static string NowStampZ() => DateTime.UtcNow.ToString("yyyy-MM-dd'T'HH:mm:ss'Z'", CultureInfo.InvariantCulture);

    public static double Unix(DateTime utc) => (utc - DateTime.UnixEpoch).TotalSeconds;
    public static DateTime FromUnix(double s) => DateTime.UnixEpoch.AddSeconds(s);

    // ---- readers over JsonNode, tolerant like dict.get()

    public static string? Str(this JsonNode? n, string key, string? dflt = null) =>
        n is JsonObject o && o.TryGetPropertyValue(key, out var v) && v is JsonValue jv ? (jv.TryGetValue<string>(out var s) ? s : jv.ToString()) : dflt;

    public static double Num(this JsonNode? n, string key, double dflt = 0) =>
        n is JsonObject o && o.TryGetPropertyValue(key, out var v) ? ToDouble(v, dflt) : dflt;

    public static double? NumOrNull(this JsonNode? n, string key) =>
        n is JsonObject o && o.TryGetPropertyValue(key, out var v) && v is not null ? ToDouble(v, double.NaN) is var d && !double.IsNaN(d) ? d : null : null;

    public static double ToDouble(JsonNode? v, double dflt)
    {
        if (v is not JsonValue jv) return dflt;
        if (jv.TryGetValue<double>(out var d)) return d;
        if (jv.TryGetValue<string>(out var s) && double.TryParse(s, NumberStyles.Float, CultureInfo.InvariantCulture, out d)) return d;
        return dflt;
    }

    public static JsonArray Arr(this JsonNode? n, string key) =>
        n is JsonObject o && o.TryGetPropertyValue(key, out var v) && v is JsonArray a ? a : new JsonArray();

    public static JsonObject Obj(this JsonNode? n, string key) =>
        n is JsonObject o && o.TryGetPropertyValue(key, out var v) && v is JsonObject a ? a : new JsonObject();

    public static bool Has(this JsonNode? n, string key) => n is JsonObject o && o.ContainsKey(key);

    /// A JsonNode as the plain CLR values Dump writes (so a loaded document can be re-emitted).
    public static object? Plain(JsonNode? n) => n switch
    {
        null => null,
        JsonObject o => o.ToDictionary(kv => kv.Key, kv => Plain(kv.Value)),
        JsonArray a => a.Select(Plain).ToList(),
        JsonValue v => v.TryGetValue<bool>(out var b) ? b
            : v.TryGetValue<long>(out var l) ? l
            : v.TryGetValue<double>(out var d) ? d
            : v.TryGetValue<string>(out var s) ? s
            : v.ToString(),
        _ => n.ToString(),
    };

    /// A new ordered dictionary (the Python dict literal).
    public static Dictionary<string, object?> D(params (string k, object? v)[] kv)
    {
        var d = new Dictionary<string, object?>();
        foreach (var (k, v) in kv) d[k] = v;
        return d;
    }
}
