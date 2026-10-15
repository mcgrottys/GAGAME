using System.Diagnostics;
using System.Globalization;
using System.Text.RegularExpressions;
using static Gagame.Harvest.JsonUtil;

namespace Gagame.Harvest;

/// What every composite harvester shares (docs/FIDELITY.md section 3, GACOMP01): the painted
/// regions of one layer, the run's measurements (seconds, bytes in and out, requests, peak
/// memory), and the product document. A composite names no place: a region is a cell of the
/// fidelity map, and what lies there is what the sources say.
public sealed class CompositeRun
{
    public const string Format = "GACOMP01";

    public readonly HarvestContext Ctx;
    public readonly Polite Polite;
    public readonly FidelityMap Map;
    public readonly string MapPath, OutDir, Layer, Kind;
    readonly Stopwatch _clock = Stopwatch.StartNew();
    readonly long _bytes0;
    readonly int _requests0;
    long _bytesIn, _bytesOut;
    public readonly List<Dictionary<string, object?>> Inputs = new();
    public readonly List<Dictionary<string, object?>> Coverage = new();
    public readonly Dictionary<string, object?> Extra = new();

    public CompositeRun(HarvestContext ctx, Polite polite, string mapPath, string outDir, string layer, string kind)
    {
        Ctx = ctx;
        Polite = polite;
        MapPath = mapPath;
        Map = FidelityMap.Load(mapPath);
        OutDir = outDir;
        Layer = layer;
        Kind = kind;
        var s = polite.Summary();
        _bytes0 = (long)s["bytes"]!;
        _requests0 = (int)s["requests"]!;
        Directory.CreateDirectory(outDir);
    }

    /// `--fidelity FILE --out DIR` from the arguments, refused when either is missing.
    public static (string map, string outDir) Paths(Args a, string name)
    {
        var map = a.Opt("--fidelity") ?? throw new HarvestStop($"{name} needs --fidelity FILE.gafid");
        var outDir = a.Opt("--out") ?? throw new HarvestStop($"{name} needs --out data/composite/<name>/");
        if (!File.Exists(map)) throw new HarvestStop($"{map}: no such fidelity map");
        return (map, outDir);
    }

    public void Log(string m) => Ctx.Log($"[{Layer}] {m}");
    public void ReadBytes(long n) => Interlocked.Add(ref _bytesIn, n);
    public void WroteBytes(long n) => Interlocked.Add(ref _bytesOut, n);
    public double Seconds => _clock.Elapsed.TotalSeconds;

    /// The painted regions of this layer, finest first, each a lon/lat box in [-180, 180]
    /// (a cell across the antimeridian is split in two) with its grain.
    public List<(Box box, double grainM)> Regions()
    {
        var out_ = new List<(Box, double)>();
        foreach (var (_, g, b) in Map.Regions(Layer))
            foreach (var box in Split(b)) out_.Add((box, g));
        return out_;
    }

    public double DefaultGrainM => Map.Get(Layer).DefaultGrainM;

    static IEnumerable<Box> Split((double lon0, double lat0, double lon1, double lat1) b)
    {
        double lon0 = b.lon0, lon1 = b.lon1;
        if (lon1 - lon0 >= 360) { yield return new Box(-180, b.lat0, 180, b.lat1); yield break; }
        var shift = 360.0 * Math.Floor((lon0 + 180.0) / 360.0);
        lon0 -= shift; lon1 -= shift;
        if (lon1 <= 180) { yield return new Box(lon0, b.lat0, lon1, b.lat1); yield break; }
        yield return new Box(lon0, b.lat0, 180, b.lat1);
        yield return new Box(-180, b.lat0, lon1 - 360, b.lat1);
    }

    public Dictionary<string, object?> Stats()
    {
        var s = Polite.Summary();
        var p = Process.GetCurrentProcess();
        p.Refresh();
        return D(("seconds", Round(Seconds, 2)), ("bytes_in", _bytesIn), ("bytes_out", _bytesOut),
            ("requests", (int)s["requests"]! - _requests0), ("bytes_fetched", (long)s["bytes"]! - _bytes0),
            ("peak_mb", Round(p.PeakWorkingSet64 / 1048576.0, 1)));
    }

    /// Writes `<layer>.composite.json` beside the product: the GACOMP01 document.
    public string Save(Dictionary<string, object?> body)
    {
        var doc = D(("format", Format), ("layer", Layer), ("kind", Kind), ("generated_utc", NowIso()),
            ("fidelity", D(("file", Rel(MapPath)), ("fnv64", Bin.Fnv1a(File.ReadAllBytes(MapPath)).ToString("x16")),
                ("layer_read", Map.Layers.Any(l => l.Name == Layer) ? Layer : FidelityMap.AllLayers),
                ("default_grain_m", double.IsInfinity(DefaultGrainM) ? null : (double?)DefaultGrainM))));
        foreach (var (k, v) in body) doc[k] = v;
        doc["inputs"] = Inputs;
        doc["coverage"] = Coverage;
        foreach (var (k, v) in Extra) doc[k] = v;
        doc["stats"] = Stats();
        var path = Path.Combine(OutDir, $"{Layer}.composite.json");
        JsonUtil.Save(path, doc);
        Log($"wrote {path}: {Dump(doc["stats"], 0)}");
        return path;
    }

    /// Repository-relative with forward slashes when under the checkout; absolute otherwise.
    public string Rel(string path)
    {
        var a = Path.GetFullPath(path);
        var root = Ctx.Root;
        return a.StartsWith(root + Path.DirectorySeparatorChar, StringComparison.OrdinalIgnoreCase)
            ? Path.GetRelativePath(root, a).Replace('\\', '/') : a.Replace('\\', '/');
    }

    /// The input entry of GACOMP01, with the moment its data is valid for and when it was
    /// fetched (Mark, 2026-10-13: the FIRST copy a cache holds is the one used; a rerun never
    /// fetches a newer forecast, observation or version).
    public Dictionary<string, object?> Input(string id, string title, double? grainM, string url, string license, string cite,
        string datum, string converted, string? validUtc = null, string? fetchedUtc = null)
    {
        var e = D(("id", id), ("title", title), ("grain_m", grainM), ("url", url), ("license", license), ("cite", cite),
            ("datum", datum), ("converted", converted), ("valid_utc", validUtc), ("fetched_utc", fetchedUtc));
        var at = Inputs.FindIndex(x => (string?)x["id"] == id);
        if (at >= 0) Inputs[at] = e; else Inputs.Add(e);
        return e;
    }

    public static string FileUtc(string path) => IsoZ(File.GetLastWriteTimeUtc(path));
}

/// THE FIRST COPY: a source whose answer changes with time (a forecast cycle, a buoy's latest
/// line, a prediction window, a nowcast, a gauge's latest value) is cached under a name carrying
/// the moment it was fetched; the composites read the EARLIEST such copy that exists and fetch
/// only when there is none (Mark, 2026-10-13). One rule for every time-varying source.
public static class FirstCopy
{
    /// The earliest file in `dir` matching `prefix` + a stamp + `suffix` (stamps sort as
    /// strings: yyyyMMdd, yyyyMMddHH), or null.
    public static string? Earliest(string dir, string prefix, string suffix)
    {
        if (!Directory.Exists(dir)) return null;
        var rx = new Regex("^" + Regex.Escape(prefix) + @"(\d{8,10})" + Regex.Escape(suffix) + "$");
        return Directory.EnumerateFiles(dir)
            .Select(p => (p, m: rx.Match(Path.GetFileName(p))))
            .Where(x => x.m.Success && new FileInfo(x.p).Length > 0)
            .OrderBy(x => x.m.Groups[1].Value, StringComparer.Ordinal)
            .Select(x => x.p).FirstOrDefault();
    }

    /// The stamp in a first-copy file name, as UTC.
    public static DateTime StampOf(string path, string prefix)
    {
        var s = Path.GetFileName(path)[prefix.Length..];
        s = new string(s.TakeWhile(char.IsDigit).ToArray());
        var fmt = s.Length == 8 ? "yyyyMMdd" : "yyyyMMddHH";
        return DateTime.ParseExact(s, fmt, CultureInfo.InvariantCulture, DateTimeStyles.AssumeUniversal | DateTimeStyles.AdjustToUniversal);
    }
}
