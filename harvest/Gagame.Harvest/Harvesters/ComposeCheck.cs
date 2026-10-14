using System.Globalization;
using System.Text.Json.Nodes;
using Gagame.Harvest.Readers;
using static Gagame.Harvest.JsonUtil;

namespace Gagame.Harvest.Harvesters;

/// THE COMPOSITES' GATES, run against a product folder (`--out`), each naming what it reads:
///
///   geotiff   every tier file read back with the repo's own GeoTiff reader: the GeoKeys (model
///             2, PixelIsArea, EPSG:4326, the tier's vertical CRS, metres), the nodata tag, the
///             tier's step, and data in it
///   bed       the tiers stacked as the engine stacks them (finer over coarser, the first that
///             answers): at a grid of points in every region and over the globe, the answer
///             against the tier's own source read directly -- the mean of the source pixels whose
///             centres fall inside the product pixel (one pixel at k = 0, so equal to the bit)
///   holes     at the same points, no point of a painted region unanswered
///   gap       a planted gap: a block of a finest tile set to nodata in a copy, the stack read
///             again there; the next tier must answer, and with its own source's value
///   against   `--against-tides`, `--against-currents`, `--against-sea` (comma-separated older
///             per-place files): every station or buoy in both, entry against entry, the keys the
///             old entry has (less the ones a place only knew: km_from_anchor) dumped and compared
///             as text; official sidecars compared byte for byte
///
/// Writes `<out>/check.json` and returns 1 if any gate fails.
public sealed class ComposeCheck : IHarvester
{
    public string Name => "compose-check";
    public string Title => "Composite gates";
    public string What => "Reads a composite product back and checks it against its own sources and older per-place files.";
    public string[] Outputs => [];
    public Source[] Sources => [];
    public Insert? Insert => null;

    HarvestContext _ctx = null!;
    readonly List<Dictionary<string, object?>> _gates = new();

    void Gate(string name, bool pass, string blind, Dictionary<string, object?> detail)
    {
        _gates.Add(D(("gate", name), ("pass", pass), ("blind_spot", blind), ("detail", detail)));
        _ctx.Log($"[check] {(pass ? "PASS" : "FAIL")} {name}: {Dump(detail, 0)[..Math.Min(400, Dump(detail, 0).Length)]}");
    }

    public int Run(HarvestContext ctx, string[] argv)
    {
        _ctx = ctx;
        var a = new Args(argv);
        var outDir = a.Opt("--out") ?? throw new HarvestStop("compose-check needs --out DIR");
        var bed = Path.Combine(outDir, "water.bed.composite.json");
        if (File.Exists(bed)) CheckBed(outDir, Load(bed)!);
        if (a.Opt("--against-tides") is string t) CheckTides(outDir, t.Split(','));
        if (a.Opt("--against-currents") is string c) CheckCurrents(outDir, c.Split(','));
        if (a.Opt("--against-sea") is string s) CheckSea(outDir, s.Split(','));
        Save(Path.Combine(outDir, "check.json"), D(("generated_utc", NowIso()), ("gates", _gates)));
        var failed = _gates.Count(g => !(bool)g["pass"]!);
        ctx.Log($"[check] {_gates.Count - failed} of {_gates.Count} gates pass -> {Path.Combine(outDir, "check.json")}");
        return failed == 0 ? 0 : 1;
    }

    // ------------------------------------------------------------------ the bed

    sealed class TierFile
    {
        public string Path = "";
        public double Lon0, Lat1, Sx, Sy;
        public int W, H;
        GeoTiff? _t;
        public bool Holds(double lon, double lat) => lon >= Lon0 && lon < Lon0 + W * Sx && lat <= Lat1 && lat > Lat1 - H * Sy;
        public float At(double lon, double lat, float[]? over = null)
        {
            int x = Math.Clamp((int)Math.Floor((lon - Lon0) / Sx), 0, W - 1), y = Math.Clamp((int)Math.Floor((Lat1 - lat) / Sy), 0, H - 1);
            if (over is not null) return over[y * W + x];
            lock (this) return (_t ??= new GeoTiff(File.ReadAllBytes(Path))).ReadWindow(x, y, 1, 1)[0];
        }
        public (double lon0, double lon1, double lat0, double lat1) Pixel(double lon, double lat)
        {
            int x = (int)Math.Floor((lon - Lon0) / Sx), y = (int)Math.Floor((Lat1 - lat) / Sy);
            return (Lon0 + x * Sx, Lon0 + (x + 1) * Sx, Lat1 - (y + 1) * Sy, Lat1 - y * Sy);
        }
    }

    sealed class Tier
    {
        public string Folder = "";
        public int Over;
        public List<TierFile> Files = new();
        public List<string> Sources = new();
        public TierFile? Holding(double lon, double lat) => Files.FirstOrDefault(f => f.Holds(lon, lat));
    }

    void CheckBed(string outDir, JsonNode doc)
    {
        var tiers = new List<Tier>();
        var bad = new List<string>();
        long files = 0;
        foreach (var td in doc.Arr("tiers"))
        {
            var tier = new Tier { Folder = td.Str("folder")!, Over = (int)td.Num("over"), Sources = td.Arr("source_files").Select(x => x!.GetValue<string>()).ToList() };
            var step = td.Num("step_deg");
            foreach (var f in Directory.GetFiles(Path.Combine(outDir, tier.Folder), "*.tif").OrderBy(f => f, StringComparer.Ordinal))
            {
                files++;
                var g = new GeoTiff(File.ReadAllBytes(f));
                var k = g.GeoKeys();
                bool Is(string key, int v) => k.GetValueOrDefault(key) is int i && i == v;
                var ok = Is("GTModelType", 2) && Is("GTRasterType", 1) && Is("GeographicType", 4326) && Is("VerticalCSType", (int)td.Num("vertical_epsg")) && Is("VerticalUnits", 9001)
                         && g.Nodata is double nd && nd == td.Num("nodata") && Math.Abs(g.Sx - step) < 1e-12 && Math.Abs(g.Sy - step) < 1e-12
                         && g.Compression == 8 && g.Predictor == 3 && g.Tiled && g.ReadWindow(0, 0, g.Width, g.Height).Any(v => !float.IsNaN(v));
                if (!ok) bad.Add(Path.GetFileName(f) + " " + Dump(k, 0));
                tier.Files.Add(new TierFile { Path = f, Lon0 = g.Lon0, Lat1 = g.Lat0, Sx = g.Sx, Sy = g.Sy, W = g.Width, H = g.Height });
            }
            tiers.Add(tier);
        }
        tiers = tiers.OrderByDescending(t => t.Over).ToList();
        Gate("geotiff", bad.Count == 0, "reads with this repo's reader, not the engine's WIC door (that is a separate check: WPF's TiffBitmapDecoder on a sample)",
            D(("files", files), ("bad", bad.Take(10).ToList())));

        // the sources, opened on demand
        var srcCache = new Dictionary<string, (GeoTiff t, double lonEdge, double latEdge)>();
        (GeoTiff t, double lonEdge, double latEdge)? Source(Tier tier, double lon, double lat)
        {
            foreach (var s in tier.Sources)
            {
                var p = Path.IsPathRooted(s) ? s : Path.Combine(_ctx.Root, s);
                if (!srcCache.TryGetValue(p, out var e))
                {
                    var g = new GeoTiff(File.ReadAllBytes(p));
                    var pt = g.GeoKeys().GetValueOrDefault("GTRasterType") is int rt && rt == 2;
                    srcCache[p] = e = (g, g.Lon0 - g.TieI * g.Sx - (pt ? 0.5 * g.Sx : 0), g.Lat0 + g.TieJ * g.Sy + (pt ? 0.5 * g.Sy : 0));
                }
                if (lon >= e.lonEdge && lon < e.lonEdge + e.t.Width * e.t.Sx && lat <= e.latEdge && lat > e.latEdge - e.t.Height * e.t.Sy) return e;
            }
            return null;
        }
        double Expected(Tier tier, TierFile f, double lon, double lat)
        {
            var s = Source(tier, lon, lat);
            if (s is null) return double.NaN;
            var (g, le, la) = s.Value;
            var (p0, p1, q0, q1) = f.Pixel(lon, lat);
            int x0 = (int)Math.Ceiling((p0 - le) / g.Sx - 0.5 - 1e-9), x1 = (int)Math.Ceiling((p1 - le) / g.Sx - 0.5 - 1e-9);
            int y0 = (int)Math.Ceiling((la - q1) / g.Sy - 0.5 - 1e-9), y1 = (int)Math.Ceiling((la - q0) / g.Sy - 0.5 - 1e-9);
            x0 = Math.Max(0, x0); y0 = Math.Max(0, y0); x1 = Math.Min(g.Width, x1); y1 = Math.Min(g.Height, y1);
            if (x1 <= x0 || y1 <= y0) return double.NaN;
            var w = g.ReadWindow(x0, y0, x1 - x0, y1 - y0);
            var valid = w.Where(v => !float.IsNaN(v)).ToList();
            return valid.Count == 0 ? double.NaN : valid.Average(v => (double)v);
        }
        (Tier? tier, TierFile? file, float v) Stack(double lon, double lat, (TierFile file, float[] vals)? planted = null)
        {
            foreach (var t in tiers)
            {
                var f = t.Holding(lon, lat);
                if (f is null) continue;
                var v = planted is { } pl && pl.file == f ? f.At(lon, lat, pl.vals) : f.At(lon, lat);
                if (!float.IsNaN(v)) return (t, f, v);
            }
            return (null, null, float.NaN);
        }

        // points: a 9 x 9 grid in every painted region's box (its top tier's coverage), and over the globe
        var boxes = doc.Arr("coverage").Where(c => c!["gap_fill"]!.GetValue<bool>() == false).Select(c => c!["box"]!.AsArray().Select(x => x!.GetValue<double>()).ToArray()).ToList();
        var points = new List<(double lon, double lat, bool painted)>();
        foreach (var b in boxes)
        {
            var globe = b[2] - b[0] >= 360;
            for (var i = 0; i < 9; i++)
                for (var j = 0; j < 9; j++)
                    points.Add((b[0] + (i + 0.5) / 9 * (b[2] - b[0]), b[1] + (j + 0.5) / 9 * (b[3] - b[1]), !globe));
        }
        var perTier = new Dictionary<string, (int n, double worst, int unknown)>();
        var unanswered = 0;
        foreach (var (lon, lat, painted) in points)
        {
            var (t, f, v) = Stack(lon, lat);
            if (t is null) { if (painted) unanswered++; continue; }
            var e = Expected(t, f!, lon, lat);
            var cur = perTier.GetValueOrDefault(t.Folder);
            perTier[t.Folder] = double.IsNaN(e) ? (cur.n + 1, cur.worst, cur.unknown + 1) : (cur.n + 1, Math.Max(cur.worst, Math.Abs(v - e)), cur.unknown);
        }
        Gate("bed", perTier.Values.All(x => x.worst <= 1e-3 && x.unknown == 0), "the reference stack reads the pixel holding a point (no bilinear) and the source by footprint: it checks the copy and the block mean, not the engine's own filtering; points are a 9 x 9 grid, not every pixel",
            D(("points", points.Count), ("bound_m", 1e-3), ("per_tier", perTier.ToDictionary(kv => kv.Key, kv => (object?)D(("answered", kv.Value.n), ("worst_abs_m", kv.Value.worst), ("source_not_found", kv.Value.unknown))))));
        Gate("holes", unanswered == 0, "a 9 x 9 grid per region: a hole narrower than a grid step is not seen",
            D(("painted_points", points.Count(p => p.painted)), ("unanswered", unanswered)));

        // the planted gap: the centre ninth of the first finest file set to nodata in a copy
        var fine = tiers.First();
        var victim = fine.Files.OrderByDescending(f => (long)f.W * f.H).First();
        var vals = new GeoTiff(File.ReadAllBytes(victim.Path)).ReadWindow(0, 0, victim.W, victim.H);
        int gx0 = victim.W / 3, gx1 = 2 * victim.W / 3, gy0 = victim.H / 3, gy1 = 2 * victim.H / 3;
        for (var y = gy0; y < gy1; y++) for (var x = gx0; x < gx1; x++) vals[y * victim.W + x] = float.NaN;
        int filled = 0, agree = 0, total = 0; double worstGap = 0;
        for (var i = 0; i < 7; i++)
            for (var j = 0; j < 7; j++)
            {
                var lon = victim.Lon0 + (gx0 + (i + 0.5) / 7 * (gx1 - gx0)) * victim.Sx;
                var lat = victim.Lat1 - (gy0 + (j + 0.5) / 7 * (gy1 - gy0)) * victim.Sy;
                total++;
                var (t, f, v) = Stack(lon, lat, (victim, vals));
                if (t is null || t == fine) continue;
                filled++;
                var e = Expected(t, f!, lon, lat);
                if (!double.IsNaN(e) && Math.Abs(v - e) <= 1e-3) agree++;
                if (!double.IsNaN(e)) worstGap = Math.Max(worstGap, Math.Abs(v - e));
            }
        Gate("gap", filled == total && agree == total, "the gap is planted in a copy of the PRODUCT's finest tile, not in the source: it proves the coarser tiers lie under the finest one there, as the engine's stack needs",
            D(("tile", Path.GetFileName(victim.Path)), ("tier", fine.Folder), ("points", total), ("filled_by_coarser", filled), ("equal_to_their_source", agree), ("worst_abs_m", worstGap)));
    }

    // ------------------------------------------------------------------ against the per-place files

    static string Entry(JsonNode? n, IEnumerable<string> keys) => Dump(keys.ToDictionary(k => k, k => Plain(n?[k])), 0);

    void Against(string what, string compositeFile, string[] olds, Func<JsonNode, IEnumerable<(string id, JsonNode e)>> entries, string[] drop, string blind, Action<string, JsonNode, JsonNode, List<string>>? extra = null)
    {
        var mine = entries(Load(compositeFile)!).ToDictionary(x => x.id, x => x.e);
        int common = 0, same = 0;
        var diffs = new List<string>();
        foreach (var old in olds)
            foreach (var (id, e) in entries(Load(old)!))
            {
                if (!mine.TryGetValue(id, out var m)) continue;
                common++;
                var keys = ((JsonObject)e).Select(kv => kv.Key).Where(k => !drop.Contains(k)).ToList();
                var a = Entry(e, keys);
                var b = Entry(m, keys);
                if (a == b) same++;
                else diffs.Add($"{Path.GetFileName(Path.GetDirectoryName(old))}/{id}: " + string.Join(", ", keys.Where(k => Dump(Plain(e[k]), 0) != Dump(Plain(m[k]), 0))));
                extra?.Invoke(old, e, m, diffs);
            }
        Gate(what, common > 0 && same == common, blind, D(("composite", compositeFile), ("against", olds.ToList()), ("common", common), ("identical", same), ("differ", diffs.Take(20).ToList())));
    }

    void CheckTides(string outDir, string[] olds) =>
        Against("tides", Path.Combine(outDir, "tides", "stations.json"), olds, d => d.Arr("stations").Select(s => (s!.Str("id")!, s!)), ["name", "river_km"],
            "compares the entry's own keys (the default file's short names and river kilometres are the old harvester's per-place list and are not compared); the coefficients are compared as the dumped text, so equal to the last printed digit",
            (old, e, m, diffs) =>
            {
                var f = e.Str("official_file");
                if (f is null) return;
                var a = Path.Combine(Path.GetDirectoryName(old)!, f);
                var b = Path.Combine(outDir, "tides", f);
                if (File.Exists(a) && File.Exists(b) && !File.ReadAllBytes(a).AsSpan().SequenceEqual(File.ReadAllBytes(b))) diffs.Add($"{f}: sidecar bytes differ");
            });

    void CheckCurrents(string outDir, string[] olds) =>
        Against("currents", Path.Combine(outDir, "currents", "currents.json"), olds,
            d => d.Arr("tidal_stations").Select(s => ((s!.Str("id") ?? "") + (s.Has("bin") ? "_b" + s.Num("bin").ToString(CultureInfo.InvariantCulture) : ""), s!)), ["km_from_anchor"],
            "a station's events depend on its prediction window: equal only where the composite read the same first copy the place did");

    void CheckSea(string outDir, string[] olds)
    {
        var mine = Path.Combine(outDir, "sea", "seastate.json");
        Against("seastate buoys", mine, olds, d => d.Obj("buoys").Select(kv => (kv.Key, kv.Value!)), [],
            "a buoy's line is its first cached copy; a buoy the place took from outside the painted regions is not in the composite and not compared");
        var pts = Load(mine)!.Arr("points");
        int common = 0, same = 0;
        var diffs = new List<string>();
        foreach (var old in olds)
        {
            var o = Load(old)!;
            var p = pts.FirstOrDefault(x => Math.Abs(x!.Num("lat") - o.Num("point_lat")) < 1e-9 && Math.Abs(x!.Num("lon") - o.Num("point_lon")) < 1e-9 && x!.Str("cycle_label") == o.Str("cycle_label"));
            if (p is null) { diffs.Add($"{old}: node ({o.Num("point_lat")}, {o.Num("point_lon")}) {o.Str("cycle_label")} not among the composite's points"); continue; }
            common++;
            if (Dump(Plain(o["hours"]), 0) == Dump(Plain(p["hours"]), 0)) same++; else diffs.Add($"{old}: hours differ");
        }
        Gate("seastate points", common > 0 && same == common, "a node and cycle in both: the composite's hours at the place's own node from the same cached grib; a place's node outside the painted regions is reported, not compared",
            D(("against", olds.ToList()), ("common", common), ("identical", same), ("differ", diffs)));
    }
}
