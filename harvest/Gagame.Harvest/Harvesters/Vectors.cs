using System.Globalization;
using System.Text.Json.Nodes;
using System.Xml.Linq;
using Gagame.Harvest.Readers;
using static Gagame.Harvest.JsonUtil;

namespace Gagame.Harvest.Harvesters;

/// M6p: THE SURVEY ORDER (harvest_vectors.py). Vector files as lossless manuscripts queryable
/// by LOD: every vertex carries the Visvalingam-Whyatt effective area at which decimation would
/// remove it -- the wedge product -- stored as sqrt(area) in metres.
public sealed class Vectors : IHarvester
{
    public string Name => "vectors";
    public string Title => "Vectors (vpack)";
    public string What => "Every vector file as a lossless LOD manuscript: the GSHHG bins, the hand survey edits, any shapefile.";
    public string[] Outputs => ["data/vectors/vectors.vpack"];
    public Source[] Sources => [new("OSM ODbL (survey edits)", "https://www.openstreetmap.org/copyright")];
    public Insert? Insert => new(InsertKind.DropFolder, "Drop ESRI shapefiles (.shp with its .dbf) here; the next run folds them in.", Folder: "cache/vectors", Pattern: "*.shp");

    static string GisDir => Path.Combine("data", "gis");
    static string CacheDir => Path.Combine("cache", "vectors");
    static string OutDir => Path.Combine("data", "vectors");

    /// Per-vertex Visvalingam-Whyatt effective area, monotone, in local metres^2. Endpoints: +inf.
    public static double[] VwImportance(List<(double lon, double lat)> pts)
    {
        var n = pts.Count;
        var imp = new double[n];
        Array.Fill(imp, double.PositiveInfinity);
        if (n < 3) return imp;
        // Python 3.12+'s sum() of floats is Neumaier-compensated; the mean must match it to the bit,
        // or near-tie vertices decimate in a different order and the importances differ.
        var lat0 = NeumaierSum(pts.Select(p => p.lat)) / n;
        var kx = 111320.0 * Math.Max(Math.Cos(lat0 * Geo.DegToRad), 0.05);
        const double ky = 110574.0;
        double Area(int i, int j, int k)
        {
            double ax = (pts[j].lon - pts[i].lon) * kx, ay = (pts[j].lat - pts[i].lat) * ky;
            double bx = (pts[k].lon - pts[i].lon) * kx, by = (pts[k].lat - pts[i].lat) * ky;
            return Math.Abs(ax * by - ay * bx) * 0.5;
        }
        var prev = new int[n]; var nxt = new int[n]; var ver = new int[n];
        for (var i = 0; i < n; i++) { prev[i] = i - 1; nxt[i] = i + 1; }
        var heap = new PriorityQueue<(int i, int v), (double a, int i, int v)>();
        for (var i = 1; i < n - 1; i++) heap.Enqueue((i, 0), (Area(i - 1, i, i + 1), i, 0));
        var floor = 0.0;
        while (heap.Count > 0)
        {
            heap.TryDequeue(out var it, out var pri);
            var (a, i, v) = (pri.a, it.i, it.v);
            if (v != ver[i]) continue;
            floor = Math.Max(floor, a);
            imp[i] = floor;
            int p = prev[i], q = nxt[i];
            prev[q] = p; nxt[p] = q;
            foreach (var j in new[] { p, q })
                if (0 < j && j < n - 1)
                {
                    ver[j]++;
                    heap.Enqueue((j, ver[j]), (Area(prev[j], j, nxt[j]), j, ver[j]));
                }
        }
        return imp;
    }

    /// CPython's sum() of floats (3.12+): Neumaier's compensated summation.
    public static double NeumaierSum(IEnumerable<double> xs)
    {
        double total = 0.0, c = 0.0;
        foreach (var x in xs)
        {
            var t = total + x;
            if (Math.Abs(total) >= Math.Abs(x)) c += (total - t) + x;
            else c += (x - t) + total;
            total = t;
        }
        return total + c;
    }

    public static double[] ToMeters(double[] imp) => imp.Select(a => double.IsInfinity(a) ? 1.0e9 : Math.Sqrt(a)).ToArray();

    // ------------------------------------------------------------------ readers

    public static (int kind, List<List<(double, double)>> polys)? ReadShp(string path)
    {
        var data = File.ReadAllBytes(path);
        if (data.Length < 100 || Bin.I32BE(data, 0) != 9994) return null;
        var lines = new List<List<(double, double)>>();
        var points = new List<(double, double)>();
        var kind = 0;
        foreach (var rec in Shapefile.Records(data))
        {
            if (rec.Type is 1 or 11 or 21 or 8 or 18 or 28) points.AddRange(rec.Points);
            else
            {
                lines.AddRange(rec.Parts);
                kind = rec.Type is 5 or 15 or 25 ? 1 : 0;
            }
        }
        if (points.Count > 0 && lines.Count == 0) return (2, points.Select(p => new List<(double, double)> { p }).ToList());
        return (kind, lines);
    }

    public static List<Dictionary<string, object?>> ReadDbf(string path)
    {
        if (!File.Exists(path)) return new();
        return Shapefile.Dbf(File.ReadAllBytes(path), 100000, System.Text.Encoding.ASCII)
            .Select(r => r.ToDictionary(kv => kv.Key, kv => (object?)kv.Value)).ToList();
    }

    public static (int kind, List<List<(double, double)>> polys, List<Dictionary<string, object?>> attrs) ReadGeoJson(string path)
    {
        var gj = Load(path)!;
        var feats = gj.Str("type") == "FeatureCollection" ? gj.Arr("features").ToList() : new List<JsonNode?> { gj };
        var lines = new List<List<(double, double)>>();
        var points = new List<(double, double)>();
        var kind = 0;
        var attrs = new List<Dictionary<string, object?>>();
        static (double, double) P(JsonNode? c) => (c![0]!.GetValue<double>(), c[1]!.GetValue<double>());
        static List<(double, double)> Ring(JsonNode? r) => r!.AsArray().Select(P).ToList();
        foreach (var ft in feats)
        {
            var g = ft.Has("geometry") ? ft!["geometry"] : ft;
            var t = g.Str("type");
            var c = g?["coordinates"];
            switch (t)
            {
                case "Point": points.Add(P(c)); break;
                case "MultiPoint": points.AddRange(c!.AsArray().Select(P)); break;
                case "LineString": lines.Add(Ring(c)); break;
                case "MultiLineString": lines.AddRange(c!.AsArray().Select(Ring)); break;
                case "Polygon": kind = 1; lines.AddRange(c!.AsArray().Select(Ring)); break;
                case "MultiPolygon": kind = 1; foreach (var poly in c!.AsArray()) lines.AddRange(poly!.AsArray().Select(Ring)); break;
            }
            attrs.Add(ft.Has("properties") && ft!["properties"] is JsonObject po ? (Dictionary<string, object?>)Plain(po)! : new());
        }
        if (points.Count > 0 && lines.Count == 0) return (2, points.Select(p => new List<(double, double)> { p }).ToList(), attrs);
        return (kind, lines, attrs);
    }

    public static (int kind, List<List<(double, double)>> polys, List<Dictionary<string, object?>> attrs) ReadKml(string path)
    {
        var root = XDocument.Load(path).Root!;
        static string Local(XElement e) => e.Name.LocalName;
        static List<(double, double)> Coords(XElement el)
        {
            var pts = new List<(double, double)>();
            foreach (var tok in (el.Value ?? "").Split((char[]?)null, StringSplitOptions.RemoveEmptyEntries))
            {
                var parts = tok.Split(',');
                if (parts.Length >= 2) pts.Add((double.Parse(parts[0], CultureInfo.InvariantCulture), double.Parse(parts[1], CultureInfo.InvariantCulture)));
            }
            return pts;
        }
        var lines = new List<List<(double, double)>>();
        var points = new List<(double, double)>();
        var kind = 0;
        var attrs = new List<Dictionary<string, object?>>();
        foreach (var pm in root.DescendantsAndSelf().Where(e => Local(e) == "Placemark"))
        {
            var a = new Dictionary<string, object?>();
            foreach (var ch in pm.Elements())
                if (Local(ch) is "name" or "description" && !string.IsNullOrEmpty(ch.Value)) a[Local(ch)] = ch.Value.Trim();
            foreach (var el in pm.DescendantsAndSelf())
            {
                var t = Local(el);
                if (t == "Point")
                    foreach (var c in el.DescendantsAndSelf().Where(x => Local(x) == "coordinates")) points.AddRange(Coords(c));
                else if (t == "LineString")
                    foreach (var c in el.DescendantsAndSelf().Where(x => Local(x) == "coordinates")) { var pts = Coords(c); if (pts.Count >= 2) lines.Add(pts); }
                else if (t == "Polygon")
                {
                    kind = 1;
                    foreach (var ring in el.DescendantsAndSelf().Where(x => Local(x) == "LinearRing"))
                        foreach (var c in ring.DescendantsAndSelf().Where(x => Local(x) == "coordinates")) { var pts = Coords(c); if (pts.Count >= 2) lines.Add(pts); }
                }
            }
            attrs.Add(a);
        }
        if (points.Count > 0 && lines.Count == 0) return (2, points.Select(p => new List<(double, double)> { p }).ToList(), attrs);
        return (kind, lines, attrs);
    }

    // ------------------------------------------------------------------ writer

    public static void WriteVpack(List<(string name, int kind, List<List<(double lon, double lat)>> polys)> layers, string path)
    {
        using var f = File.Create(path);
        f.Write("VPK1"u8);
        Bin.U32(f, (uint)layers.Count);
        foreach (var (name, kind, polys) in layers)
        {
            var nb = System.Text.Encoding.UTF8.GetBytes(name);
            Bin.U16(f, (ushort)nb.Length);
            f.Write(nb);
            f.WriteByte((byte)kind);
            f.WriteByte(0);
            Bin.U32(f, (uint)polys.Count);
            foreach (var pts in polys)
            {
                var imp = kind != 2 ? ToMeters(VwImportance(pts)) : Enumerable.Repeat(1.0e9, pts.Count).ToArray();
                Bin.U32(f, (uint)pts.Count);
                for (var i = 0; i < pts.Count; i++)
                {
                    Bin.F32(f, (float)pts[i].lon);
                    Bin.F32(f, (float)pts[i].lat);
                    Bin.F32(f, (float)Math.Min(imp[i], 1.0e9));
                }
            }
        }
    }

    static readonly Dictionary<string, object?> DefaultEdits = D(
        ("type", "FeatureCollection"),
        ("_readme", new List<string>
        {
            "HAND-EDITABLE mask overrides. Each polygon forces the land/water classifier:",
            "properties.mask = 'land' | 'water'. Coordinates are WGS84 lon/lat. The engine",
            "rasterizes these OVER the survey mask AND over the in-window live-tide classifier",
            "at load -- edit, save, rerun. DON'T hand-digitize charted structures: seed them",
            "from surveyed vectors (the repo copy carries the Merrimack jetty footprints from",
            "OSM ways 640430599/640430601/256125510/559887262 -- see data/gis/structures.kml).",
        }),
        ("features", new List<object?>()));

    public int Run(HarvestContext ctx, string[] argv)
    {
        Directory.CreateDirectory(OutDir);
        Directory.CreateDirectory(CacheDir);
        var layers = new List<(string, int, List<List<(double, double)>>)>();
        var registry = new List<Dictionary<string, object?>>();
        string[] Kinds = ["polyline", "polygon", "points"];

        foreach (var (name, kind, src) in new[] { ("coast_ne", 0, "gshhg"), ("rivers_ne", 0, "gshhg"), ("coast_global", 0, "gshhg"), ("cusp_ne", 0, "cusp"), ("nhd_rivers_ne", 0, "nhd"), ("nhd_water_ne", 1, "nhd") })
        {
            var p = Path.Combine(GisDir, $"{name}.bin");
            if (!File.Exists(p)) continue;
            var polys = PolylineBin.Read(p);
            layers.Add((name, kind, polys));
            registry.Add(D(("layer", name), ("kind", Kinds[kind]), ("source", src), ("polys", polys.Count), ("verts", polys.Sum(x => x.Count))));
        }
        foreach (var p in Directory.GetFiles(CacheDir, "*.shp").OrderBy(x => x, StringComparer.Ordinal))
        {
            var name = Path.GetFileNameWithoutExtension(p);
            var r = ReadShp(p);
            if (r is null) { ctx.Log($"[vectors] {name}.shp: not a shapefile?"); continue; }
            var (kind, polys) = r.Value;
            var attrs = ReadDbf(p[..^4] + ".dbf");
            if (attrs.Count > 0) Save(Path.Combine(OutDir, $"{name}.attrs.json"), attrs, indent: 0);
            layers.Add((name, kind, polys));
            registry.Add(D(("layer", name), ("kind", Kinds[kind]), ("source", "shp"), ("polys", polys.Count), ("verts", polys.Sum(x => x.Count)), ("attrs", attrs.Count)));
        }
        foreach (var p in Directory.GetFiles(CacheDir, "*.geojson").OrderBy(x => x, StringComparer.Ordinal))
        {
            var name = Path.GetFileNameWithoutExtension(p);
            var (kind, polys, attrs) = ReadGeoJson(p);
            if (attrs.Count > 0) Save(Path.Combine(OutDir, $"{name}.attrs.json"), attrs, indent: 0);
            layers.Add((name, kind, polys));
            registry.Add(D(("layer", name), ("kind", Kinds[kind]), ("source", "geojson"), ("polys", polys.Count), ("verts", polys.Sum(x => x.Count))));
        }
        foreach (var p in Directory.GetFiles(GisDir, "*.kml").Concat(Directory.GetFiles(CacheDir, "*.kml")).OrderBy(x => x, StringComparer.Ordinal))
        {
            var name = Path.GetFileNameWithoutExtension(p);
            var (kind, polys, attrs) = ReadKml(p);
            if (polys.Count == 0) { ctx.Log($"[vectors] {name}.kml: no geometry"); continue; }
            if (attrs.Count > 0) Save(Path.Combine(OutDir, $"{name}.attrs.json"), attrs, indent: 0);
            layers.Add((name, kind, polys));
            registry.Add(D(("layer", name), ("kind", Kinds[kind]), ("source", "kml"), ("polys", polys.Count), ("verts", polys.Sum(x => x.Count))));
        }
        WriteVpack(layers, Path.Combine(OutDir, "vectors.vpack"));
        Save(Path.Combine(OutDir, "vectors.json"), D(("format", "VPK1"),
            ("importance", "sqrt(Visvalingam effective area) in meters = |wedge|/2 (query LOD by filtering importance >= tolerance)"),
            ("crs", "EPSG:4326"), ("layers", registry)));
        ctx.Log($"[vectors] {layers.Count} layers, {registry.Sum(r => (int)r["verts"]!)} vertices (lossless) -> vectors.vpack");

        var edits = Path.Combine(GisDir, "edits.geojson");
        if (!File.Exists(edits))
        {
            Save(edits, DefaultEdits);
            ctx.Log($"[vectors] authored {edits} (jetty land overrides -- edit by hand)");
        }
        else ctx.Log($"[vectors] {edits} exists; leaving your edits alone");
        return 0;
    }
}
