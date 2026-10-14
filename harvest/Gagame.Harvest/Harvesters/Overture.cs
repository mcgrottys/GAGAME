using System.Diagnostics;
using System.Globalization;
using System.Text.Json.Nodes;
using System.Text.RegularExpressions;
using static Gagame.Harvest.JsonUtil;

namespace Gagame.Harvest.Harvesters;

/// Overture Maps buildings that carry a HEIGHT, as building solids, one GABLDG01 file per dataset
/// the height is attributed to (harvest_overture.py). The release's STAC catalog names each
/// GeoParquet file's box; the files meeting the box are read by GDAL's ogr2ogr (QGIS) over HTTP.
public sealed class Overture : IHarvester
{
    public string Name => "overture";
    public string Title => "Overture buildings (heights)";
    public string What => "Overture Maps buildings carrying a height, split by the dataset the height came from (USGS lidar, Microsoft ML, ...).";
    public string[] Outputs => [];
    public Source[] Sources => [new("Overture Maps", "https://overturemaps.org/"), new("Overture STAC", "https://stac.overturemaps.org")];
    public Insert? Insert => new(InsertKind.PlaceBox, "The box to read and the stem to name the files (--stem); needs QGIS's ogr2ogr.", Extras: ["--stem"]);
    public string? Note => "Needs QGIS (ogr2ogr); writes under the big-data drive's Overture folder.";

    const string Stac = "https://stac.overturemaps.org";
    const string Ua = "GAGAME/0.1 (hobby ocean sim; Overture buildings by bbox)";
    const string Columns = "id,height,min_height,num_floors,min_floor,sources,roof_shape,roof_height";
    const string QgisEnv = @"C:\Program Files\QGIS 4.0.0\bin\o4w_env.bat";

    static readonly HttpClient Http = new() { Timeout = TimeSpan.FromSeconds(120) };

    static JsonNode GetJson(string url, int tries = 5)
    {
        for (var k = 0; k < tries; k++)
        {
            try
            {
                using var req = new HttpRequestMessage(HttpMethod.Get, url);
                req.Headers.TryAddWithoutValidation("User-Agent", Ua);
                using var resp = Http.Send(req);
                resp.EnsureSuccessStatusCode();
                return Parse(resp.Content.ReadAsStringAsync().GetAwaiter().GetResult())!;
            }
            catch (Exception) when (k < tries - 1) { Thread.Sleep(TimeSpan.FromSeconds(2.0 * Math.Pow(2, k))); }
        }
        throw new InvalidOperationException($"could not read {url}");
    }

    static List<(string id, double[] bbox, string href)> ItemsFor(string release, double[] bbox, string cacheDir)
    {
        var path = Path.Combine(cacheDir, $"stac-{release}-building.json");
        List<(string, double[], string)> items;
        if (File.Exists(path))
            items = Load(path)!.AsArray().Select(it => (it.Str("id")!, it.Arr("bbox").Select(x => x!.GetValue<double>()).ToArray(), it.Str("href")!)).ToList();
        else
        {
            var coll = GetJson($"{Stac}/{release}/buildings/building/collection.json");
            items = new();
            foreach (var link in coll.Arr("links"))
            {
                if (link.Str("rel") != "item") continue;
                var it = GetJson(link.Str("href")!);
                var href = it.Obj("assets").Select(kv => kv.Value.Str("href")!).First(h => h.EndsWith(".parquet"));
                items.Add((it.Str("id")!, it.Arr("bbox").Select(x => x!.GetValue<double>()).ToArray(), href));
            }
            Directory.CreateDirectory(cacheDir);
            Save(path, items.Select(i => D(("id", i.Item1), ("bbox", i.Item2.ToList()), ("href", i.Item3))).ToList(), indent: 0);
        }
        double w = bbox[0], s = bbox[1], e = bbox[2], n = bbox[3];
        return items.Where(it => it.Item2[0] <= e && it.Item2[2] >= w && it.Item2[1] <= n && it.Item2[3] >= s).ToList();
    }

    static void Fetch(string href, double[] bbox, string outPath, string envBat)
    {
        var bat = outPath + ".bat";
        File.WriteAllText(bat, "@echo off\r\n" + $"call \"{envBat}\"\r\n" + $"set \"GDAL_HTTP_USERAGENT={Ua}\"\r\n" +
            $"ogr2ogr -f GeoJSONSeq \"{outPath}\" \"/vsicurl/{href}\" -spat {F(bbox[0])} {F(bbox[1])} {F(bbox[2])} {F(bbox[3])} -select \"{Columns}\" -where \"height IS NOT NULL OR num_floors IS NOT NULL\"\r\n" +
            "exit /b %ERRORLEVEL%\r\n");
        var psi = new ProcessStartInfo("cmd.exe") { ArgumentList = { "/c", bat }, UseShellExecute = false, RedirectStandardOutput = true, RedirectStandardError = true, CreateNoWindow = true };
        using var p = Process.Start(psi)!;
        var err = p.StandardError.ReadToEnd();
        p.StandardOutput.ReadToEnd();
        p.WaitForExit();
        File.Delete(bat);
        if (p.ExitCode != 0) throw new InvalidOperationException($"ogr2ogr exit {p.ExitCode}: {err.Trim()[^Math.Min(400, err.Trim().Length)..]}");
    }

    static string F(double v) => v.ToString("R", CultureInfo.InvariantCulture);

    static long OsmId(JsonNode? sources)
    {
        if (sources is not JsonArray arr) return 0;
        foreach (var src in arr)
        {
            if (src.Str("dataset") != "OpenStreetMap" || src.Str("record_id") is not string rid) continue;
            var m = Regex.Match(rid, @"([wr])(\d+)");
            if (m.Success) return long.Parse(m.Groups[2].Value) * (m.Groups[1].Value == "w" ? 1 : -1);
        }
        return 0;
    }

    static string HeightDataset(JsonNode? sources)
    {
        if (sources is JsonArray arr)
            foreach (var src in arr)
                if (src.Str("property") == "/properties/height")
                    return Regex.Replace((src.Str("dataset") ?? "unattributed").ToLowerInvariant(), "[^a-z0-9]+", "_").Trim('_');
        return "unattributed";
    }

    public int Run(HarvestContext ctx, string[] argv)
    {
        var a = new Args(argv);
        double[] bbox;
        var bi = Array.IndexOf(argv, "--bbox");
        if (bi < 0 && a.BoxOpt() is { } bx) bbox = [bx.Lon0, bx.Lat0, bx.Lon1, bx.Lat1];
        else if (bi >= 0 && bi + 4 < argv.Length && !argv[bi + 1].Contains(',')) bbox = argv.Skip(bi + 1).Take(4).Select(s => double.Parse(s, CultureInfo.InvariantCulture)).ToArray();
        else if (bi >= 0) bbox = Args.Pair(argv[bi + 1]);
        else throw new HarvestStop("overture needs --bbox lon0,lat0,lon1,lat1 and --stem NAME");
        var stem = a.Opt("--stem") ?? a.Opt("--place") ?? throw new HarvestStop("overture needs --stem NAME");
        var releaseArg = a.Opt("--release", "latest")!;
        var outDir = a.Opt("--out") ?? Path.Combine(Path.GetDirectoryName(ctx.DataCache.TrimEnd('\\', '/')) ?? ctx.DataCache, "Overture", "buildings");
        var qgisEnv = a.Opt("--qgis-env", QgisEnv)!;
        var t0 = ctx.Seconds;
        var release = releaseArg == "latest" ? GetJson($"{Stac}/catalog.json").Str("latest")! : releaseArg;
        var rawDir = Path.Combine(Path.GetDirectoryName(outDir.TrimEnd('\\', '/'))!, "raw", release);
        Directory.CreateDirectory(rawDir);
        var items = ItemsFor(release, bbox, Path.GetDirectoryName(rawDir)!);
        ctx.Log($"[overture] release {release}: {items.Count} building files meet the box");

        var roofShapes = new List<string>();
        var byDs = new Dictionary<string, Dictionary<string, Buildings.Rec>>();
        var refused = new Dictionary<string, int> { ["geometry"] = 0 };
        foreach (var (id, _, href) in items)
        {
            var part = Path.Combine(rawDir, $"{stem}-{id}.geojsons");
            if (!File.Exists(part))
            {
                Fetch(href, bbox, part + ".tmp", qgisEnv);
                File.Move(part + ".tmp", part, overwrite: true);
            }
            ctx.Log($"[overture] {id}: {new FileInfo(part).Length / 1048576.0:0.0} MB of features ({ctx.Seconds - t0:0} s)");
            foreach (var raw in File.ReadLines(part))
            {
                var line = raw.Trim('\x1e', ' ', '\r', '\n');
                if (line.Length == 0) continue;
                var f = Parse(line)!;
                var p = f.Obj("properties");
                var g = f.Obj("geometry");
                JsonNode? src = p["sources"];
                if (src is JsonValue sv && sv.TryGetValue<string>(out var ss)) src = Parse(ss);
                var polys = g.Str("type") == "Polygon" ? new List<JsonNode?> { g["coordinates"] } : g.Str("type") == "MultiPolygon" ? g.Arr("coordinates").ToList() : new();
                var geo = new List<Buildings.Ring>();
                foreach (var poly in polys)
                {
                    var rings = poly!.AsArray();
                    for (var k = 0; k < rings.Count; k++)
                    {
                        var pts = rings[k]!.AsArray().Select(pt => ((int)Math.Round(pt![0]!.GetValue<double>() * 1e7, MidpointRounding.ToEven), (int)Math.Round(pt[1]!.GetValue<double>() * 1e7, MidpointRounding.ToEven))).ToList();
                        if (pts.Count > 1 && pts[0] == pts[^1]) pts.RemoveAt(pts.Count - 1);
                        if (pts.Count >= 3) geo.Add(new Buildings.Ring((byte)(k == 0 ? 1 : 0), pts.SelectMany(q => new[] { q.Item1, q.Item2 }).ToArray()));
                    }
                }
                if (geo.Count == 0) { refused["geometry"]++; continue; }
                var roof = 255;
                if (p.Str("roof_shape") is string rs && rs.Length > 0)
                {
                    if (!roofShapes.Contains(rs)) roofShapes.Add(rs);
                    roof = roofShapes.IndexOf(rs);
                }
                float Num(string k) => p.NumOrNull(k) is double d ? (float)d : float.NaN;
                var attrs = new Buildings.Attrs(roof, Num("height"), Num("min_height"), Num("num_floors"), Num("min_floor"), Num("roof_height"));
                int x0 = geo[0].Xy[0], y0 = geo[0].Xy[1];
                var ds = HeightDataset(src);
                if (!byDs.TryGetValue(ds, out var recs)) byDs[ds] = recs = new();
                recs[p.Str("id")!] = new Buildings.Rec(Buildings.CellOf(x0, y0), OsmId(src), 0, attrs, geo);
            }
        }
        foreach (var (ds, recs) in byDs.OrderBy(kv => kv.Key, StringComparer.Ordinal))
        {
            var s = $"{stem}-{ds}";
            var (binPath, tagged) = Buildings.WriteGabldg(outDir, s, recs.Values.ToList(),
                D(("release", release), ("theme", "buildings/building"), ("files", items.Select(i => i.href).ToList()), ("heightDataset", ds)),
                "ODbL 1.0", "(c) OpenStreetMap contributors, Overture Maps Foundation", bbox.ToList(), refused, new(), roofShapes,
                "int32, 1e-7 degree, WGS84 lon/lat (Overture's doubles, rounded)");
            ctx.Log($"[overture] {s}: {recs.Count:N0} solids -> {binPath} ({new FileInfo(binPath).Length / 1048576.0:0} MB); tagged {Dump(tagged, 0)}");
        }
        ctx.Log($"[overture] done in {ctx.Seconds - t0:0} s; refused {Dump(refused, 0)}");
        return 0;
    }
}
