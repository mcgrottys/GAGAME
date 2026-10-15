using System.Globalization;
using System.Text.Json.Nodes;
using System.Text.RegularExpressions;
using static Gagame.Harvest.JsonUtil;

namespace Gagame.Harvest.Harvesters;

/// M3: currents (harvest_currents.py). CO-OPS tidal current predictions (max/slack event
/// tables) for the entrance stations, NDBC buoy 44029's ADCP, and the GoMOFS surface field
/// through the CO-OPS THREDDS server, probed not assumed.
public sealed class Currents : IHarvester
{
    public string Name => "currents";
    public string Title => "Currents (CO-OPS + GoMOFS)";
    public string What => "Tidal current predictions at the entrance stations, buoy ADCP, and the GoMOFS surface field.";
    public string[] Outputs => ["data/currents/currents.json"];
    public Source[] Sources => [new("CO-OPS currents", "https://tidesandcurrents.noaa.gov/currents/"), new("GoMOFS", "https://tidesandcurrents.noaa.gov/ofs/gomofs/gomofs.html")];
    public Insert? Insert => new(InsertKind.PlaceBox, "A place's current stations within the box.", Extras: ["--radius-km"]);

    const string Coops = "https://api.tidesandcurrents.noaa.gov/api/prod/datagetter";
    const string Ndbc = "https://www.ndbc.noaa.gov/data/realtime2";
    const string Thredds = "https://opendap.co-ops.nos.noaa.gov/thredds";
    static readonly (string id, string name)[] Stations = [("ACT0816", "Merrimack River entrance"), ("ACT0821", "Newburyport"), ("ACT0826", "Plum Island Sound entrance")];
    public sealed record Grid(double Lon0, double Lat0, double Lon1, double Lat1, int Nx, int Ny)
    {
        public double Dlon => (Lon1 - Lon0) / Nx;
        public double Dlat => (Lat1 - Lat0) / Ny;
    }
    public static readonly Grid GulfGrid = new(-71.1, 41.9, -69.3, 43.3, 280, 200);
    const double Knot = 0.514444;

    HarvestContext _ctx = null!;
    SimpleFetcher _simple = null!;
    Polite? _polite;

    /// The script's fetch: text unless binary; polite in place mode (max age ignored there).
    (byte[] data, bool cached) Fetch(string url, string? cachePath, double? maxAgeS = null, int timeoutS = 90)
    {
        if (_polite is not null) return _polite.Get(url, cachePath, timeoutS);
        return _simple.Get(url, cachePath, maxAgeS, timeoutS);
    }

    (string text, bool cached) FetchText(string url, string? cachePath, double? maxAgeS = null, int timeoutS = 90)
    {
        var (b, c) = Fetch(url, cachePath, maxAgeS, timeoutS);
        return (System.Text.Encoding.UTF8.GetString(b), c);
    }

    /// Routes this instance's requests through a place's polite fetcher (the orchestrator's use).
    public Currents UsePolite(HarvestContext ctx, Polite polite)
    {
        _ctx = ctx;
        _simple = new SimpleFetcher(ctx, 0.6, 3);
        _polite = polite;
        return this;
    }

    public int Run(HarvestContext ctx, string[] argv)
    {
        _ctx = ctx;
        _simple = new SimpleFetcher(ctx, 0.6, 3);
        var a = new Args(argv);
        if (a.Has("--place"))
        {
            var box = a.BoxOpt() ?? throw new HarvestStop("--place needs --box");
            HarvestPlace(ctx, new Polite(ctx) { Place = a.Opt("--place") }, a.Opt("--place")!, box, a.Num("--radius-km", 20), a.LonLat("--anchor"), a.Int("--max-stations", 8), a.Opt("--adcp"));
            return 0;
        }
        var outDir = Path.Combine("data", "currents");
        var cacheDir = Path.Combine("cache", "currents");
        Directory.CreateDirectory(outDir);
        var stations = new List<Dictionary<string, object?>>();
        foreach (var (sid, name) in Stations)
        {
            try { stations.Add(FetchTidalStation(sid, name, cacheDir)); }
            catch (Exception e) { ctx.Log($"[{sid}] FAILED: {e.Message}"); }
        }
        var adcp = FetchAdcp("44029", cacheDir);
        Field? field = null;
        try { field = FetchGomofsField(cacheDir); }
        catch (Exception e) { ctx.Log($"GoMOFS field: {e.Message}"); }
        var out_ = D(("generated_utc", NowIso()), ("tidal_stations", stations), ("buoy_adcp", adcp), ("field", null));
        if (field is not null)
        {
            var g = GulfGrid;
            WriteUv(Path.Combine(outDir, "gomofs_uv.f32"), field.U, field.V, false);
            out_["field"] = D(("file", "gomofs_uv.f32"), ("nx", g.Nx), ("ny", g.Ny), ("lon0", g.Lon0), ("lat0", g.Lat0), ("dlon", g.Dlon), ("dlat", g.Dlat), ("source", field.Source));
        }
        var path = Path.Combine(outDir, "currents.json");
        Save(path, out_);
        ctx.Log($"wrote {path} ({stations.Count} tidal stations, field: {(field is not null ? "yes, " + field.Source : "NO")})");
        return 0;
    }

    static void WriteUv(string path, float[] u, float[] v, bool exclusive)
    {
        using var f = new FileStream(path, exclusive ? FileMode.CreateNew : FileMode.Create);
        f.Write(System.Runtime.InteropServices.MemoryMarshal.AsBytes<float>(u));
        f.Write(System.Runtime.InteropServices.MemoryMarshal.AsBytes<float>(v));
    }

    // ================================================================= 1. tidal current predictions

    static string Q(Dictionary<string, string> p) => string.Join("&", p.Select(kv => Uri.EscapeDataString(kv.Key) + "=" + Uri.EscapeDataString(kv.Value)));

    /// 45 days of max/slack events from `from` (default ten days before now), in two requests.
    public Dictionary<string, object?> FetchTidalStation(string sid, string name, string cacheDir, int? bin = null, DateTime? from = null)
    {
        var begin = from ?? DateTime.UtcNow.AddDays(-10);
        var events = new List<(double t, string type, double ms)>();
        double? floodDeg = null, ebbDeg = null;
        for (var chunk = 0; chunk < 2; chunk++)
        {
            var b = begin.AddDays(30 * chunk);
            var e = b.AddDays(30) < begin.AddDays(45) ? b.AddDays(30) : begin.AddDays(45);
            var p = new Dictionary<string, string>
            {
                ["product"] = "currents_predictions", ["application"] = "GAGAME", ["station"] = sid,
                ["begin_date"] = b.ToString("yyyyMMdd", CultureInfo.InvariantCulture), ["end_date"] = e.ToString("yyyyMMdd", CultureInfo.InvariantCulture),
                ["interval"] = "MAX_SLACK", ["units"] = "english", ["time_zone"] = "gmt", ["format"] = "json",
            };
            var tag = sid;
            if (bin is int bn) { p["bin"] = bn.ToString(); tag = $"{sid}_b{bn}"; }
            var cache = Path.Combine(cacheDir, $"{tag}_{p["begin_date"]}_{p["end_date"]}.json");
            var (text, cached) = FetchText(Coops + "?" + Q(p), cache);
            var data = Parse(text);
            var cp = data.Obj("current_predictions").Arr("cp");
            if (cp.Count == 0 && data.Has("error")) throw new InvalidOperationException($"{sid}: {data!["error"]!.ToJsonString()}");
            foreach (var row in cp)
            {
                var t = DateTime.ParseExact(row.Str("Time")!, "yyyy-MM-dd HH:mm", CultureInfo.InvariantCulture, DateTimeStyles.AssumeUniversal | DateTimeStyles.AdjustToUniversal);
                var typ = (row.Str("Type") ?? "").ToLowerInvariant();
                var v = row.Num("Velocity_Major") * Knot;
                double signed;
                if (typ.Contains("flood")) signed = Math.Abs(v);
                else if (typ.Contains("ebb")) signed = -Math.Abs(v);
                else { signed = 0.0; typ = "slack"; }
                if (floodDeg is null && row.NumOrNull("meanFloodDir") is double fd)
                {
                    floodDeg = fd;
                    ebbDeg = row.NumOrNull("meanEbbDir") ?? (fd + 180.0) % 360.0;
                }
                events.Add((Unix(t), typ, Round(signed, 4)));
            }
            if (!cached) _ctx.Log($"    fetched {sid} {p["begin_date"]}..{p["end_date"]}: {cp.Count} events");
        }
        events.Sort((x, y) => x.t.CompareTo(y.t));
        var dedup = new List<Dictionary<string, object?>>();
        double? lastT = null;
        foreach (var ev in events)
        {
            if (lastT is null || Math.Abs(ev.t - lastT.Value) > 60)
            {
                dedup.Add(D(("t", ev.t), ("type", ev.type), ("ms", ev.ms)));
                lastT = ev.t;
            }
        }
        _ctx.Log($"[{sid}] {name}: {dedup.Count} events, flood {PyNum(floodDeg)} degT / ebb {PyNum(ebbDeg)} degT");
        var out_ = D(("id", sid), ("name", name), ("flood_deg", floodDeg), ("ebb_deg", ebbDeg), ("events", dedup));
        if (bin is int) out_["bin"] = bin;
        return out_;
    }

    static string PyNum(double? v) => v is null ? "None" : v.Value.ToString("R", CultureInfo.InvariantCulture);

    // ================================================================= 2. NDBC ADCP ground truth

    public Dictionary<string, object?>? FetchAdcp(string bid, string cacheDir)
    {
        var stamp = DateTime.UtcNow.ToString("yyyyMMddHH", CultureInfo.InvariantCulture);
        string text;
        try { (text, _) = FetchText($"{Ndbc}/{bid}.adcp", Path.Combine(cacheDir, $"{bid}_{stamp}.adcp"), maxAgeS: 1800); }
        catch (Exception e) { _ctx.Log($"buoy {bid}: no .adcp ({e.Message})"); return null; }
        var r = ParseAdcp(bid, text);
        if (r is not null) _ctx.Log($"buoy {bid}: {(double)r["ms"]!:0.00} m/s toward {(double)r["toward_deg"]!:0} degT at {(double)r["depth_m"]!:0.0} m depth");
        return r;
    }

    /// The newest line of a realtime2 .adcp file: obs time, depth, direction toward, speed.
    public static Dictionary<string, object?>? ParseAdcp(string bid, string text)
    {
        foreach (var ln in text.Split('\n'))
        {
            if (ln.StartsWith('#') || ln.Trim().Length == 0) continue;
            var t = ln.Split((char[]?)null, StringSplitOptions.RemoveEmptyEntries);
            if (t.Length < 8) continue;
            try
            {
                var obs = new DateTime(int.Parse(t[0]), int.Parse(t[1]), int.Parse(t[2]), int.Parse(t[3]), int.Parse(t[4]), 0, DateTimeKind.Utc);
                var dep = double.Parse(t[5], CultureInfo.InvariantCulture);
                var dir = double.Parse(t[6], CultureInfo.InvariantCulture);
                var spd = double.Parse(t[7], CultureInfo.InvariantCulture) / 100.0;
                return D(("id", bid), ("obs_unix", Unix(obs)), ("depth_m", dep), ("toward_deg", dir), ("ms", spd));
            }
            catch (FormatException) { continue; }
        }
        return null;
    }

    // ================================================================= 3. GoMOFS field via THREDDS

    public sealed class Field
    {
        public float[] U = Array.Empty<float>(), V = Array.Empty<float>();
        public string Source = "";
    }

    List<string> CrawlGomofsDatasets(string cacheDir)
    {
        var seen = new HashSet<string>();
        var found = new List<string>();
        var queue = new List<string> { "/thredds/catalog.xml", "/thredds/catalog/NOAA/GOMOFS/MODELS/catalog.xml" };
        var hops = 0;
        while (queue.Count > 0 && hops < 8)
        {
            var path = queue[0];
            queue.RemoveAt(0);
            if (!seen.Add(path)) continue;
            hops++;
            var url = "https://opendap.co-ops.nos.noaa.gov" + path;
            var cache = Path.Combine(cacheDir, "catalog", Regex.Replace(path, "[^A-Za-z0-9]+", "_") + ".xml");
            string xml;
            try { (xml, _) = FetchText(url, cache, maxAgeS: 6 * 3600); }
            catch (Exception e) { _ctx.Log($"    catalog {path}: {e.Message}"); continue; }
            foreach (Match m in Regex.Matches(xml, "href=\"([^\"]+\\.xml)\""))
            {
                var href = m.Groups[1].Value;
                var full = href.StartsWith('/') ? href : PyDirname(path) + "/" + href;
                if ((full.Contains("GOMOFS") || full.ToLowerInvariant().Contains("gomofs")) && !seen.Contains(full)) queue.Add(full);
            }
            foreach (Match m in Regex.Matches(xml, "urlPath=\"([^\"]+)\""))
                if (m.Groups[1].Value.ToLowerInvariant().Contains("gomofs")) found.Add(m.Groups[1].Value);
            queue.Sort(StringComparer.Ordinal);
            queue.Reverse();
        }
        return found;
    }

    static string PyDirname(string p) { var i = p.LastIndexOf('/'); return i <= 0 ? "" : p[..i]; }

    static List<double> ParseDapAscii(string text, string var)
    {
        var vals = new List<double>();
        var inBlock = false;
        foreach (var ln in text.Split('\n'))
        {
            var s = ln.Trim();
            if (!inBlock)
            {
                if (s.StartsWith(var) && s.Contains('[')) inBlock = true;
                continue;
            }
            if (s.Length == 0) break;
            var body = s;
            if (s.StartsWith('['))
            {
                var comma = s.IndexOf(',');
                if (comma < 0) continue;
                body = s[(comma + 1)..];
            }
            foreach (var tok in body.Split(','))
            {
                var t = tok.Trim();
                if (t.Length == 0) continue;
                if (double.TryParse(t, NumberStyles.Float, CultureInfo.InvariantCulture, out var v)) vals.Add(v);
            }
        }
        return vals;
    }

    Dictionary<string, List<int>> DapDdsShapes(string ds, string cacheDir)
    {
        var url = $"{Thredds}/dodsC/{ds}.dds";
        var cache = Path.Combine(cacheDir, "dds", Regex.Replace(ds, "[^A-Za-z0-9]+", "_") + ".dds");
        var (text, _) = FetchText(url, cache);
        var shapes = new Dictionary<string, List<int>>();
        foreach (Match m in Regex.Matches(text, @"Float\d+\s+(\w+)((?:\[[^\]]+\])+);"))
        {
            var dims = Regex.Matches(m.Groups[2].Value, @"=\s*(\d+)\]").Select(x => int.Parse(x.Groups[1].Value)).ToList();
            shapes[m.Groups[1].Value] = dims;
        }
        return shapes;
    }

    List<double> DapFetchVar(string ds, string var, string indexExpr, string cacheDir, string tag)
    {
        var expr = indexExpr.Replace("[", "%5B").Replace("]", "%5D");
        var url = $"{Thredds}/dodsC/{ds}.ascii?{var}{expr}";
        var cache = Path.Combine(cacheDir, "field", Regex.Replace(ds, "[^A-Za-z0-9]+", "_") + $"_{tag}.txt");
        var (text, cached) = FetchText(url, cache, timeoutS: 240);
        var vals = ParseDapAscii(text, var);
        if (!cached) _ctx.Log($"    DAP {var}{indexExpr}: {vals.Count} values");
        return vals;
    }

    Field? FetchGomofsField(string cacheDir)
    {
        var datasets = CrawlGomofsDatasets(cacheDir);
        _ctx.Log($"GoMOFS: catalog crawl found {datasets.Count} dataset paths");
        var twods = datasets.Where(d => d.Contains("2ds")).OrderByDescending(d => d, StringComparer.Ordinal).ToList();
        foreach (var ds in twods.Take(4))
        {
            Dictionary<string, List<int>> shapes;
            try { shapes = DapDdsShapes(ds, cacheDir); }
            catch (Exception e) { _ctx.Log($"    {ds}: DDS failed ({e.Message})"); continue; }
            var needed = new[] { "u_sur", "v_sur", "lon_u", "lat_u", "lon_v", "lat_v", "angle", "mask_u", "mask_v" };
            if (needed.Any(n => !shapes.ContainsKey(n))) { _ctx.Log($"    {ds}: missing some of [{string.Join(", ", needed.Select(n => "'" + n + "'"))}]"); continue; }
            const int stride = 2;
            (List<double> vals, int sny, int snx) Grab(string var, string tag, bool leadTime)
            {
                var dims = shapes[var];
                var sl = $"[0:{stride}:{dims[^2] - 1}][0:{stride}:{dims[^1] - 1}]";
                var lead = leadTime ? string.Concat(Enumerable.Repeat("[0]", dims.Count - 2)) : "";
                var vals = DapFetchVar(ds, var, lead + sl, cacheDir, tag);
                var sny = (dims[^2] - 1) / stride + 1;
                var snx = (dims[^1] - 1) / stride + 1;
                if (vals.Count < sny * snx) throw new InvalidOperationException($"{var}: {vals.Count} of {sny * snx}");
                return (vals, sny, snx);
            }
            List<double> u, lonU, latU, maskU, v, lonV, latV, maskV, ang, lonR, latR;
            int uny, unx, vny, vnx, any, anx;
            try
            {
                (u, uny, unx) = Grab("u_sur", "u", true);
                (lonU, _, _) = Grab("lon_u", "lonu", false);
                (latU, _, _) = Grab("lat_u", "latu", false);
                (maskU, _, _) = Grab("mask_u", "masku", false);
                (v, vny, vnx) = Grab("v_sur", "v", true);
                (lonV, _, _) = Grab("lon_v", "lonv", false);
                (latV, _, _) = Grab("lat_v", "latv", false);
                (maskV, _, _) = Grab("mask_v", "maskv", false);
                (ang, any, anx) = Grab("angle", "ang", false);
                (lonR, _, _) = Grab("lon_rho", "lonr", false);
                (latR, _, _) = Grab("lat_rho", "latr", false);
            }
            catch (Exception e) { _ctx.Log($"    {ds}: DAP subset failed ({e.Message})"); continue; }

            var gu = Scatter(GulfGrid, lonU, latU, u, maskU, uny, unx);
            var gv = Scatter(GulfGrid, lonV, latV, v, maskV, vny, vnx);
            var ga = Scatter(GulfGrid, lonR, latR, ang, null, any, anx);
            foreach (var gr in new[] { gu, gv, ga }) GapFill(GulfGrid, gr, 3);
            var g = GulfGrid;
            var outU = Filled(g.Nx * g.Ny, -999.0f);
            var outV = Filled(g.Nx * g.Ny, -999.0f);
            var filled = 0;
            for (var k = 0; k < g.Nx * g.Ny; k++)
            {
                float uu = gu[k], vv = gv[k], aa = ga[k];
                if (uu > -900 && vv > -900)
                {
                    var a = aa > -900 ? aa : 0.0;
                    outU[k] = (float)(uu * Math.Cos(a) - vv * Math.Sin(a));
                    outV[k] = (float)(uu * Math.Sin(a) + vv * Math.Cos(a));
                    filled++;
                }
            }
            _ctx.Log($"    resampled {ds}: {filled} of {g.Nx * g.Ny} cells water");
            if (filled < 1000) continue;
            return new Field { U = outU, V = outV, Source = ds };
        }
        return null;
    }

    static float[] Filled(int n, float v) { var a = new float[n]; Array.Fill(a, v); return a; }

    /// Nearest-cell scatter of a (possibly masked) staggered-grid variable onto the grid, 2x2 splat.
    public static float[] Scatter(Grid g, List<double> lon, List<double> lat, List<double> vals, List<double>? mask, int sny, int snx)
    {
        var out_ = Filled(g.Nx * g.Ny, -999.0f);
        var dlon = g.Dlon; var dlat = g.Dlat;
        for (var j = 0; j < sny; j++)
            for (var i = 0; i < snx; i++)
            {
                var k = j * snx + i;
                if (mask is not null && mask[k] < 0.5) continue;
                var x = vals[k];
                if (double.IsNaN(x) || Math.Abs(x) > 100) continue;
                var gx = (int)((lon[k] - g.Lon0) / dlon);
                var gy = (int)((lat[k] - g.Lat0) / dlat);
                for (var oy = 0; oy <= 1; oy++)
                    for (var ox = 0; ox <= 1; ox++)
                    {
                        int xx = gx + ox, yy = gy + oy;
                        if (0 <= xx && xx < g.Nx && 0 <= yy && yy < g.Ny) out_[yy * g.Nx + xx] = (float)x;
                    }
            }
        return out_;
    }

    public static void GapFill(Grid g, float[] grid, int sweeps)
    {
        for (var sweep = 0; sweep < sweeps; sweep++)
        {
            var need = sweep == 0 ? 2 : 1;
            var src = (float[])grid.Clone();
            for (var gy = 0; gy < g.Ny; gy++)
                for (var gx = 0; gx < g.Nx; gx++)
                {
                    var k = gy * g.Nx + gx;
                    if (src[k] > -900) continue;
                    double s = 0; var n = 0;
                    foreach (var (oy, ox) in new[] { (0, 1), (0, -1), (1, 0), (-1, 0) })
                    {
                        int yy = gy + oy, xx = gx + ox;
                        if (0 <= yy && yy < g.Ny && 0 <= xx && xx < g.Nx)
                        {
                            var kk = yy * g.Nx + xx;
                            if (src[kk] > -900) { s += src[kk]; n++; }
                        }
                    }
                    if (n >= need) grid[k] = (float)(s / n);
                }
        }
    }

    // ================================================================= a place

    public List<(string id, string name, int? bin, double km)> DiscoverPlace(Box box, double radiusKm, (double lon, double lat) anchor, int maxStations, string coopsCache)
    {
        var (raw, _) = FetchText("https://api.tidesandcurrents.noaa.gov/mdapi/prod/webapi/stations.json?type=currentpredictions",
            Path.Combine(coopsCache, "_lists", "currentpredictions.json"));
        var listed = Parse(raw).Arr("stations").ToList();
        var best = new Dictionary<string, ((double depth, long bin) key, JsonNode st, double km)>();
        foreach (var st in listed)
        {
            if (st.Str("type") == "W") continue;
            double lat = st.Num("lat"), lon = st.Num("lng");
            var clat = Math.Min(Math.Max(lat, box.Lat0), box.Lat1);
            var clon = Math.Min(Math.Max(lon, box.Lon0), box.Lon1);
            if (Geo.FlatKm(lon, lat, clon, clat) > radiusKm) continue;
            var key = (st.NumOrNull("depth") ?? 0.0, (long)(st.NumOrNull("currbin") ?? 0));
            var id = st.Str("id")!;
            if (!best.TryGetValue(id, out var cur) || key.CompareTo(cur.key) < 0)
                best[id] = (key, st, Geo.FlatKm(lon, lat, anchor.lon, anchor.lat));
        }
        var manyBins = new Dictionary<string, int>();
        foreach (var st in listed) { var id = st.Str("id")!; manyBins[id] = manyBins.GetValueOrDefault(id) + 1; }
        var picks = best.Values.OrderBy(v => v.km).Take(maxStations);
        return picks.Select(v =>
        {
            var id = v.st.Str("id")!;
            int? bin = manyBins[id] > 1 && v.st.NumOrNull("currbin") is double cb ? (int)cb : null;
            return (id, v.st.Str("name") ?? id, bin, Round(v.km, 1));
        }).ToList();
    }

    public sealed class PlaceRecord
    {
        public List<(string path, long bytes)> Files = new();
        public List<(string id, int? bin)> Stations = new();
        public List<Dictionary<string, object?>> Failed = new();
        public Dictionary<string, object?>? Adcp;
    }

    /// A field handed in by the place orchestrator: u, v and the grid keys of the schema.
    public sealed class PlaceField
    {
        public string File = "";
        public int Nx, Ny;
        public double Lon0, Lat0, Dlon, Dlat;
        public string Source = "";
        public string? ValidUtc;
        public float[] U = Array.Empty<float>(), V = Array.Empty<float>();
        public Dictionary<string, object?> Meta() => D(("file", File), ("nx", Nx), ("ny", Ny), ("lon0", Lon0), ("lat0", Lat0), ("dlon", Dlon), ("dlat", Dlat), ("source", Source), ("valid_utc", ValidUtc));
    }

    public PlaceRecord HarvestPlace(HarvestContext ctx, Polite polite, string place, Box box, double radiusKm, (double lon, double lat)? anchor = null, int maxStations = 8, string? adcp = null, PlaceField? field = null, string? outRoot = null)
    {
        _ctx = ctx;
        _simple ??= new SimpleFetcher(ctx, 0.6, 3);
        _polite = polite;
        var anc = anchor ?? (box.CenterLon, box.CenterLat);
        var outDir = Path.Combine(outRoot ?? Path.Combine("data", "currents"), place);
        var path = Path.Combine(outDir, "currents.json");
        if (File.Exists(path)) throw new HarvestStop($"refusing to overwrite {path} (places only add)");
        var cacheDir = Path.Combine("cache", "currents");
        var picks = DiscoverPlace(box, radiusKm, anc, maxStations, Path.Combine("cache", "coops"));
        ctx.Log($"[{place}] {picks.Count} current station(s): [{string.Join(", ", picks.Select(p => $"('{p.id}', {(p.bin is int b ? b.ToString() : "None")}, {p.km.ToString("R", CultureInfo.InvariantCulture)})"))}]");
        var rec = new PlaceRecord();
        var stations = new List<Dictionary<string, object?>>();
        foreach (var (sid, name, bin, km) in picks)
        {
            try
            {
                var st = FetchTidalStation(sid, name, cacheDir, bin);
                st["km_from_anchor"] = km;
                stations.Add(st);
            }
            catch (Polite.StopException) { throw; }
            catch (Exception e)
            {
                ctx.Log($"[{sid}] FAILED: {e.Message}");
                rec.Failed.Add(D(("id", sid), ("bin", bin), ("why", e.Message)));
            }
        }
        var buoy = adcp is not null ? FetchAdcp(adcp, cacheDir) : null;
        var out_ = D(("generated_utc", NowIso()), ("tidal_stations", stations), ("buoy_adcp", buoy), ("field", null));
        Directory.CreateDirectory(outDir);
        if (field is not null)
        {
            var fpath = Path.Combine(outDir, field.File);
            WriteUv(fpath, field.U, field.V, true);
            rec.Files.Add((fpath, new FileInfo(fpath).Length));
            out_["field"] = field.Meta();
        }
        Save(path, out_, exclusive: true);
        rec.Files.Add((path, new FileInfo(path).Length));
        ctx.Log($"wrote {path} ({stations.Count} tidal stations, adcp {(buoy is not null ? "yes" : "no")}, field {(field is not null ? "yes" : "no")})");
        rec.Stations = stations.Select(s => ((string)s["id"]!, s.GetValueOrDefault("bin") as int?)).ToList();
        rec.Adcp = buoy;
        return rec;
    }
}
