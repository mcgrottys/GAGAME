using System.Globalization;
using Gagame.Harvest.Readers;
using static Gagame.Harvest.JsonUtil;

namespace Gagame.Harvest.Harvesters;

/// M2: GFS-Wave sea state for the Gulf of Maine + NDBC buoy ground truth (harvest_waves.py),
/// through the NOMADS grib filter: a cycle probe, an 11-request calibration once per lifetime
/// (each variable alone, its GRIB keys learned, never a parameter table trusted), then one
/// request per forecast hour; every response cached forever.
public sealed class Waves : IHarvester
{
    public string Name => "waves";
    public string Title => "Sea state (GFS-Wave + NDBC)";
    public string What => "GFS-Wave Hs/Tp/direction for the Gulf of Maine through the NOMADS grib filter, with NDBC buoy ground truth.";
    public string[] Outputs => ["data/sea/seastate.json"];
    public Source[] Sources => [new("NOMADS GFS-Wave", "https://nomads.ncep.noaa.gov/"), new("NDBC realtime2", "https://www.ndbc.noaa.gov/data/realtime2/")];
    public Insert? Insert => new(InsertKind.PlaceBox, "A place's sea: the wave box, with the open-sea point the buoys are matched at.", Extras: ["--sea"]);

    static readonly string[] Filters = ["https://nomads.ncep.noaa.gov/cgi-bin/filter_gfswave.pl", "https://nomads.ncep.noaa.gov/gribfilter.php"];
    const string Ndbc = "https://www.ndbc.noaa.gov/data/realtime2";
    const double TargetLat = 42.35, TargetLonE = 289.35;
    static readonly string[] Vars = ["HTSGW", "PERPW", "DIRPW", "WVHGT", "WVPER", "WVDIR", "SWELL", "SWPER", "SWDIR", "WIND", "WDIR"];
    static readonly int[] Fhs = Enumerable.Range(0, 13).Select(i => i * 4).ToArray();

    public sealed record GribBox(double LeftLon, double RightLon, double TopLat, double BottomLat)
    {
        public Dictionary<string, double> ToJson() => new() { ["leftlon"] = LeftLon, ["rightlon"] = RightLon, ["toplat"] = TopLat, ["bottomlat"] = BottomLat };
        public override string ToString() => $"{{'leftlon': {F(LeftLon)}, 'rightlon': {F(RightLon)}, 'toplat': {F(TopLat)}, 'bottomlat': {F(BottomLat)}}}";
        static string F(double v) => v.ToString("R", CultureInfo.InvariantCulture);
    }

    GribBox _box = new(289.0, 290.0, 42.75, 42.0);
    HarvestContext _ctx = null!;
    SimpleFetcher _fetch = null!;
    Polite? _polite;

    public int Run(HarvestContext ctx, string[] argv)
    {
        _ctx = ctx;
        _fetch = new SimpleFetcher(ctx, 0.6, 4);
        var a = new Args(argv);
        if (a.Has("--place"))
        {
            var sea = a.LonLat("--sea") ?? throw new HarvestStop("--place needs --sea lon,lat");
            var buoys = (a.Opt("--buoys", "") ?? "").Split(',', StringSplitOptions.RemoveEmptyEntries).ToList();
            HarvestPlace(ctx, new Polite(ctx) { Place = a.Opt("--place") }, a.Opt("--place")!, sea, buoys);
            return 0;
        }
        var outDir = Path.Combine("data", "sea");
        var cacheDir = Path.Combine("cache", "gfswave");
        var ndbcCache = Path.Combine("cache", "ndbc");
        Directory.CreateDirectory(outDir);
        var (cycDt, ymd, cyc) = FindCycle(cacheDir);
        ctx.Log($"cycle gfswave {ymd} {cyc:00}z");
        var varmap = Calibrate(ymd, cyc, cacheDir);
        var keyToVar = KeyToVar(varmap);

        var hours = new List<Dictionary<string, object?>>();
        foreach (var fh in Fhs)
        {
            var data = FetchGrib(ymd, cyc, fh, Vars, cacheDir, "all");
            var point = new Dictionary<string, double?>();
            foreach (var m in Grib2.ReadMessages(data))
            {
                if (!keyToVar.TryGetValue(m.Key, out var v)) continue;
                var name = v is "SWELL" or "SWPER" or "SWDIR" ? $"{v}_{m.LevelValue}" : v;
                point[name] = NaNToNull(m.AtLatLon(TargetLat, TargetLonE));
            }
            var entry = HourEntry(fh, point);
            hours.Add(entry);
            if (fh == 0)
            {
                var comb = (Dictionary<string, object?>)entry["combined"]!;
                ctx.Log($"    f000 @ ({TargetLat}, {TargetLonE - 360.0}): Hs {Py(comb["hs"])} m, Tp {Py(comb["tp"])} s, wind {Py(entry["wind_ms"])} m/s, {((List<Dictionary<string, object?>>)entry["partitions"]!).Count} partitions");
            }
        }
        var buoysOut = new Dictionary<string, Dictionary<string, object?>>();
        foreach (var bid in new[] { "44013", "44098" })
        {
            var stamp = DateTime.UtcNow.ToString("yyyyMMddHH", CultureInfo.InvariantCulture);
            var (met, _) = _fetch.Get($"{Ndbc}/{bid}.txt", Path.Combine(ndbcCache, $"{bid}_{stamp}.txt"), maxAgeS: 1800);
            var b = ParseNdbcMet(System.Text.Encoding.UTF8.GetString(met)) ?? new();
            try
            {
                var (spec, _) = _fetch.Get($"{Ndbc}/{bid}.data_spec", Path.Combine(ndbcCache, $"{bid}_{stamp}.data_spec"), maxAgeS: 1800);
                var sp = ParseNdbcSpec(System.Text.Encoding.UTF8.GetString(spec));
                if (sp is not null) b["spectrum"] = sp;
            }
            catch (Exception e) { ctx.Log($"    {bid}.data_spec unavailable ({e.Message})"); }
            buoysOut[bid] = b;
            LogBuoy(bid, b);
        }
        var out_ = D(("generated_utc", NowIso()),
            ("source", "GFS-Wave via NOMADS grib filter (OPeNDAP retired, SCN 25-81) + NDBC realtime2"),
            ("cycle_unix", Unix(cycDt)), ("cycle_label", $"gfswave {ymd} {cyc:00}z"),
            ("point_lat", TargetLat), ("point_lon", TargetLonE - 360.0), ("hours", hours), ("buoys", buoysOut));
        var path = Path.Combine(outDir, "seastate.json");
        Save(path, out_);
        ctx.Log($"wrote {path}");
        var h0c = (Dictionary<string, object?>)hours[0]["combined"]!;
        if (h0c["hs"] is double hs && buoysOut.GetValueOrDefault("44013")?.GetValueOrDefault("hs") is double bhs)
            ctx.Log($"sanity: model f000 Hs {hs:0.00} m vs buoy 44013 Hs {bhs:0.00} m (same neighbourhood expected)");
        return 0;
    }

    static string Py(object? v) => v is null ? "None" : v is double d ? d.ToString("R", CultureInfo.InvariantCulture) : v.ToString()!;
    static double? NaNToNull(double v) => double.IsNaN(v) ? null : v;

    void LogBuoy(string bid, Dictionary<string, object?> b)
    {
        var spec = b.GetValueOrDefault("spectrum") as Dictionary<string, object?>;
        _ctx.Log($"buoy {bid}: Hs {Py(b.GetValueOrDefault("hs"))} m, DPD {Py(b.GetValueOrDefault("dpd"))} s, MWD {Py(b.GetValueOrDefault("mwd"))} degT"
                 + (spec is not null ? $", spectrum {((List<double>)spec["freq_hz"]!).Count} bins" : ""));
    }

    static Dictionary<(int, int, int, int, long), string> KeyToVar(Dictionary<string, List<(int, int, int, int, long)>> varmap)
    {
        var d = new Dictionary<(int, int, int, int, long), string>();
        foreach (var (v, keys) in varmap) foreach (var k in keys) d[k] = v;
        return d;
    }

    public static Dictionary<string, object?> HourEntry(int fh, Dictionary<string, double?> point)
    {
        double? G(string k) => point.GetValueOrDefault(k);
        var parts = new List<Dictionary<string, object?>>();
        var entry = D(("fh", fh), ("wind_ms", G("WIND")), ("wind_from_deg", G("WDIR")),
            ("combined", D(("hs", G("HTSGW")), ("tp", G("PERPW")), ("from_deg", G("DIRPW")))), ("partitions", parts));
        var ws = D(("hs", G("WVHGT")), ("tp", G("WVPER")), ("from_deg", G("WVDIR")), ("kind", "windsea"));
        if (Truthy(ws["hs"]) && Truthy(ws["tp"]) && (double)ws["tp"]! > 0.5) parts.Add(ws);
        foreach (var i in new[] { 1, 2, 3 })
        {
            var sw = D(("hs", G($"SWELL_{i}")), ("tp", G($"SWPER_{i}")), ("from_deg", G($"SWDIR_{i}")), ("kind", "swell"));
            if (Truthy(sw["hs"]) && (double)sw["hs"]! > 0.02 && Truthy(sw["tp"]) && (double)sw["tp"]! > 0.5) parts.Add(sw);
        }
        return entry;
    }

    static bool Truthy(object? v) => v is double d && d != 0.0;

    // ------------------------------------------------------------------ the grib filter

    string FilterUrl(string b, string ymd, int cyc, int fh, IEnumerable<string> vars)
    {
        var p = new List<string>
        {
            "file=" + Uri.EscapeDataString($"gfswave.t{cyc:00}z.global.0p25.f{fh:000}.grib2"), "all_lev=on", "subregion=",
            "leftlon=" + Fmt(_box.LeftLon), "rightlon=" + Fmt(_box.RightLon), "toplat=" + Fmt(_box.TopLat), "bottomlat=" + Fmt(_box.BottomLat),
            "dir=" + Uri.EscapeDataString($"/gfs.{ymd}/{cyc:00}/wave/gridded"),
        };
        foreach (var v in vars) p.Add($"var_{v}=on");
        return b + "?" + string.Join("&", p);
    }

    static string Fmt(double v) => v.ToString("R", CultureInfo.InvariantCulture);

    static bool IsGrib(byte[] d) => d.Length >= 4 && d[0] == 'G' && d[1] == 'R' && d[2] == 'I' && d[3] == 'B' || Find(d) >= 0;
    static int Find(byte[] d)
    {
        for (var i = 0; i + 4 <= Math.Min(d.Length, 128); i++)
            if (d[i] == 'G' && d[i + 1] == 'R' && d[i + 2] == 'I' && d[i + 3] == 'B') return i;
        return -1;
    }

    (byte[] data, bool cached) FetchGribPolite(string url, string cache)
    {
        if (File.Exists(cache)) return (File.ReadAllBytes(cache), true);
        var (data, _) = _polite!.Get(url, null, timeoutS: 90);
        if (IsGrib(data))
        {
            Directory.CreateDirectory(Path.GetDirectoryName(Path.GetFullPath(cache))!);
            using var f = new FileStream(cache, FileMode.CreateNew);
            f.Write(data);
        }
        return (data, false);
    }

    byte[] FetchGrib(string ymd, int cyc, int fh, IEnumerable<string> vars, string cacheDir, string tag)
    {
        var cache = Path.Combine(cacheDir, ymd, $"{cyc:00}z_f{fh:000}_{tag}.grib2");
        string? lastErr = null;
        var varList = vars.ToList();
        foreach (var b in Filters)
        {
            if (_polite is not null)
            {
                byte[] data; bool cached;
                try { (data, cached) = FetchGribPolite(FilterUrl(b, ymd, cyc, fh, varList), cache); }
                catch (Polite.StopException) { throw; }
                catch (Exception e) { lastErr = e.Message; continue; }
                if (IsGrib(data))
                {
                    if (!cached) _ctx.Log($"    fetched f{fh:000} {tag}: {data.Length} B");
                    return data;
                }
                lastErr = $"non-GRIB response ({data.Length} B) from {b[(b.LastIndexOf('/') + 1)..]}";
                continue;
            }
            try
            {
                var (data, cached) = _fetch.Get(FilterUrl(b, ymd, cyc, fh, varList), cache, timeoutS: 60);
                if (IsGrib(data))
                {
                    if (!cached) _ctx.Log($"    fetched f{fh:000} {tag}: {data.Length} B");
                    return data;
                }
                lastErr = $"non-GRIB response ({data.Length} B) from {b[(b.LastIndexOf('/') + 1)..]}";
                File.Delete(cache);
            }
            catch (Exception e) { lastErr = e.Message; }
        }
        throw new InvalidOperationException($"f{fh:000} {tag}: {lastErr}");
    }

    (DateTime cycDt, string ymd, int cyc) FindCycle(string cacheDir)
    {
        var now = DateTime.UtcNow;
        foreach (var dayBack in new[] { 0, 1 })
        {
            var day = now.AddDays(-dayBack);
            foreach (var cyc in new[] { 18, 12, 6, 0 })
            {
                var cycDt = new DateTime(day.Year, day.Month, day.Day, cyc, 0, 0, DateTimeKind.Utc);
                if (cycDt.AddHours(4.5) > now) continue;
                var ymd = cycDt.ToString("yyyyMMdd", CultureInfo.InvariantCulture);
                try
                {
                    FetchGrib(ymd, cyc, Fhs[^1], ["HTSGW"], cacheDir, "probe");
                    return (cycDt, ymd, cyc);
                }
                catch (Polite.StopException) { throw; }
                catch (Exception e) { _ctx.Log($"  cycle {ymd} {cyc:00}z not ready ({e.Message})"); }
            }
        }
        throw new InvalidOperationException("no published gfswave cycle found via the grib filter");
    }

    /// Learn each variable's (discipline, category, number, level) keys from single-variable fetches.
    Dictionary<string, List<(int, int, int, int, long)>> Calibrate(string ymd, int cyc, string cacheDir)
    {
        var mapPath = Path.Combine(cacheDir, "varmap.json");
        if (File.Exists(mapPath))
        {
            var node = Load(mapPath)!.AsObject();
            var d = new Dictionary<string, List<(int, int, int, int, long)>>();
            foreach (var (k, v) in node)
                d[k] = v!.AsArray().Select(t => ((int)t![0]!.GetValue<double>(), (int)t[1]!.GetValue<double>(), (int)t[2]!.GetValue<double>(), (int)t[3]!.GetValue<double>(), (long)t[4]!.GetValue<double>())).ToList();
            return d;
        }
        var varmap = new Dictionary<string, List<(int, int, int, int, long)>>();
        foreach (var v in Vars)
        {
            var data = FetchGrib(ymd, cyc, 0, [v], cacheDir, $"cal_{v}");
            var keys = Grib2.ReadMessages(data).Select(m => m.Key).Distinct().OrderBy(k => k).ToList();
            varmap[v] = keys;
            _ctx.Log($"    {v}: [{string.Join(", ", keys.Select(k => $"({k.Item1}, {k.Item2}, {k.Item3}, {k.Item4}, {k.Item5})"))}]");
        }
        var doc = varmap.ToDictionary(kv => kv.Key, kv => (object?)kv.Value.Select(k => new List<object?> { k.Item1, k.Item2, k.Item3, k.Item4, k.Item5 }).ToList());
        Save(mapPath, doc, indent: 0);
        return varmap;
    }

    // ------------------------------------------------------------------ NDBC

    public static Dictionary<string, object?>? ParseNdbcMet(string text)
    {
        foreach (var ln in text.Split('\n'))
        {
            if (ln.StartsWith('#') || ln.Trim().Length == 0) continue;
            var t = ln.Split((char[]?)null, StringSplitOptions.RemoveEmptyEntries);
            if (t.Length < 12) continue;
            double? Num(int i) => i < t.Length && double.TryParse(t[i], NumberStyles.Float, CultureInfo.InvariantCulture, out var v) ? v : null;
            var obs = new DateTime(int.Parse(t[0]), int.Parse(t[1]), int.Parse(t[2]), int.Parse(t[3]), int.Parse(t[4]), 0, DateTimeKind.Utc);
            return D(("obs_unix", Unix(obs)), ("wdir", Num(5)), ("wspd", Num(6)), ("hs", Num(8)), ("dpd", Num(9)), ("apd", Num(10)), ("mwd", Num(11)));
        }
        return null;
    }

    public static Dictionary<string, object?>? ParseNdbcSpec(string text)
    {
        foreach (var ln in text.Split('\n'))
        {
            if (ln.StartsWith('#') || ln.Trim().Length == 0) continue;
            var t = ln.Split((char[]?)null, StringSplitOptions.RemoveEmptyEntries);
            var freqs = new List<double>(); var dens = new List<double>();
            var i = 6;
            while (i + 1 < t.Length)
            {
                if (!double.TryParse(t[i], NumberStyles.Float, CultureInfo.InvariantCulture, out var d)) break;
                if (!double.TryParse(t[i + 1].Trim('(', ')'), NumberStyles.Float, CultureInfo.InvariantCulture, out var f)) break;
                dens.Add(d); freqs.Add(f);
                i += 2;
            }
            if (freqs.Count > 0) return D(("freq_hz", freqs), ("dens_m2_hz", dens));
        }
        return null;
    }

    // ------------------------------------------------------------------ a place

    static (int r, int c, double lat, double lonE) NearestWaveNode(Grib2Message msg, double lat, double lonE)
    {
        (double d, int r, int c, double la, double lo)? best = null;
        for (var r = 0; r < msg.Nj; r++)
        {
            var la = msg.Lat1 + r * ((msg.Scan & 0x40) != 0 ? msg.Dlat : -msg.Dlat);
            for (var c = 0; c < msg.Ni; c++)
            {
                if (double.IsNaN(msg.Values[r * msg.Ni + c])) continue;
                var lo = Grib2Message.Mod360(msg.Lon1 + c * ((msg.Scan & 0x80) != 0 ? -msg.Dlon : msg.Dlon));
                var d = Math.Sqrt(Sq((lo - lonE) * Math.Cos(lat * Geo.DegToRad)) + Sq(la - lat));
                if (best is null || d < best.Value.d) best = (d, r, c, la, lo);
            }
        }
        if (best is null) throw new InvalidOperationException("every node of the subregion is masked (land)");
        return (best.Value.r, best.Value.c, best.Value.la, best.Value.lo);
    }

    static double Sq(double x) => x * x;

    (List<Dictionary<string, object?>> hours, (int r, int c, double lat, double lonE) node) PointHours(string ymd, int cyc, string cacheDir, Dictionary<(int, int, int, int, long), string> keyToVar, (double lon, double lat) sea)
    {
        var hours = new List<Dictionary<string, object?>>();
        (int r, int c, double lat, double lonE)? node = null;
        foreach (var fh in Fhs)
        {
            var data = FetchGrib(ymd, cyc, fh, Vars, cacheDir, "all");
            var msgs = Grib2.ReadMessages(data).Select(m => (v: keyToVar.GetValueOrDefault(m.Key), m)).ToList();
            if (node is null)
            {
                var hs = msgs.First(x => x.v == "HTSGW").m;
                node = NearestWaveNode(hs, sea.lat, Grib2Message.Mod360(sea.lon));
            }
            var (r, c, _, _) = node.Value;
            var point = new Dictionary<string, double?>();
            foreach (var (v, m) in msgs)
            {
                if (v is null) continue;
                var name = v is "SWELL" or "SWPER" or "SWDIR" ? $"{v}_{m.LevelValue}" : v;
                point[name] = NaNToNull(m.Values[r * m.Ni + c]);
            }
            hours.Add(HourEntry(fh, point));
        }
        return (hours, node!.Value);
    }

    public sealed class PlaceRecord
    {
        public List<(string path, long bytes)> Files = new();
        public (double lat, double lon) Node;
        public string Cycle = "";
        public double CycleUnix;
        public Dictionary<string, double?> Buoys = new();
    }

    /// data/sea/<place>/seastate.json for the node nearest `sea` (lon, lat) and the given NDBC buoys. Never overwrites.
    public PlaceRecord HarvestPlace(HarvestContext ctx, Polite polite, string place, (double lon, double lat) sea, List<string> buoys, string? outRoot = null)
    {
        _ctx = ctx;
        _fetch ??= new SimpleFetcher(ctx, 0.6, 4);
        _polite = polite;
        var outDir = Path.Combine(outRoot ?? Path.Combine("data", "sea"), place);
        var path = Path.Combine(outDir, "seastate.json");
        if (File.Exists(path)) throw new HarvestStop($"refusing to overwrite {path} (places only add)");
        var lonE = Grib2Message.Mod360(sea.lon);
        var x0 = Math.Floor((lonE - 0.5) * 4) / 4;
        var y0 = Math.Floor((sea.lat - 0.375) * 4) / 4;
        _box = new GribBox(x0, x0 + 1.0, y0 + 0.75, y0);
        var cacheDir = Path.Combine("cache", "gfswave", place);
        var (cycDt, ymd, cyc) = FindCycle(cacheDir);
        ctx.Log($"[{place}] cycle gfswave {ymd} {cyc:00}z, box {_box}");
        var varmap = Calibrate(ymd, cyc, Path.Combine("cache", "gfswave"));
        var (hours, node) = PointHours(ymd, cyc, cacheDir, KeyToVar(varmap), sea);
        var c0 = (Dictionary<string, object?>)hours[0]["combined"]!;
        ctx.Log($"[{place}] node ({node.lat:0.000}, {node.lonE - 360.0:0.000}) nearest the seaward point ({Fmt(sea.lon)}, {Fmt(sea.lat)}); f000 Hs {Py(c0["hs"])} m, {((List<Dictionary<string, object?>>)hours[0]["partitions"]!).Count} partitions");

        var ndbcCache = Path.Combine("cache", "ndbc");
        var stamp = DateTime.UtcNow.ToString("yyyyMMddHH", CultureInfo.InvariantCulture);
        var outBuoys = new Dictionary<string, Dictionary<string, object?>>();
        foreach (var bid in buoys)
        {
            var (met, _) = polite.Get($"{Ndbc}/{bid}.txt", Path.Combine(ndbcCache, $"{bid}_{stamp}.txt"));
            var b = ParseNdbcMet(System.Text.Encoding.UTF8.GetString(met)) ?? new();
            try
            {
                var (spec, _) = polite.Get($"{Ndbc}/{bid}.data_spec", Path.Combine(ndbcCache, $"{bid}_{stamp}.data_spec"));
                var sp = ParseNdbcSpec(System.Text.Encoding.UTF8.GetString(spec));
                if (sp is not null) b["spectrum"] = sp;
            }
            catch (Polite.StopException) { throw; }
            catch (Exception e) { ctx.Log($"    {bid}.data_spec unavailable ({e.Message})"); }
            outBuoys[bid] = b;
            LogBuoy(bid, b);
        }
        var out_ = D(("generated_utc", NowIso()),
            ("source", "GFS-Wave via NOMADS grib filter (S3 mirror grids are JPEG 2000) + NDBC realtime2"),
            ("cycle_unix", Unix(cycDt)), ("cycle_label", $"gfswave {ymd} {cyc:00}z"),
            ("point_lat", node.lat), ("point_lon", node.lonE - 360.0),
            ("place", place), ("sea_point", new List<double> { sea.lon, sea.lat }), ("box", _box.ToJson()),
            ("hours", hours), ("buoys", outBuoys));
        Directory.CreateDirectory(outDir);
        Save(path, out_, exclusive: true);
        ctx.Log($"wrote {path}");
        return new PlaceRecord
        {
            Files = { (path, new FileInfo(path).Length) }, Node = (node.lat, node.lonE - 360.0),
            Cycle = $"gfswave {ymd} {cyc:00}z", CycleUnix = Unix(cycDt),
            Buoys = outBuoys.ToDictionary(kv => kv.Key, kv => kv.Value.GetValueOrDefault("obs_unix") as double?),
        };
    }
}
