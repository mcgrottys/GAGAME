using System.Globalization;
using System.Text.Json.Nodes;
using static Gagame.Harvest.JsonUtil;

namespace Gagame.Harvest.Harvesters;

/// M1: NOAA CO-OPS tides for the lower Merrimack, fetched ONCE, politely (harvest_tides.py).
/// Per station: metadata, harmonic constituents, one year of official hourly predictions,
/// then a least-squares fit of amplitude + phase AT THE EXACT HARCON FREQUENCIES, so the
/// engine's stateless sum of cosines reproduces NOAA's predictions to millimetres.
public sealed class Tides : IHarvester
{
    public string Name => "tides";
    public string Title => "Tides (CO-OPS)";
    public string What => "Harmonic constituents and a year of official predictions per station, fit at the exact harcon frequencies.";
    public string[] Outputs => ["data/tides/stations.json"];
    public Source[] Sources => [new("NOAA CO-OPS API", "https://api.tidesandcurrents.noaa.gov/api/prod/"), new("CO-OPS metadata API", "https://api.tidesandcurrents.noaa.gov/mdapi/prod/")];
    public Insert? Insert => new(InsertKind.PlaceBox, "A place's stations: every harmonic station within the box (--radius-km widens it).", Extras: ["--radius-km"]);

    const string Api = "https://api.tidesandcurrents.noaa.gov/api/prod/datagetter";
    const string MdApi = "https://api.tidesandcurrents.noaa.gov/mdapi/prod/webapi";
    const double MinAmpM = 0.001;

    public sealed record Station(string Id, bool OnRiver, string Name);

    public static readonly Station[] Default =
    [
        new("8440452", true, "Merrimack Entrance"), new("8440466", true, "Newburyport"), new("8440273", true, "Salisbury Point"),
        new("8440369", true, "Merrimacport"), new("8440889", true, "Riverside"), new("8443970", false, "Boston"),
    ];

    HarvestContext _ctx = null!;
    Fetcher _fetch = null!;
    Polite? _polite;
    bool? _yearInOne;

    public int Run(HarvestContext ctx, string[] argv)
    {
        _ctx = ctx;
        _fetch = new SimpleFetcher(ctx, gapS: 0.4, tries: 4);
        var a = new Args(argv);
        var year = a.Int("--year", DateTime.UtcNow.Year);
        var outDir = a.Opt("--out", Path.Combine("data", "tides"))!;
        var cacheDir = a.Opt("--cache", Path.Combine("cache", "coops"))!;
        foreach (var x in a.Raw)
            if (x.StartsWith("--") && x is not ("--year" or "--out" or "--cache" or "--place" or "--box" or "--radius-km" or "--anchor"))
                ctx.Log($"ignoring '{x}'");
        var place = a.Opt("--place");
        if (place is null)
        {
            Harvest(Stations(), year, outDir, cacheDir);
            return 0;
        }
        var box = a.BoxOpt() ?? throw new HarvestStop("--place needs --box lon0,lat0,lon1,lat1");
        HarvestPlace(ctx, new Polite(ctx) { Place = place }, place, box, a.Num("--radius-km", 0.0), year, cacheDir, a.LonLat("--anchor"));
        return 0;
    }

    /// Routes this instance's requests through a place's polite fetcher (the orchestrator's use).
    public Tides UsePolite(HarvestContext ctx, Polite polite)
    {
        _ctx = ctx;
        _fetch = new SimpleFetcher(ctx, 0.4, 4);
        _polite = polite;
        return this;
    }

    /// The default stations plus those the water atlas discovered (data/water/extra_tide_stations.json).
    public List<Station> Stations()
    {
        var list = Default.ToList();
        var extra = TryLoad(Path.Combine("data", "water", "extra_tide_stations.json"));
        if (extra is JsonArray arr)
            foreach (var st in arr)
            {
                var id = st.Str("id") ?? "";
                if (list.All(s => s.Id != id)) list.Add(new Station(id, false, st.Str("name") ?? id));
            }
        return list;
    }

    // ------------------------------------------------------------------ fetch

    (JsonNode data, bool cached) FetchJson(string url, string cachePath)
    {
        if (File.Exists(cachePath)) return (Load(cachePath)!, true);
        if (_polite is not null)
        {
            var (raw, c) = _polite.Get(url, cachePath, timeoutS: 60);
            return (Parse(System.Text.Encoding.UTF8.GetString(raw))!, c);
        }
        var (bytes, _) = _fetch.Get(url, null, timeoutS: 30);
        var text = System.Text.Encoding.UTF8.GetString(bytes);
        var node = Parse(text)!;
        Directory.CreateDirectory(Path.GetDirectoryName(Path.GetFullPath(cachePath))!);
        File.WriteAllText(cachePath, node.ToJsonString(), new System.Text.UTF8Encoding(false));
        return (node, false);
    }

    static int MonthDays(int year, int month) => DateTime.DaysInMonth(year, month);

    public sealed record Meta(string Name, double Lat, double Lon);

    (Meta meta, bool cached) StationMeta(string sid, string cacheDir)
    {
        var (data, cached) = FetchJson($"{MdApi}/stations/{sid}.json", Path.Combine(cacheDir, sid, "meta.json"));
        var st = data.Has("stations") ? data.Arr("stations")[0] : data;
        return (new Meta(st.Str("name") ?? sid, st.Num("lat"), st.Has("lng") ? st.Num("lng") : st.Num("lon")), cached);
    }

    /// MLLW - NAVD88 in metres, or null if the station has no NAVD link.
    double? Datums(string sid, string cacheDir)
    {
        JsonNode data;
        try { (data, _) = FetchJson($"{MdApi}/stations/{sid}/datums.json?units=metric", Path.Combine(cacheDir, sid, "datums_metric.json")); }
        catch (Exception e) { _ctx.Log($"    datums unavailable for {sid} ({e.Message})"); return null; }
        var vals = new Dictionary<string, double?>();
        foreach (var d in data.Arr("datums")) vals[d.Str("name") ?? ""] = d.NumOrNull("value");
        if (vals.TryGetValue("MLLW", out var m) && vals.TryGetValue("NAVD88", out var n) && m is not null && n is not null) return m - n;
        return null;
    }

    public sealed record Harcon(string Name, double SpeedDph, double AmpM, double PhaseGmtDeg);

    (List<Harcon> cons, bool cached) GetHarcon(string sid, string cacheDir)
    {
        var (data, cached) = FetchJson($"{MdApi}/stations/{sid}/harcon.json?units=metric", Path.Combine(cacheDir, sid, "harcon_metric.json"));
        var out_ = new List<Harcon>();
        foreach (var c in data.Arr("HarmonicConstituents"))
            out_.Add(new Harcon(c.Str("name") ?? "?", c.Num("speed"), c.Num("amplitude"), c.Num("phase_GMT")));
        // The units the answer DECLARES (CO-OPS has ignored the request's `units` before, but its answer says
        // which it holds). The old test, an M2 over 2.5 m read as feet, shrank real tides: Eastport's M2 is 2.6 m.
        if ((data.Str("units") ?? "").StartsWith("feet", StringComparison.OrdinalIgnoreCase))
        {
            _ctx.Log($"    harcon for {sid} says feet; converting to metres");
            out_ = out_.Select(c => c with { AmpM = c.AmpM * 0.3048 }).ToList();
        }
        return (out_, cached);
    }

    static string Q(Dictionary<string, string> p) => string.Join("&", p.Select(kv => Uri.EscapeDataString(kv.Key) + "=" + Uri.EscapeDataString(kv.Value)));

    static double ParseT(string t) => Unix(DateTime.ParseExact(t, "yyyy-MM-dd HH:mm", CultureInfo.InvariantCulture, DateTimeStyles.AssumeUniversal | DateTimeStyles.AdjustToUniversal));

    (List<double> times, List<double> values)? WholeYear(string sid, int year, string cacheDir)
    {
        if (_polite is null || _yearInOne == false) return null;
        if (Enumerable.Range(1, 12).All(m => File.Exists(Path.Combine(cacheDir, sid, $"pred_{year}_{m:00}.json")))) return null;
        var p = new Dictionary<string, string>
        {
            ["product"] = "predictions", ["application"] = "GAGAME", ["station"] = sid,
            ["begin_date"] = $"{year}0101", ["end_date"] = $"{year}1231",
            ["datum"] = "MLLW", ["units"] = "metric", ["time_zone"] = "gmt", ["interval"] = "h", ["format"] = "json",
        };
        var cache = Path.Combine(cacheDir, sid, $"pred_{year}_all.json");
        var fresh = !File.Exists(cache);
        JsonNode data;
        try { (data, _) = FetchJson(Api + "?" + Q(p), cache); }
        catch (HttpError e)
        {
            _ctx.Log($"    one-request year refused for {sid} (HTTP {e.Code}); monthly from now on");
            _yearInOne = false;
            return null;
        }
        var preds = data.Arr("predictions");
        if (data.Has("error") || preds.Count < 8700)
        {
            _ctx.Log($"    one-request year refused for {sid} ({data.Obj("error").Str("message") ?? preds.Count.ToString()}); monthly from now on");
            _yearInOne = false;
            return null;
        }
        if (fresh) { _yearInOne = true; _ctx.Log($"    fetched {year} in one request: {preds.Count} h"); }
        var times = new List<double>(); var values = new List<double>();
        foreach (var x in preds) { times.Add(ParseT(x.Str("t")!)); values.Add(x.Num("v")); }
        return (times, values);
    }

    (List<double> times, List<double> values) PredictionsYear(string sid, int year, string cacheDir)
    {
        if (WholeYear(sid, year, cacheDir) is { } whole) return whole;
        var times = new List<double>(); var values = new List<double>();
        for (var month = 1; month <= 12; month++)
        {
            var nd = MonthDays(year, month);
            var p = new Dictionary<string, string>
            {
                ["product"] = "predictions", ["application"] = "GAGAME", ["station"] = sid,
                ["begin_date"] = $"{year}{month:00}01", ["end_date"] = $"{year}{month:00}{nd:00}",
                ["datum"] = "MLLW", ["units"] = "metric", ["time_zone"] = "gmt", ["interval"] = "h", ["format"] = "json",
            };
            var (data, cached) = FetchJson(Api + "?" + Q(p), Path.Combine(cacheDir, sid, $"pred_{year}_{month:00}.json"));
            if (data.Has("error")) throw new InvalidOperationException($"{sid} {year}-{month:00}: CO-OPS error: {data.Obj("error").Str("message") ?? data["error"]!.ToJsonString()}");
            var preds = data.Arr("predictions");
            if (preds.Count == 0) throw new InvalidOperationException($"{sid} {year}-{month:00}: empty predictions");
            foreach (var x in preds) { times.Add(ParseT(x.Str("t")!)); values.Add(x.Num("v")); }
            if (!cached) _ctx.Log($"    fetched {year}-{month:00}: {preds.Count} h");
        }
        return (times, values);
    }

    // ------------------------------------------------------------------ the fit

    static double[] CholeskySolve(double[,] g, double[] r)
    {
        var n = r.Length;
        var l = new double[n, n];
        for (var i = 0; i < n; i++)
            for (var j = 0; j <= i; j++)
            {
                var s = g[i, j];
                for (var k = 0; k < j; k++) s -= l[i, k] * l[j, k];
                if (i == j)
                {
                    if (s <= 0) throw new InvalidOperationException($"normal matrix not SPD at {i} (s={s:g}); duplicate constituent frequencies?");
                    l[i, i] = Math.Sqrt(s);
                }
                else l[i, j] = s / l[j, j];
            }
        var y = new double[n];
        for (var i = 0; i < n; i++)
        {
            var s = r[i];
            for (var k = 0; k < i; k++) s -= l[i, k] * y[k];
            y[i] = s / l[i, i];
        }
        var x = new double[n];
        for (var i = n - 1; i >= 0; i--)
        {
            var s = y[i];
            for (var k = i + 1; k < n; k++) s -= l[k, i] * x[k];
            x[i] = s / l[i, i];
        }
        return x;
    }

    /// Least squares of v ~ mean + sum a_i cos(w_i tau) + b_i sin(w_i tau), tau = t - epoch.
    public static (double mean, List<(double a, double b)> ab, double rms, double max) Fit(List<double> times, List<double> values, double[] omegas, double epoch)
    {
        var nC = omegas.Length;
        var m = 1 + 2 * nC;
        var nS = times.Count;
        var g = new double[m, m];
        var r = new double[m];
        var row = new double[m];
        row[0] = 1.0;
        for (var s = 0; s < nS; s++)
        {
            var tau = times[s] - epoch;
            for (var i = 0; i < nC; i++)
            {
                var wt = omegas[i] * tau;
                row[1 + 2 * i] = Math.Cos(wt);
                row[2 + 2 * i] = Math.Sin(wt);
            }
            var v = values[s];
            for (var i = 0; i < m; i++)
            {
                var ri = row[i];
                r[i] += ri * v;
                for (var j = i; j < m; j++) g[i, j] += ri * row[j];
            }
        }
        for (var i = 0; i < m; i++) for (var j = 0; j < i; j++) g[i, j] = g[j, i];
        var x = CholeskySolve(g, r);
        double sq = 0, mx = 0;
        for (var s = 0; s < nS; s++)
        {
            var tau = times[s] - epoch;
            var h = x[0];
            for (var i = 0; i < nC; i++) h += x[1 + 2 * i] * Math.Cos(omegas[i] * tau) + x[2 + 2 * i] * Math.Sin(omegas[i] * tau);
            var e = h - values[s];
            sq += e * e;
            mx = Math.Max(mx, Math.Abs(e));
        }
        var ab = new List<(double, double)>();
        for (var i = 0; i < nC; i++) ab.Add((x[1 + 2 * i], x[2 + 2 * i]));
        return (x[0], ab, Math.Sqrt(sq / nS), mx);
    }

    // ------------------------------------------------------------------ places

    /// Harmonic (type R) stations inside the box or within radiusKm of it, nearest first.
    public (List<Station> found, List<Dictionary<string, object?>> subordinate) DiscoverPlace(Box box, double radiusKm, string cacheDir, (double lon, double lat)? anchor)
    {
        var (data, _) = FetchJson($"{MdApi}/stations.json?type=tidepredictions", Path.Combine(cacheDir, "_lists", "tidepredictions.json"));
        var anc = anchor ?? (box.CenterLon, box.CenterLat);
        var found = new List<(double d, double da, string id, string name)>();
        var sub = new List<Dictionary<string, object?>>();
        foreach (var st in data.Arr("stations"))
        {
            double lat = st.Num("lat"), lon = st.Num("lng");
            var d = Geo.BoxKm(lat, lon, box);
            if (d > radiusKm) continue;
            if (st.Str("type") != "R")
            {
                sub.Add(D(("id", st.Str("id")), ("name", st.Str("name") ?? "?"), ("reference_id", st.Str("reference_id")), ("km", Round(d, 1))));
                continue;
            }
            found.Add((d, Geo.Haversine_km(lat, lon, anc.lat, anc.lon), st.Str("id")!, st.Str("name") ?? st.Str("id")!));
        }
        found.Sort((x, y) => x.d != y.d ? x.d.CompareTo(y.d) : x.da != y.da ? x.da.CompareTo(y.da) : string.CompareOrdinal(x.id, y.id));
        return (found.Select(f => new Station(f.id, false, f.name)).ToList(), sub);
    }

    public sealed class Record
    {
        public List<(string path, long bytes)> Files = new();
        public List<Dictionary<string, object?>> Failed = new();
        public List<Dictionary<string, object?>> Stations = new();
        public List<Dictionary<string, object?>> Subordinate = new();
        public int Year;
        public double EpochUnix;
    }

    /// The place mode: discover, fit, write data/tides/<place>/ (never overwriting).
    public Record HarvestPlace(HarvestContext ctx, Polite polite, string place, Box box, double radiusKm, int? year = null, string? cacheDir = null, (double lon, double lat)? anchor = null, string? outRoot = null)
    {
        _ctx = ctx;
        _fetch ??= new SimpleFetcher(ctx, 0.4, 4);
        _polite = polite;
        var y = year ?? DateTime.UtcNow.Year;
        cacheDir ??= Path.Combine("cache", "coops");
        var outDir = Path.Combine(outRoot ?? Path.Combine("data", "tides"), place);
        var (stations, sub) = DiscoverPlace(box, radiusKm, cacheDir, anchor);
        ctx.Log($"[{place}] {stations.Count} harmonic station(s) within {radiusKm:g} km of {box}: [{string.Join(", ", stations.Select(s => "'" + s.Id + "'"))}]; {sub.Count} subordinate station(s) have no constituents of their own (not fitted)");
        var rec = Harvest(stations, y, outDir, cacheDir, exclusive: true, tolerant: true);
        rec.Subordinate = sub;
        return rec;
    }

    /// Fit every station and write stations.json + official sidecars into outDir.
    public Record Harvest(List<Station> stations, int year, string outDir, string cacheDir, bool exclusive = false, bool tolerant = false)
    {
        if (exclusive && File.Exists(Path.Combine(outDir, "stations.json")))
            throw new HarvestStop($"refusing to overwrite {outDir}/stations.json (places only add)");
        var rec = Fit(stations, year, outDir, cacheDir, exclusive, tolerant);
        var doc = D(
            ("generated_utc", NowIso()),
            ("source", "NOAA CO-OPS harcon frequencies + least-squares fit to official hourly predictions (see this script's docstring)"),
            ("year", year), ("epoch_unix", rec.EpochUnix), ("stations", rec.Stations));
        var path = Path.Combine(outDir, "stations.json");
        Save(path, doc, exclusive);
        _ctx.Log($"wrote {path} ({rec.Stations.Count} stations) -- the engine is now network-free");
        rec.Files.Add((path, new FileInfo(path).Length));
        return rec;
    }

    /// Fit every station and write its official sidecar into outDir: the station entries of
    /// stations.json, for a caller that writes the document itself (a place, a composite).
    public Record Fit(List<Station> stations, int year, string outDir, string cacheDir, bool exclusive = false, bool tolerant = false)
    {
        var epoch = Unix(new DateTime(year, 1, 1, 0, 0, 0, DateTimeKind.Utc));
        Directory.CreateDirectory(outDir);
        _ctx.Log($"harvest_tides: year {year}, epoch {epoch:0}, normal-equations solver");
        var rec = new Record { Year = year, EpochUnix = epoch };
        (double lat, double lon)? prev = null;
        var riverKm = 0.0;
        foreach (var (sid, onRiver, shortName) in stations)
        {
            _ctx.Log($"[{sid}] {shortName}");
            Meta meta;
            List<Harcon> harcon;
            double? mllwMinusNavd;
            List<double> times, values;
            try
            {
                (meta, _) = StationMeta(sid, cacheDir);
                (harcon, _) = GetHarcon(sid, cacheDir);
                mllwMinusNavd = Datums(sid, cacheDir);
                if (mllwMinusNavd is double mn) _ctx.Log($"    MLLW - NAVD88 = {mn:+0.000;-0.000} m");
                if (harcon.Count == 0 && tolerant) throw new InvalidOperationException("no harmonic constituents published");
                (times, values) = PredictionsYear(sid, year, cacheDir);
            }
            catch (Exception e) when (tolerant && e is not Polite.StopException)
            {
                _ctx.Log($"    SKIPPED {sid}: {e.Message}");
                rec.Failed.Add(D(("id", sid), ("name", shortName), ("why", e.Message)));
                continue;
            }
            _ctx.Log($"    {meta.Name} ({meta.Lat:0.0000}, {meta.Lon:0.0000}): {harcon.Count} constituents, {times.Count} official hours");

            var use = new List<Harcon>();
            var seen = new HashSet<double>();
            foreach (var c in harcon)
            {
                if (c.AmpM < MinAmpM || seen.Contains(c.SpeedDph)) continue;
                seen.Add(c.SpeedDph);
                use.Add(c);
            }
            var omegas = use.Select(c => c.SpeedDph * Math.PI / 180.0 / 3600.0).ToArray();
            var (mean, ab, rms, mx) = Fit(times, values, omegas, epoch);
            _ctx.Log($"    fit: {use.Count} constituents, mean {mean:0.000} m MLLW, rms {rms * 1000:0.0} mm, max {mx * 1000:0.0} mm");

            var coeffs = new List<Dictionary<string, object?>>();
            var warn = new List<string>();
            for (var i = 0; i < use.Count; i++)
            {
                var (a, b) = ab[i];
                var amp = Math.Sqrt(a * a + b * b);
                var phase = Math.Atan2(-b, a);
                coeffs.Add(D(("name", use[i].Name), ("speed_dph", use[i].SpeedDph), ("amp_m", Round(amp, 6)), ("phase_rad", Round(phase, 6))));
                if (use[i].AmpM > 0.05)
                {
                    var ratio = amp / use[i].AmpM;
                    if (!(0.70 <= ratio && ratio <= 1.30)) warn.Add($"{use[i].Name} fitted/harcon = {ratio:0.00}");
                }
            }
            if (warn.Count > 0) _ctx.Log("    WARNING amplitude ratios out of nodal range: " + string.Join(", ", warn));
            var top = use.Zip(ab).OrderByDescending(p => Math.Sqrt(p.Second.a * p.Second.a + p.Second.b * p.Second.b)).Take(4);
            _ctx.Log("    top: " + string.Join("  ", top.Select(p =>
            {
                var h = Math.Sqrt(p.Second.a * p.Second.a + p.Second.b * p.Second.b);
                return $"{p.First.Name}={h:0.000}m(x{h / p.First.AmpM:0.000})";
            })));

            if (onRiver)
            {
                if (prev is { } pv) riverKm += Geo.Haversine_km(pv.lat, pv.lon, meta.Lat, meta.Lon);
                prev = (meta.Lat, meta.Lon);
            }
            var offFile = $"official_{sid}_{year}.f32";
            var offPath = Path.Combine(outDir, offFile);
            Bin.WriteF32(offPath, values.Select(v => (float)v).ToArray(), exclusive);
            rec.Files.Add((offPath, 4L * values.Count));

            rec.Stations.Add(D(
                ("id", sid), ("name", shortName), ("full_name", meta.Name), ("lat", meta.Lat), ("lon", meta.Lon),
                ("mllw_minus_navd_m", mllwMinusNavd),
                ("river_km", onRiver ? Round(riverKm, 3) : null),
                ("mean_mllw_m", Round(mean, 5)), ("fit_rms_m", Round(rms, 5)), ("fit_max_m", Round(mx, 5)),
                ("official_file", offFile), ("official_start_unix", times[0]), ("official_dt_s", 3600), ("official_n", values.Count),
                ("coeffs", coeffs)));
        }
        return rec;
    }
}
