using System.Globalization;
using System.IO.Compression;
using System.Text.RegularExpressions;
using Gagame.Harvest.Readers;
using static Gagame.Harvest.JsonUtil;

namespace Gagame.Harvest.Harvesters;

/// M6v: THE WATER ATLAS registry (harvest_water.py): every water-parameter source near the
/// focus region, each declaring what it measures and where; the survey points for the vpack;
/// the harmonic stations harvest_tides should also fit; and --eot20, the global constituent
/// atlas converted to flat (re, im) rasters (netCDF-4 through PureHDF, in place of netCDF4).
public sealed class Water : IHarvester
{
    public string Name => "water";
    public string Title => "Water atlas registry";
    public string What => "Every water source near the focus: tide and current stations, buoys, gauges, and the EOT20 global tide.";
    public string[] Outputs => ["data/water/registry.json"];
    public Source[] Sources => [new("EOT20 (SEANOE, CC BY 4.0)", "https://doi.org/10.17882/79489"), new("NDBC station table", "https://www.ndbc.noaa.gov/data/stations/station_table.txt")];
    public Insert? Insert => new(InsertKind.Flags, "Optional products.", Flags: ["--eot20"]);
    public string? Note => "--eot20 fetches the 2.3 GB constituent grids.";

    const string MdApi = "https://api.tidesandcurrents.noaa.gov/mdapi/prod/webapi";
    const string Eot20Url = "https://www.seanoe.org/data/00683/79489/data/85762.zip";
    static string CacheDir => Path.Combine("cache", "coops");
    static string OutDir => Path.Combine("data", "water");
    static readonly Box BoxNe = new(-71.15, 42.20, -70.35, 43.05);
    static readonly string[] TidePinned = ["8440452", "8440466", "8440273", "8440369", "8440889", "8443970"];
    static readonly string[] Buoys = ["44013", "44029", "44098"];
    static readonly List<Dictionary<string, object?>> RiverGauges =
    [
        D(("id", "01100000"), ("name", "Merrimack R at Lowell"), ("lat", 42.6459), ("lon", -71.2981), ("parameter", "discharge_cms"), ("api", "waterservices.usgs.gov (migrate to api.waterdata.usgs.gov by Q1 2027)")),
        D(("id", "01100500"), ("name", "Merrimack R at Lawrence"), ("lat", 42.7028), ("lon", -71.1651), ("parameter", "discharge_cms"), ("api", "same")),
    ];

    HarvestContext _ctx = null!;
    SimpleFetcher _fetch = null!;

    List<Dictionary<string, object?>> DiscoverTideStations()
    {
        var (raw, cached) = _fetch.GetText($"{MdApi}/stations.json?type=tidepredictions", Path.Combine(CacheDir, "_lists", "tidepredictions.json"), timeoutS: 60);
        var out_ = new List<Dictionary<string, object?>>();
        foreach (var st in Parse(raw).Arr("stations"))
        {
            double lat = st.Num("lat"), lon = st.Num("lng");
            if (!BoxNe.Contains(lon, lat)) continue;
            out_.Add(D(("id", st.Str("id")), ("name", st.Str("name") ?? "?"), ("lat", lat), ("lon", lon), ("type", st.Str("type") ?? "?")));
        }
        _ctx.Log($"[water] tide stations in box: {out_.Count} ({(cached ? "cache" : "fetched")})");
        return out_;
    }

    List<Dictionary<string, object?>> DiscoverCurrentStations()
    {
        var (raw, cached) = _fetch.GetText($"{MdApi}/stations.json?type=currentpredictions", Path.Combine(CacheDir, "_lists", "currentpredictions.json"), timeoutS: 60);
        var out_ = new List<Dictionary<string, object?>>();
        foreach (var st in Parse(raw).Arr("stations"))
        {
            double lat = st.Num("lat"), lon = st.Num("lng");
            if (!BoxNe.Contains(lon, lat)) continue;
            out_.Add(D(("id", st.Str("id")), ("bin", st.Has("currbin") ? Plain(st!["currbin"]) : null), ("name", st.Str("name") ?? "?"), ("lat", lat), ("lon", lon), ("type", st.Str("type") ?? "?")));
        }
        _ctx.Log($"[water] current stations in box: {out_.Count} ({(cached ? "cache" : "fetched")})");
        return out_;
    }

    List<Dictionary<string, object?>> BuoyPositions()
    {
        var (raw, cached) = _fetch.GetText("https://www.ndbc.noaa.gov/data/stations/station_table.txt", Path.Combine(CacheDir, "_lists", "ndbc_station_table.txt"), timeoutS: 60);
        var out_ = new List<Dictionary<string, object?>>();
        var want = Buoys.Select(b => b.ToLowerInvariant()).ToHashSet();
        foreach (var line in raw.Split('\n'))
        {
            if (line.StartsWith('#') || !line.Contains('|')) continue;
            var f = line.Split('|').Select(c => c.Trim()).ToArray();
            if (!want.Contains(f[0].ToLowerInvariant())) continue;
            var loc = f[6].Split((char[]?)null, StringSplitOptions.RemoveEmptyEntries);
            if (loc.Length < 4 || !double.TryParse(loc[0], NumberStyles.Float, CultureInfo.InvariantCulture, out var la) || !double.TryParse(loc[2], NumberStyles.Float, CultureInfo.InvariantCulture, out var lo)) continue;
            var lat = la * (loc[1] == "N" ? 1 : -1);
            var lon = lo * (loc[3] == "E" ? 1 : -1);
            out_.Add(D(("id", f[0]), ("name", f[4].Length > 0 ? f[4] : f[0]), ("lat", lat), ("lon", lon), ("payload", f[1])));
        }
        _ctx.Log($"[water] buoys located: [{string.Join(", ", out_.Select(b => "'" + b["id"] + "'"))}] ({(cached ? "cache" : "fetched")})");
        return out_;
    }

    void ConvertEot20(string[] constituents)
    {
        var big = _ctx.DataCache;
        Directory.CreateDirectory(big);
        var zpath = Path.Combine(big, "eot20_85762.zip");
        if (!File.Exists(zpath))
        {
            _ctx.Log($"[eot20] downloading {Eot20Url} (~2 GB, once) -> {zpath}");
            _fetch.DownloadBig([Eot20Url], zpath, minBytes: 1_000_000_000, timeoutS: 7200, tag: "EOT20");
            _ctx.Log($"[eot20] downloaded {new FileInfo(zpath).Length / 1e9:0.00} GB");
        }
        else _ctx.Log($"[eot20] zip cached ({new FileInfo(zpath).Length / 1e9:0.00} GB)");

        Directory.CreateDirectory(OutDir);
        var outer = ZipFile.OpenRead(zpath);
        ZipArchive z;
        if (outer.GetEntry("ocean_tides.zip") is { } nested)
        {
            var inner = Path.Combine(big, "ocean_tides.zip");
            if (!File.Exists(inner))
            {
                _ctx.Log("[eot20] extracting nested ocean_tides.zip");
                nested.ExtractToFile(inner);
            }
            z = ZipFile.OpenRead(inner);
        }
        else z = outer;
        var names = z.Entries.Select(e => e.FullName).ToList();
        foreach (var con in constituents)
        {
            var outp = Path.Combine(OutDir, $"eot20_{con}.rg32");
            if (File.Exists(outp)) { _ctx.Log($"[eot20] {con}: already converted"); continue; }
            var lc = con.ToLowerInvariant();
            var cand = names.Where(n => n.ToLowerInvariant().EndsWith($"/{lc}_ocean_eot20.nc") || n.ToLowerInvariant().EndsWith($"ocean_tides/{lc}.nc")).ToList();
            if (cand.Count == 0)
                cand = names.Where(n => n.ToLowerInvariant().Contains("ocean") && n.ToLowerInvariant().Split('/')[^1].StartsWith(lc) && n.EndsWith(".nc")).ToList();
            if (cand.Count == 0)
            {
                _ctx.Log($"[eot20] {con}: no matching nc in zip (names like: [{string.Join(", ", names.Where(n => n.EndsWith(".nc")).Take(3).Select(n => "'" + n + "'"))}])");
                continue;
            }
            var member = cand[0];
            _ctx.Log($"[eot20] {con}: extracting {member}");
            var tmp = Path.Combine(big, "_eot20_member.nc");
            z.GetEntry(member)!.ExtractToFile(tmp, overwrite: true);
            double[] lat, lon, re, im;
            List<int> shape;
            string units;
            using (var nc = new NetCdf4(tmp))
            {
                (lat, _) = nc.Read("lat");
                (lon, _) = nc.Read("lon");
                units = (nc.Attr("real", "units") ?? "").Trim().ToLowerInvariant();
                (re, shape) = nc.Read("real");
                (im, _) = nc.Read("imag");
                var fillR = nc.Attr("real", "_FillValue");
                var fillI = nc.Attr("imag", "_FillValue");
                if (fillR is not null && double.TryParse(fillR, NumberStyles.Float, CultureInfo.InvariantCulture, out var fr))
                    for (var i = 0; i < re.Length; i++) if (re[i] == fr) re[i] = double.NaN;
                if (fillI is not null && double.TryParse(fillI, NumberStyles.Float, CultureInfo.InvariantCulture, out var fi))
                    for (var i = 0; i < im.Length; i++) if (im[i] == fi) im[i] = double.NaN;
            }
            File.Delete(tmp);
            int rows = shape[^2], cols = shape[^1];
            if (lat[0] < lat[^1])
            {
                re = FlipRows(re, rows, cols);
                im = FlipRows(im, rows, cols);
                Array.Reverse(lat);
            }
            double scale;
            if (units.StartsWith("cm") || units.StartsWith("centimet")) scale = 0.01;
            else if (units is "m" or "meter" or "meters" or "metre" or "metres") scale = 1.0;
            else
            {
                var mx = 0.0;
                for (var i = 0; i < re.Length; i++) { var a = Math.Sqrt(re[i] * re[i] + im[i] * im[i]); if (!double.IsNaN(a)) mx = Math.Max(mx, a); }
                scale = mx > 30.0 ? 0.01 : 1.0;
                _ctx.Log($"[eot20] {con}: WARNING no units attribute, heuristic scale {scale}");
            }
            for (var i = 0; i < re.Length; i++) { re[i] *= scale; im[i] *= scale; }
            var amp = new double[re.Length];
            for (var i = 0; i < re.Length; i++) amp[i] = Math.Sqrt(re[i] * re[i] + im[i] * im[i]);
            var finite = amp.Where(a => !double.IsNaN(a)).OrderBy(a => a).ToArray();
            var p999 = Percentile(finite, 99.9);
            var cap = Math.Max(4.0 * p999, 0.02);
            var nHot = 0;
            for (var i = 0; i < re.Length; i++)
            {
                if (!(amp[i] > cap)) continue;
                nHot++;
                var shrink = cap / Math.Max(amp[i], 1e-12);
                re[i] *= shrink; im[i] *= shrink;
            }
            if (nHot > 0) _ctx.Log($"[eot20] {con}: clamped {nHot} outlier texels to {cap:0.000} m (p99.9 {p999:0.000} m)");
            var rg = new float[re.Length * 2];
            var maxAmp = 0.0;
            for (var i = 0; i < re.Length; i++)
            {
                var r = double.IsNaN(re[i]) ? 0.0 : re[i];
                var m = double.IsNaN(im[i]) ? 0.0 : im[i];
                rg[2 * i] = (float)r; rg[2 * i + 1] = (float)m;
                maxAmp = Math.Max(maxAmp, Math.Sqrt(r * r + m * m));
            }
            Bin.WriteF32(outp, rg);
            Save(Path.Combine(OutDir, $"eot20_{con}.json"), D(("file", Path.GetFileName(outp)), ("rows", rows), ("cols", cols),
                ("lat_north", lat[0]), ("lat_south", lat[^1]), ("lon_first", lon[0]), ("lon_last", lon[^1]), ("units", "m (re, im interleaved)"),
                ("row0", "north"), ("nodata", "0 (land/out-of-domain)"), ("source", "EOT20 (Hart-Davis et al. 2021, doi:10.17882/79489, CC-BY 4.0)"),
                ("phase_convention", "Greenwich lag: h = re*cos(w*t_astro) + im*sin; the ENGINE re-references to the fit epoch via the station consensus (the epoch ladder -- see WaterAtlas)")));
            _ctx.Log($"[eot20] {con}: {cols}x{rows} -> {outp} (max amp {maxAmp:0.00} m)");
        }
        if (!ReferenceEquals(z, outer)) z.Dispose();
        outer.Dispose();
    }

    static double[] FlipRows(double[] a, int rows, int cols)
    {
        var o = new double[a.Length];
        for (var r = 0; r < rows; r++) Array.Copy(a, (rows - 1 - r) * cols, o, r * cols, cols);
        return o;
    }

    /// numpy.nanpercentile with linear interpolation over the sorted finite values.
    static double Percentile(double[] sorted, double p)
    {
        if (sorted.Length == 0) return double.NaN;
        var k = (sorted.Length - 1) * p / 100.0;
        var f = (int)Math.Floor(k);
        var c = Math.Min(f + 1, sorted.Length - 1);
        return sorted[f] + (sorted[c] - sorted[f]) * (k - f);
    }

    public int Run(HarvestContext ctx, string[] argv)
    {
        _ctx = ctx;
        _fetch = new SimpleFetcher(ctx, 0.5, 1);
        var eot = argv.Contains("--eot20");
        Directory.CreateDirectory(OutDir);
        var tide = DiscoverTideStations();
        var curr = DiscoverCurrentStations();
        var buoys = BuoyPositions();
        foreach (var t in tide)
        {
            t["pinned"] = TidePinned.Contains((string)t["id"]!);
            t["harmonic"] = (string?)t["type"] == "R";
        }
        var converted = Directory.Exists(OutDir)
            ? Directory.GetFiles(OutDir, "eot20_*.rg32").Select(f => Path.GetFileName(f)[6..^5]).OrderBy(x => x, StringComparer.Ordinal).ToList()
            : new List<string>();
        var reg = D(
            ("_readme", new List<string>
            {
                "THE WATER SURVEY: every water-parameter source near the focus region, each",
                "declaring position (WGS84), datum, and what it measures. Heterogeneity is",
                "explicit: harmonic stations carry 37-constituent fits (data/tides), currents",
                "carry prediction clocks, buoys carry spot obs + spectra, gauges carry",
                "discharge, EOT20 carries the global low-medium constituent base.",
            }),
            ("box", D(("lat0", BoxNe.Lat0), ("lat1", BoxNe.Lat1), ("lon0", BoxNe.Lon0), ("lon1", BoxNe.Lon1))),
            ("tide_stations", tide), ("current_stations", curr), ("buoys", buoys), ("river_gauges", RiverGauges),
            ("global_base", D(("name", "eot20"), ("crs", "EPSG:4326 equirect 1/8 deg"), ("coverage", "66S..66N"),
                ("constituents_converted", converted), ("license", "CC-BY 4.0 (Hart-Davis et al. 2021, doi:10.17882/79489)"))));
        Save(Path.Combine(OutDir, "registry.json"), reg);
        ctx.Log($"[water] registry: {tide.Count} tide, {curr.Count} current, {buoys.Count} buoys, {RiverGauges.Count} gauges -> data/water/registry.json");

        var feats = new List<object?>();
        Dictionary<string, object?> Feat(string kind, Dictionary<string, object?> s, bool? harmonic = null)
        {
            var props = D(("kind", kind), ("id", s["id"]), ("name", s["name"]));
            if (harmonic is not null) props["harmonic"] = harmonic;
            return D(("type", "Feature"), ("properties", props), ("geometry", D(("type", "Point"), ("coordinates", new List<object?> { s["lon"], s["lat"] }))));
        }
        foreach (var t in tide) feats.Add(Feat("tide", t, (bool)t["harmonic"]!));
        foreach (var c in curr) feats.Add(Feat("current", c));
        foreach (var b in buoys) feats.Add(Feat("buoy", b));
        foreach (var g in RiverGauges) feats.Add(Feat("river", g));
        Directory.CreateDirectory(Path.Combine("cache", "vectors"));
        Save(Path.Combine("cache", "vectors", "water_points.geojson"), D(("type", "FeatureCollection"), ("features", feats)), indent: 0);
        ctx.Log($"[water] {feats.Count} survey points -> cache/vectors/water_points.geojson (rerun harvest_vectors to fold into the vpack)");

        var known = TidePinned.ToHashSet();
        var extra = tide.Where(t => (bool)t["harmonic"]! && !known.Contains((string)t["id"]!)).ToList();
        if (extra.Count > 0)
        {
            Save(Path.Combine(OutDir, "extra_tide_stations.json"), extra.Select(t => D(("id", t["id"]), ("name", t["name"]))).ToList());
            ctx.Log($"[water] {extra.Count} harmonic stations need M1 fits: [{string.Join(", ", extra.Select(t => "'" + t["id"] + " " + t["name"] + "'"))}]");
            ctx.Log("[water]   -> harvest_tides.py picks up data/water/extra_tide_stations.json");
        }
        if (eot) ConvertEot20(["M2", "S2", "N2", "K1", "O1", "K2", "P1", "Q1", "2N2", "T2", "J1", "M4", "SA", "SSA"]);
        else ctx.Log("[water] run with --eot20 for the 2 GB global constituent base (once)");
        return 0;
    }
}
