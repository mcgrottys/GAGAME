using System.Text.RegularExpressions;
using static Gagame.Harvest.JsonUtil;

namespace Gagame.Harvest.Harvesters;

/// THE TIDES, MANY SOURCES IN, ONE FILE OUT: every harmonic station CO-OPS publishes (a point
/// layer is the provider's whole set, Mark 2026-10-21: datasets by measurement, never by place;
/// the regions say only which stations stand in which, in the coverage), fitted as `tides` fits one (Tides.Fit: the harcon frequencies, amplitude
/// and phase least-squares fitted to a year of official hourly predictions), and where no station
/// stands, MODEL STATIONS from the EOT20 atlas, all in the one data/tides/stations.json schema the
/// engine reads (sim/TideModel.h): the engine picks by position, never by name.
///
/// WHERE THE MODEL STANDS. A region wanting grain g has a model station on every node of the
/// atlas's 1/8 deg lattice taken at stride n = ceil(g / node spacing) (so stations no closer than
/// the region wants, and never closer than the atlas resolves) whose half-spacing neighbourhood
/// meets the region, where the atlas has ocean, and where no fitted station stands within half a
/// spacing. One law at every size: a small region still holds the node nearest it.
///
/// THE YEAR. One epoch a file (the schema's `epoch_unix`). The year is the EARLIEST whose
/// predictions the cache already holds for a station of this map (the first copy, Mark
/// 2026-10-13), else this year; every station is fitted to it.
///
/// THE RIVER KILOMETRE. A station on a reach of this composite's river layer (river/river.json,
/// compose-river: so compose-water runs the river before the tides) takes that reach's km as
/// `river_km` and appends `river_reach` (the reach id); elsewhere river_km stays null.
///
/// THE COST GUARD. Every station's first copies (meta, harcon, datums, the year's predictions)
/// are counted before anything is deleted; when the ledger's request budget cannot hold the
/// missing ones the run stops with the count and the old output stands.
public sealed class ComposeTides : IHarvester
{
    public string Name => "compose-tides";
    public string Title => "Tides composite (CO-OPS + EOT20)";
    public string What => "Every CO-OPS harmonic station, fitted, and EOT20 model stations in the painted regions where none stands: one stations.json.";
    public string[] Outputs => [];
    public Source[] Sources => [new("NOAA CO-OPS", "https://api.tidesandcurrents.noaa.gov/mdapi/prod/"), new("EOT20 (SEANOE, CC BY 4.0)", "https://doi.org/10.17882/79489")];
    public Insert? Insert => new(InsertKind.FileArg, "A fidelity map (.gafid) and an output folder: --fidelity FILE --out data/composite/<name>/");

    const double NodeM = TideModelAtlas.StepDeg * 111195.0;

    public int Run(HarvestContext ctx, string[] argv)
    {
        var (mapPath, outDir) = CompositeRun.Paths(new Args(argv), Name);
        var run = new CompositeRun(ctx, new Polite(ctx), mapPath, outDir, "tides", "tides");
        var cacheDir = Path.Combine("cache", "coops");
        var tidesDir = Path.Combine(outDir, "tides");
        var tides = new Tides().UsePolite(ctx, run.Polite);
        var regions = run.Regions();

        // ---- every harmonic station; the regions only list which stand in them
        var regionStations = new List<List<string>>();
        foreach (var (box, _) in regions)
            regionStations.Add(tides.Discover(box, 0.0, cacheDir, (box.CenterLon, box.CenterLat)).found.Select(s => s.Id).ToList());
        var stations = tides.Discover(new Box(-180, -90, 180, 90), 0.0, cacheDir, (0.0, 0.0)).found.OrderBy(s => s.Id, StringComparer.Ordinal).ToList();
        var year = FirstYear(stations, cacheDir) ?? DateTime.UtcNow.Year;
        run.Log($"{stations.Count} harmonic station(s) published, {regionStations.Sum(r => r.Count)} inside the {regions.Count} region(s); fit year {year} (the earliest the cache holds)");
        var need = stations.Sum(s => Missing(cacheDir, s.Id, year));
        var (_, left) = run.Polite.Remaining();
        run.Log($"{need} request(s) missing from the cache (one-request years assumed), {left} left in the ledger's budget");
        if (need > left)
            throw new HarvestStop($"[tides] {need} CO-OPS request(s) needed for {stations.Count} stations, {left} left in {run.Polite.LedgerPath}'s budget: nothing fetched, the old output stands", 3);
        if (Directory.Exists(tidesDir)) foreach (var old in Directory.GetFiles(tidesDir)) File.Delete(old);
        var rec = tides.Fit(stations, year, tidesDir, cacheDir, tolerant: true);
        foreach (var s in rec.Stations) { s["kind"] = "station"; s["source"] = "noaa.coops"; }
        var riverPath = Path.Combine(outDir, "river", "river.json");
        var onRiver = new Dictionary<string, (double km, string reach)>();
        if (File.Exists(riverPath))
            foreach (var reach in Load(riverPath).Arr("reaches"))
                foreach (var t in reach.Arr("tides"))
                    onRiver.TryAdd(t.Str("id")!, (t.Num("km"), reach.Str("id")!));
        else run.Log($"no {riverPath}: river_km stays null (compose-river first)");
        foreach (var s in rec.Stations)
            if (onRiver.TryGetValue((string)s["id"]!, out var rk)) { s["river_km"] = rk.km; s["river_reach"] = rk.reach; }
        run.Log($"{rec.Stations.Count(s => s.ContainsKey("river_reach"))} station(s) on a river reach");

        // ---- model stations where none stands
        var atlasDir = Path.Combine("data", "water");
        var model = new List<Dictionary<string, object?>>();
        var regionNodes = new List<int>();
        if (TideModelAtlas.Present(atlasDir))
        {
            var atlas = new TideModelAtlas(atlasDir);
            var taken = new HashSet<(int, int)>();
            foreach (var (box, g) in regions)
            {
                var n = Math.Max(1, (int)Math.Ceiling(g / NodeM - 1e-9));
                var half = 0.5 * n * TideModelAtlas.StepDeg;
                var spacingKm = n * NodeM / 1000.0;
                int count = 0;
                var (rTop, _, _, _) = atlas.NodeOf(Math.Min(90, box.Lat1 + half), 0);
                var (rBot, _, _, _) = atlas.NodeOf(Math.Max(-90, box.Lat0 - half), 0);
                for (var r = rTop / n * n; r <= rBot; r += n)
                {
                    var lat = 90.0 - r * TideModelAtlas.StepDeg;
                    if (lat > box.Lat1 + half || lat < box.Lat0 - half) continue;
                    var c0 = (int)Math.Floor((box.Lon0 - half) / TideModelAtlas.StepDeg / n) * n;
                    for (var cc = c0; cc * TideModelAtlas.StepDeg <= box.Lon1 + half; cc += n)
                    {
                        var c = ((cc % 2880) + 2880) % 2880;
                        if (!taken.Add((r, c))) continue;
                        var lon = cc * TideModelAtlas.StepDeg;
                        if (rec.Stations.Any(s => Geo.Haversine_km(lat, lon, (double)s["lat"]!, (double)s["lon"]!) < 0.5 * spacingKm)) continue;
                        var at = atlas.At(r, c);
                        if (at is null) continue;
                        model.Add(ModelStation(at, year, lat, lon, spacingKm));
                        count++;
                    }
                }
                regionNodes.Add(count);
            }
            run.Log($"{model.Count} EOT20 model station(s) where no fitted station stands");
            run.Input("eot20", "EOT20 global ocean tide atlas, 14 constituents, 1/8 deg (as `water --eot20` converted it)", Round(NodeM, 1), "https://doi.org/10.17882/79489",
                "CC BY 4.0", "Hart-Davis, M. G. et al. 2021, EOT20: a global ocean tide model from multi-mission satellite altimetry, ESSD 13, 3869", "mean sea surface",
                "model: amp f A, phase V0 + u - G at the fit epoch (TideModelAtlas)", validUtc: $"{year}", fetchedUtc: atlas.Files.Select(CompositeRun.FileUtc).Min());
            run.Extra["model_check"] = ModelCheck(atlas, rec.Stations, tidesDir, cacheDir, year);
        }
        else run.Log($"no EOT20 grids in {atlasDir} (run `water --eot20`): fitted stations only");

        var all = rec.Stations.Concat(model).ToList();
        var epoch = Unix(new DateTime(year, 1, 1, 0, 0, 0, DateTimeKind.Utc));
        var doc = D(("generated_utc", NowIso()),
            ("source", "NOAA CO-OPS harcon frequencies + least-squares fit to official hourly predictions; EOT20 model stations (kind: model) where no station stands"),
            ("year", year), ("epoch_unix", epoch), ("stations", all));
        var path = Path.Combine(tidesDir, "stations.json");
        Save(path, doc);
        run.WroteBytes(Directory.GetFiles(tidesDir).Sum(f => new FileInfo(f).Length));
        var preds = stations.Select(s => Path.Combine(cacheDir, s.Id)).Where(Directory.Exists).SelectMany(d => Directory.GetFiles(d, $"pred_{year}_*.json")).ToList();
        run.ReadBytes(preds.Sum(f => new FileInfo(f).Length));
        run.Input("noaa.coops", "NOAA CO-OPS harmonic constituents + official hourly predictions (MLLW, metric)", null, "https://api.tidesandcurrents.noaa.gov/",
            "public domain (NOAA CO-OPS)", "NOAA CO-OPS Tides & Currents", "MLLW (station datum link to NAVD88 per station: mllw_minus_navd_m)", "none",
            validUtc: $"{year}", fetchedUtc: preds.Count > 0 ? preds.Select(CompositeRun.FileUtc).Min() : null);
        for (var i = 0; i < regions.Count; i++)
            run.Coverage.Add(D(("box", new List<double> { regions[i].box.Lon0, regions[i].box.Lat0, regions[i].box.Lon1, regions[i].box.Lat1 }), ("grain_m", regions[i].grainM),
                ("stations", regionStations[i]), ("model_stations", i < regionNodes.Count ? regionNodes[i] : 0), ("input", regionStations[i].Count > 0 ? "noaa.coops" : "eot20")));
        run.Extra["failed"] = rec.Failed;
        run.Save(D(("file", "tides/stations.json"), ("unit", "m"), ("datum", "MLLW per station (model stations: mean sea level; mean_mllw_m is their mean above their own lowest-low mean)"),
            ("stations", rec.Stations.Count), ("model_stations", model.Count)));
        return 0;
    }

    /// The requests a station's fit still needs: its meta, harcon and datums, and its year of
    /// predictions (one request when CO-OPS grants the year in one, as it has so far).
    static int Missing(string cacheDir, string sid, int year)
    {
        var d = Path.Combine(cacheDir, sid);
        var n = new[] { "meta.json", "harcon_metric.json", "datums_metric.json" }.Count(f => !File.Exists(Path.Combine(d, f)));
        if (!File.Exists(Path.Combine(d, $"pred_{year}_all.json")) && !Enumerable.Range(1, 12).All(m => File.Exists(Path.Combine(d, $"pred_{year}_{m:00}.json")))) n++;
        return n;
    }

    static int? FirstYear(List<Tides.Station> stations, string cacheDir)
    {
        var rx = new Regex(@"^pred_(\d{4})_(all|\d\d)\.json$");
        int? best = null;
        foreach (var s in stations)
        {
            var d = Path.Combine(cacheDir, s.Id);
            if (!Directory.Exists(d)) continue;
            var years = Directory.GetFiles(d).Select(f => rx.Match(Path.GetFileName(f))).Where(m => m.Success)
                .GroupBy(m => int.Parse(m.Groups[1].Value)).Where(g => g.Any(m => m.Groups[2].Value == "all") || g.Count() >= 12).Select(g => g.Key);
            foreach (var y in years) best = best is null ? y : Math.Min(best.Value, y);
        }
        return best;
    }

    static Dictionary<string, object?> ModelStation((double re, double im)[] at, int year, double lat, double lon, double spacingKm)
    {
        var coeffs = TideModelAtlas.Coefficients(at, year);
        var mllw = MeanLowerLow(coeffs, year);
        string F(double v) => v.ToString("0.###", System.Globalization.CultureInfo.InvariantCulture);
        return D(("id", $"eot20:{F(lat)},{F(lon)}"), ("name", "EOT20 model station"), ("full_name", $"EOT20 node {F(lat)}, {F(lon)} ({F(spacingKm)} km lattice)"),
            ("lat", lat), ("lon", lon), ("mllw_minus_navd_m", null), ("river_km", null),
            ("mean_mllw_m", Round(-mllw, 5)), ("fit_rms_m", null), ("fit_max_m", null),
            ("official_file", null), ("official_start_unix", null), ("official_dt_s", 3600), ("official_n", 0),
            ("coeffs", coeffs.Select(c => D(("name", c.name), ("speed_dph", c.speed), ("amp_m", Round(c.amp, 6)), ("phase_rad", Round(c.phase, 6)))).ToList()),
            ("kind", "model"), ("source", "eot20"));
    }

    /// The mean of each tidal day's lowest hour over the year: the model's MLLW about its mean.
    static double MeanLowerLow(List<(string name, double speed, double amp, double phase)> c, int year)
    {
        var hours = (int)(new DateTime(year + 1, 1, 1) - new DateTime(year, 1, 1)).TotalHours;
        var day = 24.8412;
        var lows = new List<double>();
        var lo = double.MaxValue;
        var next = day;
        for (var h = 0; h < hours; h++)
        {
            if (h >= next) { lows.Add(lo); lo = double.MaxValue; next += day; }
            double v = 0, t = h * 3600.0;
            foreach (var k in c) v += k.amp * Math.Cos(k.speed * Geo.DegToRad / 3600.0 * t + k.phase);
            lo = Math.Min(lo, v);
        }
        return lows.Average();
    }

    /// THE INSTRUMENT for the model's phase: at every fitted station with an ocean node within one
    /// node step, (1) EOT20's amplitude and Greenwich lag against the station's own published
    /// harcon, (2) V0 + u from the astronomy against the fit's phase + the harcon's lag (the fit
    /// IS V0 + u - G, so the two agree only if the astronomy is right), (3) the model's hourly
    /// series against the official one over the year, about each one's mean.
    static Dictionary<string, object?> ModelCheck(TideModelAtlas atlas, List<Dictionary<string, object?>> stations, string tidesDir, string cacheDir, int year)
    {
        var rows = new List<Dictionary<string, object?>>();
        var epoch = new DateTime(year, 1, 1, 0, 0, 0, DateTimeKind.Utc);
        var mid = new DateTime(year, 7, 2, 12, 0, 0, DateTimeKind.Utc);
        foreach (var s in stations)
        {
            double lat = (double)s["lat"]!, lon = (double)s["lon"]!;
            var (r, c, nlat, nlon) = atlas.NodeOf(lat, lon);
            var at = atlas.At(r, c);
            if (at is null) continue;
            var har = TryLoad(Path.Combine(cacheDir, (string)s["id"]!, "harcon_metric.json")).Arr("HarmonicConstituents").ToDictionary(x => x.Str("name") ?? "", x => (amp: x.Num("amplitude"), g: x.Num("phase_GMT")));
            var fit = ((List<Dictionary<string, object?>>)s["coeffs"]!).ToDictionary(x => (string)x["name"]!, x => (double)x["phase_rad"]!);
            var cons = new Dictionary<string, object?>();
            for (var k = 0; k < TideModelAtlas.Constituents.Length; k++)
            {
                var name = TideModelAtlas.Constituents[k].name;
                if (name is not ("M2" or "S2" or "N2" or "K1" or "O1")) continue;
                if (!har.TryGetValue(name, out var h) || !fit.TryGetValue(name, out var ph)) continue;
                var (re, im) = at[k];
                var (_, u) = TideModelAtlas.Nodal(k, mid);
                var astro = Wrap(TideModelAtlas.V0(k, epoch) + u);
                var fromFit = Wrap(ph * Geo.RadToDeg + h.g);
                cons[name] = D(("harcon_amp_m", h.amp), ("eot20_amp_m", Round(Math.Sqrt(re * re + im * im), 4)), ("harcon_g_deg", h.g), ("eot20_g_deg", Round(Wrap(Math.Atan2(im, re) * Geo.RadToDeg), 2)),
                    ("v0u_astro_deg", Round(astro, 2)), ("v0u_from_fit_deg", Round(fromFit, 2)), ("v0u_diff_deg", Round(Wrap(astro - fromFit + 180) - 180, 2)));
            }
            double? rms = null;
            if (s["official_file"] is string of && File.Exists(Path.Combine(tidesDir, of)))
            {
                var off = Bin.ReadF32(Path.Combine(tidesDir, of));
                var t0 = (double)s["official_start_unix"]! - Unix(epoch);
                var coeffs = TideModelAtlas.Coefficients(at, year);
                var model = new double[off.Length];
                for (var i = 0; i < off.Length; i++)
                    foreach (var k in coeffs) model[i] += k.amp * Math.Cos(k.speed * Geo.DegToRad / 3600.0 * (t0 + 3600.0 * i) + k.phase);
                double mo = off.Average(v => (double)v), mm = model.Average(), sq = 0;
                for (var i = 0; i < off.Length; i++) sq += Math.Pow(model[i] - mm - (off[i] - mo), 2);
                rms = Round(Math.Sqrt(sq / off.Length), 4);
            }
            rows.Add(D(("id", s["id"]), ("node", new List<double> { nlat, nlon }), ("km", Round(Geo.Haversine_km(lat, lon, nlat, nlon), 2)), ("hourly_rms_m", rms), ("constituents", cons)));
        }
        return D(("what", "EOT20 at the node nearest each fitted station: its amplitude and Greenwich lag against the station's harcon; V0+u from the astronomy against (fit phase + harcon lag); the model's hourly series against the official one (rms about each mean)"),
            ("rows", rows));
    }

    static double Wrap(double deg) => ((deg % 360) + 360) % 360;
}
