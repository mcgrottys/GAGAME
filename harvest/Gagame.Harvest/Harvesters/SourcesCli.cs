using Gagame.Harvest.Readers;
using Gagame.Harvest.Sources;
using static Gagame.Harvest.JsonUtil;

namespace Gagame.Harvest.Harvesters;

/// THE SOURCE CONFIGS AND THEIR AGREEMENT (docs/FIDELITY.md 8):
///
///   list            every config of harvest/sources/{buoys,relief}, its kind's reader, rank and
///                   coverage; each is made (its reader validates the config) without a fetch
///   agree-buoys     --a ID --b ID: every station of A in the map's `seastate` regions (and `--box`), A's line
///                   (its first copy) against B's line at the same station and moment, quantity by
///                   quantity, each within half the last digit A prints (two copies of one instrument)
///   agree-relief    --a ID --b ID: every pixel of A's files in the map's `water.bed` regions (and
///                   `--box`), A against B at the same point: the pixel centres must coincide, the
///                   differences are measured by depth band
///
/// `--fidelity FILE.gafid --out DIR`; writes `<out>/sources_<what>.json`, returns 1 if its gate fails.
public sealed class SourcesCli : IHarvester
{
    public string Name => "sources";
    public string Title => "Source configs (list and agreement gates)";
    public string What => "Lists the buoy and relief source configs, and measures two sources of the same quantity against each other.";
    public string[] Outputs => [];
    public Source[] Sources => [];
    public Insert? Insert => null;

    /// Half the last digit NDBC's realtime2 prints for each quantity: WVHT 0.1 m, DPD 1 s, APD 0.1 s,
    /// WSPD 0.1 m/s, WDIR and MWD 1 degree.
    static readonly Dictionary<string, double> Tolerance = new()
    {
        ["hs"] = 0.05, ["dpd"] = 0.5, ["apd"] = 0.05, ["wspd"] = 0.05, ["wdir"] = 0.5, ["mwd"] = 0.5,
    };

    public int Run(HarvestContext ctx, string[] argv)
    {
        var a = new Args(argv);
        var what = a.Positionals("--a", "--b", "--fidelity", "--out", "--box").FirstOrDefault() ?? "list";
        var polite = new Polite(ctx);
        var env = new SourceEnv(ctx, polite, ctx.Log);
        if (what == "list") return List(ctx, env);
        var outDir = a.Opt("--out") ?? throw new HarvestStop($"sources {what} needs --out DIR");
        var map = FidelityMap.Load(a.Opt("--fidelity") ?? throw new HarvestStop($"sources {what} needs --fidelity FILE.gafid"));
        var ia = a.Opt("--a") ?? throw new HarvestStop("needs --a ID");
        var ib = a.Opt("--b") ?? throw new HarvestStop("needs --b ID");
        Directory.CreateDirectory(outDir);
        return what switch
        {
            "agree-buoys" => AgreeBuoys(ctx, env, map, a.BoxOpt(), ia, ib, outDir),
            "agree-relief" => AgreeRelief(ctx, env, map, a.BoxOpt(), ia, ib, outDir),
            _ => throw new HarvestStop($"sources: list | agree-buoys | agree-relief, not {what}"),
        };
    }

    /// The painted cells of a layer (else `*`) as lon/lat boxes.
    static List<Box> Boxes(FidelityMap map, string layer) =>
        map.Regions(layer).Select(r => new Box(r.box.lon0, r.box.lat0, r.box.lon1, r.box.lat1)).ToList();

    int List(HarvestContext ctx, SourceEnv env)
    {
        foreach (var (kind, reader, obs) in SourceFactory.Kinds) ctx.Log($"kind {kind,-18} {(obs ? "observations" : "relief"),-12} {reader}");
        foreach (var layer in new[] { SourceFactory.Observations, SourceFactory.Relief })
            foreach (var c in SourceFactory.Configs(ctx, layer))
            {
                if (layer == SourceFactory.Observations) SourceFactory.Make<IObservationSource>(c, env); else SourceFactory.Make<IReliefSource>(c, env);
                ctx.Log($"{layer,-6} {c.Id,-30} rank {c.Rank} {c.Kind,-16} {(c.Datum is null ? "" : $"{c.Datum} {c.StepDeg * 3600:0.###}\" ")}[{c.Coverage.Lon0}, {c.Coverage.Lat0}, {c.Coverage.Lon1}, {c.Coverage.Lat1}] {c.Title}");
            }
        return 0;
    }

    // ------------------------------------------------------------------ buoys

    int AgreeBuoys(HarvestContext ctx, SourceEnv env, FidelityMap map, Box? box, string ia, string ib, string outDir)
    {
        var all = SourceFactory.ObservationSources(env, $"{ia},{ib}");
        var A = all.First(s => s.Config.Id == ia);
        var B = all.First(s => s.Config.Id == ib);
        var rows = new List<Dictionary<string, object?>>();
        int compared = 0, within = 0, onlyOne = 0;
        var worst = Tolerance.Keys.ToDictionary(k => k, _ => 0.0);
        var boxes = Boxes(map, "seastate");
        if (box is not null) boxes.Add(box);
        foreach (var st in boxes.SelectMany(A.Stations).DistinctBy(s => s.Id).OrderBy(s => s.Id, StringComparer.Ordinal))
        {
            var oa = A.Observe(st);
            if (oa?.GetValueOrDefault("obs_unix") is not double t) continue;
            var ob = B.Observe(st, t);
            var q = new Dictionary<string, object?>();
            foreach (var k in Observed.Quantities)
            {
                var va = oa.GetValueOrDefault(k) as double?;
                var vb = ob?.GetValueOrDefault(k) as double?;
                if (va is null && vb is null) continue;
                if (va is null || vb is null) { onlyOne++; q[k] = D(("a", va), ("b", vb)); continue; }
                var d = k is "wdir" or "mwd" ? Math.Abs(((va.Value - vb.Value) % 360 + 540) % 360 - 180) : Math.Abs(va.Value - vb.Value);
                compared++;
                if (d <= Tolerance[k] + 1e-9) within++;
                worst[k] = Math.Max(worst[k], d);
                q[k] = D(("a", va), ("b", vb), ("abs_diff", Round(d, 4)));
            }
            rows.Add(D(("station", st.Id), ("obs_utc", IsoZ(FromUnix(t))), ("b_has_line", ob is not null), ("quantities", q)));
        }
        var pass = compared > 0 && within == compared;
        var doc = D(("generated_utc", NowIso()), ("a", ia), ("b", ib), ("gate", "every quantity both lines hold within half the last digit A prints"),
            ("tolerance", Tolerance), ("pass", pass), ("stations", rows.Count), ("compared", compared), ("within", within), ("held_by_one_only", onlyOne),
            ("worst_abs_diff", worst), ("blind_spot", "the stations of A's first copies only (one line each, its newest); a quantity one source lacks is counted, not compared; agreement of two copies of one instrument says nothing of the instrument"),
            ("rows", rows));
        var path = Path.Combine(outDir, "sources_agree_buoys.json");
        Save(path, doc);
        ctx.Log($"[sources] {(pass ? "PASS" : "FAIL")} agree-buoys {ia} vs {ib}: {rows.Count} stations, {within}/{compared} quantities within tolerance, {onlyOne} held by one only; worst {Dump(worst, 0)} -> {path}");
        return pass ? 0 : 1;
    }

    // ------------------------------------------------------------------ relief

    static GeoTiff Open(Polite p, ReliefFile f)
    {
        if (!File.Exists(f.Dest))
        {
            if (f.Make is { } make) make(p);
            else p.Download(f.Url, f.Dest, f.Size ?? f.SizeOf?.Invoke() ?? p.Head(f.Url).Length ?? throw new InvalidOperationException($"{f.Url}: no size"), note: "sources agree");
        }
        return new GeoTiff(File.ReadAllBytes(f.Dest));
    }

    static (double lonEdge, double latEdge) Edges(GeoTiff g)
    {
        var pt = g.GeoKeys().GetValueOrDefault("GTRasterType") is int rt && rt == 2;
        return (g.Lon0 - g.TieI * g.Sx - (pt ? 0.5 * g.Sx : 0), g.Lat0 + g.TieJ * g.Sy + (pt ? 0.5 * g.Sy : 0));
    }

    int AgreeRelief(HarvestContext ctx, SourceEnv env, FidelityMap map, Box? box, string ia, string ib, string outDir)
    {
        var all = SourceFactory.ReliefSources(env, $"{ia},{ib}");
        var A = all.First(s => s.Config.Id == ia);
        var B = all.First(s => s.Config.Id == ib);
        var boxes = Boxes(map, "water.bed");
        if (box is not null) boxes.Add(box);
        var bands = new (string name, double lo, double hi)[] { ("deep, b <= -200 m", double.NegativeInfinity, -200), ("shelf, -200 < b <= 0 m", -200, 0), ("land, b > 0 m", 0, double.PositiveInfinity) };
        var diffs = bands.ToDictionary(x => x.name, _ => new List<double>());
        var bCache = new Dictionary<string, (GeoTiff g, double le, double la)>();
        double worstCentre = 0;
        long points = 0, missing = 0;
        foreach (var bx in boxes)
            foreach (var fa in A.Meets(bx).DistinctBy(f => f.Dest))
            {
                var ga = Open(env.Polite, fa);
                var (lea, laa) = Edges(ga);
                var va = ga.ReadWindow(0, 0, ga.Width, ga.Height);
                for (var y = 0; y < ga.Height; y++)
                    for (var x = 0; x < ga.Width; x++)
                    {
                        double lon = lea + (x + 0.5) * ga.Sx, lat = laa - (y + 0.5) * ga.Sy;
                        if (!bx.Contains(lon, lat)) continue;
                        points++;
                        var fb = B.Meets(new Box(lon, lat, lon, lat)).FirstOrDefault();
                        if (fb is null) { missing++; continue; }
                        if (!bCache.TryGetValue(fb.Dest, out var gb)) { var g = Open(env.Polite, fb); var (le, la) = Edges(g); bCache[fb.Dest] = gb = (g, le, la); }
                        int bx_ = (int)Math.Floor((lon - gb.le) / gb.g.Sx), by_ = (int)Math.Floor((gb.la - lat) / gb.g.Sy);
                        if (bx_ < 0 || by_ < 0 || bx_ >= gb.g.Width || by_ >= gb.g.Height) { missing++; continue; }
                        worstCentre = Math.Max(worstCentre, Math.Max(Math.Abs(gb.le + (bx_ + 0.5) * gb.g.Sx - lon), Math.Abs(gb.la - (by_ + 0.5) * gb.g.Sy - lat)) / ga.Sx);
                        var vb = gb.g.ReadWindow(bx_, by_, 1, 1)[0];
                        var v = va[y * ga.Width + x];
                        if (float.IsNaN(v) || float.IsNaN(vb)) { missing++; continue; }
                        diffs[bands.First(b => vb > b.lo && vb <= b.hi).name].Add(v - vb);
                    }
            }
        Dictionary<string, object?> Stats(List<double> d)
        {
            if (d.Count == 0) return D(("n", 0));
            var s = d.OrderBy(x => x).ToArray();
            var ab = d.Select(Math.Abs).OrderBy(x => x).ToArray();
            double Q(double[] v, double p) => v[(int)(p * (v.Length - 1))];
            return D(("n", d.Count), ("median_m", Round(Q(s, 0.5), 3)), ("p05_m", Round(Q(s, 0.05), 3)), ("p95_m", Round(Q(s, 0.95), 3)),
                ("median_abs_m", Round(Q(ab, 0.5), 3)), ("p90_abs_m", Round(Q(ab, 0.9), 3)), ("max_abs_m", Round(ab[^1], 3)));
        }
        var deep = diffs[bands[0].name];
        var deepMedAbs = deep.Count == 0 ? double.NaN : deep.Select(Math.Abs).OrderBy(x => x).ElementAt((deep.Count - 1) / 2);
        // the gate, stated before the run: one lattice (centres within a thousandth of a pixel),
        // and where the sea is deep (the two compilations share their sources there) the median
        // |A - B| within 5 m; the shelf and the land are measured, not gated (each compilation
        // takes a different survey there, and the datums differ by up to a metre or two)
        var aligned = worstCentre < 1e-3;
        var pass = aligned && deep.Count > 0 && deepMedAbs <= 5.0;
        var doc = D(("generated_utc", NowIso()), ("a", ia), ("b", ib), ("a_datum", A.Config.Datum), ("b_datum", B.Config.Datum),
            ("gate", "pixel centres coincide (worst offset < 0.001 px) and the median |a - b| where b <= -200 m is <= 5 m"), ("pass", pass),
            ("boxes", boxes.Select(b => new List<double> { b.Lon0, b.Lat0, b.Lon1, b.Lat1 }).ToList()),
            ("points", points), ("not_compared", missing), ("worst_centre_offset_px", worstCentre),
            ("bands", bands.ToDictionary(b => b.name, b => (object?)Stats(diffs[b.name]))),
            ("blind_spot", "every pixel of A's files inside the boxes against the pixel of B holding its centre: it sees a lattice shift, a sign or unit error and a datum step; it cannot say which compilation is right where they differ, and B's own errors are shared where A was built from it"));
        var path = Path.Combine(outDir, "sources_agree_relief.json");
        Save(path, doc);
        ctx.Log($"[sources] {(pass ? "PASS" : "FAIL")} agree-relief {ia} vs {ib}: {points} points, centres within {worstCentre:0.######} px, deep median |d| {deepMedAbs:0.###} m; {Dump(doc["bands"], 0)} -> {path}");
        return pass ? 0 : 1;
    }
}
