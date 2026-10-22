using System.Globalization;
using static Gagame.Harvest.JsonUtil;

namespace Gagame.Harvest.Harvesters;

/// THE RIVERS, MANY GAUGES IN, ONE FILE OUT, in data/river/river.json's schema (`sites[]`: id,
/// name, discharge_cfs, discharge_cms, gage_ft, obs_iso), each site with its lat/lon: every
/// active USGS stream gauge measuring discharge inside a region the fidelity map paints (the
/// NWIS site service, one request a region), and their latest instantaneous values (one request
/// for all). THE FIRST COPY (Mark, 2026-10-13): a region's site list and the value response are
/// cached once and read from then on.
/// THE REACHES (2026-10-21): every river meeting the sea in a region with a gauge on its main
/// stem, as a line from the mouth upstream with its gauges and tide stations at their river km
/// (RiverReaches has the method). `sites` keeps every field it had; each site appends
/// `drainage_km2` (NWIS drain_area_va), and a main-stem gauge reporting discharge joins `sites`
/// even inland of the regions, its values from its own IV request so the region gauges' first
/// copies stay the ones they were.
public sealed class ComposeRiver : IHarvester
{
    public string Name => "compose-river";
    public string Title => "River composite (USGS NWIS)";
    public string What => "Every USGS discharge gauge inside the painted regions and its latest value: one river.json.";
    public string[] Outputs => [];
    public Source[] Sources => [new("USGS NWIS site + instantaneous values", "https://waterservices.usgs.gov/"),
        new("NHDPlus V2 flowlines (USGS NLDI + wmadata WFS)", "https://api.water.usgs.gov/nldi/linked-data/")];
    public Insert? Insert => new(InsertKind.FileArg, "A fidelity map (.gafid) and an output folder: --fidelity FILE --out data/composite/<name>/");

    const string SiteUrl = "https://waterservices.usgs.gov/nwis/site/?format=rdb&siteType=ST&siteStatus=active&parameterCd=00060&bBox=";
    const string IvUrl = "https://waterservices.usgs.gov/nwis/iv/?format=json&parameterCd=00060,00065&siteStatus=all&sites=";

    public int Run(HarvestContext ctx, string[] argv)
    {
        var (mapPath, outDir) = CompositeRun.Paths(new Args(argv), Name);
        var run = new CompositeRun(ctx, new Polite(ctx), mapPath, outDir, "river", "river");
        var cache = Path.Combine("cache", "river");
        Directory.CreateDirectory(cache);
        var gauges = new Dictionary<string, (double lat, double lon, string name)>();
        var regionSites = new List<List<string>>();
        foreach (var (box, _) in run.Regions())
        {
            var key = string.Join(",", new[] { box.Lon0, box.Lat0, box.Lon1, box.Lat1 }.Select(v => v.ToString("0.0000000", CultureInfo.InvariantCulture)));
            string rdb;
            var sitesPath = Path.Combine(cache, $"sites_{key.Replace(',', '_')}.rdb");
            try { (rdb, _) = run.Polite.GetTextP(SiteUrl + key, sitesPath); }
            catch (HttpError e) when (e.Code == 404)
            {
                rdb = "";   // NWIS answers 404 where no site matches: kept as an empty first copy
                File.WriteAllText(sitesPath, rdb);
            }
            var here = new List<string>();
            string[]? cols = null;
            foreach (var ln in rdb.Split('\n'))
            {
                if (ln.StartsWith('#') || ln.Trim().Length == 0) continue;
                var f = ln.TrimEnd('\r').Split('\t');
                if (cols is null) { cols = f; continue; }
                if (f.Length != cols.Length || f[0].EndsWith('s')) continue;   // the column-width row
                string Col(string c) => f[Array.IndexOf(cols, c)];
                if (!double.TryParse(Col("dec_lat_va"), NumberStyles.Float, CultureInfo.InvariantCulture, out var lat)) continue;
                if (!double.TryParse(Col("dec_long_va"), NumberStyles.Float, CultureInfo.InvariantCulture, out var lon)) continue;
                var id = Col("site_no");
                gauges[id] = (lat, lon, Col("station_nm"));
                here.Add(id);
            }
            regionSites.Add(here);
        }
        run.Log($"{gauges.Count} discharge gauge(s) in {regionSites.Count} region(s)");
        var (found, described) = RiverReaches.Find(run, cache, run.Regions(), gauges.Keys);

        var sites = new List<Dictionary<string, object?>>();
        string? firstIv = null;
        foreach (var chunk in gauges.Keys.Order(StringComparer.Ordinal).Chunk(100))
        {
            var ids = string.Join(",", chunk);
            var tag = Bin.Fnv1a(System.Text.Encoding.ASCII.GetBytes(ids)).ToString("x16");
            var have = FirstCopy.Earliest(cache, $"iv_{tag}_", ".json");
            var (text, _) = run.Polite.GetTextP(IvUrl + ids, have ?? Path.Combine(cache, $"iv_{tag}_{DateTime.UtcNow:yyyyMMddHH}.json"));
            firstIv ??= have ?? Path.Combine(cache, $"iv_{tag}_{DateTime.UtcNow:yyyyMMddHH}.json");
            foreach (var (id, s) in River.Latest(Parse(text)!))
            {
                if (gauges.TryGetValue(id, out var g)) { s["lat"] = g.lat; s["lon"] = g.lon; }
                s["source"] = "usgs.nwis";
                sites.Add(s);
            }
        }
        // the main-stem gauges inland of the regions: their own IV request(s), discharge only
        var extra = found.SelectMany(r => r.Gauges.Select(x => x.g.Id)).Where(id => !gauges.ContainsKey(id)).Distinct().Order(StringComparer.Ordinal).ToList();
        foreach (var chunk in extra.Chunk(100))
        {
            var ids = string.Join(",", chunk);
            var tag = Bin.Fnv1a(System.Text.Encoding.ASCII.GetBytes(ids)).ToString("x16");
            var have = FirstCopy.Earliest(cache, $"iv_{tag}_", ".json");
            var (text, _) = run.Polite.GetTextP(IvUrl + ids, have ?? Path.Combine(cache, $"iv_{tag}_{DateTime.UtcNow:yyyyMMddHH}.json"));
            foreach (var (id, s) in River.Latest(Parse(text)!))
            {
                if (!s.ContainsKey("discharge_cms") || !described.TryGetValue(id, out var g)) continue;
                s["lat"] = g.Lat; s["lon"] = g.Lon;
                s["source"] = "usgs.nwis";
                sites.Add(s);
            }
        }
        foreach (var s in sites) s["drainage_km2"] = described.TryGetValue((string)s["id"]!, out var dg) ? dg.DrainKm2 : null;
        var withQ = sites.Where(s => s.GetValueOrDefault("discharge_cms") is double q && !double.IsNaN(q)).Select(s => (string)s["id"]!).ToHashSet();
        // the tide stations: every CO-OPS harmonic (reference) station, the set compose-tides fits
        // (its listing, cached once); the tides layer then reads each one's river km from here
        var coops = Path.Combine("cache", "coops");
        new Tides().UsePolite(ctx, run.Polite).Discover(new Box(-180, -90, 180, 90), 0.0, coops, (0.0, 0.0));
        var tideStations = Load(Path.Combine(coops, "_lists", "tidepredictions.json")).Arr("stations").Where(t => t.Str("type") == "R" && t.Str("id") is not null)
            .Select(t => (t.Str("id")!, t.Num("lat"), t.Num("lng"))).ToList();
        var reaches = new List<Dictionary<string, object?>>();
        var reachRegion = new List<int>();
        foreach (var r in found)
            if (RiverReaches.Build(run, cache, r, tideStations, withQ.Contains) is { } built) { reaches.Add(built); reachRegion.Add(r.Region); }
        var dir = Path.Combine(outDir, "river");
        Directory.CreateDirectory(dir);
        var path = Path.Combine(dir, "river.json");
        Save(path, D(("generated_utc", NowIso()), ("source", "USGS waterservices IV (legacy; migrate to api.waterdata.usgs.gov by 2027Q1) at every discharge gauge in the fidelity map's regions"),
            ("sites", sites.OrderBy(s => (string)s["id"]!, StringComparer.Ordinal).ToList()), ("reaches", reaches)));
        run.WroteBytes(new FileInfo(path).Length);
        if (sites.Count > 0)
            run.Input("usgs.nwis", "USGS NWIS instantaneous values (discharge 00060, stage 00065)", null, "https://waterservices.usgs.gov/nwis/iv/", "public domain (USGS)", "U.S. Geological Survey, National Water Information System",
                "n/a", "cfs to m3/s (x 0.0283168)", validUtc: sites.Select(s => s.GetValueOrDefault("obs_iso") as string).Where(x => x is not null).Min(StringComparer.Ordinal),
                fetchedUtc: firstIv is not null && File.Exists(firstIv) ? CompositeRun.FileUtc(firstIv) : null);
        if (reaches.Count > 0)
            run.Input("usgs.nhdplus", "NHDPlus V2 network flowlines (USGS wmadata WFS) and NLDI navigation / position", null, RiverReaches.Nldi, "public domain (USGS, EPA)",
                "U.S. EPA and USGS, NHDPlus Version 2; USGS Network-Linked Data Index", "n/a", "NWIS drain_area_va sq mi to km2 (x 2.589988); km from the mouth = chained great-circle length of the NHD geometry");
        var regions = run.Regions();
        for (var i = 0; i < regions.Count; i++)
            run.Coverage.Add(D(("box", new List<double> { regions[i].box.Lon0, regions[i].box.Lat0, regions[i].box.Lon1, regions[i].box.Lat1 }), ("grain_m", regions[i].grainM),
                ("sites", regionSites[i]), ("input", "usgs.nwis"), ("reaches", Enumerable.Range(0, reaches.Count).Where(k => reachRegion[k] == i).Select(k => reaches[k]["id"]).ToList())));
        run.Save(D(("file", "river/river.json"), ("unit", "m3/s (discharge_cms), ft (gage_ft)"), ("datum", "gauge datum per site"), ("sites", sites.Count), ("reaches", reaches.Count)));
        return 0;
    }
}
