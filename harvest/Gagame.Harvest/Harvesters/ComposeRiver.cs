using System.Globalization;
using static Gagame.Harvest.JsonUtil;

namespace Gagame.Harvest.Harvesters;

/// THE RIVERS, MANY GAUGES IN, ONE FILE OUT, in data/river/river.json's schema (`sites[]`: id,
/// name, discharge_cfs, discharge_cms, gage_ft, obs_iso), each site with its lat/lon: every
/// active USGS stream gauge measuring discharge inside a region the fidelity map paints (the
/// NWIS site service, one request a region), and their latest instantaneous values (one request
/// for all). THE FIRST COPY (Mark, 2026-10-13): a region's site list and the value response are
/// cached once and read from then on.
public sealed class ComposeRiver : IHarvester
{
    public string Name => "compose-river";
    public string Title => "River composite (USGS NWIS)";
    public string What => "Every USGS discharge gauge inside the painted regions and its latest value: one river.json.";
    public string[] Outputs => [];
    public Source[] Sources => [new("USGS NWIS site + instantaneous values", "https://waterservices.usgs.gov/")];
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
        var dir = Path.Combine(outDir, "river");
        Directory.CreateDirectory(dir);
        var path = Path.Combine(dir, "river.json");
        Save(path, D(("generated_utc", NowIso()), ("source", "USGS waterservices IV (legacy; migrate to api.waterdata.usgs.gov by 2027Q1) at every discharge gauge in the fidelity map's regions"),
            ("sites", sites.OrderBy(s => (string)s["id"]!, StringComparer.Ordinal).ToList())));
        run.WroteBytes(new FileInfo(path).Length);
        if (sites.Count > 0)
            run.Input("usgs.nwis", "USGS NWIS instantaneous values (discharge 00060, stage 00065)", null, "https://waterservices.usgs.gov/nwis/iv/", "public domain (USGS)", "U.S. Geological Survey, National Water Information System",
                "n/a", "cfs to m3/s (x 0.0283168)", validUtc: sites.Select(s => s.GetValueOrDefault("obs_iso") as string).Where(x => x is not null).Min(StringComparer.Ordinal),
                fetchedUtc: firstIv is not null && File.Exists(firstIv) ? CompositeRun.FileUtc(firstIv) : null);
        var regions = run.Regions();
        for (var i = 0; i < regions.Count; i++)
            run.Coverage.Add(D(("box", new List<double> { regions[i].box.Lon0, regions[i].box.Lat0, regions[i].box.Lon1, regions[i].box.Lat1 }), ("grain_m", regions[i].grainM),
                ("sites", regionSites[i]), ("input", "usgs.nwis")));
        run.Save(D(("file", "river/river.json"), ("unit", "m3/s (discharge_cms), ft (gage_ft)"), ("datum", "gauge datum per site"), ("sites", sites.Count)));
        return 0;
    }
}
