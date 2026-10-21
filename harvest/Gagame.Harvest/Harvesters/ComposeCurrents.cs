using System.Globalization;
using System.Text.RegularExpressions;
using Gagame.Harvest.Readers;
using static Gagame.Harvest.JsonUtil;

namespace Gagame.Harvest.Harvesters;

/// THE CURRENTS, MANY SOURCES IN, ONE FILE OUT, in data/currents/currents.json's schema
/// (sim/CurrentModel.cpp) grown to hold every region at once:
///
///   tidal_stations  every current-prediction station CO-OPS publishes (its shallowest bin; a
///                   point layer is the provider's whole set, 2026-10-21), 45 days of max/slack
///                   events, with its own lat/lon; the coverage lists which stand in which
///                   region. Before anything is deleted the missing windows (two requests a
///                   station) are counted against the ledger's budget; short, the run stops.
///   buoy_adcp       the NDBC realtime2 ADCP buoy inside a region nearest the finest region's
///                   centre, with its lat/lon; `buoys_adcp` holds every one
///   fields          one surface field per (region, NOS OFS whose grid meets it): u then v planes,
///                   row 0 south, -999 land, at max(the region's grain, the model's own rho
///                   spacing there), each with its box, grain, valid time and `model` id
///   field           the finest of `fields`, for a reader that knows one field
///
/// Which OFS meets a region is read from the model's own vertical-datum grid (OFS_Grid_Datum/
/// <ofs>_vdatums.nc: its longitude/latitude), never from a table. Only the ROMS systems (those
/// with a <ofs>.romsgrid.nc) are converted: the others (FVCOM meshes, or no grid published) are
/// listed as skipped. A field with no water cell in its region is not written. RTOFS (a global base) is not taken: its
/// surface files are gigabytes an hour on the NODD bucket against this run's 6 GB budget.
///
/// THE FIRST COPY (Mark, 2026-10-13): a station's prediction window is the first one cached, an
/// OFS cycle is the first one on the big drive, a buoy's .adcp is the first copy taken; a rerun
/// fetches nothing it already holds.
public sealed class ComposeCurrents : IHarvester
{
    public string Name => "compose-currents";
    public string Title => "Currents composite (CO-OPS + NOS OFS + NDBC)";
    public string What => "Every current station in the painted regions, every OFS surface field that covers one, the ADCP buoys: one currents.json.";
    public string[] Outputs => [];
    public Source[] Sources => [new("CO-OPS currents", "https://tidesandcurrents.noaa.gov/currents/"), new("NOS OFS on NODD", "https://registry.opendata.aws/noaa-ofs/"), new("NDBC realtime2", "https://www.ndbc.noaa.gov/data/realtime2/")];
    public Insert? Insert => new(InsertKind.FileArg, "A fidelity map (.gafid) and an output folder: --fidelity FILE --out data/composite/<name>/");

    const int MaxFieldCells = 1 << 24;

    public int Run(HarvestContext ctx, string[] argv)
    {
        var (mapPath, outDir) = CompositeRun.Paths(new Args(argv), Name);
        var run = new CompositeRun(ctx, new Polite(ctx), mapPath, outDir, "currents", "currents");
        var polite = run.Polite;
        var dir = Path.Combine(outDir, "currents");
        var regions = run.Regions();
        var cur = new Currents().UsePolite(ctx, polite);
        var cacheDir = Path.Combine("cache", "currents");

        // ---- CO-OPS current predictions: every station; the regions only list which stand in them
        var picks = new Dictionary<string, (string name, int? bin)>();
        var regionStations = new List<List<string>>();
        foreach (var (box, _) in regions)
            regionStations.Add(cur.DiscoverPlace(box, 0.0, (box.CenterLon, box.CenterLat), int.MaxValue, Path.Combine("cache", "coops")).Select(f => f.id).ToList());
        foreach (var f in cur.DiscoverPlace(new Box(-180, -90, 180, 90), 0.0, (0.0, 0.0), int.MaxValue, Path.Combine("cache", "coops")))
            picks.TryAdd(f.id, (f.name, f.bin));
        var need = 2 * picks.Count(p => FirstWindow(cacheDir, p.Value.bin is int pb ? $"{p.Key}_b{pb}" : p.Key) is null);
        var (_, left) = polite.Remaining();
        run.Log($"{picks.Count} current station(s) published, {regionStations.Sum(r => r.Count)} inside the {regions.Count} region(s); {need} request(s) missing from the cache, {left} left in the ledger's budget");
        if (need > left)
            throw new HarvestStop($"[currents] {need} CO-OPS request(s) needed for {picks.Count} stations, {left} left in {polite.LedgerPath}'s budget: nothing fetched, the old output stands", 3);
        if (Directory.Exists(dir)) foreach (var old in Directory.GetFiles(dir)) File.Delete(old);
        Directory.CreateDirectory(dir);
        // DiscoverPlace cached the listing; its positions go on each station
        var listing = Load(Path.Combine("cache", "coops", "_lists", "currentpredictions.json"));
        var pos = listing.Arr("stations").Where(s => s.Str("id") is not null).GroupBy(s => s.Str("id")!).ToDictionary(g => g.Key, g => (lat: g.First().Num("lat"), lon: g.First().Num("lng")));
        var stations = new List<Dictionary<string, object?>>();
        var failed = new List<Dictionary<string, object?>>();
        var windows = new List<DateTime>();
        foreach (var (sid, (name, bin)) in picks.OrderBy(p => p.Key, StringComparer.Ordinal))
        {
            var tag = bin is int b ? $"{sid}_b{b}" : sid;
            var first = FirstWindow(cacheDir, tag);
            try
            {
                var st = cur.FetchTidalStation(sid, name, cacheDir, bin, first);
                windows.Add(first ?? DateTime.UtcNow.AddDays(-10));
                if (pos.TryGetValue(sid, out var p)) { st["lat"] = p.lat; st["lon"] = p.lon; }
                st["source"] = "noaa.coops";
                stations.Add(st);
            }
            catch (Polite.StopException) { throw; }
            catch (Exception e)
            {
                run.Log($"[{sid}] FAILED: {e.Message}");
                failed.Add(D(("id", sid), ("bin", bin), ("why", e.Message)));
            }
        }
        run.Log($"{stations.Count} current station(s) in {regions.Count} region(s)");
        if (stations.Count > 0)
            run.Input("noaa.coops.currents", "NOAA CO-OPS tidal current predictions (MAX_SLACK, 45 days)", null, "https://api.tidesandcurrents.noaa.gov/api/prod/",
                "public domain (NOAA CO-OPS)", "NOAA CO-OPS Tides & Currents", "n/a", "knots to m/s (x 0.514444), signed flood +",
                validUtc: $"{windows.Min():yyyy-MM-dd} .. {windows.Max().AddDays(45):yyyy-MM-dd}");

        // ---- NDBC ADCP buoys
        var firstListing = FirstCopy.Earliest(NdbcStations.Dir, "_realtime2_listing_", ".html") ?? NdbcStations.ListingPath(DateTime.UtcNow);
        var cat = NdbcStations.Read(polite, firstListing);
        var adcps = new List<Dictionary<string, object?>>();
        foreach (var (bid, lat, lon, bname) in regions.SelectMany(r => NdbcStations.Inside(cat, r.box, ["adcp"])).DistinctBy(x => x.sid))
        {
            var have = FirstCopy.Earliest(cacheDir, $"{bid}_", ".adcp");
            string text;
            try { (text, _) = polite.GetTextP($"{NdbcStations.Root}/data/realtime2/{bid}.adcp", have ?? Path.Combine(cacheDir, $"{bid}_{DateTime.UtcNow:yyyyMMddHH}.adcp")); }
            catch (Polite.StopException) { throw; }
            catch (Exception e) { run.Log($"buoy {bid}: no .adcp ({e.Message})"); continue; }
            if (Currents.ParseAdcp(bid, text) is not { } a) continue;
            a["lat"] = lat; a["lon"] = lon; a["name"] = bname;
            adcps.Add(a);
        }
        var finest = regions.Count > 0 ? regions[0].box : null;
        var nearest = finest is null ? null : adcps.OrderBy(a => Geo.FlatKm(finest.CenterLon, finest.CenterLat, (double)a["lon"]!, (double)a["lat"]!)).FirstOrDefault();
        if (adcps.Count > 0)
            run.Input("noaa.ndbc.adcp", "NOAA NDBC realtime2 ADCP (latest line of the first copy)", null, NdbcStations.Root + "/data/realtime2/", "public domain (NOAA NWS)", "NOAA NDBC", "n/a", "cm/s to m/s",
                validUtc: IsoZ(FromUnix(adcps.Min(a => (double)a["obs_unix"]!))), fetchedUtc: CompositeRun.FileUtc(firstListing));

        // ---- NOS OFS surface fields
        var fields = new List<(Dictionary<string, object?> meta, double grain)>();
        var skipped = new List<Dictionary<string, object?>>();
        var datum = Ofs.GridDatum(polite);
        var roms = datum.Select(o => Regex.Match(o.Key, @"OFS_Grid_Datum/(\w+)\.romsgrid\.nc$")).Where(m => m.Success).Select(m => m.Groups[1].Value).ToHashSet();
        foreach (var o in datum)
        {
            var m = Regex.Match(o.Key, @"OFS_Grid_Datum/(\w+)_vdatums\.nc$");
            if (!m.Success) continue;
            var ofs = m.Groups[1].Value;
            var vpath = ctx.Big("ofs", ofs, $"{ofs}_vdatums.nc");
            var plan = polite.NewPlan($"{ofs} vertical-datum grid (its domain)");
            plan.Add(Path.GetFileName(vpath), $"https://{Ofs.Bucket}.s3.amazonaws.com/{o.Key}", o.Size, vpath);
            var (keep, _) = plan.Fit();
            foreach (var it in keep.Where(i => !i.Cached)) polite.Download(it.Url, it.Dest, it.Size!.Value, note: "compose-currents domain");
            if (!File.Exists(vpath)) continue;
            var dom = Domain(vpath);
            var meets = regions.Select((r, i) => (r, i)).Where(x => dom is { } d && x.r.box.Meets(d.Lon0, d.Lat0, d.Lon1, d.Lat1)).ToList();
            if (meets.Count == 0) continue;
            if (!roms.Contains(ofs))
            {
                skipped.Add(D(("model", $"noaa.{ofs}"), ("why", "no <ofs>.romsgrid.nc in OFS_Grid_Datum/: not a ROMS grid this reader scatters (an FVCOM mesh, or none published)"), ("regions", meets.Select(x => x.i).ToList())));
                continue;
            }
            string? fieldsFile;
            try { fieldsFile = FirstFieldsFile(ctx, ofs) ?? FetchCycle(ctx, polite, ofs); }
            catch (Polite.StopException) { throw; }
            catch (Exception e) { skipped.Add(D(("model", $"noaa.{ofs}"), ("why", e.Message))); continue; }
            if (fieldsFile is null) { skipped.Add(D(("model", $"noaa.{ofs}"), ("why", "no nowcast cycle fetched (budget or listing)"))); continue; }
            run.ReadBytes(new FileInfo(fieldsFile).Length);
            string? valid = null;
            foreach (var (r, i) in meets)
            {
                var stepDeg = r.grainM / 111195.0;
                var conv = Ofs.Convert(ofs, fieldsFile, r.box, null, run.Log);
                if (conv is null) continue;
                var model = conv.Value.declared["rho_spacing_deg"] is double rs ? rs : double.NaN;
                if (stepDeg > model) conv = Ofs.Convert(ofs, fieldsFile, r.box, stepDeg, run.Log);
                var (f, declared) = conv!.Value;
                if (!f.U.Any(x => x > -900)) continue;
                if ((long)f.Nx * f.Ny >= MaxFieldCells) { skipped.Add(D(("model", $"noaa.{ofs}"), ("region", i), ("why", $"{f.Nx}x{f.Ny} cells: past the engine's 2^24 a field"))); continue; }
                f.File = $"{ofs}_r{i}_uv.f32";
                using (var fs = new FileStream(Path.Combine(dir, f.File), FileMode.Create))
                {
                    fs.Write(System.Runtime.InteropServices.MemoryMarshal.AsBytes<float>(f.U));
                    fs.Write(System.Runtime.InteropServices.MemoryMarshal.AsBytes<float>(f.V));
                }
                var meta = f.Meta();
                var grain = f.Dlat * 111195.0;
                meta["model"] = $"noaa.{ofs}";
                meta["grain_m"] = Round(grain, 1);
                meta["box"] = new List<double> { f.Lon0, f.Lat0, f.Lon0 + f.Nx * f.Dlon, f.Lat0 + f.Ny * f.Dlat };
                meta["wanted_grain_m"] = r.grainM;
                meta["row0"] = "south";
                meta["layout"] = "u plane then v plane, float32, -999 = land";
                fields.Add((meta, grain));
                valid = f.ValidUtc;
                run.Coverage.Add(D(("box", meta["box"]), ("grain_m", meta["grain_m"]), ("input", $"noaa.{ofs}"), ("file", $"currents/{f.File}")));
            }
            run.Input($"noaa.{ofs}", $"NOS {ofs.ToUpperInvariant()} nowcast, surface u/v (ROMS)", null, $"https://{Ofs.Bucket}.s3.amazonaws.com/{ofs}/", "public domain (NOAA NOS); NODD: open to the public",
                $"NOAA NOS {ofs.ToUpperInvariant()}", "n/a", "rotated to east/north by the grid angle", validUtc: valid, fetchedUtc: CompositeRun.FileUtc(fieldsFile));
        }

        for (var i = 0; i < regions.Count; i++)
            run.Coverage.Add(D(("box", new List<double> { regions[i].box.Lon0, regions[i].box.Lat0, regions[i].box.Lon1, regions[i].box.Lat1 }), ("grain_m", regions[i].grainM),
                ("stations", regionStations[i]), ("input", "noaa.coops.currents")));
        var ordered = fields.OrderBy(f => f.grain).Select(f => f.meta).ToList();
        var doc = D(("generated_utc", NowIso()), ("tidal_stations", stations), ("buoy_adcp", nearest), ("buoys_adcp", adcps),
            ("field", ordered.FirstOrDefault()), ("fields", ordered));
        var path = Path.Combine(dir, "currents.json");
        Save(path, doc);
        run.WroteBytes(Directory.GetFiles(dir).Sum(f => new FileInfo(f).Length));
        run.Extra["failed"] = failed;
        run.Extra["skipped"] = skipped;
        run.Save(D(("file", "currents/currents.json"), ("unit", "m/s"), ("datum", "n/a"), ("stations", stations.Count), ("fields", ordered.Count), ("adcp_buoys", adcps.Count)));
        run.Log($"{stations.Count} stations, {ordered.Count} field(s), {adcps.Count} ADCP buoy(s); skipped {skipped.Count}");
        return 0;
    }

    /// The begin date of the first prediction window cached for a station (`{tag}_{begin}_{end}.json`).
    static DateTime? FirstWindow(string cacheDir, string tag)
    {
        if (!Directory.Exists(cacheDir)) return null;
        var rx = new Regex("^" + Regex.Escape(tag) + @"_(\d{8})_(\d{8})\.json$");
        var first = Directory.GetFiles(cacheDir).Select(f => rx.Match(Path.GetFileName(f))).Where(m => m.Success).Select(m => m.Groups[1].Value).OrderBy(s => s, StringComparer.Ordinal).FirstOrDefault();
        return first is null ? null : DateTime.ParseExact(first, "yyyyMMdd", CultureInfo.InvariantCulture, DateTimeStyles.AssumeUniversal | DateTimeStyles.AdjustToUniversal);
    }

    /// The model domain's lon/lat box from its vertical-datum grid.
    static Box? Domain(string vdatums)
    {
        var nc = new NetCdf3(File.ReadAllBytes(vdatums));
        if (!nc.Vars.ContainsKey("longitude") || !nc.Vars.ContainsKey("latitude")) return null;
        var lon = nc.Read("longitude").values.Where(v => !double.IsNaN(v) && Math.Abs(v) <= 360).Select(v => v > 180 ? v - 360 : v).ToList();
        var lat = nc.Read("latitude").values.Where(v => !double.IsNaN(v) && Math.Abs(v) <= 90).ToList();
        return lon.Count == 0 ? null : new Box(lon.Min(), lat.Min(), lon.Max(), lat.Max());
    }

    /// The first cycle on the big drive's best surface file (Ofs.SurfaceRank), if any.
    static string? FirstFieldsFile(HarvestContext ctx, string ofs)
    {
        var root = ctx.Big("ofs", ofs);
        if (!Directory.Exists(root)) return null;
        foreach (var d in Directory.GetDirectories(root).Where(d => Regex.IsMatch(Path.GetFileName(d), @"^\d{8}$")).OrderBy(d => d, StringComparer.Ordinal))
        {
            var f = Directory.GetFiles(d, $"{ofs}.t*z.*.nc").Where(x => !double.IsInfinity(Ofs.SurfaceRank(x))).OrderBy(Ofs.SurfaceRank).FirstOrDefault();
            if (f is not null) return f;
        }
        return null;
    }

    /// The newest cycle's best surface file (Ofs.SurfaceRank), one file, planned against the budget.
    static string? FetchCycle(HarvestContext ctx, Polite polite, string ofs)
    {
        if (Ofs.SurfaceFile(polite, ofs, DateTime.UtcNow) is not var (key, size, ymd)) return null;
        var name = key[(key.LastIndexOf('/') + 1)..];
        var dest = ctx.Big("ofs", ofs, ymd, name);
        var plan = polite.NewPlan($"{ofs.ToUpperInvariant()} surface {name}");
        plan.Add(name, $"https://{Ofs.Bucket}.s3.amazonaws.com/{key}", size, dest);
        var (keep, cut) = plan.Fit();
        plan.Print(keep, cut);
        if (keep.Count == 0) return null;
        polite.Download(keep[0].Url, dest, keep[0].Size!.Value, note: "compose-currents");
        return dest;
    }
}
