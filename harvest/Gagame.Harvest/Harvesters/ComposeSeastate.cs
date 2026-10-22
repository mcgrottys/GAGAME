using System.Globalization;
using System.Text.RegularExpressions;
using Gagame.Harvest.Readers;
using static Gagame.Harvest.JsonUtil;

namespace Gagame.Harvest.Harvesters;

/// THE SEA STATE, MANY SOURCES IN, ONE FILE OUT, in data/sea/seastate.json's schema
/// (sim/SeaState.cpp) grown to hold every region at once:
///
///   points   GFS-Wave forecast points, each a model node with its own lat/lon, cycle, grib box
///            and 13 forecast hours (f000..f048 by 4: wind, combined sea, wind sea + swell
///            partitions) -- Waves.HourEntry, the place's own hour
///   buoys    every station of every observation source (harvest/sources/buoys, made by the
///            SourceFactory: NDBC realtime2, ERDDAP tabledap networks ...) inside a painted region,
///            its latest line (and spectrum where the source has one) in the engine's units, with
///            its lat/lon; a station the better-ranked source answered is not asked again
///   hours, point_lat, point_lon, box, cycle_unix, cycle_label
///            the finest region's first point, for a reader that knows one point
///
/// WHERE THE POINTS STAND. A region wanting grain g takes the model's 0.25 deg nodes at stride
/// n = ceil(g / node spacing) whose half-spacing neighbourhood meets it and where the model has
/// sea: the place's "nearest wet node" is the n = 1 case of the same law.
///
/// THE REQUESTS. Regions whose padded boxes overlap share one grib-filter subregion; each costs
/// 13 requests a cycle. THE FIRST COPY (Mark, 2026-10-13): a cycle already cached anywhere under
/// cache/gfswave whose grid covers a subregion is read, the earliest first, and nothing is
/// fetched; a buoy's first cached line is its line.
public sealed class ComposeSeastate : IHarvester
{
    public string Name => "compose-seastate";
    public string Title => "Sea-state composite (GFS-Wave + the buoy sources by configuration)";
    public string What => "GFS-Wave forecast points chosen by the fidelity map and every buoy of the configured sources (harvest/sources/buoys) in the painted regions: one seastate.json.";
    public string[] Outputs => [];
    public Source[] Sources => [new("NOMADS GFS-Wave", "https://nomads.ncep.noaa.gov/"), new("NDBC realtime2", "https://www.ndbc.noaa.gov/data/realtime2/"),
        new("ERDDAP tabledap (NDBC on CoastWatch, the Irish buoys)", "https://coastwatch.pfeg.noaa.gov/erddap/tabledap/cwwcNDBCMet.html")];
    public Insert? Insert => new(InsertKind.FileArg, "A fidelity map (.gafid) and an output folder: --fidelity FILE --out data/composite/<name>/ [--sources id,id]");

    const double NodeDeg = 0.25, NodeM = NodeDeg * 111195.0;

    sealed class Cluster
    {
        public Box Box = null!;
        public List<int> Regions = new();
    }

    public int Run(HarvestContext ctx, string[] argv)
    {
        var (mapPath, outDir) = CompositeRun.Paths(new Args(argv), Name);
        var run = new CompositeRun(ctx, new Polite(ctx), mapPath, outDir, "seastate", "seastate");
        var polite = run.Polite;
        var dir = Path.Combine(outDir, "sea");
        Directory.CreateDirectory(dir);
        var regions = run.Regions();
        int Stride(double g) => Math.Max(1, (int)Math.Ceiling(g / NodeM - 1e-9));

        // ---- subregions: padded region boxes that overlap share one
        var clusters = new List<Cluster>();
        for (var i = 0; i < regions.Count; i++)
        {
            var pad = 0.5 * Stride(regions[i].grainM) * NodeDeg;
            var b = regions[i].box;
            var box = new Box(Math.Floor((b.Lon0 - pad) / NodeDeg) * NodeDeg, Math.Floor((b.Lat0 - pad) / NodeDeg) * NodeDeg,
                Math.Ceiling((b.Lon1 + pad) / NodeDeg) * NodeDeg, Math.Ceiling((b.Lat1 + pad) / NodeDeg) * NodeDeg);
            var hit = clusters.FirstOrDefault(c => c.Box.Meets(box.Lon0, box.Lat0, box.Lon1, box.Lat1));
            if (hit is null) clusters.Add(new Cluster { Box = box, Regions = { i } });
            else
            {
                hit.Box = new Box(Math.Min(hit.Box.Lon0, box.Lon0), Math.Min(hit.Box.Lat0, box.Lat0), Math.Max(hit.Box.Lon1, box.Lon1), Math.Max(hit.Box.Lat1, box.Lat1));
                hit.Regions.Add(i);
            }
        }

        // ---- every subregion's cycle, and the points of its regions
        var points = new List<(int region, Dictionary<string, object?> p)>();
        var taken = new HashSet<(long, long)>();
        var cycles = new List<string>();
        var bytesIn = 0L;
        foreach (var cl in clusters)
        {
            var gb = new Waves.GribBox(Grib2Message.Mod360(cl.Box.Lon0), Grib2Message.Mod360(cl.Box.Lon0) + (cl.Box.Lon1 - cl.Box.Lon0), cl.Box.Lat1, cl.Box.Lat0);
            var first = FirstCycle(gb);
            var cacheDir = first?.cacheDir ?? Path.Combine("cache", "gfswave", "box_" + string.Join("_", new[] { gb.LeftLon, gb.BottomLat, gb.RightLon, gb.TopLat }.Select(v => v.ToString("0.##", CultureInfo.InvariantCulture))));
            var (cycDt, ymd, cyc, hours) = new Waves().CycleOver(ctx, polite, gb, cacheDir, first?.ymd, first?.cyc);
            bytesIn += Directory.GetFiles(Path.Combine(cacheDir, ymd), $"{cyc:00}z_f*_all.grib2").Sum(f => new FileInfo(f).Length);
            var label = $"gfswave {ymd} {cyc:00}z";
            cycles.Add(label);
            run.Log($"subregion {gb}: {label}{(first is not null ? " (cached)" : "")}");
            var hs0 = hours[0].msgs.First(x => x.name == "HTSGW").m;
            foreach (var i in cl.Regions.OrderBy(i => regions[i].grainM))
            {
                var (box, g) = regions[i];
                var n = Stride(g);
                var half = 0.5 * n * NodeDeg;
                for (var r = 0; r < hs0.Nj; r++)
                    for (var c = 0; c < hs0.Ni; c++)
                    {
                        if (double.IsNaN(hs0.Values[r * hs0.Ni + c])) continue;
                        var lat = hs0.Lat1 + r * ((hs0.Scan & 0x40) != 0 ? hs0.Dlat : -hs0.Dlat);
                        var lonE = Grib2Message.Mod360(hs0.Lon1 + c * ((hs0.Scan & 0x80) != 0 ? -hs0.Dlon : hs0.Dlon));
                        var lon = lonE > 180 ? lonE - 360 : lonE;
                        long ia = (long)Math.Round((lat + 90) / NodeDeg), io = (long)Math.Round(lonE / NodeDeg);
                        if (ia % n != 0 || io % n != 0) continue;
                        if (lat < box.Lat0 - half || lat > box.Lat1 + half || lon < box.Lon0 - half || lon > box.Lon1 + half) continue;
                        if (!taken.Add((ia, io))) continue;
                        var hrs = hours.Select(h =>
                        {
                            var pt = new Dictionary<string, double?>();
                            foreach (var (name, m) in h.msgs) { var v = m.Values[r * m.Ni + c]; pt[name] = double.IsNaN(v) ? null : v; }
                            return Waves.HourEntry(h.fh, pt);
                        }).ToList();
                        string F(double v) => v.ToString("0.###", CultureInfo.InvariantCulture);
                        points.Add((i, D(("id", $"gfswave:{F(lat)},{F(lon)}"), ("lat", lat), ("lon", lon), ("grain_m", Round(n * NodeM, 1)), ("wanted_grain_m", g),
                            ("cycle_unix", Unix(cycDt)), ("cycle_label", label), ("box", gb.ToJson()), ("hours", hrs))));
                    }
            }
            run.Input("noaa.gfswave", "NCEP GFS-Wave 0.25 deg global, through the NOMADS grib filter", Round(NodeM, 1), "https://nomads.ncep.noaa.gov/cgi-bin/filter_gfswave.pl",
                "public domain (NOAA NWS)", "NOAA NCEP GFS-Wave (WAVEWATCH III)", "n/a", "none", validUtc: string.Join("; ", cycles.Distinct()),
                fetchedUtc: CompositeRun.FileUtc(Directory.GetFiles(Path.Combine(cacheDir, ymd), $"{cyc:00}z_f000_all.grib2")[0]));
        }
        run.ReadBytes(bytesIn);
        run.Log($"{points.Count} forecast point(s) in {clusters.Count} subregion(s)");

        // ---- the buoys: every observation source (harvest/sources/buoys, or --sources), ranked;
        // a station a better-ranked source answered is not asked of the next
        var sources = SourceFactory.ObservationSources(new SourceEnv(ctx, polite, run.Log), new Args(argv).Opt("--sources"));
        var found = new Dictionary<string, Dictionary<string, object?>>(StringComparer.Ordinal);
        var regionBuoys = regions.Select(_ => new SortedSet<string>(StringComparer.Ordinal)).ToList();
        var answered = new List<string>();
        // A HOST THAT REFUSES (2026-10-21): the buoys are assimilated on top of the forecast points,
        // so they are optional. Polite's stop for the host holds (its one retry spent, nothing more
        // is asked of it in this run), but the layer is still written: the forecast points, and
        // no buoys from that host; the refusal is said once here and in the composite's
        // `refused`. A budget stop (no inner exception) still stops the layer.
        var refused = new List<Dictionary<string, object?>>();
        var refusedHosts = new HashSet<string>(StringComparer.OrdinalIgnoreCase);
        bool Refused(Polite.StopException e, string id, string host)
        {
            if (e.InnerException is null) return false;
            host = e.Message.Split(' ')[0];   // Polite says "<host> refused twice (...)"
            refusedHosts.Add(host);
            refused.Add(D(("source", id), ("host", host), ("why", e.Message)));
            run.Log($"{id}: {host} refused ({e.Message}); no buoys from it in this product, the forecast points stand");
            return true;
        }
        foreach (var src in sources)
        {
            var host = Polite.HostOf(src.Config.Server ?? src.Config.Url ?? "");
            if (refusedHosts.Contains(host)) { run.Log($"{src.Config.Id}: {host} refused earlier in this run; not asked"); continue; }
            List<IReadOnlyList<Station>> inside;
            try { inside = regions.Select(r => src.Stations(r.box)).ToList(); }
            catch (Polite.StopException e) when (Refused(e, src.Config.Id, host)) { continue; }
            catch (Exception e) when (e is not Polite.StopException) { run.Log($"{src.Config.Id}: no stations ({e.Message}); the source is left out of this product"); continue; }
            for (var i = 0; i < regions.Count; i++) regionBuoys[i].UnionWith(inside[i].Select(s => s.Id));
            string? firstObs = null;
            var got = 0;
            foreach (var st in inside.SelectMany(x => x).DistinctBy(x => x.Id).OrderBy(x => x.Id, StringComparer.Ordinal))
            {
                if (found.ContainsKey(st.Id)) continue;
                Dictionary<string, object?>? b;
                try { b = src.Observe(st); }
                catch (Polite.StopException e) when (Refused(e, src.Config.Id, host)) { break; }
                catch (Exception e) when (e is not Polite.StopException) { run.Log($"buoy {st.Id}: no line from {src.Config.Id} ({e.Message})"); continue; }
                if (b is null) continue;
                b["lat"] = st.Lat; b["lon"] = st.Lon; b["name"] = st.Name;
                found[st.Id] = b;
                got++;
                if (b.GetValueOrDefault("obs_unix") is double ou && (firstObs is null || string.CompareOrdinal(IsoZ(FromUnix(ou)), firstObs) < 0)) firstObs = IsoZ(FromUnix(ou));
            }
            if (got > 0)
            {
                answered.Add(src.Config.Label);
                run.Input(src.Config.Id, src.Config.Title, null, src.Config.Url ?? src.Config.Server ?? "", src.Config.License, src.Config.Cite, "n/a", "none", validUtc: firstObs, fetchedUtc: src.FetchedUtc);
            }
            run.Log($"{src.Config.Id}: {got} buoy(s) in the regions");
        }
        var buoys = found.OrderBy(kv => kv.Key, StringComparer.Ordinal).ToDictionary(kv => kv.Key, kv => kv.Value);
        run.Log($"{buoys.Count} buoy(s) in the regions");
        if (refused.Count > 0) run.Extra["refused"] = refused;

        for (var i = 0; i < regions.Count; i++)
            run.Coverage.Add(D(("box", new List<double> { regions[i].box.Lon0, regions[i].box.Lat0, regions[i].box.Lon1, regions[i].box.Lat1 }), ("grain_m", regions[i].grainM),
                ("points", points.Count(p => p.region == i)), ("buoys", regionBuoys[i].ToList()), ("input", "noaa.gfswave")));
        var ordered = points.Select(p => p.p).ToList();
        var head = ordered.FirstOrDefault();
        var doc = D(("generated_utc", NowIso()), ("source", $"GFS-Wave via NOMADS grib filter at the fidelity map's points + {string.Join(" + ", answered)} in its regions"),
            ("cycle_unix", head?["cycle_unix"]), ("cycle_label", head?["cycle_label"]), ("point_lat", head?["lat"]), ("point_lon", head?["lon"]), ("box", head?["box"]),
            ("hours", head?["hours"] ?? new List<object?>()), ("points", ordered), ("buoys", buoys));
        var path = Path.Combine(dir, "seastate.json");
        Save(path, doc);
        run.WroteBytes(new FileInfo(path).Length);
        run.Save(D(("file", "sea/seastate.json"), ("unit", "m, s, degrees true (from), m/s"), ("datum", "n/a"), ("points", ordered.Count), ("buoys", buoys.Count), ("subregions", clusters.Count)));
        return 0;
    }

    /// The earliest cycle cached anywhere under cache/gfswave with all forecast hours whose f000
    /// grid covers the subregion: (the folder holding its day folder, day, cycle).
    static (string cacheDir, string ymd, int cyc)? FirstCycle(Waves.GribBox gb)
    {
        var root = Path.Combine("cache", "gfswave");
        if (!Directory.Exists(root)) return null;
        var rx = new Regex(@"^(\d\d)z_f048_all\.grib2$");
        var found = new List<(string, string, int)>();
        foreach (var f48 in Directory.EnumerateFiles(root, "*z_f048_all.grib2", SearchOption.AllDirectories))
        {
            var m = rx.Match(Path.GetFileName(f48));
            var day = Path.GetDirectoryName(f48)!;
            var ymd = Path.GetFileName(day);
            if (!m.Success || !Regex.IsMatch(ymd, @"^\d{8}$")) continue;
            var cyc = int.Parse(m.Groups[1].Value);
            if (!Waves.ForecastHours.All(fh => File.Exists(Path.Combine(day, $"{cyc:00}z_f{fh:000}_all.grib2")))) continue;
            var g = Grib2.ReadMessages(File.ReadAllBytes(Path.Combine(day, $"{cyc:00}z_f000_all.grib2"))).FirstOrDefault();
            if (g is null) continue;
            double latA = g.Lat1, latB = g.Lat1 + (g.Nj - 1) * ((g.Scan & 0x40) != 0 ? g.Dlat : -g.Dlat);
            double lonA = Grib2Message.Mod360(g.Lon1), lonB = lonA + (g.Ni - 1) * g.Dlon;
            // the filter answers [left, right) in longitude and [bottom, top] in latitude, its nodes
            // a few 1e-5 deg off the quarter degrees: compare to a thousandth
            if (Math.Min(latA, latB) <= gb.BottomLat + 1e-3 && Math.Max(latA, latB) >= gb.TopLat - 1e-3 && lonA <= gb.LeftLon + 1e-3 && lonB >= gb.RightLon - NodeDeg - 1e-3)
                found.Add((Path.GetDirectoryName(day)!, ymd, cyc));
        }
        return found.Count == 0 ? null : found.OrderBy(x => x.Item2, StringComparer.Ordinal).ThenBy(x => x.Item3).First();
    }
}
