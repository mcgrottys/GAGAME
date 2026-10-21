using System.Globalization;
using System.Text.Json.Nodes;
using static Gagame.Harvest.JsonUtil;

namespace Gagame.Harvest.Harvesters;

/// THE RIVERS THAT MEET THE SEA IN A PAINTED REGION, AS LINES (Mark, 2026-10-21: a dataset is
/// never a place's, only a measurement's). For any solver box anywhere the engine derives where a
/// river crosses the box's edge, its discharge (a gauge's times the drainage-area ratio) and the
/// tide stations that stand on it at which river kilometre, from these reaches; nothing is
/// configured per river.
///
/// How a reach is found, from the data alone (no distance constant, no inland expansion):
///  1. THE MOUTHS. Every NHDPlus V2 network flowline with terminalfl = 1 (a network's last
///     flowline) that the region's box touches (the USGS wmadata WFS, one request a region), kept
///     where its OUTLET -- the flowline's downstream end -- lies inside the box: there the river
///     meets the sea in the region.
///  2. THE GAUGES ON ITS MAIN STEM. NLDI's upstream-main navigation from that terminal to the
///     source (`comid/{T}/navigation/UM/nwissite?distance=9999`, one request a mouth) lists every
///     NWIS site on the main stem with its flowline; the NWIS site service (expanded output, one
///     request per 100 ids) keeps the active stream gauges measuring discharge (00060) or stage
///     (00065) and gives each its drainage area. The main stem bounds the search, so a basin of
///     any size is searched whole and nothing off it is asked. This is the same set the
///     "downstream-main from every gauge, grouped by terminus" walk would give, less the
///     tributary gauges, which are not on the line and so have no river kilometre.
///     A mouth with no such gauge is not a reach (no discharge to give).
///  3. THE LINE. The main stem is the terminal's NHDPlus level path (every flowline with
///     levelpathi = the terminal's, one WFS request a reach), ordered by hydroseq from the mouth
///     up and cut after the farthest flowline holding a kept gauge or tide station. Each vertex
///     is [lon, lat, km from the mouth (the chained great-circle length of the NHD geometry), the
///     drainage area of its flowline (NHDPlus totdasqkm, the total upstream basin), 1 where its
///     flowline is tidal (NHDPlus `tidal`) else 0]; `tidal_km` is the km of the last vertex of
///     the tidal run that starts at the mouth: the head of tide as NHDPlus draws it.
///  4. THE TIDES. Every CO-OPS harmonic station (the set compose-tides fits) within the stem's box asks
///     NLDI for its own flowline (`comid/position`, one request a station, cached); a station
///     whose flowline is on the stem is on the reach, its km the projection onto that flowline's
///     part of the line.
/// Every response is cached under cache/river/ forever; NLDI and NWIS answer 404 where nothing
/// matches, kept as an empty first copy.
public static class RiverReaches
{
    const string WfsBase = "https://api.water.usgs.gov/geoserver/wmadata/ows?service=WFS&version=1.0.0&request=GetFeature&typeName=wmadata:nhdflowline_network&outputFormat=application/json"
        + "&propertyName=comid,gnis_name,totdasqkm,hydroseq,levelpathi,pathlength,";
    const string Wfs = WfsBase + "the_geom&CQL_FILTER=";
    /// The stem also asks NHDPlus's `tidal` value-added attribute (1 where the flowline is tidal).
    const string WfsStem = WfsBase + "tidal,the_geom&CQL_FILTER=";
    public const string Nldi = "https://api.water.usgs.gov/nldi/linked-data/";
    const string NwisExpanded = "https://waterservices.usgs.gov/nwis/site/?format=rdb&siteOutput=expanded&siteType=ST&siteStatus=active&parameterCd=00060,00065&sites=";
    const double SqMiToKm2 = 2.589988110336;
    static readonly CultureInfo Inv = CultureInfo.InvariantCulture;

    public sealed record Flow(long Comid, string Name, double TotDaKm2, double Hydroseq, double LevelPath, double PathLengthKm, int? Tidal, List<(double lon, double lat)> Pts);
    public sealed record Gauge(string Id, string Name, double Lat, double Lon, double? DrainKm2);
    sealed record V(double Lon, double Lat, double Km, double Da, long Comid, int Tidal);

    public sealed class Reach
    {
        public required Flow Terminal;
        public required int Region;
        public List<(Gauge g, long comid)> Gauges = new();
        public List<(string id, long comid, double lat, double lon)> Tides = new();
    }

    static string F(double v) => v.ToString("R", Inv);

    /// GET with a first copy forever; a 404 is kept as an empty copy and read as "".
    static string Fetch(CompositeRun run, string url, string cachePath)
    {
        Directory.CreateDirectory(Path.GetDirectoryName(cachePath)!);
        try { return run.Polite.GetTextP(url, cachePath).text; }
        catch (HttpError e) when (e.Code == 404) { File.WriteAllText(cachePath, ""); return ""; }
    }

    static List<Flow> Flows(string geojson)
    {
        var list = new List<Flow>();
        if (geojson.Trim().Length == 0) return list;
        foreach (var f in Parse(geojson).Arr("features"))
        {
            var p = f.Obj("properties");
            var pts = new List<(double, double)>();
            var g = f.Obj("geometry");
            var parts = g.Str("type") == "LineString" ? new JsonArray(g["coordinates"]!.DeepClone()) : g.Arr("coordinates");
            foreach (var part in parts)
                foreach (var c in part!.AsArray())
                {
                    var (lon, lat) = (c![0]!.GetValue<double>(), c[1]!.GetValue<double>());
                    if (pts.Count == 0 || pts[^1] != (lon, lat)) pts.Add((lon, lat));
                }
            if (pts.Count < 2) continue;
            list.Add(new Flow((long)p.Num("comid"), p.Str("gnis_name")?.Trim() ?? "", p.Num("totdasqkm", double.NaN), p.Num("hydroseq"), p.Num("levelpathi"), p.Num("pathlength", double.NaN),
                p.NumOrNull("tidal") is double td ? (td != 0 ? 1 : 0) : null, pts));
        }
        return list;
    }

    static bool Inside(Box b, double lon, double lat) => lon >= b.Lon0 && lon <= b.Lon1 && lat >= b.Lat0 && lat <= b.Lat1;

    /// Steps 1 and 2: the mouths in each region and the gauges on their main stems. `regionGauges`
    /// (the region sites already found) join the one expanded NWIS query so every site gets its
    /// drainage area; the result's dictionary holds every gauge the query returned.
    public static (List<Reach> reaches, Dictionary<string, Gauge> gauges) Find(CompositeRun run, string cache, List<(Box box, double grainM)> regions, IEnumerable<string> regionGauges)
    {
        var terminals = new Dictionary<long, (Flow f, int region)>();
        for (var i = 0; i < regions.Count; i++)
        {
            var b = regions[i].box;
            var key = string.Join("_", new[] { b.Lon0, b.Lat0, b.Lon1, b.Lat1 }.Select(v => v.ToString("0.0000000", Inv)));
            var cql = $"terminalfl=1 AND BBOX(the_geom,{F(b.Lon0)},{F(b.Lat0)},{F(b.Lon1)},{F(b.Lat1)})";
            foreach (var f in Flows(Fetch(run, Wfs + Uri.EscapeDataString(cql), Path.Combine(cache, "nhd", $"terminals_{key}.json"))))
            {
                var (lon, lat) = f.Pts[^1];
                if (Inside(b, lon, lat) && !terminals.ContainsKey(f.Comid)) terminals[f.Comid] = (f, i);
            }
        }
        run.Log($"{terminals.Count} NHDPlus terminal flowline(s) with the outlet inside a region");

        var onStem = new Dictionary<long, List<(string id, long comid)>>();
        foreach (var (comid, _) in terminals.OrderBy(kv => kv.Key))
        {
            var text = Fetch(run, $"{Nldi}comid/{comid}/navigation/UM/nwissite?distance=9999", Path.Combine(cache, "nldi", $"um_nwissite_{comid}.json"));
            var here = new List<(string, long)>();
            if (text.Trim().Length > 0)
                foreach (var s in Parse(text).Arr("features"))
                {
                    var p = s.Obj("properties");
                    var ident = p.Str("identifier") ?? "";
                    if (ident.StartsWith("USGS-", StringComparison.Ordinal)) here.Add((ident[5..], (long)p.Num("comid")));
                }
            onStem[comid] = here;
        }

        var ids = onStem.Values.SelectMany(x => x.Select(y => y.id)).Concat(regionGauges).Distinct().Order(StringComparer.Ordinal).ToList();
        var gauges = new Dictionary<string, Gauge>();
        foreach (var chunk in ids.Chunk(100))
        {
            var list = string.Join(",", chunk);
            var tag = Bin.Fnv1a(System.Text.Encoding.ASCII.GetBytes(list)).ToString("x16");
            foreach (var g in ParseExpanded(Fetch(run, NwisExpanded + list, Path.Combine(cache, $"sites_x_{tag}.rdb")))) gauges[g.Id] = g;
        }

        var reaches = new List<Reach>();
        foreach (var (comid, (f, region)) in terminals.OrderBy(kv => kv.Key))
        {
            var r = new Reach { Terminal = f, Region = region };
            foreach (var (id, c) in onStem[comid]) if (gauges.TryGetValue(id, out var g) && r.Gauges.All(x => x.g.Id != id)) r.Gauges.Add((g, c));
            if (r.Gauges.Count > 0) reaches.Add(r);
        }
        run.Log($"{reaches.Count} mouth(s) with an active stream gauge on the main stem; {gauges.Count} gauge(s) described");
        return (reaches, gauges);
    }

    static IEnumerable<Gauge> ParseExpanded(string rdb)
    {
        string[]? cols = null;
        foreach (var ln in rdb.Split('\n'))
        {
            if (ln.StartsWith('#') || ln.Trim().Length == 0) continue;
            var f = ln.TrimEnd('\r').Split('\t');
            if (cols is null) { cols = f; continue; }
            if (f.Length != cols.Length || f[0].EndsWith('s')) continue;
            string Col(string c) => Array.IndexOf(cols, c) is int k and >= 0 ? f[k] : "";
            if (!double.TryParse(Col("dec_lat_va"), NumberStyles.Float, Inv, out var lat)) continue;
            if (!double.TryParse(Col("dec_long_va"), NumberStyles.Float, Inv, out var lon)) continue;
            double? da = double.TryParse(Col("drain_area_va"), NumberStyles.Float, Inv, out var sq) ? Round(sq * SqMiToKm2, 2) : null;
            yield return new Gauge(Col("site_no"), Col("station_nm"), lat, lon, da);
        }
    }

    /// Steps 3 and 4 for one reach: the stem, the tide stations on it, the line. `tides` are the
    /// composite's CO-OPS stations (id, lat, lon). `discharge` says which gauge ids report it.
    public static Dictionary<string, object?>? Build(CompositeRun run, string cache, Reach r, List<(string id, double lat, double lon)> tides, Func<string, bool> discharge)
    {
        var lp = r.Terminal.LevelPath.ToString("0", Inv);
        var stem = Flows(Fetch(run, WfsStem + Uri.EscapeDataString($"levelpathi={lp}"), Path.Combine(cache, "nhd", $"stem_{lp}.json")))
            .GroupBy(x => x.Comid).Select(x => x.First()).OrderBy(x => x.Hydroseq).ToList();
        if (stem.Count == 0 || stem[0].Comid != r.Terminal.Comid) { run.Log($"nhdplus:{r.Terminal.Comid}: the level path does not start at the terminal; skipped"); return null; }
        var index = stem.Select((f, i) => (f.Comid, i)).ToDictionary(x => x.Comid, x => x.i);

        // the tide stations on the stem: only those inside the whole stem's box ask NLDI
        double lo0 = stem.Min(f => f.Pts.Min(p => p.lon)), lo1 = stem.Max(f => f.Pts.Max(p => p.lon));
        double la0 = stem.Min(f => f.Pts.Min(p => p.lat)), la1 = stem.Max(f => f.Pts.Max(p => p.lat));
        const double pad = 0.02;   // a station in the mouth's bay can stand just outside the line's box
        foreach (var (id, lat, lon) in tides)
        {
            if (lon < lo0 - pad || lon > lo1 + pad || lat < la0 - pad || lat > la1 + pad) continue;
            var text = Fetch(run, $"{Nldi}comid/position?coords={Uri.EscapeDataString($"POINT({F(lon)} {F(lat)})")}", Path.Combine(cache, "nldi", $"position_{id}.json"));
            if (text.Trim().Length == 0) continue;
            var c = (long)(Parse(text).Arr("features").FirstOrDefault()?.Obj("properties").Num("comid") ?? 0);
            if (index.ContainsKey(c)) r.Tides.Add((id, c, lat, lon));
        }

        var cut = r.Gauges.Select(x => index.GetValueOrDefault(x.comid, -1)).Concat(r.Tides.Select(x => index[x.comid])).Max();
        if (cut < 0) { run.Log($"nhdplus:{r.Terminal.Comid}: no gauge's flowline is on the level path; skipped"); return null; }

        // the line, mouth first: each flowline is digitised upstream to downstream, so reversed
        var line = new List<V>();
        for (var i = 0; i <= cut; i++)
        {
            var f = stem[i];
            for (var k = f.Pts.Count - 1; k >= 0; k--)
            {
                var (lon, lat) = f.Pts[k];
                if (line.Count > 0 && line[^1].Lon == lon && line[^1].Lat == lat) continue;
                var km = line.Count == 0 ? 0.0 : line[^1].Km + Geo.Haversine_km(line[^1].Lat, line[^1].Lon, lat, lon);
                line.Add(new V(lon, lat, km, f.TotDaKm2, f.Comid, f.Tidal ?? 0));
            }
        }
        // NHDPlus's own pathlength (km from a flowline's downstream end to the terminal's outlet)
        // against the chained line, at the cut flowline's downstream end: logged, not used
        var top = stem[cut];
        var topKm = line.First(v => v.Comid == top.Comid).Km;
        var name = stem.Take(cut + 1).Select(f => f.Name).FirstOrDefault(n => n.Length > 0) ?? "";
        run.Log($"nhdplus:{r.Terminal.Comid} {name}: {cut + 1} flowline(s), {line.Count} vertices, {line[^1].Km:0.00} km; NHDPlus pathlength at the top flowline {top.PathLengthKm:0.000} km, the line {topKm:0.000} km");

        // THE HEAD OF TIDE, as NHDPlus flags it: the km of the last vertex of the tidal run that
        // starts at the mouth (0 when the mouth's flowline is not tidal; null when the attribute
        // is absent on any flowline of the line)
        double? tidalKm = null;
        if (stem.Take(cut + 1).All(f => f.Tidal is not null))
        {
            tidalKm = 0.0;
            foreach (var v in line) { if (v.Tidal == 0) break; tidalKm = v.Km; }
        }
        var gauges = r.Gauges.Select(x => (x.g, x.comid, p: Project(line, x.g.Lon, x.g.Lat, x.comid)))
            .OrderBy(x => x.p.km).Select(x => D(("id", x.g.Id), ("km", Round(x.p.km, 3)), ("drainage_km2", x.g.DrainKm2), ("discharge", discharge(x.g.Id)),
                ("off_m", Round(x.p.offM, 1)), ("comid", x.comid), ("name", x.g.Name))).ToList();
        var tideRows = r.Tides.Select(x => (x.id, x.comid, p: Project(line, x.lon, x.lat, x.comid)))
            .OrderBy(x => x.p.km).Select(x => D(("id", x.id), ("km", Round(x.p.km, 3)), ("off_m", Round(x.p.offM, 1)), ("comid", x.comid))).ToList();
        return D(("id", $"nhdplus:{r.Terminal.Comid}"), ("name", name),
            ("tidal_km", tidalKm is double tk ? Round(tk, 3) : null),
            ("line", line.Select(v => new List<double> { Round(v.Lon, 6), Round(v.Lat, 6), Round(v.Km, 3), Round(v.Da, 2), v.Tidal }).ToList()),
            ("gauges", gauges), ("tides", tideRows));
    }

    /// The point's km along the line and its distance off it, on the segments of its own flowline
    /// (a segment belongs to the flowline of its upstream vertex); on the whole line when none.
    static (double km, double offM) Project(List<V> line, double lon, double lat, long comid)
    {
        var any = Enumerable.Range(1, line.Count - 1).Any(i => line[i].Comid == comid);
        double best = double.MaxValue, bestKm = 0;
        var kx = Math.Cos(lat * Math.PI / 180) * 111.195;
        const double ky = 111.195;
        for (var i = 1; i < line.Count; i++)
        {
            if (any && line[i].Comid != comid) continue;
            var (a, b) = (line[i - 1], line[i]);
            double ax = (a.Lon - lon) * kx, ay = (a.Lat - lat) * ky, bx = (b.Lon - lon) * kx, by = (b.Lat - lat) * ky;
            double dx = bx - ax, dy = by - ay, L2 = dx * dx + dy * dy;
            var t = L2 > 0 ? Math.Clamp(-(ax * dx + ay * dy) / L2, 0, 1) : 0;
            double px = ax + t * dx, py = ay + t * dy, d = Math.Sqrt(px * px + py * py);
            if (d < best) { best = d; bestKm = a.Km + t * (b.Km - a.Km); }
        }
        return (bestKm, best * 1000);
    }
}
