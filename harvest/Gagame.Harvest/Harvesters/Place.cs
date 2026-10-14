using System.Globalization;
using System.Text.Json.Nodes;
using System.Text.RegularExpressions;
using Gagame.Harvest.Readers;
using static Gagame.Harvest.JsonUtil;

namespace Gagame.Harvest.Harvesters;

/// A PLACE, whole (harvest_place.py): one name, one box, every source the Merrimack has --
/// tides, currents or the NOS forecast system, waves, bathymetry, the survey shoreline check,
/// each source's terms, and the manifest. A place only ADDS: every write is exclusive-create,
/// each step skips itself when its output exists, the budget is harvester/polite.py's ledger.
public sealed class Place : IHarvester
{
    public string Name => "place";
    public string Title => "A whole place";
    public string What => "One name, one box, every source the Merrimack has: tides, currents or OFS, waves, bathy, survey, manifest.";
    public string[] Outputs => ["data/places/haulover.json"];
    public Source[] Sources => [new("NOAA NODD buckets", "https://registry.opendata.aws/collab/noaa/"), new("Place manifests (data/places)", "https://github.com/mcgrottys/GAGAME")];
    public Insert? Insert => new(InsertKind.PlaceBox, "Any place: its box, the anchor the chart is frozen at, and an open-sea point.", Extras: ["--anchor", "--sea"], Final: true);
    public string[] DefaultArgs => ["--place", "haulover"];
    public string? Note => "Download = Haulover (the shipped demos' second place). Chesapeake is also named.";

    const string OfsBucket = "noaa-nos-ofs-pds";
    const long SafetyBytes = 30_000_000;

    public sealed class Spec
    {
        public Box Box = new(0, 0, 0, 0);
        public string? BoxWhy;
        public (double lon, double lat) Anchor, Sea;
        public double TideRadiusKm = 20.0, CurrentRadiusKm = 20.0, AdcpRadiusKm = 60.0;
        public int MaxCurrentStations = 8;
        public string? Ofs;
        public Box? GridBox;
        public string? GridName;
    }

    public static readonly Dictionary<string, Spec> Places = new()
    {
        ["haulover"] = new Spec
        {
            Box = new(-80.22, 25.82, -80.03, 26.02),
            BoxWhy = "the coordinator's box kept as given: Bakers Haulover Inlet at its centre latitude, the north of Biscayne Bay to the west, the shelf edge (~-80.03, where the reef tract drops away) to the east; two CUDEM tiles (FL n26X00/n26X25 w080X25) cover it",
            Anchor = (-80.1225, 25.9030), Sea = (-80.03, 25.90),
            TideRadiusKm = 40.0, CurrentRadiusKm = 40.0, MaxCurrentStations = 8, AdcpRadiusKm = 60.0, Ofs = null,
        },
        ["chesapeake"] = new Spec
        {
            Box = new(-77.40, 36.80, -75.60, 39.60),
            BoxWhy = "the coordinator's box kept: it holds the whole tidal bay from the Virginia capes to the Susquehanna flats and the C&D canal, the tidal Potomac to Washington and the James to Richmond's fall line (-77.43, 0.03 deg outside the west edge); its east edge (-75.60) also takes in Delmarva's Atlantic barrier islands, which the tile order (nearest Cape Henry first) reaches only after the bay's mouth",
            Anchor = (-76.01, 36.93), Sea = (-75.75, 36.95),
            TideRadiusKm = 0.0, CurrentRadiusKm = 25.0, MaxCurrentStations = 8, AdcpRadiusKm = 60.0, Ofs = "cbofs",
            GridBox = new(-76.50, 36.80, -75.75, 37.25), GridName = "chesapeake_mouth",
        },
    };

    static readonly Dictionary<string, (string url, string[] patterns)> Terms = new()
    {
        ["noaa"] = ("https://www.noaa.gov/disclaimer", ["public domain", "copyright"]),
        ["coops"] = ("https://tidesandcurrents.noaa.gov/disclaimers.html", ["public domain", "copyright", "may be used", "freely"]),
        ["ndbc"] = ("https://www.ndbc.noaa.gov/disclaimer.shtml", ["public domain", "copyright", "may be used"]),
        ["nws"] = ("https://www.weather.gov/disclaimer", ["public domain"]),
        ["nodd_gfs"] = ("https://registry.opendata.aws/noaa-gfs-bdp-pds/", ["open to the public", "can be used as desired", "attribution"]),
        ["nodd_ofs"] = ("https://registry.opendata.aws/noaa-ofs/", ["open to the public", "can be used as desired", "attribution"]),
        ["nodd_lidar"] = ("https://registry.opendata.aws/noaa-coastal-lidar/", ["open to the public", "can be used as desired", "attribution"]),
    };

    HarvestContext _ctx = null!;
    Polite _polite = null!;
    string _place = "";
    Spec _p = null!;
    Dictionary<string, object?> _rec = new();

    void Log(string m) => _ctx.Log(m);
    static DateTime NowUtc => DateTime.UtcNow;

    /// Repository-relative with forward slashes when under the repository; absolute otherwise.
    string Rel(string path)
    {
        var a = Path.GetFullPath(path);
        var cwd = Directory.GetCurrentDirectory();
        if (a.StartsWith(cwd + Path.DirectorySeparatorChar, StringComparison.OrdinalIgnoreCase)) return Path.GetRelativePath(cwd, a).Replace('\\', '/');
        return a;
    }

    // ------------------------------------------------------------------ record

    string RecordPath() => Path.Combine("out", "places", $"{_place}.record.json");

    Dictionary<string, object?> Steps => (Dictionary<string, object?>)(_rec["steps"] ??= new Dictionary<string, object?>());
    Dictionary<string, object?>? Step(string s) => Steps.GetValueOrDefault(s) as Dictionary<string, object?>;

    void LoadRecord()
    {
        var node = TryLoad(RecordPath());
        if (node is not null) { _rec = (Dictionary<string, object?>)Plain(node)!; return; }
        _rec = D(("place", _place), ("box", _p.Box.ToJson()), ("anchor", LL(_p.Anchor)), ("sea", LL(_p.Sea)), ("steps", new Dictionary<string, object?>()), ("skipped", new List<object?>()));
    }

    static List<double> LL((double lon, double lat) p) => new() { p.lon, p.lat };

    void SaveRecord()
    {
        var path = RecordPath();
        Directory.CreateDirectory(Path.GetDirectoryName(path)!);
        Save(path + ".tmp", _rec);
        File.Move(path + ".tmp", path, overwrite: true);
    }

    void Skip(string source, string why)
    {
        var list = (_rec["skipped"] as List<object?>) ?? new List<object?>();
        list = list.Where(s => s is Dictionary<string, object?> d && (string?)d["source"] != source).ToList();
        list.Add(D(("source", source), ("why", why)));
        _rec["skipped"] = list;
        Log($"SKIPPED {source}: {why}");
    }

    // ------------------------------------------------------------------ NDBC

    /// The NDBC catalog with a realtime2 listing under six hours old (a fresh one otherwise).
    NdbcStations.Catalog NdbcCatalog()
    {
        Directory.CreateDirectory(NdbcStations.Dir);
        var fresh = Directory.GetFiles(NdbcStations.Dir, "_realtime2_listing_*").Select(Path.GetFileName).OrderBy(n => n, StringComparer.Ordinal).ToList();
        string? listing = null;
        if (fresh.Count > 0)
        {
            var stamp = fresh[^1]!["_realtime2_listing_".Length..^".html".Length];
            var t = DateTime.ParseExact(stamp, "yyyyMMddHH", CultureInfo.InvariantCulture, DateTimeStyles.AssumeUniversal | DateTimeStyles.AdjustToUniversal);
            if (NowUtc - t < TimeSpan.FromHours(6)) listing = Path.Combine(NdbcStations.Dir, fresh[^1]!);
        }
        return NdbcStations.Read(_polite, listing ?? NdbcStations.ListingPath(NowUtc));
    }

    // ------------------------------------------------------------------ steps

    void StepTides()
    {
        var out_ = Path.Combine("data", "tides", _place, "stations.json");
        if (File.Exists(out_)) { Log($"[tides] {out_} exists; step already done"); return; }
        var r = new Tides().HarvestPlace(_ctx, _polite, _place, _p.Box, _p.TideRadiusKm, anchor: _p.Anchor);
        Steps["tides"] = D(("files", r.Files.Select(f => new List<object?> { Rel(f.path), f.bytes }).ToList()), ("failed", r.Failed), ("subordinate", r.Subordinate),
            ("year", r.Year), ("epoch_unix", r.EpochUnix),
            ("stations", r.Stations.Select(s => D(("id", s["id"]), ("name", s["full_name"]), ("lat", s["lat"]), ("lon", s["lon"]),
                ("n_coeffs", ((List<Dictionary<string, object?>>)s["coeffs"]!).Count), ("fit_rms_m", s["fit_rms_m"]), ("mllw_minus_navd_m", s["mllw_minus_navd_m"]))).ToList()));
    }

    void StepCurrents(Currents.PlaceField? field = null)
    {
        var out_ = Path.Combine("data", "currents", _place, "currents.json");
        if (File.Exists(out_)) { Log($"[currents] {out_} exists; step already done"); return; }
        var adcp = NdbcStations.Nearest(NdbcCatalog(), _p.Anchor, new() { "adcp" }, 1, _p.AdcpRadiusKm);
        if (adcp.Count == 0) Skip("ndbc adcp", $"no NDBC station within {_p.AdcpRadiusKm:g} km of the anchor publishes realtime2 .adcp now");
        var r = new Currents().HarvestPlace(_ctx, _polite, _place, _p.Box, _p.CurrentRadiusKm, anchor: _p.Anchor, maxStations: _p.MaxCurrentStations, adcp: adcp.Count > 0 ? adcp[0].sid : null, field: field);
        Steps["currents"] = D(("files", r.Files.Select(f => new List<object?> { Rel(f.path), f.bytes }).ToList()),
            ("stations", r.Stations.Select(s => new List<object?> { s.id, s.bin }).ToList()), ("failed", r.Failed),
            ("adcp", adcp.Count > 0 ? D(("id", adcp[0].sid), ("km", Round(adcp[0].km, 1)), ("obs_unix", r.Adcp?.GetValueOrDefault("obs_unix"))) : null),
            ("field", field is not null));
    }

    void StepWaves()
    {
        var out_ = Path.Combine("data", "sea", _place, "seastate.json");
        if (File.Exists(out_)) { Log($"[waves] {out_} exists; step already done"); return; }
        var buoys = NdbcStations.Nearest(NdbcCatalog(), _p.Sea, new() { "data_spec", "spec" }, 2, 250.0);
        Log($"[waves] nearest NDBC wave stations to ({F(_p.Sea.lon)}, {F(_p.Sea.lat)}): [{string.Join(", ", buoys.Select(b => $"('{b.sid}', {F(Round(b.km, 1))})"))}]");
        var r = new Waves().HarvestPlace(_ctx, _polite, _place, _p.Sea, buoys.Select(b => b.sid).ToList());
        Steps["waves"] = D(("files", r.Files.Select(f => new List<object?> { Rel(f.path), f.bytes }).ToList()),
            ("node", new List<double> { r.Node.lat, r.Node.lon }), ("cycle", r.Cycle), ("cycle_unix", r.CycleUnix),
            ("buoys", buoys.Select(b => D(("id", b.sid), ("km_from_sea_point", Round(b.km, 1)), ("name", b.name), ("obs_unix", r.Buoys.GetValueOrDefault(b.sid)))).ToList()));
    }

    static string F(double v) => v.ToString("R", CultureInfo.InvariantCulture);

    Polite.S3Object? OfsVdatums(string ofs) => Ofs.Vdatums(_polite, ofs);

    (string ymd, int cyc, Dictionary<string, long> files, string prefix) OfsLatestCycle(string ofs) => Ofs.LatestCycle(_polite, ofs, NowUtc);

    Dictionary<string, object?>? StepOfs(bool planOnly = false)
    {
        var ofs = _p.Ofs;
        if (ofs is null)
        {
            Skip("nos ofs", "no NOS operational forecast system covers this place on the NODD bucket (noaa-nos-ofs-pds lists cbofs ciofs creofs dbofs glofs gomofs leofs lmhofs loofs lsofs necofs negofs ngofs ngofs2 nwgofs nyofs sfbofs sjrofs sscofs tbofs wcofs; OFS_Grid_Datum/ holds a secofs_vdatums.nc dated 2026-07-08, but there is no secofs output folder)");
            return null;
        }
        var (ymd, cyc, files, prefix) = OfsLatestCycle(ofs);
        var destDir = _ctx.Big("ofs", ofs, ymd);
        var plan = _polite.NewPlan($"{ofs.ToUpperInvariant()} nowcast t{cyc:00}z {ymd} (one cycle)");
        foreach (var (name, size) in files) plan.Add(name, $"https://{OfsBucket}.s3.amazonaws.com/{prefix}{name}", size, Path.Combine(destDir, name));
        var vd = OfsVdatums(ofs);
        if (vd is not null)
        {
            var vn = vd.Key[(vd.Key.LastIndexOf('/') + 1)..];
            plan.Add(vn, $"https://{OfsBucket}.s3.amazonaws.com/{vd.Key}", vd.Size, _ctx.Big("ofs", ofs, vn));
        }
        var total = Polite.Plan.Total(plan.Items);
        if (total > 1_000_000_000)
        {
            Log($"[ofs] one cycle is {total / 1e9:0.00} GB > 1 GB: the station product only");
            plan.Items = plan.Items.Where(i => i.Key.Contains("stations") || i.Key.Contains("vdatums")).ToList();
        }
        var (keep, cut) = plan.Fit();
        plan.Print(keep, cut);
        var step = Step("ofs") ?? new Dictionary<string, object?>();
        step["cycle"] = $"{ymd} t{cyc:00}z";
        step["plan"] = plan.Items.Select(i => i.ToJson()).ToList();
        step["later"] = cut.Select(i => i.Key).ToList();
        Steps["ofs"] = step;
        if (planOnly) return null;
        var raw = new List<object?>();
        foreach (var it in keep)
        {
            _polite.Download(it.Url, it.Dest, it.Size!.Value, note: $"{ofs} {_place}");
            raw.Add(new List<object?> { it.Dest, new FileInfo(it.Dest).Length, it.Url });
        }
        step["raw"] = raw;
        return D(("ymd", ymd), ("cyc", cyc), ("dir", destDir), ("raw", raw));
    }

    (Currents.PlaceField field, Dictionary<string, object?> declared) ConvertOfs(string ofs, string ofsDir, List<string> cycleFiles, Box box)
    {
        var fields = cycleFiles.Where(n => n.Contains(".fields.n")).OrderBy(n => n, StringComparer.Ordinal).ToList();
        return Ofs.Convert(ofs, Path.Combine(ofsDir, fields[^1]), box, 0.005, Log) ?? throw new InvalidOperationException($"{fields[^1]}: no rho point inside {box}");
    }

    void StepOfsField()
    {
        if (File.Exists(Path.Combine("data", "currents", _place, "currents.json"))) { Log($"[ofsfield] data/currents/{_place}/currents.json exists; step already done"); return; }
        var ofs = Step("ofs") ?? new();
        var raw = (ofs.GetValueOrDefault("raw") as List<object?>) ?? new();
        if (raw.Count == 0) throw new InvalidOperationException("run the ofs step first");
        var paths = raw.Select(r => (string)((List<object?>)r!)[0]!).ToList();
        var names = paths.Select(Path.GetFileName).Select(n => n!).ToList();
        var fieldDir = Path.GetDirectoryName(paths.First(p => p.Contains(".fields.")))!;
        var (field, declared) = ConvertOfs(_p.Ofs!, fieldDir, names, _p.Box);
        var vd = paths.FirstOrDefault(p => p.EndsWith("_vdatums.nc"));
        if (vd is not null) declared["vdatums"] = Ofs.VdatumsDeclared(vd);
        ofs["declared"] = declared;
        StepCurrents(field);
    }

    void StepOfsCheck(int nStations = 5)
    {
        var raw = ((List<object?>)Step("ofs")!["raw"]!).Select(r => (string)((List<object?>)r!)[0]!).Where(p => p.Contains(".fields.n")).OrderBy(p => p, StringComparer.Ordinal).ToList();
        var stations = ((List<object?>)Step("tides")!["stations"]!).Cast<Dictionary<string, object?>>().Take(nStations).ToList();
        var rows = new List<Dictionary<string, object?>>();
        foreach (var path in raw)
        {
            using var nc = new NetCdf4(path);
            var (t, _) = nc.Read("ocean_time");
            var m = Regex.Match(nc.Attr("ocean_time", "units") ?? "", @"seconds since (\d{4})-(\d{2})-(\d{2})");
            var when = new DateTime(int.Parse(m.Groups[1].Value), int.Parse(m.Groups[2].Value), int.Parse(m.Groups[3].Value), 0, 0, 0, DateTimeKind.Utc).AddSeconds(t[^1]);
            var (z, _) = nc.Read("zeta");
            var (lon, _) = nc.Read("lon_rho");
            var (lat, _) = nc.Read("lat_rho");
            var (mask, _) = nc.Read("mask_rho");
            var zz = z[^lon.Length..];
            var row = D(("valid_utc", IsoZ(when)), ("stations", new Dictionary<string, object?>()));
            foreach (var s in stations)
            {
                double slon = Convert.ToDouble(s["lon"]), slat = Convert.ToDouble(s["lat"]);
                var k = -1; var best = double.MaxValue;
                for (var i = 0; i < lon.Length; i++)
                {
                    if (!(mask[i] > 0.5)) continue;
                    var d = (lon[i] - slon) * (lon[i] - slon) + (lat[i] - slat) * (lat[i] - slat);
                    if (d < best) { best = d; k = i; }
                }
                var sid = (string)s["id"]!;
                var dat = TryLoad(Path.Combine("cache", "coops", sid, "datums_metric.json"));
                var dv = dat.Arr("datums").ToDictionary(d => d.Str("name") ?? "", d => d.Num("value"));
                var pred = TryLoad(Path.Combine("cache", "coops", sid, $"pred_{when.Year}_all.json"));
                var key = when.ToString("yyyy-MM-dd HH:mm", CultureInfo.InvariantCulture);
                var pv = pred.Arr("predictions").FirstOrDefault(x => x.Str("t") == key)?.Num("v");
                double? relMsl = pv is double p && dv.ContainsKey("MSL") && dv.ContainsKey("MLLW") ? Round(p - (dv["MSL"] - dv["MLLW"]), 3) : null;
                ((Dictionary<string, object?>)row["stations"]!)[sid] = D(("model_zeta_m", Round(zz[k], 3)), ("coops_prediction_vs_msl_m", relMsl));
            }
            rows.Add(row);
        }
        var diffs = rows.SelectMany(r => ((Dictionary<string, object?>)r["stations"]!).Values.Cast<Dictionary<string, object?>>())
            .Where(v => v["coops_prediction_vs_msl_m"] is double).Select(v => (double)v["model_zeta_m"]! - (double)v["coops_prediction_vs_msl_m"]!).ToList();
        var check = D(("what", "model zeta at the nearest wet rho point vs CO-OPS prediction (MLLW -> MSL via the station's datums.json) at each nowcast hour"), ("rows", rows),
            ("model_minus_prediction_m", diffs.Count > 0 ? D(("min", Round(diffs.Min(), 3)), ("max", Round(diffs.Max(), 3)), ("mean", Round(diffs.Average(), 3))) : null));
        Step("ofs")!["check"] = check;
        Log($"[ofscheck] model - prediction over {diffs.Count} station-hours: {Dump(check["model_minus_prediction_m"], 0)}");
    }

    void StepBathy(bool planOnly = false, long reserve = SafetyBytes)
    {
        var name = _p.GridName ?? _place;
        var r = new Bathy().HarvestPlace(_ctx, _polite, _place, _p.Box, near: _p.Anchor, planOnly: planOnly, reserve: reserve, gridBox: _p.GridBox, outName: name);
        var prev = Step("bathy") ?? new();
        Steps["bathy"] = D(("plan", r.Plan), ("later", r.Later),
            ("files", r.Files.Count > 0 ? r.Files.Select(f => (object?)new List<object?> { Rel(f.path), f.bytes }).ToList() : prev.GetValueOrDefault("files") ?? new List<object?>()),
            ("declared", r.Declared is not null ? r.Declared.ToDictionary(kv => kv.Key, kv => (object?)kv.Value) : prev.GetValueOrDefault("declared") ?? new Dictionary<string, object?>()));
    }

    static List<(string name, long csize, long usize, long off)> ZipDirectory(byte[] tail, long total)
    {
        var eocd = LastIndexOf(tail, "PK\x05\x06"u8.ToArray());
        if (eocd < 0) throw new InvalidOperationException("no end-of-central-directory record in the tail");
        int n = Bin.U16(tail, eocd + 10);
        long cdSize = Bin.U32(tail, eocd + 12), cdOff = Bin.U32(tail, eocd + 16);
        if (cdOff == 0xFFFFFFFF) throw new InvalidOperationException("zip64 archive (central directory past 4 GB)");
        var bse = total - tail.Length;
        if (cdOff < bse) throw new InvalidOperationException($"central directory ({cdSize} B) does not fit the tail read");
        var p = (int)(cdOff - bse);
        var out_ = new List<(string, long, long, long)>();
        for (var i = 0; i < n; i++)
        {
            if (!(tail[p] == 'P' && tail[p + 1] == 'K' && tail[p + 2] == 1 && tail[p + 3] == 2)) break;
            long csize = Bin.U32(tail, p + 20), usize = Bin.U32(tail, p + 24);
            int ln = Bin.U16(tail, p + 28), le = Bin.U16(tail, p + 30), lc = Bin.U16(tail, p + 32);
            long off = Bin.U32(tail, p + 42);
            out_.Add((System.Text.Encoding.GetEncoding(437).GetString(tail, p + 46, ln), csize, usize, off));
            p += 46 + ln + le + lc;
        }
        return out_;
    }

    static int LastIndexOf(byte[] hay, byte[] needle)
    {
        for (var i = hay.Length - needle.Length; i >= 0; i--)
        {
            var ok = true;
            for (var k = 0; k < needle.Length; k++) if (hay[i + k] != needle[k]) { ok = false; break; }
            if (ok) return i;
        }
        return -1;
    }

    Dictionary<string, object?>? StepSurvey()
    {
        _rec["skipped"] = ((_rec["skipped"] as List<object?>) ?? new()).Where(s => (string?)((Dictionary<string, object?>)s!)["source"] != "cusp shoreline").ToList();
        if (_place == "chesapeake") { Skip("cusp shoreline", "not requested for the Chesapeake; the covering archive in the listing is DE_MD_DC_VA_Inclusive_20250930.zip (100M)"); return null; }
        if (_place != "haulover") { Skip("cusp shoreline", "not requested for this place"); return null; }
        var dir = Path.Combine("cache", "places", "_cusp");
        Directory.CreateDirectory(dir);
        var lst = Directory.GetFiles(dir, "dist_shoreline_*").Select(Path.GetFileName).OrderBy(n => n, StringComparer.Ordinal).ToList();
        if (lst.Count == 0)
        {
            var listing = Path.Combine(dir, $"dist_shoreline_{NowUtc:yyyyMMddHH}.html");
            _polite.GetTextP("https://geodesy.noaa.gov/dist_shoreline/", listing);
            lst.Add(Path.GetFileName(listing));
        }
        var html = File.ReadAllText(Path.Combine(dir, lst[^1]!));
        var zips = new Dictionary<string, string>();
        foreach (Match m in Regex.Matches(html, "href=\"([^\"/]+\\.zip)\".*?<td[^>]*>\\s*([\\d.]+[KMG])\\s*</td>", RegexOptions.Singleline)) zips[m.Groups[1].Value] = m.Groups[2].Value;
        Log($"[survey] CUSP regional zips: {Dump(zips.ToDictionary(k => k.Key, k => (object?)k.Value), 0)}");
        var cands = zips.Keys.Where(z => z.ToLowerInvariant().StartsWith("southeast") || z.ToLowerInvariant().StartsWith("fif")).ToList();
        var inspected = new Dictionary<string, object?>();
        var report = D(("listing", Rel(Path.Combine(dir, lst[^1]!))), ("zips", zips.ToDictionary(k => k.Key, k => (object?)k.Value)), ("inspected", inspected));
        foreach (var zname in cands)
        {
            var url = $"https://geodesy.noaa.gov/dist_shoreline/{zname}";
            var (tail, total, _) = _polite.GetTail(url, 262144, Path.Combine(dir, $"{zname}.tail"));
            List<(string name, long csize, long usize, long off)> entries;
            try { entries = ZipDirectory(tail, total ?? tail.Length); }
            catch (InvalidOperationException e) { inspected[zname] = D(("bytes", total), ("error", e.Message)); continue; }
            inspected[zname] = D(("bytes", total), ("members", entries.Select(e => D(("name", e.name), ("compressed", e.csize), ("size", e.usize))).ToList()));
            Log($"[survey] {zname}: {(total ?? 0) / 1e6:0.0} MB, {entries.Count} members: [{string.Join(", ", entries.Take(12).Select(e => $"('{e.name}', {Round(e.csize / 1e6, 1)})"))}]");
        }
        Steps["survey"] = report;
        var se = inspected.GetValueOrDefault("Southeast_Caribbean.zip") as Dictionary<string, object?> ?? new();
        var members = (se.GetValueOrDefault("members") as List<Dictionary<string, object?>>) ?? new();
        var shp = members.Where(m => ((string)m["name"]!).ToLowerInvariant().EndsWith(".shp")).ToList();
        if (shp.Count > 0 && shp.Max(m => (long)m["compressed"]!) > _polite.Limits.BigBytes)
            Skip("cusp shoreline", $"not cheap: Florida's Atlantic shoreline is one {(long)shp[0]["compressed"]! / 1e6:0.0} MB deflate member (Southeast_Caribbean.shp) of the {Convert.ToDouble(se["bytes"]) / 1e6:0.0} MB regional zip, and a deflate member cannot be read in part; FIF_20260908.zip is per-state inland (22 states, no Florida). Read through the zips' central directories only (two suffix-range requests)");
        return report;
    }

    void StepTerms()
    {
        var out_ = new Dictionary<string, object?>();
        var before = new Dictionary<string, Dictionary<string, object?>>();
        if (_rec.GetValueOrDefault("terms") is Dictionary<string, object?> mine)
            foreach (var (k, v) in mine) if (v is Dictionary<string, object?> d) before[k] = d;
        var rdir = Path.Combine("out", "places");
        if (Directory.Exists(rdir))
            foreach (var fn in Directory.GetFiles(rdir, "*.record.json").OrderBy(f => f, StringComparer.Ordinal))
            {
                var other = TryLoad(fn);
                if (other?["terms"] is JsonObject terms)
                    foreach (var (k, v) in terms)
                        if (v is JsonObject o && o.ContainsKey("error") && !before.ContainsKey(k)) before[k] = (Dictionary<string, object?>)Plain(o)!;
            }
        foreach (var (key, (url, patterns)) in Terms)
        {
            var cache = Path.Combine("cache", "places", "_terms", Regex.Replace(url, "[^A-Za-z0-9]+", "_") + ".html");
            if (before.TryGetValue(key, out var b) && b.ContainsKey("error") && !File.Exists(cache))
            {
                out_[key] = b;
                Log($"[terms] {key}: refused earlier ({b["error"]}); not asked again");
                continue;
            }
            string html;
            try { (html, _) = _polite.GetTextP(url, cache); }
            catch (Polite.StopException) { throw; }
            catch (Exception e) { out_[key] = D(("url", url), ("error", e.Message[..Math.Min(200, e.Message.Length)])); continue; }
            html = Regex.Replace(html, @"<(script|style)[^>]*>.*?</\1>", " ", RegexOptions.Singleline | RegexOptions.IgnoreCase);
            string? quote = null;
            var blocks = Regex.Split(html, @"<(?:p|li|td|th|div|h[1-6]|dd|dt|br|section|article)\b[^>]*>", RegexOptions.IgnoreCase);
            foreach (var pat in patterns)
            {
                foreach (var blk in blocks)
                {
                    var text = Regex.Replace(blk, "<[^>]+>", " ");
                    text = Regex.Replace(text, "&nbsp;|&#160;", " ").Replace("&amp;", "&");
                    text = Regex.Replace(text, @"\s+", " ").Trim();
                    if (!Regex.IsMatch(text, pat, RegexOptions.IgnoreCase)) continue;
                    foreach (var sent in Regex.Split(text, @"(?<=[.!?])\s+(?=[A-Z])"))
                        if (Regex.IsMatch(sent, pat, RegexOptions.IgnoreCase)) { quote = sent.Trim(); break; }
                    if (quote is not null) break;
                }
                if (quote is not null) break;
            }
            out_[key] = D(("url", url), ("quote", quote));
            Log($"[terms] {key}: {(quote is null ? "None" : "'" + quote + "'")}");
        }
        _rec["terms"] = out_;
    }

    void StepPlan()
    {
        var small = new List<Dictionary<string, object?>>();
        var (stList, subList) = new Tides().UsePolite(_ctx, _polite).DiscoverPlace(_p.Box, _p.TideRadiusKm, Path.Combine("cache", "coops"), _p.Anchor);
        int stations = stList.Count, sub = subList.Count;
        small.Add(D(("what", $"CO-OPS tide stations: meta + harcon + datums + one year of hourly predictions each ({stations} harmonic; {sub} subordinate listed only)"), ("requests", 4 * stations), ("bytes", stations * 375_000L)));
        var picks = new Currents().UsePolite(_ctx, _polite).DiscoverPlace(_p.Box, _p.CurrentRadiusKm, _p.Anchor, _p.MaxCurrentStations, Path.Combine("cache", "coops")).Count;
        small.Add(D(("what", $"CO-OPS current predictions, 45 days MAX_SLACK in two requests ({picks} stations nearest the anchor)"), ("requests", 2 * picks), ("bytes", picks * 2 * 25_000L)));
        small.Add(D(("what", "GFS-Wave point via the NOMADS grib filter: one cycle probe + 13 forecast hours (f000..f048 by 4)"), ("requests", 14), ("bytes", 14 * 12_000L)));
        small.Add(D(("what", "NDBC realtime2: listing (reused <6 h) + two wave buoys (.txt, .data_spec) + an ADCP buoy"), ("requests", 6), ("bytes", 600_000L + 5 * 60_000L)));
        small.Add(D(("what", "each source's terms page (7 pages)"), ("requests", 7), ("bytes", 7 * 80_000L)));
        Log($"PLAN {_place}: small products");
        foreach (var s in small) Log($"  {(int)s["requests"]!,5} req  {(long)s["bytes"]! / 1e6,8:0.00} MB  {s["what"]}");
        var smallReq = small.Sum(s => (int)s["requests"]!);
        var smallBytes = small.Sum(s => (long)s["bytes"]!);
        Dictionary<string, object?>? ofsPlan = null;
        if (_p.Ofs is not null)
        {
            var (ymd, cyc, files, _) = OfsLatestCycle(_p.Ofs);
            var vd = OfsVdatums(_p.Ofs);
            if (vd is not null) files[vd.Key[(vd.Key.LastIndexOf('/') + 1)..]] = vd.Size;
            ofsPlan = D(("cycle", $"{ymd} t{cyc:00}z"), ("files", files.ToDictionary(k => k.Key, k => (object?)k.Value)), ("bytes", files.Values.Sum()));
            Log($"PLAN {_place}: {_p.Ofs.ToUpperInvariant()} nowcast t{cyc:00}z {ymd} + its vertical-datum grid: {files.Count} files, {files.Values.Sum() / 1e6:0.0} MB");
            foreach (var (n, s) in files) Log($"        {n,-48} {s / 1e6,8:0.0} MB");
        }
        var bathy = new Bathy();
        var cacheDir = bathy.BigCacheFor(_ctx);
        var index = Bathy.ReadTileIndex(_polite.Get(Bathy.IndexUrl, Path.Combine(cacheDir, Bathy.IndexName)).data);
        var (_, tplan) = bathy.PlanPlace(_place, _p.Box, _p.Anchor, cacheDir, index, _polite);
        var reserve = smallBytes + (ofsPlan is not null ? (long)ofsPlan["bytes"]! : 0) + SafetyBytes;
        var (keep, cut) = tplan.Fit(reserve);
        tplan.Print(keep, cut);
        var total = reserve + Polite.Plan.Total(tplan.Items);
        var left = _polite.Remaining();
        Log($"PLAN {_place} TOTAL: {total / 1e9:0.000} GB and ~{smallReq + tplan.Items.Count + 10} requests for everything; the budget has {left.bytes / 1e9:0.000} GB and {left.requests} requests left, so {cut.Count} tile(s) ({Polite.Plan.Total(cut) / 1e9:0.000} GB) wait for a later run");
        _rec["plan"] = D(("made_utc", NowIso()), ("small", small), ("ofs", ofsPlan), ("tiles", tplan.Items.Select(i => i.ToJson()).ToList()),
            ("tiles_later", cut.Select(i => i.Key).ToList()), ("total_bytes", total), ("budget_left_bytes", left.bytes), ("budget_left_requests", left.requests));
    }

    // ------------------------------------------------------------------ manifest

    static string CudemMetaUrl => "https://noaa-nos-coastal-lidar-pds.s3.amazonaws.com/dem/NCEI_ninth_Topobathy_2014_8483/ncei_nintharcsec_dem_m8483_met.xml";
    static readonly Dictionary<string, string> GribUnits = new()
    {
        ["HTSGW"] = "m", ["PERPW"] = "s", ["DIRPW"] = "degree true (from)", ["WVHGT"] = "m", ["WVPER"] = "s", ["WVDIR"] = "degree true (from)",
        ["SWELL"] = "m", ["SWPER"] = "s", ["SWDIR"] = "degree true (from)", ["WIND"] = "m s-1", ["WDIR"] = "degree true (from)",
    };

    List<Dictionary<string, object?>> WrittenFiles(List<JsonNode> entries)
    {
        var t0 = entries.Count > 0 ? entries.Min(e => e.Num("t_end")) - 120 : 0;
        var roots = new[] { "coops", "currents", "ndbc", "gfswave", "places" }.Select(d => Path.Combine("cache", d)).ToList();
        roots.Add(_ctx.Big("cudem"));
        roots.Add(_ctx.Big("ofs"));
        var disk = new List<(string path, long size, double mtime)>();
        foreach (var r in roots)
        {
            if (!Directory.Exists(r)) continue;
            foreach (var pth in Directory.EnumerateFiles(r, "*", SearchOption.AllDirectories))
            {
                var fi = new FileInfo(pth);
                var mt = Unix(fi.LastWriteTimeUtc);
                if (mt >= t0) disk.Add((pth, fi.Length, mt));
            }
        }
        var used = new HashSet<string>();
        var out_ = new List<Dictionary<string, object?>>();
        foreach (var e in entries)
        {
            var bytes = e.Num("bytes");
            if (bytes == 0 || (e.Str("error") is not null) || e.Str("method") == "HEAD") continue;
            var dest = e.Str("dest");
            if (dest is null)
            {
                (string pth, double dt)? best = null;
                foreach (var (pth, size, mt) in disk)
                {
                    if (used.Contains(pth) || size != (long)bytes) continue;
                    var dt = Math.Abs(mt - e.Num("t_end"));
                    if (dt < 10 && (best is null || dt < best.Value.dt)) best = (pth, dt);
                }
                dest = best?.pth;
            }
            if (dest is not null) used.Add(dest);
            out_.Add(D(("path", dest is not null ? Rel(dest) : null), ("bytes", (long)bytes), ("url", e.Str("url")), ("fetched_utc", e.Str("t")), ("range", e.Str("range")), ("place", e.Str("place"))));
        }
        return out_;
    }

    static Dictionary<string, object?> TideDeclared(string sid)
    {
        var d = Path.Combine("cache", "coops", sid);
        var dat = TryLoad(Path.Combine(d, "datums_metric.json"));
        var har = TryLoad(Path.Combine(d, "harcon_metric.json"));
        var vals = dat.Arr("datums").ToDictionary(x => x.Str("name") ?? "", x => x.NumOrNull("value"));
        return D(("harcon_units", har.Str("units")), ("datums_units", dat.Str("units")), ("datum_epoch", dat.Str("epoch")), ("orthometric_datum", dat.Str("OrthometricDatum")),
            ("datums_relative_to", "STND (station datum) as published"), ("MLLW", vals.GetValueOrDefault("MLLW")), ("MSL", vals.GetValueOrDefault("MSL")), ("NAVD88", vals.GetValueOrDefault("NAVD88")));
    }

    static string? S3Modified(string keyTail)
    {
        var d = Path.Combine("cache", "places", "_s3");
        if (!Directory.Exists(d)) return null;
        foreach (var fn in Directory.GetFiles(d).OrderBy(f => f, StringComparer.Ordinal))
        {
            var xml = File.ReadAllText(fn);
            var m = Regex.Match(xml, "<Key>[^<]*" + Regex.Escape(keyTail) + "</Key><LastModified>([^<]+)</LastModified>");
            if (m.Success) return m.Groups[1].Value;
        }
        return null;
    }

    Dictionary<string, object?> RawDeclared(string path)
    {
        var name = Path.GetFileName(path);
        if (name.EndsWith(".tif"))
            return D(("datum", "vertical NAVD88 (dataset metadata), horizontal NAD83 (GeoKeys)"), ("units", "m"),
                ("valid", $"static: CUDEM tile version {name[(name.LastIndexOf('_') + 1)..^4]}, bucket LastModified {S3Modified(name) ?? "None"}"));
        if (name.Contains(".fields.n"))
        {
            var ofs = Step("ofs") ?? new();
            var rows = ((ofs.GetValueOrDefault("check") as Dictionary<string, object?>)?.GetValueOrDefault("rows") as List<object?>) ?? new();
            var fields = ((ofs.GetValueOrDefault("raw") as List<object?>) ?? new()).Select(r => (string)((List<object?>)r!)[0]!).Where(f => f.Contains(".fields.n")).OrderBy(f => f, StringComparer.Ordinal).ToList();
            string? valid = null;
            for (var i = 0; i < fields.Count && i < rows.Count; i++)
                if (Path.GetFileName(fields[i]) == name) valid = (string?)((Dictionary<string, object?>)rows[i]!)["valid_utc"];
            return D(("datum", "zeta 'free-surface' (no datum attribute; the model's datum grid relates NAVD88/MLLW/... to mean sea level)"),
                ("units", "zeta 'meter', u/v 'meter second-1', angle 'radians'"), ("valid", $"{valid ?? "None"} (ocean_time, seconds since 2016-01-01 00:00:00)"));
        }
        if (name.Contains(".stations.nowcast"))
        {
            string valid;
            try
            {
                using var nc = new NetCdf4(path);
                var (t, _) = nc.Read("ocean_time");
                var t0 = new DateTime(2016, 1, 1, 0, 0, 0, DateTimeKind.Utc);
                valid = $"{IsoZ(t0.AddSeconds(t[0]))}..{IsoZ(t0.AddSeconds(t[^1]))}, {t.Length} times ({nc.Attr("ocean_time", "units")})";
            }
            catch (Exception e) { valid = $"not read ({e.Message})"; }
            return D(("datum", "as the fields files"), ("units", "as the fields files"), ("valid", valid));
        }
        if (name.EndsWith("_vdatums.nc"))
            return D(("datum", "differences to mean sea level (navd88tomsl, mhhwtomsl, mhwtomsl, mlwtomsl, mllwtomsl), NAD83 positions"), ("units", "meters"),
                ("valid", $"static: 'File created: 09-Aug-2010' (its creation_date attribute); bucket LastModified {S3Modified(name) ?? "None"}"));
        if (name.EndsWith(".zip") || name.EndsWith(".xml")) return D(("valid", $"bucket LastModified {S3Modified(name) ?? "None"}"));
        return new();
    }

    Dictionary<string, object?> BuildManifest()
    {
        var entries = _polite.Entries();
        var mine = entries.Where(e => e.Str("place") == _place).ToList();
        var shared = entries.Where(e => e.Str("place") == "discovery").ToList();
        var written = WrittenFiles(mine.Concat(shared).ToList());
        var byUrl = new Dictionary<string, Dictionary<string, object?>>();
        foreach (var w in written) byUrl.TryAdd((string)w["url"]!, w);
        var outputs = new List<Dictionary<string, object?>>();

        var b = Step("bathy");
        var bFiles = (b?.GetValueOrDefault("files") as List<object?>) ?? new();
        if (b is not null && bFiles.Count > 0)
        {
            var meta = TryLoad(Path.Combine("data", "bathy", $"{_p.GridName ?? _place}.json"));
            var tiles = meta.Arr("tiles").ToList();
            foreach (var f in bFiles)
            {
                var fl = (List<object?>)f!;
                outputs.Add(D(("path", fl[0]), ("bytes", fl[1]), ("kind", "CUDEM topobathy grid (BathyModel format)"),
                    ("source_url", tiles.Select(t => (object?)t.Str("url")).Append(Bathy.IndexUrl).ToList()),
                    ("datum", "vertical NAVD88 height, metres -- declared by the dataset metadata (ncei_nintharcsec_dem_m8483_met.xml: 'The DEMs are referenced vertically to the North American Vertical Datum of 1988. The vertical units of the DEMs are meters.'); horizontal NAD83 geographic (GeographicType 4269 in each tile's GeoKeys; index srs " +
                              $"[{string.Join(", ", tiles.Select(t => "'" + t.Str("srs_index") + "'").Distinct().OrderBy(x => x, StringComparer.Ordinal))}]); the tiles carry no vertical GeoKey"),
                    ("units", "m (GDAL UNITTYPE 'metre' in each tile)"),
                    ("valid", $"static: CUDEM tile versions [{string.Join(", ", tiles.Select(t => { var n = t.Str("name")!; return "'" + n[(n.LastIndexOf('_') + 1)..^4] + "'"; }).Distinct().OrderBy(x => x, StringComparer.Ordinal))}]"),
                    ("grid", new[] { "nx", "ny", "lon0", "lat1", "dlon", "dlat", "step", "row0", "nodata" }.ToDictionary(k => k, k => Plain(meta?[k])))));
            }
        }

        var t = Step("tides");
        if (t is not null)
        {
            var stations = ((List<object?>)t["stations"]!).Cast<Dictionary<string, object?>>().ToList();
            var decl = stations.ToDictionary(s => (string)s["id"]!, s => TideDeclared((string)s["id"]!));
            var year = Convert.ToInt32(t["year"]);
            foreach (var f in (List<object?>)t["files"]!)
            {
                var fl = (List<object?>)f!;
                var pth = (string)fl[0]!;
                var sid = Regex.Match(pth, @"official_(\w+)_\d{4}\.f32");
                var ids = sid.Success ? new List<string> { sid.Groups[1].Value } : stations.Select(s => (string)s["id"]!).ToList();
                outputs.Add(D(("path", pth), ("bytes", fl[1]),
                    ("kind", sid.Success ? "official hourly predictions, float32 (TideModel sidecar)" : "tide constituent fits (TideModel format: stations.json)"),
                    ("source_url", byUrl.Keys.Where(u => ids.Any(i => u.Contains($"/stations/{i}") || u.Contains($"station={i}&"))).OrderBy(u => u, StringComparer.Ordinal).ToList()),
                    ("datum", "heights in metres above each station's MLLW (requests datum=MLLW, units=metric); mllw_minus_navd_m from each station's datums.json where it publishes NAVD88"),
                    ("declared", ids.ToDictionary(i => i, i => (object?)decl[i])), ("units", "m"),
                    ("valid", $"fit year {year} (epoch_unix {Convert.ToDouble(t["epoch_unix"]):0} = {year}-01-01T00:00Z); hourly predictions {year}-01-01..{year}-12-31 UTC")));
            }
        }

        var c = Step("currents");
        if (c is not null)
        {
            var cj = TryLoad(Path.Combine("data", "currents", _place, "currents.json"));
            var units = new HashSet<string>();
            foreach (var w in written)
                if (((string)w["url"]!).Contains("currents_predictions") && w["path"] is string wp && TryLoad(wp).Obj("current_predictions").Str("units") is string un) units.Add(un);
            var ev = cj.Arr("tidal_stations").SelectMany(s => s.Arr("events")).Select(e => e.Num("t")).ToList();
            var ofsRaw = ((Step("ofs")?.GetValueOrDefault("raw") as List<object?>) ?? new()).Select(r => (string)((List<object?>)r!)[2]!).Where(u => u.Contains(".fields.")).ToList();
            foreach (var f in (List<object?>)c["files"]!)
            {
                var fl = (List<object?>)f!;
                var pth = (string)fl[0]!;
                var isField = pth.EndsWith(".f32");
                outputs.Add(D(("path", pth), ("bytes", fl[1]),
                    ("kind", isField ? "model surface current field, float32 u then v (CurrentModel field)" : "tidal current clocks + ADCP (CurrentModel format)"),
                    ("source_url", isField ? ofsRaw.TakeLast(1).ToList() : byUrl.Keys.Where(u => u.Contains("currents_predictions") || u.EndsWith(".adcp")).OrderBy(u => u, StringComparer.Ordinal).ToList()),
                    ("datum", isField ? "depth: the model's top sigma level (surface)" : "CO-OPS declares units " + string.Join(", ", units.OrderBy(x => x, StringComparer.Ordinal)) + "; written as m/s signed (flood +), directions degT"),
                    ("units", isField ? "m s-1 (east, north)" : "m s-1, degT"),
                    ("valid", isField ? cj.Obj("field").Str("valid_utc") : ev.Count > 0 ? $"MAX_SLACK events {FromUnix(ev.Min()):yyyy-MM-dd'T'HH:mm'Z'}..{FromUnix(ev.Max()):yyyy-MM-dd'T'HH:mm'Z'}" : null),
                    ("adcp", c.GetValueOrDefault("adcp"))));
            }
        }

        var w_ = Step("waves");
        if (w_ is not null)
        {
            var buoys = ((List<object?>)w_["buoys"]!).Cast<Dictionary<string, object?>>().ToList();
            var node = (List<object?>)w_["node"]!;
            foreach (var f in (List<object?>)w_["files"]!)
            {
                var fl = (List<object?>)f!;
                var unitsAll = GribUnits.ToDictionary(k => k.Key, k => (object?)k.Value);
                unitsAll["ndbc"] = "per realtime2 header row: WSPD m/s, WVHT m, DPD/APD sec, MWD degT";
                outputs.Add(D(("path", fl[0]), ("bytes", fl[1]), ("kind", "GFS-Wave point forecast + NDBC buoys (SeaState format)"),
                    ("source_url", byUrl.Keys.Where(u => u.Contains("nomads") || u.Contains("realtime2/") && (u.EndsWith(".txt") || u.EndsWith(".data_spec"))).OrderBy(u => u, StringComparer.Ordinal).ToList()),
                    ("datum", $"sea state (no vertical datum); GFS-Wave node ({F(Convert.ToDouble(node[0]))}, {F(Convert.ToDouble(node[1]))}) nearest the seaward point ({F(_p.Sea.lon)}, {F(_p.Sea.lat)})"),
                    ("units", unitsAll),
                    ("valid", $"{w_["cycle"]} f000..f048 every 4 h; buoys observed at " + string.Join(", ", buoys.Where(bb => bb["obs_unix"] is double).Select(bb => $"{bb["id"]} {FromUnix((double)bb["obs_unix"]!):yyyy-MM-dd'T'HH:mm'Z'}")))));
            }
        }

        var raw = new List<Dictionary<string, object?>>();
        var tilesMeta = new Dictionary<string, Dictionary<string, object?>>(StringComparer.OrdinalIgnoreCase);
        foreach (var it in ((b?.GetValueOrDefault("plan") as List<object?>) ?? new()).Cast<Dictionary<string, object?>>())
            if (it["dest"] is string dest && File.Exists(dest)) tilesMeta[Path.GetFullPath(dest)] = it;
        foreach (var wf in written)
        {
            if (wf["path"] is not string p || !Regex.IsMatch(p, "^[A-Za-z]:")) continue;
            var entry = new Dictionary<string, object?>(wf);
            if (tilesMeta.TryGetValue(Path.GetFullPath(p), out var it))
            {
                entry["declared"] = ((b?.GetValueOrDefault("declared") as Dictionary<string, object?>) ?? new()).GetValueOrDefault((string)it["key"]!);
                entry["km_from_anchor"] = it.GetValueOrDefault("km");
            }
            foreach (var (k, v) in RawDeclared(p)) entry[k] = v;
            raw.Add(entry);
        }
        var ofs = Step("ofs");
        var cache = written.Where(w => w["path"] is string p && !Regex.IsMatch(p, "^[A-Za-z]:")).ToList();
        var later = new List<Dictionary<string, object?>>();
        if (b is not null)
        {
            var lk = ((b.GetValueOrDefault("later") as List<object?>) ?? new()).Select(x => (string)x!).ToHashSet();
            later = ((b.GetValueOrDefault("plan") as List<object?>) ?? new()).Cast<Dictionary<string, object?>>().Where(i => lk.Contains((string)i["key"]!)).ToList();
        }
        else if (_rec.GetValueOrDefault("plan") is Dictionary<string, object?> pl)
        {
            var lk = ((pl["tiles_later"] as List<object?>) ?? new()).Select(x => (string)x!).ToHashSet();
            later = ((pl["tiles"] as List<object?>) ?? new()).Cast<Dictionary<string, object?>>().Where(i => lk.Contains((string)i["key"]!)).ToList();
        }
        var reqs = new Dictionary<string, Dictionary<string, long>>();
        foreach (var e in mine)
        {
            var h = e.Str("host") ?? "?";
            if (!reqs.TryGetValue(h, out var r)) reqs[h] = r = new() { ["requests"] = 0, ["bytes"] = 0 };
            r["requests"]++;
            r["bytes"] += (long)e.Num("bytes");
        }
        var manifest = D(
            ("place", _place), ("generated_utc", NowIso()),
            ("about", "Every file the place harvest wrote (outputs under data/, raw downloads on the big-data drive, raw responses under cache/), with source URL, size, the datum and units the source declares, and the instant it is valid for; what each source says about itself; what was skipped and why. Written once by harvester/harvest_place.py --final."),
            ("box", _p.Box.ToJson()), ("box_why", _p.BoxWhy), ("anchor", LL(_p.Anchor)), ("sea_point", LL(_p.Sea)),
            ("outputs", outputs), ("raw_downloads", raw), ("cache_files", cache),
            ("ofs", ofs is not null ? ofs.Where(kv => kv.Key != "plan").ToDictionary(kv => kv.Key, kv => kv.Value) : null),
            ("sources", SourcesBlock()), ("skipped", _rec.GetValueOrDefault("skipped") ?? new List<object?>()), ("surprises", Surprises),
            ("requests_this_place", reqs.ToDictionary(k => k.Key, k => (object?)k.Value)),
            ("requests_shared_discovery", D(("requests", shared.Count), ("bytes", shared.Sum(e => (long)e.Num("bytes"))))),
            ("ledger", Rel(_polite.LedgerPath)));
        if (later.Count > 0 || _rec.ContainsKey("plan"))
        {
            var pl = (_rec.GetValueOrDefault("plan") as Dictionary<string, object?>) ?? new();
            manifest["plan"] = D(("made_utc", pl.GetValueOrDefault("made_utc")), ("total_bytes_everything", pl.GetValueOrDefault("total_bytes")),
                ("small_products", pl.GetValueOrDefault("small")), ("ofs", pl.GetValueOrDefault("ofs")),
                ("tiles_fetched", ((b?.GetValueOrDefault("plan") as List<object?>) ?? new()).Cast<Dictionary<string, object?>>().Where(i => i["dest"] is string d && File.Exists(d))
                    .Select(i => D(("name", i["key"]), ("bytes", i["size"]), ("km", i.GetValueOrDefault("km")), ("path", Rel((string)i["dest"]!)))).ToList()),
                ("tiles_remaining", later.Select(i => D(("name", i["key"]), ("bytes", i["size"]), ("km", i.GetValueOrDefault("km")), ("url", i["url"]), ("dest", i["dest"]))).ToList()),
                ("tiles_remaining_bytes", later.Sum(i => Convert.ToInt64(i["size"] ?? 0))),
                ("continue_with", $"harvest place --place {_place} --steps bathy,manifest --final, with GAGAME_LEDGER naming a fresh ledger (a fresh ledger is a fresh budget): tiles on disk are never requested again, the remaining ones are taken nearest-first from the anchor in the order listed here, and the run ADDS data/places/{_place}.<UTC stamp>.json beside this manifest"));
        }
        return manifest;
    }

    static readonly List<string> Surprises =
    [
        "CUDEM Florida tiles spell the name separator 'X' (ncei19_n26X00_w080X25_2018v1.tif); the New England ones use 'x'. harvest_bathy's name parser is now case-insensitive.",
        "CUDEM tiles declare only their horizontal CRS (GeoKey GeographicType 4269, NAD83); the vertical datum (NAVD88) is declared by the dataset metadata XML, not by the tile. The index gives srs EPSG:4269 for FL/chesapeake_bay tiles but EPSG:5498 (NAD83 + NAVD88 height) for the northeast_sandy ones.",
        "The CUDEM tile index (tileindex_NCEI_ninth_Topobathy_2014.zip) points every tile at the NODD bucket noaa-nos-coastal-lidar-pds, not at coast.noaa.gov; the old per-region directory listing did the same.",
        "GFS-Wave on the NODD bucket is GRIB2 template 5.40 (JPEG 2000) for every gridded product (checked: atlocn.0p16, global.0p25), which the stdlib-only grib2.py cannot decode; the text bulletins there (bulls.tHHz/gfswave.<id>.bull/.cbull) are per output point, carry no wind, and their two flavours disagree on direction by 180 degrees for the same swell (44099, 18z: .bull 254, .cbull 074; the buoy itself reported MWD 75 'from' at 23z, so the .cbull is 'from' and the .bull 'toward'). Buoy 41122 (Hollywood, 13 km from the Haulover sea point) is not a GFS-Wave output point.",
        "NOS OFS output on the NODD bucket is netCDF-4 (HDF5: superblock 0, the root group's links and attributes in DENSE storage -- fractal heap + v2 B-tree -- variables chunked but unfiltered); only OFS_Grid_Datum/*_vdatums.nc is classic netCDF-3. CBOFS zeta declares units 'meter' and long_name 'free-surface' but no datum; the datum grid gives NAVD88, MHHW, MHW, MLW, MLLW each 'minus mean sea level' in metres, is dated 2010 and names its model 'DRBM_3D' under the title 'Chesapeake Bay and River Model 2' (a model name that is not CBOFS's). The bucket also holds a datum grid for 'secofs' (2026-07-08) but no secofs output folder.",
        "CO-OPS current stations list one entry per depth BIN (types H, S and W per bin; W = weak and variable, no predictions), and multi-bin stations need &bin= on the request; the Haulover-area subordinate TIDE stations all reference 8723178 (Government Cut), not the harmonic stations beside them.",
        "NDBC 41122 (Hollywood Beach, a CDIP Waverider) publishes .adcp; the NDBC table lists 44010, 44071 and 44096 at the Chesapeake mouth but none of them publishes to realtime2 now.",
        "CUSP's Southeast_Caribbean.zip holds Florida's Atlantic shoreline as ONE 246.8 MB deflate member; FIF_20260908.zip is per-state inland water (22 states, no Florida).",
        "CO-OPS answered a whole year of hourly predictions in ONE request (8760 values), where harvest_tides had always asked month by month; a place asks once per station.",
        "www.noaa.gov/disclaimer and www.ndbc.noaa.gov/disclaimer.shtml answered HTTP 403 to this client (not retried); NDBC's realtime2 data files answered normally.",
    ];

    Dictionary<string, object?> SourcesBlock()
    {
        var terms = (_rec.GetValueOrDefault("terms") as Dictionary<string, object?>) ?? new();
        Dictionary<string, object?> Q(string key)
        {
            var t = (terms.GetValueOrDefault(key) as Dictionary<string, object?>) ?? new();
            var quote = t.GetValueOrDefault("quote") as string;
            if (quote is not null && quote.Contains(" so long as ")) quote = quote[..quote.IndexOf(" so long as ", StringComparison.Ordinal)] + " [...]";
            return D(("quote", quote), ("url", t.GetValueOrDefault("url")), ("error", t.GetValueOrDefault("error")));
        }
        var cudemMeta = _ctx.Big("cudem", "ncei_nintharcsec_dem_m8483_met.xml");
        var meta = File.Exists(cudemMeta) ? File.ReadAllText(cudemMeta) : "";
        var cite = Regex.Match(meta, @"Cite as:\s*([^<]+)");
        var out_ = D(
            ("public_domain_where_stated", new List<object?>
            {
                D(("url", "https://tidesandcurrents.noaa.gov/disclaimers.html"), ("quote", Q("coops")["quote"])),
                D(("url", "https://www.weather.gov/disclaimer"), ("quote", Q("nws")["quote"])),
                D(("url", CudemMetaUrl), ("quote", "Produced by the NOAA National Centers for Environmental Information. Not subject to copyright protection within the United States.")),
                D(("url", "https://www.noaa.gov/disclaimer"), ("note", Q("noaa")["error"])),
            }),
            ("cudem", D(("what", "NOAA NCEI CUDEM 1/9 arc-second topobathy tiles (dataset 8483)"), ("hosts", new List<string> { "noaa-nos-coastal-lidar-pds.s3.amazonaws.com" }),
                ("terms", new List<object?> { D(("url", CudemMetaUrl), ("quote", "Not subject to copyright protection within the United States.")), Q("nodd_lidar") }),
                ("may_keep", "yes: public domain (no copyright in the US) and NODD 'open to the public and can be used as desired'; 'Not to be used for navigation.'"),
                ("cite", cite.Success ? cite.Groups[1].Value.Trim() : null),
                ("cite_nodd", "NOAA Coastal Lidar Data was accessed on DATE from https://registry.opendata.aws/noaa-coastal-lidar."))),
            ("coops", D(("what", "NOAA CO-OPS station metadata, harmonic constituents, datums, predictions"), ("hosts", new List<string> { "api.tidesandcurrents.noaa.gov" }),
                ("terms", new List<object?> { Q("coops") }), ("may_keep", "yes: public domain per the page quoted"),
                ("cite", "NOS requests that attribution be given whenever NOS material is reproduced and re-disseminated (same page): 'NOAA Tides and Currents (CO-OPS), station <id>'."))),
            ("ndbc", D(("what", "NOAA NDBC realtime2 observations (met/wave summary, spectra, ADCP) and the station table"), ("hosts", new List<string> { "www.ndbc.noaa.gov" }),
                ("terms", new List<object?> { Q("ndbc"), Q("nws") }),
                ("may_keep", "yes: NDBC is part of NOAA's National Weather Service, whose disclaimer (quoted) puts its information in the public domain; NDBC's own disclaimer page answered 403"),
                ("cite", "NOAA National Data Buoy Center, station <id> (the NWS page asks that the information not be claimed as your own, nor used to imply endorsement)"))),
            ("gfswave", D(("what", "NCEP GFS-Wave (WAVEWATCH III) gridded forecast, subset by the NOMADS grib filter; the S3 mirror (noaa-gfs-bdp-pds) was used only to list the cycle, read two .idx files, two 512-byte message heads and one bulletin pair"),
                ("hosts", new List<string> { "nomads.ncep.noaa.gov", "noaa-gfs-bdp-pds.s3.amazonaws.com" }), ("terms", new List<object?> { Q("nws"), Q("nodd_gfs") }),
                ("may_keep", "yes: NWS public domain; NODD 'open to the public and can be used as desired'"),
                ("cite", "NOAA Global Forecast System (GFS) was accessed on DATE from https://registry.opendata.aws/noaa-gfs-bdp-pds (the registry's form)"))));
        if (Step("ofs") is not null)
            out_["cbofs"] = D(("what", "NOAA NOS Chesapeake Bay Operational Forecast System (ROMS) nowcast fields, station nowcast and the vertical-datum grid"),
                ("hosts", new List<string> { "noaa-nos-ofs-pds.s3.amazonaws.com" }), ("terms", new List<object?> { Q("nodd_ofs") }),
                ("may_keep", "yes: NODD 'open to the public and can be used as desired'"),
                ("cite", "NOAA Operational Forecast System (OFS) was accessed on DATE from https://registry.opendata.aws/noaa-ofs (the registry's form)"));
        if (_place == "haulover")
            out_["cusp"] = D(("what", "NGS Continually Updated Shoreline Product (not used: skipped)"), ("hosts", new List<string> { "geodesy.noaa.gov" }),
                ("terms", "NGS is part of NOAA: public domain per the pages quoted above; the directory listing carries no terms of its own"), ("may_keep", "yes"), ("cite", null));
        return out_;
    }

    void StepManifest(bool final)
    {
        var m = BuildManifest();
        var draft = Path.Combine("out", "places", $"{_place}.manifest.draft.json");
        Save(draft, m);
        Log($"[manifest] draft {draft}: {((List<Dictionary<string, object?>>)m["outputs"]!).Count} outputs, {((List<Dictionary<string, object?>>)m["raw_downloads"]!).Count} raw downloads, {((List<Dictionary<string, object?>>)m["cache_files"]!).Count} cache files");
        if (!final) return;
        var path = Path.Combine("data", "places", $"{_place}.json");
        if (File.Exists(path)) path = Path.Combine("data", "places", $"{_place}.{NowUtc:yyyyMMdd'T'HHmm'Z'}.json");
        Save(path, m, exclusive: true);
        Log($"[manifest] wrote {path}");
    }

    // ------------------------------------------------------------------ main

    public int Run(HarvestContext ctx, string[] argv)
    {
        _ctx = ctx;
        var a = new Args(argv);
        _place = a.Opt("--place") ?? throw new HarvestStop("harvest place --place NAME [--box lon0,lat0,lon1,lat1 --anchor lon,lat --sea lon,lat] [--steps a,b,...] [--final]");
        _p = Places.TryGetValue(_place, out var known) ? Clone(known) : new Spec();
        var hasBox = Places.ContainsKey(_place);
        if (a.BoxOpt() is { } bx) { _p.Box = bx; _p.BoxWhy ??= "given on the command line"; hasBox = true; }
        if (!hasBox) throw new HarvestStop($"unknown place '{_place}' (have [{string.Join(", ", Places.Keys.Select(k => "'" + k + "'"))}]); give --box");
        if (a.LonLat("--anchor") is { } an) _p.Anchor = an;
        else if (!Places.ContainsKey(_place)) _p.Anchor = (_p.Box.CenterLon, _p.Box.CenterLat);
        if (a.LonLat("--sea") is { } se) _p.Sea = se;
        else if (!Places.ContainsKey(_place)) _p.Sea = _p.Anchor;
        var dflt = _p.Ofs is not null ? "plan,tides,waves,ofs,ofsfield,ofscheck,bathy,survey,terms,manifest" : "tides,currents,waves,bathy,survey,terms,manifest";
        var steps = (a.Opt("--steps", dflt) ?? dflt).Split(',');
        _polite = new Polite(ctx) { Place = _place };
        LoadRecord();
        _rec["box_why"] = _p.BoxWhy;
        var left = _polite.Remaining();
        Log($"=== place {_place}: box {_p.Box}, anchor ({F(_p.Anchor.lon)}, {F(_p.Anchor.lat)}), sea ({F(_p.Sea.lon)}, {F(_p.Sea.lat)}); steps [{string.Join(", ", steps.Select(s => "'" + s + "'"))}]; budget left {left.bytes / 1e9:0.000} GB / {left.requests} requests");
        try
        {
            foreach (var s in steps)
            {
                switch (s)
                {
                    case "plan": StepPlan(); break;
                    case "tides": StepTides(); break;
                    case "currents": StepCurrents(); break;
                    case "waves": StepWaves(); break;
                    case "ofs": StepOfs(); break;
                    case "ofsfield": StepOfsField(); break;
                    case "ofscheck": StepOfsCheck(); break;
                    case "bathy": StepBathy(); break;
                    case "survey": StepSurvey(); break;
                    case "terms": StepTerms(); break;
                    case "manifest": StepManifest(a.Has("--final")); break;
                    default: throw new HarvestStop($"unknown step '{s}'");
                }
                SaveRecord();
            }
        }
        catch (Polite.StopException e)
        {
            _rec["stopped"] = e.Message;
            Log($"STOPPED: {e.Message}");
        }
        finally
        {
            _rec["ledger"] = _polite.Summary();
            SaveRecord();
        }
        var sum = _polite.Summary();
        Log($"=== {_place}: ledger {sum["requests"]} requests, {Convert.ToDouble(sum["bytes"]) / 1e9:0.000} GB; record {RecordPath()}");
        return 0;
    }

    static Spec Clone(Spec s) => new()
    {
        Box = s.Box, BoxWhy = s.BoxWhy, Anchor = s.Anchor, Sea = s.Sea, TideRadiusKm = s.TideRadiusKm, CurrentRadiusKm = s.CurrentRadiusKm,
        AdcpRadiusKm = s.AdcpRadiusKm, MaxCurrentStations = s.MaxCurrentStations, Ofs = s.Ofs, GridBox = s.GridBox, GridName = s.GridName,
    };
}
