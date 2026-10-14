using System.Globalization;
using System.IO.Compression;
using System.Text.RegularExpressions;
using Gagame.Harvest.Readers;
using static Gagame.Harvest.JsonUtil;

namespace Gagame.Harvest.Harvesters;

/// M5: NOAA CUDEM 1/9-arc-second topobathy for a window or a place (harvest_bathy.py). The
/// tiles come through the dataset's own tile index, are cached forever on the big-data drive,
/// decoded windowed, and box-resampled THALWEG-PRESERVING onto a regular grid, row 0 = north.
public sealed class Bathy : IHarvester
{
    public string Name => "bathy";
    public string Title => "Bathymetry (CUDEM 1/9\")";
    public string What => "NOAA CUDEM topobathy for a window: jetties, beach and channel in one surface, metres NAVD88.";
    public string[] Outputs => ["data/bathy/merrimack.json", "data/bathy/merrimack.f32"];
    public Source[] Sources => [new("NOAA CUDEM", "https://coast.noaa.gov/htdata/raster2/elevation/"), new("NODD coastal lidar bucket", "https://registry.opendata.aws/noaa-coastal-lidar/")];
    public Insert? Insert => new(InsertKind.PlaceBox, "Another named window, or any place by its box (--near picks the tile order).",
        Choices: ["merrimack", "capeann", "boston"], ChoiceFlag: "--window", Extras: ["--near"]);
    public string[] DefaultArgs => ["--window", "merrimack"];

    public const string S3Bucket = "noaa-nos-coastal-lidar-pds";
    public const string S3Dir = "dem/NCEI_ninth_Topobathy_2014_8483/";
    public const string IndexName = "tileindex_NCEI_ninth_Topobathy_2014.zip";
    public const string IndexUrl = $"https://{S3Bucket}.s3.amazonaws.com/{S3Dir}{IndexName}";

    public static readonly Dictionary<string, Box> Windows = new()
    {
        ["merrimack"] = new(-71.000, 42.700, -70.770, 42.845),
        ["capeann"] = new(-70.950, 42.480, -70.560, 42.700),
        ["boston"] = new(-71.100, 42.250, -70.780, 42.480),
    };
    public const int Step = 4;

    HarvestContext _ctx = null!;

    public string BigCache => _ctx.Big("cudem");

    /// The tile cache for a context: the big-data drive when it exists, else the repo cache.
    public string BigCacheFor(HarvestContext ctx)
    {
        _ctx = ctx;
        return ctx.HasDataCache ? BigCache : LegacyCache;
    }
    static string LegacyCache => Path.Combine("cache", "bathy");

    public sealed record Tile(string Name, string Region, string Url, string Srs, double[]? Bbox);
    public sealed record Needed(string Name, string Url, Box Bounds, Tile T);

    // ------------------------------------------------------------------ tiles

    /// The index shapefile: DBF columns (location = <region>/<tile>.tif, srs, URL) + each polygon's bbox.
    public static List<Tile> ReadTileIndex(byte[] zipBytes)
    {
        using var z = new ZipArchive(new MemoryStream(zipBytes), ZipArchiveMode.Read);
        byte[] Member(string ext)
        {
            var e = z.Entries.First(x => x.Name.ToLowerInvariant().EndsWith(ext));
            using var s = e.Open();
            using var ms = new MemoryStream();
            s.CopyTo(ms);
            return ms.ToArray();
        }
        var shp = Member(".shp");
        var dbf = Member(".dbf");
        var rows = Shapefile.Dbf(dbf);
        var boxes = new List<double[]?>();
        var off = 100;
        while (off + 8 <= shp.Length)
        {
            var clen = Bin.I32BE(shp, off + 4) * 2;
            var rec = shp.AsSpan(off + 8, Math.Min(clen, shp.Length - off - 8));
            off += 8 + clen;
            var st = rec.Length >= 4 ? Bin.I32(rec, 0) : 0;
            boxes.Add(st is 5 or 15 or 25 && rec.Length >= 36 ? new[] { Bin.F64(rec, 4), Bin.F64(rec, 12), Bin.F64(rec, 20), Bin.F64(rec, 28) } : null);
        }
        var tiles = new List<Tile>();
        for (var i = 0; i < rows.Count; i++)
        {
            var vals = rows[i].ToDictionary(kv => kv.Key.ToLowerInvariant(), kv => kv.Value);
            var loc = vals.GetValueOrDefault("location", "");
            var cut = loc.LastIndexOf('/');
            var region = cut >= 0 ? loc[..cut] : "";
            var name = cut >= 0 ? loc[(cut + 1)..] : loc;
            tiles.Add(new Tile(name, region, vals.GetValueOrDefault("url", ""), vals.GetValueOrDefault("srs", ""), i < boxes.Count ? boxes[i] : null));
        }
        return tiles;
    }

    /// ncei19_n43x00_w070x75_* -> the tile's NW corner; cells are 0.25 deg (Florida spells the separator 'X').
    public static Box? TileBounds(string name)
    {
        var m = Regex.Match(name, @"([ns])(\d+)x(\d+)_([ew])(\d+)x(\d+)", RegexOptions.IgnoreCase);
        if (!m.Success) return null;
        var latN = int.Parse(m.Groups[2].Value) + int.Parse(m.Groups[3].Value) / 100.0;
        var lonW = int.Parse(m.Groups[5].Value) + int.Parse(m.Groups[6].Value) / 100.0;
        if (m.Groups[1].Value.ToLowerInvariant() == "s") latN = -latN;
        if (m.Groups[4].Value.ToLowerInvariant() == "w") lonW = -lonW;
        return new Box(lonW, latN - 0.25, lonW + 0.25, latN);
    }

    /// Index tiles whose cell meets the window, in URL order.
    public static List<Needed> TilesFor(Box win, List<Tile> index)
    {
        var needed = new List<Needed>();
        foreach (var t in index.OrderBy(t => t.Url, StringComparer.Ordinal))
        {
            var b = TileBounds(t.Name);
            if (b is null) continue;
            if (b.Lon0 < win.Lon1 && b.Lon1 > win.Lon0 && b.Lat0 < win.Lat1 && b.Lat1 > win.Lat0) needed.Add(new Needed(t.Name, t.Url, b, t));
        }
        return needed;
    }

    public static string? CachedTile(Tile t, string cacheDir)
    {
        foreach (var p in new[] { Path.Combine(cacheDir, t.Region, t.Name), Path.Combine(cacheDir, t.Name), Path.Combine(LegacyCache, t.Name) })
            if (File.Exists(p)) return p;
        return null;
    }

    public static double KmToCell(Box b, double lon, double lat)
    {
        var clon = Math.Min(Math.Max(lon, b.Lon0), b.Lon1);
        var clat = Math.Min(Math.Max(lat, b.Lat0), b.Lat1);
        return Geo.FlatKm(lon, lat, clon, clat);
    }

    // ------------------------------------------------------------------ the grid

    public sealed class GridResult
    {
        public float[] Grid = Array.Empty<float>();
        public int Nx, Ny;
        public double Dlon, Dlat;
        public Dictionary<string, Dictionary<string, object?>> Declared = new();
    }

    /// Decode the tiles under `win` into a regular lat/lon grid at step * 1/9 arcsec, row 0 = north, thalweg-preserving.
    public GridResult BuildGrid(Box win, List<(string name, string url, Box b)> needed, int step, Func<string, string, byte[]> loadTile)
    {
        var srcStep = 1.0 / 9.0 / 3600.0;
        var dlat = srcStep * step;
        var dlon = srcStep * step;
        var nx = (int)((win.Lon1 - win.Lon0) / dlon);
        var ny = (int)((win.Lat1 - win.Lat0) / dlat);
        var grid = new float[nx * ny];
        Array.Fill(grid, float.NaN);
        _ctx.Log($"output grid {nx} x {ny} at ~{dlat * 110574:0.0} m");
        var res = new GridResult { Grid = grid, Nx = nx, Ny = ny, Dlon = dlon, Dlat = dlat };

        foreach (var (name, url, b) in needed)
        {
            var data = loadTile(name, url);
            var tif = new GeoTiff(data);
            res.Declared[name] = tif.GeoKeys();
            _ctx.Log($"    {tif.Width}x{tif.Height}, comp {tif.Compression}, predictor {tif.Predictor}, {(tif.Tiled ? "tiled" : "striped")} {tif.Tw}x{tif.Th}, origin ({tif.Lat0:0.0000}, {tif.Lon0:0.0000}), nodata {(tif.Nodata is double nd ? nd.ToString("R", CultureInfo.InvariantCulture) : "None")}");
            var x0 = Math.Max(tif.PxOfLon(Math.Max(win.Lon0, b.Lon0)), 0);
            var x1 = Math.Min(tif.PxOfLon(Math.Min(win.Lon1, b.Lon1)), tif.Width);
            var y0 = Math.Max(tif.PyOfLat(Math.Min(win.Lat1, b.Lat1)), 0);
            var y1 = Math.Min(tif.PyOfLat(Math.Max(win.Lat0, b.Lat0)), tif.Height);
            int w = x1 - x0, h = y1 - y0;
            if (w <= 0 || h <= 0) continue;
            _ctx.Log($"    decoding window {w} x {h} px");
            var vals = tif.ReadWindow(x0, y0, w, h);
            var cnt = new ushort[nx * ny];
            var ssum = new double[nx * ny];
            var wcnt = new ushort[nx * ny];
            var wsum = new double[nx * ny];
            var vmin = new float[nx * ny];
            Array.Fill(vmin, 1e9f);
            var colmap = new int[w];
            for (var xx = 0; xx < w; xx++)
            {
                var lon = tif.Lon0 + (x0 + xx) * tif.Sx;
                var gx = (int)((lon - win.Lon0) / dlon);
                colmap[xx] = 0 <= gx && gx < nx ? gx : -1;
            }
            long samples = 0;
            for (var yy = 0; yy < h; yy++)
            {
                var lat = tif.Lat0 - (y0 + yy) * tif.Sy;
                var gy = (int)((win.Lat1 - lat) / dlat);
                if (!(0 <= gy && gy < ny)) continue;
                var bse = gy * nx;
                for (var xx = 0; xx < w; xx++)
                {
                    var v = vals[yy * w + xx];
                    if (float.IsNaN(v)) continue;
                    var gx = colmap[xx];
                    if (gx < 0) continue;
                    var i = bse + gx;
                    cnt[i]++;
                    ssum[i] += v;
                    samples++;
                    if (v < -1.0f)
                    {
                        wcnt[i]++;
                        wsum[i] += v;
                        if (v < vmin[i]) vmin[i] = v;
                    }
                }
            }
            var copied = 0;
            for (var i = 0; i < nx * ny; i++)
            {
                if (cnt[i] == 0) continue;
                grid[i] = wcnt[i] * 2 >= cnt[i] ? (float)(0.5 * (wsum[i] / wcnt[i]) + 0.5 * vmin[i]) : (float)(ssum[i] / cnt[i]);
                copied++;
            }
            _ctx.Log($"    filled {copied} cells from {samples} samples (thalweg-preserving)");
        }
        return res;
    }

    public List<(string path, long bytes)> WriteOutputs(string outDir, string name, Box win, GridResult g, string source, Dictionary<string, object?>? extra = null, bool exclusive = false)
    {
        var valid = g.Grid.Where(v => !float.IsNaN(v)).ToList();
        if (valid.Count == 0) throw new InvalidOperationException("window decoded to all-nodata");
        float lo = valid.Min(), hi = valid.Max();
        var below = valid.Count(v => v < 0.0f);
        _ctx.Log($"elevation range [{lo:0.0}, {hi:0.0}] m, {100.0 * below / valid.Count:0}% below datum, {100.0 * valid.Count / (g.Nx * g.Ny):0}% coverage");
        var paths = new[] { Path.Combine(outDir, $"{name}.f32"), Path.Combine(outDir, $"{name}.json"), Path.Combine(outDir, $"preview_{name}.png") };
        if (exclusive)
        {
            var clash = paths.Where(File.Exists).ToList();
            if (clash.Count > 0) throw new HarvestStop($"refusing to overwrite [{string.Join(", ", clash.Select(c => "'" + c + "'"))}] (place harvests only add)");
        }
        Directory.CreateDirectory(outDir);
        var enc = g.Grid.Select(v => float.IsNaN(v) ? -9999.0f : v).ToArray();
        Bin.WriteF32(paths[0], enc, exclusive);
        var meta = D(("generated_utc", NowIso()), ("source", source), ("file", $"{name}.f32"), ("nx", g.Nx), ("ny", g.Ny),
            ("lon0", win.Lon0), ("lat1", win.Lat1), ("dlon", g.Dlon), ("dlat", g.Dlat), ("row0", "north"), ("nodata", -9999.0),
            ("min_m", (double)lo), ("max_m", (double)hi));
        if (extra is not null) foreach (var (k, v) in extra) meta[k] = v;
        Save(paths[1], meta, exclusive);
        var pix = new byte[g.Nx * g.Ny];
        for (var i = 0; i < pix.Length; i++)
        {
            var v = g.Grid[i];
            pix[i] = float.IsNaN(v) ? (byte)8 : v < 0 ? (byte)Math.Max(16, Math.Min(120, (int)(120 + v * 8))) : (byte)Math.Max(140, Math.Min(255, (int)(140 + v * 10)));
        }
        Png.WriteGray(paths[2], pix, g.Nx, g.Ny, exclusive);
        _ctx.Log($"wrote {outDir}/{name}.json + .f32 + preview ({g.Nx}x{g.Ny})");
        return paths.Select(p => (p, new FileInfo(p).Length)).ToList();
    }

    static string SourceLabel(IEnumerable<Needed> needed) =>
        $"NOAA NCEI CUDEM ninth-arc topobathy (2014_8483 {string.Join(" + ", needed.Select(n => n.T.Region).Distinct().OrderBy(r => r, StringComparer.Ordinal))})";

    // ------------------------------------------------------------------ place

    Dictionary<string, long> RegionSizes(IEnumerable<string> regions, Polite polite)
    {
        var sizes = new Dictionary<string, long>();
        foreach (var region in regions)
        {
            var (objs, _) = polite.S3List(S3Bucket, $"{S3Dir}{region}/", Path.Combine("cache", "places", "_s3", $"cudem_{region}.xml"));
            foreach (var o in objs) sizes[o.Key[(o.Key.LastIndexOf('/') + 1)..]] = o.Size;
        }
        return sizes;
    }

    public (List<Needed> needed, Polite.Plan plan) PlanPlace(string name, Box box, (double lon, double lat) near, string cacheDir, List<Tile> index, Polite polite, bool offline = false)
    {
        var needed = TilesFor(box, index);
        var sizes = offline ? new() : RegionSizes(needed.Select(n => n.T.Region).Distinct().OrderBy(r => r, StringComparer.Ordinal), polite);
        var plan = polite.NewPlan($"CUDEM tiles for {name}");
        foreach (var n in needed.OrderBy(n => KmToCell(n.Bounds, near.lon, near.lat)))
        {
            var have = CachedTile(n.T, cacheDir);
            long? size = sizes.TryGetValue(n.Name, out var s) ? s : have is not null ? new FileInfo(have).Length : null;
            plan.Add(n.Name, n.Url, size, have ?? Path.Combine(cacheDir, n.T.Region, n.Name),
                ("km", Round(KmToCell(n.Bounds, near.lon, near.lat), 1)), ("region", n.T.Region), ("srs", n.T.Srs));
        }
        return (needed, plan);
    }

    public sealed class PlaceRecord
    {
        public List<Dictionary<string, object?>> Plan = new();
        public List<string> Later = new();
        public List<(string path, long bytes)> Files = new();
        public Dictionary<string, Dictionary<string, object?>>? Declared;
    }

    public PlaceRecord HarvestPlace(HarvestContext ctx, Polite polite, string name, Box box, (double lon, double lat)? near = null, int step = Step, string? outDir = null,
        bool planOnly = false, bool offline = false, long reserve = 0, double? maxBytes = null, Box? gridBox = null, string? outName = null)
    {
        _ctx = ctx;
        outDir ??= Path.Combine("data", "bathy");
        var cacheDir = ctx.HasDataCache ? BigCache : LegacyCache;
        var nr = near ?? (box.CenterLon, box.CenterLat);
        var indexPath = Path.Combine(cacheDir, IndexName);
        if (offline && !File.Exists(indexPath)) throw new HarvestStop($"--offline: {indexPath} is not cached");
        var index = ReadTileIndex(polite.Get(IndexUrl, indexPath).data);
        var (needed, plan) = PlanPlace(name, box, nr, cacheDir, index, polite, offline);
        ctx.Log($"place '{name}' box {box}: the index names {needed.Count} tile(s) in [{string.Join(", ", needed.Select(n => n.T.Region).Distinct().OrderBy(r => r, StringComparer.Ordinal).Select(r => "'" + r + "'"))}]");
        var (keep, cut) = plan.Fit(reserve);
        if (maxBytes is double mb)
        {
            long run = 0;
            var kept = new List<Polite.PlanItem>();
            foreach (var it in keep)
            {
                if (!it.Cached)
                {
                    if (cut.Count > 0 || run + (it.Size ?? 0) > mb) { cut.Add(it); continue; }
                    run += it.Size ?? 0;
                }
                kept.Add(it);
            }
            keep = kept;
        }
        plan.Print(keep, cut);
        var record = new PlaceRecord { Plan = plan.Items.Select(i => i.ToJson()).ToList(), Later = cut.Select(i => i.Key).ToList() };
        if (planOnly) return record;
        foreach (var it in keep)
        {
            if (!it.Cached)
            {
                if (offline) throw new HarvestStop($"--offline: {it.Key} is not cached");
                polite.Download(it.Url, it.Dest, it.Size!.Value, note: $"cudem {name}");
            }
            it.Cached = true;
        }
        var have = plan.Items.Where(i => File.Exists(i.Dest)).ToDictionary(i => i.Key, i => i.Dest);
        var gridOut = Path.Combine(outDir, $"{outName ?? name}.f32");
        if (File.Exists(gridOut))
        {
            ctx.Log($"{gridOut} exists: tiles fetched, grid left as it is (a place only adds)");
            return record;
        }
        var gbox = gridBox ?? box;
        var all = TilesFor(gbox, index);
        var use = all.Where(n => have.ContainsKey(n.Name)).ToList();
        var missing = all.Where(n => !have.ContainsKey(n.Name)).Select(n => n.Name).ToList();
        if (missing.Count > 0) ctx.Log($"grid box {gbox}: {missing.Count} tile(s) not fetched yet: [{string.Join(", ", missing.Select(m => "'" + m + "'"))}]");
        if (use.Count == 0) { ctx.Log("no tiles on disk under the grid box; no grid written"); return record; }
        byte[] LoadTile(string tname, string url)
        {
            var data = File.ReadAllBytes(have[tname]);
            ctx.Log($"    {tname}: {data.Length / 1e6:0.0} MB (cached)");
            return data;
        }
        var g = BuildGrid(gbox, use.Select(n => (n.Name, n.Url, n.Bounds)).ToList(), step, LoadTile);
        var extra = D(("place", name), ("box", gbox.ToJson()), ("step", step),
            ("tiles", use.Select(n => D(("name", n.Name), ("region", n.T.Region), ("url", n.Url), ("srs_index", n.T.Srs), ("geokeys", g.Declared.GetValueOrDefault(n.Name)))).ToList()),
            ("tiles_missing", missing));
        record.Files = WriteOutputs(outDir, outName ?? name, gbox, g, SourceLabel(use), extra, exclusive: true);
        record.Declared = g.Declared;
        return record;
    }

    // ------------------------------------------------------------------ main

    public int Run(HarvestContext ctx, string[] argv)
    {
        _ctx = ctx;
        var a = new Args(argv);
        var offline = a.Has("--offline");
        if (a.Has("--place"))
        {
            var name = a.Opt("--place")!;
            var box = a.BoxOpt() ?? throw new HarvestStop("--place needs --box lon0,lat0,lon1,lat1");
            HarvestPlace(ctx, new Polite(ctx) { Place = name }, name, box, a.LonLat("--near"), a.Int("--step", Step), a.Opt("--out"),
                planOnly: a.Has("--plan"), offline: offline, maxBytes: a.Has("--max-bytes") ? a.Num("--max-bytes", 0) : null);
            return 0;
        }
        var winName = a.Opt("--window", "merrimack")!;
        if (!Windows.TryGetValue(winName, out var win)) throw new HarvestStop($"unknown window '{winName}' (have: [{string.Join(", ", Windows.Keys.Select(k => "'" + k + "'"))}])");
        var outDir = a.Opt("--out", Path.Combine("data", "bathy"))!;
        var cacheDir = ctx.HasDataCache ? BigCache : Path.Combine("cache", "bathy");
        Directory.CreateDirectory(cacheDir);
        Directory.CreateDirectory(outDir);
        ctx.Log($"window '{winName}' {win} (tile cache: {cacheDir})");
        var fetch = new SimpleFetcher(ctx, 0.6, 3);
        var indexPath = Path.Combine(cacheDir, IndexName);
        List<Tile> index;
        if (offline)
        {
            if (!File.Exists(indexPath)) throw new HarvestStop($"--offline: {indexPath} is not cached");
            index = ReadTileIndex(File.ReadAllBytes(indexPath));
        }
        else index = ReadTileIndex(fetch.Get(IndexUrl, indexPath).data);
        ctx.Log($"tile index lists {index.Count} tiles");
        var needed = TilesFor(win, index);
        ctx.Log($"window needs {needed.Count} tile(s): [{string.Join(", ", needed.Select(n => "'" + n.Name + "'"))}]");
        if (needed.Count == 0) throw new InvalidOperationException("no tile covers the window; check the name convention");
        byte[] LoadTile(string name, string url)
        {
            var t = needed.First(n => n.Name == name).T;
            var have = CachedTile(t, cacheDir);
            byte[] data; bool cached;
            if (have is not null) { data = File.ReadAllBytes(have); cached = true; }
            else if (offline) throw new HarvestStop($"--offline: {name} is not cached");
            else (data, cached) = fetch.Get(url, Path.Combine(cacheDir, name), timeoutS: 1200);
            ctx.Log($"    {name}: {data.Length / 1e6:0.0} MB {(cached ? "(cached)" : "(fetched)")}");
            return data;
        }
        var g = BuildGrid(win, needed.Select(n => (n.Name, n.Url, n.Bounds)).ToList(), Step, LoadTile);
        WriteOutputs(outDir, winName, win, g, SourceLabel(needed));
        return 0;
    }
}
