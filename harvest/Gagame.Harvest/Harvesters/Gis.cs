using System.IO.Compression;
using static Gagame.Harvest.JsonUtil;

namespace Gagame.Harvest.Harvesters;

/// M6i: GIS ground truth for the stencil overlay (harvest_gis.py). The GSHHG native-binary
/// distribution once, cached forever; full-resolution shorelines and WDBII rivers for the New
/// England window, a low-resolution global coastline, and the two raster land masks baked by
/// even-odd scanline parity per hierarchy level.
public sealed class Gis : IHarvester
{
    public string Name => "gis";
    public string Title => "Shorelines (GSHHG)";
    public string What => "Full-resolution New England shoreline and rivers plus a low-resolution global coast, for the stencil.";
    public string[] Outputs => ["data/gis/gis.json"];
    public Source[] Sources => [new("GSHHG", "https://www.ngdc.noaa.gov/mgg/shorelines/")];
    public Insert? Insert => null;

    public static string CacheDir => Path.Combine("cache", "gshhg");
    public static string OutDir => Path.Combine("data", "gis");
    static readonly string[] ZipUrls =
    [
        "https://www.soest.hawaii.edu/pwessel/gshhg/gshhg-bin-2.3.7.zip",
        "https://www.ngdc.noaa.gov/mgg/shorelines/data/gshhg/oldversions/version2.3.7/gshhg-bin-2.3.7.zip",
        "https://www.ncei.noaa.gov/pub/web/mgg/shorelines/data/gshhg/latest/gshhg-bin-2.3.7.zip",
    ];
    public static string ZipPath => Path.Combine(CacheDir, "gshhg-bin-2.3.7.zip");
    public static readonly Box Ne = new(-75.0, 40.0, -66.0, 45.5);

    // The Merrimack z14 Mercator window: MUST match main.cpp's winOrg/size (one shared frame).
    public const long WinOrgX = 4935L * 256, WinOrgY = 6008L * 256, WinSize = 16384, N14 = 16384L * 256;

    public static void FetchZip(HarvestContext ctx)
    {
        Directory.CreateDirectory(CacheDir);
        if (File.Exists(ZipPath) && new FileInfo(ZipPath).Length > 50_000_000)
        {
            ctx.Log($"[gis] cache hit: {ZipPath} ({new FileInfo(ZipPath).Length / 1048576} MB)");
            return;
        }
        var f = new SimpleFetcher(ctx, 0.0, 1);
        foreach (var url in ZipUrls)
        {
            ctx.Log($"[gis] downloading {url} (one-time; lands in the forever-cache)");
            try
            {
                f.DownloadBig([url], ZipPath, minBytes: 50_000_000, timeoutS: 600, tag: "GSHHG");
                ctx.Log($"[gis] downloaded {new FileInfo(ZipPath).Length / 1048576} MB");
                return;
            }
            catch (Exception e) { ctx.Log($"[gis] {e.Message}; trying the next mirror"); }
        }
        throw new HarvestStop("[gis] every mirror failed");
    }

    public static byte[] Member(ZipArchive zf, string baseName)
    {
        var e = zf.Entries.FirstOrDefault(x => x.Name == baseName) ?? throw new HarvestStop($"[gis] FATAL: {baseName} not in the zip");
        using var s = e.Open();
        using var ms = new MemoryStream();
        s.CopyTo(ms);
        return ms.ToArray();
    }

    /// GSHHG native binary -> (level, points). flag & 255 is the hierarchy level.
    public static List<(int level, List<(double lon, double lat)> line)> ParseGshhg(byte[] blob, double west, double east, double south, double north, int minPoints = 2, int decimate = 1, HashSet<int>? wantLevels = null)
    {
        var lines = new List<(int, List<(double, double)>)>();
        var off = 0;
        var size = blob.Length;
        while (off + 44 <= size)
        {
            int n = Bin.I32BE(blob, off + 4), flag = Bin.I32BE(blob, off + 8);
            double w = Bin.I32BE(blob, off + 12), e = Bin.I32BE(blob, off + 16), s = Bin.I32BE(blob, off + 20), nn = Bin.I32BE(blob, off + 24);
            off += 44;
            if (n <= 0 || off + 8L * n > size) break;
            var level = flag & 255;
            w *= 1e-6; e *= 1e-6; s *= 1e-6; nn *= 1e-6;
            if (w > 180.0) w -= 360.0;
            if (e > 180.0) e -= 360.0;
            var keep = !(e < west || w > east || nn < south || s > north);
            if (west <= -180.0 && east >= 180.0) keep = true;
            if (wantLevels is not null && !wantLevels.Contains(level)) keep = false;
            if (!keep) { off += 8 * n; continue; }
            var line = new List<(double, double)>(n / decimate + 1);
            for (var i = 0; i < n; i += decimate)
            {
                var lon = Bin.I32BE(blob, off + 8 * i) * 1e-6;
                var lat = Bin.I32BE(blob, off + 8 * i + 4) * 1e-6;
                if (lon > 180.0) lon -= 360.0;
                line.Add((lon, lat));
            }
            off += 8 * n;
            if (line.Count >= minPoints) lines.Add((level, line));
        }
        return lines;
    }

    /// Filled land mask by even-odd scanline parity, per GSHHG hierarchy level.
    public static byte[] BakeMask(List<(int level, List<(double lon, double lat)> line)> levelled, int rows, int cols, Func<int, double> rowToLat, Func<double, double> lonToCol, double lonMin, double lonMax)
    {
        var mask = new byte[rows * cols];
        var rowLats = new double[rows];
        for (var r = 0; r < rows; r++) rowLats[r] = rowToLat(r);
        double latHi = rowLats[0], latLo = rowLats[^1];
        int RowOf(double lat)
        {
            int lo = 0, hi = rows - 1;
            while (lo < hi)
            {
                var mid = (lo + hi) / 2;
                if (rowLats[mid] > lat) lo = mid + 1; else hi = mid;
            }
            return lo;
        }
        var crossings = new List<(double lon, int level)>[rows];
        foreach (var (level, line) in levelled)
        {
            var n = line.Count;
            for (var i = 0; i < n; i++)
            {
                var (lon0, lat0) = line[i];
                var (lon1, lat1) = line[(i + 1) % n];
                if (lat0 == lat1) continue;
                if (Math.Abs(lon1 - lon0) > 180.0) continue;
                if (Math.Max(lat0, lat1) < latLo || Math.Min(lat0, lat1) > latHi) continue;
                var r0 = RowOf(Math.Max(lat0, lat1));
                var r1 = RowOf(Math.Min(lat0, lat1));
                var lv = level == 5 ? 1 : level;
                if (lv == 6) continue;
                for (var r = Math.Max(r0, 0); r < Math.Min(r1 + 1, rows); r++)
                {
                    var lat = rowLats[r];
                    var (lo, hi) = lat0 < lat1 ? (lat0, lat1) : (lat1, lat0);
                    if (!(lo <= lat && lat < hi)) continue;
                    var t = (lat - lat0) / (lat1 - lat0);
                    (crossings[r] ??= new()).Add((lon0 + (lon1 - lon0) * t, lv));
                }
            }
        }
        for (var r = 0; r < rows; r++)
        {
            if (crossings[r] is null) continue;
            var evs = crossings[r].OrderBy(c => c.lon).ThenBy(c => c.level).ToList();
            var parity = new int[5];
            var spans = new List<(double a, double b)>();
            var prevLon = -1e9;
            foreach (var (lon, level) in evs)
            {
                var newState = (parity[1] ^ parity[2] ^ parity[3] ^ parity[4]) == 1;
                if (newState) spans.Add((prevLon, lon));
                parity[Math.Min(level, 4)] ^= 1;
                prevLon = lon;
            }
            foreach (var (a, b) in spans)
            {
                var c0 = Math.Max(0, (int)Math.Ceiling(lonToCol(Math.Max(a, lonMin)) - 0.5));
                var c1 = Math.Min(cols - 1, (int)Math.Floor(lonToCol(Math.Min(b, lonMax)) - 0.5));
                var bse = r * cols;
                for (var c = c0; c <= c1; c++) mask[bse + c] = 255;
            }
        }
        return mask;
    }

    /// Split polylines into runs whose points fall inside the box.
    public static List<List<(double lon, double lat)>> ClipToBox(IEnumerable<List<(double lon, double lat)>> lines, Box box)
    {
        var out_ = new List<List<(double, double)>>();
        foreach (var line in lines)
        {
            var run = new List<(double, double)>();
            foreach (var (lon, lat) in line)
            {
                if (box.Contains(lon, lat)) run.Add((lon, lat));
                else if (run.Count > 0)
                {
                    if (run.Count >= 2) out_.Add(run);
                    run = new();
                }
            }
            if (run.Count >= 2) out_.Add(run);
        }
        return out_;
    }

    public static double WinRowLat(int r)
    {
        var y = (WinOrgY + (r + 0.5) * WinSize / 4096.0) / N14;
        return Math.Atan(Math.Sinh(Math.PI * (1.0 - 2.0 * y))) * Geo.RadToDeg;
    }

    public static double WinLonCol(double lon) => ((lon + 180.0) / 360.0 * N14 - WinOrgX) * 4096.0 / WinSize;
    public static double WinLon0 => WinOrgX / (double)N14 * 360.0 - 180.0;
    public static double WinLon1 => (WinOrgX + WinSize) / (double)N14 * 360.0 - 180.0;

    public int Run(HarvestContext ctx, string[] argv)
    {
        FetchZip(ctx);
        Directory.CreateDirectory(OutDir);
        using var zf = ZipFile.OpenRead(ZipPath);

        var coastF = ParseGshhg(Member(zf, "gshhs_f.b"), Ne.Lon0, Ne.Lon1, Ne.Lat0, Ne.Lat1);
        var coastNe = ClipToBox(coastF.Where(l => l.level is 1 or 2).Select(l => l.line), Ne);
        PolylineBin.Write(Path.Combine(OutDir, "coast_ne.bin"), coastNe, ctx);

        var riversF = ParseGshhg(Member(zf, "wdb_rivers_f.b"), Ne.Lon0, Ne.Lon1, Ne.Lat0, Ne.Lat1);
        PolylineBin.Write(Path.Combine(OutDir, "rivers_ne.bin"), ClipToBox(riversF.Select(l => l.line), Ne), ctx);

        var coastL = ParseGshhg(Member(zf, "gshhs_l.b"), -180.0, 180.0, -90.0, 90.0, decimate: 2, wantLevels: new() { 1 });
        PolylineBin.Write(Path.Combine(OutDir, "coast_global.bin"), coastL.Select(l => l.line).ToList(), ctx);

        ctx.Log("[gis] baking the window land mask (4096^2, shared Mercator z14 frame)...");
        var maskNe = BakeMask(coastF, 4096, 4096, WinRowLat, WinLonCol, WinLon0, WinLon1);
        File.WriteAllBytes(Path.Combine(OutDir, "landmask_ne.raw"), maskNe);
        ctx.Log($"[gis] landmask_ne.raw: {100.0 * maskNe.Count(b => b != 0) / maskNe.Length:0.0}% land");

        ctx.Log("[gis] baking the global land mask (4096x2048 equirect)...");
        var coastLAll = ParseGshhg(Member(zf, "gshhs_l.b"), -180.0, 180.0, -90.0, 90.0);
        const int gw = 4096, gh = 2048;
        var maskG = BakeMask(coastLAll, gh, gw, r => 90.0 - (r + 0.5) / gh * 180.0, lon => (lon + 180.0) / 360.0 * gw, -180.0, 180.0);
        File.WriteAllBytes(Path.Combine(OutDir, "landmask_global.raw"), maskG);
        ctx.Log($"[gis] landmask_global.raw: {100.0 * maskG.Count(b => b != 0) / maskG.Length:0.0}% land (Earth is ~29%)");

        Save(Path.Combine(OutDir, "gis.json"), D(
            ("coast_ne", "coast_ne.bin"), ("rivers_ne", "rivers_ne.bin"), ("coast_global", "coast_global.bin"),
            ("landmask_ne", "landmask_ne.raw"), ("landmask_ne_dim", 4096),
            ("landmask_global", "landmask_global.raw"), ("landmask_global_w", gw), ("landmask_global_h", gh),
            ("win_org_px", new List<long> { WinOrgX, WinOrgY }), ("win_size_px", WinSize),
            ("source", "GSHHG 2.3.7 full-res shorelines + WDBII rivers (NOAA/NCEI)"), ("crs", "EPSG:4326"), ("ne_window", Ne.ToJson())));
        ctx.Log("[gis] done -> data/gis/gis.json");
        return 0;
    }
}
