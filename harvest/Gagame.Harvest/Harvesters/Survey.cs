using System.IO.Compression;
using Gagame.Harvest.Readers;
using static Gagame.Harvest.JsonUtil;

namespace Gagame.Harvest.Harvesters;

/// M7y: THE SURVEY UPGRADE (harvest_survey.py). NOAA CUSP (the coastline, polylines) and USGS
/// NHD HR (rivers and CLOSED open-water polygons) join the stencil; the land mask is rebaked
/// as the GSHHG parity base with each NHD open-water feature carved to water.
public sealed class Survey : IHarvester
{
    public string Name => "survey";
    public string Title => "Survey (CUSP + NHD)";
    public string What => "NGS CUSP coastline and USGS NHD HR waterbodies into the survey order; rebakes the land mask.";
    public string[] Outputs => ["data/gis/survey.json"];
    public Source[] Sources => [new("NOAA CUSP", "https://geodesy.noaa.gov/dist_shoreline/"), new("USGS NHD HR", "https://www.usgs.gov/national-hydrography/national-hydrography-dataset")];
    public Insert? Insert => null;
    public string? Note => "Run Vectors after it to refresh the vpack.";

    const string CuspUrl = "https://geodesy.noaa.gov/dist_shoreline/North_Atlantic.zip";
    static readonly string[] NhdHu4 = ["0106", "0107", "0109"];
    const string NhdUrl = "https://prd-tnm.s3.amazonaws.com/StagedProducts/Hydrography/NHD/HU4/GPKG/NHD_H_{0}_HU4_GPKG.zip";
    static readonly Box Focus = new(-71.85, 42.05, -69.85, 43.55);
    static string OutDir => Path.Combine("data", "gis");

    static bool FcodeIsOpenWater(int fc)
    {
        var fam = fc / 100;
        return fam == 390 || fam == 436 || fc == 49300 || fam == 460 || fc == 44500 || fam == 336 || fc == 43100;
    }

    static bool FcodeIsRiverLine(int fc)
    {
        var fam = fc / 100;
        return fam == 460 || fc == 55800 || fam == 336;
    }

    static void Fetch(HarvestContext ctx, SimpleFetcher f, string url, string path, long minBytes)
    {
        if (File.Exists(path) && new FileInfo(path).Length >= minBytes)
        {
            ctx.Log($"[survey] cache hit: {path} ({new FileInfo(path).Length / 1048576} MB)");
            return;
        }
        ctx.Log($"[survey] downloading {url}");
        f.DownloadBig([url], path, minBytes: minBytes - 1, timeoutS: 1200, tag: Path.GetFileName(path));
        ctx.Log($"[survey] done: {path}");
    }

    static (List<List<(double, double)>> rivers, List<(int fc, List<List<(double, double)>> rings)> water) ReadNhdGpkg(string gpkgPath, Box box)
    {
        using var db = Gpkg.Open(gpkgPath);
        var tables = Gpkg.FeatureTables(db);
        var geocol = Gpkg.GeometryColumns(db);
        IEnumerable<(byte[] blob, int fc)> Rows(string tname)
        {
            if (!tables.TryGetValue(tname, out var table)) yield break;
            var gc = geocol.GetValueOrDefault(table, "geom");
            var fc = Gpkg.ColumnNamed(db, table, "fcode");
            using var cmd = db.CreateCommand();
            cmd.CommandText = $"SELECT \"{gc}\"" + (fc is not null ? $", \"{fc}\"" : ", NULL") + $" FROM \"{table}\"";
            using var r = cmd.ExecuteReader();
            while (r.Read())
            {
                if (r.IsDBNull(0)) continue;
                var blob = (byte[])r[0];
                var e = Gpkg.Envelope(blob);
                if (e is not null && (e[1] < box.Lon0 || e[0] > box.Lon1 || e[3] < box.Lat0 || e[2] > box.Lat1)) continue;
                yield return (blob, r.IsDBNull(1) ? 0 : Convert.ToInt32(r[1]));
            }
        }
        var rivers = new List<List<(double, double)>>();
        var water = new List<(int, List<List<(double, double)>>)>();
        foreach (var (blob, fc) in Rows("NHDFlowline"))
        {
            if (!FcodeIsRiverLine(fc)) continue;
            foreach (var g in Gpkg.Geometries(blob))
                foreach (var pts in g.Parts)
                    if (pts.Count >= 2) rivers.Add(pts);
        }
        foreach (var tname in new[] { "NHDWaterbody", "NHDArea" })
            foreach (var (blob, fc) in Rows(tname))
            {
                if (!FcodeIsOpenWater(fc)) continue;
                foreach (var g in Gpkg.Geometries(blob))
                    if (g.Kind == "poly" && g.Parts.Count > 0) water.Add((fc, g.Parts));
            }
        return (rivers, water);
    }

    static List<List<(double, double)>> ReadCuspZip(HarvestContext ctx, string zipPath, Box box)
    {
        using var zf = ZipFile.OpenRead(zipPath);
        var shps = zf.Entries.Where(e => e.Name.ToLowerInvariant().EndsWith(".shp")).ToList();
        ctx.Log($"[survey] CUSP archive: {shps.Count} shapefiles");
        var lines = new List<List<(double, double)>>();
        foreach (var e in shps)
        {
            using var s = e.Open();
            using var ms = new MemoryStream();
            s.CopyTo(ms);
            var data = ms.ToArray();
            if (data.Length < 100 || Bin.I32BE(data, 0) != 9994) continue;
            foreach (var rec in Shapefile.Records(data, box))
                foreach (var run in rec.Parts) lines.Add(run);
        }
        return lines;
    }

    /// Clear mask texels to water inside each feature's rings (even-odd per FEATURE).
    public static int CarveWater(byte[] mask, int rows, int cols, List<(int fc, List<List<(double, double)>> rings)> features, Func<int, double> rowToLat, Func<double, double> lonToCol, double lonMin, double lonMax)
    {
        var rowLats = new double[rows];
        for (var r = 0; r < rows; r++) rowLats[r] = rowToLat(r);
        double latHi = rowLats[0], latLo = rowLats[^1];
        int RowOf(double lat)
        {
            int lo = 0, hi = rows - 1;
            while (lo < hi) { var mid = (lo + hi) / 2; if (rowLats[mid] > lat) lo = mid + 1; else hi = mid; }
            return lo;
        }
        var carved = 0;
        foreach (var (_, rings) in features)
        {
            var crossings = new Dictionary<int, List<double>>();
            foreach (var ring in rings)
            {
                var n = ring.Count;
                for (var i = 0; i < n; i++)
                {
                    var (lon0, lat0) = ring[i];
                    var (lon1, lat1) = ring[(i + 1) % n];
                    if (lat0 == lat1) continue;
                    if (Math.Max(lat0, lat1) < latLo || Math.Min(lat0, lat1) > latHi) continue;
                    var r0 = RowOf(Math.Max(lat0, lat1));
                    var r1 = RowOf(Math.Min(lat0, lat1));
                    for (var r = Math.Max(r0, 0); r < Math.Min(r1 + 1, rows); r++)
                    {
                        var lat = rowLats[r];
                        var (lo, hi) = lat0 < lat1 ? (lat0, lat1) : (lat1, lat0);
                        if (!(lo <= lat && lat < hi)) continue;
                        var t = (lat - lat0) / (lat1 - lat0);
                        if (!crossings.TryGetValue(r, out var xs)) crossings[r] = xs = new();
                        xs.Add(lon0 + (lon1 - lon0) * t);
                    }
                }
            }
            foreach (var (r, xs) in crossings)
            {
                xs.Sort();
                for (var i = 0; i + 1 < xs.Count; i += 2)
                {
                    double a = xs[i], b = xs[i + 1];
                    if (b < lonMin || a > lonMax) continue;
                    var c0 = Math.Max(0, (int)Math.Ceiling(lonToCol(Math.Max(a, lonMin)) - 0.5));
                    var c1 = Math.Min(cols - 1, (int)Math.Floor(lonToCol(Math.Min(b, lonMax)) - 0.5));
                    var bse = r * cols;
                    for (var c = c0; c <= c1; c++)
                        if (mask[bse + c] != 0) { mask[bse + c] = 0; carved++; }
                }
            }
        }
        return carved;
    }

    public int Run(HarvestContext ctx, string[] argv)
    {
        var cuspDir = ctx.Big("cusp");
        var nhdDir = ctx.Big("nhd");
        Directory.CreateDirectory(OutDir);
        Directory.CreateDirectory("proofs");
        var f = new SimpleFetcher(ctx, 0.0, 1);

        var cuspZip = Path.Combine(cuspDir, "North_Atlantic.zip");
        Fetch(ctx, f, CuspUrl, cuspZip, 150_000_000);
        var gpkgs = new List<string>();
        foreach (var hu in NhdHu4)
        {
            var z = Path.Combine(nhdDir, $"NHD_H_{hu}_HU4_GPKG.zip");
            Fetch(ctx, f, string.Format(NhdUrl, hu), z, 30_000_000);
            string? gname = null;
            using (var zf = ZipFile.OpenRead(z))
                foreach (var e in zf.Entries)
                {
                    if (!e.Name.ToLowerInvariant().EndsWith(".gpkg")) continue;
                    gname = e.Name;
                    var dst = Path.Combine(nhdDir, gname);
                    if (!File.Exists(dst) || new FileInfo(dst).Length < e.Length)
                    {
                        ctx.Log($"[survey] extracting {gname}");
                        e.ExtractToFile(dst, overwrite: true);
                    }
                    gpkgs.Add(dst);
                }
            if (gname is null) throw new HarvestStop($"[survey] FATAL: no .gpkg inside {z}");
        }

        var cusp = Gis.ClipToBox(ReadCuspZip(ctx, cuspZip, Focus), Focus);
        PolylineBin.Write(Path.Combine(OutDir, "cusp_ne.bin"), cusp, ctx);

        var rivers = new List<List<(double, double)>>();
        var water = new List<(int fc, List<List<(double, double)>> rings)>();
        foreach (var g in gpkgs)
        {
            var (r, w) = ReadNhdGpkg(g, Focus);
            ctx.Log($"[survey] {Path.GetFileName(g)}: {r.Count} river lines, {w.Count} water polys");
            rivers.AddRange(r);
            water.AddRange(w);
        }
        PolylineBin.Write(Path.Combine(OutDir, "nhd_rivers_ne.bin"), Gis.ClipToBox(rivers, Focus), ctx);
        var ringsFlat = new List<List<(double, double)>>();
        var attrs = new List<Dictionary<string, object?>>();
        foreach (var (fc, rings) in water)
            foreach (var ring in rings)
                if (ring.Count >= 3) { ringsFlat.Add(ring); attrs.Add(D(("fcode", fc))); }
        PolylineBin.Write(Path.Combine(OutDir, "nhd_water_ne.bin"), ringsFlat, ctx);
        Save(Path.Combine(OutDir, "nhd_water_ne.attrs.json"), attrs, indent: 0);

        ctx.Log("[survey] baking the survey land mask (GSHHG base + NHD carve, 4096^2 z14 frame)");
        Gis.FetchZip(ctx);
        List<(int, List<(double, double)>)> coastF;
        using (var zf = ZipFile.OpenRead(Gis.ZipPath))
            coastF = Gis.ParseGshhg(Gis.Member(zf, "gshhs_f.b"), Gis.Ne.Lon0, Gis.Ne.Lon1, Gis.Ne.Lat0, Gis.Ne.Lat1);
        var mask = Gis.BakeMask(coastF, 4096, 4096, Gis.WinRowLat, Gis.WinLonCol, Gis.WinLon0, Gis.WinLon1);
        var before = mask.Count(b => b != 0);
        var carved = CarveWater(mask, 4096, 4096, water, Gis.WinRowLat, Gis.WinLonCol, Gis.WinLon0, Gis.WinLon1);
        var after = mask.Count(b => b != 0);
        ctx.Log($"[survey] carve: {before} -> {after} land texels ({carved} cleared to water)");
        File.WriteAllBytes(Path.Combine(OutDir, "landmask_ne.raw"), mask);

        // proof: the mask at quarter size (a 4x4 box mean), the Python's PIL resize
        var proof = new byte[1024 * 1024];
        for (var y = 0; y < 1024; y++)
            for (var x = 0; x < 1024; x++)
            {
                var s = 0;
                for (var dy = 0; dy < 4; dy++) for (var dx = 0; dx < 4; dx++) s += mask[(y * 4 + dy) * 4096 + x * 4 + dx];
                proof[y * 1024 + x] = (byte)(s / 16);
            }
        Png.WriteGray(Path.Combine("proofs", "survey_mask.png"), proof, 1024, 1024);
        ctx.Log("[survey] proof -> proofs/survey_mask.png");

        Save(Path.Combine(OutDir, "survey.json"), D(
            ("cusp", D(("url", CuspUrl), ("cache", cuspZip), ("crs", "NAD83 (~1 m vs WGS84, documented not corrected)"), ("datum", "Mean High Water"), ("polylines_ne", cusp.Count))),
            ("nhd", D(("hu4", NhdHu4.ToList()), ("url_pattern", NhdUrl.Replace("{0}", "{hu}")), ("cache_dir", nhdDir), ("crs", "NAD83 EPSG:4269"), ("rivers_ne", rivers.Count), ("water_polys_ne", water.Count))),
            ("focus_box", Focus.ToJson()),
            ("landmask", D(("base", "GSHHG full-res parity (harvest_gis)"), ("carve", "NHD open-water even-odd per feature"), ("land_texels_before", before), ("land_texels_after", after))),
            ("open_water_fcodes", "390xx 436xx 49300 460xx 44500 336xx 43100 (marsh/wetland EXCLUDED: the live tide owns that call)")));
        ctx.Log("[survey] done -> data/gis/survey.json");
        return 0;
    }
}
