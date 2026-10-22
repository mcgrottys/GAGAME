using System.Globalization;
using System.Text.RegularExpressions;
using Gagame.Harvest.Readers;
using static Gagame.Harvest.JsonUtil;

namespace Gagame.Harvest.Harvesters;

/// The NOS Operational Forecast Systems on the NODD bucket, for the composites: the cycle listing, the vertical-datum grids, and a ROMS nowcast's surface current
/// rotated to east/north and scattered onto a regular lon/lat grid.
public static class Ofs
{
    public const string Bucket = "noaa-nos-ofs-pds";
    public static string S3Dir => Path.Combine("cache", "places", "_s3");

    public static Polite.S3Object? Vdatums(Polite polite, string ofs) =>
        GridDatum(polite).FirstOrDefault(o => o.Key.EndsWith($"/{ofs}_vdatums.nc"));

    /// The OFS_Grid_Datum/ listing, the first copy cached.
    public static List<Polite.S3Object> GridDatum(Polite polite)
    {
        Directory.CreateDirectory(S3Dir);
        var old = Directory.GetFiles(S3Dir, "ofs_griddatum*").Select(Path.GetFileName).OrderBy(n => n, StringComparer.Ordinal).ToList();
        var cache = Path.Combine(S3Dir, old.Count > 0 ? old[^1]! : "ofs_griddatum.xml");
        return polite.S3List(Bucket, "OFS_Grid_Datum/", cache).objects;
    }

    /// The newest complete nowcast cycle (six fields files + the stations file) in the three days
    /// before `now`.
    public static (string ymd, int cyc, Dictionary<string, long> files, string prefix) LatestCycle(Polite polite, string ofs, DateTime now)
    {
        for (var back = 0; back < 3; back++)
        {
            var day = now.AddDays(-back);
            var prefix = $"{ofs}/netcdf/{day:yyyy/MM/dd}/";
            Directory.CreateDirectory(S3Dir);
            var cache = Path.Combine(S3Dir, $"ofs_{ofs}_{day:yyyyMMdd}_{now:yyyyMMddHH}.xml");
            var older = Directory.GetFiles(S3Dir, $"ofs_{ofs}_{day:yyyyMMdd}_*").Select(Path.GetFileName).OrderBy(n => n, StringComparer.Ordinal).ToList();
            if (older.Count > 0 && back > 0) cache = Path.Combine(S3Dir, older[^1]!);
            var (objs, _) = polite.S3List(Bucket, prefix, cache);
            var sizes = objs.ToDictionary(o => o.Key[(o.Key.LastIndexOf('/') + 1)..], o => o.Size);
            foreach (var cyc in new[] { 18, 12, 6, 0 })
            {
                var names = Enumerable.Range(1, 6).Select(i => $"{ofs}.t{cyc:00}z.{day:yyyyMMdd}.fields.n{i:000}.nc").ToList();
                names.Add($"{ofs}.t{cyc:00}z.{day:yyyyMMdd}.stations.nowcast.nc");
                if (names.All(sizes.ContainsKey)) return ($"{day:yyyyMMdd}", cyc, names.ToDictionary(n => n, n => sizes[n]), prefix);
            }
        }
        throw new InvalidOperationException($"no complete {ofs} nowcast cycle in the last three days");
    }

    /// THE SURFACE FILE of a cycle, by one order: a nowcast before a forecast (the nowcast is the
    /// model's analysis of the moment), its LAST hour before its others (the newest analysis), a
    /// forecast's FIRST hour (the nearest to the analysis), and the 2-D surface product (2ds)
    /// before the full 3-D fields (the same surface, a fraction of the bytes). Lower is better.
    public static double SurfaceRank(string name)
    {
        var m = Regex.Match(name, @"\.(2ds|fields)\.([nf])(\d{3})\.nc$");
        if (!m.Success) return double.PositiveInfinity;
        var hour = int.Parse(m.Groups[3].Value);
        return (m.Groups[2].Value == "n" ? 0 : 1000) + (m.Groups[2].Value == "n" ? 100 - hour : hour) + (m.Groups[1].Value == "2ds" ? 0 : 0.5);
    }

    /// The best surface file of the newest cycle in the three days before `now`: (key, size, ymd).
    public static (string key, long size, string ymd)? SurfaceFile(Polite polite, string ofs, DateTime now)
    {
        for (var back = 0; back < 3; back++)
        {
            var day = now.AddDays(-back);
            Directory.CreateDirectory(S3Dir);
            var cache = Path.Combine(S3Dir, $"ofs_{ofs}_{day:yyyyMMdd}_{now:yyyyMMddHH}.xml");
            var older = Directory.GetFiles(S3Dir, $"ofs_{ofs}_{day:yyyyMMdd}_*").Select(Path.GetFileName).OrderBy(n => n, StringComparer.Ordinal).ToList();
            if (older.Count > 0) cache = Path.Combine(S3Dir, older[^1]!);
            var (objs, _) = polite.S3List(Bucket, $"{ofs}/netcdf/{day:yyyy/MM/dd}/", cache);
            var best = objs.Where(o => Regex.IsMatch(o.Key, @"\.t\d\dz\.")).GroupBy(o => Regex.Match(o.Key, @"\.t(\d\d)z\.").Groups[1].Value)
                .OrderByDescending(g => g.Key, StringComparer.Ordinal)
                .Select(g => g.Where(o => !double.IsInfinity(SurfaceRank(o.Key))).OrderBy(o => SurfaceRank(o.Key)).FirstOrDefault())
                .FirstOrDefault(o => o is not null);
            if (best is not null) return (best.Key, best.Size, $"{day:yyyyMMdd}");
        }
        return null;
    }

    public static float[] Scatter(Currents.Grid grid, double[] lon, double[] lat, double[] vals, double[]? mask)
    {
        var out_ = new float[grid.Nx * grid.Ny];
        Array.Fill(out_, -999.0f);
        for (var k = 0; k < vals.Length; k++)
        {
            if (mask is not null && !(mask[k] > 0.5)) continue;
            var x = vals[k];
            if (double.IsNaN(x) || Math.Abs(x) > 100) continue;
            var gx = (int)((lon[k] - grid.Lon0) / grid.Dlon);
            var gy = (int)((lat[k] - grid.Lat0) / grid.Dlat);
            for (var oy = 0; oy <= 1; oy++)
                for (var ox = 0; ox <= 1; ox++)
                {
                    int xx = gx + ox, yy = gy + oy;
                    if (0 <= xx && xx < grid.Nx && 0 <= yy && yy < grid.Ny) out_[yy * grid.Nx + xx] = (float)x;
                }
        }
        return out_;
    }

    /// The median spacing (degrees of latitude) between neighbouring rho points inside a box:
    /// the grain the model resolves there.
    public static double RhoSpacingDeg(double[] lonR, double[] latR, int nx, Box box)
    {
        var d = new List<double>();
        for (var k = 0; k + 1 < lonR.Length; k++)
        {
            if ((k + 1) % nx == 0 || !box.Contains(lonR[k], latR[k])) continue;
            var dx = (lonR[k + 1] - lonR[k]) * Math.Cos(latR[k] * Geo.DegToRad);
            var dy = latR[k + 1] - latR[k];
            d.Add(Math.Sqrt(dx * dx + dy * dy));
        }
        if (d.Count == 0) return double.NaN;
        d.Sort();
        return d[d.Count / 2];
    }

    /// A ROMS nowcast file's surface u/v, rotated to east/north by the grid's `angle`, scattered
    /// onto a regular grid of `step` degrees over the box's wet extent (null = the model's own rho
    /// spacing there), gaps of up to three cells filled. Null when the box holds no rho point.
    public static (Currents.SurfaceField field, Dictionary<string, object?> declared)? Convert(string ofs, string path, Box box, double? step, Action<string> log)
    {
        var name = Path.GetFileName(path);
        using var nc = new NetCdf4(path);
        var un = nc.Has("u_sur") ? "u_sur" : "u";
        var vn = nc.Has("v_sur") ? "v_sur" : "v";
        var need = new[] { un, vn, "lon_u", "lat_u", "lon_v", "lat_v", "mask_u", "mask_v", "angle", "lon_rho", "lat_rho", "ocean_time" };
        var missing = need.Where(n => !nc.Has(n)).ToList();
        if (missing.Count > 0) throw new InvalidOperationException($"{name}: missing [{string.Join(", ", missing)}] (has [{string.Join(", ", nc.Vars.Keys.OrderBy(k => k, StringComparer.Ordinal).Take(40))}])");
        (double[] vals, int ny, int nx) Surface(string v)
        {
            var (vals, shape) = nc.Read(v);
            int ny = shape[^2], nx = shape[^1];
            return (vals[^(ny * nx)..], ny, nx);
        }
        var rd = new[] { "lon_u", "lat_u", "lon_v", "lat_v", "mask_u", "mask_v", "angle", "lon_rho", "lat_rho" }.ToDictionary(n => n, n => nc.Read(n).values);
        double[] lonR = rd["lon_rho"], latR = rd["lat_rho"];
        var wet = Enumerable.Range(0, lonR.Length).Where(k => box.Contains(lonR[k], latR[k])).Select(k => (lonR[k], latR[k])).ToList();
        if (wet.Count == 0) return null;
        var (u, uny, unx) = Surface(un);
        var (v, vny, vnx) = Surface(vn);
        var (t, _) = nc.Read("ocean_time");
        var rhoShape = nc.Shape("lon_rho");
        var model = RhoSpacingDeg(lonR, latR, rhoShape[^1], box);
        var st = step ?? model;
        var lon0 = Math.Max(box.Lon0, wet.Min(p => p.Item1));
        var lon1 = Math.Min(box.Lon1, wet.Max(p => p.Item1));
        var lat0 = Math.Max(box.Lat0, wet.Min(p => p.Item2));
        var lat1 = Math.Min(box.Lat1, wet.Max(p => p.Item2));
        var grid = new Currents.Grid(lon0, lat0, lon0 + ((int)((lon1 - lon0) / st) + 1) * st, lat0 + ((int)((lat1 - lat0) / st) + 1) * st, (int)((lon1 - lon0) / st) + 1, (int)((lat1 - lat0) / st) + 1);
        var gu = Scatter(grid, rd["lon_u"], rd["lat_u"], u, rd["mask_u"]);
        var gv = Scatter(grid, rd["lon_v"], rd["lat_v"], v, rd["mask_v"]);
        var ga = Scatter(grid, lonR, latR, rd["angle"], null);
        foreach (var g in new[] { gu, gv, ga }) Currents.GapFill(grid, g, 3);
        var n = grid.Nx * grid.Ny;
        var outU = new float[n]; var outV = new float[n];
        Array.Fill(outU, -999.0f); Array.Fill(outV, -999.0f);
        var filled = 0;
        for (var k = 0; k < n; k++)
        {
            float uu = gu[k], vv = gv[k], aa = ga[k];
            if (uu > -900 && vv > -900)
            {
                var a = aa > -900 ? aa : 0.0;
                outU[k] = (float)(uu * Math.Cos(a) - vv * Math.Sin(a));
                outV[k] = (float)(uu * Math.Sin(a) + vv * Math.Cos(a));
                filled++;
            }
        }
        var tunits = nc.Attr("ocean_time", "units") ?? "";
        string? valid = null;
        var m = Regex.Match(tunits, @"seconds since (\d{4}-\d{2}-\d{2})[ T](\d{2}:\d{2}:\d{2})");
        if (m.Success)
        {
            var t0 = DateTime.ParseExact(m.Groups[1].Value + " " + m.Groups[2].Value, "yyyy-MM-dd HH:mm:ss", CultureInfo.InvariantCulture, DateTimeStyles.AssumeUniversal | DateTimeStyles.AdjustToUniversal);
            valid = IsoZ(t0.AddSeconds(t[^1]));
        }
        var declared = D(("file", name), ("ocean_time_units", tunits), ("valid_utc", valid),
            ("u_units", nc.Attr(un, "units")), ("u_long_name", nc.Attr(un, "long_name")), ("v_units", nc.Attr(vn, "units")),
            ("zeta_units", nc.Has("zeta") ? nc.Attr("zeta", "units") : null), ("zeta_long_name", nc.Has("zeta") ? nc.Attr("zeta", "long_name") : null),
            ("angle_units", nc.Attr("angle", "units")),
            ("global", nc.GlobalAttrs("title", "type", "Conventions", "history", "grd_file", "source", "institution").ToDictionary(kv => kv.Key, kv => (object?)kv.Value)),
            ("model_grid", D(("u", new List<int> { uny, unx }), ("v", new List<int> { vny, vnx }), ("rho", rhoShape))),
            ("rho_spacing_deg", Round(model, 6)));
        log($"[ofs] {name}: surface u/v rotated to east/north, {filled} of {n} cells water on a {grid.Nx}x{grid.Ny} grid at {st:0.#####} deg (model rho spacing {model:0.#####}); valid {valid ?? "None"}");
        var field = new Currents.SurfaceField
        {
            File = $"{ofs}_uv.f32", Nx = grid.Nx, Ny = grid.Ny, Lon0 = grid.Lon0, Lat0 = grid.Lat0, Dlon = st, Dlat = st,
            Source = $"{ofs.ToUpperInvariant()} {name} (NODD {Bucket}), surface level", ValidUtc = valid, U = outU, V = outV,
        };
        return (field, declared);
    }

    public static Dictionary<string, object?> VdatumsDeclared(string path)
    {
        var nc = new NetCdf3(File.ReadAllBytes(path));
        static object? Strip(object? s) => s is string str ? str.Trim() : s;
        return D(("global", nc.GAtts.ToDictionary(kv => kv.Key, kv => Strip(kv.Value))),
            ("variables", nc.Vars.ToDictionary(kv => kv.Key, kv => (object?)D(("long_name", Strip(kv.Value.Atts.GetValueOrDefault("long_name"))), ("unit", Strip(kv.Value.Atts.GetValueOrDefault("unit")))))));
    }
}
