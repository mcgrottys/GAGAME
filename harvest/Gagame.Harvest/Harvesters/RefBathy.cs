using System.Globalization;
using System.Text.RegularExpressions;
using static Gagame.Harvest.JsonUtil;

namespace Gagame.Harvest.Harvesters;

/// The vqview-inlet 1.5 m bathymetry composite as a GAGAME height rung (harvest_refbathy.py):
/// a .npy read without numpy, the MLLW -> NAVD88 offset resolved from data/tides/stations.json
/// the way main.cpp's ResolveDatum does, an FNV-1a content identity in the sidecar.
public sealed class RefBathy : IHarvester
{
    public string Name => "refbathy";
    public string Title => "Reference bed (vqview composite)";
    public string What => "The vqview-inlet 1.5 m eHydro + BlueTopo + CUDEM composite shifted to NAVD88, the proven first rung of the inlet bed.";
    public string[] Outputs => ["data/bathy/inlet15.json"];
    public Source[] Sources => [new("vqview-inlet (the owner's project)", "https://github.com/mcgrottys")];
    public Insert? Insert => null;
    public string? Note => "Reads bathy_comp2.npy from the vqview checkout (REFBATHY); not a download.";

    const double Lon0 = -70.8320, Lat1 = 42.8250, Lon1 = -70.7980, Lat0 = 42.8100;
    const float NoData = -9999.0f;

    static (float[] grid, int nx, int ny) LoadNpy(string path)
    {
        var b = File.ReadAllBytes(path);
        if (!(b[0] == 0x93 && b[1] == 'N' && b[2] == 'U' && b[3] == 'M' && b[4] == 'P' && b[5] == 'Y')) throw new InvalidDataException("not a .npy file");
        int major = b[6];
        int hlen; int p;
        if (major == 1) { hlen = Bin.U16(b, 8); p = 10; }
        else { hlen = (int)Bin.U32(b, 8); p = 12; }
        var hdr = System.Text.Encoding.Latin1.GetString(b, p, hlen);
        p += hlen;
        if (Regex.IsMatch(hdr, @"'fortran_order'\s*:\s*True")) throw new InvalidDataException("fortran order unsupported");
        var dt = Regex.Match(hdr, @"'descr'\s*:\s*'([^']+)'").Groups[1].Value;
        var sh = Regex.Match(hdr, @"'shape'\s*:\s*\((\d+)\s*,\s*(\d+)\s*,?\)");
        int ny = int.Parse(sh.Groups[1].Value), nx = int.Parse(sh.Groups[2].Value);
        var grid = new float[ny * nx];
        if (dt is "<f4" or "|f4" or "f4")
            for (var i = 0; i < grid.Length; i++) grid[i] = Bin.F32(b, p + 4 * i);
        else if (dt is "<f8" or "|f8" or "f8")
            for (var i = 0; i < grid.Length; i++) grid[i] = (float)Bin.F64(b, p + 8 * i);
        else throw new InvalidDataException($"dtype {dt} unsupported");
        return (grid, nx, ny);
    }

    (double off, string how) ResolveMllwMinusNavd(HarvestContext ctx)
    {
        var p = Path.Combine("data", "tides", "stations.json");
        var d = TryLoad(p);
        if (d is null)
        {
            ctx.Log($"WARNING: {p} missing; using -1.400 m (Boston-transfer value)");
            return (-1.400, "fallback constant (stations.json absent)");
        }
        var stations = d.Arr("stations").ToList();
        var focus = stations.FirstOrDefault(s => s.Str("id") == "8440452") ?? stations.FirstOrDefault(s => s.Str("id") == "8440466");
        if (focus is not null && focus.NumOrNull("mllw_minus_navd_m") is double link) return (link, $"station {focus.Str("id")} published link");
        (double delta, string id)? best = null;
        foreach (var s in stations)
        {
            var mn = s.NumOrNull("mllw_minus_navd_m");
            var mm = s.NumOrNull("mean_mllw_m");
            if (mn is null || mm is null) continue;
            var delta = -mn.Value - mm.Value;
            if (best is null || Math.Abs(delta) < Math.Abs(best.Value.delta)) best = (delta, s.Str("id")!);
        }
        if (best is null)
        {
            ctx.Log("WARNING: no station carries a NAVD link; using -1.400 m");
            return (-1.400, "fallback constant (no NAVD links)");
        }
        var mmF = focus.Num("mean_mllw_m");
        var off = -(mmF + best.Value.delta);
        return (off, $"MSL transfer via {best.Value.id} (delta {best.Value.delta:+0.000;-0.000}, focus mean_mllw {mmF:0.000})");
    }

    public int Run(HarvestContext ctx, string[] argv)
    {
        var a = new Args(argv);
        var offsetExtra = a.Num("--offset", 0.0);
        var src = Environment.GetEnvironmentVariable("REFBATHY") ?? @"C:\vqview-inlet\vqview-inlet\renders\geotiff\bathy_comp2.npy";
        if (!File.Exists(src)) { ctx.Log($"source not found: {src}"); return 1; }
        var rawBytes = File.ReadAllBytes(src);
        var content = Bin.Fnv1a(rawBytes);
        if (ctx.HasDataCache)
        {
            var keep = ctx.Big("refbathy");
            Directory.CreateDirectory(keep);
            var dst = Path.Combine(keep, "bathy_comp2.npy");
            if (!File.Exists(dst)) { File.Copy(src, dst); ctx.Log($"provenance copy -> {dst}"); }
        }
        var (grid, nx, ny) = LoadNpy(src);
        var (mllwMinusNavd, how) = ResolveMllwMinusNavd(ctx);
        var off = mllwMinusNavd + offsetExtra;
        ctx.Log($"grid {nx}x{ny}  MLLW->NAVD88 offset {off:+0.000;-0.000} m ({how}){(offsetExtra != 0 ? $"  + residual {offsetExtra:+0.000;-0.000}" : "")}");
        double vmin = 1e9, vmax = -1e9;
        var nNodata = 0;
        for (var i = 0; i < grid.Length; i++)
        {
            var v = grid[i];
            if (float.IsNaN(v) || v < -9000.0f) { grid[i] = NoData; nNodata++; }
            else
            {
                v = (float)(v + off);
                grid[i] = v;
                vmin = Math.Min(vmin, v); vmax = Math.Max(vmax, v);
            }
        }
        var outDir = Path.Combine("data", "bathy");
        Directory.CreateDirectory(outDir);
        var f32 = Path.Combine(outDir, "inlet15.f32");
        Bin.WriteF32(f32, grid);
        var meta = D(("file", "inlet15.f32"), ("nx", nx), ("ny", ny), ("lon0", Lon0), ("lat1", Lat1),
            ("dlon", (Lon1 - Lon0) / nx), ("dlat", (Lat1 - Lat0) / ny), ("row0", "north"), ("nodata", (double)NoData),
            ("min_m", Round(vmin, 3)), ("max_m", Round(vmax, 3)),
            ("source", $"vqview-inlet bathy_comp2.npy (USACE eHydro + NOAA BlueTopo + CUDEM composite, 1.5 m, MLLW) shifted {off:+0.000;-0.000} m to NAVD88 ({how}); content fnv1a {content:x16}"),
            ("content_fnv64", content.ToString("x16")), ("datum_offset_applied_m", Round(off, 4)), ("nodata_cells", nNodata), ("generated_utc", NowStampZ()));
        Save(Path.Combine(outDir, "inlet15.json"), meta);
        ctx.Log($"wrote {f32} ({nx} x {ny}, {vmin:0.00}..{vmax:0.00} m NAVD88, {nNodata} nodata)");
        var span = Math.Max(vmax - vmin, 1e-6);
        var pix = new byte[nx * ny];
        for (var i = 0; i < pix.Length; i++)
        {
            var v = grid[i];
            pix[i] = v == NoData ? (byte)0 : (byte)Math.Max(0, Math.Min(255, (int)((v - vmin) / span * 255)));
        }
        Png.WriteGray(Path.Combine(outDir, "inlet15_preview.png"), pix, nx, ny);
        ctx.Log($"preview -> {Path.Combine(outDir, "inlet15_preview.png")}");
        return 0;
    }
}
