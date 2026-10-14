using System.Globalization;
using Gagame.Harvest.Readers;
using static Gagame.Harvest.JsonUtil;

namespace Gagame.Harvest.Harvesters;

/// M6: the planet (harvest_globe.py). ETOPO 2022 relief decimated to 8192 x 4096, the New
/// England 15-arc-second window, MOLA for Mars, and the LIVE sea: GFS-Wave Hs + wind, the
/// 10 m wind vector, isobaric cloud fraction, the gap-filled ocean-colour trio, sea ice.
public sealed class Globe : IHarvester
{
    public string Name => "globe";
    public string Title => "Globe (ETOPO + live sea)";
    public string What => "ETOPO 2022 relief decimated to 8192 x 4096, today's GFS-Wave Hs and wind, ocean colour, ice, clouds, Mars.";
    public string[] Outputs => ["data/globe/globe.json", "data/globe/etopo_8192.i16"];
    public Source[] Sources => [new("ETOPO 2022", "https://www.ncei.noaa.gov/products/etopo-global-relief-model"), new("GFS-Wave on AWS", "https://registry.opendata.aws/noaa-gfs-bdp-pds/")];
    public Insert? Insert => new(InsertKind.Flags, "Refresh only some sections: tick what to SKIP.",
        Flags: ["--skip-etopo", "--skip-mars", "--skip-windvec", "--skip-clouds", "--skip-ocean", "--skip-ice", "--skip-waves"]);

    static readonly string[] EtopoUrls =
    [
        "https://www.ngdc.noaa.gov/mgg/global/relief/ETOPO2022/data/60s/60s_surface_elev_gtif/ETOPO_2022_v1_60s_N90W180_surface.tif",
        "https://www.ngdc.noaa.gov/thredds/fileServer/global/ETOPO2022/60s/60s_surface_elev_gtif/ETOPO_2022_v1_60s_N90W180_surface.tif",
    ];
    const int Onx = 8192, Ony = 4096;
    static readonly string[] MolaUrls = ["https://pds-geosciences.wustl.edu/mgs/mgs-m-mola-5-megdr-l3-v1/mgsl_300x/meg016/megt90n000eb.img"];
    static readonly (int mb, int alt)[] CloudLevels = [(1000, 110), (925, 760), (850, 1460), (700, 3010), (600, 4200), (500, 5570), (400, 7180), (300, 9160), (250, 10360), (200, 11780)];
    const string Erddap = "https://coastwatch.noaa.gov/erddap/griddap";
    static readonly (string ds, string var, string file, bool log)[] OceanFields =
    [
        ("noaacwNPPN20S3ASCIDINEOFDaily", "chlor_a", "oc_chl.f32", true),
        ("noaacwNPPN20S3AkdSCIDINEOFDaily", "kd_490", "oc_kd490.f32", false),
        ("noaacwNPPN20S3AspmSCIDINEOFDaily", "spm", "oc_spm.f32", true),
    ];
    const int OcStride = 3;
    const float OcNull = -9.0f;

    HarvestContext _ctx = null!;
    SimpleFetcher _fetch = null!;
    string Cache => Path.Combine(_ctx.Root, "cache", "globe");
    string Out => Path.Combine(_ctx.Root, "data", "globe");

    public int Run(HarvestContext ctx, string[] argv)
    {
        _ctx = ctx;
        _fetch = new SimpleFetcher(ctx, 0.0, 1);
        var skip = argv.ToHashSet();
        var meta = new Dictionary<string, object?>();
        void Merge(Dictionary<string, object?> m) { foreach (var (k, v) in m) meta[k] = v; }
        if (!skip.Contains("--skip-etopo")) { ctx.Log("[etopo] global relief"); Merge(DoEtopo()); }
        if (!skip.Contains("--skip-mars"))
        {
            ctx.Log("[mars] MOLA relief");
            try { Merge(DoMars()); }
            catch (Exception e) { ctx.Log($"  MOLA unavailable ({e.Message}); Mars stays a textured sphere"); }
        }
        if (!skip.Contains("--skip-windvec")) { ctx.Log("[windvec] global 10 m wind vector"); Merge(DoWindVec()); }
        if (!skip.Contains("--skip-waves")) { ctx.Log("[waves] live global sea state"); Merge(DoWaves()); }
        if (!skip.Contains("--skip-clouds")) { ctx.Log("[clouds] live cloud layers"); Merge(DoClouds()); }
        if (!skip.Contains("--skip-ocean")) { ctx.Log("[ocean] gap-filled ocean colour (chl / Kd490 / SPM)"); Merge(DoOceanColour()); }
        if (!skip.Contains("--skip-ice")) { ctx.Log("[ice] live sea-ice concentration"); Merge(DoSeaIce()); }
        meta["generated_utc"] = NowIso();
        var path = Path.Combine(Out, "globe.json");
        var old = TryLoad(path) is { } node ? (Dictionary<string, object?>)Plain(node)! : new Dictionary<string, object?>();
        foreach (var (k, v) in meta) old[k] = v;
        Directory.CreateDirectory(Out);
        Save(path, old);
        ctx.Log($"wrote {path}");
        return 0;
    }

    // ------------------------------------------------------------------ relief

    Dictionary<string, object?> DoEtopo()
    {
        var tifPath = Path.Combine(Cache, "ETOPO_2022_v1_60s_surface.tif");
        _fetch.DownloadBig(EtopoUrls, tifPath, tag: "ETOPO");
        _ctx.Log("  decoding + decimating (one-time; a minute of number crunching)...");
        var g = new GeoTiff(File.ReadAllBytes(tifPath));
        _ctx.Log($"  source {g.Width}x{g.Height}, compression {g.Compression}, predictor {g.Predictor}");
        var colEdges = new int[Onx + 1];
        for (var ox = 0; ox <= Onx; ox++) colEdges[ox] = (int)((long)ox * g.Width / Onx);
        var acc = new double[Onx * Ony];
        var cnt = new uint[Onx * Ony];
        var band = g.Tiled ? 512 : Math.Max(1, g.Th);
        var t0 = _ctx.Seconds;
        for (var sy0 = 0; sy0 < g.Height; sy0 += band)
        {
            var rows = Math.Min(band, g.Height - sy0);
            var block = g.ReadWindow(0, sy0, g.Width, rows);
            for (var ry = 0; ry < rows; ry++)
            {
                var oy = (int)((long)(sy0 + ry) * Ony / g.Height);
                var baseOut = oy * Onx;
                var baseSrc = ry * g.Width;
                for (var ox = 0; ox < Onx; ox++)
                {
                    int c0 = colEdges[ox], c1 = colEdges[ox + 1];
                    double s = 0;
                    for (var c = c0; c < c1; c++) s += block[baseSrc + c];
                    acc[baseOut + ox] += s;
                    cnt[baseOut + ox] += (uint)(c1 - c0);
                }
            }
            if ((sy0 / band) % 4 == 0) _ctx.Log($"    {100.0 * (sy0 + rows) / g.Height,5:0.0}%  ({_ctx.Seconds - t0:0} s)");
        }
        var out_ = new short[Onx * Ony];
        for (var i = 0; i < out_.Length; i++)
            if (cnt[i] != 0) out_[i] = (short)Math.Max(-32000, Math.Min(32000, (int)Math.Round(acc[i] / cnt[i], MidpointRounding.ToEven)));
        Directory.CreateDirectory(Out);
        var p = Path.Combine(Out, "etopo_8192.i16");
        Bin.WriteI16(p, out_);
        _ctx.Log($"  wrote data/globe/etopo_8192.i16 ({Onx}x{Ony} int16, {new FileInfo(p).Length >> 20} MB)");
        return D(("relief_file", "etopo_8192.i16"), ("nx", Onx), ("ny", Ony), ("lon0", -180.0), ("lat0", 90.0), ("dlon", 360.0 / Onx), ("dlat", -180.0 / Ony),
            ("source", "ETOPO 2022 v1 60s ice-surface (NOAA NCEI), box-decimated"));
    }

    Dictionary<string, object?> DoMars()
    {
        var imgPath = Path.Combine(Cache, "megt90n000eb.img");
        _fetch.DownloadBig(MolaUrls, imgPath, tag: "MOLA");
        var raw = File.ReadAllBytes(imgPath);
        const int nx = 5760, ny = 2880;
        if (raw.Length != nx * ny * 2) throw new InvalidOperationException($"MEGDR size {raw.Length} != {nx * ny * 2}; layout assumption broken");
        var vals = new short[nx * ny];
        for (var i = 0; i < vals.Length; i++) vals[i] = (short)((raw[2 * i] << 8) | raw[2 * i + 1]);
        var rolled = new short[nx * ny];
        var half = nx / 2;
        for (var y = 0; y < ny; y++)
        {
            Array.Copy(vals, y * nx + half, rolled, y * nx, half);
            Array.Copy(vals, y * nx, rolled, y * nx + half, half);
        }
        Directory.CreateDirectory(Out);
        Bin.WriteI16(Path.Combine(Out, "mars_relief.i16"), rolled);
        _ctx.Log($"  wrote data/globe/mars_relief.i16 ({nx}x{ny} int16, range {rolled.Min()}..{rolled.Max()} m)");
        return D(("mars_file", "mars_relief.i16"), ("mars_nx", nx), ("mars_ny", ny), ("mars_lon0", 0.0), ("mars_lat0", 90.0), ("mars_source", "MOLA MEGDR 16ppd (PDS Geosciences)"));
    }

    // ------------------------------------------------------------------ the live sea and sky

    /// One grib-filter GET, cached when it came back large enough.
    byte[] GribCached(string url, string cache, string say)
    {
        if (!(File.Exists(cache) && new FileInfo(cache).Length > 10000))
        {
            _ctx.Log($"  GET {say}");
            var data = _fetch.Download(url, 120);
            if (data.Length < 10000) throw new InvalidOperationException($"short response ({data.Length} B)");
            Directory.CreateDirectory(Cache);
            File.WriteAllBytes(cache, data);
        }
        return File.ReadAllBytes(cache);
    }

    static float[] Values(Grib2Message msg, float dflt, Func<double, double>? clamp = null)
    {
        var vals = new float[msg.Ni * msg.Nj];
        Array.Fill(vals, dflt);
        for (var i = 0; i < vals.Length; i++)
        {
            var v = msg.Values[i];
            if (!double.IsNaN(v)) vals[i] = (float)(clamp is null ? v : clamp(v));
        }
        if ((msg.Scan & 0x40) != 0)
        {
            var flipped = new float[vals.Length];
            for (var r = 0; r < msg.Nj; r++) Array.Copy(vals, (msg.Nj - 1 - r) * msg.Ni, flipped, r * msg.Ni, msg.Ni);
            vals = flipped;
        }
        return vals;
    }

    static double LatTop(Grib2Message m) => (m.Scan & 0x40) != 0 ? m.Lat1 + (m.Nj - 1) * m.Dlat : m.Lat1;

    IEnumerable<(string ymd, string cyc)> Cycles()
    {
        var now = DateTime.UtcNow;
        for (var lagH = 5; lagH < 30; lagH += 6)
        {
            var t = now.AddHours(-lagH);
            yield return (t.ToString("yyyyMMdd", CultureInfo.InvariantCulture), ((t.Hour / 6) * 6).ToString("00"));
        }
    }

    Dictionary<string, object?> DoWaves()
    {
        foreach (var (ymd, cyc) in Cycles())
        {
            var url = "https://nomads.ncep.noaa.gov/cgi-bin/filter_gfswave.pl?" +
                      $"dir=%2Fgfs.{ymd}%2F{cyc}%2Fwave%2Fgridded&file=gfswave.t{cyc}z.global.0p25.f000.grib2&" +
                      "var_HTSGW=on&var_WIND=on&all_lev=on&subregion=&toplat=90&bottomlat=-90&leftlon=0&rightlon=360";
            var cache = Path.Combine(Cache, $"gfswave_{ymd}_{cyc}_f000_hs_wind.grib2");
            try
            {
                var msgs = Grib2.ReadMessages(GribCached(url, cache, $"gfswave {ymd} {cyc}z f000 (global Hs + wind)"));
                var hs = msgs.First(m => m.Discipline == 10);
                var wind = msgs.FirstOrDefault(m => m.Discipline == 0);
                _ctx.Log($"  {ymd} {cyc}z: Hs grid {hs.Ni}x{hs.Nj}, lat1 {hs.Lat1}, lon1 {hs.Lon1}, scan 0x{hs.Scan:x}; wind {(wind is not null ? "yes" : "NO")}");
                Directory.CreateDirectory(Out);
                Bin.WriteF32(Path.Combine(Out, "hs.f32"), Values(hs, -1.0f));
                if (wind is not null) Bin.WriteF32(Path.Combine(Out, "wind.f32"), Values(wind, -1.0f));
                return D(("waves_cycle", $"{ymd}t{cyc}z"), ("waves_nx", hs.Ni), ("waves_ny", hs.Nj), ("waves_lat1", LatTop(hs)), ("waves_lon1", hs.Lon1),
                    ("waves_dlat", hs.Dlat), ("waves_dlon", hs.Dlon), ("waves_scan", 0), ("hs_file", "hs.f32"), ("wind_file", wind is not null ? "wind.f32" : null));
            }
            catch (Exception e) { _ctx.Log($"    cycle {ymd} {cyc}z unavailable ({e.Message}); stepping back"); }
        }
        _ctx.Log("  NO wave cycle reachable -- the globe will render relief without a live sea");
        return new();
    }

    Dictionary<string, object?> DoClouds()
    {
        var levParams = string.Concat(CloudLevels.Select(l => $"&lev_{l.mb}_mb=on"));
        foreach (var (ymd, cyc) in Cycles())
            foreach (var fhr in new[] { "f000", "f003" })
            {
                var url = "https://nomads.ncep.noaa.gov/cgi-bin/filter_gfs_0p50.pl?" +
                          $"dir=%2Fgfs.{ymd}%2F{cyc}%2Fatmos&file=gfs.t{cyc}z.pgrb2full.0p50.{fhr}&var_TCDC=on{levParams}&" +
                          "subregion=&toplat=90&bottomlat=-90&leftlon=0&rightlon=360";
                var cache = Path.Combine(Cache, $"gfs_{ymd}_{cyc}_{fhr}_tcdc_iso.grib2");
                try
                {
                    var msgs = Grib2.ReadMessages(GribCached(url, cache, $"gfs {ymd} {cyc}z {fhr} (isobaric cloud fraction)"));
                    var byLev = new Dictionary<int, Grib2Message>();
                    foreach (var m in msgs)
                        if (m.Discipline == 0 && m.ParamCategory == 6 && m.LevelType == 100) byLev[(int)(m.LevelValue / 100)] = m;
                    var missing = CloudLevels.Where(l => !byLev.ContainsKey(l.mb)).Select(l => l.mb).ToList();
                    if (missing.Count > 0) throw new InvalidOperationException($"levels missing: [{string.Join(", ", missing)}]");
                    var g0 = byLev[CloudLevels[0].mb];
                    _ctx.Log($"  gfs {ymd} {cyc}z {fhr}: {CloudLevels.Length} isobaric cloud levels, grid {g0.Ni}x{g0.Nj}, lat1 {g0.Lat1}, scan 0x{g0.Scan:x}");
                    Directory.CreateDirectory(Out);
                    using (var f = File.Create(Path.Combine(Out, "cloud_iso.f32")))
                        foreach (var (mb, _) in CloudLevels) f.Write(System.Runtime.InteropServices.MemoryMarshal.AsBytes<float>(Values(byLev[mb], 0.0f)));
                    return D(("clouds_cycle", $"{ymd}t{cyc}z{fhr}"), ("clouds_nx", g0.Ni), ("clouds_ny", g0.Nj), ("clouds_nz", CloudLevels.Length),
                        ("clouds_lat1", LatTop(g0)), ("clouds_lon1", g0.Lon1), ("clouds_dlat", g0.Dlat), ("clouds_dlon", g0.Dlon),
                        ("clouds_alt_m", CloudLevels.Select(l => l.alt).ToList()), ("clouds_file", "cloud_iso.f32"));
                }
                catch (Exception e) { _ctx.Log($"    {ymd} {cyc}z {fhr} unavailable ({e.Message})"); }
            }
        _ctx.Log("  NO cloud cycle reachable -- the sky stays clear");
        return new();
    }

    Dictionary<string, object?> DoWindVec()
    {
        foreach (var (ymd, cyc) in Cycles())
        {
            var url = "https://nomads.ncep.noaa.gov/cgi-bin/filter_gfs_0p50.pl?" +
                      $"dir=%2Fgfs.{ymd}%2F{cyc}%2Fatmos&file=gfs.t{cyc}z.pgrb2full.0p50.f000&" +
                      "var_UGRD=on&var_VGRD=on&lev_10_m_above_ground=on&subregion=&toplat=90&bottomlat=-90&leftlon=0&rightlon=360";
            var cache = Path.Combine(Cache, $"gfs_{ymd}_{cyc}_f000_uv10.grib2");
            try
            {
                var msgs = Grib2.ReadMessages(GribCached(url, cache, $"gfs {ymd} {cyc}z f000 (10 m wind vector)"));
                var u = msgs.First(m => m.ParamCategory == 2 && m.ParamNumber == 2);
                var v = msgs.First(m => m.ParamCategory == 2 && m.ParamNumber == 3);
                _ctx.Log($"  gfs {ymd} {cyc}z: u/v grid {u.Ni}x{u.Nj}, scan 0x{u.Scan:x}");
                Directory.CreateDirectory(Out);
                Bin.WriteF32(Path.Combine(Out, "wind_u.f32"), Values(u, 0.0f));
                Bin.WriteF32(Path.Combine(Out, "wind_v.f32"), Values(v, 0.0f));
                return D(("windvec_cycle", $"{ymd}t{cyc}z"), ("windvec_nx", u.Ni), ("windvec_ny", u.Nj), ("windvec_lat1", LatTop(u)), ("windvec_lon1", u.Lon1), ("windvec_dlat", u.Dlat), ("windvec_dlon", u.Dlon));
            }
            catch (Exception e) { _ctx.Log($"    {ymd} {cyc}z unavailable ({e.Message})"); }
        }
        return new();
    }

    Dictionary<string, object?> DoOceanColour()
    {
        Directory.CreateDirectory(Cache);
        Directory.CreateDirectory(Out);
        var meta = new Dictionary<string, object?>();
        foreach (var (ds, var, outName, asLog) in OceanFields)
        {
            var q = $"{var}%5B(last)%5D%5B(0.0)%5D%5B(89.9583):{OcStride}:(-89.9583)%5D%5B(-179.9583):{OcStride}:(179.9583)%5D";
            var cache = Path.Combine(Cache, $"oc_{var}_s{OcStride}.nc");
            try
            {
                if (!(File.Exists(cache) && new FileInfo(cache).Length > 100000))
                {
                    _ctx.Log($"  GET {ds} {var} (global, stride {OcStride})");
                    var data = _fetch.Download($"{Erddap}/{ds}.nc?{q}", 300);
                    if (data.Length < 100000) throw new InvalidOperationException($"short response ({data.Length} B)");
                    File.WriteAllBytes(cache, data);
                    Thread.Sleep(2000);
                }
                var nc = new NetCdf3(File.ReadAllBytes(cache));
                var (lat, _) = nc.Read("latitude");
                var (lon, _) = nc.Read("longitude");
                var (vals, _) = nc.Read(var);
                var atts = nc.Vars[var].Atts;
                var vmin = AttDouble(atts, "valid_min", 0.0);
                var vmax = AttDouble(atts, "valid_max", 1e30);
                int nx = lon.Length, ny = lat.Length;
                if (nx * ny != vals.Length) throw new InvalidOperationException($"{nx}x{ny} != {vals.Length} values");
                var out_ = new float[nx * ny];
                Array.Fill(out_, OcNull);
                var kept = 0;
                for (var i = 0; i < vals.Length; i++)
                {
                    var v = vals[i];
                    if (!double.IsNaN(v) && vmin <= v && v <= vmax) { out_[i] = (float)(asLog ? Math.Log10(v) : v); kept++; }
                }
                Bin.WriteF32(Path.Combine(Out, outName), out_);
                _ctx.Log($"  {var}: {nx}x{ny} lat {lat[0]:0.000}..{lat[^1]:0.000} d={lat[1] - lat[0]:+0.0000;-0.0000}, {kept * 100.0 / (nx * ny):0.0}% retrieved -> {outName}{(asLog ? " (log10)" : "")}");
                if (meta.Count == 0)
                    meta = D(("oc_nx", nx), ("oc_ny", ny), ("oc_lat1", lat[0]), ("oc_lon1", lon[0]), ("oc_dlat", lat[1] - lat[0]), ("oc_dlon", lon[1] - lon[0]),
                        ("oc_null", (double)OcNull), ("oc_source", "NOAA CoastWatch gap-filled DINEOF"));
                meta[$"{var}_file"] = outName;
            }
            catch (Exception e) { _ctx.Log($"  {var} unavailable ({e.Message}); the optics fall back to the pure-water limit"); }
        }
        if (meta.ContainsKey("chlor_a_file"))
        {
            try
            {
                var (t, _) = new NetCdf3(File.ReadAllBytes(Path.Combine(Cache, $"oc_kd_490_s{OcStride}.nc"))).Read("time");
                meta["oc_epoch_utc"] = FromUnix(t[0]).ToString("yyyy-MM-dd", CultureInfo.InvariantCulture);
            }
            catch (Exception) { }
        }
        return meta;
    }

    static double AttDouble(Dictionary<string, object?> atts, string k, double dflt) => atts.GetValueOrDefault(k) switch
    {
        double d => d,
        List<double> l when l.Count > 0 => l[0],
        string s when double.TryParse(s, NumberStyles.Float, CultureInfo.InvariantCulture, out var v) => v,
        _ => dflt,
    };

    Dictionary<string, object?> DoSeaIce()
    {
        foreach (var (ymd, cyc) in Cycles())
        {
            var url = "https://nomads.ncep.noaa.gov/cgi-bin/filter_gfs_0p25.pl?" +
                      $"dir=%2Fgfs.{ymd}%2F{cyc}%2Fatmos&file=gfs.t{cyc}z.pgrb2.0p25.f000&var_ICEC=on&lev_surface=on&" +
                      "subregion=&toplat=90&bottomlat=-90&leftlon=0&rightlon=360";
            var cache = Path.Combine(Cache, $"gfs_{ymd}_{cyc}_f000_icec.grib2");
            try
            {
                var msgs = Grib2.ReadMessages(GribCached(url, cache, $"gfs {ymd} {cyc}z f000 (sea ice)"));
                var ice = msgs.First(m => m.Discipline == 10);
                var vals = Values(ice, 0.0f, v => Math.Min(Math.Max(v, 0.0), 1.0));
                Bin.WriteF32(Path.Combine(Out, "ice.f32"), vals);
                var cov = vals.Count(v => v > 0.15f) * 100.0 / vals.Length;
                _ctx.Log($"  {ymd} {cyc}z: ice {ice.Ni}x{ice.Nj}, {cov:0.00}% of the globe over 15% concentration -> ice.f32");
                return D(("ice_cycle", $"{ymd}t{cyc}z"), ("ice_nx", ice.Ni), ("ice_ny", ice.Nj), ("ice_file", "ice.f32"));
            }
            catch (Exception e) { _ctx.Log($"    cycle {ymd} {cyc}z unavailable ({e.Message}); stepping back"); }
        }
        _ctx.Log("  NO ice cycle reachable -- the poles render as open water");
        return new();
    }
}
