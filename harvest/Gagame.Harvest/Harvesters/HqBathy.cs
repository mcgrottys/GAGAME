using System.Globalization;
using System.IO.Compression;
using System.Text.RegularExpressions;
using Gagame.Harvest.Readers;
using Microsoft.Data.Sqlite;
using static Gagame.Harvest.JsonUtil;

namespace Gagame.Harvest.Harvesters;

/// M7z: THE HQ BED (harvest_hqbathy.py). NOAA BlueTopo 4 m UTM19 COGs as the base, USACE
/// eHydro point clouds (MA State Plane usft, MLLW) overlaid as per-cell means at ~1.5 m; both
/// projections pinned before any point is trusted; a datum cross-check against the CUDEM plane
/// before anything is written.
public sealed class HqBathy : IHarvester
{
    public string Name => "hqbathy";
    public string Title => "HQ bed (BlueTopo + eHydro)";
    public string What => "The 1.5 m inlet seafloor from the primary surveys, the bed the wave solve and bedforms read.";
    public string[] Outputs => ["data/bathy/inlethq.f32"];
    public Source[] Sources => [new("NOAA BlueTopo", "https://registry.opendata.aws/noaa-ocs-nationalbathymetry/"), new("USACE eHydro", "https://navigation.usace.army.mil/Survey/Hydro")];
    public Insert? Insert => null;
    public string? Note => "Needs data/bathy/merrimack.* for its datum cross-check: the CUDEM plane the retired `bathy` window harvester wrote (kept on disk; no harvester writes it now).";

    const string S3 = "https://noaa-ocs-nationalbathymetry-pds.s3.amazonaws.com/";
    const string SchemePrefix = "BlueTopo/_BlueTopo_Tile_Scheme/";
    static readonly string[] BtTiles = ["BH4ZW5GK", "BH4ZW5GL", "BH4ZW5GM", "BH4ZX5GK", "BH4ZX5GL", "BH4ZX5GM"];
    const string EhIndex = "https://services7.arcgis.com/n1YM8pTrFmm7L4hs/arcgis/rest/services/eHydro_Survey_Data/FeatureServer/0/query";
    const string EhChannel = "CENAE_MA_01_NEB";
    static readonly string[] EhSurveys = ["20250908_CS_045", "20251007_XC_045"];
    const double Lon0 = -70.8700, Lon1 = -70.7400, Lat0 = 42.7300, Lat1 = 42.8500;
    const double Dlon = 1.5 / 81660.0, Dlat = 1.5 / 110574.0;
    static readonly int NX = (int)Math.Ceiling((Lon1 - Lon0) / Dlon);
    static readonly int NY = (int)Math.Ceiling((Lat1 - Lat0) / Dlat);
    const float NoData = -9999.0f;
    const double MllwMinusNavd = -1.400;
    const double Usft = 1200.0 / 3937.0;
    static readonly Box Chan = new(-70.828, 42.808, -70.790, 42.824);
    const double LccPinX = 256429.4, LccPinY = 952196.8, LccPinLon = -70.8110, LccPinLat = 42.8183;

    HarvestContext _ctx = null!;
    SimpleFetcher _fetch = null!;
    string BtDir => _ctx.Big("bluetopo");
    string EhDir => _ctx.Big("ehydro");
    static string OutDir => Path.Combine("data", "bathy");

    void Log(string m) => _ctx.Log($"[hqbathy] {m}");

    (byte[] data, bool cached) Fetch(string url, string path, long minBytes = 1, double? maxAgeS = null, int timeoutS = 900)
    {
        if (File.Exists(path) && new FileInfo(path).Length >= minBytes)
            if (maxAgeS is null || (DateTime.UtcNow - File.GetLastWriteTimeUtc(path)).TotalSeconds < maxAgeS) return (File.ReadAllBytes(path), true);
        Directory.CreateDirectory(Path.GetDirectoryName(path)!);
        var data = _fetch.Download(url, timeoutS);
        if (data.Length < minBytes) throw new InvalidOperationException($"only {data.Length} bytes (wanted >= {minBytes})");
        File.WriteAllBytes(path, data);
        var what = url.Split('?')[0];
        Log($"    fetched {what[(what.LastIndexOf('/') + 1)..]}: {data.Length / 1e6:0.0} MB");
        return (data, false);
    }

    List<string> S3List(string prefix, string cacheName, double maxAgeS = 7 * 86400)
    {
        var url = S3 + "?list-type=2&prefix=" + Uri.EscapeDataString(prefix);
        var (xml, _) = Fetch(url, Path.Combine(BtDir, cacheName), maxAgeS: maxAgeS);
        return Regex.Matches(System.Text.Encoding.UTF8.GetString(xml), "<Key>([^<]+)</Key>").Select(m => m.Groups[1].Value).ToList();
    }

    void ProjectionPins()
    {
        var (lat, lon) = Geo.LccMassInverse(LccPinX, LccPinY);
        double dlat = Math.Abs(lat - LccPinLat), dlon = Math.Abs(lon - LccPinLon);
        Log($"LCC anchor: ({LccPinX}, {LccPinY}) m -> ({lat:0.00000}, {lon:0.00000}); pin ({LccPinLat}, {LccPinLon}) delta ({dlat:0.00000}, {dlon:0.00000}) deg");
        if (dlat > 0.001 || dlon > 0.001) throw new HarvestStop("LCC anchor pin FAILED -- refusing to convert eHydro coordinates");
        var (la, lo) = Geo.Utm19Inverse(350000.0, 4738000.0);
        var (e2, n2) = Geo.Tm19Forward(la, lo);
        var err = Math.Sqrt((e2 - 350000.0) * (e2 - 350000.0) + (n2 - 4738000.0) * (n2 - 4738000.0));
        Log($"TM round-trip: (350000, 4738000) -> ({la:0.000000}, {lo:0.000000}) -> back, {err * 100:0.00} cm");
        if (err > 0.05) throw new HarvestStop("TM round-trip pin FAILED -- refusing to sample BlueTopo");
    }

    /// Just enough of a BlueTopo COG: tiled, N interleaved float32 samples, deflate or raw, predictor none or TechNote-3. Band 0 is Elevation.
    sealed class BlueTopoTiff
    {
        public readonly GeoTiff T;
        public readonly string Name;
        public readonly int W, H;
        public readonly double E0, N1, Sx, Sy;
        public BlueTopoTiff(byte[] data, string name)
        {
            Name = name;
            T = new GeoTiff(data);
            if (!T.Tiled || T.Bps != 32 || T.SampleFormat != 3 || T.Tag1(284, 1) != 1)
                throw new HarvestStop($"{name}: layout {T.Bps}/{T.SampleFormat} planar {T.Tag1(284, 1)} tiled {T.Tiled} -- not the interleaved float32 COG this reader knows");
            if (T.Compression is not (1 or 8 or 32946)) throw new HarvestStop($"{name}: compression {T.Compression} (JPEG/LERC/ZSTD would land here) -- STOPPING rather than fabricating bathymetry");
            W = T.Width; H = T.Height;
            var scale = T.TagDoubles(33550);
            var tie = T.TagDoubles(33922);
            Sx = scale.Length > 0 ? scale[0] : 1.0;
            Sy = scale.Length > 1 ? scale[1] : 1.0;
            E0 = (tie.Length > 3 ? tie[3] : 0) - (tie.Length > 0 ? tie[0] : 0) * Sx;
            N1 = (tie.Length > 4 ? tie[4] : 0) + (tie.Length > 1 ? tie[1] : 0) * Sy;
        }

        public float[] Band0()
        {
            var out_ = new float[W * H];
            Array.Fill(out_, float.NaN);
            var tilesX = (W + T.Tw - 1) / T.Tw;
            var expect = T.Tw * T.Th * T.Spp * 4;
            for (var idx = 0; idx < T.Offsets.Length; idx++)
            {
                if (T.Counts[idx] == 0) continue;
                var dec = T.DecodeBlockBytes(idx, T.Tw, T.Th);
                if (T.Predictor is not (0 or 1 or 3)) throw new HarvestStop($"{Name}: predictor {T.Predictor} unsupported");
                int by = idx / tilesX, bx = idx % tilesX;
                int y0 = by * T.Th, x0 = bx * T.Tw;
                int hh = Math.Min(T.Th, H - y0), ww = Math.Min(T.Tw, W - x0);
                for (var yy = 0; yy < hh; yy++)
                    for (var xx = 0; xx < ww; xx++)
                    {
                        var at = ((yy * T.Tw + xx) * T.Spp) * 4;
                        if (at + 4 > dec.Length) continue;
                        var v = System.Buffers.Binary.BinaryPrimitives.ReadSingleLittleEndian(dec.AsSpan(at));
                        out_[(y0 + yy) * W + x0 + xx] = Math.Abs(v) > 99999.0f ? float.NaN : v;
                    }
            }
            return out_;
        }
    }

    List<(string name, string delivered, BlueTopoTiff tif)> FetchBlueTopoTiles()
    {
        var keys = S3List(SchemePrefix, "_tile_scheme_listing.xml");
        var gpkgs = keys.Where(k => k.ToLowerInvariant().EndsWith(".gpkg")).OrderBy(k => k, StringComparer.Ordinal).ToList();
        if (gpkgs.Count == 0) throw new HarvestStop("no tile-scheme gpkg in the S3 listing");
        var schemeKey = gpkgs[^1];
        var schemePath = Path.Combine(BtDir, schemeKey[(schemeKey.LastIndexOf('/') + 1)..]);
        var (_, cached) = Fetch(S3 + schemeKey, schemePath, minBytes: 1_000_000);
        Log($"tile scheme: {Path.GetFileName(schemePath)} ({new FileInfo(schemePath).Length / 1e6:0.0} MB{(cached ? ", cached" : "")})");
        var rows = new Dictionary<string, (string link, string delivered, string utm)>();
        using (var db = Gpkg.Open(schemePath))
        {
            var table = Gpkg.FeatureTables(db).Keys.Single();
            using var cmd = db.CreateCommand();
            cmd.CommandText = $"SELECT tile, GeoTIFF_Link, Delivered_Date, Resolution, UTM FROM \"{table}\" WHERE tile IN ({string.Join(",", BtTiles.Select((_, i) => "$t" + i))})";
            for (var i = 0; i < BtTiles.Length; i++) cmd.Parameters.AddWithValue("$t" + i, BtTiles[i]);
            using var r = cmd.ExecuteReader();
            while (r.Read()) rows[r.GetString(0)] = (r.GetString(1), r.IsDBNull(2) ? "" : r.GetValue(2).ToString()!, r.IsDBNull(4) ? "" : r.GetValue(4).ToString()!);
        }
        var missing = BtTiles.Where(t => !rows.ContainsKey(t)).ToList();
        if (missing.Count > 0) throw new HarvestStop($"tile scheme no longer lists [{string.Join(", ", missing)}] -- the scheme moved under us");
        var tiles = new List<(string, string, BlueTopoTiff)>();
        foreach (var tname in BtTiles)
        {
            var (link, delivered, utm) = rows[tname];
            if (utm != "19") throw new HarvestStop($"{tname}: UTM zone {utm} (expected 19)");
            var path = Path.Combine(BtDir, link[(link.LastIndexOf('/') + 1)..]);
            byte[] data; bool c;
            try { (data, c) = Fetch(link, path, minBytes: 500_000); }
            catch (HttpError ex) when (ex.Code == 404)
            {
                Log($"    {tname}: 404 on {link[(link.LastIndexOf('/') + 1)..]}; re-listing prefix");
                var tiffs = S3List($"BlueTopo/{tname}/", $"_listing_{tname}.xml", 3600).Where(k => k.ToLowerInvariant().EndsWith(".tiff")).OrderBy(k => k, StringComparer.Ordinal).ToList();
                if (tiffs.Count == 0) throw new HarvestStop($"{tname}: no tiff under BlueTopo/{tname}/");
                path = Path.Combine(BtDir, tiffs[^1][(tiffs[^1].LastIndexOf('/') + 1)..]);
                (data, c) = Fetch(S3 + tiffs[^1], path, minBytes: 500_000);
            }
            var tif = new BlueTopoTiff(data, Path.GetFileName(path));
            if (Math.Abs(tif.Sx - 4.0) > 1e-6 || Math.Abs(tif.Sy - 4.0) > 1e-6)
                throw new HarvestStop($"{tname}: pixel {tif.Sx}x{tif.Sy} m (expected 4 m); the resolution ladder changed -- re-plan before compositing");
            Log($"    {tname}: {tif.W}x{tif.H} px, comp {tif.T.Compression}, pred {tif.T.Predictor}, {data.Length / 1e6:0.0} MB {(c ? "(cached)" : "(fetched)")}, delivered {delivered}");
            tiles.Add((Path.GetFileName(path), delivered, tif));
        }
        return tiles;
    }

    (float[] mosaic, int mw, int mh, double eMin, double nMax) BuildMosaic(List<(string name, string delivered, BlueTopoTiff t)> tiles)
    {
        var eMin = tiles.Min(x => x.t.E0);
        var eMax = tiles.Max(x => x.t.E0 + x.t.W * 4.0);
        var nMax = tiles.Max(x => x.t.N1);
        var nMin = tiles.Min(x => x.t.N1 - x.t.H * 4.0);
        var mw = (int)Math.Round((eMax - eMin) / 4.0);
        var mh = (int)Math.Round((nMax - nMin) / 4.0);
        var mosaic = new float[mw * mh];
        Array.Fill(mosaic, float.NaN);
        foreach (var (name, _, t) in tiles)
        {
            var fx = (t.E0 - eMin) / 4.0;
            var fy = (nMax - t.N1) / 4.0;
            int ox = (int)Math.Round(fx), oy = (int)Math.Round(fy);
            if (Math.Abs(fx - ox) > 1e-3 || Math.Abs(fy - oy) > 1e-3) throw new HarvestStop($"{name}: tile off the shared 4 m lattice by ({fx - ox}, {fy - oy}) px -- mosaic assumption broken");
            var band = t.Band0();
            long valid = 0;
            for (var y = 0; y < t.H; y++)
                for (var x = 0; x < t.W; x++)
                {
                    var v = band[y * t.W + x];
                    if (float.IsNaN(v)) continue;
                    valid++;
                    mosaic[(oy + y) * mw + ox + x] = v;
                }
            Log($"    {name}: E {t.E0:0}..{t.E0 + t.W * 4:0}, N {t.N1 - t.H * 4:0}..{t.N1:0}, {100.0 * valid / band.Length:0}% valid");
        }
        Log($"mosaic {mw}x{mh} px (4 m), {100.0 * mosaic.Count(v => !float.IsNaN(v)) / mosaic.Length:0}% valid");
        return (mosaic, mw, mh, eMin, nMax);
    }

    int SplatBlueTopo(float[] grid, byte[] src, float[] mosaic, int mw, int mh, double mE0, double mN1)
    {
        var filled = 0;
        for (var r = 0; r < NY; r++)
        {
            var latC = Lat1 - (r + 0.5) * Dlat;
            for (var c = 0; c < NX; c++)
            {
                var lonC = Lon0 + (c + 0.5) * Dlon;
                var (e, n) = Geo.Tm19Forward(latC, lonC);
                var px = (e - mE0) / 4.0 - 0.5;
                var py = (mN1 - n) / 4.0 - 0.5;
                var x0 = (int)Math.Floor(px);
                var y0 = (int)Math.Floor(py);
                var inb = x0 >= 0 && x0 < mw - 1 && y0 >= 0 && y0 < mh - 1;
                var x0c = Math.Clamp(x0, 0, mw - 2);
                var y0c = Math.Clamp(y0, 0, mh - 2);
                float fx = (float)(px - x0), fy = (float)(py - y0);
                float wsum = 0, vsum = 0;
                foreach (var (oy, ox, w) in new[] { (0, 0, (1 - fx) * (1 - fy)), (0, 1, fx * (1 - fy)), (1, 0, (1 - fx) * fy), (1, 1, fx * fy) })
                {
                    var v = mosaic[(y0c + oy) * mw + x0c + ox];
                    if (float.IsNaN(v)) continue;
                    wsum += w; vsum += v * w;
                }
                if (inb && wsum >= 0.5f)
                {
                    grid[r * NX + c] = vsum / Math.Max(wsum, 1e-9f);
                    src[r * NX + c] = 1;
                    filled++;
                }
            }
        }
        Log($"BlueTopo splat: {filled} of {NX * NY} cells ({100.0 * filled / (NX * NY):0.0}%)");
        return filled;
    }

    List<System.Text.Json.Nodes.JsonNode> EhydroIndex()
    {
        var url = EhIndex + "?where=" + Uri.EscapeDataString($"channelareaidfk='{EhChannel}'") + "&outFields=*&f=json";
        var (raw, cached) = Fetch(url, Path.Combine(EhDir, $"index_{EhChannel}.json"), minBytes: 200, maxAgeS: 30 * 86400);
        var feats = Parse(System.Text.Encoding.UTF8.GetString(raw)).Arr("features").Where(f => f is not null).Select(f => f!).ToList();
        Log($"eHydro index: {feats.Count} surveys for {EhChannel} {(cached ? "(cached)" : "(fetched)")}");
        return feats;
    }

    static List<(double x, double y, double z)> ParseXyz(string text)
    {
        var rows = new List<(double, double, double)>();
        var started = false;
        foreach (var ln in text.Split('\n'))
        {
            var p = ln.Split((char[]?)null, StringSplitOptions.RemoveEmptyEntries);
            if (p.Length != 3) continue;
            if (double.TryParse(p[0], NumberStyles.Float, CultureInfo.InvariantCulture, out var x) && double.TryParse(p[1], NumberStyles.Float, CultureInfo.InvariantCulture, out var y) && double.TryParse(p[2], NumberStyles.Float, CultureInfo.InvariantCulture, out var z))
            {
                started = true;
                rows.Add((x, y, z));
            }
        }
        if (!started) throw new InvalidOperationException("no 'X Y Z' data rows found");
        return rows;
    }

    List<Dictionary<string, object?>> OverlayEhydro(float[] grid, byte[] src, List<System.Text.Json.Nodes.JsonNode> feats)
    {
        var chosen = new List<System.Text.Json.Nodes.JsonNode>();
        foreach (var want in EhSurveys)
        {
            var hit = feats.Select(f => f.Obj("attributes")).FirstOrDefault(at => (at.Str("surveyjobidpk") ?? "").Contains(want));
            if (hit is null) { Log($"    survey *{want}*: NOT in the index (skipping; plan called it optional for the XC)"); continue; }
            chosen.Add(hit);
        }
        var used = new List<Dictionary<string, object?>>();
        foreach (var att in chosen)
        {
            var sid = att.Str("surveyjobidpk")!;
            var url = att.Str("sourcedatalocation")!;
            var path = Path.Combine(EhDir, url[(url.LastIndexOf('/') + 1)..]);
            var (data, cached) = Fetch(url, path, minBytes: 1_000_000);
            Log($"    {sid}: {data.Length / 1e6:0.0} MB {(cached ? "(cached)" : "(fetched)")}");
            using var zf = new ZipArchive(new MemoryStream(data), ZipArchiveMode.Read);
            var xyzs = zf.Entries.Where(e => e.FullName.ToUpperInvariant().EndsWith(".XYZ")).ToList();
            var member = xyzs.FirstOrDefault(e => e.FullName.ToUpperInvariant().EndsWith("_FULL.XYZ")) ?? xyzs.FirstOrDefault(e => e.FullName.ToUpperInvariant().EndsWith("_A.XYZ")) ?? xyzs.MaxBy(e => e.Length)!;
            string text;
            using (var s = new StreamReader(member.Open(), System.Text.Encoding.ASCII)) text = s.ReadToEnd();
            var pts = ParseXyz(text);
            var sums = new Dictionary<int, (double s, int n)>();
            double lonMin = 1e9, lonMax = -1e9, latMin = 1e9, latMax = -1e9, zMin = 1e9, zMax = -1e9;
            var ok = 0;
            foreach (var (x, y, z) in pts)
            {
                var (lat, lon) = Geo.LccMassInverse(x * Usft, y * Usft);
                // eHydro's Z is a DEPTH: US survey feet below MLLW, positive down (the entrance channel's
                // thinned XYZ runs 0..28 ft, median 11.8, in a channel dredged to 15). Taken as a height
                // (the sign this line carried until 2026-10-17) the channel came out +2..+4 m NAVD88, the
                // engine drew it DRY, and the orthos' photographed water stood on it as ground -- Mark's
                // eye found it. The check below (ValidateVsCudem) sampled only cells below -1 m, so it
                // could not see the cells it had put above the water; it now samples the eHydro cells.
                var zNavd = -z * Usft + MllwMinusNavd;
                lonMin = Math.Min(lonMin, lon); lonMax = Math.Max(lonMax, lon); latMin = Math.Min(latMin, lat); latMax = Math.Max(latMax, lat);
                zMin = Math.Min(zMin, zNavd); zMax = Math.Max(zMax, zNavd);
                var ix = (int)Math.Floor((lon - Lon0) / Dlon);
                var iy = (int)Math.Floor((Lat1 - lat) / Dlat);
                if (ix < 0 || ix >= NX || iy < 0 || iy >= NY) continue;
                ok++;
                var k = iy * NX + ix;
                sums[k] = sums.TryGetValue(k, out var cur) ? (cur.s + zNavd, cur.n + 1) : (zNavd, 1);
            }
            foreach (var (k, (s, n)) in sums) { grid[k] = (float)(s / n); src[k] = 2; }
            var mname = member.FullName[(member.FullName.LastIndexOf('/') + 1)..];
            Log($"    {mname}: {pts.Count} pts, {ok} in-grid -> {sums.Count} cells; lon {lonMin:0.0000}..{lonMax:0.0000} lat {latMin:0.0000}..{latMax:0.0000}, z {zMin:0.0}..{zMax:0.0} m NAVD");
            used.Add(D(("survey", sid), ("zip", url), ("xyz", mname), ("points", pts.Count), ("cells", sums.Count)));
        }
        if (used.Count == 0) throw new HarvestStop("no eHydro survey found at all -- the channel truth is the point of this harvester; refusing to ship BlueTopo alone");
        return used;
    }

    (Dictionary<string, object?> rep, double medOut) ValidateVsCudem(float[] grid, byte[] src)
    {
        var meta = Load(Path.Combine(OutDir, "merrimack.json"))!;
        var cud = Bin.ReadF32(Path.Combine(OutDir, "merrimack.f32"));
        int cnx = (int)meta.Num("nx"), cny = (int)meta.Num("ny");
        double clon0 = meta.Num("lon0"), clat1 = meta.Num("lat1"), cdlon = meta.Num("dlon"), cdlat = meta.Num("dlat");
        var rng = new Random(20260831);
        var dIn = new List<double>(); var dOut = new List<double>(); var dEh = new List<double>();
        for (var k = 0; k < 3_000_000 && (dIn.Count < 20000 || dOut.Count < 20000 || dEh.Count < 20000); k++)
        {
            int iy = rng.Next(NY), ix = rng.Next(NX);
            var v = grid[iy * NX + ix];
            var ehydro = src[iy * NX + ix] == 2;
            // THE BLIND SPOT, closed: a cell the overlay put ABOVE the water is exactly the cell a sign
            // error makes, so the eHydro cells are sampled whatever their value; the BlueTopo cells keep
            // the wet-cell filter (their land edge is a different question).
            if (!(v > -9000)) continue;
            if (!ehydro && !(v < -1.0)) continue;
            var lat = Lat1 - (iy + 0.5) * Dlat;
            var lon = Lon0 + (ix + 0.5) * Dlon;
            var px = (lon - clon0) / cdlon - 0.5;
            var py = (clat1 - lat) / cdlat - 0.5;
            var x0 = (int)Math.Floor(px); var y0 = (int)Math.Floor(py);
            if (!(x0 >= 0 && x0 < cnx - 1 && y0 >= 0 && y0 < cny - 1)) continue;
            double fx = px - x0, fy = py - y0;
            float v00 = cud[y0 * cnx + x0], v01 = cud[y0 * cnx + x0 + 1], v10 = cud[(y0 + 1) * cnx + x0], v11 = cud[(y0 + 1) * cnx + x0 + 1];
            if (!(v00 > -9000 && v01 > -9000 && v10 > -9000 && v11 > -9000)) continue;
            var cv = v00 * (1 - fx) * (1 - fy) + v01 * fx * (1 - fy) + v10 * (1 - fx) * fy + v11 * fx * fy;
            var diff = v - cv;
            if (ehydro) { if (dEh.Count < 20000) dEh.Add(diff); continue; }
            if (Chan.Contains(lon, lat)) { if (dIn.Count < 20000) dIn.Add(diff); }
            else if (dOut.Count < 20000) dOut.Add(diff);
        }
        Log("datum cross-check vs CUDEM (inlethq - cudem; BlueTopo at wet cells < -1 m, the eHydro cells whatever their value):");
        Dictionary<string, object?>? Stats(string tag, List<double> d)
        {
            if (d.Count == 0) return null;
            var s = d.OrderBy(x => x).ToList();
            double Pct(double p) { var k = (s.Count - 1) * p / 100.0; var f = (int)Math.Floor(k); var c = Math.Min(f + 1, s.Count - 1); return s[f] + (s[c] - s[f]) * (k - f); }
            Log($"    {tag}: n={d.Count} mean={d.Average():+0.000;-0.000} median={Pct(50):+0.000;-0.000} p5={Pct(5):+0.000;-0.000} p95={Pct(95):+0.000;-0.000} m");
            return D(("n", d.Count), ("mean_m", Round(d.Average(), 3)), ("median_m", Round(Pct(50), 3)));
        }
        var rIn = Stats("inside federal channel (dredge/shoal EXPECTED)", dIn);
        var rOut = Stats("outside channel (datum agreement)", dOut);
        var rEh = Stats("the eHydro cells (the survey against CUDEM: dredging apart, the same bed)", dEh);
        // THE GATE a sign error cannot pass: the survey's cells lie within a few metres of CUDEM's bed
        // (the channel is dredged 1-3 m deeper than the 2014 plane in places, never 8 m shallower).
        if (rEh is not null && Math.Abs((double)rEh["median_m"]!) > 2.5)
            throw new HarvestStop($"the eHydro cells sit {(double)rEh["median_m"]!:+0.00;-0.00} m from CUDEM's bed at the median: a sign or datum error in the overlay, not dredging; refusing to write");
        var medOut = rOut is not null ? (double)rOut["median_m"]! : 0.0;
        return (D(("inside_channel", rIn), ("outside_channel", rOut), ("ehydro_cells", rEh)), medOut);
    }

    void WritePreviews(float[] grid, byte[] src)
    {
        int w = (NX + 3) / 4, h = (NY + 3) / 4;
        var pix = new byte[w * h];
        var sp = new byte[w * h];
        for (var y = 0; y < h; y++)
            for (var x = 0; x < w; x++)
            {
                var g = grid[(y * 4) * NX + x * 4];
                pix[y * w + x] = g > -9000 && g < 0 ? (byte)Math.Clamp(120.0 + g * 8.0, 16, 120) : g >= 0 ? (byte)Math.Clamp(140.0 + g * 10.0, 140, 255) : (byte)8;
                sp[y * w + x] = (byte)(src[(y * 4) * NX + x * 4] * 127);
            }
        Png.WriteGray(Path.Combine(OutDir, "inlethq_preview.png"), pix, w, h);
        Png.WriteGray(Path.Combine(OutDir, "inlethq_src.png"), sp, w, h);
        Log($"previews -> inlethq_preview.png, inlethq_src.png ({w}x{h})");
    }

    public int Run(HarvestContext ctx, string[] argv)
    {
        _ctx = ctx;
        _fetch = new SimpleFetcher(ctx, 0.6, 3);
        if ((long)NX * NY > (1L << 26)) throw new HarvestStop($"{NX}x{NY} = {(long)NX * NY} cells exceeds BathyModel's 2^26 cap");
        Log($"target grid {NX} x {NY} = {(long)NX * NY} cells at ~1.5 m (cap 2^26 = {1 << 26}; dlon {Dlon:0.000e+00}, dlat {Dlat:0.000e+00})");
        Directory.CreateDirectory(BtDir);
        Directory.CreateDirectory(EhDir);
        Directory.CreateDirectory(OutDir);
        ProjectionPins();
        var tiles = FetchBlueTopoTiles();
        var (mosaic, mw, mh, mE0, mN1) = BuildMosaic(tiles);
        var grid = new float[NX * NY];
        Array.Fill(grid, NoData);
        var src = new byte[NX * NY];
        SplatBlueTopo(grid, src, mosaic, mw, mh, mE0, mN1);
        mosaic = Array.Empty<float>();
        var surveys = OverlayEhydro(grid, src, EhydroIndex());

        var validN = 0; float vmin = float.MaxValue, vmax = float.MinValue; int nBt = 0, nEh = 0;
        for (var i = 0; i < grid.Length; i++)
        {
            if (grid[i] > -9000) { validN++; vmin = Math.Min(vmin, grid[i]); vmax = Math.Max(vmax, grid[i]); }
            if (src[i] == 1) nBt++; else if (src[i] == 2) nEh++;
        }
        var cover = (double)validN / grid.Length;
        Log($"grid: {100 * cover:0.0}% covered (bluetopo {nBt}, ehydro {nEh} cells), range [{vmin:0.0}, {vmax:0.0}] m NAVD88");
        if (vmin < -60.0 || vmax > 40.0) Log("WARNING: range outside the expected -25..+12 envelope; inspect the preview");
        var (rep, medOut) = ValidateVsCudem(grid, src);
        if (Math.Abs(medOut) > 0.5)
            throw new HarvestStop($"outside-channel median {medOut:+0.000;-0.000} m vs CUDEM exceeds 0.5 m -- a datum or projection sign is wrong (check usft factor, the -1.400 sign, the UTM zone). NOT writing output.");
        if (Math.Abs(medOut) > 0.35) Log($"WARNING: outside-channel median {medOut:+0.000;-0.000} m > 0.35 m -- datums only loosely aligned; shipping, but the waterline gate should re-audit");

        var f32Path = Path.Combine(OutDir, "inlethq.f32");
        Bin.WriteF32(f32Path, grid);
        Log($"wrote {f32Path} ({grid.Length * 4 / 1e6:0.0} MB); hashing (FNV-1a)");
        var content = Bin.Fnv1a(System.Runtime.InteropServices.MemoryMarshal.AsBytes<float>(grid));
        var meta = D(("generated_utc", NowIso()),
            ("source", $"NOAA BlueTopo 4 m UTM19 COGs ({string.Join(", ", tiles.Select(t => t.name))}) + USACE eHydro {EhChannel} ({string.Join(", ", surveys.Select(s => s["survey"]))}), rung-rule composite at ~1.5 m: eHydro cell means replace BlueTopo bilinear; holes stay nodata (CUDEM sits beneath this rung)"),
            ("file", "inlethq.f32"), ("nx", NX), ("ny", NY), ("lon0", Lon0), ("lat1", Lat1), ("dlon", Dlon), ("dlat", Dlat), ("row0", "north"), ("nodata", (double)NoData),
            ("min_m", Round(vmin, 3)), ("max_m", Round(vmax, 3)), ("content_fnv64", content.ToString("x16")),
            ("datum", "NAVD88 (BlueTopo native EPSG:5103; eHydro converted MLLW->NAVD88 -1.400 m via CO-OPS station ladder)"),
            ("crs_note", "NAD83 vs WGS84 ~1 m: documented, not corrected (M6l precedent)"), ("coverage", Round(cover, 4)),
            ("cells_bluetopo", nBt), ("cells_ehydro", nEh), ("surveys", surveys), ("validation_vs_cudem", rep));
        Save(Path.Combine(OutDir, "inlethq.json"), meta);
        Log($"sidecar -> inlethq.json (content fnv64 {content:x16})");
        WritePreviews(grid, src);
        Log("done");
        return 0;
    }
}
