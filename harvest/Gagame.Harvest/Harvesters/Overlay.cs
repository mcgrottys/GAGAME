using Gagame.Harvest.Readers;
using static Gagame.Harvest.JsonUtil;

namespace Gagame.Harvest.Harvesters;

/// M6o: user GeoTIFF overlays as compositor planes, ALPHA AS FIBER (harvest_overlay.py). Any
/// GeoTIFF in cache/overlay (UTM 19N, RGB or RGBA, 8-bit) becomes an .rgba mip chain with an
/// alpha-weighted box soak; with none present, the demo highlights over the Merrimack inlet.
public sealed class Overlay : IHarvester
{
    public string Name => "overlay";
    public string Title => "Overlay (your GeoTIFFs)";
    public string What => "Any GeoTIFF as a compositor plane, alpha as fibre; with none present a demo highlights overlay is made.";
    public string[] Outputs => ["data/overlay/overlay.json"];
    public Source[] Sources => [new("GeoTIFF spec", "https://www.ogc.org/standard/geotiff/")];
    public Insert? Insert => new(InsertKind.DropFolder, "Drop GeoTIFFs (RGB or RGBA 8-bit, UTM 19N) here; georeferencing is read from the tags.", Folder: "cache/overlay", Pattern: "*.tif");

    static string CacheDir => Path.Combine("cache", "overlay");
    static string OutDir => Path.Combine("data", "overlay");
    static readonly HashSet<int> Utm19nEpsg = [6348, 26918, 26919, 32619];

    public sealed record Plane(string Name, byte[] Rgba, int W, int H, double E0, double N0, double E1, double N1, double M);

    /// Alpha-weighted 2x2 box: colour averages weight by alpha, alpha averages plainly.
    static (byte[] rgba, int w, int h) ReduceRgba(byte[] level, int w, int h)
    {
        int w2 = w / 2, h2 = h / 2;
        var out_ = new byte[w2 * h2 * 4];
        for (var y = 0; y < h2; y++)
            for (var x = 0; x < w2; x++)
            {
                double r = 0, g = 0, b = 0, wsum = 0, asum = 0;
                for (var dy = 0; dy < 2; dy++)
                    for (var dx = 0; dx < 2; dx++)
                    {
                        var p = ((y * 2 + dy) * w + (x * 2 + dx)) * 4;
                        double a = level[p + 3];
                        r += level[p] * a; g += level[p + 1] * a; b += level[p + 2] * a;
                        wsum += a; asum += a;
                    }
                var o = (y * w2 + x) * 4;
                var ww = Math.Max(wsum, 1e-3);
                out_[o] = (byte)Math.Clamp(r / ww, 0, 255);
                out_[o + 1] = (byte)Math.Clamp(g / ww, 0, 255);
                out_[o + 2] = (byte)Math.Clamp(b / ww, 0, 255);
                out_[o + 3] = (byte)Math.Clamp(asum / 4, 0, 255);
            }
        return (out_, w2, h2);
    }

    static List<Dictionary<string, object?>> WriteChain(byte[] img, int w, int h, string path)
    {
        var mips = new List<Dictionary<string, object?>>();
        using var f = File.Create(path);
        var level = img;
        while (true)
        {
            mips.Add(D(("px", w), ("offset", f.Position)));
            f.Write(level);
            if (w / 2 < 40 || w % 2 != 0) break;
            (level, w, h) = ReduceRgba(level, w, h);
        }
        return mips;
    }

    List<Plane> IngestGeoTiffs(HarvestContext ctx)
    {
        var entries = new List<Plane>();
        foreach (var path in Directory.GetFiles(CacheDir, "*.tif").Concat(Directory.GetFiles(CacheDir, "*.tiff")).OrderBy(x => x, StringComparer.Ordinal))
        {
            var name = Path.GetFileNameWithoutExtension(path);
            var tif = new GeoTiff(File.ReadAllBytes(path));
            var scale = tif.TagDoubles(33550);
            var tie = tif.TagDoubles(33922);
            if (scale.Length < 2 || tie.Length < 6) { ctx.Log($"[overlay] {name}: no GeoTIFF georeferencing tags; skipped"); continue; }
            var keys = tif.TagDoubles(34735);
            var epsg = 0;
            for (var k = 4; k + 3 < keys.Length; k += 4) if ((int)keys[k] == 3072) epsg = (int)keys[k + 3];
            if (epsg != 0 && !Utm19nEpsg.Contains(epsg)) { ctx.Log($"[overlay] {name}: EPSG {epsg} not UTM 19N; v1 skips (extend Projections.h to accept more)"); continue; }
            if (tif.Bps != 8 || tif.SampleFormat != 1 && tif.SampleFormat != 0) { ctx.Log($"[overlay] {name}: {tif.Bps}-bit samples (format {tif.SampleFormat}); this port reads 8-bit RGB/RGBA only; skipped"); continue; }
            var spp = tif.Spp;
            int w = tif.Width, h = tif.Height;
            var img = new byte[w * h * 4];
            for (var by = 0; by < (h + tif.Th - 1) / tif.Th; by++)
                for (var bx = 0; bx < (tif.Tiled ? tif.TilesX : 1); bx++)
                {
                    int idx = tif.Tiled ? by * tif.TilesX + bx : by;
                    int bw = tif.Tiled ? tif.Tw : w, bh = tif.Tiled ? tif.Th : Math.Min(tif.Th, h - by * tif.Th);
                    var dec = tif.DecodeBlockBytes(idx, bw, bh);
                    for (var yy = 0; yy < bh; yy++)
                        for (var xx = 0; xx < bw; xx++)
                        {
                            int X = bx * tif.Tw + xx, Y = by * tif.Th + yy;
                            if (X >= w || Y >= h) continue;
                            var s = (yy * bw + xx) * spp;
                            if (s + spp > dec.Length) continue;
                            var o = (Y * w + X) * 4;
                            if (spp == 1) { img[o] = img[o + 1] = img[o + 2] = dec[s]; img[o + 3] = 255; }
                            else { img[o] = dec[s]; img[o + 1] = dec[s + 1]; img[o + 2] = dec[s + 2]; img[o + 3] = spp >= 4 ? dec[s + 3] : (byte)255; }
                        }
                }
            double sx = scale[0], sy = scale[1];
            double i = tie[0], j = tie[1], X0 = tie[3], Y0 = tie[4];
            var e0 = X0 - i * sx;
            var n1 = Y0 + j * sy;
            var e1 = e0 + w * sx;
            var n0 = n1 - h * sy;
            entries.Add(new Plane(name, img, w, h, e0, n0, e1, n1, sx));
            ctx.Log($"[overlay] {name}: {w}x{h} rgba, UTM E {e0:0}..{e1:0} N {n0:0}..{n1:0}, {sx} m/px");
        }
        return entries;
    }

    static List<Plane> DemoHighlights(HarvestContext ctx)
    {
        double e0 = 351000.0, n0 = 4740000.0, e1 = 352500.0, n1 = 4743000.0;
        const double m = 0.5;
        int w = (int)((e1 - e0) / m), h = (int)((n1 - n0) / m);
        var img = new byte[w * h * 4];
        for (var yy = 0; yy < h; yy++)
            for (var xx = 0; xx < w; xx++)
            {
                var E = e0 + (xx + 0.5) * m;
                var N = n1 - (yy + 0.5) * m;
                var r = Math.Sqrt((E - 352260.0) * (E - 352260.0) + (N - 4741900.0) * (N - 4741900.0));
                var ring = Math.Clamp(1.0 - Math.Abs(r - 260.0) / 13.0, 0, 1);
                var r2 = Math.Sqrt((E - 351650.0) * (E - 351650.0) + (N - 4741350.0) * (N - 4741350.0));
                var disc = Math.Pow(Math.Clamp(1.0 - r2 / 320.0, 0, 1), 2);
                var a = Math.Clamp(ring * 0.62 + disc * 0.38, 0, 1);
                var o = (yy * w + xx) * 4;
                img[o] = 255;
                img[o + 1] = (byte)(ring > disc ? 176 : 64);
                img[o + 2] = (byte)(ring > disc ? 32 : 200);
                img[o + 3] = (byte)(a * 255);
            }
        ctx.Log($"[overlay] demo highlights generated ({w}x{h} @ {m} m/px)");
        return [new Plane("demo_highlights", img, w, h, e0, n0, e1, n1, m)];
    }

    static List<Plane> Squarify(HarvestContext ctx, List<Plane> entries)
    {
        var out_ = new List<Plane>();
        foreach (var p in entries)
        {
            if (p.H == p.W) { out_.Add(p); continue; }
            if (p.H % p.W != 0) ctx.Log($"[overlay] {p.Name}: dropping {p.H % p.W} remainder rows (not a full square)");
            for (var k = 0; k < p.H / p.W; k++)
            {
                var top = k * p.W;
                var nHi = p.N1 - top * p.M;
                var slice = new byte[p.W * p.W * 4];
                Array.Copy(p.Rgba, top * p.W * 4, slice, 0, slice.Length);
                out_.Add(new Plane($"{p.Name}_{k}", slice, p.W, p.W, p.E0, nHi - p.W * p.M, p.E1, nHi, p.M));
            }
        }
        return out_;
    }

    public int Run(HarvestContext ctx, string[] argv)
    {
        Directory.CreateDirectory(CacheDir);
        Directory.CreateDirectory(OutDir);
        var found = IngestGeoTiffs(ctx);
        var entries = Squarify(ctx, found.Count > 0 ? found : DemoHighlights(ctx));
        var tiles = new List<Dictionary<string, object?>>();
        foreach (var p in entries)
        {
            var raw = Path.Combine(OutDir, $"{p.Name}.rgba");
            var mips = WriteChain(p.Rgba, p.W, p.H, raw);
            var (latSw, lonSw) = Geo.Utm19InverseShort(p.E0, p.N0);
            var (latNe, lonNe) = Geo.Utm19InverseShort(p.E1, p.N1);
            tiles.Add(D(("file", $"{p.Name}.rgba"), ("channels", 4), ("utm_e0", p.E0), ("utm_n0", p.N0), ("utm_e1", p.E1), ("utm_n1", p.N1),
                ("m_per_px", p.M), ("mips", mips), ("wgs84_bbox", new List<double> { lonSw, latSw, lonNe, latNe })));
            ctx.Log($"[overlay] {raw}: {mips.Count} mips, {new FileInfo(raw).Length / 1048576} MB");
        }
        Save(Path.Combine(OutDir, "overlay.json"), D(("name", "user.highlights"), ("structure", "geotiff overlay rgba (per-pixel alpha = paint weight)"),
            ("crs", "EPSG:6348 NAD83(2011) / UTM zone 19N"), ("cm_per_px", entries[0].M * 100.0),
            ("source", "user GeoTIFFs in cache/overlay/ (or the generated demo)"), ("tiles", tiles)));
        ctx.Log($"[overlay] done -> data/overlay/overlay.json ({tiles.Count} planes)");
        return 0;
    }
}
