using System.IO.Compression;
using System.Text.RegularExpressions;
using ImageMagick;
using static Gagame.Harvest.JsonUtil;

namespace Gagame.Harvest.Harvesters;

/// M6l: MassGIS 2023 15 cm aerial orthos (harvest_aerial.py): each USNG tile's lossless JP2
/// decoded (Magick.NET, in place of Pillow + OpenJPEG) into an 8-bit RGB mip chain, box-filtered
/// down to ~19 m/px; aerial.json carries the UTM georeferencing and the WGS84 bbox.
public sealed class Aerial : IHarvester
{
    public string Name => "aerial";
    public string Title => "Aerial orthos (MassGIS 2023)";
    public string What => "15 cm leaf-off orthos for the Merrimack inlet as RGB mip chains, the first independent high-res source.";
    public string[] Outputs => ["data/aerial/aerial.json"];
    public Source[] Sources => [new("MassGIS 2023 aerial imagery", "https://www.mass.gov/info-details/massgis-data-2023-aerial-imagery")];
    public Insert? Insert => new(InsertKind.DropFolder, "Drop MassGIS USNG tile zips (19T*.zip) here.", Folder: "cache/aerial", Pattern: "19T*.zip");

    static string CacheDir => Path.Combine("cache", "aerial");
    static string OutDir => Path.Combine("data", "aerial");

    public int Run(HarvestContext ctx, string[] argv)
    {
        Directory.CreateDirectory(OutDir);
        var tiles = new List<Dictionary<string, object?>>();
        foreach (var zp in Directory.GetFiles(CacheDir, "19T*.zip").OrderBy(x => x, StringComparer.Ordinal))
        {
            var name = Path.GetFileNameWithoutExtension(zp);
            var rawPath = Path.Combine(OutDir, name + ".rgb");
            using var z = ZipFile.OpenRead(zp);
            string aux;
            using (var s = new StreamReader(z.GetEntry(name + ".jp2.aux.xml")!.Open())) aux = s.ReadToEnd();
            var mlo = Regex.Match(aux, @"<gml:lowerCorner>(\d+) (\d+)</gml:lowerCorner>");
            var mhi = Regex.Match(aux, @"<gml:upperCorner>(\d+) (\d+)</gml:upperCorner>");
            int e0 = int.Parse(mlo.Groups[1].Value), n0 = int.Parse(mlo.Groups[2].Value);
            int e1 = int.Parse(mhi.Groups[1].Value), n1 = int.Parse(mhi.Groups[2].Value);
            var mips = new List<Dictionary<string, object?>>();
            if (!File.Exists(rawPath))
            {
                var jp2 = Path.Combine(CacheDir, name + ".jp2");
                if (!File.Exists(jp2)) z.GetEntry(name + ".jp2")!.ExtractToFile(jp2);
                ctx.Log($"[aerial] decoding {name}.jp2 (10000^2 lossless; a minute or two)");
                using var im = new MagickImage(jp2);
                im.ColorSpace = ColorSpace.sRGB;
                using var f = File.Create(rawPath);
                var level = im;
                var px = (int)im.Width;
                MagickImage? owned = null;
                while (px >= 40)
                {
                    mips.Add(D(("px", px), ("offset", f.Position)));
                    using (var pixels = level.GetPixels()) f.Write(pixels.ToByteArray("RGB")!);
                    px /= 2;
                    if (px >= 40)
                    {
                        var next = (MagickImage)level.Clone();
                        next.FilterType = FilterType.Box;
                        next.Resize(new MagickGeometry((uint)px, (uint)px) { IgnoreAspectRatio = true });
                        owned?.Dispose();
                        owned = next;
                        level = next;
                    }
                }
                owned?.Dispose();
                ctx.Log($"[aerial] {name}: {mips.Count} mip levels, {new FileInfo(rawPath).Length / 1048576} MB raw");
            }
            else
            {
                long off = 0;
                var px = 10000;
                while (px >= 40)
                {
                    mips.Add(D(("px", px), ("offset", off)));
                    off += (long)px * px * 3;
                    px /= 2;
                }
                ctx.Log($"[aerial] cache hit: {rawPath}");
            }
            var (latSw, lonSw) = Geo.Utm19Inverse(e0, n0);
            var (latNe, lonNe) = Geo.Utm19Inverse(e1, n1);
            tiles.Add(D(("file", name + ".rgb"), ("utm_e0", e0), ("utm_n0", n0), ("utm_e1", e1), ("utm_n1", n1),
                ("m_per_px", 0.15), ("px", 10000), ("mips", mips), ("wgs84_bbox", new List<double> { lonSw, latSw, lonNe, latNe })));
        }
        Save(Path.Combine(OutDir, "aerial.json"), D(
            ("source", "MassGIS 2023 aerial orthos (15 cm, leaf-off, plane-flown)"), ("crs", "EPSG:6348 NAD83(2011) / UTM zone 19N"), ("tiles", tiles)));
        ctx.Log($"[aerial] done -> data/aerial/aerial.json ({tiles.Count} tiles)");
        return 0;
    }
}
