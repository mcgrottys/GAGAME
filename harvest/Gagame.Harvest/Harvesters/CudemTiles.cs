using System.IO.Compression;
using System.Text.RegularExpressions;
using Gagame.Harvest.Readers;

namespace Gagame.Harvest.Harvesters;

/// NOAA CUDEM 1/9-arc-second tile index helpers (from the retired `bathy` window harvester): the
/// dataset's own tile index shapefile read from its zip, and a tile's 0.25-degree cell from its name.
/// The relief source `cudem-index` (Sources/ReliefReaders.cs) reads tiles through these.
public static class CudemTiles
{
    public sealed record Tile(string Name, string Region, string Url, string Srs, double[]? Bbox);

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
}
