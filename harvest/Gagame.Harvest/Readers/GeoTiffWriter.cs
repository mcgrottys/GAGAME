using System.Buffers.Binary;
using System.IO.Compression;
using System.Text;

namespace Gagame.Harvest.Readers;

/// The GeoTIFF the composites write, in the layout CUDEM and ETOPO 2022 themselves ship (and so
/// the layout the engine's WIC door is already proven on, RasterFileTest's realheights gate):
/// classic little-endian TIFF, one float32 band, 256 x 256 tiles, Deflate with the floating-point
/// predictor (TIFF TechNote 3), ModelPixelScale + ModelTiepoint, PixelIsArea, EPSG:4326, the
/// vertical CRS and unit as GeoKeys held in the directory itself (the engine reads only those),
/// and GDAL's nodata tag. Edge tiles are padded with nodata to the full 256 x 256 the format asks.
public static class GeoTiffWriter
{
    public const int Tile = 256;

    /// `values` is row-major, row 0 at `lat1` (north), NaN where there is no data (written as
    /// `nodata`). `lon0`/`lat1` are the outer EDGES of pixel (0, 0); `step` is degrees a pixel.
    /// `verticalEpsg`: 5703 NAVD88 height, 3855 EGM2008 height. Returns the bytes written.
    public static long Write(string path, float[] values, int width, int height, double lon0, double lat1, double stepX, double stepY,
        int verticalEpsg, float nodata, CompressionLevel level = CompressionLevel.Optimal)
    {
        if (values.Length != (long)width * height) throw new ArgumentException($"{values.Length} values for {width}x{height}");
        int tx = (width + Tile - 1) / Tile, ty = (height + Tile - 1) / Tile, nt = tx * ty;
        var blocks = new byte[nt][];
        Parallel.For(0, nt, t => blocks[t] = EncodeTile(values, width, height, (t % tx) * Tile, (t / tx) * Tile, nodata, level));

        var tags = new SortedDictionary<ushort, (ushort type, uint count, byte[] data)>();
        void Short(ushort tag, params ushort[] v) => tags[tag] = (3, (uint)v.Length, v.SelectMany(x => BitConverter.GetBytes(x)).ToArray());
        void Long(ushort tag, params uint[] v) => tags[tag] = (4, (uint)v.Length, v.SelectMany(x => BitConverter.GetBytes(x)).ToArray());
        void Double(ushort tag, params double[] v) => tags[tag] = (12, (uint)v.Length, v.SelectMany(x => BitConverter.GetBytes(x)).ToArray());
        void Ascii(ushort tag, string s) { var b = Encoding.ASCII.GetBytes(s + "\0"); tags[tag] = (2, (uint)b.Length, b); }

        Long(256, (uint)width);
        Long(257, (uint)height);
        Short(258, 32);
        Short(259, 8);          // Deflate
        Short(262, 1);          // BlackIsZero
        Short(277, 1);
        Short(284, 1);
        Short(317, 3);          // floating-point predictor
        Short(322, Tile);
        Short(323, Tile);
        Long(324, new uint[nt]); // patched below
        Long(325, blocks.Select(b => (uint)b.Length).ToArray());
        Short(339, 3);          // IEEE float
        Double(33550, stepX, stepY, 0.0);
        Double(33922, 0, 0, 0, lon0, lat1, 0);
        // GeoKeyDirectory: version 1.1.0, then (key, location 0, count 1, value) -- every value in
        // the directory itself, which is all the engine's ImageLoader reads (location 0).
        var keys = new List<ushort[]>
        {
            new ushort[] { 1024, 0, 1, 2 },      // GTModelType: geographic
            new ushort[] { 1025, 0, 1, 1 },      // GTRasterType: PixelIsArea
            new ushort[] { 2048, 0, 1, 4326 },   // GeographicType: WGS 84
            new ushort[] { 2054, 0, 1, 9102 },   // GeogAngularUnits: degree
            new ushort[] { 4096, 0, 1, (ushort)verticalEpsg },
            new ushort[] { 4099, 0, 1, 9001 },   // VerticalUnits: metre
        };
        Short(34735, new ushort[] { 1, 1, 0, (ushort)keys.Count }.Concat(keys.SelectMany(k => k)).ToArray());
        Ascii(42113, nodata.ToString("R", System.Globalization.CultureInfo.InvariantCulture));

        // layout: header (8) | IFD | out-of-line tag data | tile blocks
        var ifdSize = 2 + 12 * tags.Count + 4;
        long dataAt = 8 + ifdSize;
        var outOfLine = new Dictionary<ushort, long>();
        foreach (var (tag, t) in tags)
            if (t.data.Length > 4) { outOfLine[tag] = dataAt; dataAt += t.data.Length + (t.data.Length & 1); }
        var offsets = new uint[nt];
        var at = dataAt;
        for (var i = 0; i < nt; i++) { offsets[i] = checked((uint)at); at += blocks[i].Length; }
        Long(324, offsets);

        using var f = new FileStream(path, FileMode.Create, FileAccess.Write, FileShare.None, 1 << 20);
        Span<byte> b8 = stackalloc byte[8];
        f.Write("II"u8);
        BinaryPrimitives.WriteUInt16LittleEndian(b8, 42); f.Write(b8[..2]);
        BinaryPrimitives.WriteUInt32LittleEndian(b8, 8); f.Write(b8[..4]);
        BinaryPrimitives.WriteUInt16LittleEndian(b8, (ushort)tags.Count); f.Write(b8[..2]);
        foreach (var (tag, t) in tags)
        {
            BinaryPrimitives.WriteUInt16LittleEndian(b8, tag); f.Write(b8[..2]);
            BinaryPrimitives.WriteUInt16LittleEndian(b8, t.type); f.Write(b8[..2]);
            BinaryPrimitives.WriteUInt32LittleEndian(b8, t.count); f.Write(b8[..4]);
            if (t.data.Length > 4) { BinaryPrimitives.WriteUInt32LittleEndian(b8, (uint)outOfLine[tag]); f.Write(b8[..4]); }
            else { b8[..4].Clear(); t.data.CopyTo(b8); f.Write(b8[..4]); }
        }
        b8[..4].Clear(); f.Write(b8[..4]);   // no next IFD
        foreach (var (tag, t) in tags)
            if (t.data.Length > 4) { f.Write(t.data); if ((t.data.Length & 1) != 0) f.WriteByte(0); }
        foreach (var blk in blocks) f.Write(blk);
        return f.Length;
    }

    /// One tile: float32 little-endian samples, byte-planed most significant first and
    /// differenced along each row (predictor 3, the inverse of GeoTiff.UndoPredictor), deflated.
    static byte[] EncodeTile(float[] values, int width, int height, int x0, int y0, float nodata, CompressionLevel level)
    {
        const int rowBytes = Tile * 4;
        var raw = new byte[Tile * rowBytes];
        var le = new byte[rowBytes];
        for (var r = 0; r < Tile; r++)
        {
            var y = y0 + r;
            for (var c = 0; c < Tile; c++)
            {
                var x = x0 + c;
                var v = x < width && y < height ? values[(long)y * width + x] : float.NaN;
                if (float.IsNaN(v)) v = nodata;
                BinaryPrimitives.WriteSingleLittleEndian(le.AsSpan(c * 4), v);
            }
            var row = raw.AsSpan(r * rowBytes, rowBytes);
            for (var c = 0; c < Tile; c++)
                for (var k = 0; k < 4; k++) row[k * Tile + c] = le[c * 4 + (3 - k)];
            for (var i = rowBytes - 1; i > 0; i--) row[i] = (byte)(row[i] - row[i - 1]);
        }
        using var ms = new MemoryStream();
        using (var z = new ZLibStream(ms, level, leaveOpen: true)) z.Write(raw);
        return ms.ToArray();
    }
}
