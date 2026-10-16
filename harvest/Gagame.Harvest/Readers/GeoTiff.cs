using System.Buffers.Binary;
using System.IO.Compression;
using System.Text;

namespace Gagame.Harvest.Readers;

/// Minimal GeoTIFF reader for NOAA's float32 tiles (geotiff.py): classic TIFF either byte
/// order, strips or tiles, compression none / LZW / Deflate, predictor none / horizontal /
/// floating-point (TIFF TechNote 3), ModelPixelScale + ModelTiepoint, GDAL's nodata and the
/// GeoKey directory. WINDOWED reads decode only the blocks under the window.
public sealed class GeoTiff
{
    const int TagWidth = 256, TagHeight = 257, TagBps = 258, TagCompression = 259;
    const int TagStripOffsets = 273, TagSpp = 277, TagRowsPerStrip = 278, TagStripCounts = 279;
    const int TagPlanar = 284, TagPredictor = 317;
    const int TagTileW = 322, TagTileH = 323, TagTileOffsets = 324, TagTileCounts = 325;
    const int TagSampleFormat = 339, TagPixelScale = 33550, TagTiepoint = 33922, TagGdalNodata = 42113;
    static readonly int[] TypeSizes = { 0, 1, 1, 2, 4, 8, 1, 1, 2, 4, 8, 4, 8, 4, 4, 4, 8, 8, 8 };

    readonly byte[] _d;
    readonly bool _big;
    readonly Dictionary<int, (int typ, int n, byte[] raw)> _tags = new();

    public int Width, Height, Bps, Compression, Predictor, SampleFormat, Spp;
    public bool Tiled;
    public int Tw, Th, TilesX;
    public long[] Offsets = Array.Empty<long>(), Counts = Array.Empty<long>();
    public double Sx, Sy, Lon0, Lat0;
    public double? Nodata;

    public GeoTiff(byte[] data)
    {
        _d = data;
        if (data[0] == (byte)'I' && data[1] == (byte)'I') _big = false;
        else if (data[0] == (byte)'M' && data[1] == (byte)'M') _big = true;
        else throw new InvalidDataException("not a TIFF");
        var magic = U16(2);
        if (magic == 43) throw new InvalidDataException("BigTIFF not supported");
        if (magic != 42) throw new InvalidDataException($"bad TIFF magic {magic}");
        ReadIfd((int)U32(4));

        Width = Tag1(TagWidth);
        Height = Tag1(TagHeight);
        Bps = Tag1(TagBps, 32);
        Compression = Tag1(TagCompression, 1);
        Predictor = Tag1(TagPredictor, 1);
        SampleFormat = Tag1(TagSampleFormat, 3);
        Spp = Tag1(TagSpp, 1);
        Tiled = _tags.ContainsKey(TagTileOffsets);
        if (Tiled)
        {
            Tw = Tag1(TagTileW);
            Th = Tag1(TagTileH);
            Offsets = TagLongs(TagTileOffsets);
            Counts = TagLongs(TagTileCounts);
            TilesX = (Width + Tw - 1) / Tw;
        }
        else
        {
            Tw = Width;
            Th = Tag1(TagRowsPerStrip, Height);
            Offsets = TagLongs(TagStripOffsets);
            Counts = TagLongs(TagStripCounts);
            TilesX = 1;
        }
        var scale = TagDoubles(TagPixelScale);
        var tie = TagDoubles(TagTiepoint);
        Sx = scale.Length > 0 ? scale[0] : 1.0;
        Sy = scale.Length > 1 ? scale[1] : 1.0;
        // CUDEM anchors pixel (0,0) at the NW corner; a general tiepoint is honoured by the callers that need it.
        Lon0 = tie.Length > 3 ? tie[3] : 0.0;
        Lat0 = tie.Length > 4 ? tie[4] : 0.0;
        TieI = tie.Length > 0 ? tie[0] : 0.0;
        TieJ = tie.Length > 1 ? tie[1] : 0.0;
        var nod = TagAscii(TagGdalNodata);
        if (nod is not null && double.TryParse(nod.Trim('\0', ' '), System.Globalization.NumberStyles.Float, System.Globalization.CultureInfo.InvariantCulture, out var nv))
            Nodata = nv;
    }

    /// The tiepoint's raster (i, j): the model origin is at that pixel, not always (0, 0).
    public double TieI, TieJ;

    ushort U16(long at) => _big ? BinaryPrimitives.ReadUInt16BigEndian(_d.AsSpan((int)at)) : BinaryPrimitives.ReadUInt16LittleEndian(_d.AsSpan((int)at));
    uint U32(long at) => _big ? BinaryPrimitives.ReadUInt32BigEndian(_d.AsSpan((int)at)) : BinaryPrimitives.ReadUInt32LittleEndian(_d.AsSpan((int)at));

    void ReadIfd(int off)
    {
        int count = U16(off);
        var p = off + 2;
        for (var i = 0; i < count; i++)
        {
            int tag = U16(p), typ = U16(p + 2);
            var n = (int)U32(p + 4);
            var size = (typ < TypeSizes.Length ? TypeSizes[typ] : 1) * n;
            byte[] raw;
            if (size <= 4) raw = _d.AsSpan(p + 8, Math.Min(size, 4)).ToArray();
            else
            {
                var vo = (int)U32(p + 8);
                raw = _d.AsSpan(vo, size).ToArray();
            }
            _tags[tag] = (typ, n, raw);
            p += 12;
        }
    }

    public bool HasTag(int tag) => _tags.ContainsKey(tag);

    public double[] TagDoubles(int tag)
    {
        if (!_tags.TryGetValue(tag, out var t)) return Array.Empty<double>();
        var (typ, n, raw) = t;
        var out_ = new double[n];
        for (var i = 0; i < n; i++)
        {
            out_[i] = typ switch
            {
                1 => raw[i],
                3 => _big ? BinaryPrimitives.ReadUInt16BigEndian(raw.AsSpan(2 * i)) : BinaryPrimitives.ReadUInt16LittleEndian(raw.AsSpan(2 * i)),
                4 => _big ? BinaryPrimitives.ReadUInt32BigEndian(raw.AsSpan(4 * i)) : BinaryPrimitives.ReadUInt32LittleEndian(raw.AsSpan(4 * i)),
                8 => _big ? BinaryPrimitives.ReadInt16BigEndian(raw.AsSpan(2 * i)) : BinaryPrimitives.ReadInt16LittleEndian(raw.AsSpan(2 * i)),
                9 => _big ? BinaryPrimitives.ReadInt32BigEndian(raw.AsSpan(4 * i)) : BinaryPrimitives.ReadInt32LittleEndian(raw.AsSpan(4 * i)),
                11 => _big ? BinaryPrimitives.ReadSingleBigEndian(raw.AsSpan(4 * i)) : BinaryPrimitives.ReadSingleLittleEndian(raw.AsSpan(4 * i)),
                12 => _big ? BinaryPrimitives.ReadDoubleBigEndian(raw.AsSpan(8 * i)) : BinaryPrimitives.ReadDoubleLittleEndian(raw.AsSpan(8 * i)),
                16 => _big ? BinaryPrimitives.ReadUInt64BigEndian(raw.AsSpan(8 * i)) : BinaryPrimitives.ReadUInt64LittleEndian(raw.AsSpan(8 * i)),
                5 => Rational(raw, i),
                _ => 0,
            };
        }
        return out_;
    }

    double Rational(byte[] raw, int i)
    {
        var a = _big ? BinaryPrimitives.ReadUInt32BigEndian(raw.AsSpan(8 * i)) : BinaryPrimitives.ReadUInt32LittleEndian(raw.AsSpan(8 * i));
        var b = _big ? BinaryPrimitives.ReadUInt32BigEndian(raw.AsSpan(8 * i + 4)) : BinaryPrimitives.ReadUInt32LittleEndian(raw.AsSpan(8 * i + 4));
        return (double)a / Math.Max(b, 1);
    }

    public long[] TagLongs(int tag) => TagDoubles(tag).Select(v => (long)v).ToArray();

    public int Tag1(int tag, int dflt = 0)
    {
        var v = TagDoubles(tag);
        return v.Length > 0 ? (int)v[0] : dflt;
    }

    public string? TagAscii(int tag) =>
        _tags.TryGetValue(tag, out var t) && t.typ == 2 ? Encoding.ASCII.GetString(t.raw) : null;

    // ---- blocks

    static byte[] LzwDecode(ReadOnlySpan<byte> data)
    {
        const int Clear = 256, Eoi = 257;
        var out_ = new MemoryStream();
        var table = new List<byte[]>(4096);
        void Reset()
        {
            table.Clear();
            for (var i = 0; i < 256; i++) table.Add(new[] { (byte)i });
            table.Add(Array.Empty<byte>());
            table.Add(Array.Empty<byte>());
        }
        Reset();
        ulong bitbuf = 0;
        var bitcnt = 0;
        var pos = 0;
        var width = 9;
        byte[]? prev = null;
        var n = data.Length;
        while (pos < n || bitcnt >= width)
        {
            while (bitcnt < width && pos < n)
            {
                bitbuf = (bitbuf << 8) | data[pos++];
                bitcnt += 8;
            }
            if (bitcnt < width) break;
            var code = (int)((bitbuf >> (bitcnt - width)) & ((1UL << width) - 1));
            bitcnt -= width;
            if (code == Clear) { Reset(); width = 9; prev = null; continue; }
            if (code == Eoi) break;
            byte[] entry;
            if (prev is null) entry = table[code];
            else if (code < table.Count)
            {
                entry = table[code];
                var add = new byte[prev.Length + 1];
                prev.CopyTo(add, 0);
                add[^1] = entry[0];
                table.Add(add);
            }
            else
            {
                entry = new byte[prev.Length + 1];
                prev.CopyTo(entry, 0);
                entry[^1] = prev[0];
                table.Add(entry);
            }
            out_.Write(entry);
            prev = entry;
            if (table.Count + 1 >= (1 << width) && width < 12) width++;
        }
        return out_.ToArray();
    }

    static byte[] Inflate(ReadOnlySpan<byte> raw)
    {
        using var ms = new MemoryStream(raw.ToArray());
        using var z = new ZLibStream(ms, CompressionMode.Decompress);
        using var o = new MemoryStream();
        z.CopyTo(o);
        return o.ToArray();
    }

    /// Undo the predictor on a decoded block of w x h samples of `bpp` bytes, `spp` samples per pixel.
    public static byte[] UndoPredictor(byte[] raw, int w, int h, int bps, int spp, int predictor)
    {
        if (predictor is 0 or 1) return raw;
        var bpp = bps / 8;
        var rowBytes = w * spp * bpp;
        if (predictor == 2)
        {
            var out_ = (byte[])raw.Clone();
            var stride = spp * bpp;
            for (var r = 0; r < h; r++)
            {
                var b = r * rowBytes;
                for (var x = stride; x < rowBytes; x++) out_[b + x] = (byte)(out_[b + x] + out_[b + x - stride]);
            }
            return out_;
        }
        if (predictor == 3)
        {
            var res = new byte[raw.Length];
            var samples = w * spp;
            var row = new byte[rowBytes];
            for (var r = 0; r < h; r++)
            {
                var b = r * rowBytes;
                Array.Copy(raw, b, row, 0, Math.Min(rowBytes, raw.Length - b));
                for (var i = 1; i < rowBytes; i++) row[i] = (byte)(row[i] + row[i - 1]);
                for (var x = 0; x < samples; x++)
                    for (var k = 0; k < bpp; k++)
                        res[b + x * bpp + (bpp - 1 - k)] = row[k * samples + x];
            }
            return res;
        }
        throw new InvalidDataException($"predictor {predictor} not supported");
    }

    /// One block decoded to raw sample bytes (after the predictor), `expect` bytes or shorter.
    public byte[] DecodeBlockBytes(int idx, int bw, int bh)
    {
        if (idx >= Offsets.Length || Counts[idx] == 0) return Array.Empty<byte>();
        var raw = _d.AsSpan((int)Offsets[idx], (int)Counts[idx]);
        byte[] dec = Compression switch
        {
            1 => raw.ToArray(),
            5 => LzwDecode(raw),
            8 or 32946 => Inflate(raw),
            _ => throw new InvalidDataException($"compression {Compression} not supported"),
        };
        return UndoPredictor(dec, bw, bh, Bps, Spp, Predictor);
    }

    /// A block as float32 samples (band 0 of an interleaved block).
    float[] DecodeBlock(int idx, int bw, int bh)
    {
        var dec = DecodeBlockBytes(idx, bw, bh);
        var vals = new float[bw * bh];
        var n = Math.Min(vals.Length, dec.Length / (4 * Spp));
        for (var i = 0; i < n; i++)
        {
            var at = i * Spp * 4;
            vals[i] = _big ? BinaryPrimitives.ReadSingleBigEndian(dec.AsSpan(at)) : BinaryPrimitives.ReadSingleLittleEndian(dec.AsSpan(at));
        }
        return vals;
    }

    /// float32 window, row-major, NaN where nodata. Decodes only intersecting blocks.
    public float[] ReadWindow(int x0, int y0, int w, int h)
    {
        var out_ = new float[w * h];
        Array.Fill(out_, float.NaN);
        // the blocks the window meets INSIDE the image: a block column past the last one would index
        // into the next row's first tile
        int bx0 = Math.Max(0, x0) / Tw, bx1 = Math.Min(x0 + w - 1, Width - 1) / Tw;
        int by0 = Math.Max(0, y0) / Th, by1 = Math.Min(y0 + h - 1, Height - 1) / Th;
        for (var by = by0; by <= by1; by++)
            for (var bx = bx0; bx <= bx1; bx++)
            {
                int idx, bw, bh;
                if (Tiled) { idx = by * TilesX + bx; bw = Tw; bh = Th; }
                else { idx = by; bw = Width; bh = Math.Min(Th, Height - by * Th); }
                if (idx >= Offsets.Length) continue;
                var block = DecodeBlock(idx, bw, bh);
                int ox = bx * Tw, oy = by * Th;
                int fx0 = Math.Max(x0, ox), fx1 = Math.Min(x0 + w, ox + bw);
                int fy0 = Math.Max(y0, oy), fy1 = Math.Min(y0 + h, oy + bh);
                for (var yy = fy0; yy < fy1; yy++)
                {
                    var src = (yy - oy) * bw + (fx0 - ox);
                    var dst = (yy - y0) * w + (fx0 - x0);
                    Array.Copy(block, src, out_, dst, fx1 - fx0);
                }
            }
        if (Nodata is double nd)
            for (var i = 0; i < out_.Length; i++)
            {
                var v = out_[i];
                if (v == nd || v < -99999.0f || v > 99999.0f) out_[i] = float.NaN;
            }
        return out_;
    }

    // ---- declared georeferencing

    static readonly Dictionary<int, string> GeoKeyNames = new()
    {
        [1024] = "GTModelType", [1025] = "GTRasterType", [1026] = "GTCitation",
        [2048] = "GeographicType", [2049] = "GeogCitation", [2050] = "GeogGeodeticDatum",
        [2054] = "GeogAngularUnits", [3072] = "ProjectedCSType", [3076] = "ProjLinearUnits",
        [4096] = "VerticalCSType", [4097] = "VerticalCitation", [4098] = "VerticalDatum", [4099] = "VerticalUnits",
    };

    /// GeoKeyDirectory resolved against its double and ASCII parameter tags, plus GDAL's
    /// metadata/nodata tags when present. Keys by name.
    public Dictionary<string, object?> GeoKeys()
    {
        var out_ = new Dictionary<string, object?>();
        var d = TagDoubles(34735);
        if (d.Length >= 4)
        {
            var dbl = TagDoubles(34736);
            var asc = TagAscii(34737) ?? "";
            var count = (int)d[3];
            for (var k = 0; k < count && 8 + 4 * k <= d.Length; k++)
            {
                int key = (int)d[4 + 4 * k], loc = (int)d[5 + 4 * k], cnt = (int)d[6 + 4 * k], val = (int)d[7 + 4 * k];
                var name = GeoKeyNames.GetValueOrDefault(key, key.ToString());
                if (loc == 0) out_[name] = val;
                else if (loc == 34736) out_[name] = cnt > 1 ? dbl.Skip(val).Take(cnt).ToList() : val < dbl.Length ? dbl[val] : null;
                else if (loc == 34737) out_[name] = asc.Substring(Math.Min(val, asc.Length), Math.Min(cnt, Math.Max(asc.Length - val, 0))).TrimEnd('|', '\0', ' ');
            }
        }
        var meta = TagAscii(42112);
        if (meta is not null && meta.Trim().Length > 0) out_["GDAL_METADATA"] = meta.Trim('\0', ' ');
        if (Nodata is double nd) out_["GDAL_NODATA"] = nd;
        return out_;
    }

    public int PxOfLon(double lon) => (int)((lon - Lon0) / Sx);
    public int PyOfLat(double lat) => (int)((Lat0 - lat) / Sy);
}
