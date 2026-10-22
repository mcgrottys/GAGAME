using System.Buffers.Binary;
using System.IO.Compression;

namespace Gagame.Harvest;

/// 8-bit grayscale PNG, no library (the hqbathy previews).
public static class Png
{
    public static void WriteGray(string path, ReadOnlySpan<byte> pix, int w, int h, bool exclusive = false)
    {
        var raw = new byte[h * (w + 1)];
        for (var y = 0; y < h; y++)
        {
            raw[y * (w + 1)] = 0;
            pix.Slice(y * w, w).CopyTo(raw.AsSpan(y * (w + 1) + 1));
        }
        using var ms = new MemoryStream();
        ms.Write(new byte[] { 0x89, (byte)'P', (byte)'N', (byte)'G', 0x0D, 0x0A, 0x1A, 0x0A });
        var ihdr = new byte[13];
        BinaryPrimitives.WriteUInt32BigEndian(ihdr, (uint)w);
        BinaryPrimitives.WriteUInt32BigEndian(ihdr.AsSpan(4), (uint)h);
        ihdr[8] = 8;
        Chunk(ms, "IHDR", ihdr);
        Chunk(ms, "IDAT", Deflate(raw));
        Chunk(ms, "IEND", Array.Empty<byte>());
        using var f = new FileStream(path, exclusive ? FileMode.CreateNew : FileMode.Create, FileAccess.Write);
        ms.WriteTo(f);
    }

    static byte[] Deflate(byte[] raw)
    {
        using var o = new MemoryStream();
        using (var z = new ZLibStream(o, CompressionLevel.Optimal, leaveOpen: true)) z.Write(raw);
        return o.ToArray();
    }

    static void Chunk(Stream s, string tag, byte[] payload)
    {
        Span<byte> len = stackalloc byte[4];
        BinaryPrimitives.WriteUInt32BigEndian(len, (uint)payload.Length);
        s.Write(len);
        var t = System.Text.Encoding.ASCII.GetBytes(tag);
        s.Write(t);
        s.Write(payload);
        var crc = Crc32(0xFFFFFFFFu, t);
        crc = Crc32(crc, payload);
        Span<byte> c = stackalloc byte[4];
        BinaryPrimitives.WriteUInt32BigEndian(c, crc ^ 0xFFFFFFFFu);
        s.Write(c);
    }

    static readonly uint[] Table = MakeTable();
    static uint[] MakeTable()
    {
        var t = new uint[256];
        for (uint n = 0; n < 256; n++)
        {
            var c = n;
            for (var k = 0; k < 8; k++) c = (c & 1) != 0 ? 0xEDB88320u ^ (c >> 1) : c >> 1;
            t[n] = c;
        }
        return t;
    }

    static uint Crc32(uint c, ReadOnlySpan<byte> b)
    {
        foreach (var x in b) c = Table[(c ^ x) & 0xFF] ^ (c >> 8);
        return c;
    }

    /// zlib's crc32 of a byte run (the PNG chunks, the Python zlib.crc32 idiom).
    public static uint Crc(ReadOnlySpan<byte> b) => Crc32(0xFFFFFFFFu, b) ^ 0xFFFFFFFFu;
}
