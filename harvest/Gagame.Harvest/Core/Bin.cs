using System.Buffers.Binary;
using System.Runtime.InteropServices;

namespace Gagame.Harvest;

/// Little-endian binary helpers for the engine's flat files (.f32, .i16, the .bin runs).
public static class Bin
{
    public static void WriteF32(string path, ReadOnlySpan<float> v, bool exclusive = false)
    {
        using var f = new FileStream(path, exclusive ? FileMode.CreateNew : FileMode.Create, FileAccess.Write);
        f.Write(MemoryMarshal.AsBytes(v));
    }

    public static void WriteI16(string path, ReadOnlySpan<short> v)
    {
        using var f = File.Create(path);
        f.Write(MemoryMarshal.AsBytes(v));
    }

    public static float[] ReadF32(string path)
    {
        var b = File.ReadAllBytes(path);
        return MemoryMarshal.Cast<byte, float>(b.AsSpan(0, b.Length / 4 * 4)).ToArray();
    }

    public static void U32(Stream s, uint v) { Span<byte> b = stackalloc byte[4]; BinaryPrimitives.WriteUInt32LittleEndian(b, v); s.Write(b); }
    public static void I32(Stream s, int v) { Span<byte> b = stackalloc byte[4]; BinaryPrimitives.WriteInt32LittleEndian(b, v); s.Write(b); }
    public static void U16(Stream s, ushort v) { Span<byte> b = stackalloc byte[2]; BinaryPrimitives.WriteUInt16LittleEndian(b, v); s.Write(b); }
    public static void I16(Stream s, short v) { Span<byte> b = stackalloc byte[2]; BinaryPrimitives.WriteInt16LittleEndian(b, v); s.Write(b); }
    public static void I64(Stream s, long v) { Span<byte> b = stackalloc byte[8]; BinaryPrimitives.WriteInt64LittleEndian(b, v); s.Write(b); }
    public static void U64(Stream s, ulong v) { Span<byte> b = stackalloc byte[8]; BinaryPrimitives.WriteUInt64LittleEndian(b, v); s.Write(b); }
    public static void F32(Stream s, float v) { Span<byte> b = stackalloc byte[4]; BinaryPrimitives.WriteSingleLittleEndian(b, v); s.Write(b); }
    public static void F64(Stream s, double v) { Span<byte> b = stackalloc byte[8]; BinaryPrimitives.WriteDoubleLittleEndian(b, v); s.Write(b); }

    public static uint U32(ReadOnlySpan<byte> b, int at) => BinaryPrimitives.ReadUInt32LittleEndian(b.Slice(at));
    public static int I32(ReadOnlySpan<byte> b, int at) => BinaryPrimitives.ReadInt32LittleEndian(b.Slice(at));
    public static ushort U16(ReadOnlySpan<byte> b, int at) => BinaryPrimitives.ReadUInt16LittleEndian(b.Slice(at));
    public static double F64(ReadOnlySpan<byte> b, int at) => BinaryPrimitives.ReadDoubleLittleEndian(b.Slice(at));
    public static float F32(ReadOnlySpan<byte> b, int at) => BinaryPrimitives.ReadSingleLittleEndian(b.Slice(at));
    public static int I32BE(ReadOnlySpan<byte> b, int at) => BinaryPrimitives.ReadInt32BigEndian(b.Slice(at));
    public static uint U32BE(ReadOnlySpan<byte> b, int at) => BinaryPrimitives.ReadUInt32BigEndian(b.Slice(at));

    /// FNV-1a 64 over a byte run (the refbathy / hqbathy content identity).
    public static ulong Fnv1a(ReadOnlySpan<byte> data)
    {
        var h = 0xcbf29ce484222325UL;
        foreach (var b in data) h = (h ^ b) * 0x100000001b3UL;
        return h;
    }
}

/// The polyline run format of data/gis/*.bin: u32 lines, then per line u32 count, count x (f32 lon, f32 lat).
public static class PolylineBin
{
    public static void Write(string path, List<List<(double lon, double lat)>> lines, HarvestContext ctx, string tag = "gis")
    {
        using (var f = File.Create(path))
        {
            Bin.U32(f, (uint)lines.Count);
            foreach (var line in lines)
            {
                Bin.U32(f, (uint)line.Count);
                foreach (var (lon, lat) in line) { Bin.F32(f, (float)lon); Bin.F32(f, (float)lat); }
            }
        }
        ctx.Log($"[{tag}] {path.Replace('\\', '/')}: {lines.Count} polylines, {lines.Sum(l => l.Count)} points");
    }

    public static List<List<(double lon, double lat)>> Read(string path)
    {
        var b = File.ReadAllBytes(path);
        var out_ = new List<List<(double, double)>>();
        var p = 0;
        var n = Bin.U32(b, p); p += 4;
        for (var i = 0; i < n; i++)
        {
            var c = Bin.U32(b, p); p += 4;
            var line = new List<(double, double)>((int)c);
            for (var k = 0; k < c; k++) { line.Add((Bin.F32(b, p), Bin.F32(b, p + 4))); p += 8; }
            out_.Add(line);
        }
        return out_;
    }
}
