using System.Buffers.Binary;
using System.IO.Compression;
using System.Text;

namespace Gagame.Harvest.Readers;

/// An OpenStreetMap PBF reader (osmpbf.py): reports the file faithfully and decides nothing.
/// Coordinates stay the file's own 1e-7 degree integers; a block whose granularity or offsets
/// are not whole 1e-7 degree is refused by name rather than rounded. LocationsOnWays (osmium
/// add-locations-to-ways, fields 9 and 10 of a way) are read, which is why OsmSharp is not used.
public static class OsmPbf
{
    public static (ulong v, int p) Varint(ReadOnlySpan<byte> b, int p)
    {
        ulong r = 0;
        var s = 0;
        while (true)
        {
            var c = b[p++];
            r |= (ulong)(c & 0x7F) << s;
            if (c < 0x80) return (r, p);
            s += 7;
        }
    }

    public readonly record struct Field(int F, ulong V, int Start, int Len);

    /// (field number, value) over one message: a varint, or a byte slice (Start, Len).
    public static IEnumerable<Field> Fields(byte[] b, int p = 0, int end = -1)
    {
        if (end < 0) end = b.Length;
        while (p < end)
        {
            var (key, q) = Varint(b, p);
            p = q;
            int f = (int)(key >> 3), wt = (int)(key & 7);
            switch (wt)
            {
                case 0: { var (v, r) = Varint(b, p); p = r; yield return new Field(f, v, 0, 0); break; }
                case 2: { var (n, r) = Varint(b, p); p = r; yield return new Field(f, 0, p, (int)n); p += (int)n; break; }
                case 1: yield return new Field(f, 0, p, 8); p += 8; break;
                case 5: yield return new Field(f, 0, p, 4); p += 4; break;
                default: throw new InvalidDataException($"wire type {wt} not in the PBF format");
            }
        }
    }

    public static ulong[] PackedU(ReadOnlySpan<byte> b)
    {
        var out_ = new List<ulong>(b.Length);
        var p = 0;
        while (p < b.Length) { var (v, q) = Varint(b, p); out_.Add(v); p = q; }
        return out_.ToArray();
    }

    public static long Zig(ulong u) => (long)(u >> 1) ^ -(long)(u & 1);

    public static long[] PackedS(ReadOnlySpan<byte> b) => PackedU(b).Select(Zig).ToArray();

    /// A packed, zigzagged, delta-coded field (ids, lats, lons, refs) as absolute int64.
    public static long[] PackedDelta(ReadOnlySpan<byte> b)
    {
        var s = PackedS(b);
        for (var i = 1; i < s.Length; i++) s[i] += s[i - 1];
        return s;
    }

    public sealed class Way
    {
        public long Id;
        public int[] Keys = Array.Empty<int>(), Vals = Array.Empty<int>();
        public byte[] Refs = Array.Empty<byte>(), Lats = Array.Empty<byte>(), Lons = Array.Empty<byte>();
    }

    public sealed class Relation
    {
        public long Id;
        public int[] Keys = Array.Empty<int>(), Vals = Array.Empty<int>();
        public long[] Members = Array.Empty<long>();
        public long[] Types = Array.Empty<long>();
        public long[] Roles = Array.Empty<long>();
    }

    /// One PrimitiveBlock: its strings, its coordinate frame and its groups, undecoded.
    public sealed class Block
    {
        public List<string> Strings = new();
        public long Gran = 100, LatOff, LonOff;
        public List<(int start, int len)> Groups = new();
        public Dictionary<string, int> Index = new();
        readonly byte[] _raw;

        public Block(byte[] raw)
        {
            _raw = raw;
            foreach (var f in Fields(raw))
            {
                if (f.F == 1)
                    foreach (var g in Fields(raw, f.Start, f.Start + f.Len))
                        if (g.F == 1) Strings.Add(Encoding.UTF8.GetString(raw, g.Start, g.Len));
                if (f.F == 2) Groups.Add((f.Start, f.Len));
                else if (f.F == 17) Gran = (long)f.V;
                else if (f.F == 19) LatOff = (long)f.V;
                else if (f.F == 20) LonOff = (long)f.V;
            }
            if (Gran % 100 != 0 || LatOff % 100 != 0 || LonOff % 100 != 0)
                throw new InvalidDataException($"block granularity {Gran} / offsets {LatOff},{LonOff} are not whole 1e-7 degree: refused rather than rounded");
            for (var i = 0; i < Strings.Count; i++) Index.TryAdd(Strings[i], i);
        }

        /// The kind of each group: dense, nodes, ways, relations, changesets, empty.
        public List<string> Kinds()
        {
            var out_ = new List<string>();
            foreach (var (s, l) in Groups)
            {
                var k = "empty";
                foreach (var f in Fields(_raw, s, s + l))
                {
                    k = f.F switch { 1 => "nodes", 2 => "dense", 3 => "ways", 4 => "relations", 5 => "changesets", _ => "?" };
                    break;
                }
                out_.Add(k);
            }
            return out_;
        }

        long E7(long v, long off) => Math.DivRem(off + Gran * v, 100, out _) is var q && (off + Gran * v) % 100 == 0 ? q : FloorDiv(off + Gran * v, 100);
        static long FloorDiv(long a, long b) { var q = a / b; return (a % b != 0 && (a < 0) != (b < 0)) ? q - 1 : q; }

        public long[] E7(long[] v, long off)
        {
            var out_ = new long[v.Length];
            for (var i = 0; i < v.Length; i++) out_[i] = FloorDiv(off + Gran * v[i], 100);
            return out_;
        }

        /// Every dense node run of the block: (ids, lat e7, lon e7).
        public IEnumerable<(long[] ids, long[] lat, long[] lon)> Dense()
        {
            foreach (var (s, l) in Groups)
                foreach (var f in Fields(_raw, s, s + l))
                {
                    if (f.F != 2) continue;
                    long[]? ids = null, lat = null, lon = null;
                    foreach (var h in Fields(_raw, f.Start, f.Start + f.Len))
                    {
                        if (h.F == 1) ids = PackedDelta(_raw.AsSpan(h.Start, h.Len));
                        else if (h.F == 8) lat = PackedDelta(_raw.AsSpan(h.Start, h.Len));
                        else if (h.F == 9) lon = PackedDelta(_raw.AsSpan(h.Start, h.Len));
                    }
                    if (ids is not null) yield return (ids, E7(lat!, LatOff), E7(lon!, LonOff));
                }
        }

        /// Plain (non-dense) nodes, one at a time: (id, lat e7, lon e7).
        public IEnumerable<(long id, long lat, long lon)> Nodes()
        {
            foreach (var (s, l) in Groups)
                foreach (var f in Fields(_raw, s, s + l))
                {
                    if (f.F != 1) continue;
                    long nid = 0, la = 0, lo = 0;
                    foreach (var h in Fields(_raw, f.Start, f.Start + f.Len))
                    {
                        if (h.F == 1) nid = Zig(h.V);
                        else if (h.F == 8) la = Zig(h.V);
                        else if (h.F == 9) lo = Zig(h.V);
                    }
                    yield return (nid, FloorDiv(LatOff + Gran * la, 100), FloorDiv(LonOff + Gran * lo, 100));
                }
        }

        /// Every way, lazily; refs/lats/lons are the packed slices (decode with PackedDelta / WayE7).
        public IEnumerable<Way> Ways()
        {
            foreach (var (s, l) in Groups)
                foreach (var f in Fields(_raw, s, s + l))
                {
                    if (f.F != 3) continue;
                    var w = new Way();
                    foreach (var h in Fields(_raw, f.Start, f.Start + f.Len))
                    {
                        if (h.F == 1) w.Id = (long)h.V;
                        else if (h.F == 2) w.Keys = PackedU(_raw.AsSpan(h.Start, h.Len)).Select(x => (int)x).ToArray();
                        else if (h.F == 3) w.Vals = PackedU(_raw.AsSpan(h.Start, h.Len)).Select(x => (int)x).ToArray();
                        else if (h.F == 8) w.Refs = _raw.AsSpan(h.Start, h.Len).ToArray();
                        else if (h.F == 9) w.Lats = _raw.AsSpan(h.Start, h.Len).ToArray();
                        else if (h.F == 10) w.Lons = _raw.AsSpan(h.Start, h.Len).ToArray();
                    }
                    yield return w;
                }
        }

        /// A way's own coordinates (LocationsOnWays) in 1e-7 degree, as (lon, lat).
        public (long[] lon, long[] lat) WayE7(Way w) => (E7(PackedDelta(w.Lons), LonOff), E7(PackedDelta(w.Lats), LatOff));

        public IEnumerable<Relation> Relations()
        {
            foreach (var (s, l) in Groups)
                foreach (var f in Fields(_raw, s, s + l))
                {
                    if (f.F != 4) continue;
                    var r = new Relation();
                    foreach (var h in Fields(_raw, f.Start, f.Start + f.Len))
                    {
                        if (h.F == 1) r.Id = (long)h.V;
                        else if (h.F == 2) r.Keys = PackedU(_raw.AsSpan(h.Start, h.Len)).Select(x => (int)x).ToArray();
                        else if (h.F == 3) r.Vals = PackedU(_raw.AsSpan(h.Start, h.Len)).Select(x => (int)x).ToArray();
                        else if (h.F == 8) r.Roles = PackedU(_raw.AsSpan(h.Start, h.Len)).Select(x => (long)x).ToArray();
                        else if (h.F == 9) r.Members = PackedDelta(_raw.AsSpan(h.Start, h.Len));
                        else if (h.F == 10) r.Types = PackedU(_raw.AsSpan(h.Start, h.Len)).Select(x => (long)x).ToArray();
                    }
                    yield return r;
                }
        }
    }

    /// (offset, type) of every blob, by seeking past the data: no decompression.
    public static List<(long off, string type)> BlockTable(string path)
    {
        var out_ = new List<(long, string)>();
        using var f = File.OpenRead(path);
        var hl = new byte[4];
        while (true)
        {
            var off = f.Position;
            if (f.Read(hl, 0, 4) < 4) return out_;
            var n = (int)BinaryPrimitives.ReadUInt32BigEndian(hl);
            var hdr = new byte[n];
            f.ReadExactly(hdr);
            var btype = "";
            long size = 0;
            foreach (var fd in Fields(hdr))
            {
                if (fd.F == 1) btype = Encoding.ASCII.GetString(hdr, fd.Start, fd.Len);
                else if (fd.F == 3) size = (long)fd.V;
            }
            f.Seek(size, SeekOrigin.Current);
            out_.Add((off, btype));
        }
    }

    /// Every blob of the file, decompressed, in order.
    public static IEnumerable<(long off, string type, byte[] raw)> Blobs(string path)
    {
        using var f = File.OpenRead(path);
        var hl = new byte[4];
        while (true)
        {
            var off = f.Position;
            if (f.Read(hl, 0, 4) < 4) yield break;
            var n = (int)BinaryPrimitives.ReadUInt32BigEndian(hl);
            var hdr = new byte[n];
            f.ReadExactly(hdr);
            var btype = "";
            var size = 0;
            foreach (var fd in Fields(hdr))
            {
                if (fd.F == 1) btype = Encoding.ASCII.GetString(hdr, fd.Start, fd.Len);
                else if (fd.F == 3) size = (int)fd.V;
            }
            var blob = new byte[size];
            f.ReadExactly(blob);
            yield return (off, btype, Decompress(blob));
        }
    }

    /// The one blob at a byte offset BlockTable reported, decompressed.
    public static byte[] BlobAt(string path, long off)
    {
        using var f = File.OpenRead(path);
        f.Seek(off, SeekOrigin.Begin);
        var hl = new byte[4];
        f.ReadExactly(hl);
        var n = (int)BinaryPrimitives.ReadUInt32BigEndian(hl);
        var hdr = new byte[n];
        f.ReadExactly(hdr);
        var size = 0;
        foreach (var fd in Fields(hdr)) if (fd.F == 3) size = (int)fd.V;
        var blob = new byte[size];
        f.ReadExactly(blob);
        return Decompress(blob);
    }

    static byte[] Decompress(byte[] blob)
    {
        byte[]? raw = null, z = null;
        foreach (var f in Fields(blob))
        {
            if (f.F == 1) raw = blob.AsSpan(f.Start, f.Len).ToArray();
            else if (f.F == 3) z = blob.AsSpan(f.Start, f.Len).ToArray();
            else if (f.F is 4 or 5 or 6 or 7) throw new InvalidDataException($"blob compression field {f.F} (lzma/bzip2/lz4/zstd) is not read here");
        }
        if (raw is not null) return raw;
        using var ms = new MemoryStream(z!);
        using var zs = new ZLibStream(ms, CompressionMode.Decompress);
        using var o = new MemoryStream();
        zs.CopyTo(o);
        return o.ToArray();
    }

    /// The OSMHeader's facts a manifest should carry: bbox, required features, replication time.
    public static Dictionary<string, object?> Header(string path)
    {
        foreach (var (_, btype, raw) in Blobs(path))
        {
            if (btype != "OSMHeader") continue;
            var req = new List<string>();
            var opt = new List<string>();
            var out_ = new Dictionary<string, object?> { ["required"] = req, ["optional"] = opt };
            foreach (var f in Fields(raw))
            {
                if (f.F == 1)
                {
                    var box = new Dictionary<int, long>();
                    foreach (var h in Fields(raw, f.Start, f.Start + f.Len)) box[h.F] = Zig(h.V);
                    out_["bbox_nanodeg"] = new List<object?> { Get(box, 1), Get(box, 4), Get(box, 2), Get(box, 3) };
                }
                else if (f.F == 4) req.Add(Encoding.UTF8.GetString(raw, f.Start, f.Len));
                else if (f.F == 5) opt.Add(Encoding.UTF8.GetString(raw, f.Start, f.Len));
                else if (f.F == 16) out_["writingprogram"] = Encoding.UTF8.GetString(raw, f.Start, f.Len);
                else if (f.F == 32) out_["replication_timestamp"] = (long)f.V;
            }
            return out_;
        }
        return new();
    }

    static object? Get(Dictionary<int, long> d, int k) => d.TryGetValue(k, out var v) ? v : null;
}
