using System.Buffers.Binary;
using System.Text;

namespace Gagame.Harvest.Readers;

/// Minimal classic NetCDF (v1/v2) reader (netcdf3.py): dimensions, attributes (enough for
/// scale_factor / add_offset / _FillValue), fixed and record variables. netCDF-4 is NetCdf4.
public sealed class NetCdf3
{
    public sealed class Var
    {
        public string Name = "";
        public List<string> Dims = new();
        public List<int> Shape = new();
        public int NcType;
        public long VSize, Begin;
        public Dictionary<string, object?> Atts = new();
        public bool IsRecord;
    }

    const int NcByte = 1, NcChar = 2, NcShort = 3, NcInt = 4, NcFloat = 5, NcDouble = 6;
    static readonly int[] Sizes = { 0, 1, 1, 2, 4, 4, 8 };

    readonly byte[] _d;
    int _pos;
    public int Version;
    public int NumRecs;
    public List<string> DimNames = new();
    public List<int> DimSizes = new();
    public Dictionary<string, object?> GAtts = new();
    public Dictionary<string, Var> Vars = new();
    public long RecordStride;

    public NetCdf3(byte[] data)
    {
        _d = data;
        if (!(data[0] == 'C' && data[1] == 'D' && data[2] == 'F') || data[3] is not (1 or 2))
            throw new InvalidDataException("not a classic NetCDF file (netCDF-4/HDF5 is not supported here)");
        Version = data[3];
        _pos = 4;
        NumRecs = (int)U32();
        ReadDims();
        GAtts = ReadAtts();
        ReadVars();
    }

    uint U32() { var v = BinaryPrimitives.ReadUInt32BigEndian(_d.AsSpan(_pos)); _pos += 4; return v; }
    ulong U64() { var v = BinaryPrimitives.ReadUInt64BigEndian(_d.AsSpan(_pos)); _pos += 8; return v; }

    string Name()
    {
        var n = (int)U32();
        var s = Encoding.UTF8.GetString(_d, _pos, n);
        _pos += (n + 3) / 4 * 4;
        return s;
    }

    void ReadDims()
    {
        var tag = U32(); var count = U32();
        if (tag == 0 && count == 0) return;
        if (tag != 0x0A) throw new InvalidDataException($"bad dim tag {tag:x}");
        for (var i = 0; i < count; i++) { DimNames.Add(Name()); DimSizes.Add((int)U32()); }
    }

    Dictionary<string, object?> ReadAtts()
    {
        var tag = U32(); var count = U32();
        var atts = new Dictionary<string, object?>();
        if (tag == 0 && count == 0) return atts;
        if (tag != 0x0C) throw new InvalidDataException($"bad att tag {tag:x}");
        for (var i = 0; i < count; i++)
        {
            var name = Name();
            var t = (int)U32();
            var n = (int)U32();
            var size = Sizes[t] * n;
            var raw = _d.AsSpan(_pos, size);
            _pos += (size + 3) / 4 * 4;
            if (t == NcChar) atts[name] = Encoding.UTF8.GetString(raw).TrimEnd('\0');
            else
            {
                var vals = Unpack(t, raw, n);
                atts[name] = n == 1 ? vals[0] : vals.ToList();
            }
        }
        return atts;
    }

    static double[] Unpack(int t, ReadOnlySpan<byte> raw, int n)
    {
        var out_ = new double[n];
        for (var i = 0; i < n; i++)
            out_[i] = t switch
            {
                NcByte => (sbyte)raw[i],
                NcShort => BinaryPrimitives.ReadInt16BigEndian(raw.Slice(2 * i)),
                NcInt => BinaryPrimitives.ReadInt32BigEndian(raw.Slice(4 * i)),
                NcFloat => BinaryPrimitives.ReadSingleBigEndian(raw.Slice(4 * i)),
                NcDouble => BinaryPrimitives.ReadDoubleBigEndian(raw.Slice(8 * i)),
                _ => throw new InvalidDataException($"nc type {t}"),
            };
        return out_;
    }

    void ReadVars()
    {
        var tag = U32(); var count = U32();
        if (tag == 0 && count == 0) return;
        if (tag != 0x0B) throw new InvalidDataException($"bad var tag {tag:x}");
        for (var i = 0; i < count; i++)
        {
            var v = new Var { Name = Name() };
            var nd = (int)U32();
            var dimids = new int[nd];
            for (var k = 0; k < nd; k++) dimids[k] = (int)U32();
            v.Atts = ReadAtts();
            v.NcType = (int)U32();
            v.VSize = U32();
            v.Begin = Version == 2 ? (long)U64() : U32();
            v.Dims = dimids.Select(d => DimNames[d]).ToList();
            v.Shape = dimids.Select(d => DimSizes[d]).ToList();
            v.IsRecord = v.Shape.Count > 0 && v.Shape[0] == 0;
            if (v.IsRecord) v.Shape[0] = NumRecs;
            Vars[v.Name] = v;
        }
        RecordStride = Vars.Values.Where(x => x.IsRecord).Sum(x => x.VSize);
    }

    static double AttNum(object? a, double dflt) => a switch
    {
        double d => d,
        List<double> l when l.Count > 0 => l[0],
        _ => dflt,
    };

    /// The whole variable as doubles, row-major, with scale/offset/fill applied (fill -> NaN).
    public (double[] values, List<int> shape) Read(string name)
    {
        var v = Vars[name];
        var t = v.NcType;
        if (t == NcChar) throw new InvalidDataException("char variables not supported");
        var esize = Sizes[t];
        long perRecord = 1;
        var dims = v.IsRecord ? v.Shape.Skip(1) : v.Shape;
        foreach (var s in dims) perRecord *= s;
        double[] out_;
        if (v.IsRecord)
        {
            out_ = new double[perRecord * v.Shape[0]];
            for (var r = 0; r < v.Shape[0]; r++)
            {
                var off = v.Begin + r * RecordStride;
                Unpack(t, _d.AsSpan((int)off, (int)(perRecord * esize)), (int)perRecord).CopyTo(out_, r * perRecord);
            }
        }
        else out_ = Unpack(t, _d.AsSpan((int)v.Begin, (int)(perRecord * esize)), (int)perRecord);

        var scale = AttNum(v.Atts.GetValueOrDefault("scale_factor"), 1.0);
        var offset = AttNum(v.Atts.GetValueOrDefault("add_offset"), 0.0);
        var fillA = v.Atts.GetValueOrDefault("_FillValue") ?? v.Atts.GetValueOrDefault("missing_value");
        double? fill = fillA is null ? null : AttNum(fillA, double.NaN);
        if (scale != 1.0 || offset != 0.0 || fill is not null)
            for (var i = 0; i < out_.Length; i++)
                out_[i] = fill is double f && out_[i] == f ? double.NaN : out_[i] * scale + offset;
        return (out_, v.Shape);
    }
}
