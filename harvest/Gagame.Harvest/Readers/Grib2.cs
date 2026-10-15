using System.Buffers.Binary;

namespace Gagame.Harvest.Readers;

/// Minimal GRIB2 reader (grib2.py): edition 2, grid template 3.0 (regular lat/lon), data
/// representation 5.0 (simple), 5.2 (complex) and 5.3 (complex + spatial differencing),
/// section-6 bitmaps; product templates 4.0/4.1/4.8. Anything else raises rather than guesses.
public sealed class Grib2Message
{
    public int Discipline, ParamCategory, ParamNumber, LevelType;
    public long LevelValue, ForecastHour;
    public int Ni, Nj;
    public double Lat1, Lon1, Lat2, Dlat, Dlon;
    public int Scan;
    /// Row-major as stored; NaN where the bitmap masks (Python's None).
    public double[] Values = Array.Empty<double>();

    public (int, int, int, int, long) Key => (Discipline, ParamCategory, ParamNumber, LevelType, LevelValue);

    /// Nearest-node value; lonE in [0, 360).
    public double AtLatLon(double lat, double lonE)
    {
        var bestR = 0; var bestD = double.MaxValue;
        for (var r = 0; r < Nj; r++)
        {
            var la = Lat1 + r * ((Scan & 0x40) != 0 ? Dlat : -Dlat);
            var d = Math.Abs(la - lat);
            if (d < bestD) { bestD = d; bestR = r; }
        }
        var bestC = 0; bestD = double.MaxValue;
        for (var c = 0; c < Ni; c++)
        {
            var lo = Mod360(Lon1 + c * ((Scan & 0x80) != 0 ? -Dlon : Dlon));
            var d = Math.Min(Math.Abs(lo - lonE), 360.0 - Math.Abs(lo - lonE));
            if (d < bestD) { bestD = d; bestC = c; }
        }
        return Values[bestR * Ni + bestC];
    }

    public static double Mod360(double x) => ((x % 360.0) + 360.0) % 360.0;
}

public static class Grib2
{
    static ulong U(ReadOnlySpan<byte> b)
    {
        ulong v = 0;
        foreach (var x in b) v = (v << 8) | x;
        return v;
    }

    /// GRIB signed-magnitude integer: high bit is the sign, NOT two's complement.
    static long Sm(ReadOnlySpan<byte> b)
    {
        var v = U(b);
        var bits = b.Length * 8;
        var sign = (v >> (bits - 1)) != 0;
        var mag = (long)(v & ((1UL << (bits - 1)) - 1));
        return sign ? -mag : mag;
    }

    sealed class Bits
    {
        readonly byte[] _d;
        public long Pos;
        public Bits(byte[] d, int byteOff) { _d = d; Pos = byteOff * 8L; }

        public long Read(int nbits)
        {
            if (nbits == 0) return 0;
            long v = 0;
            for (var i = 0; i < nbits; i++)
            {
                var p = Pos + i;
                var bit = (_d[p >> 3] >> (7 - (int)(p & 7))) & 1;
                v = (v << 1) | (long)bit;
            }
            Pos += nbits;
            return v;
        }

        public void Align() => Pos = (Pos + 7) / 8 * 8;
    }

    static double[] DecodeData(byte[] sec5, byte[]? sec6, byte[] sec7, int nPoints)
    {
        var tmpl = (int)U(sec5.AsSpan(9, 2));
        var nValues = (int)U(sec5.AsSpan(5, 4));
        var reference = BinaryPrimitives.ReadSingleBigEndian(sec5.AsSpan(11));
        var binScale = Sm(sec5.AsSpan(15, 2));
        var decScale = Sm(sec5.AsSpan(17, 2));
        int nbits = sec5[19];
        var twoE = Math.Pow(2.0, binScale);
        var tenD = Math.Pow(10.0, -decScale);
        double[] vals;
        if (tmpl == 0)
        {
            var bits = new Bits(sec7, 5);
            vals = new double[nValues];
            for (var i = 0; i < nValues; i++) vals[i] = (reference + bits.Read(nbits) * twoE) * tenD;
        }
        else if (tmpl is 2 or 3)
        {
            if (sec5[22] != 0) throw new InvalidDataException($"missing-value management {sec5[22]} not supported");
            var ng = (int)U(sec5.AsSpan(31, 4));
            int refGw = sec5[35], nbGw = sec5[36];
            var refGl = (long)U(sec5.AsSpan(37, 4));
            int lenInc = sec5[41];
            var lastLen = (long)U(sec5.AsSpan(42, 4));
            int nbSgl = sec5[46];
            var bits = new Bits(sec7, 5);
            var order = 0;
            var extra = new List<long>();
            long gmin = 0;
            if (tmpl == 3)
            {
                order = sec5[47];
                int nbytesExtra = sec5[48];
                if (order is not (1 or 2)) throw new InvalidDataException($"spatial differencing order {order} not supported");
                for (var i = 0; i < order; i++)
                {
                    extra.Add(Sm(sec7.AsSpan((int)(bits.Pos / 8), nbytesExtra)));
                    bits.Pos += nbytesExtra * 8L;
                }
                gmin = Sm(sec7.AsSpan((int)(bits.Pos / 8), nbytesExtra));
                bits.Pos += nbytesExtra * 8L;
            }
            var grefs = new long[ng];
            for (var g = 0; g < ng; g++) grefs[g] = bits.Read(nbits);
            bits.Align();
            var gwidths = new int[ng];
            for (var g = 0; g < ng; g++) gwidths[g] = refGw + (int)bits.Read(nbGw);
            bits.Align();
            var glens = new long[ng];
            for (var g = 0; g < ng; g++) glens[g] = refGl + lenInc * bits.Read(nbSgl);
            bits.Align();
            if (ng > 0) glens[ng - 1] = lastLen;
            var raw = new List<long>(nValues);
            for (var g = 0; g < ng; g++)
            {
                var w = gwidths[g];
                var r0 = grefs[g];
                for (long k = 0; k < glens[g]; k++) raw.Add(r0 + (w > 0 ? bits.Read(w) : 0));
            }
            if (raw.Count > nValues) raw.RemoveRange(nValues, raw.Count - nValues);
            if (tmpl == 3)
            {
                var x = raw.ToArray();
                for (var i = order; i < x.Length; i++) x[i] += gmin;
                for (var i = 0; i < order && i < x.Length; i++) x[i] = extra[i];
                if (order == 1) for (var i = 1; i < x.Length; i++) x[i] += x[i - 1];
                else for (var i = 2; i < x.Length; i++) x[i] += 2 * x[i - 1] - x[i - 2];
                raw = x.ToList();
            }
            vals = raw.Select(x => (reference + x * twoE) * tenD).ToArray();
        }
        else throw new InvalidDataException($"data representation template 5.{tmpl} not supported (only 5.0 / 5.2 / 5.3)");

        var ind = sec6 is not null ? sec6[5] : 255;
        if (ind == 255)
        {
            var out_ = new double[nPoints];
            for (var i = 0; i < nPoints; i++) out_[i] = i < vals.Length ? vals[i] : double.NaN;
            return out_;
        }
        if (ind != 0) throw new InvalidDataException($"bitmap indicator {ind} not supported");
        var bm = new Bits(sec6!, 6);
        var res = new double[nPoints];
        var it = 0;
        for (var i = 0; i < nPoints; i++)
            res[i] = bm.Read(1) != 0 ? (it < vals.Length ? vals[it++] : double.NaN) : double.NaN;
        return res;
    }

    static int Find(byte[] data, int from)
    {
        for (var i = from; i + 4 <= data.Length; i++)
            if (data[i] == 'G' && data[i + 1] == 'R' && data[i + 2] == 'I' && data[i + 3] == 'B') return i;
        return -1;
    }

    /// Parse every GRIB2 message in a byte string.
    public static List<Grib2Message> ReadMessages(byte[] data)
    {
        var msgs = new List<Grib2Message>();
        var pos = 0;
        while (pos + 16 <= data.Length)
        {
            var idx = Find(data, pos);
            if (idx < 0) break;
            if (data[idx + 7] != 2) throw new InvalidDataException($"GRIB edition {data[idx + 7]} not supported");
            var total = (int)U(data.AsSpan(idx + 8, 8));
            var body = data.AsSpan(idx, total).ToArray();
            if (!(body[^4] == '7' && body[^3] == '7' && body[^2] == '7' && body[^1] == '7')) throw new InvalidDataException("message does not end in 7777");
            var m = new Grib2Message { Discipline = body[6] };
            var p = 16;
            var secs = new Dictionary<int, byte[]>();
            while (p < total - 4)
            {
                var slen = (int)U(body.AsSpan(p, 4));
                int snum = body[p + 4];
                secs[snum] = body.AsSpan(p, slen).ToArray();
                if (snum == 7)
                {
                    var s3 = secs[3]; var s4 = secs[4]; var s5 = secs[5];
                    var gt = (int)U(s3.AsSpan(12, 2));
                    if (gt != 0) throw new InvalidDataException($"grid template 3.{gt} not supported");
                    m.Ni = (int)U(s3.AsSpan(30, 4));
                    m.Nj = (int)U(s3.AsSpan(34, 4));
                    m.Lat1 = Sm(s3.AsSpan(46, 4)) / 1e6;
                    m.Lon1 = Sm(s3.AsSpan(50, 4)) / 1e6;
                    m.Lat2 = Sm(s3.AsSpan(55, 4)) / 1e6;
                    m.Dlon = U(s3.AsSpan(63, 4)) / 1e6;
                    m.Dlat = U(s3.AsSpan(67, 4)) / 1e6;
                    m.Scan = s3[71];
                    var pt = (int)U(s4.AsSpan(7, 2));
                    if (pt is not (0 or 1 or 8)) throw new InvalidDataException($"product template 4.{pt} not supported");
                    m.ParamCategory = s4[9];
                    m.ParamNumber = s4[10];
                    m.ForecastHour = (long)U(s4.AsSpan(18, 4));
                    m.LevelType = s4[22];
                    var sv = (long)U(s4.AsSpan(24, 4));
                    m.LevelValue = s4[23] != 0 ? sv / (long)Math.Pow(10, s4[23]) : sv;
                    m.Values = DecodeData(s5, secs.GetValueOrDefault(6), secs[7], m.Ni * m.Nj);
                    msgs.Add(m);
                    m = new Grib2Message { Discipline = body[6] };
                }
                p += slen;
            }
            pos = idx + total;
        }
        return msgs;
    }
}
