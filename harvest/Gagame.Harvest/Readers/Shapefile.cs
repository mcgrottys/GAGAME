using System.Text;

namespace Gagame.Harvest.Readers;

/// ESRI shapefile (.shp) and dBASE (.dbf) readers as the harvesters read them: points
/// (1/11/21, multipoint 8/18/28), polylines (3/13/23) and polygons (5/15/25), Z/M ignored
/// beyond the parts; every DBF column as trimmed text.
public static class Shapefile
{
    public sealed record Record(int Type, double[] Box, List<List<(double x, double y)>> Parts, List<(double x, double y)> Points);

    /// Every record of a .shp byte run. `box` skips records whose bbox misses it (polylines/polygons).
    public static IEnumerable<Record> Records(byte[] data, Box? box = null)
    {
        if (data.Length < 100 || Bin.I32BE(data, 0) != 9994) yield break;
        var off = 100;
        while (off + 8 <= data.Length)
        {
            var clen = Bin.I32BE(data, off + 4) * 2;
            var recStart = off + 8;
            off += 8 + clen;
            if (clen < 4 || recStart + clen > data.Length) continue;
            var rec = data.AsSpan(recStart, clen);
            var st = Bin.I32(rec, 0);
            if (st is 1 or 11 or 21)
            {
                yield return new Record(st, Array.Empty<double>(), new(), new() { (Bin.F64(rec, 4), Bin.F64(rec, 12)) });
                continue;
            }
            if (st is 8 or 18 or 28)
            {
                var npt = Bin.I32(rec, 36);
                var pts = new List<(double, double)>(npt);
                for (var i = 0; i < npt; i++) pts.Add((Bin.F64(rec, 40 + 16 * i), Bin.F64(rec, 48 + 16 * i)));
                yield return new Record(st, Array.Empty<double>(), new(), pts);
                continue;
            }
            if (st is not (3 or 5 or 13 or 15 or 23 or 25)) continue;
            if (clen < 44) continue;
            var bx = new[] { Bin.F64(rec, 4), Bin.F64(rec, 12), Bin.F64(rec, 20), Bin.F64(rec, 28) };
            if (box is not null && (bx[2] < box.Lon0 || bx[0] > box.Lon1 || bx[3] < box.Lat0 || bx[1] > box.Lat1)) continue;
            int nparts = Bin.I32(rec, 36), npts = Bin.I32(rec, 40);
            var parts = new int[nparts];
            for (var i = 0; i < nparts; i++) parts[i] = Bin.I32(rec, 44 + 4 * i);
            var b = 44 + 4 * nparts;
            var all = new List<(double, double)>(npts);
            for (var i = 0; i < npts; i++) all.Add((Bin.F64(rec, b + 16 * i), Bin.F64(rec, b + 16 * i + 8)));
            var runs = new List<List<(double, double)>>();
            for (var pi = 0; pi < nparts; pi++)
            {
                var s = parts[pi];
                var e = pi + 1 < nparts ? parts[pi + 1] : npts;
                if (e - s >= 2) runs.Add(all.GetRange(s, e - s));
            }
            yield return new Record(st, bx, runs, new());
        }
    }

    public static int ShapeType(byte[] data) => data.Length >= 36 ? Bin.I32(data, 32) : 0;

    /// The DBF rows as {column: text}; up to `maxRows`.
    public static List<Dictionary<string, string>> Dbf(byte[] data, int maxRows = int.MaxValue, Encoding? enc = null)
    {
        enc ??= Encoding.Latin1;
        var out_ = new List<Dictionary<string, string>>();
        if (data.Length < 32) return out_;
        var nrec = (int)Bin.U32(data, 4);
        int hsz = Bin.U16(data, 8), rsz = Bin.U16(data, 10);
        var fields = new List<(string name, int len)>();
        var p = 32;
        while (p < hsz - 1 && data[p] != 0x0D)
        {
            var nameBytes = data.AsSpan(p, 11);
            var z = nameBytes.IndexOf((byte)0);
            var name = Encoding.ASCII.GetString(z < 0 ? nameBytes : nameBytes[..z]);
            fields.Add((name, data[p + 16]));
            p += 32;
        }
        for (var r = 0; r < Math.Min(nrec, maxRows); r++)
        {
            var at = hsz + r * rsz;
            if (at + rsz > data.Length) break;
            if (data[at] == (byte)'*') continue;
            var q = at + 1;
            var vals = new Dictionary<string, string>();
            foreach (var (name, len) in fields)
            {
                vals[name] = enc.GetString(data, q, Math.Min(len, data.Length - q)).Trim();
                q += len;
            }
            out_.Add(vals);
        }
        return out_;
    }
}
