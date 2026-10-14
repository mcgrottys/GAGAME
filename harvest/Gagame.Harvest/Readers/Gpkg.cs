using Microsoft.Data.Sqlite;

namespace Gagame.Harvest.Readers;

/// GeoPackage geometry blobs (the NHD HR HU4 files): the GP header's envelope as the cheap
/// clip, then WKB parsed to lines and polygons (rings outer first), XY/XYZ/XYM/XYZM, either
/// byte order, Multi* and collections. Points are ignored (no mask or stencil use).
public static class Gpkg
{
    public sealed record Geom(string Kind, List<List<(double x, double y)>> Parts);

    /// (minx, maxx, miny, maxy) from the GP header when present.
    public static double[]? Envelope(byte[] blob)
    {
        if (blob.Length < 8 || blob[0] != 'G' || blob[1] != 'P') return null;
        var env = (blob[3] >> 1) & 7;
        if (env == 0) return null;
        return new[] { Bin.F64(blob, 8), Bin.F64(blob, 16), Bin.F64(blob, 24), Bin.F64(blob, 32) };
    }

    public static List<Geom> Geometries(byte[] blob)
    {
        var out_ = new List<Geom>();
        if (blob.Length < 8 || blob[0] != 'G' || blob[1] != 'P') return out_;
        var flags = blob[3];
        var env = (flags >> 1) & 7;
        var envLen = env switch { 0 => 0, 1 => 32, 2 => 48, 3 => 48, 4 => 64, _ => 0 };
        var off = 8 + envLen;

        (Geom? g, int off) Parse(int o)
        {
            var little = blob[o] == 1;
            var t = little ? Bin.U32(blob, o + 1) : Bin.U32BE(blob, o + 1);
            o += 5;
            var b = (int)(t % 1000);
            var dim = (int)(t / 1000);
            var stride = 2 + (dim is 1 or 2 ? 1 : dim == 3 ? 2 : 0);
            double D(int at) => little ? Bin.F64(blob, at) : System.Buffers.Binary.BinaryPrimitives.ReadDoubleBigEndian(blob.AsSpan(at));
            uint N(int at) => little ? Bin.U32(blob, at) : Bin.U32BE(blob, at);
            (List<(double, double)> pts, int o2) Points(int at, int n)
            {
                var pts = new List<(double, double)>(n);
                for (var i = 0; i < n; i++) pts.Add((D(at + 8 * (i * stride)), D(at + 8 * (i * stride + 1))));
                return (pts, at + 8 * n * stride);
            }
            switch (b)
            {
                case 1: return (null, o + 8 * stride);
                case 2:
                    {
                        var n = (int)N(o);
                        var (pts, o2) = Points(o + 4, n);
                        return (new Geom("line", new() { pts }), o2);
                    }
                case 3:
                    {
                        var nr = (int)N(o);
                        o += 4;
                        var rings = new List<List<(double, double)>>();
                        for (var i = 0; i < nr; i++)
                        {
                            var n = (int)N(o);
                            var (pts, o2) = Points(o + 4, n);
                            rings.Add(pts);
                            o = o2;
                        }
                        return (new Geom("poly", rings), o);
                    }
                case 4: case 5: case 6: case 7:
                    {
                        var n = (int)N(o);
                        o += 4;
                        for (var i = 0; i < n; i++)
                        {
                            var (g, o2) = Parse(o);
                            if (g is not null) out_.Add(g);
                            o = o2;
                        }
                        return (null, o);
                    }
                default: return (null, o);
            }
        }

        var (g0, _) = Parse(off);
        if (g0 is not null) out_.Add(g0);
        return out_;
    }

    /// Feature tables, their geometry column, and an fcode column if any.
    public static SqliteConnection Open(string path)
    {
        var c = new SqliteConnection(new SqliteConnectionStringBuilder { DataSource = path, Mode = SqliteOpenMode.ReadOnly }.ToString());
        c.Open();
        return c;
    }

    public static Dictionary<string, string> FeatureTables(SqliteConnection db)
    {
        var out_ = new Dictionary<string, string>(StringComparer.OrdinalIgnoreCase);
        using var cmd = db.CreateCommand();
        cmd.CommandText = "SELECT table_name FROM gpkg_contents WHERE data_type='features'";
        using var r = cmd.ExecuteReader();
        while (r.Read()) out_[r.GetString(0)] = r.GetString(0);
        return out_;
    }

    public static Dictionary<string, string> GeometryColumns(SqliteConnection db)
    {
        var out_ = new Dictionary<string, string>(StringComparer.OrdinalIgnoreCase);
        using var cmd = db.CreateCommand();
        cmd.CommandText = "SELECT table_name, column_name FROM gpkg_geometry_columns";
        using var r = cmd.ExecuteReader();
        while (r.Read()) out_[r.GetString(0)] = r.GetString(1);
        return out_;
    }

    public static string? ColumnNamed(SqliteConnection db, string table, string name)
    {
        using var cmd = db.CreateCommand();
        cmd.CommandText = $"PRAGMA table_info(\"{table}\")";
        using var r = cmd.ExecuteReader();
        while (r.Read())
            if (string.Equals(r.GetString(1), name, StringComparison.OrdinalIgnoreCase)) return r.GetString(1);
        return null;
    }
}
