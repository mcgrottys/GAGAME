using PureHDF;

namespace Gagame.Harvest.Readers;

/// netCDF-4 (HDF5) files -- the NOS OFS nowcasts on the NODD bucket, the EOT20 grids -- read
/// through PureHDF, which replaces hdf5.py: a variable comes back as doubles in row-major
/// order with its shape, attributes by name.
public sealed class NetCdf4 : IDisposable
{
    readonly IH5Group _f;
    public Dictionary<string, IH5Dataset> Vars { get; } = new();

    public NetCdf4(string path)
    {
        _f = H5File.OpenRead(path);
        foreach (var child in _f.Children())
            if (child is IH5Dataset ds) Vars[ds.Name] = ds;
    }

    public bool Has(string name) => Vars.ContainsKey(name);

    public List<int> Shape(string name) => Vars[name].Space.Dimensions.Select(d => (int)d).ToList();

    /// The whole variable as doubles, row-major (no scale/offset: the OFS files carry none).
    public (double[] values, List<int> shape) Read(string name)
    {
        var ds = Vars[name];
        var shape = Shape(name);
        double[] vals;
        var cls = ds.Type.Class;
        if (cls == H5DataTypeClass.FloatingPoint && ds.Type.Size == 4) vals = ds.Read<float[]>().Select(x => (double)x).ToArray();
        else if (cls == H5DataTypeClass.FloatingPoint) vals = ds.Read<double[]>();
        else if (cls == H5DataTypeClass.FixedPoint)
            vals = ds.Type.Size switch
            {
                1 => ds.Read<sbyte[]>().Select(x => (double)x).ToArray(),
                2 => ds.Read<short[]>().Select(x => (double)x).ToArray(),
                4 => ds.Read<int[]>().Select(x => (double)x).ToArray(),
                _ => ds.Read<long[]>().Select(x => (double)x).ToArray(),
            };
        else throw new InvalidDataException($"{name}: only numeric datasets can be read");
        return (vals, shape);
    }

    /// A string attribute of a variable, or null.
    public string? Attr(string var, string name)
    {
        try
        {
            var ds = Vars[var];
            var a = ds.Attributes().FirstOrDefault(x => x.Name == name);
            return a is null ? null : AttrText(a);
        }
        catch (Exception) { return null; }
    }

    /// A global (root group) string attribute, or null.
    public string? GlobalAttr(string name)
    {
        try
        {
            var a = _f.Attributes().FirstOrDefault(x => x.Name == name);
            return a is null ? null : AttrText(a);
        }
        catch (Exception) { return null; }
    }

    public Dictionary<string, string?> GlobalAttrs(params string[] names)
    {
        var d = new Dictionary<string, string?>();
        foreach (var n in names) { var v = GlobalAttr(n); if (v is not null) d[n] = v; }
        return d;
    }

    static string AttrText(IH5Attribute a)
    {
        try
        {
            if (a.Type.Class == H5DataTypeClass.String || a.Type.Class == H5DataTypeClass.VariableLength)
            {
                var s = a.Read<string[]>();
                return s.Length == 1 ? s[0] : string.Join(" ", s);
            }
            if (a.Type.Class == H5DataTypeClass.FloatingPoint) return string.Join(" ", a.Read<double[]>().Select(x => x.ToString(System.Globalization.CultureInfo.InvariantCulture)));
            if (a.Type.Class == H5DataTypeClass.FixedPoint) return string.Join(" ", a.Read<long[]>().Select(x => x.ToString(System.Globalization.CultureInfo.InvariantCulture)));
        }
        catch (Exception) { }
        try { return a.Read<string>(); } catch (Exception) { return ""; }
    }

    public void Dispose() => (_f as IDisposable)?.Dispose();
}
