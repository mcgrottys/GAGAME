using System.Globalization;

namespace Gagame.Harvest;

/// The harvesters' flag idiom, as the Python read sys.argv: `--flag value`, `--switch`,
/// positionals in order. Unknown flags are reported by the harvester, not refused here.
public sealed class Args
{
    readonly string[] _a;
    public Args(string[] args) => _a = args;

    public string[] Raw => _a;

    public bool Has(string flag) => Array.IndexOf(_a, flag) >= 0;

    public string? Opt(string flag, string? dflt = null)
    {
        var i = Array.IndexOf(_a, flag);
        return i >= 0 && i + 1 < _a.Length ? _a[i + 1] : dflt;
    }

    public double Num(string flag, double dflt) =>
        Opt(flag) is string s ? double.Parse(s, CultureInfo.InvariantCulture) : dflt;

    public int Int(string flag, int dflt) =>
        Opt(flag) is string s ? int.Parse(s, CultureInfo.InvariantCulture) : dflt;

    /// `--box lon0,lat0,lon1,lat1`.
    public Box? BoxOpt(string flag = "--box")
    {
        var s = Opt(flag);
        if (s is null) return null;
        var v = Pair(s);
        if (v.Length != 4) throw new HarvestStop($"{flag} wants lon0,lat0,lon1,lat1");
        return new Box(v[0], v[1], v[2], v[3]);
    }

    /// `--anchor lon,lat`.
    public (double lon, double lat)? LonLat(string flag)
    {
        var s = Opt(flag);
        if (s is null) return null;
        var v = Pair(s);
        if (v.Length != 2) throw new HarvestStop($"{flag} wants lon,lat");
        return (v[0], v[1]);
    }

    /// The arguments that are not flags or flag values (the positionals).
    public List<string> Positionals(params string[] valueFlags)
    {
        var out_ = new List<string>();
        for (var i = 0; i < _a.Length; i++)
        {
            if (_a[i].StartsWith("--"))
            {
                if (valueFlags.Contains(_a[i])) i++;
                continue;
            }
            out_.Add(_a[i]);
        }
        return out_;
    }

    public static double[] Pair(string s) =>
        s.Split(',').Select(x => double.Parse(x.Trim(), CultureInfo.InvariantCulture)).ToArray();
}

/// A lon/lat box, the harvesters' `{"lon0", "lat0", "lon1", "lat1"}`.
public sealed record Box(double Lon0, double Lat0, double Lon1, double Lat1)
{
    public double CenterLon => 0.5 * (Lon0 + Lon1);
    public double CenterLat => 0.5 * (Lat0 + Lat1);
    public bool Contains(double lon, double lat) => Lon0 <= lon && lon <= Lon1 && Lat0 <= lat && lat <= Lat1;
    public bool Meets(double lon0, double lat0, double lon1, double lat1) =>
        lon0 < Lon1 && lon1 > Lon0 && lat0 < Lat1 && lat1 > Lat0;
    public Dictionary<string, double> ToJson() => new() { ["lon0"] = Lon0, ["lat0"] = Lat0, ["lon1"] = Lon1, ["lat1"] = Lat1 };
    public override string ToString() => $"{{'lon0': {Lon0.ToString(CultureInfo.InvariantCulture)}, 'lat0': {Lat0.ToString(CultureInfo.InvariantCulture)}, 'lon1': {Lon1.ToString(CultureInfo.InvariantCulture)}, 'lat1': {Lat1.ToString(CultureInfo.InvariantCulture)}}}";
}
