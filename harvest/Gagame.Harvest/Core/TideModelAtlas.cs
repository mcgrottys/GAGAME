using static Gagame.Harvest.JsonUtil;

namespace Gagame.Harvest;

/// EOT20, the global ocean-tide atlas (Hart-Davis et al. 2021, doi:10.17882/79489, CC BY 4.0), as
/// `water --eot20` converted it: per constituent a 1/8 deg grid of (re, im) metres, row 0 north,
/// lon 0..360, 0 where land. A node's constants are read by a seek, never the whole grid.
///
/// THE PHASE. EOT20 gives Greenwich phase lags G (re = A cos G, im = A sin G). The engine's tide
/// is h = mean + sum a cos(w (t - epoch) + phi) (sim/TideModel.h), so a constituent becomes
/// a = f A and phi = V0 + u - G: V0 the equilibrium argument at the epoch from the mean longitudes
/// of the Moon, Sun, lunar perigee and solar perigee (Meeus' polynomials in Julian centuries) with
/// Doodson coefficients and quarter-turn constants (the diurnal signs are the ones the instrument
/// in ComposeTides.ModelCheck measured: +90 for O1/P1/Q1, -90 for K1/J1, so V0 + u agrees with
/// fit phase + harcon lag at every station to under a degree), f and u the nodal factor and angle
/// for the fit year's middle (Pugh 1987, Table 4.3) -- the same one-year-constant amplitude and
/// phase a station's fit holds, so a model station and a fitted one are the same kind of thing.
public sealed class TideModelAtlas
{
    /// (name, speed deg/h, Doodson coefficients on T s h p N p', quarter-turn constant in degrees, nodal class)
    public static readonly (string name, double speed, int[] d, double c, string nodal)[] Constituents =
    [
        ("M2", 28.9841042, [2, -2, 2, 0, 0, 0], 0, "M2"),
        ("S2", 30.0, [2, 0, 0, 0, 0, 0], 0, "1"),
        ("N2", 28.4397295, [2, -3, 2, 1, 0, 0], 0, "M2"),
        ("K1", 15.0410686, [1, 0, 1, 0, 0, 0], -90, "K1"),
        ("O1", 13.9430356, [1, -2, 1, 0, 0, 0], 90, "O1"),
        ("K2", 30.0821373, [2, 0, 2, 0, 0, 0], 0, "K2"),
        ("P1", 14.9589314, [1, 0, -1, 0, 0, 0], 90, "1"),
        ("Q1", 13.3986609, [1, -3, 1, 1, 0, 0], 90, "O1"),
        ("2N2", 27.8953548, [2, -4, 2, 2, 0, 0], 0, "M2"),
        ("T2", 29.9589333, [2, 0, -1, 0, 0, 1], 0, "1"),
        ("J1", 15.5854433, [1, 1, 1, -1, 0, 0], -90, "J1"),
        ("M4", 57.9682084, [4, -4, 4, 0, 0, 0], 0, "M4"),
        ("SA", 0.0410686, [0, 0, 1, 0, 0, 0], 0, "1"),
        ("SSA", 0.0821373, [0, 0, 2, 0, 0, 0], 0, "1"),
    ];

    public const double StepDeg = 0.125;
    public const string Id = "eot20";
    readonly string _dir;
    readonly int _rows, _cols;
    readonly FileStream[] _grids;

    public TideModelAtlas(string dir)
    {
        _dir = dir;
        var meta = Load(Path.Combine(dir, "eot20_M2.json"))!;
        _rows = (int)meta.Num("rows");
        _cols = (int)meta.Num("cols");
        _grids = Constituents.Select(c => File.OpenRead(Path.Combine(dir, $"eot20_{c.name}.rg32"))).ToArray();
    }

    public static bool Present(string dir) => Constituents.All(c => File.Exists(Path.Combine(dir, $"eot20_{c.name}.rg32")));

    public IEnumerable<string> Files => Constituents.Select(c => Path.Combine(_dir, $"eot20_{c.name}.rg32"));

    /// The node of the grid nearest a point: (row, col, lat, lon in -180..180).
    public (int r, int c, double lat, double lon) NodeOf(double lat, double lon)
    {
        var r = (int)Math.Round((90.0 - lat) / StepDeg);
        var c = (int)Math.Round((((lon % 360) + 360) % 360) / StepDeg);
        r = Math.Clamp(r, 0, _rows - 1);
        c = Math.Clamp(c, 0, _cols - 1);
        var lo = c * StepDeg;
        return (r, c, 90.0 - r * StepDeg, lo > 180 ? lo - 360 : lo);
    }

    /// The (re, im) of every constituent at a node; null where the atlas has no ocean (all zero).
    public (double re, double im)[]? At(int r, int c)
    {
        var out_ = new (double, double)[_grids.Length];
        Span<byte> b = stackalloc byte[8];
        var any = false;
        lock (_grids)
            for (var k = 0; k < _grids.Length; k++)
            {
                _grids[k].Position = ((long)r * _cols + c) * 8;
                _grids[k].ReadExactly(b);
                double re = Bin.F32(b, 0), im = Bin.F32(b, 4);
                out_[k] = (re, im);
                any |= re != 0 || im != 0;
            }
        return any ? out_ : null;
    }

    // ------------------------------------------------------------------ astronomy

    /// Mean longitudes (deg) of the Moon s, the Sun h, lunar perigee p, the Moon's node N and
    /// solar perigee p' at a moment (Meeus 1998, ch. 22 and 47, linear terms).
    public static (double s, double h, double p, double n, double p1) Longitudes(DateTime utc)
    {
        var T = ((utc - new DateTime(2000, 1, 1, 12, 0, 0, DateTimeKind.Utc)).TotalDays) / 36525.0;
        return (218.3164477 + 481267.88123421 * T, 280.46646 + 36000.76983 * T, 83.3532465 + 4069.0137287 * T,
            125.04452 - 1934.136261 * T, 282.93735 + 1.71946 * T);
    }

    /// V0 (deg) of a constituent at `epoch`: sum of Doodson coefficients x (T, s, h, p, N, p'),
    /// T the hour angle of the mean Sun (180 deg at 0 h UT), plus the quarter-turn constant.
    public static double V0(int k, DateTime epoch)
    {
        var (s, h, p, n, p1) = Longitudes(epoch);
        var T = 180.0 + 15.0 * epoch.TimeOfDay.TotalHours;
        var d = Constituents[k].d;
        return d[0] * T + d[1] * s + d[2] * h + d[3] * p + d[4] * n + d[5] * p1 + Constituents[k].c;
    }

    /// Nodal factor f and angle u (deg) at a moment (Pugh 1987, Table 4.3).
    public static (double f, double u) Nodal(int k, DateTime at)
    {
        var n = Longitudes(at).n * Geo.DegToRad;
        double cn = Math.Cos(n), sn = Math.Sin(n);
        return Constituents[k].nodal switch
        {
            "M2" => (1.0 - 0.037 * cn, -2.1 * sn),
            "K1" => (1.006 + 0.115 * cn, -8.9 * sn),
            "O1" => (1.009 + 0.187 * cn, 10.8 * sn),
            "K2" => (1.024 + 0.286 * cn, -17.7 * sn),
            "J1" => (1.013 + 0.168 * cn, -12.9 * sn),
            "M4" => (Math.Pow(1.0 - 0.037 * cn, 2), -4.2 * sn),
            _ => (1.0, 0.0),
        };
    }

    /// The engine's coefficients for a node's constants: amp f A, phase V0 + u - G, at the fit
    /// year's epoch (1 January 0 h UT) with f and u at its middle.
    public static List<(string name, double speed, double amp, double phase)> Coefficients((double re, double im)[] at, int year)
    {
        var epoch = new DateTime(year, 1, 1, 0, 0, 0, DateTimeKind.Utc);
        var mid = new DateTime(year, 7, 2, 12, 0, 0, DateTimeKind.Utc);
        var out_ = new List<(string, double, double, double)>();
        for (var k = 0; k < Constituents.Length; k++)
        {
            var (re, im) = at[k];
            var a = Math.Sqrt(re * re + im * im);
            if (a < 0.001) continue;
            var g = Math.Atan2(im, re) * Geo.RadToDeg;
            var (f, u) = Nodal(k, mid);
            var ph = (V0(k, epoch) + u - g) * Geo.DegToRad;
            ph = Math.Atan2(Math.Sin(ph), Math.Cos(ph));
            out_.Add((Constituents[k].name, Constituents[k].speed, f * a, ph));
        }
        return out_;
    }
}
