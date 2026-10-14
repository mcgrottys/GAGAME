using System.Globalization;
using System.Text.Json.Nodes;

namespace Gagame.Harvest;

/// ONE SOURCE, ONE FILE (docs/FIDELITY.md 7): `harvest/sources/<layer>/<id>.json`. The file says
/// which reader (`kind`) opens the source, where it lives, and how ITS schema maps to the
/// engine's quantities: the variable each quantity is (by name, or by CF standard_name), the
/// units and conventions the source writes it in, the vertical datum and native grain of a
/// relief. A researcher's schema is absorbed here; the readers never branch on a source's id.
public sealed class SourceConfig
{
    public string Path = "";
    public string Layer = "";
    public string Id = "", Title = "", Label = "", Kind = "";
    public string? Server, Dataset, Url;
    /// At equal grain (relief) or for the same station (observations) the lower rank answers.
    public int Rank;
    public string License = "", Cite = "";
    public Box Coverage = new(-180, -90, 180, 90);
    public readonly Dictionary<string, VarSpec> Vars = new();
    public JsonObject Node = new();

    // relief
    public string? Datum;
    public int VerticalEpsg;
    public double StepDeg;

    /// One quantity of the engine read from the source: the variable's literal name, or the one
    /// variable whose CF standard_name is `StandardName`; the units the source writes it in (else
    /// the server's own metadata); for a direction, which way it points and its north.
    public sealed class VarSpec
    {
        public string Quantity = "";
        public string? Name, StandardName, Units, Direction, North, File;
    }

    public string S(string key, string? dflt = null) => Node[key] is JsonValue v && v.TryGetValue(out string? s) ? s : dflt ?? throw new InvalidDataException($"{Path}: needs \"{key}\"");
    public string? SOpt(string key) => Node[key] is JsonValue v && v.TryGetValue(out string? s) ? s : null;
    public double N(string key) => Node[key] is JsonValue v && v.TryGetValue(out double d) ? d : throw new InvalidDataException($"{Path}: needs the number \"{key}\"");
    public double[] Nums(string key) => Node[key] is JsonArray a ? a.Select(x => x!.GetValue<double>()).ToArray() : throw new InvalidDataException($"{Path}: needs the array \"{key}\"");
    public string[] Strs(string key) => Node[key] switch
    {
        JsonArray a => a.Select(x => x!.GetValue<string>()).ToArray(),
        JsonValue v when v.TryGetValue(out string? s) => [s],
        _ => throw new InvalidDataException($"{Path}: needs \"{key}\" (a string or a list of them)"),
    };

    public VarSpec? Var(string quantity) => Vars.GetValueOrDefault(quantity);

    public static SourceConfig Load(string path, string layer)
    {
        var node = JsonUtil.Load(path) as JsonObject ?? throw new InvalidDataException($"{path}: not a JSON object");
        var c = new SourceConfig { Path = path, Layer = layer, Node = node };
        c.Id = c.S("id");
        c.Title = c.S("title");
        c.Label = c.S("label", c.Title);
        c.Kind = c.S("kind");
        c.Server = c.SOpt("server")?.TrimEnd('/');
        c.Dataset = c.SOpt("dataset");
        c.Url = c.SOpt("url");
        c.Rank = node["rank"] is JsonValue r ? (int)r.GetValue<double>() : 0;
        c.License = c.S("license");
        c.Cite = c.S("cite");
        if (node["coverage"] is JsonArray cov) { var b = c.Nums("coverage"); c.Coverage = new Box(b[0], b[1], b[2], b[3]); }
        if (node["variables"] is JsonObject vars)
            foreach (var (q, v) in vars)
            {
                var spec = new VarSpec { Quantity = q };
                if (v is JsonValue sv) spec.Name = sv.GetValue<string>();
                else if (v is JsonObject o)
                {
                    string? G(string k) => o[k] is JsonValue x && x.TryGetValue(out string? s) ? s : null;
                    spec.Name = G("name"); spec.StandardName = G("standard_name"); spec.Units = G("units");
                    spec.Direction = G("direction"); spec.North = G("north"); spec.File = G("file");
                    if (spec.Direction is not (null or "from" or "toward")) throw new InvalidDataException($"{path}: {q}.direction is \"from\" or \"toward\"");
                    if (spec.North is not (null or "true")) throw new InvalidDataException($"{path}: {q}.north \"{spec.North}\": only true north is read (a magnetic north needs a declination model the harvest does not have)");
                }
                if (spec.Name is null && spec.StandardName is null) throw new InvalidDataException($"{path}: {q} names neither a variable nor a standard_name");
                c.Vars[q] = spec;
            }
        if (layer == "relief")
        {
            c.Datum = c.S("datum");
            c.VerticalEpsg = (int)c.N("vertical_epsg");
            c.StepDeg = ArcSec(node["step_arcsec"], path) / 3600.0;
        }
        return c;
    }

    /// `15` or `"1/9"`: the step in arc-seconds, as the division the step is (1/9" is 1.0/9.0,
    /// so the degrees are the same double the hard-coded sources computed).
    static double ArcSec(JsonNode? n, string path)
    {
        if (n is JsonValue v && v.TryGetValue(out double d)) return d;
        if (n is JsonValue s && s.TryGetValue(out string? t) && t.Split('/') is [var a, var b])
            return double.Parse(a, CultureInfo.InvariantCulture) / double.Parse(b, CultureInfo.InvariantCulture);
        throw new InvalidDataException($"{path}: needs \"step_arcsec\" (a number or \"a/b\")");
    }

    /// A path template of the config: `{datacache}` the big-data drive, `{root}` the checkout,
    /// then the caller's own fields; relative paths are the checkout's. Absolute, normalized.
    public static string Resolve(HarvestContext ctx, string template, params (string k, string v)[] fields)
    {
        var s = template.Replace("{datacache}", ctx.DataCache.TrimEnd('\\', '/')).Replace("{root}", ctx.Root);
        foreach (var (k, v) in fields) s = s.Replace("{" + k + "}", v);
        return System.IO.Path.GetFullPath(System.IO.Path.Combine(ctx.Root, s.Replace('/', System.IO.Path.DirectorySeparatorChar)));
    }
}

/// THE ENGINE'S UNITS, and a source's turned into them: one factor a unit (no offsets: none of
/// the quantities read has a shifted zero), a direction turned to "from, true north".
public static class Units
{
    /// The engine's unit of each observed quantity (sim/SeaState.cpp's buoy keys).
    public static readonly Dictionary<string, string> Engine = new()
    {
        ["hs"] = "m", ["dpd"] = "s", ["apd"] = "s", ["wspd"] = "m s-1", ["wdir"] = "degrees true, from", ["mwd"] = "degrees true, from",
        ["spectrum"] = "m2 Hz-1", ["elevation"] = "m",
    };

    static string Key(string u) => new(u.ToLowerInvariant().Where(ch => !char.IsWhiteSpace(ch) && ch != '_' && ch != '.').ToArray());

    static readonly Dictionary<string, double> Factors = new()
    {
        ["m"] = 1, ["meter"] = 1, ["meters"] = 1, ["metre"] = 1, ["metres"] = 1, ["cm"] = 0.01, ["ft"] = 0.3048, ["feet"] = 0.3048, ["foot"] = 0.3048,
        ["s"] = 1, ["sec"] = 1, ["second"] = 1, ["seconds"] = 1,
        ["m/s"] = 1, ["ms-1"] = 1, ["mps"] = 1, ["meterspersecond"] = 1, ["knots"] = 1852.0 / 3600.0, ["knot"] = 1852.0 / 3600.0, ["kt"] = 1852.0 / 3600.0, ["kts"] = 1852.0 / 3600.0,
        ["km/h"] = 1 / 3.6, ["kmh-1"] = 1 / 3.6, ["mph"] = 0.44704,
        ["degt"] = 1, ["degreestrue"] = 1, ["deg"] = 1, ["degree"] = 1, ["degrees"] = 1,
        ["m2/hz"] = 1, ["m2hz-1"] = 1, ["m^2/hz"] = 1,
    };

    /// The factor that takes `units` to the engine's unit, or a refusal naming the unit.
    public static double Factor(string quantity, string? units, string where)
    {
        if (units is null) throw new InvalidDataException($"{where}: {quantity} declares no units and the server gave none");
        var k = Key(units);
        if (k is "degm" or "degreesmagnetic" or "degmag") throw new InvalidDataException($"{where}: {quantity} is in {units}: a magnetic north needs a declination model the harvest does not have");
        return Factors.TryGetValue(k, out var f) ? f : throw new InvalidDataException($"{where}: {quantity} in \"{units}\": not a unit the harvest converts (it knows {string.Join(", ", Factors.Keys)})");
    }

    /// The direction a standard_name says, or null: CF names a direction `*_from_direction*` or `*_to_direction*`.
    public static string? DirectionOf(string? standardName) =>
        standardName is null ? null : standardName.Contains("_from_direction") ? "from" : standardName.Contains("_to_direction") ? "toward" : null;
}
