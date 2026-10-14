using System.Globalization;
using static Gagame.Harvest.JsonUtil;

namespace Gagame.Harvest.Sources;

/// An ERDDAP tabledap dataset of station time series (cdm_data_type TimeSeries): one request a
/// box, `station,longitude,latitude,time,<quantities>&longitude>=..&latitude<=..&time>=<recent>
/// &orderByMax("station,time")`, the latest row of every station in it. The config maps each
/// quantity to a variable by name or CF standard_name; the server's `info` page supplies what
/// the config leaves out (units, which way a direction points) and is checked against what it
/// declares. THE FIRST COPY: a box's answer is cached under the hour it was asked and the
/// earliest is read; a station's line at a given moment is cached under that moment.
[SourceKind("erddap-tabledap")]
public sealed class ErddapTabledap : IObservationSource
{
    public SourceConfig Config { get; }
    readonly SourceEnv _env;
    readonly string _server, _dataset, _dir, _recent;
    List<Resolved>? _q;
    string _station = "", _lat = "", _lon = "", _time = "";
    string? _name;
    readonly Dictionary<string, Dictionary<string, string>> _latest = new(StringComparer.Ordinal);
    string? _fetched;

    public ErddapTabledap(SourceConfig c, SourceEnv env)
    {
        Config = c;
        _env = env;
        _server = c.Server ?? throw new InvalidDataException($"{c.Path}: needs \"server\"");
        _dataset = c.Dataset ?? throw new InvalidDataException($"{c.Path}: needs \"dataset\"");
        _dir = SourceConfig.Resolve(env.Ctx, c.S("cache"));
        _recent = c.S("recent");
    }

    /// The variables, resolved against the server's metadata on first use (one request, cached).
    List<Resolved> Q()
    {
        if (_q is not null) return _q;
        var meta = Erddap.Info(_env, _server, _dataset, _dir);
        string Col(string q) => Resolved.Of(Config.Var(q) ?? throw new InvalidDataException($"{Config.Path}: needs variables.{q}"), meta, Config.Path, _env.Log).Column;
        _station = Col("station"); _lat = Col("lat"); _lon = Col("lon"); _time = Col("time");
        _name = Config.Var("name") is null ? null : Col("name");
        _q = Observed.Quantities.Where(q => Config.Var(q) is not null).Select(q => Resolved.Of(Config.Var(q)!, meta, Config.Path, _env.Log)).ToList();
        return _q;
    }

    string Columns() => string.Join(",", new[] { _station, _lon, _lat, _time }.Concat(_name is null ? [] : [_name]).Concat(_q!.Select(q => q.Column)));

    public string? FetchedUtc => _fetched;

    void Fetched(string path)
    {
        var t = CompositeRun.FileUtc(path);
        if (_fetched is null || string.CompareOrdinal(t, _fetched) < 0) _fetched = t;
    }

    static string Key(double v) => v.ToString("0.#####", CultureInfo.InvariantCulture);

    public IReadOnlyList<Station> Stations(Box box)
    {
        if (!Config.Coverage.Meets(box.Lon0, box.Lat0, box.Lon1, box.Lat1)) return [];
        Q();
        var prefix = $"latest_{Key(box.Lon0)}_{Key(box.Lat0)}_{Key(box.Lon1)}_{Key(box.Lat1)}_";
        var path = FirstCopy.Earliest(_dir, prefix, ".csv") ?? Path.Combine(_dir, $"{prefix}{DateTime.UtcNow:yyyyMMddHH}.csv");
        var url = $"{_server}/tabledap/{_dataset}.csv?{Erddap.Q(Columns())}" +
                  $"&{Erddap.Q($"{_lon}>={Erddap.F(box.Lon0)}")}&{Erddap.Q($"{_lon}<={Erddap.F(box.Lon1)}")}" +
                  $"&{Erddap.Q($"{_lat}>={Erddap.F(box.Lat0)}")}&{Erddap.Q($"{_lat}<={Erddap.F(box.Lat1)}")}" +
                  $"&{Erddap.Q($"{_time}>={_recent}")}&{Erddap.Q($"orderByMax(\"{_station},{_time}\")")}";
        var rows = Rows(Erddap.GetOrEmpty(_env, url, path, Columns(), "erddap tabledap latest"));
        Fetched(path);
        var out_ = new List<Station>();
        foreach (var r in rows.GroupBy(r => r[_station]).Select(g => g.MaxBy(r => r[_time], StringComparer.Ordinal)!).OrderBy(r => r[_station], StringComparer.Ordinal))
        {
            if (Erddap.Num(r[_lat]) is not double lat || Erddap.Num(r[_lon]) is not double lon) continue;
            _latest[r[_station]] = r;
            out_.Add(new Station(r[_station], lat, lon, _name is null ? null : r[_name]));
        }
        return out_;
    }

    public Dictionary<string, object?>? Observe(Station st, double? at = null)
    {
        Q();
        Dictionary<string, string>? row;
        if (at is double t)
        {
            var iso = FromUnix(t).ToString("yyyy-MM-dd'T'HH:mm:ss'Z'", CultureInfo.InvariantCulture);
            var path = Path.Combine(_dir, $"at_{st.Id}_{FromUnix(t):yyyyMMddHHmm}.csv");
            var url = $"{_server}/tabledap/{_dataset}.csv?{Erddap.Q(Columns())}&{Erddap.Q($"{_station}=\"{st.Id}\"")}&{Erddap.Q($"{_time}={iso}")}";
            row = Rows(Erddap.GetOrEmpty(_env, url, path, Columns(), "erddap tabledap at")).FirstOrDefault();
        }
        else row = _latest.GetValueOrDefault(st.Id);
        if (row is null) return null;
        var obs = DateTime.Parse(row[_time], CultureInfo.InvariantCulture, DateTimeStyles.AdjustToUniversal | DateTimeStyles.AssumeUniversal);
        var d = D(("obs_unix", Unix(obs)));
        foreach (var q in Observed.Quantities)
        {
            var r = _q!.FirstOrDefault(x => x.Quantity == q);
            d[q] = r is null ? null : r.Apply(Erddap.Num(row[r.Column]));
        }
        return d;
    }

    /// The data rows of a .csv answer (row 1 the names, row 2 the units), by column name.
    static List<Dictionary<string, string>> Rows(string text)
    {
        var all = Erddap.Csv(text);
        if (all.Count < 2) return new();
        var names = all[0];
        return all.Skip(2).Where(r => r.Length == names.Length).Select(r => names.Select((n, i) => (n, v: r[i])).ToDictionary(x => x.n, x => x.v)).ToList();
    }
}
