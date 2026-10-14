using System.Globalization;
using Gagame.Harvest.Harvesters;
using static Gagame.Harvest.JsonUtil;

namespace Gagame.Harvest.Sources;

/// NDBC's realtime2 text files (www.ndbc.noaa.gov/data/realtime2/): a station table for the
/// positions, a listing for who publishes which file, and per station a whitespace table whose
/// first line names the columns (`#YY MM DD hh mm WDIR WSPD ...`), newest line first, `MM` where
/// a value is missing. The config names the columns of each quantity and the file each is in;
/// `time` is the five columns of year, month, day, hour, minute. A spectrum is the `.data_spec`
/// line of (density, (frequency)) pairs.
[SourceKind("ndbc-realtime2")]
public sealed class NdbcRealtime2 : IObservationSource
{
    public SourceConfig Config { get; }
    readonly SourceEnv _env;
    readonly string _root, _dir, _met;
    readonly string[] _time;
    readonly List<Resolved> _q = new();
    readonly string? _specFile;
    NdbcStations.Catalog? _cat;

    public NdbcRealtime2(SourceConfig c, SourceEnv env)
    {
        Config = c;
        _env = env;
        _root = c.Server ?? throw new InvalidDataException($"{c.Path}: needs \"server\"");
        _dir = SourceConfig.Resolve(env.Ctx, c.S("cache"));
        var time = c.Var("time") ?? throw new InvalidDataException($"{c.Path}: needs variables.time");
        _time = time.Name!.Split(' ', StringSplitOptions.RemoveEmptyEntries);
        if (_time.Length != 5) throw new InvalidDataException($"{c.Path}: variables.time is the five columns of year, month, day, hour, minute");
        _met = time.File ?? "txt";
        foreach (var q in Observed.Quantities)
            if (c.Var(q) is { } spec)
            {
                if ((spec.File ?? _met) != _met) throw new InvalidDataException($"{c.Path}: {q} is in .{spec.File}; the scalar quantities are read from one file, .{_met}");
                _q.Add(Resolved.Of(spec, null, c.Path, env.Log));
            }
        if (c.Var("spectrum") is { } sp)
        {
            Units.Factor("spectrum", sp.Units, c.Path);
            _specFile = sp.File ?? throw new InvalidDataException($"{c.Path}: spectrum names its file (data_spec)");
        }
    }

    NdbcStations.Catalog Catalog()
    {
        if (_cat is not null) return _cat;
        var listing = FirstCopy.Earliest(_dir, "_realtime2_listing_", ".html") ?? Path.Combine(_dir, $"_realtime2_listing_{DateTime.UtcNow:yyyyMMddHH}.html");
        return _cat = NdbcStations.Read(_env.Polite, listing, _root);
    }

    public string? FetchedUtc => _cat is null ? null : CompositeRun.FileUtc(_cat.Listing);

    public IReadOnlyList<Station> Stations(Box box)
    {
        if (!Config.Coverage.Meets(box.Lon0, box.Lat0, box.Lon1, box.Lat1)) return [];
        return NdbcStations.Inside(Catalog(), box, [_met]).Select(s => new Station(s.sid, s.lat, s.lon, s.name)).ToList();
    }

    public Dictionary<string, object?>? Observe(Station st, double? at = null)
    {
        var stamp = DateTime.UtcNow.ToString("yyyyMMddHH", CultureInfo.InvariantCulture);
        var met = FirstCopy.Earliest(_dir, $"{st.Id}_", $".{_met}");
        var b = Line(_env.Polite.GetTextP($"{_root}/data/realtime2/{st.Id}.{_met}", met ?? Path.Combine(_dir, $"{st.Id}_{stamp}.{_met}")).text, at) ?? new();
        if (at is not null || _specFile is null) return b;
        var spec = FirstCopy.Earliest(_dir, $"{st.Id}_", $".{_specFile}");
        if (Catalog().Files.TryGetValue(st.Id, out var fs) && (fs.Contains(_specFile) || spec is not null))
        {
            try
            {
                var sp = Waves.ParseNdbcSpec(_env.Polite.GetTextP($"{_root}/data/realtime2/{st.Id}.{_specFile}", spec ?? Path.Combine(_dir, $"{st.Id}_{stamp}.{_specFile}")).text);
                if (sp is not null) b["spectrum"] = sp;
            }
            catch (Polite.StopException) { throw; }
            catch (Exception e) { _env.Log($"    {st.Id}.{_specFile} unavailable ({e.Message})"); }
        }
        return b;
    }

    /// The newest line (or the line at `at`): its time and every configured quantity, in the
    /// engine's units, by the columns the header names.
    Dictionary<string, object?>? Line(string text, double? at)
    {
        int[]? cols = null;
        int[] qcols = [];
        var need = 0;
        foreach (var ln in text.Split('\n'))
        {
            if (ln.Trim().Length == 0) continue;
            if (ln.StartsWith('#'))
            {
                if (cols is not null) continue;
                var names = ln.TrimStart('#').Split((char[]?)null, StringSplitOptions.RemoveEmptyEntries);
                int Col(string n) => Array.IndexOf(names, n) is var i and >= 0 ? i : throw new InvalidDataException($"{Config.Path}: no column {n} in the header \"{ln.Trim()}\"");
                cols = _time.Select(Col).ToArray();
                qcols = _q.Select(q => Col(q.Column)).ToArray();
                need = cols.Concat(qcols).Max() + 1;
                continue;
            }
            if (cols is null) throw new InvalidDataException($"{Config.Path}: a data line before the header");
            var t = ln.Split((char[]?)null, StringSplitOptions.RemoveEmptyEntries);
            if (t.Length < need) continue;
            double? Num(int i) => double.TryParse(t[i], NumberStyles.Float, CultureInfo.InvariantCulture, out var v) ? v : null;
            var obs = Unix(new DateTime(int.Parse(t[cols[0]]), int.Parse(t[cols[1]]), int.Parse(t[cols[2]]), int.Parse(t[cols[3]]), int.Parse(t[cols[4]]), 0, DateTimeKind.Utc));
            if (at is double want && obs != want) continue;
            var d = D(("obs_unix", obs));
            foreach (var q in Observed.Quantities)
            {
                var k = _q.FindIndex(x => x.Quantity == q);
                d[q] = k < 0 ? null : _q[k].Apply(Num(qcols[k]));
            }
            return d;
        }
        return null;
    }
}

/// The scalar quantities a buoy line carries, in the engine's key order (sim/SeaState.cpp).
public static class Observed
{
    public static readonly string[] Quantities = ["wdir", "wspd", "hs", "dpd", "apd", "mwd"];
}
