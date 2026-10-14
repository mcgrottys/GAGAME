namespace Gagame.Harvest;

/// What a source reader is handed: where the harvest runs, the polite fetcher, the log.
public sealed record SourceEnv(HarvestContext Ctx, Polite Polite, Action<string> Log);

/// A station of an observation source: its id as the engine keys it, where it stands.
public sealed record Station(string Id, double Lat, double Lon, string? Name);

/// A source of point observations (buoys first): its stations in a box and each one's line in
/// the engine's quantities and units (docs/FIDELITY.md 6: obs_unix, wdir, wspd, hs, dpd, apd,
/// mwd, spectrum). THE FIRST COPY: a station's line is the latest of the earliest copy the cache
/// holds; `at` asks for the line at that moment instead (the agreement gate).
public interface IObservationSource
{
    SourceConfig Config { get; }
    IReadOnlyList<Station> Stations(Box box);
    Dictionary<string, object?>? Observe(Station station, double? at = null);
    /// When the copy the stations were read from was fetched (the input's `fetched_utc`).
    string? FetchedUtc { get; }
}

/// One file of a relief source: a GeoTIFF on disk once fetched (`Dest`), holding `Cell`.
public sealed class ReliefFile
{
    public string Key = "";
    public Box Cell = new(0, 0, 0, 0);
    public string Dest = "";
    public string Url = "";
    public long? Size;
    /// Where the size of an uncached file is learnt (a bucket listing), asked only when it is fetched.
    public Func<long?>? SizeOf;
    /// How the file is made when it is not cached: null is a planned download of `Url` to `Dest`
    /// of `Size` bytes; a subset service (griddap) fetches its answer and writes `Dest` from it.
    public Action<Polite>? Make;
}

/// A source of relief: the files meeting a box, in EPSG:4326 at the config's native step, in
/// the config's vertical datum.
public interface IReliefSource
{
    SourceConfig Config { get; }
    List<ReliefFile> Meets(Box box);
}

/// The reader of one `kind` of source; its constructor takes (SourceConfig, SourceEnv).
[AttributeUsage(AttributeTargets.Class)]
public sealed class SourceKindAttribute(string kind) : Attribute
{
    public string Kind { get; } = kind;
}

/// THE SOURCE FACTORY: every config file under `harvest/sources/<layer>/` is one source; the
/// reader its `kind` names is found among this assembly's [SourceKind] classes and made with it.
/// A source is added by writing a config, not C#, wherever an existing kind reads it.
public static class SourceFactory
{
    public const string Observations = "buoys", Relief = "relief";

    static readonly Lazy<Dictionary<string, Type>> _kinds = new(() =>
        typeof(SourceFactory).Assembly.GetTypes()
            .Select(t => (t, a: (SourceKindAttribute?)Attribute.GetCustomAttribute(t, typeof(SourceKindAttribute))))
            .Where(x => x.a is not null && !x.t.IsAbstract)
            .ToDictionary(x => x.a!.Kind, x => x.t, StringComparer.Ordinal));

    public static IEnumerable<(string kind, string reader, bool observation)> Kinds =>
        _kinds.Value.OrderBy(k => k.Key, StringComparer.Ordinal).Select(k => (k.Key, k.Value.Name, typeof(IObservationSource).IsAssignableFrom(k.Value)));

    public static string Folder(HarvestContext ctx, string layer) => Path.Combine(ctx.Root, "harvest", "sources", layer);

    /// The configs of a layer, every file in its folder or only the ids `only` lists (comma-
    /// separated), ranked: the lower rank first, then the id.
    public static List<SourceConfig> Configs(HarvestContext ctx, string layer, string? only = null)
    {
        var dir = Folder(ctx, layer);
        if (!Directory.Exists(dir)) throw new HarvestStop($"{dir}: no source configs (docs/FIDELITY.md 7)");
        var all = Directory.GetFiles(dir, "*.json").OrderBy(f => f, StringComparer.Ordinal).Select(f => SourceConfig.Load(f, layer)).ToList();
        foreach (var g in all.GroupBy(c => c.Id).Where(g => g.Count() > 1))
            throw new HarvestStop($"source id {g.Key} is in {string.Join(" and ", g.Select(c => c.Path))}");
        if (only is not null)
        {
            var want = only.Split(',', StringSplitOptions.RemoveEmptyEntries | StringSplitOptions.TrimEntries);
            var unknown = want.Where(w => all.All(c => c.Id != w)).ToList();
            if (unknown.Count > 0) throw new HarvestStop($"--sources: no {layer} config with id {string.Join(", ", unknown)} (have: {string.Join(", ", all.Select(c => c.Id))})");
            all = all.Where(c => want.Contains(c.Id)).ToList();
        }
        return all.OrderBy(c => c.Rank).ThenBy(c => c.Id, StringComparer.Ordinal).ToList();
    }

    public static T Make<T>(SourceConfig c, SourceEnv env) where T : class
    {
        if (!_kinds.Value.TryGetValue(c.Kind, out var t))
            throw new HarvestStop($"{c.Path}: no reader of kind \"{c.Kind}\" (have: {string.Join(", ", _kinds.Value.Keys.Order())})");
        if (!typeof(T).IsAssignableFrom(t)) throw new HarvestStop($"{c.Path}: kind \"{c.Kind}\" does not read a {c.Layer} source");
        try { return (T)Activator.CreateInstance(t, c, env)!; }
        catch (System.Reflection.TargetInvocationException e) when (e.InnerException is not null) { throw e.InnerException; }
    }

    public static List<IObservationSource> ObservationSources(SourceEnv env, string? only = null) =>
        Configs(env.Ctx, Observations, only).Select(c => Make<IObservationSource>(c, env)).ToList();

    public static List<IReliefSource> ReliefSources(SourceEnv env, string? only = null) =>
        Configs(env.Ctx, Relief, only).Select(c => Make<IReliefSource>(c, env)).ToList();
}

/// One quantity of a source resolved against the server's own metadata: the variable that holds
/// it, the factor to the engine's unit, and whether its direction is turned (toward -> from).
public sealed record Resolved(string Quantity, string Column, double Factor, bool Toward)
{
    /// The engine's value of a source value (null stays null; a factor of 1 leaves it untouched).
    public double? Apply(double? v)
    {
        if (v is not double x) return null;
        if (Factor != 1) x *= Factor;
        if (Toward) x = (x + 180.0) % 360.0;
        return x;
    }

    /// The variable a spec names, given the server's variables (name -> attributes) or none:
    /// a literal name must exist; a standard_name must be held by exactly ONE variable (NDBC's
    /// DPD and APD share sea_surface_swell_wave_period, so a period is named, not matched). The
    /// units are the spec's, else the server's; a direction is the spec's, else its standard_name's.
    /// A declaration the metadata contradicts is kept and logged: the metadata is not always right
    /// (CoastWatch's MWD says sea_surface_wave_to_direction of a direction FROM, docs/FIDELITY.md 7).
    public static Resolved Of(SourceConfig.VarSpec spec, Dictionary<string, Dictionary<string, string>>? meta, string where, Action<string> log)
    {
        var name = spec.Name;
        if (name is null)
        {
            if (meta is null) throw new InvalidDataException($"{where}: {spec.Quantity} matches standard_name {spec.StandardName} but the source has no metadata to match against");
            var hits = meta.Where(kv => kv.Value.GetValueOrDefault("standard_name") == spec.StandardName).Select(kv => kv.Key).ToList();
            if (hits.Count != 1) throw new InvalidDataException($"{where}: {spec.Quantity}: standard_name {spec.StandardName} is held by {(hits.Count == 0 ? "no variable" : string.Join(", ", hits))}; name the variable");
            name = hits[0];
        }
        var atts = meta is null ? null : meta.TryGetValue(name, out var a) ? a : throw new InvalidDataException($"{where}: {spec.Quantity}: no variable {name} on the server");
        var units = spec.Units ?? atts?.GetValueOrDefault("units");
        var factor = spec.Quantity is "time" or "station" or "lat" or "lon" or "name" ? 1 : Units.Factor(spec.Quantity, units, where);
        var said = Units.DirectionOf(atts?.GetValueOrDefault("standard_name"));
        var dir = spec.Direction ?? said;
        if (spec.Direction is not null && said is not null && said != spec.Direction)
            log($"    {where}: {spec.Quantity} = {name} declared \"{spec.Direction}\", the server's standard_name says \"{said}\"; the declaration holds");
        if (Units.Engine.GetValueOrDefault(spec.Quantity, "").StartsWith("degrees") && dir is null)
            throw new InvalidDataException($"{where}: {spec.Quantity}: neither the config nor the server says whether the direction is from or toward");
        return new Resolved(spec.Quantity, name, factor, dir == "toward");
    }
}
