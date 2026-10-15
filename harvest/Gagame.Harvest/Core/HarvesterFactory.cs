using System.Reflection;

namespace Gagame.Harvest;

/// Makes harvesters by name. Every IHarvester in this assembly with a parameterless
/// constructor is registered once; the demo lists them, the console runs one.
public static class HarvesterFactory
{
    static readonly Lazy<Dictionary<string, Func<IHarvester>>> _makers = new(Discover);

    static Dictionary<string, Func<IHarvester>> Discover()
    {
        var d = new Dictionary<string, Func<IHarvester>>(StringComparer.OrdinalIgnoreCase);
        foreach (var t in typeof(HarvesterFactory).Assembly.GetTypes())
        {
            if (t.IsAbstract || !typeof(IHarvester).IsAssignableFrom(t)) continue;
            var ctor = t.GetConstructor(Type.EmptyTypes);
            if (ctor is null) continue;
            var probe = (IHarvester)ctor.Invoke(null);
            d[probe.Name] = () => (IHarvester)ctor.Invoke(null);
        }
        return d;
    }

    /// The names, in the order the README's milestones brought each in.
    public static readonly string[] Order =
    [
        "tides", "waves", "currents", "river", "bathy", "hqbathy", "refbathy", "globe", "gis",
        "survey", "vectors", "aerial", "overlay", "water", "aton", "route", "buildings",
        "planet-buildings", "overture", "land-polygons", "place",
    ];

    public static IEnumerable<string> Names =>
        Order.Where(_makers.Value.ContainsKey).Concat(_makers.Value.Keys.Except(Order, StringComparer.OrdinalIgnoreCase).Order());

    public static bool Has(string name) => _makers.Value.ContainsKey(name);

    public static IHarvester Create(string name) =>
        _makers.Value.TryGetValue(name, out var mk) ? mk()
            : throw new ArgumentException($"no harvester named '{name}' (have: {string.Join(", ", Names)})");

    public static IEnumerable<IHarvester> All() => Names.Select(Create);
}
