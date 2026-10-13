using Gagame.Harvest;

namespace GagameDemo;

/// One line of the install form's data list: a product the engine reads, the harvester that
/// writes it (by name, from the factory), where it comes from, and the custom insert it
/// supports. The catalog IS the factory; the one line that is not a harvester is the Google
/// imagery the engine fetches itself under the key.
public sealed record Dataset(
    string Name,
    string What,
    string? Script,
    string[] DefaultArgs,
    string[] Outputs,
    (string label, string url)[] Sources,
    Insert? Insert = null,
    string? Note = null)
{
    public bool Present(Repo repo) =>
        Outputs.Length > 0 && Outputs.All(o => File.Exists(Path.Combine(repo.Root, o)));

    public string Status(Repo repo)
    {
        if (Script is null) return string.IsNullOrEmpty(Repo.ReadKey()) ? "no key" : "keyed";
        if (Outputs.Length == 0) return "(see log)";
        var n = Outputs.Count(o => File.Exists(Path.Combine(repo.Root, o)));
        return n == Outputs.Length ? "present" : n == 0 ? "missing" : $"{n}/{Outputs.Length}";
    }

    public static Dataset Of(IHarvester h) => new(h.Title, h.What, h.Name, h.DefaultArgs, h.Outputs,
        h.Sources.Select(s => (s.Label, s.Url)).ToArray(), h.Insert, h.Note);
}

public static class Datasets
{
    static readonly Dataset Google = new("Google Map Tiles (imagery)",
        "Satellite imagery fetched at run time under your key and the day caps set above; without a key the Earth keeps its procedural look.",
        null, [], [],
        [("Get a Map Tiles API key", "https://developers.google.com/maps/documentation/tile/get-api-key"),
         ("Map Tiles API terms", "https://developers.google.com/maps/documentation/tile/policies")]);

    /// The factory's harvesters in milestone order, then the keyed imagery.
    public static Dataset[] All { get; } = HarvesterFactory.All().Select(Dataset.Of).Append(Google).ToArray();

    /// The ones a fresh clone needs for the shipped scenes, in the order to fetch them.
    public static readonly string[] Required = ["tides", "waves", "currents", "river", "bathy", "globe", "gis", "vectors", "overlay", "water"];
}
