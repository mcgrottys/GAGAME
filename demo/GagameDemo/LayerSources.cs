using System.Text.Json.Nodes;

namespace GagameDemo;

/// The data layers the checkout knows and the sources that feed each, read from what is there:
/// every composite product's `<layer>.composite.json` (its `layer` and `inputs`, docs/FIDELITY.md
/// section 3) and every source config under harvest/sources (each with id, title, kind, layer).
/// Nothing is listed that no file names.
public static class LayerSources
{
    public sealed record Feed(string Id, string Title, string From);

    public static SortedDictionary<string, List<Feed>> Read(string root)
    {
        var map = new SortedDictionary<string, List<Feed>>(StringComparer.Ordinal);
        void Add(string layer, Feed f)
        {
            if (!map.TryGetValue(layer, out var l)) map[layer] = l = new();
            if (!l.Any(x => x.Id == f.Id && x.From == f.From)) l.Add(f);
        }
        var composites = Path.Combine(root, "data", "composite");
        if (Directory.Exists(composites))
            foreach (var doc in Directory.EnumerateFiles(composites, "*.composite.json", SearchOption.AllDirectories))
            {
                var j = Parse(doc);
                if (j is not JsonObject || Str(j["layer"]) is not string layer) continue;
                var product = Path.GetFileName(Path.GetDirectoryName(doc))!;
                if (j["inputs"] is JsonArray ins)
                    foreach (var i in ins.OfType<JsonObject>())
                        Add(layer, new Feed(Str(i["id"]) ?? "?", Str(i["title"]) ?? "", "composite " + product));
                else Add(layer, new Feed("(no inputs listed)", "", "composite " + product));
            }
        var sources = Path.Combine(root, "harvest", "sources");
        if (Directory.Exists(sources))
            foreach (var cfg in Directory.EnumerateFiles(sources, "*.json", SearchOption.AllDirectories))
            {
                var j = Parse(cfg);
                // a config is one source or a list of them
                IEnumerable<JsonObject> each = j is JsonArray a ? a.OfType<JsonObject>()
                    : j?["sources"] is JsonArray s ? s.OfType<JsonObject>() : j is JsonObject o ? [o] : [];
                foreach (var src in each)
                    if (Str(src["layer"]) is string layer)
                        Add(layer, new Feed(Str(src["id"]) ?? Path.GetFileNameWithoutExtension(cfg),
                            Str(src["title"]) ?? "", "config " + Path.GetRelativePath(sources, cfg)));
            }
        return map;
    }

    static string? Str(JsonNode? n) => n is JsonValue v && v.TryGetValue(out string? s) ? s : null;

    static JsonNode? Parse(string path)
    {
        try { return JsonNode.Parse(File.ReadAllText(path), documentOptions: new() { CommentHandling = System.Text.Json.JsonCommentHandling.Skip, AllowTrailingCommas = true }); }
        catch (Exception e) when (e is IOException or System.Text.Json.JsonException or InvalidOperationException) { return null; }
    }
}
