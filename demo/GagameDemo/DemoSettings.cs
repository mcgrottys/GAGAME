using System.Text.Json;
using System.Text.Json.Serialization;

namespace GagameDemo;

/// What the demo remembers between runs: %LOCALAPPDATA%\GAGAME\demo.json. The key is NOT here --
/// it lives in the user environment (GAGAME_GOOGLE_MAPS_KEY), the one place the engine reads it.
public sealed class DemoSettings
{
    public string RepoRoot { get; set; } = "";

    /// The banner for missing pieces, dismissed for this list of them (it returns when the list changes).
    public string DismissedBanner { get; set; } = "";

    /// The folder the repo's `cache` junction points at ("" = whatever stands there today).
    public string CacheRoot { get; set; } = "";

    /// The big-data drive the harvesters keep raw downloads on (GAGAME_DATACACHE; "" = D:\DataCache\GAGAME).
    public string DataCache { get; set; } = "";

    /// Google tile limits, passed as `--set streaming.*` on every launch (docs/LAUNCH.md, launch-run).
    public uint TileBudget { get; set; } = 1000;
    public uint DayTiles { get; set; } = 100000;
    public double DayBytes { get; set; } = 5_000_000_000;
    public uint GoogleZoom { get; set; } = 14;

    /// The tree-prune tool's age: a tree stamped older than this lists as STALE (the demo only lists).
    public double PruneAgeDays { get; set; } = 30;

    public string LastScene { get; set; } = "scenes/demos/haulover_portal.json";

    [JsonIgnore]
    public static string FilePath => Path.Combine(
        Environment.GetFolderPath(Environment.SpecialFolder.LocalApplicationData), "GAGAME", "demo.json");

    static readonly JsonSerializerOptions Json = new() { WriteIndented = true };

    public static DemoSettings Load()
    {
        try
        {
            if (File.Exists(FilePath))
                return JsonSerializer.Deserialize<DemoSettings>(File.ReadAllText(FilePath), Json) ?? new DemoSettings();
        }
        catch (Exception) { /* a broken file is a fresh install, not a crash */ }
        return new DemoSettings();
    }

    public void Save()
    {
        Directory.CreateDirectory(Path.GetDirectoryName(FilePath)!);
        File.WriteAllText(FilePath, JsonSerializer.Serialize(this, Json));
    }

    /// The `--set key=value` pairs every launch carries, so the install form's limits override the
    /// scene's defaults. Each pair is two arguments to the engine.
    public IEnumerable<(string flag, string assignment)> EngineSetFlags()
    {
        yield return ("--set", $"streaming.tileBudget={TileBudget}");
        yield return ("--set", $"streaming.dayTiles={DayTiles}");
        yield return ("--set", $"streaming.dayBytes={DayBytes:0}");
        yield return ("--set", $"streaming.googleZoom={GoogleZoom}");
    }
}
