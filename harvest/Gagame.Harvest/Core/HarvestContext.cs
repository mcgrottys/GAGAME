using System.Diagnostics;

namespace Gagame.Harvest;

/// Where a harvest runs: the checkout (data\ and cache\ are relative to it), the big-data
/// drive, and the log. Every path a harvester writes goes through here, so one process can
/// be pointed elsewhere (the proofs, a scratch tree) without touching the harvesters.
public sealed class HarvestContext
{
    public const string UserAgent = "GAGAME/0.1 (hobby ocean simulator; contact: marksmcgrotty@gmail.com)";

    /// The checkout: `data\`, `cache\`, `out\`, `proofs\` live under it. The process's working
    /// directory is set to it, so relative paths in the harvesters mean what they meant.
    public string Root { get; }

    /// The big-data drive (GAGAME_DATACACHE, default D:\DataCache\GAGAME): raw CUDEM tiles,
    /// the CUSP and NHD archives, BlueTopo, eHydro, the OFS nowcasts, EOT20.
    public string DataCache { get; }

    public TextWriter Out { get; }
    readonly Stopwatch _clock = Stopwatch.StartNew();

    public HarvestContext(string root, string? dataCache = null, TextWriter? log = null)
    {
        Root = Path.GetFullPath(root);
        DataCache = dataCache ?? Environment.GetEnvironmentVariable("GAGAME_DATACACHE") ?? @"D:\DataCache\GAGAME";
        Out = log ?? Console.Out;
        Directory.SetCurrentDirectory(Root);
    }

    /// Does the big-data drive exist (its parent folder is there)?
    public bool HasDataCache => Directory.Exists(Path.GetDirectoryName(DataCache.TrimEnd('\\', '/')) ?? DataCache);

    public string Data(params string[] parts) => Path.Combine([Root, "data", .. parts]);
    public string Cache(params string[] parts) => Path.Combine([Root, "cache", .. parts]);
    public string Big(params string[] parts) => Path.Combine([DataCache, .. parts]);

    public void Log(string message)
    {
        Out.WriteLine(message);
        Out.Flush();
    }

    public double Seconds => _clock.Elapsed.TotalSeconds;
}

/// The harvest ended on purpose with a message (Python's SystemExit): the exit code is 1
/// unless `Code` says otherwise, and the message is the last line of the log.
public sealed class HarvestStop : Exception
{
    public int Code { get; }
    public HarvestStop(string message, int code = 1) : base(message) => Code = code;
}
