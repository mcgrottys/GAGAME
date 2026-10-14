using System.Text.Json.Nodes;

namespace Gagame.Harvest;

/// THE SOURCES' LIMITS (docs/FIDELITY.md section 2, `harvest/sources.limits.json`, or the file
/// GAGAME_SOURCE_LIMITS names): what each data source allows, with the page that says so. Polite
/// reads its pace, budgets and bans from here; a host no entry names takes the default.
public sealed class SourceLimits
{
    public const string FileName = "sources.limits.json";

    public sealed class Source
    {
        public string Id = "";
        public string Title = "";
        public string[] Hosts = [];
        public bool Banned;
        public bool Stated;
        public string? Documented;
        /// The least gap between two requests to the source: our own floor, raised to whatever
        /// pace a stated rate (per second, minute, hour or day) implies, whichever is slower.
        public double GapS;
        /// Bytes the source allows in a trailing 24 hours (0 = no such limit).
        public double DayBytes;
    }

    public double MinIntervalS = 1.1;
    public long BudgetRequests = 2000;
    public double BudgetBytes = 8e9;
    public double BackoffS = 30;
    public long BigBytes = 50_000_000;
    public readonly List<Source> Sources = new();
    public string Path = "";

    /// The file: GAGAME_SOURCE_LIMITS, else harvest/sources.limits.json under the checkout.
    public static SourceLimits Load(string root)
    {
        var path = Environment.GetEnvironmentVariable("GAGAME_SOURCE_LIMITS");
        if (string.IsNullOrEmpty(path)) path = System.IO.Path.Combine(root, "harvest", FileName);
        if (!File.Exists(path)) throw new FileNotFoundException($"the sources' limits file is missing: {path} (docs/FIDELITY.md section 2)", path);
        return Parse(File.ReadAllText(path), path);
    }

    public static SourceLimits Parse(string json, string path)
    {
        var doc = JsonNode.Parse(json) ?? throw new InvalidDataException($"{path}: empty");
        var m = new SourceLimits { Path = path };
        if (doc["default"] is JsonObject d)
        {
            m.MinIntervalS = Num(d, "minIntervalS") ?? m.MinIntervalS;
            m.BudgetRequests = (long)(Num(d, "budgetRequests") ?? m.BudgetRequests);
            m.BudgetBytes = Num(d, "budgetBytes") ?? m.BudgetBytes;
            m.BackoffS = Num(d, "backoffS") ?? m.BackoffS;
            m.BigBytes = (long)(Num(d, "bigBytes") ?? m.BigBytes);
        }
        if (doc["sources"] is JsonObject ss)
            foreach (var (id, node) in ss)
            {
                if (node is not JsonObject s) continue;
                var floor = Num(s, "minIntervalS") ?? m.MinIntervalS;
                double gap = Math.Max(floor, Math.Max(
                    Math.Max(Pace(s, "perSecond", 1), Pace(s, "perMinute", 60)),
                    Math.Max(Pace(s, "perHour", 3600), Pace(s, "perDay", 86400))));
                m.Sources.Add(new Source
                {
                    Id = id,
                    Title = s["title"]?.GetValue<string>() ?? id,
                    Hosts = s["hosts"] is JsonArray h ? h.Select(x => x!.GetValue<string>().ToLowerInvariant()).ToArray() : [],
                    Banned = s["banned"]?.GetValue<bool>() ?? false,
                    Stated = s["stated"]?.GetValue<bool>() ?? false,
                    Documented = s["documented"]?.GetValue<string>(),
                    GapS = gap,
                    DayBytes = Num(s, "dayBytes") ?? 0,
                });
            }
        return m;
    }

    static double? Num(JsonObject o, string key) => o[key] is JsonValue v && v.TryGetValue(out double x) ? x : null;
    static double Pace(JsonObject o, string key, double windowS) => Num(o, key) is double n && n > 0 ? windowS / n : 0;

    /// How well a host pattern matches a host: equal or a parent domain (its length), a pattern
    /// ending in '.' anywhere in host + '.' (its length), else -1.
    static int Match(string pattern, string host)
    {
        if (pattern.EndsWith('.')) return (host + ".").Contains(pattern) ? pattern.Length : -1;
        return host == pattern || host.EndsWith("." + pattern) ? pattern.Length : -1;
    }

    /// The most specific source naming the host, or null (the default applies).
    public Source? Of(string host)
    {
        host = host.ToLowerInvariant();
        Source? best = null;
        int bestLen = -1;
        foreach (var s in Sources)
            foreach (var p in s.Hosts)
            {
                var n = Match(p, host);
                if (n > bestLen) { bestLen = n; best = s; }
            }
        return best;
    }

    /// The banned source naming the host, if any: a ban holds however specific another entry
    /// naming the same host is.
    public Source? BannedBy(string host)
    {
        host = host.ToLowerInvariant();
        return Sources.FirstOrDefault(s => s.Banned && s.Hosts.Any(p => Match(p, host) >= 0));
    }

    /// The key requests are paced and counted under: the source's id, else the host itself.
    public string KeyOf(string host) => Of(host)?.Id ?? host;

    public double GapOf(string host) => Of(host)?.GapS ?? MinIntervalS;
}
