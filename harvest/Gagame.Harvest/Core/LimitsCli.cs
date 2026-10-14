namespace Gagame.Harvest;

/// `harvest limits [--check]`: the sources' limits as Polite resolves them, one line a source;
/// `--check` holds the file to the pace, budgets and bans Polite had as constants before the file
/// (1.1 s a host, 2.5 s on nomads.ncep.noaa.gov, 2000 requests and 8 GB a ledger, 30 s back-off,
/// 50 MB for an unplanned GET, ten Google host fragments) over every host a harvester names.
public static class LimitsCli
{
    /// Every host the harvesters fetch from or cite (grep of harvest/ for https://, 2026-10-13).
    static readonly string[] HarvesterHosts =
    [
        "www.mass.gov", "encdirect.noaa.gov", "coast.noaa.gov", "registry.opendata.aws", "download.geofabrik.de",
        "www.openstreetmap.org", "www.ngdc.noaa.gov", "www.soest.hawaii.edu", "www.ncei.noaa.gov", "pds-geosciences.wustl.edu",
        "coastwatch.noaa.gov", "nomads.ncep.noaa.gov", "tidesandcurrents.noaa.gov", "api.tidesandcurrents.noaa.gov",
        "www.ndbc.noaa.gov", "opendap.co-ops.nos.noaa.gov", "navigation.usace.army.mil", "noaa-ocs-nationalbathymetry-pds.s3.amazonaws.com",
        "services7.arcgis.com", "osmdata.openstreetmap.de", "www.ogc.org", "overturemaps.org", "stac.overturemaps.org", "github.com",
        "www.noaa.gov", "www.weather.gov", "geodesy.noaa.gov", "noaa-nos-coastal-lidar-pds.s3.amazonaws.com", "planet.openstreetmap.org",
        "osmcode.org", "waterservices.usgs.gov", "aisstream.io", "www.usgs.gov", "prd-tnm.s3.amazonaws.com", "doi.org", "www.seanoe.org",
    ];

    static readonly string[] OldBanned = ["google.", "googleapis.", "gstatic.", "ggpht.", "googleusercontent.", "youtube.",
        "withgoogle.", "gvt1.", "blogspot.", "googlevideo."];

    static readonly string[] GoogleHosts = ["tile.googleapis.com", "maps.googleapis.com", "www.google.com", "lh3.ggpht.com",
        "www.gstatic.com", "www.youtube.com", "lh5.googleusercontent.com", "www.google.co.uk"];

    public static int Run(string root, string[] args, TextWriter o)
    {
        var lim = SourceLimits.Load(root);
        o.WriteLine($"{lim.Path}");
        o.WriteLine($"default: {lim.MinIntervalS} s apart, budget {lim.BudgetRequests} requests / {lim.BudgetBytes / 1e9:0.###} GB a ledger, back-off {lim.BackoffS} s, unplanned GET up to {lim.BigBytes / 1e6:0.#} MB");
        foreach (var s in lim.Sources)
            o.WriteLine($"  {s.Id,-20} {(s.Banned ? "BANNED" : $"{s.GapS:0.###} s"),-9} {(s.DayBytes > 0 ? $"{s.DayBytes / 1e9:0.#} GB/day" : ""),-11} {(s.Stated ? "stated " : "ours   ")} {string.Join(" ", s.Hosts)}");
        if (!args.Contains("--check")) return 0;

        int fails = 0;
        void Gate(bool ok, string what) { o.WriteLine($"[limits] {(ok ? "PASS" : "FAIL")} {what}"); if (!ok) fails++; }
        Gate(lim.MinIntervalS == 1.1 && lim.BudgetRequests == 2000 && lim.BudgetBytes == 8e9 && lim.BackoffS == 30 && lim.BigBytes == 50_000_000,
            "default = Polite's old constants (1.1 s, 2000 requests, 8 GB, 30 s, 50 MB)");
        var moved = new List<string>();
        foreach (var h in HarvesterHosts)
        {
            double old = h == "nomads.ncep.noaa.gov" ? 2.5 : 1.1, now = lim.GapOf(h);
            bool oldBan = OldBanned.Any(b => (h + ".").Contains(b));
            if (now != old || oldBan != (lim.BannedBy(h) is not null)) moved.Add($"{h} {old} -> {now} s");
        }
        Gate(moved.Count == 0, $"{HarvesterHosts.Length} harvester hosts keep their old gap and ban{(moved.Count > 0 ? ": " + string.Join("; ", moved) : "")}");
        var notBanned = GoogleHosts.Where(h => lim.BannedBy(h) is null).ToList();
        var oldNotBanned = GoogleHosts.Where(h => !OldBanned.Any(b => (h + ".").Contains(b))).ToList();
        Gate(notBanned.Count == 0 && oldNotBanned.Count == 0, $"{GoogleHosts.Length} Google hosts banned by the file and by the old list{(notBanned.Count > 0 ? "; not: " + string.Join(", ", notBanned) : "")}");
        o.WriteLine(fails == 0 ? "[limits] all gates pass" : $"[limits] {fails} gate(s) FAIL");
        return fails == 0 ? 0 : 1;
    }
}
