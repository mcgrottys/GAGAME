using System.Globalization;
using System.Text.RegularExpressions;

namespace Gagame.Harvest.Harvesters;

/// NDBC's catalog, shared by the place and the composites: the station table (positions) and the
/// realtime2 listing (which station publishes which file now), both through the polite fetcher.
public static class NdbcStations
{
    public const string Root = "https://www.ndbc.noaa.gov";
    public static string Dir => Path.Combine("cache", "ndbc");

    public sealed record Catalog(Dictionary<string, (double lat, double lon, string name)> Pos, Dictionary<string, HashSet<string>> Files, string Listing);

    /// The station table and the realtime2 listing cached at `listing` (the caller picks the copy:
    /// a place takes one under six hours old, a composite the first it ever took), from `root`.
    public static Catalog Read(Polite polite, string listing, string root = Root)
    {
        var (tab, _) = polite.GetTextP($"{root}/data/stations/station_table.txt", Path.Combine("cache", "coops", "_lists", "ndbc_station_table.txt"));
        var pos = new Dictionary<string, (double, double, string)>();
        foreach (var line in tab.Split('\n'))
        {
            if (line.StartsWith('#') || !line.Contains('|')) continue;
            var f = line.Split('|').Select(c => c.Trim()).ToArray();
            if (f.Length < 7) continue;
            var m = Regex.Match(f[6], @"([\d.]+)\s+([NS])\s+([\d.]+)\s+([EW])");
            if (m.Success)
                pos[f[0].ToUpperInvariant()] = (double.Parse(m.Groups[1].Value, CultureInfo.InvariantCulture) * (m.Groups[2].Value == "N" ? 1 : -1),
                    double.Parse(m.Groups[3].Value, CultureInfo.InvariantCulture) * (m.Groups[4].Value == "E" ? 1 : -1), f[4]);
        }
        Directory.CreateDirectory(Path.GetDirectoryName(Path.GetFullPath(listing))!);
        var (html, _) = polite.GetTextP($"{root}/data/realtime2/", listing);
        var files = new Dictionary<string, HashSet<string>>();
        foreach (Match m in Regex.Matches(html, "href=\"([^\"?/][^\"]*)\""))
        {
            var href = m.Groups[1].Value;
            var dot = href.IndexOf('.');
            var sid = dot < 0 ? href : href[..dot];
            var ext = dot < 0 ? "" : href[(dot + 1)..];
            if (!files.TryGetValue(sid.ToUpperInvariant(), out var s)) files[sid.ToUpperInvariant()] = s = new();
            s.Add(ext);
        }
        return new Catalog(pos, files, listing);
    }

    public static string ListingPath(DateTime utc) => Path.Combine(Dir, $"_realtime2_listing_{utc:yyyyMMddHH}.html");

    /// Stations publishing any of `want` within radiusKm of a point, nearest first.
    public static List<(double km, string sid, string name)> Nearest(Catalog cat, (double lon, double lat) point, HashSet<string> want, int n, double radiusKm)
    {
        var got = new List<(double, string, string)>();
        foreach (var (sid, (lat, lon, name)) in cat.Pos)
        {
            if (!cat.Files.TryGetValue(sid, out var f) || !want.Overlaps(f)) continue;
            var d = Geo.FlatKm(point.lon, point.lat, lon, lat);
            if (d <= radiusKm) got.Add((d, sid, name));
        }
        return got.OrderBy(g => g.Item1).ThenBy(g => g.Item2, StringComparer.Ordinal).Take(n).ToList();
    }

    /// Stations publishing any of `want` inside a box, by id.
    public static List<(string sid, double lat, double lon, string name)> Inside(Catalog cat, Box box, HashSet<string> want) =>
        cat.Pos.Where(kv => box.Contains(kv.Value.lon, kv.Value.lat) && cat.Files.TryGetValue(kv.Key, out var f) && want.Overlaps(f))
            .Select(kv => (kv.Key, kv.Value.lat, kv.Value.lon, kv.Value.name)).OrderBy(x => x.Key, StringComparer.Ordinal).ToList();
}
