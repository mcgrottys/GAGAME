using System.Globalization;
using System.Text.Json.Nodes;
using static Gagame.Harvest.JsonUtil;

namespace Gagame.Harvest.Harvesters;

/// Surveyed aids to navigation from NOAA ENC Direct to GIS (harvest_aton.py): the charted S-57
/// aids of the enc_harbour band as ArcGIS REST JSON, paginated, cached forever on the big-data
/// drive; decoded attributes, WGS84 lon/lat, de-duplicated across overlapping cells.
public sealed class Aton : IHarvester
{
    public string Name => "aton";
    public string Title => "Aids to navigation (ENC Direct)";
    public string What => "The charted S-57 buoys and lights from the live NOAA ENC cells.";
    public string[] Outputs => ["data/gis/aton_ne.json"];
    public Source[] Sources => [new("NOAA ENC Direct to GIS", "https://encdirect.noaa.gov/")];
    public Insert? Insert => null;

    const string Base = "https://encdirect.noaa.gov/arcgis/rest/services/encdirect/enc_harbour/MapServer";
    static readonly Box BoxNe = new(-71.15, 42.20, -70.35, 43.05);
    static readonly (int id, string kind)[] Layers =
    [
        (1, "beacon_lateral"), (2, "beacon_safe_water"), (3, "beacon_special"), (4, "buoy_cardinal"), (5, "buoy_isolated_danger"),
        (6, "buoy_lateral"), (7, "buoy_safe_water"), (8, "buoy_special"), (9, "daymark"), (11, "light"), (12, "light_float"),
    ];
    static readonly Dictionary<int, string> S57Colour = new()
    {
        [1] = "white", [2] = "black", [3] = "red", [4] = "green", [5] = "blue", [6] = "yellow", [7] = "grey", [8] = "brown",
        [9] = "amber", [10] = "violet", [11] = "orange", [12] = "magenta", [13] = "pink",
    };
    static readonly Dictionary<int, string> S57Litchr = new()
    {
        [1] = "F", [2] = "Fl", [3] = "LFl", [4] = "Q", [5] = "VQ", [6] = "UQ", [7] = "Iso", [8] = "Oc", [9] = "IQ", [10] = "IVQ", [11] = "IUQ",
        [12] = "Mo", [13] = "FFl", [14] = "FlLFl", [15] = "OcFl", [16] = "FLFl", [17] = "OcAlt", [18] = "LFlAlt", [19] = "FlAlt",
        [25] = "Q+LFl", [26] = "VQ+LFl", [27] = "Al", [28] = "Al.Oc", [29] = "Al.Fl",
    };

    HarvestContext _ctx = null!;
    SimpleFetcher _fetch = null!;
    string DCache => _ctx.Big("aton");

    (byte[] data, bool cached) Fetch(string url, string cachePath)
    {
        if (File.Exists(cachePath) && new FileInfo(cachePath).Length > 2) return (File.ReadAllBytes(cachePath), true);
        var data = _fetch.Download(url, 60);
        Directory.CreateDirectory(Path.GetDirectoryName(cachePath)!);
        File.WriteAllBytes(cachePath, data);
        Thread.Sleep(600);
        return (data, false);
    }

    (List<JsonNode> feats, bool cached) QueryLayer(int layerId)
    {
        var feats = new List<JsonNode>();
        var offset = 0;
        var cachedAll = true;
        while (true)
        {
            var p = new Dictionary<string, string>
            {
                ["where"] = "1=1",
                ["geometry"] = string.Format(CultureInfo.InvariantCulture, "{0:F6},{1:F6},{2:F6},{3:F6}", BoxNe.Lon0, BoxNe.Lat0, BoxNe.Lon1, BoxNe.Lat1),
                ["geometryType"] = "esriGeometryEnvelope", ["inSR"] = "4326", ["spatialRel"] = "esriSpatialRelIntersects",
                ["outFields"] = "*", ["returnGeometry"] = "true", ["resultOffset"] = offset.ToString(), ["f"] = "json",
            };
            var url = $"{Base}/{layerId}/query?" + string.Join("&", p.Select(kv => Uri.EscapeDataString(kv.Key) + "=" + Uri.EscapeDataString(kv.Value)));
            var (data, cached) = Fetch(url, Path.Combine(DCache, $"enc_harbour_L{layerId}_o{offset}.json"));
            cachedAll &= cached;
            var j = Parse(System.Text.Encoding.UTF8.GetString(data));
            if (j.Has("error")) throw new InvalidOperationException($"layer {layerId} error: {j!["error"]!.ToJsonString()}");
            var page = j.Arr("features").Where(f => f is not null).Select(f => f!).ToList();
            feats.AddRange(page);
            var exceeded = j is JsonObject o && o.TryGetPropertyValue("exceededTransferLimit", out var ex) && ex is JsonValue v && v.TryGetValue<bool>(out var b) && b;
            if (!exceeded || page.Count == 0) return (feats, cachedAll);
            offset += page.Count;
        }
    }

    static string LightCharacteristic(JsonNode at)
    {
        var ch = S57Litchr.GetValueOrDefault((int)at.Num("LITCHR"), "");
        if (ch.Length == 0) return "";
        var grp = (at.Str("SIGGRP") ?? "").Trim();
        if (grp.Length > 0 && grp != "(1)") ch += grp;
        var colRaw = (at.Str("COLOUR") ?? "").Trim();
        var col = colRaw.All(char.IsDigit) && colRaw.Length > 0 ? S57Colour.GetValueOrDefault(int.Parse(colRaw), "") : "";
        if (col.Length > 0) ch += " " + (col switch { "white" => "W", "red" => "R", "green" => "G", "yellow" => "Y", _ => col });
        if (at.NumOrNull("SIGPER") is double per && per != 0) ch += $" {per.ToString("R", CultureInfo.InvariantCulture)}s";
        return ch;
    }

    public int Run(HarvestContext ctx, string[] argv)
    {
        _ctx = ctx;
        _fetch = new SimpleFetcher(ctx, 0.0, 3);
        var all = new List<Dictionary<string, object?>>();
        foreach (var (lid, kind) in Layers.OrderBy(l => l.id))
        {
            List<JsonNode> feats; bool cached;
            try { (feats, cached) = QueryLayer(lid); }
            catch (Exception e) { ctx.Log($"  layer {lid} ({kind}): FAILED {e.Message}"); continue; }
            ctx.Log($"  layer {lid,2} {kind,-22} {feats.Count,3} features{(cached ? "  (cache)" : "")}");
            foreach (var f in feats)
            {
                var at = f.Obj("attributes");
                var g = f.Obj("geometry");
                if (!g.Has("x")) continue;
                var colourRaw = (at.Str("COLOUR") ?? "").Trim();
                var codes = colourRaw.Split(',').Select(c => c.Trim()).Where(c => c.Length > 0 && c.All(char.IsDigit)).Select(int.Parse).ToList();
                all.Add(D(
                    ("lon", Round(g.Num("x"), 7)), ("lat", Round(g.Num("y"), 7)), ("kind", kind),
                    ("name", (at.Str("OBJNAM") ?? "").Trim()),
                    ("colour", codes.Select(c => S57Colour.GetValueOrDefault(c, c.ToString())).ToList()),
                    ("shape", (at.Str("BOYSHP") ?? at.Str("BCNSHP") ?? "").Trim()),
                    ("catlam", Plain(at["CATLAM"])), ("light", LightCharacteristic(at)),
                    ("height_m", Plain(at["HEIGHT"])), ("range_nm", Plain(at["VALNMR"])),
                    ("cell", (at.Str("DSNM") ?? "").Trim()), ("source", (at.Str("SORIND") ?? "").Trim()), ("source_date", (at.Str("SORDAT") ?? "").Trim())));
            }
        }
        var seen = new HashSet<(double, double, string)>();
        var uniq = new List<Dictionary<string, object?>>();
        foreach (var e in all)
        {
            var key = (Round((double)e["lon"]!, 6), Round((double)e["lat"]!, 6), (string)e["kind"]!);
            if (!seen.Add(key)) continue;
            uniq.Add(e);
        }
        var sorted = uniq.OrderBy(e => (string)e["kind"]!, StringComparer.Ordinal).ThenBy(e => (string)e["name"]!, StringComparer.Ordinal).ThenBy(e => (double)e["lat"]!).ToList();
        var out_ = D(
            ("_readme", new List<string>
            {
                "Charted aids to navigation from NOAA ENC Direct (enc_harbour band),",
                "S-57 attributes decoded; positions WGS84 charted/assigned - fixed aids",
                "exact, floating aids swing on moorings. Public domain; not for navigation.",
                "Source service: " + Base,
            }),
            ("box", BoxNe.ToJson()), ("generated_utc", NowStampZ()), ("count", sorted.Count), ("aids", sorted));
        var outPath = Path.Combine("data", "gis", "aton_ne.json");
        Save(outPath, out_);
        ctx.Log($"wrote {outPath}: {sorted.Count} aids ({all.Count} raw, {all.Count - sorted.Count} duplicate cell overlaps)");
        var inlet = sorted.Where(e => -70.88 <= (double)e["lon"]! && (double)e["lon"]! <= -70.75 && 42.76 <= (double)e["lat"]! && (double)e["lat"]! <= 42.86).ToList();
        ctx.Log($"Merrimack box: {inlet.Count} aids");
        foreach (var e in inlet.OrderBy(e => (string)e["kind"]!, StringComparer.Ordinal))
        {
            var name = (string)e["name"]!;
            var colours = string.Join("/", (List<string>)e["colour"]!);
            ctx.Log($"   {(string)e["kind"]!,-20} {(name.Length > 0 ? name : "-"),-28} {(colours.Length > 0 ? colours : "-")} {e["light"]}");
        }
        return 0;
    }
}
