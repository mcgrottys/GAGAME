using System.Text.Json.Nodes;
using static Gagame.Harvest.JsonUtil;

namespace Gagame.Harvest.Harvesters;

/// M5: Merrimack River discharge and stage from USGS (harvest_river.py). The legacy
/// instantaneous-values service, no key; one request, cached for 15 minutes.
public sealed class River : IHarvester
{
    public string Name => "river";
    public string Title => "River (USGS)";
    public string What => "Merrimack discharge and stage at Lowell and Lawrence, the solver's upstream boundary.";
    public string[] Outputs => ["data/river/river.json"];
    public Source[] Sources => [new("USGS NWIS instantaneous values", "https://waterservices.usgs.gov/nwis/iv/")];
    public Insert? Insert => null;

    const string Sites = "01100000,01100500";
    const string Url = $"https://waterservices.usgs.gov/nwis/iv/?format=json&sites={Sites}&parameterCd=00060,00065&siteStatus=all";
    const double CfsToCms = 0.0283168;

    public int Run(HarvestContext ctx, string[] argv)
    {
        var outDir = Path.Combine("data", "river");
        var cache = Path.Combine("cache", "river", "iv.json");
        Directory.CreateDirectory(outDir);
        Directory.CreateDirectory(Path.GetDirectoryName(cache)!);
        JsonNode data;
        if (File.Exists(cache) && (DateTime.UtcNow - File.GetLastWriteTimeUtc(cache)).TotalSeconds < 900)
        {
            data = Load(cache)!;
            ctx.Log("using cached USGS response (< 15 min old)");
        }
        else
        {
            try
            {
                var raw = new SimpleFetcher(ctx, 0.0, 1).Download(Url, 60);
                data = Parse(System.Text.Encoding.UTF8.GetString(raw))!;
                File.WriteAllText(cache, data.ToJsonString(), new System.Text.UTF8Encoding(false));
                ctx.Log("fetched USGS instantaneous values");
            }
            catch (Exception e)
            {
                ctx.Log($"USGS legacy service unavailable ({e.Message}) -- likely a decommission brownout; get the free api.waterdata.usgs.gov key for the migration");
                if (File.Exists(cache)) { data = Load(cache)!; ctx.Log("falling back to the stale cache"); }
                else data = new JsonObject();
            }
        }

        var sites = Latest(data);
        var out_ = D(("generated_utc", NowIso()),
            ("source", "USGS waterservices IV (legacy; migrate to api.waterdata.usgs.gov by 2027Q1)"),
            ("sites", sites.Values.ToList()));
        var path = Path.Combine(outDir, "river.json");
        Save(path, out_);
        foreach (var s in sites.Values)
            ctx.Log($"  {s["id"]} {s["name"]}: {s.GetValueOrDefault("discharge_cms") ?? "?"} m3/s, stage {s.GetValueOrDefault("gage_ft") ?? "?"} ft @ {s.GetValueOrDefault("obs_iso") ?? "?"}");
        ctx.Log($"wrote {path}");
        return 0;
    }

    /// Each site's latest discharge (cfs and m3/s) and stage from an IV response, by site code.
    public static Dictionary<string, Dictionary<string, object?>> Latest(JsonNode data)
    {
        var sites = new Dictionary<string, Dictionary<string, object?>>();
        foreach (var ts in data.Obj("value").Arr("timeSeries"))
        {
            var info = ts.Obj("sourceInfo");
            var code = info.Arr("siteCode").FirstOrDefault()?.Str("value") ?? "?";
            var name = info.Str("siteName") ?? code;
            var param = ts.Obj("variable").Arr("variableCode").FirstOrDefault()?.Str("value") ?? "";
            var vals = ts.Arr("values").FirstOrDefault()?.Arr("value") ?? new JsonArray();
            if (vals.Count == 0) continue;
            var last = vals[^1];
            var v = last.Num("value", double.NaN);
            var t = last.Str("dateTime") ?? "";
            if (!sites.TryGetValue(code, out var entry)) sites[code] = entry = D(("id", code), ("name", name));
            if (param == "00060") { entry["discharge_cfs"] = v; entry["discharge_cms"] = Round(v * CfsToCms, 2); }
            else if (param == "00065") entry["gage_ft"] = v;
            entry["obs_iso"] = t;
        }
        return sites;
    }
}
