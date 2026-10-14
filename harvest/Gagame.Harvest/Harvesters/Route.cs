using System.Globalization;
using static Gagame.Harvest.JsonUtil;

namespace Gagame.Harvest.Harvesters;

/// The vqview-inlet AIS traffic lane georeferenced into engine coordinates (harvest_route.py).
/// Not a download: reads ais_route_wide.json from the vqview checkout (VQVIEW env var).
public sealed class Route : IHarvester
{
    public string Name => "route";
    public string Title => "AIS lane (route prior)";
    public string What => "The vqview-inlet traffic lane georeferenced into engine coordinates: the fleet's route and speeds.";
    public string[] Outputs => ["data/gis/route_merrimack.json"];
    public Source[] Sources => [new("aisstream.io (the named live upgrade)", "https://aisstream.io/")];
    public Insert? Insert => null;
    public string? Note => "Reads the vqview-inlet lane file from its own checkout (VQVIEW); not a download.";

    const double FullLon0 = -70.8320, FullLat1 = 42.8250, FullLon1 = -70.7980, FullLat0 = 42.8100;
    const int FullNx = 1851, FullNy = 1106;
    static readonly int[] Crop = [120, 1840, 70, 760];
    const double Cell = 1.5;
    const double KOrgLon = -70.81, KOrgLat = 42.81833, KMPerLon = 81660.0, KMPerLat = 110574.0;

    public int Run(HarvestContext ctx, string[] argv)
    {
        var vq = Environment.GetEnvironmentVariable("VQVIEW") ?? @"C:\vqview-inlet\vqview-inlet";
        var routePath = Path.Combine(vq, "renders", "geotiff", "water", "ais_route_wide.json");
        if (!File.Exists(routePath)) throw new HarvestStop($"source not found: {routePath}");
        var r = Load(routePath)!;
        var dlon = (FullLon1 - FullLon0) / FullNx;
        var dlat = (FullLat1 - FullLat0) / FullNy;
        var cropWLon = FullLon0 + Crop[0] * dlon;
        var cropSLat = FullLat1 - Crop[3] * dlat;
        var pts = new List<Dictionary<string, object?>>();
        foreach (var p in r.Arr("route"))
        {
            double px = p![0]!.GetValue<double>(), pz = p[1]!.GetValue<double>();
            var lon = cropWLon + (px / Cell) * dlon;
            var lat = cropSLat + (pz / Cell) * dlat;
            var wx = (lon - KOrgLon) * KMPerLon;
            var wz = (lat - KOrgLat) * KMPerLat;
            pts.Add(D(("lon", Round(lon, 7)), ("lat", Round(lat, 7)), ("x", Round(wx, 1)), ("z", Round(wz, 1))));
        }
        var length = 0.0;
        for (var i = 1; i < pts.Count; i++)
            length += Math.Sqrt(Math.Pow((double)pts[i]["x"]! - (double)pts[i - 1]["x"]!, 2) + Math.Pow((double)pts[i]["z"]! - (double)pts[i - 1]["z"]!, 2));
        var speeds = r.Has("speeds_ms") ? Plain(r["speeds_ms"]) : new Dictionary<string, object?>();
        var out_ = D(
            ("_readme", new List<string>
            {
                "Traffic-weighted AIS lane for the Merrimack inlet, from vqview-inlet's",
                "9-day / 7087-report climatology; georeferenced from the wide-crop patch",
                "frame via bathy_comp2 bounds. speeds_ms are per-AIS-class mean SOG.",
                "Arc-length parameterize when following (waypoints are even in lon).",
            }),
            ("source", "vqview-inlet ais_route_wide.json (9 days AIS, 7087 reports)"),
            ("length_m", Round(length, 1)), ("speeds_ms", speeds), ("waypoints", pts));
        var outPath = Path.Combine("data", "gis", "route_merrimack.json");
        Save(outPath, out_);
        ctx.Log($"wrote {outPath}: {pts.Count} waypoints, {length:0} m lane");
        var last = pts[^1]; var first = pts[0];
        ctx.Log($"  seaward end: {(double)last["lat"]!:0.00000}, {(double)last["lon"]!:0.00000} (world {(double)last["x"]!:0}, {(double)last["z"]!:0})");
        ctx.Log($"  upriver end: {(double)first["lat"]!:0.00000}, {(double)first["lon"]!:0.00000} (world {(double)first["x"]!:0}, {(double)first["z"]!:0})");
        if (speeds is Dictionary<string, object?> sp)
            ctx.Log("  speeds: " + string.Join(", ", sp.OrderBy(kv => kv.Key, StringComparer.Ordinal).Select(kv => $"{kv.Key} {Convert.ToDouble(kv.Value, CultureInfo.InvariantCulture):0.00}")));
        return 0;
    }
}
