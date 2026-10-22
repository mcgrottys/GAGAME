using System.Globalization;
using System.Text.RegularExpressions;
using Gagame.Harvest.Harvesters;
using Gagame.Harvest.Readers;

namespace Gagame.Harvest.Sources;

/// A relief cut into fixed lon/lat tiles (`tile_deg`: [width, height]) named by a template of
/// their north-west corner: `{ns}{lat1:00}{ew}{lon0:000}` is N45W075. Shared by the readers that
/// hold their files on that lattice (a tiled download, a griddap cut into chunks).
static class TileLattice
{
    public static IEnumerable<(string name, Box cell)> Meets(Box box, double tw, double th, string template)
    {
        for (var lat1 = 90 - th * Math.Floor((90 - box.Lat1) / th); lat1 > box.Lat0 && lat1 > -90; lat1 -= th)
            for (var lon0 = -180 + tw * Math.Floor((box.Lon0 + 180) / tw); lon0 < box.Lon1 && lon0 < 180; lon0 += tw)
                yield return (Name(template, lon0, lat1), new Box(lon0, lat1 - th, lon0 + tw, lat1));
    }

    static string Name(string template, double lon0, double lat1) =>
        Regex.Replace(template.Replace("{ns}", lat1 >= 0 ? "N" : "S").Replace("{ew}", lon0 >= 0 ? "E" : "W"), @"\{(lat1|lon0):([0#.]+)\}",
            m => Math.Abs(m.Groups[1].Value == "lat1" ? lat1 : lon0).ToString(m.Groups[2].Value, CultureInfo.InvariantCulture));

    /// The first existing of the cache templates for `name`, else the first (where it is fetched to).
    public static (string dest, long? size) Cached(HarvestContext ctx, string[] cache, params (string k, string v)[] fields)
    {
        var paths = cache.Select(t => SourceConfig.Resolve(ctx, t, fields)).ToList();
        var have = paths.FirstOrDefault(File.Exists);
        return (have ?? paths[0], have is null ? null : new FileInfo(have).Length);
    }
}

/// A relief whose files are read as they ship (their pixels are copied into the tiers, never
/// scaled) must already be metres: a config declaring other units is refused, not converted.
static class Copied
{
    public static void Check(SourceConfig c)
    {
        var spec = c.Var("elevation") ?? throw new InvalidDataException($"{c.Path}: needs variables.elevation (its units)");
        if (Units.Factor("elevation", spec.Units, c.Path) != 1)
            throw new InvalidDataException($"{c.Path}: elevation in {spec.Units}: a {c.Kind} relief is copied as it ships and must be in metres");
    }
}

/// GeoTIFF tiles at fixed URLs, one file a tile of the lattice (ETOPO 2022's 15 degree tiles; its
/// 60" globe is one tile of 360 x 180). Config: `tile_deg`, `name`, `fetch` (the URL template),
/// `cache` (path templates, the first existing is read).
[SourceKind("tiled-geotiff")]
public sealed class TiledGeoTiff : IReliefSource
{
    public SourceConfig Config { get; }
    readonly HarvestContext _ctx;
    readonly double _tw, _th;
    readonly string _name, _fetch;
    readonly string[] _cache;

    public TiledGeoTiff(SourceConfig c, SourceEnv env)
    {
        Config = c;
        _ctx = env.Ctx;
        var t = c.Nums("tile_deg");
        (_tw, _th) = (t[0], t[1]);
        _name = c.S("name");
        _fetch = c.S("fetch");
        _cache = c.Strs("cache");
        Copied.Check(c);
    }

    public List<ReliefFile> Meets(Box box) =>
        TileLattice.Meets(box, _tw, _th, _name).Where(x => Config.Coverage.Meets(x.cell.Lon0, x.cell.Lat0, x.cell.Lon1, x.cell.Lat1)).Select(x =>
        {
            var (dest, size) = TileLattice.Cached(_ctx, _cache, ("name", x.name));
            return new ReliefFile { Key = x.name, Cell = x.cell, Url = _fetch.Replace("{name}", x.name), Dest = dest, Size = size };
        }).ToList();
}

/// NOAA's CUDEM on the NODD bucket: a zipped shapefile index of 0.25 degree tiles (location =
/// <region>/<tile>.tif and the tile's URL), the sizes from the bucket's listing of a region.
/// Config: `bucket`, `prefix`, `index`, `index_cache`, `cache` (templates of {region}, {name}),
/// `listing_cache`.
[SourceKind("cudem-index")]
public sealed class CudemIndex : IReliefSource
{
    public SourceConfig Config { get; }
    readonly SourceEnv _env;
    readonly string _bucket, _prefix, _listing;
    readonly string[] _cache;
    List<(CudemTiles.Tile t, Box cell)>? _cells;
    readonly Dictionary<string, Dictionary<string, long>> _sizes = new();

    public CudemIndex(SourceConfig c, SourceEnv env)
    {
        Config = c;
        _env = env;
        _bucket = c.S("bucket");
        _prefix = c.S("prefix");
        _cache = c.Strs("cache");
        _listing = c.S("listing_cache");
        Copied.Check(c);
    }

    List<(CudemTiles.Tile t, Box cell)> Cells()
    {
        if (_cells is not null) return _cells;
        var index = Config.S("index");
        var (zip, _) = _env.Polite.Get($"https://{_bucket}.s3.amazonaws.com/{_prefix}{index}", SourceConfig.Resolve(_env.Ctx, Config.S("index_cache"), ("index", index)));
        return _cells = CudemTiles.ReadTileIndex(zip).Select(t => (t, cell: CudemTiles.TileBounds(t.Name) ?? (t.Bbox is { } b ? new Box(b[0], b[1], b[2], b[3]) : null)))
            .Where(x => x.cell is not null).Select(x => (x.t, x.cell!)).ToList();
    }

    long? SizeOf(CudemTiles.Tile t)
    {
        if (!_sizes.TryGetValue(t.Region, out var s))
        {
            var (objs, _) = _env.Polite.S3List(_bucket, $"{_prefix}{t.Region}/", SourceConfig.Resolve(_env.Ctx, _listing, ("region", t.Region)));
            _sizes[t.Region] = s = objs.ToDictionary(o => o.Key[(o.Key.LastIndexOf('/') + 1)..], o => o.Size);
        }
        return s.TryGetValue(t.Name, out var n) ? n : null;
    }

    public List<ReliefFile> Meets(Box box) =>
        Cells().Where(c => box.Meets(c.cell.Lon0, c.cell.Lat0, c.cell.Lon1, c.cell.Lat1)).OrderBy(c => c.t.Url, StringComparer.Ordinal).Select(c =>
        {
            var (dest, size) = TileLattice.Cached(_env.Ctx, _cache, ("region", c.t.Region), ("name", c.t.Name));
            return new ReliefFile { Key = c.t.Name, Cell = c.cell, Url = c.t.Url, Dest = dest, Size = size, SizeOf = () => SizeOf(c.t) };
        }).ToList();
}

/// An ERDDAP griddap grid of elevation on latitude x longitude, cut into `chunk_deg` chunks on
/// the tile lattice: a chunk is one request for the pixel centres inside it,
/// `<var>[(lat0+s/2):1:(lat1-s/2)][(lon0+s/2):1:(lon1-s/2)]` as classic netCDF, its axes checked
/// against the lattice of `step_arcsec` (a grid not on it is refused, never resampled), and
/// written beside it as the composites' float32 GeoTIFF (north up, nodata where the fill was), so
/// every reader of a relief reads one format. The variable is named or matched by standard_name;
/// its units come from the server when the config gives none.
[SourceKind("erddap-griddap")]
public sealed class ErddapGriddap : IReliefSource
{
    public SourceConfig Config { get; }
    readonly SourceEnv _env;
    readonly string _server, _dataset, _dir, _name;
    readonly string[] _cache;
    readonly double _chunk;
    Resolved? _z;
    string _lat = "", _lon = "";

    public ErddapGriddap(SourceConfig c, SourceEnv env)
    {
        Config = c;
        _env = env;
        _server = c.Server ?? throw new InvalidDataException($"{c.Path}: needs \"server\"");
        _dataset = c.Dataset ?? throw new InvalidDataException($"{c.Path}: needs \"dataset\"");
        _dir = SourceConfig.Resolve(env.Ctx, c.S("metadata_cache"));
        _name = c.S("name");
        _cache = c.Strs("cache");
        _chunk = c.N("chunk_deg");
        var n = _chunk / c.StepDeg;
        if (Math.Abs(n - Math.Round(n)) > 1e-6) throw new InvalidDataException($"{c.Path}: chunk_deg {_chunk} is not a whole number of steps");
    }

    Resolved Z()
    {
        if (_z is not null) return _z;
        var meta = Erddap.Info(_env, _server, _dataset, _dir);
        string Col(string q) => Resolved.Of(Config.Var(q) ?? throw new InvalidDataException($"{Config.Path}: needs variables.{q}"), meta, Config.Path, _env.Log).Column;
        _lat = Col("lat"); _lon = Col("lon");
        return _z = Resolved.Of(Config.Var("elevation") ?? throw new InvalidDataException($"{Config.Path}: needs variables.elevation"), meta, Config.Path, _env.Log);
    }

    public List<ReliefFile> Meets(Box box) =>
        TileLattice.Meets(box, _chunk, _chunk, _name).Where(x => Config.Coverage.Meets(x.cell.Lon0, x.cell.Lat0, x.cell.Lon1, x.cell.Lat1)).Select(x =>
        {
            var (dest, size) = TileLattice.Cached(_env.Ctx, _cache, ("name", x.name));
            var n = (int)Math.Round(_chunk / Config.StepDeg);
            // a chunk's size is not known before it is asked: the plan charges its float32 pixels.
            // The request is spelled when the chunk is made, so a run that never makes one never
            // asks the server for its metadata.
            var f = new ReliefFile { Key = x.name, Cell = x.cell, Dest = dest, Size = size ?? 4L * n * n, Url = $"{_server}/griddap/{_dataset}.nc ({x.name})" };
            f.Make = p => Make(p, f, n);
            return f;
        }).ToList();

    string Url(Box cell)
    {
        var z = Z();
        var h = 0.5 * Config.StepDeg;
        return $"{_server}/griddap/{_dataset}.nc?" + Erddap.Q($"{z.Column}[({Erddap.F(cell.Lat0 + h)}):1:({Erddap.F(cell.Lat1 - h)})][({Erddap.F(cell.Lon0 + h)}):1:({Erddap.F(cell.Lon1 - h)})]");
    }

    void Make(Polite p, ReliefFile f, int n)
    {
        var z = Z();
        var nc = Path.ChangeExtension(f.Dest, ".nc");
        var url = Url(f.Cell);
        var (bytes, _) = p.Get(url, nc, note: "erddap griddap chunk");
        var g = new NetCdf3(bytes);
        var (lats, _) = g.Read(_lat);
        var (lons, _) = g.Read(_lon);
        var (vals, shape) = g.Read(z.Column);
        var s = Config.StepDeg;
        bool On(double[] axis, double first) => axis.Length == n && axis.Select((v, i) => Math.Abs(v - (first + (i + 0.5) * s))).Max() < 1e-3 * s;
        var southUp = lats.Length > 1 && lats[1] > lats[0];
        if (!On(lons, f.Cell.Lon0) || !On(southUp ? lats : lats.Reverse().ToArray(), f.Cell.Lat0) || shape.Count != 2 || shape[0] != n || shape[1] != n)
            throw new InvalidDataException($"{url}: the answer's axes ({lats.Length} x {lons.Length}, {lats.FirstOrDefault()}.., {lons.FirstOrDefault()}..) are not the {n} x {n} centres of {f.Cell} at {s * 3600}\": not on the lattice, refused");
        var outv = new float[n * n];
        for (var r = 0; r < n; r++)
        {
            var src = southUp ? n - 1 - r : r;
            for (var c = 0; c < n; c++) outv[r * n + c] = z.Apply(vals[src * n + c]) is double v ? (float)v : float.NaN;
        }
        GeoTiffWriter.Write(f.Dest, outv, n, n, f.Cell.Lon0, f.Cell.Lat1, s, s, Config.VerticalEpsg, -99999f);
    }
}
