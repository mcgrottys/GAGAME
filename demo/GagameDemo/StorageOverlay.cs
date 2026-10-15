using Gagame.Harvest;

namespace GagameDemo;

/// The storage view's cells: the bytes the storage index READ off the disk (StorageIndex), per
/// lattice cell, after the filters (source, parameter, store, cell level), rolled up to the
/// display level; coloured by bytes on a log ramp. The composites' coverage[] boxes are drawn as
/// who answers where, as the products wrote them.
public sealed class StorageOverlay : CellOverlay
{
    StorageIndex _index = new();
    List<StorageIndex.Entry> _filtered = new();
    (int version, int level, List<(FidelityMap.CellKey, Color)> cells)? _rolled;

    public string? Source, Param, Store, Product;
    public int? Level;
    public bool ShowCoverage = true;

    public StorageIndex Index => _index;
    public IReadOnlyList<StorageIndex.Entry> Filtered => _filtered;

    public void Show(StorageIndex index) { _index = index; Refilter(); }

    public void Refilter()
    {
        _filtered = _index.Entries.Where(e =>
            (Source is null || e.Source == Source) && (Param is null || e.Param == Param) &&
            (Store is null || e.Store == Store) && (Level is null || StorageIndex.Unpack(e.Cell).Level == Level)).ToList();
        _rolled = null;
        Version++;
    }

    /// 1 kB (t = 0) to 100 GB (t = 1), log.
    static double T(double bytes) => (Math.Log10(Math.Max(bytes, 1)) - 3) / 8;

    public override IEnumerable<(FidelityMap.CellKey, Color)> Cells(int displayLevel)
    {
        if (_rolled is { } r && r.version == Version && r.level == displayLevel) return r.cells;
        var sum = new Dictionary<FidelityMap.CellKey, long>();
        foreach (var e in _filtered)
        {
            var c = StorageIndex.Unpack(e.Cell);
            if (c.Level > displayLevel) c = new FidelityMap.CellKey(c.Face, (byte)displayLevel, c.X >> (c.Level - displayLevel), c.Y >> (c.Level - displayLevel));
            sum[c] = sum.GetValueOrDefault(c) + e.Bytes;
        }
        var cells = sum.Where(kv => kv.Value > 0).OrderBy(kv => kv.Key.Level).Select(kv => (kv.Key, Inferno(T(kv.Value)))).ToList();
        _rolled = (Version, displayLevel, cells);
        return cells;
    }

    public override string LegendTitle => "bytes stored in a cell (log)";
    public override Color Ramp(double t) => Inferno(t);
    public override (double t, string label)[] Ticks => [(0, "1 kB"), (3 / 8.0, "1 MB"), (6 / 8.0, "1 GB"), (1, "100 GB")];

    IEnumerable<StorageIndex.Coverage> Coverage => _index.AllCoverage
        .Where(c => (Param is null || c.Layer == Param) && (Source is null || c.Input == Source) && (Product is null || c.Product == Product));

    /// Who answers, by the cell each coverage entry was written for: one outline a cell, in the hue
    /// of its finest input (the tiers beneath it answer the same cell; the readout lists them all).
    public override IEnumerable<(FidelityMap.CellKey, Color)> Outlines =>
        !ShowCoverage ? [] : Coverage.Where(c => c.Cell != StorageIndex.NoCell)
            .GroupBy(c => c.Cell).Select(g => (StorageIndex.Unpack(g.Key), Hue(g.OrderBy(c => c.GrainM).First().Input)));

    /// A coverage box that is no cell of its map (the default region): drawn as written, unless it
    /// is the whole planet, which an outline cannot show.
    public override IEnumerable<(double, double, double, double, Color)> Boxes =>
        !ShowCoverage ? [] : Coverage.Where(c => c.Cell == StorageIndex.NoCell && c.Lon1 - c.Lon0 < 359.9)
            .GroupBy(c => (c.Lon0, c.Lat0, c.Lon1, c.Lat1)).Select(g => (g.Key.Lon0, g.Key.Lat0, g.Key.Lon1, g.Key.Lat1, Hue(g.OrderBy(c => c.GrainM).First().Input)));

    /// A colour per name, stable across runs (FNV of the name to a hue).
    public static Color Hue(string name)
    {
        uint h = 2166136261;
        foreach (char ch in name) h = (h ^ ch) * 16777619;
        double hue = h % 360 / 60.0;
        int i = (int)hue;
        double f = hue - i;
        int q = (int)(255 * (1 - f)), t = (int)(255 * f);
        return i switch { 0 => Color.FromArgb(255, t, 60), 1 => Color.FromArgb(q, 255, 60), 2 => Color.FromArgb(60, 255, t), 3 => Color.FromArgb(60, q, 255), 4 => Color.FromArgb(t, 60, 255), _ => Color.FromArgb(255, 60, q) };
    }

    /// The filtered bytes stored at a point, by source and store (every cell holding the point,
    /// at every level), and the coverage boxes holding it: who answers there.
    public (List<(string source, string store, long bytes, int tiles)> here, List<StorageIndex.Coverage> answers) At(double lat, double lon)
    {
        var d = new double[3];
        FidelityMap.DirOfLatLon(lat, lon, d);
        int f = FidelityMap.FaceOfDir(d, out double u, out double v);
        var here = _filtered.Where(e =>
        {
            var c = StorageIndex.Unpack(e.Cell);
            double n = 1u << c.Level;
            return c.Face == f && (uint)Math.Min(n - 1, Math.Floor(u * n)) == c.X && (uint)Math.Min(n - 1, Math.Floor(v * n)) == c.Y;
        }).GroupBy(e => (e.Source, e.Store)).Select(g => (g.Key.Source, g.Key.Store, g.Sum(e => e.Bytes), g.Sum(e => e.Tiles)))
          .OrderByDescending(x => x.Item3).ToList();
        // who answers: the coverage whose cell holds the point (a box only where it is no cell)
        var answers = Coverage.Where(c =>
        {
            if (c.Cell == StorageIndex.NoCell) return c.Lat0 <= lat && lat <= c.Lat1 && c.Lon0 <= lon && lon <= c.Lon1;
            var k = StorageIndex.Unpack(c.Cell);
            double n = 1u << k.Level;
            return k.Face == f && (uint)Math.Min(n - 1, Math.Floor(u * n)) == k.X && (uint)Math.Min(n - 1, Math.Floor(v * n)) == k.Y;
        }).OrderBy(c => c.GrainM).ToList();
        return (here, answers);
    }
}
