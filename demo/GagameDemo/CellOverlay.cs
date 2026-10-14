using Gagame.Harvest;

namespace GagameDemo;

/// What the globe paints over its base: lattice cells, each one colour, coarse first so a deeper
/// cell lies over the cell above it; a legend for the colours; and boxes outlined on top.
public abstract class CellOverlay
{
    /// Bumped whenever Cells would answer differently: the globe re-renders on a change.
    public int Version { get; protected set; }

    /// The cells at the display level (cells finer than it rolled up into it; coarser ones as they are).
    public abstract IEnumerable<(FidelityMap.CellKey cell, Color colour)> Cells(int displayLevel);

    public abstract string LegendTitle { get; }
    /// The legend runs t = 0..1 left to right.
    public abstract Color Ramp(double t);
    public abstract (double t, string label)[] Ticks { get; }

    /// Lat/lon boxes outlined over the cells.
    public virtual IEnumerable<(double lon0, double lat0, double lon1, double lat1, Color colour)> Boxes => [];
    /// Lattice cells outlined over the cells, by their own edges.
    public virtual IEnumerable<(FidelityMap.CellKey cell, Color colour)> Outlines => [];

    /// Inferno, perceptually ordered: t = 0 dark violet, t = 1 pale yellow.
    public static Color Inferno(double t)
    {
        (double t, int r, int g, int b)[] k = [(0.0, 40, 11, 84), (0.14, 85, 15, 109), (0.29, 136, 34, 106), (0.43, 186, 54, 85),
            (0.57, 227, 89, 51), (0.71, 249, 140, 10), (0.86, 249, 201, 50), (1.0, 252, 255, 164)];
        t = Math.Clamp(t, 0, 1);
        int i = 0;
        while (i < k.Length - 2 && t > k[i + 1].t) i++;
        double f = (t - k[i].t) / (k[i + 1].t - k[i].t);
        return Color.FromArgb((int)(k[i].r + f * (k[i + 1].r - k[i].r)), (int)(k[i].g + f * (k[i + 1].g - k[i].g)), (int)(k[i].b + f * (k[i + 1].b - k[i].b)));
    }
}

/// The fidelity map's painted cells of one layer, coloured by the grain they want.
public sealed class FidelityOverlay : CellOverlay
{
    FidelityMap _map = new();
    string _layer = FidelityMap.AllLayers;

    public void Show(FidelityMap map, string layer) { _map = map; _layer = layer; Version++; }
    public void Changed() => Version++;

    /// The grain's colour: fine (1 mm) bright, coarse (100 km) dark, over the log of the grain.
    public static Color GrainColour(double grainM) =>
        Inferno(double.IsFinite(grainM) ? 1 - (Math.Log10(Math.Max(grainM, 1e-9)) + 3) / 8 : 0);

    public override IEnumerable<(FidelityMap.CellKey, Color)> Cells(int displayLevel) =>
        _map.Get(_layer).Cells.OrderBy(kv => kv.Key.Level).Select(kv => (kv.Key, GrainColour(kv.Value)));

    public override string LegendTitle => "grain wanted (log)";
    public override Color Ramp(double t) => GrainColour(Math.Pow(10, -3 + 8 * t));
    public override (double t, string label)[] Ticks => [(0, "1 mm"), (3 / 8.0, "1 m"), (6 / 8.0, "1 km"), (1, "100 km")];
}
