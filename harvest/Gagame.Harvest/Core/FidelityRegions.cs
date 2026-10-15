using System.Globalization;

namespace Gagame.Harvest;

/// The regions a fidelity map is painted and measured by (docs/FIDELITY.md section 1), shared
/// by the painter (demo launcher) and `harvest fidelity`: which lattice cells a box or a brush
/// covers, clearing a region back to the default, the ground area each grain holds, and the
/// grain's units. Every region is cells of the one lattice; a box or a circle is only how a
/// person names them.
public static class FidelityRegions
{
    /// The most cells one stroke or one box writes: a region needing more is cut at a coarser
    /// level (its outline is coarser; the grain it asks for is unchanged).
    public const int MaxCellsPerStroke = 4096;

    /// The cells cut per region edge: a region is outlined by cells an eighth of its extent
    /// (a brush's diameter, a box's shorter side), ~50 cells for a circle.
    public const double CellsPerExtent = 8.0;

    /// The cell's centre direction and its angular radius (the largest angle to a corner: the
    /// cell's edges are great circles, so it lies within that cap of its centre).
    public static (double[] centre, double radius) CellCap(FidelityMap.CellKey c)
    {
        double n = 1u << c.Level;
        var m = new double[3];
        FidelityMap.DirOfFaceUv(c.Face, (c.X + 0.5) / n, (c.Y + 0.5) / n, m);
        double r = 0;
        var d = new double[3];
        for (int i = 0; i < 4; ++i)
        {
            FidelityMap.DirOfFaceUv(c.Face, (c.X + (i & 1)) / n, (c.Y + (i >> 1)) / n, d);
            r = Math.Max(r, Angle(m, d));
        }
        return (m, r);
    }

    public static double Angle(double[] a, double[] b)
    {
        // atan2 of |a x b| and a . b: exact at small angles where acos of the dot is not
        double cx = a[1] * b[2] - a[2] * b[1], cy = a[2] * b[0] - a[0] * b[2], cz = a[0] * b[1] - a[1] * b[0];
        return Math.Atan2(Math.Sqrt(cx * cx + cy * cy + cz * cz), a[0] * b[0] + a[1] * b[1] + a[2] * b[2]);
    }

    static IEnumerable<FidelityMap.CellKey> Children(FidelityMap.CellKey c)
    {
        for (uint i = 0; i < 4; ++i)
            yield return new FidelityMap.CellKey(c.Face, (byte)(c.Level + 1), c.X * 2 + (i & 1), c.Y * 2 + (i >> 1));
    }

    /// Descends the six faces to `level`, opening a cell where `meets` says the region may
    /// reach it and keeping a cell of `level` where `keep` says so; stops (null) past `cap` cells.
    static List<FidelityMap.CellKey>? Descend(int level, Func<FidelityMap.CellKey, bool> meets, Func<FidelityMap.CellKey, bool> keep, int cap)
    {
        var out_ = new List<FidelityMap.CellKey>();
        var stack = new Stack<FidelityMap.CellKey>();
        for (byte f = 0; f < 6; ++f) stack.Push(new FidelityMap.CellKey(f, 0, 0, 0));
        while (stack.Count > 0)
        {
            var c = stack.Pop();
            if (!meets(c)) continue;
            if (c.Level == level)
            {
                if (keep(c)) { out_.Add(c); if (out_.Count > cap) return null; }
                continue;
            }
            foreach (var ch in Children(c)) stack.Push(ch);
        }
        return out_;
    }

    /// The level a region of this extent (metres) is outlined at: cells CellsPerExtent across it.
    public static int LevelForExtent(FidelityMap m, double extentM) => m.LevelForEdge(extentM / CellsPerExtent);

    /// The cells whose centre lies within `radiusM` of a point (a brush dab), at `level` or the
    /// finest coarser level within MaxCellsPerStroke; the cell holding the point when none does.
    public static (List<FidelityMap.CellKey> cells, int level) Cap(FidelityMap m, double latDeg, double lonDeg, double radiusM, int level)
    {
        var c = new double[3];
        FidelityMap.DirOfLatLon(latDeg, lonDeg, c);
        double rho = radiusM / m.PlanetRadiusM;
        for (int L = level; ; --L)
        {
            var cells = Descend(L,
                k => { var (mid, r) = CellCap(k); return Angle(c, mid) <= rho + r; },
                k => { var (mid, _) = CellCap(k); return Angle(c, mid) <= rho; },
                MaxCellsPerStroke);
            if (cells is null) continue;
            if (cells.Count == 0) cells.Add(FidelityMap.CellOf(latDeg, lonDeg, L));
            return (cells, L);
        }
    }

    /// The cells meeting a lat/lon box (by their lat/lon boxes, so the box is covered whole), at
    /// `level` or, when null, the level the box's shorter side asks; coarser past MaxCellsPerStroke.
    public static (List<FidelityMap.CellKey> cells, int level) Box(FidelityMap m, double lon0, double lat0, double lon1, double lat1, int? level = null)
    {
        if (lon1 < lon0 || lat1 < lat0) throw new ArgumentException($"box {lon0},{lat0},{lon1},{lat1}: wants lon0 <= lon1 and lat0 <= lat1");
        double mid = 0.5 * (lat0 + lat1) * Geo.DegToRad, perDeg = m.PlanetRadiusM * Geo.DegToRad;
        double shortM = Math.Min((lat1 - lat0) * perDeg, (lon1 - lon0) * perDeg * Math.Cos(mid));
        int L = level ?? LevelForExtent(m, Math.Max(shortM, 1e-3));
        for (; ; --L)
        {
            var cells = Descend(L, k => Meets(FidelityMap.CellBox(k), lon0, lat0, lon1, lat1), _ => true, MaxCellsPerStroke);
            if (cells is not null) return (cells, L);
        }
    }

    static bool Meets((double lon0, double lat0, double lon1, double lat1) b, double lon0, double lat0, double lon1, double lat1)
    {
        if (b.lat1 < lat0 || b.lat0 > lat1) return false;
        for (int k = -1; k <= 1; ++k)
            if (b.lon1 + 360 * k >= lon0 && b.lon0 + 360 * k <= lon1) return true;
        return false;
    }

    /// Returns the region the cells cover to the layer's default: every painted cell inside them
    /// goes, and a painted cell reaching over them from above is cut into the children that lie
    /// outside them (each keeping its grain), so the ground around keeps what it had. Erase
    /// (docs/FIDELITY.md) falls back to the cell above; this is what a brush means by erase.
    public static void Clear(FidelityMap map, string layer, IEnumerable<FidelityMap.CellKey> cells)
    {
        var l = map.Get(layer, create: true);
        var set = cells.ToHashSet();
        bool Under(FidelityMap.CellKey k)   // k is a cell of the set or inside one
        {
            for (int L = k.Level; L >= 0; --L)
                if (set.Contains(new FidelityMap.CellKey(k.Face, (byte)L, k.X >> (k.Level - L), k.Y >> (k.Level - L)))) return true;
            return false;
        }
        var above = new HashSet<FidelityMap.CellKey>();   // strict ancestors of the set
        foreach (var s in set)
            for (var a = s; a.Level > 0;) { a = a.Parent; if (!above.Add(a)) break; }
        foreach (var k in l.Cells.Keys.Where(Under).ToList()) l.Cells.Remove(k);
        // coarsest first: a deeper painted ancestor is met while it still stands and left to its own turn
        foreach (var a in l.Cells.Keys.Where(above.Contains).OrderBy(k => k.Level).ToList())
        {
            float g = l.Cells[a];
            l.Cells.Remove(a);
            var stack = new Stack<FidelityMap.CellKey>(Children(a));
            while (stack.Count > 0)
            {
                var c = stack.Pop();
                if (set.Contains(c) || l.Cells.ContainsKey(c)) continue;
                if (above.Contains(c)) { foreach (var ch in Children(c)) stack.Push(ch); continue; }
                l.Cells[c] = g;
            }
        }
    }

    // ------------------------------------------------------------------ area

    /// A cell's solid angle (steradians): the gnomonic face patch, exact
    /// (F(s, t) = atan(s t / sqrt(1 + s^2 + t^2)) taken at the four corners).
    public static double SolidAngle(FidelityMap.CellKey c)
    {
        double n = 1u << c.Level;
        double s0 = 2 * c.X / n - 1, s1 = 2 * (c.X + 1) / n - 1, t0 = 2 * c.Y / n - 1, t1 = 2 * (c.Y + 1) / n - 1;
        static double F(double s, double t) => Math.Atan(s * t / Math.Sqrt(1 + s * s + t * t));
        return F(s1, t1) - F(s0, t1) - F(s1, t0) + F(s0, t0);
    }

    /// The ground each painted cell answers for (m^2): its area less the painted cells nearest
    /// below it, which answer for their own.
    public static Dictionary<FidelityMap.CellKey, double> OwnArea(FidelityMap map, string layer)
    {
        var cells = map.Get(layer).Cells;
        double r2 = map.PlanetRadiusM * map.PlanetRadiusM;
        var own = cells.Keys.ToDictionary(k => k, k => SolidAngle(k) * r2);
        foreach (var k in cells.Keys)
            for (var a = k; a.Level > 0;)
            {
                a = a.Parent;
                if (cells.ContainsKey(a)) { own[a] -= SolidAngle(k) * r2; break; }
            }
        return own;
    }

    public sealed record Band(double FromM, double ToM, double AreaM2, double HeightBytes);

    /// The painted ground per decade of grain, and an ESTIMATE of what a height layer at that
    /// grain would weigh: one float32 a grain cell, plus a third for the coarser levels of its
    /// pyramid. The unpainted rest counts at the default when the default is finite.
    public static (List<Band> bands, double paintedM2, double restM2, double restBytes) Stats(FidelityMap map, string layer)
    {
        var l = map.Get(layer);
        var bands = new SortedDictionary<int, (double area, double bytes)>();
        double painted = 0;
        foreach (var (k, a) in OwnArea(map, layer))
        {
            float g = l.Cells[k];
            painted += a;
            int dec = float.IsFinite(g) ? (int)Math.Floor(Math.Log10(g)) : int.MaxValue;
            var (sa, sb) = bands.GetValueOrDefault(dec);
            bands[dec] = (sa + a, sb + HeightBytes(a, g));
        }
        double whole = 4 * Math.PI * map.PlanetRadiusM * map.PlanetRadiusM, rest = Math.Max(0, whole - painted);
        var list = bands.Select(kv => kv.Key == int.MaxValue
            ? new Band(double.PositiveInfinity, double.PositiveInfinity, kv.Value.area, 0)
            : new Band(Math.Pow(10, kv.Key), Math.Pow(10, kv.Key + 1), kv.Value.area, kv.Value.bytes)).ToList();
        return (list, painted, rest, HeightBytes(rest, l.DefaultGrainM));
    }

    public static double HeightBytes(double areaM2, double grainM) =>
        double.IsFinite(grainM) && grainM > 0 ? areaM2 / (grainM * grainM) * 4.0 * 4.0 / 3.0 : 0;

    // ------------------------------------------------------------------ units

    /// A grain or a length with units: "1 mm", "5cm", "30 m", "2.5km", a bare number is metres;
    /// "inf" / "coarsest" is +Infinity.
    public static float ParseGrain(string s)
    {
        s = s.Trim().ToLowerInvariant();
        if (s is "inf" or "+inf" or "infinity" or "coarsest") return float.PositiveInfinity;
        double scale = 1;
        foreach (var (unit, f) in new[] { ("km", 1000.0), ("mm", 1e-3), ("cm", 1e-2), ("m", 1.0) })
            if (s.EndsWith(unit)) { scale = f; s = s[..^unit.Length].Trim(); break; }
        return (float)(double.Parse(s, CultureInfo.InvariantCulture) * scale);
    }

    /// A length in the unit that keeps it between 1 and 1000: mm, cm (under 10 cm), m, km.
    public static string FormatLength(double m)
    {
        if (!double.IsFinite(m)) return "coarsest";
        static string N(double v) => v.ToString(v >= 100 ? "0" : v >= 10 ? "0.#" : "0.##", CultureInfo.InvariantCulture);
        if (m >= 1000) return N(m / 1000) + " km";
        if (m >= 1) return N(m) + " m";
        if (m >= 0.01) return N(m * 100) + " cm";
        return N(m * 1000) + " mm";
    }

    public static string FormatBytes(double b) =>
        b >= 1e12 ? $"{b / 1e12:0.0} TB" : b >= 1e9 ? $"{b / 1e9:0.0} GB" : b >= 1e6 ? $"{b / 1e6:0.0} MB" : $"{b / 1e3:0.0} kB";

    public static string FormatKey(FidelityMap.CellKey k) => $"{k.Face}/{k.Level}/{k.X}/{k.Y}";
}
