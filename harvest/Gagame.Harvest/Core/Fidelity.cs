using System.Text;

namespace Gagame.Harvest;

/// THE FIDELITY MAP (docs/FIDELITY.md, file type `.gafid`, magic GAFID01): the finest grain, in
/// metres on the ground, that each region of the planet WANTS, painted on the engine's own cube
/// lattice (src/core/Lattice.cpp ComposeCubeDir / CubeFaceOfDir), so a harvester asks a region
/// for no finer data than this and the engine's tree (one lattice for everything) reads the same
/// cells. A cell is (face, level, x, y): level L cuts a face into 2^L by 2^L cells, u = (x + 1/2)
/// / 2^L. The grain at a point is the DEEPEST painted cell holding it, else the layer's default.
/// One layer per data layer by name; "*" stands for every layer a file does not name.
public sealed class FidelityMap
{
    public const string Magic = "GAFID01\0";
    public const string AllLayers = "*";
    public const int MaxLevel = 31;

    public readonly record struct CellKey(byte Face, byte Level, uint X, uint Y) : IComparable<CellKey>
    {
        public int CompareTo(CellKey o)
        {
            int c = Face.CompareTo(o.Face);
            if (c != 0) return c;
            if ((c = Level.CompareTo(o.Level)) != 0) return c;
            if ((c = Y.CompareTo(o.Y)) != 0) return c;
            return X.CompareTo(o.X);
        }
        public CellKey Parent => new(Face, (byte)(Level - 1), X >> 1, Y >> 1);
        public bool Contains(CellKey d) =>
            d.Face == Face && d.Level >= Level && (d.X >> (d.Level - Level)) == X && (d.Y >> (d.Level - Level)) == Y;
    }

    public sealed class Layer
    {
        public string Name = AllLayers;
        /// The grain where nothing is painted (metres). +Infinity = the region wants nothing of
        /// its own: the coarsest the source has (a global base) is enough.
        public float DefaultGrainM = float.PositiveInfinity;
        public readonly SortedDictionary<CellKey, float> Cells = new();
    }

    public readonly List<Layer> Layers = new() { new Layer() };
    public double PlanetRadiusM = 6371000.0;

    public Layer Get(string name, bool create = false)
    {
        var l = Layers.FirstOrDefault(x => x.Name == name);
        if (l is null && create) Layers.Add(l = new Layer { Name = name });
        return l ?? Layers.FirstOrDefault(x => x.Name == AllLayers) ?? new Layer();
    }

    /// A deep copy (the painter's undo keeps these).
    public FidelityMap Clone()
    {
        var c = new FidelityMap { PlanetRadiusM = PlanetRadiusM };
        c.Layers.Clear();
        foreach (var l in Layers)
        {
            var n = new Layer { Name = l.Name, DefaultGrainM = l.DefaultGrainM };
            foreach (var (k, g) in l.Cells) n.Cells[k] = g;
            c.Layers.Add(n);
        }
        return c;
    }

    // ------------------------------------------------------------------ the lattice

    /// The planet frame (Lattice.cpp, ExposurePage.h): x = cos lat cos lon, y = sin lat, z = cos lat sin lon.
    public static void DirOfLatLon(double latDeg, double lonDeg, double[] d)
    {
        double la = latDeg * Geo.DegToRad, lo = lonDeg * Geo.DegToRad, cl = Math.Cos(la);
        d[0] = cl * Math.Cos(lo); d[1] = Math.Sin(la); d[2] = cl * Math.Sin(lo);
    }

    public static (double lat, double lon) LatLonOfDir(double[] d)
    {
        double r = Math.Sqrt(d[0] * d[0] + d[1] * d[1] + d[2] * d[2]);
        return (Math.Asin(Math.Clamp(d[1] / r, -1.0, 1.0)) / Geo.DegToRad, Math.Atan2(d[2], d[0]) / Geo.DegToRad);
    }

    /// CubeFaceOfDir, line for line (src/core/Lattice.h).
    public static int FaceOfDir(double[] d, out double u, out double v)
    {
        double a0 = Math.Abs(d[0]), a1 = Math.Abs(d[1]), a2 = Math.Abs(d[2]), sx, t;
        int face;
        if (a0 >= a1 && a0 >= a2)
        {
            if (d[0] > 0) { face = 0; sx = -d[2] / a0; t = -d[1] / a0; }
            else { face = 1; sx = d[2] / a0; t = -d[1] / a0; }
        }
        else if (a1 >= a2)
        {
            if (d[1] > 0) { face = 2; sx = d[0] / a1; t = d[2] / a1; }
            else { face = 3; sx = d[0] / a1; t = -d[2] / a1; }
        }
        else
        {
            if (d[2] > 0) { face = 4; sx = d[0] / a2; t = -d[1] / a2; }
            else { face = 5; sx = -d[0] / a2; t = -d[1] / a2; }
        }
        u = sx * 0.5 + 0.5;
        v = t * 0.5 + 0.5;
        return face;
    }

    // CubeFaceAxes' table (Lattice.cpp): the unnormalized point is n + s a + t b.
    static readonly double[,,] Axes =
    {
        { { 1, 0, 0 }, { 0, 0, -1 }, { 0, -1, 0 } },
        { { -1, 0, 0 }, { 0, 0, 1 }, { 0, -1, 0 } },
        { { 0, 1, 0 }, { 1, 0, 0 }, { 0, 0, 1 } },
        { { 0, -1, 0 }, { 1, 0, 0 }, { 0, 0, -1 } },
        { { 0, 0, 1 }, { 1, 0, 0 }, { 0, -1, 0 } },
        { { 0, 0, -1 }, { -1, 0, 0 }, { 0, -1, 0 } },
    };

    /// ComposeCubeDir: face, u, v -> the unit direction.
    public static void DirOfFaceUv(int face, double u, double v, double[] d)
    {
        double s = 2.0 * u - 1.0, t = 2.0 * v - 1.0, r2 = 0;
        for (int i = 0; i < 3; ++i) { d[i] = Axes[face, 0, i] + s * Axes[face, 1, i] + t * Axes[face, 2, i]; r2 += d[i] * d[i]; }
        double r = Math.Sqrt(r2);
        for (int i = 0; i < 3; ++i) d[i] /= r;
    }

    public static CellKey CellOf(double latDeg, double lonDeg, int level)
    {
        var d = new double[3];
        DirOfLatLon(latDeg, lonDeg, d);
        int f = FaceOfDir(d, out double u, out double v);
        uint n = 1u << level, x = (uint)Math.Min(n - 1, Math.Max(0.0, Math.Floor(u * n))),
             y = (uint)Math.Min(n - 1, Math.Max(0.0, Math.Floor(v * n)));
        return new CellKey((byte)f, (byte)level, x, y);
    }

    /// A cell's edge on the ground at its face's centre: the face's quarter great circle cut 2^L
    /// times. (A cell near a face corner is ~1.8x smaller; this is what a brush chooses by.)
    public double CellEdgeM(int level) => Math.PI * 0.5 * PlanetRadiusM / Math.Pow(2.0, level);

    /// The finest level whose cells are no smaller than `edgeM` at a face's centre.
    public int LevelForEdge(double edgeM) =>
        Math.Clamp((int)Math.Floor(Math.Log2(Math.PI * 0.5 * PlanetRadiusM / Math.Max(edgeM, 1e-6))), 0, MaxLevel);

    /// The cell's latitude/longitude box, sampled on its edges (its edges are great circles,
    /// so a box from 9 points a side is within a hair; a cell holding a pole spans every longitude).
    public static (double lon0, double lat0, double lon1, double lat1) CellBox(CellKey c)
    {
        double n = 1u << c.Level, lat0 = 90, lat1 = -90;
        var d = new double[3];
        var lons = new List<double>();
        for (int i = 0; i <= 8; ++i)
            for (int e = 0; e < 4; ++e)
            {
                double a = i / 8.0;
                double u = e switch { 0 => a, 1 => 1, 2 => 1 - a, _ => 0 }, v = e switch { 0 => 0, 1 => a, 2 => 1, _ => 1 - a };
                DirOfFaceUv(c.Face, (c.X + u) / n, (c.Y + v) / n, d);
                var (la, lo) = LatLonOfDir(d);
                lat0 = Math.Min(lat0, la); lat1 = Math.Max(lat1, la);
                lons.Add(lo);
            }
        DirOfFaceUv(c.Face, (c.X + 0.5) / n, (c.Y + 0.5) / n, d);
        var (_, loc) = LatLonOfDir(d);
        // unwrap about the centre so a cell across the antimeridian is one interval
        double lon0 = 1e9, lon1 = -1e9;
        foreach (var lo in lons)
        {
            double w = lo - loc;
            w -= 360.0 * Math.Round(w / 360.0);
            lon0 = Math.Min(lon0, loc + w); lon1 = Math.Max(lon1, loc + w);
        }
        // a pole inside the cell: faces 2 (north) and 3 (south) hold them at their centres
        if (c.Face == 2 || c.Face == 3)
        {
            double half = n / 2.0;
            if (c.X <= half && half <= c.X + 1 && c.Y <= half && half <= c.Y + 1)
            {
                lon0 = -180; lon1 = 180;
                if (c.Face == 2) lat1 = 90; else lat0 = -90;
            }
        }
        return (lon0, lat0, lon1, lat1);
    }

    // ------------------------------------------------------------------ painting

    /// Sets a cell's grain. Its descendants go (the region is set whole); its ancestors stay, so
    /// the parts of them outside it keep what they had.
    public void Paint(string layer, CellKey c, float grainM)
    {
        var l = Get(layer, create: true);
        foreach (var k in l.Cells.Keys.Where(k => k.Level > c.Level && c.Contains(k)).ToList()) l.Cells.Remove(k);
        l.Cells[c] = grainM;
    }

    /// Removes a cell and its descendants: the region falls back to whatever holds it above.
    public void Erase(string layer, CellKey c)
    {
        var l = Get(layer, create: true);
        foreach (var k in l.Cells.Keys.Where(k => k.Level >= c.Level && c.Contains(k)).ToList()) l.Cells.Remove(k);
    }

    /// Paint over a set of cells at once, equal to painting the set's outermost cells one by one
    /// (a cell of the set inside another of the set is the same region at the same grain), in one
    /// pass over the layer instead of one pass a cell: a brush stroke is thousands of cells.
    public void PaintMany(string layer, IEnumerable<CellKey> cells, float grainM)
    {
        var l = Get(layer, create: true);
        var set = Outermost(cells);
        foreach (var k in Covered(l, set)) l.Cells.Remove(k);
        foreach (var c in set) l.Cells[c] = grainM;
    }

    static HashSet<CellKey> Outermost(IEnumerable<CellKey> cells)
    {
        var set = cells.ToHashSet();
        set.RemoveWhere(c => HasAncestorIn(c, set, c.Level - 1));
        return set;
    }

    static bool HasAncestorIn(CellKey c, HashSet<CellKey> set, int fromLevel)
    {
        for (int L = fromLevel; L >= 0; --L)
            if (set.Contains(new CellKey(c.Face, (byte)L, c.X >> (c.Level - L), c.Y >> (c.Level - L)))) return true;
        return false;
    }

    /// The layer's cells strictly inside a cell of `set`.
    static List<CellKey> Covered(Layer l, HashSet<CellKey> set)
    {
        if (set.Count == 0) return new();
        int top = set.Min(c => (int)c.Level);
        return l.Cells.Keys.Where(k => k.Level > top && HasAncestorIn(k, set, k.Level - 1)).ToList();
    }

    // ------------------------------------------------------------------ reading

    /// The grain wanted at a point: the deepest painted cell holding it, else the default.
    public float GrainAt(string layer, double latDeg, double lonDeg)
    {
        var l = Get(layer);
        if (l.Cells.Count == 0) return l.DefaultGrainM;
        var d = new double[3];
        DirOfLatLon(latDeg, lonDeg, d);
        int f = FaceOfDir(d, out double u, out double v);
        int deepest = l.Cells.Keys.Max(k => (int)k.Level);
        for (int L = deepest; L >= 0; --L)
        {
            uint n = 1u << L;
            var k = new CellKey((byte)f, (byte)L, (uint)Math.Min(n - 1, Math.Floor(u * n)), (uint)Math.Min(n - 1, Math.Floor(v * n)));
            if (l.Cells.TryGetValue(k, out float g)) return g;
        }
        return l.DefaultGrainM;
    }

    /// The finest grain any part of a lat/lon box wants (the default where the box reaches past
    /// every painted cell) -- what a harvester asks a source for over that box. A walk down the
    /// lattice from the six faces, carrying the grain the deepest painted cell above hands down:
    /// a region's grain is its own deepest cell's, so a coarse cell painted inside a fine one
    /// answers coarse there, and the default counts only where the box meets unpainted ground.
    public float FinestIn(string layer, double lon0, double lat0, double lon1, double lat1)
    {
        var l = Get(layer);
        var open = new HashSet<CellKey>();   // the painted cells and their ancestors: where the walk descends
        foreach (var k in l.Cells.Keys)
            for (var a = k; open.Add(a) && a.Level > 0; a = a.Parent) { }
        float Walk(CellKey c, float above)
        {
            if (!Overlaps(CellBox(c), lon0, lat0, lon1, lat1)) return float.PositiveInfinity;
            if (!open.Contains(c)) return above;
            float here = l.Cells.TryGetValue(c, out float g) ? g : above, best = float.PositiveInfinity;
            if (c.Level == MaxLevel) return here;
            bool split = false;
            for (uint i = 0; i < 4; ++i)
            {
                var ch = new CellKey(c.Face, (byte)(c.Level + 1), c.X * 2 + (i & 1), c.Y * 2 + (i >> 1));
                if (open.Contains(ch)) split = true;
            }
            if (!split) return here;
            for (uint i = 0; i < 4; ++i)
                best = Math.Min(best, Walk(new CellKey(c.Face, (byte)(c.Level + 1), c.X * 2 + (i & 1), c.Y * 2 + (i >> 1)), here));
            return best;
        }
        float finest = float.PositiveInfinity;
        for (byte f = 0; f < 6; ++f) finest = Math.Min(finest, Walk(new CellKey(f, 0, 0, 0), l.DefaultGrainM));
        return finest;
    }

    /// Every painted cell of a layer with its box, finest grain first: the regions a harvester walks.
    public IEnumerable<(CellKey cell, float grainM, (double lon0, double lat0, double lon1, double lat1) box)> Regions(string layer) =>
        Get(layer).Cells.Select(kv => (kv.Key, kv.Value, CellBox(kv.Key))).OrderBy(r => r.Value);

    static bool Overlaps((double lon0, double lat0, double lon1, double lat1) b, double lon0, double lat0, double lon1, double lat1)
    {
        if (b.lat1 < lat0 || b.lat0 > lat1) return false;
        for (int k = -1; k <= 1; ++k)
            if (b.lon1 + 360 * k >= lon0 && b.lon0 + 360 * k <= lon1) return true;
        return false;
    }

    // ------------------------------------------------------------------ the file

    public void Save(string path)
    {
        Directory.CreateDirectory(Path.GetDirectoryName(Path.GetFullPath(path))!);
        using var w = new BinaryWriter(File.Create(path), Encoding.UTF8);
        w.Write(Encoding.ASCII.GetBytes(Magic));
        w.Write(PlanetRadiusM);
        w.Write((uint)Layers.Count);
        foreach (var l in Layers)
        {
            var nb = Encoding.UTF8.GetBytes(l.Name);
            w.Write((ushort)nb.Length);
            w.Write(nb);
            w.Write(l.DefaultGrainM);
            w.Write((uint)l.Cells.Count);
            foreach (var (k, g) in l.Cells)
            {
                w.Write(k.Face); w.Write(k.Level); w.Write((ushort)0);
                w.Write(k.X); w.Write(k.Y); w.Write(g);
            }
        }
    }

    public static FidelityMap Load(string path)
    {
        using var r = new BinaryReader(File.OpenRead(path), Encoding.UTF8);
        if (Encoding.ASCII.GetString(r.ReadBytes(8)) != Magic) throw new InvalidDataException($"{path}: not a GAFID01 fidelity map");
        var m = new FidelityMap { PlanetRadiusM = r.ReadDouble() };
        m.Layers.Clear();
        uint nl = r.ReadUInt32();
        for (uint i = 0; i < nl; ++i)
        {
            var l = new Layer { Name = Encoding.UTF8.GetString(r.ReadBytes(r.ReadUInt16())), DefaultGrainM = r.ReadSingle() };
            uint nc = r.ReadUInt32();
            for (uint j = 0; j < nc; ++j)
            {
                byte f = r.ReadByte(), lv = r.ReadByte();
                r.ReadUInt16();
                uint x = r.ReadUInt32(), y = r.ReadUInt32();
                float g = r.ReadSingle();
                if (f > 5 || lv > MaxLevel || x >> lv != 0 || y >> lv != 0) throw new InvalidDataException($"{path}: cell {f}/{lv}/{x}/{y} is off the lattice");
                l.Cells[new CellKey(f, lv, x, y)] = g;
            }
            m.Layers.Add(l);
        }
        return m;
    }
}
