using System.Globalization;

namespace Gagame.Harvest;

/// `harvest fidelity ...`: the painter's twin for scripts (docs/FIDELITY.md section 1).
///   new FILE [--default GRAIN] [--layer NAME]
///   paint-box FILE lon0,lat0,lon1,lat1 GRAIN [--layer NAME] [--level L]
///   erase-box FILE lon0,lat0,lon1,lat1 [--layer NAME] [--level L]
///   at FILE lon,lat [--layer NAME]                  the grain wanted at a point
///   finest FILE lon0,lat0,lon1,lat1 [--layer NAME]  the finest grain a box wants
///   dump FILE                                       every layer and cell as JSON
///   stats FILE [--layer NAME]                       cells, area and a height-layer estimate per band
///   check                                           the contract's self-test (lattice, boxes, file)
/// GRAIN takes units (1mm, 5cm, 30m, 2km; bare = metres; inf = coarsest). erase-box returns the
/// box to the default (a coarser cell reaching over it is cut around it).
public static class FidelityCli
{
    public static int Run(string[] argv, TextWriter o)
    {
        var a = new Args(argv);
        var pos = a.Positionals("--layer", "--level", "--default");
        if (pos.Count == 0) { o.WriteLine("harvest fidelity new|paint-box|erase-box|at|finest|dump|stats FILE ... | harvest fidelity check"); return 1; }
        var layer = a.Opt("--layer", FidelityMap.AllLayers)!;
        int? level = a.Opt("--level") is string lv ? int.Parse(lv, CultureInfo.InvariantCulture) : null;
        string Want(int i, string what) => pos.Count > i ? pos[i] : throw new HarvestStop($"fidelity {pos[0]}: wants {what}");
        double[] Nums(string s, int n, string what)
        {
            var v = Args.Pair(s);
            return v.Length == n ? v : throw new HarvestStop($"fidelity {pos[0]}: {what} wants {n} comma-separated numbers, got '{s}'");
        }
        switch (pos[0])
        {
            case "check":
                return Check(o);
            case "new":
            {
                var file = Want(1, "FILE");
                var m = new FidelityMap();
                var l = m.Get(layer, create: true);
                if (a.Opt("--default") is string dg) l.DefaultGrainM = FidelityRegions.ParseGrain(dg);
                m.Save(file);
                o.WriteLine($"{file}: new, layer '{layer}' default {FidelityRegions.FormatLength(l.DefaultGrainM)}");
                return 0;
            }
            case "paint-box":
            case "erase-box":
            {
                var file = Want(1, "FILE");
                var b = Nums(Want(2, "lon0,lat0,lon1,lat1"), 4, "the box");
                var m = File.Exists(file) ? FidelityMap.Load(file) : new FidelityMap();
                var (cells, L) = FidelityRegions.Box(m, b[0], b[1], b[2], b[3], level);
                if (pos[0] == "paint-box")
                {
                    float g = FidelityRegions.ParseGrain(Want(3, "GRAIN"));
                    m.PaintMany(layer, cells, g);
                    o.WriteLine($"{file}: painted {cells.Count} cells of level {L} ({FidelityRegions.FormatLength(m.CellEdgeM(L))}) at {FidelityRegions.FormatLength(g)} on layer '{layer}'");
                }
                else
                {
                    FidelityRegions.Clear(m, layer, cells);
                    o.WriteLine($"{file}: cleared {cells.Count} cells of level {L} on layer '{layer}'");
                }
                if (level is int asked && asked != L) o.WriteLine($"  level {asked} would take more than {FidelityRegions.MaxCellsPerStroke} cells; cut at {L}");
                m.Save(file);
                return 0;
            }
            case "at":
            {
                var m = FidelityMap.Load(Want(1, "FILE"));
                var p = Nums(Want(2, "lon,lat"), 2, "the point");
                o.WriteLine(Num(m.GrainAt(layer, p[1], p[0])));
                return 0;
            }
            case "finest":
            {
                var m = FidelityMap.Load(Want(1, "FILE"));
                var b = Nums(Want(2, "lon0,lat0,lon1,lat1"), 4, "the box");
                o.WriteLine(Num(m.FinestIn(layer, b[0], b[1], b[2], b[3])));
                return 0;
            }
            case "dump":
            {
                var file = Want(1, "FILE");
                var m = FidelityMap.Load(file);
                o.WriteLine(JsonUtil.Dump(new Dictionary<string, object?>
                {
                    ["format"] = "GAFID01", ["file"] = file, ["planet_radius_m"] = m.PlanetRadiusM,
                    ["layers"] = m.Layers.Select(l => new Dictionary<string, object?>
                    {
                        ["name"] = l.Name, ["default_grain_m"] = Num(l.DefaultGrainM),
                        ["cells"] = l.Cells.Select(kv =>
                        {
                            var (lon0, lat0, lon1, lat1) = FidelityMap.CellBox(kv.Key);
                            return new Dictionary<string, object?>
                            {
                                ["face"] = (int)kv.Key.Face, ["level"] = (int)kv.Key.Level, ["x"] = (long)kv.Key.X, ["y"] = (long)kv.Key.Y,
                                ["grain_m"] = Num(kv.Value), ["box"] = new List<double> { lon0, lat0, lon1, lat1 },
                            };
                        }).ToList(),
                    }).ToList(),
                }));
                return 0;
            }
            case "stats":
            {
                var file = Want(1, "FILE");
                var m = FidelityMap.Load(file);
                foreach (var l in m.Layers.Where(l => a.Opt("--layer") is null || l.Name == layer))
                {
                    var (bands, painted, rest, restBytes) = FidelityRegions.Stats(m, l.Name);
                    o.WriteLine($"layer '{l.Name}': {l.Cells.Count} cells, default {FidelityRegions.FormatLength(l.DefaultGrainM)}, painted {painted / 1e6:#,0.###} km^2");
                    foreach (var b in bands)
                        o.WriteLine($"  {Band(b),-22} {b.AreaM2 / 1e6,16:#,0.###} km^2   height estimate {FidelityRegions.FormatBytes(b.HeightBytes)}");
                    o.WriteLine($"  {"unpainted (default)",-22} {rest / 1e6,16:#,0.###} km^2   height estimate {(restBytes > 0 ? FidelityRegions.FormatBytes(restBytes) : "-")}");
                    o.WriteLine($"  ESTIMATE for a height layer: {FidelityRegions.FormatBytes(bands.Sum(b => b.HeightBytes) + restBytes)} (area / grain^2 x 4 B, +1/3 for the pyramid; not what any source holds)");
                }
                return 0;
            }
        }
        o.WriteLine($"fidelity: no command '{pos[0]}'");
        return 2;
    }

    static string Band(FidelityRegions.Band b) =>
        double.IsFinite(b.FromM) ? $"{FidelityRegions.FormatLength(b.FromM)} .. {FidelityRegions.FormatLength(b.ToM)}" : "coarsest";

    /// JSON has no infinity: +Inf is written "inf".
    static object Num(double v) => double.IsFinite(v) ? v : "inf";

    // ------------------------------------------------------------------ the self-test

    /// The contract's gates, each printed with its worst measure: the lattice round trip, the
    /// cell boxes, the file round trip, the planted off-lattice cell, PaintMany and Clear against
    /// their one-cell definitions, FinestIn against dense sampling.
    public static int Check(TextWriter o)
    {
        var rng = new Random(20261013);
        int fails = 0;
        void Gate(bool ok, string what) { o.WriteLine($"[fidelity] {(ok ? "PASS" : "FAIL")} {what}"); if (!ok) fails++; }
        double[] RandDir()
        {
            double x, y, z, r;
            do { x = rng.NextDouble() * 2 - 1; y = rng.NextDouble() * 2 - 1; z = rng.NextDouble() * 2 - 1; r = x * x + y * y + z * z; } while (r > 1 || r < 1e-6);
            r = Math.Sqrt(r);
            return [x / r, y / r, z / r];
        }

        // 1. the lattice: direction -> (face, u, v) -> direction
        {
            double worst = 0;
            var back = new double[3];
            var faces = new int[6];
            for (int i = 0; i < 20000; ++i)
            {
                var d = RandDir();
                int f = FidelityMap.FaceOfDir(d, out double u, out double v);
                faces[f]++;
                FidelityMap.DirOfFaceUv(f, u, v, back);
                worst = Math.Max(worst, Math.Max(Math.Abs(back[0] - d[0]), Math.Max(Math.Abs(back[1] - d[1]), Math.Abs(back[2] - d[2]))));
            }
            Gate(worst <= 1e-12, $"lattice: 20000 directions through FaceOfDir and back by DirOfFaceUv, worst component {worst:0.0e0} (<= 1e-12; faces {string.Join("/", faces)})");
        }

        // 2. CellBox holds every sampled point of its cell (17 x 17 points, edges included)
        {
            double worst = 0;
            var d = new double[3];
            for (int i = 0; i < 3000; ++i)
            {
                var c = RandCell(rng, rng.Next(0, 32));
                var (lon0, lat0, lon1, lat1) = FidelityMap.CellBox(c);
                double n = 1u << c.Level, loc = 0.5 * (lon0 + lon1);
                for (int a = 0; a <= 16; ++a)
                    for (int b = 0; b <= 16; ++b)
                    {
                        FidelityMap.DirOfFaceUv(c.Face, (c.X + a / 16.0) / n, (c.Y + b / 16.0) / n, d);
                        var (la, lo) = FidelityMap.LatLonOfDir(d);
                        if (lon1 - lon0 >= 360) lo = loc;   // a cell holding a pole spans every longitude
                        lo = loc + (lo - loc - 360.0 * Math.Round((lo - loc) / 360.0));
                        double over = Math.Max(Math.Max(lat0 - la, la - lat1), Math.Max(lon0 - lo, lo - lon1));
                        // measured on the ground: degrees of latitude, of longitude scaled by cos lat
                        if (over > 0) worst = Math.Max(worst, (la - lat1 > 0 || lat0 - la > 0 ? Math.Max(lat0 - la, la - lat1) : over * Math.Cos(la * Geo.DegToRad)) * 111_195.0);
                    }
            }
            Gate(worst <= 1e-6, $"CellBox: 3000 random cells of levels 0..31, 289 points each, worst point outside its box {worst:0.0e0} m on the ground (<= 1 micrometre)");
        }

        // 3. the file: random paint (PaintMany, Clear, two layers) -> save -> load -> equal cell for cell
        var path = Path.Combine(Path.GetTempPath(), $"gafid-check-{Environment.ProcessId}.gafid");
        try
        {
            var m = new FidelityMap();
            m.Get("water.bed", create: true).DefaultGrainM = 30f;
            for (int s = 0; s < 60; ++s)
            {
                double lat = rng.NextDouble() * 170 - 85, lon = rng.NextDouble() * 360 - 180;
                var layer = s % 3 == 0 ? "water.bed" : FidelityMap.AllLayers;
                var (cells, _) = FidelityRegions.Cap(m, lat, lon, Math.Pow(10, rng.NextDouble() * 6), rng.Next(4, 26));
                if (s % 7 == 6) FidelityRegions.Clear(m, layer, cells);
                else m.PaintMany(layer, cells, (float)Math.Pow(10, rng.NextDouble() * 8 - 3));
            }
            m.Save(path);
            var r = FidelityMap.Load(path);
            bool same = r.PlanetRadiusM == m.PlanetRadiusM && r.Layers.Count == m.Layers.Count;
            int cells_ = 0;
            for (int i = 0; same && i < m.Layers.Count; ++i)
            {
                var (x, y) = (m.Layers[i], r.Layers[i]);
                same = x.Name == y.Name && x.DefaultGrainM.Equals(y.DefaultGrainM) && x.Cells.Count == y.Cells.Count
                    && x.Cells.All(kv => y.Cells.TryGetValue(kv.Key, out var g) && g.Equals(kv.Value));
                cells_ += x.Cells.Count;
            }
            int diff = 0, probes = 0;
            foreach (var l in m.Layers)
                for (int i = 0; i < 10000; ++i)
                {
                    var (lat, lon) = FidelityMap.LatLonOfDir(RandDir());
                    probes++;
                    if (!m.GrainAt(l.Name, lat, lon).Equals(r.GrainAt(l.Name, lat, lon))) diff++;
                }
            // probes inside painted cells too: the random directions above mostly land on the default
            foreach (var l in m.Layers)
                foreach (var k in l.Cells.Keys.Take(2000))
                {
                    var (lon0, lat0, lon1, lat1) = FidelityMap.CellBox(k);
                    double lat = 0.5 * (lat0 + lat1), lon = 0.5 * (lon0 + lon1);
                    probes++;
                    if (!m.GrainAt(l.Name, lat, lon).Equals(r.GrainAt(l.Name, lat, lon))) diff++;
                }
            Gate(same && diff == 0, $"file: {cells_} cells in {m.Layers.Count} layers saved and loaded, every cell equal = {same}; GrainAt differs at {diff} of {probes} points");

            // 4. a planted off-lattice cell refuses the file (x = 2^level, then face 6, then level 32)
            var bytes = File.ReadAllBytes(path);
            int first = FirstCellOffset(bytes);
            var refused = new List<string>();
            foreach (var (what, patch) in new (string, Action<byte[]>)[]
            {
                ("x = 2^level", b => BitConverter.TryWriteBytes(b.AsSpan(first + 4), (uint)1 << b[first + 1])),
                ("face 6", b => b[first] = 6),
                ("level 32", b => b[first + 1] = 32),
            })
            {
                var bad = (byte[])bytes.Clone();
                patch(bad);
                File.WriteAllBytes(path, bad);
                try { FidelityMap.Load(path); }
                catch (InvalidDataException) { refused.Add(what); }
            }
            Gate(refused.Count == 3, $"file: a planted off-lattice cell refuses the file ({string.Join(", ", refused)} refused of 3)");
        }
        finally { File.Delete(path); }

        // 5. PaintMany = Paint one by one; Clear = the default inside, unchanged outside
        {
            int mism = 0, outsideChanged = 0, insideNotDefault = 0;
            for (int t = 0; t < 40; ++t)
            {
                var a = new FidelityMap();
                for (int s = 0; s < 6; ++s)
                {
                    var (cells, _) = FidelityRegions.Cap(a, rng.NextDouble() * 120 - 60, rng.NextDouble() * 360 - 180, Math.Pow(10, 3 + rng.NextDouble() * 3), rng.Next(3, 12));
                    a.PaintMany(FidelityMap.AllLayers, cells, s + 1);
                }
                var many = a.Clone();
                var one = a.Clone();
                var (cs, _) = FidelityRegions.Cap(a, rng.NextDouble() * 120 - 60, rng.NextDouble() * 360 - 180, Math.Pow(10, 4 + rng.NextDouble() * 3), rng.Next(2, 10));
                cs.Add(cs[0].Level < FidelityMap.MaxLevel ? new FidelityMap.CellKey(cs[0].Face, (byte)(cs[0].Level + 1), cs[0].X * 2, cs[0].Y * 2) : cs[0]);   // a cell inside another of the set
                many.PaintMany(FidelityMap.AllLayers, cs, 99);
                foreach (var c in cs.OrderByDescending(c => c.Level)) one.Paint(FidelityMap.AllLayers, c, 99);
                var (mc, oc) = (many.Get(FidelityMap.AllLayers).Cells, one.Get(FidelityMap.AllLayers).Cells);
                if (mc.Count != oc.Count || mc.Any(kv => !oc.TryGetValue(kv.Key, out var g) || g != kv.Value)) mism++;

                var cleared = a.Clone();
                FidelityRegions.Clear(cleared, FidelityMap.AllLayers, cs);
                var set = cs.ToHashSet();
                var d = new double[3];
                for (int i = 0; i < 4000; ++i)
                {
                    var (lat, lon) = i % 2 == 0 ? FidelityMap.LatLonOfDir(RandDir()) : PointIn(cs[rng.Next(cs.Count)], rng, d);
                    var deepest = cs.Max(c => (int)c.Level);
                    bool inside = Enumerable.Range(0, deepest + 1).Any(L => set.Contains(FidelityMap.CellOf(lat, lon, L)));
                    float before = a.GrainAt(FidelityMap.AllLayers, lat, lon), after = cleared.GrainAt(FidelityMap.AllLayers, lat, lon);
                    if (inside && !float.IsPositiveInfinity(after)) insideNotDefault++;
                    if (!inside && !before.Equals(after)) outsideChanged++;
                }
            }
            Gate(mism == 0, $"PaintMany: 40 random sets (each holding a cell inside another) equal to painting the outermost one by one; {mism} differ");
            Gate(insideNotDefault == 0 && outsideChanged == 0, $"Clear: 40 x 4000 points, {insideNotDefault} inside not back to the default, {outsideChanged} outside changed");
        }

        // 6. FinestIn: no finer than the finest sampled point, and the coarse-inside-fine case
        {
            var m = new FidelityMap();
            m.Get(FidelityMap.AllLayers).DefaultGrainM = 1000f;
            var outer = FidelityMap.CellOf(10, 10, 4);
            var inner = new FidelityMap.CellKey(outer.Face, 6, outer.X * 4 + 1, outer.Y * 4 + 1);
            m.Paint(FidelityMap.AllLayers, outer, 10f);
            m.Paint(FidelityMap.AllLayers, inner, 500f);
            var (lon0, lat0, lon1, lat1) = FidelityMap.CellBox(inner);
            double cx = 0.5 * (lon0 + lon1), cy = 0.5 * (lat0 + lat1), e = 0.1 * (lat1 - lat0);
            float inside = m.FinestIn(FidelityMap.AllLayers, cx - e, cy - e, cx + e, cy + e);
            Gate(inside == 500f, $"FinestIn: a box inside a 500 m cell painted within a 10 m cell under a 1000 m default answers {inside} m (500)");

            int finer = 0, coarser = 0;
            for (int t = 0; t < 200; ++t)
            {
                var r = new FidelityMap();
                for (int s = 0; s < 5; ++s)
                {
                    var (cells, _) = FidelityRegions.Cap(r, rng.NextDouble() * 40, rng.NextDouble() * 40, Math.Pow(10, 4 + rng.NextDouble() * 2), rng.Next(4, 10));
                    r.PaintMany(FidelityMap.AllLayers, cells, (float)Math.Pow(10, rng.NextDouble() * 4));
                }
                double bx = rng.NextDouble() * 38, by = rng.NextDouble() * 38, w = rng.NextDouble() * 2, h = rng.NextDouble() * 2;
                float f = r.FinestIn(FidelityMap.AllLayers, bx, by, bx + w, by + h);
                float sampled = float.PositiveInfinity;
                for (int i = 0; i <= 60; ++i)
                    for (int j = 0; j <= 60; ++j)
                        sampled = Math.Min(sampled, r.GrainAt(FidelityMap.AllLayers, by + h * j / 60, bx + w * i / 60));
                if (f > sampled) coarser++;
                if (f < sampled) finer++;
            }
            Gate(coarser == 0, $"FinestIn: 200 random boxes, {coarser} answer coarser than a 61 x 61 sampling (never allowed), {finer} finer (cells the samples miss, or a cell box reaching past the cell)");
        }

        o.WriteLine(fails == 0 ? "[fidelity] all gates pass" : $"[fidelity] {fails} gate(s) FAIL");
        return fails == 0 ? 0 : 1;
    }

    static FidelityMap.CellKey RandCell(Random rng, int level)
    {
        ulong n = 1UL << level;
        return new FidelityMap.CellKey((byte)rng.Next(6), (byte)level, (uint)(rng.NextDouble() * n), (uint)(rng.NextDouble() * n));
    }

    static (double lat, double lon) PointIn(FidelityMap.CellKey c, Random rng, double[] d)
    {
        double n = 1u << c.Level;
        FidelityMap.DirOfFaceUv(c.Face, (c.X + 0.02 + 0.96 * rng.NextDouble()) / n, (c.Y + 0.02 + 0.96 * rng.NextDouble()) / n, d);
        return FidelityMap.LatLonOfDir(d);
    }

    /// The byte offset of the first cell of the first layer holding one.
    static int FirstCellOffset(byte[] b)
    {
        int p = 8 + 8;
        uint layers = BitConverter.ToUInt32(b, p); p += 4;
        for (uint i = 0; i < layers; ++i)
        {
            p += 2 + BitConverter.ToUInt16(b, p) + 4;
            uint n = BitConverter.ToUInt32(b, p); p += 4;
            if (n > 0) return p;
        }
        throw new InvalidDataException("no cell to plant over");
    }
}
