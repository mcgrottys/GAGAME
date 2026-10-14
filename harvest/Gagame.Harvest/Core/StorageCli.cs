using System.Diagnostics;

namespace Gagame.Harvest;

/// `harvest storage [--index FILE] [--rebuild]`: brings the storage index (StorageIndex) up to
/// date with the checkout's stores and prints what is stored, per parameter and per store, with
/// the scan's time and the index's size. `harvest storage check` plants a fake tree, archive and
/// composite of known sizes in a scratch checkout and holds the index to them cell for cell.
public static class StorageCli
{
    public static int Run(string root, string[] argv, TextWriter o)
    {
        var a = new Args(argv);
        if (a.Positionals("--index").FirstOrDefault() == "check") return Check(o);
        var path = a.Opt("--index") ?? StorageIndex.DefaultPath;
        var sw = Stopwatch.StartNew();
        var x = a.Has("--rebuild") ? new StorageIndex() : StorageIndex.Load(path, root);
        double loadS = sw.Elapsed.TotalSeconds;
        x.Update(root, log: o.WriteLine);
        sw.Restart();
        if (x.Changed) x.Save(path);
        o.WriteLine($"[storage] index {path}: {new FileInfo(path).Length / 1e6:0.00} MB, loaded in {loadS:0.00} s, saved in {sw.Elapsed.TotalSeconds:0.00} s");
        var entries = x.Entries.ToList();
        o.WriteLine($"[storage] {entries.Count:#,0} cell aggregates, {entries.Sum(e => e.Bytes) / 1e9:0.000} GB in cells, " +
                    $"{x.AllUnmapped.Sum(u => u.Bytes) / 1e9:0.000} GB not in a cell, {x.Folders.Values.Sum(f => f.Voids):#,0} void markers");
        o.WriteLine("  coverage by product: entries, the cells they were written for (outlined by their own edges), the rest boxes");
        foreach (var g in x.AllCoverage.GroupBy(c => c.Product).OrderBy(g => g.Key, StringComparer.Ordinal))
            o.WriteLine($"    {g.Key,-22} {g.Count(),6:#,0} entries, {g.Count(c => c.Cell != StorageIndex.NoCell),6:#,0} on a cell ({g.Where(c => c.Cell != StorageIndex.NoCell).Select(c => c.Cell).Distinct().Count():#,0} distinct), {g.Count(c => c.Cell == StorageIndex.NoCell),4:#,0} boxes");
        o.WriteLine("  by parameter and store:");
        foreach (var g in entries.GroupBy(e => (e.Param, e.Store)).OrderByDescending(g => g.Sum(e => e.Bytes)).Take(40))
            o.WriteLine($"    {g.Key.Param,-28} {g.Key.Store,-22} {FidelityRegions.FormatBytes(g.Sum(e => e.Bytes)),10} {g.Sum(e => (long)e.Tiles),10:#,0} tiles {g.Select(e => e.Source).Distinct().Count(),5} sources");
        foreach (var g in x.AllUnmapped.GroupBy(u => u.Why))
            o.WriteLine($"    not in a cell: {g.Key}: {FidelityRegions.FormatBytes(g.Sum(u => u.Bytes))} in {g.Sum(u => (long)u.Files):#,0} files");
        return 0;
    }

    /// The planted stores and the cells and bytes they must land in.
    public static int Check(TextWriter o)
    {
        var root = Path.Combine(Path.GetTempPath(), $"gastor-check-{Environment.ProcessId}");
        int fails = 0;
        void Gate(bool ok, string what) { o.WriteLine($"[storage] {(ok ? "PASS" : "FAIL")} {what}"); if (!ok) fails++; }
        try
        {
            var tree = Path.Combine(root, "cache", "trees", "fake.src.0badf00d");
            void Put(string rel, int bytes) { var p = Path.Combine(tree, rel); Directory.CreateDirectory(Path.GetDirectoryName(p)!); File.WriteAllBytes(p, new byte[bytes]); }
            Put(@"cube16k\f2_m4_x1_y2.bin", 1000);              // level 7 - 4 = 3
            Put(@"cube16k\f0_m0_x5_y7_deadbeef.bin", 4096);     // level 7, a compose node's keyed tile
            Put(@"cube16k\f0_m0_x5_y7_cafef00d.bin", 10);       // the same address, another key: both stored
            Put(@"cube16k\f1_m3_x0_y0.void", 0);
            Put(@"cube2097152k\f3_m2_x300_y600.bin", 777);      // level 22, to its level-14 ancestor
            Put(@"window_z14_1263360_1538048\f0_m0_x0_y0.bin", 500);
            // an archive of two records: directory read, payload never
            {
                var p = Path.Combine(tree, "cube16k.gaa");
                using var w = new BinaryWriter(File.Create(p));
                w.Write(0x52414147u); w.Write(1u); w.Write(2u); w.Write(0u);
                foreach (var (f, m, tx, ty, size) in new (ulong f, ulong m, ulong tx, ulong ty, uint size)[] { (4, 1, 3, 9, 300), (5, 2, 1, 1, 200) })
                {
                    w.Write(f << 61 | m << 56 | ty << 28 | tx); w.Write(65536UL); w.Write(size); w.Write(0u);
                }
                w.Write(new byte[65536 - 16 - 48 + 500]);
            }
            // a composite: one tier of two tiles and a coverage box
            var prod = Path.Combine(root, "data", "composite", "fake");
            Directory.CreateDirectory(Path.Combine(prod, "bed", "t", "p0"));
            File.WriteAllBytes(Path.Combine(prod, "bed", "t", "p0", "n10x5_w20x5.tif"), new byte[1234]);
            File.WriteAllBytes(Path.Combine(prod, "bed", "t", "p0", "n10x5_w20x0.tif"), new byte[66]);
            File.WriteAllText(Path.Combine(prod, "water.bed.composite.json"),
                """{"format":"GACOMP01","layer":"water.bed","kind":"height","tiers":[{"grain_m":30,"folder":"bed/t/p0","match":"*.tif","box":[-20.5,10.0,-19.5,10.5],"inputs":["fake.input"]}],"coverage":[{"box":[-20.5,10.0,-19.5,10.5],"grain_m":30,"input":"fake.input","gap_fill":false}]}""");

            var x = new StorageIndex();
            x.Update(root);
            var got = x.Entries.GroupBy(e => (e.Store, e.Cell)).ToDictionary(g => g.Key, g => g.Sum(e => e.Bytes));
            long At(string store, int f, int l, uint cx, uint cy) => got.GetValueOrDefault((store, StorageIndex.Pack(new FidelityMap.CellKey((byte)f, (byte)l, cx, cy))));

            Gate(At("loose", 2, 3, 1, 2) == 1000, $"cube16k f2 m4 x1 y2 (1000 B) -> cell 2/3/1/2: {At("loose", 2, 3, 1, 2)} B");
            Gate(At("loose", 0, 7, 5, 7) == 4106, $"cube16k f0 m0 x5 y7, two keys (4096 + 10 B) -> cell 0/7/5/7: {At("loose", 0, 7, 5, 7)} B");
            Gate(At("loose", 3, 12, 300 >> 10, 600 >> 10) == 777, $"cube2097152k f3 m2 x300 y600 (777 B, level 22) -> its level-12 ancestor 3/12/0/0: {At("loose", 3, 12, 0, 0)} B");
            Gate(At("archive", 4, 6, 3, 9) == 300 && At("archive", 5, 5, 1, 1) == 200, $"archive records (300, 200 B) -> cells 4/6/3/9 and 5/5/1/1: {At("archive", 4, 6, 3, 9)}, {At("archive", 5, 5, 1, 1)} B");
            Gate(x.Folders.Values.Sum(f => f.Voids) == 1, "the .void marker counted, no bytes");
            // the window tile: its centre (Lattice.cpp Box: Web-Mercator pixels of zoom 14) by hand
            {
                double world = (1L << 14) * 256.0, X = (1263360 + 64) / world, Y = (1538048 + 64) / world;
                double lat = Math.Atan(Math.Sinh(Math.PI * (1 - 2 * Y))) * 180 / Math.PI, lon = (X - 0.5) * 360;
                var hits = x.Entries.Where(e => e.Store == "loose" && e.Lattice.StartsWith("window")).ToList();
                var c = hits.Count == 1 ? StorageIndex.Unpack(hits[0].Cell) : default;
                bool holds = hits.Count == 1 && FidelityMap.CellOf(lat, lon, c.Level) == c && hits[0].Bytes == 500;
                Gate(holds, $"window_z14 tile (500 B) at {lat:0.0000}, {lon:0.0000} -> the cell holding it, {FidelityRegions.FormatKey(c)} (level {c.Level}, {FidelityRegions.FormatLength(new FidelityMap().CellEdgeM(c.Level))}): {(hits.Count == 1 ? hits[0].Bytes : -1)} B");
            }
            long comp = x.Entries.Where(e => e.Store == "composite:fake").Sum(e => e.Bytes);
            var compCells = x.Entries.Where(e => e.Store == "composite:fake").Select(e => StorageIndex.Unpack(e.Cell)).ToList();
            bool inBox = compCells.All(c => { var (lo0, la0, lo1, la1) = FidelityMap.CellBox(c); return lo1 >= -20.5 && lo0 <= -19.5 && la1 >= 10 && la0 <= 10.5; });
            Gate(comp == 1300 && inBox && x.Entries.Where(e => e.Store == "composite:fake").All(e => e.Source == "fake.input" && e.Param == "water.bed"),
                $"composite tier tiles (1234 + 66 B) -> {compCells.Count} cells inside the tier box, source fake.input, param water.bed: {comp} B");
            Gate(x.AllCoverage.Count() == 1 && x.AllCoverage.First().Input == "fake.input", "composite coverage[] read: 1 box, input fake.input");
            var archiveOverhead = x.AllUnmapped.Where(u => u.Why.StartsWith("archive directory")).Sum(u => u.Bytes);
            Gate(archiveOverhead == new FileInfo(Path.Combine(tree, "cube16k.gaa")).Length - 500, $"the archive's directory and alignment ({archiveOverhead} B) kept out of the cells and counted");

            // the rescan reads only what moved
            var idx = Path.Combine(root, "index.bin");
            x.Save(idx);
            var y = StorageIndex.Load(idx, root);
            y.Update(root);
            Gate(y.FoldersRead == 0 && y.Entries.Sum(e => e.Bytes) == x.Entries.Sum(e => e.Bytes), $"saved, loaded and rescanned unchanged: {y.FoldersRead} read, {y.FoldersReused} reused, bytes equal");
            Put(@"cube16k\f2_m4_x1_y3.bin", 64);
            Directory.SetLastWriteTimeUtc(Path.Combine(tree, "cube16k"), DateTime.UtcNow.AddSeconds(5));
            y.Update(root);
            Gate(y.FoldersRead == 1 && y.Entries.Sum(e => e.Bytes) == x.Entries.Sum(e => e.Bytes) + 64, $"a tile added: {y.FoldersRead} folder read again, +64 B");
        }
        finally { Directory.Delete(root, recursive: true); }
        o.WriteLine(fails == 0 ? "[storage] all gates pass" : $"[storage] {fails} gate(s) FAIL");
        return fails == 0 ? 0 : 1;
    }
}
