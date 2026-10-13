namespace Gagame.Harvest.Harvesters;

/// OSM land polygons (land-polygons-split-4326) into GALAND01 (harvest_land_polygons.py): the
/// global land/sea mask's rings, indexed by 1-degree cell so the engine pages them in.
public sealed class LandPolygons : IHarvester
{
    public string Name => "land-polygons";
    public string Title => "Land polygons (OSM coastline)";
    public string What => "osmdata.openstreetmap.de's split land polygons as GALAND01, the global land/sea mask's rings by 1-degree cell.";
    public string[] Outputs => [];
    public Source[] Sources => [new("OSM land polygons", "https://osmdata.openstreetmap.de/data/land-polygons.html")];
    public Insert? Insert => new(InsertKind.FileArg, "Pick land_polygons.shp (the split-4326 file); the output .galand goes beside it unless --out names it.", Pattern: "Shapefiles (*.shp)|*.shp|All files|*.*");
    public string? Note => "Not a download: unzip land-polygons-split-4326 first.";

    public int Run(HarvestContext ctx, string[] argv)
    {
        var a = new Args(argv);
        var pos = a.Positionals("--out", "--bbox");
        if (pos.Count == 0) throw new HarvestStop("usage: harvest land-polygons <land_polygons.shp> [<out.galand> | --out FILE]");
        var shp = pos[0];
        var out_ = pos.Count > 1 ? pos[1] : a.Opt("--out") ?? Path.ChangeExtension(shp, ".galand");
        var cells = new SortedDictionary<(int cy, int cx), List<(long ptsAt, int npts, double[] box)>>();
        var n = 0;
        using (var f = File.OpenRead(shp))
        {
            f.Seek(100, SeekOrigin.Begin);
            var h = new byte[8];
            var head = new byte[44];
            while (f.Read(h, 0, 8) == 8)
            {
                var clen = Bin.I32BE(h, 4);
                var start = f.Position;
                f.ReadExactly(head);
                var st = Bin.I32(head, 0);
                if (st == 5)
                {
                    double x0 = Bin.F64(head, 4), y0 = Bin.F64(head, 12), x1 = Bin.F64(head, 20), y1 = Bin.F64(head, 28);
                    int nparts = Bin.I32(head, 36), npts = Bin.I32(head, 40);
                    var ptsAt = start + 44 + 4L * nparts;
                    var cx = (int)Math.Floor(0.5 * (x0 + x1));
                    var cy = (int)Math.Floor(0.5 * (y0 + y1));
                    if (!cells.TryGetValue((cy, cx), out var l)) cells[(cy, cx)] = l = new();
                    l.Add((ptsAt, npts, new[] { x0, y0, x1, y1 }));
                    n++;
                }
                f.Seek(start + clen * 2L, SeekOrigin.Begin);
            }
        }
        var keys = cells.Keys.ToList();
        ctx.Log($"{n} rings in {keys.Count} cells");
        using (var f = File.OpenRead(shp))
        using (var o = File.Create(out_))
        {
            o.Write("GALAND01"u8);
            Bin.U32(o, (uint)keys.Count);
            var indexAt = o.Position;
            o.Write(new byte[16 * keys.Count]);
            var index = new List<(int cx, int cy, int rings, long off)>();
            long ptsTotal = 0;
            var buf = new byte[0];
            for (var k = 0; k < keys.Count; k++)
            {
                var (cy, cx) = keys[k];
                index.Add((cx, cy, cells[keys[k]].Count, o.Position));
                foreach (var (ptsAt, npts, box) in cells[keys[k]])
                {
                    f.Seek(ptsAt, SeekOrigin.Begin);
                    if (buf.Length < 16 * npts) buf = new byte[16 * npts];
                    f.ReadExactly(buf, 0, 16 * npts);
                    foreach (var b in box) Bin.F32(o, (float)b);
                    Bin.U32(o, (uint)npts);
                    for (var i = 0; i < 2 * npts; i++) Bin.F32(o, (float)Bin.F64(buf, 8 * i));
                    ptsTotal += npts;
                }
                if (k % 5000 == 0) ctx.Log($"  {k} of {keys.Count} cells, {ptsTotal} points");
            }
            o.Seek(indexAt, SeekOrigin.Begin);
            foreach (var (cx, cy, rings, off) in index) { Bin.I16(o, (short)cx); Bin.I16(o, (short)cy); Bin.U32(o, (uint)rings); Bin.U64(o, (ulong)off); }
            ctx.Log($"wrote {out_}: {n} rings, {ptsTotal} points");
        }
        return 0;
    }
}
