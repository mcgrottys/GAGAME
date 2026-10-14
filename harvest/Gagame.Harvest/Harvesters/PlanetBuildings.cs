using System.Buffers.Binary;
using System.Collections.Concurrent;
using Gagame.Harvest.Readers;
using static Gagame.Harvest.JsonUtil;

namespace Gagame.Harvest.Harvesters;

/// harvest_buildings' solids at the scale of the planet (harvest_planet_buildings.py): reads
/// what osmium leaves (building ways carrying their own coordinates), a pool of workers over the
/// file's blocks, records bucketed by 2.5-degree tile into temp files, each bucket sorted by
/// cell and id, concatenated; the cell index as a binary sidecar.
public sealed class PlanetBuildings : IHarvester
{
    public string Name => "planet-buildings";
    public string Title => "Planet buildings (osmium + solids)";
    public string What => "Every building of a LocationsOnWays .osm.pbf (osmium tags-filter + add-locations-to-ways) as GABLDG01 solids, bucketed and sorted at planet scale.";
    public string[] Outputs => [];
    public Source[] Sources => [new("OSM planet", "https://planet.openstreetmap.org/"), new("osmium tool", "https://osmcode.org/osmium-tool/")];
    public Insert? Insert => new(InsertKind.FileArg, "Pick the buildings .osm.pbf WITH LocationsOnWays (osmium add-locations-to-ways).", Pattern: "OSM extracts (*.osm.pbf)|*.osm.pbf|All files|*.*");
    public string? Note => "Needs osmium's output; hours for the planet.";

    static readonly int Bucket = (int)Math.Round(2.5 / Buildings.CellDeg);
    public static readonly string[] Roofs =
    [
        "flat", "skillion", "gabled", "half-hipped", "hipped", "pyramidal", "gambrel", "mansard", "dome", "onion", "round", "saltbox",
        "quadruple_saltbox", "sawtooth", "cone", "crosspitched", "side_hipped", "side_half-hipped", "double_saltbox", "gabled_height_moved", "butterfly", "many",
    ];
    const int EntrySize = 4 + 4 + 8 + 4;
    const int Flush = 32 << 20;

    static string BucketOf((int iy, int ix) cell) => $"{FloorDiv(cell.iy, Bucket):+000;-000}_{FloorDiv(cell.ix, Bucket):+000;-000}";
    static int FloorDiv(int a, int b) { var q = a / b; return (a % b != 0 && (a < 0) != (b < 0)) ? q - 1 : q; }

    sealed class Buckets
    {
        readonly string _tmp, _tag;
        readonly Dictionary<string, List<byte[]>> _buf = new();
        long _size;
        public Buckets(string tmp, string tag) { _tmp = tmp; _tag = tag; }

        public void Add((int iy, int ix) cell, long sid, byte[] rec)
        {
            var head = new byte[EntrySize];
            BinaryPrimitives.WriteInt32LittleEndian(head, cell.iy);
            BinaryPrimitives.WriteInt32LittleEndian(head.AsSpan(4), cell.ix);
            BinaryPrimitives.WriteInt64LittleEndian(head.AsSpan(8), sid);
            BinaryPrimitives.WriteUInt32LittleEndian(head.AsSpan(16), (uint)rec.Length);
            var b = BucketOf(cell);
            if (!_buf.TryGetValue(b, out var l)) _buf[b] = l = new();
            l.Add(head); l.Add(rec);
            _size += rec.Length + EntrySize;
            if (_size > Flush) FlushAll();
        }

        public void FlushAll()
        {
            foreach (var (b, items) in _buf)
            {
                using var f = new FileStream(Path.Combine(_tmp, $"{b}.{_tag}.part"), FileMode.Append);
                foreach (var it in items) f.Write(it);
            }
            _buf.Clear();
            _size = 0;
        }
    }

    sealed record Rel(long Id, int Kind, Buildings.Attrs A, List<(long member, bool outer)> Members);

    public int Run(HarvestContext ctx, string[] argv)
    {
        var a = new Args(argv);
        var pos = a.Positionals("--out", "--workers", "--bbox");
        if (pos.Count == 0) throw new HarvestStop("usage: harvest planet-buildings <buildings-loc.osm.pbf> [--workers N] [--out DIR]");
        var pbf = pos[0];
        var workers = a.Int("--workers", Math.Max(1, Environment.ProcessorCount - 2));
        var t0 = ctx.Seconds;
        var hdr = OsmPbf.Header(pbf);
        if (hdr.GetValueOrDefault("optional") is not List<string> opt || !opt.Contains("LocationsOnWays"))
            throw new HarvestStop("[planet] no LocationsOnWays in the header: run osmium add-locations-to-ways first");
        var outDir = a.Opt("--out") ?? Path.Combine(Path.GetDirectoryName(Path.GetFullPath(pbf))!, "buildings");
        var stem = Path.GetFileName(pbf).Split('.')[0];
        var tmp = Path.Combine(outDir, stem + ".tmp");
        Directory.CreateDirectory(tmp);
        foreach (var q in Directory.GetFiles(tmp)) File.Delete(q);

        var dataBlocks = OsmPbf.BlockTable(pbf).Where(b => b.type == "OSMData").Select(b => b.off).ToList();
        ctx.Log($"[planet] {dataBlocks.Count:N0} data blocks; {workers} workers ({ctx.Seconds - t0:0} s)");

        // pass 1: building ways -> bucketed records; building multipolygons -> collected
        var refused = Buildings.Refused();
        var skipped = new ConcurrentDictionary<string, int>();
        long nWays = 0;
        var rels = new ConcurrentBag<Rel>();
        var wayBlocks = new ConcurrentBag<long>();
        var chunks = Chunks(dataBlocks, 64);
        var done = 0;
        Parallel.ForEach(chunks, new ParallelOptions { MaxDegreeOfParallelism = workers }, chunk =>
        {
            var tag = $"w{Environment.CurrentManagedThreadId}_{Guid.NewGuid():N}";
            var buckets = new Buckets(tmp, tag);
            var myRefused = Buildings.Refused();
            long n = 0;
            foreach (var off in chunk)
            {
                var b = new OsmPbf.Block(OsmPbf.BlobAt(pbf, off));
                var kinds = b.Kinds().ToHashSet();
                if (kinds.Contains("ways"))
                {
                    wayBlocks.Add(off);
                    var bk = new HashSet<int> { b.Index.GetValueOrDefault("building", -1), b.Index.GetValueOrDefault("building:part", -1) };
                    foreach (var w in b.Ways())
                    {
                        if (!w.Keys.Any(bk.Contains)) continue;
                        var t = Buildings.TagsOf(b, w.Keys, w.Vals);
                        var kind = Buildings.IsBuilding(t);
                        if (kind is null) continue;
                        var r = OsmPbf.PackedDelta(w.Refs);
                        if (r.Length < 4 || r[0] != r[^1]) { skipped.AddOrUpdate("open_way", 1, (_, v) => v + 1); continue; }
                        if (w.Lats.Length == 0) { skipped.AddOrUpdate("no_locations", 1, (_, v) => v + 1); continue; }
                        var (x, y) = b.WayE7(w);
                        var xy = new int[(x.Length - 1) * 2];
                        for (var i = 0; i < x.Length - 1; i++) { xy[2 * i] = (int)x[i]; xy[2 * i + 1] = (int)y[i]; }
                        var cell = Buildings.CellOf(xy[0], xy[1]);
                        var attrs = Buildings.AttrsOf(t, myRefused, new List<string>(Roofs), grow: false);
                        buckets.Add(cell, w.Id, Buildings.Record(w.Id, kind.Value, attrs, new() { new Buildings.Ring(1, xy) }));
                        n++;
                    }
                }
                if (kinds.Contains("relations"))
                    foreach (var rel in b.Relations())
                    {
                        var t = Buildings.TagsOf(b, rel.Keys, rel.Vals);
                        var kind = Buildings.IsBuilding(t);
                        if (kind is null || t.GetValueOrDefault("type") != "multipolygon") continue;
                        var members = new List<(long, bool)>();
                        for (var i = 0; i < rel.Members.Length; i++)
                            if (rel.Types[i] == 1) members.Add((rel.Members[i], b.Strings[(int)rel.Roles[i]] != "inner"));
                        rels.Add(new Rel(rel.Id, kind.Value, Buildings.AttrsOf(t, myRefused, new List<string>(Roofs), grow: false), members));
                    }
            }
            buckets.FlushAll();
            lock (refused) foreach (var (k, v) in myRefused) refused[k] += v;
            var total = Interlocked.Add(ref nWays, n);
            var d = Interlocked.Increment(ref done);
            if (d % 200 == 0) ctx.Log($"[planet] pass 1: {total:N0} building ways, {rels.Count:N0} relations ({ctx.Seconds - t0:0} s)");
        });
        ctx.Log($"[planet] pass 1 done: {nWays:N0} ways, {rels.Count:N0} multipolygons ({ctx.Seconds - t0:0} s)");

        // pass 2: the relations' member ways -> coordinates; the parent stitches rings
        var members = rels.SelectMany(r => r.Members.Select(m => m.member)).Distinct().OrderBy(x => x).ToArray();
        var pieces = new ConcurrentDictionary<long, List<(long node, int x, int y)>>();
        Parallel.ForEach(Chunks(wayBlocks.OrderBy(x => x).ToList(), 64), new ParallelOptions { MaxDegreeOfParallelism = workers }, chunk =>
        {
            foreach (var off in chunk)
            {
                var b = new OsmPbf.Block(OsmPbf.BlobAt(pbf, off));
                foreach (var w in b.Ways())
                {
                    if (Array.BinarySearch(members, w.Id) < 0 || w.Lats.Length == 0) continue;
                    var (x, y) = b.WayE7(w);
                    var refs = OsmPbf.PackedDelta(w.Refs);
                    var l = new List<(long, int, int)>(refs.Length);
                    for (var i = 0; i < refs.Length; i++) l.Add((refs[i], (int)x[i], (int)y[i]));
                    pieces[w.Id] = l;
                }
            }
        });
        var own = new Buckets(tmp, "rel");
        var relNoRing = 0;
        foreach (var rel in rels)
        {
            var geo = new List<Buildings.Ring>();
            foreach (var outer in new[] { true, false })
            {
                var ways = rel.Members.Where(m => m.outer == outer && pieces.ContainsKey(m.member)).Select(m => pieces[m.member].Select(p => p.node).ToArray()).ToList();
                var coords = rel.Members.Where(m => m.outer == outer && pieces.ContainsKey(m.member)).SelectMany(m => pieces[m.member]).GroupBy(p => p.node).ToDictionary(g => g.Key, g => (g.First().x, g.First().y));
                foreach (var ring in Buildings.Stitch(ways))
                {
                    var xy = new int[(ring.Count - 1) * 2];
                    for (var i = 0; i < ring.Count - 1; i++) { var (x, y) = coords[ring[i]]; xy[2 * i] = x; xy[2 * i + 1] = y; }
                    geo.Add(new Buildings.Ring((byte)(outer ? 1 : 0), xy));
                }
            }
            geo = geo.OrderByDescending(g => g.Outer).ToList();
            if (geo.Count == 0 || geo[0].Outer == 0) { relNoRing++; continue; }
            own.Add(Buildings.CellOf(geo[0].Xy[0], geo[0].Xy[1]), -rel.Id, Buildings.Record(-rel.Id, rel.Kind, rel.A, geo));
        }
        own.FlushAll();
        skipped.AddOrUpdate("relation_no_ring", relNoRing, (_, v) => v + relNoRing);
        pieces.Clear();
        ctx.Log($"[planet] pass 2 done: relations stitched ({ctx.Seconds - t0:0} s)");

        // pass 3: each bucket's parts -> a chunk sorted by (iy, ix, id); concatenated in bucket order
        var parts = Directory.GetFiles(tmp, "*.part").GroupBy(q => Path.GetFileName(q).Split('.')[0]).OrderBy(g => g.Key, StringComparer.Ordinal).ToList();
        var binPath = Path.Combine(outDir, stem + ".buildings.bin");
        var cells = new List<(int iy, int ix, long off, long n)>();
        long total = 0;
        var results = new ConcurrentDictionary<string, (string chunk, List<(int iy, int ix, long off, long n)> cells, long n)>();
        Parallel.ForEach(parts, new ParallelOptions { MaxDegreeOfParallelism = Math.Min(workers, 6) }, g =>
        {
            var chunkPath = Path.Combine(tmp, g.Key + ".chunk");
            var data = new MemoryStream();
            foreach (var p in g) data.Write(File.ReadAllBytes(p));
            var buf = data.ToArray();
            var heads = new List<(int iy, int ix, long sid, int start, int n)>();
            var p0 = 0;
            while (p0 < buf.Length)
            {
                var iy = BinaryPrimitives.ReadInt32LittleEndian(buf.AsSpan(p0));
                var ix = BinaryPrimitives.ReadInt32LittleEndian(buf.AsSpan(p0 + 4));
                var sid = BinaryPrimitives.ReadInt64LittleEndian(buf.AsSpan(p0 + 8));
                var n = (int)BinaryPrimitives.ReadUInt32LittleEndian(buf.AsSpan(p0 + 16));
                heads.Add((iy, ix, sid, p0 + EntrySize, n));
                p0 += EntrySize + n;
            }
            heads.Sort((x, y) => x.iy != y.iy ? x.iy.CompareTo(y.iy) : x.ix != y.ix ? x.ix.CompareTo(y.ix) : x.sid.CompareTo(y.sid));
            var bc = new List<(int iy, int ix, long off, long n)>();
            long off = 0;
            using (var f = File.Create(chunkPath))
                foreach (var (iy, ix, _, start, n) in heads)
                {
                    if (bc.Count == 0 || bc[^1].iy != iy || bc[^1].ix != ix) bc.Add((iy, ix, off, 0));
                    bc[^1] = (iy, ix, bc[^1].off, bc[^1].n + 1);
                    f.Write(buf, start, n);
                    off += n;
                }
            foreach (var q in g) File.Delete(q);
            results[g.Key] = (chunkPath, bc, heads.Count);
        });
        using (var f = File.Create(binPath))
        {
            f.Write("GABLDG01"u8);
            foreach (var g in parts)
            {
                var (chunkPath, bc, n) = results[g.Key];
                var bse = f.Position;
                using (var c = File.OpenRead(chunkPath)) c.CopyTo(f);
                File.Delete(chunkPath);
                cells.AddRange(bc.Select(x => (x.iy, x.ix, bse + x.off, x.n)));
                total += n;
            }
        }
        cells.Sort((x, y) => x.iy != y.iy ? x.iy.CompareTo(y.iy) : x.ix.CompareTo(y.ix));
        using (var f = File.Create(Path.Combine(outDir, stem + ".buildings.idx")))
            foreach (var (iy, ix, off, n) in cells) { Bin.I32(f, ix); Bin.I32(f, iy); Bin.I64(f, off); Bin.I64(f, n); }
        var manifest = D(("format", "GABLDG01"),
            ("source", D(("file", Path.GetFileName(pbf)), ("bytes", new FileInfo(pbf).Length), ("header", hdr), ("tool", "osmium tags-filter + add-locations-to-ways"))),
            ("licence", "ODbL 1.0"), ("attribution", "(c) OpenStreetMap contributors"),
            ("units", D(("coordinates", "int32, 1e-7 degree, WGS84 lon/lat (OSM's own precision)"), ("height", "metres above the ground at the footprint"), ("levels", "count"))),
            ("datum", "ground"), ("counts", D(("records", total), ("ways", nWays), ("multipolygons", rels.Count))),
            ("refused", refused), ("skipped", skipped.ToDictionary(kv => kv.Key, kv => (object?)kv.Value)),
            ("roofShapes", Roofs.Append("(other: 254)").ToList()), ("cellDeg", Buildings.CellDeg),
            ("cellIndex", stem + ".buildings.idx"), ("cellCount", cells.Count));
        Save(Path.Combine(outDir, stem + ".buildings.json"), manifest);
        if (Directory.GetFileSystemEntries(tmp).Length == 0) Directory.Delete(tmp);
        ctx.Log($"[planet] {total:N0} solids in {cells.Count:N0} cells -> {binPath} ({new FileInfo(binPath).Length / 1073741824.0:0.0} GB) in {(ctx.Seconds - t0) / 60:0} min");
        ctx.Log($"[planet] refused {Dump(refused, 0)}; skipped {Dump(skipped.ToDictionary(k => k.Key, k => (object?)k.Value), 0)}");
        return 0;
    }

    static List<List<long>> Chunks(List<long> seq, int n)
    {
        var out_ = new List<List<long>>();
        for (var i = 0; i < seq.Count; i += n) out_.Add(seq.GetRange(i, Math.Min(n, seq.Count - i)));
        return out_;
    }
}
