using System.Collections.Concurrent;
using System.Diagnostics;
using System.Globalization;
using Gagame.Harvest.Readers;
using static Gagame.Harvest.JsonUtil;

namespace Gagame.Harvest.Harvesters;

/// THE BED, MANY SOURCES IN, ONE PRODUCT OUT (docs/FIDELITY.md 3): for every region the fidelity
/// map paints, the finest grain the sources offer that is no finer than the region wants, as
/// tiers of float32 GeoTIFFs in EPSG:4326 (one grain a tier), gaps under them filled by the
/// coarser sources' own tiers, the global base under everything.
///
/// THE SOURCES are the configs of harvest/sources/relief (or the `--sources` ids), made by the
/// SourceFactory: each its reader (`kind`), native step, datum and rank (docs/FIDELITY.md 8).
///
/// THE LAW. A source of native step a offers the grains a 2^k (k = 0 its own pixels, k > 0 the
/// mean of 2^k x 2^k of them). Of the sources of one native step holding files in a region, the
/// lowest rank answers. A region wanting grain g takes, of those, the least offered grain >= g (a
/// tie goes to the coarser native: the same grain from fewer bytes), and under it every source of
/// coarser native at ITS least grain >= g: the gap fill. A region wanting +Infinity (unpainted,
/// the default) takes the coarsest source at its own grain. The grain of a pixel is its latitude
/// extent on the sphere (the coarser axis).
///
/// THE LATTICE. A tier's pixels are its source file's own pixels (k = 0: copied, not resampled)
/// or blocks of them aligned to the global lattice of that step (k > 0), so two runs and two maps
/// write the same tile for the same (source, grain, tile). A tile is the 0.25 deg x 2^m box of at
/// most 2048 pixels a side, cut to the source file's cell; a tile with no data is not written.
///
/// THE DATUM. The product's datum is the finest source's (CUDEM: NAVD88, GeoKey 5703). Every
/// source is written in its own datum (ETOPO 2022: EGM2008, GeoKey 3855): a separation between
/// two datums is a field, not a constant, so a tier of another datum carries the offset MEASURED
/// where a product-datum source and that datum's finest source both hold the ground of this map
/// (the median over the reference's cells of it minus the product source's area mean) and its
/// spread, and the engine adds it as the entry's `offset` (compose/RasterFileSource.h).
public sealed class ComposeBed : IHarvester
{
    public string Name => "compose-bed";
    public string Title => "Bed composite (the relief sources by configuration)";
    public string What => "Every painted region's seafloor and land from the finest source that has it (harvest/sources/relief: CUDEM 1/9\" and 1/3\" over ETOPO 2022 15\" over 60\", an ERDDAP grid), as tiers of GeoTIFFs.";
    public string[] Outputs => [];
    public Source[] Sources => [new("NOAA CUDEM (NODD)", "https://registry.opendata.aws/noaa-coastal-lidar/"), new("ETOPO 2022", "https://www.ncei.noaa.gov/products/etopo-global-relief-model"),
        new("ERDDAP griddap (GEBCO 2020 on CoastWatch)", "https://coastwatch.pfeg.noaa.gov/erddap/griddap/GEBCO_2020.html")];
    public Insert? Insert => new(InsertKind.FileArg, "A fidelity map (.gafid) and an output folder: --fidelity FILE --out data/composite/<name>/ [--sources id,id]");

    const double R = 6371000.0;
    const float NoData = -99999.0f;
    const int MaxTilePx = 2048;

    // ------------------------------------------------------------------ sources

    /// A relief source as the bed ranks it: its config (harvest/sources/relief/<id>.json, made by
    /// the SourceFactory) and the law's offer.
    sealed class Src(IReliefSource r)
    {
        public readonly string Id = r.Config.Id, Short = r.Config.S("short"), Title = r.Config.Title, Label = r.Config.Label, Url = r.Config.Url ?? r.Config.Server ?? "",
            License = r.Config.License, Cite = r.Config.Cite, Datum = r.Config.Datum!;
        public readonly int VEpsg = r.Config.VerticalEpsg, Rank = r.Config.Rank;
        public readonly double StepDeg = r.Config.StepDeg;
        public double NativeM => StepDeg * Geo.DegToRad * R;
        public List<ReliefFile> Meets(Box box) => r.Meets(box);
        /// The least grain this source offers that is >= g (k, grain); +Infinity wants k = 0.
        public (int k, double grain) Offer(double g)
        {
            if (!(g > NativeM) || double.IsInfinity(g)) return (0, NativeM);
            var k = (int)Math.Ceiling(Math.Log2(g / NativeM) - 1e-9);
            return (k, NativeM * Math.Pow(2, k));
        }
    }

    CompositeRun _run = null!;

    // ------------------------------------------------------------------ tiers

    sealed record TierKey(Src S, int K)
    {
        public double Grain => S.NativeM * Math.Pow(2, K);
        public string Folder => $"{S.Short}_{Grain.ToString("0.##", CultureInfo.InvariantCulture)}m";
    }

    sealed class OutTile
    {
        public ReliefFile File = null!;
        public Box Box = null!;
        public string Name = "";
        public int Part;
    }

    /// THE PARTS OF A TIER: its tiles grouped where they touch (an edge or a corner), each group one
    /// folder. The engine takes a raster set's footprint as ONE box (compose/RasterFileSource.h), so a
    /// tier whose regions lie 1,700 km apart, read as one set, claimed every tile between them, and its
    /// trees repainted the coast from one region to the next (2026-10-13: the Merrimack still never
    /// settled, 17,700 tiles pending). A part's box is its tiles' own. Returns the parts' boxes.
    static List<Box> Parts(List<OutTile> ts)
    {
        var parent = Enumerable.Range(0, ts.Count).ToArray();
        int Find(int i) { while (parent[i] != i) i = parent[i] = parent[parent[i]]; return i; }
        const double eps = 1e-9;
        // touching tiles share a corner of the tier's own grid: index them by corner
        var byCorner = new Dictionary<(long, long), List<int>>();
        for (int i = 0; i < ts.Count; ++i)
        {
            var b = ts[i].Box;
            foreach (var lon in new[] { b.Lon0, b.Lon1 })
                foreach (var lat in new[] { b.Lat0, b.Lat1 })
                {
                    var key = ((long)Math.Round(lon / eps / 1e3), (long)Math.Round(lat / eps / 1e3));
                    if (!byCorner.TryGetValue(key, out var l)) byCorner[key] = l = new();
                    l.Add(i);
                }
        }
        foreach (var l in byCorner.Values)
            for (int k = 1; k < l.Count; ++k) parent[Find(l[k])] = Find(l[0]);
        var roots = new Dictionary<int, int>();
        var boxes = new List<Box>();
        for (int i = 0; i < ts.Count; ++i)
        {
            int r = Find(i);
            if (!roots.TryGetValue(r, out int p)) { roots[r] = p = boxes.Count; boxes.Add(ts[i].Box); }
            ts[i].Part = p;
            var b = boxes[p];
            var t = ts[i].Box;
            boxes[p] = new Box(Math.Min(b.Lon0, t.Lon0), Math.Min(b.Lat0, t.Lat0), Math.Max(b.Lon1, t.Lon1), Math.Max(b.Lat1, t.Lat1));
        }
        return boxes;
    }

    /// The aligned tiles of edge `e` degrees meeting `box`, cut to `cell`.
    static IEnumerable<Box> Tiles(Box box, Box cell, double e)
    {
        double lo0 = Math.Max(box.Lon0, cell.Lon0), lo1 = Math.Min(box.Lon1, cell.Lon1), la0 = Math.Max(box.Lat0, cell.Lat0), la1 = Math.Min(box.Lat1, cell.Lat1);
        if (!(lo0 < lo1 && la0 < la1)) yield break;
        for (var iy = (long)Math.Floor((la0 + 90) / e + 1e-9); iy * e - 90 < la1 - 1e-12; iy++)
            for (var ix = (long)Math.Floor((lo0 + 180) / e + 1e-9); ix * e - 180 < lo1 - 1e-12; ix++)
            {
                var t = new Box(Math.Max(ix * e - 180, cell.Lon0), Math.Max(iy * e - 90, cell.Lat0), Math.Min((ix + 1) * e - 180, cell.Lon1), Math.Min((iy + 1) * e - 90, cell.Lat1));
                if (t.Lon0 < t.Lon1 && t.Lat0 < t.Lat1) yield return t;
            }
    }

    static double TileEdgeDeg(double stepDeg) => 0.25 * Math.Pow(2, Math.Floor(Math.Log2(MaxTilePx * stepDeg / 0.25)));

    static string TileName(Box t)
    {
        static string F(double v) => Math.Round(Math.Abs(v), 6).ToString("0.######", CultureInfo.InvariantCulture).Replace('.', 'x');
        return $"{(t.Lat1 >= 0 ? 'n' : 's')}{F(t.Lat1)}_{(t.Lon0 >= 0 ? 'e' : 'w')}{F(t.Lon0)}.tif";
    }

    // ------------------------------------------------------------------ the run

    public int Run(HarvestContext ctx, string[] argv)
    {
        var a = new Args(argv);
        var (mapPath, outDir) = CompositeRun.Paths(a, Name);
        _run = new CompositeRun(ctx, new Polite(ctx), mapPath, outDir, "water.bed", "height");
        var sources = SourceFactory.ReliefSources(new SourceEnv(ctx, _run.Polite, _run.Log), a.Opt("--sources"))
            .Select(r => new Src(r)).OrderBy(s => s.StepDeg).ThenBy(s => s.Rank).ToList();
        if (sources.Count == 0) throw new HarvestStop("compose-bed: no relief source configured");
        // THE PRODUCT'S DATUM is the finest source's; every other datum's tiers carry the offset
        // measured against it (below)
        var datum = sources[0].Datum;
        _run.Log($"{sources.Count} relief source(s): {string.Join(", ", sources.Select(s => $"{s.Id} ({s.Datum}, rank {s.Rank})"))}");

        var regions = _run.Regions();
        regions.Add((new Box(-180, -90, 180, 90), _run.DefaultGrainM));   // the unpainted rest wants the default
        _run.Log($"{regions.Count - 1} painted region(s) + the default {(double.IsInfinity(_run.DefaultGrainM) ? "(the coarsest source)" : _run.DefaultGrainM + " m")} everywhere else");

        // ---- every region: the tiers it takes, and the tiles of each
        var tiles = new Dictionary<TierKey, Dictionary<string, OutTile>>();
        var regionTiers = new List<(Box box, double g, List<TierKey> tiers)>();
        foreach (var (box, g) in regions)
        {
            // of the sources of one native step holding files here, the best ranked answers
            var have = sources.Select(s => (s, files: s.Meets(box))).Where(x => x.files.Count > 0)
                .GroupBy(x => x.s.StepDeg).Select(grp => grp.OrderBy(x => x.s.Rank).First()).ToList();
            if (have.Count == 0) continue;
            List<TierKey> take;
            if (double.IsInfinity(g)) take = [new TierKey(have[^1].s, 0)];
            else
            {
                var best = have.Select(x => (x.s, offer: x.s.Offer(g))).OrderBy(x => x.offer.grain).ThenByDescending(x => x.s.StepDeg).First();
                take = [new TierKey(best.s, best.offer.k)];
                take.AddRange(have.Where(x => x.s.StepDeg > best.s.StepDeg).Select(x => new TierKey(x.s, x.s.Offer(g).k)));
            }
            regionTiers.Add((box, g, take));
            foreach (var tk in take)
            {
                if (!tiles.TryGetValue(tk, out var set)) tiles[tk] = set = new();
                var e = TileEdgeDeg(tk.S.StepDeg * Math.Pow(2, tk.K));
                foreach (var f in have.First(x => x.s == tk.S).files)
                    foreach (var t in Tiles(box, f.Cell, e))
                    {
                        var name = TileName(t);
                        set.TryAdd(name, new OutTile { File = f, Box = t, Name = name });
                    }
            }
        }

        // ---- fetch what is not cached, planned against the budget, finest tier first
        var plan = _run.Polite.NewPlan("bed composite sources");
        var toMake = new Dictionary<string, ReliefFile>();
        foreach (var f in tiles.OrderBy(t => t.Key.Grain).SelectMany(t => t.Value.Values.Select(o => o.File)).DistinctBy(f => f.Dest))
            if (!File.Exists(f.Dest))
            {
                f.Size ??= f.SizeOf?.Invoke() ?? _run.Polite.Head(f.Url).Length;
                plan.Add(f.Key, f.Url, f.Size, f.Dest);
                toMake[f.Dest] = f;
            }
        var later = new HashSet<string>();
        if (plan.Items.Count > 0)
        {
            var (keep, cut) = plan.Fit();
            plan.Print(keep, cut);
            foreach (var it in keep.Where(i => !i.Cached))
                if (toMake[it.Dest].Make is { } make) make(_run.Polite);
                else _run.Polite.Download(it.Url, it.Dest, it.Size!.Value, note: "compose-bed");
            foreach (var it in cut) later.Add(it.Dest);
        }

        // ---- the datums: each other datum's offset to the product's, measured against that
        // datum's finest source where a source of the product's datum holds the ground
        var otherDatums = tiles.Keys.Select(k => k.S.Datum).Where(d => d != datum).Distinct().ToList();
        var refs = otherDatums.ToDictionary(d => d, d => sources.First(s => s.Datum == d));
        var sep = otherDatums.ToDictionary(d => d, _ => new ConcurrentBag<(string file, double d)>());
        var refCache = new ConcurrentDictionary<string, GeoTiff?>();
        GeoTiff? RefAt(Src rs, Box cell)
        {
            var f = rs.Meets(new Box(cell.CenterLon, cell.CenterLat, cell.CenterLon, cell.CenterLat)).FirstOrDefault();
            if (f is null || !File.Exists(f.Dest)) return null;
            return refCache.GetOrAdd(f.Dest, p => new GeoTiff(File.ReadAllBytes(p)));
        }

        // ---- write every tier
        var tierDocs = new List<Dictionary<string, object?>>();
        var ordered = tiles.Keys.OrderBy(t => t.Grain).ThenBy(t => t.S.Rank).ToList();
        var tierStats = new Dictionary<TierKey, Dictionary<string, object?>>();
        var tierParts = new Dictionary<TierKey, List<Box>>();
        double tRead = 0, tDecode = 0, tResample = 0, tWrite = 0;
        long decodedPx = 0;
        foreach (var tk in ordered)
        {
            var clock = Stopwatch.StartNew();
            var dir = Path.Combine(outDir, "bed", tk.Folder);
            if (Directory.Exists(dir)) Directory.Delete(dir, recursive: true);   // our own output folder, written whole below
            var parts = Parts(tiles[tk].Values.ToList());
            for (int p = 0; p < parts.Count; ++p) Directory.CreateDirectory(Path.Combine(dir, $"p{p}"));
            long bytesIn = 0, bytesOut = 0, written = 0, pixels = 0;
            var byFile = tiles[tk].Values.GroupBy(o => o.File.Dest).ToList();
            foreach (var grp in byFile)
            {
                var src = grp.First().File;
                if (!File.Exists(src.Dest)) continue;
                var sw = Stopwatch.StartNew();
                var bytes = File.ReadAllBytes(src.Dest);
                tRead += sw.Elapsed.TotalSeconds;
                bytesIn += bytes.Length;
                _run.ReadBytes(bytes.Length);
                var tif = new GeoTiff(bytes);
                var pointRaster = tif.GeoKeys().GetValueOrDefault("GTRasterType") is int rt && rt == 2;
                var lonEdge = tif.Lon0 - tif.TieI * tif.Sx - (pointRaster ? 0.5 * tif.Sx : 0);
                var latEdge = tif.Lat0 + tif.TieJ * tif.Sy + (pointRaster ? 0.5 * tif.Sy : 0);
                List<(string d, GeoTiff? t)> measure = tk.S.Datum != datum ? [] : otherDatums.Select(d => (d, t: RefAt(refs[d], src.Cell))).Where(x => x.t is not null).ToList();
                double dDec = 0, dRes = 0, dWr = 0;
                var lk = new object();
                Parallel.ForEach(grp, ot =>
                {
                    var t0 = Stopwatch.GetTimestamp();
                    var cut = Cut(tif, lonEdge, latEdge, 1 << tk.K, tk.S.StepDeg * (1 << tk.K), ot.Box, out var t1, out var decoded);
                    Interlocked.Add(ref decodedPx, decoded);
                    var t2 = Stopwatch.GetTimestamp();
                    if (cut is null) return;
                    var (vals, w, h, lon0, lat1, sx, sy) = cut.Value;
                    foreach (var (d, rt) in measure) Separation(rt!, vals, w, h, lon0, lat1, sx, sy, src.Key, sep[d]);
                    var path = Path.Combine(dir, $"p{ot.Part}", ot.Name);
                    var n = GeoTiffWriter.Write(path, vals, w, h, lon0, lat1, sx, sy, tk.S.VEpsg, NoData);
                    var t3 = Stopwatch.GetTimestamp();
                    lock (lk)
                    {
                        dDec += (t1 - t0) / (double)Stopwatch.Frequency;
                        dRes += (t2 - t1) / (double)Stopwatch.Frequency;
                        dWr += (t3 - t2) / (double)Stopwatch.Frequency;
                        bytesOut += n; written++; pixels += (long)w * h;
                    }
                });
                tDecode += dDec; tResample += dRes; tWrite += dWr;
            }
            _run.WroteBytes(bytesOut);
            var files = Directory.GetFiles(dir, "*.tif", SearchOption.AllDirectories).Length;
            if (files == 0) { Directory.Delete(dir, recursive: true); continue; }
            tierParts[tk] = parts;
            tierStats[tk] = D(("seconds", Round(clock.Elapsed.TotalSeconds, 2)), ("bytes_in", bytesIn), ("bytes_out", bytesOut), ("files", files), ("pixels", pixels));
            _run.Log($"tier {tk.Folder}: {files} file(s), {pixels / 1e6:0.0} Mpx, {bytesIn / 1e6:0.0} MB in, {bytesOut / 1e6:0.0} MB out, {clock.Elapsed.TotalSeconds:0.0} s");
        }

        // ---- the separation measured, the offset declared
        var offsets = new Dictionary<string, double?>();
        var separations = new Dictionary<string, object?>();
        var finest = sources.Where(s => s.Datum == datum).ToList();
        foreach (var d in otherDatums)
        {
            var rs = refs[d];
            var cell = ArcSec(rs.StepDeg);
            var ds = sep[d].Select(x => x.d).OrderBy(x => x).ToArray();
            double? offset = offsets[d] = ds.Length > 0 ? -Median(ds) : null;
            var perFile = sep[d].GroupBy(x => x.file).OrderBy(g => g.Key, StringComparer.Ordinal).Select(g =>
            {
                var v = g.Select(x => x.d).OrderBy(x => x).ToArray();
                return D(("cudem_tile", g.Key), ("cells", v.Length), ("median_etopo_minus_cudem_m", Round(Median(v), 3)), ("mad_m", Round(Mad(v), 3)));
            }).ToList();
            if (offset is double off) _run.Log($"{datum} - {d} offset {off:+0.000;-0.000;0.000} m from {ds.Length} {cell} cells of {rs.Id} (MAD {Mad(ds):0.000} m; per tile medians {string.Join(", ", perFile.Select(p => p["median_etopo_minus_cudem_m"]))})");
            else _run.Log($"no {cell} cell where a {datum} source and {rs.Id} both hold the ground: the {d} tiers' offset is unmeasured (0 declared)");
            separations[d] = D(("what", $"{rs.Label} ({d}) minus the mean of the {string.Join(" or ", finest.Select(s => s.Label))} ({datum}) samples over the same {cell} cell, cells wholly covered by them and flat within 1 m"),
                ("cells", ds.Length), ("median_m", ds.Length > 0 ? Round(Median(ds), 3) : null), ("mad_m", ds.Length > 0 ? Round(Mad(ds), 3) : null),
                ("p05_m", ds.Length > 0 ? Round(ds[(int)(0.05 * (ds.Length - 1))], 3) : null), ("p95_m", ds.Length > 0 ? Round(ds[(int)(0.95 * (ds.Length - 1))], 3) : null),
                ("per_tile", perFile));
        }

        foreach (var tk in ordered.Where(tierStats.ContainsKey))
        {
            var other = tk.S.Datum != datum;
            var offset = other ? offsets[tk.S.Datum] : null;
            for (int p = 0; p < tierParts[tk].Count; ++p)
            {
            if (!Directory.EnumerateFiles(Path.Combine(outDir, "bed", tk.Folder, $"p{p}"), "*.tif").Any()) continue;
            var pb = tierParts[tk][p];
            tierDocs.Add(D(("grain_m", Round(tk.Grain, 3)), ("step_deg", tk.S.StepDeg * Math.Pow(2, tk.K)), ("folder", $"bed/{tk.Folder}/p{p}"), ("match", "*.tif"),
                ("box", new List<double> { pb.Lon0, pb.Lat0, pb.Lon1, pb.Lat1 }),
                ("inputs", new List<object?> { tk.S.Id }), ("decimation", 1 << tk.K),
                ("datum", tk.S.Datum), ("vertical_epsg", tk.S.VEpsg), ("unit", "m"), ("nodata", (double)NoData),
                ("offset_m", other ? Round(offset ?? 0.0, 3) : 0.0),
                ("offset_measured", other ? offset is not null : null),
                ("source_files", tiles[tk].Values.Select(o => o.File.Dest).Distinct().Where(File.Exists).Select(_run.Rel).OrderBy(x => x, StringComparer.Ordinal).ToList()),
                ("stats", tierStats[tk])));
            }
        }

        foreach (var s in sources.Where(s => tierStats.Keys.Any(k => k.S == s)))
        {
            var used = tiles.Where(t => t.Key.S == s).SelectMany(t => t.Value.Values.Select(o => o.File.Dest)).Distinct().Where(File.Exists).ToList();
            var offset = s.Datum == datum ? null : offsets.GetValueOrDefault(s.Datum);
            _run.Input(s.Id, s.Title, Round(s.NativeM, 3), s.Url, s.License, s.Cite, s.Datum,
                s.Datum == datum ? "none" : $"declared {s.Datum}; offset {Round(offset ?? 0.0, 3)} m to {datum} {(offset is null ? "unmeasured" : "measured")}",
                fetchedUtc: used.Count > 0 ? used.Select(CompositeRun.FileUtc).Min() : null);
        }
        foreach (var (box, g, take) in regionTiers)
            foreach (var tk in take.Where(tierStats.ContainsKey))
                _run.Coverage.Add(D(("box", new List<double> { box.Lon0, box.Lat0, box.Lon1, box.Lat1 }), ("wanted_grain_m", double.IsInfinity(g) ? null : g),
                    ("grain_m", Round(tk.Grain, 3)), ("input", tk.S.Id), ("tier", $"bed/{tk.Folder}"), ("gap_fill", take[0] != tk)));
        // one other datum (every product so far) writes its separation as the object; several, an object of them by datum
        _run.Extra["datum_separation"] = separations.Count == 1 ? separations.Values.First() : separations;
        _run.Extra["later"] = later.Select(_run.Rel).OrderBy(x => x, StringComparer.Ordinal).ToList();
        _run.Extra["phases_s"] = D(("read", Round(tRead, 2)), ("decode", Round(tDecode, 2)), ("resample", Round(tResample, 2)), ("encode_write", Round(tWrite, 2)), ("decoded_mpx", Round(decodedPx / 1e6, 1)),
            ("decode_mb_per_thread_s", Round(decodedPx * 4 / 1e6 / Math.Max(tDecode, 1e-9), 1)),
            ("note", "decode, resample and encode_write are summed over the worker threads; read is the file loads, one at a time"));
        _run.Save(D(("unit", "m"), ("datum", datum), ("tiers", tierDocs)));
        return later.Count == 0 ? 0 : 3;
    }

    /// The pixels of tile `t` from one source file: its own (block 1) or the mean of block x block
    /// of them, a block on the global lattice of the tier's step, taken where its samples are at
    /// least half there (so a coverage edge stays where it was). NaN where nothing is. The tile
    /// declares the tier's exact `step`, not the file's rounded one (CUDEM writes 1/9" three ways,
    /// 3.08642E-05, 3.0864194E-05, 3.0864198718E-05, and the engine takes a set of one grain to
    /// 1e-9): the origin is the file's own, so a tile's far edge moves under a millimetre.
    static (float[] vals, int w, int h, double lon0, double lat1, double sx, double sy)? Cut(GeoTiff tif, double lonEdge, double latEdge, int block, double step, Box t, out long tDecoded, out long decodedPx)
    {
        double sx = tif.Sx, sy = tif.Sy;
        long gx0 = (long)Math.Floor((lonEdge + 180) / sx + 0.5), gy0 = (long)Math.Floor((90 - latEdge) / sy + 0.5);
        int bx = (int)(((-gx0) % block + block) % block), by = (int)(((-gy0) % block + block) % block);
        double bsx = sx * block, bsy = sy * block;
        int j0 = (int)Math.Ceiling((t.Lon0 - lonEdge) / bsx - (double)bx / block - 0.5 - 1e-9), j1 = (int)Math.Ceiling((t.Lon1 - lonEdge) / bsx - (double)bx / block - 0.5 - 1e-9);
        int i0 = (int)Math.Ceiling((latEdge - t.Lat1) / bsy - (double)by / block - 0.5 - 1e-9), i1 = (int)Math.Ceiling((latEdge - t.Lat0) / bsy - (double)by / block - 0.5 - 1e-9);
        j0 = Math.Max(j0, (int)Math.Ceiling(-(double)bx / block)); i0 = Math.Max(i0, (int)Math.Ceiling(-(double)by / block));
        j1 = Math.Min(j1, (tif.Width - bx + block - 1) / block); i1 = Math.Min(i1, (tif.Height - by + block - 1) / block);
        tDecoded = Stopwatch.GetTimestamp();
        decodedPx = 0;
        int w = j1 - j0, h = i1 - i0;
        if (w <= 0 || h <= 0) return null;
        int x0 = bx + j0 * block, y0 = by + i0 * block;
        int ww = Math.Min(w * block, tif.Width - x0), wh = Math.Min(h * block, tif.Height - y0);
        var src = tif.ReadWindow(x0, y0, ww, wh);
        decodedPx = (long)ww * wh;
        tDecoded = Stopwatch.GetTimestamp();
        float[] vals;
        if (block == 1) vals = src;
        else
        {
            vals = new float[w * h];
            var need = block * block / 2.0;
            for (var i = 0; i < h; i++)
                for (var j = 0; j < w; j++)
                {
                    double s = 0; var n = 0;
                    for (var yy = i * block; yy < Math.Min((i + 1) * block, wh); yy++)
                        for (var xx = j * block; xx < Math.Min((j + 1) * block, ww); xx++)
                        {
                            var v = src[yy * ww + xx];
                            if (!float.IsNaN(v)) { s += v; n++; }
                        }
                    vals[i * w + j] = n >= need ? (float)(s / n) : float.NaN;
                }
        }
        if (!vals.Any(v => !float.IsNaN(v))) return null;
        return (vals, w, h, lonEdge + x0 * sx, latEdge - y0 * sy, step, step);
    }

    /// ETOPO 15" minus the mean of the CUDEM samples inside each 15" cell this window wholly
    /// holds, where the samples span under a metre (flat ground: the difference is the datums',
    /// not the resampling's).
    static void Separation(GeoTiff e15, float[] vals, int w, int h, double lon0, double lat1, double sx, double sy, string key, ConcurrentBag<(string, double)> into)
    {
        int ex0 = (int)Math.Ceiling((lon0 - e15.Lon0) / e15.Sx - 1e-9), ex1 = (int)Math.Floor((lon0 + w * sx - e15.Lon0) / e15.Sx + 1e-9);
        int ey0 = (int)Math.Ceiling((e15.Lat0 - lat1) / e15.Sy - 1e-9), ey1 = (int)Math.Floor((e15.Lat0 - (lat1 - h * sy)) / e15.Sy + 1e-9);
        if (ex1 <= ex0 || ey1 <= ey0) return;
        var ev = e15.ReadWindow(ex0, ey0, ex1 - ex0, ey1 - ey0);
        for (var ey = ey0; ey < ey1; ey++)
            for (var ex = ex0; ex < ex1; ex++)
            {
                var e = ev[(ey - ey0) * (ex1 - ex0) + (ex - ex0)];
                if (float.IsNaN(e)) continue;
                double cl0 = e15.Lon0 + ex * e15.Sx, cla1 = e15.Lat0 - ey * e15.Sy;
                int x0 = (int)Math.Round((cl0 - lon0) / sx), x1 = (int)Math.Round((cl0 + e15.Sx - lon0) / sx);
                int y0 = (int)Math.Round((lat1 - cla1) / sy), y1 = (int)Math.Round((lat1 - (cla1 - e15.Sy)) / sy);
                if (x0 < 0 || y0 < 0 || x1 > w || y1 > h) continue;
                double s = 0; var n = 0; float lo = float.MaxValue, hi = float.MinValue;
                var whole = true;
                for (var y = y0; y < y1 && whole; y++)
                    for (var x = x0; x < x1; x++)
                    {
                        var v = vals[y * w + x];
                        if (float.IsNaN(v)) { whole = false; break; }
                        s += v; n++; lo = Math.Min(lo, v); hi = Math.Max(hi, v);
                    }
                if (!whole || n == 0 || hi - lo > 1.0f) continue;
                into.Add((key, e - s / n));
            }
    }

    static string ArcSec(double stepDeg) => (stepDeg * 3600).ToString("0.###", CultureInfo.InvariantCulture) + "\"";

    static double Median(double[] sorted) => sorted.Length == 0 ? double.NaN : sorted.Length % 2 == 1 ? sorted[sorted.Length / 2] : 0.5 * (sorted[sorted.Length / 2 - 1] + sorted[sorted.Length / 2]);
    static double Mad(double[] sorted)
    {
        var m = Median(sorted);
        return Median(sorted.Select(x => Math.Abs(x - m)).OrderBy(x => x).ToArray());
    }
}
