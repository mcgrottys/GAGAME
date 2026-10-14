using System.Collections.Concurrent;
using System.Diagnostics;
using System.Globalization;
using System.Text;
using System.Text.Json.Nodes;
using System.Text.RegularExpressions;

namespace Gagame.Harvest;

/// THE STORAGE INDEX: how many bytes are STORED, by source, parameter, store and lattice cell,
/// read off the disk and never estimated. It reads names, sizes and directories only -- never a
/// tile's payload:
///
///  - the engine's tile trees (`cache/trees/<name>.<id>/<tag>/f<face>_m<mip>_x<x>_y<y>[_<key>].bin`,
///    src/compose/TileTree.h) and their archives (`<tag>.gaa`, src/compose/TileArchive.h: a 16-byte
///    header and 24-byte records, key = face << 61 | mip << 56 | y << 28 | x, read as a directory),
///    and the compositor's older cache (`cache/composed/<channel>/<tag>/`, the same names);
///  - the composites (`data/composite/<product>/<layer>.composite.json`, docs/FIDELITY.md section 3):
///    each tier's GeoTIFF tiles by their file sizes, each point file by its points, and coverage[].
///
/// Each byte is put in one cell of the fidelity lattice (FidelityMap: the engine's own cube
/// faces), at most MaxLevel deep. A cube tile IS a lattice cell: the realization `cube<K>k` has a
/// face of F = K x 1024 texels, a tile 128 texels high, so the tile (f, m, x, y) is the cell
/// (f, log2(F) - 7 - m, x, y) -- or, for a 256-wide tile, the two cells x2 and x2 + 1 of that
/// level, half its bytes each. A window tile (`window_z<z>_<orgX>_<orgY>`, Web-Mercator pixels of
/// zoom z - m, src/core/Lattice.cpp Box) goes to the cell holding its centre at the level of its
/// own ground size. What is DERIVED and how is said where it is done (`Derived` on each entry).
public sealed class StorageIndex
{
    public const string Magic = "GASTOR03";
    /// The deepest cell bytes are put in: about 2.4 km (a finer tile adds to its level-12 ancestor).
    /// Measured on the owner's cache (13.2 M tiles, 2026-10-14): level 14 kept 1.47 M aggregates in a
    /// 55 MB index; level 12 keeps the index small enough to hold whole in the painter.
    public const int MaxLevel = 12;

    static readonly FidelityMap Earth = new();

    /// One aggregate: bytes and tiles of one source, parameter, store and lattice in one cell.
    public sealed record Entry(string Source, string Param, string Store, string Lattice, ulong Cell, long Bytes, int Tiles, bool Derived);

    /// Bytes on disk that no cell holds (an unparsed name, a set-aside archive's tiles are mapped).
    public sealed record Unmapped(string Source, string Param, string Store, long Bytes, int Files, string Why);

    /// A composite's coverage[]: which input answers a box at what grain (read, as written).
    public sealed record Coverage(string Product, string Layer, string Input, double Lon0, double Lat0, double Lon1, double Lat1, double GrainM, bool GapFill);

    /// One folder or file the index read, keyed by its path and stamp, so a rescan reads only
    /// what changed (a directory's write time moves when a tile is added, renamed or removed).
    public sealed class Folder
    {
        public string Path = "";
        public long Stamp;
        public string Note = "";
        public int Voids;
        public List<Entry> Entries = new();
        public List<Unmapped> Unmapped = new();
        public List<Coverage> Coverage = new();
    }

    public readonly Dictionary<string, Folder> Folders = new(StringComparer.OrdinalIgnoreCase);
    public string Root = "";
    public double ScanSeconds;
    public int FoldersRead, FoldersReused;
    /// Did the last Update read or drop anything (else the saved index is already this one)?
    public bool Changed;
    public long FilesSeen;

    public IEnumerable<Entry> Entries => Folders.Values.SelectMany(f => f.Entries);
    public IEnumerable<Unmapped> AllUnmapped => Folders.Values.SelectMany(f => f.Unmapped);
    public IEnumerable<Coverage> AllCoverage => Folders.Values.SelectMany(f => f.Coverage);

    public static ulong Pack(FidelityMap.CellKey c) => (ulong)c.Face << 61 | (ulong)c.Level << 56 | (ulong)c.Y << 28 | c.X;
    public static FidelityMap.CellKey Unpack(ulong k) =>
        new((byte)(k >> 61), (byte)((k >> 56) & 31), (uint)(k & 0xFFFFFFF), (uint)((k >> 28) & 0xFFFFFFF));

    static FidelityMap.CellKey Cap(FidelityMap.CellKey c) =>
        c.Level <= MaxLevel ? c : new(c.Face, MaxLevel, c.X >> (c.Level - MaxLevel), c.Y >> (c.Level - MaxLevel));

    // ------------------------------------------------------------------ the scan

    /// Brings the index up to date with the checkout's stores: every folder whose stamp moved is
    /// read again, every folder gone is dropped, the rest kept. `trees`, `composed`, `composites`
    /// default to cache/trees, cache/composed and data/composite under `root`.
    public void Update(string root, string? trees = null, string? composed = null, string? composites = null, Action<string>? log = null)
    {
        var sw = Stopwatch.StartNew();
        Root = Path.GetFullPath(root);
        trees ??= Path.Combine(Root, "cache", "trees");
        composed ??= Path.Combine(Root, "cache", "composed");
        composites ??= Path.Combine(Root, "data", "composite");
        var jobs = new List<(string path, long stamp, Func<Folder> read)>();

        // the trees: <trees>/<name>.<id>/<tag>/ and <tag>.gaa[.stale]; composed: <channel>/<tag>/
        foreach (var (top, kind) in new[] { (trees, "tree"), (composed, "composed") })
        {
            if (!Directory.Exists(top)) continue;
            foreach (var node in new DirectoryInfo(top).EnumerateDirectories())
            {
                var (source, param) = kind == "tree" ? TreeName(node.Name) : ("composed/" + node.Name, node.Name);
                foreach (var e in node.EnumerateFileSystemInfos())
                {
                    if (e is DirectoryInfo d && ParseTag(d.Name) is not null)
                        jobs.Add((d.FullName, Stamp(d.FullName, dir: true), () => ReadTagFolder(d, source, param, "loose")));
                    else if (e is FileInfo f && f.Name.Contains(".gaa", StringComparison.OrdinalIgnoreCase))
                    {
                        var tag = f.Name[..f.Name.IndexOf(".gaa", StringComparison.OrdinalIgnoreCase)];
                        var store = f.Name.EndsWith(".gaa", StringComparison.OrdinalIgnoreCase) ? "archive" : "archive" + f.Name[(f.Name.IndexOf(".gaa", StringComparison.OrdinalIgnoreCase) + 4)..];
                        jobs.Add((f.FullName, Stamp(f.FullName, dir: false), () => ReadArchive(f, tag, source, param, store)));
                    }
                }
            }
        }
        // the composites: <product>/<layer>.composite.json, stamped by the document (a product is rewritten whole)
        if (Directory.Exists(composites))
            foreach (var product in new DirectoryInfo(composites).EnumerateDirectories())
                foreach (var doc in product.EnumerateFiles("*.composite.json"))
                {
                    jobs.Add((doc.FullName, Stamp(doc.FullName, dir: false), () => ReadComposite(doc, product.Name)));
                }

        var fresh = new ConcurrentDictionary<string, Folder>(StringComparer.OrdinalIgnoreCase);
        int read = 0, reused = 0;
        long files = 0;
        Parallel.ForEach(jobs, new ParallelOptions { MaxDegreeOfParallelism = Math.Min(8, Environment.ProcessorCount) }, job =>
        {
            if (Folders.TryGetValue(job.path, out var old) && old.Stamp == job.stamp) { fresh[job.path] = old; Interlocked.Increment(ref reused); return; }
            Folder f;
            try { f = job.read(); }
            catch (Exception e) when (e is IOException or UnauthorizedAccessException or InvalidDataException or System.Text.Json.JsonException or FormatException)
            {
                f = new Folder { Note = "unreadable: " + e.Message };
            }
            f.Path = job.path;
            f.Stamp = job.stamp;
            fresh[job.path] = f;
            Interlocked.Increment(ref read);
            Interlocked.Add(ref files, f.Entries.Sum(x => (long)x.Tiles) + f.Voids + f.Unmapped.Sum(u => (long)u.Files));
        });
        Changed = read > 0 || fresh.Count != Folders.Count;
        Folders.Clear();
        foreach (var (k, v) in fresh) Folders[k] = v;
        FoldersRead = read;
        FoldersReused = reused;
        FilesSeen = files;
        ScanSeconds = sw.Elapsed.TotalSeconds;
        log?.Invoke($"[storage] {jobs.Count} folders and files: {read} read, {reused} unchanged, {files:#,0} tiles read in {ScanSeconds:0.0} s");
    }

    /// A folder's or file's stamp, asked of the item itself: the copy of the times NTFS keeps in the
    /// PARENT's directory listing is updated lazily (a rescan stamped from it read one unchanged
    /// planted folder again; stamped from the item, three runs read none).
    static long Stamp(string path, bool dir) =>
        dir ? Directory.GetLastWriteTimeUtc(path).Ticks : File.GetLastWriteTimeUtc(path).Ticks ^ new FileInfo(path).Length;

    /// `<name>.<8 hex>[.<suffix>]` -> source = the whole folder name (the tree's identity: the
    /// hash of name and structure, TileTree.h), parameter = the name (what the tree holds).
    static (string source, string param) TreeName(string folder)
    {
        var m = Regex.Match(folder, @"^(.*)\.([0-9a-f]{8})(\..*)?$");
        return m.Success ? (folder, m.Groups[1].Value) : (folder, folder);
    }

    /// A realization tag (src/core/Lattice.cpp Tag): `cube<K>k` (face K x 1024 texels) or
    /// `<kind>_z<zoom>_<orgX>_<orgY>` (a Web-Mercator window).
    sealed record Tag(string Name, bool Cube, long FaceDim, int Zoom, long OrgX, long OrgY);

    static Tag? ParseTag(string name)
    {
        var c = Regex.Match(name, @"^cube(\d+)k$");
        if (c.Success) return new Tag(name, true, long.Parse(c.Groups[1].Value, CultureInfo.InvariantCulture) * 1024, 0, 0, 0);
        var w = Regex.Match(name, @"^[a-z]+_z(\d+)_(-?\d+)_(-?\d+)$");
        if (w.Success) return new Tag(name, false, 0, int.Parse(w.Groups[1].Value, CultureInfo.InvariantCulture),
            long.Parse(w.Groups[2].Value, CultureInfo.InvariantCulture), long.Parse(w.Groups[3].Value, CultureInfo.InvariantCulture));
        return null;
    }

    static readonly Regex TileName = new(@"^f(\d+)_m(\d+)_x(\d+)_y(\d+)(?:_[0-9a-f]{8})?\.(bin|void)$", RegexOptions.Compiled);

    readonly record struct Tile(uint Face, int Mip, uint X, uint Y, long Bytes);

    Folder ReadTagFolder(DirectoryInfo d, string source, string param, string store)
    {
        var f = new Folder();
        var tiles = new List<Tile>();
        long otherBytes = 0;
        int other = 0;
        foreach (var file in d.EnumerateFiles())
        {
            var m = TileName.Match(file.Name);
            if (!m.Success) { if (!file.Name.StartsWith('.')) { other++; otherBytes += file.Length; } continue; }
            if (m.Groups[5].Value == "void") { f.Voids++; continue; }
            tiles.Add(new Tile(uint.Parse(m.Groups[1].Value), int.Parse(m.Groups[2].Value), uint.Parse(m.Groups[3].Value), uint.Parse(m.Groups[4].Value), file.Length));
        }
        Place(f, ParseTag(d.Name)!, tiles, source, param, store);
        if (other > 0) f.Unmapped.Add(new Unmapped(source, param, store, otherBytes, other, "files not named f_m_x_y"));
        return f;
    }

    Folder ReadArchive(FileInfo file, string tag, string source, string param, string store)
    {
        var f = new Folder();
        var t = ParseTag(tag);
        using var s = file.OpenRead();
        var head = new byte[16];
        if (s.Read(head, 0, 16) != 16 || BitConverter.ToUInt32(head, 0) != 0x52414147u)
        {
            f.Unmapped.Add(new Unmapped(source, param, store, file.Length, 1, "not a GAAR archive"));
            return f;
        }
        uint n = BitConverter.ToUInt32(head, 8);
        var dir = new byte[n * 24L];
        s.ReadExactly(dir);
        var tiles = new List<Tile>((int)n);
        long payload = 0;
        for (int i = 0; i < n; ++i)
        {
            ulong key = BitConverter.ToUInt64(dir, i * 24);
            uint size = BitConverter.ToUInt32(dir, i * 24 + 16);
            payload += size;
            tiles.Add(new Tile((uint)(key >> 61), (int)((key >> 56) & 31), (uint)(key & 0xFFFFFFF), (uint)((key >> 28) & 0xFFFFFFF), size));
        }
        if (t is null) f.Unmapped.Add(new Unmapped(source, param, store, payload, (int)n, $"archive of an unknown tag '{tag}'"));
        else Place(f, t, tiles, source, param, store);
        // the directory and the 64 KB alignment are bytes on disk no tile holds
        if (file.Length > payload) f.Unmapped.Add(new Unmapped(source, param, store, file.Length - payload, 0, "archive directory and alignment"));
        return f;
    }

    /// The tile's width in texels: 128 (RGBA8, Raw4) or 256 (Half, FloatW) -- the tenant's, not
    /// written in the folder, so DERIVED from what the folder shows (`Note` says which rule held):
    /// a 256 KB tile is FloatW (256 x 128 x 8 bytes); a tile at an x or a mip a 256-wide tile
    /// cannot reach is 128; at the mip where one 256-wide tile spans a face, tiles at y = 1 and
    /// none at x = 1 are 256; else 128 is assumed.
    static (int texW, string why) TileWidth(Tag t, List<Tile> tiles)
    {
        if (tiles.Any(x => x.Bytes == 262144)) return (256, "256 KB tiles: FloatW 256x128");
        if (t.Cube)
        {
            if (tiles.Any(x => (t.FaceDim >> x.Mip) < 256 || x.X * 256L >= (t.FaceDim >> x.Mip))) return (128, "a tile past a 256-wide face: 128x128");
            int top = (int)Math.Log2(t.FaceDim / 256.0);
            var atTop = tiles.Where(x => x.Mip == top).ToList();
            if (atTop.Any(x => x.Y == 1) && atTop.All(x => x.X == 0)) return (256, "the coarsest 256-wide level holds y=1 and no x=1: 256x128");
        }
        return (128, "assumed 128x128 (a 64 KB tile is 128x128 colour or 256x128 half; nothing in the folder decides)");
    }

    void Place(Folder f, Tag t, List<Tile> tiles, string source, string param, string store)
    {
        if (tiles.Count == 0) return;
        var (texW, why) = TileWidth(t, tiles);
        f.Note = $"{t.Name}: {why}";
        var sum = new Dictionary<ulong, (long bytes, int tiles)>();
        void Add(FidelityMap.CellKey c, long b, int n)
        {
            var k = Pack(Cap(c));
            var (sb, st) = sum.GetValueOrDefault(k);
            sum[k] = (sb + b, st + n);
        }
        int unplaced = 0;
        long unplacedBytes = 0;
        foreach (var x in tiles)
        {
            if (t.Cube)
            {
                int level = (int)Math.Log2(t.FaceDim) - 7 - x.Mip;
                if (x.Face > 5 || level < 0 || level > FidelityMap.MaxLevel) { unplaced++; unplacedBytes += x.Bytes; continue; }
                if (texW == 128) Add(new FidelityMap.CellKey((byte)x.Face, (byte)level, x.X, x.Y), x.Bytes, 1);
                else
                {
                    Add(new FidelityMap.CellKey((byte)x.Face, (byte)level, x.X * 2, x.Y), x.Bytes / 2, 1);
                    Add(new FidelityMap.CellKey((byte)x.Face, (byte)level, x.X * 2 + 1, x.Y), x.Bytes - x.Bytes / 2, 0);
                }
            }
            else
            {
                double world = (double)((1L << t.Zoom) * 256L >> x.Mip);
                double X = ((t.OrgX >> x.Mip) + (double)x.X * texW + texW * 0.5) / world;
                double Y = ((t.OrgY >> x.Mip) + (double)x.Y * 128 + 64) / world;
                double lat = Math.Atan(Math.Sinh(Math.PI * (1 - 2 * Y))) / Geo.DegToRad, lon = (X - 0.5) * 360.0;
                double edge = 128 * 40075016.686 / world * Math.Cos(lat * Geo.DegToRad);
                Add(FidelityMap.CellOf(lat, lon, Math.Min(MaxLevel, Earth.LevelForEdge(edge))), x.Bytes, 1);
            }
        }
        // a window tile's cell is the one holding its centre: derived; a cube tile's is exact
        foreach (var (k, (b, n)) in sum) f.Entries.Add(new Entry(source, param, store, t.Name, k, b, n, Derived: !t.Cube || texW == 256));
        if (unplaced > 0) f.Unmapped.Add(new Unmapped(source, param, store, unplacedBytes, unplaced, "a tile off the lattice"));
    }

    // ------------------------------------------------------------------ composites

    static readonly Regex Corner = new(@"([ns])(\d+(?:x\d+)?)_([ew])(\d+(?:x\d+)?)", RegexOptions.Compiled | RegexOptions.IgnoreCase);

    static double Coord(string s) => double.Parse(s.Replace('x', '.'), CultureInfo.InvariantCulture);

    Folder ReadComposite(FileInfo doc, string product)
    {
        var f = new Folder();
        var j = JsonNode.Parse(File.ReadAllText(doc.FullName)) ?? throw new InvalidDataException(doc.FullName);
        string layer = j["layer"]?.GetValue<string>() ?? doc.Name.Replace(".composite.json", "");
        string dir = doc.DirectoryName!;
        var sum = new Dictionary<(string src, ulong cell, bool derived), (long bytes, int n)>();
        void Add(string src, FidelityMap.CellKey c, long b, int n, bool derived)
        {
            var k = (src, Pack(Cap(c)), derived);
            var (sb, sn) = sum.GetValueOrDefault(k);
            sum[k] = (sb + b, sn + n);
        }
        // a raster tier: its tiles by size, each to the cell of its centre at the level of its size
        if (j["tiers"] is JsonArray tiers)
            foreach (var tier in tiers.OfType<JsonObject>())
            {
                var folder = Path.Combine(dir, tier["folder"]?.GetValue<string>() ?? "");
                if (!Directory.Exists(folder)) continue;
                string src = tier["inputs"] is JsonArray ins && ins.Count > 0 ? string.Join("+", ins.Select(i => i!.GetValue<string>())) : "?";
                var box = tier["box"] is JsonArray bx && bx.Count == 4 ? bx.Select(v => v!.GetValue<double>()).ToArray() : null;
                var files = new DirectoryInfo(folder).GetFiles(tier["match"]?.GetValue<string>() ?? "*.tif");
                var corners = files.Select(fi => (fi, m: Corner.Match(fi.Name))).ToList();
                var lats = corners.Where(c => c.m.Success).Select(c => (c.m.Groups[1].Value.ToLowerInvariant() == "s" ? -1 : 1) * Coord(c.m.Groups[2].Value)).Distinct().OrderBy(v => v).ToList();
                var lons = corners.Where(c => c.m.Success).Select(c => (c.m.Groups[3].Value.ToLowerInvariant() == "w" ? -1 : 1) * Coord(c.m.Groups[4].Value)).Distinct().OrderBy(v => v).ToList();
                // the tile's size: the least spacing of the corners' grid, DERIVED from the names;
                // a part of one tile takes its tier box
                double Step(List<double> v, double whole) => v.Count > 1 ? v.Zip(v.Skip(1), (a, b) => b - a).Where(d => d > 1e-9).DefaultIfEmpty(whole).Min() : whole;
                double dLat = Step(lats, box is null ? 1 : box[3] - box[1]), dLon = Step(lons, box is null ? 1 : box[2] - box[0]);
                foreach (var (fi, m) in corners)
                {
                    double lat, lon;
                    if (m.Success)
                    {
                        lat = (m.Groups[1].Value.ToLowerInvariant() == "s" ? -1 : 1) * Coord(m.Groups[2].Value) - dLat / 2;
                        lon = (m.Groups[3].Value.ToLowerInvariant() == "w" ? -1 : 1) * Coord(m.Groups[4].Value) + dLon / 2;
                    }
                    else if (box is not null) { lat = 0.5 * (box[1] + box[3]); lon = 0.5 * (box[0] + box[2]); }
                    else { f.Unmapped.Add(new Unmapped(src, layer, "composite", fi.Length, 1, "a tile with no corner in its name and no tier box")); continue; }
                    double edge = Math.Min(dLat, dLon * Math.Cos(lat * Geo.DegToRad)) * Earth.PlanetRadiusM * Geo.DegToRad;
                    Add(src, FidelityMap.CellOf(lat, lon, Math.Min(MaxLevel, Earth.LevelForEdge(edge))), fi.Length, 1, derived: true);
                }
            }
        // a point layer: the file's bytes over its points, evenly (DERIVED: a JSON file has no
        // per-point size); a referenced file with a box (a current field) to its box's centre
        if (j["file"]?.GetValue<string>() is string pf && File.Exists(Path.Combine(dir, pf)))
        {
            var path = Path.Combine(dir, pf);
            long size = new FileInfo(path).Length;
            var pts = new List<(double lat, double lon, string src)>();
            var boxed = new List<(string file, double[] box, string src)>();
            void Walk(JsonNode? n, string src)
            {
                if (n is JsonObject o)
                {
                    src = o["source"] is JsonValue sv && sv.TryGetValue(out string? s) ? s : o["model"] is JsonValue mv && mv.TryGetValue(out string? ms) ? ms : src;
                    if (o["lat"] is JsonValue la && la.TryGetValue(out double lat) && o["lon"] is JsonValue lo && lo.TryGetValue(out double lon)) pts.Add((lat, lon, src));
                    if (o["file"] is JsonValue fv && fv.TryGetValue(out string? file) && o["box"] is JsonArray b && b.Count == 4)
                        boxed.Add((file, b.Select(v => v!.GetValue<double>()).ToArray(), src));
                    foreach (var (_, v) in o) Walk(v, src);
                }
                else if (n is JsonArray a) foreach (var v in a) Walk(v, src);
            }
            Walk(JsonNode.Parse(File.ReadAllText(path)), layer);
            if (pts.Count > 0)
            {
                long share = size / pts.Count, rest = size - share * pts.Count;
                int i = 0;
                foreach (var (lat, lon, src) in pts) Add(src, FidelityMap.CellOf(lat, lon, MaxLevel), share + (i++ == 0 ? rest : 0), 1, derived: true);
            }
            else f.Unmapped.Add(new Unmapped(layer, layer, "composite", size, 1, "a point file with no lat/lon"));
            foreach (var (file, b, src) in boxed)
            {
                var fp = Path.Combine(Path.GetDirectoryName(path)!, file);
                if (!File.Exists(fp)) fp = Path.Combine(dir, file);
                if (!File.Exists(fp)) continue;
                double lat = 0.5 * (b[1] + b[3]), lon = 0.5 * (b[0] + b[2]);
                double edge = Math.Min(b[3] - b[1], (b[2] - b[0]) * Math.Cos(lat * Geo.DegToRad)) * Earth.PlanetRadiusM * Geo.DegToRad;
                Add(src, FidelityMap.CellOf(lat, lon, Math.Min(MaxLevel, Earth.LevelForEdge(edge))), new FileInfo(fp).Length, 1, derived: true);
            }
        }
        foreach (var ((src, cell, derived), (b, n)) in sum) f.Entries.Add(new Entry(src, layer, "composite:" + product, "lattice", cell, b, n, derived));
        // coverage[]: who answers where, as written
        if (j["coverage"] is JsonArray cov)
            foreach (var c in cov.OfType<JsonObject>())
                if (c["box"] is JsonArray b && b.Count == 4)
                    f.Coverage.Add(new Coverage(product, layer, c["input"]?.GetValue<string>() ?? "?", b[0]!.GetValue<double>(), b[1]!.GetValue<double>(),
                        b[2]!.GetValue<double>(), b[3]!.GetValue<double>(), c["grain_m"]?.GetValue<double>() ?? double.NaN, c["gap_fill"]?.GetValue<bool>() ?? false));
        return f;
    }

    // ------------------------------------------------------------------ the file

    /// The default index file: one per machine, beside the demo's settings.
    public static string DefaultPath => Path.Combine(Environment.GetFolderPath(Environment.SpecialFolder.LocalApplicationData), "GAGAME", "storage-index.bin");

    public void Save(string path)
    {
        Directory.CreateDirectory(Path.GetDirectoryName(Path.GetFullPath(path))!);
        var strings = new Dictionary<string, int>();
        int S(string s) { if (!strings.TryGetValue(s, out int i)) strings[s] = i = strings.Count; return i; }
        foreach (var f in Folders.Values)
        {
            S(f.Path); S(f.Note);
            foreach (var e in f.Entries) { S(e.Source); S(e.Param); S(e.Store); S(e.Lattice); }
            foreach (var u in f.Unmapped) { S(u.Source); S(u.Param); S(u.Store); S(u.Why); }
            foreach (var c in f.Coverage) { S(c.Product); S(c.Layer); S(c.Input); }
        }
        var tmp = path + ".tmp";
        using (var w = new BinaryWriter(File.Create(tmp), Encoding.UTF8))
        {
            w.Write(Encoding.ASCII.GetBytes(Magic));
            w.Write(Root);
            w.Write(strings.Count);
            foreach (var s in strings.OrderBy(kv => kv.Value)) w.Write(s.Key);
            w.Write(Folders.Count);
            foreach (var f in Folders.Values)
            {
                w.Write(S(f.Path)); w.Write(f.Stamp); w.Write(S(f.Note)); w.Write(f.Voids);
                w.Write(f.Entries.Count);
                foreach (var e in f.Entries) { w.Write(S(e.Source)); w.Write(S(e.Param)); w.Write(S(e.Store)); w.Write(S(e.Lattice)); w.Write(e.Cell); w.Write(e.Bytes); w.Write(e.Tiles); w.Write(e.Derived); }
                w.Write(f.Unmapped.Count);
                foreach (var u in f.Unmapped) { w.Write(S(u.Source)); w.Write(S(u.Param)); w.Write(S(u.Store)); w.Write(u.Bytes); w.Write(u.Files); w.Write(S(u.Why)); }
                w.Write(f.Coverage.Count);
                foreach (var c in f.Coverage) { w.Write(S(c.Product)); w.Write(S(c.Layer)); w.Write(S(c.Input)); w.Write(c.Lon0); w.Write(c.Lat0); w.Write(c.Lon1); w.Write(c.Lat1); w.Write(c.GrainM); w.Write(c.GapFill); }
            }
        }
        File.Move(tmp, path, overwrite: true);
    }

    /// The index at `path`, or an empty one when it is missing, of another format or another root.
    public static StorageIndex Load(string path, string root)
    {
        var x = new StorageIndex { Root = Path.GetFullPath(root) };
        if (!File.Exists(path)) return x;
        try
        {
            using var r = new BinaryReader(File.OpenRead(path), Encoding.UTF8);
            if (Encoding.ASCII.GetString(r.ReadBytes(8)) != Magic) return x;
            if (!string.Equals(r.ReadString(), x.Root, StringComparison.OrdinalIgnoreCase)) return x;
            var s = new string[r.ReadInt32()];
            for (int i = 0; i < s.Length; ++i) s[i] = r.ReadString();
            int nf = r.ReadInt32();
            for (int i = 0; i < nf; ++i)
            {
                var f = new Folder { Path = s[r.ReadInt32()], Stamp = r.ReadInt64(), Note = s[r.ReadInt32()], Voids = r.ReadInt32() };
                int ne = r.ReadInt32();
                for (int k = 0; k < ne; ++k) f.Entries.Add(new Entry(s[r.ReadInt32()], s[r.ReadInt32()], s[r.ReadInt32()], s[r.ReadInt32()], r.ReadUInt64(), r.ReadInt64(), r.ReadInt32(), r.ReadBoolean()));
                int nu = r.ReadInt32();
                for (int k = 0; k < nu; ++k) f.Unmapped.Add(new Unmapped(s[r.ReadInt32()], s[r.ReadInt32()], s[r.ReadInt32()], r.ReadInt64(), r.ReadInt32(), s[r.ReadInt32()]));
                int nc = r.ReadInt32();
                for (int k = 0; k < nc; ++k) f.Coverage.Add(new Coverage(s[r.ReadInt32()], s[r.ReadInt32()], s[r.ReadInt32()], r.ReadDouble(), r.ReadDouble(), r.ReadDouble(), r.ReadDouble(), r.ReadDouble(), r.ReadBoolean()));
                x.Folders[f.Path] = f;
            }
        }
        catch (Exception e) when (e is IOException or EndOfStreamException or IndexOutOfRangeException)
        {
            x.Folders.Clear();
        }
        return x;
    }
}
