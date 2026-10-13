using System.Globalization;
using System.Security.Cryptography;
using System.Text.RegularExpressions;
using Gagame.Harvest.Readers;
using static Gagame.Harvest.JsonUtil;

namespace Gagame.Harvest.Harvesters;

/// OSM buildings from a .osm.pbf, as SOLIDS (harvest_buildings.py): every footprint's rings with
/// the heights OSM tagged, normalized to metres, refused (NaN) when they do not parse, never
/// guessed. Output GABLDG01: records sorted by 0.05-degree cell, a JSON manifest with the index.
public sealed class Buildings : IHarvester
{
    public string Name => "buildings";
    public string Title => "Buildings (OSM solids)";
    public string What => "Footprints with the heights OSM tagged, as prisms, from a .osm.pbf extract.";
    public string[] Outputs => [];
    public Source[] Sources => [new("Geofabrik extracts", "https://download.geofabrik.de/"), new("OSM copyright", "https://www.openstreetmap.org/copyright")];
    public Insert? Insert => new(InsertKind.FileArg, "Pick the .osm.pbf extract and the box (lon0,lat0,lon1,lat1) to keep.", Pattern: "OSM extracts (*.osm.pbf)|*.osm.pbf|All files|*.*");
    public string? Note => "No default download: pick an extract with Insert.";

    public const double CellDeg = 0.05;
    const double Ft = 0.3048, In = 0.0254;
    const string NumRe = @"[-+]?\d+(?:[.,]\d+)?";
    public static readonly string[] Keep = ["building", "building:part", "height", "min_height", "building:levels", "building:min_level", "roof:shape", "roof:height", "type"];

    /// A length tag in metres, or null when it does not parse (refused, not guessed).
    public static double? Metres(string text)
    {
        var t = text.Trim().ToLowerInvariant().Replace("~", "");
        var m = Regex.Match(t, $@"^({NumRe})\s*(m|meters?|metres?)?$");
        if (m.Success) return double.Parse(m.Groups[1].Value.Replace(',', '.'), CultureInfo.InvariantCulture);
        m = Regex.Match(t, $@"^({NumRe})\s*(ft|feet|foot|')$");
        if (m.Success) return double.Parse(m.Groups[1].Value.Replace(',', '.'), CultureInfo.InvariantCulture) * Ft;
        m = Regex.Match(t, $@"^({NumRe})\s*'\s*({NumRe})\s*(""|in)?$");
        if (m.Success) return double.Parse(m.Groups[1].Value, CultureInfo.InvariantCulture) * Ft + double.Parse(m.Groups[2].Value, CultureInfo.InvariantCulture) * In;
        return null;
    }

    public static double? Count(string text)
    {
        var t = text.Trim().Replace(',', '.');
        if (!double.TryParse(t, NumberStyles.Float, CultureInfo.InvariantCulture, out var v)) return null;
        return v >= 0 ? v : null;
    }

    public static Dictionary<string, string> TagsOf(OsmPbf.Block b, int[] keys, int[] vals)
    {
        var d = new Dictionary<string, string>();
        for (var i = 0; i < keys.Length; i++)
        {
            var k = b.Strings[keys[i]];
            if (Keep.Contains(k)) d[k] = b.Strings[vals[i]];
        }
        return d;
    }

    /// 0 building, 1 building:part, null neither.
    public static int? IsBuilding(Dictionary<string, string> t)
    {
        if (t.TryGetValue("building", out var b) && b != "no") return 0;
        if (t.TryGetValue("building:part", out var p) && p != "no") return 1;
        return null;
    }

    /// Join member ways (node id arrays) into closed rings by shared ends. Unclosable pieces drop.
    public static List<List<long>> Stitch(List<long[]> ways)
    {
        var rings = new List<List<long>>();
        var open = new List<List<long>>();
        foreach (var w in ways)
        {
            var l = w.ToList();
            if (w.Length >= 4 && w[0] == w[^1]) rings.Add(l); else open.Add(l);
        }
        while (open.Count > 0)
        {
            var cur = open[^1];
            open.RemoveAt(open.Count - 1);
            var grown = true;
            while (cur[0] != cur[^1] && grown)
            {
                grown = false;
                for (var i = 0; i < open.Count; i++)
                {
                    var w = open[i];
                    if (w[0] == cur[^1]) cur.AddRange(w.Skip(1));
                    else if (w[^1] == cur[^1]) cur.AddRange(w.Take(w.Count - 1).Reverse());
                    else if (w[^1] == cur[0]) cur = w.Take(w.Count - 1).Concat(cur).ToList();
                    else if (w[0] == cur[0]) cur = w.Skip(1).Reverse().Concat(cur).ToList();
                    else continue;
                    open.RemoveAt(i);
                    grown = true;
                    break;
                }
            }
            if (cur[0] == cur[^1] && cur.Count >= 4) rings.Add(cur);
        }
        return rings;
    }

    public readonly record struct Attrs(int Roof, float H, float Mh, float Lv, float Mlv, float Rh);

    /// A building's normalized tags: lengths in metres, counts as counts, NaN where untagged or refused.
    public static Attrs AttrsOf(Dictionary<string, string> t, Dictionary<string, int> refused, List<string> roofShapes, bool grow = true)
    {
        float Length(string k)
        {
            if (!t.TryGetValue(k, out var s)) return float.NaN;
            var v = Metres(s);
            if (v is null) { refused[k] = refused.GetValueOrDefault(k) + 1; return float.NaN; }
            return (float)v.Value;
        }
        float Levels(string k)
        {
            if (!t.TryGetValue(k, out var s)) return float.NaN;
            var v = Count(s);
            if (v is null) { refused[k] = refused.GetValueOrDefault(k) + 1; return float.NaN; }
            return (float)v.Value;
        }
        var roof = 255;
        if (t.TryGetValue("roof:shape", out var rs))
        {
            var i = roofShapes.IndexOf(rs);
            if (i >= 0) roof = i;
            else if (grow) { roofShapes.Add(rs); roof = roofShapes.Count - 1; }
            else roof = 254;
        }
        return new Attrs(roof, Length("height"), Length("min_height"), Levels("building:levels"), Levels("building:min_level"), Length("roof:height"));
    }

    public static (int iy, int ix) CellOf(long lonE7, long latE7) =>
        ((int)Math.Floor(latE7 * 1e-7 / CellDeg), (int)Math.Floor(lonE7 * 1e-7 / CellDeg));

    public sealed record Ring(byte Outer, int[] Xy);   // xy interleaved lon, lat in 1e-7 degree, closing node dropped
    public sealed record Rec((int iy, int ix) Cell, long Id, int Kind, Attrs A, List<Ring> Geo);

    public static byte[] Record(long sid, int kind, Attrs a, List<Ring> geo)
    {
        using var ms = new MemoryStream();
        Bin.I64(ms, sid);
        ms.WriteByte((byte)kind);
        ms.WriteByte((byte)a.Roof);
        Bin.U16(ms, (ushort)geo.Count);
        Bin.F32(ms, a.H); Bin.F32(ms, a.Mh); Bin.F32(ms, a.Lv); Bin.F32(ms, a.Mlv); Bin.F32(ms, a.Rh);
        foreach (var r in geo)
        {
            Bin.U32(ms, (uint)(r.Xy.Length / 2));
            ms.WriteByte(r.Outer);
            ms.Write(new byte[3]);
            ms.Write(System.Runtime.InteropServices.MemoryMarshal.AsBytes<int>(r.Xy));
        }
        return ms.ToArray();
    }

    /// recs -> .bin + .json manifest, records sorted by cell then id.
    public static (string binPath, Dictionary<string, int> tagged) WriteGabldg(string outDir, string stem, List<Rec> recs, object source, string licence, string attribution,
        object? bbox, Dictionary<string, int> refused, Dictionary<string, int> skipped, List<string> roofShapes, string coordinates = "int32, 1e-7 degree, WGS84 lon/lat (OSM's own precision)")
    {
        Directory.CreateDirectory(outDir);
        recs.Sort((x, y) => x.Cell.iy != y.Cell.iy ? x.Cell.iy.CompareTo(y.Cell.iy) : x.Cell.ix != y.Cell.ix ? x.Cell.ix.CompareTo(y.Cell.ix) : x.Id.CompareTo(y.Id));
        var binPath = Path.Combine(outDir, stem + ".buildings.bin");
        var index = new List<List<long>>();
        var tagged = new Dictionary<string, int> { ["height"] = 0, ["min_height"] = 0, ["building:levels"] = 0, ["roof:shape"] = 0 };
        using (var f = File.Create(binPath))
        {
            f.Write("GABLDG01"u8);
            (int, int)? last = null;
            foreach (var r in recs)
            {
                if (last is null || r.Cell != last.Value)
                {
                    index.Add(new List<long> { r.Cell.ix, r.Cell.iy, f.Position, 0 });
                    last = r.Cell;
                }
                index[^1][3]++;
                if (!float.IsNaN(r.A.H)) tagged["height"]++;
                if (!float.IsNaN(r.A.Mh)) tagged["min_height"]++;
                if (!float.IsNaN(r.A.Lv)) tagged["building:levels"]++;
                if (r.A.Roof != 255) tagged["roof:shape"]++;
                f.Write(Record(r.Id, r.Kind, r.A, r.Geo));
            }
        }
        var manifest = D(("format", "GABLDG01"), ("source", source), ("licence", licence), ("attribution", attribution),
            ("units", D(("coordinates", coordinates), ("height", "metres above the ground at the footprint"), ("levels", "count"))),
            ("datum", "ground"), ("bbox", bbox),
            ("counts", D(("records", recs.Count), ("buildings", recs.Count(r => r.Kind == 0)), ("parts", recs.Count(r => r.Kind == 1)), ("tagged", tagged))),
            ("refused", refused), ("skipped", skipped), ("roofShapes", roofShapes), ("cellDeg", CellDeg), ("cells", index));
        Save(Path.Combine(outDir, stem + ".buildings.json"), manifest);
        return (binPath, tagged);
    }

    public static Dictionary<string, int> Refused() => new() { ["height"] = 0, ["min_height"] = 0, ["roof:height"] = 0, ["building:levels"] = 0, ["building:min_level"] = 0 };

    public int Run(HarvestContext ctx, string[] argv)
    {
        var a = new Args(argv);
        var pos = a.Positionals("--out", "--bbox");
        if (pos.Count == 0) throw new HarvestStop("usage: harvest buildings <file.osm.pbf> [--out DIR] [--bbox lon0,lat0,lon1,lat1]");
        var pbf = pos[0];
        // --bbox as four values (the Python's nargs=4) or one comma-joined value (the demo's dialog)
        double[]? bbox = null;
        var bi = Array.IndexOf(argv, "--bbox");
        if (bi >= 0)
        {
            if (bi + 4 < argv.Length && !argv[bi + 1].Contains(',')) bbox = argv.Skip(bi + 1).Take(4).Select(s => double.Parse(s, CultureInfo.InvariantCulture)).ToArray();
            else bbox = Args.Pair(argv[bi + 1]);
        }
        var t0 = ctx.Seconds;
        var outDir = a.Opt("--out") ?? Path.Combine(Path.GetDirectoryName(Path.GetFullPath(pbf))!, "buildings");
        Directory.CreateDirectory(outDir);
        var stem = Path.GetFileName(pbf).Split('.')[0];
        var refused = Refused();
        var skipped = new Dictionary<string, int> { ["open_way"] = 0, ["missing_node"] = 0, ["relation_no_ring"] = 0, ["outside_bbox"] = 0 };
        var roofShapes = new List<string>();

        // pass 1
        var solids = new List<(long id, int kind, Attrs a, List<(byte outer, long[] refs)> rings)>();
        var memberRoles = new Dictionary<long, List<(int si, bool outer)>>();
        var wayBlocks = new List<long>();
        var denseBlocks = new List<long>();
        var hdr = OsmPbf.Header(pbf);
        foreach (var (off, btype, raw) in OsmPbf.Blobs(pbf))
        {
            if (btype != "OSMData") continue;
            var b = new OsmPbf.Block(raw);
            var kinds = b.Kinds().ToHashSet();
            if (kinds.Contains("dense") || kinds.Contains("nodes")) denseBlocks.Add(off);
            if (kinds.Contains("ways"))
            {
                wayBlocks.Add(off);
                var bk = new HashSet<int> { b.Index.GetValueOrDefault("building", -1), b.Index.GetValueOrDefault("building:part", -1) };
                foreach (var w in b.Ways())
                {
                    if (!w.Keys.Any(bk.Contains)) continue;
                    var t = TagsOf(b, w.Keys, w.Vals);
                    var kind = IsBuilding(t);
                    if (kind is null) continue;
                    var r = OsmPbf.PackedDelta(w.Refs);
                    if (r.Length < 4 || r[0] != r[^1]) { skipped["open_way"]++; continue; }
                    solids.Add((w.Id, kind.Value, AttrsOf(t, refused, roofShapes), new() { (1, r) }));
                }
            }
            if (kinds.Contains("relations"))
                foreach (var rel in b.Relations())
                {
                    var t = TagsOf(b, rel.Keys, rel.Vals);
                    var kind = IsBuilding(t);
                    if (kind is null || t.GetValueOrDefault("type") != "multipolygon") continue;
                    var si = solids.Count;
                    solids.Add((-rel.Id, kind.Value, AttrsOf(t, refused, roofShapes), new()));
                    for (var i = 0; i < rel.Members.Length; i++)
                        if (rel.Types[i] == 1)
                        {
                            if (!memberRoles.TryGetValue(rel.Members[i], out var l)) memberRoles[rel.Members[i]] = l = new();
                            l.Add((si, b.Strings[(int)rel.Roles[i]] != "inner"));
                        }
                }
        }
        ctx.Log($"[buildings] pass 1: {solids.Count:N0} building records ({memberRoles.Count:N0} relation member ways) in {ctx.Seconds - t0:0} s");

        // pass 2
        if (memberRoles.Count > 0)
        {
            var pieces = new Dictionary<(int si, bool outer), List<long[]>>();
            foreach (var off in wayBlocks)
            {
                var b = new OsmPbf.Block(OsmPbf.BlobAt(pbf, off));
                foreach (var w in b.Ways())
                    if (memberRoles.TryGetValue(w.Id, out var roles))
                        foreach (var (si, outer) in roles)
                        {
                            if (!pieces.TryGetValue((si, outer), out var l)) pieces[(si, outer)] = l = new();
                            l.Add(OsmPbf.PackedDelta(w.Refs));
                        }
            }
            foreach (var ((si, outer), ws) in pieces)
                foreach (var ring in Stitch(ws))
                    solids[si].rings.Add(((byte)(outer ? 1 : 0), ring.ToArray()));
            var before = solids.Count;
            solids = solids.Where(s => s.rings.Any(r => r.outer == 1)).ToList();
            skipped["relation_no_ring"] = before - solids.Count;
            ctx.Log($"[buildings] pass 2: member ways stitched in {ctx.Seconds - t0:0} s");
        }

        // pass 3
        var need = solids.SelectMany(s => s.rings.SelectMany(r => r.refs)).Distinct().OrderBy(x => x).ToArray();
        var lat = new long[need.Length];
        var lon = new long[need.Length];
        Array.Fill(lat, long.MinValue);
        foreach (var off in denseBlocks)
        {
            var b = new OsmPbf.Block(OsmPbf.BlobAt(pbf, off));
            foreach (var (ids, la, lo) in b.Dense())
                for (var i = 0; i < ids.Length; i++)
                {
                    var p = Array.BinarySearch(need, ids[i]);
                    if (p >= 0) { lat[p] = la[i]; lon[p] = lo[i]; }
                }
            foreach (var (nid, la, lo) in b.Nodes())
            {
                var p = Array.BinarySearch(need, nid);
                if (p >= 0) { lat[p] = la; lon[p] = lo; }
            }
        }
        ctx.Log($"[buildings] pass 3: {need.Length:N0} nodes placed in {ctx.Seconds - t0:0} s");

        var recs = new List<Rec>();
        foreach (var (sid, kind, attrs, rings) in solids)
        {
            var geo = new List<Ring>();
            var ok = true;
            foreach (var (outer, r) in rings)
            {
                var xy = new int[(r.Length - 1) * 2];
                for (var i = 0; i < r.Length - 1; i++)
                {
                    var p = Array.BinarySearch(need, r[i]);
                    if (p < 0 || lat[p] == long.MinValue) { ok = false; break; }
                    xy[2 * i] = (int)lon[p];
                    xy[2 * i + 1] = (int)lat[p];
                }
                if (!ok) break;
                geo.Add(new Ring(outer, xy));
            }
            if (!ok) { skipped["missing_node"]++; continue; }
            int x0 = geo[0].Xy[0], y0 = geo[0].Xy[1];
            if (bbox is not null && !(bbox[0] <= x0 * 1e-7 && x0 * 1e-7 <= bbox[2] && bbox[1] <= y0 * 1e-7 && y0 * 1e-7 <= bbox[3])) { skipped["outside_bbox"]++; continue; }
            recs.Add(new Rec(CellOf(x0, y0), sid, kind, attrs, geo));
        }
        string md5;
        using (var f = File.OpenRead(pbf)) md5 = Convert.ToHexStringLower(MD5.HashData(f));
        var source = D(("file", Path.GetFileName(pbf)), ("bytes", new FileInfo(pbf).Length), ("md5", md5), ("header", hdr));
        var (binPath, tagged) = WriteGabldg(outDir, stem, recs, source, "ODbL 1.0", "(c) OpenStreetMap contributors", bbox?.ToList(), refused, skipped, roofShapes);
        ctx.Log($"[buildings] {recs.Count:N0} solids -> {binPath} ({new FileInfo(binPath).Length / 1048576.0:0} MB) in {ctx.Seconds - t0:0} s");
        ctx.Log($"[buildings] tagged {Dump(tagged, 0)}; refused {Dump(refused, 0)}; skipped {Dump(skipped, 0)}");
        return 0;
    }
}
