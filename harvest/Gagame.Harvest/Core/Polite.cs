using System.Globalization;
using System.Net;
using System.Net.Http.Headers;
using System.Text;
using System.Text.Json;
using System.Text.RegularExpressions;

namespace Gagame.Harvest;

/// The polite fetcher for PLACE harvests (harvester/polite.py): cache first forever, one
/// request in flight across processes (a lock file), the least gap between requests to a
/// SOURCE (all its hosts paced as one) and the bans from the sources' limits file
/// (harvest/sources.limits.json, docs/FIDELITY.md section 2: 1.1 s by default, 2.5 s on NCEP's
/// hosts, no Google host), one retry after Retry-After or the file's back-off, a persistent
/// byte and request budget charged BEFORE each request (and a source's own day of bytes where
/// it states one), planned streaming downloads that resume from .part, and the product token
/// without the contact address.
public sealed class Polite : Fetcher
{
    public const string UA = "GAGAME/0.1 (hobby ocean simulator)";

    /// The sources' limits this fetcher obeys (read once, at construction).
    public SourceLimits Limits { get; }
    long MaxBytes => (long)Limits.BudgetBytes;
    long MaxRequests => Limits.BudgetRequests;

    public sealed class StopException : Exception
    {
        public StopException(string message, Exception? inner = null) : base(message, inner) { }
    }

    readonly HarvestContext _ctx;
    public string LedgerPath { get; }
    public string? Place { get; set; }
    readonly Ledger _ledger;

    public Polite(HarvestContext ctx)
    {
        _ctx = ctx;
        LedgerPath = Environment.GetEnvironmentVariable("GAGAME_LEDGER") ?? Path.Combine("out", "places", "ledger.jsonl");
        Limits = SourceLimits.Load(ctx.Root);
        _ledger = new Ledger(LedgerPath);
    }

    // ------------------------------------------------------------------ ledger

    sealed class Ledger
    {
        public readonly string Path;
        public long Bytes;
        public int Requests;
        public readonly Dictionary<string, long[]> PerHost = new();
        public readonly Dictionary<string, double> LastAt = new();
        public readonly List<(string host, double tEnd, long bytes)> Lines = new();
        public (long bytes, string url) Largest = (0, "");

        public Ledger(string path)
        {
            Path = path;
            if (!File.Exists(path)) return;
            foreach (var ln in File.ReadLines(path, Encoding.UTF8))
            {
                if (ln.Trim().Length == 0) continue;
                using var d = JsonDocument.Parse(ln);
                Add(d.RootElement);
            }
        }

        void Add(JsonElement e)
        {
            var n = e.TryGetProperty("bytes", out var b) && b.ValueKind == JsonValueKind.Number ? b.GetInt64() : 0;
            var host = e.GetProperty("host").GetString() ?? "?";
            Bytes += n;
            Requests++;
            if (!PerHost.TryGetValue(host, out var h)) PerHost[host] = h = new long[2];
            h[0]++;
            h[1] += n;
            var tEnd = e.TryGetProperty("t_end", out var t) && t.ValueKind == JsonValueKind.Number ? t.GetDouble() : 0.0;
            LastAt[host] = Math.Max(LastAt.GetValueOrDefault(host), tEnd);
            Lines.Add((host, tEnd, n));
            if (n > Largest.bytes) Largest = (n, e.GetProperty("url").GetString() ?? "");
        }

        public void Charge(Dictionary<string, object?> entry)
        {
            Directory.CreateDirectory(System.IO.Path.GetDirectoryName(System.IO.Path.GetFullPath(Path))!);
            var line = JsonSerializer.Serialize(entry);
            File.AppendAllText(Path, line + "\n", Encoding.UTF8);
            using var d = JsonDocument.Parse(line);
            Add(d.RootElement);
        }
    }

    public (long bytes, int requests) Remaining() => (MaxBytes - _ledger.Bytes, (int)(MaxRequests - _ledger.Requests));

    public Dictionary<string, object?> Summary() => new()
    {
        ["requests"] = _ledger.Requests,
        ["bytes"] = _ledger.Bytes,
        ["per_host"] = _ledger.PerHost.OrderBy(k => k.Key, StringComparer.Ordinal)
            .ToDictionary(k => k.Key, k => (object?)new Dictionary<string, long> { ["requests"] = k.Value[0], ["bytes"] = k.Value[1] }),
        ["largest"] = new Dictionary<string, object?> { ["bytes"] = _ledger.Largest.bytes, ["url"] = _ledger.Largest.url },
        ["budget"] = new Dictionary<string, long> { ["bytes"] = MaxBytes, ["requests"] = MaxRequests },
    };

    /// Every ledger line, parsed (the manifest reads them).
    public List<System.Text.Json.Nodes.JsonNode> Entries()
    {
        var out_ = new List<System.Text.Json.Nodes.JsonNode>();
        if (!File.Exists(LedgerPath)) return out_;
        foreach (var ln in File.ReadLines(LedgerPath, Encoding.UTF8))
            if (ln.Trim().Length > 0 && System.Text.Json.Nodes.JsonNode.Parse(ln) is { } n) out_.Add(n);
        return out_;
    }

    // ------------------------------------------------------------------ lock

    sealed class OneInFlight : IDisposable
    {
        readonly FileStream _f;
        public OneInFlight(string ledgerPath)
        {
            var path = Path.GetFullPath(ledgerPath) + ".lock";
            Directory.CreateDirectory(Path.GetDirectoryName(path)!);
            while (true)
            {
                try
                {
                    _f = new FileStream(path, FileMode.OpenOrCreate, FileAccess.ReadWrite, FileShare.None);
                    break;
                }
                catch (IOException) { Thread.Sleep(250); }
            }
        }
        public void Dispose() => _f.Dispose();
    }

    // ------------------------------------------------------------------ request

    public static string HostOf(string url) => Uri.TryCreate(url, UriKind.Absolute, out var u) ? u.Host : "?";

    /// A host any banned source names is refused, however specific another entry naming it is.
    void CheckHost(string url)
    {
        var h = HostOf(url);
        if (Limits.BannedBy(h) is { } s) throw new StopException($"refusing a banned host: {h} ({s.Title}; {Limits.Path})");
    }

    static double Now() => DateTimeOffset.UtcNow.ToUnixTimeMilliseconds() / 1000.0;

    /// The last request to any host of the same source, so a source's hosts share one pace.
    double LastAtSource(string host)
    {
        var key = Limits.KeyOf(host);
        return _ledger.LastAt.Where(kv => Limits.KeyOf(kv.Key) == key).Select(kv => kv.Value).DefaultIfEmpty(0.0).Max();
    }

    void Pace(string host)
    {
        var wait = Limits.GapOf(host) - (Now() - LastAtSource(host));
        if (wait > 0) Thread.Sleep(TimeSpan.FromSeconds(wait));
    }

    void Admit(string url, long? expectBytes)
    {
        if (Limits.Of(HostOf(url)) is { DayBytes: > 0 } src)
        {
            var since = Now() - 86400.0;
            var day = _ledger.Lines.Where(l => l.tEnd > since && Limits.KeyOf(l.host) == src.Id).Sum(l => l.bytes);
            if (day + (expectBytes ?? 0) > src.DayBytes)
                throw new StopException($"{src.Title}: {day / 1e9:0.000} GB in the last 24 h + {(expectBytes ?? 0) / 1e9:0.000} GB planned > its {src.DayBytes / 1e9:0.0} GB a day; not requesting {url}");
        }
        if (_ledger.Requests + 1 > MaxRequests)
            throw new StopException($"request budget spent ({_ledger.Requests}/{MaxRequests}); not requesting {url}");
        if (_ledger.Bytes + (expectBytes ?? 0) > MaxBytes)
            throw new StopException($"byte budget: {_ledger.Bytes / 1e9:0.000} GB used + {(expectBytes ?? 0) / 1e9:0.000} GB planned > {MaxBytes / 1e9:0.0} GB; not requesting {url}");
    }

    static bool Retryable(Exception e) => e switch
    {
        HttpError h => h.Code == 429 || (h.Code >= 500 && h.Code <= 599),
        HttpRequestException or TaskCanceledException or IOException or OperationCanceledException => true,
        _ => false,
    };

    double RetryAfter(Exception e) =>
        e is HttpError { RetryAfter: double ra } ? Math.Min(Math.Max(ra, 1.0), 300.0) : Limits.BackoffS;

    static readonly HttpClient Http = Make();
    static HttpClient Make()
    {
        var h = new HttpClient(new HttpClientHandler { AutomaticDecompression = DecompressionMethods.None, AllowAutoRedirect = true })
        { Timeout = Timeout.InfiniteTimeSpan };
        h.DefaultRequestHeaders.UserAgent.ParseAdd("GAGAME/0.1");
        h.DefaultRequestHeaders.UserAgent.Add(new ProductInfoHeaderValue("(hobby ocean simulator)"));
        h.DefaultRequestHeaders.AcceptEncoding.ParseAdd("identity");
        return h;
    }

    sealed class Partial : Exception
    {
        public readonly Exception Inner;
        public readonly long NBytes;
        public Partial(Exception inner, long n) : base(inner.Message) { Inner = inner; NBytes = n; }
    }

    /// ONE request: admit, pace, lock, send, charge. `reader` consumes the body stream and
    /// returns the bytes it took; null means read the whole body (small GET).
    (int? status, Dictionary<string, string> headers, byte[]? body) Exchange(
        string url, string method, Dictionary<string, string>? headers, int timeoutS,
        Func<HttpResponseMessage, Stream, long>? reader, long? expectBytes, string note, string? dest)
    {
        CheckHost(url);
        var host = HostOf(url);
        Admit(url, expectBytes);
        using var _ = new OneInFlight(LedgerPath);
        Pace(host);
        long got = 0;
        int? status = null;
        Exception? err = null;
        var rhdrs = new Dictionary<string, string>(StringComparer.OrdinalIgnoreCase);
        byte[]? body = null;
        try
        {
            using var req = new HttpRequestMessage(new HttpMethod(method), url);
            if (headers is not null)
                foreach (var (k, v) in headers)
                {
                    if (k == "Range") req.Headers.Range = RangeHeaderValue.Parse(v);
                    else req.Headers.TryAddWithoutValidation(k, v);
                }
            using var cts = new CancellationTokenSource(TimeSpan.FromSeconds(timeoutS));
            using var resp = Http.SendAsync(req, HttpCompletionOption.ResponseHeadersRead, cts.Token).GetAwaiter().GetResult();
            status = (int)resp.StatusCode;
            foreach (var h in resp.Headers.Concat(resp.Content.Headers)) rhdrs[h.Key] = string.Join(", ", h.Value);
            if (!resp.IsSuccessStatusCode)
            {
                double? ra = null;
                if (resp.Headers.RetryAfter?.Delta is TimeSpan ts) ra = ts.TotalSeconds;
                throw new HttpError(status.Value, url) { RetryAfter = ra };
            }
            if (method == "HEAD") { }
            else if (reader is not null)
            {
                using var s = resp.Content.ReadAsStreamAsync(cts.Token).GetAwaiter().GetResult();
                got = reader(resp, s);
            }
            else
            {
                var n = resp.Content.Headers.ContentLength;
                if (n is not null && n > Limits.BigBytes)
                    throw new InvalidOperationException($"{url} announces {n / 1e6:0.0} MB: too big for an unplanned GET (plan it with a known size)");
                body = resp.Content.ReadAsByteArrayAsync(cts.Token).GetAwaiter().GetResult();
                got = body.Length;
            }
        }
        catch (Partial p)
        {
            err = p.Inner;
            got = p.NBytes;
            if (p.Inner is HttpError he) status = he.Code;
        }
        catch (Exception e)
        {
            err = e;
            if (e is HttpError he) status = he.Code;
        }
        finally
        {
            _ledger.Charge(new Dictionary<string, object?>
            {
                ["t"] = DateTime.UtcNow.ToString("yyyy-MM-dd'T'HH:mm:ss'+00:00'", CultureInfo.InvariantCulture),
                ["t_end"] = Now(), ["host"] = host, ["method"] = method, ["url"] = url,
                ["status"] = status, ["bytes"] = got, ["range"] = headers?.GetValueOrDefault("Range"),
                ["place"] = Place, ["note"] = note, ["dest"] = dest,
                ["error"] = err is null ? null : Truncate($"{err.GetType().Name}: {err.Message}", 300),
            });
        }
        if (err is not null) throw err;
        return (status, rhdrs, body);
    }

    static string Truncate(string s, int n) => s.Length <= n ? s : s[..n];

    /// Exchange with the back-off rule: on 429/5xx/timeout wait, retry ONCE, then Stop.
    (int? status, Dictionary<string, string> headers, byte[]? body) Request(
        string url, string method = "GET", Dictionary<string, string>? headers = null, int timeoutS = 120,
        Func<HttpResponseMessage, Stream, long>? reader = null, long? expectBytes = null, string note = "", string? dest = null)
    {
        try { return Exchange(url, method, headers, timeoutS, reader, expectBytes, note, dest); }
        catch (StopException) { throw; }
        catch (Exception e)
        {
            if (!Retryable(e)) throw;
            var w = RetryAfter(e);
            _ctx.Log($"    [polite] {HostOf(url)} said {(e is HttpError he ? he.Code.ToString() : e.GetType().Name)}; waiting {w:0} s for the one retry");
            Thread.Sleep(TimeSpan.FromSeconds(w));
        }
        try { return Exchange(url, method, headers, timeoutS, reader, expectBytes, note + " (retry)", dest); }
        catch (StopException) { throw; }
        catch (Exception e)
        {
            if (Retryable(e))
                throw new StopException($"{HostOf(url)} refused twice ({(e is HttpError he ? he.Code.ToString() : "")} {e.GetType().Name}: {e.Message}) for {url}", e);
            throw;
        }
    }

    // ------------------------------------------------------------------ API

    static void WriteNew(string path, byte[] data)
    {
        Directory.CreateDirectory(Path.GetDirectoryName(Path.GetFullPath(path))!);
        using var f = new FileStream(path, FileMode.CreateNew, FileAccess.Write);   // never replaces
        f.Write(data);
    }

    /// Cache-first GET of a small object.
    public override (byte[] data, bool cached) Get(string url, string? cachePath, double? maxAgeS = null, int timeoutS = 120)
        => Get(url, cachePath, timeoutS, "", null);

    public (byte[] data, bool cached) Get(string url, string? cachePath, int timeoutS = 120, string note = "", long? expect = null)
    {
        if (cachePath is not null && File.Exists(cachePath)) return (File.ReadAllBytes(cachePath), true);
        if (expect is not null && expect > Limits.BigBytes)
        {
            Download(url, cachePath!, expect.Value, timeoutS, note);
            return (File.ReadAllBytes(cachePath!), false);
        }
        var (_, _, body) = Request(url, timeoutS: timeoutS, note: note, expectBytes: expect, dest: cachePath);
        if (cachePath is not null) WriteNew(cachePath, body!);
        return (body!, false);
    }

    public (string text, bool cached) GetTextP(string url, string? cachePath, int timeoutS = 120, string note = "")
    {
        var (raw, c) = Get(url, cachePath, timeoutS, note);
        return (Encoding.UTF8.GetString(raw), c);
    }

    public sealed record HeadInfo(int? Status, long? Length, string AcceptRanges, string LastModified, string ContentType);

    /// HEAD, counted as a request of zero bytes.
    public HeadInfo Head(string url, string note = "size probe")
    {
        var (status, hdrs, _) = Request(url, "HEAD", note: note);
        var n = hdrs.GetValueOrDefault("content-length");
        return new HeadInfo(status, n is null ? null : long.Parse(n, CultureInfo.InvariantCulture),
            hdrs.GetValueOrDefault("accept-ranges", ""), hdrs.GetValueOrDefault("last-modified", ""), hdrs.GetValueOrDefault("content-type", ""));
    }

    /// Cache-first HTTP Range read.
    public (byte[] data, bool cached) GetRange(string url, long start, long endInclusive, string? cachePath, string note = "range")
    {
        if (cachePath is not null && File.Exists(cachePath)) return (File.ReadAllBytes(cachePath), true);
        var n = endInclusive - start + 1;
        var (status, _, body) = Request(url, headers: new() { ["Range"] = $"bytes={start}-{endInclusive}" }, note: note, expectBytes: n, dest: cachePath);
        if (status != 206) throw new InvalidOperationException($"range request answered {status} (wanted 206) for {url}");
        if (cachePath is not null) WriteNew(cachePath, body!);
        return (body!, false);
    }

    /// Cache-first suffix Range read (the last n bytes). Returns (bytes, total size, cached).
    public (byte[] data, long? total, bool cached) GetTail(string url, long n, string? cachePath, string note = "tail range")
    {
        var meta = cachePath is null ? null : cachePath + ".size";
        if (cachePath is not null && File.Exists(cachePath) && File.Exists(meta!))
            return (File.ReadAllBytes(cachePath), long.Parse(File.ReadAllText(meta!).Trim(), CultureInfo.InvariantCulture), true);
        var (status, hdrs, body) = Request(url, headers: new() { ["Range"] = $"bytes=-{n}" }, note: note, expectBytes: n, dest: cachePath);
        if (status != 206) throw new InvalidOperationException($"suffix range answered {status} (wanted 206) for {url}");
        var cr = hdrs.GetValueOrDefault("content-range", "");
        long? total = cr.Contains('/') ? long.Parse(cr[(cr.LastIndexOf('/') + 1)..], CultureInfo.InvariantCulture) : null;
        if (cachePath is not null)
        {
            WriteNew(cachePath, body!);
            WriteNew(meta!, Encoding.ASCII.GetBytes(total?.ToString(CultureInfo.InvariantCulture) ?? "None"));
        }
        return (body!, total, false);
    }

    /// Planned streaming download -> dest (size known in advance). Resumes from dest.part;
    /// renames to dest only when complete and the size equals `expect`.
    public string Download(string url, string dest, long expect, int timeoutS = 900, string note = "download")
    {
        if (File.Exists(dest)) return dest;
        Directory.CreateDirectory(Path.GetDirectoryName(Path.GetFullPath(dest))!);
        var part = dest + ".part";

        void Attempt(string tag)
        {
            var have = File.Exists(part) ? new FileInfo(part).Length : 0;
            if (have > expect) throw new InvalidOperationException($"{part} is larger ({have}) than planned ({expect})");
            if (have == expect) return;
            var t0 = _ctx.Seconds;
            long Reader(HttpResponseMessage r, Stream s)
            {
                var append = have > 0 && (int)r.StatusCode == 206;
                if (have > 0 && !append) _ctx.Log($"    [polite] {HostOf(url)} ignored Range; restarting {part}");
                long n = 0;
                try
                {
                    using var f = new FileStream(part, append ? FileMode.Append : FileMode.Create, FileAccess.Write);
                    var buf = new byte[1 << 20];
                    int k;
                    while ((k = s.Read(buf, 0, buf.Length)) > 0)
                    {
                        f.Write(buf, 0, k);
                        n += k;
                        if (_ledger.Bytes + n > MaxBytes) throw new StopException($"byte budget reached mid-download of {url}");
                    }
                }
                catch (Exception e) { throw new Partial(e, n); }
                return n;
            }
            var hdrs = have > 0 ? new Dictionary<string, string> { ["Range"] = $"bytes={have}-" } : null;
            Exchange(url, "GET", hdrs, timeoutS, Reader, expect - have, note + tag, dest);
            _ctx.Log($"    [polite] {Path.GetFileName(dest)}: {new FileInfo(part).Length / 1e6:0.0} MB on disk after {_ctx.Seconds - t0:0} s");
        }

        try { Attempt(""); }
        catch (StopException) { throw; }
        catch (Exception e)
        {
            if (!Retryable(e)) throw;
            var w = RetryAfter(e);
            _ctx.Log($"    [polite] {HostOf(url)}: {e.GetType().Name} {e.Message}; waiting {w:0} s, one resume");
            Thread.Sleep(TimeSpan.FromSeconds(w));
            try { Attempt(" (resume)"); }
            catch (StopException) { throw; }
            catch (Exception e2)
            {
                if (Retryable(e2)) throw new StopException($"{HostOf(url)} failed twice downloading {url}: {e2.Message}", e2);
                throw;
            }
        }
        var size = new FileInfo(part).Length;
        if (size != expect) throw new InvalidOperationException($"{part}: {size} B, planned {expect}; kept as .part for a resume");
        File.Move(part, dest);   // refuses to replace an existing file
        return dest;
    }

    // ------------------------------------------------------------------ plans

    public sealed class PlanItem
    {
        public string Key = "";
        public string Url = "";
        public long? Size;
        public string Dest = "";
        public bool Cached;
        public Dictionary<string, object?> Info = new();

        public Dictionary<string, object?> ToJson()
        {
            var d = new Dictionary<string, object?> { ["key"] = Key, ["url"] = Url, ["size"] = Size, ["dest"] = Dest };
            foreach (var (k, v) in Info) d[k] = v;
            if (Cached) d["cached"] = true;
            return d;
        }
    }

    /// An ordered list of planned downloads with known sizes; `Fit()` keeps items in order
    /// while they fit the remaining byte budget (minus a reserve) and reports what it cut.
    public sealed class Plan
    {
        readonly Polite _p;
        public string Title;
        public List<PlanItem> Items = new();
        public Plan(Polite p, string title) { _p = p; Title = title; }

        public void Add(string key, string url, long? size, string dest, params (string k, object? v)[] info)
        {
            var it = new PlanItem { Key = key, Url = url, Size = size, Dest = dest };
            foreach (var (k, v) in info) it.Info[k] = v;
            Items.Add(it);
        }

        public static long Total(IEnumerable<PlanItem> items) => items.Sum(i => i.Size ?? 0);

        public (List<PlanItem> keep, List<PlanItem> cut) Fit(long reserve = 0)
        {
            var room = _p.Remaining().bytes - reserve;
            var keep = new List<PlanItem>();
            var cut = new List<PlanItem>();
            long run = 0;
            foreach (var it in Items)
            {
                if (File.Exists(it.Dest)) { it.Cached = true; keep.Add(it); }
                else if (cut.Count == 0 && run + (it.Size ?? 0) <= room) { run += it.Size ?? 0; keep.Add(it); }
                else cut.Add(it);
            }
            return (keep, cut);
        }

        public void Print(List<PlanItem>? keep = null, List<PlanItem>? cut = null)
        {
            _p._ctx.Log($"PLAN {Title}: {Items.Count} item(s), {Total(Items) / 1e6:#,0.0} MB in all");
            var cutKeys = (cut ?? new()).Select(i => i.Key).ToHashSet();
            foreach (var it in Items)
            {
                var mark = File.Exists(it.Dest) ? "cached" : cutKeys.Contains(it.Key) ? "LATER" : "fetch";
                var extra = string.Join("  ", it.Info.Select(kv => $"{kv.Key}={Fmt(kv.Value)}"));
                _p._ctx.Log($"  {mark,-6} {it.Key,-44} {(it.Size ?? 0) / 1e6,9:0.0} MB  {extra}");
            }
            if (keep is not null)
            {
                var fresh = keep.Where(i => !i.Cached).ToList();
                var (b, r) = _p.Remaining();
                _p._ctx.Log($"  -> fetch {fresh.Count} ({Total(fresh) / 1e6:#,0.0} MB); leave {cut?.Count ?? 0} for a later run ({Total(cut ?? new()) / 1e6:#,0.0} MB); budget left {b / 1e9:0.000} GB / {r} requests");
            }
        }

        static string Fmt(object? v) => v is IFormattable f ? f.ToString(null, CultureInfo.InvariantCulture) : v?.ToString() ?? "None";
    }

    public Plan NewPlan(string title) => new(this, title);

    // ------------------------------------------------------------------ S3

    public sealed record S3Object(string Key, long Size, string Modified);

    /// Anonymous S3 ListObjectsV2 (the NODD buckets) -> (objects, common prefixes); each page
    /// is one request, cached under cachePath (+ .pN).
    public (List<S3Object> objects, List<string> prefixes) S3List(string bucket, string prefix, string cachePath, string? delimiter = null, int maxPages = 5)
    {
        var objs = new List<S3Object>();
        var prefixes = new List<string>();
        string? token = null;
        for (var page = 0; page < maxPages; page++)
        {
            var q = new List<string> { "list-type=2", "prefix=" + Uri.EscapeDataString(prefix) };
            if (delimiter is not null) q.Add("delimiter=" + Uri.EscapeDataString(delimiter));
            if (token is not null) q.Add("continuation-token=" + Uri.EscapeDataString(token));
            var url = $"https://{bucket}.s3.amazonaws.com/?" + string.Join("&", q);
            var cp = page == 0 ? cachePath : $"{cachePath}.p{page}";
            var (xml, _) = GetTextP(url, cp, note: "s3 listing");
            foreach (Match m in Regex.Matches(xml, "<Contents>(.*?)</Contents>", RegexOptions.Singleline))
            {
                var c = m.Groups[1].Value;
                var key = Regex.Match(c, "<Key>([^<]*)</Key>").Groups[1].Value;
                var size = long.Parse(Regex.Match(c, @"<Size>(\d+)</Size>").Groups[1].Value, CultureInfo.InvariantCulture);
                var mod = Regex.Match(c, "<LastModified>([^<]*)</LastModified>") is { Success: true } mm ? mm.Groups[1].Value : "";
                objs.Add(new S3Object(key, size, mod));
            }
            foreach (Match m in Regex.Matches(xml, "<CommonPrefixes><Prefix>([^<]*)</Prefix></CommonPrefixes>"))
                prefixes.Add(m.Groups[1].Value);
            if (!xml.Contains("<IsTruncated>true</IsTruncated>")) break;
            token = Regex.Match(xml, "<NextContinuationToken>([^<]*)</NextContinuationToken>").Groups[1].Value;
        }
        return (objs, prefixes);
    }
}
