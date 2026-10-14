using System.Net;
using System.Net.Http.Headers;

namespace Gagame.Harvest;

/// The harvesters' own small fetch (every one carried a copy): cache-first, spaced requests
/// to be gentle, a few retries with backoff, the project's User-Agent. A place harvest routes
/// through Polite instead (budgeted, ledgered); `Fetcher` is the abstraction both satisfy.
public abstract class Fetcher
{
    /// Cache-first GET. Returns the bytes and whether they came from the cache.
    public abstract (byte[] data, bool cached) Get(string url, string? cachePath, double? maxAgeS = null, int timeoutS = 90);

    public (string text, bool cached) GetText(string url, string? cachePath, double? maxAgeS = null, int timeoutS = 90)
    {
        var (b, c) = Get(url, cachePath, maxAgeS, timeoutS);
        return (System.Text.Encoding.UTF8.GetString(b), c);
    }
}

public sealed class SimpleFetcher : Fetcher
{
    static readonly HttpClient Http = Make();
    static HttpClient Make()
    {
        var h = new HttpClient(new HttpClientHandler { AutomaticDecompression = DecompressionMethods.GZip | DecompressionMethods.Deflate })
        { Timeout = TimeSpan.FromSeconds(1800) };
        h.DefaultRequestHeaders.UserAgent.ParseAdd("GAGAME/0.1");
        h.DefaultRequestHeaders.UserAgent.Add(new ProductInfoHeaderValue("(hobby ocean simulator; contact: marksmcgrotty@gmail.com)"));
        return h;
    }

    readonly HarvestContext _ctx;
    readonly double _gapS;
    readonly int _tries;
    double _lastEnd = double.NegativeInfinity;

    public SimpleFetcher(HarvestContext ctx, double gapS = 0.6, int tries = 4)
    {
        _ctx = ctx;
        _gapS = gapS;
        _tries = tries;
    }

    public override (byte[] data, bool cached) Get(string url, string? cachePath, double? maxAgeS = null, int timeoutS = 90)
    {
        if (cachePath is not null && File.Exists(cachePath))
        {
            var age = (DateTime.UtcNow - File.GetLastWriteTimeUtc(cachePath)).TotalSeconds;
            if (maxAgeS is null || age < maxAgeS) return (File.ReadAllBytes(cachePath), true);
        }
        var data = Download(url, timeoutS);
        if (cachePath is not null)
        {
            Directory.CreateDirectory(Path.GetDirectoryName(Path.GetFullPath(cachePath))!);
            File.WriteAllBytes(cachePath, data);
        }
        return (data, false);
    }

    /// GET with the spacing and the retries; HTTP errors other than 429/5xx are raised at once.
    public byte[] Download(string url, int timeoutS)
    {
        Exception? last = null;
        for (var attempt = 0; attempt < _tries; attempt++)
        {
            var wait = _gapS - (_ctx.Seconds - _lastEnd);
            if (wait > 0) Thread.Sleep(TimeSpan.FromSeconds(wait));
            try
            {
                using var cts = new CancellationTokenSource(TimeSpan.FromSeconds(timeoutS));
                using var resp = Http.GetAsync(url, HttpCompletionOption.ResponseHeadersRead, cts.Token).GetAwaiter().GetResult();
                _lastEnd = _ctx.Seconds;
                if (!resp.IsSuccessStatusCode)
                {
                    var code = (int)resp.StatusCode;
                    var e = new HttpError(code, url);
                    if (code == 429 || code >= 500) { last = e; }
                    else throw e;
                }
                else
                {
                    return resp.Content.ReadAsByteArrayAsync(cts.Token).GetAwaiter().GetResult();
                }
            }
            catch (HttpError) { throw; }
            catch (Exception e)
            {
                _lastEnd = _ctx.Seconds;
                last = e;
            }
            if (attempt == _tries - 1) throw last!;
            var backoff = 2.0 * Math.Pow(2, attempt);
            _ctx.Log($"    retry in {backoff:0}s ({last!.Message})");
            Thread.Sleep(TimeSpan.FromSeconds(backoff));
        }
        throw last!;
    }

    /// Streamed download to `dest` through `dest.part`, with progress; a cached file wins.
    public void DownloadBig(string[] urls, string dest, long minBytes = 1 << 20, int timeoutS = 1200, string tag = "")
    {
        if (File.Exists(dest) && new FileInfo(dest).Length > minBytes)
        {
            _ctx.Log($"  cached: {dest} ({new FileInfo(dest).Length >> 20} MB)");
            return;
        }
        Directory.CreateDirectory(Path.GetDirectoryName(Path.GetFullPath(dest))!);
        Exception? last = null;
        foreach (var url in urls)
        {
            try
            {
                _ctx.Log($"  GET {url}");
                using var cts = new CancellationTokenSource(TimeSpan.FromSeconds(timeoutS));
                using var resp = Http.GetAsync(url, HttpCompletionOption.ResponseHeadersRead, cts.Token).GetAwaiter().GetResult();
                resp.EnsureSuccessStatusCode();
                var total = resp.Content.Headers.ContentLength ?? 0;
                using var src = resp.Content.ReadAsStreamAsync(cts.Token).GetAwaiter().GetResult();
                using var f = File.Create(dest + ".part");
                var buf = new byte[1 << 20];
                long done = 0;
                var t0 = _ctx.Seconds;
                var nextSay = 64L << 20;
                int n;
                while ((n = src.Read(buf, 0, buf.Length)) > 0)
                {
                    f.Write(buf, 0, n);
                    done += n;
                    if (done >= nextSay)
                    {
                        nextSay += 64L << 20;
                        _ctx.Log($"    {done >> 20} / {total >> 20} MB  ({(done >> 20) / Math.Max(_ctx.Seconds - t0, 1):0} MB/s)");
                    }
                }
                f.Close();
                File.Move(dest + ".part", dest, overwrite: true);
                _ctx.Log($"  done: {new FileInfo(dest).Length >> 20} MB");
                return;
            }
            catch (Exception e)
            {
                last = e;
                _ctx.Log($"    failed ({e.Message}); trying next mirror");
            }
        }
        throw new InvalidOperationException($"all mirrors failed{(tag.Length > 0 ? " for " + tag : "")}: {last?.Message}");
    }

    public static HttpClient Client => Http;
}

public sealed class HttpError : Exception
{
    public int Code { get; }
    /// The host's Retry-After in seconds, when it named one.
    public double? RetryAfter { get; init; }
    /// The start of the error's body (a server's own reason: ERDDAP's 404 says "no matching
    /// results" for an empty answer and "unknown datasetID" for a dataset it does not hold).
    public string Body { get; init; } = "";
    public HttpError(int code, string url) : base($"HTTP Error {code} for {url}") => Code = code;
}
