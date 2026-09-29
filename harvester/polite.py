"""The polite fetcher for PLACE harvests: harvest_currents.fetch's image, with a budget.

Every harvester in this folder carries its own small fetch (cache-first, spaced, identified).
A place harvest (harvest_place.py, and the --place modes of harvest_bathy and harvest_tides)
routes every request through THIS one instead, because a place is new ground and the owner's
rules for new ground are stricter:

  * CACHE FIRST, FOREVER: a file that is on disk is never requested again. Time-varying
    products get time-stamped cache names (the NDBC `<id>_<YYYYMMDDHH>.txt` idiom), so nothing
    is ever re-fetched over an existing file -- and no existing file is ever overwritten (cache
    files are created with exclusive-create, final downloads are renamed from .part with a
    rename that refuses to replace).
  * ONE request in flight -- this module is synchronous, and a lock file serializes requests
    across processes -- and at least MIN_HOST_GAP_S between two requests to the same host,
    remembered across processes through the ledger.
  * 429 / 5xx / timeout: wait (Retry-After if the host names one, else BACKOFF_S), ONE retry;
    if that fails too, raise Stop -- the run ends and says why. No loops.
  * THE BUDGET: bytes and requests are charged to a persistent ledger (JSON lines) and checked
    BEFORE each request; Stop is raised rather than exceed MAX_BYTES / MAX_REQUESTS. Every
    request counts: listings, HEADs, byte ranges, failures.
  * Anything larger than BIG_BYTES must be PLANNED: its size known in advance (a HEAD, an S3
    listing, a directory's size column) and passed as `expect`. It streams into `<name>.part`,
    resumes with a Range request when a .part is found, and is renamed to its final name only
    when complete and the size is right.
  * NO GOOGLE: requests to Google hosts are refused outright.
  * Identification: the project's product token WITHOUT the contact address -- the owner asked
    that no name, e-mail or machine detail leave this machine on a place harvest.

stdlib only.
"""

import json
import os
import socket
import sys
import time
import urllib.error
import urllib.parse
import urllib.request
from datetime import datetime, timezone

UA = "GAGAME/0.1 (hobby ocean simulator)"

MAX_BYTES = 8_000_000_000          # 8 GB, read as decimal: the smaller of the two readings
MAX_REQUESTS = 2000
MIN_HOST_GAP_S = 1.1               # >= 1 s between requests to one host, with clock slack
HOST_GAP_S = {"nomads.ncep.noaa.gov": 2.5}   # NOAA's own rate-limited server: well under 120/min
BACKOFF_S = 30.0
BIG_BYTES = 50_000_000
SMALL_GET_LIMIT = BIG_BYTES        # an unplanned GET that announces more than this is refused
LEDGER = os.environ.get("GAGAME_LEDGER", os.path.join("out", "places", "ledger.jsonl"))

# Recorded with every ledger line (the orchestrator sets the place it is working for).
CONTEXT = {"place": None}

_BANNED = ("google.", "googleapis.", "gstatic.", "ggpht.", "googleusercontent.", "youtube.",
           "withgoogle.", "gvt1.", "blogspot.", "googlevideo.")


class Stop(RuntimeError):
    """The budget would be exceeded, or a host refused twice: end the run and report."""


def log(msg):
    print(msg, flush=True)


# ------------------------------------------------------------------------------------ ledger

class _Ledger:
    def __init__(self, path):
        self.path = path
        self.bytes = 0
        self.requests = 0
        self.per_host = {}         # host -> [requests, bytes]
        self.last_at = {}          # host -> unix time the last request to it ENDED
        self.largest = (0, "")
        if os.path.exists(path):
            with open(path, "r", encoding="utf-8") as f:
                for ln in f:
                    ln = ln.strip()
                    if ln:
                        self._add(json.loads(ln))

    def _add(self, e):
        n = e.get("bytes") or 0
        self.bytes += n
        self.requests += 1
        h = self.per_host.setdefault(e["host"], [0, 0])
        h[0] += 1
        h[1] += n
        self.last_at[e["host"]] = max(self.last_at.get(e["host"], 0.0), e.get("t_end", 0.0))
        if n > self.largest[0]:
            self.largest = (n, e["url"])

    def charge(self, entry):
        os.makedirs(os.path.dirname(os.path.abspath(self.path)), exist_ok=True)
        with open(self.path, "a", encoding="utf-8") as f:
            f.write(json.dumps(entry) + "\n")
        self._add(entry)


_ledger = None


def ledger():
    global _ledger
    if _ledger is None:
        _ledger = _Ledger(LEDGER)
    return _ledger


def remaining():
    lg = ledger()
    return MAX_BYTES - lg.bytes, MAX_REQUESTS - lg.requests


def summary():
    lg = ledger()
    return {"requests": lg.requests, "bytes": lg.bytes,
            "per_host": {h: {"requests": v[0], "bytes": v[1]}
                         for h, v in sorted(lg.per_host.items())},
            "largest": {"bytes": lg.largest[0], "url": lg.largest[1]},
            "budget": {"bytes": MAX_BYTES, "requests": MAX_REQUESTS}}


# ------------------------------------------------------------------------------------ lock

class _OneInFlight:
    """An exclusive lock on a lock file held for the duration of each request, so two harvest
    processes can never have requests in flight at the same time."""

    def __enter__(self):
        path = os.path.abspath(LEDGER) + ".lock"
        os.makedirs(os.path.dirname(path), exist_ok=True)
        self.f = open(path, "a+b")
        if os.name == "nt":
            import msvcrt
            while True:
                try:
                    self.f.seek(0)
                    msvcrt.locking(self.f.fileno(), msvcrt.LK_NBLCK, 1)
                    break
                except OSError:
                    time.sleep(0.25)
        else:
            import fcntl
            fcntl.flock(self.f.fileno(), fcntl.LOCK_EX)
        return self

    def __exit__(self, *exc):
        try:
            if os.name == "nt":
                import msvcrt
                self.f.seek(0)
                msvcrt.locking(self.f.fileno(), msvcrt.LK_UNLCK, 1)
            else:
                import fcntl
                fcntl.flock(self.f.fileno(), fcntl.LOCK_UN)
        finally:
            self.f.close()
        return False


# ------------------------------------------------------------------------------------ request

def host_of(url):
    return urllib.parse.urlsplit(url).hostname or "?"


def _check_host(url):
    h = host_of(url).lower()
    if any(b in h + "." for b in _BANNED):
        raise Stop(f"refusing a Google host: {h}")


def _pace(host):
    gap = HOST_GAP_S.get(host, MIN_HOST_GAP_S)
    wait = gap - (time.time() - ledger().last_at.get(host, 0.0))
    if wait > 0:
        time.sleep(wait)


def _admit(url, expect_bytes):
    lg = ledger()
    if lg.requests + 1 > MAX_REQUESTS:
        raise Stop(f"request budget spent ({lg.requests}/{MAX_REQUESTS}); not requesting {url}")
    if lg.bytes + (expect_bytes or 0) > MAX_BYTES:
        raise Stop(f"byte budget: {lg.bytes / 1e9:.3f} GB used + {(expect_bytes or 0) / 1e9:.3f}"
                   f" GB planned > {MAX_BYTES / 1e9:.1f} GB; not requesting {url}")


def _retryable(err):
    if isinstance(err, urllib.error.HTTPError):
        return err.code == 429 or 500 <= err.code <= 599
    return isinstance(err, (urllib.error.URLError, TimeoutError, socket.timeout,
                            ConnectionError))


def _retry_after(err):
    try:
        ra = err.headers.get("Retry-After") if isinstance(err, urllib.error.HTTPError) else None
        if ra is not None:
            return min(max(float(ra), 1.0), 300.0)
    except (TypeError, ValueError, AttributeError):
        pass
    return BACKOFF_S


class _Partial(Exception):
    """Raised inside a body reader to abort; carries the bytes already received."""

    def __init__(self, inner, nbytes):
        super().__init__(str(inner))
        self.inner = inner
        self.nbytes = nbytes


def _exchange(url, method, headers, timeout, reader, expect_bytes, note, dest=None):
    """ONE request: admit, pace, lock, send, charge. Returns (status, headers, result) where
    result is reader(response) (or the body bytes when reader is None). Raises the error."""
    _check_host(url)
    host = host_of(url)
    hdrs = {"User-Agent": UA, "Accept-Encoding": "identity"}
    hdrs.update(headers or {})
    _admit(url, expect_bytes)
    with _OneInFlight():
        _pace(host)
        got = 0
        status = None
        err = None
        rhdrs = {}
        result = None
        try:
            req = urllib.request.Request(url, headers=hdrs, method=method)
            with urllib.request.urlopen(req, timeout=timeout) as r:
                status = r.status
                rhdrs = dict(r.headers.items())
                if method == "HEAD":
                    result = None
                elif reader is not None:
                    result, got = reader(r)
                else:
                    n = r.headers.get("Content-Length")
                    if n is not None and int(n) > SMALL_GET_LIMIT:
                        raise RuntimeError(f"{url} announces {int(n) / 1e6:.1f} MB: too big for "
                                           f"an unplanned GET (plan it with a known size)")
                    result = r.read()
                    got = len(result)
        except _Partial as e:
            err = e.inner
            got = e.nbytes
            status = getattr(e.inner, "code", status)
        except Exception as e:  # noqa: BLE001 - classified by the caller
            err = e
            status = getattr(e, "code", status)
        finally:
            ledger().charge({
                "t": datetime.now(timezone.utc).isoformat(timespec="seconds"),
                "t_end": time.time(), "host": host, "method": method, "url": url,
                "status": status, "bytes": got, "range": (headers or {}).get("Range"),
                "place": CONTEXT.get("place"), "note": note, "dest": dest,
                "error": None if err is None else f"{type(err).__name__}: {err}"[:300]})
    if err is not None:
        raise err
    return status, rhdrs, result


def _request(url, method="GET", headers=None, timeout=120, reader=None, expect_bytes=None,
             note="", dest=None):
    """_exchange with the owner's back-off rule: on 429/5xx/timeout wait, retry ONCE, then
    Stop. Other HTTP errors (404 ...) are raised to the caller unretried."""
    try:
        return _exchange(url, method, headers, timeout, reader, expect_bytes, note, dest)
    except Stop:
        raise
    except Exception as e:  # noqa: BLE001
        if not _retryable(e):
            raise
        w = _retry_after(e)
        log(f"    [polite] {host_of(url)} said {getattr(e, 'code', None) or type(e).__name__};"
            f" waiting {w:.0f} s for the one retry")
        time.sleep(w)
    try:
        return _exchange(url, method, headers, timeout, reader, expect_bytes, note + " (retry)",
                         dest)
    except Stop:
        raise
    except Exception as e:  # noqa: BLE001
        if _retryable(e):
            raise Stop(f"{host_of(url)} refused twice ({getattr(e, 'code', None) or ''} "
                       f"{type(e).__name__}: {e}) for {url}") from e
        raise


# ------------------------------------------------------------------------------------ API

def _write_new(path, data):
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    with open(path, "xb") as f:          # exclusive create: an existing file is never replaced
        f.write(data)


def get(url, cache_path, timeout=120, note="", expect=None):
    """Cache-first GET of a small object. Returns (bytes, cached)."""
    if cache_path and os.path.exists(cache_path):
        with open(cache_path, "rb") as f:
            return f.read(), True
    if expect is not None and expect > BIG_BYTES:
        download(url, cache_path, expect, timeout=timeout, note=note)
        with open(cache_path, "rb") as f:
            return f.read(), False
    _, _, body = _request(url, timeout=timeout, note=note, expect_bytes=expect, dest=cache_path)
    if cache_path:
        _write_new(cache_path, body)
    return body, False


def get_text(url, cache_path, **kw):
    raw, cached = get(url, cache_path, **kw)
    return raw.decode("utf-8", errors="replace"), cached


def head(url, note="size probe"):
    """HEAD -> {'status', 'length', 'accept_ranges', 'last_modified', 'content_type'}.
    Counted as a request of zero bytes."""
    status, hdrs, _ = _request(url, method="HEAD", note=note)
    low = {k.lower(): v for k, v in hdrs.items()}
    n = low.get("content-length")
    return {"status": status, "length": int(n) if n is not None else None,
            "accept_ranges": low.get("accept-ranges", ""),
            "last_modified": low.get("last-modified", ""),
            "content_type": low.get("content-type", "")}


def get_range(url, start, end_inclusive, cache_path, note="range"):
    """Cache-first HTTP Range read (bytes=start-end). Returns (bytes, cached)."""
    if cache_path and os.path.exists(cache_path):
        with open(cache_path, "rb") as f:
            return f.read(), True
    n = end_inclusive - start + 1
    status, _, body = _request(url, headers={"Range": f"bytes={start}-{end_inclusive}"},
                               note=note, expect_bytes=n, dest=cache_path)
    if status != 206:
        raise RuntimeError(f"range request answered {status} (wanted 206) for {url}")
    if cache_path:
        _write_new(cache_path, body)
    return body, False


def get_tail(url, n, cache_path, note="tail range"):
    """Cache-first suffix Range read (the last n bytes: a zip's central directory).
    Returns (bytes, total_size, cached)."""
    meta = cache_path + ".size" if cache_path else None
    if cache_path and os.path.exists(cache_path) and os.path.exists(meta):
        with open(cache_path, "rb") as f, open(meta, "r") as g:
            return f.read(), int(g.read().strip()), True
    status, hdrs, body = _request(url, headers={"Range": f"bytes=-{n}"}, note=note,
                                  expect_bytes=n, dest=cache_path)
    if status != 206:
        raise RuntimeError(f"suffix range answered {status} (wanted 206) for {url}")
    cr = {k.lower(): v for k, v in hdrs.items()}.get("content-range", "")
    total = int(cr.rsplit("/", 1)[-1]) if "/" in cr else None
    if cache_path:
        _write_new(cache_path, body)
        _write_new(meta, str(total).encode())
    return body, total, False


def download(url, dest, expect, timeout=900, note="download"):
    """Planned streaming download -> dest (size `expect` known in advance). Resumes from
    dest.part; renames to dest only when complete and its size equals `expect`."""
    if os.path.exists(dest):
        return dest
    if expect is None:
        raise RuntimeError(f"download of {url} has no planned size")
    os.makedirs(os.path.dirname(os.path.abspath(dest)), exist_ok=True)
    part = dest + ".part"

    def attempt(tag):
        have = os.path.getsize(part) if os.path.exists(part) else 0
        if have > expect:
            raise RuntimeError(f"{part} is larger ({have}) than planned ({expect})")
        if have == expect:
            return
        t0 = time.time()

        def reader(r):
            mode = "ab"
            if have and r.status != 206:
                mode = "wb"                 # host ignored the Range: restart OUR OWN .part
                log(f"    [polite] {host_of(url)} ignored Range; restarting {part}")
            elif not have:
                mode = "wb"
            n = 0
            try:
                with open(part, mode) as f:
                    while True:
                        chunk = r.read(1 << 20)
                        if not chunk:
                            break
                        f.write(chunk)
                        n += len(chunk)
                        if ledger().bytes + n > MAX_BYTES:
                            raise Stop(f"byte budget reached mid-download of {url}")
            except Exception as e:  # noqa: BLE001
                raise _Partial(e, n)
            return None, n

        hdrs = {"Range": f"bytes={have}-"} if have else {}
        _exchange(url, "GET", hdrs, timeout, reader, expect - have, f"{note}{tag}", dest)
        log(f"    [polite] {os.path.basename(dest)}: {os.path.getsize(part) / 1e6:.1f} MB "
            f"on disk after {time.time() - t0:.0f} s")

    try:
        attempt("")
    except Stop:
        raise
    except Exception as e:  # noqa: BLE001
        if not _retryable(e):
            raise
        w = _retry_after(e)
        log(f"    [polite] {host_of(url)}: {type(e).__name__} {e}; waiting {w:.0f} s, one "
            f"resume")
        time.sleep(w)
        try:
            attempt(" (resume)")
        except Stop:
            raise
        except Exception as e2:  # noqa: BLE001
            if _retryable(e2):
                raise Stop(f"{host_of(url)} failed twice downloading {url}: {e2}") from e2
            raise
    size = os.path.getsize(part)
    if size != expect:
        raise RuntimeError(f"{part}: {size} B, planned {expect}; kept as .part for a resume")
    os.rename(part, dest)       # os.rename refuses to replace an existing file on Windows
    return dest


# ------------------------------------------------------------------------------------ plans

class Plan:
    """An ordered list of planned downloads with known sizes. `fit()` keeps items in order
    while they fit the remaining byte budget (minus a reserve) and reports what it cut."""

    def __init__(self, title):
        self.title = title
        self.items = []

    def add(self, key, url, size, dest, **info):
        self.items.append(dict(key=key, url=url, size=size, dest=dest, **info))

    @staticmethod
    def total(items):
        return sum(i["size"] or 0 for i in items)

    def fit(self, reserve=0):
        room = remaining()[0] - reserve
        keep, cut = [], []
        run = 0
        for it in self.items:
            if os.path.exists(it["dest"]):
                it["cached"] = True
                keep.append(it)
            elif not cut and run + (it["size"] or 0) <= room:
                run += it["size"] or 0
                keep.append(it)
            else:
                cut.append(it)      # in order: once one item does not fit, the rest wait too
        return keep, cut

    def print(self, keep=None, cut=None):
        log(f"PLAN {self.title}: {len(self.items)} item(s), "
            f"{self.total(self.items) / 1e6:,.1f} MB in all")
        cut_keys = {i["key"] for i in (cut or [])}
        for it in self.items:
            mark = ("cached" if os.path.exists(it["dest"]) else
                    "LATER" if it["key"] in cut_keys else "fetch")
            extra = "  ".join(f"{k}={v}" for k, v in it.items()
                              if k not in ("key", "url", "size", "dest", "cached"))
            log(f"  {mark:6s} {it['key']:44s} {(it['size'] or 0) / 1e6:9.1f} MB  {extra}")
        if keep is not None:
            fresh = [i for i in keep if not i.get("cached")]
            log(f"  -> fetch {len(fresh)} ({self.total(fresh) / 1e6:,.1f} MB); leave "
                f"{len(cut or [])} for a later run ({self.total(cut or []) / 1e6:,.1f} MB); "
                f"budget left {remaining()[0] / 1e9:.3f} GB / {remaining()[1]} requests")


def s3_list(bucket, prefix, cache_path, delimiter=None, max_pages=5):
    """Anonymous S3 ListObjectsV2 (the NODD buckets) -> (objects, prefixes): objects are
    {'key', 'size', 'modified'}. Each page is one request, cached under cache_path (+ .pN)."""
    import re
    objs, prefixes = [], []
    token = None
    for page in range(max_pages):
        q = {"list-type": "2", "prefix": prefix}
        if delimiter:
            q["delimiter"] = delimiter
        if token:
            q["continuation-token"] = token
        url = f"https://{bucket}.s3.amazonaws.com/?" + urllib.parse.urlencode(q)
        cp = cache_path if page == 0 else f"{cache_path}.p{page}"
        xml, _ = get_text(url, cp, note="s3 listing")
        for m in re.finditer(r"<Contents>(.*?)</Contents>", xml, re.S):
            c = m.group(1)
            key = re.search(r"<Key>([^<]*)</Key>", c).group(1)
            size = int(re.search(r"<Size>(\d+)</Size>", c).group(1))
            mod = (re.search(r"<LastModified>([^<]*)</LastModified>", c) or [None, ""])[1]
            objs.append({"key": key, "size": size, "modified": mod})
        prefixes += re.findall(r"<CommonPrefixes><Prefix>([^<]*)</Prefix></CommonPrefixes>", xml)
        if "<IsTruncated>true</IsTruncated>" not in xml:
            break
        token = re.search(r"<NextContinuationToken>([^<]*)</NextContinuationToken>", xml).group(1)
    return objs, prefixes


def fetch_compat(url, cache_path, binary=False, max_age_s=None, timeout=90):
    """harvest_currents.fetch's signature, served politely. max_age_s is IGNORED on purpose:
    a file on disk is never requested again (callers that need freshness use stamped names)."""
    raw, cached = get(url, cache_path, timeout=timeout)
    return (raw if binary else raw.decode("utf-8", errors="replace")), cached


if __name__ == "__main__":
    print(json.dumps(summary(), indent=1))
    sys.exit(0)
