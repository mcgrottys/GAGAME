#include "core/TileProviders.h"

#include "core/Json.h"

#include <windows.h>
#include <winhttp.h>
#include <wincodec.h>

#include <chrono>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <thread>

#pragma comment(lib, "winhttp.lib")

namespace ga {

// ------------------------------------------------------------------------------ Mars

bool MarsBinProvider::Open(const std::wstring& binPath, DXGI_FORMAT fmt) {
    switch (fmt) {
        case DXGI_FORMAT_BC1_UNORM:
        case DXGI_FORMAT_BC1_UNORM_SRGB: m_tileW = 512; m_tileH = 256; break;
        case DXGI_FORMAT_BC5_SNORM: m_tileW = 256; m_tileH = 256; break;
        default: return false;
    }
    HANDLE h = CreateFileW(binPath.c_str(), GENERIC_READ, FILE_SHARE_READ, nullptr,
                           OPEN_EXISTING, FILE_FLAG_RANDOM_ACCESS, nullptr);
    if (h == INVALID_HANDLE_VALUE) {
        Log("[mars] no %S (copy the sample's .bin next to it); Mars stays procedural",
            binPath.c_str());
        return false;
    }
    LARGE_INTEGER sz{};
    GetFileSizeEx(h, &sz);
    m_file = h;
    m_tilesPerFace = static_cast<uint64_t>(sz.QuadPart) / 65536 / 6;

    // The sample's own layout walk: per face, mip tile-counts shrink by /4 (min 1) until the
    // face's tiles are consumed; our resource only uses the UNPACKED mips, whose counts match
    // real width x height products.
    uint64_t prefix = 0;
    for (uint32_t dim = 16384; dim >= (std::max)(m_tileW, m_tileH) && prefix < m_tilesPerFace;
         dim >>= 1) {
        const uint32_t w = (std::max)(1u, dim / m_tileW);
        const uint32_t hgt = (std::max)(1u, dim / m_tileH);
        m_mipPrefix.push_back(prefix);
        m_mipTilesW.push_back(w);
        m_mipTilesH.push_back(hgt);
        prefix += static_cast<uint64_t>(w) * hgt;
    }
    Log("[mars] %S: %llu tiles/face, %zu streamable mips", binPath.c_str(),
        static_cast<unsigned long long>(m_tilesPerFace), m_mipPrefix.size());
    return true;
}

TileProviderFn MarsBinProvider::Fn() {
    return [this](const TileRequest& r, std::vector<uint8_t>& out, TileLoc*) {
        if (!m_file || r.mip >= m_mipPrefix.size()) return false;
        const uint64_t tileIndex = r.face * m_tilesPerFace + m_mipPrefix[r.mip] +
                                   static_cast<uint64_t>(r.y) * m_mipTilesW[r.mip] + r.x;
        const uint64_t offset = tileIndex * 65536;
        out.resize(65536);
        OVERLAPPED ov{};
        ov.Offset = static_cast<DWORD>(offset & 0xFFFFFFFFull);
        ov.OffsetHigh = static_cast<DWORD>(offset >> 32);
        DWORD read = 0;
        if (!ReadFile(static_cast<HANDLE>(m_file), out.data(), 65536, &read, &ov)) {
            if (GetLastError() != ERROR_IO_PENDING) return false;
            if (!GetOverlappedResult(static_cast<HANDLE>(m_file), &ov, &read, TRUE)) return false;
        }
        return read == 65536;
    };
}

// ------------------------------------------------------------------------------ Google

static long long NowMs() {
    return std::chrono::duration_cast<std::chrono::milliseconds>(
               std::chrono::steady_clock::now().time_since_epoch())
        .count();
}

static std::string ReadKeyFromEnvOrRegistry() {
    char buf[256]{};
    DWORD n = GetEnvironmentVariableA("GAGAME_GOOGLE_MAPS_KEY", buf, sizeof(buf));
    if (n > 0 && n < sizeof(buf)) return buf;
    // setx lands in HKCU\Environment; processes started before it never inherit -- read direct.
    HKEY k;
    if (RegOpenKeyExA(HKEY_CURRENT_USER, "Environment", 0, KEY_READ, &k) == ERROR_SUCCESS) {
        DWORD type = 0, sz = sizeof(buf);
        const LSTATUS s =
            RegQueryValueExA(k, "GAGAME_GOOGLE_MAPS_KEY", nullptr, &type,
                             reinterpret_cast<LPBYTE>(buf), &sz);
        RegCloseKey(k);
        if (s == ERROR_SUCCESS && type == REG_SZ) return buf;
    }
    return {};
}

std::string LoggablePath(const std::wstring& pathAndQuery) {
    const std::wstring path = pathAndQuery.substr(0, pathAndQuery.find(L'?'));
    std::string out;
    out.reserve(path.size());
    for (const wchar_t c : path) out += (c > 0 && c < 128) ? static_cast<char>(c) : '_';
    return out;
}

// Minimal WinHTTP request. Returns false on any transport or non-200 status. `path` carries the
// query (the key, the session's token): a log line names the request by LoggablePath only.
static bool Http(const wchar_t* method, const std::wstring& host, const std::wstring& path,
                 const std::string& body, std::vector<uint8_t>& out) {
    bool ok = false;
    HINTERNET ses = WinHttpOpen(L"GAGAME/0.1 (hobby ocean simulator)",
                                WINHTTP_ACCESS_TYPE_AUTOMATIC_PROXY, WINHTTP_NO_PROXY_NAME,
                                WINHTTP_NO_PROXY_BYPASS, 0);
    if (!ses) return false;
    WinHttpSetTimeouts(ses, 10000, 10000, 15000, 30000);
    HINTERNET con = WinHttpConnect(ses, host.c_str(), INTERNET_DEFAULT_HTTPS_PORT, 0);
    HINTERNET req = con ? WinHttpOpenRequest(con, method, path.c_str(), nullptr,
                                             WINHTTP_NO_REFERER, WINHTTP_DEFAULT_ACCEPT_TYPES,
                                             WINHTTP_FLAG_SECURE)
                        : nullptr;
    if (req) {
        const wchar_t* hdr = body.empty() ? nullptr : L"Content-Type: application/json\r\n";
        if (WinHttpSendRequest(req, hdr, hdr ? static_cast<DWORD>(-1) : 0,
                               body.empty() ? nullptr : const_cast<char*>(body.data()),
                               static_cast<DWORD>(body.size()), static_cast<DWORD>(body.size()),
                               0) &&
            WinHttpReceiveResponse(req, nullptr)) {
            DWORD status = 0, sz = sizeof(status);
            WinHttpQueryHeaders(req, WINHTTP_QUERY_STATUS_CODE | WINHTTP_QUERY_FLAG_NUMBER,
                                WINHTTP_HEADER_NAME_BY_INDEX, &status, &sz,
                                WINHTTP_NO_HEADER_INDEX);
            if (status == 200) {
                // A BODY IS WHOLE OR IT IS A FAILURE: a query or a read that fails mid-body
                // fails the request (the part read is not a tile), and when the response says
                // its Content-Length the body must be exactly that long.
                bool whole = true;
                for (;;) {
                    DWORD avail = 0;
                    if (!WinHttpQueryDataAvailable(req, &avail)) { whole = false; break; }
                    if (avail == 0) break;   // the end of the body
                    const size_t base = out.size();
                    out.resize(base + avail);
                    DWORD got = 0;
                    if (!WinHttpReadData(req, out.data() + base, avail, &got)) {
                        out.resize(base);
                        whole = false;
                        break;
                    }
                    out.resize(base + got);
                }
                DWORD said = 0, saidSz = sizeof(said);
                const bool hasLength =
                    WinHttpQueryHeaders(req, WINHTTP_QUERY_CONTENT_LENGTH | WINHTTP_QUERY_FLAG_NUMBER,
                                        WINHTTP_HEADER_NAME_BY_INDEX, &said, &saidSz,
                                        WINHTTP_NO_HEADER_INDEX) != FALSE;
                if (whole && hasLength && out.size() != said) whole = false;
                ok = whole && !out.empty();
                if (!ok) {
                    Log("[google] a body not whole on %s (%zu bytes read%s): the request failed",
                        LoggablePath(path).c_str(), out.size(),
                        hasLength ? (", " + std::to_string(said) + " said").c_str() : "");
                }
            } else {
                Log("[google] HTTP %lu on %s", status, LoggablePath(path).c_str());
            }
        }
    }
    if (req) WinHttpCloseHandle(req);
    if (con) WinHttpCloseHandle(con);
    if (ses) WinHttpCloseHandle(ses);
    return ok;
}

bool GoogleTileProvider::Init(const std::string& mapType, uint32_t fetchBudget,
                              const DayCaps& dayCaps) {
    m_mapType = mapType;
    m_budget = fetchBudget;
    m_key = ReadKeyFromEnvOrRegistry();
    if (m_key.empty()) {
        Log("[google] GAGAME_GOOGLE_MAPS_KEY not set; Earth keeps the procedural look");
        return false;
    }
    CreateDirectoryA("cache", nullptr);
    CreateDirectoryA("cache\\google", nullptr);
    CreateDirectoryA(("cache\\google\\" + m_mapType).c_str(), nullptr);
    // One ledger for the provider, not one per map type: the owner's cap is on Google, whole.
    m_day.Open(m_cacheRoot, dayCaps, "[google]");
    return EnsureSession();
}

void GoogleTileProvider::InitCacheOnly(const std::string& cacheRoot, const std::string& mapType,
                                       uint32_t fetchBudget, const DayCaps* day,
                                       DayLedger::Clock clock, StandIn standIn) {
    m_cacheRoot = cacheRoot;
    m_mapType = mapType;
    m_budget = fetchBudget;
    m_offline = true;
    m_session = "offline";   // Ready(): the source samples through it like the real one
    m_attribution = "Imagery (c) Google";
    m_standIn = std::move(standIn);
    if (day) m_day.Open(cacheRoot, *day, "[google cache-only]", std::move(clock));
}

bool GoogleTileProvider::EnsureSession() {
    // Session tokens live ~2 weeks: cache and REUSE across runs (the polite path -- one
    // createSession, not one per launch).
    const std::string cachePath = "cache\\google\\session_" + m_mapType + ".json";
    {
        std::ifstream f(cachePath, std::ios::binary);
        if (f) {
            std::string text((std::istreambuf_iterator<char>(f)),
                             std::istreambuf_iterator<char>());
            std::string err;
            const JsonValue v = JsonParser::Parse(text, &err);
            const double expiry = v.Num("expiry", 0);
            const std::string tok = v.Str("session");
            if (!tok.empty() &&
                expiry > static_cast<double>(time(nullptr)) + 3600.0) {
                m_session = tok;
                m_attribution = v.Str("attribution", "Imagery (c) Google");
                Log("[google] session restored from cache (expires in %.1f days)",
                    (expiry - time(nullptr)) / 86400.0);
                return true;
            }
        }
    }
    std::vector<uint8_t> resp;
    const std::string body = "{\"mapType\":\"" + m_mapType +
                             "\",\"language\":\"en-US\",\"region\":\"US\"}";
    if (!Http(L"POST", L"tile.googleapis.com",
              L"/v1/createSession?key=" + std::wstring(m_key.begin(), m_key.end()), body,
              resp)) {
        Log("[google] createSession failed; Earth keeps the procedural look");
        return false;
    }
    std::string err;
    const JsonValue v =
        JsonParser::Parse(std::string(resp.begin(), resp.end()), &err);
    m_session = v.Str("session");
    m_attribution = "Imagery (c) Google";
    const double expiry = atof(v.Str("expiry").c_str());
    std::ofstream f(cachePath, std::ios::binary);
    f << "{\"session\":\"" << m_session << "\",\"expiry\":" << expiry
      << ",\"attribution\":\"Imagery (c) Google\"}";
    Log("[google] session created (%s)", m_mapType.c_str());
    return !m_session.empty();
}

std::string GoogleTileProvider::TilePath(int z, int x, int y) const {
    char path[320];
    snprintf(path, sizeof(path), "%s\\%s\\z%d_x%d_y%d.jpg", m_cacheRoot.c_str(),
             m_mapType.c_str(), z, x, y);
    return path;
}

// The cache file alone: a hit is a file of more than 200 bytes, and costs no network.
bool GoogleTileProvider::CachedTile(int z, int x, int y, std::vector<uint8_t>& jpg) {
    m_cacheOpens.fetch_add(1, std::memory_order_relaxed);
    std::ifstream f(TilePath(z, x, y), std::ios::binary);
    if (!f) return false;
    jpg.assign((std::istreambuf_iterator<char>(f)), std::istreambuf_iterator<char>());
    return jpg.size() > 200;
}

bool GoogleTileProvider::FetchTile(int z, int x, int y, std::vector<uint8_t>& jpg) {
    if (CachedTile(z, x, y, jpg)) return true;   // cache hit: zero network
    const std::string path = TilePath(z, x, y);
    // THE RUN'S OWN REFUSALS FIRST, before the day's ledger is asked: a gate render runs at
    // tileBudget 0 and refuses thousands of tiles, and none of them may take the machine's lock
    // or read the ledger file. (Request asks again under m_mx: two threads can pass this
    // together.)
    {
        std::lock_guard<std::mutex> lk(m_mx);
        if (RefusedForRunLocked(z, x, y)) return false;
    }
    // THE DAY'S CAP (core/DayLedger.h): the ledger is asked before the request and told after
    // it, under the lock every engine on the machine shares, never across it. A day's refusal
    // is the budget's refusal: counted, remembered for the run, and the paint left incomplete
    // (GoogleColorSource::Refusals), so nothing is cached for it.
    std::vector<uint8_t> body;
    const DayFetch got =
        m_day.Fetch([&](std::vector<uint8_t>& b) { return Request(z, x, y, b); }, body);
    {
        std::lock_guard<std::mutex> lk(m_mx);
        if (got == DayFetch::Refused) {
            RefuseLocked(z, x, y);
            return false;
        }
        // THE ROW OF FAILURES: a source that answers every request with an error (an expired
        // session, a quota) is asked kFailRow times in a row a run, not tileBudget times. A tile
        // that lands sets the row back to zero; a request never sent is not in the row.
        if (got == DayFetch::Fetched) m_failRow = 0;
        if (got == DayFetch::Failed && ++m_failRow >= kFailRow && !m_failStopped) {
            m_failStopped = true;
            Log("[google] %u requests in a row were sent and landed nothing -- this run asks no "
                "more (the cache still serves)",
                kFailRow);
        }
    }
    if (got != DayFetch::Fetched) return false;
    jpg.swap(body);   // the response alone: a short cache file read above is not prepended
    std::ofstream f(path, std::ios::binary);
    f.write(reinterpret_cast<const char*>(jpg.data()), jpg.size());
    return true;
}

void GoogleTileProvider::RefuseLocked(int z, int x, int y) {
    // Counted once per distinct tile and remembered (DecodedTile asks m_refused first): the
    // number a zero-budget run prints is the number of fetches it would have made.
    if (m_refused.insert(KeyOf(z, x, y)).second) {
        m_refusedCount.store(static_cast<uint32_t>(m_refused.size()), std::memory_order_relaxed);
    }
}

// The run's own refusals: the budget spent, or the row of failures has stopped the run. Under m_mx.
bool GoogleTileProvider::RefusedForRunLocked(int z, int x, int y) {
    if (m_fetched < m_budget && !m_failStopped) return false;
    RefuseLocked(z, x, y);
    if (m_fetched >= m_budget && !m_budgetLogged) {
        m_budgetLogged = true;
        Log("[google] per-run fetch budget (%u) reached -- serving cache only "
            "(--tile-budget raises it)",
            m_budget);
    }
    return true;
}

// The run's budget, the throttle and the GET. The day's ledger is not touched here: FetchTile's
// rule wraps this whole call, so the ledger's lock is never held across the network.
DaySent GoogleTileProvider::Request(int z, int x, int y, std::vector<uint8_t>& jpg) {
    long long waitMs = 0;
    {
        std::lock_guard<std::mutex> lk(m_mx);
        if (RefusedForRunLocked(z, x, y)) return DaySent::No;
        // Throttle: >= 80 ms between requests, process-wide. Google's limits are far higher;
        // we stay an order of magnitude under on principle (same doctrine as the NOAA side).
        //
        // THE SLEEP USED TO HAPPEN HERE, HOLDING m_mx -- which is also the mutex guarding the
        // decoded-tile LRU, so one thread waiting out its 80 ms locked every other loader
        // thread out of the cache for the same 80 ms. The slot is RESERVED on the shared clock
        // under the lock and waited for outside it. Reserving also spaces concurrent callers 80
        // ms apart instead of letting them all sleep to the same instant and fire together.
        const long long now = NowMs();
        const long long slot = (now > m_lastFetchMs + 80) ? now : m_lastFetchMs + 80;
        waitMs = slot - now;
        m_lastFetchMs = slot;
        ++m_fetched;
        if (m_counter) *m_counter = m_fetched;
    }
    // InitCacheOnly: the network is never touched -- its stand-in answers, or nothing is sent.
    if (m_offline) return m_standIn ? m_standIn(jpg) : DaySent::No;
    if (waitMs > 0) std::this_thread::sleep_for(std::chrono::milliseconds(waitMs));
    // Built whole, not in a fixed buffer: a long session token must not cut the key off the end.
    const std::wstring wpath =
        L"/v1/2dtiles/" + std::to_wstring(z) + L"/" + std::to_wstring(x) + L"/" +
        std::to_wstring(y) + L"?session=" + std::wstring(m_session.begin(), m_session.end()) +
        L"&key=" + std::wstring(m_key.begin(), m_key.end());
    // Any failure after the request was begun counts as sent: the cap errs toward asking less.
    return Http(L"GET", L"tile.googleapis.com", wpath, {}, jpg) ? DaySent::Landed
                                                                : DaySent::Failed;
}

// ONE THREAD AT A TIME FETCHES A TILE. Loader threads paint neighbouring pyramid tiles that need
// the same Google tile; each used to miss the LRU, miss the file and send its own request, and
// the first real run asked for every tile twice. Under m_mx: wait while the tile is in flight on
// another thread, then take what that fetch left; or enter the flight and fetch it. `hit` null:
// the decoded LRU is not looked in (Fetch's path).
GoogleTileProvider::Join GoogleTileProvider::JoinLocked(
    uint64_t key, std::unique_lock<std::mutex>& lk, std::shared_ptr<std::vector<uint8_t>>* hit) {
    bool waited = false;
    for (;;) {
        if (hit) {
            const auto it = m_decoded.find(key);
            if (it != m_decoded.end()) {
                *hit = it->second;
                return Join::Decoded;
            }
        }
        if (m_refused.count(key)) return Join::Refused;   // refused once, refused for the run
        if (!m_inFlight.count(key)) break;
        m_flightWaits.fetch_add(1, std::memory_order_relaxed);
        m_flightDone.wait(lk);   // no lock held across the network: the fetcher is outside it
        waited = true;
    }
    if (waited) return Join::Joined;
    m_inFlight.insert(key);
    return Join::Entered;
}

void GoogleTileProvider::LeaveFlight(uint64_t key) {
    {
        std::lock_guard<std::mutex> lk(m_mx);
        m_inFlight.erase(key);
    }
    m_flightDone.notify_all();
}

bool GoogleTileProvider::Fetch(int z, int x, int y, std::vector<uint8_t>& jpg, bool joinFlight) {
    if (!joinFlight) return FetchTile(z, x, y, jpg);   // the plant: every thread asks for itself
    const uint64_t key = KeyOf(z, x, y);
    Join j;
    {
        std::unique_lock<std::mutex> lk(m_mx);
        j = JoinLocked(key, lk, nullptr);
    }
    if (j == Join::Refused) return false;
    if (j == Join::Joined) return CachedTile(z, x, y, jpg);   // its file, or its failure
    struct Leave { GoogleTileProvider* p; uint64_t k; ~Leave() { p->LeaveFlight(k); } } leave{this, key};
    return FetchTile(z, x, y, jpg);
}

// Decode (or serve from the small in-memory LRU) one 256x256 google tile as RGBA.
std::shared_ptr<std::vector<uint8_t>> GoogleTileProvider::DecodedTile(int z, int x, int y) {
    const uint64_t key = KeyOf(z, x, y);
    std::shared_ptr<std::vector<uint8_t>> hit;
    Join j;
    {
        std::unique_lock<std::mutex> lk(m_mx);
        j = JoinLocked(key, lk, &hit);
    }
    std::vector<uint8_t> jpg;
    if (j == Join::Decoded) return hit;
    if (j == Join::Refused) return nullptr;
    if (j == Join::Joined) {
        // The fetch this thread waited on is over and left nothing in the LRU: its file, or its
        // failure -- never a request of this thread's own in that same moment.
        return CachedTile(z, x, y, jpg) ? Keep(key, jpg) : nullptr;
    }
    // Entered. On every way out -- landed, failed, refused, decode failed -- the tile leaves the
    // flight and the waiters wake, after Keep has put it in the LRU for them.
    struct Leave { GoogleTileProvider* p; uint64_t k; ~Leave() { p->LeaveFlight(k); } } leave{this, key};
    if (!FetchTile(z, x, y, jpg)) return nullptr;
    return Keep(key, jpg);
}

// WIC decode on this worker thread (COM per-thread init), kept in the small LRU.
std::shared_ptr<std::vector<uint8_t>> GoogleTileProvider::Keep(uint64_t key,
                                                               std::vector<uint8_t>& jpg) {
    thread_local bool comInit = false;
    if (!comInit) {
        CoInitializeEx(nullptr, COINIT_MULTITHREADED);
        comInit = true;
    }
    Com<IWICImagingFactory> fac;
    if (FAILED(CoCreateInstance(CLSID_WICImagingFactory, nullptr, CLSCTX_INPROC_SERVER,
                                IID_PPV_ARGS(&fac)))) {
        return nullptr;
    }
    Com<IWICStream> stream;
    fac->CreateStream(&stream);
    stream->InitializeFromMemory(jpg.data(), static_cast<DWORD>(jpg.size()));
    Com<IWICBitmapDecoder> dec;
    if (FAILED(fac->CreateDecoderFromStream(stream.Get(), nullptr,
                                            WICDecodeMetadataCacheOnDemand, &dec))) {
        return nullptr;
    }
    Com<IWICBitmapFrameDecode> frame;
    dec->GetFrame(0, &frame);
    Com<IWICFormatConverter> conv;
    fac->CreateFormatConverter(&conv);
    if (FAILED(conv->Initialize(frame.Get(), GUID_WICPixelFormat32bppRGBA,
                                WICBitmapDitherTypeNone, nullptr, 0.0,
                                WICBitmapPaletteTypeCustom))) {
        return nullptr;
    }
    auto px = std::make_shared<std::vector<uint8_t>>(256 * 256 * 4);
    if (FAILED(conv->CopyPixels(nullptr, 256 * 4, static_cast<UINT>(px->size()), px->data()))) {
        return nullptr;
    }
    std::lock_guard<std::mutex> lk(m_mx);
    const auto kept = m_decoded.find(key);
    if (kept != m_decoded.end()) return kept->second;   // a joined thread kept it first
    if (m_decoded.size() >= 96) {   // small LRU: neighbours reuse heavily during reprojection
        m_decoded.erase(m_decodedOrder.front());
        m_decodedOrder.erase(m_decodedOrder.begin());
    }
    m_decoded[key] = px;
    m_decodedOrder.push_back(key);
    return px;
}

// M6i: the reprojection closures (Fn / DetailFn) and the hardware-cube convention moved to the
// layer compositor (src/compose/) -- addressing belongs to a channel's REALIZATION, not to any
// one source. This class now ends at Decoded(): fetch, decode, cache, throttle.

}  // namespace ga
