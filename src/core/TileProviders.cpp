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
        case DXGI_FORMAT_BC1_UNORM: m_tileW = 512; m_tileH = 256; break;
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
    return [this](const TileRequest& r, std::vector<uint8_t>& out) {
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

// Minimal WinHTTP request. Returns false on any transport or non-200 status.
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
                for (;;) {
                    DWORD avail = 0;
                    if (!WinHttpQueryDataAvailable(req, &avail) || avail == 0) break;
                    const size_t base = out.size();
                    out.resize(base + avail);
                    DWORD got = 0;
                    if (!WinHttpReadData(req, out.data() + base, avail, &got)) break;
                    out.resize(base + got);
                }
                ok = !out.empty();
            } else {
                Log("[google] HTTP %lu on %S", status, path.substr(0, 60).c_str());
            }
        }
    }
    if (req) WinHttpCloseHandle(req);
    if (con) WinHttpCloseHandle(con);
    if (ses) WinHttpCloseHandle(ses);
    return ok;
}

bool GoogleTileProvider::Init(const std::string& mapType, uint32_t fetchBudget) {
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
    return EnsureSession();
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

bool GoogleTileProvider::FetchTile(int z, int x, int y, std::vector<uint8_t>& jpg) {
    char path[256];
    snprintf(path, sizeof(path), "cache\\google\\%s\\z%d_x%d_y%d.jpg", m_mapType.c_str(), z, x,
             y);
    {
        std::ifstream f(path, std::ios::binary);
        if (f) {
            jpg.assign((std::istreambuf_iterator<char>(f)), std::istreambuf_iterator<char>());
            if (jpg.size() > 200) return true;   // cache hit: zero network
        }
    }
    {
        std::lock_guard<std::mutex> lk(m_mx);
        if (m_fetched >= m_budget) {
            if (!m_budgetLogged) {
                m_budgetLogged = true;
                Log("[google] per-run fetch budget (%u) reached -- serving cache only "
                    "(--tile-budget raises it)",
                    m_budget);
            }
            return false;
        }
        // Throttle: >= 80 ms between requests, process-wide. Google's limits are far higher;
        // we stay an order of magnitude under on principle (same doctrine as the NOAA side).
        const long long now = NowMs();
        if (now - m_lastFetchMs < 80) {
            std::this_thread::sleep_for(std::chrono::milliseconds(80 - (now - m_lastFetchMs)));
        }
        m_lastFetchMs = NowMs();
        ++m_fetched;
        if (m_counter) *m_counter = m_fetched;
    }
    wchar_t wpath[256];
    swprintf(wpath, 256, L"/v1/2dtiles/%d/%d/%d?session=%S&key=%S", z, x, y, m_session.c_str(),
             m_key.c_str());
    if (!Http(L"GET", L"tile.googleapis.com", wpath, {}, jpg)) return false;
    std::ofstream f(path, std::ios::binary);
    f.write(reinterpret_cast<const char*>(jpg.data()), jpg.size());
    return true;
}

// Decode (or serve from the small in-memory LRU) one 256x256 google tile as RGBA.
std::shared_ptr<std::vector<uint8_t>> GoogleTileProvider::DecodedTile(int z, int x, int y) {
    const uint64_t key = (static_cast<uint64_t>(z) << 48) | (static_cast<uint64_t>(x) << 24) |
                         static_cast<uint64_t>(y);
    {
        std::lock_guard<std::mutex> lk(m_mx);
        auto it = m_decoded.find(key);
        if (it != m_decoded.end()) return it->second;
    }
    std::vector<uint8_t> jpg;
    if (!FetchTile(z, x, y, jpg)) return nullptr;

    // WIC decode on this worker thread (COM per-thread init).
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
