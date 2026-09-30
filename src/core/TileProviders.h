// ================================================================================================
//  TileProviders - M6e: the two sources feeding the ResidencyManager's texture tenants.
//
//  MarsBinProvider: the extinct D3D11 TiledResources sample's diffuse.bin / normal.bin (the
//  user's rescued copy; MOLA/Viking Mars). Format decoded from the sample's own TileLoader:
//  headerless 64KB tiles, face-major then mip-major, row-major tiles within a mip, block-linear
//  within a tile -- exactly what CopyTiles(LINEAR_BUFFER_TO_SWIZZLED) ingests. Reads are
//  OVERLAPPED-at-offset so worker threads share one handle without seek races.
//
//  GoogleTileProvider: Earth imagery via the Map Tiles API (2D session tiles), CACHE-FIRST and
//  polite by construction: every fetched tile lands in cache/google/ forever (offline replays
//  hit zero network), the session token is cached and reused across runs (~2-week validity),
//  fetches are throttled (>= 80 ms apart), hard-capped per run (default 1000; the cap
//  logging tells you when the budget bit) and capped per UTC day over every engine on the
//  machine (core/DayLedger.h: streaming.dayTiles / dayBytes, cache/google/day_<date>.json). M6i: reprojection moved OUT -- this class is now a
//  pure Mercator-tree backend (fetch + decode + LRU); GoogleColorSource wraps it for the layer
//  compositor, whose realizations own all addressing. Requires GAGAME_GOOGLE_MAPS_KEY (env or
//  HKCU); without it the provider reports unavailable and Earth keeps its procedural look.
// ================================================================================================
#pragma once

#include "core/DayLedger.h"
#include "hal/Residency.h"

#include <atomic>
#include <condition_variable>
#include <map>
#include <mutex>
#include <set>
#include <string>

namespace ga {

// WHAT A LOG LINE MAY SAY OF A REQUEST: its path BEFORE the '?', and never any part of the query,
// which holds the key and the session's token. Every line of the fetch path that names a request
// names it through this; [daytest] holds it to that, and catches the old form (the first 60
// characters of path and query, which for createSession was the key, all but one character).
std::string LoggablePath(const std::wstring& pathAndQuery);

class MarsBinProvider {
public:
    // fmt selects the layout: BC1 (512x256 tiles) or BC5 (256x256). faceDim always 16384.
    bool Open(const std::wstring& binPath, DXGI_FORMAT fmt);
    TileProviderFn Fn();
    bool Ready() const { return m_file != nullptr; }

private:
    void* m_file = nullptr;   // HANDLE; OVERLAPPED reads at absolute offsets
    uint64_t m_tilesPerFace = 0;
    uint32_t m_tileW = 0, m_tileH = 0;
    std::vector<uint64_t> m_mipPrefix;   // tile index of each mip's start within a face
    std::vector<uint32_t> m_mipTilesW, m_mipTilesH;
};

class GoogleTileProvider {
public:
    // mapType: "satellite" (imagery). Reads the key, opens the day's ledger (and logs what it
    // holds against the day's caps), restores or creates the tile session.
    bool Init(const std::string& mapType, uint32_t fetchBudget, const DayCaps& dayCaps);
    // The selftest's provider: tiles from `cacheRoot`\<mapType>\z_x_y.jpg and from nowhere
    // else. No key is read, no session made, and no connection is ever opened: a tile the
    // folder lacks is refused when the budget is spent and fails when it is not. With `day`,
    // it also keeps a day ledger in `cacheRoot` on `clock`, and `standIn` answers in place of
    // the network -- so [daytest] drives this provider's own rule, row and budget.
    using StandIn = std::function<DaySent(std::vector<uint8_t>& body)>;
    void InitCacheOnly(const std::string& cacheRoot, const std::string& mapType,
                       uint32_t fetchBudget, const DayCaps* day = nullptr,
                       DayLedger::Clock clock = {}, StandIn standIn = {});
    bool Ready() const { return !m_session.empty(); }
    const std::string& Attribution() const { return m_attribution; }
    void SetFetchCounter(uint32_t* counter) { m_counter = counter; }
    // One decoded 256x256 RGBA Mercator tile (cache-first, throttled, budget-capped); nullptr
    // on budget exhaustion or transport failure. shared_ptr on purpose: workers hold decoded
    // tiles across LRU evictions by other workers (a raw pointer here was a use-after-free
    // that deadlocked the first warm run).
    std::shared_ptr<std::vector<uint8_t>> Decoded(int z, int x, int y) {
        return DecodedTile(z, x, y);
    }
    // THE FETCHES A LARGER BUDGET WOULD HAVE MADE: the distinct source tiles (z, x, y) this run
    // asked for, did not find in the cache, and refused because streaming.tileBudget was
    // spent, the day's cap was met (streaming.dayTiles / dayBytes) or kFailRow requests in a
    // row had failed. A refusal is remembered for the run -- the budget never comes back -- so
    // a tile is asked once, not once per texel that falls in it (16384 failed opens a painted
    // tile).
    uint32_t Refused() const { return m_refusedCount.load(std::memory_order_relaxed); }
    uint32_t Fetched() const { return m_fetched; }
    uint32_t Budget() const { return m_budget; }
    // Cache files this provider tried to open, hit or miss (the selftest's count of asks).
    uint32_t CacheOpens() const { return m_cacheOpens.load(std::memory_order_relaxed); }
    // The day ledger's reads by the rule: zero for a run whose every fetch the budget refused.
    uint32_t DayReads() const { return m_day.Reads(); }
    // What Decoded() does before it decodes -- the flight, cache, the run's refusals, the day's
    // rule, the request -- without the decode: the selftest's handle, which must not start COM.
    // joinFlight = false is the rule with the in-flight set taken out ([daytest]'s plant).
    bool Fetch(int z, int x, int y, std::vector<uint8_t>& jpg, bool joinFlight = true);
    // Times a thread waited for another thread's fetch of the same tile.
    uint32_t FlightWaits() const { return m_flightWaits.load(std::memory_order_relaxed); }
    // Requests in a row that were sent and landed nothing.
    uint32_t FailRow() {
        std::lock_guard<std::mutex> lk(m_mx);
        return m_failRow;
    }
    // A RUN STOPS ASKING after this many requests in a row were sent and landed nothing.
    static constexpr uint32_t kFailRow = 8;

private:
    // ONE THREAD AT A TIME FETCHES A TILE. Under m_mx: the tile is decoded, refused, joined (it
    // was in flight on another thread, which has finished: take what that fetch left -- its
    // file, or its failure) or entered (this thread fetches it; LeaveFlight on every way out).
    enum class Join { Decoded, Refused, Joined, Entered };
    Join JoinLocked(uint64_t key, std::unique_lock<std::mutex>& lk,
                    std::shared_ptr<std::vector<uint8_t>>* hit);
    void LeaveFlight(uint64_t key);
    bool CachedTile(int z, int x, int y, std::vector<uint8_t>& jpg);   // the file alone
    std::string TilePath(int z, int x, int y) const;
    std::shared_ptr<std::vector<uint8_t>> Keep(uint64_t key, std::vector<uint8_t>& jpg);
    bool EnsureSession();
    bool FetchTile(int z, int x, int y, std::vector<uint8_t>& jpg);
    DaySent Request(int z, int x, int y, std::vector<uint8_t>& jpg);   // budget, throttle, GET
    void RefuseLocked(int z, int x, int y);                             // under m_mx
    bool RefusedForRunLocked(int z, int x, int y);   // budget spent or row stopped; under m_mx
    std::shared_ptr<std::vector<uint8_t>> DecodedTile(int z, int x, int y);
    static uint64_t KeyOf(int z, int x, int y) {
        return (static_cast<uint64_t>(z) << 48) | (static_cast<uint64_t>(x) << 24) |
               static_cast<uint64_t>(y);
    }

    std::string m_key, m_session, m_mapType = "satellite", m_attribution;
    std::string m_cacheRoot = "cache\\google";
    bool m_offline = false;              // InitCacheOnly: no request is ever sent
    uint32_t m_budget = 1000;
    uint32_t m_fetched = 0;
    bool m_budgetLogged = false;
    long long m_lastFetchMs = 0;
    std::mutex m_mx;                     // guards cache map, refusals + throttle clock
    std::map<uint64_t, std::shared_ptr<std::vector<uint8_t>>> m_decoded;
    std::vector<uint64_t> m_decodedOrder;
    std::set<uint64_t> m_refused;        // KeyOf of every tile refused for the budget
    std::atomic<uint32_t> m_refusedCount{0}, m_cacheOpens{0};
    uint32_t* m_counter = nullptr;
    uint32_t m_failRow = 0;              // requests in a row sent that landed nothing; under m_mx
    bool m_failStopped = false;          // kFailRow reached: the run asks no more; under m_mx
    StandIn m_standIn;                   // InitCacheOnly's stand-in for the network, or none
    std::set<uint64_t> m_inFlight;       // KeyOf of every tile being fetched now; under m_mx
    std::condition_variable m_flightDone;   // woken whenever a tile leaves m_inFlight
    std::atomic<uint32_t> m_flightWaits{0};
    DayLedger m_day;                     // opened by Init (and by InitCacheOnly given `day`)
};

}  // namespace ga
