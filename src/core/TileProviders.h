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
//  fetches are throttled (>= 80 ms apart) and hard-capped per run (default 1000; the cap
//  logging tells you when the budget bit). Web-Mercator tiles are decoded (WIC) and
//  reprojected onto OUR cube faces (CubeDir convention) at fetch time, so the renderer sees
//  one uniform planet-texture scheme for both worlds. Requires GAGAME_GOOGLE_MAPS_KEY (env or
//  HKCU); without it the provider reports unavailable and Earth keeps its procedural look.
// ================================================================================================
#pragma once

#include "core/Residency.h"

#include <map>
#include <mutex>
#include <string>

namespace ga {

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
    // mapType: "satellite" (imagery). Reads the key, restores or creates the tile session.
    bool Init(const std::string& mapType, uint32_t fetchBudget);
    TileProviderFn Fn(uint32_t* fetchCounter);
    // M6f: a Mercator-aligned detail WINDOW (the Merrimack pyramid). The window is a 64x64
    // block of z14 tiles (16384 px, ~9.5 m/px at this latitude); mip m maps 1:1 onto zoom
    // 14-m, so tile fills are straight pixel copies -- no resampling, no trig.
    TileProviderFn DetailFn(uint32_t* fetchCounter, long long orgPxX, long long orgPxY);
    bool Ready() const { return !m_session.empty(); }
    const std::string& Attribution() const { return m_attribution; }

private:
    bool EnsureSession();
    bool FetchTile(int z, int x, int y, std::vector<uint8_t>& jpg);
    // shared_ptr on purpose: workers hold decoded tiles across LRU evictions by other workers
    // (a raw pointer here was a use-after-free that deadlocked the first warm run).
    std::shared_ptr<std::vector<uint8_t>> DecodedTile(int z, int x, int y);

    std::string m_key, m_session, m_mapType = "satellite", m_attribution;
    uint32_t m_budget = 1000;
    uint32_t m_fetched = 0;
    bool m_budgetLogged = false;
    long long m_lastFetchMs = 0;
    std::mutex m_mx;                     // guards cache map + throttle clock
    std::map<uint64_t, std::shared_ptr<std::vector<uint8_t>>> m_decoded;
    std::vector<uint64_t> m_decodedOrder;
    uint32_t* m_counter = nullptr;
};

}  // namespace ga
