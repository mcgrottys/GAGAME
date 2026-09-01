// ================================================================================================
//  TileAtlas - M0: reserved (tiled) resources, and the self-test that proves the platform
//  guarantees the whole gameplan leans on.
//
//  THE LOAD-BEARING CLAIM (D3D12 docs, tiled resources Tier 2, verified 2026-08-28):
//      "Reading from NULL-mapped tiles treat that sampled value as zero.
//       Writes to NULL-mapped tiles are discarded."
//  and: adapters supporting feature level 12_0 all support TIER_2 or greater.
//
//  The gameplan's tiled multivector atlas stores grade banks in reserved resources where a
//  NULL-mapped tile MEANS "this component is identically zero here". The composite shaders then
//  add sparse contributions unconditionally -- no branches -- and the sim dispatches only over
//  resident tiles (because null-tile WRITES vanish silently; that is the one hazard).
//
//  RunTileSelfTest verifies empirically, on this machine, that:
//    1. a checkerboard of mapped/NULL tiles reads back: pattern on mapped, EXACTLY 0.0 on null,
//       through both the Load path and the SampleLevel path;
//    2. writes into NULL tiles are discarded (the write pass covers the whole surface);
//    3. CheckAccessFullyMapped() reports residency truthfully (the shader-side residency query);
//    4. unmap -> remap round-trips: an unmapped tile reads 0, a freshly remapped tile carries the
//       next write;
//    5. (Tier 3 only) the same holds for a 3D reserved resource, which the volume grade banks
//       would use.
//
//  M4 grows this file into the real atlas (occupancy quadtree, batched UpdateTileMappings, heap
//  pool, grade-signature Cayley skipping). M0 keeps it to the proof.
// ================================================================================================
#pragma once

#include "core/Gpu.h"
#include "core/Shader.h"

#include <string>
#include <vector>

namespace ga {

// Runs the whole self-test suite. Logs a detailed report; returns true only if every check on
// every tile passed. Requires shaders/TileTest.hlsl under shaderDir.
bool RunTileSelfTest(Gpu& gpu, ShaderCompiler& sc, const std::wstring& shaderDir);

// ================================================================================================
//  Grade signatures (M4): one bit per grade of Cl(2) -- bit0 = grade 0 (scalar), bit1 = grade 1
//  (vector), bit2 = grade 2 (bivector). A tile's signature says which grade banks are non-zero
//  there; the geometric product's OUTPUT signature follows from the algebra's grade structure,
//  so sparsity propagates through products without looking at any data. This is the atlas's
//  "null tiles prune Cayley blocks" machinery, verified against brute-force blade products in
//  the self-test.
// ================================================================================================
constexpr uint8_t kCl2GradeMul[3][3] = {
    // g0*gX          g1*gX                g2*gX
    {0b001, 0b010, 0b100},   // times g0: grades pass through
    {0b010, 0b101, 0b010},   // times g1: g1*g1 = g0 + g2 (dot + wedge), g1*g2 = g1
    {0b100, 0b010, 0b001},   // times g2: g2*g2 = -1 (a scalar)
};

// M9h: constexpr so the TYPE algebra can run at compile time -- Field<A> * Field<B> resolves
// to Field<Cl2ProductSignature(A, B)> with no runtime check and no declared result type. The
// body is unchanged and still the selftest-pinned closure.
constexpr uint8_t Cl2ProductSignature(uint8_t a, uint8_t b) {
    uint8_t out = 0;
    for (int i = 0; i < 3; ++i) {
        if (!(a & (1 << i))) continue;
        for (int j = 0; j < 3; ++j) {
            if (b & (1 << j)) out |= kCl2GradeMul[i][j];
        }
    }
    return out;
}

// ================================================================================================
//  TileAtlas2D (M4): one grade bank -- a reserved (tiled) 2D texture over a large virtual
//  domain, with CPU-side residency. NULL tiles read as zero (Tier 2), which the atlas treats as
//  MEANING: "this quantity is identically zero here". The composite shaders sample without
//  branching; simulation dispatches walk the resident-tile list only (null-tile writes would be
//  silently discarded -- the one hazard, designed around by construction).
//
//  Residency flow per frame: RequestMap / RequestUnmap -> CommitMappings (batched
//  UpdateTileMappings, heap-pool backed) -> caller CLEARS newly mapped tiles (their content is
//  undefined) -> simulation dispatch over ResidentList().
// ================================================================================================
class TileAtlas2D {
public:
    // mipLevels > 1 builds THE CHAIN (M9h). One reserved resource, one SRV, one shader path at
    // every altitude: zoom changes which tiles are resident, never which code runs. That is the
    // whole reason the chain lives inside the resource instead of beside it.
    void Init(Gpu& gpu, uint32_t widthTexels, uint32_t heightTexels, DXGI_FORMAT fmt,
              const wchar_t* name, uint32_t heapChunkTiles = 64, uint32_t mipLevels = 1);

    uint32_t TilesX() const { return m_tilesX; }          // mip 0
    uint32_t TilesY() const { return m_tilesY; }
    uint32_t TileW() const { return m_tileW; }
    uint32_t TileH() const { return m_tileH; }
    bool IsResident(uint32_t tx, uint32_t ty) const { return IsResident(0, tx, ty); }

    void RequestMap(uint32_t tx, uint32_t ty) { RequestMap(0, tx, ty); }
    void RequestUnmap(uint32_t tx, uint32_t ty) { RequestUnmap(0, tx, ty); }
    void RequestUnmapAll();

    // ---- the chain (M9h) --------------------------------------------------------------------
    // STANDARD mips are tiled per tile. The PACKED tail -- the mips small enough that D3D12
    // packs them into shared tiles -- is mapped ONCE at Init and never evicted: it is a handful
    // of tiles, it is the coarsest description of the whole field, and keeping it resident is
    // what guarantees a sample can never miss entirely. Misses degrade to blur, never to
    // garbage, and never to a pop.
    uint32_t MipCount() const { return m_mipCount; }             // standard + packed
    uint32_t StandardMips() const { return m_standardMips; }     // per-tile mappable
    uint32_t TilesX(uint32_t mip) const { return m_mip[mip].tilesX; }
    uint32_t TilesY(uint32_t mip) const { return m_mip[mip].tilesY; }
    bool IsResident(uint32_t mip, uint32_t tx, uint32_t ty) const {
        if (mip >= m_standardMips) return true;                  // packed tail: always mapped
        const MipInfo& m = m_mip[mip];
        return (tx < m.tilesX && ty < m.tilesY) && m_state[m.base + ty * m.tilesX + tx] == 1;
    }
    void RequestMap(uint32_t mip, uint32_t tx, uint32_t ty);
    void RequestUnmap(uint32_t mip, uint32_t tx, uint32_t ty);

    // The finest mip resident over a mip-0 tile region, or MipCount()-1 when only the packed
    // tail covers it. This is the number the shader clamps its LOD to.
    uint32_t FinestResident(uint32_t tx0, uint32_t ty0) const;

    // R8 residency map, one texel per mip-0 tile, value = FinestResident. Rebuilt by
    // CommitMappings; UINT32_MAX until a chain is built.
    uint32_t ResidencyMapSrv() const { return m_resMapSrv; }

    // Executes the batched UpdateTileMappings. Newly mapped tile indices (packed ty*tilesX+tx)
    // are appended to outNewlyMapped: their contents are UNDEFINED until cleared.
    void CommitMappings(Gpu& gpu, std::vector<uint32_t>* outNewlyMapped);

    const std::vector<uint32_t>& ResidentList() const { return m_residentList; }
    uint32_t ResidentCount() const { return static_cast<uint32_t>(m_residentList.size()); }
    uint64_t ResidentBytes() const { return ResidentCount() * kTileBytes; }
    uint64_t VirtualBytes() const {
        return static_cast<uint64_t>(m_tilesX) * m_tilesY * kTileBytes;
    }

    ID3D12Resource* Res() const { return m_res.Get(); }
    uint32_t Srv() const { return m_srv; }
    uint32_t Uav() const { return m_uav; }
    uint32_t Uav(uint32_t mip) const {
        return (mip < m_mipUav.size()) ? m_mipUav[mip] : UINT32_MAX;
    }

    // Map every STANDARD coarse tile (mip >= 1). Coarse levels together cost about a third of
    // mip 0 and they are the global floor, so completeness there is worth more than sparsity:
    // a hole in a coarse level is a hole no finer level can cover.
    void MapAllCoarse();

    // Fill the chain by 2x2 reduction, mip 0 upward, each level written through its own UAV.
    // Needs shaders/MipReduce.hlsl; safe to call every time the fine level changes.
    void BuildMips(Gpu& gpu, ShaderCompiler& sc, const std::wstring& shaderDir,
                   ID3D12GraphicsCommandList* cl);

    static constexpr uint64_t kTileBytes = D3D12_TILED_RESOURCE_TILE_SIZE_IN_BYTES;

private:
    Com<ID3D12Resource> m_res;
    std::vector<Com<ID3D12Heap>> m_heaps;
    uint32_t m_heapChunkTiles = 64;
    std::vector<uint32_t> m_freeTiles;        // (heapIdx << 16) | tileInHeap
    std::vector<uint32_t> m_tilePool;         // per atlas tile: pool slot when resident
    std::vector<uint8_t> m_state;             // 0 null, 1 resident
    std::vector<uint32_t> m_residentList;
    std::vector<uint32_t> m_pendingMap, m_pendingUnmap;
    uint32_t m_tilesX = 0, m_tilesY = 0, m_tileW = 0, m_tileH = 0;
    uint32_t m_srv = UINT32_MAX, m_uav = UINT32_MAX;

    // ---- the chain -------------------------------------------------------------------------
    struct MipInfo {
        uint32_t tilesX = 0, tilesY = 0;
        uint32_t base = 0;           // offset into m_state / m_tilePool
        uint32_t startTile = 0;      // first tile index within the resource
    };
    void Decode(uint32_t i, uint32_t& mip, uint32_t& tx, uint32_t& ty) const;
    void RebuildResidencyMap(Gpu& gpu);

    std::vector<MipInfo> m_mip;
    uint32_t m_mipCount = 1, m_standardMips = 1;
    uint32_t m_pinnedFloor = 0;      // the always-resident level: packed tail, or, when a
                                     // shape has no packed mips at all, the coarsest standard
    std::vector<uint32_t> m_mipUav;  // one UAV per mip, for the reduction's destination
    DXGI_FORMAT m_fmt = DXGI_FORMAT_UNKNOWN;
    uint32_t m_mipTable = UINT32_MAX;
    Com<ID3D12RootSignature> m_mipRs;
    Com<ID3D12PipelineState> m_mipPso;
    GpuTexture m_resMap;             // R8_UINT, tilesX(0) x tilesY(0)
    uint32_t m_resMapSrv = UINT32_MAX;
    std::vector<uint8_t> m_resMapCpu;
    bool m_resMapDirty = false;
};

// ================================================================================================
//  TileAtlas3D (M6c): the VOLUME grade bank -- a reserved 3D texture (tiled-resources tier 3;
//  this GPU reports tier 4, and the M0 self-test proved the 3D null-tile contract live) with
//  the same CPU residency machinery as the 2D bank. First tenant: the cloud volume -- a NULL
//  tile IS clear air. Tile shape for R16F is 32x32x16 texels (64 KB).
// ================================================================================================
class TileAtlas3D {
public:
    void Init(Gpu& gpu, uint32_t w, uint32_t h, uint32_t d, DXGI_FORMAT fmt,
              const wchar_t* name, uint32_t heapChunkTiles = 64);

    uint32_t TilesX() const { return m_tilesX; }
    uint32_t TilesY() const { return m_tilesY; }
    uint32_t TilesZ() const { return m_tilesZ; }
    uint32_t TileW() const { return m_tileW; }
    uint32_t TileH() const { return m_tileH; }
    uint32_t TileD() const { return m_tileD; }
    bool IsResident(uint32_t tx, uint32_t ty, uint32_t tz) const {
        return m_state[(static_cast<size_t>(tz) * m_tilesY + ty) * m_tilesX + tx] == 1;
    }

    void RequestMap(uint32_t tx, uint32_t ty, uint32_t tz);
    void RequestUnmapAll();
    void CommitMappings(Gpu& gpu, std::vector<uint32_t>* outNewlyMapped);

    const std::vector<uint32_t>& ResidentList() const { return m_residentList; }
    uint32_t ResidentCount() const { return static_cast<uint32_t>(m_residentList.size()); }
    uint64_t ResidentBytes() const { return ResidentCount() * TileAtlas2D::kTileBytes; }
    uint64_t VirtualBytes() const {
        return static_cast<uint64_t>(m_tilesX) * m_tilesY * m_tilesZ * TileAtlas2D::kTileBytes;
    }

    ID3D12Resource* Res() const { return m_res.Get(); }
    uint32_t Srv() const { return m_srv; }
    uint32_t Uav() const { return m_uav; }

private:
    Com<ID3D12Resource> m_res;
    std::vector<Com<ID3D12Heap>> m_heaps;
    uint32_t m_heapChunkTiles = 64;
    std::vector<uint32_t> m_freeTiles;
    std::vector<uint32_t> m_tilePool;
    std::vector<uint8_t> m_state;
    std::vector<uint32_t> m_residentList;
    std::vector<uint32_t> m_pendingMap;
    std::vector<D3D12_TILED_RESOURCE_COORDINATE> m_pendingNull;
    uint32_t m_tilesX = 0, m_tilesY = 0, m_tilesZ = 0;
    uint32_t m_tileW = 0, m_tileH = 0, m_tileD = 0;
    uint32_t m_srv = UINT32_MAX, m_uav = UINT32_MAX;
};

// M4 self-tests: the Cayley signature closure vs brute-force blade products, and a live
// map/clear/write/unmap cycle through a small atlas. Requires shaders/SeaChurn.hlsl.
bool RunAtlasSelfTest(Gpu& gpu, ShaderCompiler& sc, const std::wstring& shaderDir);

}  // namespace ga
