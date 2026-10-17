// ================================================================================================
//  MaxLane.h - THE MAX LANE OF A SCALAR PAGE TENANT (shaders/MaxLane.hlsl; TerrainRay.hlsli's cast).
//
//  A fold keeps what its readers must not lose. A scalar tenant's mips are coverage-weighted means,
//  which a picture wants and a ray cannot use: "can anything under this cell rise above me" is
//  answered by a MAXIMUM alone, and "is this cell certainly above me" by a MINIMUM. The lane is that
//  pair, kept beside the tenant on the GPU: two half floats for each 16 x 16 block of every mip of
//  every slice, the largest and the smallest value any bilinear read at that mip, or a finer resident
//  one, can return inside the block. It is a RESERVED R16G16 Texture2DArray at 1/16 of the tenant's
//  side, 7 mips, a slice for each of the tenant's, backed tile by tile only where a held tile of the
//  tenant lies under it (Hold: one 64 KB lane tile covers 8 x 16 tenant tiles), and rebuilt, tile by
//  tile, from the tiles the residency turn changed.
//
//  WHY ON THE GPU AND NOT IN THE TREE. A cached height tile's coarse mip is often its source's own
//  resample, not a fold of finer tiles (TileTree.h: "a coarse mip is the source's own resample where
//  nothing finer has been painted"), and a Half root composes rather than folds; a maximum folded
//  on the CPU would bound only where every leaf below had been painted, which a lazily painted
//  pyramid never has. What the reader reads is what is RESIDENT, and a bound of that is built from
//  the resident pages themselves, as they land: tight, exact, and with no change to any cached byte.
//
//  The jobs (one a tile) are made by the residency manager, which alone knows what is held, the
//  aliases of an address and the pyramid's addresses (ResidencyManager::RecordLanes); this class
//  owns the texture, its views and the kernel, and records the jobs level by level, finest first,
//  with a UAV barrier between levels because a level's fold reads the lanes of the level below.
// ================================================================================================
#pragma once

#include "hal/Gpu.h"

#include <cstdint>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace ga {

class ShaderCompiler;

class MaxLane {
public:
    static constexpr uint32_t kBlock = 16;   // MIRRORS shaders/MaxLane.hlsl kBlock, TerrainRay kCastBlock
    static constexpr uint32_t kMaxMips = 8;
    struct Job {
        uint32_t slice = 0, mip = 0, x = 0, y = 0;
        uint32_t wrap = 0;           // 1: a window's page, addressed modulo itself; 0: a cube face
        uint32_t child[4][4];        // slice, mip, x, y of the tile holding each quarter's children; slice ~0 = none
        uint32_t level = 0;          // the pyramid's mip (finer is smaller): the order of the fold
    };
    // `tenantSrv` is the tenant's Texture2DArray SRV (its slices and mips). False, and the lane
    // stays off, when the device cannot load R16F through a UAV or the kernel did not build.
    bool Init(Gpu& gpu, ShaderCompiler& sc, const std::wstring& shaderDir, const std::wstring& name,
              uint32_t slices, uint32_t faceDim, uint32_t mips, uint32_t texW, uint32_t texH,
              uint32_t tenantSrv);
    // A tenant slot became held (true) or let go (false): the lane tile over it is backed while any
    // held slot lies under it. Call before Record, for every slot whose held state changed.
    void Hold(Gpu& gpu, uint32_t slice, uint32_t mip, uint32_t x, uint32_t y, bool held);
    // Record the turn's jobs on the frame's list (the heap bound), finest level first. Sorts `jobs`.
    void Record(Gpu& gpu, ID3D12GraphicsCommandList* cl, std::vector<Job>& jobs);
    bool Ready() const { return m_pso != nullptr; }
    uint32_t Srv() const { return m_srv; }
    // The ledger: jobs recorded in all, the turns that recorded any, the last turn's jobs and levels.
    uint64_t jobsTotal = 0, turns = 0, overflowTurns = 0;
    uint32_t lastJobs = 0, lastLevels = 0;
    uint32_t pagesHeld = 0;   // 64 KB pool pages backing the lane now

private:
    // The job as the kernel reads it (shaders/MaxLane.hlsl LaneJob): 96 bytes.
    struct Row {
        uint32_t job[4];
        uint32_t page[4];
        uint32_t child[4][4];
    };
    static constexpr uint32_t kMaxJobs = 16384;   // 1.5 MB a frame in flight
    static constexpr uint32_t kChunkTiles = 64;   // pool pages a heap (4 MB)
    struct Tile {
        uint32_t count = 0;            // held tenant slots under the lane tile
        uint32_t page = UINT32_MAX;    // its pool page
    };
    uint32_t TakePage(Gpu& gpu);
    void Back(Gpu& gpu, const D3D12_TILED_RESOURCE_COORDINATE& c, uint32_t page);
    GpuBuffer m_up[Gpu::kFrameCount];
    std::vector<Com<ID3D12Heap>> m_heaps;
    std::vector<uint32_t> m_free;
    std::unordered_map<uint64_t, Tile> m_tiles;
    std::unordered_set<uint64_t> m_held;
    uint32_t m_tileW = 0, m_tileH = 0, m_firstPacked = 0;
    uint32_t m_texW = 256, m_texH = 128;   // the tenant's tile in texels
    Com<ID3D12Resource> m_res;
    Com<ID3D12RootSignature> m_rs;
    Com<ID3D12PipelineState> m_pso;
    uint32_t m_srv = UINT32_MAX, m_uav = UINT32_MAX;
    uint32_t m_slices = 0, m_side = 0, m_mips = 0, m_pageSide = 0, m_tenantSrv = UINT32_MAX;
    D3D12_RESOURCE_STATES m_state = D3D12_RESOURCE_STATE_UNORDERED_ACCESS;
};

}  // namespace ga
