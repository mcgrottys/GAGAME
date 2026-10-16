// MaxLane - the max lane of a scalar page tenant (MaxLane.h).
#include "hal/MaxLane.h"

#include "core/Common.h"
#include "hal/Context.h"
#include "hal/Pipeline.h"
#include "hal/Resources.h"
#include "hal/Root.h"
#include "hal/Shader.h"
#include "hal/Views.h"

#include <algorithm>

namespace ga {

bool MaxLane::Init(Gpu& gpu, ShaderCompiler& sc, const std::wstring& shaderDir, const std::wstring& name,
                   uint32_t slices, uint32_t faceDim, uint32_t mips, uint32_t texW, uint32_t texH,
                   uint32_t tenantSrv) {
    if (texW % kBlock || texH % kBlock || texW / kBlock > 16 || texH / kBlock > 8 || faceDim % (kBlock * texW) != 0) {
        Log("[maxlane] %ls: tiles of %ux%u on a %u side do not fit the kernel's 16 x 8 blocks: no lane", name.c_str(),
            texW, texH, faceDim);
        return false;
    }
    m_texW = texW;
    m_texH = texH;
    // A typed UAV load of R16G16_FLOAT is one of the formats D3D12 leaves optional: asked, not assumed.
    D3D12_FEATURE_DATA_FORMAT_SUPPORT fs{DXGI_FORMAT_R16G16_FLOAT, {}, {}};
    if (FAILED(gpu.Device()->CheckFeatureSupport(D3D12_FEATURE_FORMAT_SUPPORT, &fs, sizeof(fs))) ||
        !(fs.Support2 & D3D12_FORMAT_SUPPORT2_UAV_TYPED_LOAD)) {
        Log("[maxlane] %ls: this device cannot load R16G16_FLOAT through a UAV: no lane", name.c_str());
        return false;
    }
    m_slices = slices;
    m_pageSide = faceDim;
    m_side = faceDim / kBlock;
    m_mips = (std::min)(mips, kMaxMips);
    m_tenantSrv = tenantSrv;
    // RESERVED, as the tenant is: a lane tile is backed only while a held tile of the tenant lies
    // under it (Hold), so the lane costs what is resident, not what the slices could hold.
    D3D12_RESOURCE_DESC d = hal::TextureDesc2D(m_side, m_side, DXGI_FORMAT_R16G16_FLOAT,
                                               D3D12_RESOURCE_FLAG_ALLOW_UNORDERED_ACCESS,
                                               static_cast<uint16_t>(m_mips));
    d.DepthOrArraySize = static_cast<UINT16>(slices);
    d.Layout = D3D12_TEXTURE_LAYOUT_64KB_UNDEFINED_SWIZZLE;
    GA_CHECK(gpu.Device()->CreateReservedResource(&d, D3D12_RESOURCE_STATE_UNORDERED_ACCESS, nullptr,
                                                 IID_PPV_ARGS(&m_res)));
    m_res->SetName((name + L".maxLane").c_str());
    m_state = D3D12_RESOURCE_STATE_UNORDERED_ACCESS;
    UINT total = 0, numSub = 0;
    D3D12_PACKED_MIP_INFO packed{};
    D3D12_TILE_SHAPE shape{};
    gpu.Device()->GetResourceTiling(m_res.Get(), &total, &packed, &shape, &numSub, 0, nullptr);
    m_tileW = shape.WidthInTexels;
    m_tileH = shape.HeightInTexels;
    m_firstPacked = packed.NumPackedMips > 0 ? packed.NumStandardMips : m_mips;
    // The packed tail of every slice (the mips under one tile) is small: backed once, for good. Its
    // tiles are numbered along X from the first packed mip's subresource.
    for (uint32_t s = 0; packed.NumPackedMips > 0 && s < slices; ++s) {
        for (uint32_t k = 0; k < packed.NumTilesForPackedMips; ++k) {
            Back(gpu, D3D12_TILED_RESOURCE_COORDINATE{k, 0, 0, m_firstPacked + s * m_mips}, TakePage(gpu));
        }
    }
    m_srv = gpu.CreateSrvArray(m_res.Get(), DXGI_FORMAT_R16G16_FLOAT, m_mips, slices);
    hal::Table t = hal::Table::Alloc(gpu, kMaxMips, "maxlane.uav");
    for (uint32_t m = 0; m < kMaxMips; ++m) {
        t.UavArray(m, m_res.Get(), DXGI_FORMAT_R16G16_FLOAT, 0, slices, (std::min)(m, m_mips - 1));
    }
    m_uav = t.Base();
    m_rs = hal::RootLayout{}
               .Constants(0, 4)
               .Srv(0, 0)
               .Table({hal::SrvRange(0, hal::kUnbounded, 1)})
               .Table({hal::UavRange(0, kMaxMips)})
               .Build(gpu, "maxlane");
    m_pso = hal::BuildCompute(gpu, m_rs.Get(),
                              sc.Compile(shaderDir + L"/MaxLane.hlsl", L"CsLane", L"cs_6_0"), "maxlane");
    if (!m_pso) {
        Log("[maxlane] %ls: the kernel did not build: no lane", name.c_str());
        return false;
    }
    for (uint32_t f = 0; f < Gpu::kFrameCount; ++f) {
        m_up[f] = gpu.CreateUploadBuffer(uint64_t(kMaxJobs) * sizeof(Row), L"maxlane.jobs");
    }
    Log("[maxlane] %ls: %u slices x %u mips of %ux%u (a texel a %ux%u block), reserved in tiles of %ux%u "
        "(mips %u.. packed, backed for good), srv %u",
        name.c_str(), slices, m_mips, m_side, m_side, kBlock, kBlock, m_tileW, m_tileH, m_firstPacked, m_srv);
    return true;
}

uint32_t MaxLane::TakePage(Gpu& gpu) {
    if (m_free.empty()) {
        D3D12_HEAP_DESC hd{};
        hd.SizeInBytes = uint64_t(kChunkTiles) * 65536;
        hd.Properties.Type = D3D12_HEAP_TYPE_DEFAULT;
        hd.Flags = D3D12_HEAP_FLAG_ALLOW_ONLY_NON_RT_DS_TEXTURES;
        Com<ID3D12Heap> heap;
        GA_CHECK(gpu.Device()->CreateHeap(&hd, IID_PPV_ARGS(&heap)));
        heap->SetName(L"maxlane pool chunk");
        const uint32_t base = static_cast<uint32_t>(m_heaps.size()) * kChunkTiles;
        m_heaps.push_back(heap);
        for (uint32_t k = kChunkTiles; k-- > 0;) m_free.push_back(base + k);
    }
    const uint32_t p = m_free.back();
    m_free.pop_back();
    ++pagesHeld;
    return p;
}

// One lane tile backed by pool page `page`, or released (page ~0). Queue-side: it lands before the
// next list executes, after every list already submitted, so nothing in flight sees it change.
void MaxLane::Back(Gpu& gpu, const D3D12_TILED_RESOURCE_COORDINATE& c, uint32_t page) {
    const D3D12_TILE_REGION_SIZE one{1, FALSE, 0, 0, 0};
    const UINT n = 1;
    if (page == UINT32_MAX) {
        const D3D12_TILE_RANGE_FLAGS flag = D3D12_TILE_RANGE_FLAG_NULL;
        gpu.Queue()->UpdateTileMappings(m_res.Get(), 1, &c, &one, nullptr, 1, &flag, nullptr, &n,
                                        D3D12_TILE_MAPPING_FLAG_NONE);
        return;
    }
    const UINT offset = page % kChunkTiles;
    const D3D12_TILE_RANGE_FLAGS flag = D3D12_TILE_RANGE_FLAG_NONE;
    gpu.Queue()->UpdateTileMappings(m_res.Get(), 1, &c, &one, m_heaps[page / kChunkTiles].Get(), 1, &flag,
                                    &offset, &n, D3D12_TILE_MAPPING_FLAG_NONE);
}

// THE HOLD: a tenant slot (slice, mip, tile x, tile y) of m_texW x m_texH texels covers m_texW / 16 x
// m_texH / 16 lane texels, and a lane tile of m_tileW x m_tileH lane texels covers the rest. The lane
// tile is backed by the first held slot under it and released with the last. A recycled page holds
// another place's blocks; they are never read there, because a block is read only where its slot is
// held, and a slot's blocks are built in the turn it is held (its job follows in Record).
void MaxLane::Hold(Gpu& gpu, uint32_t slice, uint32_t mip, uint32_t x, uint32_t y, bool held) {
    if (mip >= m_firstPacked) return;   // the packed tail is backed for good
    const uint64_t slot = (uint64_t(slice) << 56) | (uint64_t(mip) << 48) | (uint64_t(y & 0xFFFFFFu) << 24) |
                          uint64_t(x & 0xFFFFFFu);
    if (held ? !m_held.insert(slot).second : m_held.erase(slot) == 0) return;   // no change
    const uint32_t lx = (x * (m_texW / kBlock)) / m_tileW, ly = (y * (m_texH / kBlock)) / m_tileH;
    const uint64_t key = (uint64_t(slice) << 56) | (uint64_t(mip) << 48) | (uint64_t(ly) << 24) | uint64_t(lx);
    const D3D12_TILED_RESOURCE_COORDINATE c{lx, ly, 0, mip + slice * m_mips};
    Tile& e = m_tiles[key];
    if (held) {
        if (e.count++ == 0) {
            e.page = TakePage(gpu);
            Back(gpu, c, e.page);
        }
        return;
    }
    if (e.count > 0 && --e.count == 0) {
        Back(gpu, c, UINT32_MAX);
        m_free.push_back(e.page);
        --pagesHeld;
        m_tiles.erase(key);
    }
}

void MaxLane::Record(Gpu& gpu, ID3D12GraphicsCommandList* cl, std::vector<Job>& jobs) {
    if (!m_pso || jobs.empty()) return;
    hal::CommandContext cmd(gpu, cl, hal::Owner::Frame);
    cmd.BindHeaps();
    cmd.Barrier(m_res.Get(), m_state, D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
    m_state = D3D12_RESOURCE_STATE_UNORDERED_ACCESS;
    cmd.ComputeRoot(m_rs.Get());
    cmd.ComputeBindless(2);
    cmd.ComputeTable(3, m_uav);
    GpuBuffer& up = m_up[gpu.FrameIndex()];
    cmd.ComputeSrvAt(1, up.gpu);
    std::stable_sort(jobs.begin(), jobs.end(), [](const Job& a, const Job& b) { return a.level < b.level; });
    // A turn past the buffer's jobs (a boot's priming of every held tile) keeps its finest levels;
    // the coarse ones it leaves hold what they last held, and are rebuilt by the next landing under
    // them.
    const size_t n = (std::min)(jobs.size(), size_t(kMaxJobs));
    if (n < jobs.size()) {
        ++overflowTurns;
        Log("[maxlane] a turn of %zu jobs, past the %u the buffer holds: the coarsest %zu wait for a later turn's",
            jobs.size(), kMaxJobs, jobs.size() - n);
    }
    Row* rows = reinterpret_cast<Row*>(up.cpu);
    for (size_t i = 0; i < n; ++i) {
        const Job& j = jobs[i];
        Row& r = rows[i];
        r.job[0] = j.slice; r.job[1] = j.mip; r.job[2] = j.x; r.job[3] = j.y;
        r.page[0] = j.wrap; r.page[1] = m_pageSide >> j.mip; r.page[2] = m_tenantSrv; r.page[3] = m_texW | (m_texH << 16);
        for (uint32_t c = 0; c < 4; ++c) {
            for (uint32_t k = 0; k < 4; ++k) r.child[c][k] = j.child[c][k];
        }
    }
    cmd.Pipeline(m_pso.Get());
    uint32_t levels = 0;
    for (size_t i = 0; i < n;) {
        size_t e = i + 1;
        while (e < n && jobs[e].level == jobs[i].level) ++e;
        if (levels) cmd.UavBarrier(m_res.Get());   // this level folds the lanes the finer one wrote
        const uint32_t turn[4] = {static_cast<uint32_t>(i), 0u, 0u, 0u};
        cl->SetComputeRoot32BitConstants(0, 4, turn, 0);
        cmd.Dispatch(static_cast<uint32_t>(e - i), 1, 1);
        ++levels;
        i = e;
    }
    // Back to what the readers take: the globe's pixels and every compute that may ask.
    const D3D12_RESOURCE_STATES read =
        D3D12_RESOURCE_STATE_PIXEL_SHADER_RESOURCE | D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE;
    cmd.Barrier(m_res.Get(), m_state, read);
    m_state = read;
    jobsTotal += n;
    ++turns;
    lastJobs = static_cast<uint32_t>(n);
    lastLevels = levels;
    if (turns <= 3 || turns % 500 == 0) {
        Log("[maxlane] turn %llu: %zu tiles built over %u levels (%llu in all); %u pages backed (%.1f MB)",
            (unsigned long long)turns, n, levels, (unsigned long long)jobsTotal, pagesHeld,
            pagesHeld * 64.0 / 1024.0);
    }
}

}  // namespace ga
