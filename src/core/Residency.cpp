#include "core/Residency.h"

#include "core/PixEvents.h"
#include "core/TileAtlas.h"   // Cl2ProductSignature -- the proven closure drives DeriveDemand

#include <algorithm>
#include <cstring>

namespace ga {

void ResidencyManager::Init(Gpu& gpu) {
    m_gpu = &gpu;
    for (uint32_t i = 0; i < Gpu::kFrameCount; ++i) {
        m_uploadRing[i] = gpu.CreateUploadBuffer(
            static_cast<uint64_t>(kMaxMapsPerFrame) * 65536, L"residency upload ring");
    }
    for (int i = 0; i < 2; ++i) {
        m_workers.emplace_back([this] { LoaderThread(); });
    }
}

void ResidencyManager::Shutdown() {
    {
        std::lock_guard<std::mutex> lk(m_mx);
        m_quit = true;
    }
    m_cv.notify_all();
    for (auto& w : m_workers) {
        if (w.joinable()) w.join();
    }
    m_workers.clear();
}

void ResidencyManager::LoaderThread() {
    for (;;) {
        std::shared_ptr<Tracked> job;
        {
            std::unique_lock<std::mutex> lk(m_mx);
            m_cv.wait(lk, [this] { return m_quit || !m_loadQueue.empty(); });
            if (m_quit) return;
            job = m_loadQueue.front();
            m_loadQueue.pop_front();
        }
        std::vector<uint8_t> data;
        const bool ok = m_tenants[job->tenant].provider(job->req, data);
        {
            std::lock_guard<std::mutex> lk(m_mx);
            job->data = ok ? std::move(data) : std::vector<uint8_t>(65536, 0);
            job->state = TileState::Loaded;
            --m_inFlight;
        }
    }
}

int ResidencyManager::AddTextureInternal(Gpu& gpu, const wchar_t* name, uint32_t faceDim,
                                         DXGI_FORMAT fmt, TileProviderFn provider,
                                         uint32_t faces) {
    Tenant t;
    t.name = name;
    t.fmt = fmt;
    t.faceDim = faceDim;
    t.faces = faces;
    t.provider = std::move(provider);

    // Mip chain stops at the ONE-TILE level, so there are no packed mips and every tile takes
    // the same CopyTiles path (the classic needed a separate UpdateSubresource path for its
    // packed tail; capping the chain removes that whole branch). The coarsest mip is mapped
    // up front and never evicted -- the planet is never bald, and the residency-clamp always
    // has somewhere to land.
    uint32_t tileW = 128, tileH = 128;   // texels per 64KB tile
    switch (fmt) {
        case DXGI_FORMAT_BC1_UNORM:
        case DXGI_FORMAT_BC1_UNORM_SRGB: tileW = 512; tileH = 256; break;
        case DXGI_FORMAT_BC5_SNORM: tileW = 256; tileH = 256; break;
        case DXGI_FORMAT_R8G8B8A8_UNORM:
        case DXGI_FORMAT_R8G8B8A8_UNORM_SRGB: tileW = 128; tileH = 128; break;   // M6j: color
                                              // is sRGB -- HARDWARE decode, no shader hack
        case DXGI_FORMAT_R16_FLOAT: tileW = 256; tileH = 128; break;   // M6i composed height
        default: GA_CHECK(E_INVALIDARG);
    }
    uint32_t mips = 1;
    while ((faceDim >> (mips - 1)) > (std::max)(tileW, tileH)) ++mips;

    D3D12_RESOURCE_DESC rd{};
    rd.Dimension = D3D12_RESOURCE_DIMENSION_TEXTURE2D;
    rd.Width = faceDim;
    rd.Height = faceDim;
    rd.DepthOrArraySize = static_cast<UINT16>(faces);
    rd.MipLevels = static_cast<UINT16>(mips);
    rd.Format = fmt;
    rd.SampleDesc.Count = 1;
    rd.Layout = D3D12_TEXTURE_LAYOUT_64KB_UNDEFINED_SWIZZLE;
    GA_CHECK(gpu.Device()->CreateReservedResource(&rd, D3D12_RESOURCE_STATE_COPY_DEST, nullptr,
                                                 IID_PPV_ARGS(&t.res)));
    t.res->SetName(name);
    t.mips = mips;

    UINT numSub = mips * faces;
    t.tilings.resize(numSub);
    UINT totalTiles = 0;
    D3D12_PACKED_MIP_INFO packed{};
    D3D12_TILE_SHAPE shape{};
    gpu.Device()->GetResourceTiling(t.res.Get(), &totalTiles, &packed, &shape, &numSub, 0,
                                    t.tilings.data());
    GA_CHECK(packed.NumPackedMips == 0 ? S_OK : E_UNEXPECTED);
    Log("[residency] %S: %ux%u x6 %u mips, %u tiles virtual (%.0f MB), tile %ux%u",
        name, faceDim, faceDim, mips, totalTiles, totalTiles / 16.0, shape.WidthInTexels,
        shape.HeightInTexels);

    // Whole-chain SRV (cube for 6 faces, plain 2D for a window).
    t.srv = gpu.SrvHeap().Alloc();
    D3D12_SHADER_RESOURCE_VIEW_DESC sv{};
    sv.Shader4ComponentMapping = D3D12_DEFAULT_SHADER_4_COMPONENT_MAPPING;
    sv.Format = fmt;
    if (faces == 6) {
        sv.ViewDimension = D3D12_SRV_DIMENSION_TEXTURECUBE;
        sv.TextureCube.MipLevels = mips;
    } else {
        sv.ViewDimension = D3D12_SRV_DIMENSION_TEXTURE2D;
        sv.Texture2D.MipLevels = mips;
    }
    gpu.Device()->CreateShaderResourceView(t.res.Get(), &sv, gpu.SrvHeap().Cpu(t.srv));

    // Residency map: R8 at base-tile granularity, initialized to "coarsest only".
    const uint32_t rw = t.tilings[0].WidthInTiles, rh = t.tilings[0].HeightInTiles;
    const uint32_t rdim = (std::max)(rw, rh);
    for (uint32_t f = 0; f < faces; ++f) t.resCpu[f].assign(rdim * rdim, (mips - 1) * 16);
    {
        D3D12_RESOURCE_DESC md{};
        md.Dimension = D3D12_RESOURCE_DIMENSION_TEXTURE2D;
        md.Width = rdim;
        md.Height = rdim;
        md.DepthOrArraySize = static_cast<UINT16>(faces);
        md.MipLevels = 1;
        md.Format = DXGI_FORMAT_R8_UNORM;
        md.SampleDesc.Count = 1;
        const D3D12_HEAP_PROPERTIES hp{D3D12_HEAP_TYPE_DEFAULT, D3D12_CPU_PAGE_PROPERTY_UNKNOWN,
                                       D3D12_MEMORY_POOL_UNKNOWN, 0, 0};
        GA_CHECK(gpu.Device()->CreateCommittedResource(
            &hp, D3D12_HEAP_FLAG_NONE, &md, D3D12_RESOURCE_STATE_COPY_DEST, nullptr,
            IID_PPV_ARGS(&t.resMap.res)));
        t.resMap.res->SetName((t.name + L".residencyMap").c_str());
        t.resMap.format = DXGI_FORMAT_R8_UNORM;
        t.resMap.width = rdim;
        t.resMap.height = rdim;
        t.resMap.state = D3D12_RESOURCE_STATE_COPY_DEST;
        t.resMapSrv = gpu.SrvHeap().Alloc();
        D3D12_SHADER_RESOURCE_VIEW_DESC rv{};
        rv.Shader4ComponentMapping = D3D12_DEFAULT_SHADER_4_COMPONENT_MAPPING;
        rv.Format = DXGI_FORMAT_R8_UNORM;
        if (faces == 6) {
            rv.ViewDimension = D3D12_SRV_DIMENSION_TEXTURECUBE;
            rv.TextureCube.MipLevels = 1;
        } else {
            rv.ViewDimension = D3D12_SRV_DIMENSION_TEXTURE2D;
            rv.Texture2D.MipLevels = 1;
        }
        gpu.Device()->CreateShaderResourceView(t.resMap.res.Get(), &rv,
                                               gpu.SrvHeap().Cpu(t.resMapSrv));
        t.resDirty = true;
    }

    const int id = static_cast<int>(m_tenants.size());
    m_tenants.push_back(std::move(t));

    // Seed: the coarsest mip of every face wanted immediately (and pinned by construction --
    // eviction never touches the last mip).
    for (uint32_t f = 0; f < faces; ++f) Want(id, f, mips - 1, 0, 0, 1, 1);
    return id;
}

void ResidencyManager::Want(int tenant, uint32_t face, uint32_t mip, float u0, float v0,
                            float u1, float v1, bool predicted) {
    Tenant& t = m_tenants[tenant];
    if (mip >= t.mips) mip = t.mips - 1;
    // The classic's mip-tail rule: wanting a fine tile implies wanting every ancestor, and
    // ancestors are enqueued FIRST so the coarse-before-fine mapping invariant can hold.
    for (int m = static_cast<int>(t.mips) - 1; m >= static_cast<int>(mip); --m) {
        const auto& ti = t.tilings[face * t.mips + m];
        const uint32_t tw = ti.WidthInTiles;
        const uint32_t th = ti.HeightInTiles;   // UINT16 in the D3D struct; widen once
        const uint32_t x0 = static_cast<uint32_t>((std::max)(0.0f, u0) * tw);
        const uint32_t y0 = static_cast<uint32_t>((std::max)(0.0f, v0) * th);
        const uint32_t x1 =
            (std::min)(tw - 1, static_cast<uint32_t>((std::min)(0.9999f, u1) * tw));
        const uint32_t y1 =
            (std::min)(th - 1, static_cast<uint32_t>((std::min)(0.9999f, v1) * th));
        for (uint32_t y = y0; y <= y1; ++y) {
            for (uint32_t x = x0; x <= x1; ++x) {
                TileRequest r{face, static_cast<uint32_t>(m), x, y};
                const Key k = MakeKey(tenant, r);
                auto it = m_tracked.find(k);
                if (it != m_tracked.end()) {
                    it->second->lastSeen = m_frame;
                    if (!predicted) it->second->predicted = false;
                    continue;
                }
                auto tr = std::make_shared<Tracked>();
                tr->tenant = tenant;
                tr->req = r;
                tr->lastSeen = m_frame;
                tr->predicted = predicted;
                m_tracked[k] = tr;
                m_seen.push_back(tr);
            }
        }
    }
}

Motor ResidencyManager::PredictNextPose(const Motor& current, double aheadFrames) {
    if (!m_havePrevPose) {
        m_prevPose = current;
        m_havePrevPose = true;
        return current;
    }
    double a[3], b[3];
    (m_prevPose.Inverse() * current).Log(a, b);
    for (int i = 0; i < 3; ++i) {
        a[i] *= aheadFrames;
        b[i] *= aheadFrames;
    }
    const Motor pred = current * Motor::Exp(a, b);
    m_prevPose = current;
    return pred;
}

uint32_t ResidencyManager::AcquirePoolTile(Gpu& gpu) {
    if (m_freePool.empty()) {
        D3D12_HEAP_DESC hd{};
        hd.SizeInBytes = static_cast<uint64_t>(kPoolChunkTiles) * 65536;
        hd.Properties.Type = D3D12_HEAP_TYPE_DEFAULT;
        hd.Flags = D3D12_HEAP_FLAG_ALLOW_ONLY_NON_RT_DS_TEXTURES;
        Com<ID3D12Heap> heap;
        GA_CHECK(gpu.Device()->CreateHeap(&hd, IID_PPV_ARGS(&heap)));
        heap->SetName(L"residency pool chunk");
        const uint32_t chunk = static_cast<uint32_t>(m_heaps.size());
        m_heaps.push_back(heap);
        for (uint32_t i = 0; i < kPoolChunkTiles; ++i) {
            m_freePool.push_back((chunk << 16) | (kPoolChunkTiles - 1 - i));
        }
    }
    const uint32_t slot = m_freePool.back();
    m_freePool.pop_back();
    return slot;
}

void ResidencyManager::UpdateResidencyByte(Tenant& t, const TileRequest& r, bool mapped) {
    const auto& base = t.tilings[r.face * t.mips + 0];
    const auto& cur = t.tilings[r.face * t.mips + r.mip];
    const uint32_t bw = base.WidthInTiles, bh = base.HeightInTiles;
    const uint32_t rdim = (std::max)(bw, bh);
    // The residency map is SQUARE (rdim x rdim: it is uv-addressed by the sampler) while the
    // base tile grid need not be (BC1 tiles are 512x256, R16F 256x128). Each base tile spans
    // sx x sy MAP texels -- writing only (x, y) left the map's far half stale on anisotropic
    // grids, silently clamping those uv ranges to the coarsest mip (M6i fix; Mars's right
    // half of every face rode that bug from M6e until now).
    const uint32_t sx = rdim / (std::max)(1u, bw);
    const uint32_t sy = rdim / (std::max)(1u, bh);
    const uint32_t cw = bw / (std::max)(1u, static_cast<uint32_t>(cur.WidthInTiles));
    const uint32_t ch = bh / (std::max)(1u, static_cast<uint32_t>(cur.HeightInTiles));
    for (uint32_t y = r.y * ch; y < (r.y + 1) * ch && y < bh; ++y) {
        for (uint32_t x = r.x * cw; x < (r.x + 1) * cw && x < bw; ++x) {
            for (uint32_t my = y * sy; my < (y + 1) * sy; ++my) {
                for (uint32_t mx = x * sx; mx < (x + 1) * sx; ++mx) {
                    uint8_t& v = t.resCpu[r.face][my * rdim + mx];
                    const int nv =
                        mapped ? (std::min)(static_cast<int>(v), static_cast<int>(r.mip) * 16)
                               : (std::max)(static_cast<int>(v),
                                            (static_cast<int>(r.mip) + 1) * 16);
                    v = static_cast<uint8_t>(nv);
                }
            }
        }
    }
    t.resDirty = true;
}

void ResidencyManager::ProcessQueues(Gpu& gpu, ID3D12GraphicsCommandList* cl) {
    PixScope scope(cl, "residency (seen->load->map->fill; classic lists, screw prefetch)");
    ++m_frame;

    // ---- start loads (newest-seen first, coarse first; predicted tiles yield to real ones)
    {
        std::lock_guard<std::mutex> lk(m_mx);
        std::sort(m_seen.begin(), m_seen.end(), [](const auto& a, const auto& b) {
            if (a->predicted != b->predicted) return !a->predicted;
            if (a->lastSeen != b->lastSeen) return a->lastSeen > b->lastSeen;
            return a->req.mip > b->req.mip;
        });
        while (m_inFlight < static_cast<int>(kMaxLoadsInFlight) && !m_seen.empty()) {
            auto tile = m_seen.front();
            m_seen.pop_front();
            tile->state = TileState::Loading;
            m_loading.push_back(tile);
            m_loadQueue.push_back(tile);
            ++m_inFlight;
            m_cv.notify_one();
        }
    }

    // ---- gather mappable tiles (loaded, parent already mapped or coarsest)
    std::vector<std::shared_ptr<Tracked>> batch;
    {
        std::lock_guard<std::mutex> lk(m_mx);
        std::sort(m_loading.begin(), m_loading.end(), [](const auto& a, const auto& b) {
            if ((a->state == TileState::Loaded) != (b->state == TileState::Loaded)) {
                return a->state == TileState::Loaded;
            }
            if (a->req.mip != b->req.mip) return a->req.mip > b->req.mip;
            return a->lastSeen > b->lastSeen;
        });
        for (auto it = m_loading.begin();
             it != m_loading.end() && batch.size() < kMaxMapsPerFrame;) {
            auto& tile = *it;
            if (tile->state != TileState::Loaded) {
                ++it;
                continue;
            }
            const Tenant& t = m_tenants[tile->tenant];
            bool parentOk = tile->req.mip + 1 >= t.mips;
            if (!parentOk) {
                TileRequest p{tile->req.face, tile->req.mip + 1, tile->req.x / 2, tile->req.y / 2};
                auto pit = m_tracked.find(MakeKey(tile->tenant, p));
                parentOk = pit != m_tracked.end() && pit->second->state == TileState::Mapped;
            }
            if (!parentOk) {
                ++it;   // waits for its ancestor; sort order makes this rare
                continue;
            }
            batch.push_back(tile);
            it = m_loading.erase(it);
        }
    }
    if (!batch.empty()) MapAndFill(gpu, cl, batch);

    // ---- residency-map refresh (tiny R8 cubes; only when dirty)
    for (auto& t : m_tenants) {
        if (!t.resDirty) continue;
        t.resDirty = false;
        const uint32_t rdim = t.resMap.width;
        const uint32_t pitch = (rdim + 255u) & ~255u;
        // Per-frame staging from the ring? Residency maps are <10 KB: borrow the tail of the
        // upload ring slab (tiles never fill it completely because batch <= kMaxMapsPerFrame).
        GpuBuffer& ring = m_uploadRing[gpu.FrameIndex()];
        const uint64_t tail = static_cast<uint64_t>(kMaxMapsPerFrame) * 65536 -
                              static_cast<uint64_t>(pitch) * rdim * t.faces;
        uint8_t* dst = ring.cpu + tail;
        gpu.Transition(cl, t.resMap, D3D12_RESOURCE_STATE_COPY_DEST);
        for (uint32_t f = 0; f < t.faces; ++f) {
            for (uint32_t y = 0; y < rdim; ++y) {
                memcpy(dst + (static_cast<uint64_t>(f) * rdim + y) * pitch,
                       &t.resCpu[f][y * rdim], rdim);
            }
            D3D12_TEXTURE_COPY_LOCATION dl{}, sl{};
            dl.pResource = t.resMap.res.Get();
            dl.Type = D3D12_TEXTURE_COPY_TYPE_SUBRESOURCE_INDEX;
            dl.SubresourceIndex = f;
            sl.pResource = ring.res.Get();
            sl.Type = D3D12_TEXTURE_COPY_TYPE_PLACED_FOOTPRINT;
            sl.PlacedFootprint.Offset = tail + static_cast<uint64_t>(f) * rdim * pitch;
            sl.PlacedFootprint.Footprint = {DXGI_FORMAT_R8_UNORM, rdim, rdim, 1, pitch};
            cl->CopyTextureRegion(&dl, 0, 0, 0, &sl, nullptr);
        }
        gpu.Transition(cl, t.resMap, D3D12_RESOURCE_STATE_PIXEL_SHADER_RESOURCE);
    }

    // ---- stats
    char s[256];
    uint64_t bytes = 0;
    for (const auto& tr : m_mapped) {
        (void)tr;
        bytes += 65536;
    }
    snprintf(s, sizeof(s), "streams %zu res (%zu t, %.0f MB, %u fetches)", m_tenants.size(),
             m_mapped.size(), bytes / 1048576.0, fetchesThisRun);
    stats = s;
    for (const auto& f : m_fields) {
        char fs[96];
        snprintf(fs, sizeof(fs), "  %s %u t %.0f MB", f.name.c_str(), f.tiles(),
                 f.bytes() / 1048576.0);
        stats += fs;
    }
}

void ResidencyManager::MapAndFill(Gpu& gpu, ID3D12GraphicsCommandList* cl,
                                  const std::vector<std::shared_ptr<Tracked>>& batch) {
    PixMarker(cl, "residency.mapAndFill");
    // Evictions to free pool slots when needed. Never evict: the coarsest mip, tiles seen
    // within the frame-overlap window (the GPU may still read them), or tiles with a mapped
    // child (the classic's pyramid invariant).
    auto childMapped = [&](const std::shared_ptr<Tracked>& tr) {
        const Tenant& t = m_tenants[tr->tenant];
        if (tr->req.mip == 0) return false;
        for (uint32_t dy = 0; dy < 2; ++dy) {
            for (uint32_t dx = 0; dx < 2; ++dx) {
                TileRequest c{tr->req.face, tr->req.mip - 1, tr->req.x * 2 + dx,
                              tr->req.y * 2 + dy};
                auto it = m_tracked.find(MakeKey(tr->tenant, c));
                if (it != m_tracked.end() && it->second->state == TileState::Mapped) return true;
            }
        }
        (void)t;
        return false;
    };

    struct PerHeap {
        std::vector<D3D12_TILED_RESOURCE_COORDINATE> coords;
        std::vector<D3D12_TILE_REGION_SIZE> sizes;
        std::vector<D3D12_TILE_RANGE_FLAGS> flags;
        std::vector<UINT> offsets;
        std::vector<UINT> counts;
    };
    // Keyed by (tenant, heapChunk); evictions keyed by (tenant, UINT32_MAX).
    std::map<std::pair<int, uint32_t>, PerHeap> calls;

    std::vector<std::shared_ptr<Tracked>> toFill;
    for (const auto& tile : batch) {
        Tenant& t = m_tenants[tile->tenant];
        if (m_freePool.empty() && m_heaps.size() * kPoolChunkTiles >= kPoolCapTiles) {
            // Pool at budget: evict.
            std::sort(m_mapped.begin(), m_mapped.end(), [](const auto& a, const auto& b) {
                if (a->lastSeen != b->lastSeen) return a->lastSeen < b->lastSeen;
                return a->req.mip < b->req.mip;
            });
            bool freed = false;
            for (auto it = m_mapped.begin(); it != m_mapped.end(); ++it) {
                auto& victim = *it;
                Tenant& vt = m_tenants[victim->tenant];
                if (victim->req.mip == vt.mips - 1) continue;
                if (victim->lastSeen + kEvictAgeFrames >= m_frame) continue;
                if (childMapped(victim)) continue;
                auto& ev = calls[{victim->tenant, UINT32_MAX}];
                ev.coords.push_back({victim->req.x, victim->req.y, 0,
                                     victim->req.face * vt.mips + victim->req.mip});
                ev.sizes.push_back({1, FALSE, 0, 0, 0});
                ev.flags.push_back(D3D12_TILE_RANGE_FLAG_NULL);
                ev.offsets.push_back(0);
                ev.counts.push_back(1);
                UpdateResidencyByte(vt, victim->req, false);
                m_freePool.push_back(victim->pool);
                m_tracked.erase(MakeKey(victim->tenant, victim->req));
                m_mapped.erase(it);
                freed = true;
                break;
            }
            if (!freed) break;   // nothing evictable yet; try again next frame
        }
        const uint32_t slot = AcquirePoolTile(gpu);
        tile->pool = slot;
        auto& mc = calls[{tile->tenant, slot >> 16}];
        mc.coords.push_back(
            {tile->req.x, tile->req.y, 0, tile->req.face * t.mips + tile->req.mip});
        mc.sizes.push_back({1, FALSE, 0, 0, 0});
        mc.flags.push_back(D3D12_TILE_RANGE_FLAG_NONE);
        mc.offsets.push_back(slot & 0xFFFF);
        mc.counts.push_back(1);
        tile->state = TileState::Mapped;
        m_mapped.push_back(tile);
        UpdateResidencyByte(t, tile->req, true);
        toFill.push_back(tile);
    }

    // Queue-side mapping updates land before this frame's list executes.
    for (auto& [key, c] : calls) {
        gpu.Queue()->UpdateTileMappings(
            m_tenants[key.first].res.Get(), static_cast<UINT>(c.coords.size()), c.coords.data(),
            c.sizes.data(), key.second == UINT32_MAX ? nullptr : m_heaps[key.second].Get(),
            static_cast<UINT>(c.flags.size()), c.flags.data(), c.offsets.data(), c.counts.data(),
            D3D12_TILE_MAPPING_FLAG_NONE);
    }

    // Fill through the frame-indexed upload ring: linear 64KB blobs -> swizzled tiles.
    GpuBuffer& ring = m_uploadRing[gpu.FrameIndex()];
    uint64_t off = 0;
    for (auto& t : m_tenants) {
        if (t.state != D3D12_RESOURCE_STATE_COPY_DEST) {
            D3D12_RESOURCE_BARRIER b{};
            b.Type = D3D12_RESOURCE_BARRIER_TYPE_TRANSITION;
            b.Transition.pResource = t.res.Get();
            b.Transition.Subresource = D3D12_RESOURCE_BARRIER_ALL_SUBRESOURCES;
            b.Transition.StateBefore = t.state;
            b.Transition.StateAfter = D3D12_RESOURCE_STATE_COPY_DEST;
            cl->ResourceBarrier(1, &b);
            t.state = D3D12_RESOURCE_STATE_COPY_DEST;
        }
    }
    for (const auto& tile : toFill) {
        Tenant& t = m_tenants[tile->tenant];
        const size_t n = tile->data.size() < 65536 ? tile->data.size() : 65536;
        memcpy(ring.cpu + off, tile->data.data(), n);
        const D3D12_TILED_RESOURCE_COORDINATE coord{
            tile->req.x, tile->req.y, 0, tile->req.face * t.mips + tile->req.mip};
        const D3D12_TILE_REGION_SIZE size{1, FALSE, 0, 0, 0};
        cl->CopyTiles(t.res.Get(), &coord, &size, ring.res.Get(), off,
                      D3D12_TILE_COPY_FLAG_LINEAR_BUFFER_TO_SWIZZLED_TILED_RESOURCE);
        off += 65536;
        tile->data.clear();
        tile->data.shrink_to_fit();
    }
    for (auto& t : m_tenants) {
        D3D12_RESOURCE_BARRIER b{};
        b.Type = D3D12_RESOURCE_BARRIER_TYPE_TRANSITION;
        b.Transition.pResource = t.res.Get();
        b.Transition.Subresource = D3D12_RESOURCE_BARRIER_ALL_SUBRESOURCES;
        b.Transition.StateBefore = t.state;
        b.Transition.StateAfter = D3D12_RESOURCE_STATE_PIXEL_SHADER_RESOURCE;
        cl->ResourceBarrier(1, &b);
        t.state = D3D12_RESOURCE_STATE_PIXEL_SHADER_RESOURCE;
    }
}

void ResidencyManager::RegisterField(const char* name, std::function<uint64_t()> residentBytes,
                                     std::function<uint32_t()> residentTiles) {
    m_fields.push_back({name, std::move(residentBytes), std::move(residentTiles)});
}

void ResidencyManager::PublishSignatures(const std::string& field, uint32_t tilesX,
                                         uint32_t tilesY, std::vector<uint8_t> sig) {
    m_signatures[field] = {tilesX, tilesY, std::move(sig)};
}

bool ResidencyManager::DeriveDemand(const std::string& srcA, const std::string& srcB,
                                    std::vector<uint8_t>& out, uint32_t& tilesX,
                                    uint32_t& tilesY) const {
    const auto a = m_signatures.find(srcA);
    const auto b = m_signatures.find(srcB);
    if (a == m_signatures.end() || b == m_signatures.end()) return false;
    if (a->second.tilesX != b->second.tilesX || a->second.tilesY != b->second.tilesY) {
        return false;
    }
    tilesX = a->second.tilesX;
    tilesY = a->second.tilesY;
    out.resize(a->second.sig.size());
    for (size_t i = 0; i < out.size(); ++i) {
        // The proven Cayley closure: where either operand is signature-zero, the product is
        // zero and the derived field needs NO tile there -- decided without reading any data.
        out[i] = Cl2ProductSignature(a->second.sig[i], b->second.sig[i]);
    }
    return true;
}

}  // namespace ga
