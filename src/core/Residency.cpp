#include "core/Residency.h"

#include "core/PixEvents.h"
#include "core/TileAtlas.h"   // Cl2ProductSignature -- the proven closure drives DeriveDemand

#include <algorithm>
#include <chrono>
#include <cstring>

namespace ga {

void ResidencyManager::Init(Gpu& gpu) {
    m_gpu = &gpu;
    for (uint32_t i = 0; i < Gpu::kFrameCount; ++i) {
        m_uploadRing[i] = gpu.CreateUploadBuffer(
            static_cast<uint64_t>(kMaxMapsPerFrame) * 65536, L"residency upload ring");
    }
    // M7w: 2 workers starved every descent (140 loads in 300 frames measured at the flood-rail
    // pose -- the patchwork was coarse fallback, not bad data). Paints are local disk/CPU work.
    const unsigned hc = std::thread::hardware_concurrency();
    const int nWorkers = (std::min)(12, (std::max)(4, static_cast<int>(hc) - 2));
    for (int i = 0; i < nWorkers; ++i) {
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
        TileLoc loc;
        // The provider fills EITHER loc (archived: the bytes stay on disk) or data (painted, or
        // a loose file). Which one it chooses is per tile, so a half-packed cache streams the
        // packed part directly and paints the rest, with no mode to select.
        //
        // AND IT IS ONLY OFFERED THE CHOICE WHEN THE CALLER CAN TAKE IT. Passing &loc
        // unconditionally was a real bug: with the reader disabled the provider still returned a
        // LOCATION, data stayed empty, and the upload ring memcpy'd zero bytes into a mapped
        // tile -- so the tile claimed residency over whatever the pool slot last held. It showed
        // as 20% of the globe changing against the reference. A null here means "bytes, please",
        // which is the contract every provider already implemented.
        const bool canStream = m_stream && m_stream->Available();
        const auto t0 = std::chrono::steady_clock::now();
        const bool ok =
            m_tenants[job->tenant].provider(job->req, data, canStream ? &loc : nullptr);
        // M9al: read or paint? The provider does not say, but its clock does -- a 64 KB file
        // is well under a millisecond, a composed tile is tens.
        const uint64_t us = static_cast<uint64_t>(
            std::chrono::duration_cast<std::chrono::microseconds>(
                std::chrono::steady_clock::now() - t0).count());
        {
            std::lock_guard<std::mutex> lk(m_mx);
            {
                Tenant& tn = m_tenants[job->tenant];
                if (us < 4000) { ++tn.loadsRead; tn.readUs += us; }
                else           { ++tn.loadsPaint; tn.paintUs += us; }
            }
            // M7w: a failed load must NEVER fabricate a zero tile -- mapping zeros makes the
            // residency map swear real data exists where the GPU holds bed=0 / black (the
            // step-11 MISMATCH). Retry a few times; then leave the tile honestly UNMAPPED so
            // consumers fall back to the coarser REAL mip (Tier-2 nulls never reach them,
            // because the claim is only ever written on a true map).
            if (ok) {
                job->loc = loc;
                job->data = std::move(data);
                job->state = TileState::Loaded;
            } else if (++job->retries < 4) {
                job->state = TileState::Seen;   // requeued by ProcessQueues from m_loading
            } else {
                job->state = TileState::Failed;
                ++m_failedLoads;
            }
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
    // M9x: lay out the flat stamp array now that every plane's tile extent is known.
    t.stampBase.resize(numSub);
    t.stampW.resize(numSub);
    {
        uint32_t acc = 0;
        for (UINT i = 0; i < numSub; ++i) {
            t.stampBase[i] = acc;
            t.stampW[i] = t.tilings[i].WidthInTiles;
            acc += t.tilings[i].WidthInTiles * t.tilings[i].HeightInTiles;
        }
        t.stamp.assign(acc, 0u);
        Log("[residency] %S: stamp array %u tiles (%.2f MB) -- Want()'s hot question leaves "
            "the hash map",
            name, acc, acc * 4.0 / 1048576.0);
    }
    Log("[residency] %S: %ux%u x%u %u mips, %u tiles virtual (%.0f MB), tile %ux%u",
        name, faceDim, faceDim, faces, mips, totalTiles, totalTiles / 16.0, shape.WidthInTexels,
        shape.HeightInTexels);

    // Whole-chain SRV: cube for 6 faces, plain 2D for a window, Texture2DArray for a page
    // tenant -- which ALSO gets a cube view over slices 0..5 (M9ap), so the globe keeps the
    // hardware's seamless cube filtering while the Mercator pages ride the array view.
    t.srv = gpu.SrvHeap().Alloc();
    D3D12_SHADER_RESOURCE_VIEW_DESC sv{};
    sv.Shader4ComponentMapping = D3D12_DEFAULT_SHADER_4_COMPONENT_MAPPING;
    sv.Format = fmt;
    if (faces == 6) {
        sv.ViewDimension = D3D12_SRV_DIMENSION_TEXTURECUBE;
        sv.TextureCube.MipLevels = mips;
    } else if (faces == 1) {
        sv.ViewDimension = D3D12_SRV_DIMENSION_TEXTURE2D;
        sv.Texture2D.MipLevels = mips;
    } else {
        sv.ViewDimension = D3D12_SRV_DIMENSION_TEXTURE2DARRAY;
        sv.Texture2DArray.MipLevels = mips;
        sv.Texture2DArray.ArraySize = faces;
    }
    gpu.Device()->CreateShaderResourceView(t.res.Get(), &sv, gpu.SrvHeap().Cpu(t.srv));
    if (faces > 6) {
        t.srvCube = gpu.SrvHeap().Alloc();
        D3D12_SHADER_RESOURCE_VIEW_DESC cv{};
        cv.Shader4ComponentMapping = D3D12_DEFAULT_SHADER_4_COMPONENT_MAPPING;
        cv.Format = fmt;
        cv.ViewDimension = D3D12_SRV_DIMENSION_TEXTURECUBEARRAY;
        cv.TextureCubeArray.MipLevels = mips;
        cv.TextureCubeArray.First2DArrayFace = 0;
        cv.TextureCubeArray.NumCubes = 1;
        gpu.Device()->CreateShaderResourceView(t.res.Get(), &cv, gpu.SrvHeap().Cpu(t.srvCube));
    }

    // Residency map: R8 at base-tile granularity, initialized to "coarsest only".
    const uint32_t rw = t.tilings[0].WidthInTiles, rh = t.tilings[0].HeightInTiles;
    const uint32_t rdim = (std::max)(rw, rh);
    t.resCpu.assign(faces, std::vector<uint8_t>());
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
        } else if (faces == 1) {
            rv.ViewDimension = D3D12_SRV_DIMENSION_TEXTURE2D;
            rv.Texture2D.MipLevels = 1;
        } else {
            rv.ViewDimension = D3D12_SRV_DIMENSION_TEXTURE2DARRAY;
            rv.Texture2DArray.MipLevels = 1;
            rv.Texture2DArray.ArraySize = faces;
        }
        gpu.Device()->CreateShaderResourceView(t.resMap.res.Get(), &rv,
                                               gpu.SrvHeap().Cpu(t.resMapSrv));
        if (faces > 6) {
            t.resMapSrvCube = gpu.SrvHeap().Alloc();
            D3D12_SHADER_RESOURCE_VIEW_DESC cv{};
            cv.Shader4ComponentMapping = D3D12_DEFAULT_SHADER_4_COMPONENT_MAPPING;
            cv.Format = DXGI_FORMAT_R8_UNORM;
            cv.ViewDimension = D3D12_SRV_DIMENSION_TEXTURECUBEARRAY;
            cv.TextureCubeArray.MipLevels = 1;
            cv.TextureCubeArray.First2DArrayFace = 0;
            cv.TextureCubeArray.NumCubes = 1;
            gpu.Device()->CreateShaderResourceView(t.resMap.res.Get(), &cv,
                                                   gpu.SrvHeap().Cpu(t.resMapSrvCube));
        }
        t.resDirty = true;
    }

    const int id = static_cast<int>(m_tenants.size());
    m_tenants.push_back(std::move(t));

    // M7w: THE CLAIM IS MADE TRUE AT BIRTH. The residency map was constructed already
    // claiming "coarsest mip resident" while the seed tiles streamed asynchronously --
    // for those frames every consumer (ComposedColor's cube, ComposedHeight's window,
    // the bank kernel's bed reads) sampled NULL tiles through a map that swore they were
    // real: Tier-2 zeros wearing provenance. The hypervisor's step-11 cross-check caught
    // it as GPU +0.00 vs stack -47.67. The coarsest mip is a handful of tiles per tenant
    // and the providers are cache-first: load, map, and fill them SYNCHRONOUSLY before
    // the tenant is visible, so there is never a moment where the map lies. A tile whose
    // provider fails at boot writes 255 ("nothing here") into its span instead -- honest
    // -- and stays on the async retry path.
    {
        Tenant& tn = m_tenants[id];
        struct BootTile {
            TileRequest req;
            uint32_t slot;
            std::vector<uint8_t> data;
        };
        std::vector<BootTile> boot;
        std::vector<TileRequest> failedBoot;
        const uint32_t coarsest = mips - 1;
        for (uint32_t f = 0; f < faces; ++f) {
            const auto& ti = tn.tilings[f * mips + coarsest];
            const uint32_t tw = (std::max)(1u, static_cast<uint32_t>(ti.WidthInTiles));
            const uint32_t th = (std::max)(1u, static_cast<uint32_t>(ti.HeightInTiles));
            for (uint32_t y = 0; y < th; ++y) {
                for (uint32_t x = 0; x < tw; ++x) {
                    TileRequest r{f, coarsest, x, y};
                    std::vector<uint8_t> data;
                    if (tn.provider(r, data, nullptr) && !data.empty()) {
                        boot.push_back({r, AcquirePoolTile(gpu), std::move(data)});
                    } else {
                        // Honest "nothing": overwrite this tile's residency span with 255
                        // and leave the async seed to retry it.
                        const uint32_t rdim = tn.resMap.width;
                        const uint32_t sx =
                            rdim / (std::max)(1u, static_cast<uint32_t>(ti.WidthInTiles));
                        const uint32_t sy =
                            rdim / (std::max)(1u, static_cast<uint32_t>(ti.HeightInTiles));
                        for (uint32_t my = y * sy; my < (y + 1) * sy && my < rdim; ++my) {
                            for (uint32_t mx = x * sx; mx < (x + 1) * sx && mx < rdim; ++mx) {
                                tn.resCpu[f][static_cast<size_t>(my) * rdim + mx] = 255;
                            }
                        }
                        Log("[residency] %S: coarsest tile f%u (%u,%u) failed at boot -- "
                            "left NULL (map says nothing; async retry queued)",
                            tn.name.c_str(), f, x, y);
                        failedBoot.push_back(r);
                    }
                }
            }
        }
        if (!boot.empty()) {
            for (const BootTile& b : boot) {
                const D3D12_TILED_RESOURCE_COORDINATE coord{
                    b.req.x, b.req.y, 0, b.req.face * mips + b.req.mip};
                const D3D12_TILE_REGION_SIZE size{1, FALSE, 0, 0, 0};
                const D3D12_TILE_RANGE_FLAGS flag = D3D12_TILE_RANGE_FLAG_NONE;
                const UINT offset = b.slot & 0xFFFF;
                const UINT count = 1;
                gpu.Queue()->UpdateTileMappings(tn.res.Get(), 1, &coord, &size,
                                                m_heaps[b.slot >> 16].Get(), 1, &flag,
                                                &offset, &count,
                                                D3D12_TILE_MAPPING_FLAG_NONE);
            }
            GpuBuffer stage = gpu.CreateUploadBuffer(
                static_cast<uint64_t>(boot.size()) * 65536, L"residency boot tiles");
            auto* cl = gpu.BeginUpload();
            uint64_t off = 0;
            for (const BootTile& b : boot) {
                const size_t n = b.data.size() < 65536 ? b.data.size() : 65536;
                memcpy(stage.cpu + off, b.data.data(), n);
                const D3D12_TILED_RESOURCE_COORDINATE coord{
                    b.req.x, b.req.y, 0, b.req.face * mips + b.req.mip};
                const D3D12_TILE_REGION_SIZE size{1, FALSE, 0, 0, 0};
                cl->CopyTiles(tn.res.Get(), &coord, &size, stage.res.Get(), off,
                              D3D12_TILE_COPY_FLAG_LINEAR_BUFFER_TO_SWIZZLED_TILED_RESOURCE);
                off += 65536;
            }
            D3D12_RESOURCE_BARRIER br{};
            br.Type = D3D12_RESOURCE_BARRIER_TYPE_TRANSITION;
            br.Transition.pResource = tn.res.Get();
            br.Transition.Subresource = D3D12_RESOURCE_BARRIER_ALL_SUBRESOURCES;
            br.Transition.StateBefore = tn.state;
            br.Transition.StateAfter = D3D12_RESOURCE_STATE_PIXEL_SHADER_RESOURCE |
                                       D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE;
            cl->ResourceBarrier(1, &br);
            tn.state = D3D12_RESOURCE_STATE_PIXEL_SHADER_RESOURCE;
            gpu.EndUpload();   // waits: the stage buffer may die, the tiles are real
            for (const BootTile& b : boot) {
                auto tr = std::make_shared<Tracked>();
                tr->tenant = id;
                tr->req = b.req;
                tr->lastSeen = m_frame;
                tr->pool = b.slot;
                tr->state = TileState::Mapped;
                m_tracked[MakeKey(id, b.req)] = tr;
                m_mapped.push_back(tr);
                UpdateResidencyByte(tn, b.req, true);
            }
        }
        // Retry-wants AFTER registration, so a mixed face cannot double-map its healthy
        // tiles (Want on a Mapped tracked entry only bumps lastSeen).
        for (const TileRequest& r : failedBoot) {
            const auto& ti = tn.tilings[r.face * mips + coarsest];
            const float uc = (r.x + 0.5f) / (std::max)(1u, static_cast<uint32_t>(ti.WidthInTiles));
            const float vc = (r.y + 0.5f) / (std::max)(1u, static_cast<uint32_t>(ti.HeightInTiles));
            Want(id, r.face, coarsest, uc, vc, uc, vc);
        }
    }
    return id;
}

void ResidencyManager::Want(int tenant, uint32_t face, uint32_t mip, float u0, float v0,
                            float u1, float v1, bool predicted) {
    Tenant& t = m_tenants[tenant];
    if (mip >= t.mips) mip = t.mips - 1;

    // M9w: THE ANCESTOR RE-WALK, SHORT-CIRCUITED.
    //
    // The mip-tail rule below is right -- wanting a fine tile implies wanting every ancestor,
    // and ancestors must be enqueued FIRST so the coarse-before-fine mapping invariant holds.
    // But it was being paid PER LEAF, and a coarse tile has many descendants: measured at
    // 23947 tile touches per frame of which 23947 -- every single one -- were already tracked.
    // 36.7 hash lookups per leaf, inserting nothing, costing 66% of SetView's 1.78 ms.
    //
    // The saving observation: the tail always refreshes the WHOLE column, coarsest to finest,
    // in one call. So if the FINEST tile of a rect already carries this frame's stamp, every
    // ancestor of it does too, and the entire column can be skipped. The check below is that
    // test, done once per finest tile instead of once per level per tile.
    //
    // The `predicted` clause is the part that would have been a silent bug. A repeat touch also
    // DOWNGRADES a tile from predicted to real, so skipping a stamped tile must not skip that
    // transition -- a tile the prefetch asked for and the real view then confirmed would stay
    // flagged speculative and be evicted on the wrong budget. Skipping is only safe when this
    // touch cannot change the flag.
    // Frames are stamped OFFSET BY ONE so that a zero-initialised array reads as "never seen".
    // m_frame starts at 0 and is incremented in ProcessQueues, i.e. AFTER this walk -- so on the
    // very first frame a raw encoding would have made every untouched tile look already-stamped
    // and skipped the entire frame's wants.
    const auto kStampFrame = [this] { return m_frame + 1u; };
    // The stamp this touch would write: frame in the high bits, predicted in bit 0.
    const uint32_t myStamp = (kStampFrame() << 1) | (predicted ? 1u : 0u);
    {
        const uint32_t plane = face * t.mips + mip;
        const auto& fine = t.tilings[plane];
        const uint32_t tw = fine.WidthInTiles, th = fine.HeightInTiles;
        const uint32_t x0 = static_cast<uint32_t>((std::max)(0.0f, u0) * tw);
        const uint32_t y0 = static_cast<uint32_t>((std::max)(0.0f, v0) * th);
        const uint32_t x1 = (std::min)(tw - 1, static_cast<uint32_t>((std::min)(0.9999f, u1) * tw));
        const uint32_t y1 = (std::min)(th - 1, static_cast<uint32_t>((std::min)(0.9999f, v1) * th));
        bool allFresh = true;
        for (uint32_t y = y0; y <= y1 && allFresh; ++y) {
            const uint32_t* row = t.stamp.data() + t.stampBase[plane] + size_t(y) * t.stampW[plane];
            for (uint32_t x = x0; x <= x1; ++x) {
                ++wantTouches;
                const uint32_t s = row[x];
                // Fresh means this frame AND not a real touch arriving on a speculative tile
                // -- that transition must still run, or a tile the prefetch asked for and the
                // view then confirmed stays flagged speculative and is evicted on the wrong
                // budget.
                if ((s >> 1) != kStampFrame() || (!predicted && (s & 1u))) {
                    allFresh = false;
                    break;
                }
                ++wantHits;
            }
        }
        if (allFresh) return;
    }

    for (int m = static_cast<int>(t.mips) - 1; m >= static_cast<int>(mip); --m) {
        bool held = false;   // M9al: a request at this level waited for its parent
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
            const uint32_t plane = face * t.mips + static_cast<uint32_t>(m);
            for (uint32_t x = x0; x <= x1; ++x) {
                TileRequest r{face, static_cast<uint32_t>(m), x, y};
                ++wantTouches;
                // The stamp answers "already handled this frame" without touching the map at
                // all. It is written on every path below, so it stays exactly as true as
                // lastSeen/predicted are -- they remain the authority for eviction; this is a
                // cache of the one question the walk asks.
                uint32_t& st = t.stamp[StampIndex(t, face, static_cast<uint32_t>(m), x, y)];
                if ((st >> 1) == kStampFrame() && !(!predicted && (st & 1u))) {
                    ++wantHits;
                    continue;
                }
                const Key k = MakeKey(tenant, r);
                auto it = m_tracked.find(k);
                if (it != m_tracked.end()) {
                    it->second->lastSeen = m_frame;
                    if (!predicted) it->second->predicted = false;
                    st = (kStampFrame() << 1) | (it->second->predicted ? 1u : 0u);
                    continue;
                }
                // M9al: THE RING GATE. A new request below the coarsest level is admitted only
                // if its parent is already MAPPED. The tile stays unstamped, so next frame's
                // walk asks again -- by which time the parent has landed, or has not, and the
                // answer is the same question one ring later. Nothing is lost, and no load slot
                // is ever spent on a tile more than one level from being displayable.
                if (ringLoads && m < static_cast<int>(t.mips) - 1) {
                    const TileRequest pr{face, static_cast<uint32_t>(m + 1), x >> 1, y >> 1};
                    const auto pit = m_tracked.find(MakeKey(tenant, pr));
                    if (pit == m_tracked.end() || pit->second->state != TileState::Mapped) {
                        held = true;
                        ++ringHeld;
                        ++ringHeldFrame;
                        continue;
                    }
                }
                st = myStamp;
                auto tr = std::make_shared<Tracked>();
                tr->tenant = tenant;
                tr->req = r;
                tr->lastSeen = m_frame;
                tr->predicted = predicted;
                m_tracked[k] = tr;
                m_seen.push_back(tr);
            }
        }
        // A level with a missing parent cannot have a mapped child: stop the column here and
        // let the next frame's walk resume one ring finer.
        if (held) return;
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

    // ---- M9ai: RETIRE LANDED DIRECT READS. A tile mapped last frame and read by DirectStorage
    // becomes claimable only once its fence signals. Until then it has been mapped but NOT
    // claimed, so every consumer has been reading its coarser ancestor -- the same degradation
    // a not-yet-arrived tile has always produced, which is why this needs no barrier and no
    // graphics-queue wait to be correct.
    if (m_stream && !m_inFlightReads.empty()) {
        for (auto it = m_inFlightReads.begin(); it != m_inFlightReads.end();) {
            if (!m_stream->Complete(it->fence)) { ++it; continue; }
            for (auto& tile : it->tiles) {
                // Evicted while in flight: its slot is gone and the claim would be a lie.
                if (tile->state != TileState::Mapped) continue;
                Tenant& t = m_tenants[tile->tenant];
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
                const D3D12_TILED_RESOURCE_COORDINATE coord{
                    tile->req.x, tile->req.y, 0, tile->req.face * t.mips + tile->req.mip};
                const D3D12_TILE_REGION_SIZE size{1, FALSE, 0, 0, 0};
                cl->CopyTiles(t.res.Get(), &coord, &size, m_stream->StagingBuffer(),
                              tile->stageOffset,
                              D3D12_TILE_COPY_FLAG_LINEAR_BUFFER_TO_SWIZZLED_TILED_RESOURCE);
                UpdateResidencyByte(t, tile->req, true);
                ++m_directLanded;
            }
            if (!m_directLogged && m_directLanded) {
                m_directLogged = true;
                Log("[dstorage] first %llu tiles landed NVMe -> GPU (fence %llu); the CPU never "
                    "addressed their bytes",
                    static_cast<unsigned long long>(m_directLanded),
                    static_cast<unsigned long long>(it->fence));
            }
            // Every slot of this batch is drained by the copies just RECORDED. They execute when
            // this frame's list runs, so the slots retire and come back kStageRetireFrames
            // later -- never in time for this frame's MapAndFill to hand them out again.
            {
                std::vector<uint32_t> slots;
                for (auto& tile : it->tiles) {
                    slots.push_back(static_cast<uint32_t>(tile->stageOffset / 65536ull));
                }
                m_stageRetire.push_back({m_frame, std::move(slots)});
            }
            m_stream->ReleaseCompleted(it->fence);
            it = m_inFlightReads.erase(it);
        }
    }

    // ---- M9al: the instrument. Deficit by mip per tenant, queue depths, slots by kind.
    if (traceRes && (m_frame % (std::max)(1u, traceEvery)) == 0) {
        std::lock_guard<std::mutex> lk(m_mx);
        std::vector<std::vector<uint32_t>> deficit(m_tenants.size());
        for (size_t t = 0; t < m_tenants.size(); ++t) deficit[t].assign(m_tenants[t].mips, 0);
        uint32_t seenNow = 0, loadingNow = 0;
        // m_seen and m_loading ARE the wanted-and-unmapped set by construction. A frame-stamp
        // filter here asked a question whose answer depended on call order, and printed zero
        // beside a queue of 834; count the queues, and say how stale their entries are.
        uint32_t stale = 0, predictedN = 0;
        auto count = [&](const std::shared_ptr<Tracked>& tr) {
            if (tr->predicted) { ++predictedN; return; }
            if (m_frame - tr->lastSeen > 2) ++stale;
            if (tr->state == TileState::Mapped) return;
            if (tr->tenant < 0 || size_t(tr->tenant) >= deficit.size()) return;
            if (tr->req.mip < deficit[tr->tenant].size()) ++deficit[tr->tenant][tr->req.mip];
        };
        for (const auto& tr : m_seen) { count(tr); ++seenNow; }
        for (const auto& tr : m_loading) { count(tr); ++loadingNow; }
        Log("[res-trace] f%u queue: seen %u, loading %u (%u predicted, %u unseen >2 frames), "
            "in flight %d (of %u), ring-held %u this frame / %u total",
            m_frame, seenNow, loadingNow, predictedN, stale, m_inFlight.load(),
            kMaxLoadsInFlight, ringHeldFrame, ringHeld);
        for (size_t t = 0; t < m_tenants.size(); ++t) {
            const Tenant& tn = m_tenants[t];
            uint32_t tot = 0;
            std::string hist;
            for (uint32_t m = 0; m < tn.mips; ++m) {
                tot += deficit[t][m];
                if (deficit[t][m]) {
                    char b[24];
                    snprintf(b, sizeof(b), " m%u:%u", m, deficit[t][m]);
                    hist += b;
                }
            }
            if (!tot && !tn.loadsPaint && !tn.loadsRead) continue;
            const uint32_t rp = tn.loadsPaint, rr = tn.loadsRead;
            Log("[res-trace]   %-44S deficit %u%s | slots: %u reads (%.2f ms avg), %u paints "
                "(%.1f ms avg)",
                tn.name.c_str(), tot, hist.c_str(), rr, rr ? tn.readUs / 1000.0 / rr : 0.0,
                rp, rp ? tn.paintUs / 1000.0 / rp : 0.0);
        }
    }
    ringHeldFrame = 0;

    // ---- start loads (newest-seen first, coarse first; predicted tiles yield to real ones)
    {
        std::lock_guard<std::mutex> lk(m_mx);
        // M7w: failed loads parked in m_loading with state Seen get their retry first.
        for (auto& tile : m_loading) {
            if (tile->state != TileState::Seen) continue;
            if (m_inFlight >= static_cast<int>(kMaxLoadsInFlight)) break;
            tile->state = TileState::Loading;
            m_loadQueue.push_back(tile);
            ++m_inFlight;
            m_cv.notify_one();
        }
        // M9af: READY BEFORE UNREADY. A tile already on the NVMe is a 64 KB read; one that is
        // not is a paint of 16384 samples and, for a tile-tree source, a network fetch behind a
        // budget. Those differ by orders of magnitude and the queue could not tell them apart,
        // so a frame's worth of load slots could all go to painting while cached tiles that
        // would have filled the picture immediately waited behind them.
        //
        // It sits UNDER the real/predicted rule -- a speculative tile being cheap is not a
        // reason to serve it before a tile the camera is actually looking at -- and OVER
        // recency, because among tiles the view wants now, the cheap ones should land first.
        // Coarse-before-fine stays last, where the mapping invariant needs it.
        std::sort(m_seen.begin(), m_seen.end(), [](const auto& a, const auto& b) {
            if (a->predicted != b->predicted) return !a->predicted;
            if (a->lastSeen != b->lastSeen) return a->lastSeen > b->lastSeen;
            return a->req.mip > b->req.mip;
        });
        // M9af: READINESS ORDERING WAS TRIED HERE AND MEASURED WORSE. See the note on
        // ResidencyManager::SetTileIndex -- preferring tiles already on the NVMe starves the
        // painting that puts them there, and this scene's cache is ~1% warm at the finest level.
        // The index stays (the DirectStorage read path needs it); the scheduling preference does
        // not, because the picture got worse and the picture decides.
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
            if (tile->state == TileState::Failed) {
                // Honestly unmapped forever: it stays in m_tracked (Want() only bumps
                // lastSeen), consumers keep reading the coarser real mip.
                it = m_loading.erase(it);
                continue;
            }
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
        gpu.Transition(cl, t.resMap, D3D12_RESOURCE_STATE_PIXEL_SHADER_RESOURCE |
                                         D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);
    }

    // ---- stats
    char s[256];
    uint64_t bytes = 0;
    for (const auto& tr : m_mapped) {
        (void)tr;
        bytes += 65536;
    }
    snprintf(s, sizeof(s), "streams %zu res (%zu t, %.0f MB, %u fetches, %llu direct/%llu ring%s)",
             m_tenants.size(), m_mapped.size(), bytes / 1048576.0, fetchesThisRun,
             static_cast<unsigned long long>(m_directTiles),
             static_cast<unsigned long long>(m_ringTiles),
             m_failedLoads ? " FAILED-TILES" : "");
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
        // A ring-filled tile is readable the moment this frame's list executes, so it may claim
        // residency now. A DIRECT-READ tile is only mapped -- its bytes are still crossing the
        // bus on another queue -- so the claim waits for the fence. Claiming early is exactly
        // the M7w failure in a new costume: the residency map would swear data exists where the
        // GPU still holds whatever the pool slot had.
        if (!(tile->loc.Valid() && m_stream && m_stream->Available())) {
            UpdateResidencyByte(t, tile->req, true);
        }
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
    std::vector<std::shared_ptr<Tracked>> direct;
    for (const auto& tile : toFill) {
        Tenant& t = m_tenants[tile->tenant];
        const D3D12_TILED_RESOURCE_COORDINATE coord{
            tile->req.x, tile->req.y, 0, tile->req.face * t.mips + tile->req.mip};
        // ---- M9ai: NVMe -> GPU where the tile is archived. The mapping was issued above, so
        // the destination tile exists; DirectStorage writes into it on its own queue while this
        // command list is still being recorded.
        // ---- M9ai: the read lands in a DEVICE buffer; the swizzle stays on the GPU where the
        // format requires it, and happens next frame when the fence says the bytes arrived.
        if (!m_stageInit) {
            m_stageInit = true;
            for (uint32_t s = TileStream::kStageSlots; s-- > 0;) m_stageFree.push_back(s);
        }
        while (!m_stageRetire.empty() &&
               m_stageRetire.front().first + kStageRetireFrames <= m_frame) {
            for (uint32_t s : m_stageRetire.front().second) m_stageFree.push_back(s);
            m_stageRetire.pop_front();
        }
        // DIAGNOSTIC (M9ap): dsSerial allows one DirectStorage batch in flight at a time; a tile
        // that would overlap a pending batch takes the ring instead.
        if (tile->loc.Valid() && m_stream && m_stream->Available() && !m_stageFree.empty() &&
            !(dsSerial && !m_inFlightReads.empty())) {
            const uint32_t slot = m_stageFree.back();
            const uint64_t slotOff = uint64_t(slot) * 65536ull;
            if (m_stream->EnqueueToBuffer(tile->loc.path, tile->loc.offset, tile->loc.size,
                                          slotOff)) {
                m_stageFree.pop_back();
                tile->stageOffset = slotOff;
                direct.push_back(tile);
                ++m_directTiles;
                continue;
            }
        }
        const D3D12_TILE_REGION_SIZE size{1, FALSE, 0, 0, 0};
        const size_t n = tile->data.size() < 65536 ? tile->data.size() : 65536;
        memcpy(ring.cpu + off, tile->data.data(), n);
        cl->CopyTiles(t.res.Get(), &coord, &size, ring.res.Get(), off,
                      D3D12_TILE_COPY_FLAG_LINEAR_BUFFER_TO_SWIZZLED_TILED_RESOURCE);
        off += 65536;
        ++m_ringTiles;
        tile->data.clear();
        tile->data.shrink_to_fit();
    }
    if (!direct.empty()) {
        InFlightRead f;
        f.fence = m_stream->Submit();
        f.tiles = std::move(direct);
        m_inFlightReads.push_back(std::move(f));
    }
    for (auto& t : m_tenants) {
        D3D12_RESOURCE_BARRIER b{};
        b.Type = D3D12_RESOURCE_BARRIER_TYPE_TRANSITION;
        b.Transition.pResource = t.res.Get();
        b.Transition.Subresource = D3D12_RESOURCE_BARRIER_ALL_SUBRESOURCES;
        b.Transition.StateBefore = t.state;
        b.Transition.StateAfter = D3D12_RESOURCE_STATE_PIXEL_SHADER_RESOURCE |
                                      D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE;   // M9ar: compute reads too
        cl->ResourceBarrier(1, &b);
        t.state = D3D12_RESOURCE_STATE_PIXEL_SHADER_RESOURCE |
                  D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE;
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
