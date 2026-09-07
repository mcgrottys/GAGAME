#include "core/Residency.h"

#include <atomic>

#include "core/PixEvents.h"
#include "core/TileAtlas.h"   // Cl2ProductSignature -- the proven closure drives DeriveDemand

#include <algorithm>
#include <chrono>
#include <cstring>

namespace ga {

void ResidencyManager::Init(Gpu& gpu) {
    m_gpu = &gpu;
    for (uint32_t i = 0; i < Gpu::kFrameCount; ++i) {
        // M9bb: the tile slab, plus a RESERVE for the residency maps -- one region per
        // tenant, never the slab's tail (see the map refresh below for what sharing it did).
        m_uploadRing[i] = gpu.CreateUploadBuffer(
            static_cast<uint64_t>(kMaxMapsPerFrame) * 65536 + kMapStageBytes,
            L"residency upload ring");
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
            // Step 4: Want()'s column scan rests on a plane's rect being exactly the parents
            // of the finer plane's rect, which is true to the bit only when every plane is a
            // power of two tiles on each axis (then u * tw halves exactly from plane to
            // plane). Every tenant today is (16384-texel faces, 512x256 / 256x128 / 128x128
            // tiles); a tenant that were not would silently change the request stream, so
            // it is refused here instead.
            const uint32_t w = t.tilings[i].WidthInTiles, h = t.tilings[i].HeightInTiles;
            GA_CHECK((w & (w - 1)) == 0 && (h & (h - 1)) == 0 && w && h ? S_OK : E_UNEXPECTED);
        }
        t.stamp.assign(acc, 0u);
        t.slot.assign(acc, nullptr);   // step 4: the Tracked at the same index
        Log("[residency] %S: stamp array %u tiles (%.2f MB) + slot array (%.2f MB) -- Want()'s "
            "hot question and its tile both leave the map",
            name, acc, acc * 4.0 / 1048576.0, acc * double(sizeof(Tracked*)) / 1048576.0);
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
                Track(tn, tr);
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
    if (predicted) {
        // Step 5: the predicted stream's hash (see Residency.h). The arguments as the caller
        // gave them, before the mip clamp -- the stream the WALK emitted is the object.
        uint32_t w[7] = {static_cast<uint32_t>(tenant), face, mip, 0u, 0u, 0u, 0u};
        memcpy(&w[3], &u0, 4);
        memcpy(&w[4], &v0, 4);
        memcpy(&w[5], &u1, 4);
        memcpy(&w[6], &v1, 4);
        const uint8_t* b = reinterpret_cast<const uint8_t*>(w);
        uint64_t h = predictedHash;
        for (size_t i = 0; i < sizeof(w); ++i) {
            h ^= b[i];
            h *= 1099511628211ull;
        }
        predictedHash = h;
        ++predictedCalls;
    }
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
    const uint32_t stampFrame = m_frame + 1u;
    // The stamp this touch would write: frame in the high bits, predicted in bit 0.
    const uint32_t myStamp = (stampFrame << 1) | (predicted ? 1u : 0u);
    // Fresh means this frame AND not a real touch arriving on a speculative tile -- that
    // transition must still run, or a tile the prefetch asked for and the view then confirmed
    // stays flagged speculative and is evicted on the wrong budget.
    const auto fresh = [&](uint32_t s) {
        return (s >> 1) == stampFrame && !(!predicted && (s & 1u));
    };
    // The rect in one plane's tiles. ONE arithmetic for every level: the column scan below
    // rests on rect(m + 1) being exactly the parents of rect(m), which holds because every
    // plane is a power of two tiles wide (faceDim and the tile shape both are), so u * tw at
    // the coarser plane is u * tw / 2 to the bit and its floor is the finer floor >> 1.
    struct Rect {
        uint32_t x0, y0, x1, y1;
    };
    const auto rectAt = [&](uint32_t plane) {
        const auto& ti = t.tilings[plane];
        const uint32_t tw = ti.WidthInTiles;
        const uint32_t th = ti.HeightInTiles;   // UINT16 in the D3D struct; widen once
        Rect r;
        r.x0 = static_cast<uint32_t>((std::max)(0.0f, u0) * tw);
        r.y0 = static_cast<uint32_t>((std::max)(0.0f, v0) * th);
        r.x1 = (std::min)(tw - 1, static_cast<uint32_t>((std::min)(0.9999f, u1) * tw));
        r.y1 = (std::min)(th - 1, static_cast<uint32_t>((std::min)(0.9999f, v1) * th));
        return r;
    };

    // Step 4 (docs/PERF_EXPERIMENT.md): THE COLUMN SCAN, BOTTOM-UP. The M9w test above only
    // paid off when the WHOLE finest rect was fresh, which two leaves that share a parent but
    // not a tile never satisfy; every other call re-read the stamp of every ancestor level for
    // the whole rect. The column invariant is stronger than that test used it: a tile is
    // stamped only after its entire ancestor column was stamped in the same call (the walk
    // is coarse-to-fine and stops at a held level before stamping anything finer), and
    // rect(m + 1) is exactly the parents of rect(m). So the first level, scanning from the
    // requested mip UPWARD, whose rect is entirely fresh has nothing but fresh levels above
    // it, and the walk below can start one level under it. That walk is the same code in
    // the same order, so the request stream is unchanged; what is gone is the stamp reads of
    // the levels above `top`. The predicted-to-real downgrade is exact for the same reason
    // the M9w test was: a real touch never finds a real-stamped child under a speculative
    // parent, because a call stamps its whole column with one flag and no tenant is touched
    // predicted-then-real within one frame -- the frame's only predicted caller is
    // PredictWants, which runs after SetView on the globe's tenants and never touches the
    // wave or exposure tenants the later Wants ask for. A ring-held ancestor is unstamped by
    // design and reads as not fresh, so it keeps the scan climbing, never stops it.
    int top = -1;   // coarsest level with work; -1 = the whole column is fresh
    for (uint32_t m = mip; m < t.mips; ++m) {
        const uint32_t plane = face * t.mips + m;
        const Rect r = rectAt(plane);
        bool allFresh = true;
        for (uint32_t y = r.y0; y <= r.y1 && allFresh; ++y) {
            const uint32_t* row =
                t.stamp.data() + t.stampBase[plane] + size_t(y) * t.stampW[plane];
            for (uint32_t x = r.x0; x <= r.x1; ++x) {
                ++wantTouches;
                if (!fresh(row[x])) {
                    allFresh = false;
                    break;
                }
                ++wantHits;
            }
        }
        if (allFresh) break;
        top = static_cast<int>(m);
    }
    if (top < 0) return;

    for (int m = top; m >= static_cast<int>(mip); --m) {
        bool held = false;   // M9al: a request at this level waited for its parent
        const uint32_t plane = face * t.mips + static_cast<uint32_t>(m);
        const Rect r = rectAt(plane);
        for (uint32_t y = r.y0; y <= r.y1; ++y) {
            const size_t rowBase = size_t(t.stampBase[plane]) + size_t(y) * t.stampW[plane];
            uint32_t* stampRow = t.stamp.data() + rowBase;
            Tracked** slotRow = t.slot.data() + rowBase;
            for (uint32_t x = r.x0; x <= r.x1; ++x) {
                ++wantTouches;
                // The stamp answers "already handled this frame" without touching the tile at
                // all. It is written on every path below, so it stays exactly as true as
                // lastSeen/predicted are -- they remain the authority for eviction; this is a
                // cache of the one question the walk asks.
                uint32_t& st = stampRow[x];
                if (fresh(st)) {
                    ++wantHits;
                    continue;
                }
                // Step 4: the tile itself sits at the same index. No key, no tree.
                if (Tracked* tr = slotRow[x]) {
                    tr->lastSeen = m_frame;
                    if (!predicted) tr->predicted = false;
                    st = (stampFrame << 1) | (tr->predicted ? 1u : 0u);
                    continue;
                }
                // M9al: THE RING GATE. A new request below the coarsest level is admitted only
                // if its parent is already MAPPED. The tile stays unstamped, so next frame's
                // walk asks again -- by which time the parent has landed, or has not, and the
                // answer is the same question one ring later. Nothing is lost, and no load slot
                // is ever spent on a tile more than one level from being displayable.
                if (ringLoads && m < static_cast<int>(t.mips) - 1) {
                    const Tracked* parent =
                        t.slot[StampIndex(t, face, static_cast<uint32_t>(m + 1), x >> 1, y >> 1)];
                    if (!parent || parent->state != TileState::Mapped) {
                        held = true;
                        ++ringHeld;
                        ++ringHeldFrame;
                        // Step 25: held under a parent that will never map -- the exact
                        // settle's "unreachable", never its deficit. The request stream is
                        // the same either way; this is a count.
                        if (parent && parent->state == TileState::Failed) ++ringHeldDeadFrame;
                        continue;
                    }
                }
                st = myStamp;
                auto tr = std::make_shared<Tracked>();
                tr->tenant = tenant;
                tr->req = TileRequest{face, static_cast<uint32_t>(m), x, y};
                tr->lastSeen = m_frame;
                tr->predicted = predicted;
                Track(t, tr);
                m_seen.push_back(tr);
            }
        }
        // A level with a missing parent cannot have a mapped child: stop the column here and
        // let the next frame's walk resume one ring finer.
        if (held) return;
    }
}

void ResidencyManager::Track(Tenant& t, const std::shared_ptr<Tracked>& tr) {
    Tracked*& s = t.slot[StampIndex(t, tr->req.face, tr->req.mip, tr->req.x, tr->req.y)];
    if (s) throw std::runtime_error("residency: Track() over a live slot");
    s = tr.get();
    tr->pos = static_cast<uint32_t>(t.tracked.size());
    t.tracked.push_back(tr);
}

void ResidencyManager::Untrack(Tenant& t, Tracked* tr) {
    Tracked*& s = t.slot[StampIndex(t, tr->req.face, tr->req.mip, tr->req.x, tr->req.y)];
    if (s != tr) throw std::runtime_error("residency: Untrack() of a tile its slot does not hold");
    s = nullptr;
    const uint32_t i = tr->pos;
    tr->pos = UINT32_MAX;   // before the list drops what may be the last reference
    if (i + 1 != t.tracked.size()) {
        t.tracked[i] = std::move(t.tracked.back());
        t.tracked[i]->pos = i;
    }
    t.tracked.pop_back();
}

void ResidencyManager::AuditSlots() const {
    size_t total = 0;
    for (size_t k = 0; k < m_tenants.size(); ++k) {
        const Tenant& t = m_tenants[k];
        size_t live = 0;
        for (const Tracked* p : t.slot) live += p != nullptr;
        bool ok = live == t.tracked.size();
        for (size_t i = 0; ok && i < t.tracked.size(); ++i) {
            const Tracked* p = t.tracked[i].get();
            ok = p->pos == i && p->tenant == static_cast<int>(k) &&
                 t.slot[StampIndex(t, p->req.face, p->req.mip, p->req.x, p->req.y)] == p;
        }
        if (!ok) {
            Log("[residency] SLOT AUDIT FAILED f%u %S: %zu live slots, %zu tracked", m_frame,
                t.name.c_str(), live, t.tracked.size());
            throw std::runtime_error("residency slot audit failed (see log)");
        }
        total += t.tracked.size();
    }
    if (m_frame == kSlotAuditFrames) {
        Log("[residency] slot audit: %u frames, %zu tracked tiles across %zu tenants -- every "
            "live slot is a list entry and every entry's slot points back at it",
            m_frame, total, m_tenants.size());
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

bool ResidencyManager::DropOne(const std::shared_ptr<Tracked>& tr) {
    tr->dropped = true;
    if (tr->state != TileState::Mapped) return false;
    UpdateResidencyByte(m_tenants[tr->tenant], tr->req, false);
    m_retiring.push_back({tr, m_frame});
    return true;
}

void ResidencyManager::Invalidate(int tenant, const TileRequest& r) {
    std::lock_guard<std::mutex> lk(m_invMx);
    m_invQ.push_back({tenant, r});
}

void ResidencyManager::Drop(int tenant) {
    Tenant& t = m_tenants[tenant];
    // Step 4: this tenant's own list, in the order the map walked it (face, mip, y, x) so
    // m_retiring and everything downstream of it -- the NULL-map calls, the freed pool slots
    // -- come out in the sequence they always did. Every slot is cleared here; the list is
    // taken whole, so this is O(dropped log dropped) plus ONE pass over m_mapped instead of a
    // walk over every tenant's tiles and a std::find over m_mapped per dropped tile.
    std::vector<std::shared_ptr<Tracked>> all;
    all.swap(t.tracked);
    std::sort(all.begin(), all.end(), [tenant](const auto& a, const auto& b) {
        return MakeKey(tenant, a->req) < MakeKey(tenant, b->req);
    });
    uint32_t mapped = 0, inflight = 0;
    for (auto& tr : all) {
        t.slot[StampIndex(t, tr->req.face, tr->req.mip, tr->req.x, tr->req.y)] = nullptr;
        tr->pos = UINT32_MAX;
        if (DropOne(tr)) ++mapped; else ++inflight;
    }
    if (mapped) {
        std::erase_if(m_mapped, [](const std::shared_ptr<Tracked>& p) { return p->dropped; });
    }
    Log("[residency] %S: dropped %u mapped tiles (retire after %u frames) and %u in flight",
        t.name.c_str(), mapped, kEvictAgeFrames, inflight);
}

void ResidencyManager::ProcessQueues(Gpu& gpu, ID3D12GraphicsCommandList* cl) {
    PixScope scope(cl, "residency (seen->load->map->fill; classic lists, screw prefetch)");
    // The phase brackets (see phaseMs in the header). `lap` closes the phase that began at the
    // previous lap; a block that is not a phase (the --res-trace print) resets the lap clock.
    using Clock = std::chrono::steady_clock;
    const auto turn0 = Clock::now();
    auto lap0 = turn0;
    for (double& p : phaseMs) p = 0.0;
    turn = TurnLedger{};   // step 28: the landing ledger, this turn's
    auto lap = [&](int k) {
        const auto t = Clock::now();
        phaseMs[k] += std::chrono::duration<double, std::milli>(t - lap0).count();
        lap0 = t;
    };
    ++m_frame;
    // Step 4: the slot array's bring-up gate rides the trace flag for the first thousand
    // frames (a full scan of every slot array; not a cost the bench ever pays).
    if (traceRes && m_frame <= kSlotAuditFrames) AuditSlots();
    // M9bb: apply the invalidations the painting threads queued (one tile each).
    {
        std::vector<std::pair<int, TileRequest>> q;
        {
            std::lock_guard<std::mutex> lk(m_invMx);
            q.swap(m_invQ);
        }
        bool compact = false;
        for (const auto& [tenant, r] : q) {
            Tracked* tr = Find(tenant, r);
            if (!tr) continue;
            Tenant& t = m_tenants[tenant];
            const std::shared_ptr<Tracked> keep = t.tracked[tr->pos];   // outlives Untrack
            Untrack(t, tr);
            if (DropOne(keep)) compact = true;
        }
        if (compact) {
            std::erase_if(m_mapped, [](const std::shared_ptr<Tracked>& p) { return p->dropped; });
        }
    }
    // M9ba: NULL-map the dropped tiles whose overlap window has passed; free their slots.
    for (auto it = m_retiring.begin(); it != m_retiring.end();) {
        if (m_frame - it->frame < kEvictAgeFrames) { ++it; continue; }
        Tenant& t = m_tenants[it->tile->tenant];
        // If this coordinate was re-requested and re-mapped since the drop (the bucket rolled
        // and the same tile came straight back under the new identity), the NEW mapping owns
        // the coordinate: NULL-mapping it here would unmap the fresh tile -- which is exactly
        // what happened (the exposure page read zeros through a residency byte that said
        // "mapped"). Only free the old slot in that case.
        const Tracked* cur = Find(it->tile->tenant, it->tile->req);
        const bool remapped = cur && cur->state == TileState::Mapped && cur != it->tile.get();
        if (!remapped) {
            const D3D12_TILED_RESOURCE_COORDINATE coord{
                it->tile->req.x, it->tile->req.y, 0,
                it->tile->req.face * t.mips + it->tile->req.mip};
            const D3D12_TILE_REGION_SIZE size{1, FALSE, 0, 0, 0};
            const D3D12_TILE_RANGE_FLAGS flag = D3D12_TILE_RANGE_FLAG_NULL;
            const UINT off = 0, cnt = 1;
            gpu.Queue()->UpdateTileMappings(t.res.Get(), 1, &coord, &size, nullptr, 1, &flag,
                                            &off, &cnt, D3D12_TILE_MAPPING_FLAG_NONE);
        }
        if (it->tile->pool != UINT32_MAX) m_freePool.push_back(it->tile->pool);
        it->tile->state = TileState::Failed;
        it = m_retiring.erase(it);
    }
    lap(0);

    // ---- M9ai: RETIRE LANDED DIRECT READS. A tile mapped last frame and read by DirectStorage
    // becomes claimable only once its fence signals. Until then it has been mapped but NOT
    // claimed, so every consumer has been reading its coarser ancestor -- the same degradation
    // a not-yet-arrived tile has always produced, which is why this needs no barrier and no
    // graphics-queue wait to be correct.
    if (m_stream && !m_inFlightReads.empty()) {
        for (auto it = m_inFlightReads.begin(); it != m_inFlightReads.end();) {
            if (!m_stream->Complete(it->fence)) { ++it; continue; }
            // Step 28: the ledger's lag -- how many turns this batch held its landing slots.
            {
                const uint32_t lag = m_frame - it->frame;
                if (!turn.landedBatches || lag < turn.lagMin) turn.lagMin = lag;
                if (lag > turn.lagMax) turn.lagMax = lag;
                ++turn.landedBatches;
            }
            for (auto& tile : it->tiles) {
                // Evicted while in flight: its slot is gone and the claim would be a lie.
                if (tile->state != TileState::Mapped) { ++turn.landedRetired; continue; }
                // AND THE CLAIM MUST BE ITS OWN. A tile that was dropped, evicted at the cap or
                // re-tracked since its read was issued no longer owns its coordinate: the copy
                // below would swizzle the OLD identity's bytes into a slot that now belongs to
                // something else, and UpdateResidencyByte would then swear real data is there.
                // Measured before this skip existed: 85 such claims over the 14:00 flood rail
                // and 126 over the 19:30Z one. The slot still retires -- that loop is below and
                // runs over every tile of the batch -- so skipping here leaks nothing.
                if (tile->dropped || Find(tile->tenant, tile->req) != tile.get()) {
                    ++turn.landedUnowned;
                    ++landedUnownedTotal;
                    continue;
                }
                ++turn.landedTiles;
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
    lap(1);

    // ---- Step 25: THE EXACT SETTLE (Residency.h settleExact). Harness only: main raises the
    // flag for the held frames of a --settle-exact still, and outside a hold this block is
    // never entered. It runs AFTER the landed reads claimed their tiles (a tile that arrived
    // this turn is mapped, not deficit) and BEFORE the loads start and the batch is gathered
    // (a stale pending tile is dropped before it takes a load slot or a map).
    settleTurn = SettleTurn{};
    if (settleExact) {
        std::lock_guard<std::mutex> lk(m_mx);
        // Mapped but not yet claimed: its bytes are still crossing the bus on a DirectStorage
        // batch, and the landed loop would claim a dropped tile (state Mapped until it
        // retires) into a byte the drop had just raised. Such a tile waits for its fence and
        // is dropped on a later turn.
        std::vector<const Tracked*> reading;
        for (const auto& b : m_inFlightReads) {
            for (const auto& tile : b.tiles) reading.push_back(tile.get());
        }
        std::sort(reading.begin(), reading.end());
        // THE CAP'S CASUALTIES. MEASURED (bird, first --settle-exact run, 2026-09-05): 988
        // wave.field mip-0 tiles wanted, in state Loaded, tracked, in no queue -- the mapped
        // sets summed to exactly kPoolCapTiles (8192) against a want set of 9176 tiles
        // (573 MB), and MapAndFill's cap branch, finding no victim (every mapped tile is
        // wanted every frame), breaks out of a batch the gather had already erased from
        // m_loading. Outside a hold that is the shipped behaviour and the reason a quiet
        // --settle-sync fires over a race-decided resident set; here the hold's pool holds the
        // want set (MapAndFill), so a Loaded tile in no queue goes back to m_loading and maps.
        std::vector<const Tracked*> queued;
        queued.reserve(m_loading.size());
        for (const auto& tile : m_loading) queued.push_back(tile.get());
        std::sort(queued.begin(), queued.end());
        // Can this wanted, unmapped tile ever land? Climb its ancestor column to the first
        // mapped tile: a Failed tile on the way (or the tile itself) means the ring gate and
        // the mapping invariant keep it out forever -- unreachable, by state. A missing
        // ancestor is not dead: the walk re-requests it next frame (the column stamps
        // coarse-to-fine, so a wanted tile's ancestors are wanted too).
        auto reachable = [&](const Tenant& t, const Tracked* tr) {
            for (const Tracked* p = tr;;) {
                if (p->state == TileState::Failed) return false;
                if (p->state == TileState::Mapped) return true;
                if (p->req.mip + 1 >= t.mips) return true;   // the floor: mapped at birth
                p = t.slot[StampIndex(t, p->req.face, p->req.mip + 1, p->req.x >> 1,
                                      p->req.y >> 1)];
                if (!p) return true;
            }
        };
        bool compact = false;
        for (Tenant& t : m_tenants) {
            uint32_t wanted = 0, mapped = 0, deficit = 0, unreachable = 0, stale = 0,
                     dropped = 0, droppedMapped = 0, requeued = 0;
            for (size_t i = 0; i < t.tracked.size();) {
                Tracked* tr = t.tracked[i].get();
                const size_t idx = StampIndex(t, tr->req.face, tr->req.mip, tr->req.x, tr->req.y);
                // Want() stamps m_frame + 1 and this turn incremented m_frame: this frame's
                // walks wrote exactly m_frame (predicted or not -- no predicted walk runs
                // during a hold, main suspends it).
                if ((t.stamp[idx] >> 1) == m_frame) {
                    ++wanted;
                    if (tr->state == TileState::Mapped) {
                        ++mapped;
                    } else {
                        if (tr->state == TileState::Loaded &&
                            !std::binary_search(queued.begin(), queued.end(), tr)) {
                            m_loading.push_back(t.tracked[i]);
                            ++requeued;
                        }
                        if (reachable(t, tr)) ++deficit;
                        else ++unreachable;
                    }
                    ++i;
                    continue;
                }
                if (tr->req.mip + 1 == t.mips) {   // the coarsest mip is never dropped
                    ++i;
                    continue;
                }
                const bool aged = tr->lastSeen + kEvictAgeFrames < m_frame;
                const bool inFlight = std::binary_search(reading.begin(), reading.end(), tr);
                if (!aged || inFlight) {
                    ++stale;
                    ++i;
                    continue;
                }
                const std::shared_ptr<Tracked> keep = t.tracked[i];   // outlives Untrack
                Untrack(t, tr);   // the back moved into i: do not advance
                if (DropOne(keep)) {
                    compact = true;
                    ++droppedMapped;
                }
                ++dropped;
            }
            t.exWanted = wanted;
            t.exMapped = mapped;
            t.exDeficit = deficit;
            t.exUnreachable = unreachable;
            t.exStale = stale;
            t.exDropped += dropped;
            t.exDroppedMapped += droppedMapped;
            t.exRequeued += requeued;
            settleTurn.wanted += wanted;
            settleTurn.mapped += mapped;
            settleTurn.deficit += deficit;
            settleTurn.unreachable += unreachable;
            settleTurn.stale += stale;
            settleTurn.dropped += dropped;
            settleTurn.requeued += requeued;
        }
        if (compact) {
            std::erase_if(m_mapped, [](const std::shared_ptr<Tracked>& p) { return p->dropped; });
        }
        // A request the ring gate held this frame is a wanted tile one ring away: deficit,
        // unless its parent's load failed, in which case it is never admitted.
        settleTurn.deficit += ringHeldFrame - ringHeldDeadFrame;
        settleTurn.unreachable += ringHeldDeadFrame;
        settleTurn.pending = static_cast<uint32_t>(m_seen.size() + m_loading.size());
        settleTurn.reads = static_cast<uint32_t>(m_inFlightReads.size());
        settleTurn.retiring = static_cast<uint32_t>(m_retiring.size());
        settleTurn.exact = settleTurn.deficit == 0 && settleTurn.stale == 0 &&
                           settleTurn.pending == 0 && settleTurn.reads == 0 &&
                           settleTurn.retiring == 0;
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
        Log("[res-trace] f%u predicted stream so far: %llu Want calls, FNV-1a %016llx",
            m_frame, static_cast<unsigned long long>(predictedCalls),
            static_cast<unsigned long long>(predictedHash));
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
    ringHeldDeadFrame = 0;
    lap0 = Clock::now();   // the trace print above is not a phase

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
        lap(3);
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
        lap(2);
        // M9af: READINESS ORDERING WAS TRIED HERE AND MEASURED WORSE. See the note on
        // ResidencyManager::SetTileIndex -- preferring tiles already on the NVMe starves the
        // painting that puts them there, and this scene's cache is ~1% warm at the finest level.
        // The index stays (the DirectStorage read path needs it); the scheduling preference does
        // not, because the picture got worse and the picture decides.
        while (m_inFlight < static_cast<int>(kMaxLoadsInFlight) && !m_seen.empty()) {
            auto tile = m_seen.front();
            m_seen.pop_front();
            if (tile->dropped) continue;   // M9ba: identity moved before it loaded
            tile->state = TileState::Loading;
            m_loading.push_back(tile);
            m_loadQueue.push_back(tile);
            ++m_inFlight;
            m_cv.notify_one();
        }
        lap(3);
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
        lap(4);
        for (auto it = m_loading.begin();
             it != m_loading.end() && batch.size() < kMaxMapsPerFrame;) {
            auto& tile = *it;
            if (tile->dropped) {   // M9ba: landed for an identity that no longer exists
                it = m_loading.erase(it);
                continue;
            }
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
                const Tracked* parent = t.slot[StampIndex(
                    t, tile->req.face, tile->req.mip + 1, tile->req.x / 2, tile->req.y / 2)];
                parentOk = parent && parent->state == TileState::Mapped;
            }
            if (!parentOk) {
                ++it;   // waits for its ancestor; sort order makes this rare
                continue;
            }
            batch.push_back(tile);
            it = m_loading.erase(it);
        }
        lap(5);
    }
    if (!batch.empty()) MapAndFill(gpu, cl, batch);   // phases 6..8 bracket themselves
    // ---- THE BARRIER THE LANDED COPIES NEVER HAD. The landed loop above moves a tenant to
    // COPY_DEST and records the CopyTiles that swizzle its arrived tiles in; the transition back
    // to shader reads used to live at the tail of MapAndFill, which a turn with an EMPTY BATCH
    // never reaches. Such a turn drew the tenant in COPY_DEST with no barrier between the copies
    // and the draw -- and the residency byte, raised in the same turn, sent the sampler to the
    // tile while the copy engine was still writing it: whatever the pool slot held before, block
    // by block, until the copy caught up. That is priors 29/31's straight-edged dark-noise
    // quadrilaterals the size of one z17 detail tile.
    //
    // MEASURED by the previous commit's ledger, on the shipped code: 89 such turns over the
    // 14:00 flood rail and 68 over the 19:30Z one, every one of them reporting `barriered` = 0,
    // clustered exactly where step 28's notes recorded them (rec746, 755, 758, 761, 764, 767,
    // 770, 774, 776, 779, 791, 800). A fast streamer -- the horizon cull, a raised pool cap --
    // is what makes such turns common, which is why this is fixed BEFORE either of those.
    //
    // Every tenant a turn leaves in COPY_DEST goes back here, batch or no batch. Note this is
    // NOT the old sequence verbatim: MapAndFill's tail transitioned every tenant unconditionally,
    // including ones already in PIXEL|NON_PIXEL, which is a StateBefore == StateAfter barrier the
    // debug layer rejects. Only tenants that are actually in COPY_DEST are transitioned now, so
    // the command stream is shorter as well as correct.
    lap0 = Clock::now();   // the barriers stay in phase 6, where MapAndFill's tail had them
    for (auto& t : m_tenants) {
        if (t.state != D3D12_RESOURCE_STATE_COPY_DEST) continue;
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
        ++turn.barriered;
    }
    lap(6);
    // The landing ledger, this turn (Residency.h TurnLedger).
    if (turn.landedTiles && batch.empty()) {
        turn.landedOnly = 1;
        ++landedOnlyTurns;
    }
    turn.stageFree = static_cast<uint32_t>(m_stageFree.size());
    for (const auto& r : m_stageRetire) turn.stageRetiring += static_cast<uint32_t>(r.second.size());
    for (const auto& b : m_inFlightReads) turn.inFlightTiles += static_cast<uint32_t>(b.tiles.size());
    if (traceTurn) {
        Log("[res-turn] rec%u f%u | landed %u batches / %u tiles (lag %u..%u turns), %u retired, "
            "%u UNOWNED%s | batch %u: direct %u, ring %u, NO-BYTES %u, evicted %u | %u tenants "
            "barriered | landing: free %u, retiring %u, in flight %u tiles / %zu batches | pool "
            "%zu mapped, %zu free | queue: seen %zu, loading %zu, in flight %d",
            traceRecFrame, m_frame, turn.landedBatches, turn.landedTiles, turn.lagMin,
            turn.lagMax, turn.landedRetired, turn.landedUnowned,
            turn.landedOnly ? ", LANDED-ONLY turn" : "", turn.batch, turn.direct, turn.ring,
            turn.ringNoBytes, turn.evicted, turn.barriered, turn.stageFree, turn.stageRetiring,
            turn.inFlightTiles, m_inFlightReads.size(), m_mapped.size(), m_freePool.size(),
            m_seen.size(), m_loading.size(), m_inFlight.load());
    }
    lap0 = Clock::now();

    // ---- residency-map refresh (tiny R8 maps; only when dirty)
    //
    // M9bb: EACH TENANT STAGES IN ITS OWN REGION. The maps used to borrow "the tail of the
    // slab", computed per tenant from its own face count -- so every tenant's staging started
    // at the same (or an overlapping) offset, and when two tenants were dirty in one frame the
    // second memcpy overwrote the first's bytes before the recorded copies executed. Tenants
    // inherited each other's residency: the exposure page (7 slices, like the height page)
    // read the height tenant's map, believed mip 3 was resident under the helm, sampled its own
    // unmapped mip 3 and got zeros -- a flat sea with the node saying 0.85. Regions now sit
    // past the tile slab, one per tenant index, and can never touch a tile fill either.
    uint64_t mapStage = static_cast<uint64_t>(kMaxMapsPerFrame) * 65536;
    for (auto& t : m_tenants) {
        if (!t.resDirty) continue;
        t.resDirty = false;
        const uint32_t rdim = t.resMap.width;
        const uint32_t pitch = (rdim + 255u) & ~255u;
        const uint64_t bytes = static_cast<uint64_t>(pitch) * rdim * t.faces;
        GpuBuffer& ring = m_uploadRing[gpu.FrameIndex()];
        if (mapStage + bytes > static_cast<uint64_t>(kMaxMapsPerFrame) * 65536 + kMapStageBytes) {
            t.resDirty = true;   // no room this frame: keep it dirty, it goes next frame
            continue;
        }
        const uint64_t tail = mapStage;
        mapStage += bytes;
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
    lap(9);
    // Step 25: a map that found no staging room this turn goes next turn -- the frame just
    // recorded samples the old bytes, so the turn is not exact yet.
    if (settleExact && settleTurn.exact) {
        for (const auto& t : m_tenants) {
            if (t.resDirty) settleTurn.exact = false;
        }
    }

    // ---- stats
    char s[256];
    uint64_t bytes = 0;
    for (const auto& tr : m_mapped) {
        (void)tr;
        bytes += 65536;
    }
    snprintf(s, sizeof(s),
             "streams %zu res (%zu t, %.0f MB, %u fetches, %llu direct/%llu ring, landing: %llu "
             "landed-only turns, %llu no-bytes fills, %llu unowned claims%s)",
             m_tenants.size(), m_mapped.size(), bytes / 1048576.0, fetchesThisRun,
             static_cast<unsigned long long>(m_directTiles),
             static_cast<unsigned long long>(m_ringTiles),
             static_cast<unsigned long long>(landedOnlyTurns),
             static_cast<unsigned long long>(ringNoBytesTotal),
             static_cast<unsigned long long>(landedUnownedTotal),
             m_failedLoads ? " FAILED-TILES" : "");
    stats = s;
    for (const auto& f : m_fields) {
        char fs[96];
        snprintf(fs, sizeof(fs), "  %s %u t %.0f MB", f.name.c_str(), f.tiles(),
                 f.bytes() / 1048576.0);
        stats += fs;
    }
    turnMs = std::chrono::duration<double, std::milli>(Clock::now() - turn0).count();
}

void ResidencyManager::MapAndFill(Gpu& gpu, ID3D12GraphicsCommandList* cl,
                                  const std::vector<std::shared_ptr<Tracked>>& batch) {
    PixMarker(cl, "residency.mapAndFill");
    // Phase brackets (header, phaseMs): 6 = evict + map + barriers, 7 = the DirectStorage
    // per-tile OpenFile + Enqueue and the Submit, 8 = the ring memcpy + CopyTiles.
    using Clock = std::chrono::steady_clock;
    auto ph0 = Clock::now();
    auto phase = [&](int k) {
        const auto t = Clock::now();
        phaseMs[k] += std::chrono::duration<double, std::milli>(t - ph0).count();
        ph0 = t;
    };
    // Evictions to free pool slots when needed. Never evict: the coarsest mip, tiles seen
    // within the frame-overlap window (the GPU may still read them), or tiles with a mapped
    // child (the classic's pyramid invariant).
    auto childMapped = [&](const std::shared_ptr<Tracked>& tr) {
        if (tr->req.mip == 0) return false;
        for (uint32_t dy = 0; dy < 2; ++dy) {
            for (uint32_t dx = 0; dx < 2; ++dx) {
                const TileRequest c{tr->req.face, tr->req.mip - 1, tr->req.x * 2 + dx,
                                    tr->req.y * 2 + dy};
                const Tracked* child = Find(tr->tenant, c);   // bounds-checked: a child
                                                              // may lie past an odd grid
                if (child && child->state == TileState::Mapped) return true;
            }
        }
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

    // THE LANDING-SLOT DRAIN STAYS WHERE IT WAS, in the fill loop below. Step 28's patch hoisted
    // it here so the ledger could report a settled `stageFree`, arguing the slots come back in
    // the same turn either way. They do NOT, in one case: an empty `toFill` never reaches the
    // loop, so the old code skipped the drain that turn and the hoist returns those slots a turn
    // early. MEASURED (2026-09-06, five settled stills, --settle-exact, prev binary vs this one
    // in one session against a same-session A/A floor): with the hoist, key7km went max |d| 9
    // against an A/A floor of 5 -- three pixels, but outside the floor, and this commit is
    // supposed to be an instrument. The ledger does not need it: `stageFree` is read at the end
    // of the turn, after the fill loop has drained, so it reports the same number.
    turn.batch = static_cast<uint32_t>(batch.size());

    std::vector<std::shared_ptr<Tracked>> toFill;
    for (const auto& tile : batch) {
        Tenant& t = m_tenants[tile->tenant];
        // Step 25: during an exact hold the pool holds the WANT SET, whatever its size -- the
        // still is defined as every wanted tile resident, and a cap below the want set makes
        // the picture a race (see the ledger's pool line). Outside a hold the cap is the
        // shipped one, and this test is the shipped test.
        if (!settleExact && m_freePool.empty() &&
            m_heaps.size() * kPoolChunkTiles >= kPoolCapTiles) {
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
                Untrack(vt, victim.get());   // m_mapped's reference keeps it alive until
                m_mapped.erase(it);          // this erase, as the map's did
                freed = true;
                ++turn.evicted;
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
    phase(6);
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
                ++turn.direct;
                phase(7);
                continue;
            }
        }
        const D3D12_TILE_REGION_SIZE size{1, FALSE, 0, 0, 0};
        const size_t n = tile->data.size() < 65536 ? tile->data.size() : 65536;
        ++turn.ring;
        if (n < 65536) {
            // Step 28, counted: a ring fill of a tile that has no bytes (an archived tile
            // that found no landing slot, or whose OpenFile failed) copies the slab's stale
            // bytes into a mapped tile.
            ++turn.ringNoBytes;
            ++ringNoBytesTotal;
            if (traceTurn) {
                Log("[res-turn]   NO BYTES: %S f%u m%u (%u,%u) -> pool %u, %zu of 65536 bytes "
                    "(loc %d, landing slots free %zu)",
                    t.name.c_str(), tile->req.face, tile->req.mip, tile->req.x, tile->req.y,
                    tile->pool, n, int(tile->loc.Valid()), m_stageFree.size());
            }
        }
        memcpy(ring.cpu + off, tile->data.data(), n);
        cl->CopyTiles(t.res.Get(), &coord, &size, ring.res.Get(), off,
                      D3D12_TILE_COPY_FLAG_LINEAR_BUFFER_TO_SWIZZLED_TILED_RESOURCE);
        off += 65536;
        ++m_ringTiles;
        tile->data.clear();
        tile->data.shrink_to_fit();
        phase(8);
    }
    if (!direct.empty()) {
        InFlightRead f;
        f.fence = m_stream->Submit();
        f.frame = m_frame;
        f.tiles = std::move(direct);
        m_inFlightReads.push_back(std::move(f));
        phase(7);
    }
    // The tenants go back to shader reads in ProcessQueues, after this call returns: the landed
    // loop's copies need that barrier on the turns this call never runs at all.
}

void ResidencyManager::LogSettleExact(uint32_t heldFrames) const {
    // The per-tenant ledger of the last turn, and the mapped set itself as one number: FNV-1a
    // over the tracked tiles in state Mapped, in key order (face, mip, y, x), so the order the
    // landings happened in -- which two runs never share -- is not in the hash and the SET is.
    for (size_t k = 0; k < m_tenants.size(); ++k) {
        const Tenant& t = m_tenants[k];
        std::vector<Key> keys;
        keys.reserve(t.tracked.size());
        for (const auto& tr : t.tracked) {
            if (tr->state == TileState::Mapped) keys.push_back(MakeKey(static_cast<int>(k), tr->req));
        }
        std::sort(keys.begin(), keys.end());
        uint64_t h = 14695981039346656037ull;
        for (const Key key : keys) {
            for (int b = 0; b < 8; ++b) {
                h ^= (key >> (8 * b)) & 0xFFu;
                h *= 1099511628211ull;
            }
        }
        if (!t.exWanted && !t.exDropped && keys.empty()) continue;
        Log("[settle-exact]   %-44S wanted %u, mapped %u, deficit %u, unreachable %u, stale %u "
            "| dropped %u over the hold (%u were mapped) | mapped set %zu tiles, FNV-1a %016llx",
            t.name.c_str(), t.exWanted, t.exMapped, t.exDeficit, t.exUnreachable, t.exStale,
            t.exDropped, t.exDroppedMapped, keys.size(), static_cast<unsigned long long>(h));
        // The tiles that keep a hold from being exact, NAMED: the wanted-and-unmapped set by
        // state and mip, whether each still sits in a queue, and the first few by address.
        // (The gate's rule: a hold that never becomes exact names its tiles; it is the next
        // residency bug, not a number to soften.)
        if (t.exDeficit || t.exUnreachable) {
            uint32_t byState[5] = {}, byMip[64] = {}, queued = 0, dropped = 0;
            std::string named;
            uint32_t shown = 0;
            for (const auto& tr : t.tracked) {
                const size_t idx = StampIndex(t, tr->req.face, tr->req.mip, tr->req.x, tr->req.y);
                if ((t.stamp[idx] >> 1) != m_frame || tr->state == TileState::Mapped) continue;
                ++byState[static_cast<int>(tr->state)];
                ++byMip[tr->req.mip < 64 ? tr->req.mip : 63];
                if (tr->dropped) ++dropped;
                bool inQ = false;
                for (const auto& q : m_seen) inQ = inQ || q == tr;
                for (const auto& q : m_loading) inQ = inQ || q == tr;
                if (inQ) ++queued;
                if (shown < 6) {
                    char b[96];
                    snprintf(b, sizeof(b), " f%u m%u (%u,%u) state %d%s%s", tr->req.face,
                             tr->req.mip, tr->req.x, tr->req.y, int(tr->state),
                             tr->dropped ? " dropped" : "", inQ ? " queued" : " UNQUEUED");
                    named += b;
                    ++shown;
                }
            }
            std::string mips;
            for (uint32_t m = 0; m < 64; ++m) {
                if (!byMip[m]) continue;
                char b[24];
                snprintf(b, sizeof(b), " m%u:%u", m, byMip[m]);
                mips += b;
            }
            Log("[settle-exact]     unmapped wanted: seen %u, loading %u, loaded %u, failed %u; "
                "%u in a queue, %u dropped; by mip%s; e.g.%s",
                byState[0], byState[1], byState[2], byState[4], queued, dropped, mips.c_str(),
                named.c_str());
        }
    }
    // The pool against the shipped cap. Over the cap, the picture outside this hold is decided
    // by which tiles landed first (the header's note on settleExact).
    uint32_t requeued = 0;
    for (const auto& t : m_tenants) requeued += t.exRequeued;
    const size_t poolTiles = m_mapped.size();
    Log("[settle-exact] pool: %zu tiles mapped (%.0f MB) against the shipped cap of %u tiles "
        "(%.0f MB); %u cap casualties re-queued over the hold%s",
        poolTiles, poolTiles / 16.0, kPoolCapTiles, kPoolCapTiles / 16.0, requeued,
        poolTiles > kPoolCapTiles
            ? " -- THE WANT SET EXCEEDS THE SHIPPED CAP: outside this hold the resident set at "
              "this pose is a race for the last slots"
            : "");
    Log("[settle-exact] held %u frames: wanted %u, mapped %u, deficit %u, unreachable %u, stale "
        "%u, pending %u, in-flight reads %u, retiring %u -- %s",
        heldFrames, settleTurn.wanted, settleTurn.mapped, settleTurn.deficit,
        settleTurn.unreachable, settleTurn.stale, settleTurn.pending, settleTurn.reads,
        settleTurn.retiring, settleTurn.exact ? "EXACT" : "not exact");
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
