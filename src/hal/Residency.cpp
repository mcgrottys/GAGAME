#include "hal/Residency.h"
#include "core/ThreadManager.h"

#include <atomic>

#include "hal/PixEvents.h"
#include "hal/TileAtlas.h"   // Cl2ProductSignature -- the proven closure drives DeriveDemand

#include <algorithm>
#include <chrono>
#include <cmath>
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
    // M7w's worker count (2 starved every descent: 140 loads in 300 frames at the flood-rail
    // pose, and the patchwork was coarse fallback, not bad data) is now the Io lane's cap, set
    // by ThreadManager::Init from the same expression. No threads are created here.
    m_quit = false;
    // The predicted walk is a reader of its own: its statement stands for its lead (1a).
    if (m_predSid < 0) m_predSid = Sampler("predicted");
    Log("[residency] loads run on the pool's Io lane (%d at once)", Threads().IoCap());
}

void ResidencyManager::Shutdown() {
    // The jobs capture `this`. Refuse new work, then WAIT for the ones already running to
    // leave -- the pool outlives this object and would otherwise still be inside it. (The old
    // pool joined its own threads here, which had the same effect for free.)
    m_quit = true;
    std::unique_lock<std::mutex> lk(m_mx);
    m_drainCv.wait(lk, [this] { return m_inFlight.load() == 0; });
}

void ResidencyManager::RunLoad(const std::shared_ptr<Tracked>& job) {
    {
        // Shutdown ran between this job being queued and it being picked up.
        if (m_quit.load()) {
            std::lock_guard<std::mutex> lk(m_mx);
            --m_inFlight;
            m_drainCv.notify_all();
            return;
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
        const bool canStream = m_stream && m_stream->Available() && !job->refresh;
        const auto t0 = std::chrono::steady_clock::now();
        g_tileIncomplete = false;   // step 5 E: a tree says here if its answer is not whole
        g_tileMagnified = false;    // PHASE A4: ... or if it is the level above, magnified
        const bool ok =
            m_tenants[job->tenant].provider(job->req, data, canStream ? &loc : nullptr);
        const bool whole = !g_tileIncomplete;
        const bool magnified = g_tileMagnified;
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
            if (job->refresh && (!ok || !whole || magnified)) {
                // THE VERSION LAW: a replacement that is not whole, magnified or failed is a refusal
                // of this attempt -- the held tile keeps its bytes; no mark on the address, no retry
                // until the tree changes for it again.
                job->state = TileState::Failed;
                job->incomplete = ok;
            } else if (ok && magnified) {
                // PHASE A4: nothing to map; the parent answers. Never retried (it is arithmetic).
                job->state = TileState::Failed;
                job->magnified = true;
                ++m_failEvents;
                m_failedKeys.push_back({job->tenant, job->req});
            } else if (ok && whole) {
                job->loc = loc;
                job->data = std::move(data);
                job->state = TileState::Loaded;
            } else if (ok) {
                // Step 5 E (finding 83): a tile not whole is not delivered: it is
                // unreachable for the run, as a tile that failed four tries is, and said.
                job->state = TileState::Failed;
                job->incomplete = true;   // the version law: retried when the tree next changes
                ++m_failedLoads;
                ++m_failEvents;
                ++incompleteTotal;
                m_failedKeys.push_back({job->tenant, job->req});
                if (incompleteTotal <= 12) {
                    Log("[residency] order: %S f%u m%u (%u,%u) answered without one of its sources "
                        "(not whole): not delivered; refused this attempt, asked again when the tree "
                        "next changes for it; its children wait until then (finding 83, the version law)",
                        m_tenants[job->tenant].name.c_str(), job->req.face, job->req.mip,
                        job->req.x, job->req.y);
                }
            } else if (++job->retries < 4) {
                job->state = TileState::Seen;   // requeued by ProcessQueues from m_loading
            } else {
                job->state = TileState::Failed;
                ++m_failedLoads;
                ++m_failEvents;   // step 5: an event for the order (under m_mx, as it reads it)
                m_failedKeys.push_back({job->tenant, job->req});
            }
            --m_inFlight;
        }
        m_drainCv.notify_all();
    }
}

int ResidencyManager::AddTextureInternal(Gpu& gpu, const wchar_t* name, uint32_t faceDim,
                                         DXGI_FORMAT fmt, TileProviderFn provider,
                                         uint32_t faces, std::vector<uint8_t> sliceTop) {
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
    // F1: each slice's coarsest held mip; a window's is its floor, the rest the array's.
    t.sliceTop.assign(faces, static_cast<uint8_t>(mips - 1));
    for (uint32_t f = 0; f < faces && f < sliceTop.size(); ++f) {
        t.sliceTop[f] = static_cast<uint8_t>((std::min)(uint32_t(sliceTop[f]), mips - 1));
    }

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
        t.want.assign(acc, uint16_t(0));   // M13: which samplers asked, this frame
        t.slot.assign(acc, nullptr);   // step 4: the Tracked at the same index
        Log("[residency] %S: stamp array %u tiles (%.2f MB) + sampler mask (%.2f MB) + slot "
            "array (%.2f MB) -- Want()'s hot question, WHO asked it, and the tile itself, all "
            "out of the map",
            name, acc, acc * 4.0 / 1048576.0, acc * 2.0 / 1048576.0,
            acc * double(sizeof(Tracked*)) / 1048576.0);
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
    // Step 5, law 9: the map is born saying nothing (255) and says what the boot
    // holds once it holds it (BirthMap).
    for (uint32_t f = 0; f < faces; ++f) {
        t.resCpu[f].assign(rdim * rdim, uint8_t(255));
    }
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
            // F1: a slice whose chain ends below the array's coarsest (a window) is born saying
            // nothing and fills by its wants once its box is placed: its floor is the rank above's
            // ground, not the planet's, and nothing of it stands before the eye does.
            if (TopOf(tn, f) != coarsest) continue;
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
                tr->landed = true;   // EndUpload waited: the bytes are on the GPU
                Track(tn, tr);
                m_mapped.push_back(tr);
            }
        }
        BirthMap(gpu, id);
        // Retry-wants AFTER registration, so a mixed face cannot double-map its healthy
        // tiles (Want on a Mapped tracked entry only bumps lastSeen).
        for (const TileRequest& r : failedBoot) {
            const auto& ti = tn.tilings[r.face * mips + coarsest];
            const float uc = (r.x + 0.5f) / (std::max)(1u, static_cast<uint32_t>(ti.WidthInTiles));
            const float vc = (r.y + 0.5f) / (std::max)(1u, static_cast<uint32_t>(ti.HeightInTiles));
            // M13: the manager's own retry at boot -- a reader like any other, named so the
            // ledger never charges a view for a tile the boot asked for.
            Want(Sampler("boot"), id, r.face, coarsest, uc, vc, uc, vc);
        }
    }
    return id;
}

void ResidencyManager::Want(int sampler, int tenant, uint32_t face, uint32_t mip, float u0,
                            float v0, float u1, float v1, bool predicted, float nearM,
                            float focusU, float focusV) {
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
    // A slice the tenant does not have is no want (Mars's height cube has six faces and no windows:
    // a reader that names a window slice of it asks for nothing). Said once a tenant.
    if (face >= t.faces) {
        if (!t.saidNoSlice) {
            t.saidNoSlice = true;
            Log("[residency] %S: slice %u wanted by %s, the tenant has %u -- no want (said once)", t.name.c_str(),
                face, SamplerName(sampler), t.faces);
        }
        return;
    }
    if (mip >= t.mips) mip = t.mips - 1;
    const uint32_t topF = TopOf(t, face);
    if (mip > topF) return;   // F1: above a window's floor nothing is held, so nothing is wanted

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
    // M13: WHO is asking. The counts belong to a frame, so they reset when the stamp frame
    // moves -- no ordering assumption about when main reads them against the turn.
    // Step 5 D: the predicted walk speaks as its own reader (decision 1).
    const int sid = (predicted && m_predSid >= 0)                        ? m_predSid
                    : (sampler >= 0 && sampler < kMaxSamplers) ? sampler : 0;
    const uint16_t bit = static_cast<uint16_t>(1u << sid);
    if (m_markFrame == kMarkFrame) Mark(("want: " + std::string(SamplerName(sid))).c_str());
    {   // when each reader last spoke, and a standing reader
        m_sampLast[sid] = stampFrame;
        if (sid != m_predSid) m_lastSpoke = (std::max)(m_lastSpoke, stampFrame);
        if ((m_pinMask >> sid) & 1u) m_pinSpoke = (std::max)(m_pinSpoke, stampFrame);
    }
    if (m_sampFrame != stampFrame) {
        m_sampFrame = stampFrame;
        for (int i = 0; i < kMaxSamplers; ++i) m_sampTiles[i] = m_sampAlone[i] = 0;
    }
    // Fresh means this frame AND not a real touch arriving on a speculative tile -- that
    // transition must still run, or a tile the prefetch asked for and the view then confirmed
    // stays flagged speculative and is evicted on the wrong budget -- AND already carrying
    // THIS sampler's bit: another sampler's touch leaves the tile stamped but unrecorded for
    // this one, and a want nobody recorded cannot be charged against a reserve.
    const auto fresh = [&](uint32_t s, uint16_t w) {
        return (s >> 1) == stampFrame && !(!predicted && (s & 1u)) && (w & bit) != 0;
    };
    // Mark a tile for this sampler. The mask is only meaningful while the stamp is this
    // frame's, so a stale stamp starts the mask over rather than adding to last frame's.
    const auto mark = [&](uint32_t& st, uint16_t& w, uint32_t newStamp, uint32_t mm, uint32_t xx,
                          uint32_t yy, size_t idx, Tracked* trk) {
        const bool sameFrame = (st >> 1) == stampFrame;
        const uint16_t was = sameFrame ? w : uint16_t(0);
        if (!(was & bit)) {
            // Step 5: the reader's statement and the tile's record (OrderNote).
            OrderNote(t, trk, tenant, sid, stampFrame, face, mm, xx, yy, idx, nearM, focusU, focusV);
            ++m_sampTiles[sid];
            if (was == 0) {
                ++m_sampAlone[sid];
            } else if (was && (was & (was - 1)) == 0) {
                // the tile had exactly one other owner, which is no longer alone on it
                for (int i = 0; i < kMaxSamplers; ++i) {
                    if (was == uint16_t(1u << i)) {
                        if (m_sampAlone[i]) --m_sampAlone[i];
                        break;
                    }
                }
            }
        }
        w = static_cast<uint16_t>(was | bit);
        st = newStamp;
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
    for (uint32_t m = mip; m <= topF; ++m) {
        const uint32_t plane = face * t.mips + m;
        const Rect r = rectAt(plane);
        bool allFresh = true;
        for (uint32_t y = r.y0; y <= r.y1 && allFresh; ++y) {
            const size_t rowBase = size_t(t.stampBase[plane]) + size_t(y) * t.stampW[plane];
            const uint32_t* row = t.stamp.data() + rowBase;
            const uint16_t* wrow = t.want.data() + rowBase;
            for (uint32_t x = r.x0; x <= r.x1; ++x) {
                ++wantTouches;
                if (!fresh(row[x], wrow[x])) {
                    allFresh = false;
                    break;
                }
                ++wantHits;
            }
        }
        if (allFresh) break;
        top = static_cast<int>(m);
    }
    // Step 5 E (decision 2): a walk's want gives every tile of its column its leaf's distance, and
    // an ancestor's weight is the least of its leaves' -- including the ancestors the column scan
    // skips because this reader already stamped them this frame.
    if (focusU < 0.0f) {
        uint32_t wb = 0;
        if (nearM > 0.0f) memcpy(&wb, &nearM, sizeof(wb));
        for (uint32_t mm = mip; mm <= topF; ++mm) {
            const uint32_t plane = face * t.mips + mm;
            const Rect r = rectAt(plane);
            for (uint32_t y = r.y0; y <= r.y1; ++y) {
                const size_t rowBase = size_t(t.stampBase[plane]) + size_t(y) * t.stampW[plane];
                for (uint32_t x = r.x0; x <= r.x1; ++x) {
                    const Tracked* tr = t.slot[rowBase + x];
                    if (!tr || tr->rec == UINT32_MAX) continue;
                    OrdRec& o = m_rec[tr->rec];   // H5: this reader's own slot, this frame
                    for (int k = 0; k < OrdRec::kSlots; ++k) {
                        if (o.ssid[k] == sid && o.sstamp[k] == stampFrame && wb < o.sweight[k]) o.sweight[k] = wb;
                    }
                }
            }
        }
    }
    if (top < 0) return;

    for (int m = top; m >= static_cast<int>(mip); --m) {
        const uint32_t plane = face * t.mips + static_cast<uint32_t>(m);
        const Rect r = rectAt(plane);
        for (uint32_t y = r.y0; y <= r.y1; ++y) {
            const size_t rowBase = size_t(t.stampBase[plane]) + size_t(y) * t.stampW[plane];
            uint32_t* stampRow = t.stamp.data() + rowBase;
            uint16_t* wantRow = t.want.data() + rowBase;
            Tracked** slotRow = t.slot.data() + rowBase;
            for (uint32_t x = r.x0; x <= r.x1; ++x) {
                ++wantTouches;
                // The stamp answers "already handled this frame" without touching the tile at
                // all. It is written on every path below, so it stays exactly as true as
                // lastSeen/predicted are -- they remain the authority for eviction; this is a
                // cache of the one question the walk asks.
                uint32_t& st = stampRow[x];
                uint16_t& wt = wantRow[x];
                if (fresh(st, wt)) {
                    ++wantHits;
                    continue;
                }
                // Step 4: the tile itself sits at the same index. No key, no tree.
                if (Tracked* tr = slotRow[x]) {
                    tr->lastSeen = m_frame;
                    if (!predicted) tr->predicted = false;
                    mark(st, wt, (stampFrame << 1) | (tr->predicted ? 1u : 0u),
                         static_cast<uint32_t>(m), x, y, rowBase + x, tr);
                    continue;
                }
                // M9al: THE RING GATE. A new request below the coarsest level is admitted only
                // if its parent is already MAPPED. The tile stays unstamped, so next frame's
                // walk asks again -- by which time the parent has landed, or has not, and the
                // answer is the same question one ring later. Nothing is lost, and no load slot
                // is ever spent on a tile more than one level from being displayable.
                mark(st, wt, myStamp, static_cast<uint32_t>(m), x, y, rowBase + x, nullptr);
                auto tr = std::make_shared<Tracked>();
                tr->tenant = tenant;
                tr->req = TileRequest{face, static_cast<uint32_t>(m), x, y};
                tr->lastSeen = m_frame;
                tr->predicted = predicted;
                Track(t, tr);
            }
        }
    }
}

// M13: the sampler registry. Ids are handed out in registration order and never move, so a
// mask bit means the same reader for the life of the run; a name asked for twice is one
// sampler (the view's layer and the loop's own wants are the same reader).
int ResidencyManager::Sampler(const char* name, bool pin) {
    const std::string n = name ? name : "?";
    int id = -1;
    for (size_t i = 0; i < m_samplers.size() && id < 0; ++i) {
        if (m_samplers[i] == n) id = static_cast<int>(i);
    }
    if (id < 0 && static_cast<int>(m_samplers.size()) >= kMaxSamplers) {
        if (!m_sampOverflowed) {
            m_sampOverflowed = true;
            Log("[residency] more than %d samplers: '%s' and any after it share the last id. "
                "The accounting merges; no tile is lost.", kMaxSamplers, n.c_str());
        }
        id = kMaxSamplers - 1;
    }
    if (id < 0) {
        m_samplers.push_back(n);
        id = static_cast<int>(m_samplers.size()) - 1;
    }
    if (pin) m_pinMask = static_cast<uint16_t>(m_pinMask | (1u << id));   // step 5: a standing reader
    return id;
}

void ResidencyManager::LogSamplers() const {
    if (Samplers() <= 1) return;   // one reader: the ledger is the totals already printed
    uint32_t sum = 0;
    for (int i = 0; i < Samplers(); ++i) sum += m_sampAlone[i];
    Log("[samplers] frame %u: %d readers on one cache", m_frame, Samplers());
    for (int i = 0; i < Samplers(); ++i) {
        Log("[samplers]   %-16s wanted %6u tiles, %6u of them alone (%5.1f %% shared)",
            m_samplers[i].c_str(), m_sampTiles[i], m_sampAlone[i],
            m_sampTiles[i] ? 100.0 * double(m_sampTiles[i] - m_sampAlone[i]) / double(m_sampTiles[i])
                           : 0.0);
    }
    Log("[samplers]   %-16s %6u tiles wanted by exactly one reader", "(alone in all)", sum);
}

void ResidencyManager::Track(Tenant& t, const std::shared_ptr<Tracked>& tr) {
    Tracked*& s = t.slot[StampIndex(t, tr->req.face, tr->req.mip, tr->req.x, tr->req.y)];
    if (s) throw std::runtime_error("residency: Track() over a live slot");
    s = tr.get();
    ++m_trackEpoch;   // step 5: an event for the order
    RecAdd(t, tr->tenant, tr.get());
    tr->pos = static_cast<uint32_t>(t.tracked.size());
    t.tracked.push_back(tr);
}

void ResidencyManager::Untrack(Tenant& t, Tracked* tr) {
    Tracked*& s = t.slot[StampIndex(t, tr->req.face, tr->req.mip, tr->req.x, tr->req.y)];
    if (s != tr) throw std::runtime_error("residency: Untrack() of a tile its slot does not hold");
    s = nullptr;
    ++m_trackEpoch;
    RecDrop(tr);
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
    // Step 5 E (1a): the prediction speaks for the eye `aheadFrames` ahead; its wants stand that long.
    m_predLead = aheadFrames > 0.0 ? static_cast<uint32_t>(std::ceil(aheadFrames)) : 0u;
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

// ---- THE FLOOR LAW (Residency.h): the construction, gated by [restest], not staged ---------

std::vector<uint32_t> ResidencyManager::CubeFloorRing(uint32_t rdim) {
    std::vector<uint32_t> ring;
    ring.reserve(size_t(6) * (4 * rdim + 4));
    const int n = static_cast<int>(rdim);
    const auto index = [n](double c) {
        return static_cast<uint32_t>(std::clamp(static_cast<int>(std::floor(c * n)), 0, n - 1));
    };
    for (uint32_t f = 0; f < 6; ++f) {
        // The cell past the square at (x, y), one of them -1 or rdim: its centre's direction in
        // this face's plane, and the true cell of the face that direction falls in.
        const auto cell = [&](int x, int y) {
            double d[3], uv[2];
            ComposeCubeDir(f, (x + 0.5) / n, (y + 0.5) / n, d);
            const uint32_t g = CubeFaceOfDir(d, uv);
            return (g * rdim + index(uv[1])) * rdim + index(uv[0]);
        };
        for (int x = -1; x <= n; ++x) ring.push_back(cell(x, -1));
        for (int y = 0; y < n; ++y) {
            ring.push_back(cell(-1, y));
            ring.push_back(cell(n, y));
        }
        for (int x = -1; x <= n; ++x) ring.push_back(cell(x, n));
    }
    return ring;
}

void ResidencyManager::FloorPad(const std::vector<std::vector<uint8_t>>& in, uint32_t rdim,
                                uint32_t s, const std::vector<uint32_t>* cubeRing,
                                std::vector<uint8_t>& padded) {
    const uint32_t p = rdim + 2;
    padded.resize(size_t(p) * p);
    for (uint32_t y = 0; y < rdim; ++y) {
        memcpy(&padded[size_t(y + 1) * p + 1], &in[s][size_t(y) * rdim], rdim);
    }
    if (cubeRing) {
        const uint32_t* r = cubeRing->data() + size_t(s) * (4 * rdim + 4);
        const uint32_t n2 = rdim * rdim;
        const auto at = [&](uint32_t i) { return in[i / n2][i % n2]; };
        for (uint32_t x = 0; x < p; ++x) padded[x] = at(*r++);
        for (uint32_t y = 1; y <= rdim; ++y) {
            padded[size_t(y) * p] = at(*r++);
            padded[size_t(y) * p + p - 1] = at(*r++);
        }
        for (uint32_t x = 0; x < p; ++x) padded[size_t(p - 1) * p + x] = at(*r++);
        return;
    }
    for (uint32_t y = 1; y <= rdim; ++y) {
        padded[size_t(y) * p] = padded[size_t(y) * p + 1];
        padded[size_t(y) * p + p - 1] = padded[size_t(y) * p + p - 2];
    }
    memcpy(&padded[0], &padded[p], p);
    memcpy(&padded[size_t(p - 1) * p], &padded[size_t(p - 2) * p], p);
}

void ResidencyManager::FloorMap(const std::vector<std::vector<uint8_t>>& in, uint32_t rdim,
                                const std::vector<uint32_t>* cubeRing,
                                std::vector<std::vector<uint8_t>>& out) {
    // `out` is written while `in`'s neighbours are still being read: never the same maps.
    const uint32_t p = rdim + 2;
    std::vector<uint8_t> pad, rows(size_t(p) * rdim);
    out.resize(in.size());
    for (uint32_t s = 0; s < in.size(); ++s) {
        FloorPad(in, rdim, s, (s < 6 && in.size() >= 6) ? cubeRing : nullptr, pad);
        // The 3 x 3 as two passes of three: the largest across each padded row, then the
        // largest of three of those down.
        for (uint32_t y = 0; y < p; ++y) {
            const uint8_t* a = &pad[size_t(y) * p];
            uint8_t* r = &rows[size_t(y) * rdim];
            for (uint32_t x = 0; x < rdim; ++x) {
                r[x] = (std::max)((std::max)(a[x], a[x + 1]), a[x + 2]);
            }
        }
        out[s].resize(size_t(rdim) * rdim);
        for (uint32_t y = 0; y < rdim; ++y) {
            const uint8_t* r0 = &rows[size_t(y) * rdim];
            const uint8_t* r1 = r0 + rdim;
            const uint8_t* r2 = r1 + rdim;
            uint8_t* o = &out[s][size_t(y) * rdim];
            for (uint32_t x = 0; x < rdim; ++x) {
                o[x] = (std::max)((std::max)(r0[x], r1[x]), r2[x]);
            }
        }
    }
}

// 4.7: a tile's bytes landed: it is held, and so is every other slot mapped to its address.
void ResidencyManager::Landed(Tracked* tile) {
    tile->landed = true;
    if (tile->rec != UINT32_MAX) m_rec[tile->rec].flags |= kHeldBit;
    NoteChanged(tile->tenant, tile->req);
    if (!tile->gkey) return;
    Tenant& t = m_tenants[tile->tenant];
    const auto it = t.held.find(tile->gkey);
    if (it == t.held.end()) return;
    it->second.landed = true;
    for (Tracked* s : it->second.slots) {
        if (s == tile || s->landed || s->state != TileState::Mapped) continue;
        s->landed = true;
        if (s->rec != UINT32_MAX) m_rec[s->rec].flags |= kHeldBit;
        NoteChanged(s->tenant, s->req);
    }
}

// 4.7: a retiring slot lets go of its address; the pool slot is freed with the last of them.
void ResidencyManager::ReleaseSlot(Tenant& t, Tracked& tile) {
    if (tile.pool == UINT32_MAX) return;
    if (tile.gkey) {
        const auto it = t.held.find(tile.gkey);
        if (it != t.held.end() && it->second.pool == tile.pool) {
            auto& v = it->second.slots;
            v.erase(std::remove(v.begin(), v.end(), &tile), v.end());
            if (!v.empty()) return;   // the address stays held by another slot
            t.held.erase(it);
        }
    }
    m_freePool.push_back(tile.pool);
}

bool ResidencyManager::DropOne(const std::shared_ptr<Tracked>& tr) {
    tr->dropped = true;
    if (tr->state != TileState::Mapped) return false;
    NoteChanged(tr->tenant, tr->req);   // untracked by now: no longer held (law 1)
    m_retiring.push_back({tr, m_frame});
    return true;
}

void ResidencyManager::Invalidate(int tenant, const TileRequest& r, bool moved) {
    std::lock_guard<std::mutex> lk(m_invMx);
    m_invQ.push_back({tenant, r, moved});
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
        {   // its record leaves with it
            RecDrop(tr.get());
            ++m_trackEpoch;
        }
        if (DropOne(tr)) ++mapped; else ++inflight;
    }
    if (mapped) {
        std::erase_if(m_mapped, [](const std::shared_ptr<Tracked>& p) { return p->dropped; });
    }
    Log("[residency] %S: dropped %u mapped tiles (retire after %u frames) and %u in flight",
        t.name.c_str(), mapped, kEvictAgeFrames, inflight);
}

void ResidencyManager::ProcessQueues(Gpu& gpu, ID3D12GraphicsCommandList* cl) {
    PixScope scope(cl, "residency (the order's turn: release, load, map, fill)");
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
    releaseLedger = ReleaseLedger{};   // PHASE B2w
    Mark("turn: the residency manager's (at the head of the frame's command list)");
    // Step 4: the slot array's bring-up gate rides the trace flag for the first thousand
    // frames (a full scan of every slot array; not a cost the bench ever pays).
    if (traceRes && m_frame <= kSlotAuditFrames) AuditSlots();
    // M9bb: apply the invalidations the painting threads queued (one tile each).
    {
        std::vector<Inv> q;
        {
            std::lock_guard<std::mutex> lk(m_invMx);
            q.swap(m_invQ);
        }
        bool compact = false;
        for (const Inv& inv : q) {
            const int tenant = inv.tenant;
            const TileRequest& r = inv.req;
            Tracked* tr = Find(tenant, r);
            if (auditEvery) AuditInvalidation(tenant, r, tr);   // counted before the drop
            if (!tr) continue;
            Tenant& t = m_tenants[tenant];
            // THE VERSION LAW: a HELD tile is not released by a change -- its version is. Its bytes
            // stay mapped and held (stale in the ledger) until its replacement lands whole, then
            // swap in place, in one turn (MapAndFill's refill); its children keep their parent.
            // B11: NOT when the slot's ground moved (a window's step): the held bytes are another
            // place's, and a reader addressing the slot would draw them there. Let go now.
            if (!inv.moved && tr->state == TileState::Mapped && tr->landed && !tr->dropped) {
                tr->stale = true;
                bool asked = false;
                for (Refresh& f : m_refresh) {
                    if (f.held.get() == tr) {
                        f.again = true;
                        asked = true;
                        break;
                    }
                }
                if (!asked) {
                    auto job = std::make_shared<Tracked>();
                    job->tenant = tr->tenant;
                    job->req = tr->req;
                    job->refresh = true;
                    m_refresh.push_back({t.tracked[tr->pos], job, false});
                }
                continue;
            }
            if (tr->state == TileState::Mapped) {   // PHASE B2w: a held tile let go by a change
                ++releaseLedger.invalidated;
                if (tr->lastSeen + 1u >= m_frame) ++releaseLedger.invalidatedNamed;
            }
            const std::shared_ptr<Tracked> keep = t.tracked[tr->pos];   // outlives Untrack
            Untrack(t, tr);
            if (DropOne(keep)) compact = true;
        }
        if (compact) {
            std::erase_if(m_mapped, [](const std::shared_ptr<Tracked>& p) { return p->dropped; });
        }
    }
    // M9ba: NULL-map the dropped tiles whose overlap window has passed; free their slots.
    // (Step 5: the NULL maps go in one call a tenant, FlushNullMaps.)
    std::vector<std::pair<int, D3D12_TILED_RESOURCE_COORDINATE>> nulls;
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
            nulls.push_back({it->tile->tenant, D3D12_TILED_RESOURCE_COORDINATE{
                                                   it->tile->req.x, it->tile->req.y, 0,
                                                   it->tile->req.face * t.mips + it->tile->req.mip}});
        }
        ReleaseSlot(t, *it->tile);   // 4.7: the pool slot goes with the last slot of its address
        it->tile->state = TileState::Failed;
        it = m_retiring.erase(it);
    }
    if (!nulls.empty()) FlushNullMaps(gpu, nulls);
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
                Landed(tile.get());   // held: the map names it from this turn (law 1); 4.7: its address's slots too
                ++m_claimEvents;
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

    // ---- THE PAGES LEDGER (Residency.h pagesEvery). HERE, at the settle's own point: after the
    // landed reads claimed their tiles, before the batch is gathered, so its "wanted this frame"
    // is the settle's to the tile. Not a phase: the lap clock restarts after the print.
    if (pagesEvery && (m_frame % pagesEvery) == 0u) {
        std::lock_guard<std::mutex> lk(m_mx);
        LogPages();
        lap0 = Clock::now();
    }

    lap0 = Clock::now();

    // ---- THE ORDER (ResidencyOrder.cpp): one comparison decides the loads, the releases, the
    // let-gos and the batch to map. The loads are submitted after m_mx is released: under
    // --jobs-inline a Submit runs the job on this thread, and RunLoad takes m_mx.
    std::vector<std::shared_ptr<Tracked>> toLoad, batch, refills;
    OrderTurn(toLoad, batch, refills);
    for (const auto& tile : toLoad) {
        Threads().Submit(Lane::Io, "residency.load", [this, tile] { RunLoad(tile); });
    }

    if (!batch.empty() || !refills.empty()) MapAndFill(gpu, cl, batch, refills);   // phases 6..8
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
    mappedTotal += turn.direct + turn.ring;   // an untraced rail's maps, either manager
    turn.stageFree = static_cast<uint32_t>(m_stageFree.size());
    for (const auto& r : m_stageRetire) turn.stageRetiring += static_cast<uint32_t>(r.second.size());
    for (const auto& b : m_inFlightReads) turn.inFlightTiles += static_cast<uint32_t>(b.tiles.size());
    if (traceTurn) {
        LogSamplers();   // M13: the readers' shares of this turn, beside the turn itself
        Log("[res-turn] rec%u f%u | landed %u batches / %u tiles (lag %u..%u turns), %u retired, "
            "%u UNOWNED%s | batch %u: direct %u, ring %u, NO-BYTES %u, evicted %u (%u "
            "reclaimed) | %u tenants "
            "barriered | landing: free %u, retiring %u, in flight %u tiles / %zu batches | pool "
            "%zu mapped, %zu free | queue: pending %u, loading %zu, in flight %d",
            traceRecFrame, m_frame, turn.landedBatches, turn.landedTiles, turn.lagMin,
            turn.lagMax, turn.landedRetired, turn.landedUnowned,
            turn.landedOnly ? ", LANDED-ONLY turn" : "", turn.batch, turn.direct, turn.ring,
            turn.ringNoBytes, turn.evicted, turn.reclaimed, turn.barriered, turn.stageFree,
            turn.stageRetiring,
            turn.inFlightTiles, m_inFlightReads.size(), m_mapped.size(), m_freePool.size(),
            m_orderPending, m_loading.size(), m_inFlight.load());
        {
            const OrderTurnLedger& o = orderTurn;
            Log("[res-turn]   order: %u candidates, cut %u, first P held %u, pending %u (%u loaded "
                "waiting for a slot) | released %u (%u nobody wanted), let go %u (%u with bytes "
                "read), forgotten %u | gathered and not filled %u | reloaded within the glance %u, "
                "wanted back while retiring %u (rescued %u) | not whole %u | pass %u | crossed the "
                "cut: in %u, out %u | spoke 0x%x",
                o.candidates, o.cut, o.firstPHeld, o.pending, o.waiting, turn.evicted,
                turn.reclaimed, o.letGo, o.letGoRead, o.forgotten,
                turn.batch - turn.direct - turn.ring, o.reloaded, o.rewanted, o.rescued,
                o.incomplete, o.pass, o.crossIn, o.crossOut, o.spoke);
        }
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
    //
    // WHAT GOES UP IS THE TRUE MAP. The floor law (Residency.h) would stage FloorMap(resCpu)
    // here instead, and was measured unsound under the colour's anisotropic sampler; it is
    // not staged.
    ApplyChanged();   // law 1: every footprint that changed, from the held set
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
    // The residency audit (ResidencyAudit.cpp): the turn's bytes against its tiles, outside turnMs.
    if (auditEvery && (m_frame % auditEvery) == 0u) LogAudit();
}

void ResidencyManager::MapAndFill(Gpu& gpu, ID3D12GraphicsCommandList* cl,
                                  const std::vector<std::shared_ptr<Tracked>>& batch,
                                  const std::vector<std::shared_ptr<Tracked>>& refills) {
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
    struct PerHeap {
        std::vector<D3D12_TILED_RESOURCE_COORDINATE> coords;
        std::vector<D3D12_TILE_REGION_SIZE> sizes;
        std::vector<D3D12_TILE_RANGE_FLAGS> flags;
        std::vector<UINT> offsets;
        std::vector<UINT> counts;
    };
    // Keyed by (tenant, heapChunk): the order's releases NULL-map in the retire loop, batched.
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

    std::vector<std::shared_ptr<Tracked>> toFill, aliases;
    for (const auto& tile : batch) {
        Tenant& t = m_tenants[tile->tenant];
        // 4.7: a mapping of bytes another slot read -- the address's pool slot, no read, no fill.
        if (tile->alias) {
            const auto it = t.held.find(tile->gkey);
            if (it == t.held.end() || it->second.pool == UINT32_MAX) {
                tile->alias = false;   // the address was let go meanwhile: ask again next turn
                tile->state = TileState::Seen;
                continue;
            }
            Tenant::Held& h = it->second;
            tile->pool = h.pool;
            h.slots.push_back(tile.get());
            auto& mc = calls[{tile->tenant, h.pool >> 16}];
            mc.coords.push_back({tile->req.x, tile->req.y, 0, tile->req.face * t.mips + tile->req.mip});
            mc.sizes.push_back({1, FALSE, 0, 0, 0});
            mc.flags.push_back(D3D12_TILE_RANGE_FLAG_NONE);
            mc.offsets.push_back(h.pool & 0xFFFF);
            mc.counts.push_back(1);
            tile->state = TileState::Mapped;
            m_mapped.push_back(tile);
            aliases.push_back(tile);
            ++sharedMapsTotal;
            continue;
        }
        const uint32_t slot = AcquirePoolTile(gpu);
        tile->pool = slot;
        if (tile->gkey) {   // the address is held by this slot's read
            Tenant::Held& h = t.held[tile->gkey];
            h.pool = slot;
            h.landed = false;
            h.reader = tile->req;
            h.slots.push_back(tile.get());
        }
        auto& mc = calls[{tile->tenant, slot >> 16}];
        mc.coords.push_back(
            {tile->req.x, tile->req.y, 0, tile->req.face * t.mips + tile->req.mip});
        mc.sizes.push_back({1, FALSE, 0, 0, 0});
        mc.flags.push_back(D3D12_TILE_RANGE_FLAG_NONE);
        mc.offsets.push_back(slot & 0xFFFF);
        mc.counts.push_back(1);
        tile->state = TileState::Mapped;
        m_mapped.push_back(tile);
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
    // THE VERSION LAW'S SWAP: a refill writes the replacement's whole bytes into the held tile's own
    // pool slot through the ring, recorded at the head of this frame's list like every fill: the
    // frame that reads it reads the new version; no frame reads neither.
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
        // Step 5: a ring fill is readable when this frame's list executes -- held, if it carried
        // a whole tile. A fill without bytes is garbage and the map never names it.
        if (n == 65536) Landed(tile.get());
        off += 65536;
        ++m_ringTiles;
        tile->data.clear();
        tile->data.shrink_to_fit();
        phase(8);
    }
    // 4.7: an alias of an address whose bytes have landed is held from this turn (the mapping
    // above lands when this frame's list executes, as a ring fill does).
    for (const auto& tile : aliases) {
        Tenant& t = m_tenants[tile->tenant];
        const auto it = t.held.find(tile->gkey);
        if (it != t.held.end() && it->second.landed) Landed(tile.get());
    }
    for (size_t i = 0; i + 1 < refills.size(); i += 2) {   // (held, job) pairs
        const std::shared_ptr<Tracked>& held = refills[i];
        const std::shared_ptr<Tracked>& job = refills[i + 1];
        Tenant& t = m_tenants[held->tenant];
        const D3D12_TILED_RESOURCE_COORDINATE coord{
            held->req.x, held->req.y, 0, held->req.face * t.mips + held->req.mip};
        const D3D12_TILE_REGION_SIZE size{1, FALSE, 0, 0, 0};
        memcpy(ring.cpu + off, job->data.data(), 65536);
        cl->CopyTiles(t.res.Get(), &coord, &size, ring.res.Get(), off,
                      D3D12_TILE_COPY_FLAG_LINEAR_BUFFER_TO_SWIZZLED_TILED_RESOURCE);
        off += 65536;
        ++m_ringTiles;
        held->stale = false;
        ++refreshedTotal;
        job->data.clear();
        job->data.shrink_to_fit();
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

void ResidencyManager::LabelSlices(int tenant, std::vector<std::string> tags,
                                   std::vector<double> ground0M) {
    Tenant& t = m_tenants[tenant];
    tags.resize(t.faces);
    ground0M.resize(t.faces, 0.0);
    // Step 5: the order's rung is ground, not a mip index: floor(log2 of the mip-0 texel in m).
    t.sliceRung0.assign(t.faces, kNoRung);
    for (uint32_t f = 0; f < t.faces; ++f) {
        if (ground0M[f] > 0.0) t.sliceRung0[f] = static_cast<int>(std::floor(std::log2(ground0M[f])));
    }
    t.sliceTag = std::move(tags);
    t.sliceGround0M = std::move(ground0M);
}

void ResidencyManager::LogPages() const {
    // Per tenant: one pass over its tracked tiles into (slice, mip) cells, then one line per run
    // of consecutive slices that sit on the same lattice. "wanted" is the settle's test (the
    // stamp says this frame, real or predicted); "held" is mapped and NOT wanted this frame --
    // what the pool keeps for nobody, the set a reclaim or a released page gives back.
    struct Cell {
        uint32_t wanted = 0, predicted = 0, mapped = 0, held = 0;
    };
    size_t poolSum = 0, shared = 0;
    for (const auto& tr : m_mapped) shared += tr->alias ? 1u : 0u;
    Log("[pages] rec%u f%u | pool %zu mapped (%zu of them shared mappings: one tile, many windows), %zu free | "
        "per mip: wanted/mapped+held",
        traceRecFrame, m_frame, m_mapped.size(), shared, m_freePool.size());
    for (size_t k = 0; k < m_tenants.size(); ++k) {
        const Tenant& t = m_tenants[k];
        if (t.tracked.empty()) continue;
        std::vector<Cell> cells(size_t(t.faces) * t.mips);
        for (const auto& tr : t.tracked) {
            const uint32_t st =
                t.stamp[StampIndex(t, tr->req.face, tr->req.mip, tr->req.x, tr->req.y)];
            const bool mapped = tr->state == TileState::Mapped;
            Cell& c = cells[size_t(tr->req.face) * t.mips + tr->req.mip];
            if ((st >> 1) == m_frame) {
                ++c.wanted;
                if (st & 1u) ++c.predicted;
                if (mapped) ++c.mapped;
            } else if (mapped) {
                ++c.held;
            }
        }
        uint32_t tw = 0, tm = 0, th = 0;
        for (const Cell& c : cells) {
            tw += c.wanted;
            tm += c.mapped;
            th += c.held;
        }
        poolSum += tm + th;
        Log("[pages]   %S: %u slices | wanted %u, mapped %u, held %u", t.name.c_str(), t.faces, tw,
            tm, th);
        auto tagOf = [&](uint32_t f) -> std::string {
            if (f < t.sliceTag.size() && !t.sliceTag[f].empty()) return t.sliceTag[f];
            return t.sliceTag.empty() ? "slice " + std::to_string(f) : std::string("unbound");
        };
        for (uint32_t f0 = 0; f0 < t.faces;) {
            uint32_t f1 = f0 + 1;
            const std::string tag = tagOf(f0);
            // An unlabelled tenant prints slice by slice; a labelled one merges a lattice's run.
            while (!t.sliceTag.empty() && f1 < t.faces && tagOf(f1) == tag) ++f1;
            Cell sum;
            std::string byMip;
            int finest = -1;
            for (uint32_t m = 0; m < t.mips; ++m) {
                Cell cm;
                for (uint32_t f = f0; f < f1; ++f) {
                    const Cell& c = cells[size_t(f) * t.mips + m];
                    cm.wanted += c.wanted;
                    cm.predicted += c.predicted;
                    cm.mapped += c.mapped;
                    cm.held += c.held;
                }
                sum.wanted += cm.wanted;
                sum.predicted += cm.predicted;
                sum.mapped += cm.mapped;
                sum.held += cm.held;
                if ((cm.mapped || cm.held) && finest < 0) finest = static_cast<int>(m);
                if (cm.wanted || cm.held) {
                    char b[48];
                    snprintf(b, sizeof(b), " m%u %u/%u+%u", m, cm.wanted, cm.mapped, cm.held);
                    byMip += b;
                }
            }
            char range[32];
            if (f1 - f0 == 1) snprintf(range, sizeof(range), "[%u]", f0);
            else snprintf(range, sizeof(range), "[%u..%u]", f0, f1 - 1);
            const double g0 = f0 < t.sliceGround0M.size() ? t.sliceGround0M[f0] : 0.0;
            char deep[48] = "nothing mapped";
            if (finest >= 0) {
                snprintf(deep, sizeof(deep), "finest m%d = %.4g m", finest,
                         g0 * double(1u << finest));
            }
            Log("[pages]     %-8s %-30s %9.4g m at m0 | wanted %u (%u predicted), mapped %u, "
                "unmapped %u, held %u | %s |%s",
                range, tag.c_str(), g0, sum.wanted, sum.predicted, sum.mapped,
                sum.wanted - sum.mapped, sum.held, deep, byMip.empty() ? " -" : byMip.c_str());
            f0 = f1;
        }
    }
    // Every mapped tile is tracked by exactly one tenant, so the tenants' mapped tiles ARE the
    // pool's. A disagreement is a tile the bookkeeping lost, and it is said, not summed away.
    if (poolSum != m_mapped.size()) {
        Log("[pages] POOL DISAGREES: the tenants hold %zu mapped tiles, the pool %zu", poolSum,
            m_mapped.size());
    }
}

void ResidencyManager::LogSettleExact(uint32_t heldFrames) const {
    // M13: WHO WANTED WHAT, when more than one reader is on the cache (silent otherwise).
    LogSamplers();
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
            "| dropped %u over the hold (%u were mapped) | mapped set %zu tiles, FNV-1a %016llx"
            " | magnified %u",
            t.name.c_str(), t.exWanted, t.exMapped, t.exDeficit, t.exUnreachable, t.exStale,
            t.exDropped, t.exDroppedMapped, keys.size(), static_cast<unsigned long long>(h),
            t.exMagnified);
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
    {
        // THE PARENT FIRST (4.7): the tiles of the order still waiting on a parent not held, and
        // that parent -- its state, whether it lands, whether this turn wants it.
        uint32_t waiting = 0;
        std::string named;
        for (const OrderEntry& e : m_need) {
            const Tracked* tr = e.tile;
            if (tr->state == TileState::Mapped && tr->landed) continue;
            const Tracked* at = nullptr;
            if (ParentGate(m_tenants[tr->tenant], tr, &at) != Gate::Wait) continue;
            if (++waiting > 6) continue;
            char b[320];
            if (!at) {
                snprintf(b, sizeof(b), " %s under an UNTRACKED parent;", TileName(MakeKey(tr->tenant, tr->req)).c_str());
            } else {
                const Tenant& t = m_tenants[at->tenant];
                const size_t idx = StampIndex(t, at->req.face, at->req.mip, at->req.x, at->req.y);
                snprintf(b, sizeof(b), " %s under %s (state %d, landed %d, retries %d, wanted now %d, pos %s);",
                         TileName(MakeKey(tr->tenant, tr->req)).c_str(),
                         TileName(MakeKey(at->tenant, at->req)).c_str(), int(at->state), int(at->landed),
                         int(at->retries), int((t.stamp[idx] >> 1) == m_frame),
                         at->pos == UINT32_MAX ? "untracked" : "tracked");
            }
            named += b;
        }
        if (waiting) Log("[settle-exact]   waiting on a parent not held: %u;%s", waiting, named.c_str());
    }
    const size_t poolTiles = m_mapped.size();
    {
        // Step 5: the order does not grow the pool in a hold. What it holds is the want's first
        // P; the rest is the want's own tail, named per tenant.
        std::string tail;
        uint32_t lost = 0;
        for (const auto& t : m_tenants) {
            if (!t.exLost) continue;
            lost += t.exLost;
            char b[160];
            snprintf(b, sizeof(b), " %S %u;", t.name.c_str(), t.exLost);
            tail += b;
        }
        Log("[settle-exact] order: %zu tiles mapped against a cut of %u (the want set: the hold "
            "lifts the pool's cap); the want's tail lost to it: %u tiles%s",
            poolTiles, m_passCut, lost, tail.empty() ? "" : (" --" + tail).c_str());
        // What the cut P would lose at this pose, outside the hold: by tenant and by rung.
        uint32_t byT[16] = {}, byR[kRungs] = {}, tailP = 0, wantedN = 0;
        CountTail(kPoolCapTiles - kSlotReserve, byT, byR, tailP, wantedN);
        std::string ten, rung;
        for (uint32_t k = 0; k < m_tenants.size() && k < 16; ++k) {
            if (!byT[k]) continue;
            char b[160];
            snprintf(b, sizeof(b), " %S %u;", m_tenants[k].name.c_str(), byT[k]);
            ten += b;
        }
        for (uint32_t r = 0; r < kRungs; ++r) {
            if (!byR[r]) continue;
            char b[48];
            snprintf(b, sizeof(b), " 2^%d m %u;", static_cast<int>(kRungTop) - static_cast<int>(r),
                     byR[r]);
            rung += b;
        }
        if (incompleteTotal) {
            Log("[settle-exact] order: %llu tiles were answered not whole (a source refused) and "
                "are unreachable for the run (finding 83)",
                static_cast<unsigned long long>(incompleteTotal));
        }
        Log("[settle-exact] order: at this pose the cut P = %u would keep %u of the %u wanted and "
            "lose %u | by tenant:%s | by rung (texel):%s",
            kPoolCapTiles - kSlotReserve, wantedN - tailP, wantedN, tailP,
            ten.empty() ? " -" : ten.c_str(), rung.empty() ? " -" : rung.c_str());
    }
    Log("[settle-exact] pool: %zu tiles mapped (%.0f MB); the pool's budget is %u tiles (%.0f MB)",
        poolTiles, poolTiles / 16.0, kPoolCapTiles, kPoolCapTiles / 16.0);
    Log("[settle-exact] held %u frames: wanted %u, mapped %u, deficit %u, unreachable %u, stale "
        "%u, pending %u, in-flight reads %u, retiring %u, magnified %u, released under a refused "
        "parent %llu (must be 0) -- %s",
        heldFrames, settleTurn.wanted, settleTurn.mapped, settleTurn.deficit,
        settleTurn.unreachable, settleTurn.stale, settleTurn.pending, settleTurn.reads,
        settleTurn.retiring, settleTurn.magnified,
        static_cast<unsigned long long>(releasedUnderRefusedTotal),
        settleTurn.exact ? "EXACT" : "not exact");
}

void ResidencyManager::RegisterField(const char* name, std::function<uint64_t()> residentBytes,
                                     std::function<uint32_t()> residentTiles) {
    m_fields.push_back({name, std::move(residentBytes), std::move(residentTiles)});
}

void ResidencyManager::PublishSignatures(const std::string& field, uint32_t tilesX,
                                         uint32_t tilesY, std::vector<uint8_t> sig,
                                         const Lattice* lattice) {
    m_signatures[field] = {tilesX, tilesY, std::move(sig), lattice};
}

bool ResidencyManager::DeriveDemand(const std::string& srcA, const std::string& srcB,
                                    std::vector<uint8_t>& out, uint32_t& tilesX,
                                    uint32_t& tilesY) const {
    const auto a = m_signatures.find(srcA);
    const auto b = m_signatures.find(srcB);
    if (a == m_signatures.end() || b == m_signatures.end()) return false;
    // M12 step 3e: THE GROUND IS PART OF THE OPERAND. An index-by-index product of two grids
    // is a statement about the ground only when both grids tile the same one, so a publisher
    // that declared its lattice is held to it: two declared lattices that are not SameGround
    // are refused, once aloud, as an empty demand -- a resample node is the missing piece, not
    // a silent mis-registration. The shipped operands (wind10m with itself) share the grid, so
    // this never fires today; it is the contract the next operand pair meets.
    if (a->second.lattice && b->second.lattice &&
        !a->second.lattice->SameGround(*b->second.lattice)) {
        if (!m_groundRefusalSaid) {
            m_groundRefusalSaid = true;
            Log("[residency] DeriveDemand %s x %s: lattices %s and %s are not the same ground "
                "-- refused (empty demand)",
                srcA.c_str(), srcB.c_str(), a->second.lattice->Tag().c_str(),
                b->second.lattice->Tag().c_str());
        }
        out.clear();
        return false;
    }
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
