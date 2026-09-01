// ================================================================================================
//  TileStream - M9ag: THE NVMe -> GPU PATH.
//
//  The composed cache was built for this from the beginning. Compositor.h has said so since it
//  was written: 64 KB tiles "laid out exactly for CopyTiles, which is the DirectStorage-ready
//  folder". 20666 of them, 1.3 GB, one file per tile, already the exact size and layout a tiled
//  resource wants. What was missing was the reader.
//
//  WHAT THE OLD PATH COSTS. ReadCached opens the file, reads 64 KB into a std::vector on a
//  worker thread, and the tile then rides an upload heap into the resource. Every tile is two
//  copies (disk -> heap-allocated vector -> staging) and one synchronous open per tile, on a
//  thread that is also painting. DirectStorage hands the queue a file handle, an offset and a
//  destination, and the bytes go disk -> GPU with the CPU never addressing them.
//
//  DEGRADING HONESTLY IS THE POINT. DirectStorage ships as a NuGet redistributable, not in the
//  Windows SDK, and it needs a driver and a DLL that a given machine may not have. So this is
//  compiled in only when the package is found (GA_HAVE_DSTORAGE), the factory is created ONCE
//  and may fail at runtime, and Available() answers the truth. A caller that gets false uses the
//  ReadFile path it already has. Nothing here is allowed to make a machine without the redist
//  worse than it was, which is why the fallback is the ORIGINAL code rather than a reimplementation
//  of it.
//
//  ONE FILE PER TILE IS NOT THE FASTEST SHAPE and this does not pretend otherwise. DirectStorage
//  is happiest with few large files and many offsets into them, because per-file open still costs
//  a system call even when the read does not. The cache is one file per tile today, so what this
//  buys is the copy elimination and the asynchrony, not the open. Packing a realization into one
//  archive with the index carrying (offset, size) is the natural follow-on -- and TileIndex
//  already stores exactly those two fields per entry, which is why they are there.
// ================================================================================================
#pragma once
#include <cstdint>
#include <string>

#include "core/Common.h"
#include "core/Gpu.h"

#if defined(GA_HAVE_DSTORAGE)
#include <dstorage.h>
#endif

namespace ga {

class TileStream {
public:
    // Created once, next to the device. Failure is normal and reported, not thrown: a machine
    // without the runtime, or with a driver that declines, must keep rendering.
    bool Init(Gpu& gpu) {
#if defined(GA_HAVE_DSTORAGE)
        if (FAILED(DStorageGetFactory(IID_PPV_ARGS(&m_factory)))) {
            Log("[dstorage] factory unavailable -- tile reads stay on the ReadFile path");
            return false;
        }
        m_factory->SetStagingBufferSize(kStagingBytes);
        DSTORAGE_QUEUE_DESC qd{};
        qd.Capacity = DSTORAGE_MAX_QUEUE_CAPACITY;
        qd.Priority = DSTORAGE_PRIORITY_NORMAL;
        qd.SourceType = DSTORAGE_REQUEST_SOURCE_FILE;
        qd.Device = gpu.Device();
        if (FAILED(m_factory->CreateQueue(&qd, IID_PPV_ARGS(&m_queue)))) {
            Log("[dstorage] queue creation failed -- tile reads stay on the ReadFile path");
            m_factory.Reset();
            return false;
        }
        if (FAILED(gpu.Device()->CreateFence(0, D3D12_FENCE_FLAG_NONE, IID_PPV_ARGS(&m_fence)))) {
            m_queue.Reset();
            m_factory.Reset();
            return false;
        }
        m_ok = true;
        Log("[dstorage] ready: %u MB staging, queue capacity %u -- NVMe -> GPU, no CPU copy",
            kStagingBytes / (1024u * 1024u), uint32_t(DSTORAGE_MAX_QUEUE_CAPACITY));
        return true;
#else
        (void)gpu;
        Log("[dstorage] not compiled in (redist absent at configure time) -- ReadFile path");
        return false;
#endif
    }

    bool Available() const { return m_ok; }

    // Enqueue one 64 KB tile straight into a reserved resource's tile. The destination is a
    // TILED REGION, which is why the cache's layout matters: the bytes are already in the
    // hardware's tile order, so this is a copy with no swizzle and no intermediate.
    bool EnqueueTile(const std::wstring& path, ID3D12Resource* dst,
                     const D3D12_TILED_RESOURCE_COORDINATE& coord, uint32_t bytes) {
#if defined(GA_HAVE_DSTORAGE)
        if (!m_ok) return false;
        Com<IDStorageFile> file;
        if (FAILED(m_factory->OpenFile(path.c_str(), IID_PPV_ARGS(&file)))) return false;
        DSTORAGE_REQUEST r{};
        r.Options.SourceType = DSTORAGE_REQUEST_SOURCE_FILE;
        r.Options.DestinationType = DSTORAGE_REQUEST_DESTINATION_TILES;
        r.Source.File.Source = file.Get();
        r.Source.File.Offset = 0;
        r.Source.File.Size = bytes;
        r.Destination.Tiles.Resource = dst;
        r.Destination.Tiles.TiledRegionStartCoordinate = coord;
        r.Destination.Tiles.TileRegionSize = {1, FALSE, 0, 0, 0};
        r.UncompressedSize = bytes;
        m_queue->EnqueueRequest(&r);
        ++m_pending;
        m_files.push_back(std::move(file));   // the queue borrows it until Submit completes
        return true;
#else
        (void)path; (void)dst; (void)coord; (void)bytes;
        return false;
#endif
    }

    // Submit what is queued and return the fence value to wait on. Batching matters more here
    // than anywhere else in the engine: a submit is the unit the hardware pipelines, so one
    // submit of forty tiles is not forty times a submit of one.
    uint64_t Submit() {
#if defined(GA_HAVE_DSTORAGE)
        if (!m_ok || !m_pending) return m_fenceValue;
        ++m_fenceValue;
        m_queue->EnqueueSignal(m_fence.Get(), m_fenceValue);
        m_queue->Submit();
        m_submitted += m_pending;
        m_pending = 0;
        return m_fenceValue;
#else
        return 0;
#endif
    }

    bool Complete(uint64_t value) const {
#if defined(GA_HAVE_DSTORAGE)
        return !m_ok || m_fence->GetCompletedValue() >= value;
#else
        (void)value;
        return true;
#endif
    }

    // Files can only be released once the reads that reference them have landed.
    void ReleaseCompleted(uint64_t value) {
#if defined(GA_HAVE_DSTORAGE)
        if (m_ok && m_fence->GetCompletedValue() >= value) m_files.clear();
#else
        (void)value;
#endif
    }

    uint64_t Submitted() const { return m_submitted; }

private:
    static constexpr uint32_t kStagingBytes = 32u * 1024u * 1024u;
    bool m_ok = false;
    uint64_t m_fenceValue = 0, m_submitted = 0;
    uint32_t m_pending = 0;
#if defined(GA_HAVE_DSTORAGE)
    Com<IDStorageFactory> m_factory;
    Com<IDStorageQueue> m_queue;
    Com<ID3D12Fence> m_fence;
    std::vector<Com<IDStorageFile>> m_files;
#endif
};

}  // namespace ga
