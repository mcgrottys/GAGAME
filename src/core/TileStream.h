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
#include <deque>
#include <string>
#include <vector>

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
        // Device-local landing ground. DirectStorage writes here; CopyTiles swizzles from here
        // into the reserved resource once the fence lands.
        m_staging = gpu.CreateDefaultBuffer(nullptr, kStageSlots * 65536ull,
                                            L"dstorage.staging");
        if (!m_staging.res) {
            m_queue.Reset();
            m_factory.Reset();
            return false;
        }
        m_ok = true;
        Log("[dstorage] ready: %u MB read staging, %u MB device landing buffer (%u tiles), "
            "queue capacity %u -- NVMe -> GPU, no CPU copy",
            kStagingBytes / (1024u * 1024u), uint32_t(kStageSlots * 65536ull / 1048576ull),
            kStageSlots, uint32_t(DSTORAGE_MAX_QUEUE_CAPACITY));
        return true;
#else
        (void)gpu;
        Log("[dstorage] not compiled in (redist absent at configure time) -- ReadFile path");
        return false;
#endif
    }

    bool Available() const { return m_ok; }

    // ---- THE DESTINATION IS A BUFFER, NOT A TILE, AND THAT IS NOT AN OPTIMISATION MISSED.
    //
    // The first version wrote straight to DESTINATION_TILES and produced corrupt terrain --
    // washed-out colour with diagonal streaks, which is what a linear image looks like when it
    // is read as a swizzled one. Compositor's "laid out exactly for CopyTiles" means laid out
    // FOR THAT CONVERSION: CopyTiles is called with LINEAR_BUFFER_TO_SWIZZLED_TILED_RESOURCE,
    // so the GPU performs the swizzle during the copy and the cached blob is linear.
    // DESTINATION_TILES writes bytes verbatim into the tile and does no such thing.
    //
    // Pre-swizzling at pack time is not the fix: the tile layout is vendor and format specific
    // and not something a cache written on one machine may assume about another.
    //
    // So the read lands in a DEVICE-LOCAL buffer and the existing CopyTiles does the swizzle,
    // unchanged, one frame later when the fence says the bytes are there. The CPU still never
    // addresses them -- which was always the point -- and the swizzle stays where the hardware
    // wants it. What this costs is a frame of latency and a staging buffer; what it buys is the
    // read never entering CPU address space.
    bool EnqueueToBuffer(const std::wstring& path, uint64_t srcOffset, uint32_t bytes,
                         uint64_t dstOffset) {
#if defined(GA_HAVE_DSTORAGE)
        if (!m_ok || !m_staging.res) return false;
        Com<IDStorageFile> file;
        if (FAILED(m_factory->OpenFile(path.c_str(), IID_PPV_ARGS(&file)))) return false;
        DSTORAGE_REQUEST r{};
        r.Options.SourceType = DSTORAGE_REQUEST_SOURCE_FILE;
        r.Options.DestinationType = DSTORAGE_REQUEST_DESTINATION_BUFFER;
        r.Source.File.Source = file.Get();
        r.Source.File.Offset = srcOffset;
        r.Source.File.Size = bytes;
        r.Destination.Buffer.Resource = m_staging.res.Get();
        r.Destination.Buffer.Offset = dstOffset;
        r.Destination.Buffer.Size = bytes;
        r.UncompressedSize = bytes;
        m_queue->EnqueueRequest(&r);
        ++m_pending;
        m_open.push_back(std::move(file));   // owned until THIS batch's fence lands
        return true;
#else
        (void)path; (void)srcOffset; (void)bytes; (void)dstOffset;
        return false;
#endif
    }

    ID3D12Resource* StagingBuffer() const { return m_staging.res.Get(); }
    uint64_t StagingBytes() const { return kStageSlots * 65536ull; }
    static constexpr uint32_t kStageSlots = 512;   // 32 MB of device-local landing ground

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
        // M9ap: the files this submit reads from travel WITH its fence. Releasing them on any
        // later fence is what let a reference into a second archive be read from a closed file.
        m_batches.push_back({m_fenceValue, std::move(m_open)});
        m_open.clear();
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

    // Files can only be released once the reads that reference them have landed -- and ONLY
    // those. The previous form cleared every open file when any fence completed: with one
    // archive per tenant that was a harmless simplification; with stored references resolving
    // into many archives it released files whose reads were still queued behind a later fence,
    // and the tiles came back as whatever the landing slot last held. Green and magenta on the
    // globe was that.
    void ReleaseCompleted(uint64_t value) {
#if defined(GA_HAVE_DSTORAGE)
        (void)value;
        if (!m_ok) return;
        const uint64_t done = m_fence->GetCompletedValue();
        while (!m_batches.empty() && m_batches.front().fence <= done) m_batches.pop_front();
        // M9ap: ASK WHETHER THE READS SUCCEEDED. A failed request leaves its landing slot
        // holding whatever tile last used it, and CopyTiles copies that stale tile into the new
        // coordinate without complaint -- oceans beside mountains. Nothing had ever asked.
        DSTORAGE_ERROR_RECORD rec{};
        m_queue->RetrieveErrorRecord(&rec);
        if (rec.FailureCount) {
            m_failures += rec.FailureCount;
            if (m_failLogged < 8) {
                ++m_failLogged;
                const auto& f = rec.FirstFailure;
                Log("[dstorage] %u FAILED request(s) in this record; first: hr 0x%08x, cmd %d "
                    "-- the landing slot kept a STALE tile",
                    rec.FailureCount, static_cast<unsigned>(f.HResult),
                    static_cast<int>(f.CommandType));
            }
        }
#else
        (void)value;
#endif
    }

    uint64_t Submitted() const { return m_submitted; }
    uint64_t Failures() const { return m_failures; }

private:
    static constexpr uint32_t kStagingBytes = 32u * 1024u * 1024u;
    GpuBuffer m_staging;
    bool m_ok = false;
    uint64_t m_fenceValue = 0, m_submitted = 0, m_failures = 0;
    uint32_t m_failLogged = 0;
    uint32_t m_pending = 0;
#if defined(GA_HAVE_DSTORAGE)
    Com<IDStorageFactory> m_factory;
    Com<IDStorageQueue> m_queue;
    Com<ID3D12Fence> m_fence;
    std::vector<Com<IDStorageFile>> m_open;   // enqueued, not yet submitted
    struct Batch {
        uint64_t fence;
        std::vector<Com<IDStorageFile>> files;
    };
    std::deque<Batch> m_batches;   // submitted, oldest first; released as fences land
#endif
};

}  // namespace ga
