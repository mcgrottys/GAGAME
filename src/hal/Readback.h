// ================================================================================================
//  hal::RegionReadback - texture REGIONS brought back to the CPU without a stall.
//
//  The frame ring already waits: Gpu::BeginFrame blocks on the fence of the frame that last used
//  the slot it is about to reuse, never on the GPU going idle. So a copy recorded into frame slot s
//  has landed by the time slot s is recorded again, and reading it then costs nothing more -- the
//  GpuProfiler's pattern (its query resolves), generalised to copies of texture regions. Latency is
//  Gpu::kFrameCount frames. There is no WaitIdle here and there never will be: a consumer that needs
//  an answer sooner than the ring delivers it needs a different design, not a flush.
//
//  THE CONTRACT A CALLER KEEPS
//    - BeginSlot once per frame, while recording that frame's list (Owner::Frame), before any copy:
//      it reads what the slot's previous frame copied (Delivered) and opens the slot again.
//    - The source is in COPY_SOURCE when CopyRegion records against it (the caller owns its states).
//    - Regions of reserved (tiled) textures are fine: unmapped tiles read zero, as they do to shaders.
// ================================================================================================
#pragma once

#include "hal/Context.h"
#include "hal/Gpu.h"

#include <cstdint>
#include <vector>

namespace ga::hal {

class RegionReadback {
public:
    // bytesPerSlot bounds what one frame may copy (every region's texels, row-pitched).
    void Init(Gpu& gpu, uint64_t bytesPerSlot, const wchar_t* name);
    bool Ready() const { return m_buffer != nullptr; }

    // One region as it came back: `w` x `h` texels of `texelBytes` each, de-pitched, row 0 first.
    // `tag` is the caller's, carried from CopyRegion.
    struct Region {
        uint32_t tag = 0;
        uint32_t x0 = 0, y0 = 0, w = 0, h = 0, texelBytes = 0;
        std::vector<uint8_t> bytes;
    };

    // Open this frame's slot (the Gpu's frame index): what that slot's previous frame copied is read
    // into Delivered() first. Returns the number of regions delivered.
    size_t BeginSlot(uint32_t frameIndex);
    const std::vector<Region>& Delivered() const { return m_delivered; }

    // Record a copy of [x0, x0 + w) x [y0, y0 + h) of subresource `sub` of `src` (texels `fmt`: R32F,
    // R16F, RG32F, RGBA16F or RGBA32F) into the open slot. False, and nothing recorded, when the slot
    // has no room or the format is not one of those.
    bool CopyRegion(CommandContext& cmd, Resource src, uint32_t sub, DXGI_FORMAT fmt, uint32_t x0,
                    uint32_t y0, uint32_t w, uint32_t h, uint32_t tag);

private:
    struct Copy {
        uint64_t offset = 0;
        uint32_t rowPitch = 0;
        Region region;   // bytes empty until read
    };
    struct Slot {
        bool pending = false;
        uint64_t used = 0;
        std::vector<Copy> copies;
    };
    Gpu* m_gpu = nullptr;
    Com<ID3D12Resource> m_buffer;
    uint64_t m_bytesPerSlot = 0;
    Slot m_slots[Gpu::kFrameCount];
    uint32_t m_cur = UINT32_MAX;
    std::vector<Region> m_delivered;
};

}  // namespace ga::hal
