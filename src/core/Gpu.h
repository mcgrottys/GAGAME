// ================================================================================================
//  Gpu - the D3D12 device layer. Carried from vqview-inlet; the design notes below still hold.
//
//  THREE DECISIONS HERE ARE FOR THE FUTURE, NOT FOR TODAY
//  ------------------------------------------------------
//  1. ONE BIG SHADER-VISIBLE SRV HEAP, ADDRESSED BY INDEX.
//     Shaders read textures out of an unbounded descriptor table -- `Texture2D g_tex[] :
//     register(t0, space1)` -- so a texture is identified by a uint32 heap slot, not by a root
//     signature slot. It also happens to be exactly the indirection tiled resources need: a
//     reserved-resource tile pool is still just slots. (GAGAME uses that today, in TileAtlas.)
//
//  2. FRAME RING WITH A FENCE PER FRAME, NOT WaitIdle PER FRAME.
//
//  3. UPLOAD IS A PER-FRAME LINEAR ARENA, PERSISTENTLY MAPPED.
//     Constants are bump-allocated and handed out as GPU virtual addresses.
//
//  GAGAME additions over vqview: Queue() (UpdateTileMappings is a queue-side operation),
//  CreateTextureUav / CreateSrv3D (the tile self-test and future atlas banks need UAVs and 3D
//  views), and the tiled-resources tier is stored, not just logged (M0's whole point).
// ================================================================================================
#pragma once

#include "core/Common.h"

#include <d3d12.h>
#include <dxgi1_6.h>
#include <vector>

namespace ga {

// ------------------------------------------------------------------ small resource wrappers

struct GpuBuffer {
    Com<ID3D12Resource> res;
    D3D12_GPU_VIRTUAL_ADDRESS gpu = 0;
    uint8_t* cpu = nullptr;   // non-null only for upload-heap buffers
    uint64_t size = 0;
    bool Valid() const { return res != nullptr; }
};

struct GpuTexture {
    Com<ID3D12Resource> res;
    DXGI_FORMAT format = DXGI_FORMAT_UNKNOWN;
    uint32_t width = 0, height = 0;
    uint32_t srv = UINT32_MAX;   // slot in the shader-visible SRV heap; this is the shader handle
    D3D12_RESOURCE_STATES state = D3D12_RESOURCE_STATE_COMMON;
    bool Valid() const { return res != nullptr; }
};

// ------------------------------------------------------------------ descriptor heap

class DescriptorHeap {
public:
    void Init(ID3D12Device* dev, D3D12_DESCRIPTOR_HEAP_TYPE type, uint32_t capacity,
              bool shaderVisible, const wchar_t* name);
    uint32_t Alloc(uint32_t count = 1);
    D3D12_CPU_DESCRIPTOR_HANDLE Cpu(uint32_t index) const;
    D3D12_GPU_DESCRIPTOR_HANDLE Gpu(uint32_t index) const;
    ID3D12DescriptorHeap* Heap() const { return m_heap.Get(); }
    uint32_t Used() const { return m_used; }
    uint32_t Capacity() const { return m_capacity; }

private:
    Com<ID3D12DescriptorHeap> m_heap;
    D3D12_CPU_DESCRIPTOR_HANDLE m_cpuStart{};
    D3D12_GPU_DESCRIPTOR_HANDLE m_gpuStart{};
    uint32_t m_stride = 0, m_used = 0, m_capacity = 0;
    bool m_shaderVisible = false;
};

// ------------------------------------------------------------------ the device

class Gpu {
public:
    static constexpr uint32_t kFrameCount = 2;
    static constexpr uint32_t kSrvHeapCapacity = 4096;

    // hwnd may be null: that is headless mode, which creates no swapchain. Headless exists so the
    // renderer (and the tile self-test) can be verified from a shell with no desktop session.
    void Init(HWND hwnd, uint32_t width, uint32_t height, bool wantDebugLayer);
    void Shutdown();

    ID3D12Device* Device() const { return m_device.Get(); }
    // UpdateTileMappings lives on the queue, not the command list; the atlas needs this.
    ID3D12CommandQueue* Queue() const { return m_queue.Get(); }
    bool Headless() const { return m_hwnd == nullptr; }
    uint32_t Width() const { return m_width; }
    uint32_t Height() const { return m_height; }
    uint32_t FrameIndex() const { return m_frameIndex; }

    // Queried once at Init. Tier 2 is the load-bearing guarantee (NULL-mapped tiles read as zero,
    // writes discarded); Tier 3 adds 3D tiled resources for future volume banks.
    D3D12_TILED_RESOURCES_TIER TiledTier() const { return m_tiledTier; }

    DescriptorHeap& SrvHeap() { return m_srvHeap; }
    DescriptorHeap& RtvHeap() { return m_rtvHeap; }
    DescriptorHeap& DsvHeap() { return m_dsvHeap; }

    // ---- frame lifecycle
    ID3D12GraphicsCommandList* BeginFrame();
    void EndFrame(bool present);
    void WaitIdle();
    void Resize(uint32_t width, uint32_t height);

    ID3D12Resource* BackBuffer() const;             // null when headless
    D3D12_CPU_DESCRIPTOR_HANDLE BackBufferRtv() const;

    // ---- per-frame linear constant upload. Returns a GPU VA usable with
    // SetGraphicsRootConstantBufferView. Alignment is handled.
    D3D12_GPU_VIRTUAL_ADDRESS PushConstants(const void* data, size_t bytes);

    // ---- resource creation
    GpuBuffer CreateUploadBuffer(uint64_t bytes, const wchar_t* name);
    GpuBuffer CreateDefaultBuffer(const void* data, uint64_t bytes, const wchar_t* name);
    GpuTexture CreateTexture2D(uint32_t w, uint32_t h, DXGI_FORMAT fmt,
                               D3D12_RESOURCE_FLAGS flags, D3D12_RESOURCE_STATES initialState,
                               const wchar_t* name, const D3D12_CLEAR_VALUE* clear = nullptr,
                               uint16_t mips = 1);
    // Uploads tightly-packed rows and leaves the texture in PIXEL_SHADER_RESOURCE.
    void UploadTexture(GpuTexture& tex, const void* rows, uint32_t srcRowPitchBytes,
                       uint32_t mip = 0);
    uint32_t CreateSrv(ID3D12Resource* res, DXGI_FORMAT fmt);
    uint32_t CreateSrv3D(ID3D12Resource* res, DXGI_FORMAT fmt);
    uint32_t CreateStructuredBufferSrv(ID3D12Resource* res, uint32_t numElements,
                                       uint32_t strideBytes);
    // UAV for a texture (2D or 3D decided by `dim`). Allocated from the shader-visible heap so
    // compute passes can reach it through a descriptor table.
    uint32_t CreateTextureUav(ID3D12Resource* res, DXGI_FORMAT fmt, D3D12_UAV_DIMENSION dim);

    void Transition(ID3D12GraphicsCommandList* cl, GpuTexture& tex, D3D12_RESOURCE_STATES to);

    // ---- readback: copies a texture to system memory. Synchronous; only used by --dump.
    std::vector<uint8_t> ReadbackTexture(GpuTexture& tex, uint32_t* outRowPitch);
    // M7l: one texel of one subresource -- the hypervisor's compose-tile cross-check.
    // Returns up to 16 bytes of the texel in out; true on success.
    bool ReadbackTexel(ID3D12Resource* res, uint32_t subresource, uint32_t x, uint32_t y,
                       D3D12_RESOURCE_STATES state, uint8_t out[16]);
    // Readback for a plain buffer (self-test results). Synchronous.
    std::vector<uint8_t> ReadbackBuffer(ID3D12Resource* buf, uint64_t bytes,
                                        D3D12_RESOURCE_STATES currentState);

    // Immediate-submit helper for one-off setup work (texture uploads, buffer initialisation).
    ID3D12GraphicsCommandList* BeginUpload();
    void EndUpload();

    // Rewind the current frame's constant arena. ONLY legal when the GPU is provably idle --
    // i.e. right after EndUpload (which waits) -- used by solver spin-up loops that submit many
    // batches outside the frame lifecycle and would otherwise exhaust the 4 MB arena.
    void ResetConstantArenaAfterIdle() { m_cbOffset[m_frameIndex] = 0; }

private:
    void CreateSwapchain();
    void CreateFrameResources();

    HWND m_hwnd = nullptr;
    uint32_t m_width = 0, m_height = 0;

    Com<IDXGIFactory6> m_factory;
    Com<ID3D12Device> m_device;
    Com<ID3D12CommandQueue> m_queue;
    Com<IDXGISwapChain3> m_swapchain;

    Com<ID3D12Resource> m_backBuffers[kFrameCount];
    uint32_t m_backBufferRtv[kFrameCount] = {};

    Com<ID3D12CommandAllocator> m_alloc[kFrameCount];
    Com<ID3D12GraphicsCommandList> m_cmdList;
    GpuBuffer m_cbArena[kFrameCount];
    uint64_t m_cbOffset[kFrameCount] = {};

    Com<ID3D12Fence> m_fence;
    HANDLE m_fenceEvent = nullptr;
    uint64_t m_fenceValue = 0;
    uint64_t m_frameFence[kFrameCount] = {};
    uint32_t m_frameIndex = 0;

    D3D12_TILED_RESOURCES_TIER m_tiledTier = D3D12_TILED_RESOURCES_TIER_NOT_SUPPORTED;

    // one-off upload path
    Com<ID3D12CommandAllocator> m_uploadAlloc;
    Com<ID3D12GraphicsCommandList> m_uploadList;
    std::vector<Com<ID3D12Resource>> m_uploadKeepAlive;

    DescriptorHeap m_srvHeap, m_rtvHeap, m_dsvHeap;
};

}  // namespace ga
