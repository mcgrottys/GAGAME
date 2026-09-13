// ================================================================================================
//  Views.h - M12 step 3d: THE DESCRIPTOR TABLE, and the views that fill it.
//
//  WHAT THE LAYERS WERE DOING. Seven compute kernels each allocate a contiguous run of the
//  shader-visible heap (SrvHeap().Alloc(n)), fill slot base+i with a hand-built
//  D3D12_SHADER_RESOURCE_VIEW_DESC or D3D12_UNORDERED_ACCESS_VIEW_DESC through the raw device,
//  and bind the run with ComputeTable(param, base). Twenty-five such views outside src/hal/,
//  in six shapes: Texture2D, Texture2DArray and Texture3D, as SRV and as UAV -- and every one
//  of them is a slot of a table; no layer makes a view that stands alone. Two of the tables
//  are re-filled later (the churn kernel's solver and bed slots once the solver exists; the
//  solver's own bed slots when the height page is bound), so a table is an OBJECT that keeps
//  its base, its count and its device, not a number a layer adds to.
//
//  THE DESC IS THE LAW. Each creator builds the one desc its shape needs, with every field an
//  old site set: Shader4ComponentMapping (the default swizzle, always), the mip window
//  (MostDetailedMip + MipLevels), the slice window (FirstArraySlice + ArraySize, UINT32_MAX
//  for every slice), the UAV's MipSlice, the 3D UAV's whole depth (WSize -1). PlaneSlice and
//  ResourceMinLODClamp stay 0: no site ever set them. That the creators' descs ARE the old
//  sites' was step 3d's gate: every view site compared its hand-built desc with the creator's,
//  bitwise, in one probe run, before the hand-built block was deleted.
//
//  THE RENDERER'S TARGETS live in the device's RTV and DSV heaps (Gpu.h: 64 and 16 slots, not
//  shader-visible). A target is allocated once and RE-CREATED into the same slot on every
//  resize, so Rtv() and Dsv() take the slot back: UINT32_MAX allocates. An RTV takes the
//  resource's own format (no desc); a DSV is the format over Texture2D mip 0.
//
//  DX12-first: the formats are DXGI's, the resources ID3D12Resource*, the slot the heap's own
//  index -- the uint32 a shader indexes the bindless table with. Nothing here is virtual.
// ================================================================================================
#pragma once

#include "hal/Gpu.h"

#include <cstdint>
#include <stdexcept>
#include <string>

namespace ga::hal {

// ---- the desc law: one function per view shape ----------------------------------------------
inline D3D12_SHADER_RESOURCE_VIEW_DESC SrvDesc2D(DXGI_FORMAT fmt, uint32_t mostDetailedMip,
                                                 uint32_t mipLevels) {
    D3D12_SHADER_RESOURCE_VIEW_DESC s{};
    s.Shader4ComponentMapping = D3D12_DEFAULT_SHADER_4_COMPONENT_MAPPING;
    s.Format = fmt;
    s.ViewDimension = D3D12_SRV_DIMENSION_TEXTURE2D;
    s.Texture2D.MostDetailedMip = mostDetailedMip;
    s.Texture2D.MipLevels = mipLevels;
    return s;
}
inline D3D12_SHADER_RESOURCE_VIEW_DESC SrvDescArray(DXGI_FORMAT fmt, uint32_t firstSlice,
                                                    uint32_t sliceCount, uint32_t mostDetailedMip,
                                                    uint32_t mipLevels) {
    D3D12_SHADER_RESOURCE_VIEW_DESC s{};
    s.Shader4ComponentMapping = D3D12_DEFAULT_SHADER_4_COMPONENT_MAPPING;
    s.Format = fmt;
    s.ViewDimension = D3D12_SRV_DIMENSION_TEXTURE2DARRAY;
    s.Texture2DArray.MostDetailedMip = mostDetailedMip;
    s.Texture2DArray.MipLevels = mipLevels;
    s.Texture2DArray.FirstArraySlice = firstSlice;
    s.Texture2DArray.ArraySize = sliceCount;
    return s;
}
inline D3D12_SHADER_RESOURCE_VIEW_DESC SrvDesc3D(DXGI_FORMAT fmt, uint32_t mostDetailedMip,
                                                 uint32_t mipLevels) {
    D3D12_SHADER_RESOURCE_VIEW_DESC s{};
    s.Shader4ComponentMapping = D3D12_DEFAULT_SHADER_4_COMPONENT_MAPPING;
    s.Format = fmt;
    s.ViewDimension = D3D12_SRV_DIMENSION_TEXTURE3D;
    s.Texture3D.MostDetailedMip = mostDetailedMip;
    s.Texture3D.MipLevels = mipLevels;
    return s;
}
inline D3D12_UNORDERED_ACCESS_VIEW_DESC UavDesc2D(DXGI_FORMAT fmt, uint32_t mip) {
    D3D12_UNORDERED_ACCESS_VIEW_DESC u{};
    u.Format = fmt;
    u.ViewDimension = D3D12_UAV_DIMENSION_TEXTURE2D;
    u.Texture2D.MipSlice = mip;
    return u;
}
inline D3D12_UNORDERED_ACCESS_VIEW_DESC UavDescArray(DXGI_FORMAT fmt, uint32_t firstSlice,
                                                     uint32_t sliceCount, uint32_t mip) {
    D3D12_UNORDERED_ACCESS_VIEW_DESC u{};
    u.Format = fmt;
    u.ViewDimension = D3D12_UAV_DIMENSION_TEXTURE2DARRAY;
    u.Texture2DArray.MipSlice = mip;
    u.Texture2DArray.FirstArraySlice = firstSlice;
    u.Texture2DArray.ArraySize = sliceCount;
    return u;
}
// The whole depth: FirstWSlice 0, WSize -1 (the cloud volume; Gpu::CreateTextureUav says the
// same for its 3D case).
inline D3D12_UNORDERED_ACCESS_VIEW_DESC UavDesc3D(DXGI_FORMAT fmt, uint32_t mip) {
    D3D12_UNORDERED_ACCESS_VIEW_DESC u{};
    u.Format = fmt;
    u.ViewDimension = D3D12_UAV_DIMENSION_TEXTURE3D;
    u.Texture3D.MipSlice = mip;
    u.Texture3D.FirstWSlice = 0;
    u.Texture3D.WSize = UINT(-1);
    return u;
}
inline D3D12_DEPTH_STENCIL_VIEW_DESC DsvDesc2D(DXGI_FORMAT fmt) {
    D3D12_DEPTH_STENCIL_VIEW_DESC d{};
    d.Format = fmt;
    d.ViewDimension = D3D12_DSV_DIMENSION_TEXTURE2D;
    return d;
}

// ---- the table ------------------------------------------------------------------------------
// `count` consecutive slots of the shader-visible heap, filled by index and bound by Base().
// Default-constructed it is empty; Alloc() gives it a base. A slot outside the table throws
// under the tag: the old sites wrote base+i unchecked.
class Table {
public:
    Table() = default;
    static Table Alloc(Gpu& gpu, uint32_t count, const char* tag) {
        Table t;
        t.m_gpu = &gpu;
        t.m_base = gpu.SrvHeap().Alloc(count);
        t.m_count = count;
        t.m_tag = tag;
        return t;
    }
    bool Valid() const { return m_gpu != nullptr; }
    uint32_t Base() const { return m_base; }
    uint32_t Count() const { return m_count; }
    uint32_t Slot(uint32_t i) const {
        if (i >= m_count) {
            throw std::runtime_error(std::string("descriptor table ") + m_tag + ": slot " +
                                     std::to_string(i) + " of " + std::to_string(m_count));
        }
        return m_base + i;
    }

    // SRVs: one mip from the top unless said otherwise. `res` may be null -- a NULL view reads
    // zeros, and a slot wired later starts with one (the churn kernel's t2..t4).
    void Srv2D(uint32_t i, ID3D12Resource* res, DXGI_FORMAT fmt, uint32_t mostDetailedMip = 0,
               uint32_t mipLevels = 1) {
        Srv(i, res, SrvDesc2D(fmt, mostDetailedMip, mipLevels));
    }
    void SrvArray(uint32_t i, ID3D12Resource* res, DXGI_FORMAT fmt, uint32_t firstSlice,
                  uint32_t sliceCount, uint32_t mipLevels = 1, uint32_t mostDetailedMip = 0) {
        Srv(i, res, SrvDescArray(fmt, firstSlice, sliceCount, mostDetailedMip, mipLevels));
    }
    void Srv3D(uint32_t i, ID3D12Resource* res, DXGI_FORMAT fmt, uint32_t mostDetailedMip = 0,
               uint32_t mipLevels = 1) {
        Srv(i, res, SrvDesc3D(fmt, mostDetailedMip, mipLevels));
    }
    // UAVs: one mip level, named.
    void Uav2D(uint32_t i, ID3D12Resource* res, DXGI_FORMAT fmt, uint32_t mip = 0) {
        Uav(i, res, UavDesc2D(fmt, mip));
    }
    void UavArray(uint32_t i, ID3D12Resource* res, DXGI_FORMAT fmt, uint32_t firstSlice,
                  uint32_t sliceCount, uint32_t mip = 0) {
        Uav(i, res, UavDescArray(fmt, firstSlice, sliceCount, mip));
    }
    void Uav3D(uint32_t i, ID3D12Resource* res, DXGI_FORMAT fmt, uint32_t mip = 0) {
        Uav(i, res, UavDesc3D(fmt, mip));
    }

private:
    void Srv(uint32_t i, ID3D12Resource* res, const D3D12_SHADER_RESOURCE_VIEW_DESC& d) {
        m_gpu->Device()->CreateShaderResourceView(res, &d, m_gpu->SrvHeap().Cpu(Slot(i)));
    }
    void Uav(uint32_t i, ID3D12Resource* res, const D3D12_UNORDERED_ACCESS_VIEW_DESC& d) {
        m_gpu->Device()->CreateUnorderedAccessView(res, nullptr, &d,
                                                   m_gpu->SrvHeap().Cpu(Slot(i)));
    }

    Gpu* m_gpu = nullptr;
    uint32_t m_base = UINT32_MAX, m_count = 0;
    const char* m_tag = "";
};

// ---- single views ---------------------------------------------------------------------------
// A view in a slot of its own: the same desc law, a table of one. (Gpu::CreateSrv and its
// siblings are the older single-view creators the layers call for their draw-side textures;
// no site outside src/hal/ needed these in step 3d -- every view a layer makes is a table
// entry. They exist so the single-view path and the table path cannot drift.)
inline uint32_t Srv2D(Gpu& gpu, ID3D12Resource* res, DXGI_FORMAT fmt,
                      uint32_t mostDetailedMip = 0, uint32_t mipLevels = 1) {
    Table t = Table::Alloc(gpu, 1, "srv");
    t.Srv2D(0, res, fmt, mostDetailedMip, mipLevels);
    return t.Base();
}
inline uint32_t Uav2D(Gpu& gpu, ID3D12Resource* res, DXGI_FORMAT fmt, uint32_t mip = 0) {
    Table t = Table::Alloc(gpu, 1, "uav");
    t.Uav2D(0, res, fmt, mip);
    return t.Base();
}

// ---- the renderer's targets -----------------------------------------------------------------
inline uint32_t Rtv(Gpu& gpu, ID3D12Resource* res, uint32_t slot = UINT32_MAX) {
    if (slot == UINT32_MAX) slot = gpu.RtvHeap().Alloc();
    gpu.Device()->CreateRenderTargetView(res, nullptr, gpu.RtvHeap().Cpu(slot));
    return slot;
}
inline uint32_t Dsv(Gpu& gpu, ID3D12Resource* res, DXGI_FORMAT fmt, uint32_t slot = UINT32_MAX) {
    if (slot == UINT32_MAX) slot = gpu.DsvHeap().Alloc();
    const D3D12_DEPTH_STENCIL_VIEW_DESC d = DsvDesc2D(fmt);
    gpu.Device()->CreateDepthStencilView(res, &d, gpu.DsvHeap().Cpu(slot));
    return slot;
}


}  // namespace ga::hal
