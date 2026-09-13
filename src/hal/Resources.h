// ================================================================================================
//  Resources.h - M12 step 3d: THE COMMITTED TEXTURE, named and justified.
//
//  The atlas pages almost everything (TileAtlas.h): a bank is reserved, its tiles mapped on
//  demand, and a dense committed texture is the exception -- the FFT's working set (a 256^2
//  ping-pong that is never partly resident), the gulf's two derived fields, the clouds' source
//  volume. Three sites spelled the exception out by hand: a D3D12_RESOURCE_DESC, DEFAULT heap
//  properties, CreateCommittedResource, SetName. Committed() is that block with the reason
//  attached: `why` is logged once at creation under the resource's name, so the log says why
//  this texture is not a bank. Two shapes, because two exist: Committed() for a Texture2D
//  (mips optional), Committed3D() for the volume. No clear value, no custom heap, no buffer --
//  no site had one; Gpu::CreateTexture2D keeps the render targets' clear values and the
//  buffers stay Gpu's.
//
//  THE DESC IS THE OLD SITE'S: DEFAULT heap with UNKNOWN page property and pool preference
//  (Gpu.cpp's HeapProps says the same), HEAP_FLAG_NONE, layout UNKNOWN, one sample, the flags
//  and the initial state the site passes -- gated bitwise against the three hand-written
//  blocks in step 3d's probe before they were deleted.
//
//  GetCopyableFootprints stays raw (GlobeLayer.cpp, the cloud source's upload): a footprint is
//  a copy-layout query, not a creation, and one site is not a pattern.
// ================================================================================================
#pragma once

#include "hal/Gpu.h"

#include <cstdint>
#include <stdexcept>
#include <string>

namespace ga::hal {

inline D3D12_HEAP_PROPERTIES DefaultHeap() {
    D3D12_HEAP_PROPERTIES p{};
    p.Type = D3D12_HEAP_TYPE_DEFAULT;
    p.CPUPageProperty = D3D12_CPU_PAGE_PROPERTY_UNKNOWN;
    p.MemoryPoolPreference = D3D12_MEMORY_POOL_UNKNOWN;
    return p;
}
inline D3D12_RESOURCE_DESC TextureDesc2D(uint32_t w, uint32_t h, DXGI_FORMAT fmt,
                                         D3D12_RESOURCE_FLAGS flags, uint16_t mips) {
    D3D12_RESOURCE_DESC d{};
    d.Dimension = D3D12_RESOURCE_DIMENSION_TEXTURE2D;
    d.Width = w;
    d.Height = h;
    d.DepthOrArraySize = 1;
    d.MipLevels = mips;
    d.Format = fmt;
    d.SampleDesc.Count = 1;
    d.Flags = flags;
    return d;
}
inline D3D12_RESOURCE_DESC TextureDesc3D(uint32_t w, uint32_t h, uint32_t depth, DXGI_FORMAT fmt,
                                         D3D12_RESOURCE_FLAGS flags) {
    D3D12_RESOURCE_DESC d{};
    d.Dimension = D3D12_RESOURCE_DIMENSION_TEXTURE3D;
    d.Width = w;
    d.Height = h;
    d.DepthOrArraySize = static_cast<UINT16>(depth);
    d.MipLevels = 1;
    d.Format = fmt;
    d.SampleDesc.Count = 1;
    d.Flags = flags;
    return d;
}

// The creation: DEFAULT heap, no clear value, named, and the reason in the log. Throws on
// failure as every site's GA_CHECK did.
inline Com<ID3D12Resource> CommittedRaw(Gpu& gpu, const wchar_t* name,
                                        const D3D12_RESOURCE_DESC& d,
                                        D3D12_RESOURCE_STATES initialState, const char* why) {
    const D3D12_HEAP_PROPERTIES hp = DefaultHeap();
    Com<ID3D12Resource> res;
    GA_CHECK(gpu.Device()->CreateCommittedResource(&hp, D3D12_HEAP_FLAG_NONE, &d, initialState,
                                                  nullptr, IID_PPV_ARGS(&res)));
    res->SetName(name);
    Log("[committed] %ls: %s", name, why);
    return res;
}
inline Com<ID3D12Resource> Committed(Gpu& gpu, const wchar_t* name, uint32_t w, uint32_t h,
                                     DXGI_FORMAT fmt, D3D12_RESOURCE_FLAGS flags,
                                     D3D12_RESOURCE_STATES initialState, const char* why,
                                     uint16_t mips = 1) {
    return CommittedRaw(gpu, name, TextureDesc2D(w, h, fmt, flags, mips), initialState, why);
}
inline Com<ID3D12Resource> Committed3D(Gpu& gpu, const wchar_t* name, uint32_t w, uint32_t h,
                                       uint32_t depth, DXGI_FORMAT fmt, D3D12_RESOURCE_FLAGS flags,
                                       D3D12_RESOURCE_STATES initialState, const char* why) {
    return CommittedRaw(gpu, name, TextureDesc3D(w, h, depth, fmt, flags), initialState, why);
}


}  // namespace ga::hal
