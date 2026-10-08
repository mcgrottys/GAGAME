#include "hal/Readback.h"

#include <cstring>

namespace ga::hal {

namespace {

uint32_t TexelBytes(DXGI_FORMAT fmt) {
    switch (fmt) {
        case DXGI_FORMAT_R32_FLOAT: return 4;
        case DXGI_FORMAT_R16_FLOAT: return 2;
        case DXGI_FORMAT_R32G32_FLOAT: return 8;
        case DXGI_FORMAT_R16G16B16A16_FLOAT: return 8;
        case DXGI_FORMAT_R32G32B32A32_FLOAT: return 16;
        default: return 0;
    }
}

}  // namespace

void RegionReadback::Init(Gpu& gpu, uint64_t bytesPerSlot, const wchar_t* name) {
    m_gpu = &gpu;
    // Every slot starts on the placement alignment (CopyRegion aligns within the slot).
    m_bytesPerSlot = (bytesPerSlot + D3D12_TEXTURE_DATA_PLACEMENT_ALIGNMENT - 1u) &
                     ~uint64_t(D3D12_TEXTURE_DATA_PLACEMENT_ALIGNMENT - 1u);
    D3D12_HEAP_PROPERTIES hp{};
    hp.Type = D3D12_HEAP_TYPE_READBACK;
    D3D12_RESOURCE_DESC rd{};
    rd.Dimension = D3D12_RESOURCE_DIMENSION_BUFFER;
    rd.Width = m_bytesPerSlot * Gpu::kFrameCount;
    rd.Height = 1;
    rd.DepthOrArraySize = 1;
    rd.MipLevels = 1;
    rd.SampleDesc.Count = 1;
    rd.Layout = D3D12_TEXTURE_LAYOUT_ROW_MAJOR;
    GA_CHECK(gpu.Device()->CreateCommittedResource(&hp, D3D12_HEAP_FLAG_NONE, &rd,
                                                   D3D12_RESOURCE_STATE_COPY_DEST, nullptr,
                                                   IID_PPV_ARGS(&m_buffer)));
    m_buffer->SetName(name);
}

size_t RegionReadback::BeginSlot(uint32_t frameIndex) {
    m_delivered.clear();
    if (!m_buffer || frameIndex >= Gpu::kFrameCount) {
        m_cur = UINT32_MAX;
        return 0;
    }
    m_cur = frameIndex;
    Slot& s = m_slots[m_cur];
    // Gpu::BeginFrame waited on this slot's previous fence before this frame began recording, so
    // its copies have executed: read them, then reuse the slot.
    if (s.pending && !s.copies.empty()) {
        const uint64_t base = uint64_t(m_cur) * m_bytesPerSlot;
        D3D12_RANGE range{SIZE_T(base), SIZE_T(base + s.used)};
        void* p = nullptr;
        if (SUCCEEDED(m_buffer->Map(0, &range, &p)) && p) {
            const uint8_t* bytes = static_cast<const uint8_t*>(p);
            for (Copy& c : s.copies) {
                Region r = c.region;
                const size_t rowBytes = size_t(r.w) * r.texelBytes;
                r.bytes.resize(rowBytes * r.h);
                for (uint32_t y = 0; y < r.h; ++y) {
                    memcpy(r.bytes.data() + size_t(y) * rowBytes,
                           bytes + c.offset + uint64_t(y) * c.rowPitch, rowBytes);
                }
                m_delivered.push_back(std::move(r));
            }
            D3D12_RANGE none{0, 0};
            m_buffer->Unmap(0, &none);
        }
    }
    s.pending = false;
    s.used = 0;
    s.copies.clear();
    return m_delivered.size();
}

bool RegionReadback::CopyRegion(CommandContext& cmd, Resource src, uint32_t sub, DXGI_FORMAT fmt,
                                uint32_t x0, uint32_t y0, uint32_t w, uint32_t h, uint32_t tag) {
    if (!m_buffer || m_cur == UINT32_MAX || !src || w == 0 || h == 0) return false;
    const uint32_t tb = TexelBytes(fmt);
    if (tb == 0) return false;
    Slot& s = m_slots[m_cur];

    D3D12_RESOURCE_DESC td{};
    td.Dimension = D3D12_RESOURCE_DIMENSION_TEXTURE2D;
    td.Width = w;
    td.Height = h;
    td.DepthOrArraySize = 1;
    td.MipLevels = 1;
    td.Format = fmt;
    td.SampleDesc.Count = 1;
    D3D12_PLACED_SUBRESOURCE_FOOTPRINT fp{};
    UINT64 total = 0;
    m_gpu->Device()->GetCopyableFootprints(&td, 0, 1, 0, &fp, nullptr, nullptr, &total);
    // A placed footprint's offset is a multiple of D3D12_TEXTURE_DATA_PLACEMENT_ALIGNMENT (512) --
    // D3D12's baseline rule; only an adapter reporting the unrestricted-pitch option relaxes it, and
    // nothing asks. 8 was invalid on the rest (the solver queues its eta and uv copies back to back).
    // The slots' stride is a multiple of 512 too (Init), so the absolute offset is.
    constexpr uint64_t kPlace = D3D12_TEXTURE_DATA_PLACEMENT_ALIGNMENT;
    const uint64_t at = (s.used + kPlace - 1u) & ~(kPlace - 1u);
    if (at + total > m_bytesPerSlot) return false;

    D3D12_TEXTURE_COPY_LOCATION dst{}, from{};
    dst.pResource = m_buffer.Get();
    dst.Type = D3D12_TEXTURE_COPY_TYPE_PLACED_FOOTPRINT;
    dst.PlacedFootprint = fp;
    dst.PlacedFootprint.Offset = uint64_t(m_cur) * m_bytesPerSlot + at;
    from.pResource = src;
    from.Type = D3D12_TEXTURE_COPY_TYPE_SUBRESOURCE_INDEX;
    from.SubresourceIndex = sub;
    const D3D12_BOX box{x0, y0, 0, x0 + w, y0 + h, 1};
    cmd.Native()->CopyTextureRegion(&dst, 0, 0, 0, &from, &box);

    Copy c;
    c.offset = uint64_t(m_cur) * m_bytesPerSlot + at;
    c.rowPitch = fp.Footprint.RowPitch;
    c.region.tag = tag;
    c.region.x0 = x0;
    c.region.y0 = y0;
    c.region.w = w;
    c.region.h = h;
    c.region.texelBytes = tb;
    s.copies.push_back(std::move(c));
    s.used = at + total;
    s.pending = true;
    return true;
}

}  // namespace ga::hal
