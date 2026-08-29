#include "scene/FieldSet.h"

#include "core/Image.h"

namespace ga {

void FieldSet::Init(Gpu& gpu, const std::wstring& dir, float patchWidthM, float patchHeightM) {
    m_gpu = &gpu;
    m_dir = dir;
    if (!m_dir.empty() && m_dir.back() != L'/' && m_dir.back() != L'\\') m_dir += L'/';
    m_patchW = patchWidthM;
    m_patchH = patchHeightM;
}

uint32_t FieldSet::Add(const std::wstring& fileName, FieldLayout layout, const float scale[4],
                       const float bias[4]) {
    if (auto it = m_byName.find(fileName); it != m_byName.end()) return it->second;

    const std::wstring path = m_dir + fileName;
    ImageData img = LoadPng(path);
    if (!img.Valid()) {
        Log("[fields] MISSING %S", path.c_str());
        return UINT32_MAX;
    }

    GpuTexture tex = m_gpu->CreateTexture2D(img.width, img.height, img.format,
                                            D3D12_RESOURCE_FLAG_NONE,
                                            D3D12_RESOURCE_STATE_COPY_DEST, fileName.c_str());
    m_gpu->UploadTexture(tex, img.pixels.data(), img.rowPitch);
    tex.srv = m_gpu->CreateSrv(tex.res.Get(), img.format);

    FieldDesc d{};
    // World metres -> [0,1]. The patch spans [0, patchW] east and [0, patchH] north; row 0 of a
    // source raster is NORTH, so v is flipped.
    d.worldToUv[0] = 1.0f / m_patchW;
    d.worldToUv[1] = -1.0f / m_patchH;
    d.worldToUv[2] = 0.0f;
    d.worldToUv[3] = 1.0f;
    for (int i = 0; i < 4; ++i) {
        d.valueScale[i] = scale ? scale[i] : 1.0f;
        d.valueBias[i] = bias ? bias[i] : 0.0f;
    }
    d.srvIndex = tex.srv;
    d.layout = static_cast<uint32_t>(layout);

    const uint32_t index = static_cast<uint32_t>(m_descs.size());
    m_descs.push_back(d);
    m_textures.push_back(std::move(tex));
    m_byName.emplace(fileName, index);
    Log("[fields] %-34S %ux%u  srv=%u  idx=%u", fileName.c_str(), img.width, img.height,
        m_descs.back().srvIndex, index);
    return index;
}

void FieldSet::Finalize() {
    if (m_descs.empty()) {
        // See the header: root parameter 2 must be bound to SOMETHING, and vqview learned that
        // the hard way (device removal). One zeroed row keeps the address valid.
        m_descs.push_back(FieldDesc{});
        Log("[fields] no fields registered; binding a dummy row so root param 2 stays legal");
    }
    m_table = m_gpu->CreateDefaultBuffer(m_descs.data(), m_descs.size() * sizeof(FieldDesc),
                                         L"field table");
    m_tableSrv = m_gpu->CreateStructuredBufferSrv(m_table.res.Get(),
                                                  static_cast<uint32_t>(m_descs.size()),
                                                  sizeof(FieldDesc));
    Log("[fields] %zu fields, table srv=%u", m_descs.size(), m_tableSrv);
}

uint32_t FieldSet::Find(const std::wstring& fileName) const {
    auto it = m_byName.find(fileName);
    return it == m_byName.end() ? UINT32_MAX : it->second;
}

}  // namespace ga
