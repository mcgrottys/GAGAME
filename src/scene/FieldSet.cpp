#include "scene/FieldSet.h"

namespace ga {

void FieldSet::Init(Gpu& gpu, const std::wstring& dir, float patchWidthM, float patchHeightM) {
    m_gpu = &gpu;
    m_dir = dir;
    if (!m_dir.empty() && m_dir.back() != L'/' && m_dir.back() != L'\\') m_dir += L'/';
    m_patchW = patchWidthM;
    m_patchH = patchHeightM;
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
