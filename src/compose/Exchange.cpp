#include "compose/Exchange.h"

#include <cstring>

namespace ga {

int Exchange::Register(const std::string& name, const GaBufferLayout& layout,
                       const std::string& producer) {
    Channel ch;
    ch.name = name;
    ch.producer = producer;
    ch.layout = layout;
    m_channels.push_back(std::move(ch));
    return static_cast<int>(m_channels.size()) - 1;
}

void Exchange::Publish(Gpu& gpu, int channel, const void* data, size_t bytes) {
    Channel& ch = m_channels[channel];
    if (ch.version > 0) gpu.WaitIdle();   // rare republish: drain readers before overwrite
    if (!ch.buf.res || ch.buf.size < bytes) {
        ch.buf = gpu.CreateUploadBuffer((std::max)(bytes, static_cast<size_t>(256)),
                                        L"exchange channel");
    }
    memcpy(ch.buf.cpu, data, bytes);
    ch.bytes = bytes;
    ++ch.version;
}

Exchange::View Exchange::Query(const std::string& name) const {
    for (const auto& ch : m_channels) {
        if (ch.name != name) continue;
        View v;
        v.va = ch.buf.res ? ch.buf.res->GetGPUVirtualAddress() : 0;
        v.elements = ch.layout.stride ? static_cast<uint32_t>(ch.bytes / ch.layout.stride) : 0;
        v.version = ch.version;
        v.layout = ch.layout;
        v.valid = ch.version > 0;
        return v;
    }
    return {};
}

void Exchange::LogRegistry() const {
    for (const auto& ch : m_channels) {
        Log("[exchange] %-22s <- %-16s %4u B/elem  grades %02x  %6zu elems v%u  \"%s\"",
            ch.name.c_str(), ch.producer.c_str(), ch.layout.stride, ch.layout.gradeSig,
            ch.layout.stride ? ch.bytes / ch.layout.stride : 0, ch.version,
            ch.layout.semantic.c_str());
    }
}

}  // namespace ga
