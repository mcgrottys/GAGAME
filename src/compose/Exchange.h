// ================================================================================================
//  Exchange - M6j: the plugin bus. Named, versioned GPU buffer channels with DECLARED
//  multivector layouts -- the buffer-side sibling of the compositor's texture channels.
//
//  The contract (what a plugin sees):
//    * Register(name, layout, producer): declare a channel once. The layout says what an
//      element IS -- stride in bytes, a grade-signature byte (the same Cl bits the atlas
//      speaks: bit0 scalar, bit1 vector, bit2 bivector, bit3 trivector, bit4 quadvector...),
//      and a human-readable semantic ("pga-motor + scale + rgb", "lonlat-degrees line-list").
//    * Publish(gpu, id, data, bytes): upload a new version. The consumer sees a GPU VA +
//      element count + version; a version bump means re-read.
//    * Query(name): consumers (layers, exporters, physics) resolve by NAME -- they know the
//      channel, never the producer. Same doctrine as textures: earth color is earth color.
//
//  This is where "CPU does GA for organization, GPU does GA for rendering" gets its socket:
//  a plugin computes motors / multivector products on the CPU (Pga.h -- conventions pinned by
//  selftest), publishes them as a GA product buffer, and a shader applies the sandwich to
//  geometry (GA.hlsli MotorPoint, the SAME formulas). A mesh/vertex buffer is just a channel
//  whose layout says so -- and it can sit inside a larger GA product buffer, which is exactly
//  the user's plugin plan for physics-driven geometry.
//
//  Concurrency: Publish waits for the GPU to drain before overwriting an existing buffer
//  (channels update rarely -- registration-time, forecast-cycle, or user action; per-frame
//  streams keep using the frame-indexed arenas). Growth reallocates.
// ================================================================================================
#pragma once

#include "core/Gpu.h"

#include <string>
#include <vector>

namespace ga {

struct GaBufferLayout {
    uint32_t stride = 0;      // bytes per element
    uint8_t gradeSig = 0;     // Cl grade-signature bits of the element's multivector content
    std::string semantic;     // human contract, shown in the registry log
};

class Exchange {
public:
    struct View {
        D3D12_GPU_VIRTUAL_ADDRESS va = 0;
        uint32_t elements = 0;
        uint32_t version = 0;
        GaBufferLayout layout;
        bool valid = false;
    };

    int Register(const std::string& name, const GaBufferLayout& layout,
                 const std::string& producer);
    void Publish(Gpu& gpu, int channel, const void* data, size_t bytes);
    View Query(const std::string& name) const;
    void LogRegistry() const;   // the buffer table: name, producer, semantic, grades, count

private:
    struct Channel {
        std::string name, producer;
        GaBufferLayout layout;
        GpuBuffer buf;
        size_t bytes = 0;
        uint32_t version = 0;
    };
    std::vector<Channel> m_channels;
};

}  // namespace ga
