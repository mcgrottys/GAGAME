// What the GPU readers of the impacts are handed each frame (sim/ImpactField.h Pack): the events'
// buffer (a bindless uint buffer: 8 words a header, then each event's terms), how many events, the
// instant on the field's base, the planet's radius and gravity. The two constant buffers carry it
// (gImpA / gImpU in Globe.hlsl and WaterBank.hlsl; shaders/ImpactRead.hlsli reads it).
#pragma once
#include <cstdint>

namespace ga {
struct ImpactGpu {
    uint32_t slot = 0xFFFFFFFFu;   // the events' buffer SRV (gBufU), none = 0xFFFFFFFF
    uint32_t events = 0;           // events in the buffer (0 = nothing to sum)
    float tRel = 0.0f;             // the water clock's instant less the field's base (s)
    float planetR = 6371000.0f;
    float g = 9.81f;
};
}  // namespace ga
