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
// STAGE 2b (2026-10-27): THE TAIL to the readers of the far field (shaders/TailRead.hlsli; SweSolver::TailGpuDesc):
// its h and bed textures on the heap, its cells a side, its chart (the anchor's up, east and north in the planet's
// frame, R, the cell's angle east and north over R), and the display exaggeration on the drawn departure.
struct TailGpu {
    uint32_t hSrv = 0xFFFFFFFFu, bedSrv = 0xFFFFFFFFu;
    uint32_t cells = 0;   // 0 = no tail
    float up[3] = {0.0f, 1.0f, 0.0f}, east[3] = {1.0f, 0.0f, 0.0f}, north[3] = {0.0f, 0.0f, 1.0f};
    float R = 6371000.0f, ds = 0.0f, dt = 0.0f;
    float exag = 1.0f;
};
}  // namespace ga
